/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_unmap.c -- native UNMAP fan-out.
 *
 * Supported caller chain (Longhorn V2 EC sharded volume):
 *   workload fstrim
 *     -> ext4/xfs discard
 *     -> NVMe-oF initiator on engine node
 *     -> bdev_raid1 (one base bdev for EC)
 *     -> bdev_nvme -> NVMe-oF target on ShardGroup node
 *     -> head lvol -> lvol store (4 MiB cluster)
 *     -> bdev_ec  <-- arrives here, stripe-aligned
 *     -> bdev_nvme x (k+m) -> NVMe-oF targets on shard nodes
 *     -> per-shard lvol -> bdev_aio -> ext4 PUNCH_HOLE / NVMe DEALLOCATE
 *
 * UNMAP correctness lives in bdev_ec, not in the base bdevs. The EC
 * layer owns a persistent per-stripe unmapped bitmap (raw-replicated
 * across every disk, double-buffered; see bdev_ec_bitmap.c); every
 * read consults it before issuing base I/O, so a stripe whose bit is
 * set returns synthesized zeros regardless of whether any base bdev
 * deallocates-to-zero. ec_io_type_supported(UNMAP) is therefore
 * unconditionally true.
 *
 * The physical spdk_bdev_unmap_blocks fan-out below is best-effort
 * space reclamation. A slot that fails to reclaim wastes space on
 * that disk but cannot corrupt the array, because no reader trusts a
 * discarded range.
 *
 * Visibility / durability contract (pessimistic): the UNMAP'd bit
 * becomes visible to readers (and to rebuild / RMW / scrub) only
 * after the bitmap persist has acked at the m+1 durability threshold.
 * This file's submit path stages the new bits into a per-UNMAP shadow
 * (uctx->staged_map), persists that shadow, and atomically copies it
 * into the live ec->stripe_unmapped_map only after persist ack -- so
 * a persist failure is observable as "the UNMAP just failed" with no
 * visible side effect, never as "reads briefly returned zero and now
 * return real data again."
 *
 * Stripe alignment is the load-bearing precondition for issuing a
 * real spdk_bdev_unmap_blocks to the bases. Realistic callers
 * (blobstore -> NVMe-oF target -> bdev_ec) issue cluster-aligned
 * UNMAPs, and lvol clusters are integer multiples of EC stripes in
 * every supported layout. Sub-stripe UNMAPs route to WRITE_ZEROES;
 * mixed-alignment multi-stripe UNMAPs are rejected.
 */

#include "bdev_ec_internal.h"

#include "spdk/bdev_module.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/log.h"

/*
 * Inner-fanout completion callback. Invoked exactly once per
 * ec_unmap_inner_fanout invocation -- either from the bitmap-persist
 * failure path, the fan-out submit-failure path, or after every base
 * UNMAP completion has acked. The default callback (used by
 * ec_submit_unmap on the aligned-multi-stripe path) completes the
 * parent bdev_io directly. The segment-dispatcher uses a coordinator
 * callback that decrements a segment counter and only completes the
 * parent once every segment has reported back.
 */
typedef void (*ec_unmap_inner_complete_cb)(void *cb_arg,
					   enum spdk_bdev_io_status status);

/*
 * Per-UNMAP context. Tracks the claimed stripe range so completion
 * can release every stripe-busy bit, plus the staged shadow of the
 * unmapped bitmap with the new bits set. The shadow lives only for
 * the duration of one UNMAP; ec->stripe_unmapped_map is updated from
 * it atomically once the bitmap persist acks.
 */
struct ec_unmap_ctx {
	struct ec_bdev_io           *ec_io;
	uint64_t                     start_stripe;
	uint64_t                     end_stripe;      /* exclusive */
	uint64_t                     disk_lba;        /* per-base offset */
	uint64_t                     disk_num_blocks; /* per-base length */
	uint64_t                    *staged_map;      /* shadow: live | new bits */
	uint32_t                     writes_remaining;
	enum spdk_bdev_io_status     status;
	ec_unmap_inner_complete_cb   cb_fn;
	void                        *cb_arg;
};

static int  ec_unmap_fanout(struct ec_unmap_ctx *uctx);
static int  ec_unmap_inner_fanout(struct ec_bdev_io *ec_io,
				  uint64_t start_stripe, uint64_t end_stripe,
				  ec_unmap_inner_complete_cb cb_fn,
				  void *cb_arg);
static void ec_unmap_bitmap_persist_done(void *cb_arg, int persist_rc);
static void ec_unmap_child_complete(struct spdk_bdev_io *child,
				    bool success, void *cb_arg);
static void ec_unmap_dispatch_fanout_on_submitter(void *ctx);
static void ec_unmap_fire_cb_on_submitter(void *ctx);

/*
 * Multi-segment UNMAP dispatcher state. An unaligned multi-stripe UNMAP
 * gets split into up to three segments: a partial-stripe head fragment
 * (RMW zero-fill), the inner stripe-aligned range (native UNMAP
 * fan-out via the bitmap), and a partial-stripe tail fragment (RMW
 * zero-fill). Segments are dispatched serially in the order inner ->
 * head -> tail; the dispatcher waits for each segment's completion
 * before submitting the next.
 *
 * Why inner first: it is the segment most likely to encounter -EAGAIN at
 * submit time (bitmap-persist contention with concurrent UNMAPs is a
 * real failure mode under fstrim). If inner fails synchronously, no
 * other segments have been submitted and the dispatcher can cleanly
 * return -EAGAIN to the bdev layer for a normal NOMEM-queue requeue.
 * Head/tail RMW only encounter -EAGAIN on the much rarer scrub-defer
 * or stripe-busy paths (WIB persist contention is handled internally
 * by wib_deferred_writes), so post-inner -EAGAIN is rare and treated
 * as parent-FAILED with kernel-driven retry.
 *
 * Why serial rather than parallel: parallel dispatch would have to
 * either (a) pre-acquire every resource atomically before submitting
 * any segment, or (b) handle "in-flight earlier segment + sibling segment
 * submit failure" cleanup. Both are more complex than serial
 * dispatch and gain at most a few microseconds of overlap on segments
 * that are bounded by base-bdev I/O latency anyway.
 *
 * Bitmap is set only for stripes in [inner_start_stripe,
 * inner_end_stripe); the head/tail stripes are partially live and
 * cannot be marked fully unmapped. Their RMW physically zeros only
 * the requested sub-range and recomputes parity, leaving the rest
 * of those stripes intact.
 */
struct ec_unmap_split_ctx {
	struct ec_bdev_io       *ec_io;

	/* Head segment: zero-fill the partial stripe at the front of the request.
	 * head_num_blocks == 0 means the request was already head-aligned. */
	uint64_t                 head_offset_blocks;
	uint64_t                 head_num_blocks;

	/* Tail segment: zero-fill the partial stripe at the back of the request.
	 * tail_num_blocks == 0 means the request was already tail-aligned. */
	uint64_t                 tail_offset_blocks;
	uint64_t                 tail_num_blocks;

	/* Which segment to submit next. Set by the entry point after the inner
	 * segment is submitted; advanced by ec_unmap_split_continue after each
	 * segment's completion callback fires. */
	enum {
		EC_UNMAP_SPLIT_STAGE_HEAD,
		EC_UNMAP_SPLIT_STAGE_TAIL,
		EC_UNMAP_SPLIT_STAGE_DONE,
	} next_stage;

	enum spdk_bdev_io_status status;
};

static int  ec_submit_unmap_split(struct ec_bdev_io *ec_io,
				  uint64_t head_off, uint64_t head_len,
				  uint64_t inner_start_stripe,
				  uint64_t inner_end_stripe,
				  uint64_t tail_off, uint64_t tail_len);
static void ec_unmap_split_continue(struct ec_unmap_split_ctx *sctx);
static void ec_unmap_split_segment_done(void *cb_arg,
				    enum spdk_bdev_io_status status);
static void ec_unmap_split_complete(struct ec_unmap_split_ctx *sctx);

/*
 * Default inner-fanout completion: cb_arg is the parent ec_bdev_io.
 * Used by the aligned-multi-stripe path in ec_submit_unmap; it completes
 * the parent bdev_io directly, with no segment coordination.
 *
 * Bumps unmaps_completed on SUCCESS here -- at the parent-request
 * completion boundary -- rather than inside ec_unmap_child_complete.
 * The multi-segment dispatcher (ec_unmap_split_complete) completes the
 * parent through a different terminal function; bumping here keeps the
 * counter aligned with "one parent UNMAP, one bump" regardless of which
 * internal path was taken.
 */
static void
ec_unmap_inner_complete_default(void *cb_arg,
				enum spdk_bdev_io_status status)
{
	struct ec_bdev_io *ec_io = cb_arg;
	struct ec_bdev    *ec    = ec_from_bdev_io(ec_io->bdev_io);

	if (status == SPDK_BDEV_IO_STATUS_SUCCESS) {
		__atomic_fetch_add(&ec->unmaps_completed, 1, __ATOMIC_RELAXED);
	} else {
		__atomic_fetch_add(&ec->unmaps_failed,    1, __ATOMIC_RELAXED);
	}

	/*
	 * spdk_bdev_io_complete asserts the owner (submitter) thread. The fan-out
	 * success path lands on submitter (routed there in bitmap_persist_done);
	 * the persist-failure path runs on home and must hop. Stash status in
	 * ec_io->status so the helper can read it on the submitter side.
	 */
	if (spdk_likely(spdk_get_thread() == ec_io->submitter_thread)) {
		spdk_bdev_io_complete(ec_io->bdev_io, status);
		return;
	}

	ec_io->status = status;
	ec_io_route_complete_to_submitter(ec_io, "UNMAP completion");
}

static void
ec_unmap_release_claims(struct ec_bdev *ec,
			uint64_t start_stripe, uint64_t end_stripe)
{
	uint64_t stripe;

	for (stripe = start_stripe; stripe < end_stripe; stripe++) {
		ec_stripe_clear_dirty(ec, stripe);
	}
}

/*
 * Route an UNMAP that cannot use the real fan-out (sub-stripe range)
 * through the existing WRITE_ZEROES path. ec_bdev_io_init() set
 * is_zero_fill = false for UNMAP; flip it so the RMW modify step
 * memsets zero instead of copying from a NULL iov.
 */
static int
ec_unmap_route_to_zeros(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);

	ec_io->is_zero_fill = true;
	ec->unmaps_via_write_zeros++;
	return ec_submit_write(ec_io);
}

/*
 * Active-scrub guard. Any stripe in the UNMAP range that the
 * scrubber has not yet passed defers the entire UNMAP. The scrubber
 * reads old data and re-encodes parity from it; if we punched holes
 * on the data disks before the scrubber's parity write landed,
 * parity would diverge from the post-UNMAP zero state. Generalised
 * across the WIB regions an UNMAP can span (the bitmap persist is
 * whole-blob and has no region concept, but the scrubber still works
 * region-by-region).
 *
 * Returns true if the UNMAP should be deferred.
 */
static bool
ec_unmap_must_defer_for_scrub(struct ec_bdev *ec,
			      uint64_t start_stripe, uint64_t end_stripe)
{
	struct ec_scrub_ctx *scrub_ctx = ec->scrub_ctx;
	uint32_t region, end_region;

	if (scrub_ctx == NULL) {
		return false;
	}

	region     = ec_wib_stripe_to_region(start_stripe);
	end_region = ec_wib_stripe_to_region(end_stripe - 1);

	for (; region <= end_region; region++) {
		if (region == scrub_ctx->current_region) {
			uint64_t lo = spdk_max(start_stripe,
				(uint64_t)region * EC_WIB_REGION_STRIPES);
			uint64_t hi = spdk_min(end_stripe,
				(uint64_t)(region + 1) * EC_WIB_REGION_STRIPES);

			/*
			 * Defer if any stripe in this region's overlap is at or
			 * ahead of the scrub cursor. Instead of scanning every
			 * stripe, one comparison is enough: such a stripe exists in
			 * [lo, hi) when current_stripe < hi (lo < hi holds here).
			 */
			if (hi > lo && scrub_ctx->current_stripe < hi) {
				return true;
			}
		} else if (region > scrub_ctx->current_region &&
			   ec_wib_crash_is_dirty(ec, region)) {
			return true;
		}
	}
	return false;
}

static void ec_submit_unmap_on_home(void *ctx);

int
ec_submit_unmap(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec  = ec_from_bdev_io(ec_io->bdev_io);
	int             rc;
	uint64_t        off = ec_io->offset_blocks;
	uint64_t        len = ec_io->num_blocks;
	uint64_t        end = off + len;
	uint64_t        start_stripe, end_stripe;

	if (len == 0) {
		/* No home-thread state touched; complete inline on the
		 * submitter (= owner) thread. Count here so the no-op path
		 * still shows up in the accounting; see the field-cluster
		 * comment in bdev_ec_internal.h for the closed identity. */
		__atomic_fetch_add(&ec->unmaps_submitted, 1, __ATOMIC_RELAXED);
		__atomic_fetch_add(&ec->unmaps_completed, 1, __ATOMIC_RELAXED);
		spdk_bdev_io_complete(ec_io->bdev_io,
				      SPDK_BDEV_IO_STATUS_SUCCESS);
		return 0;
	}

	/*
	 * Entry routing: the unmap path mutates home-only state
	 * (stripe-busy claim range, bitmap shadow / persist orchestration)
	 * below. Route to home if we're not already there. When the caller
	 * is already on the home thread the inline path runs.
	 *
	 * unmaps_submitted is bumped AFTER this check so the cross-thread
	 * re-entry through ec_submit_unmap_on_home -> ec_submit_unmap
	 * counts the request exactly once (on home). Counting before the
	 * hop would double every cross-thread UNMAP and break the closed
	 * accounting identity in the field-cluster comment.
	 */
	if (spdk_unlikely(spdk_get_thread() != ec->home_thread)) {
		return spdk_thread_send_msg(ec->home_thread,
					    ec_submit_unmap_on_home, ec_io);
	}

	__atomic_fetch_add(&ec->unmaps_submitted, 1, __ATOMIC_RELAXED);

	/*
	 * Single-stripe shortcut: every block of the request lies within one
	 * stripe (first/last block share a stripe index). A request that straddles
	 * a boundary but contains no full inner stripe is NOT single-stripe and
	 * must fall through to the multi-stripe path below -- routing it to the
	 * RMW zero-fill helper would overflow that helper's one-stripe scratch.
	 */
	/*
	 * Dispatch by alignment shape. Every branch sets rc; non-zero,
	 * non-EAGAIN returns are sync-terminal failures that no async cb_fn
	 * will ever close out, so they bump unmaps_failed at the tail to
	 * keep the closed identity in bdev_ec_internal.h's field cluster.
	 * -EAGAIN is the deferred-busy bucket (deferred_busy was bumped
	 * inside the helper) and -EINPROGRESS-style returns of 0 mean the
	 * cb_fn will land later in inner_complete_default or
	 * ec_unmap_split_complete, which handle their own SUCCESS / FAILED
	 * accounting.
	 */
	if (off / ec->stripe_blocks == (end - 1) / ec->stripe_blocks) {
		/*
		 * Single-stripe UNMAP: route through the WRITE_ZEROES / RMW
		 * zero-fill path. This intentionally does NOT set the unmapped
		 * bit or issue a physical base-bdev UNMAP -- at single-stripe
		 * granularity a whole-blob bitmap persist is not worth it. The
		 * read-as-zero contract still holds (the stripe is zeroed), and
		 * unmaps_via_write_zeros counts these. Bitmap-backed reclaim is
		 * reserved for the stripe-aligned multi-stripe paths below.
		 */
		rc = ec_unmap_route_to_zeros(ec_io);
		goto out;
	}

	/*
	 * Multi-stripe request. Compute the inner stripe-aligned range
	 * [start_stripe, end_stripe). The inner range may be EMPTY
	 * (start_stripe == end_stripe) when the request straddles exactly
	 * one stripe boundary with no full inner stripe -- this is the
	 * boundary-straddler case that the split dispatcher handles as
	 * "head segment + tail segment, no inner segment."
	 */
	start_stripe = (off + ec->stripe_blocks - 1) / ec->stripe_blocks;
	end_stripe   = end / ec->stripe_blocks;

	/*
	 * Aligned multi-stripe (head_len == 0 && tail_len == 0): take
	 * the existing inner-only fast path with no split_ctx overhead.
	 * Guaranteed to have at least one inner stripe here (otherwise
	 * the single-stripe check above would have fired).
	 */
	if (off == start_stripe * ec->stripe_blocks &&
	    end == end_stripe   * ec->stripe_blocks) {
		rc = ec_unmap_inner_fanout(ec_io, start_stripe, end_stripe,
					   ec_unmap_inner_complete_default,
					   ec_io);
		goto out;
	}

	/*
	 * Unaligned multi-stripe UNMAP: at least one of head_len /
	 * tail_len > 0, possibly with an empty inner range. Split into
	 * up to three segments:
	 *   - head [off, start_stripe*stripe_blocks): partial-stripe RMW
	 *     zero-fill via ec_submit_rmw_zero_fill_range (skip if 0).
	 *   - inner [start_stripe, end_stripe): stripe-aligned native
	 *     UNMAP via ec_unmap_inner_fanout (skip if empty).
	 *   - tail [end_stripe*stripe_blocks, off+len): same shape as
	 *     head (skip if 0).
	 *
	 * Returning -EINVAL here (the historical behavior for unaligned
	 * multi-stripe) caused the Linux NVMe driver to disable discard
	 * for the namespace after enough invalid-field responses,
	 * silently killing every subsequent fstrim. Splitting fixes that
	 * AND fully reclaims the partial fragments.
	 */
	{
		uint64_t head_off = off;
		uint64_t head_len = (start_stripe * ec->stripe_blocks) - off;
		uint64_t tail_off = end_stripe * ec->stripe_blocks;
		uint64_t tail_len = end - tail_off;

		rc = ec_submit_unmap_split(ec_io,
					   head_off, head_len,
					   start_stripe, end_stripe,
					   tail_off, tail_len);
	}

out:
	if (rc != 0 && rc != -EAGAIN) {
		__atomic_fetch_add(&ec->unmaps_failed, 1, __ATOMIC_RELAXED);
	}
	return rc;
}

/*
 * Re-enters ec_submit_unmap on the home thread. On sync failure, route the
 * bdev_io completion back to the submitter with the appropriate status
 * (NOMEM for retryable, FAILED for hard). If the completion hand-off also
 * fails the bdev_io stays in flight rather than being completed on the
 * wrong thread.
 */
static void
ec_submit_unmap_on_home(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;
	int                rc;

	rc = ec_submit_unmap(ec_io);
	if (rc == 0) {
		return;
	}

	ec_io->status = (rc == -EAGAIN || rc == -ENOMEM)
			? SPDK_BDEV_IO_STATUS_NOMEM
			: SPDK_BDEV_IO_STATUS_FAILED;
	ec_io_route_complete_to_submitter(ec_io, "submit failure");
}

/*
 * Inner stripe-aligned native UNMAP fan-out. Claims every stripe in
 * [start_stripe, end_stripe), stages the new bitmap bits in a per-UNMAP
 * shadow, persists the shadow at m+1 durability, then fans out
 * spdk_bdev_unmap_blocks to every writable base bdev.
 *
 * The caller is responsible for verifying that the requested user range
 * is exactly [start_stripe, end_stripe) stripes -- this function makes
 * no assumption about the relationship between ec_io->offset_blocks /
 * num_blocks and the range it actually processes.
 *
 * cb_fn / cb_arg are invoked exactly once with the final aggregated
 * status when the fan-out completes (or earlier on the persist-failure
 * or fan-out-submit-failure paths). The default callback used by
 * ec_submit_unmap on the aligned-multi-stripe path completes the
 * parent bdev_io directly; the segment dispatcher (for unaligned
 * multi-stripe UNMAPs) passes a coordinator callback so the parent
 * bdev_io is only completed once every segment has reported back.
 *
 * Returns 0 if the persist was successfully submitted (cb_fn will
 * fire asynchronously). Returns -EAGAIN if a stripe is busy or the
 * persist is already in flight; -ENOMEM if context allocation fails.
 * On non-zero return, cb_fn is NOT invoked -- the caller owns
 * completion of the parent bdev_io.
 */
static int
ec_unmap_inner_fanout(struct ec_bdev_io *ec_io,
		      uint64_t start_stripe, uint64_t end_stripe,
		      ec_unmap_inner_complete_cb cb_fn, void *cb_arg)
{
	struct ec_bdev      *ec = ec_from_bdev_io(ec_io->bdev_io);
	struct ec_unmap_ctx *uctx;
	uint64_t             map_words;
	int                  persist_rc;

	/*
	 * Home-thread invariant: the bitmap persist below uses
	 * ec->bitmap_chans[], which are owned by the home thread. The
	 * routing layer (ec_submit_unmap's entry hop) ensures every call
	 * chain reaching this function has already been dispatched to home;
	 * the assert is the inner sanity check.
	 */
	assert(spdk_get_thread() == ec->home_thread);

	if (ec_unmap_must_defer_for_scrub(ec, start_stripe, end_stripe)) {
		ec->unmaps_deferred_busy++;
		return -EAGAIN;
	}

	/*
	 * Multi-stripe stripe-busy claim. The claim path runs under the
	 * home thread invariant, so check-all-then-set-all has no
	 * concurrent-claimer race. If any stripe is busy (RMW, full-stripe
	 * write, rebuild, prior UNMAP), defer the whole request via NOMEM
	 * requeue. Under heavy small-write pressure UNMAP can starve; that
	 * surfaces via the unmaps_deferred_busy counter.
	 */
	{
		uint64_t stripe;

		for (stripe = start_stripe; stripe < end_stripe; stripe++) {
			if (ec_stripe_is_dirty(ec, stripe)) {
				ec->unmaps_deferred_busy++;
				return -EAGAIN;
			}
		}
		for (stripe = start_stripe; stripe < end_stripe; stripe++) {
			ec_stripe_set_dirty(ec, stripe);
		}
	}

	uctx = calloc(1, sizeof(*uctx));
	if (uctx == NULL) {
		ec_unmap_release_claims(ec, start_stripe, end_stripe);
		return -ENOMEM;
	}

	/*
	 * Stage the new bitmap state in a per-UNMAP shadow. The live
	 * ec->stripe_unmapped_map is NOT modified here; readers continue
	 * to see the pre-UNMAP state until the persist acks and the
	 * completion callback copies staged -> live atomically.
	 *
	 * TODO(perf): this whole-bitmap calloc per UNMAP is the documented
	 * fstrim hot-path cost. A shared per-channel scratch is NOT safe as-is:
	 * two UNMAPs to different stripe ranges both pass the per-stripe claim
	 * and reach this staging step before the bitmap_persist_in_flight gate
	 * (only checked at ec_bitmap_persist_async below), so they would race
	 * on a shared buffer that each still reads at persist-done. Reuse would
	 * require gating staging on bitmap_persist_in_flight here first -- a
	 * behavioral change to the UNMAP path, deferred pending I/O tests.
	 */
	map_words        = EC_BITMAP_WORDS(ec->num_stripes);
	uctx->staged_map = calloc(map_words, sizeof(uint64_t));
	if (uctx->staged_map == NULL) {
		ec_unmap_release_claims(ec, start_stripe, end_stripe);
		free(uctx);
		return -ENOMEM;
	}

	memcpy(uctx->staged_map, ec->stripe_unmapped_map,
	       map_words * sizeof(uint64_t));
	{
		uint64_t stripe;

		for (stripe = start_stripe; stripe < end_stripe; stripe++) {
			ec_bitmap_word_set(uctx->staged_map, stripe);
		}
	}

	uctx->ec_io           = ec_io;
	uctx->start_stripe    = start_stripe;
	uctx->end_stripe      = end_stripe;
	uctx->disk_lba        = ec_stripe_base_lba(ec, start_stripe);
	uctx->disk_num_blocks = (end_stripe - start_stripe) * ec->strip_size;
	uctx->status          = SPDK_BDEV_IO_STATUS_SUCCESS;
	uctx->cb_fn           = cb_fn;
	uctx->cb_arg          = cb_arg;

	/*
	 * Kick off the bitmap persist. UNMAP wants the m+1 durability
	 * ack (cb_durable) so it can apply staged->live and start fan-out
	 * the moment durability is achieved -- it does not care about
	 * full drainout. -EBUSY (concurrent persist in flight, including
	 * the post-ack pre-drainout window of a previous persist) maps
	 * to -EAGAIN so SPDK's NOMEM queue requeues the bdev_io. Other
	 * failures (-ENOMEM, -EIO with no writable disks) propagate.
	 */
	persist_rc = ec_bitmap_persist_async(ec, uctx->staged_map,
				      ec_unmap_bitmap_persist_done, uctx,
				      NULL, NULL);
	if (persist_rc != 0) {
		ec_unmap_release_claims(ec, start_stripe, end_stripe);
		free(uctx->staged_map);
		free(uctx);
		if (persist_rc == -EBUSY) {
			ec->unmaps_deferred_busy++;
			return -EAGAIN;
		}
		return persist_rc;
	}

	/* ec_unmap_bitmap_persist_done resumes the chain on persist ack. */
	return 0;
}

static void
ec_unmap_bitmap_persist_done(void *cb_arg, int persist_rc)
{
	struct ec_unmap_ctx *uctx = cb_arg;
	struct ec_bdev      *ec   = ec_from_bdev_io(uctx->ec_io->bdev_io);

	if (persist_rc != 0) {
		/*
		 * Persist did not reach the m+1 durability threshold. The
		 * staged bits never became visible to readers (live was
		 * untouched), so there is nothing to roll back. Surface the
		 * failure to the caller; the next UNMAP retry will rebuild
		 * the staged shadow from a fresh snapshot of live. WARNLOG
		 * (not ERRLOG) because the kernel-side retry path will
		 * succeed once a fresh persist lands.
		 */
		SPDK_WARNLOG("EC bdev %s: bitmap persist failed (rc=%d) "
			     "before UNMAP fan-out at stripes [%" PRIu64 "..%" PRIu64 ")\n",
			     ec->bdev.name, persist_rc,
			     uctx->start_stripe, uctx->end_stripe);
		ec_unmap_release_claims(ec, uctx->start_stripe,
					uctx->end_stripe);
		{
			ec_unmap_inner_complete_cb cb_fn  = uctx->cb_fn;
			void                      *cb_arg = uctx->cb_arg;

			free(uctx->staged_map);
			free(uctx);
			cb_fn(cb_arg, SPDK_BDEV_IO_STATUS_FAILED);
		}
		return;
	}

	/*
	 * Apply: durability is established, so the new bits become visible
	 * to readers now.
	 *
	 * The shadow differs from live only in [start_stripe, end_stripe),
	 * so a per-stripe loop calling ec_stripe_set_unmapped is equivalent
	 * to a whole-map memcpy but has two important properties the memcpy
	 * lacks:
	 *
	 *   1. Release-store semantics. The helper publishes each bit with
	 *      __ATOMIC_RELEASE, so a submitter-thread reader using
	 *      acquire-load sees the post-state correctly. A whole-map
	 *      memcpy is a plain word-by-word copy with no ordering, racing
	 *      the reader formally (and surfacing under TSAN even when
	 *      benign on x86).
	 *
	 *   2. Touches only the changed words. At large geometries the
	 *      whole-map is tens of MiB; the per-stripe loop touches one
	 *      word per up-to-64 stripes (the TODO(perf) at the staging
	 *      site notes the same cost).
	 */
	{
		uint64_t stripe;

		for (stripe = uctx->start_stripe; stripe < uctx->end_stripe; stripe++) {
			ec_stripe_set_unmapped(ec, stripe);
		}
	}
	free(uctx->staged_map);
	uctx->staged_map = NULL;

	/*
	 * Fan-out routing: base UNMAPs dispatch on
	 * ec_io->ch->base_chans[] (submitter-owned). When the caller is
	 * already on the submitter thread the inline path runs; otherwise
	 * we hop to the submitter so spdk_bdev_unmap_blocks goes out on
	 * the channel-owning thread.
	 */
	if (spdk_likely(spdk_get_thread() == uctx->ec_io->submitter_thread)) {
		int fanout_rc = ec_unmap_fanout(uctx);
		if (fanout_rc != 0) {
			SPDK_ERRLOG("EC bdev %s: UNMAP fan-out submitted no I/O at "
				    "stripes [%" PRIu64 "..%" PRIu64 ") (rc=%d); completing FAILED "
				    "(bitmap already marked unmapped; reads still "
				    "synthesize zeros)\n",
				    ec->bdev.name, uctx->start_stripe,
				    uctx->end_stripe, fanout_rc);
			/*
			 * Fan-out could not submit any I/O. The bitmap already
			 * says "unmapped" so reads of the affected stripes
			 * return synthesized zeros regardless -- the read-as-
			 * zero contract is intact. Still complete the bdev_io
			 * as FAILED because the caller asked for physical
			 * reclamation and we did not deliver any of it; the
			 * next UNMAP / rebuild will retry.
			 */
			ec_unmap_release_claims(ec, uctx->start_stripe,
						uctx->end_stripe);
			{
				ec_unmap_inner_complete_cb cb_fn  = uctx->cb_fn;
				void                      *cb_arg = uctx->cb_arg;

				free(uctx);
				cb_fn(cb_arg, SPDK_BDEV_IO_STATUS_FAILED);
			}
		}
		return;
	}

	{
		int send_rc;

		send_rc = spdk_thread_send_msg(uctx->ec_io->submitter_thread,
			ec_unmap_dispatch_fanout_on_submitter, uctx);
		if (send_rc != 0) {
			/*
			 * Cannot reach the submitter. Release the stripe-busy
			 * claims, mark FAILED, and route the cb_fn invocation
			 * back via a second send_msg (cb_fn ultimately calls
			 * spdk_bdev_io_complete, which asserts owner thread).
			 * If even the second send_msg fails, the bdev_io stays
			 * in flight rather than being completed on the wrong
			 * thread. The bitmap apply already happened, so the
			 * read-as-zero contract is intact; the FAILED return
			 * tells the caller no physical reclamation occurred.
			 */
			int complete_rc;

			SPDK_ERRLOG("EC bdev %s: cannot hand off UNMAP fan-out "
				    "to submitter thread '%s' (rc=%d %s) at stripes "
				    "[%" PRIu64 "..%" PRIu64 "); failing\n",
				    ec->bdev.name,
				    spdk_thread_get_name(uctx->ec_io->submitter_thread),
				    send_rc, spdk_strerror(-send_rc),
				    uctx->start_stripe, uctx->end_stripe);
			ec_unmap_release_claims(ec, uctx->start_stripe,
						uctx->end_stripe);
			uctx->status = SPDK_BDEV_IO_STATUS_FAILED;
			complete_rc = spdk_thread_send_msg(
				uctx->ec_io->submitter_thread,
				ec_unmap_fire_cb_on_submitter, uctx);
			if (complete_rc != 0) {
				SPDK_ERRLOG("EC bdev %s: also cannot hand off "
					    "failure completion to submitter thread "
					    "'%s' (rc=%d %s); bdev_io stays in-flight\n",
					    ec->bdev.name,
					    spdk_thread_get_name(uctx->ec_io->submitter_thread),
					    complete_rc, spdk_strerror(-complete_rc));
				free(uctx);
			}
		}
	}
}

/*
 * send_msg target for routing the UNMAP fan-out to the submitter
 * thread after the bitmap persist completes on home. Runs ec_unmap_fanout
 * on the submitter; on synchronous fan-out failure, fires cb_fn(FAILED)
 * directly here (we are on submitter = bdev_io owner, so cb_fn's
 * eventual spdk_bdev_io_complete is safe).
 */
static void
ec_unmap_dispatch_fanout_on_submitter(void *ctx)
{
	struct ec_unmap_ctx *uctx = ctx;
	struct ec_bdev      *ec   = ec_from_bdev_io(uctx->ec_io->bdev_io);
	int                  rc;

	rc = ec_unmap_fanout(uctx);
	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: UNMAP fan-out submitted no I/O at "
			    "stripes [%" PRIu64 "..%" PRIu64 ") (rc=%d); completing FAILED "
			    "(bitmap already marked unmapped; reads still "
			    "synthesize zeros)\n",
			    ec->bdev.name, uctx->start_stripe,
			    uctx->end_stripe, rc);
		ec_unmap_release_claims(ec, uctx->start_stripe,
					uctx->end_stripe);
		{
			ec_unmap_inner_complete_cb cb_fn  = uctx->cb_fn;
			void                      *cb_arg = uctx->cb_arg;

			free(uctx);
			cb_fn(cb_arg, SPDK_BDEV_IO_STATUS_FAILED);
		}
	}
}

/*
 * send_msg target for routing the cb_fn invocation back to the
 * submitter on the second-stage failure where the fan-out hand-off
 * itself failed. uctx->status carries the completion status (set by
 * the caller to FAILED before send_msg). Frees uctx after firing.
 */
static void
ec_unmap_fire_cb_on_submitter(void *ctx)
{
	struct ec_unmap_ctx        *uctx   = ctx;
	ec_unmap_inner_complete_cb  cb_fn  = uctx->cb_fn;
	void                       *cb_arg = uctx->cb_arg;
	enum spdk_bdev_io_status    status = uctx->status;

	free(uctx);
	cb_fn(cb_arg, status);
}

static int
ec_unmap_fanout(struct ec_unmap_ctx *uctx)
{
	struct ec_bdev_io *ec_io = uctx->ec_io;
	struct ec_bdev    *ec    = ec_from_bdev_io(ec_io->bdev_io);
	uint32_t           i;
	uint32_t           writable = 0;
	int                rc;

	/*
	 * Dispatch invariant: base UNMAPs use ec_io->ch->base_chans[],
	 * owned by the submitter thread. The bitmap-persist-done ->
	 * submitter hop in ec_unmap_bitmap_persist_done ensures this
	 * holds; when the caller is already on the submitter thread the
	 * hop inlines.
	 */
	assert(spdk_get_thread() == ec_io->submitter_thread);

	for (i = 0; i < ec->n; i++) {
		if (ec_slot_is_writable(ec, i)) {
			writable++;
		}
	}
	if (writable == 0) {
		SPDK_ERRLOG("EC bdev %s: UNMAP fan-out: no writable slots\n",
			    ec->bdev.name);
		return -EIO;
	}

	uctx->writes_remaining = writable;

	for (i = 0; i < ec->n; i++) {
		if (!ec_slot_is_writable(ec, i)) {
			continue;
		}
		rc = spdk_bdev_unmap_blocks(ec->descs[i],
					    ec_io->ch->base_chans[i],
					    uctx->disk_lba,
					    uctx->disk_num_blocks,
					    ec_unmap_child_complete,
					    uctx);
		if (rc != 0) {
			SPDK_WARNLOG("EC bdev %s: UNMAP submit failed slot %u "
				     "(rc=%d) lba=%" PRIu64 " len=%" PRIu64 "\n",
				     ec->bdev.name, i, rc,
				     uctx->disk_lba, uctx->disk_num_blocks);
			uctx->writes_remaining--;
			uctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}

	if (uctx->writes_remaining == 0) {
		/* All submits failed synchronously; no callbacks will fire. */
		return -EIO;
	}
	return 0;
}

static void
ec_unmap_child_complete(struct spdk_bdev_io *child, bool success, void *cb_arg)
{
	struct ec_unmap_ctx *uctx = cb_arg;
	struct ec_bdev      *ec   = ec_from_bdev_io(uctx->ec_io->bdev_io);

	spdk_bdev_free_io(child);

	if (!success) {
		uctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		__atomic_fetch_add(&ec->unmap_fanout_misses, 1, __ATOMIC_RELAXED);
	}

	if (--uctx->writes_remaining > 0) {
		return;
	}

	ec_unmap_release_claims(ec, uctx->start_stripe, uctx->end_stripe);

	/*
	 * Don't bump unmaps_completed here -- that happens at parent
	 * completion (see ec_unmap_inner_complete_default); bumping per
	 * sub-segment would over-count a multi-segment request.
	 */
	{
		ec_unmap_inner_complete_cb cb_fn  = uctx->cb_fn;
		void                      *cb_arg = uctx->cb_arg;
		enum spdk_bdev_io_status   status = uctx->status;

		free(uctx);
		cb_fn(cb_arg, status);
	}
}

/* =========================================================================
 * Multi-segment dispatcher
 *
 * See the long comment above struct ec_unmap_split_ctx for the design
 * rationale (serial-not-parallel, inner-first ordering, why the head
 * and tail RMW segments leave the bitmap untouched).
 * ========================================================================= */

static int
ec_submit_unmap_split(struct ec_bdev_io *ec_io,
		      uint64_t head_off, uint64_t head_len,
		      uint64_t inner_start_stripe, uint64_t inner_end_stripe,
		      uint64_t tail_off, uint64_t tail_len)
{
	struct ec_unmap_split_ctx *sctx;
	bool                       has_inner = (inner_start_stripe <
						inner_end_stripe);
	bool                       has_head  = (head_len > 0);
	int                        rc;

	sctx = calloc(1, sizeof(*sctx));
	if (sctx == NULL) {
		return -ENOMEM;
	}

	sctx->ec_io               = ec_io;
	sctx->head_offset_blocks  = head_off;
	sctx->head_num_blocks     = head_len;
	sctx->tail_offset_blocks  = tail_off;
	sctx->tail_num_blocks     = tail_len;
	sctx->status              = SPDK_BDEV_IO_STATUS_SUCCESS;

	/*
	 * Submit the FIRST present segment synchronously from the entry point
	 * so a sync error (-EAGAIN from persist or stripe-busy contention,
	 * -ENOMEM from context alloc) propagates back to the bdev layer
	 * for a clean NOMEM-queue requeue with nothing in flight.
	 *
	 * Inner first when present (largest reclamation; most likely to
	 * -EAGAIN on bitmap persist contention). Otherwise the boundary-
	 * straddler case (crosses exactly one stripe boundary with no full
	 * inner stripe) starts with head. has_head must be true in that
	 * case: tail-only is unreachable here because off would be stripe-
	 * aligned with start_stripe == end_stripe, which the single-stripe
	 * shortcut in ec_submit_unmap would have routed to write-zeros
	 * before reaching this dispatcher.
	 */
	if (has_inner) {
		sctx->next_stage = EC_UNMAP_SPLIT_STAGE_HEAD;
		rc = ec_unmap_inner_fanout(ec_io,
					   inner_start_stripe, inner_end_stripe,
					   ec_unmap_split_segment_done, sctx);
	} else {
		assert(has_head);
		sctx->next_stage = EC_UNMAP_SPLIT_STAGE_TAIL;
		rc = ec_submit_rmw_zero_fill_range(ec_io,
						   head_off, head_len,
						   ec_unmap_split_segment_done,
						   sctx);
	}

	if (rc != 0) {
		free(sctx);
		return rc;
	}

	/* First segment is async; ec_unmap_split_segment_done -> _advance will
	 * drive the remaining segments. */
	return 0;
}

/*
 * Submit one zero-fill segment (head or tail), or skip it and advance if the segment
 * is empty. The caller sets sctx->next_stage before calling. On a synchronous
 * submit failure the parent is marked FAILED and completed -- the kernel
 * retries the whole UNMAP, and RMW zero-fill plus the already-applied bitmap
 * state are idempotent, so the retry converges. On success the segment completes
 * asynchronously via ec_unmap_split_segment_done, which re-enters advance.
 */
static void
ec_unmap_split_submit_segment(struct ec_unmap_split_ctx *sctx,
			  uint64_t off, uint64_t len)
{
	int rc;

	if (len == 0) {
		ec_unmap_split_continue(sctx);
		return;
	}

	rc = ec_submit_rmw_zero_fill_range(sctx->ec_io, off, len,
					   ec_unmap_split_segment_done, sctx);
	if (rc != 0) {
		sctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		ec_unmap_split_complete(sctx);
	}
}

/*
 * Submit the next segment, or complete the parent if none remain or the
 * status has already gone FAILED. Recursion is bounded at three frames
 * (one per stage) because every stage either submits async (returns
 * without recursing) or advances on a missing segment / status==FAILED.
 */
static void
ec_unmap_split_continue(struct ec_unmap_split_ctx *sctx)
{
	/*
	 * Short-circuit on prior failure. We don't waste I/O on remaining
	 * segments because the parent bdev_io is going to complete FAILED
	 * regardless and the kernel will retry the whole UNMAP -- at
	 * which point every segment runs fresh (RMW zero-fill and bitmap
	 * persist are both idempotent).
	 */
	if (sctx->status != SPDK_BDEV_IO_STATUS_SUCCESS) {
		ec_unmap_split_complete(sctx);
		return;
	}

	switch (sctx->next_stage) {
	case EC_UNMAP_SPLIT_STAGE_HEAD:
		sctx->next_stage = EC_UNMAP_SPLIT_STAGE_TAIL;
		ec_unmap_split_submit_segment(sctx, sctx->head_offset_blocks,
					  sctx->head_num_blocks);
		return;

	case EC_UNMAP_SPLIT_STAGE_TAIL:
		sctx->next_stage = EC_UNMAP_SPLIT_STAGE_DONE;
		ec_unmap_split_submit_segment(sctx, sctx->tail_offset_blocks,
					  sctx->tail_num_blocks);
		return;

	case EC_UNMAP_SPLIT_STAGE_DONE:
		ec_unmap_split_complete(sctx);
		return;
	}
}

/*
 * send_msg target that re-enters ec_unmap_split_continue on the home
 * thread. Used by ec_unmap_split_segment_done to route the post-
 * segment dispatch chain back to home, because the next segment's
 * dispatch eventually reaches ec_rmw_submit_core (head/tail RMW
 * zero-fill via ec_submit_rmw_zero_fill_range) which asserts home.
 */
static void
ec_unmap_split_continue_on_home(void *ctx)
{
	ec_unmap_split_continue((struct ec_unmap_split_ctx *)ctx);
}

static void
ec_unmap_split_segment_done(void *cb_arg, enum spdk_bdev_io_status status)
{
	struct ec_unmap_split_ctx *sctx = cb_arg;
	struct ec_bdev            *ec   = ec_from_bdev_io(sctx->ec_io->bdev_io);

	if (status != SPDK_BDEV_IO_STATUS_SUCCESS) {
		sctx->status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	/*
	 * Route continue+dispatch to home. cb_fn can fire on submitter (success
	 * path: fan-out completions land on submitter) or on home (persist failure).
	 * The next-segment dispatch needs home (ec_submit_rmw_zero_fill_range ->
	 * ec_rmw_submit_core asserts home). The final spdk_bdev_io_complete in
	 * ec_unmap_split_complete handles its own owner-thread routing.
	 */
	if (spdk_unlikely(spdk_get_thread() != ec->home_thread)) {
		int send_rc = spdk_thread_send_msg(ec->home_thread,
			ec_unmap_split_continue_on_home, sctx);
		if (send_rc != 0) {
			/*
			 * Cannot reach home to advance the chain. The bitmap
			 * apply may have already happened for inner; mark
			 * FAILED and let ec_unmap_split_complete run inline
			 * here to finalize. The complete function will route
			 * the bdev_io completion to the submitter (= owner).
			 */
			SPDK_ERRLOG("EC bdev %s: cannot hand off multi-segment "
				    "UNMAP advance to home thread '%s' (rc=%d %s); "
				    "failing\n",
				    ec->bdev.name,
				    spdk_thread_get_name(ec->home_thread),
				    send_rc, spdk_strerror(-send_rc));
			sctx->status = SPDK_BDEV_IO_STATUS_FAILED;
			ec_unmap_split_complete(sctx);
		}
		return;
	}

	ec_unmap_split_continue(sctx);
}

static void
ec_unmap_split_complete(struct ec_unmap_split_ctx *sctx)
{
	struct ec_bdev_io       *ec_io  = sctx->ec_io;
	struct ec_bdev          *ec     = ec_from_bdev_io(ec_io->bdev_io);
	enum spdk_bdev_io_status status = sctx->status;

	/*
	 * Bump unmaps_completed / unmaps_failed at the parent-completion
	 * boundary (same convention as ec_unmap_inner_complete_default) so
	 * the closed accounting identity in bdev_ec_internal.h holds.
	 */
	if (status == SPDK_BDEV_IO_STATUS_SUCCESS) {
		__atomic_fetch_add(&ec->unmaps_completed, 1, __ATOMIC_RELAXED);
	} else {
		__atomic_fetch_add(&ec->unmaps_failed,    1, __ATOMIC_RELAXED);
	}
	free(sctx);

	/*
	 * spdk_bdev_io_complete asserts the owner thread.
	 * ec_unmap_split_continue may run on home (routed in
	 * ec_unmap_split_segment_done), so this function can be reached
	 * with submitter != current. Route via send_msg in that case.
	 * Stash status in ec_io->status so the helper can read it on the
	 * other thread.
	 */
	if (spdk_likely(spdk_get_thread() == ec_io->submitter_thread)) {
		spdk_bdev_io_complete(ec_io->bdev_io, status);
		return;
	}

	ec_io->status = status;
	ec_io_route_complete_to_submitter(ec_io, "multi-segment UNMAP completion");
}
