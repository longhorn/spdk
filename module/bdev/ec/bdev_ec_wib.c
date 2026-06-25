/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_wib.c -- Write-Intent Bitmap (WIB) machinery.
 *
 * The WIB protects against the RMW write-hole. One dirty bit per
 * region of EC_WIB_REGION_STRIPES stripes, stored on-disk as two
 * alternating front-placed copies on every parity disk, immediately
 * after the unmapped-bitmap reservation. Three flows live here:
 *
 *   Runtime persist  RMW asks the WIB to mark a region dirty on disk
 *                    before its data writes go out. The idle poller
 *                    later clears region bits that have been quiet for
 *                    EC_WIB_IDLE_MS.
 *
 *   Startup load     ec_bdev_create_async reads both on-disk copies
 *                    from every parity disk, picks the higher
 *                    generation, and merges into ec->wib_region_map.
 *
 *   Status query     ec_bdev_get_wib_status answers the JSON-RPC.
 *
 * Contract with RMW (in bdev_ec_rmw.c): a region's dirty bit must be
 * on disk before any data or parity write for that region is issued.
 * RMW calls ec_wib_persist; if a persist is already in flight, the
 * RMW context queues on ec->wib_deferred_writes and is drained by
 * ec_wib_deferred_drain when the in-flight persist completes.
 */

#include "bdev_ec_internal.h"

#include "spdk/bdev_module.h"
#include "spdk/crc32.h"
#include "spdk/string.h"
#include "spdk/thread.h"

#include <assert.h>

/* =========================================================================
 * Write-Intent Bitmap (WIB) helpers
 * ========================================================================= */

/*
 * Count dirty WIB regions. Returns 0 if wib_region_map is not yet
 * allocated. O(num_regions); region counts are small (a few hundred
 * for typical multi-TiB volumes) so a linear scan is fine.
 */
uint32_t
ec_wib_count_dirty(const struct ec_bdev *ec)
{
	uint32_t dirty = 0, region;

	if (!ec->wib_region_map) {
		return 0;
	}
	for (region = 0; region < ec->wib_num_regions; region++) {
		if (ec_wib_region_is_dirty(ec, region)) {
			dirty++;
		}
	}
	return dirty;
}

/* CRC32C over [buf, crc_ptr), using SPDK's hardware-accelerated implementation. */
static uint32_t
ec_wib_buf_crc(const void *buf, const uint32_t *crc_ptr)
{
	uint32_t len = (uint32_t)((const uint8_t *)crc_ptr - (const uint8_t *)buf);

	return spdk_crc32c_update(buf, len, 0);
}

/*
 * Serialize the in-memory WIB into ec->wib_buf.
 * Layout: [ ec_wib_header ][ region_bits[] ][ crc32c u32 ]
 */
static void
ec_wib_fill_buf(struct ec_bdev *ec)
{
	struct ec_wib_header *hdr = (struct ec_wib_header *)ec->wib_buf;
	uint64_t             *bits;
	uint32_t              map_words = EC_BITMAP_WORDS(ec->wib_num_regions);
	uint64_t              strip_bytes = (uint64_t)ec->strip_size * ec->bdev.blocklen;
	uint32_t             *crc_ptr;

	/*
	 * The serialized WIB (header + region words + CRC) must fit in the
	 * one-strip wib_buf. ec_max_num_stripes bounds num_stripes -- and thus
	 * wib_num_regions -- so this always holds; assert to catch a geometry
	 * that slipped past the create / resize ceiling checks.
	 */
	assert(sizeof(struct ec_wib_header) + (uint64_t)map_words * sizeof(uint64_t)
	       + sizeof(uint32_t) <= strip_bytes);

	hdr->magic       = EC_WIB_MAGIC;
	hdr->version     = EC_WIB_VERSION;
	hdr->generation  = ec->wib_generation;
	hdr->num_regions = ec->wib_num_regions;

	bits = (uint64_t *)(hdr + 1);
	memcpy(bits, ec->wib_region_map, map_words * sizeof(uint64_t));

	crc_ptr = (uint32_t *)(bits + map_words);
	*crc_ptr = ec_wib_buf_crc(ec->wib_buf, crc_ptr);
}

/*
 * Context for a multi-disk WIB persist operation.
 * One ec_wib_persist_ctx is allocated per persist call; it counts down
 * the m parity-disk writes and fires the caller's callback when all complete.
 */
struct ec_wib_persist_ctx {
	struct ec_bdev   *ec;
	uint32_t          writes_remaining;
	int               status;          /* 0 or first error code */

	/* callback to invoke when all m writes complete; NULL = fire-and-forget */
	void (*cb)(void *cb_arg, int rc);
	void             *cb_arg;
	uint8_t           next_copy;   /* copy index being written; flip active on success */
};

/*
 * Drain all RMW contexts that were deferred while waiting for a WIB
 * persist to confirm their dirty bits on disk. Called once the
 * follow-up persist completes (successfully or not).
 */
static void
ec_wib_deferred_drain(void *cb_arg, int rc)
{
	struct ec_bdev    *ec = cb_arg;
	struct ec_rmw_ctx *mctx, *tmp;

	if (rc != 0) {
		/*
		 * The follow-up WIB persist failed: any dirty bit set after
		 * the prior persist started is still in memory only, so the
		 * deferred RMWs' resumed reads must NOT proceed -- doing so
		 * would (after a successful subsequent persist) let writes
		 * land with stale parity protected only by an unrecorded
		 * write-intent, recreating the write-hole the WIB exists to
		 * prevent (visible on crash via degraded read of the affected
		 * stripe). Complete each deferred RMW with NOMEM status so
		 * SPDK retries the bdev_io from scratch; the next attempt will
		 * re-enter ec_rmw_persist_and_dispatch and either find the
		 * in-memory bit already durable from a later persist or
		 * trigger a fresh persist. The in-memory bit is left set: it
		 * correctly reflects "this region needs persisting" and will
		 * be drained by the next successful persist.
		 */
		SPDK_ERRLOG("EC bdev %s: WIB deferred persist failed "
			    "(rc=%d); failing deferred RMW(s) via NOMEM "
			    "retry to preserve write-intent ordering\n",
			    ec->bdev.name, rc);
		TAILQ_FOREACH_SAFE(mctx, &ec->wib_deferred_writes,
				   wib_defer_link, tmp) {
			TAILQ_REMOVE(&ec->wib_deferred_writes, mctx,
				     wib_defer_link);
			mctx->status = SPDK_BDEV_IO_STATUS_NOMEM;
			ec_rmw_complete(mctx);
		}
		return;
	}

	/*
	 * Persist succeeded: resume each deferred RMW at the read-dispatch
	 * step. The WIB persist runs BEFORE reads, so a deferred RMW has
	 * not read anything yet -- its chunk_bufs are zeroed DMA. Resuming
	 * at ec_rmw_submit_writes would fan out all-zero data +
	 * parity-encoded-from-zeros over live stripe data -> silent
	 * corruption.
	 *
	 * TAILQ_FOREACH_SAFE captures tmp before the body runs, so the
	 * post-call ownership transfer in ec_rmw_dispatch_reads (which may
	 * self-complete and free mctx on total submit failure) is safe.
	 */
	TAILQ_FOREACH_SAFE(mctx, &ec->wib_deferred_writes, wib_defer_link,
			   tmp) {
		TAILQ_REMOVE(&ec->wib_deferred_writes, mctx, wib_defer_link);
		ec_rmw_dispatch_reads(mctx);
	}
}

static void
ec_wib_persist_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_wib_persist_ctx *pctx = cb_arg;
	struct ec_bdev            *ec   = pctx->ec;

	spdk_bdev_free_io(bdev_io);

	if (!success && pctx->status == 0) {
		pctx->status = -EIO;
		SPDK_WARNLOG("EC bdev %s: WIB persist write failed\n",
			     ec->bdev.name);
	}

	pctx->writes_remaining--;
	if (pctx->writes_remaining != 0) {
		return;
	}

	/*
	 * All m parity-disk writes complete.
	 * Flip active_copy only on success.
	 *
	 * Clear wib_persist_in_flight BEFORE invoking cb, in case cb
	 * re-enters ec_wib_persist.
	 */
	ec->wib_persist_in_flight = false;
	if (pctx->status == 0) {
		ec->wib_active_copy = pctx->next_copy;
	}

	if (pctx->cb) {
		pctx->cb(pctx->cb_arg, pctx->status);
	}
	/* Save status before free; pctx is freed below but the drain
	 * branch below needs to know if the persist succeeded. */
	int persist_status = pctx->status;
	free(pctx);

	/*
	 * Release any deferred slot channels before kicking the follow-up
	 * persist below -- otherwise sustained WIB churn would keep restarting
	 * the persist and starve the cleanup. Never frees ec.
	 */
	ec_drain_deferred_slot_releases(ec);

	/*
	 * If a new dirty bit was set while this persist was in flight,
	 * start a follow-up persist so the bit reaches disk.
	 * Deferred writes are drained only once no repersist is needed.
	 */
	if (ec->wib_repersist_needed) {
		ec->wib_repersist_needed = false;
		if (ec_wib_persist(ec, NULL, NULL) != 0) {
			ec_wib_deferred_drain(ec, -ENOMEM);
		}
	} else if (!TAILQ_EMPTY(&ec->wib_deferred_writes)) {
		/*
		 * Propagate the just-completed persist's status: if it failed,
		 * the in-memory dirty bits set after it started are not on
		 * disk, so the deferred RMWs must not have their data writes
		 * submitted (write-hole on crash). ec_wib_deferred_drain
		 * completes the deferred RMWs with NOMEM so SPDK retries them.
		 */
		ec_wib_deferred_drain(ec, persist_status);
	}

	/*
	 * Last statement: finish a deferred delete. In that case this frees ec,
	 * so nothing may touch ec after this call. Only the async write
	 * completion drains -- the sync-finish path runs nested inside
	 * ec_wib_persist and never coincides with a pending deferral.
	 */
	ec_drain_deferred_unregister(ec);
}

/*
 * Finish a WIB persist that completes synchronously inside ec_wib_persist --
 * every parity write was skipped (disk failed) or failed to submit, so no
 * ec_wib_persist_write_cb will ever run for this ctx. Clears the in-flight
 * flag, invokes the caller's completion, frees the ctx, and drains any RMWs
 * deferred behind this persist (see below). Returns 0 (ec_wib_persist's
 * success return) for use as a tail call.
 */
static int
ec_wib_persist_sync_finish(struct ec_bdev *ec, struct ec_wib_persist_ctx *pctx)
{
	int status = pctx->status;

	ec->wib_persist_in_flight = false;
	if (pctx->cb) {
		pctx->cb(pctx->cb_arg, status);
	}
	free(pctx);

	/*
	 * No parity write was issued, so no ec_wib_persist_write_cb will fire
	 * to drain RMWs deferred behind this persist. A follow-up persist would
	 * hit the same dead parity, so drain here instead of looping. Without
	 * this, a parity failure landing between an in-flight persist and its
	 * repersist strands every deferred RMW (I/O hang). Proceeding without
	 * the on-disk bit is safe: with no writable parity there is nothing to
	 * leave inconsistent, and rebuild recomputes parity wholesale.
	 */
	ec->wib_repersist_needed = false;
	if (!TAILQ_EMPTY(&ec->wib_deferred_writes)) {
		ec_wib_deferred_drain(ec, status);
	}
	return 0;
}

/*
 * Persist the in-memory WIB region_map to the inactive on-disk copy on all
 * m parity disks, then flip the active copy on completion.
 *
 * cb / cb_arg: invoked when all m writes complete. Pass NULL/NULL for
 *              fire-and-forget (e.g. the idle-clear path).
 *
 * Caller must ensure ec->wib_persist_in_flight is false; this sets it true.
 *
 * Home-thread only (asserted), so the WIB has a single writer. Each persist
 * writes the inactive double-buffer slot (1 - active_copy) and bumps
 * wib_generation, which the load path uses to pick the latest valid copy.
 * Concurrent writers would collide on the slot (interleaving bytes so the
 * copy fails CRC) and on the generation counter.
 *
 * Per-channel writers would be cleaner long-term, but the I/O-path callers
 * (ec_submit_full_write, ec_rmw_persist_and_dispatch) still consume the
 * synchronous rc for rollback / NOMEM retry. Converting that to a
 * cb-delivered NOMEM is a deferred follow-up; until then the assert enforces
 * the single writer.
 */
int
ec_wib_persist(struct ec_bdev *ec,
	       void (*cb)(void *cb_arg, int rc), void *cb_arg)
{
	struct ec_wib_persist_ctx *pctx;
	uint8_t    next_copy;
	uint64_t   lba;
	uint32_t   j;
	int        rc;

	/* Home-thread only; see rationale above. */
	assert(spdk_get_thread() == ec->home_thread);

	pctx = calloc(1, sizeof(*pctx));
	if (!pctx) {
		SPDK_WARNLOG("EC bdev %s: WIB persist context alloc failed\n", ec->bdev.name);
		return -ENOMEM;
	}

	ec->wib_generation++;
	next_copy = 1 - ec->wib_active_copy;

	ec_wib_fill_buf(ec);

	pctx->ec               = ec;
	pctx->writes_remaining = ec->m;
	pctx->status           = 0;
	pctx->cb               = cb;
	pctx->cb_arg           = cb_arg;
	pctx->next_copy        = next_copy;

	lba = ec_wib_lba(ec, next_copy);

	ec->wib_persist_in_flight = true;

	for (j = 0; j < ec->m; j++) {
		uint32_t pslot = ec->k + j;

		if (!ec->descs[pslot] || !ec->wib_chans[j] ||
		    ec->base_states[pslot] == EC_BASE_STATE_FAILED) {
			/*
			 * Parity disk failed -- skip it; the WIB still lands on
			 * the surviving parity disks. With all parity failed,
			 * every slot is skipped: there is no on-disk parity to
			 * fall out of sync with the data, so the WIB has nothing
			 * to record and the persist completes cleanly.
			 */
			pctx->writes_remaining--;
			if (pctx->writes_remaining == 0) {
				return ec_wib_persist_sync_finish(ec, pctx);
			}
			continue;
		}

		rc = spdk_bdev_write(ec->descs[pslot],
				     ec->wib_chans[j],
				     ec->wib_buf,
				     lba * ec->bdev.blocklen,
				     ec->strip_size * ec->bdev.blocklen,
				     ec_wib_persist_write_cb,
				     pctx);
		if (rc != 0) {
			SPDK_WARNLOG("EC bdev %s: failed to submit WIB write "
				     "to parity slot %u (rc=%d)\n",
				     ec->bdev.name, pslot, rc);
			if (pctx->status == 0) {
				pctx->status = rc;
			}
			pctx->writes_remaining--;
			if (pctx->writes_remaining == 0) {
				return ec_wib_persist_sync_finish(ec, pctx);
			}
		}
	}

	/* wib_active_copy is flipped in the write callback, not here. */
	return 0;
}

/*
 * Runs every EC_WIB_POLL_PERIOD_US microseconds. Scans all regions:
 * for each region where in_flight==0 and the region has been dirty on
 * disk for longer than EC_WIB_IDLE_MS, clears the in-memory bit and
 * initiates a fire-and-forget persist to write the cleared state.
 *
 * Only one persist can be in flight at a time (wib_persist_in_flight).
 * If a persist is already running, the poller defers and tries again
 * on the next tick.
 */
int
ec_wib_idle_poller_cb(void *arg)
{
	struct ec_bdev *ec = arg;
	uint64_t        now = spdk_get_ticks();
	uint64_t        ticks_per_second = spdk_get_ticks_hz();
	uint64_t        idle_ticks = ticks_per_second * EC_WIB_IDLE_MS / 1000;
	uint32_t        region;
	bool            any_cleared = false;

	/*
	 * No resize interlock is required. Resize never touches wib_buf
	 * (it's geometry-invariant -- one strip on every geometry), so a
	 * persist's in-flight DMA stays valid across a concurrent resize.
	 * ec_wib_fill_buf reads wib_region_map and wib_num_regions
	 * synchronously inside this poller before submitting any I/O;
	 * subsequent reallocs in ec_resize_quiesce_cb do not affect work
	 * already in flight. The persist's on-disk LBA is fixed by
	 * strip_size + bitmap reservation, both unchanged across resize,
	 * so the on-disk slot is the same before and after.
	 */
	if (ec->wib_persist_in_flight) {
		return SPDK_POLLER_BUSY;
	}

	for (region = 0; region < ec->wib_num_regions; region++) {
		if (!ec_wib_region_is_dirty(ec, region)) {
			continue;
		}
		if (ec_wib_region_inflight_get(ec, region) > 0) {
			continue;
		}
		if ((now - ec->wib_region_dirty_ticks[region]) < idle_ticks) {
			continue;
		}

		ec_wib_region_clear_dirty(ec, region);
		any_cleared = true;
	}

	if (any_cleared) {
		int persist_rc = ec_wib_persist(ec, NULL, NULL);

		if (persist_rc != 0) {
			/*
			 * The in-memory bits were cleared but the on-disk WIB
			 * was not updated. This is not a data hazard: the
			 * on-disk bits stay set, so those regions are simply
			 * re-examined (and harmlessly re-scrubbed) at the next
			 * startup. Log it so a persistently failing parity disk
			 * is visible rather than the clear being dropped
			 * silently.
			 */
			SPDK_WARNLOG("EC bdev %s: idle WIB clear persist failed "
				     "(rc=%d); on-disk dirty bits remain and will "
				     "be re-examined at next startup\n",
				     ec->bdev.name, persist_rc);
		}
	}

	return any_cleared ? SPDK_POLLER_BUSY : SPDK_POLLER_IDLE;
}

/* =========================================================================
 * WIB load and startup scrub
 *
 * ec_wib_load_async -- read the on-disk WIB from parity disks at startup.
 *                  Selects the copy with the highest generation counter.
 *                  Merges across all m parity disks (OR of all copies):
 *                  if any copy has a bit set, the region is considered dirty.
 *                  Asynchronous: a callback chain reads via wib_chans[]
 *                  without ever blocking the reactor.
 *
 * ec_wib_validate_buf -- checks magic, version, CRC; returns 0 if valid.
 *
 * The startup scrub consumes the loaded WIB -- it replays each dirty
 * region (re-encoding parity from data) and clears the bit when done.
 * The scrub chain (ec_scrub_*) lives in bdev_ec_rebuild.c; the guard that
 * defers a write over a region still being scrubbed lives with
 * ec_submit_rmw_write. See those for the details.
 * ========================================================================= */

/*
 * Returns 0 if the buffer contains a valid WIB copy (magic, version, CRC).
 * Returns -EINVAL otherwise.
 */
static int
ec_wib_validate_buf(const struct ec_bdev *ec, const void *buf, uint64_t *gen_out)
{
	const struct ec_wib_header *hdr = (const struct ec_wib_header *)buf;
	const uint64_t             *bits;
	const uint32_t             *crc_ptr;
	uint32_t                    map_words;
	uint32_t                    expected_crc, actual_crc;

	if (hdr->magic != EC_WIB_MAGIC) {
		return -EINVAL;
	}
	if (hdr->version != EC_WIB_VERSION) {
		return -EINVAL;
	}
	if (hdr->num_regions != ec->wib_num_regions) {
		return -EINVAL;
	}

	map_words = EC_BITMAP_WORDS(ec->wib_num_regions);
	bits      = (const uint64_t *)(hdr + 1);
	crc_ptr   = (const uint32_t *)(bits + map_words);

	/* Recompute CRC32C over header + bits */
	actual_crc   = ec_wib_buf_crc(buf, crc_ptr);
	expected_crc = *crc_ptr;

	if (actual_crc != expected_crc) {
		return -EINVAL;
	}

	*gen_out = hdr->generation;
	return 0;
}

/*
 * ec_wib_load_async context.
 *
 * Drives a callback chain: for each parity disk, reads copy 0 then copy 1
 * without blocking the reactor, picks the copy with the higher generation,
 * ORs its region_bits into ec->wib_region_map, then advances to the next disk.
 *
 * After all m disks are processed, ec->wib_region_map reflects the union of
 * the valid copies: a region is dirty if any parity disk's valid copy has
 * the bit set. If every copy on every disk is invalid (first boot or
 * corrupt header/CRC), the in-memory map stays all-zeros -- safe because
 * there is nothing to scrub.
 *
 * Called from ec_bdev_create on the EC bdev's home thread before the bdev is
 * registered, so no other I/O competes on wib_chans[].
 *
 * done_fn is always called. rc is 0 once the async read chain starts (per-disk
 * read failures are non-fatal and skipped); rc is -ENOMEM if context or
 * DMA-buffer allocation fails before the chain starts.
 */
struct ec_wib_load_async_ctx {
	struct ec_bdev      *ec;
	void                *scratch;    /* DMA buffer: 2 x buf_bytes        */
	size_t               buf_bytes;
	void                *bufa;       /* copy 0 target (== scratch)        */
	void                *bufb;       /* copy 1 target (scratch+buf_bytes) */
	uint32_t             parity_idx;
	/* per-disk state, reset for each parity disk */
	uint64_t             gen_a, gen_b;
	bool                 valid_a, valid_b;
	/* overall */
	bool                 any_valid;
	/* completion */
	ec_bdev_create_cb_fn done_fn;
	void                *done_arg;
};

/* -------------------------------------------------------------------------
 * ec_wib_load_async -- non-blocking WIB load driven by callback chain.
 *
 * Submits each read via spdk_bdev_read and advances through parity disks
 * entirely from I/O completion callbacks -- the reactor thread is never held.
 *
 * Sequence for each parity disk:
 *   advance() -> submit read(copy 0) -> copy0_cb()
 *             -> submit read(copy 1) -> copy1_cb()
 *             -> merge_disk() -> advance to next disk -> ...
 *   when parity_idx == m: finish() -> done_fn(done_arg, rc)
 *
 * done_fn is always called. rc is 0 on success, including when individual
 * per-disk reads fail and are skipped (a missed merge means a missed scrub
 * at most -- non-fatal). rc is -ENOMEM if context or buffer allocation
 * fails synchronously below, before the chain starts.
 * ------------------------------------------------------------------------- */

static void ec_wib_load_async_continue(struct ec_wib_load_async_ctx *ctx);

static void
ec_wib_load_async_finish(struct ec_wib_load_async_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;
	ec_bdev_create_cb_fn done_fn = ctx->done_fn;
	void *done_arg = ctx->done_arg;
	bool any_valid = ctx->any_valid;

	spdk_dma_free(ctx->scratch);
	free(ctx);

	if (any_valid) {
		SPDK_NOTICELOG("EC bdev %s: WIB loaded -- %u/%u regions dirty\n",
			       ec->bdev.name, ec_wib_count_dirty(ec),
			       ec->wib_num_regions);
	} else {
		SPDK_NOTICELOG("EC bdev %s: no valid WIB found -- assuming clean\n",
			       ec->bdev.name);
	}

	done_fn(done_arg, 0);
}

/*
 * Merge the best valid copy for disk ctx->parity_idx into ec->wib_region_map.
 */
static void
ec_wib_load_async_merge_disk(struct ec_wib_load_async_ctx *ctx)
{
	struct ec_bdev *ec        = ctx->ec;
	uint32_t        map_words = EC_BITMAP_WORDS(ec->wib_num_regions);

	if (!ctx->valid_a && !ctx->valid_b) {
		SPDK_WARNLOG("EC bdev %s: no valid WIB copy on parity "
			     "disk %u -- treating as clean\n",
			     ec->bdev.name, ec->k + ctx->parity_idx);
		return;
	}

	{
		bool pick_b = ctx->valid_b &&
			      (!ctx->valid_a || ctx->gen_b >= ctx->gen_a);
		const void *best = pick_b ? ctx->bufb : ctx->bufa;
		const struct ec_wib_header *hdr = (const struct ec_wib_header *)best;
		const uint64_t             *bits = (const uint64_t *)(hdr + 1);
		uint64_t                    best_gen = pick_b ? ctx->gen_b : ctx->gen_a;
		uint32_t                    w;

		/*
		 * The OR-merge below bypasses ec_wib_region_set_dirty's
		 * release-store discipline. Safe by the same logic as
		 * ec_bitmap_apply_buf: this runs in the ec_wib_load_async
		 * chain during ec_bdev_create_async, before the create-RPC
		 * returns. The bdev is registered before the async load
		 * starts, but the only reader that can reach it in the
		 * register-to-load-finished window is SPDK examine -- which
		 * runs on the register thread (= home), so it serializes
		 * with the merge here on home. Workload (cross-reactor)
		 * readers cannot reach the bdev until the create-RPC
		 * returns, which waits for spdk_bdev_wait_for_examine after
		 * the load completes. By the time any non-home reader runs
		 * acquire-load on wib_region_map, this merge is published
		 * by the post-create release barriers SPDK inserts when the
		 * bdev becomes visible to consumers.
		 */
		for (w = 0; w < map_words; w++) {
			ec->wib_region_map[w] |= bits[w];
		}

		if (best_gen > ec->wib_generation) {
			ec->wib_generation  = best_gen;
			ec->wib_active_copy = pick_b ? 1 : 0;
		}
		ctx->any_valid = true;
	}
}

static void
ec_wib_load_async_copy1_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_wib_load_async_ctx *ctx = cb_arg;
	struct ec_bdev               *ec  = ctx->ec;

	spdk_bdev_free_io(bdev_io);

	if (success && ec_wib_validate_buf(ec, ctx->bufb, &ctx->gen_b) == 0) {
		ctx->valid_b = true;
	}

	ec_wib_load_async_merge_disk(ctx);

	ctx->parity_idx++;
	ec_wib_load_async_continue(ctx);
}

static void
ec_wib_load_async_copy0_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_wib_load_async_ctx *ctx   = cb_arg;
	struct ec_bdev               *ec    = ctx->ec;
	uint32_t                      pslot = ec->k + ctx->parity_idx;
	int                           rc;

	spdk_bdev_free_io(bdev_io);

	if (success && ec_wib_validate_buf(ec, ctx->bufa, &ctx->gen_a) == 0) {
		ctx->valid_a = true;
	}

	/* Read copy 1 -- failure is non-fatal, merge proceeds with copy 0 only */
	rc = spdk_bdev_read(ec->descs[pslot],
			    ec->wib_chans[ctx->parity_idx],
			    ctx->bufb,
			    ec_wib_lba(ec, 1) * ec->bdev.blocklen,
			    ctx->buf_bytes,
			    ec_wib_load_async_copy1_cb,
			    ctx);
	if (rc != 0) {
		SPDK_WARNLOG("EC bdev %s: failed to submit WIB copy 1 read "
			     "for parity disk %u (rc=%d) -- using copy 0 only\n",
			     ec->bdev.name, pslot, rc);
		ec_wib_load_async_merge_disk(ctx);
		ctx->parity_idx++;
		ec_wib_load_async_continue(ctx);
	}
}

/*
 * Find the next available parity disk and submit a copy-0 read.
 * Called at startup and from copy1_cb to advance to the next disk.
 * When all disks are processed, calls ec_wib_load_async_finish().
 */
static void
ec_wib_load_async_continue(struct ec_wib_load_async_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;

	while (ctx->parity_idx < ec->m) {
		uint32_t pslot = ec->k + ctx->parity_idx;
		int      rc;

		if (!ec->descs[pslot] || !ec->wib_chans[ctx->parity_idx]) {
			ctx->parity_idx++;
			continue;
		}

		/* Reset per-disk state */
		ctx->gen_a   = 0;
		ctx->gen_b   = 0;
		ctx->valid_a = false;
		ctx->valid_b = false;

		rc = spdk_bdev_read(ec->descs[pslot],
				    ec->wib_chans[ctx->parity_idx],
				    ctx->bufa,
				    ec_wib_lba(ec, 0) * ec->bdev.blocklen,
				    ctx->buf_bytes,
				    ec_wib_load_async_copy0_cb,
				    ctx);
		if (rc != 0) {
			SPDK_WARNLOG("EC bdev %s: failed to submit WIB copy 0 read "
				     "for parity disk %u (rc=%d) -- skipping disk\n",
				     ec->bdev.name, pslot, rc);
			ctx->parity_idx++;
			continue;
		}

		return;  /* I/O submitted; callback drives the next step */
	}

	/* All parity disks processed */
	ec_wib_load_async_finish(ctx);
}

void
ec_wib_load_async(struct ec_bdev *ec, ec_bdev_create_cb_fn done_fn, void *done_arg)
{
	struct ec_wib_load_async_ctx *ctx;
	size_t  buf_bytes = (size_t)ec->strip_size * ec->bdev.blocklen;
	void   *scratch;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		SPDK_ERRLOG("EC bdev %s: OOM for async WIB load ctx\n",
			    ec->bdev.name);
		done_fn(done_arg, -ENOMEM);
		return;
	}

	scratch = spdk_dma_zmalloc(2 * buf_bytes, EC_DMA_ALIGN, NULL);
	if (!scratch) {
		SPDK_ERRLOG("EC bdev %s: OOM for WIB load scratch buffer\n",
			    ec->bdev.name);
		free(ctx);
		done_fn(done_arg, -ENOMEM);
		return;
	}

	ctx->ec         = ec;
	ctx->scratch    = scratch;
	ctx->buf_bytes  = buf_bytes;
	ctx->bufa       = scratch;
	ctx->bufb       = (uint8_t *)scratch + buf_bytes;
	ctx->parity_idx = 0;
	ctx->any_valid  = false;
	ctx->done_fn    = done_fn;
	ctx->done_arg   = done_arg;

	ec_wib_load_async_continue(ctx);
}

/*
 * Returns -ENODEV if the named EC bdev does not exist.
 * Returns 0 on success.
 *
 * dirty_regions is the count of region bits currently set in wib_region_map.
 */
int
ec_bdev_get_wib_status(const char *ec_name,
		       uint32_t   *num_regions,
		       uint32_t   *dirty_regions,
		       uint64_t   *generation,
		       bool       *persist_pending)
{
	struct ec_bdev *ec = ec_bdev_find(ec_name);

	if (!ec) {
		return -ENODEV;
	}

	*num_regions     = ec->wib_num_regions;
	*dirty_regions   = ec_wib_count_dirty(ec);
	*generation      = ec->wib_generation;
	*persist_pending = ec->wib_persist_in_flight;

	return 0;
}
