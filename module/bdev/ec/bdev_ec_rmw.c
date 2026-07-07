/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_rmw.c -- sub-stripe writes via Read-Modify-Write.
 *
 * A sub-stripe write can't compute parity from its payload alone. RMW
 * reads the surviving data chunks, overlays the payload, re-encodes
 * parity, then writes back. Each function's header documents its place
 * in the async chain; the WIB persist that gates the writes lives in
 * bdev_ec_wib.c. The persist runs BEFORE the reads (ordering enforced
 * in ec_rmw_persist_and_dispatch) so that a crash mid-RMW finds the
 * WIB bit on disk and a scrub recomputes parity at recovery.
 */

#include "bdev_ec_internal.h"

#include "spdk/bdev_module.h"
#include "spdk/string.h"
#include "spdk/thread.h"
#include "spdk/util.h"

#include <isa-l/erasure_code.h>

/* Concurrent RMW to the same stripe is serialised via the stripe dirty
 * bit: if a second RMW lands while the first is in flight,
 * ec_submit_rmw_write returns -EAGAIN, which the dispatcher maps to
 * SPDK_BDEV_IO_STATUS_NOMEM so SPDK requeues without an explicit timer. */

/*
 * Free all DMA buffers held by an ec_rmw_ctx and then free the context itself.
 * Must be called after the dirty bit and rmw_in_flight counter have already
 * been updated (or were never set, in the allocation-failure path).
 */
static void
ec_rmw_free_ctx(struct ec_rmw_ctx *mctx, const struct ec_bdev *ec)
{
	uint32_t i;

	for (i = 0; i < ec->n; i++) {
		if (mctx->chunk_bufs[i]) {
			spdk_dma_free(mctx->chunk_bufs[i]);
			mctx->chunk_bufs[i] = NULL;
		}
	}

	free(mctx);
}

/*
 * Release the stripe-busy claim, decrement rmw_in_flight and the WIB
 * region inflight count, then free mctx (DMA buffers via
 * ec_rmw_free_ctx). mctx is invalid after return.
 */
static void
ec_rmw_teardown(struct ec_rmw_ctx *mctx)
{
	struct ec_bdev *ec     = ec_from_bdev_io(mctx->ec_io->bdev_io);
	uint32_t        region = ec_wib_stripe_to_region(mctx->stripe_index);

	ec_stripe_clear_dirty(ec, mctx->stripe_index);
	ec_rmw_in_flight_dec(ec);
	ec_wib_region_inflight_dec(ec, region);
	ec_rmw_free_ctx(mctx, ec);
}

/*
 * Terminal step of the RMW chain. Called once all write children have
 * completed (writes_remaining == 0).
 *
 * Clears the stripe dirty bit, decrements rmw_in_flight, decrements the
 * WIB region in-flight counter (the region clear is handled by the idle
 * poller), frees the context, and completes the parent bdev_io.
 */
static void ec_rmw_complete_on_submitter(void *ctx);

void
ec_rmw_complete(struct ec_rmw_ctx *mctx)
{
	struct ec_bdev_io *ec_io = mctx->ec_io;
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	enum spdk_bdev_io_status status;
	void (*cb_fn)(void *, enum spdk_bdev_io_status);
	void *cb_arg;

	/*
	 * Route to the submitter (= bdev_io owner) thread so the eventual
	 * spdk_bdev_io_complete (or cb_fn that walks to one) runs on the
	 * thread SPDK requires. ec_rmw_complete is reached from:
	 *
	 *   - ec_rmw_write_cb (writes' final completion) -- on submitter
	 *     (writes dispatched on submitter's base_chans complete there);
	 *     the routing inlines.
	 *   - ec_rmw_wib_set_cb persist-failure -- on home;
	 *     hops to submitter.
	 *   - ec_rmw_persist_and_dispatch's persist-alloc-failure -- on home;
	 *     hops to submitter.
	 *   - ec_wib_deferred_drain's failure path -- on home; hops.
	 *
	 * Routing the WHOLE function (cleanup + completion) is safe
	 * because the cleanup ops are atomic: ec_stripe_clear_dirty,
	 * ec_rmw_in_flight_dec, ec_wib_region_inflight_dec all use
	 * relaxed atomics and work from any thread.
	 *
	 * On send_msg failure, perform the cleanup inline here (so mctx
	 * is not leaked) and accept the bdev_io hang. The on-disk WIB
	 * bit is set, so a crash here is scrub-recoverable.
	 */
	if (spdk_unlikely(spdk_get_thread() != ec_io->submitter_thread)) {
		int send_rc = spdk_thread_send_msg(ec_io->submitter_thread,
			ec_rmw_complete_on_submitter, mctx);
		if (send_rc != 0) {
			SPDK_ERRLOG("EC bdev %s: cannot hand off RMW "
				    "completion to submitter thread '%s' (rc=%d %s) "
				    "at stripe %" PRIu64 "; bdev_io stays "
				    "in-flight\n",
				    ec->bdev.name,
				    spdk_thread_get_name(ec_io->submitter_thread),
				    send_rc, spdk_strerror(-send_rc), mctx->stripe_index);
			ec_rmw_teardown(mctx);
		}
		return;
	}

	status = mctx->status;
	cb_fn  = mctx->cb_fn;
	cb_arg = mctx->cb_arg;

	ec_rmw_teardown(mctx);

	/*
	 * cb_fn == NULL means complete the parent bdev_io directly (ordinary
	 * WRITE path). Non-NULL is the multi-segment UNMAP coordinator: it only
	 * completes the parent once every segment has reported back.
	 */
	if (cb_fn != NULL) {
		cb_fn(cb_arg, status);
	} else {
		spdk_bdev_io_complete(ec_io->bdev_io, status);
	}
}

/*
 * Called for each write I/O submitted during the RMW write phase.
 * Decrements writes_remaining; when zero calls ec_rmw_complete.
 */
static void
ec_rmw_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_rmw_ctx *mctx = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	mctx->writes_remaining--;

	if (mctx->writes_remaining == 0) {
		ec_rmw_complete(mctx);
	}
}

/* Forward decls. ec_rmw_dispatch_reads is also exposed in
 * bdev_ec_internal.h: ec_wib_deferred_drain calls it because a deferred
 * RMW resumes at the read-dispatch step (its DMA bufs are still zeroed,
 * and the persist-before-reads ordering means a write fan-out would
 * corrupt the stripe). */
static void ec_rmw_persist_and_dispatch(struct ec_rmw_ctx *mctx, bool was_clean);
static void ec_rmw_wib_set_cb(void *cb_arg, int rc);
static void ec_rmw_submit_writes(struct ec_rmw_ctx *mctx);

/*
 * send_msg target for routing the bdev_io completion to the submitter
 * thread on the unhappy path where the post-persist write dispatch
 * cannot be handed off to the submitter (a second-stage send_msg
 * failure: the on-disk WIB is set, so a crash here is scrub-recoverable).
 */
static void
ec_rmw_complete_on_submitter(void *ctx)
{
	ec_rmw_complete((struct ec_rmw_ctx *)ctx);
}

/*
 * send_msg target for routing the read dispatch to the submitter
 * thread. Re-enters ec_rmw_dispatch_reads; on the submitter thread
 * the routing check at the top inlines and runs the dispatch body.
 */
static void
ec_rmw_dispatch_reads_on_submitter(void *ctx)
{
	ec_rmw_dispatch_reads((struct ec_rmw_ctx *)ctx);
}

/*
 * send_msg target for routing a FAILED bdev_io completion to the
 * submitter when the read-dispatch hand-off itself fails AND the
 * mctx has already been freed by the time we discover the failure.
 * Takes ec_bdev_io (which lives inside bdev_io->driver_ctx and
 * therefore outlives mctx).
 */
static void
ec_rmw_complete_bdev_io_failed_on_submitter(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;

	spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
}

/*
 * Called when the WIB persist (marking the region dirty on disk)
 * completes. The persist runs BEFORE reads, so on success this resumes
 * by dispatching the reads -- not by submitting writes (the RMW has
 * not read anything yet and its chunk_bufs are zeroed DMA).
 *
 * On persist failure, the on-disk WIB bit is not durable. Resuming
 * reads/writes anyway would risk silent corruption: stale parity
 * could land with no on-disk dirty bit to trigger the startup scrub.
 * Instead we clear the in-memory dirty bit (so the next RMW for this
 * region re-persists rather than taking the skip-persist fast path)
 * and complete with NOMEM so SPDK requeues. ec_rmw_complete unwinds
 * the stripe-busy claim, the in-flight counters, and the DMA buffers.
 */
static void
ec_rmw_wib_set_cb(void *cb_arg, int rc)
{
	struct ec_rmw_ctx *mctx = cb_arg;
	struct ec_bdev    *ec   = ec_from_bdev_io(mctx->ec_io->bdev_io);

	if (rc != 0) {
		uint32_t region = ec_wib_stripe_to_region(mctx->stripe_index);

		/* Soft failure: ec_rmw_complete returns NOMEM below so SPDK
		 * requeues the bdev_io. WARNLOG (not ERRLOG) so monitoring
		 * does not alert on a retryable error. */
		SPDK_WARNLOG("EC bdev %s: WIB set persist failed (rc=%d) "
			     "before RMW read dispatch at stripe %" PRIu64 "; "
			     "requeueing to preserve write-intent ordering\n",
			     ec->bdev.name, rc, mctx->stripe_index);

		ec_wib_region_clear_dirty(ec, region);
		mctx->status = SPDK_BDEV_IO_STATUS_NOMEM;
		ec_rmw_complete(mctx);
		return;
	}

	ec_rmw_dispatch_reads(mctx);
}

/*
 * Called once all read children for this RMW have completed.
 *
 * Steps:
 *   1. If degraded: for each unreadable DATA slot, reconstruct its chunk
 *      from the k surviving readable chunks already in chunk_bufs.
 *   2. Flatten chunk_bufs[0..k-1] into a scratch buffer, apply the write
 *      payload at the correct offset, then redistribute back.
 *   3. Re-encode all m parity chunks from chunk_bufs[0..k-1].
 *   3b. Ensure the WIB region bit is set on disk before writing.
 *   4. Write modified data chunk + all writable parity slots back to disk.
 */
static void
ec_rmw_reads_done(struct ec_rmw_ctx *mctx)
{
	struct ec_bdev_io *ec_io      = mctx->ec_io;
	struct ec_bdev    *ec         = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t           chunk_bytes = ec->strip_size * ec->bdev.blocklen;
	uint64_t           total_data_bytes = ec->stripe_blocks * ec->bdev.blocklen;
	uint32_t           i;
	int                rc;

	/* Step 1: reconstruct any unreadable data chunks; ec_reconstruct_multi_data handles up to m failed data slots in one pass. */
	if (ec->failed_count > 0) {
		uint32_t failed_data_slots[EC_MAX_BASE_BDEVS];
		uint8_t *out_bufs[EC_MAX_BASE_BDEVS];
		uint32_t num_failed = 0;

		/* Collect all unreadable DATA slots */
		for (i = 0; i < ec->k; i++) {
			if (!ec_slot_is_readable(ec, i)) {
				failed_data_slots[num_failed] = i;
				out_bufs[num_failed]          = mctx->chunk_bufs[i];
				num_failed++;
			}
		}

		if (num_failed > 0) {
			/*
			 * Reconstruct all failed data slots in one pass:
			 * one gf_invert_matrix call, one ec_encode_data call.
			 * num_failed <= ec->m is guaranteed by the offline guard in
			 * ec_handle_base_bdev_failure (offline when failed_count > m).
			 */
			rc = ec_reconstruct_multi_data(ec,
						       mctx->chunk_bufs,
						       out_bufs,
						       failed_data_slots,
						       num_failed,
						       chunk_bytes);
			if (rc != 0) {
				SPDK_ERRLOG("EC bdev %s: RMW multi-reconstruction "
					    "failed (%u slots) stripe %" PRIu64 "\n",
					    ec->bdev.name, num_failed,
					    mctx->stripe_index);
				mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
				ec_rmw_complete(mctx);
				return;
			}
		}
	}

	/* ------------------------------------------------------------------ */
	/* Step 2: overlay the write payload                                  */
	/*                                                                    */
	/* Reassemble the k data chunks into a contiguous scratch buffer,     */
	/* apply the write payload at the correct byte offset, then           */
	/* redistribute back into chunk_bufs[0..k-1].                        */
	/*                                                                    */
	/* scratch is a regular heap allocation (not DMA) because it is only  */
	/* used for CPU-side data manipulation, not for I/O.                  */
	/* ------------------------------------------------------------------ */
	{
		uint8_t *scratch;
		uint64_t stripe_off_bytes = mctx->stripe_off_blocks *
					    ec->bdev.blocklen;
		uint64_t payload_bytes    = mctx->num_blocks *
					    ec->bdev.blocklen;

		scratch = malloc(total_data_bytes);
		if (!scratch) {
			SPDK_ERRLOG("EC bdev %s: OOM for RMW scratch buffer "
				    "(%" PRIu64 " bytes, stripe %" PRIu64 ")\n",
				    ec->bdev.name, total_data_bytes,
				    mctx->stripe_index);
			mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
			ec_rmw_complete(mctx);
			return;
		}

		/* Reassemble: pack chunk_bufs[0..k-1] into scratch linearly */
		for (i = 0; i < ec->k; i++) {
			memcpy(scratch + (i * chunk_bytes),
			       mctx->chunk_bufs[i],
			       chunk_bytes);
		}

		/*
		 * Overlay the write payload at stripe_off_bytes.
		 *
		 * WRITE_ZEROES, UNMAP-emulated-as-WRITE_ZEROES, and the
		 * partial-stripe head/tail segments of an unaligned multi-stripe
		 * UNMAP (ec_submit_rmw_zero_fill_range) all carry no user
		 * iov; the payload is conceptually all zeros. Use memset to
		 * fill the modified region directly. The parity re-encode
		 * below then computes the correct parity for "old data with
		 * a zero hole" -- exactly the deallocated-reads-as-zero
		 * semantics expected by callers.
		 */
		if (mctx->is_zero_fill) {
			memset(scratch + stripe_off_bytes, 0, payload_bytes);
		} else {
			spdk_copy_iovs_to_buf(scratch + stripe_off_bytes,
					      payload_bytes,
					      ec_io->iovs,
					      ec_io->iovcnt);
		}

		/* Redistribute back into chunk_bufs[0..k-1] */
		for (i = 0; i < ec->k; i++) {
			memcpy(mctx->chunk_bufs[i],
			       scratch + (i * chunk_bytes),
			       chunk_bytes);
		}

		free(scratch);
	}

	/* ------------------------------------------------------------------ */
	/* Step 3: re-encode all m parity chunks                              */
	/*                                                                    */
	/* data_ptrs[0..k-1]  -> chunk_bufs[0..k-1]   (modified data)        */
	/* parity_ptrs[0..m-1] -> chunk_bufs[k..n-1]  (parity output)        */
	/* ------------------------------------------------------------------ */
	{
		uint8_t *data_ptrs[EC_MAX_BASE_BDEVS];
		uint8_t *parity_ptrs[EC_MAX_BASE_BDEVS];

		for (i = 0; i < ec->k; i++) {
			data_ptrs[i] = mctx->chunk_bufs[i];
		}
		for (i = 0; i < ec->m; i++) {
			parity_ptrs[i] = mctx->chunk_bufs[ec->k + i];
		}

		ec_encode_data((int)chunk_bytes,
			       (int)ec->k,
			       (int)ec->m,
			       ec->g_tbls,
			       data_ptrs,
			       parity_ptrs);
	}

	/*
	 * The WIB persist already happened in the setup phase (before reads),
	 * so all that's left is dispatching the writes. The single-writer-on-
	 * home invariant for stripe_dirty_map, wib_region_inflight, etc. is
	 * preserved because the persist decision lived in setup -- we never
	 * reach here without those state mutations having been performed on
	 * home. The reads_done -> writes transition stays on submitter (no
	 * home thread state mutation between here and submit_writes).
	 */
	ec_rmw_submit_writes(mctx);
}

/*
 * WIB persist decision + dispatch hand-off. Runs on the home thread
 * (called from ec_rmw_submit_core's setup tail). The persist must
 * land on disk BEFORE the read fan-out (and therefore before any
 * write), so a crash mid-RMW always finds the WIB bit set on disk
 * and a startup scrub re-encodes parity for that stripe.
 *
 * was_clean reflects whether the in-memory WIB region bit was clear
 * before the setup-phase set_dirty call (captured BEFORE set_dirty
 * so the dirty_ticks recording in setup is accurate). It drives the
 * persist decision:
 *
 *   was_clean = false  (region already dirty in memory pre-setup)
 *     The bit may or may not be on disk yet. If wib_persist_in_flight,
 *     a persist is in flight whose buffer snapshot might or might
 *     not include this bit; defer this RMW on wib_deferred_writes
 *     until durability is confirmed. Otherwise the bit is durable
 *     and we proceed directly to ec_rmw_dispatch_reads.
 *
 *   was_clean = true  (setup just set the bit)
 *     If wib_persist_in_flight, a persist is in flight whose buffer
 *     snapshot predates the bit; defer and set wib_repersist_needed
 *     so the follow-up persist puts the new bit on disk. Otherwise
 *     start a persist (ec_rmw_wib_set_cb -> dispatch_reads).
 *
 * Two deferral paths both queue on wib_deferred_writes;
 * ec_wib_deferred_drain calls ec_rmw_dispatch_reads on each entry
 * once the in-flight persist (or its follow-up) completes.
 *
 * wib_region_inflight[region] was already incremented in setup, so
 * every exit path through ec_rmw_complete has a matching decrement
 * regardless of which branch fires here.
 */
static void
ec_rmw_persist_and_dispatch(struct ec_rmw_ctx *mctx, bool was_clean)
{
	struct ec_bdev *ec     = ec_from_bdev_io(mctx->ec_io->bdev_io);
	uint32_t        region = ec_wib_stripe_to_region(mctx->stripe_index);
	int             persist_rc;

	/* Home-thread invariant: ec_wib_persist uses ec->wib_chans[]. */
	assert(spdk_get_thread() == ec->home_thread);

	if (!was_clean) {
		if (ec->wib_persist_in_flight) {
			/* Bit was set pre-setup but may not be on disk
			 * (set before or after the in-flight persist's
			 * snapshot). Defer reads until durability is
			 * confirmed; ec_wib_deferred_drain dispatches
			 * reads when the persist completes. */
			TAILQ_INSERT_TAIL(&ec->wib_deferred_writes, mctx,
					  wib_defer_link);
			return;
		}
		/* No persist in flight: the pre-setup bit is durable. */
		ec_rmw_dispatch_reads(mctx);
		return;
	}

	/* Setup just set the in-memory bit (this is the first RMW into
	 * this region in the current dirty window). */
	if (ec->wib_persist_in_flight) {
		/* The in-flight persist's buffer snapshot predates the
		 * bit setup just wrote. Defer and request a follow-up
		 * persist; ec_wib_deferred_drain dispatches reads once
		 * the bit is durable. */
		ec->wib_repersist_needed = true;
		TAILQ_INSERT_TAIL(&ec->wib_deferred_writes, mctx,
				  wib_defer_link);
		return;
	}

	persist_rc = ec_wib_persist(ec, ec_rmw_wib_set_cb, mctx);
	if (persist_rc != 0) {
		/* Fail the I/O rather than risk data/parity inconsistency
		 * on crash. */
		SPDK_ERRLOG("EC bdev %s: WIB persist alloc failed (rc=%d); "
			    "failing I/O\n", ec->bdev.name, persist_rc);
		/*
		 * Roll back the region bit setup wrote: no persist recorded it
		 * and ec_rmw_complete does not clear it, so leaving it set would
		 * let a retry skip the WIB persist and write with no on-disk
		 * intent. No reads/writes have been issued, so nothing needs
		 * scrubbing.
		 */
		ec_wib_region_clear_dirty(ec, region);
		mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		ec_rmw_complete(mctx);
	}
	/* else: ec_rmw_wib_set_cb will call ec_rmw_dispatch_reads. */
}

/*
 * Submit the data and parity writes for a completed RMW. Sole caller is
 * ec_rmw_reads_done; the WIB persist already ran in the setup phase, so by
 * the time the reads complete the region bit is on disk and the writes can
 * fan out unconditionally.
 *
 * Only the modified data chunks in [modified_chunk_first, modified_chunk_last]
 * are written back. For ordinary WRITE this is always a single chunk
 * (enforced by ec_submit_rmw_write's canary). For WRITE_ZEROES the range
 * can span multiple chunks within one stripe; the overlay step has already
 * filled the entire range with zeros.
 *
 * All m writable parity chunks are always written (recomputed from all k
 * data chunks).
 */
static void
ec_rmw_submit_writes(struct ec_rmw_ctx *mctx)
{
	struct ec_bdev_io *ec_io = mctx->ec_io;
	struct ec_bdev    *ec    = ec_from_bdev_io(ec_io->bdev_io);
	uint32_t           i;
	int                rc;
	uint32_t           writable_count = 0;

	/*
	 * Dispatch invariant: the sole caller (ec_rmw_reads_done) runs on
	 * the submitter thread because base-bdev read completions land on
	 * the channel's owning thread. ec_io->ch->base_chans[] are owned
	 * here, so dispatch can run inline with no routing hop.
	 */
	assert(spdk_get_thread() == ec_io->submitter_thread);

	/*
	 * Count writable slots: every modified data chunk in the range plus
	 * every writable parity chunk. Data chunks outside the modified
	 * range are not written back (unchanged on disk).
	 */
	for (i = mctx->modified_chunk_first; i <= mctx->modified_chunk_last; i++) {
		if (ec_slot_is_writable(ec, i)) {
			writable_count++;
		}
	}
	for (i = 0; i < ec->m; i++) {
		if (ec_slot_is_writable(ec, ec->k + i)) {
			writable_count++;
		}
	}

	if (writable_count == 0) {
		SPDK_ERRLOG("EC bdev %s: RMW stripe %" PRIu64 " -- no writable "
			    "slots\n", ec->bdev.name, mctx->stripe_index);
		mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		ec_rmw_complete(mctx);
		return;
	}

	mctx->writes_remaining = writable_count;

	/*
	 * Write every modified data chunk in the range. For a single-chunk
	 * RMW (the common case for ordinary WRITE) the loop iterates once;
	 * for a multi-chunk WRITE_ZEROES that straddles a strip boundary it
	 * iterates over the affected chunks. All writes target the same
	 * disk_lba on their respective base bdev -- they share one stripe.
	 */
	for (i = mctx->modified_chunk_first; i <= mctx->modified_chunk_last; i++) {
		if (!ec_slot_is_writable(ec, i)) {
			continue;
		}
		rc = spdk_bdev_writev_blocks(
				ec->descs[i],
				ec_io->ch->base_chans[i],
				&mctx->chunk_iovs[i], 1,
				mctx->disk_lba,
				ec->strip_size,
				ec_rmw_write_cb,
				mctx);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: RMW write submit failed "
				    "for data slot %u stripe %" PRIu64 " (rc=%d)\n",
				    ec->bdev.name, i,
				    mctx->stripe_index, rc);
			mctx->writes_remaining--;
			mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}

	/* Write parity slots */
	for (i = 0; i < ec->m; i++) {
		uint32_t slot = ec->k + i;

		if (!ec_slot_is_writable(ec, slot)) {
			continue;
		}
		rc = spdk_bdev_writev_blocks(ec->descs[slot],
					     ec_io->ch->base_chans[slot],
					     &mctx->chunk_iovs[slot], 1,
					     mctx->disk_lba,
					     ec->strip_size,
					     ec_rmw_write_cb,
					     mctx);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: RMW write submit failed "
				    "for parity slot %u stripe %" PRIu64 " "
				    "(rc=%d)\n",
				    ec->bdev.name, slot,
				    mctx->stripe_index, rc);
			mctx->writes_remaining--;
			mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}

	/*
	 * If all write submits failed synchronously, writes_remaining
	 * reached zero inside the loop. No callbacks will fire, so
	 * complete now.
	 */
	if (mctx->writes_remaining == 0) {
		ec_rmw_complete(mctx);
	}
}

/*
 * Called for each read child submitted during the RMW read phase.
 * Decrements reads_remaining; when zero calls ec_rmw_reads_done (on success)
 * or ec_rmw_complete (on accumulated failure).
 */
static void
ec_rmw_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_rmw_ctx *mctx = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	mctx->reads_remaining--;

	if (mctx->reads_remaining == 0) {
		if (mctx->status != SPDK_BDEV_IO_STATUS_SUCCESS) {
			SPDK_ERRLOG("EC bdev %s: RMW read failed for stripe %" PRIu64 "\n",
				    mctx->ec_io->bdev_io->bdev->name,
				    mctx->stripe_index);
			ec_rmw_complete(mctx);
			return;
		}
		ec_rmw_reads_done(mctx);
	}
}

/*
 * Mark RMW backpressure as active. First call after a quiet period
 * logs a NOTICE; later calls only refresh counter state. Callers
 * pass the reason so the log line names the cause (active scrub vs.
 * deferred-scrub guard).
 */
static void
ec_rmw_backpressure_begin(struct ec_bdev *ec, const char *reason)
{
	if (ec->rmw_backpressure_active) {
		return;
	}
	ec->rmw_backpressure_active           = true;
	ec->rmw_backpressure_since_ticks      = spdk_get_ticks();
	ec->rmw_backpressure_count_at_start   = ec->rmw_deferred_scrub +
					        ec->rmw_deferred_dirty;
	SPDK_NOTICELOG("EC bdev %s: RMW backpressure begun -- %s\n",
		       ec->bdev.name, reason);
}

void
ec_rmw_backpressure_end(struct ec_bdev *ec, const char *reason)
{
	uint64_t now;
	uint64_t ticks_per_second;
	uint64_t deferred;
	uint64_t elapsed_milliseconds;

	if (!ec->rmw_backpressure_active) {
		return;
	}

	now = spdk_get_ticks();
	ticks_per_second = spdk_get_ticks_hz();
	deferred = (ec->rmw_deferred_scrub + ec->rmw_deferred_dirty) -
		   ec->rmw_backpressure_count_at_start;
	elapsed_milliseconds = (ticks_per_second > 0) ?
		     ((now - ec->rmw_backpressure_since_ticks) * 1000 / ticks_per_second) : 0;

	SPDK_NOTICELOG("EC bdev %s: RMW backpressure cleared (%s) -- "
		       "%" PRIu64 " stripes deferred over %" PRIu64 "ms\n",
		       ec->bdev.name, reason, deferred, elapsed_milliseconds);

	ec->rmw_backpressure_active = false;
}

/*
 * Decide whether an RMW write to stripe_index may proceed right now.
 * Three guards apply:
 *
 *   1. Active-scrub guard: a startup scrubber in this region (or any
 *      region ahead of it that's still dirty) can race the RMW into
 *      the write-hole. See the long comment inline for the race
 *      analysis.
 *
 *   2. Deferred-scrub guard: no scrub is running and the region is
 *      crash-dirty. A degraded RMW would reconstruct from stale parity.
 *
 *   3. Per-stripe dirty guard: a prior RMW to the same stripe is
 *      still in flight; serialise to prevent corrupting its in-memory
 *      data.
 *
 * Returns 0 if the RMW may proceed. Returns -EAGAIN if any guard
 * fires (ec_submit_request maps this to SPDK_BDEV_IO_STATUS_NOMEM so
 * SPDK requeues automatically).
 */
static int
ec_rmw_check_guards(struct ec_bdev *ec, uint64_t stripe_index)
{
	uint32_t region = ec_wib_stripe_to_region(stripe_index);

	/*
	 * Active-scrub guard: if scrub_ctx exists AND stripe_index is in
	 * the region currently being scrubbed AND the stripe has not yet
	 * been passed by the scrubber, requeue. Stripes already passed
	 * are safe. Regions ahead of current_region that are still dirty
	 * are also blocked.
	 *
	 * The race we are preventing: the scrubber reads old data, the
	 * RMW writes new data + new parity, then the scrubber writes
	 * parity computed from the old data -- overwriting the RMW's
	 * correct parity with stale parity. Result: new data, old parity
	 * -- exactly the write-hole the WIB exists to prevent.
	 */
	if (ec_scrub_blocks_stripe(ec, stripe_index)) {
		ec->rmw_deferred_scrub++;
		ec_rmw_backpressure_begin(ec, "scrub active");
		return -EAGAIN;
	}

	/*
	 * Deferred-scrub guard: no scrub is running (deferred behind a failed
	 * disk, or not yet started) but the region is crash-dirty. A degraded
	 * RMW would reconstruct the missing chunk from stale parity and persist
	 * wrong parity, so defer until the scrub re-encodes it. Write-intent
	 * alone does not block here -- its parity is already consistent.
	 */
	if (ec->scrub_ctx == NULL && ec->failed_count > 0 &&
	    ec_wib_crash_is_dirty(ec, region)) {
		ec->rmw_deferred_dirty++;
		ec_rmw_backpressure_begin(ec, "deferred scrub (degraded)");
		return -EAGAIN;
	}

	/* Per-stripe serialisation: prior RMW to same stripe still in flight. */
	if (ec_stripe_is_dirty(ec, stripe_index)) {
		ec->rmw_deferred_inflight++;
		return -EAGAIN;
	}

	return 0;
}

/*
 * Submitter-side resume of the RMW chain: dispatches reads on
 * ec_io->ch->base_chans[] (submitter-owned). Reached from three home-thread
 * call sites in the post-persist resume:
 *
 *   - ec_rmw_persist_and_dispatch (inline, when the region bit was already durable).
 *   - ec_rmw_wib_set_cb (after a successful WIB persist).
 *   - ec_wib_deferred_drain (after a deferred persist completes).
 *
 * When the caller is already on the submitter the routing check inlines.
 *
 * Takes ownership of mctx on the all-reads-failed path: undoes the home-side
 * bookkeeping using relaxed atomics, completes the bdev_io FAILED, frees the
 * ctx. The in-memory WIB region bit is intentionally left set on this path;
 * the on-disk bit is durable and the idle WIB poller is the clean owner.
 */
void
ec_rmw_dispatch_reads(struct ec_rmw_ctx *mctx)
{
	struct ec_bdev_io *ec_io           = mctx->ec_io;
	struct ec_bdev    *ec              = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t           stripe_index    = mctx->stripe_index;
	uint32_t           reads_submitted = 0;
	uint32_t           disk;
	int                rc;

	/* Route the read dispatch to the submitter thread; see the function header for the three home-side callers. The inline path runs when the caller is already on submitter. */
	if (spdk_unlikely(spdk_get_thread() != ec_io->submitter_thread)) {
		int send_rc = spdk_thread_send_msg(ec_io->submitter_thread,
			ec_rmw_dispatch_reads_on_submitter, mctx);
		if (send_rc != 0) {
			/*
			 * Persist already completed; we cannot reach the
			 * submitter for the read dispatch. The on-disk WIB
			 * bit is set, so a crash here is scrub-recoverable.
			 * Tear down the home-side bookkeeping (claim,
			 * counters) inline -- the atomic helpers are safe
			 * from home -- then complete the bdev_io with
			 * FAILED via a second send_msg
			 * (spdk_bdev_io_complete asserts owner thread).
			 * Capture ec_io before freeing mctx; ec_io lives in
			 * bdev_io->driver_ctx so it outlives mctx. If this
			 * second send_msg also fails the bdev_io stays in
			 * flight; on-disk WIB is set so a crash here is
			 * scrub-recoverable.
			 */
			SPDK_ERRLOG("EC bdev %s: cannot hand off RMW reads "
				    "to submitter thread '%s' (rc=%d %s) at stripe "
				    "%" PRIu64 "; failing bdev_io\n",
				    ec->bdev.name,
				    spdk_thread_get_name(ec_io->submitter_thread),
				    send_rc, spdk_strerror(-send_rc), stripe_index);
			ec_rmw_teardown(mctx);

			int complete_rc = spdk_thread_send_msg(
				ec_io->submitter_thread,
				ec_rmw_complete_bdev_io_failed_on_submitter,
				ec_io);
			if (complete_rc != 0) {
				SPDK_ERRLOG("EC bdev %s: also cannot hand "
					    "off failure completion to submitter "
					    "thread '%s' (rc=%d %s); bdev_io stays "
					    "in-flight\n",
					    ec->bdev.name,
					    spdk_thread_get_name(ec_io->submitter_thread),
					    complete_rc, spdk_strerror(-complete_rc));
			}
		}
		return;
	}

	/*
	 * Dispatch invariant: post-routing, we run on the submitter
	 * thread. ec_io->ch->base_chans[] are owned here.
	 */
	assert(spdk_get_thread() == ec_io->submitter_thread);
	mctx->reads_remaining = 0;

	for (disk = 0; disk < ec->n; disk++) {
		if (!ec_slot_is_readable(ec, disk)) {
			continue;
		}
		if (reads_submitted >= ec->k) {
			break;
		}

		mctx->reads_remaining++;

		rc = spdk_bdev_readv_blocks(ec->descs[disk],
					    ec_io->ch->base_chans[disk],
					    &mctx->chunk_iovs[disk], 1,
					    mctx->disk_lba,
					    ec->strip_size,
					    ec_rmw_read_cb,
					    mctx);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: RMW read submit failed for "
				    "disk %u stripe %" PRIu64 " (rc=%d)\n",
				    ec->bdev.name, disk, stripe_index, rc);
			mctx->reads_remaining--;
			mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
		} else {
			reads_submitted++;
		}
	}

	if (reads_submitted < ec->k) {
		SPDK_ERRLOG("EC bdev %s: RMW only %u/%u reads submitted for "
			    "stripe %" PRIu64 "\n",
			    ec->bdev.name, reads_submitted, ec->k, stripe_index);

		if (mctx->reads_remaining == 0) {
			struct spdk_bdev_io *bdev_io = ec_io->bdev_io;

			/*
			 * All submits failed synchronously. No callbacks will
			 * fire. Tear down the home-side bookkeeping (claim,
			 * counters) and self-complete the bdev_io. The
			 * in-memory WIB bit is intentionally NOT cleared here
			 * -- see the function preamble.
			 */
			ec_rmw_teardown(mctx);
			spdk_bdev_io_complete(bdev_io,
					      SPDK_BDEV_IO_STATUS_FAILED);
			return;
		}

		/*
		 * Some submits succeeded. Their callbacks will eventually
		 * call ec_rmw_complete with the FAILED status we already set.
		 */
		mctx->status = SPDK_BDEV_IO_STATUS_FAILED;
	}
}

/*
 * Setup half of the RMW core: validates the request, allocates the
 * ec_rmw_ctx + DMA buffers, claims the stripe-busy bit, and bumps the
 * per-region in-flight counter. All mutations here touch home-owned
 * state (claim test-and-set, WIB region map, dirty_ticks); the home-
 * thread assertion enforces that.
 *
 * After setup, hands off to ec_rmw_persist_and_dispatch which decides
 * whether to start a WIB persist (followed by ec_rmw_dispatch_reads on
 * persist completion) or proceed directly to the read dispatch when
 * the region bit is already durable. ec_rmw_dispatch_reads carries
 * the persist-completion -> submitter hop; when the caller is already
 * on the submitter thread the hop inlines and runs on the calling
 * thread.
 */
static int
ec_rmw_submit_core(struct ec_bdev_io *ec_io,
		   uint64_t offset_blocks, uint64_t num_blocks,
		   bool is_zero_fill,
		   void (*cb_fn)(void *, enum spdk_bdev_io_status),
		   void *cb_arg)
{
	struct ec_bdev    *ec          = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t           chunk_bytes = ec->strip_size * ec->bdev.blocklen;
	uint64_t           stripe_index;
	struct ec_rmw_ctx *mctx;
	uint32_t           disk;
	bool               was_clean;
	int                rc;

	/* Setup invariant: the stripe-busy claim is a non-atomic test-and-set, and home is the only setter for stripe_dirty_map / wib_region_inflight / dirty_ticks. Running setup off home would race a concurrent claimant. */
	assert(spdk_get_thread() == ec->home_thread);

	stripe_index = offset_blocks / ec->stripe_blocks;

	rc = ec_rmw_check_guards(ec, stripe_index);
	if (rc != 0) {
		return rc;
	}

	ec->rmw_total++;

	/* ------------------------------------------------------------------ */
	/* Allocate context                                                   */
	/*                                                                    */
	/* Per-I/O allocation: each sub-stripe write allocates this ctx plus  */
	/* n DMA chunk buffers (loop below), freed on completion. Under heavy */
	/* small-random-write load this calloc + n * spdk_dma_zmalloc per I/O */
	/* is the dominant RMW cost and the documented DMA-pressure risk.     */
	/* TODO(perf): pool ec_rmw_ctx + its DMA buffers on a per-channel     */
	/* free list and route pool exhaustion through the existing DMA       */
	/* back-pressure path. Deferred pending an I/O benchmark to validate  */
	/* the pool lifecycle against in-flight child writes.                 */
	/* ------------------------------------------------------------------ */
	mctx = calloc(1, sizeof(*mctx));
	if (!mctx) {
		return -ENOMEM;
	}

	mctx->ec_io                  = ec_io;
	mctx->num_blocks             = num_blocks;
	mctx->is_zero_fill           = is_zero_fill;
	mctx->stripe_index           = stripe_index;
	mctx->disk_lba               = ec_stripe_base_lba(ec, stripe_index);
	mctx->stripe_off_blocks      = offset_blocks % ec->stripe_blocks;
	mctx->status                 = SPDK_BDEV_IO_STATUS_SUCCESS;
	mctx->modified_chunk_first   = (uint32_t)(mctx->stripe_off_blocks / ec->strip_size);
	mctx->modified_chunk_last    = (uint32_t)((mctx->stripe_off_blocks + num_blocks - 1) / ec->strip_size);
	mctx->cb_fn                  = cb_fn;
	mctx->cb_arg                 = cb_arg;

	/*
	 * Cross-chunk policy split by I/O type:
	 *
	 *   WRITE: SPDK's _bdev_rw_split aligns at optimal_io_boundary
	 *     (= strip_size) and never lets a sub-stripe WRITE cross a
	 *     strip boundary. If we see a multi-chunk WRITE here it means
	 *     either SPDK regressed or some path bypassed the splitter.
	 *     Fail loudly -- this is the production canary, not a defensive
	 *     paper cut.
	 *
	 *   WRITE_ZEROES: SPDK's bdev_write_zeroes_split (lib/bdev/bdev.c)
	 *     only caps size at max_write_zeroes and does NOT align to
	 *     optimal_io_boundary, so a sub-stripe WRITE_ZEROES can
	 *     legitimately straddle a strip boundary within one stripe.
	 *     The overlay/encode steps already handle stripe-wide
	 *     modifications; ec_rmw_submit_writes writes back every chunk
	 *     in [modified_chunk_first, modified_chunk_last] alongside parity.
	 *
	 *   UNMAP head/tail zero-fill segment: same shape as WRITE_ZEROES,
	 *     ec_submit_rmw_zero_fill_range sets is_zero_fill = true.
	 */
	if (mctx->modified_chunk_last != mctx->modified_chunk_first &&
	    !is_zero_fill) {
		SPDK_ERRLOG("EC bdev %s: RMW payload spans chunks %u..%u "
			    "(stripe_off=%" PRIu64 " num_blocks=%" PRIu64 " strip_size=%" PRIu64 "); "
			    "split_on_optimal_io_boundary violated\n",
			    ec->bdev.name,
			    mctx->modified_chunk_first,
			    mctx->modified_chunk_last,
			    mctx->stripe_off_blocks,
			    num_blocks, ec->strip_size);
		free(mctx);
		return -EINVAL;
	}

	/* Allocate DMA buffers for all n slots (data + parity) */
	for (disk = 0; disk < ec->n; disk++) {
		mctx->chunk_bufs[disk] = spdk_dma_zmalloc(chunk_bytes,
							   EC_DMA_ALIGN, NULL);
		if (!mctx->chunk_bufs[disk]) {
			SPDK_ERRLOG("EC bdev %s: OOM for RMW chunk buf "
				    "(%" PRIu64 " bytes, slot %u stripe %" PRIu64 ")\n",
				    ec->bdev.name, chunk_bytes, disk, stripe_index);
			ec_rmw_free_ctx(mctx, ec);
			return -ENOMEM;
		}
		mctx->chunk_iovs[disk].iov_base = mctx->chunk_bufs[disk];
		mctx->chunk_iovs[disk].iov_len  = chunk_bytes;
	}

	ec_stripe_set_dirty(ec, stripe_index);
	ec_rmw_in_flight_inc(ec);

	/*
	 * Increment per-region in-flight count here -- not in
	 * ec_rmw_reads_done -- so that every code path through ec_rmw_complete
	 * has a matching increment regardless of whether reads_done reaches
	 * step 4. If reconstruction fails at step 1, or OOM at step 2,
	 * ec_rmw_complete is called directly and decrements this counter; the
	 * increment here ensures the counter stays consistent.
	 *
	 * The WIB region mark + persist decision lives here (rather than after
	 * modify/encode). The persist must land on disk BEFORE the read fan-out
	 * so a single multi-reactor hop at the persist-completion -> reads edge
	 * covers the entire submitter half. ec_wib_mark_region sets the bit,
	 * takes an inflight ref, and stamps dirty_ticks; was_clean (whether this
	 * call set the bit) drives the persist decision in
	 * ec_rmw_persist_and_dispatch.
	 */
	was_clean = ec_wib_mark_region(ec, stripe_index);

	/*
	 * Setup complete. Hand off to the persist + dispatch decision.
	 * persist_and_dispatch owns the mctx from this point: it either
	 * starts a persist (ec_rmw_wib_set_cb -> ec_rmw_dispatch_reads),
	 * queues on wib_deferred_writes (ec_wib_deferred_drain ->
	 * ec_rmw_dispatch_reads), or proceeds directly to dispatch when
	 * the bit was already durable. The setup half returns 0 ("queued
	 * via the cb chain"); any error completion happens through
	 * ec_rmw_complete -> spdk_bdev_io_complete.
	 */
	ec_rmw_persist_and_dispatch(mctx, was_clean);

	return 0;
}

/*
 * Standard sub-stripe write entry point. Pulls offset / length /
 * is_zero_fill from the bdev_io and routes through the shared core
 * with the legacy completion semantics (cb_fn = NULL means
 * ec_rmw_complete will call spdk_bdev_io_complete on the parent).
 */
int
ec_submit_rmw_write(struct ec_bdev_io *ec_io)
{
	return ec_rmw_submit_core(ec_io,
				  ec_io->offset_blocks,
				  ec_io->num_blocks,
				  ec_io->is_zero_fill,
				  NULL, NULL);
}

/*
 * Internal sub-stripe zero-fill entry point used by the multi-segment
 * UNMAP dispatcher. The range must lie within a single stripe; the
 * dispatcher guarantees that by construction (head/tail fragments are
 * each at most stripe_blocks-1 blocks and bounded by a stripe
 * boundary). A defensive assertion catches future misuse.
 *
 * is_zero_fill is implicit in the entry-point name and hard-wired to
 * true on the ctx; the iov fields of ec_io are NOT consulted.
 */
int
ec_submit_rmw_zero_fill_range(struct ec_bdev_io *ec_io,
			      uint64_t offset_blocks,
			      uint64_t num_blocks,
			      void (*cb_fn)(void *cb_arg,
					    enum spdk_bdev_io_status status),
			      void *cb_arg)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t        end_blocks;

	if (num_blocks == 0 || num_blocks > ec->stripe_blocks) {
		SPDK_ERRLOG("EC bdev %s: zero-fill range invalid "
			    "(offset=%" PRIu64 " num_blocks=%" PRIu64 " stripe_blocks=%" PRIu64 ")\n",
			    ec->bdev.name, offset_blocks, num_blocks,
			    ec->stripe_blocks);
		return -EINVAL;
	}

	end_blocks = offset_blocks + num_blocks;
	if (offset_blocks / ec->stripe_blocks !=
	    (end_blocks - 1) / ec->stripe_blocks) {
		SPDK_ERRLOG("EC bdev %s: zero-fill range straddles stripe "
			    "boundary (offset=%" PRIu64 " num_blocks=%" PRIu64 " "
			    "stripe_blocks=%" PRIu64 ")\n",
			    ec->bdev.name, offset_blocks, num_blocks,
			    ec->stripe_blocks);
		return -EINVAL;
	}

	return ec_rmw_submit_core(ec_io,
				  offset_blocks,
				  num_blocks,
				  true /* is_zero_fill */,
				  cb_fn, cb_arg);
}
