/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_rebuild.c -- background rebuild + startup scrub.
 *
 * Two SPDK-poller-driven background subsystems share this file:
 *
 *   Rebuild:  reconstructs data on REPLACING slots stripe-by-stripe,
 *             transitioning slots back to NORMAL on completion. Started
 *             from the bdev_ec_start_rebuild RPC.
 *
 *   Scrub:    re-encodes parity for stripes whose WIB region was dirty at
 *             crash time, then clears the region bit. Started from
 *             ec_bdev_create_finalize after WIB and bitmap load, or from
 *             ec_rebuild_finish when scrub was deferred at boot because
 *             a data disk was not NORMAL.
 *
 * They share the per-stripe read-reconstruct-write pattern and the
 * reconstruction helpers in bdev_ec_io.c (called via the internal API).
 * Both transition cleanly into each other's state machine, hence one file.
 */

#include "bdev_ec_internal.h"

#include "spdk/bdev_module.h"
#include "spdk/thread.h"
#include "spdk/util.h"

#include <isa-l/erasure_code.h>

/* =========================================================================
 * Background rebuild
 *
 * A poller (ec_rebuild_poller_cb) walks the volume one stripe at a
 * time. For each stripe it reads k NORMAL chunks, reconstructs the
 * REPLACING slot's chunk, and writes it back. Slots stay in
 * REPLACING throughout -- the transition to NORMAL is published all
 * at once in ec_rebuild_finish, so concurrent user reads see a
 * consistent state.
 *
 * A REPLACING disk that fails again during rebuild is handled by
 * ec_handle_base_bdev_failure (which flips the slot back to FAILED).
 * The next read/write child for that slot returns -EIO and the chain
 * unwinds through ec_rebuild_finish.
 * ========================================================================= */

/* Forward declarations for the rebuild chain */
static int  ec_rebuild_poller_cb(void *arg);
static void ec_rebuild_submit_stripe_reads(struct ec_rebuild_ctx *ctx);
static void ec_rebuild_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ec_rebuild_reconstruct_write(struct ec_rebuild_ctx *ctx);
static void ec_rebuild_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ec_rebuild_move_to_next_slot(struct ec_rebuild_ctx *ctx);
static void ec_rebuild_finish(struct ec_rebuild_ctx *ctx, int rc);

/*
 * Release the per-slot I/O channels and DMA chunk buffers held by a background
 * (rebuild or scrub) context. Both engines hold identically-shaped per-slot
 * arrays, so they share this teardown.
 */
static void
ec_free_channels_and_buffers(struct spdk_io_channel **chans, uint8_t **bufs, uint32_t n)
{
	uint32_t i;

	for (i = 0; i < n; i++) {
		if (chans[i]) {
			spdk_put_io_channel(chans[i]);
			chans[i] = NULL;
		}
		if (bufs[i]) {
			spdk_dma_free(bufs[i]);
			bufs[i] = NULL;
		}
	}
}

/*
 * Release all DMA buffers and I/O channels held by the rebuild context.
 * Called from ec_rebuild_finish() before freeing the context.
 */
static void
ec_rebuild_free_resources(struct ec_rebuild_ctx *ctx)
{
	ec_free_channels_and_buffers(ctx->rebuild_chans, ctx->chunk_bufs, ctx->ec->n);
}

/*
 * Reconstruct any unreadable data chunks, then re-encode all m parity
 * chunks into chunk_bufs[k..n-1].
 *
 * A data slot that is REPLACING (rebuild) or FAILED (scrub) was skipped
 * on the read phase and still holds stale bytes; encoding from those
 * would land wrong parity on disk. Reconstruct first.
 *
 * Returns 0 on success; negative rc from ec_reconstruct_multi_data on
 * failure (chunk_bufs untouched). Caller logs and aborts the stripe.
 */
static int
ec_reencode_parity_from_chunk_bufs(const struct ec_bdev *ec,
				   uint8_t *chunk_bufs[EC_MAX_BASE_BDEVS],
				   uint64_t chunk_bytes)
{
	uint8_t *data_ptrs[EC_MAX_BASE_BDEVS];
	uint8_t *parity_ptrs[EC_MAX_BASE_BDEVS];
	uint32_t failed_data_slots[EC_MAX_BASE_BDEVS];
	uint8_t *failed_out_bufs[EC_MAX_BASE_BDEVS];
	uint32_t num_failed = 0;
	uint32_t i;
	int      rc;

	for (i = 0; i < ec->k; i++) {
		if (!ec_slot_is_readable(ec, i)) {
			failed_data_slots[num_failed] = i;
			failed_out_bufs[num_failed]   = chunk_bufs[i];
			num_failed++;
		}
	}

	if (num_failed > 0) {
		rc = ec_reconstruct_multi_data(ec, chunk_bufs, failed_out_bufs,
					       failed_data_slots, num_failed,
					       chunk_bytes);
		if (rc != 0) {
			return rc;
		}
	}

	for (i = 0; i < ec->k; i++) {
		data_ptrs[i] = chunk_bufs[i];
	}
	for (i = 0; i < ec->m; i++) {
		parity_ptrs[i] = chunk_bufs[ec->k + i];
	}

	ec_encode_data((int)chunk_bytes, (int)ec->k, (int)ec->m,
		       ec->g_tbls, data_ptrs, parity_ptrs);

	return 0;
}

/*
 * ec_rebuild_finish  [terminal step for success and failure]
 *
 * Finalises the rebuild:
 *   - On success (rc == 0): transition each rebuilt slot REPLACING -> NORMAL,
 *     clear needs_rebuild[], decrement failed_count.
 *   - On any outcome: unregister poller, release resources, clear
 *     ec->rebuild_ctx, invoke cb_fn.
 *
 * The slot state is only mutated here (not in the per-stripe callbacks) so
 * the transition is atomic with respect to the app thread: user I/O sees
 * the slot as REPLACING right up until the instant it becomes NORMAL.
 */
static void
ec_rebuild_finish(struct ec_rebuild_ctx *ctx, int rc)
{
	struct ec_bdev *ec = ctx->ec;
	uint32_t        i;

	if (ctx->poller) {
		spdk_poller_unregister(&ctx->poller);
	}

	/*
	 * Release any held stripe-busy claim before the slot-state transition
	 * below. The claim guards parity from concurrent foreground writers --
	 * the rebuild write has either landed (success path) or been abandoned
	 * (error path), so neither side can be racing the parity now.
	 */
	if (ctx->stripe_claimed) {
		ec_stripe_clear_dirty(ec, ctx->current_stripe);
		ctx->stripe_claimed = false;
	}

	/*
	 * Drain any stripes deferred during the main scan. On the failure path
	 * we discard them: the rebuild is aborting, so the foreground writers
	 * that triggered the defers will simply land on a still-FAILED or
	 * still-REPLACING slot and the next rebuild attempt will replay the
	 * full per-slot scan.
	 */
	{
		struct ec_rebuild_deferred_stripe *d, *tmp;

		TAILQ_FOREACH_SAFE(d, &ctx->deferred_stripes, link, tmp) {
			TAILQ_REMOVE(&ctx->deferred_stripes, d, link);
			free(d);
		}
	}

	if (rc == 0) {
		/*
		 * Transition all REPLACING slots that we successfully rebuilt.
		 * We identify them by checking needs_rebuild[] because a slot
		 * that failed a second time during the rebuild would have had
		 * its needs_rebuild[] cleared by ec_handle_base_bdev_failure.
		 *
		 * This is sound only because new replacements cannot appear
		 * mid-rebuild: ec_bdev_replace_base_bdev rejects with -EBUSY
		 * while ec->rebuild_ctx is set. Every REPLACING slot still
		 * carrying needs_rebuild[] was therefore present at rebuild
		 * start and walked forward by ec_rebuild_move_to_next_slot.
		 */
		for (i = 0; i < ec->n; i++) {
			if (ec->base_states[i] == EC_BASE_STATE_REPLACING &&
			    ec->needs_rebuild[i]) {
				ec->base_states[i]  = EC_BASE_STATE_NORMAL;
				ec->needs_rebuild[i] = false;
				ec->failed_count--;

				SPDK_NOTICELOG("EC bdev %s: slot %u rebuild "
					       "complete -- REPLACING -> NORMAL. "
					       "failed_count now %u.\n",
					       ec->bdev.name, i,
					       ec->failed_count);

				/*
				 * Reopen WIB I/O channel for rebuilt parity
				 * slots. The channel was released in
				 * ec_handle_base_bdev_failure when the old
				 * parity disk failed; now that the replacement
				 * disk is NORMAL, WIB persists must resume
				 * writing to it.
				 */
				if (i >= ec->k) {
					uint32_t parity_idx = i - ec->k;
					if (!ec->wib_chans[parity_idx] && ec->descs[i]) {
						ec->wib_chans[parity_idx] = spdk_bdev_get_io_channel(ec->descs[i]);
						if (!ec->wib_chans[parity_idx]) {
							SPDK_WARNLOG("EC bdev %s: failed to reopen "
								     "WIB channel for rebuilt parity "
								     "slot %u\n",
								     ec->bdev.name, i);
						}
					}
				}
			}
		}
		if (ec->offline && ec->failed_count <= ec->m) {
			ec->offline = false;
			SPDK_NOTICELOG("EC bdev %s: failed_count back to %u "
				       "(<= m=%u), clearing offline flag\n",
				       ec->bdev.name, ec->failed_count,
				       ec->m);
		}
		{
			uint64_t ticks_per_second = spdk_get_ticks_hz();
			uint64_t elapsed_seconds  = ticks_per_second != 0 ?
				(spdk_get_ticks() - ctx->heartbeat.start_ticks) / ticks_per_second : 0;
			uint32_t wib_dirty        = ec_wib_count_dirty(ec);

			SPDK_NOTICELOG("EC bdev %s: rebuild complete: "
				       "%" PRIu64 " stripes written across %u "
				       "slot(s) in %" PRIu64 "s; failed_count now %u; "
				       "WIB dirty regions remaining: %u%s\n",
				       ec->bdev.name, ctx->stripes_rebuilt,
				       ctx->slots_to_rebuild, elapsed_seconds,
				       ec->failed_count, wib_dirty,
				       wib_dirty > 0 ? " (scrub will sweep them)" : "");
		}

		/*
		 * Attempt the startup scrub if it was deferred at
		 * boot because a data disk was not NORMAL at that time.
		 *
		 * Condition: rebuild succeeded (all data disks now NORMAL),
		 * no scrub is already running, and at least one WIB region
		 * is still dirty (set by ec_wib_load_async at startup).
		 *
		 * If ec_bdev_start_scrub succeeds it installs ec->scrub_ctx
		 * and the scrub runs in the background. If it fails or finds
		 * no dirty regions, the dirty region bits remain set and will
		 * be re-examined on the next startup.
		 */
		if (ec->scrub_ctx == NULL && ec_wib_count_dirty(ec) > 0) {
			int scrub_rc = ec_bdev_start_scrub(ec);
			if (scrub_rc != 0) {
				SPDK_WARNLOG("EC bdev %s: deferred "
					     "scrub start failed "
					     "(rc=%d); dirty regions "
					     "remain set\n",
					     ec->bdev.name, scrub_rc);
			}
		}

		/*
		 * If deferred-scrub guard was holding RMW backpressure (rebuild
		 * needed to restore a failed data disk first), clear the flag.
		 * If a scrub starts here it will install its own backpressure
		 * episode when its active-scrub guard fires.
		 */
		ec_rmw_backpressure_end(ec, "rebuild restored failed slot");
	} else {
		SPDK_ERRLOG("EC bdev %s: rebuild aborted (rc=%d) after "
			    "%" PRIu64 " stripes.\n",
			    ec->bdev.name, rc, ctx->stripes_rebuilt);
	}

	ec_rebuild_free_resources(ctx);

	ec->rebuild_ctx = NULL;

	ctx->cb_fn(ctx->cb_arg, rc, ctx->stripes_rebuilt);
	free(ctx);
}

/*
 * Called when the current slot's stripe loop is complete.
 * Scans forward for the next slot with needs_rebuild[slot]==true.
 * If found, resets current_stripe and returns (poller continues).
 * If not found, calls ec_rebuild_finish(ctx, 0).
 */
static void
ec_rebuild_move_to_next_slot(struct ec_rebuild_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;
	uint32_t        slot;

	for (slot = ctx->current_slot + 1; slot < ec->n; slot++) {
		if (ec->needs_rebuild[slot] &&
		    ec->base_states[slot] == EC_BASE_STATE_REPLACING) {
			ctx->current_slot   = slot;
			ctx->current_stripe = 0;

			SPDK_NOTICELOG("EC bdev %s: rebuild advancing to "
				       "slot %u\n", ec->bdev.name, slot);
			return;
		}
	}

	/* No more slots -- done */
	ec_rebuild_finish(ctx, 0);
}

/* Emit a rebuild-progress NOTICE per the EC_BG_HEARTBEAT_* policy;
 * called once per completed stripe. */
static void
ec_rebuild_heartbeat(struct ec_rebuild_ctx *ctx)
{
	uint64_t total_stripe_writes = ctx->num_stripes * (uint64_t)ctx->slots_to_rebuild;
	struct ec_bg_heartbeat_decision decision;

	decision = ec_heartbeat_should_fire(&ctx->heartbeat,
					    ctx->stripes_rebuilt, total_stripe_writes);
	if (!decision.fire) {
		return;
	}

	SPDK_NOTICELOG("EC bdev %s: rebuild progress %u%% "
		       "(slot %u, %" PRIu64 "/%" PRIu64 " stripes total, "
		       "%" PRIu64 "s elapsed, ~%" PRIu64 "s remaining)\n",
		       ctx->ec->bdev.name, decision.percent,
		       ctx->current_slot,
		       ctx->stripes_rebuilt, total_stripe_writes,
		       decision.elapsed_seconds, decision.remaining_seconds);
}

/*
 * Write completion callback for one stripe.
 * On success: releases the stripe-busy claim, advances current_stripe (only
 *             in the main scan -- the deferred-drain pass picks the next
 *             stripe from the queue), increments stripes_rebuilt, clears
 *             io_in_flight so the poller submits the next stripe.
 * On failure: aborts the rebuild via ec_rebuild_finish (which releases the
 *             claim and drains the deferred queue).
 */
static void
ec_rebuild_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_rebuild_ctx *ctx = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		SPDK_ERRLOG("EC bdev %s: rebuild write failed at stripe %" PRIu64 " "
			    "slot %u\n",
			    ctx->ec->bdev.name, ctx->current_stripe,
			    ctx->current_slot);
		ec_rebuild_finish(ctx, -EIO);
		return;
	}

	if (ctx->stripe_claimed) {
		ec_stripe_clear_dirty(ctx->ec, ctx->current_stripe);
		ctx->stripe_claimed = false;
	}

	ctx->stripes_rebuilt++;
	if (!ctx->draining_deferred_stripes) {
		ctx->current_stripe++;
	}
	ctx->io_in_flight = false;

	ec_rebuild_heartbeat(ctx);
}

/*
 * Called when all reads for a stripe have completed successfully.
 * Reconstructs the missing chunk and submits a write to the REPLACING disk.
 *
 * DATA slot (current_slot < k):
 *   Uses ec_reconstruct_data_chunk() -- RS decode, same as degraded read.
 *
 * PARITY slot (current_slot >= k):
 *   All k data disks are readable (any REPLACING data disk would have already
 *   caused a read failure or aborted the rebuild). Uses ec_encode_data() to
 *   re-encode just the one parity column, which is simpler and faster.
 */
static void
ec_rebuild_reconstruct_write(struct ec_rebuild_ctx *ctx)
{
	struct ec_bdev *ec   = ctx->ec;
	uint32_t        slot = ctx->current_slot;
	int             rc;

	if (slot < ec->k) {
		/*
		 * DATA slot: RS decode from k surviving NORMAL chunks.
		 * chunk_bufs[] already holds the read data for NORMAL disks.
		 * ec_reconstruct_data_chunk() writes the result into
		 * chunk_bufs[slot].
		 */
		rc = ec_reconstruct_data_chunk(ec,
					       ctx->chunk_bufs,
					       ctx->chunk_bufs[slot],
					       slot,
					       ctx->chunk_bytes);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: ISA-L reconstruction failed "
				    "at stripe %" PRIu64 " slot %u\n",
				    ec->bdev.name, ctx->current_stripe, slot);
			ec_rebuild_finish(ctx, -EIO);
			return;
		}
	} else {
		/*
		 * PARITY slot: re-encode from all k data chunks. We compute
		 * all m parity chunks (cheap; ec_encode_data does both in one
		 * pass) but only write the target slot; the extra m-1 land in
		 * the other parity slots' scratch buffers.
		 *
		 * Multi-failure rebuild corner case: a DATA slot that is also
		 * REPLACING (rebuilt in an earlier pass of this session) was
		 * skipped on the read phase and its chunk_bufs entry is still
		 * zeroed. ec_reencode_parity_from_chunk_bufs reconstructs any
		 * such gap from the k NORMAL survivors before encoding so
		 * the parity it computes matches the true stripe content.
		 * ec_rebuild_finish only transitions rebuilt slots to NORMAL
		 * once the whole rebuild is done, so a slot that finished
		 * earlier in this same session is still REPLACING here --
		 * which is what makes the guard necessary.
		 */
		rc = ec_reencode_parity_from_chunk_bufs(ec, ctx->chunk_bufs,
							ctx->chunk_bytes);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: rebuild parity slot %u "
				    "-- multi-data reconstruction failed "
				    "at stripe %" PRIu64 " (rc=%d)\n",
				    ec->bdev.name, slot,
				    ctx->current_stripe, rc);
			ec_rebuild_finish(ctx, -EIO);
			return;
		}
	}

	/* Submit the write to the REPLACING disk */
	rc = spdk_bdev_writev_blocks(ec->descs[slot],
				     ctx->rebuild_chans[slot],
				     &ctx->chunk_iovs[slot], 1,
				     ec_stripe_base_lba(ec, ctx->current_stripe),
				     ec->strip_size,
				     ec_rebuild_write_cb,
				     ctx);
	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: failed to submit rebuild write "
			    "at stripe %" PRIu64 " slot %u (rc=%d)\n",
			    ec->bdev.name, ctx->current_stripe, slot, rc);
		ec_rebuild_finish(ctx, rc);
	}
}

/* Per-read completion: dispatch reconstruct on success, abort on failure once all reads return. */
static void
ec_rebuild_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_rebuild_ctx *ctx = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		ctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
		SPDK_ERRLOG("EC bdev %s: rebuild read failed at stripe %" PRIu64 " "
			    "slot %u\n",
			    ctx->ec->bdev.name, ctx->current_stripe,
			    ctx->current_slot);
	}

	ctx->reads_remaining--;

	if (ctx->reads_remaining == 0) {
		if (ctx->io_status != SPDK_BDEV_IO_STATUS_SUCCESS) {
			ec_rebuild_finish(ctx, -EIO);
			return;
		}
		ec_rebuild_reconstruct_write(ctx);
	}
}

/*
 * Submits read I/Os for k NORMAL (readable) disks for the current stripe.
 * We need exactly k reads to feed the reconstruction of the current REPLACING
 * slot. If fewer than k readable disks exist the rebuild cannot proceed
 * (more failures than ec->m), so we abort.
 *
 * When multiple REPLACING slots exist, the rebuild outer loop
 * processes them one at a time. On each pass the k reads come from the
 * remaining NORMAL disks (data + parity combined), which always number at
 * least k as long as failed_count <= m. Slots in REPLACING state remain
 * unreadable throughout the rebuild; their content is reconstructed from the
 * NORMAL survivors via ISA-L RS decode (data slots) or ec_encode_data (parity
 * slots). Any k readable rows of the RS encode matrix can be inverted, so
 * reconstruction succeeds regardless of which specific combination of data
 * and parity slots is readable.
 *
 * disk_lba = ec_stripe_base_lba(ec, current_stripe)
 *   (each disk stores one strip per stripe at the same per-disk LBA)
 */
static void
ec_rebuild_submit_stripe_reads(struct ec_rebuild_ctx *ctx)
{
	struct ec_bdev *ec       = ctx->ec;
	uint64_t        disk_lba = ec_stripe_base_lba(ec, ctx->current_stripe);
	uint32_t        disk;
	uint32_t        reads_submitted = 0;
	int             rc;

	ctx->io_status       = SPDK_BDEV_IO_STATUS_SUCCESS;
	ctx->reads_remaining = 0;
	ctx->io_in_flight    = true;

	for (disk = 0; disk < ec->n; disk++) {
		if (!ec_slot_is_readable(ec, disk)) {
			continue;
		}
		if (reads_submitted >= ec->k) {
			break;
		}

		ctx->reads_remaining++;

		rc = spdk_bdev_readv_blocks(ec->descs[disk],
					    ctx->rebuild_chans[disk],
					    &ctx->chunk_iovs[disk], 1,
					    disk_lba,
					    ec->strip_size,
					    ec_rebuild_read_cb,
					    ctx);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: failed to submit rebuild "
				    "read for disk %u stripe %" PRIu64 " (rc=%d)\n",
				    ec->bdev.name, disk,
				    ctx->current_stripe, rc);
			ctx->reads_remaining--;
			ctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
		} else {
			reads_submitted++;
		}
	}

	if (reads_submitted < ec->k) {
		SPDK_ERRLOG("EC bdev %s: only %u/%u reads submitted for "
			    "rebuild stripe %" PRIu64 "; aborting\n",
			    ec->bdev.name, reads_submitted, ec->k,
			    ctx->current_stripe);
		if (ctx->reads_remaining == 0) {
			/*
			 * All submits failed synchronously; no callbacks
			 * will fire. Clean up now.
			 */
			ctx->io_in_flight = false;
			ec_rebuild_finish(ctx, -EIO);
			return;
		}
		/*
		 * Some submits succeeded -- their callbacks will decrement
		 * reads_remaining and eventually call ec_rebuild_finish
		 * via ec_rebuild_read_cb when io_status is FAILED.
		 */
		ctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
	}
}

/*
 * SPDK poller that drives the rebuild loop. Registered with period
 * EC_BG_POLL_PERIOD_US so the reactor can service other I/O between
 * rebuild stripes.
 *
 * State machine per invocation:
 *   1. paused              -> return IDLE (QoS gate).
 *   2. cancel_requested    -> drain in-flight I/O, then finish.
 *   3. io_in_flight        -> return BUSY (single stripe in flight).
 *   4. rate-limit exceeded -> return IDLE (QoS gate).
 *   5. Pick next stripe:
 *        draining_deferred_stripes  -> pop from deferred_stripes; if empty, advance slot.
 *        main scan done    -> if deferred_stripes non-empty, flip into deferred
 *                              pass; else advance slot.
 *        normal            -> use current_stripe as is.
 *   6. Stripe-busy check:
 *        ec_stripe_is_dirty(current_stripe) -> defer this stripe and return.
 *        otherwise -> claim, submit reads.
 */
static int
ec_rebuild_poller_cb(void *arg)
{
	struct ec_rebuild_ctx *ctx = arg;
	struct ec_bdev        *ec  = ctx->ec;

	/*
	 * Wait while a stripe's I/O is outstanding. Checked before the pause/QoS
	 * gates below (which return IDLE) so a paused or rate-limited rebuild cannot
	 * starve teardown: the gate term (ec_teardown_must_defer) holds device
	 * teardown and base-failure channel release until this drains.
	 */
	if (ctx->io_in_flight) {
		return SPDK_POLLER_BUSY;
	}

	/* Release any base-bdev-failure slot cleanup parked behind the drained I/O. */
	ec_drain_deferred_slot_releases(ec);

	/*
	 * A delete or shutdown started. ec_finish_device_unregister cleans up scrub
	 * but never rebuild, so tear the rebuild down here before the deferred
	 * teardown frees ec: ec_rebuild_finish unregisters this poller (safe from
	 * within it, as the cancel gate below also does), releases the rebuild
	 * channels and buffers, clears ec->rebuild_ctx, and reports -ECANCELED to
	 * the caller. Then drive the deferred teardown; it may free ec, so nothing
	 * touches it afterward.
	 */
	if (ec->destructing) {
		ec_rebuild_finish(ctx, -ECANCELED);
		ec_drain_deferred_unregister(ec);
		return SPDK_POLLER_BUSY;
	}

	/* QoS: pause gate */
	if (ctx->paused) {
		return SPDK_POLLER_IDLE;
	}

	/* Cancel gate: in-flight I/O already drained above, so finish now. */
	if (ctx->cancel_requested) {
		ec_rebuild_finish(ctx, -ECANCELED);
		return SPDK_POLLER_BUSY;
	}

	/* QoS: rate-limit gate */
	if (ctx->max_stripes_per_sec > 0) {
		uint64_t now = spdk_get_ticks();

		if (now - ctx->window_start_ticks >= spdk_get_ticks_hz()) {
			ctx->window_start_ticks  = now;
			ctx->stripes_this_window = 0;
		}
		if (ctx->stripes_this_window >= ctx->max_stripes_per_sec) {
			return SPDK_POLLER_IDLE;
		}
	}

	/* Pick the stripe to operate on. */
	if (ctx->draining_deferred_stripes) {
		struct ec_rebuild_deferred_stripe *entry =
			TAILQ_FIRST(&ctx->deferred_stripes);

		if (entry == NULL) {
			/* Deferred queue drained; the slot is fully rebuilt. */
			ctx->draining_deferred_stripes = false;
			ec_rebuild_move_to_next_slot(ctx);
			return SPDK_POLLER_BUSY;
		}

		TAILQ_REMOVE(&ctx->deferred_stripes, entry, link);
		ctx->current_stripe = entry->stripe_index;
		free(entry);
	} else if (ctx->current_stripe >= ctx->num_stripes) {
		/*
		 * Main per-slot scan finished. If any stripes were deferred
		 * (busy under foreground writers at the time), drain them
		 * before transitioning to the next slot.
		 */
		if (!TAILQ_EMPTY(&ctx->deferred_stripes)) {
			ctx->draining_deferred_stripes = true;
			return SPDK_POLLER_BUSY;
		}

		ec_rebuild_move_to_next_slot(ctx);
		/*
		 * ec_rebuild_move_to_next_slot either:
		 *   - Updates current_slot/current_stripe and returns (poller
		 *     continues on next tick for the new slot), or
		 *   - Calls ec_rebuild_finish which unregisters the poller.
		 * Either way returning BUSY here is correct: the poller has
		 * done real work (slot transition or finish).
		 */
		return SPDK_POLLER_BUSY;
	}

	/*
	 * Skip unmapped stripes. An unmapped stripe is logically zero: every
	 * read returns zeros without touching the disks, so the replacement
	 * slot's chunk is never read and there is nothing to reconstruct. If
	 * the stripe is written again it is repopulated through the
	 * write-into-unmapped path.
	 *
	 * Count it as rebuilt so progress reaches 100%. Advance in the main
	 * scan; in the deferred pass the entry was already dequeued.
	 */
	if (ec->stripe_unmapped_map != NULL &&
	    ec_stripe_is_unmapped(ec, ctx->current_stripe)) {
		ctx->stripes_rebuilt++;
		if (!ctx->draining_deferred_stripes) {
			ctx->current_stripe++;
		}
		return SPDK_POLLER_BUSY;
	}

	/*
	 * Stripe-busy interlock.
	 *
	 * A foreground RMW, full-stripe write, or UNMAP may have claimed this
	 * stripe between the previous tick and now. If so, park the stripe on
	 * the deferred queue and skip ahead: in the main scan we advance
	 * current_stripe so the slot scan keeps progressing; in the deferred
	 * pass we leave current_stripe alone (the next tick will pop another
	 * entry from the queue and overwrite it). On allocation failure we
	 * silently retry the same stripe on the next tick -- the foreground
	 * claim is short-lived and the rebuild is rate-limited anyway.
	 */
	if (ec_stripe_is_dirty(ec, ctx->current_stripe)) {
		struct ec_rebuild_deferred_stripe *entry =
			calloc(1, sizeof(*entry));

		if (entry == NULL) {
			return SPDK_POLLER_BUSY;
		}

		entry->stripe_index = ctx->current_stripe;
		TAILQ_INSERT_TAIL(&ctx->deferred_stripes, entry, link);

		if (!ctx->draining_deferred_stripes) {
			ctx->current_stripe++;
		}
		return SPDK_POLLER_BUSY;
	}

	/* Claim the stripe and submit reads. */
	ec_stripe_set_dirty(ec, ctx->current_stripe);
	ctx->stripe_claimed = true;

	/*
	 * Count this stripe against the QoS window before submitting: a total
	 * synchronous submit failure finishes the rebuild and frees ctx inside
	 * ec_rebuild_submit_stripe_reads, so ctx must not be read afterward.
	 */
	if (ctx->max_stripes_per_sec > 0) {
		ctx->stripes_this_window++;
	}

	ec_rebuild_submit_stripe_reads(ctx);

	return SPDK_POLLER_BUSY;
}

int
ec_bdev_start_rebuild(const char *ec_name,
		      ec_rebuild_cb_fn cb_fn, void *cb_arg)
{
	struct ec_bdev        *ec;
	struct ec_rebuild_ctx *ctx;
	uint32_t               i;
	uint32_t               first_slot = UINT32_MAX;
	uint32_t               slots_to_rebuild = 0;

	ec = ec_bdev_find(ec_name);
	if (!ec) {
		SPDK_ERRLOG("bdev_ec_start_rebuild: EC bdev '%s' not found\n",
			    ec_name);
		return -ENODEV;
	}

	/* Reject if a rebuild or resize is already in progress. */
	if (ec->rebuild_ctx != NULL || ec->resize_ctx != NULL) {
		SPDK_ERRLOG("EC bdev %s: rebuild rejected -- "
			    "rebuild or resize already in progress\n",
			    ec->bdev.name);
		return -EBUSY;
	}

	/* Count REPLACING slots that still need a rebuild. */
	for (i = 0; i < ec->n; i++) {
		if (ec->base_states[i] == EC_BASE_STATE_REPLACING &&
		    ec->needs_rebuild[i]) {
			slots_to_rebuild++;
			if (first_slot == UINT32_MAX) {
				first_slot = i;
			}
		}
	}
	if (slots_to_rebuild == 0) {
		SPDK_NOTICELOG("EC bdev %s: rebuild rejected -- "
			       "no REPLACING slots\n", ec->bdev.name);
		return -ENOENT;
	}

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		return -ENOMEM;
	}

	ctx->ec                   = ec;
	ctx->current_slot         = first_slot;
	ctx->current_stripe       = 0;
	ctx->num_stripes = ec->bdev.blockcnt / ec->stripe_blocks;
	ctx->chunk_bytes = ec->strip_size * ec->bdev.blocklen;
	ctx->io_in_flight = false;
	ctx->stripes_rebuilt = 0;
	ctx->slots_to_rebuild = slots_to_rebuild;
	ctx->heartbeat.start_ticks = spdk_get_ticks();
	ctx->heartbeat.last_heartbeat_ticks = ctx->heartbeat.start_ticks;
	ctx->heartbeat.next_heartbeat_percent = EC_BG_HEARTBEAT_PERCENT_STEP;
	ctx->stripe_claimed = false;
	ctx->draining_deferred_stripes = false;
	TAILQ_INIT(&ctx->deferred_stripes);
	ctx->cb_fn = cb_fn;
	ctx->cb_arg = cb_arg;

	/* Open dedicated I/O channels for each live disk. */
	for (i = 0; i < ec->n; i++) {
		if (!ec->descs[i]) {
			ctx->rebuild_chans[i] = NULL;
			continue;
		}
		ctx->rebuild_chans[i] = spdk_bdev_get_io_channel(ec->descs[i]);
		if (!ctx->rebuild_chans[i]) {
			SPDK_ERRLOG("EC bdev %s: rebuild -- failed to open "
				    "channel for slot %u\n",
				    ec->bdev.name, i);
			ec_rebuild_free_resources(ctx);
			free(ctx);
			return -ENOMEM;
		}
	}

	/* Allocate DMA buffers (reused for every stripe). */
	for (i = 0; i < ec->n; i++) {
		ctx->chunk_bufs[i] = spdk_dma_zmalloc(ctx->chunk_bytes,
						       EC_DMA_ALIGN, NULL);
		if (!ctx->chunk_bufs[i]) {
			SPDK_ERRLOG("EC bdev %s: rebuild -- OOM for chunk buf "
				    "slot %u\n",
				    ec->bdev.name, i);
			ec_rebuild_free_resources(ctx);
			free(ctx);
			return -ENOMEM;
		}
		ctx->chunk_iovs[i].iov_base = ctx->chunk_bufs[i];
		ctx->chunk_iovs[i].iov_len  = ctx->chunk_bytes;
	}

	/* Register the poller and publish the context. */
	ec->rebuild_ctx = ctx;

	ctx->poller = spdk_poller_register(ec_rebuild_poller_cb, ctx,
					   EC_BG_POLL_PERIOD_US);
	if (!ctx->poller) {
		SPDK_ERRLOG("EC bdev %s: rebuild -- failed to register poller\n",
			    ec->bdev.name);
		ec_rebuild_free_resources(ctx);
		ec->rebuild_ctx = NULL;
		free(ctx);
		return -ENOMEM;
	}

	SPDK_NOTICELOG("EC bdev %s: rebuild started -- %" PRIu64 " stripes, "
		       "first slot %u\n",
		       ec->bdev.name, ctx->num_stripes, first_slot);

	return 0;
}

int
ec_bdev_stop_rebuild(const char *ec_name)
{
	struct ec_bdev *ec = ec_bdev_find(ec_name);

	if (!ec) {
		return -ENODEV;
	}
	if (!ec->rebuild_ctx) {
		return -ENOENT;
	}

	ec->rebuild_ctx->cancel_requested = true;

	SPDK_NOTICELOG("EC bdev %s: rebuild cancel requested\n",
		       ec->bdev.name);

	return 0;
}

int
ec_bdev_set_rebuild_qos(const char *ec_name,
			uint32_t max_stripes_per_sec,
			bool paused)
{
	struct ec_bdev *ec = ec_bdev_find(ec_name);

	if (!ec) {
		return -ENODEV;
	}
	if (!ec->rebuild_ctx) {
		return -ENOENT;
	}

	ec->rebuild_ctx->max_stripes_per_sec = max_stripes_per_sec;
	ec->rebuild_ctx->paused              = paused;

	SPDK_NOTICELOG("EC bdev %s: rebuild QoS updated -- "
		       "max_stripes_per_sec=%u paused=%s\n",
		       ec->bdev.name, max_stripes_per_sec,
		       paused ? "true" : "false");

	return 0;
}

int
ec_bdev_get_rebuild_progress(const char *ec_name,
			     struct ec_rebuild_progress *out)
{
	struct ec_bdev *ec = ec_bdev_find(ec_name);

	if (!ec) {
		return -ENODEV;
	}
	if (!ec->rebuild_ctx) {
		return -ENOENT;
	}

	out->current_slot     = ec->rebuild_ctx->current_slot;
	out->current_stripe   = ec->rebuild_ctx->current_stripe;
	out->num_stripes      = ec->rebuild_ctx->num_stripes;
	out->stripes_rebuilt  = ec->rebuild_ctx->stripes_rebuilt;
	out->slots_to_rebuild = ec->rebuild_ctx->slots_to_rebuild;

	return 0;
}

/* -------------------------------------------------------------------------
 * Startup scrub chain
 * ------------------------------------------------------------------------- */

void
ec_scrub_free_resources(struct ec_scrub_ctx *sctx)
{
	ec_free_channels_and_buffers(sctx->scrub_chans, sctx->chunk_bufs, sctx->ec->n);
}

/*
 * Called when all dirty regions have been scrubbed. Persists the cleared
 * WIB (fire-and-forget), frees resources, and clears ec->scrub_ctx so
 * RMW writes are no longer gated.
 */
static void
ec_scrub_finish(struct ec_scrub_ctx *sctx)
{
	struct ec_bdev *ec = sctx->ec;

	if (sctx->poller) {
		spdk_poller_unregister(&sctx->poller);
	}

	{
		uint64_t ticks_per_second = spdk_get_ticks_hz();
		uint64_t elapsed_seconds  = ticks_per_second != 0 ?
			(spdk_get_ticks() - sctx->heartbeat.start_ticks) / ticks_per_second : 0;

		SPDK_NOTICELOG("EC bdev %s: startup scrub complete: "
			       "%" PRIu64 "/%u dirty region(s), "
			       "%" PRIu64 " stripes scrubbed in %" PRIu64 "s; "
			       "WIB now clean\n",
			       ec->bdev.name,
			       sctx->regions_scrubbed,
			       sctx->total_dirty_regions,
			       sctx->stripes_scrubbed,
			       elapsed_seconds);
	}

	ec_rmw_backpressure_end(ec, "scrub finished");

	/*
	 * Persist the cleared bitmap (all region bits are now cleared in
	 * memory). Fire-and-forget: if this write fails, the next startup
	 * will re-scrub the same regions -- safe but not optimal.
	 */
	if (!ec->wib_persist_in_flight) {
		ec_wib_persist(ec, NULL, NULL);
	}

	ec_scrub_free_resources(sctx);
	ec->scrub_ctx = NULL;
	free(sctx);
}

/* Emit a scrub-progress NOTICE per the EC_BG_HEARTBEAT_* policy.
 * Regions span EC_WIB_REGION_STRIPES stripes, so firings are naturally
 * infrequent. */
static void
ec_scrub_heartbeat(struct ec_scrub_ctx *sctx)
{
	struct ec_bg_heartbeat_decision decision;

	decision = ec_heartbeat_should_fire(&sctx->heartbeat,
					    sctx->regions_scrubbed,
					    sctx->total_dirty_regions);
	if (!decision.fire) {
		return;
	}

	SPDK_NOTICELOG("EC bdev %s: scrub progress %u%% "
		       "(%" PRIu64 "/%u regions, %" PRIu64 " stripes scrubbed, "
		       "%" PRIu64 "s elapsed, ~%" PRIu64 "s remaining)\n",
		       sctx->ec->bdev.name, decision.percent,
		       sctx->regions_scrubbed, sctx->total_dirty_regions,
		       sctx->stripes_scrubbed,
		       decision.elapsed_seconds, decision.remaining_seconds);
}

/*
 * Move to the next dirty region. If none remain, call ec_scrub_finish.
 */
static void
ec_scrub_move_to_next_region(struct ec_scrub_ctx *sctx)
{
	struct ec_bdev *ec = sctx->ec;
	uint32_t        region;

	for (region = sctx->current_region + 1; region < ec->wib_num_regions; region++) {
		if (ec_wib_region_is_dirty(ec, region)) {
			sctx->current_region   = region;
			sctx->current_stripe   = (uint64_t)region * EC_WIB_REGION_STRIPES;
			sctx->region_end_stripe = spdk_min(
				sctx->current_stripe + EC_WIB_REGION_STRIPES,
				ec->num_stripes);
			return;
		}
	}

	/* No more dirty regions */
	ec_scrub_finish(sctx);
}

/*
 * Called for each parity write submitted during scrub.
 */
static void
ec_scrub_write_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_scrub_ctx *sctx = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		sctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
		SPDK_WARNLOG("EC bdev %s: scrub parity write failed at "
			     "stripe %" PRIu64 "\n",
			     sctx->ec->bdev.name, sctx->current_stripe);
	}

	sctx->writes_remaining--;
	if (sctx->writes_remaining != 0) {
		return;
	}

	/*
	 * Advance regardless of outcome. On write failure the region bit
	 * stays set, so the next startup re-scrubs this stripe; only count
	 * it as scrubbed on success.
	 */
	sctx->current_stripe++;
	if (sctx->io_status == SPDK_BDEV_IO_STATUS_SUCCESS) {
		sctx->stripes_scrubbed++;
	}

	sctx->io_in_flight = false;
}

/*
 * All k reads for the current stripe completed. Re-encode parity and
 * write back to all writable parity slots.
 */
static void
ec_scrub_reads_done(struct ec_scrub_ctx *sctx)
{
	struct ec_bdev  *ec         = sctx->ec;
	uint64_t         disk_lba   = ec_stripe_base_lba(ec, sctx->current_stripe);
	uint32_t         i;
	uint32_t         writable_parity = 0;
	int              rc;

	/*
	 * Reconstruct any unreadable data chunks first, then re-encode all m
	 * parity chunks into chunk_bufs[k..n-1]. Shared with the rebuild
	 * parity branch -- both must guarantee every data slot holds the
	 * correct payload before encoding parity, or the parity write below
	 * lands corrupt bytes on disk.
	 */
	rc = ec_reencode_parity_from_chunk_bufs(ec, sctx->chunk_bufs,
						sctx->chunk_bytes);
	if (rc != 0) {
		SPDK_WARNLOG("EC bdev %s: scrub -- multi-data reconstruction "
			     "failed at stripe %" PRIu64 " (rc=%d); skipping\n",
			     ec->bdev.name, sctx->current_stripe, rc);
		sctx->current_stripe++;
		sctx->io_in_flight = false;
		return;
	}

	/* Count writable parity slots */
	for (i = 0; i < ec->m; i++) {
		if (ec_slot_is_writable(ec, ec->k + i)) {
			writable_parity++;
		}
	}

	if (writable_parity == 0) {
		/* No parity to write -- stripe is fine, advance */
		sctx->current_stripe++;
		sctx->stripes_scrubbed++;
		sctx->io_in_flight = false;
		return;
	}

	sctx->writes_remaining = writable_parity;
	sctx->io_status        = SPDK_BDEV_IO_STATUS_SUCCESS;

	for (i = 0; i < ec->m; i++) {
		uint32_t pslot = ec->k + i;

		if (!ec_slot_is_writable(ec, pslot)) {
			continue;
		}
		if (!sctx->scrub_chans[pslot]) {
			sctx->writes_remaining--;
			sctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
			continue;
		}

		rc = spdk_bdev_writev_blocks(ec->descs[pslot],
					     sctx->scrub_chans[pslot],
					     &sctx->chunk_iovs[pslot], 1,
					     disk_lba,
					     ec->strip_size,
					     ec_scrub_write_cb,
					     sctx);
		if (rc != 0) {
			SPDK_WARNLOG("EC bdev %s: scrub parity write submit "
				     "failed for slot %u stripe %" PRIu64 " (rc=%d)\n",
				     ec->bdev.name, pslot,
				     sctx->current_stripe, rc);
			sctx->writes_remaining--;
			sctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}

	if (sctx->writes_remaining == 0) {
		sctx->current_stripe++;
		sctx->io_in_flight = false;
	}
}

static void
ec_scrub_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_scrub_ctx *sctx = cb_arg;

	spdk_bdev_free_io(bdev_io);

	if (!success) {
		sctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	sctx->reads_remaining--;
	if (sctx->reads_remaining != 0) {
		return;
	}

	if (sctx->io_status != SPDK_BDEV_IO_STATUS_SUCCESS) {
		SPDK_WARNLOG("EC bdev %s: scrub read failed at stripe %" PRIu64 " "
			     "-- skipping\n",
			     sctx->ec->bdev.name, sctx->current_stripe);
		sctx->current_stripe++;
		sctx->io_in_flight = false;
		return;
	}

	ec_scrub_reads_done(sctx);
}

/*
 * Submit k reads for the current stripe from readable disks.
 * Uses sctx->scrub_chans[] -- dedicated per-disk channels opened at scrub
 * start, covering all n slots (both data and parity disks).
 */
static void
ec_scrub_submit_reads(struct ec_scrub_ctx *sctx)
{
	struct ec_bdev *ec       = sctx->ec;
	uint64_t        disk_lba = ec_stripe_base_lba(ec, sctx->current_stripe);
	uint32_t        disk;
	uint32_t        reads_submitted = 0;
	int             rc;

	sctx->io_status       = SPDK_BDEV_IO_STATUS_SUCCESS;
	sctx->reads_remaining = 0;
	sctx->io_in_flight    = true;

	for (disk = 0; disk < ec->n && reads_submitted < ec->k; disk++) {
		if (!ec_slot_is_readable(ec, disk)) {
			continue;
		}
		if (!sctx->scrub_chans[disk]) {
			continue;   /* channel not available -- treat as unreadable */
		}

		sctx->reads_remaining++;

		rc = spdk_bdev_readv_blocks(ec->descs[disk],
					    sctx->scrub_chans[disk],
					    &sctx->chunk_iovs[disk], 1,
					    disk_lba,
					    ec->strip_size,
					    ec_scrub_read_cb,
					    sctx);
		if (rc != 0) {
			sctx->reads_remaining--;
			sctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
			SPDK_WARNLOG("EC bdev %s: scrub read submit failed "
				     "for disk %u stripe %" PRIu64 " (rc=%d)\n",
				     ec->bdev.name, disk,
				     sctx->current_stripe, rc);
		} else {
			reads_submitted++;
		}
	}

	if (reads_submitted < ec->k) {
		SPDK_WARNLOG("EC bdev %s: scrub stripe %" PRIu64 " -- only %u/%u "
			     "reads submitted; skipping\n",
			     ec->bdev.name, sctx->current_stripe,
			     reads_submitted, ec->k);
		if (sctx->reads_remaining == 0) {
			sctx->current_stripe++;
			sctx->io_in_flight = false;
		} else {
			sctx->io_status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}
}

/*
 * State machine per tick:
 *   io_in_flight              -> outstanding I/O; return BUSY
 *   current_stripe >= region_end_stripe -> clear region bit; advance region
 *   stripe is dirty in stripe_dirty_map -> skip; foreground writer or rebuild
 *                                          owns this stripe and is updating
 *                                          parity correctly
 *   else                      -> submit reads for current stripe
 */
static int
ec_scrub_poller_cb(void *arg)
{
	struct ec_scrub_ctx *sctx = arg;
	struct ec_bdev      *ec   = sctx->ec;

	if (sctx->io_in_flight) {
		return SPDK_POLLER_BUSY;
	}

	/*
	 * Before submitting the next stripe, run the deferrals that parked behind
	 * the drained I/O: a failed slot's channel release, and -- if a delete or
	 * shutdown started -- the device teardown itself. Teardown frees sctx, so
	 * return immediately after.
	 *
	 * This sits above both the next-stripe submit and the region-transition WIB
	 * persist below, so a destructing scrub launches no fresh I/O; keep it ahead
	 * of both if the poller steps are ever reordered.
	 */
	ec_drain_deferred_slot_releases(ec);
	if (ec->destructing) {
		ec_drain_deferred_unregister(ec);
		return SPDK_POLLER_BUSY;
	}

	/* Current region complete */
	if (sctx->current_stripe >= sctx->region_end_stripe) {
		/*
		 * Clear the region bit now that all its stripes have been
		 * scrubbed. The next persist (from idle poller or finish)
		 * will write the cleared state to disk.
		 */
		ec_wib_region_clear_dirty(ec, sctx->current_region);
		sctx->regions_scrubbed++;

		ec_scrub_heartbeat(sctx);

		ec_scrub_move_to_next_region(sctx);
		/*
		 * ec_scrub_move_to_next_region either updates current_region and
		 * returns (poller continues), or calls ec_scrub_finish which
		 * unregisters the poller.
		 */
		return SPDK_POLLER_BUSY;
	}

	/*
	 * Skip unmapped stripes. An unmapped stripe reads as zeros without
	 * consulting parity, so its on-disk parity is never used and there is
	 * nothing to re-encode. The region bit is still cleared at
	 * region_end_stripe (see the region-consistency note below).
	 */
	if (ec->stripe_unmapped_map != NULL &&
	    ec_stripe_is_unmapped(ec, sctx->current_stripe)) {
		sctx->current_stripe++;
		sctx->stripes_scrubbed++;
		return SPDK_POLLER_BUSY;
	}

	/*
	 * Stripe-busy interlock.
	 *
	 * If another actor (rebuild, full-stripe write, RMW writeback after
	 * the scrub-guard window, or UNMAP) holds the stripe-dirty claim on
	 * this stripe, skip it. The claimant is responsible for writing a
	 * consistent parity for the stripe -- scrub's re-encode would race
	 * the claimant's parity write and could land stale parity computed
	 * from a partial view of the data.
	 *
	 * Skipping is safe even though we still clear the region bit at the
	 * region boundary: the claimant's write completes with up-to-date
	 * parity. The WIB dirty bit on this region only signals that some
	 * stripe in the region was being modified at crash time; once the
	 * region has been walked end-to-end (every stripe either re-encoded
	 * by scrub, owned by an in-flight writer, or unmapped), the region is
	 * by definition consistent.
	 */
	if (ec_stripe_is_dirty(ec, sctx->current_stripe)) {
		sctx->current_stripe++;
		return SPDK_POLLER_BUSY;
	}

	ec_scrub_submit_reads(sctx);
	return SPDK_POLLER_BUSY;
}

/*
 * Allocates the scrub context, finds the first dirty region, and registers
 * the scrub poller. Called from ec_bdev_create_finalize after the bdev is
 * registered if ec_wib_load_async found any dirty regions, and from
 * ec_rebuild_finish when a boot-deferred scrub can finally run.
 *
 * The scrub runs in the background; the bdev accepts reads immediately.
 * RMW writes to a stripe whose region has not yet been scrubbed are
 * requeued (NOMEM) via the guard in ec_submit_rmw_write.
 */
int
ec_bdev_start_scrub(struct ec_bdev *ec)
{
	struct ec_scrub_ctx *sctx;
	uint32_t             first_dirty = ec->wib_num_regions;
	uint32_t             region, i;

	/*
	 * Deferred-scrub guard no longer applies once an active scrub is
	 * being installed. Close out any deferred-scrub backpressure episode
	 * now; the active-scrub guard will open a new one when it fires.
	 */
	ec_rmw_backpressure_end(ec, "scrub starting");

	/*
	 * Block scrub while a resize is in progress.
	 *
	 * Resize updates geometry (blockcnt, num_stripes) and reallocates
	 * the dirty bitmap and WIB arrays. If the scrub runs concurrently
	 * it could access stale pointers or out-of-bounds indices.
	 */
	if (ec->resize_ctx != NULL) {
		SPDK_WARNLOG("EC bdev %s: scrub deferred -- resize "
			     "in progress\n", ec->bdev.name);
		return 0;
	}

	/*
	 * Correctness pre-condition: all k data disks must be NORMAL.
	 *
	 * ec_scrub_reads_done re-encodes parity from chunk_bufs[0..k-1],
	 * which are filled by ec_scrub_submit_reads iterating over readable
	 * slots in physical-slot order. If data disk d is FAILED, slot d is
	 * skipped and chunk_bufs[d] stays zero-filled -- ec_encode_data then
	 * treats slot d as zero rather than its actual data, producing wrong
	 * parity for every scrubbed stripe. The corruption is latent: it only
	 * surfaces when a different disk later fails and reconstruction uses
	 * the incorrect parity.
	 *
	 * Safe course of action: skip the scrub entirely when any data slot
	 * is not NORMAL. Leave all dirty region bits set. On next startup
	 * (after the rebuild has restored the failed disk to NORMAL), the
	 * scrub will run correctly. This is conservative but always safe.
	 */
	for (i = 0; i < ec->k; i++) {
		if (ec->base_states[i] != EC_BASE_STATE_NORMAL) {
			SPDK_WARNLOG("EC bdev %s: startup scrub deferred -- "
				     "data slot %u is not NORMAL (state=%d). "
				     "Scrub will run after rebuild completes.\n",
				     ec->bdev.name, i, (int)ec->base_states[i]);
			return 0;
		}
	}

	/* Find first dirty region and count total dirty regions */
	uint32_t dirty_count = 0;
	for (region = 0; region < ec->wib_num_regions; region++) {
		if (ec_wib_region_is_dirty(ec, region)) {
			if (first_dirty == ec->wib_num_regions) {
				first_dirty = region;
			}
			dirty_count++;
		}
	}
	if (first_dirty == ec->wib_num_regions) {
		return 0;   /* Nothing to scrub */
	}

	sctx = calloc(1, sizeof(*sctx));
	if (!sctx) {
		return -ENOMEM;
	}

	sctx->ec                  = ec;
	sctx->current_region      = first_dirty;
	sctx->current_stripe      = (uint64_t)first_dirty * EC_WIB_REGION_STRIPES;
	sctx->region_end_stripe = spdk_min(
		sctx->current_stripe + EC_WIB_REGION_STRIPES,
		ec->num_stripes);
	sctx->chunk_bytes = ec->strip_size * ec->bdev.blocklen;
	sctx->io_in_flight = false;
	sctx->total_dirty_regions = dirty_count;
	sctx->heartbeat.start_ticks = spdk_get_ticks();
	sctx->heartbeat.last_heartbeat_ticks = sctx->heartbeat.start_ticks;
	sctx->heartbeat.next_heartbeat_percent = EC_BG_HEARTBEAT_PERCENT_STEP;

	/* Open dedicated I/O channels for all n disks */
	for (i = 0; i < ec->n; i++) {
		if (!ec->descs[i]) {
			sctx->scrub_chans[i] = NULL;
			continue;
		}
		sctx->scrub_chans[i] = spdk_bdev_get_io_channel(ec->descs[i]);
		if (!sctx->scrub_chans[i]) {
			SPDK_ERRLOG("EC bdev %s: OOM for scrub channel "
				    "slot %u\n", ec->bdev.name, i);
			ec_scrub_free_resources(sctx);
			free(sctx);
			return -ENOMEM;
		}
	}

	/* Allocate DMA buffers for all n slots */
	for (i = 0; i < ec->n; i++) {
		sctx->chunk_bufs[i] = spdk_dma_zmalloc(sctx->chunk_bytes,
						        EC_DMA_ALIGN, NULL);
		if (!sctx->chunk_bufs[i]) {
			SPDK_ERRLOG("EC bdev %s: OOM for scrub chunk buf "
				    "slot %u\n", ec->bdev.name, i);
			ec_scrub_free_resources(sctx);
			free(sctx);
			return -ENOMEM;
		}
		sctx->chunk_iovs[i].iov_base = sctx->chunk_bufs[i];
		sctx->chunk_iovs[i].iov_len  = sctx->chunk_bytes;
	}

	ec->scrub_ctx = sctx;

	sctx->poller = spdk_poller_register(ec_scrub_poller_cb, sctx,
					    EC_BG_POLL_PERIOD_US);
	if (!sctx->poller) {
		ec_scrub_free_resources(sctx);
		ec->scrub_ctx = NULL;
		free(sctx);
		return -ENOMEM;
	}

	SPDK_NOTICELOG("EC bdev %s: startup scrub started -- "
		       "first dirty region %u\n",
		       ec->bdev.name, first_dirty);
	return 0;
}

/*
 * Returns -ENODEV if the named EC bdev does not exist.
 * Returns -ENOENT if no scrub is currently in progress.
 * Returns 0 and fills *out on success.
 */
int
ec_bdev_get_scrub_progress(const char *ec_name,
			   struct ec_scrub_progress *out)
{
	struct ec_bdev *ec = ec_bdev_find(ec_name);

	if (!ec) {
		return -ENODEV;
	}
	if (!ec->scrub_ctx) {
		return -ENOENT;
	}

	out->current_region      = ec->scrub_ctx->current_region;
	out->num_regions         = ec->wib_num_regions;
	out->total_dirty_regions = ec->scrub_ctx->total_dirty_regions;
	out->current_stripe      = ec->scrub_ctx->current_stripe;
	out->stripes_scrubbed    = ec->scrub_ctx->stripes_scrubbed;
	out->regions_scrubbed    = ec->scrub_ctx->regions_scrubbed;

	return 0;
}
