/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_resize.c -- in-place capacity expansion for EC bdevs.
 *
 * Triggered after the underlying base bdevs have been resized externally.
 * k, m, and the encode tables are unchanged; only the EC bdev's blockcnt,
 * num_stripes, WIB region count, and per-stripe bitmaps change.
 *
 * The WIB and bitmap regions live at fixed front offsets that never move
 * on resize. Resize is mostly in-memory arithmetic: its only metadata write
 * is re-stamping the unmapped bitmap at the new geometry. There is no
 * on-disk relocation and no rollback path; an OOM mid-realloc clamps
 * geometry back to the old size in place. The async control flow and
 * per-step detail are documented at ec_bdev_resize / ec_resize_quiesce_cb.
 */

#include "bdev_ec_internal.h"
#include "spdk/log.h"

/* =========================================================================
 * In-place resize (same k/m, bigger disks)
 * ========================================================================= */

/*
 * Terminal handler -- clean up context, unquiesce, call user callback.
 */
static void
ec_resize_finish(struct ec_resize_ctx *ctx, int rc)
{
	struct ec_bdev *ec = ctx->ec;
	uint64_t quiesced_blockcnt = ctx->old_blockcnt;
	uint64_t committed_blockcnt = ec->num_stripes * ec->stripe_blocks;
	int notify_rc;

	ec->resize_ctx = NULL;

	/*
	 * Commit the final size to the bdev layer now that the geometry is
	 * settled. blockcnt still holds the old value here, so notify fires
	 * the resize event and updates blockcnt on a successful grow; it is a
	 * no-op when an OOM clamp left num_stripes (and thus
	 * committed_blockcnt) at the old geometry.
	 */
	notify_rc = spdk_bdev_notify_blockcnt_change(&ec->bdev, committed_blockcnt);
	if (notify_rc != 0) {
		SPDK_ERRLOG("EC bdev %s: resize -- blockcnt notify failed (rc=%d)\n",
			    ec->bdev.name, notify_rc);
	}

	/*
	 * Unquiesce the exact range we quiesced. We pass quiesced_blockcnt
	 * (the old size) rather than calling plain spdk_bdev_unquiesce,
	 * because after a successful resize bdev.blockcnt has already grown
	 * and would no longer match the quiesced range.
	 *
	 * We don't check the return value on purpose. If the earlier quiesce
	 * succeeded, this unquiesce can't fail -- we're releasing the exact
	 * range we're still holding. The case that does return an error is
	 * when the quiesce itself failed and we still ended up here (via
	 * ec_resize_quiesce_cb): nothing was ever quiesced, so no I/O is stuck,
	 * and SPDK already logs it. Either way there's nothing for us to do
	 * with the return value.
	 */
	spdk_bdev_unquiesce_range(&ec->bdev, &ec_if,
				  0, quiesced_blockcnt, NULL, NULL);

	ctx->cb_fn(ctx->cb_arg, rc);
	free(ctx);
}

/*
 * Completion for the post-resize unmapped-bitmap re-stamp (fired from
 * ec_resize_realloc_stripe_bitmaps below).
 *
 * On a failed re-persist we roll the geometry back to the old size. We do
 * this for clean failure, not for safety: the old on-disk copy still loads
 * on restart (ec_bitmap_validate_buf accepts a smaller copy,
 * ec_bitmap_apply_buf zero-fills the new stripes), so no UNMAP is lost
 * either way. The rollback just keeps a failed resize invisible --
 * ec_resize_finish publishes the new blockcnt only after this callback --
 * so the caller sees "nothing changed" and can retry.
 *
 * The larger WIB and stripe-map allocations stay; the extra space is
 * unused until the next resize or teardown, like the OOM-clamp path in
 * ec_resize_realloc_stripe_bitmaps.
 */
static void
ec_resize_bitmap_persist_done(void *arg, int rc)
{
	struct ec_resize_ctx *ctx = arg;
	struct ec_bdev       *ec  = ctx->ec;

	if (rc != 0) {
		uint64_t old_num_stripes = ctx->old_blockcnt / ec->stripe_blocks;

		SPDK_ERRLOG("EC bdev %s: resize -- unmapped-bitmap re-persist "
			    "failed (rc=%d); rolling back to old geometry\n",
			    ec->bdev.name, rc);
		ec->num_stripes     = old_num_stripes;
		ec->wib_num_regions = (uint32_t)
			((old_num_stripes + EC_WIB_REGION_STRIPES - 1) /
			 EC_WIB_REGION_STRIPES);
		ec_resize_finish(ctx, rc);
		return;
	}
	ec_resize_finish(ctx, 0);
}

/*
 * Reallocate the per-stripe in-memory bitmaps to match the new geometry:
 *
 *   stripe_dirty_map      -- transient RMW interlock; cleared by quiesce,
 *                            so the old content is discarded.
 *   stripe_unmapped_map   -- persistent allocation state; old content is
 *                            preserved via memcpy. Newly added stripes
 *                            initialise to mapped (bit=0) since the
 *                            underlying lvol/blob extension reads back as
 *                            zero, so a read of a new stripe returns
 *                            zeros via the normal data path. Marking new
 *                            stripes UNMAPPED as a read-side optimisation
 *                            is a future improvement; for crash-safety
 *                            either initialisation is correct.
 *
 * Both reallocations are treated atomically: either both succeed and
 * geometry expands, or neither does and num_stripes clamps back to the
 * old geometry. A partial expansion would leave one map sized for new and
 * the other for old, with OOB heap access on the very next I/O.
 *
 * Without the unmapped-map realloc here, ec_stripe_{set,clear,is}_unmapped
 * indexes past the end of the old allocation as soon as
 * spdk_bdev_unquiesce_range below resumes I/O on the expanded address
 * space, corrupting the heap. libc detects the corruption on the next
 * free() ("double free or corruption (out)") and aborts the process.
 *
 * Called from ec_resize_quiesce_cb after the WIB region arrays have been
 * reallocated. Finishes the resize on success or after an OOM clamp.
 */
static void
ec_resize_realloc_stripe_bitmaps(struct ec_resize_ctx *ctx)
{
	struct ec_bdev *ec               = ctx->ec;
	uint64_t        new_num_stripes  = ec->num_stripes;
	uint64_t        old_num_stripes  = ctx->old_blockcnt / ec->stripe_blocks;
	uint64_t        new_map_words    = EC_BITMAP_WORDS(new_num_stripes);
	uint64_t        old_map_words    = EC_BITMAP_WORDS(old_num_stripes);
	uint64_t       *new_dirty_map;
	uint64_t       *new_unmapped_map;

	new_dirty_map    = calloc(new_map_words, sizeof(uint64_t));
	new_unmapped_map = calloc(new_map_words, sizeof(uint64_t));

	if (new_dirty_map && new_unmapped_map) {
		/*
		 * Preserve persistent unmapped bits across the resize. New
		 * stripes [old_num_stripes, new_num_stripes) are zeroed by
		 * calloc (mapped). dirty_map starts fresh -- quiesce
		 * guarantees no RMW is in flight, so old content is
		 * discardable.
		 */
		memcpy(new_unmapped_map, ec->stripe_unmapped_map,
		       old_map_words * sizeof(uint64_t));

		free(ec->stripe_dirty_map);
		ec->stripe_dirty_map = new_dirty_map;

		free(ec->stripe_unmapped_map);
		ec->stripe_unmapped_map = new_unmapped_map;
	} else {
		/*
		 * OOM on either map -- clamp geometry back to what the
		 * old allocations cover. Partial expansion would set us
		 * up for heap OOB on the next I/O to a stripe past
		 * old_num_stripes.
		 */
		free(new_dirty_map);
		free(new_unmapped_map);

		SPDK_WARNLOG("EC bdev %s: resize -- OOM for per-stripe "
			     "bitmaps; clamping to old geometry\n",
			     ec->bdev.name);

		ec->num_stripes = old_num_stripes;

		/*
		 * Step 2 already grew the WIB arrays to the new geometry.
		 * Clamp wib_num_regions back to match the rolled-back
		 * num_stripes so WIB status and the next persist report the
		 * correct region count; the larger WIB allocations simply
		 * keep unused tail capacity until the next resize or teardown.
		 *
		 * stripe_dirty_map and stripe_unmapped_map are left as-is:
		 * the quiesce drained every in-flight RMW / full-stripe write /
		 * UNMAP / rebuild before this path runs, so stripe_dirty_map is
		 * already zero across the old range and the unmapped bitmap
		 * content remains valid at the clamped (old) size.
		 */
		ec->wib_num_regions = (uint32_t)
			((old_num_stripes + EC_WIB_REGION_STRIPES - 1) /
			 EC_WIB_REGION_STRIPES);
	}

	if (ec->num_stripes != old_num_stripes) {
		int rc;

		/*
		 * Re-stamp the unmapped bitmap at the new num_stripes. This is an
		 * optimization, not a correctness requirement: an old-geometry copy
		 * still loads after a restart (validate accepts a smaller copy, apply
		 * zero-extends the tail). The re-stamp just restores both copies to
		 * the new size now instead of at the next persist.
		 */
		rc = ec_bitmap_persist_async(ec, ec->stripe_unmapped_map,
					     NULL, NULL,
					     ec_resize_bitmap_persist_done, ctx);
		if (rc == 0) {
			return;  /* resize completes from ec_resize_bitmap_persist_done */
		}
		SPDK_ERRLOG("EC bdev %s: resize -- could not start unmapped-bitmap "
			    "re-persist (rc=%d); rolling back to old geometry\n",
			    ec->bdev.name, rc);
		ec->num_stripes     = old_num_stripes;
		ec->wib_num_regions = (uint32_t)
			((old_num_stripes + EC_WIB_REGION_STRIPES - 1) /
			 EC_WIB_REGION_STRIPES);
		ec_resize_finish(ctx, rc);
		return;
	}

	/*
	 * Did not grow: ec_bdev_resize rejects no-op requests with -EALREADY
	 * before quiesce, so reaching here means an OOM clamp fired in
	 * ec_resize_realloc_wib_arrays or in the per-stripe realloc above.
	 * Report -ENOMEM so the caller can distinguish a clamped no-grow from a
	 * real resize. ec_resize_finish still unquiesces; the
	 * spdk_bdev_notify_blockcnt_change call inside it is a no-op because
	 * committed_blockcnt equals the old value after clamp.
	 */
	ec_resize_finish(ctx, -ENOMEM);
}

/*
 * Reallocate the WIB region arrays for the new (larger) geometry. On OOM the
 * new arrays are discarded and num_stripes is clamped to what the existing WIB
 * coverage supports. wib_buf itself is not reallocated (it is one strip,
 * geometry-invariant). The WIB is reset to clean on grow (the old dirty bits
 * are intentionally not carried forward) -- see the realloc below for why.
 */
static void
ec_resize_realloc_wib_arrays(struct ec_resize_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;
	uint32_t  new_wib_regions = (uint32_t)
		((ec->num_stripes + EC_WIB_REGION_STRIPES - 1) /
		 EC_WIB_REGION_STRIPES);
	uint32_t  new_wib_words   = EC_BITMAP_WORDS(new_wib_regions);

	uint64_t *new_region_map      = calloc(new_wib_words, sizeof(uint64_t));
	uint32_t *new_region_inflight = calloc(new_wib_regions, sizeof(uint32_t));
	uint64_t *new_region_dirty_ts = calloc(new_wib_regions, sizeof(uint64_t));

	if (new_region_map && new_region_inflight && new_region_dirty_ts) {
		/*
		 * Reset the in-memory WIB to clean on grow -- do NOT carry the old
		 * dirty bits forward (new_region_map is calloc'd, already zero).
		 *
		 * The on-disk WIB stays in the old geometry until the next persist,
		 * so the next restart rejects it (num_regions mismatch in
		 * ec_wib_validate_buf) and loads all-clean. Carrying the bits
		 * forward in memory would leave a region marked dirty with no
		 * loadable on-disk record; since the RMW / full-stripe paths skip
		 * the WIB persist for an already-dirty region, a post-resize write
		 * into it would issue data with no recoverable write-intent -- a
		 * write-hole if it tears before the next persist re-stamps the WIB.
		 * Resetting to clean matches what a restart reconstructs and forces
		 * the next write into each region to re-persist its intent in the
		 * new geometry. Dropping the old bits is safe because resize
		 * quiesced I/O and runs only on a healthy array (guards reject an
		 * in-progress scrub or any failed disk), so every set bit was an
		 * already-completed write with consistent parity -- nothing to scrub.
		 */
		free(ec->wib_region_map);
		free(ec->wib_region_inflight);
		free(ec->wib_region_dirty_ticks);

		ec->wib_region_map      = new_region_map;
		ec->wib_region_inflight = new_region_inflight;
		ec->wib_region_dirty_ticks = new_region_dirty_ts;
		ec->wib_num_regions     = new_wib_regions;
	} else {
		/*
		 * OOM -- free partial allocations and clamp geometry back to
		 * the exact old size, mirroring the per-stripe OOM path. The
		 * WIB arrays remain at their old (smaller) size, and
		 * wib_num_regions is unchanged (still the old value), which
		 * matches the restored num_stripes.
		 */
		free(new_region_map);
		free(new_region_inflight);
		free(new_region_dirty_ts);

		SPDK_WARNLOG("EC bdev %s: resize -- OOM for WIB "
			     "arrays; clamping capacity to existing "
			     "WIB coverage\n", ec->bdev.name);

		ec->num_stripes = ctx->old_blockcnt / ec->stripe_blocks;
	}
}

/*
 * Called after the EC bdev has been quiesced. Performs the actual resize:
 *   1. Update num_stripes. The bdev blockcnt is committed last, in
 *      ec_resize_finish, once the reallocs below have succeeded.
 *   2. Reallocate the WIB region arrays. WIB position on disk is a
 *      function of strip_size + bitmap reservation (both fixed-max), so no
 *      relocation is needed -- on a future persist, the idle poller writes
 *      the new region map to the same on-disk LBAs.
 *   3. Reallocate per-stripe bitmaps (ec_resize_realloc_stripe_bitmaps) and
 *      finish.
 *
 * Either reallocation may OOM. Both clamp capacity in place without an
 * async unwind: the WIB realloc clamps to the existing WIB coverage; the
 * per-stripe realloc clamps num_stripes back to the old geometry. A
 * partial expansion would leave one map sized for new and the other for
 * old, with OOB heap access on the very next I/O.
 */
static void
ec_resize_quiesce_cb(void *cb_arg, int status)
{
	struct ec_resize_ctx *ctx = cb_arg;
	struct ec_bdev       *ec  = ctx->ec;
	uint64_t              new_blockcnt    = ctx->new_blockcnt;
	uint64_t              new_num_stripes = ctx->new_num_stripes;

	if (status != 0) {
		SPDK_ERRLOG("EC bdev %s: resize quiesce failed (rc=%d)\n",
			    ec->bdev.name, status);
		ec_resize_finish(ctx, status);
		return;
	}

	/*
	 * Step 1: Update num_stripes. Must happen before the WIB realloc so
	 * that the new wib_num_regions is derived from new_num_stripes.
	 *
	 * The bdev blockcnt is deliberately NOT committed here. It is deferred
	 * to ec_resize_finish (via spdk_bdev_notify_blockcnt_change) so the
	 * resize event fires to consumers only after the fallible WIB / bitmap
	 * reallocs have succeeded, and so an OOM rollback never has to shrink
	 * blockcnt back -- which the open-descriptor guard would reject.
	 */
	ec->num_stripes = new_num_stripes;

	SPDK_NOTICELOG("EC bdev %s: resize -- blockcnt %" PRIu64 " -> %" PRIu64 ", "
		       "num_stripes %" PRIu64 "\n",
		       ec->bdev.name, ctx->old_blockcnt, new_blockcnt,
		       new_num_stripes);

	/*
	 * Step 2: Grow the WIB region arrays for the larger volume. More
	 * stripes means more WIB regions, so region_map / region_inflight /
	 * region_dirty_ticks are reallocated to the new region count and reset
	 * to all-clean -- the old dirty bits are intentionally not carried
	 * forward (see ec_resize_realloc_wib_arrays for why).
	 * On OOM the new arrays are dropped and num_stripes is clamped back
	 * to what the old arrays already cover.
	 *
	 * This does not touch wib_buf: it is one strip regardless of volume
	 * size, which is what lets resize run without interlocking the WIB
	 * idle poller (see the concurrency note atop ec_bdev_resize).
	 */
	ec_resize_realloc_wib_arrays(ctx);

	/*
	 * Step 3: Reallocate per-stripe bitmaps. Terminates the resize
	 * (calls ec_resize_finish).
	 */
	ec_resize_realloc_stripe_bitmaps(ctx);
}

/*
 * Expands the EC bdev in-place after the underlying base bdevs have been
 * resized externally. k, m, and encode tables are unchanged.
 *
 * Returns 0 and kicks off the async resize (cb_fn called on completion).
 * Returns negative errno on validation failure.
 */
int
ec_bdev_resize(const char *ec_name,
	       ec_resize_cb_fn cb_fn, void *cb_arg)
{
	struct ec_bdev       *ec;
	struct ec_resize_ctx *ctx;
	uint64_t              min_blockcnt = UINT64_MAX;
	uint64_t              old_effective, new_effective;
	uint64_t              new_total_physical_stripes;
	uint64_t              new_num_stripes, new_blockcnt;
	uint32_t              i;
	int                   quiesce_rc;

	/* Step 1: Find EC bdev */
	ec = ec_bdev_find(ec_name);
	if (!ec) {
		return -ENODEV;
	}

	/* Step 2: Guard conditions */
	if (ec->rebuild_ctx != NULL ||
	    ec->resize_ctx != NULL) {
		SPDK_ERRLOG("EC bdev %s: resize rejected -- "
			    "another operation in progress\n", ec_name);
		return -EBUSY;
	}
	if (ec->scrub_ctx != NULL) {
		SPDK_ERRLOG("EC bdev %s: resize rejected -- "
			    "scrub in progress\n", ec_name);
		return -EBUSY;
	}
	if (ec->failed_count != 0) {
		SPDK_ERRLOG("EC bdev %s: resize rejected -- "
			    "%u failed disks\n", ec_name, ec->failed_count);
		return -EBUSY;
	}
	if (ec->offline) {
		return -EIO;
	}
	/*
	 * No interlock with the WIB idle poller is required. Resize does
	 * not reallocate wib_buf (size is geometry-invariant), so an
	 * in-flight persist's DMA reads remain valid throughout.
	 * ec_wib_fill_buf captures wib_region_map and wib_num_regions
	 * synchronously before submitting I/O; later reallocs in
	 * ec_resize_quiesce_cb do not affect the in-flight DMA. The
	 * persist's on-disk LBA is fixed (function of strip_size + bitmap
	 * reservation, both unchanged across resize).
	 */

	/* Step 3: Find minimum base bdev size. The failed_count != 0 guard
	 * above already excludes any non-NORMAL slot, so every descs[i] is
	 * non-NULL here. */
	for (i = 0; i < ec->n; i++) {
		struct spdk_bdev *base = spdk_bdev_desc_get_bdev(ec->descs[i]);

		if (base->blockcnt < min_blockcnt) {
			min_blockcnt = base->blockcnt;
		}
	}

	/*
	 * Step 4: Validate growth.
	 *
	 * Both metadata reservations (bitmap + WIB) are fixed-max and at the
	 * front of every disk; only the trailing user region grows on resize.
	 * new_num_stripes is the post-resize physical stripe count minus the
	 * same data_offset_stripes reservation that ec_compute_geometry carved
	 * out. old_effective and new_effective are both measured in user-region
	 * blocks, so the growth check below compares them in the same units.
	 */
	old_effective = ec->num_stripes * ec->strip_size;
	new_total_physical_stripes = min_blockcnt / ec->strip_size;

	if (new_total_physical_stripes <= ec->data_offset_stripes) {
		/*
		 * Hard geometry error, not a benign no-op: the disks are too
		 * small to hold even the front metadata reservation. Return
		 * -ERANGE (not -EALREADY) so the caller does not mistake it for
		 * the idempotent "have not grown" retry below.
		 */
		SPDK_ERRLOG("EC bdev %s: resize -- base bdevs too small to "
			    "hold the in-band bitmap reservation "
			    "(physical stripes=%" PRIu64 ", reservation=%" PRIu64 ")\n",
			    ec_name, new_total_physical_stripes,
			    ec->data_offset_stripes);
		return -ERANGE;
	}

	new_num_stripes = new_total_physical_stripes - ec->data_offset_stripes;
	new_effective   = new_num_stripes * ec->strip_size;

	if (new_effective <= old_effective) {
		SPDK_NOTICELOG("EC bdev %s: resize -- base bdevs have not "
			       "grown (old_effective=%" PRIu64 ", new_effective=%" PRIu64 ")\n",
			       ec_name, old_effective, new_effective);
		return -EALREADY;
	}

	/*
	 * The front reservations are sized for ec_max_num_stripes; growing past
	 * it would overrun the one-strip WIB buffer and the bitmap slots into
	 * user data. ec_compute_geometry rejects this at create via the
	 * WIB-fits-one-strip check, but resize computes geometry directly and
	 * must enforce the same ceiling.
	 */
	if (new_num_stripes > ec_max_num_stripes(ec)) {
		SPDK_ERRLOG("EC bdev %s: resize -- new stripe count %" PRIu64 " exceeds "
			    "the max %" PRIu64 " the front metadata was reserved for; "
			    "create with a larger strip_size_kb for volumes this large\n",
			    ec_name, new_num_stripes, ec_max_num_stripes(ec));
		return -ERANGE;
	}

	/* Step 5: Compute new geometry */
	new_blockcnt = new_num_stripes * ec->stripe_blocks;

	SPDK_NOTICELOG("EC bdev %s: resize requested -- blockcnt %" PRIu64 " -> %" PRIu64 ", "
		       "stripes %" PRIu64 " -> %" PRIu64 "; about to quiesce I/O\n",
		       ec_name, ec->bdev.blockcnt, new_blockcnt,
		       ec->num_stripes, new_num_stripes);

	/* Step 6: Allocate context */
	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		return -ENOMEM;
	}

	ctx->ec              = ec;
	ctx->cb_fn           = cb_fn;
	ctx->cb_arg          = cb_arg;
	ctx->new_blockcnt    = new_blockcnt;
	ctx->new_num_stripes = new_num_stripes;
	ctx->old_blockcnt    = ec->bdev.blockcnt;

	/* Step 7: Quiesce and proceed in callback */
	ec->resize_ctx = ctx;

	quiesce_rc = spdk_bdev_quiesce(&ec->bdev, &ec_if,
				ec_resize_quiesce_cb, ctx);
	if (quiesce_rc != 0) {
		SPDK_ERRLOG("EC bdev %s: resize -- failed to quiesce "
			    "(rc=%d)\n", ec_name, quiesce_rc);
		ec->resize_ctx = NULL;
		free(ctx);
		return quiesce_rc;
	}

	return 0;
}
