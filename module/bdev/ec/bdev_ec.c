/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * Implementation of the bdev_ec module's lifecycle, JSON config dump,
 * base-bdev event handling, and Reed-Solomon table management. See
 * bdev_ec_internal.h's THREADING MODEL block for the home / submitter
 * thread split that governs every cross-file callback in this file.
 */

#include "bdev_ec_internal.h"
#include "spdk/stdinc.h"
#include "spdk/bdev.h"
#include "spdk/bdev_module.h"
#include "spdk/env.h"
#include "spdk/log.h"
#include "spdk/string.h"
#include "spdk/util.h"
#include "spdk/thread.h"

/* ISA-L library header for Reed-Solomon erasure coding */
#include <isa-l/erasure_code.h>

struct ec_all_tailq g_ec_bdev_list = TAILQ_HEAD_INITIALIZER(g_ec_bdev_list);

/* Forward declaration; defined near the bottom of this file. Used by
 * _ec_bdev_create to wire the bdev fn_table at registration. */
static const struct spdk_bdev_fn_table g_ec_fn_table;

/* =========================================================================
 * Async descriptor cleanup context
 * ========================================================================= */

struct ec_base_bdev_cleanup_ctx {
	struct ec_bdev         *ec;
	uint32_t                slot;
	struct spdk_io_channel *reset_ch;
	bool                    quiesced;
};

static void ec_cleanup_close_descriptor(struct ec_base_bdev_cleanup_ctx *ctx);
static void ec_cleanup_channel_cb(struct spdk_io_channel_iter *i, int status);
static void ec_cleanup_reset_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg);
static void ec_cleanup_quiesce_cb(void *cb_arg, int status);

/* =========================================================================
 * Module init / ctx size
 * ========================================================================= */

static int
ec_bdev_init(void)
{
	return 0;
}

static int
ec_bdev_get_ctx_size(void)
{
	return sizeof(struct ec_bdev_io);
}

struct spdk_bdev_module ec_if = {
	.name         = "ec",
	.module_init  = ec_bdev_init,
	.get_ctx_size = ec_bdev_get_ctx_size,
	.async_init   = false,
	.async_fini   = false,
};

SPDK_BDEV_MODULE_REGISTER(ec, &ec_if)

/* =========================================================================
 * Helpers
 * ========================================================================= */

/*
 * ec_free_runtime_arrays -- safe-to-repeat release of the per-bdev state
 * arrays. Used both by ec_alloc_runtime_arrays on OOM unwind and by
 * ec_bdev_free on destruct. Every pointer is NULLed so the helper can
 * be called repeatedly without harm.
 */
static void
ec_free_runtime_arrays(struct ec_bdev *ec)
{
	free(ec->stripe_dirty_map);
	ec->stripe_dirty_map = NULL;

	free(ec->stripe_unmapped_map);
	ec->stripe_unmapped_map = NULL;

	if (ec->wib_buf) {
		spdk_dma_free(ec->wib_buf);
		ec->wib_buf = NULL;
	}
	free(ec->wib_region_map);
	ec->wib_region_map = NULL;

	free(ec->wib_region_inflight);
	ec->wib_region_inflight = NULL;

	free(ec->wib_region_dirty_ticks);
	ec->wib_region_dirty_ticks = NULL;

	/*
	 * Bit-clear waiter queues. By the time the bdev is being torn down
	 * the I/O path is quiesced, so any still-queued waiters had their
	 * bdev_io aborted upstream; drain them with -ECANCELED so the
	 * callbacks (if any are still wired) don't leak state. The shadow
	 * map is freed regardless.
	 */
	{
		struct ec_pending_bit_clear *w, *tmp;

		TAILQ_FOREACH_SAFE(w, &ec->pending_bit_clears, link, tmp) {
			TAILQ_REMOVE(&ec->pending_bit_clears, w, link);
			if (w->cb_fn) {
				w->cb_fn(w->cb_arg, -ECANCELED);
			}
			free(w);
		}
		TAILQ_FOREACH_SAFE(w, &ec->in_flight_bit_clears, link, tmp) {
			TAILQ_REMOVE(&ec->in_flight_bit_clears, w, link);
			if (w->cb_fn) {
				w->cb_fn(w->cb_arg, -ECANCELED);
			}
			free(w);
		}
	}
	free(ec->clear_staged_map);
	ec->clear_staged_map = NULL;
}

static void
ec_bdev_free(struct ec_bdev *ec)
{
	if (!ec) {
		return;
	}
	free(ec->bdev.name);
	free(ec->encode_matrix);
	free(ec->g_tbls);
	ec_free_runtime_arrays(ec);
	free(ec);
}

struct ec_bdev *
ec_bdev_find(const char *name)
{
	struct ec_bdev *ec;

	TAILQ_FOREACH(ec, &g_ec_bdev_list, link) {
		if (strcmp(ec->bdev.name, name) == 0) {
			return ec;
		}
	}
	return NULL;
}

/* Population count over `words` uint64_t entries. */
static uint64_t
ec_count_set_bits(const uint64_t *map, uint64_t words)
{
	uint64_t count = 0;
	uint64_t i;

	if (map == NULL) {
		return 0;
	}
	for (i = 0; i < words; i++) {
		count += __builtin_popcountll(map[i]);
	}
	return count;
}

/* Count the number of set bits in the dirty map (for diagnostics). */
static uint64_t
ec_stripe_count_dirty(const struct ec_bdev *ec)
{
	if (ec->stripe_dirty_map == NULL) {
		return 0;
	}
	return ec_count_set_bits(ec->stripe_dirty_map,
				 EC_BITMAP_WORDS(ec->num_stripes));
}

/* =========================================================================
 * Async cleanup chain
 * ========================================================================= */

static void
ec_cleanup_close_descriptor(struct ec_base_bdev_cleanup_ctx *ctx)
{
	struct ec_bdev *ec   = ctx->ec;
	uint32_t        slot = ctx->slot;

	if (ec->descs[slot]) {
		SPDK_NOTICELOG("EC bdev %s: closing descriptor for failed slot %u\n",
			       ec->bdev.name, slot);
		spdk_bdev_close(ec->descs[slot]);
		ec->descs[slot] = NULL;
	}

	if (ctx->quiesced) {
		spdk_bdev_unquiesce(&ec->bdev, &ec_if, NULL, NULL);
	}

	SPDK_NOTICELOG("EC bdev %s: async cleanup complete for slot %u\n",
		       ec->bdev.name, slot);

	free(ctx);
}

struct ec_cleanup_channel_ctx {
	struct ec_base_bdev_cleanup_ctx *cleanup_ctx;
};

static void
ec_cleanup_channel_iter_cb(struct spdk_io_channel_iter *i)
{
	struct ec_cleanup_channel_ctx   *cctx  = spdk_io_channel_iter_get_ctx(i);
	struct ec_base_bdev_cleanup_ctx *ctx   = cctx->cleanup_ctx;
	uint32_t                         slot  = ctx->slot;
	struct spdk_io_channel          *ch    = spdk_io_channel_iter_get_channel(i);
	struct ec_io_channel            *ec_ch = spdk_io_channel_get_ctx(ch);

	if (ec_ch->base_chans[slot]) {
		spdk_put_io_channel(ec_ch->base_chans[slot]);
		ec_ch->base_chans[slot] = NULL;
		SPDK_DEBUGLOG(bdev_ec,
			"EC bdev %s: released base_chan[%u] on thread %s\n",
			ctx->ec->bdev.name, slot,
			spdk_thread_get_name(spdk_get_thread()));
	}

	spdk_for_each_channel_continue(i, 0);
}

static void
ec_cleanup_channel_cb(struct spdk_io_channel_iter *i, int status)
{
	struct ec_cleanup_channel_ctx   *cctx = spdk_io_channel_iter_get_ctx(i);
	struct ec_base_bdev_cleanup_ctx *ctx  = cctx->cleanup_ctx;

	free(cctx);

	if (status != 0) {
		SPDK_WARNLOG("EC bdev %s: channel walk status %d; "
			     "proceeding with descriptor close\n",
			     ctx->ec->bdev.name, status);
	}

	ec_cleanup_close_descriptor(ctx);
}

static void
ec_cleanup_reset_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_base_bdev_cleanup_ctx *ctx = cb_arg;
	struct ec_bdev                  *ec  = ctx->ec;
	struct ec_cleanup_channel_ctx   *cctx;

	if (bdev_io) {
		spdk_bdev_free_io(bdev_io);
	}

	if (ctx->reset_ch) {
		spdk_put_io_channel(ctx->reset_ch);
		ctx->reset_ch = NULL;
	}

	if (!success) {
		SPDK_WARNLOG("EC bdev %s: reset of slot %u did not succeed "
			     "(expected for dead device); continuing\n",
			     ec->bdev.name, ctx->slot);
	}

	cctx = calloc(1, sizeof(*cctx));
	if (!cctx) {
		SPDK_ERRLOG("EC bdev %s: OOM for channel walk ctx; "
			    "closing descriptor directly\n", ec->bdev.name);
		ec_cleanup_close_descriptor(ctx);
		return;
	}

	cctx->cleanup_ctx = ctx;

	spdk_for_each_channel(ec,
		ec_cleanup_channel_iter_cb,
		cctx,
		ec_cleanup_channel_cb);
}

static void
ec_cleanup_quiesce_cb(void *cb_arg, int status)
{
	struct ec_base_bdev_cleanup_ctx *ctx  = cb_arg;
	struct ec_bdev                  *ec   = ctx->ec;
	uint32_t                         slot = ctx->slot;
	int                              rc;

	if (status != 0) {
		SPDK_ERRLOG("EC bdev %s: quiesce failed (status %d); "
			    "closing descriptor without full cleanup\n",
			    ec->bdev.name, status);
		if (ec->descs[slot]) {
			spdk_bdev_close(ec->descs[slot]);
			ec->descs[slot] = NULL;
		}
		free(ctx);
		return;
	}

	ctx->quiesced = true;

	if (!ec->descs[slot]) {
		SPDK_WARNLOG("EC bdev %s: slot %u descriptor already NULL "
			     "after quiesce; skipping reset\n",
			     ec->bdev.name, slot);
		ec_cleanup_reset_cb(NULL, true, ctx);
		return;
	}

	ctx->reset_ch = spdk_bdev_get_io_channel(ec->descs[slot]);
	if (!ctx->reset_ch) {
		SPDK_WARNLOG("EC bdev %s: failed to get reset channel for "
			     "slot %u; skipping to channel cleanup\n",
			     ec->bdev.name, slot);
		ec_cleanup_reset_cb(NULL, false, ctx);
		return;
	}

	rc = spdk_bdev_reset(ec->descs[slot], ctx->reset_ch,
			     ec_cleanup_reset_cb, ctx);
	if (rc != 0) {
		SPDK_WARNLOG("EC bdev %s: failed to submit reset for slot %u "
			     "(rc=%d); skipping to channel cleanup\n",
			     ec->bdev.name, slot, rc);
		spdk_put_io_channel(ctx->reset_ch);
		ctx->reset_ch = NULL;
		ec_cleanup_reset_cb(NULL, false, ctx);
	}
}

static void
ec_start_base_bdev_cleanup(struct ec_bdev *ec, uint32_t slot)
{
	struct ec_base_bdev_cleanup_ctx *ctx;

	SPDK_NOTICELOG("EC bdev %s: starting async cleanup for slot %u\n",
		       ec->bdev.name, slot);

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		SPDK_ERRLOG("EC bdev %s: OOM for cleanup ctx slot %u\n",
			    ec->bdev.name, slot);
		return;
	}

	ctx->ec       = ec;
	ctx->slot     = slot;
	ctx->quiesced = false;

	spdk_bdev_quiesce(&ec->bdev, &ec_if, ec_cleanup_quiesce_cb, ctx);
}

/*
 * True while teardown must wait: a WIB or bitmap persist is still writing to
 * the dedicated channels, or the create chain still owns them
 * (create_in_progress). Closing a channel or freeing the ec_bdev while this is
 * true hits the bdev layer's io_outstanding assert.
 */
static inline bool
ec_teardown_must_defer(const struct ec_bdev *ec)
{
	return ec->wib_persist_in_flight || ec->bitmap_persist_in_flight ||
	       ec->create_in_progress;
}

/*
 * Release the home-thread channels a single slot holds: its WIB channel
 * (parity slots only), its bitmap channel, and any live scrub / rebuild
 * channel. Each is NULL-checked, so this is safe to call once per slot
 * regardless of which channels that slot actually opened.
 *
 * Caller must ensure no persist write is outstanding on the WIB / bitmap
 * channels (see ec_teardown_must_defer and the deferral in
 * ec_handle_base_bdev_failure) -- putting a channel with I/O still in
 * flight trips the bdev-layer io_outstanding assert.
 */
static void
ec_release_slot_dedicated_channels(struct ec_bdev *ec, uint32_t slot)
{
	if (slot >= ec->k) {
		uint32_t parity_idx = slot - ec->k;
		if (ec->wib_chans[parity_idx]) {
			spdk_put_io_channel(ec->wib_chans[parity_idx]);
			ec->wib_chans[parity_idx] = NULL;
		}
	}

	if (ec->bitmap_chans[slot]) {
		spdk_put_io_channel(ec->bitmap_chans[slot]);
		ec->bitmap_chans[slot] = NULL;
	}

	if (ec->scrub_ctx != NULL && ec->scrub_ctx->scrub_chans[slot] != NULL) {
		spdk_put_io_channel(ec->scrub_ctx->scrub_chans[slot]);
		ec->scrub_ctx->scrub_chans[slot] = NULL;
	}

	if (ec->rebuild_ctx != NULL && ec->rebuild_ctx->rebuild_chans[slot] != NULL) {
		spdk_put_io_channel(ec->rebuild_ctx->rebuild_chans[slot]);
		ec->rebuild_ctx->rebuild_chans[slot] = NULL;
	}
}

/* =========================================================================
 * Failure detection
 * ========================================================================= */

static void
ec_handle_base_bdev_failure(struct ec_bdev *ec, struct spdk_bdev *bdev)
{
	uint32_t    i;
	const char *bdev_name = spdk_bdev_get_name(bdev);

	for (i = 0; i < ec->n; i++) {
		if (ec->descs[i] && spdk_bdev_desc_get_bdev(ec->descs[i]) == bdev) {
			if (ec->base_states[i] == EC_BASE_STATE_FAILED) {
				SPDK_DEBUGLOG(bdev_ec,
					"Base bdev %s (slot %u) already marked failed\n",
					bdev_name, i);
				return;
			}

			if (ec->base_states[i] == EC_BASE_STATE_REPLACING) {
				/*
				 * Replacement disk failed before rebuild finished.
				 * Clear rebuild flags; allow a new replace.
				 */
				SPDK_WARNLOG("EC bdev %s: REPLACEMENT disk %s "
					     "(slot %u) failed before rebuild; "
					     "slot returns to FAILED\n",
					     ec->bdev.name, bdev_name, i);
				ec->needs_rebuild[i]    = false;
				ec->replace_in_progress = false;
				/*
				 * If a rebuild is in progress for this slot,
				 * it will fail on its next I/O and call
				 * ec_rebuild_finish with a non-zero rc.
				 * We do not abort it here; let the I/O error
				 * path handle it naturally.
				 */
			} else {
				ec->failed_count++;
			}

			ec->base_states[i] = EC_BASE_STATE_FAILED;

			if (ec->failed_count > ec->m) {
				SPDK_ERRLOG("EC bdev %s fault tolerance exceeded: "
					    "%u/%u disks failed (max %u). OFFLINE.\n",
					    ec->bdev.name, ec->failed_count, ec->n, ec->m);
				ec->offline = true;
			} else {
				SPDK_WARNLOG("EC bdev %s: %s disk %s (slot %u/%u) failed "
					     "(%u/%u disks failed, can tolerate %u more)\n",
					     ec->bdev.name,
					     i >= ec->k ? "PARITY" : "DATA",
					     bdev_name, i, ec->n,
					     ec->failed_count, ec->n,
					     ec->m - ec->failed_count);
			}

			/*
			 * Release this slot's dedicated channels, then start the
			 * async quiesce/reset/close. Defer both if a persist write
			 * is still outstanding on the slot's WIB / bitmap channel --
			 * releasing then would trip the bdev-layer io_outstanding
			 * assert. The slot is already FAILED, so reads degrade
			 * regardless of when the channels are freed.
			 *
			 * The deferral holds the base descriptor open only briefly:
			 * the stuck write is aborted at ctrlr-loss, the same event
			 * as this REMOVE, and its completion resumes the release.
			 */
			if (ec_teardown_must_defer(ec)) {
				ec->dedicated_release_pending[i] = true;
				SPDK_NOTICELOG("EC bdev %s: slot %u channel "
					       "release deferred behind an "
					       "in-flight persist\n",
					       ec->bdev.name, i);
				return;
			}

			ec_release_slot_dedicated_channels(ec, i);
			ec_start_base_bdev_cleanup(ec, i);
			return;
		}
	}

	SPDK_WARNLOG("Failed to find descriptor for base bdev %s in EC bdev %s\n",
		     bdev_name, ec->bdev.name);
}

static void
ec_base_bdev_event_cb(enum spdk_bdev_event_type type,
		      struct spdk_bdev *bdev, void *event_ctx)
{
	struct ec_bdev *ec = event_ctx;

	switch (type) {
	case SPDK_BDEV_EVENT_REMOVE:
		SPDK_DEBUGLOG(bdev_ec, "Base bdev %s: SPDK_BDEV_EVENT_REMOVE on EC bdev %s\n",
			      spdk_bdev_get_name(bdev), ec->bdev.name);
		ec_handle_base_bdev_failure(ec, bdev);
		break;
	case SPDK_BDEV_EVENT_RESIZE:
		SPDK_DEBUGLOG(bdev_ec, "Base bdev %s: SPDK_BDEV_EVENT_RESIZE on EC bdev %s\n",
			      spdk_bdev_get_name(bdev), ec->bdev.name);
		break;
	case SPDK_BDEV_EVENT_MEDIA_MANAGEMENT:
		SPDK_DEBUGLOG(bdev_ec, "Base bdev %s: SPDK_BDEV_EVENT_MEDIA_MANAGEMENT on EC bdev %s\n",
			      spdk_bdev_get_name(bdev), ec->bdev.name);
		break;
	default:
		SPDK_NOTICELOG("Base bdev %s: unknown event %d on EC bdev %s\n",
			       spdk_bdev_get_name(bdev), type, ec->bdev.name);
		break;
	}
}

static void
ec_close_base_bdevs(struct ec_bdev *ec)
{
	uint32_t i;

	for (i = 0; i < ec->n; i++) {
		if (ec->descs[i]) {
			spdk_bdev_close(ec->descs[i]);
			ec->descs[i] = NULL;
		}
	}
}

static int
ec_open_base_bdevs(struct ec_bdev *ec, const char **base_bdev_names)
{
	struct spdk_bdev *base_bdev;
	uint32_t          i;
	int               rc;
	bool              blocklen_set = false;

	for (i = 0; i < ec->n; i++) {
		/* Empty string means the slot is intentionally missing
		 * (e.g., crash recovery with a failed disk). Mark it
		 * FAILED and skip opening.
		 */
		if (base_bdev_names[i] == NULL ||
		    base_bdev_names[i][0] == '\0') {
			SPDK_NOTICELOG("EC bdev %s: slot %u marked FAILED "
				       "(missing base bdev)\n",
				       ec->bdev.name, i);
			ec->descs[i]       = NULL;
			ec->base_states[i] = EC_BASE_STATE_FAILED;
			ec->failed_count++;
			continue;
		}

		SPDK_NOTICELOG("Opening base bdev %s for EC bdev %s\n",
			       base_bdev_names[i], ec->bdev.name);

		rc = spdk_bdev_open_ext(base_bdev_names[i], true,
					ec_base_bdev_event_cb, ec,
					&ec->descs[i]);
		if (rc != 0) {
			SPDK_ERRLOG("Failed to open base bdev %s: %s\n",
				    base_bdev_names[i], spdk_strerror(-rc));
			return rc;
		}

		base_bdev = spdk_bdev_desc_get_bdev(ec->descs[i]);

		if (!blocklen_set) {
			ec->bdev.blocklen = base_bdev->blocklen;
			blocklen_set = true;
		} else if (ec->bdev.blocklen != base_bdev->blocklen) {
			SPDK_ERRLOG("Block length mismatch for %s\n",
				    base_bdev_names[i]);
			return -EINVAL;
		}
	}

	if (!blocklen_set) {
		SPDK_ERRLOG("EC bdev %s: all base bdevs are missing\n",
			    ec->bdev.name);
		return -EINVAL;
	}

	if (ec->failed_count > ec->m) {
		SPDK_ERRLOG("EC bdev %s: too many missing slots (%u) "
			    "exceeds fault tolerance m=%u\n",
			    ec->bdev.name, ec->failed_count, ec->m);
		return -EINVAL;
	}

	return 0;
}

/*
 * ec_compute_geometry -- pure derivation of EC bdev geometry from k, m,
 * strip_size_kb, and the first open base bdev's blockcnt. No allocation,
 * no side effects beyond filling in fields on ec. Returns -EINVAL on
 * inputs that produce an unworkable layout; the caller has nothing to
 * clean up on failure.
 */
static int
ec_compute_geometry(struct ec_bdev *ec)
{
	uint64_t          min_blockcnt = UINT64_MAX;
	uint64_t          max_blockcnt = 0;
	uint32_t          gi;
	uint64_t          total_physical_stripes;
	uint64_t          map_bytes_needed;
	uint64_t          wib_total_needed;
	uint64_t          buf_available;

	/*
	 * Size the EC bdev to the smallest open base disk's blockcnt so no
	 * user stripe can map past any one disk's EOF. The resize path uses
	 * the same min(blockcnt) rule (bdev_ec_resize.c). Track max so a
	 * mismatch surfaces in the log -- a heterogeneous create still
	 * succeeds, but the wasted blocks on oversized slots are worth a
	 * NOTICE so an operator can spot a provisioning typo.
	 */
	for (gi = 0; gi < ec->n; gi++) {
		struct spdk_bdev *base;

		if (!ec->descs[gi]) {
			continue;
		}
		base = spdk_bdev_desc_get_bdev(ec->descs[gi]);
		if (base->blockcnt < min_blockcnt) {
			min_blockcnt = base->blockcnt;
		}
		if (base->blockcnt > max_blockcnt) {
			max_blockcnt = base->blockcnt;
		}
	}
	if (min_blockcnt == UINT64_MAX) {
		SPDK_ERRLOG("EC bdev %s: no open base bdevs for geometry\n",
			    ec->bdev.name);
		return -EINVAL;
	}
	if (min_blockcnt != max_blockcnt) {
		SPDK_NOTICELOG("EC bdev %s: base bdev capacities differ "
			       "(min=%" PRIu64 ", max=%" PRIu64 " blocks); "
			       "sizing to min, %" PRIu64 " blocks unused per oversized slot\n",
			       ec->bdev.name, min_blockcnt, max_blockcnt,
			       max_blockcnt - min_blockcnt);
	}

	/*
	 * The commit-record stamp occupies one block, so the block must hold the
	 * struct plus its CRC trailer.
	 */
	if (ec->bdev.blocklen < sizeof(struct ec_bitmap_commit) + sizeof(uint32_t)) {
		SPDK_ERRLOG("EC bdev %s: blocklen=%u too small for the commit record "
			    "(need >= %zu bytes)\n", ec->bdev.name, ec->bdev.blocklen,
			    sizeof(struct ec_bitmap_commit) + sizeof(uint32_t));
		return -EINVAL;
	}

	ec->strip_size = ((uint64_t)ec->strip_size_kb * 1024) / ec->bdev.blocklen;
	if (ec->strip_size == 0 ||
	    ((uint64_t)ec->strip_size_kb * 1024) % ec->bdev.blocklen != 0) {
		SPDK_ERRLOG("Invalid strip size: strip_size_kb=%u not a multiple "
			    "of blocklen=%u\n", ec->strip_size_kb, ec->bdev.blocklen);
		return -EINVAL;
	}

	/*
	 * ec_encode_data() takes the chunk byte length as an int. Reject strips
	 * larger than INT_MAX bytes, or the cast at the encode call sites
	 * truncates the length and corrupts parity.
	 */
	if (ec->strip_size * ec->bdev.blocklen > (uint64_t)INT_MAX) {
		SPDK_ERRLOG("EC bdev %s: strip size too large: strip_size_kb=%u "
			    "(%" PRIu64 " bytes) exceeds the ISA-L per-chunk limit "
			    "of %d bytes\n",
			    ec->bdev.name, ec->strip_size_kb,
			    (uint64_t)ec->strip_size * ec->bdev.blocklen, INT_MAX);
		return -EINVAL;
	}

	ec->stripe_blocks = ec->k * ec->strip_size;

	/*
	 * Front-placed metadata:
	 *   [ bitmap_reservation_strips ][ 2 commit strips ][ 2 WIB strips ][ user data ]
	 *
	 * All three regions are fixed-max and never move on resize; only the
	 * trailing user-data region grows. data_offset_stripes is their combined
	 * size -- the LBA where user stripe 0 starts. It is identical on every
	 * disk: data disks reserve the commit and WIB strips even though they
	 * never write the WIB, so one layout rule covers all k+m disks.
	 *
	 * A disk that cannot even hold the combined reservation is rejected
	 * here; the WIB-fits-in-one-strip check still runs at the bottom of
	 * this function as an independent invariant.
	 */
	total_physical_stripes  = min_blockcnt / ec->strip_size;
	ec->data_offset_stripes = ec_bitmap_reservation_stripes(ec)
				  + EC_BITMAP_COMMIT_STRIPS  /* commit record */
				  + 2;                       /* WIB copy 0 + copy 1 */

	if (total_physical_stripes <= ec->data_offset_stripes) {
		SPDK_ERRLOG("EC bdev %s: disk too small to reserve front "
			    "metadata (physical stripes=%" PRIu64 ", "
			    "data_offset_stripes=%" PRIu64 " = bitmap + 2 commit + 2 WIB strips)\n",
			    ec->bdev.name, total_physical_stripes,
			    ec->data_offset_stripes);
		return -EINVAL;
	}

	ec->num_stripes   = total_physical_stripes - ec->data_offset_stripes;
	ec->bdev.blockcnt = ec->num_stripes * ec->stripe_blocks;
	ec->wib_num_regions = (uint32_t)((ec->num_stripes + EC_WIB_REGION_STRIPES - 1) /
				EC_WIB_REGION_STRIPES);

	/*
	 * write_unit_size=1: allows sub-stripe writes (RMW path).
	 * optimal_io_boundary=strip_size: SPDK splits cross-strip writes.
	 * max_write_zeroes=0: WRITE_ZEROES is left unset so the bdev layer
	 * auto-emulates it as a buffer-backed WRITE that respects
	 * optimal_io_boundary. See ec_io_type_supported for the full rationale.
	 */
	ec->bdev.write_unit_size              = 1;
	ec->bdev.optimal_io_boundary          = ec->strip_size;
	ec->bdev.split_on_write_unit          = false;
	ec->bdev.split_on_optimal_io_boundary = true;

	/*
	 * Publish max_unmap and max_unmap_segments derived from base bdevs.
	 * SPDK splits an UNMAP request whose num_blocks exceeds max_unmap or
	 * whose segment count exceeds max_unmap_segments. An EC-level UNMAP
	 * fans out as one UNMAP per base bdev at num_blocks / k per slot, so
	 * the EC-level limit is k * min(base->max_unmap). Segments pass
	 * through 1:1. Zero on any base means "no limit" and is excluded from
	 * the min; if every base reports zero, EC reports zero too (no
	 * splitting). This sets the layer-above splitting boundary so a large
	 * fstrim does not arrive at ec_submit_unmap larger than the smallest
	 * base bdev can absorb in one operation.
	 */
	{
		uint32_t min_max_unmap          = UINT32_MAX;
		uint32_t min_max_unmap_segments = UINT32_MAX;
		uint32_t i;
		uint64_t max_unmap_blocks;

		for (i = 0; i < ec->n; i++) {
			struct spdk_bdev *base;

			if (!ec->descs[i]) {
				continue;
			}
			base = spdk_bdev_desc_get_bdev(ec->descs[i]);
			if (base->max_unmap > 0) {
				min_max_unmap = spdk_min(min_max_unmap,
							 base->max_unmap);
			}
			if (base->max_unmap_segments > 0) {
				min_max_unmap_segments =
					spdk_min(min_max_unmap_segments,
						 base->max_unmap_segments);
			}
		}
		max_unmap_blocks = (uint64_t)min_max_unmap * ec->k;
		ec->bdev.max_unmap = (min_max_unmap == UINT32_MAX) ? 0 :
				     (uint32_t)spdk_min(max_unmap_blocks, (uint64_t)UINT32_MAX);
		ec->bdev.max_unmap_segments =
			(min_max_unmap_segments == UINT32_MAX) ?
			0 : min_max_unmap_segments;

		/*
		 * Soft-cap max_unmap to one WIB region's worth of EC-level
		 * blocks (EC_WIB_REGION_STRIPES * stripe_blocks). This is a
		 * work bound, not a correctness requirement: ec_submit_unmap
		 * handles UNMAPs that span multiple WIB regions (the whole-blob
		 * bitmap persist has no region concept; the scrubber defers
		 * region by region), so a larger request is served correctly.
		 * Bounding it here just keeps one UNMAP from spanning many
		 * regions and deferring behind the startup scrub. In typical
		 * hardware (4 MiB base max_unmap, k=4, 64 KiB strip -> 16 MiB
		 * EC max_unmap, 256 MiB WIB region) the cap never fires. The
		 * value is EC-level blocks; each base bdev sees num_blocks / k
		 * after fan-out.
		 */
		{
			uint64_t wib_region_blocks =
				EC_WIB_REGION_STRIPES * ec->stripe_blocks;
			if (ec->bdev.max_unmap == 0 ||
			    ec->bdev.max_unmap > wib_region_blocks) {
				ec->bdev.max_unmap =
					(wib_region_blocks > UINT32_MAX) ?
					UINT32_MAX : (uint32_t)wib_region_blocks;
			}
		}
	}

	/*
	 * Validate that one strip is large enough to hold the WIB on-disk
	 * layout: sizeof(ec_wib_header) + ceil(wib_num_regions/64)*8 + 4 (CRC).
	 * This is guaranteed for any realistic strip size / disk size combination,
	 * but we check explicitly to catch edge cases (very small strips on very
	 * large disks) before silently producing a too-small buffer.
	 *
	 * This word-granular check and ec_max_num_stripes are the same ceiling
	 * (both round the region bitmap up to whole words): passing it means
	 * num_stripes <= ec_max_num_stripes, which also keeps the unmapped-bitmap
	 * I/O within its reserved slot (see ec_bitmap_slot_io_blocks). Resize
	 * checks ec_max_num_stripes directly, since it does not run this path.
	 */
	map_bytes_needed = ((uint64_t)EC_BITMAP_WORDS(ec->wib_num_regions))
			    * sizeof(uint64_t);
	wib_total_needed = sizeof(struct ec_wib_header) + map_bytes_needed
			    + sizeof(uint32_t);  /* CRC */
	buf_available    = (uint64_t)ec->strip_size * ec->bdev.blocklen;
	if (wib_total_needed > buf_available) {
		SPDK_ERRLOG("EC bdev %s: WIB on-disk layout (%" PRIu64 " bytes) "
			    "exceeds one strip (%" PRIu64 " bytes). "
			    "Increase strip_size_kb or reduce disk size.\n",
			    ec->bdev.name, wib_total_needed, buf_available);
		return -EINVAL;
	}

	return 0;
}

/*
 * ec_alloc_runtime_arrays -- allocate the per-bdev state arrays, sized from
 * the geometry in ec. Called once at create on a fresh ec, never on a live
 * bdev. On OOM, frees the partial allocations so the caller can free ec.
 */
static int
ec_alloc_runtime_arrays(struct ec_bdev *ec)
{
	uint64_t map_words        = EC_BITMAP_WORDS(ec->num_stripes);
	uint64_t wib_region_words = EC_BITMAP_WORDS(ec->wib_num_regions);

	assert(ec->wib_region_map == NULL);

	ec->stripe_dirty_map = calloc(map_words, sizeof(uint64_t));
	if (!ec->stripe_dirty_map) {
		SPDK_ERRLOG("EC bdev %s: OOM for stripe_dirty_map "
			    "(%" PRIu64 " stripes, %" PRIu64 " words)\n",
			    ec->bdev.name, ec->num_stripes, map_words);
		goto err;
	}

	ec->stripe_unmapped_map = calloc(map_words, sizeof(uint64_t));
	if (!ec->stripe_unmapped_map) {
		SPDK_ERRLOG("EC bdev %s: OOM for stripe_unmapped_map "
			    "(%" PRIu64 " stripes, %" PRIu64 " words)\n",
			    ec->bdev.name, ec->num_stripes, map_words);
		goto err;
	}

	ec->wib_region_map = calloc(wib_region_words, sizeof(uint64_t));
	if (!ec->wib_region_map) {
		SPDK_ERRLOG("EC bdev %s: OOM for wib_region_map "
			    "(%u regions, %" PRIu64 " words)\n",
			    ec->bdev.name, ec->wib_num_regions, wib_region_words);
		goto err;
	}

	ec->wib_region_inflight = calloc(ec->wib_num_regions, sizeof(uint32_t));
	if (!ec->wib_region_inflight) {
		SPDK_ERRLOG("EC bdev %s: OOM for wib_region_inflight "
			    "(%u regions, %" PRIu64 " bytes)\n",
			    ec->bdev.name, ec->wib_num_regions,
			    (uint64_t)ec->wib_num_regions * sizeof(uint32_t));
		goto err;
	}

	ec->wib_region_dirty_ticks = calloc(ec->wib_num_regions, sizeof(uint64_t));
	if (!ec->wib_region_dirty_ticks) {
		SPDK_ERRLOG("EC bdev %s: OOM for wib_region_dirty_ticks "
			    "(%u regions, %" PRIu64 " bytes)\n",
			    ec->bdev.name, ec->wib_num_regions,
			    (uint64_t)ec->wib_num_regions * sizeof(uint64_t));
		goto err;
	}

	{
		uint64_t wib_buf_bytes = (uint64_t)ec->strip_size * ec->bdev.blocklen;

		ec->wib_buf = spdk_dma_zmalloc(wib_buf_bytes, EC_DMA_ALIGN, NULL);
		if (!ec->wib_buf) {
			SPDK_ERRLOG("EC bdev %s: OOM for wib_buf "
				    "(%" PRIu64 " bytes)\n",
				    ec->bdev.name, wib_buf_bytes);
			goto err;
		}
	}

	/* Scalar / pointer / array fields are already zeroed by the calloc
	 * in _ec_bdev_create; only the list heads need explicit init. */
	TAILQ_INIT(&ec->wib_deferred_writes);
	TAILQ_INIT(&ec->pending_bit_clears);
	TAILQ_INIT(&ec->in_flight_bit_clears);

	SPDK_DEBUGLOG(bdev_ec,
		"EC bdev %s: runtime arrays allocated -- %" PRIu64 " stripes, "
		"dirty map %" PRIu64 " bytes, WIB %u regions\n",
		ec->bdev.name, ec->num_stripes, map_words * sizeof(uint64_t),
		ec->wib_num_regions);

	return 0;

err:
	ec_free_runtime_arrays(ec);
	return -ENOMEM;
}

static int
ec_init_isa_l_tables(struct ec_bdev *ec)
{
	/*
	 * gf_gen_rs_matrix(a, n, k) writes n*k bytes:
	 *   rows 0..k-1  = kxk identity matrix (data rows)
	 *   rows k..n-1  = mxk parity generator rows
	 *
	 * ec_init_tables(k, m, &a[k*k], g_tbls) uses only the m parity rows
	 * (starting at byte offset k*k) and writes 32*k*m bytes to g_tbls.
	 *
	 * We must allocate n*k bytes for encode_matrix so gf_gen_rs_matrix can
	 * write all n rows. Allocating only m*k underallocates by k*k bytes and
	 * causes a buffer overrun for m < k.
	 */
	ec->encode_matrix = malloc(ec->n * ec->k);
	if (!ec->encode_matrix) {
		SPDK_ERRLOG("EC bdev %s: OOM for encode_matrix "
			    "(%u bytes, n=%u k=%u)\n",
			    ec->bdev.name, ec->n * ec->k, ec->n, ec->k);
		return -ENOMEM;
	}

	ec->g_tbls = malloc(EC_ISAL_GF_TABLE_BYTES * ec->k * ec->m);
	if (!ec->g_tbls) {
		SPDK_ERRLOG("EC bdev %s: OOM for g_tbls (%u bytes, k=%u m=%u)\n",
			    ec->bdev.name, EC_ISAL_GF_TABLE_BYTES * ec->k * ec->m, ec->k, ec->m);
		free(ec->encode_matrix);
		ec->encode_matrix = NULL;
		return -ENOMEM;
	}

	gf_gen_rs_matrix(ec->encode_matrix, ec->n, ec->k);
	ec_init_tables(ec->k, ec->m, &ec->encode_matrix[ec->k * ec->k], ec->g_tbls);

	return 0;
}

static int
_ec_bdev_create(const char *name, uint32_t strip_size_kb, uint32_t k, uint32_t m,
		const struct spdk_uuid *uuid, struct ec_bdev **ec_bdev_out)
{
	struct ec_bdev   *ec;
	struct spdk_bdev *ec_bdev_gen;
	uint32_t          num_base_bdevs = k + m;
	int               rc;

	if (strnlen(name, EC_BDEV_NAME_MAX) == EC_BDEV_NAME_MAX) {
		SPDK_ERRLOG("EC bdev name '%s' exceeds %d characters\n",
			    name, EC_BDEV_NAME_MAX - 1);
		return -EINVAL;
	}

	if (spdk_bdev_get_by_name(name) != NULL) {
		SPDK_ERRLOG("Duplicate EC bdev name: %s\n", name);
		return -EEXIST;
	}

	/*
	 * Bound k and m individually before relying on their sum: the addition
	 * is done in uint32_t and a JSON-RPC caller can pass huge values that
	 * wrap k+m to a small number, bypassing the n>255 / EC_MAX_BASE_BDEVS
	 * checks below while leaving the huge values stored in ec->k / ec->m
	 * where gf_gen_rs_matrix() and ec_init_isa_l_tables() use them for
	 * memory sizing. Capping each at 255 (the GF(2^8) limit, and the natural
	 * ceiling for n=k+m) keeps the sum and every downstream allocation
	 * bounded.
	 */
	if (k == 0 || k > EC_GF8_MAX_CHUNKS || m == 0 || m > EC_GF8_MAX_CHUNKS) {
		SPDK_ERRLOG("Invalid EC geometry: k=%u m=%u (must be in 1..255)\n",
			    k, m);
		return -EINVAL;
	}

	/*
	 * ISA-L GF(2^8) constraint: n = k+m must not exceed 255.
	 *
	 * gf_gen_rs_matrix() (isa-l/erasure_code/ec_base.c) builds each
	 * parity row from a different power of 2: 2^0, 2^1, 2^2, and so
	 * on. In GF(2^8) only 255 of these are distinct -- at 2^255 the
	 * sequence wraps back to 1 -- so more than 255 rows reuse an
	 * earlier power. Two rows built from the same power come out
	 * identical, and identical rows break recovery: the parity can
	 * no longer rebuild data from every set of k surviving disks.
	 */
	if (num_base_bdevs > EC_GF8_MAX_CHUNKS) {
		SPDK_ERRLOG("Invalid EC geometry: n = k+m = %u exceeds GF(2^8) "
			    "limit of 255\n", num_base_bdevs);
		return -EINVAL;
	}

	if (num_base_bdevs > EC_MAX_BASE_BDEVS) {
		SPDK_ERRLOG("Too many base bdevs %u (max %d)\n",
			    num_base_bdevs, EC_MAX_BASE_BDEVS);
		return -EINVAL;
	}

	if (strip_size_kb == 0 || spdk_u32_is_pow2(strip_size_kb) == false) {
		SPDK_ERRLOG("Invalid strip size %" PRIu32 "\n", strip_size_kb);
		return -EINVAL;
	}

	ec = calloc(1, sizeof(*ec));
	if (!ec) {
		SPDK_ERRLOG("OOM for ec_bdev struct (%" PRIu64 " bytes)\n",
			    (uint64_t)sizeof(*ec));
		return -ENOMEM;
	}

	ec->k             = k;
	ec->m             = m;
	ec->n             = num_base_bdevs;
	ec->strip_size_kb = strip_size_kb;

	rc = ec_init_isa_l_tables(ec);
	if (rc != 0) {
		ec_bdev_free(ec);
		return rc;
	}

	ec_bdev_gen = &ec->bdev;
	ec_bdev_gen->name = strdup(name);
	if (!ec_bdev_gen->name) {
		ec_bdev_free(ec);
		return -ENOMEM;
	}

	ec_bdev_gen->product_name = "ErasureCode Volume";
	ec_bdev_gen->ctxt         = ec;
	ec_bdev_gen->fn_table     = &g_ec_fn_table;
	ec_bdev_gen->module       = &ec_if;
	ec_bdev_gen->write_cache  = 0;

	if (uuid) {
		spdk_uuid_copy(&ec_bdev_gen->uuid, uuid);
	}

	TAILQ_INSERT_TAIL(&g_ec_bdev_list, ec, link);

	*ec_bdev_out = ec;
	return 0;
}

/* =========================================================================
 * I/O channel management
 * ========================================================================= */

static int
ec_create_ch(void *io_device, void *ctx_buf)
{
	struct ec_bdev       *ec    = io_device;
	struct ec_io_channel *ec_ch = ctx_buf;
	uint32_t              i;

	for (i = 0; i < ec->n; i++) {
		/* FAILED: no descriptor. REPLACING: descriptor is live. */
		if (ec->base_states[i] == EC_BASE_STATE_FAILED || !ec->descs[i]) {
			ec_ch->base_chans[i] = NULL;
			continue;
		}

		ec_ch->base_chans[i] = spdk_bdev_get_io_channel(ec->descs[i]);
		if (!ec_ch->base_chans[i]) {
			SPDK_ERRLOG("Failed to get I/O channel for base bdev "
				    "index %u\n", i);
			goto err_cleanup;
		}
	}

	return 0;

err_cleanup:
	while (i > 0) {
		i--;
		if (ec_ch->base_chans[i]) {
			spdk_put_io_channel(ec_ch->base_chans[i]);
			ec_ch->base_chans[i] = NULL;
		}
	}
	return -ENOMEM;
}

static void
ec_destroy_ch(void *io_device, void *ctx_buf)
{
	struct ec_io_channel *ec_ch = ctx_buf;
	struct ec_bdev       *ec    = io_device;
	uint32_t              i;

	for (i = 0; i < ec->n; i++) {
		if (ec_ch->base_chans[i]) {
			spdk_put_io_channel(ec_ch->base_chans[i]);
			ec_ch->base_chans[i] = NULL;
		}
	}
}

/* =========================================================================
 * Public creation / deletion
 * ========================================================================= */

static void ec_device_unregister_done(void *io_device);
static void ec_release_dedicated_channels(struct ec_bdev *ec);
/* ec_scrub_free_resources declared in bdev_ec_internal.h. */

/*
 * Context threaded through the async ec_bdev_create_async chain:
 * WIB load (ec_wib_load_async) -> bitmap load (ec_bitmap_load_async) ->
 * finalize (ec_bdev_create_finalize), which starts the scrub and gates the
 * caller's done_fn on bdev_examine completion.
 */
struct ec_bdev_create_async_ctx {
	struct ec_bdev       *ec;
	ec_bdev_create_cb_fn done_fn;
	void                 *done_arg;
	int                   deferred_rc;  /* rc preserved across async unregister teardown
					      * (set on any create-failure path) */
	bool                  salvage_requested;
};

static void ec_bdev_create_abort(struct ec_bdev_create_async_ctx *ctx);

/*
 * ec_bdev_create_examine_done -- fires after bdev_examine on the newly
 * registered EC bdev has completed. Releases the create context and
 * signals success to the caller.
 *
 * Gating the JSON-RPC reply here closes a race where callers doing
 * examine-dependent discovery (e.g., bdev_lvol_get_lvstores for a
 * pre-existing lvstore on the encoded blocks during a salvage flow)
 * would otherwise see the lvstore as missing because the lvol module's
 * async examine_disk chain had not yet completed.
 */
static void
ec_bdev_create_examine_done(void *cb_arg)
{
	struct ec_bdev_create_async_ctx *ctx = cb_arg;
	struct ec_bdev                  *ec  = ctx->ec;
	ec_bdev_create_cb_fn done_fn;
	void *done_arg;

	/*
	 * Last step of the create. A delete or shutdown during the
	 * wait-for-examine window aborts here instead; runs on the home thread, so
	 * the drain is safe.
	 */
	if (ec->destructing) {
		ec_bdev_create_abort(ctx);
		ec_drain_deferred_slot_releases(ec);
		ec_drain_deferred_unregister(ec);
		return;
	}

	/*
	 * Create done: drop the gate, then release any slot cleanup parked during
	 * the window (the drain no-ops until the gate is down), then answer the RPC.
	 */
	ec->create_in_progress = false;
	ec_drain_deferred_slot_releases(ec);

	done_fn = ctx->done_fn;
	done_arg = ctx->done_arg;
	free(ctx);
	done_fn(done_arg, 0);
}

/*
 * Post-WIB-load finalization. Starts the background scrub if dirty regions
 * were found, then gates the caller's done_fn on bdev_examine completion.
 * Extracted so the self-test path can dispatch into the same flow on success.
 */
static void
ec_bdev_create_finalize(struct ec_bdev_create_async_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;
	int             rc;

	/*
	 * Callers check destructing in the same frame just before calling this, so
	 * it cannot be set here. If a future async hop breaks that, add a checked
	 * abort at the hop instead of relaxing this assert.
	 */
	assert(!ec->destructing);

	rc = ec_bdev_start_scrub(ec);
	if (rc != 0) {
		SPDK_WARNLOG("EC bdev %s: failed to start startup scrub "
			     "(rc=%d); parity may be stale in dirty regions\n",
			     ec->bdev.name, rc);
		/* Non-fatal */
	}

	SPDK_NOTICELOG("Created EC bdev %s (k=%u m=%u, WIB %u regions%s)\n",
		       ec->bdev.name, ec->k, ec->m, ec->wib_num_regions,
		       ec->scrub_ctx ? ", scrub in progress" : "");

	/*
	 * Gate the create-completion callback on bdev_examine.
	 * spdk_bdev_register dispatched examine_disk callbacks asynchronously;
	 * modules like vbdev_lvol do I/O through this EC bdev to import an
	 * existing lvstore, and that I/O typically outlives the WIB load.
	 * Without this wait, ec_bdev_create_async's caller could observe
	 * the create as complete before lvol examine has registered the
	 * lvstore, causing salvage-mode discovery to fail.
	 *
	 * spdk_bdev_wait_for_examine waits on every bdev module's
	 * action_in_progress counter globally, not just on this bdev's
	 * examines. In our typical workload, bdev create RPCs are
	 * serialized enough that this coupling is invisible.
	 */
	rc = spdk_bdev_wait_for_examine(ec_bdev_create_examine_done, ctx);
	if (rc != 0) {
		ec_bdev_create_cb_fn done_fn = ctx->done_fn;
		void *done_arg = ctx->done_arg;

		SPDK_WARNLOG("EC bdev %s: spdk_bdev_wait_for_examine failed: %s; "
			     "completing create without examine gate (caller may race)\n",
			     ec->bdev.name, spdk_strerror(-rc));
		/*
		 * Last step of the create (without the examine gate). Drop the gate and
		 * release parked slot cleanup before answering. destructing can't be set
		 * here -- same frame as the entry assert.
		 */
		ec->create_in_progress = false;
		ec_drain_deferred_slot_releases(ec);
		free(ctx);
		done_fn(done_arg, 0);
	}
}


/*
 * Async-unregister callback after a create-time failure. By the time
 * this fires the ec_bdev is gone; the rc to report was preserved on
 * the create ctx as deferred_rc by whichever upstream branch issued
 * the unregister.
 */
static void
ec_bdev_create_unregister_done(void *cb_arg, int unregister_rc)
{
	struct ec_bdev_create_async_ctx *ctx = cb_arg;
	ec_bdev_create_cb_fn done_fn = ctx->done_fn;
	void *done_arg = ctx->done_arg;
	int rc = ctx->deferred_rc;

	(void)unregister_rc;  /* deferred_rc is the user-meaningful failure */
	free(ctx);
	done_fn(done_arg, rc);
}

/*
 * A delete or shutdown started mid-create (ec->destructing). Answer the create
 * RPC with -ECANCELED and free the ctx. Does not unregister (the delete already
 * did), free ec, or drain -- each caller decides whether to drain (see the call
 * sites).
 */
static void
ec_bdev_create_abort(struct ec_bdev_create_async_ctx *ctx)
{
	ec_bdev_create_cb_fn done_fn = ctx->done_fn;
	void *done_arg = ctx->done_arg;

	ctx->ec->create_in_progress = false;
	free(ctx);
	done_fn(done_arg, -ECANCELED);
}

/*
 * Fresh-create bitmap persist completion: both bitmap slots (and both commit slots) now
 * hold our generations, so a stale blob on a reused base bdev cannot out-rank
 * ours on a later load.
 */
static void
ec_bdev_create_bitmap_persist_done(void *cb_arg, int rc)
{
	struct ec_bdev_create_async_ctx *ctx = cb_arg;
	struct ec_bdev                  *ec  = ctx->ec;

	/*
	 * Do not drain here: this runs inside ec_bitmap_persist_write_cb, whose tail
	 * already drains. Draining now would free ec out from under it -- just answer
	 * the RPC and let that tail free ec.
	 */
	if (ec->destructing) {
		ec_bdev_create_abort(ctx);
		return;
	}

	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: fresh-create both-copy bitmap persist "
			    "failed (rc=%d); refusing to expose -- a slot may "
			    "still hold stale base-bdev bytes that could out-rank "
			    "our fresh copy on a future load\n", ec->bdev.name, rc);
		ec->create_in_progress = false;
		ctx->deferred_rc = rc;
		spdk_bdev_unregister(&ec->bdev, ec_bdev_create_unregister_done,
				     ctx);
		return;
	}

	SPDK_NOTICELOG("EC bdev %s: fresh bitmap persisted to both slots "
		       "(gen %" PRIu64 ")\n", ec->bdev.name, ec->bitmap_generation);

	ec_bdev_create_finalize(ctx);
}

/*
 * Bitmap-load completion: pick the post-load path based on whether
 * load found a valid copy and whether salvage_requested is set.
 *
 *   load found a copy -> bitmap is established, proceed.
 *   no copy + salvage_requested=false -> fresh create. Persist an
 *     all-mapped bitmap (stripe_unmapped_map is calloc'd to zero) to
 *     both slots on every disk so the next load sees a valid blob.
 *   no copy + salvage_requested=true -> the operator asked us to
 *     recreate an established volume but the on-disk bitmap is gone
 *     or unreadable. Fail loudly rather than silently inventing an
 *     all-mapped state, which would resurrect stale non-zero data on
 *     read.
 *
 * ec_bitmap_load_async always passes rc == 0 (load failure is
 * non-fatal at that layer); the "did we find anything" signal is
 * ec->bitmap_generation, which is 0 when no committed copy was applied.
 */
static void
ec_bdev_create_bitmap_load_done(void *cb_arg, int rc)
{
	struct ec_bdev_create_async_ctx *ctx = cb_arg;
	struct ec_bdev                  *ec  = ctx->ec;

	/* Runs as the last statement of the bitmap-load completion, so draining
	 * (which may free ec) is safe here. */
	if (ec->destructing) {
		ec_bdev_create_abort(ctx);
		ec_drain_deferred_slot_releases(ec);
		ec_drain_deferred_unregister(ec);
		return;
	}

	if (rc != 0) {
		/*
		 * The load failed (OOM allocating the load context or DMA scratch
		 * buffers). On-disk state is untouched, so abort the create and let
		 * the caller retry once memory recovers.
		 *
		 * Falling through is unsafe: bitmap_generation == 0 looks the same
		 * for "no bitmap on disk" and "load could not run," so a non-salvage
		 * create would persist a fresh all-mapped bitmap over the existing
		 * on-disk copies and silently drop every previously-recorded UNMAP.
		 * Matches ec_bdev_create_wib_done.
		 */
		SPDK_ERRLOG("EC bdev %s: bitmap load failed (rc=%d); aborting "
			    "create to preserve on-disk bitmap for a retry\n",
			    ec->bdev.name, rc);
		ec->create_in_progress = false;
		ctx->deferred_rc = rc;
		spdk_bdev_unregister(&ec->bdev, ec_bdev_create_unregister_done,
				     ctx);
		return;
	}

	if (ec->bitmap_generation > 0) {
		ec_bdev_create_finalize(ctx);
		return;
	}

	if (ctx->salvage_requested) {
		SPDK_ERRLOG("EC bdev %s: salvage requested but no valid bitmap "
			    "copy found on disk; refusing to expose -- "
			    "reconstructing as all-mapped would resurrect stale "
			    "non-zero data on read\n", ec->bdev.name);
		ec->create_in_progress = false;
		ctx->deferred_rc = -ENODATA;
		spdk_bdev_unregister(&ec->bdev, ec_bdev_create_unregister_done,
				     ctx);
		return;
	}

	/*
	 * Fresh create: no on-disk bitmap exists yet. Overwrite BOTH copies on
	 * every disk with a fresh all-mapped blob before exposing the bdev,
	 * so a stale blob from a reused base bdev cannot out-rank our
	 * fresh-create copy on a subsequent load. ec_bitmap_persist_both_copies
	 * does this as two drain-gated persists (gen 1 -> slot 1, gen 2 ->
	 * slot 0); after both drain, active = 0 and gen = 2.
	 */
	SPDK_NOTICELOG("EC bdev %s: no bitmap on disk; persisting fresh "
		       "all-mapped bitmap to both slots\n", ec->bdev.name);

	rc = ec_bitmap_persist_both_copies(ec, ec_bdev_create_bitmap_persist_done, ctx);
	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: fresh-create bitmap persist submit "
			    "failed (rc=%d)\n", ec->bdev.name, rc);
		ec->create_in_progress = false;
		ctx->deferred_rc = rc;
		spdk_bdev_unregister(&ec->bdev, ec_bdev_create_unregister_done,
				     ctx);
	}
}

static void
ec_bdev_create_wib_done(void *cb_arg, int rc)
{
	struct ec_bdev_create_async_ctx *ctx = cb_arg;
	struct ec_bdev                  *ec  = ctx->ec;

	/* Runs as the last statement of the WIB-load completion, so draining
	 * (which may free ec) is safe here. */
	if (ec->destructing) {
		ec_bdev_create_abort(ctx);
		ec_drain_deferred_slot_releases(ec);
		ec_drain_deferred_unregister(ec);
		return;
	}

	if (rc != 0) {
		/*
		 * The WIB could not be loaded (allocation failure). The startup
		 * scrub cannot run, and the degraded-read guard would see an
		 * empty (all-clean) in-memory WIB -- so any region left torn by a
		 * prior crash would be reconstructed from stale parity instead of
		 * rejected. Fail the create rather than expose the volume with its
		 * write-hole protection silently disabled. The on-disk write-intent
		 * is left intact (teardown never writes the WIB), so a retry loads
		 * and scrubs it. This matches how every other create-time
		 * allocation failure is handled.
		 */
		SPDK_ERRLOG("EC bdev %s: WIB load failed (rc=%d); failing create "
			    "to preserve on-disk write-intent for a retry\n",
			    ec->bdev.name, rc);
		ec->create_in_progress = false;
		ctx->deferred_rc = rc;
		spdk_bdev_unregister(&ec->bdev, ec_bdev_create_unregister_done,
				     ctx);
		return;
	}

	/*
	 * Established volume came up with no valid WIB on any parity disk
	 * (wib_generation is 0 only when no valid copy was found -- a written
	 * copy is always generation >= 1). We don't know which regions were
	 * mid-write at the crash, so mark every region dirty and let the startup
	 * scrub re-encode all parity from the data disks.
	 *
	 * salvage_requested tells an established volume apart from a fresh create,
	 * which has no WIB yet and correctly stays clean. We scrub rather than
	 * fail the create because the WIB is only a scrub hint and a scrub fully
	 * rebuilds it -- unlike the unmapped bitmap, which is authoritative and
	 * must fail loud when lost.
	 */
	if (ec->wib_generation == 0 && ctx->salvage_requested) {
		uint32_t region;

		SPDK_WARNLOG("EC bdev %s: salvage requested but no valid WIB found "
			     "on any parity disk; marking all %u regions dirty for "
			     "a full-volume scrub\n",
			     ec->bdev.name, ec->wib_num_regions);

		for (region = 0; region < ec->wib_num_regions; region++) {
			ec_wib_region_set_dirty(ec, region);
		}
	}

	ec_bitmap_load_async(ec, ec_bdev_create_bitmap_load_done, ctx);
}

/*
 * Non-blocking EC bdev creation. All synchronous steps (geometry,
 * wib_chans, bdev_register) complete on the caller's stack; the async
 * WIB read is driven by ec_wib_load_async and done_fn is called once
 * the bdev is fully ready.
 *
 * Returns 0 if the async operation was started (done_fn WILL be called).
 * Returns negative errno on immediate failure (done_fn is NOT called).
 */
int
ec_bdev_create_async(const char *name, uint32_t strip_size_kb, uint32_t k, uint32_t m,
		     const char **base_bdev_names, const struct spdk_uuid *uuid,
		     bool salvage_requested,
		     ec_bdev_create_cb_fn done_fn, void *done_arg)
{
	struct ec_bdev                  *ec;
	struct ec_bdev_create_async_ctx *ctx;
	int      rc;
	uint32_t i, j;
	bool     io_device_registered = false;

	rc = _ec_bdev_create(name, strip_size_kb, k, m, uuid, &ec);
	if (rc != 0) {
		return rc;
	}

	/*
	 * Capture this thread as the home thread. The subsequent
	 * spdk_bdev_get_io_channel calls all run here, so the cached
	 * bitmap_chans[] / wib_chans[] are bound to this thread. Only this
	 * thread writes the persist coordination state
	 * (bitmap_persist_in_flight, bitmap_active_copy, bitmap_generation,
	 * pending_bit_clears); off-thread callers route their work here via
	 * spdk_thread_send_msg.
	 */
	ec->home_thread = spdk_get_thread();

	if (spdk_uuid_is_null(&ec->bdev.uuid)) {
		spdk_uuid_generate(&ec->bdev.uuid);
	}

	rc = ec_open_base_bdevs(ec, base_bdev_names);
	if (rc != 0) {
		goto error_cleanup;
	}

	rc = ec_compute_geometry(ec);
	if (rc != 0) {
		goto error_cleanup;
	}

	rc = ec_alloc_runtime_arrays(ec);
	if (rc != 0) {
		goto error_cleanup;
	}

	/* Open dedicated WIB channels for each parity disk. */
	for (j = 0; j < ec->m; j++) {
		uint32_t pslot = ec->k + j;

		if (!ec->descs[pslot]) {
			ec->wib_chans[j] = NULL;
			continue;
		}
		ec->wib_chans[j] = spdk_bdev_get_io_channel(ec->descs[pslot]);
		if (!ec->wib_chans[j]) {
			SPDK_ERRLOG("EC bdev %s: failed to open WIB channel "
				    "for parity slot %u\n", name, pslot);
			rc = -ENOMEM;
			goto error_cleanup;
		}
	}

	/*
	 * Open dedicated bitmap channels for every disk. The in-band
	 * unmapped bitmap is raw-replicated to all n disks (unlike the
	 * WIB which lives only on the m parity disks), so the channel
	 * array is n entries wide.
	 */
	for (i = 0; i < ec->n; i++) {
		if (!ec->descs[i]) {
			ec->bitmap_chans[i] = NULL;
			continue;
		}
		ec->bitmap_chans[i] = spdk_bdev_get_io_channel(ec->descs[i]);
		if (!ec->bitmap_chans[i]) {
			SPDK_ERRLOG("EC bdev %s: failed to open bitmap channel "
				    "for slot %u\n", name, i);
			rc = -ENOMEM;
			goto error_cleanup;
		}
	}

	ec->wib_poller = spdk_poller_register(ec_wib_idle_poller_cb, ec,
					      EC_WIB_POLL_PERIOD_US);
	if (!ec->wib_poller) {
		SPDK_ERRLOG("EC bdev %s: failed to register WIB poller\n", name);
		rc = -ENOMEM;
		goto error_cleanup;
	}

	spdk_io_device_register(ec, ec_create_ch, ec_destroy_ch,
				sizeof(struct ec_io_channel), name);
	io_device_registered = true;

	/*
	 * Allocate the create context before spdk_bdev_register, so a calloc
	 * failure cannot leave a registered bdev without the async load chain
	 * that drives the fresh-create vs. salvage-fail decision.
	 */
	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		SPDK_ERRLOG("EC bdev %s: OOM for create ctx\n", name);
		rc = -ENOMEM;
		goto error_cleanup;
	}

	rc = spdk_bdev_register(&ec->bdev);
	if (rc != 0) {
		SPDK_ERRLOG("Failed to register EC bdev %s: %s\n",
			    name, spdk_strerror(-rc));
		/*
		 * ctx belongs to this function only until ec_wib_load_async takes
		 * it below; register failure is the one error path inside that
		 * window, so free it here.
		 */
		free(ctx);
		goto error_cleanup;
	}
	ec->bdev_registered = true;
	/* The bdev is discoverable now, but the create chain still owns the
	 * dedicated channels until examine-done; defer teardown and reject delete
	 * until then. */
	ec->create_in_progress = true;

	ctx->ec                = ec;
	ctx->done_fn           = done_fn;
	ctx->done_arg          = done_arg;
	ctx->salvage_requested = salvage_requested;

	/*
	 * Hand off to the async WIB load. The reactor is free to process
	 * other RPCs while the parity disk reads complete.
	 * done_fn is called from ec_bdev_create_wib_done().
	 */
	ec_wib_load_async(ec, ec_bdev_create_wib_done, ctx);
	return 0;

error_cleanup:
	ec_release_dedicated_channels(ec);
	TAILQ_REMOVE(&g_ec_bdev_list, ec, link);
	if (io_device_registered) {
		spdk_io_device_unregister(ec, ec_device_unregister_done);
	} else {
		ec_close_base_bdevs(ec);
		ec_bdev_free(ec);
	}
	return rc;
}

/*
 * Release the dedicated WIB and bitmap I/O channels and stop the WIB idle
 * poller. Each channel is NULL-checked and NULLed, so this is safe on both
 * the create-error and device-unregister paths.
 */
static void
ec_release_dedicated_channels(struct ec_bdev *ec)
{
	uint32_t i;

	if (ec->wib_poller) {
		spdk_poller_unregister(&ec->wib_poller);
	}
	for (i = 0; i < ec->m; i++) {
		if (ec->wib_chans[i]) {
			spdk_put_io_channel(ec->wib_chans[i]);
			ec->wib_chans[i] = NULL;
		}
	}
	for (i = 0; i < ec->n; i++) {
		if (ec->bitmap_chans[i]) {
			spdk_put_io_channel(ec->bitmap_chans[i]);
			ec->bitmap_chans[i] = NULL;
		}
	}
}

/*
 * Tail of device-unregister teardown: release dedicated channels, abort any
 * startup scrub, close base descriptors, complete the destruct protocol, and
 * free the ec_bdev. Split out so the deferred path (a persist was still in
 * flight when the unregister callback fired) can re-enter here from the
 * persist drain.
 */
static void
ec_finish_device_unregister(struct ec_bdev *ec)
{
	ec_release_dedicated_channels(ec);

	/* Abort any in-progress startup scrub. */
	if (ec->scrub_ctx) {
		if (ec->scrub_ctx->poller) {
			spdk_poller_unregister(&ec->scrub_ctx->poller);
		}
		ec_scrub_free_resources(ec->scrub_ctx);
		free(ec->scrub_ctx);
		ec->scrub_ctx = NULL;
	}

	/* All per-thread channels destroyed; safe to close descriptors. */
	ec_close_base_bdevs(ec);

	/*
	 * Only complete the destruct protocol for a bdev that was actually
	 * registered. The create-failure teardown path reaches this same
	 * io_device-unregister callback (to release the io_device) before
	 * spdk_bdev_register has succeeded; calling spdk_bdev_destruct_done on
	 * a never-registered bdev is a bdev-layer protocol violation.
	 */
	if (ec->bdev_registered) {
		spdk_bdev_destruct_done(&ec->bdev, 0);
	}
	ec_bdev_free(ec);
}

static void
ec_device_unregister_done(void *io_device)
{
	struct ec_bdev *ec = io_device;

	/*
	 * A persist may still have a write outstanding on a WIB / bitmap
	 * channel; releasing those channels now would trip the bdev-layer
	 * io_outstanding assert. Defer the rest of teardown until the persist
	 * drains (ec_drain_deferred_unregister resumes it).
	 */
	if (ec_teardown_must_defer(ec)) {
		ec->unregister_release_pending = true;
		return;
	}

	ec_finish_device_unregister(ec);
}

/*
 * Release dedicated channels for slots whose failure cleanup was deferred
 * behind an in-flight persist, and start their async cleanup. Never frees ec,
 * so a completion may call this before kicking its follow-up persist -- which
 * is what keeps sustained WIB churn on the surviving disks from starving the
 * cleanup. A released slot is simply skipped by the next persist (the submit
 * loops NULL-check channels).
 *
 * Skipped entirely while a delete is pending: ec_finish_device_unregister
 * releases every channel itself, and starting a per-slot quiesce against a
 * bdev whose unregister is about to free ec would fire the quiesce callback
 * on freed memory.
 */
void
ec_drain_deferred_slot_releases(struct ec_bdev *ec)
{
	uint32_t i;

	if (ec_teardown_must_defer(ec) || ec->unregister_release_pending) {
		return;
	}

	for (i = 0; i < ec->n; i++) {
		if (ec->dedicated_release_pending[i]) {
			ec->dedicated_release_pending[i] = false;
			ec_release_slot_dedicated_channels(ec, i);
			ec_start_base_bdev_cleanup(ec, i);
		}
	}
}

/*
 * Finish a device-unregister that was deferred behind an in-flight persist.
 * Frees ec, so the caller MUST treat this as its last statement. The unregister
 * tail releases every dedicated channel and closes every base descriptor, so
 * any pending per-slot release is redundant -- clear those flags and skip the
 * per-slot cleanup, which would otherwise quiesce a bdev that is being freed.
 */
void
ec_drain_deferred_unregister(struct ec_bdev *ec)
{
	uint32_t i;

	if (ec_teardown_must_defer(ec) || !ec->unregister_release_pending) {
		return;
	}

	ec->unregister_release_pending = false;
	for (i = 0; i < ec->n; i++) {
		ec->dedicated_release_pending[i] = false;
	}
	ec_finish_device_unregister(ec);
}

static int
ec_destruct(void *ctx)
{
	struct ec_bdev *ec = ctx;

	/* Tell the create chain to abort before teardown proceeds. Home-thread
	 * only, so the create chain (also home) sees it on its next step. */
	assert(spdk_get_thread() == ec->home_thread);
	ec->destructing = true;

	TAILQ_REMOVE(&g_ec_bdev_list, ec, link);

	/*
	 * Async: ec_destroy_ch runs on every thread to put base_chans[] refs.
	 * ec_device_unregister_done fires when all channels are torn down.
	 */
	spdk_io_device_unregister(ec, ec_device_unregister_done);
	return 1;  /* async destruct */
}

/* =========================================================================
 * bdev fn_table callbacks
 * ========================================================================= */

/*
 * Advertises which I/O types the EC bdev accepts to the SPDK bdev layer.
 *
 * This switch is the consumer-facing contract; g_ec_submit_dispatch[]
 * (defined further down in this file) is the implementation that handles
 * each accepted type. Both MUST be edited together when adding or
 * removing an I/O type:
 *   - new "return true" arm here   <-> add a row to g_ec_submit_dispatch
 *   - new "return false" arm here  <-> ensure the dispatch row is absent
 *                                       or points at a reject helper
 *                                       (see ec_submit_reject_write_zeroes)
 */
static bool
ec_io_type_supported(void *ctx, enum spdk_bdev_io_type type)
{
	(void)ctx;

	switch (type) {
	case SPDK_BDEV_IO_TYPE_READ:
	case SPDK_BDEV_IO_TYPE_WRITE:
		return true;
	case SPDK_BDEV_IO_TYPE_WRITE_ZEROES:
		/*
		 * Native WRITE_ZEROES is intentionally NOT advertised.
		 *
		 * The SPDK bdev layer splits WRITE_ZEROES purely by size
		 * (max_write_zeroes) and does not honor optimal_io_boundary,
		 * so a sub-stripe-aligned WRITE_ZEROES can straddle a stripe
		 * boundary and arrive at the RMW path with
		 * (stripe_off_blocks + num_blocks) > stripe_blocks. That
		 * overruns the per-stripe scratch buffer and corrupts the
		 * heap.
		 *
		 * Returning false here lets the bdev layer auto-emulate
		 * WRITE_ZEROES as a regular zero-buffer WRITE, which then
		 * goes through optimal_io_boundary splitting like any other
		 * write and lands on the RMW / full-stripe paths correctly
		 * bounded. UNMAP retains the is_zero_fill RMW shortcut
		 * because it sets the flag itself (bdev_ec_unmap.c).
		 */
		return false;
	case SPDK_BDEV_IO_TYPE_UNMAP:
		/*
		 * Native UNMAP is always supported under the in-band
		 * unmapped-bitmap design: correctness comes from the EC
		 * layer's own per-stripe bitmap (every read consults it
		 * before issuing base I/O), independent of whether any
		 * particular base bdev deallocates-to-zero on discard. The
		 * physical UNMAP fan-out is best-effort space reclamation;
		 * a slot that fails to reclaim just wastes space, it cannot
		 * resurrect non-zero data because no reader trusts a
		 * discarded range. See bdev_ec_unmap.c.
		 */
		return true;
	default:
		return false;
	}
}

static struct spdk_io_channel *
ec_get_io_channel(void *ctx)
{
	struct ec_bdev *ec = ctx;
	return spdk_get_io_channel(ec);
}

/*
 * Resolve a live base bdev name for a non-FAILED slot, or "<unknown>"
 * if the descriptor is missing (state machine inconsistency) or the
 * descriptor has no backing bdev. Caller writes "<failed>" directly
 * for FAILED slots, so this helper covers only NORMAL/REPLACING.
 */
static const char *
ec_slot_name_for_json(const struct ec_bdev *ec, uint32_t slot)
{
	struct spdk_bdev *base;

	if (!ec->descs[slot]) {
		return "<unknown>";
	}
	base = spdk_bdev_desc_get_bdev(ec->descs[slot]);
	return base ? spdk_bdev_get_name(base) : "<unknown>";
}

/*
 * ec_write_base_bdevs_array_json -- write the "base_bdevs": [...] array.
 *
 * Used by both ec_dump_info_json (bdev_get_bdevs path) and
 * rpc_bdev_ec_get_bdevs (bdev_ec_get_bdevs RPC). Per-slot fields:
 *
 *   slot           uint32
 *   role           "data" | "parity"
 *   name           live base bdev name, or "<failed>" / "<unknown>"
 *   state          "normal" | "failed" | "replacing"
 *   needs_rebuild  bool   (only when state == "replacing")
 *
 * When descs[i] is NULL on a non-FAILED slot (state machine
 * inconsistency, should not happen in normal operation), writes
 * name="<unknown>" defensively.
 */
void
ec_write_base_bdevs_array_json(struct spdk_json_write_ctx *w,
			       const struct ec_bdev *ec)
{
	uint32_t i;

	spdk_json_write_named_array_begin(w, "base_bdevs");
	for (i = 0; i < ec->n; i++) {
		spdk_json_write_object_begin(w);
		spdk_json_write_named_uint32(w, "slot", i);
		spdk_json_write_named_string(w, "role",
					     i < ec->k ? "data" : "parity");

		switch (ec->base_states[i]) {
		case EC_BASE_STATE_FAILED:
			spdk_json_write_named_string(w, "name",  "<failed>");
			spdk_json_write_named_string(w, "state", "failed");
			break;
		case EC_BASE_STATE_REPLACING:
			spdk_json_write_named_string(w, "name",
				ec_slot_name_for_json(ec, i));
			spdk_json_write_named_string(w, "state", "replacing");
			spdk_json_write_named_bool(w,   "needs_rebuild",
				ec->needs_rebuild[i]);
			break;
		case EC_BASE_STATE_NORMAL:
		default:
			spdk_json_write_named_string(w, "name",
				ec_slot_name_for_json(ec, i));
			spdk_json_write_named_string(w, "state", "normal");
			break;
		}
		spdk_json_write_object_end(w);
	}
	spdk_json_write_array_end(w);
}

/*
 * ec_write_rebuild_progress_json -- write the named "rebuild_progress"
 * object describing a live rebuild_ctx. Caller must have verified
 * ec->rebuild_ctx != NULL.
 */
void
ec_write_rebuild_progress_json(struct spdk_json_write_ctx *w,
			       const struct ec_bdev *ec)
{
	const struct ec_rebuild_ctx *rctx = ec->rebuild_ctx;

	spdk_json_write_named_object_begin(w, "rebuild_progress");
	spdk_json_write_named_uint32(w, "current_slot",    rctx->current_slot);
	spdk_json_write_named_uint64(w, "current_stripe",  rctx->current_stripe);
	spdk_json_write_named_uint64(w, "num_stripes",     rctx->num_stripes);
	spdk_json_write_named_uint64(w, "stripes_rebuilt", rctx->stripes_rebuilt);
	spdk_json_write_object_end(w);
}

/*
 * ec_write_io_stats_json -- write the RMW / full-stripe-write / UNMAP /
 * degraded-read counter run shared verbatim by ec_dump_info_json and the
 * bdev_ec_get_bdevs RPC. Only this common, identically-ordered run is
 * factored out; each caller writes its own trailing fields afterward.
 */
void
ec_write_io_stats_json(struct spdk_json_write_ctx *w, const struct ec_bdev *ec)
{
	spdk_json_write_named_uint32(w, "rmw_in_flight", ec_rmw_in_flight_get(ec));
	if (ec->stripe_dirty_map) {
		spdk_json_write_named_uint64(w, "dirty_stripes",
					     ec_stripe_count_dirty(ec));
	}
	spdk_json_write_named_uint64(w, "rmw_total",
				     ec->rmw_total);
	spdk_json_write_named_uint64(w, "rmw_deferred_scrub",
				     ec->rmw_deferred_scrub);
	spdk_json_write_named_uint64(w, "rmw_deferred_dirty",
				     ec->rmw_deferred_dirty);
	spdk_json_write_named_uint64(w, "rmw_deferred_inflight",
				     ec->rmw_deferred_inflight);
	spdk_json_write_named_uint64(w, "full_stripe_writes",
				     ec->full_stripe_writes);
	spdk_json_write_named_uint64(w, "full_stripe_writes_deferred",
				     ec->full_stripe_writes_deferred);
	spdk_json_write_named_uint64(w, "unmaps_submitted",
				     __atomic_load_n(&ec->unmaps_submitted, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "unmaps_completed",
				     __atomic_load_n(&ec->unmaps_completed, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "unmaps_deferred_busy",
				     ec->unmaps_deferred_busy);
	spdk_json_write_named_uint64(w, "unmaps_via_write_zeros",
				     ec->unmaps_via_write_zeros);
	spdk_json_write_named_uint64(w, "unmaps_failed",
				     __atomic_load_n(&ec->unmaps_failed, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "unmap_fanout_misses",
				     __atomic_load_n(&ec->unmap_fanout_misses, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "unmapped_reads_synthesized",
				     __atomic_load_n(&ec->unmapped_reads_synthesized, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "writes_into_unmapped",
				     __atomic_load_n(&ec->writes_into_unmapped, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "writes_into_unmapped_failed",
				     __atomic_load_n(&ec->writes_into_unmapped_failed, __ATOMIC_RELAXED));
	spdk_json_write_named_uint64(w, "unmapped_stripes",
				     ec_count_unmapped_stripes(ec));
	spdk_json_write_named_uint64(w, "degraded_reads_reconstructed",
				     __atomic_load_n(&ec->degraded_reads_reconstructed, __ATOMIC_RELAXED));
}

static int
ec_dump_info_json(void *ctx, struct spdk_json_write_ctx *w)
{
	struct ec_bdev *ec = ctx;

	spdk_json_write_named_object_begin(w, "ec");
	spdk_json_write_named_uint32(w, "data_chunk_count",   ec->k);
	spdk_json_write_named_uint32(w, "parity_chunk_count", ec->m);
	spdk_json_write_named_uint32(w, "strip_size_kb",      ec->strip_size_kb);
	spdk_json_write_named_uint32(w, "num_base_bdevs",     ec->n);
	spdk_json_write_named_uint32(w, "failed_count",       ec->failed_count);
	spdk_json_write_named_bool(w,   "offline",            ec->offline);
	spdk_json_write_named_bool(w,   "replace_in_progress", ec->replace_in_progress);

	ec_write_io_stats_json(w, ec);

	spdk_json_write_named_bool(w, "rebuild_in_progress", ec->rebuild_ctx != NULL);
	if (ec->rebuild_ctx) {
		ec_write_rebuild_progress_json(w, ec);
	}

	ec_write_base_bdevs_array_json(w, ec);

	spdk_json_write_named_uint32(w, "wib_num_regions",     ec->wib_num_regions);
	spdk_json_write_named_uint64(w, "wib_generation",      ec->wib_generation);
	/* JSON key remains "wib_persist_pending" for wire compatibility;
	 * the C field is wib_persist_in_flight (true until persist completes
	 * end-to-end). */
	spdk_json_write_named_bool(w,   "wib_persist_pending",  ec->wib_persist_in_flight);
	if (ec->wib_region_map) {
		spdk_json_write_named_uint32(w, "wib_dirty_regions",
					     ec_wib_count_dirty(ec));
	}

	spdk_json_write_named_uint64(w, "degraded_read_eio_dirty",
				     __atomic_load_n(&ec->degraded_read_eio_dirty, __ATOMIC_RELAXED));
	spdk_json_write_named_bool(w, "scrub_in_progress", ec->scrub_ctx != NULL);
	if (ec->scrub_ctx) {
		struct ec_scrub_ctx *sctx = ec->scrub_ctx;
		spdk_json_write_named_object_begin(w, "scrub_progress");
		spdk_json_write_named_uint32(w, "current_region",    sctx->current_region);
		spdk_json_write_named_uint32(w, "num_regions",       ec->wib_num_regions);
		spdk_json_write_named_uint64(w, "current_stripe",    sctx->current_stripe);
		spdk_json_write_named_uint64(w, "stripes_scrubbed",  sctx->stripes_scrubbed);
		spdk_json_write_named_uint64(w, "regions_scrubbed",  sctx->regions_scrubbed);
		spdk_json_write_object_end(w);
	}

	spdk_json_write_object_end(w);
	return 0;
}

static void
ec_write_config_json(struct spdk_bdev *bdev, struct spdk_json_write_ctx *w)
{
	struct ec_bdev   *ec = SPDK_CONTAINEROF(bdev, struct ec_bdev, bdev);
	struct spdk_bdev *base_bdev;
	uint32_t          i;

	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "method", "bdev_ec_create");
	spdk_json_write_named_object_begin(w, "params");
	spdk_json_write_named_string(w, "name",               bdev->name);
	spdk_json_write_named_uint32(w, "data_chunk_count",   ec->k);
	spdk_json_write_named_uint32(w, "parity_chunk_count", ec->m);
	spdk_json_write_named_uint32(w, "strip_size_kb",      ec->strip_size_kb);
	spdk_json_write_named_array_begin(w, "base_bdevs");
	for (i = 0; i < ec->n; i++) {
		if (ec->descs[i]) {
			base_bdev = spdk_bdev_desc_get_bdev(ec->descs[i]);
			if (base_bdev) {
				spdk_json_write_string(w, spdk_bdev_get_name(base_bdev));
			}
		}
	}
	spdk_json_write_array_end(w);
	spdk_json_write_object_end(w);
	spdk_json_write_object_end(w);
}

/*
 * RESET / FLUSH have no payload at the EC layer: complete immediately
 * with SUCCESS. Returning 0 keeps the dispatch table's "non-zero rc =>
 * status mapping" invariant uniform across every entry.
 */
static int
ec_submit_noop_success(struct ec_bdev_io *ec_io)
{
	spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_SUCCESS);
	return 0;
}

/*
 * Defensive: ec_io_type_supported returns false for WRITE_ZEROES so the
 * bdev layer always emulates it as a buffer-backed WRITE. Reaching this
 * dispatch entry means that contract changed and the RMW heap-overflow
 * regression is back -- fail loudly instead of corrupting memory.
 */
static int
ec_submit_reject_write_zeroes(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);

	SPDK_ERRLOG("EC bdev %s: unexpected native WRITE_ZEROES "
		    "(emulation not engaged)\n", ec->bdev.name);
	return -EINVAL;
}

/*
 * Type-indexed dispatch table for ec_submit_request. This is the
 * implementation side of the contract advertised by
 * ec_io_type_supported above; the two MUST be edited together (see the
 * comment above ec_io_type_supported for the pairing rules).
 * NULL entries are unsupported types: the dispatcher logs and fails them.
 */
typedef int (*ec_io_submit_fn)(struct ec_bdev_io *ec_io);

static const ec_io_submit_fn g_ec_submit_dispatch[] = {
	[SPDK_BDEV_IO_TYPE_READ]         = ec_submit_read,
	[SPDK_BDEV_IO_TYPE_WRITE]        = ec_submit_write,
	[SPDK_BDEV_IO_TYPE_WRITE_ZEROES] = ec_submit_reject_write_zeroes,
	[SPDK_BDEV_IO_TYPE_UNMAP]        = ec_submit_unmap,
	[SPDK_BDEV_IO_TYPE_RESET]        = ec_submit_noop_success,
	[SPDK_BDEV_IO_TYPE_FLUSH]        = ec_submit_noop_success,
};

static void
ec_submit_request(struct spdk_io_channel *ch, struct spdk_bdev_io *bdev_io)
{
	struct ec_bdev_io    *ec_io = (struct ec_bdev_io *)bdev_io->driver_ctx;
	struct ec_io_channel *ec_ch = spdk_io_channel_get_ctx(ch);
	struct ec_bdev       *ec    = ec_from_bdev_io(bdev_io);
	ec_io_submit_fn       submit;
	int                   rc;

	ec_bdev_io_init(ec_io, ec_ch, bdev_io);

	if (ec->offline) {
		SPDK_DEBUGLOG(bdev_ec, "EC bdev %s OFFLINE -- rejecting I/O\n", ec->bdev.name);
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	if ((size_t)bdev_io->type >= SPDK_COUNTOF(g_ec_submit_dispatch) ||
	    g_ec_submit_dispatch[bdev_io->type] == NULL) {
		SPDK_ERRLOG("EC bdev %s: unsupported IO type %d\n",
			    ec->bdev.name, bdev_io->type);
		spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	submit = g_ec_submit_dispatch[bdev_io->type];
	rc     = submit(ec_io);
	if (rc != 0) {
		/*
		 * -EAGAIN: RMW stripe dirty conflict -- requeue via NOMEM.
		 * -ENOMEM: allocation failure -- requeue via NOMEM.
		 * Other:   hard failure.
		 */
		if (rc == -EAGAIN || rc == -ENOMEM) {
			spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_NOMEM);
		} else {
			spdk_bdev_io_complete(bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		}
	}
}

static const struct spdk_bdev_fn_table g_ec_fn_table = {
	.destruct          = ec_destruct,
	.submit_request    = ec_submit_request,
	.io_type_supported = ec_io_type_supported,
	.get_io_channel    = ec_get_io_channel,
	.dump_info_json    = ec_dump_info_json,
	.write_config_json = ec_write_config_json,
};

void
ec_bdev_delete(const char *name, spdk_bdev_unregister_cb cb_fn, void *cb_arg)
{
	struct ec_bdev *ec;
	int             rc;

	/*
	 * Refuse delete while a create, rebuild, or resize is in progress.
	 *
	 * Rebuild and resize own pollers and channels that the unregister path does
	 * not stop, so deleting under them frees the ec_bdev while a poller is still
	 * scheduled -- a use-after-free. Create is already crash-safe (the create
	 * chain aborts if a delete slips through); rejecting here just avoids
	 * cancelling an in-flight create, so the caller can retry.
	 */
	ec = ec_bdev_find(name);
	if (ec != NULL) {
		if (ec->create_in_progress) {
			SPDK_ERRLOG("EC bdev %s: delete rejected -- "
				    "create still in progress\n", name);
			cb_fn(cb_arg, -EBUSY);
			return;
		}
		if (ec->rebuild_ctx != NULL) {
			SPDK_ERRLOG("EC bdev %s: delete rejected -- "
				    "rebuild in progress\n", name);
			cb_fn(cb_arg, -EBUSY);
			return;
		}
		if (ec->resize_ctx != NULL) {
			SPDK_ERRLOG("EC bdev %s: delete rejected -- "
				    "resize in progress\n", name);
			cb_fn(cb_arg, -EBUSY);
			return;
		}
	}

	rc = spdk_bdev_unregister_by_name(name, &ec_if, cb_fn, cb_arg);
	if (rc != 0) {
		cb_fn(cb_arg, rc);
	}
}

/* =========================================================================
 * ec_bdev_replace_base_bdev -- hot-swap a failed disk slot
 * ========================================================================= */

struct ec_replace_chan_ctx {
	struct ec_replace_ctx *rctx;
	int                    rc;   /* failure rc carried through the release walk */
};

static void ec_replace_acquire_chan_iter(struct spdk_io_channel_iter *i);
static void ec_replace_chan_walk_done(struct spdk_io_channel_iter *i, int status);
static void ec_replace_finish(struct ec_replace_ctx *rctx, int rc);
static void ec_replace_start_channel_walk(struct ec_replace_ctx *rctx);
static void ec_replace_release_chan_iter(struct spdk_io_channel_iter *i);
static void ec_replace_release_done(struct spdk_io_channel_iter *i, int status);
static void ec_replace_finish_failed(struct ec_replace_ctx *rctx, int rc);


static void
ec_replace_finish(struct ec_replace_ctx *rctx, int rc)
{
	struct ec_bdev *ec   = rctx->ec;
	uint32_t        slot = rctx->slot;

	ec->replace_in_progress = false;

	if (rc == 0) {
		/*
		 * Rejoin the hot-swapped slot to the bitmap quorum. Its bitmap
		 * channel was released when the old disk failed
		 * (ec_release_slot_dedicated_channels) and is reopened nowhere else,
		 * so without this the slot is skipped by every bitmap persist for the
		 * life of the bdev. Writing it while REPLACING is safe: the bitmap is
		 * whole-volume metadata and the rebuilder never touches the front
		 * reservation region.
		 */
		if (!ec->bitmap_chans[slot] && ec->descs[slot]) {
			ec->bitmap_chans[slot] =
				spdk_bdev_get_io_channel(ec->descs[slot]);
			if (!ec->bitmap_chans[slot]) {
				SPDK_WARNLOG("EC bdev %s: failed to reopen bitmap "
					     "channel for replacement slot %u; slot "
					     "will not receive bitmap persists until "
					     "reload\n", ec->bdev.name, slot);
			}
		}

		/*
		 * Overwrite both bitmap copies on the new slot so a stale/foreign
		 * blob it may carry cannot out-rank ours on a later load, and the
		 * committed blob is back on the m+1 copies the stamp attests --
		 * same protection as fresh create.
		 */
		if (ec->bitmap_chans[slot]) {
			ec_bitmap_resync_after_replace(ec);
		}

		SPDK_NOTICELOG("EC bdev %s: slot %u hot-swap complete -- "
			       "disk '%s' is REPLACING; needs_rebuild=true. "
			       "Run bdev_ec_start_rebuild to restore NORMAL.\n",
			       ec->bdev.name, slot, rctx->new_bdev_name);
		rctx->cb_fn(rctx->cb_arg, rc);
		free(rctx);
		return;
	}

	SPDK_ERRLOG("EC bdev %s: hot-swap of slot %u failed (rc=%d); "
		    "releasing channels and reverting to FAILED\n",
		    ec->bdev.name, slot, rc);

	/*
	 * The acquire walk may have set base_chans[slot] on threads it visited
	 * before failing. Those channels reference rctx->new_desc; release them
	 * on every thread before closing the descriptor. Closing a descriptor
	 * with live channels leaves the next replace's acquire iter putting a
	 * freed channel (use-after-free), so the close must wait for the walk.
	 */
	{
		struct ec_replace_chan_ctx *cctx = calloc(1, sizeof(*cctx));

		if (cctx != NULL) {
			cctx->rctx = rctx;
			cctx->rc   = rc;
			spdk_for_each_channel(ec, ec_replace_release_chan_iter,
					      cctx, ec_replace_release_done);
			return;
		}
	}

	/*
	 * Best-effort fallback if even the tiny release-walk ctx cannot be
	 * allocated (vanishingly unlikely during an already-failing replace):
	 * close synchronously and accept the rare dangling per-thread channel.
	 */
	SPDK_ERRLOG("EC bdev %s: OOM for replace release walk on slot %u; "
		    "closing descriptor best-effort\n", ec->bdev.name, slot);
	ec_replace_finish_failed(rctx, rc);
}

/*
 * Put base_chans[slot] on the running thread, if the acquire walk had set it,
 * so the descriptor can be closed safely once the walk completes.
 */
static void
ec_replace_release_chan_iter(struct spdk_io_channel_iter *i)
{
	struct ec_replace_chan_ctx *cctx  = spdk_io_channel_iter_get_ctx(i);
	uint32_t                    slot  = cctx->rctx->slot;
	struct spdk_io_channel     *ch    = spdk_io_channel_iter_get_channel(i);
	struct ec_io_channel       *ec_ch = spdk_io_channel_get_ctx(ch);

	if (ec_ch->base_chans[slot]) {
		spdk_put_io_channel(ec_ch->base_chans[slot]);
		ec_ch->base_chans[slot] = NULL;
	}

	spdk_for_each_channel_continue(i, 0);
}

static void
ec_replace_release_done(struct spdk_io_channel_iter *i, int status)
{
	struct ec_replace_chan_ctx *cctx = spdk_io_channel_iter_get_ctx(i);
	struct ec_replace_ctx      *rctx = cctx->rctx;
	int                         rc   = cctx->rc;

	(void)status;
	free(cctx);
	ec_replace_finish_failed(rctx, rc);
}

/* Close the replacement descriptor, revert the slot to FAILED, report rc. */
static void
ec_replace_finish_failed(struct ec_replace_ctx *rctx, int rc)
{
	struct ec_bdev *ec   = rctx->ec;
	uint32_t        slot = rctx->slot;

	/*
	 * descs[slot] is the sole owner of new_desc (set in
	 * ec_replace_start_channel_walk). A NULL here means the failure-cleanup
	 * chain for a replacement disk that died mid-replace already closed it --
	 * closing new_desc again would double-close.
	 */
	if (ec->descs[slot] != NULL) {
		spdk_bdev_close(ec->descs[slot]);
		ec->descs[slot] = NULL;
	}

	ec->base_states[slot]   = EC_BASE_STATE_FAILED;
	ec->needs_rebuild[slot] = false;

	rctx->cb_fn(rctx->cb_arg, rc);
	free(rctx);
}

static void
ec_replace_chan_walk_done(struct spdk_io_channel_iter *i, int status)
{
	struct ec_replace_chan_ctx *cctx = spdk_io_channel_iter_get_ctx(i);
	struct ec_replace_ctx      *rctx = cctx->rctx;

	free(cctx);
	ec_replace_finish(rctx, status);
}

static void
ec_replace_acquire_chan_iter(struct spdk_io_channel_iter *i)
{
	struct ec_replace_chan_ctx *cctx  = spdk_io_channel_iter_get_ctx(i);
	struct ec_replace_ctx      *rctx  = cctx->rctx;
	struct ec_bdev             *ec    = rctx->ec;
	uint32_t                    slot  = rctx->slot;
	struct spdk_io_channel     *ch    = spdk_io_channel_iter_get_channel(i);
	struct ec_io_channel       *ec_ch = spdk_io_channel_get_ctx(ch);

	if (ec_ch->base_chans[slot]) {
		spdk_put_io_channel(ec_ch->base_chans[slot]);
		ec_ch->base_chans[slot] = NULL;
	}

	ec_ch->base_chans[slot] = spdk_bdev_get_io_channel(ec->descs[slot]);
	if (!ec_ch->base_chans[slot]) {
		SPDK_ERRLOG("EC bdev %s: failed to get channel for replacement "
			    "slot %u on thread %s\n",
			    ec->bdev.name, slot,
			    spdk_thread_get_name(spdk_get_thread()));
		spdk_for_each_channel_continue(i, -ENOMEM);
		return;
	}

	SPDK_DEBUGLOG(bdev_ec,
		"EC bdev %s: acquired channel for replacement slot %u "
		"on thread %s\n", ec->bdev.name, slot,
		spdk_thread_get_name(spdk_get_thread()));

	spdk_for_each_channel_continue(i, 0);
}

int
ec_bdev_replace_base_bdev(const char *ec_name, uint32_t slot,
			  const char *new_bdev_name,
			  ec_replace_cb_fn cb_fn, void *cb_arg)
{
	struct ec_bdev        *ec;
	struct ec_replace_ctx *rctx;
	struct spdk_bdev      *new_bdev;
	struct spdk_bdev      *orig_bdev;
	uint32_t               i;
	int                    rc;

	ec = ec_bdev_find(ec_name);
	if (!ec) {
		return -ENODEV;
	}

	if (slot >= ec->n) {
		return -ENOENT;
	}

	if (ec->base_states[slot] != EC_BASE_STATE_FAILED) {
		return -EINVAL;
	}

	if (ec->replace_in_progress) {
		return -EBUSY;
	}

	if (ec->rebuild_ctx != NULL) {
		/*
		 * A rebuild walks the REPLACING slots forward from a fixed
		 * start slot and, on completion, transitions every slot it
		 * believes it rebuilt (identified via needs_rebuild[]). Letting
		 * a new replacement appear mid-rebuild would let
		 * ec_rebuild_finish mark a slot NORMAL that this session never
		 * walked, silently exposing un-rebuilt data. Reject; the caller
		 * retries once the in-progress rebuild completes.
		 */
		return -EBUSY;
	}

	rctx = calloc(1, sizeof(*rctx));
	if (!rctx) {
		return -ENOMEM;
	}
	rctx->ec     = ec;
	rctx->slot   = slot;
	rctx->cb_fn  = cb_fn;
	rctx->cb_arg = cb_arg;

	if (strnlen(new_bdev_name, EC_BDEV_NAME_MAX) >= EC_BDEV_NAME_MAX) {
		free(rctx);
		return -EINVAL;
	}
	snprintf(rctx->new_bdev_name, sizeof(rctx->new_bdev_name),
		 "%s", new_bdev_name);

	rc = spdk_bdev_open_ext(new_bdev_name, true,
				ec_base_bdev_event_cb, ec,
				&rctx->new_desc);
	if (rc != 0) {
		free(rctx);
		return rc;
	}

	new_bdev = spdk_bdev_desc_get_bdev(rctx->new_desc);

	if (new_bdev->blocklen != ec->bdev.blocklen) {
		spdk_bdev_close(rctx->new_desc);
		free(rctx);
		return -EINVAL;
	}

	orig_bdev = NULL;
	for (i = 0; i < ec->n; i++) {
		if (ec->base_states[i] == EC_BASE_STATE_NORMAL && ec->descs[i]) {
			orig_bdev = spdk_bdev_desc_get_bdev(ec->descs[i]);
			break;
		}
	}
	if (orig_bdev != NULL && new_bdev->blockcnt < orig_bdev->blockcnt) {
		spdk_bdev_close(rctx->new_desc);
		free(rctx);
		return -EINVAL;
	}

	ec->replace_in_progress = true;

	ec_replace_start_channel_walk(rctx);
	return 0;
}

static void
ec_replace_start_channel_walk(struct ec_replace_ctx *rctx)
{
	struct ec_bdev             *ec   = rctx->ec;
	uint32_t                    slot = rctx->slot;
	struct ec_replace_chan_ctx *cctx;

	ec->descs[slot]         = rctx->new_desc;
	ec->base_states[slot]   = EC_BASE_STATE_REPLACING;
	ec->needs_rebuild[slot] = true;

	SPDK_NOTICELOG("EC bdev %s: slot %u -> REPLACING with '%s'; "
		       "walking channels\n",
		       ec->bdev.name, slot, rctx->new_bdev_name);

	cctx = calloc(1, sizeof(*cctx));
	if (!cctx) {
		SPDK_ERRLOG("EC bdev %s: OOM for channel walk ctx; "
			    "rolling slot %u back from REPLACING to FAILED\n",
			    ec->bdev.name, slot);
		ec->descs[slot]         = NULL;
		ec->base_states[slot]   = EC_BASE_STATE_FAILED;
		ec->needs_rebuild[slot] = false;
		ec->replace_in_progress = false;
		spdk_bdev_close(rctx->new_desc);
		rctx->cb_fn(rctx->cb_arg, -ENOMEM);
		free(rctx);
		return;
	}
	cctx->rctx = rctx;

	spdk_for_each_channel(ec,
		ec_replace_acquire_chan_iter,
		cctx,
		ec_replace_chan_walk_done);
}

SPDK_LOG_REGISTER_COMPONENT(bdev_ec)
