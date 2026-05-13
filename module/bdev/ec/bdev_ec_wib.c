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
	uint32_t             *crc_ptr;

	hdr->magic       = EC_WIB_MAGIC;
	hdr->version     = EC_WIB_VERSION;
	hdr->generation  = ec->wib_generation;
	hdr->num_regions = ec->wib_num_regions;
	hdr->_pad        = 0;

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
		SPDK_WARNLOG("EC bdev %s: WIB deferred persist failed "
			     "(rc=%d); proceeding with deferred writes "
			     "(degraded crash safety)\n",
			     ec->bdev.name, rc);
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
	free(pctx);

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
		ec_wib_deferred_drain(ec, 0);
	}
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
 * Write the current in-memory WIB region_map to the inactive on-disk copy
 * on all m parity disks. On completion, the active copy index is flipped.
 *
 * cb / cb_arg: invoked when all m writes complete. Pass NULL/NULL for
 *              fire-and-forget (e.g. the idle-clear path).
 *
 * Caller must ensure ec->wib_persist_in_flight is false before calling.
 * Sets ec->wib_persist_in_flight = true.
 *
 * Why I/O-path WIB persists (RMW, full-stripe write) route through the
 * home thread instead of running on each ec_io_channel's own wib_chans[]:
 *
 * The WIB is shared on-disk state replicated to every parity disk. Like
 * the bitmap (see "Why bitmap persists ..." above ec_bitmap_persist_async
 * in bdev_ec_bitmap.c), its consistency model needs a single writer for
 * two coordination signals:
 *
 *   - wib_active_copy. Each persist writes the slot the active copy is
 *     NOT in (next_copy = 1 - active_copy). Two writers both reading
 *     active_copy = 0 would both write slot 1; their bytes interleave
 *     and the loaded blob fails CRC.
 *   - wib_generation. Monotonic counter, incremented by exactly one
 *     writer per persist. Two writers both reading N and both writing
 *     N+1 with different content break the "highest valid generation
 *     wins" rule that the WIB load path relies on to pick a winner.
 *
 * Distributing writers across per-channel wib_chans[] would mean the
 * same coordination-protocol replacement the bitmap rationale rejects.
 * Per-channel writers would also break the m+1 quorum bookkeeping in
 * ec_wib_persist_ctx (writes_remaining), which assumes one in-flight
 * persist at a time.
 *
 * Routing is the cleaner long-term answer, but the I/O-path callers
 * (ec_submit_full_write, ec_rmw_persist_and_submit) currently consume
 * the synchronous rc from ec_wib_persist to drive rollback / NOMEM
 * retry. Translating those sync failures into cb-delivered
 * SPDK_BDEV_IO_STATUS_NOMEM completions at every caller is the same
 * contract change deferred for the UNMAP-path bitmap persist; it is
 * tracked as a focused follow-up. Until then, asserts at the call
 * sites + this function enforce the home-thread invariant: under
 * multi-reactor they abort at the exact spot the follow-up needs to
 * land. Single-reactor is unaffected.
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
		if (ec->wib_region_inflight[region] > 0) {
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
ec_wib_validate_buf(const struct ec_bdev *ec, const void *buf, uint32_t *gen_out)
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
