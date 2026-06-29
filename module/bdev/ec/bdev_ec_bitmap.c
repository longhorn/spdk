/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_bitmap.c -- in-band unmapped bitmap machinery.
 *
 * The persistent per-stripe unmapped bitmap records, for every user
 * stripe, whether it is logically zero (unmapped) or holds real data.
 * It lets the EC layer synthesize zeros for discarded ranges without
 * trusting the base bdevs to zero on discard.
 *
 * On-disk it lives in-band and raw-replicated: every base disk reserves
 * two slots (copy A / copy B) at the front of its address space, each
 * holding an identical copy of the whole bitmap blob. Crash-safety is
 * double-buffer + CRC, not EC redundancy. The blob survives n-1 disk
 * loss by construction -- more than the array's own m, by design: the
 * metadata outlives the data it describes.
 *
 * This file holds the I/O-free core: geometry math (blob_bytes,
 * reservation) and the serialize / validate / apply of one whole blob.
 * The async persist (fan out to every online disk at the inactive slot)
 * and load (scan all 2n copies, max-generation over CRC-valid) chains
 * build on these.
 *
 * On-disk slot layout (one self-contained extent per slot, replicated
 * identically on every disk):
 *
 *   [ ec_bitmap_header ][ uint64_t span[] ][ crc32c u32 ]
 *   \________________ blob_bytes _________/trailer at offset blob_bytes
 *
 * The CRC32C covers exactly [start, start + blob_bytes) and is stored
 * in the four bytes immediately following. Any trailing slack between
 * the CRC and the next strip boundary is unused padding.
 */

#include "bdev_ec_internal.h"

#include "spdk/bdev_module.h"
#include "spdk/crc32.h"

#include <assert.h>

/*
 * ec_bitmap_blob_bytes -- length of the CRC-covered region (header +
 * span) for a bitmap blob covering num_stripes user stripes. The CRC
 * trailer lives at offset blob_bytes; total on-disk slot extent is
 * blob_bytes + sizeof(uint32_t).
 *
 * Single source of truth for the layout formula: the reservation
 * function uses it with max_num_stripes (the WIB-imposed worst case);
 * fill / validate use it with ec->num_stripes (the current geometry).
 */
uint64_t
ec_bitmap_blob_bytes(uint64_t num_stripes)
{
	return sizeof(struct ec_bitmap_header)
	       + EC_BITMAP_WORDS(num_stripes) * sizeof(uint64_t);
}

/*
 * The largest user stripe count the volume may ever hold. The ceiling is
 * imposed by the WIB, whose header + region bitmap + CRC must fit in one
 * strip. The region bitmap is written in whole uint64_t words, so the bound
 * is the number of regions that fit in the payload as whole words, not as
 * loose bits:
 *
 *   wib_payload_bytes = strip_bytes - sizeof(ec_wib_header) - 4
 *   max_regions       = (wib_payload_bytes / 8) * 64
 *   max_num_stripes   = max_regions * EC_WIB_REGION_STRIPES
 *
 * Using loose bits would admit up to 32 regions that ec_wib_fill_buf then
 * writes as one extra word, overrunning the one-strip wib_buf. The word
 * bound matches ec_wib_fill_buf and the create-time WIB-fits check exactly.
 *
 * Depends only on strip_size and blocklen, never on disk size -- which is
 * what lets the front reservations be fixed-max. num_stripes must never
 * exceed it: ec_compute_geometry and ec_bdev_resize enforce it, and the
 * assert in ec_wib_fill_buf backstops the WIB buffer.
 */
uint64_t
ec_max_num_stripes(const struct ec_bdev *ec)
{
	uint64_t strip_bytes = ec->strip_size * ec->bdev.blocklen;
	uint64_t wib_payload_bytes;
	uint64_t max_regions;

	assert(strip_bytes > sizeof(struct ec_wib_header) + sizeof(uint32_t));

	wib_payload_bytes = strip_bytes - sizeof(struct ec_wib_header) - sizeof(uint32_t);
	max_regions = (wib_payload_bytes / sizeof(uint64_t)) * 64;

	return max_regions * EC_WIB_REGION_STRIPES;
}

/*
 * ec_bitmap_reservation_stripes -- per-disk front reservation for the
 * unmapped bitmap, in physical stripes. Every disk reserves two slots
 * (copy A / copy B), each holding the worst-case blob at ec_max_num_stripes:
 *
 *   slot_strips = ceil((blob_bytes(max) + CRC) / strip_bytes)
 *   return        slot_strips * 2
 *
 * Sized for the max, so it is fixed at create and never moves on resize.
 * Every disk carries a full copy (raw replication), so the metadata
 * survives n-1 disk loss and load needs no encode/decode.
 */
uint64_t
ec_bitmap_reservation_stripes(const struct ec_bdev *ec)
{
	uint64_t strip_bytes = ec->strip_size * ec->bdev.blocklen;
	uint64_t slot_extent_bytes;
	uint64_t slot_strips;

	slot_extent_bytes = ec_bitmap_blob_bytes(ec_max_num_stripes(ec))
			    + sizeof(uint32_t);

	slot_strips = (slot_extent_bytes + strip_bytes - 1) / strip_bytes;

	return slot_strips * 2;
}

/*
 * ec_bitmap_fill_buf -- serialize source_map into buf as a complete
 * on-disk blob (header + span + CRC32C trailer).
 *
 * source_map must hold at least EC_BITMAP_WORDS(ec->num_stripes)
 * uint64_t words; buf must hold at least blob_bytes + sizeof(uint32_t)
 * bytes and must be zero-initialised on entry (the trailing slack past
 * the CRC, if any, is left untouched).
 *
 * source_map is taken as an explicit parameter -- not read directly
 * from ec->stripe_unmapped_map -- so a later caller in the UNMAP path
 * can persist a staged copy of the map without first applying the
 * staged changes to the live, reader-visible map. The create-time bootstrap
 * persist simply passes ec->stripe_unmapped_map.
 */
void
ec_bitmap_fill_buf(struct ec_bdev *ec, const uint64_t *source_map,
		   uint64_t generation, void *buf)
{
	struct ec_bitmap_header *hdr        = buf;
	uint64_t                 blob_bytes = ec_bitmap_blob_bytes(ec->num_stripes);
	uint64_t                 map_words  = EC_BITMAP_WORDS(ec->num_stripes);
	uint64_t                *span       = (uint64_t *)(hdr + 1);
	uint32_t                *crc_ptr    = (uint32_t *)((uint8_t *)buf + blob_bytes);

	hdr->magic       = EC_BITMAP_MAGIC;
	hdr->version     = EC_BITMAP_VERSION;
	hdr->generation  = generation;
	hdr->blob_bytes  = blob_bytes;
	hdr->num_stripes = ec->num_stripes;
	hdr->reserved    = 0;

	memcpy(span, source_map, map_words * sizeof(uint64_t));

	/*
	 * Whole-blob CRC, recomputed over all blob_bytes on every persist even
	 * when a single bit changed -- the cost of the raw-replicated,
	 * single-atomic-blob model. blob_bytes is bounded by the WIB-imposed
	 * stripe ceiling (tens of KiB for any volume), so this is acceptable.
	 * TODO(perf): if a write-heavy fstrim workload makes persist hot,
	 * switch to an incremental CRC over only the changed words.
	 */
	*crc_ptr = spdk_crc32c_update(buf, blob_bytes, 0);
}

/*
 * ec_bitmap_validate_buf -- check a slot read back from disk.
 *
 *   1. magic matches.
 *   2. version is the one this build understands.
 *   3. num_stripes is not larger than the current volume. A copy written
 *      before a resize carries a smaller count and is still valid -- apply
 *      zero-extends the rest, the same as a resize does in memory. A larger
 *      count means a foreign or corrupt blob (the volume never shrinks).
 *   4. blob_bytes matches the copy's own num_stripes -- the copy is
 *      internally consistent.
 *   5. CRC32C over [start, start + blob_bytes) matches the trailer at
 *      offset blob_bytes.
 *
 * Returns 0 and fills *gen_out on success, -EINVAL otherwise. A torn
 * write looks the same as any other invalid slot from the caller's
 * point of view.
 */
int
ec_bitmap_validate_buf(const struct ec_bdev *ec, const void *buf,
		       uint64_t *gen_out)
{
	const struct ec_bitmap_header *hdr        = buf;
	uint64_t                       blob_bytes;
	const uint32_t                *crc_ptr;
	uint32_t                       actual_crc, expected_crc;

	if (hdr->magic != EC_BITMAP_MAGIC) {
		return -EINVAL;
	}
	if (hdr->version != EC_BITMAP_VERSION) {
		return -EINVAL;
	}
	if (hdr->num_stripes > ec->num_stripes) {
		return -EINVAL;
	}
	if (hdr->blob_bytes != ec_bitmap_blob_bytes(hdr->num_stripes)) {
		return -EINVAL;
	}

	blob_bytes   = hdr->blob_bytes;
	crc_ptr      = (const uint32_t *)((const uint8_t *)buf + blob_bytes);
	actual_crc   = spdk_crc32c_update(buf, blob_bytes, 0);
	expected_crc = *crc_ptr;
	if (actual_crc != expected_crc) {
		return -EINVAL;
	}

	*gen_out = hdr->generation;
	return 0;
}

/* Serialize a commit record (the "stamp") into buf with a CRC32C trailer. */
void
ec_bitmap_commit_fill_buf(uint64_t committed_gen, uint32_t blob_crc, void *buf)
{
	struct ec_bitmap_commit *rec     = buf;
	uint32_t                *crc_ptr = (uint32_t *)(rec + 1);

	rec->magic         = EC_BITMAP_COMMIT_MAGIC;
	rec->version       = EC_BITMAP_COMMIT_VERSION;
	rec->committed_gen = committed_gen;
	rec->blob_crc      = blob_crc;
	rec->reserved      = 0;

	*crc_ptr = spdk_crc32c_update(buf, sizeof(*rec), 0);
}

/* Validate a commit record read from disk: magic, version, CRC32C trailer. */
int
ec_bitmap_commit_validate_buf(const void *buf, uint64_t *gen_out,
			      uint32_t *blob_crc_out)
{
	const struct ec_bitmap_commit *rec     = buf;
	const uint32_t                *crc_ptr = (const uint32_t *)(rec + 1);
	uint32_t                       actual_crc;

	if (rec->magic != EC_BITMAP_COMMIT_MAGIC) {
		return -EINVAL;
	}
	if (rec->version != EC_BITMAP_COMMIT_VERSION) {
		return -EINVAL;
	}

	actual_crc = spdk_crc32c_update(buf, sizeof(*rec), 0);
	if (actual_crc != *crc_ptr) {
		return -EINVAL;
	}

	*gen_out      = rec->committed_gen;
	*blob_crc_out = rec->blob_crc;
	return 0;
}

/*
 * ec_bitmap_apply_buf -- copy a validated blob's span into
 * ec->stripe_unmapped_map. Call only after ec_bitmap_validate_buf has
 * returned 0 for this buffer.
 *
 * Threading: this runs on the home thread (the creation thread that
 * issued the async load chain). The bdev IS already registered at
 * this point -- spdk_bdev_register fires before the load chain runs
 * apply_buf -- so a strict "no reader exists" argument does not hold.
 *
 * The window from register to apply_buf is non-empty, but the only
 * reader that can enter it is SPDK examine (lvol superblock import
 * etc.), which runs on the same spdk_thread that registered the bdev
 * -- i.e. home. Examine and apply_buf therefore serialize on home,
 * and the memcpy is observably atomic to examine reads.
 *
 * Workload (cross-reactor) readers cannot reach the bdev until the
 * create-RPC returns, which waits for both apply_buf and
 * spdk_bdev_wait_for_examine to finish (see the create-finalize
 * comment in bdev_ec.c). So by the time any non-home reader runs an
 * acquire-load on stripe_unmapped_map, the memcpy is complete and
 * its effect is published by the post-create release barriers SPDK
 * inserts when handing the bdev to its consumers.
 *
 * The same logic lets resize.c:165 use memcpy under quiesce.
 */
void
ec_bitmap_apply_buf(struct ec_bdev *ec, const void *buf)
{
	const struct ec_bitmap_header *hdr       = buf;
	const uint64_t                *span      = (const uint64_t *)(hdr + 1);
	uint64_t                       hdr_words = EC_BITMAP_WORDS(hdr->num_stripes);
	uint64_t                       map_words = EC_BITMAP_WORDS(ec->num_stripes);

	/*
	 * Copy the blob's span; zero any stripes added since it was written
	 * (a pre-resize blob has fewer). validate_buf ensured hdr_words <=
	 * map_words.
	 */
	memcpy(ec->stripe_unmapped_map, span, hdr_words * sizeof(uint64_t));
	if (map_words > hdr_words) {
		memset(ec->stripe_unmapped_map + hdr_words, 0,
		       (map_words - hdr_words) * sizeof(uint64_t));
	}
}

/* =========================================================================
 * Async I/O: persist and load chains
 *
 * The bitmap region at the front of every disk is laid out as two slots
 * (copy A / copy B), each occupying the same number of strips. A persist
 * writes the inactive slot on every online writable disk; a load scans
 * all 2n {disk, slot} copies and picks the max-generation CRC-valid
 * winner.
 *
 * Slot LBA on disk d, copy c is c * slot_reserved_blocks. The same value
 * on every disk -- raw replication, no per-disk geometry.
 * ========================================================================= */

/*
 * One slot's reserved size, in blocks: half the two-slot reservation.
 * Sized from the max stripe count, so it never changes on resize. This is
 * what places the slots -- copy B stays at the same LBA after a grow.
 */
static uint64_t
ec_bitmap_slot_reserved_blocks(const struct ec_bdev *ec)
{
	return (ec_bitmap_reservation_stripes(ec) / 2) * ec->strip_size;
}

static uint64_t
ec_bitmap_slot_lba_blocks(const struct ec_bdev *ec, uint8_t copy)
{
	return (uint64_t)copy * ec_bitmap_slot_reserved_blocks(ec);
}

/*
 * Bytes read or written for one slot at the current size: header + span +
 * CRC, rounded up to a whole strip. Sizes the DMA buffer and the I/O
 * length, and never exceeds one reserved slot.
 */
static uint64_t
ec_bitmap_slot_io_blocks(const struct ec_bdev *ec)
{
	uint64_t strip_bytes  = ec->strip_size * ec->bdev.blocklen;
	uint64_t slot_extent  = ec_bitmap_blob_bytes(ec->num_stripes) +
				sizeof(uint32_t);
	uint64_t slot_strips  = (slot_extent + strip_bytes - 1) / strip_bytes;
	uint64_t io_blocks    = slot_strips * ec->strip_size;

	/*
	 * Backstop the placement invariant: the I/O extent must fit inside one
	 * reserved slot, or a persist would write past it into the WIB strips
	 * and user data. ec_compute_geometry / ec_bdev_resize keep num_stripes
	 * <= ec_max_num_stripes, which guarantees this.
	 */
	assert(io_blocks <= ec_bitmap_slot_reserved_blocks(ec));

	return io_blocks;
}

/* -------------------------------------------------------------------------
 * Persist
 * ------------------------------------------------------------------------- */

/*
 * Context for one in-flight persist. The DMA buffer holds the
 * serialised blob (header + span + CRC + trailing slack to the strip
 * boundary) and is shared by every disk's write -- raw replication
 * writes identical bytes to every disk.
 *
 * Lifecycle: alloc on submit, freed in the write-completion callback
 * once the last in-flight write completes. Two callbacks are wired:
 *
 *   cb_durable fires at the m+1-ack moment (or, on failure, at full
 *   drainout) -- the caller's "durability achieved" hook. UNMAP uses
 *   this to apply staged->live and release its caller before slow
 *   disks finish.
 *
 *   cb_drained fires at full drainout, after bitmap_persist_in_flight
 *   has been cleared -- the caller's "this persist is fully settled"
 *   hook. Bootstrap uses this to chain the second-slot persist
 *   without racing the first one's stragglers.
 *
 * Either callback may be NULL. bitmap_persist_in_flight stays true
 * until drainout regardless of when cb_durable fires, so a second
 * persist arriving in the post-ack-pre-drainout window is blocked
 * (with -EBUSY); this is the correctness invariant that prevents
 * straggler writes from a previous persist overwriting fresh writes
 * from a subsequent persist on the same slot LBA.
 */
struct ec_bitmap_persist_ctx {
	struct ec_bdev          *ec;
	void                    *dma_buf;
	uint8_t                  next_copy;
	uint32_t                 writes_in_flight;
	uint32_t                 successes;
	uint32_t                 required;
	bool                     acked;
	int                      first_err;
	ec_bitmap_persist_cb_fn  cb_durable;
	void                    *cb_durable_arg;
	ec_bitmap_persist_cb_fn  cb_drained;
	void                    *cb_drained_arg;
};

static void
ec_bitmap_persist_write_cb(struct spdk_bdev_io *bdev_io, bool success,
			   void *cb_arg)
{
	struct ec_bitmap_persist_ctx *ctx = cb_arg;
	struct ec_bdev               *ec  = ctx->ec;
	int                           final_rc;

	spdk_bdev_free_io(bdev_io);

	if (success) {
		ctx->successes++;
	} else if (ctx->first_err == 0) {
		ctx->first_err = -EIO;
		SPDK_WARNLOG("EC bdev %s: bitmap persist write failed\n",
			     ec->bdev.name);
	}

	/*
	 * Pre-completion durability ack: fire cb_durable as soon as the
	 * threshold is met so the caller can release its dependents (UNMAP
	 * applies staged->live and acks its bdev_io). bitmap_active_copy
	 * is flipped here, but bitmap_persist_in_flight is NOT cleared --
	 * it stays true until full drainout so a subsequent persist
	 * cannot start while this persist's slow writes are still in
	 * flight to the same slot LBA. Clearing pending too early lets
	 * stragglers from this persist overwrite the next persist's
	 * fresh writes after that next persist had already been acked,
	 * silently regressing durability past an acknowledged
	 * commit.
	 */
	if (!ctx->acked && ctx->successes >= ctx->required) {
		ctx->acked             = true;
		ec->bitmap_active_copy = ctx->next_copy;
		if (ctx->cb_durable) {
			ctx->cb_durable(ctx->cb_durable_arg, 0);
		}
	}

	ctx->writes_in_flight--;
	if (ctx->writes_in_flight != 0) {
		return;
	}

	/*
	 * Last write done. Now safe to clear bitmap_persist_in_flight so the
	 * next persist may begin -- all on-disk state from this persist is
	 * either committed (success) or terminally failed.
	 */
	ec->bitmap_persist_in_flight = false;

	/*
	 * Release any deferred slot channels before the follow-up persist kick
	 * below -- otherwise a steady stream of bit-clears would keep restarting
	 * the persist and starve the cleanup. Never frees ec.
	 */
	ec_drain_deferred_slot_releases(ec);

	/*
	 * If write-into-unmapped completions queued bit-clears while this
	 * persist was in flight (whether THIS persist was their kick or it
	 * was somebody else's, e.g., UNMAP), fire a single follow-up
	 * persist covering all of them. The kick must come BEFORE any
	 * cb_drained that might itself enqueue new bit-clears recursively;
	 * if it does, those land on pending_bit_clears and the next persist
	 * drainout picks them up -- the recursion is naturally bounded.
	 */
	ec_bit_clear_flush_if_pending(ec);

	if (!ctx->acked) {
		/*
		 * Threshold was never reached. Report the failure via
		 * cb_durable now (the caller's "did the persist succeed?"
		 * hook), leaving the active_copy / generation as they were
		 * so the bitmap stays on its prior on-disk content.
		 */
		SPDK_ERRLOG("EC bdev %s: bitmap persist did not reach durability "
			    "threshold (succeeded=%u, required=%u)\n",
			    ec->bdev.name, ctx->successes, ctx->required);
		final_rc = ctx->first_err ? ctx->first_err : -EIO;
		if (ctx->cb_durable) {
			ctx->cb_durable(ctx->cb_durable_arg, final_rc);
		}
	} else {
		final_rc = 0;
	}

	if (ctx->cb_drained) {
		ctx->cb_drained(ctx->cb_drained_arg, final_rc);
	}

	spdk_dma_free(ctx->dma_buf);
	free(ctx);

	/*
	 * Last statement: finish a deferred delete. In that case this frees ec,
	 * so nothing may touch ec after this call.
	 */
	ec_drain_deferred_unregister(ec);
}

/*
 * Why bitmap persists triggered by I/O paths (UNMAP, write-into-unmapped)
 * route to the home thread instead of running on each ec_io_channel's
 * own bitmap_chans[]:
 *
 * The same bitmap blob is written to every disk on every persist. The
 * consistency model needs a single writer to keep three coordination
 * signals well-defined:
 *
 *   - bitmap_active_copy. Each persist writes the slot the active copy
 *     is NOT in (next_copy = 1 - active_copy). Two writers both reading
 *     active_copy = 0 would both write slot 1; their bytes interleave
 *     and the loaded blob fails CRC.
 *   - bitmap_generation. Monotonic counter, incremented by exactly one
 *     writer per persist. Two writers both reading N and both writing
 *     N+1 with different content break the "highest valid generation
 *     wins" rule that ec_bitmap_load_async relies on to pick the committed copy.
 *   - cb_drained. Fires when every write for THIS persist has acked,
 *     which gates the next bit-clear flush. Concurrent persists make
 *     "all acked" ambiguous (which persist?); the flush ordering
 *     breaks.
 *
 * Distributing writers across per-channel bitmap_chans[] would mean a
 * new coordination protocol: atomic generation counters, cross-channel
 * drainout, a cross-thread lock on bitmap_persist_in_flight. That is a
 * different on-disk consistency story -- a new storage protocol, not
 * a refactor of this one. Routing each persist trigger to the home
 * thread via spdk_thread_send_msg keeps the existing single-writer
 * model. One cross-thread message per persist costs microseconds and
 * preserves every invariant verbatim.
 */

int
ec_bitmap_persist_async(struct ec_bdev *ec, const uint64_t *source_map,
			ec_bitmap_persist_cb_fn cb_durable, void *cb_durable_arg,
			ec_bitmap_persist_cb_fn cb_drained, void *cb_drained_arg)
{
	struct ec_bitmap_persist_ctx *ctx;
	uint64_t slot_lba_blocks;
	uint64_t slot_size_blocks;
	uint64_t slot_size_bytes;

	/*
	 * Home-thread only: this function uses bitmap_chans[] (thread-affine
	 * to the creation thread) and mutates bitmap_persist_in_flight,
	 * bitmap_active_copy, and bitmap_generation. I/O-path callers (UNMAP,
	 * write-into-unmapped's bit-clear) route here via the helpers above.
	 */
	assert(spdk_get_thread() == ec->home_thread);
	uint32_t n_writable = 0;
	uint32_t i;
	int      rc;

	if (ec->bitmap_persist_in_flight) {
		return -EBUSY;
	}

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		return -ENOMEM;
	}

	slot_size_blocks = ec_bitmap_slot_io_blocks(ec);
	slot_size_bytes  = slot_size_blocks * ec->bdev.blocklen;

	ctx->dma_buf = spdk_dma_zmalloc(slot_size_bytes, EC_DMA_ALIGN, NULL);
	if (!ctx->dma_buf) {
		free(ctx);
		return -ENOMEM;
	}

	/* Count writable, attached disks with channels available. */
	for (i = 0; i < ec->n; i++) {
		if (ec->descs[i] && ec->bitmap_chans[i] &&
		    ec_slot_is_writable(ec, i)) {
			n_writable++;
		}
	}
	if (n_writable == 0) {
		spdk_dma_free(ctx->dma_buf);
		free(ctx);
		return -EIO;
	}

	ec->bitmap_generation++;

	ctx->ec              = ec;
	ctx->next_copy       = 1 - ec->bitmap_active_copy;
	ctx->required        = spdk_min(n_writable, ec->m + 1);
	ctx->cb_durable      = cb_durable;
	ctx->cb_durable_arg  = cb_durable_arg;
	ctx->cb_drained      = cb_drained;
	ctx->cb_drained_arg  = cb_drained_arg;

	/*
	 * Serialise once into the shared DMA buffer; raw replication means
	 * every disk receives the same bytes.
	 */
	ec_bitmap_fill_buf(ec, source_map, ec->bitmap_generation, ctx->dma_buf);

	slot_lba_blocks = ec_bitmap_slot_lba_blocks(ec, ctx->next_copy);

	ec->bitmap_persist_in_flight = true;

	for (i = 0; i < ec->n; i++) {
		if (!ec->descs[i] || !ec->bitmap_chans[i] ||
		    !ec_slot_is_writable(ec, i)) {
			continue;
		}

		ctx->writes_in_flight++;
		rc = spdk_bdev_write(ec->descs[i],
				     ec->bitmap_chans[i],
				     ctx->dma_buf,
				     slot_lba_blocks * ec->bdev.blocklen,
				     slot_size_bytes,
				     ec_bitmap_persist_write_cb,
				     ctx);
		if (rc != 0) {
			SPDK_WARNLOG("EC bdev %s: bitmap persist submit failed "
				     "for slot %u (rc=%d)\n",
				     ec->bdev.name, i, rc);
			ctx->writes_in_flight--;
			if (ctx->first_err == 0) {
				ctx->first_err = rc;
			}
		}
	}

	if (ctx->writes_in_flight == 0) {
		/*
		 * No write got submitted. Roll the persist back synchronously
		 * -- the caller treats this exactly like any other -errno
		 * return, and cb is not invoked.
		 */
		ec->bitmap_persist_in_flight = false;
		rc = ctx->first_err ? ctx->first_err : -EIO;
		spdk_dma_free(ctx->dma_buf);
		free(ctx);
		return rc;
	}

	return 0;
}

/*
 * Two chained persists overwrite both bitmap copies (and both commit copies)
 * on every writable disk with the current map. cb_drained (not cb_durable)
 * gates the second persist: starting it before the first fully drains would
 * let the first's straggler writes race the second on the slot it overwrites.
 *
 * Persist 1: gen N   -> slot 1 - active.
 * Persist 2: gen N+1 -> the other slot (active flipped after persist 1's ack).
 */
struct ec_bitmap_persist_both_ctx {
	struct ec_bdev          *ec;
	ec_bitmap_persist_cb_fn  done_fn;
	void                    *done_arg;
};

/* Report the final outcome and free the ctx. Signature matches
 * ec_bitmap_persist_cb_fn so it also serves as the second persist's cb_drained. */
static void
ec_bitmap_persist_both_report(void *arg, int rc)
{
	struct ec_bitmap_persist_both_ctx *sctx     = arg;
	ec_bitmap_persist_cb_fn     done_fn  = sctx->done_fn;
	void                       *done_arg = sctx->done_arg;

	free(sctx);
	if (done_fn) {
		done_fn(done_arg, rc);
	}
}

/* First persist drained; chain the second onto the other slot. */
static void
ec_bitmap_persist_both_first_done(void *arg, int rc)
{
	struct ec_bitmap_persist_both_ctx *sctx = arg;
	int                         persist_rc;

	if (rc != 0) {
		ec_bitmap_persist_both_report(sctx, rc);
		return;
	}

	persist_rc = ec_bitmap_persist_async(sctx->ec, sctx->ec->stripe_unmapped_map,
					     NULL, NULL,
					     ec_bitmap_persist_both_report, sctx);
	if (persist_rc != 0) {
		ec_bitmap_persist_both_report(sctx, persist_rc);
	}
}

int
ec_bitmap_persist_both_copies(struct ec_bdev *ec,
			   ec_bitmap_persist_cb_fn done_fn, void *done_arg)
{
	struct ec_bitmap_persist_both_ctx *sctx;
	int                         rc;

	assert(spdk_get_thread() == ec->home_thread);

	sctx = calloc(1, sizeof(*sctx));
	if (!sctx) {
		return -ENOMEM;
	}
	sctx->ec       = ec;
	sctx->done_fn  = done_fn;
	sctx->done_arg = done_arg;

	rc = ec_bitmap_persist_async(ec, ec->stripe_unmapped_map, NULL, NULL,
				     ec_bitmap_persist_both_first_done, sctx);
	if (rc != 0) {
		free(sctx);
		return rc;
	}
	return 0;
}

/* -------------------------------------------------------------------------
 * Load
 *
 * Serial scan: for each {disk, slot} in turn, read into read_buf and
 * validate. If the new copy beats the running best (higher generation,
 * or no best yet), swap pointers so best_buf holds the winner. At the
 * end, apply best_buf if any winner exists.
 *
 * Serial keeps memory bounded at 2 * slot_size_bytes regardless of disk
 * count or volume size; load is one-shot at startup and the latency cost
 * (2n disk reads serially, sub-millisecond each on NVMe) is acceptable.
 * ------------------------------------------------------------------------- */

struct ec_bitmap_load_ctx {
	struct ec_bdev      *ec;

	void                *read_buf;       /* current read target            */
	void                *best_buf;       /* current best-generation winner */
	uint64_t             slot_size_bytes;

	uint32_t             cur_disk;       /* base-bdev slot 0..n-1          */
	uint8_t              cur_copy;       /* 0 or 1                          */

	bool                 has_best;
	uint64_t             best_gen;
	uint8_t              best_copy;

	ec_bdev_create_cb_fn done_fn;
	void                *done_arg;
};

static void ec_bitmap_load_continue(struct ec_bitmap_load_ctx *ctx);

static void
ec_bitmap_load_finish(struct ec_bitmap_load_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;
	ec_bdev_create_cb_fn done_fn = ctx->done_fn;
	void *done_arg = ctx->done_arg;

	if (ctx->has_best) {
		ec_bitmap_apply_buf(ec, ctx->best_buf);
		ec->bitmap_generation  = ctx->best_gen;
		ec->bitmap_active_copy = ctx->best_copy;
		SPDK_NOTICELOG("EC bdev %s: bitmap loaded (gen %" PRIu64 ", slot %u)\n",
			       ec->bdev.name, ctx->best_gen, ctx->best_copy);
	} else {
		SPDK_NOTICELOG("EC bdev %s: no valid bitmap copy found -- "
			       "stripe_unmapped_map left zero\n",
			       ec->bdev.name);
	}

	spdk_dma_free(ctx->read_buf);
	spdk_dma_free(ctx->best_buf);
	free(ctx);

	done_fn(done_arg, 0);
}

/* Advance the load cursor: copy 0 -> copy 1 on the same disk, then next disk. */
static void
ec_bitmap_load_next_copy(struct ec_bitmap_load_ctx *ctx)
{
	if (ctx->cur_copy == 0) {
		ctx->cur_copy = 1;
	} else {
		ctx->cur_copy = 0;
		ctx->cur_disk++;
	}
}

static void
ec_bitmap_load_read_cb(struct spdk_bdev_io *bdev_io, bool success, void *cb_arg)
{
	struct ec_bitmap_load_ctx *ctx = cb_arg;
	struct ec_bdev            *ec  = ctx->ec;
	uint64_t                   generation;

	spdk_bdev_free_io(bdev_io);

	if (success && ec_bitmap_validate_buf(ec, ctx->read_buf, &generation) == 0) {
		if (!ctx->has_best || generation > ctx->best_gen) {
			void *tmp     = ctx->best_buf;
			ctx->best_buf = ctx->read_buf;
			ctx->read_buf = tmp;
			ctx->best_gen  = generation;
			ctx->best_copy = ctx->cur_copy;
			ctx->has_best  = true;
		}
	}

	ec_bitmap_load_next_copy(ctx);

	ec_bitmap_load_continue(ctx);
}

static void
ec_bitmap_load_continue(struct ec_bitmap_load_ctx *ctx)
{
	struct ec_bdev *ec = ctx->ec;
	int             rc;

	while (ctx->cur_disk < ec->n) {
		uint32_t disk = ctx->cur_disk;

		if (!ec->descs[disk] || !ec->bitmap_chans[disk] ||
		    !ec_slot_is_readable(ec, disk)) {
			ctx->cur_disk++;
			ctx->cur_copy = 0;
			continue;
		}

		rc = spdk_bdev_read(ec->descs[disk],
				    ec->bitmap_chans[disk],
				    ctx->read_buf,
				    ec_bitmap_slot_lba_blocks(ec, ctx->cur_copy)
				    * ec->bdev.blocklen,
				    ctx->slot_size_bytes,
				    ec_bitmap_load_read_cb,
				    ctx);
		if (rc != 0) {
			SPDK_WARNLOG("EC bdev %s: bitmap load read submit failed "
				     "for slot %u copy %u (rc=%d) -- skipping\n",
				     ec->bdev.name, disk, ctx->cur_copy, rc);
			/* Skip this copy, try the next. */
			ec_bitmap_load_next_copy(ctx);
			continue;
		}

		return;  /* I/O submitted; callback drives the next step. */
	}

	ec_bitmap_load_finish(ctx);
}

void
ec_bitmap_load_async(struct ec_bdev *ec,
		     ec_bdev_create_cb_fn done_fn, void *done_arg)
{
	struct ec_bitmap_load_ctx *ctx;
	uint64_t                   slot_size_bytes;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		SPDK_ERRLOG("EC bdev %s: OOM for bitmap load ctx\n",
			    ec->bdev.name);
		done_fn(done_arg, -ENOMEM);
		return;
	}

	slot_size_bytes = ec_bitmap_slot_io_blocks(ec) * ec->bdev.blocklen;

	ctx->read_buf = spdk_dma_zmalloc(slot_size_bytes, EC_DMA_ALIGN, NULL);
	ctx->best_buf = spdk_dma_zmalloc(slot_size_bytes, EC_DMA_ALIGN, NULL);
	if (!ctx->read_buf || !ctx->best_buf) {
		SPDK_ERRLOG("EC bdev %s: OOM for bitmap load buffers "
			    "(slot_size=%" PRIu64 " bytes)\n",
			    ec->bdev.name, slot_size_bytes);
		spdk_dma_free(ctx->read_buf);
		spdk_dma_free(ctx->best_buf);
		free(ctx);
		done_fn(done_arg, -ENOMEM);
		return;
	}

	ctx->ec              = ec;
	ctx->slot_size_bytes = slot_size_bytes;
	ctx->done_fn         = done_fn;
	ctx->done_arg        = done_arg;
	/* cur_disk = cur_copy = 0; has_best = false; (calloc'd) */

	ec_bitmap_load_continue(ctx);
}

/* -------------------------------------------------------------------------
 * Status query
 * ------------------------------------------------------------------------- */

/*
 * Count set bits in stripe_unmapped_map. Returns 0 if the map is not
 * yet allocated. Linear popcount; tens of K words for a multi-TiB
 * volume, so a microsecond-scale scan.
 */
uint64_t
ec_count_unmapped_stripes(const struct ec_bdev *ec)
{
	uint64_t map_words;
	uint64_t total = 0;
	uint64_t i;

	if (!ec->stripe_unmapped_map) {
		return 0;
	}

	map_words = EC_BITMAP_WORDS(ec->num_stripes);
	for (i = 0; i < map_words; i++) {
		total += __builtin_popcountll(ec->stripe_unmapped_map[i]);
	}
	return total;
}

int
ec_bdev_get_unmap_status(const char *ec_name,
			 uint64_t   *num_stripes,
			 uint64_t   *unmapped_stripes,
			 uint64_t   *blob_bytes,
			 uint64_t   *generation,
			 uint8_t    *active_copy,
			 bool       *persist_pending)
{
	struct ec_bdev *ec = ec_bdev_find(ec_name);

	if (!ec) {
		return -ENODEV;
	}

	*num_stripes      = ec->num_stripes;
	*unmapped_stripes = ec_count_unmapped_stripes(ec);
	*blob_bytes       = ec_bitmap_blob_bytes(ec->num_stripes);
	*generation       = ec->bitmap_generation;
	*active_copy      = ec->bitmap_active_copy;
	*persist_pending  = ec->bitmap_persist_in_flight;

	return 0;
}

/* =========================================================================
 * Bit-clear waiter queue
 *
 * Used by the write-into-unmapped path to clear stripe_unmapped_map
 * bits after data has landed. The two-list design (pending vs in_flight)
 * lets new clears queue while a previous persist is still draining, and
 * coalesces a burst into one follow-up persist per roundtrip.
 *
 * Visibility ordering mirrors UNMAP's staged_map pattern:
 *   - clear_staged_map = copy of live with target bits cleared
 *   - on cb_durable (m+1 ack): apply cleared bits to live, invoke waiters
 *   - on cb_drained (persist fully settled): kick next persist if any
 *
 * The in-memory live map is updated ONLY after the persist acks. This is
 * what makes the post-crash recovery story work: if we crashed between
 * staging and ack, the on-disk bitmap still reads as "unmapped" for those
 * stripes, reads synthesise zeros, and the unacked writes are correctly
 * lost. Updating live before ack would let readers see "mapped" before
 * the on-disk bitmap recorded it -- a crash window where reads return
 * real data that may not survive recovery.
 * ========================================================================= */

static void ec_bit_clear_flush(struct ec_bdev *ec);

static void
ec_bit_clear_on_durable(void *arg, int rc)
{
	struct ec_bdev              *ec = arg;
	struct ec_pending_bit_clear *w, *tmp;

	/*
	 * Drain the in_flight list. On success, apply the bit-clear to live
	 * (the persist's bytes already reflect the cleared state). On
	 * failure, leave live untouched -- the on-disk bitmap is also
	 * unchanged, so stripes remain unmapped and reads continue to
	 * synthesise zeros. The waiter's cb_fn propagates the failure up.
	 */
	TAILQ_FOREACH_SAFE(w, &ec->in_flight_bit_clears, link, tmp) {
		TAILQ_REMOVE(&ec->in_flight_bit_clears, w, link);
		if (rc == 0) {
			ec_stripe_clear_unmapped(ec, w->stripe_index);
		}
		if (w->cb_fn) {
			w->cb_fn(w->cb_arg, rc);
		}
		free(w);
	}
}

static void
ec_bit_clear_on_drained(void *arg, int rc)
{
	struct ec_bdev *ec = arg;

	/*
	 * Persist committed (rc == 0) or terminally failed (on_durable already
	 * propagated the failure to waiters). Free the shadow and kick a fresh
	 * persist if new waiters arrived during drainout. Freeing here -- not
	 * earlier -- is what makes ec_bit_clear_flush_if_pending's NULL-shadow
	 * precondition hold for the post-drain kick.
	 */
	free(ec->clear_staged_map);
	ec->clear_staged_map = NULL;

	if (!ec->bitmap_persist_in_flight && !TAILQ_EMPTY(&ec->pending_bit_clears)) {
		ec_bit_clear_flush(ec);
	}

	(void)rc;
}

/*
 * Move every waiter from pending_bit_clears to in_flight_bit_clears,
 * build clear_staged_map by copying live and clearing the in-flight
 * bits, and submit a single persist for the whole batch.
 *
 * Precondition: bitmap_persist_in_flight == false, pending list non-empty.
 *
 * On submit failure, the in_flight list is drained with the error and
 * the staged map is freed; clear_staged_map is reset to NULL so the
 * next kick can start clean.
 */
static void
ec_bit_clear_flush(struct ec_bdev *ec)
{
	struct ec_pending_bit_clear *w;
	uint64_t                     map_words;
	int                          rc;

	/*
	 * Home-thread only: touches pending_bit_clears, clear_staged_map, and
	 * submits a persist via ec_bitmap_persist_async. The legitimate callers
	 * are the routed enqueue (ec_bit_clear_enqueue_on_home) and the
	 * waiter-queue follow-up hook (ec_bit_clear_flush_if_pending).
	 */
	assert(spdk_get_thread() == ec->home_thread);

	assert(!ec->bitmap_persist_in_flight);
	assert(!TAILQ_EMPTY(&ec->pending_bit_clears));
	assert(TAILQ_EMPTY(&ec->in_flight_bit_clears));
	assert(ec->clear_staged_map == NULL);

	map_words = EC_BITMAP_WORDS(ec->num_stripes);
	ec->clear_staged_map = calloc(map_words, sizeof(uint64_t));
	if (!ec->clear_staged_map) {
		/*
		 * Allocation failure -- drain waiters with -ENOMEM so callers
		 * release stripe-busy and the bdev_io fails loudly rather than
		 * stalling forever. Next kick will retry from scratch.
		 */
		while ((w = TAILQ_FIRST(&ec->pending_bit_clears)) != NULL) {
			TAILQ_REMOVE(&ec->pending_bit_clears, w, link);
			if (w->cb_fn) {
				w->cb_fn(w->cb_arg, -ENOMEM);
			}
			free(w);
		}
		return;
	}

	memcpy(ec->clear_staged_map, ec->stripe_unmapped_map,
	       map_words * sizeof(uint64_t));

	/* Move pending -> in_flight, applying each clear to the shadow. */
	while ((w = TAILQ_FIRST(&ec->pending_bit_clears)) != NULL) {
		TAILQ_REMOVE(&ec->pending_bit_clears, w, link);
		ec_bitmap_word_clear(ec->clear_staged_map, w->stripe_index);
		TAILQ_INSERT_TAIL(&ec->in_flight_bit_clears, w, link);
	}

	rc = ec_bitmap_persist_async(ec, ec->clear_staged_map,
				     ec_bit_clear_on_durable, ec,
				     ec_bit_clear_on_drained, ec);
	if (rc != 0) {
		SPDK_WARNLOG("EC bdev %s: bit-clear persist submit failed "
			     "(rc=%d); draining waiters\n",
			     ec->bdev.name, rc);
		ec_bit_clear_on_durable(ec, rc);
		/* Sync submit failure: cb_drained will not fire, so free the shadow map here. */
		free(ec->clear_staged_map);
		ec->clear_staged_map = NULL;
	}
}

void
ec_bit_clear_flush_if_pending(struct ec_bdev *ec)
{
	/*
	 * Home-thread only: fires from the bit-clear persist's write_cb chain,
	 * which runs on the channel-owning thread. Same invariant as
	 * ec_bit_clear_flush.
	 */
	assert(spdk_get_thread() == ec->home_thread);

	if (ec->bitmap_persist_in_flight) {
		/*
		 * Another persist already in flight (race-free because both
		 * setter and reader run on the home thread -- this hook is
		 * called only from a persist's own write_cb where pending
		 * was just set to false, but a nested invocation pattern
		 * could in theory re-set it). Bail; the next persist's
		 * drainout will retry.
		 */
		return;
	}
	if (ec->clear_staged_map != NULL) {
		/*
		 * A bit-clear persist's cb_drained hasn't fired yet -- our
		 * own staged shadow is still set. This hook ran from inside
		 * ec_bitmap_persist_write_cb between "pending = false" and
		 * cb_drained, which is the natural ordering. cb_drained will
		 * free the shadow and then kick any new waiters that arrived
		 * during this persist. Bailing here avoids tripping the
		 * "clear_staged_map == NULL" precondition in ec_bit_clear_flush.
		 */
		return;
	}
	if (TAILQ_EMPTY(&ec->pending_bit_clears)) {
		return;
	}
	ec_bit_clear_flush(ec);
}

/*
 * Home-thread half of ec_submit_bit_clear_async. ec_submit_bit_clear_async
 * runs on the submitter thread, allocates the waiter (so -ENOMEM stays
 * synchronous), and then calls this -- inline if it already is the home
 * thread, otherwise via spdk_thread_send_msg. The queue insert and the
 * persist trigger both mutate home-thread state.
 */
static void
ec_bit_clear_enqueue_on_home(void *arg)
{
	struct ec_pending_bit_clear *w  = arg;
	struct ec_bdev              *ec = w->ec;

	assert(spdk_get_thread() == ec->home_thread);

	TAILQ_INSERT_TAIL(&ec->pending_bit_clears, w, link);
	if (!ec->bitmap_persist_in_flight) {
		ec_bit_clear_flush(ec);
	}
}

int
ec_submit_bit_clear_async(struct ec_bdev *ec, uint64_t stripe_index,
			  void (*cb_fn)(void *cb_arg, int rc),
			  void *cb_arg)
{
	struct ec_pending_bit_clear *w;

	if (stripe_index >= ec->num_stripes) {
		return -EINVAL;
	}

	w = calloc(1, sizeof(*w));
	if (!w) {
		return -ENOMEM;
	}
	w->ec           = ec;
	w->stripe_index = stripe_index;
	w->cb_fn        = cb_fn;
	w->cb_arg       = cb_arg;

	/*
	 * Route the enqueue + flush trigger to the home thread.
	 * write-into-unmapped, which drives this, completes on the bdev_io
	 * owner thread (nvmf poll group, raid child, ...) -- which differs
	 * from the EC creation thread under a multi-reactor SPDK target.
	 * pending_bit_clears, bitmap_persist_in_flight, and bitmap_chans[] are
	 * all home-thread state.
	 *
	 * Fast path: when the caller is already on the home thread, skip
	 * send_msg and run inline -- no added latency.
	 *
	 * See "why route, not per-channel" above ec_bitmap_persist_async.
	 */
	if (spdk_get_thread() == ec->home_thread) {
		ec_bit_clear_enqueue_on_home(w);
	} else {
		int rc = spdk_thread_send_msg(ec->home_thread,
					      ec_bit_clear_enqueue_on_home, w);
		if (rc != 0) {
			free(w);
			return rc;
		}
	}
	/*
	 * Return 0 means "enqueued; cb_fn will fire later" -- NOT that the bit
	 * has been cleared. If a persist is in flight, the waiter sits on
	 * pending until that persist's drainout fires
	 * ec_bit_clear_flush_if_pending.
	 */
	return 0;
}
