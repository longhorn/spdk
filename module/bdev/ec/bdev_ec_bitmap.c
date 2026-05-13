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
 * ec_bitmap_reservation_stripes -- size the in-band unmapped bitmap
 * region, in physical stripes per disk, from strip_size + blocklen alone.
 *
 * Layout under the raw-replicated blob model: every disk reserves two
 * slots (copy A / copy B), each large enough to hold the worst-case
 * bitmap blob. The returned value is the per-disk total in physical
 * stripes -- both slots, on every disk uniformly.
 *
 *   blob_bytes  = sizeof(ec_bitmap_header)
 *               + EC_BITMAP_WORDS(max_num_stripes) * sizeof(uint64_t)
 *               + sizeof(uint32_t)        // CRC32C
 *   slot_strips = ceil(blob_bytes / strip_bytes)
 *   return        slot_strips * 2
 *
 * The bitmap needs one bit per user stripe, sized for the maximum stripe
 * count the volume could ever reach. That ceiling is imposed by the WIB,
 * which must fit its header + region bitmap + CRC inside a single strip:
 *
 *   wib_payload_bytes = strip_bytes - sizeof(ec_wib_header) - 4
 *   max_num_stripes   = wib_payload_bytes * 8 * EC_WIB_REGION_STRIPES
 *
 * max_num_stripes depends only on strip_size and blocklen -- never on the
 * disk size -- which is what makes the reservation "fixed-max": it does
 * not grow on resize and the reserved region never moves.
 *
 * Per-disk cost is `2 * blob_bytes` (k * larger than an EC-encoded layout
 * would be, by design): every disk carries a full copy, so the metadata
 * survives n-1 disk loss and load needs no encode/decode.
 */
uint64_t
ec_bitmap_reservation_stripes(const struct ec_bdev *ec)
{
	uint64_t strip_bytes = ec->strip_size * ec->bdev.blocklen;
	uint64_t wib_payload_bytes;
	uint64_t max_num_stripes;
	uint64_t slot_extent_bytes;
	uint64_t slot_strips;

	wib_payload_bytes = strip_bytes - sizeof(struct ec_wib_header)
			    - sizeof(uint32_t);
	max_num_stripes   = wib_payload_bytes * 8 * EC_WIB_REGION_STRIPES;

	/* Full on-disk extent of one slot: CRC-covered region + CRC trailer. */
	slot_extent_bytes = ec_bitmap_blob_bytes(max_num_stripes)
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
		   uint32_t generation, void *buf)
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
		       uint32_t *gen_out)
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

/*
 * ec_bitmap_apply_buf -- copy a validated blob's span into
 * ec->stripe_unmapped_map. Call only after ec_bitmap_validate_buf has
 * returned 0 for this buffer.
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

	return slot_strips * ec->strip_size;
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
