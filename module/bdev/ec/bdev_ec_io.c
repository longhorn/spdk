/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_io.c -- read and full-stripe write paths.
 *
 * Three submission entry points live here:
 *
 *   ec_submit_read         direct or degraded read
 *   ec_submit_full_write   stripe-aligned write (encode + scatter)
 *   ec_submit_write        dispatcher: full-stripe vs RMW
 *
 * Sub-stripe writes go to ec_submit_rmw_write in bdev_ec_rmw.c.
 *
 * The ISA-L reconstruction wrappers (ec_reconstruct_data_chunk,
 * ec_reconstruct_multi_data) live here too, since they were extracted
 * from the degraded read path and rebuild reuses them through the
 * internal API.
 */

#include "bdev_ec_internal.h"

#include "spdk/bdev_module.h"
#include "spdk/string.h"
#include "spdk/util.h"

#include <isa-l/erasure_code.h>

/* =========================================================================
 * Degraded read context
 * ========================================================================= */

struct ec_degraded_read_ctx {
	struct ec_bdev_io  *ec_io;
	uint32_t            target_chunk;
	uint64_t            chunk_blocks;
	uint64_t            chunk_offset;
	uint8_t            *chunk_bufs[EC_MAX_BASE_BDEVS];
	struct iovec        chunk_iovs[EC_MAX_BASE_BDEVS];
	uint32_t            reads_remaining;
	enum spdk_bdev_io_status status;
};

/* =========================================================================
 * I/O buffer helpers
 * ========================================================================= */

static void
ec_free_io_buffers(struct ec_bdev_io *ec_io, const struct ec_bdev *ec)
{
	uint32_t i;

	if (ec_io->bounce_buf) {
		spdk_dma_free(ec_io->bounce_buf);
		ec_io->bounce_buf = NULL;
	}

	for (i = 0; i < ec->m; i++) {
		if (ec_io->parity_bufs[i]) {
			spdk_dma_free(ec_io->parity_bufs[i]);
			ec_io->parity_bufs[i] = NULL;
		}
	}
}

static int
ec_alloc_full_stripe(struct ec_bdev_io *ec_io, const struct ec_bdev *ec)
{
	uint32_t i;
	uint64_t chunk_bytes       = ec->strip_size * ec->bdev.blocklen;
	uint64_t total_data_bytes  = ec->stripe_blocks * ec->bdev.blocklen;

	ec_io->bounce_buf = spdk_dma_zmalloc(total_data_bytes, EC_DMA_ALIGN, NULL);
	if (!ec_io->bounce_buf) {
		return -ENOMEM;
	}

	/*
	 * WRITE_ZEROES fast path: spdk_dma_zmalloc already returned a zeroed
	 * buffer, so the data chunks are zero and the parity (RS_encode of all
	 * zeros) is also zero. Skip the iov copy.
	 */
	if (!ec_io->is_zero_fill) {
		spdk_copy_iovs_to_buf(ec_io->bounce_buf, total_data_bytes,
				      ec_io->iovs, ec_io->iovcnt);
	}

	for (i = 0; i < ec->k; i++) {
		ec_io->data_iovs[i].iov_base = (uint8_t *)ec_io->bounce_buf + (i * chunk_bytes);
		ec_io->data_iovs[i].iov_len  = chunk_bytes;
	}

	for (i = 0; i < ec->m; i++) {
		ec_io->parity_bufs[i] = spdk_dma_zmalloc(chunk_bytes, EC_DMA_ALIGN, NULL);
		if (!ec_io->parity_bufs[i]) {
			return -ENOMEM;
		}
		ec_io->parity_iovs[i].iov_base = ec_io->parity_bufs[i];
		ec_io->parity_iovs[i].iov_len  = chunk_bytes;
	}

	return 0;
}

/* =========================================================================
 * I/O submission helpers
 * ========================================================================= */

void
ec_bdev_io_init(struct ec_bdev_io *ec_io, struct ec_io_channel *ch,
		struct spdk_bdev_io *bdev_io)
{
	ec_io->bdev_io = bdev_io;
	ec_io->ch      = ch;

	ec_io->offset_blocks = bdev_io->u.bdev.offset_blocks;
	ec_io->num_blocks    = bdev_io->u.bdev.num_blocks;
	ec_io->iovs          = bdev_io->u.bdev.iovs;
	ec_io->iovcnt        = bdev_io->u.bdev.iovcnt;

	/*
	 * WRITE_ZEROES has no user payload; the full-stripe and RMW modify
	 * steps consult is_zero_fill to skip the iov copy and treat the
	 * modified region as zero. SPDK emulates UNMAP as WRITE_ZEROES when
	 * io_type_supported(UNMAP) is false, so UNMAP requests reach the
	 * native zero-fill path through this flag too.
	 */
	ec_io->is_zero_fill  = (bdev_io->type == SPDK_BDEV_IO_TYPE_WRITE_ZEROES);

	ec_io->base_io_remaining = 0;
	ec_io->status            = SPDK_BDEV_IO_STATUS_SUCCESS;

	ec_io->bounce_buf  = NULL;
	memset(ec_io->parity_bufs, 0, sizeof(ec_io->parity_bufs));

	ec_io->stripe_claimed     = false;
	ec_io->stripe_claim_index = 0;

	ec_io->wib_inflight_held  = false;
	ec_io->wib_region         = 0;

	ec_io->is_write_into_unmapped = false;
}

static void
ec_child_io_complete(struct spdk_bdev_io *child_io, bool success, void *cb_arg)
{
	struct ec_bdev_io *ec_io = cb_arg;
	struct ec_bdev    *ec    = ec_from_bdev_io(ec_io->bdev_io);

	spdk_bdev_free_io(child_io);

	if (!success) {
		ec_io->status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	ec_io->base_io_remaining--;
}

/* =========================================================================
 * Degraded read reconstruction
 * ========================================================================= */

static void
ec_free_degraded_read_ctx(struct ec_degraded_read_ctx *dctx,
			  const struct ec_bdev *ec)
{
	uint32_t i;

	for (i = 0; i < ec->n; i++) {
		if (dctx->chunk_bufs[i]) {
			spdk_dma_free(dctx->chunk_bufs[i]);
			dctx->chunk_bufs[i] = NULL;
		}
	}

	free(dctx);
}

/*
 * Shared ISA-L RS reconstruction helper used by degraded reads, RMW, and
 * rebuild (data slots only).
 *
 * Preconditions:
 *   - src_bufs[i] is valid for all readable slots (ec_slot_is_readable).
 *   - out_buf receives the reconstructed data for failed_slot.
 *   - failed_slot < ec->k  (data chunk only; parity uses encode path).
 *   - chunk_len is the number of bytes per chunk.
 *
 * Returns 0 on success, -1 on matrix inversion failure.
 */
int
ec_reconstruct_data_chunk(const struct ec_bdev *ec,
			  uint8_t *src_bufs[EC_MAX_BASE_BDEVS],
			  uint8_t *out_buf,
			  uint32_t failed_slot,
			  uint64_t chunk_len)
{
	uint32_t k = ec->k;
	uint32_t n = ec->n;
	uint8_t  avail_matrix[EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS];
	uint8_t  invert_matrix[EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS];
	uint8_t  decode_matrix[EC_MAX_BASE_BDEVS];
	uint8_t  decode_tbls[EC_ISAL_GF_TABLE_BYTES * EC_MAX_BASE_BDEVS];
	uint8_t *kptrs[EC_MAX_BASE_BDEVS];   /* k source pointers */
	uint8_t *optr[1];                    /* 1 output */
	uint32_t avail_row = 0;
	uint32_t disk;
	int      rc;

	for (disk = 0; disk < n && avail_row < k; disk++) {
		if (!ec_slot_is_readable(ec, disk)) {
			continue;
		}
		memcpy(&avail_matrix[avail_row * k],
		       &ec->encode_matrix[disk * k],
		       k);
		kptrs[avail_row] = src_bufs[disk];
		avail_row++;
	}

	if (avail_row < k) {
		SPDK_ERRLOG("EC bdev %s: not enough readable disks to reconstruct "
			    "(need %u, found %u)\n",
			    ec->bdev.name, k, avail_row);
		return -1;
	}

	rc = gf_invert_matrix(avail_matrix, invert_matrix, k);
	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: gf_invert_matrix failed (rc=%d) "
			    "while reconstructing slot %u (k=%u, %u readable "
			    "disks selected); the survivor set produced a "
			    "singular matrix -- likely a base bdev returned "
			    "corrupt data or the encode_matrix was clobbered\n",
			    ec->bdev.name, rc, failed_slot, k, avail_row);
		return -1;
	}

	/* Row failed_slot of the inverted matrix decodes that chunk. */
	memcpy(decode_matrix, &invert_matrix[failed_slot * k], k);

	ec_init_tables(k, 1, decode_matrix, decode_tbls);

	optr[0] = out_buf;
	ec_encode_data((int)chunk_len, (int)k, 1, decode_tbls, kptrs, optr);

	return 0;
}

/*
 * Reconstruct f simultaneously-failed DATA slots in a single ISA-L pass.
 * One gf_invert_matrix + one ec_encode_data, strictly cheaper than f calls to
 * ec_reconstruct_data_chunk for f > 1.
 *
 * Preconditions:
 *   - failed_data_slots[0..f-1] all < ec->k  (data slots only)
 *   - 1 <= f <= ec->m  (guaranteed by offline guard)
 *   - out_bufs[j] is a valid DMA buffer of chunk_len bytes for each j
 *   - src_bufs[i] is valid for all readable i
 *
 * Returns 0 on success, -1 on matrix inversion failure.
 */
int
ec_reconstruct_multi_data(const struct ec_bdev *ec,
			   uint8_t *src_bufs[EC_MAX_BASE_BDEVS],
			   uint8_t *out_bufs[],
			   const uint32_t failed_data_slots[],
			   uint32_t f,
			   uint64_t chunk_len)
{
	uint32_t k = ec->k;
	uint32_t n = ec->n;
	uint8_t  avail_matrix[EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS];
	uint8_t  invert_matrix[EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS];
	uint8_t  decode_matrix[EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS]; /* fxk */
	/*
	 * ec_init_tables requires EC_ISAL_GF_TABLE_BYTES * k * f bytes.
	 * Sized at EC_ISAL_GF_TABLE_BYTES * EC_MAX_BASE_BDEVS^2 (32 KB) to
	 * cover all k, f <= EC_MAX_BASE_BDEVS. SPDK reactor threads have 8
	 * MB stacks; 35 KB total frame is fine.
	 */
	uint8_t  decode_tbls[EC_ISAL_GF_TABLE_BYTES * EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS];
	uint8_t *kptrs[EC_MAX_BASE_BDEVS];   /* k source pointers */
	uint32_t avail_row = 0;
	uint32_t disk, j;
	int      rc;

	assert(f >= 1 && f <= ec->m);

	/* Step 1: build kxk avail_matrix from readable disk rows */
	for (disk = 0; disk < n && avail_row < k; disk++) {
		if (!ec_slot_is_readable(ec, disk)) {
			continue;
		}
		memcpy(&avail_matrix[avail_row * k],
		       &ec->encode_matrix[disk * k],
		       k);
		kptrs[avail_row] = src_bufs[disk];
		avail_row++;
	}

	if (avail_row < k) {
		SPDK_ERRLOG("EC bdev %s: not enough readable disks to reconstruct "
			    "%u failed slots (need %u, found %u)\n",
			    ec->bdev.name, f, k, avail_row);
		return -1;
	}

	/* Step 2: invert the kxk avail_matrix */
	rc = gf_invert_matrix(avail_matrix, invert_matrix, k);
	if (rc != 0) {
		/*
		 * Format the failed-slot list for the diagnostic log.
		 * Sized for EC_MAX_BASE_BDEVS three-digit slot numbers with
		 * comma separators plus the ",..." truncation marker; the
		 * loop appends the marker on overflow so the operator can
		 * tell the list was clipped.
		 */
		char failed_slots_str[EC_MAX_BASE_BDEVS * 4 + 8];
		size_t off = 0;
		bool truncated = false;

		for (j = 0; j < f; j++) {
			int written = snprintf(failed_slots_str + off,
					       sizeof(failed_slots_str) - off,
					       "%s%u",
					       j == 0 ? "" : ",",
					       failed_data_slots[j]);
			if (written < 0 ||
			    (size_t)written >= sizeof(failed_slots_str) - off) {
				truncated = true;
				break;
			}
			off += (size_t)written;
		}
		if (truncated) {
			snprintf(failed_slots_str + off,
				 sizeof(failed_slots_str) - off, ",...");
		}
		SPDK_ERRLOG("EC bdev %s: gf_invert_matrix failed (rc=%d) "
			    "while multi-decode reconstructing %u failed data "
			    "slots [%s] (k=%u, %u readable disks selected); the "
			    "survivor set produced a singular matrix\n",
			    ec->bdev.name, rc, f, failed_slots_str, k, avail_row);
		return -1;
	}

	/*
	 * Step 3: extract f rows from invert_matrix.
	 *
	 * Row failed_data_slots[j] of invert_matrix is the decode vector for
	 * data slot failed_data_slots[j].  (Proof: rows 0..k-1 of the RS
	 * encode_matrix are the identity; so for a surviving data row the
	 * avail_matrix row is the identity row, and its inverse row is also
	 * the identity row. For a failed data slot not in avail_matrix, the
	 * inverse row is the decode combination of the parity sources.)
	 * Place each extracted row as row j of the fxk decode_matrix.
	 */
	for (j = 0; j < f; j++) {
		memcpy(&decode_matrix[j * k],
		       &invert_matrix[failed_data_slots[j] * k],
		       k);
	}

	/* Step 4 + 5: initialise tables and decode all f outputs in one pass */
	ec_init_tables(k, f, decode_matrix, decode_tbls);
	ec_encode_data((int)chunk_len, (int)k, (int)f, decode_tbls, kptrs, out_bufs);

	return 0;
}

/*
 * Wrapper used by the degraded read path.
 * Fills dctx->chunk_bufs[failed_slot] with reconstructed data.
 */
static int
ec_reconstruct_missing_chunk(const struct ec_bdev *ec,
			     struct ec_degraded_read_ctx *dctx,
			     uint32_t failed_slot)
{
	uint64_t chunk_bytes = dctx->chunk_blocks * ec->bdev.blocklen;

	return ec_reconstruct_data_chunk(ec, dctx->chunk_bufs,
					 dctx->chunk_bufs[failed_slot],
					 failed_slot, chunk_bytes);
}

static void
ec_degraded_read_complete(struct ec_degraded_read_ctx *dctx)
{
	struct ec_bdev_io *ec_io      = dctx->ec_io;
	struct ec_bdev    *ec         = ec_from_bdev_io(ec_io->bdev_io);
	uint32_t           target     = dctx->target_chunk;
	uint64_t           chunk_bytes = dctx->chunk_blocks * ec->bdev.blocklen;
	uint64_t           offset_bytes, copy_bytes;
	int                rc;

	if (dctx->status != SPDK_BDEV_IO_STATUS_SUCCESS) {
		SPDK_ERRLOG("EC bdev %s: degraded read child I/O failed "
			    "for chunk %u\n", ec->bdev.name, target);
		ec_free_degraded_read_ctx(dctx, ec);
		spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	if (!ec_slot_is_readable(ec, target) && target < ec->k) {
		rc = ec_reconstruct_missing_chunk(ec, dctx, target);
		if (rc != 0) {
			SPDK_ERRLOG("EC bdev %s: reconstruction failed for "
				    "chunk %u\n", ec->bdev.name, target);
			ec_free_degraded_read_ctx(dctx, ec);
			spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
			return;
		}
	}

	offset_bytes = dctx->chunk_offset * ec->bdev.blocklen;
	copy_bytes   = ec_io->num_blocks  * ec->bdev.blocklen;

	if (offset_bytes + copy_bytes > chunk_bytes) {
		SPDK_ERRLOG("EC bdev %s: copy range [%" PRIu64 "+%" PRIu64 "] exceeds "
			    "chunk size %" PRIu64 "\n",
			    ec->bdev.name, offset_bytes, copy_bytes, chunk_bytes);
		ec_free_degraded_read_ctx(dctx, ec);
		spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	spdk_copy_buf_to_iovs(ec_io->iovs, ec_io->iovcnt,
		(uint8_t *)dctx->chunk_bufs[target] + offset_bytes,
		copy_bytes);

	ec_free_degraded_read_ctx(dctx, ec);
	spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_SUCCESS);
}

static void
ec_degraded_read_child_complete(struct spdk_bdev_io *child_io,
				bool success, void *cb_arg)
{
	struct ec_degraded_read_ctx *dctx = cb_arg;

	spdk_bdev_free_io(child_io);

	if (!success) {
		dctx->status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	dctx->reads_remaining--;

	if (dctx->reads_remaining == 0) {
		ec_degraded_read_complete(dctx);
	}
}

/*
 * Direct (non-degraded) read of a single chunk: the target chunk is readable,
 * so map it straight to one base-bdev read with no reconstruction. Shared by
 * the healthy fast path, the parity-only-failure path, and the degraded-read
 * "target still readable" case.
 */
static int
ec_submit_direct_read(struct ec_bdev_io *ec_io, uint32_t chunk_idx, uint64_t base_lba)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);

	ec_io->base_io_remaining = 1;
	ec_io->status = SPDK_BDEV_IO_STATUS_SUCCESS;

	return spdk_bdev_readv_blocks(ec->descs[chunk_idx],
		ec_io->ch->base_chans[chunk_idx],
		ec_io->iovs, ec_io->iovcnt,
		base_lba, ec_io->num_blocks,
		ec_child_io_complete, ec_io);
}

static int
ec_submit_degraded_read(struct ec_bdev_io *ec_io)
{
	struct ec_bdev              *ec = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t                     stripe_index, chunk_offset, base_lba;
	uint32_t                     chunk_idx;
	struct ec_degraded_read_ctx *dctx;
	uint64_t                     chunk_bytes;
	uint32_t                     disk, reads_submitted;
	uint64_t                     disk_lba;
	int                          rc;

	ec_calc_mapping(ec, ec_io->offset_blocks, &stripe_index, &chunk_idx,
			&chunk_offset, &base_lba);

	/* Case A: target chunk is readable -> direct read */
	if (ec_slot_is_readable(ec, chunk_idx)) {
		return ec_submit_direct_read(ec_io, chunk_idx, base_lba);
	}

	/* Case B: targeting a failed parity slot -- logic error */
	if (chunk_idx >= ec->k) {
		SPDK_ERRLOG("EC bdev %s: read targeting failed parity slot %u\n",
			    ec->bdev.name, chunk_idx);
		return -EINVAL;
	}

	/*
	 * Case C: target data chunk is unavailable -> ISA-L reconstruction.
	 *
	 * This path handles up to m simultaneous disk failures
	 * (any combination of data and parity). We only need to reconstruct
	 * the ONE target chunk -- we do not need to reconstruct the other failed
	 * data slots. ec_reconstruct_data_chunk picks k readable rows from the
	 * encode_matrix (which includes parity rows), inverts the kxk submatrix,
	 * and extracts the decode vector for the target slot. The MDS property
	 * of Reed-Solomon guarantees this inversion succeeds as long as at least
	 * k readable disks remain (i.e. failed_count <= m <= n-k).
	 *
	 * Dirty-region guard: if the target stripe's WIB region is still dirty
	 * (either because the startup scrub is running and has not yet cleared
	 * this region, or because the scrub was deferred due to a failed data
	 * disk), parity on disk may be stale relative to the data chunks.
	 * Reconstruction uses parity as an input; stale parity produces a
	 * result that is neither the pre-crash nor the post-crash value --
	 * silently wrong bytes. Return -EIO rather than serve garbage.
	 * The caller surfaces this as a hard I/O error; the data in this
	 * region is indeterminate until the scrub completes.
	 */
	if (ec_wib_region_is_dirty(ec, ec_wib_stripe_to_region(stripe_index))) {
		/*
		 * Region-level guard: conservative by design. The WIB marks
		 * an entire 1024-stripe region dirty even if only one stripe
		 * was mid-RMW at the crash, so this EIO fires for all stripes
		 * in the region, not just the one with stale parity.
		 *
		 * Future refinement: when scrub_ctx is active, stripes with
		 * stripe_index < sctx->current_stripe have already been
		 * re-encoded by the scrubber and are safe to reconstruct.
		 * Not implemented here because the EIO window is typically
		 * well under one second per region on local NVMe (one region =
		 * k x strip_size_kb x 1024 bytes; e.g. 64 MiB for k=2/32KB,
		 * 256 MiB for k=4/64KB), and real availability impact has not
		 * been observed. Add the sctx->current_stripe range check if
		 * production data shows this window is wide enough to matter.
		 */
		/*
		 * Production signal: ec->degraded_read_eio_dirty is the
		 * counter exposed via bdev_get_bdevs / bdev_ec_get_bdevs.
		 * No per-I/O log -- a sustained read-storm in the brief
		 * scrub window would flood the system log otherwise.
		 */
		ec->degraded_read_eio_dirty++;
		return -EIO;
	}

	ec->degraded_reads_reconstructed++;

	dctx = calloc(1, sizeof(*dctx));
	if (!dctx) {
		return -ENOMEM;
	}

	dctx->ec_io        = ec_io;
	dctx->target_chunk = chunk_idx;
	dctx->chunk_offset = chunk_offset;
	dctx->status       = SPDK_BDEV_IO_STATUS_SUCCESS;
	dctx->chunk_blocks = ec->strip_size;
	disk_lba           = ec_stripe_base_lba(ec, stripe_index);
	chunk_bytes        = dctx->chunk_blocks * ec->bdev.blocklen;

	/*
	 * Allocate a buffer for every slot. Only the first k readable slots
	 * (read sources) plus the target slot (reconstruction output / result
	 * copied to the caller) are actually used, so m-1 buffers are over-
	 * allocated per degraded read. Trimming them is deliberately not done:
	 * it would interleave allocation with the first-k-readable selection in
	 * the read loop below and hinge on the target-readable-beyond-k edge
	 * case, adding failure-path complexity to save a page-aligned buffer in
	 * degraded mode only. The simple allocate-all is obviously correct;
	 * revisit with degraded-read test coverage if it ever matters.
	 */
	for (disk = 0; disk < ec->n; disk++) {
		dctx->chunk_bufs[disk] = spdk_dma_zmalloc(chunk_bytes,
							   EC_DMA_ALIGN, NULL);
		if (!dctx->chunk_bufs[disk]) {
			ec_free_degraded_read_ctx(dctx, ec);
			return -ENOMEM;
		}
		dctx->chunk_iovs[disk].iov_base = dctx->chunk_bufs[disk];
		dctx->chunk_iovs[disk].iov_len  = chunk_bytes;
	}

	reads_submitted       = 0;
	dctx->reads_remaining = 0;

	for (disk = 0; disk < ec->n; disk++) {
		if (!ec_slot_is_readable(ec, disk)) {
			continue;
		}
		if (reads_submitted >= ec->k) {
			break;
		}

		dctx->reads_remaining++;

		rc = spdk_bdev_readv_blocks(ec->descs[disk],
			ec_io->ch->base_chans[disk],
			&dctx->chunk_iovs[disk], 1,
			disk_lba, dctx->chunk_blocks,
			ec_degraded_read_child_complete, dctx);

		if (rc != 0) {
			dctx->reads_remaining--;
			dctx->status = SPDK_BDEV_IO_STATUS_FAILED;
			SPDK_ERRLOG("EC bdev %s: failed to submit degraded "
				    "read child for disk %u (rc=%d)\n",
				    ec->bdev.name, disk, rc);
		} else {
			reads_submitted++;
		}
	}

	if (reads_submitted < ec->k) {
		SPDK_ERRLOG("EC bdev %s: only %u/%u reads submitted; "
			    "failing I/O\n",
			    ec->bdev.name, reads_submitted, ec->k);
		if (dctx->reads_remaining == 0) {
			ec_free_degraded_read_ctx(dctx, ec);
			return -EIO;
		}
		dctx->status = SPDK_BDEV_IO_STATUS_FAILED;
	}

	return 0;
}
ec_submit_read(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t        stripe_index, chunk_offset, base_lba;
	uint32_t        chunk_idx;

	/* Parity-only failure -- all data disks healthy, direct read */
	if (ec_only_parity_failed(ec)) {
		ec_calc_mapping(ec, ec_io->offset_blocks, &stripe_index, &chunk_idx,
				&chunk_offset, &base_lba);
		return ec_submit_direct_read(ec_io, chunk_idx, base_lba);
	}

	/* Data disk failed or REPLACING -- degraded read */
	if (ec->failed_count > 0) {
		return ec_submit_degraded_read(ec_io);
	}

	/* Fast path: all disks healthy */
	ec_calc_mapping(ec, ec_io->offset_blocks, &stripe_index, &chunk_idx,
			&chunk_offset, &base_lba);
	return ec_submit_direct_read(ec_io, chunk_idx, base_lba);
}

/* =========================================================================
 * Full-stripe write path
 * ========================================================================= */

static void ec_full_write_fanout(struct ec_bdev_io *ec_io);

/*
 * Release the resources a full-stripe write holds on an error path: the
 * WIB-region inflight count, the stripe-busy claim, and the chunk buffers.
 * Each guard is idempotent, so callers just complete (or return) afterward.
 */
static void
ec_full_write_unwind(struct ec_bdev_io *ec_io, struct ec_bdev *ec)
{
	if (ec_io->wib_inflight_held) {
		ec_wib_region_inflight_dec(ec, ec_io->wib_region);
		ec_io->wib_inflight_held = false;
	}
	if (ec_io->stripe_claimed) {
		ec_stripe_clear_dirty(ec, ec_io->stripe_claim_index);
		ec_io->stripe_claimed = false;
	}
	ec_free_io_buffers(ec_io, ec);
}

/*
 * Callback fired by ec_wib_persist when the WIB region bit for this
 * full-stripe write has been persisted to disk. On success, continues
 * the submission chain by encoding parity and fanning out child writes.
 * On failure, releases the stripe-busy claim and the WIB inflight count,
 * then completes the bdev_io with FAILED status.
 */
static void
ec_full_write_wib_set_cb(void *cb_arg, int rc)
{
	struct ec_bdev_io *ec_io = cb_arg;
	struct ec_bdev    *ec    = ec_from_bdev_io(ec_io->bdev_io);

	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: full-stripe WIB persist failed (rc=%d) "
			    "before write fan-out at stripe %" PRIu64 "\n",
			    ec->bdev.name, rc, ec_io->stripe_claim_index);
		ec_full_write_release(ec_io, ec);
		spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
		return;
	}

	ec_full_write_fanout(ec_io);
}

static int
ec_submit_full_write(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	int             rc = 0;

	/*
	 * This function runs on the home thread because the WIB persist
	 * below (ec_wib_persist) uses ec->wib_chans[], which are owned by
	 * that thread.
	 *
	 * Routing the call to home when entered from a different thread is
	 * deferred to the multi-reactor follow-up. It would mean replacing
	 * the synchronous error returns here (-EAGAIN for retry, or a hard
	 * failure that rolls back the WIB dirty bit and completes the
	 * bdev_io as FAILED) with cb-delivered SPDK_BDEV_IO_STATUS_NOMEM
	 * completions; the UNMAP and RMW paths need the same change.
	 *
	 * Only single-reactor SPDK targets are supported. The assert
	 * catches accidental cross-thread use; without it the persist
	 * would dispatch I/O on a channel it doesn't own.
	 */
	assert(spdk_get_thread() == ec->home_thread);

	/*
	 * Active-scrub guard: a full-stripe write encodes parity from the new
	 * data it supplies, then writes both. If the startup scrubber is
	 * concurrently reading the same stripe's old data to re-encode parity,
	 * this sequence produces a race:
	 *   1. Scrubber reads (D0_old ... Dk_old).
	 *   2. Full-stripe write lands: data becomes (D0_new ... Dk_new),
	 *      parity becomes encode(D0_new ... Dk_new).
	 *   3. Scrubber writes encode(D0_old ... Dk_old), overwriting step 2's
	 *      correct parity with parity derived from stale data.
	 * Result: new data, old parity -- exactly the write-hole the WIB exists
	 * to prevent.
	 *
	 * The guard mirrors the active-scrub check in ec_submit_rmw_write:
	 * block stripes not yet passed by the scrubber in the current region,
	 * and all stripes in dirty regions ahead of it. Stripes the scrubber
	 * has already processed (stripe_index < sctx->current_stripe) are safe
	 * because the scrubber will not revisit them.
	 */
	if (ec->scrub_ctx != NULL) {
		struct ec_scrub_ctx *sctx        = ec->scrub_ctx;
		uint64_t             stripe_idx  = ec_io->offset_blocks / ec->stripe_blocks;
		uint32_t             region      = ec_wib_stripe_to_region(stripe_idx);

		if (region == sctx->current_region &&
		    stripe_idx >= sctx->current_stripe) {
			ec->full_stripe_writes_deferred++;
			return -EAGAIN;
		}
		if (region > sctx->current_region &&
		    ec_wib_region_is_dirty(ec, region)) {
			ec->full_stripe_writes_deferred++;
			return -EAGAIN;
		}
	}

	/*
	 * Stripe-busy interlock. Claim the stripe before submitting child
	 * writes so a concurrent rebuild or UNMAP on the same stripe defers
	 * via -EAGAIN. This closes a pre-existing race where an in-flight
	 * rebuild on this stripe could read pre-write data from NORMAL
	 * slots, reconstruct from it, and write the stale chunk to the
	 * REPLACING slot after the foreground writes had already landed --
	 * leaving the REPLACING slot inconsistent with the rest of the
	 * array. ec_child_io_complete releases the claim on final completion.
	 */
	{
		uint64_t stripe_idx = ec_io->offset_blocks / ec->stripe_blocks;

		if (ec_stripe_is_dirty(ec, stripe_idx)) {
			return -EAGAIN;
		}
		ec_stripe_set_dirty(ec, stripe_idx);
		ec_io->stripe_claimed     = true;
		ec_io->stripe_claim_index = stripe_idx;
	}

	/*
	 * ec_submit_full_write handles exactly one stripe per call.
	 * ec_alloc_full_stripe copies exactly stripe_blocks * blocklen bytes
	 * from the caller's iovecs, so a multi-stripe call would silently
	 * process only the first stripe and lose the rest.
	 *
	 * ec_submit_write only routes here when num_blocks is a non-zero
	 * multiple of stripe_blocks and the offset is stripe-aligned. With
	 * optimal_io_boundary = strip_size and split_on_optimal_io_boundary,
	 * the upper layer never sends a write larger than one strip, so
	 * num_blocks == stripe_blocks is the only realistic case today.
	 *
	 * If that invariant is ever violated (e.g. a raw bdev caller bypasses
	 * splitting), fail loudly rather than silently corrupt data.
	 */
	if (ec_io->num_blocks != ec->stripe_blocks) {
		SPDK_ERRLOG("EC bdev %s: ec_submit_full_write called with "
			    "num_blocks=%" PRIu64 " but stripe_blocks=%" PRIu64
			    "; splitter bypass detected\n",
			    ec->bdev.name,
			    ec_io->num_blocks, ec->stripe_blocks);
		rc = -EINVAL;
		goto error;
	}

	/*
	 * WIB region marking. A crash partway through the fan-out can
	 * leave a subset of the k+m chunks at the new value and the rest
	 * at the old value. Without a WIB bit, recovery has no record
	 * that this stripe was mid-write, so no scrub runs and parity
	 * stays inconsistent with whatever data did land -- a later disk
	 * failure would reconstruct using stale parity and surface
	 * silently wrong bytes (the same write-hole the WIB prevents for
	 * RMW). The mark + inflight + dirty_ts trio mirrors the RMW and
	 * UNMAP submit paths exactly; the idle WIB poller is gated by
	 * all three so the bit cannot be cleared mid-fanout.
	 */
	{
		uint64_t stripe_idx = ec_io->stripe_claim_index;
		uint32_t region     = ec_wib_stripe_to_region(stripe_idx);
		bool     any_was_clean = false;

		if (!ec_wib_region_is_dirty(ec, region)) {
			ec_wib_region_set_dirty(ec, region);
			any_was_clean = true;
		}
		ec_wib_region_inflight_inc(ec, region);
		ec->wib_region_dirty_ticks[region] = spdk_get_ticks();

		ec_io->wib_region        = region;
		ec_io->wib_inflight_held = true;

		/*
		 * Persist decision tree mirrors the RMW / UNMAP submit
		 * paths. wib_persist_in_flight must be consulted on every
		 * branch -- even when the in-memory bit was already dirty,
		 * an in-flight persist may be *clearing* the bit (initiated
		 * by the idle poller before our setter ran). Proceeding
		 * straight to fan-out in that window would let a crash
		 * leave on-disk WIB clean while the stripe is mid-write.
		 */
		if (ec->wib_persist_in_flight) {
			ec->wib_repersist_needed = true;
			ec->full_stripe_writes_deferred++;
			rc = -EAGAIN;
			goto error;
		}

		/*
		 * Past every reject (-EINVAL) and defer (-EAGAIN) gate: the
		 * write is now accepted and will fan out. Count it here so a
		 * deferred-then-retried write is not counted on each attempt and
		 * an invalid request is not counted at all.
		 */
		ec->full_stripe_writes++;

		if (any_was_clean) {
			int persist_rc;

			rc = ec_alloc_full_stripe(ec_io, ec);
			if (rc != 0) {
				/*
				 * We set this region's WIB bit above, but no
				 * persist will record it and the error path does
				 * not clear it. Roll it back so the in-memory bit
				 * never outlives its on-disk record: otherwise the
				 * NOMEM retry would see the region already dirty,
				 * skip the WIB persist, and fan out data with no
				 * recoverable write-intent. Safe -- no fan-out
				 * happened, so nothing needs scrubbing.
				 */
				ec_wib_region_clear_dirty(ec, region);
				goto error;
			}

			persist_rc = ec_wib_persist(ec, ec_full_write_wib_set_cb, ec_io);
			if (persist_rc != 0) {
				/* Same rollback as the alloc-failure path above. */
				ec_wib_region_clear_dirty(ec, region);
				rc = persist_rc;
				goto error;
			}
			return 0;
		}
	}

	rc = ec_alloc_full_stripe(ec_io, ec);
	if (rc != 0) {
		goto error;
	}

	ec_full_write_fanout(ec_io);
	return 0;

error:
	ec_full_write_unwind(ec_io, ec);
	return rc;
}

/*
 * Encode parity and fan out the k+m child writes. Reached either
 * directly from ec_submit_full_write (region was already dirty on
 * disk) or via ec_full_write_wib_set_cb after a WIB persist completes
 * (region transitioned clean -> dirty). The bdev_io is completed
 * asynchronously via ec_child_io_complete once all child writes have
 * finished.
 */
static void
ec_full_write_fanout(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec   = ec_from_bdev_io(ec_io->bdev_io);
	uint32_t        i;
	uint64_t        chunk_blocks = ec->strip_size;
	uint64_t        chunk_bytes  = chunk_blocks * ec->bdev.blocklen;
	unsigned char  *data_ptrs[EC_MAX_BASE_BDEVS];
	unsigned char  *parity_ptrs[EC_MAX_BASE_BDEVS];
	uint64_t        stripe_index;
	uint64_t        offset_in_disk;
	uint32_t        writable_count;
	int             rc;

	for (i = 0; i < ec->k; i++) {
		data_ptrs[i] = ec_io->data_iovs[i].iov_base;
	}
	for (i = 0; i < ec->m; i++) {
		parity_ptrs[i] = ec_io->parity_iovs[i].iov_base;
	}

	ec_encode_data(chunk_bytes, ec->k, ec->m, ec->g_tbls, data_ptrs, parity_ptrs);

	/* Full-stripe write touches every chunk, so only the stripe index is
	 * needed (not the per-chunk mapping); the base LBA is the same for all. */
	stripe_index   = ec_io->offset_blocks / ec->stripe_blocks;
	offset_in_disk = ec_stripe_base_lba(ec, stripe_index);

	writable_count = 0;
	for (i = 0; i < ec->n; i++) {
		if (ec_slot_is_writable(ec, i)) {
			writable_count++;
		}
	}

	ec_io->base_io_remaining = writable_count;
	ec_io->status = SPDK_BDEV_IO_STATUS_SUCCESS;

	if (ec_io->base_io_remaining == 0) {
		SPDK_ERRLOG("EC bdev %s: full-stripe write to stripe %" PRIu64 " has no "
			    "writable slots (%u of %u disks unavailable); "
			    "completing FAILED\n",
			    ec->bdev.name, stripe_index,
			    ec->n - writable_count, ec->n);
		goto error;
	}

	for (i = 0; i < ec->k; i++) {
		if (!ec_slot_is_writable(ec, i)) {
			continue;
		}
		rc = spdk_bdev_writev_blocks(ec->descs[i],
			ec_io->ch->base_chans[i],
			&ec_io->data_iovs[i], 1,
			offset_in_disk, chunk_blocks,
			ec_child_io_complete, ec_io);
		if (rc != 0) {
			ec_io->base_io_remaining--;
			ec_io->status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}

	for (i = 0; i < ec->m; i++) {
		uint32_t bdev_idx = ec->k + i;
		if (!ec_slot_is_writable(ec, bdev_idx)) {
			continue;
		}
		rc = spdk_bdev_writev_blocks(ec->descs[bdev_idx],
			ec_io->ch->base_chans[bdev_idx],
			&ec_io->parity_iovs[i], 1,
			offset_in_disk, chunk_blocks,
			ec_child_io_complete, ec_io);
		if (rc != 0) {
			ec_io->base_io_remaining--;
			ec_io->status = SPDK_BDEV_IO_STATUS_FAILED;
		}
	}

	if (ec_io->base_io_remaining == 0) {
		goto error;
	}

	return;

error:
	ec_full_write_unwind(ec_io, ec);
	spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
}

int
ec_submit_write(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);

	/*
	 * Full-stripe fast path: the I/O covers one or more complete stripes
	 * with no partial leading or trailing data.
	 *
	 * write_unit_size=1 means any size/offset is legal. With
	 * split_on_optimal_io_boundary=true (boundary = strip_size) the
	 * upper layer splits on strip boundaries, so in practice
	 * num_blocks <= strip_size always. A write of exactly stripe_blocks
	 * at a stripe-aligned offset is the only case that reaches here as a
	 * full stripe; all other cases go to RMW.
	 *
	 * We keep the condition general (multiple of stripe_blocks, aligned)
	 * in case the caller issues a larger aligned write without splitting.
	 */
	if (ec_io->num_blocks % ec->stripe_blocks == 0 &&
	    ec_io->offset_blocks % ec->stripe_blocks == 0) {
		return ec_submit_full_write(ec_io);
	}

	return ec_submit_rmw_write(ec_io);
}
