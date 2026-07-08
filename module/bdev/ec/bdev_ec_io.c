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

/* Forward declaration; defined near the full-stripe write path. */
static void ec_io_release_state(struct ec_bdev_io *ec_io, struct ec_bdev *ec);

static void
ec_free_io_buffers(struct ec_bdev_io *ec_io, const struct ec_bdev *ec)
{
	uint32_t i;

	if (ec_io->bounce_buf) {
		spdk_dma_free(ec_io->bounce_buf);
		ec_io->bounce_buf = NULL;
	}

	if (ec_io->parity_bufs) {
		for (i = 0; i < ec->m; i++) {
			if (ec_io->parity_bufs[i]) {
				spdk_dma_free(ec_io->parity_bufs[i]);
			}
		}
		free(ec_io->parity_bufs);
		ec_io->parity_bufs = NULL;
	}

	if (ec_io->parity_iovs) {
		free(ec_io->parity_iovs);
		ec_io->parity_iovs = NULL;
	}

	if (ec_io->data_iovs) {
		free(ec_io->data_iovs);
		ec_io->data_iovs = NULL;
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

	ec_io->data_iovs = calloc(ec->k, sizeof(struct iovec));
	if (!ec_io->data_iovs) {
		return -ENOMEM;
	}

	for (i = 0; i < ec->k; i++) {
		ec_io->data_iovs[i].iov_base = (uint8_t *)ec_io->bounce_buf + (i * chunk_bytes);
		ec_io->data_iovs[i].iov_len  = chunk_bytes;
	}

	ec_io->parity_iovs = calloc(ec->m, sizeof(struct iovec));
	ec_io->parity_bufs = calloc(ec->m, sizeof(void *));
	if (!ec_io->parity_iovs || !ec_io->parity_bufs) {
		return -ENOMEM;
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
	 * is_zero_fill defaults to false on entry. Internal zero-fill paths
	 * flip it to true after init -- ec_submit_unmap's single-stripe
	 * shortcut and ec_submit_write_into_unmapped's pre-fill save/restore
	 * dance. WRITE_ZEROES is not advertised by ec_io_type_supported, so
	 * the bdev layer always emulates it as a buffer-backed WRITE and
	 * never enters here with bdev_io->type == WRITE_ZEROES.
	 */
	ec_io->is_zero_fill  = false;

	ec_io->base_io_remaining = 0;
	ec_io->status            = SPDK_BDEV_IO_STATUS_SUCCESS;

	ec_io->data_iovs   = NULL;
	ec_io->parity_iovs = NULL;
	ec_io->parity_bufs = NULL;
	ec_io->bounce_buf  = NULL;

	ec_io->stripe_claimed     = false;
	ec_io->stripe_claim_index = 0;

	ec_io->wib_inflight_held  = false;
	ec_io->wib_region         = 0;

	ec_io->is_write_into_unmapped = false;

	/*
	 * ec_bdev_io_init runs on the channel's owner thread (= the thread
	 * that submitted the bdev_io), since SPDK invokes submit_request on
	 * that thread. Capture it here so completion routing can dispatch
	 * spdk_bdev_io_complete back to this thread when needed.
	 */
	ec_io->submitter_thread = spdk_get_thread();
}

/*
 * Finalize routine for the write-into-unmapped path, invoked on the
 * bdev_io's owning spdk_thread (see ec_write_into_unmapped_bit_cleared
 * for the thread hand-off).
 */
static void
ec_write_into_unmapped_finalize(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;
	struct ec_bdev    *ec    = ec_from_bdev_io(ec_io->bdev_io);

	ec_io_release_state(ec_io, ec);
	spdk_bdev_io_complete(ec_io->bdev_io, ec_io->status);
}

/*
 * Bit-clear completion callback for the write-into-unmapped path.
 * Invoked from ec_submit_bit_clear_async's persist drainage after the
 * unmapped bit has been durably cleared (m+1 ack) -- the load-bearing
 * step that flips the bdev_io from "data on disk but masked by bitmap"
 * to "data visible to readers." Only NOW is it safe to ack the caller.
 *
 * Failure (rc != 0): the on-disk bitmap still says unmapped, so reads
 * will continue synthesising zeros -- the just-written data is
 * effectively lost. We fail the bdev_io so the caller knows; they may
 * retry, which will land back through ec_submit_write_into_unmapped and
 * attempt the bit-clear again.
 *
 * Thread hand-off: ec->bitmap_chans[] are owned by the EC bdev's
 * creation spdk_thread (typically the app main thread), so the bitmap
 * persist completion fires there. The WRITE bdev_io we are about to
 * complete is owned by whichever spdk_thread submitted it (an nvmf
 * poll group, a raid child, etc.). spdk_bdev_io_complete asserts that
 * the caller thread == spdk_bdev_io_get_thread(bdev_io); calling it
 * cross-thread aborts the process. We send the finalize routine to
 * the bdev_io's owner thread via spdk_thread_send_msg.
 *
 * The existing UNMAP path does not need this because its bitmap-persist
 * completion is followed by a fan-out to the base bdevs whose
 * completions naturally land on the bdev_io's owner thread (the same
 * thread that opened the per-channel base_chans the fan-out uses).
 * Write-into-unmapped has no equivalent post-persist fan-out: the data
 * writes already completed before the bit-clear started, so the
 * persist-completion thread is the last hop before the caller ack.
 */
static void
ec_write_into_unmapped_bit_cleared(void *cb_arg, int rc)
{
	struct ec_bdev_io  *ec_io = cb_arg;
	struct ec_bdev     *ec    = ec_from_bdev_io(ec_io->bdev_io);
	struct spdk_thread *owner;

	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: write-into-unmapped bit-clear persist "
			    "failed (rc=%d) at stripe %" PRIu64 "; failing bdev_io. "
			    "Data is on disk but masked by bitmap.\n",
			    ec->bdev.name, rc, ec_io->stripe_claim_index);
		ec_io->status = SPDK_BDEV_IO_STATUS_FAILED;
		__atomic_fetch_add(&ec->writes_into_unmapped_failed, 1, __ATOMIC_RELAXED);
	}

	owner = spdk_bdev_io_get_thread(ec_io->bdev_io);
	if (spdk_likely(owner == spdk_get_thread())) {
		ec_write_into_unmapped_finalize(ec_io);
	} else {
		int send_rc = spdk_thread_send_msg(owner,
						   ec_write_into_unmapped_finalize, ec_io);
		if (send_rc != 0) {
			SPDK_ERRLOG("EC bdev %s: cannot hand off bdev_io completion "
				    "to owner thread '%s' (rc=%d %s) at stripe %" PRIu64 "; "
				    "releasing EC-layer state, bdev_io stays in-flight "
				    "until SPDK timeout fires\n",
				    ec->bdev.name, spdk_thread_get_name(owner),
				    send_rc, spdk_strerror(-send_rc), ec_io->stripe_claim_index);
			ec_io_release_state(ec_io, ec);
			/* spdk_bdev_io_complete is owner-thread-only -- the
			 * bdev_io cannot be completed from here. */
		}
	}
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

	if (ec_io->base_io_remaining == 0) {
		/*
		 * Write-into-unmapped routing: data has landed (success) or
		 * partially failed. On success, defer bdev_io completion until
		 * the bit-clear persist acks at m+1. On failure, fall through
		 * to immediate completion with the failed status -- skipping
		 * the bit-clear is correct because the on-disk data is
		 * inconsistent and the bitmap-still-says-unmapped state is
		 * exactly what masks the bad chunks from readers.
		 *
		 * The bit-clear submit can itself fail synchronously (-ENOMEM
		 * on the waiter alloc, -EINVAL on bad args). Treat that the
		 * same as a fanout failure: leave the bitmap saying unmapped,
		 * fail the bdev_io. The stripe-busy and buffers are torn down
		 * in the fall-through.
		 *
		 * The bit-clear runs after we have released wib_inflight (the
		 * WIB region is no longer load-bearing once all child writes
		 * have completed), so we drop wib_inflight here regardless of
		 * which branch runs next.
		 */
		if (ec_io->wib_inflight_held) {
			if (ec_io->status != SPDK_BDEV_IO_STATUS_SUCCESS) {
				/*
				 * Partial failure may leave parity inconsistent with
				 * data. Mark crash-dirty before releasing the inflight
				 * ref (see ec_wib_mark_failed_write for why the order
				 * matters).
				 */
				ec_wib_mark_failed_write(ec, ec_io->wib_region);
			}
			ec_wib_region_inflight_dec(ec, ec_io->wib_region);
			ec_io->wib_inflight_held = false;
		}

		if (ec_io->is_write_into_unmapped &&
		    ec_io->status == SPDK_BDEV_IO_STATUS_SUCCESS) {
			int rc;

			rc = ec_submit_bit_clear_async(ec,
						       ec_io->stripe_claim_index,
						       ec_write_into_unmapped_bit_cleared,
						       ec_io);
			if (rc == 0) {
				/* Async path: cb fires later, do not complete. */
				return;
			}
			SPDK_ERRLOG("EC bdev %s: write-into-unmapped bit-clear "
				    "submit failed (rc=%d) at stripe %" PRIu64 "\n",
				    ec->bdev.name, rc,
				    ec_io->stripe_claim_index);
			ec_io->status = SPDK_BDEV_IO_STATUS_FAILED;
			__atomic_fetch_add(&ec->writes_into_unmapped_failed, 1, __ATOMIC_RELAXED);
		}

		ec_io_release_state(ec_io, ec);
		spdk_bdev_io_complete(ec_io->bdev_io, ec_io->status);
	}
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
	uint8_t  decode_tbls[32 * EC_MAX_BASE_BDEVS];
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
	 * ec_init_tables requires 32*k*f bytes.
	 * Sized at 32 * EC_MAX_BASE_BDEVS^2 (32 KB) to cover all k, f <= 32.
	 * SPDK reactor threads have 8 MB stacks; 35 KB total frame is fine.
	 */
	uint8_t  decode_tbls[32 * EC_MAX_BASE_BDEVS * EC_MAX_BASE_BDEVS];
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

	/* Case C: targeting a failed parity slot -- logic error */
	if (chunk_idx >= ec->k) {
		SPDK_ERRLOG("EC bdev %s: read targeting failed parity slot %u\n",
			    ec->bdev.name, chunk_idx);
		return -EINVAL;
	}

	/*
	 * Case B: target data chunk is unavailable -> ISA-L reconstruction.
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
		__atomic_fetch_add(&ec->degraded_read_eio_dirty, 1, __ATOMIC_RELAXED);
		return -EIO;
	}

	__atomic_fetch_add(&ec->degraded_reads_reconstructed, 1, __ATOMIC_RELAXED);

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

int
ec_submit_read(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t        stripe_index, chunk_offset, base_lba;
	uint32_t        chunk_idx;

	/*
	 * Dispatch invariant: reads use ec_io->ch->base_chans[], which are
	 * owned by the submitter thread. The read path is already cross-
	 * thread safe (no home-only state touched on the hot path), but the
	 * assertion documents the contract for any future refactor.
	 */
	assert(spdk_get_thread() == ec_io->submitter_thread);

	/*
	 * Unmapped-bitmap consultation. The EC bdev publishes
	 * optimal_io_boundary = strip_size with split_on_optimal_io_boundary,
	 * so every read arrives here within a single strip (and therefore
	 * within a single stripe). Computing stripe_index from offset_blocks
	 * alone is sufficient -- no per-stripe split needed.
	 *
	 * If the target stripe is marked unmapped, the read returns zeros
	 * directly from the caller's iovs without touching any base bdev.
	 * This is the load-bearing read-as-zero contract that decouples EC
	 * correctness from whether the base bdev chain happens to honour
	 * discard with zero-fill semantics. Without this short-circuit, a
	 * read of a discarded range would return whatever bytes the base
	 * bdev physically holds -- zero only by luck.
	 *
	 * Applies uniformly to all three branches below (parity-only-failed,
	 * degraded, healthy) -- the bitmap is authoritative for the stripe's
	 * content regardless of slot state. The degraded path additionally
	 * benefits: an unmapped stripe needs no ISA-L reconstruction because
	 * RS_encode(0,...,0) = (0,...,0) and the synthesised zeros are the
	 * correct decode result anyway.
	 *
	 * ec_io->iovs is aliased to bdev_io->u.bdev.iovs (see
	 * bdev_ec_io.c:145), so the memset writes directly into the caller's
	 * destination buffers -- no bounce.
	 */
	if (ec->stripe_unmapped_map != NULL) {
		stripe_index = ec_io->offset_blocks / ec->stripe_blocks;
		if (ec_stripe_is_unmapped(ec, stripe_index)) {
			int i;

			for (i = 0; i < ec_io->iovcnt; i++) {
				memset(ec_io->iovs[i].iov_base, 0,
				       ec_io->iovs[i].iov_len);
			}
			__atomic_fetch_add(&ec->unmapped_reads_synthesized, 1, __ATOMIC_RELAXED);
			spdk_bdev_io_complete(ec_io->bdev_io,
					      SPDK_BDEV_IO_STATUS_SUCCESS);
			return 0;
		}
	}

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
 * Drop the WIB region inflight count, release the stripe-busy claim,
 * and free the chunk buffers held by ec_io. Each step is idempotent;
 * safe from any error path. Caller completes the bdev_io afterward.
 */
static void
ec_io_release_state(struct ec_bdev_io *ec_io, struct ec_bdev *ec)
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
 * send_msg target that completes the bdev_io with FAILED on the submitter
 * (= bdev_io owner) thread. Used by the multi-reactor failure path when
 * the WIB persist completes on home but the bdev_io owner is elsewhere
 * -- spdk_bdev_io_complete asserts owner-thread, so the completion must
 * be routed via send_msg.
 */
static void
ec_full_write_complete_failed_on_submitter(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;

	spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
}

/* send_msg trampoline to run ec_full_write_fanout on the submitter thread. */
static void
ec_full_write_dispatch_on_submitter(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;

	ec_full_write_fanout(ec_io);
}

/*
 * Callback fired by ec_wib_persist when the WIB region bit for this
 * full-stripe write has been persisted to disk. On success, continues
 * the submission chain by encoding parity and fanning out child writes.
 * On failure, releases the stripe-busy claim and the WIB inflight count,
 * then completes the bdev_io with FAILED status.
 *
 * Thread routing: this cb fires on the WIB persist completion thread
 * (= ec->home_thread, since ec->wib_chans[] are home-owned). The
 * downstream fan-out dispatches base I/O on the submitter's per-channel
 * base_chans[], so it must run on the submitter thread. Similarly, the
 * failure path's spdk_bdev_io_complete must run on the bdev_io owner
 * thread (= submitter). Both are routed via spdk_thread_send_msg; the
 * inline fast path skips the hop when the caller is already on the
 * target thread.
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
		ec_io_release_state(ec_io, ec);

		if (spdk_likely(spdk_get_thread() == ec_io->submitter_thread)) {
			spdk_bdev_io_complete(ec_io->bdev_io,
					      SPDK_BDEV_IO_STATUS_FAILED);
		} else {
			int send_rc = spdk_thread_send_msg(ec_io->submitter_thread,
				ec_full_write_complete_failed_on_submitter, ec_io);
			if (send_rc != 0) {
				SPDK_ERRLOG("EC bdev %s: cannot hand off failure "
					    "completion to owner thread '%s' (rc=%d %s); "
					    "bdev_io stays in-flight until SPDK "
					    "timeout fires\n",
					    ec->bdev.name,
					    spdk_thread_get_name(ec_io->submitter_thread),
					    send_rc, spdk_strerror(-send_rc));
			}
		}
		return;
	}

	if (spdk_likely(spdk_get_thread() == ec_io->submitter_thread)) {
		ec_full_write_fanout(ec_io);
		return;
	}

	int send_rc = spdk_thread_send_msg(ec_io->submitter_thread,
		ec_full_write_dispatch_on_submitter, ec_io);
	if (send_rc != 0) {
		/*
		 * Persist succeeded but we cannot reach the submitter for the
		 * data fan-out. The on-disk WIB bit is set, so a crash here is
		 * scrub-recoverable; release in-memory EC-layer state and fail
		 * the bdev_io via a second send_msg. If even that fails, log
		 * and accept the hang: the bdev_io stays in flight rather than
		 * being completed on the wrong thread.
		 */
		SPDK_ERRLOG("EC bdev %s: cannot hand off fan-out to submitter "
			    "thread '%s' (rc=%d %s) at stripe %" PRIu64 "; releasing "
			    "EC-layer state\n",
			    ec->bdev.name,
			    spdk_thread_get_name(ec_io->submitter_thread),
			    send_rc, spdk_strerror(-send_rc), ec_io->stripe_claim_index);
		ec_io_release_state(ec_io, ec);

		int complete_rc = spdk_thread_send_msg(ec_io->submitter_thread,
			ec_full_write_complete_failed_on_submitter, ec_io);
		if (complete_rc != 0) {
			SPDK_ERRLOG("EC bdev %s: also cannot hand off failure "
				    "completion to submitter thread '%s' (rc=%d %s); "
				    "bdev_io stays in-flight\n",
				    ec->bdev.name,
				    spdk_thread_get_name(ec_io->submitter_thread),
				    complete_rc, spdk_strerror(-complete_rc));
		}
	}
}

/*
 * Claim the stripe for this write so a concurrent rebuild or UNMAP on the
 * same stripe defers. Returns false if the stripe is already claimed (caller
 * returns -EAGAIN); on success it records the claim on ec_io, which
 * ec_io_release_state later clears.
 *
 * The claim is recorded here -- unlike the WIB held-state, which
 * ec_wib_mark_region leaves to the caller -- because both callers record it
 * the same way and unwind through the same ec_io_release_state gate.
 */
static bool
ec_stripe_try_claim(struct ec_bdev *ec, struct ec_bdev_io *ec_io)
{
	uint64_t stripe_idx = ec_io->offset_blocks / ec->stripe_blocks;

	if (ec_stripe_is_dirty(ec, stripe_idx)) {
		return false;
	}
	ec_stripe_set_dirty(ec, stripe_idx);
	ec_io->stripe_claimed     = true;
	ec_io->stripe_claim_index = stripe_idx;
	return true;
}

/*
 * Carry out the persist/dispatch decision for a full-stripe write whose WIB
 * region is already marked: defer it, start the WIB persist (fan-out then
 * resumes in ec_full_write_wib_set_cb), or fan out now if the region was
 * already durable.
 *
 * Returns 0 once the write is handed off (caller returns 0), or a negative
 * errno on defer/failure (caller jumps to its error unwind). On the
 * was_clean path, an alloc or persist failure clears the region bit this
 * write set before returning.
 *
 * Mirrors ec_rmw_persist_and_dispatch, with two deliberate differences:
 *   - defers with -EAGAIN (which becomes a NOMEM requeue upstream) instead
 *     of queuing on wib_deferred_writes;
 *   - always sets wib_repersist_needed, not only on the was_clean path
 *     (harmless -- at worst one extra persist).
 */
static int
ec_full_write_persist_and_dispatch(struct ec_bdev_io *ec_io, bool was_clean)
{
	struct ec_bdev *ec     = ec_from_bdev_io(ec_io->bdev_io);
	uint32_t        region = ec_io->wib_region;
	int             rc;

	/*
	 * wib_persist_in_flight must be consulted even when the in-memory bit
	 * was already dirty -- an in-flight persist may be *clearing* the bit
	 * (initiated by the idle poller before our setter ran). Proceeding
	 * straight to fan-out in that window would let a crash leave on-disk
	 * WIB clean while the stripe is mid-write.
	 */
	if (ec->wib_persist_in_flight) {
		ec->wib_repersist_needed = true;
		ec->full_stripe_writes_deferred++;
		return -EAGAIN;
	}

	/*
	 * Past every reject (-EINVAL) and defer (-EAGAIN) gate: the write is
	 * now accepted and will fan out. Count it here so a deferred-then-
	 * retried write is not counted on each attempt and an invalid request
	 * is not counted at all.
	 */
	ec->full_stripe_writes++;

	if (was_clean) {
		int persist_rc;

		rc = ec_alloc_full_stripe(ec_io, ec);
		if (rc != 0) {
			/*
			 * We set this region's WIB bit in the mark step, but no
			 * persist will record it and the caller's error path does
			 * not clear it. Roll it back so the in-memory bit never
			 * outlives its on-disk record: otherwise the NOMEM retry
			 * would see the region already dirty, skip the WIB persist,
			 * and fan out data with no recoverable write-intent. Safe --
			 * no fan-out happened, so nothing needs scrubbing.
			 */
			ec_wib_region_clear_dirty(ec, region);
			return rc;
		}

		persist_rc = ec_wib_persist(ec, ec_full_write_wib_set_cb, ec_io);
		if (persist_rc != 0) {
			/* Same rollback as the alloc-failure path above. */
			ec_wib_region_clear_dirty(ec, region);
			return persist_rc;
		}
		return 0;
	}

	/* Region already durable on disk: alloc and fan out directly. */
	rc = ec_alloc_full_stripe(ec_io, ec);
	if (rc != 0) {
		return rc;
	}

	/*
	 * Fan-out routing: dispatches base I/O on ec_io->ch->base_chans[]
	 * (submitter-owned). When the caller is already on the submitter
	 * thread the inline path runs; otherwise we hop to the submitter so
	 * the base I/O dispatch happens on the channel-owning thread.
	 */
	if (spdk_likely(spdk_get_thread() == ec_io->submitter_thread)) {
		ec_full_write_fanout(ec_io);
		return 0;
	}

	rc = spdk_thread_send_msg(ec_io->submitter_thread,
				  ec_full_write_dispatch_on_submitter, ec_io);
	if (rc != 0) {
		return rc;
	}
	return 0;
}

static int
ec_submit_full_write(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	bool            was_clean;
	int             rc = 0;

	/*
	 * Home-thread invariant: the WIB persist below (ec_wib_persist)
	 * uses ec->wib_chans[], which are owned by the home thread. The
	 * routing layer (ec_submit_write's entry hop) ensures every call
	 * chain reaching this function has already been dispatched to home;
	 * the assert is the inner sanity check.
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
	if (ec_scrub_blocks_stripe(ec, ec_io->offset_blocks / ec->stripe_blocks)) {
		ec->full_stripe_writes_deferred++;
		return -EAGAIN;
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
	if (!ec_stripe_try_claim(ec, ec_io)) {
		return -EAGAIN;
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
			    "num_blocks=%" PRIu64 " but stripe_blocks=%" PRIu64 "; "
			    "multi-stripe writes not yet supported\n",
			    ec->bdev.name,
			    ec_io->num_blocks, ec->stripe_blocks);
		rc = -EINVAL;
		goto error;
	}

	/*
	 * WIB region marking. A crash mid-fan-out can leave some of the
	 * k+m chunks new and the rest old; without a WIB bit, recovery has
	 * no record the stripe was mid-write, so no scrub runs and a later
	 * disk failure reconstructs from stale parity -- the write hole.
	 * ec_wib_mark_region marks the region and takes an inflight ref (which
	 * gates the idle poller so the bit can't clear mid-fan-out); the
	 * persist decision follows in ec_full_write_persist_and_dispatch.
	 */
	was_clean = ec_wib_mark_region(ec, ec_io->stripe_claim_index);
	ec_io->wib_region        = ec_wib_stripe_to_region(ec_io->stripe_claim_index);
	ec_io->wib_inflight_held = true;

	rc = ec_full_write_persist_and_dispatch(ec_io, was_clean);
	if (rc != 0) {
		goto error;
	}
	return 0;

error:
	ec_io_release_state(ec_io, ec);
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

	/*
	 * Dispatch invariant: base I/O is submitted on
	 * ec_io->ch->base_chans[], which are owned by the submitter thread
	 * (= the bdev_io's owner thread). Dispatching from any other thread
	 * is undefined behavior (channel ownership). The persist-done ->
	 * submitter hop in ec_full_write_wib_set_cb / ec_submit_full_write
	 * ensures this assertion holds; if a future routing commit drops a
	 * hop, this catches it in debug instead of silently corrupting.
	 */
	assert(spdk_get_thread() == ec_io->submitter_thread);

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
	ec_io_release_state(ec_io, ec);
	/*
	 * Symmetric observability: the write-into-unmapped path bumps
	 * writes_into_unmapped_failed at the other failure sites (alloc
	 * failure, send_msg hand-off failure, post-bit-clear failure in
	 * ec_write_into_unmapped_bit_cleared). The fan-out-all-failed
	 * path went here unobserved before; bump the counter so the
	 * gauge reflects every write-into-unmapped that failed to land.
	 */
	if (ec_io->is_write_into_unmapped) {
		__atomic_fetch_add(&ec->writes_into_unmapped_failed, 1,
				   __ATOMIC_RELAXED);
	}
	spdk_bdev_io_complete(ec_io->bdev_io, SPDK_BDEV_IO_STATUS_FAILED);
}

/*
 * Write-into-unmapped path. Used when ec_submit_write detects that the
 * target stripe's unmapped bit is set.
 *
 * The LEP-described semantics: treat ANY write into an unmapped stripe
 * as a full-stripe write regardless of payload size. The stripe is
 * defined to be all-zero, so the "old data" is known without a read --
 * no RMW read phase, no degraded reconstruction.
 *
 * Crash-safety ordering (load-bearing -- DO NOT REORDER):
 *
 *   1. Claim stripe-busy.                                  (here)
 *   2. Allocate a zero-initialised full-stripe scratch.    (here)
 *   3. Copy caller payload to its sub-stripe offset.       (here)
 *   4. Skip the WIB. The stripe is logically zero, so a    (here)
 *      torn fanout still reads as all-zero to consumers
 *      (bitmap still says unmapped). No torn-RMW window
 *      for the WIB to protect.
 *   5. Encode parity, fan out k+m chunk writes.            (ec_full_write_fanout)
 *   6. After ALL k+m chunk writes complete successfully,   (ec_child_io_complete)
 *      submit a bit-clear via ec_submit_bit_clear_async.
 *   7. After the bit-clear persist acks at m+1, apply      (ec_bit_clear_on_durable)
 *      cleared bit to live stripe_unmapped_map.
 *   8. ONLY THEN ack the bdev_io.                          (ec_write_into_unmapped_bit_cleared)
 *
 * Reordering -- in particular, clearing the bit before fan-out, or
 * acking the bdev_io before the bit-clear persist acks -- opens a
 * silent-corruption window. A crash between "bit cleared on disk"
 * and "data on disk" would leave readers seeing "mapped" while the
 * stripe is inconsistent (parity-mismatch, missing chunks, or both),
 * and ISA-L reconstruction would surface undefined bytes.
 *
 * Crash windows under the correct ordering:
 *
 *   - Crash mid step 5: bitmap says unmapped; chunks partial; reads
 *     synthesise zeros. Caller's write was never acked. Correct.
 *   - Crash post step 5 / pre step 6: bitmap says unmapped; chunks
 *     fully written with consistent parity; reads still synthesise
 *     zeros. The committed data is masked but consistent. Correct.
 *   - Crash mid step 6 / before m+1 ack: bitmap may have new-gen on
 *     fewer than m+1 disks. Max-generation load picks the new-gen if
 *     any disk has it (reads return real data) or the old-gen
 *     otherwise (reads synthesise zeros). Either is consistent.
 *   - Crash post step 7 / pre step 8: bitmap says mapped; data on
 *     disk; caller may retry (didn't see ack) -- safe to repeat because
 *     a future write into the now-MAPPED stripe will go through the
 *     ordinary RMW or full-write path, not back through here.
 *
 * Concurrency: stripe-busy claim is held across the entire sequence
 * (steps 1-8). RMW / UNMAP / scrub / rebuild defer on the same
 * stripe. Reads are NOT serialised against this path -- they consult
 * stripe_unmapped_map at their entry. Until step 7 flips the live
 * bit, concurrent reads synthesise zeros; after step 7 they read
 * real data; the in-between window (post step 7, pre step 8) is OK
 * because data is durably on disk for them to find.
 */
static int
ec_submit_write_into_unmapped(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	uint64_t        stripe_idx = ec_io->offset_blocks / ec->stripe_blocks;
	uint64_t        stripe_offset_bytes;
	uint64_t        payload_bytes;
	bool            saved_is_zero_fill;
	int             rc;

	/*
	 * Home-thread invariant: this function mutates home-only state
	 * (scrub_ctx inspection, stripe-busy claim, is_write_into_unmapped
	 * flag). The routing layer at the top of ec_submit_write ensures we
	 * are on home; the assert is the inner tripwire matching the
	 * equivalent guard at ec_submit_full_write and ec_unmap_inner_fanout.
	 */
	assert(spdk_get_thread() == ec->home_thread);

	/*
	 * Active-scrub guard. Mirrors the guard in ec_submit_full_write:
	 * if the scrubber is at-or-behind this stripe's region, defer.
	 * Without this guard, the scrubber could read stripe chunks, then
	 * we write new chunks + parity, then the scrubber writes parity
	 * derived from the stale read -- leaving new data with stale
	 * parity. The bitmap-still-says-unmapped state masks the
	 * inconsistency from current readers, but a future post-failure
	 * degraded read (after the bit clears) would reconstruct using
	 * stale parity and surface wrong bytes.
	 *
	 * Stripes the scrubber has already passed
	 * (stripe_index < sctx->current_stripe) are safe -- the scrubber
	 * will not revisit them.
	 */
	if (ec_scrub_blocks_stripe(ec, stripe_idx)) {
		return -EAGAIN;
	}

	/* 1. Claim stripe-busy. */
	if (!ec_stripe_try_claim(ec, ec_io)) {
		return -EAGAIN;
	}
	ec_io->is_write_into_unmapped = true;

	/*
	 * 2. Allocate full-stripe scratch zero-initialised. We trick
	 * ec_alloc_full_stripe into skipping the iov copy by temporarily
	 * setting is_zero_fill = true; we then place the caller's payload
	 * at the correct sub-stripe offset ourselves in step 3. The real
	 * is_zero_fill is restored so the rest of the path (encode +
	 * fanout + completion) sees the original value.
	 */
	saved_is_zero_fill   = ec_io->is_zero_fill;
	ec_io->is_zero_fill  = true;
	rc                   = ec_alloc_full_stripe(ec_io, ec);
	ec_io->is_zero_fill  = saved_is_zero_fill;
	if (rc != 0) {
		ec_io->is_write_into_unmapped = false;
		ec_io_release_state(ec_io, ec);
		__atomic_fetch_add(&ec->writes_into_unmapped_failed, 1, __ATOMIC_RELAXED);
		return rc;
	}

	/*
	 * 3. Place caller payload at its sub-stripe offset. For
	 * WRITE_ZEROES into an unmapped stripe the bounce is already zero
	 * everywhere, so the copy is a no-op -- skip it. For a regular
	 * sub-stripe write, copy the iovs starting at
	 * (offset_blocks % stripe_blocks) * blocklen so the payload lands
	 * at its true position within the stripe; the unwritten leading
	 * and trailing regions stay zero.
	 */
	if (!saved_is_zero_fill) {
		stripe_offset_bytes = (ec_io->offset_blocks % ec->stripe_blocks)
				      * ec->bdev.blocklen;
		payload_bytes       = ec_io->num_blocks * ec->bdev.blocklen;
		spdk_copy_iovs_to_buf((uint8_t *)ec_io->bounce_buf + stripe_offset_bytes,
				      payload_bytes,
				      ec_io->iovs, ec_io->iovcnt);
	}

	/*
	 * 4. Skip the WIB (no code). The stripe stays marked unmapped for the
	 * whole fan-out, so a torn write reads back as zeros -- there is no
	 * torn-write window for the WIB to guard (full crash-window analysis in
	 * the function header above).
	 */

	/*
	 * 5. Encode + fan out. ec_full_write_fanout uses
	 * ec_stripe_base_lba(stripe_idx) for the child write offsets, so
	 * passing a sub-stripe offset in ec_io->offset_blocks is harmless
	 * (it gets resolved to the same stripe). On completion,
	 * ec_child_io_complete sees is_write_into_unmapped and routes
	 * through the bit-clear path instead of immediate bdev_io
	 * completion.
	 *
	 * Fan-out routing mirrors ec_submit_full_write: dispatch on
	 * ec_io->ch->base_chans[] (submitter-owned). When the caller is
	 * already on the submitter thread the inline path runs; otherwise
	 * we hop to the submitter so base I/O dispatch happens on the
	 * channel-owning thread.
	 */
	__atomic_fetch_add(&ec->writes_into_unmapped, 1, __ATOMIC_RELAXED);

	if (spdk_likely(spdk_get_thread() == ec_io->submitter_thread)) {
		ec_full_write_fanout(ec_io);
		return 0;
	}

	rc = spdk_thread_send_msg(ec_io->submitter_thread,
				  ec_full_write_dispatch_on_submitter, ec_io);
	if (rc != 0) {
		/*
		 * Cannot reach submitter; roll back the same state the
		 * alloc-failure rollback above does. The bit-clear path
		 * was not engaged (no fan-out happened), so the
		 * stripe_unmapped_map stays in its pre-call state and a
		 * caller retry replays the whole sequence cleanly.
		 */
		ec_io->is_write_into_unmapped = false;
		ec_io_release_state(ec_io, ec);
		__atomic_fetch_add(&ec->writes_into_unmapped_failed, 1,
				   __ATOMIC_RELAXED);
		return rc;
	}
	return 0;
}

/*
 * Shared send_msg target for owner-routed parent-bdev_io completion.
 * Declared in bdev_ec_internal.h so all owner-route hand-offs (write
 * entry-routing failure here, UNMAP entry-routing failure, inner-fanout
 * completion, split-segment completion) share one implementation.
 * Callers stash the final status into ec_io->status before invoking
 * spdk_thread_send_msg.
 */
void
ec_io_complete_status_on_submitter(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;

	spdk_bdev_io_complete(ec_io->bdev_io, ec_io->status);
}

void
ec_io_route_complete_to_submitter(struct ec_bdev_io *ec_io, const char *what)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);
	int             send_rc;

	send_rc = spdk_thread_send_msg(ec_io->submitter_thread,
				       ec_io_complete_status_on_submitter, ec_io);
	if (send_rc != 0) {
		SPDK_ERRLOG("EC bdev %s: cannot hand off %s to submitter "
			    "thread '%s' (rc=%d %s); bdev_io stays in-flight\n",
			    ec->bdev.name, what,
			    spdk_thread_get_name(ec_io->submitter_thread),
			    send_rc, spdk_strerror(-send_rc));
	}
}

static void ec_submit_write_on_home(void *ctx);

int
ec_submit_write(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);

	/*
	 * Entry routing: the write path mutates home-only state
	 * (stripe-busy claim, WIB region map, scrub_ctx counters, persist
	 * orchestration) inside ec_submit_write_into_unmapped /
	 * ec_submit_full_write / ec_submit_rmw_write. Route the entire
	 * call to home if we're not already there. When the caller is
	 * already on the home thread the inline branch runs.
	 *
	 * The bitmap consult below (ec_stripe_is_unmapped) uses
	 * __ATOMIC_ACQUIRE per the release/acquire discipline that pairs
	 * with the home-side mutators, so the outcome of the unmapped
	 * check is reproducible on home after the hop.
	 */
	if (spdk_unlikely(spdk_get_thread() != ec->home_thread)) {
		return spdk_thread_send_msg(ec->home_thread,
					    ec_submit_write_on_home, ec_io);
	}

	/*
	 * Unmapped-bitmap consultation. A write that lands on an unmapped
	 * stripe is treated as a full-stripe write into all-zero "old
	 * data" -- no RMW read, no WIB participation, with a load-bearing
	 * bit-clear-and-persist after data lands. See
	 * ec_submit_write_into_unmapped for the full ordering rationale
	 * and crash-window analysis.
	 *
	 * Without this routing, a write into a trimmed range would go
	 * through ordinary RMW: data lands on base, parity recomputed,
	 * but the bitmap still says unmapped. The read-side bitmap check
	 * (ec_submit_read) would then synthesise zeros over the written
	 * data -- silent data loss.
	 */
	if (ec->stripe_unmapped_map != NULL) {
		uint64_t stripe_idx = ec_io->offset_blocks / ec->stripe_blocks;

		if (ec_stripe_is_unmapped(ec, stripe_idx)) {
			return ec_submit_write_into_unmapped(ec_io);
		}
	}

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

/*
 * Re-enters ec_submit_write on the home thread. The routing check at
 * the top of ec_submit_write fast-paths to the body now that we are
 * on home. On sync failure, route the bdev_io completion back to the
 * submitter with the appropriate status (NOMEM for retryable errors,
 * FAILED for hard errors), mirroring ec_submit_request's status
 * mapping for the inline path. If even the completion hand-off fails,
 * the bdev_io stays in flight rather than being completed on the wrong
 * thread.
 */
static void
ec_submit_write_on_home(void *ctx)
{
	struct ec_bdev_io *ec_io = ctx;
	int                rc;

	rc = ec_submit_write(ec_io);
	if (rc == 0) {
		return;
	}

	ec_io->status = (rc == -EAGAIN || rc == -ENOMEM)
			? SPDK_BDEV_IO_STATUS_NOMEM
			: SPDK_BDEV_IO_STATUS_FAILED;
	ec_io_route_complete_to_submitter(ec_io, "submit failure");
}
