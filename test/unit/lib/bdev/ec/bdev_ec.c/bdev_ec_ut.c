/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"
#include "spdk_internal/cunit.h"
#include "spdk/env.h"

#include "common/lib/test_env.c"

#include "bdev/ec/bdev_ec.c"
#include "bdev/ec/bdev_ec_bitmap.c"
#include "bdev/ec/bdev_ec_resize.c"
#include "bdev/ec/bdev_ec_rebuild.c"

/*
 * Unit coverage: the in-band unmapped bitmap front reservation.
 *
 * These tests pin the geometry math introduced by the front-reservation
 * commit:
 *   - ec_bitmap_reservation_stripes(): fixed-max sizing from
 *     strip_size + blocklen, independent of disk size.
 *   - ec_compute_geometry(): carves data_offset_stripes off the front,
 *     reports num_stripes / blockcnt for the user region, and hard-fails
 *     a disk too small to hold the reservation.
 *   - ec_stripe_base_lba() / ec_calc_mapping(): every user stripe maps
 *     past the reserved region.
 */

/* ------------------------------------------------------------------ */
/* Stubs for the SPDK surface bdev_ec.c links against.                */
/* ------------------------------------------------------------------ */

static struct spdk_bdev g_base_bdev;

struct spdk_bdev *
spdk_bdev_desc_get_bdev(struct spdk_bdev_desc *desc)
{
	return (struct spdk_bdev *)desc;
}

DEFINE_STUB_V(spdk_bdev_module_list_add, (struct spdk_bdev_module *bdev_module));
DEFINE_STUB(spdk_bdev_get_by_name, struct spdk_bdev *, (const char *bdev_name), NULL);
DEFINE_STUB(spdk_bdev_open_ext, int, (const char *bdev_name, bool write,
				      spdk_bdev_event_cb_t event_cb, void *event_ctx,
				      struct spdk_bdev_desc **desc), 0);
DEFINE_STUB_V(spdk_bdev_close, (struct spdk_bdev_desc *desc));
DEFINE_STUB(spdk_bdev_get_name, const char *, (const struct spdk_bdev *bdev), "base");
DEFINE_STUB(spdk_bdev_register, int, (struct spdk_bdev *bdev), 0);
DEFINE_STUB_V(spdk_bdev_unregister, (struct spdk_bdev *bdev, spdk_bdev_unregister_cb cb_fn,
				     void *cb_arg));
DEFINE_STUB(spdk_bdev_unregister_by_name, int, (const char *bdev_name,
		struct spdk_bdev_module *module, spdk_bdev_unregister_cb cb_fn, void *cb_arg), 0);
DEFINE_STUB_V(spdk_bdev_destruct_done, (struct spdk_bdev *bdev, int bdeverrno));
DEFINE_STUB(spdk_bdev_get_io_channel, struct spdk_io_channel *,
	    (struct spdk_bdev_desc *desc), NULL);
DEFINE_STUB(spdk_bdev_reset, int, (struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
				   spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_quiesce, int, (struct spdk_bdev *bdev, struct spdk_bdev_module *module,
				     spdk_bdev_quiesce_cb cb_fn, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_unquiesce, int, (struct spdk_bdev *bdev, struct spdk_bdev_module *module,
				       spdk_bdev_quiesce_cb cb_fn, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_unquiesce_range, int, (struct spdk_bdev *bdev,
		struct spdk_bdev_module *module, uint64_t offset, uint64_t length,
		spdk_bdev_quiesce_cb cb_fn, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_notify_blockcnt_change, int, (struct spdk_bdev *bdev, uint64_t size), 0);
DEFINE_STUB(spdk_bdev_readv_blocks, int, (struct spdk_bdev_desc *desc,
		struct spdk_io_channel *ch, struct iovec *iov, int iovcnt,
		uint64_t offset_blocks, uint64_t num_blocks,
		spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_writev_blocks, int, (struct spdk_bdev_desc *desc,
		struct spdk_io_channel *ch, struct iovec *iov, int iovcnt,
		uint64_t offset_blocks, uint64_t num_blocks,
		spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB_V(spdk_bdev_free_io, (struct spdk_bdev_io *bdev_io));
DEFINE_STUB_V(spdk_bdev_io_complete, (struct spdk_bdev_io *bdev_io,
				      enum spdk_bdev_io_status status));
DEFINE_STUB(spdk_bdev_wait_for_examine, int, (spdk_bdev_wait_for_examine_cb cb_fn, void *cb_arg),
	    0);

DEFINE_STUB(spdk_get_io_channel, struct spdk_io_channel *, (void *io_device), NULL);
DEFINE_STUB_V(spdk_put_io_channel, (struct spdk_io_channel *ch));
DEFINE_STUB_V(spdk_io_device_register, (void *io_device, spdk_io_channel_create_cb create_cb,
					spdk_io_channel_destroy_cb destroy_cb, uint32_t ctx_size,
					const char *name));
DEFINE_STUB_V(spdk_io_device_unregister, (void *io_device,
		spdk_io_device_unregister_cb unregister_cb));
DEFINE_STUB_V(spdk_for_each_channel, (void *io_device, spdk_channel_msg fn, void *ctx,
				      spdk_channel_for_each_cpl cpl));
DEFINE_STUB_V(spdk_for_each_channel_continue, (struct spdk_io_channel_iter *i, int status));
DEFINE_STUB(spdk_io_channel_iter_get_channel, struct spdk_io_channel *,
	    (struct spdk_io_channel_iter *i), NULL);
DEFINE_STUB(spdk_io_channel_iter_get_ctx, void *, (struct spdk_io_channel_iter *i), NULL);
DEFINE_STUB(spdk_get_thread, struct spdk_thread *, (void), NULL);
DEFINE_STUB(spdk_thread_get_name, const char *, (const struct spdk_thread *thread), "ut");
DEFINE_STUB(spdk_thread_send_msg, int,
	    (const struct spdk_thread *thread, spdk_msg_fn fn, void *ctx), 0);
DEFINE_STUB(spdk_poller_register, struct spdk_poller *, (spdk_poller_fn fn, void *arg,
		uint64_t period_microseconds), NULL);
DEFINE_STUB_V(spdk_poller_unregister, (struct spdk_poller **ppoller));

/* Cross-file EC functions defined in other .c files of the module. */
DEFINE_STUB(ec_wib_idle_poller_cb, int, (void *arg), 0);
DEFINE_STUB_V(ec_wib_load_async, (struct ec_bdev *ec, ec_bdev_create_cb_fn done_fn,
				  void *done_arg));
DEFINE_STUB(ec_wib_count_dirty, uint32_t, (const struct ec_bdev *ec), 0);
DEFINE_STUB_V(ec_bdev_io_init, (struct ec_bdev_io *ec_io, struct ec_io_channel *ch,
				struct spdk_bdev_io *bdev_io));
DEFINE_STUB(ec_submit_read, int, (struct ec_bdev_io *ec_io), 0);
DEFINE_STUB(ec_submit_write, int, (struct ec_bdev_io *ec_io), 0);
DEFINE_STUB(ec_submit_unmap, int, (struct ec_bdev_io *ec_io), 0);

/* Link-only stubs: bdev_ec_rebuild.c references these on paths the skip
 * tests never run, so a no-op definition is enough to link. */
DEFINE_STUB(ec_reconstruct_data_chunk, int, (const struct ec_bdev *ec,
		uint8_t *src_bufs[EC_MAX_BASE_BDEVS], uint8_t *out_buf,
		uint32_t failed_slot, uint64_t chunk_len), 0);
DEFINE_STUB(ec_reconstruct_multi_data, int, (const struct ec_bdev *ec,
		uint8_t *src_bufs[EC_MAX_BASE_BDEVS], uint8_t *out_bufs[],
		const uint32_t failed_data_slots[], uint32_t f, uint64_t chunk_len), 0);
DEFINE_STUB(ec_wib_persist, int, (struct ec_bdev *ec, void (*cb)(void *cb_arg, int rc),
				  void *cb_arg), 0);
DEFINE_STUB_V(ec_rmw_backpressure_end, (struct ec_bdev *ec, const char *reason));
/* ec_encode_data is the real ISA-L symbol (libisal.a is linked into the UT). */

/* ------------------------------------------------------------------ */
/* Test fixtures.                                                     */
/* ------------------------------------------------------------------ */

#define UT_BLOCKLEN 4096u

/*
 * Build a minimal ec_bdev wired just enough for ec_compute_geometry:
 * k+m descriptors all pointing at the same fake base bdev, blocklen set,
 * strip_size_kb set. ec_compute_geometry derives the rest.
 */
static void
ut_init_ec(struct ec_bdev *ec, uint32_t k, uint32_t m, uint32_t strip_size_kb,
	   uint64_t base_blockcnt)
{
	uint32_t i;

	memset(ec, 0, sizeof(*ec));
	memset(&g_base_bdev, 0, sizeof(g_base_bdev));

	ec->k = k;
	ec->m = m;
	ec->n = k + m;
	ec->strip_size_kb = strip_size_kb;
	ec->bdev.name = "ec_ut";
	ec->bdev.blocklen = UT_BLOCKLEN;

	g_base_bdev.blockcnt = base_blockcnt;
	g_base_bdev.max_unmap = 0;
	g_base_bdev.max_unmap_segments = 0;

	for (i = 0; i < ec->n; i++) {
		ec->descs[i] = (struct spdk_bdev_desc *)&g_base_bdev;
	}
}

/*
 * Reference implementation of the fixed-max reservation formula, kept
 * independent of the module so the test catches a drift in either
 * direction.
 *
 * Under the raw-replicated blob model the reservation depends only on
 * strip_size (and blocklen) -- k drops out, because every disk holds an
 * identical copy of the whole blob rather than a 1/k EC-encoded share.
 */
static uint64_t
ut_expected_reservation(uint32_t strip_size_blocks)
{
	uint64_t strip_bytes = (uint64_t)strip_size_blocks * UT_BLOCKLEN;
	uint64_t wib_payload_bytes = strip_bytes - sizeof(struct ec_wib_header)
				     - sizeof(uint32_t);
	/* Word-granular: the region bitmap is written in whole uint64_t words. */
	uint64_t max_regions = (wib_payload_bytes / sizeof(uint64_t)) * 64;
	uint64_t max_num_stripes = max_regions * EC_WIB_REGION_STRIPES;
	uint64_t blob_bytes = sizeof(struct ec_bitmap_header)
			      + EC_BITMAP_WORDS(max_num_stripes) * sizeof(uint64_t)
			      + sizeof(uint32_t);
	uint64_t slot_strips = (blob_bytes + strip_bytes - 1) / strip_bytes;

	return slot_strips * 2;
}

/* ------------------------------------------------------------------ */
/* Tests.                                                             */
/* ------------------------------------------------------------------ */

/*
 * _ec_bdev_create must reject huge k/m values that would wrap k+m in
 * uint32_t and bypass the n>255 / EC_MAX_BASE_BDEVS checks. The downstream
 * ISA-L setup multiplies the unwrapped values for buffer sizing, so a
 * silent wrap turns into an out-of-bounds write in gf_gen_rs_matrix().
 * Individual k,m <=255 keeps every later size computation bounded.
 */
static void
test_create_rejects_overflow_k_m(void)
{
	struct ec_bdev *out = NULL;
	int rc;

	/* k + m wraps to 1 in uint32 -- pre-fix this slipped past n>255. */
	rc = _ec_bdev_create("ec_ut_overflow", 64, 0xFFFFFFFFu, 2, NULL, &out);
	CU_ASSERT(rc == -EINVAL);
	CU_ASSERT(out == NULL);

	/* k > 255 alone, m small. */
	rc = _ec_bdev_create("ec_ut_k256", 64, 256, 2, NULL, &out);
	CU_ASSERT(rc == -EINVAL);

	/* m > 255 alone, k small. */
	rc = _ec_bdev_create("ec_ut_m256", 64, 4, 256, NULL, &out);
	CU_ASSERT(rc == -EINVAL);

	/* Zero k still rejected (regression guard on the existing rule). */
	rc = _ec_bdev_create("ec_ut_k0", 64, 0, 2, NULL, &out);
	CU_ASSERT(rc == -EINVAL);
}

/*
 * ec_compute_geometry on a comfortably-sized disk: the reservation is
 * carved off the front, num_stripes / blockcnt describe only the user
 * region, and the two relate consistently.
 */
static void
test_compute_geometry_basic(void)
{
	struct ec_bdev ec;
	uint64_t strip_size, stripe_blocks, total_physical;
	int rc;

	/* 1 GiB per base disk, 4+2, 64 KiB strips. */
	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);

	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == 0);

	strip_size    = (64u * 1024u) / UT_BLOCKLEN;       /* 16 blocks */
	stripe_blocks = 4u * strip_size;                   /* 64 blocks */
	CU_ASSERT(ec.strip_size == strip_size);
	CU_ASSERT(ec.stripe_blocks == stripe_blocks);

	/*
	 * Both bitmap and WIB are at the front. data_offset_stripes =
	 * bitmap_reservation + 2 (WIB copy 0 + copy 1). No tail reservation.
	 */
	total_physical = g_base_bdev.blockcnt / strip_size;

	CU_ASSERT(ec.data_offset_stripes ==
		  ut_expected_reservation(strip_size) + 2);
	CU_ASSERT(ec.data_offset_stripes > 0);
	CU_ASSERT(ec.num_stripes == total_physical - ec.data_offset_stripes);
	CU_ASSERT(ec.bdev.blockcnt == ec.num_stripes * stripe_blocks);
}

/*
 * ec_compute_geometry rejects a strip larger than INT_MAX bytes -- the
 * limit of the int ec_encode_data() takes -- so the encode-site cast can't
 * truncate the chunk length and corrupt parity.
 */
static void
test_compute_geometry_rejects_oversize_strip(void)
{
	struct ec_bdev ec;
	int rc;

	/*
	 * strip_size_kb = 2^21 -> strip bytes = 2^31, one past INT_MAX. The disk
	 * is sized large so only the new bound can reject this geometry.
	 */
	ut_init_ec(&ec, 4, 2, 1u << 21, (1ull << 40) / UT_BLOCKLEN);

	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == -EINVAL);
}

/*
 * ec_compute_geometry sizes to the smallest open base disk, mirroring the
 * resize path. With heterogeneous blockcnts, total_physical_stripes and
 * num_stripes must derive from the smallest disk -- so no user stripe maps
 * past any one disk's EOF.
 */
static void
test_compute_geometry_min_blockcnt(void)
{
	struct ec_bdev ec;
	struct spdk_bdev bases[6];
	uint64_t big_blockcnt   = (1ull << 30) / UT_BLOCKLEN;
	uint64_t small_blockcnt = big_blockcnt - 256;  /* 1 MiB smaller */
	uint64_t strip_size, expected_total, expected_num_stripes;
	uint32_t i;
	int rc;

	memset(&ec, 0, sizeof(ec));
	memset(bases, 0, sizeof(bases));

	ec.k = 4;
	ec.m = 2;
	ec.n = 6;
	ec.strip_size_kb = 64;
	ec.bdev.name = "ec_ut";
	ec.bdev.blocklen = UT_BLOCKLEN;

	/* Slot 3 is the smallest; everyone else is big_blockcnt. */
	for (i = 0; i < 6; i++) {
		bases[i].blockcnt = (i == 3) ? small_blockcnt : big_blockcnt;
		ec.descs[i] = (struct spdk_bdev_desc *)&bases[i];
	}

	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == 0);

	/* Capacity must derive from the smallest disk, not the first slot. */
	strip_size           = (64u * 1024u) / UT_BLOCKLEN;
	expected_total       = small_blockcnt / strip_size;
	expected_num_stripes = expected_total - ec.data_offset_stripes;
	CU_ASSERT(ec.num_stripes == expected_num_stripes);
	CU_ASSERT(ec.bdev.blockcnt == expected_num_stripes * ec.stripe_blocks);
}

/*
 * The reservation is fixed-max: it depends only on strip_size + blocklen,
 * never on the disk size. A 1 GiB disk and a 64 GiB disk with the
 * same shape must produce the identical data_offset_stripes.
 */
static void
test_reservation_is_fixed_max(void)
{
	struct ec_bdev small, large;
	int rc;

	ut_init_ec(&small, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&small);
	CU_ASSERT(rc == 0);

	ut_init_ec(&large, 4, 2, 64, (64ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&large);
	CU_ASSERT(rc == 0);

	CU_ASSERT(small.data_offset_stripes == large.data_offset_stripes);
	CU_ASSERT(large.num_stripes > small.num_stripes);
}

/*
 * A disk too small to hold the combined front reservation (bitmap + WIB)
 * with at least one user stripe must be rejected, not silently produce a
 * zero- or negative-capacity user region.
 */
static void
test_compute_geometry_hard_floor(void)
{
	struct ec_bdev ec;
	uint64_t strip_size, reservation, just_under;
	int rc;

	strip_size  = (64u * 1024u) / UT_BLOCKLEN;
	reservation = ut_expected_reservation(strip_size);

	/*
	 * physical_stripes = blockcnt / strip_size and
	 * data_offset_stripes = bitmap_reservation + 2 (WIB at front).
	 * Pick blockcnt so physical_stripes equals data_offset_stripes
	 * exactly -- one stripe short of a usable volume.
	 */
	just_under = (reservation + 2) * strip_size;

	ut_init_ec(&ec, 4, 2, 64, just_under);
	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == -EINVAL);
}

/*
 * ec_wib_lba returns front-placed offsets that depend only on
 * strip_size and the bitmap reservation -- not on any disk's blockcnt.
 * Four properties matter:
 *   1. WIB copy 0 sits immediately after the bitmap reservation.
 *   2. WIB copy 1 sits one strip after copy 0.
 *   3. Both LBAs are inside the [0, data_offset_stripes * strip_size)
 *      reserved region -- they must never overlap user data.
 *   4. Re-evaluating against a larger disk produces the SAME LBAs:
 *      resize does not change WIB position on disk.
 */
static void
test_wib_lba_front_placement(void)
{
	struct ec_bdev small, large;
	uint64_t bitmap_strips, expected0, expected1, reserved_lba;
	int rc;

	ut_init_ec(&small, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&small);
	CU_ASSERT(rc == 0);

	bitmap_strips = ec_bitmap_reservation_stripes(&small);
	expected0     = bitmap_strips * small.strip_size;
	expected1     = (bitmap_strips + 1) * small.strip_size;
	reserved_lba  = small.data_offset_stripes * small.strip_size;

	/* Copy 0 sits immediately after the bitmap. */
	CU_ASSERT(ec_wib_lba(&small, 0) == expected0);
	/* Copy 1 sits one strip after copy 0. */
	CU_ASSERT(ec_wib_lba(&small, 1) == expected1);
	/* Both LBAs are inside the reserved front region. */
	CU_ASSERT(ec_wib_lba(&small, 0) < reserved_lba);
	CU_ASSERT(ec_wib_lba(&small, 1) < reserved_lba);
	/* WIB occupies exactly the last 2 strips of the reservation. */
	CU_ASSERT(ec_wib_lba(&small, 1) + small.strip_size == reserved_lba);

	/*
	 * A second volume with the same (k, m, strip_size) but a 64x larger
	 * disk produces identical WIB LBAs. This is the disk-size-invariant
	 * property of ec_wib_lba -- the foundation that lets resize skip
	 * any on-disk WIB relocation. The unit-test scope can't drive
	 * ec_bdev_resize end-to-end (that needs the async quiesce
	 * machinery), so this stands in for the resize-time invariant.
	 */
	ut_init_ec(&large, 4, 2, 64, (64ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&large);
	CU_ASSERT(rc == 0);
	CU_ASSERT(ec_wib_lba(&large, 0) == ec_wib_lba(&small, 0));
	CU_ASSERT(ec_wib_lba(&large, 1) == ec_wib_lba(&small, 1));
}

/*
 * ec_stripe_base_lba and ec_calc_mapping must place every user stripe
 * past the reserved [0, data_offset_stripes) region.
 */
static void
test_stripe_base_lba_offset(void)
{
	struct ec_bdev ec;
	uint64_t reserved_lba, stripe_index, chunk_offset, base_lba;
	uint32_t chunk_idx;
	int rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == 0);

	reserved_lba = ec.data_offset_stripes * ec.strip_size;

	/* User stripe 0 starts exactly at the end of the reserved region. */
	CU_ASSERT(ec_stripe_base_lba(&ec, 0) == reserved_lba);
	CU_ASSERT(ec_stripe_base_lba(&ec, 1) == reserved_lba + ec.strip_size);
	CU_ASSERT(ec_stripe_base_lba(&ec, 100) ==
		  (100 + ec.data_offset_stripes) * ec.strip_size);

	/* ec_calc_mapping for an offset in user stripe 3, chunk 1. */
	ec_calc_mapping(&ec, 3 * ec.stripe_blocks + ec.strip_size + 5,
			&stripe_index, &chunk_idx, &chunk_offset, &base_lba);
	CU_ASSERT(stripe_index == 3);
	CU_ASSERT(chunk_idx == 1);
	CU_ASSERT(chunk_offset == 5);
	CU_ASSERT(base_lba == ec_stripe_base_lba(&ec, 3) + 5);
	CU_ASSERT(base_lba >= reserved_lba);
}

/*
 * Unit coverage: in-band unmapped bitmap pure-logic (whole-blob).
 *
 * These tests pin the raw-replicated blob primitives:
 *   - ec_bitmap_blob_bytes: header + span sizing.
 *   - ec_bitmap_fill_buf: header + span + CRC round-trip.
 *   - ec_bitmap_validate_buf: rejects bad magic, wrong num_stripes,
 *     bad blob_bytes, and corrupted CRC.
 *   - ec_bitmap_apply_buf: restores bits into stripe_unmapped_map.
 */

/*
 * Allocate stripe_unmapped_map directly rather than calling
 * ec_alloc_runtime_arrays (which also requires wib_num_regions and other
 * geometry already set up). The bitmap tests only need the map itself.
 */
static void
ut_alloc_unmapped_map(struct ec_bdev *ec)
{
	uint64_t map_words = EC_BITMAP_WORDS(ec->num_stripes);

	ec->stripe_unmapped_map = calloc(map_words, sizeof(uint64_t));
	SPDK_CU_ASSERT_FATAL(ec->stripe_unmapped_map != NULL);
}

static void
ut_free_unmapped_map(struct ec_bdev *ec)
{
	free(ec->stripe_unmapped_map);
	ec->stripe_unmapped_map = NULL;
}

/*
 * Dirty map used as a no-I/O sink: a mapped stripe marked dirty takes the
 * stripe-busy path instead of submitting reads, so the negative case runs
 * without driving the real rebuild/scrub I/O machinery.
 */
static void
ut_alloc_dirty_map(struct ec_bdev *ec)
{
	uint64_t map_words = EC_BITMAP_WORDS(ec->num_stripes);

	ec->stripe_dirty_map = calloc(map_words, sizeof(uint64_t));
	SPDK_CU_ASSERT_FATAL(ec->stripe_dirty_map != NULL);
}

static void
ut_free_dirty_map(struct ec_bdev *ec)
{
	free(ec->stripe_dirty_map);
	ec->stripe_dirty_map = NULL;
}

/*
 * Sanity-check the blob-bytes formula: must equal header + an integer
 * number of uint64_t words, must cover enough bits for num_stripes, and
 * must monotonically grow with num_stripes.
 */
static void
test_bitmap_blob_bytes(void)
{
	uint64_t b_small = ec_bitmap_blob_bytes(1);
	uint64_t b_large = ec_bitmap_blob_bytes(1u << 20);

	/* Always at least the header. */
	CU_ASSERT(b_small >= sizeof(struct ec_bitmap_header));

	/* Whole uint64_t words past the header. */
	CU_ASSERT(((b_small - sizeof(struct ec_bitmap_header)) % sizeof(uint64_t)) == 0);
	CU_ASSERT(((b_large - sizeof(struct ec_bitmap_header)) % sizeof(uint64_t)) == 0);

	/* Monotonically grows with stripe count. */
	CU_ASSERT(b_large > b_small);

	/* The span covers at least one bit per stripe. */
	CU_ASSERT((b_small - sizeof(struct ec_bitmap_header)) * 8 >= 1);
	CU_ASSERT((b_large - sizeof(struct ec_bitmap_header)) * 8 >= (1u << 20));
}

/*
 * Allocate a buffer sized for the full on-disk slot extent
 * (blob_bytes + CRC trailer). Zero-initialised, as fill_buf requires.
 */
static uint8_t *
ut_alloc_slot_buf(const struct ec_bdev *ec)
{
	uint64_t slot_extent = ec_bitmap_blob_bytes(ec->num_stripes)
			       + sizeof(uint32_t);
	uint8_t *buf         = calloc(1, slot_extent);

	SPDK_CU_ASSERT_FATAL(buf != NULL);
	return buf;
}

/*
 * Fill a blob and validate it back: magic, version, num_stripes,
 * blob_bytes, CRC, and generation must all survive the round-trip.
 */
static void
test_bitmap_fill_validate(void)
{
	struct ec_bdev ec;
	uint8_t       *buf;
	uint64_t       gen;
	int            rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == 0);

	ut_alloc_unmapped_map(&ec);

	ec_stripe_set_unmapped(&ec, 0);
	ec_stripe_set_unmapped(&ec, 7);
	ec_stripe_set_unmapped(&ec, 63);

	buf = ut_alloc_slot_buf(&ec);

	ec_bitmap_fill_buf(&ec, ec.stripe_unmapped_map, 42 /* generation */, buf);

	gen = 0;
	rc  = ec_bitmap_validate_buf(&ec, buf, &gen);
	CU_ASSERT(rc == 0);
	CU_ASSERT(gen == 42);

	free(buf);
	ut_free_unmapped_map(&ec);
}

/*
 * validate_buf must reject: bad magic, num_stripes mismatch (a slot
 * persisted at a different geometry), and a flipped byte in the span
 * (CRC mismatch while header fields remain otherwise valid).
 */
static void
test_bitmap_validate_failures(void)
{
	struct ec_bdev ec;
	uint8_t       *buf;
	uint64_t       gen;
	int            rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == 0);

	ut_alloc_unmapped_map(&ec);
	buf = ut_alloc_slot_buf(&ec);

	/* Bad magic: zero out the magic field. */
	ec_bitmap_fill_buf(&ec, ec.stripe_unmapped_map, 1, buf);
	memset(buf, 0, sizeof(uint64_t)); /* magic is the first field */
	rc = ec_bitmap_validate_buf(&ec, buf, &gen);
	CU_ASSERT(rc == -EINVAL);

	/*
	 * num_stripes larger than the volume: fill with the current geometry,
	 * then bump the header's num_stripes field and recompute the CRC. The
	 * CRC is now self-consistent, but validate must still reject because a
	 * copy claiming more stripes than the volume can only be foreign or
	 * corrupt (the volume never shrinks).
	 */
	ec_bitmap_fill_buf(&ec, ec.stripe_unmapped_map, 1, buf);
	{
		struct ec_bitmap_header *hdr = (struct ec_bitmap_header *)buf;
		uint64_t                 blob_bytes = hdr->blob_bytes;
		uint32_t                *crc_ptr    = (uint32_t *)(buf + blob_bytes);

		hdr->num_stripes++;
		*crc_ptr = spdk_crc32c_update(buf, (uint32_t)blob_bytes, 0);
	}
	rc = ec_bitmap_validate_buf(&ec, buf, &gen);
	CU_ASSERT(rc == -EINVAL);

	/* CRC mismatch: corrupt a byte in the span (past the header). */
	ec_bitmap_fill_buf(&ec, ec.stripe_unmapped_map, 1, buf);
	buf[sizeof(struct ec_bitmap_header)] ^= 0xFF;
	rc = ec_bitmap_validate_buf(&ec, buf, &gen);
	CU_ASSERT(rc == -EINVAL);

	free(buf);
	ut_free_unmapped_map(&ec);
}

/*
 * Fill a commit record and validate it back: the committed generation and
 * blob CRC survive the round-trip.
 */
static void
test_commit_fill_validate(void)
{
	uint8_t *buf;
	uint64_t gen;
	uint32_t blob_crc;
	int      rc;

	buf = calloc(1, sizeof(struct ec_bitmap_commit) + sizeof(uint32_t));
	SPDK_CU_ASSERT_FATAL(buf != NULL);

	ec_bitmap_commit_fill_buf(99 /* gen */, 0xDEADBEEF /* blob_crc */, buf);

	gen      = 0;
	blob_crc = 0;
	rc = ec_bitmap_commit_validate_buf(buf, &gen, &blob_crc);
	CU_ASSERT(rc == 0);
	CU_ASSERT(gen == 99);
	CU_ASSERT(blob_crc == 0xDEADBEEF);

	free(buf);
}

/*
 * commit_validate_buf must reject bad magic, a wrong version, and a flipped
 * byte (CRC mismatch).
 */
static void
test_commit_validate_failures(void)
{
	uint8_t *buf;
	uint64_t gen;
	uint32_t blob_crc;
	int      rc;

	buf = calloc(1, sizeof(struct ec_bitmap_commit) + sizeof(uint32_t));
	SPDK_CU_ASSERT_FATAL(buf != NULL);

	/* Bad magic: zero out the first field. */
	ec_bitmap_commit_fill_buf(1, 0, buf);
	memset(buf, 0, sizeof(uint64_t)); /* magic is the first field */
	rc = ec_bitmap_commit_validate_buf(buf, &gen, &blob_crc);
	CU_ASSERT(rc == -EINVAL);

	/*
	 * Wrong version: bump the version field and recompute the CRC so the
	 * record is self-consistent. validate must still reject a version this
	 * build does not understand.
	 */
	ec_bitmap_commit_fill_buf(1, 0, buf);
	{
		struct ec_bitmap_commit *rec     = (struct ec_bitmap_commit *)buf;
		uint32_t                *crc_ptr = (uint32_t *)(rec + 1);

		rec->version++;
		*crc_ptr = spdk_crc32c_update(buf, sizeof(*rec), 0);
	}
	rc = ec_bitmap_commit_validate_buf(buf, &gen, &blob_crc);
	CU_ASSERT(rc == -EINVAL);

	/* CRC mismatch: corrupt a field without recomputing the trailer. */
	ec_bitmap_commit_fill_buf(1, 0, buf);
	buf[sizeof(uint64_t)] ^= 0xFF; /* flip a byte in committed_gen */
	rc = ec_bitmap_commit_validate_buf(buf, &gen, &blob_crc);
	CU_ASSERT(rc == -EINVAL);

	free(buf);
}

/*
 * Fill a blob, wipe stripe_unmapped_map, apply the blob, and verify
 * that the previously-set bits are restored and no spurious bits appear.
 */
static void
test_bitmap_apply(void)
{
	struct ec_bdev ec;
	uint8_t       *buf;
	uint64_t       map_words;
	int            rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	CU_ASSERT(rc == 0);

	ut_alloc_unmapped_map(&ec);

	ec_stripe_set_unmapped(&ec, 0);
	ec_stripe_set_unmapped(&ec, 5);
	ec_stripe_set_unmapped(&ec, 63);

	buf = ut_alloc_slot_buf(&ec);

	ec_bitmap_fill_buf(&ec, ec.stripe_unmapped_map, 1, buf);

	/* Wipe the in-memory map; apply must restore it. */
	map_words = EC_BITMAP_WORDS(ec.num_stripes);
	memset(ec.stripe_unmapped_map, 0, map_words * sizeof(uint64_t));

	ec_bitmap_apply_buf(&ec, buf);

	CU_ASSERT(ec_stripe_is_unmapped(&ec, 0));
	CU_ASSERT(ec_stripe_is_unmapped(&ec, 5));
	CU_ASSERT(ec_stripe_is_unmapped(&ec, 63));
	CU_ASSERT(!ec_stripe_is_unmapped(&ec, 1));
	CU_ASSERT(!ec_stripe_is_unmapped(&ec, 64));

	free(buf);
	ut_free_unmapped_map(&ec);
}

/*
 * Copy B's LBA must not move when the volume grows. The slot LBA comes
 * from the fixed-max reservation, not the live num_stripes, so a copy
 * written before a resize is still found at the same LBA afterward. This
 * is the bitmap-slot analog of test_wib_lba_front_placement, and the
 * invariant a resize-then-crash depends on.
 */
static void
test_bitmap_slot_lba_invariance(void)
{
	struct ec_bdev small, large;
	int rc;

	ut_init_ec(&small, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&small);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	ut_init_ec(&large, 4, 2, 64, (64ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&large);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	/* The larger volume really does have more stripes. */
	CU_ASSERT(large.num_stripes > small.num_stripes);

	/* Copy A is always at LBA 0. */
	CU_ASSERT(ec_bitmap_slot_lba_blocks(&small, 0) == 0);
	CU_ASSERT(ec_bitmap_slot_lba_blocks(&large, 0) == 0);

	/* Copy B sits at the same LBA on both -- placement is independent of
	 * num_stripes. */
	CU_ASSERT(ec_bitmap_slot_lba_blocks(&small, 1) ==
		  ec_bitmap_slot_lba_blocks(&large, 1));

	/* The reserved footprint is identical, and the live I/O size never
	 * exceeds it on either volume. */
	CU_ASSERT(ec_bitmap_slot_reserved_blocks(&small) ==
		  ec_bitmap_slot_reserved_blocks(&large));
	CU_ASSERT(ec_bitmap_slot_io_blocks(&small) <=
		  ec_bitmap_slot_reserved_blocks(&small));
	CU_ASSERT(ec_bitmap_slot_io_blocks(&large) <=
		  ec_bitmap_slot_reserved_blocks(&large));
}

/*
 * A copy written before a resize (smaller num_stripes) must still load
 * after the volume grows: validate accepts it as a valid prefix, and
 * apply zero-extends the grown tail to MAPPED. A copy larger than the
 * current volume is rejected -- the volume never shrinks.
 */
static void
test_bitmap_validate_accepts_smaller(void)
{
	struct ec_bdev small, large;
	uint8_t       *buf;
	uint64_t       gen;
	uint64_t       small_stripes;
	int            rc;

	ut_init_ec(&small, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&small);
	SPDK_CU_ASSERT_FATAL(rc == 0);
	ut_alloc_unmapped_map(&small);

	ut_init_ec(&large, 4, 2, 64, (64ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&large);
	SPDK_CU_ASSERT_FATAL(rc == 0);
	ut_alloc_unmapped_map(&large);

	SPDK_CU_ASSERT_FATAL(large.num_stripes > small.num_stripes);
	small_stripes = small.num_stripes;

	/* Persist a few unmapped bits at the small (pre-resize) geometry. */
	ec_stripe_set_unmapped(&small, 0);
	ec_stripe_set_unmapped(&small, 9);
	ec_stripe_set_unmapped(&small, small_stripes - 1);

	buf = ut_alloc_slot_buf(&small);
	ec_bitmap_fill_buf(&small, small.stripe_unmapped_map, 7, buf);

	/* The small copy validates against the grown volume. */
	gen = 0;
	rc  = ec_bitmap_validate_buf(&large, buf, &gen);
	CU_ASSERT(rc == 0);
	CU_ASSERT(gen == 7);

	/* Apply onto the grown map: small bits restored, grown tail MAPPED. */
	ec_bitmap_apply_buf(&large, buf);
	CU_ASSERT(ec_stripe_is_unmapped(&large, 0));
	CU_ASSERT(ec_stripe_is_unmapped(&large, 9));
	CU_ASSERT(ec_stripe_is_unmapped(&large, small_stripes - 1));
	CU_ASSERT(!ec_stripe_is_unmapped(&large, large.num_stripes - 1));

	free(buf);

	/* The reverse must be rejected: a larger copy on a smaller volume. */
	buf = ut_alloc_slot_buf(&large);
	ec_bitmap_fill_buf(&large, large.stripe_unmapped_map, 1, buf);
	rc = ec_bitmap_validate_buf(&small, buf, &gen);
	CU_ASSERT(rc == -EINVAL);
	free(buf);

	ut_free_unmapped_map(&small);
	ut_free_unmapped_map(&large);
}

/*
 * Unit coverage: resize resets the WIB instead of carrying it.
 *
 * ec_resize_realloc_wib_arrays must NOT carry the old dirty bits
 * forward on a grow. The on-disk WIB stays in the old geometry (the
 * next restart rejects it on the num_regions check and loads
 * all-clean), and the write path skips the WIB persist for an
 * already-dirty region -- so a carried-forward bit would let a
 * post-resize write issue with no recoverable write-intent, a
 * write-hole if it tears. This pins the reset-to-clean behavior.
 */

static void
test_resize_wib_reset_on_grow(void)
{
	struct ec_bdev       ec;
	struct ec_resize_ctx ctx;
	uint32_t             old_regions, new_regions;
	uint64_t             old_blockcnt;
	int                  rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);
	rc = ec_alloc_runtime_arrays(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	old_regions  = ec.wib_num_regions;
	old_blockcnt = ec.bdev.blockcnt;
	SPDK_CU_ASSERT_FATAL(old_regions >= 1);

	/* A write touched region 0 before the resize -- mark it dirty. */
	ec_wib_region_set_dirty(&ec, 0);
	CU_ASSERT(ec_wib_region_is_dirty(&ec, 0));

	/*
	 * ec_resize_quiesce_cb updates num_stripes first, then reallocates the
	 * WIB arrays. Grow by two whole WIB regions so the region count changes.
	 */
	ec.num_stripes += (uint64_t)EC_WIB_REGION_STRIPES * 2;
	new_regions = (uint32_t)((ec.num_stripes + EC_WIB_REGION_STRIPES - 1) /
				 EC_WIB_REGION_STRIPES);
	SPDK_CU_ASSERT_FATAL(new_regions > old_regions);

	memset(&ctx, 0, sizeof(ctx));
	ctx.ec           = &ec;
	ctx.old_blockcnt = old_blockcnt;

	ec_resize_realloc_wib_arrays(&ctx);

	/* Geometry grew, and the previously-dirty region now reads clean. */
	CU_ASSERT(ec.wib_num_regions == new_regions);
	CU_ASSERT(!ec_wib_region_is_dirty(&ec, 0));

	ec_free_runtime_arrays(&ec);
}

/*
 * Unit coverage: deferred dedicated-channel teardown.
 *
 * A persist write to a failing disk can sit outstanding on a WIB /
 * bitmap channel for the full NVMe-oF ctrlr-loss timeout, so a slot
 * failure can race an in-flight persist. Releasing the channel then
 * trips the bdev-layer io_outstanding assert. These tests pin the
 * release helper and the in-flight gate that defers it.
 */

static void
test_dedicated_channel_release(void)
{
	struct ec_bdev          ec;
	struct spdk_io_channel *fake = (struct spdk_io_channel *)0x1;
	int                     rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	/* Parity slot 5 (>= k=4): owns WIB channel parity_idx 1 and a bitmap
	 * channel. Both must be released. */
	ec.wib_chans[5 - 4] = fake;
	ec.bitmap_chans[5]  = fake;
	ec_release_slot_dedicated_channels(&ec, 5);
	CU_ASSERT(ec.wib_chans[1] == NULL);
	CU_ASSERT(ec.bitmap_chans[5] == NULL);

	/* Data slot 2 (< k): bitmap channel only; it must not touch any WIB
	 * channel (those belong to parity slots). */
	ec.bitmap_chans[2] = fake;
	ec.wib_chans[0]    = fake;
	ec_release_slot_dedicated_channels(&ec, 2);
	CU_ASSERT(ec.bitmap_chans[2] == NULL);
	CU_ASSERT(ec.wib_chans[0] == fake);
}

static void
test_dedicated_release_deferred_gate(void)
{
	struct ec_bdev          ec;
	struct spdk_io_channel *fake = (struct spdk_io_channel *)0x1;
	int                     rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	ec.wib_chans[5 - 4] = fake;
	ec.bitmap_chans[5]  = fake;
	ec.dedicated_release_pending[5] = true;

	/* Bitmap persist in flight: drain must not release the channels. */
	ec.bitmap_persist_in_flight = true;
	ec_drain_deferred_slot_releases(&ec);
	CU_ASSERT(ec.bitmap_chans[5] == fake);
	CU_ASSERT(ec.wib_chans[1] == fake);
	CU_ASSERT(ec.dedicated_release_pending[5] == true);

	/* WIB persist still in flight: still gated. */
	ec.bitmap_persist_in_flight = false;
	ec.wib_persist_in_flight    = true;
	ec_drain_deferred_slot_releases(&ec);
	CU_ASSERT(ec.bitmap_chans[5] == fake);
	CU_ASSERT(ec.dedicated_release_pending[5] == true);

	/* Both idle: the deferred release fires. */
	ec.wib_persist_in_flight = false;
	ec_drain_deferred_slot_releases(&ec);
	CU_ASSERT(ec.bitmap_chans[5] == NULL);
	CU_ASSERT(ec.wib_chans[1] == NULL);
	CU_ASSERT(ec.dedicated_release_pending[5] == false);
}

/*
 * When a slot release and a delete are both pending, the per-slot path must
 * stand down -- the delete teardown releases every channel itself, and a
 * per-slot quiesce against a bdev whose unregister is about to free ec would
 * fire the quiesce callback on freed memory. Pin that the slot drain is a
 * no-op while unregister is pending, even with persists idle.
 */
static void
test_dedicated_release_unregister_precedence(void)
{
	struct ec_bdev          ec;
	struct spdk_io_channel *fake = (struct spdk_io_channel *)0x1;
	int                     rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	ec.bitmap_chans[5]  = fake;
	ec.wib_chans[5 - 4] = fake;
	ec.dedicated_release_pending[5] = true;
	ec.unregister_release_pending   = true;

	/* Persists idle, but a delete is pending: the slot path must not run
	 * (otherwise it would quiesce a bdev the unregister tail is about to
	 * free). The unregister tail will release these channels instead. */
	ec_drain_deferred_slot_releases(&ec);
	CU_ASSERT(ec.bitmap_chans[5] == fake);
	CU_ASSERT(ec.wib_chans[1] == fake);
	CU_ASSERT(ec.dedicated_release_pending[5] == true);
}

/*
 * The bitmap slot I/O extent must never exceed one reserved slot, or a persist
 * would write past it into the WIB strips and user data. At the ceiling
 * (num_stripes == ec_max_num_stripes) the invariant must still hold -- this is
 * what ec_compute_geometry / ec_bdev_resize keep true and the assert in
 * ec_bitmap_slot_io_blocks backstops.
 */
static void
test_bitmap_slot_io_within_reserved(void)
{
	struct ec_bdev ec;
	int            rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	CU_ASSERT(ec_max_num_stripes(&ec) > 0);

	/* At the live geometry. */
	CU_ASSERT(ec_bitmap_slot_io_blocks(&ec) <=
		  ec_bitmap_slot_reserved_blocks(&ec));

	/* At the ceiling -- the boundary the reservation is sized for. */
	ec.num_stripes = ec_max_num_stripes(&ec);
	CU_ASSERT(ec_bitmap_slot_io_blocks(&ec) <=
		  ec_bitmap_slot_reserved_blocks(&ec));
}

/*
 * The serialized WIB (header + region words + CRC) must fit one strip at the
 * ceiling, and one region past it must not. Pins that ec_max_num_stripes is
 * the exact word-granular bound ec_wib_fill_buf relies on -- a bit-granular
 * bound would admit a geometry whose extra word overruns wib_buf. (ut does
 * not link bdev_ec_wib.c, so the check mirrors ec_wib_fill_buf's layout math.)
 */
static void
test_wib_fits_strip_at_ceiling(void)
{
	struct ec_bdev ec;
	uint64_t       strip_bytes, max_stripes, regions, wib_bytes;
	int            rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);

	strip_bytes = (uint64_t)ec.strip_size * ec.bdev.blocklen;
	max_stripes = ec_max_num_stripes(&ec);

	regions   = (max_stripes + EC_WIB_REGION_STRIPES - 1) / EC_WIB_REGION_STRIPES;
	wib_bytes = sizeof(struct ec_wib_header)
		    + (uint64_t)EC_BITMAP_WORDS(regions) * sizeof(uint64_t)
		    + sizeof(uint32_t);
	CU_ASSERT(wib_bytes <= strip_bytes);

	/* One region past the ceiling overflows -- the bound is tight. */
	regions  += 1;
	wib_bytes = sizeof(struct ec_wib_header)
		    + (uint64_t)EC_BITMAP_WORDS(regions) * sizeof(uint64_t)
		    + sizeof(uint32_t);
	CU_ASSERT(wib_bytes > strip_bytes);
}

/*
 * Unit coverage: scrub/rebuild skip of unmapped stripes.
 *
 * Each poller skips an unmapped stripe: counts it processed, advances the
 * cursor, and submits no I/O. The mapped-but-dirty case checks the skip is
 * conditional on the unmapped bit, not taken for every stripe.
 */

/*
 * Rebuild poller. The unmapped stripe is counted as rebuilt and skipped
 * with no I/O. The mapped, dirty stripe is NOT counted -- it falls into
 * the stripe-busy interlock and is parked on the deferred queue.
 */
static void
test_rebuild_skips_unmapped_stripe(void)
{
	struct ec_bdev        ec;
	struct ec_rebuild_ctx ctx;
	struct ec_rebuild_deferred_stripe *deferred;
	const uint64_t        unmapped_stripe = 5;
	const uint64_t        mapped_stripe   = 10;
	int                   rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);
	SPDK_CU_ASSERT_FATAL(ec.num_stripes > mapped_stripe);

	ut_alloc_unmapped_map(&ec);
	ut_alloc_dirty_map(&ec);
	ec_stripe_set_unmapped(&ec, unmapped_stripe);

	memset(&ctx, 0, sizeof(ctx));
	ctx.ec             = &ec;
	ctx.num_stripes    = ec.num_stripes;
	ctx.current_stripe = unmapped_stripe;
	TAILQ_INIT(&ctx.deferred_stripes);

	/* Unmapped stripe: counted as rebuilt, advanced, no I/O, not parked. */
	rc = ec_rebuild_poller_cb(&ctx);
	CU_ASSERT(rc == SPDK_POLLER_BUSY);
	CU_ASSERT(ctx.stripes_rebuilt == 1);
	CU_ASSERT(ctx.current_stripe == unmapped_stripe + 1);
	CU_ASSERT(ctx.io_in_flight == false);
	CU_ASSERT(TAILQ_EMPTY(&ctx.deferred_stripes));

	/* Mapped, dirty stripe: skip NOT taken; parked by the busy interlock. */
	ctx.current_stripe  = mapped_stripe;
	ctx.stripes_rebuilt = 0;
	ec_stripe_set_dirty(&ec, mapped_stripe);

	rc = ec_rebuild_poller_cb(&ctx);
	CU_ASSERT(rc == SPDK_POLLER_BUSY);
	CU_ASSERT(ctx.stripes_rebuilt == 0);
	CU_ASSERT(ctx.current_stripe == mapped_stripe + 1);
	CU_ASSERT(ctx.io_in_flight == false);
	CU_ASSERT(!TAILQ_EMPTY(&ctx.deferred_stripes));

	/* Drain the parked entry so valgrind stays clean. */
	deferred = TAILQ_FIRST(&ctx.deferred_stripes);
	SPDK_CU_ASSERT_FATAL(deferred != NULL);
	CU_ASSERT(deferred->stripe_index == mapped_stripe);
	TAILQ_REMOVE(&ctx.deferred_stripes, deferred, link);
	free(deferred);

	ut_free_dirty_map(&ec);
	ut_free_unmapped_map(&ec);
}

/*
 * Scrub poller. The unmapped stripe is counted as scrubbed and skipped
 * with no I/O. The mapped, dirty stripe is NOT counted -- it advances via
 * the stripe-busy interlock. region_end_stripe is held above the cursor so
 * the poller never takes the region-complete branch.
 */
static void
test_scrub_skips_unmapped_stripe(void)
{
	struct ec_bdev      ec;
	struct ec_scrub_ctx sctx;
	const uint64_t      unmapped_stripe = 5;
	const uint64_t      mapped_stripe   = 10;
	int                 rc;

	ut_init_ec(&ec, 4, 2, 64, (1ull << 30) / UT_BLOCKLEN);
	rc = ec_compute_geometry(&ec);
	SPDK_CU_ASSERT_FATAL(rc == 0);
	SPDK_CU_ASSERT_FATAL(ec.num_stripes > mapped_stripe);

	ut_alloc_unmapped_map(&ec);
	ut_alloc_dirty_map(&ec);
	ec_stripe_set_unmapped(&ec, unmapped_stripe);

	memset(&sctx, 0, sizeof(sctx));
	sctx.ec                = &ec;
	sctx.current_stripe    = unmapped_stripe;
	sctx.region_end_stripe = ec.num_stripes;

	/* Unmapped stripe: counted as scrubbed, advanced, no I/O. */
	rc = ec_scrub_poller_cb(&sctx);
	CU_ASSERT(rc == SPDK_POLLER_BUSY);
	CU_ASSERT(sctx.stripes_scrubbed == 1);
	CU_ASSERT(sctx.current_stripe == unmapped_stripe + 1);
	CU_ASSERT(sctx.io_in_flight == false);

	/* Mapped, dirty stripe: skip NOT taken; advanced by the busy interlock. */
	sctx.current_stripe   = mapped_stripe;
	sctx.stripes_scrubbed = 0;
	ec_stripe_set_dirty(&ec, mapped_stripe);

	rc = ec_scrub_poller_cb(&sctx);
	CU_ASSERT(rc == SPDK_POLLER_BUSY);
	CU_ASSERT(sctx.stripes_scrubbed == 0);
	CU_ASSERT(sctx.current_stripe == mapped_stripe + 1);
	CU_ASSERT(sctx.io_in_flight == false);

	ut_free_dirty_map(&ec);
	ut_free_unmapped_map(&ec);
}

int
main(int argc, char **argv)
{
	CU_pSuite suite = NULL;
	unsigned int num_failures;

	CU_initialize_registry();

	suite = CU_add_suite("bdev_ec", NULL, NULL);

	CU_ADD_TEST(suite, test_create_rejects_overflow_k_m);
	CU_ADD_TEST(suite, test_compute_geometry_basic);
	CU_ADD_TEST(suite, test_compute_geometry_rejects_oversize_strip);
	CU_ADD_TEST(suite, test_compute_geometry_min_blockcnt);
	CU_ADD_TEST(suite, test_reservation_is_fixed_max);
	CU_ADD_TEST(suite, test_compute_geometry_hard_floor);
	CU_ADD_TEST(suite, test_wib_lba_front_placement);
	CU_ADD_TEST(suite, test_stripe_base_lba_offset);

	CU_ADD_TEST(suite, test_bitmap_blob_bytes);
	CU_ADD_TEST(suite, test_bitmap_fill_validate);
	CU_ADD_TEST(suite, test_bitmap_validate_failures);
	CU_ADD_TEST(suite, test_commit_fill_validate);
	CU_ADD_TEST(suite, test_commit_validate_failures);
	CU_ADD_TEST(suite, test_bitmap_apply);
	CU_ADD_TEST(suite, test_bitmap_slot_lba_invariance);
	CU_ADD_TEST(suite, test_bitmap_validate_accepts_smaller);

	CU_ADD_TEST(suite, test_resize_wib_reset_on_grow);

	CU_ADD_TEST(suite, test_dedicated_channel_release);
	CU_ADD_TEST(suite, test_dedicated_release_deferred_gate);
	CU_ADD_TEST(suite, test_dedicated_release_unregister_precedence);
	CU_ADD_TEST(suite, test_bitmap_slot_io_within_reserved);
	CU_ADD_TEST(suite, test_wib_fits_strip_at_ceiling);

	CU_ADD_TEST(suite, test_rebuild_skips_unmapped_stripe);
	CU_ADD_TEST(suite, test_scrub_skips_unmapped_stripe);

	num_failures = spdk_ut_run_tests(argc, argv, NULL);
	CU_cleanup_registry();

	return num_failures;
}
