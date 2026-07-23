/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"
#include "spdk_internal/cunit.h"
#include "spdk/env.h"

#include "common/lib/test_env.c"

#include "bdev/ec/bdev_ec_rmw.c"

/*
 * Tests the crash-dirty gate in ec_rmw_teardown: a WIB region is marked
 * crash-dirty only when the RMW fails after a data/parity write was submitted
 * (mctx->writes_issued). Pre-write failures leave parity intact, so marking
 * them would pin a healthy region and send its degraded reads to -EIO until
 * the next scrub.
 *
 * The test calls ec_rmw_teardown directly on the hand-built ec_bdev from
 * ut_reset below -- no async chain or channel mocking needed.
 */

/* ---- SPDK surface bdev_ec_rmw.c links against ---- */
DEFINE_STUB(spdk_get_thread, struct spdk_thread *, (void), NULL);
DEFINE_STUB_V(spdk_bdev_free_io, (struct spdk_bdev_io *bdev_io));
DEFINE_STUB_V(spdk_bdev_io_complete, (struct spdk_bdev_io *bdev_io,
				      enum spdk_bdev_io_status status));
DEFINE_STUB(spdk_bdev_readv_blocks, int, (struct spdk_bdev_desc *desc,
		struct spdk_io_channel *ch, struct iovec *iov, int iovcnt,
		uint64_t offset_blocks, uint64_t num_blocks,
		spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_writev_blocks, int, (struct spdk_bdev_desc *desc,
		struct spdk_io_channel *ch, struct iovec *iov, int iovcnt,
		uint64_t offset_blocks, uint64_t num_blocks,
		spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB(spdk_thread_send_msg, int, (const struct spdk_thread *thread,
		spdk_msg_fn fn, void *ctx), 0);
DEFINE_STUB(spdk_thread_get_name, const char *, (const struct spdk_thread *thread), "ut");

/* ---- Cross-file EC + ISA-L symbols bdev_ec_rmw.c links against ---- */
DEFINE_STUB(ec_wib_persist, int, (struct ec_bdev *ec,
				  void (*cb)(void *cb_arg, int rc), void *cb_arg), 0);
DEFINE_STUB(ec_reconstruct_multi_data, int, (const struct ec_bdev *ec,
		uint8_t *src_bufs[EC_MAX_BASE_BDEVS], uint8_t *out_bufs[],
		const uint32_t failed_data_slots[], uint32_t f, uint64_t chunk_len), 0);
DEFINE_STUB_V(ec_encode_data, (int len, int k, int rows, unsigned char *g_tbls,
			       unsigned char **data, unsigned char **coding));
DEFINE_STUB_V(ec_stripe_waitq_kick, (struct ec_bdev *ec));

/* ---- Fixture ---- */
#define UT_N       6u
#define UT_STRIPE  0ULL
#define UT_REGION  0u    /* stripe 0 -> region 0 (EC_WIB_REGION_STRIPES) */

static struct ec_bdev      g_ec;
static struct spdk_bdev_io g_bdev_io;
static struct ec_bdev_io   g_ec_io;
static uint64_t            g_stripe_dirty[1];
static uint64_t            g_crash_dirty[1];
static uint32_t            g_region_inflight[1];

/*
 * Minimal ec_bdev with only what ec_rmw_teardown touches. The inflight
 * counters start at 1 -- teardown releases them to 0.
 */
static void
ut_reset(void)
{
	memset(&g_ec, 0, sizeof(g_ec));
	memset(&g_bdev_io, 0, sizeof(g_bdev_io));
	memset(&g_ec_io, 0, sizeof(g_ec_io));
	g_stripe_dirty[0]    = ~0ULL;    /* dirty; teardown clears the stripe bit */
	g_crash_dirty[0]     = 0;
	g_region_inflight[0] = 1;

	g_ec.n                   = UT_N;
	g_ec.bdev.name           = "ut_ec";
	g_ec.bdev.ctxt           = &g_ec;
	g_ec.stripe_dirty_map    = g_stripe_dirty;
	g_ec.wib_crash_dirty_map = g_crash_dirty;
	g_ec.wib_region_inflight = g_region_inflight;
	g_ec.rmw_in_flight       = 1;

	g_bdev_io.bdev  = &g_ec.bdev;
	g_ec_io.bdev_io = &g_bdev_io;
}

static struct ec_rmw_ctx *
ut_make_mctx(bool writes_issued, enum spdk_bdev_io_status status)
{
	struct ec_rmw_ctx *mctx = calloc(1, sizeof(*mctx));

	SPDK_CU_ASSERT_FATAL(mctx != NULL);
	mctx->ec_io         = &g_ec_io;
	mctx->stripe_index  = UT_STRIPE;
	mctx->status        = status;
	mctx->writes_issued = writes_issued;
	/* chunk_bufs[] left NULL by calloc -- free_ctx skips them. */
	return mctx;
}

/* A write went out and the RMW still failed -> region marked crash-dirty. */
static void
test_teardown_marks_crash_dirty_after_write(void)
{
	ut_reset();
	ec_rmw_teardown(ut_make_mctx(true, SPDK_BDEV_IO_STATUS_FAILED));

	CU_ASSERT(ec_wib_crash_is_dirty(&g_ec, UT_REGION) == true);
	CU_ASSERT(g_ec.rmw_in_flight == 0);
	CU_ASSERT(g_region_inflight[0] == 0);
}

/* Pre-write failure (no write issued) -> parity intact -> NOT crash-dirty. */
static void
test_teardown_no_mark_prewrite_failed(void)
{
	ut_reset();
	ec_rmw_teardown(ut_make_mctx(false, SPDK_BDEV_IO_STATUS_FAILED));

	CU_ASSERT(ec_wib_crash_is_dirty(&g_ec, UT_REGION) == false);
	CU_ASSERT(g_ec.rmw_in_flight == 0);
	CU_ASSERT(g_region_inflight[0] == 0);
}

/*
 * Retryable NOMEM before any write: SPDK requeues and the retry usually
 * succeeds, so a crash-dirty mark here would falsely flag a stripe that ends
 * up written correctly.
 */
static void
test_teardown_no_mark_prewrite_nomem(void)
{
	ut_reset();
	ec_rmw_teardown(ut_make_mctx(false, SPDK_BDEV_IO_STATUS_NOMEM));

	CU_ASSERT(ec_wib_crash_is_dirty(&g_ec, UT_REGION) == false);
}

/* Success -> NOT crash-dirty, even though a write went out. */
static void
test_teardown_no_mark_on_success(void)
{
	ut_reset();
	ec_rmw_teardown(ut_make_mctx(true, SPDK_BDEV_IO_STATUS_SUCCESS));

	CU_ASSERT(ec_wib_crash_is_dirty(&g_ec, UT_REGION) == false);
}

int
main(int argc, char **argv)
{
	CU_pSuite suite = NULL;
	unsigned int num_failures;

	CU_initialize_registry();

	suite = CU_add_suite("bdev_ec_rmw", NULL, NULL);

	CU_ADD_TEST(suite, test_teardown_marks_crash_dirty_after_write);
	CU_ADD_TEST(suite, test_teardown_no_mark_prewrite_failed);
	CU_ADD_TEST(suite, test_teardown_no_mark_prewrite_nomem);
	CU_ADD_TEST(suite, test_teardown_no_mark_on_success);

	num_failures = spdk_ut_run_tests(argc, argv, NULL);
	CU_cleanup_registry();

	return num_failures;
}
