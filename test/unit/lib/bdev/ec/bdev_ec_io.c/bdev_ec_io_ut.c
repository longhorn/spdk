/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"
#include "spdk_internal/cunit.h"
#include "spdk/env.h"

#include "common/lib/test_env.c"

#include "bdev/ec/bdev_ec_io.c"

/*
 * Tests the stripe-conflict wait queue: writes park when their stripe is
 * busy or earlier writes for it are parked, drain in FIFO order per stripe
 * when the stripe is released, and fail as a group on reset/destruct.
 *
 * spdk_get_thread and home_thread are both NULL, so every routing check
 * takes the inline home-thread path. ec_submit_rmw_write is stubbed and
 * marks the stripe dirty like the real claim, so a drained write blocks
 * later parked writes for the same stripe.
 */

/* ---- SPDK surface bdev_ec_io.c links against ---- */
DEFINE_STUB(spdk_get_thread, struct spdk_thread *, (void), NULL);
DEFINE_STUB_V(spdk_bdev_free_io, (struct spdk_bdev_io *bdev_io));
DEFINE_STUB_V(spdk_bdev_io_complete, (struct spdk_bdev_io *bdev_io,
				      enum spdk_bdev_io_status status));
DEFINE_STUB(spdk_bdev_writev_blocks, int, (struct spdk_bdev_desc *desc,
		struct spdk_io_channel *ch, struct iovec *iov, int iovcnt,
		uint64_t offset_blocks, uint64_t num_blocks,
		spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB(spdk_thread_send_msg, int, (const struct spdk_thread *thread,
		spdk_msg_fn fn, void *ctx), 0);
DEFINE_STUB(spdk_thread_get_name, const char *, (const struct spdk_thread *thread), "ut");
DEFINE_STUB(spdk_bdev_get_name, const char *, (const struct spdk_bdev *bdev), "ut_base");
DEFINE_STUB(spdk_bdev_io_get_thread, struct spdk_thread *, (struct spdk_bdev_io *bdev_io), NULL);

/* ---- Cross-file EC symbols bdev_ec_io.c links against ---- */
DEFINE_STUB(ec_wib_persist, int, (struct ec_bdev *ec,
				  void (*cb)(void *cb_arg, int rc), void *cb_arg), 0);
DEFINE_STUB(ec_submit_bit_clear_async, int, (struct ec_bdev *ec,
		uint64_t stripe_index, void (*cb_fn)(void *cb_arg, int rc),
		void *cb_arg), 0);

/* Read stub: records each child read so fan-out layout can be checked. */
#define UT_MAX_READS 8
struct ut_read {
	struct spdk_bdev_desc *desc;
	struct iovec          *iov;
	int                    iovcnt;
	uint64_t               offset_blocks;
	uint64_t               num_blocks;
};
static struct ut_read g_reads[UT_MAX_READS];
static int            g_read_calls;
static int            g_read_rc;

int
spdk_bdev_readv_blocks(struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
		       struct iovec *iov, int iovcnt,
		       uint64_t offset_blocks, uint64_t num_blocks,
		       spdk_bdev_io_completion_cb cb, void *cb_arg)
{
	if (g_read_calls < UT_MAX_READS) {
		g_reads[g_read_calls] = (struct ut_read) {
			.desc = desc, .iov = iov, .iovcnt = iovcnt,
			.offset_blocks = offset_blocks, .num_blocks = num_blocks,
		};
	}
	g_read_calls++;
	return g_read_rc;
}

/* RMW stub: claims the stripe like the real path so FIFO gating is testable. */
static int g_rmw_rc;
static int g_rmw_calls;

int
ec_submit_rmw_write(struct ec_bdev_io *ec_io)
{
	struct ec_bdev *ec = ec_from_bdev_io(ec_io->bdev_io);

	g_rmw_calls++;
	if (g_rmw_rc == 0) {
		ec->stripe_dirty_map[0] |=
			1ULL << (ec_io->offset_blocks / ec->stripe_blocks);
	}
	return g_rmw_rc;
}

/* ---- Fixture ---- */
#define UT_STRIPE_BLOCKS 16ULL

static struct ec_bdev g_ec;
static uint64_t       g_stripe_dirty[1];

struct ut_io {
	struct spdk_bdev_io bdev_io;
	struct ec_bdev_io   ec_io;
};

static void
ut_reset(void)
{
	memset(&g_ec, 0, sizeof(g_ec));
	g_stripe_dirty[0] = 0;
	g_rmw_rc          = 0;
	g_rmw_calls       = 0;
	g_read_rc         = 0;
	g_read_calls      = 0;
	memset(g_reads, 0, sizeof(g_reads));

	g_ec.bdev.name        = "ut_ec";
	g_ec.bdev.ctxt        = &g_ec;
	g_ec.bdev.blocklen    = 512;
	g_ec.k                = 2;
	g_ec.strip_size       = UT_STRIPE_BLOCKS / 2;
	g_ec.stripe_blocks    = UT_STRIPE_BLOCKS;
	g_ec.stripe_dirty_map = g_stripe_dirty;
	TAILQ_INIT(&g_ec.stripe_waitq);
}

static void
ut_io_init(struct ut_io *io, uint64_t stripe_index)
{
	memset(io, 0, sizeof(*io));
	io->bdev_io.bdev        = &g_ec.bdev;
	io->ec_io.bdev_io       = &io->bdev_io;
	io->ec_io.offset_blocks = stripe_index * UT_STRIPE_BLOCKS;
	io->ec_io.num_blocks    = 1;    /* misaligned -> RMW path */
}

static void
ut_mark_dirty(uint64_t stripe_index)
{
	g_stripe_dirty[0] |= 1ULL << stripe_index;
}

/* Clears the bit directly, without the release-side drain kick. */
static void
ut_clear_dirty_raw(uint64_t stripe_index)
{
	g_stripe_dirty[0] &= ~(1ULL << stripe_index);
}

/* One-buffer read fixture: parent payload in a single iovec. */
static struct ec_io_channel g_chan;
static char                 g_read_buf[UT_STRIPE_BLOCKS * 512];
static struct iovec         g_read_iov;

static void
ut_read_init(struct ut_io *io, uint64_t offset_blocks, uint64_t num_blocks)
{
	uint32_t i;

	for (i = 0; i < g_ec.k; i++) {
		g_ec.descs[i] = (struct spdk_bdev_desc *)(uintptr_t)(0x100 + i);
	}
	g_read_iov = (struct iovec) {
		.iov_base = g_read_buf,
		.iov_len = num_blocks * g_ec.bdev.blocklen,
	};

	memset(io, 0, sizeof(*io));
	io->bdev_io.bdev        = &g_ec.bdev;
	io->ec_io.bdev_io       = &io->bdev_io;
	io->ec_io.ch            = &g_chan;
	io->ec_io.offset_blocks = offset_blocks;
	io->ec_io.num_blocks    = num_blocks;
	io->ec_io.iovs          = &g_read_iov;
	io->ec_io.iovcnt        = 1;
}

/* Clean stripe: write dispatches straight to RMW, nothing parks. */
static void
test_write_clean_stripe_dispatches(void)
{
	struct ut_io a;

	ut_reset();
	ut_io_init(&a, 0);

	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	CU_ASSERT(g_rmw_calls == 1);
	CU_ASSERT(TAILQ_EMPTY(&g_ec.stripe_waitq));
	CU_ASSERT(g_ec.stripe_waitq_depth == 0);
	CU_ASSERT(g_ec.stripe_waitq_parked == 0);
}

/* Busy stripe: write parks instead of dispatching. */
static void
test_write_busy_stripe_parks(void)
{
	struct ut_io a;

	ut_reset();
	ut_io_init(&a, 0);
	ut_mark_dirty(0);

	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	CU_ASSERT(g_rmw_calls == 0);
	CU_ASSERT(g_ec.stripe_waitq_depth == 1);
	CU_ASSERT(g_ec.stripe_waitq_parked == 1);
	CU_ASSERT(g_ec.stripe_waitq_max_depth == 1);
	CU_ASSERT(TAILQ_FIRST(&g_ec.stripe_waitq) == &a.ec_io);
}

/*
 * A write behind a parked write for the same stripe parks even when the
 * stripe itself is clean; a write for another clean stripe dispatches.
 */
static void
test_write_parks_behind_parked_same_stripe(void)
{
	struct ut_io a, b, c;

	ut_reset();
	ut_io_init(&a, 0);
	ut_io_init(&b, 0);
	ut_io_init(&c, 1);

	ut_mark_dirty(0);
	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	ut_clear_dirty_raw(0);

	CU_ASSERT(ec_submit_write(&b.ec_io) == 0);
	CU_ASSERT(g_ec.stripe_waitq_depth == 2);
	CU_ASSERT(TAILQ_FIRST(&g_ec.stripe_waitq) == &a.ec_io);

	CU_ASSERT(ec_submit_write(&c.ec_io) == 0);
	CU_ASSERT(g_rmw_calls == 1);
	CU_ASSERT(g_ec.stripe_waitq_depth == 2);
	CU_ASSERT(g_ec.stripe_waitq_max_depth == 2);
}

/*
 * Drain resumes the head write; its dispatch re-claims the stripe, so the
 * second parked write for the same stripe stays queued.
 */
static void
test_drain_resumes_fifo(void)
{
	struct ut_io a, b;

	ut_reset();
	ut_io_init(&a, 0);
	ut_io_init(&b, 0);

	ut_mark_dirty(0);
	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	CU_ASSERT(ec_submit_write(&b.ec_io) == 0);
	ut_clear_dirty_raw(0);

	ec_stripe_waitq_drain_on_home(&g_ec);
	CU_ASSERT(g_rmw_calls == 1);
	CU_ASSERT(g_ec.stripe_waitq_depth == 1);
	CU_ASSERT(TAILQ_FIRST(&g_ec.stripe_waitq) == &b.ec_io);

	ut_clear_dirty_raw(0);
	ec_stripe_waitq_drain_on_home(&g_ec);
	CU_ASSERT(g_rmw_calls == 2);
	CU_ASSERT(g_ec.stripe_waitq_depth == 0);
	CU_ASSERT(TAILQ_EMPTY(&g_ec.stripe_waitq));
}

/* Drain leaves writes whose stripe is still busy untouched. */
static void
test_drain_skips_busy_stripe(void)
{
	struct ut_io a;

	ut_reset();
	ut_io_init(&a, 0);

	ut_mark_dirty(0);
	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);

	ec_stripe_waitq_drain_on_home(&g_ec);
	CU_ASSERT(g_rmw_calls == 0);
	CU_ASSERT(g_ec.stripe_waitq_depth == 1);
	CU_ASSERT(TAILQ_FIRST(&g_ec.stripe_waitq) == &a.ec_io);
}

/* A resumed write that defers again (-EAGAIN) completes NOMEM. */
static void
test_drain_nomem_completion(void)
{
	struct ut_io a;

	ut_reset();
	ut_io_init(&a, 0);

	ut_mark_dirty(0);
	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	ut_clear_dirty_raw(0);

	g_rmw_rc = -EAGAIN;
	ec_stripe_waitq_drain_on_home(&g_ec);
	CU_ASSERT(g_rmw_calls == 1);
	CU_ASSERT(g_ec.stripe_waitq_depth == 0);
	CU_ASSERT(a.ec_io.status == SPDK_BDEV_IO_STATUS_NOMEM);
	CU_ASSERT(g_ec.nomem_completions == 1);
}

/* fail_all completes every parked write with the given status. */
static void
test_fail_all(void)
{
	struct ut_io a, b;

	ut_reset();
	ut_io_init(&a, 0);
	ut_io_init(&b, 1);

	ut_mark_dirty(0);
	ut_mark_dirty(1);
	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	CU_ASSERT(ec_submit_write(&b.ec_io) == 0);

	ec_stripe_waitq_fail_all(&g_ec, SPDK_BDEV_IO_STATUS_ABORTED);
	CU_ASSERT(TAILQ_EMPTY(&g_ec.stripe_waitq));
	CU_ASSERT(g_ec.stripe_waitq_depth == 0);
	CU_ASSERT(a.ec_io.status == SPDK_BDEV_IO_STATUS_ABORTED);
	CU_ASSERT(b.ec_io.status == SPDK_BDEV_IO_STATUS_ABORTED);
}

/* ---- ec_iov_slice ---- */

/* Slice inside one iov: single output entry with adjusted base. */
static void
test_iov_slice_within_one(void)
{
	uint8_t      buf[64];
	struct iovec in  = { .iov_base = buf, .iov_len = sizeof(buf) };
	struct iovec out[2];
	int          cnt = -1;

	CU_ASSERT(ec_iov_slice(&in, 1, 16, 32, out, 2, &cnt) == 0);
	CU_ASSERT(cnt == 1);
	CU_ASSERT(out[0].iov_base == buf + 16);
	CU_ASSERT(out[0].iov_len == 32);
}

/* Slice spanning two iovs: tail of the first, head of the second. */
static void
test_iov_slice_across_boundary(void)
{
	uint8_t      buf0[32], buf1[32];
	struct iovec in[2] = {
		{ .iov_base = buf0, .iov_len = sizeof(buf0) },
		{ .iov_base = buf1, .iov_len = sizeof(buf1) },
	};
	struct iovec out[2];
	int          cnt = -1;

	CU_ASSERT(ec_iov_slice(in, 2, 24, 16, out, 2, &cnt) == 0);
	CU_ASSERT(cnt == 2);
	CU_ASSERT(out[0].iov_base == buf0 + 24);
	CU_ASSERT(out[0].iov_len == 8);
	CU_ASSERT(out[1].iov_base == buf1);
	CU_ASSERT(out[1].iov_len == 8);
}

/* Full-range slice reproduces the input layout. */
static void
test_iov_slice_full_range(void)
{
	uint8_t      buf0[32], buf1[32];
	struct iovec in[2] = {
		{ .iov_base = buf0, .iov_len = sizeof(buf0) },
		{ .iov_base = buf1, .iov_len = sizeof(buf1) },
	};
	struct iovec out[2];
	int          cnt = -1;

	CU_ASSERT(ec_iov_slice(in, 2, 0, 64, out, 2, &cnt) == 0);
	CU_ASSERT(cnt == 2);
	CU_ASSERT(out[0].iov_base == buf0);
	CU_ASSERT(out[0].iov_len == 32);
	CU_ASSERT(out[1].iov_base == buf1);
	CU_ASSERT(out[1].iov_len == 32);
}

/* Range past the payload end fails. */
static void
test_iov_slice_out_of_range(void)
{
	uint8_t      buf[32];
	struct iovec in  = { .iov_base = buf, .iov_len = sizeof(buf) };
	struct iovec out[2];
	int          cnt = -1;

	CU_ASSERT(ec_iov_slice(&in, 1, 16, 32, out, 2, &cnt) == -EINVAL);
	CU_ASSERT(ec_iov_slice(&in, 1, 32, 1, out, 2, &cnt) == -EINVAL);
}

/* Output array too small fails. */
static void
test_iov_slice_out_too_small(void)
{
	uint8_t      buf0[32], buf1[32];
	struct iovec in[2] = {
		{ .iov_base = buf0, .iov_len = sizeof(buf0) },
		{ .iov_base = buf1, .iov_len = sizeof(buf1) },
	};
	struct iovec out[1];
	int          cnt = -1;

	CU_ASSERT(ec_iov_slice(in, 2, 24, 16, out, 1, &cnt) == -EINVAL);
}

/* Read within one strip: one direct read using the parent iovs. */
static void
test_read_single_strip_direct(void)
{
	struct ut_io a;

	ut_reset();
	ut_read_init(&a, 2, 4);

	CU_ASSERT(ec_submit_read(&a.ec_io) == 0);
	CU_ASSERT(g_read_calls == 1);
	CU_ASSERT(g_reads[0].desc == g_ec.descs[0]);
	CU_ASSERT(g_reads[0].iov == &g_read_iov);
	CU_ASSERT(g_reads[0].offset_blocks == 2);
	CU_ASSERT(g_reads[0].num_blocks == 4);
	CU_ASSERT(a.ec_io.base_io_remaining == 1);
	CU_ASSERT(a.ec_io.data_iovs == NULL);
}

/* Read crossing a strip boundary: one child read per strip, payload
 * sliced at the boundary. */
static void
test_read_fanout_two_strips(void)
{
	struct ut_io a;

	ut_reset();
	ut_read_init(&a, 4, 8);	/* blocks 4-11: 4 in strip 0, 4 in strip 1 */

	CU_ASSERT(ec_submit_read(&a.ec_io) == 0);
	CU_ASSERT(g_read_calls == 2);
	CU_ASSERT(a.ec_io.base_io_remaining == 2);
	SPDK_CU_ASSERT_FATAL(a.ec_io.data_iovs != NULL);

	CU_ASSERT(g_reads[0].desc == g_ec.descs[0]);
	CU_ASSERT(g_reads[0].offset_blocks == 4);
	CU_ASSERT(g_reads[0].num_blocks == 4);
	CU_ASSERT(g_reads[0].iovcnt == 1);
	CU_ASSERT(g_reads[0].iov[0].iov_base == g_read_buf);
	CU_ASSERT(g_reads[0].iov[0].iov_len == 4 * 512);

	CU_ASSERT(g_reads[1].desc == g_ec.descs[1]);
	CU_ASSERT(g_reads[1].offset_blocks == 0);
	CU_ASSERT(g_reads[1].num_blocks == 4);
	CU_ASSERT(g_reads[1].iovcnt == 1);
	CU_ASSERT(g_reads[1].iov[0].iov_base == g_read_buf + 4 * 512);
	CU_ASSERT(g_reads[1].iov[0].iov_len == 4 * 512);

	free(a.ec_io.data_iovs);
	a.ec_io.data_iovs = NULL;
}

/* Fan-out where every child submit fails: caller gets an error and the
 * slice array is released. */
static void
test_read_fanout_all_submits_fail(void)
{
	struct ut_io a;

	ut_reset();
	ut_read_init(&a, 4, 8);
	g_read_rc = -ENOMEM;

	CU_ASSERT(ec_submit_read(&a.ec_io) == -EIO);
	CU_ASSERT(g_read_calls == 2);
	CU_ASSERT(a.ec_io.data_iovs == NULL);
	CU_ASSERT(a.ec_io.status == SPDK_BDEV_IO_STATUS_FAILED);
}

/* Degraded array, but every chunk the read touches is readable: routes
 * to the plain read path, no reconstruction. */
static void
test_degraded_read_readable_chunks_direct(void)
{
	struct ut_io a;

	ut_reset();
	ut_read_init(&a, 2, 4);	/* chunk 0 only */
	g_ec.failed_count    = 1;
	g_ec.base_states[1]  = EC_BASE_STATE_FAILED;

	CU_ASSERT(ec_submit_read(&a.ec_io) == 0);
	CU_ASSERT(g_read_calls == 1);
	CU_ASSERT(g_reads[0].desc == g_ec.descs[0]);
	CU_ASSERT(g_reads[0].num_blocks == 4);
}

/* Sub-stripe write crossing a strip boundary: still one RMW dispatch. */
static void
test_write_cross_strip_routes_rmw(void)
{
	struct ut_io a;

	ut_reset();
	ut_io_init(&a, 0);
	a.ec_io.offset_blocks = 4;
	a.ec_io.num_blocks    = 8;	/* strips 0 and 1, not a full stripe */

	CU_ASSERT(ec_submit_write(&a.ec_io) == 0);
	CU_ASSERT(g_rmw_calls == 1);
}

int
main(int argc, char **argv)
{
	CU_pSuite suite = NULL;
	unsigned int num_failures;

	CU_initialize_registry();

	suite = CU_add_suite("bdev_ec_io", NULL, NULL);

	CU_ADD_TEST(suite, test_write_clean_stripe_dispatches);
	CU_ADD_TEST(suite, test_write_busy_stripe_parks);
	CU_ADD_TEST(suite, test_write_parks_behind_parked_same_stripe);
	CU_ADD_TEST(suite, test_drain_resumes_fifo);
	CU_ADD_TEST(suite, test_drain_skips_busy_stripe);
	CU_ADD_TEST(suite, test_drain_nomem_completion);
	CU_ADD_TEST(suite, test_fail_all);
	CU_ADD_TEST(suite, test_iov_slice_within_one);
	CU_ADD_TEST(suite, test_iov_slice_across_boundary);
	CU_ADD_TEST(suite, test_iov_slice_full_range);
	CU_ADD_TEST(suite, test_iov_slice_out_of_range);
	CU_ADD_TEST(suite, test_iov_slice_out_too_small);
	CU_ADD_TEST(suite, test_read_single_strip_direct);
	CU_ADD_TEST(suite, test_read_fanout_two_strips);
	CU_ADD_TEST(suite, test_read_fanout_all_submits_fail);
	CU_ADD_TEST(suite, test_degraded_read_readable_chunks_direct);
	CU_ADD_TEST(suite, test_write_cross_strip_routes_rmw);

	num_failures = spdk_ut_run_tests(argc, argv, NULL);
	CU_cleanup_registry();

	return num_failures;
}
