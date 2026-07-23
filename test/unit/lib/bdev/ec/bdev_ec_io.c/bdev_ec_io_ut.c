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
DEFINE_STUB(spdk_bdev_get_name, const char *, (const struct spdk_bdev *bdev), "ut_base");
DEFINE_STUB(spdk_bdev_io_get_thread, struct spdk_thread *, (struct spdk_bdev_io *bdev_io), NULL);

/* ---- Cross-file EC symbols bdev_ec_io.c links against ---- */
DEFINE_STUB(ec_wib_persist, int, (struct ec_bdev *ec,
				  void (*cb)(void *cb_arg, int rc), void *cb_arg), 0);
DEFINE_STUB(ec_submit_bit_clear_async, int, (struct ec_bdev *ec,
		uint64_t stripe_index, void (*cb_fn)(void *cb_arg, int rc),
		void *cb_arg), 0);

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

	g_ec.bdev.name        = "ut_ec";
	g_ec.bdev.ctxt        = &g_ec;
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

	num_failures = spdk_ut_run_tests(argc, argv, NULL);
	CU_cleanup_registry();

	return num_failures;
}
