/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

#include "spdk/stdinc.h"
#include "spdk_internal/cunit.h"
#include "spdk/env.h"

#include "common/lib/test_env.c"

#include "bdev/ec/bdev_ec_wib.c"

/*
 * Tests the WIB idle-clear poller (ec_wib_idle_poller_cb) and its
 * crash-region skip -- the line that keeps crash evidence alive until the
 * scrub retires it. A regression there is silent, so it gets its own binary.
 *
 * Fixture, no mocks needed:
 *   - descs[] all NULL: ec_wib_persist skips every parity write and finishes
 *     synchronously in memory (generation bump, in-flight flag cycle).
 *   - the idle check is (now - dirty_ticks >= idle_ticks): set dirty_ticks to
 *     0 and advance the clock past the window to make a region look quiet.
 *
 * Not covered here: the load-merge that seeds the crash map from the on-disk
 * WIB (ec_wib_load_async) -- it needs the async read chain, covered by the
 * create/load integration path.
 */

/* ---- SPDK surface bdev_ec_wib.c links against ---- */
DEFINE_STUB(spdk_get_thread, struct spdk_thread *, (void), NULL);
DEFINE_STUB(spdk_bdev_write, int, (struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
				   void *buf, uint64_t offset, uint64_t nbytes,
				   spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB(spdk_bdev_read, int, (struct spdk_bdev_desc *desc, struct spdk_io_channel *ch,
				  void *buf, uint64_t offset, uint64_t nbytes,
				  spdk_bdev_io_completion_cb cb, void *cb_arg), 0);
DEFINE_STUB_V(spdk_bdev_free_io, (struct spdk_bdev_io *bdev_io));

/* ---- Cross-file EC functions defined in other .c of the module ---- */
DEFINE_STUB(ec_bdev_find, struct ec_bdev *, (const char *name), NULL);
/* Feeds ec_wib_lba's on-disk offset; unused here since every write is
 * skipped (descs[] NULL), so any value is fine. */
DEFINE_STUB(ec_bitmap_reservation_stripes, uint64_t, (const struct ec_bdev *ec), 0);
DEFINE_STUB_V(ec_drain_deferred_slot_releases, (struct ec_bdev *ec));
DEFINE_STUB_V(ec_drain_deferred_unregister, (struct ec_bdev *ec));
DEFINE_STUB_V(ec_rmw_complete, (struct ec_rmw_ctx *mctx));
DEFINE_STUB_V(ec_rmw_dispatch_reads, (struct ec_rmw_ctx *mctx));

/* ---- Fixture ---- */
#define UT_WIB_BLOCKLEN 512u
#define UT_WIB_STRIP    128u    /* blocks; wib_buf = 64 KiB, ample for the header */
#define UT_WIB_REGIONS  4u

static void
ut_wib_init(struct ec_bdev *ec)
{
	uint32_t words = (UT_WIB_REGIONS + 63) / 64;

	memset(ec, 0, sizeof(*ec));
	ec->k = 4;
	ec->m = 2;
	ec->n = 6;
	ec->bdev.name       = "ec_wib_ut";
	ec->bdev.blocklen   = UT_WIB_BLOCKLEN;
	ec->strip_size      = UT_WIB_STRIP;
	ec->num_stripes     = (uint64_t)UT_WIB_REGIONS * EC_WIB_REGION_STRIPES;
	ec->wib_num_regions = UT_WIB_REGIONS;
	ec->home_thread     = NULL;   /* matches the spdk_get_thread() stub */

	ec->wib_region_map         = calloc(words, sizeof(uint64_t));
	ec->wib_crash_dirty_map    = calloc(words, sizeof(uint64_t));
	ec->wib_region_inflight    = calloc(UT_WIB_REGIONS, sizeof(uint32_t));
	ec->wib_region_dirty_ticks = calloc(UT_WIB_REGIONS, sizeof(uint64_t));
	ec->wib_buf = spdk_dma_zmalloc((uint64_t)UT_WIB_STRIP * UT_WIB_BLOCKLEN, 4096, NULL);
	SPDK_CU_ASSERT_FATAL(ec->wib_region_map && ec->wib_crash_dirty_map &&
			     ec->wib_region_inflight && ec->wib_region_dirty_ticks &&
			     ec->wib_buf);
	TAILQ_INIT(&ec->wib_deferred_writes);
}

static void
ut_wib_free(struct ec_bdev *ec)
{
	free(ec->wib_region_map);
	free(ec->wib_crash_dirty_map);
	free(ec->wib_region_inflight);
	free(ec->wib_region_dirty_ticks);
	spdk_dma_free(ec->wib_buf);
}

/* Superset invariant: every crash-dirty region is also live-dirty. */
static void
ut_wib_assert_superset(const struct ec_bdev *ec)
{
	uint32_t r;

	for (r = 0; r < ec->wib_num_regions; r++) {
		if (ec_wib_crash_is_dirty(ec, r)) {
			CU_ASSERT(ec_wib_region_is_dirty(ec, r));
		}
	}
}

/*
 * A write-intent-only region that has gone idle is cleared by the poller,
 * which then runs the (synchronous, all-parity-skipped) persist. This is the
 * behavior the crash-region skip must NOT break for ordinary traffic.
 */
static void
test_idle_poller_clears_write_intent(void)
{
	struct ec_bdev ec;
	uint64_t       gen_before;
	int            rc;

	ut_wib_init(&ec);

	ec_wib_region_set_dirty(&ec, 1);
	ec.wib_region_dirty_ticks[1] = 0;   /* written "long ago" (tick 0) */
	CU_ASSERT(ec_wib_region_is_dirty(&ec, 1));
	CU_ASSERT(!ec_wib_crash_is_dirty(&ec, 1));

	/* Past the idle window so region 1 reads as quiet. */
	MOCK_SET(spdk_get_ticks, (uint64_t)spdk_get_ticks_hz() * EC_WIB_IDLE_MS / 1000 + 1);

	gen_before = ec.wib_generation;
	rc = ec_wib_idle_poller_cb(&ec);

	CU_ASSERT(rc == SPDK_POLLER_BUSY);              /* a region was cleared */
	CU_ASSERT(!ec_wib_region_is_dirty(&ec, 1));     /* cleared from the live map */
	CU_ASSERT(ec.wib_generation == gen_before + 1); /* the persist ran */
	CU_ASSERT(ec.wib_persist_in_flight == false);   /* sync-finish cycled it back */
	ut_wib_assert_superset(&ec);

	MOCK_CLEAR(spdk_get_ticks);
	ut_wib_free(&ec);
}

/*
 * A crash-dirty region is NEVER cleared by the poller, across repeated ticks
 * and regardless of scrub / degraded state. The skip is what keeps the crash
 * record alive on disk (the poller never persists it clean) for the scrub.
 */
static void
ut_run_preserve_case(struct ec_scrub_ctx *scrub_ctx, uint32_t failed_count)
{
	struct ec_bdev ec;
	uint64_t       gen_before;
	int            rc;
	int            tick;

	ut_wib_init(&ec);
	ec.scrub_ctx    = scrub_ctx;
	ec.failed_count = failed_count;

	ec_wib_region_set_dirty(&ec, 2);    /* load-merge seeds both maps */
	ec_wib_crash_set_dirty(&ec, 2);
	ec.wib_region_dirty_ticks[2] = 0;   /* written "long ago" (tick 0) */

	/*
	 * Past the idle window so region 2 IS quiet -- without the crash skip
	 * the poller would clear it now. That makes the skip the thing tested.
	 */
	MOCK_SET(spdk_get_ticks, (uint64_t)spdk_get_ticks_hz() * EC_WIB_IDLE_MS / 1000 + 1);

	gen_before = ec.wib_generation;

	for (tick = 0; tick < 3; tick++) {
		rc = ec_wib_idle_poller_cb(&ec);
		CU_ASSERT(rc == SPDK_POLLER_IDLE);          /* nothing cleared */
		CU_ASSERT(ec_wib_crash_is_dirty(&ec, 2));
		CU_ASSERT(ec_wib_region_is_dirty(&ec, 2));
		ut_wib_assert_superset(&ec);
	}
	CU_ASSERT(ec.wib_generation == gen_before);     /* no persist ran */

	MOCK_CLEAR(spdk_get_ticks);
	ut_wib_free(&ec);
}

static void
test_idle_poller_preserves_crash_regions(void)
{
	struct ec_scrub_ctx scrub_ctx;

	/* Deferred-scrub window: no scrub active. */
	ut_run_preserve_case(NULL, 0);

	/*
	 * Poller-vs-scrub race: a scrub is active with its cursor before the
	 * crash region. The skip is by crash-map membership, not scrub state,
	 * so it holds -- pinned explicitly because this is the scenario that
	 * regresses silently if the skip is ever relaxed.
	 */
	memset(&scrub_ctx, 0, sizeof(scrub_ctx));
	scrub_ctx.current_region = 0;
	scrub_ctx.current_stripe = 0;
	ut_run_preserve_case(&scrub_ctx, 0);

	/*
	 * Deferred-scrub-after-rebuild, reduced to its unit-testable core: a
	 * crash region survives repeated poller ticks while degraded
	 * (failed_count > 0), so it is still present for the post-rebuild scrub.
	 */
	ut_run_preserve_case(NULL, 1);
}

int
main(int argc, char **argv)
{
	CU_pSuite suite = NULL;
	unsigned int num_failures;

	CU_initialize_registry();

	suite = CU_add_suite("bdev_ec_wib", NULL, NULL);

	CU_ADD_TEST(suite, test_idle_poller_clears_write_intent);
	CU_ADD_TEST(suite, test_idle_poller_preserves_crash_regions);

	num_failures = spdk_ut_run_tests(argc, argv, NULL);
	CU_cleanup_registry();

	return num_failures;
}
