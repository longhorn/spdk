/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_internal.h -- internal definitions for the EC bdev module.
 *
 * Include only from the module's .c files. Anything that would be
 * appropriate for an external consumer belongs in bdev_ec.h instead.
 */

/* =========================================================================
 * THREADING MODEL
 *
 * Each ec_bdev has a single home thread (ec->home_thread, captured at
 * create time) that owns all shared ec_bdev state: control-plane
 * mutations, WIB persist/load, scrub/rebuild orchestration, bitmap
 * coordination, and the rebuild/resize/scrub busy gates. Anything that
 * touches ec_bdev fields must run on the home thread or hand off via
 * spdk_thread_send_msg.
 *
 * I/O submission is per-channel. Each ec_io_channel owns base_chans[]
 * for its reactor. The submitter thread (the thread that received
 * bdev_io from the stack) is captured into ec_io->submitter_thread and
 * every base-bdev dispatch site re-asserts it. Completion callbacks
 * from base bdevs may fire on any thread; they are routed back to the
 * submitter thread via the routing helper, which is a no-op fast path
 * when the caller is already on the target thread.
 *
 * No locks. Cross-thread fields use C11 atomics; everything else relies
 * on the "all writers on home_thread, or all writers on submitter_thread"
 * invariant being honoured at every site.
 *
 * =========================================================================
 * CHANNEL INVENTORY
 *
 * Five kinds of I/O channels are managed by this module, each with a
 * distinct owner and lifetime:
 *
 *   ec_io_channel.base_chans[]       per-thread; created in ec_create_ch,
 *                                    destroyed in ec_destroy_ch (in
 *                                    bdev_ec.c). Used by user I/O paths
 *                                    in bdev_ec_io.c and bdev_ec_rmw.c.
 *
 *   ec_bdev.wib_chans[]              one per parity disk; created in
 *                                    ec_bdev_create_async (bdev_ec.c),
 *                                    released in ec_device_unregister_done.
 *                                    Also released per-slot in
 *                                    ec_handle_base_bdev_failure when a
 *                                    parity disk fails. Used by the WIB
 *                                    idle poller (bdev_ec_wib.c).
 *
 *   ec_bdev.bitmap_chans[]           one per disk (all n; the unmapped
 *                                    bitmap is raw-replicated to every
 *                                    disk); created in ec_bdev_create_async
 *                                    (bdev_ec.c), released in
 *                                    ec_device_unregister_done. Also
 *                                    released per-slot in
 *                                    ec_handle_base_bdev_failure. Used by
 *                                    the bitmap persist/load
 *                                    (bdev_ec_bitmap.c).
 *
 *   ec_rebuild_ctx.rebuild_chans[]   one per live slot; created in
 *                                    ec_bdev_start_rebuild
 *                                    (bdev_ec_rebuild.c), released in
 *                                    ec_rebuild_free_resources. Per-slot
 *                                    release also from
 *                                    ec_handle_base_bdev_failure.
 *
 *   ec_scrub_ctx.scrub_chans[]       one per live slot; created in
 *                                    ec_bdev_start_scrub
 *                                    (bdev_ec_rebuild.c), released in
 *                                    ec_scrub_free_resources. Per-slot
 *                                    release also from
 *                                    ec_handle_base_bdev_failure.
 *
 * The dedicated channels (wib, bitmap, rebuild, scrub) bypass the per-thread
 * ec_io_channel ref-counting so that long-lived background work does
 * not pin every user-thread channel for the duration.
 * ========================================================================= */

#ifndef SPDK_BDEV_EC_INTERNAL_H
#define SPDK_BDEV_EC_INTERNAL_H

#include "bdev_ec.h"

#include "spdk/stdinc.h"
#include "spdk/bdev.h"
#include "spdk/bdev_module.h"
#include "spdk/log.h"
#include "spdk/queue.h"

/* Maximum number of base bdevs (data + parity) */
#define EC_MAX_BASE_BDEVS 32

/*
 * Max chunk count (n = k + m, and each of k and m) the GF(2^8) Reed-Solomon
 * field allows.
 */
#define EC_GF8_MAX_CHUNKS 255u

/* Maximum length (including NUL) of an EC bdev or base-bdev name. */
#define EC_BDEV_NAME_MAX 256

/*
 * Thread-local context for EC bdev I/O. One instance per (bdev, thread).
 * base_chans[i] is the SPDK I/O channel for base bdev i, opened when the
 * EC bdev's I/O channel is created on this thread and released on destroy.
 */
struct ec_io_channel {
	struct spdk_io_channel *base_chans[EC_MAX_BASE_BDEVS];
};

/* Alignment for DMA buffers used in SPDK allocations */
#define EC_DMA_ALIGN 4096

/* Rebuild/scrub poller period (us). One stripe per tick; capped to limit
 * foreground I/O impact. */
#define EC_REBUILD_POLL_PERIOD_US 100

/* Rebuild/scrub progress heartbeat: emit a NOTICE every N seconds or
 * every M percent of total progress, whichever fires first. Bounds the
 * log to ~tens of lines for an entire operation while preserving a
 * wall-clock trajectory in post-mortem logs. Shared between rebuild and
 * scrub since they have the same operational shape. */
#define EC_REBUILD_HEARTBEAT_SEC 30
#define EC_REBUILD_HEARTBEAT_PERCENT_STEP 10

/*
 * Number of uint64_t words needed to cover `bits` bitmap entries.
 * Used uniformly for stripe_dirty_map and wib_region_map sizing.
 */
#define EC_BITMAP_WORDS(bits) (((bits) + 63) / 64)

/*
 * Base bdev state tracking for failure detection and hot-swap replacement.
 *
 * State machine:
 *
 *   NORMAL    --[REMOVE event]--> FAILED
 *   FAILED    --[replace RPC]---> REPLACING
 *   REPLACING --[rebuild done]--> NORMAL
 *
 * REPLACING means a new disk has been accepted but the background rebuild
 * has not yet finished. The slot is writable; reads still reconstruct.
 */
enum ec_base_bdev_state {
	EC_BASE_STATE_NORMAL    = 0,  /* Base bdev is healthy and operational */
	EC_BASE_STATE_FAILED    = 1,  /* Base bdev has failed or been removed */
	EC_BASE_STATE_REPLACING = 2,  /* Replacement inserted; rebuild pending */
};

/* =========================================================================
 * Hot-swap replace context
 * ========================================================================= */

/* Carried through the async per-thread channel walk. Freed in ec_replace_finish(). */
struct ec_replace_ctx {
	struct ec_bdev        *ec;
	uint32_t               slot;
	char                   new_bdev_name[EC_BDEV_NAME_MAX];
	struct spdk_bdev_desc *new_desc;
	ec_replace_cb_fn       cb_fn;
	void                  *cb_arg;
};
struct ec_bdev {
	/* Generic SPDK bdev structure (must be first) */
	struct spdk_bdev bdev;

	/*
	 * Thread that created the EC bdev and owns its persist state:
	 * bitmap_chans[], pending_bit_clears, bitmap_persist_in_flight,
	 * bitmap_active_copy, and bitmap_generation. SPDK bdev channels are
	 * thread-affine and the persist serializes through this single writer,
	 * so I/O-path callers on other reactors must route bitmap-persist
	 * work here via spdk_thread_send_msg.
	 */
	struct spdk_thread *home_thread;

	/*
	 * EC configuration.
	 *
	 * strip_size_kb: user-configured per-chunk strip size in KiB.
	 * strip_size:    per-chunk strip size in blocks (strip_size_kb * 1024 / blocklen).
	 * stripe_blocks: full stripe in blocks (k * strip_size).
	 */
	uint32_t k;
	uint32_t m;
	uint32_t n;
	uint32_t strip_size_kb;
	uint64_t strip_size;
	uint64_t stripe_blocks;

	/* ISA-L Encoding Tables */
	uint8_t *encode_matrix;
	uint8_t *g_tbls;

	/* Base Device Management */
	struct spdk_bdev_desc *descs[EC_MAX_BASE_BDEVS];

	/* Failure Tracking */
	enum ec_base_bdev_state base_states[EC_MAX_BASE_BDEVS];

	uint8_t failed_count;          /* non-NORMAL slot count (FAILED + REPLACING) */
	bool offline;

	bool needs_rebuild[EC_MAX_BASE_BDEVS]; /* set on FAILED -> REPLACING */
	bool replace_in_progress;              /* serialise replace ops     */
	bool bdev_registered;                  /* spdk_bdev_register succeeded:
	                                        * gates spdk_bdev_destruct_done so
	                                        * create-failure teardown never
	                                        * completes a destruct that never
	                                        * started */

	struct ec_rebuild_ctx *rebuild_ctx;    /* non-NULL while rebuilding */
	struct ec_scrub_ctx *scrub_ctx;        /* non-NULL while scrubbing  */
	struct ec_resize_ctx *resize_ctx;      /* non-NULL while resizing   */

	/*
	 * RMW write-hole tracking.
	 *
	 * stripe_dirty_map: one bit per stripe, set while any write-side path
	 * (RMW, full-stripe write, UNMAP, rebuild) holds the stripe. A second
	 * claim on a dirty stripe returns NOMEM (requeue).
	 */
	uint64_t  num_stripes;           /* user-visible stripe count            */
	/*
	 * Front reservation, in physical stripes, covering the in-band
	 * unmapped bitmap slots plus the two WIB copies in
	 * [0, data_offset_stripes). Fixed-max: sized once at create from
	 * strip_size + blocklen, so it never moves or grows on resize.
	 * User stripe u maps to physical stripe u + data_offset_stripes --
	 * see ec_stripe_base_lba().
	 */
	uint64_t  data_offset_stripes;
	uint64_t *stripe_dirty_map;      /* (num_stripes+63)/64 uint64_t words   */
	/*
	 * Persistent per-stripe unmapped bitmap, one bit per user stripe,
	 * 1 = the stripe is logically zero (unmapped). In-memory mirror of
	 * the in-band on-disk bitmap in the [0, data_offset_stripes) region;
	 * see bdev_ec_bitmap.c. User-stripe-indexed, same as stripe_dirty_map.
	 */
	uint64_t *stripe_unmapped_map;   /* (num_stripes+63)/64 uint64_t words   */
	uint32_t  rmw_in_flight;         /* count of concurrent RMW operations   */

	/* Write-Intent Bitmap (WIB) -- see WIB section above for overview. */
	uint32_t                 wib_num_regions;
	uint64_t                *wib_region_map;      /* in-memory dirty bits  */
	uint32_t                *wib_region_inflight;  /* per-region RMW count  */
	uint64_t                *wib_region_dirty_ticks;  /* tick when marked dirty*/
	struct spdk_io_channel  *wib_chans[EC_MAX_BASE_BDEVS]; /* m entries */
	void                    *wib_buf;              /* DMA buf, one strip   */
	uint32_t                 wib_generation;
	uint8_t                  wib_active_copy;      /* 0 or 1               */
	bool                     wib_persist_in_flight;
	/* New dirty bit arrived while persist in flight; triggers follow-up. */
	bool                     wib_repersist_needed;
	struct spdk_poller      *wib_poller;

	/*
	 * RMW contexts deferred because wib_persist_in_flight was true when a
	 * new region needed its dirty bit persisted. Drained by
	 * ec_wib_deferred_drain once the follow-up persist completes.
	 */
	TAILQ_HEAD(, ec_rmw_ctx) wib_deferred_writes;

	/*
	 * In-band unmapped bitmap -- see the bitmap section earlier in this
	 * header and bdev_ec_bitmap.c for the on-disk model. bitmap_chans[]
	 * holds n entries (touched by every disk, unlike WIB which only
	 * touches the m parity disks); it is opened by the create path and
	 * closed by destruct. bitmap_active_copy is a single global slot
	 * index (0 or 1), not per-disk, so a disk that missed a prior persist
	 * re-syncs automatically on the next persist.
	 */
	struct spdk_io_channel  *bitmap_chans[EC_MAX_BASE_BDEVS]; /* n entries */
	uint32_t                 bitmap_generation;
	uint8_t                  bitmap_active_copy;    /* 0 or 1               */
	bool                     bitmap_persist_in_flight;

	/*
	 * Bit-clear waiter queue (write-into-unmapped path).
	 *
	 * Each entry represents one stripe whose unmapped bit needs to be
	 * cleared after its data writes have landed. The waiter holds a
	 * callback that fires when the bit-clear persist has durably acked
	 * at m+1, at which point the caller (write-into-unmapped completion)
	 * can ack its bdev_io and release the stripe-busy claim.
	 *
	 * Two-list staging so a new clear can be enqueued while a previous
	 * persist's writes are still draining:
	 *   pending_bit_clears: arrived since the last persist kicked, not
	 *     yet included in any persist.
	 *   in_flight_bit_clears: included in the current bit-clear persist,
	 *     awaiting cb_durable.
	 *
	 * Coalescing: a burst of N completions during one in-flight bitmap
	 * persist (UNMAP's or a prior write-into-unmapped's) all queue into
	 * pending; on that persist's drainout, ec_bit_clear_flush_if_pending
	 * fires a single follow-up persist covering all N at once. Caps
	 * worst-case post-trim write throughput at one extra persist per
	 * roundtrip rather than one persist per write.
	 */
	TAILQ_HEAD(, ec_pending_bit_clear) pending_bit_clears;
	TAILQ_HEAD(, ec_pending_bit_clear) in_flight_bit_clears;
	/* Shadow of the live map with the in-flight clears applied; NULL when
	 * idle. Built, persisted, and freed by the bit-clear waiter queue in
	 * bdev_ec_bitmap.c (ec_bit_clear_flush / ec_bit_clear_on_drained). */
	uint64_t                          *clear_staged_map;

	/*
	 * Cumulative I/O counters. Sampled by tooling via bdev_ec_get_bdevs
	 * twice to derive rates. The per-I/O DEBUG logs that used to cover
	 * these were removed because they flooded; counters are the supported
	 * trace surface.
	 */
	uint64_t degraded_read_eio_dirty;        /* reads rejected: dirty WIB region */
	uint64_t degraded_reads_reconstructed;   /* reads served via reconstruction  */
	uint64_t rmw_total;                      /* sub-stripe writes accepted       */
	uint64_t rmw_deferred_scrub;             /* RMW EAGAIN: scrub-active region  */
	uint64_t rmw_deferred_dirty;             /* RMW EAGAIN: deferred-scrub guard */
	uint64_t rmw_deferred_inflight;          /* RMW EAGAIN: stripe already dirty */
	uint64_t full_stripe_writes;             /* full-stripe writes accepted      */
	uint64_t full_stripe_writes_deferred;    /* full-stripe EAGAIN: scrub guard  */
	uint64_t unmaps_submitted;               /* all UNMAP submit attempts        */
	uint64_t unmaps_completed;               /* native UNMAP requests succeeded  */
	uint64_t unmaps_deferred_busy;           /* UNMAP EAGAIN: stripe-busy/persist */
	uint64_t unmaps_via_write_zeros;         /* UNMAP routed to write-zeros path */
	uint64_t unmap_fanout_misses;            /* per-disk spdk_bdev_unmap_blocks
						  * failure -- physical space not
						  * reclaimed on that disk; not a
						  * bdev_io failure (bitmap already
						  * says unmapped, reads still
						  * synthesise zeros) */
	uint64_t unmapped_reads_synthesized;     /* reads short-circuited to zero
						  * fill because the target stripe's
						  * unmapped bit was set. Production
						  * signal that bitmap consultation
						  * is firing -- if this stays at 0
						  * after fstrim activity, the
						  * read-path hookup is broken. */
	uint64_t writes_into_unmapped;           /* writes routed through the
						  * write-into-unmapped full-stripe
						  * path (skip-WIB, zero-fill old
						  * data, clear bit on completion).
						  * Production signal that the
						  * write-side hookup is firing on
						  * post-trim write workloads. */
	uint64_t writes_into_unmapped_failed;    /* write-into-unmapped paths
						  * that failed at stripe-alloc
						  * setup or at the bit-clear
						  * submit/persist step. A data
						  * (fanout) write failure is not
						  * counted here; it completes as a
						  * normal failed bdev_io. The
						  * unmapped bit stays set in every
						  * failure case. */

	/*
	 * Backpressure transition tracking. Set when the scrub guards (active
	 * scrub or deferred scrub) first defer an RMW; cleared when the
	 * underlying condition ends (scrub finishes, scrub starts after being
	 * deferred, rebuild restores failed disks). Drives one-shot NOTICE
	 * logs at each transition -- the per-retry log was removed because it
	 * spammed thousands of identical lines while a single stripe waited.
	 */
	bool      rmw_backpressure_active;
	uint64_t  rmw_backpressure_since_ticks;
	uint64_t  rmw_backpressure_count_at_start;

	/* List Entry */
	TAILQ_ENTRY(ec_bdev) link;
};

/* Global list type */
TAILQ_HEAD(ec_all_tailq, ec_bdev);
extern struct ec_all_tailq g_ec_bdev_list;

/* SPDK bdev module identity, referenced by quiesce/unquiesce and other helpers. */
extern struct spdk_bdev_module ec_if;

/* =========================================================================
 * Inline helpers
 *
 * Small, hot-path helpers shared by multiple .c files in the module.
 * Each translation unit gets its own copy and is free to inline locally.
 * ========================================================================= */

static inline struct ec_bdev *
ec_from_bdev_io(struct spdk_bdev_io *bdev_io)
{
	return (struct ec_bdev *)bdev_io->bdev->ctxt;
}

/*
 * Translate a user stripe index to its per-base-disk LBA, accounting for
 * the in-band unmapped bitmap reservation at the front of the address
 * space. Every base-LBA computation in the module must route through this
 * helper -- a raw stripe_index * strip_size would address into the
 * reserved [0, data_offset_stripes) region.
 */
static inline uint64_t
ec_stripe_base_lba(const struct ec_bdev *ec, uint64_t stripe_index)
{
	return (stripe_index + ec->data_offset_stripes) * ec->strip_size;
}

static inline void
ec_calc_mapping(const struct ec_bdev *ec, uint64_t offset_blocks,
		uint64_t *stripe_index, uint32_t *chunk_idx,
		uint64_t *chunk_offset, uint64_t *base_lba)
{
	uint64_t stripe_off = offset_blocks % ec->stripe_blocks;

	*stripe_index = offset_blocks / ec->stripe_blocks;
	*chunk_idx    = stripe_off / ec->strip_size;
	*chunk_offset = stripe_off % ec->strip_size;
	*base_lba     = ec_stripe_base_lba(ec, *stripe_index) + *chunk_offset;
}

/*
 * True only for NORMAL slots. REPLACING slots have a live descriptor but
 * their data is incomplete until rebuild finishes; reading them during
 * reconstruction would silently corrupt the result.
 */
static inline bool
ec_slot_is_readable(const struct ec_bdev *ec, uint32_t slot)
{
	return ec->base_states[slot] == EC_BASE_STATE_NORMAL;
}

/*
 * True for NORMAL and REPLACING. REPLACING disks receive live writes so
 * the rebuilder only needs to fill stripes written before the swap.
 */
static inline bool
ec_slot_is_writable(const struct ec_bdev *ec, uint32_t slot)
{
	return ec->base_states[slot] == EC_BASE_STATE_NORMAL ||
	       ec->base_states[slot] == EC_BASE_STATE_REPLACING;
}

/*
 * True if every non-NORMAL slot is a parity disk (slot >= k). Used by
 * ec_submit_read to take the zero-overhead fast path. False if any DATA
 * slot is REPLACING (reconstruction required).
 */
static inline bool
ec_only_parity_failed(const struct ec_bdev *ec)
{
	uint32_t i;

	if (ec->failed_count == 0) {
		return false;
	}
	for (i = 0; i < ec->k; i++) {
		if (ec->base_states[i] != EC_BASE_STATE_NORMAL) {
			return false;
		}
	}
	return true;
}

#endif /* SPDK_BDEV_EC_INTERNAL_H */
