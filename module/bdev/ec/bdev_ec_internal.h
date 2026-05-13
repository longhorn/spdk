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
 * The EC module runs single-threaded on the SPDK app thread. All RPC
 * handlers, I/O submission, completion callbacks, pollers, and shared
 * ec_bdev state mutations occur on that thread. No locks are used.
 *
 * spdk_for_each_channel walks per-thread ec_io_channel instances on
 * each reactor; those callbacks touch only per-channel state
 * (ec_io_channel.base_chans[]), not the shared ec_bdev.
 *
 * Any change that introduces concurrent access to ec_bdev fields
 * (worker thread, multi-reactor dispatch, etc.) MUST re-audit every
 * check-then-act pattern in this module -- especially the busy gates
 * around rebuild/resize/scrub and the WIB persist machinery.
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

/* ISA-L gf_gen_decode_matrix() requires (k * m * 32) bytes for the
 * encode / decode tables. */
#define EC_ISAL_GF_TABLE_BYTES 32u

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

/* =========================================================================
 * Background rebuild context
 * ========================================================================= */

/*
 * Deferred-stripe queue entry. Used when the rebuild poller picks a stripe
 * that is currently claimed by a foreground writer (RMW, full-stripe write,
 * or UNMAP). The stripe index is parked on ec_rebuild_ctx::deferred_stripes
 * and revisited after the main per-slot scan completes.
 */
struct ec_rebuild_deferred_stripe {
	uint64_t                                 stripe_index;
	TAILQ_ENTRY(ec_rebuild_deferred_stripe)  link;
};

/*
 * One instance per ec_bdev_start_rebuild() call. Freed by ec_rebuild_finish().
 * Entirely single-threaded on the SPDK app thread; no locking needed.
 * One stripe per poller tick: reads -> reconstruct -> write.
 */
struct ec_rebuild_ctx {
	struct ec_bdev  *ec;

	/* current REPLACING slot being rebuilt (slot index 0..n-1) */
	uint32_t         current_slot;

	/* stripe index within current_slot (0..num_stripes-1) */
	uint64_t         current_stripe;

	/* total stripes = ec->bdev.blockcnt / ec->stripe_blocks */
	uint64_t         num_stripes;

	/* bytes per chunk = ec->strip_size * ec->bdev.blocklen */
	uint64_t         chunk_bytes;

	/* DMA-safe I/O buffers, one per slot; reused across all stripes. */
	uint8_t         *chunk_bufs[EC_MAX_BASE_BDEVS];
	struct iovec     chunk_iovs[EC_MAX_BASE_BDEVS];

	/* Dedicated I/O channels, separate from user ec_io_channel instances. */
	struct spdk_io_channel *rebuild_chans[EC_MAX_BASE_BDEVS];

	/* True while reads or a write are outstanding; guards the poller. */
	bool             io_in_flight;

	/* countdown of outstanding reads for the current stripe */
	uint32_t         reads_remaining;

	/* accumulated I/O status for the current stripe */
	enum spdk_bdev_io_status io_status;

	/* total stripes written successfully (across all rebuilt slots) */
	uint64_t         stripes_rebuilt;

	/* REPLACING slot count at rebuild start; for percent_complete calc. */
	uint32_t         slots_to_rebuild;

	/* Heartbeat progress logging (see EC_REBUILD_HEARTBEAT_*). */
	uint64_t  start_ticks;           /* ticks when rebuild was registered */
	uint64_t  last_heartbeat_ticks;  /* ticks at last heartbeat NOTICE */
	uint32_t  next_heartbeat_percent;    /* next percent milestone (10, 20, ...) */

	/* SPDK poller driving the rebuild loop */
	struct spdk_poller *poller;

	/* QoS: rate-limit rebuild to reduce foreground I/O impact. */
	uint32_t  max_stripes_per_sec;  /* 0 = unlimited */
	uint64_t  window_start_ticks;   /* start of current 1-second window */
	uint32_t  stripes_this_window;  /* stripes submitted in current window */
	bool      paused;               /* manual pause via RPC */
	bool      cancel_requested;     /* set by stop_rebuild; poller drains I/O then finishes */

	/*
	 * Stripe-busy interlock.
	 *
	 * Rebuild now participates in the universal stripe-busy claim that
	 * gates RMW, full-stripe writes, and UNMAP. When the poller picks a
	 * stripe that is already claimed by a foreground writer it parks the
	 * index on deferred_stripes and moves on. After the main per-slot
	 * scan completes the poller flips into draining_deferred_stripes mode and
	 * drains the queue. stripe_claimed tracks whether current_stripe is
	 * presently held in ec->stripe_dirty_map so failure / cancel paths
	 * can release it.
	 */
	bool      stripe_claimed;
	bool      draining_deferred_stripes;
	TAILQ_HEAD(ec_rebuild_deferred_tailq, ec_rebuild_deferred_stripe) deferred_stripes;

	/* completion callback and opaque argument from the RPC layer */
	ec_rebuild_cb_fn cb_fn;
	void            *cb_arg;
};

/* =========================================================================
 * Write-Intent Bitmap (WIB)
 *
 * Protects against the RMW write-hole. One dirty bit per region of
 * EC_WIB_REGION_STRIPES stripes, stored on-disk as two alternating
 * copies in the two front-placed reservation strips on every parity disk. See
 * bdev_ec_wib.c for the on-disk layout and persist protocol.
 * ========================================================================= */

/* Stripes per WIB region -- one dirty bit covers this many stripes. */
#define EC_WIB_REGION_STRIPES   1024u

/* Region idle threshold before the region bit is cleared on disk (ms). */
#define EC_WIB_IDLE_MS          500u

/* Poller period for the WIB region-clear scan (us). */
#define EC_WIB_POLL_PERIOD_US   100000u   /* 100 ms */

/* Magic number in each on-disk WIB copy header. */
#define EC_WIB_MAGIC   UINT64_C(0x45432057494230)   /* "EC WIB0" */

/*
 * On-disk WIB header version. ec_wib_validate_buf rejects any non-matching
 * version, so a stale blob fails the load path loudly instead of being read
 * against the wrong layout.
 */
#define EC_WIB_VERSION 1u

/* On-disk WIB copy header; region_bits[] follows immediately after. */
struct ec_wib_header {
	uint64_t magic;
	uint32_t version;
	uint32_t generation;   /* only ever increases across persist calls */
	uint32_t num_regions;
	uint32_t _pad;
	/* uint64_t region_bits[] follows in the DMA buffer */
};

/* =========================================================================
 * In-band unmapped bitmap
 *
 * The persistent per-stripe unmapped bitmap is stored in-band and
 * raw-replicated: every base disk reserves two slots (copy A / copy B)
 * at the front of its address space, each holding an identical copy of
 * the whole bitmap blob. Crash-safety is double-buffer + CRC, not EC
 * redundancy; the blob survives n-1 disk loss. A persist writes the
 * inactive slot on every online disk and flips the global active-copy
 * index; load scans all 2n {disk, slot} copies and picks the
 * max-generation CRC-valid copy. See bdev_ec_bitmap.c.
 * ========================================================================= */

/* Magic in each on-disk bitmap slot header. */
#define EC_BITMAP_MAGIC   UINT64_C(0x4543554d41500000)   /* "ECUMAP\0\0" */

/*
 * On-disk bitmap slot header version. ec_bitmap_validate_buf rejects any
 * non-matching version.
 */
#define EC_BITMAP_VERSION 1u

/*
 * On-disk header at the front of every bitmap slot. The bitmap payload
 * (uint64_t words) follows immediately; a uint32_t crc32c sits at the
 * end of the blob and covers [header_start, header_start + blob_bytes).
 *
 * blob_bytes is the explicit length of the CRC-covered region (header +
 * span). The CRC trailer lives at offset blob_bytes; total on-disk
 * extent of one slot is blob_bytes + sizeof(uint32_t). Carrying
 * blob_bytes explicitly -- rather than deriving it from num_stripes --
 * is forward-compat: a later format revision can append fields at the
 * tail and an older reader can still CRC-validate the prefix it
 * understands.
 *
 * num_stripes records the user-stripe count the span covers at persist
 * time. Load rejects a slot whose num_stripes disagrees with the
 * volume's current geometry.
 */
struct ec_bitmap_header {
	uint64_t magic;
	uint32_t version;
	uint32_t generation;    /* only ever increases across persists           */
	uint64_t blob_bytes;    /* explicit length of header + span (CRC follows)*/
	uint64_t num_stripes;   /* user-stripe count the span covers             */
};

/* =========================================================================
 * Startup scrub context
 *
 * On startup, dirty WIB regions (mid-RMW at crash) are scrubbed:
 * re-encode parity from data chunks, then clear the region bit.
 * RMW writes to a scrubbing region return NOMEM (requeue).
 * ========================================================================= */

/* One instance during startup if any WIB regions were dirty. */
struct ec_scrub_ctx {
	struct ec_bdev  *ec;

	/* Scrub cursor: region/stripe currently being scrubbed. */
	uint32_t         current_region;   /* region currently being scrubbed */
	uint64_t         current_stripe;   /* stripe within current_region    */
	uint64_t         region_end_stripe;/* exclusive end stripe (clamped to num_stripes) */

	/* I/O state -- same gate pattern as rebuild */
	bool             io_in_flight;
	uint32_t         reads_remaining;
	enum spdk_bdev_io_status io_status;

	/* Dedicated per-disk I/O channels; NULL for FAILED slots. */
	struct spdk_io_channel *scrub_chans[EC_MAX_BASE_BDEVS];

	/* DMA buffers: k data + m parity, reused per stripe */
	uint8_t         *chunk_bufs[EC_MAX_BASE_BDEVS];
	struct iovec     chunk_iovs[EC_MAX_BASE_BDEVS];
	uint64_t         chunk_bytes;

	/* write half: only parity slots */
	uint32_t         writes_remaining;

	uint64_t         stripes_scrubbed;
	uint64_t         regions_scrubbed;
	uint32_t         total_dirty_regions; /* dirty count at scrub start */

	/* Heartbeat progress logging (see EC_REBUILD_HEARTBEAT_*). */
	uint64_t         start_ticks;          /* ticks when scrub was registered */
	uint64_t         last_heartbeat_ticks; /* ticks at last heartbeat NOTICE  */
	uint32_t         next_heartbeat_percent;   /* next percent milestone          */

	struct spdk_poller *poller;
};

/* =========================================================================
 * In-place resize context
 * ========================================================================= */

/*
 * One instance per ec_bdev_resize() call. Freed in ec_resize_finish().
 * Updates blockcnt / num_stripes / WIB region arrays / per-stripe bitmaps.
 * No data movement, no parity recomputation, no encode table changes,
 * and no on-disk WIB relocation -- the WIB sits at a fixed front offset
 * that is a pure function of strip_size and the bitmap reservation.
 */
struct ec_resize_ctx {
	struct ec_bdev     *ec;
	ec_resize_cb_fn     cb_fn;
	void               *cb_arg;

	/* New geometry values (computed at entry, applied under quiesce). */
	uint64_t            new_blockcnt;
	uint64_t            new_num_stripes;
	uint64_t            old_blockcnt;
};

/* =========================================================================
 * ec_bdev -- main EC bdev instance
 * ========================================================================= */

/*
 * Represents an Erasure Coded virtual block device.
 * Maps logical addresses across k data disks and m parity disks.
 */
struct ec_rmw_ctx;  /* defined later in this header */
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

/*
 * struct ec_bdev_io
 *
 * Per-I/O context stored in bdev_io->driver_ctx.
 */
struct ec_bdev_io {
	struct spdk_bdev_io *bdev_io;
	struct ec_io_channel *ch;

	uint64_t offset_blocks;
	uint64_t num_blocks;
	struct iovec *iovs;
	int iovcnt;

	/*
	 * Zero-fill semantics. Set by EC's own UNMAP and zero-fill-range
	 * paths (bdev_ec_unmap.c, ec_submit_rmw_zero_fill_range); native
	 * WRITE_ZEROES is not advertised (see ec_io_type_supported), so it
	 * never reaches here. The full-stripe and RMW modify steps skip the
	 * iov-copy and rely on the zero-initialised DMA buffers / a memset of
	 * the modified region. iovs/iovcnt are unused on the zero-fill path.
	 */
	bool     is_zero_fill;

	uint32_t base_io_remaining;   /* outstanding child base I/Os; parent completes at 0 */
	enum spdk_bdev_io_status status;

	/*
	 * One contiguous, DMA-aligned buffer holding a full stripe's data. The
	 * caller's scattered write iovs are gathered into it so each of the k
	 * data chunks is a contiguous slice -- what ISA-L encode and the base
	 * writes need.
	 */
	void *bounce_buf;
	struct iovec *data_iovs;
	struct iovec *parity_iovs;
	void **parity_bufs;

	/*
	 * Stripe-dirty claim release info. Set by ec_submit_full_write when
	 * it claims its stripe; cleared by ec_child_io_complete on final
	 * completion. Lets the shared completion path release the claim
	 * without re-deriving the stripe from offset_blocks.
	 *
	 * stripe_claimed = true means "release stripe_claim_index on
	 * completion." Other callers (read path) leave it false.
	 */
	bool     stripe_claimed;
	uint64_t stripe_claim_index;

	/*
	 * WIB region inflight tracking for full-stripe writes. Set by
	 * ec_submit_full_write after incrementing wib_region_inflight[];
	 * cleared by ec_child_io_complete on final completion, which
	 * decrements the counter. Required so the idle WIB poller does
	 * not clear and persist the region bit while child writes are
	 * still in flight (mirror of the same field on ec_rmw_ctx and
	 * ec_unmap_ctx). Other callers (read path, RMW path, UNMAP path)
	 * leave it false.
	 */
	bool     wib_inflight_held;
	uint32_t wib_region;

	/*
	 * Write-into-unmapped flag. Set by ec_submit_write_into_unmapped
	 * to mark this bdev_io as taking the post-completion bit-clear
	 * path: ec_child_io_complete intercepts at base_io_remaining == 0,
	 * submits a bit-clear via ec_submit_bit_clear_async, and defers
	 * bdev_io completion (and stripe-busy release / buffer free) until
	 * the bit-clear's persist acks at m+1. False on all other paths.
	 */
	bool     is_write_into_unmapped;
};

/* =========================================================================
 * Read-Modify-Write context
 *
 * Used for sub-stripe writes. Sequence:
 *   1. Set dirty bit for this stripe.
 *   2. Read k chunks from readable disks.
 *   3. If degraded, reconstruct unreadable data via ISA-L.
 *      Apply write payload, re-encode all m parity chunks.
 *   4. Write modified data + parity back to disk.
 *   5. Clear dirty bit, complete parent bdev_io.
 *
 * The struct is in this header because ec_wib_deferred_drain (in
 * bdev_ec_wib.c) walks ec->wib_deferred_writes via TAILQ_FOREACH_SAFE,
 * which needs the wib_defer_link field offset. All other access to
 * the context lives in bdev_ec_rmw.c.
 * ========================================================================= */

struct ec_rmw_ctx {
	struct ec_bdev_io       *ec_io;

	/*
	 * Effective payload geometry for this RMW. Populated at context
	 * setup from either ec_io->{offset_blocks,num_blocks,is_zero_fill}
	 * (the standard write path) or caller-supplied overrides
	 * (ec_submit_rmw_zero_fill_range, used by the multi-segment UNMAP
	 * dispatcher to zero-fill the partial-stripe head/tail fragments
	 * of an unaligned multi-stripe UNMAP). The async chain that follows
	 * reads num_blocks / is_zero_fill and the derived stripe fields
	 * below, never ec_io's copies, so the two callers route through the
	 * same RMW machinery. The source offset is consumed only at setup,
	 * to derive stripe_index / stripe_off_blocks.
	 */
	uint64_t                 num_blocks;
	bool                     is_zero_fill;

	/* Stripe index for this RMW (offset_blocks / stripe_blocks) */
	uint64_t                 stripe_index;

	/* Per-disk LBA for this stripe = ec_stripe_base_lba(ec, stripe_index) */
	uint64_t                 disk_lba;

	/*
	 * Offset within the stripe where the write payload begins, in blocks.
	 * stripe_off_blocks = offset_blocks % stripe_blocks
	 */
	uint64_t                 stripe_off_blocks;

	/*
	 * DMA-safe I/O buffers: [0..k-1] data, [k..n-1] parity.
	 * Allocated in ec_rmw_submit_core(); freed via ec_rmw_free_ctx()
	 * (from ec_rmw_complete or an ec_rmw_submit_core error path).
	 */
	uint8_t                 *chunk_bufs[EC_MAX_BASE_BDEVS];
	struct iovec             chunk_iovs[EC_MAX_BASE_BDEVS];

	/* Number of read completions still pending */
	uint32_t                 reads_remaining;

	/* Number of write completions still pending */
	uint32_t                 writes_remaining;

	/* Accumulated I/O status (FAILED if any child fails) */
	enum spdk_bdev_io_status status;

	/*
	 * Inclusive range of data chunks modified by this RMW (0..k-1).
	 * Derived at context creation from stripe_off_blocks and num_blocks.
	 *
	 * For ordinary WRITE this range is always one chunk -- SPDK's WRITE
	 * splitter is boundary-aware (split_on_optimal_io_boundary) and
	 * never lets a sub-stripe WRITE cross a strip boundary. The
	 * defensive check in ec_submit_rmw_write enforces that.
	 *
	 * For WRITE_ZEROES the range can span multiple chunks within one
	 * stripe, because SPDK's WRITE_ZEROES splitter (bdev_write_zeroes_split
	 * in lib/bdev/bdev.c) only caps size at max_write_zeroes and does
	 * not align to optimal_io_boundary. Handled in ec_rmw_submit_writes
	 * by writing back every chunk in [modified_chunk_first,
	 * modified_chunk_last] alongside the parity chunks.
	 */
	uint32_t                 modified_chunk_first;
	uint32_t                 modified_chunk_last;

	/* Linkage for deferred-write queue (wib_deferred_writes) */
	TAILQ_ENTRY(ec_rmw_ctx)  wib_defer_link;

	/*
	 * Optional completion override. When non-NULL, ec_rmw_complete
	 * invokes cb_fn(cb_arg, status) instead of completing the parent
	 * bdev_io directly. The standard write path leaves both NULL,
	 * preserving today's spdk_bdev_io_complete semantics; the
	 * multi-segment UNMAP dispatcher passes a coordinator callback so the
	 * parent bdev_io only completes once every segment has reported back.
	 */
	void                   (*cb_fn)(void *cb_arg,
					enum spdk_bdev_io_status status);
	void                    *cb_arg;
};

/* Global list type */
TAILQ_HEAD(ec_all_tailq, ec_bdev);
extern struct ec_all_tailq g_ec_bdev_list;

/* Function table for EC bdev operations */
extern const struct spdk_bdev_fn_table g_ec_fn_table;

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

/*
 * Stripe dirty bitmap helpers. One bit per stripe, set during in-flight
 * RMW. All operations O(1); no locking (single-threaded reactor).
 */
static inline void
ec_stripe_set_dirty(struct ec_bdev *ec, uint64_t stripe_index)
{
	ec->stripe_dirty_map[stripe_index / 64] |= (UINT64_C(1) << (stripe_index % 64));
}

static inline void
ec_stripe_clear_dirty(struct ec_bdev *ec, uint64_t stripe_index)
{
	ec->stripe_dirty_map[stripe_index / 64] &= ~(UINT64_C(1) << (stripe_index % 64));
}

static inline bool
ec_stripe_is_dirty(const struct ec_bdev *ec, uint64_t stripe_index)
{
	return !!(ec->stripe_dirty_map[stripe_index / 64] &
		  (UINT64_C(1) << (stripe_index % 64)));
}

/*
 * Stripe unmapped bitmap helpers. One bit per user stripe, 1 = the
 * stripe is logically zero (unmapped). Same single-thread O(1)
 * discipline and same user-stripe indexing as the dirty bitmap.
 */
static inline void
ec_stripe_set_unmapped(struct ec_bdev *ec, uint64_t stripe_index)
{
	ec->stripe_unmapped_map[stripe_index / 64] |= (UINT64_C(1) << (stripe_index % 64));
}

static inline void
ec_stripe_clear_unmapped(struct ec_bdev *ec, uint64_t stripe_index)
{
	ec->stripe_unmapped_map[stripe_index / 64] &= ~(UINT64_C(1) << (stripe_index % 64));
}

static inline bool
ec_stripe_is_unmapped(const struct ec_bdev *ec, uint64_t stripe_index)
{
	return !!(ec->stripe_unmapped_map[stripe_index / 64] &
		  (UINT64_C(1) << (stripe_index % 64)));
}

/*
 * Word-level bit ops on a standalone bitmap (a plain uint64_t[] array, not one
 * of the ec->stripe_* maps). The UNMAP staging shadow (uctx->staged_map) and
 * the bit-clear shadow (ec->clear_staged_map) are separate arrays that the
 * ec_stripe_*_unmapped helpers cannot target, so they use these rather than
 * re-deriving the word/bit split inline. Keeps the 64-bit arithmetic in one
 * place alongside EC_BITMAP_WORDS.
 */
static inline void
ec_bitmap_word_set(uint64_t *map, uint64_t idx)
{
	map[idx / 64] |= (UINT64_C(1) << (idx % 64));
}

static inline void
ec_bitmap_word_clear(uint64_t *map, uint64_t idx)
{
	map[idx / 64] &= ~(UINT64_C(1) << (idx % 64));
}

/* WIB region dirty-bit helpers. Same single-thread O(1) discipline. */
static inline uint32_t
ec_wib_stripe_to_region(uint64_t stripe_index)
{
	return (uint32_t)(stripe_index / EC_WIB_REGION_STRIPES);
}

static inline bool
ec_wib_region_is_dirty(const struct ec_bdev *ec, uint32_t region)
{
	return !!(ec->wib_region_map[region / 64] &
		  (UINT64_C(1) << (region % 64)));
}

static inline void
ec_wib_region_set_dirty(struct ec_bdev *ec, uint32_t region)
{
	ec->wib_region_map[region / 64] |= (UINT64_C(1) << (region % 64));
}

static inline void
ec_wib_region_clear_dirty(struct ec_bdev *ec, uint32_t region)
{
	ec->wib_region_map[region / 64] &= ~(UINT64_C(1) << (region % 64));
}

/*
 * Bracket the in-flight write counter for a single WIB region. Two
 * paths inc it: ec_submit_rmw_write (sub-stripe RMW) and
 * ec_submit_full_write (full-stripe write). Each inc must be
 * balanced by exactly one dec -- for RMW, in ec_rmw_complete or the
 * synchronous read-submit failure cleanup; for full-stripe writes,
 * in ec_child_io_complete via ec_bdev_io.wib_inflight_held. The dec
 * guards against underflow defensively; underflow indicates a
 * balance bug higher up the chain.
 */
static inline void
ec_wib_region_inflight_inc(struct ec_bdev *ec, uint32_t region)
{
	ec->wib_region_inflight[region]++;
}

static inline void
ec_wib_region_inflight_dec(struct ec_bdev *ec, uint32_t region)
{
	if (ec->wib_region_inflight[region] > 0) {
		ec->wib_region_inflight[region]--;
	} else {
		SPDK_ERRLOG("EC bdev %s: wib_region_inflight[%u] underflow\n",
			    ec->bdev.name, region);
	}
}

/* =========================================================================
 * Cross-file internal API
 *
 * Non-inline functions defined in one .c file of the module and called
 * from another. Static helpers used by only one .c file stay in that file.
 * ========================================================================= */

/* Helpers for diagnostics; loop-based, not worth inlining. */
uint32_t ec_wib_count_dirty(const struct ec_bdev *ec);

/* =========================================================================
 * In-band unmapped bitmap (bdev_ec_bitmap.c)
 *
 * Geometry math and whole-blob serialize / validate / apply for the
 * persistent per-stripe unmapped bitmap. The blob is raw-replicated to
 * every disk and double-buffered; the async persist and load chains
 * build on these helpers. See bdev_ec_bitmap.c for the on-disk layout.
 * ========================================================================= */

/*
 * ec_bitmap_blob_bytes -- length of the CRC-covered region (header +
 * span) for a bitmap blob covering num_stripes user stripes. The CRC32C
 * trailer lives at offset blob_bytes; total on-disk slot extent is
 * blob_bytes + sizeof(uint32_t).
 */
uint64_t ec_bitmap_blob_bytes(uint64_t num_stripes);

uint64_t ec_bitmap_reservation_stripes(const struct ec_bdev *ec);

/*
 * ec_bitmap_fill_buf -- serialize source_map into buf as a complete
 * on-disk blob (header + span + CRC32C). source_map must hold at least
 * EC_BITMAP_WORDS(ec->num_stripes) uint64_t words; buf must hold at
 * least ec_bitmap_blob_bytes(ec->num_stripes) + sizeof(uint32_t) bytes
 * and must be zero-initialised on entry (the trailing slack past the
 * CRC, if any, is left untouched).
 *
 * source_map is explicit (not always ec->stripe_unmapped_map) so a
 * later caller can persist a staged copy of the map -- the foundation
 * for pessimistic visibility in the UNMAP path. The create-time
 * bootstrap persist simply passes ec->stripe_unmapped_map.
 */
void ec_bitmap_fill_buf(struct ec_bdev *ec, const uint64_t *source_map,
			uint32_t generation, void *buf);

/*
 * ec_bitmap_validate_buf -- check a slot read back from disk: magic,
 * version, num_stripes match the current volume geometry, blob_bytes
 * is exactly the expected length for that geometry, and the CRC32C
 * over [start, start + blob_bytes) matches the trailer at offset
 * blob_bytes. Returns 0 and fills *gen_out on success, -EINVAL on any
 * mismatch -- which is also what a never-written or torn slot looks
 * like.
 */
int  ec_bitmap_validate_buf(const struct ec_bdev *ec, const void *buf,
			    uint32_t *gen_out);

/*
 * ec_bitmap_apply_buf -- copy the span out of a validated blob into
 * ec->stripe_unmapped_map. Call only after ec_bitmap_validate_buf has
 * returned 0 for this buffer.
 */
void ec_bitmap_apply_buf(struct ec_bdev *ec, const void *buf);

/*
 * Count of set bits in ec->stripe_unmapped_map. Linear popcount over
 * the live map; cheap enough to call from the JSON-dump paths
 * (bdev_get_bdevs, bdev_ec_get_bdevs, bdev_ec_get_unmap_status).
 * Returns 0 if the map has not yet been allocated.
 */
uint64_t ec_count_unmapped_stripes(const struct ec_bdev *ec);

/*
 * ec_bitmap_persist_async -- write source_map to disk as a fresh
 * generation of the bitmap blob. Bumps ec->bitmap_generation, picks
 * next_copy = 1 - bitmap_active_copy, fills a DMA scratch buffer from
 * source_map at the new generation, then fans the write out to every
 * online writable disk's next_copy slot.
 *
 * Two completion callbacks, each optional (pass NULL to skip):
 *
 *   cb_durable -- fires the moment durability is achieved: at least
 *     m+1 disks (or all online disks, if fewer than m+1 are online)
 *     have completed the write. At this point bitmap_active_copy is
 *     flipped to next_copy. The remaining in-flight writes continue
 *     to drain in the background; bitmap_persist_in_flight is NOT yet
 *     cleared. Used by the UNMAP path to release its caller before
 *     a slow disk has finished writing -- the staged bits are
 *     applied to the live bitmap from this callback.
 *
 *     If the m+1 threshold is never reached, cb_durable is called
 *     with rc < 0 once all in-flight writes have completed (and
 *     bitmap_active_copy is not flipped -- the bitmap stays on the
 *     prior generation).
 *
 *   cb_drained -- fires when the last write has completed, after
 *     bitmap_persist_in_flight has been cleared. The rc argument
 *     reflects the final outcome (0 if durability was achieved,
 *     negative errno otherwise). Used by callers that need to
 *     serialise work behind the persist's full drainout -- the
 *     fresh-create bootstrap chains a second persist (to the other
 *     slot) from this callback, since starting it earlier would
 *     race the first persist's stragglers writing to the same slot.
 *
 * source_map is taken explicitly (not read from ec->stripe_unmapped_map)
 * so the UNMAP path can persist a staged copy of the map without first
 * applying the staged changes to the live, reader-visible map.
 *
 * Returns 0 if the async chain started, or a negative errno on setup
 * failure (-ENOMEM, -EBUSY if a persist is already in flight, or -EIO
 * if no writable disk could be issued a write). Neither callback is
 * invoked on synchronous failure.
 *
 * Precondition: ec->bitmap_persist_in_flight must be false on entry.
 */
int ec_bitmap_persist_async(struct ec_bdev *ec, const uint64_t *source_map,
			    ec_bitmap_persist_cb_fn cb_durable, void *cb_durable_arg,
			    ec_bitmap_persist_cb_fn cb_drained, void *cb_drained_arg);

/*
 * ec_bitmap_persist_both_copies -- overwrite BOTH bitmap copies (and both commit
 * copies) on every writable disk with the current map, via two drain-gated
 * persists. Used at fresh create and after a hot-swap so a stale/foreign blob
 * on a reused base bdev cannot out-rank ours on a later load. done_fn (may be
 * NULL) fires with the final rc. Home-thread only.
 */
int ec_bitmap_persist_both_copies(struct ec_bdev *ec,
			       ec_bitmap_persist_cb_fn done_fn, void *done_arg);

/*
 * ec_bitmap_load_async -- read the bitmap blob from disk at startup
 * (or after a process restart) and apply the max-generation
 * CRC-validated copy into stripe_unmapped_map.
 *
 * Reads all 2n {disk, slot} copies serially (one in flight at a time),
 * validates each, and applies the highest-generation valid one.
 * bitmap_generation and bitmap_active_copy are set from that copy's
 * header. If no copy
 * validates -- never-written region, all-disks-down, or a torn first
 * persist -- stripe_unmapped_map is left zeroed and the create path
 * decides what that means based on salvage_requested.
 *
 * done_fn is always invoked with rc == 0; load failure is non-fatal at
 * this layer. Consumes ec->bitmap_chans[], which must already be open.
 */
void ec_bitmap_load_async(struct ec_bdev *ec,
			  ec_bdev_create_cb_fn done_fn, void *done_arg);

/*
 * Per-stripe bit-clear waiter. One entry per call to
 * ec_submit_bit_clear_async. Lives on either ec->pending_bit_clears
 * (queued, not yet persisting) or ec->in_flight_bit_clears (included
 * in the current persist).
 */
struct ec_pending_bit_clear {
	/*
	 * Back-pointer to the owning EC bdev. The waiter is allocated on the
	 * submitter thread and routed to the home thread via
	 * spdk_thread_send_msg, which only passes one void argument; this
	 * lets the home-thread handler recover ec.
	 */
	struct ec_bdev                     *ec;
	uint64_t                            stripe_index;
	void                              (*cb_fn)(void *cb_arg, int rc);
	void                               *cb_arg;
	TAILQ_ENTRY(ec_pending_bit_clear)   link;
};

/*
 * ec_submit_bit_clear_async -- request that one stripe's unmapped bit be
 * cleared and the bitmap re-persisted. Used by the write-into-unmapped
 * path AFTER all k+m chunk writes for the stripe have landed.
 *
 * Ordering invariant (load-bearing -- see write-into-unmapped helper):
 *   1. Data must be on disk on at least the k+m chunks.
 *   2. THEN call this function.
 *   3. cb_fn fires only after the bit-clear persist acks at m+1.
 *   4. Caller acks its bdev_io ONLY after cb_fn fires.
 *
 * Reordering -- clearing the bit before data lands -- opens a silent
 * corruption window. A crash between bit-clear-persist and data-land
 * leaves on-disk bitmap saying "mapped" while chunks are inconsistent
 * or absent, and reads return undefined bytes instead of the
 * synthesise-zero contract.
 *
 * Coalescing: if a bitmap persist is already in flight (UNMAP or a
 * prior bit-clear), this call queues onto pending_bit_clears and
 * returns 0 (async). On that persist's drainout, a single follow-up
 * persist fires for ALL queued bits at once -- worst-case post-trim
 * write throughput is capped at one extra persist per roundtrip rather
 * than one persist per write.
 *
 * Returns 0 on enqueue. cb_fn fires later with rc == 0 on success or
 * a negative errno on persist failure (caller should fail the bdev_io
 * and release stripe-busy).
 *
 * Single-reactor model: caller and callback run on the same thread; no
 * locking around the queues.
 */
int ec_submit_bit_clear_async(struct ec_bdev *ec, uint64_t stripe_index,
			      void (*cb_fn)(void *cb_arg, int rc),
			      void *cb_arg);

/*
 * Internal: invoked from ec_bitmap_persist_write_cb after
 * bitmap_persist_in_flight is cleared. If the bit-clear pending queue is
 * non-empty, kicks a new persist covering all queued bits. Not for
 * caller use.
 */
void ec_bit_clear_flush_if_pending(struct ec_bdev *ec);

/* Reconstruction (ISA-L wrappers). Used by io, rmw, and rebuild paths. */
int ec_reconstruct_data_chunk(const struct ec_bdev *ec,
			      uint8_t *src_bufs[EC_MAX_BASE_BDEVS],
			      uint8_t *out_buf, uint32_t failed_slot,
			      uint64_t chunk_len);
int ec_reconstruct_multi_data(const struct ec_bdev *ec,
			      uint8_t *src_bufs[EC_MAX_BASE_BDEVS],
			      uint8_t *out_bufs[],
			      const uint32_t failed_data_slots[],
			      uint32_t f, uint64_t chunk_len);

/* =========================================================================
 * WIB <-> RMW protocol
 *
 * RMW and WIB live in separate files (bdev_ec_rmw.c and bdev_ec_wib.c)
 * but share state in struct ec_bdev: wib_persist_in_flight,
 * wib_repersist_needed, wib_deferred_writes, and the wib_region_*
 * arrays. Two function calls bridge them:
 *
 *   bdev_ec_rmw.c -> bdev_ec_wib.c
 *     ec_rmw_persist_and_submit calls ec_wib_persist to set a region's
 *     on-disk dirty bit before the data + parity writes go out. When
 *     a persist is already in flight, the RMW context is queued on
 *     wib_deferred_writes and wib_repersist_needed is set.
 *
 *   bdev_ec_wib.c -> bdev_ec_rmw.c
 *     ec_wib_persist_write_cb, on the final write completion, drains
 *     wib_deferred_writes via ec_wib_deferred_drain, which calls
 *     ec_rmw_submit_writes for each queued context.
 *
 * Both bridge functions (ec_wib_persist and ec_rmw_submit_writes) are
 * declared in the section immediately below.
 * ========================================================================= */

/*
 * LBA of WIB copy 0 or 1 on a parity disk.
 *
 * Per-disk layout:
 *   [ bitmap region: bitmap_reservation_stripes strips ]
 *   [ WIB copy 0:    1 strip                           ]
 *   [ WIB copy 1:    1 strip                           ]
 *   [ user data:     num_stripes strips                ]
 *
 * Pure arithmetic over strip_size and the bitmap reservation, both
 * fixed-max at create time -- no I/O, no descriptor lookups.
 */
static inline uint64_t
ec_wib_lba(const struct ec_bdev *ec, uint8_t copy)
{
	return (ec_bitmap_reservation_stripes(ec) + copy) *
	       (uint64_t)ec->strip_size;
}

/* WIB helpers needed by the resize and scrub chains, plus the
 * RMW->WIB bridge ec_wib_persist documented above. */
int      ec_wib_persist(struct ec_bdev *ec,
			void (*cb)(void *cb_arg, int rc), void *cb_arg);
int      ec_wib_idle_poller_cb(void *arg);
void     ec_wib_load_async(struct ec_bdev *ec,
			   ec_bdev_create_cb_fn done_fn, void *done_arg);

/* WIB->RMW bridge: drained by ec_wib_deferred_drain. See the
 * WIB <-> RMW protocol section above. */
void     ec_rmw_submit_writes(struct ec_rmw_ctx *mctx);

/*
 * Complete a deferred RMW with mctx->status (set by the caller) and tear
 * down its resources (stripe-busy claim, WIB region in-flight counter,
 * DMA buffers, ctx). Called from ec_wib_deferred_drain when a follow-up
 * WIB persist fails, so the deferred RMWs are returned to SPDK via
 * SPDK_BDEV_IO_STATUS_NOMEM rather than having their data writes go out
 * with no durable write-intent (write-hole).
 */
void     ec_rmw_complete(struct ec_rmw_ctx *mctx);

/* Startup scrub kick-off, called from create-done and rebuild_finish. */
int ec_bdev_start_scrub(struct ec_bdev *ec);

/*
 * Backpressure transition logger. Called when the underlying condition
 * that caused RMW deferral ends (scrub finish, scrub start after defer,
 * rebuild restoring failed slots). Logs the "cleared" NOTICE and resets
 * the active flag; no-op if backpressure was not active.
 */
void ec_rmw_backpressure_end(struct ec_bdev *ec, const char *reason);

/*
 * Free per-scrub resources (channels, buffers) on a still-active
 * scrub_ctx. Used by the unregister path to abort an in-flight scrub
 * after the device-unregister channel walk has completed.
 */
void ec_scrub_free_resources(struct ec_scrub_ctx *sctx);

/*
 * Submit a sub-stripe RMW write. Called by ec_submit_write to handle
 * any write that doesn't cover one or more full stripes aligned.
 */
int ec_submit_rmw_write(struct ec_bdev_io *ec_io);

/*
 * Submit a sub-stripe RMW zero-fill on an explicit (offset, length)
 * range within a single stripe, with caller-supplied completion. The
 * range must satisfy num_blocks > 0, num_blocks <= ec->stripe_blocks,
 * and (offset_blocks, offset_blocks + num_blocks) entirely within one
 * stripe (i.e., the same stripe_index for both endpoints).
 *
 * The ec_io's offset_blocks / num_blocks / is_zero_fill / iovs are
 * NOT read; this entry point lets the multi-segment UNMAP dispatcher
 * zero-fill the partial-stripe head and tail fragments of an
 * unaligned multi-stripe UNMAP without synthesizing a child bdev_io.
 * ec_io is still used for the I/O channel (ec_io->ch).
 *
 * cb_fn is invoked exactly once with the aggregated status when the
 * RMW completes. On non-zero return (-EAGAIN / -ENOMEM / -EINVAL) the
 * cb_fn is NOT invoked -- the caller owns completion accounting.
 */
int ec_submit_rmw_zero_fill_range(struct ec_bdev_io *ec_io,
				  uint64_t offset_blocks,
				  uint64_t num_blocks,
				  void (*cb_fn)(void *cb_arg,
						enum spdk_bdev_io_status status),
				  void *cb_arg);

/*
 * I/O entry points defined in bdev_ec_io.c and dispatched from
 * ec_submit_request in bdev_ec.c.
 */
void ec_bdev_io_init(struct ec_bdev_io *ec_io, struct ec_io_channel *ch,
		     struct spdk_bdev_io *bdev_io);
int  ec_submit_read(struct ec_bdev_io *ec_io);
int  ec_submit_write(struct ec_bdev_io *ec_io);

/*
 * Native UNMAP entry point. Defined in bdev_ec_unmap.c. UNMAP is
 * unconditionally advertised by ec_io_type_supported because
 * correctness is internal to bdev_ec (the in-band unmapped bitmap
 * synthesizes zeros on read regardless of what the bases do on
 * discard). Returns 0 on async submit, -EAGAIN for NOMEM requeue
 * (busy stripe / bitmap persist in flight), -EINVAL for shapes the
 * path cannot handle, -ENOMEM on alloc failure.
 */
int  ec_submit_unmap(struct ec_bdev_io *ec_io);

#endif /* SPDK_BDEV_EC_INTERNAL_H */
