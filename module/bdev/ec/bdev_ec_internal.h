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

/* ISA-L gf_gen_decode_matrix() requires (k * m * 32) bytes for the
 * encode / decode tables. */
#define EC_ISAL_GF_TABLE_BYTES 32u

/*
 * ISA-L ec_init_tables requires 32 bytes per (k * f) entry to build
 * the GF lookup tables. Used to size the on-stack decode_tbls[] scratch
 * for both single- and multi-failure reconstruction paths.
 */
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
 * Write-Intent Bitmap (WIB)
 *
 * Protects against the RMW write-hole. One dirty bit per region of
 * EC_WIB_REGION_STRIPES stripes, stored on-disk as two alternating
 * front-placed copies on every parity disk, immediately after the
 * unmapped-bitmap reservation. See bdev_ec_wib.c for the on-disk layout
 * and persist protocol.
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
	/*
	 * Inline scratch for full-stripe writes. Bounded by k+m
	 * <= EC_MAX_BASE_BDEVS, so each ec_bdev_io carries its own slots
	 * without three per-submission callocs (parity bdev pointers come
	 * from ec_alloc_full_stripe, the iovs are filled from the bounce
	 * buffer). Read and RMW paths leave these unused.
	 */
	struct iovec data_iovs[EC_MAX_BASE_BDEVS];
	struct iovec parity_iovs[EC_MAX_BASE_BDEVS];
	void        *parity_bufs[EC_MAX_BASE_BDEVS];

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
	/*
	 * Two alternating front-placed copies. At this commit there is no
	 * other front-placed reservation, so copy 0 starts at LBA 0 and
	 * copy 1 starts one strip later. Once the in-band unmapped bitmap
	 * lands (c19), this offset shifts behind it via
	 * ec_bitmap_reservation_stripes(), but the two-copies-back-to-back
	 * layout is preserved.
	 */
	return ((uint64_t)copy) * ec->strip_size;
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

/*
 * Submit a sub-stripe RMW write. Called by ec_submit_write to handle
 * any write that doesn't cover one or more full stripes aligned.
 */
int ec_submit_rmw_write(struct ec_bdev_io *ec_io);

/*
 * I/O entry points defined in bdev_ec_io.c and dispatched from
 * ec_submit_request in bdev_ec.c.
 */
void ec_bdev_io_init(struct ec_bdev_io *ec_io, struct ec_io_channel *ch,
		     struct spdk_bdev_io *bdev_io);
int  ec_submit_read(struct ec_bdev_io *ec_io);
int  ec_submit_write(struct ec_bdev_io *ec_io);

#endif /* SPDK_BDEV_EC_INTERNAL_H */
