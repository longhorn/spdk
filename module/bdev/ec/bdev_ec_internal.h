/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_internal.h -- internal definitions for the EC bdev module.
 *
 * Include only from the module's .c files. Anything that would be
 * appropriate for an external consumer belongs in bdev_ec.h instead.
 * See the glossary at the top of bdev_ec.h for acronyms (WIB, RMW,
 * UNMAP) and the home / submitter thread terminology used throughout
 * this header.
 */

/* =========================================================================
 * THREADING MODEL
 *
 * Two thread roles bracket every I/O path: home and submitter.
 *
 *   - home is the SPDK thread on which the EC bdev was created. It owns
 *     mutating access to shared ec_bdev state (rebuild/scrub/resize
 *     contexts, persist orchestration, busy gates, the bitmap mutation
 *     queues) and to the long-lived dedicated channels (wib_chans,
 *     bitmap_chans, rebuild_chans, scrub_chans). RPC handlers, pollers,
 *     and persist completion callbacks all run here.
 *
 *   - submitter is the SPDK thread on which a given bdev_io was issued
 *     by the consumer (typically an NVMe-oF poll-group thread). It owns
 *     the per-thread ec_io_channel.base_chans[] used to dispatch the
 *     child reads / writes / unmaps to base bdevs. Completions for those
 *     child I/Os land back on the submitter, since SPDK delivers bdev_io
 *     completions on the channel's owning thread.
 *
 * When the caller is already on the target thread, the routing helper
 * resolves inline; otherwise the data plane uses spdk_thread_send_msg
 * to hop between submitter and home. Entry points
 * (ec_submit_read / ec_submit_write / ec_submit_unmap) route to home
 * before touching shared state; fan-out dispatchers route to the
 * submitter so base I/O is issued on the channel-owning thread; final
 * spdk_bdev_io_complete asserts the submitter thread and is reached
 * via a shared owner-route helper (see ec_io_complete_status_on_submitter).
 *
 * Fields shared across the boundary are tagged with their access
 * discipline at the point of declaration: relaxed atomic for counters
 * that are only ever incremented, release/acquire CAS for bitmap-style
 * words where a reader on the submitter must observe state set by the
 * mutator on home. Any new shared field MUST follow one of those
 * patterns -- never plain non-atomic access.
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
#include "spdk/assert.h"
#include "spdk/bdev.h"
#include "spdk/bdev_module.h"
#include "spdk/env.h"
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

/* Background-walk poller period (us); shared by rebuild and scrub. */
#define EC_BG_POLL_PERIOD_US 100

/* Background-walk heartbeat: emit a NOTICE every N seconds or every M
 * percent, whichever fires first. */
#define EC_BG_HEARTBEAT_SEC 30
#define EC_BG_HEARTBEAT_PERCENT_STEP 10

/* Heartbeat state for a background walk; embedded in ec_rebuild_ctx and
 * ec_scrub_ctx. Each caller owns its own NOTICE format. */
struct ec_bg_heartbeat_state {
	uint64_t  start_ticks;            /* ticks when the walk was registered  */
	uint64_t  last_heartbeat_ticks;   /* ticks at the last heartbeat NOTICE  */
	uint32_t  next_heartbeat_percent; /* next percent milestone (10, 20, ...) */
};

/* Heartbeat decision returned by ec_heartbeat_should_fire. */
struct ec_bg_heartbeat_decision {
	bool      fire;
	uint32_t  percent;
	uint64_t  elapsed_seconds;
	uint64_t  remaining_seconds;
};

/*
 * Decide whether the walk should emit a heartbeat NOTICE; advance state
 * when it should. progress/total are walk counts (stripes for rebuild,
 * regions for scrub).
 *
 * Returns fire == false when total == 0, the tick rate is unknown, or
 * percent >= 100 (the matching "complete" NOTICE in *_finish owns that
 * case).
 */
static inline struct ec_bg_heartbeat_decision
ec_heartbeat_should_fire(struct ec_bg_heartbeat_state *state,
			 uint64_t progress, uint64_t total)
{
	struct ec_bg_heartbeat_decision out = { 0 };
	uint64_t ticks_per_second = spdk_get_ticks_hz();
	uint64_t now;
	bool     by_time;
	bool     by_percent;

	if (total == 0 || ticks_per_second == 0) {
		return out;
	}

	out.percent = (uint32_t)((progress * 100) / total);

	/* *_finish logs 100% on its own; skip the heartbeat at the boundary
	 * so we don't log completion twice. */
	if (out.percent >= 100) {
		return out;
	}

	now = spdk_get_ticks();
	by_time = (now - state->last_heartbeat_ticks) >=
		  (uint64_t)EC_BG_HEARTBEAT_SEC * ticks_per_second;
	by_percent = out.percent >= state->next_heartbeat_percent;

	if (!by_time && !by_percent) {
		return out;
	}

	out.elapsed_seconds = (now - state->start_ticks) / ticks_per_second;
	/* percent is in [1, 99] here: >= 100 returned above, == 0 guarded by ?: */
	out.remaining_seconds = (out.percent > 0) ?
		(out.elapsed_seconds * (100 - out.percent) / out.percent) : 0;
	out.fire = true;

	state->last_heartbeat_ticks = now;
	if (by_percent) {
		state->next_heartbeat_percent =
			(out.percent / EC_BG_HEARTBEAT_PERCENT_STEP + 1) *
			EC_BG_HEARTBEAT_PERCENT_STEP;
	}

	return out;
}

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
 * Mutated only on the home thread (poller, RPC handlers, async completion
 * callbacks for the rebuild_chans[] I/O channels which the home thread
 * owns); no locking needed. One stripe per poller tick: reads ->
 * reconstruct -> write.
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

	/* of those, data-slot stripes rebuilt in a crash-dirty region --
	 * write-hole suspects; see ec->crash_dirty_stripes_rebuilt. */
	uint64_t         crash_dirty_stripes;

	/* REPLACING slot count at rebuild start; for percent_complete calc. */
	uint32_t         slots_to_rebuild;

	struct ec_bg_heartbeat_state heartbeat;

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
	uint64_t generation;   /* only ever increases across persist calls */
	uint32_t version;
	uint32_t num_regions;
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
	uint64_t generation;    /* only ever increases across persists           */
	uint64_t blob_bytes;    /* explicit length of header + span (CRC follows)*/
	uint64_t num_stripes;   /* user-stripe count the span covers             */
	uint32_t version;
	uint32_t reserved;      /* zeroed; aligns span[] to an 8-byte (uint64_t) boundary */
};

/* Magic in the on-disk bitmap commit record. */
#define EC_BITMAP_COMMIT_MAGIC   UINT64_C(0x4543434f4d4d4954)   /* "ECCOMMIT" */

/*
 * The commit record's own format version, separate from EC_BITMAP_VERSION. Load
 * rejects a record whose version this build does not understand.
 */
#define EC_BITMAP_COMMIT_VERSION 1u

/*
 * On-disk commit record ("stamp") for the unmapped bitmap. It names the
 * bitmap generation that has been durably committed, letting load tell a
 * committed generation from a partially-written one. A uint32_t crc32c
 * trailer follows the struct.
 *
 * blob_crc is the CRC of the committed blob, so a record can only be paired
 * with the blob it commits.
 *
 * The CRC covers the whole struct, so every byte must belong to a field with
 * a known value -- no compiler padding. reserved is a uint64_t for that
 * reason: as a uint32_t the fields would total 28 bytes and the compiler
 * would pad the struct out to 32 to keep it 8-byte aligned, and those 4 pad
 * bytes (which no field sets) would still fall under the CRC. The wider
 * reserved field fills that space itself, so all 32 bytes are named and
 * zero-initialized. It also doubles as spare room for a future field.
 */
struct ec_bitmap_commit {
	uint64_t magic;
	uint64_t committed_gen;  /* the bitmap generation this record commits        */
	uint32_t blob_crc;       /* CRC32C of the committed blob                      */
	uint32_t version;
	uint64_t reserved;       /* zeroed spare */
};
SPDK_STATIC_ASSERT(sizeof(struct ec_bitmap_commit) == 32,
		   "on-disk commit record size must stay 32 bytes");

/*
 * Strips reserved for the commit record at the front of every disk, between
 * the unmapped-bitmap reservation and the WIB: one strip per double-buffer
 * copy.
 */
#define EC_BITMAP_COMMIT_STRIPS 2u

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

	struct ec_bg_heartbeat_state heartbeat;

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

	/*
	 * [shared] Bdev pointer per open slot, kept in lockstep with descs[].
	 * Lets completion threads map a bdev to a slot by pointer compare
	 * without touching descs[i], which home may be closing. Home writes
	 * with release (NULL before spdk_bdev_close); readers use acquire
	 * and never dereference the pointer.
	 */
	struct spdk_bdev *base_bdevs[EC_MAX_BASE_BDEVS];

	/* Failure Tracking */
	enum ec_base_bdev_state base_states[EC_MAX_BASE_BDEVS];

	uint32_t failed_count;         /* non-NORMAL slot count (FAILED + REPLACING) */
	bool offline;

	bool needs_rebuild[EC_MAX_BASE_BDEVS]; /* set on FAILED -> REPLACING */
	bool replace_in_progress;              /* serialise replace ops     */
	bool bdev_registered;                  /* spdk_bdev_register succeeded:
	                                        * gates spdk_bdev_destruct_done so
	                                        * create-failure teardown never
	                                        * completes a destruct that never
	                                        * started */
	/*
	 * True from register until the create RPC is answered (in
	 * ec_bdev_create_examine_done). The create chain owns the dedicated
	 * channels the whole time, so teardown defers and the delete RPC is
	 * rejected while it is set -- otherwise a racing delete or shutdown could
	 * release those channels or free ec mid-create.
	 */
	bool create_in_progress;
	/*
	 * Set first in ec_destruct. The create chain checks it at each step and
	 * aborts, rather than finishing a create onto a bdev being torn down.
	 */
	bool destructing;

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
	/*
	 * Regions loaded dirty at startup whose parity the scrub has not yet
	 * re-encoded. wib_region_map above also tracks runtime write-intent
	 * and is idle-cleared by the poller; this map is set only at
	 * load/salvage and cleared only by the scrub, so a crash region
	 * survives the poller until it is actually repaired.
	 *
	 * wib_region_map stays a superset of this map: the poller skips crash
	 * regions, the scrub clears both, and write rollbacks clear only the
	 * bit that write set. That lets the degraded-read guard read
	 * wib_region_map and ignore this map -- so never clear a live bit
	 * without checking the crash bit.
	 *
	 * Home-thread only, so plain access -- no atomics.
	 */
	uint64_t                *wib_crash_dirty_map;
	struct spdk_io_channel  *wib_chans[EC_MAX_BASE_BDEVS]; /* m entries */
	void                    *wib_buf;              /* DMA buf, one strip   */
	uint64_t                 wib_generation;
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
	 * Writes parked because their target stripe is busy or earlier
	 * writes for the same stripe are already parked. Home-thread only.
	 * Drained by ec_stripe_waitq_kick when a stripe is released; failed
	 * by ec_stripe_waitq_fail_all on reset, offline, and destruct.
	 *
	 * stripe_waitq_depth mirrors the queue length. Written on home;
	 * read atomically by ec_stripe_clear_dirty on submitter threads.
	 */
	TAILQ_HEAD(, ec_bdev_io) stripe_waitq;
	uint32_t                 stripe_waitq_depth;

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
	uint64_t                 bitmap_generation;
	uint8_t                  bitmap_active_copy;        /* 0 or 1                       */
	uint8_t                  bitmap_commit_active_copy; /* 0 or 1; latest stamp slot    */
	bool                     bitmap_persist_in_flight;
	/*
	 * A hot-swapped slot rejoined the persist quorum (bitmap_chans[]
	 * reopened in ec_replace_finish) but a both-copy persist is still owed --
	 * a persist was in flight when replace finished, or it failed.
	 * Drained on a later persist completion (ec_bitmap_persist_write_cb).
	 * Bitmap analog of wib_repersist_needed.
	 */
	bool                     bitmap_resync_pending;

	/*
	 * Deferred dedicated-channel teardown. A persist write to a failing
	 * disk can sit outstanding on wib_chans[] / bitmap_chans[] for the full
	 * NVMe-oF ctrlr-loss timeout, so a base-bdev REMOVE (or a delete) can
	 * arrive while that write is still in flight. Putting the channel then
	 * would trip the bdev-layer io_outstanding assert. These flags defer the
	 * release until the persist drains. The failure handler, the destruct
	 * callback, and both persist completions all run on home, so no locking
	 * is needed.
	 */
	bool                     dedicated_release_pending[EC_MAX_BASE_BDEVS];
	bool                     unregister_release_pending;

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
	 *
	 * Thread discipline (do not "fix" without checking): a counter touched
	 * only on the home thread uses a plain ++ and relies on the home-thread
	 * assert at its increment site; a counter touched from a submitter (or
	 * read cross-thread) uses __atomic_fetch_add / __atomic_load_n. Making a
	 * home-only counter atomic is harmless but needless; making a
	 * cross-thread counter plain is a data race.
	 */
	uint64_t degraded_read_eio_dirty;        /* [shared] reads rejected: dirty WIB region */
	uint64_t wib_failed_write_marks;         /* [shared] partial-write crash-dirty marks */
	uint64_t degraded_reads_reconstructed;   /* [shared] reads served via reconstruction  */
	/*
	 * Data-slot stripes the rebuild reconstructed while their region was
	 * crash-dirty -- write-hole suspects. If a write was torn at the crash
	 * and this slot's data chunk was among the lost ones, the rebuild
	 * decoded from divergent survivors, so the stripe may hold silently-
	 * wrong data. A conservative superset -- most stripes in a crash region
	 * were never torn. Parity-slot rebuild re-encodes parity from data (a
	 * repair) and is not counted. In-memory cumulative: it resets to zero on
	 * restart, so the WARNLOG in the journal is the durable record. Cross-
	 * thread read via get_bdevs, so atomic per the counter rule above.
	 */
	uint64_t crash_dirty_stripes_rebuilt;    /* [shared] batch add; see comment above */
	/*
	 * RMW / full-stripe write accounting. Unlike the UNMAP cluster
	 * below, these counters do NOT form a closed-bucket identity:
	 * accepted submissions whose bdev_io completes successfully are
	 * NOT counted again at completion (the SPDK bdev layer owns that
	 * accounting via spdk_bdev_io_complete). What is counted is:
	 *   - <path>_total / <path>_writes    : a submission was accepted
	 *                                        past every gate.
	 *   - <path>_deferred_*               : the submission returned
	 *                                        -EAGAIN at this specific
	 *                                        gate; SPDK requeues via
	 *                                        NOMEM and the next attempt
	 *                                        is a fresh accepted++ bump.
	 *
	 * Useful derived quantity (steady state):
	 *   accepted_and_completed = <path>_total - sum(<path>_deferred_*)
	 * counts submissions that made it past all defer gates. It does
	 * not distinguish success from terminal failure -- those land on
	 * the bdev_io owner thread via spdk_bdev_io_complete, not here.
	 */
	uint64_t rmw_total;                      /* [home] sub-stripe writes accepted       */
	uint64_t rmw_deferred_scrub;             /* [home] RMW EAGAIN: scrub-active region  */
	uint64_t rmw_deferred_dirty;             /* [home] RMW EAGAIN: deferred-scrub guard */
	uint64_t rmw_deferred_inflight;          /* [home] RMW EAGAIN: stripe already dirty */
	uint64_t full_stripe_writes;             /* [home] full-stripe writes accepted      */
	uint64_t full_stripe_writes_deferred;    /* [home] full-stripe EAGAIN: scrub guard  */
	uint64_t full_stripe_deferred_claim;     /* [home] full-stripe EAGAIN: stripe busy  */
	uint64_t full_stripe_deferred_wib;       /* [home] full-stripe EAGAIN: WIB persist
						  * in flight */
	uint64_t stripe_waitq_parked;            /* [home] writes parked on stripe conflict */
	uint64_t stripe_waitq_max_depth;         /* [home] parked-writes high-water mark    */
	/*
	 * bdev_ios completed as NOMEM by ec_submit_request. Each NOMEM
	 * stalls new submissions on that channel until in-flight IO
	 * drains, so growth under load means the channel queue depth is
	 * collapsing.
	 */
	uint64_t nomem_completions;              /* [shared] -EAGAIN/-ENOMEM -> NOMEM */
	/*
	 * UNMAP accounting. Each call to ec_submit_unmap that gets past the
	 * cross-thread routing hop bumps unmaps_submitted exactly once and
	 * then terminates in one of four buckets:
	 *
	 *   completed   -- native fan-out parent completion landed SUCCESS.
	 *   via_zeros   -- single-stripe UNMAP took the RMW zero-fill fast
	 *                  path (no native fan-out, no bitmap set).
	 *   deferred    -- stripe-busy or persist-in-flight returned -EAGAIN;
	 *                  the bdev layer requeues via NOMEM and the next
	 *                  attempt is a fresh submitted++ bump.
	 *   failed      -- terminal failure: a sync error other than -EAGAIN
	 *                  at submit, a fan-out submit failure, a bitmap
	 *                  persist failure, or an async cb_fn(FAILED).
	 *
	 * Closed identity (steady state, no in-flight UNMAPs):
	 *   submitted == completed + via_zeros + deferred + failed
	 *
	 * In the in-flight window the residual
	 *   submitted - completed - via_zeros - deferred - failed
	 * counts UNMAPs that have been accepted by the EC layer but whose
	 * terminal outcome has not yet landed. A nonzero steady-state
	 * residual is a real bug (silent drop in the EC layer).
	 *
	 * The len == 0 fast path bumps submitted and completed together: it
	 * is a no-op the EC layer hands off to SPDK as SUCCESS with no
	 * fan-out, but operators counting "UNMAPs the EC saw" expect both
	 * counters to advance so the identity stays closed.
	 */
	uint64_t unmaps_submitted;               /* [shared] parent UNMAP submits (per request) */
	uint64_t unmaps_completed;               /* [shared] native fan-out completed */
	uint64_t unmaps_deferred_busy;           /* [home] UNMAP EAGAIN: stripe-busy/persist */
	uint64_t unmaps_via_write_zeros;         /* [home] single-stripe UNMAP -> RMW zero-fill */
	uint64_t unmaps_failed;                  /* [shared] terminal failure (sync or cb_fn) */
	uint64_t unmap_fanout_misses;            /* [shared] per-disk spdk_bdev_unmap_blocks
						  * failure -- physical space not
						  * reclaimed on that disk; not a
						  * bdev_io failure (bitmap already
						  * says unmapped, reads still
						  * synthesise zeros) */
	uint64_t unmapped_reads_synthesized;     /* [shared] reads short-circuited to zero
						  * fill because the target stripe's
						  * unmapped bit was set. Production
						  * signal that bitmap consultation
						  * is firing -- if this stays at 0
						  * after fstrim activity, the
						  * read-path hookup is broken. */
	/*
	 * Write-into-unmapped accounting. Closed identity (steady state):
	 *   writes_into_unmapped == succeeded + writes_into_unmapped_failed
	 * where succeeded is implicit (not counted -- it is the residual
	 * after subtracting the explicit _failed bucket). A growing
	 * _failed counter without a corresponding stuck bdev_io is the
	 * production signal for stripe-alloc / bit-clear-persist trouble.
	 */
	uint64_t writes_into_unmapped;           /* [shared] writes routed through the
						  * write-into-unmapped full-stripe
						  * path (skip-WIB, zero-fill old
						  * data, clear bit on completion).
						  * Production signal that the
						  * write-side hookup is firing on
						  * post-trim write workloads. */
	uint64_t writes_into_unmapped_failed;    /* [shared] write-into-unmapped paths
						  * that failed at stripe-alloc
						  * setup or at the bit-clear
						  * submit/persist step. A data
						  * (fanout) write failure is not
						  * counted here; it completes as a
						  * normal failed bdev_io. The
						  * unmapped bit stays set in every
						  * failure case. */

	uint64_t child_io_failures[EC_MAX_BASE_BDEVS]; /* [shared] failed child
						  * submits + completions per slot;
						  * gates the failure ERRLOGs;
						  * reset on hot-swap. */

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

	/*
	 * SPDK thread on which the consumer submitted this bdev_io. Used by
	 * routing helpers to deliver child completions back to the submitter
	 * (the channel-owning thread) so base_chans[] writes happen there.
	 * Asserted at every base-I/O dispatch site.
	 */
	struct spdk_thread *submitter_thread;

	/* Linkage for the stripe-conflict wait queue (ec->stripe_waitq). */
	TAILQ_ENTRY(ec_bdev_io) waitq_link;
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
	 * Set once a data/parity write has been submitted. ec_rmw_teardown
	 * marks the region crash-dirty only on a failure after this point --
	 * a pre-write failure (WIB persist, reads) leaves parity intact.
	 */
	bool                     writes_issued;

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
 * Increment a SHARED statistics counter -- one that more than one thread
 * can bump (home + submitter writers, or child completions racing across
 * submitter reactors). Relaxed is enough: these are statistics, read back
 * with a relaxed load and tolerant of staleness. Home-serialized counters
 * (one writer, on home) use plain ++ -- see the class tag at each
 * declaration. ec_counter_add takes a batch count; ec_counter_inc is +1.
 */
static inline void
ec_counter_add(uint64_t *counter, uint64_t n)
{
	__atomic_fetch_add(counter, n, __ATOMIC_RELAXED);
}

static inline void
ec_counter_inc(uint64_t *counter)
{
	ec_counter_add(counter, 1);
}

/*
 * Update a slot's cached bdev pointer. Home-thread only; call at every
 * descs[slot] set/clear, with NULL before spdk_bdev_close. See the
 * base_bdevs[] declaration.
 */
static inline void
ec_slot_publish_base_bdev(struct ec_bdev *ec, uint32_t slot,
			  struct spdk_bdev *bdev)
{
	__atomic_store_n(&ec->base_bdevs[slot], bdev, __ATOMIC_RELEASE);
}

/*
 * Rate gate for the per-slot failure ERRLOGs: log the 1st and every
 * 1024th failure. Counts both submit and completion failures. Unknown
 * slot (slot == ec->n) is not counted and always logs.
 */
static inline bool
ec_slot_failure_should_log(struct ec_bdev *ec, uint32_t slot,
			   uint64_t *failures_out)
{
	uint64_t failures = 1;

	if (slot < ec->n) {
		failures = __atomic_add_fetch(&ec->child_io_failures[slot], 1,
					      __ATOMIC_RELAXED);
	}
	*failures_out = failures;
	return failures == 1 || failures % 1024 == 0;
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
 * Min and max blockcnt across open base bdevs. Slots without an open
 * descriptor are skipped, so this is safe on degraded and salvage paths.
 * Sets *min_out to UINT64_MAX when no base is open; callers decide whether
 * that is an error. max_out may be NULL when only the min is needed.
 */
static inline void
ec_base_blockcnt_range(const struct ec_bdev *ec, uint64_t *min_out, uint64_t *max_out)
{
	uint64_t min = UINT64_MAX;
	uint64_t max = 0;
	uint32_t i;

	for (i = 0; i < ec->n; i++) {
		struct spdk_bdev *base;

		if (!ec->descs[i]) {
			continue;
		}
		base = spdk_bdev_desc_get_bdev(ec->descs[i]);
		if (base->blockcnt < min) {
			min = base->blockcnt;
		}
		if (base->blockcnt > max) {
			max = base->blockcnt;
		}
	}
	*min_out = min;
	if (max_out) {
		*max_out = max;
	}
}

/*
 * WIB on-disk blob size: header + region bitmap (one bit per region, rounded
 * up to whole uint64_t words) + a CRC trailer. Single source of truth for the
 * layout, shared by the geometry fit check and the WIB fill/persist path.
 */
static inline uint64_t
ec_wib_map_bytes(const struct ec_bdev *ec)
{
	return (uint64_t)EC_BITMAP_WORDS(ec->wib_num_regions) * sizeof(uint64_t);
}

static inline uint64_t
ec_wib_total_size(const struct ec_bdev *ec)
{
	return sizeof(struct ec_wib_header) + ec_wib_map_bytes(ec) + sizeof(uint32_t); /* CRC */
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
 * Stripe-conflict wait queue (bdev_ec_io.c). kick schedules a home-thread
 * drain of ec->stripe_waitq; fail_all completes every parked write with
 * the given status (home-thread only).
 */
void ec_stripe_waitq_kick(struct ec_bdev *ec);
void ec_stripe_waitq_fail_all(struct ec_bdev *ec,
			      enum spdk_bdev_io_status status);

/*
 * Stripe dirty bitmap helpers. One bit per stripe, set during in-flight
 * RMW or full-stripe write to gate concurrent claimers (returns -EAGAIN
 * for a second claim on the same stripe).
 *
 * Threading: under the multi-reactor design, sets happen only on home
 * (all submit paths reach the claim through entry-routing), but clears
 * happen on the submitter thread that completes the last base I/O. The
 * set/clear pair is therefore cross-thread, so the word-level read-
 * modify-write ops must be atomic. Relaxed ordering is sufficient: the
 * bit is just a serialization gate, not a data-publication signal.
 */
static inline void
ec_stripe_set_dirty(struct ec_bdev *ec, uint64_t stripe_index)
{
	uint64_t mask = UINT64_C(1) << (stripe_index % 64);

	__atomic_or_fetch(&ec->stripe_dirty_map[stripe_index / 64], mask,
			  __ATOMIC_RELAXED);
}

static inline void
ec_stripe_clear_dirty(struct ec_bdev *ec, uint64_t stripe_index)
{
	uint64_t mask = UINT64_C(1) << (stripe_index % 64);

	__atomic_and_fetch(&ec->stripe_dirty_map[stripe_index / 64], ~mask,
			   __ATOMIC_RELAXED);

	/*
	 * A release may unblock parked writes. Depth is read atomically
	 * because clears also run on submitter threads.
	 */
	if (__atomic_load_n(&ec->stripe_waitq_depth, __ATOMIC_ACQUIRE) != 0) {
		ec_stripe_waitq_kick(ec);
	}
}

static inline bool
ec_stripe_is_dirty(const struct ec_bdev *ec, uint64_t stripe_index)
{
	uint64_t mask = UINT64_C(1) << (stripe_index % 64);
	uint64_t word = __atomic_load_n(&ec->stripe_dirty_map[stripe_index / 64],
					__ATOMIC_RELAXED);

	return !!(word & mask);
}

/*
 * Stripe unmapped bitmap helpers. One bit per user stripe, 1 = the
 * stripe is logically zero (unmapped); reads return synthesized zeros
 * for set bits.
 *
 * Threading: mutations happen on home (UNMAP apply-staged, write-into-
 * unmapped bit-clear via the bit-clear queue, both routed to home).
 * Reads happen on the submitter thread (ec_submit_read consults the
 * map before dispatching base reads). Because a stale "bit=0" read
 * could return garbage from a base bdev whose stripe was just logically
 * unmapped, the mutator must publish with release-store and the reader
 * must observe with acquire-load -- that guarantees a reader who sees
 * the post-mutation bit also sees the post-mutation base-bdev state.
 */
static inline void
ec_stripe_set_unmapped(struct ec_bdev *ec, uint64_t stripe_index)
{
	uint64_t mask = UINT64_C(1) << (stripe_index % 64);
	uint64_t *word = &ec->stripe_unmapped_map[stripe_index / 64];
	uint64_t old;

	/*
	 * Release-store on the whole word. __atomic_or_fetch with release
	 * publishes both the bit set and any prior writes (the base-bdev
	 * UNMAP fan-out, the bit-clear's data-write) to readers using
	 * acquire-load.
	 */
	old = __atomic_load_n(word, __ATOMIC_RELAXED);
	while (!__atomic_compare_exchange_n(word, &old, old | mask, false,
					    __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
		/* old updated to current word; retry */
	}
}

static inline void
ec_stripe_clear_unmapped(struct ec_bdev *ec, uint64_t stripe_index)
{
	uint64_t mask = UINT64_C(1) << (stripe_index % 64);
	uint64_t *word = &ec->stripe_unmapped_map[stripe_index / 64];
	uint64_t old;

	old = __atomic_load_n(word, __ATOMIC_RELAXED);
	while (!__atomic_compare_exchange_n(word, &old, old & ~mask, false,
					    __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
		/* old updated to current word; retry */
	}
}

static inline bool
ec_stripe_is_unmapped(const struct ec_bdev *ec, uint64_t stripe_index)
{
	uint64_t mask = UINT64_C(1) << (stripe_index % 64);
	uint64_t word = __atomic_load_n(&ec->stripe_unmapped_map[stripe_index / 64],
					__ATOMIC_ACQUIRE);

	return !!(word & mask);
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

/*
 * WIB region dirty-bit helpers.
 *
 * Threading: mutations happen on home (RMW/full-stripe-write setup
 * sets the bit; idle WIB poller and error-rollback paths clear it).
 * Reads happen on home for the persist decision, but also on the
 * SUBMITTER during ec_submit_degraded_read's guard
 * (ec_wib_region_is_dirty gates degraded reconstruction so it does
 * not read potentially-stale parity for a region with an in-flight
 * write).
 *
 * Because a stale 'clean' read on the submitter could let a degraded
 * reconstruction proceed against in-flight-modified parity and
 * surface silently wrong bytes, the mutator side must publish with
 * release-store and the reader side must observe with acquire-load
 * (same shape as stripe_unmapped_map's release/acquire discipline). Per-word
 * ordering is sufficient because each region's bit lives in exactly
 * one word.
 */
static inline uint32_t
ec_wib_stripe_to_region(uint64_t stripe_index)
{
	return (uint32_t)(stripe_index / EC_WIB_REGION_STRIPES);
}

static inline bool
ec_wib_region_is_dirty(const struct ec_bdev *ec, uint32_t region)
{
	uint64_t mask = UINT64_C(1) << (region % 64);
	uint64_t word = __atomic_load_n(&ec->wib_region_map[region / 64],
					__ATOMIC_ACQUIRE);

	return !!(word & mask);
}

static inline void
ec_wib_region_set_dirty(struct ec_bdev *ec, uint32_t region)
{
	uint64_t mask = UINT64_C(1) << (region % 64);
	uint64_t *word = &ec->wib_region_map[region / 64];
	uint64_t old;

	old = __atomic_load_n(word, __ATOMIC_RELAXED);
	while (!__atomic_compare_exchange_n(word, &old, old | mask, false,
					    __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
		/* old updated to current word; retry */
	}
}

static inline void
ec_wib_region_clear_dirty(struct ec_bdev *ec, uint32_t region)
{
	uint64_t mask = UINT64_C(1) << (region % 64);
	uint64_t *word = &ec->wib_region_map[region / 64];
	uint64_t old;

	old = __atomic_load_n(word, __ATOMIC_RELAXED);
	while (!__atomic_compare_exchange_n(word, &old, old & ~mask, false,
					    __ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
		/* old updated to current word; retry */
	}
}

/*
 * Read and set the crash-dirty bit for a WIB region. Reads run on the home
 * thread, but a partial write failure sets the bit from the submitter thread,
 * so access is atomic -- release on set, acquire on read -- so every reader
 * sees a runtime set.
 */
static inline bool
ec_wib_crash_is_dirty(const struct ec_bdev *ec, uint32_t region)
{
	uint64_t word = __atomic_load_n(&ec->wib_crash_dirty_map[region / 64],
					__ATOMIC_ACQUIRE);
	return !!(word & (UINT64_C(1) << (region % 64)));
}

static inline void
ec_wib_crash_set_dirty(struct ec_bdev *ec, uint32_t region)
{
	__atomic_fetch_or(&ec->wib_crash_dirty_map[region / 64],
			  UINT64_C(1) << (region % 64), __ATOMIC_RELEASE);
}

/*
 * Mark the stripe's WIB region crash-dirty after a partial write failure. The
 * stripe's parity may no longer match its data, and the on-disk WIB bit
 * (persisted before fan-out) is the only durable record of that. Crash-dirty
 * stops the idle poller from clearing the region, makes degraded reads return
 * -EIO, and gets the stripe re-encoded by the scrub -- so reads see -EIO, never
 * garbage, until it is repaired. That scrub runs at next load today; a runtime
 * scrub kick is a follow-up.
 *
 * Call this before releasing the write's WIB inflight ref: the idle poller
 * reads inflight before crash-dirty, so marking first means it can never see
 * inflight == 0 without also seeing this mark.
 */
static inline void
ec_wib_mark_failed_write(struct ec_bdev *ec, uint32_t region)
{
	uint64_t mask = UINT64_C(1) << (region % 64);
	uint64_t old  = __atomic_fetch_or(&ec->wib_crash_dirty_map[region / 64],
					  mask, __ATOMIC_RELEASE);

	if (!(old & mask)) {
		/* First failure to dirty this region -- count and warn once. */
		ec_counter_inc(&ec->wib_failed_write_marks);
		SPDK_WARNLOG("EC bdev %s: partial write failure left WIB region %u "
			     "with inconsistent parity; marked for scrub re-encode "
			     "(degraded reads return -EIO until repaired)\n",
			     ec->bdev.name, region);
	}
}

static inline void
ec_wib_crash_clear_dirty(struct ec_bdev *ec, uint32_t region)
{
	ec->wib_crash_dirty_map[region / 64] &= ~(UINT64_C(1) << (region % 64));
}

static inline uint32_t
ec_wib_crash_count(const struct ec_bdev *ec)
{
	uint32_t region, count = 0;

	for (region = 0; region < ec->wib_num_regions; region++) {
		if (ec_wib_crash_is_dirty(ec, region)) {
			count++;
		}
	}
	return count;
}

/*
 * Returns true if a parity-modifying write to stripe_idx must defer
 * because the startup scrub still owns it:
 *   - current region, at-or-after the scrubber, or
 *   - a region ahead of the scrubber that is still crash-dirty
 *     (parity not yet re-encoded).
 *
 * Stripes the scrubber has already passed are safe. Returns false when
 * no scrub is active. Pure predicate; each caller bumps its own
 * deferred counter.
 */
static inline bool
ec_scrub_blocks_stripe(const struct ec_bdev *ec, uint64_t stripe_idx)
{
	struct ec_scrub_ctx *sctx = ec->scrub_ctx;
	uint32_t             region;

	if (sctx == NULL) {
		return false;
	}

	region = ec_wib_stripe_to_region(stripe_idx);

	if (region == sctx->current_region &&
	    stripe_idx >= sctx->current_stripe) {
		return true;
	}
	if (region > sctx->current_region &&
	    ec_wib_crash_is_dirty(ec, region)) {
		return true;
	}
	return false;
}

/*
 * Per-region in-flight write counter. A write increments it on the home thread
 * when it claims the region (ec_submit_rmw_write, ec_submit_full_write) and
 * decrements it on the submitter thread when it completes (ec_rmw_complete for
 * RMW, ec_child_io_complete for full-stripe writes). Every inc must be matched
 * by exactly one dec; the dec guards against underflow, which would mean a
 * balance bug upstream.
 *
 * Both are atomic because home and submitter touch the counter concurrently.
 * The dec is RELEASE and the poller's read is ACQUIRE: a failed write marks its
 * region crash-dirty and then decs here, so the poller can never see the count
 * reach zero without also seeing that mark. The inc stays relaxed.
 *
 * The dec uses a CAS loop, not a plain fetch_sub: fetch_sub on a zero counter
 * would briefly wrap to UINT32_MAX before restoring, and the idle poller could
 * read that garbage. The CAS decrements only when the count is positive.
 */
static inline void
ec_wib_region_inflight_inc(struct ec_bdev *ec, uint32_t region)
{
	__atomic_fetch_add(&ec->wib_region_inflight[region], 1, __ATOMIC_RELAXED);
}

static inline void
ec_wib_region_inflight_dec(struct ec_bdev *ec, uint32_t region)
{
	uint32_t current;

	current = __atomic_load_n(&ec->wib_region_inflight[region], __ATOMIC_RELAXED);
	while (current > 0) {
		if (__atomic_compare_exchange_n(&ec->wib_region_inflight[region],
						&current, current - 1, false,
						__ATOMIC_RELEASE, __ATOMIC_RELAXED)) {
			return;
		}
		/* current updated with latest value; loop and re-check > 0 */
	}
	SPDK_ERRLOG("EC bdev %s: wib_region_inflight[%u] underflow\n",
		    ec->bdev.name, region);
}

static inline uint32_t
ec_wib_region_inflight_get(const struct ec_bdev *ec, uint32_t region)
{
	return __atomic_load_n(&ec->wib_region_inflight[region], __ATOMIC_ACQUIRE);
}

/*
 * Claim the stripe's WIB region for an in-flight write: set the dirty bit if
 * it was clean, take an inflight ref, and stamp dirty_ticks. Returns true if
 * this call set the bit. Shared by ec_submit_full_write and ec_rmw_submit_core.
 *
 * Does not record held-state on any io context: full-write gates its unwind
 * on ec_io->wib_inflight_held (ec_io flows through paths that never took a
 * ref), while RMW's mctx lifecycle guarantees the ref once setup ran and its
 * teardown decs unconditionally. Each caller records its own.
 */
static inline bool
ec_wib_mark_region(struct ec_bdev *ec, uint64_t stripe_idx)
{
	uint32_t region    = ec_wib_stripe_to_region(stripe_idx);
	bool     was_clean = !ec_wib_region_is_dirty(ec, region);

	if (was_clean) {
		ec_wib_region_set_dirty(ec, region);
	}
	ec_wib_region_inflight_inc(ec, region);
	ec->wib_region_dirty_ticks[region] = spdk_get_ticks();

	return was_clean;
}

/*
 * Global in-flight RMW counter helpers. Same shape as
 * wib_region_inflight: inc on home during RMW setup, dec on submitter
 * during completion / partial-failure cleanup. Stats-only today (the
 * JSON dump reports it; nothing branches on the value), but kept
 * atomic for consistency with the per-region counter and so that any
 * future drain/teardown gate that reads it cannot observe a torn
 * value across the inc-home/dec-submitter boundary.
 */
static inline void
ec_rmw_in_flight_inc(struct ec_bdev *ec)
{
	__atomic_fetch_add(&ec->rmw_in_flight, 1, __ATOMIC_RELAXED);
}

static inline void
ec_rmw_in_flight_dec(struct ec_bdev *ec)
{
	uint32_t current;

	current = __atomic_load_n(&ec->rmw_in_flight, __ATOMIC_RELAXED);
	while (current > 0) {
		if (__atomic_compare_exchange_n(&ec->rmw_in_flight,
						&current, current - 1, false,
						__ATOMIC_RELAXED, __ATOMIC_RELAXED)) {
			return;
		}
		/* current updated; re-check > 0 */
	}
	SPDK_ERRLOG("EC bdev %s: rmw_in_flight underflow\n", ec->bdev.name);
}

static inline uint32_t
ec_rmw_in_flight_get(const struct ec_bdev *ec)
{
	return __atomic_load_n(&ec->rmw_in_flight, __ATOMIC_RELAXED);
}

/* =========================================================================
 * Cross-file internal API
 *
 * Non-inline functions defined in one .c file of the module and called
 * from another. Static helpers used by only one .c file stay in that file.
 * ========================================================================= */

/* Helpers for diagnostics; loop-based, not worth inlining. */
uint32_t ec_wib_count_dirty(const struct ec_bdev *ec);

/* Rebuild progress snapshot; percent is derived at the RPC marshaller. */
struct ec_rebuild_progress {
	uint32_t current_slot;
	uint64_t current_stripe;
	uint64_t num_stripes;
	uint64_t stripes_rebuilt;
	uint32_t slots_to_rebuild;
};

/* Scrub progress snapshot; percent is derived at the RPC marshaller. */
struct ec_scrub_progress {
	uint32_t current_region;
	uint32_t num_regions;
	uint32_t total_dirty_regions;
	uint64_t current_stripe;
	uint64_t stripes_scrubbed;
	uint64_t regions_scrubbed;
};

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
 * Largest user stripe count the WIB can track in one strip -- the ceiling
 * the front reservations are sized for. num_stripes must never exceed it.
 */
uint64_t ec_max_num_stripes(const struct ec_bdev *ec);

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
			uint64_t generation, void *buf);

/*
 * ec_bitmap_validate_buf -- check a slot read back from disk: magic,
 * version, num_stripes match the current volume geometry, blob_bytes
 * is exactly the expected length for that geometry, and the CRC32C
 * over [start, start + blob_bytes) matches the trailer at offset
 * blob_bytes. Returns 0 and fills *gen_out -- and *blob_crc_out, when
 * non-NULL, with the validated CRC trailer -- on success, -EINVAL on any
 * mismatch -- which is also what a never-written or torn slot looks like.
 */
int  ec_bitmap_validate_buf(const struct ec_bdev *ec, const void *buf,
			    uint64_t *gen_out, uint32_t *blob_crc_out);

/*
 * ec_bitmap_commit_fill_buf -- serialize a commit record into buf, followed
 * by a CRC32C trailer. buf must hold at least
 * sizeof(struct ec_bitmap_commit) + sizeof(uint32_t) bytes.
 */
void ec_bitmap_commit_fill_buf(uint64_t committed_gen, uint32_t blob_crc,
			       void *buf);

/*
 * ec_bitmap_commit_validate_buf -- check a commit record read back from
 * disk: magic, version, and the CRC32C trailer over the struct. Returns 0
 * and fills *gen_out / *blob_crc_out on success, -EINVAL on any mismatch --
 * which is also what a never-written or torn record looks like.
 */
int  ec_bitmap_commit_validate_buf(const void *buf, uint64_t *gen_out,
				   uint32_t *blob_crc_out);

/*
 * A (generation, blob CRC) pair pulled from one scanned copy -- a bitmap slot
 * or a commit record. ec_bitmap_select_committed matches the bitmap set
 * against the commit set.
 */
struct ec_bitmap_gen_crc {
	uint64_t generation;
	uint32_t blob_crc;
};

/*
 * ec_bitmap_select_committed -- the pure load decision. Given every
 * CRC-valid bitmap copy and every CRC-valid commit record scanned off disk,
 * return the index into bitmaps[] of the highest-generation bitmap copy that
 * a commit record stamps (same generation AND same blob_crc). Returns -1
 * when no scanned bitmap copy is committed.
 *
 * This is the durability gate. Only a generation that reached the m+1 commit
 * threshold gets a stamp, so a sub-threshold partial persist is never adopted
 * on reload. The blob_crc match matters too: after a crash two blobs can share
 * a generation -- on reload the in-memory counter resets to the committed value
 * and climbs again -- so generation alone cannot distinguish them.
 */
int  ec_bitmap_select_committed(const struct ec_bitmap_gen_crc *bitmaps,
				uint32_t n_bitmaps,
				const struct ec_bitmap_gen_crc *commits,
				uint32_t n_commits);

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
 *   cb_durable -- fires once durability is achieved: the blob and its
 *     stamp have each reached m+1 disks (or all online disks, if fewer
 *     than m+1 are online). Both bitmap_active_copy and
 *     bitmap_commit_active_copy flip at this point, so the active slot
 *     tracks the committed generation. The remaining writes drain in
 *     the background; bitmap_persist_in_flight is not yet cleared. The
 *     UNMAP path uses this to release its caller before a slow disk
 *     finishes, applying the staged bits to the live bitmap here.
 *
 *     If either round misses its threshold, cb_durable is called with
 *     rc < 0 after all writes complete, and neither index flips -- the
 *     bitmap stays on the prior committed generation.
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
 * ec_bitmap_resync_after_replace -- rejoin a hot-swapped slot to the bitmap
 * quorum by overwriting both copies now, or deferring behind an in-flight persist
 * (bitmap_resync_pending). Best-effort. Home-thread only.
 */
void ec_bitmap_resync_after_replace(struct ec_bdev *ec);

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
 * Enqueues a deferred bit-clear. Callers may run on any thread; the
 * helper routes the enqueue to the home thread via spdk_thread_send_msg
 * so pending_bit_clears mutation stays single-threaded.
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

/*
 * Defined in bdev_ec.c; called from the WIB and bitmap persist completions
 * to resume dedicated-channel teardown deferred behind a persist. Two halves:
 * slot releases run before the completion kicks its follow-up persist (never
 * free ec); the unregister tail runs as the completion's last statement (frees
 * ec in the delete case).
 */
void ec_drain_deferred_slot_releases(struct ec_bdev *ec);
void ec_drain_deferred_unregister(struct ec_bdev *ec);

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
 *     ec_rmw_persist_and_dispatch calls ec_wib_persist to set a
 *     region's on-disk dirty bit BEFORE the reads go out (the
 *     persist runs right after setup, so the persist-completion ->
 *     reads edge is the single multi-reactor hop point). When a
 *     persist is already in flight, the RMW context is queued on
 *     wib_deferred_writes and (on the was_clean = true branch)
 *     wib_repersist_needed is set.
 *
 *   bdev_ec_wib.c -> bdev_ec_rmw.c
 *     ec_wib_persist_write_cb, on the final write completion, drains
 *     wib_deferred_writes via ec_wib_deferred_drain, which calls
 *     ec_rmw_dispatch_reads for each queued context.
 *
 * Both bridge functions (ec_wib_persist and ec_rmw_dispatch_reads) are
 * declared in the section immediately below.
 * ========================================================================= */

/*
 * Per-disk front-metadata layout, all fixed-max at create time:
 *   [ bitmap region:  bitmap_reservation_stripes strips ]
 *   [ commit record:  EC_BITMAP_COMMIT_STRIPS strips     ]
 *   [ WIB copy 0:     1 strip                            ]
 *   [ WIB copy 1:     1 strip                            ]
 *   [ user data:      num_stripes strips                 ]
 *
 * ec_bitmap_commit_lba and ec_wib_lba give the copy LBAs. The commit record
 * is raw-replicated on every disk; the WIB is written only on parity disks
 * but reserved on all. Both are pure arithmetic over strip_size and the
 * bitmap reservation -- no I/O, no descriptor lookups.
 */
static inline uint64_t
ec_bitmap_commit_lba(const struct ec_bdev *ec, uint8_t copy)
{
	return (ec_bitmap_reservation_stripes(ec) + copy) *
	       (uint64_t)ec->strip_size;
}

static inline uint64_t
ec_wib_lba(const struct ec_bdev *ec, uint8_t copy)
{
	return (ec_bitmap_reservation_stripes(ec) + EC_BITMAP_COMMIT_STRIPS + copy) *
	       (uint64_t)ec->strip_size;
}

/* WIB helpers needed by the resize and scrub chains, plus the
 * RMW->WIB bridge ec_wib_persist documented above. */
int      ec_wib_persist(struct ec_bdev *ec,
			void (*cb)(void *cb_arg, int rc), void *cb_arg);
int      ec_wib_idle_poller_cb(void *arg);
void     ec_wib_load_async(struct ec_bdev *ec,
			   ec_bdev_create_cb_fn done_fn, void *done_arg);

/* WIB->RMW bridges. ec_wib_deferred_drain resumes each queued RMW by
 * dispatching its reads -- the WIB persist runs before reads, so a
 * deferred RMW has not read yet and its DMA bufs are still zeroed
 * (resuming at writes would fan out zeros over live stripe data).
 * See the WIB <-> RMW protocol section above. */
void     ec_rmw_dispatch_reads(struct ec_rmw_ctx *mctx);

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

/*
 * Shared spdk_thread_send_msg target that fires the parent bdev_io
 * completion with ec_io->status on the submitter (= owner) thread.
 * Used by every owner-route hand-off across the write / UNMAP paths
 * (entry-routing failure, inner-fanout completion, split-segment
 * completion). Callers stash the final status into ec_io->status
 * before invoking spdk_thread_send_msg.
 */
void ec_io_complete_status_on_submitter(void *ctx);

/*
 * Hand off the parent bdev_io completion to the submitter thread. The
 * caller sets ec_io->status first; `what` labels the error log. If the
 * send fails, the bdev_io stays in-flight -- it cannot be completed from
 * the wrong thread.
 */
void ec_io_route_complete_to_submitter(struct ec_bdev_io *ec_io,
				       const char *what);

#endif /* SPDK_BDEV_EC_INTERNAL_H */
