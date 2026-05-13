/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec.h -- public API surface of the EC bdev module.
 *
 * Internal types (struct ec_bdev, ec_rebuild_ctx, ec_scrub_ctx, etc.) and
 * globals (g_ec_bdev_list) live in bdev_ec_internal.h. Files that
 * constitute the module itself include both headers; future consumers
 * outside the module should include only this one.
 */

#ifndef SPDK_BDEV_EC_H
#define SPDK_BDEV_EC_H

#include "spdk/stdinc.h"
#include "spdk/bdev.h"
#include "spdk/bdev_module.h"  /* for spdk_bdev_unregister_cb */

/* Opaque forward declarations for the public API. */
struct ec_bdev;
struct spdk_json_write_ctx;

/* =========================================================================
 * Callback typedefs
 * ========================================================================= */

/*
 * Callback invoked when ec_bdev_create_async() completes.
 * rc == 0 on success; negative errno on failure.
 * Called on the EC bdev's home thread.
 */
typedef void (*ec_bdev_create_cb_fn)(void *cb_arg, int rc);

/* rc == 0 on success, negative errno on failure. Called on the EC bdev's home thread. */
typedef void (*ec_replace_cb_fn)(void *cb_arg, int rc);

/*
 * rc == 0 on success, negative errno on failure. Called on the EC bdev's home thread.
 * stripes_rebuilt is the count of stripes reconstructed during the rebuild.
 */
typedef void (*ec_rebuild_cb_fn)(void *cb_arg, int rc, uint64_t stripes_rebuilt);

/*
 * Completion callback for ec_bdev_resize.
 * Called on the EC bdev's home thread when resize finishes or fails.
 *
 * cb_arg -- opaque pointer supplied by the caller
 * rc     -- 0 on success, negative errno on failure
 */
typedef void (*ec_resize_cb_fn)(void *cb_arg, int rc);

/*
 * Completion callback for ec_bitmap_persist_async().
 * rc == 0 when at least m+1 disks (or all online disks, if fewer than
 * m+1 are online) have durably written the new blob. rc < 0 if not
 * enough writes succeeded to meet that threshold; the bitmap is then
 * still on the prior generation and the next persist will retry.
 * Called on the EC bdev's home thread.
 */
typedef void (*ec_bitmap_persist_cb_fn)(void *cb_arg, int rc);

/* =========================================================================
 * Public API for RPC layer
 * ========================================================================= */

/*
 * Look up an EC bdev by name in g_ec_bdev_list.
 * Returns NULL if no EC bdev with the given name exists.
 * Safe to call only from the EC bdev's home thread.
 */
struct ec_bdev *ec_bdev_find(const char *name);

/*
 * Shared JSON writers used by both the bdev dump_info path
 * (ec_dump_info_json) and the bdev_ec_get_bdevs RPC.
 *
 *   ec_write_base_bdevs_array_json: writes the "base_bdevs": [...] array
 *     describing each slot's role/state/needs_rebuild and the live base
 *     bdev name (or "<failed>" / "<unknown>" sentinels).
 *
 *   ec_write_io_stats_json: writes the shared RMW / full-stripe-write /
 *     UNMAP / degraded-read counter run -- the fields common to both
 *     callers, in identical order. Each caller writes its own trailing
 *     fields (e.g. degraded_read_eio_dirty, WIB/scrub stats) after it.
 */
void ec_write_base_bdevs_array_json(struct spdk_json_write_ctx *w,
				    const struct ec_bdev *ec);
void ec_write_rebuild_progress_json(struct spdk_json_write_ctx *w,
				    const struct ec_bdev *ec);
void ec_write_io_stats_json(struct spdk_json_write_ctx *w,
			    const struct ec_bdev *ec);

/*
 * Asynchronously create an EC bdev. Loads the persisted WIB and the
 * in-band unmapped bitmap, then fires done_fn once the bdev is fully
 * ready. The RPC response must be sent from done_fn, not from the
 * caller.
 *
 * salvage_requested gates the in-band bitmap's missing-on-disk
 * decision:
 *   - false: fresh-create path. If the on-disk bitmap is missing or
 *            unreadable, persist a fresh all-mapped bitmap to both
 *            slots on every online disk (chained via the persist
 *            chain's cb_drained), then proceed. Overwriting both copies
 *            up front ensures a stale blob from a reused base bdev
 *            cannot out-rank our fresh bitmap on a future load.
 *   - true:  recreate path. If the on-disk bitmap is missing,
 *            **fail the create loudly** -- an established volume's map
 *            must never be silently reconstructed as all-mapped, which
 *            would resurrect stale non-zero data as if it were live.
 *
 * Returns 0 if the async operation was started (done_fn will be called).
 * Returns negative errno on immediate failure (done_fn is NOT called).
 */
int ec_bdev_create_async(const char *name, uint32_t strip_size_kb, uint32_t k, uint32_t m,
			 const char **base_bdev_names, const struct spdk_uuid *uuid,
			 bool salvage_requested,
			 ec_bdev_create_cb_fn done_fn, void *done_arg);

void ec_bdev_delete(const char *name, spdk_bdev_unregister_cb cb_fn, void *cb_arg);

/*
 * Asynchronously replace the FAILED disk in slot with new_bdev_name.
 * Transitions slot: FAILED -> REPLACING, opens channels on all threads.
 * cb_fn NOT called on synchronous error return.
 *
 * Returns: 0, -ENODEV, -ENOENT, -EINVAL, -EBUSY, -ENOMEM
 */
int ec_bdev_replace_base_bdev(const char *ec_name, uint32_t slot,
			      const char *new_bdev_name,
			      ec_replace_cb_fn cb_fn, void *cb_arg);

/*
 * Start background rebuild for all REPLACING slots in ec_name.
 * cb_fn NOT called on synchronous error return.
 *
 * Returns: 0, -ENODEV, -EBUSY, -ENOENT, -ENOMEM
 */
int ec_bdev_start_rebuild(const char *ec_name,
			  ec_rebuild_cb_fn cb_fn, void *cb_arg);

/* Query live rebuild progress. Returns -ENODEV or -ENOENT on error. */
int ec_bdev_get_rebuild_progress(const char *ec_name,
				 uint32_t *current_slot,
				 uint64_t *current_stripe,
				 uint64_t *num_stripes,
				 uint64_t *stripes_rebuilt,
				 uint32_t *slots_to_rebuild);

/* Query live scrub progress. Returns -ENODEV or -ENOENT on error. */
int ec_bdev_get_scrub_progress(const char *ec_name,
			       uint32_t   *current_region,
			       uint32_t   *num_regions,
			       uint32_t   *total_dirty_regions,
			       uint64_t   *current_stripe,
			       uint64_t   *stripes_scrubbed,
			       uint64_t   *regions_scrubbed);

/*
 * Set rebuild QoS parameters on a running rebuild.
 * max_stripes_per_sec: 0 = unlimited. paused: true = pause rebuild.
 * Returns -ENODEV, -ENOENT (no rebuild running).
 */
int ec_bdev_set_rebuild_qos(const char *ec_name,
			    uint32_t max_stripes_per_sec,
			    bool paused);

/*
 * Cancel a running rebuild. Slots remain in REPLACING state.
 * Returns -ENODEV, -ENOENT (no rebuild running).
 */
int ec_bdev_stop_rebuild(const char *ec_name);

/* Query WIB status. Returns -ENODEV on error. */
int ec_bdev_get_wib_status(const char *ec_name,
			   uint32_t   *num_regions,
			   uint32_t   *dirty_regions,
			   uint32_t   *generation,
			   bool       *persist_pending);

/*
 * Query in-band unmapped-bitmap status. Returns -ENODEV on error.
 *
 *   num_stripes      -- user-stripe count covered by the bitmap.
 *   unmapped_stripes -- count of set bits in stripe_unmapped_map; how
 *                       many user stripes are currently flagged zero.
 *   blob_bytes       -- on-disk CRC-covered length of one slot (header
 *                       + span); the CRC trailer adds sizeof(uint32_t)
 *                       to the slot's full on-disk extent.
 *   generation       -- bitmap_generation; only ever increases
 *                       across persists.
 *   active_copy      -- global slot index (0 or 1) the last persist
 *                       committed to.
 *   persist_pending  -- true while a bitmap persist is in flight (or
 *                       its stragglers are still draining; see the
 *                       "stay true until drainout" invariant in
 *                       bdev_ec_bitmap.c).
 */
int ec_bdev_get_unmap_status(const char *ec_name,
			     uint64_t   *num_stripes,
			     uint64_t   *unmapped_stripes,
			     uint64_t   *blob_bytes,
			     uint32_t   *generation,
			     uint8_t    *active_copy,
			     bool       *persist_pending);

/*
 * Expand the EC bdev in-place after base bdevs have grown.
 * The WIB and unmapped-bitmap reservations sit at fixed front offsets and do
 * not move; resize only grows the user-data region, updating blockcnt, stripe
 * count, and the dirty bitmap.
 *
 * Returns: 0, -ENODEV, -EBUSY, -EIO, -EALREADY, -ENOMEM
 */
int ec_bdev_resize(const char *ec_name,
		   ec_resize_cb_fn cb_fn, void *cb_arg);

#endif /* SPDK_BDEV_EC_H */
