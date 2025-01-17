/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2017 Intel Corporation.
 *   All rights reserved.
 *   Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#ifndef SPDK_INTERNAL_LVOLSTORE_H
#define SPDK_INTERNAL_LVOLSTORE_H

#include "spdk/blob.h"
#include "spdk/lvol.h"
#include "spdk/queue.h"
#include "spdk/tree.h"
#include "spdk/uuid.h"
#include "spdk/thread.h"

/* Default size of blobstore cluster */
#define SPDK_LVS_OPTS_CLUSTER_SZ (4 * 1024 * 1024)

/* Creation time format in RFC 3339 format */
#define SPDK_CREATION_TIME_MAX 21 /* 20 characters + null terminator */

/* UUID + '_' + blobid (20 characters for uint64_t).
 * Null terminator is already included in SPDK_UUID_STRING_LEN. */
#define SPDK_LVOL_UNIQUE_ID_MAX (SPDK_UUID_STRING_LEN + 1 + 20)

struct spdk_lvs_req {
	spdk_lvs_op_complete    cb_fn;
	void                    *cb_arg;
	struct spdk_lvol_store		*lvol_store;
	int				lvserrno;
};

struct spdk_lvs_grow_req {
	struct spdk_lvs_req	base;
	spdk_lvs_op_complete	cb_fn;
	void			*cb_arg;
	struct lvol_store_bdev	*lvs_bdev;
	int			lvol_cnt;
};

struct spdk_lvol_req {
	spdk_lvol_op_complete   cb_fn;
	void                    *cb_arg;
	struct spdk_lvol	*lvol;
	/* Only set while lvol is being deleted and has a clone. */
	struct spdk_lvol	*clone_lvol;
	size_t			sz;
	struct spdk_io_channel	*channel;
	char			name[SPDK_LVOL_NAME_MAX];
};

struct spdk_lvol_copy_req {
	spdk_lvol_op_complete	cb_fn;
	void			*cb_arg;
	struct spdk_lvol	*lvol;
	struct spdk_io_channel	*channel;
	struct spdk_bs_dev	*ext_dev;
};

struct spdk_lvs_with_handle_req {
	spdk_lvs_op_with_handle_complete cb_fn;
	void				*cb_arg;
	struct spdk_lvol_store		*lvol_store;
	struct spdk_bs_dev		*bs_dev;
	struct spdk_bdev		*base_bdev;
	int				lvserrno;
};

struct spdk_lvs_destroy_req {
	spdk_lvs_op_complete    cb_fn;
	void                    *cb_arg;
	struct spdk_lvol_store	*lvs;
};

struct spdk_lvol_with_handle_req {
	spdk_lvol_op_with_handle_complete cb_fn;
	void				*cb_arg;
	struct spdk_lvol		*lvol;
	struct spdk_lvol		*origlvol;
	char				**xattr_names;
	char				**xattrs_external;
};

struct spdk_lvol_bs_dev_req {
	struct spdk_lvol	*lvol;
	struct spdk_bs_dev	*bs_dev;
	spdk_lvol_op_complete	cb_fn;
	void			*cb_arg;
};

struct spdk_lvs_degraded_lvol_set;

struct spdk_lvol_store {
	struct spdk_bs_dev		*bs_dev;
	struct spdk_blob_store		*blobstore;
	struct spdk_blob		*super_blob;
	spdk_blob_id			super_blob_id;
	struct spdk_uuid		uuid;
	int				lvol_count;
	int				lvols_opened;
	TAILQ_HEAD(, spdk_lvol)		lvols;
	TAILQ_HEAD(, spdk_lvol)		pending_lvols;
	TAILQ_HEAD(, spdk_lvol)		retry_open_lvols;
	bool				load_esnaps;
	bool				on_list;
	TAILQ_ENTRY(spdk_lvol_store)	link;
	char				name[SPDK_LVS_NAME_MAX];
	char				new_name[SPDK_LVS_NAME_MAX];
	spdk_bs_esnap_dev_create	esnap_bs_dev_create;
	RB_HEAD(degraded_lvol_sets_tree, spdk_lvs_degraded_lvol_set)	degraded_lvol_sets_tree;
	struct spdk_thread		*thread;
};

typedef TAILQ_HEAD(, freeze_range) lvol_freeze_range_tailq_t;

struct spdk_lvol {
	struct spdk_lvol_store		*lvol_store;
	struct spdk_blob		*blob;
	spdk_blob_id			blob_id;
	char				unique_id[SPDK_LVOL_UNIQUE_ID_MAX];
	char				name[SPDK_LVOL_NAME_MAX];
	struct spdk_uuid		uuid;
	char				uuid_str[SPDK_UUID_STRING_LEN];
	char				creation_time[SPDK_CREATION_TIME_MAX];
	struct spdk_bdev		*bdev;
	int				ref_count;
	bool				action_in_progress;
	enum blob_clear_method		clear_method;
	TAILQ_ENTRY(spdk_lvol)		link;
	struct spdk_lvs_degraded_lvol_set *degraded_set;
	TAILQ_ENTRY(spdk_lvol)		degraded_link;

	struct spdk_spinlock		spinlock;
	/*
	 * Currently freezed ranges for this lvol. Used to populate new channels.
	 * Protected by spinlock.
	 */
	lvol_freeze_range_tailq_t	freezed_ranges;
	/* Pending freezed ranges for this lvol. These ranges are not currently
	 * freezed due to overlapping with another freezed range.
	 * Protected by spinlock.
	 */
	lvol_freeze_range_tailq_t	pending_freezed_ranges;
};

struct spdk_fragmap {
	struct spdk_bit_array *map;

	uint64_t cluster_size;
	uint64_t block_size;
	uint64_t num_clusters;
	uint64_t num_allocated_clusters;
};

struct spdk_fragmap_req {
	struct spdk_bdev *bdev;
	struct spdk_bdev_desc *bdev_desc;
	struct spdk_io_channel *bdev_io_channel;

	struct spdk_fragmap fragmap;

	uint64_t offset;
	uint64_t size;
	uint64_t current_offset;

	spdk_lvol_op_with_fragmap_handle_complete cb_fn;
	void *cb_arg;
};

struct lvol_store_bdev *vbdev_lvol_store_first(void);
struct lvol_store_bdev *vbdev_lvol_store_next(struct lvol_store_bdev *prev);

void spdk_lvol_resize(struct spdk_lvol *lvol, uint64_t sz, spdk_lvol_op_complete cb_fn,
		      void *cb_arg);

void spdk_lvol_set_read_only(struct spdk_lvol *lvol, spdk_lvol_op_complete cb_fn,
			     void *cb_arg);

int spdk_lvs_esnap_missing_add(struct spdk_lvol_store *lvs, struct spdk_lvol *lvol,
			       const void *esnap_id, uint32_t id_len);
void spdk_lvs_esnap_missing_remove(struct spdk_lvol *lvol);
bool spdk_lvs_notify_hotplug(const void *esnap_id, uint32_t id_len,
			     spdk_lvol_op_with_handle_complete cb_fn, void *cb_arg);

#endif /* SPDK_INTERNAL_LVOLSTORE_H */
