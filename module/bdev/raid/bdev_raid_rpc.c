/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2018 Intel Corporation.
 *   All rights reserved.
 */

#include "spdk/rpc.h"
#include "spdk/bdev.h"
#include "bdev_raid.h"
#include "spdk/util.h"
#include "spdk/string.h"
#include "spdk/log.h"
#include "spdk/env.h"
#include "spdk_internal/rpc_autogen.h"
#include "spdk/bit_array.h"

/*
 * Decoder object for RPC get_raids
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_get_bdevs_decoders[] = {
	{"category", offsetof(struct rpc_bdev_raid_get_bdevs_ctx, category), rpc_decode_bdev_raid_state},
};

/*
 * brief:
 * rpc_bdev_raid_get_bdevs function is the RPC for rpc_bdev_raid_get_bdevs. This is used to list
 * all the raid bdev names based on the input category requested. Category should be
 * one of "all", "online", "configuring" or "offline". "all" means all the raids
 * whether they are online or configuring or offline. "online" is the raid bdev which
 * is registered with bdev layer. "configuring" is the raid bdev which does not have
 * full configuration discovered yet. "offline" is the raid bdev which is not
 * registered with bdev as of now and it has encountered any error or user has
 * requested to offline the raid.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_get_bdevs(struct spdk_jsonrpc_request *request,
			const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_get_bdevs_ctx req = {};
	struct spdk_json_write_ctx  *w;
	struct raid_bdev            *raid_bdev;

	if (spdk_json_decode_object(params, rpc_bdev_raid_get_bdevs_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_get_bdevs_decoders),
				    &req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		return;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_array_begin(w);

	/* Get raid bdev list based on the category requested */
	TAILQ_FOREACH(raid_bdev, &g_raid_bdev_list, global_link) {
		if (raid_bdev->state == (enum spdk_bdev_raid_state)req.category ||
		    req.category == RPC_BDEV_RAID_STATE_ALL) {
			char uuid_str[SPDK_UUID_STRING_LEN];

			spdk_json_write_object_begin(w);
			spdk_json_write_named_string(w, "name", raid_bdev->bdev.name);
			spdk_uuid_fmt_lower(uuid_str, sizeof(uuid_str), &raid_bdev->bdev.uuid);
			spdk_json_write_named_string(w, "uuid", uuid_str);
			raid_bdev_write_info_json(raid_bdev, w);
			spdk_json_write_object_end(w);
		}
	}
	spdk_json_write_array_end(w);
	spdk_jsonrpc_end_result(request, w);
}
SPDK_RPC_REGISTER("bdev_raid_get_bdevs", rpc_bdev_raid_get_bdevs, SPDK_RPC_RUNTIME)

/*
 * Decoder object for RPC bdev_raid_create
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_create_decoders[] = {
	{"name", offsetof(struct rpc_bdev_raid_create_ctx, name), spdk_json_decode_string},
	{"strip_size_kb", offsetof(struct rpc_bdev_raid_create_ctx, strip_size_kb), spdk_json_decode_uint32, true},
	{"raid_level", offsetof(struct rpc_bdev_raid_create_ctx, raid_level), rpc_decode_bdev_raid_level},
	{"base_bdevs", offsetof(struct rpc_bdev_raid_create_ctx, base_bdevs), rpc_decode_raid_base_bdevs},
	{"uuid", offsetof(struct rpc_bdev_raid_create_ctx, uuid), spdk_json_decode_uuid, true},
	{"superblock", offsetof(struct rpc_bdev_raid_create_ctx, superblock), spdk_json_decode_bool, true},
	{"delta_bitmap", offsetof(struct rpc_bdev_raid_create_ctx, delta_bitmap), spdk_json_decode_bool, true},
};

static void
rpc_bdev_raid_create_cb(void *_ctx, int status)
{
	struct rpc_bdev_raid_create_ctx *ctx = _ctx;

	if (status != 0) {
		spdk_jsonrpc_send_error_response_fmt(ctx->request, status,
						     "Failed to create RAID bdev %s: %s",
						     ctx->name,
						     spdk_strerror(-status));
	} else {
		spdk_jsonrpc_send_bool_response(ctx->request, true);
	}

	free_rpc_bdev_raid_create(ctx);
	free(ctx);
}

/*
 * brief:
 * rpc_bdev_raid_create function is the RPC for creating RAID bdevs. It takes
 * input as raid bdev name, raid level, strip size in KB and list of base bdev names.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_create(struct spdk_jsonrpc_request *request,
		     const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_create_ctx	*ctx;
	int				rc;
	size_t				i;
	size_t				num_base_bdevs;

	ctx = calloc(1, sizeof(*ctx));
	if (ctx == NULL) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_raid_create_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_create_decoders),
				    ctx)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}
	num_base_bdevs = ctx->base_bdevs.count;

	for (i = 0; i < num_base_bdevs; i++) {
		if (strlen(ctx->base_bdevs.items[i]) == 0) {
			spdk_jsonrpc_send_error_response_fmt(request, -EINVAL,
							     "The base bdev name cannot be empty: %s",
							     spdk_strerror(EINVAL));
			goto cleanup;
		}
	}

	ctx->request = request;

	rc = raid_bdev_create(ctx->name, ctx->strip_size_kb, num_base_bdevs,
			      ctx->base_bdevs.items, (enum spdk_bdev_raid_level)ctx->raid_level,
			      ctx->superblock, &ctx->uuid, ctx->delta_bitmap,
			      rpc_bdev_raid_create_cb, ctx);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "Failed to create RAID bdev %s: %s",
						     ctx->name, spdk_strerror(-rc));
		goto cleanup;
	}

	return;
cleanup:
	free_rpc_bdev_raid_create(ctx);
	free(ctx);
}
SPDK_RPC_REGISTER("bdev_raid_create", rpc_bdev_raid_create, SPDK_RPC_RUNTIME)

/*
 * Decoder object for RPC raid_bdev_delete
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_delete_decoders[] = {
	{"name", offsetof(struct rpc_bdev_raid_delete_ctx, name), spdk_json_decode_string},
	{"clear_sb", offsetof(struct rpc_bdev_raid_delete_ctx, clear_sb), spdk_json_decode_bool, true},
};

/*
 * brief:
 * params:
 * cb_arg - pointer to the callback context.
 * rc - return code of the deletion of the raid bdev.
 * returns:
 * none
 */
static void
bdev_raid_delete_done(void *cb_arg, int rc)
{
	struct rpc_bdev_raid_delete_ctx *ctx = cb_arg;

	if (rc != 0) {
		SPDK_ERRLOG("Failed to delete raid bdev %s (%d): %s\n",
			    ctx->name, rc, spdk_strerror(-rc));
		spdk_jsonrpc_send_error_response(ctx->request, SPDK_JSONRPC_ERROR_INTERNAL_ERROR,
						 spdk_strerror(-rc));
		goto exit;
	}

	spdk_jsonrpc_send_bool_response(ctx->request, true);
exit:
	free_rpc_bdev_raid_delete(ctx);
	free(ctx);
}

/*
 * brief:
 * rpc_bdev_raid_delete function is the RPC for deleting a raid bdev. It takes raid
 * name as input and delete that raid bdev including freeing the base bdev
 * resources.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_delete(struct spdk_jsonrpc_request *request,
		     const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_delete_ctx *ctx;
	struct raid_bdev *raid_bdev;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_raid_delete_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_delete_decoders),
				    ctx)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	raid_bdev = raid_bdev_find_by_name(ctx->name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "raid bdev %s not found",
						     ctx->name);
		goto cleanup;
	}

	ctx->request = request;

	raid_bdev_delete(raid_bdev, ctx->clear_sb, bdev_raid_delete_done, ctx);

	return;

cleanup:
	free_rpc_bdev_raid_delete(ctx);
	free(ctx);
}
SPDK_RPC_REGISTER("bdev_raid_delete", rpc_bdev_raid_delete, SPDK_RPC_RUNTIME)

/*
 * Decoder object for RPC bdev_raid_add_base_bdev
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_add_base_bdev_decoders[] = {
	{"base_bdev", offsetof(struct rpc_bdev_raid_add_base_bdev_ctx, base_bdev), spdk_json_decode_string},
	{"raid_bdev", offsetof(struct rpc_bdev_raid_add_base_bdev_ctx, raid_bdev), spdk_json_decode_string},
};

static void
rpc_bdev_raid_add_base_bdev_done(void *ctx, int status)
{
	struct spdk_jsonrpc_request *request = ctx;

	if (status != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, status, "Failed to add base bdev to RAID bdev: %s",
						     spdk_strerror(-status));
		return;
	}

	spdk_jsonrpc_send_bool_response(request, true);
}

static void
rpc_bdev_raid_event_cb(enum spdk_bdev_event_type type, struct spdk_bdev *bdev, void *event_ctx)
{
}

/*
 * brief:
 * bdev_raid_add_base_bdev function is the RPC for adding base bdev to a raid bdev.
 * It takes base bdev and raid bdev names as input.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_add_base_bdev(struct spdk_jsonrpc_request *request,
			    const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_add_base_bdev_ctx req = {};
	struct raid_bdev *raid_bdev;
	int rc;

	if (spdk_json_decode_object(params, rpc_bdev_raid_add_base_bdev_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_add_base_bdev_decoders),
				    &req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INTERNAL_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	raid_bdev = raid_bdev_find_by_name(req.raid_bdev);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV, "raid bdev %s is not found in config",
						     req.raid_bdev);
		goto cleanup;
	}

	rc = raid_bdev_add_base_bdev(raid_bdev, req.base_bdev, rpc_bdev_raid_add_base_bdev_done, request);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "Failed to add base bdev %s to RAID bdev %s: %s",
						     req.base_bdev, req.raid_bdev,
						     spdk_strerror(-rc));
		goto cleanup;
	}

cleanup:
	free_rpc_bdev_raid_add_base_bdev(&req);
}
SPDK_RPC_REGISTER("bdev_raid_add_base_bdev", rpc_bdev_raid_add_base_bdev, SPDK_RPC_RUNTIME)

/*
 * Decoder object for RPC bdev_raid_remove_base_bdev
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_remove_base_bdev_decoders[] = {
	{"name", offsetof(struct rpc_bdev_raid_remove_base_bdev_ctx, name), spdk_json_decode_string},
};

static void
rpc_bdev_raid_remove_base_bdev_done(void *ctx, int status)
{
	struct spdk_jsonrpc_request *request = ctx;

	if (status != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, status, "Failed to remove base bdev from raid bdev");
		return;
	}

	spdk_jsonrpc_send_bool_response(request, true);
}

/*
 * brief:
 * bdev_raid_remove_base_bdev function is the RPC for removing base bdev from a raid bdev.
 * It takes base bdev name as input.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_remove_base_bdev(struct spdk_jsonrpc_request *request,
			       const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_remove_base_bdev_ctx req = {};
	struct spdk_bdev_desc *desc;
	int rc;

	if (spdk_json_decode_object(params, rpc_bdev_raid_remove_base_bdev_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_remove_base_bdev_decoders),
				    &req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		return;
	}

	rc = spdk_bdev_open_ext(req.name, false, rpc_bdev_raid_event_cb, NULL, &desc);
	free_rpc_bdev_raid_remove_base_bdev(&req);
	if (rc != 0) {
		goto err;
	}

	rc = raid_bdev_remove_base_bdev(spdk_bdev_desc_get_bdev(desc), rpc_bdev_raid_remove_base_bdev_done,
					request);
	spdk_bdev_close(desc);
	if (rc != 0) {
		goto err;
	}

	return;
err:
	rpc_bdev_raid_remove_base_bdev_done(request, rc);
}
SPDK_RPC_REGISTER("bdev_raid_remove_base_bdev", rpc_bdev_raid_remove_base_bdev, SPDK_RPC_RUNTIME)

static const struct spdk_json_object_decoder rpc_bdev_raid_set_options_decoders[] = {
	{"process_window_size_kb", offsetof(struct rpc_bdev_raid_set_options_ctx, process_window_size_kb), spdk_json_decode_uint32, true},
	{"process_max_bandwidth_mb_sec", offsetof(struct rpc_bdev_raid_set_options_ctx, process_max_bandwidth_mb_sec), spdk_json_decode_uint32, true},
	{"resize_on_base_bdev_resize", offsetof(struct rpc_bdev_raid_set_options_ctx, resize_on_base_bdev_resize), spdk_json_decode_bool, true},
};

static void
rpc_bdev_raid_set_options(struct spdk_jsonrpc_request *request, const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_set_options_ctx req = {};
	struct spdk_raid_bdev_opts opts;
	int rc;

	raid_bdev_get_opts(&opts);
	req.process_window_size_kb = opts.process_window_size_kb;
	req.process_max_bandwidth_mb_sec = opts.process_max_bandwidth_mb_sec;
	req.resize_on_base_bdev_resize = opts.resize_on_base_bdev_resize;
	if (params && spdk_json_decode_object(params, rpc_bdev_raid_set_options_decoders,
					      SPDK_COUNTOF(rpc_bdev_raid_set_options_decoders),
					      &req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		return;
	}
	opts.process_window_size_kb = req.process_window_size_kb;
	opts.process_max_bandwidth_mb_sec = req.process_max_bandwidth_mb_sec;
	opts.resize_on_base_bdev_resize = req.resize_on_base_bdev_resize;

	rc = raid_bdev_set_opts(&opts);
	if (rc) {
		spdk_jsonrpc_send_error_response(request, rc, spdk_strerror(-rc));
	} else {
		spdk_jsonrpc_send_bool_response(request, true);
	}

	return;
}
SPDK_RPC_REGISTER("bdev_raid_set_options", rpc_bdev_raid_set_options,
		  SPDK_RPC_STARTUP | SPDK_RPC_RUNTIME)

/*
 * Decoder object for RPC bdev_raid_grow_base_bdev
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_grow_base_bdev_decoders[] = {
	{"raid_name", offsetof(struct rpc_bdev_raid_grow_base_bdev_ctx, raid_name), spdk_json_decode_string},
	{"base_name", offsetof(struct rpc_bdev_raid_grow_base_bdev_ctx, base_name), spdk_json_decode_string},
};

/*
 * brief:
 * params:
 * cb_arg - pointer to the callback context.
 * rc - return code of the growing a base bdev.
 * returns:
 * none
 */
static void
bdev_raid_grow_base_bdev_done(void *cb_arg, int rc)
{
	struct rpc_bdev_raid_grow_base_bdev_ctx *ctx = cb_arg;
	struct spdk_jsonrpc_request *request = ctx->request;

	if (rc != 0) {
		SPDK_ERRLOG("Failed to grow raid %s adding base bdev %s (%d): %s\n",
			    ctx->raid_name, ctx->base_name, rc, spdk_strerror(-rc));
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INTERNAL_ERROR,
						 spdk_strerror(-rc));
		goto exit;
	}

	spdk_jsonrpc_send_bool_response(request, true);
exit:
	free_rpc_bdev_raid_grow_base_bdev(ctx);
	free(ctx);
}

/*
 * brief:
 * bdev_raid_grow_base_bdev is the RPC to add a base bdev to a raid bdev, growing the raid's size
 * if there isn't an empty base bdev slot. It takes raid bdev name and base bdev name as input.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_grow_base_bdev(struct spdk_jsonrpc_request *request,
			     const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_grow_base_bdev_ctx *ctx;
	struct raid_bdev *raid_bdev;
	struct spdk_bdev *base_bdev;
	int rc;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_raid_grow_base_bdev_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_grow_base_bdev_decoders),
				    ctx)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	raid_bdev = raid_bdev_find_by_name(ctx->raid_name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "raid bdev %s not found",
						     ctx->raid_name);
		goto cleanup;
	}

	base_bdev = spdk_bdev_get_by_name(ctx->base_name);
	if (base_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "base bdev %s not found",
						     ctx->base_name);
		goto cleanup;
	}

	ctx->request = request;

	rc = raid_bdev_grow_base_bdev(raid_bdev, ctx->base_name, bdev_raid_grow_base_bdev_done,
				      ctx);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "Failed to grow raid %s adding base bdev %s: %s",
						     ctx->raid_name, ctx->base_name,
						     spdk_strerror(-rc));
		goto cleanup;
	}

	return;

cleanup:
	free_rpc_bdev_raid_grow_base_bdev(ctx);
	free(ctx);
}
SPDK_RPC_REGISTER("bdev_raid_grow_base_bdev", rpc_bdev_raid_grow_base_bdev, SPDK_RPC_RUNTIME)

/*
 * Decoder object for RPC bdev_raid_resize
 */
static const struct spdk_json_object_decoder rpc_bdev_raid_resize_decoders[] = {
	{"name", offsetof(struct rpc_bdev_raid_resize_ctx, name), spdk_json_decode_string},
};

struct bdev_raid_resize_rpc_ctx {
	struct rpc_bdev_raid_resize_ctx req;
	uint64_t old_blockcnt;
};

static void
bdev_raid_resize_free(struct bdev_raid_resize_rpc_ctx *ctx)
{
	free_rpc_bdev_raid_resize(&ctx->req);
	free(ctx);
}

static void
bdev_raid_resize_send_result(struct bdev_raid_resize_rpc_ctx *ctx, bool resized)
{
	struct spdk_json_write_ctx *w;
	struct raid_bdev *raid_bdev;

	raid_bdev = raid_bdev_find_by_name(ctx->req.name);

	w = spdk_jsonrpc_begin_result(ctx->req.request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "name", ctx->req.name);
	spdk_json_write_named_uint64(w, "old_blockcnt", ctx->old_blockcnt);
	spdk_json_write_named_uint64(w, "new_blockcnt",
				     raid_bdev != NULL ? raid_bdev->bdev.blockcnt : 0);
	spdk_json_write_named_bool(w, "resized", resized);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(ctx->req.request, w);
}

/*
 * brief:
 * params:
 * cb_arg - pointer to the callback context.
 * rc - return code of the raid bdev resize. Only the superblock update can fail here.
 * returns:
 * none
 */
static void
bdev_raid_resize_done(void *cb_arg, int rc)
{
	struct bdev_raid_resize_rpc_ctx *ctx = cb_arg;
	struct raid_bdev *raid_bdev;

	raid_bdev = raid_bdev_find_by_name(ctx->req.name);
	if (rc != 0) {
		/* The size is already changed in memory when the superblock update fails. */
		spdk_jsonrpc_send_error_response_fmt(ctx->req.request, rc,
						     "raid bdev %s was resized to %" PRIu64 " blocks but its superblock update failed: %s",
						     ctx->req.name, raid_bdev != NULL ? raid_bdev->bdev.blockcnt : 0,
						     spdk_strerror(-rc));
	} else {
		/* A retried superblock update does not change the size. */
		bdev_raid_resize_send_result(ctx, raid_bdev != NULL &&
					     raid_bdev->bdev.blockcnt != ctx->old_blockcnt);
	}

	bdev_raid_resize_free(ctx);
}

/*
 * brief:
 * bdev_raid_resize is the RPC to recalculate the size of a raid bdev from its current base
 * bdevs. It takes the raid bdev name as input. The response is sent after the superblock is
 * updated, and reports whether the raid bdev size changed.
 * params:
 * request - pointer to json rpc request
 * params - pointer to request parameters
 * returns:
 * none
 */
static void
rpc_bdev_raid_resize(struct spdk_jsonrpc_request *request, const struct spdk_json_val *params)
{
	struct bdev_raid_resize_rpc_ctx *ctx;
	struct raid_bdev *raid_bdev;
	int rc;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_raid_resize_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_resize_decoders),
				    &ctx->req)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	raid_bdev = raid_bdev_find_by_name(ctx->req.name);
	if (raid_bdev == NULL) {
		spdk_jsonrpc_send_error_response_fmt(request, -ENODEV,
						     "raid bdev %s not found",
						     ctx->req.name);
		goto cleanup;
	}

	ctx->req.request = request;
	ctx->old_blockcnt = raid_bdev->bdev.blockcnt;

	/* On success, the callback sends the response and frees the context. */
	rc = raid_bdev_resize_explicit(raid_bdev, bdev_raid_resize_done, ctx);
	if (rc == 0) {
		return;
	}

	if (rc == -EALREADY) {
		bdev_raid_resize_send_result(ctx, false);
	} else {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "Failed to resize raid bdev %s: %s",
						     ctx->req.name, spdk_strerror(-rc));
	}

cleanup:
	bdev_raid_resize_free(ctx);
}
SPDK_RPC_REGISTER("bdev_raid_resize", rpc_bdev_raid_resize, SPDK_RPC_RUNTIME)

/* delta bitmap */

/* Structure to decode the input parameters for delta bitmap RPC methods. */
static const struct spdk_json_object_decoder rpc_bdev_raid_base_bdev_delta_bitmap_decoders[] = {
	{"base_bdev_name", 0, spdk_json_decode_string},
};

struct rpc_bdev_raid_delta_bitmap_ctx {
	char *base_bdev_name;
	struct spdk_jsonrpc_request *request;
};

static void
rpc_bdev_raid_get_base_bdev_delta_bitmap(struct spdk_jsonrpc_request *request,
		const struct spdk_json_val *params)
{
	char *base_bdev_name = NULL;
	struct spdk_json_write_ctx *w;
	struct spdk_bit_array *delta_bitmap;
	char *encoded;
	uint64_t region_size;
	int rc;

	rc = spdk_json_decode_object(params, rpc_bdev_raid_base_bdev_delta_bitmap_decoders,
				     SPDK_COUNTOF(rpc_bdev_raid_base_bdev_delta_bitmap_decoders),
				     &base_bdev_name);
	if (rc) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	delta_bitmap = raid_bdev_get_base_bdev_delta_bitmap(base_bdev_name);
	if (delta_bitmap == NULL) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INVALID_PARAMS,
						 "Invalid parameters");
		goto cleanup;
	}

	encoded = spdk_bit_array_to_base64_string(delta_bitmap);
	if (encoded == NULL) {
		SPDK_ERRLOG("Failed to encode delta map to base64 string\n");
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INTERNAL_ERROR,
						 spdk_strerror(ENOMEM));
		goto cleanup;
	}

	region_size = raid_bdev_region_size_base_bdev_delta_bitmap(base_bdev_name);

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);

	spdk_json_write_named_uint64(w, "region_size", region_size);
	spdk_json_write_named_string(w, "delta_bitmap", encoded);

	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

	free(encoded);

cleanup:
	free(base_bdev_name);
}
SPDK_RPC_REGISTER("bdev_raid_get_base_bdev_delta_bitmap", rpc_bdev_raid_get_base_bdev_delta_bitmap,
		  SPDK_RPC_RUNTIME)

/*
 * brief:
 * params:
 * cb_arg - pointer to the callback context.
 * rc - return code of the delta bitmap stopping.
 * returns:
 * none
 */
static void
bdev_raid_stop_base_bdev_delta_bitmap_done(void *cb_arg, int rc)
{
	struct rpc_bdev_raid_delta_bitmap_ctx *ctx = cb_arg;
	struct spdk_jsonrpc_request *request = ctx->request;

	if (rc != 0) {
		SPDK_ERRLOG("Failed to stop base bdev %s delta map: %s",
			    ctx->base_bdev_name, spdk_strerror(-rc));
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INTERNAL_ERROR,
						 spdk_strerror(-rc));
		goto exit;
	}

	spdk_jsonrpc_send_bool_response(request, true);
exit:
	free(ctx->base_bdev_name);
	free(ctx);
}

static void
rpc_bdev_raid_stop_base_bdev_delta_bitmap(struct spdk_jsonrpc_request *request,
		const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_delta_bitmap_ctx *ctx;
	int rc;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_raid_base_bdev_delta_bitmap_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_base_bdev_delta_bitmap_decoders),
				    &ctx->base_bdev_name)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	ctx->request = request;

	rc = raid_bdev_stop_base_bdev_delta_bitmap(ctx->base_bdev_name,
			bdev_raid_stop_base_bdev_delta_bitmap_done, ctx);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "Failed to stop base bdev %s delta map: %s",
						     ctx->base_bdev_name, spdk_strerror(-rc));
		goto cleanup;
	}

	return;

cleanup:
	if (ctx->base_bdev_name) {
		free(ctx->base_bdev_name);
	}
	free(ctx);
}
SPDK_RPC_REGISTER("bdev_raid_stop_base_bdev_delta_bitmap",
		  rpc_bdev_raid_stop_base_bdev_delta_bitmap,
		  SPDK_RPC_RUNTIME)

/*
 * brief:
 * params:
 * cb_arg - pointer to the callback context.
 * rc - return code of the faulty base bdev clearing.
 * returns:
 * none
 */
static void
bdev_raid_clear_base_bdev_faulty_state_done(void *cb_arg, int rc)
{
	struct rpc_bdev_raid_delta_bitmap_ctx *ctx = cb_arg;
	struct spdk_jsonrpc_request *request = ctx->request;

	if (rc != 0) {
		SPDK_ERRLOG("Failed to clear base bdev %s faulty state: %s",
			    ctx->base_bdev_name, spdk_strerror(-rc));
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INTERNAL_ERROR,
						 spdk_strerror(-rc));
		goto exit;
	}

	spdk_jsonrpc_send_bool_response(request, true);
exit:
	free(ctx->base_bdev_name);
	free(ctx);
}

static void
rpc_bdev_raid_clear_base_bdev_faulty_state(struct spdk_jsonrpc_request *request,
		const struct spdk_json_val *params)
{
	struct rpc_bdev_raid_delta_bitmap_ctx *ctx;
	int rc;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, spdk_strerror(ENOMEM));
		return;
	}

	if (spdk_json_decode_object(params, rpc_bdev_raid_base_bdev_delta_bitmap_decoders,
				    SPDK_COUNTOF(rpc_bdev_raid_base_bdev_delta_bitmap_decoders),
				    &ctx->base_bdev_name)) {
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_PARSE_ERROR,
						 "spdk_json_decode_object failed");
		goto cleanup;
	}

	ctx->request = request;

	rc = raid_bdev_clear_base_bdev_faulty_state(ctx->base_bdev_name,
			bdev_raid_clear_base_bdev_faulty_state_done,
			ctx);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response_fmt(request, rc,
						     "Failed to clear base bdev %s faulty state: %s",
						     ctx->base_bdev_name, spdk_strerror(-rc));
		goto cleanup;
	}

	return;

cleanup:
	if (ctx->base_bdev_name) {
		free(ctx->base_bdev_name);
	}
	free(ctx);
}
SPDK_RPC_REGISTER("bdev_raid_clear_base_bdev_faulty_state",
		  rpc_bdev_raid_clear_base_bdev_faulty_state, SPDK_RPC_RUNTIME)
