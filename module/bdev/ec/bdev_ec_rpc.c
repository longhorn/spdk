/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2026 Longhorn Authors.
 *   All rights reserved.
 */

/*
 * bdev_ec_rpc.c -- JSON-RPC handlers for the bdev_ec_* methods.
 *
 * Thin glue: each handler decodes its arguments, calls into the EC core
 * (bdev_ec.c and the subsystem files), and encodes the JSON response.
 */

#include "bdev_ec_internal.h"
#include "spdk/rpc.h"
#include "spdk/util.h"
#include "spdk/string.h"
#include "spdk/log.h"

/* Canonical "decode failed" response shared by every RPC handler. */
static void
ec_rpc_send_decode_error(struct spdk_jsonrpc_request *request)
{
	spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INVALID_PARAMS,
					 "spdk_json_decode_object failed");
}

/* =========================================================================
 * RPC: bdev_ec_create
 * ========================================================================= */

struct rpc_bdev_ec_create {
	char *name;
	uint32_t strip_size_kb;
	uint32_t data_chunk_count;
	uint32_t parity_chunk_count;
	char *base_bdevs[EC_MAX_BASE_BDEVS];
	size_t num_base_bdevs;
	struct spdk_uuid uuid;
	bool salvage_requested;
};

static void
free_rpc_bdev_ec_create(struct rpc_bdev_ec_create *req)
{
	size_t i;

	free(req->name);

	for (i = 0; i < req->num_base_bdevs; i++) {
		free(req->base_bdevs[i]);
	}
}

static int
decode_base_bdevs(const struct spdk_json_val *val, void *out)
{
	struct rpc_bdev_ec_create *req = out;
	return spdk_json_decode_array(val, spdk_json_decode_string, req->base_bdevs,
				      EC_MAX_BASE_BDEVS, &req->num_base_bdevs, sizeof(char *));
}

/*
 * Async RPC context: keeps decoded params and request pointer alive until
 * ec_bdev_create_async() fires the completion callback.
 */
struct rpc_bdev_ec_create_ctx {
	struct rpc_bdev_ec_create    req;
	struct spdk_jsonrpc_request *request;
};

/*
 * Completion callback invoked by ec_bdev_create_async() once the bdev is
 * registered and the WIB load is done. Sends the JSON-RPC response and
 * frees the context. rc == 0 on success.
 */
static void
rpc_bdev_ec_create_done(void *cb_arg, int rc)
{
	struct rpc_bdev_ec_create_ctx *ctx = cb_arg;

	if (rc != 0) {
		spdk_jsonrpc_send_error_response(ctx->request, rc,
						 spdk_strerror(-rc));
	} else {
		spdk_jsonrpc_send_bool_response(ctx->request, true);
	}

	free_rpc_bdev_ec_create(&ctx->req);
	free(ctx);
}

static void
rpc_bdev_ec_create(struct spdk_jsonrpc_request *request,
		   const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_create_ctx *ctx;
	int rc;

	ctx = calloc(1, sizeof(*ctx));
	if (!ctx) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM,
						 "Out of memory");
		return;
	}
	ctx->request = request;

	if (spdk_json_decode_object(params, rpc_bdev_ec_create_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_create_decoders),
				    &ctx->req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed\n");
		ec_rpc_send_decode_error(request);
		/*
		 * decode_object keeps any fields it managed to parse before the
		 * failure (name, base_bdevs[] strings), so they must be freed
		 * even on the error path.
		 */
		free_rpc_bdev_ec_create(&ctx->req);
		free(ctx);
		return;
	}

	if (ctx->req.num_base_bdevs !=
	    (ctx->req.data_chunk_count + ctx->req.parity_chunk_count)) {
		SPDK_ERRLOG("Number of base bdevs (%zu) must equal "
			    "data_chunk_count + parity_chunk_count (%u)\n",
			    ctx->req.num_base_bdevs,
			    ctx->req.data_chunk_count + ctx->req.parity_chunk_count);
		spdk_jsonrpc_send_error_response(request, SPDK_JSONRPC_ERROR_INVALID_PARAMS,
						 "Invalid base_bdevs count");
		free_rpc_bdev_ec_create(&ctx->req);
		free(ctx);
		return;
	}

	rc = ec_bdev_create_async(ctx->req.name,
				  ctx->req.strip_size_kb,
				  ctx->req.data_chunk_count,
				  ctx->req.parity_chunk_count,
				  (const char **)ctx->req.base_bdevs,
				  spdk_uuid_is_null(&ctx->req.uuid) ? NULL : &ctx->req.uuid,
				  ctx->req.salvage_requested,
				  rpc_bdev_ec_create_done,
				  ctx);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response(request, rc, spdk_strerror(-rc));
		free_rpc_bdev_ec_create(&ctx->req);
		free(ctx);
		return;
	}

	/*
	 * Async path: the reactor is free. rpc_bdev_ec_create_done() sends
	 * the JSON-RPC response once WIB load completes.
	 */
}

static void
free_rpc_bdev_ec_delete(struct rpc_bdev_ec_delete *req)
{
	free(req->name);
}

static void
rpc_bdev_ec_delete_cb(void *cb_arg, int bdeverrno)
{
	struct spdk_jsonrpc_request *request = cb_arg;

	if (bdeverrno == 0) {
		spdk_jsonrpc_send_bool_response(request, true);
	} else {
		spdk_jsonrpc_send_error_response(request, bdeverrno, spdk_strerror(-bdeverrno));
	}
}

static void
rpc_bdev_ec_delete(struct spdk_jsonrpc_request *request,
		   const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_delete req = {};

	if (spdk_json_decode_object(params, rpc_bdev_ec_delete_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_delete_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	ec_bdev_delete(req.name, rpc_bdev_ec_delete_cb, request);

cleanup:
	free_rpc_bdev_ec_delete(&req);
}

static void
free_rpc_bdev_ec_replace(struct rpc_bdev_ec_replace *req)
{
	free(req->ec_name);
	free(req->new_bdev_name);
	free(req);
}

/* Shared by the replace submit and completion paths (rpc_bdev_ec_replace_*). */
static const char *
ec_errno_to_msg(int rc)
{
	switch (-rc) {
	case ENODEV:  return "EC bdev not found";
	case ENOENT:  return "Slot index out of range";
	case EINVAL:  return "Slot not in FAILED state or geometry mismatch";
	case EBUSY:   return "Another replace is already in progress";
	case ENOMEM:  return "Out of memory";
	default:      return spdk_strerror(-rc);
	}
}

static void
rpc_bdev_ec_replace_cb(void *cb_arg, int rc)
{
	struct rpc_bdev_ec_replace  *req     = cb_arg;
	struct spdk_jsonrpc_request *request = req->request;
	struct spdk_json_write_ctx  *w;
	struct ec_bdev              *ec      = NULL;

	if (rc != 0) {
		spdk_jsonrpc_send_error_response(request, rc, ec_errno_to_msg(rc));
		free_rpc_bdev_ec_replace(req);
		return;
	}

	ec = ec_bdev_find(req->ec_name);
	if (ec == NULL) {
		/*
		 * The bdev was deleted between accepting the replace and this
		 * completion. Report the race rather than a "replacing" success
		 * for a bdev that no longer exists.
		 */
		spdk_jsonrpc_send_error_response(request, -ENODEV,
						 "EC bdev deleted before replace completed");
		free_rpc_bdev_ec_replace(req);
		return;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "ec_name",       req->ec_name);
	spdk_json_write_named_uint32(w, "slot",           req->slot);
	spdk_json_write_named_string(w, "new_bdev_name",  req->new_bdev_name);
	spdk_json_write_named_string(w, "state",          "replacing");
	spdk_json_write_named_bool(w,   "needs_rebuild",
		ec->needs_rebuild[req->slot]);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

	free_rpc_bdev_ec_replace(req);
}

static void
rpc_bdev_ec_replace_base_bdev(struct spdk_jsonrpc_request *request,
			      const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_replace *req;
	int rc;

	req = calloc(1, sizeof(*req));
	if (!req) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, "Out of memory");
		return;
	}
	req->request = request;

	if (spdk_json_decode_object(params, rpc_bdev_ec_replace_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_replace_decoders),
				    req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_replace_base_bdev\n");
		ec_rpc_send_decode_error(request);
		free_rpc_bdev_ec_replace(req);
		return;
	}

	rc = ec_bdev_replace_base_bdev(req->ec_name, req->slot, req->new_bdev_name,
				       rpc_bdev_ec_replace_cb, req);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response(request, rc, ec_errno_to_msg(rc));
		free_rpc_bdev_ec_replace(req);
	}
}