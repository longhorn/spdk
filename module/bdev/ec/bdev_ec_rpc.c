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

static const struct spdk_json_object_decoder rpc_bdev_ec_create_decoders[] = {
	{"name", offsetof(struct rpc_bdev_ec_create, name), spdk_json_decode_string},
	{"strip_size_kb", offsetof(struct rpc_bdev_ec_create, strip_size_kb), spdk_json_decode_uint32},
	{"data_chunk_count", offsetof(struct rpc_bdev_ec_create, data_chunk_count), spdk_json_decode_uint32},
	{"parity_chunk_count", offsetof(struct rpc_bdev_ec_create, parity_chunk_count), spdk_json_decode_uint32},
	{"base_bdevs", 0, decode_base_bdevs},
	{"uuid", offsetof(struct rpc_bdev_ec_create, uuid), spdk_json_decode_uuid, true},
	{"salvage_requested", offsetof(struct rpc_bdev_ec_create, salvage_requested),
		spdk_json_decode_bool, true},
};

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
SPDK_RPC_REGISTER("bdev_ec_create", rpc_bdev_ec_create, SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_delete
 * ========================================================================= */

struct rpc_bdev_ec_delete {
	char *name;
};

static void
free_rpc_bdev_ec_delete(struct rpc_bdev_ec_delete *req)
{
	free(req->name);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_delete_decoders[] = {
	{"name", offsetof(struct rpc_bdev_ec_delete, name), spdk_json_decode_string},
};

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
SPDK_RPC_REGISTER("bdev_ec_delete", rpc_bdev_ec_delete, SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_replace_base_bdev
 * ========================================================================= */

struct rpc_bdev_ec_replace {
	char    *ec_name;
	uint32_t slot;
	char    *new_bdev_name;
	struct spdk_jsonrpc_request *request;
};

static void
free_rpc_bdev_ec_replace(struct rpc_bdev_ec_replace *req)
{
	free(req->ec_name);
	free(req->new_bdev_name);
	free(req);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_replace_decoders[] = {
	{"ec_name",       offsetof(struct rpc_bdev_ec_replace, ec_name),       spdk_json_decode_string},
	{"slot",          offsetof(struct rpc_bdev_ec_replace, slot),          spdk_json_decode_uint32},
	{"new_bdev_name", offsetof(struct rpc_bdev_ec_replace, new_bdev_name), spdk_json_decode_string},
};

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
SPDK_RPC_REGISTER("bdev_ec_replace_base_bdev", rpc_bdev_ec_replace_base_bdev, SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_get_bdevs
 * ========================================================================= */

struct rpc_bdev_ec_get_bdevs {
	char *name;
};

static void
free_rpc_bdev_ec_get_bdevs(struct rpc_bdev_ec_get_bdevs *req)
{
	free(req->name);
}

/* name is optional: absent lists every EC bdev, present filters to that one. */
static const struct spdk_json_object_decoder rpc_bdev_ec_get_bdevs_decoders[] = {
	{"name", offsetof(struct rpc_bdev_ec_get_bdevs, name), spdk_json_decode_string, true},
};

static void
ec_write_bdev_info_json(struct spdk_json_write_ctx *w, struct ec_bdev *ec)
{
	spdk_json_write_object_begin(w);

	spdk_json_write_named_string(w,  "name",               ec->bdev.name);
	spdk_json_write_named_uint32(w,  "k",                  ec->k);
	spdk_json_write_named_uint32(w,  "m",                  ec->m);
	spdk_json_write_named_uint32(w,  "n",                  ec->n);
	spdk_json_write_named_uint32(w,  "strip_size_kb",      ec->strip_size_kb);
	spdk_json_write_named_uint32(w,  "failed_count",       ec->failed_count);
	spdk_json_write_named_bool(w,    "offline",            ec->offline);
	spdk_json_write_named_bool(w,    "replace_in_progress",
				   ec->replace_in_progress);

	ec_write_io_stats_json(w, ec);
	spdk_json_write_named_uint64(w, "degraded_read_eio_dirty",
				     ec->degraded_read_eio_dirty);

	spdk_json_write_named_bool(w, "rebuild_in_progress",
				   ec->rebuild_ctx != NULL);
	if (ec->rebuild_ctx) {
		ec_write_rebuild_progress_json(w, ec);
	}

	ec_write_base_bdevs_array_json(w, ec);

	spdk_json_write_object_end(w);
}

static void
rpc_bdev_ec_get_bdevs(struct spdk_jsonrpc_request *request,
		      const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_get_bdevs  req = {};
	struct spdk_json_write_ctx   *w;
	struct ec_bdev               *ec = NULL;

	if (params && spdk_json_decode_object(params, rpc_bdev_ec_get_bdevs_decoders,
					      SPDK_COUNTOF(rpc_bdev_ec_get_bdevs_decoders),
					      &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_get_bdevs\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	if (req.name) {
		ec = ec_bdev_find(req.name);
		if (ec == NULL) {
			spdk_jsonrpc_send_error_response(request, -ENODEV, "EC bdev not found");
			goto cleanup;
		}
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_array_begin(w);

	if (req.name) {
		ec_write_bdev_info_json(w, ec);
	} else {
		TAILQ_FOREACH(ec, &g_ec_bdev_list, link) {
			ec_write_bdev_info_json(w, ec);
		}
	}

	spdk_json_write_array_end(w);
	spdk_jsonrpc_end_result(request, w);

cleanup:
	free_rpc_bdev_ec_get_bdevs(&req);
}
SPDK_RPC_REGISTER("bdev_ec_get_bdevs", rpc_bdev_ec_get_bdevs, SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_start_rebuild
 * ========================================================================= */

struct rpc_bdev_ec_start_rebuild {
	char *ec_name;
};

static void
free_rpc_bdev_ec_start_rebuild(struct rpc_bdev_ec_start_rebuild *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_start_rebuild_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_start_rebuild, ec_name), spdk_json_decode_string},
};

static void
rpc_bdev_ec_start_rebuild_cb(void *cb_arg, int rc, uint64_t stripes_rebuilt)
{
	const char *ec_name = cb_arg;

	if (rc == 0) {
		SPDK_NOTICELOG("EC bdev %s: rebuild finished successfully "
			       "(%" PRIu64 " stripes)\n", ec_name, stripes_rebuilt);
	} else {
		SPDK_ERRLOG("EC bdev %s: rebuild failed (rc=%d) after "
			    "%" PRIu64 " stripes\n", ec_name, rc, stripes_rebuilt);
	}

	free(cb_arg);
}

static void
rpc_bdev_ec_start_rebuild(struct spdk_jsonrpc_request *request,
			  const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_start_rebuild req = {};
	struct spdk_json_write_ctx      *w;
	struct ec_bdev                  *ec = NULL;
	char                            *ec_name_copy;
	const char                      *errmsg;
	int                              rc;

	if (spdk_json_decode_object(params, rpc_bdev_ec_start_rebuild_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_start_rebuild_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_start_rebuild\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	/*
	 * Copy the EC bdev name for the async completion callback.
	 * The req struct will be freed at cleanup before the rebuild finishes.
	 */
	ec_name_copy = strdup(req.ec_name);
	if (!ec_name_copy) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM, "Out of memory");
		goto cleanup;
	}

	rc = ec_bdev_start_rebuild(req.ec_name, rpc_bdev_ec_start_rebuild_cb,
				   ec_name_copy);
	if (rc != 0) {
		free(ec_name_copy);
		switch (-rc) {
		case ENODEV: errmsg = "EC bdev not found";           break;
		case EBUSY:  errmsg = "Rebuild already in progress"; break;
		case ENOENT: errmsg = "No REPLACING slots to rebuild"; break;
		case ENOMEM: errmsg = "Out of memory";               break;
		default:     errmsg = spdk_strerror(-rc);            break;
		}
		spdk_jsonrpc_send_error_response(request, rc, errmsg);
		goto cleanup;
	}

	/* Locate the EC bdev to read back live state for the response */
	ec = ec_bdev_find(req.ec_name);

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w,  "ec_name",    req.ec_name);
	spdk_json_write_named_uint64(w,  "num_stripes",
		ec && ec->rebuild_ctx ? ec->rebuild_ctx->num_stripes : 0);
	spdk_json_write_named_uint32(w,  "first_slot",
		ec && ec->rebuild_ctx ? ec->rebuild_ctx->current_slot : 0);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

cleanup:
	free_rpc_bdev_ec_start_rebuild(&req);
}
SPDK_RPC_REGISTER("bdev_ec_start_rebuild", rpc_bdev_ec_start_rebuild, SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_get_rebuild_progress
 * ========================================================================= */

struct rpc_bdev_ec_get_rebuild_progress {
	char *ec_name;
};

static void
free_rpc_bdev_ec_get_rebuild_progress(struct rpc_bdev_ec_get_rebuild_progress *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder
rpc_bdev_ec_get_rebuild_progress_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_get_rebuild_progress, ec_name),
	 spdk_json_decode_string},
};

static void
rpc_bdev_ec_get_rebuild_progress(struct spdk_jsonrpc_request *request,
				 const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_get_rebuild_progress req = {};
	struct spdk_json_write_ctx *w;
	struct ec_rebuild_progress  p;
	uint32_t percent;
	int      rc;

	if (spdk_json_decode_object(params,
				    rpc_bdev_ec_get_rebuild_progress_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_get_rebuild_progress_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for "
			    "bdev_ec_get_rebuild_progress\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	rc = ec_bdev_get_rebuild_progress(req.ec_name, &p);
	if (rc != 0) {
		const char *errmsg;
		switch (-rc) {
		case ENODEV: errmsg = "EC bdev not found";     break;
		case ENOENT: errmsg = "No rebuild in progress"; break;
		default:     errmsg = spdk_strerror(-rc);       break;
		}
		spdk_jsonrpc_send_error_response(request, rc, errmsg);
		goto cleanup;
	}

	{
		uint64_t total_stripes = p.num_stripes * p.slots_to_rebuild;
		percent = (total_stripes > 0)
			? (uint32_t)((p.stripes_rebuilt * 100) / total_stripes)
			: 100;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w,  "ec_name",          req.ec_name);
	spdk_json_write_named_uint32(w,  "current_slot",     p.current_slot);
	spdk_json_write_named_uint64(w,  "current_stripe",   p.current_stripe);
	spdk_json_write_named_uint64(w,  "num_stripes",      p.num_stripes);
	spdk_json_write_named_uint64(w,  "stripes_rebuilt",  p.stripes_rebuilt);
	spdk_json_write_named_uint32(w,  "slots_to_rebuild", p.slots_to_rebuild);
	spdk_json_write_named_uint32(w,  "percent_complete", percent);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

cleanup:
	free_rpc_bdev_ec_get_rebuild_progress(&req);
}
SPDK_RPC_REGISTER("bdev_ec_get_rebuild_progress", rpc_bdev_ec_get_rebuild_progress,
		  SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_get_wib_status
 * ========================================================================= */

struct rpc_bdev_ec_get_wib_status {
	char *ec_name;
};

static void
free_rpc_bdev_ec_get_wib_status(struct rpc_bdev_ec_get_wib_status *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_get_wib_status_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_get_wib_status, ec_name),
	 spdk_json_decode_string},
};

static void
rpc_bdev_ec_get_wib_status(struct spdk_jsonrpc_request *request,
			   const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_get_wib_status req = {};
	struct spdk_json_write_ctx *w;
	uint32_t num_regions, dirty_regions;
	uint64_t generation;
	bool     persist_pending;
	int      rc;

	if (spdk_json_decode_object(params,
				    rpc_bdev_ec_get_wib_status_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_get_wib_status_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_get_wib_status\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	rc = ec_bdev_get_wib_status(req.ec_name,
				    &num_regions,
				    &dirty_regions,
				    &generation,
				    &persist_pending);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response(request, rc,
						 rc == -ENODEV ? "EC bdev not found"
							       : spdk_strerror(-rc));
		goto cleanup;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w,  "ec_name",         req.ec_name);
	spdk_json_write_named_uint32(w,  "num_regions",     num_regions);
	spdk_json_write_named_uint32(w,  "dirty_regions",   dirty_regions);
	spdk_json_write_named_uint64(w,  "generation",      generation);
	spdk_json_write_named_bool(w,    "persist_pending",  persist_pending);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

cleanup:
	free_rpc_bdev_ec_get_wib_status(&req);
}
SPDK_RPC_REGISTER("bdev_ec_get_wib_status", rpc_bdev_ec_get_wib_status,
		  SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_get_unmap_status
 * ========================================================================= */

struct rpc_bdev_ec_get_unmap_status {
	char *ec_name;
};

static void
free_rpc_bdev_ec_get_unmap_status(struct rpc_bdev_ec_get_unmap_status *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_get_unmap_status_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_get_unmap_status, ec_name),
	 spdk_json_decode_string},
};

static void
rpc_bdev_ec_get_unmap_status(struct spdk_jsonrpc_request *request,
			     const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_get_unmap_status req = {};
	struct spdk_json_write_ctx *w;
	uint64_t num_stripes, unmapped_stripes, blob_bytes;
	uint64_t generation;
	uint8_t  active_copy;
	bool     persist_pending;
	int      rc;

	if (spdk_json_decode_object(params,
				    rpc_bdev_ec_get_unmap_status_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_get_unmap_status_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_get_unmap_status\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	rc = ec_bdev_get_unmap_status(req.ec_name,
				      &num_stripes,
				      &unmapped_stripes,
				      &blob_bytes,
				      &generation,
				      &active_copy,
				      &persist_pending);
	if (rc != 0) {
		spdk_jsonrpc_send_error_response(request, rc,
						 rc == -ENODEV ? "EC bdev not found"
							       : spdk_strerror(-rc));
		goto cleanup;
	}

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w,  "ec_name",          req.ec_name);
	spdk_json_write_named_uint64(w,  "num_stripes",      num_stripes);
	spdk_json_write_named_uint64(w,  "unmapped_stripes", unmapped_stripes);
	spdk_json_write_named_uint64(w,  "blob_bytes",       blob_bytes);
	spdk_json_write_named_uint64(w,  "generation",       generation);
	spdk_json_write_named_uint32(w,  "active_copy",      active_copy);
	spdk_json_write_named_bool(w,    "persist_pending",  persist_pending);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

cleanup:
	free_rpc_bdev_ec_get_unmap_status(&req);
}
SPDK_RPC_REGISTER("bdev_ec_get_unmap_status", rpc_bdev_ec_get_unmap_status,
		  SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_get_scrub_progress
 * ========================================================================= */

struct rpc_bdev_ec_get_scrub_progress {
	char *ec_name;
};

static void
free_rpc_bdev_ec_get_scrub_progress(struct rpc_bdev_ec_get_scrub_progress *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_get_scrub_progress_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_get_scrub_progress, ec_name),
	 spdk_json_decode_string},
};

static void
rpc_bdev_ec_get_scrub_progress(struct spdk_jsonrpc_request *request,
			       const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_get_scrub_progress req = {};
	struct spdk_json_write_ctx *w;
	struct ec_scrub_progress    p;
	uint32_t percent;
	int      rc;

	if (spdk_json_decode_object(params,
				    rpc_bdev_ec_get_scrub_progress_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_get_scrub_progress_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_get_scrub_progress\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	rc = ec_bdev_get_scrub_progress(req.ec_name, &p);
	if (rc != 0) {
		const char *errmsg;
		switch (-rc) {
		case ENODEV: errmsg = "EC bdev not found";     break;
		case ENOENT: errmsg = "No scrub in progress";  break;
		default:     errmsg = spdk_strerror(-rc);       break;
		}
		spdk_jsonrpc_send_error_response(request, rc, errmsg);
		goto cleanup;
	}

	percent = (p.total_dirty_regions > 0)
		? (uint32_t)((p.regions_scrubbed * 100) / p.total_dirty_regions)
		: 100;

	w = spdk_jsonrpc_begin_result(request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w,  "ec_name",              req.ec_name);
	spdk_json_write_named_uint32(w,  "current_region",       p.current_region);
	spdk_json_write_named_uint32(w,  "num_regions",          p.num_regions);
	spdk_json_write_named_uint32(w,  "total_dirty_regions",  p.total_dirty_regions);
	spdk_json_write_named_uint64(w,  "current_stripe",       p.current_stripe);
	spdk_json_write_named_uint64(w,  "stripes_scrubbed",     p.stripes_scrubbed);
	spdk_json_write_named_uint64(w,  "regions_scrubbed",     p.regions_scrubbed);
	spdk_json_write_named_uint32(w,  "percent_complete",     percent);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(request, w);

cleanup:
	free_rpc_bdev_ec_get_scrub_progress(&req);
}
SPDK_RPC_REGISTER("bdev_ec_get_scrub_progress", rpc_bdev_ec_get_scrub_progress,
		  SPDK_RPC_RUNTIME)

/* =========================================================================
 * bdev_ec_resize -- in-place resize (same k/m, bigger disks)
 * ========================================================================= */

struct rpc_bdev_ec_resize {
	char                        *ec_name;
	uint64_t                     old_blockcnt;
	struct spdk_jsonrpc_request *request;
};

static void
free_rpc_bdev_ec_resize(struct rpc_bdev_ec_resize *req)
{
	free(req->ec_name);
	free(req);
}

static const struct spdk_json_object_decoder rpc_bdev_ec_resize_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_resize, ec_name),
	 spdk_json_decode_string},
};

static void
rpc_bdev_ec_resize_cb(void *cb_arg, int rc)
{
	struct rpc_bdev_ec_resize   *req = cb_arg;
	struct spdk_json_write_ctx  *w;
	struct ec_bdev              *ec  = NULL;

	if (rc != 0) {
		SPDK_ERRLOG("EC bdev %s: resize failed (rc=%d)\n",
			    req->ec_name, rc);
		spdk_jsonrpc_send_error_response(req->request, rc,
						 spdk_strerror(-rc));
		goto out;
	}

	ec = ec_bdev_find(req->ec_name);

	SPDK_NOTICELOG("EC bdev %s: resize finished, blockcnt %" PRIu64 " -> %" PRIu64 "\n",
		       req->ec_name, req->old_blockcnt,
		       ec ? ec->bdev.blockcnt : 0);

	w = spdk_jsonrpc_begin_result(req->request);
	spdk_json_write_object_begin(w);
	spdk_json_write_named_string(w, "ec_name",      req->ec_name);
	spdk_json_write_named_uint64(w, "old_blockcnt", req->old_blockcnt);
	spdk_json_write_named_uint64(w, "new_blockcnt", ec ? ec->bdev.blockcnt : 0);
	spdk_json_write_object_end(w);
	spdk_jsonrpc_end_result(req->request, w);

out:
	free_rpc_bdev_ec_resize(req);
}

static void
rpc_bdev_ec_resize(struct spdk_jsonrpc_request *request,
		   const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_resize  tmp_req = {};
	struct rpc_bdev_ec_resize *req;
	struct ec_bdev            *ec = NULL;
	int                        rc;

	if (spdk_json_decode_object(params, rpc_bdev_ec_resize_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_resize_decoders),
				    &tmp_req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for bdev_ec_resize\n");
		ec_rpc_send_decode_error(request);
		free(tmp_req.ec_name);
		return;
	}

	req = calloc(1, sizeof(*req));
	if (!req) {
		spdk_jsonrpc_send_error_response(request, -ENOMEM,
						 "Out of memory");
		free(tmp_req.ec_name);
		return;
	}
	/* Transfer ownership of the decoded string into req. */
	req->ec_name = tmp_req.ec_name;
	req->request = request;

	ec = ec_bdev_find(req->ec_name);
	/* Capture old_blockcnt before the async resize changes it. */
	req->old_blockcnt = ec ? ec->bdev.blockcnt : 0;

	/*
	 * ec_bdev_resize() is async: it quiesces the bdev, then calls
	 * rpc_bdev_ec_resize_cb once ec->bdev.blockcnt has been updated.
	 * The JSON response is sent from the callback.
	 */
	rc = ec_bdev_resize(req->ec_name, rpc_bdev_ec_resize_cb, req);
	if (rc != 0) {
		const char *errmsg;
		switch (-rc) {
		case ENODEV:   errmsg = "EC bdev not found";             break;
		case EBUSY:    errmsg = "Another operation in progress"; break;
		case EIO:      errmsg = "EC bdev is offline";            break;
		case EALREADY: errmsg = "Base bdevs have not grown";     break;
		case ENOMEM:   errmsg = "Out of memory";                 break;
		default:       errmsg = spdk_strerror(-rc);              break;
		}
		spdk_jsonrpc_send_error_response(request, rc, errmsg);
		free_rpc_bdev_ec_resize(req);
	}
}
SPDK_RPC_REGISTER("bdev_ec_resize", rpc_bdev_ec_resize,
		  SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_set_rebuild_qos
 * ========================================================================= */

struct rpc_bdev_ec_set_rebuild_qos {
	char    *ec_name;
	uint32_t max_stripes_per_sec;
	bool     paused;
};

static void
free_rpc_bdev_ec_set_rebuild_qos(struct rpc_bdev_ec_set_rebuild_qos *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder
rpc_bdev_ec_set_rebuild_qos_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_set_rebuild_qos, ec_name),
	 spdk_json_decode_string},
	{"max_stripes_per_sec", offsetof(struct rpc_bdev_ec_set_rebuild_qos, max_stripes_per_sec),
	 spdk_json_decode_uint32, true},
	{"paused", offsetof(struct rpc_bdev_ec_set_rebuild_qos, paused),
	 spdk_json_decode_bool, true},
};

static void
rpc_bdev_ec_set_rebuild_qos(struct spdk_jsonrpc_request *request,
			    const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_set_rebuild_qos req = {};
	struct spdk_json_val *v_max = NULL, *v_paused = NULL;
	bool have_max, have_paused;
	int rc;

	if (spdk_json_decode_object(params,
				    rpc_bdev_ec_set_rebuild_qos_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_set_rebuild_qos_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for "
			    "bdev_ec_set_rebuild_qos\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	/*
	 * Both fields are optional in the decoder table. Without a probe, the
	 * zero-init defaults (0 / false) make "omitted (keep current)"
	 * indistinguishable from "set to 0 / false (clobber)". The
	 * spdk_json_find lookups below recover that signal so the merge can
	 * preserve absent fields.
	 *
	 * Order matters: spdk_json_find asserts on a NULL object, so it must
	 * run AFTER decode_object validates params. Decode is read-only on
	 * the JSON tree, so the lookups still see exactly what the caller
	 * sent.
	 */
	spdk_json_find((struct spdk_json_val *)params,
		       "max_stripes_per_sec", NULL, &v_max,
		       SPDK_JSON_VAL_NUMBER);
	spdk_json_find((struct spdk_json_val *)params,
		       "paused", NULL, &v_paused,
		       SPDK_JSON_VAL_TRUE | SPDK_JSON_VAL_FALSE);
	have_max    = (v_max    != NULL);
	have_paused = (v_paused != NULL);

	/*
	 * If a knob was omitted, fill it in from the live rebuild context so
	 * the absent field is left alone. Both omitted is a harmless re-apply;
	 * both present is the fast path (no extra lookup).
	 */
	if (!have_max || !have_paused) {
		struct ec_bdev *ec = ec_bdev_find(req.ec_name);

		if (!ec) {
			spdk_jsonrpc_send_error_response(request, -ENODEV,
							 "EC bdev not found");
			goto cleanup;
		}
		if (!ec->rebuild_ctx) {
			spdk_jsonrpc_send_error_response(request, -ENOENT,
							 "No rebuild in progress");
			goto cleanup;
		}
		if (!have_max) {
			req.max_stripes_per_sec = ec->rebuild_ctx->max_stripes_per_sec;
		}
		if (!have_paused) {
			req.paused = ec->rebuild_ctx->paused;
		}
	}

	rc = ec_bdev_set_rebuild_qos(req.ec_name,
				     req.max_stripes_per_sec,
				     req.paused);
	if (rc != 0) {
		const char *errmsg;
		switch (-rc) {
		case ENODEV: errmsg = "EC bdev not found";      break;
		case ENOENT: errmsg = "No rebuild in progress"; break;
		default:     errmsg = spdk_strerror(-rc);        break;
		}
		spdk_jsonrpc_send_error_response(request, rc, errmsg);
		goto cleanup;
	}

	spdk_jsonrpc_send_bool_response(request, true);

cleanup:
	free_rpc_bdev_ec_set_rebuild_qos(&req);
}
SPDK_RPC_REGISTER("bdev_ec_set_rebuild_qos", rpc_bdev_ec_set_rebuild_qos,
		  SPDK_RPC_RUNTIME)

/* =========================================================================
 * RPC: bdev_ec_stop_rebuild
 * ========================================================================= */

struct rpc_bdev_ec_stop_rebuild {
	char *ec_name;
};

static void
free_rpc_bdev_ec_stop_rebuild(struct rpc_bdev_ec_stop_rebuild *req)
{
	free(req->ec_name);
}

static const struct spdk_json_object_decoder
rpc_bdev_ec_stop_rebuild_decoders[] = {
	{"ec_name", offsetof(struct rpc_bdev_ec_stop_rebuild, ec_name),
	 spdk_json_decode_string},
};

static void
rpc_bdev_ec_stop_rebuild(struct spdk_jsonrpc_request *request,
			 const struct spdk_json_val *params)
{
	struct rpc_bdev_ec_stop_rebuild req = {};
	int rc;

	if (spdk_json_decode_object(params,
				    rpc_bdev_ec_stop_rebuild_decoders,
				    SPDK_COUNTOF(rpc_bdev_ec_stop_rebuild_decoders),
				    &req)) {
		SPDK_ERRLOG("spdk_json_decode_object failed for "
			    "bdev_ec_stop_rebuild\n");
		ec_rpc_send_decode_error(request);
		goto cleanup;
	}

	rc = ec_bdev_stop_rebuild(req.ec_name);
	if (rc != 0) {
		const char *errmsg;
		switch (-rc) {
		case ENODEV: errmsg = "EC bdev not found";      break;
		case ENOENT: errmsg = "No rebuild in progress"; break;
		default:     errmsg = spdk_strerror(-rc);        break;
		}
		spdk_jsonrpc_send_error_response(request, rc, errmsg);
		goto cleanup;
	}

	spdk_jsonrpc_send_bool_response(request, true);

cleanup:
	free_rpc_bdev_ec_stop_rebuild(&req);
}
SPDK_RPC_REGISTER("bdev_ec_stop_rebuild", rpc_bdev_ec_stop_rebuild,
		  SPDK_RPC_RUNTIME)
