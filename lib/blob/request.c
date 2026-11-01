/*   SPDX-License-Identifier: BSD-3-Clause
 *   Copyright (C) 2017 Intel Corporation.
 *   All rights reserved.
 *   Copyright (c) 2022-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#include "spdk/stdinc.h"

#include "blobstore.h"
#include "request.h"

#include "spdk/thread.h"
#include "spdk/queue.h"
#include "spdk/trace.h"

#include "spdk_internal/trace_defs.h"
#include "spdk/log.h"

void
bs_call_cpl(struct spdk_bs_cpl *cpl, int bserrno)
{
	switch (cpl->type) {
	case SPDK_BS_CPL_TYPE_BS_BASIC:
		cpl->u.bs_basic.cb_fn(cpl->u.bs_basic.cb_arg,
				      bserrno);
		break;
	case SPDK_BS_CPL_TYPE_BS_HANDLE:
		cpl->u.bs_handle.cb_fn(cpl->u.bs_handle.cb_arg,
				       bserrno == 0 ? cpl->u.bs_handle.bs : NULL,
				       bserrno);
		break;
	case SPDK_BS_CPL_TYPE_BLOB_BASIC:
		cpl->u.blob_basic.cb_fn(cpl->u.blob_basic.cb_arg,
					bserrno);
		break;
	case SPDK_BS_CPL_TYPE_BLOBID:
		cpl->u.blobid.cb_fn(cpl->u.blobid.cb_arg,
				    bserrno == 0 ? cpl->u.blobid.blobid : SPDK_BLOBID_INVALID,
				    bserrno);
		break;
	case SPDK_BS_CPL_TYPE_BLOB_HANDLE:
		cpl->u.blob_handle.cb_fn(cpl->u.blob_handle.cb_arg,
					 bserrno == 0 ? cpl->u.blob_handle.blob : NULL,
					 bserrno);
		break;
	case SPDK_BS_CPL_TYPE_NESTED_SEQUENCE:
		cpl->u.nested_seq.cb_fn(cpl->u.nested_seq.cb_arg,
					cpl->u.nested_seq.parent,
					bserrno);
		break;
	case SPDK_BS_CPL_TYPE_NONE:
		/* this completion's callback is handled elsewhere */
		break;
	}
}

static void
bs_request_set_complete(struct spdk_bs_request_set *set)
{
	struct spdk_bs_cpl cpl = set->cpl;
	struct blob_esnap_channel *esnap_ch = set->esnap_ch;
	struct spdk_blob *esnap_blob = set->esnap_blob;
	uint32_t esnap_reads = set->esnap_inflight;
	struct spdk_io_channel *ch = NULL;
	int bserrno = set->bserrno;

	spdk_trace_record(TRACE_BLOB_REQ_SET_COMPLETE, 0, 0, (uintptr_t)&set->cb_args,
			  (uintptr_t)set->cpl.u.blob_basic.cb_arg);

	/* The completion callback may reuse this request set. Keep the old esnap reads
	 * protected until the entire request, including its callback, has completed.
	 */
	set->esnap_ch = NULL;
	set->esnap_blob = NULL;
	set->esnap_inflight = 0;
	TAILQ_INSERT_TAIL(&set->channel->reqs, set, link);

	if (esnap_reads != 0) {
		/* The callback may release its bs channel. Keep it until the esnap
		 * request's final reference to the channel is dropped.
		 */
		ch = spdk_io_channel_ref(spdk_io_channel_from_ctx(set->channel));
	}
	bs_call_cpl(&cpl, bserrno);
	if (esnap_reads != 0) {
		blob_esnap_channel_io_done(esnap_ch, esnap_reads);
		spdk_put_io_channel(ch);
	}
	if (esnap_blob != NULL) {
		blob_close_barrier_put(esnap_blob);
	}
}

static void
bs_sequence_completion(struct spdk_io_channel *channel, void *cb_arg, int bserrno)
{
	struct spdk_bs_request_set *set = cb_arg;

	set->bserrno = bserrno;
	set->u.sequence.cb_fn((spdk_bs_sequence_t *)set, set->u.sequence.cb_arg, bserrno);
}

static inline spdk_bs_sequence_t *
bs_sequence_start(struct spdk_io_channel *_channel, struct spdk_bs_cpl *cpl,
		  struct spdk_io_channel *back_channel, struct blob_esnap_channel *esnap_ch)
{
	struct spdk_bs_channel		*channel;
	struct spdk_bs_request_set	*set;

	channel = spdk_io_channel_get_ctx(_channel);
	assert(channel != NULL);
	set = TAILQ_FIRST(&channel->reqs);
	if (!set) {
		return NULL;
	}
	TAILQ_REMOVE(&channel->reqs, set, link);

	spdk_trace_record(TRACE_BLOB_REQ_SET_START, 0, 0, (uintptr_t)&set->cb_args,
			  (uintptr_t)cpl->u.blob_basic.cb_arg);

	set->cpl = *cpl;
	set->bserrno = 0;
	set->channel = channel;
	set->back_channel = back_channel;
	set->esnap_ch = esnap_ch;
	set->esnap_blob = NULL;
	set->esnap_inflight = 0;

	set->cb_args.cb_fn = bs_sequence_completion;
	set->cb_args.cb_arg = set;
	set->cb_args.channel = channel->dev_channel;
	set->ext_io_opts = NULL;

	return (spdk_bs_sequence_t *)set;
}

/* Use when performing IO directly on the blobstore (e.g. metadata - not a blob). */
spdk_bs_sequence_t *
bs_sequence_start_bs(struct spdk_io_channel *_channel, struct spdk_bs_cpl *cpl)
{
	return bs_sequence_start(_channel, cpl, _channel, NULL);
}

/* Use when performing IO on a blob. */
spdk_bs_sequence_t *
bs_sequence_start_blob(struct spdk_io_channel *_channel, struct spdk_bs_cpl *cpl,
		       struct spdk_blob *blob, bool *barrier_held)
{
	struct spdk_io_channel		*back_channel = _channel;
	struct blob_esnap_channel	*esnap_channel = NULL;
	spdk_bs_sequence_t		*seq;

	*barrier_held = false;
	if (spdk_blob_is_esnap_clone(blob)) {
		/* A close may be draining an older channel while this request creates a new one. */
		blob_close_barrier_get(blob);
		*barrier_held = true;
		esnap_channel = blob_esnap_get_channel(_channel, blob);
		if (esnap_channel == NULL) {
			return NULL;
		}
		back_channel = blob_esnap_channel_io_channel(esnap_channel);
	}
	seq = bs_sequence_start(_channel, cpl, back_channel, esnap_channel);
	if (seq != NULL && esnap_channel != NULL) {
		seq->esnap_blob = blob;
		*barrier_held = false;
	}
	return seq;
}

/* Counts a read submitted on the esnap channel of the request. Must be called before the read. */
static inline void
bs_request_set_esnap_io_start(struct spdk_bs_request_set *set)
{
	if (set->esnap_ch != NULL) {
		set->esnap_inflight++;
		blob_esnap_channel_io_start(set->esnap_ch);
	}
}

void
bs_sequence_read_bs_dev(spdk_bs_sequence_t *seq, struct spdk_bs_dev *bs_dev,
			void *payload, uint64_t lba, uint32_t lba_count,
			spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)seq;
	struct spdk_io_channel		*back_channel = set->back_channel;

	SPDK_DEBUGLOG(blob_rw, "Reading %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	bs_request_set_esnap_io_start(set);
	bs_dev->read(bs_dev, back_channel, payload, lba, lba_count, &set->cb_args);
}

void
bs_sequence_read_dev(spdk_bs_sequence_t *seq, void *payload,
		     uint64_t lba, uint32_t lba_count,
		     spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set      *set = (struct spdk_bs_request_set *)seq;
	struct spdk_bs_channel       *channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Reading %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	channel->dev->read(channel->dev, channel->dev_channel, payload, lba, lba_count, &set->cb_args);
}

void
bs_sequence_write_dev(spdk_bs_sequence_t *seq, void *payload,
		      uint64_t lba, uint32_t lba_count,
		      spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set      *set = (struct spdk_bs_request_set *)seq;
	struct spdk_bs_channel       *channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Writing %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	channel->dev->write(channel->dev, channel->dev_channel, payload, lba, lba_count,
			    &set->cb_args);
}

void
bs_sequence_readv_bs_dev(spdk_bs_sequence_t *seq, struct spdk_bs_dev *bs_dev,
			 struct iovec *iov, int iovcnt, uint64_t lba, uint32_t lba_count,
			 spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set      *set = (struct spdk_bs_request_set *)seq;
	struct spdk_io_channel		*back_channel = set->back_channel;

	SPDK_DEBUGLOG(blob_rw, "Reading %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	bs_request_set_esnap_io_start(set);
	if (set->ext_io_opts) {
		assert(bs_dev->readv_ext);
		bs_dev->readv_ext(bs_dev, back_channel, iov, iovcnt, lba, lba_count,
				  &set->cb_args, set->ext_io_opts);
	} else {
		bs_dev->readv(bs_dev, back_channel, iov, iovcnt, lba, lba_count, &set->cb_args);
	}
}

void
bs_sequence_readv_dev(spdk_bs_sequence_t *seq, struct iovec *iov, int iovcnt,
		      uint64_t lba, uint32_t lba_count, spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set      *set = (struct spdk_bs_request_set *)seq;
	struct spdk_bs_channel       *channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Reading %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;
	if (set->ext_io_opts) {
		assert(channel->dev->readv_ext);
		channel->dev->readv_ext(channel->dev, channel->dev_channel, iov, iovcnt, lba, lba_count,
					&set->cb_args, set->ext_io_opts);
	} else {
		channel->dev->readv(channel->dev, channel->dev_channel, iov, iovcnt, lba, lba_count, &set->cb_args);
	}
}

void
bs_sequence_writev_dev(spdk_bs_sequence_t *seq, struct iovec *iov, int iovcnt,
		       uint64_t lba, uint32_t lba_count,
		       spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set      *set = (struct spdk_bs_request_set *)seq;
	struct spdk_bs_channel       *channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Writing %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	if (set->ext_io_opts) {
		assert(channel->dev->writev_ext);
		channel->dev->writev_ext(channel->dev, channel->dev_channel, iov, iovcnt, lba, lba_count,
					 &set->cb_args, set->ext_io_opts);
	} else {
		channel->dev->writev(channel->dev, channel->dev_channel, iov, iovcnt, lba, lba_count,
				     &set->cb_args);
	}
}

void
bs_sequence_write_zeroes_dev(spdk_bs_sequence_t *seq,
			     uint64_t lba, uint64_t lba_count,
			     spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set      *set = (struct spdk_bs_request_set *)seq;
	struct spdk_bs_channel       *channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "writing zeroes to %" PRIu64 " blocks at LBA %" PRIu64 "\n",
		      lba_count, lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	channel->dev->write_zeroes(channel->dev, channel->dev_channel, lba, lba_count,
				   &set->cb_args);
}

void
bs_sequence_copy_dev(spdk_bs_sequence_t *seq, uint64_t dst_lba, uint64_t src_lba,
		     uint64_t lba_count, spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set *set = (struct spdk_bs_request_set *)seq;
	struct spdk_bs_channel     *channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Copying %" PRIu64 " blocks from LBA %" PRIu64 " to LBA %" PRIu64 "\n",
		      lba_count, src_lba, dst_lba);

	set->u.sequence.cb_fn = cb_fn;
	set->u.sequence.cb_arg = cb_arg;

	channel->dev->copy(channel->dev, channel->dev_channel, dst_lba, src_lba, lba_count, &set->cb_args);
}

void
bs_sequence_finish(spdk_bs_sequence_t *seq, int bserrno)
{
	if (bserrno != 0) {
		seq->bserrno = bserrno;
	}
	bs_request_set_complete((struct spdk_bs_request_set *)seq);
}

void
bs_user_op_sequence_finish(void *cb_arg, int bserrno)
{
	spdk_bs_sequence_t *seq = cb_arg;

	bs_sequence_finish(seq, bserrno);
}

static void
bs_batch_completion(struct spdk_io_channel *_channel,
		    void *cb_arg, int bserrno)
{
	struct spdk_bs_request_set	*set = cb_arg;

	set->u.batch.outstanding_ops--;
	if (bserrno != 0) {
		set->bserrno = bserrno;
	}

	if (set->u.batch.outstanding_ops == 0 && set->u.batch.batch_closed) {
		if (set->u.batch.cb_fn) {
			set->cb_args.cb_fn = bs_sequence_completion;
			set->u.batch.cb_fn((spdk_bs_sequence_t *)set, set->u.batch.cb_arg, bserrno);
		} else {
			bs_request_set_complete(set);
		}
	}
}

/* The blob is only used to find the esnap channel. It may be NULL for IO on the blobstore only. */
spdk_bs_batch_t *
bs_batch_open(struct spdk_io_channel *_channel, struct spdk_bs_cpl *cpl, struct spdk_blob *blob,
	      bool *barrier_held)
{
	struct spdk_bs_channel		*channel;
	struct spdk_bs_request_set	*set;
	struct spdk_io_channel		*back_channel = _channel;
	struct blob_esnap_channel	*esnap_channel = NULL;

	*barrier_held = false;
	if (blob != NULL && spdk_blob_is_esnap_clone(blob)) {
		blob_close_barrier_get(blob);
		*barrier_held = true;
		esnap_channel = blob_esnap_get_channel(_channel, blob);
		if (esnap_channel == NULL) {
			return NULL;
		}
		back_channel = blob_esnap_channel_io_channel(esnap_channel);
	}

	channel = spdk_io_channel_get_ctx(_channel);
	assert(channel != NULL);
	set = TAILQ_FIRST(&channel->reqs);
	if (!set) {
		return NULL;
	}
	TAILQ_REMOVE(&channel->reqs, set, link);

	spdk_trace_record(TRACE_BLOB_REQ_SET_START, 0, 0, (uintptr_t)&set->cb_args,
			  (uintptr_t)cpl->u.blob_basic.cb_arg);

	set->cpl = *cpl;
	set->bserrno = 0;
	set->channel = channel;
	set->back_channel = back_channel;
	set->esnap_ch = esnap_channel;
	set->esnap_blob = esnap_channel != NULL ? blob : NULL;
	*barrier_held = false;
	set->esnap_inflight = 0;

	set->u.batch.cb_fn = NULL;
	set->u.batch.cb_arg = NULL;
	set->u.batch.outstanding_ops = 0;
	set->u.batch.batch_closed = 0;

	set->cb_args.cb_fn = bs_batch_completion;
	set->cb_args.cb_arg = set;
	set->cb_args.channel = channel->dev_channel;

	return (spdk_bs_batch_t *)set;
}

void
bs_batch_read_bs_dev(spdk_bs_batch_t *batch, struct spdk_bs_dev *bs_dev,
		     void *payload, uint64_t lba, uint32_t lba_count)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)batch;
	struct spdk_io_channel		*back_channel = set->back_channel;

	SPDK_DEBUGLOG(blob_rw, "Reading %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.batch.outstanding_ops++;
	bs_request_set_esnap_io_start(set);
	bs_dev->read(bs_dev, back_channel, payload, lba, lba_count, &set->cb_args);
}

void
bs_batch_read_dev(spdk_bs_batch_t *batch, void *payload,
		  uint64_t lba, uint32_t lba_count)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)batch;
	struct spdk_bs_channel		*channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Reading %" PRIu32 " blocks from LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.batch.outstanding_ops++;
	channel->dev->read(channel->dev, channel->dev_channel, payload, lba, lba_count, &set->cb_args);
}

void
bs_batch_write_dev(spdk_bs_batch_t *batch, void *payload,
		   uint64_t lba, uint32_t lba_count)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)batch;
	struct spdk_bs_channel		*channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Writing %" PRIu32 " blocks to LBA %" PRIu64 "\n", lba_count, lba);

	set->u.batch.outstanding_ops++;
	channel->dev->write(channel->dev, channel->dev_channel, payload, lba, lba_count,
			    &set->cb_args);
}

void
bs_batch_unmap_dev(spdk_bs_batch_t *batch,
		   uint64_t lba, uint64_t lba_count)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)batch;
	struct spdk_bs_channel		*channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Unmapping %" PRIu64 " blocks at LBA %" PRIu64 "\n", lba_count,
		      lba);

	set->u.batch.outstanding_ops++;
	channel->dev->unmap(channel->dev, channel->dev_channel, lba, lba_count,
			    &set->cb_args);
}

void
bs_batch_write_zeroes_dev(spdk_bs_batch_t *batch,
			  uint64_t lba, uint64_t lba_count)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)batch;
	struct spdk_bs_channel		*channel = set->channel;

	SPDK_DEBUGLOG(blob_rw, "Zeroing %" PRIu64 " blocks at LBA %" PRIu64 "\n", lba_count, lba);

	set->u.batch.outstanding_ops++;
	channel->dev->write_zeroes(channel->dev, channel->dev_channel, lba, lba_count,
				   &set->cb_args);
}

void
bs_batch_close(spdk_bs_batch_t *batch)
{
	struct spdk_bs_request_set	*set = (struct spdk_bs_request_set *)batch;

	set->u.batch.batch_closed = 1;

	if (set->u.batch.outstanding_ops == 0) {
		if (set->u.batch.cb_fn) {
			set->cb_args.cb_fn = bs_sequence_completion;
			set->u.batch.cb_fn((spdk_bs_sequence_t *)set, set->u.batch.cb_arg, set->bserrno);
		} else {
			bs_request_set_complete(set);
		}
	}
}

spdk_bs_batch_t *
bs_sequence_to_batch(spdk_bs_sequence_t *seq, spdk_bs_sequence_cpl cb_fn, void *cb_arg)
{
	struct spdk_bs_request_set *set = (struct spdk_bs_request_set *)seq;

	set->u.batch.cb_fn = cb_fn;
	set->u.batch.cb_arg = cb_arg;
	set->u.batch.outstanding_ops = 0;
	set->u.batch.batch_closed = 0;

	set->cb_args.cb_fn = bs_batch_completion;

	return set;
}

spdk_bs_user_op_t *
bs_user_op_alloc(struct spdk_io_channel *_channel, struct spdk_bs_cpl *cpl,
		 enum spdk_blob_op_type op_type, struct spdk_blob *blob,
		 void *payload, int iovcnt, uint64_t offset, uint64_t length)
{
	struct spdk_bs_channel		*channel;
	struct spdk_bs_request_set	*set;
	struct spdk_bs_user_op_args	*args;

	channel = spdk_io_channel_get_ctx(_channel);
	assert(channel != NULL);
	set = TAILQ_FIRST(&channel->reqs);
	if (!set) {
		return NULL;
	}
	TAILQ_REMOVE(&channel->reqs, set, link);

	spdk_trace_record(TRACE_BLOB_REQ_SET_START, 0, 0, (uintptr_t)&set->cb_args,
			  (uintptr_t)cpl->u.blob_basic.cb_arg);

	set->cpl = *cpl;
	set->channel = channel;
	set->back_channel = NULL;
	set->esnap_ch = NULL;
	set->esnap_blob = NULL;
	set->esnap_inflight = 0;
	set->ext_io_opts = NULL;

	args = &set->u.user_op;

	args->type = op_type;
	args->iovcnt = iovcnt;
	args->blob = blob;
	args->offset = offset;
	args->length = length;
	args->payload = payload;

	return (spdk_bs_user_op_t *)set;
}

void
bs_user_op_execute(spdk_bs_user_op_t *op)
{
	struct spdk_bs_request_set	*set;
	struct spdk_bs_user_op_args	*args;
	struct spdk_io_channel		*ch;

	set = (struct spdk_bs_request_set *)op;
	args = &set->u.user_op;
	ch = spdk_io_channel_from_ctx(set->channel);

	switch (args->type) {
	case SPDK_BLOB_READ:
		spdk_blob_io_read(args->blob, ch, args->payload, args->offset, args->length,
				  set->cpl.u.blob_basic.cb_fn, set->cpl.u.blob_basic.cb_arg);
		break;
	case SPDK_BLOB_WRITE:
		spdk_blob_io_write(args->blob, ch, args->payload, args->offset, args->length,
				   set->cpl.u.blob_basic.cb_fn, set->cpl.u.blob_basic.cb_arg);
		break;
	case SPDK_BLOB_UNMAP:
		spdk_blob_io_unmap(args->blob, ch, args->offset, args->length,
				   set->cpl.u.blob_basic.cb_fn, set->cpl.u.blob_basic.cb_arg);
		break;
	case SPDK_BLOB_WRITE_ZEROES:
		spdk_blob_io_write_zeroes(args->blob, ch, args->offset, args->length,
					  set->cpl.u.blob_basic.cb_fn, set->cpl.u.blob_basic.cb_arg);
		break;
	case SPDK_BLOB_READV:
		spdk_blob_io_readv_ext(args->blob, ch, args->payload, args->iovcnt,
				       args->offset, args->length,
				       set->cpl.u.blob_basic.cb_fn, set->cpl.u.blob_basic.cb_arg,
				       set->ext_io_opts);
		break;
	case SPDK_BLOB_WRITEV:
		spdk_blob_io_writev_ext(args->blob, ch, args->payload, args->iovcnt,
					args->offset, args->length,
					set->cpl.u.blob_basic.cb_fn, set->cpl.u.blob_basic.cb_arg,
					set->ext_io_opts);
		break;
	}
	TAILQ_INSERT_TAIL(&set->channel->reqs, set, link);
}

void
bs_user_op_abort(spdk_bs_user_op_t *op, int bserrno)
{
	struct spdk_bs_request_set	*set;

	set = (struct spdk_bs_request_set *)op;

	set->cpl.u.blob_basic.cb_fn(set->cpl.u.blob_basic.cb_arg, bserrno);
	TAILQ_INSERT_TAIL(&set->channel->reqs, set, link);
}

SPDK_LOG_REGISTER_COMPONENT(blob_rw)
