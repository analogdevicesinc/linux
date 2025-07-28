// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2016-2018 Oracle.  All rights reserved.
 *
 * Use the core R/W API to move RPC-over-RDMA Read and Write chunks.
 */

#include <rdma/rw.h>

#include <linux/sunrpc/xdr.h>
#include <linux/sunrpc/rpc_rdma.h>
#include <linux/sunrpc/svc_rdma.h>

#include "xprt_rdma.h"
#include <trace/events/rpcrdma.h>

static void svc_rdma_write_done(struct ib_cq *cq, struct ib_wc *wc);
static void svc_rdma_wc_read_done(struct ib_cq *cq, struct ib_wc *wc);

/* Each R/W context contains state for one chain of RDMA Read or
 * Write Work Requests.
 *
 * Each WR chain handles a single contiguous server-side buffer,
 * because scatterlist entries after the first have to start on
 * page alignment. xdr_buf iovecs cannot guarantee alignment.
 *
 * Each WR chain handles only one R_key. Each RPC-over-RDMA segment
 * from a client may contain a unique R_key, so each WR chain moves
 * up to one segment at a time.
 *
 * The scatterlist makes this data structure over 4KB in size. To
 * make it less likely to fail, and to handle the allocation for
 * smaller I/O requests without disabling bottom-halves, these
 * contexts are created on demand, but cached and reused until the
 * controlling svcxprt_rdma is destroyed.
 */
struct svc_rdma_rw_ctxt {
	struct llist_node	rw_node;
	struct list_head	rw_list;
	struct rdma_rw_ctx	rw_ctx;
	unsigned int		rw_nents;
	unsigned int		rw_first_sgl_nents;
	struct sg_table		rw_sg_table;
	struct scatterlist	rw_first_sgl[];
};

static inline struct svc_rdma_rw_ctxt *
svc_rdma_next_ctxt(struct list_head *list)
{
	return list_first_entry_or_null(list, struct svc_rdma_rw_ctxt,
					rw_list);
}

static struct svc_rdma_rw_ctxt *
svc_rdma_get_rw_ctxt(struct svcxprt_rdma *rdma, unsigned int sges)
{
	struct ib_device *dev = rdma->sc_cm_id->device;
	unsigned int first_sgl_nents = dev->attrs.max_send_sge;
	struct svc_rdma_rw_ctxt *ctxt;
	struct llist_node *node;

	spin_lock(&rdma->sc_rw_ctxt_lock);
	node = llist_del_first(&rdma->sc_rw_ctxts);
	spin_unlock(&rdma->sc_rw_ctxt_lock);
	if (node) {
		ctxt = llist_entry(node, struct svc_rdma_rw_ctxt, rw_node);
	} else {
		ctxt = kmalloc_node(struct_size(ctxt, rw_first_sgl, first_sgl_nents),
				    GFP_KERNEL, ibdev_to_node(dev));
		if (!ctxt)
			goto out_noctx;

		INIT_LIST_HEAD(&ctxt->rw_list);
		ctxt->rw_first_sgl_nents = first_sgl_nents;
	}

	ctxt->rw_sg_table.sgl = ctxt->rw_first_sgl;
	if (sg_alloc_table_chained(&ctxt->rw_sg_table, sges,
				   ctxt->rw_sg_table.sgl,
				   first_sgl_nents))
		goto out_free;
	return ctxt;

out_free:
	kfree(ctxt);
out_noctx:
	trace_svcrdma_rwctx_empty(rdma, sges);
	return NULL;
}

static void __svc_rdma_put_rw_ctxt(struct svc_rdma_rw_ctxt *ctxt,
				   struct llist_head *list)
{
	sg_free_table_chained(&ctxt->rw_sg_table, ctxt->rw_first_sgl_nents);
	llist_add(&ctxt->rw_node, list);
}

static void svc_rdma_put_rw_ctxt(struct svcxprt_rdma *rdma,
				 struct svc_rdma_rw_ctxt *ctxt)
{
	__svc_rdma_put_rw_ctxt(ctxt, &rdma->sc_rw_ctxts);
}

/**
 * svc_rdma_destroy_rw_ctxts - Free accumulated R/W contexts
 * @rdma: transport about to be destroyed
 *
 */
void svc_rdma_destroy_rw_ctxts(struct svcxprt_rdma *rdma)
{
	struct svc_rdma_rw_ctxt *ctxt;
	struct llist_node *node;

	while ((node = llist_del_first(&rdma->sc_rw_ctxts)) != NULL) {
		ctxt = llist_entry(node, struct svc_rdma_rw_ctxt, rw_node);
		kfree(ctxt);
	}
}

/**
 * svc_rdma_rw_ctx_init - Prepare a R/W context for I/O
 * @rdma: controlling transport instance
 * @ctxt: R/W context to prepare
 * @offset: RDMA offset
 * @handle: RDMA tag/handle
 * @direction: I/O direction
 *
 * Returns on success, the number of WQEs that will be needed
 * on the workqueue, or a negative errno.
 */
static int svc_rdma_rw_ctx_init(struct svcxprt_rdma *rdma,
				struct svc_rdma_rw_ctxt *ctxt,
				u64 offset, u32 handle,
				enum dma_data_direction direction)
{
	int ret;

	ret = rdma_rw_ctx_init(&ctxt->rw_ctx, rdma->sc_qp, rdma->sc_port_num,
			       ctxt->rw_sg_table.sgl, ctxt->rw_nents,
			       0, offset, handle, direction);
	if (unlikely(ret < 0)) {
		trace_svcrdma_dma_map_rw_err(rdma, offset, handle,
					     ctxt->rw_nents, ret);
		svc_rdma_put_rw_ctxt(rdma, ctxt);
	}
	return ret;
}

/**
 * svc_rdma_cc_init - Initialize an svc_rdma_chunk_ctxt
 * @rdma: controlling transport instance
 * @cc: svc_rdma_chunk_ctxt to be initialized
 */
void svc_rdma_cc_init(struct svcxprt_rdma *rdma,
		      struct svc_rdma_chunk_ctxt *cc)
{
	struct rpc_rdma_cid *cid = &cc->cc_cid;

	if (unlikely(!cid->ci_completion_id))
		svc_rdma_send_cid_init(rdma, cid);

	INIT_LIST_HEAD(&cc->cc_rwctxts);
	cc->cc_sqecount = 0;
}

/**
 * svc_rdma_cc_release - Release resources held by a svc_rdma_chunk_ctxt
 * @rdma: controlling transport instance
 * @cc: svc_rdma_chunk_ctxt to be released
 * @dir: DMA direction
 */
void svc_rdma_cc_release(struct svcxprt_rdma *rdma,
			 struct svc_rdma_chunk_ctxt *cc,
			 enum dma_data_direction dir)
{
	struct llist_node *first, *last;
	struct svc_rdma_rw_ctxt *ctxt;
	LLIST_HEAD(free);

	trace_svcrdma_cc_release(&cc->cc_cid, cc->cc_sqecount);

	first = last = NULL;
	while ((ctxt = svc_rdma_next_ctxt(&cc->cc_rwctxts)) != NULL) {
		list_del(&ctxt->rw_list);

		rdma_rw_ctx_destroy(&ctxt->rw_ctx, rdma->sc_qp,
				    rdma->sc_port_num, ctxt->rw_sg_table.sgl,
				    ctxt->rw_nents, dir);
		__svc_rdma_put_rw_ctxt(ctxt, &free);

		ctxt->rw_node.next = first;
		first = &ctxt->rw_node;
		if (!last)
			last = first;
	}
	if (first)
		llist_add_batch(first, last, &rdma->sc_rw_ctxts);
}

static struct svc_rdma_write_info *
svc_rdma_write_info_alloc(struct svcxprt_rdma *rdma,
			  const struct svc_rdma_chunk *chunk)
{
	struct svc_rdma_write_info *info;

	info = kzalloc_node(sizeof(*info), GFP_KERNEL,
			    ibdev_to_node(rdma->sc_cm_id->device));
	if (!info)
		return info;

	info->wi_rdma = rdma;
	info->wi_chunk = chunk;
	svc_rdma_cc_init(rdma, &info->wi_cc);
	info->wi_cc.cc_cqe.done = svc_rdma_write_done;
	return info;
}

static void svc_rdma_write_info_free_async(struct work_struct *work)
{
	struct svc_rdma_write_info *info;

	info = container_of(work, struct svc_rdma_write_info, wi_work);
	svc_rdma_cc_release(info->wi_rdma, &info->wi_cc, DMA_TO_DEVICE);
	kfree(info);
}

static void svc_rdma_write_info_free(struct svc_rdma_write_info *info)
{
	INIT_WORK(&info->wi_work, svc_rdma_write_info_free_async);
	queue_work(svcrdma_wq, &info->wi_work);
}

/**
 * svc_rdma_reply_chunk_release - Release Reply chunk I/O resources
 * @rdma: controlling transport
 * @ctxt: Send context that is being released
 */
void svc_rdma_reply_chunk_release(struct svcxprt_rdma *rdma,
				  struct svc_rdma_send_ctxt *ctxt)
{
	struct svc_rdma_chunk_ctxt *cc = &ctxt->sc_reply_info.wi_cc;

	if (!cc->cc_sqecount)
		return;
	svc_rdma_cc_release(rdma, cc, DMA_TO_DEVICE);
}

/**
 * svc_rdma_reply_done - Reply chunk Write completion handler
 * @cq: controlling Completion Queue
 * @wc: Work Completion report
 *
 * Pages under I/O are released by a subsequent Send completion.
 */
static void svc_rdma_reply_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct ib_cqe *cqe = wc->wr_cqe;
	struct svc_rdma_chunk_ctxt *cc =
			container_of(cqe, struct svc_rdma_chunk_ctxt, cc_cqe);
	struct svcxprt_rdma *rdma = cq->cq_context;

	switch (wc->status) {
	case IB_WC_SUCCESS:
		trace_svcrdma_wc_reply(&cc->cc_cid);
		return;
	case IB_WC_WR_FLUSH_ERR:
		trace_svcrdma_wc_reply_flush(wc, &cc->cc_cid);
		break;
	default:
		trace_svcrdma_wc_reply_err(wc, &cc->cc_cid);
	}

	svc_xprt_deferred_close(&rdma->sc_xprt);
}

/**
 * svc_rdma_write_done - Write chunk completion
 * @cq: controlling Completion Queue
 * @wc: Work Completion
 *
 * Pages under I/O are freed by a subsequent Send completion.
 */
static void svc_rdma_write_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct svcxprt_rdma *rdma = cq->cq_context;
	struct ib_cqe *cqe = wc->wr_cqe;
	struct svc_rdma_chunk_ctxt *cc =
			container_of(cqe, struct svc_rdma_chunk_ctxt, cc_cqe);
	struct svc_rdma_write_info *info =
			container_of(cc, struct svc_rdma_write_info, wi_cc);

	switch (wc->status) {
	case IB_WC_SUCCESS:
		trace_svcrdma_wc_write(&cc->cc_cid);
		break;
	case IB_WC_WR_FLUSH_ERR:
		trace_svcrdma_wc_write_flush(wc, &cc->cc_cid);
		break;
	default:
		trace_svcrdma_wc_write_err(wc, &cc->cc_cid);
	}

	svc_rdma_wake_send_waiters(rdma, cc->cc_sqecount);

	if (unlikely(wc->status != IB_WC_SUCCESS))
		svc_xprt_deferred_close(&rdma->sc_xprt);

	svc_rdma_write_info_free(info);
}

/**
 * svc_rdma_wc_read_done - Handle completion of an RDMA Read ctx
 * @cq: controlling Completion Queue
 * @wc: Work Completion
 *
 */
static void svc_rdma_wc_read_done(struct ib_cq *cq, struct ib_wc *wc)
{
	struct svcxprt_rdma *rdma = cq->cq_context;
	struct ib_cqe *cqe = wc->wr_cqe;
	struct svc_rdma_chunk_ctxt *cc =
			container_of(cqe, struct svc_rdma_chunk_ctxt, cc_cqe);
	struct svc_rdma_recv_ctxt *ctxt;

	svc_rdma_wake_send_waiters(rdma, cc->cc_sqecount);

	ctxt = container_of(cc, struct svc_rdma_recv_ctxt, rc_cc);
	switch (wc->status) {
	case IB_WC_SUCCESS:
		trace_svcrdma_wc_read(wc, &cc->cc_cid, ctxt->rc_readbytes,
				      cc->cc_posttime);

		spin_lock(&rdma->sc_rq_dto_lock);
		list_add_tail(&ctxt->rc_list, &rdma->sc_read_complete_q);
		/* the unlock pairs with the smp_rmb in svc_xprt_ready */
		set_bit(XPT_DATA, &rdma->sc_xprt.xpt_flags);
		spin_unlock(&rdma->sc_rq_dto_lock);
		svc_xprt_enqueue(&rdma->sc_xprt);
		return;
	case IB_WC_WR_FLUSH_ERR:
		trace_svcrdma_wc_read_flush(wc, &cc->cc_cid);
		break;
	default:
		trace_svcrdma_wc_read_err(wc, &cc->cc_cid);
	}

	/* The RDMA Read has flushed, so the incoming RPC message
	 * cannot be constructed and must be dropped. Signal the
	 * loss to the client by closing the connection.
	 */
	svc_rdma_cc_release(rdma, cc, DMA_FROM_DEVICE);
	svc_rdma_recv_ctxt_put(rdma, ctxt);
	svc_xprt_deferred_close(&rdma->sc_xprt);
}

/*
 * Assumptions:
 * - If ib_post_send() succeeds, only one completion is expected,
 *   even if one or more WRs are flushed. This is true when posting
 *   an rdma_rw_ctx or when posting a single signaled WR.
 */
static int svc_rdma_post_chunk_ctxt(struct svcxprt_rdma *rdma,
				    struct svc_rdma_chunk_ctxt *cc)
{
	struct ib_send_wr *first_wr;
	const struct ib_send_wr *bad_wr;
	struct list_head *tmp;
	struct ib_cqe *cqe;
	int ret;

	might_sleep();

	if (cc->cc_sqecount > rdma->sc_sq_depth)
		return -EINVAL;

	first_wr = NULL;
	cqe = &cc->cc_cqe;
	list_for_each(tmp, &cc->cc_rwctxts) {
		struct svc_rdma_rw_ctxt *ctxt;

		ctxt = list_entry(tmp, struct svc_rdma_rw_ctxt, rw_list);
		first_wr = rdma_rw_ctx_wrs(&ctxt->rw_ctx, rdma->sc_qp,
					   rdma->sc_port_num, cqe, first_wr);
		cqe = NULL;
	}

	do {
		if (atomic_sub_return(cc->cc_sqecount,
				      &rdma->sc_sq_avail) > 0) {
			cc->cc_posttime = ktime_get();
			ret = ib_post_send(rdma->sc_qp, first_wr, &bad_wr);
			if (ret)
				break;
			return 0;
		}

		percpu_counter_inc(&svcrdma_stat_sq_starve);
		trace_svcrdma_sq_full(rdma, &cc->cc_cid);
		atomic_add(cc->cc_sqecount, &rdma->sc_sq_avail);
		wait_event(rdma->sc_send_wait,
			   atomic_read(&rdma->sc_sq_avail) > cc->cc_sqecount);
		trace_svcrdma_sq_retry(rdma, &cc->cc_cid);
	} while (1);

	trace_svcrdma_sq_post_err(rdma, &cc->cc_cid, ret);
	svc_xprt_deferred_close(&rdma->sc_xprt);

	/* If even one was posted, there will be a completion. */
	if (bad_wr != first_wr)
		return 0;

	atomic_add(cc->cc_sqecount, &rdma->sc_sq_avail);
	wake_up(&rdma->sc_send_wait);
	return -ENOTCONN;
}

/* Build and DMA-map an SGL that covers one kvec in an xdr_buf
 */
static void svc_rdma_vec_to_sg(struct svc_rdma_write_info *info,
			       unsigned int len,
			       struct svc_rdma_rw_ctxt *ctxt)
{
	struct scatterlist *sg = ctxt->rw_sg_table.sgl;

	sg_set_buf(&sg[0], info->wi_base, len);
	info->wi_base += len;

	ctxt->rw_nents = 1;
}

/* Build and DMA-map an SGL that covers part of an xdr_buf's pagelist.
 */
static void svc_rdma_pagelist_to_sg(struct svc_rdma_write_info *info,
				    unsigned int remaining,
				    struct svc_rdma_rw_ctxt *ctxt)
{
	unsigned int sge_no, sge_bytes, page_off, page_no;
	const struct xdr_buf *xdr = info->wi_xdr;
	struct scatterlist *sg;
	struct page **page;

	page_off = info->wi_next_off + xdr->page_base;
	page_no = page_off >> PAGE_SHIFT;
	page_off = offset_in_page(page_off);
	page = xdr->pages + page_no;
	info->wi_next_off += remaining;
	sg = ctxt->rw_sg_table.sgl;
	sge_no = 0;
	do {
		sge_bytes = min_t(unsigned int, remaining,
				  PAGE_SIZE - page_off);
		sg_set_page(sg, *page, sge_bytes, page_off);

		remaining -= sge_bytes;
		sg = sg_next(sg);
		page_off = 0;
		sge_no++;
		page++;
	} while (remaining);

	ctxt->rw_nents = sge_no;
}

/* Construct RDMA Write WRs to send a portion of an xdr_buf containing
 * an RPC Reply.
 */
static int
svc_rdma_build_writes(struct svc_rdma_write_info *info,
		      void (*constructor)(struct svc_rdma_write_info *info,
					  unsigned int len,
					  struct svc_rdma_rw_ctxt *ctxt),
		      unsigned int remaining)
{
	struct svc_rdma_chunk_ctxt *cc = &info->wi_cc;
	struct svcxprt_rdma *rdma = info->wi_rdma;
	const struct svc_rdma_segment *seg;
	struct svc_rdma_rw_ctxt *ctxt;
	int ret;

	do {
		unsigned int write_len;
		u64 offset;

		if (info->wi_seg_no >= info->wi_chunk->ch_segcount)
			goto out_overflow;

		seg = &info->wi_chunk->ch_segments[info->wi_seg_no];
		write_len = min(remaining, seg->rs_length - info->wi_seg_off);
		if (!write_len)
			goto out_overflow;
		ctxt = svc_rdma_get_rw_ctxt(rdma,
					    (write_len >> PAGE_SHIFT) + 2);
		if (!ctxt)
			return -ENOMEM;

		constructor(info, write_len, ctxt);
		offset = seg->rs_offset + info->wi_seg_off;
		ret = svc_rdma_rw_ctx_init(rdma, ctxt, offset, seg->rs_handle,
					   DMA_TO_DEVICE);
		if (ret < 0)
			return -EIO;
		percpu_counter_inc(&svcrdma_stat_write);

		list_add(&ctxt->rw_list, &cc->cc_rwctxts);
		cc->cc_sqecount += ret;
		if (write_len == seg->rs_length - info->wi_seg_off) {
			info->wi_seg_no++;
			info->wi_seg_off = 0;
		} else {
			info->wi_seg_off += write_len;
		}
		remaining -= write_len;
	} while (remaining);

	return 0;

out_overflow:
	trace_svcrdma_small_wrch_err(&cc->cc_cid, remaining, info->wi_seg_no,
				     info->wi_chunk->ch_segcount);
	return -E2BIG;
}

/**
 * svc_rdma_iov_write - Construct RDMA Writes from an iov
 * @info: pointer to write arguments
 * @iov: kvec to write
 *
 * Returns:
 *   On success, returns zero
 *   %-E2BIG if the client-provided Write chunk is too small
 *   %-ENOMEM if a resource has been exhausted
 *   %-EIO if an rdma-rw error occurred
 */
static int svc_rdma_iov_write(struct svc_rdma_write_info *info,
			      const struct kvec *iov)
{
	info->wi_base = iov->iov_base;
	return svc_rdma_build_writes(info, svc_rdma_vec_to_sg,
				     iov->iov_len);
}

/**
 * svc_rdma_pages_write - Construct RDMA Writes from pages
 * @info: pointer to write arguments
 * @xdr: xdr_buf with pages to write
 * @offset: offset into the content of @xdr
 * @length: number of bytes to write
 *
 * Returns:
 *   On success, returns zero
 *   %-E2BIG if the client-provided Write chunk is too small
 *   %-ENOMEM if a resource has been exhausted
 *   %-EIO if an rdma-rw error occurred
 */
static int svc_rdma_pages_write(struct svc_rdma_write_info *info,
				const struct xdr_buf *xdr,
				unsigned int offset,
				unsigned long length)
{
	info->wi_xdr = xdr;
	info->wi_next_off = offset - xdr->head[0].iov_len;
	return svc_rdma_build_writes(info, svc_rdma_pagelist_to_sg,
				     length);
}

/**
 * svc_rdma_xb_write - Construct RDMA Writes to write an xdr_buf
 * @xdr: xdr_buf to write
 * @data: pointer to write arguments
 *
 * Returns:
 *   On success, returns zero
 *   %-E2BIG if the client-provided Write chunk is too small
 *   %-ENOMEM if a resource has been exhausted
 *   %-EIO if an rdma-rw error occurred
 */
static int svc_rdma_xb_write(const struct xdr_buf *xdr, void *data)
{
	struct svc_rdma_write_info *info = data;
	int ret;

	if (xdr->head[0].iov_len) {
		ret = svc_rdma_iov_write(info, &xdr->head[0]);
		if (ret < 0)
			return ret;
	}

	if (xdr->page_len) {
		ret = svc_rdma_pages_write(info, xdr, xdr->head[0].iov_len,
					   xdr->page_len);
		if (ret < 0)
			return ret;
	}

	if (xdr->tail[0].iov_len) {
		ret = svc_rdma_iov_write(info, &xdr->tail[0]);
		if (ret < 0)
			return ret;
	}

	return xdr->len;
}

static int svc_rdma_send_write_chunk(struct svcxprt_rdma *rdma,
				     const struct svc_rdma_chunk *chunk,
				     const struct xdr_buf *xdr)
{
	struct svc_rdma_write_info *info;
	struct svc_rdma_chunk_ctxt *cc;
	struct xdr_buf payload;
	int ret;

	if (xdr_buf_subsegment(xdr, &payload, chunk->ch_position,
			       chunk->ch_payload_length))
		return -EMSGSIZE;

	info = svc_rdma_write_info_alloc(rdma, chunk);
	if (!info)
		return -ENOMEM;
	cc = &info->wi_cc;

	ret = svc_rdma_xb_write(&payload, info);
	if (ret != payload.len)
		goto out_err;

	trace_svcrdma_post_write_chunk(&cc->cc_cid, cc->cc_sqecount);
	ret = svc_rdma_post_chunk_ctxt(rdma, cc);
	if (ret < 0)
		goto out_err;
	return 0;

out_err:
	svc_rdma_write_info_free(info);
	return ret;
}

/**
 * svc_rdma_send_write_list - Send all chunks on the Write list
 * @rdma: controlling RDMA transport
 * @rctxt: Write list provisioned by the client
 * @xdr: xdr_buf containing an RPC Reply message
 *
 * Returns zero on success, or a negative errno if one or more
 * Write chunks could not be sent.
 */
int svc_rdma_send_write_list(struct svcxprt_rdma *rdma,
			     const struct svc_rdma_recv_ctxt *rctxt,
			     const struct xdr_buf *xdr)
{
	struct svc_rdma_chunk *chunk;
	int ret;

	pcl_for_each_chunk(chunk, &rctxt->rc_write_pcl) {
		if (!chunk->ch_payload_length)
			break;
		ret = svc_rdma_send_write_chunk(rdma, chunk, xdr);
		if (ret < 0)
			return ret;
	}
	return 0;
}

/**
 * svc_rdma_prepare_reply_chunk - Construct WR chain for writing the Reply chunk
 * @rdma: controlling RDMA transport
 * @write_pcl: Write chunk list provided by client
 * @reply_pcl: Reply chunk provided by client
 * @sctxt: Send WR resources
 * @xdr: xdr_buf containing an RPC Reply
 *
 * Returns a non-negative number of bytes the chunk consumed, or
 *	%-E2BIG if the payload was larger than the Reply chunk,
 *	%-EINVAL if client provided too many segments,
 *	%-ENOMEM if rdma_rw context pool was exhausted,
 *	%-ENOTCONN if posting failed (connection is lost),
 *	%-EIO if rdma_rw initialization failed (DMA mapping, etc).
 */
int svc_rdma_prepare_reply_chunk(struct svcxprt_rdma *rdma,
				 const struct svc_rdma_pcl *write_pcl,
				 const struct svc_rdma_pcl *reply_pcl,
				 struct svc_rdma_send_ctxt *sctxt,
				 const struct xdr_buf *xdr)
{
	struct svc_rdma_write_info *info = &sctxt->sc_reply_info;
	struct svc_rdma_chunk_ctxt *cc = &info->wi_cc;
	struct ib_send_wr *first_wr;
	struct list_head *pos;
	struct ib_cqe *cqe;
	int ret;

	info->wi_rdma = rdma;
	info->wi_chunk = pcl_first_chunk(reply_pcl);
	info->wi_seg_off = 0;
	info->wi_seg_no = 0;
	info->wi_cc.cc_cqe.done = svc_rdma_reply_done;

	ret = pcl_process_nonpayloads(write_pcl, xdr,
				      svc_rdma_xb_write, info);
	if (ret < 0)
		return ret;

	first_wr = sctxt->sc_wr_chain;
	cqe = &cc->cc_cqe;
	list_for_each(pos, &cc->cc_rwctxts) {
		struct svc_rdma_rw_ctxt *rwc;

		rwc = list_entry(pos, struct svc_rdma_rw_ctxt, rw_list);
		first_wr = rdma_rw_ctx_wrs(&rwc->rw_ctx, rdma->sc_qp,
					   rdma->sc_port_num, cqe, first_wr);
		cqe = NULL;
	}
	sctxt->sc_wr_chain = first_wr;
	sctxt->sc_sqecount += cc->cc_sqecount;

	trace_svcrdma_post_reply_chunk(&cc->cc_cid, cc->cc_sqecount);
	return xdr->len;
}

/**
 * svc_rdma_build_read_segment - Build RDMA Read WQEs to pull one RDMA segment
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 * @segment: co-ordinates of remote memory to be read
 *
 * Returns:
 *   %0: the Read WR chain was constructed successfully
 *   %-EINVAL: there were not enough rq_pages to finish
 *   %-ENOMEM: allocating a local resources failed
 *   %-EIO: a DMA mapping error occurred
 */
static int svc_rdma_build_read_segment(struct svc_rqst *rqstp,
				       struct svc_rdma_recv_ctxt *head,
				       const struct svc_rdma_segment *segment)
{
	struct svcxprt_rdma *rdma = svc_rdma_rqst_rdma(rqstp);
	struct svc_rdma_chunk_ctxt *cc = &head->rc_cc;
	unsigned int sge_no, seg_len, len;
	struct svc_rdma_rw_ctxt *ctxt;
	struct scatterlist *sg;
	int ret;

	len = segment->rs_length;
	sge_no = PAGE_ALIGN(head->rc_pageoff + len) >> PAGE_SHIFT;
	ctxt = svc_rdma_get_rw_ctxt(rdma, sge_no);
	if (!ctxt)
		return -ENOMEM;
	ctxt->rw_nents = sge_no;

	sg = ctxt->rw_sg_table.sgl;
	for (sge_no = 0; sge_no < ctxt->rw_nents; sge_no++) {
		seg_len = min_t(unsigned int, len,
				PAGE_SIZE - head->rc_pageoff);

		if (!head->rc_pageoff)
			head->rc_page_count++;

		sg_set_page(sg, rqstp->rq_pages[head->rc_curpage],
			    seg_len, head->rc_pageoff);
		sg = sg_next(sg);

		head->rc_pageoff += seg_len;
		if (head->rc_pageoff == PAGE_SIZE) {
			head->rc_curpage++;
			head->rc_pageoff = 0;
		}
		len -= seg_len;

		if (len && ((head->rc_curpage + 1) > ARRAY_SIZE(rqstp->rq_pages)))
			goto out_overrun;
	}

	ret = svc_rdma_rw_ctx_init(rdma, ctxt, segment->rs_offset,
				   segment->rs_handle, DMA_FROM_DEVICE);
	if (ret < 0)
		return -EIO;
	percpu_counter_inc(&svcrdma_stat_read);

	list_add(&ctxt->rw_list, &cc->cc_rwctxts);
	cc->cc_sqecount += ret;
	return 0;

out_overrun:
	trace_svcrdma_page_overrun_err(&cc->cc_cid, head->rc_curpage);
	return -EINVAL;
}

/**
 * svc_rdma_build_read_chunk - Build RDMA Read WQEs to pull one RDMA chunk
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 * @chunk: Read chunk to pull
 *
 * Return values:
 *   %0: the Read WR chain was constructed successfully
 *   %-EINVAL: there were not enough resources to finish
 *   %-ENOMEM: allocating a local resources failed
 *   %-EIO: a DMA mapping error occurred
 */
static int svc_rdma_build_read_chunk(struct svc_rqst *rqstp,
				     struct svc_rdma_recv_ctxt *head,
				     const struct svc_rdma_chunk *chunk)
{
	const struct svc_rdma_segment *segment;
	int ret;

	ret = -EINVAL;
	pcl_for_each_segment(segment, chunk) {
		ret = svc_rdma_build_read_segment(rqstp, head, segment);
		if (ret < 0)
			break;
		head->rc_readbytes += segment->rs_length;
	}
	return ret;
}

/**
 * svc_rdma_copy_inline_range - Copy part of the inline content into pages
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 * @offset: offset into the Receive buffer of region to copy
 * @remaining: length of region to copy
 *
 * Take a page at a time from rqstp->rq_pages and copy the inline
 * content from the Receive buffer into that page. Update
 * head->rc_curpage and head->rc_pageoff so that the next RDMA Read
 * result will land contiguously with the copied content.
 *
 * Return values:
 *   %0: Inline content was successfully copied
 *   %-EINVAL: offset or length was incorrect
 */
static int svc_rdma_copy_inline_range(struct svc_rqst *rqstp,
				      struct svc_rdma_recv_ctxt *head,
				      unsigned int offset,
				      unsigned int remaining)
{
	unsigned char *dst, *src = head->rc_recv_buf;
	unsigned int page_no, numpages;

	numpages = PAGE_ALIGN(head->rc_pageoff + remaining) >> PAGE_SHIFT;
	for (page_no = 0; page_no < numpages; page_no++) {
		unsigned int page_len;

		page_len = min_t(unsigned int, remaining,
				 PAGE_SIZE - head->rc_pageoff);

		if (!head->rc_pageoff)
			head->rc_page_count++;

		dst = page_address(rqstp->rq_pages[head->rc_curpage]);
		memcpy(dst + head->rc_curpage, src + offset, page_len);

		head->rc_readbytes += page_len;
		head->rc_pageoff += page_len;
		if (head->rc_pageoff == PAGE_SIZE) {
			head->rc_curpage++;
			head->rc_pageoff = 0;
		}
		remaining -= page_len;
		offset += page_len;
	}

	return -EINVAL;
}

/**
 * svc_rdma_read_multiple_chunks - Construct RDMA Reads to pull data item Read chunks
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 *
 * The chunk data lands in rqstp->rq_arg as a series of contiguous pages,
 * like an incoming TCP call.
 *
 * Return values:
 *   %0: RDMA Read WQEs were successfully built
 *   %-EINVAL: client provided too many chunks or segments,
 *   %-ENOMEM: rdma_rw context pool was exhausted,
 *   %-ENOTCONN: posting failed (connection is lost),
 *   %-EIO: rdma_rw initialization failed (DMA mapping, etc).
 */
static noinline int
svc_rdma_read_multiple_chunks(struct svc_rqst *rqstp,
			      struct svc_rdma_recv_ctxt *head)
{
	const struct svc_rdma_pcl *pcl = &head->rc_read_pcl;
	struct svc_rdma_chunk *chunk, *next;
	unsigned int start, length;
	int ret;

	start = 0;
	chunk = pcl_first_chunk(pcl);
	length = chunk->ch_position;
	ret = svc_rdma_copy_inline_range(rqstp, head, start, length);
	if (ret < 0)
		return ret;

	pcl_for_each_chunk(chunk, pcl) {
		ret = svc_rdma_build_read_chunk(rqstp, head, chunk);
		if (ret < 0)
			return ret;

		next = pcl_next_chunk(pcl, chunk);
		if (!next)
			break;

		start += length;
		length = next->ch_position - head->rc_readbytes;
		ret = svc_rdma_copy_inline_range(rqstp, head, start, length);
		if (ret < 0)
			return ret;
	}

	start += length;
	length = head->rc_byte_len - start;
	return svc_rdma_copy_inline_range(rqstp, head, start, length);
}

/**
 * svc_rdma_read_data_item - Construct RDMA Reads to pull data item Read chunks
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 *
 * The chunk data lands in the page list of rqstp->rq_arg.pages.
 *
 * Currently NFSD does not look at the rqstp->rq_arg.tail[0] kvec.
 * Therefore, XDR round-up of the Read chunk and trailing
 * inline content must both be added at the end of the pagelist.
 *
 * Return values:
 *   %0: RDMA Read WQEs were successfully built
 *   %-EINVAL: client provided too many chunks or segments,
 *   %-ENOMEM: rdma_rw context pool was exhausted,
 *   %-ENOTCONN: posting failed (connection is lost),
 *   %-EIO: rdma_rw initialization failed (DMA mapping, etc).
 */
static int svc_rdma_read_data_item(struct svc_rqst *rqstp,
				   struct svc_rdma_recv_ctxt *head)
{
	return svc_rdma_build_read_chunk(rqstp, head,
					 pcl_first_chunk(&head->rc_read_pcl));
}

/**
 * svc_rdma_read_chunk_range - Build RDMA Read WRs for portion of a chunk
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 * @chunk: parsed Call chunk to pull
 * @offset: offset of region to pull
 * @length: length of region to pull
 *
 * Return values:
 *   %0: RDMA Read WQEs were successfully built
 *   %-EINVAL: there were not enough resources to finish
 *   %-ENOMEM: rdma_rw context pool was exhausted,
 *   %-ENOTCONN: posting failed (connection is lost),
 *   %-EIO: rdma_rw initialization failed (DMA mapping, etc).
 */
static int svc_rdma_read_chunk_range(struct svc_rqst *rqstp,
				     struct svc_rdma_recv_ctxt *head,
				     const struct svc_rdma_chunk *chunk,
				     unsigned int offset, unsigned int length)
{
	const struct svc_rdma_segment *segment;
	int ret;

	ret = -EINVAL;
	pcl_for_each_segment(segment, chunk) {
		struct svc_rdma_segment dummy;

		if (offset > segment->rs_length) {
			offset -= segment->rs_length;
			continue;
		}

		dummy.rs_handle = segment->rs_handle;
		dummy.rs_length = min_t(u32, length, segment->rs_length) - offset;
		dummy.rs_offset = segment->rs_offset + offset;

		ret = svc_rdma_build_read_segment(rqstp, head, &dummy);
		if (ret < 0)
			break;

		head->rc_readbytes += dummy.rs_length;
		length -= dummy.rs_length;
		offset = 0;
	}
	return ret;
}

/**
 * svc_rdma_read_call_chunk - Build RDMA Read WQEs to pull a Long Message
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 *
 * Return values:
 *   %0: RDMA Read WQEs were successfully built
 *   %-EINVAL: there were not enough resources to finish
 *   %-ENOMEM: rdma_rw context pool was exhausted,
 *   %-ENOTCONN: posting failed (connection is lost),
 *   %-EIO: rdma_rw initialization failed (DMA mapping, etc).
 */
static int svc_rdma_read_call_chunk(struct svc_rqst *rqstp,
				    struct svc_rdma_recv_ctxt *head)
{
	const struct svc_rdma_chunk *call_chunk =
			pcl_first_chunk(&head->rc_call_pcl);
	const struct svc_rdma_pcl *pcl = &head->rc_read_pcl;
	struct svc_rdma_chunk *chunk, *next;
	unsigned int start, length;
	int ret;

	if (pcl_is_empty(pcl))
		return svc_rdma_build_read_chunk(rqstp, head, call_chunk);

	start = 0;
	chunk = pcl_first_chunk(pcl);
	length = chunk->ch_position;
	ret = svc_rdma_read_chunk_range(rqstp, head, call_chunk,
					start, length);
	if (ret < 0)
		return ret;

	pcl_for_each_chunk(chunk, pcl) {
		ret = svc_rdma_build_read_chunk(rqstp, head, chunk);
		if (ret < 0)
			return ret;

		next = pcl_next_chunk(pcl, chunk);
		if (!next)
			break;

		start += length;
		length = next->ch_position - head->rc_readbytes;
		ret = svc_rdma_read_chunk_range(rqstp, head, call_chunk,
						start, length);
		if (ret < 0)
			return ret;
	}

	start += length;
	length = call_chunk->ch_length - start;
	return svc_rdma_read_chunk_range(rqstp, head, call_chunk,
					 start, length);
}

/**
 * svc_rdma_read_special - Build RDMA Read WQEs to pull a Long Message
 * @rqstp: RPC transaction context
 * @head: context for ongoing I/O
 *
 * The start of the data lands in the first page just after the
 * Transport header, and the rest lands in rqstp->rq_arg.pages.
 *
 * Assumptions:
 *	- A PZRC is never sent in an RDMA_MSG message, though it's
 *	  allowed by spec.
 *
 * Return values:
 *   %0: RDMA Read WQEs were successfully built
 *   %-EINVAL: client provided too many chunks or segments,
 *   %-ENOMEM: rdma_rw context pool was exhausted,
 *   %-ENOTCONN: posting failed (connection is lost),
 *   %-EIO: rdma_rw initialization failed (DMA mapping, etc).
 */
static noinline int svc_rdma_read_special(struct svc_rqst *rqstp,
					  struct svc_rdma_recv_ctxt *head)
{
	return svc_rdma_read_call_chunk(rqstp, head);
}

/* Pages under I/O have been copied to head->rc_pages. Ensure that
 * svc_xprt_release() does not put them when svc_rdma_recvfrom()
 * returns. This has to be done after all Read WRs are constructed
 * to properly handle a page that happens to be part of I/O on behalf
 * of two different RDMA segments.
 *
 * Note: if the subsequent post_send fails, these pages have already
 * been moved to head->rc_pages and thus will be cleaned up by
 * svc_rdma_recv_ctxt_put().
 */
static void svc_rdma_clear_rqst_pages(struct svc_rqst *rqstp,
				      struct svc_rdma_recv_ctxt *head)
{
	unsigned int i;

	for (i = 0; i < head->rc_page_count; i++) {
		head->rc_pages[i] = rqstp->rq_pages[i];
		rqstp->rq_pages[i] = NULL;
	}
}

/**
 * svc_rdma_process_read_list - Pull list of Read chunks from the client
 * @rdma: controlling RDMA transport
 * @rqstp: set of pages to use as Read sink buffers
 * @head: pages under I/O collect here
 *
 * The RPC/RDMA protocol assumes that the upper layer's XDR decoders
 * pull each Read chunk as they decode an incoming RPC message.
 *
 * On Linux, however, the server needs to have a fully-constructed RPC
 * message in rqstp->rq_arg when there is a positive return code from
 * ->xpo_recvfrom. So the Read list is safety-checked immediately when
 * it is received, then here the whole Read list is pulled all at once.
 * The ingress RPC message is fully reconstructed once all associated
 * RDMA Reads have completed.
 *
 * Return values:
 *   %1: all needed RDMA Reads were posted successfully,
 *   %-EINVAL: client provided too many chunks or segments,
 *   %-ENOMEM: rdma_rw context pool was exhausted,
 *   %-ENOTCONN: posting failed (connection is lost),
 *   %-EIO: rdma_rw initialization failed (DMA mapping, etc).
 */
int svc_rdma_process_read_list(struct svcxprt_rdma *rdma,
			       struct svc_rqst *rqstp,
			       struct svc_rdma_recv_ctxt *head)
{
	struct svc_rdma_chunk_ctxt *cc = &head->rc_cc;
	int ret;

	cc->cc_cqe.done = svc_rdma_wc_read_done;
	cc->cc_sqecount = 0;
	head->rc_pageoff = 0;
	head->rc_curpage = 0;
	head->rc_readbytes = 0;

	if (pcl_is_empty(&head->rc_call_pcl)) {
		if (head->rc_read_pcl.cl_count == 1)
			ret = svc_rdma_read_data_item(rqstp, head);
		else
			ret = svc_rdma_read_multiple_chunks(rqstp, head);
	} else
		ret = svc_rdma_read_special(rqstp, head);
	svc_rdma_clear_rqst_pages(rqstp, head);
	if (ret < 0)
		return ret;

	trace_svcrdma_post_read_chunk(&cc->cc_cid, cc->cc_sqecount);
	ret = svc_rdma_post_chunk_ctxt(rdma, cc);
	return ret < 0 ? ret : 1;
}
