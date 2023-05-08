/*
 *    VSI V4L2 decoder entry.
 *
 *    Copyright (c) 2019, VeriSilicon Inc.
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License, version 2, as
 *    published by the Free Software Foundation.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License version 2 for more details.
 *
 *    You may obtain a copy of the GNU General Public License
 *    Version 2 at the following locations:
 *    https://opensource.org/licenses/gpl-2.0.php
 */

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/platform_device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>
#include <linux/delay.h>
#include <linux/version.h>
#include "vsi-v4l2-priv.h"

static int vsi_dec_querycap(
	struct file *file,
	void *priv,
	struct v4l2_capability *cap)
{
	struct vsi_v4l2_dev_info *hwinfo;

	v4l2_klog(LOGLVL_FLOW, "%s", __func__);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	hwinfo = vsiv4l2_get_hwinfo();
	if (hwinfo->decformat == 0)
		return -ENODEV;
	strlcpy(cap->driver, "vsi_v4l2", sizeof("vsi_v4l2"));
	strlcpy(cap->card, "vsi_v4l2dec", sizeof("vsi_v4l2dec"));
	strlcpy(cap->bus_info, "platform:vsi_v4l2dec", sizeof("platform:vsi_v4l2dec"));

	cap->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vsi_dec_reqbufs(
	struct file *filp,
	void *priv,
	struct v4l2_requestbuffers *p)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);
	int ret;
	struct vb2_queue *q;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(p->type, ctx->flag))
		return -EINVAL;

	if (binputqueue(p->type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;
	ret = vb2_reqbufs(q, p);
	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d ask for %d buffer, got %d:%d",
		ctx->ctxid, __func__, p->type, p->count, q->num_buffers, ret);
	if (ret == 0) {
		print_queinfo(q);
		if (p->count == 0 && binputqueue(p->type)) {
			p->capabilities = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_USERPTR | V4L2_BUF_CAP_SUPPORTS_DMABUF;
			//ctx->status = VSI_STATUS_INIT;
		}
	}
	return ret;
}

static int vsi_dec_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);

	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d", ctx->ctxid, __func__, f->type);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(f->type, ctx->flag))
		return -EINVAL;

	return vsiv4l2_getfmt(ctx, f);
}

static int vsi_dec_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	int ret;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);

	v4l2_klog(LOGLVL_CONFIG, "%s fmt:%x, res:%dx%d\n", __func__,
		  f->fmt.pix.pixelformat, f->fmt.pix.width,
		  f->fmt.pix.height);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(f->type, ctx->flag))
		return -EINVAL;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	ret = vsiv4l2_setfmt(ctx, f);

	if (V4L2_TYPE_IS_OUTPUT(f->type) && !test_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag)) {
		struct v4l2_format fc;

		memset(&fc, 0, sizeof(fc));
		fc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		fc.fmt.pix.pixelformat = ctx->mediacfg.outfmt_fourcc;
		fc.fmt.pix.width = f->fmt.pix.width;
		fc.fmt.pix.height = f->fmt.pix.height;
		ret = vsiv4l2_setfmt(ctx, &fc);
	}

	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_dec_querybuf(
	struct file *filp,
	void *priv,
	struct v4l2_buffer *buf)
{
	int ret;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);
	struct vb2_queue *q;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(buf->type, ctx->flag))
		return -EINVAL;
	if (binputqueue(buf->type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;
	v4l2_klog(LOGLVL_FLOW, "%s::%lx:%d:%d", __func__, ctx->flag, buf->type, buf->index);
	ret = vb2_querybuf(q, buf);
	if (buf->memory == V4L2_MEMORY_MMAP) {
		if (ret == 0 && q == &ctx->output_que)
			buf->m.offset += OUTF_BASE;
	}
	return ret;
}

static int vsi_dec_qbuf(struct file *filp, void *priv, struct v4l2_buffer *buf)
{
	int ret;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);
	struct video_device *vdev = ctx->dev->vdec;

	v4l2_klog(LOGLVL_FLOW, "%llx:%s:%d:%d:%d", ctx->ctxid, __func__, buf->type, buf->index, buf->bytesused);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(buf->type, ctx->flag))
		return -EINVAL;
	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	if (binputqueue(buf->type))
		set_bit(CTX_FLAG_SRCBUF_BIT, &ctx->flag);
	if (binputqueue(buf->type) && buf->bytesused == 0 &&
		ctx->status == DEC_STATUS_DECODING) {
		if (test_and_clear_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag)) {
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_CMD_STOP, NULL);
			ctx->status = DEC_STATUS_DRAINING;
		} else
			ret = 0;
		mutex_unlock(&ctx->ctxlock);
		return ret;
	}
	if (!binputqueue(buf->type)) {
		ctx->outbuflen[buf->index] = buf->length;
		ret = vb2_qbuf(&ctx->output_que, vdev->v4l2_dev->mdev, buf);
	} else {
		ctx->inbuflen[buf->index] = buf->length;
		ctx->inbufbytes[buf->index] = buf->bytesused;
		if (buf->timestamp.tv_sec < 0 || buf->timestamp.tv_usec < 0)
			set_bit(BUF_FLAG_TIMESTAMP_INVALID, &ctx->srcvbufflag[buf->index]);
		ret = vb2_qbuf(&ctx->input_que, vdev->v4l2_dev->mdev, buf);
	}
	if (ret == 0)
		ret = vsi_dec_output_on(ctx);
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_dec_dec2drain(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	if (ctx->status == DEC_STATUS_DECODING &&
		test_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag)) {
		ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_CMD_STOP, NULL);
		ctx->status = DEC_STATUS_DRAINING;
	}
	return ret;
}

int vsi_dec_output_on(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	if (!ctx->need_output_on)
		return 0;
	if (ctx->input_que.queued_count < ctx->input_que.min_buffers_needed)
		return 0;

	v4l2_klog(LOGLVL_FLOW, "%llx:%s start streaming", ctx->ctxid, __func__);
	ctx->status = DEC_STATUS_DECODING;
	ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMON_OUTPUT, NULL);
	if (ret == 0)
		vsi_dec_dec2drain(ctx);

	ctx->need_output_on = false;
	return ret;
}

int vsi_dec_capture_on(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	if (!ctx->need_capture_on || !ctx->reschange_cnt)
		return 0;

	if (ctx->reschange_notified && !vb2_is_streaming(&ctx->input_que)) {
		v4l2_klog(LOGLVL_BRIEF, "handle seek first, then source change\n");
		return 0;
	}

	ret = vb2_streamon(&ctx->output_que, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (ret)
		return ret;

	if (ctx->status != DEC_STATUS_SEEK && ctx->status != DEC_STATUS_ENDSTREAM)
		ctx->status = DEC_STATUS_DECODING;
	ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMON_CAPTURE, NULL);
	if (ret == 0)
		vsi_dec_dec2drain(ctx);
	if (test_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag)) {
		struct vb2_buffer *vb = ctx->output_que.bufs[0];

		vb->planes[0].bytesused = 0;
		ctx->lastcapbuffer_idx = 0;
		vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
	}
	ctx->need_capture_on = false;
	ctx->reschange_notified = false;

	return ret;
}

int vsi_dec_capture_off(struct vsi_v4l2_ctx *ctx)
{
	int ret;
	struct vb2_queue *q = &ctx->output_que;

	if (!vb2_is_streaming(q))
		return 0;

	ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMOFF_CAPTURE, NULL);
	if (ret < 0)
		return -EFAULT;

	mutex_unlock(&ctx->ctxlock);
	if (wait_event_interruptible(ctx->capoffdone_queue, vsi_checkctx_capoffdone(ctx) != 0))
		v4l2_klog(LOGLVL_WARNING, "%llx wait capture streamoff done timeout\n", ctx->ctxid);
	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	ctx->buffed_capnum = 0;
	ctx->buffed_cropcapnum = 0;
	return_all_buffers(q, VB2_BUF_STATE_ERROR, 1);
	return vb2_streamoff(q, q->type);
}

static int vsi_dec_streamon(struct file *filp, void *priv, enum v4l2_buf_type type)
{
	int ret = 0;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(type, ctx->flag))
		return -EINVAL;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	v4l2_klog(LOGLVL_BRIEF, "%llx %s:%d in status %d", ctx->ctxid, __func__, type, ctx->status);
	if (!binputqueue(type)) {
		vb2_clear_last_buffer_dequeued(&ctx->output_que);
		ctx->need_capture_on = true;
		ret = vsi_dec_capture_on(ctx);
		printbufinfo(&ctx->output_que);
	} else {
		ret = vb2_streamon(&ctx->input_que, type);
		if (ret == 0) {
			ctx->need_output_on = true;
			ret = vsi_dec_output_on(ctx);
		}
		printbufinfo(&ctx->input_que);
	}
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_dec_checkctx_srcbuf(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	if (ctx->queued_srcnum == 0 || ctx->error < 0)
		ret = 1;
	return ret;
}

void vsi_dec_update_reso(struct vsi_v4l2_ctx *ctx)
{
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;

	ctx->reschanged_need_notify = true;
	pcfg->decparams.dec_info.dec_info = pcfg->decparams_bkup.dec_info.dec_info;
	pcfg->decparams.dec_info.io_buffer.srcwidth = pcfg->decparams_bkup.io_buffer.srcwidth;
	pcfg->decparams.dec_info.io_buffer.srcheight = pcfg->decparams_bkup.io_buffer.srcheight;
	pcfg->decparams.dec_info.io_buffer.output_width = pcfg->decparams_bkup.io_buffer.output_width;
	pcfg->decparams.dec_info.io_buffer.output_height = pcfg->decparams_bkup.io_buffer.output_height;
	pcfg->decparams.dec_info.io_buffer.output_wstride = pcfg->decparams_bkup.io_buffer.output_wstride;
	pcfg->bytesperline = pcfg->decparams_bkup.io_buffer.output_wstride;
	pcfg->orig_dpbsize = pcfg->sizeimagedst_bkup;
	pcfg->src_pixeldepth = pcfg->decparams_bkup.dec_info.dec_info.bit_depth;
	pcfg->minbuf_4output = pcfg->minbuf_4capture = pcfg->minbuf_4output_bkup;
	pcfg->sizeimagedst[0] = pcfg->sizeimagedst_bkup;
	pcfg->sizeimagedst[1] = 0;
	pcfg->sizeimagedst[2] = 0;
	pcfg->sizeimagedst[3] = 0;
}

static void vsi_dec_return_queued_buffers(struct vb2_queue *q)
{
	struct vb2_buffer *vb;

	list_for_each_entry(vb, &q->queued_list, queued_entry) {
		if (vb->state == VB2_BUF_STATE_QUEUED)
			vb->state = VB2_BUF_STATE_DEQUEUED;
	}
	INIT_LIST_HEAD(&q->queued_list);
	q->queued_count = 0;
	q->waiting_for_buffers = !q->is_output;
}

static int vsi_dec_streamoff(
	struct file *file,
	void *priv,
	enum v4l2_buf_type type)
{
	int ret = 0;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *q;
	enum v4l2_buf_type otype;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (ctx->error < 0)
		return -EFAULT;
	if (!isvalidtype(type, ctx->flag))
		return -EINVAL;

	otype = (type == V4L2_BUF_TYPE_VIDEO_CAPTURE ? V4L2_BUF_TYPE_VIDEO_OUTPUT :
		V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (binputqueue(type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	if (!binputqueue(type)) {
		vb2_clear_last_buffer_dequeued(q);
		ctx->need_capture_on = false;
		if (!vb2_is_streaming(q)) {
			vsi_dec_return_queued_buffers(q);
			mutex_unlock(&ctx->ctxlock);
			return 0;
		}
	}
	v4l2_klog(LOGLVL_BRIEF, "%llx %s:%d:%d:%d\n", ctx->ctxid, __func__, type, ctx->status, ctx->queued_srcnum);
	if (!vb2_is_streaming(q) && ctx->status == VSI_STATUS_INIT) {
		mutex_unlock(&ctx->ctxlock);
		return 0;
	}

	if (binputqueue(type)) {
		if (!ctx->need_output_on)
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMOFF_OUTPUT, NULL);
		ctx->status = DEC_STATUS_SEEK;
		ctx->need_output_on = false;
	} else {
		ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMOFF_CAPTURE, NULL);
		if (ctx->status != DEC_STATUS_SEEK && ctx->status != DEC_STATUS_ENDSTREAM)
			ctx->status = DEC_STATUS_STOPPED;
	}
	if (ret < 0) {
		mutex_unlock(&ctx->ctxlock);
		return -EFAULT;
	}
	if (binputqueue(type)) {
		if (!(test_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag))) {
			//here we need to wait all OUTPUT buffer returned
			mutex_unlock(&ctx->ctxlock);
			if (wait_event_interruptible(ctx->retbuf_queue,
				vsi_dec_checkctx_srcbuf(ctx) != 0))
				v4l2_klog(LOGLVL_WARNING, "%llx wait output buffer return timeout\n", ctx->ctxid);
			if (mutex_lock_interruptible(&ctx->ctxlock))
				return -EBUSY;
		}
		clear_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag);
		clear_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag);
		clear_bit(CTX_FLAG_SRCBUF_BIT, &ctx->flag);
		if (test_and_clear_bit(CTX_FLAG_DELAY_SRCCHANGED_BIT, &ctx->flag)) {
			vsi_dec_update_reso(ctx);
			vsi_v4l2_send_reschange(ctx);
			v4l2_klog(LOGLVL_BRIEF, "%llx send delayed src change in streamoff", ctx->ctxid);
		}
	} else {
		mutex_unlock(&ctx->ctxlock);
		if (wait_event_interruptible(ctx->capoffdone_queue,
			vsi_checkctx_capoffdone(ctx) != 0))
			v4l2_klog(LOGLVL_WARNING, "%llx wait capture streamoff done timeout\n", ctx->ctxid);
		if (mutex_lock_interruptible(&ctx->ctxlock))
			return -EBUSY;
		ctx->buffed_capnum = 0;
		ctx->buffed_cropcapnum = 0;
	}
	return_all_buffers(q, VB2_BUF_STATE_DONE, 1);
	ret = vb2_streamoff(q, type);
	mutex_unlock(&ctx->ctxlock);

	return ret;
}

static int vsi_dec_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	int ret = 0;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *q;
	struct vb2_buffer *vb;
	struct vsi_vpu_buf *vsibuf;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(p->type, ctx->flag))
		return -EINVAL;
	if (binputqueue(p->type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;

	if (!vb2_is_streaming(q))
		return -EPIPE;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	ret = vb2_dqbuf(q, p, file->f_flags & O_NONBLOCK);
	if (ret == 0) {
		vb = q->bufs[p->index];
		vsibuf = vb_to_vsibuf(vb);
		list_del(&vsibuf->list);
		if (!binputqueue(p->type)) {
			clear_bit(BUF_FLAG_DONE, &ctx->vbufflag[p->index]);
			ctx->buffed_capnum--;
			ctx->buffed_cropcapnum--;
		} else
			clear_bit(BUF_FLAG_DONE, &ctx->srcvbufflag[p->index]);
		if (ctx->status != DEC_STATUS_ENDSTREAM &&
			!(test_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag)) &&
			p->bytesused == 0) {
			mutex_unlock(&ctx->ctxlock);
			return -EAGAIN;
		}
	}
	if (!binputqueue(p->type)) {
		p->reserved = ctx->rfc_luma_offset[p->index];
		p->reserved2 = ctx->rfc_chroma_offset[p->index];
		v4l2_klog(LOGLVL_FLOW, "rfc offest update=%x:%x", p->reserved, p->reserved2);

		if (p->bytesused == 0 && (ctx->status == DEC_STATUS_ENDSTREAM || test_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag))) {
			ctx->status = DEC_STATUS_ENDSTREAM;
			clear_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag);
			v4l2_klog(LOGLVL_BRIEF, "send eos flag");
		} else if (test_bit(CTX_FLAG_DELAY_SRCCHANGED_BIT, &ctx->flag) && ctx->buffed_capnum == 0) {
			vsi_dec_update_reso(ctx);
			vsi_v4l2_send_reschange(ctx);
			clear_bit(CTX_FLAG_DELAY_SRCCHANGED_BIT, &ctx->flag);
			v4l2_klog(LOGLVL_BRIEF, "%llx send delayed src change", ctx->ctxid);
		} else if (test_and_clear_bit(BUF_FLAG_CROPCHANGE, &ctx->vbufflag[p->index])) {
			struct v4l2_event event;

			if (update_and_removecropinfo(ctx)) {
				memset((void *)&event, 0, sizeof(struct v4l2_event));
				event.type = V4L2_EVENT_CROPCHANGE;
				v4l2_event_queue_fh(&ctx->fh, &event);
			}
			v4l2_klog(LOGLVL_BRIEF, "send delayed crop change at buf %d", p->index);
		}
		p->field = V4L2_FIELD_NONE;
	}
	v4l2_klog(LOGLVL_FLOW, "%llx:%s:%d:%d:%x:%d", ctx->ctxid, __func__, p->type, p->index, p->flags, p->bytesused);
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_dec_prepare_buf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *p)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *q;
	struct video_device *vdev = ctx->dev->vdec;

	v4l2_klog(LOGLVL_FLOW, "%s", __func__);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(p->type, ctx->flag))
		return -EINVAL;

	if (binputqueue(p->type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;
	return vb2_prepare_buf(q, vdev->v4l2_dev->mdev, p);
}

static int vsi_dec_expbuf(
	struct file *file,
	void *priv,
	struct v4l2_exportbuffer *p)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *q;

	v4l2_klog(LOGLVL_FLOW, "%s", __func__);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(p->type, ctx->flag))
		return -EINVAL;

	if (binputqueue(p->type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;
	return vb2_expbuf(q, p);
}

static int vsi_dec_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;

	vsiv4l2_verifyfmt(ctx, f, 1);
	vsi_dec_getvui(ctx, f);
	return 0;
}

static int vsi_dec_enum_fmt(struct file *file, void *prv, struct v4l2_fmtdesc *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vsi_video_fmt *pfmt;
	int braw = brawfmt(ctx->flag, f->type);

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(f->type, ctx->flag))
		return -EINVAL;

	pfmt = vsi_enum_dec_format(f->index, braw, ctx);
	if (pfmt == NULL)
		return -EINVAL;

	if (pfmt->name && strlen(pfmt->name))
		strlcpy(f->description, pfmt->name, strlen(pfmt->name) + 1);
	f->pixelformat = pfmt->fourcc;
	f->flags = pfmt->flag;
	v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d:%x", __func__, f->index, f->type, pfmt->fourcc);
	return 0;
}

static int vsi_dec_get_selection(struct file *file, void *prv, struct v4l2_selection *s)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE:
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_PADDED:
		s->r.left = pcfg->decparams.dec_info.dec_info.visible_rect.left;
		s->r.top = pcfg->decparams.dec_info.dec_info.visible_rect.top;
		s->r.width = pcfg->decparams.dec_info.dec_info.visible_rect.width;
		s->r.height = pcfg->decparams.dec_info.dec_info.visible_rect.height;
		break;
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = pcfg->decparams.dec_info.io_buffer.output_width;
		s->r.height = pcfg->decparams.dec_info.io_buffer.output_width;
		break;
	default:
		return -EINVAL;
	}
	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d,%d,%d,%d",
		ctx->ctxid, __func__, s->r.left, s->r.top, s->r.width, s->r.height);

	return 0;
}

static int vsi_dec_subscribe_event(
	struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	case V4L2_EVENT_SKIP:
		return v4l2_event_subscribe(fh, sub, 16, NULL);
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	case V4L2_EVENT_EOS:
	case V4L2_EVENT_CODEC_ERROR:
	case V4L2_EVENT_CROPCHANGE:
	case V4L2_EVENT_INVALID_OPTION:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}
}

static int vsi_dec_handlestop_unspec(struct vsi_v4l2_ctx *ctx)
{
	//some unexpected condition fro CTS, not quite conformant to spec
	if ((ctx->status == VSI_STATUS_INIT && !test_bit(CTX_FLAG_DAEMONLIVE_BIT, &ctx->flag)) ||
		ctx->status == DEC_STATUS_STOPPED) {
		clear_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag);
		vsi_v4l2_sendeos(ctx);
		return 0;
	}
	if (ctx->status == DEC_STATUS_SEEK && !test_bit(CTX_FLAG_SRCBUF_BIT, &ctx->flag)) {
		vsi_v4l2_sendeos(ctx);
		return 0;
	}
	return 0;
}

static int vsi_dec_start_cmd(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	if (ctx->status == DEC_STATUS_STOPPED) {
		ctx->status = DEC_STATUS_DECODING;
		ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_CMD_START, NULL);
		if (ret < 0)
			return ret;
	}
	if (ctx->reschange_notified) {
		if (vb2_is_streaming(&ctx->output_que)) {
			ret = vsi_dec_capture_off(ctx);
			if (ret < 0) {
				v4l2_klog(LOGLVL_ERROR,
					  "ctx[%lld] capture off in start cmd fail\n",
					  ctx->ctxid & 0xffff);
				return ret;
			}
			ctx->need_capture_on = true;
		}
		if (ctx->need_capture_on)
			ret = vsi_dec_capture_on(ctx);
	}

	return ret;
}

static int vsi_dec_try_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
		cmd->stop.pts = 0;
		break;
	case V4L2_ENC_CMD_START:
		cmd->start.speed = 0;
		cmd->start.format = V4L2_DEC_START_FMT_NONE;
		break;
	case V4L2_DEC_CMD_RESET:
		break;
	case V4L2_ENC_CMD_PAUSE:
	case V4L2_ENC_CMD_RESUME:
	default:
		return -EINVAL;
	}

	cmd->flags = 0;

	return 0;
}

int vsi_dec_decoder_cmd(struct file *file, void *fh, struct v4l2_decoder_cmd *cmd)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	int ret = 0;

	v4l2_klog(LOGLVL_BRIEF, "%llx:%s:%d in state %d:%d", ctx->ctxid, __func__,
		cmd->cmd, ctx->status, vb2_is_streaming(&ctx->output_que));
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	switch (cmd->cmd) {
	case V4L2_DEC_CMD_STOP:
		set_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag);
		if (ctx->status == DEC_STATUS_DECODING) {
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_CMD_STOP, NULL);
			ctx->status = DEC_STATUS_DRAINING;
		} else
			ret = vsi_dec_handlestop_unspec(ctx);
		break;
	case V4L2_DEC_CMD_START:
		ret = vsi_dec_start_cmd(ctx);
		break;
	case V4L2_DEC_CMD_RESET:
		ret = vsi_v4l2_reset_ctx(ctx);
		break;
	case V4L2_DEC_CMD_PAUSE:
	case V4L2_DEC_CMD_RESUME:
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&ctx->ctxlock);
	return ret;
}


static const struct v4l2_ioctl_ops vsi_dec_ioctl = {
	.vidioc_querycap = vsi_dec_querycap,
	.vidioc_reqbufs             = vsi_dec_reqbufs,
	.vidioc_prepare_buf         = vsi_dec_prepare_buf,
	//create_buf can be provided now since we don't know buf type in param
	.vidioc_querybuf            = vsi_dec_querybuf,
	.vidioc_qbuf                = vsi_dec_qbuf,
	.vidioc_dqbuf               = vsi_dec_dqbuf,
	.vidioc_streamon        = vsi_dec_streamon,
	.vidioc_streamoff       = vsi_dec_streamoff,
	.vidioc_g_fmt_vid_cap = vsi_dec_g_fmt,
	//.vidioc_g_fmt_vid_cap_mplane = vsi_dec_g_fmt,
	.vidioc_s_fmt_vid_cap = vsi_dec_s_fmt,
	//.vidioc_s_fmt_vid_cap_mplane = vsi_dec_s_fmt,
	.vidioc_expbuf = vsi_dec_expbuf,		//this is used to export MMAP ptr as prime fd to user space app

	.vidioc_g_fmt_vid_out = vsi_dec_g_fmt,
	//.vidioc_g_fmt_vid_out_mplane = vsi_dec_g_fmt,
	.vidioc_s_fmt_vid_out = vsi_dec_s_fmt,
	//.vidioc_s_fmt_vid_out_mplane = vsi_dec_s_fmt,
	.vidioc_try_fmt_vid_cap = vsi_dec_try_fmt,
	//.vidioc_try_fmt_vid_cap_mplane = vsi_dec_try_fmt,
	.vidioc_try_fmt_vid_out = vsi_dec_try_fmt,
	//.vidioc_try_fmt_vid_out_mplane = vsi_dec_try_fmt,

	.vidioc_enum_fmt_vid_cap = vsi_dec_enum_fmt,
	.vidioc_enum_fmt_vid_out = vsi_dec_enum_fmt,

	.vidioc_g_fmt_vid_out = vsi_dec_g_fmt,
	//.vidioc_g_fmt_vid_out_mplane = vsi_dec_g_fmt,

	.vidioc_g_selection = vsi_dec_get_selection,		//VIDIOC_G_SELECTION, VIDIOC_G_CROP

	.vidioc_subscribe_event = vsi_dec_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
	.vidioc_try_decoder_cmd = vsi_dec_try_decoder_cmd,
	.vidioc_decoder_cmd = vsi_dec_decoder_cmd,
};

/*setup buffer information before real allocation*/
static int vsi_dec_queue_setup(
	struct vb2_queue *vq,
	unsigned int *nbuffers,
	unsigned int *nplanes,
	unsigned int sizes[],
	struct device *alloc_devs[])
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(vq->drv_priv);
	int i;

	vsiv4l2_buffer_config(ctx, vq, nbuffers, nplanes, sizes);
	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d,%d,%d", ctx->ctxid, __func__, *nbuffers, *nplanes, sizes[0]);

	for (i = 0; i < *nplanes; i++)
		alloc_devs[i] = ctx->dev->dev;
	return 0;
}

static void vsi_dec_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(vq->drv_priv);
	struct vsi_vpu_buf *vsibuf;
	int ret;

	v4l2_klog(LOGLVL_FLOW, "%s:%d:%d", __func__, vq->type, vb->index);
	vsibuf = vb_to_vsibuf(vb);
	if (!binputqueue(vq->type)) {
		set_bit(BUF_FLAG_QUEUED, &ctx->vbufflag[vb->index]);
		list_add_tail(&vsibuf->list, &ctx->output_list);
	} else {
		set_bit(BUF_FLAG_QUEUED, &ctx->srcvbufflag[vb->index]);
		list_add_tail(&vsibuf->list, &ctx->input_list);
		ctx->queued_srcnum++;
	}
	ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_BUF_RDY, vb);
}

static int vsi_dec_buf_init(struct vb2_buffer *vb)
{
	return 0;
}

static int vsi_dec_buf_prepare(struct vb2_buffer *vb)
{
	return 0;
}

static int vsi_dec_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(q->drv_priv);

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		ctx->out_sequence = 0;
	else
		ctx->cap_sequence = 0;

	return 0;
}
static void vsi_dec_stop_streaming(struct vb2_queue *vq)
{
}

static void vsi_dec_buf_finish(struct vb2_buffer *vb)
{
}

static void vsi_dec_buf_cleanup(struct vb2_buffer *vb)
{
}

static void vsi_dec_buf_wait_finish(struct vb2_queue *vq)
{
	vb2_ops_wait_finish(vq);
}

static void vsi_dec_buf_wait_prepare(struct vb2_queue *vq)
{
	vb2_ops_wait_prepare(vq);
}

static struct vb2_ops vsi_dec_qops = {
	.queue_setup = vsi_dec_queue_setup,
	.wait_prepare = vsi_dec_buf_wait_prepare,	/*these two are just mutex protection for done_que*/
	.wait_finish = vsi_dec_buf_wait_finish,
	.buf_init = vsi_dec_buf_init,
	.buf_prepare = vsi_dec_buf_prepare,
	.buf_finish = vsi_dec_buf_finish,
	.buf_cleanup = vsi_dec_buf_cleanup,
	.start_streaming = vsi_dec_start_streaming,
	.stop_streaming = vsi_dec_stop_streaming,
	.buf_queue = vsi_dec_buf_queue,
};

static int vsi_v4l2_dec_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;
	struct vsi_v4l2_ctx *ctx = ctrl_to_ctx(ctrl);

	v4l2_klog(LOGLVL_CONFIG, "%s:%x=%d", __func__, ctrl->id, ctrl->val);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_VP8_PROFILE:
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		ret = vsi_set_profile(ctx, ctrl->id, ctrl->val);
		return ret;
	case V4L2_CID_DIS_REORDER:
		ctx->mediacfg.decparams.io_buffer.no_reordering_decoding = ctrl->val;
		break;
	case V4L2_CID_SECUREMODE:
		ctx->mediacfg.decparams.io_buffer.securemode_on = ctrl->val;
		break;
	default:
		return 0;
	}
	return 0;
}

static int vsi_v4l2_dec_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vsi_v4l2_ctx *ctx = ctrl_to_ctx(ctrl);

	v4l2_klog(LOGLVL_CONFIG, "%s:%x", __func__, ctrl->id);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	switch (ctrl->id) {
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ctrl->val = ctx->mediacfg.minbuf_4capture;	//these two may come from resoultion change
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_OUTPUT:
		ctrl->val = ctx->mediacfg.minbuf_4output;
		break;
	case V4L2_CID_HDR10META:
		if (ctrl->p_new.p) {
			if (!test_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag))
				memset(ctrl->p_new.p, 0, sizeof(struct v4l2_hdr10_meta));
			else
				memcpy(ctrl->p_new.p,
					&ctx->mediacfg.decparams.dec_info.dec_info.vpu_hdr10_meta,
					sizeof(struct v4l2_hdr10_meta));
		}
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/********* for ext ctrl *************/
static bool vsi_dec_ctrl_equal(const struct v4l2_ctrl *ctrl,
		      union v4l2_ctrl_ptr ptr1,
		      union v4l2_ctrl_ptr ptr2)
{
	//always update now, fix it later
	return 0;
}

static void vsi_dec_ctrl_init(const struct v4l2_ctrl *ctrl, u32 from_idx,
			      union v4l2_ctrl_ptr ptr)
{
	void *p = ptr.p + from_idx * ctrl->elem_size;

	memset(p, 0, (ctrl->elems - from_idx) * ctrl->elem_size);
}

static void vsi_dec_ctrl_log(const struct v4l2_ctrl *ctrl)
{
	//do nothing now
}

static int vsi_dec_ctrl_validate(const struct v4l2_ctrl *ctrl,
			union v4l2_ctrl_ptr ptr)
{
	//always true
	return 0;
}

static const struct v4l2_ctrl_type_ops vsi_dec_type_ops = {
	.equal = vsi_dec_ctrl_equal,
	.init = vsi_dec_ctrl_init,
	.log = vsi_dec_ctrl_log,
	.validate = vsi_dec_ctrl_validate,
};
/********* for ext ctrl *************/

static const struct v4l2_ctrl_ops vsi_dec_ctrl_ops = {
	.s_ctrl = vsi_v4l2_dec_s_ctrl,
	.g_volatile_ctrl = vsi_v4l2_dec_g_volatile_ctrl,
};

static struct v4l2_ctrl_config vsi_v4l2_dec_ctrl_defs[] = {
	{
		.ops = &vsi_dec_ctrl_ops,
		.id = V4L2_CID_DIS_REORDER,
		.name = "frame disable reorder ctrl",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &vsi_dec_ctrl_ops,
		.type_ops = &vsi_dec_type_ops,
		.id = V4L2_CID_HDR10META,
		.name = "vsi get 10bit meta",
		.type = VSI_V4L2_CMPTYPE_HDR10META,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.elem_size = sizeof(struct v4l2_hdr10_meta),
	},
	/* kernel defined controls */
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
		.max = V4L2_MPEG_VIDEO_H264_PROFILE_MULTIVIEW_HIGH,
		.def = V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VP8_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_VP8_PROFILE_0,
		.max = V4L2_MPEG_VIDEO_VP8_PROFILE_3,
		.def = V4L2_MPEG_VIDEO_VP8_PROFILE_0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VP9_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
		.max = V4L2_MPEG_VIDEO_VP9_PROFILE_3,
		.def = V4L2_MPEG_VIDEO_VP9_PROFILE_0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_PROFILE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min =  V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
		.max = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN_10,
		.def = V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN,
	},
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_VOLATILE,	//volatile contains read
		.min = 1,
		.max = MAX_MIN_BUFFERS_FOR_CAPTURE,
		.step = 1,
		.def = 1,
	},
	{
		.id = V4L2_CID_MIN_BUFFERS_FOR_OUTPUT,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
		.min = 1,
		.max = MAX_MIN_BUFFERS_FOR_OUTPUT,
		.step = 1,
		.def = 1,
	},
	{
		.ops = &vsi_dec_ctrl_ops,
		.id = V4L2_CID_SECUREMODE,
		.name = "en/disable secure mode",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
};

static int vsi_dec_setup_ctrls(struct v4l2_ctrl_handler *handler)
{
	int i, ctrl_num = ARRAY_SIZE(vsi_v4l2_dec_ctrl_defs);
	struct v4l2_ctrl *ctrl = NULL;

	v4l2_ctrl_handler_init(handler, ctrl_num);

	if (handler->error)
		return handler->error;

	for (i = 0; i < ctrl_num; i++) {
		vsi_v4l2_update_ctrlcfg(&vsi_v4l2_dec_ctrl_defs[i]);
		if (is_vsi_ctrl(vsi_v4l2_dec_ctrl_defs[i].id))
			ctrl = v4l2_ctrl_new_custom(handler, &vsi_v4l2_dec_ctrl_defs[i], NULL);
		else {
			if (vsi_v4l2_dec_ctrl_defs[i].type == V4L2_CTRL_TYPE_MENU) {
				ctrl = v4l2_ctrl_new_std_menu(handler, &vsi_dec_ctrl_ops,
					vsi_v4l2_dec_ctrl_defs[i].id,
					vsi_v4l2_dec_ctrl_defs[i].max,
					0,
					vsi_v4l2_dec_ctrl_defs[i].def);
			} else {
				ctrl = v4l2_ctrl_new_std(handler,
					&vsi_dec_ctrl_ops,
					vsi_v4l2_dec_ctrl_defs[i].id,
					vsi_v4l2_dec_ctrl_defs[i].min,
					vsi_v4l2_dec_ctrl_defs[i].max,
					vsi_v4l2_dec_ctrl_defs[i].step,
					vsi_v4l2_dec_ctrl_defs[i].def);
			}
		}
		if (ctrl && (vsi_v4l2_dec_ctrl_defs[i].flags & V4L2_CTRL_FLAG_VOLATILE))
			ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

		if (handler->error) {
			v4l2_klog(LOGLVL_ERROR, "%s fail to set ctrl %d:%d", __func__, i, handler->error);
			break;
		}
	}

	v4l2_ctrl_handler_setup(handler);
	return handler->error;
}

static int v4l2_dec_open(struct file *filp)
{
	//struct video_device *vdev = video_devdata(filp);
	struct vsi_v4l2_device *dev = video_drvdata(filp);
	struct vsi_v4l2_ctx *ctx = NULL;
	struct vb2_queue *q;
	int ret = 0;
	struct v4l2_fh *vfh;
	pid_t  pid;

	/* Allocate memory for context */
	//fh->video_devdata = struct video_device, struct video_device->video_drvdata = struct vsi_v4l2_device
	if (vsi_v4l2_addinstance(&pid) < 0)
		return -EBUSY;

	ctx = vsi_create_ctx();
	if (ctx == NULL) {
		vsi_v4l2_quitinstance();
		return -ENOMEM;
	}

	v4l2_fh_init(&ctx->fh, video_devdata(filp));
	filp->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);
	ctx->dev = dev;
	mutex_init(&ctx->ctxlock);
	ctx->flag = CTX_FLAG_DEC;
	set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);

	ctx->frameidx = 0;
	q = &ctx->input_que;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->min_buffers_needed = 1;
	q->drv_priv = &ctx->fh;
	q->lock = &ctx->ctxlock;
	q->buf_struct_size = sizeof(struct vsi_vpu_buf);		//used to alloc mem control structures in reqbuf
	q->ops = &vsi_dec_qops;		/*it might be used to identify input and output */
	q->mem_ops = &vb2_dma_contig_memops;
	q->memory = VB2_MEMORY_UNKNOWN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	INIT_LIST_HEAD(&ctx->input_list);
	ret = vb2_queue_init(q);
	/*q->buf_ops = &v4l2_buf_ops is set here*/
	if (ret)
		goto err_enc_dec_exit;

	q = &ctx->output_que;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = &ctx->fh;
	q->lock = &ctx->ctxlock;
	q->buf_struct_size = sizeof(struct vsi_vpu_buf);
	q->ops = &vsi_dec_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->memory = VB2_MEMORY_UNKNOWN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->min_buffers_needed = 1;
	INIT_LIST_HEAD(&ctx->output_list);
	ret = vb2_queue_init(q);
	if (ret) {
		vb2_queue_release(&ctx->input_que);
		goto err_enc_dec_exit;
	}
	q->quirk_poll_must_check_waiting_for_buffers = false;
	vsiv4l2_initcfg(ctx);
	vsi_dec_setup_ctrls(&ctx->ctrlhdl);
	vfh = (struct v4l2_fh *)filp->private_data;
	vfh->ctrl_handler = &ctx->ctrlhdl;
	atomic_set(&ctx->srcframen, 0);
	atomic_set(&ctx->dstframen, 0);
	ctx->status = VSI_STATUS_INIT;

	//dev->vdev->queue = q;
	//single queue is used for v4l2 default ops such as ioctl, read, write and poll
	//If we wanna manage queue by ourselves, leave it null and don't use default v4l2 ioctl/read/write/poll interfaces.

	return 0;

err_enc_dec_exit:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	vsi_remove_ctx(ctx);
	kfree(ctx);
	vsi_v4l2_quitinstance();
	return ret;
}

static int v4l2_dec_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	int ret;

	v4l2_klog(LOGLVL_FLOW, "%s", __func__);
	if (offset < OUTF_BASE) {
		ret = vb2_mmap(&ctx->input_que, vma);
	} else {
		vma->vm_pgoff -= (OUTF_BASE >> PAGE_SHIFT);
		offset -= OUTF_BASE;
		ret = vb2_mmap(&ctx->output_que, vma);
	}
	return ret;
}

static __poll_t vsi_dec_poll(struct file *file, poll_table *wait)
{
	__poll_t ret = 0;
	struct v4l2_fh *fh = file->private_data;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	int dstn = atomic_read(&ctx->dstframen);
	int srcn = atomic_read(&ctx->srcframen);

	/*
	 * poll_wait() MUST be called on the first invocation on all the
	 * potential queues of interest, even if we are not interested in their
	 * events during this first call. Failure to do so will result in
	 * queue's events to be ignored because the poll_table won't be capable
	 * of adding new wait queues thereafter.
	 */
	poll_wait(file, &ctx->input_que.done_wq, wait);
	poll_wait(file, &ctx->output_que.done_wq, wait);
	poll_wait(file, &fh->wait, wait);

	if (!vsi_v4l2_daemonalive())
		ret |= EPOLLERR;

	if (v4l2_event_pending(&ctx->fh)) {
		v4l2_klog(LOGLVL_BRIEF, "%s event", __func__);
		ret |= EPOLLPRI;
	}
	if (ctx->output_que.last_buffer_dequeued)
		ret |= (EPOLLIN | EPOLLRDNORM);
	if (vb2_is_streaming(&ctx->output_que))
		ret |= vb2_poll(&ctx->output_que, file, wait);
	ret |= vb2_poll(&ctx->input_que, file, wait);

	/*recheck for poll hang*/
	if (ret == 0) {
		if (dstn != atomic_read(&ctx->dstframen))
			ret |= vb2_poll(&ctx->output_que, file, wait);
		if (srcn != atomic_read(&ctx->srcframen))
			ret |= vb2_poll(&ctx->input_que, file, wait);
	}
	if (ctx->error < 0)
		ret |= EPOLLERR;
	v4l2_klog(LOGLVL_VERBOSE, "%s:%x", __func__, ret);
	return ret;
}

static const struct v4l2_file_operations v4l2_dec_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_dec_open,
	.release = vsi_v4l2_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_dec_mmap,
	.poll = vsi_dec_poll,
};

struct video_device *vsi_v4l2_probe_dec(struct platform_device *pdev, struct vsi_v4l2_device *vpu)
{
	struct video_device *vdec;
	int ret = 0;

	v4l2_klog(LOGLVL_BRIEF, "%s", __func__);
	vdec = video_device_alloc();
	if (!vdec) {
		v4l2_err(&vpu->v4l2_dev, "Failed to allocate dec device\n");
		ret = -ENOMEM;
		goto err;
	}

	vdec->fops = &v4l2_dec_fops;
	vdec->ioctl_ops = &vsi_dec_ioctl;
	vdec->device_caps = V4L2_CAP_VIDEO_M2M | V4L2_CAP_STREAMING;
	vdec->release = video_device_release;
	vdec->lock = &vpu->lock;
	vdec->v4l2_dev = &vpu->v4l2_dev;
	vdec->vfl_dir = VFL_DIR_M2M;
	vdec->vfl_type = VSI_DEVTYPE;
	vpu->vdec = vdec;
	vdec->queue = NULL;

	video_set_drvdata(vdec, vpu);

	ret = video_register_device(vdec, VSI_DEVTYPE, 0);
	if (ret) {
		v4l2_err(&vpu->v4l2_dev, "Failed to register dec device\n");
		video_device_release(vdec);
		goto err;
	}
	return vdec;

err:
	return NULL;
}

void vsi_v4l2_release_dec(struct video_device *vdec)
{
	video_unregister_device(vdec);
}

