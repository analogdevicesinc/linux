/*
 *    VSI V4L2 encoder entry.
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

static int vsi_enc_querycap(
	struct file *file,
	void *priv,
	struct v4l2_capability *cap)
{
	struct vsi_v4l2_dev_info *hwinfo;

	v4l2_klog(LOGLVL_CONFIG, "%s", __func__);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	hwinfo = vsiv4l2_get_hwinfo();
	if (hwinfo->encformat == 0)
		return -ENODEV;

	strlcpy(cap->driver, "vsi_v4l2", sizeof("vsi_v4l2"));
	strlcpy(cap->card, "vsi_v4l2enc", sizeof("vsi_v4l2enc"));
	strlcpy(cap->bus_info, "platform:vsi_v4l2enc", sizeof("platform:vsi_v4l2enc"));

	cap->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int vsi_enc_reqbufs(
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
	if (!binputqueue(p->type) && p->count == 0)
		set_bit(CTX_FLAG_ENC_FLUSHBUF, &ctx->flag);
	v4l2_klog(LOGLVL_BRIEF, "%llx:%s:%d ask for %d buffer, got %d:%d:%d",
		ctx->ctxid, __func__, p->type, p->count, q->num_buffers, ret, ctx->status);
	return ret;
}

static int vsi_enc_create_bufs(struct file *filp, void *priv,
				struct v4l2_create_buffers *create)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);
	int ret;
	struct vb2_queue *q;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(create->format.type, ctx->flag))
		return -EINVAL;

	if (binputqueue(create->format.type))
		q = &ctx->input_que;
	else
		q = &ctx->output_que;

	ret = vb2_create_bufs(q, create);

	if (!binputqueue(create->format.type) && create->count == 0)
		set_bit(CTX_FLAG_ENC_FLUSHBUF, &ctx->flag);
	v4l2_klog(LOGLVL_BRIEF, "%llx:%s:%d create for %d buffer, got %d:%d:%d\n",
		ctx->ctxid, __func__, create->format.type, create->count,
		q->num_buffers, ret, ctx->status);
	return ret;
}

static int vsi_enc_s_parm(struct file *filp, void *priv, struct v4l2_streamparm *parm)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);

	v4l2_klog(LOGLVL_CONFIG, "%s", __func__);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;

	if (!isvalidtype(parm->type, ctx->flag))
		return -EINVAL;

	if (!binputqueue(parm->type))
		return -EINVAL;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;

	memset(parm->parm.output.reserved, 0, sizeof(parm->parm.output.reserved));
	if (!parm->parm.output.timeperframe.denominator)
		parm->parm.output.timeperframe.denominator = ctx->mediacfg.outputparam.timeperframe.denominator;
	else
		ctx->mediacfg.outputparam.timeperframe.denominator = parm->parm.output.timeperframe.denominator;
	if (!parm->parm.output.timeperframe.numerator)
		parm->parm.output.timeperframe.numerator = ctx->mediacfg.outputparam.timeperframe.numerator;
	else
		ctx->mediacfg.outputparam.timeperframe.numerator = parm->parm.output.timeperframe.numerator;
	ctx->mediacfg.encparams.general.inputRateNumer = parm->parm.output.timeperframe.denominator;
	ctx->mediacfg.encparams.general.inputRateDenom = parm->parm.output.timeperframe.numerator;
	ctx->mediacfg.encparams.general.outputRateNumer = parm->parm.output.timeperframe.denominator;
	ctx->mediacfg.encparams.general.outputRateDenom = parm->parm.output.timeperframe.numerator;
	parm->parm.output.capability = V4L2_CAP_TIMEPERFRAME;

	set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);
	mutex_unlock(&ctx->ctxlock);

	v4l2_klog(LOGLVL_BRIEF, "%llx:%s set fps number %d,denom %d\n",
		ctx->ctxid, __func__,  ctx->mediacfg.encparams.general.inputRateNumer, ctx->mediacfg.encparams.general.inputRateDenom);
	return 0;
}

static int vsi_enc_g_parm(struct file *filp, void *priv, struct v4l2_streamparm *parm)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);

	v4l2_klog(LOGLVL_CONFIG, "%s", __func__);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(parm->type, ctx->flag))
		return -EINVAL;
	if (!binputqueue(parm->type))
		return -EINVAL;

	parm->parm.output = ctx->mediacfg.outputparam;

	return 0;
}

static int vsi_enc_g_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);

	v4l2_klog(LOGLVL_CONFIG, "%s:%d", __func__, f->type);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(f->type, ctx->flag))
		return -EINVAL;
	return vsiv4l2_getfmt(ctx, f);
}

static int vsi_enc_s_fmt(struct file *file, void *priv, struct v4l2_format *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	int ret;

	v4l2_klog(LOGLVL_CONFIG, "%s fmt:%x, res:%dx%d\n", __func__,
		  f->fmt.pix_mp.pixelformat, f->fmt.pix_mp.width,
		  f->fmt.pix_mp.height);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(f->type, ctx->flag))
		return -EINVAL;
	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	ret = vsiv4l2_setfmt(ctx, f);
	set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_enc_querybuf(
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
	v4l2_klog(LOGLVL_FLOW, "%s:%lx:%d:%d", __func__, ctx->flag, buf->type, buf->index);
	ret = vb2_querybuf(q, buf);
	if (buf->memory == V4L2_MEMORY_MMAP) {
		if (ret == 0 && q == &ctx->output_que)
			buf->m.planes[0].m.mem_offset += OUTF_BASE;
	}

	return ret;
}

static int vsi_enc_trystartenc(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	v4l2_klog(LOGLVL_BRIEF, "%s:streaming:%d:%d, queued buf:%d:%d",
		__func__, ctx->input_que.streaming, ctx->output_que.streaming,
		ctx->input_que.queued_count, ctx->output_que.queued_count);
	if (vb2_is_streaming(&ctx->input_que) && vb2_is_streaming(&ctx->output_que)) {
		if ((ctx->status == VSI_STATUS_INIT ||
			ctx->status == ENC_STATUS_STOPPED ||
			ctx->status == ENC_STATUS_EOS) &&
			ctx->input_que.queued_count >= ctx->input_que.min_buffers_needed &&
			ctx->output_que.queued_count >= ctx->output_que.min_buffers_needed) {
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMON, NULL);
			if (ret == 0) {
				ctx->status = ENC_STATUS_ENCODING;
				if (test_and_clear_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag)) {
					ret |= vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_CMD_STOP, NULL);
					ctx->status = ENC_STATUS_DRAINING;
				}
			}
		}
	}
	return ret;
}

static int vsi_enc_qbuf(struct file *filp, void *priv, struct v4l2_buffer *buf)
{
	int ret;
	//struct vb2_queue *vq = vb->vb2_queue;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);
	struct video_device *vdev = ctx->dev->venc;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(buf->type, ctx->flag))
		return -EINVAL;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;

	if (!binputqueue(buf->type))
		ret = vb2_qbuf(&ctx->output_que, vdev->v4l2_dev->mdev, buf);
	else {
		if (test_and_clear_bit(CTX_FLAG_FORCEIDR_BIT, &ctx->flag))
			ctx->srcvbufflag[buf->index] |= FORCE_IDR;

		ret = vb2_qbuf(&ctx->input_que, vdev->v4l2_dev->mdev, buf);
	}
	v4l2_klog(LOGLVL_FLOW, "%llx:%s:%d:%d:%d, %d:%d, %d:%d",
		ctx->ctxid, __func__, buf->type, buf->index, buf->bytesused,
		buf->m.planes[0].bytesused, buf->m.planes[0].length,
		buf->m.planes[1].bytesused, buf->m.planes[1].length);
	if (ret == 0 && ctx->status != ENC_STATUS_ENCODING && ctx->status != ENC_STATUS_EOS)
		ret = vsi_enc_trystartenc(ctx);
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_enc_streamon(struct file *filp, void *priv, enum v4l2_buf_type type)
{
	int ret = 0;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);

	v4l2_klog(LOGLVL_BRIEF, "%s:%d", __func__, type);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(type, ctx->flag))
		return -EINVAL;
	if (ctx->status == ENC_STATUS_ENCODING)
		return 0;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	if (!binputqueue(type)) {
		ret = vb2_streamon(&ctx->output_que, type);
		printbufinfo(&ctx->output_que);
	} else {
		ret = vb2_streamon(&ctx->input_que, type);
		printbufinfo(&ctx->input_que);
	}

	if (ret == 0) {
		if (ctx->status == ENC_STATUS_EOS) {
			//to avoid no queued buf when streamon
			ctx->status = ENC_STATUS_STOPPED;
		}
		ret = vsi_enc_trystartenc(ctx);
	}

	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_enc_streamoff(
	struct file *file,
	void *priv,
	enum v4l2_buf_type type)
{
	int i, ret;
	u32 binput = binputqueue(type);
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(priv);
	struct vb2_queue *q;

	v4l2_klog(LOGLVL_BRIEF, "%s:%d", __func__, type);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(type, ctx->flag))
		return -EINVAL;
	if (ctx->status == VSI_STATUS_INIT)
		return 0;

	if (binput)
		q = &ctx->input_que;
	else
		q = &ctx->output_que;

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	if (binput)
		vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMOFF_OUTPUT, &binput);
	else
		vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_STREAMOFF_CAPTURE, &binput);
	mutex_unlock(&ctx->ctxlock);

	if (binput)
		ret = wait_event_interruptible(ctx->capoffdone_queue, vsi_checkctx_outputoffdone(ctx));
	else
		ret = wait_event_interruptible(ctx->capoffdone_queue, vsi_checkctx_capoffdone(ctx));
	if (ret != 0)
		v4l2_klog(LOGLVL_WARNING, "%llx binput:%d, enc wait strmoff done fail\n",
			  ctx->ctxid, binput);

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	ctx->status = ENC_STATUS_STOPPED;
	if (binput) {
		clear_bit(CTX_FLAG_FORCEIDR_BIT, &ctx->flag);
		for (i = 0; i < VIDEO_MAX_FRAME; i++)
			ctx->srcvbufflag[i] = 0;
	}

	return_all_buffers(q, VB2_BUF_STATE_DONE, 1);
	ret = vb2_streamoff(q, type);
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_enc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
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

	if (ctx->status == ENC_STATUS_STOPPED ||
		ctx->status == ENC_STATUS_EOS) {
		p->bytesused = 0;
		return -EPIPE;
	}

	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	ret = vb2_dqbuf(q, p, file->f_flags & O_NONBLOCK);
	if (ret == 0) {
		vb = q->bufs[p->index];
		vsibuf = vb_to_vsibuf(vb);
		list_del(&vsibuf->list);
		p->flags &= ~(V4L2_BUF_FLAG_KEYFRAME | V4L2_BUF_FLAG_PFRAME | V4L2_BUF_FLAG_BFRAME);
		if (!binputqueue(p->type)) {
			if (ctx->vbufflag[p->index] & FRAMETYPE_I)
				p->flags |= V4L2_BUF_FLAG_KEYFRAME;
			else  if (ctx->vbufflag[p->index] & FRAMETYPE_P)
				p->flags |= V4L2_BUF_FLAG_PFRAME;
			else  if (ctx->vbufflag[p->index] & FRAMETYPE_B)
				p->flags |= V4L2_BUF_FLAG_BFRAME;
		}
	}
	if (!binputqueue(p->type)) {
		if (ret == 0) {
			if (ctx->vbufflag[p->index] & LAST_BUFFER_FLAG) {
				vsi_v4l2_sendeos(ctx);
				if (ctx->status == ENC_STATUS_DRAINING)
					ctx->status = ENC_STATUS_EOS;
				v4l2_klog(LOGLVL_BRIEF, "dqbuf get eos flag");
			}
		}
	}
	mutex_unlock(&ctx->ctxlock);
	v4l2_klog(LOGLVL_FLOW, "%s:%d:%d:%d:%x:%d", __func__, p->type, p->index, ret, p->flags, ctx->status);
	return ret;
}

static int vsi_enc_prepare_buf(
	struct file *file,
	void *priv,
	struct v4l2_buffer *p)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *q;
	struct video_device *vdev = ctx->dev->venc;

	v4l2_klog(LOGLVL_FLOW, "%s:%d", __func__, p->type);
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

static int vsi_enc_expbuf(
	struct file *file,
	void *priv,
	struct v4l2_exportbuffer *p)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vb2_queue *q;

	v4l2_klog(LOGLVL_FLOW, "%s:%d", __func__, p->type);
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

static int vsi_enc_try_fmt(struct file *file, void *prv, struct v4l2_format *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;

	vsiv4l2_verifyfmt(ctx, f, 1);
	return 0;
}

static int vsi_enc_enum_fmt(struct file *file, void *prv, struct v4l2_fmtdesc *f)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vsi_video_fmt *pfmt;
	int braw = brawfmt(ctx->flag, f->type);

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (!isvalidtype(f->type, ctx->flag))
		return -EINVAL;

	pfmt = vsi_enum_encformat(f->index, braw);
	if (pfmt == NULL)
		return -EINVAL;

	if (pfmt->name && strlen(pfmt->name))
		strlcpy(f->description, pfmt->name, strlen(pfmt->name) + 1);
	f->pixelformat = pfmt->fourcc;
	f->flags = pfmt->flag;
	v4l2_klog(LOGLVL_CONFIG, "%s:%d:%d:%x", __func__, f->index, f->type, pfmt->fourcc);
	return 0;
}

static int vsi_enc_valid_crop(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_daemon_enc_general_cmd *general = &ctx->mediacfg.encparams.general;
	struct v4l2_frmsizeenum fsize;

	vsi_enum_encfsize(&fsize, ctx->mediacfg.outfmt_fourcc);

	general->horOffsetSrc = ALIGN(general->horOffsetSrc, fsize.stepwise.step_width);
	general->verOffsetSrc = ALIGN(general->verOffsetSrc, fsize.stepwise.step_height);
	general->width = ALIGN(general->width, fsize.stepwise.step_width);
	general->height = ALIGN(general->height, fsize.stepwise.step_height);

	general->width = min(general->width, ctx->mediacfg.width_src - general->horOffsetSrc);
	general->width = max_t(u32, general->width, fsize.stepwise.min_width);
	general->height = min(general->height, ctx->mediacfg.height_src - general->verOffsetSrc);
	general->height = max_t(u32, general->height, fsize.stepwise.min_height);

	return 0;
}

static int vsi_enc_set_selection(struct file *file, void *prv, struct v4l2_selection *s)
{
	int ret = 0;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
		s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;
	if (s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	ret = vsiv4l2_verifycrop(s);
	if (!ret) {
		if (mutex_lock_interruptible(&ctx->ctxlock))
			return -EBUSY;
		pcfg->encparams.general.horOffsetSrc = s->r.left;
		pcfg->encparams.general.verOffsetSrc = s->r.top;
		pcfg->encparams.general.width = s->r.width;
		pcfg->encparams.general.height = s->r.height;
		vsi_enc_valid_crop(ctx);
		set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);
		mutex_unlock(&ctx->ctxlock);
	}
	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d,%d,%d,%d",
		ctx->ctxid, __func__, s->r.left, s->r.top, s->r.width, s->r.height);

	return ret;
}

static int vsi_enc_get_selection(struct file *file, void *prv, struct v4l2_selection *s)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
		s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r.left = pcfg->encparams.general.horOffsetSrc;
		s->r.top = pcfg->encparams.general.verOffsetSrc;
		s->r.width = pcfg->encparams.general.width;
		s->r.height = pcfg->encparams.general.height;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = pcfg->width_src;
		s->r.height = pcfg->height_src;
		break;
	default:
		return -EINVAL;
	}
	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d,%d,%d,%d",
		ctx->ctxid, __func__, s->r.left, s->r.top, s->r.width, s->r.height);

	return 0;
}

static int vsi_enc_subscribe_event(
	struct v4l2_fh *fh,
	const struct v4l2_event_subscription *sub)
{
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;

	v4l2_klog(LOGLVL_CONFIG, "%s:%d", __func__, sub->type);
	switch (sub->type) {
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	case V4L2_EVENT_SKIP:
		return v4l2_event_subscribe(fh, sub, 16, NULL);
	case V4L2_EVENT_EOS:
	case V4L2_EVENT_CODEC_ERROR:
	case V4L2_EVENT_INVALID_OPTION:
		return v4l2_event_subscribe(fh, sub, 0, NULL);
	default:
		return -EINVAL;
	}
}

static int vsi_enc_try_encoder_cmd(struct file *file, void *fh, struct v4l2_encoder_cmd *cmd)
{
	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
	case V4L2_ENC_CMD_START:
	case V4L2_ENC_CMD_PAUSE:
	case V4L2_ENC_CMD_RESUME:
		cmd->flags = 0;
		return 0;
	default:
		return -EINVAL;
	}
}

static int vsi_enc_encoder_cmd(struct file *file, void *fh, struct v4l2_encoder_cmd *cmd)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	int ret = 0;

	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (mutex_lock_interruptible(&ctx->ctxlock))
		return -EBUSY;
	v4l2_klog(LOGLVL_BRIEF, "%s:%d:%d", __func__, ctx->status, cmd->cmd);
	switch (cmd->cmd) {
	case V4L2_ENC_CMD_STOP:
		set_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag);
		if (ctx->status == ENC_STATUS_ENCODING) {
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_CMD_STOP, cmd);
			if (ret == 0) {
				ctx->status = ENC_STATUS_DRAINING;
				clear_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag);
			}
		}
		break;
	case V4L2_ENC_CMD_START:
		if (ctx->status == ENC_STATUS_STOPPED ||
			ctx->status == ENC_STATUS_EOS) {
			ret = vb2_streamon(&ctx->input_que, V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE);
			if (ret)
				break;

			ret = vb2_streamon(&ctx->output_que, V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
			if (ret)
				break;

			ret = vsi_enc_trystartenc(ctx);
		}
		break;
	case V4L2_ENC_CMD_PAUSE:
	case V4L2_ENC_CMD_RESUME:
	default:
		ret = -EINVAL;
		break;
	}
	mutex_unlock(&ctx->ctxlock);
	return ret;
}

static int vsi_enc_encoder_enum_framesizes(struct file *file, void *priv,
				  struct v4l2_frmsizeenum *fsize)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(file->private_data);
	struct v4l2_format fmt;

	v4l2_klog(LOGLVL_CONFIG, "%s:%x", __func__, fsize->pixel_format);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	if (fsize->index != 0)		//only stepwise
		return -EINVAL;

	fmt.type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	fmt.fmt.pix_mp.pixelformat = fsize->pixel_format;
	if (vsi_find_format(ctx,  &fmt) != NULL)
		vsi_enum_encfsize(fsize, ctx->mediacfg.outfmt_fourcc);
	else {
		fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
		fmt.fmt.pix_mp.pixelformat = fsize->pixel_format;
		if (vsi_find_format(ctx,  &fmt) == NULL)
			return -EINVAL;
		vsi_enum_encfsize(fsize, fsize->pixel_format);
	}

	return 0;
}

/* ioctl handler */
/* take VIDIOC_S_INPUT for example, ioctl goes to V4l2-ioctl.c.: v4l_s_input() -> V4l2-dev.c: v4l2_ioctl_ops.vidioc_s_input() */
/* ioctl cmd could be disabled by v4l2_disable_ioctl() */
static const struct v4l2_ioctl_ops vsi_enc_ioctl = {
	.vidioc_querycap = vsi_enc_querycap,
	.vidioc_reqbufs             = vsi_enc_reqbufs,
	.vidioc_create_bufs         = vsi_enc_create_bufs,
	.vidioc_prepare_buf         = vsi_enc_prepare_buf,
	//create_buf can be provided now since we don't know buf type in param
	.vidioc_querybuf            = vsi_enc_querybuf,
	.vidioc_qbuf                = vsi_enc_qbuf,
	.vidioc_dqbuf               = vsi_enc_dqbuf,
	.vidioc_streamon        = vsi_enc_streamon,
	.vidioc_streamoff       = vsi_enc_streamoff,
	.vidioc_s_parm		= vsi_enc_s_parm,
	.vidioc_g_parm		= vsi_enc_g_parm,
	//.vidioc_g_fmt_vid_cap = vsi_enc_g_fmt,
	.vidioc_g_fmt_vid_cap_mplane = vsi_enc_g_fmt,
	//.vidioc_s_fmt_vid_cap = vsi_enc_s_fmt,
	.vidioc_s_fmt_vid_cap_mplane = vsi_enc_s_fmt,
	.vidioc_expbuf = vsi_enc_expbuf,		//this is used to export MMAP ptr as prime fd to user space app

	//.vidioc_g_fmt_vid_out = vsi_enc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vsi_enc_g_fmt,
	//.vidioc_s_fmt_vid_out = vsi_enc_s_fmt,
	.vidioc_s_fmt_vid_out_mplane = vsi_enc_s_fmt,
	//.vidioc_try_fmt_vid_cap = vsi_enc_try_fmt,
	.vidioc_try_fmt_vid_cap_mplane = vsi_enc_try_fmt,
	//.vidioc_try_fmt_vid_out = vsi_enc_try_fmt,
	.vidioc_try_fmt_vid_out_mplane = vsi_enc_try_fmt,
	.vidioc_enum_fmt_vid_cap = vsi_enc_enum_fmt,
	.vidioc_enum_fmt_vid_out = vsi_enc_enum_fmt,

	//.vidioc_g_fmt_vid_out = vsi_enc_g_fmt,
	.vidioc_g_fmt_vid_out_mplane = vsi_enc_g_fmt,

	.vidioc_s_selection = vsi_enc_set_selection,		//VIDIOC_S_SELECTION, VIDIOC_S_CROP
	.vidioc_g_selection = vsi_enc_get_selection,		//VIDIOC_G_SELECTION, VIDIOC_G_CROP

	.vidioc_subscribe_event = vsi_enc_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,

	.vidioc_try_encoder_cmd = vsi_enc_try_encoder_cmd,
	//fixme: encoder cmd stop will make streamoff not coming from ffmpeg. Maybe this is the right way to get finished, check later
	.vidioc_encoder_cmd = vsi_enc_encoder_cmd,
	.vidioc_enum_framesizes = vsi_enc_encoder_enum_framesizes,
};

/*setup buffer information before real allocation*/
static int vsi_enc_queue_setup(
	struct vb2_queue *vq,
	unsigned int *nbuffers,
	unsigned int *nplanes,
	unsigned int sizes[],
	struct device *alloc_devs[])
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(vq->drv_priv);
	int i, ret;

	v4l2_klog(LOGLVL_CONFIG, "%llx:%s:%d,%d,%d\n", ctx->ctxid, __func__, *nbuffers, *nplanes, sizes[0]);
	ret = vsiv4l2_buffer_config(ctx, vq, nbuffers, nplanes, sizes);
	if (ret == 0) {
		for (i = 0; i < *nplanes; i++)
			alloc_devs[i] = ctx->dev->dev;
	}
	return ret;
}

static void vsi_enc_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_queue *vq = vb->vb2_queue;
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(vq->drv_priv);
	struct vsi_vpu_buf *vsibuf;
	int ret;

	v4l2_klog(LOGLVL_FLOW, "%s:%d:%d", __func__, vb->type, vb->index);
	vsibuf = vb_to_vsibuf(vb);
	if (!binputqueue(vq->type))
		list_add_tail(&vsibuf->list, &ctx->output_list);
	else
		list_add_tail(&vsibuf->list, &ctx->input_list);
	ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_BUF_RDY, vb);
}

static int vsi_enc_buf_init(struct vb2_buffer *vb)
{
	return 0;
}

static int vsi_enc_buf_prepare(struct vb2_buffer *vb)
{
	/*any valid init operation on buffer vb*/
	/*gspca and rockchip both check buffer size here*/
	//like vb2_set_plane_payload(vb, 0, 1920*1080);
	return 0;
}

static int vsi_enc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(q->drv_priv);

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		ctx->out_sequence = 0;
	else
		ctx->cap_sequence = 0;

	return 0;
}
static void vsi_enc_stop_streaming(struct vb2_queue *vq)
{
}

static void vsi_enc_buf_finish(struct vb2_buffer *vb)
{
}

static void vsi_enc_buf_cleanup(struct vb2_buffer *vb)
{
}

static void vsi_enc_buf_wait_finish(struct vb2_queue *vq)
{
	vb2_ops_wait_finish(vq);
}

static void vsi_enc_buf_wait_prepare(struct vb2_queue *vq)
{
	vb2_ops_wait_prepare(vq);
}

static struct vb2_ops vsi_enc_qops = {
	.queue_setup = vsi_enc_queue_setup,
	.wait_prepare = vsi_enc_buf_wait_prepare,	/*these two are just mutex protection for done_que*/
	.wait_finish = vsi_enc_buf_wait_finish,
	.buf_init = vsi_enc_buf_init,
	.buf_prepare = vsi_enc_buf_prepare,
	.buf_finish = vsi_enc_buf_finish,
	.buf_cleanup = vsi_enc_buf_cleanup,
	.start_streaming = vsi_enc_start_streaming,
	.stop_streaming = vsi_enc_stop_streaming,
	.buf_queue = vsi_enc_buf_queue,
	//fill_user_buffer
	//int (*buf_out_validate)(struct vb2_buffer *vb);
	//void (*buf_request_complete)(struct vb2_buffer *vb);
};

static int vsi_v4l2_enc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	int ret;
	struct vsi_v4l2_ctx *ctx = ctrl_to_ctx(ctrl);

	v4l2_klog(LOGLVL_CONFIG, "%s:%x=%d", __func__, ctrl->id, ctrl->val);
	if (!vsi_v4l2_daemonalive())
		return -ENODEV;
	switch (ctrl->id) {
	case V4L2_CID_MPEG_VIDEO_GOP_SIZE:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.intraPicRate = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VP8_PROFILE:
	case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
	case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
	case V4L2_CID_MPEG_VIDEO_HEVC_PROFILE:
		ret = vsi_set_profile(ctx, ctrl->id, ctrl->val);
		return ret;
	case V4L2_CID_MPEG_VIDEO_BITRATE:
		ctx->mediacfg.encparams.general.bitPerSecond = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_LEVEL:
		ret = vsi_get_Level(ctx, 0, 1, ctrl->val);
		if (ret >= 0)
			ctx->mediacfg.encparams.specific.enc_h26x_cmd.avclevel = ret;
		else
			return ret;
		break;
	case V4L2_CID_MPEG_VIDEO_HEVC_LEVEL:
		ret = vsi_get_Level(ctx, 1, 1, ctrl->val);
		if (ret >= 0)
			ctx->mediacfg.encparams.specific.enc_h26x_cmd.hevclevel = ret;
		else
			return ret;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_MAX_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpMax_vpx = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MAX_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpMax_h26x = ctrl->val;
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpMaxI = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_MIN_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpMin_vpx = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_MIN_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpMin_h26x = ctrl->val;
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpMinI = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_B_FRAMES:
		if (ctrl->val != 0)
			return -EINVAL;
		/*in fact nothing to do*/
		break;
	case V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.bFrameQpDelta = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_BITRATE_MODE:
		if (ctrl->val == V4L2_MPEG_VIDEO_BITRATE_MODE_VBR)
			ctx->mediacfg.encparams.specific.enc_h26x_cmd.hrdConformance = 0;
		else
			ctx->mediacfg.encparams.specific.enc_h26x_cmd.hrdConformance = 1;
		break;
	case V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME:
		set_bit(CTX_FLAG_FORCEIDR_BIT, &ctx->flag);
		break;
	case V4L2_CID_MPEG_VIDEO_HEADER_MODE:
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE:
		ctx->mediacfg.multislice_mode = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.sliceSize = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.picRc = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.ctbRc = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpHdrI_h26x = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_I_FRAME_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpHdrI_vpx = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP:
	case V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpHdrP_h26x = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.cpbSize = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.chromaQpOffset = ctrl->val;
		break;
	case V4L2_CID_MPEG_VIDEO_VPX_P_FRAME_QP:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.qpHdrP_vpx = ctrl->val;
		break;
	case V4L2_CID_ROTATE:
		switch (ctrl->val) {
		case 90:
			ctx->mediacfg.encparams.general.rotation = VCENC_ROTATE_90L;
			break;
		case 180:
			ctx->mediacfg.encparams.general.rotation = VCENC_ROTATE_180R;
			break;
		case 270:
			ctx->mediacfg.encparams.general.rotation = VCENC_ROTATE_90R;
			break;
		case 0:
		default:
			ctx->mediacfg.encparams.general.rotation = VCENC_ROTATE_0;
			break;
		}
		break;
	case V4L2_CID_ROI:
		if (ctrl->p_new.p)
			vsiv4l2_setROI(ctx, ctrl->p_new.p);
		break;
	case V4L2_CID_IPCM:
		if (ctrl->p_new.p)
			vsiv4l2_setIPCM(ctx, ctrl->p_new.p);
		break;
	case V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER:
		ctx->mediacfg.encparams.specific.enc_h26x_cmd.idrHdr = ctrl->val;
		break;
	default:
		return 0;
	}
	set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);
	return 0;
}

static int vsi_v4l2_enc_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
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
	case V4L2_CID_ROI_COUNT:
		ctrl->val = vsiv4l2_getROIcount();
		break;
	case V4L2_CID_IPCM_COUNT:
		ctrl->val = vsiv4l2_getIPCMcount();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}
/********* for ext ctrl *************/
static bool vsi_enc_ctrl_equal(const struct v4l2_ctrl *ctrl,
		      union v4l2_ctrl_ptr ptr1,
		      union v4l2_ctrl_ptr ptr2)
{
	//always update now, fix it later
	return 0;
}

static void vsi_enc_ctrl_init(const struct v4l2_ctrl *ctrl, u32 from_idx,
			      union v4l2_ctrl_ptr ptr)
{
	void *p = ptr.p + from_idx * ctrl->elem_size;

	memset(p, 0, (ctrl->elems - from_idx) * ctrl->elem_size);
}

static void vsi_enc_ctrl_log(const struct v4l2_ctrl *ctrl)
{
	//do nothing now
}

static int vsi_enc_ctrl_validate(const struct v4l2_ctrl *ctrl,
			union v4l2_ctrl_ptr ptr)
{
	//always true
	return 0;
}

static const struct v4l2_ctrl_type_ops vsi_enc_type_ops = {
	.equal = vsi_enc_ctrl_equal,
	.init = vsi_enc_ctrl_init,
	.log = vsi_enc_ctrl_log,
	.validate = vsi_enc_ctrl_validate,
};
/********* for ext ctrl *************/

static const struct v4l2_ctrl_ops vsi_encctrl_ops = {
	.s_ctrl = vsi_v4l2_enc_s_ctrl,
	.g_volatile_ctrl = vsi_v4l2_enc_g_volatile_ctrl,
};

static struct v4l2_ctrl_config vsi_v4l2_encctrl_defs[] = {
	{
		.ops = &vsi_encctrl_ops,
		.id = V4L2_CID_ROI_COUNT,
		.name = "get max ROI region number",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = V4L2_MAX_ROI_REGIONS,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &vsi_encctrl_ops,
		.type_ops = &vsi_enc_type_ops,
		.id = V4L2_CID_ROI,
		.name = "vsi priv v4l2 roi params set",
		.type = VSI_V4L2_CMPTYPE_ROI,
		.min = 0,
		.max = V4L2_MAX_ROI_REGIONS,
		.step = 1,
		.def = 0,
		.elem_size = sizeof(struct v4l2_enc_roi_params),
	},
	{
		.ops = &vsi_encctrl_ops,
		.id = V4L2_CID_IPCM_COUNT,
		.name = "get max IPCM region number",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
		.min = 0,
		.max = V4L2_MAX_IPCM_REGIONS,
		.step = 1,
		.def = 0,
	},
	{
		.ops = &vsi_encctrl_ops,
		.type_ops = &vsi_enc_type_ops,
		.id = V4L2_CID_IPCM,
		.name = "vsi priv v4l2 ipcm params set",
		.type = VSI_V4L2_CMPTYPE_IPCM,
		.min = 0,
		.max = V4L2_MAX_IPCM_REGIONS,
		.step = 1,
		.def = 0,
		.elem_size = sizeof(struct v4l2_enc_ipcm_params),
	},
	/* kernel defined controls */
	{
		.id = V4L2_CID_MPEG_VIDEO_GOP_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = MAX_INTRA_PIC_RATE,
		.step = 1,
		.def = DEFAULT_INTRA_PIC_RATE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 10000,
		.max = 288000000,
		.step = 1,
		.def = 2097152,
	},
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
		.id = V4L2_CID_MPEG_VIDEO_H264_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_H264_LEVEL_1_0,
		.max = V4L2_MPEG_VIDEO_H264_LEVEL_5_2,
		.def = V4L2_MPEG_VIDEO_H264_LEVEL_5_0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_LEVEL,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_HEVC_LEVEL_1,
		.max = V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1,
		.def = V4L2_MPEG_VIDEO_HEVC_LEVEL_5,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 51,
		.step = 1,
		.def = 51,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 51,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 51,
		.step = 1,
		.def = 51,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 51,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEADER_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_HEADER_MODE_SEPARATE,
		.max = V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
		.def = V4L2_MPEG_VIDEO_HEADER_MODE_JOINED_WITH_1ST_FRAME,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_B_FRAMES,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_B_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 51,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_BITRATE_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
		.max = V4L2_MPEG_VIDEO_BITRATE_MODE_CBR,
		.def = V4L2_MPEG_VIDEO_BITRATE_MODE_VBR,
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
		.id = V4L2_CID_MPEG_VIDEO_FORCE_KEY_FRAME,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.min = 0,
		.max = 0,
		.step = 0,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 51,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 51,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_CPB_SIZE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 288000000,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_H264_CHROMA_QP_INDEX_OFFSET,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -12,
		.max = 12,
		.step = 1,
		.def = DEFAULT_CHROMA_QP_INDEX_OFFSET,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 51,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_HEVC_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 51,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MODE,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
		.max = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_MAX_MB,
		.def = V4L2_MPEG_VIDEO_MULTI_SLICE_MODE_SINGLE,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MULTI_SLICE_MAX_MB,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 120,		//1920 div 16
		.step = 1,
		.def = 1,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_FRAME_RC_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_MB_RC_ENABLE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_I_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 127,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_P_FRAME_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -1,
		.max = 127,
		.step = 1,
		.def = DEFAULT_QP,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_MIN_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 127,
		.step = 1,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_VPX_MAX_QP,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 127,
		.step = 1,
		.def = 127,
	},
	{
		.id = V4L2_CID_ROTATE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 270,
		.step = 90,
		.def = 0,
	},
	{
		.id = V4L2_CID_MPEG_VIDEO_REPEAT_SEQ_HEADER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
	},
};

static int vsi_setup_enc_ctrls(struct v4l2_ctrl_handler *handler)
{
	int i, ctrl_num = ARRAY_SIZE(vsi_v4l2_encctrl_defs);
	struct v4l2_ctrl *ctrl = NULL;

	v4l2_ctrl_handler_init(handler, ctrl_num);

	if (handler->error)
		return handler->error;

	for (i = 0; i < ctrl_num; i++) {
		vsi_v4l2_update_ctrlcfg(&vsi_v4l2_encctrl_defs[i]);
		if (is_vsi_ctrl(vsi_v4l2_encctrl_defs[i].id))
			ctrl = v4l2_ctrl_new_custom(handler, &vsi_v4l2_encctrl_defs[i], NULL);
		else {
			if (vsi_v4l2_encctrl_defs[i].type == V4L2_CTRL_TYPE_MENU) {
				ctrl = v4l2_ctrl_new_std_menu(handler, &vsi_encctrl_ops,
					vsi_v4l2_encctrl_defs[i].id,
					vsi_v4l2_encctrl_defs[i].max,
					0,
					vsi_v4l2_encctrl_defs[i].def);
			} else {
				ctrl = v4l2_ctrl_new_std(handler,
					&vsi_encctrl_ops,
					vsi_v4l2_encctrl_defs[i].id,
					vsi_v4l2_encctrl_defs[i].min,
					vsi_v4l2_encctrl_defs[i].max,
					vsi_v4l2_encctrl_defs[i].step,
					vsi_v4l2_encctrl_defs[i].def);
			}
		}
		if (ctrl && (vsi_v4l2_encctrl_defs[i].flags & V4L2_CTRL_FLAG_VOLATILE))
			ctrl->flags |= V4L2_CTRL_FLAG_VOLATILE;

		if (handler->error) {
			v4l2_klog(LOGLVL_ERROR, "fail to set ctrl %d:%d", i, handler->error);
			break;
		}
	}

	v4l2_ctrl_handler_setup(handler);
	return handler->error;
}

static int v4l2_enc_open(struct file *filp)
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
	ctx->flag = CTX_FLAG_ENC;
	set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);
	set_bit(CTX_FLAG_ENC_FLUSHBUF, &ctx->flag);

	ctx->frameidx = 0;
	q = &ctx->input_que;
	q->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->min_buffers_needed = MIN_FRAME_4ENC;
	q->drv_priv = &ctx->fh;
	q->lock = &ctx->ctxlock;
	q->buf_struct_size = sizeof(struct vsi_vpu_buf);		//used to alloc mem control structures in reqbuf
	q->ops = &vsi_enc_qops;		/*it might be used to identify input and output */
	q->mem_ops = &vb2_dma_contig_memops;
	q->memory = VB2_MEMORY_UNKNOWN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	INIT_LIST_HEAD(&ctx->input_list);
	ret = vb2_queue_init(q);
	/*q->buf_ops = &v4l2_buf_ops is set here*/
	if (ret)
		goto err_enc_dec_exit;

	q = &ctx->output_que;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->min_buffers_needed = 1;
	q->drv_priv = &ctx->fh;
	q->lock = &ctx->ctxlock;
	q->buf_struct_size = sizeof(struct vsi_vpu_buf);
	q->ops = &vsi_enc_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->memory = VB2_MEMORY_UNKNOWN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	INIT_LIST_HEAD(&ctx->output_list);
	ret = vb2_queue_init(q);
	if (ret) {
		vb2_queue_release(&ctx->input_que);
		goto err_enc_dec_exit;
	}
	vsiv4l2_initcfg(ctx);
	vsi_setup_enc_ctrls(&ctx->ctrlhdl);
	vfh = (struct v4l2_fh *)filp->private_data;
	vfh->ctrl_handler = &ctx->ctrlhdl;
	atomic_set(&ctx->srcframen, 0);
	atomic_set(&ctx->dstframen, 0);
	ctx->status = VSI_STATUS_INIT;

	return 0;

err_enc_dec_exit:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	vsi_remove_ctx(ctx);
	kfree(ctx);
	vsi_v4l2_quitinstance();
	return ret;
}

static int v4l2_enc_mmap(struct file *filp, struct vm_area_struct *vma)
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

static __poll_t vsi_enc_poll(struct file *file, poll_table *wait)
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
		ret |= POLLERR;

	if (v4l2_event_pending(&ctx->fh)) {
		v4l2_klog(LOGLVL_BRIEF, "%s event", __func__);
		ret |= POLLPRI;
	}
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
		ret |= POLLERR;

	v4l2_klog(LOGLVL_VERBOSE, "%s %x", __func__, ret);
	return ret;
}

static const struct v4l2_file_operations v4l2_enc_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_enc_open,
	.release = vsi_v4l2_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_enc_mmap,
	.poll = vsi_enc_poll,
};

struct video_device *vsi_v4l2_probe_enc(struct platform_device *pdev, struct vsi_v4l2_device *vpu)
{
	struct video_device *venc;
	int ret = 0;

	v4l2_klog(LOGLVL_BRIEF, "%s", __func__);

	/*init video device0, encoder */
	venc = video_device_alloc();
	if (!venc) {
		v4l2_err(&vpu->v4l2_dev, "Failed to allocate enc device\n");
		ret = -ENOMEM;
		goto err;
	}

	venc->fops = &v4l2_enc_fops;
	venc->ioctl_ops = &vsi_enc_ioctl;
	venc->device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;
	venc->release = video_device_release;
	venc->lock = &vpu->lock;
	venc->v4l2_dev = &vpu->v4l2_dev;
	venc->vfl_dir = VFL_DIR_M2M;
	venc->vfl_type = VSI_DEVTYPE;
	venc->queue = NULL;

	video_set_drvdata(venc, vpu);

	ret = video_register_device(venc, VSI_DEVTYPE, 0);
	if (ret) {
		v4l2_err(&vpu->v4l2_dev, "Failed to register enc device\n");
		video_device_release(venc);
		goto err;
	}

	return venc;
err:
	return NULL;
}

void vsi_v4l2_release_enc(struct video_device *venc)
{
	video_unregister_device(venc);
}

