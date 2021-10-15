/*
 *    VSI V4L2 kernel driver main entrance.
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

#define DRIVER_NAME	"vsiv4l2"

int vsi_kloglvl = LOGLVL_ERROR;
module_param(vsi_kloglvl, int, 0644);

static struct platform_device *gvsidev;
static struct idr vsi_inst_array;
static struct device *vsidaemondev;
static struct mutex vsi_ctx_array_lock;		//it only protect ctx between release from app and msg from daemon
static ulong ctx_seqid;

static ssize_t BandWidth_show(struct device *kdev,
				     struct device_attribute *attr, char *buf)
{
	/*
	 * sys/bus/platform/drivers/vsiv4l2/xxxxx.vpu/BandWidth
	 * used to show bandwidth info to user space
	 */
	u64 bandwidth;

	bandwidth = vsi_v4l2_getbandwidth();
	return snprintf(buf, PAGE_SIZE, "%lld\n", bandwidth);
}

static DEVICE_ATTR_RO(BandWidth);

static struct attribute *vsi_v4l2_attrs[] = {
	&dev_attr_BandWidth.attr,
	NULL,
};

static const struct attribute_group vsi_v4l2_attr_group = {
	.attrs = vsi_v4l2_attrs,
};

static void release_ctx(struct vsi_v4l2_ctx *ctx, int notifydaemon)
{
	int ret = 0;

	if (notifydaemon == 1 && test_bit(CTX_FLAG_DAEMONLIVE_BIT, &ctx->flag)) {
		if (isdecoder(ctx))
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_DESTROY_DEC, NULL);
		else
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_DESTROY_ENC, NULL);
	}
	/*vsi_vpu_buf obj is freed here, together with all buffer memory */
	return_all_buffers(&ctx->input_que, VB2_BUF_STATE_DONE, 0);
	return_all_buffers(&ctx->output_que, VB2_BUF_STATE_DONE, 0);
	removeallcropinfo(ctx);
	if (mutex_lock_interruptible(&vsi_ctx_array_lock))
		return;
	idr_remove(&vsi_inst_array, CTX_ARRAY_ID(ctx->ctxid));
	mutex_unlock(&vsi_ctx_array_lock);
	vb2_queue_release(&ctx->input_que);
	vb2_queue_release(&ctx->output_que);
	v4l2_ctrl_handler_free(&ctx->ctrlhdl);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	ctx = NULL;
}

void vsi_remove_ctx(struct vsi_v4l2_ctx *ctx)
{
	if (mutex_lock_interruptible(&vsi_ctx_array_lock))
		return;
	idr_remove(&vsi_inst_array, CTX_ARRAY_ID(ctx->ctxid));
	mutex_unlock(&vsi_ctx_array_lock);
}

struct vsi_v4l2_ctx *vsi_create_ctx(void)
{
	struct vsi_v4l2_ctx *ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);

	if (!ctx)
		return NULL;
	if (mutex_lock_interruptible(&vsi_ctx_array_lock)) {
		kfree(ctx);
		return NULL;
	}
	ctx->ctxid = idr_alloc(&vsi_inst_array, (void *)ctx, 1, 0, GFP_KERNEL);
	if ((int)ctx->ctxid < 0) {
		kfree(ctx);
		ctx = NULL;
	} else {
		ctx_seqid++;
		if (ctx_seqid >= CTX_SEQID_UPLIMT)
			ctx_seqid = 1;
		ctx->ctxid |= (ctx_seqid << 32);
		v4l2_klog(LOGLVL_BRIEF, "create ctx with %lx", ctx->ctxid);
	}
	mutex_unlock(&vsi_ctx_array_lock);
	init_waitqueue_head(&ctx->retbuf_queue);
	init_waitqueue_head(&ctx->capoffdone_queue);

	return ctx;
}

void vsi_set_ctx_error(struct vsi_v4l2_ctx *ctx, s32 error)
{
	ctx->error = error;
	if (error < 0) {
		struct v4l2_event event;

		memset(&event, 0, sizeof(struct v4l2_event));
		event.type = V4L2_EVENT_CODEC_ERROR,
		v4l2_event_queue_fh(&ctx->fh, &event);
	}
}
void wakeup_ctxqueues(void)
{
	struct vsi_v4l2_ctx *ctx;
	int id;

	idr_for_each_entry(&vsi_inst_array, ctx, id) {
		if (ctx) {
			vsi_set_ctx_error(ctx, DAEMON_ERR_DAEMON_MISSING);
			wake_up_interruptible_all(&ctx->input_que.done_wq);
			wake_up_interruptible_all(&ctx->output_que.done_wq);
			wake_up_interruptible_all(&ctx->retbuf_queue);
			wake_up_interruptible_all(&ctx->capoffdone_queue);
			wake_up_interruptible_all(&ctx->fh.wait);
		}
	}
}

static struct vsi_v4l2_ctx *find_ctx(unsigned long ctxid)
{
	unsigned long id = CTX_ARRAY_ID(ctxid);
	unsigned long seq = CTX_SEQ_ID(ctxid);
	struct vsi_v4l2_ctx *ctx;

	if (mutex_lock_interruptible(&vsi_ctx_array_lock))
		return NULL;
	ctx  = (struct vsi_v4l2_ctx *)idr_find(&vsi_inst_array, id);
	mutex_unlock(&vsi_ctx_array_lock);
	if (ctx && (CTX_SEQ_ID(ctx->ctxid)  == seq))
		return ctx;
	else
		return NULL;
}

static void vsi_v4l2_clear_event(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_event event;
	int ret;

	if (v4l2_event_pending(&ctx->fh)) {
		while (v4l2_event_pending(&ctx->fh)) {
			ret = v4l2_event_dequeue(&ctx->fh, &event, 1);
			if (ret)
				return;
		};
	}
}

int vsi_v4l2_reset_ctx(struct vsi_v4l2_ctx *ctx)
{
	int ret = 0;

	if (ctx->status != VSI_STATUS_INIT) {
		v4l2_klog(LOGLVL_BRIEF, "reset ctx %lx", ctx->ctxid);
		ctx->queued_srcnum = ctx->buffed_capnum = ctx->buffed_cropcapnum = 0;
		vsi_v4l2_clear_event(ctx);
		if (isdecoder(ctx)) {
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_DESTROY_DEC, NULL);
			ctx->flag = CTX_FLAG_DEC;
		} else {
			ret = vsiv4l2_execcmd(ctx, V4L2_DAEMON_VIDIOC_DESTROY_ENC, NULL);
			ctx->flag = CTX_FLAG_ENC;
			set_bit(CTX_FLAG_ENC_FLUSHBUF, &ctx->flag);
		}
		set_bit(CTX_FLAG_CONFIGUPDATE_BIT, &ctx->flag);
		return_all_buffers(&ctx->input_que, VB2_BUF_STATE_DONE, 0);
		return_all_buffers(&ctx->output_que, VB2_BUF_STATE_DONE, 0);
		removeallcropinfo(ctx);
		ctx->status = VSI_STATUS_INIT;
		vsi_set_ctx_error(ctx, 0);
		if (isdecoder(ctx)) {
			wake_up_interruptible_all(&ctx->retbuf_queue);
			wake_up_interruptible_all(&ctx->capoffdone_queue);
		}
	}
	return ret;
}

int vsi_v4l2_release(struct file *filp)
{
	struct vsi_v4l2_ctx *ctx = fh_to_ctx(filp->private_data);

	/*normal streaming end should fall here*/
	v4l2_klog(LOGLVL_BRIEF, "%s ctx %lx", __func__, ctx->ctxid);
	vsi_clear_daemonmsg(CTX_ARRAY_ID(ctx->ctxid));
	release_ctx(ctx, 1);
	vsi_v4l2_quitinstance();
	return 0;
}

/*orphan error msg from daemon write, should not call daemon back*/
int vsi_v4l2_handle_picconsumed(struct vsi_v4l2_msg *pmsg)
{
	unsigned long ctxid = pmsg->inst_id;
	struct vsi_v4l2_ctx *ctx;
	struct v4l2_event event;

	v4l2_klog(LOGLVL_WARNING, "%lx got picconsumed event", ctxid);
	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -1;

	memset((void *)&event, 0, sizeof(struct v4l2_event));
	event.type = V4L2_EVENT_SKIP;
	if (isdecoder(ctx))
		event.u.data[0] = pmsg->params.dec_params.io_buffer.inbufidx;

	v4l2_event_queue_fh(&ctx->fh, &event);
	return 0;
}

void vsi_v4l2_sendeos(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_event event;

	memset((void *)&event, 0, sizeof(struct v4l2_event));
	event.type = V4L2_EVENT_EOS;
	v4l2_event_queue_fh(&ctx->fh, &event);
}

int vsi_v4l2_handleerror(unsigned long ctxid, int error)
{
	struct vsi_v4l2_ctx *ctx;

	v4l2_klog(LOGLVL_ERROR, "%lx got error %d", ctxid, error);
	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -1;

	if (error == DAEMON_ERR_DEC_METADATA_ONLY)
		vsi_v4l2_sendeos(ctx);
	else {
		vsi_set_ctx_error(ctx, error > 0 ? -error:error);
		wake_up_interruptible_all(&ctx->retbuf_queue);
		wake_up_interruptible_all(&ctx->capoffdone_queue);
		wake_up_interruptible_all(&ctx->input_que.done_wq);
		wake_up_interruptible_all(&ctx->output_que.done_wq);
		wake_up_interruptible_all(&ctx->fh.wait);
	}
	return 0;
}

int vsi_v4l2_send_reschange(struct vsi_v4l2_ctx *ctx)
{
	struct v4l2_event event;

	if (ctx->need_capture_on)
		vsi_dec_capture_on(ctx);

	if (!ctx->reschanged_need_notify)
		return 0;

	vsi_v4l2_update_decfmt(ctx);

	memset((void *)&event, 0, sizeof(struct v4l2_event));
	event.type = V4L2_EVENT_SOURCE_CHANGE,
	event.u.src_change.changes = V4L2_EVENT_SRC_CH_RESOLUTION,
	v4l2_event_queue_fh(&ctx->fh, &event);
	ctx->reschanged_need_notify = false;
	return 0;
}

int vsi_v4l2_notify_reschange(struct vsi_v4l2_msg *pmsg)
{
	unsigned long ctxid = pmsg->inst_id;
	struct vsi_v4l2_ctx *ctx;

	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -ESRCH;

	if (isdecoder(ctx)) {
		struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
		struct v4l2_daemon_dec_info *decinfo = &pmsg->params.dec_params.dec_info.dec_info;

		if (mutex_lock_interruptible(&ctx->ctxlock))
			return -EBUSY;
		v4l2_klog(LOGLVL_BRIEF, "%lx sending event res change:%d, delay=%d", ctx->ctxid, ctx->status,
			(ctx->status == DEC_STATUS_DECODING || ctx->status == DEC_STATUS_DRAINING) && !list_empty(&ctx->output_que.done_list));
		v4l2_klog(LOGLVL_BRIEF, "reso=%d:%d,bitdepth=%d,stride=%d,dpb=%d:%d,orig yuvfmt=%d",
			decinfo->frame_width, decinfo->frame_height, decinfo->bit_depth, pmsg->params.dec_params.io_buffer.output_wstride,
			decinfo->needed_dpb_nums, decinfo->dpb_buffer_size, decinfo->src_pix_fmt);
		ctx->reschange_cnt++;
		pcfg->decparams_bkup.dec_info = pmsg->params.dec_params.dec_info;
		pcfg->decparams_bkup.io_buffer.srcwidth = pmsg->params.dec_params.io_buffer.srcwidth;
		pcfg->decparams_bkup.io_buffer.srcheight = pmsg->params.dec_params.io_buffer.srcheight;
		pcfg->decparams_bkup.io_buffer.output_width = pmsg->params.dec_params.io_buffer.output_width;
		pcfg->decparams_bkup.io_buffer.output_height = pmsg->params.dec_params.io_buffer.output_height;
		pcfg->decparams_bkup.io_buffer.output_wstride = pmsg->params.dec_params.io_buffer.output_wstride;
		pcfg->minbuf_4output_bkup = pmsg->params.dec_params.dec_info.dec_info.needed_dpb_nums;
		pcfg->sizeimagedst_bkup = pmsg->params.dec_params.io_buffer.OutBufSize;
		if ((ctx->status == DEC_STATUS_DECODING || ctx->status == DEC_STATUS_DRAINING)
			&& !list_empty(&ctx->output_que.done_list)) {
			set_bit(CTX_FLAG_DELAY_SRCCHANGED_BIT, &ctx->flag);
		} else {
			vsi_dec_update_reso(ctx);
			vsi_v4l2_send_reschange(ctx);
		}
		if (pmsg->params.dec_params.dec_info.dec_info.colour_description_present_flag)
			vsi_dec_updatevui(&pmsg->params.dec_params.dec_info.dec_info, &pcfg->decparams.dec_info.dec_info);
		set_bit(CTX_FLAG_SRCCHANGED_BIT, &ctx->flag);
		mutex_unlock(&ctx->ctxlock);
	}
	return 0;
}

static int convert_daemonwarning_to_appwarning(int daemon_warnmsg)
{
	switch (daemon_warnmsg) {
	case WARN_ROIREGION:
		return RIOREGION_NOTALLOW;
	case WARN_IPCMREGION:
		return IPCMREGION_NOTALLOW;
	case WARN_LEVEL:
		return LEVEL_UPDATED;
	default:
		return UNKONW_WARNING;
	}
}

int vsi_v4l2_handle_warningmsg(struct vsi_v4l2_msg *pmsg)
{
	unsigned long ctxid = pmsg->inst_id;
	struct vsi_v4l2_ctx *ctx;
	struct v4l2_event event;

	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -ESRCH;
	memset((void *)&event, 0, sizeof(struct v4l2_event));
	event.type = V4L2_EVENT_INVALID_OPTION,
	event.id = convert_daemonwarning_to_appwarning(pmsg->error);
	v4l2_klog(LOGLVL_WARNING, "%lx got warning msg %d", ctxid, pmsg->error);
	v4l2_event_queue_fh(&ctx->fh, &event);
	return 0;
}

int vsi_v4l2_handle_streamoffdone(struct vsi_v4l2_msg *pmsg)
{
	unsigned long ctxid = pmsg->inst_id;
	struct vsi_v4l2_ctx *ctx;

	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -ESRCH;
	if (pmsg->cmd_id == V4L2_DAEMON_VIDIOC_STREAMOFF_CAPTURE_DONE)
		set_bit(CTX_FLAG_CAPTUREOFFDONE, &ctx->flag);
	else
		set_bit(CTX_FLAG_OUTPUTOFFDONE, &ctx->flag);
	wake_up_interruptible_all(&ctx->capoffdone_queue);
	v4l2_klog(LOGLVL_FLOW, "%lx got cap streamoff done", ctxid);
	return 0;
}

int vsi_v4l2_handle_cropchange(struct vsi_v4l2_msg *pmsg)
{
	unsigned long ctxid = pmsg->inst_id;
	struct vsi_v4l2_ctx *ctx;

	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -ESRCH;

	if (isdecoder(ctx)) {
		struct vsi_v4l2_mediacfg *pcfg = &ctx->mediacfg;
		struct v4l2_event event;

		if (mutex_lock_interruptible(&ctx->ctxlock))
			return -EBUSY;
		v4l2_klog(LOGLVL_BRIEF, "%lx sending crop change:%d:%d:%d", ctx->ctxid, ctx->status, ctx->buffed_cropcapnum, ctx->lastcapbuffer_idx);
		v4l2_klog(LOGLVL_BRIEF, "crop info:%d:%d:%d:%d:%d:%d:%d",
			pmsg->params.dec_params.pic_info.pic_info.width,
			pmsg->params.dec_params.pic_info.pic_info.height,
			pmsg->params.dec_params.pic_info.pic_info.pic_wstride,
			pmsg->params.dec_params.pic_info.pic_info.crop_left,
			pmsg->params.dec_params.pic_info.pic_info.crop_top,
			pmsg->params.dec_params.pic_info.pic_info.crop_width,
			pmsg->params.dec_params.pic_info.pic_info.crop_height);
		if ((ctx->status == DEC_STATUS_DECODING || ctx->status == DEC_STATUS_DRAINING)
			&& ctx->buffed_cropcapnum > 0) {
			if (addcropmsg(ctx, pmsg) != 0) {
				vsi_set_ctx_error(ctx, DAEMON_ERR_NO_MEM);
				v4l2_klog(LOGLVL_ERROR, "driver out of mem");
			} else
				set_bit(BUF_FLAG_CROPCHANGE, &ctx->vbufflag[ctx->lastcapbuffer_idx]);
		} else {
			pcfg->decparams.dec_info.io_buffer.output_width = pmsg->params.dec_params.pic_info.pic_info.width;
			pcfg->decparams.dec_info.io_buffer.output_height = pmsg->params.dec_params.pic_info.pic_info.height;
			pcfg->decparams.dec_info.io_buffer.output_wstride = pmsg->params.dec_params.pic_info.pic_info.pic_wstride;
			pcfg->decparams.dec_info.dec_info.frame_width = pmsg->params.dec_params.pic_info.pic_info.width;
			pcfg->bytesperline = pmsg->params.dec_params.pic_info.pic_info.pic_wstride;
			pcfg->decparams.dec_info.dec_info.frame_height = pmsg->params.dec_params.pic_info.pic_info.height;
			pcfg->decparams.dec_info.dec_info.visible_rect.left = pmsg->params.dec_params.pic_info.pic_info.crop_left;
			pcfg->decparams.dec_info.dec_info.visible_rect.top = pmsg->params.dec_params.pic_info.pic_info.crop_top;
			pcfg->decparams.dec_info.dec_info.visible_rect.width = pmsg->params.dec_params.pic_info.pic_info.crop_width;
			pcfg->decparams.dec_info.dec_info.visible_rect.height = pmsg->params.dec_params.pic_info.pic_info.crop_height;
			memset((void *)&event, 0, sizeof(struct v4l2_event));
			event.type = V4L2_EVENT_CROPCHANGE,
			v4l2_event_queue_fh(&ctx->fh, &event);
		}
		mutex_unlock(&ctx->ctxlock);
	}
	return 0;
}

int vsi_v4l2_bufferdone(struct vsi_v4l2_msg *pmsg)
{
	unsigned long ctxid = pmsg->inst_id;
	int inbufidx, outbufidx, bytesused[4] = {0};
	struct vsi_v4l2_ctx *ctx;
	struct vb2_queue *vq = NULL;
	struct vb2_buffer	*vb;

	ctx = find_ctx(ctxid);
	if (ctx == NULL)
		return -1;

	if (isencoder(ctx)) {
		inbufidx = pmsg->params.enc_params.io_buffer.inbufidx;
		outbufidx = pmsg->params.enc_params.io_buffer.outbufidx;
		bytesused[0] = pmsg->params.enc_params.io_buffer.bytesused;
	} else {
		inbufidx = pmsg->params.dec_params.io_buffer.inbufidx;
		outbufidx = pmsg->params.dec_params.io_buffer.outbufidx;
		bytesused[0] = pmsg->params.dec_params.io_buffer.bytesused;
	}
	v4l2_klog(LOGLVL_FLOW, "%lx:%s:%lx:%d:%d",
		ctx->ctxid, __func__, ctx->flag, inbufidx, outbufidx);
	//write comes over once, so avoid this problem.
	if (inbufidx >= 0 && inbufidx < ctx->input_que.num_buffers) {
		vq = &ctx->input_que;
		vb = vq->bufs[inbufidx];
		if (mutex_lock_interruptible(&ctx->ctxlock))
			return -1;
		atomic_inc(&ctx->srcframen);
		if (ctx->input_que.streaming && vb->state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
		if (isdecoder(ctx)) {
			ctx->queued_srcnum--;
			if (!test_bit(BUF_FLAG_QUEUED, &ctx->srcvbufflag[inbufidx])) {
				v4l2_klog(LOGLVL_WARNING, "got unqueued srcbuf %d", inbufidx);
			} else {
				clear_bit(BUF_FLAG_QUEUED, &ctx->srcvbufflag[inbufidx]);
				set_bit(BUF_FLAG_DONE, &ctx->srcvbufflag[inbufidx]);
			}
		}
		mutex_unlock(&ctx->ctxlock);
	}
	if (outbufidx >= 0 && outbufidx < ctx->output_que.num_buffers) {
		if (mutex_lock_interruptible(&ctx->ctxlock))
			return -EBUSY;
		if (!inst_isactive(ctx)) {
			if (!vb2_is_streaming(&ctx->output_que))
				v4l2_klog(LOGLVL_ERROR, "%lx ignore dst buffer %d in state %d", ctx->ctxid, outbufidx, ctx->status);
			mutex_unlock(&ctx->ctxlock);
			return 0;
		}
		if (bytesused[0] > 0)
			ctx->frameidx++;
		vq = &ctx->output_que;
		vb = vq->bufs[outbufidx];
		atomic_inc(&ctx->dstframen);
		if (vb->state == VB2_BUF_STATE_ACTIVE) {
			vb->planes[0].bytesused = bytesused[0];
			if (isencoder(ctx)) {
				vb->timestamp = pmsg->params.enc_params.io_buffer.timestamp;
				ctx->vbufflag[outbufidx] = pmsg->param_type;
				v4l2_klog(LOGLVL_FLOW,  "enc output framed %d size = %d,flag=%lx, timestamp=%lld",
						outbufidx, vb->planes[0].bytesused, ctx->vbufflag[outbufidx], vb->timestamp);
				if (vb->planes[0].bytesused == 0 || (pmsg->param_type & LAST_BUFFER_FLAG)) {
					ctx->vbufflag[outbufidx] |= LAST_BUFFER_FLAG;
					v4l2_klog(LOGLVL_BRIEF, "%lx encoder got eos buffer", ctx->ctxid);
				}
			} else {
				ctx->lastcapbuffer_idx = outbufidx;
				if (!test_bit(BUF_FLAG_QUEUED, &ctx->vbufflag[outbufidx])) {
					v4l2_klog(LOGLVL_WARNING, "got unqueued dstbuf %d", outbufidx);
				} else {
					clear_bit(BUF_FLAG_QUEUED, &ctx->vbufflag[outbufidx]);
					set_bit(BUF_FLAG_DONE, &ctx->vbufflag[outbufidx]);
				}
				ctx->rfc_luma_offset[outbufidx] = pmsg->params.dec_params.io_buffer.rfc_luma_offset;
				ctx->rfc_chroma_offset[outbufidx] = pmsg->params.dec_params.io_buffer.rfc_chroma_offset;
				if (bytesused[0] == 0) {
					v4l2_klog(LOGLVL_BRIEF, "%lx decoder got zero buffer in state %d", ctx->ctxid, ctx->status);
					if ((ctx->status == DEC_STATUS_DRAINING) || test_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag)) {
						ctx->status = DEC_STATUS_ENDSTREAM;
						set_bit(CTX_FLAG_ENDOFSTRM_BIT, &ctx->flag);
						clear_bit(CTX_FLAG_PRE_DRAINING_BIT, &ctx->flag);
					}
				} else
					vb->timestamp = pmsg->params.dec_params.io_buffer.timestamp;
				ctx->buffed_capnum++;
				ctx->buffed_cropcapnum++;
				v4l2_klog(LOGLVL_FLOW, "dec output framed %d size = %d", outbufidx, vb->planes[0].bytesused);
			}
			if (vb->state == VB2_BUF_STATE_ACTIVE)
				vb2_buffer_done(vb, VB2_BUF_STATE_DONE);
			else
				v4l2_klog(LOGLVL_WARNING, "dstbuf %d is not active", outbufidx);
		}
		mutex_unlock(&ctx->ctxlock);
	}
	if (ctx->queued_srcnum == 0)
		wake_up_interruptible_all(&ctx->retbuf_queue);
	return 0;
}

static void vsi_daemonsdevice_release(struct device *dev)
{
}

static int v4l2_probe(struct platform_device *pdev)
{
	struct vsi_v4l2_device *vpu = NULL;
	struct video_device *venc, *vdec;
	int ret = 0;

	v4l2_klog(LOGLVL_BRIEF, "%s", __func__);
	if (gvsidev != NULL)
		return 0;
	vpu = kzalloc(sizeof(*vpu), GFP_KERNEL);
	if (!vpu)
		return -ENOMEM;

	vpu->dev = &pdev->dev;
	vpu->pdev = pdev;
	mutex_init(&vpu->lock);
	mutex_init(&vpu->irqlock);

	ret = v4l2_device_register(&pdev->dev, &vpu->v4l2_dev);
	if (ret) {
		v4l2_klog(LOGLVL_ERROR, "Failed to register v4l2 device\n");
		kfree(vpu);
		return ret;
	}
	platform_set_drvdata(pdev, vpu);

	vpu->venc = NULL;
	vpu->vdec = NULL;
	venc = vsi_v4l2_probe_enc(pdev, vpu);
	if (venc == NULL)
		goto err;
	vpu->venc = venc;

	vdec = vsi_v4l2_probe_dec(pdev, vpu);
	if (vdec == NULL)
		goto err;
	vpu->vdec = vdec;

	ret = vsiv4l2_initdaemon();
	if (ret < 0)
		goto err;

	vsidaemondev = kzalloc(sizeof(struct device), GFP_KERNEL);
	vsidaemondev->class = class_create(THIS_MODULE, "vsi_class");
	vsidaemondev->parent = NULL;
	vsidaemondev->devt = MKDEV(VSI_DAEMON_DEVMAJOR, 0);
	dev_set_name(vsidaemondev, "%s", VSI_DAEMON_FNAME);
	vsidaemondev->release = vsi_daemonsdevice_release;
	ret = device_register(vsidaemondev);
	if (ret < 0) {
		kfree(vsidaemondev);
		vsidaemondev = NULL;
		vsiv4l2_cleanupdaemon();
		goto err;
	}
	idr_init(&vsi_inst_array);

	gvsidev = pdev;
	mutex_init(&vsi_ctx_array_lock);
	ctx_seqid = 0;
	if (devm_device_add_group(&gvsidev->dev, &vsi_v4l2_attr_group))
		v4l2_klog(LOGLVL_ERROR, "fail to create sysfs API");

	v4l2_klog(LOGLVL_BRIEF, "vpu v4l2: module inserted. Major = %d\n", VSI_DAEMON_DEVMAJOR);
	return 0;

err:
	v4l2_klog(LOGLVL_ERROR, "vsi v4l2 dev probe fail with errno %d", ret);
	if (vpu->venc) {
		vsi_v4l2_release_enc(vpu->venc);
		video_device_release(vpu->venc);
	}
	if (vpu->vdec) {
		vsi_v4l2_release_dec(vpu->vdec);
		video_device_release(vpu->vdec);
	}
	v4l2_device_unregister(&vpu->v4l2_dev);
	kfree(vpu);

	return ret;
}

static int v4l2_remove(struct platform_device *pdev)
{
	void *obj;
	int id;
	struct vsi_v4l2_device *vpu = platform_get_drvdata(pdev);

	vsi_v4l2_release_dec(vpu->vdec);
	vsi_v4l2_release_enc(vpu->venc);
	v4l2_device_unregister(&vpu->v4l2_dev);
	platform_set_drvdata(pdev, NULL);
	kfree(vpu);

	idr_for_each_entry(&vsi_inst_array, obj, id) {
		if (obj) {
			release_ctx(obj, 0);
			vsi_v4l2_quitinstance();
		}
	}

	device_unregister(vsidaemondev);
	class_destroy(vsidaemondev->class);
	kfree(vsidaemondev);
	vsiv4l2_cleanupdaemon();
	gvsidev = NULL;
	return 0;
}

static const struct platform_device_id v4l2_platform_ids[] = {
	{
		.name            = DRIVER_NAME,
	},
	{ },
};

static const struct of_device_id v4l2_of_match[] = {
	{ .compatible = "nxp,imx8m-vsiv4l2", },
	{/* sentinel */}
};

static struct platform_driver v4l2_drm_platform_driver = {
	.probe      = v4l2_probe,
	.remove      = v4l2_remove,
	.driver      = {
		.name      = DRIVER_NAME,
		.owner		= THIS_MODULE,
		.of_match_table = v4l2_of_match,
	},
	.id_table = v4l2_platform_ids,
};

static const struct platform_device_info v4l2_platform_info = {
	.name		= DRIVER_NAME,
	.id		= -1,
	.dma_mask	= DMA_BIT_MASK(64),
};

module_platform_driver(v4l2_drm_platform_driver);

/* module description */
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Verisilicon");
MODULE_DESCRIPTION("VSI v4l2 manager");

