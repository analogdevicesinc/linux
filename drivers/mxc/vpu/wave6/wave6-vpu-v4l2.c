// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - v4l2 interface
 *
 * Copyright (C) 2022 CHIPS&MEDIA INC
 */

#include "wave6-vpu.h"
#include "wave6-vpu-dbg.h"
#include <linux/math64.h>

void wave6_update_pix_fmt(struct v4l2_pix_format_mplane *pix_mp,
			  unsigned int width,
			  unsigned int height)
{
	const struct v4l2_format_info *fmt_info;
	unsigned int stride_y;
	int i;

	pix_mp->width = width;
	pix_mp->height = height;
	pix_mp->flags = 0;
	pix_mp->field = V4L2_FIELD_NONE;
	memset(pix_mp->reserved, 0, sizeof(pix_mp->reserved));

	fmt_info = v4l2_format_info(pix_mp->pixelformat);
	if (!fmt_info) {
		pix_mp->plane_fmt[0].bytesperline = 0;
		if (!pix_mp->plane_fmt[0].sizeimage)
			pix_mp->plane_fmt[0].sizeimage = width * height;

		return;
	}

	stride_y = width * fmt_info->bpp[0];
	if (pix_mp->plane_fmt[0].bytesperline <= W6_MAX_PIC_STRIDE)
		stride_y = max(stride_y, pix_mp->plane_fmt[0].bytesperline);
	stride_y = round_up(stride_y, 32);
	pix_mp->plane_fmt[0].bytesperline = stride_y;
	pix_mp->plane_fmt[0].sizeimage = stride_y * height;

	for (i = 1; i < fmt_info->comp_planes; i++) {
		unsigned int stride_c, sizeimage_c;

		stride_c = DIV_ROUND_UP(stride_y, fmt_info->hdiv) *
			   fmt_info->bpp[i];
		sizeimage_c = stride_c * DIV_ROUND_UP(height, fmt_info->vdiv);

		if (fmt_info->mem_planes == 1) {
			pix_mp->plane_fmt[0].sizeimage += sizeimage_c;
		} else {
			pix_mp->plane_fmt[i].bytesperline = stride_c;
			pix_mp->plane_fmt[i].sizeimage = sizeimage_c;
		}
	}
}

struct vb2_v4l2_buffer *wave6_get_dst_buf_by_addr(struct vpu_instance *inst,
						  dma_addr_t addr)
{
	struct vb2_v4l2_buffer *vb2_v4l2_buf;
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vb2_v4l2_buffer *dst_buf = NULL;

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vb2_v4l2_buf = &v4l2_m2m_buf->vb;
		if (addr == vb2_dma_contig_plane_dma_addr(&vb2_v4l2_buf->vb2_buf, 0)) {
			dst_buf = vb2_v4l2_buf;
			break;
		}
	}

	return dst_buf;
}

dma_addr_t wave6_get_dma_addr(struct vb2_v4l2_buffer *buf, unsigned int plane_no)
{
	return vb2_dma_contig_plane_dma_addr(&buf->vb2_buf, plane_no) +
			buf->planes[plane_no].data_offset;
}

int wave6_vpu_wait_interrupt(struct vpu_instance *inst, unsigned int timeout)
{
	int ret;

	ret = wait_for_completion_timeout(&inst->dev->irq_done,
					  msecs_to_jiffies(timeout));
	if (!ret)
		return -ETIMEDOUT;

	reinit_completion(&inst->dev->irq_done);

	return 0;
}

int wave6_vpu_subscribe_event(struct v4l2_fh *fh,
			      const struct v4l2_event_subscription *sub)
{
	struct vpu_instance *inst = wave6_to_vpu_inst(fh);
	bool is_decoder = (inst->type == VPU_INST_TYPE_DEC) ? true : false;

	dev_dbg(inst->dev->dev, "%s: [%s] type: %d id: %d | flags: %d\n",
		__func__, is_decoder ? "decoder" : "encoder", sub->type,
		sub->id, sub->flags);

	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		if (is_decoder)
			return v4l2_src_change_event_subscribe(fh, sub);
		return -EINVAL;
	case V4L2_EVENT_CTRL:
		return v4l2_ctrl_subscribe_event(fh, sub);
	default:
		return -EINVAL;
	}
}

void wave6_vpu_return_buffers(struct vpu_instance *inst,
			      unsigned int type, enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *buf;
	int i;

	if (V4L2_TYPE_IS_OUTPUT(type)) {
		while ((buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx)))
			v4l2_m2m_buf_done(buf, state);
	} else {
		while ((buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx))) {
			for (i = 0; i < inst->dst_fmt.num_planes; i++)
				vb2_set_plane_payload(&buf->vb2_buf, i, 0);
			v4l2_m2m_buf_done(buf, state);
		}
	}
}

static bool wave6_vpu_check_fb_available(struct vpu_instance *inst)
{
	struct vb2_v4l2_buffer *vb2_v4l2_buf;
	struct v4l2_m2m_buffer *v4l2_m2m_buf;
	struct vpu_buffer *vpu_buf;

	v4l2_m2m_for_each_dst_buf(inst->v4l2_fh.m2m_ctx, v4l2_m2m_buf) {
		vb2_v4l2_buf = &v4l2_m2m_buf->vb;
		vpu_buf = wave6_to_vpu_buf(vb2_v4l2_buf);

		if (!vpu_buf->used)
			return true;
	}

	return false;
}

static int wave6_vpu_job_ready(void *priv)
{
	struct vpu_instance *inst = priv;

	dev_dbg(inst->dev->dev, "[%d]%s: state %d\n",
		inst->id, __func__, inst->state);

	if (inst->state < VPU_INST_STATE_PIC_RUN)
		return 0;
	if (inst->state == VPU_INST_STATE_STOP && inst->eos)
		return 0;
	if (!wave6_vpu_check_fb_available(inst))
		return 0;

	return 1;
}

static void wave6_vpu_device_run_timeout(struct work_struct *work)
{
	struct delayed_work *dwork = to_delayed_work(work);
	struct vpu_device *dev = container_of(dwork, struct vpu_device, task_timer);
	struct vpu_instance *inst = v4l2_m2m_get_curr_priv(dev->m2m_dev);
	struct vb2_v4l2_buffer *src_buf = NULL;
	struct vb2_v4l2_buffer *dst_buf = NULL;

	if (!inst)
		return;

	dev_err(inst->dev->dev, "[%d] sequence %d timeout\n", inst->id, inst->sequence);
	src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (src_buf) {
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);
		if (inst->type == VPU_INST_TYPE_DEC)
			inst->processed_buf_num++;
		inst->error_buf_num++;
	}

	dst_buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (dst_buf)
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

	vb2_queue_error(v4l2_m2m_get_src_vq(inst->v4l2_fh.m2m_ctx));
	vb2_queue_error(v4l2_m2m_get_dst_vq(inst->v4l2_fh.m2m_ctx));

	v4l2_m2m_job_finish(inst->dev->m2m_dev, inst->v4l2_fh.m2m_ctx);
}

static void wave6_vpu_device_run(void *priv)
{
	struct vpu_instance *inst = priv;
	int ret;

	dev_dbg(inst->dev->dev, "[%d]%s: state %d\n",
		inst->id, __func__, inst->state);

	ret = inst->ops->start_process(inst);
	if (!ret)
		schedule_delayed_work(&inst->dev->task_timer, msecs_to_jiffies(W6_VPU_TIMEOUT));
	else
		v4l2_m2m_job_finish(inst->dev->m2m_dev, inst->v4l2_fh.m2m_ctx);
}

void wave6_vpu_finish_job(struct vpu_instance *inst)
{
	cancel_delayed_work(&inst->dev->task_timer);
	v4l2_m2m_job_finish(inst->dev->m2m_dev, inst->v4l2_fh.m2m_ctx);
}

void wave6_vpu_handle_performance(struct vpu_instance *inst, struct vpu_buffer *vpu_buf)
{
	s64 latency, time_spent;

	if (!inst || !vpu_buf)
		return;

	if (!inst->performance.ts_first)
		inst->performance.ts_first = vpu_buf->ts_input;
	inst->performance.ts_last = vpu_buf->ts_output;

	latency = vpu_buf->ts_output - vpu_buf->ts_input;
	time_spent = vpu_buf->ts_finish - vpu_buf->ts_start;

	if (!inst->performance.latency_first)
		inst->performance.latency_first = latency;
	inst->performance.latency_max = max_t(s64, latency, inst->performance.latency_max);

	if (!inst->performance.min_process_time)
		inst->performance.min_process_time = time_spent;
	else if (inst->performance.min_process_time > time_spent)
		inst->performance.min_process_time = time_spent;

	if (inst->performance.max_process_time < time_spent)
		inst->performance.max_process_time = time_spent;

	inst->performance.total_sw_time += time_spent;
	inst->performance.total_hw_time += vpu_buf->hw_time;
}

void wave6_vpu_reset_performance(struct vpu_instance *inst)
{
	if (!inst)
		return;

	if (inst->processed_buf_num) {
		s64 tmp;
		s64 fps_act, fps_sw, fps_hw;
		struct vpu_performance_info *perf = &inst->performance;

		tmp = MSEC_PER_SEC * inst->processed_buf_num;
		fps_act = DIV_ROUND_CLOSEST(tmp, (perf->ts_last - perf->ts_first) / NSEC_PER_MSEC);
		fps_sw = DIV_ROUND_CLOSEST(tmp, perf->total_sw_time / NSEC_PER_MSEC);
		fps_hw = DIV_ROUND_CLOSEST(tmp, perf->total_hw_time / NSEC_PER_MSEC);
		dprintk(inst->dev->dev,
			"[%d] fps actual: %lld, sw: %lld, hw: %lld, latency(ms) %llu.%06llu\n",
			inst->id, fps_act, fps_sw, fps_hw,
			perf->latency_first / NSEC_PER_MSEC,
			perf->latency_first % NSEC_PER_MSEC);
	}

	memset(&inst->performance, 0, sizeof(inst->performance));
}

static const struct v4l2_m2m_ops wave6_vpu_m2m_ops = {
	.device_run = wave6_vpu_device_run,
	.job_ready = wave6_vpu_job_ready,
};

int wave6_vpu_init_m2m_dev(struct vpu_device *dev)
{
	dev->m2m_dev = v4l2_m2m_init(&wave6_vpu_m2m_ops);
	if (IS_ERR(dev->m2m_dev)) {
		dev_err(dev->dev, "v4l2_m2m_init fail: %ld\n", PTR_ERR(dev->m2m_dev));
		return PTR_ERR(dev->m2m_dev);
	}

	INIT_DELAYED_WORK(&dev->task_timer, wave6_vpu_device_run_timeout);

	return 0;
}

void wave6_vpu_release_m2m_dev(struct vpu_device *dev)
{
	v4l2_m2m_release(dev->m2m_dev);
}
