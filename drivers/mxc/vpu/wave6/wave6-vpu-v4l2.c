// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
/*
 * Wave6 series multi-standard codec IP - v4l2 interface
 *
 * Copyright (C) 2022 CHIPS&MEDIA INC
 */

#include "wave6-vpu.h"

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

	src_buf = v4l2_m2m_src_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (src_buf)
		v4l2_m2m_buf_done(src_buf, VB2_BUF_STATE_ERROR);

	dst_buf = v4l2_m2m_dst_buf_remove(inst->v4l2_fh.m2m_ctx);
	if (dst_buf)
		v4l2_m2m_buf_done(dst_buf, VB2_BUF_STATE_ERROR);

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

static void wave6_vpu_job_abort(void *priv)
{
	struct vpu_instance *inst = priv;

	dev_dbg(inst->dev->dev, "[%d]%s: state %d\n",
		inst->id, __func__, inst->state);

	inst->ops->stop_process(inst);
}

static const struct v4l2_m2m_ops wave6_vpu_m2m_ops = {
	.device_run = wave6_vpu_device_run,
	.job_ready = wave6_vpu_job_ready,
	.job_abort = wave6_vpu_job_abort,
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
