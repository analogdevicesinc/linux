// SPDX-License-Identifier: GPL-2.0
/*
 * V4L2 Capture ISI subdev driver for i.MX8QXP/QM platform
 *
 * ISI is a Image Sensor Interface of i.MX8QXP/QM platform, which
 * used to process image from camera sensor to memory or DC
 *
 * Copyright 2019-2021 NXP
 *
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/pm_runtime.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "imx8-isi-hw.h"
#include "imx8-common.h"
#include "imx8-isi-fmt.h"

#define sd_to_cap_dev(ptr)	container_of(ptr, struct mxc_isi_cap_dev, sd)
static int mxc_isi_cap_streamoff(struct file *file, void *priv,
				 enum v4l2_buf_type type);

/*
 * Pixel link input format
 */
struct mxc_isi_fmt mxc_isi_src_formats[] = {
	{
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.depth		= { 32 },
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
	}, {
		.name		= "YUV32 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.memplanes	= 1,
		.colplanes	= 1,
		.align		= 2,
	}
};

/*
 * lookup mxc_isi color format by fourcc or media bus format
 */
static struct mxc_isi_fmt *mxc_isi_find_format(const u32 *pixelformat,
					       const u32 *mbus_code,
					       int index)
{
	struct mxc_isi_fmt *fmt, *def_fmt = NULL;
	unsigned int i;
	int id = 0;

	if (index >= (int)mxc_isi_out_formats_size)
		return NULL;

	for (i = 0; i < mxc_isi_out_formats_size; i++) {
		fmt = &mxc_isi_out_formats[i];
		if (pixelformat && fmt->fourcc == *pixelformat)
			return fmt;
		if (mbus_code && fmt->mbus_code == *mbus_code)
			return fmt;
		if (index == id)
			def_fmt = fmt;
		id++;
	}
	return def_fmt;
}

static struct mxc_isi_fmt *mxc_isi_get_src_fmt(struct v4l2_subdev_format *sd_fmt)
{
	u32 index;

	/* two fmt RGB32 and YUV444 from pixellink */
	if (sd_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16 ||
	    sd_fmt->format.code == MEDIA_BUS_FMT_YVYU8_2X8 ||
	    sd_fmt->format.code == MEDIA_BUS_FMT_AYUV8_1X32 ||
	    sd_fmt->format.code == MEDIA_BUS_FMT_UYVY8_2X8 ||
	    sd_fmt->format.code == MEDIA_BUS_FMT_UYVY8_1X16||
	    sd_fmt->format.code == MEDIA_BUS_FMT_YUYV8_2X8)
		index = 1;
	else
		index = 0;
	return &mxc_isi_src_formats[index];
}

static inline struct mxc_isi_buffer *to_isi_buffer(struct vb2_v4l2_buffer *v4l2_buf)
{
	return container_of(v4l2_buf, struct mxc_isi_buffer, v4l2_buf);
}

/*
 * mxc_isi_pipeline_enable() - Enable streaming on a pipeline
 */
static int mxc_isi_pipeline_enable(struct mxc_isi_cap_dev *isi_cap, bool enable)
{
	struct device *dev = &isi_cap->pdev->dev;
	struct media_entity *entity = &isi_cap->vdev.entity;
	struct media_device *mdev = entity->graph_obj.mdev;
	struct media_graph graph;
	struct v4l2_subdev *subdev;
	int ret = 0;

	mutex_lock(&mdev->graph_mutex);

	ret = media_graph_walk_init(&graph, entity->graph_obj.mdev);
	if (ret) {
		mutex_unlock(&mdev->graph_mutex);
		return ret;
	}
	media_graph_walk_start(&graph, entity);

	while ((entity = media_graph_walk_next(&graph))) {
		if (!entity) {
			dev_dbg(dev, "entity is NULL\n");
			continue;
		}

		if (!is_media_entity_v4l2_subdev(entity)) {
			dev_dbg(dev, "%s is no v4l2 subdev\n", entity->name);
			continue;
		}

		subdev = media_entity_to_v4l2_subdev(entity);
		if (!subdev) {
			dev_dbg(dev, "%s subdev is NULL\n", entity->name);
			continue;
		}

		ret = v4l2_subdev_call(subdev, video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(dev, "subdev %s s_stream failed\n", subdev->name);
			break;
		}
	}
	mutex_unlock(&mdev->graph_mutex);
	media_graph_walk_cleanup(&graph);

	return ret;
}

void mxc_isi_cap_frame_write_done(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_cap_dev *isi_cap = mxc_isi->isi_cap;
	struct device *dev = &isi_cap->pdev->dev;
	struct mxc_isi_buffer *buf;
	struct vb2_buffer *vb2;
	unsigned long flags;

	spin_lock_irqsave(&isi_cap->slock, flags);

	if (list_empty(&isi_cap->out_active)) {
		dev_warn(dev, "trying to access empty active list\n");
		goto unlock;
	}

	buf = list_first_entry(&isi_cap->out_active, struct mxc_isi_buffer, list);

	/*
	 * Skip frame when buffer number is not match ISI trigger
	 * buffer
	 */
	if ((is_buf_active(mxc_isi, 1) && buf->id == MXC_ISI_BUF1) ||
	    (is_buf_active(mxc_isi, 2) && buf->id == MXC_ISI_BUF2)) {
		dev_dbg(dev, "status=0x%x id=%d\n", mxc_isi->status, buf->id);
		goto unlock;
	}

	if (buf->discard) {
		list_move_tail(isi_cap->out_active.next, &isi_cap->out_discard);
	} else {
		vb2 = &buf->v4l2_buf.vb2_buf;
		list_del_init(&buf->list);
		buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_DONE);
	}

	isi_cap->frame_count++;

	if (list_empty(&isi_cap->out_pending)) {
		if (list_empty(&isi_cap->out_discard)) {
			dev_warn(dev, "trying to access empty discard list\n");
			goto unlock;
		}

		buf = list_first_entry(&isi_cap->out_discard,
				       struct mxc_isi_buffer, list);
		buf->v4l2_buf.sequence = isi_cap->frame_count;
		mxc_isi_channel_set_outbuf_loc(mxc_isi, buf);
		list_move_tail(isi_cap->out_discard.next, &isi_cap->out_active);
		goto unlock;
	}

	/* ISI channel output buffer */
	buf = list_first_entry(&isi_cap->out_pending, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = isi_cap->frame_count;
	mxc_isi_channel_set_outbuf_loc(mxc_isi, buf);
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(isi_cap->out_pending.next, &isi_cap->out_active);

unlock:
	spin_unlock_irqrestore(&isi_cap->slock, flags);
}
EXPORT_SYMBOL_GPL(mxc_isi_cap_frame_write_done);

static int cap_vb2_queue_setup(struct vb2_queue *q,
			       unsigned int *num_buffers,
			       unsigned int *num_planes,
			       unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct mxc_isi_cap_dev *isi_cap = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *format = &isi_cap->pix;
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	struct mxc_isi_fmt *fmt = dst_f->fmt;
	int i;

	if (!fmt)
		return -EINVAL;

	if (*num_planes) {
		if (*num_planes != fmt->memplanes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++)
			if (sizes[i] < dst_f->sizeimage[i])
				return -EINVAL;
		return 0;
	}

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		alloc_devs[i] = &isi_cap->pdev->dev;
		sizes[i] = format->plane_fmt[i].sizeimage;
	}

	dev_dbg(&isi_cap->pdev->dev, "%s, buf_n=%d, size=%d\n",
		__func__, *num_buffers, sizes[0]);

	return 0;
}

static int cap_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *q = vb2->vb2_queue;
	struct mxc_isi_cap_dev *isi_cap = vb2_get_drv_priv(q);
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	int i;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	if (!isi_cap->dst_f.fmt)
		return -EINVAL;

	for (i = 0; i < dst_f->fmt->memplanes; i++) {
		unsigned long size = dst_f->sizeimage[i];

		if (vb2_plane_size(vb2, i) < size) {
			v4l2_err(&isi_cap->vdev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb2, i), size);

			return -EINVAL;
		}

		vb2_set_plane_payload(vb2, i, size);
	}

	return 0;
}

static int cap_vb2_buffer_init(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_buffer *buf = to_isi_buffer(v4l2_buf);
	struct mxc_isi_cap_dev *isi_cap = vb2_get_drv_priv(vb2->vb2_queue);
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	int i;

	for (i = 0; i < dst_f->fmt->memplanes; ++i)
		buf->dma_addrs[i] = vb2_dma_contig_plane_dma_addr(vb2, i);

	if (dst_f->fmt->colplanes != dst_f->fmt->memplanes) {
		unsigned int size = dst_f->bytesperline[0] * dst_f->height;

		for (i = 1; i < dst_f->fmt->colplanes; ++i)
			buf->dma_addrs[i] = buf->dma_addrs[i-1] + size;
	}

	return 0;
}

static void cap_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_buffer *buf = to_isi_buffer(v4l2_buf);
	struct mxc_isi_cap_dev *isi_cap = vb2_get_drv_priv(vb2->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&isi_cap->slock, flags);

	list_add_tail(&buf->list, &isi_cap->out_pending);
	v4l2_buf->field = V4L2_FIELD_NONE;

	spin_unlock_irqrestore(&isi_cap->slock, flags);
}

static int cap_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_cap_dev *isi_cap = vb2_get_drv_priv(q);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	struct mxc_isi_buffer *buf;
	struct vb2_buffer *vb2;
	unsigned long flags;
	int i, j;
	int ret;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	if (count < 2) {
		ret = -ENOBUFS;
		goto err;
	}

	if (!mxc_isi) {
		ret = -EINVAL;
		goto err;
	}

	/* Create a buffer for discard operation */
	for (i = 0; i < isi_cap->pix.num_planes; i++) {
		isi_cap->discard_size[i] = isi_cap->dst_f.sizeimage[i];
		isi_cap->discard_buffer[i] =
			dma_alloc_coherent(&isi_cap->pdev->dev,
					   PAGE_ALIGN(isi_cap->discard_size[i]),
					   &isi_cap->discard_buffer_dma[i],
					   GFP_DMA | GFP_KERNEL);
		if (!isi_cap->discard_buffer[i]) {
			for (j = 0; j < i; j++) {
				dma_free_coherent(&isi_cap->pdev->dev,
						  PAGE_ALIGN(isi_cap->discard_size[j]),
						  isi_cap->discard_buffer[j],
						  isi_cap->discard_buffer_dma[j]);
				dev_err(&isi_cap->pdev->dev,
					"alloc dma buffer(%d) fail\n", j);
			}
			ret = -ENOMEM;
			goto err;
		}
		dev_dbg(&isi_cap->pdev->dev,
			"%s: num_plane=%d discard_size=%d discard_buffer=%p\n"
			, __func__, i,
			PAGE_ALIGN((int)isi_cap->discard_size[i]),
			isi_cap->discard_buffer[i]);
	}

	spin_lock_irqsave(&isi_cap->slock, flags);

	/* add two list member to out_discard list head */
	isi_cap->buf_discard[0].discard = true;
	vb2 = &isi_cap->buf_discard[0].v4l2_buf.vb2_buf;
	vb2->num_planes = isi_cap->pix.num_planes;
	list_add_tail(&isi_cap->buf_discard[0].list, &isi_cap->out_discard);

	isi_cap->buf_discard[1].discard = true;
	vb2 = &isi_cap->buf_discard[1].v4l2_buf.vb2_buf;
	vb2->num_planes = isi_cap->pix.num_planes;
	list_add_tail(&isi_cap->buf_discard[1].list, &isi_cap->out_discard);

	/* ISI channel output buffer 1 */
	buf = list_first_entry(&isi_cap->out_discard, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = 0;
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	mxc_isi_channel_set_outbuf_loc(mxc_isi, buf);
	list_move_tail(isi_cap->out_discard.next, &isi_cap->out_active);

	/* ISI channel output buffer 2 */
	buf = list_first_entry(&isi_cap->out_pending, struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = 1;
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	mxc_isi_channel_set_outbuf_loc(mxc_isi, buf);
	list_move_tail(isi_cap->out_pending.next, &isi_cap->out_active);

	/* Clear frame count */
	isi_cap->frame_count = 1;
	spin_unlock_irqrestore(&isi_cap->slock, flags);

	return 0;

err:
	spin_lock_irqsave(&isi_cap->slock, flags);
	while (!list_empty(&isi_cap->out_active)) {
		buf = list_entry(isi_cap->out_active.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		if (buf->discard)
			continue;

		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&isi_cap->out_pending)) {
		buf = list_entry(isi_cap->out_pending.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}
	spin_unlock_irqrestore(&isi_cap->slock, flags);

	return ret;
}

static void cap_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_cap_dev *isi_cap = vb2_get_drv_priv(q);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	struct mxc_isi_buffer *buf;
	unsigned long flags;
	int i;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	mxc_isi_channel_disable_loc(mxc_isi);

	spin_lock_irqsave(&isi_cap->slock, flags);

	while (!list_empty(&isi_cap->out_active)) {
		buf = list_entry(isi_cap->out_active.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		if (buf->discard)
			continue;

		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&isi_cap->out_pending)) {
		buf = list_entry(isi_cap->out_pending.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&isi_cap->out_discard)) {
		buf = list_entry(isi_cap->out_discard.next,
				 struct mxc_isi_buffer, list);
		list_del_init(&buf->list);
	}

	INIT_LIST_HEAD(&isi_cap->out_active);
	INIT_LIST_HEAD(&isi_cap->out_pending);
	INIT_LIST_HEAD(&isi_cap->out_discard);

	spin_unlock_irqrestore(&isi_cap->slock, flags);

	for (i = 0; i < isi_cap->pix.num_planes; i++)
		dma_free_coherent(&isi_cap->pdev->dev,
				  PAGE_ALIGN(isi_cap->discard_size[i]),
				  isi_cap->discard_buffer[i],
				  isi_cap->discard_buffer_dma[i]);
}

static struct vb2_ops mxc_cap_vb2_qops = {
	.queue_setup		= cap_vb2_queue_setup,
	.buf_prepare		= cap_vb2_buffer_prepare,
	.buf_init		= cap_vb2_buffer_init,
	.buf_queue		= cap_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= cap_vb2_start_streaming,
	.stop_streaming		= cap_vb2_stop_streaming,
};

/*
 * V4L2 controls handling
 */
static inline struct mxc_isi_cap_dev *ctrl_to_isi_cap(struct v4l2_ctrl *ctrl)
{
	return container_of(ctrl->handler, struct mxc_isi_cap_dev, ctrls.handler);
}

static int mxc_isi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mxc_isi_cap_dev *isi_cap = ctrl_to_isi_cap(ctrl);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	unsigned long flags;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	spin_lock_irqsave(&mxc_isi->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		if (ctrl->val < 0)
			return -EINVAL;
		mxc_isi->hflip = (ctrl->val > 0) ? 1 : 0;
		break;

	case V4L2_CID_VFLIP:
		if (ctrl->val < 0)
			return -EINVAL;
		mxc_isi->vflip = (ctrl->val > 0) ? 1 : 0;
		break;

	case V4L2_CID_ALPHA_COMPONENT:
		if (ctrl->val < 0 || ctrl->val > 255)
			return -EINVAL;
		mxc_isi->alpha = ctrl->val;
		mxc_isi->alphaen = 1;
		break;

	default:
		dev_err(&isi_cap->pdev->dev,
			"%s: Not support %d CID\n", __func__, ctrl->id);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&mxc_isi->slock, flags);
	return 0;
}

static const struct v4l2_ctrl_ops mxc_isi_ctrl_ops = {
	.s_ctrl = mxc_isi_s_ctrl,
};

static int mxc_isi_ctrls_create(struct mxc_isi_cap_dev *isi_cap)
{
	struct mxc_isi_ctrls *ctrls = &isi_cap->ctrls;
	struct v4l2_ctrl_handler *handler = &ctrls->handler;

	if (isi_cap->ctrls.ready)
		return 0;

	v4l2_ctrl_handler_init(handler, 4);

	ctrls->hflip = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					 V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					 V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrls->alpha = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					 V4L2_CID_ALPHA_COMPONENT,
					 0, 0xff, 1, 0);

	if (!handler->error)
		ctrls->ready = true;

	return handler->error;
}

static void mxc_isi_ctrls_delete(struct mxc_isi_cap_dev *isi_cap)
{
	struct mxc_isi_ctrls *ctrls = &isi_cap->ctrls;

	if (ctrls->ready) {
		v4l2_ctrl_handler_free(&ctrls->handler);
		ctrls->ready = false;
		ctrls->alpha = NULL;
	}
}

static struct media_pad
*mxc_isi_get_remote_source_pad(struct v4l2_subdev *subdev)
{
	struct media_pad *sink_pad, *source_pad;
	int i;

	while (1) {
		source_pad = NULL;
		for (i = 0; i < subdev->entity.num_pads; i++) {
			sink_pad = &subdev->entity.pads[i];

			if (sink_pad->flags & MEDIA_PAD_FL_SINK) {
				source_pad = media_pad_remote_pad_first(sink_pad);
				if (source_pad)
					break;
			}
		}
		/* return first pad point in the loop  */
		return source_pad;
	}

	if (i == subdev->entity.num_pads)
		v4l2_err(subdev, "(%d): No remote pad found!\n", __LINE__);

	return NULL;
}

static struct v4l2_subdev *mxc_get_remote_subdev(struct v4l2_subdev *subdev,
						 const char * const label)
{
	struct media_pad *source_pad;
	struct v4l2_subdev *sen_sd;

	/* Get remote source pad */
	source_pad = mxc_isi_get_remote_source_pad(subdev);
	if (!source_pad) {
		v4l2_err(subdev, "%s, No remote pad found!\n", label);
		return NULL;
	}

	/* Get remote source pad subdev */
	sen_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (!sen_sd) {
		v4l2_err(subdev, "%s, No remote subdev found!\n", label);
		return NULL;
	}

	return sen_sd;
}

static bool is_entity_link_setup(struct mxc_isi_cap_dev *isi_cap)
{
	struct video_device *vdev = &isi_cap->vdev;
	struct v4l2_subdev *csi_sd, *sen_sd;

	if (!vdev->entity.num_links || !isi_cap->sd.entity.num_links)
		return false;

	csi_sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!csi_sd || !csi_sd->entity.num_links)
		return false;

	/* No sensor subdev for hdmi rx */
	if (strstr(csi_sd->name, "hdmi"))
		return true;

	sen_sd = mxc_get_remote_subdev(csi_sd, __func__);
	if (!sen_sd || !sen_sd->entity.num_links)
		return false;

	return true;
}

static int isi_cap_fmt_init(struct mxc_isi_cap_dev *isi_cap)
{
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	struct mxc_isi_frame *src_f = &isi_cap->src_f;
	struct v4l2_subdev_format src_fmt;
	struct v4l2_subdev *src_sd;
	int i, ret;

	src_sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!src_sd) {
		v4l2_err(&isi_cap->sd, "get remote subdev fail!\n");
		return -EINVAL;
	}

	memset(&src_fmt, 0, sizeof(src_fmt));
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(src_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(&isi_cap->sd, "get remote fmt fail!\n");
		return ret;
	}

	if (dst_f->width == 0 || dst_f->height == 0)
		set_frame_bounds(dst_f, src_fmt.format.width, src_fmt.format.height);

	if (!dst_f->fmt)
		dst_f->fmt = &mxc_isi_out_formats[0];

	for (i = 0; i < dst_f->fmt->memplanes; i++) {
		if (dst_f->bytesperline[i] == 0)
			dst_f->bytesperline[i] = dst_f->width * dst_f->fmt->depth[i] >> 3;
		if (dst_f->sizeimage[i] == 0)
			dst_f->sizeimage[i] = dst_f->bytesperline[i] * dst_f->height;
	}

	if (!src_f->fmt)
		memcpy(src_f, dst_f, sizeof(*dst_f));
	return 0;
}


static int mxc_isi_capture_open(struct file *file)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	struct device *dev = &isi_cap->pdev->dev;
	int ret = -EBUSY;

	mutex_lock(&isi_cap->lock);
	isi_cap->is_link_setup = is_entity_link_setup(isi_cap);
	if (!isi_cap->is_link_setup) {
		mutex_unlock(&isi_cap->lock);
		return 0;
	}
	mutex_unlock(&isi_cap->lock);

	if (mxc_isi->m2m_enabled) {
		dev_err(dev, "ISI channel[%d] is busy\n", isi_cap->id);
		return ret;
	}

	mutex_lock(&isi_cap->lock);
	ret = v4l2_fh_open(file);
	if (ret) {
		mutex_unlock(&isi_cap->lock);
		return ret;
	}
	mutex_unlock(&isi_cap->lock);

	pm_runtime_get_sync(dev);

	mutex_lock(&isi_cap->lock);
	ret = isi_cap_fmt_init(isi_cap);
	mutex_unlock(&isi_cap->lock);

	/* increase usage count for ISI channel */
	mutex_lock(&mxc_isi->lock);
	atomic_inc(&mxc_isi->usage_count);
	mxc_isi->cap_enabled = true;
	mutex_unlock(&mxc_isi->lock);

	return ret;
}

static int mxc_isi_capture_release(struct file *file)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	struct device *dev = &isi_cap->pdev->dev;
	struct vb2_queue *q = vdev->queue;
	int ret = -1;

	if (!isi_cap->is_link_setup)
		return 0;

	if (isi_cap->is_streaming[isi_cap->id])
		mxc_isi_cap_streamoff(file, NULL, q->type);

	mutex_lock(&isi_cap->lock);
	ret = _vb2_fop_release(file, NULL);
	if (ret) {
		dev_err(dev, "%s fail\n", __func__);
		mutex_unlock(&isi_cap->lock);
		goto label;
	}
	mutex_unlock(&isi_cap->lock);

	if (atomic_read(&mxc_isi->usage_count) > 0 &&
	    atomic_dec_and_test(&mxc_isi->usage_count))
		mxc_isi_channel_deinit(mxc_isi);

label:
	mutex_lock(&mxc_isi->lock);
	if (atomic_read(&mxc_isi->usage_count) == 0) {
		mxc_isi->cap_enabled = false;
		pm_runtime_put(dev);
	}
	mutex_unlock(&mxc_isi->lock);

	return (ret) ? ret : 0;
}

static const struct v4l2_file_operations mxc_isi_capture_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_isi_capture_open,
	.release	= mxc_isi_capture_release,
	.poll		= vb2_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= vb2_fop_mmap,
};

/*
 * The video node ioctl operations
 */
static int mxc_isi_cap_querycap(struct file *file, void *priv,
				struct v4l2_capability *cap)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);

	strscpy(cap->driver, MXC_ISI_DRIVER_NAME, sizeof(cap->driver));
	strscpy(cap->card, MXC_ISI_DRIVER_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s.%d",
		 dev_name(&isi_cap->pdev->dev), isi_cap->id);

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mxc_isi_cap_enum_fmt(struct file *file, void *priv,
				       struct v4l2_fmtdesc *f)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct mxc_isi_fmt *fmt;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);
	if (f->index >= (int)mxc_isi_out_formats_size)
		return -EINVAL;

	fmt = &mxc_isi_out_formats[f->index];

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);

	f->pixelformat = fmt->fourcc;

	return 0;
}

static int mxc_isi_cap_g_fmt_mplane(struct file *file, void *fh,
				    struct v4l2_format *f)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	int i;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	pix->width = dst_f->o_width;
	pix->height = dst_f->o_height;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = dst_f->fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->num_planes = dst_f->fmt->memplanes;

	for (i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = dst_f->bytesperline[i];
		pix->plane_fmt[i].sizeimage = dst_f->sizeimage[i];
	}

	return 0;
}

static struct mxc_isi_fmt *
mxc_isi_cap_fmt_try(struct mxc_isi_cap_dev *isi_cap,
		    struct v4l2_pix_format_mplane *pix)
{
	struct mxc_isi_fmt *fmt = NULL;
	int i;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	for (i = 0; i < mxc_isi_out_formats_size; i++) {
		fmt = &mxc_isi_out_formats[i];
		if (fmt->fourcc == pix->pixelformat)
			break;
	}

	if (i >= mxc_isi_out_formats_size) {
		fmt = &mxc_isi_out_formats[0];
		v4l2_warn(&isi_cap->sd, "Not match format, set default\n");
	}

	/*
	 * The bit width in CHNL_IMG_CFG[HEIGHT/WIDTH] is 13, so the maximum
	 * theorical value for image width/height should be 8K, but due to ISI
	 * line buffer size limitation, the maximum value is 4K
	 *
	 * For efficient data transmission, the minimum data width should be
	 * 16(128/8)
	 */
	v4l_bound_align_image(&pix->width, 16, ISI_4K, fmt->align,
			      &pix->height, 16, ISI_4K, 1, 0);

	pix->num_planes = fmt->memplanes;
	pix->pixelformat = fmt->fourcc;
	pix->field = V4L2_FIELD_NONE;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->ycbcr_enc = V4L2_MAP_YCBCR_ENC_DEFAULT(pix->colorspace);
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	memset(pix->reserved, 0x00, sizeof(pix->reserved));

	for (i = 0; i < fmt->colplanes; i++) {
		struct v4l2_plane_pix_format *plane = &pix->plane_fmt[i];
		unsigned int bpl;

		if (i == 0)
			bpl = clamp(plane->bytesperline,
				    pix->width * fmt->depth[0] / 8,
				    65535U);
		else
			bpl = pix->plane_fmt[0].bytesperline;

		plane->bytesperline = bpl;
		plane->sizeimage = plane->bytesperline * pix->height;

		if ((i == 1) && (pix->pixelformat == V4L2_PIX_FMT_NV12 ||
				 pix->pixelformat == V4L2_PIX_FMT_NV12M))
			plane->sizeimage /= 2;
	}

	if (fmt->colplanes != fmt->memplanes) {
		for (i = 1; i < fmt->colplanes; ++i) {
			struct v4l2_plane_pix_format *plane = &pix->plane_fmt[i];

			pix->plane_fmt[0].sizeimage += plane->sizeimage;
			plane->bytesperline = 0;
			plane->sizeimage = 0;
		}
	}

	return fmt;
}

static int mxc_isi_cap_try_fmt_mplane(struct file *file, void *fh,
				      struct v4l2_format *f)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;

	mxc_isi_cap_fmt_try(isi_cap, pix);
	return 0;
}

/* Update input frame size and formate  */
static int mxc_isi_source_fmt_init(struct mxc_isi_cap_dev *isi_cap)
{
	struct mxc_isi_frame *src_f = &isi_cap->src_f;
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	struct v4l2_subdev_format src_fmt;
	struct media_pad *source_pad;
	struct v4l2_subdev *src_sd;
	int ret;

	source_pad = mxc_isi_get_remote_source_pad(&isi_cap->sd);
	if (!source_pad) {
		v4l2_err(&isi_cap->sd,
			 "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	src_sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!src_sd)
		return -EINVAL;

	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	src_fmt.format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	src_fmt.format.width = dst_f->width;
	src_fmt.format.height = dst_f->height;
	ret = v4l2_subdev_call(src_sd, pad, set_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(&isi_cap->sd, "set remote fmt fail!\n");
		return ret;
	}

	memset(&src_fmt, 0, sizeof(src_fmt));
	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(src_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(&isi_cap->sd, "get remote fmt fail!\n");
		return ret;
	}

	/* Pixel link master will transfer format to RGB32 or YUV32 */
	src_f->fmt = mxc_isi_get_src_fmt(&src_fmt);

	set_frame_bounds(src_f, src_fmt.format.width, src_fmt.format.height);

	if (dst_f->width > src_f->width || dst_f->height > src_f->height) {
		dev_err(&isi_cap->pdev->dev,
			"%s: src:(%d,%d), dst:(%d,%d) Not support upscale\n",
			__func__,
			src_f->width, src_f->height,
			dst_f->width, dst_f->height);
		return -EINVAL;
	}

	return 0;
}

static int mxc_isi_cap_s_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_format *f)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	struct mxc_isi_fmt *fmt;
	int i;

	/* Step1: Check format with output support format list.
	 * Step2: Update output frame information.
	 * Step3: Checkout the format whether is supported by remote subdev
	 *	 Step3.1: If Yes, call remote subdev set_fmt.
	 *	 Step3.2: If NO, call remote subdev get_fmt.
	 * Step4: Update input frame information.
	 * Step5: Update mxc isi channel configuration.
	 */

	dev_dbg(&isi_cap->pdev->dev, "%s, fmt=0x%X\n", __func__, pix->pixelformat);
	if (vb2_is_busy(&isi_cap->vb2_q))
		return -EBUSY;

	fmt = mxc_isi_cap_fmt_try(isi_cap, pix);
	if (!fmt)
		fmt = &mxc_isi_out_formats[0];

	dst_f->fmt = fmt;
	dst_f->width = pix->width;
	dst_f->height = pix->height;

	for (i = 0; i < pix->num_planes; i++) {
		dst_f->bytesperline[i] = pix->plane_fmt[i].bytesperline;
		dst_f->sizeimage[i]    = pix->plane_fmt[i].sizeimage;
	}

	memcpy(&isi_cap->pix, pix, sizeof(*pix));
	set_frame_bounds(dst_f, pix->width, pix->height);

	return 0;
}

int mxc_isi_config_parm(struct mxc_isi_cap_dev *isi_cap)
{
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	int ret;

	ret = mxc_isi_source_fmt_init(isi_cap);
	if (ret < 0)
		return -EINVAL;

	mxc_isi_channel_init(mxc_isi);
	mxc_isi_channel_config_loc(mxc_isi, &isi_cap->src_f, &isi_cap->dst_f);

	return 0;
}
EXPORT_SYMBOL_GPL(mxc_isi_config_parm);

static int mxc_isi_cap_g_parm(struct file *file, void *fh,
			      struct v4l2_streamparm *a)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct v4l2_subdev_frame_interval ival = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_subdev *sd;
	int ret;

	sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!sd)
		return -ENODEV;

	ret = v4l2_subdev_call(sd, pad, get_frame_interval, NULL, &ival);
	if (ret < 0) {
		dev_err(&isi_cap->pdev->dev, "failed to call get_frame_interval\n");
		return ret;
	}

	a->parm.capture.timeperframe = ival.interval;
	return 0;
}

static int mxc_isi_cap_s_parm(struct file *file, void *fh,
			      struct v4l2_streamparm *a)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct v4l2_subdev_frame_interval ival = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.interval = a->parm.capture.timeperframe,
	};
	struct v4l2_subdev *sd;

	sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!sd)
		return -ENODEV;

	return v4l2_subdev_call(sd, pad, set_frame_interval, NULL, &ival);
}


static int mxc_isi_cap_streamon(struct file *file, void *priv,
				enum v4l2_buf_type type)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	struct device *dev = &isi_cap->pdev->dev;
	struct vb2_queue *q = &isi_cap->vb2_q;
	struct v4l2_subdev *src_sd;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	if (!isi_cap->is_streaming[isi_cap->id]) {
		src_sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
		ret = (!src_sd) ? -EINVAL : v4l2_subdev_call(src_sd, core, s_power, 1);
		if (ret) {
			v4l2_err(&isi_cap->sd, "Call subdev s_power fail!\n");
			return ret;
		}

		ret = mxc_isi_config_parm(isi_cap);
		if (ret < 0)
			goto power;
	}

	ret = vb2_ioctl_streamon(file, priv, type);
	if (ret < 0) {
		if (!isi_cap->is_streaming[isi_cap->id])
			goto power;
		else
			return ret;
	}

	if (!isi_cap->is_streaming[isi_cap->id] &&
	     q->start_streaming_called) {
		mxc_isi_channel_enable_loc(mxc_isi, mxc_isi->m2m_enabled);
		ret = mxc_isi_pipeline_enable(isi_cap, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			goto disable;

		isi_cap->is_streaming[isi_cap->id] = 1;
		mxc_isi->is_streaming = 1;
	}

	return 0;

disable:
	mxc_isi_channel_disable_loc(mxc_isi);
power:
	v4l2_subdev_call(src_sd, core, s_power, 0);
	return ret;
}

static int mxc_isi_cap_streamoff(struct file *file, void *priv,
				 enum v4l2_buf_type type)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	struct device *dev = &isi_cap->pdev->dev;
	struct v4l2_subdev *src_sd;
	int ret;

	dev_dbg(dev, "%s\n", __func__);

	ret = vb2_ioctl_streamoff(file, priv, type);
	if (ret < 0)
		return ret;

	if (isi_cap->is_streaming[isi_cap->id]) {
		mxc_isi_pipeline_enable(isi_cap, 0);
		mxc_isi_channel_disable_loc(mxc_isi);

		isi_cap->is_streaming[isi_cap->id] = 0;
		mxc_isi->is_streaming = 0;

		src_sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
		return v4l2_subdev_call(src_sd, core, s_power, 0);
	}

	return 0;
}

static int mxc_isi_cap_g_selection(struct file *file, void *fh,
				   struct v4l2_selection *s)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct mxc_isi_frame *f = &isi_cap->dst_f;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = f->o_width;
		s->r.height = f->o_height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
		s->r.left = f->h_off;
		s->r.top = f->v_off;
		s->r.width = f->c_width;
		s->r.height = f->c_height;
		return 0;
	}

	return -EINVAL;
}

static int enclosed_rectangle(struct v4l2_rect *a, struct v4l2_rect *b)
{
	if (a->left < b->left || a->top < b->top)
		return 0;

	if (a->left + a->width > b->left + b->width)
		return 0;

	if (a->top + a->height > b->top + b->height)
		return 0;

	return 1;
}

static int mxc_isi_cap_s_selection(struct file *file, void *fh,
				   struct v4l2_selection *s)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct mxc_isi_frame *f;
	struct v4l2_rect rect = s->r;
	unsigned long flags;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);
	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT &&
	    s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	if (s->target == V4L2_SEL_TGT_COMPOSE)
		f = &isi_cap->dst_f;
	else
		return -EINVAL;

	bounds_adjust(f, &rect);

	if (s->flags & V4L2_SEL_FLAG_LE &&
	    !enclosed_rectangle(&rect, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE &&
	    !enclosed_rectangle(&s->r, &rect))
		return -ERANGE;

	if ((s->flags & V4L2_SEL_FLAG_LE) &&
	    (s->flags & V4L2_SEL_FLAG_GE) &&
	    (rect.width != s->r.width || rect.height != s->r.height))
		return -ERANGE;

	s->r = rect;
	spin_lock_irqsave(&isi_cap->slock, flags);
	set_frame_crop(f, s->r.left, s->r.top, s->r.width,
		       s->r.height);
	spin_unlock_irqrestore(&isi_cap->slock, flags);

	return 0;
}

static int mxc_isi_cap_enum_framesizes(struct file *file, void *priv,
				       struct v4l2_frmsizeenum *fsize)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct device_node *parent;
	struct v4l2_subdev *sd;
	struct mxc_isi_fmt *fmt;
	struct v4l2_subdev_frame_size_enum fse = {
		.index = fsize->index,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	fmt = mxc_isi_find_format(&fsize->pixel_format, NULL, 0);
	if (!fmt || fmt->fourcc != fsize->pixel_format)
		return -EINVAL;
	fse.code = fmt->mbus_code;

	sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!sd) {
		v4l2_err(&isi_cap->sd, "Can't find subdev\n");
		return -ENODEV;
	}

	ret = v4l2_subdev_call(sd, pad, enum_frame_size, NULL, &fse);
	if (ret)
		return ret;

	parent = of_get_parent(isi_cap->pdev->dev.of_node);
	if ((of_device_is_compatible(parent, "fsl,imx8mp-isi")) &&
	    (fse.max_width > ISI_2K || fse.min_width > ISI_2K) &&
	    (isi_cap->id == 1))
		return -EINVAL;

	if (fse.min_width == fse.max_width &&
	    fse.min_height == fse.max_height) {
		fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
		fsize->discrete.width = fse.min_width;
		fsize->discrete.height = fse.min_height;
		return 0;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fsize->stepwise.min_width = fse.min_width;
	fsize->stepwise.max_width = fse.max_width;
	fsize->stepwise.min_height = fse.min_height;
	fsize->stepwise.max_height = fse.max_height;
	fsize->stepwise.step_width = 1;
	fsize->stepwise.step_height = 1;

	return 0;
}

static int mxc_isi_cap_enum_frameintervals(struct file *file, void *fh,
					   struct v4l2_frmivalenum *interval)
{
	struct mxc_isi_cap_dev *isi_cap = video_drvdata(file);
	struct device_node *parent;
	struct v4l2_subdev *sd;
	struct mxc_isi_fmt *fmt;
	struct v4l2_subdev_frame_interval_enum fie = {
		.index = interval->index,
		.width = interval->width,
		.height = interval->height,
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	int ret;

	fmt = mxc_isi_find_format(&interval->pixel_format, NULL, 0);
	if (!fmt || fmt->fourcc != interval->pixel_format)
		return -EINVAL;
	fie.code = fmt->mbus_code;

	sd = mxc_get_remote_subdev(&isi_cap->sd, __func__);
	if (!sd)
		return -EINVAL;

	ret = v4l2_subdev_call(sd, pad, enum_frame_interval, NULL, &fie);
	if (ret)
		return ret;

	parent = of_get_parent(isi_cap->pdev->dev.of_node);
	if (of_device_is_compatible(parent, "fsl,imx8mp-isi") &&
	    fie.width > ISI_2K && isi_cap->id == 1)
		return -EINVAL;

	interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	interval->discrete = fie.interval;

	return 0;
}

static const struct v4l2_ioctl_ops mxc_isi_capture_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_cap_querycap,

	.vidioc_enum_fmt_vid_cap	= mxc_isi_cap_enum_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= mxc_isi_cap_try_fmt_mplane,
	.vidioc_s_fmt_vid_cap_mplane	= mxc_isi_cap_s_fmt_mplane,
	.vidioc_g_fmt_vid_cap_mplane	= mxc_isi_cap_g_fmt_mplane,

	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,

	.vidioc_g_parm			= mxc_isi_cap_g_parm,
	.vidioc_s_parm			= mxc_isi_cap_s_parm,

	.vidioc_streamon		= mxc_isi_cap_streamon,
	.vidioc_streamoff		= mxc_isi_cap_streamoff,

	.vidioc_g_selection		= mxc_isi_cap_g_selection,
	.vidioc_s_selection		= mxc_isi_cap_s_selection,

	.vidioc_enum_framesizes = mxc_isi_cap_enum_framesizes,
	.vidioc_enum_frameintervals = mxc_isi_cap_enum_frameintervals,

	.vidioc_subscribe_event   =  v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event =  v4l2_event_unsubscribe,
};

/* Capture subdev media entity operations */
static int mxc_isi_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct mxc_isi_cap_dev *isi_cap = v4l2_get_subdevdata(sd);

	if (WARN_ON(!isi_cap))
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED))
		return 0;

	/* Add ISI source and sink pad link configuration */
	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case MXC_ISI_SD_PAD_SOURCE_DC0:
		case MXC_ISI_SD_PAD_SOURCE_DC1:
			break;
		case MXC_ISI_SD_PAD_SOURCE_MEM:
			break;
		default:
			dev_err(&isi_cap->pdev->dev, "invalid source pad\n");
			return -EINVAL;
		}
	} else if (local->flags & MEDIA_PAD_FL_SINK) {
		switch (local->index) {
		case MXC_ISI_SD_PAD_SINK_MIPI0_VC0:
		case MXC_ISI_SD_PAD_SINK_MIPI0_VC1:
		case MXC_ISI_SD_PAD_SINK_MIPI0_VC2:
		case MXC_ISI_SD_PAD_SINK_MIPI0_VC3:
		case MXC_ISI_SD_PAD_SINK_MIPI1_VC0:
		case MXC_ISI_SD_PAD_SINK_MIPI1_VC1:
		case MXC_ISI_SD_PAD_SINK_MIPI1_VC2:
		case MXC_ISI_SD_PAD_SINK_MIPI1_VC3:
		case MXC_ISI_SD_PAD_SINK_HDMI:
		case MXC_ISI_SD_PAD_SINK_DC0:
		case MXC_ISI_SD_PAD_SINK_DC1:
		case MXC_ISI_SD_PAD_SINK_MEM:
		case MXC_ISI_SD_PAD_SINK_PARALLEL_CSI:
			break;
		default:
			dev_err(&isi_cap->pdev->dev,
				"%s invalid sink pad\n", __func__);
			return -EINVAL;
		}
	}

	return 0;
}

static const struct media_entity_operations mxc_isi_sd_media_ops = {
	.link_setup = mxc_isi_link_setup,
};

static int mxc_isi_subdev_enum_mbus_code(struct v4l2_subdev *sd,
					 struct v4l2_subdev_state *sd_state,
					 struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int mxc_isi_subdev_get_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_cap_dev *isi_cap = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f;
	struct v4l2_mbus_framefmt *mf = &fmt->format;

	mutex_lock(&isi_cap->lock);

	switch (fmt->pad) {
	case MXC_ISI_SD_PAD_SOURCE_MEM:
	case MXC_ISI_SD_PAD_SOURCE_DC0:
	case MXC_ISI_SD_PAD_SOURCE_DC1:
		f = &isi_cap->dst_f;
		break;
	case MXC_ISI_SD_PAD_SINK_MIPI0_VC0:
	case MXC_ISI_SD_PAD_SINK_MIPI0_VC1:
	case MXC_ISI_SD_PAD_SINK_MIPI0_VC2:
	case MXC_ISI_SD_PAD_SINK_MIPI0_VC3:
	case MXC_ISI_SD_PAD_SINK_MIPI1_VC0:
	case MXC_ISI_SD_PAD_SINK_MIPI1_VC1:
	case MXC_ISI_SD_PAD_SINK_MIPI1_VC2:
	case MXC_ISI_SD_PAD_SINK_MIPI1_VC3:
	case MXC_ISI_SD_PAD_SINK_HDMI:
	case MXC_ISI_SD_PAD_SINK_DC0:
	case MXC_ISI_SD_PAD_SINK_DC1:
	case MXC_ISI_SD_PAD_SINK_MEM:
		f = &isi_cap->src_f;
		break;
	default:
		mutex_unlock(&isi_cap->lock);
		v4l2_err(&isi_cap->sd,
			 "%s, Pad is not support now!\n", __func__);
		return -1;
	}

	if (!WARN_ON(!f->fmt))
		mf->code = f->fmt->mbus_code;

	/* Source/Sink pads crop rectangle size */
	mf->width = f->width;
	mf->height = f->height;
	mf->colorspace = V4L2_COLORSPACE_SRGB;

	mutex_unlock(&isi_cap->lock);

	return 0;
}

static int mxc_isi_subdev_set_fmt(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_cap_dev *isi_cap = v4l2_get_subdevdata(sd);
	struct device_node *parent;
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct mxc_isi_frame *dst_f = &isi_cap->dst_f;
	struct mxc_isi_fmt *out_fmt;
	int i;

	if (fmt->pad < MXC_ISI_SD_PAD_SOURCE_MEM &&
	    vb2_is_busy(&isi_cap->vb2_q))
		return -EBUSY;

	for (i = 0; i < mxc_isi_out_formats_size; i++) {
		out_fmt = &mxc_isi_out_formats[i];
		if (mf->code == out_fmt->mbus_code)
			break;
	}
	if (i >= mxc_isi_out_formats_size) {
		v4l2_err(&isi_cap->sd,
			 "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	parent = of_get_parent(isi_cap->pdev->dev.of_node);
	if (of_device_is_compatible(parent, "nxp,imx8mn-isi") &&
	    mf->width > ISI_2K)
		return -EINVAL;

	mutex_lock(&isi_cap->lock);
	/* update out put frame size and formate */
	dst_f->fmt = &mxc_isi_out_formats[i];
	set_frame_bounds(dst_f, mf->width, mf->height);
	mutex_unlock(&isi_cap->lock);

	dev_dbg(&isi_cap->pdev->dev, "pad%d: code: 0x%x, %dx%d",
		fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int mxc_isi_subdev_get_selection(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *sd_state,
					struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_cap_dev *isi_cap = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &isi_cap->src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;

	mutex_lock(&isi_cap->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &isi_cap->dst_f;
		fallthrough;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		r->width = f->o_width;
		r->height = f->o_height;
		r->left = 0;
		r->top = 0;
		mutex_unlock(&isi_cap->lock);
		return 0;

	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_state_get_crop(sd_state, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_state_get_compose(sd_state, sel->pad);
		f = &isi_cap->dst_f;
		break;
	default:
		mutex_unlock(&isi_cap->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		sel->r = *try_sel;
	} else {
		r->left = f->h_off;
		r->top = f->v_off;
		r->width = f->width;
		r->height = f->height;
	}

	dev_dbg(&isi_cap->pdev->dev,
		"%s, target %#x: l:%d, t:%d, %dx%d, f_w: %d, f_h: %d",
		__func__, sel->pad, r->left, r->top, r->width, r->height,
		f->c_width, f->c_height);

	mutex_unlock(&isi_cap->lock);
	return 0;
}

static int mxc_isi_subdev_set_selection(struct v4l2_subdev *sd,
					struct v4l2_subdev_state *sd_state,
					struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_cap_dev *isi_cap = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &isi_cap->src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;
	unsigned long flags;

	mutex_lock(&isi_cap->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_state_get_crop(sd_state, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_state_get_compose(sd_state, sel->pad);
		f = &isi_cap->dst_f;
		break;
	default:
		mutex_unlock(&isi_cap->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		*try_sel = sel->r;
	} else {
		spin_lock_irqsave(&isi_cap->slock, flags);
		set_frame_crop(f, r->left, r->top, r->width, r->height);
		spin_unlock_irqrestore(&isi_cap->slock, flags);
	}

	dev_dbg(&isi_cap->pdev->dev, "%s, target %#x: (%d,%d)/%dx%d", __func__,
		sel->target, r->left, r->top, r->width, r->height);

	mutex_unlock(&isi_cap->lock);

	return 0;
}

static struct v4l2_subdev_pad_ops mxc_isi_subdev_pad_ops = {
	.enum_mbus_code = mxc_isi_subdev_enum_mbus_code,
	.get_selection  = mxc_isi_subdev_get_selection,
	.set_selection  = mxc_isi_subdev_set_selection,
	.get_fmt = mxc_isi_subdev_get_fmt,
	.set_fmt = mxc_isi_subdev_set_fmt,
};

static struct v4l2_subdev_ops mxc_isi_subdev_ops = {
	.pad = &mxc_isi_subdev_pad_ops,
};

static int mxc_isi_register_cap_device(struct mxc_isi_cap_dev *isi_cap,
				       struct v4l2_device *v4l2_dev)
{
	struct video_device *vdev = &isi_cap->vdev;
	struct vb2_queue *q = &isi_cap->vb2_q;
	int ret = -ENOMEM;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);
	memset(vdev, 0, sizeof(*vdev));
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.%d.capture", isi_cap->id);

	vdev->fops	= &mxc_isi_capture_fops;
	vdev->ioctl_ops	= &mxc_isi_capture_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->queue	= q;
	vdev->lock	= &isi_cap->lock;

	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	video_set_drvdata(vdev, isi_cap);

	INIT_LIST_HEAD(&isi_cap->out_pending);
	INIT_LIST_HEAD(&isi_cap->out_active);
	INIT_LIST_HEAD(&isi_cap->out_discard);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = isi_cap;
	q->ops = &mxc_cap_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mxc_isi_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &isi_cap->lock;
	q->min_queued_buffers = 2;
	q->dev = &isi_cap->pdev->dev;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_ctx;


	isi_cap->cap_pad.flags = MEDIA_PAD_FL_SINK;
	vdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vdev->entity, 1, &isi_cap->cap_pad);
	if (ret)
		goto err_free_ctx;

	ret = mxc_isi_ctrls_create(isi_cap);
	if (ret)
		goto err_me_cleanup;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_ctrl_free;

	vdev->ctrl_handler = &isi_cap->ctrls.handler;
	v4l2_info(v4l2_dev, "Registered %s as /dev/%s\n",
		  vdev->name, video_device_node_name(vdev));

	return 0;

err_ctrl_free:
	mxc_isi_ctrls_delete(isi_cap);
err_me_cleanup:
	media_entity_cleanup(&vdev->entity);
err_free_ctx:
	return ret;
}

static int mxc_isi_subdev_registered(struct v4l2_subdev *sd)
{
	struct mxc_isi_cap_dev *isi_cap = sd_to_cap_dev(sd);
	int ret;

	if (!isi_cap)
		return -ENXIO;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	ret = mxc_isi_register_cap_device(isi_cap, sd->v4l2_dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void mxc_isi_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct mxc_isi_cap_dev *isi_cap = v4l2_get_subdevdata(sd);
	struct video_device *vdev;

	if (!isi_cap)
		return;

	dev_dbg(&isi_cap->pdev->dev, "%s\n", __func__);

	mutex_lock(&isi_cap->lock);
	vdev = &isi_cap->vdev;
	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		mxc_isi_ctrls_delete(isi_cap);
		media_entity_cleanup(&vdev->entity);
	}
	mutex_unlock(&isi_cap->lock);
}

static const struct v4l2_subdev_internal_ops mxc_isi_capture_sd_internal_ops = {
	.registered = mxc_isi_subdev_registered,
	.unregistered = mxc_isi_subdev_unregistered,
};

static int isi_cap_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct mxc_isi_dev *mxc_isi;
	struct mxc_isi_cap_dev *isi_cap;
	struct v4l2_subdev *sd;
	int ret;

	isi_cap = devm_kzalloc(dev, sizeof(*isi_cap), GFP_KERNEL);
	if (!isi_cap)
		return -ENOMEM;

	dev->parent = mxc_isi_dev_get_parent(pdev);
	if (!dev->parent) {
		dev_info(dev, "deferring %s device registration\n", dev_name(dev));
		return -EPROBE_DEFER;
	}

	mxc_isi = mxc_isi_get_hostdata(pdev);
	if (!mxc_isi) {
		dev_info(dev, "deferring %s device registration\n", dev_name(dev));
		return -EPROBE_DEFER;
	}

	isi_cap->runtime_suspend = of_property_read_bool(node, "runtime_suspend");

	isi_cap->pdev = pdev;
	isi_cap->id = mxc_isi->id;
	mxc_isi->isi_cap = isi_cap;

	spin_lock_init(&isi_cap->slock);
	mutex_init(&isi_cap->lock);

	sd = &isi_cap->sd;
	v4l2_subdev_init(sd, &mxc_isi_subdev_ops);
	snprintf(sd->name, sizeof(sd->name), "mxc_isi.%d", isi_cap->id);

	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	/* ISI Sink pads */
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC0].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC1].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC2].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC3].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC0].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC1].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC2].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC3].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_DC0].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_DC1].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_HDMI].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_MEM].flags = MEDIA_PAD_FL_SINK;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SINK_PARALLEL_CSI].flags = MEDIA_PAD_FL_SINK;

	/* ISI source pads */
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SOURCE_MEM].flags = MEDIA_PAD_FL_SOURCE;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SOURCE_DC0].flags = MEDIA_PAD_FL_SOURCE;
	isi_cap->sd_pads[MXC_ISI_SD_PAD_SOURCE_DC1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, MXC_ISI_SD_PADS_NUM, isi_cap->sd_pads);
	if (ret)
		return ret;

	sd->entity.ops   = &mxc_isi_sd_media_ops;
	sd->internal_ops = &mxc_isi_capture_sd_internal_ops;

	v4l2_set_subdevdata(sd, isi_cap);
	platform_set_drvdata(pdev, isi_cap);

	pm_runtime_enable(dev);
	return 0;
}

static void isi_cap_remove(struct platform_device *pdev)
{
	struct mxc_isi_cap_dev *isi_cap = platform_get_drvdata(pdev);
	struct v4l2_subdev *sd = &isi_cap->sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
	pm_runtime_disable(&pdev->dev);
}

static int mxc_isi_cap_pm_suspend(struct device *dev)
{
	struct mxc_isi_cap_dev *isi_cap = dev_get_drvdata(dev);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);

	if (!isi_cap->runtime_suspend) {
		if (mxc_isi->is_streaming) {
			dev_warn(dev, "running, prevent entering suspend.\n");
			return -EAGAIN;
		}
	}

	return pm_runtime_force_suspend(dev);
}

static int mxc_isi_cap_pm_resume(struct device *dev)
{
	return pm_runtime_force_resume(dev);
}

static int mxc_isi_cap_runtime_suspend(struct device *dev)
{
	struct mxc_isi_cap_dev *isi_cap = dev_get_drvdata(dev);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);

	if (isi_cap->runtime_suspend) {
		if (isi_cap->is_streaming[isi_cap->id]) {
			disp_mix_clks_enable(mxc_isi, false);
			mxc_isi_channel_disable_loc(mxc_isi);
		}
	}

	mxc_isi_clk_disable(mxc_isi);

	return 0;
}

static int mxc_isi_cap_runtime_resume(struct device *dev)
{
	struct mxc_isi_cap_dev *isi_cap = dev_get_drvdata(dev);
	struct mxc_isi_dev *mxc_isi = mxc_isi_get_hostdata(isi_cap->pdev);
	int ret;

	ret = mxc_isi_clk_enable(mxc_isi);
	if (ret) {
		dev_err(dev, "%s clk enable fail\n", __func__);
		return ret;
	}

	if (isi_cap->runtime_suspend) {
		if (isi_cap->is_streaming[isi_cap->id]) {
			disp_mix_sft_rstn(mxc_isi, false);
			disp_mix_clks_enable(mxc_isi, true);
			mxc_isi_clean_registers(mxc_isi);
			mxc_isi_config_parm(mxc_isi->isi_cap);
			mxc_isi_channel_enable_loc(mxc_isi, mxc_isi->m2m_enabled);
		}
	}

	return 0;
}

static const struct dev_pm_ops mxc_isi_cap_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(mxc_isi_cap_pm_suspend, mxc_isi_cap_pm_resume)
	SET_RUNTIME_PM_OPS(mxc_isi_cap_runtime_suspend, mxc_isi_cap_runtime_resume, NULL)
};

static const struct of_device_id isi_cap_of_match[] = {
	{.compatible = "imx-isi-capture",},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, isi_cap_of_match);

static struct platform_driver isi_cap_driver = {
	.probe  = isi_cap_probe,
	.remove = isi_cap_remove,
	.driver = {
		.of_match_table = isi_cap_of_match,
		.name		= "isi-capture",
		.pm		= &mxc_isi_cap_pm_ops,
	},
};
module_platform_driver(isi_cap_driver);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX8 Image Sensor Interface Capture driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("ISI Capture");
MODULE_VERSION("1.0");
