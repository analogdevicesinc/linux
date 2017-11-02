/*
 * Copyright (C) 2017 Freescale Semiconductor, Inc. All Rights Reserved.
 */
/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
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

#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <soc/imx8/sc/sci.h>

#include "mxc-isi-core.h"
#include "mxc-isi-hw.h"
#include "mxc-media-dev.h"
#include "max9286.h"

struct mxc_isi_fmt mxc_isi_out_formats[] = {
	{
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_RGB565,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_RGB565_1X16,
	}, {
		.name		= "RGB24",
		.fourcc		= V4L2_PIX_FMT_RGB24,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_XRGB32,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_XRGB32,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_RGB888_1X24,
	}, {
		.name		= "BGR24",
		.fourcc		= V4L2_PIX_FMT_BGR24,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_XBGR32,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_BGR888_1X24,
	}, {
		.name		= "ARGB32",
		.fourcc		= V4L2_PIX_FMT_ARGB32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_ARGB32,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code  = MEDIA_BUS_FMT_ARGB8888_1X32,
	}, {
		.name		= "YUYV-16",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= MXC_ISI_OUT_FMT_YUV422_1P8P,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	}, {
		.name		= "YUV32 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.color		= MXC_ISI_OUT_FMT_YUV444_1P8,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_AYUV8_1X32,
	},
};

struct mxc_isi_fmt mxc_isi_src_formats[] = {
	/* Pixel link input format */
	{
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.depth		= { 32 },
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV32 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.memplanes	= 1,
		.colplanes	= 1,
	}
};

struct mxc_isi_fmt mxc_isi_m2m_in_formats[] = {
	{
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color		= MXC_ISI_M2M_IN_FMT_RGB565,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV 4:2:2 packed, YCbYCr",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color		= MXC_ISI_M2M_IN_FMT_YUV422_1P8P,
		.memplanes	= 1,
		.colplanes	= 1,
		.mbus_code	= MEDIA_BUS_FMT_YUYV8_1X16,
	},
};

struct mxc_isi_fmt *mxc_isi_get_format(unsigned int index)
{
	return &mxc_isi_out_formats[index];
}

/**
 * mxc_isi_find_format - lookup mxc_isi color format by fourcc or media bus format
 */
struct mxc_isi_fmt *mxc_isi_find_format(const u32 *pixelformat, const u32 *mbus_code,
				  unsigned int mask, int index)
{
	struct mxc_isi_fmt *fmt, *def_fmt = NULL;
	unsigned int i;
	int id = 0;

	if (index >= (int)ARRAY_SIZE(mxc_isi_out_formats))
		return NULL;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		fmt = &mxc_isi_out_formats[i];
		if (!(fmt->flags & mask))
			continue;
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

struct mxc_isi_fmt *mxc_isi_get_src_fmt(struct v4l2_subdev_format *sd_fmt)
{
	u32 index;

	/* two fmt RGB32 and YUV444 from pixellink */
	if (sd_fmt->format.code == MEDIA_BUS_FMT_YUYV8_1X16)
		index = 1;
	else
		index = 0;
	return &mxc_isi_src_formats[index];
}

/*
 * mxc_isi_pipeline_enable() - Enable streaming on a pipeline
 *
 */
static int mxc_isi_pipeline_enable(struct mxc_isi_dev *mxc_isi, bool enable)
{
	struct media_entity *entity = &mxc_isi->isi_cap.vdev.entity;
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
		if (entity == NULL) {
			dev_dbg(&mxc_isi->pdev->dev,
					"%s ,entity is NULL\n", __func__);
			continue;
		}

		if (!is_media_entity_v4l2_subdev(entity)) {
			dev_dbg(&mxc_isi->pdev->dev,
					"%s ,entity is no v4l2, %s\n", __func__, entity->name);
			continue;
		}

		subdev = media_entity_to_v4l2_subdev(entity);
		if (subdev == NULL) {
			dev_dbg(&mxc_isi->pdev->dev,
					"%s ,%s,subdev is NULL\n", __func__, entity->name);
			continue;
		}

		ret = v4l2_subdev_call(subdev, video, s_stream, enable);
		if (ret < 0 && ret != -ENOIOCTLCMD) {
			dev_err(&mxc_isi->pdev->dev,
					"%s ,subdev %s s_stream failed\n", __func__, subdev->name);
			break;
		}
	}
	mutex_unlock(&mdev->graph_mutex);
	media_graph_walk_cleanup(&graph);

	return ret;
}

static int mxc_isi_update_buf_paddr(struct mxc_isi_buffer *buf, int memplanes)
{
	struct frame_addr *paddr = &buf->paddr;
	struct vb2_buffer *vb2 = &buf->v4l2_buf.vb2_buf;
	int ret = 0;

	/* support one plane now */
	paddr->cb = 0;
	paddr->cr = 0;

	switch (memplanes) {
	case 3:
		paddr->cr = vb2_dma_contig_plane_dma_addr(vb2, 2);
	case 2:
		paddr->cb = vb2_dma_contig_plane_dma_addr(vb2, 1);
	case 1:
		paddr->y = vb2_dma_contig_plane_dma_addr(vb2, 0);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

void mxc_isi_frame_write_done(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_buffer *buf;
	struct vb2_buffer *vb2;

	/* Retrun if no pending buffer */
	if (!list_empty(&mxc_isi->isi_cap.out_active)) {

		buf = list_first_entry(&mxc_isi->isi_cap.out_active,
					struct mxc_isi_buffer, list);

		list_del(&buf->list);
		buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		buf->v4l2_buf.sequence = mxc_isi->isi_cap.frame_count++;
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_DONE);
	}

	if (!list_empty(&mxc_isi->isi_cap.out_pending)) {

		/* ISI channel output buffer */
		buf = list_first_entry(&mxc_isi->isi_cap.out_pending,
						struct mxc_isi_buffer, list);
		list_del(&buf->list);

		buf->v4l2_buf.sequence = mxc_isi->isi_cap.frame_count;
		mxc_isi_channel_set_outbuf(mxc_isi, buf);
		vb2 = &buf->v4l2_buf.vb2_buf;
		vb2->state = VB2_BUF_STATE_ACTIVE;
		list_add_tail(&buf->list, &mxc_isi->isi_cap.out_active);
	}
}

static int cap_vb2_queue_setup(struct vb2_queue *q,
		       unsigned int *num_buffers, unsigned int *num_planes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	struct mxc_isi_dev *mxc_isi = q->drv_priv;
	struct mxc_isi_frame *dst_f = &mxc_isi->isi_cap.dst_f;
	struct mxc_isi_fmt *fmt = dst_f->fmt;
	unsigned long wh;
	int i;

	alloc_devs[0] = &mxc_isi->pdev->dev;
	wh = dst_f->width * dst_f->height;

	if (fmt == NULL)
		return -EINVAL;

	*num_planes = fmt->memplanes;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth[i]) / 8;
		sizes[i] = max_t(u32, size, dst_f->sizeimage[i]);
	}
	dev_dbg(&mxc_isi->pdev->dev, "%s, buf_n=%d, size=%d\n",
					__func__,  *num_buffers, sizes[0]);

	return 0;
}

static int cap_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *q = vb2->vb2_queue;
	struct mxc_isi_dev *mxc_isi = q->drv_priv;
	struct mxc_isi_frame	*dst_f = &mxc_isi->isi_cap.dst_f;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (mxc_isi->isi_cap.dst_f.fmt == NULL)
		return -EINVAL;

	for (i = 0; i < dst_f->fmt->memplanes; i++) {
		unsigned long size = dst_f->sizeimage[i];

		if (vb2_plane_size(vb2, i) < size) {
			v4l2_err(&mxc_isi->isi_cap.vdev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb2, i), size);
			return -EINVAL;
		}
#if 0 //debug only
		if (vb2_plane_vaddr(vb2, i))
			memset((void *)vb2_plane_vaddr(vb2, i), 0xaa,
					vb2_get_plane_payload(vb2, i));
#endif
		vb2_set_plane_payload(vb2, i, size);
	}

	return 0;
}

static void cap_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_buffer *buf
			= container_of(v4l2_buf, struct mxc_isi_buffer, v4l2_buf);
	struct mxc_isi_dev *mxc_isi = vb2_get_drv_priv(vb2->vb2_queue);
	unsigned long flags;

	spin_lock_irqsave(&mxc_isi->slock, flags);

	mxc_isi_update_buf_paddr(buf, mxc_isi->isi_cap.dst_f.fmt->mdataplanes);
	list_add_tail(&buf->list, &mxc_isi->isi_cap.out_pending);

	spin_unlock_irqrestore(&mxc_isi->slock, flags);
}


static int cap_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_dev *mxc_isi = q->drv_priv;
	struct mxc_isi_buffer *buf;
	struct vb2_buffer *vb2;
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	spin_lock_irqsave(&mxc_isi->slock, flags);

	mxc_isi->isi_cap.frame_count = 0;

	/* ISI channel output buffer 1 */
	buf = list_first_entry(&mxc_isi->isi_cap.out_pending,
					struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = 0;
	mxc_isi_channel_set_outbuf(mxc_isi, buf);
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(mxc_isi->isi_cap.out_pending.next, &mxc_isi->isi_cap.out_active);

	/* ISI channel output buffer 2 */
	buf = list_first_entry(&mxc_isi->isi_cap.out_pending,
					struct mxc_isi_buffer, list);
	buf->v4l2_buf.sequence = 1;
	mxc_isi_channel_set_outbuf(mxc_isi, buf);
	vb2 = &buf->v4l2_buf.vb2_buf;
	vb2->state = VB2_BUF_STATE_ACTIVE;
	list_move_tail(mxc_isi->isi_cap.out_pending.next, &mxc_isi->isi_cap.out_active);
	spin_unlock_irqrestore(&mxc_isi->slock, flags);

	return 0;
}


static void cap_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_dev *mxc_isi = q->drv_priv;
	struct mxc_isi_buffer *buf, *tmp;
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	mxc_isi_channel_disable(mxc_isi);

	spin_lock_irqsave(&mxc_isi->slock, flags);

	while (!list_empty(&mxc_isi->isi_cap.out_active)) {
		buf = list_entry(mxc_isi->isi_cap.out_active.next, struct mxc_isi_buffer, list);

		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	while (!list_empty(&mxc_isi->isi_cap.out_pending)) {
		buf = list_entry(mxc_isi->isi_cap.out_pending.next, struct mxc_isi_buffer, list);

		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(buf, tmp,
				&mxc_isi->isi_cap.out_active, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	list_for_each_entry_safe(buf, tmp,
				&mxc_isi->isi_cap.out_pending, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&mxc_isi->isi_cap.out_active);
	INIT_LIST_HEAD(&mxc_isi->isi_cap.out_pending);

	spin_unlock_irqrestore(&mxc_isi->slock, flags);
}

static struct vb2_ops mxc_cap_vb2_qops = {
	.queue_setup		= cap_vb2_queue_setup,
	.buf_prepare		= cap_vb2_buffer_prepare,
	.buf_queue			= cap_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= cap_vb2_start_streaming,
	.stop_streaming		= cap_vb2_stop_streaming,
};

/*
 * V4L2 controls handling
 */
#define ctrl_to_mxc_isi(__ctrl) \
	container_of((__ctrl)->handler, struct mxc_isi_dev, ctrls.handler)

static int mxc_isi_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct mxc_isi_dev *mxc_isi = ctrl_to_mxc_isi(ctrl);
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (ctrl->flags & V4L2_CTRL_FLAG_INACTIVE)
		return 0;

	spin_lock_irqsave(&mxc_isi->slock, flags);

	switch (ctrl->id) {
	case V4L2_CID_HFLIP:
		mxc_isi->hflip = ctrl->val;
		break;

	case V4L2_CID_VFLIP:
		mxc_isi->vflip = ctrl->val;
		break;

	case V4L2_CID_ALPHA_COMPONENT:
		mxc_isi->alpha = ctrl->val;
		break;
	}

	spin_unlock_irqrestore(&mxc_isi->slock, flags);

	return 0;
}

static const struct v4l2_ctrl_ops mxc_isi_ctrl_ops = {
	.s_ctrl = mxc_isi_s_ctrl,
};

int mxc_isi_ctrls_create(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_ctrls *ctrls = &mxc_isi->ctrls;
	struct v4l2_ctrl_handler *handler = &ctrls->handler;

	if (mxc_isi->ctrls.ready)
		return 0;

	v4l2_ctrl_handler_init(handler, 4);

	ctrls->hflip = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					V4L2_CID_HFLIP, 0, 1, 1, 0);
	ctrls->vflip = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					V4L2_CID_VFLIP, 0, 1, 1, 0);

	ctrls->alpha = v4l2_ctrl_new_std(handler, &mxc_isi_ctrl_ops,
					V4L2_CID_ALPHA_COMPONENT, 0, 0xff, 1, 0);

	if (!handler->error)
		ctrls->ready = true;

	return handler->error;
}

void mxc_isi_ctrls_delete(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_ctrls *ctrls = &mxc_isi->ctrls;

	if (ctrls->ready) {
		v4l2_ctrl_handler_free(&ctrls->handler);
		ctrls->ready = false;
		ctrls->alpha = NULL;
	}
}

static int mxc_isi_capture_open(struct file *file)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	int ret = -EBUSY;

	dev_dbg(&mxc_isi->pdev->dev, "%s, ISI%d\n", __func__, mxc_isi->id);

	mutex_lock(&mxc_isi->lock);
	ret = v4l2_fh_open(file);
	mutex_unlock(&mxc_isi->lock);

	mxc_isi_channel_init(mxc_isi);
	return 0;
}

static int mxc_isi_capture_release(struct file *file)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	int ret;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	mutex_lock(&mxc_isi->lock);
	ret = _vb2_fop_release(file, NULL);
	mutex_unlock(&mxc_isi->lock);
	mxc_isi_channel_deinit(mxc_isi);

	return ret;
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
 * Format and crop negotiation helpers
 */

/*
 * The video node ioctl operations
 */
static int mxc_isi_cap_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);

	strlcpy(cap->driver, MXC_ISI_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MXC_ISI_DRIVER_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s.%d",
		 dev_name(&mxc_isi->pdev->dev), mxc_isi->id);

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_CAPTURE_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mxc_isi_cap_enum_fmt_mplane(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct mxc_isi_fmt *fmt;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	if (f->index >= (int)ARRAY_SIZE(mxc_isi_out_formats))
		return -EINVAL;

	fmt = &mxc_isi_out_formats[f->index];
	if (!fmt)
		return -EINVAL;

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);

	f->pixelformat = fmt->fourcc;

	return 0;
}

static int mxc_isi_cap_g_fmt_mplane(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *dst_f = &mxc_isi->isi_cap.dst_f;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	pix->width = dst_f->o_width;
	pix->height = dst_f->o_height;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = dst_f->fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->num_planes = dst_f->fmt->memplanes;

	for (i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = dst_f->bytesperline[i];
		pix->plane_fmt[i].sizeimage = dst_f->sizeimage[i];
	}
	return 0;
}


static int mxc_isi_cap_try_fmt_mplane(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_fmt *fmt;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		fmt = &mxc_isi_out_formats[i];
		if (fmt->fourcc == pix->pixelformat && fmt->memplanes == pix->num_planes)
			break;
	}
	if (i >= ARRAY_SIZE(mxc_isi_out_formats)) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	return 0;
}


static struct media_pad *mxc_isi_get_remote_source_pad(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_cap_dev *isi_cap = &mxc_isi->isi_cap;
	struct v4l2_subdev *subdev = &isi_cap->sd;
	struct media_pad *sink_pad, *source_pad;
	int i;

	while (1) {
		source_pad = NULL;
		for (i = 0; i < subdev->entity.num_pads; i++) {
			sink_pad = &subdev->entity.pads[i];

			if (sink_pad->flags & MEDIA_PAD_FL_SINK) {
				source_pad = media_entity_remote_pad(sink_pad);
				if (source_pad)
					break;
			}
		}
		/* return first pad point in the loop  */
		return source_pad;
	}

	if (i == subdev->entity.num_pads)
		v4l2_err(mxc_isi->v4l2_dev, "%s, No remote pad found!\n", __func__);

	return NULL;
}

/* Update input frame size and formate  */
static int mxc_isi_source_fmt_init(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_frame *src_f = &mxc_isi->isi_cap.src_f;
	struct v4l2_subdev_format src_fmt;
	struct media_pad *source_pad;
	struct v4l2_subdev *src_sd;
	int ret;

	/* Get remote source pad */
	source_pad = mxc_isi_get_remote_source_pad(mxc_isi);
	if (source_pad == NULL) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, No remote pad found!\n", __func__);
		return -EINVAL;
	}

	/* Get remote source pad subdev */
	src_sd = media_entity_to_v4l2_subdev(source_pad->entity);
	if (src_sd == NULL) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, No remote subdev found!\n", __func__);
		return -EINVAL;
	}

	src_fmt.pad = source_pad->index;
	src_fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(src_sd, pad, get_fmt, NULL, &src_fmt);
	if (ret < 0 && ret != -ENOIOCTLCMD) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, get remote fmt faile!\n", __func__);
		return -EINVAL;
	}

	/* Pixel link master will transfer format to RGB32 or YUV32 */
	src_f->fmt = mxc_isi_get_src_fmt(&src_fmt);

	set_frame_bounds(src_f, src_fmt.format.width, src_fmt.format.height);

	return 0;
}

static int mxc_isi_cap_s_fmt_mplane(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *dst_f = &mxc_isi->isi_cap.dst_f;
	struct mxc_isi_fmt *fmt;
	int i;

	/* Step1: Check format with output support format list.
	 * Step2: Update output frame information.
	 * Step3: Checkout the format whether is supported by remote subdev
	 *	 Step3.1: If Yes, call remote subdev set_fmt.
	 *	 Step3.2: If NO, call remote subdev get_fmt.
	 * Step4: Update input frame information.
	 * Step5: Update mxc isi channel configuration.
	 * */

	dev_dbg(&mxc_isi->pdev->dev, "%s, fmt=0x%X\n", __func__, pix->pixelformat);
	if (vb2_is_busy(&mxc_isi->isi_cap.vb2_q)) {
		return -EBUSY;
	}

	/* Check out put format */
	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		fmt = &mxc_isi_out_formats[i];
		if (fmt->fourcc == pix->pixelformat && fmt->memplanes == pix->num_planes)
			break;
	}
	if (i >= ARRAY_SIZE(mxc_isi_out_formats)) {
		dev_dbg(&mxc_isi->pdev->dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	/* update out put frame size and formate */
	if (pix->height <= 0 || pix->width <= 0)
		return -EINVAL;

	dst_f->fmt = fmt;
	dst_f->height = pix->height;
	dst_f->width = pix->width;

	if (pix->num_planes > 1) {
		for (i = 0; i < pix->num_planes; i++) {
			dst_f->bytesperline[i] = pix->plane_fmt[i].bytesperline;
			dst_f->sizeimage[i] = pix->plane_fmt[i].sizeimage;
		}
	} else {
		dst_f->bytesperline[0] = dst_f->width * dst_f->fmt->depth[0] / 8;
		dst_f->sizeimage[0] = dst_f->height * dst_f->bytesperline[0];
	}

	set_frame_bounds(dst_f, pix->width, pix->height);

	mxc_isi_source_fmt_init(mxc_isi);

	/* configure mxc isi channel */
	mxc_isi_channel_config(mxc_isi);

	return 0;
}

static int mxc_isi_cap_streamon(struct file *file, void *priv,
			     enum v4l2_buf_type type)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	int ret;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	mxc_isi_pipeline_enable(mxc_isi, 1);

	ret = vb2_ioctl_streamon(file, priv, type);

	mxc_isi_channel_enable(mxc_isi);

	return ret;
}

static int mxc_isi_cap_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	int ret;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	mxc_isi_pipeline_enable(mxc_isi, 0);

	ret = vb2_ioctl_streamoff(file, priv, type);
	mxc_isi_channel_disable(mxc_isi);
	return ret;
}

static int mxc_isi_cap_g_selection(struct file *file, void *fh,
				struct v4l2_selection *s)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct mxc_isi_frame *f = &mxc_isi->isi_cap.src_f;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_COMPOSE_DEFAULT:
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &mxc_isi->isi_cap.dst_f;
	case V4L2_SEL_TGT_CROP_BOUNDS:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = f->o_width;
		s->r.height = f->o_height;
		return 0;

	case V4L2_SEL_TGT_COMPOSE:
		f = &mxc_isi->isi_cap.dst_f;
	case V4L2_SEL_TGT_CROP:
		s->r.left = f->h_off;
		s->r.top = f->v_off;
		s->r.width = f->width;
		s->r.height = f->height;
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
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct mxc_isi_frame *f;
	struct v4l2_rect rect = s->r;
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	if (s->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	if (s->target == V4L2_SEL_TGT_COMPOSE)
		f = &mxc_isi->isi_cap.dst_f;
	else if (s->target == V4L2_SEL_TGT_CROP)
		f = &mxc_isi->isi_cap.src_f;
	else
		return -EINVAL;

	if (s->flags & V4L2_SEL_FLAG_LE &&
	    !enclosed_rectangle(&rect, &s->r))
		return -ERANGE;

	if (s->flags & V4L2_SEL_FLAG_GE &&
	    !enclosed_rectangle(&s->r, &rect))
		return -ERANGE;

	s->r = rect;
	spin_lock_irqsave(&mxc_isi->slock, flags);
	set_frame_crop(f, s->r.left, s->r.top, s->r.width,
		       s->r.height);
	spin_unlock_irqrestore(&mxc_isi->slock, flags);

	return 0;
}

static struct v4l2_subdev *mxc_isi_get_subdev_by_name(struct v4l2_device *v4l2,
			const char *name)
{
	struct v4l2_subdev *sd;
	bool found = false;

	list_for_each_entry(sd, &v4l2->subdevs, list) {
		if (strstr(sd->name, name) != NULL) {
			found = true;
			break;
		}
	}

	return (found) ? sd : NULL;
}

static int mxc_isi_cap_g_chip_ident(struct file *file, void *fb,
			struct v4l2_dbg_chip_ident *chip)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_device *v4l2_dev = mxc_isi->isi_cap.sd.v4l2_dev;
	struct video_device *vdev = video_devdata(file);
	struct sensor_data *max9286;
	struct v4l2_subdev *sd;

	sd = mxc_isi_get_subdev_by_name(v4l2_dev, "max9286_mipi");
	if (sd == NULL) {
		v4l2_err(&mxc_isi->isi_cap.sd, "Can't find sub device\n");
		return -ENODEV;
	}

	max9286 = container_of(sd, struct sensor_data, subdev);
	if (max9286->sensor_is_there & (0x1 << vdev->num))
		sprintf(chip->match.name, "max9286_mipi%d\n", vdev->num);
	else
		return -ENODEV;

	return 0;
}

static int mxc_isi_cap_g_parm(struct file *file, void *fh,
			struct v4l2_streamparm *a)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_device *v4l2_dev = mxc_isi->isi_cap.sd.v4l2_dev;
	struct v4l2_subdev *sd;

	sd = mxc_isi_get_subdev_by_name(v4l2_dev, "max9286_mipi");
	if (sd == NULL) {
		v4l2_err(&mxc_isi->isi_cap.sd, "Can't find subdev\n");
		return -ENODEV;
	}
	return v4l2_subdev_call(sd, video, g_parm, a);
}

static int mxc_isi_cap_s_parm(struct file *file, void *fh,
			struct v4l2_streamparm *a)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_device *v4l2_dev = mxc_isi->isi_cap.sd.v4l2_dev;
	struct v4l2_subdev *sd;

	sd = mxc_isi_get_subdev_by_name(v4l2_dev, "max9286_mipi");
	if (sd == NULL) {
		v4l2_err(&mxc_isi->isi_cap.sd, "Can't find subdev\n");
		return -ENODEV;
	}
	return v4l2_subdev_call(sd, video, s_parm, a);
}

static const struct v4l2_ioctl_ops mxc_isi_capture_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_cap_querycap,

	.vidioc_enum_fmt_vid_cap_mplane	= mxc_isi_cap_enum_fmt_mplane,
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

	.vidioc_streamon		= mxc_isi_cap_streamon,
	.vidioc_streamoff		= mxc_isi_cap_streamoff,

	.vidioc_g_selection		= mxc_isi_cap_g_selection,
	.vidioc_s_selection		= mxc_isi_cap_s_selection,
	.vidioc_g_chip_ident	= mxc_isi_cap_g_chip_ident,

	.vidioc_g_parm			= mxc_isi_cap_g_parm,
	.vidioc_s_parm			= mxc_isi_cap_s_parm,
};

/* Capture subdev media entity operations */
static int mxc_isi_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);

	if (WARN_ON(mxc_isi == NULL))
		return 0;

	if (!(flags & MEDIA_LNK_FL_ENABLED)) {
		return 0;
	}
	/* TODO */
	/* Add ISI source and sink pad link configuration */
	if (local->flags & MEDIA_PAD_FL_SOURCE) {
		switch (local->index) {
		case MXC_ISI_SD_PAD_SOURCE_DC0:
		case MXC_ISI_SD_PAD_SOURCE_DC1:
			break;
		case MXC_ISI_SD_PAD_SOURCE_MEM:
		default:
			return 0;
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
			break;
		}
	}

	return 0;
}

static const struct media_entity_operations mxc_isi_sd_media_ops = {
	.link_setup = mxc_isi_link_setup,
};

static int mxc_isi_subdev_enum_mbus_code(struct v4l2_subdev *sd,
				      struct v4l2_subdev_pad_config *cfg,
				      struct v4l2_subdev_mbus_code_enum *code)
{
	return 0;
}

static int mxc_isi_subdev_get_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f;
	struct v4l2_mbus_framefmt *mf;

	mutex_lock(&mxc_isi->lock);

	switch (fmt->pad) {
	case MXC_ISI_SD_PAD_SOURCE_MEM:
	case MXC_ISI_SD_PAD_SOURCE_DC0:
	case MXC_ISI_SD_PAD_SOURCE_DC1:
		f = &mxc_isi->isi_cap.dst_f;
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
		f = &mxc_isi->isi_cap.src_f;
		break;
	default:
		mutex_unlock(&mxc_isi->lock);
		v4l2_err(mxc_isi->v4l2_dev, "%s, Pad is not support now!\n", __func__);
		return -1;
	}

	if (!WARN_ON(f->fmt == NULL))
		mf->code = f->fmt->mbus_code;

	/* Source/Sink pads crop rectangle size */
	mf->width = f->width;
	mf->height = f->height;

	mutex_unlock(&mxc_isi->lock);
	mf->colorspace = V4L2_COLORSPACE_JPEG;

	return 0;
}

static int mxc_isi_subdev_set_fmt(struct v4l2_subdev *sd,
			       struct v4l2_subdev_pad_config *cfg,
			       struct v4l2_subdev_format *fmt)
{
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *mf = &fmt->format;
	struct mxc_isi_frame *dst_f = &mxc_isi->isi_cap.dst_f;
	struct mxc_isi_fmt *out_fmt;
	int i;

	if (fmt->pad < MXC_ISI_SD_PAD_SOURCE_MEM &&
					vb2_is_busy(&mxc_isi->isi_cap.vb2_q))
		return -EBUSY;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		out_fmt = &mxc_isi_out_formats[i];
		if (mf->code == out_fmt->mbus_code)
			break;
	}
	if (i >= ARRAY_SIZE(mxc_isi_out_formats)) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&mxc_isi->lock);
	/* update out put frame size and formate */
	dst_f->fmt = &mxc_isi_out_formats[i];
	set_frame_bounds(dst_f, mf->width, mf->height);
	mutex_unlock(&mxc_isi->lock);

	dev_dbg(&mxc_isi->pdev->dev, "pad%d: code: 0x%x, %dx%d",
	    fmt->pad, mf->code, mf->width, mf->height);

	return 0;
}

static int mxc_isi_subdev_get_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &mxc_isi->isi_cap.src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;

	mutex_lock(&mxc_isi->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_COMPOSE_BOUNDS:
		f = &mxc_isi->isi_cap.dst_f;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		r->width = f->o_width;
		r->height = f->o_height;
		r->left = 0;
		r->top = 0;
		mutex_unlock(&mxc_isi->lock);
		return 0;

	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &mxc_isi->isi_cap.dst_f;
		break;
	default:
		mutex_unlock(&mxc_isi->lock);
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

	dev_dbg(&mxc_isi->pdev->dev, "%s, target %#x: l:%d, t:%d, %dx%d, f_w: %d, f_h: %d",
			__func__, sel->pad, r->left, r->top, r->width, r->height,
			f->c_width, f->c_height);

	mutex_unlock(&mxc_isi->lock);
	return 0;
}

static int mxc_isi_subdev_set_selection(struct v4l2_subdev *sd,
				     struct v4l2_subdev_pad_config *cfg,
				     struct v4l2_subdev_selection *sel)
{
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);
	struct mxc_isi_frame *f = &mxc_isi->isi_cap.src_f;
	struct v4l2_rect *r = &sel->r;
	struct v4l2_rect *try_sel;
	unsigned long flags;

	mutex_lock(&mxc_isi->lock);

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:
		try_sel = v4l2_subdev_get_try_crop(sd, cfg, sel->pad);
		break;
	case V4L2_SEL_TGT_COMPOSE:
		try_sel = v4l2_subdev_get_try_compose(sd, cfg, sel->pad);
		f = &mxc_isi->isi_cap.dst_f;
		break;
	default:
		mutex_unlock(&mxc_isi->lock);
		return -EINVAL;
	}

	if (sel->which == V4L2_SUBDEV_FORMAT_TRY) {
		*try_sel = sel->r;
	} else {
		spin_lock_irqsave(&mxc_isi->slock, flags);
		set_frame_crop(f, r->left, r->top, r->width, r->height);
		spin_unlock_irqrestore(&mxc_isi->slock, flags);
	}

	dev_dbg(&mxc_isi->pdev->dev, "%s, target %#x: (%d,%d)/%dx%d", __func__,
			sel->target, r->left, r->top, r->width, r->height);

	mutex_unlock(&mxc_isi->lock);

	return 0;
}

static struct v4l2_subdev_pad_ops mxc_isi_subdev_pad_ops = {
	.enum_mbus_code = mxc_isi_subdev_enum_mbus_code,
	.get_selection = mxc_isi_subdev_get_selection,
	.set_selection = mxc_isi_subdev_set_selection,
	.get_fmt = mxc_isi_subdev_get_fmt,
	.set_fmt = mxc_isi_subdev_set_fmt,
};

static struct v4l2_subdev_ops mxc_isi_subdev_ops = {
	.pad = &mxc_isi_subdev_pad_ops,
};

static int mxc_isi_register_cap_device(struct mxc_isi_dev *mxc_isi,
				 struct v4l2_device *v4l2_dev)
{
	struct video_device *vdev = &mxc_isi->isi_cap.vdev;
	struct vb2_queue *q = &mxc_isi->isi_cap.vb2_q;
	struct mxc_isi_cap_dev *isi_cap = &mxc_isi->isi_cap;
	int ret = -ENOMEM;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.%d.capture", mxc_isi->id);
	memset(vdev, 0, sizeof(*vdev));

	vdev->fops	= &mxc_isi_capture_fops;
	vdev->ioctl_ops	= &mxc_isi_capture_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->queue	= q;
	vdev->lock	= &mxc_isi->lock;

	video_set_drvdata(vdev, mxc_isi);

	INIT_LIST_HEAD(&mxc_isi->isi_cap.out_pending);
	INIT_LIST_HEAD(&mxc_isi->isi_cap.out_active);

	memset(q, 0, sizeof(*q));
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	q->drv_priv = mxc_isi;
	q->ops = &mxc_cap_vb2_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct mxc_isi_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &mxc_isi->lock;

	ret = vb2_queue_init(q);
	if (ret)
		goto err_free_ctx;

	/* Default configuration  */
	isi_cap->dst_f.width = 1280;
	isi_cap->dst_f.height = 800;
	isi_cap->dst_f.fmt = &mxc_isi_out_formats[0];;
	isi_cap->src_f.fmt = isi_cap->dst_f.fmt;

	isi_cap->cap_pad.flags = MEDIA_PAD_FL_SINK;
	vdev->entity.function = MEDIA_ENT_F_PROC_VIDEO_SCALER;
	ret = media_entity_pads_init(&vdev->entity, 1, &isi_cap->cap_pad);
	if (ret)
		goto err_free_ctx;

	ret = mxc_isi_ctrls_create(mxc_isi);
	if (ret)
		goto err_me_cleanup;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret)
		goto err_ctrl_free;

	vdev->ctrl_handler = &mxc_isi->ctrls.handler;
	v4l2_info(v4l2_dev, "Registered %s as /dev/%s\n",
		  vdev->name, video_device_node_name(vdev));

	return 0;

err_ctrl_free:
	mxc_isi_ctrls_delete(mxc_isi);
err_me_cleanup:
	media_entity_cleanup(&vdev->entity);
err_free_ctx:
	return ret;
}

static int mxc_isi_capture_subdev_registered(struct v4l2_subdev *sd)
{
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);
	int ret;

	if (mxc_isi == NULL)
		return -ENXIO;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
#if 0
	if (mxc_isi->id == 0) {
		/* ISI channel 0 support source input image from memory  */
		ret = mxc_isi_register_m2m_device(mxc_isi, sd->v4l2_dev);
		if (ret < 0)
			return ret;
	}
#endif

	ret = mxc_isi_register_cap_device(mxc_isi, sd->v4l2_dev);
	if (ret < 0)
		return ret;

	return ret;
}

static void mxc_isi_capture_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct mxc_isi_dev *mxc_isi = v4l2_get_subdevdata(sd);
	struct video_device *vdev;

	if (mxc_isi == NULL)
		return;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	mutex_lock(&mxc_isi->lock);

	vdev = &mxc_isi->isi_cap.vdev;
	if (video_is_registered(vdev)) {
		video_unregister_device(vdev);
		mxc_isi_ctrls_delete(mxc_isi);
		media_entity_cleanup(&vdev->entity);
	}

	mutex_unlock(&mxc_isi->lock);
}

static const struct v4l2_subdev_internal_ops mxc_isi_capture_sd_internal_ops = {
	.registered = mxc_isi_capture_subdev_registered,
	.unregistered = mxc_isi_capture_subdev_unregistered,
};

int mxc_isi_initialize_capture_subdev(struct mxc_isi_dev *mxc_isi)
{
	struct v4l2_subdev *sd = &mxc_isi->isi_cap.sd;
	int ret;

	v4l2_subdev_init(sd, &mxc_isi_subdev_ops);
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, sizeof(sd->name), "mxc_isi.%d", mxc_isi->id);

	sd->entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;

	/* ISI Sink pads */
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC0].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC1].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC2].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI0_VC3].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC0].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC1].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC2].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MIPI1_VC3].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_DC0].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_DC1].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_HDMI].flags = MEDIA_PAD_FL_SINK;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SINK_MEM].flags = MEDIA_PAD_FL_SINK;

	/* ISI source pads */
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SOURCE_MEM].flags = MEDIA_PAD_FL_SOURCE;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SOURCE_DC0].flags = MEDIA_PAD_FL_SOURCE;
	mxc_isi->isi_cap.sd_pads[MXC_ISI_SD_PAD_SOURCE_DC1].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&sd->entity, MXC_ISI_SD_PADS_NUM,
				mxc_isi->isi_cap.sd_pads);
	if (ret)
		return ret;

	sd->entity.ops = &mxc_isi_sd_media_ops;
	sd->internal_ops = &mxc_isi_capture_sd_internal_ops;
	v4l2_set_subdevdata(sd, mxc_isi);

	return 0;
}

void mxc_isi_unregister_capture_subdev(struct mxc_isi_dev *mxc_isi)
{
	struct v4l2_subdev *sd = &mxc_isi->isi_cap.sd;

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_set_subdevdata(sd, NULL);
}
