/*
 * Copyright 2017-2018 NXP
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
#include <linux/of_graph.h>

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

#define to_isi_buffer(x)	\
	container_of((x), struct mxc_isi_buffer, v4l2_buf)

#define file_to_ctx(file)		\
	container_of(file->private_data, struct mxc_isi_ctx, fh);

extern struct mxc_isi_fmt mxc_isi_out_formats[8];

struct mxc_isi_fmt mxc_isi_input_formats[] = {
	/* Pixel link input format */
	{
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_XRGB32,
		.depth		= { 32 },
		.color =	MXC_ISI_M2M_IN_FMT_XRGB8,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "BGR32",
		.fourcc		= V4L2_PIX_FMT_BGR32,
		.depth		= { 32 },
		.color =	MXC_ISI_M2M_IN_FMT_XBGR8,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "RGB565",
		.fourcc		= V4L2_PIX_FMT_RGB565,
		.depth		= { 16 },
		.color =	MXC_ISI_M2M_IN_FMT_RGB565,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV32 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= { 32 },
		.color = MXC_ISI_M2M_IN_FMT_YUV444_1P8P,
		.memplanes	= 1,
		.colplanes	= 1,
	}, {
		.name		= "YUV16 (X-Y-U-V)",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= { 16 },
		.color = MXC_ISI_M2M_IN_FMT_YUV422_1P8P,
		.memplanes	= 1,
		.colplanes	= 1,
	}
};

static struct mxc_isi_buffer *vb2_to_isi_buffer(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *v4l2_buf;
	struct mxc_isi_buffer *buf;

	v4l2_buf = to_vb2_v4l2_buffer(vb2);
	buf = to_isi_buffer(v4l2_buf);

	return buf;
}

static struct v4l2_m2m_buffer *to_v4l2_m2m_buffer(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2);
	struct v4l2_m2m_buffer *b;

	b = container_of(vbuf, struct v4l2_m2m_buffer, vb);
	return b;
}

static void mxc_isi_m2m_device_run(void *priv)
{
	struct mxc_isi_ctx *mxc_ctx = priv;
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	struct v4l2_fh *fh = &mxc_ctx->fh;
	struct vb2_buffer *src_vb2;
	struct mxc_isi_buffer *src_buf;
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s enter\n", __func__);

	spin_lock_irqsave(&mxc_isi->slock, flags);

	/* SRC */
	src_vb2 = v4l2_m2m_next_src_buf(fh->m2m_ctx);
	if (!src_vb2) {
		dev_err(&mxc_isi->pdev->dev, "Null src buf\n");
		goto unlock;
	}

	src_buf = vb2_to_isi_buffer(src_vb2);


	mxc_isi_channel_set_m2m_src_addr(mxc_isi, src_buf);
	mxc_isi_m2m_channel_enable(mxc_isi);

unlock:
	spin_unlock_irqrestore(&mxc_isi->slock, flags);
	msleep(50);
}

static int mxc_isi_m2m_job_ready(void *priv)
{
	struct mxc_isi_ctx *mxc_ctx = priv;
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	struct v4l2_fh *fh = &mxc_ctx->fh;
	unsigned int num_src_bufs_ready;
	unsigned int num_dst_bufs_ready;
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	spin_lock_irqsave(&mxc_isi->slock, flags);
	num_src_bufs_ready = v4l2_m2m_num_src_bufs_ready(fh->m2m_ctx);
	num_dst_bufs_ready = v4l2_m2m_num_dst_bufs_ready(fh->m2m_ctx);
	spin_unlock_irqrestore(&mxc_isi->slock, flags);

	if (num_src_bufs_ready >= 1 && num_dst_bufs_ready >= 1)
		return 1;
	return 0;
}

static void mxc_isi_m2m_job_abort(void *priv)
{
	struct mxc_isi_ctx *mxc_ctx = priv;
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;

	mxc_isi->m2m.aborting = 1;
	dev_dbg(&mxc_isi->pdev->dev, "Abort requested\n");
}

static struct v4l2_m2m_ops mxc_isi_m2m_ops = {
	.device_run = mxc_isi_m2m_device_run,
	.job_ready  = mxc_isi_m2m_job_ready,
	.job_abort  = mxc_isi_m2m_job_abort,
};

static int m2m_vb2_queue_setup(struct vb2_queue *q,
		       unsigned int *num_buffers, unsigned int *num_planes,
		       unsigned int sizes[], struct device *alloc_devs[])
{
	struct mxc_isi_ctx *mxc_ctx = vb2_get_drv_priv(q);
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	struct mxc_isi_frame *frame;
	struct mxc_isi_fmt *fmt;
	unsigned long wh;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (*num_buffers < 1) {
		dev_err(&mxc_isi->pdev->dev, "%s at least need one buffer\n", __func__);
		return -EINVAL;
	}

	if (q->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		frame = &mxc_isi->m2m.dst_f;
	else
		frame = &mxc_isi->m2m.src_f;

	fmt = frame->fmt;
	if (fmt == NULL)
		return -EINVAL;

	for (i = 0; i < fmt->memplanes; i++)
		alloc_devs[i] = &mxc_isi->pdev->dev;

	*num_planes = fmt->memplanes;
	wh = frame->width * frame->height;

	for (i = 0; i < fmt->memplanes; i++) {
		unsigned int size = (wh * fmt->depth[i]) >> 3;

		if (i == 1 && fmt->fourcc == V4L2_PIX_FMT_NV12)
			size >>= 1;
		sizes[i] = max_t(u32, size, frame->sizeimage[i]);

		dev_dbg(&mxc_isi->pdev->dev, "%s, buf_n=%d, planes[%d]->size=%d\n",
					__func__,  *num_buffers, i, sizes[i]);
	}

	return 0;
}

static int m2m_vb2_buffer_prepare(struct vb2_buffer *vb2)
{
	struct vb2_queue *vq = vb2->vb2_queue;
	struct mxc_isi_ctx *mxc_ctx = vb2_get_drv_priv(vq);
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	struct mxc_isi_frame *frame;
	int i;

	if (vq->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		frame = &mxc_isi->m2m.dst_f;
	else
		frame = &mxc_isi->m2m.src_f;

	if (frame == NULL)
		return -EINVAL;

	for (i = 0; i < frame->fmt->memplanes; i++) {
		unsigned long size = frame->sizeimage[i];

		if (vb2_plane_size(vb2, i) < size) {
			dev_err(&mxc_isi->pdev->dev,
				 "User buffer too small (%ld < %ld)\n",
				 vb2_plane_size(vb2, i), size);
			return -EINVAL;
		}
		vb2_set_plane_payload(vb2, i, size);
	}

	return 0;
}

static void m2m_vb2_buffer_queue(struct vb2_buffer *vb2)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb2);
	struct mxc_isi_ctx *mxc_ctx = vb2_get_drv_priv(vb2->vb2_queue);
	struct v4l2_fh *fh = &mxc_ctx->fh;

	v4l2_m2m_buf_queue(fh->m2m_ctx, vbuf);
}

static int m2m_vb2_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_isi_ctx *mxc_ctx = vb2_get_drv_priv(q);
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	struct v4l2_fh *fh = &mxc_ctx->fh;
	struct vb2_buffer *dst_vb2;
	struct  v4l2_m2m_buffer *b;
	struct mxc_isi_buffer *dst_buf;
	unsigned long flags;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (V4L2_TYPE_IS_OUTPUT(q->type))
		return 0;

	if (count < 2) {
		dev_err(&mxc_isi->pdev->dev, "Need to at leas 2 buffers\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&mxc_isi->slock, flags);

	/* BUF1 */
	dst_vb2 = v4l2_m2m_next_dst_buf(fh->m2m_ctx);
	if (!dst_vb2) {
		dev_err(&mxc_isi->pdev->dev, "%d: Null dst buf\n", __LINE__);
		goto unlock;
	}
	dst_vb2->state = VB2_BUF_STATE_ACTIVE;
	dst_buf = vb2_to_isi_buffer(dst_vb2);
	dst_buf->v4l2_buf.sequence = 0;
	mxc_isi_channel_set_m2m_out_addr(mxc_isi, dst_buf);
	v4l2_m2m_dst_buf_remove(fh->m2m_ctx);
	b = to_v4l2_m2m_buffer(dst_vb2);
	list_add_tail(&b->list, &mxc_isi->m2m.out_active);

	/* BUF2 */
	dst_vb2 = v4l2_m2m_next_dst_buf(fh->m2m_ctx);
	if (!dst_vb2) {
		dev_err(&mxc_isi->pdev->dev, "%d: Null dst buf\n", __LINE__);
		goto unlock;
	}
	dst_vb2->state = VB2_BUF_STATE_ACTIVE;
	dst_buf = vb2_to_isi_buffer(dst_vb2);
	dst_buf->v4l2_buf.sequence = 1;
	mxc_isi_channel_set_m2m_out_addr(mxc_isi, dst_buf);
	v4l2_m2m_dst_buf_remove(fh->m2m_ctx);
	b = to_v4l2_m2m_buffer(dst_vb2);
	list_add_tail(&b->list, &mxc_isi->m2m.out_active);

	mxc_isi->m2m.frame_count = 1;
unlock:
	spin_unlock_irqrestore(&mxc_isi->slock, flags);

	return 0;
}

static void m2m_vb2_stop_streaming(struct vb2_queue *q)
{
	struct mxc_isi_ctx *mxc_ctx = vb2_get_drv_priv(q);
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	struct vb2_v4l2_buffer *vb2;
	struct mxc_isi_buffer *buf;
	unsigned long flags;

	spin_lock_irqsave(&mxc_isi->slock, flags);

	while ((vb2 = v4l2_m2m_src_buf_remove(mxc_ctx->fh.m2m_ctx)) != NULL)
		v4l2_m2m_buf_done(vb2, VB2_BUF_STATE_ERROR);

	while ((vb2 = v4l2_m2m_dst_buf_remove(mxc_ctx->fh.m2m_ctx)) != NULL)
		v4l2_m2m_buf_done(vb2, VB2_BUF_STATE_ERROR);

	while (!list_empty(&mxc_isi->m2m.out_active)) {
		buf = list_entry(mxc_isi->m2m.out_active.next, struct mxc_isi_buffer, list);
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	INIT_LIST_HEAD(&mxc_isi->isi_cap.out_active);

	spin_unlock_irqrestore(&mxc_isi->slock, flags);
}

static struct vb2_ops mxc_m2m_vb2_qops = {
	.queue_setup		= m2m_vb2_queue_setup,
	.buf_prepare		= m2m_vb2_buffer_prepare,
	.buf_queue			= m2m_vb2_buffer_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= m2m_vb2_start_streaming,
	.stop_streaming		= m2m_vb2_stop_streaming,
};


static int mxc_m2m_queue_init(void *priv, struct vb2_queue *src_vq,
			       struct vb2_queue *dst_vq)
{
	struct mxc_isi_ctx *mxc_ctx = priv;
	struct mxc_isi_dev *mxc_isi = mxc_ctx->isi_dev;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	src_vq->drv_priv = mxc_ctx;
	src_vq->buf_struct_size = sizeof(struct mxc_isi_buffer);
	src_vq->ops = &mxc_m2m_vb2_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &mxc_isi->lock;
	src_vq->dev = &mxc_isi->pdev->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	dst_vq->drv_priv = mxc_ctx;
	dst_vq->buf_struct_size = sizeof(struct mxc_isi_buffer);
	dst_vq->ops = &mxc_m2m_vb2_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &mxc_isi->lock;
	dst_vq->dev = &mxc_isi->pdev->dev;

	ret = vb2_queue_init(dst_vq);
	return ret;
}

static int mxc_isi_m2m_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct mxc_isi_m2m_dev *isi_m2m = &mxc_isi->m2m;
	struct device *dev = &mxc_isi->pdev->dev;
	struct mxc_isi_ctx *mxc_ctx = NULL;
	int ret = 0;

	dev_dbg(dev, "%s, ISI%d\n", __func__, mxc_isi->id);

	if (atomic_read(&mxc_isi->open_count) > 0) {
		dev_err(dev, "%s: ISI channel[%d] is busy\n", __func__, mxc_isi->id);
		return -EBUSY;
	}

	if (mutex_lock_interruptible(&mxc_isi->lock))
		return -ERESTARTSYS;
	mxc_ctx = kzalloc(sizeof(*mxc_ctx), GFP_KERNEL);
	if (!mxc_ctx) {
		ret = -ENOMEM;
		goto unlock;
	}

	mxc_ctx->isi_dev = mxc_isi;

	v4l2_fh_init(&mxc_ctx->fh, vdev);
	file->private_data = &mxc_ctx->fh;

	mxc_ctx->fh.m2m_ctx =
		v4l2_m2m_ctx_init(isi_m2m->m2m_dev, mxc_ctx, mxc_m2m_queue_init);
	if (IS_ERR(mxc_ctx->fh.m2m_ctx)) {
		dev_err(dev, "%s v4l2_m2m_ctx_init fail\n", __func__);
		ret = PTR_ERR(mxc_ctx->fh.m2m_ctx);
		v4l2_fh_exit(&mxc_ctx->fh);
		kfree(mxc_ctx);
		goto unlock;
	}
	v4l2_fh_add(&mxc_ctx->fh);

	pm_runtime_get_sync(dev);
	if (atomic_inc_return(&mxc_isi->open_count) == 1)
		mxc_isi_m2m_channel_init(mxc_isi);

	mxc_isi->is_m2m = 1;
unlock:
	mutex_unlock(&mxc_isi->lock);
	return ret;
}

static int mxc_isi_m2m_release(struct file *file)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct device *dev = &mxc_isi->pdev->dev;
	struct mxc_isi_ctx *mxc_ctx = file_to_ctx(file);

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	v4l2_fh_del(&mxc_ctx->fh);
	v4l2_fh_exit(&mxc_ctx->fh);

	mutex_lock(&mxc_isi->lock);
	v4l2_m2m_ctx_release(mxc_ctx->fh.m2m_ctx);
	mutex_unlock(&mxc_isi->lock);

	kfree(mxc_ctx);
	if (atomic_dec_and_test(&mxc_isi->open_count))
		mxc_isi_channel_deinit(mxc_isi);

	mxc_isi->is_m2m = 0;
	pm_runtime_put(dev);
	return 0;
}

static const struct v4l2_file_operations mxc_isi_m2m_fops = {
	.owner			= THIS_MODULE,
	.open			= mxc_isi_m2m_open,
	.release		= mxc_isi_m2m_release,
	.poll			= v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap			= v4l2_m2m_fop_mmap,
};

static int mxc_isi_m2m_querycap(struct file *file, void *priv,
					struct v4l2_capability *cap)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);

	strlcpy(cap->driver, MXC_ISI_DRIVER_NAME, sizeof(cap->driver));
	strlcpy(cap->card, MXC_ISI_DRIVER_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s.%d",
		 dev_name(&mxc_isi->pdev->dev), mxc_isi->id);

	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE |
		V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_VIDEO_OUTPUT_MPLANE;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int mxc_isi_m2m_enum_fmt_vid_out(struct file *file, void *priv,
				    struct v4l2_fmtdesc *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct mxc_isi_fmt *fmt;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	if (f->index >= (int)ARRAY_SIZE(mxc_isi_input_formats))
		return -EINVAL;

	fmt = &mxc_isi_input_formats[f->index];
	if (!fmt)
		return -EINVAL;

	strncpy(f->description, fmt->name, sizeof(f->description) - 1);

	f->pixelformat = fmt->fourcc;

	return 0;
}

static int mxc_isi_m2m_enum_fmt_vid_cap(struct file *file, void *priv,
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

static int mxc_isi_m2m_try_fmt_vid_out(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_fmt *fmt = NULL;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_input_formats); i++) {
		fmt = &mxc_isi_input_formats[i];
		if (fmt->fourcc == pix->pixelformat)
			break;
	}

	if (i >= ARRAY_SIZE(mxc_isi_input_formats)) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	if (pix->width <= 0 || pix->height <= 0) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, width %d, height %d is not valid\n"
				, __func__, pix->width, pix->height);
		return -EINVAL;
	}

	return 0;
}

static int mxc_isi_m2m_try_fmt_vid_cap(struct file *file, void *fh,
				   struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_fmt *fmt = NULL;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		fmt = &mxc_isi_out_formats[i];
		if (fmt->fourcc == pix->pixelformat)
			break;
	}

	if (i >= ARRAY_SIZE(mxc_isi_out_formats)) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	if (pix->width <= 0 || pix->height <= 0) {
		v4l2_err(mxc_isi->v4l2_dev, "%s, width %d, height %d is not valid\n"
				, __func__, pix->width, pix->height);
		return -EINVAL;
	}

	return 0;
}

static int mxc_isi_m2m_s_fmt_vid_out(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct mxc_isi_frame *frame = &mxc_isi->m2m.src_f;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_fmt *fmt;
	struct vb2_queue *vq;
	int bpl, i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	vq = v4l2_m2m_get_vq(fh->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		dev_err(&mxc_isi->pdev->dev, "queue busy\n");
		return -EBUSY;
	}

	for (i = 0; i < ARRAY_SIZE(mxc_isi_input_formats); i++) {
		fmt = &mxc_isi_input_formats[i];
		if (pix && fmt->fourcc == pix->pixelformat)
			break;
	}

	if (i >= ARRAY_SIZE(mxc_isi_input_formats)) {
		dev_dbg(&mxc_isi->pdev->dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	/* update out put frame size and formate */
	if (pix->height <= 0 || pix->width <= 0)
		return -EINVAL;

	frame->fmt = fmt;
	frame->height = pix->height;
	frame->width = pix->width;

	pix->num_planes = fmt->memplanes;
	for (i = 0; i < pix->num_planes; i++) {
		bpl = pix->plane_fmt[i].bytesperline;

		if ((bpl == 0) || (bpl / (fmt->depth[i] >> 3)) < pix->width)
			pix->plane_fmt[i].bytesperline =
						(pix->width * fmt->depth[i]) >> 3;

		if (pix->plane_fmt[i].sizeimage == 0)
			pix->plane_fmt[i].sizeimage = (pix->width * pix->height *
						fmt->depth[i] >> 3);
	}

	frame->bytesperline[0] = frame->width * frame->fmt->depth[0] / 8;
	frame->sizeimage[0] = frame->height * frame->bytesperline[0];

	set_frame_bounds(frame, pix->width, pix->height);
	mxc_isi_m2m_config_src(mxc_isi);

	return 0;
}

static int mxc_isi_m2m_s_fmt_vid_cap(struct file *file, void *priv,
				 struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_fh *fh = file->private_data;
	struct mxc_isi_frame *frame = &mxc_isi->m2m.dst_f;
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_fmt *fmt;
	struct vb2_queue *vq;
	int bpl, i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	vq = v4l2_m2m_get_vq(fh->m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	if (vb2_is_busy(vq)) {
		dev_err(&mxc_isi->pdev->dev, "queue busy\n");
		return -EBUSY;
	}

	for (i = 0; i < ARRAY_SIZE(mxc_isi_out_formats); i++) {
		fmt = &mxc_isi_out_formats[i];
		if (pix && fmt->fourcc == pix->pixelformat)
			break;
	}

	if (i >= ARRAY_SIZE(mxc_isi_out_formats)) {
		dev_dbg(&mxc_isi->pdev->dev, "%s, format is not support!\n", __func__);
		return -EINVAL;
	}

	/* update out put frame size and formate */
	if (pix->height <= 0 || pix->width <= 0)
		return -EINVAL;

	frame->fmt = fmt;
	frame->height = pix->height;
	frame->width = pix->width;

	pix->num_planes = fmt->memplanes;
	for (i = 0; i < pix->num_planes; i++) {
		bpl = pix->plane_fmt[i].bytesperline;

		if ((bpl == 0) || (bpl / (fmt->depth[i] >> 3)) < pix->width)
			pix->plane_fmt[i].bytesperline =
						(pix->width * fmt->depth[i]) >> 3;

		if (pix->plane_fmt[i].sizeimage == 0) {

			if ((i == 1) && (pix->pixelformat == V4L2_PIX_FMT_NV12))
				pix->plane_fmt[i].sizeimage =
					(pix->width * (pix->height >> 1) * fmt->depth[i] >> 3);
			else
				pix->plane_fmt[i].sizeimage = (pix->width * pix->height *
						fmt->depth[i] >> 3);
		}
	}

	if (pix->num_planes > 1) {
		for (i = 0; i < pix->num_planes; i++) {
			frame->bytesperline[i] = pix->plane_fmt[i].bytesperline;
			frame->sizeimage[i] = pix->plane_fmt[i].sizeimage;
		}
	} else {
		frame->bytesperline[0] = frame->width * frame->fmt->depth[0] / 8;
		frame->sizeimage[0] = frame->height * frame->bytesperline[0];
	}

	memcpy(&mxc_isi->pix, pix, sizeof(*pix));

	set_frame_bounds(frame, pix->width, pix->height);
	mxc_isi_m2m_config_dst(mxc_isi);

	return 0;
}

static int mxc_isi_m2m_g_fmt_vid_cap(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *frame = &mxc_isi->m2m.dst_f;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;

	pix->width = frame->o_width;
	pix->height = frame->o_height;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = frame->fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->num_planes = frame->fmt->memplanes;

	for (i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = frame->bytesperline[i];
		pix->plane_fmt[i].sizeimage = frame->sizeimage[i];
	}

	return 0;
}

static int mxc_isi_m2m_g_fmt_vid_out(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	struct mxc_isi_frame *frame = &mxc_isi->m2m.src_f;
	int i;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		return -EINVAL;

	pix->width = frame->o_width;
	pix->height = frame->o_height;
	pix->field = V4L2_FIELD_NONE;
	pix->pixelformat = frame->fmt->fourcc;
	pix->colorspace = V4L2_COLORSPACE_JPEG;
	pix->num_planes = frame->fmt->memplanes;

	for (i = 0; i < pix->num_planes; ++i) {
		pix->plane_fmt[i].bytesperline = frame->bytesperline[i];
		pix->plane_fmt[i].sizeimage = frame->sizeimage[i];
	}

	return 0;
}

static int mxc_isi_m2m_streamon(struct file *file, void *priv,
			     enum v4l2_buf_type type)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	int ret;

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		mxc_isi->m2m.frame_count = 0;
		mxc_isi_m2m_channel_config(mxc_isi);
	}

	ret = v4l2_m2m_ioctl_streamon(file, priv, type);

	return ret;
}

static int mxc_isi_m2m_streamoff(struct file *file, void *priv,
			    enum v4l2_buf_type type)
{
	struct mxc_isi_dev *mxc_isi = video_drvdata(file);
	int ret;

	if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		mxc_isi_channel_disable(mxc_isi);

	ret = v4l2_m2m_ioctl_streamoff(file, priv, type);

	return ret;
}

static const struct v4l2_ioctl_ops mxc_isi_m2m_ioctl_ops = {
	.vidioc_querycap		= mxc_isi_m2m_querycap,

	.vidioc_enum_fmt_vid_cap_mplane = mxc_isi_m2m_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out_mplane = mxc_isi_m2m_enum_fmt_vid_out,

	.vidioc_try_fmt_vid_cap_mplane = mxc_isi_m2m_try_fmt_vid_cap,
	.vidioc_try_fmt_vid_out_mplane = mxc_isi_m2m_try_fmt_vid_out,

	.vidioc_s_fmt_vid_cap_mplane = mxc_isi_m2m_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_out_mplane = mxc_isi_m2m_s_fmt_vid_out,

	.vidioc_g_fmt_vid_cap_mplane = mxc_isi_m2m_g_fmt_vid_cap,
	.vidioc_g_fmt_vid_out_mplane = mxc_isi_m2m_g_fmt_vid_out,

	.vidioc_reqbufs			= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf		= v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf			= v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf			= v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf			= v4l2_m2m_ioctl_expbuf,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,

	.vidioc_streamon		= mxc_isi_m2m_streamon,
	.vidioc_streamoff		= mxc_isi_m2m_streamoff,
};

int mxc_isi_register_m2m_device(struct mxc_isi_dev *mxc_isi,
				 struct v4l2_device *v4l2_dev)
{
	struct device *dev = &mxc_isi->pdev->dev;
	struct video_device *vdev = &mxc_isi->m2m.vdev;
	struct mxc_isi_m2m_dev *isi_m2m = &mxc_isi->m2m;
	int ret = -ENOMEM;

	/* Only ISI channel0 support memory to memory */
	if (mxc_isi->id != 0)
		return -EINVAL;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	/* m2m */
	isi_m2m->m2m_dev = v4l2_m2m_init(&mxc_isi_m2m_ops);
	if (IS_ERR(isi_m2m->m2m_dev)) {
		dev_err(dev, "%s fail to get m2m device\n", __func__);
		return PTR_ERR(isi_m2m->m2m_dev);
	}

	INIT_LIST_HEAD(&isi_m2m->out_active);

	/* Video device */
	memset(vdev, 0, sizeof(*vdev));
	snprintf(vdev->name, sizeof(vdev->name), "mxc_isi.%d.m2m", mxc_isi->id);

	vdev->fops	= &mxc_isi_m2m_fops;
	vdev->ioctl_ops	= &mxc_isi_m2m_ioctl_ops;
	vdev->v4l2_dev	= v4l2_dev;
	vdev->minor	= -1;
	vdev->release	= video_device_release_empty;
	vdev->vfl_dir = VFL_DIR_M2M;
	vdev->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(dev, "%s fail to register video device\n", __func__);
		goto free_m2m;
	}
	video_set_drvdata(vdev, mxc_isi);

	return 0;

free_m2m:
	v4l2_m2m_release(isi_m2m->m2m_dev);
	return ret;
}

void mxc_isi_unregister_m2m_device(struct mxc_isi_dev *mxc_isi)
{
	struct video_device *vdev;

	vdev = &mxc_isi->m2m.vdev;
	if (video_is_registered(vdev))
		video_unregister_device(vdev);

	v4l2_m2m_release(mxc_isi->m2m.m2m_dev);
}

void mxc_isi_m2m_frame_read_done(struct mxc_isi_dev *mxc_isi)
{
	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);
	mxc_isi->m2m.read_done = 1;
}

void mxc_isi_m2m_frame_write_done(struct mxc_isi_dev *mxc_isi)
{
	struct v4l2_fh *fh;
	struct mxc_isi_ctx *curr_mxc_ctx;
	struct vb2_buffer *src_vb2, *dst_vb2;
	struct mxc_isi_buffer *src_buf, *dst_buf;
	struct v4l2_m2m_buffer *b;

	dev_dbg(&mxc_isi->pdev->dev, "%s\n", __func__);

	curr_mxc_ctx = v4l2_m2m_get_curr_priv(mxc_isi->m2m.m2m_dev);
	if (!curr_mxc_ctx) {
		dev_err(&mxc_isi->pdev->dev,
					"Instance released before the end of transaction\n");
		return;
	}
	fh = &curr_mxc_ctx->fh;

	if (!mxc_isi->m2m.read_done)
		return;

	if (mxc_isi->m2m.aborting) {
		mxc_isi_channel_disable(mxc_isi);
		dev_warn(&mxc_isi->pdev->dev, "Aborting current job\n");
		goto job_finish;
	}

	src_vb2 = v4l2_m2m_next_src_buf(fh->m2m_ctx);
	if (!src_vb2) {
		dev_err(&mxc_isi->pdev->dev, "No enought source buffers\n");
		goto job_finish;
	}
	src_buf = vb2_to_isi_buffer(src_vb2);
	v4l2_m2m_src_buf_remove(fh->m2m_ctx);
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(src_vb2), VB2_BUF_STATE_DONE);

	if (!list_empty(&mxc_isi->m2m.out_active)) {
		dst_buf = list_first_entry(&mxc_isi->m2m.out_active,
						struct mxc_isi_buffer, list);
		dst_vb2 = &dst_buf->v4l2_buf.vb2_buf;
		list_del_init(&dst_buf->list);
		dst_buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
		v4l2_m2m_buf_done(to_vb2_v4l2_buffer(dst_vb2), VB2_BUF_STATE_DONE);

#if 0 // for debug
		char *s_buf, *d_buf;
		s_buf = vb2_plane_vaddr(src_vb2, 0);
		if (s_buf) {
			pr_info("src: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
					s_buf[0],s_buf[1], s_buf[2],s_buf[3],s_buf[4],
					s_buf[5],s_buf[6], s_buf[7],s_buf[8],s_buf[9]);
		}
		d_buf = vb2_plane_vaddr(dst_vb2, 0);
		if (d_buf) {
			pr_info("dst: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
					d_buf[0],d_buf[1], d_buf[2], d_buf[3],d_buf[4],
					d_buf[5],d_buf[6], d_buf[7],d_buf[8],d_buf[9]);
		}
#endif
	}
	mxc_isi->m2m.frame_count++;

	dst_vb2 = v4l2_m2m_next_dst_buf(fh->m2m_ctx);
	if (dst_vb2) {
		dst_vb2->state = VB2_BUF_STATE_ACTIVE;
		dst_buf = vb2_to_isi_buffer(dst_vb2);
		dst_buf->v4l2_buf.sequence = mxc_isi->m2m.frame_count;
		mxc_isi_channel_set_m2m_out_addr(mxc_isi, dst_buf);
		v4l2_m2m_dst_buf_remove(fh->m2m_ctx);
		b = to_v4l2_m2m_buffer(dst_vb2);
		list_add_tail(&b->list, &mxc_isi->m2m.out_active);
	}

job_finish:
	v4l2_m2m_job_finish(mxc_isi->m2m.m2m_dev, fh->m2m_ctx);
	mxc_isi->m2m.read_done = 0;
}
