/*
 * Driver for the AXI-HDMI-RX core
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/dmaengine.h>
#include <linux/firmware.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>

#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/i2c/adv7604.h>

#define AXI_HDMI_RX_REG_VERSION		0x000
#define AXI_HDMI_RX_REG_ID		0x004
#define AXI_HDMI_RX_REG_ENABLE		0x040
#define AXI_HDMI_RX_REG_CONFIG		0x044
#define AXI_HDMI_RX_REG_CLK_COUNT	0x054
#define AXI_HDMI_RX_REG_CLK_RATIO	0x058
#define AXI_HDMI_RX_REG_DMA_STATUS	0x060
#define AXI_HDMI_RX_REG_TPM_STATUS	0x064
#define AXI_HDMI_RX_REG_STATUS		0x080
#define AXI_HDMI_RX_REG_TIMING		0x400
#define AXI_HDMI_RX_REG_DETECTED_TIMING 0x404

#define AXI_HDMI_RX_CONFIG_EDGE_SEL	BIT(3)
#define AXI_HDMI_RX_CONFIG_BGR		BIT(2)
#define AXI_HDMI_RX_CONFIG_PACKED	BIT(1)
#define AXI_HDMI_RX_CONFIG_CSC_BYPASS	BIT(0)

struct axi_hdmi_rx_stream {
	struct video_device vdev;
	struct vb2_queue q;
	struct v4l2_subdev *subdev;
	struct mutex lock;
	spinlock_t spinlock;
	u32 width, height;
	u32 stride;

	__u32 pixelformat;

	struct dma_chan *chan;
	struct list_head queued_buffers;
};

struct axi_hdmi_rx {
	struct v4l2_device v4l2_dev;

	struct axi_hdmi_rx_stream stream;

	int hotplug_gpio;

	void __iomem *base;

	struct v4l2_async_notifier notifier;
	struct v4l2_async_subdev asd;
	struct v4l2_async_subdev *asds[1];

	u8 bus_width;

	u8 edid_data[256];
	u8 edid_blocks;
};

struct axi_hdmi_rx_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head head;
};

static void axi_hdmi_rx_write(struct axi_hdmi_rx *axi_hdmi_rx,
	unsigned int reg, unsigned int val)
{
	writel(val, axi_hdmi_rx->base + reg);
}

static unsigned int axi_hdmi_rx_read(struct axi_hdmi_rx *axi_hdmi_rx,
	unsigned int reg)
{
	return readl(axi_hdmi_rx->base + reg);
}

static struct axi_hdmi_rx *to_axi_hdmi_rx(struct v4l2_device *v4l2_dev)
{
	return container_of(v4l2_dev, struct axi_hdmi_rx, v4l2_dev);
}

static struct axi_hdmi_rx_stream *axi_hdmi_rx_file_to_stream(struct file *file)
{
	struct axi_hdmi_rx *axi_hdmi_rx = video_drvdata(file);
	return &axi_hdmi_rx->stream;
}

static struct axi_hdmi_rx_buffer *vb2_buf_to_hdmi_rx_buf(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *v4l2_buf = to_vb2_v4l2_buffer(vb);
	return container_of(v4l2_buf, struct axi_hdmi_rx_buffer, vb);
}

static const struct v4l2_file_operations axi_hdmi_rx_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

static int axi_hdmi_rx_queue_setup(struct vb2_queue *q,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], struct device *alloc_ctxs[])
{
	struct axi_hdmi_rx *hdmi_rx = vb2_get_drv_priv(q);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	if (*num_buffers < 1)
		*num_buffers = 1;

	if (*num_planes) {
		if (sizes[0] < s->stride * s->height)
			return -EINVAL;
	} else {
		sizes[0] = s->stride * s->height;
		*num_planes = 1;
	}

	return 0;
}

static int axi_hdmi_rx_buf_prepare(struct vb2_buffer *vb)
{
	struct axi_hdmi_rx *hdmi_rx = vb2_get_drv_priv(vb->vb2_queue);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	unsigned size;

	size = s->stride * s->height;
	if (vb2_plane_size(vb, 0) < size) {
		pr_info("data will not fit into plane (%lu < %u)\n",
					vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, size);
	return 0;
}

static void axi_hdmi_rx_dma_done(void *arg)
{
	struct axi_hdmi_rx_buffer *buf = arg;
	struct vb2_queue *q = buf->vb.vb2_buf.vb2_queue;
	struct axi_hdmi_rx *hdmi_rx = vb2_get_drv_priv(q);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	unsigned long flags;

	spin_lock_irqsave(&s->spinlock, flags);
	list_del(&buf->head);
	spin_unlock_irqrestore(&s->spinlock, flags);

	buf->vb.vb2_buf.timestamp = ktime_get_ns();
	vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
}

static void axi_hdmi_rx_buf_queue(struct vb2_buffer *vb)
{
	struct axi_hdmi_rx_buffer *buf = vb2_buf_to_hdmi_rx_buf(vb);
	struct dma_async_tx_descriptor *desc;
	struct vb2_queue *q = vb->vb2_queue;
	struct axi_hdmi_rx *hdmi_rx = vb2_get_drv_priv(q);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	unsigned int bpp;
	struct dma_interleaved_template *xt;
	unsigned long size;
	unsigned long flags;
	dma_addr_t addr;
	dma_cookie_t cookie;

	addr = vb2_dma_contig_plane_dma_addr(vb, 0);
	size = vb2_get_plane_payload(vb, 0);

	xt = kzalloc(sizeof(struct dma_async_tx_descriptor) +
				sizeof(struct data_chunk), GFP_KERNEL);
	if (!xt) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	switch (s->pixelformat) {
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		bpp = 3;
		break;
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_UYVY:
		bpp = 2;
		break;
	default:
		bpp = 4;
		break;
	}

	xt->dst_start = addr;
	xt->src_inc = false;
	xt->dst_inc = true;
	xt->src_sgl = false;
	xt->dst_sgl = true;
	xt->frame_size = 1;
	xt->numf = s->height;
	xt->sgl[0].size = s->width * bpp;
	xt->sgl[1].icg = s->stride - (s->width * bpp);
	xt->dir = DMA_DEV_TO_MEM;

	desc = dmaengine_prep_interleaved_dma(s->chan, xt, DMA_PREP_INTERRUPT);
	kfree(xt);
	if (!desc) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}
	desc->callback = axi_hdmi_rx_dma_done;
	desc->callback_param = buf;

	cookie = dmaengine_submit(desc);
	if (cookie < 0) {
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	spin_lock_irqsave(&s->spinlock, flags);
	list_add_tail(&buf->head, &s->queued_buffers);
	spin_unlock_irqrestore(&s->spinlock, flags);

	if (vb2_is_streaming(q))
		dma_async_issue_pending(s->chan);
}

static int axi_hdmi_rx_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct axi_hdmi_rx *hdmi_rx = vb2_get_drv_priv(q);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	dma_async_issue_pending(s->chan);
	return 0;
}

static void axi_hdmi_rx_stop_streaming(struct vb2_queue *q)
{
	struct axi_hdmi_rx *hdmi_rx = vb2_get_drv_priv(q);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	struct axi_hdmi_rx_buffer *buf;
	unsigned long flags;

	dmaengine_terminate_all(s->chan);

	spin_lock_irqsave(&s->spinlock, flags);

	list_for_each_entry(buf, &s->queued_buffers, head)
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);

	INIT_LIST_HEAD(&s->queued_buffers);

	spin_unlock_irqrestore(&s->spinlock, flags);

	vb2_wait_for_all_buffers(q);
}

static const struct vb2_ops axi_hdmi_rx_qops = {
	.queue_setup = axi_hdmi_rx_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,

	.buf_prepare = axi_hdmi_rx_buf_prepare,
	.buf_queue = axi_hdmi_rx_buf_queue,
	.start_streaming = axi_hdmi_rx_start_streaming,
	.stop_streaming = axi_hdmi_rx_stop_streaming,
};

#ifdef CONFIG_VIDEO_ADV_DEBUG

static int axi_hdmi_rx_g_register(struct file *file, void *priv_fh,
	struct v4l2_dbg_register *reg)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);

	switch (reg->reg) {
	case AXI_HDMI_RX_REG_VERSION:
	case AXI_HDMI_RX_REG_ID:
	case AXI_HDMI_RX_REG_ENABLE:
	case AXI_HDMI_RX_REG_CONFIG:
	case AXI_HDMI_RX_REG_CLK_COUNT:
	case AXI_HDMI_RX_REG_CLK_RATIO:
	case AXI_HDMI_RX_REG_DMA_STATUS:
	case AXI_HDMI_RX_REG_TPM_STATUS:
	case AXI_HDMI_RX_REG_STATUS:
	case AXI_HDMI_RX_REG_TIMING:
	case AXI_HDMI_RX_REG_DETECTED_TIMING:
		break;
	default:
		return -EINVAL;
	}

	reg->val = axi_hdmi_rx_read(hdmi_rx, reg->reg);
	reg->size = 4;

	return 0;
}

static int axi_hdmi_rx_s_register(struct file *file, void *priv_fh,
	const struct v4l2_dbg_register *reg)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);

	switch (reg->reg) {
	case AXI_HDMI_RX_REG_ENABLE:
	case AXI_HDMI_RX_REG_CONFIG:
	case AXI_HDMI_RX_REG_DMA_STATUS:
	case AXI_HDMI_RX_REG_TPM_STATUS:
	case AXI_HDMI_RX_REG_STATUS:
	case AXI_HDMI_RX_REG_TIMING:
		break;
	default:
		return -EINVAL;
	}

	axi_hdmi_rx_write(hdmi_rx, reg->reg, reg->val);

	return 0;
}
#endif

static int axi_hdmi_rx_log_status(struct file *file, void *priv)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);

	v4l2_device_call_all(&hdmi_rx->v4l2_dev, 0, core, log_status);
	return 0;
}

static int axi_hdmi_rx_querycap(struct file *file, void *priv_fh,
	struct v4l2_capability *vcap)
{
	strlcpy(vcap->driver, "axi_hdmi_rx", sizeof(vcap->driver));
	strlcpy(vcap->card, "axi_hdmi_rx", sizeof(vcap->card));
	snprintf(vcap->bus_info, sizeof(vcap->bus_info), "platform:axi-hdmi-rx");
	vcap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	vcap->capabilities = vcap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}

static int axi_hdmi_rx_streamon(struct file *file, void *priv_fh,
	enum v4l2_buf_type buffer_type)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = axi_hdmi_rx_file_to_stream(file);

	if (buffer_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	/* Clear status bits */
	axi_hdmi_rx_write(hdmi_rx, AXI_HDMI_RX_REG_STATUS, 0xf);
	axi_hdmi_rx_write(hdmi_rx, AXI_HDMI_RX_REG_ENABLE, 1);

	return vb2_streamon(&s->q, buffer_type);
}

static int axi_hdmi_rx_streamoff(struct file *file, void *priv_fh,
	enum v4l2_buf_type buffer_type)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = axi_hdmi_rx_file_to_stream(file);
	int ret;

	if (buffer_type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
		return -EINVAL;

	ret = vb2_streamoff(&s->q, buffer_type);

	axi_hdmi_rx_write(hdmi_rx, AXI_HDMI_RX_REG_ENABLE, 0);

	return ret;
}

static int axi_hdmi_rx_s_dv_timings(struct file *file, void *priv_fh,
	struct v4l2_dv_timings *timings)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	return v4l2_subdev_call(s->subdev, video, s_dv_timings, timings);
}

static int axi_hdmi_rx_g_dv_timings(struct file *file, void *priv_fh,
	struct v4l2_dv_timings *timings)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	return v4l2_subdev_call(s->subdev, video, g_dv_timings, timings);
}

static int axi_hdmi_rx_enum_dv_timings(struct file *file, void *priv_fh,
	struct v4l2_enum_dv_timings *timings)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	return v4l2_subdev_call(s->subdev, pad, enum_dv_timings, timings);
}

static int axi_hdmi_rx_query_dv_timings(struct file *file, void *priv_fh,
	struct v4l2_dv_timings *timings)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	return v4l2_subdev_call(s->subdev, video, query_dv_timings,
		timings);
}

static int axi_hdmi_rx_dv_timings_cap(struct file *file, void *priv_fh,
	struct v4l2_dv_timings_cap *cap)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	return v4l2_subdev_call(s->subdev, pad, dv_timings_cap, cap);
}

static int axi_hdmi_rx_enum_fmt_vid_cap(struct file *file, void *priv_fh,
	struct v4l2_fmtdesc *f)
{
	switch (f->index) {
	case 0:
		strlcpy(f->description, "BGR32", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_BGR32;
		break;
	case 1:
		strlcpy(f->description, "RGB24", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_RGB24;
		break;
	case 2:
		strlcpy(f->description, "RGB32", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_RGB32;
		break;
	case 3:
		strlcpy(f->description, "BGR24", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_BGR24;
		break;
	case 4:
		strlcpy(f->description, "YCBCr", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_YVYU;
		break;
	case 5:
		strlcpy(f->description, "YCrCb", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_YUYV;
		break;
	case 6:
		strlcpy(f->description, "CbCrY", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_VYUY;
		break;
	case 7:
		strlcpy(f->description, "CrCbY", sizeof(f->description));
		f->pixelformat = V4L2_PIX_FMT_UYVY;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int axi_hdmi_rx_g_fmt_vid_cap(struct file *file, void *priv_fh,
	struct v4l2_format *f)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;

	pix->width = s->width;
	pix->height = s->height;
	pix->bytesperline = s->stride;
	pix->field = V4L2_FIELD_NONE;

	switch (s->pixelformat) {
	case V4L2_PIX_FMT_RGB32:
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_BGR24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_REC709;
		break;
	default:
		return -EINVAL;
	}

	pix->pixelformat = s->pixelformat;
	pix->sizeimage = pix->bytesperline * pix->height;

	return 0;
}

static int axi_hdmi_rx_try_fmt_vid_cap(struct file *file, void *priv_fh,
	struct v4l2_format *f)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	struct v4l2_subdev_format fmt;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	int ret;

	v4l_bound_align_image(&pix->width, 176, 1920, 0, &pix->height, 144,
		1080, 0, 0);

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = ADV7611_PAD_SOURCE;
	ret = v4l2_subdev_call(s->subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	v4l2_fill_pix_format(pix, &fmt.format);

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_BGR32:
	case V4L2_PIX_FMT_RGB32:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->bytesperline = pix->width * 4;
		break;
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_VYUY:
	case V4L2_PIX_FMT_UYVY:
		pix->colorspace = V4L2_COLORSPACE_REC709;;
		pix->bytesperline = pix->width * 2;
		break;
	default:
		pix->pixelformat = V4L2_PIX_FMT_RGB24;
	case V4L2_PIX_FMT_RGB24:
	case V4L2_PIX_FMT_BGR24:
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->bytesperline = pix->width * 3;
		break;
	}

	pix->sizeimage = pix->bytesperline * pix->height;
	pix->field = V4L2_FIELD_NONE;
	pix->priv = 0;

	return 0;
}

static int axi_hdmi_rx_s_fmt_vid_cap(struct file *file, void *priv_fh,
	struct v4l2_format *f)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev_format fmt;
	unsigned int config;
	int ret;

	if (axi_hdmi_rx_try_fmt_vid_cap(file, priv_fh, f))
		return -EINVAL;

	s->width = pix->width;
	s->height = pix->height;
	s->stride = pix->bytesperline;

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_YVYU:
		fmt.format.code = MEDIA_BUS_FMT_YVYU8_1X16;
		break;
	case V4L2_PIX_FMT_VYUY:
		fmt.format.code = MEDIA_BUS_FMT_VYUY8_1X16;
		break;
	case V4L2_PIX_FMT_UYVY:
		fmt.format.code = MEDIA_BUS_FMT_UYVY8_1X16;
		break;
	case V4L2_PIX_FMT_YUYV:
		fmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;
		break;
	default:
		if (hdmi_rx->bus_width >= 24)
			fmt.format.code = MEDIA_BUS_FMT_RGB888_1X24;
		else /* CSC expects this */
			fmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;
		break;
	}

	switch (pix->pixelformat) {
	case V4L2_PIX_FMT_BGR32:
		config = 0;
		break;
	case V4L2_PIX_FMT_BGR24:
		config = AXI_HDMI_RX_CONFIG_PACKED;
		break;
	case V4L2_PIX_FMT_RGB32:
		config = AXI_HDMI_RX_CONFIG_BGR;
		break;
	case V4L2_PIX_FMT_RGB24:
		config = AXI_HDMI_RX_CONFIG_PACKED | AXI_HDMI_RX_CONFIG_BGR;
		break;
	case V4L2_PIX_FMT_YUYV:
	case V4L2_PIX_FMT_YVYU:
	case V4L2_PIX_FMT_UYVY:
	case V4L2_PIX_FMT_VYUY:
		config = AXI_HDMI_RX_CONFIG_CSC_BYPASS |
		    AXI_HDMI_RX_CONFIG_PACKED;
		break;
	default:
		return -EINVAL;
	}

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = ADV7611_PAD_SOURCE;
	ret = v4l2_subdev_call(s->subdev, pad, set_fmt, NULL, &fmt);
	if (ret)
		return ret;

	s->pixelformat = pix->pixelformat;

	axi_hdmi_rx_write(hdmi_rx, AXI_HDMI_RX_REG_TIMING, 
		(s->height << 16) | s->width);

	config |= AXI_HDMI_RX_CONFIG_EDGE_SEL;

	axi_hdmi_rx_write(hdmi_rx, AXI_HDMI_RX_REG_CONFIG, config);

	return 0;
}

static int axi_hdmi_rx_enum_input(struct file *file, void *priv_fh,
	struct v4l2_input *inp)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	switch (inp->index) {
	case 0:
		snprintf(inp->name, sizeof(inp->name), "HDMI-0");
		break;
	default:
		return -EINVAL;
	}

	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->capabilities = V4L2_IN_CAP_DV_TIMINGS;

	return v4l2_subdev_call(s->subdev, video, g_input_status, &inp->status);
}

static int axi_hdmi_rx_g_input(struct file *file, void *priv_fh, unsigned int *i)
{
	*i = 0;
	return 0;
}

static int axi_hdmi_rx_s_input(struct file *file, void *priv_fh, unsigned int i)
{
	struct axi_hdmi_rx *hdmi_rx = video_drvdata(file);
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;

	if (i != 0)
		return -EINVAL;

	return v4l2_subdev_call(s->subdev, video, s_routing,
		ADV76XX_PAD_HDMI_PORT_A, 0, 0);
}

static const struct v4l2_ioctl_ops axi_hdmi_rx_ioctl_ops = {
	.vidioc_querycap		= axi_hdmi_rx_querycap,
	.vidioc_log_status		= axi_hdmi_rx_log_status,
	.vidioc_streamon		= axi_hdmi_rx_streamon,
	.vidioc_streamoff		= axi_hdmi_rx_streamoff,
	.vidioc_enum_input		= axi_hdmi_rx_enum_input,
	.vidioc_g_input			= axi_hdmi_rx_g_input,
	.vidioc_s_input			= axi_hdmi_rx_s_input,
	.vidioc_enum_fmt_vid_cap	= axi_hdmi_rx_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap		= axi_hdmi_rx_g_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap		= axi_hdmi_rx_s_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap		= axi_hdmi_rx_try_fmt_vid_cap,
	.vidioc_s_dv_timings		= axi_hdmi_rx_s_dv_timings,
	.vidioc_g_dv_timings		= axi_hdmi_rx_g_dv_timings,
	.vidioc_query_dv_timings	= axi_hdmi_rx_query_dv_timings,
	.vidioc_enum_dv_timings		= axi_hdmi_rx_enum_dv_timings,
	.vidioc_dv_timings_cap		= axi_hdmi_rx_dv_timings_cap,
	.vidioc_subscribe_event		= v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event	= v4l2_event_unsubscribe,
	.vidioc_create_bufs 		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.vidioc_g_register		= axi_hdmi_rx_g_register,
	.vidioc_s_register		= axi_hdmi_rx_s_register,
#endif
};

static void axi_hdmi_rx_notify(struct v4l2_subdev *sd, unsigned int notification,
	void *arg)
{
	struct axi_hdmi_rx *hdmi_rx = to_axi_hdmi_rx(sd->v4l2_dev);
	long hotplug = (long)arg;

	switch (notification) {
	case ADV76XX_HOTPLUG:
		gpio_set_value_cansleep(hdmi_rx->hotplug_gpio, hotplug);
		break;
	default:
		break;
	}
}

static int axi_hdmi_rx_nodes_register(struct axi_hdmi_rx *hdmi_rx)
{
	struct axi_hdmi_rx_stream *s = &hdmi_rx->stream;
	struct video_device *vdev = &s->vdev;
	struct vb2_queue *q = &s->q;
	int ret;

	mutex_init(&s->lock);
	snprintf(vdev->name, sizeof(vdev->name),
		 "%s", hdmi_rx->v4l2_dev.name);
	vdev->v4l2_dev = &hdmi_rx->v4l2_dev;
	vdev->fops = &axi_hdmi_rx_fops;
	vdev->release = video_device_release_empty;
	vdev->ctrl_handler = s->subdev->ctrl_handler;
	vdev->lock = &s->lock;
	vdev->queue = q;
	q->lock = &s->lock;
	q->dev = hdmi_rx->v4l2_dev.dev;

	INIT_LIST_HEAD(&s->queued_buffers);
	spin_lock_init(&s->spinlock);

	s->width = 800;
	s->height = 600;
	s->pixelformat = V4L2_PIX_FMT_BGR32;
	s->stride = s->width * 4;

	vdev->ioctl_ops = &axi_hdmi_rx_ioctl_ops;

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP | VB2_USERPTR | VB2_READ;
	q->drv_priv = hdmi_rx;
	q->buf_struct_size = sizeof(struct axi_hdmi_rx_buffer);
	q->ops = &axi_hdmi_rx_qops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;

	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	return video_register_device(vdev, VFL_TYPE_GRABBER, -1);
}

static struct axi_hdmi_rx *notifier_to_axi_hdmi_rx(struct v4l2_async_notifier *n)
{
	return container_of(n, struct axi_hdmi_rx, notifier);
}

static int axi_hdmi_rx_async_bound(struct v4l2_async_notifier *notifier,
	struct v4l2_subdev *subdev, struct v4l2_async_subdev *asd)
{
	struct axi_hdmi_rx *hdmi_rx = notifier_to_axi_hdmi_rx(notifier);
	struct v4l2_subdev_format fmt;
	int ret;

	struct v4l2_subdev_edid edid = {
		.pad = 0,
		.start_block = 0,
		.blocks = hdmi_rx->edid_blocks,
		.edid = hdmi_rx->edid_data,
	};

	hdmi_rx->stream.subdev = subdev;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	fmt.pad = ADV7611_PAD_SOURCE;
	fmt.format.code = MEDIA_BUS_FMT_YUYV8_1X16;
	ret = v4l2_subdev_call(subdev, pad, set_fmt, NULL, &fmt);
	if (ret)
		return ret;

	ret = v4l2_subdev_call(subdev, video, s_routing, ADV76XX_PAD_HDMI_PORT_A,
		0, 0);
	if (ret)
		return ret;

	return v4l2_subdev_call(subdev, pad, set_edid, &edid);
}

static int axi_hdmi_rx_async_complete(struct v4l2_async_notifier *notifier)
{
	struct axi_hdmi_rx *hdmi_rx = notifier_to_axi_hdmi_rx(notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&hdmi_rx->v4l2_dev);
	if (ret < 0)
		return ret;

	return axi_hdmi_rx_nodes_register(hdmi_rx);
}

static const struct v4l2_async_notifier_operations axi_hdmi_rx_async_ops = {
	.bound = axi_hdmi_rx_async_bound,
	.complete = axi_hdmi_rx_async_complete,
};

static int axi_hdmi_rx_load_edid(struct platform_device *pdev,
	struct axi_hdmi_rx *hdmi_rx)
{
	const struct firmware *fw;
	int ret;

	ret = request_firmware(&fw, "imageon_edid.bin", &pdev->dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to load firmware: %d\n", ret);
		return ret;
	}

	if (fw->size > 256) {
		dev_err(&pdev->dev, "EDID firmware data too large.\n");
		release_firmware(fw);
		return -EINVAL;
	}

	if (fw->size > 128)
		hdmi_rx->edid_blocks = 2;
	else
		hdmi_rx->edid_blocks = 1;

	memcpy(hdmi_rx->edid_data, fw->data, fw->size);

	release_firmware(fw);

	return 0;
}

static int axi_hdmi_rx_probe(struct platform_device *pdev)
{
	struct device_node *ep_node;
	struct axi_hdmi_rx *hdmi_rx;
	struct resource *res;
	struct v4l2_fwnode_endpoint bus_cfg;
	int ret;

	hdmi_rx = devm_kzalloc(&pdev->dev, sizeof(*hdmi_rx), GFP_KERNEL);
	if (hdmi_rx == NULL)
		return -ENOMEM;

	hdmi_rx->hotplug_gpio = of_get_gpio(pdev->dev.of_node, 0);
	if (!gpio_is_valid(hdmi_rx->hotplug_gpio))
		return hdmi_rx->hotplug_gpio;

	ret = devm_gpio_request_one(&pdev->dev,
		hdmi_rx->hotplug_gpio, GPIOF_OUT_INIT_LOW, "HPD");
	if (ret < 0)
		return ret;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	hdmi_rx->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(hdmi_rx->base))
		return PTR_ERR(hdmi_rx->base);

	hdmi_rx->stream.chan = dma_request_slave_channel(&pdev->dev, "rx");
	if (!hdmi_rx->stream.chan)
		return -EPROBE_DEFER;

	ret = axi_hdmi_rx_load_edid(pdev, hdmi_rx);
	if (ret)
		goto err_dma_release_channel;

	snprintf(hdmi_rx->v4l2_dev.name, sizeof(hdmi_rx->v4l2_dev.name),
		"axi_hdmi_rx");
	hdmi_rx->v4l2_dev.notify = axi_hdmi_rx_notify;

	video_set_drvdata(&hdmi_rx->stream.vdev, hdmi_rx);

	ret = v4l2_device_register(&pdev->dev, &hdmi_rx->v4l2_dev);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register card: %d\n", ret);
		goto err_dma_release_channel;
	}

	ep_node = of_graph_get_next_endpoint(pdev->dev.of_node, NULL);
	if (!ep_node) {
		ret = -EINVAL;
		goto err_device_unregister;
	}
	bus_cfg.bus.parallel.bus_width = 0;
	v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep_node), &bus_cfg);
	if (bus_cfg.bus.parallel.bus_width)
		hdmi_rx->bus_width = bus_cfg.bus.parallel.bus_width;
	else
		hdmi_rx->bus_width = 16;

	hdmi_rx->asd.match_type = V4L2_ASYNC_MATCH_FWNODE;
	hdmi_rx->asd.match.fwnode = of_fwnode_handle(of_graph_get_remote_port_parent(ep_node));

	hdmi_rx->asds[0] = &hdmi_rx->asd;
	hdmi_rx->notifier.subdevs = hdmi_rx->asds;
	hdmi_rx->notifier.num_subdevs = ARRAY_SIZE(hdmi_rx->asds);
	hdmi_rx->notifier.ops = &axi_hdmi_rx_async_ops;

	ret = v4l2_async_notifier_register(&hdmi_rx->v4l2_dev,
		&hdmi_rx->notifier);
	if (ret) {
		dev_err(&pdev->dev, "Error %d registering device nodes\n", ret);
		goto err_device_unregister;
	}

	axi_hdmi_rx_write(hdmi_rx, AXI_HDMI_RX_REG_CONFIG,
			AXI_HDMI_RX_CONFIG_EDGE_SEL);

	return 0;

err_device_unregister:
	v4l2_device_unregister(&hdmi_rx->v4l2_dev);
err_dma_release_channel:
	dma_release_channel(hdmi_rx->stream.chan);
	return ret;
}

static int axi_hdmi_rx_remove(struct platform_device *pdev)
{
	struct axi_hdmi_rx *hdmi_rx = platform_get_drvdata(pdev);

	v4l2_async_notifier_unregister(&hdmi_rx->notifier);
	video_unregister_device(&hdmi_rx->stream.vdev);
	v4l2_device_unregister(&hdmi_rx->v4l2_dev);
	dma_release_channel(hdmi_rx->stream.chan);

	return 0;
}

static const struct of_device_id axi_hdmi_rx_of_match[] = {
	{ .compatible = "adi,axi-hdmi-rx-1.00.a", },
	{},
};
MODULE_DEVICE_TABLE(of, axi_hdmi_rx_of_match);

static struct platform_driver axi_hdmi_rx_driver = {
	.driver = {
		.name = "axi-hdmi-rx",
		.owner = THIS_MODULE,
		.of_match_table = axi_hdmi_rx_of_match,
	},
	.probe = axi_hdmi_rx_probe,
	.remove = axi_hdmi_rx_remove,
};
module_platform_driver(axi_hdmi_rx_driver);
