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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/irqreturn.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/string.h>

#include <media/v4l2-mem2mem.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-event.h>
#include <media/videobuf2-dma-contig.h>

#include "mxc-jpeg-hw.h"
#include "mxc-jpeg.h"

static struct mxc_jpeg_fmt mxc_formats[] = {
	{
		.name		= "JPEG",
		.fourcc		= V4L2_PIX_FMT_JPEG,
		.colplanes	= 1,
		.flags		= MXC_JPEG_FMT_TYPE_ENC,
	},
	{
		.name		= "RGB32",
		.fourcc		= V4L2_PIX_FMT_RGB32,
		.depth		= 32,
		.colplanes	= 1,
		.h_align	= 0,
		.v_align	= 0,
		.flags		= MXC_JPEG_FMT_TYPE_RAW,
	},
	{
		.name		= "YUV422",
		.fourcc		= V4L2_PIX_FMT_YUYV,
		.depth		= 16,
		.colplanes	= 1,
		.h_align	= 2,
		.v_align	= 0,
		.flags		= MXC_JPEG_FMT_TYPE_RAW,
	},
	{
		.name		= "YUV444",
		.fourcc		= V4L2_PIX_FMT_YUV32,
		.depth		= 32,
		.colplanes	= 1,
		.h_align	= 0,
		.v_align	= 0,
		.flags		= MXC_JPEG_FMT_TYPE_RAW,
	},
};

static const struct of_device_id mxc_jpeg_match[] = {
	{
		.compatible = "fsl,imx8-jpgdec",
		.data       = (void *)MXC_JPEG_DECODE,
	},
	{
		.compatible = "fsl,imx8-jpgenc",
		.data       = (void *)MXC_JPEG_ENCODE,
	},
	{ },
};

static const unsigned char hactbl[615] = {
0xFF, 0xD8, 0xFF, 0xE0, 0x00, 0x10, 0x4A,
0x46, 0x49, 0x46, 0x00, 0x01, 0x01, 0x00,
0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0xFF,
0xDB, 0x00, 0x84, 0x00, 0x10, 0x0B, 0x0C,
0x0E, 0x0C, 0x0A, 0x10, 0x0E, 0x0D, 0x0E,
0x12, 0x11, 0x10, 0x13, 0x18, 0x28, 0x1A,
0x18, 0x16, 0x16, 0x18, 0x31, 0x23, 0x25,
0x1D, 0x28, 0x3A, 0x33, 0x3D, 0x3C, 0x39,
0x33, 0x38, 0x37, 0x40, 0x48, 0x5C, 0x4E,
0x40, 0x44, 0x57, 0x45, 0x37, 0x38, 0x50,
0x6D, 0x51, 0x57, 0x5F, 0x62, 0x67, 0x68,
0x67, 0x3E, 0x4D, 0x71, 0x79, 0x70, 0x64,
0x78, 0x5C, 0x65, 0x67, 0x63, 0x01, 0x11,
0x12, 0x12, 0x18, 0x15, 0x18, 0x2F, 0x1A,
0x1A, 0x2F, 0x63, 0x42, 0x38, 0x42, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0x63, 0x63, 0x63,
0xFF, 0xC0, 0x00, 0x11, 0x08, 0x01, 0x00,
0x01, 0x00, 0x03, 0x01, 0x11, 0x00, 0x02,
0x11, 0x01, 0x03, 0x11, 0x01, 0xFF, 0xC4,
0x01, 0xA2, 0x00, 0x00, 0x01, 0x05, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08,
0x09, 0x0A, 0x0B, 0x10, 0x00, 0x02, 0x01,
0x03, 0x03, 0x02, 0x04, 0x03, 0x05, 0x05,
0x04, 0x04, 0x00, 0x00, 0x01, 0x7D, 0x01,
0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61,
0x07, 0x22, 0x71, 0x14, 0x32, 0x81, 0x91,
0xA1, 0x08, 0x23, 0x42, 0xB1, 0xC1, 0x15,
0x52, 0xD1, 0xF0, 0x24, 0x33, 0x62, 0x72,
0x82, 0x09, 0x0A, 0x16, 0x17, 0x18, 0x19,
0x1A, 0x25, 0x26, 0x27, 0x28, 0x29, 0x2A,
0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x3A,
0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49,
0x4A, 0x53, 0x54, 0x55, 0x56, 0x57, 0x58,
0x59, 0x5A, 0x63, 0x64, 0x65, 0x66, 0x67,
0x68, 0x69, 0x6A, 0x73, 0x74, 0x75, 0x76,
0x77, 0x78, 0x79, 0x7A, 0x83, 0x84, 0x85,
0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93,
0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A,
0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4,
0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9,
0xDA, 0xE1, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6,
0xE7, 0xE8, 0xE9, 0xEA, 0xF1, 0xF2, 0xF3,
0xF4, 0xF5, 0xF6, 0xF7, 0xF8, 0xF9, 0xFA,
0x01, 0x00, 0x03, 0x01, 0x01, 0x01, 0x01,
0x01, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x03,
0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
0x0B, 0x11, 0x00, 0x02, 0x01, 0x02, 0x04,
0x04, 0x03, 0x04, 0x07, 0x05, 0x04, 0x04,
0x00, 0x01, 0x02, 0x77, 0x00, 0x01, 0x02,
0x03, 0x11, 0x04, 0x05, 0x21, 0x31, 0x06,
0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13,
0x22, 0x32, 0x81, 0x08, 0x14, 0x42, 0x91,
0xA1, 0xB1, 0xC1, 0x09, 0x23, 0x33, 0x52,
0xF0, 0x15, 0x62, 0x72, 0xD1, 0x0A, 0x16,
0x24, 0x34, 0xE1, 0x25, 0xF1, 0x17, 0x18,
0x19, 0x1A, 0x26, 0x27, 0x28, 0x29, 0x2A,
0x35, 0x36, 0x37, 0x38, 0x39, 0x3A, 0x43,
0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4A,
0x53, 0x54, 0x55, 0x56, 0x57, 0x58, 0x59,
0x5A, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
0x69, 0x6A, 0x73, 0x74, 0x75, 0x76, 0x77,
0x78, 0x79, 0x7A, 0x82, 0x83, 0x84, 0x85,
0x86, 0x87, 0x88, 0x89, 0x8A, 0x92, 0x93,
0x94, 0x95, 0x96, 0x97, 0x98, 0x99, 0x9A,
0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8,
0xA9, 0xAA, 0xB2, 0xB3, 0xB4, 0xB5, 0xB6,
0xB7, 0xB8, 0xB9, 0xBA, 0xC2, 0xC3, 0xC4,
0xC5, 0xC6, 0xC7, 0xC8, 0xC9, 0xCA, 0xD2,
0xD3, 0xD4, 0xD5, 0xD6, 0xD7, 0xD8, 0xD9,
0xDA, 0xE2, 0xE3, 0xE4, 0xE5, 0xE6, 0xE7,
0xE8, 0xE9, 0xEA, 0xF2, 0xF3, 0xF4, 0xF5,
0xF6, 0xF7, 0xF8, 0xF9, 0xFA, 0xFF, 0xDD,
0x00, 0x04, 0x00, 0x20, 0xFF, 0xDA, 0x00,
0x0C, 0x03, 0x01, 0x00, 0x02, 0x11, 0x03,
0x11, 0x00, 0x3F, 0x00, 0xFF, 0xD9
};

struct mxc_jpeg_src_buf {
	struct vb2_v4l2_buffer b;
	struct list_head list;
	struct mxc_jpeg_desc *desc;
	void *tbl;
	int flags;
	dma_addr_t handle;
	dma_addr_t cfg_handle;
	dma_addr_t tbl_handle;
};

#define MXC_NUM_FORMATS ARRAY_SIZE(mxc_formats)
static inline u32 mxc_jpeg_align(u32 val, u32 align)
{
	return (val + align - 1) & ~(align - 1);
}


static inline struct mxc_jpeg_ctx *mxc_jpeg_fh_to_ctx(struct v4l2_fh *fh)
{
	return container_of(fh, struct mxc_jpeg_ctx, fh);
}

static inline struct mxc_jpeg_src_buf *mxc_jpeg_vb2_to_srcbuf(
							struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct mxc_jpeg_src_buf, b);
}

static int enum_fmt(struct mxc_jpeg_fmt *mxc_formats, int n,
		    struct v4l2_fmtdesc *f, u32 type)
{
	int i, num = 0;

	for (i = 0; i < n; ++i) {
		if (mxc_formats[i].flags & type) {
			/* index-th format of type type found ? */
			if (num == f->index)
				break;
			/* Correct type but haven't reached our index yet,
			 * just increment per-type index
			 */
			++num;
		}
	}

	/* Format not found */
	if (i >= n)
		return -EINVAL;

	strlcpy(f->description, mxc_formats[i].name, sizeof(f->description));
	f->pixelformat = mxc_formats[i].fourcc;

	return 0;
}

static void print_output(void *addr)
{
	int i;

	for (i = 0x0; i < 0x200; i += 4) {
		printk(KERN_DEBUG "%02x %02x %02x %02x\n", ((char *)addr)[i],
		       ((char *)addr)[i+1], ((char *)addr)[i+2],
		       ((char *)addr)[i+3]);
	}
}

void mxc_jpeg_addrs(struct mxc_jpeg_desc *desc, struct vb2_buffer *b_base0_buf,
struct vb2_buffer *bufbase_buf, int offset)
{
	u32 buf_base0, bufbase;

	buf_base0 = vb2_dma_contig_plane_dma_addr(b_base0_buf, 0);
	bufbase = vb2_dma_contig_plane_dma_addr(bufbase_buf, 0);
	mxc_jpeg_set_addrs(desc, buf_base0, bufbase + offset);
}

static irqreturn_t mxc_jpeg_dec_irq(int irq, void *priv)
{
	struct mxc_jpeg_dev *jpeg = priv;
	struct mxc_jpeg_ctx *ctx;
	void __iomem *reg = jpeg->base_reg;
	struct vb2_buffer *src_buf, *dst_buf;
	struct mxc_jpeg_src_buf *jpeg_src_buf;
	enum vb2_buffer_state buf_state = VB2_BUF_STATE_ERROR;
	struct mxc_jpeg_desc *desc;
	dma_addr_t dma_handle;
	u32 dec_ret, offset, slot;
	struct mxc_jpeg_q_data *q_data;
	void *testaddri;
	void *testaddro;

	dec_ret = readl(reg + 0x10000);
	writel(dec_ret, reg + 0x10000);
	if (!(dec_ret & 0x8))
		return IRQ_HANDLED;
	ctx = v4l2_m2m_get_curr_priv(jpeg->m2m_dev);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	jpeg_src_buf = mxc_jpeg_vb2_to_srcbuf(src_buf);

	if (ctx->mode == MXC_JPEG_ENCODE &&
	    ctx->enc_state == MXC_JPEG_ENC_CONF) {
		q_data = &ctx->out_q;
		desc = jpeg_src_buf->desc;
		dma_handle = jpeg_src_buf->handle;

		testaddri = vb2_plane_vaddr(src_buf, 0);
		testaddro = vb2_plane_vaddr(dst_buf, 0);

		offset = mxc_jpeg_get_offset(jpeg->base_reg, 0);
		mxc_jpeg_addrs(desc, src_buf, dst_buf, offset);
		slot = mxc_jpeg_get_slot(jpeg->base_reg);
		mxc_jpeg_set_params(desc, mxc_jpeg_align(
				    vb2_plane_size(src_buf, 0), 1024),
				    q_data->bytesperline[0],
				    q_data->fmt->fourcc);
		mxc_jpeg_set_res(desc, q_data->w, q_data->h);
		ctx->enc_state = MXC_JPEG_ENC_DONE;
		mxc_jpeg_set_desc(dma_handle, jpeg->base_reg, slot);
		return IRQ_HANDLED;
	}
	buf_state = VB2_BUF_STATE_DONE;
	v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(src_buf), buf_state);
	v4l2_m2m_buf_done(to_vb2_v4l2_buffer(dst_buf), buf_state);
	v4l2_m2m_job_finish(jpeg->m2m_dev, ctx->fh.m2m_ctx);
	return IRQ_HANDLED;
}
void mxc_jpeg_config_enc(struct vb2_buffer *out_buf,
			 struct mxc_jpeg_src_buf *jpeg_src_buf,
			 struct mxc_jpeg_dev *jpeg)
{
	dma_addr_t jpg_handle;

	jpg_handle = vb2_dma_contig_plane_dma_addr(out_buf, 0);
	jpeg_src_buf->tbl = dma_zalloc_coherent(jpeg->dev,
				  sizeof(unsigned char) * 615,
				  &jpeg_src_buf->tbl_handle, 0);
	memcpy(jpeg_src_buf->tbl, &hactbl, sizeof(unsigned char) * 615);

	mxc_jpeg_enc_config(jpeg->base_reg, jpeg_src_buf->desc, jpeg_src_buf->handle,
			    jpeg_src_buf->tbl_handle, jpg_handle);

}
static void mxc_jpeg_device_run(void *priv)
{
	struct mxc_jpeg_ctx *ctx = priv;
	struct mxc_jpeg_dev *jpeg = ctx->mxc_jpeg;
	struct vb2_buffer *src_buf, *dst_buf;
	unsigned long flags;
	struct mxc_jpeg_q_data *cap_q_data = &ctx->cap_q;
	struct mxc_jpeg_src_buf *jpeg_src_buf;
	int slot;
	void *testaddri;
	void *testaddro;
	dma_addr_t dma_handle;
	struct mxc_jpeg_desc *desc;


	src_buf = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	dst_buf = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	jpeg_src_buf = mxc_jpeg_vb2_to_srcbuf(src_buf);
	desc = jpeg_src_buf->desc;
	dma_handle = jpeg_src_buf->handle;


	testaddri = vb2_plane_vaddr(src_buf, 0);
	testaddro = vb2_plane_vaddr(dst_buf, 0);
	spin_lock_irqsave(&ctx->mxc_jpeg->hw_lock, flags);
	mxc_jpeg_enable(jpeg->base_reg);

	if (ctx->mode == MXC_JPEG_ENCODE) {
		ctx->enc_state = MXC_JPEG_ENC_CONF;
		mxc_jpeg_config_enc(dst_buf, jpeg_src_buf, jpeg);
	} else {
		mxc_jpeg_addrs(desc, dst_buf, src_buf, 0);
		slot = mxc_jpeg_get_slot(jpeg->base_reg);
		mxc_jpeg_enable_slot(jpeg->base_reg, slot);
		mxc_jpeg_set_params(desc,
				    mxc_jpeg_align(vb2_plane_size(src_buf, 0),
						   1024),
				    cap_q_data->bytesperline[0],
				    cap_q_data->fmt->fourcc);
		mxc_jpeg_enable_irq(jpeg->base_reg, slot);
		mxc_jpeg_set_desc(dma_handle, jpeg->base_reg, slot);
		mxc_jpeg_go(jpeg->base_reg);
	}
	//print_descriptor_info(desc);
	////print_output(testaddro);
	//print_cast_decoder_info(jpeg->base_reg);
	spin_unlock_irqrestore(&ctx->mxc_jpeg->hw_lock, flags);
}
static int mxc_jpeg_job_ready(void *priv)
{
	return 1;
}
static void mxc_jpeg_job_abort(void *priv)
{
}

static struct mxc_jpeg_q_data *mxc_jpeg_get_q_data(struct mxc_jpeg_ctx *ctx,
						   enum v4l2_buf_type type)
{
	if (V4L2_TYPE_IS_OUTPUT(type))
		return &ctx->out_q;
	return &ctx->cap_q;
}

static int mxc_jpeg_queue_setup(struct vb2_queue *q,
				unsigned int *num_buffers,
				unsigned int *num_planes,
				unsigned int sizes[],
				struct device *alloc_ctxs[])
{
	struct mxc_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	struct mxc_jpeg_q_data *q_data = NULL;

	q_data = mxc_jpeg_get_q_data(ctx, q->type);
	if (!q_data)
		return -EINVAL;
	*num_planes = 1;
	sizes[0] = 10;

	if (q_data->sizeimage[0] > 0)
		sizes[0] = q_data->sizeimage[0];

	return 0;
}
static int mxc_jpeg_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mxc_jpeg_ctx *ctx = vb2_get_drv_priv(q);
	int ret;

	ret = pm_runtime_get_sync(ctx->mxc_jpeg->dev);
	return ret > 0 ? 0 : ret;
}

static void mxc_jpeg_stop_streaming(struct vb2_queue *q)
{
	struct mxc_jpeg_ctx *ctx = vb2_get_drv_priv(q);

	pm_runtime_put(&ctx->mxc_jpeg->pdev->dev);
}
struct mxc_jpeg_stream {
	u8 *addr;
	u32 loc;
	u32 end;
};
static u8 get_byte(struct mxc_jpeg_stream *stream)
{
	u8 ret;

	if (stream->loc >= stream->end)
		return -1;
	ret = stream->addr[stream->loc];
	stream->loc++;
	return ret;
}
static u16 get_word_be(struct mxc_jpeg_stream *stream, u32 *word)
{
	u16 ret1;
	u16 ret2;

	ret1 = get_byte(stream);
	ret2 = get_byte(stream);
	if (ret1 != -1 && ret2 != -1) {
		*word = ((ret1 << 8) | ret2);
		return 0;
	}
	return -1;
}
static int mxc_jpeg_parse(struct mxc_jpeg_desc *desc, u8 *src_addr, u32 size)
{
	struct mxc_jpeg_stream stream;
	bool notfound = true;
	int height, width, byte, length, word;

	stream.addr = src_addr;
	stream.end = size;
	stream.loc = 0;
	while (notfound) {
		byte = get_byte(&stream);
		if (byte == -1)
			return -EINVAL;
		if (byte != 0xff)
			continue;
		do {
			byte = get_byte(&stream);
		} while (byte == 0xff);
		if (byte == -1)
			return false;
		if (byte == 0)
			continue;
		length = 0;
		switch (byte) {
		case SOF2:
		case SOF0:
			if (get_word_be(&stream, &word))
				break;
			length = (long)word - 2;
			if (!length)
				return false;
			if (get_byte(&stream) == -1)
				break;
			if (get_word_be(&stream, &height))
				break;
			if (get_word_be(&stream, &width))
				break;
			notfound = false;
			break;
		default:
			notfound = true;
		}
	}
	if (width % 8 != 0 || height % 8 != 0)
		return -EINVAL;
	desc->w = width;
	desc->h = height;
	return 0;
}
static void mxc_jpeg_buf_queue(struct vb2_buffer *vb)
{
	int ret;
	struct mxc_jpeg_src_buf *jpeg_src_buf;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct mxc_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	if (vb->vb2_queue->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		goto end;
	jpeg_src_buf = container_of(vbuf, struct mxc_jpeg_src_buf, b);
	jpeg_src_buf->desc = dma_zalloc_coherent(ctx->mxc_jpeg->dev,
					     sizeof(struct mxc_jpeg_desc),
					     &jpeg_src_buf->handle, 0);
	if (ctx->mode != MXC_JPEG_DECODE)
		goto end;
	ret = mxc_jpeg_parse(jpeg_src_buf->desc, (u8 *)vb2_plane_vaddr(vb, 0),
			     vb2_get_plane_payload(vb, 0));
	if (ret) {
		v4l2_err(&ctx->mxc_jpeg->v4l2_dev,
			 "driver does not support this resolution\n");
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
		return;
	}

	if (ctx->state == MXC_JPEG_INIT) {
		struct vb2_queue *dst_vq = v4l2_m2m_get_vq(
			ctx->fh.m2m_ctx, V4L2_BUF_TYPE_VIDEO_OUTPUT);
		static const struct v4l2_event ev_src_ch = {
			.type = V4L2_EVENT_SOURCE_CHANGE,
			.u.src_change.changes =
			V4L2_EVENT_SRC_CH_RESOLUTION,
		};

		v4l2_event_queue_fh(&ctx->fh, &ev_src_ch);
		if (vb2_is_streaming(dst_vq))
			ctx->state = MXC_JPEG_RUNNING;
	}

end:
	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, vbuf);
}
static int mxc_jpeg_buf_prepare(struct vb2_buffer *vb)
{
	struct mxc_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mxc_jpeg_q_data *q_data = NULL;

	q_data = mxc_jpeg_get_q_data(ctx, vb->vb2_queue->type);
	if (!q_data)
		return -EINVAL;

	vb2_set_plane_payload(vb, 0, q_data->sizeimage[0]);
	return 0;
}

static void mxc_jpeg_buf_clean(struct vb2_buffer *vb)
{
	struct mxc_jpeg_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct mxc_jpeg_src_buf *jpeg_src_buf;

	jpeg_src_buf = container_of(vbuf, struct mxc_jpeg_src_buf, b);

	if (ctx->mode == MXC_JPEG_ENCODE &&
		ctx->enc_state == MXC_JPEG_ENC_CONF)
			dma_free_coherent(ctx->mxc_jpeg->dev, sizeof(hactbl),
					  jpeg_src_buf->tbl, jpeg_src_buf->tbl_handle);

	dma_free_coherent(ctx->mxc_jpeg->dev, sizeof(struct mxc_jpeg_desc),
			  jpeg_src_buf->desc, jpeg_src_buf->handle);
}

static const struct vb2_ops mxc_jpeg_qops = {
	.queue_setup		= mxc_jpeg_queue_setup,

	.buf_prepare		= mxc_jpeg_buf_prepare,
	.buf_queue		= mxc_jpeg_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= mxc_jpeg_start_streaming,
	.stop_streaming		= mxc_jpeg_stop_streaming,
	.buf_cleanup		= mxc_jpeg_buf_clean,
};
static int mxc_jpeg_queue_init(void *priv, struct vb2_queue *src_vq,
			       struct vb2_queue *dst_vq)
{
	struct mxc_jpeg_ctx *ctx = priv;
	int ret;

	src_vq->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	src_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	src_vq->drv_priv = ctx;
	src_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	src_vq->ops = &mxc_jpeg_qops;
	src_vq->mem_ops = &vb2_dma_contig_memops;
	src_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	src_vq->lock = &ctx->mxc_jpeg->lock;
	src_vq->dev = ctx->mxc_jpeg->dev;

	ret = vb2_queue_init(src_vq);
	if (ret)
		return ret;

	dst_vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	dst_vq->io_modes = VB2_MMAP | VB2_USERPTR;
	dst_vq->drv_priv = ctx;
	dst_vq->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dst_vq->ops = &mxc_jpeg_qops;
	dst_vq->mem_ops = &vb2_dma_contig_memops;
	dst_vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dst_vq->lock = &ctx->mxc_jpeg->lock;
	dst_vq->dev = ctx->mxc_jpeg->dev;

	ret = vb2_queue_init(dst_vq);
	return ret;
}

static struct mxc_jpeg_fmt *mxc_jpeg_find_format(struct mxc_jpeg_ctx *ctx,
						 u32 pixelformat,
						 unsigned int fmt_type)
{
	unsigned int k, fmt_flag;

	fmt_flag = fmt_type;

	if (ctx->mode == MXC_JPEG_ENCODE)
		fmt_flag = (fmt_type == MXC_JPEG_FMT_TYPE_RAW) ?
			MXC_JPEG_FMT_TYPE_ENC :
			MXC_JPEG_FMT_TYPE_RAW;

	for (k = 0; k < MXC_JPEG_NUM_FORMATS; k++) {
		struct mxc_jpeg_fmt *fmt = &mxc_formats[k];

		if (fmt->fourcc == pixelformat && fmt->flags == fmt_flag)
			return fmt;
	}
	return NULL;
}


static int mxc_jpeg_open(struct file *file)
{
	struct mxc_jpeg_dev *mxc_jpeg = video_drvdata(file);
	struct video_device *mxc_vfd = video_devdata(file);
	struct mxc_jpeg_ctx *ctx;
	struct mxc_jpeg_fmt *out_fmt, *cap_fmt;
	int ret = 0;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	if (mutex_lock_interruptible(&mxc_jpeg->lock)) {
		ret = -ERESTARTSYS;
		goto free;
	}

	v4l2_fh_init(&ctx->fh, mxc_vfd);
	file->private_data = &ctx->fh;
	v4l2_fh_add(&ctx->fh);

	ctx->mxc_jpeg = mxc_jpeg;
	if (mxc_jpeg->mode == MXC_JPEG_ENCODE) {
		ctx->mode = MXC_JPEG_ENCODE;
		cap_fmt = mxc_jpeg_find_format(ctx, V4L2_PIX_FMT_RGB24,
							MXC_JPEG_FMT_TYPE_ENC);
		out_fmt = mxc_jpeg_find_format(ctx, V4L2_PIX_FMT_JPEG,
							MXC_JPEG_FMT_TYPE_RAW);
	} else {
		ctx->mode = MXC_JPEG_DECODE;
		out_fmt = mxc_jpeg_find_format(ctx, V4L2_PIX_FMT_RGB24,
							MXC_JPEG_FMT_TYPE_RAW);
		cap_fmt = mxc_jpeg_find_format(ctx, V4L2_PIX_FMT_JPEG,
							MXC_JPEG_FMT_TYPE_ENC);
	}
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(mxc_jpeg->m2m_dev, ctx,
					    mxc_jpeg_queue_init);
	ctx->out_q.fmt = out_fmt;
	ctx->cap_q.fmt = cap_fmt;
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		goto error;
	}



	mutex_unlock(&mxc_jpeg->lock);
	return 0;

error:
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	mutex_unlock(&mxc_jpeg->lock);
free:
	kfree(ctx);
	return ret;
}

static int mxc_jpeg_querycap(struct file *file, void *priv,
			     struct v4l2_capability *cap)
{
	struct mxc_jpeg_dev *mxc_jpeg = video_drvdata(file);

	strlcpy(cap->driver, MXC_JPEG_NAME " decoder", sizeof(cap->driver));
	strlcpy(cap->card, MXC_JPEG_NAME " decoder", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(mxc_jpeg->dev));
	cap->device_caps = V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;

	return 0;
}
static int mxc_jpeg_enum_fmt_vid_cap(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	return enum_fmt(mxc_formats, MXC_NUM_FORMATS, f,
				 MXC_IN_FORMAT);
}

static int mxc_jpeg_enum_fmt_vid_out(struct file *file, void *priv,
				     struct v4l2_fmtdesc *f)
{
	return enum_fmt(mxc_formats, MXC_NUM_FORMATS, f,
				 MXC_OUT_FORMAT);
}
static void mxc_jpeg_bound_align_image(u32 *w, unsigned int wmin,
				       unsigned int wmax, unsigned int walign,
				       u32 *h, unsigned int hmin,
				       unsigned int hmax, unsigned int halign)
{
	int width, height, w_step, h_step;

	width = *w;
	height = *h;
	w_step = 1 << walign;
	h_step = 1 << halign;

	v4l_bound_align_image(w, wmin, wmax, walign, h, hmin, hmax, halign, 0);
	if (*w < width && (*w + w_step) <= wmax)
		*w += w_step;
	if (*h < height && (*h + h_step) <= hmax)
		*h += h_step;
}
static int mxc_jpeg_try_fmt(struct v4l2_format *f, struct mxc_jpeg_fmt *fmt,
			    struct mxc_jpeg_ctx *ctx, int q_type)
{
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct v4l2_plane_pix_format *pfmt = &pix_mp->plane_fmt[0];
	u32 stride, h;

	memset(pix_mp->reserved, 0, sizeof(pix_mp->reserved));
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->num_planes = fmt->colplanes;
	pix_mp->pixelformat = fmt->fourcc;

	if (q_type == MXC_JPEG_FMT_TYPE_ENC && ctx->mode == MXC_JPEG_DECODE) {
		struct v4l2_plane_pix_format *pfmt = &pix_mp->plane_fmt[0];

		mxc_jpeg_bound_align_image(&pix_mp->width,
					   MXC_JPEG_MIN_WIDTH,
					   MXC_JPEG_MAX_WIDTH,
					   MXC_JPEG_MIN_WIDTH,
					   &pix_mp->height,
					   MXC_JPEG_MIN_HEIGHT,
					   MXC_JPEG_MAX_HEIGHT,
					   MXC_JPEG_MIN_WIDTH);

		memset(pfmt->reserved, 0, sizeof(pfmt->reserved));
		pfmt->bytesperline = 0;
		/* Source size must be aligned to 128 */
		pfmt->sizeimage = mxc_jpeg_align(pfmt->sizeimage, 128);
		if (pfmt->sizeimage == 0)
			pfmt->sizeimage = MXC_JPEG_DEFAULT_SIZEIMAGE;
		return 0;
	}

	/* type is MXC_JPEG_FMT_TYPE_RAW */
	mxc_jpeg_bound_align_image(&pix_mp->width, MXC_JPEG_MIN_WIDTH,
				   MXC_JPEG_MAX_WIDTH, MXC_JPEG_MIN_WIDTH,
				   &pix_mp->height, MXC_JPEG_MIN_HEIGHT,
				   MXC_JPEG_MAX_HEIGHT, MXC_JPEG_MIN_WIDTH);

	pfmt = &pix_mp->plane_fmt[0];
	stride = pix_mp->width * 3;
	h = pix_mp->height;
	memset(pfmt->reserved, 0, sizeof(pfmt->reserved));
	pfmt->bytesperline = stride;
	pfmt->sizeimage = stride * h;

	return 0;
}
static int mxc_jpeg_try_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mxc_jpeg_ctx *ctx = mxc_jpeg_fh_to_ctx(priv);
	struct mxc_jpeg_fmt *fmt;

	fmt = mxc_jpeg_find_format(ctx, f->fmt.pix_mp.pixelformat,
				   MXC_JPEG_FMT_TYPE_RAW);
	return mxc_jpeg_try_fmt(f, fmt, ctx, MXC_JPEG_FMT_TYPE_RAW);
}
static int mxc_jpeg_try_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	struct mxc_jpeg_ctx *ctx = mxc_jpeg_fh_to_ctx(priv);
	struct mxc_jpeg_fmt *fmt;

	fmt = mxc_jpeg_find_format(ctx, f->fmt.pix_mp.pixelformat,
				   MXC_JPEG_FMT_TYPE_ENC);
	return mxc_jpeg_try_fmt(f, fmt, ctx, MXC_JPEG_FMT_TYPE_ENC);
}
static int mxc_jpeg_s_fmt(struct mxc_jpeg_ctx *ctx,
			  struct v4l2_format *f)
{
	struct vb2_queue *vq;
	struct mxc_jpeg_q_data *q_data = NULL;
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	struct mxc_jpeg_dev *jpeg = ctx->mxc_jpeg;
	unsigned int f_type;

	vq = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	if (!vq)
		return -EINVAL;

	q_data = mxc_jpeg_get_q_data(ctx, f->type);

	if (vb2_is_busy(vq)) {
		v4l2_err(&jpeg->v4l2_dev, "queue busy\n");
		return -EBUSY;
	}

	f_type = f->type;

	q_data->fmt = mxc_jpeg_find_format(ctx, pix_mp->pixelformat, f_type);
	q_data->w = pix_mp->width;
	q_data->h = pix_mp->height;
	q_data->bytesperline[0] = pix_mp->plane_fmt[0].bytesperline;
	q_data->sizeimage[0] = pix_mp->plane_fmt[0].sizeimage;

	return 0;
}
static int mxc_jpeg_s_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = mxc_jpeg_try_fmt_vid_cap(file, priv, f);
	if (ret)
		return ret;

	return mxc_jpeg_s_fmt(mxc_jpeg_fh_to_ctx(priv), f);
}
static int mxc_jpeg_s_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	int ret;

	ret = mxc_jpeg_try_fmt_vid_out(file, priv, f);
	if (ret)
		return ret;

	return mxc_jpeg_s_fmt(mxc_jpeg_fh_to_ctx(priv), f);
}
static int mxc_jpeg_g_fmt_vid_cap(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return 0;
}
static int mxc_jpeg_g_fmt_vid_out(struct file *file, void *priv,
				struct v4l2_format *f)
{
	return 0;
}

static int mxc_jpeg_qbuf(struct file *file, void *priv, struct v4l2_buffer *buf)
{
	struct v4l2_fh *fh = file->private_data;
	struct mxc_jpeg_ctx *ctx = mxc_jpeg_fh_to_ctx(priv);
	struct vb2_queue *vq;
	struct vb2_buffer *vb;
	struct mxc_jpeg_src_buf *jpeg_src_buf;

	if (buf->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
		goto end;

	vq = v4l2_m2m_get_vq(fh->m2m_ctx, buf->type);
	if (buf->index >= vq->num_buffers) {
		dev_err(ctx->mxc_jpeg->dev, "buffer index out of range\n");
		return -EINVAL;
	}

	vb = vq->bufs[buf->index];
	jpeg_src_buf = mxc_jpeg_vb2_to_srcbuf(vb);
end:
	return v4l2_m2m_qbuf(file, fh->m2m_ctx, buf);
}

static const struct v4l2_ioctl_ops mxc_jpeg_ioctl_ops = {
	.vidioc_querycap		= mxc_jpeg_querycap,
	.vidioc_enum_fmt_vid_cap	= mxc_jpeg_enum_fmt_vid_cap,
	.vidioc_enum_fmt_vid_out	= mxc_jpeg_enum_fmt_vid_out,

	.vidioc_try_fmt_vid_cap		= mxc_jpeg_try_fmt_vid_cap,
	.vidioc_try_fmt_vid_out		= mxc_jpeg_try_fmt_vid_out,

	.vidioc_s_fmt_vid_cap		= mxc_jpeg_s_fmt_vid_cap,
	.vidioc_s_fmt_vid_out		= mxc_jpeg_s_fmt_vid_out,

	.vidioc_g_fmt_vid_cap		= mxc_jpeg_g_fmt_vid_cap,
	.vidioc_g_fmt_vid_out		= mxc_jpeg_g_fmt_vid_out,

	.vidioc_qbuf			= mxc_jpeg_qbuf,

	.vidioc_create_bufs		= v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf		= v4l2_m2m_ioctl_prepare_buf,
	.vidioc_reqbufs                 = v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf                = v4l2_m2m_ioctl_querybuf,
	.vidioc_dqbuf                   = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf                  = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon                = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff               = v4l2_m2m_ioctl_streamoff,
};

static int mxc_jpeg_release(struct file *file)
{
	struct mxc_jpeg_dev *mxc_jpeg = video_drvdata(file);
	struct mxc_jpeg_ctx *ctx = mxc_jpeg_fh_to_ctx(file->private_data);

	mutex_lock(&mxc_jpeg->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	v4l2_fh_del(&ctx->fh);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	mutex_unlock(&mxc_jpeg->lock);
	return 0;
}

static const struct v4l2_file_operations mxc_jpeg_fops = {
	.owner		= THIS_MODULE,
	.open		= mxc_jpeg_open,
	.release	= mxc_jpeg_release,
	.poll		= v4l2_m2m_fop_poll,
	.unlocked_ioctl	= video_ioctl2,
	.mmap		= v4l2_m2m_fop_mmap,
};
static struct v4l2_m2m_ops mxc_jpeg_m2m_ops = {
	.device_run	= mxc_jpeg_device_run,
	.job_ready	= mxc_jpeg_job_ready,
	.job_abort	= mxc_jpeg_job_abort,
};

static int mxc_jpeg_probe(struct platform_device *pdev)
{
	struct mxc_jpeg_dev *jpeg;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int dec_irq;
	int ret;
	int mode;
	const struct of_device_id *of_id;

	of_id = of_match_node(mxc_jpeg_match, dev->of_node);
	mode = (int)(u64) of_id->data;

	jpeg = devm_kzalloc(dev, sizeof(struct mxc_jpeg_dev), GFP_KERNEL);
	if (!jpeg)
		return -ENOMEM;

	mutex_init(&jpeg->lock);
	spin_lock_init(&jpeg->hw_lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dec_irq = platform_get_irq(pdev, 0);
	if (!res || dec_irq < 0) {
		dev_err(&pdev->dev, "Failed to get dec_irq %d.\n", dec_irq);
		ret = -EINVAL;
		goto err_irq;
	}
	ret = devm_request_irq(&pdev->dev, dec_irq, mxc_jpeg_dec_irq, 0,
			       pdev->name, jpeg);
	if (ret) {
		dev_err(&pdev->dev, "Failed to request dec_irq %d (%d)\n",
			dec_irq, ret);
		ret = -EINVAL;
		goto err_irq;
	}
	jpeg->base_reg = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(jpeg->base_reg))
		return PTR_ERR(jpeg->base_reg);

	jpeg->pdev = pdev;
	jpeg->dev = dev;
	jpeg->mode = mode;

	/* Start clock */
	jpeg->clk = devm_clk_get(dev, "ipg");
	if (IS_ERR(jpeg->clk)) {
		dev_err(dev, "failed to get clock: ipg\n");
		goto err_clk;
	}
	ret = clk_prepare_enable(jpeg->clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock: ipg\n");
		goto err_clk;
	}
	jpeg->clk = devm_clk_get(dev, "per");
	if (IS_ERR(jpeg->clk)) {
		dev_err(dev, "failed to get clock: per\n");
		goto err_clk;
	}
	ret = clk_prepare_enable(jpeg->clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable clock: per\n");
		goto err_clk;
	}
	/* v4l2 */
	ret = v4l2_device_register(dev, &jpeg->v4l2_dev);
	if (ret) {
		dev_err(dev, "failed to register v4l2 device\n");
		goto err_register;
	}
	jpeg->m2m_dev = v4l2_m2m_init(&mxc_jpeg_m2m_ops);
	if (IS_ERR(jpeg->m2m_dev)) {
		dev_err(dev, "failed to register v4l2 device\n");
		goto err_m2m;
	}

	jpeg->dec_vdev = video_device_alloc();
	if (!jpeg->dec_vdev) {
		dev_err(dev, "failed to register v4l2 device\n");
		goto err_vdev_alloc;
	}
	if (mode == MXC_JPEG_ENCODE)
		snprintf(jpeg->dec_vdev->name,
				sizeof(jpeg->dec_vdev->name),
				"%s-enc", MXC_JPEG_M2M_NAME);
	else
		snprintf(jpeg->dec_vdev->name,
				sizeof(jpeg->dec_vdev->name),
				"%s-dec", MXC_JPEG_M2M_NAME);

	jpeg->dec_vdev->fops = &mxc_jpeg_fops;
	jpeg->dec_vdev->ioctl_ops = &mxc_jpeg_ioctl_ops;
	jpeg->dec_vdev->minor = -1;
	jpeg->dec_vdev->release = video_device_release;
	jpeg->dec_vdev->lock = &jpeg->lock;
	jpeg->dec_vdev->v4l2_dev = &jpeg->v4l2_dev;
	jpeg->dec_vdev->vfl_dir = VFL_DIR_M2M;
	jpeg->dec_vdev->device_caps = V4L2_CAP_STREAMING |
					V4L2_CAP_VIDEO_M2M;

	ret = video_register_device(jpeg->dec_vdev, VFL_TYPE_GRABBER, -1);
	if (ret) {
		dev_err(dev, "failed to register video device\n");
		goto err_vdev_register;
	}
	video_set_drvdata(jpeg->dec_vdev, jpeg);
	if (mode == MXC_JPEG_ENCODE)
		v4l2_info(&jpeg->v4l2_dev,
			"encoder device registered as /dev/video%d (%d,%d)\n",
			jpeg->dec_vdev->num, VIDEO_MAJOR,
			jpeg->dec_vdev->minor);
	else
		v4l2_info(&jpeg->v4l2_dev,
			"decoder device registered as /dev/video%d (%d,%d)\n",
			jpeg->dec_vdev->num, VIDEO_MAJOR,
			jpeg->dec_vdev->minor);

	platform_set_drvdata(pdev, jpeg);
	pm_runtime_enable(dev);
	return 0;

err_vdev_register:
	video_device_release(jpeg->dec_vdev);

err_vdev_alloc:
	v4l2_m2m_release(jpeg->m2m_dev);

err_m2m:
	v4l2_device_unregister(&jpeg->v4l2_dev);

err_register:
err_irq:
err_clk:
	return ret;
}

static int mxc_jpeg_remove(struct platform_device *pdev)
{
	struct mxc_jpeg_dev *jpeg = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	video_unregister_device(jpeg->dec_vdev);
	video_device_release(jpeg->dec_vdev);
	v4l2_m2m_release(jpeg->m2m_dev);
	v4l2_device_unregister(&jpeg->v4l2_dev);
	return 0;
}

MODULE_DEVICE_TABLE(of, mxc_jpeg_match);

static struct platform_driver mxc_jpeg_driver = {
	.probe = mxc_jpeg_probe,
	.remove = mxc_jpeg_remove,
	.driver = {
		.name = "mxc-jpeg",
		.of_match_table = mxc_jpeg_match,
	},
};
module_platform_driver(mxc_jpeg_driver);
MODULE_LICENSE("GPL v2");

