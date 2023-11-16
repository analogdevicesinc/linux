// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019-2020 NXP
 *
 */
#include <dt-bindings/pinctrl/pads-imx8qxp.h>

#include <linux/module.h>
#include "imx8-isi-hw.h"
#include "imx8-common.h"

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("IMX8 Image Sensor Interface Hardware driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

#define	ISI_DOWNSCALE_THRESHOLD		0x4000

#ifdef DEBUG
void dump_isi_regs(struct mxc_isi_dev *mxc_isi)
{
	struct device *dev = &mxc_isi->pdev->dev;
	struct {
		u32 offset;
		const char *const name;
	} registers[] = {
		{ 0x00, "CHNL_CTRL" },
		{ 0x04, "CHNL_IMG_CTRL" },
		{ 0x08, "CHNL_OUT_BUF_CTRL" },
		{ 0x0C, "CHNL_IMG_CFG" },
		{ 0x10, "CHNL_IER" },
		{ 0x14, "CHNL_STS" },
		{ 0x18, "CHNL_SCALE_FACTOR" },
		{ 0x1C, "CHNL_SCALE_OFFSET" },
		{ 0x20, "CHNL_CROP_ULC" },
		{ 0x24, "CHNL_CROP_LRC" },
		{ 0x28, "CHNL_CSC_COEFF0" },
		{ 0x2C, "CHNL_CSC_COEFF1" },
		{ 0x30, "CHNL_CSC_COEFF2" },
		{ 0x34, "CHNL_CSC_COEFF3" },
		{ 0x38, "CHNL_CSC_COEFF4" },
		{ 0x3C, "CHNL_CSC_COEFF5" },
		{ 0x40, "CHNL_ROI_0_ALPHA" },
		{ 0x44, "CHNL_ROI_0_ULC" },
		{ 0x48, "CHNL_ROI_0_LRC" },
		{ 0x4C, "CHNL_ROI_1_ALPHA" },
		{ 0x50, "CHNL_ROI_1_ULC" },
		{ 0x54, "CHNL_ROI_1_LRC" },
		{ 0x58, "CHNL_ROI_2_ALPHA" },
		{ 0x5C, "CHNL_ROI_2_ULC" },
		{ 0x60, "CHNL_ROI_2_LRC" },
		{ 0x64, "CHNL_ROI_3_ALPHA" },
		{ 0x68, "CHNL_ROI_3_ULC" },
		{ 0x6C, "CHNL_ROI_3_LRC" },
		{ 0x70, "CHNL_OUT_BUF1_ADDR_Y" },
		{ 0x74, "CHNL_OUT_BUF1_ADDR_U" },
		{ 0x78, "CHNL_OUT_BUF1_ADDR_V" },
		{ 0x7C, "CHNL_OUT_BUF_PITCH" },
		{ 0x80, "CHNL_IN_BUF_ADDR" },
		{ 0x84, "CHNL_IN_BUF_PITCH" },
		{ 0x88, "CHNL_MEM_RD_CTRL" },
		{ 0x8C, "CHNL_OUT_BUF2_ADDR_Y" },
		{ 0x90, "CHNL_OUT_BUF2_ADDR_U" },
		{ 0x94, "CHNL_OUT_BUF2_ADDR_V" },
		{ 0x98, "CHNL_SCL_IMG_CFG" },
		{ 0x9C, "CHNL_FLOW_CTRL" },
	};
	u32 i;

	dev_dbg(dev, "ISI CHNLC register dump, isi%d\n", mxc_isi->id);
	for (i = 0; i < ARRAY_SIZE(registers); i++) {
		u32 reg = readl(mxc_isi->regs + registers[i].offset);
		dev_dbg(dev, "%20s[0x%.2x]: %.2x\n",
			registers[i].name, registers[i].offset, reg);
	}
}
#else
void dump_isi_regs(struct mxc_isi_dev *mxc_isi)
{
}
#endif

/* 
 * A2,A1,      B1, A3,     B3, B2,
 * C2, C1,     D1, C3,     D3, D2
 */
static const u32 coeffs[2][6] = {
	/* YUV2RGB */
	{ 0x0000012A, 0x012A0198, 0x0730079C,
	  0x0204012A, 0x01F00000, 0x01800180 },

	/* RGB->YUV */
	{ 0x00810041, 0x07db0019, 0x007007b6,
	  0x07a20070, 0x001007ee, 0x00800080 },
};

static void printk_pixelformat(char *prefix, int val)
{
	pr_info("%s %c%c%c%c\n", prefix ? prefix : "pixelformat",
		val & 0xff,
		(val >> 8)  & 0xff,
		(val >> 16) & 0xff,
		(val >> 24) & 0xff);
}

static bool is_rgb(u32 pix_fmt)
{
	if ((pix_fmt == V4L2_PIX_FMT_RGB565) ||
	    (pix_fmt == V4L2_PIX_FMT_RGB24)  ||
	    (pix_fmt == V4L2_PIX_FMT_RGB32)  ||
	    (pix_fmt == V4L2_PIX_FMT_BGR32)  ||
	    (pix_fmt == V4L2_PIX_FMT_XRGB32) ||
	    (pix_fmt == V4L2_PIX_FMT_XBGR32) ||
	    (pix_fmt == V4L2_PIX_FMT_BGR24)  ||
	    (pix_fmt == V4L2_PIX_FMT_RGBA32) ||
	    (pix_fmt == V4L2_PIX_FMT_ABGR32) ||
	    (pix_fmt == V4L2_PIX_FMT_ARGB32))
		return true;
	else
		return false;
}

static bool is_yuv(u32 pix_fmt)
{
	if ((pix_fmt == V4L2_PIX_FMT_YUYV)  ||
	    (pix_fmt == V4L2_PIX_FMT_YUV32) ||
	    (pix_fmt == V4L2_PIX_FMT_YUV444M) ||
	    (pix_fmt == V4L2_PIX_FMT_NV12) ||
	    (pix_fmt == V4L2_PIX_FMT_NV12M))
		return true;
	else
		return false;
}

bool is_buf_active(struct mxc_isi_dev *mxc_isi, int buf_id)
{
	u32 status = mxc_isi->status;
	bool reverse = mxc_isi->buf_active_reverse;

	return (buf_id == 1) ? ((reverse) ? (status & 0x100) : (status & 0x200)) :
			       ((reverse) ? (status & 0x200) : (status & 0x100));
}
EXPORT_SYMBOL_GPL(is_buf_active);

static void chain_buf(struct mxc_isi_dev *mxc_isi, struct mxc_isi_frame *frm)
{
	u32 val;

	if (frm->o_width > ISI_2K) {
		val = readl(mxc_isi->regs + CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		val |= (CHNL_CTRL_CHAIN_BUF_2_CHAIN << CHNL_CTRL_CHAIN_BUF_OFFSET);
		writel(val, mxc_isi->regs + CHNL_CTRL);
		if (mxc_isi->chain)
			regmap_write(mxc_isi->chain, CHNL_CTRL, CHNL_CTRL_CLK_EN_MASK);
		mxc_isi->chain_buf = 1;
	} else {
		val = readl(mxc_isi->regs + CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		writel(val, mxc_isi->regs + CHNL_CTRL);
		mxc_isi->chain_buf = 0;
	}
}

struct device *mxc_isi_dev_get_parent(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *parent;
	struct platform_device *parent_pdev;

	if (!pdev)
		return NULL;

	/* Get parent for isi capture device */
	parent = of_get_parent(dev->of_node);
	parent_pdev = of_find_device_by_node(parent);
	if (!parent_pdev) {
		of_node_put(parent);
		return NULL;
	}
	of_node_put(parent);

	return &parent_pdev->dev;
}
EXPORT_SYMBOL_GPL(mxc_isi_dev_get_parent);

struct mxc_isi_dev *mxc_isi_get_hostdata(struct platform_device *pdev)
{
	struct mxc_isi_dev *mxc_isi;

	if (!pdev || !pdev->dev.parent)
		return NULL;

	mxc_isi = (struct mxc_isi_dev *)dev_get_drvdata(pdev->dev.parent);
	if (!mxc_isi)
		return NULL;

	return mxc_isi;
}
EXPORT_SYMBOL_GPL(mxc_isi_get_hostdata);

void mxc_isi_channel_set_outbuf_loc(struct mxc_isi_dev *mxc_isi,
				    struct mxc_isi_buffer *buf)
{
	struct vb2_buffer *vb2_buf = &buf->v4l2_buf.vb2_buf;
	u32 framecount = buf->v4l2_buf.sequence;
	dma_addr_t *dma_addrs = buf->dma_addrs;
	struct mxc_isi_cap_dev *isi_cap = mxc_isi->isi_cap;
	int val = 0, i;

	for (i = 0; i < vb2_buf->num_planes; ++i) {
		if (buf->discard)
			dma_addrs[i] = isi_cap->discard_buffer_dma[i];
		else
			dma_addrs[i] = vb2_dma_contig_plane_dma_addr(vb2_buf, i);
	}

	val = readl(mxc_isi->regs + CHNL_OUT_BUF_CTRL);

	if (framecount == 0 || ((is_buf_active(mxc_isi, 2)) && (framecount != 1))) {
		writel(dma_addrs[0], mxc_isi->regs + CHNL_OUT_BUF1_ADDR_Y);
		writel(dma_addrs[1], mxc_isi->regs + CHNL_OUT_BUF1_ADDR_U);
		writel(dma_addrs[2], mxc_isi->regs + CHNL_OUT_BUF1_ADDR_V);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR_MASK;
		buf->id = MXC_ISI_BUF1;
	} else if (framecount == 1 || is_buf_active(mxc_isi, 1)) {
		writel(dma_addrs[0], mxc_isi->regs + CHNL_OUT_BUF2_ADDR_Y);
		writel(dma_addrs[1], mxc_isi->regs + CHNL_OUT_BUF2_ADDR_U);
		writel(dma_addrs[2], mxc_isi->regs + CHNL_OUT_BUF2_ADDR_V);
		val ^= CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR_MASK;
		buf->id = MXC_ISI_BUF2;
	}
	writel(val, mxc_isi->regs + CHNL_OUT_BUF_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_set_outbuf_loc);

void mxc_isi_channel_set_m2m_src_addr(struct mxc_isi_dev *mxc_isi,
			struct mxc_isi_buffer *buf)
{
	struct vb2_buffer *vb2_buf = &buf->v4l2_buf.vb2_buf;
	dma_addr_t *dma_addrs = buf->dma_addrs;

	/* Only support one plane */
	dma_addrs[0] = vb2_dma_contig_plane_dma_addr(vb2_buf, 0);
	writel(dma_addrs[0], mxc_isi->regs + CHNL_IN_BUF_ADDR);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_set_m2m_src_addr);

void mxc_isi_channel_sw_reset(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_CTRL);
	val |= CHNL_CTRL_SW_RST;
	writel(val, mxc_isi->regs + CHNL_CTRL);
	mdelay(5);
	val &= ~CHNL_CTRL_SW_RST;
	writel(val, mxc_isi->regs + CHNL_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_sw_reset);

void mxc_isi_channel_source_config(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_CTRL);
	val &= ~(CHNL_CTRL_MIPI_VC_ID_MASK |
		 CHNL_CTRL_SRC_INPUT_MASK | CHNL_CTRL_SRC_TYPE_MASK);

	switch (mxc_isi->interface[IN_PORT]) {
	case ISI_INPUT_INTERFACE_MIPI0_CSI2:
		val |= mxc_isi->pdata->chan_src->src_mipi0;
		if (mxc_isi->interface[SUB_IN_PORT] <= CHNL_CTRL_MIPI_VC_ID_VC3 &&
		    mxc_isi->interface[SUB_IN_PORT] >= CHNL_CTRL_MIPI_VC_ID_VC0)
			val |= (mxc_isi->interface[SUB_IN_PORT] << CHNL_CTRL_MIPI_VC_ID_OFFSET);
		break;
	case ISI_INPUT_INTERFACE_MIPI1_CSI2:
		val |= mxc_isi->pdata->chan_src->src_mipi1;
		if (mxc_isi->interface[SUB_IN_PORT] <= CHNL_CTRL_MIPI_VC_ID_VC3 &&
		    mxc_isi->interface[SUB_IN_PORT] >= CHNL_CTRL_MIPI_VC_ID_VC0)
			val |= (mxc_isi->interface[SUB_IN_PORT] << CHNL_CTRL_MIPI_VC_ID_OFFSET);
		break;
	case ISI_INPUT_INTERFACE_DC0:
		val |= mxc_isi->pdata->chan_src->src_dc0;
		break;
	case ISI_INPUT_INTERFACE_DC1:
		val |= mxc_isi->pdata->chan_src->src_dc1;
		break;
	case ISI_INPUT_INTERFACE_HDMI:
		val |= mxc_isi->pdata->chan_src->src_hdmi;
		break;
	case ISI_INPUT_INTERFACE_PARALLEL_CSI:
		val |= mxc_isi->pdata->chan_src->src_csi;
		break;
	case ISI_INPUT_INTERFACE_MEM:
		val |= mxc_isi->pdata->chan_src->src_mem;
		val |= (CHNL_CTRL_SRC_TYPE_MEMORY << CHNL_CTRL_SRC_TYPE_OFFSET);
		break;
	default:
		dev_err(&mxc_isi->pdev->dev, "invalid interface\n");
		break;
	}

	writel(val, mxc_isi->regs + CHNL_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_source_config);

void mxc_isi_channel_set_flip_loc(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_VFLIP_EN_MASK | CHNL_IMG_CTRL_HFLIP_EN_MASK);

	if (mxc_isi->vflip)
		val |= (CHNL_IMG_CTRL_VFLIP_EN_ENABLE << CHNL_IMG_CTRL_VFLIP_EN_OFFSET);
	if (mxc_isi->hflip)
		val |= (CHNL_IMG_CTRL_HFLIP_EN_ENABLE << CHNL_IMG_CTRL_HFLIP_EN_OFFSET);

	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_set_chain_buf);

void mxc_isi_channel_set_csc(struct mxc_isi_dev *mxc_isi,
			     struct mxc_isi_frame *src_f,
			     struct mxc_isi_frame *dst_f)
{
	struct mxc_isi_fmt *src_fmt = src_f->fmt;
	struct mxc_isi_fmt *dst_fmt = dst_f->fmt;
	u32 val, csc = 0;

	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_FORMAT_MASK |
		 CHNL_IMG_CTRL_YCBCR_MODE_MASK |
		 CHNL_IMG_CTRL_CSC_BYPASS_MASK |
		 CHNL_IMG_CTRL_CSC_MODE_MASK);

	/* set outbuf format */
	val |= dst_fmt->color << CHNL_IMG_CTRL_FORMAT_OFFSET;

	mxc_isi->cscen = 1;

	if (is_yuv(src_fmt->fourcc) && is_rgb(dst_fmt->fourcc)) {
		/* YUV2RGB */
		csc = YUV2RGB;
		/* YCbCr enable???  */
		val |= (CHNL_IMG_CTRL_CSC_MODE_YCBCR2RGB << CHNL_IMG_CTRL_CSC_MODE_OFFSET);
		val |= (CHNL_IMG_CTRL_YCBCR_MODE_ENABLE << CHNL_IMG_CTRL_YCBCR_MODE_OFFSET);
	} else if (is_rgb(src_fmt->fourcc) && is_yuv(dst_fmt->fourcc)) {
		/* RGB2YUV */
		csc = RGB2YUV;
		val |= (CHNL_IMG_CTRL_CSC_MODE_RGB2YCBCR << CHNL_IMG_CTRL_CSC_MODE_OFFSET);
	} else {
		/* Bypass CSC */
		pr_info("bypass csc\n");
		mxc_isi->cscen = 0;
		val |= CHNL_IMG_CTRL_CSC_BYPASS_ENABLE;
	}

	printk_pixelformat("input fmt", src_fmt->fourcc);
	printk_pixelformat("output fmt", dst_fmt->fourcc);

	if (mxc_isi->cscen) {
		writel(coeffs[csc][0], mxc_isi->regs + CHNL_CSC_COEFF0);
		writel(coeffs[csc][1], mxc_isi->regs + CHNL_CSC_COEFF1);
		writel(coeffs[csc][2], mxc_isi->regs + CHNL_CSC_COEFF2);
		writel(coeffs[csc][3], mxc_isi->regs + CHNL_CSC_COEFF3);
		writel(coeffs[csc][4], mxc_isi->regs + CHNL_CSC_COEFF4);
		writel(coeffs[csc][5], mxc_isi->regs + CHNL_CSC_COEFF5);
	}

	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);
}

void mxc_isi_channel_set_alpha_roi0(struct mxc_isi_dev *mxc_isi,
				    struct v4l2_rect *rect)
{
	u32 val0, val1;

	val0 = (rect->left << 16) | rect->top;
	writel(val0, mxc_isi->regs + CHNL_ROI_0_ULC);
	val1 = (rect->width << 16) | rect->height;
	writel(val0 + val1, mxc_isi->regs + CHNL_ROI_0_LRC);
}

void mxc_isi_channel_set_alpha_loc(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~(CHNL_IMG_CTRL_GBL_ALPHA_VAL_MASK | CHNL_IMG_CTRL_GBL_ALPHA_EN_MASK);

	if (mxc_isi->alphaen)
		val |= ((mxc_isi->alpha << CHNL_IMG_CTRL_GBL_ALPHA_VAL_OFFSET) |
			(CHNL_IMG_CTRL_GBL_ALPHA_EN_ENABLE << CHNL_IMG_CTRL_GBL_ALPHA_EN_OFFSET));

	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);
}

void mxc_isi_channel_set_panic_threshold(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_set_thd *set_thd = mxc_isi->pdata->set_thd;
	u32 val;

	val = readl(mxc_isi->regs + CHNL_OUT_BUF_CTRL);

	val &= ~(set_thd->panic_set_thd_y.mask);
	val |= set_thd->panic_set_thd_y.threshold << set_thd->panic_set_thd_y.offset;

	val &= ~(set_thd->panic_set_thd_u.mask);
	val |= set_thd->panic_set_thd_u.threshold << set_thd->panic_set_thd_u.offset;

	val &= ~(set_thd->panic_set_thd_v.mask);
	val |= set_thd->panic_set_thd_v.threshold << set_thd->panic_set_thd_v.offset;

	writel(val, mxc_isi->regs + CHNL_OUT_BUF_CTRL);
}

void mxc_isi_channel_set_chain_buf(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	if (mxc_isi->chain_buf) {
		val = readl(mxc_isi->regs + CHNL_CTRL);
		val &= ~CHNL_CTRL_CHAIN_BUF_MASK;
		val |= (CHNL_CTRL_CHAIN_BUF_2_CHAIN << CHNL_CTRL_CHAIN_BUF_OFFSET);

		writel(val, mxc_isi->regs + CHNL_CTRL);
	}
}

void mxc_isi_channel_deinterlace_init(struct mxc_isi_dev *mxc_isi)
{
	/* Config for Blending deinterlace */
}

void mxc_isi_channel_set_deinterlace(struct mxc_isi_dev *mxc_isi)
{
	/* de-interlacing method
	 * Weaving-------------Yes
	 * Line Doubling-------No
	 * Blending -----------TODO
	 */
	u32 val;

	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_DEINT_MASK;
	if (mxc_isi->deinterlace)
		val |= mxc_isi->deinterlace << CHNL_IMG_CTRL_DEINT_OFFSET;
	if ((mxc_isi->deinterlace == CHNL_IMG_CTRL_DEINT_LDOUBLE_ODD_EVEN) ||
	    (mxc_isi->deinterlace == CHNL_IMG_CTRL_DEINT_LDOUBLE_EVEN_ODD))
		mxc_isi_channel_deinterlace_init(mxc_isi);

	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);
}

void mxc_isi_channel_set_crop(struct mxc_isi_dev *mxc_isi,
			      struct mxc_isi_frame *dst_f)
{
	struct v4l2_rect crop;
	u32 val, val0, val1;

	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_CROP_EN_MASK;

	if ((dst_f->o_height == dst_f->c_height) &&
	    (dst_f->o_width == dst_f->c_width)) {
		mxc_isi->crop = 0;
		writel(val, mxc_isi->regs + CHNL_IMG_CTRL);
		return;
	}

	crop.left = dst_f->h_off;
	crop.top  = dst_f->v_off;
	crop.width  = dst_f->c_width - 1;
	crop.height = dst_f->c_height - 1;

	mxc_isi->crop = 1;
	val |= (CHNL_IMG_CTRL_CROP_EN_ENABLE << CHNL_IMG_CTRL_CROP_EN_OFFSET);
	val0 = crop.top | (crop.left << CHNL_CROP_ULC_X_OFFSET);
	val1 = (crop.top + crop.height) | ((crop.left + crop.width) << CHNL_CROP_LRC_X_OFFSET);

	writel(val0, mxc_isi->regs + CHNL_CROP_ULC);
	writel(val1, mxc_isi->regs + CHNL_CROP_LRC);
	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);
}

static void mxc_isi_channel_clear_scaling(struct mxc_isi_dev *mxc_isi)
{
	u32 val0;

	writel(0x10001000, mxc_isi->regs + CHNL_SCALE_FACTOR);

	val0 = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val0 &= ~(CHNL_IMG_CTRL_DEC_X_MASK | CHNL_IMG_CTRL_DEC_Y_MASK);
	writel(val0, mxc_isi->regs + CHNL_IMG_CTRL);
}

void mxc_isi_channel_set_scaling(struct mxc_isi_dev *mxc_isi,
				 struct mxc_isi_frame *src_f,
				 struct mxc_isi_frame *dst_f)
{
	u32 decx, decy;
	u32 xscale, yscale;
	u32 xdec = 0, ydec = 0;
	u32 val0, val1;

	if (dst_f->height == src_f->height &&
	    dst_f->width == src_f->width) {
		mxc_isi->scale = 0;
		mxc_isi_channel_clear_scaling(mxc_isi);
		dev_dbg(&mxc_isi->pdev->dev, "%s: no scale\n", __func__);
		return;
	}

	dev_info(&mxc_isi->pdev->dev, "input_size(%d,%d), output_size(%d,%d)\n",
		 src_f->width, src_f->height, dst_f->width, dst_f->height);

	mxc_isi->scale = 1;

	decx = src_f->width / dst_f->width;
	decy = src_f->height / dst_f->height;

	if (decx > 1) {
		/* Down */
		if (decx >= 2 && decx < 4) {
			decx = 2;
			xdec = 1;
		} else if (decx >= 4 && decx < 8) {
			decx = 4;
			xdec = 2;
		} else if (decx >= 8) {
			decx = 8;
			xdec = 3;
		}
		xscale = src_f->width * 0x1000 / (dst_f->width * decx);
	} else {
		/* Up  */
		xscale = src_f->width * 0x1000 / dst_f->width;
	}

	if (decy > 1) {
		if (decy >= 2 && decy < 4) {
			decy = 2;
			ydec = 1;
		} else if (decy >= 4 && decy < 8) {
			decy = 4;
			ydec = 2;
		} else if (decy >= 8) {
			decy = 8;
			ydec = 3;
		}
		yscale = src_f->height * 0x1000 / (dst_f->height * decy);
	} else {
		yscale = src_f->height * 0x1000 / dst_f->height;
	}

	val0 = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val0 |= CHNL_IMG_CTRL_YCBCR_MODE_MASK;//YCbCr  Sandor???
	val0 &= ~(CHNL_IMG_CTRL_DEC_X_MASK | CHNL_IMG_CTRL_DEC_Y_MASK);
	val0 |= (xdec << CHNL_IMG_CTRL_DEC_X_OFFSET) |
			(ydec << CHNL_IMG_CTRL_DEC_Y_OFFSET);
	writel(val0, mxc_isi->regs + CHNL_IMG_CTRL);

	if (xscale > ISI_DOWNSCALE_THRESHOLD)
		xscale = ISI_DOWNSCALE_THRESHOLD;
	if (yscale > ISI_DOWNSCALE_THRESHOLD)
		yscale = ISI_DOWNSCALE_THRESHOLD;

	val1 = xscale | (yscale << CHNL_SCALE_FACTOR_Y_SCALE_OFFSET);

	writel(val1, mxc_isi->regs + CHNL_SCALE_FACTOR);

	/* Update scale config if scaling enabled */
	val1 = dst_f->o_width | (dst_f->o_height << CHNL_SCL_IMG_CFG_HEIGHT_OFFSET);
	writel(val1, mxc_isi->regs + CHNL_SCL_IMG_CFG);

	writel(0, mxc_isi->regs + CHNL_SCALE_OFFSET);

	return;
}

void mxc_isi_channel_init(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	/* sw reset */
	mxc_isi_channel_sw_reset(mxc_isi);

	/* Init channel clk first */
	val = readl(mxc_isi->regs + CHNL_CTRL);
	val |= (CHNL_CTRL_CLK_EN_ENABLE << CHNL_CTRL_CLK_EN_OFFSET);
	writel(val, mxc_isi->regs + CHNL_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_init);

void mxc_isi_channel_deinit(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	/* sw reset */
	mxc_isi_channel_sw_reset(mxc_isi);

	/* deinit channel clk first */
	val = (CHNL_CTRL_CLK_EN_DISABLE << CHNL_CTRL_CLK_EN_OFFSET);
	writel(val, mxc_isi->regs + CHNL_CTRL);

	if (mxc_isi->chain_buf && mxc_isi->chain)
		regmap_write(mxc_isi->chain, CHNL_CTRL, 0x0);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_deinit);

void mxc_isi_channel_config_loc(struct mxc_isi_dev *mxc_isi,
				struct mxc_isi_frame *src_f,
				struct mxc_isi_frame *dst_f)
{
	u32 val;

	/* images having higher than 2048 horizontal resolution */
	chain_buf(mxc_isi, src_f);

	/* config output frame size and format */
	val = src_f->o_width | (src_f->o_height << CHNL_IMG_CFG_HEIGHT_OFFSET);
	writel(val, mxc_isi->regs + CHNL_IMG_CFG);

	/* scale size need to equal input size when scaling disabled*/
	writel(val, mxc_isi->regs + CHNL_SCL_IMG_CFG);

	/* check csc and scaling  */
	mxc_isi_channel_set_csc(mxc_isi, src_f, dst_f);

	mxc_isi_channel_set_scaling(mxc_isi, src_f, dst_f);

	/* set cropping */
	mxc_isi_channel_set_crop(mxc_isi, dst_f);

	/* select the source input / src type / virtual channel for mipi*/
	mxc_isi_channel_source_config(mxc_isi);

	/* line pitch */
	val = dst_f->bytesperline[0];
	writel(val, mxc_isi->regs + CHNL_OUT_BUF_PITCH);

	/* TODO */
	mxc_isi_channel_set_flip_loc(mxc_isi);

	mxc_isi_channel_set_alpha_loc(mxc_isi);

	mxc_isi_channel_set_panic_threshold(mxc_isi);

	val = readl(mxc_isi->regs + CHNL_CTRL);
	val &= ~CHNL_CTRL_CHNL_BYPASS_MASK;

	/*  Bypass channel */
	if (!mxc_isi->cscen && !mxc_isi->scale)
		val |= (CHNL_CTRL_CHNL_BYPASS_ENABLE << CHNL_CTRL_CHNL_BYPASS_OFFSET);

	writel(val, mxc_isi->regs + CHNL_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_config_loc);

void mxc_isi_clean_registers(struct mxc_isi_dev *mxc_isi)
{
	u32 status;

	status = mxc_isi_get_irq_status(mxc_isi);
	mxc_isi_clean_irq_status(mxc_isi, status);
}
EXPORT_SYMBOL_GPL(mxc_isi_clean_registers);

void mxc_isi_channel_enable_loc(struct mxc_isi_dev *mxc_isi, bool m2m_enabled)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_CTRL);
	val |= 0xff << CHNL_CTRL_BLANK_PXL_OFFSET;

	if (m2m_enabled) {
		val &= ~(CHNL_CTRL_SRC_TYPE_MASK | CHNL_CTRL_SRC_INPUT_MASK);
		val |= (mxc_isi->pdata->chan_src->src_mem << CHNL_CTRL_SRC_INPUT_OFFSET |
			CHNL_CTRL_SRC_TYPE_MEMORY << CHNL_CTRL_SRC_TYPE_OFFSET);
	}

	val &= ~CHNL_CTRL_CHNL_EN_MASK;
	val |= CHNL_CTRL_CHNL_EN_ENABLE << CHNL_CTRL_CHNL_EN_OFFSET;
	writel(val, mxc_isi->regs + CHNL_CTRL);

	mxc_isi_clean_registers(mxc_isi);
	mxc_isi_enable_irq(mxc_isi);

	if (m2m_enabled) {
		mxc_isi_m2m_start_read(mxc_isi);
		return;
	}

	dump_isi_regs(mxc_isi);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_enable_loc);

void mxc_isi_channel_disable_loc(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	mxc_isi_disable_irq(mxc_isi);

	val = readl(mxc_isi->regs + CHNL_CTRL);
	val &= ~(CHNL_CTRL_CHNL_EN_MASK | CHNL_CTRL_CLK_EN_MASK);
	val |= (CHNL_CTRL_CHNL_EN_DISABLE << CHNL_CTRL_CHNL_EN_OFFSET);
	val |= (CHNL_CTRL_CLK_EN_DISABLE << CHNL_CTRL_CLK_EN_OFFSET);
	writel(val, mxc_isi->regs + CHNL_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_channel_disable_loc);

void  mxc_isi_enable_irq(struct mxc_isi_dev *mxc_isi)
{
	struct mxc_isi_ier_reg *ier_reg = mxc_isi->pdata->ier_reg;
	u32 val;

	val = CHNL_IER_FRM_RCVD_EN_MASK |
		CHNL_IER_AXI_WR_ERR_U_EN_MASK |
		CHNL_IER_AXI_WR_ERR_V_EN_MASK |
		CHNL_IER_AXI_WR_ERR_Y_EN_MASK;

	/* Y/U/V overflow enable */
	val |= ier_reg->oflw_y_buf_en.mask |
	       ier_reg->oflw_u_buf_en.mask |
	       ier_reg->oflw_v_buf_en.mask;

	/* Y/U/V excess overflow enable */
	val |= ier_reg->excs_oflw_y_buf_en.mask |
	       ier_reg->excs_oflw_u_buf_en.mask |
	       ier_reg->excs_oflw_v_buf_en.mask;

	/* Y/U/V panic enable */
	val |= ier_reg->panic_y_buf_en.mask |
	       ier_reg->panic_u_buf_en.mask |
	       ier_reg->panic_v_buf_en.mask;

	writel(val, mxc_isi->regs + CHNL_IER);
}
EXPORT_SYMBOL_GPL(mxc_isi_enable_irq);

void mxc_isi_disable_irq(struct mxc_isi_dev *mxc_isi)
{
	writel(0, mxc_isi->regs + CHNL_IER);
}
EXPORT_SYMBOL_GPL(mxc_isi_disable_irq);

u32 mxc_isi_get_irq_status(struct mxc_isi_dev *mxc_isi)
{
	return readl(mxc_isi->regs + CHNL_STS);
}
EXPORT_SYMBOL_GPL(mxc_isi_get_irq_status);

void mxc_isi_clean_irq_status(struct mxc_isi_dev *mxc_isi, u32 val)
{
	writel(val, mxc_isi->regs + CHNL_STS);
}
EXPORT_SYMBOL_GPL(mxc_isi_clean_irq_status);

void mxc_isi_m2m_config_src(struct mxc_isi_dev *mxc_isi,
			    struct mxc_isi_frame *src_f)
{
	u32 val;

	/* source format */
	val = readl(mxc_isi->regs + CHNL_MEM_RD_CTRL);
	val &= ~CHNL_MEM_RD_CTRL_IMG_TYPE_MASK;
	val |= src_f->fmt->color << CHNL_MEM_RD_CTRL_IMG_TYPE_OFFSET;
	writel(val, mxc_isi->regs + CHNL_MEM_RD_CTRL);

	/* source image width and height */
	val = (src_f->width << CHNL_IMG_CFG_WIDTH_OFFSET |
	       src_f->height << CHNL_IMG_CFG_HEIGHT_OFFSET);
	writel(val, mxc_isi->regs + CHNL_IMG_CFG);

	/* source pitch */
	val = src_f->bytesperline[0] << CHNL_IN_BUF_PITCH_LINE_PITCH_OFFSET;
	writel(val, mxc_isi->regs + CHNL_IN_BUF_PITCH);
}
EXPORT_SYMBOL_GPL(mxc_isi_m2m_config_src);

void mxc_isi_m2m_config_dst(struct mxc_isi_dev *mxc_isi,
			    struct mxc_isi_frame *dst_f)
{
	u32 val;

	/* out format */
	val = readl(mxc_isi->regs + CHNL_IMG_CTRL);
	val &= ~CHNL_IMG_CTRL_FORMAT_MASK;
	val |= dst_f->fmt->color << CHNL_IMG_CTRL_FORMAT_OFFSET;
	writel(val, mxc_isi->regs + CHNL_IMG_CTRL);

	/* out pitch */
	val = readl(mxc_isi->regs + CHNL_OUT_BUF_PITCH);
	val &= ~CHNL_IN_BUF_PITCH_LINE_PITCH_MASK;
	val |= dst_f->bytesperline[0] << CHNL_OUT_BUF_PITCH_LINE_PITCH_OFFSET;
	writel(val, mxc_isi->regs + CHNL_OUT_BUF_PITCH);
}
EXPORT_SYMBOL_GPL(mxc_isi_m2m_config_dst);

void mxc_isi_m2m_start_read(struct mxc_isi_dev *mxc_isi)
{
	u32 val;

	val = readl(mxc_isi->regs + CHNL_MEM_RD_CTRL);
	val &= ~ CHNL_MEM_RD_CTRL_READ_MEM_MASK;
	writel(val, mxc_isi->regs + CHNL_MEM_RD_CTRL);
	udelay(300);

	val |= CHNL_MEM_RD_CTRL_READ_MEM_ENABLE << CHNL_MEM_RD_CTRL_READ_MEM_OFFSET;
	writel(val, mxc_isi->regs + CHNL_MEM_RD_CTRL);
}
EXPORT_SYMBOL_GPL(mxc_isi_m2m_start_read);
