/*
 * Copyright 2017-2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */
#include <drm/drm_fourcc.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <soc/imx8/sc/sci.h>
#include <video/imx8-prefetch.h>

#define SET					0x4
#define CLR					0x8
#define TOG					0xc

#define SYSTEM_CTRL0				0x00
#define BCMD2AXI_MASTR_ID_CTRL			BIT(16)
#define SW_SHADOW_LOAD_SEL			BIT(4)
#define SHADOW_LOAD_EN				BIT(3)
#define REPEAT_EN				BIT(2)
#define SOFT_RESET				BIT(1)
#define RUN_EN					BIT(0)	/* self-clearing */

#define IRQ_MASK				0x20
#define IRQ_MASK_STATUS				0x30
#define IRQ_NONMASK_STATUS			0x40
#define DPR2RTR_FIFO_LOAD_BUF_RDY_UV_ERROR	BIT(7)
#define DPR2RTR_FIFO_LOAD_BUF_RDY_YRGB_ERROR	BIT(6)
#define DPR2RTR_UV_FIFO_OVFL			BIT(5)
#define DPR2RTR_YRGB_FIFO_OVFL			BIT(4)
#define IRQ_AXI_READ_ERROR			BIT(3)
#define IRQ_DPR_SHADOW_LOADED_MASK		BIT(2)
#define IRQ_DPR_RUN				BIT(1)
#define IRQ_DPR_CRTL_DONE			BIT(0)
#define IRQ_ERROR_MASK				0xf8
#define IRQ_CTRL_MASK				0x7

#define MODE_CTRL0				0x50
#define PIX_COMP_SEL_MASK			0x3fc00
#define A_COMP_SEL(byte)			(((byte) & 0x3) << 16)
#define R_COMP_SEL(byte)			(((byte) & 0x3) << 14)
#define G_COMP_SEL(byte)			(((byte) & 0x3) << 12)
#define B_COMP_SEL(byte)			(((byte) & 0x3) << 10)
#define PIX_UV_SWAP				BIT(9)
#define VU					BIT(9)
#define UV					0
#define PIXEL_LUMA_UV_SWAP			BIT(8)
#define UYVY					BIT(8)
#define YUYV					0
#define PIX_SIZE				0xc0
enum {
	PIX_SIZE_8BIT = (0 << 6),
	PIX_SIZE_16BIT = (1 << 6),
	PIX_SIZE_32BIT = (2 << 6),
	PIX_SIZE_RESERVED = (3 << 6),
};
#define COMP_2PLANE_EN				BIT(5)
#define YUV_EN					BIT(4)
#define TILE_TYPE				0xc
enum {
	LINEAR_TILE = (0 << 2),
	GPU_STANDARD_TILE = (1 << 2),
	GPU_SUPER_TILE = (2 << 2),
	VPU_TILE = (3 << 2),
};
#define RTR_4LINE_BUF_EN			BIT(1)
#define LINE4					BIT(1)
#define LINE8					0
#define RTR_3BUF_EN				BIT(0)
#define BUF3					BIT(0)
#define BUF2					0

#define FRAME_CTRL0				0x70
#define PITCH(n)				(((n) & 0xffff) << 16)
#define ROT_FLIP_ORDER_EN			BIT(4)
#define ROT_FIRST				BIT(4)
#define FLIP_FIRST				0
#define ROT_ENC					0xc
#define DEGREE(n)				((((n) / 90) & 0x3) << 2)
#define VFLIP_EN				BIT(1)
#define HFLIP_EN				BIT(0)

#define FRAME_1P_CTRL0				0x90
#define FRAME_2P_CTRL0				0xe0
#define MAX_BYTES_PREQ				0x7
enum {
	BYTE_64 = 0x0,
	BYTE_128 = 0x1,
	BYTE_256 = 0x2,
	BYTE_512 = 0x3,
	BYTE_1K = 0x4,
	BYTE_2K = 0x5,
	BYTE_4K = 0x6,
};

#define FRAME_1P_PIX_X_CTRL			0xa0
#define FRAME_2P_PIX_X_CTRL			0xf0
#define NUM_X_PIX_WIDE(n)			((n) & 0xffff)
#define FRAME_PIX_X_ULC_CTRL			0xf0
#define CROP_ULC_X(n)				((n) & 0xffff)

#define FRAME_1P_PIX_Y_CTRL			0xb0
#define FRAME_2P_PIX_Y_CTRL			0x100
#define NUM_Y_PIX_HIGH(n)			((n) & 0xffff)
#define FRAME_PIX_Y_ULC_CTRL			0x100
#define CROP_ULC_Y(n)				((n) & 0xffff)

#define FRAME_1P_BASE_ADDR_CTRL0		0xc0
#define FRAME_2P_BASE_ADDR_CTRL0		0x110

#define STATUS_CTRL0				0x130
#define STATUS_SRC_SEL				0x70000
enum {
	DPR_CTRL = 0x0,
	PREFETCH_1PLANE = 0x1,
	RESPONSE_1PLANE = 0x2,
	PREFETCH_2PLANE = 0x3,
	RESPONSE_2PLANE = 0x4,
};
#define STATUS_MUX_SEL				0x7

#define STATUS_CTRL1				0x140

#define RTRAM_CTRL0				0x200
#define ABORT_SEL				BIT(7)
#define ABORT					BIT(7)
#define STALL					0
#define THRES_LOW_MASK				0x70
#define THRES_LOW(n)				(((n) & 0x7) << 4)
#define THRES_HIGH_MASK				0xe
#define THRES_HIGH(n)				(((n) & 0x7) << 1)
#define NUM_ROWS_ACTIVE				BIT(0)
#define ROWS_0_6				BIT(0)
#define ROWS_0_4				0

struct dprc_devtype {
	bool has_fixup;
};

static const struct dprc_devtype dprc_type_v1 = {
	.has_fixup = false,
};

static const struct dprc_devtype dprc_type_v2 = {
	.has_fixup = true,
};

struct dprc {
	struct device *dev;
	const struct dprc_devtype *devtype;
	void __iomem *base;
	struct list_head list;
	struct clk *clk_apb;
	struct clk *clk_b;
	struct clk *clk_rtram;
	spinlock_t spin_lock;
	u32 sc_resource;
	bool is_blit_chan;

	/* The second one, if non-NULL, is auxiliary for UV buffer. */
	struct prg *prgs[2];
	bool has_aux_prg;
	bool use_aux_prg;
};

struct dprc_format_info {
	u32 format;
	u8 depth;
	u8 num_planes;
	u8 cpp[3];
	u8 hsub;
	u8 vsub;
};

static const struct dprc_format_info formats[] = {
	{
	  .format = DRM_FORMAT_RGB565,
	  .depth = 16, .num_planes = 1, .cpp = { 2, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_ARGB8888,
	  .depth = 32, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_XRGB8888,
	  .depth = 24, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_ABGR8888,
	  .depth = 32, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_XBGR8888,
	  .depth = 24, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_RGBA8888,
	  .depth = 32, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_RGBX8888,
	  .depth = 24, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_BGRA8888,
	  .depth = 32, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_BGRX8888,
	  .depth = 24, .num_planes = 1, .cpp = { 4, 0, 0 },
	  .hsub = 1,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_NV12,
	  .depth = 0,  .num_planes = 2, .cpp = { 1, 2, 0 },
	  .hsub = 2,   .vsub = 2,
	}, {
	  .format = DRM_FORMAT_NV21,
	  .depth = 0,  .num_planes = 2, .cpp = { 1, 2, 0 },
	  .hsub = 2,   .vsub = 2,
	}, {
	  .format = DRM_FORMAT_YUYV,
	  .depth = 0,  .num_planes = 1, .cpp = { 2, 0, 0 },
	  .hsub = 2,   .vsub = 1,
	}, {
	  .format = DRM_FORMAT_UYVY,
	  .depth = 0,  .num_planes = 1, .cpp = { 2, 0, 0 },
	  .hsub = 2,   .vsub = 1,
	}
};

static const struct dprc_format_info *dprc_format_info(u32 format)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(formats); ++i) {
		if (formats[i].format == format)
			return &formats[i];
	}

	return NULL;
}

static DEFINE_MUTEX(dprc_list_mutex);
static LIST_HEAD(dprc_list);

static inline u32 dprc_read(struct dprc *dprc, unsigned int offset)
{
	return readl(dprc->base + offset);
}

static inline void dprc_write(struct dprc *dprc, u32 value, unsigned int offset)
{
	writel(value, dprc->base + offset);
}

static void dprc_reset(struct dprc *dprc)
{
	dprc_write(dprc, SOFT_RESET, SYSTEM_CTRL0 + SET);
	usleep_range(1000, 2000);
	dprc_write(dprc, SOFT_RESET, SYSTEM_CTRL0 + CLR);
}

void dprc_enable(struct dprc *dprc)
{
	if (WARN_ON(!dprc))
		return;

	prg_enable(dprc->prgs[0]);
	if (dprc->use_aux_prg)
		prg_enable(dprc->prgs[1]);
}
EXPORT_SYMBOL_GPL(dprc_enable);

void dprc_disable(struct dprc *dprc)
{
	if (WARN_ON(!dprc))
		return;

	dprc_write(dprc, SHADOW_LOAD_EN | SW_SHADOW_LOAD_SEL, SYSTEM_CTRL0);

	prg_disable(dprc->prgs[0]);
	if (dprc->has_aux_prg)
		prg_disable(dprc->prgs[1]);

	prg_reg_update(dprc->prgs[0]);
	if (dprc->has_aux_prg)
		prg_reg_update(dprc->prgs[1]);
}
EXPORT_SYMBOL_GPL(dprc_disable);

static void dprc_dpu_gpr_configure(struct dprc *dprc, unsigned int stream_id)
{
	sc_err_t sciErr;
	sc_ipc_t ipcHndl = 0;
	u32 mu_id;

	if (WARN_ON(!dprc))
		return;

	sciErr = sc_ipc_getMuID(&mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(dprc->dev, "cannot obtain MU ID %d\n", sciErr);
		return;
	}

	sciErr = sc_ipc_open(&ipcHndl, mu_id);
	if (sciErr != SC_ERR_NONE) {
		dev_err(dprc->dev, "sc_ipc_open failed %d\n", sciErr);
		return;
	}

	sciErr = sc_misc_set_control(ipcHndl, dprc->sc_resource,
					SC_C_KACHUNK_SEL, stream_id);
	if (sciErr != SC_ERR_NONE)
		dev_err(dprc->dev, "sc_misc_set_control failed %d\n", sciErr);

	sc_ipc_close(mu_id);
}

void dprc_configure(struct dprc *dprc, unsigned int stream_id,
		    unsigned int width, unsigned int height,
		    unsigned int x_offset, unsigned int y_offset,
		    unsigned int stride, u32 format, u64 modifier,
		    unsigned long baddr, unsigned long uv_baddr,
		    bool start, bool aux_start, bool interlace_frame)
{
	const struct dprc_format_info *info = dprc_format_info(format);
	unsigned int dprc_width = width + x_offset;
	unsigned int dprc_height;
	unsigned int p1_w, p1_h, p2_w, p2_h;
	unsigned int prg_stride = width * info->cpp[0];
	unsigned int bpp = 8 * info->cpp[0];
	unsigned int preq;
	unsigned int mt_w = 0, mt_h = 0;	/* w/h in a micro-tile */
	u32 val;

	if (WARN_ON(!dprc))
		return;

	dprc->use_aux_prg = false;

	if (start) {
		dprc_reset(dprc);

		if (!dprc->is_blit_chan)
			dprc_dpu_gpr_configure(dprc, stream_id);
	}

	if (interlace_frame) {
		height /= 2;
		y_offset /= 2;
	}

	dprc_height = height + y_offset;

	/* disable all control irqs and enable all error irqs */
	dprc_write(dprc, IRQ_CTRL_MASK, IRQ_MASK);

	if (info->num_planes > 1) {
		p1_w = round_up(dprc_width, modifier ? 8 : 64);
		p1_h = round_up(dprc_height, 8);

		p2_w = p1_w;
		if (modifier)
			p2_h = dprc_height / info->vsub;
		else
			p2_h = round_up((dprc_height / info->vsub), 8);

		preq = modifier ? BYTE_64 : BYTE_1K;

		dprc_write(dprc, preq, FRAME_2P_CTRL0);
		if (!dprc->devtype->has_fixup) {
			dprc_write(dprc,
				NUM_X_PIX_WIDE(p2_w), FRAME_2P_PIX_X_CTRL);
			dprc_write(dprc,
				NUM_Y_PIX_HIGH(p2_h), FRAME_2P_PIX_Y_CTRL);
		}
		dprc_write(dprc, uv_baddr, FRAME_2P_BASE_ADDR_CTRL0);
	} else {
		switch (modifier) {
		case DRM_FORMAT_MOD_VIVANTE_TILED:
			p1_w = round_up(dprc_width, info->cpp[0] == 2 ? 8 : 4);
			break;
		case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
			p1_w = round_up(dprc_width, 64);
			break;
		default:
			p1_w = round_up(dprc_width,
					info->cpp[0] == 2 ? 32 : 16);
			break;
		}
		p1_h = round_up(dprc_height, 4);
	}

	dprc_write(dprc, PITCH(stride), FRAME_CTRL0);
	switch (modifier) {
	case DRM_FORMAT_MOD_AMPHION_TILED:
		preq = BYTE_64;
		mt_w = 8;
		mt_h = 8;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		if (bpp == 16) {
			preq = BYTE_64;
			mt_w = 8;
		} else {
			if (dprc->devtype->has_fixup)
				preq = (x_offset % 8) ? BYTE_64 : BYTE_128;
			else
				preq = BYTE_128;
			mt_w = 4;
		}
		mt_h = 4;
		break;
	default:
		preq = BYTE_1K;
		break;
	}
	dprc_write(dprc, preq, FRAME_1P_CTRL0);
	dprc_write(dprc, NUM_X_PIX_WIDE(p1_w), FRAME_1P_PIX_X_CTRL);
	dprc_write(dprc, NUM_Y_PIX_HIGH(p1_h), FRAME_1P_PIX_Y_CTRL);
	dprc_write(dprc, baddr, FRAME_1P_BASE_ADDR_CTRL0);
	if (dprc->devtype->has_fixup && modifier) {
		dprc_write(dprc, CROP_ULC_X(round_down(x_offset, mt_w)),
							FRAME_PIX_X_ULC_CTRL);
		dprc_write(dprc, CROP_ULC_Y(round_down(y_offset, mt_h)),
							FRAME_PIX_Y_ULC_CTRL);
	}

	val = dprc_read(dprc, RTRAM_CTRL0);
	val &= ~THRES_LOW_MASK;
	val |= THRES_LOW(3);
	val &= ~THRES_HIGH_MASK;
	val |= THRES_HIGH(7);
	dprc_write(dprc, val, RTRAM_CTRL0);

	val = dprc_read(dprc, MODE_CTRL0);
	val &= ~PIX_UV_SWAP;
	val &= ~PIXEL_LUMA_UV_SWAP;
	val &= ~COMP_2PLANE_EN;
	val &= ~YUV_EN;
	val &= ~TILE_TYPE;
	switch (modifier) {
	case DRM_FORMAT_MOD_NONE:
		break;
	case DRM_FORMAT_MOD_AMPHION_TILED:
		val |= VPU_TILE;
		break;
	case DRM_FORMAT_MOD_VIVANTE_TILED:
		val |= GPU_STANDARD_TILE;
		break;
	case DRM_FORMAT_MOD_VIVANTE_SUPER_TILED:
		val |= GPU_SUPER_TILE;
		break;
	default:
		dev_err(dprc->dev, "unsupported modifier 0x%016llx\n",
								modifier);
		return;
	}
	val &= ~RTR_4LINE_BUF_EN;
	val |= info->num_planes > 1 ? LINE8 : LINE4;
	val &= ~RTR_3BUF_EN;
	val |= BUF2;
	val &= ~(PIX_COMP_SEL_MASK | PIX_SIZE);
	switch (format) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
		/*
		 * It turns out pixel components are mapped directly
		 * without position change via DPR processing with
		 * the following color component configurations.
		 * Leave the pixel format to be handled by the
		 * display controllers.
		 */
		val |= A_COMP_SEL(3) | R_COMP_SEL(2) |
		       G_COMP_SEL(1) | B_COMP_SEL(0);
		val |= PIX_SIZE_32BIT;
		break;
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		val |= YUV_EN;
		/* fall-through */
	case DRM_FORMAT_RGB565:
		val |= PIX_SIZE_16BIT;
		break;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		dprc->use_aux_prg = true;

		val |= COMP_2PLANE_EN;
		val |= YUV_EN;
		val |= PIX_SIZE_8BIT;
		break;
	default:
		dev_err(dprc->dev, "unsupported format 0x%08x\n", format);
		return;
	}
	dprc_write(dprc, val, MODE_CTRL0);

	if (start) {
		/* software shadow load for the first frame */
		val = SW_SHADOW_LOAD_SEL;
		if (dprc->is_blit_chan) {
			val |= RUN_EN | REPEAT_EN | SHADOW_LOAD_EN;
			dprc_write(dprc, val, SYSTEM_CTRL0);
		} else {
			val |= SHADOW_LOAD_EN;
			dprc_write(dprc, val, SYSTEM_CTRL0);

			/* and then, run... */
			val |= RUN_EN | REPEAT_EN;
			dprc_write(dprc, val, SYSTEM_CTRL0);
		}
	}

	prg_configure(dprc->prgs[0], width, height, x_offset, y_offset,
			prg_stride, bpp, baddr, format, modifier, start);
	if (dprc->use_aux_prg)
		prg_configure(dprc->prgs[1], width, height, x_offset, y_offset,
			prg_stride, 8, uv_baddr, format, modifier, aux_start);

	dev_dbg(dprc->dev, "w-%u, h-%u, s-%u, fmt-0x%08x, mod-0x%016llx\n",
				width, height, stride, format, modifier);
}
EXPORT_SYMBOL_GPL(dprc_configure);

void dprc_reg_update(struct dprc *dprc)
{
	if (WARN_ON(!dprc))
		return;

	prg_reg_update(dprc->prgs[0]);
	if (dprc->use_aux_prg)
		prg_reg_update(dprc->prgs[1]);
}
EXPORT_SYMBOL_GPL(dprc_reg_update);

void dprc_first_frame_handle(struct dprc *dprc)
{
	if (WARN_ON(!dprc))
		return;

	if (dprc->is_blit_chan)
		dprc_write(dprc, SW_SHADOW_LOAD_SEL, SYSTEM_CTRL0 + CLR);
	else
		dprc_write(dprc, REPEAT_EN, SYSTEM_CTRL0);

	prg_shadow_enable(dprc->prgs[0]);
	if (dprc->use_aux_prg)
		prg_shadow_enable(dprc->prgs[1]);
}
EXPORT_SYMBOL_GPL(dprc_first_frame_handle);

void dprc_irq_handle(struct dprc *dprc)
{
	u32 mask, status;

	if (WARN_ON(!dprc))
		return;

	spin_lock(&dprc->spin_lock);

	mask = dprc_read(dprc, IRQ_MASK);
	mask = ~mask;
	status = dprc_read(dprc, IRQ_MASK_STATUS);
	status &= mask;

	/* disable irqs to be handled */
	dprc_write(dprc, status, IRQ_MASK + SET);

	/* clear status */
	dprc_write(dprc, status, IRQ_MASK_STATUS);

	if (status & DPR2RTR_FIFO_LOAD_BUF_RDY_UV_ERROR)
		dev_err(dprc->dev,
			"DPR to RTRAM FIFO load UV buffer ready error\n");

	if (status & DPR2RTR_FIFO_LOAD_BUF_RDY_YRGB_ERROR)
		dev_err(dprc->dev,
			"DPR to RTRAM FIFO load YRGB buffer ready error\n");

	if (status & DPR2RTR_UV_FIFO_OVFL)
		dev_err(dprc->dev, "DPR to RTRAM FIFO UV FIFO overflow\n");

	if (status & DPR2RTR_YRGB_FIFO_OVFL)
		dev_err(dprc->dev, "DPR to RTRAM FIFO YRGB FIFO overflow\n");

	if (status & IRQ_AXI_READ_ERROR)
		dev_err(dprc->dev, "AXI read error\n");

	if (status & IRQ_DPR_CRTL_DONE)
		dprc_first_frame_handle(dprc);

	spin_unlock(&dprc->spin_lock);
}
EXPORT_SYMBOL_GPL(dprc_irq_handle);

void dprc_enable_ctrl_done_irq(struct dprc *dprc)
{
	unsigned long lock_flags;

	if (WARN_ON(!dprc))
		return;

	spin_lock_irqsave(&dprc->spin_lock, lock_flags);
	dprc_write(dprc, IRQ_DPR_CRTL_DONE, IRQ_MASK + CLR);
	spin_unlock_irqrestore(&dprc->spin_lock, lock_flags);
}
EXPORT_SYMBOL_GPL(dprc_enable_ctrl_done_irq);

bool dprc_format_supported(struct dprc *dprc, u32 format, u64 modifier)
{
	if (WARN_ON(!dprc))
		return false;

	switch (format) {
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
	case DRM_FORMAT_BGRA8888:
	case DRM_FORMAT_BGRX8888:
	case DRM_FORMAT_RGB565:
		return (modifier == DRM_FORMAT_MOD_NONE ||
			modifier == DRM_FORMAT_MOD_VIVANTE_TILED ||
			modifier == DRM_FORMAT_MOD_VIVANTE_SUPER_TILED);
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
		switch (dprc->sc_resource) {
		case SC_R_DC_0_FRAC0:
		case SC_R_DC_1_FRAC0:
		case SC_R_DC_0_WARP:
		case SC_R_DC_1_WARP:
			return false;
		}
		return modifier == DRM_FORMAT_MOD_NONE;
	case DRM_FORMAT_NV12:
	case DRM_FORMAT_NV21:
		switch (dprc->sc_resource) {
		case SC_R_DC_0_FRAC0:
		case SC_R_DC_1_FRAC0:
		case SC_R_DC_0_WARP:
		case SC_R_DC_1_WARP:
			return false;
		}
		return (dprc->has_aux_prg &&
			(modifier == DRM_FORMAT_MOD_NONE ||
			 modifier == DRM_FORMAT_MOD_AMPHION_TILED));
	}

	return false;
}
EXPORT_SYMBOL_GPL(dprc_format_supported);

bool dprc_stride_supported(struct dprc *dprc,
			   unsigned int stride, unsigned int uv_stride,
			   unsigned int width, u32 format)
{
	const struct dprc_format_info *info = dprc_format_info(format);
	unsigned int prg_stride = width * info->cpp[0];

	if (WARN_ON(!dprc))
		return false;

	if (stride > 0xffff)
		return false;

	if (info->num_planes > 1 && stride != uv_stride)
		return false;

	return prg_stride_supported(dprc->prgs[0], prg_stride);
}
EXPORT_SYMBOL_GPL(dprc_stride_supported);

bool dprc_stride_double_check(struct dprc *dprc,
			      unsigned int stride, unsigned int uv_stride,
			      unsigned int width, u32 format,
			      dma_addr_t baddr, dma_addr_t uv_baddr)
{
	const struct dprc_format_info *info = dprc_format_info(format);
	unsigned int prg_stride = width * info->cpp[0];

	if (WARN_ON(!dprc))
		return false;

	if (!prg_stride_double_check(dprc->prgs[0], prg_stride, baddr))
		return false;

	if (info->num_planes > 1 &&
	    !prg_stride_double_check(dprc->prgs[1], prg_stride, uv_baddr))
		return false;

	return true;
}
EXPORT_SYMBOL_GPL(dprc_stride_double_check);

struct dprc *
dprc_lookup_by_phandle(struct device *dev, const char *name, int index)
{
	struct device_node *dprc_node = of_parse_phandle(dev->of_node,
							 name, index);
	struct dprc *dprc;

	mutex_lock(&dprc_list_mutex);
	list_for_each_entry(dprc, &dprc_list, list) {
		if (dprc_node == dprc->dev->of_node) {
			mutex_unlock(&dprc_list_mutex);
			device_link_add(dev, dprc->dev, DL_FLAG_AUTOREMOVE);
			return dprc;
		}
	}
	mutex_unlock(&dprc_list_mutex);

	return NULL;
}
EXPORT_SYMBOL_GPL(dprc_lookup_by_phandle);

static const struct of_device_id dprc_dt_ids[] = {
	{ .compatible = "fsl,imx8qm-dpr-channel", .data = &dprc_type_v1, },
	{ .compatible = "fsl,imx8qxp-dpr-channel", .data = &dprc_type_v2, },
	{ /* sentinel */ },
};

static int dprc_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(dprc_dt_ids, &pdev->dev);
	struct device *dev = &pdev->dev;
	struct resource *res;
	struct dprc *dprc;
	int ret, i;

	dprc = devm_kzalloc(dev, sizeof(*dprc), GFP_KERNEL);
	if (!dprc)
		return -ENOMEM;

	dprc->devtype = of_id->data;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	dprc->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(dprc->base))
		return PTR_ERR(dprc->base);

	dprc->clk_apb = devm_clk_get(dev, "apb");
	if (IS_ERR(dprc->clk_apb))
		return PTR_ERR(dprc->clk_apb);
	clk_prepare_enable(dprc->clk_apb);

	dprc->clk_b = devm_clk_get(dev, "b");
	if (IS_ERR(dprc->clk_b))
		return PTR_ERR(dprc->clk_b);
	clk_prepare_enable(dprc->clk_b);

	dprc->clk_rtram = devm_clk_get(dev, "rtram");
	if (IS_ERR(dprc->clk_rtram))
		return PTR_ERR(dprc->clk_rtram);
	clk_prepare_enable(dprc->clk_rtram);

	ret = of_property_read_u32(pdev->dev.of_node,
					"fsl,sc-resource", &dprc->sc_resource);
	if (ret) {
		dev_err(dev, "cannot get SC resource %d\n", ret);
		return ret;
	}

	switch (dprc->sc_resource) {
	case SC_R_DC_0_BLIT1:
		if (dprc->devtype->has_fixup)
			dprc->has_aux_prg = true;
		/* fall-through */
	case SC_R_DC_0_BLIT0:
	case SC_R_DC_1_BLIT0:
	case SC_R_DC_1_BLIT1:
		dprc->is_blit_chan = true;
		/* fall-through */
	case SC_R_DC_0_FRAC0:
	case SC_R_DC_1_FRAC0:
		break;
	case SC_R_DC_0_VIDEO0:
	case SC_R_DC_0_VIDEO1:
	case SC_R_DC_1_VIDEO0:
	case SC_R_DC_1_VIDEO1:
	case SC_R_DC_0_WARP:
	case SC_R_DC_1_WARP:
		dprc->has_aux_prg = true;
		break;
	default:
		dev_err(dev, "wrong SC resource %u\n", dprc->sc_resource);
		return -EINVAL;
	}

	for (i = 0; i < 2; i++) {
		if (i == 1 && !dprc->has_aux_prg)
			break;

		dprc->prgs[i] = prg_lookup_by_phandle(dev, "fsl,prgs", i);
		if (!dprc->prgs[i])
			return -EPROBE_DEFER;

		if (i == 1)
			prg_set_auxiliary(dprc->prgs[i]);

		if (dprc->is_blit_chan)
			prg_set_blit(dprc->prgs[i]);
	}

	dprc->dev = dev;
	spin_lock_init(&dprc->spin_lock);
	platform_set_drvdata(pdev, dprc);
	mutex_lock(&dprc_list_mutex);
	list_add(&dprc->list, &dprc_list);
	mutex_unlock(&dprc_list_mutex);

	dprc_reset(dprc);

	return 0;
}

static int dprc_remove(struct platform_device *pdev)
{
	struct dprc *dprc = platform_get_drvdata(pdev);

	mutex_lock(&dprc_list_mutex);
	list_del(&dprc->list);
	mutex_unlock(&dprc_list_mutex);

	clk_disable_unprepare(dprc->clk_rtram);
	clk_disable_unprepare(dprc->clk_b);
	clk_disable_unprepare(dprc->clk_apb);

	return 0;
}

struct platform_driver dprc_drv = {
	.probe = dprc_probe,
	.remove = dprc_remove,
	.driver = {
		.name = "imx8-dpr-channel",
		.of_match_table = dprc_dt_ids,
	},
};
module_platform_driver(dprc_drv);

MODULE_DESCRIPTION("i.MX8 DPRC driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
