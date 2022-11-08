// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2019, 2022 NXP.
 */

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_rect.h>

#include "dcss-dev.h"

#define DTRC_F0_OFS			0x00
#define DTRC_F1_OFS			0x60

#define DCSS_DTRC_DYDSADDR			0x00
#define DCSS_DTRC_DCDSADDR			0x04
#define DCSS_DTRC_DYTSADDR			0x08
#define DCSS_DTRC_DCTSADDR			0x0C
#define DCSS_DTRC_SIZE				0x10
#define   FRAME_WIDTH_POS			0
#define   FRAME_WIDTH_MASK			GENMASK(9, 0)
#define   FRAME_HEIGHT_POS			16
#define   FRAME_HEIGHT_MASK			GENMASK(25, 16)
#define DCSS_DTRC_SYSSA				0x14
#define DCSS_DTRC_SYSEA				0x18
#define DCSS_DTRC_SUVSSA			0x1C
#define DCSS_DTRC_SUVSEA			0x20
#define DCSS_DTRC_CROPORIG			0x24
#define DCSS_DTRC_CROPSIZE			0x28
#define   CROP_HEIGHT_POS			16
#define   CROP_HEIGHT_MASK			GENMASK(28, 16)
#define   CROP_WIDTH_POS			0
#define   CROP_WIDTH_MASK			GENMASK(12, 0)
#define DCSS_DTRC_DCTL				0x2C
#define   CROPPING_EN				BIT(18)
#define   COMPRESSION_DIS			BIT(17)
#define   PIX_DEPTH_8BIT_EN			BIT(1)
#define   CONFIG_READY				BIT(0)
#define DCSS_DTRC_DYDSADDR_EXT			0x30
#define DCSS_DTRC_DCDSADDR_EXT			0x34
#define DCSS_DTRC_DYTSADDR_EXT			0x38
#define DCSS_DTRC_DCTSADDR_EXT			0x3C
#define DCSS_DTRC_SYSSA_EXT			0x40
#define DCSS_DTRC_SYSEA_EXT			0x44
#define DCSS_DTRC_SUVSSA_EXT			0x48
#define DCSS_DTRC_SUVSEA_EXT			0x4C

#define DCSS_DTRC_INTEN				0xC0
#define DCSS_DTRC_FDINTR			0xC4
#define DCSS_DTRC_DTCTRL			0xC8
#define   CURRENT_FRAME				BIT(31)
#define   ADDRESS_ID_ENABLE			BIT(30)
#define   ENDIANNESS_10BIT			BIT(29)
#define   MERGE_ARID_ENABLE			BIT(28)
#define   NON_G1_2_SWAP_MODE_POS		24
#define   NON_G1_2_SWAP_MODE_MASK		GENMASK(27, 24)
#define   TABLE_DATA_SWAP_POS			20
#define   TABLE_DATA_SWAP_MASK			GENMASK(23, 20)
#define   TILED_SWAP_POS			16
#define   TILED_SWAP_MASK			GENMASK(19, 16)
#define   RASTER_SWAP_POS			12
#define   RASTER_SWAP_MASK			GENMASK(15, 12)
#define   BURST_LENGTH_POS			4
#define   BURST_LENGTH_MASK			GENMASK(11, 4)
#define   G1_TILED_DATA_EN			BIT(3)
#define   HOT_RESET				BIT(2)
#define   ARIDR_MODE_DETILE			0
#define   ARIDR_MODE_BYPASS			2
#define DCSS_DTRC_ARIDR				0xCC
#define DCSS_DTRC_DTID2DDR			0xD0
#define DCSS_DTRC_CONFIG			0xD4
#define DCSS_DTRC_VER				0xD8
#define DCSS_DTRC_PFCTRL			0xF0
#define DCSS_DTRC_PFCR				0xF4
#define DCSS_DTRC_TOCR				0xF8

struct dcss_dtrc_ch {
	struct dcss_dtrc *dtrc;

	void __iomem *base_reg;
	u32 base_ofs;

	u32 xres;
	u32 yres;
	u32 pix_format;
	u64 format_modifier;
	u32 y_dec_ofs;
	u32 uv_dec_ofs;

	int curr_frame;

	u32 dctl;

	bool bypass;
	bool running;

	int irq;
	int ch_num;
};

struct dcss_dtrc {
	struct device *dev;

	struct dcss_dtrc_ch ch[2];

	u32 ctx_id;
	struct dcss_ctxld *ctxld;
};

static irqreturn_t dcss_dtrc_irq_handler(int irq, void *data)
{
	struct dcss_dtrc_ch *ch = data;
	u32 b0, b1, curr_bank;

	b0 = dcss_readl(ch->base_reg + DCSS_DTRC_DCTL) & 0x1;
	b1 = dcss_readl(ch->base_reg + DTRC_F1_OFS + DCSS_DTRC_DCTL) & 0x1;
	curr_bank = dcss_readl(ch->base_reg + DCSS_DTRC_DTCTRL) >> 31;

	dcss_update(1, 1, ch->base_reg + DCSS_DTRC_FDINTR);

	return IRQ_HANDLED;
}

static int dcss_dtrc_irq_config(struct dcss_dtrc *dtrc, int ch_num)
{
	struct platform_device *pdev = to_platform_device(dtrc->dev);
	struct dcss_dtrc_ch *ch = &dtrc->ch[ch_num];
	char irq_name[20];
	int ret;

	sprintf(irq_name, "dtrc_ch%d", ch_num + 1);
	irq_name[8] = 0;

	ch->irq = platform_get_irq_byname(pdev, irq_name);
	if (ch->irq < 0) {
		dev_err(dtrc->dev, "dtrc: can't get DTRC irq\n");
		return ch->irq;
	}

	ret = request_irq(ch->irq, dcss_dtrc_irq_handler,
			  0, "dcss-dtrc", ch);
	if (ret) {
		ch->irq = 0;
		dev_err(dtrc->dev, "dtrc: irq request failed.\n");
		return ret;
	}

	dcss_writel(1, ch->base_reg + DCSS_DTRC_INTEN);

	return 0;
}

static int dcss_dtrc_ch_init_all(struct dcss_dtrc *dtrc, u32 dtrc_base)
{
	struct dcss_dtrc_ch *ch;
	int i, ret;

	for (i = 0; i < 2; i++) {
		ch = &dtrc->ch[i];

		ch->base_ofs = dtrc_base + i * 0x1000;

		ch->base_reg = ioremap(ch->base_ofs, SZ_4K);
		if (!ch->base_reg) {
			dev_err(dtrc->dev, "dtrc: unable to remap ch base\n");
			return -ENOMEM;
		}

		ch->ch_num = i;
		ch->dtrc = dtrc;

		ret = dcss_dtrc_irq_config(dtrc, i);
		if (ret)
			return ret;
	}

	return 0;
}

static void dcss_dtrc_write(struct dcss_dtrc_ch *ch, u32 val, u32 ofs)
{
	dcss_ctxld_write(ch->dtrc->ctxld, ch->dtrc->ctx_id,
			 val, ch->base_ofs + ofs);
}

static void dcss_dtrc_write_irqsafe(struct dcss_dtrc_ch *ch, u32 val, u32 ofs)
{
	dcss_ctxld_write_irqsafe(ch->dtrc->ctxld, ch->dtrc->ctx_id,
				 val, ch->base_ofs + ofs);
}

int dcss_dtrc_init(struct dcss_dev *dcss, unsigned long dtrc_base)
{
	struct dcss_dtrc *dtrc;

	dtrc = kzalloc(sizeof(*dtrc), GFP_KERNEL);
	if (!dtrc)
		return -ENOMEM;

	dcss->dtrc = dtrc;
	dtrc->dev = dcss->dev;
	dtrc->ctxld = dcss->ctxld;
	dtrc->ctx_id = CTX_SB_HP;

	if (dcss_dtrc_ch_init_all(dtrc, dtrc_base)) {
		struct dcss_dtrc_ch *ch;
		int i;

		for (i = 0; i < 2; i++) {
			ch = &dtrc->ch[i];

			if (ch->irq)
				free_irq(ch->irq, ch);

			if (ch->base_reg)
				iounmap(ch->base_reg);
		}

		kfree(dtrc);

		return -ENOMEM;
	}

	return 0;
}

void dcss_dtrc_exit(struct dcss_dtrc *dtrc)
{
	int ch_no;

	for (ch_no = 0; ch_no < 2; ch_no++) {
		struct dcss_dtrc_ch *ch = &dtrc->ch[ch_no];

		if (ch->base_reg) {
			/* reset the module to default */
			dcss_writel(HOT_RESET,
				    ch->base_reg + DCSS_DTRC_DTCTRL);
			iounmap(ch->base_reg);
		}

		if (ch->irq)
			free_irq(ch->irq, ch);
	}

	kfree(dtrc);
}

void dcss_dtrc_bypass(struct dcss_dtrc *dtrc, int ch_num)
{
	struct dcss_dtrc_ch *ch;

	if (ch_num == 0)
		return;

	ch = &dtrc->ch[ch_num - 1];

	if (ch->bypass)
		return;

	dcss_dtrc_write(ch, ARIDR_MODE_BYPASS, DCSS_DTRC_DTCTRL);
	dcss_dtrc_write(ch, 0, DCSS_DTRC_DYTSADDR);
	dcss_dtrc_write(ch, 0, DCSS_DTRC_DCTSADDR);
	dcss_dtrc_write(ch, 0x0f0e0100, DCSS_DTRC_ARIDR);
	dcss_dtrc_write(ch, 0x0f0e, DCSS_DTRC_DTID2DDR);

	ch->running = false;
	ch->bypass = true;
}

void dcss_dtrc_addr_set(struct dcss_dtrc *dtrc, int ch_num,
			u32 p1_ba, u32 p2_ba, uint64_t dec_table_ofs)
{
	struct dcss_dtrc_ch *ch;

	if (ch_num == 0)
		return;

	ch = &dtrc->ch[ch_num - 1];

	dcss_dtrc_write(ch, p1_ba, DCSS_DTRC_DYDSADDR);
	dcss_dtrc_write(ch, p2_ba, DCSS_DTRC_DCDSADDR);

	dcss_dtrc_write(ch, p1_ba, DTRC_F1_OFS + DCSS_DTRC_DYDSADDR);
	dcss_dtrc_write(ch, p2_ba, DTRC_F1_OFS + DCSS_DTRC_DCDSADDR);

	if (ch->format_modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED) {
		ch->y_dec_ofs = dec_table_ofs & 0xFFFFFFFF;
		ch->uv_dec_ofs = dec_table_ofs >> 32;

		dcss_dtrc_write(ch, p1_ba + ch->y_dec_ofs,
				DCSS_DTRC_DYTSADDR);
		dcss_dtrc_write(ch, p1_ba + ch->uv_dec_ofs,
				DCSS_DTRC_DCTSADDR);
		dcss_dtrc_write(ch, p1_ba + ch->y_dec_ofs,
				DTRC_F1_OFS + DCSS_DTRC_DYTSADDR);
		dcss_dtrc_write(ch, p1_ba + ch->uv_dec_ofs,
				DTRC_F1_OFS + DCSS_DTRC_DCTSADDR);
	}

	ch->bypass = false;
}

void dcss_dtrc_set_res(struct dcss_dtrc *dtrc, int ch_num,
		       struct drm_plane_state *state, u32 *dtrc_w, u32 *dtrc_h)
{
	struct drm_framebuffer *fb = state->fb;
	u32 pixel_format = fb->format->format;
	struct dcss_dtrc_ch *ch;
	u32 frame_height, frame_width;
	u32 crop_w, crop_h, crop_orig_w, crop_orig_h;
	int bank;
	u32 old_xres, old_yres, xres, yres;
	u32 x1, y1, x2, y2;
	u32 pix_depth;
	u16 width_align = 0;

	if (ch_num == 0)
		return;

	ch = &dtrc->ch[ch_num - 1];

	bank = dcss_readl(ch->base_reg + DCSS_DTRC_DTCTRL) >> 31;

	ch->pix_format = pixel_format;
	ch->format_modifier = fb->modifier;

	pix_depth = ch->pix_format == DRM_FORMAT_NV15 ? 10 : 8;

	old_xres = state->src_w >> 16;
	old_yres = state->src_h >> 16;

	x1 = (state->src.x1 >> 16) & ~1;
	y1 = (state->src.y1 >> 16) & ~1;
	x2 = state->src.x2 >> 16;
	y2 = state->src.y2 >> 16;

	xres = x2 - x1;
	yres = y2 - y1;

	frame_height = ((old_yres >> 3) << FRAME_HEIGHT_POS) & FRAME_HEIGHT_MASK;
	frame_width = ((old_xres >> 3) << FRAME_WIDTH_POS) & FRAME_WIDTH_MASK;

	dcss_dtrc_write(ch, frame_height | frame_width,
			DTRC_F1_OFS * bank + DCSS_DTRC_SIZE);

	dcss_dtrc_write(ch, frame_height | frame_width,
			DTRC_F1_OFS * (bank ^ 1) + DCSS_DTRC_SIZE);

	/*
	 * Image original size is aligned:
	 *   - 128 pixels for width (8-bit) or 256 (10-bit);
	 *   - 8 lines for height;
	 */
	width_align = ch->pix_format == DRM_FORMAT_NV15 ? 0xff : 0x7f;

	if (xres == old_xres && !(xres & width_align) &&
	    yres == old_yres && !(yres & 0xf)) {
		ch->dctl &= ~CROPPING_EN;
		goto exit;
	}

	/* align the image size: down align for compressed formats */
	if (ch->format_modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED && x1)
		xres = xres & ~width_align;
	else
		xres = (xres + width_align) & ~width_align;

	if (ch->format_modifier == DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED && y1)
		yres = yres & ~0xf;
	else
		yres = (yres + 0xf) & ~0xf;

	crop_orig_w = (x1 << CROP_WIDTH_POS) & CROP_WIDTH_MASK;
	crop_orig_h = (y1 << CROP_HEIGHT_POS) & CROP_HEIGHT_MASK;

	dcss_dtrc_write(ch, crop_orig_w | crop_orig_h,
			DCSS_DTRC_CROPORIG);
	dcss_dtrc_write(ch, crop_orig_w | crop_orig_h,
			DTRC_F1_OFS + DCSS_DTRC_CROPORIG);

	crop_w = (xres << CROP_WIDTH_POS) & CROP_WIDTH_MASK;
	crop_h = (yres << CROP_HEIGHT_POS) & CROP_HEIGHT_MASK;

	dcss_dtrc_write(ch, crop_w | crop_h,
			DTRC_F1_OFS * bank + DCSS_DTRC_CROPSIZE);
	dcss_dtrc_write(ch, crop_w | crop_h,
			DTRC_F1_OFS * (bank ^ 1) + DCSS_DTRC_CROPSIZE);

	ch->dctl |= CROPPING_EN;

exit:
	dcss_dtrc_write(ch, xres * yres * pix_depth / 8,
			DCSS_DTRC_SYSEA);
	dcss_dtrc_write(ch, xres * yres * pix_depth / 8,
			DTRC_F1_OFS + DCSS_DTRC_SYSEA);

	dcss_dtrc_write(ch, 0x10000000 + xres * yres * pix_depth / 8 / 2,
			DCSS_DTRC_SUVSEA);
	dcss_dtrc_write(ch, 0x10000000 + xres * yres * pix_depth / 8 / 2,
			DTRC_F1_OFS + DCSS_DTRC_SUVSEA);

	*dtrc_w = xres;
	*dtrc_h = yres;

	if (ch->running)
		return;

	dcss_dtrc_write(ch, 0x0, DCSS_DTRC_SYSSA);
	dcss_dtrc_write(ch, 0x0, DTRC_F1_OFS + DCSS_DTRC_SYSSA);

	dcss_dtrc_write(ch, 0x10000000, DCSS_DTRC_SUVSSA);
	dcss_dtrc_write(ch, 0x10000000, DTRC_F1_OFS + DCSS_DTRC_SUVSSA);
}

void dcss_dtrc_enable(struct dcss_dtrc *dtrc, int ch_num, bool enable)
{
	struct dcss_dtrc_ch *ch;
	int curr_frame;
	u32 fdctl, dtctrl;

	if (ch_num == 0)
		return;

	ch = &dtrc->ch[ch_num - 1];

	if (ch->bypass)
		return;

	if (!enable) {
		ch->running = false;
		return;
	}

	if (ch->running)
		return;

	dcss_update(HOT_RESET, HOT_RESET, ch->base_reg + DCSS_DTRC_DTCTRL);
	while (dcss_readl(ch->base_reg + DCSS_DTRC_DTCTRL) & HOT_RESET)
		usleep_range(100, 200);

	dcss_dtrc_write(ch, 0x0f0e0100,
			DCSS_DTRC_ARIDR);
	dcss_dtrc_write(ch, 0x0f0e,
			DCSS_DTRC_DTID2DDR);

	dtctrl = ADDRESS_ID_ENABLE | MERGE_ARID_ENABLE |
		 ((0xF << TABLE_DATA_SWAP_POS) & TABLE_DATA_SWAP_MASK) |
		 ((0x10 << BURST_LENGTH_POS) & BURST_LENGTH_MASK);

	if (ch->format_modifier == DRM_FORMAT_MOD_VSI_G1_TILED)
		dtctrl |= G1_TILED_DATA_EN;

	dcss_dtrc_write(ch, dtctrl, DCSS_DTRC_DTCTRL);

	curr_frame = dcss_readl(ch->base_reg + DCSS_DTRC_DTCTRL) >> 31;

	fdctl = ch->dctl & ~(PIX_DEPTH_8BIT_EN | COMPRESSION_DIS);

	fdctl |= ch->pix_format == DRM_FORMAT_NV15 ? 0 : PIX_DEPTH_8BIT_EN;

	if (ch->format_modifier != DRM_FORMAT_MOD_VSI_G2_TILED_COMPRESSED)
		fdctl |= COMPRESSION_DIS;

	dcss_dtrc_write(ch, fdctl,
			(curr_frame ^ 1) * DTRC_F1_OFS + DCSS_DTRC_DCTL);
	dcss_dtrc_write(ch, fdctl | CONFIG_READY,
			curr_frame * DTRC_F1_OFS + DCSS_DTRC_DCTL);

	ch->curr_frame = curr_frame;
	ch->dctl = fdctl;
	ch->running = true;
}

bool dcss_dtrc_ch_running(struct dcss_dtrc *dtrc, int ch_num)
{
	struct dcss_dtrc_ch *ch;

	if (ch_num == 0)
		return false;

	ch = &dtrc->ch[ch_num - 1];

	return ch->running;
}

bool dcss_dtrc_is_running(struct dcss_dtrc *dtrc)
{
	return dtrc->ch[0].running || dtrc->ch[1].running;
}

static void dcss_dtrc_ch_switch_banks(struct dcss_dtrc *dtrc, int dtrc_ch)
{
	struct dcss_dtrc_ch *ch = &dtrc->ch[dtrc_ch];
	u32 b0, b1;

	if (!ch->running)
		return;

	b0 = dcss_readl(ch->base_reg + DCSS_DTRC_DCTL) & 0x1;
	b1 = dcss_readl(ch->base_reg + DTRC_F1_OFS + DCSS_DTRC_DCTL) & 0x1;

	ch->curr_frame = dcss_readl(ch->base_reg + DCSS_DTRC_DTCTRL) >> 31;

	dcss_dtrc_write_irqsafe(ch, ch->dctl | CONFIG_READY,
				(ch->curr_frame ^ 1) * DTRC_F1_OFS + DCSS_DTRC_DCTL);
}

void dcss_dtrc_switch_banks(struct dcss_dtrc *dtrc)
{
	dcss_dtrc_ch_switch_banks(dtrc, 0);
	dcss_dtrc_ch_switch_banks(dtrc, 1);
}
