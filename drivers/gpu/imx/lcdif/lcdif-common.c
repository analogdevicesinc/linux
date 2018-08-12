/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/busfreq-imx.h>
#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/media-bus-format.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <drm/drm_fourcc.h>
#include <video/imx-lcdif.h>
#include <video/videomode.h>

#include "lcdif-regs.h"

#define DRIVER_NAME "imx-lcdif"

/* TODO: add this to platform data later */
#define DISP_MIX_SFT_RSTN_CSR		0x00
#define DISP_MIX_CLK_EN_CSR		0x04

/* 'DISP_MIX_SFT_RSTN_CSR' bit fields */
#define BUS_RSTN_BLK_SYNC_SFT_EN	BIT(6)

/* 'DISP_MIX_CLK_EN_CSR' bit fields */
#define BUS_BLK_CLK_SFT_EN		BIT(12)
#define LCDIF_PIXEL_CLK_SFT_EN		BIT(7)
#define LCDIF_APB_CLK_SFT_EN		BIT(6)

struct lcdif_soc {
	struct device *dev;

	int irq;
	void __iomem *base;
	struct regmap *gpr;
	atomic_t rpm_suspended;

	struct clk *clk_pix;
	struct clk *clk_disp_axi;
	struct clk *clk_disp_apb;
};

struct lcdif_soc_pdata {
	bool hsync_invert;
	bool vsync_invert;
	bool de_invert;
};

struct lcdif_platform_reg {
	struct lcdif_client_platformdata pdata;
	char *name;
};

struct lcdif_platform_reg client_reg[] = {
	{
		.pdata = { },
		.name  = "imx-lcdif-crtc",
	},
};

struct lcdif_soc_pdata imx8mm_pdata = {
	.hsync_invert = true,
	.vsync_invert = true,
	.de_invert    = true,
};

static const struct of_device_id imx_lcdif_dt_ids[] = {
	{ .compatible = "fsl,imx8mm-lcdif", .data = &imx8mm_pdata, },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, lcdif_dt_ids);

#ifdef CONFIG_PM
static int imx_lcdif_runtime_suspend(struct device *dev);
static int imx_lcdif_runtime_resume(struct device *dev);
#else
static int imx_lcdif_runtime_suspend(struct device *dev)
{
	return 0;
}
static int imx_lcdif_runtime_resume(struct device *dev)
{
	return 0;
}
#endif

void disp_mix_bus_rstn_reset(struct regmap *gpr, bool reset)
{
	if (!reset)
		/* release reset */
		regmap_update_bits(gpr, DISP_MIX_SFT_RSTN_CSR,
				   BUS_RSTN_BLK_SYNC_SFT_EN,
				   BUS_RSTN_BLK_SYNC_SFT_EN);
	else
		/* hold reset */
		regmap_update_bits(gpr, DISP_MIX_SFT_RSTN_CSR,
				   BUS_RSTN_BLK_SYNC_SFT_EN,
				   0x0);
}

void disp_mix_lcdif_clks_enable(struct regmap *gpr, bool enable)
{
	if (enable)
		/* enable lcdif clks */
		regmap_update_bits(gpr, DISP_MIX_CLK_EN_CSR,
				   LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN,
				   LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN);
	else
		/* disable lcdif clks */
		regmap_update_bits(gpr, DISP_MIX_CLK_EN_CSR,
				   LCDIF_PIXEL_CLK_SFT_EN | LCDIF_APB_CLK_SFT_EN,
				   0x0);
}

static int lcdif_enable_clocks(struct lcdif_soc *lcdif)
{
	int ret;

	if (lcdif->clk_disp_axi) {
		ret = clk_prepare_enable(lcdif->clk_disp_axi);
		if (ret)
			return ret;
	}

	if (lcdif->clk_disp_apb) {
		ret = clk_prepare_enable(lcdif->clk_disp_apb);
		if (ret)
			goto disable_disp_axi;
	}

	ret = clk_prepare_enable(lcdif->clk_pix);
	if (ret)
		goto disable_disp_apb;

	return 0;

disable_disp_apb:
	if (lcdif->clk_disp_apb)
		clk_disable_unprepare(lcdif->clk_disp_apb);
disable_disp_axi:
	if (lcdif->clk_disp_axi)
		clk_disable_unprepare(lcdif->clk_disp_axi);

	return ret;
}

static void lcdif_disable_clocks(struct lcdif_soc *lcdif)
{
	clk_disable_unprepare(lcdif->clk_pix);

	if (lcdif->clk_disp_axi)
		clk_disable_unprepare(lcdif->clk_disp_axi);

	if (lcdif->clk_disp_apb)
		clk_disable_unprepare(lcdif->clk_disp_apb);
}

int lcdif_vblank_irq_get(struct lcdif_soc *lcdif)
{
	return lcdif->irq;
}
EXPORT_SYMBOL(lcdif_vblank_irq_get);

void lcdif_dump_registers(struct lcdif_soc *lcdif)
{
	pr_info("%#x	: %#x\n", LCDIF_CTRL,
				  readl(lcdif->base + LCDIF_CTRL));
	pr_info("%#x	: %#x\n", LCDIF_CTRL1,
				  readl(lcdif->base + LCDIF_CTRL1));
	pr_info("%#x	: %#x\n", LCDIF_CTRL2,
				  readl(lcdif->base + LCDIF_CTRL2));
	pr_info("%#x	: %#x\n", LCDIF_TRANSFER_COUNT,
				  readl(lcdif->base + LCDIF_TRANSFER_COUNT));
	pr_info("%#x	: %#x\n", LCDIF_CUR_BUF,
				  readl(lcdif->base + LCDIF_CUR_BUF));
	pr_info("%#x	: %#x\n", LCDIF_NEXT_BUF,
				  readl(lcdif->base + LCDIF_NEXT_BUF));
	pr_info("%#x	: %#x\n", LCDIF_VDCTRL0,
				  readl(lcdif->base + LCDIF_VDCTRL0));
	pr_info("%#x	: %#x\n", LCDIF_VDCTRL1,
				  readl(lcdif->base + LCDIF_VDCTRL1));
	pr_info("%#x	: %#x\n", LCDIF_VDCTRL2,
				  readl(lcdif->base + LCDIF_VDCTRL2));
	pr_info("%#x	: %#x\n", LCDIF_VDCTRL3,
				  readl(lcdif->base + LCDIF_VDCTRL3));
	pr_info("%#x	: %#x\n", LCDIF_VDCTRL4,
				  readl(lcdif->base + LCDIF_VDCTRL4));
}
EXPORT_SYMBOL(lcdif_dump_registers);

void lcdif_vblank_irq_enable(struct lcdif_soc *lcdif)
{
	writel(CTRL1_CUR_FRAME_DONE_IRQ, lcdif->base + LCDIF_CTRL1 + REG_CLR);
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN, lcdif->base + LCDIF_CTRL1 + REG_SET);
}
EXPORT_SYMBOL(lcdif_vblank_irq_enable);

void lcdif_vblank_irq_disable(struct lcdif_soc *lcdif)
{
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN, lcdif->base + LCDIF_CTRL1 + REG_CLR);
	writel(CTRL1_CUR_FRAME_DONE_IRQ, lcdif->base + LCDIF_CTRL1 + REG_CLR);
}
EXPORT_SYMBOL(lcdif_vblank_irq_disable);

void lcdif_vblank_irq_clear(struct lcdif_soc *lcdif)
{
	writel(CTRL1_CUR_FRAME_DONE_IRQ, lcdif->base + LCDIF_CTRL1 + REG_CLR);
}
EXPORT_SYMBOL(lcdif_vblank_irq_clear);

static uint32_t lcdif_get_bpp_from_fmt(uint32_t format)
{
	/* TODO: only support RGB for now */

	switch (format) {
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		return 16;
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		return 32;
	default:
		/* unsupported format */
		return 0;
	}
}

/*
 * Get the bus format supported by LCDIF
 * according to drm fourcc format
 */
int lcdif_get_bus_fmt_from_pix_fmt(struct lcdif_soc *lcdif,
				   uint32_t format)
{
	uint32_t bpp;

	bpp = lcdif_get_bpp_from_fmt(format);
	if (!bpp)
		return -EINVAL;

	switch (bpp) {
	case 16:
		return MEDIA_BUS_FMT_RGB565_1X16;
	case 18:
		return MEDIA_BUS_FMT_RGB666_1X18;
	case 24:
	case 32:
		return MEDIA_BUS_FMT_RGB888_1X24;
	default:
		return -EINVAL;
	}
}
EXPORT_SYMBOL(lcdif_get_bus_fmt_from_pix_fmt);

int lcdif_set_pix_fmt(struct lcdif_soc *lcdif, u32 format)
{
	struct drm_format_name_buf format_name;
	u32 ctrl = 0, ctrl1 = 0;

	/* TODO: lcdif should be disabled to set pixel format */

	ctrl  = readl(lcdif->base + LCDIF_CTRL);
	ctrl1 = readl(lcdif->base + LCDIF_CTRL1);

	/* clear pixel format related bits */
	ctrl  &= ~(CTRL_SHIFT_NUM(0x3f)  | CTRL_INPUT_SWIZZLE(0x3) |
		   CTRL_CSC_SWIZZLE(0x3) | CTRL_SET_WORD_LENGTH(0x3));

	ctrl1 &= ~CTRL1_SET_BYTE_PACKAGING(0xf);

	/* default is 'RGB' order */
	writel(CTRL2_ODD_LINE_PATTERN(0x7) |
	       CTRL2_EVEN_LINE_PATTERN(0x7),
	       lcdif->base + LCDIF_CTRL2 + REG_CLR);

	switch (format) {
		/* bpp 16 */
	case DRM_FORMAT_RGB565:
	case DRM_FORMAT_BGR565:
	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
	case DRM_FORMAT_ABGR1555:
	case DRM_FORMAT_XBGR1555:
		/* Data format */
		ctrl = (format == DRM_FORMAT_RGB565 ||
			format == DRM_FORMAT_BGR565) ?
			(ctrl & ~CTRL_DF16) : (ctrl | CTRL_DF16);

		ctrl |= CTRL_SET_WORD_LENGTH(0x0);

		/* Byte packing */
		ctrl1 |= CTRL1_SET_BYTE_PACKAGING(0xf);

		/* 'BGR' order */
		if (format == DRM_FORMAT_BGR565		||
		    format == DRM_FORMAT_ABGR1555	||
		    format == DRM_FORMAT_XBGR1555)
			writel(CTRL2_ODD_LINE_PATTERN(0x5) |
			       CTRL2_EVEN_LINE_PATTERN(0x5),
			       lcdif->base + LCDIF_CTRL2 + REG_SET);
		break;
		/* bpp 32 */
	case DRM_FORMAT_ARGB8888:
	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ABGR8888:
	case DRM_FORMAT_XBGR8888:
	case DRM_FORMAT_RGBA8888:
	case DRM_FORMAT_RGBX8888:
		/*Data format */
		ctrl &= ~CTRL_DF24;
		ctrl |= CTRL_SET_WORD_LENGTH(3);

		if (format == DRM_FORMAT_RGBA8888 ||
		    format == DRM_FORMAT_RGBX8888)
			ctrl |= CTRL_SHIFT_DIR(1) | CTRL_SHIFT_NUM(8);

		/* Byte packing */
		ctrl1 |= CTRL1_SET_BYTE_PACKAGING(0x7);

		/* 'BGR' order */
		if (format == DRM_FORMAT_ABGR8888 ||
		    format == DRM_FORMAT_XBGR8888)
			writel(CTRL2_ODD_LINE_PATTERN(0x5) |
			       CTRL2_EVEN_LINE_PATTERN(0x5),
			       lcdif->base + LCDIF_CTRL2 + REG_SET);
		break;
	default:
		dev_err(lcdif->dev, "unsupported pixel format: %s\n",
			drm_get_format_name(format, &format_name));
		return -EINVAL;
	}

	writel(ctrl,  lcdif->base + LCDIF_CTRL);
	writel(ctrl1, lcdif->base + LCDIF_CTRL1);

	return 0;
}
EXPORT_SYMBOL(lcdif_set_pix_fmt);

void lcdif_set_bus_fmt(struct lcdif_soc *lcdif, u32 bus_format)
{
	u32 bus_width;

	switch (bus_format) {
	case MEDIA_BUS_FMT_RGB565_1X16:
		bus_width = CTRL_SET_BUS_WIDTH(STMLCDIF_16BIT);
		break;
	case MEDIA_BUS_FMT_RGB666_1X18:
		bus_width = CTRL_SET_BUS_WIDTH(STMLCDIF_18BIT);
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		bus_width = CTRL_SET_BUS_WIDTH(STMLCDIF_24BIT);
		break;
	default:
		dev_err(lcdif->dev, "unknown bus format: %#x\n", bus_format);
		return;
	}

	writel(CTRL_SET_BUS_WIDTH(0x3), lcdif->base + LCDIF_CTRL + REG_CLR);
	writel(bus_width, lcdif->base + LCDIF_CTRL + REG_SET);
}
EXPORT_SYMBOL(lcdif_set_bus_fmt);

void lcdif_set_fb_addr(struct lcdif_soc *lcdif, int id, u32 addr)
{
	switch (id) {
	case 0:
		/* primary plane */
		writel(addr, lcdif->base + LCDIF_NEXT_BUF);
		break;
	default:
		/* TODO: add overlay support */
		return;
	}
}
EXPORT_SYMBOL(lcdif_set_fb_addr);

void lcdif_set_fb_hcrop(struct lcdif_soc *lcdif, u32 src_w,
			u32 fb_w, bool crop)
{
	u32 mask_cnt, htotal, hcount;
	u32 vdctrl2, vdctrl3, vdctrl4, transfer_count;
	u32 pigeon_12_0, pigeon_12_1, pigeon_12_2;

	if (!crop) {
		writel(0x0, lcdif->base + HW_EPDC_PIGEON_12_0);
		writel(0x0, lcdif->base + HW_EPDC_PIGEON_12_1);

		return;
	}

	/* transfer_count's hcount, vdctrl2's htotal and vdctrl4's
	 * H_VALID_DATA_CNT should use fb width instead of hactive
	 * when requires cropping.
	 * */
	transfer_count = readl(lcdif->base + LCDIF_TRANSFER_COUNT);
	hcount = TRANSFER_COUNT_GET_HCOUNT(transfer_count);

	transfer_count &= ~TRANSFER_COUNT_SET_HCOUNT(0xffff);
	transfer_count |= TRANSFER_COUNT_SET_HCOUNT(fb_w);
	writel(transfer_count, lcdif->base + LCDIF_TRANSFER_COUNT);

	vdctrl2 = readl(lcdif->base + LCDIF_VDCTRL2);
	htotal  = VDCTRL2_GET_HSYNC_PERIOD(vdctrl2);
	htotal  += fb_w - hcount;
	vdctrl2 &= ~VDCTRL2_SET_HSYNC_PERIOD(0x3ffff);
	vdctrl2 |= VDCTRL2_SET_HSYNC_PERIOD(htotal);
	writel(vdctrl2, lcdif->base + LCDIF_VDCTRL2);

	vdctrl4 = readl(lcdif->base + LCDIF_VDCTRL4);
	vdctrl4 &= ~SET_DOTCLK_H_VALID_DATA_CNT(0x3ffff);
	vdctrl4 |= SET_DOTCLK_H_VALID_DATA_CNT(fb_w);
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);

	/* configure related pigeon registers */
	vdctrl3  = readl(lcdif->base + LCDIF_VDCTRL3);
	mask_cnt = GET_HOR_WAIT_CNT(vdctrl3) - 5;

	pigeon_12_0 = PIGEON_12_0_SET_STATE_MASK(0x24)		|
		      PIGEON_12_0_SET_MASK_CNT(mask_cnt)	|
		      PIGEON_12_0_SET_MASK_CNT_SEL(0x6)		|
		      PIGEON_12_0_POL_ACTIVE_LOW		|
		      PIGEON_12_0_EN;
	writel(pigeon_12_0, lcdif->base + HW_EPDC_PIGEON_12_0);

	pigeon_12_1 = PIGEON_12_1_SET_CLR_CNT(src_w) |
		      PIGEON_12_1_SET_SET_CNT(0x0);
	writel(pigeon_12_1, lcdif->base + HW_EPDC_PIGEON_12_1);

	pigeon_12_2 = 0x0;
	writel(pigeon_12_2, lcdif->base + HW_EPDC_PIGEON_12_2);
}
EXPORT_SYMBOL(lcdif_set_fb_hcrop);


void lcdif_set_mode(struct lcdif_soc *lcdif, struct videomode *vmode)
{
	const struct of_device_id *of_id =
			of_match_device(imx_lcdif_dt_ids, lcdif->dev);
	const struct lcdif_soc_pdata *soc_pdata = of_id->data;
	u32 vdctrl0, vdctrl1, vdctrl2, vdctrl3, vdctrl4, htotal;

	/* Clear the FIFO */
	writel(CTRL1_FIFO_CLEAR, lcdif->base + LCDIF_CTRL1 + REG_SET);
	writel(CTRL1_FIFO_CLEAR, lcdif->base + LCDIF_CTRL1 + REG_CLR);

	/* set pixel clock rate */
	clk_disable_unprepare(lcdif->clk_pix);
	clk_set_rate(lcdif->clk_pix, vmode->pixelclock);
	clk_prepare_enable(lcdif->clk_pix);

	/* config display timings */
	writel(TRANSFER_COUNT_SET_VCOUNT(vmode->vactive) |
	       TRANSFER_COUNT_SET_HCOUNT(vmode->hactive),
	       lcdif->base + LCDIF_TRANSFER_COUNT);

	vdctrl0 = VDCTRL0_ENABLE_PRESENT		|
		  VDCTRL0_VSYNC_PERIOD_UNIT 		|
		  VDCTRL0_VSYNC_PULSE_WIDTH_UNIT	|
		  VDCTRL0_SET_VSYNC_PULSE_WIDTH(vmode->vsync_len);

	/* Polarities */
	if (soc_pdata) {
		if ((soc_pdata->hsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_HSYNC_LOW) ||
		    (!soc_pdata->hsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_HSYNC_HIGH))
				vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;

		if ((soc_pdata->vsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_VSYNC_LOW) ||
		    (!soc_pdata->vsync_invert &&
		     vmode->flags & DISPLAY_FLAGS_VSYNC_HIGH))
				vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;

		if ((soc_pdata->de_invert &&
		     vmode->flags & DISPLAY_FLAGS_DE_LOW) ||
		    (!soc_pdata->de_invert &&
		     vmode->flags & DISPLAY_FLAGS_DE_HIGH))
				vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
	} else {
		if (vmode->flags & DISPLAY_FLAGS_HSYNC_HIGH)
			vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
		if (vmode->flags & DISPLAY_FLAGS_VSYNC_HIGH)
			vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
		if (vmode->flags & DISPLAY_FLAGS_DE_HIGH)
			vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
	}

	if (vmode->flags & DISPLAY_FLAGS_PIXDATA_POSEDGE)
		vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

	writel(vdctrl0, lcdif->base + LCDIF_VDCTRL0);

	vdctrl1 = vmode->vactive + vmode->vsync_len +
		  vmode->vfront_porch + vmode->vback_porch;
	writel(vdctrl1, lcdif->base + LCDIF_VDCTRL1);

	htotal = vmode->hactive + vmode->hsync_len +
		 vmode->hfront_porch + vmode->hback_porch;
	vdctrl2 = VDCTRL2_SET_HSYNC_PULSE_WIDTH(vmode->hsync_len) |
		  VDCTRL2_SET_HSYNC_PERIOD(htotal);
	writel(vdctrl2, lcdif->base + LCDIF_VDCTRL2);

	vdctrl3 = SET_HOR_WAIT_CNT(vmode->hsync_len + vmode->hback_porch) |
		  SET_VERT_WAIT_CNT(vmode->vsync_len + vmode->vback_porch);
	writel(vdctrl3, lcdif->base + LCDIF_VDCTRL3);

	vdctrl4 = SET_DOTCLK_H_VALID_DATA_CNT(vmode->hactive);
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);
}
EXPORT_SYMBOL(lcdif_set_mode);

void lcdif_enable_controller(struct lcdif_soc *lcdif)
{
	u32 ctrl2, vdctrl4;

	ctrl2	= readl(lcdif->base + LCDIF_CTRL2);
	vdctrl4 = readl(lcdif->base + LCDIF_VDCTRL4);

	ctrl2 &= ~CTRL2_OUTSTANDING_REQS(0x7);
	ctrl2 |= CTRL2_OUTSTANDING_REQS(REQ_16);
	writel(ctrl2, lcdif->base + LCDIF_CTRL2);

	/* Continous dotclock mode */
	writel(CTRL_BYPASS_COUNT | CTRL_DOTCLK_MODE,
	       lcdif->base + LCDIF_CTRL + REG_SET);

	/* enable the SYNC signals first, then the DMA engine */
	vdctrl4 |= VDCTRL4_SYNC_SIGNALS_ON;
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);

	/* enable underflow recovery */
	writel(CTRL1_RECOVERY_ON_UNDERFLOW,
	       lcdif->base + LCDIF_CTRL1 + REG_SET);

	/* run lcdif */
	writel(CTRL_MASTER, lcdif->base + LCDIF_CTRL + REG_SET);
	writel(CTRL_RUN, lcdif->base + LCDIF_CTRL + REG_SET);
}
EXPORT_SYMBOL(lcdif_enable_controller);

void lcdif_disable_controller(struct lcdif_soc *lcdif)
{
	int ret;
	u32 ctrl, vdctrl4;

	writel(CTRL_RUN, lcdif->base + LCDIF_CTRL + REG_CLR);
	writel(CTRL_DOTCLK_MODE, lcdif->base + LCDIF_CTRL + REG_CLR);

	ret = readl_poll_timeout(lcdif->base + LCDIF_CTRL, ctrl,
				 !(ctrl & CTRL_RUN), 0, 1000);
	if (WARN_ON(ret))
		dev_err(lcdif->dev, "disable lcdif run timeout\n");

	writel(CTRL_MASTER, lcdif->base + LCDIF_CTRL + REG_CLR);

	vdctrl4 = readl(lcdif->base + LCDIF_VDCTRL4);
	vdctrl4 &= ~VDCTRL4_SYNC_SIGNALS_ON;
	writel(vdctrl4, lcdif->base + LCDIF_VDCTRL4);
}
EXPORT_SYMBOL(lcdif_disable_controller);

static int platform_remove_device_fn(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);

	return 0;
}

static void platform_device_unregister_children(struct platform_device *pdev)
{
	device_for_each_child(&pdev->dev, NULL, platform_remove_device_fn);
}

static int lcdif_add_client_devices(struct lcdif_soc *lcdif)
{
	int ret = 0, i;
	struct device *dev = lcdif->dev;
	struct platform_device *pdev = NULL;
	struct device_node *of_node;

	for (i = 0; i < ARRAY_SIZE(client_reg); i++) {
		of_node = of_graph_get_port_by_id(dev->of_node, i);
		if (!of_node) {
			dev_info(dev, "no port@%d node in %s\n",
				 i, dev->of_node->full_name);
			continue;
		}
		of_node_put(of_node);

		pdev = platform_device_alloc(client_reg[i].name, i);
		if (!pdev) {
			dev_err(dev, "Can't allocate port pdev\n");
			ret = -ENOMEM;
			goto err_register;
		}

		pdev->dev.parent = dev;
		client_reg[i].pdata.of_node = of_node;

		ret = platform_device_add_data(pdev, &client_reg[i].pdata,
					       sizeof(client_reg[i].pdata));
		if (!ret)
			ret = platform_device_add(pdev);
		if (ret) {
			platform_device_put(pdev);
			goto err_register;
		}

		pdev->dev.of_node = of_node;
	}

	if (!pdev)
		return -ENODEV;

	return 0;

err_register:
	platform_device_unregister_children(to_platform_device(dev));
	return ret;
}

static int imx_lcdif_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct lcdif_soc *lcdif;
	struct resource *res;

	dev_dbg(dev, "%s: probe begin\n", __func__);

	lcdif = devm_kzalloc(dev, sizeof(*lcdif), GFP_KERNEL);
	if (!lcdif) {
		dev_err(dev, "Can't allocate 'lcdif_soc' structure\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	lcdif->irq = platform_get_irq(pdev, 0);
	if (lcdif->irq < 0)
		return -ENODEV;

	lcdif->clk_pix = devm_clk_get(dev, "pix");
	if (IS_ERR(lcdif->clk_pix))
		return PTR_ERR(lcdif->clk_pix);

	lcdif->clk_disp_axi = devm_clk_get(dev, "disp-axi");
	if (IS_ERR(lcdif->clk_disp_axi))
		lcdif->clk_disp_axi = NULL;

	lcdif->clk_disp_apb = devm_clk_get(dev, "disp-apb");
	if (IS_ERR(lcdif->clk_disp_apb))
		lcdif->clk_disp_apb = NULL;

	lcdif->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(lcdif->base))
		return PTR_ERR(lcdif->base);

	lcdif->gpr = syscon_regmap_lookup_by_phandle(np, "lcdif-gpr");
	if (IS_ERR(lcdif->gpr))
		return PTR_ERR(lcdif->gpr);

	lcdif->dev = dev;
	platform_set_drvdata(pdev, lcdif);

	atomic_set(&lcdif->rpm_suspended, 0);
	pm_runtime_enable(dev);
	atomic_inc(&lcdif->rpm_suspended);

	dev_dbg(dev, "%s: probe end\n", __func__);

	return lcdif_add_client_devices(lcdif);
}

static int imx_lcdif_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int imx_lcdif_suspend(struct device *dev)
{
	return imx_lcdif_runtime_suspend(dev);
}

static int imx_lcdif_resume(struct device *dev)
{
	return imx_lcdif_runtime_resume(dev);
}
#else
static int imx_lcdif_suspend(struct device *dev)
{
	return 0;
}
static int imx_lcdif_resume(struct device *dev)
{
	return 0;
}
#endif

#ifdef CONFIG_PM
static int imx_lcdif_runtime_suspend(struct device *dev)
{
	struct lcdif_soc *lcdif = dev_get_drvdata(dev);

	if (atomic_inc_return(&lcdif->rpm_suspended) > 1)
		return 0;

	lcdif_disable_clocks(lcdif);

	release_bus_freq(BUS_FREQ_HIGH);

	return 0;
}

static int imx_lcdif_runtime_resume(struct device *dev)
{
	int ret = 0;
	struct lcdif_soc *lcdif = dev_get_drvdata(dev);

	if (unlikely(!atomic_read(&lcdif->rpm_suspended))) {
		dev_warn(lcdif->dev, "Unbalanced %s!\n", __func__);
		return 0;
	}

	if (!atomic_dec_and_test(&lcdif->rpm_suspended))
		return 0;

	request_bus_freq(BUS_FREQ_HIGH);

	ret = lcdif_enable_clocks(lcdif);
	if (ret) {
		release_bus_freq(BUS_FREQ_HIGH);
		return ret;
	}

	disp_mix_bus_rstn_reset(lcdif->gpr, false);
	disp_mix_lcdif_clks_enable(lcdif->gpr, true);

	/* Pull LCDIF out of reset */
	writel(0x0, lcdif->base + LCDIF_CTRL);

	return ret;
}
#endif

static const struct dev_pm_ops imx_lcdif_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(imx_lcdif_suspend, imx_lcdif_resume)
	SET_RUNTIME_PM_OPS(imx_lcdif_runtime_suspend,
			   imx_lcdif_runtime_resume, NULL)
};

struct platform_driver imx_lcdif_driver = {
	.probe    = imx_lcdif_probe,
	.remove   = imx_lcdif_remove,
	.driver   = {
		.name = DRIVER_NAME,
		.of_match_table = imx_lcdif_dt_ids,
		.pm = &imx_lcdif_pm_ops,
	},
};

module_platform_driver(imx_lcdif_driver);

MODULE_DESCRIPTION("NXP i.MX LCDIF Display Controller driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
