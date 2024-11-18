/*
 * Copyright (C) 2010 Juergen Beisert, Pengutronix
 *
 * This code is based on:
 * Author: Vitaly Wool <vital@embeddedalley.com>
 *
 * Copyright 2017-2019,2021 NXP
 * Copyright 2008-2015 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2008 Embedded Alley Solutions, Inc All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define DRIVER_NAME "mxsfb"

/**
 * @file
 * @brief LCDIF driver for i.MX23 and i.MX28
 *
 * The LCDIF support four modes of operation
 * - MPU interface (to drive smart displays) -> not supported yet
 * - VSYNC interface (like MPU interface plus Vsync) -> not supported yet
 * - Dotclock interface (to drive LC displays with RGB data and sync signals)
 * - DVI (to drive ITU-R BT656)  -> not supported yet
 *
 * This driver depends on a correct setup of the pins used for this purpose
 * (platform specific).
 *
 * For the developer: Don't forget to set the data bus width to the display
 * in the imx_fb_videomode structure. You will else end up with ugly colours.
 * If you fight against jitter you can vary the clock delay. This is a feature
 * of the i.MX28 and you can vary it between 2 ns ... 8 ns in 2 ns steps. Give
 * the required value in the imx_fb_videomode structure.
 */

#include <linux/busfreq-imx.h>
#include <linux/console.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_qos.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>
#include <linux/pinctrl/consumer.h>
#include <linux/fb.h>
#include <linux/mxcfb.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/uaccess.h>

#include "mxc/mxc_dispdrv.h"

#define REG_SET	4
#define REG_CLR	8

#define LCDC_CTRL			0x00
#define LCDC_CTRL1			0x10
#define LCDC_V4_CTRL2			0x20
#define LCDC_V3_TRANSFER_COUNT		0x20
#define LCDC_V4_TRANSFER_COUNT		0x30
#define LCDC_V4_CUR_BUF			0x40
#define LCDC_V4_NEXT_BUF		0x50
#define LCDC_V3_CUR_BUF			0x30
#define LCDC_V3_NEXT_BUF		0x40
#define LCDC_TIMING			0x60
#define LCDC_VDCTRL0			0x70
#define LCDC_VDCTRL1			0x80
#define LCDC_VDCTRL2			0x90
#define LCDC_VDCTRL3			0xa0
#define LCDC_VDCTRL4			0xb0
#define LCDC_DVICTRL0			0xc0
#define LCDC_DVICTRL1			0xd0
#define LCDC_DVICTRL2			0xe0
#define LCDC_DVICTRL3			0xf0
#define LCDC_DVICTRL4			0x100
#define LCDC_V4_DATA			0x180
#define LCDC_V3_DATA			0x1b0
#define LCDC_V4_DEBUG0			0x1d0
#define LCDC_V3_DEBUG0			0x1f0
#define LCDC_AS_CTRL			0x210
#define LCDC_AS_BUF			0x220
#define LCDC_AS_NEXT_BUF		0x230

#define CTRL_SFTRST			(1 << 31)
#define CTRL_CLKGATE			(1 << 30)
#define CTRL_BYPASS_COUNT		(1 << 19)
#define CTRL_VSYNC_MODE			(1 << 18)
#define CTRL_DOTCLK_MODE		(1 << 17)
#define CTRL_DATA_SELECT		(1 << 16)
#define CTRL_SET_BUS_WIDTH(x)		(((x) & 0x3) << 10)
#define CTRL_GET_BUS_WIDTH(x)		(((x) >> 10) & 0x3)
#define CTRL_SET_WORD_LENGTH(x)		(((x) & 0x3) << 8)
#define CTRL_GET_WORD_LENGTH(x)		(((x) >> 8) & 0x3)
#define CTRL_MASTER			(1 << 5)
#define CTRL_DF16			(1 << 3)
#define CTRL_DF18			(1 << 2)
#define CTRL_DF24			(1 << 1)
#define CTRL_RUN			(1 << 0)

#define CTRL1_RECOVERY_ON_UNDERFLOW		(1 << 24)
#define CTRL1_FIFO_CLEAR				(1 << 21)
#define CTRL1_SET_BYTE_PACKAGING(x)		(((x) & 0xf) << 16)
#define CTRL1_GET_BYTE_PACKAGING(x)		(((x) >> 16) & 0xf)
#define CTRL1_OVERFLOW_IRQ_EN			(1 << 15)
#define CTRL1_UNDERFLOW_IRQ_EN			(1 << 14)
#define CTRL1_CUR_FRAME_DONE_IRQ_EN		(1 << 13)
#define CTRL1_VSYNC_EDGE_IRQ_EN			(1 << 12)
#define CTRL1_OVERFLOW_IRQ				(1 << 11)
#define CTRL1_UNDERFLOW_IRQ				(1 << 10)
#define CTRL1_CUR_FRAME_DONE_IRQ		(1 << 9)
#define CTRL1_VSYNC_EDGE_IRQ			(1 << 8)
#define CTRL1_IRQ_ENABLE_MASK			(CTRL1_OVERFLOW_IRQ_EN | \
						 CTRL1_UNDERFLOW_IRQ_EN | \
						 CTRL1_CUR_FRAME_DONE_IRQ_EN | \
						 CTRL1_VSYNC_EDGE_IRQ_EN)
#define CTRL1_IRQ_ENABLE_SHIFT			12
#define CTRL1_IRQ_STATUS_MASK			(CTRL1_OVERFLOW_IRQ | \
						 CTRL1_UNDERFLOW_IRQ | \
						 CTRL1_CUR_FRAME_DONE_IRQ | \
						 CTRL1_VSYNC_EDGE_IRQ)
#define CTRL1_IRQ_STATUS_SHIFT			8

#define CTRL2_OUTSTANDING_REQS__REQ_16		(4 << 21)

#define TRANSFER_COUNT_SET_VCOUNT(x)	(((x) & 0xffff) << 16)
#define TRANSFER_COUNT_GET_VCOUNT(x)	(((x) >> 16) & 0xffff)
#define TRANSFER_COUNT_SET_HCOUNT(x)	((x) & 0xffff)
#define TRANSFER_COUNT_GET_HCOUNT(x)	((x) & 0xffff)


#define VDCTRL0_ENABLE_PRESENT		(1 << 28)
#define VDCTRL0_VSYNC_ACT_HIGH		(1 << 27)
#define VDCTRL0_HSYNC_ACT_HIGH		(1 << 26)
#define VDCTRL0_DOTCLK_ACT_FALLING	(1 << 25)
#define VDCTRL0_ENABLE_ACT_HIGH		(1 << 24)
#define VDCTRL0_VSYNC_PERIOD_UNIT	(1 << 21)
#define VDCTRL0_VSYNC_PULSE_WIDTH_UNIT	(1 << 20)
#define VDCTRL0_HALF_LINE		(1 << 19)
#define VDCTRL0_HALF_LINE_MODE		(1 << 18)
#define VDCTRL0_SET_VSYNC_PULSE_WIDTH(x) ((x) & 0x3ffff)
#define VDCTRL0_GET_VSYNC_PULSE_WIDTH(x) ((x) & 0x3ffff)

#define VDCTRL2_SET_HSYNC_PERIOD(x)	((x) & 0x3ffff)
#define VDCTRL2_GET_HSYNC_PERIOD(x)	((x) & 0x3ffff)

#define VDCTRL3_MUX_SYNC_SIGNALS	(1 << 29)
#define VDCTRL3_VSYNC_ONLY		(1 << 28)
#define SET_HOR_WAIT_CNT(x)		(((x) & 0xfff) << 16)
#define GET_HOR_WAIT_CNT(x)		(((x) >> 16) & 0xfff)
#define SET_VERT_WAIT_CNT(x)		((x) & 0xffff)
#define GET_VERT_WAIT_CNT(x)		((x) & 0xffff)

#define VDCTRL4_SET_DOTCLK_DLY(x)	(((x) & 0x7) << 29) /* v4 only */
#define VDCTRL4_GET_DOTCLK_DLY(x)	(((x) >> 29) & 0x7) /* v4 only */
#define VDCTRL4_SYNC_SIGNALS_ON		(1 << 18)
#define SET_DOTCLK_H_VALID_DATA_CNT(x)	((x) & 0x3ffff)

#define DEBUG0_HSYNC			(1 < 26)
#define DEBUG0_VSYNC			(1 < 25)

#define MIN_XRES			120
#define MIN_YRES			120

#define RED 0
#define GREEN 1
#define BLUE 2
#define TRANSP 3

#define STMLCDIF_8BIT  1 /** pixel data bus to the display is of 8 bit width */
#define STMLCDIF_16BIT 0 /** pixel data bus to the display is of 16 bit width */
#define STMLCDIF_18BIT 2 /** pixel data bus to the display is of 18 bit width */
#define STMLCDIF_24BIT 3 /** pixel data bus to the display is of 24 bit width */

#define FB_SYNC_OE_LOW_ACT		0x80000000
#define FB_SYNC_CLK_LAT_FALL	0x40000000

enum mxsfb_devtype {
	MXSFB_V3,
	MXSFB_V4,
	MXSFB_V5,
};

/* CPU dependent register offsets */
struct mxsfb_devdata {
	unsigned transfer_count;
	unsigned cur_buf;
	unsigned next_buf;
	unsigned debug0;
	unsigned hs_wdth_mask;
	unsigned hs_wdth_shift;
	unsigned ipversion;
	u32 flags;
};

struct mxsfb_layer;

struct mxsfb_layer_ops {
	void (*enable)(struct mxsfb_layer *ofb);
	void (*disable)(struct mxsfb_layer *ofb);
	void (*setup)(struct mxsfb_layer *ofb);
};

struct mxsfb_layer {
	struct fb_info		*ol_fb;
	int			id;
	int			registered;
	atomic_t		usage;
	int			blank_state;
	uint32_t		global_alpha;

	struct mxsfb_layer_ops	*ops;

	struct device		*dev;
	void __iomem		*video_mem;
	unsigned long		video_mem_phys;
	size_t			video_mem_size;

	struct mxsfb_info	*fbi;
};

#define NAME_LEN	32

struct mxsfb_info {
	struct fb_info *fb_info;
	struct platform_device *pdev;
	struct clk *clk_pix;
	struct clk *clk_axi;
	struct clk *clk_disp_axi;
	bool clk_pix_enabled;
	bool clk_axi_enabled;
	bool clk_disp_axi_enabled;
	void __iomem *base;	/* registers */
	u32 sync;		/* record display timing polarities */
	unsigned allocated_size;
	int enabled;
	unsigned ld_intf_width;
	unsigned dotclk_delay;
	const struct mxsfb_devdata *devdata;
	struct regulator *reg_lcd;
	bool wait4vsync;
	struct completion vsync_complete;
	struct completion flip_complete;
	int cur_blank;
	int restore_blank;
	char disp_dev[NAME_LEN];
	struct mxc_dispdrv_handle *dispdrv;
	int id;
	struct fb_var_screeninfo var;
	struct pm_qos_request pm_qos_req;

	char disp_videomode[NAME_LEN];

#ifdef CONFIG_FB_MXC_OVERLAY
	struct mxsfb_layer overlay;
#endif
};

#define mxsfb_is_v3(host) (host->devdata->ipversion == 3)
#define mxsfb_is_v4(host) (host->devdata->ipversion == 4)
#define mxsfb_is_v5(host) (host->devdata->ipversion == 5)

#define MXSFB_FLAG_NULL		0x0
#define MXSFB_FLAG_BUSFREQ	0x1
#define MXSFB_FLAG_PMQOS	0x2

static const struct mxsfb_devdata mxsfb_devdata[] = {
	[MXSFB_V3] = {
		.transfer_count = LCDC_V3_TRANSFER_COUNT,
		.cur_buf = LCDC_V3_CUR_BUF,
		.next_buf = LCDC_V3_NEXT_BUF,
		.debug0 = LCDC_V3_DEBUG0,
		.hs_wdth_mask = 0xff,
		.hs_wdth_shift = 24,
		.ipversion = 3,
		.flags = MXSFB_FLAG_NULL,
	},
	[MXSFB_V4] = {
		.transfer_count = LCDC_V4_TRANSFER_COUNT,
		.cur_buf = LCDC_V4_CUR_BUF,
		.next_buf = LCDC_V4_NEXT_BUF,
		.debug0 = LCDC_V4_DEBUG0,
		.hs_wdth_mask = 0x3fff,
		.hs_wdth_shift = 18,
		.ipversion = 4,
		.flags = MXSFB_FLAG_BUSFREQ,
	},
	[MXSFB_V5] = {
		.transfer_count = LCDC_V4_TRANSFER_COUNT,
		.cur_buf = LCDC_V4_CUR_BUF,
		.next_buf = LCDC_V4_NEXT_BUF,
		.debug0 = LCDC_V4_DEBUG0,
		.hs_wdth_mask = 0x3fff,
		.hs_wdth_shift = 18,
		.ipversion = 4,
		.flags = MXSFB_FLAG_PMQOS,
	},
};

static int mxsfb_map_videomem(struct fb_info *info);
static int mxsfb_unmap_videomem(struct fb_info *info);
static int mxsfb_set_par(struct fb_info *fb_info);

/* enable lcdif pix clock */
static inline void clk_enable_pix(struct mxsfb_info *host)
{
	if (!host->clk_pix_enabled && (host->clk_pix != NULL)) {
		clk_prepare_enable(host->clk_pix);
		host->clk_pix_enabled = true;
	}
}

/* disable lcdif pix clock */
static inline void clk_disable_pix(struct mxsfb_info *host)
{
	if (host->clk_pix_enabled && (host->clk_pix != NULL)) {
		clk_disable_unprepare(host->clk_pix);
		host->clk_pix_enabled = false;
	}
}

/* enable lcdif axi clock */
static inline void clk_enable_axi(struct mxsfb_info *host)
{
	if (!host->clk_axi_enabled && (host->clk_axi != NULL)) {
		clk_prepare_enable(host->clk_axi);
		host->clk_axi_enabled = true;
	}
}

/* disable lcdif axi clock */
static inline void clk_disable_axi(struct mxsfb_info *host)
{
	if (host->clk_axi_enabled && (host->clk_axi != NULL)) {
		clk_disable_unprepare(host->clk_axi);
		host->clk_axi_enabled = false;
	}
}

/* enable DISP axi clock */
static inline void clk_enable_disp_axi(struct mxsfb_info *host)
{
	if (!host->clk_disp_axi_enabled && (host->clk_disp_axi != NULL)) {
		clk_prepare_enable(host->clk_disp_axi);
		host->clk_disp_axi_enabled = true;
	}
}

/* disable DISP axi clock */
static inline void clk_disable_disp_axi(struct mxsfb_info *host)
{
	if (host->clk_disp_axi_enabled && (host->clk_disp_axi != NULL)) {
		clk_disable_unprepare(host->clk_disp_axi);
		host->clk_disp_axi_enabled = false;
	}
}

/* mask and shift depends on architecture */
static inline u32 set_hsync_pulse_width(struct mxsfb_info *host, unsigned val)
{
	return (val & host->devdata->hs_wdth_mask) <<
		host->devdata->hs_wdth_shift;
}

static inline u32 get_hsync_pulse_width(struct mxsfb_info *host, unsigned val)
{
	return (val >> host->devdata->hs_wdth_shift) &
		host->devdata->hs_wdth_mask;
}

static const struct fb_bitfield def_rgb565[] = {
	[RED] = {
		.offset = 11,
		.length = 5,
	},
	[GREEN] = {
		.offset = 5,
		.length = 6,
	},
	[BLUE] = {
		.offset = 0,
		.length = 5,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

#ifdef CONFIG_FB_MXC_OVERLAY

static const struct fb_bitfield def_argb555[] = {
	[RED] = {
		.offset = 10,
		.length = 5,
	},
	[GREEN] = {
		.offset = 5,
		.length = 5,
	},
	[BLUE] = {
		.offset = 0,
		.length = 5,
	},
	[TRANSP] = {
		.offset = 15,
		.length = 0,
	}
};

static const struct fb_bitfield def_rgb555[] = {
	[RED] = {
		.offset = 10,
		.length = 5,
	},
	[GREEN] = {
		.offset = 5,
		.length = 5,
	},
	[BLUE] = {
		.offset = 0,
		.length = 5,
	},
	[TRANSP] = {
		.offset = 0,
		.length = 0,
	}
};

static const struct fb_bitfield def_argb444[] = {
	[RED] = {
		.offset = 8,
		.length = 4,
	},
	[GREEN] = {
		.offset = 4,
		.length = 4,
	},
	[BLUE] = {
		.offset = 0,
		.length = 4,
	},
	[TRANSP] = {
		.offset = 12,
		.length = 4,
	}
};

static const struct fb_bitfield def_rgb444[] = {
	[RED] = {
		.offset = 8,
		.length = 4,
	},
	[GREEN] = {
		.offset = 4,
		.length = 4,
	},
	[BLUE] = {
		.offset = 0,
		.length = 4,
	},
	[TRANSP] = {
		.offset = 0,
		.length = 0,
	}
};
#endif

static const struct fb_bitfield def_rgb666[] = {
	[RED] = {
		.offset = 16,
		.length = 6,
	},
	[GREEN] = {
		.offset = 8,
		.length = 6,
	},
	[BLUE] = {
		.offset = 0,
		.length = 6,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

static const struct fb_bitfield def_rgb888[] = {
	[RED] = {
		.offset = 16,
		.length = 8,
	},
	[GREEN] = {
		.offset = 8,
		.length = 8,
	},
	[BLUE] = {
		.offset = 0,
		.length = 8,
	},
	[TRANSP] = {	/* no support for transparency */
		.length = 0,
	}
};

static const struct fb_bitfield def_argb32[] = {
	[RED] = {
		.offset = 16,
		.length = 8,
	},
	[GREEN] = {
		.offset = 8,
		.length = 8,
	},
	[BLUE] = {
		.offset = 0,
		.length = 8,
	},
	[TRANSP] = {
		.offset = 24,
		.length = 8,
	}
};

#define bitfield_is_equal(f1, f2)  (!memcmp(&(f1), &(f2), sizeof(f1)))

static inline bool pixfmt_is_equal(struct fb_var_screeninfo *var,
				   const struct fb_bitfield *f)
{
	if (bitfield_is_equal(var->red, f[RED]) &&
	    bitfield_is_equal(var->green, f[GREEN]) &&
	    bitfield_is_equal(var->blue, f[BLUE]))
		return true;

	return false;
}

static inline unsigned chan_to_field(unsigned chan, struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static irqreturn_t mxsfb_irq_handler(int irq, void *dev_id)
{
	struct mxsfb_info *host = dev_id;
	u32 ctrl1, enable, status, acked_status;

	ctrl1 = readl(host->base + LCDC_CTRL1);
	enable = (ctrl1 & CTRL1_IRQ_ENABLE_MASK) >> CTRL1_IRQ_ENABLE_SHIFT;
	status = (ctrl1 & CTRL1_IRQ_STATUS_MASK) >> CTRL1_IRQ_STATUS_SHIFT;
	acked_status = (enable & status) << CTRL1_IRQ_STATUS_SHIFT;

	if ((acked_status & CTRL1_VSYNC_EDGE_IRQ) && host->wait4vsync) {
		writel(CTRL1_VSYNC_EDGE_IRQ,
				host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL1_VSYNC_EDGE_IRQ_EN,
			     host->base + LCDC_CTRL1 + REG_CLR);
		host->wait4vsync = 0;
		complete(&host->vsync_complete);
	}

	if (acked_status & CTRL1_CUR_FRAME_DONE_IRQ) {
		writel(CTRL1_CUR_FRAME_DONE_IRQ,
				host->base + LCDC_CTRL1 + REG_CLR);
		writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
			     host->base + LCDC_CTRL1 + REG_CLR);
		complete(&host->flip_complete);
	}

	if (acked_status & CTRL1_UNDERFLOW_IRQ)
		writel(CTRL1_UNDERFLOW_IRQ, host->base + LCDC_CTRL1 + REG_CLR);

	if (acked_status & CTRL1_OVERFLOW_IRQ)
		writel(CTRL1_OVERFLOW_IRQ, host->base + LCDC_CTRL1 + REG_CLR);

	return IRQ_HANDLED;
}

static int mxsfb_check_var(struct fb_var_screeninfo *var,
		struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	const struct fb_bitfield *rgb = NULL;

	if (var->xres < MIN_XRES)
		var->xres = MIN_XRES;
	if (var->yres < MIN_YRES)
		var->yres = MIN_YRES;

	if (var->xres_virtual > var->xres) {
		dev_dbg(fb_info->device, "stride not supported\n");
		return -EINVAL;
	}

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 16))
		var->bits_per_pixel = 32;

	switch (var->bits_per_pixel) {
	case 16:
		/* always expect RGB 565 */
		rgb = def_rgb565;
		break;
	case 32:
		switch (host->ld_intf_width) {
		case STMLCDIF_8BIT:
			pr_debug("Unsupported LCD bus width mapping\n");
			return -EINVAL;
		case STMLCDIF_16BIT:
			/* 24 bit to 18 bit mapping */
			rgb = def_rgb666;
			break;
		case STMLCDIF_18BIT:
			if (pixfmt_is_equal(var, def_rgb666))
				/* 24 bit to 18 bit mapping */
				rgb = def_rgb666;
			else
				rgb = def_rgb888;
			break;
		case STMLCDIF_24BIT:
			/* real 24 bit */
			rgb = def_rgb888;
			break;
		default:
			/*
			 * 32-bit output is possible through I/O muxing, if this
			 * option is available on chip. Currently not
			 * implemented.
			 */
			pr_debug("Currently unsupported output colour depth: %u\n",
				 host->ld_intf_width);
			return -EINVAL;
		}
		break;
	default:
		pr_debug("Unsupported colour depth: %u\n", var->bits_per_pixel);
		return -EINVAL;
	}

	/*
	 * Copy the RGB parameters for this display
	 * from the machine specific parameters.
	 */
	var->red    = rgb[RED];
	var->green  = rgb[GREEN];
	var->blue   = rgb[BLUE];
	var->transp = rgb[TRANSP];

	return 0;
}

static void mxsfb_enable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	u32 reg;
	int ret;
#ifdef CONFIG_FB_IMX64_DEBUG
	static int pix_enable;
#endif

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	if (host->dispdrv && host->dispdrv->drv->setup) {
		ret = host->dispdrv->drv->setup(host->dispdrv, fb_info);
		if (ret < 0) {
			dev_err(&host->pdev->dev, "failed to setup"
				"dispdrv:%s\n", host->dispdrv->drv->name);
			return;
		}
		host->sync = fb_info->var.sync;
	}

	if (host->reg_lcd) {
		ret = regulator_enable(host->reg_lcd);
		if (ret) {
			dev_err(&host->pdev->dev,
				"lcd regulator enable failed:	%d\n", ret);
			return;
		}
	}

	if (host->dispdrv && host->dispdrv->drv->enable) {
		ret = host->dispdrv->drv->enable(host->dispdrv, fb_info);
		if (ret < 0)
			dev_err(&host->pdev->dev, "failed to enable "
				"dispdrv:%s\n", host->dispdrv->drv->name);
	}

#ifdef CONFIG_FB_IMX64_DEBUG
	if (unlikely(!pix_enable)) {
		/* the pixel clock should be disabled before
		 * trying to set its clock rate successfully.
		 */
#else
		clk_disable_pix(host);
#endif
		ret = clk_set_rate(host->clk_pix,
				PICOS2KHZ(fb_info->var.pixclock) * 1000U);
		if (ret) {
			dev_err(&host->pdev->dev,
				"lcd pixel rate set failed: %d\n", ret);

			if (host->reg_lcd) {
				ret = regulator_disable(host->reg_lcd);
				if (ret)
					dev_err(&host->pdev->dev,
						"lcd regulator disable failed: %d\n",
						ret);
			}
			return;
		}
		clk_enable_pix(host);
#ifdef CONFIG_FB_IMX64_DEBUG
		pix_enable++;
	}
#endif

	writel(CTRL2_OUTSTANDING_REQS__REQ_16,
		host->base + LCDC_V4_CTRL2 + REG_SET);

	/* if it was disabled, re-enable the mode again */
	writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_SET);

	/* enable the SYNC signals first, then the DMA engine */
	reg = readl(host->base + LCDC_VDCTRL4);
	reg |= VDCTRL4_SYNC_SIGNALS_ON;
	writel(reg, host->base + LCDC_VDCTRL4);

	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_SET);
	writel(CTRL_RUN, host->base + LCDC_CTRL + REG_SET);

	/* Recovery on underflow */
	writel(CTRL1_RECOVERY_ON_UNDERFLOW, host->base + LCDC_CTRL1 + REG_SET);

	host->enabled = 1;

}

static void mxsfb_disable_controller(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	unsigned loop;
	u32 reg;
	int ret;

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	writel(CTRL_RUN, host->base + LCDC_CTRL + REG_CLR);

	if (host->dispdrv && host->dispdrv->drv->disable)
		host->dispdrv->drv->disable(host->dispdrv, fb_info);

	/*
	 * Even if we disable the controller here, it will still continue
	 * until its FIFOs are running out of data
	 */
	writel(CTRL_DOTCLK_MODE, host->base + LCDC_CTRL + REG_CLR);

	loop = 1000;
	while (loop) {
		reg = readl(host->base + LCDC_CTRL);
		if (!(reg & CTRL_RUN))
			break;
		loop--;
	}

	writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);

	reg = readl(host->base + LCDC_VDCTRL4);
	writel(reg & ~VDCTRL4_SYNC_SIGNALS_ON, host->base + LCDC_VDCTRL4);

	host->enabled = 0;

	if (host->reg_lcd) {
		ret = regulator_disable(host->reg_lcd);
		if (ret)
			dev_err(&host->pdev->dev,
				"lcd regulator disable failed: %d\n", ret);
	}
}

/**
   This function compare the fb parameter see whether it was different
   parameter for hardware, if it was different parameter, the hardware
   will reinitialize. All will compared except x/y offset.
 */
static bool mxsfb_par_equal(struct fb_info *fbi, struct mxsfb_info *host)
{
	/* Here we set the xoffset, yoffset to zero, and compare two
	 * var see have different or not. */
	struct fb_var_screeninfo oldvar = host->var;
	struct fb_var_screeninfo newvar = fbi->var;

	if ((fbi->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW &&
	    fbi->var.activate & FB_ACTIVATE_FORCE)
		return false;

	oldvar.xoffset = newvar.xoffset = 0;
	oldvar.yoffset = newvar.yoffset = 0;

	return memcmp(&oldvar, &newvar, sizeof(struct fb_var_screeninfo)) == 0;
}

static int mxsfb_set_par(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	u32 ctrl, vdctrl0, vdctrl4;
	int line_size, fb_size;
	int reenable = 0;
	static u32 equal_bypass;

#ifdef CONFIG_FB_IMX64_DEBUG
	static int time;

	if (time == 1)
		return 0;
	time++;
#endif

	if (likely(equal_bypass > 1)) {
		/* If parameter no change, don't reconfigure. */
		if (mxsfb_par_equal(fb_info, host))
			return 0;
	} else
		equal_bypass++;

	dev_dbg(&host->pdev->dev, "%s\n", __func__);

	/* If fb is in blank mode, it is
	 * unnecessary to really set par here.
	 * It can be delayed when unblank fb
	 */
	if (host->cur_blank != FB_BLANK_UNBLANK)
		return 0;

	line_size =  fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.line_length = line_size;
	fb_size = fb_info->var.yres_virtual * line_size;

	if (fb_size > fb_info->fix.smem_len) {
		dev_err(&host->pdev->dev, "exceeds the fb buffer size limit!\n");
		return -ENOMEM;
	}

	/*
	 * It seems, you can't re-program the controller if it is still running.
	 * This may lead into shifted pictures (FIFO issue?).
	 * So, first stop the controller and drain its FIFOs
	 */
	if (host->enabled) {
		reenable = 1;
		mxsfb_disable_controller(fb_info);
	}

	/* clear the FIFOs */
	writel(CTRL1_FIFO_CLEAR, host->base + LCDC_CTRL1 + REG_SET);

	ctrl = CTRL_BYPASS_COUNT | CTRL_MASTER |
		CTRL_SET_BUS_WIDTH(host->ld_intf_width);

	switch (fb_info->var.bits_per_pixel) {
	case 16:
		dev_dbg(&host->pdev->dev, "Setting up RGB565 mode\n");
		ctrl |= CTRL_SET_WORD_LENGTH(0);
		writel(CTRL1_SET_BYTE_PACKAGING(0xf), host->base + LCDC_CTRL1);
		break;
	case 32:
		dev_dbg(&host->pdev->dev, "Setting up RGB888/666 mode\n");
		ctrl |= CTRL_SET_WORD_LENGTH(3);
		switch (host->ld_intf_width) {
		case STMLCDIF_8BIT:
			dev_dbg(&host->pdev->dev,
					"Unsupported LCD bus width mapping\n");
			return -EINVAL;
		case STMLCDIF_16BIT:
			/* 24 bit to 18 bit mapping */
			ctrl |= CTRL_DF24; /* ignore the upper 2 bits in
					    *  each colour component
					    */
			break;
		case STMLCDIF_18BIT:
			if (pixfmt_is_equal(&fb_info->var, def_rgb666))
				/* 24 bit to 18 bit mapping */
				ctrl |= CTRL_DF24; /* ignore the upper 2 bits in
						    *  each colour component
						    */
			break;
		case STMLCDIF_24BIT:
			/* real 24 bit */
			break;
		}
		/* do not use packed pixels = one pixel per word instead */
		writel(CTRL1_SET_BYTE_PACKAGING(0x7), host->base + LCDC_CTRL1);
		break;
	default:
		dev_dbg(&host->pdev->dev, "Unhandled color depth of %u\n",
				fb_info->var.bits_per_pixel);
		return -EINVAL;
	}

	writel(ctrl, host->base + LCDC_CTRL);

	writel(TRANSFER_COUNT_SET_VCOUNT(fb_info->var.yres) |
			TRANSFER_COUNT_SET_HCOUNT(fb_info->var.xres),
			host->base + host->devdata->transfer_count);

	vdctrl0 = VDCTRL0_ENABLE_PRESENT |	/* always in DOTCLOCK mode */
		VDCTRL0_VSYNC_PERIOD_UNIT |
		VDCTRL0_VSYNC_PULSE_WIDTH_UNIT |
		VDCTRL0_SET_VSYNC_PULSE_WIDTH(fb_info->var.vsync_len);
	/* use the saved sync to avoid wrong sync information */
	if (host->sync & FB_SYNC_HOR_HIGH_ACT)
		vdctrl0 |= VDCTRL0_HSYNC_ACT_HIGH;
	if (host->sync & FB_SYNC_VERT_HIGH_ACT)
		vdctrl0 |= VDCTRL0_VSYNC_ACT_HIGH;
#ifndef CONFIG_FB_IMX64_DEBUG
	if (!(host->sync & FB_SYNC_OE_LOW_ACT))
		vdctrl0 |= VDCTRL0_ENABLE_ACT_HIGH;
#endif
	if (host->sync & FB_SYNC_CLK_LAT_FALL)
		vdctrl0 |= VDCTRL0_DOTCLK_ACT_FALLING;

	writel(vdctrl0, host->base + LCDC_VDCTRL0);

	/* frame length in lines */
	writel(fb_info->var.upper_margin + fb_info->var.vsync_len +
		fb_info->var.lower_margin + fb_info->var.yres,
		host->base + LCDC_VDCTRL1);

	/* line length in units of clocks or pixels */
	writel(set_hsync_pulse_width(host, fb_info->var.hsync_len) |
		VDCTRL2_SET_HSYNC_PERIOD(fb_info->var.left_margin +
		fb_info->var.hsync_len + fb_info->var.right_margin +
		fb_info->var.xres),
		host->base + LCDC_VDCTRL2);

	writel(SET_HOR_WAIT_CNT(fb_info->var.left_margin +
		fb_info->var.hsync_len) |
		SET_VERT_WAIT_CNT(fb_info->var.upper_margin +
			fb_info->var.vsync_len),
		host->base + LCDC_VDCTRL3);

	vdctrl4 = SET_DOTCLK_H_VALID_DATA_CNT(fb_info->var.xres);
	if (mxsfb_is_v4(host))
		vdctrl4 |= VDCTRL4_SET_DOTCLK_DLY(host->dotclk_delay);
	writel(vdctrl4, host->base + LCDC_VDCTRL4);

	writel(fb_info->fix.smem_start +
			fb_info->fix.line_length * fb_info->var.yoffset,
			host->base + host->devdata->next_buf);

	if (reenable)
		mxsfb_enable_controller(fb_info);

	/* Clear activate as not Reconfiguring framebuffer again */
	if ((fb_info->var.activate & FB_ACTIVATE_FORCE) &&
		(fb_info->var.activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		fb_info->var.activate = FB_ACTIVATE_NOW;

	host->var = fb_info->var;
	return 0;
}

static int mxsfb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
		u_int transp, struct fb_info *fb_info)
{
	unsigned int val;
	int ret = -EINVAL;

	/*
	 * If greyscale is true, then we convert the RGB value
	 * to greyscale no matter what visual we are using.
	 */
	if (fb_info->var.grayscale)
		red = green = blue = (19595 * red + 38470 * green +
					7471 * blue) >> 16;

	switch (fb_info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
		/*
		 * 12 or 16-bit True Colour.  We encode the RGB value
		 * according to the RGB bitfield information.
		 */
		if (regno < 16) {
			u32 *pal = fb_info->pseudo_palette;

			val  = chan_to_field(red, &fb_info->var.red);
			val |= chan_to_field(green, &fb_info->var.green);
			val |= chan_to_field(blue, &fb_info->var.blue);

			pal[regno] = val;
			ret = 0;
		}
		break;

	case FB_VISUAL_STATIC_PSEUDOCOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		break;
	}

	return ret;
}

static int mxsfb_wait_for_vsync(struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;
	int ret = 0;

	if (host->cur_blank != FB_BLANK_UNBLANK) {
		dev_err(fb_info->device, "can't wait for VSYNC when fb "
			"is blank\n");
		return -EINVAL;
	}

	init_completion(&host->vsync_complete);

	host->wait4vsync = 1;
	writel(CTRL1_VSYNC_EDGE_IRQ_EN,
		host->base + LCDC_CTRL1 + REG_SET);
	ret = wait_for_completion_interruptible_timeout(
				&host->vsync_complete, 1 * HZ);
	if (ret == 0) {
		dev_err(fb_info->device,
			"mxs wait for vsync timeout\n");
		host->wait4vsync = 0;
		ret = -ETIME;
	} else if (ret > 0) {
		ret = 0;
	}
	return ret;
}

static int mxsfb_ioctl(struct fb_info *fb_info, unsigned int cmd,
			unsigned long arg)
{
	int ret = -EINVAL;

	switch (cmd) {
	case MXCFB_WAIT_FOR_VSYNC:
		ret = mxsfb_wait_for_vsync(fb_info);
		break;
	default:
		break;
	}
	return ret;
}

static int mxsfb_blank(int blank, struct fb_info *fb_info)
{
	struct mxsfb_info *host = fb_info->par;

#ifdef CONFIG_FB_IMX64_DEBUG
	return 0;
#endif

	host->cur_blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (host->enabled) {
			mxsfb_disable_controller(fb_info);
			pm_runtime_put_sync_suspend(&host->pdev->dev);
		}

		clk_disable_disp_axi(host);
		clk_disable_axi(host);
		clk_disable_pix(host);
		break;

	case FB_BLANK_UNBLANK:
		fb_info->var.activate = (fb_info->var.activate & ~FB_ACTIVATE_MASK) |
				FB_ACTIVATE_NOW | FB_ACTIVATE_FORCE;

		clk_enable_pix(host);
		clk_enable_axi(host);
		clk_enable_disp_axi(host);

		if (!host->enabled) {
			pm_runtime_get_sync(&host->pdev->dev);

			writel(0, host->base + LCDC_CTRL);
			mxsfb_set_par(host->fb_info);
			mxsfb_enable_controller(fb_info);
		}
		break;
	}
	return 0;
}

static int mxsfb_pan_display(struct fb_var_screeninfo *var,
		struct fb_info *fb_info)
{
	int ret = 0;
	struct mxsfb_info *host = fb_info->par;
	unsigned offset;

	if (host->cur_blank != FB_BLANK_UNBLANK) {
		dev_dbg(fb_info->device, "can't do pan display when fb "
			"is blank\n");
		return -EINVAL;
	}

	if (var->xoffset > 0) {
		dev_dbg(fb_info->device, "x panning not supported\n");
		return -EINVAL;
	}

	if ((var->yoffset + var->yres > var->yres_virtual)) {
		dev_err(fb_info->device, "y panning exceeds\n");
		return -EINVAL;
	}

	init_completion(&host->flip_complete);

	offset = fb_info->fix.line_length * var->yoffset;

	/* update on next VSYNC */
	writel(fb_info->fix.smem_start + offset,
			host->base + host->devdata->next_buf);

	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
		host->base + LCDC_CTRL1 + REG_SET);

	ret = wait_for_completion_timeout(&host->flip_complete, HZ / 2);
	if (!ret) {
		dev_err(fb_info->device,
			"mxs wait for pan flip timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static int mxsfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;

	if (offset < info->fix.smem_len) {
		/* mapping framebuffer memory */
		len = info->fix.smem_len - offset;
		vma->vm_pgoff = (info->fix.smem_start + offset) >> PAGE_SHIFT;
	} else
		return -EINVAL;

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(info->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

static struct fb_ops mxsfb_ops = {
	.owner = THIS_MODULE,
	.fb_check_var = mxsfb_check_var,
	.fb_set_par = mxsfb_set_par,
	.fb_setcolreg = mxsfb_setcolreg,
	.fb_ioctl = mxsfb_ioctl,
	.fb_blank = mxsfb_blank,
	.fb_pan_display = mxsfb_pan_display,
	.fb_mmap = mxsfb_mmap,
	.fb_fillrect = cfb_fillrect,
	.fb_copyarea = cfb_copyarea,
	.fb_imageblit = cfb_imageblit,
};

static int mxsfb_restore_mode(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	unsigned line_count;
	unsigned period;
	unsigned long pa, fbsize;
	int bits_per_pixel, ofs;
	u32 transfer_count, vdctrl0, vdctrl2, vdctrl3, vdctrl4, ctrl;
	struct fb_videomode vmode;

	clk_enable_axi(host);
	clk_enable_disp_axi(host);

#ifndef CONFIG_FB_IMX64_DEBUG
	/* Enable pixel clock earlier since in 7D
	 * the lcdif registers should be accessed
	 * when the pixel clock is enabled, otherwise
	 * the bus will be hang.
	 */
	clk_enable_pix(host);
#endif

	/* Only restore the mode when the controller is running */
	ctrl = readl(host->base + LCDC_CTRL);
	if (!(ctrl & CTRL_RUN))
		return -EINVAL;

	memset(&vmode, 0, sizeof(vmode));

	vdctrl0 = readl(host->base + LCDC_VDCTRL0);
	vdctrl2 = readl(host->base + LCDC_VDCTRL2);
	vdctrl3 = readl(host->base + LCDC_VDCTRL3);
	vdctrl4 = readl(host->base + LCDC_VDCTRL4);

	transfer_count = readl(host->base + host->devdata->transfer_count);

	vmode.xres = TRANSFER_COUNT_GET_HCOUNT(transfer_count);
	vmode.yres = TRANSFER_COUNT_GET_VCOUNT(transfer_count);

	switch (CTRL_GET_WORD_LENGTH(ctrl)) {
	case 0:
		bits_per_pixel = 16;
		break;
	case 3:
		bits_per_pixel = 32;
		break;
	case 1:
	default:
		return -EINVAL;
	}

	fb_info->var.bits_per_pixel = bits_per_pixel;

	vmode.pixclock = clk_get_rate(host->clk_pix) / 1000U;
	if (vmode.pixclock)
		vmode.pixclock = KHZ2PICOS(vmode.pixclock);
	vmode.hsync_len = get_hsync_pulse_width(host, vdctrl2);
	vmode.left_margin = GET_HOR_WAIT_CNT(vdctrl3) - vmode.hsync_len;
	vmode.right_margin = VDCTRL2_GET_HSYNC_PERIOD(vdctrl2) - vmode.hsync_len -
		vmode.left_margin - vmode.xres;
	vmode.vsync_len = VDCTRL0_GET_VSYNC_PULSE_WIDTH(vdctrl0);
	period = readl(host->base + LCDC_VDCTRL1);
	vmode.upper_margin = GET_VERT_WAIT_CNT(vdctrl3) - vmode.vsync_len;
	vmode.lower_margin = period - vmode.vsync_len - vmode.upper_margin - vmode.yres;

	vmode.vmode = FB_VMODE_NONINTERLACED;

	vmode.sync = 0;
	if (vdctrl0 & VDCTRL0_HSYNC_ACT_HIGH)
		vmode.sync |= FB_SYNC_HOR_HIGH_ACT;
	if (vdctrl0 & VDCTRL0_VSYNC_ACT_HIGH)
		vmode.sync |= FB_SYNC_VERT_HIGH_ACT;

	pr_debug("Reconstructed video mode:\n");
	pr_debug("%dx%d, hsync: %u left: %u, right: %u, vsync: %u, upper: %u, lower: %u\n",
			vmode.xres, vmode.yres,
			vmode.hsync_len, vmode.left_margin, vmode.right_margin,
			vmode.vsync_len, vmode.upper_margin, vmode.lower_margin);
	pr_debug("pixclk: %ldkHz\n", PICOS2KHZ(vmode.pixclock));

	fb_add_videomode(&vmode, &fb_info->modelist);

	host->ld_intf_width = CTRL_GET_BUS_WIDTH(ctrl);
	host->dotclk_delay = VDCTRL4_GET_DOTCLK_DLY(vdctrl4);

	fb_info->fix.line_length = vmode.xres * (bits_per_pixel >> 3);

	pa = readl(host->base + host->devdata->cur_buf);
	fbsize = fb_info->fix.line_length * vmode.yres;
	if (pa < fb_info->fix.smem_start)
		return -EINVAL;
	if (pa + fbsize > fb_info->fix.smem_start + fb_info->fix.smem_len)
		return -EINVAL;
	ofs = pa - fb_info->fix.smem_start;
	if (ofs) {
		memmove(fb_info->screen_base, fb_info->screen_base + ofs, fbsize);
		writel(fb_info->fix.smem_start, host->base + host->devdata->next_buf);
	}

	line_count = fb_info->fix.smem_len / fb_info->fix.line_length;
	fb_info->fix.ypanstep = 1;
	fb_info->fix.ywrapstep = 1;

	host->enabled = 1;

	return 0;
}

static int mxsfb_init_fbinfo_dt(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct device *dev = &host->pdev->dev;
	struct device_node *np = host->pdev->dev.of_node;
	struct device_node *display_np;
	struct device_node *timings_np;
	struct display_timings *timings = NULL;
	const char *disp_dev, *disp_videomode;
	u32 width;
	int i;
	int ret = 0;

	host->id = of_alias_get_id(np, "lcdif");

	display_np = of_parse_phandle(np, "display", 0);
	if (!display_np) {
		dev_err(dev, "failed to find display phandle\n");
		return -ENOENT;
	}

	ret = of_property_read_u32(display_np, "bus-width", &width);
	if (ret < 0) {
		dev_err(dev, "failed to get property bus-width\n");
		goto put_display_node;
	}

	switch (width) {
	case 8:
		host->ld_intf_width = STMLCDIF_8BIT;
		break;
	case 16:
		host->ld_intf_width = STMLCDIF_16BIT;
		break;
	case 18:
		host->ld_intf_width = STMLCDIF_18BIT;
		break;
	case 24:
		host->ld_intf_width = STMLCDIF_24BIT;
		break;
	default:
		dev_err(dev, "invalid bus-width value\n");
		ret = -EINVAL;
		goto put_display_node;
	}

	ret = of_property_read_u32(display_np, "bits-per-pixel",
				   &var->bits_per_pixel);
	if (ret < 0) {
		dev_err(dev, "failed to get property bits-per-pixel\n");
		goto put_display_node;
	}

	ret = of_property_read_string(np, "disp-dev", &disp_dev);
	if (!ret) {
		memcpy(host->disp_dev, disp_dev, strlen(disp_dev));

		if (!of_property_read_string(np, "disp-videomode",
					    &disp_videomode)) {
			memcpy(host->disp_videomode, disp_videomode,
			       strlen(disp_videomode));
		}

		/* Timing is from encoder driver */
		goto put_display_node;
	}

	timings = of_get_display_timings(display_np);
	if (!timings) {
		dev_err(dev, "failed to get display timings\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	timings_np = of_find_node_by_name(display_np,
					  "display-timings");
	if (!timings_np) {
		dev_err(dev, "failed to find display-timings node\n");
		ret = -ENOENT;
		goto put_display_node;
	}

	for (i = 0; i < of_get_child_count(timings_np); i++) {
		struct videomode vm;
		struct fb_videomode fb_vm;

		ret = videomode_from_timings(timings, &vm, i);
		if (ret < 0)
			goto put_timings_node;
		ret = fb_videomode_from_videomode(&vm, &fb_vm);
		if (ret < 0)
			goto put_timings_node;

		if (!(vm.flags & DISPLAY_FLAGS_DE_HIGH))
			fb_vm.sync |= FB_SYNC_OE_LOW_ACT;
		if (vm.flags & DISPLAY_FLAGS_PIXDATA_NEGEDGE)
			fb_vm.sync |= FB_SYNC_CLK_LAT_FALL;
		fb_add_videomode(&fb_vm, &fb_info->modelist);
	}

put_timings_node:
	of_node_put(timings_np);
put_display_node:
	if (timings)
		kfree(timings);
	of_node_put(display_np);
	return ret;
}

static int mxsfb_init_fbinfo(struct mxsfb_info *host)
{
	int ret;
	struct fb_info *fb_info = host->fb_info;
	struct fb_var_screeninfo *var = &fb_info->var;
	struct fb_modelist *modelist;

	fb_info->fbops = &mxsfb_ops;
	fb_info->flags = FBINFO_FLAG_DEFAULT | FBINFO_READS_FAST;
	fb_info->fix.type = FB_TYPE_PACKED_PIXELS;
	fb_info->fix.ypanstep = 1;
	fb_info->fix.ywrapstep = 1;
	fb_info->fix.visual = FB_VISUAL_TRUECOLOR,
	fb_info->fix.accel = FB_ACCEL_NONE;

	ret = mxsfb_init_fbinfo_dt(host);
	if (ret)
		return ret;

	if (host->id < 0)
		sprintf(fb_info->fix.id, "mxs-lcdif");
	else
		sprintf(fb_info->fix.id, "mxs-lcdif%d", host->id);

	if (!list_empty(&fb_info->modelist)) {
		/* first video mode in the modelist as default video mode  */
		modelist = list_first_entry(&fb_info->modelist,
				struct fb_modelist, list);
		fb_videomode_to_var(var, &modelist->mode);
	}
	/* save the sync value getting from dtb */
	host->sync = fb_info->var.sync;

	var->nonstd = 0;
	var->activate = FB_ACTIVATE_NOW;
	var->accel_flags = 0;
	var->vmode = FB_VMODE_NONINTERLACED;

	/* init the color fields */
	mxsfb_check_var(var, fb_info);

	fb_info->fix.line_length =
		fb_info->var.xres * (fb_info->var.bits_per_pixel >> 3);
	fb_info->fix.smem_len = SZ_32M;

	/* Memory allocation for framebuffer */
	if (mxsfb_map_videomem(fb_info) < 0)
		return -ENOMEM;

	if (mxsfb_restore_mode(host))
		memset((char *)fb_info->screen_base, 0, fb_info->fix.smem_len);

	return 0;
}

static int mxsfb_dispdrv_init(struct platform_device *pdev,
			      struct fb_info *fbi)
{
	struct mxsfb_info *host = fbi->par;
	struct mxc_dispdrv_setting setting;
	struct device *dev = &pdev->dev;
	char disp_dev[32];

	if (!strlen(host->disp_dev))
		return 0;

	memset(&setting, 0x0, sizeof(setting));
	setting.fbi = fbi;
	memcpy(disp_dev, host->disp_dev, strlen(host->disp_dev));
	disp_dev[strlen(host->disp_dev)] = '\0';

	/* Use videomode name from dtb, if any given */
	if (host->disp_videomode[0]) {
		setting.dft_mode_str = kmalloc(NAME_LEN, GFP_KERNEL);
		if (setting.dft_mode_str) {
			memset(setting.dft_mode_str, 0x0, NAME_LEN);
			memcpy(setting.dft_mode_str, host->disp_videomode,
			       strlen(host->disp_videomode));
		}
	}

	host->dispdrv = mxc_dispdrv_gethandle(disp_dev, &setting);

	kfree(setting.dft_mode_str);

	if (IS_ERR(host->dispdrv))
		return -EPROBE_DEFER;
	else
		dev_info(dev, "registered mxc display driver %s\n",
			 disp_dev);

	return 0;
}

static void mxsfb_free_videomem(struct mxsfb_info *host)
{
	struct fb_info *fb_info = host->fb_info;

	mxsfb_unmap_videomem(fb_info);
}

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxsfb_map_videomem(struct fb_info *fbi)
{
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_wc(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA | GFP_KERNEL);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mxsfb_unmap_videomem(struct fb_info *fbi)
{
	dma_free_wc(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

static const struct platform_device_id mxsfb_devtype[] = {
	{
		.name = "imx23-fb",
		.driver_data = MXSFB_V3,
	}, {
		.name = "imx28-fb",
		.driver_data = MXSFB_V4,
	}, {
		.name = "imx7ulp-fb",
		.driver_data = MXSFB_V5,
	}, {
		/* sentinel */
	}
};
MODULE_DEVICE_TABLE(platform, mxsfb_devtype);

static const struct of_device_id mxsfb_dt_ids[] = {
	{ .compatible = "fsl,imx23-lcdif", .data = &mxsfb_devtype[0], },
	{ .compatible = "fsl,imx28-lcdif", .data = &mxsfb_devtype[1], },
	{ .compatible = "fsl,imx7ulp-lcdif", .data = &mxsfb_devtype[2], },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxsfb_dt_ids);

#ifdef CONFIG_FB_MXC_OVERLAY
static int overlay_fmt_support(uint32_t fmt)
{
	switch (fmt) {
	case V4L2_PIX_FMT_ARGB32:
	case V4L2_PIX_FMT_XRGB32:
		return 32;
	case V4L2_PIX_FMT_ARGB555:
	case V4L2_PIX_FMT_ARGB444:
	case V4L2_PIX_FMT_XRGB555:
	case V4L2_PIX_FMT_RGB444:
	case V4L2_PIX_FMT_RGB565:
		return 16;
	default:
		return -EINVAL;
	}
}

/* alpha mode */
#define ALPHA_CTRL_EMBEDDED	0x0
#define ALPHA_CTRL_OVERRIDE	0x1
#define ALPHA_CTRL_MULTIPLY	0x2
#define ALPHA_CTRL_ROPS		0x3

static void overlayfb_enable(struct mxsfb_layer *ofb)
{
	struct mxsfb_info *fbi = ofb->fbi;

	if (fbi->cur_blank == FB_BLANK_UNBLANK) {
		mxsfb_disable_controller(fbi->fb_info);
		writel(CTRL1_FIFO_CLEAR, fbi->base + LCDC_CTRL1 + REG_SET);
	}

	writel(0x1, fbi->base + LCDC_AS_CTRL + REG_SET);

	if (fbi->cur_blank == FB_BLANK_UNBLANK) {
		writel(CTRL1_FIFO_CLEAR, fbi->base + LCDC_CTRL1 + REG_CLR);
		mxsfb_enable_controller(fbi->fb_info);
	}
}

static void overlayfb_disable(struct mxsfb_layer *ofb)
{
	struct mxsfb_info *fbi = ofb->fbi;

	writel(0x1, fbi->base + LCDC_AS_CTRL + REG_CLR);
}

static void overlayfb_setup(struct mxsfb_layer *ofb)
{
	uint32_t as_next_buf, as_ctrl = 0;
	uint8_t format, alpha_ctrl, global_alpha_en = 0;
	struct mxsfb_info *fbi = ofb->fbi;
	struct fb_var_screeninfo *var = &ofb->ol_fb->var;

	/* set fb1 framebuffer address */
	as_next_buf = ofb->video_mem_phys;
	writel(as_next_buf, fbi->base + LCDC_AS_NEXT_BUF);

	/* clear the LCDC_AS_CTRL */
	writel(0x0, fbi->base + LCDC_AS_CTRL);

	switch (var->grayscale) {
	case 0: /* color */
		switch (var->bits_per_pixel) {
		case 16: /* RGB565 */
			format = 0xE;
			global_alpha_en = 1;
			break;
		case 32: /* ARGB8888 */
			format = 0x0;
			global_alpha_en = 1;
			break;
		default:
			return;
		}
		break;
	case 1: /* grayscale */
		return;
	default:
		switch (var->grayscale) {
		case V4L2_PIX_FMT_ARGB32:
			format = 0x0;
			break;
		case V4L2_PIX_FMT_XRGB32:
			format = 0x4;
			global_alpha_en = 1;
			break;
		case V4L2_PIX_FMT_ARGB555:
			format = 0x8;
			break;
		case V4L2_PIX_FMT_ARGB444:
			format = 0x9;
			break;
		case V4L2_PIX_FMT_RGB555:
			format = 0xC;
			global_alpha_en = 1;
			break;
		case V4L2_PIX_FMT_RGB444:
			format = 0xD;
			global_alpha_en = 1;
			break;
		case V4L2_PIX_FMT_RGB565:
			format = 0xE;
			global_alpha_en = 1;
			break;
		default:
			return;
		}
		break;
	}
	as_ctrl |= ((format & 0xf) << 4);

	alpha_ctrl = global_alpha_en ? ALPHA_CTRL_OVERRIDE :
				       ALPHA_CTRL_EMBEDDED;
	as_ctrl |= ((alpha_ctrl & 0x3) << 1);
	if (global_alpha_en)
		as_ctrl |= ((ofb->global_alpha & 0xff) << 8);

	writel(as_ctrl, fbi->base + LCDC_AS_CTRL);
}

static struct mxsfb_layer_ops ofb_ops = {
	.enable		= overlayfb_enable,
	.disable	= overlayfb_disable,
	.setup		= overlayfb_setup,
};

static int overlayfb_open(struct fb_info *info, int user)
{
	struct mxsfb_layer *ofb = (struct mxsfb_layer *)info->par;
	struct mxsfb_info  *fbi = ofb->fbi;

	if (atomic_inc_return(&ofb->usage) == 1) {
		ofb->ol_fb->var.xres		= fbi->fb_info->var.xres;
		ofb->ol_fb->var.yres		= fbi->fb_info->var.yres;
		ofb->ol_fb->var.xres_virtual	= fbi->fb_info->var.xres_virtual;
		ofb->ol_fb->var.yres_virtual	= fbi->fb_info->var.yres;
		ofb->ol_fb->var.bits_per_pixel	= fbi->fb_info->var.bits_per_pixel;
		ofb->ol_fb->var.vmode		= FB_VMODE_NONINTERLACED;
	}

	return 0;
}

static int overlayfb_release(struct fb_info *info, int user)
{
	struct mxsfb_layer *ofb = (struct mxsfb_layer *)info->par;

	BUG_ON(!atomic_read(&ofb->usage));

	if (atomic_dec_return(&ofb->usage) == 0) {
		if (ofb->blank_state == FB_BLANK_UNBLANK)
			ofb->ops->disable(ofb);

		ofb->blank_state = -1;
	}

	return 0;
}

static void fill_fmt_bitfields(struct fb_var_screeninfo *var,
			       const struct fb_bitfield *color)
{
	var->red    = color[RED];
	var->green  = color[GREEN];
	var->blue   = color[BLUE];
	var->transp = color[TRANSP];
}

static int overlayfb_check_var(struct fb_var_screeninfo *var,
			       struct fb_info *info)
{
	int bpp;
	struct mxsfb_layer *ofb = (struct mxsfb_layer *)info->par;
	struct mxsfb_info *fbi  = ofb->fbi;
	const struct fb_bitfield *rgb = NULL;

	/* lcdif doesn't support different bpp of AS and PS */
	if (var->bits_per_pixel != fbi->fb_info->var.bits_per_pixel)
		return -EINVAL;

	/* overlay width & should be equal to fb0 */
	if ((var->xres != fbi->fb_info->var.xres) ||
	    (var->yres != fbi->fb_info->var.yres))
		return -EINVAL;

	if ((var->xres > 2048) || (var->yres > 2048))
		return -EINVAL;

	if (var->xres_virtual > var->xres)
		return -EINVAL;

	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;
	if (var->yres_virtual < var->yres)
		var->yres_virtual = var->yres;

	switch (var->grayscale) {
	case 0:		/* color */
		switch (var->bits_per_pixel) {
		case 16:/* RGB565 */
			rgb = def_rgb565;
			break;
		case 32:/* ARGB8888 */
			rgb = def_argb32;
			break;
		default:
			return -EINVAL;
		}
		break;
	case 1:		/* grayscale */
		return -EINVAL;
	default:	/* fourcc */
		if ((bpp = overlay_fmt_support(var->grayscale)) < 0) {
			dev_err(info->dev, "unsupport pixel format for overlay\n");
			return -EINVAL;
		}

		var->bits_per_pixel = bpp;
		if (var->bits_per_pixel < 16)
			return -EINVAL;

		switch (var->grayscale) {
		case V4L2_PIX_FMT_ARGB32:
			rgb = def_argb32;
			break;
		case V4L2_PIX_FMT_XRGB32:
			rgb = def_rgb888;
			break;
		case V4L2_PIX_FMT_ARGB555:
			rgb = def_argb555;
			break;
		case V4L2_PIX_FMT_ARGB444:
			rgb = def_argb444;
			break;
		case V4L2_PIX_FMT_RGB555:
			rgb = def_rgb555;
			break;
		case V4L2_PIX_FMT_RGB444:
			rgb = def_rgb444;
			break;
		case V4L2_PIX_FMT_RGB565:
			rgb = def_rgb565;
			break;
		default:
			/*
			 * This should never be reached since the verification
			 * is done in overlay_fmt_support(), but handle this in
			 * case there will be a sync error between formats
			 * supported in fmt_support and this function.
			 */
			return -EINVAL;
		}
		break;
	}

	if (var->xres_virtual * var->yres_virtual * var->bits_per_pixel / 8 >
		info->fix.smem_len)
		return -EINVAL;

	fill_fmt_bitfields(var, rgb);

	return 0;
}

static int overlayfb_set_par(struct fb_info *info)
{
	int size, bpp;
	struct mxsfb_layer *ofb = (struct mxsfb_layer *)info->par;
	struct mxsfb_info  *fbi = ofb->fbi;
	struct fb_var_screeninfo *var = &ofb->ol_fb->var;

	bpp = var->bits_per_pixel;
	ofb->ol_fb->fix.line_length = var->xres_virtual * bpp / 8;

	size = PAGE_ALIGN(ofb->ol_fb->fix.line_length * var->yres_virtual);
	if (ofb->video_mem_size < size)
		return -EINVAL;

	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_enable_pix(fbi);
		clk_enable_axi(fbi);
		clk_enable_disp_axi(fbi);
	}

	if (ofb->blank_state == FB_BLANK_UNBLANK)
		ofb->ops->disable(ofb);

	ofb->ops->setup(ofb);

	if (ofb->blank_state == FB_BLANK_UNBLANK)
		ofb->ops->enable(ofb);

	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_disable_disp_axi(fbi);
		clk_disable_axi(fbi);
		clk_disable_pix(fbi);
	}

	if ((var->activate & FB_ACTIVATE_FORCE) &&
	    (var->activate & FB_ACTIVATE_MASK) == FB_ACTIVATE_NOW)
		var->activate = FB_ACTIVATE_NOW;

	return 0;
}

static int overlayfb_blank(int blank, struct fb_info *info)
{
	struct mxsfb_layer *ofb = (struct mxsfb_layer *)info->par;
	struct mxsfb_info  *fbi  = ofb->fbi;

	if (ofb->blank_state == blank)
		return 0;

	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_enable_pix(fbi);
		clk_enable_axi(fbi);
		clk_enable_disp_axi(fbi);
	}

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		ofb->ops->disable(ofb);
		break;
	case FB_BLANK_UNBLANK:
		ofb->ops->enable(ofb);
		break;
	}

	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_disable_disp_axi(fbi);
		clk_disable_axi(fbi);
		clk_disable_pix(fbi);
	}

	ofb->blank_state = blank;

	return 0;
}

static int overlayfb_pan_display(struct fb_var_screeninfo *var,
				 struct fb_info *info)
{
	int ret = 0;
	unsigned int bytes_offset;
	struct mxsfb_layer *ofb = (struct mxsfb_layer *)info->par;
	struct mxsfb_info  *fbi = ofb->fbi;

	init_completion(&fbi->flip_complete);

	if (fbi->cur_blank != FB_BLANK_UNBLANK)
		return -EINVAL;

	bytes_offset = info->fix.line_length * var->yoffset;
	writel(info->fix.smem_start + bytes_offset,
	       fbi->base + LCDC_AS_NEXT_BUF);

	/* update on next VSYNC */
	writel(CTRL1_CUR_FRAME_DONE_IRQ_EN,
	       fbi->base + LCDC_CTRL1 + REG_SET);

	ret = wait_for_completion_timeout(&fbi->flip_complete, HZ / 2);
	if (!ret) {
		dev_err(info->device,
			"overlay wait for pane flip timeout\n");
		return -ETIMEDOUT;
	}

	return 0;
}

static struct fb_ops overlay_fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= overlayfb_open,
	.fb_release	= overlayfb_release,
	.fb_check_var	= overlayfb_check_var,
	.fb_set_par	= overlayfb_set_par,
	.fb_blank	= overlayfb_blank,
	.fb_pan_display = overlayfb_pan_display,
	.fb_mmap	= mxsfb_mmap,
};

static void init_mxsfb_overlay(struct mxsfb_info *fbi,
			       struct mxsfb_layer *ofb)
{
	dev_dbg(&fbi->pdev->dev, "AS overlay init\n");

	ofb->ol_fb->fix.type		= FB_TYPE_PACKED_PIXELS;
	ofb->ol_fb->fix.xpanstep	= 0;
	ofb->ol_fb->fix.ypanstep	= 1;
	ofb->ol_fb->fix.ywrapstep	= 1;
	ofb->ol_fb->fix.visual		= FB_VISUAL_TRUECOLOR;
	ofb->ol_fb->fix.accel		= FB_ACCEL_NONE;

	ofb->ol_fb->var.activate	= FB_ACTIVATE_NXTOPEN;
	ofb->ol_fb->var.xres		= fbi->fb_info->var.xres;
	ofb->ol_fb->var.yres		= fbi->fb_info->var.yres;
	ofb->ol_fb->var.xres_virtual	= fbi->fb_info->var.xres_virtual;
	ofb->ol_fb->var.yres_virtual	= fbi->fb_info->var.yres;
	ofb->ol_fb->var.bits_per_pixel	= fbi->fb_info->var.bits_per_pixel;
	ofb->ol_fb->var.vmode		= FB_VMODE_NONINTERLACED;
	ofb->ol_fb->var.nonstd		= 0;

	/* Copy timings of primary fb */
	ofb->ol_fb->var.pixclock	= fbi->fb_info->var.pixclock;
	ofb->ol_fb->var.left_margin	= fbi->fb_info->var.left_margin;
	ofb->ol_fb->var.right_margin	= fbi->fb_info->var.right_margin;
	ofb->ol_fb->var.upper_margin	= fbi->fb_info->var.upper_margin;
	ofb->ol_fb->var.lower_margin	= fbi->fb_info->var.lower_margin;
	ofb->ol_fb->var.hsync_len	= fbi->fb_info->var.hsync_len;
	ofb->ol_fb->var.vsync_len	= fbi->fb_info->var.vsync_len;

	ofb->ol_fb->fbops = &overlay_fb_ops;
	ofb->ol_fb->node  = -1;
	ofb->ol_fb->par	  = ofb;
	INIT_LIST_HEAD(&ofb->ol_fb->modelist);

	ofb->id = 0;
	ofb->ops = &ofb_ops;
	atomic_set(&ofb->usage, 0);
	ofb->blank_state = -1;
	ofb->global_alpha = 255;
	ofb->fbi = fbi;

	sprintf(ofb->ol_fb->fix.id, "FG");
}

static int mxsfb_overlay_map_video_memory(struct mxsfb_info *fbi,
					  struct mxsfb_layer *ofb)
{
	struct fb_info *fb = fbi->fb_info;
	BUG_ON(!fb->fix.smem_len);

	ofb->video_mem_size = fb->fix.smem_len;
	ofb->video_mem = dma_alloc_wc(ofb->dev,
				      ofb->video_mem_size,
				      (dma_addr_t *)&ofb->video_mem_phys,
				      GFP_DMA | GFP_KERNEL);

	if (ofb->video_mem == NULL) {
		dev_err(ofb->dev, "Unable to allocate overlay fb memory\n");
		return -ENOMEM;
	}

	/* clear overlay fb memory buffer */
	memset(ofb->video_mem, 0x0, ofb->video_mem_size);

	ofb->ol_fb->fix.smem_start = ofb->video_mem_phys;
	ofb->ol_fb->fix.smem_len   = ofb->video_mem_size;
	ofb->ol_fb->screen_base    = ofb->video_mem;

	return 0;
}

static void mxsfb_overlay_init(struct mxsfb_info *fbi)
{
	int ret;
	struct mxsfb_layer *ofb = &fbi->overlay;
	struct fb_videomode ofb_vm;

	ofb->dev = &fbi->pdev->dev;
	ofb->ol_fb = framebuffer_alloc(0, ofb->dev);
	if (!ofb->ol_fb) {
		dev_err(ofb->dev, "Failed to allocate overlay fbinfo\n");
		return;
	}

	init_mxsfb_overlay(fbi, ofb);

	/* add videomode to overlay fb */
	fb_var_to_videomode(&ofb_vm, &fbi->fb_info->var);
	ret = fb_add_videomode(&ofb_vm, &ofb->ol_fb->modelist);
	if (ret) {
		dev_err(ofb->dev, "add vm to ofb failed\n");
		goto fb_release;
	}

	ret = register_framebuffer(ofb->ol_fb);
	if (ret) {
		dev_err(ofb->dev, "failed to register overlay\n");
		goto fb_release;
	}

	ret = mxsfb_overlay_map_video_memory(fbi, ofb);
	if (ret) {
		dev_err(ofb->dev, "failed to map video mem for overlay\n");
		goto fb_unregister;
	}

	/* setup the initial params for overlay fb */
	overlayfb_check_var(&ofb->ol_fb->var, ofb->ol_fb);
	overlayfb_set_par(ofb->ol_fb);

	ofb->registered = 1;

	return;

fb_unregister:
	unregister_framebuffer(ofb->ol_fb);
fb_release:
	framebuffer_release(ofb->ol_fb);
}

static void mxsfb_overlay_exit(struct mxsfb_info *fbi)
{
	struct mxsfb_layer *ofb = &fbi->overlay;

	if (ofb->registered) {
		if (ofb->video_mem)
			dma_free_wc(ofb->dev, ofb->video_mem_size,
				    ofb->video_mem, ofb->video_mem_phys);

		unregister_framebuffer(ofb->ol_fb);
		framebuffer_release(ofb->ol_fb);
	}
}

#ifdef CONFIG_PM_SLEEP
static u32 saved_as_ctrl;
static u32 saved_as_next_buf;

static void mxsfb_overlay_resume(struct mxsfb_info *fbi)
{
	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_enable_pix(fbi);
		clk_enable_axi(fbi);
		clk_enable_disp_axi(fbi);
	}

	/* Pull LCDIF out of reset */
	writel(0xc0000000, fbi->base + LCDC_CTRL + REG_CLR);

	writel(saved_as_ctrl, fbi->base + LCDC_AS_CTRL);
	writel(saved_as_next_buf, fbi->base + LCDC_AS_NEXT_BUF);

	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_disable_disp_axi(fbi);
		clk_disable_axi(fbi);
		clk_disable_pix(fbi);
	}

}

static void mxsfb_overlay_suspend(struct mxsfb_info *fbi)
{
	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_enable_pix(fbi);
		clk_enable_axi(fbi);
		clk_enable_disp_axi(fbi);
	}

	saved_as_ctrl = readl(fbi->base + LCDC_AS_CTRL);
	saved_as_next_buf = readl(fbi->base + LCDC_AS_NEXT_BUF);

	if (fbi->cur_blank != FB_BLANK_UNBLANK) {
		clk_disable_disp_axi(fbi);
		clk_disable_axi(fbi);
		clk_disable_pix(fbi);
	}
}
#endif

#else
static void mxsfb_overlay_init(struct mxsfb_info *fbi) {}
static void mxsfb_overlay_exit(struct mxsfb_info *fbi) {}
static void mxsfb_overlay_resume(struct mxsfb_info *fbi) {}
static void mxsfb_overlay_suspend(struct mxsfb_info *fbi) {}
#endif

static int mxsfb_probe(struct platform_device *pdev)
{
	const struct of_device_id *of_id =
			of_match_device(mxsfb_dt_ids, &pdev->dev);
	struct resource *res;
	struct mxsfb_info *host;
	struct fb_info *fb_info;
	struct pinctrl *pinctrl;
	int irq = platform_get_irq(pdev, 0);
	int gpio, ret;

	if (of_id)
		pdev->id_entry = of_id->data;

	gpio = of_get_named_gpio(pdev->dev.of_node, "enable-gpio", 0);
	if (gpio == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	if (gpio_is_valid(gpio)) {
		ret = devm_gpio_request_one(&pdev->dev, gpio,
					GPIOF_OUT_INIT_LOW, "lcd_pwr_en");
		if (ret) {
			dev_err(&pdev->dev,
			"failed to request gpio %d, ret = %d\n", gpio, ret);
			return ret;
		}
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get memory IO resource\n");
		return -ENODEV;
	}

	host = devm_kzalloc(&pdev->dev, sizeof(struct mxsfb_info), GFP_KERNEL);
	if (!host) {
		dev_err(&pdev->dev, "Failed to allocate IO resource\n");
		return -ENOMEM;
	}

	fb_info = framebuffer_alloc(0, &pdev->dev);
	if (!fb_info) {
		dev_err(&pdev->dev, "Failed to allocate fbdev\n");
		devm_kfree(&pdev->dev, host);
		return -ENOMEM;
	}
	host->fb_info = fb_info;
	fb_info->par = host;

	ret = devm_request_irq(&pdev->dev, irq, mxsfb_irq_handler, 0,
			  dev_name(&pdev->dev), host);
	if (ret) {
		dev_err(&pdev->dev, "request_irq (%d) failed with error %d\n",
				irq, ret);
		ret = -ENODEV;
		goto fb_release;
	}

	host->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(host->base)) {
		dev_err(&pdev->dev, "ioremap failed\n");
		ret = PTR_ERR(host->base);
		goto fb_release;
	}

	host->pdev = pdev;
	platform_set_drvdata(pdev, host);

	host->devdata = &mxsfb_devdata[pdev->id_entry->driver_data];

	host->clk_pix = devm_clk_get(&host->pdev->dev, "pix");
	if (IS_ERR(host->clk_pix)) {
		host->clk_pix = NULL;
		ret = PTR_ERR(host->clk_pix);
		goto fb_release;
	}

	host->clk_axi = devm_clk_get(&host->pdev->dev, "axi");
	if (IS_ERR(host->clk_axi)) {
		host->clk_axi = NULL;
		ret = PTR_ERR(host->clk_axi);
		dev_err(&pdev->dev, "Failed to get axi clock: %d\n", ret);
		goto fb_release;
	}

	host->clk_disp_axi = devm_clk_get(&host->pdev->dev, "disp_axi");
	if (IS_ERR(host->clk_disp_axi)) {
		host->clk_disp_axi = NULL;
		ret = PTR_ERR(host->clk_disp_axi);
		dev_err(&pdev->dev, "Failed to get disp_axi clock: %d\n", ret);
		goto fb_release;
	}

	host->reg_lcd = devm_regulator_get(&pdev->dev, "lcd");
	if (IS_ERR(host->reg_lcd))
		host->reg_lcd = NULL;

	fb_info->pseudo_palette = devm_kcalloc(&pdev->dev, 16, sizeof(u32),
					       GFP_KERNEL);
	if (!fb_info->pseudo_palette) {
		ret = -ENOMEM;
		dev_err(&pdev->dev, "Failed to allocate pseudo_palette memory\n");
		goto fb_release;
	}

	INIT_LIST_HEAD(&fb_info->modelist);

	pm_runtime_enable(&host->pdev->dev);

	ret = mxsfb_init_fbinfo(host);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to initialize fbinfo: %d\n", ret);
		goto fb_pm_runtime_disable;
	}

	ret = mxsfb_dispdrv_init(pdev, fb_info);
	if (ret != 0) {
		if (ret == -EPROBE_DEFER)
			dev_info(&pdev->dev,
				 "Defer fb probe due to dispdrv not ready\n");
		goto fb_free_videomem;
	}

	if (!host->dispdrv) {
		pinctrl = devm_pinctrl_get_select_default(&pdev->dev);
		if (IS_ERR(pinctrl)) {
			ret = PTR_ERR(pinctrl);
			goto fb_pm_runtime_disable;
		}
	}

	if (!host->enabled) {
		writel(0, host->base + LCDC_CTRL);
		mxsfb_set_par(fb_info);
		mxsfb_enable_controller(fb_info);
		pm_runtime_get_sync(&host->pdev->dev);
	}

	ret = register_framebuffer(fb_info);
	if (ret != 0) {
		dev_err(&pdev->dev, "Failed to register framebuffer\n");
		goto fb_destroy;
	}

	mxsfb_overlay_init(host);

#ifndef CONFIG_FB_IMX64_DEBUG
	console_lock();
	ret = fb_blank(fb_info, FB_BLANK_UNBLANK);
	console_unlock();
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to unblank framebuffer\n");
		goto fb_unregister;
	}
#endif

	dev_info(&pdev->dev, "initialized\n");

	return 0;

#ifndef CONFIG_FB_IMX64_DEBUG
fb_unregister:
	unregister_framebuffer(fb_info);
#endif
fb_destroy:
	fb_destroy_modelist(&fb_info->modelist);
fb_free_videomem:
	mxsfb_free_videomem(host);
fb_pm_runtime_disable:
	clk_disable_pix(host);
	clk_disable_axi(host);
	clk_disable_disp_axi(host);

	pm_runtime_disable(&host->pdev->dev);
	devm_kfree(&pdev->dev, fb_info->pseudo_palette);
fb_release:
	framebuffer_release(fb_info);
	devm_kfree(&pdev->dev, host);

	return ret;
}

static int mxsfb_remove(struct platform_device *pdev)
{
	struct mxsfb_info *host = platform_get_drvdata(pdev);
	struct fb_info *fb_info = host->fb_info;

	if (host->enabled)
		mxsfb_disable_controller(fb_info);

	if (host->devdata->flags & MXSFB_FLAG_PMQOS)
		cpu_latency_qos_remove_request(&host->pm_qos_req);

	pm_runtime_disable(&host->pdev->dev);
	mxsfb_overlay_exit(host);
	unregister_framebuffer(fb_info);
	mxsfb_free_videomem(host);

	platform_set_drvdata(pdev, NULL);

	devm_kfree(&pdev->dev, fb_info->pseudo_palette);
	framebuffer_release(fb_info);
	devm_kfree(&pdev->dev, host);

	return 0;
}

static void mxsfb_shutdown(struct platform_device *pdev)
{
	struct mxsfb_info *host = platform_get_drvdata(pdev);

	/*
	 * Force stop the LCD controller as keeping it running during reboot
	 * might interfere with the BootROM's boot mode pads sampling.
	 */
	if (host->cur_blank == FB_BLANK_UNBLANK) {
		writel(CTRL_RUN, host->base + LCDC_CTRL + REG_CLR);
		writel(CTRL_MASTER, host->base + LCDC_CTRL + REG_CLR);
	}
}

#ifdef CONFIG_PM
static int mxsfb_runtime_suspend(struct device *dev)
{
	struct mxsfb_info *host = dev_get_drvdata(dev);

	if (host->devdata->flags & MXSFB_FLAG_BUSFREQ)
		release_bus_freq(BUS_FREQ_HIGH);

	if (host->devdata->flags & MXSFB_FLAG_PMQOS)
		cpu_latency_qos_remove_request(&host->pm_qos_req);

	dev_dbg(dev, "mxsfb busfreq high release.\n");

	return 0;
}

static int mxsfb_runtime_resume(struct device *dev)
{
	struct mxsfb_info *host = dev_get_drvdata(dev);

	if (host->devdata->flags & MXSFB_FLAG_BUSFREQ)
		request_bus_freq(BUS_FREQ_HIGH);

	if (host->devdata->flags & MXSFB_FLAG_PMQOS)
		cpu_latency_qos_add_request(&host->pm_qos_req, 0);

	dev_dbg(dev, "mxsfb busfreq high request.\n");

	return 0;
}
#endif

#ifdef CONFIG_PM_SLEEP
static int mxsfb_suspend(struct device *pdev)
{
	struct mxsfb_info *host = dev_get_drvdata(pdev);
	struct fb_info *fb_info = host->fb_info;
	int saved_blank;

	console_lock();
	mxsfb_overlay_suspend(host);
	fb_set_suspend(fb_info, 1);
	saved_blank = host->cur_blank;
	mxsfb_blank(FB_BLANK_POWERDOWN, fb_info);
	host->restore_blank = saved_blank;
	console_unlock();

	pinctrl_pm_select_sleep_state(pdev);

	return 0;
}

static int mxsfb_resume(struct device *pdev)
{
	struct mxsfb_info *host = dev_get_drvdata(pdev);
	struct fb_info *fb_info = host->fb_info;

	pinctrl_pm_select_default_state(pdev);

	console_lock();
	mxsfb_overlay_resume(host);
	mxsfb_blank(host->restore_blank, fb_info);
	fb_set_suspend(fb_info, 0);
	console_unlock();

	return 0;
}
#endif

static const struct dev_pm_ops mxsfb_pm_ops = {
	SET_RUNTIME_PM_OPS(mxsfb_runtime_suspend, mxsfb_runtime_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(mxsfb_suspend, mxsfb_resume)
};

static struct platform_driver mxsfb_driver = {
	.probe = mxsfb_probe,
	.remove = mxsfb_remove,
	.shutdown = mxsfb_shutdown,
	.id_table = mxsfb_devtype,
	.driver = {
		   .name = DRIVER_NAME,
		   .of_match_table = mxsfb_dt_ids,
		   .pm = &mxsfb_pm_ops,
	},
};

module_platform_driver(mxsfb_driver);

MODULE_DESCRIPTION("Freescale mxs framebuffer driver");
MODULE_AUTHOR("Sascha Hauer, Pengutronix");
MODULE_LICENSE("GPL");
