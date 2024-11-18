/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017 NXP.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/bitops.h>
#include <linux/gcd.h>
#include <linux/mipi_dsi_northwest.h>
#include <linux/module.h>
#include <linux/mxcfb.h>
#include <linux/pm_runtime.h>
#include <linux/busfreq-imx.h>
#include <linux/backlight.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <video/mipi_display.h>
#include <video/mxc_edid.h>
#include <linux/mfd/syscon.h>

#include "mipi_dsi.h"

#define DISPDRV_MIPI			"mipi_dsi_northwest"
#define ROUND_UP(x)			((x)+1)
#define NS2PS_RATIO			(1000)
#define	MIPI_LCD_SLEEP_MODE_DELAY	(120)
#define MIPI_FIFO_TIMEOUT		msecs_to_jiffies(250)
#define PICOS_PER_SEC			(1000000000ULL)
#define PICOS2KHZ2(a, bpp)		\
	DIV_ROUND_CLOSEST_ULL(PICOS_PER_SEC * (bpp), (a))

static struct mipi_dsi_match_lcd mipi_dsi_lcd_db[] = {
#ifdef CONFIG_FB_MXC_TRULY_WVGA_SYNC_PANEL
	{
	 "TRULY-WVGA",
	 {mipid_hx8369_get_lcd_videomode, mipid_hx8369_lcd_setup}
	},
#endif
#ifdef CONFIG_FB_MXC_TRULY_PANEL_TFT3P5079E
	{
	 "TRULY-WVGA-TFT3P5079E",
	 {mipid_otm8018b_get_lcd_videomode, mipid_otm8018b_lcd_setup}
	},
#endif
#ifdef CONFIG_FB_MXC_TRULY_PANEL_TFT3P5581E
	{
	 "TRULY-WVGA-TFT3P5581E",
	 {mipid_hx8363_get_lcd_videomode, mipid_hx8363_lcd_setup}
	},
#endif
#ifdef CONFIG_FB_MXC_RK_PANEL_RK055AHD042
	{
	 "ROCKTECH-WXGA-RK055AHD042",
	 {mipid_rm68200_get_lcd_videomode, mipid_rm68200_lcd_setup}
	},
#endif
#ifdef CONFIG_FB_MXC_RK_PANEL_RK055IQH042
	{
	 "ROCKTECH-QHD-RK055IQH042",
	 {mipid_rm68191_get_lcd_videomode, mipid_rm68191_lcd_setup}
	},
#endif
	{
	"", {NULL, NULL}
	}
};

enum mipi_dsi_mode {
	DSI_COMMAND_MODE,
	DSI_VIDEO_MODE
};

#define DSI_LP_MODE	0
#define DSI_HS_MODE	1

enum mipi_dsi_payload {
	DSI_PAYLOAD_CMD,
	DSI_PAYLOAD_VIDEO,
};

struct pll_divider {
	unsigned int cm;  /* multiplier */
	unsigned int cn;  /* predivider */
	unsigned int co;  /* outdivider */
};

/**
 * 'CM' value to 'CM' reigister config value map
 * 'CM' = [16, 255];
 */
static unsigned int cm_map_table[240] = {
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,	/* 16 ~ 23 */
	0xf8, 0xf9, 0xfa, 0xfb, 0xfc, 0xfd, 0xfe, 0xff,	/* 24 ~ 31 */

	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,	/* 32 ~ 39 */
	0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf, /* 40 ~ 47 */

	0xd0, 0xd1, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, /* 48 ~ 55 */
	0xd8, 0xd9, 0xda, 0xdb, 0xdc, 0xdd, 0xde, 0xdf, /* 56 ~ 63 */

	0x80, 0x81, 0x82, 0x83, 0x84, 0x85, 0x86, 0x87, /* 64 ~ 71 */
	0x88, 0x89, 0x8a, 0x8b, 0x8c, 0x8d, 0x8e, 0x8f, /* 72 ~ 79 */

	0x90, 0x91, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, /* 80 ~ 87 */
	0x98, 0x99, 0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 0x9f, /* 88 ~ 95 */

	0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, /* 96  ~ 103 */
	0xa8, 0xa9, 0xaa, 0xab, 0xac, 0xad, 0xae, 0xaf, /* 104 ~ 111 */

	0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, /* 112 ~ 119 */
	0xb8, 0xb9, 0xba, 0xbb, 0xbc, 0xbd, 0xbe, 0xbf, /* 120 ~ 127 */

	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, /* 128 ~ 135 */
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, /* 136 ~ 143 */

	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, /* 144 ~ 151 */
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, /* 152 ~ 159 */

	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, /* 160 ~ 167 */
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, /* 168 ~ 175 */

	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, /* 176 ~ 183 */
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, /* 184 ~ 191 */

	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, /* 192 ~ 199 */
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, /* 200 ~ 207 */

	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57, /* 208 ~ 215 */
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f, /* 216 ~ 223 */

	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, /* 224 ~ 231 */
	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, /* 232 ~ 239 */

	0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, /* 240 ~ 247 */
	0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f	/* 248 ~ 255 */
};

/**
 * map 'CN' value to 'CN' reigister config value
 * 'CN' = [1, 32];
 */
static unsigned int cn_map_table[32] = {
	0x1f, 0x00, 0x10, 0x18, 0x1c, 0x0e, 0x07, 0x13,	/* 1  ~ 8  */
	0x09, 0x04, 0x02, 0x11, 0x08, 0x14, 0x0a, 0x15,	/* 9  ~ 16 */
	0x1a, 0x1d, 0x1e, 0x0f, 0x17, 0x1b, 0x0d, 0x16,	/* 17 ~ 24 */
	0x0b, 0x05, 0x12, 0x19, 0x0c, 0x06, 0x03, 0x01	/* 25 ~ 32 */
};

/**
 * map 'CO' value to 'CO' reigister config value
 * 'CO' = { 1, 2, 4, 8 };
 */
static unsigned int co_map_table[4] = {
	0x0, 0x1, 0x2, 0x3
};

static DECLARE_COMPLETION(dsi_rx_done);
static DECLARE_COMPLETION(dsi_tx_done);

static void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
			      uint8_t mode);
static int mipi_dsi_dcs_cmd(struct mipi_dsi_info *mipi_dsi,
			    u8 cmd, const u32 *param, int num);

static int mipi_dsi_lcd_init(struct mipi_dsi_info *mipi_dsi,
			     struct mxc_dispdrv_setting *setting)
{
	u32 data_lane_num, max_data_rate;
	int i, size, err = 0;
	struct fb_videomode *mipi_lcd_modedb;
	struct fb_videomode mode;
	struct device *dev = &mipi_dsi->pdev->dev;

	err = of_property_read_u32(dev->of_node,
			"data-lanes-num", &data_lane_num);
	if (err) {
		dev_err(dev, "failed to get data lane num\n");
		goto err0;
	} else if (data_lane_num > 4) {
		dev_err(dev, "invalid data lane number\n");
		err = -EINVAL;
		goto err0;
	}

	err = of_property_read_u32(dev->of_node,
			"max-data-rate", &max_data_rate);
	if (err) {
		dev_err(dev, "failed to get max data rate\n");
		goto err0;
	}

	if (mipi_dsi->encoder) {
		mipi_dsi->lcd_config->virtual_ch = 0;
		mipi_dsi->lcd_config->data_lane_num = data_lane_num;
		mipi_dsi->lcd_config->max_phy_clk = max_data_rate;
		mipi_dsi->lcd_config->dpi_fmt = MIPI_RGB888;
		setting->fbi->var.bits_per_pixel = 32;

		/* TODO Add bandwidth check */

		if (setting->fbi->fbops->fb_check_var)
			err = setting->fbi->fbops->fb_check_var(&setting->fbi->var,
					setting->fbi);
		if (err)
			goto err0;

		err = fb_add_videomode(mipi_dsi->mode,
				&setting->fbi->modelist);
		if (err)
			goto err0;

		fb_videomode_to_var(&setting->fbi->var, mipi_dsi->mode);
		setting->fbi->mode = mipi_dsi->mode;
err0:
		return err;
	}

	for (i = 0; i < ARRAY_SIZE(mipi_dsi_lcd_db); i++) {
		if (!strcmp(mipi_dsi->lcd_panel,
					mipi_dsi_lcd_db[i].lcd_panel)) {
			mipi_dsi->lcd_callback =
				&mipi_dsi_lcd_db[i].lcd_callback;
			break;
		}
	}
	if (i == ARRAY_SIZE(mipi_dsi_lcd_db)) {
		dev_err(dev, "failed to find supported lcd panel.\n");
		return -EINVAL;
	}

	/* set default bpp to 32 if not set*/
	if (!setting->default_bpp)
		setting->default_bpp = 32;

	mipi_dsi->lcd_callback->get_mipi_lcd_videomode(&mipi_lcd_modedb, &size,
			&mipi_dsi->lcd_config);

	err = fb_find_mode(&setting->fbi->var, setting->fbi,
			setting->dft_mode_str,
			mipi_lcd_modedb, size, NULL,
			setting->default_bpp);
	if (err != 1)
		fb_videomode_to_var(&setting->fbi->var, mipi_lcd_modedb);

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < size; i++) {
		fb_var_to_videomode(&mode, &setting->fbi->var);
		if (fb_mode_is_equal(&mode, mipi_lcd_modedb + i)) {
			err = fb_add_videomode(mipi_lcd_modedb + i,
					&setting->fbi->modelist);
			mipi_dsi->mode = mipi_lcd_modedb + i;
			break;
		}
	}

	if ((err < 0) || (size == i)) {
		dev_err(dev, "failed to add videomode.\n");
		return err;
	}

	setting->fbi->mode = mipi_dsi->mode;

	return 0;
}

static int mipi_dsi_disp_init(struct mxc_dispdrv_handle *disp,
			      struct mxc_dispdrv_setting *setting)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);
	struct device *dev = &mipi_dsi->pdev->dev;
	struct device_node *np = dev->of_node;
	struct reset_control *reset = NULL;
	int ret = 0;

	if (!mipi_dsi->encoder) {
		reset = of_reset_control_get(np, NULL);
		if (IS_ERR(reset))
			return PTR_ERR(reset);
	}

	ret = mipi_dsi_lcd_init(mipi_dsi, setting);
	if (ret) {
		dev_err(dev, "failed to init mipi dsi lcd\n");
		goto out;
	}

	dev_info(dev, "MIPI DSI dispdv inited\n");

out:
	if (!mipi_dsi->encoder)
		reset_control_put(reset);

	return ret;
}

static void mipi_dsi_disp_deinit(struct mxc_dispdrv_handle *disp)
{
	struct mipi_dsi_info *mipi_dsi;

	mipi_dsi = mxc_dispdrv_getdata(disp);

	if (mipi_dsi->bl)
		backlight_device_unregister(mipi_dsi->bl);
}

static void mipi_dsi_set_mode(struct mipi_dsi_info *mipi_dsi,
			      uint8_t mode)
{
	uint32_t pkt_control;

	pkt_control = readl(mipi_dsi->mmio_base + HOST_PKT_CONTROL);

	switch (mode) {
	case DSI_LP_MODE:
		writel(0x1, mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK);
		break;
	case DSI_HS_MODE:
		writel(0x0, mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK);
		break;
	default:
		dev_err(&mipi_dsi->pdev->dev,
			"invalid dsi mode\n");
		return;
	}

	mdelay(1);
}

static uint32_t fmt_to_bpp(enum mipi_dsi_dpi_fmt dpi_fmt)
{
	switch (dpi_fmt) {
	case MIPI_RGB888:
		return 24;
	case MIPI_RGB565_PACKED:
		return 16;
	default:
		return 0;
	}
}

/*
static void dphy_calc_dividers(int *cm, int *cn, int *co)
{
}
*/

static int mipi_dsi_dphy_init(struct mipi_dsi_info *mipi_dsi)
{
	int i, best_div = -1;
	int64_t delta;
	uint64_t least_delta = ~0U;
	uint32_t bpp, time_out = 100;
	uint32_t lock;
	uint32_t req_bit_clk;
	uint64_t limit, div_result;
	uint64_t denominator, numerator, divisor;
	uint64_t norm_denom, norm_num, split_denom;
	struct pll_divider div = { 0 };
	struct fb_videomode *mode = mipi_dsi->mode;
	struct mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

#ifndef CONFIG_FB_IMX64
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1,
			   MIPI_ISO_DISABLE, MIPI_ISO_DISABLE);
#endif

	bpp = fmt_to_bpp(lcd_config->dpi_fmt);
	req_bit_clk = PICOS2KHZ2(mode->pixclock, bpp) * 1000U;

	switch (lcd_config->data_lane_num) {
	case 1:
		break;
	case 2:
		req_bit_clk = req_bit_clk >> 1;
		break;
	case 4:
		req_bit_clk = req_bit_clk >> 2;
		break;
	default:
		dev_err(&mipi_dsi->pdev->dev,
			"requested data lane num is invalid\n");
		return -EINVAL;
	}

	if (mipi_dsi->encoder) {
		if (req_bit_clk > lcd_config->max_phy_clk)
			return -EINVAL;
	}

	/* calc CM, CN and CO according to PHY PLL formula:
	 *
	 * 'PLL out bitclk = refclk * CM / (CN * CO);'
	 *
	 * Let:
	 * 'numerator   = bitclk / divisor';
	 * 'denominator = refclk / divisor';
	 * Then:
	 * 'numerator / denominator = CM / (CN * CO)';
	 *
	 * CM is in [16, 255]
	 * CN is in [1, 32]
	 * CO is in { 1, 2, 4, 8 };
	 */
	divisor = gcd(mipi_dsi->phy_ref_clkfreq, req_bit_clk);
	WARN_ON(divisor == 1);

	div_result = req_bit_clk;
	do_div(div_result, divisor);
	numerator = div_result;

	div_result = mipi_dsi->phy_ref_clkfreq;
	do_div(div_result, divisor);
	denominator = div_result;

	/* denominator & numerator out of range check */
	if (DIV_ROUND_CLOSEST_ULL(numerator, denominator) > 255 ||
	    DIV_ROUND_CLOSEST_ULL(denominator, numerator) > 32 * 8)
		return -EINVAL;

	/* Normalization: reduce or increase
	 * numerator	to [16, 255]
	 * denominator	to [1, 32 * 8]
	 * Reduce normalization result is 'approximiate'
	 * Increase nomralization result is 'precise'
	 */
	if (numerator > 255 || denominator > 32 * 8) {
		/* approximate */
		if (likely(numerator > denominator)) {
			/* 'numerator > 255';
			 * 'limit' should meet below conditions:
			 *  a. '(numerator   / limit) >= 16'
			 *  b. '(denominator / limit) >= 1'
			 */
			limit = min(denominator,
				    DIV_ROUND_CLOSEST_ULL(numerator, 16));

			/* Let:
			 * norm_num   = numerator   / i;
			 * norm_denom = denominator / i;
			 *
			 * So:
			 * delta = numerator * norm_denom -
			 * 	   denominator * norm_num
			 */
			for (i = 2; i <= limit; i++) {
				norm_num = DIV_ROUND_CLOSEST_ULL(numerator, i);
				if (norm_num > 255)
					continue;

				norm_denom = DIV_ROUND_CLOSEST_ULL(denominator, i);

				/* 'norm_num <= 255' && 'norm_num > norm_denom'
				 * so, 'norm_denom < 256'
				 */
				delta = numerator * norm_denom -
					denominator * norm_num;
				delta = abs(delta);
				if (delta < least_delta) {
					least_delta = delta;
					best_div = i;
				} else if (delta == least_delta) {
					/* choose better one IF:
					 * 'norm_denom' derived from last 'best_div'
					 * needs later split, i.e, 'norm_denom > 32'.
					 */
					if (DIV_ROUND_CLOSEST_ULL(denominator, best_div) > 32) {
						least_delta = delta;
						best_div = i;
					}
				}
			}
		} else {
			/* 'denominator > 32 * 8';
			 * 'limit' should meet below conditions:
			 *  a. '(numerator   / limit >= 16'
			 *  b. '(denominator / limit >= 1': obviously.
			 */
			limit = DIV_ROUND_CLOSEST_ULL(numerator, 16);
			if (!limit ||
			    DIV_ROUND_CLOSEST_ULL(denominator, limit) > 32 * 8)
				return -EINVAL;

			for (i = 2; i <= limit; i++) {
				norm_denom = DIV_ROUND_CLOSEST_ULL(denominator, i);
				if (norm_denom > 32 * 8)
					continue;

				norm_num = DIV_ROUND_CLOSEST_ULL(numerator, i);

				/* 'norm_denom <= 256' && 'norm_num < norm_denom'
				 * so, 'norm_num <= 255'
				 */
				delta = numerator * norm_denom -
					denominator * norm_num;
				delta = abs(delta);
				if (delta < least_delta) {
					least_delta = delta;
					best_div = i;
				} else if (delta == least_delta) {
					if (DIV_ROUND_CLOSEST_ULL(denominator, best_div) > 32) {
						least_delta = delta;
						best_div = i;
					}
				}
			}
		}

		numerator   = DIV_ROUND_CLOSEST_ULL(numerator, best_div);
		denominator = DIV_ROUND_CLOSEST_ULL(denominator, best_div);
	} else if (numerator < 16) {
		/* precise */

		/* 'limit' should meet below conditions:
		 *  a. 'denominator * limit <= 32 * 8'
		 *  b. '16 <= numerator * limit <= 255'
		 *  Choose 'limit' to be the least value
		 *  which makes 'numerator * limit' to be
		 *  in [16, 255].
		 */
		limit = min(256 / (uint32_t)denominator,
			    255 / (uint32_t)numerator);
		if (limit == 1 || limit < DIV_ROUND_UP_ULL(16, numerator))
			return -EINVAL;

		/* choose the least available value for 'limit' */
		limit = DIV_ROUND_UP_ULL(16, numerator);
		numerator   = numerator * limit;
		denominator = denominator * limit;

		WARN_ON(numerator < 16 || denominator > 32 * 8);
	}

	div.cm = cm_map_table[numerator - 16];

	/* split 'denominator' to 'CN' and 'CO' */
	if (denominator > 32) {
		/* traverse four possible values of 'CO'
		 * there must be some value of 'CO' can be used
		 */
		least_delta = ~0U;
		for (i = 0; i < 4; i++) {
			split_denom = DIV_ROUND_CLOSEST_ULL(denominator, 1 << i);
			if (split_denom > 32)
				continue;

			/* calc deviation to choose the best one */
			delta = denominator - split_denom * (1 << i);
			delta = abs(delta);
			if (delta < least_delta) {
				least_delta = delta;
				div.co = co_map_table[i];
				div.cn = cn_map_table[split_denom - 1];
			}
		}
	} else {
		div.co = co_map_table[1 >> 1];
		div.cn = cn_map_table[denominator - 1];
	}

	writel(div.cn, mipi_dsi->mmio_base + DPHY_CN);
	writel(div.cm, mipi_dsi->mmio_base + DPHY_CM);
	writel(div.co, mipi_dsi->mmio_base + DPHY_CO);

	writel(0x25, mipi_dsi->mmio_base + DPHY_TST);
	writel(0x0, mipi_dsi->mmio_base + DPHY_PD_PLL);

	while(!(lock = readl(mipi_dsi->mmio_base + DPHY_LOCK))) {
		udelay(10);
		time_out--;
		if (time_out == 0) {
			dev_err(&mipi_dsi->pdev->dev, "cannot get the dphy lock = 0x%x\n", lock);
			return -EINVAL;
		}
	}

	dev_dbg(&mipi_dsi->pdev->dev, "%s: dphy lock = 0x%x\n", __func__, lock);

	writel(0x0, mipi_dsi->mmio_base + DPHY_LOCK_BYP);
	writel(0x1, mipi_dsi->mmio_base + DPHY_RTERM_SEL);
	writel(0x0, mipi_dsi->mmio_base + DPHY_AUTO_PD_EN);
	writel(0x1, mipi_dsi->mmio_base + DPHY_RXLPRP);
	writel(0x1, mipi_dsi->mmio_base + DPHY_RXCDRP);
	writel(0x0, mipi_dsi->mmio_base + DPHY_M_PRG_HS_PREPARE);
	writel(0x0, mipi_dsi->mmio_base + DPHY_MC_PRG_HS_PREPARE);
	writel(0x9, mipi_dsi->mmio_base + DPHY_M_PRG_HS_ZERO);
	writel(0x20, mipi_dsi->mmio_base + DPHY_MC_PRG_HS_ZERO);
	writel(0x5, mipi_dsi->mmio_base + DPHY_M_PRG_HS_TRAIL);
	writel(0x5, mipi_dsi->mmio_base + DPHY_MC_PRG_HS_TRAIL);
	writel(0x0, mipi_dsi->mmio_base + DPHY_PD_DPHY);

#ifndef CONFIG_FB_IMX64
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			   DSI_PLL_EN, DSI_PLL_EN);
#endif

	return 0;
}

static int mipi_dsi_host_init(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t lane_num;
	struct mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	switch (lcd_config->data_lane_num) {
	case 1:
		lane_num = 0x0;
		break;
	case 2:
		lane_num = 0x1;
		break;
	case 4:
		lane_num = 0x3;
		break;
	default:
		/* Invalid lane num */
		return -EINVAL;
	}

#ifdef CONFIG_FB_IMX64_DEBUG
	printk("%s: data_lane_num = %d\n", __func__, lcd_config->data_lane_num);
#endif

	writel(lane_num, mipi_dsi->mmio_base + HOST_CFG_NUM_LANES);
	writel(mipi_dsi->encoder ? 0x0 : 0x1,
	       mipi_dsi->mmio_base + HOST_CFG_NONCONTINUOUS_CLK);
	writel(0x1, mipi_dsi->mmio_base + HOST_CFG_T_PRE);
	writel(52, mipi_dsi->mmio_base + HOST_CFG_T_POST);
	writel(13, mipi_dsi->mmio_base + HOST_CFG_TX_GAP);
	writel(mipi_dsi->encoder ? 0x0 : 0x1,
	       mipi_dsi->mmio_base + HOST_CFG_AUTOINSERT_EOTP);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_EXTRA_CMDS_AFTER_EOTP);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_HTX_TO_COUNT);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_LRX_H_TO_COUNT);
	writel(0x0, mipi_dsi->mmio_base + HOST_CFG_BTA_H_TO_COUNT);
	writel(0x3A98, mipi_dsi->mmio_base + HOST_CFG_TWAKEUP);

	return 0;
}

static int mipi_dsi_dpi_init(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t bpp, color_coding, pixel_fmt;
	uint32_t pixel_fifo_level, hfp_period, hbp_period, hsa_period;
	struct fb_videomode *mode = mipi_dsi->mode;
	struct mipi_lcd_config *lcd_config = mipi_dsi->lcd_config;

	bpp = fmt_to_bpp(lcd_config->dpi_fmt);

	writel(mode->xres, mipi_dsi->mmio_base + DPI_PIXEL_PAYLOAD_SIZE);

	switch (mipi_dsi->traffic_mode) {
	case DSI_NON_BURST_WITH_SYNC_PULSE:
#ifdef CONFIG_FB_IMX64
		pixel_fifo_level = 8;
		hfp_period = mode->right_margin - DSI_HFP_PKT_OVERHEAD;
		hbp_period = mode->left_margin  - DSI_HBP_PKT_OVERHEAD;
		hsa_period = mode->hsync_len - DSI_HSA_PKT_OVERHEAD;
#else
		pixel_fifo_level = mode->xres;
		hfp_period = 0x10;
		hbp_period = 0x60;
		hsa_period = 0xf0;
#endif
		break;
	case DSI_BURST_MODE:
		pixel_fifo_level = mode->xres;
#ifdef CONFIG_FB_IMX64
		hfp_period = mode->right_margin;
		hbp_period = mode->left_margin;
		hsa_period = mode->hsync_len;
#else
		hfp_period = mode->right_margin * (bpp >> 3);
		hbp_period = mode->left_margin * (bpp >> 3);
		hsa_period = mode->hsync_len * (bpp >> 3);
#endif
		break;
	default:
		pr_debug("unsupport traffic mode: %d\n",
			 mipi_dsi->traffic_mode);
		return -EINVAL;
	}
	writel(pixel_fifo_level, mipi_dsi->mmio_base + DPI_PIXEL_FIFO_SEND_LEVEL);

	switch (bpp) {
	case 24:
		color_coding = 5;
		pixel_fmt = 3;
		break;
	case 16:
	case 18:
	default:
		break;
	}
	writel(color_coding, mipi_dsi->mmio_base + DPI_INTERFACE_COLOR_CODING);
	writel(pixel_fmt, mipi_dsi->mmio_base + DPI_PIXEL_FORMAT);
#ifdef CONFIG_FB_IMX64
	writel(0x1, mipi_dsi->mmio_base + DPI_VSYNC_POLARITY);
	writel(0x1, mipi_dsi->mmio_base + DPI_HSYNC_POLARITY);
#else
	writel(0x0, mipi_dsi->mmio_base + DPI_VSYNC_POLARITY);
	writel(0x0, mipi_dsi->mmio_base + DPI_HSYNC_POLARITY);
#endif
	writel(mipi_dsi->traffic_mode,
	       mipi_dsi->mmio_base + DPI_VIDEO_MODE);

	writel(hfp_period, mipi_dsi->mmio_base + DPI_HFP);
	writel(hbp_period, mipi_dsi->mmio_base + DPI_HBP);
	writel(hsa_period, mipi_dsi->mmio_base + DPI_HSA);

	writel(0x0, mipi_dsi->mmio_base + DPI_ENABLE_MULT_PKTS);

	writel(mode->upper_margin, mipi_dsi->mmio_base + DPI_VBP);
	writel(mode->lower_margin, mipi_dsi->mmio_base + DPI_VFP);
	writel(0x1, mipi_dsi->mmio_base + DPI_BLLP_MODE);
	writel(0x0, mipi_dsi->mmio_base + DPI_USE_NULL_PKT_BLLP);

	writel(mode->yres - 1, mipi_dsi->mmio_base + DPI_VACTIVE);

	writel(0x0, mipi_dsi->mmio_base + DPI_VC);

	return 0;
}

static void mipi_dsi_init_interrupt(struct mipi_dsi_info *mipi_dsi)
{
	uint32_t irqs_enable;

	/* disable all the irqs */
	writel(0xffffffff, mipi_dsi->mmio_base + HOST_IRQ_MASK);
	writel(0x7, mipi_dsi->mmio_base + HOST_IRQ_MASK2);

	irqs_enable = ~(HOST_IRQ_MASK_TX_PKT_DONE_MASK |
			HOST_IRQ_MASK_RX_PKT_HDR_RCVD_MASK);

	writel(irqs_enable, mipi_dsi->mmio_base + HOST_IRQ_MASK);
}

static int mipi_display_enter_sleep(struct mxc_dispdrv_handle *disp)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_OFF,
			       NULL, 0);
	if (err)
		return -EINVAL;
	msleep(50);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_ENTER_SLEEP_MODE,
			NULL, 0);
	if (err) {
		dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command sleep in error!\n");
	}
	msleep(MIPI_LCD_SLEEP_MODE_DELAY);

	return err;
}

static int mipi_display_exit_sleep(struct mxc_dispdrv_handle *disp)
{
	int err;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_EXIT_SLEEP_MODE,
			NULL, 0);
	if (err) {
		dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command sleep-out error!\n");
		return err;
	}
	msleep(MIPI_LCD_SLEEP_MODE_DELAY);

	err = mipi_dsi_dcs_cmd(mipi_dsi, MIPI_DCS_SET_DISPLAY_ON,
			NULL, 0);
	msleep(20);

	return err;
}

static void reset_dsi_domains(struct mipi_dsi_info *mipi_dsi, bool reset)
{
#ifdef CONFIG_FB_IMX64
	/* pclk domain */
	regmap_update_bits(mipi_dsi->regmap, SRC_MIPIPHY_RCR,
		MIPI_DSI_PCLK_RESET_N, (reset ? 0 : MIPI_DSI_PCLK_RESET_N));
	/* escape domain */
	regmap_update_bits(mipi_dsi->regmap, SRC_MIPIPHY_RCR,
		MIPI_DSI_ESC_RESET_N, (reset ? 0 : MIPI_DSI_ESC_RESET_N));
	/* byte domain */
	regmap_update_bits(mipi_dsi->regmap, SRC_MIPIPHY_RCR,
		MIPI_DSI_RESET_BYTE_N, (reset ? 0 : MIPI_DSI_RESET_BYTE_N));
	/* dpi domain */
	regmap_update_bits(mipi_dsi->regmap, SRC_MIPIPHY_RCR,
		MIPI_DSI_DPI_RESET_N, (reset ? 0 : MIPI_DSI_DPI_RESET_N));
#else
	/* escape domain */
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			DSI_RST_ESC_N, (reset ? 0 : DSI_RST_ESC_N));
	/* byte domain */
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			DSI_RST_BYTE_N, (reset ? 0 : DSI_RST_BYTE_N));

	/* dpi domain */
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			DSI_RST_DPI_N, (reset ? 0 : DSI_RST_DPI_N));
#endif
}

static int mipi_dsi_enable(struct mxc_dispdrv_handle *disp,
			   struct fb_info *fbi)
{
	int ret;
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

#ifndef CONFIG_FB_IMX64
	if (!mipi_dsi->dsi_power_on)
		pm_runtime_get_sync(&mipi_dsi->pdev->dev);
#endif

	if (!mipi_dsi->lcd_inited) {
#ifdef CONFIG_FB_IMX64
		reset_dsi_domains(mipi_dsi, 0);
#else
		ret = clk_set_rate(mipi_dsi->esc_clk, 80000000);
		if (ret) {
			dev_err(&mipi_dsi->pdev->dev,
					"clk enable error: %d!\n", ret);
			return ret;
		}

		ret = clk_prepare_enable(mipi_dsi->esc_clk);
		if (ret) {
			dev_err(&mipi_dsi->pdev->dev,
			"clk enable error: %d!\n", ret);
			return -EINVAL;
		}
#endif

		if ((ret = mipi_dsi_dphy_init(mipi_dsi)) < 0)
			return ret;

		if ((ret = mipi_dsi_host_init(mipi_dsi)) < 0)
			return ret;

		mipi_dsi_dpi_init(mipi_dsi);

#ifndef CONFIG_FB_IMX64
		reset_dsi_domains(mipi_dsi, 0);

		/* display_en */
		regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
				   DSI_SD, 0x0);
		/* normal cm */
		regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
				   DSI_CM, 0x0);
#endif
		msleep(20);

		if (!mipi_dsi->encoder) {
			ret = device_reset(&mipi_dsi->pdev->dev);
			if (ret) {
				dev_err(&mipi_dsi->pdev->dev,
						"failed to reset device: %d\n", ret);
				return -EINVAL;
			}
			msleep(60);

			mipi_dsi_init_interrupt(mipi_dsi);

			ret = mipi_dsi->lcd_callback->mipi_lcd_setup(mipi_dsi);
			if (ret < 0) {
				dev_err(&mipi_dsi->pdev->dev,
						"failed to init mipi lcd.\n");
				return ret;
			}
			mipi_dsi_set_mode(mipi_dsi, DSI_HS_MODE);
		}

		mipi_dsi->lcd_inited = 1;
	} else {
#ifndef CONFIG_FB_IMX64
		ret = clk_prepare_enable(mipi_dsi->esc_clk);
		if (ret) {
			dev_err(&mipi_dsi->pdev->dev,
				"clk enable error: %d!\n", ret);
			return -EINVAL;
		}
#endif

		reset_dsi_domains(mipi_dsi, 0);

		if (!mipi_dsi->encoder) {
			ret = mipi_display_exit_sleep(mipi_dsi->disp_mipi);
			if (ret) {
				dev_err(&mipi_dsi->pdev->dev, "exit sleep failed\n");
				return -EINVAL;
			}
		}
	}

	return 0;
}

static void mipi_dsi_wr_tx_header(struct mipi_dsi_info *mipi_dsi,
				  u8 di, u8 data0, u8 data1, u8 mode, u8 need_bta)
{
	uint32_t pkt_control = 0;
	uint16_t word_count = 0;

	word_count = data0 | (data1 << 8);
	pkt_control = HOST_PKT_CONTROL_WC(word_count) |
		      HOST_PKT_CONTROL_VC(0)	      |
		      HOST_PKT_CONTROL_DT(di)	      |
		      HOST_PKT_CONTROL_HS_SEL(mode)   |
		      HOST_PKT_CONTROL_BTA_TX(need_bta);

	dev_dbg(&mipi_dsi->pdev->dev, "pkt_control = %x\n", pkt_control);
	writel(pkt_control, mipi_dsi->mmio_base + HOST_PKT_CONTROL);
}

static void mipi_dsi_wr_tx_data(struct mipi_dsi_info *mipi_dsi,
				uint32_t tx_data)
{
	writel(tx_data, mipi_dsi->mmio_base + HOST_TX_PAYLOAD);
}

static void mipi_dsi_long_data_wr(struct mipi_dsi_info *mipi_dsi,
			const uint8_t *data0, uint32_t data_size)
{
	uint32_t data_cnt = 0, payload = 0;

	/* in case that data count is more than 4 */
	for (data_cnt = 0; data_cnt < data_size; data_cnt += 4) {
		/*
		 * after sending 4bytes per one time,
		 * send remainder data less then 4.
		 */
		if ((data_size - data_cnt) < 4) {
			if ((data_size - data_cnt) == 3) {
				payload = data0[data_cnt] |
					  (data0[data_cnt + 1] << 8) |
					  (data0[data_cnt + 2] << 16);
				dev_dbg(&mipi_dsi->pdev->dev, "count = 3 payload = %x, %x %x %x\n",
					payload, data0[data_cnt], data0[data_cnt + 1], data0[data_cnt + 2]);
			} else if ((data_size - data_cnt) == 2) {
				payload = data0[data_cnt] |
					  (data0[data_cnt + 1] << 8);
				dev_dbg(&mipi_dsi->pdev->dev, "count = 2 payload = %x, %x %x\n",
					payload, data0[data_cnt], data0[data_cnt + 1]);
			} else if ((data_size - data_cnt) == 1) {
				payload = data0[data_cnt];
				dev_dbg(&mipi_dsi->pdev->dev, "count = 1 payload = %x, %x\n",
					payload, data0[data_cnt]);
			}

			mipi_dsi_wr_tx_data(mipi_dsi, payload);
		} else {
			payload = data0[data_cnt] |
				  (data0[data_cnt + 1] << 8) |
				  (data0[data_cnt + 2] << 16) |
				  (data0[data_cnt + 3] << 24);

			dev_dbg(&mipi_dsi->pdev->dev,
				"count = 4 payload = %x, %x %x %x %x\n",
                                payload, *(u8 *)(data0 + data_cnt),
                                data0[data_cnt + 1],
                                data0[data_cnt + 2],
                                data0[data_cnt + 3]);

			mipi_dsi_wr_tx_data(mipi_dsi, payload);
		}
	}
}

static int mipi_dsi_pkt_write(struct mipi_dsi_info *mipi_dsi,
			u8 data_type, const u32 *buf, int len)
{
	int ret = 0;
	struct platform_device *pdev = mipi_dsi->pdev;
	const uint8_t *data = (const uint8_t *)buf;

	if (len == 0)
		/* handle generic long write command */
		mipi_dsi_wr_tx_header(mipi_dsi, data_type, data[0], data[1], DSI_LP_MODE, 0);
	else {
		reinit_completion(&dsi_tx_done);

		/* handle generic long write command */
		mipi_dsi_long_data_wr(mipi_dsi, data, len);
		mipi_dsi_wr_tx_header(mipi_dsi, data_type, len & 0xff,
				      (len & 0xff00) >> 8, DSI_LP_MODE, 0);
	}

	/* send packet */
	writel(0x1, mipi_dsi->mmio_base + HOST_SEND_PACKET);
	ret = wait_for_completion_timeout(&dsi_tx_done, MIPI_FIFO_TIMEOUT);

	if (!ret) {
		dev_err(&pdev->dev, "wait tx done timeout!\n");
		return -ETIMEDOUT;
	}
	mdelay(10);

	return 0;
}

static uint32_t mipi_dsi_rd_rx_header(struct mipi_dsi_info *mipi_dsi)
{
	return readl(mipi_dsi->mmio_base + HOST_PKT_RX_PKT_HEADER);
}

static int mipi_dsi_pkt_read(struct mipi_dsi_info *mipi_dsi,
			     uint8_t data_type, uint32_t *buf, int len)
{
	int ret;
	uint32_t rx_hdr;
	struct platform_device *pdev = mipi_dsi->pdev;
	const uint8_t *data = (const uint8_t *)buf;

	if (len <= 4) {
		reinit_completion(&dsi_rx_done);

		mipi_dsi_wr_tx_header(mipi_dsi, data_type, data[0], data[1], DSI_LP_MODE, 1);
		writel(0x1, mipi_dsi->mmio_base + HOST_SEND_PACKET);

		ret = wait_for_completion_timeout(&dsi_rx_done, MIPI_FIFO_TIMEOUT);
		if (!ret) {
			dev_err(&pdev->dev, "wait rx done timeout!\n");
			return -ETIMEDOUT;
		}

		rx_hdr = mipi_dsi_rd_rx_header(mipi_dsi);
		dev_dbg(&pdev->dev, "rx: rx_hdr = 0x%x, data type = 0x%x, word_count = 0x%x\n",
				     rx_hdr, (rx_hdr >> 16) & 0x3f, rx_hdr & 0xffff);

		buf[0] = rx_hdr & 0xff;
	} else {
		/* TODO: add support later */
	}

	return 0;
}

static int mipi_dsi_dcs_cmd(struct mipi_dsi_info *mipi_dsi,
			    u8 cmd, const u32 *param, int num)
{
	int err = 0;
	u32 buf[DSI_CMD_BUF_MAXSIZE];

	switch (cmd) {
	case MIPI_DCS_EXIT_SLEEP_MODE:
	case MIPI_DCS_ENTER_SLEEP_MODE:
	case MIPI_DCS_SET_DISPLAY_ON:
	case MIPI_DCS_SET_DISPLAY_OFF:
		buf[0] = cmd;
		buf[1] = 0x0;
		err = mipi_dsi_pkt_write(mipi_dsi,
				MIPI_DSI_DCS_SHORT_WRITE, buf, 0);
		break;

	default:
		dev_err(&mipi_dsi->pdev->dev,
			"MIPI DSI DCS Command:0x%x Not supported!\n", cmd);
		break;
	}

	return err;
}

static void mipi_dsi_disable(struct mxc_dispdrv_handle *disp,
			     struct fb_info *fbi)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);

	if (!mipi_dsi->encoder)
		mipi_display_enter_sleep(mipi_dsi->disp_mipi);

#ifndef CONFIG_FB_IMX64
	clk_disable_unprepare(mipi_dsi->esc_clk);
#endif
	reset_dsi_domains(mipi_dsi, 1);
#ifdef CONFIG_FB_IMX64
	regmap_update_bits(mipi_dsi->regmap, SRC_MIPIPHY_RCR,
			MIPI_DSI_PCLK_RESET_N, 0x0);
#else
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			   DSI_PLL_EN, 0x0);
#endif
}

static int mipi_dsi_setup(struct mxc_dispdrv_handle *disp,
			  struct fb_info *fbi)
{
	struct mipi_dsi_info *mipi_dsi = mxc_dispdrv_getdata(disp);
	int xres_virtual = fbi->var.xres_virtual;
	int yres_virtual = fbi->var.yres_virtual;
	int xoffset = fbi->var.xoffset;
	int yoffset = fbi->var.yoffset;
	int pixclock = fbi->var.pixclock;

	if (!mipi_dsi->mode)
		return 0;

	/* set the mode back to var in case userspace changes it */
	fb_videomode_to_var(&fbi->var, mipi_dsi->mode);

	/* restore some var entries cached */
	fbi->var.xres_virtual = xres_virtual;
	fbi->var.yres_virtual = yres_virtual;
	fbi->var.xoffset = xoffset;
	fbi->var.yoffset = yoffset;
	fbi->var.pixclock = pixclock;

	return 0;
}

static struct mxc_dispdrv_driver mipi_dsi_drv = {
	.name = DISPDRV_MIPI,
	.init = mipi_dsi_disp_init,
	.deinit = mipi_dsi_disp_deinit,
	.enable = mipi_dsi_enable,
	.disable = mipi_dsi_disable,
	.setup = mipi_dsi_setup,
};

static irqreturn_t mipi_dsi_irq_handler(int irq, void *data)
{
	uint32_t irq_status;
	struct mipi_dsi_info *mipi_dsi = data;
	struct platform_device *pdev = mipi_dsi->pdev;

	irq_status = readl(mipi_dsi->mmio_base + HOST_IRQ_STATUS);

	dev_dbg(&pdev->dev, "irq_status = 0x%x\n", irq_status);

	if (irq_status & HOST_IRQ_STATUS_TX_PKT_DONE) {
		dev_dbg(&pdev->dev, "payload tx finished\n");
		complete(&dsi_tx_done);
	}

	if (irq_status & HOST_IRQ_STATUS_RX_PKT_HDR_RCVD) {
		dev_dbg(&pdev->dev, "rx data finished\n");
		complete(&dsi_rx_done);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_FB_IMX64
static int dsi_clks_init(struct mipi_dsi_info *minfo)
{
	int ret = 0;
	struct platform_device *pdev = minfo->pdev;
	struct device_node *np = pdev->dev.of_node;

	minfo->core_clk = devm_clk_get(&pdev->dev, "core");
	BUG_ON(IS_ERR(minfo->core_clk));

	minfo->phy_ref_clk = devm_clk_get(&pdev->dev, "phy_ref");
	BUG_ON(IS_ERR(minfo->phy_ref_clk));

	minfo->rxesc_clk = devm_clk_get(&pdev->dev, "rxesc");
	BUG_ON(IS_ERR(minfo->rxesc_clk));

	minfo->txesc_clk = devm_clk_get(&pdev->dev, "txesc");
	BUG_ON(IS_ERR(minfo->txesc_clk));

	minfo->dbi_clk = devm_clk_get(&pdev->dev, "dbi");
	BUG_ON(IS_ERR(minfo->dbi_clk));


	ret = clk_set_rate(minfo->phy_ref_clk, minfo->phy_ref_clkfreq);
	if (ret < 0) {
		dev_err(&pdev->dev, "set phy_ref clock rate failed\n");
		goto out;
	}

	ret = clk_set_rate(minfo->rxesc_clk, 80000000);
	if (ret < 0) {
		dev_err(&pdev->dev, "set rxesc clock rate failed\n");
		goto out;
	}

	ret = clk_set_rate(minfo->txesc_clk, 20000000);
	if (ret < 0) {
		dev_err(&pdev->dev, "set txesc clock rate failed\n");
		goto out;
	}

	clk_prepare_enable(minfo->core_clk);
	clk_prepare_enable(minfo->phy_ref_clk);
	clk_prepare_enable(minfo->rxesc_clk);
	clk_prepare_enable(minfo->txesc_clk);
	/* TODO: dbi clk is not used yet */

out:
	return ret;
}
#endif

/**
 * This function is called by the driver framework to initialize the MIPI DSI
 * device.
 *
 * @param	pdev	The device structure for the MIPI DSI passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int mipi_dsi_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mipi_dsi_info *mipi_dsi;
	struct device_node *endpoint = NULL, *remote;
	struct resource *res;
	const char *lcd_panel;
	int ret = 0;
	u32 vmode_index;
	uint32_t phy_ref_clkfreq;

	mipi_dsi = devm_kzalloc(&pdev->dev, sizeof(*mipi_dsi), GFP_KERNEL);
	if (!mipi_dsi)
		return -ENOMEM;
	mipi_dsi->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "failed to get platform mem resource\n");
		return -ENOMEM;
	}

	mipi_dsi->mmio_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mipi_dsi->mmio_base))
		return -ENODEV;

	mipi_dsi->irq = platform_get_irq(pdev, 0);
	if (mipi_dsi->irq < 0) {
		dev_err(&pdev->dev, "failed to get device irq\n");
		return -EINVAL;
	}

	ret = devm_request_irq(&pdev->dev, mipi_dsi->irq,
				mipi_dsi_irq_handler,
				0, "mipi_dsi_northwest", mipi_dsi);
	if (ret) {
		dev_err(&pdev->dev, "failed to request mipi dsi irq\n");
		return ret;
	}

	ret = of_property_read_u32(np, "phy-ref-clkfreq",
				   &phy_ref_clkfreq);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to get phy reference clock rate\n");
		return -EINVAL;
	}

	if (phy_ref_clkfreq < 24000000 || phy_ref_clkfreq > 200000000) {
		dev_err(&pdev->dev, "invalid phy reference clock rate\n");
		return -EINVAL;
	}
	mipi_dsi->phy_ref_clkfreq = phy_ref_clkfreq;

#ifdef CONFIG_FB_IMX64
	dsi_clks_init(mipi_dsi);

	mipi_dsi->regmap = syscon_regmap_lookup_by_phandle(np, "reset");
	if (IS_ERR(mipi_dsi->regmap)) {
		dev_err(&pdev->dev, "failed to get reset regmap\n");
		return -EINVAL;
	}

	mipi_dsi->mux_sel = syscon_regmap_lookup_by_phandle(np, "mux-sel");
	if (IS_ERR(mipi_dsi->mux_sel)) {
		dev_err(&pdev->dev, "failed to get mux_sel regmap\n");
		return -EINVAL;
	}

	/* TODO: use lcdif for source */
	regmap_update_bits(mipi_dsi->mux_sel, IOMUXC_GPR_GPR13,
			GPR_MIPI_MUX_SEL, 0x0);

#else
	mipi_dsi->esc_clk = devm_clk_get(&pdev->dev, "mipi_dsi_clk");
	if (IS_ERR(mipi_dsi->esc_clk)) {
		dev_err(&pdev->dev, "failed to get esc clk\n");
		return PTR_ERR(mipi_dsi->esc_clk);
	}

	mipi_dsi->regmap = syscon_regmap_lookup_by_phandle(np, "sim");
	if (IS_ERR(mipi_dsi->regmap)) {
		dev_err(&pdev->dev, "failed to get parent regmap\n");
		return -EINVAL;
	}
#endif
	/* check whether an encoder exists */
	endpoint = of_graph_get_next_endpoint(np, NULL);
	if (endpoint) {
		remote = of_graph_get_remote_port_parent(endpoint);
		if (!remote)
			return -EINVAL;

		ret = of_property_read_u32(remote, "video-mode", &vmode_index);
		if ((ret < 0) || (vmode_index >= ARRAY_SIZE(mxc_cea_mode)))
			return -EINVAL;
		mipi_dsi->vmode_index = vmode_index;

		mipi_dsi->mode = devm_kzalloc(&pdev->dev,
					      sizeof(struct fb_videomode),
					      GFP_KERNEL);
		if (!mipi_dsi->mode)
			return -ENOMEM;

		memcpy(mipi_dsi->mode, &mxc_cea_mode[vmode_index],
		       sizeof(struct fb_videomode));

		ret = of_property_read_u32(remote, "dsi-traffic-mode",
					   &mipi_dsi->traffic_mode);
		if (ret < 0 || mipi_dsi->traffic_mode > 2) {
			devm_kfree(&pdev->dev, mipi_dsi->mode);
			return -EINVAL;
		}

		mipi_dsi->lcd_config = devm_kzalloc(&pdev->dev,
						sizeof(struct mipi_lcd_config),
						GFP_KERNEL);
		if (!mipi_dsi->lcd_config) {
			kfree(mipi_dsi->mode);
			return -ENOMEM;
		}

		mipi_dsi->encoder = 1;
	} else {
		/* Default, using 'BURST-MODE' for mipi panel */
		mipi_dsi->traffic_mode = 2;

		ret = of_property_read_string(np, "lcd_panel", &lcd_panel);
		if (ret) {
			dev_err(&pdev->dev, "failed to read lcd_panel property\n");
			return ret;
		}

		/* mipi VDDA is sw1 in PMIC which is always on */

		mipi_dsi->lcd_panel = kstrdup(lcd_panel, GFP_KERNEL);
		if (!mipi_dsi->lcd_panel) {
			dev_err(&pdev->dev, "failed to allocate lcd panel\n");
			ret = -ENOMEM;
		}
	}

	mipi_dsi->disp_mipi = mxc_dispdrv_register(&mipi_dsi_drv);
	if (IS_ERR(mipi_dsi->disp_mipi)) {
		dev_err(&pdev->dev, "mxc_dispdrv_register error\n");
		ret = PTR_ERR(mipi_dsi->disp_mipi);
		goto dispdrv_reg_fail;
	}

	mipi_dsi->mipi_dsi_pkt_read  = mipi_dsi_pkt_read;
	mipi_dsi->mipi_dsi_pkt_write = mipi_dsi_pkt_write;
	mipi_dsi->mipi_dsi_dcs_cmd   = mipi_dsi_dcs_cmd;

#ifndef CONFIG_FB_IMX64
	pm_runtime_enable(&pdev->dev);
#endif
	mxc_dispdrv_setdata(mipi_dsi->disp_mipi, mipi_dsi);
	dev_set_drvdata(&pdev->dev, mipi_dsi);

	dev_info(&pdev->dev, "i.MX MIPI DSI driver probed\n");
	return ret;

dispdrv_reg_fail:
	if (mipi_dsi->lcd_panel)
		kfree(mipi_dsi->lcd_panel);
	return ret;
}

static int mipi_dsi_remove(struct platform_device *pdev)
{
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	mxc_dispdrv_puthandle(mipi_dsi->disp_mipi);
	mxc_dispdrv_unregister(mipi_dsi->disp_mipi);

	kfree(mipi_dsi->lcd_panel);
	dev_set_drvdata(&pdev->dev, NULL);

	return 0;
}

static void mipi_dsi_shutdown(struct platform_device *pdev)
{
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	if (mipi_dsi->lcd_inited) {
#ifndef CONFIG_FB_IMX64
		clk_prepare_enable(mipi_dsi->esc_clk);
#endif
		if (!mipi_dsi->encoder)
			mipi_display_enter_sleep(mipi_dsi->disp_mipi);

		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_PLL);
		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_DPHY);
#ifndef CONFIG_FB_IMX64
		clk_disable_unprepare(mipi_dsi->esc_clk);
#endif
		mipi_dsi->lcd_inited = 0;
	}

	reset_dsi_domains(mipi_dsi, 1);

#ifdef CONFIG_FB_IMX64
	regmap_update_bits(mipi_dsi->regmap, SRC_MIPIPHY_RCR,
				   MIPI_DSI_PCLK_RESET_N, 0x0);
#else
	regmap_update_bits(mipi_dsi->regmap, SIM_SOPT1CFG,
			   DSI_PLL_EN, 0x0);
#endif
}

static const struct of_device_id imx_mipi_dsi_dt_ids[] = {
	{ .compatible = "fsl,imx7ulp-mipi-dsi", .data = NULL, },
	{ .compatible = "fsl,imx8mq-mipi-dsi", .data = NULL, },
	{ }
};
MODULE_DEVICE_TABLE(of, imx_mipi_dsi_dt_ids);

static int mipi_dsi_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	mipi_dsi->dsi_power_on = 0;

	return 0;
}

static int mipi_dsi_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	if (!mipi_dsi->dsi_power_on) {
		request_bus_freq(BUS_FREQ_HIGH);
		dev_dbg(dev, "mipi dsi busfreq high request.\n");

		mipi_dsi->dsi_power_on = 1;
	}

	return 0;
}

static int mipi_dsi_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mipi_dsi_info *mipi_dsi = dev_get_drvdata(&pdev->dev);

	if (unlikely(mipi_dsi->lcd_inited)) {
#ifndef CONFIG_FB_IMX64
		clk_prepare_enable(mipi_dsi->esc_clk);
#endif

		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_PLL);
		writel(0x1, mipi_dsi->mmio_base + DPHY_PD_DPHY);

#ifndef CONFIG_FB_IMX64
		clk_disable_unprepare(mipi_dsi->esc_clk);
#endif
		mipi_dsi->lcd_inited = 0;
	}

	pinctrl_pm_select_sleep_state(dev);

	return 0;
}

static int mipi_dsi_resume(struct device *dev)
{
	pinctrl_pm_select_default_state(dev);

	return 0;
}

static const struct dev_pm_ops mipi_dsi_pm_ops = {
	.suspend = mipi_dsi_suspend,
	.resume  = mipi_dsi_resume,
	.runtime_suspend = mipi_dsi_runtime_suspend,
	.runtime_resume  = mipi_dsi_runtime_resume,
	.runtime_idle	 = NULL,
};

static struct platform_driver mipi_dsi_driver = {
	.driver = {
		.of_match_table = imx_mipi_dsi_dt_ids,
		.name = "mipi_dsi_northwest",
		.pm = &mipi_dsi_pm_ops,
	},
	.probe  = mipi_dsi_probe,
	.remove = mipi_dsi_remove,
	.shutdown = mipi_dsi_shutdown,
};

static int __init mipi_dsi_init(void)
{
	int err;

	err = platform_driver_register(&mipi_dsi_driver);
	if (err) {
		pr_err("mipi_dsi_driver register failed\n");
		return err;
	}

	pr_debug("MIPI DSI driver module loaded: %s\n", mipi_dsi_driver.driver.name);

	return 0;
}

static void __exit mipi_dsi_exit(void)
{
	platform_driver_unregister(&mipi_dsi_driver);
}

module_init(mipi_dsi_init);
module_exit(mipi_dsi_exit);

MODULE_AUTHOR("NXP Semiconductor, Inc.");
MODULE_DESCRIPTION("i.MX MIPI DSI driver");
MODULE_LICENSE("GPL");
