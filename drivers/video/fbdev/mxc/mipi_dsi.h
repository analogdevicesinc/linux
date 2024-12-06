/*
 * Copyright (C) 2011-2016 Freescale Semiconductor, Inc. All Rights Reserved.
 * Copyright 2017 NXP.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
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

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */
#ifndef __MIPI_DSI_H__
#define __MIPI_DSI_H__

#include <linux/regmap.h>
#include "mxc_dispdrv.h"

#ifdef DEBUG
#define mipi_dbg(fmt, ...) printk(KERN_DEBUG pr_fmt(fmt), ##__VA_ARGS__)
#else
#define mipi_dbg(fmt, ...)
#endif

#define	DSI_CMD_BUF_MAXSIZE         (128)

#define DSI_NON_BURST_WITH_SYNC_PULSE  0
#define DSI_NON_BURST_WITH_SYNC_EVENT  1
#define DSI_BURST_MODE                 2

#define DSI_HSA_PKT_OVERHEAD	10
#define DSI_HFP_PKT_OVERHEAD	8
#define DSI_HBP_PKT_OVERHEAD	14

/* DPI interface pixel color coding map */
enum mipi_dsi_dpi_fmt {
	MIPI_RGB565_PACKED = 0,
	MIPI_RGB565_LOOSELY,
	MIPI_RGB565_CONFIG3,
	MIPI_RGB666_PACKED,
	MIPI_RGB666_LOOSELY,
	MIPI_RGB888,
};

struct mipi_lcd_config {
	u32				virtual_ch;
	u32				data_lane_num;
	/* device max DPHY clock in MHz unit */
	u32				max_phy_clk;
	enum mipi_dsi_dpi_fmt		dpi_fmt;
};

struct mipi_dsi_info;
struct mipi_dsi_lcd_callback {
	/* callback for lcd panel operation */
	void (*get_mipi_lcd_videomode)(struct fb_videomode **, int *,
			struct mipi_lcd_config **);
	int  (*mipi_lcd_setup)(struct mipi_dsi_info *);

};

struct mipi_dsi_match_lcd {
	char *lcd_panel;
	struct mipi_dsi_lcd_callback lcd_callback;
};

struct mipi_dsi_bus_mux {
	int reg;
	int mask;
	int (*get_mux) (int dev_id, int disp_id);
};

/* driver private data */
struct mipi_dsi_info {
	struct platform_device		*pdev;
	void __iomem			*mmio_base;
	void __iomem			*phy_base;
	struct regmap			*regmap;
	struct regmap			*mux_sel;
	const struct mipi_dsi_bus_mux	*bus_mux;
	int				dsi_power_on;
	int				lcd_inited;
	int				encoder;
	int				traffic_mode;
	u32				dphy_pll_config;
	int				dev_id;
	int				disp_id;
	char				*lcd_panel;
	int				irq;
	uint32_t			phy_ref_clkfreq;
#ifdef CONFIG_FB_IMX64
	struct clk			*core_clk;
	struct clk			*phy_ref_clk;
	struct clk			*dbi_clk;
	struct clk			*rxesc_clk;
	struct clk			*txesc_clk;
#else
	struct clk			*dphy_clk;
	struct clk			*cfg_clk;
	struct clk			*esc_clk;
#endif
	struct mxc_dispdrv_handle	*disp_mipi;
	int				vmode_index;
	struct  fb_videomode		*mode;
	struct regulator		*disp_power_on;
	struct  mipi_lcd_config		*lcd_config;
	/* board related power control */
	struct backlight_device		*bl;
	/* callback for lcd panel operation */
	struct mipi_dsi_lcd_callback	*lcd_callback;

	int (*mipi_dsi_pkt_read)(struct mipi_dsi_info *mipi,
			u8 data_type, u32 *buf, int len);
	int (*mipi_dsi_pkt_write)(struct mipi_dsi_info *mipi_dsi,
			u8 data_type, const u32 *buf, int len);
	int (*mipi_dsi_dcs_cmd)(struct mipi_dsi_info *mipi,
			u8 cmd, const u32 *param, int num);
};

#ifdef CONFIG_FB_MXC_TRULY_WVGA_SYNC_PANEL
void mipid_hx8369_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data);
int mipid_hx8369_lcd_setup(struct mipi_dsi_info *);
#endif
#ifdef CONFIG_FB_MXC_TRULY_PANEL_TFT3P5079E
void mipid_otm8018b_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data);
int mipid_otm8018b_lcd_setup(struct mipi_dsi_info *);
#endif
#ifdef CONFIG_FB_MXC_TRULY_PANEL_TFT3P5581E
void mipid_hx8363_get_lcd_videomode(struct fb_videomode **mode, int *size,
		struct mipi_lcd_config **data);
int mipid_hx8363_lcd_setup(struct mipi_dsi_info *);
#endif
#ifdef CONFIG_FB_MXC_RK_PANEL_RK055AHD042
void mipid_rm68200_get_lcd_videomode(struct fb_videomode **mode, int *size,
                                     struct mipi_lcd_config **data);
int mipid_rm68200_lcd_setup(struct mipi_dsi_info *mipi_dsi);
#endif
#ifdef CONFIG_FB_MXC_RK_PANEL_RK055IQH042
void mipid_rm68191_get_lcd_videomode(struct fb_videomode **mode, int *size,
                                     struct mipi_lcd_config **data);
int mipid_rm68191_lcd_setup(struct mipi_dsi_info *mipi_dsi);
#endif

#endif
