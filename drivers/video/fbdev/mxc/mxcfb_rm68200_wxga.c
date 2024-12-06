/*
 * Copyright 2018 NXP
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
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <linux/spinlock.h>
#include <linux/mipi_dsi.h>
#include <linux/mxcfb.h>
#include <linux/backlight.h>
#include <video/mipi_display.h>

#include "mipi_dsi.h"

#define RM68200_MAX_DPHY_CLK					(850)

#define CHECK_RETCODE(ret)					\
do {								\
	if (ret < 0) {						\
		dev_err(&mipi_dsi->pdev->dev,			\
			"%s ERR: ret:%d, line:%d.\n",		\
			__func__, ret, __LINE__);		\
		return ret;					\
	}							\
} while (0)

static void parse_variadic(int n, u8 *buf, ...)
{
	int i = 0;
	va_list args;

	if (unlikely(!n)) return;

	va_start(args, buf);

	for (i = 0; i < n; i++)
		buf[i + 1] = (u8)va_arg(args, int);

	va_end(args);
}

#define RM68200_DCS_write_1A_nP(n, addr, ...) {			\
	int err;						\
								\
	buf[0] = addr;						\
	parse_variadic(n, buf, ##__VA_ARGS__);			\
								\
	if (n >= 2)						\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,		\
			MIPI_DSI_DCS_LONG_WRITE, (u32*)buf, n + 1);	\
	else if (n == 1)					\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,	\
			MIPI_DSI_DCS_SHORT_WRITE_PARAM, (u32*)buf, 0);	\
	else if (n == 0)					\
	{							\
		buf[1] = 0;					\
		err = mipi_dsi->mipi_dsi_pkt_write(mipi_dsi,	\
			MIPI_DSI_DCS_SHORT_WRITE, (u32*)buf, 0);	\
	}							\
	CHECK_RETCODE(err);					\
}

#define RM68200_DCS_write_1A_0P(addr)			\
	RM68200_DCS_write_1A_nP(0, addr)

#define RM68200_DCS_write_1A_1P(addr, ...)		\
	RM68200_DCS_write_1A_nP(1, addr, __VA_ARGS__)

#define ACTIVE_HIGH_NAME	"RK-WXGA-SYNC-HIGH"
#define ACTIVE_LOW_NAME		"RK-WXGA-SYNC-LOW"

static struct fb_videomode rk_lcd_modedb[] = {
	/* 720 x 1280 */
	{
		ACTIVE_HIGH_NAME, 60, 720, 1280, 16040,
		32, 32,
		14, 16,
		8, 2,
		0x0,
		FB_VMODE_NONINTERLACED,
		0,
	}, {
		ACTIVE_LOW_NAME, 60, 720, 1280, 16040,
		32, 32,
		14, 16,
		8, 2,
		FB_SYNC_OE_LOW_ACT,
		FB_VMODE_NONINTERLACED,
		0,
	},
};

static struct mipi_lcd_config lcd_config = {
	.virtual_ch	= 0x0,
	.data_lane_num  = 2,
	.max_phy_clk    = RM68200_MAX_DPHY_CLK,
	.dpi_fmt	= MIPI_RGB888,
};

void mipid_rm68200_get_lcd_videomode(struct fb_videomode **mode, int *size,
				     struct mipi_lcd_config **data)
{
	*mode = &rk_lcd_modedb[0];
	*size = ARRAY_SIZE(rk_lcd_modedb);
	*data = &lcd_config;
}

int mipid_rm68200_lcd_setup(struct mipi_dsi_info *mipi_dsi)
{
	u8 buf[DSI_CMD_BUF_MAXSIZE];

	dev_dbg(&mipi_dsi->pdev->dev, "MIPI DSI LCD RM68200 setup.\n");

	/* change to MCS Page 0 */
	RM68200_DCS_write_1A_1P(0xFE, 0x01);
	RM68200_DCS_write_1A_1P(0x24, 0xc0);	/* External PWR IC Control */
	RM68200_DCS_write_1A_1P(0x25, 0x53);
	RM68200_DCS_write_1A_1P(0x26, 0x00);
	RM68200_DCS_write_1A_1P(0x2B, 0xE5);
	RM68200_DCS_write_1A_1P(0x27, 0x0A);
	RM68200_DCS_write_1A_1P(0x29, 0x0A);
	RM68200_DCS_write_1A_1P(0x16, 0x52);
	RM68200_DCS_write_1A_1P(0x2F, 0x53);
	RM68200_DCS_write_1A_1P(0x34, 0x5A);
	RM68200_DCS_write_1A_1P(0x1B, 0x00);
	RM68200_DCS_write_1A_1P(0x12, 0x0A);
	RM68200_DCS_write_1A_1P(0x1A, 0x06);
	RM68200_DCS_write_1A_1P(0x46, 0x56);
	RM68200_DCS_write_1A_1P(0x52, 0xA0);
	RM68200_DCS_write_1A_1P(0x53, 0x00);
	RM68200_DCS_write_1A_1P(0x54, 0xA0);
	RM68200_DCS_write_1A_1P(0x55, 0x00);
	RM68200_DCS_write_1A_1P(0x5F, 0x11);	/* 2 data lanes */

	/* change to MCS Page 2 */
	RM68200_DCS_write_1A_1P(0xFE, 0x03);
	RM68200_DCS_write_1A_1P(0x00, 0x05);
	RM68200_DCS_write_1A_1P(0x02, 0x0B);
	RM68200_DCS_write_1A_1P(0x03, 0x0F);
	RM68200_DCS_write_1A_1P(0x04, 0x7D);
	RM68200_DCS_write_1A_1P(0x05, 0x00);
	RM68200_DCS_write_1A_1P(0x06, 0x50);
	RM68200_DCS_write_1A_1P(0x07, 0x05);
	RM68200_DCS_write_1A_1P(0x08, 0x16);
	RM68200_DCS_write_1A_1P(0x09, 0x0D);
	RM68200_DCS_write_1A_1P(0x0A, 0x11);
	RM68200_DCS_write_1A_1P(0x0B, 0x7D);
	RM68200_DCS_write_1A_1P(0x0C, 0x00);
	RM68200_DCS_write_1A_1P(0x0D, 0x50);
	RM68200_DCS_write_1A_1P(0x0E, 0x07);
	RM68200_DCS_write_1A_1P(0x0F, 0x08);
	RM68200_DCS_write_1A_1P(0x10, 0x01);
	RM68200_DCS_write_1A_1P(0x11, 0x02);
	RM68200_DCS_write_1A_1P(0x12, 0x00);
	RM68200_DCS_write_1A_1P(0x13, 0x7D);
	RM68200_DCS_write_1A_1P(0x14, 0x00);
	RM68200_DCS_write_1A_1P(0x15, 0x85);
	RM68200_DCS_write_1A_1P(0x16, 0x08);
	RM68200_DCS_write_1A_1P(0x17, 0x03);
	RM68200_DCS_write_1A_1P(0x18, 0x04);
	RM68200_DCS_write_1A_1P(0x19, 0x05);
	RM68200_DCS_write_1A_1P(0x1A, 0x06);
	RM68200_DCS_write_1A_1P(0x1B, 0x00);
	RM68200_DCS_write_1A_1P(0x1C, 0x7D);
	RM68200_DCS_write_1A_1P(0x1D, 0x00);
	RM68200_DCS_write_1A_1P(0x1E, 0x85);
	RM68200_DCS_write_1A_1P(0x1F, 0x08);
	RM68200_DCS_write_1A_1P(0x20, 0x00);
	RM68200_DCS_write_1A_1P(0x21, 0x00);
	RM68200_DCS_write_1A_1P(0x22, 0x00);
	RM68200_DCS_write_1A_1P(0x23, 0x00);
	RM68200_DCS_write_1A_1P(0x24, 0x00);
	RM68200_DCS_write_1A_1P(0x25, 0x00);
	RM68200_DCS_write_1A_1P(0x26, 0x00);
	RM68200_DCS_write_1A_1P(0x27, 0x00);
	RM68200_DCS_write_1A_1P(0x28, 0x00);
	RM68200_DCS_write_1A_1P(0x29, 0x00);
	RM68200_DCS_write_1A_1P(0x2A, 0x07);
	RM68200_DCS_write_1A_1P(0x2B, 0x08);
	RM68200_DCS_write_1A_1P(0x2D, 0x01);
	RM68200_DCS_write_1A_1P(0x2F, 0x02);
	RM68200_DCS_write_1A_1P(0x30, 0x00);
	RM68200_DCS_write_1A_1P(0x31, 0x40);
	RM68200_DCS_write_1A_1P(0x32, 0x05);
	RM68200_DCS_write_1A_1P(0x33, 0x08);
	RM68200_DCS_write_1A_1P(0x34, 0x54);
	RM68200_DCS_write_1A_1P(0x35, 0x7D);
	RM68200_DCS_write_1A_1P(0x36, 0x00);
	RM68200_DCS_write_1A_1P(0x37, 0x03);
	RM68200_DCS_write_1A_1P(0x38, 0x04);
	RM68200_DCS_write_1A_1P(0x39, 0x05);
	RM68200_DCS_write_1A_1P(0x3A, 0x06);
	RM68200_DCS_write_1A_1P(0x3B, 0x00);
	RM68200_DCS_write_1A_1P(0x3D, 0x40);
	RM68200_DCS_write_1A_1P(0x3F, 0x05);
	RM68200_DCS_write_1A_1P(0x40, 0x08);
	RM68200_DCS_write_1A_1P(0x41, 0x54);
	RM68200_DCS_write_1A_1P(0x42, 0x7D);
	RM68200_DCS_write_1A_1P(0x43, 0x00);
	RM68200_DCS_write_1A_1P(0x44, 0x00);
	RM68200_DCS_write_1A_1P(0x45, 0x00);
	RM68200_DCS_write_1A_1P(0x46, 0x00);
	RM68200_DCS_write_1A_1P(0x47, 0x00);
	RM68200_DCS_write_1A_1P(0x48, 0x00);
	RM68200_DCS_write_1A_1P(0x49, 0x00);
	RM68200_DCS_write_1A_1P(0x4A, 0x00);
	RM68200_DCS_write_1A_1P(0x4B, 0x00);
	RM68200_DCS_write_1A_1P(0x4C, 0x00);
	RM68200_DCS_write_1A_1P(0x4D, 0x00);
	RM68200_DCS_write_1A_1P(0x4E, 0x00);
	RM68200_DCS_write_1A_1P(0x4F, 0x00);
	RM68200_DCS_write_1A_1P(0x50, 0x00);
	RM68200_DCS_write_1A_1P(0x51, 0x00);
	RM68200_DCS_write_1A_1P(0x52, 0x00);
	RM68200_DCS_write_1A_1P(0x53, 0x00);
	RM68200_DCS_write_1A_1P(0x54, 0x00);
	RM68200_DCS_write_1A_1P(0x55, 0x00);
	RM68200_DCS_write_1A_1P(0x56, 0x00);
	RM68200_DCS_write_1A_1P(0x58, 0x00);
	RM68200_DCS_write_1A_1P(0x59, 0x00);
	RM68200_DCS_write_1A_1P(0x5A, 0x00);
	RM68200_DCS_write_1A_1P(0x5B, 0x00);
	RM68200_DCS_write_1A_1P(0x5C, 0x00);
	RM68200_DCS_write_1A_1P(0x5D, 0x00);
	RM68200_DCS_write_1A_1P(0x5E, 0x00);
	RM68200_DCS_write_1A_1P(0x5F, 0x00);
	RM68200_DCS_write_1A_1P(0x60, 0x00);
	RM68200_DCS_write_1A_1P(0x61, 0x00);
	RM68200_DCS_write_1A_1P(0x62, 0x00);
	RM68200_DCS_write_1A_1P(0x63, 0x00);
	RM68200_DCS_write_1A_1P(0x64, 0x00);
	RM68200_DCS_write_1A_1P(0x65, 0x00);
	RM68200_DCS_write_1A_1P(0x66, 0x00);
	RM68200_DCS_write_1A_1P(0x67, 0x00);
	RM68200_DCS_write_1A_1P(0x68, 0x00);
	RM68200_DCS_write_1A_1P(0x69, 0x00);
	RM68200_DCS_write_1A_1P(0x6A, 0x00);
	RM68200_DCS_write_1A_1P(0x6B, 0x00);
	RM68200_DCS_write_1A_1P(0x6C, 0x00);
	RM68200_DCS_write_1A_1P(0x6D, 0x00);
	RM68200_DCS_write_1A_1P(0x6E, 0x00);
	RM68200_DCS_write_1A_1P(0x6F, 0x00);
	RM68200_DCS_write_1A_1P(0x70, 0x00);
	RM68200_DCS_write_1A_1P(0x71, 0x00);
	RM68200_DCS_write_1A_1P(0x72, 0x20);
	RM68200_DCS_write_1A_1P(0x73, 0x00);
	RM68200_DCS_write_1A_1P(0x74, 0x08);
	RM68200_DCS_write_1A_1P(0x75, 0x08);
	RM68200_DCS_write_1A_1P(0x76, 0x08);
	RM68200_DCS_write_1A_1P(0x77, 0x08);
	RM68200_DCS_write_1A_1P(0x78, 0x08);
	RM68200_DCS_write_1A_1P(0x79, 0x08);
	RM68200_DCS_write_1A_1P(0x7A, 0x00);
	RM68200_DCS_write_1A_1P(0x7B, 0x00);
	RM68200_DCS_write_1A_1P(0x7C, 0x00);
	RM68200_DCS_write_1A_1P(0x7D, 0x00);
	RM68200_DCS_write_1A_1P(0x7E, 0xBF);
	RM68200_DCS_write_1A_1P(0x7F, 0x02);
	RM68200_DCS_write_1A_1P(0x80, 0x06);
	RM68200_DCS_write_1A_1P(0x81, 0x14);
	RM68200_DCS_write_1A_1P(0x82, 0x10);
	RM68200_DCS_write_1A_1P(0x83, 0x16);
	RM68200_DCS_write_1A_1P(0x84, 0x12);
	RM68200_DCS_write_1A_1P(0x85, 0x08);
	RM68200_DCS_write_1A_1P(0x86, 0x3F);
	RM68200_DCS_write_1A_1P(0x87, 0x3F);
	RM68200_DCS_write_1A_1P(0x88, 0x3F);
	RM68200_DCS_write_1A_1P(0x89, 0x3F);
	RM68200_DCS_write_1A_1P(0x8A, 0x3F);
	RM68200_DCS_write_1A_1P(0x8B, 0x0C);
	RM68200_DCS_write_1A_1P(0x8C, 0x0A);
	RM68200_DCS_write_1A_1P(0x8D, 0x0E);
	RM68200_DCS_write_1A_1P(0x8E, 0x3F);
	RM68200_DCS_write_1A_1P(0x8F, 0x3F);
	RM68200_DCS_write_1A_1P(0x90, 0x00);
	RM68200_DCS_write_1A_1P(0x91, 0x04);
	RM68200_DCS_write_1A_1P(0x92, 0x3F);
	RM68200_DCS_write_1A_1P(0x93, 0x3F);
	RM68200_DCS_write_1A_1P(0x94, 0x3F);
	RM68200_DCS_write_1A_1P(0x95, 0x3F);
	RM68200_DCS_write_1A_1P(0x96, 0x05);
	RM68200_DCS_write_1A_1P(0x97, 0x01);
	RM68200_DCS_write_1A_1P(0x98, 0x3F);
	RM68200_DCS_write_1A_1P(0x99, 0x3F);
	RM68200_DCS_write_1A_1P(0x9A, 0x0F);
	RM68200_DCS_write_1A_1P(0x9B, 0x0B);
	RM68200_DCS_write_1A_1P(0x9C, 0x0D);
	RM68200_DCS_write_1A_1P(0x9D, 0x3F);
	RM68200_DCS_write_1A_1P(0x9E, 0x3F);
	RM68200_DCS_write_1A_1P(0x9F, 0x3F);
	RM68200_DCS_write_1A_1P(0xA0, 0x3F);
	RM68200_DCS_write_1A_1P(0xA2, 0x3F);
	RM68200_DCS_write_1A_1P(0xA3, 0x09);
	RM68200_DCS_write_1A_1P(0xA4, 0x13);
	RM68200_DCS_write_1A_1P(0xA5, 0x17);
	RM68200_DCS_write_1A_1P(0xA6, 0x11);
	RM68200_DCS_write_1A_1P(0xA7, 0x15);
	RM68200_DCS_write_1A_1P(0xA9, 0x07);
	RM68200_DCS_write_1A_1P(0xAA, 0x03);
	RM68200_DCS_write_1A_1P(0xAB, 0x3F);
	RM68200_DCS_write_1A_1P(0xAC, 0x3F);
	RM68200_DCS_write_1A_1P(0xAD, 0x05);
	RM68200_DCS_write_1A_1P(0xAE, 0x01);
	RM68200_DCS_write_1A_1P(0xAF, 0x17);
	RM68200_DCS_write_1A_1P(0xB0, 0x13);
	RM68200_DCS_write_1A_1P(0xB1, 0x15);
	RM68200_DCS_write_1A_1P(0xB2, 0x11);
	RM68200_DCS_write_1A_1P(0xB3, 0x0F);
	RM68200_DCS_write_1A_1P(0xB4, 0x3F);
	RM68200_DCS_write_1A_1P(0xB5, 0x3F);
	RM68200_DCS_write_1A_1P(0xB6, 0x3F);
	RM68200_DCS_write_1A_1P(0xB7, 0x3F);
	RM68200_DCS_write_1A_1P(0xB8, 0x3F);
	RM68200_DCS_write_1A_1P(0xB9, 0x0B);
	RM68200_DCS_write_1A_1P(0xBA, 0x0D);
	RM68200_DCS_write_1A_1P(0xBB, 0x09);
	RM68200_DCS_write_1A_1P(0xBC, 0x3F);
	RM68200_DCS_write_1A_1P(0xBD, 0x3F);
	RM68200_DCS_write_1A_1P(0xBE, 0x07);
	RM68200_DCS_write_1A_1P(0xBF, 0x03);
	RM68200_DCS_write_1A_1P(0xC0, 0x3F);
	RM68200_DCS_write_1A_1P(0xC1, 0x3F);
	RM68200_DCS_write_1A_1P(0xC2, 0x3F);
	RM68200_DCS_write_1A_1P(0xC3, 0x3F);
	RM68200_DCS_write_1A_1P(0xC4, 0x02);
	RM68200_DCS_write_1A_1P(0xC5, 0x06);
	RM68200_DCS_write_1A_1P(0xC6, 0x3F);
	RM68200_DCS_write_1A_1P(0xC7, 0x3F);
	RM68200_DCS_write_1A_1P(0xC8, 0x08);
	RM68200_DCS_write_1A_1P(0xC9, 0x0C);
	RM68200_DCS_write_1A_1P(0xCA, 0x0A);
	RM68200_DCS_write_1A_1P(0xCB, 0x3F);
	RM68200_DCS_write_1A_1P(0xCC, 0x3F);
	RM68200_DCS_write_1A_1P(0xCD, 0x3F);
	RM68200_DCS_write_1A_1P(0xCE, 0x3F);
	RM68200_DCS_write_1A_1P(0xCF, 0x3F);
	RM68200_DCS_write_1A_1P(0xD0, 0x0E);
	RM68200_DCS_write_1A_1P(0xD1, 0x10);
	RM68200_DCS_write_1A_1P(0xD2, 0x14);
	RM68200_DCS_write_1A_1P(0xD3, 0x12);
	RM68200_DCS_write_1A_1P(0xD4, 0x16);
	RM68200_DCS_write_1A_1P(0xD5, 0x00);
	RM68200_DCS_write_1A_1P(0xD6, 0x04);
	RM68200_DCS_write_1A_1P(0xD7, 0x3F);
	RM68200_DCS_write_1A_1P(0xDC, 0x02);
	RM68200_DCS_write_1A_1P(0xDE, 0x12);
	RM68200_DCS_write_1A_1P(0xFE, 0x0E);
	RM68200_DCS_write_1A_1P(0x01, 0x75);

	/* change to MCS Page 3: Gamma Settings */
	RM68200_DCS_write_1A_1P(0xFE, 0x04);
	RM68200_DCS_write_1A_1P(0x60, 0x00);
	RM68200_DCS_write_1A_1P(0x61, 0x0C);
	RM68200_DCS_write_1A_1P(0x62, 0x12);
	RM68200_DCS_write_1A_1P(0x63, 0x0E);
	RM68200_DCS_write_1A_1P(0x64, 0x06);
	RM68200_DCS_write_1A_1P(0x65, 0x12);
	RM68200_DCS_write_1A_1P(0x66, 0x0E);
	RM68200_DCS_write_1A_1P(0x67, 0x0B);
	RM68200_DCS_write_1A_1P(0x68, 0x15);
	RM68200_DCS_write_1A_1P(0x69, 0x0B);
	RM68200_DCS_write_1A_1P(0x6A, 0x10);
	RM68200_DCS_write_1A_1P(0x6B, 0x07);
	RM68200_DCS_write_1A_1P(0x6C, 0x0F);
	RM68200_DCS_write_1A_1P(0x6D, 0x12);
	RM68200_DCS_write_1A_1P(0x6E, 0x0C);
	RM68200_DCS_write_1A_1P(0x6F, 0x00);
	RM68200_DCS_write_1A_1P(0x70, 0x00);
	RM68200_DCS_write_1A_1P(0x71, 0x0C);
	RM68200_DCS_write_1A_1P(0x72, 0x12);
	RM68200_DCS_write_1A_1P(0x73, 0x0E);
	RM68200_DCS_write_1A_1P(0x74, 0x06);
	RM68200_DCS_write_1A_1P(0x75, 0x12);
	RM68200_DCS_write_1A_1P(0x76, 0x0E);
	RM68200_DCS_write_1A_1P(0x77, 0x0B);
	RM68200_DCS_write_1A_1P(0x78, 0x15);
	RM68200_DCS_write_1A_1P(0x79, 0x0B);
	RM68200_DCS_write_1A_1P(0x7A, 0x10);
	RM68200_DCS_write_1A_1P(0x7B, 0x07);
	RM68200_DCS_write_1A_1P(0x7C, 0x0F);
	RM68200_DCS_write_1A_1P(0x7D, 0x12);
	RM68200_DCS_write_1A_1P(0x7E, 0x0C);
	RM68200_DCS_write_1A_1P(0x7F, 0x00);

	/* change to MCS Page 0 */
	RM68200_DCS_write_1A_1P(0xFE, 0x00);
	RM68200_DCS_write_1A_1P(0x11, 0x00);
	mdelay(200);
	RM68200_DCS_write_1A_1P(0x29, 0x00);
	mdelay(100);
	RM68200_DCS_write_1A_0P(0x2C);
	RM68200_DCS_write_1A_1P(0x35, 0x00);
	mdelay(200);

	return 0;
}

static int mipid_bl_update_status(struct backlight_device *bl)
{
	return 0;
}

static int mipid_bl_get_brightness(struct backlight_device *bl)
{
	return 255;
}

static int mipi_bl_check_fb(struct backlight_device *bl, struct fb_info *fbi)
{
	return 0;
}

static const struct backlight_ops mipid_lcd_bl_ops = {
	.update_status = mipid_bl_update_status,
	.get_brightness = mipid_bl_get_brightness,
	.check_fb = mipi_bl_check_fb,
};
