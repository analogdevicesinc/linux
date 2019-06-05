/*
 * Copyright 2019 NXP
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

#ifndef __IMX8MN_DISPMIX_H__
#define __IMX8MN_DISPMIX_H__

/* DISPMIX soft reset */
#define IMX8MN_MIPI_DSI_PCLK_RESET			0
#define IMX8MN_MIPI_DSI_CLKREF_RESET			1
#define IMX8MN_MIPI_CSI_PCLK_RESET			2
#define IMX8MN_MIPI_CSI_ACLK_RESET			3
#define IMX8MN_LCDIF_PIXEL_CLK_RESET			4
#define IMX8MN_LCDIF_APB_CLK_RESET			5
#define IMX8MN_ISI_PROC_CLK_RESET			6
#define IMX8MN_ISI_APB_CLK_RESET			7
#define IMX8MN_BUS_BLK_CLK_RESET			8
#define IMX8MN_DISPMIX_SFT_RSTN_NUM			9

/* DISPMIX clock soft enable */
#define IMX8MN_MIPI_DSI_PCLK_EN				0
#define IMX8MN_MIPI_DSI_CLKREF_EN			1
#define IMX8MN_MIPI_CSI_PCLK_EN				2
#define IMX8MN_MIPI_CSI_ACLK_EN				3
#define IMX8MN_LCDIF_PIXEL_CLK_EN			4
#define IMX8MN_LCDIF_APB_CLK_EN				5
#define IMX8MN_ISI_PROC_CLK_EN				6
#define IMX8MN_ISI_APB_CLK_EN				7
#define IMX8MN_BUS_BLK_CLK_EN				8
#define IMX8MN_DISPMIX_CLK_EN_NUM			9

/* MIPI reset */
#define IMX8MN_MIPI_S_RESET				0
#define IMX8MN_MIPI_M_RESET				1
#define IMX8MN_MIPI_RESET_NUM				2

#endif
