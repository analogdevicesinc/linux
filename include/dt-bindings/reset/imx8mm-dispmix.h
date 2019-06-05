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

#ifndef __IMX8MM_DISPMIX_H__
#define __IMX8MM_DISPMIX_H__

/* DISPMIX soft reset */
#define IMX8MM_CSI_BRIDGE_CHIP_RESET			0
#define IMX8MM_CSI_BRIDGE_IPG_HARD_ASYNC_RESET		1
#define IMX8MM_CSI_BRIDGE_CSI_HRESET			2
#define IMX8MM_CAMERA_PIXEL_RESET			3
#define IMX8MM_MIPI_CSI_I_PRESET			4
#define IMX8MM_MIPI_DSI_I_PRESET			5
#define IMX8MM_BUS_RSTN_BLK_SYNC			6
#define IMX8MM_DISPMIX_SFT_RSTN_NUM			7

/* DISPMIX clock soft enable */
#define IMX8MM_CSI_BRIDGE_CSI_HCLK_EN			0
#define IMX8MM_CSI_BRIDGE_SPU_CLK_EN			1
#define IMX8MM_CSI_BRIDGE_MEM_WRAPPER_CLK_EN		2
#define IMX8MM_CSI_BRIDGE_IPG_CLK_EN			3
#define IMX8MM_CSI_BRIDGE_IPG_CLK_S_EN			4
#define IMX8MM_CSI_BRIDGE_IPG_CLK_S_RAW_EN		5
#define IMX8MM_LCDIF_APB_CLK_EN				6
#define IMX8MM_LCDIF_PIXEL_CLK_EN			7
#define IMX8MM_MIPI_DSI_PCLK_EN				8
#define IMX8MM_MIPI_DSI_CLKREF_EN			9
#define IMX8MM_MIPI_CSI_ACLK_EN				10
#define IMX8MM_MIPI_CSI_PCLK_EN				11
#define IMX8MM_BUS_BLK_CLK_EN				12
#define IMX8MM_DISPMIX_CLK_EN_NUM			13

/* MIPI reset */
#define IMX8MM_MIPI_S_RESET				0
#define IMX8MM_MIPI_M_RESET				1
#define IMX8MM_MIPI_RESET_NUM				2

#endif
