/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc. All Rights Reserved.
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
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#ifndef __INCLUDE_MIPI_DSI_NORTHWEST_H
#define __INCLUDE_MIPI_DSI_NORTHWEST_H

/* ----------------------------	register offsets --------------------------- */

/* sim */
#define SIM_SOPT1		0x0
#define MIPI_ISO_DISABLE	0x8

#define SIM_SOPT1CFG		0x4
#define DSI_RST_DPI_N		0x80000000
#define DSI_RST_ESC_N		0x40000000
#define DSI_RST_BYTE_N		0x20000000
#define DSI_SD			0x200
#define DSI_CM			0x100
#define DSI_PLL_EN		0x80

/* dphy */
#define DPHY_PD_DPHY			0x300
#define DPHY_M_PRG_HS_PREPARE		0x304
#define DPHY_MC_PRG_HS_PREPARE		0x308
#define DPHY_M_PRG_HS_ZERO		0x30c
#define DPHY_MC_PRG_HS_ZERO		0x310
#define DPHY_M_PRG_HS_TRAIL		0x314
#define DPHY_MC_PRG_HS_TRAIL		0x318
#define DPHY_PD_PLL			0x31c
#define DPHY_TST			0x320
#define DPHY_CN				0x324
#define DPHY_CM				0x328
#define DPHY_CO				0x32c
#define DPHY_LOCK			0x330
#define DPHY_LOCK_BYP			0x334
#define DPHY_RTERM_SEL			0x338
#define DPHY_AUTO_PD_EN			0x33c
#define DPHY_RXLPRP			0x340
#define DPHY_RXCDRP			0x344

/* host */
#define HOST_CFG_NUM_LANES		0x0
#define HOST_CFG_NONCONTINUOUS_CLK	0x4
#define HOST_CFG_T_PRE			0x8
#define HOST_CFG_T_POST			0xc
#define HOST_CFG_TX_GAP			0x10
#define HOST_CFG_AUTOINSERT_EOTP	0x14
#define HOST_CFG_EXTRA_CMDS_AFTER_EOTP	0x18
#define HOST_CFG_HTX_TO_COUNT		0x1c
#define HOST_CFG_LRX_H_TO_COUNT		0x20
#define HOST_CFG_BTA_H_TO_COUNT		0x24
#define HOST_CFG_TWAKEUP		0x28
#define HOST_CFG_STATUS_OUT		0x2c
#define HOST_RX_ERROR_STATUS		0x30

/* dpi */
#define DPI_PIXEL_PAYLOAD_SIZE		0x200
#define DPI_PIXEL_FIFO_SEND_LEVEL	0x204
#define DPI_INTERFACE_COLOR_CODING	0x208
#define DPI_PIXEL_FORMAT		0x20c
#define DPI_VSYNC_POLARITY		0x210
#define DPI_HSYNC_POLARITY		0x214
#define DPI_VIDEO_MODE			0x218
#define DPI_HFP				0x21c
#define DPI_HBP				0x220
#define DPI_HSA				0x224
#define DPI_ENABLE_MULT_PKTS		0x228
#define DPI_VBP				0x22c
#define DPI_VFP				0x230
#define DPI_BLLP_MODE			0x234
#define DPI_USE_NULL_PKT_BLLP		0x238
#define DPI_VACTIVE			0x23c
#define DPI_VC				0x240

/* apb pkt */
#define HOST_TX_PAYLOAD			0x280

#define HOST_PKT_CONTROL		0x284
#define HOST_PKT_CONTROL_WC(x)		(((x) & 0xffff) << 0)
#define HOST_PKT_CONTROL_VC(x)		(((x) & 0x3) << 16)
#define HOST_PKT_CONTROL_DT(x)		(((x) & 0x3f) << 18)
#define HOST_PKT_CONTROL_HS_SEL(x)	(((x) & 0x1) << 24)
#define HOST_PKT_CONTROL_BTA_TX(x)	(((x) & 0x1) << 25)
#define HOST_PKT_CONTROL_BTA_NO_TX(x)	(((x) & 0x1) << 26)

#define HOST_SEND_PACKET		0x288
#define HOST_PKT_STATUS			0x28c
#define HOST_PKT_FIFO_WR_LEVEL		0x290
#define HOST_PKT_FIFO_RD_LEVEL		0x294
#define HOST_PKT_RX_PAYLOAD		0x298

#define HOST_PKT_RX_PKT_HEADER		0x29c
#define HOST_PKT_RX_PKT_HEADER_WC(x)	(((x) & 0xffff) << 0)
#define HOST_PKT_RX_PKT_HEADER_DT(x)	(((x) & 0x3f) << 16)
#define HOST_PKT_RX_PKT_HEADER_VC(x)	(((x) & 0x3) << 22)

#define HOST_IRQ_STATUS			0x2a0
#define HOST_IRQ_STATUS_SM_NOT_IDLE			(1 << 0)
#define HOST_IRQ_STATUS_TX_PKT_DONE			(1 << 1)
#define HOST_IRQ_STATUS_DPHY_DIRECTION			(1 << 2)
#define HOST_IRQ_STATUS_TX_FIFO_OVFLW			(1 << 3)
#define HOST_IRQ_STATUS_TX_FIFO_UDFLW			(1 << 4)
#define HOST_IRQ_STATUS_RX_FIFO_OVFLW			(1 << 5)
#define HOST_IRQ_STATUS_RX_FIFO_UDFLW			(1 << 6)
#define HOST_IRQ_STATUS_RX_PKT_HDR_RCVD			(1 << 7)
#define HOST_IRQ_STATUS_RX_PKT_PAYLOAD_DATA_RCVD	(1 << 8)
#define HOST_IRQ_STATUS_HOST_BTA_TIMEOUT		(1 << 29)
#define HOST_IRQ_STATUS_LP_RX_TIMEOUT			(1 << 30)
#define HOST_IRQ_STATUS_HS_TX_TIMEOUT			(1 << 31)

#define HOST_IRQ_STATUS2		0x2a4
#define HOST_IRQ_STATUS2_SINGLE_BIT_ECC_ERR		(1 << 0)
#define HOST_IRQ_STATUS2_MULTI_BIT_ECC_ERR		(1 << 1)
#define HOST_IRQ_STATUS2_CRC_ERR			(1 << 2)

#define HOST_IRQ_MASK			0x2a8
#define HOST_IRQ_MASK_SM_NOT_IDLE_MASK			(1 << 0)
#define HOST_IRQ_MASK_TX_PKT_DONE_MASK			(1 << 1)
#define HOST_IRQ_MASK_DPHY_DIRECTION_MASK		(1 << 2)
#define HOST_IRQ_MASK_TX_FIFO_OVFLW_MASK		(1 << 3)
#define HOST_IRQ_MASK_TX_FIFO_UDFLW_MASK		(1 << 4)
#define HOST_IRQ_MASK_RX_FIFO_OVFLW_MASK		(1 << 5)
#define HOST_IRQ_MASK_RX_FIFO_UDFLW_MASK		(1 << 6)
#define HOST_IRQ_MASK_RX_PKT_HDR_RCVD_MASK		(1 << 7)
#define HOST_IRQ_MASK_RX_PKT_PAYLOAD_DATA_RCVD_MASK	(1 << 8)
#define HOST_IRQ_MASK_HOST_BTA_TIMEOUT_MASK		(1 << 29)
#define HOST_IRQ_MASK_LP_RX_TIMEOUT_MASK		(1 << 30)
#define HOST_IRQ_MASK_HS_TX_TIMEOUT_MASK		(1 << 31)

#define HOST_IRQ_MASK2			0x2ac
#define HOST_IRQ_MASK2_SINGLE_BIT_ECC_ERR_MASK		(1 << 0)
#define HOST_IRQ_MASK2_MULTI_BIT_ECC_ERR_MASK		(1 << 1)
#define HOST_IRQ_MASK2_CRC_ERR_MASK			(1 << 2)

/* ------------------------------------- end -------------------------------- */

#endif
