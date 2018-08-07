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

#ifndef __LCDIF_REGS_H
#define __LCDIF_REGS_H

#define REG_SET	4
#define REG_CLR	8

/* regs offset */
#define LCDIF_CTRL			0x00
#define LCDIF_CTRL1			0X10
#define LCDIF_CTRL2			0X20
#define LCDIF_TRANSFER_COUNT		0x30
#define LCDIF_CUR_BUF			0x40
#define LCDIF_NEXT_BUF			0x50
#define LCDIF_TIMING			0x60
#define LCDIF_VDCTRL0			0x70
#define LCDIF_VDCTRL1			0x80
#define LCDIF_VDCTRL2			0x90
#define LCDIF_VDCTRL3			0xa0
#define LCDIF_VDCTRL4			0xb0

/* pigeon registers for crop */
#define HW_EPDC_PIGEON_12_0		0xb00
#define HW_EPDC_PIGEON_12_1		0xb10
#define HW_EPDC_PIGEON_12_2		0xb20

/* reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

#define SWIZZLE_LE		0 /* Little-Endian or No swap */
#define SWIZZLE_BE		1 /* Big-Endian or swap all */
#define SWIZZLE_HWD		2 /* Swap half-words */
#define SWIZZLE_HWD_BYTE	3 /* Swap bytes within each half-word */

/* regs bit fields */
#define CTRL_SFTRST			BIT(31)
#define CTRL_CLKGATE			BIT(30)
#define CTRL_SHIFT_DIR(x)		REG_PUT((x), 26, 26)
#define CTRL_SHIFT_NUM(x)		REG_PUT((x), 25, 21)
#define CTRL_BYPASS_COUNT		BIT(19)
#define CTRL_VSYNC_MODE			BIT(18)
#define CTRL_DOTCLK_MODE		BIT(17)
#define CTRL_DATA_SELECT		BIT(16)
#define CTRL_INPUT_SWIZZLE(x)		REG_PUT((x), 15, 14)
#define CTRL_CSC_SWIZZLE(x)		REG_PUT((x), 13, 12)
#define CTRL_SET_BUS_WIDTH(x)		REG_PUT((x), 11, 10)
#define CTRL_GET_BUS_WIDTH(x)		REG_GET((x), 11, 10)
#define CTRL_BUS_WIDTH_MASK		REG_PUT((0x3), 11, 10)
#define CTRL_SET_WORD_LENGTH(x)		REG_PUT((x), 9, 8)
#define CTRL_GET_WORD_LENGTH(x)		REG_GET((x), 9, 8)
#define CTRL_MASTER			BIT(5)
#define CTRL_DF16			BIT(3)
#define CTRL_DF18			BIT(2)
#define CTRL_DF24			BIT(1)
#define CTRL_RUN			BIT(0)

#define CTRL1_RECOVERY_ON_UNDERFLOW	BIT(24)
#define CTRL1_FIFO_CLEAR		BIT(21)
#define CTRL1_SET_BYTE_PACKAGING(x)	REG_PUT((x), 19, 16)
#define CTRL1_GET_BYTE_PACKAGING(x)	REG_GET((x), 19, 16)
#define CTRL1_CUR_FRAME_DONE_IRQ_EN	BIT(13)
#define CTRL1_CUR_FRAME_DONE_IRQ	BIT(9)

#define REQ_1	0
#define REQ_2	1
#define REQ_4	2
#define REQ_8	3
#define REQ_16	4

#define CTRL2_OUTSTANDING_REQS(x)	REG_PUT((x), 23, 21)
#define CTRL2_ODD_LINE_PATTERN(x)	REG_PUT((x), 18, 16)
#define CTRL2_EVEN_LINE_PATTERN(x)	REG_PUT((x), 14, 12)

#define TRANSFER_COUNT_SET_VCOUNT(x)	(((x) & 0xffff) << 16)
#define TRANSFER_COUNT_GET_VCOUNT(x)	(((x) >> 16) & 0xffff)
#define TRANSFER_COUNT_SET_HCOUNT(x)	((x) & 0xffff)
#define TRANSFER_COUNT_GET_HCOUNT(x)	((x) & 0xffff)

#define VDCTRL0_ENABLE_PRESENT		BIT(28)
#define VDCTRL0_VSYNC_ACT_HIGH		BIT(27)
#define VDCTRL0_HSYNC_ACT_HIGH		BIT(26)
#define VDCTRL0_DOTCLK_ACT_FALLING	BIT(25)
#define VDCTRL0_ENABLE_ACT_HIGH		BIT(24)
#define VDCTRL0_VSYNC_PERIOD_UNIT	BIT(21)
#define VDCTRL0_VSYNC_PULSE_WIDTH_UNIT	BIT(20)
#define VDCTRL0_HALF_LINE		BIT(19)
#define VDCTRL0_HALF_LINE_MODE		BIT(18)
#define VDCTRL0_SET_VSYNC_PULSE_WIDTH(x) ((x) & 0x3ffff)
#define VDCTRL0_GET_VSYNC_PULSE_WIDTH(x) ((x) & 0x3ffff)

#define VDCTRL2_SET_HSYNC_PULSE_WIDTH(x) (((x) & 0x3fff) << 18)
#define VDCTRL2_GET_HSYNC_PULSE_WIDTH(x) (((x) >> 18) & 0x3fff)
#define VDCTRL2_SET_HSYNC_PERIOD(x)	((x) & 0x3ffff)
#define VDCTRL2_GET_HSYNC_PERIOD(x)	((x) & 0x3ffff)

#define VDCTRL3_MUX_SYNC_SIGNALS	BIT(29)
#define VDCTRL3_VSYNC_ONLY		BIT(28)
#define SET_HOR_WAIT_CNT(x)		(((x) & 0xfff) << 16)
#define GET_HOR_WAIT_CNT(x)		(((x) >> 16) & 0xfff)
#define SET_VERT_WAIT_CNT(x)		((x) & 0xffff)
#define GET_VERT_WAIT_CNT(x)		((x) & 0xffff)

#define VDCTRL4_SET_DOTCLK_DLY(x)	(((x) & 0x7) << 29) /* v4 only */
#define VDCTRL4_GET_DOTCLK_DLY(x)	(((x) >> 29) & 0x7) /* v4 only */
#define VDCTRL4_SYNC_SIGNALS_ON		BIT(18)
#define SET_DOTCLK_H_VALID_DATA_CNT(x)	((x) & 0x3ffff)

#define PIGEON_12_0_SET_STATE_MASK(x)	REG_PUT((x), 31, 24)
#define PIGEON_12_0_SET_MASK_CNT(x)	REG_PUT((x), 23, 12)
#define PIGEON_12_0_SET_MASK_CNT_SEL(x)	REG_PUT((x), 11,  8)
#define PIGEON_12_0_SET_OFFSET(x)	REG_PUT((x),  7,  4)
#define PIGEON_12_0_SET_INC_SEL(x)	REG_PUT((x),  3,  2)
#define PIGEON_12_0_POL_ACTIVE_LOW	BIT(1)
#define PIGEON_12_0_EN			BIT(0)

#define PIGEON_12_1_SET_CLR_CNT(x)	REG_PUT((x), 31, 16)
#define PIGEON_12_1_SET_SET_CNT(x)	REG_PUT((x), 15,  0)

#define STMLCDIF_8BIT  1 /* pixel data bus to the display is of 8 bit width */
#define STMLCDIF_16BIT 0 /* pixel data bus to the display is of 16 bit width */
#define STMLCDIF_18BIT 2 /* pixel data bus to the display is of 18 bit width */
#define STMLCDIF_24BIT 3 /* pixel data bus to the display is of 24 bit width */

#define MIN_XRES			120
#define MIN_YRES			120
#define MAX_XRES			0xffff
#define MAX_YRES			0xffff

#endif
