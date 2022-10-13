/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2019 NXP
 */

#ifndef __LCDIFV3_REGS_H
#define __LCDIFV3_REGS_H

/* regs offset */
#define LCDIFV3_CTRL			0x00
#define LCDIFV3_CTRL_SET		0x04
#define LCDIFV3_CTRL_CLR		0x08
#define LCDIFV3_CTRL_TOG		0x0c
#define LCDIFV3_DISP_PARA		0x10
#define LCDIFV3_DISP_SIZE		0x14
#define LCDIFV3_HSYN_PARA		0x18
#define LCDIFV3_VSYN_PARA		0x1c
#define LCDIFV3_VSYN_HSYN_WIDTH		0x20
#define LCDIFV3_INT_STATUS_D0		0x24
#define LCDIFV3_INT_ENABLE_D0		0x28
#define LCDIFV3_INT_STATUS_D1		0x30
#define LCDIFV3_INT_ENABLE_D1		0x34

#define LCDIFV3_CTRLDESCL0_1		0x200
#define LCDIFV3_CTRLDESCL0_3		0x208
#define LCDIFV3_CTRLDESCL_LOW0_4	0x20c
#define LCDIFV3_CTRLDESCL_HIGH0_4	0x210
#define LCDIFV3_CTRLDESCL0_5		0x214
#define LCDIFV3_CSC0_CTRL		0x21c
#define LCDIFV3_CSC0_COEF0		0x220
#define LCDIFV3_CSC0_COEF1		0x224
#define LCDIFV3_CSC0_COEF2		0x228
#define LCDIFV3_CSC0_COEF3		0x22c
#define LCDIFV3_CSC0_COEF4		0x230
#define LCDIFV3_CSC0_COEF5		0x234
#define LCDIFV3_PANIC0_THRES		0x238

/* reg bit manipulation */
#define REG_MASK(e, s) (((1 << ((e) - (s) + 1)) - 1) << (s))
#define REG_PUT(x, e, s) (((x) << (s)) & REG_MASK(e, s))
#define REG_GET(x, e, s) (((x) & REG_MASK(e, s)) >> (s))

/* regs bit fields */
#define CTRL_SW_RESET			BIT(31)
#define CTRL_FETCH_START_OPTION(x)	REG_PUT((x), 9, 8)
   #define FPV		0
   #define PWV		1
   #define BPV		2
   #define RESV		3
#define CTRL_NEG			BIT(4)
#define CTRL_INV_PXCK			BIT(3)
#define CTRL_INV_DE			BIT(2)
#define CTRL_INV_VS			BIT(1)
#define CTRL_INV_HS			BIT(0)

#define DISP_PARA_DISP_ON		BIT(31)
#define DISP_PARA_SWAP_EN		BIT(30)
#define DISP_PARA_LINE_PATTERN(x)	REG_PUT((x), 29, 26)
   /* line pattern formats (output) */
   #define LP_RGB888_OR_YUV444		0x0
   #define LP_RBG888			0x1
   #define LP_GBR888			0x2
   #define LP_GRB888_OR_UYV444		0x3
   #define LP_BRG888			0x4
   #define LP_BGR888			0x5
   #define LP_RGB555			0x6
   #define LP_RGB565			0x7
   #define LP_YUYV_16_0			0x8
   #define LP_UYVY_16_0			0x9
   #define LP_YVYU_16_0			0xa
   #define LP_VYUY_16_0			0xb
   #define LP_YUYV_23_8			0xc
   #define LP_UYVY_23_8			0xd
   #define LP_YVYU_23_8			0xe
   #define LP_VYUY_23_8			0xf

#define DISP_PARA_DISP_MODE(x)		REG_PUT((x), 25, 24)
#define DISP_PARA_BGND_R(x)		REG_PUT((x), 23, 16)
#define DISP_PARA_BGND_G(x)		REG_PUT((x), 15,  8)
#define DISP_PARA_BGND_B(x)		REG_PUT((x),  7,  0)

#define DISP_SIZE_DELTA_Y(x)		REG_PUT((x), 31, 16)
#define DISP_SIZE_DELTA_X(x)		REG_PUT((x), 15,  0)

#define HSYN_PARA_BP_H(x)		REG_PUT((x), 31, 16)
#define HSYN_PARA_FP_H(x)		REG_PUT((x), 15,  0)

#define VSYN_PARA_BP_V(x)		REG_PUT((x), 31, 16)
#define VSYN_PARA_FP_V(x)		REG_PUT((x), 15,  0)

#define VSYN_HSYN_WIDTH_PW_V(x)		REG_PUT((x), 31, 16)
#define VSYN_HSYN_WIDTH_PW_H(x)		REG_PUT((x), 15,  0)

#define INT_STATUS_D0_FIFO_EMPTY	BIT(24)
#define INT_STATUS_D0_DMA_DONE		BIT(16)
#define INT_STATUS_D0_DMA_ERR		BIT(8)
#define INT_STATUS_D0_VS_BLANK		BIT(2)
#define INT_STATUS_D0_UNDERRUN		BIT(1)
#define INT_STATUS_D0_VSYNC		BIT(0)

#define INT_ENABLE_D0_FIFO_EMPTY_EN	BIT(24)
#define INT_ENABLE_D0_DMA_DONE_EN	BIT(16)
#define INT_ENABLE_D0_DMA_ERR_EN	BIT(8)
#define INT_ENABLE_D0_VS_BLANK_EN	BIT(2)
#define INT_ENABLE_D0_UNDERRUN_EN	BIT(1)
#define INT_ENABLE_D0_VSYNC_EN		BIT(0)

#define INT_STATUS_D1_PLANE_PANIC	BIT(0)
#define INT_ENABLE_D1_PLANE_PANIC_EN	BIT(0)

#define CTRLDESCL0_1_HEIGHT(x)		REG_PUT((x), 31, 16)
#define CTRLDESCL0_1_WIDTH(x)		REG_PUT((x), 15,  0)
#define CTRLDESCL0_3_STATE_CLEAR_VSYNC	BIT(23)
#define CTRLDESCL0_3_P_SIZE(x)		REG_PUT((x), 22, 20)
#define CTRLDESCL0_3_T_SIZE(x)		REG_PUT((x), 17, 16)
#define CTRLDESCL0_3_PITCH(x)		REG_PUT((x), 15,  0)
//#define CTRLDESCL_LOW0_4_ADDR_LOW(x)	REG_PUT((x), 31,  0)
#define CTRLDESCL_HIGH0_4_ADDR_HIGH(x)	REG_PUT((x),  3,  0)
#define CTRLDESCL0_5_EN			BIT(31)	/* enable layer for DMA */
#define CTRLDESCL0_5_SHADOW_LOAD_EN	BIT(30)
#define CTRLDESCL0_5_BPP(x)		REG_PUT((x), 27, 24)
   /* layer encoding formats (input) */
   #define BPP16_RGB565			0x4
   #define BPP16_ARGB1555		0x5
   #define BPP16_ARGB4444		0x6
   #define BPP16_YCbCr422		0x7
   #define BPP24_RGB888			0x8
   #define BPP32_ARGB8888		0x9
   #define BPP32_ABGR8888		0xa
#define CTRLDESCL0_5_YUV_FORMAT(x)	REG_PUT((x), 15, 14)

#define CSC0_CTRL_CSC_MODE(x)		REG_PUT((x),  2,  1)
#define CSC0_CTRL_BYPASS		BIT(0)
#define CSC0_COEF0_A2(x)		REG_PUT((x), 26, 16)
#define CSC0_COEF0_A1(x)		REG_PUT((x), 10,  0)
#define CSC0_COEF1_B1(x)		REG_PUT((x), 26, 16)
#define CSC0_COEF1_A3(x)		REG_PUT((x), 10,  0)
#define CSC0_COEF2_B3(x)		REG_PUT((x), 26, 16)
#define CSC0_COEF2_B2(x)		REG_PUT((x), 10,  0)
#define CSC0_COEF3_C2(x)		REG_PUT((x), 26, 16)
#define CSC0_COEF3_C1(x)		REG_PUT((x), 10,  0)
#define CSC0_COEF4_D1(x)		REG_PUT((x), 24, 16)
#define CSC0_COEF4_C3(x)		REG_PUT((x), 10,  0)
#define CSC0_COEF5_D3(x)		REG_PUT((x), 24, 16)
#define CSC0_COEF5_D2(x)		REG_PUT((x),  8,  0)

#define PANIC0_THRES_PANIC_THRES_LOW(x)	REG_PUT((x), 24, 16)
#define PANIC0_THRES_PANIC_THRES_HIGH(x)	REG_PUT((x), 8, 0)

#endif /* __LCDIFV3_REGS_H */
