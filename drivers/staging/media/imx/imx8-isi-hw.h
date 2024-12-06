/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2019-2020 NXP
 *
 */

#ifndef __MXC_ISI_HW_H__
#define __MXC_ISI_HW_H__

#include <linux/delay.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

#include "imx8-isi-core.h"

/* ISI Registers Define  */
/* Channel Control Register */
#define  CHNL_CTRL				0x0
#define  CHNL_CTRL_CHNL_EN_OFFSET		31
#define  CHNL_CTRL_CHNL_EN_MASK			0x80000000
#define  CHNL_CTRL_CHNL_EN_DISABLE		0
#define  CHNL_CTRL_CHNL_EN_ENABLE		1
#define  CHNL_CTRL_CLK_EN_OFFSET		30
#define  CHNL_CTRL_CLK_EN_MASK			0x40000000
#define  CHNL_CTRL_CLK_EN_DISABLE		0
#define  CHNL_CTRL_CLK_EN_ENABLE		1
#define  CHNL_CTRL_CHNL_BYPASS_OFFSET		29
#define  CHNL_CTRL_CHNL_BYPASS_MASK		0x20000000
#define  CHNL_CTRL_CHNL_BYPASS_ENABLE		1
#define  CHNL_CTRL_CHAIN_BUF_OFFSET		25
#define  CHNL_CTRL_CHAIN_BUF_MASK		0x6000000
#define  CHNL_CTRL_CHAIN_BUF_NO_CHAIN		0
#define  CHNL_CTRL_CHAIN_BUF_2_CHAIN		1
#define  CHNL_CTRL_SW_RST_OFFSET		24
#define  CHNL_CTRL_SW_RST_MASK			0x1000000
#define  CHNL_CTRL_SW_RST			0x1000000
#define  CHNL_CTRL_BLANK_PXL_OFFSET		16
#define  CHNL_CTRL_MIPI_VC_ID_OFFSET		6
#define  CHNL_CTRL_MIPI_VC_ID_MASK		0xc0
#define  CHNL_CTRL_MIPI_VC_ID_VC0		0
#define  CHNL_CTRL_MIPI_VC_ID_VC1		1
#define  CHNL_CTRL_MIPI_VC_ID_VC2		2
#define  CHNL_CTRL_MIPI_VC_ID_VC3		3
#define  CHNL_CTRL_SRC_TYPE_OFFSET		4
#define  CHNL_CTRL_SRC_TYPE_MASK		0x10
#define  CHNL_CTRL_SRC_TYPE_DEVICE		0
#define  CHNL_CTRL_SRC_TYPE_MEMORY		1
#define  CHNL_CTRL_SRC_INPUT_OFFSET		0
#define  CHNL_CTRL_SRC_INPUT_MASK		0x7
#define  CHNL_CTRL_SRC_INPUT_MEMORY		5

/* Channel Image Control Register */
#define  CHNL_IMG_CTRL				0x4
#define  CHNL_IMG_CTRL_FORMAT_OFFSET		24
#define  CHNL_IMG_CTRL_FORMAT_MASK		0x3F000000
#define  CHNL_IMG_CTRL_GBL_ALPHA_VAL_OFFSET	16
#define  CHNL_IMG_CTRL_GBL_ALPHA_VAL_MASK	0xFF0000
#define  CHNL_IMG_CTRL_GBL_ALPHA_EN_OFFSET	15
#define  CHNL_IMG_CTRL_GBL_ALPHA_EN_ENABLE	1
#define  CHNL_IMG_CTRL_GBL_ALPHA_EN_MASK	0x8000
#define  CHNL_IMG_CTRL_DEINT_OFFSET		12
#define  CHNL_IMG_CTRL_DEINT_MASK		0x7000
#define  CHNL_IMG_CTRL_DEINT_WEAVE_ODD_EVEN	2
#define  CHNL_IMG_CTRL_DEINT_WEAVE_EVEN_ODD	3
#define  CHNL_IMG_CTRL_DEINT_BLEND_ODD_EVEN	4
#define  CHNL_IMG_CTRL_DEINT_BLEND_EVEN_ODD	5
#define  CHNL_IMG_CTRL_DEINT_LDOUBLE_ODD_EVEN	6
#define  CHNL_IMG_CTRL_DEINT_LDOUBLE_EVEN_ODD	7
#define  CHNL_IMG_CTRL_DEC_X_OFFSET		10
#define  CHNL_IMG_CTRL_DEC_X_MASK		0xC00
#define  CHNL_IMG_CTRL_DEC_X_0			0
#define  CHNL_IMG_CTRL_DEC_X_2			1
#define  CHNL_IMG_CTRL_DEC_X_4			2
#define  CHNL_IMG_CTRL_DEC_X_8			3
#define  CHNL_IMG_CTRL_DEC_Y_OFFSET		8
#define  CHNL_IMG_CTRL_DEC_Y_MASK		0x300
#define  CHNL_IMG_CTRL_DEC_Y_0			0
#define  CHNL_IMG_CTRL_DEC_Y_2			1
#define  CHNL_IMG_CTRL_DEC_Y_4			2
#define  CHNL_IMG_CTRL_DEC_Y_8			3
#define  CHNL_IMG_CTRL_CROP_EN_OFFSET		7
#define  CHNL_IMG_CTRL_CROP_EN_MASK		0x80
#define  CHNL_IMG_CTRL_CROP_EN_ENABLE		1
#define  CHNL_IMG_CTRL_VFLIP_EN_OFFSET		6
#define  CHNL_IMG_CTRL_VFLIP_EN_MASK		0x40
#define  CHNL_IMG_CTRL_VFLIP_EN_ENABLE		1
#define  CHNL_IMG_CTRL_HFLIP_EN_OFFSET		5
#define  CHNL_IMG_CTRL_HFLIP_EN_MASK		0x20
#define  CHNL_IMG_CTRL_HFLIP_EN_ENABLE		1
#define  CHNL_IMG_CTRL_YCBCR_MODE_OFFSET	3
#define  CHNL_IMG_CTRL_YCBCR_MODE_MASK		0x8
#define  CHNL_IMG_CTRL_YCBCR_MODE_ENABLE	1
#define  CHNL_IMG_CTRL_CSC_MODE_OFFSET		1
#define  CHNL_IMG_CTRL_CSC_MODE_MASK		0x6
#define  CHNL_IMG_CTRL_CSC_MODE_YUV2RGB		0
#define  CHNL_IMG_CTRL_CSC_MODE_YCBCR2RGB	1
#define  CHNL_IMG_CTRL_CSC_MODE_RGB2YUV		2
#define  CHNL_IMG_CTRL_CSC_MODE_RGB2YCBCR	3
#define  CHNL_IMG_CTRL_CSC_BYPASS_OFFSET	0
#define  CHNL_IMG_CTRL_CSC_BYPASS_MASK		0x1
#define  CHNL_IMG_CTRL_CSC_BYPASS_ENABLE	0x1

/* Channel Output Buffer Control Register */
#define  CHNL_OUT_BUF_CTRL					0x8
#define  CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR_OFFSET		15
#define  CHNL_OUT_BUF_CTRL_LOAD_BUF2_ADDR_MASK			0x8000
#define  CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR_OFFSET		14
#define  CHNL_OUT_BUF_CTRL_LOAD_BUF1_ADDR_MASK			0x4000
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_OFFSET		6
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_MASK		0xC0
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_NO_PANIC	0
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_PANIC_25	1
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_PANIC_50	2
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_V_PANIC_75	3
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_OFFSET		3
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_MASK		0x18
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_NO_PANIC	0
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_PANIC_25	1
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_PANIC_50	2
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_U_PANIC_75	3
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_OFFSET		0
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_MASK		0x3
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_NO_PANIC	0
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_PANIC_25	1
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_PANIC_50	2
#define  CHNL_OUT_BUF_CTRL_OFLW_PANIC_SET_THD_Y_PANIC_75	3

/* Channel Image Configuration */
#define  CHNL_IMG_CFG				0xC
#define  CHNL_IMG_CFG_HEIGHT_OFFSET		16
#define  CHNL_IMG_CFG_HEIGHT_MASK		0x1FFF0000
#define  CHNL_IMG_CFG_WIDTH_OFFSET		0
#define  CHNL_IMG_CFG_WIDTH_MASK		0x1FFF

/* Channel Interrupt Enable Register */
#define  CHNL_IER				0x10
#define  CHNL_IER_MEM_RD_DONE_EN_OFFSET		31
#define  CHNL_IER_MEM_RD_DONE_EN_MASK		0x80000000
#define  CHNL_IER_MEM_RD_DONE_EN_ENABLE		1
#define  CHNL_IER_LINE_RCVD_EN_OFFSET		30
#define  CHNL_IER_LINE_RCVD_EN_MASK		0x40000000
#define  CHNL_IER_LINE_RCVD_EN_ENABLE		1
#define  CHNL_IER_FRM_RCVD_EN_OFFSET		29
#define  CHNL_IER_FRM_RCVD_EN_MASK		0x20000000
#define  CHNL_IER_FRM_RCVD_EN_ENABLE		1
#define  CHNL_IER_AXI_WR_ERR_V_EN_OFFSET	28
#define  CHNL_IER_AXI_WR_ERR_V_EN_MASK		0x10000000
#define  CHNL_IER_AXI_WR_ERR_V_EN_ENABLE	1
#define  CHNL_IER_AXI_WR_ERR_U_EN_OFFSET	27
#define  CHNL_IER_AXI_WR_ERR_U_EN_MASK		0x8000000
#define  CHNL_IER_AXI_WR_ERR_U_EN_ENABLE	1
#define  CHNL_IER_AXI_WR_ERR_Y_EN_OFFSET	26
#define  CHNL_IER_AXI_WR_ERR_Y_EN_MASK		0x4000000
#define  CHNL_IER_AXI_WR_ERR_Y_EN_ENABLE	1
#define  CHNL_IER_AXI_RD_ERR_EN_OFFSET		25
#define  CHNL_IER_AXI_RD_ERR_EN_MASK		0x2000000
#define  CHNL_IER_AXI_RD_ERR_EN_ENABLE		1

/* Channel Status Register */
#define  CHNL_STS				0x14
#define  CHNL_STS_MEM_RD_DONE_OFFSET		31
#define  CHNL_STS_MEM_RD_DONE_MASK		0x80000000
#define  CHNL_STS_MEM_RD_DONE_ENABLE		1
#define  CHNL_STS_LINE_STRD_OFFSET		30
#define  CHNL_STS_LINE_STRD_MASK		0x40000000
#define  CHNL_STS_LINE_STRD_ENABLE		1
#define  CHNL_STS_FRM_STRD_OFFSET		29
#define  CHNL_STS_FRM_STRD_MASK			0x20000000
#define  CHNL_STS_FRM_STRD_ENABLE		1
#define  CHNL_STS_AXI_WR_ERR_V_OFFSET		28
#define  CHNL_STS_AXI_WR_ERR_V_MASK		0x10000000
#define  CHNL_STS_AXI_WR_ERR_V_ENABLE		1
#define  CHNL_STS_AXI_WR_ERR_U_OFFSET		27
#define  CHNL_STS_AXI_WR_ERR_U_MASK		0x8000000
#define  CHNL_STS_AXI_WR_ERR_U_ENABLE		1
#define  CHNL_STS_AXI_WR_ERR_Y_OFFSET		26
#define  CHNL_STS_AXI_WR_ERR_Y_MASK		0x4000000
#define  CHNL_STS_AXI_WR_ERR_Y_ENABLE		1
#define  CHNL_STS_AXI_RD_ERR_OFFSET		25
#define  CHNL_STS_AXI_RD_ERR_MASK		0x2000000
#define  CHNL_STS_AXI_RD_ERR_ENABLE		1
#define  CHNL_STS_OFLW_PANIC_V_BUF_OFFSET	24
#define  CHNL_STS_OFLW_PANIC_V_BUF_MASK		0x1000000
#define  CHNL_STS_OFLW_PANIC_V_BUF_ENABLE	1
#define  CHNL_STS_EXCS_OFLW_V_BUF_OFFSET	23
#define  CHNL_STS_EXCS_OFLW_V_BUF_MASK		0x800000
#define  CHNL_STS_EXCS_OFLW_V_BUF_ENABLE	1
#define  CHNL_STS_OFLW_V_BUF_OFFSET		22
#define  CHNL_STS_OFLW_V_BUF_MASK		0x400000
#define  CHNL_STS_OFLW_V_BUF_ENABLE		1
#define  CHNL_STS_OFLW_PANIC_U_BUF_OFFSET	21
#define  CHNL_STS_OFLW_PANIC_U_BUF_MASK		0x200000
#define  CHNL_STS_OFLW_PANIC_U_BUF_ENABLE	1
#define  CHNL_STS_EXCS_OFLW_U_BUF_OFFSET	20
#define  CHNL_STS_EXCS_OFLW_U_BUF_MASK		0x100000
#define  CHNL_STS_EXCS_OFLW_U_BUF_ENABLE	1
#define  CHNL_STS_OFLW_U_BUF_OFFSET		19
#define  CHNL_STS_OFLW_U_BUF_MASK		0x80000
#define  CHNL_STS_OFLW_U_BUF_ENABLE		1
#define  CHNL_STS_OFLW_PANIC_Y_BUF_OFFSET	18
#define  CHNL_STS_OFLW_PANIC_Y_BUF_MASK		0x40000
#define  CHNL_STS_OFLW_PANIC_Y_BUF_ENABLE	1
#define  CHNL_STS_EXCS_OFLW_Y_BUF_OFFSET	17
#define  CHNL_STS_EXCS_OFLW_Y_BUF_MASK		0x20000
#define  CHNL_STS_EXCS_OFLW_Y_BUF_ENABLE	1
#define  CHNL_STS_OFLW_Y_BUF_OFFSET		16
#define  CHNL_STS_OFLW_Y_BUF_MASK		0x10000
#define  CHNL_STS_OFLW_Y_BUF_ENABLE		1
#define  CHNL_STS_OFLW_BYTES_OFFSET		0
#define  CHNL_STS_OFLW_BYTES_MASK		0xFF

/* Channel Scale Factor Register */
#define  CHNL_SCALE_FACTOR			0x18
#define  CHNL_SCALE_FACTOR_Y_SCALE_OFFSET	16
#define  CHNL_SCALE_FACTOR_Y_SCALE_MASK		0x3FFF0000
#define  CHNL_SCALE_FACTOR_X_SCALE_OFFSET	0
#define  CHNL_SCALE_FACTOR_X_SCALE_MASK		0x3FFF

/* Channel Scale Offset Register */
#define  CHNL_SCALE_OFFSET			0x1C
#define  CHNL_SCALE_OFFSET_Y_SCALE_OFFSET	16
#define  CHNL_SCALE_OFFSET_Y_SCALE_MASK		0xFFF0000
#define  CHNL_SCALE_OFFSET_X_SCALE_OFFSET	0
#define  CHNL_SCALE_OFFSET_X_SCALE_MASK		0xFFF

/* Channel Crop Upper Left Corner Coordinate Register */
#define  CHNL_CROP_ULC				0x20
#define  CHNL_CROP_ULC_X_OFFSET			16
#define  CHNL_CROP_ULC_X_MASK			0xFFF0000
#define  CHNL_CROP_ULC_Y_OFFSET			0
#define  CHNL_CROP_ULC_Y_MASK			0xFFF

/* Channel Crop Lower Right Corner Coordinate Register */
#define  CHNL_CROP_LRC				0x24
#define  CHNL_CROP_LRC_X_OFFSET			16
#define  CHNL_CROP_LRC_X_MASK			0xFFF0000
#define  CHNL_CROP_LRC_Y_OFFSET			0
#define  CHNL_CROP_LRC_Y_MASK			0xFFF

/* Channel Color Space Conversion Coefficient Register 0 */
#define  CHNL_CSC_COEFF0			0x28
#define  CHNL_CSC_COEFF0_A2_OFFSET		16
#define  CHNL_CSC_COEFF0_A2_MASK		0x7FF0000
#define  CHNL_CSC_COEFF0_A1_OFFSET		0
#define  CHNL_CSC_COEFF0_A1_MASK		0x7FF

/* Channel Color Space Conversion Coefficient Register 1 */
#define  CHNL_CSC_COEFF1			0x2C
#define  CHNL_CSC_COEFF1_B1_OFFSET		16
#define  CHNL_CSC_COEFF1_B1_MASK		0x7FF0000
#define  CHNL_CSC_COEFF1_A3_OFFSET		0
#define  CHNL_CSC_COEFF1_A3_MASK		0x7FF

/* Channel Color Space Conversion Coefficient Register 2 */
#define  CHNL_CSC_COEFF2			0x30
#define  CHNL_CSC_COEFF2_B3_OFFSET		16
#define  CHNL_CSC_COEFF2_B3_MASK		0x7FF0000
#define  CHNL_CSC_COEFF2_B2_OFFSET		0
#define  CHNL_CSC_COEFF2_B2_MASK		0x7FF

/* Channel Color Space Conversion Coefficient Register 3 */
#define  CHNL_CSC_COEFF3			0x34
#define  CHNL_CSC_COEFF3_C2_OFFSET		16
#define  CHNL_CSC_COEFF3_C2_MASK		0x7FF0000
#define  CHNL_CSC_COEFF3_C1_OFFSET		0
#define  CHNL_CSC_COEFF3_C1_MASK		0x7FF

/* Channel Color Space Conversion Coefficient Register 4 */
#define  CHNL_CSC_COEFF4			0x38
#define  CHNL_CSC_COEFF4_D1_OFFSET		16
#define  CHNL_CSC_COEFF4_D1_MASK		0x1FF0000
#define  CHNL_CSC_COEFF4_C3_OFFSET		0
#define  CHNL_CSC_COEFF4_C3_MASK		0x7FF

/* Channel Color Space Conversion Coefficient Register 5 */
#define  CHNL_CSC_COEFF5			0x3C
#define  CHNL_CSC_COEFF5_D3_OFFSET		16
#define  CHNL_CSC_COEFF5_D3_MASK		0x1FF0000
#define  CHNL_CSC_COEFF5_D2_OFFSET		0
#define  CHNL_CSC_COEFF5_D2_MASK		0x1FF

/* Channel Alpha Value Register for ROI 0 */
#define  CHNL_ROI_0_ALPHA			0x40
#define  CHNL_ROI_0_ALPHA_OFFSET		24
#define  CHNL_ROI_0_ALPHA_MASK			0xFF000000
#define  CHNL_ROI_0_ALPHA_EN_OFFSET		16
#define  CHNL_ROI_0_ALPHA_EN_MASK		0x10000

/* Channel Upper Left Coordinate Register for ROI 0 */
#define  CHNL_ROI_0_ULC				0x44
#define  CHNL_ROI_0_ULC_X_OFFSET		16
#define  CHNL_ROI_0_ULC_X_MASK			0xFFF0000
#define  CHNL_ROI_0_ULC_Y_OFFSET		0
#define  CHNL_ROI_0_ULC_Y_MASK			0xFFF

/* Channel Lower Right Coordinate Register for ROI 0 */
#define  CHNL_ROI_0_LRC				0x48
#define  CHNL_ROI_0_LRC_X_OFFSET		16
#define  CHNL_ROI_0_LRC_X_MASK			0xFFF0000
#define  CHNL_ROI_0_LRC_Y_OFFSET		0
#define  CHNL_ROI_0_LRC_Y_MASK			0xFFF

/* Channel Alpha Value Register for ROI 1 */
#define  CHNL_ROI_1_ALPHA			0x4C
#define  CHNL_ROI_1_ALPHA_OFFSET		24
#define  CHNL_ROI_1_ALPHA_MASK			0xFF000000
#define  CHNL_ROI_1_ALPHA_EN_OFFSET		16
#define  CHNL_ROI_1_ALPHA_EN_MASK		0x10000

/* Channel Upper Left Coordinate Register for ROI 1 */
#define  CHNL_ROI_1_ULC				0x50
#define  CHNL_ROI_1_ULC_X_OFFSET		16
#define  CHNL_ROI_1_ULC_X_MASK			0xFFF0000
#define  CHNL_ROI_1_ULC_Y_OFFSET		0
#define  CHNL_ROI_1_ULC_Y_MASK			0xFFF

/* Channel Lower Right Coordinate Register for ROI 1 */
#define  CHNL_ROI_1_LRC				0x54
#define  CHNL_ROI_1_LRC_X_OFFSET		16
#define  CHNL_ROI_1_LRC_X_MASK			0xFFF0000
#define  CHNL_ROI_1_LRC_Y_OFFSET		0
#define  CHNL_ROI_1_LRC_Y_MASK			0xFFF

/* Channel Alpha Value Register for ROI 2 */
#define  CHNL_ROI_2_ALPHA			0x58
#define  CHNL_ROI_2_ALPHA_OFFSET		24
#define  CHNL_ROI_2_ALPHA_MASK			0xFF000000
#define  CHNL_ROI_2_ALPHA_EN_OFFSET		16
#define  CHNL_ROI_2_ALPHA_EN_MASK		0x10000

/* Channel Upper Left Coordinate Register for ROI 2 */
#define  CHNL_ROI_2_ULC				0x5C
#define  CHNL_ROI_2_ULC_X_OFFSET		16
#define  CHNL_ROI_2_ULC_X_MASK			0xFFF0000
#define  CHNL_ROI_2_ULC_Y_OFFSET		0
#define  CHNL_ROI_2_ULC_Y_MASK			0xFFF

/* Channel Lower Right Coordinate Register for ROI 2 */
#define  CHNL_ROI_2_LRC				0x60
#define  CHNL_ROI_2_LRC_X_OFFSET		16
#define  CHNL_ROI_2_LRC_X_MASK			0xFFF0000
#define  CHNL_ROI_2_LRC_Y_OFFSET		0
#define  CHNL_ROI_2_LRC_Y_MASK			0xFFF

/* Channel Alpha Value Register for ROI 3 */
#define  CHNL_ROI_3_ALPHA			0x64
#define  CHNL_ROI_3_ALPHA_OFFSET		24
#define  CHNL_ROI_3_ALPHA_MASK			0xFF000000
#define  CHNL_ROI_3_ALPHA_EN_OFFSET		16
#define  CHNL_ROI_3_ALPHA_EN_MASK		0x10000

/* Channel Upper Left Coordinate Register for ROI 3 */
#define  CHNL_ROI_3_ULC				0x68
#define  CHNL_ROI_3_ULC_X_OFFSET		16
#define  CHNL_ROI_3_ULC_X_MASK			0xFFF0000
#define  CHNL_ROI_3_ULC_Y_OFFSET		0
#define  CHNL_ROI_3_ULC_Y_MASK			0xFFF

/* Channel Lower Right Coordinate Register for ROI 3 */
#define  CHNL_ROI_3_LRC				0x6C
#define  CHNL_ROI_3_LRC_X_OFFSET		16
#define  CHNL_ROI_3_LRC_X_MASK			0xFFF0000
#define  CHNL_ROI_3_LRC_Y_OFFSET		0
#define  CHNL_ROI_3_LRC_Y_MASK			0xFFF

/* Channel RGB or Luma (Y) Output Buffer 1 Address */
#define  CHNL_OUT_BUF1_ADDR_Y			0x70

/* Channel Chroma (U/Cb/UV/CbCr) Output Buffer 1 Address */
#define  CHNL_OUT_BUF1_ADDR_U			0x74

/* Channel Chroma (V/Cr) Output Buffer 1 Address */
#define  CHNL_OUT_BUF1_ADDR_V			0x78

/* Channel Output Buffer Pitch */
#define  CHNL_OUT_BUF_PITCH			0x7C
#define  CHNL_OUT_BUF_PITCH_LINE_PITCH_OFFSET	0
#define  CHNL_OUT_BUF_PITCH_LINE_PITCH_MASK	0xFFFF

/* Channel Input Buffer Address */
#define  CHNL_IN_BUF_ADDR			0x80

/* Channel Input Buffer Pitch */
#define  CHNL_IN_BUF_PITCH			0x84
#define  CHNL_IN_BUF_PITCH_FRM_PITCH_OFFSET	16
#define  CHNL_IN_BUF_PITCH_FRM_PITCH_MASK	0xFFFF0000
#define  CHNL_IN_BUF_PITCH_LINE_PITCH_OFFSET	0
#define  CHNL_IN_BUF_PITCH_LINE_PITCH_MASK	0xFFFF

/* Channel Memory Read Control */
#define  CHNL_MEM_RD_CTRL			0x88
#define  CHNL_MEM_RD_CTRL_IMG_TYPE_OFFSET	28
#define  CHNL_MEM_RD_CTRL_IMG_TYPE_MASK		0xF0000000
#define  CHNL_MEM_RD_CTRL_READ_MEM_OFFSET	0
#define  CHNL_MEM_RD_CTRL_READ_MEM_MASK		1
#define  CHNL_MEM_RD_CTRL_READ_MEM_ENABLE	1

/* Channel RGB or Luma (Y) Output Buffer 2 Address */
#define  CHNL_OUT_BUF2_ADDR_Y		0x8C

/* Channel Chroma (U/Cb/UV/CbCr) Output Buffer 2 Address  */
#define  CHNL_OUT_BUF2_ADDR_U		0x90

/* Channel Chroma (V/Cr) Output Buffer 2 Address   */
#define  CHNL_OUT_BUF2_ADDR_V		0x94

/* Channel scale image config */
#define  CHNL_SCL_IMG_CFG			0x98
#define  CHNL_SCL_IMG_CFG_HEIGHT_OFFSET		16
#define  CHNL_SCL_IMG_CFG_HEIGHT_MASK		0x1FFF0000
#define  CHNL_SCL_IMG_CFG_WIDTH_OFFSET		0
#define  CHNL_SCL_IMG_CFG_WIDTH_MASK		0x1FFF

/* Channel Flow Control Register */
#define  CHNL_FLOW_CTRL				0x9C
#define  CHNL_FLOW_CTRL_FC_DENOM_MASK		0xFF
#define  CHNL_FLOW_CTRL_FC_DENOM_OFFSET		0
#define  CHNL_FLOW_CTRL_FC_NUMER_MASK		0xFF0000
#define  CHNL_FLOW_CTRL_FC_NUMER_OFFSET		0

/* The AXI limit register offset */
#define AXI_LIMIT_CONTROL_OFFSET		0x12c
#define AXI_LIMIT_THRESH1_OFFSET		0x134

/* The AXI limit enable register config */
#define AXI_LIMIT_LCDIF0_EN			BIT(0)
#define AXI_LIMIT_LCDIF1_EN			BIT(1)
#define AXI_LIMIT_ISI_EN			BIT(2)
#define AXI_LIMIT_DEWARP_EN			BIT(3)

/* The AXI isi limit thresh */
#define AXI_LIMIT_ISI_THRESH			0x10

enum isi_csi_coeff {
	YUV2RGB = 0,
	RGB2YUV,
};

int mxc_isi_config_parm(struct mxc_isi_cap_dev *isi_cap);

void mxc_isi_channel_init(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_deinit(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_enable_loc(struct mxc_isi_dev *mxc_isi, bool m2m_enabled);
void mxc_isi_channel_disable_loc(struct mxc_isi_dev *mxc_isi);
void mxc_isi_cap_frame_write_done(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_deinterlace(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_sw_reset(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_hw_reset(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_source_config(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_flip_loc(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_alpha_loc(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_chain_buf(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_deinterlace(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_crop(struct mxc_isi_dev *mxc_isi,
			      struct mxc_isi_frame *dst_f);
void mxc_isi_channel_set_memory_image(struct mxc_isi_dev *mxc_isi);
void mxc_isi_channel_set_panic_threshold(struct mxc_isi_dev *mxc_isi);

void mxc_isi_channel_set_scaling(struct mxc_isi_dev *mxc_isi,
				 struct mxc_isi_frame *src_f,
				 struct mxc_isi_frame *dst_f);

void mxc_isi_channel_set_outbuf_loc(struct mxc_isi_dev *mxc_isi,
				    struct mxc_isi_buffer *buf);

void mxc_isi_channel_set_csc(struct mxc_isi_dev *mxc_isi,
			     struct mxc_isi_frame *src_f,
			     struct mxc_isi_frame *dst_f);

void mxc_isi_channel_config_loc(struct mxc_isi_dev *mxc_isi,
				struct mxc_isi_frame *src_f,
				struct mxc_isi_frame *dst_f);

void mxc_isi_channel_set_alpha_roi0(struct mxc_isi_dev *mxc_isi,
				    struct v4l2_rect *rect);
void mxc_isi_channel_set_m2m_src_addr(struct mxc_isi_dev *mxc_isi,
			struct mxc_isi_buffer *buf);

void mxc_isi_m2m_config_src(struct mxc_isi_dev *mxc_isi,
			    struct mxc_isi_frame *src_f);
void mxc_isi_m2m_config_dst(struct mxc_isi_dev *mxc_isi,
			    struct mxc_isi_frame *dst_f);

void mxc_isi_m2m_start_read(struct mxc_isi_dev *mxc_isi);
void mxc_isi_m2m_frame_write_done(struct mxc_isi_dev *mxc_isi);
void mxc_isi_clean_irq_status(struct mxc_isi_dev *mxc_isi, u32 val);
void mxc_isi_clean_registers(struct mxc_isi_dev *mxc_isi);
void mxc_isi_enable_irq(struct mxc_isi_dev *mxc_isi);
void mxc_isi_disable_irq(struct mxc_isi_dev *mxc_isi);
void dump_isi_regs(struct mxc_isi_dev *mxc_isi);

u32 mxc_isi_get_irq_status(struct mxc_isi_dev *mxc_isi);
bool is_buf_active(struct mxc_isi_dev *mxc_isi, int buf_id);

struct device *mxc_isi_dev_get_parent(struct platform_device *pdev);
struct mxc_isi_dev *mxc_isi_get_hostdata(struct platform_device *pdev);

#endif /* __MXC_ISI_HW_H__ */
