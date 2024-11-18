// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2021-2022 NXP. */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#include "phy-fsl-lynx-xgkr-algorithm.h"

#define LYNX_28G_NUM_LANE			8
#define LYNX_28G_NUM_PLL			2

/* SoC IP wrapper for protocol converters */
#define LYNX_28G_PCC8				0x10a0
#define LYNX_28G_PCC8_SGMIIa_KX			BIT(3)
#define LYNX_28G_PCC8_SGMIIa_CFG		BIT(0)

#define LYNX_28G_PCCC				0x10b0
#define LYNX_28G_PCCC_SXGMIIn_XFI		BIT(3)
#define LYNX_28G_PCCC_SXGMIIn_CFG		BIT(0)

#define LYNX_28G_PCCD				0x10b4
#define LYNX_28G_PCCD_E25Gn_CFG			BIT(0)

#define LYNX_28G_PCCE				0x10b8
#define LYNX_28G_PCCE_E40Gn_LRV			BIT(3)
#define LYNX_28G_PCCE_E40Gn_CFG			BIT(0)
#define LYNX_28G_PCCE_E50Gn_LRV			BIT(3)
#define LYNX_28G_PCCE_E50GnCFG			BIT(0)
#define LYNX_28G_PCCE_E100Gn_LRV		BIT(3)
#define LYNX_28G_PCCE_E100Gn_CFG		BIT(0)

#define SGMII_CFG(id)			(28 - (id) * 4) /* Offset into PCC8 */
#define SXGMII_CFG(id)			(28 - (id) * 4) /* Offset into PCCC */
#define E25G_CFG(id)			(28 - (id) * 4) /* Offset into PCCD */
#define E40G_CFG(id)			(28 - (id) * 4) /* Offset into PCCE */
#define E50G_CFG(id)			(20 - (id) * 4) /* Offset into PCCE */
#define E100G_CFG(id)			(12 - (id) * 4) /* Offset into PCCE */

/* Per PLL registers */
#define LYNX_28G_PLLnRSTCTL(pll)		(0x400 + (pll) * 0x100 + 0x0)
#define LYNX_28G_PLLnRSTCTL_DIS(rstctl)		(((rstctl) & BIT(24)) >> 24)
#define LYNX_28G_PLLnRSTCTL_LOCK(rstctl)	(((rstctl) & BIT(23)) >> 23)

#define LYNX_28G_PLLnCR0(pll)			(0x400 + (pll) * 0x100 + 0x4)
#define LYNX_28G_PLLnCR0_REFCLK_SEL(cr0)	(((cr0) & GENMASK(20, 16)))
#define LYNX_28G_PLLnCR0_REFCLK_SEL_100MHZ	0x0
#define LYNX_28G_PLLnCR0_REFCLK_SEL_125MHZ	0x10000
#define LYNX_28G_PLLnCR0_REFCLK_SEL_156MHZ	0x20000
#define LYNX_28G_PLLnCR0_REFCLK_SEL_150MHZ	0x30000
#define LYNX_28G_PLLnCR0_REFCLK_SEL_161MHZ	0x40000

#define LYNX_28G_PLLnCR1(pll)			(0x400 + (pll) * 0x100 + 0x8)
#define LYNX_28G_PLLnCR1_FRATE_SEL(cr1)		(((cr1) & GENMASK(28, 24)))
#define LYNX_28G_PLLnCR1_FRATE_5G_10GVCO	0x0
#define LYNX_28G_PLLnCR1_FRATE_5G_25GVCO	0x10000000
#define LYNX_28G_PLLnCR1_FRATE_10G_20GVCO	0x6000000
#define LYNX_28G_PLLnCR1_FRATE_12G_25GVCO	0x16000000
#define LYNX_28G_PLLnCR1_EX_DLY_SEL(x)		((x) & GENMASK(1, 0))
#define  EX_DLY_SEL_MSK				LYNX_28G_PLLnCR1_EX_DLY_SEL(3)
#define  EX_DLY_SEL_312_5_MHZ			LYNX_28G_PLLnCR1_EX_DLY_SEL(2)

/* Per SerDes lane registers */
/* Lane a General Control Register */
#define LYNX_28G_LNaGCR0(lane)			(0x800 + (lane) * 0x100 + 0x0)
#define LYNX_28G_LNaGCR0_PROTO_SEL_MSK		GENMASK(7, 3)
#define LYNX_28G_LNaGCR0_PROTO_SEL_SGMII	0x8
#define LYNX_28G_LNaGCR0_PROTO_SEL_XFI		0x50
#define LYNX_28G_LNaGCR0_PROTO_SEL_25G		0xD0
#define LYNX_28G_LNaGCR0_IF_WIDTH_MSK		GENMASK(2, 0)
#define LYNX_28G_LNaGCR0_IF_WIDTH_10_BIT	0x0
#define LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT	0x2
#define LYNX_28G_LNaGCR0_IF_WIDTH_40_BIT	0x4

/* Lane a Tx Reset Control Register */
#define LYNX_28G_LNaTRSTCTL(lane)		(0x800 + (lane) * 0x100 + 0x20)
#define LYNX_28G_LNaTRSTCTL_DIS			BIT(24)
#define LYNX_28G_LNaTRSTCTL_STP_REQ		BIT(26)
#define LYNX_28G_LNaTRSTCTL_HLT_REQ		BIT(27)
#define LYNX_28G_LNaTRSTCTL_RST_DONE		BIT(30)
#define LYNX_28G_LNaTRSTCTL_RST_REQ		BIT(31)

/* Lane a Tx General Control Register */
#define LYNX_28G_LNaTGCR0(lane)			(0x800 + (lane) * 0x100 + 0x24)
#define LYNX_28G_LNaTGCR0_USE_PLLF		0x0
#define LYNX_28G_LNaTGCR0_USE_PLLS		BIT(28)
#define LYNX_28G_LNaTGCR0_USE_PLL_MSK		BIT(28)
#define LYNX_28G_LNaTGCR0_N_RATE_FULL		0x0
#define LYNX_28G_LNaTGCR0_N_RATE_HALF		0x1000000
#define LYNX_28G_LNaTGCR0_N_RATE_QUARTER	0x2000000
#define LYNX_28G_LNaTGCR0_N_RATE_DOUBLE		0x3000000
#define LYNX_28G_LNaTGCR0_N_RATE_MSK		GENMASK(26, 24)

#define LYNX_28G_LNaTGCR1(lane)			(0x800 + (lane) * 0x100 + 0x28)

#define LYNX_28G_LNaTECR0(lane)			(0x800 + (lane) * 0x100 + 0x30)

#define LYNX_28G_LNaTECR0_EQ_TYPE_MSK		GENMASK(30, 28)
#define LYNX_28G_LNaTECR0_EQ_TYPE(x)		(((x) << 28) & LYNX_28G_LNaTECR0_EQ_TYPE_MSK)
#define LYNX_28G_LNaTECR0_EQ_TYPE_X(x)		(((x) & LYNX_28G_LNaTECR0_EQ_TYPE_MSK) >> 28)
#define LYNX_28G_LNaTECR0_EQ_SGN_PREQ		BIT(23)
#define LYNX_28G_LNaTECR0_EQ_PREQ_MSK		GENMASK(19, 16)
#define LYNX_28G_LNaTECR0_EQ_PREQ(x)		(((x) << 16) & LYNX_28G_LNaTECR0_EQ_PREQ_MSK)
#define LYNX_28G_LNaTECR0_EQ_PREQ_X(x)		(((x) & LYNX_28G_LNaTECR0_EQ_PREQ_MSK) >> 16)
#define LYNX_28G_LNaTECR0_EQ_SGN_POST1Q		BIT(15)
#define LYNX_28G_LNaTECR0_EQ_POST1Q_MSK		GENMASK(12, 8)
#define LYNX_28G_LNaTECR0_EQ_POST1Q(x)		(((x) << 8) & LYNX_28G_LNaTECR0_EQ_POST1Q_MSK)
#define LYNX_28G_LNaTECR0_EQ_POST1Q_X(x)	(((x) & LYNX_28G_LNaTECR0_EQ_POST1Q_MSK) >> 8)
#define LYNX_28G_LNaTECR0_EQ_AMP_RED_MSK	GENMASK(5, 0)
#define LYNX_28G_LNaTECR0_EQ_AMP_RED(x)		((x) & LYNX_28G_LNaTECR0_EQ_AMP_RED_MSK)

#define LYNX_28G_LNaTECR1(lane)			(0x800 + (lane) * 0x100 + 0x34)

#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ_DRVR_DIS	BIT(31)
#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ_MSK	GENMASK(29, 24)
#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ(x)		(((x) << 24) & LYNX_28G_LNaTECR1_EQ_ADPT_EQ_MSK)
#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ_X(x)	(((x) & LYNX_28G_LNaTECR1_EQ_ADPT_EQ_MSK) >> 24)

/* Lane a Rx Reset Control Register */
#define LYNX_28G_LNaRRSTCTL(lane)		(0x800 + (lane) * 0x100 + 0x40)
#define LYNX_28G_LNaRRSTCTL_DIS			BIT(24)
#define LYNX_28G_LNaRRSTCTL_STP_REQ		BIT(26)
#define LYNX_28G_LNaRRSTCTL_HLT_REQ		BIT(27)
#define LYNX_28G_LNaRRSTCTL_RST_DONE		BIT(30)
#define LYNX_28G_LNaRRSTCTL_RST_REQ		BIT(31)
#define LYNX_28G_LNaRRSTCTL_CDR_LOCK		BIT(12)

/* Lane a Rx General Control Register */
#define LYNX_28G_LNaRGCR0(lane)			(0x800 + (lane) * 0x100 + 0x44)
#define LYNX_28G_LNaRGCR0_USE_PLLF		0x0
#define LYNX_28G_LNaRGCR0_USE_PLLS		BIT(28)
#define LYNX_28G_LNaRGCR0_USE_PLL_MSK		BIT(28)
#define LYNX_28G_LNaRGCR0_N_RATE_MSK		GENMASK(26, 24)
#define LYNX_28G_LNaRGCR0_N_RATE_FULL		0x0
#define LYNX_28G_LNaRGCR0_N_RATE_HALF		0x1000000
#define LYNX_28G_LNaRGCR0_N_RATE_QUARTER	0x2000000
#define LYNX_28G_LNaRGCR0_N_RATE_DOUBLE		0x3000000
#define LYNX_28G_LNaRGCR0_N_RATE_MSK		GENMASK(26, 24)
#define LYNX_28G_LNaRGCR0_INTACCPL_DIS		BIT(5)
#define LYNX_28G_LNaRGCR0_CMADJ_DIS		BIT(4)

#define LYNX_28G_LNaRGCR1(lane)			(0x800 + (lane) * 0x100 + 0x48)

#define LYNX_28G_LNaRGCR1_RX_ORD_ELECIDLE	BIT(31)
#define LYNX_28G_LNaRGCR1_DATA_LOST_FLT		BIT(30)
#define LYNX_28G_LNaRGCR1_DATA_LOST		BIT(29)
#define LYNX_28G_LNaRGCR1_IDLE_CONFIG		BIT(28)
#define LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL_MSK GENMASK(26, 24)
#define LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL(x)	(((x) << 24) & LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL_MSK)
#define LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL_X(x) (((x) & LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL_MSK) >> 24)
#define LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL_MSK GENMASK(22, 20)
#define LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL(x)	(((x) << 20) & LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL_MSK)
#define LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL_X(x)	(((x) & LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL_MSK) >> 20)
#define LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL_MSK	GENMASK(18, 16)
#define LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL(x)	(((x) << 16) & LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL_MSK)
#define LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL_X(x)	(((x) & LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL_MSK) >> 16)
#define LYNX_28G_LNaRGCR1_EXT_REC_CLK_SEL(x)	(((x) << 8) & GENMASK(10, 8))
#define LYNX_28G_LNaRGCR1_EXT_REC_CLK_SEL_X(x)	(((x) & GENMASK(10, 8)) >> 8)
#define LYNX_28G_LNaRGCR1_WAKE_TX_DIS		BIT(5)
#define LYNX_28G_LNaRGCR1_PHY_RDY		BIT(4)
#define LYNX_28G_LNaRGCR1_CHANGE_RX_CLK		BIT(3)
#define LYNX_28G_LNaRGCR1_PWR_MGT(x)		((x) & GENMASK(2, 0))

#define LYNX_28G_LNaRECR0(lane)			(0x800 + (lane) * 0x100 + 0x50)

#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_EN	BIT(31)
#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_MSK	GENMASK(28, 24)
#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV(x)	(((x) << 24) & LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_MSK)
#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_X(x)	(((x) & LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_MSK) >> 24)
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_EN	BIT(23)
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_MSK	GENMASK(20, 16)
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV(x)	(((x) << 16) & LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_MSK)
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_X(x)	(((x) & LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_MSK) >> 16)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_EN	BIT(7)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_DIS	BIT(6)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_MSK	GENMASK(4, 0)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV(x)	((x) & LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_MSK)

#define LYNX_28G_LNaRECR1(lane)			(0x800 + (lane) * 0x100 + 0x54)

#define LYNX_28G_LNaRECR1_EQ_BLW_OV_EN		BIT(31)
#define LYNX_28G_LNaRECR1_EQ_BLW_OV_MSK		GENMASK(28, 24)
#define LYNX_28G_LNaRECR1_EQ_BLW_OV(x)		(((x) << 24) & LYNX_28G_LNaRECR1_EQ_BLW_OV_MSK)
#define LYNX_28G_LNaRECR1_EQ_BLW_OV_X(x)	(((x) & LYNX_28G_LNaRECR1_EQ_BLW_OV_MSK) >> 24)
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV_EN	BIT(23)
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV_MSK	GENMASK(21, 16)
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV(x)	(((x) << 16) & LYNX_28G_LNaRECR1_EQ_OFFSET_OV_MSK)
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV_X(x)	(((x) & LYNX_28G_LNaRECR1_EQ_OFFSET_OV_MSK) >> 16)

#define LYNX_28G_LNaRECR2(lane)			(0x800 + (lane) * 0x100 + 0x58)

#define LYNX_28G_LNaRECR2_EQ_OFFSET_RNG_DBL	BIT(31)
#define LYNX_28G_LNaRECR2_EQ_BOOST_MSK		GENMASK(29, 28)
#define LYNX_28G_LNaRECR2_EQ_BOOST(x)		(((x) << 28) & LYNX_28G_LNaRECR2_EQ_BOOST_MSK)
#define LYNX_28G_LNaRECR2_EQ_BOOST_X(x)		(((x) & LYNX_28G_LNaRECR2_EQ_BOOST_MSK) >> 28)
#define LYNX_28G_LNaRECR2_EQ_BLW_SEL_MSK	GENMASK(25, 24)
#define LYNX_28G_LNaRECR2_EQ_BLW_SEL(x)		(((x) << 24) & LYNX_28G_LNaRECR2_EQ_BLW_SEL_MSK)
#define LYNX_28G_LNaRECR2_EQ_BLW_SEL_X(x)	(((x) & LYNX_28G_LNaRECR2_EQ_BLW_SEL_MSK) >> 24)
#define LYNX_28G_LNaRECR2_EQ_ZERO_MSK		GENMASK(17, 16)
#define LYNX_28G_LNaRECR2_EQ_ZERO(x)		(((x) << 16) & LYNX_28G_LNaRECR2_EQ_ZERO_MSK)
#define LYNX_28G_LNaRECR2_EQ_ZERO_X(x)		(((x) & LYNX_28G_LNaRECR2_EQ_ZERO_MSK) >> 16)
#define LYNX_28G_LNaRECR2_EQ_IND(x)		(((x) << 12) & GENMASK(13, 12))
#define LYNX_28G_LNaRECR2_EQ_IND_X(x)		(((x) & GENMASK(13, 12)) >> 12)
#define LYNX_28G_LNaRECR2_EQ_BIN_DATA_AVG_TC(x)	(((x) << 4) & GENMASK(5, 4))
#define LYNX_28G_LNaRECR2_EQ_BIN_DATA_AVG_TC_X(x) (((x) & GENMASK(5, 4)) >> 4)
#define LYNX_28G_LNaRECR2_SPARE_IN_MSK		GENMASK(1, 0)
#define LYNX_28G_LNaRECR2_SPARE_IN(x)		((x) & LYNX_28G_LNaRECR2_SPARE_IN_MSK)

#define LYNX_28G_LNaRECR3(lane)			(0x800 + (lane) * 0x100 + 0x5c)

#define LYNX_28G_LNaRECR3_EQ_SNAP_START		BIT(31)
#define LYNX_28G_LNaRECR3_EQ_SNAP_DONE		BIT(30)
#define LYNX_28G_LNaRECR3_EQ_GAINK2_HF_STAT_MSK	GENMASK(28, 24)
#define LYNX_28G_LNaRECR3_EQ_GAINK2_HF_STAT_X(x) (((x) & LYNX_28G_LNaRECR3_EQ_GAINK2_HF_STAT_MSK) >> 24)
#define LYNX_28G_LNaRECR3_EQ_GAINK3_MF_STAT_MSK	GENMASK(20, 16)
#define LYNX_28G_LNaRECR3_EQ_GAINK3_MF_STAT_X(x) (((x) & LYNX_28G_LNaRECR3_EQ_GAINK3_MF_STAT_MSK) >> 16)
#define LYNX_28G_LNaRECR3_SPARE_OUT_MSK		GENMASK(13, 12)
#define LYNX_28G_LNaRECR3_SPARE_OUT_X(x)	(((x) & LYNX_28G_LNaRECR3_SPARE_OUT_MSK) >> 12)
#define LYNX_28G_LNaRECR3_EQ_GAINK4_LF_STAT_MSK	GENMASK(4, 0)
#define LYNX_28G_LNaRECR3_EQ_GAINK4_LF_STAT_X(x) ((x) & LYNX_28G_LNaRECR3_EQ_GAINK4_LF_STAT_MSK)

#define LYNX_28G_LNaRECR4(lane)			(0x800 + (lane) * 0x100 + 0x60)

#define LYNX_28G_LNaRECR4_BLW_STAT_MSK		GENMASK(28, 24)
#define LYNX_28G_LNaRECR4_BLW_STAT_X(x)		(((x) & LYNX_28G_LNaRECR4_BLW_STAT_MSK) >> 24)
#define LYNX_28G_LNaRECR4_EQ_OFFSET_STAT_MSK	GENMASK(21, 16)
#define LYNX_28G_LNaRECR4_EQ_OFFSET_STAT_X(x)	(((x) & LYNX_28G_LNaRECR4_EQ_OFFSET_STAT_MSK) >> 16)
#define LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL_MSK	GENMASK(15, 12)
#define LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL(x)	(((x) << 12) & LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL_MSK)
#define LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL_X(x)	(((x) & LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL_MSK) >> 12)
#define LYNX_28G_LNaRECR4_EQ_BIN_DATA_MSK	GENMASK(8, 0) /* bit 9 is reserved */
#define LYNX_28G_LNaRECR4_EQ_BIN_DATA(x)	((x) & LYNX_28G_LNaRECR4_EQ_BIN_DATA_MSK)
#define LYNX_28G_LNaRECR4_EQ_BIN_DATA_SGN	BIT(8)

#define LYNX_28G_LNaRCCR0(lane)			(0x800 + (lane) * 0x100 + 0x68)

#define LYNX_28G_LNaRCCR0_CAL_EN		BIT(31)
#define LYNX_28G_LNaRCCR0_MEAS_EN		BIT(30)
#define LYNX_28G_LNaRCCR0_CAL_BIN_SEL		BIT(28)
#define LYNX_28G_LNaRCCR0_CAL_DC3_DIS		BIT(27)
#define LYNX_28G_LNaRCCR0_CAL_DC2_DIS		BIT(26)
#define LYNX_28G_LNaRCCR0_CAL_DC1_DIS		BIT(25)
#define LYNX_28G_LNaRCCR0_CAL_DC0_DIS		BIT(24)
#define LYNX_28G_LNaRCCR0_CAL_AC3_OV_EN		BIT(15)
#define LYNX_28G_LNaRCCR0_CAL_AC3_OV(x)		(((x) << 8) & GENMASK(11, 8))
#define LYNX_28G_LNaRCCR0_CAL_AC3_OV_X(x)	(((x) & GENMASK(11, 8)) >> 8)
#define LYNX_28G_LNaRCCR0_CAL_AC2_OV_EN		BIT(7)
#define LYNX_28G_LNaRCCR0_CAL_AC2_OV(x)		((x) & GENMASK(3, 0))

#define LYNX_28G_LNaRSCCR0(lane)		(0x800 + (lane) * 0x100 + 0x74)

#define LYNX_28G_LNaRSCCR0_SMP_OFF_EN		BIT(31)
#define LYNX_28G_LNaRSCCR0_SMP_OFF_OV_EN	BIT(30)
#define LYNX_28G_LNaRSCCR0_SMP_MAN_OFF_EN	BIT(29)
#define LYNX_28G_LNaRSCCR0_SMP_OFF_RNG_OV_EN	BIT(27)
#define LYNX_28G_LNaRSCCR0_SMP_OFF_RNG_4X_OV	BIT(25)
#define LYNX_28G_LNaRSCCR0_SMP_OFF_RNG_2X_OV	BIT(24)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_PD		BIT(23)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL_MSK	GENMASK(19, 16)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL(x)	(((x) << 16) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL_MSK)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL_X(x)	(((x) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL_MSK) >> 16)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R_MSK	GENMASK(13, 12)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R(x)	(((x) << 12) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R_MSK)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R_X(x)	(((x) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R_MSK) >> 12)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F_MSK	GENMASK(9, 8)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F(x)	(((x) << 8) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F_MSK)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F_X(x)	(((x) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F_MSK) >> 8)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R_MSK	GENMASK(5, 4)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R(x)	(((x) << 4) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R_MSK)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R_X(x)	(((x) & LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R_MSK) >> 4)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1F(x)	((x) & GENMASK(1, 0))

#define LYNX_28G_LNaTTLCR0(lane)		(0x800 + (lane) * 0x100 + 0x80)

#define LYNX_28G_LNaTTLCR0_TTL_FLT_SEL(x)	(((x) << 24) & GENMASK(29, 24))
#define LYNX_28G_LNaTTLCR0_TTL_FLT_SEL_X(x)	(((x) & GENMASK(29, 24)) >> 24)
#define LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP	BIT(22)
#define LYNX_28G_LNaTTLCR0_STALL_DET_DIS		BIT(21)
#define LYNX_28G_LNaTTLCR0_INACT_MON_DIS		BIT(20)
#define LYNX_28G_LNaTTLCR0_CDR_OV(x)		(((x) << 16) & GENMASK(18, 16))
#define LYNX_28G_LNaTTLCR0_CDR_OV_X(x)		(((x) & GENMASK(18, 16)) >> 16)
#define LYNX_28G_LNaTTLCR0_DATA_IN_SSC		BIT(15)
#define LYNX_28G_LNaTTLCR0_CDR_MIN_SMP_ON(x)	((x) & GENMASK(1, 0))

#define LYNX_28G_LNaTCSR0(lane)			(0x800 + (lane) * 0x100 + 0xa0)

#define LYNX_28G_LNaTCSR0_SD_STAT_OBS_EN	BIT(31)
#define LYNX_28G_LNaTCSR0_SD_LPBK_SEL_MSK	GENMASK(29, 28)
#define LYNX_28G_LNaTCSR0_SD_LPBK_SEL_X(x)	(((x) & LYNX_28G_LNaTCSR0_SD_LPBK_SEL_MSK) >> 28)
#define LYNX_28G_LNaTCSR0_SD_LPBK_SEL(x)	(((x) << 28) & LYNX_28G_LNaTCSR0_SD_LPBK_SEL_MSK)

#define LYNX_28G_LNaPSS(lane)			(0x1000 + (lane) * 0x4)
#define LYNX_28G_LNaPSS_TYPE(pss)		(((pss) & GENMASK(30, 24)) >> 24)
#define LYNX_28G_LNaPSS_TYPE_SGMII		(PROTO_SEL_SGMII_BASEX_KX << 2)
#define LYNX_28G_LNaPSS_TYPE_XFI		(PROTO_SEL_XFI_10GBASER_KR_SXGMII << 2)
#define LYNX_28G_LNaPSS_TYPE_40G		((PROTO_SEL_XFI_10GBASER_KR_SXGMII << 2) | 3)
#define LYNX_28G_LNaPSS_TYPE_25G		(PROTO_SEL_25G_50G_100G << 2)
#define LYNX_28G_LNaPSS_TYPE_100G		((PROTO_SEL_25G_50G_100G << 2) | 2)

/* MDEV_PORT is at the same bitfield address for all protocol converters */
#define LYNX_28G_MDEV_PORT_MSK			GENMASK(31, 27)
#define LYNX_28G_MDEV_PORT_X(x)			(((x) & LYNX_28G_MDEV_PORT_MSK) >> 27)

#define LYNX_28G_SGMIIaCR0(lane)		(0x1800 + (lane) * 0x10)
#define LYNX_28G_SGMIIaCR1(lane)		(0x1804 + (lane) * 0x10)
#define LYNX_28G_SGMIIaCR1_SGPCS_EN		BIT(11)
#define LYNX_28G_SGMIIaCR1_SGPCS_MSK		BIT(11)

#define LYNX_28G_ANLTaCR0(lane)			(0x1a00 + (lane) * 0x10)
#define LYNX_28G_ANLTaCR1(lane)			(0x1a04 + (lane) * 0x10)

#define LYNX_28G_SXGMIIaCR0(lane)		(0x1a80 + (lane) * 0x10)
#define LYNX_28G_SXGMIIaCR0_RST			BIT(31)
#define LYNX_28G_SXGMIIaCR0_PD			BIT(30)

#define LYNX_28G_SXGMIIaCR1(lane)		(0x1a84 + (lane) * 0x10)

#define LYNX_28G_E25GaCR0(lane)			(0x1b00 + (lane) * 0x10)
#define LYNX_28G_E25GaCR0_RST			BIT(31)
#define LYNX_28G_E25GaCR0_PD			BIT(30)

#define LYNX_28G_E25GaCR1(lane)			(0x1b04 + (lane) * 0x10)

#define LYNX_28G_E25GaCR2(lane)			(0x1b08 + (lane) * 0x10)
#define LYNX_28G_E25GaCR2_FEC_ENA		BIT(23)
#define LYNX_28G_E25GaCR2_FEC_ERR_ENA		BIT(22)
#define LYNX_28G_E25GaCR2_FEC91_ENA		BIT(20)

#define LYNX_28G_E40GaCR0(pcvt)			(0x1b40 + (pcvt) * 0x20)
#define LYNX_28G_E40GaCR1(pcvt)			(0x1b44 + (pcvt) * 0x20)

#define LYNX_28G_E50GaCR1(pcvt)			(0x1b84 + (pcvt) * 0x10)

#define LYNX_28G_E100GaCR1(pcvt)		(0x1c04 + (pcvt) * 0x20)

#define CR(x)					((x) * 4)

#define LYNX_28G_CDR_SLEEP_US			50
#define LYNX_28G_CDR_TIMEOUT_US			500

#define LYNX_28G_SNAPSHOT_SLEEP_US		1
#define LYNX_28G_SNAPSHOT_TIMEOUT_US		1000

enum lynx_28g_eq_bin_data_type {
	EQ_BIN_DATA_SEL_BIN_1 = 0,
	EQ_BIN_DATA_SEL_BIN_2 = 1,
	EQ_BIN_DATA_SEL_BIN_3 = 2,
	EQ_BIN_DATA_SEL_BIN_4 = 3,
	EQ_BIN_DATA_SEL_OFFSET = 4,
	EQ_BIN_DATA_SEL_BIN_BLW = 8,
	EQ_BIN_DATA_SEL_BIN_DATA_AVG = 9,
	EQ_BIN_DATA_SEL_BIN_M1 = 0xc,
	EQ_BIN_DATA_SEL_BIN_LONG = 0xd,
};

enum lynx_28g_eq_type {
	EQ_TYPE_NO_EQ = 0,
	EQ_TYPE_2TAP = 1,
	EQ_TYPE_3TAP = 2,
};

enum lynx_28g_proto_sel {
	PROTO_SEL_PCIE = 0,
	PROTO_SEL_SGMII_BASEX_KX = 1,
	PROTO_SEL_SATA = 2,
	PROTO_SEL_XAUI = 4,
	PROTO_SEL_XFI_10GBASER_KR_SXGMII = 0xa,
	PROTO_SEL_25G_50G_100G = 0x1a,
};

enum lynx_28g_lane_mode {
	LANE_MODE_UNKNOWN,
	LANE_MODE_1000BASEX_SGMII,
	LANE_MODE_10GBASER,
	LANE_MODE_USXGMII,
	LANE_MODE_25GBASER,
	LANE_MODE_1000BASEKX,
	LANE_MODE_10GBASEKR,
	LANE_MODE_25GBASEKR,
	LANE_MODE_40GBASER_XLAUI,
	LANE_MODE_40GBASEKR4,
	LANE_MODE_MAX,
};

struct lynx_28g_proto_conf {
	/* LNaGCR0 */
	int proto_sel;
	int if_width;
	/* LNaTECR0 */
	int teq_type;
	int sgn_preq;
	int ratio_preq;
	int sgn_post1q;
	int ratio_post1q;
	int amp_red;
	/* LNaTECR1 */
	int adpt_eq;
	/* LNaRGCR1 */
	int enter_idle_flt_sel;
	int exit_idle_flt_sel;
	int data_lost_th_sel;
	/* LNaRECR0 */
	int gk2ovd;
	int gk3ovd;
	int gk4ovd;
	int gk2ovd_en;
	int gk3ovd_en;
	int gk4ovd_en;
	/* LNaRECR1 ? */
	int eq_offset_ovd;
	int eq_offset_ovd_en;
	/* LNaRECR2 */
	int eq_offset_rng_dbl;
	int eq_blw_sel;
	int eq_boost;
	int spare_in;
	/* LNaRSCCR0 */
	int smp_autoz_d1r;
	int smp_autoz_eg1r;
	/* LNaRCCR0 */
	int rccr0;
	/* LNaTTLCR0 */
	int ttlcr0;
};

static const struct lynx_28g_proto_conf lynx_28g_proto_conf[LANE_MODE_MAX] = {
	[LANE_MODE_1000BASEX_SGMII] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_SGMII,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_10_BIT,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 0,
		.amp_red = 6,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 4,
		.exit_idle_flt_sel = 3,
		.data_lost_th_sel = 1,
		.gk2ovd = 0x1f,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 1,
		.gk3ovd_en = 1,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 0,
		.eq_blw_sel = 0,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 0,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_1000BASEKX] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_SGMII,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_10_BIT,
		.teq_type = EQ_TYPE_NO_EQ,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 0,
		.amp_red = 0,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0x1f,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 1,
		.gk3ovd_en = 1,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 0,
		.eq_blw_sel = 0,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 0,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_USXGMII] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_XFI,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 3,
		.amp_red = 7,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_10GBASER] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_XFI,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_2TAP,
		.sgn_preq = 1,
		.ratio_preq = 0,
		.sgn_post1q = 1,
		.ratio_post1q = 3,
		.amp_red = 7,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_10GBASEKR] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_XFI,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.amp_red = 0,
		.adpt_eq = 41,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_25GBASER] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_25G,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_40_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 7,
		.amp_red = 0,
		.adpt_eq = 48,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 5,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 1,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 2,
		.spare_in = 3,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 2,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN |
			 LYNX_28G_LNaRCCR0_CAL_DC3_DIS |
			 LYNX_28G_LNaRCCR0_CAL_DC2_DIS |
			 LYNX_28G_LNaRCCR0_CAL_DC1_DIS |
			 LYNX_28G_LNaRCCR0_CAL_DC0_DIS,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_DATA_IN_SSC |
			  LYNX_28G_LNaTTLCR0_CDR_MIN_SMP_ON(1),
	},
	[LANE_MODE_25GBASEKR] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_25G,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_40_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 7,
		.amp_red = 0, // FIXME 32 for C2C?
		.adpt_eq = 38,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 5,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 1,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 2,
		.spare_in = 3,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 2,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN |
			 LYNX_28G_LNaRCCR0_CAL_DC3_DIS |
			 LYNX_28G_LNaRCCR0_CAL_DC2_DIS |
			 LYNX_28G_LNaRCCR0_CAL_DC1_DIS |
			 LYNX_28G_LNaRCCR0_CAL_DC0_DIS,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_DATA_IN_SSC |
			  LYNX_28G_LNaTTLCR0_CDR_MIN_SMP_ON(1),
	},
	[LANE_MODE_40GBASER_XLAUI] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_XFI,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.amp_red = 0,
		.adpt_eq = 41,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
	[LANE_MODE_40GBASEKR4] = {
		.proto_sel = LYNX_28G_LNaGCR0_PROTO_SEL_XFI,
		.if_width = LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT,
		.teq_type = EQ_TYPE_3TAP,
		.sgn_preq = 1,
		.ratio_preq = 2,
		.sgn_post1q = 1,
		.ratio_post1q = 5,
		.amp_red = 0,
		.adpt_eq = 41,
		.enter_idle_flt_sel = 0,
		.exit_idle_flt_sel = 0,
		.data_lost_th_sel = 0,
		.gk2ovd = 0,
		.gk3ovd = 0,
		.gk4ovd = 0,
		.gk2ovd_en = 0,
		.gk3ovd_en = 0,
		.gk4ovd_en = 0,
		.eq_offset_ovd = 0x1f,
		.eq_offset_ovd_en = 0,
		.eq_offset_rng_dbl = 1,
		.eq_blw_sel = 1,
		.eq_boost = 0,
		.spare_in = 0,
		.smp_autoz_d1r = 2,
		.smp_autoz_eg1r = 0,
		.rccr0 = LYNX_28G_LNaRCCR0_CAL_EN,
		.ttlcr0 = LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			  LYNX_28G_LNaTTLCR0_DATA_IN_SSC,
	},
};

struct lynx_pccr {
	int offset;
	int width;
	int shift;
};

struct lynx_28g_priv;

struct lynx_28g_pll {
	struct lynx_28g_priv *priv;
	u32 rstctl, cr0, cr1;
	int id;
	int ex_dly_clk_use_count;
	DECLARE_BITMAP(supported, LANE_MODE_MAX);
	/*
	 * There are fewer PLLs than lanes. This serializes calls to
	 * lynx_28g_pll_get_ex_dly_clk() and lynx_28g_pll_put_ex_dly_clk().
	 */
	spinlock_t lock;
};

struct lynx_28g_lane {
	struct lynx_28g_priv *priv;
	struct phy *phy;
	bool powered_up;
	bool init;
	unsigned int id;
	enum lynx_28g_lane_mode mode;
	struct lynx_xgkr_algorithm *algorithm;
};

struct lynx_info {
	int (*get_pccr)(enum lynx_28g_lane_mode lane_mode, int lane,
			struct lynx_pccr *pccr);
	int (*get_pcvt_offset)(int lane, enum lynx_28g_lane_mode mode);
	bool (*lane_supports_mode)(int lane, enum lynx_28g_lane_mode mode);
	int first_lane;
};

struct lynx_28g_priv {
	void __iomem *base;
	struct device *dev;
	const struct lynx_info *info;
	/* Serialize concurrent access to registers shared between lanes,
	 * like PCCn
	 */
	spinlock_t pcc_lock;
	struct lynx_28g_pll pll[LYNX_28G_NUM_PLL];
	struct lynx_28g_lane lane[LYNX_28G_NUM_LANE];

	struct delayed_work cdr_check;
};

static const int lynx_28g_bin_type_to_bin_sel[] = {
	[BIN_1] = EQ_BIN_DATA_SEL_BIN_1,
	[BIN_2] = EQ_BIN_DATA_SEL_BIN_2,
	[BIN_3] = EQ_BIN_DATA_SEL_BIN_3,
	[BIN_4] = EQ_BIN_DATA_SEL_BIN_4,
	[BIN_OFFSET] = EQ_BIN_DATA_SEL_OFFSET,
	[BIN_M1] = EQ_BIN_DATA_SEL_BIN_M1,
	[BIN_LONG] = EQ_BIN_DATA_SEL_BIN_LONG,
};

static void lynx_28g_rmw(struct lynx_28g_priv *priv, unsigned long off,
			 u32 val, u32 mask)
{
	void __iomem *reg = priv->base + off;
	u32 orig, tmp;

	orig = ioread32(reg);
	tmp = orig & ~mask;
	tmp |= val;
	iowrite32(tmp, reg);
}

#define lynx_28g_read(priv, off) \
	ioread32((priv)->base + (off))
#define lynx_28g_write(priv, off, val) \
	iowrite32(val, (priv)->base + (off))
#define lynx_28g_lane_rmw(lane, reg, val, mask)	\
	lynx_28g_rmw((lane)->priv, LYNX_28G_##reg(lane->id), val, mask)
#define lynx_28g_lane_read(lane, reg)			\
	ioread32((lane)->priv->base + LYNX_28G_##reg((lane)->id))
#define lynx_28g_lane_write(lane, reg, val)		\
	iowrite32(val, (lane)->priv->base + LYNX_28G_##reg((lane)->id))
#define lynx_28g_pll_read(pll, reg)			\
	ioread32((pll)->priv->base + LYNX_28G_##reg((pll)->id))
#define lynx_28g_pll_write(pll, reg, val)		\
	iowrite32(val, (pll)->priv->base + LYNX_28G_##reg((pll)->id))

static enum lynx_28g_lane_mode phy_interface_to_lane_mode(phy_interface_t intf)
{
	switch (intf) {
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_1000BASEX:
		return LANE_MODE_1000BASEX_SGMII;
	case PHY_INTERFACE_MODE_10GBASER:
		return LANE_MODE_10GBASER;
	case PHY_INTERFACE_MODE_USXGMII:
		return LANE_MODE_USXGMII;
	case PHY_INTERFACE_MODE_25GBASER:
		return LANE_MODE_25GBASER;
	/* TODO: PHY_INTERFACE_MODE_40GBASER to LANE_MODE_40GBASER_XLAUI */
	default:
		return LANE_MODE_UNKNOWN;
	}
}

static enum lynx_28g_lane_mode
link_mode_to_lane_mode(enum ethtool_link_mode_bit_indices link_mode)
{
	switch (link_mode) {
	case ETHTOOL_LINK_MODE_1000baseKX_Full_BIT:
		return LANE_MODE_1000BASEKX;
	case ETHTOOL_LINK_MODE_10000baseKR_Full_BIT:
		return LANE_MODE_10GBASEKR;
	case ETHTOOL_LINK_MODE_25000baseKR_Full_BIT:
	case ETHTOOL_LINK_MODE_25000baseKR_S_Full_BIT:
		return LANE_MODE_25GBASEKR;
	case ETHTOOL_LINK_MODE_40000baseKR4_Full_BIT:
		return LANE_MODE_40GBASEKR4;
	default:
		return LANE_MODE_UNKNOWN;
	}
}

static bool lynx_28g_supports_lane_mode(struct lynx_28g_priv *priv,
					enum lynx_28g_lane_mode mode)
{
	int i;

	for (i = 0; i < LYNX_28G_NUM_PLL; i++) {
		if (LYNX_28G_PLLnRSTCTL_DIS(priv->pll[i].rstctl))
			continue;

		if (test_bit(mode, priv->pll[i].supported))
			return true;
	}

	return false;
}

static int lynx_28g_lane_mode_num_lanes(enum lynx_28g_lane_mode lane_mode)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		return 1;
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		return 4;
	default:
		return -EOPNOTSUPP;
	}
}

static struct lynx_28g_pll *lynx_28g_pll_get(struct lynx_28g_priv *priv,
					     enum lynx_28g_lane_mode mode)
{
	struct lynx_28g_pll *pll;
	int i;

	for (i = 0; i < LYNX_28G_NUM_PLL; i++) {
		pll = &priv->pll[i];

		if (LYNX_28G_PLLnRSTCTL_DIS(pll->rstctl))
			continue;

		if (test_bit(mode, pll->supported))
			return pll;
	}

	return NULL;
}

static void lynx_28g_lane_set_nrate(struct lynx_28g_lane *lane,
				    struct lynx_28g_pll *pll,
				    enum lynx_28g_lane_mode mode)
{
	switch (LYNX_28G_PLLnCR1_FRATE_SEL(pll->cr1)) {
	case LYNX_28G_PLLnCR1_FRATE_5G_10GVCO:
	case LYNX_28G_PLLnCR1_FRATE_5G_25GVCO:
		switch (mode) {
		case LANE_MODE_1000BASEX_SGMII:
		case LANE_MODE_1000BASEKX:
			lynx_28g_lane_rmw(lane, LNaTGCR0,
					  LYNX_28G_LNaTGCR0_N_RATE_QUARTER,
					  LYNX_28G_LNaTGCR0_N_RATE_MSK);
			lynx_28g_lane_rmw(lane, LNaRGCR0,
					  LYNX_28G_LNaRGCR0_N_RATE_QUARTER,
					  LYNX_28G_LNaRGCR0_N_RATE_MSK);
			break;
		default:
			break;
		}
		break;
	case LYNX_28G_PLLnCR1_FRATE_10G_20GVCO:
		switch (mode) {
		case LANE_MODE_10GBASER:
		case LANE_MODE_USXGMII:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_40GBASER_XLAUI:
		case LANE_MODE_40GBASEKR4:
			lynx_28g_lane_rmw(lane, LNaTGCR0,
					  LYNX_28G_LNaTGCR0_N_RATE_FULL,
					  LYNX_28G_LNaTGCR0_N_RATE_MSK);
			lynx_28g_lane_rmw(lane, LNaRGCR0,
					  LYNX_28G_LNaRGCR0_N_RATE_FULL,
					  LYNX_28G_LNaRGCR0_N_RATE_MSK);
			break;
		default:
			break;
		}
		break;
	case LYNX_28G_PLLnCR1_FRATE_12G_25GVCO:
		switch (mode) {
		case LANE_MODE_25GBASER:
		case LANE_MODE_25GBASEKR:
			lynx_28g_lane_rmw(lane, LNaTGCR0,
					  LYNX_28G_LNaTGCR0_N_RATE_DOUBLE,
					  LYNX_28G_LNaTGCR0_N_RATE_MSK);
			lynx_28g_lane_rmw(lane, LNaRGCR0,
					  LYNX_28G_LNaRGCR0_N_RATE_DOUBLE,
					  LYNX_28G_LNaRGCR0_N_RATE_MSK);
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void lynx_28g_lane_set_pll(struct lynx_28g_lane *lane,
				  struct lynx_28g_pll *pll)
{
	if (pll->id == 0) {
		lynx_28g_lane_rmw(lane, LNaTGCR0, LYNX_28G_LNaTGCR0_USE_PLLF,
				  LYNX_28G_LNaTGCR0_USE_PLL_MSK);
		lynx_28g_lane_rmw(lane, LNaRGCR0, LYNX_28G_LNaRGCR0_USE_PLLF,
				  LYNX_28G_LNaRGCR0_USE_PLL_MSK);
	} else {
		lynx_28g_lane_rmw(lane, LNaTGCR0, LYNX_28G_LNaTGCR0_USE_PLLS,
				  LYNX_28G_LNaTGCR0_USE_PLL_MSK);
		lynx_28g_lane_rmw(lane, LNaRGCR0, LYNX_28G_LNaRGCR0_USE_PLLS,
				  LYNX_28G_LNaRGCR0_USE_PLL_MSK);
	}
}

static bool lynx_28g_cdr_lock_check(struct lynx_28g_lane *lane)
{
	u32 rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);

	if (rrstctl & LYNX_28G_LNaRRSTCTL_CDR_LOCK)
		return true;

	/* Omit resetting the receiver unless the lane is up. Otherwise,
	 * if powered down, it won't complete the operation.
	 */
	if (!lane->init || !lane->powered_up)
		return false;

	dev_dbg(&lane->phy->dev, "CDR unlocked, resetting lane receiver...\n");

	lynx_28g_lane_rmw(lane, LNaRRSTCTL,
			  LYNX_28G_LNaRRSTCTL_RST_REQ,
			  LYNX_28G_LNaRRSTCTL_RST_REQ);
	do {
		rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);
	} while (!(rrstctl & LYNX_28G_LNaRRSTCTL_RST_DONE));

	return !!(rrstctl & LYNX_28G_LNaRRSTCTL_CDR_LOCK);
}

/* Halting puts the lane in a mode in which it can be reconfigured */
static void lynx_28g_lane_halt(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	u32 trstctl, rrstctl;

	/* Issue a halt request */
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LYNX_28G_LNaTRSTCTL_HLT_REQ,
			  LYNX_28G_LNaTRSTCTL_HLT_REQ);
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LYNX_28G_LNaRRSTCTL_HLT_REQ,
			  LYNX_28G_LNaRRSTCTL_HLT_REQ);

	/* Wait until the halting process is complete */
	do {
		trstctl = lynx_28g_lane_read(lane, LNaTRSTCTL);
		rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);
	} while ((trstctl & LYNX_28G_LNaTRSTCTL_HLT_REQ) ||
		 (rrstctl & LYNX_28G_LNaRRSTCTL_HLT_REQ));
}

static void lynx_28g_lane_reset(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	u32 trstctl, rrstctl;

	/* Issue a reset request on the lane */
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LYNX_28G_LNaTRSTCTL_RST_REQ,
			  LYNX_28G_LNaTRSTCTL_RST_REQ);
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LYNX_28G_LNaRRSTCTL_RST_REQ,
			  LYNX_28G_LNaRRSTCTL_RST_REQ);

	/* Wait until the reset sequence is completed */
	do {
		trstctl = lynx_28g_lane_read(lane, LNaTRSTCTL);
		rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);
	} while (!(trstctl & LYNX_28G_LNaTRSTCTL_RST_DONE) ||
		 !(rrstctl & LYNX_28G_LNaRRSTCTL_RST_DONE));
}

static int lynx_28g_power_off(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	u32 trstctl, rrstctl;

	if (!lane->powered_up)
		return 0;

	/* Issue a stop request */
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LYNX_28G_LNaTRSTCTL_STP_REQ,
			  LYNX_28G_LNaTRSTCTL_STP_REQ);
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LYNX_28G_LNaRRSTCTL_STP_REQ,
			  LYNX_28G_LNaRRSTCTL_STP_REQ);

	/* Wait until the stop process is complete */
	do {
		trstctl = lynx_28g_lane_read(lane, LNaTRSTCTL);
		rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);
	} while ((trstctl & LYNX_28G_LNaTRSTCTL_STP_REQ) ||
		 (rrstctl & LYNX_28G_LNaRRSTCTL_STP_REQ));

	/* Power down the RX and TX portions of the lane */
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, LYNX_28G_LNaRRSTCTL_DIS,
			  LYNX_28G_LNaRRSTCTL_DIS);
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, LYNX_28G_LNaTRSTCTL_DIS,
			  LYNX_28G_LNaTRSTCTL_DIS);

	lane->powered_up = false;

	return 0;
}

static int lynx_28g_power_on(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	if (lane->powered_up)
		return 0;

	/* Power up the RX and TX portions of the lane */
	lynx_28g_lane_rmw(lane, LNaRRSTCTL, 0, LYNX_28G_LNaRRSTCTL_DIS);
	lynx_28g_lane_rmw(lane, LNaTRSTCTL, 0, LYNX_28G_LNaTRSTCTL_DIS);

	lynx_28g_lane_reset(phy);

	lane->powered_up = true;

	return 0;
}

static int lynx_28g_e25g_pcvt(int lane)
{
	return 7 - lane;
}

static int lynx_28g_e40g_pcvt(int lane)
{
	return lane < 4 ? 1 : 0;
}

static int lynx_28g_get_pccr(enum lynx_28g_lane_mode lane_mode, int lane,
			     struct lynx_pccr *pccr)
{
	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		pccr->offset = LYNX_28G_PCC8;
		pccr->width = 4;
		pccr->shift = SGMII_CFG(lane);
		break;
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		pccr->offset = LYNX_28G_PCCC;
		pccr->width = 4;
		pccr->shift = SXGMII_CFG(lane);
		break;
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		pccr->offset = LYNX_28G_PCCD;
		pccr->width = 4;
		pccr->shift = E25G_CFG(lynx_28g_e25g_pcvt(lane));
		break;
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		pccr->offset = LYNX_28G_PCCE;
		pccr->width = 4;
		pccr->shift = E40G_CFG(lynx_28g_e40g_pcvt(lane));
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int lynx_28g_get_pcvt_offset(int lane, enum lynx_28g_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		return LYNX_28G_SGMIIaCR0(lane);
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return LYNX_28G_SXGMIIaCR0(lane);
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		return LYNX_28G_E25GaCR0(lynx_28g_e25g_pcvt(lane));
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		return LYNX_28G_E40GaCR0(lynx_28g_e40g_pcvt(lane));
	default:
		return -EOPNOTSUPP;
	}
}

static int lynx_28g_get_anlt_offset(int lane, enum lynx_28g_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_25GBASEKR:
	case LANE_MODE_40GBASEKR4:
		return LYNX_28G_ANLTaCR0(lane);
	default:
		return -EOPNOTSUPP;
	}
}

static bool lx2160a_serdes1_lane_supports_mode(int lane,
					       enum lynx_28g_lane_mode mode)
{
	return true;
}

static bool lx2160a_serdes2_lane_supports_mode(int lane,
					       enum lynx_28g_lane_mode mode)
{
	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		return true;
	case LANE_MODE_USXGMII:
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		return lane == 6 || lane == 7;
	default:
		return false;
	}
}

static bool lx2160a_serdes3_lane_supports_mode(int lane,
					       enum lynx_28g_lane_mode mode)
{
	/*
	 * Non-networking SerDes, and this driver supports only
	 * networking protocols
	 */
	return false;
}

static bool lx2162a_serdes1_lane_supports_mode(int lane,
					       enum lynx_28g_lane_mode mode)
{
	return true;
}

static bool lx2162a_serdes2_lane_supports_mode(int lane,
					       enum lynx_28g_lane_mode mode)
{
	return lx2160a_serdes2_lane_supports_mode(lane, mode);
}

static const struct lynx_info lynx_info_lx2160a_serdes1 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2160a_serdes1_lane_supports_mode,
};

static const struct lynx_info lynx_info_lx2160a_serdes2 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2160a_serdes2_lane_supports_mode,
};

static const struct lynx_info lynx_info_lx2160a_serdes3 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2160a_serdes3_lane_supports_mode,
};

static const struct lynx_info lynx_info_lx2162a_serdes1 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2162a_serdes1_lane_supports_mode,
	.first_lane = 4,
};

static const struct lynx_info lynx_info_lx2162a_serdes2 = {
	.get_pccr = lynx_28g_get_pccr,
	.get_pcvt_offset = lynx_28g_get_pcvt_offset,
	.lane_supports_mode = lx2162a_serdes2_lane_supports_mode,
};

static int lynx_pccr_read(struct lynx_28g_lane *lane, enum lynx_28g_lane_mode mode,
			  u32 *val)
{
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_pccr pccr;
	u32 tmp;
	int err;

	err = lynx_28g_get_pccr(mode, lane->id, &pccr);
	if (err)
		return err;

	tmp = lynx_28g_read(priv, pccr.offset);
	*val = (tmp >> pccr.shift) & GENMASK(pccr.width - 1, 0);

	return 0;
}

static int lynx_pccr_write(struct lynx_28g_lane *lane,
			   enum lynx_28g_lane_mode mode, u32 val)
{
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_pccr pccr;
	u32 old, tmp, mask;
	int err;

	err = lynx_28g_get_pccr(mode, lane->id, &pccr);
	if (err)
		return err;

	old = lynx_28g_read(priv, pccr.offset);
	mask = GENMASK(pccr.width - 1, 0) << pccr.shift;
	tmp = (old & ~mask) | (val << pccr.shift);
	lynx_28g_write(priv, pccr.offset, tmp);

	dev_dbg(&lane->phy->dev, "PCCR@0x%x: 0x%x -> 0x%x\n",
		pccr.offset, old, tmp);

	return 0;
}

static int lynx_pcvt_read(struct lynx_28g_lane *lane, enum lynx_28g_lane_mode mode,
			  int cr, u32 *val)
{
	struct lynx_28g_priv *priv = lane->priv;
	int offset;

	offset = lynx_28g_get_pcvt_offset(lane->id, mode);
	if (offset < 0)
		return offset;

	*val = lynx_28g_read(priv, offset + cr);

	return 0;
}

static int lynx_pcvt_write(struct lynx_28g_lane *lane, enum lynx_28g_lane_mode mode,
			   int cr, u32 val)
{
	struct lynx_28g_priv *priv = lane->priv;
	int offset;

	offset = lynx_28g_get_pcvt_offset(lane->id, mode);
	if (offset < 0)
		return offset;

	lynx_28g_write(priv, offset + cr, val);

	return 0;
}

static int lynx_pcvt_rmw(struct lynx_28g_lane *lane, enum lynx_28g_lane_mode mode,
			 int cr, u32 val, u32 mask)
{
	int err;
	u32 tmp;

	err = lynx_pcvt_read(lane, mode, cr, &tmp);
	if (err)
		return err;

	tmp &= ~mask;
	tmp |= val;

	return lynx_pcvt_write(lane, mode, cr, tmp);
}

static int lynx_anlt_read(struct lynx_28g_lane *lane, enum lynx_28g_lane_mode mode,
			  int cr, u32 *val)
{
	struct lynx_28g_priv *priv = lane->priv;
	int offset;

	switch (mode) {
	case LANE_MODE_1000BASEKX:
	case LANE_MODE_10GBASEKR:
		/* For 1G and 10G, AN/LT registers are merged with the PCS */
		return lynx_pcvt_read(lane, mode, cr, val);
	case LANE_MODE_25GBASEKR:
	case LANE_MODE_40GBASEKR4:
		offset = lynx_28g_get_anlt_offset(lane->id, mode);
		if (offset < 0)
			return offset;

		*val = lynx_28g_read(priv, offset + cr);

		return 0;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

/* Enabling ex_dly_clk does not require turning the PLL off, and does not
 * affect the state of the lanes mapped to it. It is one of the few safe things
 * that can be done with it at runtime.
 */
static void lynx_28g_pll_ex_dly_clk_enable(struct lynx_28g_pll *pll,
					   bool enable)
{
	dev_dbg(pll->priv->dev, "Turning %s EX_DLY_CLK on PLL%c\n",
		enable ? "on" : "off", pll->id == 0 ? 'F' : 'S');

	pll->cr1 &= ~EX_DLY_SEL_MSK;
	if (enable)
		pll->cr1 |= EX_DLY_SEL_312_5_MHZ;

	lynx_28g_pll_write(pll, PLLnCR1, pll->cr1);
}

static void lynx_28g_pll_get_ex_dly_clk(struct lynx_28g_pll *pll)
{
	spin_lock(&pll->lock);

	if (++pll->ex_dly_clk_use_count > 1) {
		spin_unlock(&pll->lock);
		return;
	}

	lynx_28g_pll_ex_dly_clk_enable(pll, true);

	spin_unlock(&pll->lock);
}

static void lynx_28g_pll_put_ex_dly_clk(struct lynx_28g_pll *pll)
{
	spin_lock(&pll->lock);

	if (--pll->ex_dly_clk_use_count != 0) {
		spin_unlock(&pll->lock);
		return;
	}

	lynx_28g_pll_ex_dly_clk_enable(pll, false);

	spin_unlock(&pll->lock);
}

static void lynx_28g_lane_remap_pll(struct lynx_28g_lane *lane,
				    enum lynx_28g_lane_mode lane_mode)
{
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_28g_pll *pll;

	/* Switch to the PLL that works with this interface type */
	pll = lynx_28g_pll_get(priv, lane_mode);
	lynx_28g_lane_set_pll(lane, pll);

	/* Choose the portion of clock net to be used on this lane */
	lynx_28g_lane_set_nrate(lane, pll, lane_mode);
}

static void lynx_28g_lane_change_proto_conf(struct lynx_28g_lane *lane,
					    enum lynx_28g_lane_mode mode)
{
	const struct lynx_28g_proto_conf *conf = &lynx_28g_proto_conf[mode];

	lynx_28g_lane_rmw(lane, LNaGCR0, conf->proto_sel | conf->if_width,
			  LYNX_28G_LNaGCR0_PROTO_SEL_MSK |
			  LYNX_28G_LNaGCR0_IF_WIDTH_MSK);

	lynx_28g_lane_rmw(lane, LNaTECR0, LYNX_28G_LNaTECR0_EQ_TYPE(conf->teq_type) |
			  (conf->sgn_preq ? LYNX_28G_LNaTECR0_EQ_SGN_PREQ : 0) |
			  LYNX_28G_LNaTECR0_EQ_PREQ(conf->ratio_preq) |
			  (conf->sgn_post1q ? LYNX_28G_LNaTECR0_EQ_SGN_POST1Q : 0) |
			  LYNX_28G_LNaTECR0_EQ_POST1Q(conf->ratio_post1q) |
			  LYNX_28G_LNaTECR0_EQ_AMP_RED(conf->amp_red),
			  LYNX_28G_LNaTECR0_EQ_TYPE_MSK |
			  LYNX_28G_LNaTECR0_EQ_SGN_PREQ |
			  LYNX_28G_LNaTECR0_EQ_PREQ_MSK |
			  LYNX_28G_LNaTECR0_EQ_SGN_POST1Q |
			  LYNX_28G_LNaTECR0_EQ_POST1Q_MSK |
			  LYNX_28G_LNaTECR0_EQ_AMP_RED_MSK);

	lynx_28g_lane_rmw(lane, LNaTECR1,
			  LYNX_28G_LNaTECR1_EQ_ADPT_EQ(conf->adpt_eq),
			  LYNX_28G_LNaTECR1_EQ_ADPT_EQ_MSK);

	lynx_28g_lane_rmw(lane, LNaRGCR1,
			  LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL(conf->enter_idle_flt_sel) |
			  LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL(conf->exit_idle_flt_sel) |
			  LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL(conf->data_lost_th_sel),
			  LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL_MSK |
			  LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL_MSK |
			  LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL_MSK);

	lynx_28g_lane_rmw(lane, LNaRECR0,
			  (conf->gk2ovd_en ? LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_EN : 0) |
			  (conf->gk3ovd_en ? LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_EN : 0) |
			  (conf->gk4ovd_en ? LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_EN : 0) |
			  LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV(conf->gk2ovd) |
			  LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV(conf->gk3ovd) |
			  LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV(conf->gk4ovd),
			  LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_MSK |
			  LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_MSK |
			  LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_MSK |
			  LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_EN |
			  LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_EN |
			  LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_EN);

	lynx_28g_lane_rmw(lane, LNaRECR1,
			  LYNX_28G_LNaRECR1_EQ_OFFSET_OV(conf->eq_offset_ovd) |
			  (conf->eq_offset_ovd_en ? LYNX_28G_LNaRECR1_EQ_OFFSET_OV_EN : 0),
			  LYNX_28G_LNaRECR1_EQ_OFFSET_OV_MSK |
			  LYNX_28G_LNaRECR1_EQ_OFFSET_OV_EN);

	lynx_28g_lane_rmw(lane, LNaRECR2,
			  (conf->eq_offset_rng_dbl ? LYNX_28G_LNaRECR2_EQ_OFFSET_RNG_DBL : 0) |
			  LYNX_28G_LNaRECR2_EQ_BLW_SEL(conf->eq_blw_sel) |
			  LYNX_28G_LNaRECR2_EQ_BOOST(conf->eq_boost) |
			  LYNX_28G_LNaRECR2_SPARE_IN(conf->spare_in),
			  LYNX_28G_LNaRECR2_EQ_OFFSET_RNG_DBL |
			  LYNX_28G_LNaRECR2_EQ_BLW_SEL_MSK |
			  LYNX_28G_LNaRECR2_EQ_BOOST_MSK |
			  LYNX_28G_LNaRECR2_SPARE_IN_MSK);

	lynx_28g_lane_rmw(lane, LNaRSCCR0,
			  LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R(conf->smp_autoz_d1r) |
			  LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R(conf->smp_autoz_eg1r),
			  LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R_MSK |
			  LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R_MSK);

	lynx_28g_lane_write(lane, LNaRCCR0, conf->rccr0);
	lynx_28g_lane_write(lane, LNaTTLCR0, conf->ttlcr0);
}

static int lynx_28g_lane_disable_pcvt(struct lynx_28g_lane *lane,
				      enum lynx_28g_lane_mode mode)
{
	struct lynx_28g_priv *priv = lane->priv;
	int err;

	spin_lock(&priv->pcc_lock);

	err = lynx_pccr_write(lane, mode, 0);
	if (err)
		goto out;

	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		err = lynx_pcvt_rmw(lane, mode, CR(1), 0,
				    LYNX_28G_SGMIIaCR1_SGPCS_MSK);
		break;
	default:
		err = 0;
	}

out:
	spin_unlock(&priv->pcc_lock);

	return err;
}

static int lynx_28g_lane_enable_pcvt(struct lynx_28g_lane *lane,
				     enum lynx_28g_lane_mode mode)
{
	struct lynx_28g_priv *priv = lane->priv;
	u32 val;
	int err;

	spin_lock(&priv->pcc_lock);

	switch (mode) {
	case LANE_MODE_1000BASEX_SGMII:
	case LANE_MODE_1000BASEKX:
		err = lynx_pcvt_rmw(lane, mode, CR(1), LYNX_28G_SGMIIaCR1_SGPCS_EN,
				    LYNX_28G_SGMIIaCR1_SGPCS_MSK);
		break;
	default:
		err = 0;
	}

	val = 0;

	switch (mode) {
	case LANE_MODE_1000BASEKX:
		val |= LYNX_28G_PCC8_SGMIIa_KX;
		fallthrough;
	case LANE_MODE_1000BASEX_SGMII:
		val |= LYNX_28G_PCC8_SGMIIa_CFG;
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_10GBASEKR:
		val |= LYNX_28G_PCCC_SXGMIIn_XFI;
		fallthrough;
	case LANE_MODE_USXGMII:
		val |= LYNX_28G_PCCC_SXGMIIn_CFG;
		break;
	case LANE_MODE_25GBASER:
	case LANE_MODE_25GBASEKR:
		val |= LYNX_28G_PCCD_E25Gn_CFG;
		break;
	case LANE_MODE_40GBASER_XLAUI:
	case LANE_MODE_40GBASEKR4:
		val |= LYNX_28G_PCCE_E40Gn_CFG;
		break;
	default:
		break;
	}

	err = lynx_pccr_write(lane, mode, val);

	spin_unlock(&priv->pcc_lock);

	return err;
}

static int lynx_28g_set_lane_mode(struct phy *phy, enum lynx_28g_lane_mode lane_mode)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_priv *priv = lane->priv;
	bool powered_up = lane->powered_up;
	bool is_backplane;
	int err;

	if (lane->mode == LANE_MODE_UNKNOWN)
		return -EOPNOTSUPP;

	if (lane_mode == lane->mode)
		return 0;

	/* If the lane is powered up, put the lane into the halt state while
	 * the reconfiguration is being done.
	 */
	if (powered_up)
		lynx_28g_lane_halt(phy);

	err = lynx_28g_lane_disable_pcvt(lane, lane->mode);
	if (err)
		goto out;

	lynx_28g_lane_change_proto_conf(lane, lane_mode);
	lynx_28g_lane_remap_pll(lane, lane_mode);
	WARN_ON(lynx_28g_lane_enable_pcvt(lane, lane_mode));

	is_backplane = lane_mode == LANE_MODE_1000BASEKX ||
		       lane_mode == LANE_MODE_10GBASEKR ||
		       lane_mode == LANE_MODE_25GBASEKR ||
		       lane_mode == LANE_MODE_40GBASEKR4;
	/* Enable observation of SerDes status on all status registers */
	lynx_28g_lane_rmw(lane, LNaTCSR0,
			  is_backplane ? LYNX_28G_LNaTCSR0_SD_STAT_OBS_EN : 0,
			  LYNX_28G_LNaTCSR0_SD_STAT_OBS_EN);

	/* 1000Base-KX lanes need their PLL to generate a 312.5 MHz frequency
	 * through EX_DLY_CLK.
	 */
	if (lane_mode == LANE_MODE_1000BASEKX)
		lynx_28g_pll_get_ex_dly_clk(lynx_28g_pll_get(priv, lane_mode));
	else if (lane->mode == LANE_MODE_1000BASEKX)
		lynx_28g_pll_put_ex_dly_clk(lynx_28g_pll_get(priv, lane->mode));

	lane->mode = lane_mode;

out:
	if (powered_up)
		lynx_28g_lane_reset(phy);

	return err;
}

static int lynx_28g_set_interface(struct phy *phy, phy_interface_t submode)
{
	return lynx_28g_set_lane_mode(phy, phy_interface_to_lane_mode(submode));
}

static void lynx_28g_tune_tx_eq(struct phy *phy,
				const struct lynx_xgkr_tx_eq *tx_eq)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	lynx_28g_lane_rmw(lane, LNaTECR0,
			  LYNX_28G_LNaTECR0_EQ_PREQ(tx_eq->ratio_preq) |
			  LYNX_28G_LNaTECR0_EQ_POST1Q(tx_eq->ratio_post1q) |
			  LYNX_28G_LNaTECR0_EQ_AMP_RED(tx_eq->amp_reduction),
			  LYNX_28G_LNaTECR0_EQ_PREQ_MSK |
			  LYNX_28G_LNaTECR0_EQ_POST1Q_MSK |
			  LYNX_28G_LNaTECR0_EQ_AMP_RED_MSK);

	lynx_28g_lane_rmw(lane, LNaTECR1,
			  LYNX_28G_LNaTECR1_EQ_ADPT_EQ(tx_eq->adapt_eq),
			  LYNX_28G_LNaTECR1_EQ_ADPT_EQ_MSK);

	udelay(1);
}

static void lynx_28g_read_tx_eq(struct phy *phy, struct lynx_xgkr_tx_eq *tx_eq)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	int val;

	val = lynx_28g_lane_read(lane, LNaTECR0);
	tx_eq->ratio_preq = LYNX_28G_LNaTECR0_EQ_PREQ_X(val);
	tx_eq->ratio_post1q = LYNX_28G_LNaTECR0_EQ_POST1Q_X(val);
	tx_eq->amp_reduction = LYNX_28G_LNaTECR0_EQ_AMP_RED(val);

	val = lynx_28g_lane_read(lane, LNaTECR1);
	tx_eq->adapt_eq = LYNX_28G_LNaTECR1_EQ_ADPT_EQ_X(val);
}

static int lynx_28g_snapshot_rx_eq(struct phy *phy, int bin_sel, void *ctx,
				   void (*cb)(struct phy *phy, void *ctx))
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	bool cdr_locked;
	int err, val;

	err = read_poll_timeout(lynx_28g_cdr_lock_check, cdr_locked,
				cdr_locked, LYNX_28G_CDR_SLEEP_US,
				LYNX_28G_CDR_TIMEOUT_US, false, lane);
	if (err) {
		dev_err(&phy->dev, "CDR not locked, cannot collect RX EQ snapshots\n");
		return err;
	}

	/* wait until a previous snapshot has cleared */
	err = read_poll_timeout(lynx_28g_lane_read, val,
				!(val & LYNX_28G_LNaRECR3_EQ_SNAP_DONE),
				LYNX_28G_SNAPSHOT_SLEEP_US,
				LYNX_28G_SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR3);
	if (err)
		return err;

	/* select the binning register we would like to snapshot */
	lynx_28g_lane_rmw(lane, LNaRECR4, LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL(bin_sel),
			  LYNX_28G_LNaRECR4_EQ_BIN_DATA_SEL_MSK);

	/* start snapshot */
	lynx_28g_lane_rmw(lane, LNaRECR3, LYNX_28G_LNaRECR3_EQ_SNAP_START,
			  LYNX_28G_LNaRECR3_EQ_SNAP_START);

	/* wait for the snapshot to finish */
	err = read_poll_timeout(lynx_28g_lane_read, val,
				!!(val & LYNX_28G_LNaRECR3_EQ_SNAP_DONE),
				LYNX_28G_SNAPSHOT_SLEEP_US,
				LYNX_28G_SNAPSHOT_TIMEOUT_US,
				false, lane, LNaRECR3);
	if (err) {
		dev_err(&phy->dev,
			"Failed to snapshot RX EQ: undetected loss of CDR lock?\n");
		lynx_28g_lane_rmw(lane, LNaRECR3, 0, LYNX_28G_LNaRECR3_EQ_SNAP_START);
		return err;
	}

	cb(phy, ctx);

	/* terminate the snapshot */
	lynx_28g_lane_rmw(lane, LNaRECR3, 0, LYNX_28G_LNaRECR3_EQ_SNAP_START);

	return 0;
}

struct lynx_28g_snapshot_gains_ctx {
	u8 *gaink2;
	u8 *gaink3;
	u8 *eq_offset;
};

static void lynx_28g_snapshot_gains_cb(struct phy *phy, void *priv)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_snapshot_gains_ctx *ctx = priv;
	int recr3, recr4;

	recr3 = lynx_28g_lane_read(lane, LNaRECR3);
	recr4 = lynx_28g_lane_read(lane, LNaRECR4);

	*(ctx->gaink2) = LYNX_28G_LNaRECR3_EQ_GAINK2_HF_STAT_X(recr3);
	*(ctx->gaink3) = LYNX_28G_LNaRECR3_EQ_GAINK3_MF_STAT_X(recr3);
	*(ctx->eq_offset) = LYNX_28G_LNaRECR4_EQ_OFFSET_STAT_X(recr4);
}

static int lynx_28g_snapshot_rx_eq_gains(struct phy *phy, u8 *gaink2,
					 u8 *gaink3, u8 *eq_offset)
{
	struct lynx_28g_snapshot_gains_ctx ctx = {
		.gaink2 = gaink2,
		.gaink3 = gaink3,
		.eq_offset = eq_offset,
	};

	return lynx_28g_snapshot_rx_eq(phy, EQ_BIN_DATA_SEL_BIN_1, &ctx,
				       lynx_28g_snapshot_gains_cb);
}

static void lynx_28g_snapshot_bin_cb(struct phy *phy, void *priv)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	s16 *bin = priv;
	int val;

	/* The snapshot is a 2's complement 9 bit long value (-256 to 255) */
	val = LYNX_28G_LNaRECR4_EQ_BIN_DATA(lynx_28g_lane_read(lane, LNaRECR4));
	if (val & LYNX_28G_LNaRECR4_EQ_BIN_DATA_SGN) {
		val &= ~LYNX_28G_LNaRECR4_EQ_BIN_DATA_SGN;
		val -= 256;
	}

	*bin = (s16)val;
}

static int lynx_28g_snapshot_rx_eq_bin(struct phy *phy, enum lynx_bin_type bin_type,
				       s16 *bin)
{
	return lynx_28g_snapshot_rx_eq(phy, lynx_28g_bin_type_to_bin_sel[bin_type],
				       bin, lynx_28g_snapshot_bin_cb);
}

static const struct lynx_xgkr_algorithm_ops lynx_28g_xgkr_ops = {
	.tune_tx_eq = lynx_28g_tune_tx_eq,
	.read_tx_eq = lynx_28g_read_tx_eq,
	.snapshot_rx_eq_gains = lynx_28g_snapshot_rx_eq_gains,
	.snapshot_rx_eq_bin = lynx_28g_snapshot_rx_eq_bin,
};

static int lynx_28g_set_link_mode(struct phy *phy,
				  enum ethtool_link_mode_bit_indices submode)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_xgkr_algorithm *algorithm;
	int err;

	err = lynx_28g_set_lane_mode(phy, link_mode_to_lane_mode(submode));
	if (err)
		return err;

	algorithm = lynx_xgkr_algorithm_create(phy, &lynx_28g_xgkr_ops);
	if (!algorithm)
		return -ENOMEM;

	if (lane->algorithm)
		lynx_xgkr_algorithm_destroy(lane->algorithm);

	lane->algorithm = algorithm;

	return 0;
}

static int lynx_28g_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	switch (mode) {
	case PHY_MODE_ETHERNET:
		return lynx_28g_set_interface(phy, submode);
	case PHY_MODE_ETHERNET_LINKMODE:
		return lynx_28g_set_link_mode(phy, submode);
	default:
		return -EOPNOTSUPP;
	}
}

static int lynx_28g_validate_interface(struct phy *phy, phy_interface_t submode)
{
	enum lynx_28g_lane_mode lane_mode = phy_interface_to_lane_mode(submode);
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_priv *priv = lane->priv;

	if (!lynx_28g_supports_lane_mode(priv, lane_mode))
		return -EOPNOTSUPP;

	if (lynx_28g_lane_mode_num_lanes(lane_mode) !=
	    lynx_28g_lane_mode_num_lanes(lane->mode))
		return -EOPNOTSUPP;

	return 0;
}

static int lynx_28g_validate_link_mode(struct phy *phy,
				       enum ethtool_link_mode_bit_indices submode)
{
	enum lynx_28g_lane_mode lane_mode = link_mode_to_lane_mode(submode);
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_priv *priv = lane->priv;

	if (!lynx_28g_supports_lane_mode(priv, lane_mode))
		return -EOPNOTSUPP;

	if (lynx_28g_lane_mode_num_lanes(lane_mode) !=
	    lynx_28g_lane_mode_num_lanes(lane->mode))
		return -EOPNOTSUPP;

	return 0;
}

static int lynx_28g_validate(struct phy *phy, enum phy_mode mode, int submode,
			     union phy_configure_opts *opts __always_unused)
{
	switch (mode) {
	case PHY_MODE_ETHERNET:
		return lynx_28g_validate_interface(phy, submode);
	case PHY_MODE_ETHERNET_LINKMODE:
		return lynx_28g_validate_link_mode(phy, submode);
	default:
		return -EOPNOTSUPP;
	}
}

static int lynx_28g_init(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	/* Mark the fact that the lane was init */
	lane->init = true;

	/* SerDes lanes are powered on at boot time.  Any lane that is managed
	 * by this driver will get powered down at init time aka at dpaa2-eth
	 * probe time.
	 */
	lane->powered_up = true;
	lynx_28g_power_off(phy);

	return 0;
}

static int lynx_28g_exit(struct phy *phy)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	/* The lane returns to the state where it isn't managed by the
	 * consumer, so we must treat is as if it isn't initialized, and always
	 * powered on.
	 */
	lane->init = false;
	lane->powered_up = false;
	lynx_28g_power_on(phy);

	return 0;
}

static void lynx_28g_check_cdr_lock(struct phy *phy,
				    struct phy_status_opts_cdr *cdr)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	cdr->cdr_locked = lynx_28g_cdr_lock_check(lane);
}

static void lynx_28g_get_pcvt_count(struct phy *phy,
				    struct phy_status_opts_pcvt_count *opts)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	enum lynx_28g_lane_mode lane_mode = lane->mode;

	switch (opts->type) {
	case PHY_PCVT_ETHERNET_PCS:
		switch (lane_mode) {
		case LANE_MODE_1000BASEX_SGMII:
		case LANE_MODE_1000BASEKX:
		case LANE_MODE_10GBASER:
		case LANE_MODE_USXGMII:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_25GBASER:
		case LANE_MODE_25GBASEKR:
		case LANE_MODE_40GBASER_XLAUI:
		case LANE_MODE_40GBASEKR4:
			opts->num_pcvt = 1;
			break;
		default:
			break;
		}
		break;
	case PHY_PCVT_ETHERNET_ANLT:
		switch (lane_mode) {
		case LANE_MODE_1000BASEKX:
		case LANE_MODE_10GBASEKR:
		case LANE_MODE_25GBASEKR:
		case LANE_MODE_40GBASEKR4:
			opts->num_pcvt = 1;
			break;
		default:
			break;
		}
		break;
	default:
		break;
	}
}

static void lynx_28g_get_pcvt_addr(struct phy *phy,
				   struct phy_status_opts_pcvt *pcvt)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	enum lynx_28g_lane_mode lane_mode = lane->mode;
	u32 cr1;

	switch (pcvt->type) {
	case PHY_PCVT_ETHERNET_PCS:
		WARN_ON(lynx_pcvt_read(lane, lane_mode, CR(1), &cr1));
		pcvt->addr.mdio = LYNX_28G_MDEV_PORT_X(cr1);
		break;
	case PHY_PCVT_ETHERNET_ANLT:
		WARN_ON(lynx_anlt_read(lane, lane_mode, CR(1), &cr1));
		dev_info(&phy->dev, "ANLT%dCR1 = 0x%x\n", lane->id, cr1);
		pcvt->addr.mdio = LYNX_28G_MDEV_PORT_X(cr1);
		break;
	default:
		break;
	}
}

static int lynx_28g_get_status(struct phy *phy, enum phy_status_type type,
			       union phy_status_opts *opts)
{
	switch (type) {
	case PHY_STATUS_CDR_LOCK:
		lynx_28g_check_cdr_lock(phy, &opts->cdr);
		break;
	case PHY_STATUS_PCVT_COUNT:
		lynx_28g_get_pcvt_count(phy, &opts->pcvt_count);
		break;
	case PHY_STATUS_PCVT_ADDR:
		lynx_28g_get_pcvt_addr(phy, &opts->pcvt);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int lynx_28g_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	return lynx_xgkr_algorithm_configure(lane->algorithm, &opts->ethernet);
}

static const struct phy_ops lynx_28g_ops = {
	.init		= lynx_28g_init,
	.exit		= lynx_28g_exit,
	.power_on	= lynx_28g_power_on,
	.power_off	= lynx_28g_power_off,
	.set_mode	= lynx_28g_set_mode,
	.validate	= lynx_28g_validate,
	.get_status	= lynx_28g_get_status,
	.configure	= lynx_28g_configure,
	.owner		= THIS_MODULE,
};

static void lynx_28g_pll_read_configuration(struct lynx_28g_priv *priv)
{
	struct lynx_28g_pll *pll;
	int i, ex_dly_sel;

	for (i = 0; i < LYNX_28G_NUM_PLL; i++) {
		pll = &priv->pll[i];
		pll->priv = priv;
		pll->id = i;

		pll->rstctl = lynx_28g_pll_read(pll, PLLnRSTCTL);
		pll->cr0 = lynx_28g_pll_read(pll, PLLnCR0);
		pll->cr1 = lynx_28g_pll_read(pll, PLLnCR1);

		if (LYNX_28G_PLLnRSTCTL_DIS(pll->rstctl))
			continue;

		ex_dly_sel = LYNX_28G_PLLnCR1_EX_DLY_SEL(pll->cr1);
		if (ex_dly_sel) {
			dev_dbg(priv->dev, "PLL%cCR1[EX_DLY_SEL] found set\n",
				pll->id == 0 ? 'F' : 'S');
			pll->ex_dly_clk_use_count = 1;
		}

		switch (LYNX_28G_PLLnCR1_FRATE_SEL(pll->cr1)) {
		case LYNX_28G_PLLnCR1_FRATE_5G_10GVCO:
		case LYNX_28G_PLLnCR1_FRATE_5G_25GVCO:
			/* 5GHz clock net */
			__set_bit(LANE_MODE_1000BASEX_SGMII, pll->supported);
			if (ex_dly_sel && ex_dly_sel != EX_DLY_SEL_312_5_MHZ) {
				dev_dbg(priv->dev,
					"PLL%c has ex_dly_clk provisioned for a frequency incompatible with 1000Base-KX\n",
					pll->id == 0 ? 'F' : 'S');
			} else {
				__set_bit(LANE_MODE_1000BASEKX, pll->supported);
			}
			break;
		case LYNX_28G_PLLnCR1_FRATE_10G_20GVCO:
			/* 10.3125GHz clock net */
			__set_bit(LANE_MODE_10GBASER, pll->supported);
			__set_bit(LANE_MODE_USXGMII, pll->supported);
			__set_bit(LANE_MODE_10GBASEKR, pll->supported);
			__set_bit(LANE_MODE_40GBASER_XLAUI, pll->supported);
			__set_bit(LANE_MODE_40GBASEKR4, pll->supported);
			break;
		case LYNX_28G_PLLnCR1_FRATE_12G_25GVCO:
			/* 12.890625GHz clock net */
			__set_bit(LANE_MODE_25GBASER, pll->supported);
			__set_bit(LANE_MODE_25GBASEKR, pll->supported);
			break;
		default:
			/* 6GHz, 8GHz */
			break;
		}
	}
}

#define work_to_lynx(w) container_of((w), struct lynx_28g_priv, cdr_check.work)

static void lynx_28g_cdr_lock_check_work(struct work_struct *work)
{
	struct lynx_28g_priv *priv = work_to_lynx(work);
	struct lynx_28g_lane *lane;
	int i;

	for (i = priv->info->first_lane; i < LYNX_28G_NUM_LANE; i++) {
		lane = &priv->lane[i];

		mutex_lock(&lane->phy->mutex);

		if (!lane->init || !lane->powered_up) {
			mutex_unlock(&lane->phy->mutex);
			continue;
		}

		lynx_28g_cdr_lock_check(lane);

		mutex_unlock(&lane->phy->mutex);
	}
	queue_delayed_work(system_power_efficient_wq, &priv->cdr_check,
			   msecs_to_jiffies(1000));
}

static void lynx_28g_lane_read_configuration(struct lynx_28g_lane *lane)
{
	u32 pccc, pss, protocol;

	pss = lynx_28g_lane_read(lane, LNaPSS);
	protocol = LYNX_28G_LNaPSS_TYPE(pss);
	switch (protocol) {
	case LYNX_28G_LNaPSS_TYPE_SGMII:
		lane->mode = LANE_MODE_1000BASEX_SGMII;
		break;
	case LYNX_28G_LNaPSS_TYPE_XFI:
		lynx_pccr_read(lane, LANE_MODE_10GBASER, &pccc);
		if (pccc & LYNX_28G_PCCC_SXGMIIn_XFI)
			lane->mode = LANE_MODE_10GBASER;
		else
			lane->mode = LANE_MODE_USXGMII;
		break;
	case LYNX_28G_LNaPSS_TYPE_25G:
		lane->mode = LANE_MODE_25GBASER;
		break;
	case LYNX_28G_LNaPSS_TYPE_40G:
		lane->mode = LANE_MODE_40GBASER_XLAUI;
		break;
	default:
		lane->mode = LANE_MODE_UNKNOWN;
	}
}

static struct phy *lynx_28g_xlate(struct device *dev,
				  const struct of_phandle_args *args)
{
	struct lynx_28g_priv *priv = dev_get_drvdata(dev);
	int idx = args->args[0];

	if (WARN_ON(idx >= LYNX_28G_NUM_LANE ||
		    idx < priv->info->first_lane))
		return ERR_PTR(-EINVAL);

	return priv->lane[idx].phy;
}

static int lynx_28g_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct lynx_28g_priv *priv;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->dev = &pdev->dev;
	priv->info = of_device_get_match_data(dev);

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	lynx_28g_pll_read_configuration(priv);

	for (i = priv->info->first_lane; i < LYNX_28G_NUM_LANE; i++) {
		struct lynx_28g_lane *lane = &priv->lane[i];
		struct phy *phy;

		memset(lane, 0, sizeof(*lane));

		phy = devm_phy_create(&pdev->dev, NULL, &lynx_28g_ops);
		if (IS_ERR(phy))
			return PTR_ERR(phy);

		lane->priv = priv;
		lane->phy = phy;
		lane->id = i;
		phy_set_drvdata(phy, lane);
		lynx_28g_lane_read_configuration(lane);
	}

	dev_set_drvdata(dev, priv);

	spin_lock_init(&priv->pcc_lock);
	INIT_DELAYED_WORK(&priv->cdr_check, lynx_28g_cdr_lock_check_work);

	queue_delayed_work(system_power_efficient_wq, &priv->cdr_check,
			   msecs_to_jiffies(1000));

	dev_set_drvdata(&pdev->dev, priv);
	provider = devm_of_phy_provider_register(&pdev->dev, lynx_28g_xlate);

	return PTR_ERR_OR_ZERO(provider);
}

static void lynx_28g_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lynx_28g_priv *priv = dev_get_drvdata(dev);

	cancel_delayed_work_sync(&priv->cdr_check);
}

static const struct of_device_id lynx_28g_of_match_table[] = {
	{ .compatible = "fsl,lx2160a-serdes1", .data = &lynx_info_lx2160a_serdes1 },
	{ .compatible = "fsl,lx2160a-serdes2", .data = &lynx_info_lx2160a_serdes2 },
	{ .compatible = "fsl,lx2160a-serdes3", .data = &lynx_info_lx2160a_serdes3 },
	{ .compatible = "fsl,lx2162a-serdes1", .data = &lynx_info_lx2162a_serdes1 },
	{ .compatible = "fsl,lx2162a-serdes2", .data = &lynx_info_lx2162a_serdes2 },
	{ .compatible = "fsl,lynx-28g", .data = &lynx_info_lx2160a_serdes1 }, /* fallback */
	{ },
};
MODULE_DEVICE_TABLE(of, lynx_28g_of_match_table);

static struct platform_driver lynx_28g_driver = {
	.probe	= lynx_28g_probe,
	.remove_new = lynx_28g_remove,
	.driver	= {
		.name = "lynx-28g",
		.of_match_table = lynx_28g_of_match_table,
	},
};
module_platform_driver(lynx_28g_driver);

MODULE_AUTHOR("Ioana Ciornei <ioana.ciornei@nxp.com>");
MODULE_DESCRIPTION("Lynx 28G SerDes PHY driver for Layerscape SoCs");
MODULE_LICENSE("GPL v2");
