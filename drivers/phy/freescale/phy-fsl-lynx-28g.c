// SPDX-License-Identifier: GPL-2.0+
/* Copyright (c) 2021-2022 NXP. */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>

#define LYNX_28G_NUM_LANE			8
#define LYNX_28G_NUM_PLL			2

/* General registers per SerDes block */
#define LYNX_28G_PCC8				0x10a0
#define LYNX_28G_PCC8_SGMIInCFG(lane, x)	(((x) & GENMASK(2, 0)) << LYNX_28G_LNa_PCC_OFFSET(lane))
#define LYNX_28G_PCC8_SGMIInCFG_EN(lane)	LYNX_28G_PCC8_SGMIInCFG(lane, 1)
#define LYNX_28G_PCC8_SGMIInCFG_MSK(lane)	LYNX_28G_PCC8_SGMIInCFG(lane, GENMASK(2, 0))
#define LYNX_28G_PCC8_SGMIIn_KX(lane, x)	((((x) << 3) & BIT(3)) << LYNX_28G_LNa_PCC_OFFSET(lane))
#define LYNX_28G_PCC8_SGMIIn_KX_MSK(lane)	LYNX_28G_PCC8_SGMIIn_KX(lane, 1)
#define LYNX_28G_PCC8_MSK(lane)			LYNX_28G_PCC8_SGMIInCFG_MSK(lane) | \
						LYNX_28G_PCC8_SGMIIn_KX_MSK(lane)

#define LYNX_28G_PCCC				0x10b0
#define LYNX_28G_PCCC_SXGMIInCFG(lane, x)	(((x) & GENMASK(2, 0)) << LYNX_28G_LNa_PCC_OFFSET(lane))
#define LYNX_28G_PCCC_SXGMIInCFG_EN(lane)	LYNX_28G_PCCC_SXGMIInCFG(lane, 1)
#define LYNX_28G_PCCC_SXGMIInCFG_MSK(lane)	LYNX_28G_PCCC_SXGMIInCFG(lane, GENMASK(2, 0))
#define LYNX_28G_PCCC_SXGMIInCFG_XFI(lane, x)	((((x) << 3) & BIT(3)) << LYNX_28G_LNa_PCC_OFFSET(lane))
#define LYNX_28G_PCCC_SXGMIInCFG_XFI_X(lane, x)	((((x) >> LYNX_28G_LNa_PCC_OFFSET(lane)) & BIT(3)) >> 3)
#define LYNX_28G_PCCC_SXGMIInCFG_XFI_MSK(lane)	LYNX_28G_PCCC_SXGMIInCFG_XFI(lane, 1)
#define LYNX_28G_PCCC_MSK(lane)			LYNX_28G_PCCC_SXGMIInCFG_MSK(lane) | \
						LYNX_28G_PCCC_SXGMIInCFG_XFI_MSK(lane)

#define LYNX_28G_PCCD				0x10b4
#define LYNX_28G_PCCD_E25GnCFG(lane, x)		(((x) & GENMASK(2, 0)) << LYNX_28G_LNa_PCCD_OFFSET(lane))
#define LYNX_28G_PCCD_E25GnCFG_EN(lane)		LYNX_28G_PCCD_E25GnCFG(lane, 1)
#define LYNX_28G_PCCD_E25GnCFG_MSK(lane)	LYNX_28G_PCCD_E25GnCFG(lane, GENMASK(2, 0))
#define LYNX_28G_PCCD_MSK(lane)			LYNX_28G_PCCD_E25GnCFG_MSK(lane)

#define LYNX_28G_LNa_PCC_OFFSET(lane)		(4 * (LYNX_28G_NUM_LANE - (lane->id) - 1))
#define LYNX_28G_LNa_PCCD_OFFSET(lane)		(4 * (lane->id))

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

#define LYNX_28G_LNaTECR0(lane)			(0x800 + (lane) * 0x100 + 0x30)

#define LYNX_28G_LNaTECR0_EQ_TYPE(x)		(((x) << 28) & GENMASK(30, 28))
#define LYNX_28G_LNaTECR0_EQ_TYPE_X(x)		(((x) & GENMASK(30, 28)) >> 28)
#define LYNX_28G_LNaTECR0_EQ_SGN_PREQ		BIT(23)
#define LYNX_28G_LNaTECR0_EQ_PREQ(x)		(((x) << 16) & GENMASK(19, 16))
#define LYNX_28G_LNaTECR0_EQ_PREQ_X(x)		(((x) & GENMASK(19, 16)) >> 16)
#define LYNX_28G_LNaTECR0_EQ_SGN_POST1Q		BIT(15)
#define LYNX_28G_LNaTECR0_EQ_POST1Q(x)		(((x) << 8) & GENMASK(12, 8))
#define LYNX_28G_LNaTECR0_EQ_POST1Q_X(x)		(((x) & GENMASK(12, 8)) >> 8)
#define LYNX_28G_LNaTECR0_EQ_AMP_RED(x)		((x) & GENMASK(5, 0))

#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ_DRVR_DIS	BIT(31)
#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ(x)		(((x) << 24) & GENMASK(29, 24))
#define LYNX_28G_LNaTECR1_EQ_ADPT_EQ_X(x)	(((x) & GENMASK(29, 24)) >> 24)

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

#define LYNX_28G_LNaRGCR1(lane)			(0x800 + (lane) * 0x100 + 0x48)

#define LYNX_28G_LNaRGCR1_RX_ORD_ELECIDLE	BIT(31)
#define LYNX_28G_LNaRGCR1_DATA_LOST_FLT		BIT(30)
#define LYNX_28G_LNaRGCR1_DATA_LOST		BIT(29)
#define LYNX_28G_LNaRGCR1_IDLE_CONFIG		BIT(28)
#define LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL(x)	(((x) << 24) & GENMASK(26, 24))
#define LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL_X(x) (((x) & GENMASK(26, 24)) >> 24)
#define LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL(x)	(((x) << 20) & GENMASK(22, 20))
#define LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL_X(x) (((x) & GENMASK(22, 20)) >> 20)
#define LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL(x)	(((x) << 16) & GENMASK(18, 16))
#define LYNX_28G_LNaRGCR1_DATA_LOST_TH_SEL_X(x)	(((x) & GENMASK(18, 16)) >> 16)
#define LYNX_28G_LNaRGCR1_EXT_REC_CLK_SEL(x)	(((x) << 8) & GENMASK(10, 8))
#define LYNX_28G_LNaRGCR1_EXT_REC_CLK_SEL_X(x)	(((x) & GENMASK(10, 8)) >> 8)
#define LYNX_28G_LNaRGCR1_WAKE_TX_DIS		BIT(5)
#define LYNX_28G_LNaRGCR1_PHY_RDY		BIT(4)
#define LYNX_28G_LNaRGCR1_CHANGE_RX_CLK		BIT(3)
#define LYNX_28G_LNaRGCR1_PWR_MGT(x)		((x) & GENMASK(2, 0))

#define LYNX_28G_LNaRECR0(lane)			(0x800 + (lane) * 0x100 + 0x50)

#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_EN	BIT(31)
#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV(x)	(((x) << 24) & GENMASK(28, 24))
#define LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_X(x)	(((x) & GENMASK(28, 24)) >> 24)
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_EN	BIT(23)
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV(x)	(((x) << 16) & GENMASK(20, 16))
#define LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_X(x)	(((x) & GENMASK(20, 16)) >> 16)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_EN	BIT(7)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_DIS	BIT(6)
#define LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV(x)	((x) & GENMASK(4, 0))

#define LYNX_28G_LNaRECR1(lane)			(0x800 + (lane) * 0x100 + 0x54)

#define LYNX_28G_LNaRECR1_EQ_BLW_OV_EN		BIT(31)
#define LYNX_28G_LNaRECR1_EQ_BLW_OV(x)		(((x) << 24) & GENMASK(28, 24))
#define LYNX_28G_LNaRECR1_EQ_BLW_OV_X(x)	(((x) & GENMASK(28, 24)) >> 24)
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV_EN	BIT(23)
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV(x)	(((x) << 16) & GENMASK(21, 16))
#define LYNX_28G_LNaRECR1_EQ_OFFSET_OV_X(x)	(((x) & GENMASK(21, 16)) >> 16)

#define LYNX_28G_LNaRECR2(lane)			(0x800 + (lane) * 0x100 + 0x58)

#define LYNX_28G_LNaRECR2_EQ_OFFSET_RNG_DBL	BIT(31)
#define LYNX_28G_LNaRECR2_EQ_BOOST(x)		(((x) << 28) & GENMASK(29, 28))
#define LYNX_28G_LNaRECR2_EQ_BOOST_X(x)		(((x) & GENMASK(29, 28)) >> 28)
#define LYNX_28G_LNaRECR2_EQ_BLW_SEL(x)		(((x) << 24) & GENMASK(25, 24))
#define LYNX_28G_LNaRECR2_EQ_BLW_SEL_X(x)	(((x) & GENMASK(25, 24)) >> 24)
#define LYNX_28G_LNaRECR2_EQ_ZERO(x)		(((x) << 16) & GENMASK(17, 16))
#define LYNX_28G_LNaRECR2_EQ_ZERO_X(x)		(((x) & GENMASK(17, 16)) >> 16)
#define LYNX_28G_LNaRECR2_EQ_IND(x)		(((x) << 12) & GENMASK(13, 12))
#define LYNX_28G_LNaRECR2_EQ_IND_X(x)		(((x) & GENMASK(13, 12)) >> 12)
#define LYNX_28G_LNaRECR2_EQ_BIN_DATA_AVG_TC(x)	(((x) << 4) & GENMASK(5, 4))
#define LYNX_28G_LNaRECR2_EQ_BIN_DATA_AVG_TC_X(x) (((x) & GENMASK(5, 4)) >> 4)
#define LYNX_28G_LNaRECR2_SPARE_IN(x)		((x) & GENMASK(1, 0))

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
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL(x)	(((x) << 16) & GENMASK(19, 16))
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_CTRL_X(x)	(((x) & GENMASK(19, 16)) >> 16)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R(x)	(((x) << 12) & GENMASK(13, 12))
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R_X(x)	(((x) & GENMASK(13, 12)) >> 12)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F(x)	(((x) << 8) & GENMASK(9, 8))
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F(x)	(((x) << 8) & GENMASK(9, 8))
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1F_X(x)	(((x) & GENMASK(9, 8)) >> 8)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R(x)	(((x) << 4) & GENMASK(5, 4))
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R_X(x)	(((x) & GENMASK(5, 4)) >> 4)
#define LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1F(x)	((x) & GENMASK(1, 0))

#define LYNX_28G_LNaTTLCR0(lane)		(0x800 + (lane) * 0x100 + 0x80)

#define LYNX_28G_LNaTTLCR0_TTL_FLT_SEL(x)	(((x) << 24) & GENMASK(29, 24))
#define LYNX_28G_LNaTTLCR0_TTL_FLT_SEL_X(x)	(((x) & GENMASK(29, 24)) >> 24)
#define LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP	BIT(22)
#define LYNX_28G_LNaTTLCR0_STALL_DET_DIS	BIT(21)
#define LYNX_28G_LNaTTLCR0_INACT_MON_DIS	BIT(20)
#define LYNX_28G_LNaTTLCR0_CDR_OV(x)		(((x) << 16) & GENMASK(18, 16))
#define LYNX_28G_LNaTTLCR0_CDR_OV_X(x)		(((x) & GENMASK(18, 16)) >> 16)
#define LYNX_28G_LNaTTLCR0_DATA_IN_SSC		BIT(15)
#define LYNX_28G_LNaTTLCR0_CDR_MIN_SMP_ON(x)	((x) & GENMASK(1, 0))

#define LYNX_28G_LNaPSS(lane)			(0x1000 + (lane) * 0x4)
#define LYNX_28G_LNaPSS_TYPE(pss)		(((pss) & GENMASK(30, 24)) >> 24)
#define LYNX_28G_LNaPSS_TYPE_SGMII		0x4
#define LYNX_28G_LNaPSS_TYPE_XFI		0x28
#define LYNX_28G_LNaPSS_TYPE_25G		0x68

#define LYNX_28G_SGMIIaCR1(lane)		(0x1804 + (lane) * 0x10)
#define LYNX_28G_SGMIIaCR1_SGPCS_EN		BIT(11)
#define LYNX_28G_SGMIIaCR1_SGPCS_MSK		BIT(11)

enum lynx_28g_eq_type {
	EQ_TYPE_NO_EQ = 0,
	EQ_TYPE_2TAP = 1,
	EQ_TYPE_3TAP = 2,
};

enum lynx_28g_lane_mode {
	LANE_MODE_UNKNOWN,
	LANE_MODE_1000BASEX_SGMII,
	LANE_MODE_10GBASER,
	LANE_MODE_USXGMII,
	LANE_MODE_25GBASER,
	LANE_MODE_MAX,
};

struct lynx_28g_priv;

struct lynx_28g_pll {
	struct lynx_28g_priv *priv;
	u32 rstctl, cr0, cr1;
	int id;
	DECLARE_BITMAP(supported, LANE_MODE_MAX);
};

struct lynx_28g_lane {
	struct lynx_28g_priv *priv;
	struct phy *phy;
	bool powered_up;
	bool init;
	unsigned int id;
	enum lynx_28g_lane_mode mode;
};

struct lynx_28g_priv {
	void __iomem *base;
	struct device *dev;
	/* Serialize concurrent access to registers shared between lanes,
	 * like PCCn
	 */
	spinlock_t pcc_lock;
	struct lynx_28g_pll pll[LYNX_28G_NUM_PLL];
	struct lynx_28g_lane lane[LYNX_28G_NUM_LANE];

	struct delayed_work cdr_check;
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
#define lynx_28g_lane_rmw(lane, reg, val, mask)	\
	lynx_28g_rmw((lane)->priv, LYNX_28G_##reg(lane->id), val, mask)
#define lynx_28g_lane_read(lane, reg)			\
	ioread32((lane)->priv->base + LYNX_28G_##reg((lane)->id))
#define lynx_28g_lane_write(lane, reg, val)		\
	iowrite32(val, (lane)->priv->base + LYNX_28G_##reg((lane)->id))
#define lynx_28g_pll_read(pll, reg)			\
	ioread32((pll)->priv->base + LYNX_28G_##reg((pll)->id))

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

static void lynx_28g_cleanup_lane(struct lynx_28g_lane *lane)
{
	struct lynx_28g_priv *priv = lane->priv;

	switch (lane->mode) {
	case LANE_MODE_10GBASER:
	case LANE_MODE_USXGMII:
		/* Cleanup the protocol configuration registers */
		lynx_28g_rmw(priv, LYNX_28G_PCCC, 0,
			     LYNX_28G_PCCC_MSK(lane));
		break;
	case LANE_MODE_1000BASEX_SGMII:
		/* Cleanup the protocol configuration registers */
		lynx_28g_rmw(priv, LYNX_28G_PCC8, 0,
			     LYNX_28G_PCC8_MSK(lane));

		/* Disable the SGMII PCS */
		lynx_28g_lane_rmw(lane, SGMIIaCR1, 0,
				  LYNX_28G_SGMIIaCR1_SGPCS_MSK);

		break;
	case LANE_MODE_25GBASER:
		/* Cleanup the protocol configuration registers */
		lynx_28g_rmw(priv, LYNX_28G_PCCD, 0,
			     LYNX_28G_PCCD_MSK(lane));
		break;
	default:
		break;
	}
}

static bool lynx_28g_cdr_lock_check(struct lynx_28g_lane *lane)
{
	u32 rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);

	if (rrstctl & LYNX_28G_LNaRRSTCTL_CDR_LOCK)
		return true;

	dev_dbg(&lane->phy->dev, "CDR unlocked, resetting lane receiver...\n");

	lynx_28g_lane_rmw(lane, LNaRRSTCTL,
			  LYNX_28G_LNaRRSTCTL_RST_REQ,
			  LYNX_28G_LNaRRSTCTL_RST_REQ);
	do {
		rrstctl = lynx_28g_lane_read(lane, LNaRRSTCTL);
	} while (!(rrstctl & LYNX_28G_LNaRRSTCTL_RST_DONE));

	return !!(rrstctl & LYNX_28G_LNaRRSTCTL_CDR_LOCK);
}

static void lynx_28g_lane_set_sgmii(struct lynx_28g_lane *lane)
{
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_28g_pll *pll;

	lynx_28g_cleanup_lane(lane);

	/* Setup the lane to run in SGMII */
	lynx_28g_rmw(priv, LYNX_28G_PCC8, LYNX_28G_PCC8_SGMIInCFG_EN(lane),
		     LYNX_28G_PCC8_MSK(lane));

	/* Setup the protocol select and SerDes parallel interface width */
	lynx_28g_lane_rmw(lane, LNaGCR0, LYNX_28G_LNaGCR0_PROTO_SEL_SGMII,
			  LYNX_28G_LNaGCR0_PROTO_SEL_MSK);
	lynx_28g_lane_rmw(lane, LNaGCR0, LYNX_28G_LNaGCR0_IF_WIDTH_10_BIT,
			  LYNX_28G_LNaGCR0_IF_WIDTH_MSK);

	/* Switch to the PLL that works with this interface type */
	pll = lynx_28g_pll_get(priv, LANE_MODE_1000BASEX_SGMII);
	lynx_28g_lane_set_pll(lane, pll);

	/* Choose the portion of clock net to be used on this lane */
	lynx_28g_lane_set_nrate(lane, pll, LANE_MODE_1000BASEX_SGMII);

	/* Enable the SGMII PCS */
	lynx_28g_lane_rmw(lane, SGMIIaCR1, LYNX_28G_SGMIIaCR1_SGPCS_EN,
			  LYNX_28G_SGMIIaCR1_SGPCS_MSK);

	/* Configure the appropriate equalization parameters for the protocol */
	lynx_28g_lane_write(lane, LNaTECR0,
			    LYNX_28G_LNaTECR0_EQ_SGN_PREQ |
			    LYNX_28G_LNaTECR0_EQ_SGN_POST1Q |
			    LYNX_28G_LNaTECR0_EQ_AMP_RED(6));
	lynx_28g_lane_write(lane, LNaRGCR1,
			    LYNX_28G_LNaRGCR1_ENTER_IDLE_FLT_SEL(4) |
			    LYNX_28G_LNaRGCR1_EXIT_IDLE_FLT_SEL(3) |
			    LYNX_28G_LNaRGCR1_DATA_LOST_FLT);
	lynx_28g_lane_write(lane, LNaRECR0,
			    LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV_EN |
			    LYNX_28G_LNaRECR0_EQ_GAINK2_HF_OV(0x1f) |
			    LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV_EN |
			    LYNX_28G_LNaRECR0_EQ_GAINK3_MF_OV(0));
	lynx_28g_lane_write(lane, LNaRECR1, LYNX_28G_LNaRECR1_EQ_OFFSET_OV(31));
	lynx_28g_lane_write(lane, LNaRECR2, 0);
	lynx_28g_lane_write(lane, LNaRSCCR0, 0);
}

static void lynx_28g_lane_set_10g(struct lynx_28g_lane *lane,
				  enum lynx_28g_lane_mode lane_mode)
{
	bool is_xfi = lane_mode != LANE_MODE_USXGMII;
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_28g_pll *pll;

	lynx_28g_cleanup_lane(lane);

	/* Enable the SXGMII lane */
	lynx_28g_rmw(priv, LYNX_28G_PCCC,
		     LYNX_28G_PCCC_SXGMIInCFG_EN(lane) |
		     LYNX_28G_PCCC_SXGMIInCFG_XFI(lane, is_xfi),
		     LYNX_28G_PCCC_MSK(lane));

	/* Setup the protocol select and SerDes parallel interface width */
	lynx_28g_lane_rmw(lane, LNaGCR0, LYNX_28G_LNaGCR0_PROTO_SEL_XFI,
			  LYNX_28G_LNaGCR0_PROTO_SEL_MSK);
	lynx_28g_lane_rmw(lane, LNaGCR0, LYNX_28G_LNaGCR0_IF_WIDTH_20_BIT,
			  LYNX_28G_LNaGCR0_IF_WIDTH_MSK);

	/* Switch to the PLL that works with this interface type */
	pll = lynx_28g_pll_get(priv, lane_mode);
	lynx_28g_lane_set_pll(lane, pll);

	/* Choose the portion of clock net to be used on this lane */
	lynx_28g_lane_set_nrate(lane, pll, lane_mode);

	/* Configure the appropriate equalization parameters for the protocol */
	lynx_28g_lane_write(lane, LNaTECR0,
			    LYNX_28G_LNaTECR0_EQ_TYPE(EQ_TYPE_2TAP) |
			    LYNX_28G_LNaTECR0_EQ_SGN_PREQ |
			    LYNX_28G_LNaTECR0_EQ_PREQ(0) |
			    LYNX_28G_LNaTECR0_EQ_SGN_POST1Q |
			    LYNX_28G_LNaTECR0_EQ_POST1Q(3) |
			    LYNX_28G_LNaTECR0_EQ_AMP_RED(7));
	lynx_28g_lane_write(lane, LNaRGCR1, LYNX_28G_LNaRGCR1_IDLE_CONFIG);
	lynx_28g_lane_write(lane, LNaRECR0, 0);
	lynx_28g_lane_write(lane, LNaRECR1, LYNX_28G_LNaRECR1_EQ_OFFSET_OV(31));
	lynx_28g_lane_write(lane, LNaRECR2,
			    LYNX_28G_LNaRECR2_EQ_OFFSET_RNG_DBL |
			    LYNX_28G_LNaRECR2_EQ_BLW_SEL(1) |
			    LYNX_28G_LNaRECR2_EQ_BIN_DATA_AVG_TC(2));
	lynx_28g_lane_write(lane, LNaRSCCR0,
			    LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R(2));
	lynx_28g_lane_write(lane, LNaRCCR0, LYNX_28G_LNaRCCR0_CAL_EN);
	lynx_28g_lane_write(lane, LNaTTLCR0,
			    LYNX_28G_LNaTTLCR0_TTL_SLO_PM_BYP |
			    LYNX_28G_LNaTTLCR0_DATA_IN_SSC);
}

static void lynx_28g_lane_set_25gbaser(struct lynx_28g_lane *lane)
{
	struct lynx_28g_priv *priv = lane->priv;
	struct lynx_28g_pll *pll;

	lynx_28g_cleanup_lane(lane);

	/* Enable the E25G lane */
	lynx_28g_rmw(priv, LYNX_28G_PCCD, LYNX_28G_PCCD_E25GnCFG_EN(lane),
		     LYNX_28G_PCCD_MSK(lane));

	/* Setup the protocol select and SerDes parallel interface width */
	lynx_28g_lane_rmw(lane, LNaGCR0, LYNX_28G_LNaGCR0_PROTO_SEL_25G,
			  LYNX_28G_LNaGCR0_PROTO_SEL_MSK);
	lynx_28g_lane_rmw(lane, LNaGCR0, LYNX_28G_LNaGCR0_IF_WIDTH_40_BIT,
			  LYNX_28G_LNaGCR0_IF_WIDTH_MSK);

	/* Switch to the PLL that works with this interface type */
	pll = lynx_28g_pll_get(priv, LANE_MODE_25GBASER);
	lynx_28g_lane_set_pll(lane, pll);

	/* Choose the portion of clock net to be used on this lane */
	lynx_28g_lane_set_nrate(lane, pll, LANE_MODE_25GBASER);

	/* Configure the appropriate equalization parameters for 25GBASE-R */
	lynx_28g_lane_write(lane, LNaTECR0,
			    LYNX_28G_LNaTECR0_EQ_TYPE(EQ_TYPE_3TAP) |
			    LYNX_28G_LNaTECR0_EQ_SGN_PREQ |
			    LYNX_28G_LNaTECR0_EQ_PREQ(2) |
			    LYNX_28G_LNaTECR0_EQ_SGN_POST1Q |
			    LYNX_28G_LNaTECR0_EQ_POST1Q(7));
	lynx_28g_lane_write(lane, LNaRGCR1, LYNX_28G_LNaRGCR1_IDLE_CONFIG);
	lynx_28g_lane_write(lane, LNaRECR0,
			    LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV_EN |
			    LYNX_28G_LNaRECR0_EQ_GAINK4_LF_OV(5));
	lynx_28g_lane_write(lane, LNaRECR1, LYNX_28G_LNaRECR1_EQ_OFFSET_OV(31));
	lynx_28g_lane_write(lane, LNaRECR2,
			    LYNX_28G_LNaRECR2_EQ_OFFSET_RNG_DBL |
			    LYNX_28G_LNaRECR2_EQ_BOOST(2) |
			    LYNX_28G_LNaRECR2_EQ_BLW_SEL(1) |
			    LYNX_28G_LNaRECR2_EQ_BIN_DATA_AVG_TC(2) |
			    LYNX_28G_LNaRECR2_SPARE_IN(3));
	lynx_28g_lane_write(lane, LNaRSCCR0,
			    LYNX_28G_LNaRSCCR0_SMP_AUTOZ_D1R(2) |
			    LYNX_28G_LNaRSCCR0_SMP_AUTOZ_EG1R(2));
	lynx_28g_lane_write(lane, LNaRCCR0,
			    LYNX_28G_LNaRCCR0_CAL_EN |
			    LYNX_28G_LNaRCCR0_CAL_DC3_DIS |
			    LYNX_28G_LNaRCCR0_CAL_DC2_DIS |
			    LYNX_28G_LNaRCCR0_CAL_DC1_DIS |
			    LYNX_28G_LNaRCCR0_CAL_DC0_DIS);
	lynx_28g_lane_write(lane, LNaTTLCR0,
			    LYNX_28G_LNaTTLCR0_DATA_IN_SSC |
			    LYNX_28G_LNaTTLCR0_CDR_MIN_SMP_ON(1));
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

static int lynx_28g_set_mode(struct phy *phy, enum phy_mode mode, int submode)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_priv *priv = lane->priv;
	int powered_up = lane->powered_up;
	enum lynx_28g_lane_mode lane_mode;
	int err = 0;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	if (lane->mode == LANE_MODE_UNKNOWN)
		return -EOPNOTSUPP;

	lane_mode = phy_interface_to_lane_mode(submode);

	if (!lynx_28g_supports_lane_mode(priv, lane_mode))
		return -EOPNOTSUPP;

	/* If the lane is powered up, put the lane into the halt state while
	 * the reconfiguration is being done.
	 */
	if (powered_up)
		lynx_28g_lane_halt(phy);

	spin_lock(&priv->pcc_lock);

	switch (lane_mode) {
	case LANE_MODE_1000BASEX_SGMII:
		lynx_28g_lane_set_sgmii(lane);
		break;
	case LANE_MODE_10GBASER:
	case LANE_MODE_USXGMII:
		lynx_28g_lane_set_10g(lane, lane_mode);
		break;
	case LANE_MODE_25GBASER:
		lynx_28g_lane_set_25gbaser(lane);
		break;
	default:
		if (lane_mode != lane->mode)
			err = -EOPNOTSUPP;
		goto out;
	}

	lane->mode = lane_mode;

out:
	spin_unlock(&priv->pcc_lock);

	/* Reset the lane if necessary */
	if (powered_up)
		lynx_28g_lane_reset(phy);

	return err;
}

static int lynx_28g_validate(struct phy *phy, enum phy_mode mode, int submode,
			     union phy_configure_opts *opts __always_unused)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);
	struct lynx_28g_priv *priv = lane->priv;

	if (mode != PHY_MODE_ETHERNET)
		return -EOPNOTSUPP;

	if (!lynx_28g_supports_lane_mode(priv,
					 phy_interface_to_lane_mode(submode)))
		return -EOPNOTSUPP;

	return 0;
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

static void lynx_28g_check_cdr_lock(struct phy *phy,
				    struct phy_status_opts_cdr *cdr)
{
	struct lynx_28g_lane *lane = phy_get_drvdata(phy);

	cdr->cdr_locked = lynx_28g_cdr_lock_check(lane);
}

static int lynx_28g_get_status(struct phy *phy, enum phy_status_type type,
			       union phy_status_opts *opts)
{
	switch (type) {
	case PHY_STATUS_CDR_LOCK:
		lynx_28g_check_cdr_lock(phy, &opts->cdr);
		break;
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static const struct phy_ops lynx_28g_ops = {
	.init		= lynx_28g_init,
	.power_on	= lynx_28g_power_on,
	.power_off	= lynx_28g_power_off,
	.set_mode	= lynx_28g_set_mode,
	.validate	= lynx_28g_validate,
	.get_status	= lynx_28g_get_status,
	.owner		= THIS_MODULE,
};

static void lynx_28g_pll_read_configuration(struct lynx_28g_priv *priv)
{
	struct lynx_28g_pll *pll;
	int i;

	for (i = 0; i < LYNX_28G_NUM_PLL; i++) {
		pll = &priv->pll[i];
		pll->priv = priv;
		pll->id = i;

		pll->rstctl = lynx_28g_pll_read(pll, PLLnRSTCTL);
		pll->cr0 = lynx_28g_pll_read(pll, PLLnCR0);
		pll->cr1 = lynx_28g_pll_read(pll, PLLnCR1);

		if (LYNX_28G_PLLnRSTCTL_DIS(pll->rstctl))
			continue;

		switch (LYNX_28G_PLLnCR1_FRATE_SEL(pll->cr1)) {
		case LYNX_28G_PLLnCR1_FRATE_5G_10GVCO:
		case LYNX_28G_PLLnCR1_FRATE_5G_25GVCO:
			/* 5GHz clock net */
			__set_bit(LANE_MODE_1000BASEX_SGMII, pll->supported);
			break;
		case LYNX_28G_PLLnCR1_FRATE_10G_20GVCO:
			/* 10.3125GHz clock net */
			__set_bit(LANE_MODE_10GBASER, pll->supported);
			__set_bit(LANE_MODE_USXGMII, pll->supported);
			break;
		case LYNX_28G_PLLnCR1_FRATE_12G_25GVCO:
			/* 12.890625GHz clock net */
			__set_bit(LANE_MODE_25GBASER, pll->supported);
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

	for (i = 0; i < LYNX_28G_NUM_LANE; i++) {
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
	struct lynx_28g_priv *priv = lane->priv;
	u32 pccc, pss, protocol;

	pss = lynx_28g_lane_read(lane, LNaPSS);
	protocol = LYNX_28G_LNaPSS_TYPE(pss);
	switch (protocol) {
	case LYNX_28G_LNaPSS_TYPE_SGMII:
		lane->mode = LANE_MODE_1000BASEX_SGMII;
		break;
	case LYNX_28G_LNaPSS_TYPE_XFI:
		pccc = lynx_28g_read(priv, LYNX_28G_PCCC);
		if (LYNX_28G_PCCC_SXGMIInCFG_XFI_X(lane, pccc))
			lane->mode = LANE_MODE_10GBASER;
		else
			lane->mode = LANE_MODE_USXGMII;
		break;
	case LYNX_28G_LNaPSS_TYPE_25G:
		lane->mode = LANE_MODE_25GBASER;
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

	if (WARN_ON(idx >= LYNX_28G_NUM_LANE))
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

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	lynx_28g_pll_read_configuration(priv);

	for (i = 0; i < LYNX_28G_NUM_LANE; i++) {
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
	{ .compatible = "fsl,lynx-28g" },
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
