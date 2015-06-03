/*
 * AD9739A SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DEF_AD9739A_H
#define _DEF_AD9739A_H

/* AD9379A Registers */

#define REG_MODE				0x00
#define REG_POWER_DOWN			0x01
#define REG_CNT_CLK_DIS			0x02
#define REG_IRQ_EN				0x03
#define REG_IRQ_REQ				0x04
#define REG_FSC_1				0x06
#define REG_FSC_2				0x07
#define REG_DEC_CNT				0x08
#define REG_LVDS_STAT1			0x0C
#define REG_LVDS_REC_CNT1		0x10
#define REG_LVDS_REC_CNT2		0x11
#define REG_LVDS_REC_CNT3		0x12
#define REG_LVDS_REC_CNT4		0x13
#define REG_LVDS_REC_CNT5		0x14
#define REG_LVDS_REC_STAT1		0x19
#define REG_LVDS_REC_STAT2		0x1A
#define REG_LVDS_REC_STAT3		0x1B
#define REG_LVDS_REC_STAT4		0x1C
#define REG_LVDS_REC_STAT9		0x21
#define REG_CROSS_CNT1			0x22
#define REG_CROSS_CNT2			0x23
#define REG_PHS_DET				0x24
#define REG_MU_DUTY				0x25
#define REG_MU_CNT1				0x26
#define REG_MU_CNT2				0x27
#define REG_MU_CNT3				0x28
#define REG_MU_CNT4				0x29
#define REG_MU_STAT1			0x2A
#define REG_PART_ID				0x35

/* REG_MODE */
#define MODE_SDIO_DIR			((1 << 7) | (1 << 0))
#define MODE_LSB				((1 << 6) | (1 << 1))
#define MODE_RESET				((1 << 5) | (1 << 2))

/* REG_POWER_DOWN */
#define POWER_DOWN_LVDS_DRVR_PD	(1 << 5)
#define POWER_DOWN_LVDS_RCVR_PD	(1 << 4)
#define POWER_DOWN_CLK_RCVR_PD	(1 << 1)
#define POWER_DOWN_DAC_BIAS_PD	(1 << 0)

/* REG_CNT_CLK_DIS */
#define CNT_CLK_DIS_CLKGEN_PD	(1 << 3)
#define CNT_CLK_DIS_REC_CNT_CLK	(1 << 1)
#define CNT_CLK_DIS_MU_CNT_CLK	(1 << 0)

/* REG_IRQ_EN */
#define IRQ_EN_MU_LST_EN		(1 << 3)
#define IRQ_EN_MU_LCK_EN		(1 << 2)
#define IRQ_EN_RCV_LST_EN		(1 << 1)
#define IRQ_EN_RCV_LCK_EN		(1 << 0)

/* REG_IRQ_REQ */
#define IRQ_REQ_MU_LST_IRQ		(1 << 3)
#define IRQ_REQ_MU_LCK_IRQ		(1 << 2)
#define IRQ_REQ_RCV_LST_IRQ		(1 << 1)
#define IRQ_REQ_RCV_LCK_IRQ		(1 << 0)

/* REG_FSC_1 */
#define FSC_1_FSC_1(x) 			(((x) & 0xFF) << 0)

/* REG_FSC_2 */
#define FSC_2_FSC_2(x)			(((x) & 0x3) << 0)
#define FSC_2_SLEEP				(1 << 7)

/* REG_DEC_CNT */
#define DEC_CNT_DAC_DEC(x)		(((x) & 0x3) << 0)
#define NORMAL_BASEBAND			0	// DEC_CNT_DAC_DEC(x) option.
#define MIX_MODE				2	// DEC_CNT_DAC_DEC(x) option.

/* REG_LVDS_STAT1 */
#define LVDS_STAT1_DCI_PRE_PH0		(1 << 2)
#define LVDS_STAT1_DCI_PST_PH0		(1 << 0)

/* REG_LVDS_REC_CNT1 */
#define LVDS_REC_CNT1_RCVR_FLG_RST	(1 << 2)
#define LVDS_REC_CNT1_RCVR_LOOP_ON	(1 << 1)
#define LVDS_REC_CNT1_RCVR_CNT_ENA	(1 << 0)

/* REG_LVDS_REC_CNT2 */
#define LVDS_REC_CNT2_SMP_DEL(x)	(((x) & 0x3) << 6)

/* REG_LVDS_REC_CNT3 */
#define LVDS_REC_CNT3_SMP_DEL(x)	(((x) & 0xFF) << 0)

/* REG_LVDS_REC_CNT4 */
#define LVDS_REC_CNT4_DCI_DEL(x)		(((x) & 0xF) << 4)
#define LVDS_REC_CNT4_FINE_DEL_SKEW(x)	(((x) & 0xF) << 0)

/* REG_LVDS_REC_CNT5 */
#define LVDS_REC_CNT5_DCI_DEL(x)	(((x) & 0x3F) << 0)

/* REG_LVDS_REC_STAT1 */
#define LVDS_REC_STAT1_SMP_DEL(x)	(((x) & 0x3) << 6)

/* REG_LVDS_REC_STAT2 */
#define LVDS_REC_STAT2_SMP_DEL(x)	(((x) & 0xFF) << 0)

/* REG_LVDS_REC_STAT3 */
#define LVDS_REC_STAT3_DCI_DEL(x)	(((x) & 0x3) << 6)

/* REG_LVDS_REC_STAT4 */
#define LVDS_REC_STAT4_DCI_DEL(x)	(((x) & 0xFF) << 0)

/* REG_LVDS_REC_STAT9 */
#define LVDS_REC_STAT9_RCVR_TRK_ON	(1 << 3)
#define LVDS_REC_STAT9_RCVR_FE_ON	(1 << 2)
#define LVDS_REC_STAT9_RCVR_LST		(1 << 1)
#define LVDS_REC_STAT9_RCVR_LCK		(1 << 0)

/* REG_CROSS_CNT1 */
#define CROSS_CNT1_DIR_P			(1 << 4)
#define CROSS_CNT1_CLKP_OFFSET(x)	(((x) & 0xF) << 0)

/* REG_CROSS_CNT2 */
#define CROSS_CNT2_DIR_N			(1 << 4)
#define CROSS_CNT2_CLKN_OFFSET(x)	(((x) & 0xF) << 0)

/* REG_PHS_DET */
#define PHS_DET_CMP_BST				(1 << 5)
#define PHS_DET_PHS_DET_AUTO_EN		(1 << 4)

/* REG_MU_DUTY */
#define MU_DUTY_MU_DUTY_AUTO_EN		(1 << 7)

/* REG_MU_CNT1 */
#define MU_CNT1_SLOPE				(1 << 6)
#define MU_CNT1_MODE(x)				(((x) & 0x3) << 4)
#define MU_CNT1_READ				(1 << 3)
#define MU_CNT1_GAIN(x)				(((x) & 0x3) << 1)
#define MU_CNT1_ENABLE				(1 << 0)

/* REG_MU_CNT2 */
#define MU_CNT2_MUDEL				(1 << 7)
#define MU_CNT2_SRCH_MODE(x)		((((x) & 0x3) << 5))
#define MU_CNT2_SET_PHS(x)			((((x) & 0x1F) << 0))

/* REG_MU_CNT3 */
#define MU_CNT3_MUDEL(x)			((((x) & 0xFF) << 0))

/* REG_MU_CNT4 */
#define MU_CNT4_SEARCH_TOL			(1 << 7)
#define MU_CNT4_RETRY				(1 << 6)
#define MU_CNT4_CONTRST				(1 << 5)
#define MU_CNT4_GUARD(x)			((((x) & 0x1F) << 0))

/* REG_MU_STAT1 */
#define MU_STAT1_MU_LST				(1 << 1)
#define MU_STAT1_MU_LKD				(1 << 0)

/* REG_PART_ID */
#define PART_ID_PART_ID(x)			((((x) & 0xFF) << 0))
#define AD9739A_ID					0x24

#define AD9739A_MIN_FSC				8580	// 8.58 mA
#define AD9739A_MAX_FSC				31700	// 31.6998 mA

enum operation_mode {
	NORMAL_BASEBAND_OPERATION,
	MIX_MODE_OPERATION,
};

#endif /* end ifndef _DEF_AD9739A_H */
