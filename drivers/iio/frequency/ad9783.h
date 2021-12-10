/*
 * AD9783 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#ifndef _DEF_AD9783_H
#define _DEF_AD9783_H

/* AD9783 Registers */

// added from datasheet
#define REG_SPI_CTRL			0x00
#define REG_DATA_CTRL 			0x02
#define REG_POWER_DOWN 			0x03
#define REG_SETUP_HOLD 			0x04
#define REG_SMP				0x05
#define REG_SEEK			0x06 // READ ONLY REGISTER
#define REG_MIX_MODE			0x0A
#define REG_DAC_1_FSC			0x0B
#define REG_DAC_1_FSC_MSB		0x0C
#define REG_AUXDAC1			0x0D
#define REG_AUXDAC1_MSB			0x0E
#define REG_DAC_2_FSC			0x0F
#define REG_DAC_2_FSC_MSB		0x10
#define REG_BIST_CTRL			0x1A
#define REG_BIST_RES_1_LOW		0x1B
#define REG_BIST_RES_1_HIGH		0x1C
#define REG_BIST_RES_2_LOW		0x1D
#define REG_BIST_RES_2_HIGH		0x1E
#define REG_VERSION_PART_ID		0x1F
#define AD9783_ID			0x00


/* REG_DAC_1_FSC */
#define MODE_DAC_1_FSC			0xF9
#define MODE_DAC_1_FSC_MSB		0x01

/* REG_DAC_2_FSC */
#define MODE_DAC_2_FSC			0xF9
#define MODE_DAC_2_FSC_MSB		0x01

/* REG_POWER_DOWN */
#define POWER_DOWN_DCO_PD		(1 << 7)
#define POWER_DOWN_INPT_PD		(1 << 6)
#define POWER_DOWN_AUX2_PD		(1 << 5)
#define POWER_DOWN_AUX1_PD		(1 << 4)
#define POWER_DOWN_BIAS_PD		(1 << 3)
#define POWER_DOWN_CLK_RCVR_PD		(1 << 2)
#define POWER_DOWN_DAC_2_PD		(1 << 1)
#define POWER_DOWN_DAC_1_PD		(1 << 0)

/* REG_MIX_MODE */
#define NORMAL_BASEBAND			0
#define MIX_MODE			2



// existent already from AD9739A
//#define REG_CNT_CLK_DIS			0x02
//#define REG_IRQ_EN			0x03
//#define REG_IRQ_REQ			0x04
//#define REG_FSC_1			0x06
//#define REG_FSC_2			0x07
//#define REG_LVDS_STAT1		0x0C
//#define REG_LVDS_REC_CNT1		0x10
//#define REG_LVDS_REC_CNT2		0x11
//#define REG_LVDS_REC_CNT3		0x12
//#define REG_LVDS_REC_CNT4		0x13
//#define REG_LVDS_REC_CNT5		0x14
//#define REG_LVDS_REC_STAT1		0x19
//#define REG_LVDS_REC_STAT2		0x1A
//#define REG_LVDS_REC_STAT3		0x1B
//#define REG_LVDS_REC_STAT4		0x1C
//#define REG_LVDS_REC_STAT9		0x21


///* REG_MODE */
//#define MODE_SDIO_DIR			((1 << 7) | (1 << 0))
//#define MODE_LSB			((1 << 6) | (1 << 1))
//#define MODE_RESET			((1 << 5) | (1 << 2))

///* REG_CNT_CLK_DIS */
//#define CNT_CLK_DIS_CLKGEN_PD		(1 << 3)
//#define CNT_CLK_DIS_REC_CNT_CLK		(1 << 1)
//#define CNT_CLK_DIS_MU_CNT_CLK		(1 << 0)

///* REG_IRQ_EN */
//#define IRQ_EN_MU_LST_EN		(1 << 3)
//#define IRQ_EN_MU_LCK_EN		(1 << 2)
//#define IRQ_EN_RCV_LST_EN		(1 << 1)
//#define IRQ_EN_RCV_LCK_EN		(1 << 0)

///* REG_IRQ_REQ */
//#define IRQ_REQ_MU_LST_IRQ		(1 << 3)
//#define IRQ_REQ_MU_LCK_IRQ		(1 << 2)
//#define IRQ_REQ_RCV_LST_IRQ		(1 << 1)
//#define IRQ_REQ_RCV_LCK_IRQ		(1 << 0)

/* REG_FSC_1 */
#define FSC_1_FSC_1(x) 			(((x) & 0xFF) << 0)

/* REG_FSC_2 */
#define FSC_2_FSC_2(x)			(((x) & 0x3) << 0)
#define FSC_2_SLEEP			(1 << 7)

///* REG_LVDS_STAT1 */
//#define LVDS_STAT1_DCI_PRE_PH0		(1 << 2)
//#define LVDS_STAT1_DCI_PST_PH0		(1 << 0)
//
///* REG_LVDS_REC_CNT1 */
//#define LVDS_REC_CNT1_RCVR_FLG_RST	(1 << 2)
//#define LVDS_REC_CNT1_RCVR_LOOP_ON	(1 << 1)
//#define LVDS_REC_CNT1_RCVR_CNT_ENA	(1 << 0)
//
///* REG_LVDS_REC_CNT2 */
//#define LVDS_REC_CNT2_SMP_DEL(x)	(((x) & 0x3) << 6)
//
///* REG_LVDS_REC_CNT3 */
//#define LVDS_REC_CNT3_SMP_DEL(x)	(((x) & 0xFF) << 0)
//
///* REG_LVDS_REC_CNT4 */
//#define LVDS_REC_CNT4_DCI_DEL(x)	(((x) & 0xF) << 4)
//#define LVDS_REC_CNT4_FINE_DEL_SKEW(x)	(((x) & 0xF) << 0)
//
///* REG_LVDS_REC_CNT5 */
//#define LVDS_REC_CNT5_DCI_DEL(x)	(((x) & 0x3F) << 0)
//
///* REG_LVDS_REC_STAT1 */
//#define LVDS_REC_STAT1_SMP_DEL(x)	(((x) & 0x3) << 6)
//
///* REG_LVDS_REC_STAT2 */
//#define LVDS_REC_STAT2_SMP_DEL(x)	(((x) & 0xFF) << 0)
//
///* REG_LVDS_REC_STAT3 */
//#define LVDS_REC_STAT3_DCI_DEL(x)	(((x) & 0x3) << 6)
//
///* REG_LVDS_REC_STAT4 */
//#define LVDS_REC_STAT4_DCI_DEL(x)	(((x) & 0xFF) << 0)
//
///* REG_LVDS_REC_STAT9 */
//#define LVDS_REC_STAT9_RCVR_TRK_ON	(1 << 3)
//#define LVDS_REC_STAT9_RCVR_FE_ON	(1 << 2)
//#define LVDS_REC_STAT9_RCVR_LST		(1 << 1)
//#define LVDS_REC_STAT9_RCVR_LCK		(1 << 0)


#define AD9783_MIN_FSC			8580	// 8.58 mA
#define AD9783_MAX_FSC			31700	// 31.6998 mA

enum operation_mode {
	NORMAL_BASEBAND_OPERATION,
	MIX_MODE_OPERATION,
};

#endif /* end ifndef _DEF_AD9783_H */
