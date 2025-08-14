/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MAX77675 Register Definitions
 * Reference: MAX77675 Datasheet
 */

#ifndef __MAX77675_REG_H__
#define __MAX77675_REG_H__

#include <linux/bitops.h>

/* Register Addresses */
#define MAX77675_REG_CNFG_GLBL_A     0x00
#define MAX77675_REG_CNFG_GLBL_B     0x01
#define MAX77675_REG_INT_GLBL        0x02
#define MAX77675_REG_INTM_GLBL       0x03
#define MAX77675_REG_STAT_GLBL       0x04
#define MAX77675_REG_ERCF_GLBL       0x05
#define MAX77675_REG_CID             0x06
#define MAX77675_REG_CNFG_SBB_TOP_A  0x07
#define MAX77675_REG_CNFG_SBB0_A     0x08
#define MAX77675_REG_CNFG_SBB0_B     0x09
#define MAX77675_REG_CNFG_SBB1_A     0x0A
#define MAX77675_REG_CNFG_SBB1_B     0x0B
#define MAX77675_REG_CNFG_SBB2_A     0x0C
#define MAX77675_REG_CNFG_SBB2_B     0x0D
#define MAX77675_REG_CNFG_SBB3_A     0x0E
#define MAX77675_REG_CNFG_SBB3_B     0x0F
#define MAX77675_REG_CNFG_SBB_TOP_B  0x10

/* CNFG_GLBL_A (0x00) bit masks and shifts */
#define MAX77675_MRT_MASK           GENMASK(7, 6)    /* Manual Reset Time (bits 7:6) */
#define MAX77675_MRT_SHIFT          6
#define MAX77675_PU_DIS_BIT         BIT(5)           /* Pullup Disable (bit 5) */
#define MAX77675_PU_DIS_SHIFT       5
#define MAX77675_BIAS_LPM_BIT       BIT(4)           /* Bias Low Power Mode (bit 4) */
#define MAX77675_BIAS_LPM_SHIFT     4
#define MAX77675_SIMO_CH_DIS_BIT    BIT(3)           /* SIMO Internal Channel Disable (bit 3) */
#define MAX77675_SIMO_CH_DIS_SHIFT  3
#define MAX77675_EN_MODE_MASK      GENMASK(2, 1)    /* nEN Mode (bits 2:1) */
#define MAX77675_EN_MODE_SHIFT     1
#define MAX77675_DBEN_EN_BIT      BIT(0)           /* Debounce Enable (bit 0) */
#define MAX77675_DBEN_EN_SHIFT    0

/* CNFG_GLBL_B (0x01) */
#define MAX77675_SFT_CTRL_MASK      GENMASK(2, 0)    /* Soft Start Control */
#define MAX77675_SFT_CTRL_SHIFT     0

/* INT_GLBL (0x02) bit bits and shifts */
#define MAX77675_INT_SBB3_F_BIT     BIT(7)
#define MAX77675_INT_SBB3_F_SHIFT   7
#define MAX77675_INT_SBB2_F_BIT     BIT(6)
#define MAX77675_INT_SBB2_F_SHIFT   6
#define MAX77675_INT_SBB1_F_BIT     BIT(5)
#define MAX77675_INT_SBB1_F_SHIFT   5
#define MAX77675_INT_SBB0_F_BIT     BIT(4)
#define MAX77675_INT_SBB0_F_SHIFT   4
#define MAX77675_INT_TJAL2_R_BIT    BIT(3)
#define MAX77675_INT_TJAL2_R_SHIFT  3
#define MAX77675_INT_TJAL1_R_BIT    BIT(2)
#define MAX77675_INT_TJAL1_R_SHIFT  2
#define MAX77675_INT_EN_R_BIT       BIT(1)
#define MAX77675_INT_EN_R_SHIFT     1
#define MAX77675_INT_EN_F_BIT       BIT(0)
#define MAX77675_INT_EN_F_SHIFT     0

/* INTM_GLBL (0x03) bits and shifts */
#define MAX77675_INTM_SBB3_F_BIT    BIT(7)
#define MAX77675_INTM_SBB3_F_SHIFT  7
#define MAX77675_INTM_SBB2_F_BIT    BIT(6)
#define MAX77675_INTM_SBB2_F_SHIFT  6
#define MAX77675_INTM_SBB1_F_BIT    BIT(5)
#define MAX77675_INTM_SBB1_F_SHIFT  5
#define MAX77675_INTM_SBB0_F_BIT    BIT(4)
#define MAX77675_INTM_SBB0_F_SHIFT  4
#define MAX77675_INTM_TJAL2_R_BIT   BIT(3)
#define MAX77675_INTM_TJAL2_R_SHIFT 3
#define MAX77675_INTM_TJAL1_R_BIT   BIT(2)
#define MAX77675_INTM_TJAL1_R_SHIFT 2
#define MAX77675_INTM_EN_R_BIT      BIT(1)
#define MAX77675_INTM_EN_R_SHIFT    1
#define MAX77675_INTM_EN_F_BIT      BIT(0)
#define MAX77675_INTM_EN_F_SHIFT    0

/* STAT_GLBL (0x04) bits and shifts */
#define MAX77675_STAT_SBB3_S_BIT    BIT(7)
#define MAX77675_STAT_SBB3_S_SHIFT  7
#define MAX77675_STAT_SBB2_S_BIT    BIT(6)
#define MAX77675_STAT_SBB2_S_SHIFT  6
#define MAX77675_STAT_SBB1_S_BIT    BIT(5)
#define MAX77675_STAT_SBB1_S_SHIFT  5
#define MAX77675_STAT_SBB0_S_BIT    BIT(4)
#define MAX77675_STAT_SBB0_S_SHIFT  4
#define MAX77675_STAT_TJAL2_S_BIT   BIT(2)
#define MAX77675_STAT_TJAL2_S_SHIFT 2
#define MAX77675_STAT_TJAL1_S_BIT   BIT(1)
#define MAX77675_STAT_TJAL1_S_SHIFT 1
#define MAX77675_STAT_STAT_EN_BIT   BIT(0)
#define MAX77675_STAT_STAT_EN_SHIFT 0

#define MAX77675_STAT_STAT_EN_BIT   BIT(0)
#define MAX77675_STAT_STAT_EN_SHIFT 0

/* ERCFLAG (0x05) bits and shifts */
#define MAX77675_SFT_CRST_F_BIT      BIT(5)  /* Software Cold Reset Flag */
#define MAX77675_SFT_CRST_F_SHIFT    5
#define MAX77675_SFT_OFF_F_BIT       BIT(4)  /* Software Off Flag */
#define MAX77675_SFT_OFF_F_SHIFT     4
#define MAX77675_MRST_BIT            BIT(3)  /* Manual Reset Timer Flag */
#define MAX77675_MRST_SHIFT          3
#define MAX77675_UVLO_BIT            BIT(2)  /* Undervoltage Lockout Flag */
#define MAX77675_UVLO_SHIFT          2
#define MAX77675_OVLO_BIT            BIT(1)  /* Overvoltage Lockout Flag */
#define MAX77675_OVLO_SHIFT          1
#define MAX77675_TOVLD_BIT           BIT(0)  /* Thermal Overload Flag */
#define MAX77675_TOVLD_SHIFT         0

/* CID (0x06) bits and shifts */
#define MAX77675_CID_MASK           GENMASK(4, 0)  /* Chip Identification Code mask */
#define MAX77675_CID_SHIFT          0              /* Starts at bit 0 */

/* CNFG_SBB_TOP_A (0x07) bits and shifts */
#define MAX77675_STEP_SZ_SBB3_BIT   BIT(5)
#define MAX77675_STEP_SZ_SBB3_SHIFT 5
#define MAX77675_STEP_SZ_SBB2_BIT   BIT(4)
#define MAX77675_STEP_SZ_SBB2_SHIFT 4
#define MAX77675_STEP_SZ_SBB1_BIT   BIT(3)
#define MAX77675_STEP_SZ_SBB1_SHIFT 3
#define MAX77675_STEP_SZ_SBB0_BIT   BIT(2)
#define MAX77675_STEP_SZ_SBB0_SHIFT 2
#define MAX77675_DRV_SBB_MASK       GENMASK(1, 0)
#define MAX77675_DRV_SBB_SHIFT      0

/* CNFG_SBB0_A (0x08) bits and shifts */
#define MAX77675_TV_SBB0_MASK       GENMASK(7, 0)
#define MAX77675_TV_SBB0_SHIFT      0

/* CNFG_SBB0_B (0x09) bits and shifts */
#define MAX77675_ADE_SBB0_BIT       BIT(3)
#define MAX77675_ADE_SBB0_SHIFT     3
#define MAX77675_EN_SBB0_MASK       GENMASK(2, 0)
#define MAX77675_EN_SBB0_SHIFT      0

/* CNFG_SBB1_A (0x0A) bits and shifts */
#define MAX77675_TV_SBB1_MASK       GENMASK(7, 0)
#define MAX77675_TV_SBB1_SHIFT      0

/* CNFG_SBB1_B (0x0B) bits and shifts */
#define MAX77675_ADE_SBB1_BIT       BIT(3)
#define MAX77675_ADE_SBB1_SHIFT     3
#define MAX77675_EN_SBB1_MASK       GENMASK(2, 0)
#define MAX77675_EN_SBB1_SHIFT      0

/* CNFG_SBB2_A (0x0C) bits and shifts */
#define MAX77675_TV_SBB2_MASK       GENMASK(7, 0)
#define MAX77675_TV_SBB2_SHIFT      0

/* CNFG_SBB2_B (0x0D) bits and shifts */
#define MAX77675_ADE_SBB2_BIT       BIT(3)
#define MAX77675_ADE_SBB2_SHIFT     3
#define MAX77675_EN_SBB2_MASK       GENMASK(2, 0)
#define MAX77675_EN_SBB2_SHIFT      0

/* CNFG_SBB3_A (0x0E) bits and shifts */
#define MAX77675_TV_SBB3_MASK       GENMASK(7, 0)
#define MAX77675_TV_SBB3_SHIFT      0

/* CNFG_SBB3_B (0x0F) bits and shifts */
#define MAX77675_ADE_SBB3_BIT       BIT(3)
#define MAX77675_ADE_SBB3_SHIFT     3
#define MAX77675_EN_SBB3_MASK       GENMASK(2, 0)
#define MAX77675_EN_SBB3_SHIFT      0

#define MAX77675_EN_SBB_MASK       GENMASK(2, 0)

/* CNFG_SBB_TOP_B (0x10) bits and shifts */
#define MAX77675_DVS_SLEW_BIT       BIT(5)
#define MAX77675_DVS_SLEW_SHIFT     5
#define MAX77675_LAT_MODE_BIT       BIT(4)
#define MAX77675_LAT_MODE_SHIFT     4
#define MAX77675_SR_SBB3_BIT        BIT(3)
#define MAX77675_SR_SBB3_SHIFT      3
#define MAX77675_SR_SBB2_BIT        BIT(2)
#define MAX77675_SR_SBB2_SHIFT      2
#define MAX77675_SR_SBB1_BIT        BIT(1)
#define MAX77675_SR_SBB1_SHIFT      1
#define MAX77675_SR_SBB0_BIT        BIT(0)
#define MAX77675_SR_SBB0_SHIFT      0

#define MAX77675_MAX_REGISTER       0x10

/* Common minimum voltage (in microvolts) */
#define MAX77675_MIN_UV             500000     // 500 mV

/* Voltage step configuration for 25mV mode */
#define MAX77675_STEP_25MV          25000      // Step size: 25 mV
#define MAX77675_MAX_UV_25MV        5500000    // Max voltage: 5.5 V
#define MAX77675_NUM_LEVELS_25MV    201        // levels = (5500mV - 500mV) / 25mV + 1

/* Voltage step configuration for 12.5mV mode */
#define MAX77675_STEP_12_5MV        12500      // Step size: 12.5 mV
#define MAX77675_MAX_UV_12_5MV      3687500    // Max voltage: 3.6875 V
#define MAX77675_NUM_LEVELS_12_5MV  255        // levels = (3687.5mV - 500mV) / 12.5mV + 1

#define MAX77675_ENABLE_MASK        0x07
#define MAX77675_ENABLE_OFF         0x04
#define MAX77675_ENABLE_ON          0x06

#define MAX77675_REGULATOR_AD_MASK  BIT(3)
#define MAX77675_REGULATOR_AD_OFF   0x00
#define MAX77675_REGULATOR_AD_ON    BIT(3)

/* FPS Source */
enum max77675_fps_src {
	MAX77675_FPS_SLOT_0,
	MAX77675_FPS_SLOT_1,
	MAX77675_FPS_SLOT_2,
	MAX77675_FPS_SLOT_3,
	MAX77675_FPS_NONE,
	MAX77675_FPS_DEF,
};

/* Regulator ID enumeration */
enum max77675_regulator_id {
	ID_SBB0 = 0,
	ID_SBB1,
	ID_SBB2,
	ID_SBB3,
	MAX77675_NUM_REGULATORS
};

/* nEN Mode options */
enum max77675_en_mode {
	EN_PUSH_BUTTON = 0x0,
	EN_SLIDE_SWITCH = 0x1,
	EN_LOGIC = 0x2,
};

/* Use DVS_SLEW for SR_SBBx (0 = 2mV/us, 1 = use DVS_SLEW) */
enum max77675_slew_rate {
	SLEW_RATE_2MV_PER_US = 0,
	SLEW_RATE_DVS_SLEW   = 1,
};

/* Software Control (SFT_CTRL) values */
enum max77675_sft_ctrl {
	SFT_NO_ACTION        = 0x0,
	SFT_COLD_RESET       = 0x1,
	SFT_OFF              = 0x2,
	SFT_STBY             = 0x3,
	SFT_EXIT_STBY        = 0x4,
};

enum max77675_drv_sbb_strength {
	DRV_SBB_FASTEST = 0x0,  // ~0.6ns
	DRV_SBB_FAST	= 0x1,	// ~1.2ns
	DRV_SBB_SLOW	= 0x2,	// ~1.8ns
	DRV_SBB_SLOWEST = 0x3,	// ~8ns
};

struct max77675_irq_status {
	bool sbb3_fault;
	bool sbb2_fault;
	bool sbb1_fault;
	bool sbb0_fault;
	bool tjal2_rising;
	bool tjal1_rising;
	bool nen_rising;
	bool nen_falling;
};

struct max77675_status {
	bool sbb3_overload;
	bool sbb2_overload;
	bool sbb1_overload;
	bool sbb0_overload;
	bool tjal2_status;
	bool tjal1_status;
	bool nen_status;
};

struct max77675_ercf_status {
	bool sft_crst_f;
	bool sft_off_f;
	bool mrst;
	bool uvlo;
	bool ovlo;
	bool tovld;
};

#endif /* __MAX77675_REG_H__ */
