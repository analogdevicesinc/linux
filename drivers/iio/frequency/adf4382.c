// SPDX-License-Identifier: GPL-2.0
/*
 * ADF4382 Microwave Wideband Synthesizer with Integrated VCO
 *
 * Copyright 2022-2024 Analog Devices Inc.
 */

// TODO:JONATHANC: about the patch message
// Hi Ciprian,
// No real need to repeat all of this.  Perhaps a shorter version for this patch, or something about what features are implemented instead of so much device info.
// Various questions and comments inline.  In some cases I have picked out some examples of a particular code platform, please generalize and apply to all similar code for v2.
// Thanks,
// Jonathan

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/gcd.h>
#include <linux/math64.h>
#include <linux/units.h>
#include <linux/util_macros.h>

/* ADF4382 REG0000 Map */
#define ADF4382_SOFT_RESET_R_MSK		BIT(7)
#define ADF4382_LSB_FIRST_R_MSK			BIT(6)
#define ADF4382_ADDRESS_ASC_R_MSK		BIT(5)
#define ADF4382_SDO_ACTIVE_R_MSK		BIT(4)
#define ADF4382_SDO_ACTIVE_MSK			BIT(3)
#define ADF4382_ADDRESS_ASC_MSK			BIT(2)
#define ADF4382_LSB_FIRST_MSK			BIT(1)
#define ADF4382_SOFT_RESET_MSK			BIT(0)

/* ADF4382_REG0 */
#define ADF4382_ADDR_ASC_MSK			BIT(2)
#define ADF4382_ADDR_ASC_R_MSK			BIT(5)
#define ADF4382_SDO_ACT_MSK			BIT(3)
#define ADF4382_SDO_ACT_R_MSK			BIT(4)
#define ADF4382_RESET_CMD			0x81

/* ADF4382 REG0000 Bit Definition */
#define ADF4382_SDO_ACTIVE_SPI_3W		0x0
#define ADF4382_SDO_ACTIVE_SPI_4W		0x1

#define ADF4382_ADDR_ASC_AUTO_DECR		0x0
#define ADF4382_ADDR_ASC_AUTO_INCR		0x1

#define ADF4382_LSB_FIRST_MSB			0x0
#define ADF4382_LSB_FIRST_LSB			0x1

#define ADF4382_SOFT_RESET_N_OP			0x0
#define ADF4382_SOFT_RESET_EN			0x1

/* ADF4368 REG0011 Map*/
#define ADF4368_CLKOUT_DIV_MSK			GENMASK(7, 6)
#define ADF4368_INT_MODE_MSK			BIT(5)

/* ADF4368 REG004E Map */
#define ADF4368_DCLK_DIV1_MSK			GENMASK(5, 4)

/* ADF4368 REG0039 Map */
#define ADF4368_SYNTH_MSB_MSK			GENMASK(6, 0)

/* ADF4368 REG0038 Map */
#define ADF4368_SYNTH_LSB_MSK			GENMASK(7, 0)

/* ADF4368 REG003B Map */
#define ADF4368_VCO_ALC_MSB_MSK			GENMASK(6, 0)

/* ADF4368 REG003A Map */
#define ADF4368_VCO_ALC_LSB_MSK			GENMASK(7, 0)

/* ADF4368 REG0035 Map */
#define ADF4368_DCLK_MODE_MSK			BIT(2)

/* ADF4368 REG0031 Map */
#define ADF4368_EN_ADC_CLK_MSK			BIT(3)

/* ADF4368 REG0058 Map */
#define ADF4368_ADC_BUSY_MSK			BIT(2)
#define ADF4368_FSM_BUSY_MSK			BIT(1)

/* ADF4382 REG0001 Map */
#define ADF4382_SINGLE_INSTR_MSK		BIT(7)
#define ADF4382_MASTER_RB_CTRL_MSK		BIT(5)

/* ADF4382 REG0001 Bit Definition */
#define ADF4382_SPI_STREAM_EN			0x0
#define ADF4382_SPI_STREAM_DIS			0x1

#define ADF4382_RB_SLAVE_REG			0x0
#define ADF4382_RB_MASTER_REG			0x1

/* ADF4382 REG0003 Bit Definition */
#define ADF4382_CHIP_TYPE			0x06

/* ADF4382 REG0004 Bit Definition */
#define ADF4382_PRODUCT_ID_LSB			0x0005

/* ADF4382 REG0005 Bit Definition */
#define ADF4382_PRODUCT_ID_MSB			0x0005

/* ADF4382 REG000A Map */
#define ADF4382_SCRATCHPAD_MSK			GENMASK(7, 0)

/* ADF4382 REG000C Bit Definition */
#define ADF4382_VENDOR_ID_LSB			0x56

/* ADF4382 REG000D Bit Definition */
#define ADF4382_VENDOR_ID_MSB			0x04

/* ADF4382 REG000F Bit Definition */
#define ADF4382_M_S_TRANSF_BIT_MSK		BIT(0)

/* ADF4382 REG0010 Map*/
#define ADF4382_N_INT_LSB_MSK			GENMASK(7, 0)

/* ADF4382 REG0011 Map*/
#define ADF4382_CLKOUT_DIV_MSK			GENMASK(7, 5)
#define ADF4382_INV_CLK_OUT_MSK			BIT(4)
#define ADF4382_N_INT_MSB_MSK			GENMASK(3, 0)

/* ADF4382 REG0015 Map */
#define ADF4382_M_VCO_BAND_LSB_MSK		BIT(7)
#define ADF4382_M_VCO_CORE_MSK			BIT(6)
#define ADF4382_BIAS_DEC_MODE_MSK		GENMASK(5, 3)
#define ADF4382_INT_MODE_MSK			BIT(2)
#define ADF4382_PFD_POL_MSK			BIT(1)
#define ADF4382_FRAC1WORD_MSB			BIT(0)

/* ADF4382 REG0016 Map */
#define ADF4382_M_VCO_BAND_MSB_MSK		GENMASK(7, 0)

/* ADF4382 REG001D Map */
#define ADF4382_BLEED_I_LSB_MSK			GENMASK(7, 0)

/* ADF4382 REG001E Map */
#define ADF4382_EN_PHASE_RESYNC_MSK		BIT(7)
#define ADF4382_EN_REF_RST_MSK			BIT(6)
#define ADF4382_TIMED_SYNC_MSK			BIT(5)
#define ADF4382_BLEED_I_MSB_MSK			GENMASK(4, 0)

/* ADF4382 REG001F Map */
#define ADF4382_SW_SYNC_MSK			BIT(7)
#define ADF4382_SPARE_1F_MSK			BIT(6)
#define ADF4382_BLEED_POL_MSK			BIT(5)
#define ADF4382_EN_BLEED_MSK			BIT(4)
#define ADF4382_CP_I_MSK			GENMASK(3, 0)

/* ADF4382 REG0020 Map */
#define ADF4382_EN_AUTOCAL_MSK			BIT(7)
#define ADF4382_EN_RDBLR_MSK			BIT(6)
#define ADF4382_R_DIV_MSK			GENMASK(5, 0)

/* ADF4382 REG0021 Map */
#define ADF4382_PHASE_WORD_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0022 Map */
#define ADF4382_PHASE_WORD_MID_MSK		GENMASK(7, 0)

/* ADF4382 REG0023 Map */
#define ADF4382_PHASE_WORD_MSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0024 Map */
#define ADF4382_SPARE_24_MSK			GENMASK(7, 5)
#define ADF4382_DCLK_DIV_SEL_MSK		BIT(4)
#define ADF4382_DNCLK_DIV1_MSK			GENMASK(3, 2)
#define ADF4382_DCLK_DIV1_MSK			GENMASK(1, 0)

/* ADF4382 REG0025 Map */
#define ADF4382_RESYNC_WAIT_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0026 Map */
#define ADF4382_RESYNC_WAIT_MSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0027 Map */
#define ADF4382_CAL_BLEED_FINE_MIN_MSK		GENMASK(7, 4)
#define ADF4382_BLEED_ADJ_SCALE_MSK		GENMASK(3, 0)

/* ADF4382 REG0028 Map */
#define ADF4382_PH_RESYNC_RB_SEL_MSK		BIT(7)
#define ADF4382_LSB_P1_MSK			BIT(6)
#define ADF4382_VAR_MOD_EN_MSK			BIT(5)
#define ADF4382_DITHER1_SCALE_MSK		GENMASK(4, 2)
#define ADF4382_EN_DITHER2_MSK			BIT(1)
#define ADF4382_EN_DITHER1_MSK			BIT(0)

/* ADF4382 REG0029 Map */
#define ADF4382_CLK2_OPWR_MSK			GENMASK(7, 4)
#define ADF4382_CLK1_OPWR_MSK			GENMASK(3, 0)

/* ADF4382 REG002A Map */
#define ADF4382_FN_DBL_MSK			BIT(7)
#define ADF4382_PD_NDIV_TL_MSK	BIT(6)
#define ADF4382_CLKOUT_BST_MSK			BIT(5)
#define ADF4382_PD_SYNC_MSK			BIT(4)
#define ADF4382_PD_CLK_MSK			BIT(3)
#define ADF4382_PD_RDET_MSK			BIT(2)
#define ADF4382_PD_ADC_MSK			BIT(1)
#define ADF4382_PD_CALGEN_MSK			BIT(0)

/* ADF4382 REG002B Map */
#define ADF4382_PD_ALL_MSK			BIT(7)
#define ADF4382_PD_RDIV_TL_MSK			BIT(6)
#define ADF4382_PD_NDIV_MSK			BIT(5)
#define ADF4382_PD_VCO_MSK			BIT(4)
#define ADF4382_PD_LD_MSK			BIT(3)
#define ADF4382_PD_PFDCP_MSK			BIT(2)
#define ADF4382_PD_CLKOUT1_MSK			BIT(1)
#define ADF4382_PD_CLKOUT2_MSK			BIT(0)

/* ADF4382 REG002C Map */
#define ADF4382_LDWIN_PW_MSK			GENMASK(7, 4)
#define ADF4382_LD_COUNT_OPWR_MSK		GENMASK(3, 0)

/* ADF4382 REG002D Map */
#define ADF4382_EN_DNCLK_MSK			BIT(7)
#define ADF4382_EN_DRCLK_MSK			BIT(6)
#define ADF4382_EN_LOL_MSK			BIT(5)
#define ADF4382_EN_LDWIN_MSK			BIT(4)
#define ADF4382_PDET_POL_MSK			BIT(3)
#define ADF4382_RST_LD_MSK			BIT(2)
#define ADF4382_LD_O_CTRL_MSK			GENMASK(1, 0)

/* ADF4382 REG002E Map */
#define ADF4382_MUXOUT_MSK			GENMASK(7, 4)
#define ADF4382_ABPW_WD_MSK			BIT(3)
#define ADF4382_EN_CPTEST_MSK			BIT(2)
#define ADF4382_CP_DOWN_MSK			BIT(1)
#define ADF4382_CP_UP_MSK			BIT(0)

/* ADF4382 REG002F Map*/
#define ADF4382_BST_REF_MSK			BIT(7)
#define ADF4382_FILT_REF_MSK			BIT(6)
#define ADF4382_RDBLR_DC_MSK			GENMASK(5, 0)

/* ADF4382 REG0030 Map */
#define ADF4382_MUTE_NCLK_MSK			BIT(7)
#define ADF4382_MUTE_RCLK_MSK			BIT(6)
#define ADF4382_REF_SEL_MSK			BIT(5)
#define ADF4382_INV_RDBLR_MSK			BIT(4)
#define ADF4382_RDBLR_DEL_SEL_MSK		GENMASK(3, 0)

/* ADF4382 REG0031 Map */
#define ADF4382_SYNC_DEL_MSK			GENMASK(7, 5)
#define ADF4382_RST_SYS_MSK			BIT(4)
#define ADF4382_EN_ADC_CLK_MSK			BIT(3)
#define ADF4382_EN_VCAL_MSK			BIT(2)
#define ADF4382_CAL_CT_SEL_MSK			BIT(1)
#define ADF4382_DCLK_MODE_MSK			BIT(0)

/* ADF4382 REG0032 Map */
#define ADF4382_SPARE_32_MSK			BIT(7)
#define ADF4382_BLEED_ADJ_CAL_MSK		BIT(6)
#define ADF4382_DEL_MODE_MSK			BIT(5)
#define ADF4382_EN_AUTO_ALIGN_MSK		BIT(4)
#define ADF4382_PHASE_ADJ_POL_MSK		BIT(3)
#define ADF4382_EFM3_MODE_MSK			GENMASK(2, 0)

/* ADF4382 REG0033 Map */
#define ADF4382_PHASE_ADJUST_MSK		GENMASK(7, 0)

/* ADF4382 REG0034 Map */
#define ADF4382_PHASE_ADJ_MSK			BIT(7)
#define ADF4382_DRCLK_DEL_MSK			GENMASK(6, 4)
#define ADF4382_DNCLK_DEL_MSK			GENMASK(3, 1)
#define ADF4382_RST_CNTR_MSK			BIT(0)

/* ADF4382 REG0035 Map */
#define ADF4382_SPARE_35_MSK			GENMASK(7, 6)
#define ADF4382_M_VCO_BIAS_MSK			GENMASK(5, 0)

/* ADF4382 REG0036 Map */
#define ADF4382_CLKODIV_DB_MSK			BIT(7)
#define ADF4382_DCLK_DIV_DB_MSK			BIT(6)
#define ADF4382_SPARE_36_MSK			GENMASK(5, 2)
#define ADF4382_EN_LUT_GEN_MSK			BIT(1)
#define ADF4382_EN_LUT_CAL_MSK			BIT(0)

/* ADF4382 REG0037 Map */
#define ADF4382_CAL_COUNT_TO_MSK		GENMASK(7, 0)

/* ADF4382 REG0038 Map */
#define ADF4382_CAL_VTUNE_TO_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0039 Map */
#define ADF4382_O_VCO_DB_MSK			BIT(7)
#define ADF4382_CAL_VTUNE_TO_MSB_MSK		GENMASK(6, 0)

/* ADF4382 REG003A Map */
#define ADF4382_CAL_VCO_TO_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG003B Map */
#define ADF4382_DEL_CTRL_DB_MSK			BIT(7)
#define ADF4382_CAL_VCO_TO_MSB_MSK		GENMASK(6, 0)

/* ADF4382 REG003C Map */
#define ADF4382_CNTR_DIV_WORD_MSK		GENMASK(7, 0)

/* ADF4382 REG003D Map */
#define ADF4382_SPARE_3D_MSK			BIT(7)
#define ADF4382_SYNC_SP_DB_MSK			BIT(6)
#define ADF4382_CMOS_OV_MSK			BIT(5)
#define ADF4382_READ_MODE_MSK			BIT(4)
#define ADF4382_CNTR_DIV_WORD_MSB_MSK		GENMASK(3, 0)

/* ADF4382 REG003E Map */
#define ADF4382_ADC_CLK_DIV_MSK			GENMASK(7, 0)

/* ADF4382 REG003F Map */
#define ADF4382_EN_ADC_CNV_MSK			BIT(7)
#define ADF4382_EN_ADC_VTEST_MSK		BIT(6)
#define ADF4382_ADC_VTEST_SEL_MSK		BIT(5)
#define ADF4382_ADC_MUX_SEL_MSK			BIT(4)
#define ADF4382_ADC_F_CONV_MSK			BIT(3)
#define ADF4382_ADC_C_CONV_MSK			BIT(2)
#define ADF4382_EN_ADC_MSK			BIT(1)
#define ADF4382_SPARE_3F_MSK			BIT(0)

/* ADF4382 REG0040 Map */
#define ADF4382_EXT_DIV_DEC_SEL_MSK		BIT(7)
#define ADF4382_ADC_CLK_TEST_SEL_MSK		BIT(6)
#define ADF4382_MUTE_CLKOUT2_MSK		GENMASK(5, 3)
#define ADF4382_MUTE_CLKOUT1_MSK		GENMASK(2, 0)

/* ADF4382 REG0041 Map */
#define ADF4382_EXT_DIV_MSK			GENMASK(7, 5)
#define ADF4382_EN_VCO_CAP_TEST_MSK		BIT(4)
#define ADF4382_EN_CALGEN_CAP_TEST_MSK		BIT(3)
#define ADF4382_EN_CP_CAP_TEST_MSK		BIT(2)
#define ADF4382_CAP_TEST_STATE_MSK		BIT(1)
#define ADF4382_TRANS_LOOP_SEL_MSK		BIT(0)

/* ADF4382 REG0042 Map */
#define ADF4382_NDIV_PWRUP_TIMEOUT_MSK		GENMASK(7, 0)

/* ADF4382 REG0043 Map */
#define ADF4382_CAL_BLEED_FINE_MAX_MSK		GENMASK(7, 0)

/* ADF4382 REG0044 Map */
#define ADF4382_VCAL_ZERO_MSK			BIT(7)
#define ADF4382_VPTAT_CALGEN_MSK		GENMASK(6, 0)

/* ADF4382 REG0045 Map */
#define ADF4382_SPARE_45_MSK			BIT(7)
#define ADF4382_VCTAT_CALGEN_MSK		GENMASK(6, 0)

/* ADF4382 REG0046 Map */
#define ADF4382_NVMDIN_MSK			GENMASK(7, 0)

/* ADF4382 REG0047 Map */
#define ADF4382_SPARE_47_MSK			BIT(7)
#define ADF4382_NVMADDR_MSK			GENMASK(6, 3)
#define ADF4382_NVMBIT_SEL			GENMASK(2, 0)

/* ADF4382 REG0048 Map */
#define ADF4382_TRIM_LATCH_MSK			BIT(7)
#define ADF4382_NVMTEST_MSK			BIT(6)
#define ADF4382_NVMPROG_MSK			BIT(5)
#define ADF4382_NVMRD_MSK			BIT(4)
#define ADF4382_NVMSTART_MSK			BIT(3)
#define ADF4382_NVMON_MSK			BIT(2)
#define ADF4382_MARGIN_MSK			GENMASK(1, 0)

/* ADF4382 REG0049 Map */
#define ADF4382_NVMDOUT_MSK			GENMASK(7, 0)

/* ADF4382 REG004A Map */
#define ADF4382_SCAN_MODE_CODE_MSK		GENMASK(7, 0)

/* ADF4382 REG004B Map */
#define ADF4382_TEMP_OFFSET_MSK			GENMASK(7, 0)

/* ADF4382 REG004C Map */
#define ADF4382_SPARE_4C_MSK			GENMASK(7, 6)
#define ADF4382_TEMP_SLOPE_MSK			GENMASK(5, 0)

/* ADF4382 REG004D Map */
#define ADF4382_VCO_FSM_TEST_MUX_MSK		GENMASK(7, 5)
#define ADF4382_SPARE_4D_MSK			GENMASK(4, 3)
#define ADF4382_O_VCO_BIAS_MSK			BIT(2)
#define ADF4382_O_VCO_BAND_MSK			BIT(1)
#define ADF4382_O_VCO_CORE_MSK			BIT(0)

/* ADF4382 REG004E Map */
#define ADF4382_EN_TWO_PASS_CALL_MSK		BIT(4)
#define ADF4382_TWO_PASS_BAND_START_MSK		GENMASK(3, 0)

/* ADF4382 REG004F Map */
#define ADF4382_LUT_SCALE_MSK			GENMASK(7, 0)

/* ADF4382 REG0050 Map */
#define ADF4382_SPARE0_MSK			GENMASK(7, 0)

/* ADF4382 REG0051 Map */
#define ADF4382_SPARE1_MSK			GENMASK(7, 0)

/* ADF4382 REG0052 Map */
#define ADF4382_SYNC_REF_SPARE_MSK		GENMASK(7, 4)
#define ADF4382_SYNC_MON_DEL_MSK		GENMASK(3, 0)

/* ADF4382 REG0053 Map */
#define ADF4382_PD_SYNC_MON_MSK			BIT(6)
#define ADF4382_SYNC_SEL_MSK			BIT(5)
#define ADF4382_RST_SYNC_MON_MSK		BIT(4)
#define ADF4382_SYNC_SH_DEL_MSK			GENMASK(3, 0)

/* ADF4382 REG0054 Map */
#define ADF4382_ADC_ST_CNV_MSK			BIT(0)

/* ADF4382 REG0058 Map */
#define ADF4382_PLL_LOCK_MSK			BIT(0)

#define ADF4382_N_INT_MSK			GENMASK(11, 0)
#define ADF4382_MOD2WORD_MSK			GENMASK(23, 0)
#define ADF4382_FRAC1WORD_MS_BIT_MSK		BIT(24)
#define ADF4382_FRAC1WORD_MSK			GENMASK(24, 0)
#define ADF4382_FRAC2WORD_MSK			GENMASK(23, 0)

#define ADF4382_DEL_CNT_LSB_MSK			GENMASK(7, 0)
#define ADF4382_DEL_CNT_MSB_MSK			GENMASK(15, 8)

#define ADF4382_DEL_CNT_FINE_MSK		GENMASK(8, 0)
#define ADF4382_DEL_CNT_COARSE_MSK		GENMASK(12, 9)
#define ADF4382_DEL_CNT_BLEED_POL_MSK		BIT(15)

#define ADF4382_REF_MIN				10ULL * MEGA	/* 10MHz */
#define ADF4382_REF_MAX				5ULL * GIGA	/* 5GHz */
#define ADF4382_VCO_FREQ_MIN			11ULL * GIGA	/* 11GHz */
#define ADF4382_VCO_FREQ_MAX			22ULL * GIGA	/* 22GHz */
#define ADF4382A_VCO_FREQ_MIN			11500ULL * MEGA	/* 11.5GHz */
#define ADF4382A_VCO_FREQ_MAX			21ULL * GIGA	/* 21GHz */
#define ADF4368_VCO_FREQ_MIN			6400ULL * MEGA	// 6.4GHz
#define ADF4368_VCO_FREQ_MAX			12800ULL * MEGA // 12.8GHz
#define ADF4382_PFD_FREQ_MAX			625ULL * MEGA	/* 625MHz */
#define ADF4382_PFD_FREQ_FRAC_MAX		250ULL * MEGA	/* 250MHz */
#define ADF4382_PFD_FREQ_MIN			5400ULL * KILO	/* 5.4MHz */
#define ADF4382_MOD1WORD			0x2000000ULL	/* 2^25 */
#define ADF4382_MOD2WORD_MAX			0xFFFFFFU	/* 2^24 - 1 */
#define ADF4382_PHASE_RESYNC_MOD2WORD_MAX	0x1FFFFU	/* 2^17 - 1 */
#define ADF4382_CHANNEL_SPACING_MAX		78125U
#define ADF4382_DCLK_DIV1_0_MAX			160ULL * MEGA	/* 160MHz */
#define ADF4382_DCLK_DIV1_1_MAX			320ULL * MEGA	/* 320MHz */
#define ADF4382_OUT_PWR_MAX			15
#define ADF4382_CLKOUT_DIV_REG_VAL_MAX		4
#define ADF4382A_CLKOUT_DIV_REG_VAL_MAX		2
#define ADF4368_CLKOUT_DIV_REG_VAL_MAX		3

#define ADF4382_CP_I_DEFAULT			15
#define ADF4382_OPOWER_DEFAULT			11
#define ADF4382_REF_DIV_DEFAULT			1
#define ADF4382_RFOUT_DEFAULT			2875ULL * MEGA	/* 2.875GHz */

#define ADF4382_SCRATCHPAD_VAL			0xA5

#define ADF4382_PHASE_BLEED_CNST_MUL		511
#define ADF4382_PHASE_BLEED_CNST_DIV		285
#define ADF4382_VCO_CAL_CNT			202
#define ADF4382_VCO_CAL_VTUNE			124
#define ADF4382_VCO_CAL_ALC			250

#define PERIOD_IN_DEG				360
#define PERIOD_IN_DEG_MS			360000

#define ADF4368_PHASE_BLEED_CNST		2044000
#define ADF4368_BLEED_N_INT_TH			35
#define ADF4368_COARSE_BLEED_CNST		202 /* 202ua */ 
#define ADF4368_FINE_BLEED_CNST1		250 /* 250 ua */
#define ADF4368_FINE_BLEED_CNST2		512 /* 250 ua */
#define ADF4368_FINE_BLEED_CNST3		567 /* 567 na */

#define ADF4368_FRAC_N_INT_MIN			19
#define ADF4368_INT_N_INT_MIN			4

#ifdef CONFIG_64BIT
#define ADF4382_CLK_SCALE			1
#else
#define	ADF4382_CLK_SCALE			10ULL
#endif
// REVIEW:JONATHANC: this is about clk framework
// No to this.  Make the maths work either way as this just makes the code hard to
// read.  We don't really care about 32bit much any more so just use the relevant
// handlers and 64 bit maths. It will slower on a 32bit system but we don't care.

// If this is about the int size of val and val2, just assume 32 bit always and
// make sure what you use fits.

enum {
	ADF4382_FREQ,
	ADF4382_EN_AUTO_ALIGN,
};

// TODO:JONATHANC:
//This should go away - see later.
// ETODO: kept enum for now, to check later

struct adf4382_chip_info {
	const char		*name;
	u64			vco_max;
	u64			vco_min;
	u8			clkout_div_reg_val_max;
};

enum adf4382_chip_id {
	ADF4382,
	ADF4382A,
	ADF4368,
};

struct adf4382_state {
	const struct adf4382_chip_info *info;
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clkin;
	struct clk		*clkout;
	struct clk_hw		clk_hw;
	/* Protect against concurrent accesses to the device and data content */
// TODO:JONATHANC:
// Good to be more specific. I'm guessing there are read modify write sequences or
// things that must be read sequentially that are not protected by the regmap locking.
	struct mutex		lock;
	struct notifier_block	nb;
	unsigned int		ref_freq_hz;
	u8			cp_i;
	u8			opwr_a;
	u64			freq;
	bool			spi_3wire_en;
	bool			ref_doubler_en;
	bool			auto_align_en;
	u8			ref_div;
	u16			bleed_word;
	int			phase;
	bool			cmos_3v3;
	u16			synth_lock_timeout;
	u16			vco_alc_timeout;
	u16			vco_band_div;

};

#define to_adf4382_state(_hw) container_of(_hw, struct adf4382_state, clk_hw)

/* Charge pump current values expressed in uA */
static const int adf4382_ci_ua[] = {
	790, 990, 1190, 1380, 1590, 1980, 2390, 2790,
	3180, 3970, 4770, 5570,	6330, 7910, 9510, 11100
};

static const struct reg_sequence adf4382_reg_default[] = {
	{ 0x00a, 0xA5 }, { 0x200, 0x00 }, { 0x201, 0x00 }, { 0x202, 0x00 },
	{ 0x203, 0x00 }, { 0x203, 0x00 }, { 0x203, 0x00 }, { 0x100, 0x25 },
	{ 0x101, 0x3F }, { 0x102, 0x3F }, { 0x103, 0x3F }, { 0x104, 0x3F },
	{ 0x105, 0x3F }, { 0x106, 0x3F }, { 0x107, 0x3F }, { 0x108, 0x3F },
	{ 0x109, 0x25 }, { 0x10A, 0x25 }, { 0x10B, 0x3F }, { 0x10C, 0x3F },
	{ 0x10D, 0x3F }, { 0x10E, 0x3F }, { 0x10F, 0x3F }, { 0x110, 0x3F },
	{ 0x111, 0x3F }, { 0x054, 0x00 }, { 0x053, 0x45 }, { 0x052, 0x00 },
	{ 0x051, 0x00 }, { 0x050, 0x00 }, { 0x04f, 0x08 }, { 0x04e, 0x06 },
	{ 0x04d, 0x00 }, { 0x04c, 0x2B }, { 0x04b, 0x5D }, { 0x04a, 0x00 },
	{ 0x048, 0x00 }, { 0x047, 0x00 }, { 0x046, 0x00 }, { 0x045, 0x62 },
	{ 0x044, 0x3F }, { 0x043, 0xB8 }, { 0x042, 0x01 }, { 0x041, 0x00 },
	{ 0x040, 0x00 }, { 0x03f, 0x82 }, { 0x03e, 0x4E }, { 0x03c, 0x00 },
	{ 0x03b, 0x00 }, { 0x03a, 0xFA }, { 0x039, 0x00 }, { 0x038, 0x71 },
	{ 0x037, 0x82 }, { 0x036, 0xC0 }, { 0x035, 0x00 }, { 0x034, 0x36 },
	{ 0x033, 0x00 }, { 0x032, 0x40 }, { 0x031, 0x63 }, { 0x030, 0x0F },
	{ 0x02f, 0x3F }, { 0x02e, 0x00 }, { 0x02d, 0xF1 }, { 0x02c, 0x0E },
	{ 0x02b, 0x01 }, { 0x02a, 0x30 }, { 0x029, 0x09 }, { 0x028, 0x00 },
	{ 0x027, 0xF0 }, { 0x026, 0x00 }, { 0x025, 0x01 }, { 0x024, 0x01 },
	{ 0x023, 0x00 }, { 0x022, 0x00 }, { 0x021, 0x00 }, { 0x01e, 0x20 },
	{ 0x01d, 0x00 }, { 0x01c, 0x00 }, { 0x01b, 0x00 }, { 0x01a, 0x00 },
	{ 0x019, 0x00 }, { 0x018, 0x00 }, { 0x017, 0x00 }, { 0x016, 0x00 },
	{ 0x015, 0x06 }, { 0x014, 0x00 }, { 0x013, 0x00 }, { 0x012, 0x00 },
	{ 0x011, 0x00 }, { 0x010, 0x50 }
// REVIEW:JONATHANC: This is typically done like this
// Where possible build these up from appropriate defines of the fields.
// Will take more code, but give a ready way to see what default means and
// compare with functions that change it.
};

static const struct reg_sequence adf4368_reg_defaults[] = {

	{ 0x054, 0x00 },
	{ 0x053, 0x25 },
	{ 0x052, 0x00 },
	{ 0x051, 0x00 },
	{ 0x050, 0x00 },
	{ 0x04f, 0x00 },
	{ 0x04e, 0x10 },
	{ 0x04d, 0x00 },
	{ 0x04c, 0x2B },
	{ 0x04b, 0x5D },
	{ 0x04a, 0x00 },
	{ 0x048, 0x00 },
	{ 0x047, 0x00 },
	{ 0x046, 0x00 },
	{ 0x045, 0x08 },
	{ 0x044, 0x18 },
	{ 0x043, 0x09 },
	{ 0x042, 0x09 },
	{ 0x041, 0x00 },
	{ 0x040, 0x00 },
	{ 0x03f, 0x83 },
	{ 0x03e, 0x26 },
	{ 0x03d, 0xC0 },
	{ 0x03c, 0x00 },
	{ 0x03b, 0x8C },
	{ 0x03a, 0x00 },
	{ 0x039, 0xB0 },
	{ 0x038, 0x00 },
	{ 0x037, 0x3E },
	{ 0x036, 0xD6 },
	{ 0x035, 0x04 },
	{ 0x034, 0x99 },
	{ 0x033, 0x32 },
	{ 0x032, 0xD3 },
	{ 0x031, 0x69 },
	{ 0x030, 0x1B },
	{ 0x02f, 0xA7 },
	{ 0x02e, 0x00 },
	{ 0x02d, 0xF1 },
	{ 0x02c, 0x4C },
	{ 0x02b, 0x03 },
	{ 0x02a, 0x10 },
	{ 0x029, 0xDD },
	{ 0x028, 0x20 },
	{ 0x027, 0x28 },
	{ 0x026, 0x80 },
	{ 0x025, 0x00 },
	{ 0x024, 0x00 },
	{ 0x023, 0x80 },
	{ 0x022, 0x00 },
	{ 0x021, 0x09 },
	{ 0x020, 0xC1 },
	{ 0x01f, 0x1E },
	{ 0x01e, 0x28 },
	{ 0x01d, 0xDD },
	{ 0x01c, 0xFF },
	{ 0x01b, 0xFF },
	{ 0x01a, 0xFF },
	{ 0x019, 0x55 },
	{ 0x018, 0x55 },
	{ 0x017, 0x55 },
	{ 0x016, 0x00 },
	{ 0x015, 0x00 },
	{ 0x014, 0x43 },
	{ 0x013, 0x55 },
	{ 0x012, 0x55 },
	{ 0x011, 0x00 },
	{ 0x010, 0x2B },
};

static const struct adf4382_chip_info adf4382_chip_tbl = {
	.name = "adf4382",
	.vco_max = ADF4382_VCO_FREQ_MAX,
	.vco_min = ADF4382_VCO_FREQ_MIN,
	.clkout_div_reg_val_max = ADF4382_CLKOUT_DIV_REG_VAL_MAX
};

static const struct adf4382_chip_info adf4382a_chip_tbl = {
	.name = "adf4382a",
	.vco_max = ADF4382A_VCO_FREQ_MAX,
	.vco_min = ADF4382A_VCO_FREQ_MIN,
	.clkout_div_reg_val_max = ADF4382A_CLKOUT_DIV_REG_VAL_MAX,
};

static const struct adf4382_chip_info adf4368_chip_tbl = {
	.name = "adf4368",
	.vco_max = ADF4368_VCO_FREQ_MAX,
	.vco_min = ADF4368_VCO_FREQ_MIN,
	.clkout_div_reg_val_max = ADF4368_CLKOUT_DIV_REG_VAL_MAX,
};

static const struct regmap_config adf4382_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int adf4382_pfd_compute(struct adf4382_state *st, u64 *pfd_freq_hz)
{
	u64 tmp;

	tmp = DIV_ROUND_CLOSEST(st->ref_freq_hz, st->ref_div);
	if (st->ref_doubler_en)
		tmp *= 2;

	if (tmp < ADF4382_PFD_FREQ_MIN || tmp > ADF4382_PFD_FREQ_MAX)
		return -EINVAL;

	*pfd_freq_hz = tmp;

	return 0;
}

static int adf4382_frac2_compute(struct adf4382_state *st, u64 res,
				 unsigned int pfd_freq_hz, u32 *frac2_word,
				 u32 *mod2_word)
{
	u32 channel_spacing;
	u8 en_phase_resync;
	u32 chsp_freq;
	u32 mod2_tmp;
	u32 mod2_max;
	u32 mod2_wd;
	u32 gcd_var;

	channel_spacing = 1;
	mod2_wd = 1;

	en_phase_resync = regmap_test_bits(st->regmap, 0x1E,
					   ADF4382_EN_PHASE_RESYNC_MSK);
	if (en_phase_resync < 0)
		return en_phase_resync;

	if (en_phase_resync)
		mod2_max = ADF4382_PHASE_RESYNC_MOD2WORD_MAX;
	else
		mod2_max = ADF4382_MOD2WORD_MAX;

	do {
		chsp_freq = channel_spacing * ADF4382_MOD1WORD;
		gcd_var = gcd(chsp_freq, pfd_freq_hz);
		mod2_tmp = DIV_ROUND_UP(pfd_freq_hz, gcd_var);

		if (mod2_tmp > mod2_max) {
			channel_spacing *= 5;
		} else {
			mod2_wd = mod2_tmp;
			break;
		}

	} while (channel_spacing < ADF4382_CHANNEL_SPACING_MAX);

	if (!en_phase_resync)
		mod2_wd *= DIV_ROUND_DOWN_ULL(mod2_max, mod2_wd);

	*frac2_word = DIV_ROUND_CLOSEST_ULL(res * mod2_wd, pfd_freq_hz);
	*mod2_word = mod2_wd;

	return 0;
}

static int adf4382_pll_fract_n_compute(struct adf4382_state *st, unsigned int pfd_freq_hz,
				       u16 *n_int, u32 *frac1_word, u32 *frac2_word,
				       u32 *mod2_word)
{
	u64 rem;
	u64 res;

	*n_int = div64_u64_rem(st->freq, pfd_freq_hz, &rem);

	res = rem * ADF4382_MOD1WORD;
	*frac1_word = (u32)div64_u64_rem(res, pfd_freq_hz, &rem);

	*frac2_word = 0;
	*mod2_word = 0;

	if (pfd_freq_hz > ADF4382_PFD_FREQ_FRAC_MAX) {
		dev_warn(&st->spi->dev, "PFD frequency exceeds 250MHz.");
		dev_warn(&st->spi->dev, "Only integer mode available.");
	}

	if (rem > 0)
		return adf4382_frac2_compute(st, rem, pfd_freq_hz, frac2_word,
					     mod2_word);

	return 0;
}

static int adf4382_optimize_bleed_word(struct adf4382_state *st, u16  n_int)
{
	u32 coarse_bleed, fine_bleed;
	u64 pfd_freq;
	u64 t_bleed, bleed_i;
	bool bleed_pol;
	u64 rem;
	u8 val;
	int ret;

	ret = adf4382_pfd_compute(st, &pfd_freq);
	if (pfd_freq >= 120000000UL) {
		/* 4.2GHz > RFout */
		if (st->freq > 4200000000UL) {
			t_bleed = 390;
			bleed_pol = false;
		}
		/* 4.2GHz < RFout < 3GHz */
		else if (st->freq > 3000000000UL) {
			t_bleed = 900;
			bleed_pol = false;
		}
		/* 3GHz < RFout < 1.8GHz */
		else if (st->freq > 1800000000UL) {
			t_bleed = 1200;
			bleed_pol = false;
		}
		/* st->freq < 1.8 GHz */
		else {
			t_bleed = 1400;
			bleed_pol = false;
		}
	} else {

		if (st->freq > 4200000000UL) {
			t_bleed = 390;
			bleed_pol = false;
		}
		/* 4.2GHz < RFout < 3GHz */
		else if (st->freq > 3000000000UL) {
			t_bleed = n_int < ADF4368_BLEED_N_INT_TH ?
				  900 : 1200;
			bleed_pol = n_int < ADF4368_BLEED_N_INT_TH ?
				    false : true;
		}
		/* 3GHz < RFout < 1.8GHz */
		else if (st->freq > 1800000000UL) {
			t_bleed = 1200;
			bleed_pol = n_int < ADF4368_BLEED_N_INT_TH ?
				    true : false;
		}
		/* 1.8GHz < RFout < 1.2GHz */
		else if (st->freq > 1200000000UL) {
			t_bleed = 1400;
			bleed_pol = n_int < ADF4368_BLEED_N_INT_TH ?
				    true : false;
		}
		/* st->freq < 1.2 GHz */
		else {
			t_bleed = n_int < ADF4368_BLEED_N_INT_TH ?
				  2000 : 1400;
			bleed_pol = n_int < ADF4368_BLEED_N_INT_TH ?
				    true : false;
		}

	}

	// Calculating bleed current
	bleed_i = t_bleed * adf4382_ci_ua[st->cp_i];
	bleed_i = DIV_ROUND_UP_ULL(bleed_i, PSEC_PER_SEC);
	coarse_bleed = div64_u64_rem(bleed_i, ADF4368_COARSE_BLEED_CNST, &rem);

	fine_bleed = rem * ADF4368_FINE_BLEED_CNST2;
	fine_bleed = DIV_ROUND_UP(fine_bleed, ADF4368_FINE_BLEED_CNST1);

	st->bleed_word = (coarse_bleed << 9) | fine_bleed;

	ret = regmap_update_bits(st->regmap, 0x1F, ADF4382_BLEED_POL_MSK,
				 FIELD_PREP(ADF4382_BLEED_POL_MSK, bleed_pol));
	if (ret)
		return ret;
	
	val = st->bleed_word & ADF4382_BLEED_I_LSB_MSK;
	ret = regmap_write(st->regmap, 0x1D, val);
	if (ret)
		return ret;
	
	val = (st->bleed_word >> 8) & ADF4382_BLEED_I_MSB_MSK;
	return regmap_write(st->regmap, 0x1E, val);
}

static int adf4368_set_freq(struct adf4382_state *st)
{
	//implement same as no-os to check if PLL locks like this
	u8 clkout_div, dclk_div1, int_mode, en_bleed, ldwin_pw, div1, var;
	u32 frac2_word = 0, frac1_word = 0, mod2_word = 0;
	u8 clkout_div_max = st->info->clkout_div_reg_val_max;
	u64 pfd_freq_hz, tmp, vco;
	u32 read_val;
	u16 n_int;
	u64 rem = 0, res = 0;
	int ret;
	
	ret = regmap_update_bits(st->regmap, 0x20,
				 ADF4382_EN_RDBLR_MSK | ADF4382_R_DIV_MSK,
				 FIELD_PREP(ADF4382_EN_RDBLR_MSK,
				       	    st->ref_doubler_en) |
			   	 FIELD_PREP(ADF4382_R_DIV_MSK, st->ref_div));
	if (ret)
		return ret;
	
	for(clkout_div = 0; clkout_div <= clkout_div_max; clkout_div++) {
		tmp = (1 << clkout_div) * st->freq;
		if (tmp < st->info->vco_min || tmp > st->info->vco_max)
			continue;
		vco = tmp;
		break;
	}

	if(!vco) {
		dev_err(&st->spi->dev, "Output frequency is out of range.\n");
		return -EINVAL;
	}

	ret = adf4382_pfd_compute(st, &pfd_freq_hz);
	if (ret) {
		dev_err(&st->spi->dev, "PFD frequency is out of range.\n");
		return ret;
	}

	ret = regmap_update_bits(st->regmap, 0x1F, ADF4382_CP_I_MSK,
				 FIELD_PREP(ADF4382_CP_I_MSK, st->cp_i));
	if (ret)
		return ret;
	
	/* Calculate n_int */
	n_int = div64_u64_rem(st->freq, pfd_freq_hz, &rem);

	/* Calculate frac1_word */
	if (rem) {
		res = rem * ADF4382_MOD1WORD;
		frac1_word = (u32) div64_u64_rem(res, pfd_freq_hz, &rem);

		/* Calculate frac2_word and mod2_word */
		if (rem > 0) {
			ret = adf4382_frac2_compute(st, rem, pfd_freq_hz, &frac2_word,
						    &mod2_word);
			if (ret)
				return ret;
		}
	}

	if (frac1_word || frac2_word) {
		if (n_int < ADF4368_FRAC_N_INT_MIN) {
			dev_info(&st->spi->dev, "err frac n int min\n");
			return -EINVAL;
		}
		
		int_mode = 0;
		en_bleed = 1;

		if(pfd_freq_hz <= (40 * HZ_PER_MHZ))
			ldwin_pw = 7;
		else if(pfd_freq_hz <= (50 * HZ_PER_MHZ))
			ldwin_pw = 6;
		else if(pfd_freq_hz <= (100 * HZ_PER_MHZ))
			ldwin_pw = 5;
		else if(pfd_freq_hz <= (200 * HZ_PER_MHZ))
			ldwin_pw = 4;
		else if(pfd_freq_hz <= (250 * HZ_PER_MHZ)) {
			if(st->freq >= (5000U * HZ_PER_MHZ) &&
			   st->freq < (6400U * HZ_PER_MHZ))
				ldwin_pw = 3;
			else
				ldwin_pw = 2;
		}
		ret = adf4382_optimize_bleed_word(st, n_int);
		if (ret)
			return ret;
		
	}
	else {
		if (n_int < ADF4368_INT_N_INT_MIN)
			return -EINVAL;
		int_mode = 1;
		en_bleed = 0;
		st->bleed_word = 0;
		ldwin_pw = 0;
	}

	dev_info(&st->spi->dev, "FOR ADF4368 FUNC VCO=%llu PFD=%llu RFout_div=%u N=%u FRAC1=%u FRAC2=%u MOD2=%u\n",
		 vco, pfd_freq_hz, 1 << clkout_div, n_int, frac1_word, frac2_word, mod2_word);
	
	if (frac2_word) {
		ret = regmap_update_bits(st->regmap, 0x28, ADF4382_VAR_MOD_EN_MSK,
					 ADF4382_VAR_MOD_EN_MSK);
		if (ret)
			return ret;
	}
	else {
		ret = regmap_update_bits(st->regmap, 0x28, ADF4382_VAR_MOD_EN_MSK, 0);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, 0x11, ADF4368_INT_MODE_MSK,
				 FIELD_PREP(ADF4368_INT_MODE_MSK, int_mode));
	if (ret)
		return ret;
	
	ret = regmap_update_bits(st->regmap, 0x1F, ADF4382_EN_BLEED_MSK,
				 FIELD_PREP(ADF4382_EN_BLEED_MSK, en_bleed));
	if (ret)
		return ret;
	
	put_unaligned_be24(mod2_word, &mod2_word);
	ret = regmap_bulk_write(st->regmap, 0x1C, &mod2_word, 3);
	if (ret)
		return ret;
	
	put_unaligned_be24(frac2_word, &frac2_word);
	ret = regmap_bulk_write(st->regmap, 0x19, &frac2_word, 3);
	if (ret)
		return ret;
	
	ret = regmap_update_bits(st->regmap, 0x15, ADF4382_FRAC1WORD_MSB,
				FIELD_GET(ADF4382_FRAC1WORD_MS_BIT_MSK, frac1_word));
	if (ret)
		return ret;

	put_unaligned_be24(frac1_word, &frac1_word);
	ret = regmap_bulk_write(st->regmap, 0x14, &frac1_word, 3);
	if (ret)
		return ret;
	

	dclk_div1 = 2;
	div1 = 8;
	if (pfd_freq_hz <= ADF4382_DCLK_DIV1_0_MAX) {
		dclk_div1 = 0;
		div1 = 1;
	} else if (pfd_freq_hz <= ADF4382_DCLK_DIV1_1_MAX) {
		dclk_div1 = 1;
		div1 = 2;
	}

	ret = regmap_update_bits(st->regmap, 0x4E, ADF4368_DCLK_DIV1_MSK,
				 FIELD_PREP(ADF4368_DCLK_DIV1_MSK, dclk_div1));
	if (ret)
		return ret;
	
	ret = regmap_update_bits(st->regmap, 0x35, ADF4368_DCLK_MODE_MSK, 0xFF);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, 0x35, ADF4368_EN_ADC_CLK_MSK, 0xFF);
	if (ret)
		return ret;
	
	/* Calculate ADC CLK DIV */
	tmp = DIV_ROUND_UP(div_u64(pfd_freq_hz, div1 * 400000) - 2, 4);
	tmp = clamp_t(u64, tmp, 0U, 255U);
	ret = regmap_write(st->regmap, 0x3E, FIELD_PREP(ADF4382_ADC_CLK_DIV_MSK, tmp));
	if (ret)
		return ret;
	
	ret = regmap_update_bits(st->regmap, 0x2C, ADF4382_LD_COUNT_OPWR_MSK,
				 FIELD_PREP(ADF4382_LD_COUNT_OPWR_MSK, st->opwr_a));
	if (ret)
		return ret;
	
	ret =  regmap_update_bits(st->regmap, 0x2C, ADF4382_LDWIN_PW_MSK,
				  FIELD_PREP(ADF4382_LDWIN_PW_MSK, ldwin_pw));
	if (ret)
		return ret;
	
	ret = regmap_update_bits(st->regmap, 0x11, ADF4368_CLKOUT_DIV_MSK,
				 FIELD_PREP(ADF4368_CLKOUT_DIV_MSK, clkout_div));
	if(ret)
		return ret;

	var = (n_int >> 8) & ADF4382_N_INT_MSB_MSK;
	ret = regmap_update_bits(st->regmap, 0x11, ADF4382_N_INT_MSB_MSK, var);
	if(ret)
		return ret;
	
	var = n_int & ADF4382_N_INT_LSB_MSK;
	ret = regmap_update_bits(st->regmap, 0x10, ADF4382_N_INT_LSB_MSK, var);
	if (ret)
		return ret;
	
	mdelay(10);

	ret = regmap_read(st->regmap, 0x58, &read_val);
	if (ret)
		return ret;

	if (!FIELD_GET(ADF4382_PLL_LOCK_MSK, read_val)) {
		dev_err(&st->spi->dev, "PLL is not locked.\n");
		return -EINVAL;
	}

	return 0;
}

static int _adf4382_set_freq(struct adf4382_state *st)
{
	u8 clkout_div, dclk_div1, int_mode, en_bleed, ldwin_pw, div1, var;
	u32 frac2_word = 0, frac1_word = 0, mod2_word = 0;
	u8 clkout_div_max = st->info->clkout_div_reg_val_max;
	u64 pfd_freq_hz, tmp, vco;
	u32 read_val;
	u16 n_int;
	int ret;

	if (strcmp(st->info->name, "adf4368") == 0)
		return adf4368_set_freq(st);

	ret = adf4382_pfd_compute(st, &pfd_freq_hz);
	if (ret) {
		dev_err(&st->spi->dev, "PFD frequency is out of range.\n");
		return ret;
	}

	for (clkout_div = 0; clkout_div <= clkout_div_max + 1; clkout_div++) {
		tmp =  (1 << clkout_div) * st->freq;
		if (tmp < st->info->vco_min || tmp > st->info->vco_max)
			continue;

		vco = tmp;
		break;
	}

	if (clkout_div > clkout_div_max) {
		dev_err(&st->spi->dev, "Output frequency is out of range.\n");
		return -EINVAL;
	}

	ret = adf4382_pll_fract_n_compute(st, pfd_freq_hz, &n_int, &frac1_word,
					  &frac2_word, &mod2_word);
	if (ret)
		return ret;

	if (frac1_word || frac2_word) {
		int_mode = 0;
		en_bleed = 1;

		if (pfd_freq_hz <= (40 * HZ_PER_MHZ)) {
			ldwin_pw = 7;
		} else if (pfd_freq_hz <= (50 * HZ_PER_MHZ)) {
			ldwin_pw = 6;
		} else if (pfd_freq_hz <= (100 * HZ_PER_MHZ)) {
			ldwin_pw = 5;
		} else if (pfd_freq_hz <= (200 * HZ_PER_MHZ)) {
			ldwin_pw = 4;
		} else if (pfd_freq_hz <= (250 * HZ_PER_MHZ)) {
			if (st->freq >= (5000U * HZ_PER_MHZ) &&
			    st->freq < (6400U * HZ_PER_MHZ)) {
				ldwin_pw = 3;
			} else {
				ldwin_pw = 2;
			}
		}
		ret = adf4382_optimize_bleed_word(st, n_int);
		if (ret)
			return ret;
	} else {
		int_mode = 1;
		en_bleed = 0;

		tmp = DIV_ROUND_UP_ULL(pfd_freq_hz, MICRO);
		tmp *= adf4382_ci_ua[st->cp_i];
		tmp = DIV_ROUND_UP_ULL(st->bleed_word, tmp);
		if(strcmp(st->info->name, "adf4368") == 0) {
			ldwin_pw = 0;
			st->bleed_word = 0;
		}
		else {
			if (tmp <= 85)
				ldwin_pw = 0;
			else
				ldwin_pw = 1;
		}
	}

	dev_info(&st->spi->dev,
		"VCO=%llu PFD=%llu RFout_div=%u N=%u FRAC1=%u FRAC2=%u MOD2=%u\n",
		vco, pfd_freq_hz, 1 << clkout_div, n_int,
		frac1_word, frac2_word, mod2_word);

	ret = regmap_update_bits(st->regmap, 0x28, ADF4382_VAR_MOD_EN_MSK,
				 frac2_word != 0 ? ADF4382_VAR_MOD_EN_MSK : 0);
	if (ret)
		return ret;

	dclk_div1 = 2;
	div1 = 8;
	if (pfd_freq_hz <= ADF4382_DCLK_DIV1_0_MAX) {
		dclk_div1 = 0;
		div1 = 1;
	} else if (pfd_freq_hz <= ADF4382_DCLK_DIV1_1_MAX) {
		dclk_div1 = 1;
		div1 = 2;
	}

	if(strcmp(st->info->name, "adf4368") == 0) {
		dev_info(&st->spi->dev, "ADF4368\n");
		ret = regmap_update_bits(st->regmap, 0x11, ADF4368_INT_MODE_MSK,
					FIELD_PREP(ADF4368_INT_MODE_MSK, int_mode));
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, 0x1F, ADF4382_EN_BLEED_MSK,
					FIELD_PREP(ADF4382_EN_BLEED_MSK, en_bleed));
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, 0x4e, ADF4368_DCLK_DIV1_MSK,
					 FIELD_PREP(ADF4368_DCLK_DIV1_MSK,
					 	    dclk_div1));
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x35, ADF4368_DCLK_MODE_MSK,
					 0xff);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x31, ADF4368_EN_ADC_CLK_MSK,
					 0xff);
		if (ret)
			return ret;
		ret = regmap_write(st->regmap, 0x1D,
				FIELD_GET(ADF4382_BLEED_I_LSB_MSK, st->bleed_word));
		if (ret)
			return ret;

		var = (st->bleed_word >> 8) & ADF4382_BLEED_I_MSB_MSK;
		ret = regmap_update_bits(st->regmap, 0x1E, ADF4382_BLEED_I_MSB_MSK, var);
		if (ret)
			return ret;

		put_unaligned_be24(mod2_word, &mod2_word);
		ret = regmap_bulk_write(st->regmap, 0x1C, &mod2_word, 3);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x15, ADF4382_FRAC1WORD_MSB,
					FIELD_GET(ADF4382_FRAC1WORD_MS_BIT_MSK, frac1_word));
		if (ret)
			return ret;

		put_unaligned_be24(frac1_word, &frac1_word);
		ret = regmap_bulk_write(st->regmap, 0x14, &frac1_word, 3);
		if (ret)
			return ret;

		put_unaligned_be24(frac2_word, &frac2_word);
		ret = regmap_bulk_write(st->regmap, 0x19, &frac2_word, 3);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, 0x37, st->vco_band_div);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, 0x38, FIELD_PREP(ADF4368_SYNTH_LSB_MSK,
								st->synth_lock_timeout));
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x39, ADF4368_SYNTH_MSB_MSK,
					 FIELD_PREP(ADF4368_SYNTH_MSB_MSK,
					 	    st->synth_lock_timeout >> 8));
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, 0x3A, FIELD_PREP(ADF4368_VCO_ALC_LSB_MSK,
								st->vco_alc_timeout));
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x3B, ADF4368_VCO_ALC_MSB_MSK,
					 FIELD_PREP(ADF4368_VCO_ALC_MSB_MSK,
					 	    st->synth_lock_timeout >> 8));
		if (ret)
			return ret;
		
		var = DIV_ROUND_UP(div_u64(pfd_freq_hz, div1 * 400000) - 2, 4);
		var = clamp_t(u8, var, 0U, 255U);
		ret = regmap_write(st->regmap, 0x3e, var);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x2c, ADF4382_LD_COUNT_OPWR_MSK,
					 10);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x2c, ADF4382_LDWIN_PW_MSK,
					 FIELD_PREP(ADF4382_LDWIN_PW_MSK,
					 	    ldwin_pw));
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x11, ADF4368_CLKOUT_DIV_MSK,
					 FIELD_PREP(ADF4368_CLKOUT_DIV_MSK,
					 	    clkout_div));
		if (ret)
			return ret;

		var = (n_int >> 8) & ADF4382_N_INT_MSB_MSK;
		ret = regmap_update_bits(st->regmap, 0x11, ADF4382_N_INT_MSB_MSK,
					 var);
		if (ret)
			return ret;

		ret  = regmap_write(st->regmap, 0x10,
				FIELD_PREP(ADF4382_N_INT_LSB_MSK, n_int));
		if (ret)
			return ret;

		ret = regmap_read_poll_timeout(st->regmap, 0x58, read_val,
				       !(read_val & (ADF4368_FSM_BUSY_MSK)), 200, 200 * 100);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x2d, ADF4382_EN_DNCLK_MSK,
					 0);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x2d, ADF4382_EN_DRCLK_MSK,
					 0);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x31, ADF4382_EN_ADC_CLK_MSK,
					 0);
		if (ret)
			return ret;
	} else {
		ret = regmap_update_bits(st->regmap, 0x15, ADF4382_INT_MODE_MSK,
					FIELD_PREP(ADF4382_INT_MODE_MSK, int_mode));
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, 0x24, ADF4382_DCLK_DIV1_MSK,
					FIELD_PREP(ADF4382_DCLK_DIV1_MSK, dclk_div1));
		if (ret)
			return ret;

		ret = regmap_set_bits(st->regmap, 0x31, ADF4382_DCLK_MODE_MSK);
		if (ret)
			return ret;

		ret = regmap_set_bits(st->regmap, 0x31, ADF4382_CAL_CT_SEL_MSK);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, 0x38, ADF4382_VCO_CAL_VTUNE);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, 0x3a, ADF4382_VCO_CAL_ALC);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, 0x37, ADF4382_VCO_CAL_CNT);
		if (ret)
			return ret;

		var = DIV_ROUND_UP(div_u64(pfd_freq_hz, div1 * 400000) - 2, 4);
		var = clamp_t(u8, var, 0U, 255U);
		ret = regmap_write(st->regmap, 0x3e, var);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x2c, ADF4382_LD_COUNT_OPWR_MSK,
					10);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x2c, ADF4382_LDWIN_PW_MSK,
					FIELD_PREP(ADF4382_LDWIN_PW_MSK, ldwin_pw));
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, 0x11, ADF4382_CLKOUT_DIV_MSK,
					FIELD_PREP(ADF4382_CLKOUT_DIV_MSK, clkout_div));
		if (ret)
			return ret;

		var = (n_int >> 8) & ADF4382_N_INT_MSB_MSK;
		ret = regmap_update_bits(st->regmap, 0x11, ADF4382_N_INT_MSB_MSK, var);
		if (ret)
			return ret;

		ret  = regmap_write(st->regmap, 0x10,
				FIELD_PREP(ADF4382_N_INT_LSB_MSK, n_int));
		if (ret)
			return ret;
	}

	mdelay(9);

	ret = regmap_read(st->regmap, 0x58, &read_val);
	if (ret)
		return ret;

	if (!FIELD_GET(ADF4382_PLL_LOCK_MSK, read_val)) {
		dev_err(&st->spi->dev, "PLL is not locked.\n");
		return -EINVAL;
	}

	return 0;
}

static int adf4382_set_freq(struct adf4382_state *st)
{
	guard(mutex)(&st->lock);
	return _adf4382_set_freq(st);
}

static int adf4382_get_freq(struct adf4382_state *st, u64 *val)
{
	u32 frac1 = 0, frac2 = 0, mod2 = 0;
	unsigned int tmp;
	u64 freq, pfd;
	u16 n;
	int ret;

	ret = adf4382_pfd_compute(st, &pfd);
	if (ret)
		return ret;

	ret = regmap_bulk_read(st->regmap, 0x11, &tmp, 2);
	if (ret)
		return ret;

	tmp = get_unaligned_be16(&tmp);
	n = FIELD_GET(ADF4382_N_INT_MSK, tmp);

	ret = regmap_bulk_read(st->regmap, 0x15, &tmp, 4);
	if (ret)
		return ret;

	tmp = get_unaligned_be32(&tmp);
	frac1 = FIELD_GET(ADF4382_FRAC1WORD_MSK, tmp);

	ret = regmap_bulk_read(st->regmap, 0x19, &tmp, 3);
	if (ret)
		return ret;

	tmp = get_unaligned_be24(&tmp);
	frac2 = FIELD_GET(ADF4382_FRAC2WORD_MSK, tmp);

	ret = regmap_bulk_read(st->regmap, 0x1c, &tmp, 3);
	if (ret)
		return ret;

	tmp = get_unaligned_be24(&tmp);
	mod2 = FIELD_GET(ADF4382_MOD2WORD_MSK, tmp);

	if (mod2 == 0)
		mod2 = 1;

	freq = frac2 * pfd;
	freq = div_u64(freq, mod2);
	freq = freq + (frac1 * pfd);
	freq = div_u64(freq, ADF4382_MOD1WORD);
	freq = freq + (n * pfd);

	*val = freq;
	return 0;
}

static int adf4382_set_phase_adjust(struct adf4382_state *st, u32 phase_fs)
{
	u8 phase_reg_value;
	u64 phase_deg_fs;
	u64 phase_deg_ns;
	u64 phase_deg_ms;
	u64 phase_bleed;
	u64 phase_value;
	u64 pfd_freq_hz;
	u64 phase_ci;
	int ret;

	ret = regmap_set_bits(st->regmap, 0x1E, ADF4382_EN_PHASE_RESYNC_MSK);
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, 0x1F, ADF4382_EN_BLEED_MSK);
	if (ret)
		return ret;

	ret = regmap_clear_bits(st->regmap, 0x32, ADF4382_DEL_MODE_MSK);
	if (ret)
		return ret;

	ret = adf4382_pfd_compute(st, &pfd_freq_hz);
	if (ret) {
		dev_err(&st->spi->dev, "PFD frequency is out of range.\n");
		return ret;
	}

	// Determine the phase adjustment in degrees relative the output freq.
	phase_deg_fs = phase_fs * st->freq;
	phase_deg_ns = div_u64(phase_deg_fs, (FEMTO / NANO));
	phase_deg_ns = PERIOD_IN_DEG * phase_deg_ns;
// TODO:JONATHANC:
// That PERIOD_IN_DEG rather implies that some of these were not phase_deg.
// I don't really understand the steps here. Maybe add a comment with the maths would help.
	phase_deg_ms = div_u64(phase_deg_ns, (NANO / MILLI));

	if (phase_deg_ms > PERIOD_IN_DEG_MS) {
		dev_err(&st->spi->dev, "Phase adjustment is out of range.\n");
		return -EINVAL;
	}

	/*
	 * The charge pump current will also need to be taken in to account
	 * as well as the Bleed constant
	 */
	phase_ci = phase_deg_ms * adf4382_ci_ua[st->cp_i];
	phase_bleed = phase_ci * ADF4382_PHASE_BLEED_CNST_MUL;
	phase_bleed = div_u64(phase_bleed, ADF4382_PHASE_BLEED_CNST_DIV);

	// Computation of the register value for the phase adjust
	phase_value = phase_bleed * pfd_freq_hz;
	phase_value = div64_u64(phase_value, st->freq);
	phase_value = div_u64(phase_value, PERIOD_IN_DEG);
	phase_value = DIV_ROUND_CLOSEST_ULL(phase_value, MILLI);

	// Mask the value to 8 bits
// TODO:JONATHANC:
// All comments in IIO use /* */
	phase_reg_value = phase_value & 0xff;

	ret = regmap_write(st->regmap, 0x33, phase_reg_value);
	if (ret)
		return ret;

	if (st->auto_align_en)
		return regmap_set_bits(st->regmap, 0x32,
				       ADF4382_EN_AUTO_ALIGN_MSK);

	ret = regmap_clear_bits(st->regmap, 0x32, ADF4382_EN_AUTO_ALIGN_MSK);
	if (ret)
		return ret;

	return regmap_set_bits(st->regmap, 0x34, ADF4382_PHASE_ADJ_MSK);
}

static int adf4382_get_phase_adjust(struct adf4382_state *st, u32 *val)
{
	unsigned int tmp;
	u8 phase_reg_value;
	u64 phase_value;
	u64 pfd_freq_hz;
	int ret;

	ret = regmap_read(st->regmap, 0x33, &tmp);
	if (ret)
		return ret;

	phase_reg_value = tmp;

	ret = adf4382_pfd_compute(st, &pfd_freq_hz);
	if (ret) {
		dev_err(&st->spi->dev, "PFD frequency is out of range.\n");
		return ret;
	}

	phase_value = phase_reg_value * PERIOD_IN_DEG;
// TODO:JONATHANC:
// Clear the intermediates are not phase_value.  Can we figure a naming scheme out
// that makes the intermediate steps more obvious.
	phase_value = phase_value * st->freq;
	phase_value = div64_u64(phase_value, pfd_freq_hz);

	phase_value = phase_value * ADF4382_PHASE_BLEED_CNST_DIV;
	phase_value = phase_value * (NANO / MILLI);
// TODO:JONATHANC:
// phase_value *= ADF4382_PHASE_BLEED_CNST_DIV * MS_PER_NS;
	phase_value = div_u64(phase_value, ADF4382_PHASE_BLEED_CNST_MUL);
	phase_value = phase_value * MILLI;
// TODO:JONATHANC:
// phase_value *= MILLI;
	phase_value = div_u64(phase_value, adf4382_ci_ua[st->cp_i]);

	phase_value = phase_value * (FEMTO / NANO);
// TODO:JONATHANC:
// *= here as well.
	phase_value = div_u64(phase_value, PERIOD_IN_DEG);
	phase_value = div64_u64(phase_value, st->freq);

	*val = (u32)phase_value;

	return 0;
}

static int adf4382_set_phase_pol(struct adf4382_state *st, bool ph_pol)
{
	return regmap_update_bits(st->regmap, 0x32, ADF4382_PHASE_ADJ_POL_MSK,
				  FIELD_PREP(ADF4382_PHASE_ADJ_POL_MSK, ph_pol));
}

static int adf4382_get_phase_pol(struct adf4382_state *st, bool *ph_pol)
{
	unsigned int tmp;
	int ret;

	ret =  regmap_read(st->regmap, 0x32, &tmp);
	if (ret)
		return ret;

	*ph_pol = FIELD_GET(ADF4382_PHASE_ADJ_POL_MSK, tmp);

	return 0;
}

static int adf4382_set_out_power(struct adf4382_state *st, int ch, int pwr)
{
	if (pwr > ADF4382_OUT_PWR_MAX)
		pwr = ADF4382_OUT_PWR_MAX;

	if (!ch)
		return regmap_update_bits(st->regmap, 0x29, ADF4382_CLK1_OPWR_MSK,
					  FIELD_PREP(ADF4382_CLK1_OPWR_MSK, pwr));
	else
		return regmap_update_bits(st->regmap, 0x29, ADF4382_CLK2_OPWR_MSK,
					   FIELD_PREP(ADF4382_CLK2_OPWR_MSK, pwr));

};

static int adf4382_get_out_power(struct adf4382_state *st, int ch, int *pwr)
{
	unsigned int tmp;
	int ret;

	ret = regmap_read(st->regmap, 0x29, &tmp);
	if (ret)
		return ret;

	if (!ch)
		*pwr = FIELD_GET(ADF4382_CLK1_OPWR_MSK, tmp);
	else
		*pwr = FIELD_GET(ADF4382_CLK2_OPWR_MSK, tmp);

	return 0;
}

static int adf4382_set_en_chan(struct adf4382_state *st, int ch, int en)
{
	if (!ch) {
		return regmap_update_bits(st->regmap, 0x2B,
					  ADF4382_PD_CLKOUT1_MSK,
					  FIELD_PREP(ADF4382_PD_CLKOUT1_MSK, !en));
	}

	return regmap_update_bits(st->regmap, 0x2B, ADF4382_PD_CLKOUT2_MSK,
				  FIELD_PREP(ADF4382_PD_CLKOUT2_MSK, !en));
}

static int adf4382_get_en_chan(struct adf4382_state *st, int ch, int *en)
{
	int disable;

	if (!ch)
		disable = regmap_test_bits(st->regmap, 0x2B,
					   ADF4382_PD_CLKOUT1_MSK);
	else
		disable = regmap_test_bits(st->regmap, 0x2B,
					   ADF4382_PD_CLKOUT2_MSK);
	if (disable < 0)
		return disable;

	*en = !disable;

	return 0;
}

static ssize_t adf4382_write(struct iio_dev *indio_dev, uintptr_t private,
			     const struct iio_chan_spec *chan, const char *buf,
			     size_t len)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	unsigned long long val;
	int ret;

	ret = kstrtoull(buf, 10, &val);
	if (ret)
		return ret;

	switch ((u32)private) {
	case ADF4382_FREQ:
		st->freq = val;
		ret = adf4382_set_freq(st);
// TODO:JONATHANC:
// As below, overlap in code doesn't warrant having a single write function.
		break;
	case ADF4382_EN_AUTO_ALIGN:
		st->auto_align_en = !!val;
		ret = adf4382_set_phase_adjust(st, 0);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t adf4382_read(struct iio_dev *indio_dev, uintptr_t private,
			    const struct iio_chan_spec *chan, char *buf)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	unsigned int val = 0;
	u64 val_u64 = 0;
	int ret;

	switch ((u32)private) {
	case ADF4382_FREQ:
		ret = adf4382_get_freq(st, &val_u64);
// TODO:JONATHANC:
// There is almost no overlap, so just have separate callbacks.
		if (ret)
			return ret;
		return sysfs_emit(buf, "%llu\n", val_u64);
	case ADF4382_EN_AUTO_ALIGN:
		ret = regmap_read(st->regmap, 0x32, &val);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n",
				  FIELD_GET(ADF4382_EN_AUTO_ALIGN_MSK, val));
	default:
		return -EINVAL;
	}
}

#define _ADF4382_EXT_INFO(_name, _shared, _ident) { \
		.name = _name, \
		.read = adf4382_read, \
		.write = adf4382_write, \
		.private = _ident, \
		.shared = _shared, \
}

// TODO:JONATHANC:
// For two entries?  Macro is just making it harder to read so put the struct
// initializer inline without the macro.

static const struct iio_chan_spec_ext_info adf4382_ext_info[] = {
	/*
	 * Usually we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz.
	 */
	_ADF4382_EXT_INFO("frequency", IIO_SHARED_BY_TYPE, ADF4382_FREQ),
// TODO:JONATHANC:
// We have IIO_VAL_INT64 for that. It's a bit fiddly but should work here.
	_ADF4382_EXT_INFO("en_auto_align", IIO_SHARED_BY_TYPE, ADF4382_EN_AUTO_ALIGN),
// TODO:JONATHANC:
// What is this?  Needs documentation in Documentation/ABI/testing/sysfs-bus-iio-adf4382
	{ }
};

static int adf4382_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	bool pol;
	u32 tmp;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = adf4382_get_out_power(st, chan->channel, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_ENABLE:
		ret = adf4382_get_en_chan(st, chan->channel, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		ret = adf4382_get_phase_adjust(st, &tmp);
		if (ret)
			return ret;
		*val = tmp;

		ret = adf4382_get_phase_pol(st, &pol);
		if (ret)
			return ret;

		if (pol)
			*val *= -1;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int adf4382_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return adf4382_set_out_power(st, chan->channel, val);
	case IIO_CHAN_INFO_ENABLE:
		return adf4382_set_en_chan(st, chan->channel, val);
	case IIO_CHAN_INFO_PHASE:
		st->phase = val;

		ret = adf4382_set_phase_pol(st, val < 0);
		if (ret)
			return ret;

		return adf4382_set_phase_adjust(st, abs(val));
	default:
		return -EINVAL;
	}
}

static int adf4382_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int write_val, unsigned int *read_val)
{
	struct adf4382_state *st = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(st->regmap, reg, read_val);

	return regmap_write(st->regmap, reg, write_val);
}

static const struct iio_info adf4382_info = {
	.read_raw = &adf4382_read_raw,
	.write_raw = &adf4382_write_raw,
	.debugfs_reg_access = &adf4382_reg_access,
};

static const struct iio_chan_spec adf4382_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PHASE),
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = adf4382_ext_info,
	},
	{
		.type = IIO_ALTVOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_PHASE),
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.ext_info = adf4382_ext_info,
	},
};

static int adf4382_show_del_cnt_raw(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct adf4382_state *st = iio_priv(indio_dev);
	u16 tmp;
	int ret;

	ret = regmap_bulk_read(st->regmap, 0x65, &tmp, 2);
	if (ret)
		return ret;

	*val = get_unaligned_be16(&tmp);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adf4382_del_cnt_raw_fops,
			 adf4382_show_del_cnt_raw, NULL, "%llu\n");

static int adf4382_show_bleed_pol(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct adf4382_state *st = iio_priv(indio_dev);
	u16 del_cnt;
	u16 tmp;
	int ret;

	ret = regmap_bulk_read(st->regmap, 0x65, &tmp, 2);
	if (ret)
		return ret;
	del_cnt = get_unaligned_be16(&tmp);

	*val = FIELD_GET(ADF4382_DEL_CNT_BLEED_POL_MSK, del_cnt);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adf4382_bleed_pol_fops,
			 adf4382_show_bleed_pol, NULL, "%llu\n");

static int adf4382_show_fine_current(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct adf4382_state *st = iio_priv(indio_dev);
	u16 del_cnt;
	u16 tmp;
	int ret;

	ret = regmap_bulk_read(st->regmap, 0x65, &tmp, 2);
	if (ret)
		return ret;
	del_cnt = get_unaligned_be16(&tmp);

	*val = FIELD_GET(ADF4382_DEL_CNT_FINE_MSK, del_cnt);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adf4382_fine_current_fops,
			 adf4382_show_fine_current, NULL, "%llu\n");

static int adf4382_show_coarse_current(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct adf4382_state *st = iio_priv(indio_dev);
	u16 del_cnt;
	u16 tmp;
	int ret;

	ret = regmap_bulk_read(st->regmap, 0x65, &tmp, 2);
	if (ret)
		return ret;
	del_cnt = get_unaligned_be16(&tmp);

	*val = FIELD_GET(ADF4382_DEL_CNT_COARSE_MSK, del_cnt);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(adf4382_coarse_current_fops,
			 adf4382_show_coarse_current, NULL, "%llu\n");

static void adf4382_debugfs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);

	debugfs_create_file_unsafe("del_cnt_raw", 0400, d,
				   indio_dev, &adf4382_del_cnt_raw_fops);

	debugfs_create_file_unsafe("bleed_pol", 0400, d,
				   indio_dev, &adf4382_bleed_pol_fops);

	debugfs_create_file_unsafe("fine_current", 0400, d,
				   indio_dev, &adf4382_fine_current_fops);

	debugfs_create_file_unsafe("coarse_current", 0400, d,
				   indio_dev, &adf4382_coarse_current_fops);
}

static int adf4382_parse_device(struct adf4382_state *st)
{
	u8 tmp;

	st->freq = ADF4382_RFOUT_DEFAULT;
	device_property_read_u64(&st->spi->dev, "adi,power-up-frequency",
				 &st->freq);

	st->bleed_word = 0;
	device_property_read_u16(&st->spi->dev, "adi,bleed-word",
				 &st->bleed_word);

	st->cp_i = ADF4382_CP_I_DEFAULT;
	device_property_read_u8(&st->spi->dev, "adi,charge-pump-microamp",
				&st->cp_i);

	tmp = ADF4382_OPOWER_DEFAULT;
	device_property_read_u8(&st->spi->dev, "adi,output-power-value", &tmp);
	st->opwr_a = clamp_t(u8, tmp, 0, 15);

	st->ref_div = ADF4382_REF_DIV_DEFAULT;
	device_property_read_u8(&st->spi->dev, "adi,ref-divider", &st->ref_div);
// REVIEW:JONATHANC:
// read a u8 if you want an u8 and define the DT binding as such.
// However, I raised that I'm not yet convinced this should be in that binding.

	st->spi_3wire_en = device_property_read_bool(&st->spi->dev,
						     "adi,spi-3wire-enable");
	st->ref_doubler_en = device_property_read_bool(&st->spi->dev,
						       "adi,ref-doubler-enable");
	st->cmos_3v3 = device_property_read_bool(&st->spi->dev, "adi,cmos-3v3");

	st->clkin = devm_clk_get_enabled(&st->spi->dev, "ref_clk");

	return PTR_ERR_OR_ZERO(st->clkin);
}

static int adf4382_scratchpad_check(struct adf4382_state *st)
{
	unsigned int val;
	int ret;

	ret = regmap_write(st->regmap, 0x0A, ADF4382_SCRATCHPAD_VAL);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, 0x0A, &val);
	if (ret)
		return ret;

	if (val != ADF4382_SCRATCHPAD_VAL)
		return dev_err_probe(&st->spi->dev, -EINVAL,
				      "Scratch pad test failed please check SPI connection");

	return 0;
}

static int adf4382_compute_timeout(struct adf4382_state *st, u64 freq)
{
	u64 pfd_freq_hz;
	unsigned int f_div_rclk;
	u8 dclk_div1;
	u8 div1;
	u8 dclk_mode = 1;
	int ret;

	ret = adf4382_pfd_compute(st, &pfd_freq_hz);
	if (ret)
		return ret;


	//TODO: COMPUTE DCLK_DIV1, DCLK_MODE, F_div_RCLK
	dclk_div1 = 2;
	div1 = 8;
	if (pfd_freq_hz <= ADF4382_DCLK_DIV1_0_MAX) {
		dclk_div1 = 0;
		div1 = 2;
	} else if (pfd_freq_hz <= ADF4382_DCLK_DIV1_1_MAX) {
		dclk_div1 = 1;
		div1 = 4;
	}
	f_div_rclk = (unsigned int) pfd_freq_hz / div1;

	st->synth_lock_timeout = DIV_ROUND_UP(f_div_rclk, 50000);
	st->vco_alc_timeout = DIV_ROUND_UP(f_div_rclk, 20000);
	st->vco_band_div = DIV_ROUND_UP(f_div_rclk, 150000 * 16 * (1 << dclk_mode));

	return 0;
}

static int adf4382_init(struct adf4382_state *st)
{
	int ret;
	bool en = true;

	ret = regmap_write(st->regmap, 0x00, ADF4382_RESET_CMD);
	if (ret)
		return ret;

	if (st->spi->mode & SPI_3WIRE || st->spi_3wire_en)
		en = false;

	ret = regmap_write(st->regmap, 0x00,
			   FIELD_PREP(ADF4382_SDO_ACT_MSK, en) |
// REVIEW:JONATHANC:
// Field names should provide an indication of which register they are in.
// Here, given they registers aren't named, perhaps
// ADF4382_REG0000_SDO_ACT_MSK etc

			   FIELD_PREP(ADF4382_SDO_ACT_R_MSK, en));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, 0x3D,
			   FIELD_PREP(ADF4382_CMOS_OV_MSK, st->cmos_3v3));
	if (ret)
		return ret;

	ret = adf4382_scratchpad_check(st);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, 0x20,
			   ADF4382_EN_AUTOCAL_MSK |
			   FIELD_PREP(ADF4382_EN_RDBLR_MSK, st->ref_doubler_en) |
			   FIELD_PREP(ADF4382_R_DIV_MSK, st->ref_div));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, 0x1f, st->cp_i);
	if (ret)
		return ret;

	if (strcmp(st->info->name, "adf4368") == 0)
		ret = regmap_multi_reg_write(st->regmap, adf4368_reg_defaults,
				     	     ARRAY_SIZE(adf4368_reg_defaults));
	else
		ret = regmap_multi_reg_write(st->regmap, adf4382_reg_default,
					     ARRAY_SIZE(adf4382_reg_default));
	if (ret)
		return ret;

	st->ref_freq_hz = clk_get_rate(st->clkin);

	ret = adf4382_set_out_power(st, 0, st->opwr_a);
	if (ret)
		return ret;

	ret = adf4382_set_out_power(st, 1, st->opwr_a);
	if (ret)
		return ret;

	if(strcmp(st->info->name, "adf4368") == 0) {
		ret = adf4382_compute_timeout(st, st->freq);		
		if (ret)
			return ret;
	}

	return _adf4382_set_freq(st);
}

static int adf4382_freq_change(struct notifier_block *nb, unsigned long action,
			       void *data)
{
	struct adf4382_state *st = container_of(nb, struct adf4382_state, nb);

	if (action == POST_RATE_CHANGE) {
		guard(mutex)(&st->lock);
		return notifier_from_errno(adf4382_init(st));
	}

	return NOTIFY_OK;
}

static int adf4382_clock_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct adf4382_state *st = to_adf4382_state(hw);

	st->ref_freq_hz = parent_rate;
	st->freq = rate * ADF4382_CLK_SCALE;

	return adf4382_set_freq(st);
}

static void adf4382_clk_notifier_unreg(void *data)
{
	struct adf4382_state *st = data;

	clk_notifier_unregister(st->clkin, &st->nb);
}

static unsigned long adf4382_clock_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct adf4382_state *st = to_adf4382_state(hw);
	u64 freq = 0;
	int ret;

	ret = adf4382_get_freq(st, &freq);
	if (ret)
		dev_err(&st->spi->dev, "Failed to get frequency\n");

	return DIV_ROUND_CLOSEST_ULL(freq, ADF4382_CLK_SCALE);
}

static int adf4382_clock_enable(struct clk_hw *hw)
{
	struct adf4382_state *st = to_adf4382_state(hw);

	return regmap_clear_bits(st->regmap, 0x2B,
				 ADF4382_PD_CLKOUT1_MSK |
				 ADF4382_PD_CLKOUT2_MSK);
}

static void adf4382_clock_disable(struct clk_hw *hw)
{
	struct adf4382_state *st = to_adf4382_state(hw);

	regmap_set_bits(st->regmap, 0x2B,
			ADF4382_PD_CLKOUT1_MSK |
			ADF4382_PD_CLKOUT2_MSK);
}

static long adf4382_clock_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct adf4382_state *st = to_adf4382_state(hw);
	u64 freq = rate * ADF4382_CLK_SCALE;
	u8 div_rate;
	u64 tmp;

	for (div_rate = 0; div_rate <= st->info->clkout_div_reg_val_max; div_rate++) {
		tmp = (1 << div_rate) * freq;
		if (tmp >= st->info->vco_min)
			break;
	}
	div_rate = clamp_t(u8, div_rate, 0U, st->info->clkout_div_reg_val_max);
	freq = clamp_t(u64, tmp, st->info->vco_min, st->info->vco_max);
	freq = div_u64(freq, 1 << div_rate);

	return DIV_ROUND_CLOSEST_ULL(freq, ADF4382_CLK_SCALE);
}

static const struct clk_ops adf4382_clock_ops = {
	.set_rate = adf4382_clock_set_rate,
	.recalc_rate = adf4382_clock_recalc_rate,
	.round_rate = adf4382_clock_round_rate,
	.enable = adf4382_clock_enable,
	.disable = adf4382_clock_disable,
};

static int adf4382_setup_clk(struct adf4382_state *st)
{
	struct device *dev = &st->spi->dev;
	struct clk_init_data init;
	struct clk *clk;
	const char *parent_name;

	if (!device_property_present(dev, "#clock-cells"))
		return 0;

	if (device_property_read_string(dev, "clock-output-names", &init.name)) {
		init.name = devm_kasprintf(dev, GFP_KERNEL, "%s-clk",
					   fwnode_get_name(dev_fwnode(dev)));
		if (!init.name)
			return -ENOMEM;
	}

	parent_name = of_clk_get_parent_name(dev->of_node, 0);
	if (!parent_name)
		return -EINVAL;

	init.ops = &adf4382_clock_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	st->clk_hw.init = &init;
	clk = devm_clk_register(dev, &st->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	st->clkout = clk;

	return devm_of_clk_add_hw_provider(dev, of_clk_hw_simple_get, clk);
}

static int adf4382_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct adf4382_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adf4382_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st = iio_priv(indio_dev);

	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -ENODEV;

	indio_dev->name = st->info->name;
	indio_dev->info = &adf4382_info;

	st->regmap = regmap;
	st->spi = spi;

	mutex_init(&st->lock);
// FIXME:JONATHANC: not avilable in this version
// ret = devm_mutex_init(&st->lock)
// if (ret)
//	return ret;
// Only makes a difference when some debug features are turned on, but it's
// cheap to do so preferred in new code.

	ret = adf4382_parse_device(st);
	if (ret)
		return ret;

	st->nb.notifier_call = adf4382_freq_change;
	ret = clk_notifier_register(st->clkin, &st->nb);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4382_clk_notifier_unreg, st);
	if (ret)
		return ret;

	ret = adf4382_init(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "adf4382 init failed\n");

	ret = adf4382_setup_clk(st);
	if (ret)
		return ret;

	if (!st->clkout) {
// REVIEW:JONATHANC:
// If you have set clkout, does it actually make sense to register the iio device
// with no channels?  What is that bringing us?
		indio_dev->channels = adf4382_channels;
		indio_dev->num_channels = ARRAY_SIZE(adf4382_channels);
	}

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		adf4382_debugfs_init(indio_dev);

	return 0;
}

static const struct spi_device_id adf4382_id[] = {
	{ "adf4382",  (kernel_ulong_t)&adf4382_chip_tbl },
	{ "adf4382a", (kernel_ulong_t)&adf4382a_chip_tbl },
	{ "adf4368", (kernel_ulong_t)&adf4368_chip_tbl },
	{ }
};
MODULE_DEVICE_TABLE(spi, adf4382_id);

static const struct of_device_id adf4382_of_match[] = {
	{ .compatible = "adi,adf4382",  .data = &adf4382_chip_tbl },
	{ .compatible = "adi,adf4382a", .data = &adf4382a_chip_tbl },
	{ .compatible = "adi,adf4368", .data = &adf4368_chip_tbl },
	{ }
};
MODULE_DEVICE_TABLE(of, adf4382_of_match);

static struct spi_driver adf4382_driver = {
	.driver = {
		.name = "adf4382",
		.of_match_table = adf4382_of_match,
	},
	.probe = adf4382_probe,
	.id_table = adf4382_id,
};
module_spi_driver(adf4382_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF4382");
MODULE_LICENSE("GPL");
