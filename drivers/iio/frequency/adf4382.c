// SPDX-License-Identifier: GPL-2.0-only
/*
 * adf4382 driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/property.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/gcd.h>
#include <linux/math64.h>


/* ADF4382 REG0000 Map */
#define ADF4382_SOFT_RESET_R_MSK	BIT(7)
#define ADF4382_LSB_FIRST_R_MSK		BIT(6)
#define ADF4382_ADDRESS_ASC_R_MSK	BIT(5)
#define ADF4382_SDO_ACTIVE_R_MSK	BIT(4)
#define ADF4382_SDO_ACTIVE_MSK		BIT(3)
#define ADF4382_ADDRESS_ASC_MSK		BIT(2)
#define ADF4382_LSB_FIRST_MSK		BIT(1)
#define ADF4382_SOFT_RESET_MSK		BIT(0)


/* ADF4382_REG0 */
#define ADF4382_ADDR_ASC_MSK		BIT(2)
#define ADF4382_ADDR_ASC(x)		FIELD_PREP(ADF4382_ADDR_ASC_MSK, x)
#define ADF4382_ADDR_ASC_R_MSK		BIT(5)
#define ADF4382_ADDR_ASC_R(x)		FIELD_PREP(ADF4382_ADDR_ASC_R_MSK, x)
#define ADF4382_SDO_ACT_MSK		BIT(3)
#define ADF4382_SDO_ACT(x)		FIELD_PREP(ADF4382_SDO_ACT_MSK, x)
#define ADF4382_SDO_ACT_R_MSK		BIT(4)
#define ADF4382_SDO_ACT_R(x)		FIELD_PREP(ADF4382_SDO_ACT_R_MSK, x)
#define ADF4382_RESET_CMD		0x81

/* ADF4382 REG0000 Bit Definition */
#define ADF4382_SDO_ACTIVE_SPI_3W	0x0
#define ADF4382_SDO_ACTIVE_SPI_4W	0x1

#define ADF4382_ADDR_ASC_AUTO_DECR	0x0
#define ADF4382_ADDR_ASC_AUTO_INCR	0x1

#define ADF4382_LSB_FIRST_MSB		0x0
#define ADF4382_LSB_FIRST_LSB		0x1

#define ADF4382_SOFT_RESET_N_OP		0x0
#define ADF4382_SOFT_RESET_EN		0x1

/* ADF4382 REG0001 Map */
#define ADF4382_SINGLE_INSTR_MSK	BIT(7)
#define ADF4382_MASTER_RB_CTRL_MSK	BIT(5)

/* ADF4382 REG0001 Bit Definition */
#define ADF4382_SPI_STREAM_EN		0x0
#define ADF4382_SPI_STREAM_DIS		0x1

#define ADF4382_RB_SLAVE_REG		0x0
#define ADF4382_RB_MASTER_REG		0x1

/* ADF4382 REG0003 Bit Definition */
#define ADF4382_CHIP_TYPE		0x06

/* ADF4382 REG0004 Bit Definition */
#define ADF4382_PRODUCT_ID_LSB		0x0005

/* ADF4382 REG0005 Bit Definition */
#define ADF4382_PRODUCT_ID_MSB		0x0005

/* ADF4382 REG000A Map */
#define ADF4382_SCRATCHPAD_MSK		GENMASK(7, 0)

/* ADF4382 REG000C Bit Definition */
#define ADF4382_VENDOR_ID_LSB		0x56

/* ADF4382 REG000D Bit Definition */
#define ADF4382_VENDOR_ID_MSB		0x04

/* ADF4382 REG000F Bit Definition */
#define ADF4382_M_S_TRANSF_BIT_MSK	BIT(0)

/* ADF4382 REG0010 Map*/
#define ADF4382_N_INT_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG0011 Map*/
#define ADF4382_CLKOUT_DIV_MSK		GENMASK(7, 5)
#define ADF4382_INV_CLK_OUT_MSK		BIT(4)
#define ADF4382_N_INT_MSB_MSK		GENMASK(3, 0)

/* ADF4382 REG0012 Map */
#define ADF4382_FRAC1WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0013 Map */
#define ADF4382_FRAC1WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG0014 Map */
#define ADF4382_FRAC1WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0015 Map */
#define ADF4382_M_VCO_BAND_LSB_MSK	BIT(7)
#define ADF4382_M_VCO_CORE_MSK		BIT(6)
#define ADF4382_BIAS_DEC_MODE_MSK	GENMASK(5, 3)
#define ADF4382_INT_MODE_MSK		BIT(2)
#define ADF4382_PFD_POL_MSK		BIT(1)
#define ADF4382_FRAC1WORD_BIT25_MSK	BIT(0)

/* ADF4382 REG0016 Map */
#define ADF4382_M_VCO_BAND_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0017 Map */
#define ADF4382_FRAC2WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0018 Map */
#define ADF4382_FRAC2WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG0019 Map */
#define ADF4382_FRAC2WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG001A Map */
#define ADF4382_MOD2WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG001B Map */
#define ADF4382_MOD2WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG001C Map */
#define ADF4382_MOD2WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG001D Map */
#define ADF4382_BLEED_I_LSB_MSK		GENMASK(7, 0)

/* ADF4382 REG001E Map */
#define ADF4382_EN_PHASE_RESYNC_MSK	BIT(7)
#define ADF4382_EN_REF_RST_MSK		BIT(6)
#define ADF4382_TIMED_SYNC_MSK		BIT(5)
#define ADF4382_BLEED_I_MSB_MSK		GENMASK(4, 0)

/* ADF4382 REG001F Map */
#define ADF4382_SW_SYNC_MSK		BIT(7)
#define ADF4382_SPARE_1F_MSK		BIT(6)
#define ADF4382_BLEED_POL_MSK		BIT(5)
#define ADF4382_EN_BLEED_MSK		BIT(4)
#define ADF4382_CP_I_MSK		GENMASK(3, 0)

/* ADF4382 REG0020 Map */
#define ADF4382_EN_AUTOCAL_MSK		BIT(7)
#define ADF4382_EN_RDBLR_MSK		BIT(6)
#define ADF4382_R_DIV_MSK		GENMASK(5, 0)

/* ADF4382 REG0021 Map */
#define ADF4382_PHASE_WORD_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0022 Map */
#define ADF4382_PHASE_WORD_MID_MSK	GENMASK(7, 0)

/* ADF4382 REG0023 Map */
#define ADF4382_PHASE_WORD_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0024 Map */
#define ADF4382_SPARE_24_MSK		GENMASK(7, 5)
#define ADF4382_DCLK_DIV_SEL_MSK	BIT(4)
#define ADF4382_DNCLK_DIV1_MSK		GENMASK(3, 2)
#define ADF4382_DCLK_DIV1_MSK		GENMASK(1, 0)

/* ADF4382 REG0025 Map */
#define ADF4382_RESYNC_WAIT_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0026 Map */
#define ADF4382_RESYNC_WAIT_MSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0027 Map */
#define ADF4382_CAL_BLEED_FINE_MIN_MSK	GENMASK(7, 4)
#define ADF4382_BLEED_ADJ_SCALE_MSK	GENMASK(3, 0)

/* ADF4382 REG0028 Map */
#define ADF4382_PH_RESYNC_RB_SEL_MSK	BIT(7)
#define ADF4382_LSB_P1_MSK		BIT(6)
#define ADF4382_VAR_MOD_EN_MSK		BIT(5)
#define ADF4382_DITHER1_SCALE_MSK	GENMASK(4, 2)
#define ADF4382_EN_DITHER2_MSK		BIT(1)
#define ADF4382_EN_DITHER1_MSK		BIT(0)

/* ADF4382 REG0029 Map */
#define ADF4382_CLK2_OPWR_MSK		GENMASK(7, 4)
#define ADF4382_CLK1_OPWR		GENMASK(3, 0)

/* ADF4382 REG002A Map */
#define ADF4382_FN_DBL_MSK		BIT(7)
#define ADF4382_PD_NDIV_TL_MSK	BIT(6)
#define ADF4382_CLKOUT_BST_MSK		BIT(5)
#define ADF4382_PD_SYNC_MSK		BIT(4)
#define ADF4382_PD_CLK_MSK		BIT(3)
#define ADF4382_PD_RDET_MSK		BIT(2)
#define ADF4382_PD_ADC_MSK		BIT(1)
#define ADF4382_PD_CALGEN_MSK		BIT(0)

/* ADF4382 REG002B Map */
#define ADF4382_PD_ALL_MSK		BIT(7)
#define ADF4382_PD_RDIV_TL_MSK		BIT(6)
#define ADF4382_PD_NDIV_MSK		BIT(5)
#define ADF4382_PD_VCO_MSK		BIT(4)
#define ADF4382_PD_LD_MSK		BIT(3)
#define ADF4382_PD_PFDCP_MSK		BIT(2)
#define ADF4382_PD_CLKOUT1_MSK		BIT(1)
#define ADF4382_PD_CLKOUT2_MSK		BIT(0)

/* ADF4382 REG002C Map */
#define ADF4382_LDWIN_PW_MSK		GENMASK(7, 4)
#define ADF4382_LD_COUNT_OPWR_MSK	GENMASK(3, 0)

/* ADF4382 REG002D Map */
#define ADF4382_EN_DNCLK_MSK		BIT(7)
#define ADF4382_EN_DRCLK_MSK		BIT(6)
#define ADF4382_EN_LOL_MSK		BIT(5)
#define ADF4382_EN_LDWIN_MSK		BIT(4)
#define ADF4382_PDET_POL_MSK		BIT(3)
#define ADF4382_RST_LD_MSK		BIT(2)
#define ADF4382_LD_O_CTRL_MSK		GENMASK(1, 0)

/* ADF4382 REG002E Map */
#define ADF4382_MUXOUT_MSK		GENMASK(7, 4)
#define ADF4382_ABPW_WD_MSK		BIT(3)
#define ADF4382_EN_CPTEST_MSK		BIT(2)
#define ADF4382_CP_DOWN_MSK		BIT(1)
#define ADF4382_CP_UP_MSK		BIT(0)

/* ADF4382 REG002F Map*/
#define ADF4382_BST_REF_MSK		BIT(7)
#define ADF4382_FILT_REF_MSK		BIT(6)
#define ADF4382_RDBLR_DC_MSK		GENMASK(5, 0)

/* ADF4382 REG0030 Map */
#define ADF4382_MUTE_NCLK_MSK		BIT(7)
#define ADF4382_MUTE_RCLK_MSK		BIT(6)
#define ADF4382_REF_SEL_MSK		BIT(5)
#define ADF4382_INV_RDBLR_MSK		BIT(4)
#define ADF4382_RDBLR_DEL_SEL_MSK	GENMASK(3, 0)

/* ADF4382 REG0031 Map */
#define ADF4382_SYNC_DEL_MSK		GENMASK(7, 5)
#define ADF4382_RST_SYS_MSK		BIT(4)
#define ADF4382_EN_ADC_CLK_MSK		BIT(3)
#define ADF4382_EN_VCAL_MSK		BIT(2)
#define ADF4382_CAL_CT_SEL_MSK		BIT(1)
#define ADF4382_DCLK_MODE_MSK		BIT(0)

/* ADF4382 REG0032 Map */
#define ADF4382_SPARE_32_MSK		BIT(7)
#define ADF4382_BLEED_ADJ_CAL_MSK	BIT(6)
#define ADF4382_DEL_MODE_MSK		BIT(5)
#define ADF4382_EN_AUTO_ALIGN_MSK	BIT(4)
#define ADF4382_PHASE_ADJ_POL_MSK	BIT(3)
#define ADF4382_EFM3_MODE_MSK		GENMASK(2, 0)

/* ADF4382 REG0033 Map */
#define ADF4382_PHASE_ADJUST_MSK	GENMASK(7, 0)

/* ADF4382 REG0034 Map */
#define ADF4382_PHASE_ADJ_MSK		BIT(7)
#define ADF4382_DRCLK_DEL_MSK		GENMASK(6, 4)
#define ADF4382_DNCLK_DEL_MSK		GENMASK(3, 1)
#define ADF4382_RST_CNTR_MSK		BIT(0)

/* ADF4382 REG0035 Map */
#define ADF4382_SPARE_35_MSK		GENMASK(7, 6)
#define ADF4382_M_VCO_BIAS_MSK		GENMASK(5, 0)

/* ADF4382 REG0036 Map */
#define ADF4382_CLKODIV_DB_MSK		BIT(7)
#define ADF4382_DCLK_DIV_DB_MSK		BIT(6)
#define ADF4382_SPARE_36_MSK		GENMASK(5, 2)
#define ADF4382_EN_LUT_GEN_MSK		BIT(1)
#define ADF4382_EN_LUT_CAL_MSK		BIT(0)

/* ADF4382 REG0037 Map */
#define ADF4382_CAL_COUNT_TO_MSK	GENMASK(7, 0)

/* ADF4382 REG0038 Map */
#define ADF4382_CAL_VTUNE_TO_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG0039 Map */
#define ADF4382_O_VCO_DB_MSK		BIT(7)
#define ADF4382_CAL_VTUNE_TO_MSB_MSK	GENMASK(6, 0)

/* ADF4382 REG003A Map */
#define ADF4382_CAL_VCO_TO_LSB_MSK	GENMASK(7, 0)

/* ADF4382 REG003B Map */
#define ADF4382_DEL_CTRL_DB_MSK		BIT(7)
#define ADF4382_CAL_VCO_TO_MSB_MSK	GENMASK(6, 0)

/* ADF4382 REG003C Map */
#define ADF4382_CNTR_DIV_WORD_MSK	GENMASK(7, 0)

/* ADF4382 REG003D Map */
#define ADF4382_SPARE_3D_MSK		BIT(7)
#define ADF4382_SYNC_SP_DB_MSK		BIT(6)
#define ADF4382_CMOS_OV_MSK		BIT(5)
#define ADF4382_READ_MODE_MSK		BIT(4)
#define ADF4382_CNTR_DIV_WORD_MSB_MSK	GENMASK(3, 0)

/* ADF4382 REG003E Map */
#define ADF4382_ADC_CLK_DIV_MSK		GENMASK(7, 0)

/* ADF4382 REG003F Map */
#define ADF4382_EN_ADC_CNV_MSK		BIT(7)
#define ADF4382_EN_ADC_VTEST_MSK	BIT(6)
#define ADF4382_ADC_VTEST_SEL_MSK	BIT(5)
#define ADF4382_ADC_MUX_SEL_MSK		BIT(4)
#define ADF4382_ADC_F_CONV_MSK		BIT(3)
#define ADF4382_ADC_C_CONV_MSK		BIT(2)
#define ADF4382_EN_ADC_MSK		BIT(1)
#define ADF4382_SPARE_3F_MSK		BIT(0)

/* ADF4382 REG0040 Map */
#define ADF4382_EXT_DIV_DEC_SEL_MSK	BIT(7)
#define ADF4382_ADC_CLK_TEST_SEL_MSK	BIT(6)
#define ADF4382_MUTE_CLKOUT2_MSK	GENMASK(5, 3)
#define ADF4382_MUTE_CLKOUT1_MSK	GENMASK(2, 0)

/* ADF4382 REG0041 Map */
#define ADF4382_EXT_DIV_MSK		GENMASK(7, 5)
#define ADF4382_EN_VCO_CAP_TEST_MSK	BIT(4)
#define ADF4382_EN_CALGEN_CAP_TEST_MSK	BIT(3)
#define ADF4382_EN_CP_CAP_TEST_MSK	BIT(2)
#define ADF4382_CAP_TEST_STATE_MSK	BIT(1)
#define ADF4382_TRANS_LOOP_SEL_MSK	BIT(0)

/* ADF4382 REG0042 Map */
#define ADF4382_NDIV_PWRUP_TIMEOUT_MSK	GENMASK(7, 0)

/* ADF4382 REG0043 Map */
#define ADF4382_CAL_BLEED_FINE_MAX_MSK	GENMASK(7, 0)

/* ADF4382 REG0044 Map */
#define ADF4382_VCAL_ZERO_MSK		BIT(7)
#define ADF4382_VPTAT_CALGEN_MSK	GENMASK(6, 0)

/* ADF4382 REG0045 Map */
#define ADF4382_SPARE_45_MSK		BIT(7)
#define ADF4382_VCTAT_CALGEN_MSK	GENMASK(6, 0)

/* ADF4382 REG0046 Map */
#define ADF4382_NVMDIN_MSK		GENMASK(7, 0)

/* ADF4382 REG0047 Map */
#define ADF4382_SPARE_47_MSK		BIT(7)
#define ADF4382_NVMADDR_MSK		GENMASK(6, 3)
#define ADF4382_NVMBIT_SEL		GENMASK(2, 0)

/* ADF4382 REG0048 Map */
#define ADF4382_TRIM_LATCH_MSK		BIT(7)
#define ADF4382_NVMTEST_MSK		BIT(6)
#define ADF4382_NVMPROG_MSK		BIT(5)
#define ADF4382_NVMRD_MSK		BIT(4)
#define ADF4382_NVMSTART_MSK		BIT(3)
#define ADF4382_NVMON_MSK		BIT(2)
#define ADF4382_MARGIN_MSK		GENMASK(1, 0)

/* ADF4382 REG0049 Map */
#define ADF4382_NVMDOUT_MSK		GENMASK(7, 0)

/* ADF4382 REG004A Map */
#define ADF4382_SCAN_MODE_CODE_MSK	GENMASK(7, 0)

/* ADF4382 REG004B Map */
#define ADF4382_TEMP_OFFSET_MSK		GENMASK(7, 0)

/* ADF4382 REG004C Map */
#define ADF4382_SPARE_4C_MSK		GENMASK(7, 6)
#define ADF4382_TEMP_SLOPE_MSK		GENMASK(5, 0)

/* ADF4382 REG004D Map */
#define ADF4382_VCO_FSM_TEST_MUX_MSK	GENMASK(7, 5)
#define ADF4382_SPARE_4D_MSK		GENMASK(4, 3)
#define ADF4382_O_VCO_BIAS_MSK		BIT(2)
#define ADF4382_O_VCO_BAND_MSK		BIT(1)
#define ADF4382_O_VCO_CORE_MSK		BIT(0)

/* ADF4382 REG004E Map */
#define ADF4382_EN_TWO_PASS_CALL_MSK	BIT(4)
#define ADF4382_TWO_PASS_BAND_START_MSK	GENMASK(3, 0)

/* ADF4382 REG004F Map */
#define ADF4382_LUT_SCALE_MSK		GENMASK(7, 0)

/* ADF4382 REG0050 Map */
#define ADF4382_SPARE0_MSK		GENMASK(7, 0)

/* ADF4382 REG0051 Map */
#define ADF4382_SPARE1_MSK		GENMASK(7, 0)

/* ADF4382 REG0052 Map */
#define ADF4382_SYNC_REF_SPARE_MSK	GENMASK(7, 4)
#define ADF4382_SYNC_MON_DEL_MSK	GENMASK(3, 0)

/* ADF4382 REG0053 Map */
#define ADF4382_PD_SYNC_MON_MSK		BIT(6)
#define ADF4382_SYNC_SEL_MSK		BIT(5)
#define ADF4382_RST_SYNC_MON_MSK	BIT(4)
#define ADF4382_SYNC_SH_DEL_MSK		GENMASK(3, 0)

/* ADF4382 REG0054 Map */
#define ADF4382_ADC_ST_CNV_MSK		BIT(0)

#define ADI_ADF4382_REF_MIN                   10000000U    // 10MHz
#define ADI_ADF4382_REF_MAX                   5000000000U  // 5GHz
#define ADI_ADF4382_VCO_FREQ_MIN              11000000000U // 11GHz
#define ADI_ADF4382_VCO_FREQ_MAX              22000000000U // 22GHz
#define ADI_ADF4382_RFOUT_MIN                 687500000U   // 687.50MHz
#define ADI_ADF4382_RFOUT_MAX                 (ADI_ADF4382_VCO_FREQ_MAX)
#define ADI_ADF4382_MOD1WORD                  0x2000000U   // 2^25
#define ADI_ADF4382_MOD2WORD_MAX              0xFFFFFFU    // 2^24 - 1
#define ADI_ADF4382_PHASE_RESYNC_MOD2WORD_MAX 0x1FFFFU     // 2^17 - 1
#define ADI_ADF4382_CHANNEL_SPACING_MAX       78125U
#define ADI_ADF4382_DCLK_DIV1_0_MAX           160000000U   // 160MHz
#define ADI_ADF4382_DCLK_DIV1_1_MAX           320000000U   // 320MHz
#define ADI_ADF4382_M_VCO_BIAS_POST_CAL_FIX	0x3F

#define ADF4382_CP_I_DEFAULT		   	15
#define ADF4382_REF_DIV_DEFAULT		   	1
#define ADF4382_RFOUT_DEFAULT		   	2305000000ULL

enum {
	ADF4382_FREQ,
};

struct adf4382_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	struct clk		*clkin;
	struct clk_hw		clk_hw;
	struct clock_scale 	scale;
	/* Protect against concurrent accesses to the device and data content */
	struct mutex		lock;
	struct notifier_block	nb;
	unsigned int		ref_freq_hz;
	u8			cp_i;
	u64			freq;
	bool			spi_3wire_en;
	bool			ref_doubler_en;
	u8			ref_div;
};

//TODO Rewrite using defines
static const struct reg_sequence adf4382_reg_default[] = {
	{ 0x000, 0x18 },
	{ 0x00a, 0xA5 },
	{ 0x200, 0x00 },
	{ 0x201, 0x00 },
	{ 0x202, 0x00 },
	{ 0x203, 0x00 },
	{ 0x203, 0x00 },
	{ 0x203, 0x00 },
	{ 0x100, 0x25 },
	{ 0x101, 0x3F },
	{ 0x102, 0x3F },
	{ 0x103, 0x3F },
	{ 0x104, 0x3F },
	{ 0x105, 0x3F },
	{ 0x106, 0x3F },
	{ 0x107, 0x3F },
	{ 0x108, 0x3F },
	{ 0x109, 0x25 },
	{ 0x10A, 0x25 },
	{ 0x10B, 0x3F },
	{ 0x10C, 0x3F },
	{ 0x10D, 0x3F },
	{ 0x10E, 0x3F },
	{ 0x10F, 0x3F },
	{ 0x110, 0x3F },
	{ 0x111, 0x3F },
	{ 0x054, 0x00 },
	{ 0x053, 0x45 },
	{ 0x052, 0x00 },
	{ 0x051, 0x00 },
	{ 0x050, 0x00 },
	{ 0x04f, 0x08 },
	{ 0x04e, 0x06 },
	{ 0x04d, 0x00 },
	{ 0x04c, 0x2B },
	{ 0x04b, 0x5D },
	{ 0x04a, 0x00 },
	{ 0x048, 0x00 },
	{ 0x047, 0x00 },
	{ 0x046, 0x00 },
	{ 0x045, 0x62 },
	{ 0x044, 0x3F },
	{ 0x043, 0xB8 },
	{ 0x042, 0x01 },
	{ 0x041, 0x00 },
	{ 0x040, 0x00 },
	{ 0x03f, 0x82 },
	{ 0x03e, 0x4E },
	{ 0x03d, 0x00 },
	{ 0x03c, 0x00 },
	{ 0x03b, 0x00 },
	{ 0x03a, 0xFA },
	{ 0x039, 0x00 },
	{ 0x038, 0x71 },
	{ 0x037, 0x82 },
	{ 0x036, 0xC0 },
	{ 0x035, 0x00 },
	{ 0x034, 0x36 },
	{ 0x033, 0x00 },
	{ 0x032, 0x40 },
	{ 0x031, 0x63 },
	{ 0x030, 0x0F },
	{ 0x02f, 0x3F },
	{ 0x02e, 0x00 },
	{ 0x02d, 0xF1 },
	{ 0x02c, 0x0E },
	{ 0x02b, 0x01 },
	{ 0x02a, 0x30 },
	{ 0x029, 0x09 },
	{ 0x028, 0x00 },
	{ 0x027, 0xF0 },
	{ 0x026, 0x00 },
	{ 0x025, 0x01 },
	{ 0x024, 0x01 },
	{ 0x023, 0x00 },
	{ 0x022, 0x00 },
	{ 0x021, 0x00 },
	{ 0x020, 0xC1 },
	{ 0x01f, 0x0F },
	{ 0x01e, 0x20 },
	{ 0x01d, 0x00 },
	{ 0x01c, 0x00 },
	{ 0x01b, 0x00 },
	{ 0x01a, 0x00 },
	{ 0x019, 0x00 },
	{ 0x018, 0x00 },
	{ 0x017, 0x00 },
	{ 0x016, 0x00 },
	{ 0x015, 0x06 },
	{ 0x014, 0x00 },
	{ 0x013, 0x00 },
	{ 0x012, 0x00 },
	{ 0x011, 0x00 },
	{ 0x010, 0x50 },
};

static const struct regmap_config adf4382_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
//	.max_register = 0x54,
};

static int adf4382_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int write_val,
			      unsigned int *read_val)
{
	struct adf4382_state *st = iio_priv(indio_dev);

	if (read_val)
		return regmap_read(st->regmap, reg, read_val);

	return regmap_write(st->regmap, reg, write_val);
}

static const struct iio_info adf4382_info = {
	.debugfs_reg_access = &adf4382_reg_access,
};

void adf4382_pfd_compute(struct adf4382_state *st, unsigned int *pfd_freq_hz)
{
	unsigned int tmp;

	tmp = DIV_ROUND_CLOSEST(st->ref_freq_hz, st->ref_div);
	if (st->ref_doubler_en)
		tmp *= 2;

	*pfd_freq_hz = tmp;
	
	return;
}

int adf4382_set_freq(struct adf4382_state *st)
{
	u32 channel_spacing = 1;
	unsigned int pfd_freq_hz;
	u32 tmp_r;
	u32 frac2_word = 0;
	u8 en_phase_resync;
	u32 mod2_word = 0;
	u8 dclk_div1 = 2;
	u64 residue;
	u32 frac1_word;
	u8 clkout_div;
	u32 mod2_max;
	u32 mod2_tmp;
	u8 div1 = 8;
	u16 n_int;
	u64 tmp;
	u64 vco;
	int ret;
	u8 var;
	u8 ldwin_pw;

	adf4382_pfd_compute(st, &pfd_freq_hz);

	for (clkout_div = 0; clkout_div < 4; clkout_div++)
	{
		tmp = (1 << clkout_div) * st->freq;
		if (tmp < ADI_ADF4382_VCO_FREQ_MIN || tmp > ADI_ADF4382_VCO_FREQ_MAX)
		{
			continue;
		}

		vco = tmp;
		break;
	}

	if (vco == 0) {
		dev_err(&st->spi->dev, "VCO is 0");
		return -EINVAL;
	}

	ret = regmap_update_bits(st->regmap, 0x11, ADF4382_CLKOUT_DIV_MSK,
				 FIELD_PREP(ADF4382_CLKOUT_DIV_MSK, clkout_div));
	if (ret)
		return ret;


	n_int = div_u64_rem(st->freq, pfd_freq_hz, &tmp_r);
	residue = (u64)tmp_r * ADI_ADF4382_MOD1WORD;
	frac1_word = (u32)div_u64_rem(residue, pfd_freq_hz, &tmp_r);
	residue = tmp_r;

	if (residue > 0)
	{
		mod2_word = 1;

		en_phase_resync = regmap_test_bits(st->regmap, 0x1E,
						   ADF4382_EN_PHASE_RESYNC_MSK);
		if (en_phase_resync)
			mod2_max = ADI_ADF4382_PHASE_RESYNC_MOD2WORD_MAX;
		else
			mod2_max = ADI_ADF4382_MOD2WORD_MAX;
		do
		{
			mod2_tmp = DIV_ROUND_UP(pfd_freq_hz,
						 gcd(channel_spacing *
						     ADI_ADF4382_MOD1WORD,
						     pfd_freq_hz));
			if (mod2_tmp > mod2_max)
			{
				channel_spacing *= 5;
			}
			else {
				mod2_word = mod2_tmp;
				break;
			}

		} while (channel_spacing < ADI_ADF4382_CHANNEL_SPACING_MAX);

	        if (!en_phase_resync)
	        {
	            mod2_word *= DIV_ROUND_DOWN_ULL(mod2_max, mod2_word);
	        }

	        frac2_word = DIV_ROUND_CLOSEST_ULL(residue * mod2_word, pfd_freq_hz);
	}


	dev_info(&st->spi->dev,"n_int %u frac1_word %u frac2_word %u mod1_word %u mod2_word %u\n",
		n_int, frac1_word, frac2_word, ADI_ADF4382_MOD1WORD, mod2_word);

	if (frac1_word == 0 && frac2_word == 0) {
		// uint16_t bleed_i;
		// uint8_t cp_i;
		// u8 tmpval;

		// regmap_read(st->regmap, 0x1f, tmpval);
		// cp_i = tmpval & ADF4382_CP_I_MSK;
		// regmap_read(st->regmap, 0x1d, tmpval);
		// bleed_i = tmpval;
		// regmap_read(st->regmap, 0x1e, tmpval);
		// bleed_i |= (tmpval & ADF4382_BLEED_I_MSB_MSK) << 8;

		ldwin_pw = 0;

		ret = regmap_update_bits(st->regmap, 0x15, ADF4382_INT_MODE_MSK,
					 0xff);
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, 0x1f, ADF4382_EN_BLEED_MSK,
					 0);
		if (ret)
			return ret;
	} else {
		if (pfd_freq_hz <= 40000000) {
			ldwin_pw = 7;
		} else if (pfd_freq_hz <= 50000000) {
			ldwin_pw = 6;
		} else if (pfd_freq_hz <= 100000000) {
			ldwin_pw = 5;
		} else if (pfd_freq_hz <= 200000000) {
			ldwin_pw = 4;
		} else if (pfd_freq_hz <= 250000000) {
			if (st->freq >= 5000000000 && st->freq< 6400000000) {
				ldwin_pw = 3;
			} else {
				ldwin_pw = 2;
			}
		}
		ret = regmap_update_bits(st->regmap, 0x15, ADF4382_INT_MODE_MSK,
					 0);
		if (ret)
			return ret;
		ret = regmap_update_bits(st->regmap, 0x1f, ADF4382_EN_BLEED_MSK,
					 0xff);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, 0x28, ADF4382_VAR_MOD_EN_MSK,
				 frac2_word != 0 ? 0xff : 0);
	if (ret)
		return ret;

	var = mod2_word & ADF4382_MOD2WORD_LSB_MSK;
	ret = regmap_write(st->regmap, 0x1A, var);
	if (ret)
		return ret;
	var = (mod2_word >> 8) & ADF4382_MOD2WORD_MID_MSK;
	ret = regmap_write(st->regmap, 0x1B, var);
	if (ret)
		return ret;
	var = (mod2_word >> 16) & ADF4382_MOD2WORD_MSB_MSK;
	ret = regmap_write(st->regmap, 0x1C, var);
	if (ret)
		return ret;

	var = frac1_word  & ADF4382_FRAC1WORD_LSB_MSK;
	ret = regmap_write(st->regmap, 0x12, var);
	if (ret)
		return ret;
	var = (frac1_word >> 8)  & ADF4382_FRAC1WORD_MID_MSK;
	ret = regmap_write(st->regmap, 0x13, var);
	if (ret)
		return ret;
	var =  (frac1_word >> 16)  & ADF4382_FRAC1WORD_MSB_MSK;
	ret = regmap_write(st->regmap, 0x14, var);
	if (ret)
		return ret;
	var =  (frac1_word >> 24)  & ADF4382_FRAC1WORD_BIT25_MSK;
	ret = regmap_update_bits(st->regmap, 0x15, ADF4382_FRAC1WORD_BIT25_MSK, var);
	if (ret)
		return ret;

	var = frac2_word  & ADF4382_FRAC2WORD_LSB_MSK;
	ret = regmap_write(st->regmap, 0x17, var);
	if (ret)
		return ret;
	var = (frac2_word >> 8)  & ADF4382_FRAC2WORD_MID_MSK;
	ret = regmap_write(st->regmap, 0x18, var);
	if (ret)
		return ret;
	var = (frac2_word >> 16) & ADF4382_FRAC2WORD_MSB_MSK;
	ret = regmap_write(st->regmap, 0x19, var);
	if (ret)
		return ret;

	if (pfd_freq_hz <= ADI_ADF4382_DCLK_DIV1_0_MAX) {
		dclk_div1 = 0;
		div1 = 1;
	} else if (pfd_freq_hz <= ADI_ADF4382_DCLK_DIV1_1_MAX) {
		dclk_div1 = 1;
		div1 = 2;
	}

	ret = regmap_update_bits(st->regmap, 0x24, ADF4382_DCLK_DIV1_MSK,
				 FIELD_PREP(ADF4382_DCLK_DIV1_MSK, dclk_div1));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, 0x31, ADF4382_DCLK_MODE_MSK, 0xff);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, 0x31, ADF4382_CAL_CT_SEL_MSK, 0xff);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, 0x38, 124);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, 0x3a, 250);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, 0x37, 202);
	if (ret)
		return ret;


	var = DIV_ROUND_UP(div_u64(pfd_freq_hz, div1 * 400000) - 2, 4);
	var = clamp_t(u8, var, 0U, 255U);
	ret = regmap_write(st->regmap, 0x3e, var);
	if (ret)
		return ret;

	// Set LD COUNT
	ret = regmap_update_bits(st->regmap, 0x2c, ADF4382_LD_COUNT_OPWR_MSK,
				 14);
	if (ret)
		return ret;

	// Set LD COUNT
	ret = regmap_update_bits(st->regmap, 0x2c, ADF4382_LDWIN_PW_MSK,
				 ldwin_pw);
	if (ret)
		return ret;

	// Set output power ch1 = 0x7 ch2 =0xf
	ret = regmap_write(st->regmap, 0x29, 0x0b);
	if (ret)
		return ret;

	// Need to set N_INT last to trigger an auto-calibration

	var = (n_int >> 8) & ADF4382_N_INT_MSB_MSK;
	ret = regmap_update_bits(st->regmap, 0x11, ADF4382_N_INT_MSB_MSK, var);
	if (ret)
		return ret;

	var = n_int & ADF4382_N_INT_LSB_MSK;
	ret = regmap_write(st->regmap, 0x10, var);
	if (ret)
		return ret;

	// Work-around to reduce frequency gaps.
	ret = regmap_update_bits(st->regmap, 0x35, ADF4382_M_VCO_BIAS_MSK,
				 ADI_ADF4382_M_VCO_BIAS_POST_CAL_FIX);
	if (ret)
		return ret;

	return 0;
}

int adf4382_get_freq(struct adf4382_state *st, u64 *freq)
{
	return st->freq;
}

static ssize_t adf4382_write(struct iio_dev *indio_dev, uintptr_t private,
			     const struct iio_chan_spec *chan, const char *buf,
			     size_t len)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	unsigned long long freq;
	int ret;

	mutex_lock(&st->lock);
	switch ((u32)private) {
	case ADF4382_FREQ:
		ret = kstrtoull(buf, 10, &freq);
		if (ret)
			break;
		st->freq = freq;

		ret = adf4382_set_freq(st);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static ssize_t adf4382_read(struct iio_dev *indio_dev, uintptr_t private,
			    const struct iio_chan_spec *chan, char *buf)
{
	struct adf4382_state *st = iio_priv(indio_dev);
	u64 val = 0;
	int ret;

	switch ((u32)private) {
	case ADF4382_FREQ:
		ret = adf4382_get_freq(st, &val);
		break;
	default:
		ret = -EINVAL;
		val = 0;
		break;
	}

	return ret ?: sysfs_emit(buf, "%llu\n", val);
}

#define _ADF4382_EXT_INFO(_name, _shared, _ident) { \
		.name = _name, \
		.read = adf4382_read, \
		.write = adf4382_write, \
		.private = _ident, \
		.shared = _shared, \
	}

static const struct iio_chan_spec_ext_info adf4382_ext_info[] = {
	/*
	 * Usually we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 in order to support the entire frequency range
	 * in Hz.
	 */
	_ADF4382_EXT_INFO("frequency", IIO_SHARED_BY_ALL, ADF4382_FREQ),
	{ },
};

static const struct iio_chan_spec adf4382_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = adf4382_ext_info,
	},
};

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
				 ADF4382_SDO_ACT(en) | ADF4382_SDO_ACT_R(en));
	if (ret < 0)
		return ret;

	ret = regmap_multi_reg_write(st->regmap, adf4382_reg_default,
				     ARRAY_SIZE(adf4382_reg_default));
	if (ret)
		return ret;

	st->ref_freq_hz = clk_get_rate(st->clkin);

	ret = regmap_update_bits(st->regmap, 0x20, ADF4382_EN_RDBLR_MSK,
				 st->ref_doubler_en);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, 0x20, ADF4382_R_DIV_MSK,
				 st->ref_div);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, 0x1f, ADF4382_CP_I_MSK,
				 st->cp_i);
	if (ret)
		return ret;

	return adf4382_set_freq(st);
}

static int adf4382_freq_change(struct notifier_block *nb, unsigned long action, void *data)
{
	struct adf4382_state *st = container_of(nb, struct adf4382_state, nb);
	int ret;

	if (action == POST_RATE_CHANGE) {
		mutex_lock(&st->lock);
		ret = notifier_from_errno(adf4382_init(st));
		mutex_unlock(&st->lock);
		return ret;
	}

	return NOTIFY_OK;
}

static void adf4382_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void adf4382_clk_notifier_unreg(void *data)
{
	struct adf4382_state *st = data;

	clk_notifier_unregister(st->clkin, &st->nb);
}

static long adf4382_clock_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	return rate;
}

static unsigned long adf4382_clock_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct adf4382_state *st = container_of(hw, struct adf4382_state, clk_hw);

	return to_ccf_scaled(st->freq, &st->scale);
}

static int adf4382_clock_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct adf4382_state *st = container_of(hw, struct adf4382_state, clk_hw);
	unsigned long long scaled_rate;

	scaled_rate = from_ccf_scaled(rate, &st->scale);

	st->ref_freq_hz = parent_rate;
	st->freq = scaled_rate;

	return adf4382_set_freq(st);
}

static const struct clk_ops adf4382_clock_ops = {
	.set_rate = adf4382_clock_set_rate,
	.recalc_rate = adf4382_clock_recalc_rate,
	.round_rate = adf4382_clock_round_rate,
};

static int adf4382_setup_clk(struct adf4382_state *st)
{
	struct device *dev = &st->spi->dev;
	struct device_node *of_node = dev_of_node(dev);
	struct clk_init_data init;
	const char *clk_name;
	struct clk *clk;
	const char *parent_name;

	if (!of_node)
		return 0;

	parent_name = __clk_get_name(st->clkin);

	clk_name = of_node->name;
	of_property_read_string(of_node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &adf4382_clock_ops;
	init.parent_names = (parent_name ? &parent_name : NULL);
	init.num_parents = (parent_name ? 1 : 0);

	st->clk_hw.init = &init;
	clk = devm_clk_register(dev, &st->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	return of_clk_add_provider(of_node, of_clk_src_simple_get, clk);
}

static int adf4382_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct adf4382_state *st;
	u32 tmp;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adf4382_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st = iio_priv(indio_dev);

	indio_dev->info = &adf4382_info;
	indio_dev->name = "adf4382";
	indio_dev->channels = adf4382_channels;
	indio_dev->num_channels = ARRAY_SIZE(adf4382_channels);

	st->regmap = regmap;
	st->spi = spi;

	mutex_init(&st->lock);

	ret = of_clk_get_scale(st->spi->dev.of_node, NULL, &st->scale);
	if (ret < 0) {
		st->scale.mult = 1;
		st->scale.div = 10;
	}

	ret = device_property_read_u64(&st->spi->dev, "adi,power-up-frequency",
				       &st->freq);
	if (ret)
		st->freq = ADF4382_RFOUT_DEFAULT;
	
	ret = device_property_read_u32(&st->spi->dev, "adi,charge-pump-current",
				      &tmp);
	if (ret)
		st->cp_i = ADF4382_CP_I_DEFAULT;
	else 
		st->cp_i = (u8)tmp;
	dev_info(&st->spi->dev, "cp_i = %d", st->cp_i);

	ret = device_property_read_u32(&st->spi->dev, "adi,ref-divider",
				      &tmp);
	if ((ret) || (!tmp))
		st->ref_div = ADF4382_REF_DIV_DEFAULT;
	else
		st->ref_div = (u8)tmp;
	dev_info(&st->spi->dev, "ref_div = %d", st->ref_div);

	st->spi_3wire_en = device_property_read_bool(&st->spi->dev,
						     "adi,spi-3wire-enable");
	st->ref_doubler_en = device_property_read_bool(&st->spi->dev,
						     "adi,ref-doubler-enable");
	dev_info(&st->spi->dev, "ref_doubler_en = %d", st->ref_doubler_en);

	st->clkin = devm_clk_get(&spi->dev, "ref_clk");
	if (IS_ERR(st->clkin))
		return PTR_ERR(st->clkin);

	ret = clk_prepare_enable(st->clkin);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, adf4382_clk_disable, st->clkin);
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
	if (ret) {
		dev_err(&spi->dev, "adf4382 init failed\n");
		return ret;
	}

	adf4382_setup_clk(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id adf4382_id[] = {
	{ "adf4382", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, adf4382_id);

static const struct of_device_id adf4382_of_match[] = {
	{ .compatible = "adi,adf4382" },
	{},
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
MODULE_DESCRIPTION("Analog Devices ADF4382");
MODULE_LICENSE("GPL v2");
