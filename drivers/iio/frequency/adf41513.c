// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADF41513 SPI PLL Frequency Synthesizer driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/units.h>

/* Registers */
#define ADF41513_REG0		0
#define ADF41513_REG1		1
#define ADF41513_REG2		2
#define ADF41513_REG3		3
#define ADF41513_REG4		4
#define ADF41513_REG5		5
#define ADF41513_REG6		6
#define ADF41513_REG7		7
#define ADF41513_REG8		8
#define ADF41513_REG9		9
#define ADF41513_REG10		10
#define ADF41513_REG11		11
#define ADF41513_REG12		12
#define ADF41513_REG13		13
#define ADF41513_REG_NUM	14

#define ADF41513_SYNC_REG0	BIT_U16(ADF41513_REG0)
#define ADF41513_SYNC_REG1	BIT_U16(ADF41513_REG1)
#define ADF41513_SYNC_REG2	BIT_U16(ADF41513_REG2)
#define ADF41513_SYNC_REG3	BIT_U16(ADF41513_REG3)
#define ADF41513_SYNC_REG4	BIT_U16(ADF41513_REG4)
#define ADF41513_SYNC_REG5	BIT_U16(ADF41513_REG5)
#define ADF41513_SYNC_REG6	BIT_U16(ADF41513_REG6)
#define ADF41513_SYNC_REG7	BIT_U16(ADF41513_REG7)
#define ADF41513_SYNC_REG9	BIT_U16(ADF41513_REG9)
#define ADF41513_SYNC_REG11	BIT_U16(ADF41513_REG11)
#define ADF41513_SYNC_REG12	BIT_U16(ADF41513_REG12)
#define ADF41513_SYNC_REG13	BIT_U16(ADF41513_REG13)
#define ADF41513_SYNC_DIFF	0
#define ADF41513_SYNC_ALL	GENMASK_U16(ADF41513_REG13, ADF41513_REG0)

/* REG0 Bit Definitions */
#define ADF41513_REG0_CTRL_BITS_MSK		GENMASK_U32(3, 0)
#define ADF41513_REG0_INT_MSK			GENMASK_U32(19, 4)
#define ADF41513_REG0_INT(x)			FIELD_PREP(ADF41513_REG0_INT_MSK, x)
#define ADF41513_REG0_VAR_MOD_MSK		BIT_U32(28)
#define ADF41513_REG0_VAR_MOD(x)		FIELD_PREP(ADF41513_REG0_VAR_MOD_MSK, x)

/* REG1 Bit Definitions */
#define ADF41513_REG1_FRAC1_MSK			GENMASK_U32(28, 4)
#define ADF41513_REG1_FRAC1(x)			FIELD_PREP(ADF41513_REG1_FRAC1_MSK, x)
#define ADF41513_REG1_DITHER2_MSK		BIT_U32(31)
#define ADF41513_REG1_DITHER2(x)		FIELD_PREP(ADF41513_REG1_DITHER2_MSK, x)

/* REG2 Bit Definitions */
#define ADF41513_REG2_PHASE_VAL_MSK		GENMASK_U32(15, 4)
#define ADF41513_REG2_PHASE_VAL(x)		FIELD_PREP(ADF41513_REG2_PHASE_VAL_MSK, x)
#define ADF41513_REG2_PHASE_ADJ_MSK		BIT_U32(31)
#define ADF41513_REG2_PHASE_ADJ(x)		FIELD_PREP(ADF41513_REG2_PHASE_ADJ_MSK, x)

/* REG3 Bit Definitions */
#define ADF41513_REG3_FRAC2_MSK			GENMASK_U32(27, 4)
#define ADF41513_REG3_FRAC2(x)			FIELD_PREP(ADF41513_REG3_FRAC2_MSK, x)

/* REG4 Bit Definitions */
#define ADF41513_REG4_MOD2_MSK			GENMASK_U32(27, 4)
#define ADF41513_REG4_MOD2(x)			FIELD_PREP(ADF41513_REG4_MOD2_MSK, x)

/* REG5 Bit Definitions */
#define ADF41513_REG5_CLK1_DIV_MSK		GENMASK_U32(15, 4)
#define ADF41513_REG5_CLK1_DIV(x)		FIELD_PREP(ADF41513_REG5_CLK1_DIV_MSK, x)
#define ADF41513_REG5_R_CNT_MSK			GENMASK_U32(20, 16)
#define ADF41513_REG5_R_CNT(x)			FIELD_PREP(ADF41513_REG5_R_CNT_MSK, x)
#define ADF41513_REG5_REF_DOUBLER_MSK		BIT_U32(21)
#define ADF41513_REG5_REF_DOUBLER(x)		FIELD_PREP(ADF41513_REG5_REF_DOUBLER_MSK, x)
#define ADF41513_REG5_RDIV2_MSK			BIT_U32(22)
#define ADF41513_REG5_RDIV2(x)			FIELD_PREP(ADF41513_REG5_RDIV2_MSK, x)
#define ADF41513_REG5_PRESCALER_MSK		BIT_U32(23)
#define ADF41513_REG5_PRESCALER(x)		FIELD_PREP(ADF41513_REG5_PRESCALER_MSK, x)
#define ADF41513_REG5_LSB_P1_MSK		BIT_U32(24)
#define ADF41513_REG5_LSB_P1(x)			FIELD_PREP(ADF41513_REG5_LSB_P1_MSK, x)
#define ADF41513_REG5_CP_CURRENT_MSK		GENMASK_U32(28, 25)
#define ADF41513_REG5_CP_CURRENT(x)		FIELD_PREP(ADF41513_REG5_CP_CURRENT_MSK, x)
#define ADF41513_REG5_DLD_MODES_MSK		GENMASK_U32(31, 30)
#define ADF41513_REG5_DLD_MODES(x)		FIELD_PREP(ADF41513_REG5_DLD_MODES_MSK, x)

/* REG6 Bit Definitions */
#define ADF41513_REG6_COUNTER_RESET_MSK		BIT_U32(4)
#define ADF41513_REG6_COUNTER_RESET(x)		FIELD_PREP(ADF41513_REG6_COUNTER_RESET_MSK, x)
#define ADF41513_REG6_CP_TRISTATE_MSK		BIT_U32(5)
#define ADF41513_REG6_CP_TRISTATE(x)		FIELD_PREP(ADF41513_REG6_CP_TRISTATE_MSK, x)
#define ADF41513_REG6_POWER_DOWN_MSK		BIT_U32(6)
#define ADF41513_REG6_POWER_DOWN(x)		FIELD_PREP(ADF41513_REG6_POWER_DOWN_MSK, x)
#define ADF41513_REG6_PD_POLARITY_MSK		BIT_U32(7)
#define ADF41513_REG6_PD_POLARITY(x)		FIELD_PREP(ADF41513_REG6_PD_POLARITY_MSK, x)
#define ADF41513_REG6_LDP_MSK			GENMASK_U32(9, 8)
#define ADF41513_REG6_LDP(x)			FIELD_PREP(ADF41513_REG6_LDP_MSK, x)
#define ADF41513_REG6_CP_TRISTATE_PD_ON_MSK	BIT_U32(16)
#define ADF41513_REG6_CP_TRISTATE_PD_ON(x)	FIELD_PREP(ADF41513_REG6_CP_TRISTATE_PD_ON_MSK, x)
#define ADF41513_REG6_SD_RESET_MSK		BIT_U32(17)
#define ADF41513_REG6_SD_RESET(x)		FIELD_PREP(ADF41513_REG6_SD_RESET_MSK, x)
#define ADF41513_REG6_LOL_ENABLE_MSK		BIT_U32(18)
#define ADF41513_REG6_LOL_ENABLE(x)		FIELD_PREP(ADF41513_REG6_LOL_ENABLE_MSK, x)
#define ADF41513_REG6_ABP_MSK			BIT_U32(19)
#define ADF41513_REG6_ABP(x)			FIELD_PREP(ADF41513_REG6_ABP_MSK, x)
#define ADF41513_REG6_INT_MODE_MSK		BIT_U32(20)
#define ADF41513_REG6_INT_MODE(x)		FIELD_PREP(ADF41513_REG6_INT_MODE_MSK, x)
#define ADF41513_REG6_BLEED_ENABLE_MSK		BIT_U32(22)
#define ADF41513_REG6_BLEED_ENABLE(x)		FIELD_PREP(ADF41513_REG6_BLEED_ENABLE_MSK, x)
#define ADF41513_REG6_BLEED_POLARITY_MSK	BIT_U32(23)
#define ADF41513_REG6_BLEED_POLARITY(x)		FIELD_PREP(ADF41513_REG6_BLEED_POLARITY_MSK, x)
#define ADF41513_REG6_BLEED_CURRENT_MSK		GENMASK_U32(31, 24)
#define ADF41513_REG6_BLEED_CURRENT(x)		FIELD_PREP(ADF41513_REG6_BLEED_CURRENT_MSK, x)

/* REG7 Bit Definitions */
#define ADF41513_REG7_CLK2_DIV_MSK		GENMASK_U32(17, 6)
#define ADF41513_REG7_CLK2_DIV(x)		FIELD_PREP(ADF41513_REG7_CLK2_DIV_MSK, x)
#define ADF41513_REG7_CLK_DIV_MODE_MSK		GENMASK_U32(19, 18)
#define ADF41513_REG7_CLK_DIV_MODE(x)		FIELD_PREP(ADF41513_REG7_CLK_DIV_MODE_MSK, x)
#define ADF41513_REG7_PS_BIAS_MSK		GENMASK_U32(21, 20)
#define ADF41513_REG7_PS_BIAS(x)		FIELD_PREP(ADF41513_REG7_PS_BIAS_MSK, x)
#define ADF41513_REG7_N_DELAY_MSK		GENMASK_U32(23, 22)
#define ADF41513_REG7_N_DELAY(x)		FIELD_PREP(ADF41513_REG7_N_DELAY_MSK, x)
#define ADF41513_REG7_LD_CLK_SEL_MSK		BIT_U32(26)
#define ADF41513_REG7_LD_CLK_SEL(x)		FIELD_PREP(ADF41513_REG7_LD_CLK_SEL_MSK, x)
#define ADF41513_REG7_LD_COUNT_MSK		GENMASK_U32(29, 27)
#define ADF41513_REG7_LD_COUNT(x)		FIELD_PREP(ADF41513_REG7_LD_COUNT_MSK, x)

/* REG9 Bit Definitions */
#define ADF41513_REG9_LD_BIAS_MSK		GENMASK_U32(31, 30)
#define ADF41513_REG9_LD_BIAS(x)		FIELD_PREP(ADF41513_REG9_LD_BIAS_MSK, x)

/* REG11 Bit Definitions */
#define ADF41513_REG11_POWER_DOWN_SEL_MSK	BIT_U32(31)
#define ADF41513_REG11_POWER_DOWN_SEL(x)	FIELD_PREP(ADF41513_REG11_POWER_DOWN_SEL_MSK, x)

/* REG12 Bit Definitions */
#define ADF41513_REG12_READBACK_SEL_MSK		GENMASK_U32(19, 14)
#define ADF41513_REG12_READBACK_SEL(x)		FIELD_PREP(ADF41513_REG12_READBACK_SEL_MSK, x)
#define ADF41513_REG12_LE_SELECT_MSK		BIT_U32(20)
#define ADF41513_REG12_LE_SELECT(x)		FIELD_PREP(ADF41513_REG12_LE_SELECT_MSK, x)
#define ADF41513_REG12_MASTER_RESET_MSK		BIT_U32(22)
#define ADF41513_REG12_MASTER_RESET(x)		FIELD_PREP(ADF41513_REG12_MASTER_RESET_MSK, x)
#define ADF41513_REG12_LOGIC_LEVEL_MSK		BIT_U32(27)
#define ADF41513_REG12_LOGIC_LEVEL(x)		FIELD_PREP(ADF41513_REG12_LOGIC_LEVEL_MSK, x)
#define ADF41513_REG12_MUXOUT_MSK		GENMASK_U32(31, 28)
#define ADF41513_REG12_MUXOUT(x)		FIELD_PREP(ADF41513_REG12_MUXOUT_MSK, x)

/* MUXOUT Selection */
#define ADF41513_MUXOUT_TRISTATE		0x0
#define ADF41513_MUXOUT_DVDD			0x1
#define ADF41513_MUXOUT_DGND			0x2
#define ADF41513_MUXOUT_R_DIV			0x3
#define ADF41513_MUXOUT_N_DIV			0x4
#define ADF41513_MUXOUT_DIG_LD			0x6
#define ADF41513_MUXOUT_SDO			0x7
#define ADF41513_MUXOUT_READBACK		0x8
#define ADF41513_MUXOUT_CLK1_DIV		0xA
#define ADF41513_MUXOUT_R_DIV2			0xD
#define ADF41513_MUXOUT_N_DIV2			0xE

/* DLD Mode Selection */
#define ADF41513_DLD_TRISTATE			0x0
#define ADF41513_DLD_DIG_LD			0x1
#define ADF41513_DLD_LOW			0x2
#define ADF41513_DLD_HIGH			0x3

/* Prescaler Selection */
#define ADF41513_PRESCALER_4_5			0
#define ADF41513_PRESCALER_8_9			1
#define ADF41513_PRESCALER_AUTO			2

/* Specifications */
#define ADF41513_MIN_RF_FREQ			(1000ULL * HZ_PER_MHZ)
#define ADF41510_MAX_RF_FREQ			(10000ULL * HZ_PER_MHZ)
#define ADF41513_MAX_RF_FREQ			(26500ULL * HZ_PER_MHZ)

#define ADF41513_MIN_REF_FREQ			(10U * HZ_PER_MHZ)
#define ADF41513_MAX_REF_FREQ			(800U * HZ_PER_MHZ)
#define ADF41513_MAX_REF_FREQ_DOUBLER		(225U * HZ_PER_MHZ)

#define ADF41513_MAX_PFD_FREQ_INT_N_UHZ		(250ULL * HZ_PER_MHZ * MICROHZ_PER_HZ)
#define ADF41513_MAX_PFD_FREQ_FRAC_N_UHZ	(125ULL * HZ_PER_MHZ * MICROHZ_PER_HZ)

#define ADF41513_MIN_INT_4_5			20
#define ADF41513_MAX_INT_4_5			511
#define ADF41513_MIN_INT_8_9			64
#define ADF41513_MAX_INT_8_9			1023

#define ADF41513_MIN_INT_FRAC_4_5		23
#define ADF41513_MIN_INT_FRAC_8_9		75

#define ADF41513_MIN_R_CNT			1
#define ADF41513_MAX_R_CNT			32

#define ADF41513_MIN_CP_VOLTAGE_mV		810
#define ADF41513_MAX_CP_VOLTAGE_mV		12960

#define ADF41513_MAX_CLK_DIVIDER		4095
#define ADF41513_MAX_LD_BIAS_uA			40
#define ADF41513_LD_BIAS_STEP_uA		10

#define ADF41513_FIXED_MODULUS			BIT_U64(25)
#define ADF41513_MAX_MOD2			(BIT_U64(24) - 1)

#define ADF41513_HZ_DECIMAL_PRECISION		6

enum {
	ADF41513_FREQ,
	ADF41513_POWER_DOWN,
	ADF41513_FREQ_RESOLUTION,
	ADF41513_FREQ_REFIN,
};

enum adf41513_pll_mode {
	ADF41513_MODE_INTEGER_N,
	ADF41513_MODE_FIXED_MODULUS,
	ADF41513_MODE_VARIABLE_MODULUS,
	ADF41513_MODE_INVALID,
};

struct adf41513_chip_info {
	const char *name;
	bool has_prescaler_8_9;
	u64 max_rf_freq_hz;
};

struct adf41513_data {
	u64 power_up_frequency_hz;
	u64 freq_resolution_uhz;
	u32 charge_pump_voltage_mv;

	u8 ref_div_factor;
	bool ref_doubler_en;
	bool ref_div2_en;
	bool phase_detector_polarity;

	u8 lock_detect_precision;
	u8 lock_detect_count;
	u8 lock_detect_bias;
	bool fast_lock_en;

	u8 muxout_select;
	bool muxout_1v8_en;

	bool phase_resync_en;
	bool load_enable_sync;
	u16 phase_resync_clk_div[2];
};

struct adf41513_pll_settings {
	enum adf41513_pll_mode mode;

	/* reference path parameters */
	u8 r_counter;
	u8 ref_doubler;
	u8 ref_div2;
	u8 prescaler;

	/* frequency parameters */
	u64 target_frequency_uhz;
	u64 actual_frequency_uhz;
	u64 pfd_frequency_uhz;

	/* pll parameters */
	u16 int_value;
	u32 frac1;
	u32 frac2;
	u32 mod2;
};

struct adf41513_state {
	const struct adf41513_chip_info *chip_info;
	struct spi_device *spi;
	struct gpio_desc *lock_detect;
	struct gpio_desc *chip_enable;
	struct clk *ref_clk;
	struct clk *clk_out;
	struct clk_hw clk_hw;

	u64 ref_freq_hz;

	/*
	 * Lock for accessing device registers. Some operations require
	 * multiple consecutive R/W operations, during which the device
	 * shouldn't be interrupted. The buffers are also shared across
	 * all operations so need to be protected on stand alone reads and
	 * writes.
	 */
	struct mutex lock;

	/* Cached register values */
	u32 regs[ADF41513_REG_NUM];
	u32 regs_hw[ADF41513_REG_NUM];

	/* PLL dt data */
	struct adf41513_data data;

	/* PLL configuration */
	struct adf41513_pll_settings settings;

	/*
	 * DMA (thus cache coherency maintenance) may require that
	 * transfer buffers live in their own cache lines.
	 */
	__be32 buf __aligned(IIO_DMA_MINALIGN);
};

#define to_adf41513_state(_hw) container_of(_hw, struct adf41513_state, clk_hw)

static const u32 adf41513_cp_voltage_mv[] = {
	810, 1620, 2430, 3240, 4050, 4860, 5670, 6480,		/* 0 - 7 */
	7290, 8100, 8910, 9720, 10530, 11340, 12150, 12960	/* 8 - 15 */
};

static const char * const adf41513_muxout_modes[] = {
	[ADF41513_MUXOUT_TRISTATE] = "high_z",
	[ADF41513_MUXOUT_DVDD] = "muxout_high",
	[ADF41513_MUXOUT_DGND] = "muxout_low",
	[ADF41513_MUXOUT_R_DIV] = "f_div_rclk",
	[ADF41513_MUXOUT_N_DIV] = "f_div_nclk",
	[ADF41513_MUXOUT_DIG_LD] = "lock_detect",
	[ADF41513_MUXOUT_SDO] = "serial_data",
	[ADF41513_MUXOUT_READBACK] = "readback",
	[ADF41513_MUXOUT_CLK1_DIV] = "f_div_clk1",
	[ADF41513_MUXOUT_R_DIV2] = "f_div_rclk_2",
	[ADF41513_MUXOUT_N_DIV2] = "f_div_nclk_2",
};

/* Parse fixed point frequency string in Hz into microhertz u64 value:
 * This driver supports sub-Hz frequency resolution with frequency ranges
 * up to several GHz (> 2^32). To achieve this, frequency calculations are
 * done in microhertz using u64 variables. iio core parse helpers only support
 * 64-bit integers or 32-bit integers plus fractional part. Here, we need
 * to 64-bit integer plus fractional part (6 decimal places).
 * See __iio_str_to_fixpoint and iio_write_channel_info in
 * drivers/iio/industrialio-core.c
 */
static int adf41513_parse_uhz(const char *str, u64 *freq_uhz)
{
	u64 uhz = 0;
	int f_count = ADF41513_HZ_DECIMAL_PRECISION;
	bool frac_part = false;

	if (str[0] == '+')
		str++;

	while (*str && f_count > 0) {
		if ('0' <= *str && *str <= '9') {
			uhz = uhz * 10 + *str - '0';
			if (frac_part)
				f_count--;
		} else if (*str == '\n') {
			if (*(str + 1) == '\0')
				break;
			return -EINVAL;
		} else if (*str == '.' && !frac_part) {
			frac_part = true;
		} else {
			return -EINVAL;
		}
		str++;
	}

	for (; f_count > 0; f_count--)
		uhz *= 10;

	*freq_uhz = uhz;

	return 0;
}

static int adf41513_uhz_to_str(u64 freq_uhz, char *buf)
{
	u64 frac_part;
	u64 int_part = div64_u64_rem(freq_uhz, MICROHZ_PER_HZ, &frac_part);

	return sysfs_emit(buf, "%llu.%06llu\n", int_part, frac_part);
}

static int adf41513_sync_config(struct adf41513_state *st, u16 sync_mask)
{
	int ret;
	int i;

	/* write registers in reverse order (R13 to R0)*/
	for (i = ADF41513_REG13; i >= ADF41513_REG0; i--) {
		if (st->regs_hw[i] != st->regs[i] || sync_mask & BIT_U16(i)) {
			st->buf = cpu_to_be32(st->regs[i] | i);
			ret = spi_write(st->spi, &st->buf, sizeof(st->buf));
			if (ret < 0)
				return ret;
			st->regs_hw[i] = st->regs[i];
			dev_dbg(&st->spi->dev, "REG%d <= 0x%08X\n", i, st->regs[i] | i);
		}
	}

	return 0;
}

static u64 adf41513_pll_get_rate(struct adf41513_state *st)
{
	if (st->settings.mode != ADF41513_MODE_INVALID)
		return st->settings.actual_frequency_uhz;

	/* get pll settings from regs_hw */
	st->settings.int_value =
		FIELD_GET(ADF41513_REG0_INT_MSK, st->regs_hw[ADF41513_REG0]);
	st->settings.frac1 =
		FIELD_GET(ADF41513_REG1_FRAC1_MSK, st->regs_hw[ADF41513_REG1]);
	st->settings.frac2 =
		FIELD_GET(ADF41513_REG3_FRAC2_MSK, st->regs_hw[ADF41513_REG3]);
	st->settings.mod2 =
		FIELD_GET(ADF41513_REG4_MOD2_MSK, st->regs_hw[ADF41513_REG4]);
	st->settings.r_counter =
		FIELD_GET(ADF41513_REG5_R_CNT_MSK, st->regs_hw[ADF41513_REG5]);
	st->settings.ref_doubler =
		FIELD_GET(ADF41513_REG5_REF_DOUBLER_MSK, st->regs_hw[ADF41513_REG5]);
	st->settings.ref_div2 =
		FIELD_GET(ADF41513_REG5_RDIV2_MSK, st->regs_hw[ADF41513_REG5]);
	st->settings.prescaler =
		FIELD_GET(ADF41513_REG5_PRESCALER_MSK, st->regs_hw[ADF41513_REG5]);

	/* calculate pfd frequency */
	st->settings.pfd_frequency_uhz = st->ref_freq_hz * MICROHZ_PER_HZ;
	if (st->settings.ref_doubler)
		st->settings.pfd_frequency_uhz <<= 1;
	if (st->settings.ref_div2)
		st->settings.pfd_frequency_uhz >>= 1;
	st->settings.pfd_frequency_uhz = div_u64(st->settings.pfd_frequency_uhz,
						 st->settings.r_counter);
	st->settings.actual_frequency_uhz =
		(u64)st->settings.int_value * st->settings.pfd_frequency_uhz;

	/* check if int mode is selected */
	if (FIELD_GET(ADF41513_REG6_INT_MODE_MSK, st->regs_hw[ADF41513_REG6])) {
		st->settings.mode = ADF41513_MODE_INTEGER_N;
	} else {
		st->settings.actual_frequency_uhz +=
			mul_u64_u64_div_u64(st->settings.frac1,
					    st->settings.pfd_frequency_uhz,
					    ADF41513_FIXED_MODULUS);

		/* check if variable modulus is selected */
		if (FIELD_GET(ADF41513_REG0_VAR_MOD_MSK, st->regs_hw[ADF41513_REG0])) {
			st->settings.actual_frequency_uhz +=
				mul_u64_u64_div_u64(st->settings.frac2,
						    st->settings.pfd_frequency_uhz,
						    ADF41513_FIXED_MODULUS * st->settings.mod2);

			st->settings.mode = ADF41513_MODE_VARIABLE_MODULUS;
		} else {
			st->settings.mode = ADF41513_MODE_FIXED_MODULUS;
		}
	}

	st->settings.target_frequency_uhz = st->settings.actual_frequency_uhz;

	return st->settings.actual_frequency_uhz;
}

static int adf41513_calc_pfd_frequency(struct adf41513_state *st,
				       struct adf41513_pll_settings *result,
				       u64 fpfd_limit_uhz)
{
	/* set initial values from platform data */
	result->ref_div2 = st->data.ref_div2_en ? 1 : 0;
	result->ref_doubler = st->data.ref_doubler_en ? 1 : 0;

	if (st->data.ref_doubler_en &&
	    st->ref_freq_hz > ADF41513_MAX_REF_FREQ_DOUBLER) {
		result->ref_doubler = 0;
		dev_warn(&st->spi->dev, "Disabling ref doubler due to high reference frequency\n");
	}

	/* set R counter starting with the div factor from platform data */
	result->r_counter = st->data.ref_div_factor - 1;

	do {
		result->r_counter++;
		result->pfd_frequency_uhz = div64_u64(st->ref_freq_hz * MICROHZ_PER_HZ *
					    (result->ref_doubler ? 2 : 1), ((result->r_counter) *
					    (result->ref_div2 ? 2 : 1)));
	} while (result->pfd_frequency_uhz > fpfd_limit_uhz);

	if (result->r_counter > ADF41513_MAX_R_CNT) {
		dev_err(&st->spi->dev, "Cannot optimize PFD frequency\n");
		return -ERANGE;
	}

	return 0;
}

static int adf41513_calc_integer_n(struct adf41513_state *st,
				   struct adf41513_pll_settings *result)
{
	u16 max_int = (st->chip_info->has_prescaler_8_9) ?
		      ADF41513_MAX_INT_8_9 : ADF41513_MAX_INT_4_5;

	u64 freq_error_uhz;
	u16 int_value = (u16)div64_u64_rem(result->target_frequency_uhz,
		result->pfd_frequency_uhz, &freq_error_uhz);

	/* check if N is close to an integer (within tolerance) */
	if (freq_error_uhz > (result->pfd_frequency_uhz >> 1) && int_value < max_int) {
		int_value++;
		freq_error_uhz = result->pfd_frequency_uhz - freq_error_uhz;
	}

	if (freq_error_uhz > st->data.freq_resolution_uhz)
		return -ERANGE;

	/* set prescaler */
	if (st->chip_info->has_prescaler_8_9 && int_value >= ADF41513_MIN_INT_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_4_5 && int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	result->actual_frequency_uhz = (u64)int_value * result->pfd_frequency_uhz;

	result->mode = ADF41513_MODE_INTEGER_N;
	result->int_value = int_value;
	result->frac1 = 0;
	result->frac2 = 0;
	result->mod2 = 0;

	return 0;
}

static int adf41513_calc_fixed_mod(struct adf41513_state *st,
				   struct adf41513_pll_settings *result)
{
	const u64 resolution_uhz = div64_u64(result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS);

	u64 fractional_part_uhz, freq_error_uhz;
	u32 frac1;

	/* in fixed modulus LSB_P1 adds a frequency offset of f_pfd/2^26 */
	u64 target_frequency_uhz = result->target_frequency_uhz - (resolution_uhz >> 1);
	u16 int_value = (u16)div64_u64_rem(target_frequency_uhz, result->pfd_frequency_uhz,
					   &fractional_part_uhz);

	/* set prescaler */
	if (st->chip_info->has_prescaler_8_9 && int_value >= ADF41513_MIN_INT_FRAC_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_FRAC_4_5 && int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	/* calculate frac1 */
	frac1 = mul_u64_u64_div_u64(fractional_part_uhz, ADF41513_FIXED_MODULUS,
				    result->pfd_frequency_uhz);

	freq_error_uhz = fractional_part_uhz -
		mul_u64_u64_div_u64((u64)frac1, result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS);

	if (freq_error_uhz > (resolution_uhz >> 1) && frac1 < (ADF41513_FIXED_MODULUS - 1)) {
		frac1++;
		freq_error_uhz = resolution_uhz - freq_error_uhz;
	}

	if (freq_error_uhz > st->data.freq_resolution_uhz)
		return -ERANGE;

	/* calculate actual frequency in uHz */
	result->actual_frequency_uhz = (u64)int_value * result->pfd_frequency_uhz +
		(resolution_uhz >> 1) +
		mul_u64_u64_div_u64((u64)frac1, result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS);

	result->mode = ADF41513_MODE_FIXED_MODULUS;
	result->int_value = int_value;
	result->frac1 = frac1;
	result->frac2 = 0;
	result->mod2 = 0;

	return 0;
}

static int adf41513_calc_variable_mod(struct adf41513_state *st,
				      struct adf41513_pll_settings *result)
{
	u64 fractional_part_uhz, tmp, remaining_frac;
	u32 frac1, frac2, mod2;
	u16 int_value = (u16)div64_u64_rem(result->target_frequency_uhz,
		result->pfd_frequency_uhz, &fractional_part_uhz);

	/* set prescaler */
	if (st->chip_info->has_prescaler_8_9 && int_value >= ADF41513_MIN_INT_FRAC_8_9 &&
	    int_value <= ADF41513_MAX_INT_8_9)
		result->prescaler = 1;
	else if (int_value >= ADF41513_MIN_INT_FRAC_4_5 && int_value <= ADF41513_MAX_INT_4_5)
		result->prescaler = 0;
	else
		return -ERANGE;

	/* calculate required MOD2 based on target resolution / 2 (enhanced for uHz) */
	mod2 = min_t(u64,
		     DIV64_U64_ROUND_CLOSEST(result->pfd_frequency_uhz * 2,
					     st->data.freq_resolution_uhz * ADF41513_FIXED_MODULUS),
		     ADF41513_MAX_MOD2);

	/* ensure MOD2 is at least 2 */
	if (mod2 < 2)
		mod2 = 2;

	/* calculate frac1 and frac2 */
	frac1 = mul_u64_u64_div_u64(fractional_part_uhz, ADF41513_FIXED_MODULUS,
				    result->pfd_frequency_uhz);
	remaining_frac = fractional_part_uhz -
		mul_u64_u64_div_u64((u64)frac1, result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS);
	frac2 = mul_u64_u64_div_u64(remaining_frac, ADF41513_FIXED_MODULUS * mod2,
				    result->pfd_frequency_uhz);

	/* calculate actual frequency in uHz */
	tmp = (u64)frac1 * mod2 + frac2;
	result->actual_frequency_uhz = (u64)int_value * result->pfd_frequency_uhz +
		mul_u64_u64_div_u64(tmp, result->pfd_frequency_uhz, ADF41513_FIXED_MODULUS * mod2);

	result->mode = ADF41513_MODE_VARIABLE_MODULUS;
	result->int_value = int_value;
	result->frac1 = frac1;
	result->frac2 = frac2;
	result->mod2 = mod2;

	return 0;
}

static int adf41513_calc_pll_settings(struct adf41513_state *st,
				      struct adf41513_pll_settings *result,
				      u64 rf_out_uhz)
{
	const u64 max_rf_freq_uhz = st->chip_info->max_rf_freq_hz * MICROHZ_PER_HZ;
	const u64 min_rf_freq_uhz = ADF41513_MIN_RF_FREQ * MICROHZ_PER_HZ;

	u64 pfd_freq_limit_uhz;
	int ret;

	/* input validation */
	if (rf_out_uhz < min_rf_freq_uhz || rf_out_uhz > max_rf_freq_uhz) {
		dev_err(&st->spi->dev, "RF frequency %llu uHz out of range [%llu, %llu] uHz\n",
			rf_out_uhz, min_rf_freq_uhz, max_rf_freq_uhz);
		return -EINVAL;
	}
	result->target_frequency_uhz = rf_out_uhz;

	/* try integer-N first (best phase noise performance) */
	pfd_freq_limit_uhz = min_t(u64, div_u64(rf_out_uhz, ADF41513_MIN_INT_4_5),
				   ADF41513_MAX_PFD_FREQ_INT_N_UHZ);
	ret = adf41513_calc_pfd_frequency(st, result, pfd_freq_limit_uhz);
	if (ret < 0)
		return ret;

	ret = adf41513_calc_integer_n(st, result);
	if (ret < 0) {
		/* try fractional-N: recompute pfd frequency if necessary */
		pfd_freq_limit_uhz = min_t(u64, div_u64(rf_out_uhz, ADF41513_MIN_INT_FRAC_4_5),
					   ADF41513_MAX_PFD_FREQ_FRAC_N_UHZ);
		if (pfd_freq_limit_uhz < result->pfd_frequency_uhz) {
			ret = adf41513_calc_pfd_frequency(st, result, pfd_freq_limit_uhz);
			if (ret < 0)
				return ret;
		}

		/* fixed-modulus attempt */
		ret = adf41513_calc_fixed_mod(st, result);
		if (ret < 0) {
			/* variable-modulus attempt */
			ret = adf41513_calc_variable_mod(st, result);
			if (ret < 0) {
				dev_err(&st->spi->dev,
					"no valid PLL configuration found for %llu uHz\n",
					rf_out_uhz);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int adf41513_set_frequency(struct adf41513_state *st, u64 freq_uhz, u16 sync_mask)
{
	struct adf41513_pll_settings result;
	u32 bleed_value = 0;
	int ret;

	/* calculate pll settings candidate */
	ret = adf41513_calc_pll_settings(st, &result, freq_uhz);
	if (ret < 0)
		return ret;

	/* apply computed results to state pll settings */
	memcpy(&st->settings, &result, sizeof(struct adf41513_pll_settings));

	/* log calculation result */
	dev_dbg(&st->spi->dev, "%s mode: int=%u, frac1=%u, frac2=%u, mod2=%u, fpdf=%llu Hz, prescaler=%s\n",
		(result.mode == ADF41513_MODE_INTEGER_N) ? "integer-n" :
		(result.mode == ADF41513_MODE_FIXED_MODULUS) ? "fixed-modulus" : "variable-modulus",
		result.int_value, result.frac1, result.frac2, result.mod2,
		div64_u64(result.pfd_frequency_uhz, MICROHZ_PER_HZ),
		result.prescaler ? "8/9" : "4/5");

	/* int */
	st->regs[ADF41513_REG0] = ADF41513_REG0_INT(st->settings.int_value);
	if (st->settings.mode == ADF41513_MODE_VARIABLE_MODULUS)
		st->regs[ADF41513_REG0] |= ADF41513_REG0_VAR_MOD_MSK;
	/* frac1 */
	st->regs[ADF41513_REG1] = ADF41513_REG1_FRAC1(st->settings.frac1);
	if (st->settings.mode != ADF41513_MODE_INTEGER_N)
		st->regs[ADF41513_REG1] |= ADF41513_REG1_DITHER2_MSK;

	/* frac2 */
	st->regs[ADF41513_REG3] = ADF41513_REG3_FRAC2(st->settings.frac2);
	/* mod2 */
	st->regs[ADF41513_REG4] &= ADF41513_REG4_MOD2_MSK;
	st->regs[ADF41513_REG4] |= ADF41513_REG4_MOD2(st->settings.mod2);

	/* r-cnt | doubler | rdiv2 | prescaler */
	st->regs[ADF41513_REG5] &= ~(ADF41513_REG5_R_CNT_MSK |
				     ADF41513_REG5_REF_DOUBLER_MSK |
				     ADF41513_REG5_RDIV2_MSK |
				     ADF41513_REG5_PRESCALER_MSK);
	st->regs[ADF41513_REG5] |= ADF41513_REG5_R_CNT(st->settings.r_counter % 32) |
				   ADF41513_REG5_REF_DOUBLER(st->settings.ref_doubler) |
				   ADF41513_REG5_RDIV2(st->settings.ref_div2) |
				   ADF41513_REG5_PRESCALER(st->settings.prescaler);

	/* Enable integer mode if no fractional part */
	if (st->settings.mode == ADF41513_MODE_INTEGER_N) {
		/* integer mode */
		st->regs[ADF41513_REG6] |= ADF41513_REG6_INT_MODE_MSK;
		st->regs[ADF41513_REG6] &= ~ADF41513_REG6_BLEED_ENABLE_MSK;
	} else {
		/* fractional mode */
		st->regs[ADF41513_REG6] &= ~ADF41513_REG6_INT_MODE_MSK;
		st->regs[ADF41513_REG6] |= ADF41513_REG6_BLEED_ENABLE_MSK;
	}

	/* set bleed current value */
	if (st->data.phase_detector_polarity)
		bleed_value = 90;
	else
		bleed_value = 144;

	bleed_value *= FIELD_GET(ADF41513_REG5_CP_CURRENT_MSK, st->regs[ADF41513_REG5]) + 1;
	bleed_value = mul_u64_u64_div_u64(st->settings.pfd_frequency_uhz,
					  (u64)bleed_value,
					  16 * 100000000000000ULL /* 100 MHz in uHz */);

	st->regs[ADF41513_REG6] &= ~ADF41513_REG6_BLEED_CURRENT_MSK;
	st->regs[ADF41513_REG6] |= ADF41513_REG6_BLEED_CURRENT(bleed_value);

	return adf41513_sync_config(st, sync_mask | ADF41513_SYNC_REG0);
}

static int adf41513_suspend(struct adf41513_state *st)
{
	st->regs[ADF41513_REG6] |= ADF41513_REG6_POWER_DOWN(1);
	if (st->data.load_enable_sync)
		st->regs[ADF41513_REG12] &= ~ADF41513_REG12_LE_SELECT_MSK;

	return adf41513_sync_config(st, ADF41513_SYNC_DIFF);
}

static int adf41513_resume(struct adf41513_state *st)
{
	int ret;

	st->regs[ADF41513_REG6] &= ~ADF41513_REG6_POWER_DOWN_MSK;
	ret = adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	if (ret < 0)
		return ret;

	/* check if load enable sync needs to be set */
	if (st->data.load_enable_sync) {
		st->regs[ADF41513_REG12] |= ADF41513_REG12_LE_SELECT_MSK;
		ret = adf41513_sync_config(st, ADF41513_SYNC_DIFF);
	}

	return ret;
}

static ssize_t adf41513_read(struct iio_dev *indio_dev,
			     uintptr_t private,
			     const struct iio_chan_spec *chan,
			     char *buf)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	u64 val = 0;
	int ret = 0;

	guard(mutex)(&st->lock);

	switch ((u32)private) {
	case ADF41513_FREQ:
		ret = adf41513_uhz_to_str(adf41513_pll_get_rate(st), buf);
		if (st->lock_detect)
			if (!gpiod_get_value_cansleep(st->lock_detect)) {
				dev_dbg(&st->spi->dev, "PLL un-locked\n");
				ret = -EBUSY;
			}
		break;
	case ADF41513_FREQ_REFIN:
		st->ref_freq_hz = clk_get_rate(st->ref_clk);
		val = st->ref_freq_hz;
		ret = sysfs_emit(buf, "%llu\n", val);
		break;
	case ADF41513_FREQ_RESOLUTION:
		ret = adf41513_uhz_to_str(st->data.freq_resolution_uhz, buf);
		break;
	case ADF41513_POWER_DOWN:
		val = (u64)FIELD_GET(ADF41513_REG6_POWER_DOWN_MSK, st->regs_hw[ADF41513_REG6]);
		ret = sysfs_emit(buf, "%llu\n", val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static ssize_t adf41513_write(struct iio_dev *indio_dev,
			      uintptr_t private,
			      const struct iio_chan_spec *chan,
			      const char *buf, size_t len)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	unsigned long long readin;
	unsigned long tmp;
	u64 freq_uhz;
	int ret;

	guard(mutex)(&st->lock);

	switch ((u32)private) {
	case ADF41513_FREQ:
		ret = adf41513_parse_uhz(buf, &freq_uhz);
		if (ret)
			return ret;
		ret = adf41513_set_frequency(st, freq_uhz, ADF41513_SYNC_DIFF);
		break;
	case ADF41513_FREQ_REFIN:
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		if (readin < ADF41513_MIN_REF_FREQ || readin > ADF41513_MAX_REF_FREQ) {
			ret = -EINVAL;
			break;
		}

		tmp = clk_round_rate(st->ref_clk, readin);
		if (tmp != readin) {
			ret = -EINVAL;
			break;
		}
		ret = clk_set_rate(st->ref_clk, tmp);
		if (ret < 0)
			break;

		st->ref_freq_hz = readin;
		/* update RF out */
		ret = adf41513_set_frequency(st,
					     st->settings.target_frequency_uhz,
					     ADF41513_SYNC_DIFF);
		break;
	case ADF41513_FREQ_RESOLUTION:
		ret = adf41513_parse_uhz(buf, &freq_uhz);
		if (ret)
			return ret;
		if (freq_uhz == 0)
			ret = -EINVAL;
		else
			st->data.freq_resolution_uhz = freq_uhz;
		break;
	case ADF41513_POWER_DOWN:
		ret = kstrtoull(buf, 10, &readin);
		if (ret)
			return ret;

		if (readin)
			ret = adf41513_suspend(st);
		else
			ret = adf41513_resume(st);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret ? ret : len;
}

#define _ADF41513_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = adf41513_read, \
	.write = adf41513_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info adf41513_ext_info[] = {
	/*
	 * Ideally we use IIO_CHAN_INFO_FREQUENCY, but there are
	 * values > 2^32 with sub-Hz resolution
	 */
	_ADF41513_EXT_INFO("frequency", ADF41513_FREQ),
	_ADF41513_EXT_INFO("frequency_resolution", ADF41513_FREQ_RESOLUTION),
	_ADF41513_EXT_INFO("refin_frequency", ADF41513_FREQ_REFIN),
	_ADF41513_EXT_INFO("powerdown", ADF41513_POWER_DOWN),
	{ },
};

static const struct iio_chan_spec adf41513_chan = {
	.type = IIO_ALTVOLTAGE,
	.indexed = 1,
	.output = 1,
	.channel = 0,
	.ext_info = adf41513_ext_info,
};

static int adf41513_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int writeval,
			       unsigned int *readval)
{
	struct adf41513_state *st = iio_priv(indio_dev);
	int ret;

	if (reg > ADF41513_REG13)
		return -EINVAL;

	guard(mutex)(&st->lock);

	if (!readval) {
		/* direct register access invalidates cached pll settings */
		st->settings.mode = ADF41513_MODE_INVALID;

		st->regs[reg] = writeval & ~0xF; /* Clear control bits */
		ret = adf41513_sync_config(st, 1 << reg);
	} else {
		*readval = st->regs_hw[reg];
		ret = 0;
	}

	return ret;
}

static const struct iio_info adf41513_info = {
	.debugfs_reg_access = &adf41513_reg_access,
};

static void adf41513_clk_del_provider(void *data)
{
	struct adf41513_state *st = data;

	of_clk_del_provider(st->spi->dev.of_node);
}

static unsigned long adf41513_clk_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	return (unsigned long)div_u64(adf41513_pll_get_rate(st), MICROHZ_PER_HZ);
}

static long adf41513_clk_round_rate(struct clk_hw *hw,
				    unsigned long rate,
				    unsigned long *parent_rate)
{
	return rate;
}

static int adf41513_clk_set_rate(struct clk_hw *hw,
				 unsigned long rate,
				 unsigned long parent_rate)
{
	struct adf41513_state *st = to_adf41513_state(hw);
	u64 freq_uhz = (u64)rate * MICROHZ_PER_HZ;

	if (parent_rate < ADF41513_MIN_REF_FREQ ||
	    parent_rate > ADF41513_MAX_REF_FREQ)
		return -EINVAL;

	guard(mutex)(&st->lock);
	st->ref_freq_hz = parent_rate;
	return adf41513_set_frequency(st, freq_uhz, ADF41513_SYNC_DIFF);
}

static int adf41513_clk_prepare(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	return adf41513_resume(st);
}

static void adf41513_clk_unprepare(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	adf41513_suspend(st);
}

static int adf41513_clk_is_enabled(struct clk_hw *hw)
{
	struct adf41513_state *st = to_adf41513_state(hw);

	guard(mutex)(&st->lock);
	return (st->regs_hw[ADF41513_REG6] & ADF41513_REG6_POWER_DOWN_MSK) == 0;
}

static const struct clk_ops adf41513_clk_ops = {
	.recalc_rate = adf41513_clk_recalc_rate,
	.round_rate = adf41513_clk_round_rate,
	.set_rate = adf41513_clk_set_rate,
	.prepare = adf41513_clk_prepare,
	.unprepare = adf41513_clk_unprepare,
	.is_enabled = adf41513_clk_is_enabled,
};

static int adf41513_clk_register(struct adf41513_state *st)
{
	struct spi_device *spi = st->spi;
	struct clk_init_data init;
	struct clk *clk = NULL;
	const char *parent_name;
	int ret;

	if (!device_property_present(&spi->dev, "#clock-cells"))
		return 0;

	if (device_property_read_string(&spi->dev, "clock-output-names", &init.name)) {
		init.name = devm_kasprintf(&spi->dev, GFP_KERNEL, "%s-clk",
					   fwnode_get_name(dev_fwnode(&spi->dev)));
		if (!init.name)
			return -ENOMEM;
	}

	parent_name = of_clk_get_parent_name(spi->dev.of_node, 0);
	if (!parent_name)
		return -EINVAL;

	init.ops = &adf41513_clk_ops;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.flags = CLK_SET_RATE_PARENT;

	st->clk_hw.init = &init;
	clk = devm_clk_register(&spi->dev, &st->clk_hw);
	if (IS_ERR(clk))
		return PTR_ERR(clk);

	ret = of_clk_add_provider(spi->dev.of_node, of_clk_src_simple_get, clk);
	if (ret)
		return ret;

	st->clk_out = clk;

	return devm_add_action_or_reset(&spi->dev, adf41513_clk_del_provider, st);
}

static int adf41513_parse_fw(struct adf41513_state *st)
{
	struct device *dev = &st->spi->dev;

	int ret;
	u32 tmp;
	u32 cp_resistance;
	u32 cp_current;

	/* power-up frequency */
	st->data.power_up_frequency_hz = ADF41510_MAX_RF_FREQ; /* Default 10 GHz */
	if (!device_property_read_u64(dev, "adi,power-up-frequency",
				      &st->data.power_up_frequency_hz)) {
		if (st->data.power_up_frequency_hz < ADF41513_MIN_RF_FREQ ||
		    st->data.power_up_frequency_hz > ADF41513_MAX_RF_FREQ)
			return dev_err_probe(dev, -ERANGE,
					     "power-up frequency %llu Hz out of range\n",
					     st->data.power_up_frequency_hz);
	}

	/* reference divider factor - optional minimal value */
	st->data.ref_div_factor = ADF41513_MIN_R_CNT; /* Default R = 1 */
	if (!device_property_read_u32(dev, "adi,reference-div-factor", &tmp)) {
		if (tmp >= ADF41513_MIN_R_CNT && tmp <= ADF41513_MAX_R_CNT)
			st->data.ref_div_factor = tmp;
		else
			return dev_err_probe(dev, -ERANGE,
					     "invalid reference div factor %u\n", tmp);
	}

	/* reference controls */
	st->data.ref_doubler_en = device_property_read_bool(dev, "adi,reference-doubler-enable");
	st->data.ref_div2_en = device_property_read_bool(dev, "adi,reference-div2-enable");

	/* charge pump resistor */
	cp_resistance = 2700; /* Default 2.7 kOhms */
	if (!device_property_read_u32(dev, "adi,charge-pump-resistor-ohms", &tmp)) {
		if (tmp >= 1800 && tmp <= 10000)
			cp_resistance = tmp;
		else
			return dev_err_probe(dev, -ERANGE, "RSET %u Ohms out of range\n", tmp);
	}

	/* charge pump current */
	st->data.charge_pump_voltage_mv = 6480; /* Default 2.7 kOhm * 2.4 mA = 6.48 V */
	if (!device_property_read_u32(dev, "adi,charge-pump-current-microamp", &cp_current)) {
		tmp = (cp_current * cp_resistance) / 1000; /* Convert to mV */
		 /* Validate charge pump voltage */
		if (tmp >= ADF41513_MIN_CP_VOLTAGE_mV && tmp <= ADF41513_MAX_CP_VOLTAGE_mV)
			st->data.charge_pump_voltage_mv = tmp;
		else
			return dev_err_probe(dev, -ERANGE,
				"charge pump current %u uA (R_set = %u Ohms) out of range\n",
				cp_current, cp_resistance);
	}

	/* phase detector polarity */
	st->data.phase_detector_polarity =
		device_property_read_bool(dev, "adi,phase-detector-polarity-positive-enable");

	/* muxout selection */
	ret = device_property_match_property_string(dev, "adi,muxout-select",
						    adf41513_muxout_modes,
						    ARRAY_SIZE(adf41513_muxout_modes));
	if (ret >= 0)
		st->data.muxout_select = ret;
	else
		st->data.muxout_select = ADF41513_MUXOUT_TRISTATE;

	/* muxout logic level: default 3v3 */
	st->data.muxout_1v8_en = device_property_read_bool(dev, "adi,muxout-level-1v8-enable");

	/* lock detector settings */
	st->data.lock_detect_bias = 0; /* default 40 uA */
	if (!device_property_read_u32(dev, "lock-detect-bias-microamp", &tmp)) {
		tmp = (ADF41513_MAX_LD_BIAS_uA - tmp) / ADF41513_LD_BIAS_STEP_uA;
		if (tmp > 3)
			return dev_err_probe(dev, -ERANGE,
					     "invalid lock detect bias setting: %u\n", tmp);
		else
			st->data.lock_detect_bias = tmp;
	}

	st->data.lock_detect_precision = 0; /* default 0 */
	if (!device_property_read_u32(dev, "adi,lock-detect-precision", &tmp)) {
		if (tmp > 3)
			return dev_err_probe(dev, -ERANGE,
					     "invalid lock detect precision: %u\n", tmp);
		else
			st->data.lock_detect_precision = tmp;
	}

	st->data.lock_detect_count = 0; /* default 0 */
	if (!device_property_read_u32(dev, "adi,lock-detect-count", &tmp)) {
		if (tmp > 7)
			return dev_err_probe(dev, -ERANGE,
					     "lock detect count setting too high %u\n", tmp);
		else
			st->data.lock_detect_count = tmp;
	}

	/* lock detect clk select */
	st->data.fast_lock_en = device_property_read_bool(dev, "adi,fast-lock-enable");

	/* phase resync configuration */
	st->data.phase_resync_en = device_property_read_bool(dev, "adi,phase-resync-enable");

	st->data.phase_resync_clk_div[0] = 1;
	if (!device_property_read_u32(dev, "adi,12bit-clk-divider", &tmp)) {
		if (tmp > ADF41513_MAX_CLK_DIVIDER)
			return dev_err_probe(dev, -ERANGE, "invalid clk1 divider: %u\n", tmp);
		st->data.phase_resync_clk_div[0] = tmp;
	}

	st->data.phase_resync_clk_div[1] = 1;
	if (!device_property_read_u32(dev, "adi,12bit-clk2-divider", &tmp)) {
		if (tmp > ADF41513_MAX_CLK_DIVIDER)
			return dev_err_probe(dev, -ERANGE, "invalid clk2 divider: %u\n", tmp);
		st->data.phase_resync_clk_div[1] = tmp;
	}

	/* load enable sync with ref input */
	st->data.load_enable_sync = device_property_read_bool(dev, "adi,le-sync-enable");

	/* initial frequency resolution: Default to 1 Hz */
	st->data.freq_resolution_uhz = MICROHZ_PER_HZ;
	if (!device_property_read_u64(dev, "adi,freq-resolution",
				      &st->data.freq_resolution_uhz)) {
		if (!st->data.freq_resolution_uhz)
			return dev_err_probe(dev, -ERANGE, "frequency resolution cannot be zero\n");
	}

	return 0;
}

static int adf41513_setup(struct adf41513_state *st)
{
	int ret;
	u32 cp_index;

	memset(st->regs_hw, 0xFF, sizeof(st->regs_hw));

	/* assume DLD pin is used for digital lock detect */
	st->regs[ADF41513_REG5] |= ADF41513_REG5_DLD_MODES(ADF41513_DLD_DIG_LD);

	/* configure charge pump current settings */
	cp_index = find_closest(st->data.charge_pump_voltage_mv,
				adf41513_cp_voltage_mv,
				ARRAY_SIZE(adf41513_cp_voltage_mv));
	st->regs[ADF41513_REG5] |= ADF41513_REG5_CP_CURRENT(cp_index);

	/* Configure phase detector polarity */
	if (st->data.phase_detector_polarity)
		st->regs[ADF41513_REG6] |= ADF41513_REG6_PD_POLARITY_MSK;

	/* narrow ABP | loss of lock detect enable | SD reset | LDP from data */
	st->regs[ADF41513_REG6] |= ADF41513_REG6_ABP_MSK |
				   ADF41513_REG6_LOL_ENABLE_MSK |
				   ADF41513_REG6_SD_RESET_MSK |
				   ADF41513_REG6_LDP(st->data.lock_detect_precision);

	/* LD count from data | PS bias */
	st->regs[ADF41513_REG7] |= ADF41513_REG7_LD_COUNT(st->data.lock_detect_count) |
				   ADF41513_REG7_LD_CLK_SEL(st->data.fast_lock_en) |
				   ADF41513_REG7_PS_BIAS(2);

	/* enable phase resync and configure the clk divs */
	st->regs[ADF41513_REG5] |= ADF41513_REG5_CLK1_DIV(st->data.phase_resync_clk_div[0]);
	st->regs[ADF41513_REG7] |= ADF41513_REG7_CLK_DIV_MODE(st->data.phase_resync_en ? 2 : 0) |
				   ADF41513_REG7_CLK2_DIV(st->data.phase_resync_clk_div[1]);

	/* lock detect bias */
	st->regs[ADF41513_REG9] |= ADF41513_REG9_LD_BIAS(st->data.lock_detect_bias);

	/* power down select */
	st->regs[ADF41513_REG11] |= ADF41513_REG11_POWER_DOWN_SEL_MSK;

	/* muxout */
	st->regs[ADF41513_REG12] |= ADF41513_REG12_MUXOUT(st->data.muxout_select) |
				    ADF41513_REG12_LOGIC_LEVEL(st->data.muxout_1v8_en ? 0 : 1) |
				    ADF41513_REG12_LE_SELECT(st->data.phase_resync_en);

	/* set power-up frequency */
	ret = adf41513_set_frequency(st, (u64)st->data.power_up_frequency_hz * MICROHZ_PER_HZ,
				     ADF41513_SYNC_ALL);
	if (ret < 0) {
		return dev_err_probe(&st->spi->dev, ret,
				     "Failed to set power-up frequency: %d\n", ret);
	}

	return 0;
}

static void adf41513_power_down(void *data)
{
	struct adf41513_state *st = data;

	guard(mutex)(&st->lock);
	adf41513_suspend(st);

	if (st->chip_enable)
		gpiod_set_value_cansleep(st->chip_enable, 0);
}

static int adf41513_pm_suspend(struct device *dev)
{
	struct adf41513_state *st = dev_get_drvdata(dev);

	return adf41513_suspend(st);
}

static int adf41513_pm_resume(struct device *dev)
{
	struct adf41513_state *st = dev_get_drvdata(dev);

	return adf41513_resume(st);
}

static const struct adf41513_chip_info adf41513_chip_info = {
	.name = "adf41513",
	.has_prescaler_8_9 = true,
	.max_rf_freq_hz = ADF41513_MAX_RF_FREQ,
};

static const struct adf41513_chip_info adf41510_chip_info = {
	.name = "adf41510",
	.has_prescaler_8_9 = false,
	.max_rf_freq_hz = ADF41510_MAX_RF_FREQ,
};

static int adf41513_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adf41513_state *st;

	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = spi_get_device_match_data(spi);

	/* get reference clock */
	st->ref_clk = devm_clk_get_enabled(&spi->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return -EPROBE_DEFER;

	/* parse device properties */
	ret = adf41513_parse_fw(st);
	if (ret)
		return ret;

	spi_set_drvdata(spi, st);

	/* vcc regulator */
	ret = devm_regulator_get_enable(&spi->dev, "vcc");
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "failed to get and enable vcc regulator\n");

	/* get chip enable gpio */
	st->chip_enable = devm_gpiod_get_optional(&spi->dev, "chip-enable", GPIOD_OUT_HIGH);
	if (IS_ERR(st->chip_enable))
		return dev_err_probe(&spi->dev, PTR_ERR(st->chip_enable),
				     "fail to request chip enable GPIO\n");

	/* get lock detect gpio */
	st->lock_detect = devm_gpiod_get_optional(&spi->dev, "lock-detect", GPIOD_IN);
	if (IS_ERR(st->lock_detect))
		return dev_err_probe(&spi->dev, PTR_ERR(st->lock_detect),
				     "fail to request lock detect GPIO\n");

	/* get reference frequency */
	st->ref_freq_hz = clk_get_rate(st->ref_clk);

	/* validate reference frequency */
	if (st->ref_freq_hz < ADF41513_MIN_REF_FREQ || st->ref_freq_hz > ADF41513_MAX_REF_FREQ)
		return dev_err_probe(&spi->dev, -ERANGE,
				     "reference frequency %llu Hz out of range\n",
				     st->ref_freq_hz);

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to initialize mutex: %d\n", ret);

	ret = adf41513_clk_register(st);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret, "failed to register clock: %d\n", ret);

	/* configure IIO device */
	indio_dev->name = st->chip_info->name;
	indio_dev->info = &adf41513_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = &adf41513_chan;
	indio_dev->num_channels = 1;

	/* initialize device */
	ret = adf41513_setup(st);
	if (ret < 0)
		return ret;

	/* add power down action */
	ret = devm_add_action_or_reset(&spi->dev, adf41513_power_down, st);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to add power down action: %d\n", ret);

	/* register IIO device */
	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to register IIO device: %d\n", ret);

	dev_info(&spi->dev, "ADF41513 PLL synthesizer registered\n");

	return 0;
}

static const struct spi_device_id adf41513_id[] = {
	{"adf41510", (kernel_ulong_t)&adf41510_chip_info},
	{"adf41513", (kernel_ulong_t)&adf41513_chip_info},
	{}
};
MODULE_DEVICE_TABLE(spi, adf41513_id);

static const struct of_device_id adf41513_of_match[] = {
	{ .compatible = "adi,adf41510", .data = &adf41510_chip_info },
	{ .compatible = "adi,adf41513", .data = &adf41513_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, adf41513_of_match);

DEFINE_SIMPLE_DEV_PM_OPS(adf41513_pm_ops,
			 adf41513_pm_suspend,
			 adf41513_pm_resume);

static struct spi_driver adf41513_driver = {
	.driver = {
		.name = "adf41513",
		.pm = pm_ptr(&adf41513_pm_ops),
		.of_match_table = adf41513_of_match,
	},
	.probe = adf41513_probe,
	.id_table = adf41513_id,
};
module_spi_driver(adf41513_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADF41513 PLL Frequency Synthesizer");
MODULE_LICENSE("GPL");
