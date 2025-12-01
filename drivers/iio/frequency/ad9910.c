// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 SPI DDS (Direct Digital Synthesizer) driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/units.h>

/* Register addresses */
#define AD9910_REG_CFR1			0x00
#define AD9910_REG_CFR2			0x01
#define AD9910_REG_CFR3			0x02
#define AD9910_REG_AUX_DAC		0x03
#define AD9910_REG_IO_UPDATE_RATE	0x04
#define AD9910_REG_FTW			0x07
#define AD9910_REG_POW			0x08
#define AD9910_REG_ASF			0x09
#define AD9910_REG_MULTICHIP_SYNC	0x0A
#define AD9910_REG_DRG_LIMIT		0x0B
#define AD9910_REG_DRG_STEP		0x0C
#define AD9910_REG_DRG_RATE		0x0D
#define AD9910_REG_PROFILE0		0x0E
#define AD9910_REG_PROFILE1		0x0F
#define AD9910_REG_PROFILE2		0x10
#define AD9910_REG_PROFILE3		0x11
#define AD9910_REG_PROFILE4		0x12
#define AD9910_REG_PROFILE5		0x13
#define AD9910_REG_PROFILE6		0x14
#define AD9910_REG_PROFILE7		0x15
#define AD9910_REG_RAM			0x16

#define AD9910_REG_NUM_CACHED		0x16

#define AD9910_REG_PROFILE(x)		(AD9910_REG_PROFILE0 + (x))
#define AD9910_REG_HIGH32		0x100

/* CFR1 bit definitions */
#define AD9910_CFR1_RAM_ENABLE_MSK		BIT(31)
#define AD9910_CFR1_RAM_PLAYBACK_DEST_MSK	GENMASK(30, 29)
#define AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK	BIT(23)
#define AD9910_CFR1_INV_SINC_EN_MSK		BIT(22)
#define AD9910_CFR1_INT_PROFILE_CTL_MSK		GENMASK(20, 17)
#define AD9910_CFR1_SELECT_SINE_MSK		BIT(16)
#define AD9910_CFR1_LOAD_LRR_IO_UPDATE_MSK	BIT(15)
#define AD9910_CFR1_AUTOCLR_DIG_RAMP_ACCUM_MSK	BIT(14)
#define AD9910_CFR1_AUTOCLR_PHASE_ACCUM_MSK	BIT(13)
#define AD9910_CFR1_CLEAR_DIG_RAMP_ACCUM_MSK	BIT(12)
#define AD9910_CFR1_CLEAR_PHASE_ACCUM_MSK	BIT(11)
#define AD9910_CFR1_LOAD_ARR_IO_UPDATE_MSK	BIT(10)
#define AD9910_CFR1_OSK_ENABLE_MSK		BIT(9)
#define AD9910_CFR1_SELECT_AUTO_OSK_MSK		BIT(8)
#define AD9910_CFR1_DIGITAL_POWER_DOWN_MSK	BIT(7)
#define AD9910_CFR1_DAC_POWER_DOWN_MSK		BIT(6)
#define AD9910_CFR1_REFCLK_INPUT_POWER_DOWN_MSK	BIT(5)
#define AD9910_CFR1_AUX_DAC_POWER_DOWN_MSK	BIT(4)
#define AD9910_CFR1_SOFT_POWER_DOWN_MSK		GENMASK(7, 4)
#define AD9910_CFR1_EXT_POWER_DOWN_CTL_MSK	BIT(3)
#define AD9910_CFR1_SDIO_INPUT_ONLY_MSK		BIT(1)
#define AD9910_CFR1_LSB_FIRST_MSK		BIT(0)

/* CFR2 bit definitions */
#define AD9910_CFR2_AMP_SCALE_SINGLE_TONE_MSK	BIT(24)
#define AD9910_CFR2_INTERNAL_IO_UPDATE_MSK	BIT(23)
#define AD9910_CFR2_SYNC_CLK_EN_MSK		BIT(22)
#define AD9910_CFR2_DRG_DEST_MSK		GENMASK(21, 20)
#define AD9910_CFR2_DRG_ENABLE_MSK		BIT(19)
#define AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK	BIT(18)
#define AD9910_CFR2_DRG_NO_DWELL_LOW_MSK	BIT(17)
#define AD9910_CFR2_READ_EFFECTIVE_FTW_MSK	BIT(16)
#define AD9910_CFR2_IO_UPDATE_RATE_CTL_MSK	GENMASK(15, 14)
#define AD9910_CFR2_PDCLK_ENABLE_MSK		BIT(11)
#define AD9910_CFR2_PDCLK_INVERT_MSK		BIT(10)
#define AD9910_CFR2_TXENABLE_INVERT_MSK		BIT(9)
#define AD9910_CFR2_MATCHED_LATENCY_EN_MSK	BIT(7)
#define AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK	BIT(6)
#define AD9910_CFR2_SYNC_TIMING_VAL_DISABLE_MSK	BIT(5)
#define AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK	BIT(4)
#define AD9910_CFR2_FM_GAIN_MSK			GENMASK(3, 0)

/* CFR3 bit definitions */
#define AD9910_CFR3_OPEN_MSK			0x08070000
#define AD9910_CFR3_DRV0_MSK			GENMASK(29, 28)
#define AD9910_CFR3_VCO_SEL_MSK			GENMASK(26, 24)
#define AD9910_CFR3_ICP_MSK			GENMASK(21, 19)
#define AD9910_CFR3_REFCLK_DIV_BYPASS_MSK	BIT(15)
#define AD9910_CFR3_REFCLK_DIV_RESETB_MSK	BIT(14)
#define AD9910_CFR3_PFD_RESET_MSK		BIT(10)
#define AD9910_CFR3_PLL_EN_MSK			BIT(8)
#define AD9910_CFR3_N_MSK			GENMASK(7, 1)

/* Auxiliary DAC Control Register Bits */
#define AD9910_AUX_DAC_FSC_MSK			GENMASK(7, 0)

/* ASF Register Bits */
#define AD9910_ASF_AMP_RAMP_RATE_MSK		GENMASK(31, 16)
#define AD9910_ASF_AMP_SCALE_FACTOR_MSK		GENMASK(15, 2)
#define AD9910_ASF_AMP_STEP_SIZE_MSK		GENMASK(1, 0)

/* Multichip Sync Register Bits */
#define AD9910_MC_SYNC_VALIDATION_DELAY_MSK	GENMASK(31, 28)
#define AD9910_MC_SYNC_RECEIVER_ENABLE_MSK	BIT(27)
#define AD9910_MC_SYNC_GENERATOR_ENABLE_MSK	BIT(26)
#define AD9910_MC_SYNC_GENERATOR_POLARITY_MSK	BIT(25)
#define AD9910_MC_SYNC_STATE_PRESET_MSK		GENMASK(23, 18)
#define AD9910_MC_SYNC_OUTPUT_DELAY_MSK		GENMASK(15, 11)
#define AD9910_MC_SYNC_INPUT_DELAY_MSK		GENMASK(7, 3)

/* Digital Ramp Limit Register */
#define AD9910_DRG_LIMIT_UPPER_MSK		GENMASK_ULL(63, 32)
#define AD9910_DRG_LIMIT_LOWER_MSK		GENMASK_ULL(31, 0)

/* Digital Ramp Step Register */
#define AD9910_DRG_STEP_DEC_MSK			GENMASK_ULL(63, 32)
#define AD9910_DRG_STEP_INC_MSK			GENMASK_ULL(31, 0)

/* Digital Ramp Rate Register */
#define AD9910_DRG_RATE_DEC_MSK			GENMASK(31, 16)
#define AD9910_DRG_RATE_INC_MSK			GENMASK(15, 0)

/* Profile Register Format (Single Tone Mode) */
#define AD9910_PROFILE_ST_ASF_MSK		GENMASK_ULL(61, 48)
#define AD9910_PROFILE_ST_POW_MSK		GENMASK_ULL(47, 32)
#define AD9910_PROFILE_ST_FTW_MSK		GENMASK_ULL(31, 0)

/* Profile Register Format (RAM Mode) */
#define AD9910_PROFILE_RAM_OPEN_MSK		GENMASK_ULL(61, 57)
#define AD9910_PROFILE_RAM_STEP_RATE_MSK	GENMASK_ULL(55, 40)
#define AD9910_PROFILE_RAM_END_ADDR_MSK		GENMASK_ULL(39, 30)
#define AD9910_PROFILE_RAM_START_ADDR_MSK	GENMASK_ULL(23, 14)
#define AD9910_PROFILE_RAM_NO_DWELL_HIGH_MSK	BIT_ULL(5)
#define AD9910_PROFILE_RAM_ZERO_CROSSING_MSK	BIT_ULL(3)
#define AD9910_PROFILE_RAM_MODE_CONTROL_MSK	GENMASK_ULL(2, 0)

/* Device constants */
#define AD9910_MAX_SYSCLK_HZ		(1000UL * HZ_PER_MHZ)

#define AD9910_ASF_MAX			(BIT(14) - 1)
#define AD9910_STEP_RATE_MAX		(BIT(16) - 1)
#define AD9910_NUM_PROFILES		8

/* PLL constants */
#define AD9910_PLL_MIN_N		12
#define AD9910_PLL_MAX_N		127

#define AD9910_PLL_IN_MIN_FREQ_HZ	(3200UL * HZ_PER_KHZ)
#define AD9910_PLL_IN_MAX_FREQ_HZ	(60000UL * HZ_PER_KHZ)

#define AD9910_PLL_OUT_MIN_FREQ_HZ	(420UL * HZ_PER_MHZ)
#define AD9910_PLL_OUT_MAX_FREQ_HZ	(1000UL * HZ_PER_MHZ)

#define AD9910_VCO0_RANGE_AUTO_MAX_HZ	(465UL * HZ_PER_MHZ)
#define AD9910_VCO1_RANGE_AUTO_MAX_HZ	(545UL * HZ_PER_MHZ)
#define AD9910_VCO2_RANGE_AUTO_MAX_HZ	(650UL * HZ_PER_MHZ)
#define AD9910_VCO3_RANGE_AUTO_MAX_HZ	(790UL * HZ_PER_MHZ)
#define AD9910_VCO4_RANGE_AUTO_MAX_HZ	(885UL * HZ_PER_MHZ)
#define AD9910_VCO_RANGE_NUM		6

#define AD9910_ICP_MIN_uA		212
#define AD9910_ICP_MAX_uA		387

#define AD9910_DAC_IOUT_MAX_uA		31590
#define AD9910_DAC_IOUT_DEFAULT_uA	20070
#define AD9910_DAC_IOUT_MIN_uA		8640

#define AD9910_REFDIV2_MIN_FREQ_HZ	(120UL * HZ_PER_MHZ)
#define AD9910_REFDIV2_MAX_FREQ_HZ	(1900UL * HZ_PER_MHZ)

#define AD9910_DEST_FREQUENCY		0
#define AD9910_DEST_PHASE		1
#define AD9910_DEST_AMPLITUDE		2
#define AD9910_DEST_POLAR		3

#define AD9910_DRG_DEST_NUM		3
#define AD9910_RAM_DEST_NUM		4

#define AD9910_DRG_MODE_RAMP_DOWN	0x0
#define AD9910_DRG_MODE_RAMP_UP		0x1
#define AD9910_DRG_MODE_RAMP_BIDIR	0x2

#define AD9910_RAM_MODE_DIRECT_SWITCH	0x0
#define AD9910_RAM_MODE_RAMP_UP		0x1
#define AD9910_RAM_MODE_BIDIR		0x2
#define AD9910_RAM_MODE_BIDIR_CONT	0x3
#define AD9910_RAM_MODE_RAMP_UP_CONT	0x4
#define AD9910_RAM_MODE_SEQ		0x5
#define AD9910_RAM_MODE_SEQ_CONT	0x6

#define AD9910_RAM_SIZE_MAX		(1024 * 4)
#define AD9910_RAM_ADDR_MAX		1023

#define AD9910_CHANNEL_SINGLE_TONE	0
#define AD9910_CHANNEL_OSK		1
#define AD9910_CHANNEL_DRG		2
#define AD9910_CHANNEL_RAM		3
#define AD9910_CHANNEL_PARALLEL_PORT	4

#define AD9910_RAM_PROFILE_CTL_CONT_MSK		BIT(4)
#define AD9910_SPI_READ				BIT(7)
#define AD9910_SPI_ADDR_MASK			GENMASK(4, 0)

enum {
	AD9910_PROFILE,
	AD9910_POWERDOWN,
	AD9910_OSK_MANUAL_EXTCTL,
	AD9910_OSK_AUTO_STEP,
	AD9910_DRG_DWELL_HIGH_ENABLE,
	AD9910_DRG_DWELL_LOW_ENABLE,
	AD9910_DRG_FREQ_UPPER_LIMIT,
	AD9910_DRG_FREQ_LOWER_LIMIT,
	AD9910_DRG_FREQ_INC_STEP,
	AD9910_DRG_FREQ_DEC_STEP,
	AD9910_DRG_PHASE_UPPER_LIMIT,
	AD9910_DRG_PHASE_LOWER_LIMIT,
	AD9910_DRG_PHASE_INC_STEP,
	AD9910_DRG_PHASE_DEC_STEP,
	AD9910_DRG_AMP_UPPER_LIMIT,
	AD9910_DRG_AMP_LOWER_LIMIT,
	AD9910_DRG_AMP_INC_STEP,
	AD9910_DRG_AMP_DEC_STEP,
	AD9910_DRG_INC_STEP_RATE,
	AD9910_DRG_DEC_STEP_RATE,
	AD9910_RAM_START_ADDR,
	AD9910_RAM_END_ADDR,
};

struct ad9910_data {
	/* PLL configuration */
	u8 pll_multiplier;
	u8 pll_vco_range;
	u16 pll_charge_pump_current;

	bool ref_div2_en;
	u8 refclk_out_drv;

	/* Feature flags */
	bool inverse_sinc_enable;
	bool select_sine_output;
	bool sync_clk_enable;
	bool pdclk_enable;
	bool pdclk_invert;
	bool tx_enable_invert;

	/* DAC configuration */
	u32 dac_output_current;
};

struct ad9910_state {
	struct spi_device *spi;
	struct clk *refclk;

	struct gpio_desc *gpio_powerdown;
	struct gpio_desc *gpio_m_reset;
	struct gpio_desc *gpio_io_reset;
	struct gpio_desc *gpio_io_update;
	struct gpio_desc *gpio_profile[3];

	u8 profile;
	u32 sysclk_hz;
	struct ad9910_data data;

	/* cached registers */
	union {
		u64 val64;
		u32 val32;
		u16 val16;
	} reg[AD9910_REG_NUM_CACHED];

	/*
	 * alternate profile registers used to store RAM profile settings when
	 * RAM mode is disabled and Single Tone profile settings when RAM mode
	 * is enabled.
	 */
	u64 reg_profile[AD9910_NUM_PROFILES];

	/*
	 * Lock for accessing device registers and state variables.
	 */
	struct mutex lock;

	/*
	 * DMA (thus cache coherency maintenance) requires the transfer
	 * buffers to live in their own cache lines.
	 */
	union {
		__be64 be64;
		__be32 be32;
		__be16 be16;
	} buf __aligned(IIO_DMA_MINALIGN);
};

static const char * const ad9910_power_supplies[] = {
	"dvdd-io33", "avdd33", "dvdd18", "avdd18",
};

static const char * const ad9910_channel_str[] = {
	[AD9910_CHANNEL_SINGLE_TONE] = "single_tone",
	[AD9910_CHANNEL_OSK] = "output_shift_keying",
	[AD9910_CHANNEL_DRG] = "digital_ramp_generator",
	[AD9910_CHANNEL_RAM] = "ram_control",
	[AD9910_CHANNEL_PARALLEL_PORT] = "parallel_port",
};

static const char * const ad9910_destination_str[] = {
	[AD9910_DEST_FREQUENCY] = "frequency",
	[AD9910_DEST_PHASE] = "phase",
	[AD9910_DEST_AMPLITUDE] = "amplitude",
	[AD9910_DEST_POLAR] = "polar"
};

static const char * const ad9910_ram_oper_mode_str[] = {
	[AD9910_RAM_MODE_DIRECT_SWITCH] = "direct_switch",
	[AD9910_RAM_MODE_RAMP_UP] = "ramp_up",
	[AD9910_RAM_MODE_BIDIR] = "bidirectional",
	[AD9910_RAM_MODE_BIDIR_CONT] = "bidirectional_continuous",
	[AD9910_RAM_MODE_RAMP_UP_CONT] = "ramp_up_continuous",
	[AD9910_RAM_MODE_SEQ] = "sequenced",
	[AD9910_RAM_MODE_SEQ_CONT] = "sequenced_continuous",
};

static const u16 ad9910_charge_pump_currents[] = {
	AD9910_ICP_MIN_uA, 237, 262, 287, 312, 337, 363, AD9910_ICP_MAX_uA
};

/*
 * Perform scaling of input given a reference. Due to the lack of support under
 * /lib/math/div64.c for rounding mul_u64_u64_div_u64, we add a helper function
 * here that implements closest rounding.
 */
static u64 ad9910_rational_scale(u64 input, u64 scale, u64 reference)
{
	u64 output = mul_u64_u64_div_u64(input, scale, reference);

	/* multiplications might overflow, but it does not matter here */
	if ((input * scale - reference * output) > (reference >> 1))
		output++;

	return output;
}

static int ad9910_io_update(struct ad9910_state *st)
{
	if (st->gpio_io_update) {
		gpiod_set_value_cansleep(st->gpio_io_update, 1);
		udelay(1);
		gpiod_set_value_cansleep(st->gpio_io_update, 0);
	}

	return 0;
}

static int ad9910_spi_read(struct ad9910_state *st, u8 reg, void *data, size_t len)
{
	__aligned(IIO_DMA_MINALIGN) u8 inst = AD9910_SPI_READ | (reg & AD9910_SPI_ADDR_MASK);

	struct spi_transfer t[] = {
		{ .tx_buf = &inst, .len = 1, },
		{ .rx_buf = data, .len = len, },
	};

	return spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
}

static int ad9910_spi_write(struct ad9910_state *st, u8 reg, const void *data,
			    size_t len, bool update)
{
	__aligned(IIO_DMA_MINALIGN) u8 inst = reg & AD9910_SPI_ADDR_MASK;
	int ret;

	struct spi_transfer t[] = {
		{ .tx_buf = &inst, .len = 1, },
		{ .tx_buf = data, .len = len, },
	};

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (!ret && update)
		return ad9910_io_update(st);

	return ret;
}

static inline int ad9910_reg16_read(struct ad9910_state *st, u8 reg, u16 *data)
{
	int ret;

	ret = ad9910_spi_read(st, reg, &st->buf, sizeof(u16));
	if (ret < 0)
		return ret;

	*data = be16_to_cpu(st->buf.be16);
	return ret;
}

static inline int ad9910_reg16_write(struct ad9910_state *st, u8 reg, u16 data,
				     bool update)
{
	int ret;

	st->buf.be16 = cpu_to_be16(data);
	ret = ad9910_spi_write(st, reg, &st->buf, sizeof(u16), update);
	if (ret < 0)
		return ret;

	st->reg[reg].val16 = data;
	dev_dbg(&st->spi->dev, "REG[0x%02X] <= 0x%04X\n", reg, data);
	return ret;
}

static inline int ad9910_reg32_read(struct ad9910_state *st, u8 reg, u32 *data)
{
	int ret;

	ret = ad9910_spi_read(st, reg, &st->buf, sizeof(u32));
	if (ret < 0)
		return ret;

	*data = be32_to_cpu(st->buf.be32);
	return ret;
}

static inline int ad9910_reg32_write(struct ad9910_state *st, u8 reg, u32 data,
				     bool update)
{
	int ret;

	st->buf.be32 = cpu_to_be32(data);
	ret = ad9910_spi_write(st, reg, &st->buf, sizeof(u32), update);
	if (ret < 0)
		return ret;

	st->reg[reg].val32 = data;
	dev_dbg(&st->spi->dev, "REG[0x%02X] <= 0x%08X\n", reg, data);
	return ret;
}

static int ad9910_reg32_update(struct ad9910_state *st, u8 reg, u32 mask,
			       u32 data, bool update)
{
	u32 reg_val = (st->reg[reg].val32 & ~mask) | (data & mask);

	if (reg_val == st->reg[reg].val32 && !update)
		return 0;

	return ad9910_reg32_write(st, reg, reg_val, update);
}

static inline int ad9910_reg64_read(struct ad9910_state *st, u8 reg, u64 *data)
{
	int ret;

	ret = ad9910_spi_read(st, reg, &st->buf, sizeof(u64));
	if (ret < 0)
		return ret;

	*data = be64_to_cpu(st->buf.be64);
	return ret;
}

static inline int ad9910_reg64_write(struct ad9910_state *st, u8 reg, u64 data,
				     bool update)
{
	int ret;

	st->buf.be64 = cpu_to_be64(data);
	ret = ad9910_spi_write(st, reg, &st->buf, sizeof(st->buf), update);
	if (ret < 0)
		return ret;

	st->reg[reg].val64 = data;
	dev_dbg(&st->spi->dev, "REG[0x%02X] <= 0x%016llX\n", reg, data);
	return ret;
}

static int ad9910_reg64_update(struct ad9910_state *st, u8 reg, u64 mask,
			       u64 data, bool update)
{
	u64 reg_val = (st->reg[reg].val64 & ~mask) | (data & mask);

	if (reg_val == st->reg[reg].val64 && !update)
		return 0;

	return ad9910_reg64_write(st, reg, reg_val, update);
}

static int ad9910_set_profile(struct ad9910_state *st, u8 profile)
{
	if (profile >= AD9910_NUM_PROFILES)
		return -EINVAL;

	if (st->gpio_profile[0])
		gpiod_set_value_cansleep(st->gpio_profile[0], profile & 0x01);
	if (st->gpio_profile[1])
		gpiod_set_value_cansleep(st->gpio_profile[1], (profile >> 1) & 0x01);
	if (st->gpio_profile[2])
		gpiod_set_value_cansleep(st->gpio_profile[2], (profile >> 2) & 0x01);

	st->profile = profile;
	return 0;
}

static int ad9910_set_drg_destination(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return ad9910_reg32_update(st, AD9910_REG_CFR2,
				   AD9910_CFR2_DRG_DEST_MSK,
				   FIELD_PREP(AD9910_CFR2_DRG_DEST_MSK, val),
				   true);
}

static int ad9910_get_drg_destination(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return FIELD_GET(AD9910_CFR2_DRG_DEST_MSK,
			 st->reg[AD9910_REG_CFR2].val32);
}

static int ad9910_set_ram_destination(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return -EBUSY;

	return ad9910_reg32_update(st, AD9910_REG_CFR1,
				   AD9910_CFR1_RAM_PLAYBACK_DEST_MSK,
				   FIELD_PREP(AD9910_CFR1_RAM_PLAYBACK_DEST_MSK, val),
				   true);
}

static int ad9910_get_ram_destination(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return FIELD_GET(AD9910_CFR1_RAM_PLAYBACK_DEST_MSK,
			 st->reg[AD9910_REG_CFR1].val32);
}

static int ad9910_set_ram_oper_mode(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	u32 profile_target, profile_ctl = 0;
	int ret;

	guard(mutex)(&st->lock);

	/*
	 * Sequenced modes use the internal profile control.
	 * Active profile defines the internal profile control target, unless
	 * profile 0 is selected, in which case the target is profile 7.
	 */
	profile_target = st->profile ? st->profile : AD9910_NUM_PROFILES - 1;
	if (val == AD9910_RAM_MODE_SEQ)
		profile_ctl = profile_target;
	else if (val == AD9910_RAM_MODE_SEQ_CONT)
		profile_ctl = AD9910_RAM_PROFILE_CTL_CONT_MSK | (profile_target - 1);

	ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
				  AD9910_CFR1_INT_PROFILE_CTL_MSK,
				  FIELD_PREP(AD9910_CFR1_INT_PROFILE_CTL_MSK, profile_ctl),
				  true);
	if (ret || val >= AD9910_RAM_MODE_SEQ)
		return ret;

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
					   AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
					   FIELD_PREP(AD9910_PROFILE_RAM_MODE_CONTROL_MSK, val),
					   true);

	st->reg_profile[st->profile] &= ~AD9910_PROFILE_RAM_MODE_CONTROL_MSK;
	st->reg_profile[st->profile] |= FIELD_PREP(AD9910_PROFILE_RAM_MODE_CONTROL_MSK, val);

	return 0;
}

static int ad9910_get_ram_oper_mode(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	u32 profile_ctl;

	guard(mutex)(&st->lock);

	profile_ctl = FIELD_GET(AD9910_CFR1_INT_PROFILE_CTL_MSK,
				st->reg[AD9910_REG_CFR1].val32);
	if (profile_ctl) {
		if (profile_ctl & AD9910_RAM_PROFILE_CTL_CONT_MSK)
			return AD9910_RAM_MODE_SEQ_CONT;
		else
			return AD9910_RAM_MODE_SEQ;
	}

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return FIELD_GET(AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
				 st->reg[AD9910_REG_PROFILE(st->profile)].val64);
	else
		return FIELD_GET(AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
				 st->reg_profile[st->profile]);
}

static ssize_t ad9910_read_ext(struct iio_dev *indio_dev,
			       uintptr_t private,
			       const struct iio_chan_spec *chan,
			       char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PROFILE:
		val = st->profile;
		break;
	case AD9910_POWERDOWN:
		val = !!FIELD_GET(AD9910_CFR1_SOFT_POWER_DOWN_MSK,
				  st->reg[AD9910_REG_CFR1].val32);
		break;
	case AD9910_OSK_MANUAL_EXTCTL:
		val = !!FIELD_GET(AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK,
				  st->reg[AD9910_REG_CFR1].val32);
		break;
	case AD9910_OSK_AUTO_STEP:
		val = 0;
		if (!!FIELD_GET(AD9910_CFR1_SELECT_AUTO_OSK_MSK, st->reg[AD9910_REG_CFR1].val32))
			val = BIT(FIELD_GET(AD9910_ASF_AMP_STEP_SIZE_MSK,
					    st->reg[AD9910_REG_ASF].val32));
		break;
	case AD9910_DRG_DWELL_HIGH_ENABLE:
		val = !FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK,
				 st->reg[AD9910_REG_CFR2].val32);
		break;
	case AD9910_DRG_DWELL_LOW_ENABLE:
		val = !FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_LOW_MSK,
				 st->reg[AD9910_REG_CFR2].val32);
		break;
	case AD9910_RAM_START_ADDR:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			val = FIELD_GET(AD9910_PROFILE_RAM_START_ADDR_MSK,
					st->reg[AD9910_REG_PROFILE(st->profile)].val64);
		else
			val = FIELD_GET(AD9910_PROFILE_RAM_START_ADDR_MSK,
					st->reg_profile[st->profile]);
		break;
	case AD9910_RAM_END_ADDR:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			val = FIELD_GET(AD9910_PROFILE_RAM_END_ADDR_MSK,
					st->reg[AD9910_REG_PROFILE(st->profile)].val64);
		else
			val = FIELD_GET(AD9910_PROFILE_RAM_END_ADDR_MSK,
					st->reg_profile[st->profile]);
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, IIO_VAL_INT, 1, &val);
}

static ssize_t ad9910_read_step_rate(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int vals[2];
	u32 tmp32;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_INC_STEP_RATE:
		tmp32 = FIELD_GET(AD9910_DRG_RATE_INC_MSK,
				  st->reg[AD9910_REG_DRG_RATE].val32);
		break;
	case AD9910_DRG_DEC_STEP_RATE:
		tmp32 = FIELD_GET(AD9910_DRG_RATE_DEC_MSK,
				  st->reg[AD9910_REG_DRG_RATE].val32);
		break;
	default:
		return -EINVAL;
	}

	if (tmp32 == 0)
		return -ERANGE;

	tmp32 *= 4;
	vals[0] = st->sysclk_hz / tmp32;
	vals[1] = div_u64((u64)(st->sysclk_hz % tmp32) * MICRO, tmp32);

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_read_drg_attrs(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	unsigned int type;
	int vals[2];
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_FREQ_UPPER_LIMIT:
	case AD9910_DRG_PHASE_UPPER_LIMIT:
	case AD9910_DRG_AMP_UPPER_LIMIT:
		tmp64 = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK,
				  st->reg[AD9910_REG_DRG_LIMIT].val64);
		break;
	case AD9910_DRG_FREQ_LOWER_LIMIT:
	case AD9910_DRG_PHASE_LOWER_LIMIT:
	case AD9910_DRG_AMP_LOWER_LIMIT:
		tmp64 = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK,
				  st->reg[AD9910_REG_DRG_LIMIT].val64);
		break;
	case AD9910_DRG_FREQ_INC_STEP:
	case AD9910_DRG_PHASE_INC_STEP:
	case AD9910_DRG_AMP_INC_STEP:
		tmp64 = FIELD_GET(AD9910_DRG_STEP_INC_MSK,
				  st->reg[AD9910_REG_DRG_STEP].val64);
		break;
	case AD9910_DRG_FREQ_DEC_STEP:
	case AD9910_DRG_PHASE_DEC_STEP:
	case AD9910_DRG_AMP_DEC_STEP:
		tmp64 = FIELD_GET(AD9910_DRG_STEP_DEC_MSK,
				  st->reg[AD9910_REG_DRG_STEP].val64);
		break;
	default:
		return -EINVAL;
	}

	switch (private) {
	case AD9910_DRG_FREQ_UPPER_LIMIT:
	case AD9910_DRG_FREQ_LOWER_LIMIT:
	case AD9910_DRG_FREQ_INC_STEP:
	case AD9910_DRG_FREQ_DEC_STEP:
		type = IIO_VAL_INT_PLUS_MICRO;
		tmp64 *= st->sysclk_hz;
		vals[0] = tmp64 >> 32;
		vals[1] = ((tmp64 & 0xFFFFFFFFULL) * MICRO) >> 32;
		break;
	case AD9910_DRG_PHASE_UPPER_LIMIT:
	case AD9910_DRG_PHASE_LOWER_LIMIT:
	case AD9910_DRG_PHASE_INC_STEP:
	case AD9910_DRG_PHASE_DEC_STEP:
		type = IIO_VAL_INT_PLUS_MICRO;
		tmp64 *= 360;
		vals[0] = tmp64 >> 32;
		vals[1] = ((tmp64 & 0xFFFFFFFFULL) * MICRO) >> 32;
		break;
	case AD9910_DRG_AMP_UPPER_LIMIT:
	case AD9910_DRG_AMP_LOWER_LIMIT:
	case AD9910_DRG_AMP_INC_STEP:
	case AD9910_DRG_AMP_DEC_STEP:
		type = IIO_VAL_INT_PLUS_NANO;
		vals[0] = 0;
		vals[1] = tmp64 * NANO >> 32;
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, type, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_write_ext(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	u32 val32;
	int ret;

	ret = kstrtou32(buf, 10, &val32);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PROFILE:
		if (val32 > 7)
			return -EINVAL;
		ret = ad9910_set_profile(st, val32);
		break;
	case AD9910_POWERDOWN:
		val32 = val32 ? AD9910_CFR1_SOFT_POWER_DOWN_MSK : 0;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR1, AD9910_CFR1_SOFT_POWER_DOWN_MSK,
					  val32, true);
		break;
	case AD9910_OSK_MANUAL_EXTCTL:
		val32 = val32 ? AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK : 0;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR1, AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK,
					  val32, true);
		break;
	case AD9910_OSK_AUTO_STEP:
		if (val32 != 0 || val32 > 8 || !is_power_of_2(val32))
			return -EINVAL;

		if (val32) {
			val32 = ilog2(val32);
			ret = ad9910_reg32_update(st, AD9910_REG_ASF,
						  AD9910_ASF_AMP_STEP_SIZE_MSK,
						  FIELD_PREP(AD9910_ASF_AMP_STEP_SIZE_MSK, val32),
						  true);
			if (ret)
				return ret;

			val32 = AD9910_CFR1_SELECT_AUTO_OSK_MSK;
		}

		ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
					  AD9910_CFR1_SELECT_AUTO_OSK_MSK,
					  val32, true);
		break;
	case AD9910_DRG_DWELL_HIGH_ENABLE:
		val32 = val32 ? 0 : AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR2,
					  AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK,
					  val32, true);
		break;
	case AD9910_DRG_DWELL_LOW_ENABLE:
		val32 = val32 ? 0 : AD9910_CFR2_DRG_NO_DWELL_LOW_MSK;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR2,
					  AD9910_CFR2_DRG_NO_DWELL_LOW_MSK,
					  val32, true);
		break;
	case AD9910_RAM_START_ADDR:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			return -EBUSY;

		if (val32 > FIELD_GET(AD9910_PROFILE_RAM_END_ADDR_MSK,
				      st->reg_profile[st->profile]))
			return -EINVAL;

		st->reg_profile[st->profile] &= ~AD9910_PROFILE_RAM_START_ADDR_MSK;
		st->reg_profile[st->profile] |= FIELD_PREP(AD9910_PROFILE_RAM_START_ADDR_MSK,
							   val32);
		break;
	case AD9910_RAM_END_ADDR:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			return -EBUSY;

		if (val32 > AD9910_RAM_ADDR_MAX ||
		    val32 < FIELD_GET(AD9910_PROFILE_RAM_START_ADDR_MSK,
				      st->reg_profile[st->profile]))
			return -EINVAL;

		st->reg_profile[st->profile] &= ~AD9910_PROFILE_RAM_END_ADDR_MSK;
		st->reg_profile[st->profile] |= FIELD_PREP(AD9910_PROFILE_RAM_END_ADDR_MSK,
							   val32);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t ad9910_write_step_rate(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u64 rate_val;
	u64 sysclk_uhz;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
	if (ret)
		return ret;

	sysclk_uhz = (u64)st->sysclk_hz * MICROHZ_PER_HZ;
	rate_val = ((u64)val * MICROHZ_PER_HZ + val2) * 4;
	if (rate_val == 0 || rate_val > sysclk_uhz)
		return -EINVAL;

	rate_val = min(DIV64_U64_ROUND_CLOSEST(sysclk_uhz, rate_val),
		       AD9910_STEP_RATE_MAX);

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_INC_STEP_RATE:
		ret = ad9910_reg32_update(st, AD9910_REG_DRG_RATE,
					  AD9910_DRG_RATE_INC_MSK,
					  FIELD_PREP(AD9910_DRG_RATE_INC_MSK, rate_val),
					  true);
		break;
	case AD9910_DRG_DEC_STEP_RATE:
		ret = ad9910_reg32_update(st, AD9910_REG_DRG_RATE,
					  AD9910_DRG_RATE_DEC_MSK,
					  FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, rate_val), true);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t ad9910_write_drg_attrs(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u64 tmp64;
	int ret;

	switch (private) {
	case AD9910_DRG_FREQ_UPPER_LIMIT:
	case AD9910_DRG_FREQ_LOWER_LIMIT:
	case AD9910_DRG_FREQ_INC_STEP:
	case AD9910_DRG_FREQ_DEC_STEP:
		ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
		if (ret)
			return ret;

		if (val >= st->sysclk_hz / 2)
			return -EINVAL;

		tmp64 = ad9910_rational_scale((u64)val * MICRO + val2, BIT_ULL(32),
					      (u64)MICRO * st->sysclk_hz);
		break;
	case AD9910_DRG_PHASE_UPPER_LIMIT:
	case AD9910_DRG_PHASE_LOWER_LIMIT:
	case AD9910_DRG_PHASE_INC_STEP:
	case AD9910_DRG_PHASE_DEC_STEP:
		ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
		if (ret)
			return ret;

		val %= 360;
		if (val < 0)
			val += 360;

		tmp64 = ((u64)val * MICRO + val2) << 32;
		tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, MICRO * 360);
		break;
	case AD9910_DRG_AMP_UPPER_LIMIT:
	case AD9910_DRG_AMP_LOWER_LIMIT:
	case AD9910_DRG_AMP_INC_STEP:
	case AD9910_DRG_AMP_DEC_STEP:
		ret = iio_str_to_fixpoint(buf, 100000000, &val, &val2);
		if (ret)
			return ret;

		if (val < 0 || val > 1 || (val == 1 && val2 > 0))
			return -EINVAL;

		tmp64 = ((u64)val * NANO + val2) << 32;
		tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, NANO);
		break;
	default:
		return -EINVAL;
	}

	tmp64 = min(tmp64, U32_MAX);
	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_FREQ_UPPER_LIMIT:
	case AD9910_DRG_PHASE_UPPER_LIMIT:
	case AD9910_DRG_AMP_UPPER_LIMIT:
		ret = ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
					  AD9910_DRG_LIMIT_UPPER_MSK,
					  FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, tmp64),
					  true);
		break;
	case AD9910_DRG_FREQ_LOWER_LIMIT:
	case AD9910_DRG_PHASE_LOWER_LIMIT:
	case AD9910_DRG_AMP_LOWER_LIMIT:
		ret = ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
					  AD9910_DRG_LIMIT_LOWER_MSK,
					  FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, tmp64),
					  true);
		break;
	case AD9910_DRG_FREQ_INC_STEP:
	case AD9910_DRG_PHASE_INC_STEP:
	case AD9910_DRG_AMP_INC_STEP:
		ret = ad9910_reg64_update(st, AD9910_REG_DRG_STEP,
					  AD9910_DRG_STEP_INC_MSK,
					  FIELD_PREP(AD9910_DRG_STEP_INC_MSK, tmp64),
					  true);
		break;
	case AD9910_DRG_FREQ_DEC_STEP:
	case AD9910_DRG_PHASE_DEC_STEP:
	case AD9910_DRG_AMP_DEC_STEP:
		ret = ad9910_reg64_update(st, AD9910_REG_DRG_STEP,
					  AD9910_DRG_STEP_DEC_MSK,
					  FIELD_PREP(AD9910_DRG_STEP_DEC_MSK, tmp64),
					  true);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t ram_data_write(struct file *filp, struct kobject *kobj,
			      const struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct ad9910_state *st = iio_priv(dev_to_iio_dev(kobj_to_dev(kobj)));
	u64 tmp64, backup;
	int ret, ret2;

	if (off + count > AD9910_RAM_SIZE_MAX || !count ||
	    off % sizeof(u32) != 0 || count % sizeof(u32) != 0)
		return -EINVAL;

	guard(mutex)(&st->lock);

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return -EBUSY;

	/* backup profile register */
	backup = st->reg[AD9910_REG_PROFILE(st->profile)].val64;
	tmp64 = AD9910_PROFILE_RAM_STEP_RATE_MSK |
		FIELD_PREP(AD9910_PROFILE_RAM_START_ADDR_MSK, off / sizeof(u32)) |
		FIELD_PREP(AD9910_PROFILE_RAM_END_ADDR_MSK, (off + count) / sizeof(u32) - 1);
	ret = ad9910_reg64_write(st, AD9910_REG_PROFILE(st->profile), tmp64, true);
	if (ret)
		return ret;

	/* ensure profile is selected */
	ret = ad9910_set_profile(st, st->profile);
	if (ret)
		return ret;

	/* write ram data and restore profile register */
	ret = ad9910_spi_write(st, AD9910_REG_RAM, buf, count, true);
	ret2 = ad9910_reg64_write(st, AD9910_REG_PROFILE(st->profile), backup, true);
	if (!ret)
		ret = ret2;

	return ret ? ret : count;
}

static const BIN_ATTR_WO(ram_data, AD9910_RAM_SIZE_MAX);

static struct attribute *ad9910_attrs[] = {
	NULL
};

static const struct bin_attribute *const ad9910_bin_attrs[] = {
	&bin_attr_ram_data,
	NULL
};

static const struct attribute_group ad9910_attrs_group = {
	.attrs = ad9910_attrs,
	.bin_attrs = ad9910_bin_attrs,
};

static const struct iio_enum ad9910_drg_destination_enum = {
	.items = ad9910_destination_str,
	.num_items = AD9910_DRG_DEST_NUM,
	.set = ad9910_set_drg_destination,
	.get = ad9910_get_drg_destination,
};

static const struct iio_enum ad9910_ram_destination_enum = {
	.items = ad9910_destination_str,
	.num_items = AD9910_RAM_DEST_NUM,
	.set = ad9910_set_ram_destination,
	.get = ad9910_get_ram_destination,
};

static const struct iio_enum ad9910_ram_oper_mode_enum = {
	.items = ad9910_ram_oper_mode_str,
	.num_items = ARRAY_SIZE(ad9910_ram_oper_mode_str),
	.set = ad9910_set_ram_oper_mode,
	.get = ad9910_get_ram_oper_mode,
};

#define AD9910_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = ad9910_read_ext, \
	.write = ad9910_write_ext, \
	.private = _ident, \
	.shared = _shared, \
}

#define AD9910_STEP_RATE_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9910_read_step_rate, \
	.write = ad9910_write_step_rate, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

#define AD9910_DRG_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9910_read_drg_attrs, \
	.write = ad9910_write_drg_attrs, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info ad9910_shared_ext_info[] = {
	AD9910_EXT_INFO("profile", AD9910_PROFILE, IIO_SHARED_BY_TYPE),
	AD9910_EXT_INFO("powerdown", AD9910_POWERDOWN, IIO_SHARED_BY_TYPE),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_osk_ext_info[] = {
	AD9910_EXT_INFO("pinctrl_en", AD9910_OSK_MANUAL_EXTCTL, IIO_SEPARATE),
	AD9910_EXT_INFO("scale_increment", AD9910_OSK_AUTO_STEP, IIO_SEPARATE),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_drg_ext_info[] = {
	IIO_ENUM("destination", IIO_SEPARATE, &ad9910_drg_destination_enum),
	IIO_ENUM_AVAILABLE("destination", IIO_SEPARATE, &ad9910_drg_destination_enum),
	AD9910_EXT_INFO("dwell_max_en", AD9910_DRG_DWELL_HIGH_ENABLE, IIO_SEPARATE),
	AD9910_EXT_INFO("dwell_min_en", AD9910_DRG_DWELL_LOW_ENABLE, IIO_SEPARATE),
	AD9910_DRG_EXT_INFO("frequency_max", AD9910_DRG_FREQ_UPPER_LIMIT),
	AD9910_DRG_EXT_INFO("frequency_min", AD9910_DRG_FREQ_LOWER_LIMIT),
	AD9910_DRG_EXT_INFO("frequency_increment", AD9910_DRG_FREQ_INC_STEP),
	AD9910_DRG_EXT_INFO("frequency_decrement", AD9910_DRG_FREQ_DEC_STEP),
	AD9910_DRG_EXT_INFO("phase_max", AD9910_DRG_PHASE_UPPER_LIMIT),
	AD9910_DRG_EXT_INFO("phase_min", AD9910_DRG_PHASE_LOWER_LIMIT),
	AD9910_DRG_EXT_INFO("phase_increment", AD9910_DRG_PHASE_INC_STEP),
	AD9910_DRG_EXT_INFO("phase_decrement", AD9910_DRG_PHASE_DEC_STEP),
	AD9910_DRG_EXT_INFO("scale_max", AD9910_DRG_AMP_UPPER_LIMIT),
	AD9910_DRG_EXT_INFO("scale_min", AD9910_DRG_AMP_LOWER_LIMIT),
	AD9910_DRG_EXT_INFO("scale_increment", AD9910_DRG_AMP_INC_STEP),
	AD9910_DRG_EXT_INFO("scale_decrement", AD9910_DRG_AMP_DEC_STEP),
	AD9910_STEP_RATE_EXT_INFO("increment_sampling_frequency", AD9910_DRG_INC_STEP_RATE),
	AD9910_STEP_RATE_EXT_INFO("decrement_sampling_frequency", AD9910_DRG_DEC_STEP_RATE),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_ram_ext_info[] = {
	IIO_ENUM("destination", IIO_SEPARATE, &ad9910_ram_destination_enum),
	IIO_ENUM_AVAILABLE("destination", IIO_SEPARATE, &ad9910_ram_destination_enum),
	IIO_ENUM("operating_mode", IIO_SEPARATE, &ad9910_ram_oper_mode_enum),
	IIO_ENUM_AVAILABLE("operating_mode", IIO_SEPARATE, &ad9910_ram_oper_mode_enum),
	AD9910_EXT_INFO("address_start", AD9910_RAM_START_ADDR, IIO_SEPARATE),
	AD9910_EXT_INFO("address_end", AD9910_RAM_END_ADDR, IIO_SEPARATE),
	{ },
};

static const struct iio_chan_spec ad9910_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_SINGLE_TONE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.ext_info = ad9910_shared_ext_info,
	},
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_OSK,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_osk_ext_info,
	},
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_DRG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = ad9910_drg_ext_info,
	},
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_RAM,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_ram_ext_info,
	},
};

static int ad9910_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	u64 tmp64;
	u32 tmp32;
	u32 ram_en;

	guard(mutex)(&st->lock);

	ram_en = FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK,
			   st->reg[AD9910_REG_CFR1].val32);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->channel) {
		case AD9910_CHANNEL_OSK:
			*val = FIELD_GET(AD9910_CFR1_OSK_ENABLE_MSK,
					 st->reg[AD9910_REG_CFR1].val32);
			break;
		case AD9910_CHANNEL_DRG:
			*val = FIELD_GET(AD9910_CFR2_DRG_ENABLE_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_RAM:
			*val = ram_en;
			break;
		default:
			return -EINVAL;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		if (chan->channel == AD9910_CHANNEL_SINGLE_TONE) {
			if (!ram_en)
				tmp32 = FIELD_GET(AD9910_PROFILE_ST_FTW_MSK,
						  st->reg[AD9910_REG_PROFILE(st->profile)].val64);
			else
				tmp32 = FIELD_GET(AD9910_PROFILE_ST_FTW_MSK,
						  st->reg_profile[st->profile]);
		} else {
			tmp32 = st->reg[AD9910_REG_FTW].val32;
		}
		tmp64 = (u64)tmp32 * st->sysclk_hz;
		*val = tmp64 >> 32;
		*val2 = (tmp64 & 0xFFFFFFFFULL) * MICRO >> 32;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_PHASE:
		if (chan->channel == AD9910_CHANNEL_SINGLE_TONE) {
			if (!ram_en)
				tmp32 = FIELD_GET(AD9910_PROFILE_ST_POW_MSK,
						  st->reg[AD9910_REG_PROFILE(st->profile)].val64);
			else
				tmp32 = FIELD_GET(AD9910_PROFILE_ST_POW_MSK,
						  st->reg_profile[st->profile]);
		} else {
			tmp32 = st->reg[AD9910_REG_POW].val16;
		}
		tmp32 *= 360;
		*val = tmp32 >> 16;
		*val2 = ((u64)tmp32 & 0xFFFF) * MICRO >> 16;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		if (chan->channel == AD9910_CHANNEL_SINGLE_TONE) {
			if (!ram_en)
				tmp32 = FIELD_GET(AD9910_PROFILE_ST_ASF_MSK,
						  st->reg[AD9910_REG_PROFILE(st->profile)].val64);
			else
				tmp32 = FIELD_GET(AD9910_PROFILE_ST_ASF_MSK,
						 st->reg_profile[st->profile]);
		} else {
			tmp32 = FIELD_GET(AD9910_ASF_AMP_SCALE_FACTOR_MSK,
					 st->reg[AD9910_REG_ASF].val32);
		}
		*val = 0;
		*val2 = (u64)tmp32 * MICRO >> 14;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->channel) {
		case AD9910_CHANNEL_OSK:
			tmp32 = FIELD_GET(AD9910_ASF_AMP_RAMP_RATE_MSK,
					  st->reg[AD9910_REG_ASF].val32);
			break;
		case AD9910_CHANNEL_RAM:
			if (ram_en)
				tmp32 = FIELD_GET(AD9910_PROFILE_RAM_STEP_RATE_MSK,
						  st->reg[AD9910_REG_PROFILE(st->profile)].val64);
			else
				tmp32 = FIELD_GET(AD9910_PROFILE_RAM_STEP_RATE_MSK,
						  st->reg_profile[st->profile]);
			break;
		default:
			return -EINVAL;
		}
		tmp32 *= 4;
		*val = st->sysclk_hz / tmp32;
		*val2 = div_u64((u64)(st->sysclk_hz % tmp32) * MICRO, tmp32);
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad9910_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	u64 tmp64;
	u32 tmp32;
	u16 tmp16;
	u32 ram_en;

	int ret = 0;

	guard(mutex)(&st->lock);

	ram_en = FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK,
			   st->reg[AD9910_REG_CFR1].val32);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		val = val ? 1 : 0;
		switch (chan->channel) {
		case AD9910_CHANNEL_OSK:
			return ad9910_reg32_update(st, AD9910_REG_CFR1,
						   AD9910_CFR1_OSK_ENABLE_MSK,
						   FIELD_PREP(AD9910_CFR1_OSK_ENABLE_MSK, val),
						   true);
		case AD9910_CHANNEL_DRG:
			return ad9910_reg32_update(st, AD9910_REG_CFR2,
						   AD9910_CFR2_DRG_ENABLE_MSK,
						   FIELD_PREP(AD9910_CFR2_DRG_ENABLE_MSK, val),
						   true);
		case AD9910_CHANNEL_RAM:
			if (ram_en == val)
				return 0;

			/* switch profile configs */
			for (int i = 0; i < AD9910_NUM_PROFILES; i++) {
				tmp64 = st->reg[AD9910_REG_PROFILE(i)].val64;
				ret = ad9910_reg64_write(st,
							 AD9910_REG_PROFILE(i),
							 st->reg_profile[i],
							 false);
				if (ret)
					return ret;
				st->reg_profile[i] = tmp64;
			}

			return ad9910_reg32_update(st, AD9910_REG_CFR1,
						   AD9910_CFR1_RAM_ENABLE_MSK,
						   FIELD_PREP(AD9910_CFR1_RAM_ENABLE_MSK, val),
						   true);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_FREQUENCY:
		if (val < 0 || val >= st->sysclk_hz / 2)
			return -EINVAL;

		tmp32 = ad9910_rational_scale((u64)val * MICRO + val2, BIT_ULL(32),
					      (u64)MICRO * st->sysclk_hz);

		if (chan->channel != AD9910_CHANNEL_SINGLE_TONE)
			return ad9910_reg32_write(st, AD9910_REG_FTW, tmp32, true);

		if (!ram_en)
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
						   AD9910_PROFILE_ST_FTW_MSK,
						   FIELD_PREP(AD9910_PROFILE_ST_FTW_MSK, tmp32),
						   true);

		st->reg_profile[st->profile] &= ~AD9910_PROFILE_ST_FTW_MSK;
		st->reg_profile[st->profile] |= FIELD_PREP(AD9910_PROFILE_ST_FTW_MSK, tmp32);
		break;
	case IIO_CHAN_INFO_PHASE:
		val %= 360;
		if (val < 0)
			val += 360;

		tmp64 = ((u64)val * MICRO + val2) << 16;
		tmp16 = DIV_U64_ROUND_CLOSEST(tmp64, MICRO * 360);

		if (chan->channel != AD9910_CHANNEL_SINGLE_TONE)
			return ad9910_reg16_write(st, AD9910_REG_POW, tmp16, true);

		tmp64 = FIELD_PREP(AD9910_PROFILE_ST_POW_MSK, tmp16);
		if (!ram_en)
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
						   AD9910_PROFILE_ST_POW_MSK,
						   tmp64, true);

		st->reg_profile[st->profile] &= ~AD9910_PROFILE_ST_POW_MSK;
		st->reg_profile[st->profile] |= tmp64;
		break;
	case IIO_CHAN_INFO_SCALE:
		if (val < 0 || val > 1 || (val == 1 && val2 > 0))
			return -EINVAL;

		tmp64 = ((u64)val * MICRO + val2) << 14;
		tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, MICRO);
		tmp16 = min(tmp64, AD9910_ASF_MAX);

		if (chan->channel != AD9910_CHANNEL_SINGLE_TONE)
			return ad9910_reg32_update(st, AD9910_REG_ASF,
						   AD9910_ASF_AMP_SCALE_FACTOR_MSK,
						   FIELD_PREP(AD9910_ASF_AMP_SCALE_FACTOR_MSK, tmp16),
						   true);

		tmp64 = FIELD_PREP(AD9910_PROFILE_ST_ASF_MSK, tmp16);
		if (!ram_en)
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
						   AD9910_PROFILE_ST_ASF_MSK,
						   tmp64, true);

		st->reg_profile[st->profile] &= ~AD9910_PROFILE_ST_ASF_MSK;
		st->reg_profile[st->profile] |= tmp64;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		tmp64 = ((u64)val * MICRO + val2) * 4;
		if (!tmp64)
			return -EINVAL;

		tmp64 = DIV64_U64_ROUND_CLOSEST((u64)st->sysclk_hz * MICROHZ_PER_HZ, tmp64);
		tmp32 = clamp(tmp64, 1U, AD9910_STEP_RATE_MAX);

		switch (chan->channel) {
		case AD9910_CHANNEL_OSK:
			tmp32 = FIELD_PREP(AD9910_ASF_AMP_RAMP_RATE_MSK, tmp32);
			ret = ad9910_reg32_update(st, AD9910_REG_ASF,
						  AD9910_ASF_AMP_RAMP_RATE_MSK,
						  tmp32, true);
			break;
		case AD9910_CHANNEL_RAM:
			tmp64 = FIELD_PREP(AD9910_PROFILE_RAM_STEP_RATE_MSK, tmp32);
			if (ram_en)
				return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
							   AD9910_PROFILE_RAM_STEP_RATE_MSK,
							   tmp64, true);

			st->reg_profile[st->profile] &= ~AD9910_PROFILE_RAM_STEP_RATE_MSK;
			st->reg_profile[st->profile] |= tmp64;
			break;
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int ad9910_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
	case IIO_CHAN_INFO_PHASE:
	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad9910_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int ret;
	u64 tmp64;
	u32 tmp32;
	u16 tmp16;
	bool high32 = !!(reg & AD9910_REG_HIGH32);

	reg &= ~AD9910_REG_HIGH32;
	if (reg >= AD9910_REG_RAM)
		return -EINVAL;

	guard(mutex)(&st->lock);

	switch (reg) {
	case AD9910_REG_DRG_LIMIT:
	case AD9910_REG_DRG_STEP:
	case AD9910_REG_PROFILE0:
	case AD9910_REG_PROFILE1:
	case AD9910_REG_PROFILE2:
	case AD9910_REG_PROFILE3:
	case AD9910_REG_PROFILE4:
	case AD9910_REG_PROFILE5:
	case AD9910_REG_PROFILE6:
	case AD9910_REG_PROFILE7:
		if (readval) {
			ret = ad9910_reg64_read(st, reg, &tmp64);
			if (ret < 0)
				return ret;

			if (high32)
				*readval = tmp64 >> 32;
			else
				*readval = tmp64 & 0xFFFFFFFFULL;
		} else {
			tmp64 = st->reg[reg].val64;
			if (high32) {
				tmp64 &= 0xFFFFFFFFULL;
				tmp64 |= (u64)writeval << 32;
			} else {
				tmp64 &= 0xFFFFFFFF00000000ULL;
				tmp64 |= (u64)writeval;
			}
			return ad9910_reg64_write(st, reg, tmp64, true);
		}
		break;
	case AD9910_REG_POW:
		if (!readval)
			return ad9910_reg16_write(st, reg, writeval, true);

		ret = ad9910_reg16_read(st, reg, &tmp16);
		if (ret < 0)
			return ret;
		*readval = tmp16;
		break;
	default:
		if (!readval)
			return ad9910_reg32_write(st, reg, writeval, true);

		ret = ad9910_reg32_read(st, reg, &tmp32);
		if (ret < 0)
			return ret;
		*readval = tmp32;
		break;
	}

	return ret;
}

static int ad9910_read_label(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     char *label)
{
	return sysfs_emit(label, "%s\n", ad9910_channel_str[chan->channel]);
}

static const struct iio_info ad9910_info = {
	.read_raw = ad9910_read_raw,
	.write_raw = ad9910_write_raw,
	.write_raw_get_fmt = ad9910_write_raw_get_fmt,
	.read_label = ad9910_read_label,
	.attrs = &ad9910_attrs_group,
	.debugfs_reg_access = &ad9910_reg_access,
};

static int ad9910_set_dac_current(struct ad9910_state *st, bool update)
{
	u32 fsc_code;

	/* FSC = (86.4 / Rset) * (1 + CODE/256) where Rset = 10k ohms */
	fsc_code = DIV_ROUND_CLOSEST(st->data.dac_output_current, 90) - 96;
	fsc_code &= 0xFFU;

	return ad9910_reg32_write(st, AD9910_REG_AUX_DAC, fsc_code, update);
}

static int ad9910_cfg_sysclk(struct ad9910_state *st, bool update)
{
	u32 cp_index;
	u32 cfr3 = AD9910_CFR3_OPEN_MSK;

	cfr3 |= FIELD_PREP(AD9910_CFR3_DRV0_MSK, st->data.refclk_out_drv);
	st->sysclk_hz = clk_get_rate(st->refclk);

	if (st->data.pll_multiplier) {
		st->sysclk_hz *= st->data.pll_multiplier;
		if (st->sysclk_hz < AD9910_PLL_OUT_MIN_FREQ_HZ ||
		    st->sysclk_hz > AD9910_PLL_OUT_MAX_FREQ_HZ) {
			dev_err(&st->spi->dev, "invalid vco frequency: %u Hz\n", st->sysclk_hz);
			return -ERANGE;
		}

		if (st->data.pll_vco_range >= AD9910_VCO_RANGE_NUM) {
			if (st->sysclk_hz <= AD9910_VCO0_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 0;
			else if (st->sysclk_hz <= AD9910_VCO1_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 1;
			else if (st->sysclk_hz <= AD9910_VCO2_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 2;
			else if (st->sysclk_hz <= AD9910_VCO3_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 3;
			else if (st->sysclk_hz <= AD9910_VCO4_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 4;
			else
				st->data.pll_vco_range = 5;
			dev_dbg(&st->spi->dev, "auto-selected VCO range: %u\n",
				st->data.pll_vco_range);
		}

		cp_index = find_closest(st->data.pll_charge_pump_current,
					ad9910_charge_pump_currents,
					ARRAY_SIZE(ad9910_charge_pump_currents));
		cfr3 |= FIELD_PREP(AD9910_CFR3_VCO_SEL_MSK, st->data.pll_vco_range) |
			FIELD_PREP(AD9910_CFR3_ICP_MSK, cp_index) |
			FIELD_PREP(AD9910_CFR3_N_MSK, st->data.pll_multiplier) |
			AD9910_CFR3_PLL_EN_MSK;
	} else {
		cfr3 |= AD9910_CFR3_VCO_SEL_MSK |
			AD9910_CFR3_ICP_MSK |
			AD9910_CFR3_REFCLK_DIV_RESETB_MSK |
			FIELD_PREP(AD9910_CFR3_REFCLK_DIV_BYPASS_MSK, !st->data.ref_div2_en) |
			AD9910_CFR3_PFD_RESET_MSK;
		if (st->data.ref_div2_en)
			st->sysclk_hz >>= 1;
	}

	return ad9910_reg32_write(st, AD9910_REG_CFR3, cfr3, update);
}

static int ad9910_parse_fw(struct ad9910_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	/* PLL configuration */
	st->data.pll_multiplier = 0;
	ret = device_property_read_u32(dev, "adi,pll-multiplier", &tmp);
	if (!ret) {
		if (tmp < AD9910_PLL_MIN_N && tmp > AD9910_PLL_MAX_N)
			return dev_err_probe(dev, -ERANGE,
						"invalid PLL multiplier %u\n", tmp);
		st->data.pll_multiplier = tmp;
	}

	if (st->data.pll_multiplier) {
		st->data.pll_vco_range = AD9910_VCO_RANGE_NUM;
		ret = device_property_read_u32(dev, "adi,pll-vco-range", &tmp);
		if (!ret) {
			if (tmp >= AD9910_VCO_RANGE_NUM)
				dev_err_probe(dev, -ERANGE,
					      "invalid VCO range: %u\n", tmp);
			st->data.pll_vco_range = tmp;
		}

		st->data.pll_charge_pump_current = AD9910_ICP_MAX_uA;
		ret = device_property_read_u32(dev, "adi,charge-pump-current-microamp", &tmp);
		if (!ret) {
			if (tmp < AD9910_ICP_MIN_uA && tmp > AD9910_ICP_MAX_uA)
				return dev_err_probe(dev, -ERANGE,
						     "invalid charge pump current %u\n", tmp);
			st->data.pll_charge_pump_current = tmp;
		}
	}

	/* Feature flags */
	st->data.ref_div2_en = device_property_read_bool(dev, "adi,reference-div2-enable");
	st->data.inverse_sinc_enable = device_property_read_bool(dev, "adi,inverse-sinc-enable");
	st->data.select_sine_output = device_property_read_bool(dev, "adi,select-sine-output");
	st->data.sync_clk_enable = !device_property_read_bool(dev, "adi,sync-clk-disable");
	st->data.pdclk_enable = !device_property_read_bool(dev, "adi,pdclk-disable");
	st->data.pdclk_invert = device_property_read_bool(dev, "adi,pdclk-invert");
	st->data.tx_enable_invert = device_property_read_bool(dev, "adi,tx-enable-invert");

	/* DAC full-scale current */
	st->data.dac_output_current = AD9910_DAC_IOUT_DEFAULT_uA;
	ret = device_property_read_u32(dev, "adi,dac-output-current-microamp", &tmp);
	if (!ret) {
		if (tmp < AD9910_DAC_IOUT_MIN_uA || tmp > AD9910_DAC_IOUT_MAX_uA)
			return dev_err_probe(dev, -ERANGE, "Invalid DAC output current %u\n", tmp);
		st->data.dac_output_current = tmp;
	}

	return 0;
}

static int ad9910_setup(struct ad9910_state *st)
{
	u32 reg32;
	int ret;

	/* out of reset */
	if (st->gpio_m_reset)
		gpiod_set_value_cansleep(st->gpio_m_reset, 0);

	if (st->gpio_io_reset)
		gpiod_set_value_cansleep(st->gpio_io_reset, 0);

	/* configure CFR1 */
	reg32 = AD9910_CFR1_SDIO_INPUT_ONLY_MSK;
	reg32 |= FIELD_PREP(AD9910_CFR1_INV_SINC_EN_MSK, st->data.inverse_sinc_enable) |
		 FIELD_PREP(AD9910_CFR1_SELECT_SINE_MSK, st->data.select_sine_output);

	ret = ad9910_reg32_write(st, AD9910_REG_CFR1, reg32, false);
	if (ret < 0)
		return ret;

	/* configure CFR2 */
	reg32 = AD9910_CFR2_AMP_SCALE_SINGLE_TONE_MSK;
	reg32 |= AD9910_CFR2_SYNC_TIMING_VAL_DISABLE_MSK |
		 FIELD_PREP(AD9910_CFR2_SYNC_CLK_EN_MSK, st->data.sync_clk_enable) |
		 FIELD_PREP(AD9910_CFR2_PDCLK_ENABLE_MSK, st->data.pdclk_enable) |
		 FIELD_PREP(AD9910_CFR2_PDCLK_INVERT_MSK, st->data.pdclk_invert) |
		 FIELD_PREP(AD9910_CFR2_TXENABLE_INVERT_MSK, st->data.tx_enable_invert);

	ret = ad9910_reg32_write(st, AD9910_REG_CFR2, reg32, false);
	if (ret < 0)
		return ret;

	/* configure sysclk (CFR3) */
	ret = ad9910_cfg_sysclk(st, false);
	if (ret < 0)
		return ret;

	ret = ad9910_set_dac_current(st, false);
	if (ret < 0)
		return ret;

	/* configure step rate with default values */
	reg32 = FIELD_PREP(AD9910_ASF_AMP_RAMP_RATE_MSK, 1);
	ret = ad9910_reg32_write(st, AD9910_REG_ASF, reg32, false);
	if (ret < 0)
		return ret;

	reg32 = FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, 1) |
		FIELD_PREP(AD9910_DRG_RATE_INC_MSK, 1);
	ret = ad9910_reg32_write(st, AD9910_REG_DRG_RATE, reg32, false);
	if (ret < 0)
		return ret;

	for (int i = 0; i < AD9910_NUM_PROFILES; i++) {
		st->reg_profile[i] = AD9910_PROFILE_RAM_OPEN_MSK;
		st->reg_profile[i] |= FIELD_PREP(AD9910_PROFILE_RAM_STEP_RATE_MSK, 1);
		st->reg_profile[i] |= FIELD_PREP(AD9910_PROFILE_RAM_END_ADDR_MSK,
						 AD9910_RAM_ADDR_MAX);
	}

	return ad9910_io_update(st);
}

static void ad9910_power_down(void *data)
{
	struct ad9910_state *st = data;

	if (st->gpio_powerdown)
		gpiod_set_value_cansleep(st->gpio_powerdown, 1);
	else
		ad9910_reg32_update(st, AD9910_REG_CFR1,
				    AD9910_CFR1_SOFT_POWER_DOWN_MSK,
				    AD9910_CFR1_SOFT_POWER_DOWN_MSK,
				    true);
}

static int ad9910_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9910_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	spi_set_drvdata(spi, indio_dev);

	st->refclk = devm_clk_get_enabled(&spi->dev, NULL);
	if (IS_ERR(st->refclk))
		return -EPROBE_DEFER;

	ret = devm_regulator_bulk_get_enable(&spi->dev,
					     ARRAY_SIZE(ad9910_power_supplies),
					     ad9910_power_supplies);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to get regulators\n");

	st->gpio_powerdown = devm_gpiod_get_optional(&spi->dev, "powerdown",
						     GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_powerdown))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_powerdown),
				     "failed to get powerdown gpio\n");

	st->gpio_m_reset = devm_gpiod_get_index_optional(&spi->dev, "reset", 0,
							 GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_m_reset))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_m_reset),
				     "failed to get master reset gpio\n");

	st->gpio_io_reset = devm_gpiod_get_index_optional(&spi->dev, "reset", 1,
							  GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_io_reset))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_io_reset),
				     "failed to get io reset gpio\n");

	st->gpio_io_update = devm_gpiod_get_optional(&spi->dev, "update",
						     GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_io_update))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_io_update),
				     "failed to get io update gpio\n");

	/* profile pins */
	st->gpio_profile[0] = devm_gpiod_get_index_optional(&spi->dev,
							    "profile", 0,
							    GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_profile[0]))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_profile[0]),
				     "failed to get profile0 gpio\n");

	st->gpio_profile[1] = devm_gpiod_get_index_optional(&spi->dev,
							    "profile", 1,
							    GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_profile[1]))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_profile[1]),
				     "failed to get profile1 gpio\n");

	st->gpio_profile[2] = devm_gpiod_get_index_optional(&spi->dev,
							    "profile", 2,
							    GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_profile[2]))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_profile[2]),
				     "failed to get profile2 gpio\n");

	ret = ad9910_parse_fw(st);
	if (ret)
		return ret;

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return ret;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9910_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9910_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9910_channels);

	ret = ad9910_setup(st);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret, "device setup failed\n");

	ret = devm_add_action_or_reset(&spi->dev, ad9910_power_down, st);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to add power down action\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad9910_id[] = {
	{"ad9910", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9910_id);

static const struct of_device_id ad9910_of_match[] = {
	{ .compatible = "adi,ad9910" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9910_of_match);

static struct spi_driver ad9910_driver = {
	.driver = {
		.name = "ad9910",
		.of_match_table = ad9910_of_match,
	},
	.probe = ad9910_probe,
	.id_table = ad9910_id,
};
module_spi_driver(ad9910_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9910 DDS driver");
MODULE_LICENSE("GPL");
