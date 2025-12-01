// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 SPI DDS (Direct Digital Synthesizer) driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/unaligned.h>
#include <linux/util_macros.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <linux/iio/frequency/ad9910.h>

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
#define AD9910_REG_HIGH32_FLAG_MSK	BIT(8)

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
#define AD9910_CFR2_DRG_NO_DWELL_MSK		GENMASK(18, 17)
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

/* POW Register Bits */
#define AD9910_POW_PP_LSB_MSK			GENMASK(7, 0)

/* ASF Register Bits */
#define AD9910_ASF_RAMP_RATE_MSK		GENMASK(31, 16)
#define AD9910_ASF_SCALE_FACTOR_MSK		GENMASK(15, 2)
#define AD9910_ASF_SCALE_FACTOR_PP_LSB_MSK	GENMASK(7, 2)
#define AD9910_ASF_STEP_SIZE_MSK		GENMASK(1, 0)

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
#define AD9910_PI_NANORAD		3141592653UL

#define AD9910_MAX_SYSCLK_HZ		(1000 * HZ_PER_MHZ)
#define AD9910_MAX_PHASE_MICRORAD	(AD9910_PI_NANORAD / 500)

#define AD9910_ASF_MAX			GENMASK(13, 0)
#define AD9910_ASF_PP_LSB_MAX		GENMASK(5, 0)
#define AD9910_POW_MAX			GENMASK(15, 0)
#define AD9910_POW_PP_LSB_MAX		GENMASK(7, 0)
#define AD9910_STEP_RATE_MAX		GENMASK(15, 0)

#define AD9910_RAM_SIZE_MAX_WORDS	1024
#define AD9910_RAM_WORD_SIZE		sizeof(__be32)
#define AD9910_RAM_SIZE_MAX_BYTES	(AD9910_RAM_SIZE_MAX_WORDS * AD9910_RAM_WORD_SIZE)
#define AD9910_RAM_ADDR_MAX		(AD9910_RAM_SIZE_MAX_WORDS - 1)

#define AD9910_RAM_ENABLED(st)		\
	FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, (st)->reg[AD9910_REG_CFR1].val32)

/* PLL constants */
#define AD9910_PLL_MIN_N		12
#define AD9910_PLL_MAX_N		127

#define AD9910_PLL_IN_MIN_FREQ_HZ	(3200 * HZ_PER_KHZ)
#define AD9910_PLL_IN_MAX_FREQ_HZ	(60 * HZ_PER_MHZ)

#define AD9910_PLL_OUT_MIN_FREQ_HZ	(420 * HZ_PER_MHZ)
#define AD9910_PLL_OUT_MAX_FREQ_HZ	(1000 * HZ_PER_MHZ)

#define AD9910_VCO0_RANGE_AUTO_MAX_HZ	(457 * HZ_PER_MHZ)
#define AD9910_VCO1_RANGE_AUTO_MAX_HZ	(530 * HZ_PER_MHZ)
#define AD9910_VCO2_RANGE_AUTO_MAX_HZ	(632 * HZ_PER_MHZ)
#define AD9910_VCO3_RANGE_AUTO_MAX_HZ	(775 * HZ_PER_MHZ)
#define AD9910_VCO4_RANGE_AUTO_MAX_HZ	(897 * HZ_PER_MHZ)
#define AD9910_VCO_RANGE_NUM		6

#define AD9910_REFCLK_OUT_DRV_DISABLED	0

#define AD9910_ICP_MIN_uA		212
#define AD9910_ICP_MAX_uA		387
#define AD9910_ICP_STEP_uA		25

#define AD9910_DAC_IOUT_MAX_uA		31590
#define AD9910_DAC_IOUT_DEFAULT_uA	20070
#define AD9910_DAC_IOUT_MIN_uA		8640

#define AD9910_REFDIV2_MIN_FREQ_HZ	(120 * HZ_PER_MHZ)
#define AD9910_REFDIV2_MAX_FREQ_HZ	(1900 * HZ_PER_MHZ)

#define AD9910_SPI_DATA_IDX		1
#define AD9910_SPI_DATA_LEN_MAX		AD9910_RAM_SIZE_MAX_BYTES
#define AD9910_SPI_MESSAGE_LEN_MAX	(AD9910_SPI_DATA_IDX + AD9910_SPI_DATA_LEN_MAX)
#define AD9910_SPI_READ_MSK		BIT(7)
#define AD9910_SPI_ADDR_MSK		GENMASK(4, 0)

#ifndef FIELD_MODIFY /* backport fix */
#define FIELD_MODIFY(mask, reg_p, val) 						\
	({									\
		*(reg_p) &= ~(mask);						\
		*(reg_p) |= (((typeof(mask))(val) << __bf_shf(mask)) & (mask));	\
	})
#endif

enum {
	AD9910_CHAN_IDX_PHY,
	AD9910_CHAN_IDX_PROFILE_0,
	AD9910_CHAN_IDX_PROFILE_1,
	AD9910_CHAN_IDX_PROFILE_2,
	AD9910_CHAN_IDX_PROFILE_3,
	AD9910_CHAN_IDX_PROFILE_4,
	AD9910_CHAN_IDX_PROFILE_5,
	AD9910_CHAN_IDX_PROFILE_6,
	AD9910_CHAN_IDX_PROFILE_7,
	AD9910_CHAN_IDX_PARALLEL_PORT,
	AD9910_CHAN_IDX_DRG,
	AD9910_CHAN_IDX_DRG_RAMP_UP,
	AD9910_CHAN_IDX_DRG_RAMP_DOWN,
	AD9910_CHAN_IDX_RAM,
	AD9910_CHAN_IDX_OSK,
};

enum {
	AD9910_POWERDOWN,
	AD9910_PP_FREQ_SCALE,
	AD9910_PP_FREQ_OFFSET,
	AD9910_PP_PHASE_OFFSET,
	AD9910_PP_AMP_OFFSET,
	AD9910_DRG_FREQ_STEP,
	AD9910_DRG_PHASE_STEP,
	AD9910_DRG_AMP_STEP,
	AD9910_OSK_MANUAL_EXTCTL,
	AD9910_OSK_AUTO_STEP,
};

struct ad9910_data {
	u32 sysclk_freq_hz;
	u32 dac_output_current;

	u16 pll_charge_pump_current;
	u8 refclk_out_drv;
	bool pll_enabled;
};

union ad9910_reg {
	u64 val64;
	u32 val32;
	u16 val16;
};

struct ad9910_state {
	struct spi_device *spi;
	struct ad9910_backend *back;
	struct clk *refclk;
	struct fw_upload *ram_fwu;

	struct gpio_desc *gpio_pwdown;
	struct gpio_desc *gpio_update;
	struct gpio_descs *gpio_profile;

	/* cached registers */
	union ad9910_reg reg[AD9910_REG_NUM_CACHED];

	/*
	 * alternate profile registers used to store RAM profile settings when
	 * RAM mode is disabled and Single Tone profile settings when RAM mode
	 * is enabled.
	 */
	u64 reg_profile[AD9910_NUM_PROFILES];

	/* Lock for accessing device registers and state variables */
	struct mutex lock; 

	struct ad9910_data data;
	u8 profile;

	bool ram_fwu_cancel;
	char ram_fwu_name[20];

	union {
		__be64 be64;
		__be32 be32;
		__be16 be16;
	} rx_buf;
	/*
	 * RAM loading requires a reasonable amount of bytes, at the same time
	 * DMA capable SPI drivers requires the transfer buffers to live in
	 * their own cache lines.
	 */
	u8 tx_buf[AD9910_SPI_MESSAGE_LEN_MAX] __aligned(IIO_DMA_MINALIGN);
};

/* Helper struct to manage AD9910 backend registrations */
struct ad9910_backend_entry {
	struct list_head head;
	struct ad9910_backend backend;
	void *priv;
};

static LIST_HEAD(ad9910_backend_list);
static DEFINE_MUTEX(ad9910_backend_lock);

static void ad9910_backend_unregister(void *arg)
{
	struct ad9910_backend_entry *entry = arg;

	guard(mutex)(&ad9910_backend_lock);
	list_del(&entry->head);
}

/**
 * devm_ad9910_backend_register - Device managed AD9910 backend device register
 * @dev: Backend device being registered
 * @info: AD9910 Backend info
 * @priv: Device private data
 *
 * @info and @priv are mandatory. Not providing them results in -EINVAL.
 *
 * RETURNS:
 * 0 on success, negative error number on failure.
 */
int devm_ad9910_backend_register(struct device *dev,
				 const struct ad9910_backend_info *info,
				 void *priv)
{
	struct ad9910_backend_entry *entry;
	int ret;

	if (!info || !info->ops || !priv)
		return dev_err_probe(dev, -EINVAL,
				     "No backend ops or priv provided\n");

	ret = devm_iio_backend_register(dev, &info->base, priv);
	if (ret)
		return ret;

	entry = devm_kzalloc(dev, sizeof(*entry), GFP_KERNEL);
	if (!entry)
		return -ENOMEM;

	entry->priv = priv;
	entry->backend.ops = info->ops;
	scoped_guard(mutex, &ad9910_backend_lock)
		list_add(&entry->head, &ad9910_backend_list);

	return devm_add_action_or_reset(dev, ad9910_backend_unregister, entry);
}
EXPORT_SYMBOL_NS_GPL(devm_ad9910_backend_register, AD9910);

static struct ad9910_backend *devm_ad9910_backend_get_optional(struct device *dev)
{
	struct ad9910_backend_entry *entry;
	struct iio_backend *iio_back;
	void *priv;

	iio_back = devm_iio_backend_get_optional(dev, NULL);
	if (IS_ERR(iio_back))
		return ERR_CAST(iio_back);

	if (!iio_back)
		return NULL;

	/* make sure backend was registered as an ad9910_backend */
	priv = iio_backend_get_priv(iio_back);
	guard(mutex)(&ad9910_backend_lock);
	list_for_each_entry(entry, &ad9910_backend_list, head) {
		if (entry->priv == priv) {
			entry->backend.iio_back = iio_back;
			return &entry->backend;
		}
	}

	return ERR_PTR(-ENODEV);
}

/**
 * ad9910_rational_scale() - Perform scaling of input given a reference.
 * @input: The input value to be scaled.
 * @scale: The numerator of the scaling factor.
 * @reference: The denominator of the scaling factor.
 *
 * Closest rounding with mul_u64_u64_div_u64
 *
 * Return: The scaled value.
 */
static u64 ad9910_rational_scale(u64 input, u64 scale, u64 reference)
{
	u64 output = mul_u64_u64_div_u64(input, scale, reference);

	if (input * scale - output * reference >= (reference >> 1))
		output++;

	return output;
}

static inline u64 ad9910_ram_profile_val(struct ad9910_state *st)
{
	if (AD9910_RAM_ENABLED(st))
		return st->reg[AD9910_REG_PROFILE(st->profile)].val64;
	else
		return st->reg_profile[st->profile];
}

static inline u64 ad9910_st_profile_val(struct ad9910_state *st, u8 profile)
{
	if (AD9910_RAM_ENABLED(st))
		return st->reg_profile[profile];
	else
		return st->reg[AD9910_REG_PROFILE(profile)].val64;
}

static int ad9910_io_update(struct ad9910_state *st)
{
	if (st->back)
		return ad9910_backend_op_call(st->back, io_update);

	if (st->gpio_update) {
		gpiod_set_value_cansleep(st->gpio_update, 1);
		udelay(1);
		gpiod_set_value_cansleep(st->gpio_update, 0);
	}

	return 0;
}

static inline int ad9910_spi_read(struct ad9910_state *st, u8 reg, size_t len)
{
	st->tx_buf[0] = AD9910_SPI_READ_MSK |
			FIELD_PREP(AD9910_SPI_ADDR_MSK, reg);
	return spi_write_then_read(st->spi, st->tx_buf, 1, &st->rx_buf, len);
}

static inline int ad9910_spi_write(struct ad9910_state *st, u8 reg, size_t len,
				   bool update)
{
	int ret;

	st->tx_buf[0] = FIELD_PREP(AD9910_SPI_ADDR_MSK, reg);
	ret = spi_write(st->spi, st->tx_buf, AD9910_SPI_DATA_IDX + len);
	if (!ret && update)
		return ad9910_io_update(st);

	return ret;
}

#define AD9910_REG_READ_FN(nb)						\
static int ad9910_reg##nb##_read(struct ad9910_state *st, u8 reg,	\
				 u##nb * data)				\
{									\
	int ret;							\
									\
	ret = ad9910_spi_read(st, reg, sizeof(*data));			\
	if (ret)							\
		return ret;						\
									\
	*data = be##nb##_to_cpu(st->rx_buf.be##nb);			\
	return ret;							\
}

AD9910_REG_READ_FN(16)
AD9910_REG_READ_FN(32)
AD9910_REG_READ_FN(64)

#define AD9910_REG_WRITE_FN(nb)						\
static int ad9910_reg##nb##_write(struct ad9910_state *st, u8 reg,	\
				  u##nb data, bool update)		\
{									\
	int ret;							\
									\
	put_unaligned_be##nb(data, &st->tx_buf[AD9910_SPI_DATA_IDX]);	\
	ret = ad9910_spi_write(st, reg, sizeof(data), update);		\
	if (ret)							\
		return ret;						\
									\
	st->reg[reg].val##nb = data;					\
	return ret;							\
}

AD9910_REG_WRITE_FN(16)
AD9910_REG_WRITE_FN(32)
AD9910_REG_WRITE_FN(64)

#define AD9910_REG_UPDATE_FN(nb)					\
static int ad9910_reg##nb##_update(struct ad9910_state *st,		\
				   u8 reg, u##nb mask,			\
				   u##nb data, bool update)		\
{									\
	u##nb reg_val = (st->reg[reg].val##nb & ~mask) | (data & mask);	\
									\
	if (reg_val == st->reg[reg].val##nb && !update)			\
		return 0;						\
									\
	return ad9910_reg##nb##_write(st, reg, reg_val, update);	\
}

AD9910_REG_UPDATE_FN(16)
AD9910_REG_UPDATE_FN(32)
AD9910_REG_UPDATE_FN(64)

static int ad9910_set_dac_current(struct ad9910_state *st, bool update)
{
	u32 fsc_code;

	/* FSC = (86.4 / Rset) * (1 + CODE/256) where Rset = 10k ohms */
	fsc_code = DIV_ROUND_CLOSEST(st->data.dac_output_current, 90) - 96;
	fsc_code &= 0xFFU;

	return ad9910_reg32_write(st, AD9910_REG_AUX_DAC, fsc_code, update);
}

static int ad9910_set_sysclk_freq(struct ad9910_state *st, u32 freq_hz,
				  bool update)
{
	u32 sysclk_freq_hz, refclk_freq_hz = clk_get_rate(st->refclk);
	u32 tmp32, vco_sel;
	int ret;

	if (st->data.pll_enabled) {
		if (refclk_freq_hz < AD9910_PLL_IN_MIN_FREQ_HZ ||
		    refclk_freq_hz > AD9910_PLL_IN_MAX_FREQ_HZ) {
			dev_err(&st->spi->dev,
				"REF_CLK frequency %u Hz is out of PLL input range\n",
				refclk_freq_hz);
			return -ERANGE;
		}

		tmp32 = DIV_ROUND_CLOSEST(freq_hz, refclk_freq_hz);
		tmp32 = clamp(tmp32, AD9910_PLL_MIN_N, AD9910_PLL_MAX_N);
		sysclk_freq_hz = refclk_freq_hz * tmp32;

		if (sysclk_freq_hz < AD9910_PLL_OUT_MIN_FREQ_HZ ||
		    sysclk_freq_hz > AD9910_PLL_OUT_MAX_FREQ_HZ) {
			dev_err(&st->spi->dev,
				"PLL output frequency %u Hz is out of range\n",
				sysclk_freq_hz);
			return -ERANGE;
		}

		if (sysclk_freq_hz <= AD9910_VCO0_RANGE_AUTO_MAX_HZ)
			vco_sel = 0;
		else if (sysclk_freq_hz <= AD9910_VCO1_RANGE_AUTO_MAX_HZ)
			vco_sel = 1;
		else if (sysclk_freq_hz <= AD9910_VCO2_RANGE_AUTO_MAX_HZ)
			vco_sel = 2;
		else if (sysclk_freq_hz <= AD9910_VCO3_RANGE_AUTO_MAX_HZ)
			vco_sel = 3;
		else if (sysclk_freq_hz <= AD9910_VCO4_RANGE_AUTO_MAX_HZ)
			vco_sel = 4;
		else
			vco_sel = 5;

		ret = ad9910_reg32_update(st, AD9910_REG_CFR3,
					  AD9910_CFR3_N_MSK | AD9910_CFR3_VCO_SEL_MSK,
					  FIELD_PREP(AD9910_CFR3_N_MSK, tmp32) |
					  FIELD_PREP(AD9910_CFR3_VCO_SEL_MSK, vco_sel),
					  update);
		if (ret)
			return ret;
	} else {
		tmp32 = DIV_ROUND_CLOSEST(refclk_freq_hz, freq_hz);
		tmp32 = clamp(tmp32, 1, 2);
		sysclk_freq_hz = refclk_freq_hz / tmp32;
		tmp32 = FIELD_PREP(AD9910_CFR3_REFCLK_DIV_BYPASS_MSK, tmp32 % 2);
		ret = ad9910_reg32_update(st, AD9910_REG_CFR3,
					  AD9910_CFR3_REFCLK_DIV_BYPASS_MSK,
					  tmp32, update);
		if (ret)
			return ret;
	}

	st->data.sysclk_freq_hz = sysclk_freq_hz;
	if (st->back)
		return iio_backend_set_sampling_freq(st->back->iio_back,
						     AD9910_CHANNEL_PARALLEL_PORT,
						     st->data.sysclk_freq_hz / 4);

	return 0;
}

static int ad9910_profile_set(struct ad9910_state *st, u8 profile)
{
	DECLARE_BITMAP(values, BITS_PER_TYPE(profile));

	st->profile = profile;
	if (st->back)
		return ad9910_backend_op_call(st->back, profile_set, profile);

	values[0] = profile;
	gpiod_multi_set_value_cansleep(st->gpio_profile, values);

	return 0;
}

static int ad9910_powerdown_set(struct ad9910_state *st, bool enable)
{
	if (st->back)
		return ad9910_backend_op_call(st->back, powerdown_set, enable);

	gpiod_set_value_cansleep(st->gpio_pwdown, enable);
	return 0;
}

static inline int ad9910_drg_destination_set(struct ad9910_state *st,
					     enum ad9910_destination dest,
					     bool update)
{
	return ad9910_reg32_update(st, AD9910_REG_CFR2,
				   AD9910_CFR2_DRG_DEST_MSK,
				   FIELD_PREP(AD9910_CFR2_DRG_DEST_MSK, dest),
				   update);
}

static ssize_t ad9910_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_POWERDOWN:
		val = !!FIELD_GET(AD9910_CFR1_SOFT_POWER_DOWN_MSK,
				  st->reg[AD9910_REG_CFR1].val32);
		break;
	case AD9910_PP_FREQ_SCALE:
		val = BIT(FIELD_GET(AD9910_CFR2_FM_GAIN_MSK,
				    st->reg[AD9910_REG_CFR2].val32));
		break;
	case AD9910_OSK_MANUAL_EXTCTL:
		val = FIELD_GET(AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK,
				st->reg[AD9910_REG_CFR1].val32);
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, IIO_VAL_INT, 1, &val);
}

static ssize_t ad9910_ext_info_write(struct iio_dev *indio_dev,
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
	case AD9910_POWERDOWN:
		val32 = val32 ? AD9910_CFR1_SOFT_POWER_DOWN_MSK : 0;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
					  AD9910_CFR1_SOFT_POWER_DOWN_MSK,
					  val32, true);
		break;
	case AD9910_PP_FREQ_SCALE:
		if (val32 > BIT(15) || !is_power_of_2(val32))
			return -EINVAL;

		val32 = FIELD_PREP(AD9910_CFR2_FM_GAIN_MSK, ilog2(val32));
		ret = ad9910_reg32_update(st, AD9910_REG_CFR2,
					  AD9910_CFR2_FM_GAIN_MSK,
					  val32, true);
		break;
	case AD9910_OSK_MANUAL_EXTCTL:
		val32 = val32 ? AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK : 0;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
					  AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK,
					  val32, true);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

static ssize_t ad9910_pp_attrs_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan,
				    char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int vals[2];
	u32 tmp32;
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PP_FREQ_OFFSET:
		tmp64 = (u64)st->reg[AD9910_REG_FTW].val32 * st->data.sysclk_freq_hz;
		vals[0] = tmp64 >> 32;
		vals[1] = ((tmp64 & GENMASK_ULL(31, 0)) * MICRO) >> 32;
		break;
	case AD9910_PP_PHASE_OFFSET:
		tmp32 = FIELD_GET(AD9910_POW_PP_LSB_MSK,
				  st->reg[AD9910_REG_POW].val16);
		tmp32 = (tmp32 * AD9910_MAX_PHASE_MICRORAD) >> 16;
		vals[0] = tmp32 / MICRO;
		vals[1] = tmp32 % MICRO;
		break;
	case AD9910_PP_AMP_OFFSET:
		tmp32 = FIELD_GET(AD9910_ASF_SCALE_FACTOR_PP_LSB_MSK,
				  st->reg[AD9910_REG_ASF].val32);
		vals[0] = 0;
		vals[1] = (u64)tmp32 * MICRO >> 14;
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_pp_attrs_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u32 tmp32;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PP_FREQ_OFFSET:
		if (!in_range(val, 0, st->data.sysclk_freq_hz / 2))
			return -EINVAL;

		tmp32 = ad9910_rational_scale((u64)val * MICRO + val2, BIT_ULL(32),
					      (u64)MICRO * st->data.sysclk_freq_hz);
		ret = ad9910_reg32_write(st, AD9910_REG_FTW, tmp32, true);
		break;
	case AD9910_PP_PHASE_OFFSET:
		if (val)
			return -EINVAL;

		if (!in_range(val2, 0, (AD9910_MAX_PHASE_MICRORAD >> 8)))
			return -EINVAL;

		tmp32 = DIV_ROUND_CLOSEST((u32)val2 << 16, AD9910_MAX_PHASE_MICRORAD);
		tmp32 = min(tmp32, AD9910_POW_PP_LSB_MAX);
		tmp32 = FIELD_PREP(AD9910_POW_PP_LSB_MSK, tmp32);
		ret = ad9910_reg16_update(st, AD9910_REG_POW,
					  AD9910_POW_PP_LSB_MSK,
					  tmp32, true);
		break;
	case AD9910_PP_AMP_OFFSET:
		if (val)
			return -EINVAL;

		if (!in_range(val2, 0, (MICRO >> 8)))
			return -EINVAL;

		tmp32 = DIV_ROUND_CLOSEST((u32)val2 << 14, MICRO);
		tmp32 = min(tmp32, AD9910_ASF_PP_LSB_MAX);
		tmp32 = FIELD_PREP(AD9910_ASF_SCALE_FACTOR_PP_LSB_MSK, tmp32);
		ret = ad9910_reg32_update(st, AD9910_REG_ASF,
					  AD9910_ASF_SCALE_FACTOR_PP_LSB_MSK,
					  tmp32, true);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

static ssize_t ad9910_drg_attrs_read(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	unsigned int type;
	int vals[2];
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (chan->channel) {
	case AD9910_CHANNEL_DRG_RAMP_UP:
		tmp64 = FIELD_GET(AD9910_DRG_STEP_INC_MSK,
				  st->reg[AD9910_REG_DRG_STEP].val64);
		break;
	case AD9910_CHANNEL_DRG_RAMP_DOWN:
		tmp64 = FIELD_GET(AD9910_DRG_STEP_DEC_MSK,
				  st->reg[AD9910_REG_DRG_STEP].val64);
		break;
	default:
		return -EINVAL;
	}

	switch (private) {
	case AD9910_DRG_FREQ_STEP:
		type = IIO_VAL_INT_PLUS_MICRO;
		tmp64 *= st->data.sysclk_freq_hz;
		vals[0] = tmp64 >> 32;
		vals[1] = ((tmp64 & GENMASK_ULL(31, 0)) * MICRO) >> 32;
		break;
	case AD9910_DRG_PHASE_STEP:
		type = IIO_VAL_INT_PLUS_NANO;
		tmp64 *= AD9910_PI_NANORAD;
		tmp64 >>= 31;
		vals[0] = div_u64_rem(tmp64, NANO, &vals[1]);
		break;
	case AD9910_DRG_AMP_STEP:
		type = IIO_VAL_INT_PLUS_NANO;
		vals[0] = 0;
		vals[1] = tmp64 * NANO >> 32;
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, type, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_drg_attrs_write(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	enum ad9910_destination dest;
	int val, val2;
	u64 tmp64;
	int ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_FREQ_STEP:
		ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
		if (ret)
			return ret;

		if (!in_range(val, 0, st->data.sysclk_freq_hz / 2))
			return -EINVAL;

		tmp64 = (u64)val * MICRO + val2;
		tmp64 = ad9910_rational_scale(tmp64, BIT_ULL(32),
					      (u64)MICRO * st->data.sysclk_freq_hz);
		dest = AD9910_DEST_FREQUENCY;
		break;
	case AD9910_DRG_PHASE_STEP:
		ret = iio_str_to_fixpoint(buf, 100000000, &val, &val2);
		if (ret)
			return ret;

		if (val < 0 || val2 < 0)
			return -EINVAL;

		tmp64 = (u64)val * NANO + val2;
		if (tmp64 > 2ULL * AD9910_PI_NANORAD)
			return -EINVAL;

		tmp64 <<= 31;
		tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, AD9910_PI_NANORAD);
		dest = AD9910_DEST_PHASE;
		break;
	case AD9910_DRG_AMP_STEP:
		ret = iio_str_to_fixpoint(buf, 100000000, &val, &val2);
		if (ret)
			return ret;

		if (val < 0 || val > 1 || (val == 1 && val2 > 0))
			return -EINVAL;

		tmp64 = ((u64)val * NANO + val2) << 32;
		tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, NANO);
		dest = AD9910_DEST_AMPLITUDE;
		break;
	default:
		return -EINVAL;
	}

	tmp64 = min(tmp64, U32_MAX);
	ret = ad9910_drg_destination_set(st, dest, false);
	if (ret)
		return ret;

	switch (chan->channel) {
	case AD9910_CHANNEL_DRG_RAMP_UP:
		ret = ad9910_reg64_update(st, AD9910_REG_DRG_STEP,
					  AD9910_DRG_STEP_INC_MSK,
					  FIELD_PREP(AD9910_DRG_STEP_INC_MSK, tmp64),
					  true);
		break;
	case AD9910_CHANNEL_DRG_RAMP_DOWN:
		ret = ad9910_reg64_update(st, AD9910_REG_DRG_STEP,
					  AD9910_DRG_STEP_DEC_MSK,
					  FIELD_PREP(AD9910_DRG_STEP_DEC_MSK, tmp64),
					  true);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

static const u16 ad9910_osk_ustep[] = {
	0, 61, 122, 244, 488,
};

static ssize_t ad9910_osk_attrs_read(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int vals[2];
	bool auto_en;
	u32 raw_val;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_OSK_AUTO_STEP:
		auto_en = FIELD_GET(AD9910_CFR1_SELECT_AUTO_OSK_MSK,
				    st->reg[AD9910_REG_CFR1].val32);
		raw_val = FIELD_GET(AD9910_ASF_STEP_SIZE_MSK,
				    st->reg[AD9910_REG_ASF].val32);
		vals[0] = 0;
		vals[1] = auto_en ? ad9910_osk_ustep[raw_val + 1] : 0;

		return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, 2, vals);
	default:
		return -EINVAL;
	}
}

static ssize_t ad9910_osk_attrs_write(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	int ret;
	u32 raw_val;

	ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_OSK_AUTO_STEP:
		if (val != 0)
			return -EINVAL;

		raw_val = find_closest(val2, ad9910_osk_ustep,
				       ARRAY_SIZE(ad9910_osk_ustep));
		if (raw_val) {
			/* set OSK step and get automatic OSK enabled */
			raw_val = FIELD_PREP(AD9910_ASF_STEP_SIZE_MSK,
					     raw_val - 1);
			ret = ad9910_reg32_update(st, AD9910_REG_ASF,
						  AD9910_ASF_STEP_SIZE_MSK,
						  raw_val, true);
			if (ret)
				return ret;

			raw_val = AD9910_CFR1_SELECT_AUTO_OSK_MSK;
		}

		ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
					  AD9910_CFR1_SELECT_AUTO_OSK_MSK,
					  raw_val, true);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

#define AD9910_EXT_INFO_TMPL(_name, _ident, _shared, _fn_desc) { \
	.name = _name, \
	.read = ad9910_ ## _fn_desc ## _read, \
	.write = ad9910_ ## _fn_desc ## _write, \
	.private = _ident, \
	.shared = _shared, \
}

#define AD9910_EXT_INFO(_name, _ident, _shared) \
	AD9910_EXT_INFO_TMPL(_name, _ident, _shared, ext_info)

#define AD9910_PP_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, pp_attrs)

#define AD9910_DRG_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, drg_attrs)

#define AD9910_OSK_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, osk_attrs)

static const struct iio_chan_spec_ext_info ad9910_phy_ext_info[] = {
	AD9910_EXT_INFO("powerdown", AD9910_POWERDOWN, IIO_SEPARATE),
	{ }
};

static const struct iio_chan_spec_ext_info ad9910_pp_ext_info[] = {
	AD9910_EXT_INFO("frequency_scale", AD9910_PP_FREQ_SCALE, IIO_SEPARATE),
	AD9910_PP_EXT_INFO("frequency_offset", AD9910_PP_FREQ_OFFSET),
	AD9910_PP_EXT_INFO("phase_offset", AD9910_PP_PHASE_OFFSET),
	AD9910_PP_EXT_INFO("scale_offset", AD9910_PP_AMP_OFFSET),
	{ }
};

static const struct iio_chan_spec_ext_info ad9910_drg_ramp_ext_info[] = {
	AD9910_DRG_EXT_INFO("frequency_step", AD9910_DRG_FREQ_STEP),
	AD9910_DRG_EXT_INFO("phase_step", AD9910_DRG_PHASE_STEP),
	AD9910_DRG_EXT_INFO("scale_step", AD9910_DRG_AMP_STEP),
	{ }
};

static const struct iio_chan_spec_ext_info ad9910_osk_ext_info[] = {
	AD9910_EXT_INFO("pinctrl_en", AD9910_OSK_MANUAL_EXTCTL, IIO_SEPARATE),
	AD9910_OSK_EXT_INFO("scale_step", AD9910_OSK_AUTO_STEP),
	{ }
};

static const struct iio_scan_type ad9910_pp_scan_type[] = {
	[AD9910_PP_SCAN_TYPE_FULL] = {
		.sign = 'u',
		.realbits = 18,
		.storagebits = 32,
		.shift = 0,
	},
	[AD9910_PP_SCAN_TYPE_DATA_ONLY] ={
		.sign = 'u',
		.realbits = 16,
		.storagebits = 16,
		.shift = 0,
	},
};

#define AD9910_PROFILE_CHAN(idx) {				\
	.type = IIO_ALTVOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = AD9910_CHANNEL_PROFILE_ ## idx,		\
	.address = AD9910_CHAN_IDX_PROFILE_ ## idx,		\
	.scan_index = -1,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |	\
			      BIT(IIO_CHAN_INFO_FREQUENCY) |	\
			      BIT(IIO_CHAN_INFO_PHASE) |	\
			      BIT(IIO_CHAN_INFO_SCALE),		\
}

static const struct iio_chan_spec ad9910_channels[] = {
	[AD9910_CHAN_IDX_PHY] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_PHY,
		.address = AD9910_CHAN_IDX_PHY,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_phy_ext_info,
	},
	[AD9910_CHAN_IDX_PROFILE_0] = AD9910_PROFILE_CHAN(0),
	[AD9910_CHAN_IDX_PROFILE_1] = AD9910_PROFILE_CHAN(1),
	[AD9910_CHAN_IDX_PROFILE_2] = AD9910_PROFILE_CHAN(2),
	[AD9910_CHAN_IDX_PROFILE_3] = AD9910_PROFILE_CHAN(3),
	[AD9910_CHAN_IDX_PROFILE_4] = AD9910_PROFILE_CHAN(4),
	[AD9910_CHAN_IDX_PROFILE_5] = AD9910_PROFILE_CHAN(5),
	[AD9910_CHAN_IDX_PROFILE_6] = AD9910_PROFILE_CHAN(6),
	[AD9910_CHAN_IDX_PROFILE_7] = AD9910_PROFILE_CHAN(7),
	[AD9910_CHAN_IDX_PARALLEL_PORT] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_PARALLEL_PORT,
		.address = AD9910_CHAN_IDX_PARALLEL_PORT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_scan_type = ad9910_pp_scan_type,
		.num_ext_scan_type = ARRAY_SIZE(ad9910_pp_scan_type),
		.ext_info = ad9910_pp_ext_info,
		.has_ext_scan_type = 1
	},
	[AD9910_CHAN_IDX_DRG] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_DRG,
		.address = AD9910_CHAN_IDX_DRG,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
	},
	[AD9910_CHAN_IDX_DRG_RAMP_UP] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_DRG_RAMP_UP,
		.address = AD9910_CHAN_IDX_DRG_RAMP_UP,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_drg_ramp_ext_info,
	},
	[AD9910_CHAN_IDX_DRG_RAMP_DOWN] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_DRG_RAMP_DOWN,
		.address = AD9910_CHAN_IDX_DRG_RAMP_DOWN,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_drg_ramp_ext_info,
	},
	[AD9910_CHAN_IDX_RAM] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_RAM,
		.address = AD9910_CHAN_IDX_RAM,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	[AD9910_CHAN_IDX_OSK] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_OSK,
		.address = AD9910_CHAN_IDX_OSK,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_osk_ext_info,
	},
};

static int ad9910_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	u64 tmp64;
	u32 tmp32;

	guard(mutex)(&st->lock);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = (chan->channel - AD9910_CHANNEL_PROFILE_0);
			*val = (tmp32 == st->profile);
			break;
		case AD9910_CHANNEL_PARALLEL_PORT:
			*val = FIELD_GET(AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_DRG:
			*val = FIELD_GET(AD9910_CFR2_DRG_ENABLE_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_DRG_RAMP_UP:
			*val = FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			*val = FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_LOW_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_RAM:
			*val = FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK,
					 st->reg[AD9910_REG_CFR1].val32);
			break;
		case AD9910_CHANNEL_OSK:
			*val = FIELD_GET(AD9910_CFR1_OSK_ENABLE_MSK,
					 st->reg[AD9910_REG_CFR1].val32);
			break;
		default:
			return -EINVAL;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			tmp32 = FIELD_GET(AD9910_PROFILE_ST_FTW_MSK,
					  ad9910_st_profile_val(st, tmp32));
			break;
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp32 = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK,
					  st->reg[AD9910_REG_DRG_LIMIT].val64);
			break;
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp32 = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK,
					  st->reg[AD9910_REG_DRG_LIMIT].val64);
			break;
		case AD9910_CHANNEL_RAM:
			tmp32 = st->reg[AD9910_REG_FTW].val32;
			break;
		default:
			return -EINVAL;
		}
		tmp64 = (u64)tmp32 * st->data.sysclk_freq_hz;
		*val = tmp64 >> 32;
		*val2 = ((tmp64 & GENMASK_ULL(31, 0)) * MICRO) >> 32;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_PHASE:
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			tmp64 = FIELD_GET(AD9910_PROFILE_ST_POW_MSK,
					  ad9910_st_profile_val(st, tmp32));
			tmp32 = (tmp64 * AD9910_MAX_PHASE_MICRORAD) >> 16;
			*val = tmp32 / MICRO;
			*val2 = tmp32 % MICRO;
			return IIO_VAL_INT_PLUS_MICRO;
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp64 = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK,
					  st->reg[AD9910_REG_DRG_LIMIT].val64);
			tmp64 = (tmp64 * AD9910_PI_NANORAD) >> 31;
			*val = div_u64_rem(tmp64, NANO, val2);
			return IIO_VAL_INT_PLUS_NANO;
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp64 = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK,
					  st->reg[AD9910_REG_DRG_LIMIT].val64);
			tmp64 = (tmp64 * AD9910_PI_NANORAD) >> 31;
			*val = div_u64_rem(tmp64, NANO, val2);
			return IIO_VAL_INT_PLUS_NANO;
		case AD9910_CHANNEL_RAM:
			tmp64 = st->reg[AD9910_REG_POW].val16;
			tmp32 = (tmp64 * AD9910_MAX_PHASE_MICRORAD) >> 16;
			*val = tmp32 / MICRO;
			*val2 = tmp32 % MICRO;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			tmp64 = FIELD_GET(AD9910_PROFILE_ST_ASF_MSK,
					  ad9910_st_profile_val(st, tmp32));
			*val = 0;
			*val2 = tmp64 * MICRO >> 14;
			return IIO_VAL_INT_PLUS_MICRO;
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp64 = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK,
					  st->reg[AD9910_REG_DRG_LIMIT].val64);
			*val = 0;
			*val2 = tmp64 * NANO >> 32;
			return IIO_VAL_INT_PLUS_NANO;
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp64 = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK,
					  st->reg[AD9910_REG_DRG_LIMIT].val64);
			*val = 0;
			*val2 = tmp64 * NANO >> 32;
			return IIO_VAL_INT_PLUS_NANO;
		case AD9910_CHANNEL_OSK:
			tmp64 = FIELD_GET(AD9910_ASF_SCALE_FACTOR_MSK,
					  st->reg[AD9910_REG_ASF].val32);
			*val = 0;
			*val2 = tmp64 * MICRO >> 14;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan->channel) {
		case AD9910_CHANNEL_PHY:
			*val = st->data.sysclk_freq_hz;
			return IIO_VAL_INT;
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp32 = FIELD_GET(AD9910_DRG_RATE_INC_MSK,
					  st->reg[AD9910_REG_DRG_RATE].val32);
			break;
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp32 = FIELD_GET(AD9910_DRG_RATE_DEC_MSK,
					  st->reg[AD9910_REG_DRG_RATE].val32);
			break;
		case AD9910_CHANNEL_RAM:
			tmp32 = FIELD_GET(AD9910_PROFILE_RAM_STEP_RATE_MSK,
					  ad9910_ram_profile_val(st));
			break;
		case AD9910_CHANNEL_OSK:
			tmp32 = FIELD_GET(AD9910_ASF_RAMP_RATE_MSK,
					  st->reg[AD9910_REG_ASF].val32);
			break;
		default:
			return -EINVAL;
		}
		if (!tmp32)
			return -ERANGE;
		tmp32 *= 4;
		*val = st->data.sysclk_freq_hz / tmp32;
		*val2 = div_u64((u64)(st->data.sysclk_freq_hz % tmp32) * MICRO, tmp32);
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
	int ret;

	guard(mutex)(&st->lock);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			if (!val) {
				if (tmp32 != st->profile)
					return 0;

				tmp32 = (tmp32 + 1) % AD9910_NUM_PROFILES;
			}

			return ad9910_profile_set(st, tmp32);
		case AD9910_CHANNEL_PARALLEL_PORT:
			if (st->back) {
				if (val)
					iio_backend_chan_enable(st->back->iio_back,
								chan->channel);
				else
					iio_backend_chan_disable(st->back->iio_back,
								 chan->channel);
			}

			tmp32 = FIELD_PREP(AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK, !!val);
			return ad9910_reg32_update(st, AD9910_REG_CFR2,
						   AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_DRG:
			tmp32 = FIELD_PREP(AD9910_CFR2_DRG_ENABLE_MSK, !!val);
			return ad9910_reg32_update(st, AD9910_REG_CFR2,
						   AD9910_CFR2_DRG_ENABLE_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp32 = FIELD_PREP(AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK, !!val);
			ret = ad9910_reg32_update(st, AD9910_REG_CFR2,
						  AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK,
						  tmp32, true);
			if (ret)
				return ret;

			if (st->back) {
				tmp32 = FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_MSK,
						  st->reg[AD9910_REG_CFR2].val32);
				ad9910_backend_op_call(st->back, drg_oper_mode_set, tmp32);
			}
			return 0;
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp32 = FIELD_PREP(AD9910_CFR2_DRG_NO_DWELL_LOW_MSK, !!val);
			ret = ad9910_reg32_update(st, AD9910_REG_CFR2,
						  AD9910_CFR2_DRG_NO_DWELL_LOW_MSK,
						  tmp32, true);
			if (ret)
				return ret;

			if (st->back) {
				tmp32 = FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_MSK,
						  st->reg[AD9910_REG_CFR2].val32);
				ad9910_backend_op_call(st->back, drg_oper_mode_set, tmp32);
			}
			return 0;
		case AD9910_CHANNEL_RAM:
			if (AD9910_RAM_ENABLED(st) == !!val)
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

			tmp32 = FIELD_PREP(AD9910_CFR1_RAM_ENABLE_MSK, !!val);
			return ad9910_reg32_update(st, AD9910_REG_CFR1,
						   AD9910_CFR1_RAM_ENABLE_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_OSK:
			tmp32 = FIELD_PREP(AD9910_CFR1_OSK_ENABLE_MSK, !!val);
			return ad9910_reg32_update(st, AD9910_REG_CFR1,
						   AD9910_CFR1_OSK_ENABLE_MSK,
						   tmp32, true);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_FREQUENCY:
		if (!in_range(val, 0, st->data.sysclk_freq_hz / 2))
			return -EINVAL;

		tmp64 = ad9910_rational_scale((u64)val * MICRO + val2, BIT_ULL(32),
					      (u64)MICRO * st->data.sysclk_freq_hz);
		tmp64 = min(tmp64, U32_MAX);
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			if (AD9910_RAM_ENABLED(st)) {
				FIELD_MODIFY(AD9910_PROFILE_ST_FTW_MSK,
					     &st->reg_profile[tmp32], tmp64);
				return 0;
			}
			tmp64 = FIELD_PREP(AD9910_PROFILE_ST_FTW_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(tmp32),
						   AD9910_PROFILE_ST_FTW_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_DRG_RAMP_UP:
			ret = ad9910_drg_destination_set(st,
							 AD9910_DEST_FREQUENCY,
							 false);
			if (ret)
				return ret;

			tmp64 = FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
						   AD9910_DRG_LIMIT_UPPER_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			ret = ad9910_drg_destination_set(st,
							 AD9910_DEST_FREQUENCY,
							 false);
			if (ret)
				return ret;

			tmp64 = FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
						   AD9910_DRG_LIMIT_LOWER_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_RAM:
			return ad9910_reg32_write(st, AD9910_REG_FTW, tmp64, true);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val2 < 0)
			return -EINVAL;

		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			tmp64 = (u64)val * MICRO + val2;
			if (tmp64 >= AD9910_MAX_PHASE_MICRORAD)
				return -EINVAL;

			tmp64 <<= 16;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, AD9910_MAX_PHASE_MICRORAD);
			tmp16 = min(tmp64, AD9910_POW_MAX);

			if (AD9910_RAM_ENABLED(st)) {
				FIELD_MODIFY(AD9910_PROFILE_ST_POW_MSK,
					     &st->reg_profile[tmp32], tmp16);
				return 0;
			}

			tmp64 = FIELD_PREP(AD9910_PROFILE_ST_POW_MSK, tmp16);
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(tmp32),
						   AD9910_PROFILE_ST_POW_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp64 = (u64)val * NANO + val2;
			if (tmp64 > 2ULL * AD9910_PI_NANORAD)
				return -EINVAL;

			ret = ad9910_drg_destination_set(st, AD9910_DEST_PHASE,
							 false);
			if (ret)
				return ret;

			tmp64 <<= 31;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, AD9910_PI_NANORAD);
			tmp64 = min(tmp64, U32_MAX);
			tmp64 = FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
						   AD9910_DRG_LIMIT_UPPER_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp64 = (u64)val * NANO + val2;
			if (tmp64 > 2ULL * AD9910_PI_NANORAD)
				return -EINVAL;

			ret = ad9910_drg_destination_set(st, AD9910_DEST_PHASE,
							 false);
			if (ret)
				return ret;

			tmp64 <<= 31;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, AD9910_PI_NANORAD);
			tmp64 = min(tmp64, U32_MAX);
			tmp64 = FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
						   AD9910_DRG_LIMIT_LOWER_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_RAM:
			tmp64 = (u64)val * MICRO + val2;
			if (tmp64 >= AD9910_MAX_PHASE_MICRORAD)
				return -EINVAL;

			tmp64 <<= 16;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, AD9910_MAX_PHASE_MICRORAD);
			tmp16 = min(tmp64, AD9910_POW_MAX);
			return ad9910_reg16_write(st, AD9910_REG_POW, tmp16, true);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		if (val < 0 || val > 1 || (val == 1 && val2 > 0))
			return -EINVAL;

		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
			tmp32 = chan->channel - AD9910_CHANNEL_PROFILE_0;
			tmp64 = ((u64)val * MICRO + val2) << 14;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, MICRO);
			tmp16 = min(tmp64, AD9910_ASF_MAX);

			if (AD9910_RAM_ENABLED(st)) {
				FIELD_MODIFY(AD9910_PROFILE_ST_ASF_MSK,
					     &st->reg_profile[tmp32], tmp16);
				return 0;
			}

			tmp64 = FIELD_PREP(AD9910_PROFILE_ST_ASF_MSK, tmp16);
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(tmp32),
						   AD9910_PROFILE_ST_ASF_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_DRG_RAMP_UP:
			ret = ad9910_drg_destination_set(st,
							 AD9910_DEST_AMPLITUDE,
							 false);
			if (ret)
				return ret;

			tmp64 = ((u64)val * NANO + val2) << 32;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, NANO);
			tmp64 = min(tmp64, U32_MAX);
			tmp64 = FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
						   AD9910_DRG_LIMIT_UPPER_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			ret = ad9910_drg_destination_set(st,
							 AD9910_DEST_AMPLITUDE,
							 false);
			if (ret)
				return ret;

			tmp64 = ((u64)val * NANO + val2) << 32;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, NANO);
			tmp64 = min(tmp64, U32_MAX);
			tmp64 = FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, tmp64);
			return ad9910_reg64_update(st, AD9910_REG_DRG_LIMIT,
						   AD9910_DRG_LIMIT_LOWER_MSK,
						   tmp64, true);
		case AD9910_CHANNEL_OSK:
			tmp64 = ((u64)val * MICRO + val2) << 14;
			tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, MICRO);
			tmp32 = min(tmp64, AD9910_ASF_MAX);
			tmp32 = FIELD_PREP(AD9910_ASF_SCALE_FACTOR_MSK, tmp32);
			return ad9910_reg32_update(st, AD9910_REG_ASF,
						   AD9910_ASF_SCALE_FACTOR_MSK,
						   tmp32, true);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->channel == AD9910_CHANNEL_PHY)
			return ad9910_set_sysclk_freq(st, val, true);

		tmp64 = ((u64)val * MICRO + val2) * 4;
		if (!tmp64)
			return -EINVAL;

		tmp64 = DIV64_U64_ROUND_CLOSEST((u64)st->data.sysclk_freq_hz * MICRO, tmp64);
		tmp32 = clamp(tmp64, 1U, AD9910_STEP_RATE_MAX);

		switch (chan->channel) {
		case AD9910_CHANNEL_DRG_RAMP_UP:
			tmp32 = FIELD_PREP(AD9910_DRG_RATE_INC_MSK, tmp32);
			return ad9910_reg32_update(st, AD9910_REG_DRG_RATE,
						   AD9910_DRG_RATE_INC_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			tmp32 = FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, tmp32);
			return ad9910_reg32_update(st, AD9910_REG_DRG_RATE,
						   AD9910_DRG_RATE_DEC_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_RAM:
			if (AD9910_RAM_ENABLED(st)) {
				tmp64 = FIELD_PREP(AD9910_PROFILE_RAM_STEP_RATE_MSK, tmp32);
				return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
							   AD9910_PROFILE_RAM_STEP_RATE_MSK,
							   tmp64, true);
			}

			FIELD_MODIFY(AD9910_PROFILE_RAM_STEP_RATE_MSK,
				     &st->reg_profile[st->profile], tmp32);
			return 0;
		case AD9910_CHANNEL_OSK:
			return ad9910_reg32_update(st, AD9910_REG_ASF,
						   AD9910_ASF_RAMP_RATE_MSK,
						   FIELD_PREP(AD9910_ASF_RAMP_RATE_MSK, tmp32),
						   true);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int ad9910_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_PHASE:
	case IIO_CHAN_INFO_SCALE:
		switch (chan->channel) {
		case AD9910_CHANNEL_PROFILE_0 ... AD9910_CHANNEL_PROFILE_7:
		case AD9910_CHANNEL_RAM:
		case AD9910_CHANNEL_OSK:
			return IIO_VAL_INT_PLUS_MICRO;
		case AD9910_CHANNEL_DRG_RAMP_UP:
		case AD9910_CHANNEL_DRG_RAMP_DOWN:
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->channel == AD9910_CHANNEL_PHY)
			return IIO_VAL_INT;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int ad9910_get_current_scan_type(const struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	if (chan->channel != AD9910_CHANNEL_PARALLEL_PORT)
		return -EINVAL;

	if (st->back)
		return iio_backend_scan_type_get(st->back->iio_back, chan);

	return AD9910_PP_SCAN_TYPE_FULL;
}

static int ad9910_debugfs_reg_read(struct ad9910_state *st, bool high32,
				   unsigned int reg, unsigned int *readval)
{
	union ad9910_reg tmp;
	int ret;

	switch (reg) {
	case AD9910_REG_DRG_LIMIT:
	case AD9910_REG_DRG_STEP:
	case AD9910_REG_PROFILE0 ... AD9910_REG_PROFILE7:
		ret = ad9910_reg64_read(st, reg, &tmp.val64);
		if (ret)
			return ret;
		*readval = high32 ? upper_32_bits(tmp.val64) :
				    lower_32_bits(tmp.val64);
		return 0;
	case AD9910_REG_POW:
		ret = ad9910_reg16_read(st, reg, &tmp.val16);
		if (ret)
			return ret;
		*readval = tmp.val16;
		return 0;
	default:
		ret = ad9910_reg32_read(st, reg, &tmp.val32);
		if (ret)
			return ret;
		*readval = tmp.val32;
		return 0;
	}
}

static int ad9910_debugfs_reg_write(struct ad9910_state *st, bool high32,
				    unsigned int reg, unsigned int writeval)
{
	switch (reg) {
	case AD9910_REG_DRG_LIMIT:
	case AD9910_REG_DRG_STEP:
	case AD9910_REG_PROFILE0 ... AD9910_REG_PROFILE7:
		if (high32)
			return ad9910_reg64_update(st, reg, GENMASK_ULL(63, 32),
						   FIELD_PREP(GENMASK_ULL(63, 32), writeval),
						   true);

		return ad9910_reg64_update(st, reg, GENMASK_ULL(31, 0),
					   writeval, true);
	case AD9910_REG_POW:
		return ad9910_reg16_write(st, reg, writeval, true);
	default:
		return ad9910_reg32_write(st, reg, writeval, true);
	}
}

static int ad9910_debugfs_reg_access(struct iio_dev *indio_dev,
				     unsigned int reg, unsigned int writeval,
				     unsigned int *readval)
{
	bool high32 = FIELD_GET(AD9910_REG_HIGH32_FLAG_MSK, reg);
	struct ad9910_state *st = iio_priv(indio_dev);

	/*
	 * REG_HIGH32_FLAG is only used for regiter access to indicate upper 32
	 * bits of 64-bit registers. It is a workaround for debugfs_reg_access()
	 * limitation which only supports 32-bit values.
	 */
	reg &= ~AD9910_REG_HIGH32_FLAG_MSK;
	if (reg >= AD9910_REG_RAM)
		return -EINVAL;

	guard(mutex)(&st->lock);

	if (readval)
		return ad9910_debugfs_reg_read(st, high32, reg, readval);
	else
		return ad9910_debugfs_reg_write(st, high32, reg, writeval);
}

static enum fw_upload_err ad9910_ram_fwu_prepare(struct fw_upload *fw_upload,
						 const u8 *data, u32 size)
{
	struct ad9910_state *st = fw_upload->dd_handle;
	const struct ad9910_ram_fw *fw_data = (const struct ad9910_ram_fw *)data;
	u32 wcount, bcount;

	if (size < sizeof(struct ad9910_ram_fw))
		return FW_UPLOAD_ERR_INVALID_SIZE;

	if (get_unaligned_be32(&fw_data->magic) != AD9910_RAM_FW_MAGIC)
		return FW_UPLOAD_ERR_FW_INVALID;

	wcount = get_unaligned_be32(&fw_data->wcount);
	bcount = size - sizeof(struct ad9910_ram_fw);
	if (wcount > AD9910_RAM_SIZE_MAX_WORDS ||
	    bcount != (wcount * AD9910_RAM_WORD_SIZE))
		return FW_UPLOAD_ERR_INVALID_SIZE;

	guard(mutex)(&st->lock);
	st->ram_fwu_cancel = false;

	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err ad9910_ram_fwu_write(struct fw_upload *fw_upload,
					       const u8 *data, u32 offset,
					       u32 size, u32 *written)
{
	struct ad9910_state *st = fw_upload->dd_handle;
	const struct ad9910_ram_fw *fw_data = (const struct ad9910_ram_fw *)data;
	int ret, ret2, idx, wcount;
	u64 tmp64, backup;

	if (offset != 0)
		return FW_UPLOAD_ERR_INVALID_SIZE;

	guard(mutex)(&st->lock);

	if (st->ram_fwu_cancel)
		return FW_UPLOAD_ERR_CANCELED;

	if (AD9910_RAM_ENABLED(st))
		return FW_UPLOAD_ERR_HW_ERROR;

	/* copy ram profiles */
	for (idx = 0; idx < AD9910_NUM_PROFILES; idx++)
		st->reg_profile[idx] = get_unaligned_be64(&fw_data->profiles[idx]) |
				       AD9910_PROFILE_RAM_OPEN_MSK;

	/* update CFR1 */
	ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
				  AD9910_CFR1_RAM_PLAYBACK_DEST_MSK |
				  AD9910_CFR1_INT_PROFILE_CTL_MSK,
				  get_unaligned_be32(&fw_data->cfr1), true);
	if (ret)
		return FW_UPLOAD_ERR_RW_ERROR;

	wcount = get_unaligned_be32(&fw_data->wcount);
	if (!wcount) {
		*written = size;
		return FW_UPLOAD_ERR_NONE; /* nothing else to write */
	}

	/* ensure profile is selected */
	ret = ad9910_profile_set(st, st->profile);
	if (ret)
		return FW_UPLOAD_ERR_HW_ERROR;

	/* backup profile register and update it with required address range */
	backup = st->reg[AD9910_REG_PROFILE(st->profile)].val64;
	tmp64 = AD9910_PROFILE_RAM_STEP_RATE_MSK |
		FIELD_PREP(AD9910_PROFILE_RAM_START_ADDR_MSK, 0) |
		FIELD_PREP(AD9910_PROFILE_RAM_END_ADDR_MSK, wcount - 1);
	ret = ad9910_reg64_write(st, AD9910_REG_PROFILE(st->profile), tmp64, true);
	if (ret)
		return FW_UPLOAD_ERR_RW_ERROR;

	/* populate words into tx_buf[1:] */
	memcpy(&st->tx_buf[1], fw_data->words, wcount * AD9910_RAM_WORD_SIZE);

	/* write ram data and restore profile register */
	ret = ad9910_spi_write(st, AD9910_REG_RAM,
			       wcount * AD9910_RAM_WORD_SIZE, false);
	ret2 = ad9910_reg64_write(st, AD9910_REG_PROFILE(st->profile), backup, true);
	if (ret || ret2)
		return FW_UPLOAD_ERR_RW_ERROR;

	*written = size;
	return FW_UPLOAD_ERR_NONE;
}

static enum fw_upload_err ad9910_ram_fwu_poll_complete(struct fw_upload *fw_upload)
{
	return FW_UPLOAD_ERR_NONE;
}

static void ad9910_ram_fwu_cancel(struct fw_upload *fw_upload)
{
	struct ad9910_state *st = fw_upload->dd_handle;

	guard(mutex)(&st->lock);
	st->ram_fwu_cancel = true;
}

static void ad9910_ram_fwu_unregister(void *data)
{
	firmware_upload_unregister(data);
}

static const struct fw_upload_ops ad9910_ram_fwu_ops = {
	.prepare = ad9910_ram_fwu_prepare,
	.write = ad9910_ram_fwu_write,
	.poll_complete = ad9910_ram_fwu_poll_complete,
	.cancel = ad9910_ram_fwu_cancel
};

static const char * const ad9910_channel_str[] = {
	[AD9910_CHAN_IDX_PHY] = "phy",
	[AD9910_CHAN_IDX_PROFILE_0] = "profile[0]",
	[AD9910_CHAN_IDX_PROFILE_1] = "profile[1]",
	[AD9910_CHAN_IDX_PROFILE_2] = "profile[2]",
	[AD9910_CHAN_IDX_PROFILE_3] = "profile[3]",
	[AD9910_CHAN_IDX_PROFILE_4] = "profile[4]",
	[AD9910_CHAN_IDX_PROFILE_5] = "profile[5]",
	[AD9910_CHAN_IDX_PROFILE_6] = "profile[6]",
	[AD9910_CHAN_IDX_PROFILE_7] = "profile[7]",
	[AD9910_CHAN_IDX_PARALLEL_PORT] = "parallel_port",
	[AD9910_CHAN_IDX_DRG] = "digital_ramp_generator",
	[AD9910_CHAN_IDX_DRG_RAMP_UP] = "digital_ramp_up",
	[AD9910_CHAN_IDX_DRG_RAMP_DOWN] = "digital_ramp_down",
	[AD9910_CHAN_IDX_RAM] = "ram_control",
	[AD9910_CHAN_IDX_OSK] = "output_shift_keying",
};

static int ad9910_read_label(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     char *label)
{
	return sysfs_emit(label, "%s\n", ad9910_channel_str[chan->address]);
}

static const struct iio_info ad9910_info = {
	.read_raw = ad9910_read_raw,
	.write_raw = ad9910_write_raw,
	.write_raw_get_fmt = ad9910_write_raw_get_fmt,
	.read_label = ad9910_read_label,
	.get_current_scan_type = ad9910_get_current_scan_type,
	.debugfs_reg_access = &ad9910_debugfs_reg_access,
};

static int ad9910_cfg_sysclk(struct ad9910_state *st, bool update)
{
	u32 tmp32, cfr3 = AD9910_CFR3_OPEN_MSK;

	cfr3 |= AD9910_CFR3_VCO_SEL_MSK |
		FIELD_PREP(AD9910_CFR3_DRV0_MSK, st->data.refclk_out_drv);

	if (st->data.pll_enabled) {
		tmp32 = st->data.pll_charge_pump_current - AD9910_ICP_MIN_uA;
		tmp32 = DIV_ROUND_CLOSEST(tmp32, AD9910_ICP_STEP_uA);
		cfr3 |= FIELD_PREP(AD9910_CFR3_ICP_MSK, tmp32) |
			AD9910_CFR3_PLL_EN_MSK;
	} else {
		cfr3 |= AD9910_CFR3_REFCLK_DIV_RESETB_MSK |
			AD9910_CFR3_PFD_RESET_MSK;
	}
	st->reg[AD9910_REG_CFR3].val32 = cfr3;

	return ad9910_set_sysclk_freq(st, AD9910_PLL_OUT_MAX_FREQ_HZ, update);
}

static int ad9910_parse_fw(struct ad9910_state *st)
{
	static const char * const refclk_out_drv0[] = {
		"disabled", "low", "medium", "high",
	};
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	st->data.pll_enabled = device_property_read_bool(dev, "adi,pll-enable");
	if (st->data.pll_enabled) {
		tmp = AD9910_ICP_MIN_uA;
		device_property_read_u32(dev, "adi,charge-pump-current-microamp", &tmp);
		if (tmp < AD9910_ICP_MIN_uA || tmp > AD9910_ICP_MAX_uA)
			return dev_err_probe(dev, -ERANGE,
					     "invalid charge pump current %u\n", tmp);
		st->data.pll_charge_pump_current = tmp;

		st->data.refclk_out_drv = AD9910_REFCLK_OUT_DRV_DISABLED;
		ret = device_property_match_property_string(dev,
							    "adi,refclk-out-drive-strength",
							    refclk_out_drv0,
							    ARRAY_SIZE(refclk_out_drv0));
		if (ret >= 0)
			st->data.refclk_out_drv = ret;
	}

	tmp = AD9910_DAC_IOUT_DEFAULT_uA;
	device_property_read_u32(dev, "adi,dac-output-current-microamp", &tmp);
	if (tmp < AD9910_DAC_IOUT_MIN_uA || tmp > AD9910_DAC_IOUT_MAX_uA)
		return dev_err_probe(dev, -ERANGE,
				     "Invalid DAC output current %u uA\n", tmp);
	st->data.dac_output_current = tmp;

	return 0;
}

static int ad9910_setup(struct ad9910_state *st, struct reset_control *dev_rst)
{
	u32 reg32;
	int ret, i;

	ret = reset_control_deassert(dev_rst);
	if (ret)
		return ret;

	ret = ad9910_reg32_write(st, AD9910_REG_CFR1,
				 AD9910_CFR1_SDIO_INPUT_ONLY_MSK, false);
	if (ret)
		return ret;

	reg32 = AD9910_CFR2_AMP_SCALE_SINGLE_TONE_MSK |
		AD9910_CFR2_SYNC_TIMING_VAL_DISABLE_MSK |
		AD9910_CFR2_DRG_NO_DWELL_MSK |
		AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK |
		AD9910_CFR2_SYNC_CLK_EN_MSK |
		AD9910_CFR2_PDCLK_ENABLE_MSK;
	ret = ad9910_reg32_write(st, AD9910_REG_CFR2, reg32, false);
	if (ret)
		return ret;

	ret = ad9910_cfg_sysclk(st, false);
	if (ret)
		return ret;

	ret = ad9910_set_dac_current(st, false);
	if (ret)
		return ret;

	/* configure step rate with default values */
	ret = ad9910_reg32_write(st, AD9910_REG_ASF,
				 FIELD_PREP(AD9910_ASF_RAMP_RATE_MSK, 1),
				 false);
	if (ret)
		return ret;

	reg32 = FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, 1) |
		FIELD_PREP(AD9910_DRG_RATE_INC_MSK, 1);
	ret = ad9910_reg32_write(st, AD9910_REG_DRG_RATE, reg32, false);
	if (ret)
		return ret;

	for (i = 0; i < AD9910_NUM_PROFILES; i++) {
		st->reg_profile[i] = AD9910_PROFILE_RAM_OPEN_MSK;
		st->reg_profile[i] |= FIELD_PREP(AD9910_PROFILE_RAM_STEP_RATE_MSK, 1);
		st->reg_profile[i] |= FIELD_PREP(AD9910_PROFILE_RAM_END_ADDR_MSK,
						 AD9910_RAM_ADDR_MAX);
	}

	return ad9910_io_update(st);
}

static int ad9910_extend_channels(struct device *dev,
				  struct iio_dev *indio_dev,
				  struct iio_backend *iio_back)
{
	const struct iio_chan_spec_ext_info *orig_ext_info, *tmp_ext_info;
	struct iio_chan_spec_ext_info *ext_info;
	size_t orig_num, back_num, ext_info_sz;
	struct iio_chan_spec *channels;
	int ch_idx;

	/*
	 * The channels already have extended attributes, so in order to be
	 * able to extend them, channels are copied over and have a new ext_info
	 * allocated that appends original channel ext info with the backend
	 * provided ext info. This is needed because of the current design
	 * limitations on the iio backend implementation to extend constant
	 * channels with extended attributes.
	 */
	channels = devm_kzalloc(dev, sizeof(ad9910_channels), GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	memcpy(channels, ad9910_channels, sizeof(ad9910_channels));

	/*
	 * Starting from the parallel port channel, allowing to any channel to
	 * be extended by the backend.
	 */
	for (ch_idx = AD9910_CHAN_IDX_PARALLEL_PORT;
	     ch_idx < ARRAY_SIZE(ad9910_channels); ch_idx++) {
		orig_ext_info = channels[ch_idx].ext_info;
		channels[ch_idx].ext_info = NULL;

		iio_backend_extend_chan_spec(iio_back, &channels[ch_idx]);
		if (!channels[ch_idx].ext_info) {
			channels[ch_idx].ext_info = orig_ext_info;
			continue;
		}

		if (!orig_ext_info)
			continue;

		orig_num = 0;
		tmp_ext_info = orig_ext_info;
		while (tmp_ext_info->name) {
			tmp_ext_info++;
			orig_num++;
		}

		back_num = 0;
		tmp_ext_info = channels[ch_idx].ext_info;
		while (tmp_ext_info->name) {
			tmp_ext_info++;
			back_num++;
		}

		ext_info_sz = sizeof(*ext_info) * (orig_num + back_num + 1);
		ext_info = devm_kzalloc(dev, ext_info_sz, GFP_KERNEL);
		if (!ext_info)
			return -ENOMEM;

		memcpy(ext_info, orig_ext_info, sizeof(*ext_info) * orig_num);
		memcpy(ext_info + orig_num, channels[ch_idx].ext_info,
		       sizeof(*ext_info) * back_num);
		channels[ch_idx].ext_info = ext_info;
	}

	indio_dev->channels = channels;

	return 0;
}

static void ad9910_release(void *data)
{
	struct ad9910_state *st = data;

	if (!ad9910_powerdown_set(st, true))
		return;

	ad9910_reg32_update(st, AD9910_REG_CFR1,
			    AD9910_CFR1_SOFT_POWER_DOWN_MSK,
			    AD9910_CFR1_SOFT_POWER_DOWN_MSK,
			    true);
}

static void ad9910_debugfs_init(struct ad9910_state *st,
				struct iio_dev *indio_dev)
{
	char buf[64];

	/*
	 * symlinks are created here so iio userspace tools can refer to them
	 * as debug attributes.
	 */
	snprintf(buf, sizeof(buf), "/sys/class/firmware/%s/loading", st->ram_fwu_name);
	debugfs_create_symlink("ram_loading", iio_get_debugfs_dentry(indio_dev), buf);

	snprintf(buf, sizeof(buf), "/sys/class/firmware/%s/data", st->ram_fwu_name);
	debugfs_create_symlink("ram_data", iio_get_debugfs_dentry(indio_dev), buf);

	if(st->back) {
		iio_backend_debugfs_add(st->back->iio_back, indio_dev);
		debugfs_create_symlink("backend0_reg_access",
				       iio_get_debugfs_dentry(indio_dev),
				       "./backend0/direct_reg_access");
	}
}

static int ad9910_probe(struct spi_device *spi)
{
	static const char * const supplies[] = {
		"dvdd-io33", "avdd33", "dvdd18", "avdd18",
	};
	struct reset_control *dev_rst, *io_rst;
	struct gpio_desc *io_rst_gpio;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad9910_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->refclk = devm_clk_get_enabled(dev, "ref_clk");
	if (IS_ERR(st->refclk))
		return dev_err_probe(dev, PTR_ERR(st->refclk),
				     "Failed to get reference clock\n");

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(supplies), supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	indio_dev->name = "ad9910";
	indio_dev->info = &ad9910_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9910_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9910_channels);

	dev_rst = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(dev_rst))
		return dev_err_probe(dev, PTR_ERR(dev_rst),
				     "failed to get device reset control\n");

	st->back = devm_ad9910_backend_get_optional(dev);
	if (IS_ERR(st->back))
		return dev_err_probe(dev, PTR_ERR(st->back),
				     "failed to get iio backend\n");

	/*
	 * The IO RESET pin is not used in this driver, as we assume that all
	 * SPI transfers are complete, but if it is wired up, we need to make
	 * sure it is not floating. We can use either a reset controller or a
	 * GPIO for this.
	 */
	if (st->back) {
		io_rst = devm_reset_control_get_optional_exclusive(dev, "io");
		if (IS_ERR(io_rst))
			return dev_err_probe(dev, PTR_ERR(io_rst),
					     "failed to get io reset control\n");

		ret = reset_control_deassert(io_rst);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to deassert io reset control\n");

		ret = devm_iio_backend_request_buffer(dev, st->back->iio_back,
						      indio_dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to request iio backend buffer\n");

		ret = ad9910_extend_channels(dev, indio_dev, st->back->iio_back);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to extend iio channels\n");

	} else {
		io_rst_gpio = devm_gpiod_get_optional(dev, "io-reset",
						      GPIOD_OUT_LOW);
		if (IS_ERR(io_rst_gpio))
			return dev_err_probe(dev, PTR_ERR(io_rst_gpio),
					     "failed to get io reset gpio\n");

		st->gpio_pwdown = devm_gpiod_get_optional(dev, "powerdown",
							  GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_pwdown))
			return dev_err_probe(dev, PTR_ERR(st->gpio_pwdown),
					     "failed to get powerdown gpio\n");

		st->gpio_update = devm_gpiod_get_optional(dev, "update",
							  GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_update))
			return dev_err_probe(dev, PTR_ERR(st->gpio_update),
					     "failed to get update gpio\n");

		st->gpio_profile = devm_gpiod_get_array_optional(dev, "profile",
								 GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_profile))
			return dev_err_probe(dev, PTR_ERR(st->gpio_profile),
					     "failed to get profile gpios\n");
	}

	ret = ad9910_parse_fw(st);
	if (ret)
		return ret;

	ret = ad9910_setup(st, dev_rst);
	if (ret)
		return dev_err_probe(dev, ret, "device setup failed\n");

	ret = devm_add_action_or_reset(dev, ad9910_release, st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add release action\n");

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	snprintf(st->ram_fwu_name, sizeof(st->ram_fwu_name), "%s:ram",
		 dev_name(&indio_dev->dev));
	st->ram_fwu = firmware_upload_register(THIS_MODULE, dev, st->ram_fwu_name,
					       &ad9910_ram_fwu_ops, st);
	if (IS_ERR(st->ram_fwu))
		return dev_err_probe(dev, PTR_ERR(st->ram_fwu),
				     "failed to register to the RAM Upload\n");

	ad9910_debugfs_init(st, indio_dev);

	return devm_add_action_or_reset(dev, ad9910_ram_fwu_unregister, st->ram_fwu);
}

static const struct spi_device_id ad9910_id[] = {
	{ "ad9910" },
	{ }
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
MODULE_IMPORT_NS(IIO_BACKEND);
