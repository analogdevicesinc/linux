// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 SPI DDS (Direct Digital Synthesizer) driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/unaligned.h>

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
#define AD9910_REG_HIGH32_FLAG		0x100
#define AD9910_REG_HIGH32_MSK		GENMASK_ULL(63, 32)
#define AD9910_REG_LOW32_MSK		GENMASK_ULL(31, 0)

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
#define AD9910_ASF_AMP_RAMP_RATE_MSK		GENMASK(31, 16)
#define AD9910_ASF_AMP_SCALE_FACTOR_MSK		GENMASK(15, 2)
#define AD9910_ASF_AMP_SCALE_FACTOR_PP_LSB_MSK	GENMASK(7, 2)
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
#define AD9910_DRG_LIMIT_UPPER_MSK		AD9910_REG_HIGH32_MSK
#define AD9910_DRG_LIMIT_LOWER_MSK		AD9910_REG_LOW32_MSK

/* Digital Ramp Step Register */
#define AD9910_DRG_STEP_DEC_MSK			AD9910_REG_HIGH32_MSK
#define AD9910_DRG_STEP_INC_MSK			AD9910_REG_LOW32_MSK

/* Digital Ramp Rate Register */
#define AD9910_DRG_RATE_DEC_MSK			GENMASK(31, 16)
#define AD9910_DRG_RATE_INC_MSK			GENMASK(15, 0)

/* Profile Register Format (Single Tone Mode) */
#define AD9910_PROFILE_ST_ASF_MSK		GENMASK_ULL(61, 48)
#define AD9910_PROFILE_ST_POW_MSK		GENMASK_ULL(47, 32)
#define AD9910_PROFILE_ST_FTW_MSK		AD9910_REG_LOW32_MSK

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

#define AD9910_ASF_MAX			(BIT(14) - 1)
#define AD9910_ASF_PP_LSB_MAX		(BIT(6) - 1)
#define AD9910_POW_MAX			(BIT(16) - 1)
#define AD9910_POW_PP_LSB_MAX		(BIT(8) - 1)
#define AD9910_STEP_RATE_MAX		(BIT(16) - 1)
#define AD9910_NUM_PROFILES		8

/* PLL constants */
#define AD9910_PLL_MIN_N		12
#define AD9910_PLL_MAX_N		127

#define AD9910_PLL_IN_MIN_FREQ_HZ	(3200 * HZ_PER_KHZ)
#define AD9910_PLL_IN_MAX_FREQ_HZ	(60 * HZ_PER_MHZ)

#define AD9910_PLL_OUT_MIN_FREQ_HZ	(420 * HZ_PER_MHZ)
#define AD9910_PLL_OUT_MAX_FREQ_HZ	(1000 * HZ_PER_MHZ)

#define AD9910_VCO0_RANGE_AUTO_MAX_HZ	(465 * HZ_PER_MHZ)
#define AD9910_VCO1_RANGE_AUTO_MAX_HZ	(545 * HZ_PER_MHZ)
#define AD9910_VCO2_RANGE_AUTO_MAX_HZ	(650 * HZ_PER_MHZ)
#define AD9910_VCO3_RANGE_AUTO_MAX_HZ	(790 * HZ_PER_MHZ)
#define AD9910_VCO4_RANGE_AUTO_MAX_HZ	(885 * HZ_PER_MHZ)
#define AD9910_VCO_RANGE_NUM		6

#define AD9910_REFCLK_OUT_DRV_DISABLED	0

#define AD9910_ICP_MIN_uA		212
#define AD9910_ICP_MAX_uA		387

#define AD9910_DAC_IOUT_MAX_uA		31590
#define AD9910_DAC_IOUT_DEFAULT_uA	20070
#define AD9910_DAC_IOUT_MIN_uA		8640

#define AD9910_REFDIV2_MIN_FREQ_HZ	(120 * HZ_PER_MHZ)
#define AD9910_REFDIV2_MAX_FREQ_HZ	(1900 * HZ_PER_MHZ)

#define AD9910_DRG_DEST_NUM		3
#define AD9910_RAM_DEST_NUM		4

#define AD9910_RAM_MODE_DIRECT_SWITCH	0x0
#define AD9910_RAM_MODE_RAMP_UP		0x1
#define AD9910_RAM_MODE_BIDIR		0x2
#define AD9910_RAM_MODE_BIDIR_CONT	0x3
#define AD9910_RAM_MODE_RAMP_UP_CONT	0x4
#define AD9910_RAM_MODE_SEQ		0x5
#define AD9910_RAM_MODE_SEQ_CONT	0x6

#define AD9910_RAM_SIZE_MAX_WORDS	1024
#define AD9910_RAM_WORD_SIZE		sizeof(u32)
#define AD9910_RAM_SIZE_MAX_BYTES	(AD9910_RAM_SIZE_MAX_WORDS * AD9910_RAM_WORD_SIZE)
#define AD9910_RAM_ADDR_MAX		(AD9910_RAM_SIZE_MAX_WORDS - 1)

#define AD9910_RAM_PROFILE_CTL_CONT_MSK	BIT(4)
#define AD9910_SPI_DATA_IDX		1
#define AD9910_SPI_DATA_LEN_MAX		sizeof(__be64)
#define AD9910_SPI_MESSAGE_LEN_MAX	(AD9910_SPI_DATA_IDX + AD9910_SPI_DATA_LEN_MAX)
#define AD9910_SPI_READ			BIT(7)
#define AD9910_SPI_ADDR_MASK		GENMASK(4, 0)

enum {
	AD9910_PROFILE,
	AD9910_POWERDOWN,
	AD9910_PP_HOLD_LAST_EN,
	AD9910_PP_FREQ_SCALE,
	AD9910_PP_FREQ_OFFSET,
	AD9910_PP_PHASE_OFFSET,
	AD9910_PP_AMP_OFFSET,
	AD9910_DRG_CTL_MODE,
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
	AD9910_OSK_MANUAL_EXTCTL,
	AD9910_OSK_AUTO_STEP,
};

struct ad9910_data {
	u32 sysclk_freq_hz;
	u32 dac_output_current;

	/* PLL configuration */
	u16 pll_charge_pump_current;
	u8 pll_multiplier;
	u8 pll_vco_range;

	bool ref_div2_en;
	u8 refclk_out_drv;

	/* Feature flags */
	bool inverse_sinc_enable;
	bool sine_output_enable;
	bool sync_clk_enable;
	bool pdclk_enable;
	bool pdclk_invert;
	bool tx_enable_invert;
};

struct ad9910_state {
	struct spi_device *spi;
	struct ad9910_backend *back;
	struct clk *refclk;

	struct gpio_desc *gpio_pwdown;
	struct gpio_desc *gpio_io_update;
	struct gpio_desc *gpio_profile[3];

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

	struct ad9910_data data;
	u8 profile;

	/*
	 * DMA (thus cache coherency maintenance) requires the transfer
	 * buffers to live in their own cache lines.
	 */
	u8 buf[AD9910_SPI_MESSAGE_LEN_MAX] __aligned(IIO_DMA_MINALIGN);
};

static const char * const ad9910_power_supplies[] = {
	"dvdd-io33", "avdd33", "dvdd18", "avdd18",
};

static const char * const ad9910_refclk_out_drv0[] = {
	"disabled", "low", "medium", "high",
};

static const char * const ad9910_channel_str[] = {
	[AD9910_CHANNEL_SINGLE_TONE] = "single_tone",
	[AD9910_CHANNEL_PARALLEL_PORT] = "parallel_port",
	[AD9910_CHANNEL_DRG] = "digital_ramp_generator",
	[AD9910_CHANNEL_RAM] = "ram_control",
	[AD9910_CHANNEL_OSK] = "output_shift_keying",
};

static const char * const ad9910_destination_str[] = {
	[AD9910_DEST_FREQUENCY] = "frequency",
	[AD9910_DEST_PHASE] = "phase",
	[AD9910_DEST_AMPLITUDE] = "amplitude",
	[AD9910_DEST_POLAR] = "polar",
};

static const char * const ad9910_drg_oper_mode_str[] = {
	[AD9910_DRG_OPER_MODE_BIDIR] = "bidirectional",
	[AD9910_DRG_OPER_MODE_RAMP_DOWN] = "ramp_down",
	[AD9910_DRG_OPER_MODE_RAMP_UP] = "ramp_up",
	[AD9910_DRG_OPER_MODE_BIDIR_CONT] = "bidirectional_continuous",
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
	AD9910_ICP_MIN_uA, 237, 262, 287, 312, 337, 363, AD9910_ICP_MAX_uA,
};

static const u16 ad9910_osk_ustep[] = {
	0, 61, 122, 244, 488,
};

/*
 * Helper struct to manage AD9910 backend registrations
 */
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
EXPORT_SYMBOL_NS_GPL(devm_ad9910_backend_register, "AD9910");

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
 * Closest rounding with mul_u64_add_u64_div_u64
 *
 * Return: The scaled value.
 */
#define ad9910_rational_scale(input, scale, reference) ({	\
	u64 _tmp = (reference);					\
	mul_u64_add_u64_div_u64(input, scale, _tmp >> 1, _tmp);	\
})

static int ad9910_io_update(struct ad9910_state *st)
{
	if (st->back)
		return ad9910_backend_op_call(st->back, io_update);

	if (st->gpio_io_update) {
		gpiod_set_value_cansleep(st->gpio_io_update, 1);
		udelay(1);
		gpiod_set_value_cansleep(st->gpio_io_update, 0);
	}

	return 0;
}

static inline int ad9910_spi_read(struct ad9910_state *st, u8 reg, size_t len)
{
	st->buf[0] = AD9910_SPI_READ | (reg & AD9910_SPI_ADDR_MASK);
	return spi_write_then_read(st->spi, &st->buf[0], 1,
				   &st->buf[AD9910_SPI_DATA_IDX], len);
}

static inline int ad9910_spi_write(struct ad9910_state *st, u8 reg, size_t len,
				   bool update)
{
	int ret;

	st->buf[0] = reg & AD9910_SPI_ADDR_MASK;
	ret = spi_write(st->spi, st->buf, AD9910_SPI_DATA_IDX + len);
	if (!ret && update)
		return ad9910_io_update(st);

	return ret;
}

static inline int ad9910_ram_load(struct ad9910_state *st, void* data,
				  size_t count)
{
	struct spi_transfer t[] = {
		{ .tx_buf = st->buf, .len = 1, },
		{ .tx_buf = data, .len = count, },
	};

	st->buf[0] = AD9910_REG_RAM;
	return spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
}

#define AD9910_REG_READ_FN(nb)						\
static inline int ad9910_reg##nb##_read(struct ad9910_state *st,	\
					u8 reg, u##nb *data)		\
{									\
	int ret;							\
									\
	ret = ad9910_spi_read(st, reg, sizeof(*data));			\
	if (ret)							\
		return ret;						\
									\
	*data = get_unaligned_be##nb(&st->buf[AD9910_SPI_DATA_IDX]);	\
	return ret;							\
}

AD9910_REG_READ_FN(16)
AD9910_REG_READ_FN(32)
AD9910_REG_READ_FN(64)

#define AD9910_REG_WRITE_FN(nb)						\
static inline int ad9910_reg##nb##_write(struct ad9910_state *st,	\
					 u8 reg, u##nb data,		\
					 bool update)			\
{									\
	int ret;							\
									\
	put_unaligned_be##nb(data, &st->buf[AD9910_SPI_DATA_IDX]);	\
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

static int ad9910_profile_set(struct ad9910_state *st, u8 profile)
{
	if (profile >= AD9910_NUM_PROFILES)
		return -EINVAL;

	st->profile = profile;
	if (st->back)
		return ad9910_backend_op_call(st->back, profile_set, profile);

	gpiod_set_value_cansleep(st->gpio_profile[0], profile & 0x01);
	gpiod_set_value_cansleep(st->gpio_profile[1], (profile >> 1) & 0x01);
	gpiod_set_value_cansleep(st->gpio_profile[2], (profile >> 2) & 0x01);

	return 0;
}

static int ad9910_powerdown_set(struct ad9910_state *st, bool enable)
{
	if (st->back)
		return ad9910_backend_op_call(st->back, powerdown_set, enable);

	return gpiod_set_value_cansleep(st->gpio_pwdown, enable);
}

static int ad9910_chan_destination_set(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	switch (chan->channel) {
	case AD9910_CHANNEL_DRG:
		return ad9910_reg32_update(st, AD9910_REG_CFR2,
					   AD9910_CFR2_DRG_DEST_MSK,
					   FIELD_PREP(AD9910_CFR2_DRG_DEST_MSK, val),
					   true);
	case AD9910_CHANNEL_RAM:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			return -EBUSY;

		return ad9910_reg32_update(st, AD9910_REG_CFR1,
					   AD9910_CFR1_RAM_PLAYBACK_DEST_MSK,
					   FIELD_PREP(AD9910_CFR1_RAM_PLAYBACK_DEST_MSK, val),
					   true);
	default:
		return -EINVAL;
	}
}

static int ad9910_chan_destination_get(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	switch (chan->channel) {
	case AD9910_CHANNEL_DRG:
		return FIELD_GET(AD9910_CFR2_DRG_DEST_MSK,
				 st->reg[AD9910_REG_CFR2].val32);
	case AD9910_CHANNEL_RAM:
		return FIELD_GET(AD9910_CFR1_RAM_PLAYBACK_DEST_MSK,
				 st->reg[AD9910_REG_CFR1].val32);
	default:
		return -EINVAL;
	}
}

static int ad9910_drg_oper_mode_set(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (st->back)
		ad9910_backend_op_call(st->back, drg_oper_mode_set, val);

	return ad9910_reg32_update(st, AD9910_REG_CFR2,
				   AD9910_CFR2_DRG_NO_DWELL_MSK,
				   FIELD_PREP(AD9910_CFR2_DRG_NO_DWELL_MSK, val),
				   true);
}

static int ad9910_drg_oper_mode_get(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return FIELD_GET(AD9910_CFR2_DRG_NO_DWELL_MSK,
			 st->reg[AD9910_REG_CFR2].val32);
}

static int ad9910_ram_oper_mode_set(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	u32 profile_ctl;
	int ret;

	guard(mutex)(&st->lock);

	/*
	 * RAM sequenced modes use the internal profile control:
	 *  - Sequence mode takes precedence over regular profile modes
	 *  - Active profile defines the internal profile control target
	 *  - Profile 0 cannot be used as sequenced mode target
	 *  - Profile X cannot be set as sequenced mode target if another
	 *    profile is currently set.
	 */
	profile_ctl = FIELD_GET(AD9910_CFR1_INT_PROFILE_CTL_MSK,
				st->reg[AD9910_REG_CFR1].val32);
	if (AD9910_RAM_PROFILE_CTL_CONT_MSK & profile_ctl)
		profile_ctl = (profile_ctl & ~AD9910_RAM_PROFILE_CTL_CONT_MSK) + 1;


	if (val >= AD9910_RAM_MODE_SEQ) {
		if (!st->profile)
			return -EINVAL;

		if (profile_ctl && profile_ctl != st->profile)
			return -EBUSY;

		/* update profile control */
		profile_ctl = st->profile;
		if (val == AD9910_RAM_MODE_SEQ_CONT)
			profile_ctl = AD9910_RAM_PROFILE_CTL_CONT_MSK | (profile_ctl - 1);
		profile_ctl = FIELD_PREP(AD9910_CFR1_INT_PROFILE_CTL_MSK, profile_ctl);
		return ad9910_reg32_update(st, AD9910_REG_CFR1,
					   AD9910_CFR1_INT_PROFILE_CTL_MSK,
					   profile_ctl, true);
	}

	if (profile_ctl && profile_ctl == st->profile) {
		/* clear internal profile control */
		ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
					  AD9910_CFR1_INT_PROFILE_CTL_MSK,
					  0, true);
		if (ret)
			return ret;
	}

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
					   AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
					   FIELD_PREP(AD9910_PROFILE_RAM_MODE_CONTROL_MSK, val),
					   true);

	FIELD_MODIFY(AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
		     &st->reg_profile[st->profile], val);
	return 0;
}

static int ad9910_ram_oper_mode_get(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	u32 profile_ctl;
	bool seq_cont = false;

	guard(mutex)(&st->lock);

	profile_ctl = FIELD_GET(AD9910_CFR1_INT_PROFILE_CTL_MSK,
				st->reg[AD9910_REG_CFR1].val32);
	if (AD9910_RAM_PROFILE_CTL_CONT_MSK & profile_ctl) {
		seq_cont = true;
		profile_ctl = (profile_ctl & ~AD9910_RAM_PROFILE_CTL_CONT_MSK) + 1;
	}

	if (profile_ctl && profile_ctl == st->profile)
		return (seq_cont) ? AD9910_RAM_MODE_SEQ_CONT : AD9910_RAM_MODE_SEQ;

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return FIELD_GET(AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
				 st->reg[AD9910_REG_PROFILE(st->profile)].val64);
	else
		return FIELD_GET(AD9910_PROFILE_RAM_MODE_CONTROL_MSK,
				 st->reg_profile[st->profile]);
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
	case AD9910_PROFILE:
		val = st->profile;
		break;
	case AD9910_POWERDOWN:
		val = !!FIELD_GET(AD9910_CFR1_SOFT_POWER_DOWN_MSK,
				  st->reg[AD9910_REG_CFR1].val32);
		break;
	case AD9910_PP_HOLD_LAST_EN:
		val = !!FIELD_GET(AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK,
				  st->reg[AD9910_REG_CFR2].val32);
		break;
	case AD9910_PP_FREQ_SCALE:
		val = BIT(FIELD_GET(AD9910_CFR2_FM_GAIN_MSK,
				    st->reg[AD9910_REG_CFR2].val32));
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
	case AD9910_OSK_MANUAL_EXTCTL:
		val = !!FIELD_GET(AD9910_CFR1_OSK_MANUAL_EXT_CTL_MSK,
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
	case AD9910_PROFILE:
		if (val32 > 7)
			return -EINVAL;
		ret = ad9910_profile_set(st, val32);
		break;
	case AD9910_POWERDOWN:
		val32 = val32 ? AD9910_CFR1_SOFT_POWER_DOWN_MSK : 0;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR1,
					  AD9910_CFR1_SOFT_POWER_DOWN_MSK,
					  val32, true);
		break;
	case AD9910_PP_HOLD_LAST_EN:
		val32 = val32 ? AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK : 0;
		ret = ad9910_reg32_update(st, AD9910_REG_CFR2,
					  AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK,
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
	case AD9910_RAM_START_ADDR:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			return -EBUSY;

		if (val32 > AD9910_RAM_ADDR_MAX)
			return -EINVAL;

		if (val32 > FIELD_GET(AD9910_PROFILE_RAM_END_ADDR_MSK,
				      st->reg_profile[st->profile]))
			FIELD_MODIFY(AD9910_PROFILE_RAM_END_ADDR_MSK,
				     &st->reg_profile[st->profile], val32);

		FIELD_MODIFY(AD9910_PROFILE_RAM_START_ADDR_MSK,
			     &st->reg_profile[st->profile], val32);
		break;
	case AD9910_RAM_END_ADDR:
		if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
			return -EBUSY;

		if (val32 > AD9910_RAM_ADDR_MAX ||
		    val32 < FIELD_GET(AD9910_PROFILE_RAM_START_ADDR_MSK,
				      st->reg_profile[st->profile]))
			return -EINVAL;

		FIELD_MODIFY(AD9910_PROFILE_RAM_END_ADDR_MSK,
			     &st->reg_profile[st->profile], val32);
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
		vals[0] = upper_32_bits(tmp64);
		vals[1] = upper_32_bits((u64)lower_32_bits(tmp64) * MICRO);
		break;
	case AD9910_PP_PHASE_OFFSET:
		tmp32 = FIELD_GET(AD9910_POW_PP_LSB_MSK,
				  st->reg[AD9910_REG_POW].val16);
		tmp32 = (tmp32 * AD9910_MAX_PHASE_MICRORAD) >> 16;
		vals[0] = tmp32 / MICRO;
		vals[1] = tmp32 % MICRO;
		break;
	case AD9910_PP_AMP_OFFSET:
		tmp32 = FIELD_GET(AD9910_ASF_AMP_SCALE_FACTOR_PP_LSB_MSK,
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

	ret = iio_str_to_fixpoint(buf, MICRO / 10, &val, &val2);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PP_FREQ_OFFSET:
		if (val < 0 || val >= st->data.sysclk_freq_hz / 2)
			return -EINVAL;

		tmp32 = ad9910_rational_scale((u64)val * MICRO + val2, BIT_ULL(32),
					      (u64)MICRO * st->data.sysclk_freq_hz);
		ret = ad9910_reg32_write(st, AD9910_REG_FTW, tmp32, true);
		break;
	case AD9910_PP_PHASE_OFFSET:
		if (val != 0 || val2 < 0 || val2 >= (AD9910_MAX_PHASE_MICRORAD >> 8))
			return -EINVAL;

		tmp32 = DIV_ROUND_CLOSEST((u32)val2 << 16, AD9910_MAX_PHASE_MICRORAD);
		tmp32 = min(tmp32, AD9910_POW_PP_LSB_MAX);
		tmp32 = FIELD_PREP(AD9910_POW_PP_LSB_MSK, tmp32);
		ret = ad9910_reg16_update(st, AD9910_REG_POW,
					  AD9910_POW_PP_LSB_MSK,
					  tmp32, true);
		break;
	case AD9910_PP_AMP_OFFSET:
		if (val != 0 || val2 < 0 || val2 >= (MICRO >> 8))
			return -EINVAL;

		tmp32 = DIV_ROUND_CLOSEST((u32)val2 << 14, MICRO);
		tmp32 = min(tmp32, AD9910_ASF_PP_LSB_MAX);
		tmp32 = FIELD_PREP(AD9910_ASF_AMP_SCALE_FACTOR_PP_LSB_MSK, tmp32);
		ret = ad9910_reg32_update(st, AD9910_REG_ASF,
					  AD9910_ASF_AMP_SCALE_FACTOR_PP_LSB_MSK,
					  tmp32, true);
		break;
	default:
		return -EINVAL;
	}

	return ret ?: len;
}

static ssize_t ad9910_step_rate_read(struct iio_dev *indio_dev,
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
	vals[0] = st->data.sysclk_freq_hz / tmp32;
	vals[1] = div_u64((u64)(st->data.sysclk_freq_hz % tmp32) * MICRO, tmp32);

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_step_rate_write(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u64 rate_val;
	u64 sysclk_uhz;
	int ret;

	ret = iio_str_to_fixpoint(buf, MICRO / 10, &val, &val2);
	if (ret)
		return ret;

	sysclk_uhz = (u64)st->data.sysclk_freq_hz * MICROHZ_PER_HZ;
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
		tmp64 *= st->data.sysclk_freq_hz;
		vals[0] = upper_32_bits(tmp64);
		vals[1] = upper_32_bits((u64)lower_32_bits(tmp64) * MICRO);
		break;
	case AD9910_DRG_PHASE_UPPER_LIMIT:
	case AD9910_DRG_PHASE_LOWER_LIMIT:
	case AD9910_DRG_PHASE_INC_STEP:
	case AD9910_DRG_PHASE_DEC_STEP:
		type = IIO_VAL_INT_PLUS_NANO;
		tmp64 *= AD9910_PI_NANORAD;
		tmp64 >>= 31;
		vals[0] = div_u64_rem(tmp64, NANO, &vals[1]);
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

static ssize_t ad9910_drg_attrs_write(struct iio_dev *indio_dev,
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
		ret = iio_str_to_fixpoint(buf, MICRO / 10, &val, &val2);
		if (ret)
			return ret;

		if (val >= st->data.sysclk_freq_hz / 2)
			return -EINVAL;

		tmp64 = (u64)val * MICRO + val2;
		tmp64 = ad9910_rational_scale(tmp64, BIT_ULL(32),
					      (u64)MICRO * st->data.sysclk_freq_hz);
		break;
	case AD9910_DRG_PHASE_UPPER_LIMIT:
	case AD9910_DRG_PHASE_LOWER_LIMIT:
	case AD9910_DRG_PHASE_INC_STEP:
	case AD9910_DRG_PHASE_DEC_STEP:
		ret = iio_str_to_fixpoint(buf, NANO / 10, &val, &val2);
		if (ret)
			return ret;

		if (val < 0 || val2 < 0)
			return -EINVAL;

		tmp64 = (u64)val * NANO + val2;
		if (tmp64 > 2ULL * AD9910_PI_NANORAD)
			return -EINVAL;

		tmp64 <<= 31;
		tmp64 = DIV_U64_ROUND_CLOSEST(tmp64, AD9910_PI_NANORAD);
		break;
	case AD9910_DRG_AMP_UPPER_LIMIT:
	case AD9910_DRG_AMP_LOWER_LIMIT:
	case AD9910_DRG_AMP_INC_STEP:
	case AD9910_DRG_AMP_DEC_STEP:
		ret = iio_str_to_fixpoint(buf, NANO / 10, &val, &val2);
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

	return ret ?: len;
}

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
			auto_en = !!FIELD_GET(AD9910_CFR1_SELECT_AUTO_OSK_MSK,
					st->reg[AD9910_REG_CFR1].val32);
			raw_val = FIELD_GET(AD9910_ASF_AMP_STEP_SIZE_MSK,
					st->reg[AD9910_REG_ASF].val32);
			vals[0] = 0;
			vals[1] = (auto_en)? ad9910_osk_ustep[raw_val + 1] : 0;

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

	ret = iio_str_to_fixpoint(buf, MICRO / 10, &val, &val2);
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
			raw_val = FIELD_PREP(AD9910_ASF_AMP_STEP_SIZE_MSK,
					     raw_val - 1);
			ret = ad9910_reg32_update(st, AD9910_REG_ASF,
						  AD9910_ASF_AMP_STEP_SIZE_MSK,
						  raw_val,
						  true);
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

static ssize_t sysclk_frequency_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%u\n", st->data.sysclk_freq_hz);
}

static ssize_t ram_data_write(struct file *filp, struct kobject *kobj,
			      const struct bin_attribute *attr, char *buf,
			      loff_t off, size_t count)
{
	struct ad9910_state *st = iio_priv(dev_to_iio_dev(kobj_to_dev(kobj)));
	u64 tmp64, backup;
	int ret, ret2;

	if (off + count > AD9910_RAM_SIZE_MAX_BYTES || !count ||
	    off % AD9910_RAM_WORD_SIZE != 0 ||
	    count % AD9910_RAM_WORD_SIZE != 0)
		return -EINVAL;

	guard(mutex)(&st->lock);

	if (!!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg[AD9910_REG_CFR1].val32))
		return -EBUSY;

	/* ensure profile is selected */
	ret = ad9910_profile_set(st, st->profile);
	if (ret)
		return ret;

	/* backup profile register */
	backup = st->reg[AD9910_REG_PROFILE(st->profile)].val64;
	tmp64 = AD9910_PROFILE_RAM_STEP_RATE_MSK |
		FIELD_PREP(AD9910_PROFILE_RAM_START_ADDR_MSK, off / AD9910_RAM_WORD_SIZE) |
		FIELD_PREP(AD9910_PROFILE_RAM_END_ADDR_MSK, (off + count) / AD9910_RAM_WORD_SIZE - 1);
	ret = ad9910_reg64_write(st, AD9910_REG_PROFILE(st->profile), tmp64, true);
	if (ret)
		return ret;

	/* write ram data and restore profile register */
	ret = ad9910_ram_load(st, buf, count);
	ret2 = ad9910_reg64_write(st, AD9910_REG_PROFILE(st->profile), backup, true);
	if (!ret)
		ret = ret2;

	return ret ?: count;
}

static IIO_DEVICE_ATTR_RO(sysclk_frequency, 0);
static const BIN_ATTR_WO(ram_data, AD9910_RAM_SIZE_MAX_BYTES);

static struct attribute *ad9910_attrs[] = {
	&iio_dev_attr_sysclk_frequency.dev_attr.attr,
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
	.set = ad9910_chan_destination_set,
	.get = ad9910_chan_destination_get,
};

static const struct iio_enum ad9910_drg_oper_mode_enum = {
	.items = ad9910_drg_oper_mode_str,
	.num_items = ARRAY_SIZE(ad9910_drg_oper_mode_str),
	.set = ad9910_drg_oper_mode_set,
	.get = ad9910_drg_oper_mode_get,
};

static const struct iio_enum ad9910_ram_destination_enum = {
	.items = ad9910_destination_str,
	.num_items = AD9910_RAM_DEST_NUM,
	.set = ad9910_chan_destination_set,
	.get = ad9910_chan_destination_get,
};

static const struct iio_enum ad9910_ram_oper_mode_enum = {
	.items = ad9910_ram_oper_mode_str,
	.num_items = ARRAY_SIZE(ad9910_ram_oper_mode_str),
	.set = ad9910_ram_oper_mode_set,
	.get = ad9910_ram_oper_mode_get,
};

#define AD9910_EXT_INFO_TMPL(_name, _ident, _shared, _fn_desc) { \
	.name = _name, \
	.read = ad9910_ ## _fn_desc ## _read, \
	.write = ad9910_ ## _fn_desc ## _write, \
	.private = _ident, \
	.shared = _shared, \
}

#define AD9910_EXT_INFO(_name, _ident, _shared) \
	AD9910_EXT_INFO_TMPL(_name, _ident, _shared, ext_info)

#define AD9910_STEP_RATE_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, step_rate)

#define AD9910_PP_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, pp_attrs)

#define AD9910_DRG_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, drg_attrs)

#define AD9910_OSK_EXT_INFO(_name, _ident) \
	AD9910_EXT_INFO_TMPL(_name, _ident, IIO_SEPARATE, osk_attrs)

static const struct iio_chan_spec_ext_info ad9910_shared_ext_info[] = {
	AD9910_EXT_INFO("profile", AD9910_PROFILE, IIO_SHARED_BY_TYPE),
	AD9910_EXT_INFO("powerdown", AD9910_POWERDOWN, IIO_SHARED_BY_TYPE),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_pp_ext_info[] = {
	AD9910_EXT_INFO("hold_last_en", AD9910_PP_HOLD_LAST_EN, IIO_SEPARATE),
	AD9910_EXT_INFO("frequency_scale", AD9910_PP_FREQ_SCALE, IIO_SEPARATE),
	AD9910_PP_EXT_INFO("frequency_offset", AD9910_PP_FREQ_OFFSET),
	AD9910_PP_EXT_INFO("phase_offset", AD9910_PP_PHASE_OFFSET),
	AD9910_PP_EXT_INFO("scale_offset", AD9910_PP_AMP_OFFSET),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_drg_ext_info[] = {
	IIO_ENUM("destination", IIO_SEPARATE, &ad9910_drg_destination_enum),
	IIO_ENUM_AVAILABLE("destination", IIO_SEPARATE, &ad9910_drg_destination_enum),
	IIO_ENUM("operating_mode", IIO_SEPARATE, &ad9910_drg_oper_mode_enum),
	IIO_ENUM_AVAILABLE("operating_mode", IIO_SEPARATE, &ad9910_drg_oper_mode_enum),
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

static const struct iio_chan_spec_ext_info ad9910_osk_ext_info[] = {
	AD9910_EXT_INFO("pinctrl_en", AD9910_OSK_MANUAL_EXTCTL, IIO_SEPARATE),
	AD9910_OSK_EXT_INFO("scale_increment", AD9910_OSK_AUTO_STEP),
	{ },
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

static const struct iio_chan_spec ad9910_channels[] = {
	[AD9910_CHANNEL_SINGLE_TONE] ={
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_SINGLE_TONE,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SCALE),
		.ext_info = ad9910_shared_ext_info,
	},
	[AD9910_CHANNEL_PARALLEL_PORT] ={
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_PARALLEL_PORT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_scan_type = ad9910_pp_scan_type,
		.num_ext_scan_type = ARRAY_SIZE(ad9910_pp_scan_type),
		.ext_info = ad9910_pp_ext_info,
		.has_ext_scan_type = 1
	},
	[AD9910_CHANNEL_DRG] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_DRG,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = ad9910_drg_ext_info,
	},
	[AD9910_CHANNEL_RAM] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_RAM,
		.scan_index = -1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE) |
				      BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.ext_info = ad9910_ram_ext_info,
	},
	[AD9910_CHANNEL_OSK] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_OSK,
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
	u32 tmp32, ram_en;

	guard(mutex)(&st->lock);

	ram_en = FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK,
			   st->reg[AD9910_REG_CFR1].val32);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->channel) {
		case AD9910_CHANNEL_PARALLEL_PORT:
			*val = FIELD_GET(AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_DRG:
			*val = FIELD_GET(AD9910_CFR2_DRG_ENABLE_MSK,
					 st->reg[AD9910_REG_CFR2].val32);
			break;
		case AD9910_CHANNEL_RAM:
			*val = ram_en;
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
		tmp64 = (u64)tmp32 * st->data.sysclk_freq_hz;
		*val = upper_32_bits(tmp64);
		*val2 = upper_32_bits((u64)lower_32_bits(tmp64) * MICRO);
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
		tmp32 = ((u64)tmp32 * AD9910_MAX_PHASE_MICRORAD) >> 16;
		*val = tmp32 / MICRO;
		*val2 = tmp32 % MICRO;
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
		case AD9910_CHANNEL_RAM:
			if (ram_en)
				tmp32 = FIELD_GET(AD9910_PROFILE_RAM_STEP_RATE_MSK,
						  st->reg[AD9910_REG_PROFILE(st->profile)].val64);
			else
				tmp32 = FIELD_GET(AD9910_PROFILE_RAM_STEP_RATE_MSK,
						  st->reg_profile[st->profile]);
			break;
		case AD9910_CHANNEL_OSK:
			tmp32 = FIELD_GET(AD9910_ASF_AMP_RAMP_RATE_MSK,
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
	int ram_en, ret = 0;

	guard(mutex)(&st->lock);

	ram_en = FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK,
			   st->reg[AD9910_REG_CFR1].val32);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		val = val ? 1 : 0;
		switch (chan->channel) {
		case AD9910_CHANNEL_PARALLEL_PORT:
			tmp32 = FIELD_PREP(AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK, val);
			return ad9910_reg32_update(st, AD9910_REG_CFR2,
						   AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_DRG:
			tmp32 = FIELD_PREP(AD9910_CFR2_DRG_ENABLE_MSK, val);
			return ad9910_reg32_update(st, AD9910_REG_CFR2,
						   AD9910_CFR2_DRG_ENABLE_MSK,
						   tmp32, true);
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

			tmp32 = FIELD_PREP(AD9910_CFR1_RAM_ENABLE_MSK, val);
			return ad9910_reg32_update(st, AD9910_REG_CFR1,
						   AD9910_CFR1_RAM_ENABLE_MSK,
						   tmp32, true);
		case AD9910_CHANNEL_OSK:
			tmp32 = FIELD_PREP(AD9910_CFR1_OSK_ENABLE_MSK, val);
			return ad9910_reg32_update(st, AD9910_REG_CFR1,
						   AD9910_CFR1_OSK_ENABLE_MSK,
						   tmp32, true);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_FREQUENCY:
		if (val < 0 || val >= st->data.sysclk_freq_hz / 2)
			return -EINVAL;

		tmp32 = ad9910_rational_scale((u64)val * MICRO + val2, BIT_ULL(32),
					      (u64)MICRO * st->data.sysclk_freq_hz);
		if (chan->channel != AD9910_CHANNEL_SINGLE_TONE)
			return ad9910_reg32_write(st, AD9910_REG_FTW, tmp32, true);

		if (!ram_en)
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
						   AD9910_PROFILE_ST_FTW_MSK,
						   FIELD_PREP(AD9910_PROFILE_ST_FTW_MSK, tmp32),
						   true);

		FIELD_MODIFY(AD9910_PROFILE_ST_FTW_MSK,
			     &st->reg_profile[st->profile], tmp32);
		break;
	case IIO_CHAN_INFO_PHASE:
		tmp64 = (u64)val * MICRO + val2;
		if (val < 0 || val2 < 0 || tmp64 >= AD9910_MAX_PHASE_MICRORAD)
			return -EINVAL;

		tmp32 = DIV_U64_ROUND_CLOSEST(tmp64 << 16, AD9910_MAX_PHASE_MICRORAD);
		tmp16 = min(tmp32, AD9910_POW_MAX);

		if (chan->channel != AD9910_CHANNEL_SINGLE_TONE)
			return ad9910_reg16_write(st, AD9910_REG_POW, tmp16, true);

		if (!ram_en)
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
						   AD9910_PROFILE_ST_POW_MSK,
						   FIELD_PREP(AD9910_PROFILE_ST_POW_MSK, tmp16),
						   true);

		FIELD_MODIFY(AD9910_PROFILE_ST_POW_MSK,
			     &st->reg_profile[st->profile], tmp16);
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

		if (!ram_en)
			return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
						   AD9910_PROFILE_ST_ASF_MSK,
						   FIELD_PREP(AD9910_PROFILE_ST_ASF_MSK, tmp16),
						   true);

		FIELD_MODIFY(AD9910_PROFILE_ST_ASF_MSK,
			     &st->reg_profile[st->profile], tmp16);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		tmp64 = ((u64)val * MICRO + val2) * 4;
		if (!tmp64)
			return -EINVAL;

		tmp64 = DIV64_U64_ROUND_CLOSEST((u64)st->data.sysclk_freq_hz * MICRO, tmp64);
		tmp32 = clamp(tmp64, 1U, AD9910_STEP_RATE_MAX);

		switch (chan->channel) {
		case AD9910_CHANNEL_RAM:
			if (ram_en) {
				tmp64 = FIELD_PREP(AD9910_PROFILE_RAM_STEP_RATE_MSK, tmp32);
				return ad9910_reg64_update(st, AD9910_REG_PROFILE(st->profile),
							   AD9910_PROFILE_RAM_STEP_RATE_MSK,
							   tmp64, true);
			}

			FIELD_MODIFY(AD9910_PROFILE_RAM_STEP_RATE_MSK,
				     &st->reg_profile[st->profile], tmp32);
			break;
		case AD9910_CHANNEL_OSK:
			return ad9910_reg32_update(st, AD9910_REG_ASF,
						   AD9910_ASF_AMP_RAMP_RATE_MSK,
						   FIELD_PREP(AD9910_ASF_AMP_RAMP_RATE_MSK, tmp32),
						   true);
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
	bool high32 = !!(reg & AD9910_REG_HIGH32_FLAG);

	reg &= ~AD9910_REG_HIGH32_FLAG;
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
				*readval = upper_32_bits(tmp64);
			else
				*readval = lower_32_bits(tmp64);
		} else {
			tmp64 = st->reg[reg].val64;
			if (high32)
				FIELD_MODIFY(AD9910_REG_HIGH32_MSK, &tmp64, writeval);
			else
				FIELD_MODIFY(AD9910_REG_LOW32_MSK, &tmp64, writeval);

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
	.attrs = &ad9910_attrs_group,
	.get_current_scan_type = ad9910_get_current_scan_type,
	.debugfs_reg_access = &ad9910_reg_access,
	.read_label = ad9910_read_label,
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
	u32 cp_index, cfr3 = AD9910_CFR3_OPEN_MSK;
	int ret;

	cfr3 |= FIELD_PREP(AD9910_CFR3_DRV0_MSK, st->data.refclk_out_drv);
	st->data.sysclk_freq_hz = clk_get_rate(st->refclk);

	if (st->data.pll_multiplier) {
		st->data.sysclk_freq_hz *= st->data.pll_multiplier;
		if (st->data.sysclk_freq_hz < AD9910_PLL_OUT_MIN_FREQ_HZ ||
		    st->data.sysclk_freq_hz > AD9910_PLL_OUT_MAX_FREQ_HZ) {
			dev_err(&st->spi->dev, "invalid vco frequency: %u Hz\n",
				st->data.sysclk_freq_hz);
			return -ERANGE;
		}

		if (st->data.pll_vco_range >= AD9910_VCO_RANGE_NUM) {
			if (st->data.sysclk_freq_hz <= AD9910_VCO0_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 0;
			else if (st->data.sysclk_freq_hz <= AD9910_VCO1_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 1;
			else if (st->data.sysclk_freq_hz <= AD9910_VCO2_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 2;
			else if (st->data.sysclk_freq_hz <= AD9910_VCO3_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 3;
			else if (st->data.sysclk_freq_hz <= AD9910_VCO4_RANGE_AUTO_MAX_HZ)
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
			st->data.sysclk_freq_hz >>= 1;
	}

	if (st->back) {
		ret = iio_backend_set_sampling_freq(st->back->iio_back,
						    AD9910_CHANNEL_PARALLEL_PORT,
						    st->data.sysclk_freq_hz / 4);
		if (ret)
			return ret;
	}

	return ad9910_reg32_write(st, AD9910_REG_CFR3, cfr3, update);
}

static int ad9910_parse_fw(struct ad9910_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	ret = device_property_read_u32(dev, "adi,pll-multiplier", &tmp);
	if (!ret) {
		if (tmp < AD9910_PLL_MIN_N && tmp > AD9910_PLL_MAX_N)
			return dev_err_probe(dev, -ERANGE,
					     "invalid PLL multiplier %u\n", tmp);
		st->data.pll_multiplier = tmp;

		tmp = AD9910_VCO_RANGE_NUM;
		ret = device_property_read_u32(dev, "adi,pll-vco-range", &tmp);
		if (!ret && tmp >= AD9910_VCO_RANGE_NUM)
			return dev_err_probe(dev, -ERANGE,
					     "invalid VCO range: %u\n", tmp);
		st->data.pll_vco_range = tmp;

		tmp = AD9910_ICP_MAX_uA;
		device_property_read_u32(dev, "adi,charge-pump-current-microamp", &tmp);
		if (tmp < AD9910_ICP_MIN_uA && tmp > AD9910_ICP_MAX_uA)
			return dev_err_probe(dev, -ERANGE,
					     "invalid charge pump current %u\n", tmp);
		st->data.pll_charge_pump_current = tmp;

		st->data.refclk_out_drv = AD9910_REFCLK_OUT_DRV_DISABLED;
		ret = device_property_match_property_string(dev,
							    "adi,refclk-out-drive-strength",
							    ad9910_refclk_out_drv0,
							    ARRAY_SIZE(ad9910_refclk_out_drv0));
		if (ret >= 0)
			st->data.refclk_out_drv = ret;
	}

	st->data.ref_div2_en = device_property_read_bool(dev, "adi,reference-div2-enable");
	st->data.inverse_sinc_enable = device_property_read_bool(dev, "adi,inverse-sinc-enable");
	st->data.sine_output_enable = device_property_read_bool(dev, "adi,sine-output-enable");
	st->data.sync_clk_enable = !device_property_read_bool(dev, "adi,sync-clk-disable");
	st->data.pdclk_enable = !device_property_read_bool(dev, "adi,pdclk-disable");
	st->data.pdclk_invert = device_property_read_bool(dev, "adi,pdclk-invert");
	st->data.tx_enable_invert = device_property_read_bool(dev, "adi,tx-enable-invert");

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
	int ret;

	ret = reset_control_deassert(dev_rst);
	if (ret)
		return ret;

	reg32 = AD9910_CFR1_SDIO_INPUT_ONLY_MSK;
	reg32 |= FIELD_PREP(AD9910_CFR1_INV_SINC_EN_MSK, st->data.inverse_sinc_enable) |
		 FIELD_PREP(AD9910_CFR1_SELECT_SINE_MSK, st->data.sine_output_enable);

	ret = ad9910_reg32_write(st, AD9910_REG_CFR1, reg32, false);
	if (ret)
		return ret;

	reg32 = AD9910_CFR2_AMP_SCALE_SINGLE_TONE_MSK;
	reg32 |= AD9910_CFR2_SYNC_TIMING_VAL_DISABLE_MSK |
		 AD9910_CFR2_DRG_NO_DWELL_MSK |
		 AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK |
		 FIELD_PREP(AD9910_CFR2_SYNC_CLK_EN_MSK, st->data.sync_clk_enable) |
		 FIELD_PREP(AD9910_CFR2_PDCLK_ENABLE_MSK, st->data.pdclk_enable) |
		 FIELD_PREP(AD9910_CFR2_PDCLK_INVERT_MSK, st->data.pdclk_invert) |
		 FIELD_PREP(AD9910_CFR2_TXENABLE_INVERT_MSK, st->data.tx_enable_invert);

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
	reg32 = FIELD_PREP(AD9910_ASF_AMP_RAMP_RATE_MSK, 1);
	ret = ad9910_reg32_write(st, AD9910_REG_ASF, reg32, false);
	if (ret)
		return ret;

	reg32 = FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, 1) |
		FIELD_PREP(AD9910_DRG_RATE_INC_MSK, 1);
	ret = ad9910_reg32_write(st, AD9910_REG_DRG_RATE, reg32, false);
	if (ret)
		return ret;

	for (int i = 0; i < AD9910_NUM_PROFILES; i++) {
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
	for (ch_idx = AD9910_CHANNEL_PARALLEL_PORT;
	     ch_idx < ARRAY_SIZE(ad9910_channels); ch_idx++) {
		orig_ext_info = channels[ch_idx].ext_info;
		channels[ch_idx].ext_info = NULL;

		iio_backend_extend_chan_spec(iio_back, &channels[ch_idx]);
		if (!channels[ch_idx].ext_info) {
			channels[ch_idx].ext_info = orig_ext_info;
			continue;
		}

		orig_num = 0;
		tmp_ext_info = orig_ext_info;
		while (tmp_ext_info && tmp_ext_info->name) {
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

static void ad9910_power_down(void *data)
{
	struct ad9910_state *st = data;

	if (!ad9910_powerdown_set(st, true))
		return;

	ad9910_reg32_update(st, AD9910_REG_CFR1,
			    AD9910_CFR1_SOFT_POWER_DOWN_MSK,
			    AD9910_CFR1_SOFT_POWER_DOWN_MSK,
			    true);
}

static int ad9910_probe(struct spi_device *spi)
{
	struct reset_control *dev_rst, *io_rst;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad9910_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	spi_set_drvdata(spi, indio_dev);

	st->refclk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(st->refclk))
		return -EPROBE_DEFER;

	ret = devm_regulator_bulk_get_enable(dev,
					     ARRAY_SIZE(ad9910_power_supplies),
					     ad9910_power_supplies);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9910_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9910_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9910_channels);

	dev_rst = devm_reset_control_get_optional_exclusive(dev, "dev");
	if (IS_ERR(dev_rst))
		return dev_err_probe(dev, PTR_ERR(dev_rst),
				     "failed to get device reset control\n");

	ret = reset_control_assert(dev_rst);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to assert device reset control\n");

	io_rst = devm_reset_control_get_optional_exclusive_deasserted(dev, "io");
	if (IS_ERR(io_rst))
		return dev_err_probe(dev, PTR_ERR(io_rst),
				     "failed to get io reset control\n");

	st->back = devm_ad9910_backend_get_optional(dev);
	if (IS_ERR(st->back))
		return dev_err_probe(dev, PTR_ERR(st->back),
				     "failed to get iio backend\n");

	if (st->back) {
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
		st->gpio_pwdown = devm_gpiod_get_optional(dev, "powerdown",
							  GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_pwdown))
			return dev_err_probe(dev, PTR_ERR(st->gpio_pwdown),
					     "failed to get powerdown gpio\n");

		st->gpio_io_update = devm_gpiod_get_optional(dev, "update",
							     GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_io_update))
			return dev_err_probe(dev, PTR_ERR(st->gpio_io_update),
					     "failed to get io update gpio\n");

		st->gpio_profile[0] = devm_gpiod_get_index_optional(dev,
								    "profile", 0,
								    GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_profile[0]))
			return dev_err_probe(dev, PTR_ERR(st->gpio_profile[0]),
					     "failed to get profile0 gpio\n");

		st->gpio_profile[1] = devm_gpiod_get_index_optional(dev,
								    "profile", 1,
								    GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_profile[1]))
			return dev_err_probe(dev, PTR_ERR(st->gpio_profile[1]),
					     "failed to get profile1 gpio\n");

		st->gpio_profile[2] = devm_gpiod_get_index_optional(dev,
								    "profile", 2,
								    GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_profile[2]))
			return dev_err_probe(dev, PTR_ERR(st->gpio_profile[2]),
					     "failed to get profile2 gpio\n");
	}

	ret = ad9910_parse_fw(st);
	if (ret)
		return ret;

	ret = ad9910_setup(st, dev_rst);
	if (ret)
		return dev_err_probe(dev, ret, "device setup failed\n");

	ret = devm_add_action_or_reset(dev, ad9910_power_down, st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add power down action\n");

	ret = devm_iio_device_register(dev, indio_dev);
	if (!ret && st->back)
		iio_backend_debugfs_add(st->back->iio_back, indio_dev);

	return ret;
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
MODULE_IMPORT_NS("IIO_BACKEND");
