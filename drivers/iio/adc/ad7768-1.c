// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7768-1 SPI ADC driver
 *
 * Copyright 2017 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/gpio/driver.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include "linux/util_macros.h"
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine-ex.h>
#include <linux/units.h>
#include <linux/rational.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

/* AD7768 registers definition */
#define AD7768_REG_CHIP_TYPE		0x3
#define AD7768_REG_PROD_ID_L		0x4
#define AD7768_REG_PROD_ID_H		0x5
#define AD7768_REG_CHIP_GRADE		0x6
#define AD7768_REG_SCRATCH_PAD		0x0A
#define AD7768_REG_VENDOR_L		0x0C
#define AD7768_REG_VENDOR_H		0x0D
#define AD7768_REG_INTERFACE_FORMAT	0x14
#define AD7768_REG_POWER_CLOCK		0x15
#define AD7768_REG_ANALOG		0x16
#define AD7768_REG_ANALOG2		0x17
#define AD7768_REG_CONVERSION		0x18
#define AD7768_REG_DIGITAL_FILTER	0x19
#define AD7768_REG_SINC3_DEC_RATE_MSB	0x1A
#define AD7768_REG_SINC3_DEC_RATE_LSB	0x1B
#define AD7768_REG_DUTY_CYCLE_RATIO	0x1C
#define AD7768_REG_SYNC_RESET		0x1D
#define AD7768_REG_GPIO_CONTROL		0x1E
#define AD7768_REG_GPIO_WRITE		0x1F
#define AD7768_REG_GPIO_READ		0x20
#define AD7768_REG_OFFSET_HI		0x21
#define AD7768_REG_OFFSET_MID		0x22
#define AD7768_REG_OFFSET_LO		0x23
#define AD7768_REG_GAIN_HI		0x24
#define AD7768_REG_GAIN_MID		0x25
#define AD7768_REG_GAIN_LO		0x26
#define AD7768_REG_SPI_DIAG_ENABLE	0x28
#define AD7768_REG_ADC_DIAG_ENABLE	0x29
#define AD7768_REG_DIG_DIAG_ENABLE	0x2A
#define AD7768_REG_ADC_DATA		0x2C
#define AD7768_REG_MASTER_STATUS	0x2D
#define AD7768_REG_SPI_DIAG_STATUS	0x2E
#define AD7768_REG_ADC_DIAG_STATUS	0x2F
#define AD7768_REG_DIG_DIAG_STATUS	0x30
#define AD7768_REG_MCLK_COUNTER		0x31

/* AD7768_REG_POWER_CLOCK */
#define AD7768_PWR_MCLK_DIV_MSK		GENMASK(5, 4)
#define AD7768_PWR_MCLK_DIV(x)		FIELD_PREP(AD7768_PWR_MCLK_DIV_MSK, x)
#define AD7768_PWR_PWRMODE_MSK		GENMASK(1, 0)
#define AD7768_PWR_PWRMODE(x)		FIELD_PREP(AD7768_PWR_PWRMODE_MSK, x)

/* AD7768_REG_DIGITAL_FILTER */
#define AD7768_DIG_FIL_FIL_MSK		GENMASK(6, 4)
#define AD7768_DIG_FIL_FIL(x)		FIELD_PREP(AD7768_DIG_FIL_FIL_MSK, x)
#define AD7768_DIG_FIL_DEC_MSK		GENMASK(2, 0)
#define AD7768_DIG_FIL_DEC_RATE(x)	FIELD_PREP(AD7768_DIG_FIL_DEC_MSK, x)

/* AD7768_SINC3_DEC_RATE */
#define AD7768_SINC3_DEC_RATE_MSB_MSK	GENMASK(12, 8)
#define AD7768_SINC3_DEC_RATE_LSB_MSK	GENMASK(7, 0)

/* AD7768_REG_CONVERSION */
#define AD7768_CONV_MODE_MSK		GENMASK(2, 0)
#define AD7768_CONV_MODE(x)		FIELD_PREP(AD7768_CONV_MODE_MSK, x)

/* AD7768_REG_GPIO_CONTROL */
#define AD7768_GPIO_CONTROL_MSK		GENMASK(3, 0)
#define AD7768_GPIO_UNIVERSAL_EN	BIT(7)
#define AD7768_GPIO_PGIA_EN		(AD7768_GPIO_UNIVERSAL_EN | GENMASK(2, 0))

/* AD7768_REG_GPIO_WRITE */
#define AD7768_GPIO_WRITE_MSK		GENMASK(3, 0)
#define AD7768_GPIO_WRITE(x)		FIELD_PREP(AD7768_GPIO_WRITE_MSK, x)

/* AD7768_REG_GPIO_READ */
#define AD7768_GPIO_READ_MSK		GENMASK(3, 0)
#define AD7768_GPIO_READ(x)		FIELD_PREP(AD7768_GPIO_READ_MSK, x)

/* AD7768_REG_CONVLEN */
#define AD7768_REG_CONVLEN_MSK		GENMASK(3, 3)
#define AD7768_REG_CONVLEN(x)		FIELD_PREP(AD7768_REG_CONVLEN_MSK, x)

#define AD7768_GPIO_INPUT(x)		0x00
#define AD7768_GPIO_OUTPUT(x)		BIT(x)

#define AD7768_RD_FLAG_MSK(x)		(BIT(6) | ((x) & 0x3F))
#define AD7768_WR_FLAG_MSK(x)		((x) & 0x3F)

#define AD7768_CHAN_INFO_NONE		0

#define ADAQ776X_GAIN_MAX_NANO		(128 * NANO)
#define ADAQ776X_MAX_GAIN_MODES		8

enum ad7768_conv_mode {
	AD7768_CONTINUOUS,
	AD7768_ONE_SHOT,
	AD7768_SINGLE,
	AD7768_PERIODIC,
	AD7768_STANDBY
};

enum ad7768_pwrmode {
	AD7768_ECO_MODE = 0,
	AD7768_MED_MODE = 2,
	AD7768_FAST_MODE = 3
};

enum ad7768_mclk_div {
	AD7768_MCLK_DIV_16,
	AD7768_MCLK_DIV_8,
	AD7768_MCLK_DIV_4,
	AD7768_MCLK_DIV_2
};

enum ad7768_flt_mode {
	SINC5,
	SINC5_DEC_X8,
	SINC5_DEC_X16,
	SINC3,
	WIDEBAND
};

enum ad7768_dec_rate {
	AD7768_DEC_RATE_32 = 0,
	AD7768_DEC_RATE_64 = 1,
	AD7768_DEC_RATE_128 = 2,
	AD7768_DEC_RATE_256 = 3,
	AD7768_DEC_RATE_512 = 4,
	AD7768_DEC_RATE_1024 = 5
};

enum ad7768_scan_type {
	AD7768_SCAN_TYPE_NORMAL,
	AD7768_SCAN_TYPE_HIGH_SPEED,
};

struct ad7768_clk_configuration {
	enum ad7768_mclk_div mclk_div;
	enum ad7768_dec_rate dec_rate;
	unsigned int clk_div;
};

enum {
	AD7768_PGA_GAIN_0,
	AD7768_PGA_GAIN_1,
	AD7768_PGA_GAIN_2,
	AD7768_PGA_GAIN_3,
	AD7768_PGA_GAIN_4,
	AD7768_PGA_GAIN_5,
	AD7768_PGA_GAIN_6,
	AD7768_PGA_GAIN_7,
	AD7768_MAX_PGA_GAIN,
};

enum {
	AD7768_AAF_IN1,
	AD7768_AAF_IN2,
	AD7768_AAF_IN3,
};

/*
 * Gains computed as fractions of 1000 so they can be expressed by integers.
 */
static const int adaq7768_gains[7] = {
	[AD7768_PGA_GAIN_0] = 325,
	[AD7768_PGA_GAIN_1] = 650,
	[AD7768_PGA_GAIN_2] = 1300,
	[AD7768_PGA_GAIN_3] = 2600,
	[AD7768_PGA_GAIN_4] = 5200,
	[AD7768_PGA_GAIN_5] = 10400,
	[AD7768_PGA_GAIN_6] = 20800
};

static const int adaq7769_gains[8] = {
	[AD7768_PGA_GAIN_0] = 1000,
	[AD7768_PGA_GAIN_1] = 2000,
	[AD7768_PGA_GAIN_2] = 4000,
	[AD7768_PGA_GAIN_3] = 8000,
	[AD7768_PGA_GAIN_4] = 16000,
	[AD7768_PGA_GAIN_5] = 32000,
	[AD7768_PGA_GAIN_6] = 64000,
	[AD7768_PGA_GAIN_7] = 128000
};

static const int ad7768_aaf_gains[3] = {
	[AD7768_AAF_IN1] = 1000,
	[AD7768_AAF_IN2] = 364,
	[AD7768_AAF_IN3] = 143
};

static const char * const ad7768_vcm_modes[] = {
	"(AVDD1-AVSS)/2",
	"2V5",
	"2V05",
	"1V9",
	"1V65",
	"1V1",
	"0V9",
	"OFF",
};

struct ad7768_clk_div_range {
	unsigned int clk_div_min;
	unsigned int clk_div_max;
};

static const struct ad7768_clk_div_range ad7768_clk_div_ranges[] = {
	[SINC5] = {.clk_div_min = 64, .clk_div_max = 16384},
	[SINC5_DEC_X8] = {.clk_div_min = 16, .clk_div_max = 128},
	[SINC5_DEC_X16] = {.clk_div_min = 32, .clk_div_max = 256},
	[SINC3] = {.clk_div_min = 64, .clk_div_max = 327680},
	[WIDEBAND] = {.clk_div_min = 64, .clk_div_max = 16384},
};

static const struct ad7768_clk_configuration ad7768_sinc5_wideband_clk_conf[] = {
	{ AD7768_MCLK_DIV_2, AD7768_DEC_RATE_32, 64},
	{ AD7768_MCLK_DIV_2, AD7768_DEC_RATE_64, 128 },
	{ AD7768_MCLK_DIV_2, AD7768_DEC_RATE_128, 256 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_128, 512 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_256, 1024 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_512, 2048 },
	{ AD7768_MCLK_DIV_4, AD7768_DEC_RATE_1024, 4096 },
	{ AD7768_MCLK_DIV_8, AD7768_DEC_RATE_1024, 8192 },
	{ AD7768_MCLK_DIV_16, AD7768_DEC_RATE_1024, 16384 },
};

static int ad7768_freq_available_range[5][3];

static const int ad7768_mclk_div_rates[4] = {
	16, 8, 4, 2
};

static const int dec_rate_values[6] = {
	32, 64, 128, 256, 512, 1024,
};

static const  int sinc3_dec_rate_max_values[4] = {
	20480, 40960, 81920, 163840,
};

static const struct iio_scan_type ad7768_scan_type[] = {
	[AD7768_SCAN_TYPE_NORMAL] = {
		.sign = 's',
		.realbits = 24,
		.storagebits = 32,
	},
	[AD7768_SCAN_TYPE_HIGH_SPEED] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
	},
};

static const char * const ad7768_filter_enum[] = {
	[SINC5] = "sinc5",
	[SINC5_DEC_X8] = "sinc5-dec8",
	[SINC5_DEC_X16] = "sinc5-dec16",
	[SINC3] = "sinc3",
	[WIDEBAND] = "wideband",
};

static const char * const ad7768_pwr_mode_enum[] = {
	[AD7768_ECO_MODE]   = "eco_mode",
	[AD7768_MED_MODE]   = "med_mode",
	[AD7768_FAST_MODE]  = "fast_mode",
};

static int ad7768_get_vcm(struct iio_dev *dev, const struct iio_chan_spec *chan);
static int ad7768_set_vcm(struct iio_dev *dev, const struct iio_chan_spec *chan,
			  unsigned int mode);
static int ad7768_get_dig_fil_attr(struct iio_dev *dev,
			      const struct iio_chan_spec *chan);
static int ad7768_set_dig_fil_attr(struct iio_dev *dev,
			      const struct iio_chan_spec *chan, unsigned int filter);
static int ad7768_get_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan);
static int ad7768_set_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan, unsigned int pwr_mode);

static const struct iio_enum ad7768_pwr_mode_iio_enum = {
	.items = ad7768_pwr_mode_enum,
	.num_items = ARRAY_SIZE(ad7768_pwr_mode_enum),
	.set = ad7768_set_pwr_mode,
	.get = ad7768_get_pwr_mode,
};

static const struct iio_enum ad7768_flt_type_iio_enum = {
	.items = ad7768_filter_enum,
	.num_items = ARRAY_SIZE(ad7768_filter_enum),
	.set = ad7768_set_dig_fil_attr,
	.get = ad7768_get_dig_fil_attr,
};

static const struct iio_enum ad7768_vcm_mode_enum = {
	.items = ad7768_vcm_modes,
	.num_items = ARRAY_SIZE(ad7768_vcm_modes),
	.set = ad7768_set_vcm,
	.get = ad7768_get_vcm,
};

static struct iio_chan_spec_ext_info ad7768_ext_info[] = {
	IIO_ENUM("common_mode_voltage",
		 IIO_SHARED_BY_ALL,
		 &ad7768_vcm_mode_enum),
	IIO_ENUM_AVAILABLE("common_mode_voltage",
			   IIO_SHARED_BY_ALL,
			   &ad7768_vcm_mode_enum),
	IIO_ENUM("power_mode", IIO_SHARED_BY_ALL, &ad7768_pwr_mode_iio_enum),
	IIO_ENUM_AVAILABLE("power_mode", IIO_SHARED_BY_ALL, &ad7768_pwr_mode_iio_enum),
	IIO_ENUM("filter_mode", IIO_SHARED_BY_ALL, &ad7768_flt_type_iio_enum),
	IIO_ENUM_AVAILABLE("filter_mode", IIO_SHARED_BY_ALL, &ad7768_flt_type_iio_enum),
	{ },
};

#define AD7768_CHAN(_idx, _msk_avail) {	\
		.type = IIO_VOLTAGE,\
		.info_mask_separate_available = _msk_avail,\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.ext_info = ad7768_ext_info,\
		.indexed = 1,\
		.channel = _idx,\
		.scan_index = 0,\
		.has_ext_scan_type = 1,\
		.ext_scan_type = ad7768_scan_type,\
		.num_ext_scan_type = ARRAY_SIZE(ad7768_scan_type),\
}

static const struct iio_chan_spec ad7768_channels[] = {
	AD7768_CHAN(0, AD7768_CHAN_INFO_NONE),
};

static const struct iio_chan_spec adaq776x_channels[] = {
	AD7768_CHAN(0, BIT(IIO_CHAN_INFO_SCALE)),
};

struct ad7768_chip_info {
	const char *name;
	bool has_variable_aaf;
	bool has_pga;
	int num_pga_modes;
	int default_pga_mode;
	int pgia_mode2pin_offset;
	const int *pga_gains;
	const struct iio_chan_spec *channel_spec;
	const unsigned long *available_masks;
	int num_channels;
};

struct ad7768_state {
	const struct ad7768_chip_info *chip;
	struct spi_device *spi;
	struct regulator *regulator;
	int bits_per_word;
	int vref;
	int pga_gain_mode;
	int aaf_gain;
	struct mutex lock;
	struct clk *mclk;
	struct gpio_chip gpiochip;
	struct spi_transfer offload_xfer;
	struct spi_message offload_msg;
	unsigned int gpio_avail_map;
	unsigned int mclk_freq;
	unsigned int samp_freq;
	unsigned int common_mode_voltage;
	struct completion completion;
	struct iio_trigger *trig;
	struct gpio_desc *gpio_sync_in;
	const char *labels[ARRAY_SIZE(ad7768_channels)];
	int scale_tbl[ADAQ776X_MAX_GAIN_MODES][2];
	struct gpio_desc *gpio_reset;
	enum ad7768_flt_mode filter_mode;
	enum ad7768_pwrmode pwrmode;
	bool spi_is_dma_mapped;
	bool en_spi_sync;
	bool en_gpio_sync;
	int irq;
	/*
	 * DMA (thus cache coherency maintenance) may require the
	 * transfer buffers to live in their own cache lines.
	 */
	union {
		unsigned char buf[6];
		__be32 word;
		struct {
			__be32 chan;
			s64 timestamp;
		} scan;
	} data __aligned(IIO_DMA_MINALIGN);
};

static int ad7768_set_freq(struct ad7768_state *st,
			   unsigned int freq);

static int ad7768_spi_reg_read(struct ad7768_state *st, unsigned int addr,
			       unsigned int *data, unsigned int len)
{
	struct spi_transfer xfer = {
		.rx_buf = st->data.buf,
		.len = len + 1,
		.bits_per_word = (len == 3 ? 32 : 16),
	};
	unsigned char tx_data[4];
	int ret;

	tx_data[len] = AD7768_RD_FLAG_MSK(addr);
	xfer.tx_buf = tx_data;
	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret < 0)
		return ret;
	*data = (len == 1 ? st->data.buf[0] : st->data.word);

	return ret;
}

static int ad7768_spi_reg_write(struct ad7768_state *st,
				unsigned int addr,
				unsigned int val)
{
	struct spi_transfer xfer = {
		.rx_buf = st->data.buf,
		.len = 2,
		.bits_per_word = 16,
	};
	unsigned char tx_data[2];

	tx_data[1] = AD7768_WR_FLAG_MSK(addr);
	tx_data[0] = val & 0xFF;
	xfer.tx_buf = tx_data;
	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad7768_spi_reg_write_masked(struct ad7768_state *st,
				       unsigned int addr,
				       unsigned int mask,
				       unsigned int val)
{
	unsigned int reg_val;
	int ret;

	ret = ad7768_spi_reg_read(st, addr, &reg_val, 1);
	if (ret < 0)
		return ret;

	return ad7768_spi_reg_write(st, addr, (reg_val & ~mask) | val);
}

static int ad7768_send_sync_pulse(struct ad7768_state *st)
{
	int ret = 0;

	if (st->en_spi_sync)
		ret = ad7768_spi_reg_write(st, AD7768_REG_SYNC_RESET, 0x00);

	if (st->en_gpio_sync) {
		gpiod_set_value(st->gpio_sync_in, 1);
		gpiod_set_value(st->gpio_sync_in, 0);
	}

	return ret;
}

static int ad7768_set_mclk_div(struct ad7768_state *st, unsigned int mclk_div)
{
	unsigned int mclk_div_value;
	int ret;

	mutex_lock(&st->lock);

	ret = ad7768_spi_reg_read(st, AD7768_REG_POWER_CLOCK, &mclk_div_value, 1);
	if (ret)
		goto out;

	mclk_div_value &= ~AD7768_PWR_MCLK_DIV_MSK;
	mclk_div_value |= AD7768_PWR_MCLK_DIV(mclk_div);
	ret = ad7768_spi_reg_write(st, AD7768_REG_POWER_CLOCK, mclk_div_value);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad7768_set_mode(struct ad7768_state *st,
			   enum ad7768_conv_mode mode)
{
	int ret, regval;

	ret = ad7768_spi_reg_read(st, AD7768_REG_CONVERSION, &regval, 1);
	if (ret < 0)
		return ret;

	regval &= ~AD7768_CONV_MODE_MSK;
	regval |= AD7768_CONV_MODE(mode);

	return ad7768_spi_reg_write(st, AD7768_REG_CONVERSION, regval);
}

static int ad7768_scan_direct(struct iio_dev *indio_dev)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int readval, ret;

	if (!st->spi_is_dma_mapped)
		reinit_completion(&st->completion);

	ret = ad7768_set_mode(st, AD7768_ONE_SHOT);
	if (ret < 0)
		return ret;

	if (!st->spi_is_dma_mapped) {
		ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
		if (!ret)
			return -ETIMEDOUT;
	}

	ret = ad7768_spi_reg_read(st, AD7768_REG_ADC_DATA, &readval, 3);
	if (ret < 0)
		return ret;

	if (st->filter_mode == SINC5_DEC_X8)
		readval = readval >> 8;
	/*
	 * Any SPI configuration of the AD7768-1 can only be
	 * performed in continuous conversion mode.
	 */
	ret = ad7768_set_mode(st, AD7768_CONTINUOUS);
	if (ret < 0)
		return ret;

	return readval;
}

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval) {
		ret = ad7768_spi_reg_read(st, reg, readval, 1);
		if (ret < 0)
			goto err_unlock;
	} else {
		ret = ad7768_spi_reg_write(st, reg, writeval);
	}
err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_set_sinc3_dec_rate(struct ad7768_state *st, unsigned int dec_rate)
{
	unsigned int dec_rate_msb, dec_rate_lsb;
	int ret;

	mutex_lock(&st->lock);

	dec_rate = clamp_t(unsigned int, dec_rate, 0, 5119);
	dec_rate_msb = FIELD_GET(AD7768_SINC3_DEC_RATE_MSB_MSK, dec_rate);
	dec_rate_lsb = FIELD_GET(AD7768_SINC3_DEC_RATE_LSB_MSK, dec_rate);

	ret = ad7768_spi_reg_write(st, AD7768_REG_SINC3_DEC_RATE_MSB, dec_rate_msb);
	if (ret < 0)
		goto out;

	ret = ad7768_spi_reg_write(st, AD7768_REG_SINC3_DEC_RATE_LSB, dec_rate_lsb);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad7768_set_dec_rate(struct ad7768_state *st, unsigned int dec_rate)
{
	unsigned int mode;
	int ret;

	mutex_lock(&st->lock);

	ret = ad7768_spi_reg_read(st, AD7768_REG_DIGITAL_FILTER, &mode, 1);
	if (ret)
		goto out;

	mode &= ~AD7768_DIG_FIL_DEC_MSK;
	mode |= AD7768_DIG_FIL_DEC_RATE(dec_rate);
	ret = ad7768_spi_reg_write(st, AD7768_REG_DIGITAL_FILTER, mode);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static void ad7768_fill_scale_tbl(struct ad7768_state *st)
{
	int val, val2, tmp0, tmp1, i;
	unsigned long denominator, numerator;
	u64 tmp2;

	val2 = st->bits_per_word - 1;
	for (i = 0; i < st->chip->num_pga_modes; i++) {
		/* Convert gain to a fraction format */
		numerator = st->chip->pga_gains[i];
		denominator = MILLI;
		if (st->chip->has_variable_aaf) {
			numerator *= ad7768_aaf_gains[st->aaf_gain];
			denominator *= MILLI;
		}
		rational_best_approximation(numerator, denominator, __INT_MAX__, __INT_MAX__,
					    &numerator, &denominator);

		val = st->vref / 1000;
		/* Multiply by MILLI here to avoid losing precision */
		val = mult_frac(val, denominator * MILLI, numerator);
		/* Would multiply by NANO here but we already multiplied by MILLI */
		tmp2 = shift_right((u64)val * MICRO, val2);
		tmp0 = (int)div_s64_rem(tmp2, NANO, &tmp1);
		st->scale_tbl[i][0] = tmp0; /* Integer part */
		st->scale_tbl[i][1] = abs(tmp1); /* Fractional part */
	}
}

static int ad7768_set_dig_fil(struct ad7768_state *st,
			      enum ad7768_flt_mode filter_mode)
{
	unsigned int mode;
	int ret;

	mutex_lock(&st->lock);
	st->filter_mode = filter_mode;
	if (st->filter_mode == SINC5_DEC_X8)
		st->bits_per_word = 16;
	else
		st->bits_per_word = 24;

	ret = ad7768_spi_reg_read(st, AD7768_REG_DIGITAL_FILTER, &mode, 1);
	if (ret)
		goto out;
	mode &= ~AD7768_DIG_FIL_FIL_MSK;
	mode |= AD7768_DIG_FIL_FIL(filter_mode);

	ret = ad7768_spi_reg_write(st, AD7768_REG_DIGITAL_FILTER, mode);

	/* Update scale table: scale values vary according to the precision */
	ad7768_fill_scale_tbl(st);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad7768_set_dig_fil_attr(struct iio_dev *dev,
			      const struct iio_chan_spec *chan,
			      unsigned int filter)
{
	struct ad7768_state *st = iio_priv(dev);
	int ret;

	ret = ad7768_set_dig_fil(st, filter);
	if (ret)
		return ret;

	return ad7768_set_freq(st, st->samp_freq);
}

static int ad7768_get_dig_fil_attr(struct iio_dev *dev,
			      const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = iio_priv(dev);
	int ret;
	unsigned int mode;

	ret = ad7768_spi_reg_read(st, AD7768_REG_DIGITAL_FILTER, &mode, 1);
	if (ret)
		return ret;

	return FIELD_GET(AD7768_DIG_FIL_FIL_MSK, mode);
}

static int ad7768_calc_pga_gain(struct ad7768_state *st, int gain_int,
				int gain_fract,
				int precision)
{
	u64 gain_nano, tmp;
	int gain_idx;

	precision--;
	gain_nano = gain_int * NANO + gain_fract;
	if (gain_nano < 0 || gain_nano > ADAQ776X_GAIN_MAX_NANO)
		return -EINVAL;

	tmp = DIV_ROUND_CLOSEST_ULL(gain_nano << precision, NANO);
	gain_nano = DIV_ROUND_CLOSEST_ULL(st->vref, tmp);
	if (st->chip->has_variable_aaf)
		/* remove the AAF gain from the gain */
		gain_nano = DIV_ROUND_CLOSEST_ULL(gain_nano *  MILLI,
						  ad7768_aaf_gains[st->aaf_gain]);
	tmp = st->chip->num_pga_modes;
	gain_idx = find_closest(gain_nano, st->chip->pga_gains, tmp);

	return gain_idx;
}

static int ad7768_set_pga_gain(struct ad7768_state *st,
			       int gain_mode)
{
	int ret;
	int check_val;
	int pgia_pins_value = abs(gain_mode - st->chip->pgia_mode2pin_offset);

	mutex_lock(&st->lock);

	/* Check GPIO control register */
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &check_val, 1);
	if (ret < 0)
		goto out;
	if ((check_val & AD7768_GPIO_PGIA_EN) != AD7768_GPIO_PGIA_EN) {
		/* Enable PGIA GPIOs and set them as output */
		ret = ad7768_spi_reg_write(st, AD7768_REG_GPIO_CONTROL, AD7768_GPIO_PGIA_EN);
		if (ret < 0)
			goto out;
	}

	/* Write GPIOs 0-2 with the gain mode equivalente value */
	ret = ad7768_spi_reg_write(st, AD7768_REG_GPIO_WRITE,
				   AD7768_GPIO_WRITE(pgia_pins_value));
	if (ret < 0)
		goto out;

	st->pga_gain_mode = gain_mode;
out:
	mutex_unlock(&st->lock);
	return ret;
}

int ad7768_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_write_masked(st,
					  AD7768_REG_GPIO_CONTROL,
					  BIT(offset),
					  AD7768_GPIO_INPUT(offset));
	mutex_unlock(&st->lock);

	return ret;
}

int ad7768_gpio_direction_output(struct gpio_chip *chip,
				 unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_write_masked(st,
					  AD7768_REG_GPIO_CONTROL,
					  BIT(offset),
					  AD7768_GPIO_OUTPUT(offset));
	mutex_unlock(&st->lock);

	return ret;
}

int ad7768_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &val, 1);
	if (ret < 0)
		goto gpio_get_err;

	if (val & BIT(offset))
		ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_WRITE, &val, 1);
	else
		ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_READ, &val, 1);
	if (ret < 0)
		goto gpio_get_err;

	ret = !!(val & BIT(offset));

gpio_get_err:
	mutex_unlock(&st->lock);

	return ret;
}

void ad7768_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &val, 1);
	if (ret < 0)
		goto gpio_set_err;

	if (val & BIT(offset))
		ad7768_spi_reg_write_masked(st,
					    AD7768_REG_GPIO_WRITE,
					    BIT(offset),
					    (value << offset));

gpio_set_err:
	mutex_unlock(&st->lock);
}

int ad7768_gpio_request(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);

	if (!(st->gpio_avail_map & BIT(offset)))
		return -ENODEV;

	st->gpio_avail_map &= ~BIT(offset);

	return 0;
}

int ad7768_gpio_init(struct ad7768_state *st)
{
	int ret;

	ret = ad7768_spi_reg_write(st,
				   AD7768_REG_GPIO_CONTROL,
				   AD7768_GPIO_UNIVERSAL_EN);
	if (ret < 0)
		return ret;

	st->gpio_avail_map = AD7768_GPIO_CONTROL_MSK;
	st->gpiochip.label = "ad7768_1_gpios";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 4;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad7768_gpio_direction_input;
	st->gpiochip.direction_output = ad7768_gpio_direction_output;
	st->gpiochip.get = ad7768_gpio_get;
	st->gpiochip.set = ad7768_gpio_set;
	st->gpiochip.request = ad7768_gpio_request;
	st->gpiochip.owner = THIS_MODULE;

	return gpiochip_add_data(&st->gpiochip, st);
}

static int ad7768_set_freq(struct ad7768_state *st,
			   unsigned int freq)
{
	unsigned int diff_new, diff_old, i, idx;
	int res, dec_rate, mclk_div, temp, ret = 0;

	diff_old = U32_MAX;
	idx = 0;

	if (freq == 0)
		return -EINVAL;

	res = DIV_ROUND_CLOSEST(st->mclk_freq, freq);

	if (st->filter_mode == SINC3) {
		/* Find the closest match for the desired sampling frequency */
		for (i = 0; i < ARRAY_SIZE(ad7768_mclk_div_rates); i++) {
			/* calculate decimation theorical decimation */
			temp = res / ad7768_mclk_div_rates[i];
			/* Maximum decimation rate depends on the mclk divider */
			temp = clamp_t(int, temp, 32, sinc3_dec_rate_max_values[i]);
			diff_new = abs(res - (temp * ad7768_mclk_div_rates[i]));
			if (diff_new < diff_old) {
				diff_old = diff_new;
				idx = i;
				dec_rate = temp;
			}
		}
		mclk_div = idx;
		/* Set decimation rate for sinc3 filter mode */
		ret = ad7768_set_sinc3_dec_rate(st, (dec_rate / 32) - 1);
	} else if ((st->filter_mode == SINC5_DEC_X16)
		   || (st->filter_mode == SINC5_DEC_X8)) {
		dec_rate = 8 * st->filter_mode;
		/* Find the closest mclk_div for the desired sampling frequency */
		mclk_div = find_closest_descending(res / dec_rate, ad7768_mclk_div_rates,
						   ARRAY_SIZE(ad7768_mclk_div_rates));
	} else {
		/* Find the closest match for the desired sampling frequency */
		for (i = 0; i < ARRAY_SIZE(ad7768_sinc5_wideband_clk_conf); i++) {
			diff_new = abs(res - ad7768_sinc5_wideband_clk_conf[i].clk_div);
			if (diff_new < diff_old) {
				diff_old = diff_new;
				idx = i;
			}
		}
		mclk_div = ad7768_sinc5_wideband_clk_conf[idx].mclk_div;
		/* Set decimation rate for sinc5 and wideband filter modes */
		ret = ad7768_set_dec_rate(st, ad7768_sinc5_wideband_clk_conf[idx].dec_rate);
		dec_rate = dec_rate_values[ad7768_sinc5_wideband_clk_conf[idx].dec_rate];
	}
	if (ret)
		return ret;
	/* Set mclk_div keeping the value of pwrmode */
	ret = ad7768_set_mclk_div(st, mclk_div);
	if (ret)
		return ret;

	st->samp_freq = DIV_ROUND_CLOSEST(st->mclk_freq,
					  ad7768_mclk_div_rates[mclk_div] * dec_rate);

	/* A sync-in pulse is required every time the filter dec rate changes */
	return ad7768_send_sync_pulse(st);
}

static int ad7768_get_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = iio_priv(dev);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);

	ret = ad7768_spi_reg_read(st, AD7768_REG_POWER_CLOCK, &val, 1);
	if (ret)
		goto out;

	ret = FIELD_GET(AD7768_PWR_PWRMODE_MSK, val);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad7768_set_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan, unsigned int pwr_mode)
{
	struct ad7768_state *st = iio_priv(dev);
	unsigned int mode;
	int ret;

	mutex_lock(&st->lock);

	st->pwrmode = pwr_mode;
	ret = ad7768_spi_reg_read(st, AD7768_REG_POWER_CLOCK, &mode, 1);
	if (ret)
		goto out;

	mode &= ~AD7768_PWR_PWRMODE_MSK;
	mode |= AD7768_PWR_PWRMODE(pwr_mode);
	ret = ad7768_spi_reg_write(st, AD7768_REG_POWER_CLOCK, mode);
	if (ret < 0)
		goto out;

	ret = ad7768_send_sync_pulse(st);
out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad7768_get_vcm(struct iio_dev *dev, const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = iio_priv(dev);

	return st->common_mode_voltage;
}

static int ad7768_set_vcm(struct iio_dev *dev,
			  const struct iio_chan_spec *chan,
			  unsigned int mode)
{
	int ret;
	struct ad7768_state *st = iio_priv(dev);

	ret = ad7768_spi_reg_write(st, AD7768_REG_ANALOG2, mode);

	if (ret == 0)
		st->common_mode_voltage = mode;

	return ret;
}

static ssize_t sampling_frequency_available_show(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int freq;
	int i, len = 0;

	/* Return Frequency available in range format */
	buf[len++] = '[';
	for (i = 0; i < ARRAY_SIZE(ad7768_freq_available_range[st->filter_mode]); i++) {
		freq = ad7768_freq_available_range[st->filter_mode][i];
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ", freq);
	}

	buf[len - 1] = ']';
	buf[len++] = '\n';

	return len;
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	int ret;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad7768_scan_direct(indio_dev);
		if (ret >= 0)
			*val = sign_extend32(ret, scan_type->realbits - 1);

		iio_device_release_direct_mode(indio_dev);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		if (st->chip->has_pga) {
			mutex_lock(&st->lock);
			*val = st->scale_tbl[st->pga_gain_mode][0];
			*val2 = st->scale_tbl[st->pga_gain_mode][1];
			mutex_unlock(&st->lock);
			return IIO_VAL_INT_PLUS_NANO;
		}
		*val = st->vref / 1000;
		if (st->chip->has_variable_aaf)
			*val = (*val * MILLI) / ad7768_aaf_gains[st->aaf_gain];
		*val2 = scan_type->realbits - 1;

		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->samp_freq;

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad7768_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (int *)st->scale_tbl;
		*length = st->chip->num_pga_modes * 2;
		*type = IIO_VAL_INT_PLUS_NANO;
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad7768_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int gain_mode;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7768_set_freq(st, val);
	case IIO_CHAN_INFO_SCALE:
		if (!st->chip->has_pga)
			return -EPERM;
		gain_mode = ad7768_calc_pga_gain(st, val, val2,
						 st->bits_per_word);
		return ad7768_set_pga_gain(st, gain_mode);
	default:
		return -EINVAL;
	}
}

static int ad7768_read_label(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, char *label)
{
	struct ad7768_state *st = iio_priv(indio_dev);

	return sprintf(label, "%s\n", st->labels[chan->channel]);
}

static struct attribute *ad7768_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad7768_group = {
	.attrs = ad7768_attributes,
};

static int ad7768_get_current_scan_type(const struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = iio_priv(indio_dev);

	return st->filter_mode == SINC5_DEC_X8 ? AD7768_SCAN_TYPE_HIGH_SPEED
					    : AD7768_SCAN_TYPE_NORMAL;
}

static const struct iio_info ad7768_info = {
	.attrs = &ad7768_group,
	.read_raw = &ad7768_read_raw,
	.read_avail = &ad7768_read_avail,
	.write_raw = &ad7768_write_raw,
	.write_raw_get_fmt = &ad7768_write_raw_get_fmt,
	.read_label = ad7768_read_label,
	.get_current_scan_type = &ad7768_get_current_scan_type,
	.debugfs_reg_access = &ad7768_reg_access,
};

static void ad7768_fill_freq_available_range(struct ad7768_state *st)
{
	int i;

	for (i = 0; i <= WIDEBAND; i++) {
		ad7768_freq_available_range[i][0] = DIV_ROUND_CLOSEST(st->mclk_freq,
						    ad7768_clk_div_ranges[i].clk_div_max);
		ad7768_freq_available_range[i][1] = DIV_ROUND_CLOSEST(st->mclk_freq,
						    ad7768_clk_div_ranges[i].clk_div_max);
		ad7768_freq_available_range[i][2] = DIV_ROUND_CLOSEST(st->mclk_freq,
						    ad7768_clk_div_ranges[i].clk_div_min);
	}
}

static int ad7768_setup(struct ad7768_state *st)
{
	int ret;

	st->gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
						 GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	if (st->gpio_reset) {
		gpiod_direction_output(st->gpio_reset, 1);
		usleep_range(10, 15);
		gpiod_direction_output(st->gpio_reset, 0);
		usleep_range(10, 15);
	}

	/*
	 * Two writes to the SPI_RESET[1:0] bits are required to initiate
	 * a software reset. The bits must first be set to 11, and then
	 * to 10. When the sequence is detected, the reset occurs.
	 * See the datasheet, page 70.
	 */
	ret = ad7768_spi_reg_write(st, AD7768_REG_SYNC_RESET, 0x3);
	if (ret)
		return ret;

	ret = ad7768_spi_reg_write(st, AD7768_REG_SYNC_RESET, 0x2);
	if (ret)
		return ret;

	if (device_property_present(&st->spi->dev, "adi,sync-in-gpios")) {
		st->en_gpio_sync = true;
		st->gpio_sync_in = devm_gpiod_get(&st->spi->dev, "adi,sync-in",
					  GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_sync_in))
			return PTR_ERR(st->gpio_sync_in);

	}

	if (device_property_present(&st->spi->dev, "adi,sync-in-spi"))
		st->en_spi_sync = true;

	ret = ad7768_gpio_init(st);
	if (ret < 0)
		return ret;

	/* fill the frequency available range for every filter mode */
	ad7768_fill_freq_available_range(st);

	/* Set Default Filter mode */
	ret = ad7768_set_dig_fil(st, SINC5);
	if (ret < 0)
		return ret;

	/**
	 * Set the default sampling frequency to 256 kSPS for hardware buffer,
	 * or 32 kSPS for triggered buffer
	 */
	return ad7768_set_freq(st, st->spi_is_dma_mapped ? 256000 : 32000);
}

static irqreturn_t ad7768_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	ret = spi_read(st->spi, &st->data.scan.chan, 3);
	if (ret < 0)
		goto err_unlock;

	iio_push_to_buffers_with_timestamp(indio_dev, &st->data.scan,
					   iio_get_time_ns(indio_dev));

err_unlock:
	iio_trigger_notify_done(indio_dev->trig);
	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static irqreturn_t ad7768_interrupt(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct ad7768_state *st = iio_priv(indio_dev);

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->completion);

	return IRQ_HANDLED;
};

static int ad7768_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	struct spi_transfer xfer = {
		.len = 1,
	};
	unsigned int rx_data[2];
	int ret;

	scan_type = iio_get_current_scan_type(indio_dev, &indio_dev->channels[0]);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	st->offload_xfer.len = roundup_pow_of_two(BITS_TO_BYTES(scan_type->realbits));
	st->offload_xfer.bits_per_word = scan_type->realbits;

	/*
	* Write a 1 to the LSB of the INTERFACE_FORMAT register to enter
	* continuous read mode. Subsequent data reads do not require an
	* initial 8-bit write to query the ADC_DATA register.
	*/
	ret =  ad7768_spi_reg_write(st, AD7768_REG_INTERFACE_FORMAT, 0x01);
	if (ret)
		return ret;

	if (st->spi_is_dma_mapped) {
		st->offload_xfer.rx_buf = rx_data;
		spi_message_init_with_transfers(&st->offload_msg, &st->offload_xfer, 1);

		ret = spi_optimize_message(st->spi, &st->offload_msg);
		if (ret < 0)
			return ret;

		spi_bus_lock(st->spi->master);

		ret = spi_engine_ex_offload_load_msg(st->spi, &st->offload_msg);
		if (ret < 0)
			return ret;
		spi_engine_ex_offload_enable(st->spi, true);
	}

	return ret;
}

static int ad7768_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int regval;

	if (st->spi_is_dma_mapped) {
		spi_engine_ex_offload_enable(st->spi, false);
		spi_bus_unlock(st->spi->master);
	}
	spi_unoptimize_message(&st->offload_msg);

	/*
	 * To exit continuous read mode, perform a single read of the ADC_DATA
	 * reg (0x2C), which allows further configuration of the device.
	 */
	return ad7768_spi_reg_read(st, AD7768_REG_ADC_DATA, &regval, 3);
}

static const struct iio_buffer_setup_ops ad7768_buffer_ops = {
	.postenable = &ad7768_buffer_postenable,
	.predisable = &ad7768_buffer_predisable,
};

static const struct iio_trigger_ops ad7768_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static void ad7768_regulator_disable(void *data)
{
	struct ad7768_state *st = data;

	regulator_disable(st->regulator);
}

static int ad7768_triggered_buffer_alloc(struct iio_dev *indio_dev)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;

	st->trig = devm_iio_trigger_alloc(indio_dev->dev.parent, "%s-dev%d",
					  indio_dev->name, iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ad7768_trigger_ops;
	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(indio_dev->dev.parent, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	init_completion(&st->completion);

	ret = devm_request_irq(indio_dev->dev.parent, st->irq,
			       &ad7768_interrupt,
			       IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			       indio_dev->name, indio_dev);
	if (ret)
		return ret;

	return devm_iio_triggered_buffer_setup(indio_dev->dev.parent, indio_dev,
					       &iio_pollfunc_store_time,
					       &ad7768_trigger_handler,
					       &ad7768_buffer_ops);
}

static int ad7768_hardware_buffer_alloc(struct iio_dev *indio_dev)
{
	indio_dev->setup_ops = &ad7768_buffer_ops;
	return devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
					       indio_dev, "rx",
					       IIO_BUFFER_DIRECTION_IN);
}

static int ad7768_set_channel_label(struct iio_dev *indio_dev,
						int num_channels)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	struct device *device = indio_dev->dev.parent;
	struct fwnode_handle *fwnode;
	struct fwnode_handle *child;
	const char *label;
	int crt_ch = 0;

	fwnode = dev_fwnode(device);
	fwnode_for_each_child_node(fwnode, child) {
		if (fwnode_property_read_u32(child, "reg", &crt_ch))
			continue;

		if (crt_ch >= num_channels)
			continue;

		if (fwnode_property_read_string(child, "label", &label))
			continue;

		st->labels[crt_ch] = label;
	}

	return 0;
}

static const unsigned long ad7768_channel_masks[] = {
	BIT(0),
	0,
};

static const struct ad7768_chip_info ad7768_chip_info = {
	.name = "ad7768-1",
	.channel_spec = ad7768_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
};

static const struct ad7768_chip_info adaq7767_chip_info = {
	.name = "adaq7767-1",
	.channel_spec = ad7768_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
	.has_pga = false,
	.has_variable_aaf = true
};

static const struct ad7768_chip_info adaq7768_chip_info = {
	.name = "adaq7768-1",
	.channel_spec = adaq776x_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
	.pga_gains = adaq7768_gains,
	.default_pga_mode = AD7768_PGA_GAIN_2,
	.num_pga_modes = ARRAY_SIZE(adaq7768_gains),
	.pgia_mode2pin_offset = 6,
	.has_pga = true,
	.has_variable_aaf = false
};

static const struct ad7768_chip_info adaq7769_chip_info = {
	.name = "adaq7769-1",
	.channel_spec = adaq776x_channels,
	.num_channels = 1,
	.available_masks = ad7768_channel_masks,
	.pga_gains = adaq7769_gains,
	.default_pga_mode = AD7768_PGA_GAIN_0,
	.num_pga_modes = ARRAY_SIZE(adaq7769_gains),
	.pgia_mode2pin_offset = 0,
	.has_pga = true,
	.has_variable_aaf = true
};

static int ad7768_probe(struct spi_device *spi)
{
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	u32 val;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	/*
	 * The ADC SDI line must be kept high when
	 * data is not being clocked out of the controller.
	 * Request the SPI controller to make MOSI idle high.
	 */
	spi->mode |= SPI_MOSI_IDLE_HIGH;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;
	st->spi = spi;

	st->regulator = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->regulator))
		return PTR_ERR(st->regulator);

	ret = regulator_enable(st->regulator);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable specified vref supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad7768_regulator_disable, st);
	if (ret)
		return ret;

	st->mclk = devm_clk_get_enabled(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);

	st->mclk_freq = clk_get_rate(st->mclk);
	st->spi_is_dma_mapped = spi_engine_ex_offload_supported(spi);
	st->irq = spi->irq;
	st->vref = regulator_get_voltage(st->regulator);

	st->chip = spi_get_device_match_data(spi);
	if (!st->chip)
		return dev_err_probe(&spi->dev, -ENODEV,
				     "Could not find chip info data\n");

	mutex_init(&st->lock);

	indio_dev->channels = st->chip->channel_spec;
	indio_dev->num_channels = st->chip->num_channels;
	indio_dev->name = st->chip->name;
	indio_dev->info = &ad7768_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	st->aaf_gain = AD7768_AAF_IN1;
	ret = device_property_read_u32(&spi->dev, "adi,aaf-gain", &val);
	if (!ret) {
		if (!st->chip->has_variable_aaf)
			return dev_err_probe(&spi->dev, -EINVAL,
					     "This device does not support AAF");

		switch (val) {
		case 1000:
			st->aaf_gain = AD7768_AAF_IN1;
			break;
		case 364:
			st->aaf_gain = AD7768_AAF_IN2;
			break;
		case 143:
			st->aaf_gain = AD7768_AAF_IN3;
			break;
		default:
			return dev_err_probe(&spi->dev, -EINVAL,
					     "Invalid firmware provided gain\n");
		}
	}

	ret = ad7768_setup(st);
	if (ret < 0) {
		dev_err(&spi->dev, "AD7768 setup failed\n");
		return ret;
	}

	if (st->chip->has_pga)
		ad7768_set_pga_gain(st, st->chip->default_pga_mode);

	ret = ad7768_set_channel_label(indio_dev, st->chip->num_channels);
	if (ret)
		return ret;

	if (st->spi_is_dma_mapped)
		ret = ad7768_hardware_buffer_alloc(indio_dev);
	else
		ret = ad7768_triggered_buffer_alloc(indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad7768_id_table[] = {
	{ "ad7768-1", (kernel_ulong_t)&ad7768_chip_info },
	{ "adaq7767-1", (kernel_ulong_t)&adaq7767_chip_info },
	{ "adaq7768-1", (kernel_ulong_t)&adaq7768_chip_info },
	{ "adaq7769-1", (kernel_ulong_t)&adaq7769_chip_info },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7768_id_table);

static const struct of_device_id ad7768_of_match[] = {
	{ .compatible = "adi,ad7768-1", .data = &ad7768_chip_info },
	{ .compatible = "adi,adaq7767-1", .data = &adaq7767_chip_info },
	{ .compatible = "adi,adaq7768-1", .data = &adaq7768_chip_info },
	{ .compatible = "adi,adaq7769-1", .data = &adaq7769_chip_info },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7768_of_match);

static struct spi_driver ad7768_driver = {
	.driver = {
		.name = "ad7768-1",
		.of_match_table = ad7768_of_match,
	},
	.probe = ad7768_probe,
	.id_table = ad7768_id_table,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768-1 ADC driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
