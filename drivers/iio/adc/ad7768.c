// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD7768 ADC driver
 *
 * Copyright 2018-2025 Analog Devices Inc.
 */

#include "linux/iio/types.h"
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/clk.h>
#include <linux/gpio/driver.h>
#include <linux/pm_runtime.h>

/* AD7768 registers definition */
#define AD7768_CH_STANDBY			0x00
#define AD7768_CH_MODE(x)			(0x01 + (x))
#define AD7768_CH_MODE_SEL			0x03
#define AD7768_POWER_MODE			0x04
#define AD7768_GENERAL_CONFIG			0x05
#define AD7768_DATA_CONTROL			0x06
#define AD7768_INTERFACE_CFG			0x07
#define AD7768_REV_ID				0x0A
#define AD7768_GPIO_CTRL			0x0E
#define AD7768_PRECHARGE_BUF1			0x11
#define AD7768_PRECHARGE_BUF2			0x12
#define AD7768_REFP_BUF				0x13
#define AD7768_REFN_BUF				0x14
#define AD7768_REG_GPIO_CONTROL			0x0E
#define AD7768_REG_OFFSET(ch)			((0x1E + (3* ch)))
#define AD7768_REG_GAIN(ch)			((0x36 + (3 * ch)))
#define AD7768_REG_PHASE(ch)			(0x4E + (ch))
#define AD7768_4_REG_OFFSET(ch)			((0x1E + (3 * ((ch < 2) ? ch : (ch + 2)))))
#define AD7768_4_REG_GAIN(ch)			((0x36 + (3 * ((ch < 2) ? ch : (ch + 2)))))
#define AD7768_4_REG_PHASE(ch)			(0x4E + ((ch < 2) ? ch : (ch + 2)))
#define AD7768_REG_DIAGNOSTIC_RX		0x56
#define AD7768_REG_DIAGNOSTIC_MUX_CTRL		0x57
#define AD7768_MODULATOR_DELAY_CTRL		0x58
#define AD7768_REG_CHOP_CTRL			0x59


/* AD7768_REG_GPIO_CONTROL */
#define AD7768_GPIO_UGPIO_ENABLE		BIT(7)

#define AD7768_GPIO_INPUT(x)			0x00
#define AD7768_GPIO_OUTPUT(x)			BIT(x)

#define AD7768_REG_GPIO_WRITE			0x0F
#define AD7768_REG_GPIO_READ			0x10

/* AD7768_CH_MODE */
#define AD7768_CH_MODE_FILTER_TYPE_MSK		BIT(3)
#define AD7768_CH_MODE_FILTER_TYPE_MODE(x)	(((x) & 0x1) << 3)
#define AD7768_CH_MODE_GET_FILTER_TYPE(x)	(((x) >> 3) & 0x1)
#define AD7768_CH_MODE_DEC_RATE_MSK		GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE_MODE(x)		(((x) & 0x7) << 0)

/* AD7768_CH_MODE_SEL - Channel mode read configuration */
#define AD7768_CH_MODE_READ_MSK			GENMASK(7, 0)
#define AD7768_CH_MODE_READ_MODE(x)		((x) & 0xFF)
#define AD7768_CH_MODE_READ_GET(x)		((x) & 0xFF)

/* Channel mode read bit positions */
#define AD7768_CH_MODE_READ_CHn(n)		BIT(n)
#define AD7768_GET_CH_MODE_8CH(ch, x)		!!((x) & BIT(ch))
#define AD7768_GET_CH_MODE_4CH(ch, x)		!!((x) & BIT((ch < 2) ? ch : (ch + 2)))

#define AD7768_CH_MODE_SEL_8CH(ch)		BIT(ch)
#define AD7768_CH_MODE_SEL_4CH(ch)		(ch < 2) ? BIT(ch) : BIT(ch + 2)

/* AD7768_POWER_MODE */
#define AD7768_SLEEP_MODE_MSK			BIT(7)
#define AD7768_POWER_MODE_POWER_MODE_MSK	GENMASK(5, 4)
#define AD7768_POWER_MODE_POWER_MODE(x)		(((x) & 0x3) << 4)
#define AD7768_POWER_MODE_GET_POWER_MODE(x)	(((x) >> 4) & 0x3)
#define AD7768_POWER_MODE_MCLK_DIV_MSK		GENMASK(1, 0)
#define AD7768_POWER_MODE_MCLK_DIV_MODE(x)	(((x) & 0x3) << 0)
#define ad7768_map_power_mode_to_regval(x)	((x) ? ((x) + 1) : 0)
#define ad7768_map_regval_to_power_mode(x)	((x) ? ((x) - 1) : 0)

/* AD7768_DATA_CONTROL */
#define AD7768_DATA_CONTROL_SPI_RESET_MSK	GENMASK(1, 0)
#define AD7768_DATA_CONTROL_SPI_RESET_1		0x03
#define AD7768_DATA_CONTROL_SPI_RESET_2		0x02
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK	BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC		BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR	0

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK	GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)	(4 - ffs(x))
#define AD7768_MAX_DCLK_DIV			8

#define AD7768_INTERFACE_CFG_CRC_SELECT_MSK	GENMASK(3, 2)
/* only 4 samples CRC calculation support exists */
#define AD7768_INTERFACE_CFG_CRC_SELECT		0x01

/* AD7768_GENERAL_CONFIG */
#define AD7768_GEN_CONFIG_VCM_SEL_MSK		GENMASK(1, 0)
#define AD7768_GEN_CONFIG_VCM_PD		BIT(4)

/* AD7768_PRECHARGE_BUF1 and 2*/
#define AD7768_PRECHARGE_BUF1_MSK(val)		(val & GENMASK(7, 0))
#define AD7768_PRECHARGE_BUF2_MSK(val)		((val & GENMASK(15, 8)) >> 8)
#define AD7768_4_PRECHARGE_BUF1_MSK(val)	(val & GENMASK(3, 0))
#define AD7768_4_PRECHARGE_BUF2_MSK(val)	((val & GENMASK(7, 4)) >> 4)
#define AD7768_4_PRECHARGE_REFBUF1_MSK(val)	(val & GENMASK(1, 0)) | ((val & GENMASK(5, 4)) >> 4)
#define AD7768_4_PRECHARGE_REFBUF2_MSK(val)	((val & GENMASK(3, 2)) >> 2) | ((val & GENMASK(7, 6)) >> 6)
 #define AD7768_4_GET_REFBUF(ch)		(ch < 2) ? BIT(ch) : BIT(ch + 2)

#define AD7768_PREBUF_POS_EN(ch)		BIT(ch * 2)
#define AD7768_PREBUF_NEG_EN(ch)		BIT((ch * 2) + 1)

#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

#define AD7768_OUTPUT_MODE_TWOS_COMPLEMENT	0x01

#define SAMPLE_SIZE				32
#define MAX_FREQ_PER_MODE			6

#define AD7768_MAX_CHANNEL  8
#define AD7768_CALIB_REG_MSB_MASK(val)		((val & 0xFF0000) >> 16)
#define AD7768_CALIB_REG_MID_MASK(val)		((val & 0x00FF00) >> 8)
#define AD7768_CALIB_REG_LSB_MASK(val)		(val & 0x0000FF)

enum ad7768_filter_type {
	AD7768_FILTER_TYPE_WIDEBAND,
	AD7768_FILTER_TYPE_SINC5,
};

enum ad7768_power_modes {
	AD7768_LOW_POWER_MODE,
	AD7768_MEDIAN_MODE,
	AD7768_FAST_MODE,
	AD7768_NUM_POWER_MODES
};

struct ad7768_precharge_config {
	bool prebufp_en;
	bool prebufn_en;
	bool refbufp;
	bool refbufn;
};

struct ad7768_freq_config {
	unsigned int freq;
	unsigned int dec_rate;
};

struct ad7768_avail_freq {
	unsigned int n_freqs;
	struct ad7768_freq_config freq_cfg[MAX_FREQ_PER_MODE];
};

struct ad7768_chip_info {
	unsigned int			id;
	const char *name;
	unsigned			num_channels;
	const struct iio_chan_spec channel[AD7768_MAX_CHANNEL];
};

struct ad7768_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct mutex lock;
	struct clk *mclk;
	struct gpio_chip gpiochip;
	u64 vref_nv;
	unsigned int datalines;
	unsigned int sampling_freq;
	enum ad7768_power_modes power_mode;
	const struct ad7768_chip_info *chip_info;
	struct ad7768_avail_freq avail_freq[AD7768_NUM_POWER_MODES];
	unsigned int chn_mode[AD7768_MAX_CHANNEL];
	struct ad7768_precharge_config precharge_cfg[AD7768_MAX_CHANNEL];
	unsigned int channel_freq[AD7768_MAX_CHANNEL];
	struct iio_backend *back;
	__be16 d16;
	unsigned int num_en_channels;
};

enum ad7768_device_ids {
	ID_AD7768,
	ID_AD7768_4
};

static const int ad7768_dec_rate[6] = {
	32, 64, 128, 256, 512, 1024
};

static const int ad7768_mclk_div[3] = {
	32, 8, 4
};

static const unsigned int ad7768_available_datalines[] = {
	1, 2, 8
};

static const unsigned int ad7768_4_available_datalines[] = {
	1, 4
};

static int ad7768_regmap_read(void *context, const void *reg_buf, size_t reg_size,
			       void *val_buf, size_t val_size)
{
	struct spi_device *spi = context;
	struct ad7768_state *st = spi_get_drvdata(spi);
	unsigned int reg;
	int ret;
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d16,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = &st->d16,
			.len = 2,
		},
	};

	if (reg_size != 1 || val_size != 1)
		return -EINVAL;

	reg = *(const u8 *)reg_buf;

	st->d16 = cpu_to_be16(((reg & 0x7F) | 0x80) << 8);

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	*(u8 *)val_buf = be16_to_cpu(st->d16) & 0xFF;

	return ret;
}

static int ad7768_regmap_write(void *context, const void *data, size_t count)
{
	struct spi_device *spi = context;
	struct ad7768_state *st = spi_get_drvdata(spi);
	const u8 *buf = data;
	unsigned int reg, val;

	if (count != 2)
		return -EINVAL;

	reg = buf[0];
	val = buf[1];

	st->d16 = cpu_to_be16(((reg & 0x7F) << 8) | (val & 0xFF));

	return spi_write(spi, &st->d16, 2);
}

static const struct regmap_bus ad7768_regmap_bus = {
	.read = ad7768_regmap_read,
	.write = ad7768_regmap_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static bool ad7768_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD7768_CH_STANDBY ... AD7768_REV_ID:
	case AD7768_GPIO_CTRL ... AD7768_REFN_BUF:
		return true;
	case AD7768_REG_OFFSET(0) ... AD7768_REG_OFFSET(7) + 2:
	case AD7768_REG_GAIN(0) ... AD7768_REG_GAIN(7) + 2:
	case AD7768_REG_PHASE(0) ... AD7768_REG_PHASE(7):
	case AD7768_REG_DIAGNOSTIC_RX ... AD7768_REG_CHOP_CTRL:
		return true;
	default:
		return false;
	}
}

static bool ad7768_4_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AD7768_CH_STANDBY ... AD7768_REV_ID:
	case AD7768_GPIO_CTRL ... AD7768_REFN_BUF:
		return true;
	case AD7768_4_REG_OFFSET(0) ... AD7768_4_REG_OFFSET(1) + 2:
	case AD7768_4_REG_OFFSET(2) ... AD7768_4_REG_OFFSET(3) + 2:
	case AD7768_4_REG_GAIN(0) ... AD7768_4_REG_GAIN(1) + 2:
	case AD7768_4_REG_GAIN(2) ... AD7768_4_REG_GAIN(3) + 2:
	case AD7768_4_REG_PHASE(0) ... AD7768_4_REG_PHASE(1):
	case AD7768_4_REG_PHASE(2) ... AD7768_4_REG_PHASE(3):
	case AD7768_REG_DIAGNOSTIC_RX ... AD7768_REG_CHOP_CTRL:
		return true;
	default:
		return false;
	}
}

static const struct regmap_config ad7768_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AD7768_REG_CHOP_CTRL,
	/* .readable_reg assigned at runtime based on chip variant */
};

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7768_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static int ad7768_sync(struct ad7768_state *st)
{
	int ret;

	ret = regmap_update_bits(st->regmap, AD7768_DATA_CONTROL,
				 AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				 AD7768_DATA_CONTROL_SPI_SYNC_CLEAR);
	if (ret < 0)
		return ret;

	return regmap_update_bits(st->regmap, AD7768_DATA_CONTROL,
				  AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				  AD7768_DATA_CONTROL_SPI_SYNC);
}

static int ad7768_set_clk_divs(struct ad7768_state *st,
			       unsigned int freq, unsigned int ch)
{
	unsigned int mclk, dclk, dclk_div, i;
	struct ad7768_freq_config f_cfg;
	unsigned int chan_per_doutx;
	int ret = 0;

	mclk = clk_get_rate(st->mclk);

	chan_per_doutx = st->chip_info->num_channels / st->datalines;

	for (i = 0; i < st->avail_freq[st->power_mode].n_freqs; i++) {
		f_cfg = st->avail_freq[st->power_mode].freq_cfg[i];
		if (freq == f_cfg.freq)
			break;
	}
	if (i == st->avail_freq[st->power_mode].n_freqs)
		return -EINVAL;

	dclk = f_cfg.freq * SAMPLE_SIZE * chan_per_doutx;
	if (dclk > mclk)
		return -EINVAL;

	/* Set dclk_div to the nearest power of 2 less than the original value */
	dclk_div = DIV_ROUND_CLOSEST_ULL(mclk, dclk);
	if (dclk_div > AD7768_MAX_DCLK_DIV)
		dclk_div = AD7768_MAX_DCLK_DIV;
	else if (hweight32(dclk_div) != 1)
		dclk_div = 1 << (fls(dclk_div) - 1);

	ret = regmap_update_bits(st->regmap, AD7768_INTERFACE_CFG,
				 AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
				 AD7768_INTERFACE_CFG_DCLK_DIV_MODE(dclk_div));
	if (ret < 0)
		return ret;

	return regmap_update_bits(st->regmap, AD7768_INTERFACE_CFG,
				  AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
				  AD7768_INTERFACE_CFG_DCLK_DIV_MODE(dclk_div));
}

static int ad7768_set_channel_decimation(struct ad7768_state *st,
					  unsigned int freq, unsigned int ch)
{
	struct ad7768_freq_config f_cfg;
	unsigned int i;

	for (i = 0; i < st->avail_freq[st->power_mode].n_freqs; i++) {
		f_cfg = st->avail_freq[st->power_mode].freq_cfg[i];
		if (freq == f_cfg.freq)
			break;
	}
	if (i == st->avail_freq[st->power_mode].n_freqs)
		return -EINVAL;

	return regmap_update_bits(st->regmap, AD7768_CH_MODE(st->chn_mode[ch]),
				  AD7768_CH_MODE_DEC_RATE_MSK,
				  AD7768_CH_MODE_DEC_RATE_MODE(f_cfg.dec_rate));
}

static int ad7768_set_sampling_freq(struct iio_dev *indio_dev,
				    unsigned int freq, unsigned int ch)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret = 0;
	unsigned int max_freq = 0;
	int i;

	if (!freq)
		return -EINVAL;

	mutex_lock(&st->lock);

	st->channel_freq[ch] = freq;

	for (i = 0; i < indio_dev->num_channels; i++) {
		unsigned int ch_num = indio_dev->channels[i].channel;
		if (st->channel_freq[ch_num] > max_freq) {
			max_freq = st->channel_freq[ch_num];
		}
	}

	ret = ad7768_set_clk_divs(st, max_freq, ch);
	if (ret < 0)
		goto freq_err;

	ret = ad7768_set_channel_decimation(st, freq, ch);
	if (ret < 0)
		goto freq_err;

	ret = ad7768_sync(st);
	if (ret < 0)
		goto freq_err;

freq_err:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_get_channel_mode(struct iio_dev *indio_dev, unsigned int ch, unsigned int *mode)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	ret = regmap_read(st->regmap, AD7768_CH_MODE_SEL, &regval);
	if (ret < 0)
		return ret;

	if (st->chip_info->num_channels == AD7768_MAX_CHANNEL) {
		*mode = AD7768_GET_CH_MODE_8CH(ch, regval);
	} else {
		*mode = AD7768_GET_CH_MODE_4CH(ch, regval);
	}

	return 0;
}

static int ad7768_set_channel_mode(struct iio_dev *indio_dev, unsigned int ch, unsigned int mode) {
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;
	unsigned int mask;

	if (st->chip_info->num_channels == AD7768_MAX_CHANNEL) {
		mask = AD7768_CH_MODE_SEL_8CH(ch);
	} else {
		mask = AD7768_CH_MODE_SEL_4CH(ch);
	}

	ret = regmap_update_bits(st->regmap, AD7768_CH_MODE_SEL, mask, mode ? mask : 0);
	if (ret)
		return ret;

	st->chn_mode[ch] = mode;

	return ret;
}

static int ad7768_set_power_mode(struct iio_dev *indio_dev, unsigned int mode)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	struct ad7768_avail_freq avail_freq;
	int max_mode_freq;
	unsigned int regval;
	int ret;

	st->power_mode = mode;

	regval = ad7768_map_power_mode_to_regval(mode);
	ret = regmap_update_bits(st->regmap, AD7768_POWER_MODE,
				 AD7768_POWER_MODE_POWER_MODE_MSK,
				 AD7768_POWER_MODE_POWER_MODE(regval));
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, AD7768_POWER_MODE,
				 AD7768_POWER_MODE_MCLK_DIV_MSK,
				 AD7768_POWER_MODE_MCLK_DIV_MODE(regval));
	if (ret < 0)
		return ret;

	avail_freq = st->avail_freq[mode];
	max_mode_freq = avail_freq.freq_cfg[avail_freq.n_freqs - 1].freq;

	st->sampling_freq = max_mode_freq;

	ret = ad7768_sync(st);
	if (ret < 0)
		return ret;

	return ret;
}

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int msb, mid, lsb, base_reg;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->channel_freq[chan->channel];
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBBIAS:
		if (st->chip_info->num_channels == AD7768_MAX_CHANNEL)
			base_reg = AD7768_REG_OFFSET(chan->address);
		else
			base_reg = AD7768_4_REG_OFFSET(chan->address);

		ret = regmap_read(st->regmap, base_reg + 0, &msb);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, base_reg + 1, &mid);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, base_reg + 2, &lsb);
		if (ret)
			return ret;

		*val = (msb << 16) | (mid << 8) | lsb;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_CALIBSCALE:
		if (st->chip_info->num_channels == AD7768_MAX_CHANNEL)
			base_reg = AD7768_REG_GAIN(chan->address);
		else
			base_reg = AD7768_4_REG_GAIN(chan->address);

		ret = regmap_read(st->regmap, base_reg + 0, &msb);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, base_reg + 1, &mid);
		if (ret)
			return ret;

		ret = regmap_read(st->regmap, base_reg + 2, &lsb);
		if (ret)
			return ret;

		*val = (msb << 16) | (mid << 8) | lsb;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_PHASE:
		if (st->chip_info->num_channels == AD7768_MAX_CHANNEL)
			base_reg = AD7768_REG_PHASE(chan->address);
		else
			base_reg = AD7768_4_REG_PHASE(chan->address);

		ret = regmap_read(st->regmap, base_reg, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;
	unsigned int base_reg;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7768_set_sampling_freq(indio_dev, val, chan->address);

	case IIO_CHAN_INFO_CALIBBIAS:
		if (st->chip_info->num_channels == AD7768_MAX_CHANNEL)
			base_reg = AD7768_REG_OFFSET(chan->address);
		else
			base_reg = AD7768_4_REG_OFFSET(chan->address);

		ret = regmap_write(st->regmap, base_reg, AD7768_CALIB_REG_MSB_MASK(val));
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, base_reg + 1, AD7768_CALIB_REG_MID_MASK(val));
		if (ret)
			return ret;

		return regmap_write(st->regmap, base_reg + 2, AD7768_CALIB_REG_LSB_MASK(val));

	case IIO_CHAN_INFO_CALIBSCALE:
		if (st->chip_info->num_channels == AD7768_MAX_CHANNEL)
			base_reg = AD7768_REG_GAIN(chan->address);
		else
			base_reg = AD7768_4_REG_GAIN(chan->address);

		ret = regmap_write(st->regmap, base_reg, AD7768_CALIB_REG_MSB_MASK(val));
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, base_reg + 1, AD7768_CALIB_REG_MID_MASK(val));
		if (ret)
			return ret;

		return regmap_write(st->regmap, base_reg + 2, AD7768_CALIB_REG_LSB_MASK(val));

	case IIO_CHAN_INFO_PHASE:
		if (st->chip_info->num_channels == AD7768_MAX_CHANNEL)
			base_reg = AD7768_REG_PHASE(chan->address);
		else
			base_reg = AD7768_4_REG_PHASE(chan->address);

		return regmap_write(st->regmap, base_reg, val);

	default:
		return -EINVAL;
	}
}

static int ad7768_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int c;
	int ret;

	for (c = 0; c < st->chip_info->num_channels; c++) {
		if (test_bit(c, scan_mask))
			ret = iio_backend_chan_enable(st->back, c);
		else
			ret = iio_backend_chan_disable(st->back, c);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct ad7768_chip_info ad7768_chip_info = {
	.id = ID_AD7768,
	.name = "ad7768",
	.num_channels = 8,
};

static const struct ad7768_chip_info ad7768_4_chip_info = {
	.id = ID_AD7768_4,
	.name = "ad7768-4",
	.num_channels = 4,
};

static const struct iio_info ad7768_info = {
	.debugfs_reg_access = ad7768_reg_access,
	.read_raw = ad7768_read_raw,
	.write_raw = ad7768_write_raw,
	.update_scan_mode = ad7768_update_scan_mode,
};

static void ad7768_set_available_sampl_freq(struct ad7768_state *st)
{
	unsigned int mode;
	unsigned int dec;
	unsigned int mclk = clk_get_rate(st->mclk);
	struct ad7768_avail_freq *avail_freq;

	for (mode = 0; mode < AD7768_NUM_POWER_MODES; mode++) {
		avail_freq = &st->avail_freq[mode];
		for (dec = ARRAY_SIZE(ad7768_dec_rate); dec > 0; dec--) {
			struct ad7768_freq_config freq_cfg;

			freq_cfg.dec_rate = dec - 1;
			freq_cfg.freq = mclk / (ad7768_dec_rate[dec - 1] *
					ad7768_mclk_div[mode]);
			avail_freq->freq_cfg[avail_freq->n_freqs++] = freq_cfg;
		}
	}

	/* The max frequency is not supported in one data line configuration */
	if (st->datalines == 1)
		st->avail_freq[AD7768_FAST_MODE].n_freqs--;
}

static int ad7768_input_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);

	return regmap_update_bits(st->regmap,
				  AD7768_REG_GPIO_CONTROL,
				  BIT(offset),
				  AD7768_GPIO_INPUT(offset));
}

static int ad7768_output_gpio(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	ret = regmap_update_bits(st->regmap,
				 AD7768_REG_GPIO_CONTROL,
				 BIT(offset),
				 AD7768_GPIO_OUTPUT(offset));
	if (ret < 0)
		return ret;

	return regmap_update_bits(st->regmap,
				  AD7768_REG_GPIO_WRITE,
				  BIT(offset),
				  (value << offset));
}

static int ad7768_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, AD7768_REG_GPIO_CONTROL, &val);
	if (ret < 0)
		return ret;

	if (val & BIT(offset))
		ret = regmap_read(st->regmap, AD7768_REG_GPIO_WRITE, &val);
	else
		ret = regmap_read(st->regmap, AD7768_REG_GPIO_READ, &val);
	if (ret < 0)
		return ret;

	return !!(val & BIT(offset));
}

static int ad7768_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, AD7768_REG_GPIO_CONTROL, &val);
	if (ret < 0)
		return ret;

	if (val & BIT(offset)) {
		ret = regmap_update_bits(st->regmap,
					 AD7768_REG_GPIO_WRITE,
					 BIT(offset),
					 (value << offset));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ad7768_gpio_setup(struct ad7768_state *st)
{
	int ret;

	ret = regmap_write(st->regmap,
			   AD7768_REG_GPIO_CONTROL,
			   AD7768_GPIO_UGPIO_ENABLE);
	if (ret < 0)
		return ret;

	st->gpiochip.label = st->chip_info->name;
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 5;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad7768_input_gpio;
	st->gpiochip.direction_output = ad7768_output_gpio;
	st->gpiochip.get = ad7768_get_gpio;
	st->gpiochip.set = ad7768_set_gpio;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static int ad7768_set_filter_mode(struct iio_dev *indio_dev, const struct iio_chan_spec *chan, unsigned int mode)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;
	unsigned int ch_mode;

	ret = ad7768_get_channel_mode(indio_dev, chan->address, &ch_mode);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, AD7768_CH_MODE(ch_mode),
				 AD7768_CH_MODE_FILTER_TYPE_MSK,
				 AD7768_CH_MODE_FILTER_TYPE_MODE(mode));
	if (ret < 0)
		goto out;

	ad7768_sync(st);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_get_filter_mode(struct iio_dev *indio_dev, const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	unsigned int ch_mode;
	int ret;

	ret = ad7768_get_channel_mode(indio_dev, chan->address, &ch_mode);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, AD7768_CH_MODE(ch_mode), &ch_mode);
	if (ret < 0)
		return ret;

	return AD7768_CH_MODE_GET_FILTER_TYPE(ch_mode);
}

static int ad7768_configure_precharge_buffers(struct iio_dev *indio_dev, struct ad7768_precharge_config *precharge_cfg)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	int ret;
	u8 ch;
	u16 prebuf_mask = 0;
	u8 prebuf1_val = 0;
	u8 prebuf2_val = 0;
	u8 refbufp_val = 0;
	u8 refbufn_val = 0;

	if (!precharge_cfg)
		return -EINVAL;

	mutex_lock(&st->lock);

	if (st->chip_info->num_channels == AD7768_MAX_CHANNEL) {
		for (ch = 0; ch < st->num_en_channels; ch++) {
			if (precharge_cfg[ch].prebufp_en)
				prebuf_mask |= AD7768_PREBUF_POS_EN(ch);

			if (precharge_cfg[ch].prebufn_en)
				prebuf_mask |= AD7768_PREBUF_NEG_EN(ch);

			if (precharge_cfg[ch].refbufp)
				refbufp_val |= BIT(ch);

			if (precharge_cfg[ch].refbufn)
				refbufn_val |= BIT(ch);
		}

		prebuf1_val = AD7768_PRECHARGE_BUF1_MSK(prebuf_mask);
		prebuf2_val = AD7768_PRECHARGE_BUF2_MSK(prebuf_mask);
	} else {
		for (ch = 0; ch < st->num_en_channels; ch++) {
			if (precharge_cfg[ch].prebufp_en)
				prebuf_mask |= AD7768_PREBUF_POS_EN(ch);

			if (precharge_cfg[ch].prebufn_en)
				prebuf_mask |= AD7768_PREBUF_NEG_EN(ch);

			if (precharge_cfg[ch].refbufp)
				refbufp_val |= AD7768_4_GET_REFBUF(ch);

			if (precharge_cfg[ch].refbufn)
				refbufn_val |= AD7768_4_GET_REFBUF(ch);
		}

		prebuf1_val = AD7768_4_PRECHARGE_BUF1_MSK(prebuf_mask);
		prebuf2_val = AD7768_4_PRECHARGE_BUF2_MSK(prebuf_mask);
	}

	pr_info("Precharge mask: 0x%02x:" , prebuf_mask);

	pr_info("Precharge 1 value: 0x%02x:" , prebuf1_val);
	pr_info("Precharge 2 value: 0x%02x:" , prebuf2_val);

	ret = regmap_write(st->regmap, AD7768_PRECHARGE_BUF1, prebuf1_val);
	if (ret < 0)
		goto unlock;

	ret = regmap_write(st->regmap, AD7768_PRECHARGE_BUF2, prebuf2_val);
	if (ret < 0)
		goto unlock;

	ret = regmap_write(st->regmap, AD7768_REFP_BUF, refbufp_val);
	if (ret < 0)
		goto unlock;

	ret = regmap_write(st->regmap, AD7768_REFN_BUF, refbufn_val);

unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static const char *const ad7768_filter_types[] = {
	[AD7768_FILTER_TYPE_WIDEBAND] = "wideband",
	[AD7768_FILTER_TYPE_SINC5] = "sinc5",
};

static const struct iio_enum ad7768_filter_types_enum = {
	.items = ad7768_filter_types,
	.num_items = ARRAY_SIZE(ad7768_filter_types),
	.set = ad7768_set_filter_mode,
	.get = ad7768_get_filter_mode,
};

static struct iio_chan_spec_ext_info ad7768_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SEPARATE,
		 &ad7768_filter_types_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SEPARATE, &ad7768_filter_types_enum),
	{ },
};

static const struct iio_chan_spec ad7768_channel_template = {
	.type = IIO_VOLTAGE,
	.info_mask_separate =	BIT(IIO_CHAN_INFO_CALIBBIAS) | \
				BIT(IIO_CHAN_INFO_CALIBSCALE) | \
				BIT(IIO_CHAN_INFO_PHASE) |\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.indexed = 1,
	.scan_type = {
		.sign = 's',
		.realbits = 24,
		.storagebits = 32,
	},
	.ext_info = ad7768_ext_info,
};

static int ad7768_parse_config(struct iio_dev *indio_dev,
			       struct device *dev)
{
	struct ad7768_state *st = iio_priv(indio_dev);
	const unsigned int *available_datalines;
	struct iio_chan_spec *chan;
	unsigned int channel;
	unsigned int i, len, vcm_sel, vcm_pd , ch_mode, pwr_mode;
	int chan_idx = 0;
	int ret;

	st->num_en_channels = device_get_child_node_count(dev);

	if (st->num_en_channels > st->chip_info->num_channels)
		return dev_err_probe(dev, -EINVAL, "Too many channels defined\n");

	chan = devm_kcalloc(indio_dev->dev.parent, st->num_en_channels,
			    sizeof(*chan), GFP_KERNEL);
	if (!chan)
		return -ENOMEM;

	indio_dev->channels = chan;
	indio_dev->num_channels = st->num_en_channels;

	ret = regmap_write(st->regmap, AD7768_CH_STANDBY, 0x0F);
	if (ret < 0)
		return ret;

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &channel);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to parse reg property of %pfwP\n", child);

		ret = regmap_update_bits(st->regmap,
					 AD7768_CH_STANDBY,
					 BIT(channel),
					 0);
		if (ret < 0)
			return ret;

		ret = fwnode_property_read_u32(child, "adi,ch-mode", &ch_mode);
		if (ret)
			return dev_err_probe(dev, ret,
					      "Failed to parse property adi,ch-mode %pfwP\n", child);

		ret = ad7768_set_channel_mode(indio_dev, channel, ch_mode);
		if (ret)
			return ret;

		if (fwnode_property_read_bool(child, "adi,prebuf-pos-en")) {
			st->precharge_cfg[channel].prebufp_en = true;
		}

		if (fwnode_property_read_bool(child, "adi,prebuf-neg-en")) {
			st->precharge_cfg[channel].prebufn_en = true;
		}

		if (fwnode_property_read_bool(child, "adi,refbuf-pos-en")) {
			st->precharge_cfg[channel].refbufp = true;
		}

		if (fwnode_property_read_bool(child, "adi,refbuf-neg-en")) {
			st->precharge_cfg[channel].refbufn = true;
		}

		chan[chan_idx] = ad7768_channel_template;
		chan[chan_idx].address = channel;
		chan[chan_idx].channel = channel;
		chan[chan_idx].scan_index = channel;
		chan_idx++;
	}

	ret = ad7768_configure_precharge_buffers(indio_dev, st->precharge_cfg);
	if (ret < 0)
		return ret;

	st->datalines = 1;
	ret = device_property_read_u32(&st->spi->dev, "adi,data-lines-number",
				       &st->datalines);
	if (ret) {
		dev_err(&st->spi->dev, "Missing \"data-lines-number\" property\n");
		return ret;
	}

	ret = device_property_read_u32(&st->spi->dev, "adi,common-mode-output-v", &vcm_sel);
	if (ret) {
		dev_err(&st->spi->dev, "Missing \"adi,common-mode-output-V\" property\n");
		return ret;
	}

	ret = regmap_update_bits(st->regmap,
				 AD7768_GENERAL_CONFIG,
				 AD7768_GEN_CONFIG_VCM_SEL_MSK,
				 vcm_sel);
	if (ret < 0)
		return ret;

	vcm_pd = device_property_read_bool(&st->spi->dev, "adi,vcm-power-down");

	ret = regmap_update_bits(st->regmap,
				 AD7768_GENERAL_CONFIG,
				 AD7768_GEN_CONFIG_VCM_PD,
				 vcm_pd ? AD7768_GEN_CONFIG_VCM_PD : 0);
	if (ret < 0)
		return ret;

	ret = device_property_read_u32(&st->spi->dev, "adi,power-mode",
				       &pwr_mode);
	if (ret) {
		dev_err(&st->spi->dev, "Missing \"adi,power-mode\" property\n");
		return ret;
	}

	ret = ad7768_set_power_mode(indio_dev, pwr_mode);
	if (ret) {
		dev_err(&st->spi->dev, "Failed to set power mode\n");
		return ret;
	}

	for (i = 0; i < indio_dev->num_channels; i++) {
		unsigned int ch = indio_dev->channels[i].channel;
		ret = ad7768_set_sampling_freq(indio_dev, st->sampling_freq, ch);
		if (ret < 0) {
			dev_err(&st->spi->dev, "Failed to set sampling freq for channel %d\n", ch);
			return ret;
		}
	}

	switch (st->chip_info->id) {
	case ID_AD7768:
		available_datalines = ad7768_available_datalines;
		len = ARRAY_SIZE(ad7768_available_datalines);
		break;
	case ID_AD7768_4:
		available_datalines = ad7768_4_available_datalines;
		len = ARRAY_SIZE(ad7768_4_available_datalines);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		if (available_datalines[i] == st->datalines)
			return 0;
	}

	return -EINVAL;
}

static int ad7768_probe(struct spi_device *spi)
{
	struct gpio_desc *gpio_reset;
	struct iio_dev *indio_dev;
	struct ad7768_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, st);

	mutex_init(&st->lock);

	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return -ENODEV;

	st->mclk = devm_clk_get_enabled(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);

	/* Create regmap config with chip-specific readable_reg function */
	struct regmap_config regmap_cfg = ad7768_regmap_config;
	if (st->chip_info->id == ID_AD7768) {
		regmap_cfg.readable_reg = ad7768_readable_reg;
	} else {
		regmap_cfg.readable_reg = ad7768_4_readable_reg;
	}

	st->regmap = devm_regmap_init(&spi->dev, &ad7768_regmap_bus, spi, &regmap_cfg);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	/* get out of reset state */
	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		fsleep(2);
		gpiod_set_value_cansleep(gpio_reset, 0);
		fsleep(1660);
	}

	ad7768_set_available_sampl_freq(st);

	ret = ad7768_parse_config(indio_dev, &spi->dev);
	if (ret < 0)
		return ret;

	ret = regmap_update_bits(st->regmap, AD7768_INTERFACE_CFG,
				 AD7768_INTERFACE_CFG_CRC_SELECT_MSK,
				 AD7768_INTERFACE_CFG_CRC_SELECT);
	if (ret < 0)
		return ret;

	ret = ad7768_gpio_setup(st);
	if (ret < 0)
		return ret;

	indio_dev->name = st->chip_info->name;
	indio_dev->info = &ad7768_info;

	st->back = devm_iio_backend_get(&spi->dev, NULL);
	if (IS_ERR(st->back))
		return PTR_ERR(st->back);

	ret = devm_iio_backend_request_buffer(&spi->dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(&spi->dev, st->back);
	if (ret)
		return ret;

	ret = iio_backend_set_num_lanes(st->back, st->datalines);
	if (ret)
		return ret;
	ret = iio_backend_crc_enable(st->back);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(&spi->dev, 1000);
	pm_runtime_use_autosuspend(&spi->dev);
	pm_runtime_set_active(&spi->dev);
	pm_runtime_enable(&spi->dev);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static int ad7768_runtime_suspend(struct device *dev) {
	u8 val = FIELD_PREP(AD7768_SLEEP_MODE_MSK, 0x1);

	struct ad7768_state *st = dev_get_drvdata(dev);
	return regmap_write(st->regmap, AD7768_POWER_MODE, val);
}

static int ad7768_runtime_resume(struct device *dev) {
	u8 val = FIELD_PREP(AD7768_SLEEP_MODE_MSK, 0x0);

	struct ad7768_state *st = dev_get_drvdata(dev);
	return regmap_write(st->regmap, AD7768_POWER_MODE, val);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ad7768_pm_ops, ad7768_runtime_suspend,
	ad7768_runtime_resume);

static const struct of_device_id ad7768_of_match[]  = {
	{ .compatible = "adi,ad7768", .data = &ad7768_chip_info },
	{ .compatible = "adi,ad7768-4", .data = &ad7768_4_chip_info },
	{},
};

static const struct spi_device_id ad7768_spi_id[] = {
	{"ad7768", (kernel_ulong_t)&ad7768_chip_info},
	{"ad7768-4", (kernel_ulong_t)&ad7768_4_chip_info},
	{},
};
MODULE_DEVICE_TABLE(spi, ad7768_spi_id);

static struct spi_driver ad7768_driver = {
	.probe = ad7768_probe,
	.driver = {
		.name   = "ad7768",
		.of_match_table = ad7768_of_match,
		.pm = &ad7768_pm_ops,
	},
	.id_table = ad7768_spi_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 ADC driver");
MODULE_LICENSE("GPL");
