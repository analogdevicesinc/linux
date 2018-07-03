// SPDX-License-Identifier: GPL-2.0+
/*
 * AD5770R Digital to analog converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/property.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

/* SPI configuration registers */
#define AD5770R_INTERFACE_CONFIG_A	0x00

/* AD5770R configuration registers */
#define AD5770R_CHANNEL_CONFIG		0x14
#define AD5770R_OUTPUT_RANGE(ch)	(0x15 + (ch))
#define AD5770R_REFERENCE		0x1B
#define AD5770R_DAC_LSB(ch)		(0x26 + 2 * (ch))
#define AD5770R_DAC_MSB(ch)		(0x27 + 2 * (ch))
#define AD5770R_CH_SELECT		0x34
#define AD5770R_CH_ENABLE		0x44

/* AD5770R_INTERFACE_CONFIG_A */
#define AD5770R_ITF_CFG_A_SW_RESET(x)		(((x) & 0x1) | 0x80)

/* AD5770R_CHANNEL_CONFIG */
#define AD5770R_CFG_CH0_SINK_EN(x)		(((x) & 0x1) << 7)
#define AD5770R_CFG_SHUTDOWN_B(x, ch)		(((x) & 0x1) << (ch))

/* AD5770R_OUTPUT_RANGE */
#define AD5770R_RANGE_OUTPUT_SCALING(x)		(((x) & 0x3F) << 2)
#define AD5770R_RANGE_MODE(x)			((x) & 0x03)

/* AD5770R_REFERENCE */
#define AD5770R_REF_RESISTOR_SEL(x)		(((x) & 0x1) << 2)
#define AD5770R_REF_SEL(x)			(((x) & 0x3) << 0)

/* AD5770R_CH_ENABLE */
#define AD5770R_CH_SET(x, channel)		(((x) & 0x1) << (channel))

#define AD5770R_MAX_CHANNELS	6
#define AD5770R_MAX_CH_MODES	14
#define AD5770R_LOW_VREF	1250
#define AD5770R_HIGH_VREF	2500

enum ad5770r_ch {
	AD5770R_CH0 = 0,
	AD5770R_CH1,
	AD5770R_CH2,
	AD5770R_CH3,
	AD5770R_CH4,
	AD5770R_CH5
};

enum ad5770r_ch0_modes {
	AD5770R_CH0_0_300 = 0,
	AD5770R_CH0_NEG_60_0,
	AD5770R_CH0_NEG_60_300
};

enum ad5770r_ch1_modes {
	AD5770R_CH1_0_140_LOW_HEAD = 1,
	AD5770R_CH1_0_140_LOW_NOISE,
	AD5770R_CH1_0_250
};

enum ad5770r_ch2_5_modes {
	AD5770R_CH_LOW_RANGE = 0,
	AD5770R_CH_HIGH_RANGE
};

enum ad5770r_ref_v {
	AD5770R_EXT_2_5_V = 0,
	AD5770R_INT_1_25_V_OUT_ON,
	AD5770R_EXT_1_25_V,
	AD5770R_INT_1_25_V_OUT_OFF
};

struct ad5770r_out_range {
	unsigned char	out_scale;
	unsigned char	out_range_mode;
};

/**
 * struct ad5770R_state - driver instance specific data
 * @spi			spi_device
 * @regmap:		regmap
 * @regulator		fixed regulator for reference configuration
 * @gpio_reset		gpio descriptor
 * @output_mode		array contains channels output ranges
 * @vref		reference value
 * @internal_ref	internal reference flag
 * @ch_pwr_down		powerdown flags
 */
struct ad5770r_state {
	struct spi_device		*spi;
	struct regmap			*regmap;
	struct regulator		*vref_reg;
	struct gpio_desc		*gpio_reset;
	struct ad5770r_out_range	output_mode[AD5770R_MAX_CHANNELS];
	int				vref;
	bool				internal_ref;
	bool				ch_pwr_down[AD5770R_MAX_CHANNELS];
};

static const struct regmap_config ad5770r_spi_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

struct ad5770r_output_modes {
	enum ad5770r_ch ch;
	unsigned char mode;
	int min;
	int max;
};

static struct ad5770r_output_modes ad5770r_rng_tbl[] = {
	{ AD5770R_CH0, AD5770R_CH0_0_300, 0, 300 },
	{ AD5770R_CH0, AD5770R_CH0_NEG_60_0, -60, 0 },
	{ AD5770R_CH0, AD5770R_CH0_NEG_60_300, -60, 300 },
	{ AD5770R_CH1, AD5770R_CH1_0_140_LOW_HEAD, 0, 140 },
	{ AD5770R_CH1, AD5770R_CH1_0_140_LOW_NOISE, 0, 140 },
	{ AD5770R_CH1, AD5770R_CH1_0_250, 0, 250 },
	{ AD5770R_CH2, AD5770R_CH_LOW_RANGE, 0, 55 },
	{ AD5770R_CH2, AD5770R_CH_HIGH_RANGE, 0, 150 },
	{ AD5770R_CH3, AD5770R_CH_LOW_RANGE, 0, 45 },
	{ AD5770R_CH3, AD5770R_CH_HIGH_RANGE, 0, 100 },
	{ AD5770R_CH4, AD5770R_CH_LOW_RANGE, 0, 45 },
	{ AD5770R_CH4, AD5770R_CH_HIGH_RANGE, 0, 100 },
	{ AD5770R_CH5, AD5770R_CH_LOW_RANGE, 0, 45 },
	{ AD5770R_CH5, AD5770R_CH_HIGH_RANGE, 0, 100 },
};

static int ad5770r_set_output_mode(struct ad5770r_state *st,
				   const struct ad5770r_out_range *out_mode,
				   enum ad5770r_ch channel)
{
	unsigned int regval;

	regval = AD5770R_RANGE_OUTPUT_SCALING(out_mode->out_scale) |
		 AD5770R_RANGE_MODE(out_mode->out_range_mode);

	return regmap_write(st->regmap,
			    AD5770R_OUTPUT_RANGE(channel), regval);
}

static int ad5770r_set_reference(struct ad5770r_state *st)
{
	unsigned int regval = 0;

	if (st->internal_ref) {
		regval = AD5770R_REF_RESISTOR_SEL(0) |
			 AD5770R_REF_SEL(AD5770R_INT_1_25_V_OUT_OFF);
	} else {
		switch (st->vref) {
		case AD5770R_LOW_VREF:
			regval |= AD5770R_REF_SEL(AD5770R_EXT_1_25_V);
			break;
		case AD5770R_HIGH_VREF:
			regval |= AD5770R_REF_SEL(AD5770R_EXT_2_5_V);
			break;
		default:
			regval = AD5770R_REF_RESISTOR_SEL(st->internal_ref) |
				 AD5770R_REF_SEL(AD5770R_INT_1_25_V_OUT_OFF);
			break;
		}
	}

	return regmap_write(st->regmap, AD5770R_REFERENCE, regval);
}

static int ad5770r_soft_reset(struct ad5770r_state *st)
{
	return regmap_write(st->regmap, AD5770R_INTERFACE_CONFIG_A,
			    AD5770R_ITF_CFG_A_SW_RESET(1));
}

static int ad5770r_reset(struct ad5770r_state *st)
{
	if (st->gpio_reset) {
		gpiod_set_value(st->gpio_reset, 0);
		udelay(100);
		gpiod_set_value(st->gpio_reset, 1);
	} else {
		/* Perform a software reset */
		return ad5770r_soft_reset(st);
	}

	/* data must not be written during reset timeframe */
	mdelay(1); /* TODO update with value from datasheet once available */

	return 0;
}

static int ad5770r_get_range(struct ad5770r_state *st,
			     enum ad5770r_ch ch, int *min, int *max)
{
	int i;
	unsigned char tbl_ch, tbl_mode, out_range;

	out_range = st->output_mode[ch].out_range_mode;

	for (i = 0; i < AD5770R_MAX_CH_MODES; i++) {
		tbl_ch = ad5770r_rng_tbl[i].ch;
		tbl_mode = ad5770r_rng_tbl[i].mode;
		if (tbl_ch == ch && tbl_mode == out_range) {
			*min = ad5770r_rng_tbl[i].min;
			*max = ad5770r_rng_tbl[i].max;
			return 0;
		}
	}

	return -EINVAL;
}

static int ad5770r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long info)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	int max, min, ret;
	unsigned char buf[2];

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_bulk_read(st->regmap,
				       chan->address,
				       buf, 2);
		if (ret)
			return 0;
		*val = ((u16)buf[0] << 6) + (buf[1] >> 2);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = ad5770r_get_range(st, chan->channel, &min, &max);
		if (ret < 0)
			return ret;
		if (min < 0)
			min = 0;
		*val = max - min;
		*val2 = 14;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad5770r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long info)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	unsigned char buf[2];

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		buf[0] = ((u16)val >> 6) & 0xFF;
		buf[1] = (val & 0x3F) << 2;
		return regmap_bulk_write(st->regmap, chan->address,
					 buf, ARRAY_SIZE(buf));
	default:
		return -EINVAL;
	}
}

static int ad5770r_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct ad5770r_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info ad5770r_info = {
	.read_raw = ad5770r_read_raw,
	.write_raw = ad5770r_write_raw,
	.debugfs_reg_access = &ad5770r_reg_access,
};

static int ad5770r_store_output_range(struct ad5770r_state *st,
				      int min, int max, int index)
{
	int i;

	for (i = 0; i < AD5770R_MAX_CH_MODES; i++) {
		if (ad5770r_rng_tbl[i].ch != index)
			continue;
		if (ad5770r_rng_tbl[i].min != min ||
		    ad5770r_rng_tbl[i].max != max)
			continue;
		st->output_mode[index].out_range_mode = ad5770r_rng_tbl[i].mode;
		return 0;
	}

	return -EINVAL;
}

static ssize_t ad5770r_read_dac_powerdown(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan, char *buf)
{
	struct ad5770r_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->ch_pwr_down[chan->channel]);
}

static ssize_t ad5770r_write_dac_powerdown(struct iio_dev *indio_dev,
		uintptr_t private, const struct iio_chan_spec *chan,
		const char *buf, size_t len)
{
	struct ad5770r_state *st = iio_priv(indio_dev);
	unsigned int regval;
	bool readin;
	int ret;

	ret = kstrtobool(buf, &readin);
	if (ret)
		return ret;

	readin = !readin;

	regval = AD5770R_CFG_SHUTDOWN_B(readin, chan->channel);
	if (chan->channel == AD5770R_CH0 &&
	    st->output_mode[AD5770R_CH0].out_range_mode > AD5770R_CH0_0_300) {
		regval |= AD5770R_CFG_CH0_SINK_EN(readin);
		ret = regmap_update_bits(st->regmap, AD5770R_CHANNEL_CONFIG,
					 BIT(chan->channel) + BIT(7), regval);
	} else {
		ret = regmap_update_bits(st->regmap, AD5770R_CHANNEL_CONFIG,
					 BIT(chan->channel), regval);
	}
	if (ret)
		return ret;

	regval = AD5770R_CH_SET(readin, chan->channel);
	ret = regmap_update_bits(st->regmap, AD5770R_CH_ENABLE,
				 BIT(chan->channel), regval);
	if (ret)
		return ret;

	st->ch_pwr_down[chan->channel] = !readin;

	return ret ? ret : len;
}

static const struct iio_chan_spec_ext_info ad5770r_ext_info[] = {
	{
		.name = "powerdown",
		.read = ad5770r_read_dac_powerdown,
		.write = ad5770r_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	{ },
};

#define AD5770R_IDAC_CHANNEL(index, reg) {			\
	.type = IIO_CURRENT,					\
	.address = reg,						\
	.indexed = 1,						\
	.channel = index,					\
	.output = 1,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = ad5770r_ext_info,				\
}

static const struct iio_chan_spec ad5770r_channels[] = {
	AD5770R_IDAC_CHANNEL(0, AD5770R_DAC_MSB(0)),
	AD5770R_IDAC_CHANNEL(1, AD5770R_DAC_MSB(1)),
	AD5770R_IDAC_CHANNEL(2, AD5770R_DAC_MSB(2)),
	AD5770R_IDAC_CHANNEL(3, AD5770R_DAC_MSB(3)),
	AD5770R_IDAC_CHANNEL(4, AD5770R_DAC_MSB(4)),
	AD5770R_IDAC_CHANNEL(5, AD5770R_DAC_MSB(5)),
};

static int ad5770r_channel_config(struct ad5770r_state *st)
{
	int ret, tmp[2], min, max, i;
	unsigned int num;
	struct fwnode_handle *child;
	bool ch_config[AD5770R_MAX_CHANNELS] = {0};

	device_for_each_child_node(&st->spi->dev, child) {
		ret = fwnode_property_read_u32(child, "num", &num);
		if (ret)
			return ret;
		if (num > AD5770R_MAX_CHANNELS)
			return -EINVAL;

		ret = fwnode_property_read_u32_array(child,
						     "adi,range-microamp",
						     tmp, 2);
		if (ret)
			return ret;

		min = tmp[0] / 1000;
		max = tmp[1] / 1000;
		ret = ad5770r_store_output_range(st, min, max, num);
		if (ret)
			return ret;

		ch_config[num] = true;
	}

	for (i = 0; i < AD5770R_MAX_CHANNELS; i++)
		if (!ch_config[i])
			return -EINVAL;

	return ret;
}

static int ad5770r_init(struct ad5770r_state *st)
{
	int ret, i;

	st->gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
			 GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	/* Perform a reset */
	ret = ad5770r_reset(st);
	if (ret)
		return ret;

	/* Set output range */
	ret = ad5770r_channel_config(st);
	if (ret)
		return ret;

	for (i = 0; i < AD5770R_MAX_CHANNELS; i++) {
		ret = ad5770r_set_output_mode(st,
					      &st->output_mode[i], i);
		if (ret)
			return ret;
	}

	ret = ad5770r_set_reference(st);
	if (ret)
		return ret;

	/* Set outputs off */
	ret = regmap_write(st->regmap, AD5770R_CHANNEL_CONFIG, 0x00);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD5770R_CH_ENABLE, 0x00);
	if (ret)
		return ret;

	for (i = 0; i < AD5770R_MAX_CHANNELS; i++)
		st->ch_pwr_down[i] = true;

	return ret;
}

static int ad5770r_probe(struct spi_device *spi)
{
	struct ad5770r_state *st;
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	regmap = devm_regmap_init_spi(spi, &ad5770r_spi_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}
	st->regmap = regmap;

	st->vref_reg = devm_regulator_get(&spi->dev, "vref");
	if (!IS_ERR(st->vref_reg)) {
		ret = regulator_enable(st->vref_reg);
		if (ret)
			dev_err(&spi->dev,
				"Failed to enable vref regulators: %d\n", ret);

		ret = regulator_get_voltage(st->vref_reg);
		if (ret < 0) {
			st->vref = AD5770R_LOW_VREF;
			st->internal_ref = true;
		} else {
			st->vref = ret / 1000;
		}
	} else {
		st->vref = AD5770R_LOW_VREF;
		st->internal_ref = true;
	}

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad5770r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5770r_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5770r_channels);

	ret = ad5770r_init(st);
	if (ret < 0) {
		dev_err(&spi->dev, "AD5770R init failed\n");
		return ret;
	}

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static const struct spi_device_id ad5770r_id[] = {
	{ "ad5770r", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5770r_id);

static struct spi_driver ad5770r_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
	},
	.probe = ad5770r_probe,
	.id_table = ad5770r_id,
};

module_spi_driver(ad5770r_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5770R IDAC");
MODULE_LICENSE("GPL v2");
