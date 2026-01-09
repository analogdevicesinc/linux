// SPDX-License-Identifier: GPL-2.0
/*
 * AD8366 and similar Gain Amplifiers
 * This driver supports the following gain amplifiers:
 *   AD8366 Dual-Digital Variable Gain Amplifier (VGA)
 *   ADA4961 BiCMOS RF Digital Gain Amplifier (DGA)
 *   ADL5240 Digitally controlled variable gain amplifier (VGA)
 *   ADRF5720: 0.5 dB LSB, 6-Bit, Silicon Digital Attenuator, 9 kHz to 40 GHz
 *   ADRF5730: 0.5 dB LSB, 6-Bit, Silicon Digital Attenuator, 100 MHz to 40 GHz
 *   ADRF5731: 2 dB LSB, 4-Bit, Silicon Digital Attenuator, 100 MHz to 40 GHz
 *   HMC271A: 1dB LSB 5-Bit Digital Attenuator SMT, 0.7 - 3.7 GHz
 *   HMC792A 0.25 dB LSB GaAs MMIC 6-Bit Digital Attenuator
 *   HMC1018A: 1.0 dB LSB GaAs MMIC 5-BIT DIGITAL ATTENUATOR, 0.1 - 30 GHz
 *   HMC1019A: 0.5 dB LSB GaAs MMIC 5-BIT DIGITAL ATTENUATOR, 0.1 - 30 GHz
 *   HMC1119 0.25 dB LSB, 7-Bit, Silicon Digital Attenuator
 *
 * Copyright 2012-2026 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/bitrev.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

enum ad8366_type {
	ID_AD8366,
	ID_ADA4961,
	ID_ADL5240,
	ID_ADRF5720,
	ID_ADRF5730,
	ID_ADRF5731,
	ID_HMC271,
	ID_HMC792,
	ID_HMC1018,
	ID_HMC1019,
	ID_HMC1119,
};

struct ad8366_state;

struct ad8366_info {
	int gain_min;
	int gain_max;
	int gain_step;
	int num_channels;
	size_t (*pack_code)(struct ad8366_state *st);
};

struct ad8366_state {
	struct spi_device	*spi;
	struct mutex            lock; /* protect sensor state */
	struct gpio_desc	*reset_gpio;
	struct gpio_desc	*enable_gpio;
	unsigned char		ch[2];
	enum ad8366_type	type;
	const struct ad8366_info *info;
	/*
	 * DMA (thus cache coherency maintenance) may require the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char		data[2] __aligned(IIO_DMA_MINALIGN);
};

static int ad8366_write_code(struct ad8366_state *st)
{
	const struct ad8366_info *inf = st->info;

	return spi_write(st->spi, st->data, inf->pack_code(st));
}

static size_t ad8366_pack_code(struct ad8366_state *st)
{
	u8 ch_a = bitrev8(st->ch[0] & 0x3F);
	u8 ch_b = bitrev8(st->ch[1] & 0x3F);

	st->data[0] = ch_b >> 4;
	st->data[1] = (ch_b << 4) | (ch_a >> 2);
	return 2;
}

static size_t simple_pack_code(struct ad8366_state *st)
{
	st->data[0] = st->ch[0];
	return 1;
}

static size_t adrf5731_pack_code(struct ad8366_state *st)
{
	st->data[0] = st->ch[0] << 2;
	return 1;
}

static size_t hmc271_pack_code(struct ad8366_state *st)
{
	st->data[0] = bitrev8(st->ch[0] & 0x1F) >> 3;
	return 1;
}

static const struct ad8366_info ad8366_infos[] = {
	[ID_AD8366] = {
		.gain_min = 4500,
		.gain_max = 20500,
		.gain_step = 253,
		.num_channels = 2,
		.pack_code = ad8366_pack_code,
	},
	[ID_ADA4961] = {
		.gain_min = -6000,
		.gain_max = 15000,
		.gain_step = -1000,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_ADL5240] = {
		.gain_min = -11500,
		.gain_max = 20000,
		.gain_step = 500,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_ADRF5720] = {
		.gain_min = -31500,
		.gain_max = 0,
		.gain_step = -500,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_ADRF5730] = {
		.gain_min = -31500,
		.gain_max = 0,
		.gain_step = -500,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_ADRF5731] = {
		.gain_min = -30000,
		.gain_max = 0,
		.gain_step = -2000,
		.num_channels = 1,
		.pack_code = adrf5731_pack_code,
	},
	[ID_HMC271] = {
		.gain_min = -31000,
		.gain_max = 0,
		.gain_step = 1000,
		.num_channels = 1,
		.pack_code = hmc271_pack_code,
	},
	[ID_HMC792] = {
		.gain_min = -15750,
		.gain_max = 0,
		.gain_step = 250,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_HMC1018] = {
		.gain_min = -31000,
		.gain_max = 0,
		.gain_step = 1000,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_HMC1019] = {
		.gain_min = -15500,
		.gain_max = 0,
		.gain_step = 500,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
	[ID_HMC1119] = {
		.gain_min = -31750,
		.gain_max = 0,
		.gain_step = -250,
		.num_channels = 1,
		.pack_code = simple_pack_code,
	},
};

static int ad8366_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct ad8366_state *st = iio_priv(indio_dev);
	const struct ad8366_info *inf = st->info;
	int gain = inf->gain_step > 0 ? inf->gain_min : inf->gain_max;

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		gain += inf->gain_step * st->ch[chan->channel];
		*val = gain / 1000;
		*val2 = (gain % 1000) * 1000;
		return IIO_VAL_INT_PLUS_MICRO_DB;
	default:
		return -EINVAL;
	}
};

static int ad8366_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad8366_state *st = iio_priv(indio_dev);
	const struct ad8366_info *inf = st->info;
	int code, gain, gain_base;

	if (val < 0)
		gain = (val * 1000) - (val2 / 1000);
	else
		gain = (val * 1000) + (val2 / 1000);

	if (gain > inf->gain_max || gain < inf->gain_min)
		return -EINVAL;

	gain_base = inf->gain_step > 0 ? inf->gain_min : inf->gain_max;
	code = DIV_ROUND_CLOSEST(gain - gain_base, inf->gain_step);

	guard(mutex)(&st->lock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		st->ch[chan->channel] = code;
		return ad8366_write_code(st);
	default:
		return -EINVAL;
	}
}

static int ad8366_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad8366_info = {
	.read_raw = &ad8366_read_raw,
	.write_raw = &ad8366_write_raw,
	.write_raw_get_fmt = &ad8366_write_raw_get_fmt,
};

#define AD8366_CHAN(_channel) {				\
	.type = IIO_VOLTAGE,				\
	.output = 1,					\
	.indexed = 1,					\
	.channel = _channel,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),\
}

static const struct iio_chan_spec ad8366_channels[] = {
	AD8366_CHAN(0),
	AD8366_CHAN(1),
};

static int ad8366_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad8366_state *st;
	struct device *dev = &spi->dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -EINVAL;

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulator\n");

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	st->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(st->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(st->enable_gpio),
				     "Failed to get enable GPIO\n");

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad8366_info;
	indio_dev->channels = ad8366_channels;
	indio_dev->num_channels = st->info->num_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad8366_write_code(st);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to write initial gain\n");

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ad8366_id[] = {
	{"ad8366", (kernel_ulong_t)&ad8366_infos[ID_AD8366]},
	{"ada4961", (kernel_ulong_t)&ad8366_infos[ID_ADA4961]},
	{"adrf5720", (kernel_ulong_t)&ad8366_infos[ID_ADRF5720]},
	{"adrf5730", (kernel_ulong_t)&ad8366_infos[ID_ADRF5730]},
	{"adrf5731", (kernel_ulong_t)&ad8366_infos[ID_ADRF5731]},
	{"adl5240", (kernel_ulong_t)&ad8366_infos[ID_ADL5240]},
	{"hmc271a", (kernel_ulong_t)&ad8366_infos[ID_HMC271]},
	{"hmc792a", (kernel_ulong_t)&ad8366_infos[ID_HMC792]},
	{"hmc1018a", (kernel_ulong_t)&ad8366_infos[ID_HMC1018]},
	{"hmc1019a", (kernel_ulong_t)&ad8366_infos[ID_HMC1019]},
	{"hmc1119", (kernel_ulong_t)&ad8366_infos[ID_HMC1119]},
	{ }
};
MODULE_DEVICE_TABLE(spi, ad8366_id);

static const struct of_device_id ad8366_of_match[] = {
	{ .compatible = "adi,ad8366", .data = &ad8366_infos[ID_AD8366] },
	{ .compatible = "adi,ada4961", .data = &ad8366_infos[ID_ADA4961] },
	{ .compatible = "adi,adrf5720", .data = &ad8366_infos[ID_ADRF5720] },
	{ .compatible = "adi,adrf5730", .data = &ad8366_infos[ID_ADRF5730] },
	{ .compatible = "adi,adrf5731", .data = &ad8366_infos[ID_ADRF5731] },
	{ .compatible = "adi,adl5240", .data = &ad8366_infos[ID_ADL5240] },
	{ .compatible = "adi,hmc271a", .data = &ad8366_infos[ID_HMC271] },
	{ .compatible = "adi,hmc792a", .data = &ad8366_infos[ID_HMC792] },
	{ .compatible = "adi,hmc1018a", .data = &ad8366_infos[ID_HMC1018] },
	{ .compatible = "adi,hmc1019a", .data = &ad8366_infos[ID_HMC1019] },
	{ .compatible = "adi,hmc1119", .data = &ad8366_infos[ID_HMC1119] },
	{ }
};
MODULE_DEVICE_TABLE(of, ad8366_of_match);

static struct spi_driver ad8366_driver = {
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= ad8366_of_match,
	},
	.probe		= ad8366_probe,
	.id_table	= ad8366_id,
};

module_spi_driver(ad8366_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD8366 and similar Gain Amplifiers");
MODULE_LICENSE("GPL v2");
