// SPDX-License-Identifier: GPL-2.0
/*
 * AD8366 and similar Gain Amplifiers
 * This driver supports the following gain amplifiers:
 *   AD8366 Dual-Digital Variable Gain Amplifier (VGA)
 *   ADA4961 BiCMOS RF Digital Gain Amplifier (DGA)
 *   ADL5240 Digitally controlled variable gain amplifier (VGA)
 *   HMC792A 0.25 dB LSB GaAs MMIC 6-Bit Digital Attenuator
 *   HMC1119 0.25 dB LSB, 7-Bit, Silicon Digital Attenuator
 *
 * Copyright 2012-2019 Analog Devices Inc.
 */

#include <linux/bitrev.h>
#include <linux/bits.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include <linux/iio/iio.h>

enum ad8366_type {
	ID_AD8366,
	ID_ADA4961,
	ID_ADL5240,
	ID_HMC792,
	ID_HMC1119,
};

struct ad8366_info {
	int gain_min;
	int gain_max;
	int gain_step;
	size_t num_channels;
	size_t (*pack_code)(const unsigned char *code, size_t num_channels,
			    unsigned char *data);
};

struct ad8366_state {
	struct spi_device	*spi;
	struct mutex            lock; /* protect sensor state */
	unsigned char		ch[2];
	const struct ad8366_info *info;
	/*
	 * DMA (thus cache coherency maintenance) may require the
	 * transfer buffers to live in their own cache lines.
	 */
	unsigned char		data[2] __aligned(IIO_DMA_MINALIGN);
};

static size_t ad8366_pack_code(const unsigned char *code, size_t num_channels,
			       unsigned char *data)
{
	u8 ch_a = bitrev8(code[0]) >> 2;
	u8 ch_b = bitrev8(code[1]) >> 2;

	put_unaligned_be16((ch_b << 6) | ch_a, &data[0]);
	return sizeof(__be16);
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
	},
	[ID_ADL5240] = {
		.gain_min = -11500,
		.gain_max = 20000,
		.gain_step = 500,
		.num_channels = 1,
	},
	[ID_HMC792] = {
		.gain_min = -15750,
		.gain_max = 0,
		.gain_step = 250,
		.num_channels = 1,
	},
	[ID_HMC1119] = {
		.gain_min = -31750,
		.gain_max = 0,
		.gain_step = -250,
		.num_channels = 1,
	},
};

static int ad8366_write_code(struct ad8366_state *st)
{
	const struct ad8366_info *inf = st->info;
	size_t len = 1;

	if (inf->pack_code)
		len = inf->pack_code(st->ch, inf->num_channels, st->data);
	else
		st->data[0] = st->ch[0];

	return spi_write(st->spi, st->data, len);
}

static int ad8366_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct ad8366_state *st = iio_priv(indio_dev);
	const struct ad8366_info *inf = st->info;
	int ret;
	int code, gain = 0;

	mutex_lock(&st->lock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		code = st->ch[chan->channel];
		gain = inf->gain_step > 0 ? inf->gain_min : inf->gain_max;
		gain += inf->gain_step * code;
		/* Values in dB */
		*val = gain / 1000;
		*val2 = (gain % 1000) * 1000;

		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
};

static int ad8366_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad8366_state *st = iio_priv(indio_dev);
	const struct ad8366_info *inf = st->info;
	int code = 0, gain;
	int ret;

	/* Values in dB */
	if (val < 0)
		gain = (val * 1000) - (val2 / 1000);
	else
		gain = (val * 1000) + (val2 / 1000);

	if (gain > inf->gain_max || gain < inf->gain_min)
		return -EINVAL;

	gain -= inf->gain_step > 0 ? inf->gain_min : inf->gain_max;
	code = DIV_ROUND_CLOSEST(gain, inf->gain_step);

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		st->ch[chan->channel] = code;
		ret = ad8366_write_code(st);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
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
	struct device *dev = &spi->dev;
	struct reset_control *rstc;
	struct iio_dev *indio_dev;
	struct ad8366_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulator\n");

	st->spi = spi;
	st->info = &ad8366_infos[spi_get_device_id(spi)->driver_data];

	rstc = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rstc))
		return dev_err_probe(dev, PTR_ERR(rstc),
				     "Failed to get reset controller\n");

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad8366_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad8366_channels;
	indio_dev->num_channels = st->info->num_channels;

	ret = ad8366_write_code(st);
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to write initial gain\n");

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ad8366_id[] = {
	{"ad8366",  ID_AD8366},
	{"ada4961", ID_ADA4961},
	{"adl5240", ID_ADL5240},
	{"hmc792a", ID_HMC792},
	{"hmc1119", ID_HMC1119},
	{ }
};
MODULE_DEVICE_TABLE(spi, ad8366_id);

static struct spi_driver ad8366_driver = {
	.driver = {
		.name	= KBUILD_MODNAME,
	},
	.probe		= ad8366_probe,
	.id_table	= ad8366_id,
};

module_spi_driver(ad8366_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD8366 and similar Gain Amplifiers");
MODULE_LICENSE("GPL v2");
