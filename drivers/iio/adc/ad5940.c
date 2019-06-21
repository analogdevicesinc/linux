// SPDX-License-Identifier: GPL
/*
 * AD5940 SPI ADC driver
 *
 * Copyright (C) 2019 Song Qiang <songqiang1304521@gmail.com>
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

struct ad5940_state {
	struct spi_device *spi;
};

static int ad5940_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		*val = 1000;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 20;

		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

#define AD5940_CHANNEL(idx)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = idx,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	}

static const struct iio_chan_spec ad5940_channels[] = {
	AD5940_CHANNEL(0),
};

static const struct iio_info ad5940_info = {
	.read_raw = &ad5940_read_raw,
};

static int ad5940_setup(struct ad5940_state *st)
{
	return 0;
}

static int ad5940_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad5940_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad5940_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5940_channels);
	indio_dev->info = &ad5940_info;

	ret = ad5940_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad5940_dt_match[] = {
	{ .compatible = "adi,ad5940" },
	{},
};
MODULE_DEVICE_TABLE(of, ad5940_spi_ids);

static struct spi_driver ad5940_driver = {
	.driver = {
		.name = "ad5940",
		.of_match_table = ad5940_dt_match,
	},
	.probe = ad5940_probe,
};
module_spi_driver(ad5940_driver);

MODULE_AUTHOR("Song Qiang <songqiang1304521@gmail.com>");
MODULE_DESCRIPTION("Analog Device AD5940 ADC driver");
MODULE_LICENSE("GPL");
