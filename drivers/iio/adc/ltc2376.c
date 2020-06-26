// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Linear Technology LTC2376 Low Power SAR ADC
 *
 * Copyright (C) 2020 Analog Devices, Inc.
 */

#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

struct ltc2376_state {
	struct spi_device *spi;
	struct regulator *vref;

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;

	unsigned int num_bits;
};

static int ltc2376_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	return 0;
}

static int ltc2376_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	return 0;
}

static const struct iio_info ltc2376_info = {
	.read_raw = &ltc2376_read_raw,
	.write_raw = &ltc2376_write_raw,
};

static const struct iio_chan_spec ltc2376_channels[] = {
};

static int ltc2376_probe(struct spi_device *spi)
{
	struct ltc2376_state *st;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "ltc2376";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ltc2376_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltc2376_channels);
	indio_dev->info = &ltc2376_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ltc2376_of_match[] = {
	{ .compatible = "adi,ltc2376" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc2376_of_match);

static struct spi_driver ltc2376_driver = {
	.driver = {
		.name = "ltc2376",
		.of_match_table = ltc2376_of_match,
	},
	.probe = ltc2376_probe,
};
module_spi_driver(ltc2376_driver);

MODULE_AUTHOR("Meenal Parakh <meenalparakh18@gmail.com>");
MODULE_DESCRIPTION("Linear Technology LTC2376 SAR ADC driver");
MODULE_LICENSE("GPL");
