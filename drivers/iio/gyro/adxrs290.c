// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADXRS290 SPI Gyroscope Driver
 *
 * Copyright (C) 2020 Analog Devices, Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

struct adxrs290_state {
	struct spi_device *spi;
};

static int adxrs290_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long mask)
{
	return 0;
}

static int adxrs290_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val,
			      int val2,
			      long mask)
{
	return 0;
}

static const struct iio_chan_spec adxrs290_channels[] = {
};

static const struct iio_info adxrs290_info = {
	.read_raw = &adxrs290_read_raw,
	.write_raw = &adxrs290_write_raw,
};

static int adxrs290_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adxrs290_state *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = "adxrs290";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxrs290_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxrs290_channels);
	indio_dev->info = &adxrs290_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id adxrs290_of_match[] = {
	{ .compatible = "adi,adxrs290" },
	{ },
};
MODULE_DEVICE_TABLE(of, adxrs290_of_match);

static struct spi_driver adxrs290_driver = {
	.driver = {
		.name = "adxrs290",
		.of_match_table = adxrs290_of_match,
	},
	.probe = adxrs290_probe,
};
module_spi_driver(adxrs290_driver);

MODULE_AUTHOR("Nishant Malpani <nish.malpani25@gmail.com>");
MODULE_DESCRIPTION("Analog Devices ADXRS290 Gyroscope SPI driver");
MODULE_LICENSE("GPL");
