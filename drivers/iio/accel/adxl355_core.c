// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADXL355 3-Axis Digital Accelerometer IIO core driver
 *
 * Copyright (c) 2021 Puranjay Mohan <puranjay12@gmail.com>
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf
 */

#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "adxl355.h"

struct adxl355_data {
	struct regmap *regmap;
};

static int adxl355_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	return 0;
}

static int adxl355_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	return 0;
}

static const struct iio_info adxl355_info = {
	.read_raw	= adxl355_read_raw,
	.write_raw	= adxl355_write_raw,
};

static const struct iio_chan_spec adxl355_channels[] = {
};

int adxl355_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name)
{
	struct adxl355_data *data;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = regmap;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &adxl355_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxl355_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl355_channels);

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(adxl355_core_probe);

MODULE_AUTHOR("Puranjay Mohan <puranjay12@gmail.com>");
MODULE_DESCRIPTION("ADXL355 3-Axis Digital Accelerometer core driver");
MODULE_LICENSE("GPL v2");
