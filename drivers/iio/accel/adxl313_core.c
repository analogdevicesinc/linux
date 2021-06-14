// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADXL313 3-Axis Digital Accelerometer
 *
 * Copyright (c) 2021 Lucas Stankus <lucas.p.stankus@gmail.com>
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL313.pdf
 */

#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "adxl313.h"

struct adxl313_data {
	struct regmap *regmap;
};

static int adxl313_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	return 0;
}

static int adxl313_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	return 0;
}

static const struct iio_info adxl313_info = {
	.read_raw	= adxl313_read_raw,
	.write_raw	= adxl313_write_raw,
};

static const struct iio_chan_spec adxl313_channels[] = {
};

int adxl313_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name)
{
	struct adxl313_data *data;
	struct iio_dev *indio_dev;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = regmap;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &adxl313_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxl313_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl313_channels);

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(adxl313_core_probe);

MODULE_AUTHOR("Lucas Stankus <lucas.p.stankus@gmail.com>");
MODULE_DESCRIPTION("ADXL313 3-Axis Digital Accelerometer core driver");
MODULE_LICENSE("GPL v2");
