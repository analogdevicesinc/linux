// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices AD5110 digital potentiometer driver
 *
 * Copyright (C) 2021 Mugilraj Dhavachelvan <dmugil2000@gmail.com>
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/AD5110_5112_5114.pdf
 *
 */

#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <linux/iio/iio.h>

struct ad5110_data {
	struct i2c_client       *client;
};

static const struct iio_chan_spec ad5110_channels[] = {
};

static int ad5110_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	return 0;
}

static int ad5110_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	return 0;
}

static const struct iio_info ad5110_info = {
	.read_raw = ad5110_read_raw,
	.write_raw = ad5110_write_raw,
};

static int ad5110_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct ad5110_data *data;
	
	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->dev.parent = dev;
	indio_dev->info = &ad5110_info;
	indio_dev->channels = ad5110_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad5110_channels);
	indio_dev->name = client->name;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad5110_of_match[] = {
	{ .compatible = "adi,ad5110" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5110_of_match);

static const struct i2c_device_id ad5110_id[] = {
	{ "ad5110", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ad5110_id);

static struct i2c_driver ad5110_driver = {
	.driver = {
		.name	= "ad5110",
		.of_match_table = ad5110_of_match,
	},
	.probe_new		= ad5110_probe,
	.id_table	= ad5110_id,
};

module_i2c_driver(ad5110_driver);

MODULE_AUTHOR("Mugilraj Dhavachelvan <dmugil2000@gmail.com>");
MODULE_DESCRIPTION("AD5110 digital potentiometer");
MODULE_LICENSE("GPL v2");
