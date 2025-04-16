// SPDX-License-Identifier: GPL-2.0+
/*
 * MAX40080 Digital Current-Sense Amplifier driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include <linux/iio/iio.h>

struct max40080_state {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const struct regmap_config max40080_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int max40080_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long mask)
{
	struct max40080_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int max40080_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int write_val,
			       unsigned int *read_val)
{
	struct max40080_state *st = iio_priv(indio_dev);

	if (read_val)
		return regmap_bulk_read(st->regmap, reg, read_val, 2);

	return regmap_bulk_write(st->regmap, reg, &write_val, 2);
}

static const struct iio_info max40080_info = {
	.read_raw = max40080_read_raw,
	.debugfs_reg_access = &max40080_reg_access,
};

static const struct iio_chan_spec max40080_channels[] = {
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = 0,
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 1,
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int max40080_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct max40080_state *st;
	struct regmap *regmap;
	int ret;

	dev_info(dev, "Probing MAX40080\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct iio_dev));
	if (!indio_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, indio_dev);
	
	regmap = devm_regmap_init_i2c(client, &max40080_regmap_config);
	if (IS_ERR(regmap))
	    return PTR_ERR(regmap);
	
	st = iio_priv(indio_dev);
	st->client = client;
	st->regmap = regmap;

	indio_dev->name = "max40080";
	indio_dev->info = &max40080_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = max40080_channels;
	indio_dev->num_channels = ARRAY_SIZE(max40080_channels);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	return 0;
}


static const struct i2c_device_id max40080_i2c_ids[] = {
	{"adi,max40080", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max40080_i2c_ids);

static const struct of_device_id max40080_device_match[] = {
	{ .compatible = "adi,max40080" },
	{}
};
MODULE_DEVICE_TABLE(of, max40080_device_match);

static struct i2c_driver max40080_driver = {
	.driver = {
		.name = "max40080",
		.of_match_table = max40080_device_match,
	},
	.probe = max40080_probe,
};
module_i2c_driver(max40080_driver)

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices MAX40080 driver");
MODULE_LICENSE("GPL");