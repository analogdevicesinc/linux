// SPDX-License-Identifier: GPL-2.0
/*
 * AD7294/AD7294-2 I2C driver
 *
 * Copyright (c) 2024 Analog Devices Inc.
 * Author: Anshul Dalal <anshulusr@gmail.com>
 */

#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>

#define AD7294_REG_CMD	  0x00
#define AD7294_REG_DAC_A  0x01

#define AD7294_DAC_CHAN(chan_id)                                    \
	{                                                           \
		.type = IIO_VOLTAGE, .channel = chan_id,            \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),       \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW), \
		.indexed = 1, .output = 1,                          \
	}

bool ad7294_readable_reg(struct device *dev, unsigned int reg)
{
	return reg != AD7294_REG_CMD;
};

static const struct regmap_config ad7294_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x27,
	.readable_reg = ad7294_readable_reg,
};

struct ad7294_data {
	struct mutex lock;
	struct regmap *regmap;
	u16 dac_value[2];
};

struct iio_chan_spec ad7294_chan_spec[] = {
	AD7294_DAC_CHAN(0),
	AD7294_DAC_CHAN(1),
	AD7294_DAC_CHAN(2),
	AD7294_DAC_CHAN(3),
};

static int ad7294_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct ad7294_data *data = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_VOLTAGE:
			*val = data->dac_value[chan->channel];
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	}
	return -EINVAL;
}

static int ad7294_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	struct ad7294_data *data = iio_priv(indio_dev);

	guard(mutex)(&data->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->output) {
			/* DAC has 12-bit channels */
			if (val < 0 || val > 0xFFF || val2)
				return -EINVAL;
			ret = regmap_write(data->regmap,
					   AD7294_REG_DAC_A + chan->channel,
					   val);
			if (ret)
				return ret;
			data->dac_value[chan->channel] = val;
		}
	}

	return 0;
}

static int ad7294_reg_access(struct iio_dev *indio_dev, unsigned reg,
			     unsigned writeval, unsigned *readval)
{
	struct ad7294_data *data = iio_priv(indio_dev);
	if (readval)
		return regmap_read(data->regmap, reg, readval);
	return regmap_write(data->regmap, reg, writeval);
}

struct iio_info ad7294_info = {
	.read_raw = ad7294_read_raw,
	.write_raw = ad7294_write_raw,
	.debugfs_reg_access = ad7294_reg_access,
};

static int ad7294_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct ad7294_data *data;

	dev_info(&client->dev, "Driver Probed\n");
	indio_dev =
		devm_iio_device_alloc(&client->dev, sizeof(struct ad7294_data));
	if (!indio_dev)
		return -1;
	indio_dev->name = "ad7294";
	indio_dev->info = &ad7294_info;
	indio_dev->channels = ad7294_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(ad7294_chan_spec);

	data = iio_priv(indio_dev);

	mutex_init(&data->lock);

	data->regmap = devm_regmap_init_i2c(client, &ad7294_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(data->regmap),
				     "regmap initialization failed\n");

	return devm_iio_device_register(&client->dev, indio_dev);
}

static void ad7294_remove(struct i2c_client *client)
{
	dev_info(&client->dev, "Driver Removed\n");
}

static struct of_device_id ad7294_of_table[] = { { .compatible = "adi,ad7294" },
						 { .compatible =
							   "adi,ad7294-2" },
						 { /* Sentinel */ } };

static struct i2c_driver ad7294_driver = {
	.driver = { .name = "ad7294", .of_match_table = ad7294_of_table },
	.probe = ad7294_probe,
	.remove = ad7294_remove,
};

module_i2c_driver(ad7294_driver);

MODULE_AUTHOR("Anshul Dalal <anshulusr@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD7294/AD7294-2 ADDAC");
MODULE_LICENSE("GPL");
