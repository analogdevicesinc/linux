// SPDX-License-Identifier: GPL-2.0-only
/*
 *  max538x.c - Support for max5380/1/2 DAC
 *
 *  Copyright (c) 2023 Analog Devices Inc.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

struct max538x_chip_info {
	unsigned long int_vref;
	const struct iio_chan_spec channel;
};

enum max538x_device_type {
	MAX5380,
	MAX5381,
	MAX5382,
	MAX538X_FAMILY_SIZE,
};

struct max538x_data {
	struct i2c_client *client;
	struct regulator *vref_reg;
	const struct max538x_chip_info *chip_info;
	bool use_intref;
	int vref;
	u8 value;
};

static int max538x_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct max538x_data *data = iio_priv(indio_dev);
	int ret;
	u8 buf[1];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = i2c_master_recv(data->client, buf, sizeof(buf));
		if (ret < 0) {
			dev_err(&indio_dev->dev, "read error\n");
			return ret;
		}
		data->value = buf[0];
		*val = data->value;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = data->vref;
		*val2 = 8;
		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int max538x_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct max538x_data *data = iio_priv(indio_dev);
	int ret;
	u8 buf[1];

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > 255)
			return -EINVAL;
		data->value = val;
		buf[0] = data->value;
		ret = i2c_master_send(data->client, buf, sizeof(buf));
		if (ret < 0) {
			dev_err(&indio_dev->dev, "write error\n");
			return ret;
		}
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static const struct iio_info max538x_info = {
	.read_raw = max538x_read_raw,
	.write_raw = max538x_write_raw,
};

static const struct iio_chan_spec max538x_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
		BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct max538x_chip_info max538x_chip_info[] = {
	[MAX5380] = {
		.int_vref = 2000,
	},
	[MAX5381] = {
		.int_vref = 4000,
	},
	[MAX5382] = {
		.int_vref = 0,
	},
};

static int max538x_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct max538x_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->client = client;
	data->chip_info = &max538x_chip_info[id->driver_data];

	data->vref_reg = devm_regulator_get_optional(&data->client->dev, "vref");
	if (PTR_ERR(data->vref_reg) == -ENODEV) {
		/* Use Internal regulator */
		if (!data->chip_info->int_vref) {
			dev_err(&data->client->dev,
				"Voltage reference not found\n");
			return -EIO;
		}

		data->use_intref = true;
		data->vref = data->chip_info->int_vref;

		return 0;
	}

	if (id->driver_data == MAX5382) {
		data->vref_reg = devm_regulator_get(&data->client->dev, "vref");
		if (IS_ERR(data->vref_reg)) {
			dev_err(&data->client->dev,
				"Error getting voltage reference regulator\n");
			ret = PTR_ERR(data->vref_reg);
			goto error_disable_regulator_vref;
		}

		ret = regulator_enable(data->vref_reg);
		if (ret) {
			dev_err(&client->dev,
				"Failed to enable vrefl regulator: %d\n", ret);
			goto error_disable_regulator_vref;
		}

		ret = regulator_get_voltage(data->vref_reg);
		if (ret < 0) {
			dev_err(&client->dev,
				"Failed to read vrefl regulator: %d\n", ret);
			goto error_disable_regulator_vref;
		}

		data->vref = ((ret / 1000) * 900) / 1000;
		data->use_intref = false;
	} else {
		data->vref_reg = NULL;
	}

	i2c_set_clientdata(client, indio_dev);
	indio_dev->name = client->name;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &max538x_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = max538x_channels;
	indio_dev->num_channels = 1;

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&client->dev,
			"Failed to register iio device: %d\n", ret);
		goto disable_regulator_vref;
	}

	return 0;

disable_regulator_vref:
	regulator_disable(data->vref_reg);
	return ret;

error_disable_regulator_vref:
	regulator_disable(data->vref_reg);
	data->vref_reg = NULL;
	return ret;
}

static const struct i2c_device_id max538x_id[] = {
	{ "max5380", MAX5380 },
	{ "max5381", MAX5381 },
	{ "max5382", MAX5382 },
	{},
};
MODULE_DEVICE_TABLE(i2c, max538x_id);

static const struct of_device_id max538x_of_match[] = {
	{ .compatible = "maxim,max5380", },
	{ .compatible = "maxim,max5381", },
	{ .compatible = "maxim,max5382", },
	{},
};

MODULE_DEVICE_TABLE(of, max538x_of_match);
static struct i2c_driver max538x_driver = {
	.driver = {
		.name = "max538x",
		.of_match_table = max538x_of_match,
	},
	.probe = max538x_probe,
	.id_table = max538x_id,
};
module_i2c_driver(max538x_driver);

MODULE_AUTHOR("Marc Paolo Sosa marcpaolo.sosa@analog.com");
MODULE_DESCRIPTION("MAX5380/1/2 8-bit DAC driver");
MODULE_LICENSE("GPL v2");
