// SPDX-License-Identifier: GPL-2.0-only
/*
 * Maxim Integrated
 * 7-bit, Multi-Channel Sink/Source Current DAC Driver
 * Copyright (C) 2017 Maxim Integrated
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <linux/time64.h>

#include <linux/iio/driver.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>

#define DS4422_MAX_DAC_CHANNELS		2
#define DS4424_MAX_DAC_CHANNELS		4

#define DS4424_DAC_MASK			GENMASK(6, 0)
#define DS4424_DAC_SOURCE		BIT(7)

#define DS4424_DAC_ADDR(chan)   ((chan) + 0xf8)

#define DS4424_CHANNEL(chan) { \
	.type = IIO_CURRENT, \
	.indexed = 1, \
	.output = 1, \
	.channel = chan, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
}

struct ds4424_chip_info {
	const char *name;
	u8 num_channels;
};

static const struct ds4424_chip_info ds4402_info = {
	.name = "ds4402",
	.num_channels = DS4422_MAX_DAC_CHANNELS,
};

static const struct ds4424_chip_info ds4404_info = {
	.name = "ds4404",
	.num_channels = DS4424_MAX_DAC_CHANNELS,
};

static const struct ds4424_chip_info ds4422_info = {
	.name = "ds4422",
	.num_channels = DS4422_MAX_DAC_CHANNELS,
};

static const struct ds4424_chip_info ds4424_info = {
	.name = "ds4424",
	.num_channels = DS4424_MAX_DAC_CHANNELS,
};

struct ds4424_data {
	struct i2c_client *client;
	struct mutex lock;
	uint8_t save[DS4424_MAX_DAC_CHANNELS];
	struct regulator *vcc_reg;
	uint8_t raw[DS4424_MAX_DAC_CHANNELS];
};

static const struct iio_chan_spec ds4424_channels[] = {
	DS4424_CHANNEL(0),
	DS4424_CHANNEL(1),
	DS4424_CHANNEL(2),
	DS4424_CHANNEL(3),
};

static int ds4424_get_value(struct iio_dev *indio_dev,
			     int *val, int channel)
{
	struct ds4424_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	ret = i2c_smbus_read_byte_data(data->client, DS4424_DAC_ADDR(channel));
	if (ret < 0)
		goto fail;

	*val = ret;

fail:
	mutex_unlock(&data->lock);
	return ret;
}

static int ds4424_set_value(struct iio_dev *indio_dev,
			     int val, struct iio_chan_spec const *chan)
{
	struct ds4424_data *data = iio_priv(indio_dev);
	int ret;

	mutex_lock(&data->lock);
	ret = i2c_smbus_write_byte_data(data->client,
			DS4424_DAC_ADDR(chan->channel), val);
	if (ret < 0)
		goto fail;

	data->raw[chan->channel] = val;

fail:
	mutex_unlock(&data->lock);
	return ret;
}

static int ds4424_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	int ret, regval;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ds4424_get_value(indio_dev, &regval, chan->channel);
		if (ret < 0) {
			dev_err_ratelimited(indio_dev->dev.parent,
					    "Failed to read channel %d: %pe\n",
					    chan->channel, ERR_PTR(ret));
			return ret;
		}

		*val = regval & DS4424_DAC_MASK;
		if (!(regval & DS4424_DAC_SOURCE))
			*val = -*val;

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ds4424_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	unsigned int abs_val;

	if (val2 != 0)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		abs_val = abs(val);
		if (abs_val > DS4424_DAC_MASK)
			return -EINVAL;

		/*
		 * Currents exiting the IC (Source) are positive. 0 is a valid
		 * value for no current flow; the direction bit (Source vs Sink)
		 * is treated as don't-care by the hardware at 0.
		 */
		if (val > 0)
			abs_val |= DS4424_DAC_SOURCE;

		return ds4424_set_value(indio_dev, abs_val, chan);

	default:
		return -EINVAL;
	}
}

static int ds4424_verify_chip(struct iio_dev *indio_dev)
{
	int ret, val;

	ret = ds4424_get_value(indio_dev, &val, 0);
	if (ret < 0)
		dev_err(&indio_dev->dev,
				"%s failed. ret: %d\n", __func__, ret);

	return ret;
}

static int ds4424_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ds4424_data *data = iio_priv(indio_dev);
	int ret = 0;
	int i;

	for (i = 0; i < indio_dev->num_channels; i++) {
		data->save[i] = data->raw[i];
		ret = ds4424_set_value(indio_dev, 0,
				&indio_dev->channels[i]);
		if (ret < 0)
			return ret;
	}
	return ret;
}

static int ds4424_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ds4424_data *data = iio_priv(indio_dev);
	int ret = 0;
	int i;

	for (i = 0; i < indio_dev->num_channels; i++) {
		ret = ds4424_set_value(indio_dev, data->save[i],
				&indio_dev->channels[i]);
		if (ret < 0)
			return ret;
	}
	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(ds4424_pm_ops, ds4424_suspend, ds4424_resume);

static const struct iio_info ds4424_iio_info = {
	.read_raw = ds4424_read_raw,
	.write_raw = ds4424_write_raw,
};

static int ds4424_probe(struct i2c_client *client)
{
	const struct ds4424_chip_info *chip_info;
	struct ds4424_data *data;
	struct iio_dev *indio_dev;
	int ret;

	chip_info = i2c_get_match_data(client);
	if (!chip_info)
		return -ENODEV;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	data->client = client;
	indio_dev->name = chip_info->name;

	data->vcc_reg = devm_regulator_get(&client->dev, "vcc");
	if (IS_ERR(data->vcc_reg))
		return dev_err_probe(&client->dev, PTR_ERR(data->vcc_reg),
				     "Failed to get vcc-supply regulator.\n");

	mutex_init(&data->lock);
	ret = regulator_enable(data->vcc_reg);
	if (ret < 0) {
		dev_err(&client->dev,
				"Unable to enable the regulator.\n");
		return ret;
	}

	/*
	 * The datasheet does not specify a power-up to I2C ready time.
	 * Maintain the existing conservative 1ms delay to ensure the
	 * device is ready for communication.
	 */
	fsleep(1 * USEC_PER_MSEC);

	ret = ds4424_verify_chip(indio_dev);
	if (ret < 0)
		goto fail;

	indio_dev->num_channels = chip_info->num_channels;
	indio_dev->channels = ds4424_channels;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ds4424_iio_info;

	ret = iio_device_register(indio_dev);
	if (ret < 0) {
		dev_err(&client->dev,
				"iio_device_register failed. ret: %d\n", ret);
		goto fail;
	}

	return ret;

fail:
	regulator_disable(data->vcc_reg);
	return ret;
}

static void ds4424_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct ds4424_data *data = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	regulator_disable(data->vcc_reg);
}

static const struct i2c_device_id ds4424_id[] = {
	{ "ds4402", (kernel_ulong_t)&ds4402_info },
	{ "ds4404", (kernel_ulong_t)&ds4404_info },
	{ "ds4422", (kernel_ulong_t)&ds4422_info },
	{ "ds4424", (kernel_ulong_t)&ds4424_info },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ds4424_id);

static const struct of_device_id ds4424_of_match[] = {
	{ .compatible = "maxim,ds4402", .data = &ds4402_info },
	{ .compatible = "maxim,ds4404", .data = &ds4404_info },
	{ .compatible = "maxim,ds4422", .data = &ds4422_info },
	{ .compatible = "maxim,ds4424", .data = &ds4424_info },
	{ }
};

MODULE_DEVICE_TABLE(of, ds4424_of_match);

static struct i2c_driver ds4424_driver = {
	.driver = {
		.name	= "ds4424",
		.of_match_table = ds4424_of_match,
		.pm     = pm_sleep_ptr(&ds4424_pm_ops),
	},
	.probe		= ds4424_probe,
	.remove		= ds4424_remove,
	.id_table	= ds4424_id,
};
module_i2c_driver(ds4424_driver);

MODULE_DESCRIPTION("Maxim DS4424 DAC Driver");
MODULE_AUTHOR("Ismail H. Kose <ismail.kose@maximintegrated.com>");
MODULE_AUTHOR("Vishal Sood <vishal.sood@maximintegrated.com>");
MODULE_AUTHOR("David Jung <david.jung@maximintegrated.com>");
MODULE_LICENSE("GPL v2");
