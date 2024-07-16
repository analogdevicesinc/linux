// SPDX-License-Identifier: GPL-2.0-only
/*
 * NXP P3T1085 Temperature Sensor Driver
 *
 * Copyright 2024 NXP
 */
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/module.h>
#include <linux/bitops.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/regmap.h>
#include <linux/limits.h>

#include "p3t1085.h"

static int p3t1085_read_raw(struct iio_dev *indio_dev,
		struct iio_chan_spec const *channel, int *val,
		int *val2, long mask)
{
	struct p3t1085_data *data = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(data->regmap, P3T1085_REG_TEMP, val);
		if (ret < 0) {
			dev_err(data->dev, "failed to read temperature register\n");
			return ret;
		}
		*val = *val >> 4;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static const struct iio_chan_spec p3t1085_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	},
};

static const struct iio_info p3t1085_info = {
	.read_raw = p3t1085_read_raw,
};

int p3t1085_probe(struct device *dev, int irq, int hw_id, struct regmap *regmap)
{
	struct iio_dev *iio_dev;
	struct p3t1085_data *data;
	int ret;

	iio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!iio_dev)
		return -ENOMEM;

	data = iio_priv(iio_dev);
	data->dev = dev;
	data->regmap = regmap;

	iio_dev->name = "p3t1085";
	iio_dev->modes = INDIO_DIRECT_MODE;
	iio_dev->info = &p3t1085_info;

	iio_dev->channels = p3t1085_channels;
	iio_dev->num_channels = ARRAY_SIZE(p3t1085_channels);

	ret = devm_iio_device_register(dev, iio_dev);
	if (ret)
		dev_info(dev, "Temperature sensor fail to probe %d\n", ret);
	else
		dev_info(dev, "Temperature sensor probe successfully\n");

	return ret;
}
EXPORT_SYMBOL_NS(p3t1085_probe, IIO_P3T1085);

MODULE_AUTHOR("Xiaoning Wang <xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("NXP P3T1085 driver");
MODULE_LICENSE("GPL");
