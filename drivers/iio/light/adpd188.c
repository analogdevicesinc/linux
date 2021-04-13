// SPDX-License-Identifier: GPL-2.0+
/*
 * ADPD188 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/iio/iio.h>
#include <linux/device.h>
#include <linux/module.h>

#include "adpd188.h"

struct adpd188 {
	struct regmap *regmap;
};

static int adpd188_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int tx_val,
	unsigned int *rx_val)
{
	struct adpd188 *dev = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(dev->regmap, reg, rx_val);
	else
		return regmap_write(dev->regmap, reg, tx_val);
}

static const struct iio_info adpd188_info = {
	.debugfs_reg_access = &adpd188_reg_access,
};

int adpd188_core_probe(struct device *dev, struct regmap *regmap,
                       const char *name)
{
	struct iio_dev *indio_dev;
	struct adpd188 *dev_data;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*dev_data));
	if (!indio_dev) {
		dev_err(dev, "Error allocating IIO device memory: %ld\n",
			PTR_ERR(indio_dev));
		return -ENOMEM;
	}

	dev_set_drvdata(dev, indio_dev);
	dev_data = iio_priv(indio_dev);
	dev_data->regmap = regmap;

	indio_dev->dev.parent = dev;
	indio_dev->info = &adpd188_info;
	indio_dev->name = name;

	ret = iio_device_register(indio_dev);
        if (ret < 0)
                dev_err(dev, "iio_device_register failed: %d\n", ret);
	else
		dev_info(dev, "%s probed\n", indio_dev->name);

	return ret;
}
EXPORT_SYMBOL_GPL(adpd188_core_probe);

int adpd188_core_remove(struct device *dev)
{
        struct iio_dev *indio_dev = dev_get_drvdata(dev);

        iio_device_unregister(indio_dev);

	return 0;
}

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188core driver");
MODULE_LICENSE("GPL v2");

