// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include "iio-regmap.h"

struct iio_regmap {
	struct device *dev;
	struct regmap *regmap;
};

static const struct iio_info iio_regmap_info = {
};

int iio_regmap_probe(struct device *dev, struct regmap *regmap,
		     const char *name)
{
	struct iio_dev *indio_dev;
	struct iio_regmap *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	dev_set_drvdata(dev, indio_dev);

	st->dev = dev;
	st->regmap = regmap;

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &iio_regmap_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret < 0)
		dev_err(&indio_dev->dev, "iio-regmap device register failed\n");

	return ret;
}
EXPORT_SYMBOL_GPL(iio_regmap_probe);

MODULE_AUTHOR("Alexandru Tachici <alexandru.tachici@analog.com>");
MODULE_DESCRIPTION("Generic IIO access driver");
MODULE_LICENSE("GPL v2");
