// SPDX-License-Identifier: GPL-2.0
/*
 * Generic IIO access driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/regmap.h>

#include "iio-regmap.h"

struct iio_regmap {
	struct device *dev;
	struct regmap *regmap;
};

static const struct iio_info iio_regmap_info = {
};

struct regmap_config *iio_regmap_read_config(struct device *dev)
{
	struct regmap_config *regmap_cfg;
	u32 reg_bits;
	u32 val_bits;
	int ret;

	regmap_cfg = devm_kzalloc(dev, sizeof(*regmap_cfg),
				  GFP_KERNEL);
	if (!regmap_cfg)
		return ERR_PTR(-ENOMEM);

	ret = device_property_read_u32(dev, "reg-bits", &reg_bits);
	if (ret < 0) {
		dev_err(dev, "Reading reg-bits property failed!\n");
		return ERR_PTR(-EINVAL);
	}

	ret = device_property_read_u32(dev, "val-bits", &val_bits);
	if (ret < 0) {
		dev_err(dev, "Reading val-bits property failed!\n");
		return ERR_PTR(-EINVAL);
	}

	regmap_cfg->reg_bits = reg_bits;
	regmap_cfg->val_bits = val_bits;

	return regmap_cfg;
}
EXPORT_SYMBOL_GPL(iio_regmap_read_config);

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
