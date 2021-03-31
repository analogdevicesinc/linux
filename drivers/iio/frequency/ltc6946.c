// SPDX-License-Identifier: GPL-2.0+
/*
 * LTC6946 driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

enum supported_parts {
	LTC6946,
};

struct ltc6946 {
	struct regmap *regmap;
};

static const struct regmap_config ltc6946_regmap_config = {
	.reg_bits = 7,
	.pad_bits = 1,
	.val_bits = 8,
	.read_flag_mask = BIT(0),
	.max_register = 0x0B,
};

static int ltc6946_reg_access(struct iio_dev *indio_dev,
	unsigned int reg,
	unsigned int tx_val,
	unsigned int *rx_val)
{
	struct ltc6946 *dev = iio_priv(indio_dev);

	if (rx_val)
		return regmap_read(dev->regmap, reg, rx_val);
	else
		return regmap_write(dev->regmap, reg, tx_val);
}

static const struct iio_info ltc6946_info = {
	.debugfs_reg_access = &ltc6946_reg_access,
};

static int ltc6946_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct regmap *regmap;
	struct ltc6946 *dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*dev));
	if (!indio_dev)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &ltc6946_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	spi_set_drvdata(spi, indio_dev);

	dev = iio_priv(indio_dev);
	dev->regmap = regmap;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &ltc6946_info;
	indio_dev->name = spi->dev.of_node->name;

	ret = iio_device_register(indio_dev);

	dev_info(&spi->dev, "%s probed\n", indio_dev->name);

	return ret;
}

static int ltc6946_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct spi_device_id ltc6946_id[] = {
	{ "ltc6946", LTC6946 },
	{}
};
MODULE_DEVICE_TABLE(spi, ltc6946_id);

static const struct of_device_id ltc6946_of_match[] = {
	{ .compatible = "adi,ltc6946" },
	{},
};
MODULE_DEVICE_TABLE(of, ltc6946_of_match);

static struct spi_driver ltc6946_driver = {
	.driver = {
			.name = "ltc6946",
			.of_match_table = of_match_ptr(ltc6946_of_match),
		},
	.probe = ltc6946_probe,
	.remove = ltc6946_remove,
	.id_table = ltc6946_id,
};
module_spi_driver(ltc6946_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTC6946");
MODULE_LICENSE("GPL v2");
