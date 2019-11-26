// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD5766, AD5767
 * Digital to Analog Converters driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

enum ad5766_type {
	ID_AD5766,
	ID_AD5767,
};

/**
 * struct ad5766_state - driver instance specific data
 * @spi:		    spi_device
 */

struct ad5766_state {
	struct spi_device *spi;
};

static int ad5766_setup(struct ad5766_state *st)
{
	return 0;
}

static int ad5766_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad5766_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->dev.of_node = spi->dev.of_node;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = ad5766_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad5766_dt_match[] = {
	{ .compatible = "adi,ad5766" },
	{ .compatible = "adi,ad5767" },
	{},
};
MODULE_DEVICE_TABLE(of, ad5766_dt_match);

static const struct spi_device_id ad5766_spi_ids[] = {
	{ "ad5766", ID_AD5766 },
	{ "ad5767", ID_AD5767 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5766_spi_ids);

static struct spi_driver ad5766_driver = {
	.driver = {
		.name = "ad5766",
		.of_match_table = ad5766_dt_match,
	},
	.probe = ad5766_probe,
	.id_table = ad5766_spi_ids,
};
module_spi_driver(ad5766_driver);

MODULE_AUTHOR("Denis-Gabriel Gheorghescu <denis.gheorghescu@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5766/AD5767 DACs");
MODULE_LICENSE("GPL v2");
