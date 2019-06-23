// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7292 SPI ADC driver
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

struct ad7292_state {
	struct spi_device *spi;
};

static int ad7292_setup(struct ad7292_state *st)
{
	return 0;
}

static int ad7292_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	return 0;
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	return 0;
}

static const struct iio_info ad7292_info = {
	.read_raw = ad7292_read_raw,
	.write_raw = &ad7768_write_raw,
};

static const struct iio_chan_spec ad7292_channels[] = {
};

static int ad7292_probe(struct spi_device *spi)
{
	struct ad7292_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	spi_set_drvdata(spi, indio_dev);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad7292_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7292_channels);
	indio_dev->info = &ad7292_info;

	ret = ad7292_setup(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad7292_id_table[] = {
	{ "ad7292", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad7292_id_table);

static const struct of_device_id ad7292_of_match[] = {
	{ .compatible = "adi,ad7292" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7292_of_match);

static struct spi_driver ad7292_driver = {
	.driver = {
		.name = "ad7292",
		.of_match_table = ad7292_of_match,
	},
	.probe = ad7292_probe,
	.id_table = ad7292_id_table,
};
module_spi_driver(ad7292_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt1@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD7292 ADC driver");
MODULE_LICENSE("GPL");
