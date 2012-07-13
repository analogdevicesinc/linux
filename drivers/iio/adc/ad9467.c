/*
 * AD9467 SPI DAC driver for DDS PCORE/COREFPGA Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/io.h>

#include <linux/iio/iio.h>
#include "cf_axi_adc.h"

static int ad9467_spi_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = 0x80 | (reg >> 8);
		buf[1] = reg & 0xFF;

		ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
		if (ret < 0)
			return ret;

		return buf[2];
	}
	return -ENODEV;
}

static int ad9467_spi_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	if (spi) {
		buf[0] = reg >> 8;
		buf[1] = reg & 0xFF;
		buf[2] = val;
		ret = spi_write_then_read(spi, buf, 3, NULL, 0);
		if (ret < 0)
			return ret;

		return 0;
	}
	return -ENODEV;
}


static int __devinit ad9467_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	int ret;

	conv = kzalloc(sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	spi->mode = SPI_MODE_0 | SPI_3WIRE;

	conv->id = ad9467_spi_read(spi, ADC_REG_CHIP_ID);
	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
 		ret = -ENODEV;
 		goto out;
	}

	conv->write = ad9467_spi_write;
	conv->read = ad9467_spi_read;
	conv->spi = spi;
	spi_set_drvdata(spi, conv);

	return 0;

out:
	kfree(conv);
	return ret;
}

static int ad9467_remove(struct spi_device *spi)
{
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9467_id[] = {
	{"ad9467", CHIPID_AD9467},
	{"ad9643", CHIPID_AD9643},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9467_id);

static struct spi_driver ad9467_driver = {
	.driver = {
		.name	= "ad9467",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9467_probe,
	.remove		= __devexit_p(ad9467_remove),
	.id_table	= ad9467_id,
};
module_spi_driver(ad9467_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9467 ADC");
MODULE_LICENSE("GPL v2");
