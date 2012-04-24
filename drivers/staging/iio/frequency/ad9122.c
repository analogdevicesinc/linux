/*
 * AD9122 SPI DAC driver for DDS PCORE/COREFPGA Module
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

#include "../iio.h"
#include "cf_ad9122.h"

#define CHIPID_AD9122 0x8

static const unsigned char ad9122_reg_defaults[][2] = {
	{0x00, 0x00},
	{0x00, 0x20},
	{0x00, 0x00},
	{0x01, 0x10},
	{0x03, 0x80},
	{0x04, 0x00},
	{0x05, 0x00},
	{0x08, 0x3F},
	{0x0A, 0x40},
	{0x0C, 0xD1},
	{0x0D, 0xD6},
	{0x10, 0x48},
	{0x11, 0x00},
	{0x16, 0x00},
	{0x17, 0x04},
	{0x18, 0x00},
	{0x1B, 0xE0},
	{0x1C, 0x01},
	{0x1D, 0x01},
	{0x1E, 0x01},
	{0x30, 0x00},
	{0x31, 0x00},
	{0x32, 0x00},
	{0x33, 0x00},
	{0x34, 0x00},
	{0x35, 0x00},
	{0x36, 0x00},
	{0x38, 0x00},
	{0x39, 0x00},
	{0x3A, 0x00},
	{0x3B, 0x00},
	{0x3C, 0x00},
	{0x3D, 0x00},
	{0x3E, 0x00},
	{0x3F, 0x00},
	{0x40, 0xF9},
	{0x41, 0x01},
	{0x42, 0x00},
	{0x43, 0x00},
	{0x44, 0xF9},
	{0x45, 0x01},
	{0x46, 0x00},
	{0x47, 0x00},
	{0x48, 0x02},
	{0x18, 0x02},
};

static int ad9122_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[2];
	int ret;

	buf[0] = 0x80 | reg;

	ret = spi_write_then_read(spi, &buf[0], 1, &buf[1], 1);
	if (ret < 0)
		return ret;

	return buf[1];
}

static int ad9122_write(struct spi_device *spi,
			 unsigned reg, unsigned val)
{
	unsigned char buf[2];
	int ret;

	buf[0] = reg;
	buf[1] = val;
	ret = spi_write(spi, buf, 2);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9122_setup(struct spi_device *spi, unsigned mode)
{
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(ad9122_reg_defaults); i++)
			ad9122_write(spi, ad9122_reg_defaults[i][0],
					     ad9122_reg_defaults[i][1]);
	i = 255;
	do {
		mdelay(1);
		ret = ad9122_read(spi, 0x18);
		if (ret < 0)
			return ret;

	} while (i-- && ((ret & (1 << 2) == 0)));

	ad9122_write(spi, 0x18, 0x0);
	ad9122_write(spi, 0x10, 0x88);

	i = 255;
	do {
		mdelay(1);
		ret = ad9122_read(spi, 0x12);
		if (ret < 0)
			return ret;

	} while (i-- && ((ret & (1 << 6)) == 0));

	return 0;
}


static int __devinit ad9122_probe(struct spi_device *spi)
{
	struct ad9122_converter *conv;
	unsigned id;
	int ret;

	conv = kzalloc(sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;


	id = ad9122_read(spi, 0x1f);
	if (id != CHIPID_AD9122) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
 		ret = -ENODEV;
 		goto out;
	}

	conv->write = ad9122_write;
	conv->read = ad9122_read;
	conv->setup = ad9122_setup;
	conv->spi = spi;
	conv->id = ID_AD9122;
	spi_set_drvdata(spi, conv);

	return 0;
out:
	kfree(conv);
	return ret;
}

static int ad9122_remove(struct spi_device *spi)
{
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9122_id[] = {
	{"ad9122", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9122_id);

static struct spi_driver ad9122_driver = {
	.driver = {
		.name	= "ad9122",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9122_probe,
	.remove		= __devexit_p(ad9122_remove),
	.id_table	= ad9122_id,
};
module_spi_driver(ad9122_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices AD9122 ADC");
MODULE_LICENSE("GPL v2");
