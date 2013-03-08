/*
 * AD9467 SPI DAC driver for DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
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

#include <linux/clk.h>
#include <linux/clkdev.h>

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

		if ((reg == ADC_REG_TRANSFER) && (val == TRANSFER_SYNC) &&
			(spi_get_device_id(spi)->driver_data == CHIPID_AD9265))
			ad9467_spi_write(spi, ADC_REG_TRANSFER, 0);

		return 0;
	}

	return -ENODEV;
}

static int __devinit ad9250_setup(struct spi_device *spi)
{
	int ret;
	unsigned pll_stat;
	static int sel = 0;

	ret = ad9467_spi_write(spi, 0x5f, (0x16 | 0x1)); // trail bits, ilas normal & pd
	ret |= ad9467_spi_write(spi, 0x5e, 0x22); // m=2, l=2
	ret |= ad9467_spi_write(spi, 0x66, sel++); // lane id
	ret |= ad9467_spi_write(spi, 0x67, sel++); // lane id
	ret |= ad9467_spi_write(spi, 0x6e, 0x81); // scr, 2-lane
	ret |= ad9467_spi_write(spi, 0x70, 0x1f); // no. of frames per multi frame
	ret |= ad9467_spi_write(spi, 0x3a, 0x1e); // sysref enabled
	ret |= ad9467_spi_write(spi, 0x5f, (0x16 | 0x0)); // enable
	ret |= ad9467_spi_write(spi, 0x14, 0x00); // offset binary
	ret |= ad9467_spi_write(spi, 0x0d, 0x00); // test patterns

	ret |= ad9467_spi_write(spi, 0xff, 0x01);
	ret |= ad9467_spi_write(spi, 0xff, 0x00);

	pll_stat = ad9467_spi_read(spi, 0x0A);

	dev_info(&spi->dev, "PLL %s, JESD204B Link %s\n",
		 pll_stat & 0x80 ? "LOCKED" : "UNLOCKED",
		 pll_stat & 0x01 ? "Ready" : "Fail");

	return ret;
}

static int __devinit ad9467_probe(struct spi_device *spi)
{
	struct axiadc_converter *conv;
	struct clk *clk = NULL;
	int ret;

	clk = devm_clk_get(&spi->dev, NULL);
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	spi->mode = SPI_MODE_0 | SPI_3WIRE;

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	conv->clk = clk;

	conv->id = ad9467_spi_read(spi, ADC_REG_CHIP_ID);
	if (conv->id != spi_get_device_id(spi)->driver_data) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", conv->id);
 		ret = -ENODEV;
 		goto out;
	}

	if (conv->id == CHIPID_AD9250) {
		ret = ad9250_setup(spi);
		if (ret) {
			dev_err(&spi->dev, "Failed to initialize\n");
			ret = -EIO;
			goto out;
		}
	}

	conv->write = ad9467_spi_write;
	conv->read = ad9467_spi_read;
	conv->spi = spi;
	spi_set_drvdata(spi, conv);

	return 0;

out:
	clk_disable_unprepare(clk);

	return ret;
}

static int ad9467_remove(struct spi_device *spi)
{
	struct axiadc_converter *conv = spi_get_drvdata(spi);

	clk_disable_unprepare(conv->clk);
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9467_id[] = {
	{"ad9467", CHIPID_AD9467},
	{"ad9643", CHIPID_AD9643},
	{"ad9250", CHIPID_AD9250},
	{"ad9265", CHIPID_AD9265},
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
