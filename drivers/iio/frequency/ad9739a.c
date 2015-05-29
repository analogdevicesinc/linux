/*
 * AD9739A SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2015 Analog Devices Inc.
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
#include <linux/of.h>
#include <linux/clk.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ad9739a.h"
#include "cf_axi_dds.h"

static int ad9739a_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[2];
	int ret;

	buf[0] = 0x80 | (0x7F & reg);

	ret = spi_write_then_read(spi, &buf[0], 1, &buf[1], 1);
	if (ret < 0)
		return ret;

	return buf[1];
}

static int ad9739a_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg & 0x7F;
	buf[1] = val;
	ret = spi_write_then_read(spi, buf, 2, NULL, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9739a_setup(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;
	int repeat, status;

	/* Configure for the 4-wire SPI mode with MSB. */
	ad9739a_write(spi, REG_MODE, 0x00);

	/* Software reset to default SPI values. */
	ad9739a_write(spi, REG_MODE, MODE_RESET);

	/* Clear the reset bit. */
	ad9739a_write(spi, REG_MODE, 0x00);

	/* Set the common-mode voltage of DACCLK_P and DACCLK_N inputs */
	ad9739a_write(spi, REG_CROSS_CNT1, CROSS_CNT1_CLKP_OFFSET(0xF));
	ad9739a_write(spi, REG_CROSS_CNT2, CROSS_CNT2_CLKN_OFFSET(0xF));

	/* Configure the Mu controller. */
	ad9739a_write(spi, REG_PHS_DET, PHS_DET_CMP_BST | PHS_DET_PHS_DET_AUTO_EN);
	ad9739a_write(spi, REG_MU_DUTY, MU_DUTY_MU_DUTY_AUTO_EN);
	ad9739a_write(spi, REG_MU_CNT2, MU_CNT2_SRCH_MODE(2) | MU_CNT2_SET_PHS(4));
	ad9739a_write(spi, REG_MU_CNT3, MU_CNT3_MUDEL(0x6C));

	for (repeat = 0; repeat < 3; repeat++) {
		ad9739a_write(spi, REG_MU_CNT4,
				MU_CNT4_SEARCH_TOL | MU_CNT4_RETRY | MU_CNT4_GUARD(0xB));
		ad9739a_write(spi, REG_MU_CNT1, MU_CNT1_GAIN(1));
		/* Enable the Mu controller search and track mode. */
		ad9739a_write(spi, REG_MU_CNT1, MU_CNT1_GAIN(1) | MU_CNT1_ENABLE);
		mdelay(10);
		status = ad9739a_read(spi, REG_MU_STAT1);
		if (status == MU_STAT1_MU_LKD)
			return 0;
	}

	dev_err(&spi->dev, "Mu lock failure\n\r");

	return -1;
}

static int ad9739a_prepare(struct cf_axi_converter *conv)
{
	struct spi_device *spi = conv->spi;
	int repeat, status;

	for (repeat = 0; repeat < 3; repeat++) {
		/* Set FINE_DEL_SKEW to 2. */
		ad9739a_write(spi, REG_LVDS_REC_CNT4,
				LVDS_REC_CNT4_DCI_DEL(0x7) | LVDS_REC_CNT4_FINE_DEL_SKEW(0x2));
		/* Disable the data Rx controller before enabling it. */
		ad9739a_write(spi, REG_LVDS_REC_CNT1, 0x00);
		/* Enable the data Rx controller for loop and IRQ. */
		ad9739a_write(spi, REG_LVDS_REC_CNT1, LVDS_REC_CNT1_RCVR_LOOP_ON);
		/* Enable the data Rx controller for search and track mode. */
		ad9739a_write(spi, REG_LVDS_REC_CNT1,
				LVDS_REC_CNT1_RCVR_LOOP_ON | LVDS_REC_CNT1_RCVR_CNT_ENA);
		mdelay(10);
		status = ad9739a_read(spi, REG_LVDS_REC_STAT9);
		if (status == (LVDS_REC_STAT9_RCVR_TRK_ON | LVDS_REC_STAT9_RCVR_LCK)) {
			return 0;
		}
	}

	dev_err(&spi->dev, "Rx data lock failure\n\r");

	return -1;
}

static unsigned long ad9739a_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static int ad9739a_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	return 0;
}

static int ad9739a_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	return 0;
}

static struct attribute *ad9739a_attributes[] = {
	NULL,
};

static const struct attribute_group ad9739a_attribute_group = {
	.attrs = ad9739a_attributes,
};

static int ad9739a_probe(struct spi_device *spi)
{
	struct cf_axi_converter *conv;
	struct clk *clk;
	unsigned id;
	int ret;

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL) {
		ret = -ENOMEM;
		dev_err(&spi->dev, "Failed to allocate memory\n");
		goto out;
	}

	id = ad9739a_read(spi, REG_PART_ID);
	if (id != AD9739A_ID) {
		ret = -ENODEV;
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		goto out;
	}

	conv->write = ad9739a_write;
	conv->read = ad9739a_read;
	conv->setup = ad9739a_prepare;

	conv->get_data_clk = ad9739a_get_data_clk;
	conv->write_raw = ad9739a_write_raw;
	conv->read_raw = ad9739a_read_raw;
	conv->attrs = &ad9739a_attribute_group;
	conv->spi = spi;
	conv->id = ID_AD9739A;

	clk = clk_get(&conv->spi->dev, "dac_clk");
	if (IS_ERR(clk)) {
		ret = -EPROBE_DEFER;
		dev_err(&spi->dev, "Failed to get dac_clk\n");
		goto out;
	}

	ret = clk_prepare_enable(clk);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to prepare dac_clk\n");
		goto out;
	}

	conv->clk[CLK_DAC] = clk;

	ret = ad9739a_setup(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	spi_set_drvdata(spi, conv);

	return 0;
out:
	return ret;
}

static int ad9739a_remove(struct spi_device *spi)
{
	spi_set_drvdata(spi, NULL);

	return 0;
}

static const struct spi_device_id ad9739a_id[] = {
	{"ad9739a", 9739},
	{}
};

MODULE_DEVICE_TABLE(spi, ad9739a_id);

static struct spi_driver ad9739a_driver = {
	.driver = {
		   .name = "ad9739a",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9739a_probe,
	.remove = ad9739a_remove,
	.id_table = ad9739a_id,
};
module_spi_driver(ad9739a_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9739 DAC");
MODULE_LICENSE("GPL v2");
