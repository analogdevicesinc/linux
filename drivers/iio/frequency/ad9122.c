/*
 * AD9122 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
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
#include "ad9122.h"
#include "cf_axi_dds.h"

static const unsigned char ad9122_reg_defaults[][2] = {
	{AD9122_REG_COMM, 0x00},
	{AD9122_REG_COMM, AD9122_COMM_RESET},
	{AD9122_REG_COMM, 0x00},
	{AD9122_REG_POWER_CTRL, AD9122_POWER_CTRL_PD_AUX_ADC},
	{AD9122_REG_DATA_FORMAT, AD9122_DATA_FORMAT_BINARY},
	{AD9122_REG_INTERRUPT_EN_1, 0x00},
	{AD9122_REG_INTERRUPT_EN_2, 0x00},
	{AD9122_REG_CLK_REC_CTRL, AD9122_CLK_REC_CTRL_DACCLK_CROSS_CORRECTION |
				  AD9122_CLK_REC_CTRL_REFCLK_CROSS_CORRECTION |
				  0xF},
	{AD9122_REG_PLL_CTRL_1, AD9122_PLL_CTRL_1_PLL_MANUAL_EN},
	{AD9122_REG_PLL_CTRL_2, AD9122_PLL_CTRL_2_PLL_LOOP_BANDWIDTH(3) |
				AD9122_PLL_CTRL_2_PLL_CHARGE_PUMP_CURRENT(0x11)},
	{AD9122_REG_PLL_CTRL_3, AD9122_PLL_CTRL_3_N2(3) |
				AD9122_PLL_CTRL_3_PLL_CROSS_CTRL_EN |
				AD9122_PLL_CTRL_3_N0(1) |
				AD9122_PLL_CTRL_3_N1(2)},
	{AD9122_REG_SYNC_CTRL_1, AD9122_SYNC_CTRL_1_DATA_FIFO_RATE_TOGGLE |
				 AD9122_SYNC_CTRL_1_RISING_EDGE_SYNC},
	{AD9122_REG_SYNC_CTRL_2, 0x00},
	{AD9122_REG_DCI_DELAY, 0x00},
	{AD9122_REG_FIFO_CTRL, AD9122_FIFO_CTRL_FIFO_PHA_OFFSET(4)},
	{AD9122_REG_FIFO_STATUS_1, 0x00},
	{AD9122_REG_DATAPATH_CTRL, AD9122_DATAPATH_CTRL_BYPASS_PREMOD |
				   AD9122_DATAPATH_CTRL_BYPASS_INV_SINC |
				   AD9122_DATAPATH_CTRL_BYPASS_NCO},
	{AD9122_REG_HB1_CTRL, AD9122_HB1_CTRL_BYPASS_HB1},
	{AD9122_REG_HB2_CTRL, AD9122_HB2_CTRL_BYPASS_HB2},
	{AD9122_REG_HB3_CTRL, AD9122_HB3_CTRL_BYPASS_HB3},
	{AD9122_REG_FTW_7_0, 0x00},
	{AD9122_REG_FTW_15_8, 0x00},
	{AD9122_REG_FTW_23_16, 0x00},
	{AD9122_REG_FTW_31_24, 0x00},
	{AD9122_REG_NCO_PHA_OFFSET_LSB, 0x00},
	{AD9122_REG_NCO_PHA_OFFSET_MSB, 0x00},
	{AD9122_REG_NCO_FTW_UPDATE, 0x00},
	{AD9122_REG_I_PHA_ADJ_LSB, 0x00},
	{AD9122_REG_I_PHA_ADJ_MSB, 0x00},
	{AD9122_REG_Q_PHA_ADJ_LSB, 0x00},
	{AD9122_REG_Q_PHA_ADJ_MSB, 0x00},
	{AD9122_REG_I_DAC_OFFSET_LSB, 0x00},
	{AD9122_REG_I_DAC_OFFSET_MSB, 0x00},
	{AD9122_REG_Q_DAC_OFFSET_LSB, 0x00},
	{AD9122_REG_Q_DAC_OFFSET_MSB, 0x00},
	{AD9122_REG_I_DAC_FS_ADJ, 0xF9},
	{AD9122_REG_I_DAC_CTRL, 0x01},
	{AD9122_REG_I_AUX_DAC_DATA, 0x00},
	{AD9122_REG_I_AUX_DAC_CTRL, 0x00},
	{AD9122_REG_Q_DAC_FS_ADJ, 0xF9},
	{AD9122_REG_Q_DAC_CTRL, 0x01},
	{AD9122_REG_Q_AUX_DAC_DATA, 0x00},
	{AD9122_REG_Q_AUX_DAC_CTRL, 0x00},
	{AD9122_REG_DIE_TEMP_RANGE_CTRL, AD9122_DIE_TEMP_RANGE_CTRL_REF_CURRENT(1)},
	{AD9122_REG_FIFO_STATUS_1, AD9122_FIFO_STATUS_1_FIFO_SOFT_ALIGN_REQ},
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
	ret = spi_write_then_read(spi, buf, 2, NULL, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9122_setup(struct spi_device *spi, unsigned mode)
{
	int ret, timeout, i;

	for (i = 0; i < ARRAY_SIZE(ad9122_reg_defaults); i++)
			ad9122_write(spi, ad9122_reg_defaults[i][0],
					     ad9122_reg_defaults[i][1]);
	timeout = 255;
	do {
		mdelay(1);
		ret = ad9122_read(spi, AD9122_REG_FIFO_STATUS_1);
		if (ret < 0)
			return ret;

	} while (timeout-- && !(ret & AD9122_FIFO_STATUS_1_FIFO_SOFT_ALIGN_ACK));

	ad9122_write(spi, AD9122_REG_FIFO_STATUS_1, 0x0);
	ad9122_write(spi, AD9122_REG_SYNC_CTRL_1,
		     AD9122_SYNC_CTRL_1_SYNC_EN |
		     AD9122_SYNC_CTRL_1_RISING_EDGE_SYNC);

	timeout = 255;
	do {
		mdelay(1);
		ret = ad9122_read(spi, AD9122_REG_SYNC_STATUS_1);
		if (ret < 0)
			return ret;

	} while (timeout-- && !(ret & AD9122_SYNC_STATUS_1_SYNC_LOCKED));

	return 0;
}

static int __devinit ad9122_probe(struct spi_device *spi)
{
	struct cf_axi_dds_converter *conv;
	unsigned id;
	int ret;

	conv = kzalloc(sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;


	id = ad9122_read(spi, AD9122_REG_CHIP_ID);
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
	struct cf_axi_dds_converter *conv = spi_get_drvdata(spi);

	spi_set_drvdata(spi, NULL);
	kfree(conv);

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

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9122 ADC");
MODULE_LICENSE("GPL v2");
