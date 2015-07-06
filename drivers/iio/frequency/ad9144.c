/*
 * AD9144 SPI DAC driver for AXI DDS PCORE/COREFPGA Module
 *
 * Copyright 2014 Analog Devices Inc.
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
#include "ad9144.h"
#include "cf_axi_dds.h"

struct ad9144_platform_data {
	u8 xbar_lane0_sel;
	u8 xbar_lane1_sel;
	u8 xbar_lane2_sel;
	u8 xbar_lane3_sel;
};

static const char *clk_names[3] = { "jesd_dac_clk", "dac_clk", "dac_sysref" };

static int ad9144_read(struct spi_device *spi, unsigned reg)
{
	unsigned char buf[3];
	int ret;

	buf[0] = 0x80 | (reg >> 8);
	buf[1] = reg & 0xFF;

	ret = spi_write_then_read(spi, &buf[0], 2, &buf[2], 1);
	if (ret < 0)
		return ret;

	return buf[2];
}

static int ad9144_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	unsigned char buf[3];
	int ret;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xFF;
	buf[2] = val;
	ret = spi_write_then_read(spi, buf, 3, NULL, 0);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad9144_get_temperature_code(struct cf_axi_converter *conv)
{
	unsigned tmp;

	tmp = ad9144_read(conv->spi, REG_DIE_TEMP0) & 0xFF;
	tmp |= (ad9144_read(conv->spi, REG_DIE_TEMP1) & 0xFF) << 8;
	return tmp;
}

// static int ad9144_get_fifo_status(struct cf_axi_converter *conv)
// {
//      return 0;
// }

static int ad9144_setup(struct cf_axi_converter *conv, struct ad9144_platform_data *pdata)
{
	struct spi_device *spi = conv->spi;
	u8 val, i, timeout;

	// power-up and dac initialization

	mdelay(5);

	ad9144_write(spi, REG_SPI_INTFCONFA, SOFTRESET_M | SOFTRESET);	// reset
	ad9144_write(spi, REG_SPI_INTFCONFA, 0x00);	// reset

	mdelay(4);

	ad9144_write(spi, 0x011, 0x00);	// dacs - power up everything
	ad9144_write(spi, 0x080, 0x00);	// clocks - power up everything
	ad9144_write(spi, 0x081, 0x00);	// sysref - power up/falling edge

	// required device configurations

	ad9144_write(spi, 0x12d, 0x8b);	// data-path
	ad9144_write(spi, 0x146, 0x01);	// data-path
	ad9144_write(spi, 0x2a4, 0xff);	// clock
	ad9144_write(spi, 0x1c4, 0x73);	// dac-pll
	ad9144_write(spi, 0x291, 0x49);	// serde-pll
	ad9144_write(spi, 0x29c, 0x24);	// serde-pll
	ad9144_write(spi, 0x29f, 0x73);	// serde-pll
	ad9144_write(spi, 0x232, 0xff);	// jesd
	ad9144_write(spi, 0x333, 0x01);	// jesd

	// digital data path

	ad9144_write(spi, 0x112, 0x00);	// interpolation (bypass)
	ad9144_write(spi, 0x110, 0x00);	// 2's complement

	// transport layer

	ad9144_write(spi, 0x200, 0x00);	// phy - power up
	ad9144_write(spi, 0x201, 0x00);	// phy - power up
	ad9144_write(spi, 0x300, 0x01);	// single link - link 0
	ad9144_write(spi, 0x450, 0x00);	// device id (0x400)
	ad9144_write(spi, 0x451, 0x00);	// bank id (0x401)
	ad9144_write(spi, 0x452, 0x04);	// lane-id (0x402)
	ad9144_write(spi, 0x453, 0x83);	// descrambling, 4 lanes
	ad9144_write(spi, 0x454, 0x00);	// octects per frame per lane (1)
	ad9144_write(spi, 0x455, 0x1f);	// mult-frame - framecount (32)
	ad9144_write(spi, 0x456, 0x01);	// no-of-converters (2)
	ad9144_write(spi, 0x457, 0x0f);	// no CS bits, 16bit dac
	ad9144_write(spi, 0x458, 0x2f);	// subclass 1, 16bits per sample
	ad9144_write(spi, 0x459, 0x20);	// jesd204b, 1 samples per converter per device
	ad9144_write(spi, 0x45a, 0x80);	// HD mode, no CS bits
	ad9144_write(spi, 0x45d, 0x49);	// check-sum of 0x450 to 0x45c
	ad9144_write(spi, 0x46c, 0x0f);	// enable deskew for all lanes
	ad9144_write(spi, 0x476, 0x01);	// frame - bytecount (1)
	ad9144_write(spi, 0x47d, 0x0f);	// enable all lanes

	// physical layer

	ad9144_write(spi, 0x2aa, 0xb7);	// jesd termination
	ad9144_write(spi, 0x2ab, 0x87);	// jesd termination
	ad9144_write(spi, 0x2b1, 0xb7);	// jesd termination
	ad9144_write(spi, 0x2b2, 0x87);	// jesd termination
	ad9144_write(spi, 0x2a7, 0x01);	// input termination calibration
	ad9144_write(spi, 0x2ae, 0x01);	// input termination calibration
	ad9144_write(spi, 0x314, 0x01);	// pclk == qbd master clock
	ad9144_write(spi, 0x230, 0x28);	// cdr mode - halfrate, no division
	ad9144_write(spi, 0x206, 0x00);	// cdr reset
	ad9144_write(spi, 0x206, 0x01);	// cdr reset
	ad9144_write(spi, 0x289, 0x04);	// data-rate == 10Gbps
	ad9144_write(spi, 0x280, 0x01);	// enable serdes pll
	ad9144_write(spi, 0x280, 0x05);	// enable serdes calibration
	mdelay(20);

	if ((ad9144_read(spi, 0x281) & 0x01) == 0x00) {
		dev_err(&spi->dev, "AD9144, PLL/link errors!!\n\r");
	}

	ad9144_write(spi, 0x268, 0x62);	// equalizer

	// cross-bar

	ad9144_write(spi, REG_XBAR_LN_0_1, SRC_LANE0(pdata->xbar_lane0_sel) |
		SRC_LANE1(pdata->xbar_lane1_sel));	// lane selects
	ad9144_write(spi, REG_XBAR_LN_2_3, SRC_LANE2(pdata->xbar_lane2_sel) |
		SRC_LANE3(pdata->xbar_lane3_sel));	// lane selects

	// data link layer

	ad9144_write(spi, 0x301, 0x01);	// subclass-1
	ad9144_write(spi, 0x304, 0x00);	// lmfc delay
	ad9144_write(spi, 0x305, 0x00);	// lmfc delay
	ad9144_write(spi, 0x306, 0x0a);	// receive buffer delay
	ad9144_write(spi, 0x307, 0x0a);	// receive buffer delay
	ad9144_write(spi, 0x03a, 0x01);	// sync-oneshot mode
	ad9144_write(spi, 0x03a, 0x81);	// sync-enable
	ad9144_write(spi, 0x03a, 0xc1);	// sysref-armed
	ad9144_write(spi, 0x300, 0x01);	// enable link

	// dac calibration

	ad9144_write(spi, 0x0e7, 0x38);	// set calibration clock to 1m
	ad9144_write(spi, 0x0ed, 0xa6);	// use isb reference of 38 to set cal
	ad9144_write(spi, 0x0e8, 0x03);	// cal 2 dacs at once
	ad9144_write(spi, 0x0e9, 0x01);	// single cal enable
	ad9144_write(spi, 0x0e9, 0x03);	// single cal start
	mdelay(10);

	for (i = 0, timeout = 30; i < 2; i++) {
		ad9144_write(spi, 0x0e8, 1 << i); // read dac-x
		do {
			mdelay(1);
			val = ad9144_read(spi, 0x0e9);
		} while ((val & CAL_ACTIVE) && timeout--);

		if ((val & (CAL_FIN | CAL_ERRHI | CAL_ERRLO)) != CAL_FIN) {
			dev_err(&spi->dev, "AD9144, dac-%d calibration failed (0x%X)!!\n", i, val);
		}
	}

	ad9144_write(spi, 0x0e7, 0x30);	// turn off cal clock

	return 0;
}

static int ad9144_get_clks(struct cf_axi_converter *conv)
{
	struct clk *clk;
	int i, ret;

	for (i = 0; i < 3; i++) {
		clk = clk_get(&conv->spi->dev, &clk_names[i][0]);
		if (IS_ERR(clk)) {
			return -EPROBE_DEFER;
		}

		if (i > 0) {
			ret = clk_prepare_enable(clk);
			if (ret < 0)
				return ret;
		}
		conv->clk[i] = clk;
	}
	return 0;
}

static unsigned long ad9144_get_data_clk(struct cf_axi_converter *conv)
{
	return clk_get_rate(conv->clk[CLK_DAC]);
}

static int ad9144_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned tmp;

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:

		*val = ad9144_get_data_clk(conv);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		*val = conv->temp_calib_code;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PROCESSED:
		if (!conv->temp_calib_code)
			return -EINVAL;

		tmp = ad9144_get_temperature_code(conv);

		*val = ((tmp - conv->temp_calib_code) * 77
			+ conv->temp_calib * 10 + 10000) / 10;

		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad9144_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBBIAS:
		conv->temp_calib_code = val;
		break;
	case IIO_CHAN_INFO_PROCESSED:
		/*
		 * Writing in_temp0_input with the device temperature in milli
		 * degrees Celsius triggers the calibration.
		 */
		conv->temp_calib_code = ad9144_get_temperature_code(conv);
		conv->temp_calib = val;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int ad9144_prepare(struct cf_axi_converter *conv)
{
	return 0;
}

#ifdef CONFIG_OF
static struct ad9144_platform_data *ad9144_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct ad9144_platform_data *pdata;
	unsigned int tmp;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "could not allocate memory for platform data\n");
		return NULL;
	}

	tmp = 0;
	of_property_read_u32(np, "adi,jesd-xbar-lane0-sel", &tmp);
	pdata->xbar_lane0_sel = tmp;

	tmp = 1;
	of_property_read_u32(np, "adi,jesd-xbar-lane1-sel", &tmp);
	pdata->xbar_lane1_sel = tmp;

	tmp = 2;
	of_property_read_u32(np, "adi,jesd-xbar-lane2-sel", &tmp);
	pdata->xbar_lane2_sel = tmp;

	tmp = 3;
	of_property_read_u32(np, "adi,jesd-xbar-lane3-sel", &tmp);
	pdata->xbar_lane3_sel = tmp;

	return pdata;
}
#else
static
struct ad9144_platform_data *ad9144_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static int ad9144_probe(struct spi_device *spi)
{
	struct cf_axi_converter *conv;
	struct ad9144_platform_data *pdata;
	unsigned id;
	int ret;

	if (spi->dev.of_node)
		pdata = ad9144_parse_dt(&spi->dev);
	else
		pdata = spi->dev.platform_data;

	if (!pdata) {
		dev_err(&spi->dev, "no platform data?\n");
		return -EINVAL;
	}

	conv = devm_kzalloc(&spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->reset_gpio = devm_gpiod_get(&spi->dev, "reset");
	if (!IS_ERR(conv->reset_gpio)) {
		ret = gpiod_direction_output(conv->reset_gpio, 1);
	}

	conv->txen_gpio = devm_gpiod_get(&spi->dev, "txen");
	if (!IS_ERR(conv->txen_gpio)) {
		ret = gpiod_direction_output(conv->txen_gpio, 1);
	}

	id = ad9144_read(spi, REG_SPI_PRODIDL);
	if (id != CHIPID_AD9144) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		ret = -ENODEV;
		goto out;
	}

	ad9144_write(spi, REG_SPI_SCRATCHPAD, 0xAD);
	if (ad9144_read(spi, REG_SPI_SCRATCHPAD) != 0xAD)
		return -EIO;

	conv->write = ad9144_write;
	conv->read = ad9144_read;
	conv->setup = ad9144_prepare;

	conv->get_data_clk = ad9144_get_data_clk;
	conv->write_raw = ad9144_write_raw;
	conv->read_raw = ad9144_read_raw;
	conv->spi = spi;
	conv->id = ID_AD9144;

	ret = ad9144_get_clks(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	ret = ad9144_setup(conv, pdata);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	clk_prepare_enable(conv->clk[0]);
	spi_set_drvdata(spi, conv);

	return 0;
out:
	return ret;
}

static int ad9144_remove(struct spi_device *spi)
{
	spi_set_drvdata(spi, NULL);
	return 0;
}

static const struct spi_device_id ad9144_id[] = {
	{"ad9144", 9144},
	{}
};

MODULE_DEVICE_TABLE(spi, ad9144_id);

static struct spi_driver ad9144_driver = {
	.driver = {
		   .name = "ad9144",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9144_probe,
	.remove = ad9144_remove,
	.id_table = ad9144_id,
};

module_spi_driver(ad9144_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9144 DAC");
MODULE_LICENSE("GPL v2");
