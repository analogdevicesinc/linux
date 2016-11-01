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
#include <linux/regmap.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include "ad9144.h"
#include "cf_axi_dds.h"

enum chip_id {
	CHIPID_AD9144 = 0x44,
	CHIPID_AD9152 = 0x52,
};

struct ad9144_platform_data {
	u8 xbar_lane0_sel;
	u8 xbar_lane1_sel;
	u8 xbar_lane2_sel;
	u8 xbar_lane3_sel;
	bool lanes2_3_swap_data;
};

struct ad9144_state {
	struct cf_axi_converter conv;
	enum chip_id id;
	struct regmap *map;
};

static const char *clk_names[3] = { "jesd_dac_clk", "dac_clk", "dac_sysref" };

static int ad9144_read(struct spi_device *spi, unsigned reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);
	unsigned int val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9144_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);

	return regmap_write(st->map, reg, val);
}

static int ad9144_get_temperature_code(struct cf_axi_converter *conv)
{
	struct ad9144_state *st = container_of(conv, struct ad9144_state, conv);
	unsigned val1, val2;

	regmap_read(st->map, REG_DIE_TEMP0, &val1);
	regmap_read(st->map, REG_DIE_TEMP1, &val2);
	return ((val2 & 0xFF) << 8) | (val1 & 0xFF);
}

// static int ad9144_get_fifo_status(struct cf_axi_converter *conv)
// {
//      return 0;
// }

static int ad9144_setup(struct ad9144_state *st,
		struct ad9144_platform_data *pdata)
{
	struct regmap *map = st->map;
	struct device *dev = regmap_get_device(map);
	unsigned int val;
	u8 i, timeout;
	unsigned long lane_rate_kHz;

	lane_rate_kHz = clk_get_rate(st->conv.clk[1]);
	lane_rate_kHz = (lane_rate_kHz / 1000) * 10;	// FIXME for other configurations

	// power-up and dac initialization

	msleep(5);

	regmap_write(map, REG_SPI_INTFCONFA, SOFTRESET_M | SOFTRESET);	// reset
	regmap_write(map, REG_SPI_INTFCONFA, 0x00);	// reset

	msleep(4);

	regmap_write(map, 0x011, 0x00);	// dacs - power up everything
	regmap_write(map, 0x080, 0x00);	// clocks - power up everything
	regmap_write(map, 0x081, 0x00);	// sysref - power up/falling edge

	if (st->id == CHIPID_AD9144) {
		// required device configurations

		regmap_write(map, 0x12d, 0x8b);	// data-path
		regmap_write(map, 0x146, 0x01);	// data-path
		regmap_write(map, 0x2a4, 0xff);	// clock
		regmap_write(map, 0x1c4, 0x73);	// dac-pll
		regmap_write(map, 0x291, 0x49);	// serde-pll
		regmap_write(map, 0x29c, 0x24);	// serde-pll
		regmap_write(map, 0x29f, 0x73);	// serde-pll
		regmap_write(map, 0x232, 0xff);	// jesd
		regmap_write(map, 0x333, 0x01);	// jesd

		/* Write optimal settings for the SERDES PLL, as per table 39 of the
		 * datasheet. */
		regmap_write(map, 0x284, 0x62);
		regmap_write(map, 0x285, 0xc9);
		regmap_write(map, 0x286, 0x0e);
		regmap_write(map, 0x287, 0x12);
		regmap_write(map, 0x28a, 0x7b);
		regmap_write(map, 0x28b, 0x00);
		regmap_write(map, 0x290, 0x89);
		regmap_write(map, 0x294, 0x24);
		regmap_write(map, 0x296, 0x03);
		regmap_write(map, 0x297, 0x0d);
		regmap_write(map, 0x299, 0x02);
		regmap_write(map, 0x29a, 0x8e);
		regmap_write(map, 0x29c, 0x2a);
		regmap_write(map, 0x29f, 0x78);
		regmap_write(map, 0x2a0, 0x06);
	}

	// digital data path

	regmap_write(map, 0x112, 0x00);	// interpolation (bypass)
	regmap_write(map, 0x110, 0x00);	// 2's complement

	// transport layer

	regmap_write(map, 0x200, 0x00);	// phy - power up
	regmap_write(map, 0x201, 0x00);	// phy - power up
	if (st->id == CHIPID_AD9152) {
		regmap_write(map, 0x230, 0x28); // half-rate CDR
		regmap_write(map, 0x312, 0x20); // half-rate CDR
	}
	regmap_write(map, 0x300, 0x00);	// single link - link 0
	regmap_write(map, 0x450, 0x00);	// device id (0x400)
	regmap_write(map, 0x451, 0x00);	// bank id (0x401)
	regmap_write(map, 0x452, 0x04);	// lane-id (0x402)
	regmap_write(map, 0x453, 0x83);	// descrambling, 4 lanes
	regmap_write(map, 0x454, 0x00);	// octects per frame per lane (1)
	regmap_write(map, 0x455, 0x1f);	// mult-frame - framecount (32)
	regmap_write(map, 0x456, 0x01);	// no-of-converters (2)
	regmap_write(map, 0x457, 0x0f);	// no CS bits, 16bit dac
	regmap_write(map, 0x458, 0x2f);	// subclass 1, 16bits per sample
	regmap_write(map, 0x459, 0x20);	// jesd204b, 1 samples per converter per device
	regmap_write(map, 0x45a, 0x80);	// HD mode, no CS bits
	regmap_write(map, 0x45d, 0x49);	// check-sum of 0x450 to 0x45c
	regmap_write(map, 0x478, 0x01);	// ilas mf count
	regmap_write(map, 0x46c, 0x0f);	// enable deskew for all lanes
	regmap_write(map, 0x476, 0x01);	// frame - bytecount (1)
	regmap_write(map, 0x47d, 0x0f);	// enable all lanes

	// physical layer

	regmap_write(map, 0x2aa, 0xb7);	// jesd termination
	regmap_write(map, 0x2ab, 0x87);	// jesd termination
	if (st->id == CHIPID_AD9144) {
		regmap_write(map, 0x2b1, 0xb7);	// jesd termination
		regmap_write(map, 0x2b2, 0x87);	// jesd termination
	}
	regmap_write(map, 0x2a7, 0x01);	// input termination calibration
	if (st->id == CHIPID_AD9144)
		regmap_write(map, 0x2ae, 0x01);	// input termination calibration
	regmap_write(map, 0x314, 0x01);	// pclk == qbd master clock
	if (lane_rate_kHz < 2880000)
		regmap_write(map, 0x230, 0x0A);			// CDR_OVERSAMP
	else
		if (lane_rate_kHz > 5520000)
			regmap_write(map, 0x230, 0x28);		// ENHALFRATE
		else
			regmap_write(map, 0x230, 0x08);
	regmap_write(map, 0x206, 0x00);	// cdr reset
	regmap_write(map, 0x206, 0x01);	// cdr reset
	if (lane_rate_kHz < 2880000)
		regmap_write(map, 0x289, 0x06);	// data-rate < 2.88 Gbps
	else
		if (lane_rate_kHz > 5520000)
			regmap_write(map, 0x289, 0x04);	// data-rate > 5.52 Gbps
		else
			regmap_write(map, 0x289, 0x05);
	regmap_write(map, 0x280, 0x01);	// enable serdes pll
	regmap_write(map, 0x280, 0x05);	// enable serdes calibration
	msleep(20);

	regmap_read(map, 0x281, &val);
	if ((val & 0x01) == 0x00)
		dev_err(dev, "PLL/link errors!!\n\r");

	regmap_write(map, 0x268, 0x62);	// equalizer

	// cross-bar

	regmap_write(map, REG_XBAR_LN_0_1, SRC_LANE0(pdata->xbar_lane0_sel) |
		SRC_LANE1(pdata->xbar_lane1_sel));	// lane selects
	regmap_write(map, REG_XBAR_LN_2_3, SRC_LANE2(pdata->xbar_lane2_sel) |
		SRC_LANE3(pdata->xbar_lane3_sel));	// lane selects

	// data link layer

	regmap_write(map, 0x301, 0x01);	// subclass-1
	regmap_write(map, 0x304, 0x00);	// lmfc delay
	if (st->id == CHIPID_AD9144)
		regmap_write(map, 0x305, 0x00);	// lmfc delay
	regmap_write(map, 0x306, 0x0a);	// receive buffer delay
	if (st->id == CHIPID_AD9144)
		regmap_write(map, 0x307, 0x0a);	// receive buffer delay
	regmap_write(map, 0x03a, 0x01);	// sync-oneshot mode
	regmap_write(map, 0x03a, 0x81);	// sync-enable
	regmap_write(map, 0x03a, 0xc1);	// sysref-armed
	regmap_write(map, 0x300, 0x01);	// enable link

	if (st->id == CHIPID_AD9144) {
		// dac calibration

		regmap_write(map, 0x0e7, 0x38);	// set calibration clock to 1m
		regmap_write(map, 0x0ed, 0xa6);	// use isb reference of 38 to set cal
		regmap_write(map, 0x0e8, 0x03);	// cal 2 dacs at once
		regmap_write(map, 0x0e9, 0x01);	// single cal enable
		regmap_write(map, 0x0e9, 0x03);	// single cal start
		msleep(10);

		for (i = 0, timeout = 30; i < 2; i++) {
			regmap_write(map, 0x0e8, 1 << i); // read dac-x
			do {
				msleep(1);
				regmap_read(map, 0x0e9, &val);
			} while ((val & CAL_ACTIVE) && timeout--);

			if ((val & (CAL_FIN | CAL_ERRHI | CAL_ERRLO)) != CAL_FIN) {
				dev_err(dev, "AD9144, dac-%d calibration failed (0x%X)!!\n", i, val);
			}
		}
	}

	regmap_write(map, 0x0e7, 0x30);	// turn off cal clock

	/* TODO: remove me
	 * Fix for an early DAQ3 design bug (swapped SERDIN+ / SERDIN- pins) */
	if (st->id == CHIPID_AD9152)
		regmap_write(map, 0x334,
				pdata->lanes2_3_swap_data ? 0x0c : 0x00);

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

static unsigned long long ad9144_get_data_clk(struct cf_axi_converter *conv)
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

	pdata->lanes2_3_swap_data = of_property_read_bool(np,
			"adi,lanes2-3-swap-data");

	return pdata;
}
#else
static
struct ad9144_platform_data *ad9144_parse_dt(struct device *dev)
{
	return NULL;
}
#endif

static const struct regmap_config ad9144_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	/* TODO: Add volatile/writeable registers tables */
	.cache_type = REGCACHE_NONE,
};

static int ad9144_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct cf_axi_converter *conv;
	struct ad9144_platform_data *pdata;
	struct ad9144_state *st;
	unsigned long lane_rate_kHz;
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

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->id = (enum chip_id) dev_id->driver_data;
	conv = &st->conv;

	conv->reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_HIGH);

	conv->txen_gpio = devm_gpiod_get(&spi->dev, "txen", GPIOD_OUT_HIGH);

	st->map = devm_regmap_init_spi(spi, &ad9144_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	ret = regmap_read(st->map, REG_SPI_PRODIDL, &id);
	if (ret < 0)
		return ret;

	if (id != st->id) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		ret = -ENODEV;
		goto out;
	}

	regmap_write(st->map, REG_SPI_SCRATCHPAD, 0xAD);
	regmap_read(st->map, REG_SPI_SCRATCHPAD, &id);
	if (id != 0xAD)
		return -EIO;

	conv->write = ad9144_write;
	conv->read = ad9144_read;
	conv->setup = ad9144_prepare;

	conv->get_data_clk = ad9144_get_data_clk;
	conv->write_raw = ad9144_write_raw;
	conv->read_raw = ad9144_read_raw;
	conv->spi = spi;
	conv->id = st->id == CHIPID_AD9144 ? ID_AD9144 : ID_AD9152;

	ret = ad9144_get_clks(conv);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to get clocks\n");
		goto out;
	}

	ret = ad9144_setup(st, pdata);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to setup device\n");
		goto out;
	}

	clk_prepare_enable(conv->clk[0]);

	lane_rate_kHz = clk_get_rate(st->conv.clk[1]);
	lane_rate_kHz = (lane_rate_kHz / 1000) * 10;	// FIXME for other configurations
	clk_set_rate(conv->clk[0], lane_rate_kHz);

	spi_set_drvdata(spi, conv);

	dev_info(&spi->dev, "Probed.\n");
	return 0;
out:
	return ret;
}

static const struct spi_device_id ad9144_id[] = {
	{ "ad9144", CHIPID_AD9144 },
	{ "ad9152", CHIPID_AD9152 },
	{}
};

MODULE_DEVICE_TABLE(spi, ad9144_id);

static struct spi_driver ad9144_driver = {
	.driver = {
		   .name = "ad9144",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9144_probe,
	.id_table = ad9144_id,
};

module_spi_driver(ad9144_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9144 DAC");
MODULE_LICENSE("GPL v2");
