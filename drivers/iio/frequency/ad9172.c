/*
 * Licensed under the GPL-2.
 *
 * NOT A DRIVER. DO NOT USE.
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
#include "cf_axi_dds.h"

enum chip_id {
	CHIPID_AD9172 = 0x72,
};

struct ad9172_state {
	struct cf_axi_converter conv;
	enum chip_id id;
	struct regmap *map;
	unsigned int interpolation;
};

static const char * const clk_names[] = {
	[CLK_DATA] = "jesd_dac_clk",
	[CLK_DAC] = "dac_clk",
	[CLK_REF] = "dac_sysref"
};

static int ad9172_read(struct spi_device *spi, unsigned reg)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);
	unsigned int val;
	int ret = regmap_read(st->map, reg, &val);

	return ret < 0 ? ret : val;
}

static int ad9172_write(struct spi_device *spi, unsigned reg, unsigned val)
{
	struct cf_axi_converter *conv = spi_get_drvdata(spi);
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);

	return regmap_write(st->map, reg, val);
}

static int ad9172_get_clks(struct cf_axi_converter *conv)
{
	struct clk *clk;
	int i, ret;

	for (i = 0; i < 3; i++) {
		clk = devm_clk_get(&conv->spi->dev, clk_names[i]);
		if (IS_ERR(clk))
			return PTR_ERR(clk);

		if (i > 0) {
			ret = clk_prepare_enable(clk);
			if (ret < 0)
				return ret;
		}
		conv->clk[i] = clk;
	}
	return 0;
}

static unsigned long long ad9172_get_data_clk(struct cf_axi_converter *conv)
{
	struct ad9172_state *st = container_of(conv, struct ad9172_state, conv);

	return clk_get_rate(conv->clk[CLK_DAC]) / st->interpolation;
}

static int ad9172_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long m)
{
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);

	switch (m) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = ad9172_get_data_clk(conv);
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int ad9172_prepare(struct cf_axi_converter *conv)
{
	return 0;
}

static const struct regmap_config ad9172_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
	/* TODO: Add volatile/writeable registers tables */
	.cache_type = REGCACHE_NONE,
};

#define REG_SPI_INTFCONFA                        0x000 /* Interface configuration A */
#define REG_SPI_INTFCONFB                        0x001 /* Interface configuration B */
#define REG_SPI_DEVCONF                          0x002 /* Device Configuration */
#define REG_SPI_PRODIDL                          0x004 /* Product Identification Low Byte */
#define REG_SPI_PRODIDH                          0x005 /* Product Identification High Byte */
#define REG_SPI_CHIPGRADE                        0x006 /* Chip Grade */
#define REG_SPI_PAGEINDX                         0x008 /* Page Pointer or Device Index */
#define REG_SPI_DEVINDX2                         0x009 /* Secondary Device Index */
#define REG_SPI_SCRATCHPAD                       0x00A /* Scratch Pad */

static int ad9172_probe(struct spi_device *spi)
{
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct cf_axi_converter *conv;
	struct ad9172_state *st;
	unsigned long lane_rate_kHz;
	unsigned int val;
	unsigned int id;
	int ret;

	st = devm_kzalloc(&spi->dev, sizeof(*st), GFP_KERNEL);
	if (st == NULL)
		return -ENOMEM;

	st->id = (enum chip_id)dev_id->driver_data;
	conv = &st->conv;

	st->map = devm_regmap_init_spi(spi, &ad9172_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	conv->write = ad9172_write;
	conv->read = ad9172_read;
	conv->setup = ad9172_prepare;

	conv->get_data_clk = ad9172_get_data_clk;
	conv->read_raw = ad9172_read_raw;
	conv->spi = spi;
	conv->id = ID_AD9172;

	ret = ad9172_get_clks(conv);
	if (ret < 0) {
		if (ret != -EPROBE_DEFER)
			dev_err(&spi->dev, "Failed to get clocks\n");
		return ret;
	}

	regmap_write(st->map, 0x000, 0x81);
	regmap_write(st->map, 0x000, 0x3c);
	regmap_write(st->map, 0x705, 0x01);
	regmap_write(st->map, 0x091, 0x00);
	regmap_write(st->map, 0x206, 0x01);
	regmap_write(st->map, 0x705, 0x01);
	regmap_write(st->map, 0x090, 0x00);
	regmap_write(st->map, 0x253, 0x01);
	regmap_write(st->map, 0x254, 0x01);

	regmap_write(st->map, 0x095, 0x01);
	regmap_write(st->map, 0x790, 0x0f);
	regmap_write(st->map, 0x791, 0x1f);

	mdelay(1);
	ret = regmap_read(st->map, REG_SPI_PRODIDL, &id);
	if (ret < 0)
		return ret;

	if (id != st->id) {
		dev_err(&spi->dev, "Unrecognized CHIP_ID 0x%X\n", id);
		return -ENODEV;
	}

	regmap_write(st->map, REG_SPI_SCRATCHPAD, 0xAD);
	regmap_read(st->map, REG_SPI_SCRATCHPAD, &id);
	if (id != 0xAD)
		return -EIO;

	regmap_write(st->map, 0xc0, 0x01);
	regmap_write(st->map, 0xdb, 0x00);
	regmap_write(st->map, 0xdb, 0x01);
	regmap_write(st->map, 0xdb, 0x00);
	regmap_write(st->map, 0xc1, 0x48);
	regmap_write(st->map, 0xc1, 0x49);
	regmap_write(st->map, 0xc7, 0x01);

	regmap_write(st->map, 0x50, 0x2a);
	regmap_write(st->map, 0x61, 0x68);
	regmap_write(st->map, 0x51, 0x82);
	regmap_write(st->map, 0x51, 0x83);
	regmap_write(st->map, 0x81, 0x03);

	regmap_write(st->map, 0x100, 0x00);
	regmap_write(st->map, 0x110, 0x0a);
	regmap_write(st->map, 0x111, 0x11);
	regmap_write(st->map, 0x084, 0x00);
	regmap_write(st->map, 0x300, 0x01);
	regmap_write(st->map, 0x475, 0x09);
	regmap_write(st->map, 0x453, 0x87);
	regmap_write(st->map, 0x457, 0x0f);
	regmap_write(st->map, 0x458, 0x2f);
	regmap_write(st->map, 0x459, 0x01);
	regmap_write(st->map, 0x475, 0x01);
	regmap_write(st->map, 0x300, 0x00);

	regmap_write(st->map, 0x240, 0xaa);
	regmap_write(st->map, 0x241, 0xaa);
	regmap_write(st->map, 0x242, 0x55);
	regmap_write(st->map, 0x243, 0x55);
	regmap_write(st->map, 0x244, 0x1f);
	regmap_write(st->map, 0x245, 0x1f);
	regmap_write(st->map, 0x246, 0x1f);
	regmap_write(st->map, 0x247, 0x1f);
	regmap_write(st->map, 0x248, 0x1f);
	regmap_write(st->map, 0x249, 0x1f);
	regmap_write(st->map, 0x24a, 0x1f);
	regmap_write(st->map, 0x24b, 0x1f);

	regmap_write(st->map, 0x201, 0x00);
	regmap_write(st->map, 0x203, 0x01);

	regmap_write(st->map, 0x210, 0x16);
	regmap_write(st->map, 0x216, 0x05);
	regmap_write(st->map, 0x212, 0xff);
	regmap_write(st->map, 0x212, 0x00);
	regmap_write(st->map, 0x210, 0x87);
	regmap_write(st->map, 0x216, 0x11);
	regmap_write(st->map, 0x213, 0x01);
	regmap_write(st->map, 0x213, 0x00);
	regmap_write(st->map, 0x200, 0x00);
	msleep(100);
	regmap_write(st->map, 0x210, 0x86);
	regmap_write(st->map, 0x216, 0x40);
	regmap_write(st->map, 0x213, 0x01);
	regmap_write(st->map, 0x213, 0x00);
	regmap_write(st->map, 0x210, 0x86);
	regmap_write(st->map, 0x216, 0x00);
	regmap_write(st->map, 0x213, 0x01);
	regmap_write(st->map, 0x213, 0x00);
	regmap_write(st->map, 0x210, 0x86);
	regmap_write(st->map, 0x216, 0x01);
	regmap_write(st->map, 0x213, 0x01);
	regmap_write(st->map, 0x213, 0x00);
	regmap_write(st->map, 0x210, 0x87);
	regmap_write(st->map, 0x216, 0x01);
	regmap_write(st->map, 0x213, 0x01);
	regmap_write(st->map, 0x213, 0x00);
	regmap_write(st->map, 0x280, 0x05);
	regmap_write(st->map, 0x280, 0x01);
	regmap_write(st->map, 0x206, 0x01);

	regmap_read(st->map, 0x281, &val);

	dev_err(&spi->dev, "SERDES locked: %d\n", val & 1);

	regmap_write(st->map, 0x306, 0x0c);
	regmap_write(st->map, 0x307, 0x0c);
	regmap_write(st->map, 0x03b, 0xf1);
	regmap_write(st->map, 0x03a, 0x02);
	msleep(10);
	regmap_write(st->map, 0x300, 0x01);

	regmap_write(st->map, 0x008, 0xc0);
	regmap_write(st->map, 0x596, 0x1c);

	st->interpolation = 1;

	lane_rate_kHz = clk_get_rate(st->conv.clk[1]) / st->interpolation;
	lane_rate_kHz = DIV_ROUND_CLOSEST(lane_rate_kHz, 200);
	ret = clk_set_rate(conv->clk[0], lane_rate_kHz);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to set lane rate to %ld kHz: %d\n",
			lane_rate_kHz, ret);
		return ret;
	}

	ret = clk_prepare_enable(conv->clk[0]);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to enable JESD204 link: %d\n", ret);
		return ret;
	}

	spi_set_drvdata(spi, conv);

	dev_dbg(&spi->dev, "Probed.\n");
	return 0;
}

static const struct spi_device_id ad9172_id[] = {
	{ "ad9172", CHIPID_AD9172 },
	{}
};
MODULE_DEVICE_TABLE(spi, ad9172_id);

static struct spi_driver ad9172_driver = {
	.driver = {
		   .name = "ad9172",
		   .owner = THIS_MODULE,
		   },
	.probe = ad9172_probe,
	.id_table = ad9172_id,
};
module_spi_driver(ad9172_driver);

MODULE_LICENSE("GPL v2");
