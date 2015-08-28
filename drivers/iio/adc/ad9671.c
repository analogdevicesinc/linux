/*
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>

#define AD9671_CHANNEL(_ch) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _ch, \
	.modified = 0, \
	.channel2 = 0, \
	.address = 0, \
	.scan_index = _ch, \
	.scan_type = { \
		.sign = 's', \
		.realbits = 16, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec ad9671_channels[] = {
	AD9671_CHANNEL(0),
	AD9671_CHANNEL(1),
	AD9671_CHANNEL(2),
	AD9671_CHANNEL(3),
	AD9671_CHANNEL(4),
	AD9671_CHANNEL(5),
	AD9671_CHANNEL(6),
	AD9671_CHANNEL(7),
};

struct ad9671_state {
	struct spi_device *spi;
	struct clk *ref_clk;
	struct regmap *regmap;
	struct gpio_desc *tx_trig;
};

static int ad9671_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct ad9671_state *st = iio_priv(indio_dev);

	if (readval == NULL)
		return regmap_write(st->regmap, reg, writeval);
	else
		return regmap_read(st->regmap, reg, readval);
}

static const struct iio_info ad9671_info = {
	.debugfs_reg_access = &ad9671_reg_access,
	.driver_module = THIS_MODULE,
};

static int ad9671_setup(struct ad9671_state *st, unsigned int clk_rate)
{
	unsigned int timeout = 10;
	unsigned int val;
	int ret;

	regmap_write(st->regmap, 0x000, 0x3c); // software reset

	mdelay(10);

	regmap_write(st->regmap, 0xf00, 0xff); // profile (cont. run)
	regmap_write(st->regmap, 0xf01, 0x7f); // profile (cont. run)
	regmap_write(st->regmap, 0xf02, 0x00); // profile (digital high-pass)
	regmap_write(st->regmap, 0xf03, 0x80); // profile (power up channels)
	regmap_write(st->regmap, 0xf04, 0x0c); // profile (coeff. blk 0)
	regmap_write(st->regmap, 0xf05, 0x00); // profile (decimate by 4, gain 16 == 001c / decimate by 2, gain 16 == 000c ) 
	regmap_write(st->regmap, 0xf06, 0x00); // profile (demod. freq)
	regmap_write(st->regmap, 0xf07, 0x20); // profile (demod. freq)

	if (clk_rate == 80000000)
		regmap_write(st->regmap, 0x002, 0x22); // speed mode 80 MHz
	else
		regmap_write(st->regmap, 0x002, 0x02); // speed mode 40 MHz

	regmap_write(st->regmap, 0x0ff, 0x01); // apply speed mode change

	if (clk_rate == 80000000) {
		regmap_write(st->regmap, 0x113, 0x27); // decimator and demodulator bypass
		regmap_write(st->regmap, 0x181, 0x02); // PLL divider
	} else {
		regmap_write(st->regmap, 0x113, 0x23); // rf decimator, decimator and demodulator bypass
		regmap_write(st->regmap, 0x181, 0x03); // PLL divider
	}

	regmap_write(st->regmap, 0x011, 0x06);
	regmap_write(st->regmap, 0x10c, 0x00); // profile index
	regmap_write(st->regmap, 0x014, 0x00);
	regmap_write(st->regmap, 0x008, 0x00);
	regmap_write(st->regmap, 0x021, 0x21);
	regmap_write(st->regmap, 0x199, 0x80);
	regmap_write(st->regmap, 0x142, 0x14); // ILAS enabled
	regmap_write(st->regmap, 0x188, 0x01);
	regmap_write(st->regmap, 0x18B, 0x27);
	regmap_write(st->regmap, 0x18C, 0x72);
	regmap_write(st->regmap, 0x150, 0x81); // scrambler enabled, 2 lanes per link (4lane - 0x83)
	regmap_write(st->regmap, 0x182, 0x8F);
	regmap_write(st->regmap, 0x10c, 0x20);
	regmap_write(st->regmap, 0x00f, 0x30);
	regmap_write(st->regmap, 0x02b, 0x40);
	regmap_write(st->regmap, 0x004, 0x0f); // select all channels
	regmap_write(st->regmap, 0x005, 0x0f); // select all channels

	do {
		mdelay(10);
		ret = regmap_read(st->regmap, 0x00a, &val);
		if (ret)
			return ret;
	} while (val != 0x81 && timeout--);

	if (val != 0x81) {
		dev_err(&st->spi->dev, "Failed to lock: %x\n", val);
		return -ENODEV;
	}

	return 0;
}

static const struct regmap_config ad9671_regmap_config = {
	.val_bits = 8,
	.reg_bits = 16,

	.max_register = 0x1fff,
};

static int ad9671_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9671_state *st;
	unsigned int clk_rate;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->regmap = devm_regmap_init_spi(spi, &ad9671_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	st->ref_clk = devm_clk_get(&spi->dev, "clk");
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	clk_rate = clk_get_rate(st->ref_clk);

	switch (clk_rate) {
	case 40000000:
	case 80000000:
		break;
	default:
		dev_err(&spi->dev, "Invalid reference clock rate: %d\n", clk_rate);
		return -EINVAL;
	}

	st->tx_trig = devm_gpiod_get_optional(&spi->dev, "tx-trig", GPIOD_OUT_LOW);
	if (IS_ERR(st->tx_trig))
		return PTR_ERR(st->tx_trig);

	if (st->tx_trig) {
		gpiod_export(st->tx_trig, false);
		gpiod_export_link(&spi->dev, "tx-trig", st->tx_trig);
	}

	ret = clk_prepare_enable(st->ref_clk);
	if (ret)
		goto err_gpio_unexport;

	ret = ad9671_setup(st, clk_rate);
	if (ret)
		goto err_clk_disable;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9671_info;
	indio_dev->channels = ad9671_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9671_channels);
	indio_dev->modes = INDIO_BUFFER_HARDWARE;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_clk_disable;

	spi_set_drvdata(spi, indio_dev);

	return 0;

err_gpio_unexport:
	gpiod_unexport(st->tx_trig);
err_clk_disable:
	clk_disable_unprepare(st->ref_clk);
	return ret;
}

static int ad9671_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad9671_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	gpiod_unexport(st->tx_trig);
	clk_disable_unprepare(st->ref_clk);

	return 0;
}

static const struct spi_device_id ad9671_id[] = {
	{"ad9671", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9671_id);

static struct spi_driver ad9671_driver = {
	.driver = {
		.name	= "ad9671",
	},
	.probe		= ad9671_probe,
	.remove		= ad9671_remove,
	.id_table	= ad9671_id,
};
module_spi_driver(ad9671_driver);

MODULE_DESCRIPTION("Analog Devices AD9671");
MODULE_LICENSE("GPL v2");
