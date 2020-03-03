/*
 * Copyright 2015 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/delay.h>
#include <linux/clk.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>

#define AD9963_REG_SERIAL_PORT_CFG	0x00
#define AD9963_REG_ADC_ADDRESS		0x05
#define AD9963_REG_RXCML		0x0f
#define AD9963_REG_DIGITAL_FILTERS	0x30
#define AD9963_REG_TX_DATA_INF		0x31
#define AD9963_REG_RX_DATA_INF0		0x32
#define AD9963_REG_RX_DATA_INF1		0x3f
#define AD9963_REG_DAC12_CONFIG		0x40
#define AD9963_REG_DAC12_MSB(x)		(0x41 + (x) * 2)
#define AD9963_REG_DAC12_LSB(x)		(0x42 + (x) * 2)
#define AD9963_REG_POWER_DOWN0		0x60
#define AD9963_REG_DLL_CTRL0		0x71
#define AD9963_REG_DLL_CTRL1		0x72
#define AD9963_REG_DLL_CTRL2		0x75
#define AD9963_REG_AUX_ADC_CFG		0x77
#define AD9963_REG_AUX_ADC_MSB		0x78
#define AD9963_REG_AUX_ADC_LSB		0x79
#define AD9963_REG_AUX_ADC_CTRL0	0x7a
#define AD9963_REG_SYNC_REGS		0xff

struct ad9963 {
	struct clk *clk;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
};

static int ad9963_aux_dac12_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val)
{
	struct ad9963 *ad9963 = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(ad9963->regmap, AD9963_REG_DAC12_LSB(chan->address),
			&reg_val);
	if (ret)
		return ret;

	*val = reg_val & 0x0f;

	ret = regmap_read(ad9963->regmap, AD9963_REG_DAC12_MSB(chan->address),
			&reg_val);
	if (ret)
		return ret;

	*val |= reg_val << 4;

	return IIO_VAL_INT;
}

static int ad9963_aux_adc_read(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val)
{
	struct ad9963 *ad9963 = iio_priv(indio_dev);
	unsigned int timeout = 3;
	unsigned int reg_val;
	int ret;

	ret = regmap_write(ad9963->regmap, AD9963_REG_AUX_ADC_CFG,
		chan->address);
	if (ret)
		return ret;
	do {
		usleep_range(10000, 12000);
		ret = regmap_read(ad9963->regmap, AD9963_REG_AUX_ADC_LSB,
			&reg_val);
		if (ret)
			return ret;
	} while (!(reg_val & BIT(3)) && timeout--);

	if (!(reg_val & BIT(3)))
		return -EIO;

	*val = reg_val >> 4;

	ret = regmap_read(ad9963->regmap, AD9963_REG_AUX_ADC_MSB,
		&reg_val);
	if (ret)
		return ret;

	*val |= reg_val << 4;

	return IIO_VAL_INT;
}

static int ad9963_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (chan->output)
			return ad9963_aux_dac12_read(indio_dev, chan, val);
		else
			return ad9963_aux_adc_read(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			*val = 1000;
			*val2 = 5;
			return IIO_VAL_FRACTIONAL;
		}

		*val = 3200;
		*val2 = 12;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP)
			*val = -1366; /* 0 = 0K => 273200 / (1000 / 5) = 0 C */
		else
			return -EINVAL;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9963_aux_dac12_write(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int val)
{
	struct ad9963 *ad9963 = iio_priv(indio_dev);
	int ret;

	if (val < 0 || val > 4095)
		return -EINVAL;

	ret = regmap_write(ad9963->regmap, AD9963_REG_DAC12_MSB(chan->address),
		val >> 4);
	if (ret)
		return ret;
	ret = regmap_write(ad9963->regmap, AD9963_REG_DAC12_LSB(chan->address),
		val & 0xf);
	if (ret)
		return ret;

	return 0;
}

static int ad9963_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad9963_aux_dac12_write(indio_dev, chan, val);
	default:
		return -EINVAL;
	}
}

static int ad9963_debugfs_reg_access(struct iio_dev *indio_dev,
	unsigned int reg, unsigned int writeval, unsigned int *readval)
{
	struct ad9963 *ad9963 = iio_priv(indio_dev);

	if (readval)
		return regmap_read(ad9963->regmap, reg, readval);
	else
		return regmap_write(ad9963->regmap, reg, writeval);
}

static const struct iio_info ad9963_info = {
	.read_raw = &ad9963_read_raw,
	.write_raw = &ad9963_write_raw,
	.debugfs_reg_access = ad9963_debugfs_reg_access,
};

#define AD9963_CHAN_AUX_ADC(x, _addr) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (_addr), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			BIT(IIO_CHAN_INFO_SCALE), \
	.scan_index = -1, \
}

#define AD9963_CHAN_AUX_DAC(x) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (x), \
	.address = (x), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.scan_index = -1, \
	.output = 1, \
}

static const struct iio_chan_spec ad9963_channels[] = {
	AD9963_CHAN_AUX_ADC(0, 0),
	AD9963_CHAN_AUX_ADC(1, 1),
	AD9963_CHAN_AUX_ADC(2, 2),
	AD9963_CHAN_AUX_ADC(3, 4),
	AD9963_CHAN_AUX_ADC(4, 5),
	AD9963_CHAN_AUX_ADC(5, 6),

	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.address = 3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			BIT(IIO_CHAN_INFO_SCALE) |
			BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = -1,
	},

	AD9963_CHAN_AUX_DAC(0),
	AD9963_CHAN_AUX_DAC(1),
};

static int ad9963_m2k_setup(struct ad9963 *ad9963)
{
	/* Static setup for the M2K */

	regmap_write(ad9963->regmap, AD9963_REG_SERIAL_PORT_CFG, 0xa5);
	regmap_write(ad9963->regmap, AD9963_REG_SERIAL_PORT_CFG, 0x00);

	regmap_write(ad9963->regmap, AD9963_REG_DIGITAL_FILTERS, 0x37);
	regmap_write(ad9963->regmap, AD9963_REG_TX_DATA_INF, 0xa1);
	regmap_write(ad9963->regmap, AD9963_REG_RX_DATA_INF0, 0x23);
	regmap_write(ad9963->regmap, AD9963_REG_RX_DATA_INF1, 0x01);
	regmap_write(ad9963->regmap, AD9963_REG_DAC12_CONFIG, 0x32);

	regmap_write(ad9963->regmap, AD9963_REG_AUX_ADC_CTRL0, 0x80);
	regmap_write(ad9963->regmap, 0x66, 0x00);
	regmap_write(ad9963->regmap, 0x7b, 0x80); /* temp sense enable */

	regmap_write(ad9963->regmap, AD9963_REG_ADC_ADDRESS, 0x03);
	regmap_write(ad9963->regmap, AD9963_REG_RXCML, 0x02);
	/* PD Internal Reference, enable DLL */
	regmap_write(ad9963->regmap, AD9963_REG_POWER_DOWN0, 0x80);

	/* Configure DLL, DAC source = DLL, DLL rate = 3/2 * 100 = 150 */
	regmap_write(ad9963->regmap, AD9963_REG_DLL_CTRL0, 0x52);
	regmap_write(ad9963->regmap, AD9963_REG_DLL_CTRL1, 0x02);

	/* Reset DLL */
	regmap_write(ad9963->regmap, AD9963_REG_DLL_CTRL2, 0x00);
	regmap_write(ad9963->regmap, AD9963_REG_DLL_CTRL2, 0x08);
	regmap_write(ad9963->regmap, AD9963_REG_DLL_CTRL2, 0x00);

	regmap_write(ad9963->regmap, AD9963_REG_SYNC_REGS, 0x01);
	regmap_write(ad9963->regmap, AD9963_REG_SYNC_REGS, 0x00);

	return 0;
}

static const struct regmap_config ad9963_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = 0xff,
};

static int ad9963_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9963 *ad9963;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*ad9963));
	if (!indio_dev)
		return -ENOMEM;

	ad9963 = iio_priv(indio_dev);

	ad9963->clk = devm_clk_get(&spi->dev, "clk");
	if (IS_ERR(ad9963->clk))
		return PTR_ERR(ad9963->clk);

	ad9963->reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset",
		GPIOD_OUT_HIGH);
	if (IS_ERR(ad9963->reset_gpio))
		return PTR_ERR(ad9963->reset_gpio);

	if (ad9963->reset_gpio) {
		usleep_range(10000, 12000);
		gpiod_set_value(ad9963->reset_gpio, 0);
	}

	ad9963->regmap = devm_regmap_init_spi(spi, &ad9963_regmap_config);
	if (IS_ERR(ad9963->regmap))
		return PTR_ERR(ad9963->regmap);

	ret = clk_prepare_enable(ad9963->clk);
	if (ret)
		return ret;

	ad9963_m2k_setup(ad9963);

	indio_dev->info = &ad9963_info;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->channels = ad9963_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9963_channels);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_clk_disable;

	spi_set_drvdata(spi, indio_dev);

	return 0;

err_clk_disable:
	clk_disable_unprepare(ad9963->clk);

	return ret;
}

static int ad9963_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad9963 *ad9963 = iio_priv(indio_dev);

	clk_disable_unprepare(ad9963->clk);

	return 0;
}

static const struct spi_device_id ad9963_id[] = {
	{"ad9963", 0},
	{ }
};

static struct spi_driver ad9963_driver = {
	.driver = {
		.name	= "ad9963",
		.owner	= THIS_MODULE,
	},
	.probe		= ad9963_probe,
	.remove		= ad9963_remove,
	.id_table	= ad9963_id,
};
module_spi_driver(ad9963_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Driver for the Analog Devices AD9963");
