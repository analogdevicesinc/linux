// SPDX-License-Identifier: GPL-2.0-only
/*
 * MAX14001/MAX14002 SPI ADC driver
 *
 * Copyright (c) 2025 Marilene Andrade Garcia <marilene.agarcia@gmail.com>
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/MAX14001-MAX14002.pdf
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regulator/consumer.h>

/* MAX14001 registers definition */
#define MAX14001_REG_ADC				0x00
#define MAX14001_REG_FADC				0x01
#define MAX14001_REG_FLAGS				0x02
#define MAX14001_REG_FLTEN				0x03
#define MAX14001_REG_THL				0x04
#define MAX14001_REG_THU				0x05
#define MAX14001_REG_INRR				0x06
#define MAX14001_REG_INRT				0x07
#define MAX14001_REG_INRP				0x08
#define MAX14001_REG_CFG				0x09
#define MAX14001_REG_ENBL				0x0A
#define MAX14001_REG_ACT				0x0B
#define MAX14001_REG_WEN				0x0C

/* MAX14001 CONTROL values*/
#define MAX14001_REG_WRITE				0x1
#define MAX14001_REG_READ				0x0

/* MAX14001 MASKS */
#define MAX14001_MASK_ADDR				GENMASK(15, 11)
#define MAX14001_MASK_WR				BIT(10)
#define MAX14001_MASK_DATA				GENMASK(9, 0)

enum max14001_chip_model {
	max14001,
	max14002,
};

struct max14001_chip_info {
	const char *name;
};

struct max14001_state {
	const struct max14001_chip_info *chip_info;
	struct spi_device *spi;
	int vref_mv;

	__be16 rx_buffer __aligned(IIO_DMA_MINALIGN);
	__be16 tx_buffer;
};

static struct max14001_chip_info max14001_chip_info_tbl[] = {
	[max14001] = {
		.name = "max14001",
	},
	[max14002] = {
		.name = "max14002",
	},
};

static int max14001_spi_read(struct max14001_state *st, u16 reg, int *val)
{
	struct spi_transfer xfer[] = {
		{
			.tx_buf = &st->tx_buffer,
			.len = sizeof(st->tx_buffer),
			.cs_change = 1,
		},
		{
			.rx_buf = &st->rx_buffer,
			.len = sizeof(st->rx_buffer),
		},
	};
	int ret;

	st->tx_buffer = FIELD_PREP(MAX14001_MASK_ADDR, reg) |
			FIELD_PREP(MAX14001_MASK_WR, MAX14001_REG_READ);
	st->tx_buffer = bitrev16(st->tx_buffer);

	ret = spi_sync_transfer(st->spi, xfer, ARRAY_SIZE(xfer));
	if (ret < 0)
		return ret;

	st->rx_buffer = bitrev16(be16_to_cpu(st->rx_buffer));
	*val = FIELD_GET(MAX14001_MASK_DATA, st->rx_buffer);

	return 0;
}

static int max14001_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max14001_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = max14001_spi_read(st, MAX14001_REG_ADC, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		ret = max14001_spi_read(st, MAX14001_REG_FADC, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		*val = st->vref_mv;
		*val2 = 10;

		return IIO_VAL_FRACTIONAL_LOG2;
	}

	return -EINVAL;
}

static const struct iio_info max14001_info = {
	.read_raw = max14001_read_raw,
};

static const struct iio_chan_spec max14001_channel[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_AVERAGE_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	}
};

static int max14001_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct max14001_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = spi_get_device_match_data(spi);
	if (!st->chip_info)
		return dev_err_probe(dev, -ENODEV, "Failed to get match data\n");

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &max14001_info;
	indio_dev->channels = max14001_channel;
	indio_dev->num_channels = ARRAY_SIZE(max14001_channel);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
			"Failed to enable specified Vdd supply\n");

	ret = devm_regulator_get_enable(dev, "vddl");
	if (ret)
		return dev_err_probe(dev, ret,
			"Failed to enable specified Vddl supply\n");

	ret = devm_regulator_get_enable_read_voltage(dev, "vrefin");
	if (ret < 0)
		st->vref_mv = 1250000 / 1000;
	else
		st->vref_mv = ret / 1000;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id max14001_id_table[] = {
	{ "max14001", (kernel_ulong_t)&max14001_chip_info_tbl[max14001] },
	{ "max14002", (kernel_ulong_t)&max14001_chip_info_tbl[max14002] },
	{}
};
MODULE_DEVICE_TABLE(spi, max14001_id_table);

static const struct of_device_id max14001_of_match[] = {
	{ .compatible = "adi,max14001",
	  .data = &max14001_chip_info_tbl[max14001], },
	{ .compatible = "adi,max14002",
	  .data = &max14001_chip_info_tbl[max14002], },
	{ }
};
MODULE_DEVICE_TABLE(of, max14001_of_match);

static struct spi_driver max14001_driver = {
	.driver = {
		.name = "max14001",
		.of_match_table = max14001_of_match,
	},
	.probe = max14001_probe,
	.id_table = max14001_id_table,
};
module_spi_driver(max14001_driver);

MODULE_AUTHOR("Marilene Andrade Garcia <marilene.agarcia@gmail.com>");
MODULE_DESCRIPTION("Analog Devices MAX14001/MAX14002 ADCs driver");
MODULE_LICENSE("GPL");
