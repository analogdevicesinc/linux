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
#include <linux/of.h>

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

/* MAX14001 verification registers definition */
#define MAX14001_REG_VERIFICATION(x)	(0x10 + (x))
#define MAX14001_REG_FLTV				0x13
#define MAX14001_REG_THLV				0x14
#define MAX14001_REG_THUV				0x15
#define MAX14001_REG_INRRV				0x16
#define MAX14001_REG_INRTV				0x17
#define MAX14001_REG_INRPV				0x18
#define MAX14001_REG_CFGV				0x19
#define MAX14001_REG_ENBLV				0x1A

/* MAX14001 CONTROL values*/
#define MAX14001_REG_WRITE				0x1
#define MAX14001_REG_READ				0x0

/* MAX14001 MASKS */
#define MAX14001_MASK_ADDR				GENMASK(15, 11)
#define MAX14001_MASK_WR				BIT(10)
#define MAX14001_MASK_DATA				GENMASK(9, 0)

/* MAX14001_REG_FLAGS MASKS */
#define MAX14001_MASK_FLAGS_ADC			BIT(1)
#define MAX14001_MASK_FLAGS_INRD		BIT(2)
#define MAX14001_MASK_FLAGS_SPI			BIT(3)
#define MAX14001_MASK_FLAGS_COM			BIT(4)
#define MAX14001_MASK_FLAGS_CRCL		BIT(5)
#define MAX14001_MASK_FLAGS_CRCF		BIT(6)
#define MAX14001_MASK_FLAGS_FET			BIT(7)
#define MAX14001_MASK_FLAGS_MV			BIT(8)

/* MAX14001_REG_FLTEN MASKS */
#define MAX14001_MASK_FLTEN_DYEN		BIT(0)
#define MAX14001_MASK_FLTEN_EADC		BIT(1)
#define MAX14001_MASK_FLTEN_EINRD		BIT(2)
#define MAX14001_MASK_FLTEN_ESPI		BIT(3)
#define MAX14001_MASK_FLTEN_ECOM		BIT(4)
#define MAX14001_MASK_FLTEN_ECRCL		BIT(5)
#define MAX14001_MASK_FLTEN_ECRCF		BIT(6)
#define MAX14001_MASK_FLTEN_EFET		BIT(7)
#define MAX14001_MASK_FLTEN_EMV			BIT(8)

/* MAX14001_REG_WEN values*/
#define MAX14001_REG_WEN_WRITE_ENABLE	0x294
#define MAX14001_REG_WEN_WRITE_DISABLE	0x0

enum max14001_chips {
	max14001,
	max14002,
};

struct max14001_state {
	struct spi_device *spi;
};

static int max14001_spi_read(struct max14001_state *st, u16 reg, u16 *val)
{
	u16 tx = 0;
	u16 rx = 0;
	u16 reversed = 0;
	int ret = 0;

	pr_err("[Log Debug] max14001_spi_read: reg: %x, val: %x\n", reg, *val);

	tx |= FIELD_PREP(MAX14001_MASK_ADDR, reg);
	tx |= FIELD_PREP(MAX14001_MASK_WR, MAX14001_REG_READ);
	reversed = bitrev16(tx);

	ret = spi_write_then_read(st->spi, &reversed, 2, &rx, 2);
	if (ret < 0)
		return ret;

	reversed = bitrev16(be16_to_cpu(rx));
	*val = MAX14001_MASK_ADDR&reversed;

	return ret;
}

static int max14001_spi_write(struct max14001_state *st, u16 reg, u16 val)
{
	u16 tx = 0;
	u16 msg = 0;
	u16 reversed = 0;
	int ret = 0;

	pr_err("[Log Debug] max14001_spi_write: reg: %x, val: %x\n", reg, val);

	struct spi_transfer xfer = {
		.tx_buf = NULL,
		.len = 0,
	};

	msg |= FIELD_PREP(MAX14001_MASK_ADDR, reg);
	msg |= FIELD_PREP(MAX14001_MASK_WR, MAX14001_REG_WRITE);
	msg |= FIELD_PREP(MAX14001_MASK_DATA, val);

	reversed = bitrev16(msg);
	put_unaligned_be16(reversed, &tx);

	xfer.tx_buf = &tx;
	xfer.len = sizeof(tx);

	pr_err("[Log Debug] max14001_spi_write: msg: %x, tx: %x\n", msg, tx);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int max14001_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct max14001_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		pr_err("[Log Debug] max14001_read_raw: IIO_CHAN_INFO_RAW\n");
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		pr_err("[Log Debug] max14001_read_raw: IIO_CHAN_INFO_SCALE\n");
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int max14001_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct max14001_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		pr_err("[Log Debug] max14001_write_raw: IIO_CHAN_INFO_RAW\n");
		return 0;
	}

	return -EINVAL;
}

static const struct iio_info max14001_info = {
	.read_raw = max14001_read_raw,
	.write_raw = max14001_write_raw,
};

static const struct iio_chan_spec max14001_channel_voltage[] = {
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 0,
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct iio_chan_spec max14001_channel_current[] = {
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = 0,
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_SCALE),
	}
};

static int max14001_probe(struct spi_device *spi)
{
	pr_err("[Log Debug] max14001_probe\n");

	struct max14001_state *st;
	struct iio_dev *indio_dev;
	bool current_channel = false;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = "max14001"; //spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &max14001_info;

	for_each_available_child_of_node_scoped(spi->dev.of_node, child) {
		current_channel = of_property_read_bool(child, "current-channel");
		if (current_channel)
			break;
	}

	if (current_channel) {
		indio_dev->channels = max14001_channel_current;
		indio_dev->num_channels = ARRAY_SIZE(max14001_channel_current);
	} else {
		indio_dev->channels = max14001_channel_voltage;
		indio_dev->num_channels = ARRAY_SIZE(max14001_channel_voltage);
	}

	//Enable register write
	max14001_spi_write(st, MAX14001_REG_WEN, MAX14001_REG_WEN_WRITE_ENABLE);
	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max14001_id_table[] = {
	{ "max14001", max14001 },
	{ "max14002", max14002 },
	{}
};
MODULE_DEVICE_TABLE(spi, max14001_id_table);

static const struct of_device_id max14001_of_match[] = {
	{ .compatible = "adi,max14001" },
	{ .compatible = "adi,max14002" },
	{}
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
MODULE_LICENSE("GPL v2");