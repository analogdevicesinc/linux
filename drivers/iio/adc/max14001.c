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

/* MAX14001 10-bit ADC */
#define MAX14001_NUMBER_OF_DATA_BITS	10
#define MAX14001_BIT_DIV				(1 << 10)

enum max14001_chip_model {
	max14001,
	max14002,
};

struct max14001_chip_info {
	const char *name;
	/* TODO: Add more information */
};

static struct max14001_chip_info max14001_chip_info_tbl[] = {
	[max14001] = {
		.name = "max14001",
	},
	[max14002] = {
		.name = "max14002",
	},
};

struct max14001_state {
	struct spi_device *spi;
	const struct max14001_chip_info *chip_info;
	int vref_mV;
};

static int max14001_get_scale(struct max14001_state *st)
{
	int scale;

	/* scale = range / 2^10 */
	scale = st->vref_mV / MAX14001_BIT_DIV;
	return scale;
}

static int max14001_get_vref_mV(struct max14001_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret = 0;

	ret = devm_regulator_get_enable_read_voltage(dev, "vrefin");
	if (ret < 0){
		st->vref_mV = 1250000 / 1000;
		dev_info(&st->spi->dev, "%s: vrefin not found. vref_mV %d\n", __func__, st->vref_mV);
	} else {
		st->vref_mV = ret / 1000;
		dev_info(&st->spi->dev, "%s: vrefin found. vref_mV %d\n", __func__, st->vref_mV);
	}

	return ret;
}

static int max14001_init_required_regulators(struct max14001_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret = 0;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable specified Vdd supply\n");

	ret = devm_regulator_get_enable(dev, "vddl");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable specified Vddl supply\n");

	return ret;
}

static int max14001_spi_read(struct max14001_state *st, u16 reg, int *val)
{
	u16 tx, rx, reversed;
	int ret;

	dev_info(&st->spi->dev, "%s: reg: %x, val: %x\n", __func__, reg, *val);

	tx |= FIELD_PREP(MAX14001_MASK_ADDR, reg);
	tx |= FIELD_PREP(MAX14001_MASK_WR, MAX14001_REG_READ);
	reversed = bitrev16(tx);

	ret = spi_write_then_read(st->spi, &reversed, 2, &rx, 2);
	if (ret < 0)
		return ret;

	/* TODO: Validate this line in the hw, could be le16_to_cpu */
	reversed = bitrev16(be16_to_cpu(rx));
	*val = FIELD_GET(MAX14001_MASK_DATA, reversed);

	return ret;
}

static int max14001_spi_write(struct max14001_state *st, u16 reg, u16 val)
{
	struct spi_transfer xfer;
	int ret;
	u16 tx, reversed;
	u16 msg = 0;

	dev_info(&st->spi->dev, "%s: reg: %x, val: %x\n", __func__, reg, val);

	msg |= FIELD_PREP(MAX14001_MASK_ADDR, reg);
	msg |= FIELD_PREP(MAX14001_MASK_WR, MAX14001_REG_WRITE);
	msg |= FIELD_PREP(MAX14001_MASK_DATA, val);

	reversed = bitrev16(msg);
	/* TODO: Validate this line in the hw, could be put_unaligned_le16 */
	put_unaligned_be16(reversed, &tx);

	xfer.tx_buf = &tx;
	xfer.len = sizeof(tx);

	dev_info(&st->spi->dev, "%s: msg: %x, tx: %x\n", __func__, msg, tx);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret < 0)
		return ret;

	return ret;
}

static int max14001_spi_write_single_reg(struct max14001_state *st, u16 reg, u16 val)
{
	int ret;

	/* Enable register write */
	ret = max14001_spi_write(st, MAX14001_REG_WEN, MAX14001_REG_WEN_WRITE_ENABLE);
	if (ret < 0)
		return ret;

	/* Write data into register */
	ret = max14001_spi_write(st, reg, val);
	if (ret < 0)
		return ret;

	/* Disable register write */
	ret = max14001_spi_write(st, MAX14001_REG_WEN, MAX14001_REG_WEN_WRITE_DISABLE);
	if (ret < 0)
		return ret;

	return ret;
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
		dev_info(&st->spi->dev, "%s: IIO_CHAN_INFO_RAW: channel: %d, val: %d\n", __func__, chan->channel, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_AVERAGE_RAW:
		ret = max14001_spi_read(st, MAX14001_REG_FADC, val);
		dev_info(&st->spi->dev, "%s: IIO_CHAN_INFO_AVERAGE_RAW: channel: %d, val: %d\n", __func__, chan->channel, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		ret = max14001_get_scale(st);
		*val = ret;
		*val2 = MAX14001_NUMBER_OF_DATA_BITS;
		dev_info(&st->spi->dev, "%s: IIO_CHAN_INFO_SCALE: val: %d, val2: %d\n", __func__, val, val2);

		return IIO_VAL_FRACTIONAL_LOG2;
	}

	return -EINVAL;
}

/* TODO: Check if this method is nedeed */
static int max14001_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val, int val2, long mask)
{
	struct max14001_state *st = iio_priv(indio_dev);

	switch (mask) {

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
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_AVERAGE_RAW) |
					  BIT(IIO_CHAN_INFO_SCALE),
	}
};

static const struct iio_chan_spec max14001_channel_current[] = {
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					  BIT(IIO_CHAN_INFO_AVERAGE_RAW) |
					  BIT(IIO_CHAN_INFO_SCALE),
	}
};

static int max14001_probe(struct spi_device *spi)
{
	const struct max14001_chip_info *info;
	struct device *dev = &spi->dev;
	struct max14001_state *st;
	struct iio_dev *indio_dev;
	bool current_channel = false;
	int ret;

	info = spi_get_device_match_data(spi);
	if (!dev)
		return dev_err_probe(dev, -ENODEV, "Failed to get match data\n");

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->chip_info = info;

	indio_dev->name = st->chip_info->name;
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

	dev_info(&st->spi->dev, "%s: probe\n", __func__);

	max14001_init_required_regulators(st);
	max14001_get_vref_mV(st);

	return devm_iio_device_register(&spi->dev, indio_dev);
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

