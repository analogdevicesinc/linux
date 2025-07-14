// SPDX-License-Identifier: GPL-2.0-only
/*
 * MAX22531 SPI ADC Driver
 *
 * Copyright (C) 2025 Abhinav Jain
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/max22530-max22532.pdf
 */

#include <linux/module.h>
#include <asm/unaligned.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regulator/consumer.h>

#define MAX22531_REG_PROD_ID		0x00
#define MAX22531_REG_ADC_CHAN(x)	((x) + 1)
#define MAX22531_REG_FADC_CHAN(x)	((x) + 1)

#define MAX22531_VREF_MV		1800
#define MAX22531_DEVICE_REV_MSK		GENMASK(6, 0)
#define MAX22531_DEVICE_REV		0x01

#define MAX22531_REG_ADDR_MASK		GENMASK(7, 2)
#define MAX22531_REG_WRITE_MASK		BIT(1)

enum max22531_id {
	max22530,
	max22531,
	max22532,
};

struct max22531_chip_info {
	const char *name;
};

static struct max22531_chip_info max22531_chip_info_tbl[] = {
	[max22530] = {
		.name = "max22530",
	},
	[max22531] = {
		.name = "max22531",
	},
	[max22532] = {
		.name = "max22532",
	},
};

struct max22531 {
	struct spi_device *spi_dev;
	const struct max22531_chip_info *chip_info;
};

#define MAX22531_CHANNEL(ch)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.channel = (ch),					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
				BIT(IIO_CHAN_INFO_AVERAGE_RAW),         \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
	}

static const struct iio_chan_spec max22531_channels[] = {
	MAX22531_CHANNEL(0),
	MAX22531_CHANNEL(1),
	MAX22531_CHANNEL(2),
	MAX22531_CHANNEL(3),
};

static int max22531_reg_read(struct max22531 *adc, unsigned int reg,
			     unsigned int *readval)
{
	u8 cmd;

	cmd = FIELD_PREP(MAX22531_REG_ADDR_MASK, reg);
	*readval = spi_w8r16be(adc->spi_dev, cmd);
	if (*readval < 0)
		return *readval;

	return 0;
}

static int max22531_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max22531 *adc = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = max22531_reg_read(adc, MAX22531_REG_ADC_CHAN(chan->channel), val);
		if (ret)
			return ret;
	return IIO_VAL_INT;

	case IIO_CHAN_INFO_AVERAGE_RAW:
		ret = max22531_reg_read(adc, MAX22531_REG_FADC_CHAN(chan->channel), val);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = MAX22531_VREF_MV;
		*val2 = 12;

		return IIO_VAL_FRACTIONAL_LOG2;

	default:
		return -EINVAL;
	}
}

static const struct iio_info max22531_info = {
	.read_raw = max22531_read_raw,
};

static int max22531_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct max22531 *adc;
	unsigned int prod_id;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi_dev = spi;
	adc->chip_info = spi_get_device_match_data(spi);
	if (!adc->chip_info)
		return dev_err_probe(&spi->dev, -EINVAL, "no chip info\n");

	indio_dev->name = adc->chip_info->name;
	indio_dev->info = &max22531_info;
	indio_dev->channels = max22531_channels;
	indio_dev->num_channels = ARRAY_SIZE(max22531_channels);

	ret = devm_regulator_get_enable(&spi->dev, "vddl");
	if (ret)
		return dev_err_probe(&spi->dev, ret,
		       "Failed to retrieve power logic supply.\n");

	ret = devm_regulator_get_enable(&spi->dev, "vddpl");
	if (ret)
		return dev_err_probe(&spi->dev, ret,
		       "Failed to retrieve isolated DC-DC supply.\n");

	ret = max22531_reg_read(adc, MAX22531_REG_PROD_ID, &prod_id);
	if (ret ||
	    FIELD_GET(MAX22531_DEVICE_REV_MSK, prod_id) != MAX22531_DEVICE_REV)
		dev_warn(&spi->dev, "PROD_ID verification failed\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max22531_id[] = {
	{ "max22530", (kernel_ulong_t)&max22531_chip_info_tbl[max22530] },
	{ "max22531", (kernel_ulong_t)&max22531_chip_info_tbl[max22531] },
	{ "max22532", (kernel_ulong_t)&max22531_chip_info_tbl[max22532] },
	{ }
};
MODULE_DEVICE_TABLE(spi, max22531_id);

static const struct of_device_id max22531_spi_of_id[] = {
	{ .compatible = "adi,max22530",
		.data = &max22531_chip_info_tbl[max22530], },
	{ .compatible = "adi,max22531",
		.data = &max22531_chip_info_tbl[max22531], },
	{ .compatible = "adi,max22532",
		.data = &max22531_chip_info_tbl[max22532], },
	{ }
};
MODULE_DEVICE_TABLE(of, max22531_spi_of_id);

static struct spi_driver max22531_driver = {
	.driver = {
		.name = "max22531",
		.of_match_table = max22531_spi_of_id,
	},
	.probe		= max22531_probe,
	.id_table	= max22531_id,
};
module_spi_driver(max22531_driver);

MODULE_AUTHOR("Abhinav Jain <jain.abhinav177@gmail.com>");
MODULE_DESCRIPTION("MAX22531 ADC");
MODULE_LICENSE("GPL");
