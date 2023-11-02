// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD7091R8 12-bit SAR ADC driver
 *
 * Copyright 2023 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/spi/spi.h>

#include "ad7091r-base.h"

#define AD7091R8_REG_ADDR_MSK				GENMASK(15, 11)
#define AD7091R8_RD_WR_FLAG_MSK				BIT(10)
#define AD7091R8_REG_DATA_MSK				GENMASK(9, 0)

#define AD7091R2_DEV_NAME				"ad7091r-2"
#define AD7091R4_DEV_NAME				"ad7091r-4"
#define AD7091R8_DEV_NAME				"ad7091r-8"

#define AD7091R_SPI_CHIP_INFO(n) {					\
	.name = AD7091R##n##_DEV_NAME,					\
	.type =	AD7091R##n,						\
	.channels = ad7091r##n##_channels,				\
	.num_channels = ARRAY_SIZE(ad7091r##n##_channels),		\
	.vref_mV = 2500,						\
}

#define AD7091R_SPI_CHIP_INFO_IRQ(n) {					\
	.name = AD7091R##n##_DEV_NAME,					\
	.type =	AD7091R##n,						\
	.channels = ad7091r##n##_channels_irq,				\
	.num_channels = ARRAY_SIZE(ad7091r##n##_channels_irq),		\
	.vref_mV = 2500,						\
}

static const struct iio_chan_spec ad7091r2_channels[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
};

static const struct iio_chan_spec ad7091r4_channels[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
	AD7091R_CHANNEL(2, 12, NULL, 0),
	AD7091R_CHANNEL(3, 12, NULL, 0),
};

static const struct iio_chan_spec ad7091r4_channels_irq[] = {
	AD7091R_CHANNEL(0, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(1, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(2, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(3, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
};

static const struct iio_chan_spec ad7091r8_channels[] = {
	AD7091R_CHANNEL(0, 12, NULL, 0),
	AD7091R_CHANNEL(1, 12, NULL, 0),
	AD7091R_CHANNEL(2, 12, NULL, 0),
	AD7091R_CHANNEL(3, 12, NULL, 0),
	AD7091R_CHANNEL(4, 12, NULL, 0),
	AD7091R_CHANNEL(5, 12, NULL, 0),
	AD7091R_CHANNEL(6, 12, NULL, 0),
	AD7091R_CHANNEL(7, 12, NULL, 0),
};

static const struct iio_chan_spec ad7091r8_channels_irq[] = {
	AD7091R_CHANNEL(0, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(1, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(2, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(3, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(4, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(5, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(6, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
	AD7091R_CHANNEL(7, 12, ad7091r_events, ARRAY_SIZE(ad7091r_events)),
};

static const struct ad7091r_chip_info ad7091r_spi_chip_info[] = {
	[AD7091R2] = AD7091R_SPI_CHIP_INFO(2),
	[AD7091R4] = AD7091R_SPI_CHIP_INFO(4),
	[AD7091R8] = AD7091R_SPI_CHIP_INFO(8),
};

static const struct ad7091r_chip_info ad7091r_spi_chip_info_irq[] = {
	[AD7091R4] = AD7091R_SPI_CHIP_INFO_IRQ(4),
	[AD7091R8] = AD7091R_SPI_CHIP_INFO_IRQ(8),
};

static void ad7091r_pulse_convst(struct ad7091r_state *st)
{
	gpiod_set_value_cansleep(st->convst_gpio, 1);
	gpiod_set_value_cansleep(st->convst_gpio, 0);
}

static const struct regmap_config ad7091r_spi_regmap_config[] = {
	[AD7091R2] = {
		.reg_bits = 5,
		.pad_bits = 3,
		.val_bits = 16,
		.volatile_reg = ad7091r_volatile_reg,
		.writeable_reg = ad7091r_writeable_reg,
		.max_register = AD7091R_REG_CH_HYSTERESIS(2),
	},
	[AD7091R4] = {
		.reg_bits = 5,
		.pad_bits = 3,
		.val_bits = 16,
		.volatile_reg = ad7091r_volatile_reg,
		.writeable_reg = ad7091r_writeable_reg,
		.max_register = AD7091R_REG_CH_HYSTERESIS(4),
	},
	[AD7091R8] = {
		.reg_bits = 5,
		.pad_bits = 3,
		.write_flag_mask = BIT(2),
		.val_bits = 16,
		.volatile_reg = ad7091r_volatile_reg,
		.writeable_reg = ad7091r_writeable_reg,
		.max_register = AD7091R_REG_CH_HYSTERESIS(8),
	},
};

static int ad7091r_regmap_bus_reg_read(void *context, unsigned int reg,
				       unsigned int *val)
{
	struct ad7091r_state *st = context;
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	const struct regmap_config *conf = &ad7091r_spi_regmap_config[st->chip_info->type];
	int ret;

	struct spi_transfer t[] = {
		{
			.tx_buf = &st->tx_buf,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = &st->rx_buf,
			.len = 2,
		}
	};

	if (reg == AD7091R_REG_RESULT)
		ad7091r_pulse_convst(st);

	reg <<= conf->pad_bits;
	st->tx_buf = cpu_to_be16(reg << 8);

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	*val = be16_to_cpu(st->rx_buf);
	return 0;
}

static int ad7091r_regmap_bus_reg_write(void *context, unsigned int reg,
					unsigned int val)
{
	struct ad7091r_state *st = context;
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);

	/*
	 * AD7091R-2/-4/-8 protocol (datasheet page 31) is to do a single SPI
	 * transfer with reg address set in bits B15:B11 and value set in B9:B0.
	 */
	st->tx_buf = cpu_to_be16(FIELD_PREP(AD7091R8_REG_DATA_MSK, val) |
			 FIELD_PREP(AD7091R8_RD_WR_FLAG_MSK, 1) |
			 FIELD_PREP(AD7091R8_REG_ADDR_MSK, reg));

	return spi_write(spi, &st->tx_buf, 2);
}

static struct regmap_bus ad7091r8_regmap_bus = {
	.reg_read = ad7091r_regmap_bus_reg_read,
	.reg_write = ad7091r_regmap_bus_reg_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static int ad7091r8_gpio_setup(struct ad7091r_state *st)
{
	st->convst_gpio = devm_gpiod_get(st->dev, "adi,conversion-start",
					 GPIOD_OUT_LOW);
	if (IS_ERR(st->convst_gpio))
		return dev_err_probe(st->dev, PTR_ERR(st->convst_gpio),
				     "Error getting convst GPIO: %ld\n",
				     PTR_ERR(st->convst_gpio));

	st->reset_gpio =  devm_gpiod_get_optional(st->dev, "reset",
						  GPIOD_OUT_HIGH);
	if (IS_ERR(st->reset_gpio))
		return PTR_ERR(st->reset_gpio);

	if (st->reset_gpio) {
		fsleep(20);
		gpiod_set_value_cansleep(st->reset_gpio, 0);
	}

	return 0;
}

static int ad7091r8_spi_probe(struct spi_device *spi)
{
	const struct ad7091r_chip_info *chip_info;
	struct ad7091r_state *st;
	struct iio_dev *iio_dev;
	struct regmap *map;
	int ret;

	chip_info = device_get_match_data(&spi->dev);
	if (!chip_info) {
		chip_info = (const struct ad7091r_chip_info *)spi_get_device_id(spi)->driver_data;
		if (!chip_info)
			return -EINVAL;
	}

	iio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!iio_dev)
		return -ENOMEM;

	st = iio_priv(iio_dev);
	st->dev = &spi->dev;

	map = devm_regmap_init(&spi->dev, &ad7091r8_regmap_bus, st,
			       &ad7091r_spi_regmap_config[chip_info->type]);

	if (IS_ERR(map))
		return dev_err_probe(&spi->dev, PTR_ERR(map),
				     "Error initializing spi regmap: %ld\n",
				     PTR_ERR(map));

	ret = ad7091r8_gpio_setup(st);
	if (ret < 0)
		return ret;

	if (spi->irq)
		chip_info = &ad7091r_spi_chip_info_irq[chip_info->type];

	return ad7091r_probe(iio_dev, chip_info->name, chip_info, map, spi->irq);
}

static const struct of_device_id ad7091r8_of_match[] = {
	{ .compatible = "adi,ad7091r2", .data = &ad7091r_spi_chip_info[AD7091R2] },
	{ .compatible = "adi,ad7091r4", .data = &ad7091r_spi_chip_info[AD7091R4] },
	{ .compatible = "adi,ad7091r8", .data = &ad7091r_spi_chip_info[AD7091R8] },
	{ },
};
MODULE_DEVICE_TABLE(of, ad7091r8_of_match);

static const struct spi_device_id ad7091r8_spi_id[] = {
	{ "ad7091r2", (kernel_ulong_t)&ad7091r_spi_chip_info[AD7091R2] },
	{ "ad7091r4", (kernel_ulong_t)&ad7091r_spi_chip_info[AD7091R4] },
	{ "ad7091r8", (kernel_ulong_t)&ad7091r_spi_chip_info[AD7091R8] },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad7091r8_spi_id);

static struct spi_driver ad7091r8_driver = {
	.driver = {
		.name = "ad7091r8",
		.of_match_table = ad7091r8_of_match,
	},
	.probe = ad7091r8_spi_probe,
	.id_table = ad7091r8_spi_id,
};
module_spi_driver(ad7091r8_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7091R8 ADC driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_AD7091R);
