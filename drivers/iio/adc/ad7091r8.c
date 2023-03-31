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

static const struct regmap_range ad7091r2_readable_ranges[] = {
	regmap_reg_range(AD7091R_REG_RESULT,
			 AD7091R_REG_CH_HYSTERESIS(AD7091R2_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r4_readable_ranges[] = {
	regmap_reg_range(AD7091R_REG_RESULT,
			 AD7091R_REG_CH_HYSTERESIS(AD7091R4_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r8_readable_ranges[] = {
	regmap_reg_range(AD7091R_REG_RESULT,
			 AD7091R_REG_CH_HYSTERESIS(AD7091R8_NUM_CHANNELS)),
};

static const struct regmap_access_table ad7091r2_readable_regs_table = {
	.yes_ranges = ad7091r2_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r2_readable_ranges),
};

static const struct regmap_access_table ad7091r4_readable_regs_table = {
	.yes_ranges = ad7091r4_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r4_readable_ranges),
};

static const struct regmap_access_table ad7091r8_readable_regs_table = {
	.yes_ranges = ad7091r8_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r8_readable_ranges),
};

static const struct regmap_range ad7091r2_writable_ranges[] = {
	regmap_reg_range(AD7091R_REG_CHANNEL, AD7091R_REG_CONF),
	regmap_reg_range(AD7091R_REG_CH_LOW_LIMIT(0),
			 AD7091R_REG_CH_HYSTERESIS(AD7091R2_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r4_writable_ranges[] = {
	regmap_reg_range(AD7091R_REG_CHANNEL, AD7091R_REG_CONF),
	regmap_reg_range(AD7091R_REG_CH_LOW_LIMIT(0),
			 AD7091R_REG_CH_HYSTERESIS(AD7091R4_NUM_CHANNELS)),
};

static const struct regmap_range ad7091r8_writable_ranges[] = {
	regmap_reg_range(AD7091R_REG_CHANNEL, AD7091R_REG_CONF),
	regmap_reg_range(AD7091R_REG_CH_LOW_LIMIT(0),
			 AD7091R_REG_CH_HYSTERESIS(AD7091R8_NUM_CHANNELS)),
};

static const struct regmap_access_table ad7091r2_writable_regs_table = {
	.yes_ranges = ad7091r2_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r2_writable_ranges),
};

static const struct regmap_access_table ad7091r4_writable_regs_table = {
	.yes_ranges = ad7091r4_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r4_writable_ranges),
};

static const struct regmap_access_table ad7091r8_writable_regs_table = {
	.yes_ranges = ad7091r8_writable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad7091r8_writable_ranges),
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

static void ad7091r_pulse_reset(struct ad7091r_state *st)
{
	gpiod_set_value_cansleep(st->reset_gpio, 1);
	gpiod_set_value_cansleep(st->reset_gpio, 0);
}

static int ad7091r_spi_read(void *context, const void *reg, size_t reg_size,
			    void *val, size_t val_size)
{
	struct ad7091r_state *st = context;
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	unsigned int reg_add = *(unsigned int *)reg & 0x1f;
	__be16 rx;
	__be16 tx;
	int ret;

	struct spi_transfer t[] = {
		{
			.tx_buf = &tx,
			.len = val_size,
			.cs_change = 1,
		}, {
			.rx_buf = &rx,
			.len = val_size,
		}
	};
	/* Only pulse CONVST if new readings are required */
	if (reg_add == AD7091R_REG_RESULT)
		ad7091r_pulse_convst(st);

	/* A write command to a read only register is considered NOP. */
	tx = cpu_to_be16(FIELD_PREP(AD7091R8_RD_WR_FLAG_MSK, 0) |
			 FIELD_PREP(AD7091R8_REG_ADDR_MSK, reg_add));
	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	ret = be16_to_cpu(rx);
	memcpy(val, &ret, val_size);
	return 0;
}

static int ad7091r_spi_write(void *context, const void *data, size_t count)
{
	struct ad7091r_state *st = context;
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	u32 write_data = *((int *)data);
	u8 reg_add = write_data & 0xFF;
	u16 tx_data = write_data >> 8;
	__be16 tx;

	/*
	 * AD7091R-2/-4/-8 protocol (datasheet page 31) is to do a single SPI
	 * transfer with reg address set in bits B15:B11 and value set in B9:B0.
	 */
	tx = cpu_to_be16(FIELD_PREP(AD7091R8_REG_DATA_MSK, tx_data) |
			 FIELD_PREP(AD7091R8_RD_WR_FLAG_MSK, 1) |
			 FIELD_PREP(AD7091R8_REG_ADDR_MSK, reg_add));
	return spi_write(spi, &tx, 2);
}

static struct regmap_bus ad7091r8_regmap_bus = {
	.read = ad7091r_spi_read,
	.write = ad7091r_spi_write,
	.reg_format_endian_default = REGMAP_ENDIAN_NATIVE,
	.val_format_endian_default = REGMAP_ENDIAN_NATIVE,
};

static const struct regmap_config ad7091r_spi_regmap_config[] = {
	[AD7091R2] = {
		.reg_bits = 8,
		.val_bits = 16,
		.rd_table = &ad7091r2_readable_regs_table,
		.wr_table = &ad7091r2_writable_regs_table,
		.max_register = AD7091R_REG_CH_HYSTERESIS(2),
	},
	[AD7091R4] = {
		.reg_bits = 8,
		.val_bits = 16,
		.rd_table = &ad7091r4_readable_regs_table,
		.wr_table = &ad7091r4_writable_regs_table,
		.max_register = AD7091R_REG_CH_HYSTERESIS(4),
	},
	[AD7091R8] = {
		.reg_bits = 8,
		.val_bits = 16,
		.rd_table = &ad7091r8_readable_regs_table,
		.wr_table = &ad7091r8_writable_regs_table,
		.max_register = AD7091R_REG_CH_HYSTERESIS(8),
	},
};

static int ad7091r8_gpio_setup(struct ad7091r_state *st)
{
	st->reset_gpio = devm_gpiod_get(st->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(st->dev, PTR_ERR(st->reset_gpio),
				     "Error getting reset GPIO: %ld\n",
				     PTR_ERR(st->reset_gpio));

	st->convst_gpio = devm_gpiod_get(st->dev, "adi,conversion-start",
					 GPIOD_OUT_LOW);
	if (IS_ERR(st->convst_gpio))
		return dev_err_probe(st->dev, PTR_ERR(st->convst_gpio),
				     "Error getting convst GPIO: %ld\n",
				     PTR_ERR(st->convst_gpio));
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

	ad7091r_pulse_reset(st);

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
