// SPDX-License-Identifier: GPL-2.0
/*
 * AD5672R, AD5674R, AD5676, AD5676R, AD5679R,
 * AD5681R, AD5682R, AD5683, AD5683R, AD5684,
 * AD5684R, AD5685R, AD5686, AD5686R
 * Digital to analog converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

#include <asm/byteorder.h>

#include "ad5686.h"

static int ad5686_spi_write(struct ad5686_state *st,
			    u8 cmd, u8 addr, u16 val)
{
	struct spi_device *spi = to_spi_device(st->dev);
	u8 tx_len, *buf;

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		st->data[0].d16 = cpu_to_be16(AD5310_CMD(cmd) |
					      val);
		buf = &st->data[0].d8[0];
		tx_len = 2;
		break;
	case AD5683_REGMAP:
		st->data[0].d32 = cpu_to_be32(AD5686_CMD(cmd) |
					      AD5683_DATA(val));
		buf = &st->data[0].d8[1];
		tx_len = 3;
		break;
	case AD5686_REGMAP:
		st->data[0].d32 = cpu_to_be32(AD5686_CMD(cmd) |
					      AD5686_ADDR(addr) |
					      val);
		buf = &st->data[0].d8[1];
		tx_len = 3;
		break;
	default:
		return -EINVAL;
	}

	return spi_write(spi, buf, tx_len);
}

static int ad5686_spi_read(struct ad5686_state *st, u8 addr)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->data[0].d8[1],
			.len = 3,
			.cs_change = 1,
		}, {
			.tx_buf = &st->data[1].d8[1],
			.rx_buf = &st->data[2].d8[1],
			.len = 3,
		},
	};
	struct spi_device *spi = to_spi_device(st->dev);
	u8 cmd = 0;
	int ret;

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		return -ENOTSUPP;
	case AD5683_REGMAP:
		cmd = AD5686_CMD_READBACK_ENABLE_V2;
		break;
	case AD5686_REGMAP:
		cmd = AD5686_CMD_READBACK_ENABLE;
		break;
	default:
		return -EINVAL;
	}

	st->data[0].d32 = cpu_to_be32(AD5686_CMD(cmd) |
				      AD5686_ADDR(addr));
	st->data[1].d32 = cpu_to_be32(AD5686_CMD(AD5686_CMD_NOOP));

	ret = spi_sync_transfer(spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	return be32_to_cpu(st->data[2].d32);
}

static int ad5686_spi_probe(struct spi_device *spi)
{
	return ad5686_probe(&spi->dev, spi_get_device_match_data(spi),
			    spi->modalias, ad5686_spi_write, ad5686_spi_read);
}

static const struct spi_device_id ad5686_spi_id[] = {
	{ "ad5310r",  (kernel_ulong_t)&ad5310r_chip_info },
	{ "ad5672r",  (kernel_ulong_t)&ad5672r_chip_info },
	{ "ad5674r",  (kernel_ulong_t)&ad5674r_chip_info },
	{ "ad5676",   (kernel_ulong_t)&ad5676_chip_info },
	{ "ad5676r",  (kernel_ulong_t)&ad5676r_chip_info },
	{ "ad5679r",  (kernel_ulong_t)&ad5679r_chip_info },
	{ "ad5681r",  (kernel_ulong_t)&ad5681r_chip_info },
	{ "ad5682r",  (kernel_ulong_t)&ad5682r_chip_info },
	{ "ad5683",   (kernel_ulong_t)&ad5683_chip_info },
	{ "ad5683r",  (kernel_ulong_t)&ad5683r_chip_info },
	{ "ad5684",   (kernel_ulong_t)&ad5684_chip_info },
	{ "ad5684r",  (kernel_ulong_t)&ad5684r_chip_info },
	{ "ad5685",   (kernel_ulong_t)&ad5685r_chip_info }, /* Does not exist */
	{ "ad5685r",  (kernel_ulong_t)&ad5685r_chip_info },
	{ "ad5686",   (kernel_ulong_t)&ad5686_chip_info },
	{ "ad5686r",  (kernel_ulong_t)&ad5686r_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5686_spi_id);

static struct spi_driver ad5686_spi_driver = {
	.driver = {
		.name = "ad5686",
	},
	.probe = ad5686_spi_probe,
	.id_table = ad5686_spi_id,
};

module_spi_driver(ad5686_spi_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5686 and similar multi-channel DACs");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS("IIO_AD5686");
