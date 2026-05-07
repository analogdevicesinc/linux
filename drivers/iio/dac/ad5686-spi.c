// SPDX-License-Identifier: GPL-2.0
/*
 * SPI driver for AD5686 and similar Digital to Analog Converters
 *
 * Copyright 2018-2026 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/err.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/overflow.h>
#include <linux/spi/spi.h>

#include <asm/byteorder.h>

#include "ad5686.h"

struct ad5686_spi_data {
	struct spi_message msg;
	unsigned int size;
	unsigned int capacity;
	struct spi_transfer xfers[] __counted_by(capacity);
};

static int ad5686_spi_write(struct ad5686_state *st,
			    u8 cmd, u8 addr, u16 val)
{
	struct ad5686_spi_data *bus_data = st->bus_data;
	struct spi_transfer *xfer;

	if (bus_data->size >= bus_data->capacity)
		return -E2BIG;

	if (bus_data->size)
		bus_data->xfers[bus_data->size - 1].cs_change = 1;
	else
		spi_message_init(&bus_data->msg);

	xfer = &bus_data->xfers[bus_data->size];
	xfer->rx_buf = NULL;
	xfer->cs_change = 0;

	switch (st->chip_info->regmap_type) {
	case AD5310_REGMAP:
		st->data[bus_data->size].d16 = cpu_to_be16(AD5310_CMD(cmd) |
							   val);
		xfer->tx_buf = &st->data[bus_data->size].d8[0];
		xfer->len = 2;
		break;
	case AD5683_REGMAP:
		st->data[bus_data->size].d32 = cpu_to_be32(AD5686_CMD(cmd) |
							   AD5683_DATA(val));
		xfer->tx_buf = &st->data[bus_data->size].d8[1];
		xfer->len = 3;
		break;
	case AD5686_REGMAP:
		st->data[bus_data->size].d32 = cpu_to_be32(AD5686_CMD(cmd) |
							   AD5686_ADDR(addr) |
							   val);
		xfer->tx_buf = &st->data[bus_data->size].d8[1];
		xfer->len = 3;
		break;
	default:
		return -EINVAL;
	}

	spi_message_add_tail(xfer, &bus_data->msg);
	bus_data->size++;

	return 0;
}

static int ad5686_spi_sync(struct ad5686_state *st)
{
	struct spi_device *spi = to_spi_device(st->dev);
	struct ad5686_spi_data *bus_data = st->bus_data;

	bus_data->size = 0; /* always reset, even on sync failure */
	return spi_sync(spi, &bus_data->msg);
}

static int ad5686_spi_read(struct ad5686_state *st, u8 addr)
{
	struct spi_device *spi = to_spi_device(st->dev);
	struct ad5686_spi_data *bus_data = st->bus_data;
	struct spi_transfer *xfer = &bus_data->xfers[0];
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

	xfer[0].tx_buf = &st->data[0].d8[1];
	xfer[0].len = 3;
	xfer[0].cs_change = 1;
	xfer[1].tx_buf = &st->data[1].d8[1];
	xfer[1].rx_buf = &st->data[2].d8[1];
	xfer[1].len = 3;
	xfer[1].cs_change = 0;

	spi_message_init_with_transfers(&bus_data->msg, xfer, 2);

	ret = spi_sync(spi, &bus_data->msg);
	if (ret)
		return ret;

	return be32_to_cpu(st->data[2].d32);
}

static const struct ad5686_bus_ops ad5686_spi_ops = {
	.write = ad5686_spi_write,
	.read = ad5686_spi_read,
	.sync = ad5686_spi_sync,
};

static int ad5686_spi_probe(struct spi_device *spi)
{
	const struct ad5686_chip_info *info = spi_get_device_match_data(spi);
	struct ad5686_spi_data *bus_data;
	unsigned int capacity;

	/* read operation requires at least 2 transfers */
	capacity = max(info->num_channels, 2);
	bus_data = devm_kzalloc(&spi->dev,
				struct_size(bus_data, xfers, capacity),
				GFP_KERNEL);
	if (!bus_data)
		return -ENOMEM;

	bus_data->capacity = capacity;

	return ad5686_probe(&spi->dev, info, spi->modalias, &ad5686_spi_ops,
			    bus_data);
}

static const struct spi_device_id ad5686_spi_id[] = {
	{ "ad5310r",  (kernel_ulong_t)&ad5310r_chip_info },
	{ "ad5313r",  (kernel_ulong_t)&ad5338r_chip_info },
	{ "ad5317r",  (kernel_ulong_t)&ad5317r_chip_info },
	{ "ad5672r",  (kernel_ulong_t)&ad5672r_chip_info },
	{ "ad5674",   (kernel_ulong_t)&ad5674_chip_info },
	{ "ad5674r",  (kernel_ulong_t)&ad5674r_chip_info },
	{ "ad5676",   (kernel_ulong_t)&ad5676_chip_info },
	{ "ad5676r",  (kernel_ulong_t)&ad5676r_chip_info },
	{ "ad5679",   (kernel_ulong_t)&ad5679_chip_info },
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
	{ "ad5687",   (kernel_ulong_t)&ad5687_chip_info },
	{ "ad5687r",  (kernel_ulong_t)&ad5687r_chip_info },
	{ "ad5689",   (kernel_ulong_t)&ad5689_chip_info },
	{ "ad5689r",  (kernel_ulong_t)&ad5689r_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5686_spi_id);

static const struct of_device_id ad5686_of_match[] = {
	{ .compatible = "adi,ad5310r", .data = &ad5310r_chip_info },
	{ .compatible = "adi,ad5313r", .data = &ad5338r_chip_info },
	{ .compatible = "adi,ad5317r", .data = &ad5317r_chip_info },
	{ .compatible = "adi,ad5672r", .data = &ad5672r_chip_info },
	{ .compatible = "adi,ad5674",  .data = &ad5674_chip_info },
	{ .compatible = "adi,ad5674r", .data = &ad5674r_chip_info },
	{ .compatible = "adi,ad5676",  .data = &ad5676_chip_info },
	{ .compatible = "adi,ad5676r", .data = &ad5676r_chip_info },
	{ .compatible = "adi,ad5679",  .data = &ad5679_chip_info },
	{ .compatible = "adi,ad5679r", .data = &ad5679r_chip_info },
	{ .compatible = "adi,ad5681r", .data = &ad5681r_chip_info },
	{ .compatible = "adi,ad5682r", .data = &ad5682r_chip_info },
	{ .compatible = "adi,ad5683",  .data = &ad5683_chip_info },
	{ .compatible = "adi,ad5683r", .data = &ad5683r_chip_info },
	{ .compatible = "adi,ad5684",  .data = &ad5684_chip_info },
	{ .compatible = "adi,ad5684r", .data = &ad5684r_chip_info },
	{ .compatible = "adi,ad5685r", .data = &ad5685r_chip_info },
	{ .compatible = "adi,ad5686",  .data = &ad5686_chip_info },
	{ .compatible = "adi,ad5686r", .data = &ad5686r_chip_info },
	{ .compatible = "adi,ad5687",  .data = &ad5687_chip_info },
	{ .compatible = "adi,ad5687r", .data = &ad5687r_chip_info },
	{ .compatible = "adi,ad5689",  .data = &ad5689_chip_info },
	{ .compatible = "adi,ad5689r", .data = &ad5689r_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5686_of_match);

static struct spi_driver ad5686_spi_driver = {
	.driver = {
		.name = "ad5686",
		.of_match_table = ad5686_of_match,
	},
	.probe = ad5686_spi_probe,
	.id_table = ad5686_spi_id,
};

module_spi_driver(ad5686_spi_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5686 and similar multi-channel DACs");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS("IIO_AD5686");
