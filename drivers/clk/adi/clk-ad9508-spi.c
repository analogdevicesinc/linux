// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9508 1.65 GHz Clock Fanout Buffer, SPI bus driver
 *
 * Copyright 2026 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#include "clk-ad9508.h"

/*
 * 16-bit instruction word: R/W, byte count minus one, then the address of
 * the first byte transferred. In MSB-first mode, the default, the address
 * is decremented after each byte, so a multi-byte register is addressed by
 * its most significant byte and its data is sent MSB first.
 */
#define AD9508_SPI_READ			BIT(15)
#define AD9508_SPI_LEN_MSK		GENMASK(14, 13)
#define AD9508_SPI_ADDR_MSK		GENMASK(12, 0)
#define AD9508_SPI_MAX_LEN		3

static u16 ad9508_spi_instr(unsigned int reg, unsigned int len)
{
	return FIELD_PREP(AD9508_SPI_LEN_MSK, len - 1) |
	       FIELD_PREP(AD9508_SPI_ADDR_MSK, reg + len - 1);
}

static int ad9508_spi_read(void *context, unsigned int reg, unsigned int len,
			   unsigned int *val)
{
	struct spi_device *spi = context;
	u8 rx[AD9508_SPI_MAX_LEN];
	unsigned int i;
	__be16 instr;
	int ret;

	if (len > AD9508_SPI_MAX_LEN)
		return -EINVAL;

	instr = cpu_to_be16(AD9508_SPI_READ | ad9508_spi_instr(reg, len));

	ret = spi_write_then_read(spi, &instr, sizeof(instr), rx, len);
	if (ret)
		return ret;

	*val = 0;
	for (i = 0; i < len; i++)
		*val = *val << 8 | rx[i];

	return 0;
}

static int ad9508_spi_write(void *context, unsigned int reg, unsigned int len,
			    unsigned int val)
{
	struct spi_device *spi = context;
	u8 tx[sizeof(__be16) + AD9508_SPI_MAX_LEN];
	unsigned int i;

	if (len > AD9508_SPI_MAX_LEN)
		return -EINVAL;

	put_unaligned_be16(ad9508_spi_instr(reg, len), tx);
	for (i = 0; i < len; i++)
		tx[sizeof(__be16) + i] = val >> (8 * (len - 1 - i));

	return spi_write_then_read(spi, tx, sizeof(__be16) + len, NULL, 0);
}

static const struct ad9508_bus_ops ad9508_spi_bus_ops = {
	.read = ad9508_spi_read,
	.write = ad9508_spi_write,
};

static int ad9508_spi_probe(struct spi_device *spi)
{
	return ad9508_probe(&spi->dev, &ad9508_spi_bus_ops, spi,
			    !(spi->mode & SPI_3WIRE));
}

static const struct of_device_id ad9508_spi_of_match[] = {
	{ .compatible = "adi,ad9508" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9508_spi_of_match);

static const struct spi_device_id ad9508_spi_id[] = {
	{ "ad9508" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad9508_spi_id);

static struct spi_driver ad9508_spi_driver = {
	.driver = {
		.name = "ad9508",
		.of_match_table = ad9508_spi_of_match,
	},
	.probe = ad9508_spi_probe,
	.id_table = ad9508_spi_id,
};
module_spi_driver(ad9508_spi_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9508 clock fanout buffer SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("CLK_AD9508");
