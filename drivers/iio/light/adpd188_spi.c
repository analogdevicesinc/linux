// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADPD188 SPI driver
 *
 * Copyright 2021 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

#include "adpd188.h"

static int adpd188_reg_write(void *bus, int addr, int value)
{
	struct spi_device *spi = bus;
	u8 buff[3];
	struct spi_message m;
	struct spi_transfer t = {0};

	buff[0] = (addr << 1) | 1;
	buff[1] = value >> 8;
	buff[2] = value & 0xff;

	t.tx_buf = buff;
	t.len = 3;

	spi_message_init_with_transfers(&m, &t, 1);

	return spi_sync(spi, &m);
}

static int adpd188_reg_read(void *bus, int addr, int *value)
{
	struct spi_device *spi = bus;
	uint8_t buff[3];
	struct spi_message m;
	struct spi_transfer t = {0};
	int ret;

	buff[0] = addr << 1;

	t.rx_buf = buff;
	t.tx_buf = buff;
	t.len = 3;

	spi_message_init_with_transfers(&m, &t, 1);

	ret = spi_sync(spi, &m);
	if (ret < 0)
		return ret;

	*value = (buff[1] << 8) | buff[2];

	return ret;
}

static int adpd188_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	struct adpd188_ops *phy;

	phy = devm_kzalloc(&spi->dev, sizeof(*phy), GFP_KERNEL);
	if (!phy)
		return -ENOMEM;
	phy->reg_write = adpd188_reg_write;
	phy->reg_read = adpd188_reg_read;

	return adpd188_core_probe(spi, phy, ADPD188_SPI, 1, id->name, spi->irq);
}

static const struct spi_device_id adpd188_spi_id[] = {
	{ "adpd188", ADPD188 },
	{}
};
MODULE_DEVICE_TABLE(spi, adpd188_spi_id);

static const struct of_device_id adpd188_of_match[] = {
	{ .compatible = "adi,adpd188" },
	{},
};
MODULE_DEVICE_TABLE(of, adpd188_of_match);

static struct spi_driver adpd188_spi_driver = {
	.driver = {
			.name = "adpd188_spi",
			.of_match_table = adpd188_of_match,
		},
	.probe = adpd188_spi_probe,
	.id_table = adpd188_spi_id,
};
module_spi_driver(adpd188_spi_driver);

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188 SPI driver");
MODULE_LICENSE("GPL v2");

