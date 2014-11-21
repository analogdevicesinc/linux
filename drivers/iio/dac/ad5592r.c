/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2014 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include "ad5592r.h"

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/spi/spi.h>

static int ad5592r_dac_write(struct ad5592r_state *st, unsigned chan, u16 value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	u16 msg = cpu_to_be16(BIT(15) | (chan << 12) | value);

	return spi_write(spi, &msg, sizeof(msg));
}

static int ad5592r_adc_read(struct ad5592r_state *st, unsigned chan, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	u16 msg = cpu_to_be16((AD5592R_REG_ADC_SEQ << 11) | BIT(chan));
	int ret = spi_write(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	spi_read(spi, &msg, sizeof(msg)); /* Invalid data */

	ret = spi_read(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	*value = be16_to_cpu(msg);
	return 0;
}

static int ad5592r_reg_write(struct ad5592r_state *st, u8 reg, u16 value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	u16 msg = cpu_to_be16((reg << 11) | value);
	return spi_write(spi, &msg, sizeof(msg));
}

static int ad5592r_reg_read(struct ad5592r_state *st, u8 reg, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	u16 msg = cpu_to_be16((AD5592R_REG_LDAC << 11) | BIT(6) | (reg << 2));
	int ret = spi_write(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	ret = spi_read(spi, &msg, sizeof(msg));
	if (ret)
		return ret;

	if (value)
		*value = be16_to_cpu(msg);
	return 0;
}

static const struct ad5592r_rw_ops ad5592r_rw_ops = {
	.dac_write = ad5592r_dac_write,
	.adc_read = ad5592r_adc_read,
	.reg_write = ad5592r_reg_write,
	.reg_read = ad5592r_reg_read,
};

static int ad5592r_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);
	return ad5592r_probe(&spi->dev, id->driver_data,
			id->name, &ad5592r_rw_ops);
}

static int ad5592r_spi_remove(struct spi_device *spi)
{
	return ad5592r_remove(&spi->dev);
}

static const struct spi_device_id ad5592r_spi_ids[] = {
	{"ad5592r", ID_AD5592R},
	{}
};
MODULE_DEVICE_TABLE(spi, ad5592r_spi_ids);

static struct spi_driver ad5592r_spi_driver = {
	.driver = {
		   .name = "ad5592r",
		   .owner = THIS_MODULE,
	},
	.probe = ad5592r_spi_probe,
	.remove = ad5592r_spi_remove,
	.id_table = ad5592r_spi_ids,
};
module_spi_driver(ad5592r_spi_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
