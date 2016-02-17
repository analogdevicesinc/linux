/*
 * AD5592R Digital <-> Analog converters driver
 *
 * Copyright 2015 Analog Devices Inc.
 * Author: Paul Cercueil <paul.cercueil@analog.com>
 *
 * Licensed under the GPL-2.
 */

#include "ad5592r-base.h"

#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/spi/spi.h>

static int ad5592r_write(struct ad5592r_state *st, unsigned chan, u16 value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);

	st->spi_msg = cpu_to_be16(BIT(15) | (chan << 12) | value);

	return spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
}

static int ad5592r_read(struct ad5592r_state *st, unsigned chan, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	int ret;

	st->spi_msg = cpu_to_be16((AD5592R_REG_ADC_SEQ << 11) | BIT(chan));

	ret = spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	/* Invalid data */
	ret = spi_read(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	ret = spi_read(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	*value = be16_to_cpu(st->spi_msg);

	return 0;
}

static int ad5592r_reg_write(struct ad5592r_state *st, u8 reg, u16 value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);

	st->spi_msg = cpu_to_be16((reg << 11) | value);

	return spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
}

static int ad5592r_reg_read(struct ad5592r_state *st, u8 reg, u16 *value)
{
	struct spi_device *spi = container_of(st->dev, struct spi_device, dev);
	int ret;

	st->spi_msg = cpu_to_be16(
			(AD5592R_REG_LDAC << 11) | BIT(6) | (reg << 2));

	ret = spi_write(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	ret = spi_read(spi, &st->spi_msg, sizeof(st->spi_msg));
	if (ret)
		return ret;

	if (value)
		*value = be16_to_cpu(st->spi_msg);

	return 0;
}

static const struct ad5592r_rw_ops ad5592r_rw_ops = {
	.write = ad5592r_write,
	.read = ad5592r_read,
	.reg_write = ad5592r_reg_write,
	.reg_read = ad5592r_reg_read,
};

static int ad5592r_spi_probe(struct spi_device *spi)
{
	const struct spi_device_id *id = spi_get_device_id(spi);

	return ad5592r_probe(&spi->dev, id->name, &ad5592r_rw_ops);
}

static int ad5592r_spi_remove(struct spi_device *spi)
{
	return ad5592r_remove(&spi->dev);
}

static const struct spi_device_id ad5592r_spi_ids[] = {
	{ .name = "ad5592r", },
	{}
};
MODULE_DEVICE_TABLE(spi, ad5592r_spi_ids);

static const struct of_device_id ad5592r_of_match[] = {
	{ .compatible = "adi,ad5592r", },
	{},
};
MODULE_DEVICE_TABLE(of, ad5592r_of_match);

static struct spi_driver ad5592r_spi_driver = {
	.driver = {
		.name = "ad5592r",
		.of_match_table = of_match_ptr(ad5592r_of_match),
	},
	.probe = ad5592r_spi_probe,
	.remove = ad5592r_spi_remove,
	.id_table = ad5592r_spi_ids,
};
module_spi_driver(ad5592r_spi_driver);

MODULE_AUTHOR("Paul Cercueil <paul.cercueil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5592R multi-channel converters");
MODULE_LICENSE("GPL v2");
