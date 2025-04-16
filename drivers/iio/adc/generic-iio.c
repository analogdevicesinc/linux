// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Generic IIO Analog Devices, Inc. Driver
 *
 * Copyright (C) 2025 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>

#define READ_FLAG_MASK	0x8

#define REG_RUN		0x0480
#define REG_VERSION	0x04FE
 
struct gen_iio_priv {
	struct spi_device *spi;
};

int gen_iio_read(struct iio_dev *indio_dev, unsigned addr, unsigned *val)
{
	struct gen_iio_priv *priv = iio_priv(indio_dev);
	unsigned char buff[8];
	unsigned n_rx;
	int ret;
 
	buff[0] = addr >> 4;
	buff[1] = (addr << 4) & 0xFF;
	buff[1] |= READ_FLAG_MASK;
 
	if (addr >= REG_RUN && addr <= REG_VERSION)
		n_rx = 2;	/* 16 bits registers */
	else
		n_rx = 4;	/* 32 bits registers */

	ret = spi_write_then_read(priv->spi, &buff[0], 2, &buff[2], n_rx);
	if (ret < 0)
		return ret;

	if (addr >= REG_RUN && addr <= REG_VERSION)
		*val = (buff[2] << 8) | (buff[3] << 0);
	else
		*val = (buff[2] << 24) | (buff[3] << 16) | (buff[4] << 8) | (buff[5] << 0);
 
	 return ret;
}

int gen_iio_write(struct iio_dev *indio_dev, unsigned addr, unsigned val)
{
	struct gen_iio_priv *priv = iio_priv(indio_dev);
	unsigned char buff[6];
	unsigned n_tx;
 
	buff[0] = (addr >> 4) & 0xFF;
	buff[1] = (addr << 4) & 0xFF;
 
	if (addr >= REG_RUN && addr <= REG_VERSION) {
		n_tx = 4;	/* 16 bits registers */
		buff[2] = (val >> 8) & 0xFF;
		buff[3] = (val >> 0) & 0xFF;
	} else {
		n_tx = 6;	/* 32 bits registers */
		buff[2] = (val >> 24) & 0xFF;
		buff[3] = (val >> 16) & 0xFF;
		buff[4] = (val >> 8) & 0xFF;
		buff[5] = (val >> 0) & 0xFF;
	}
 
	 return spi_write_then_read(priv->spi, &buff[0], n_tx, NULL, 0);
}

static int gen_iio_reg_access(struct iio_dev *indio_dev,
	unsigned reg, unsigned writeval,
	unsigned *readval)
{
	if (readval)
		return gen_iio_read(indio_dev, reg, readval);
	else
		return gen_iio_write(indio_dev, reg, writeval);
}

static const struct iio_info gen_iio_info = {
	.debugfs_reg_access = &gen_iio_reg_access,
};

static int gen_iio_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct gen_iio_priv *priv;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->spi = spi;

	indio_dev->name = "generic-iio";
	indio_dev->info = &gen_iio_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}
 
static struct spi_driver gen_iio_driver = {
	.driver = {
		.name = "generic-iio",
	},
	.probe = gen_iio_probe,
};
module_spi_driver(gen_iio_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Generic IIO ADI Driver");
MODULE_LICENSE("GPL v2");