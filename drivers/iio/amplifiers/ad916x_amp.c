// SPDX-License-Identifier: GPL-2.0
/*
 * SPI Amplifier Driver for the A916x series
 *
 * Copyright 2019 Analog Devices Inc.
 *
 */
#include <linux/device.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>

#define AD916X_AMP_REG(x)	x
#define AD916X_AMP_ENABLE	0x00
#define AD916X_AMP_DISABLE	0x3B

struct ad916x_amp_state {
	struct regmap *map;
	struct spi_device *spi;
};

static const struct regmap_config ad916x_amp_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = 0x80,
};

static int ad916x_amp_read_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val,
			       int *val2,
			       long mask)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_read(st->map, AD916X_AMP_REG(0x10), val);
		if (ret)
			return ret;

		*val = !(*val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad916x_amp_write_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int val,
				int val2,
				long mask)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val != 0 && val != 1)
			return -EINVAL;

		return regmap_write(st->map, AD916X_AMP_REG(0x10),
				    val == 1 ? AD916X_AMP_ENABLE :
				    AD916X_AMP_DISABLE);
	default:
		return -EINVAL;
	}
}

static int ad916x_amp_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				 unsigned int writeval, unsigned int *readval)
{
	struct ad916x_amp_state *st = iio_priv(indio_dev);

	if (!readval)
		return regmap_write(st->map, reg, writeval);
	else
		return regmap_read(st->map, reg, readval);
}

static const struct iio_info ad916x_amp_info = {
	.read_raw = ad916x_amp_read_raw,
	.write_raw = ad916x_amp_write_raw,
	.debugfs_reg_access = &ad916x_amp_reg_access,
};

#define AD916X_AMP_CHAN(index)	{ \
	.type = IIO_ALTVOLTAGE, \
	.indexed = 1, \
	.channel = index, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE), \
	.output = 1, \
}

static struct iio_chan_spec ad916x_amp_chan_spec[] = {
	AD916X_AMP_CHAN(0),
};

static void ad916x_amp_setup(const struct ad916x_amp_state *st)
{
	if (!(st->spi->mode & SPI_3WIRE))
		/* set SPI to 4-wire mode */
		regmap_write(st->map, AD916X_AMP_REG(0x00), 0x18);
	/*
	 * TODO: these values are being hardcoded because there is no (still)
	 * information about the register map on the amplifier. This values
	 * were taken from the ACE plugin logs for a DAC FSC of 40mA. As soon
	 * as we know how to derivate this values from the fsc we have to make
	 * this configurable. Maybe, by having a dts phandle which links to
	 * the DAC were the fsc paremeter is defined...
	 */
	regmap_write(st->map, AD916X_AMP_REG(0x18), 0x0B);
	regmap_write(st->map, AD916X_AMP_REG(0x19), 0xa0);
}

static int ad916x_amp_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad916x_amp_state *st;
	const struct spi_device_id *dev_id = spi_get_device_id(spi);
	struct device_node *np = spi->dev.of_node;
	const char *dev_name;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev) {
		dev_err(&spi->dev, "Failed to alloc iio dev\n");
		return -ENOMEM;
	}

	st = iio_priv(indio_dev);

	st->map = devm_regmap_init_spi(spi, &ad916x_amp_regmap_config);
	if (IS_ERR(st->map))
		return PTR_ERR(st->map);

	st->spi = spi;

	dev_name = np ? np->name : dev_id->name;

	ad916x_amp_setup(st);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = dev_name;
	indio_dev->channels = ad916x_amp_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(ad916x_amp_chan_spec);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad916x_amp_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad916x_amp_dt_id[] = {
	{ .compatible = "adi,ad9166-amp" },
	{},
};
MODULE_DEVICE_TABLE(of, ad916x_amp_dt_id);

static const struct spi_device_id ad916x_amp_id[] = {
	{ "ad9166-amp", 0 },
	{},
};
MODULE_DEVICE_TABLE(spi, ad916x_amp_id);

static struct spi_driver ad916x_amp_driver = {
	.driver = {
		.name = "ad916x-amp",
		.of_match_table = ad916x_amp_dt_id,
	},
	.probe = ad916x_amp_probe,
	.id_table = ad916x_amp_id,
};

module_spi_driver(ad916x_amp_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD916x Amplifier");
MODULE_LICENSE("GPL v2");
