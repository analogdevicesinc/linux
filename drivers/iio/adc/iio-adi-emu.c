// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * IIO Analog Devices, Inc. Emulator Driver
 *
 * Copyright (C) 2022 Analog Devices, Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>

#define ADI_EMU_REG_DEVICE_CONFIG	0x02
#define  ADI_EMU_MASK_POWER_DOWN	BIT(5)
#define ADI_EMU_REG_CNVST		0x03
#define  ADI_EMU_MASK_CNVST		BIT(0)
#define ADI_EMU_REG_CH0_DATA_HIGH	0x04
#define ADI_EMU_REG_CH0_DATA_LOW	0x05
#define ADI_EMU_REG_CH1_DATA_HIGH	0x06
#define ADI_EMU_REG_CH1_DATA_LOW	0x07

struct adi_emu_state {
	bool		enable;
	struct regmap	*regmap;
};

static int adi_emu_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct adi_emu_state *st = iio_priv(indio_dev);
	unsigned int high, low;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = st->enable;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ret = regmap_write(st->regmap, ADI_EMU_REG_CNVST, ADI_EMU_MASK_CNVST);
		if (ret)
			return ret;

		if (chan->channel) {
			ret = regmap_read(st->regmap, ADI_EMU_REG_CH1_DATA_HIGH,
					  &high);
			if (ret)
				return ret;

			ret = regmap_read(st->regmap, ADI_EMU_REG_CH1_DATA_LOW,
					  &low);
			if (ret)
				return ret;
		} else {
			ret = regmap_read(st->regmap, ADI_EMU_REG_CH0_DATA_HIGH,
					  &high);
			if (ret)
				return ret;

			ret = regmap_read(st->regmap, ADI_EMU_REG_CH0_DATA_LOW,
					  &low);
			if (ret)
				return ret;
		}

		*val = (high << 8) | low;
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int adi_emu_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct adi_emu_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		if (val)
			ret = regmap_write(st->regmap, ADI_EMU_REG_DEVICE_CONFIG,
					   0);
		else
			ret = regmap_write(st->regmap, ADI_EMU_REG_DEVICE_CONFIG,
					   ADI_EMU_MASK_POWER_DOWN);

		st->enable = val;
		return ret;
	}

	return -EINVAL;
}

static int adi_emu_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct adi_emu_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info adi_emu_info = {
	.read_raw = &adi_emu_read_raw,
	.write_raw = &adi_emu_write_raw,
	.debugfs_reg_access = &adi_emu_reg_access,
};

static const struct iio_chan_spec adi_emu_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0,
		.indexed = 1,
		.channel = 0,
	},
	{
		.type = IIO_VOLTAGE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 0,
		.indexed = 1,
		.channel = 1,
	}
};

static const struct regmap_config adi_emu_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x8,
};

static int adi_emu_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct adi_emu_state *st;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->enable = false;
	st->regmap = devm_regmap_init_spi(spi, &adi_emu_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	indio_dev->name = "iio-adi-emu";
	indio_dev->channels = adi_emu_channels;
	indio_dev->num_channels = ARRAY_SIZE(adi_emu_channels);
	indio_dev->info = &adi_emu_info;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver adi_emu_driver = {
	.driver = {
		.name = "iio-adi-emu",
	},
	.probe = adi_emu_probe,
};
module_spi_driver(adi_emu_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("IIO ADI Emulator Driver");
MODULE_LICENSE("GPL v2");

