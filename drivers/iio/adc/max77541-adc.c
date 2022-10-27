// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2022 Analog Devices, Inc.
 * ADI MAX77541 ADC Driver with IIO interface
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/bitfield.h>
#include <linux/iio/iio.h>
#include <linux/mfd/max77541.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/of_regulator.h>

#define MAX77541_ADC_CHANNEL(_channel, _name, _type, _reg) \
	{							\
		.type = _type,					\
		.indexed = 1,					\
		.channel = _channel,				\
		.address = _reg,				\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |	\
				      BIT(IIO_CHAN_INFO_SCALE) |\
				      BIT(IIO_CHAN_INFO_OFFSET),\
		.datasheet_name = _name,			\
	}

enum {
	MAX77541_ADC_CH1_I = 0,
	MAX77541_ADC_CH2_I,
	MAX77541_ADC_CH3_I,
	MAX77541_ADC_CH6_I,

	MAX77541_ADC_IRQMAX_I,
};

struct max77541_adc_iio {
	struct regmap	*regmap;
	int irq;
	int irq_arr[MAX77541_ADC_IRQMAX_I];
};

enum max77541_adc_channel {
	MAX77541_ADC_VSYS_V = 0,
	MAX77541_ADC_VOUT1_V,
	MAX77541_ADC_VOUT2_V,
	MAX77541_ADC_TEMP,
};

static int max77541_adc_offset(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int *val, int *val2)
{
	switch (chan->channel) {
	case MAX77541_ADC_VSYS_V:
	case MAX77541_ADC_VOUT1_V:
	case MAX77541_ADC_VOUT2_V:
		*val = 0;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77541_ADC_TEMP:
		*val = -273;
		*val2 = 0;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int max77541_adc_scale(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val, int *val2)
{
	struct max77541_adc_iio *info = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	switch (chan->channel) {
	case MAX77541_ADC_VSYS_V:
		*val = 0;
		*val2 = 25000;

		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77541_ADC_VOUT1_V:
		ret = regmap_read(info->regmap, MAX77541_REG_M2_CFG1, &reg_val);
		if (ret)
			return ret;
		reg_val = FIELD_GET(MAX77541_BITS_MX_CFG1_RNG, reg_val);

		*val = 0;

		if (reg_val == LOW_RANGE)
			*val2 = 6250;
		else if (reg_val == MID_RANGE)
			*val2 = 12500;
		else if (reg_val == HIGH_RANGE)
			*val2 = 25000;
		else
			return -EINVAL;

		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77541_ADC_VOUT2_V:
		ret = regmap_read(info->regmap, MAX77541_REG_M2_CFG1, &reg_val);
		if (ret)
			return ret;
		reg_val = FIELD_GET(MAX77541_BITS_MX_CFG1_RNG, reg_val);

		*val = 0;

		if (reg_val == LOW_RANGE)
			*val2 = 6250;
		else if (reg_val == MID_RANGE)
			*val2 = 12500;
		else if (reg_val == HIGH_RANGE)
			*val2 = 25000;
		else
			return -EINVAL;

		return IIO_VAL_INT_PLUS_MICRO;
	case MAX77541_ADC_TEMP:
		*val = 1;
		*val2 = 725000;
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static int max77541_adc_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val)
{
	struct max77541_adc_iio *info = iio_priv(indio_dev);
	int ret;
	unsigned int reg_val;

	ret = regmap_read(info->regmap, chan->address, &reg_val);
	if (ret)
		return ret;

	*val = reg_val;

	return IIO_VAL_INT;
}

static const struct iio_chan_spec max77541_adc_channels[] = {
	MAX77541_ADC_CHANNEL(MAX77541_ADC_VSYS_V, "vsys_v", IIO_VOLTAGE,
			     MAX77541_REG_ADC_DATA_CH1),
	MAX77541_ADC_CHANNEL(MAX77541_ADC_VOUT1_V, "vout1_v", IIO_VOLTAGE,
			     MAX77541_REG_ADC_DATA_CH2),
	MAX77541_ADC_CHANNEL(MAX77541_ADC_VOUT2_V, "vout2_v", IIO_VOLTAGE,
			     MAX77541_REG_ADC_DATA_CH3),
	MAX77541_ADC_CHANNEL(MAX77541_ADC_TEMP, "temp", IIO_TEMP,
			     MAX77541_REG_ADC_DATA_CH6),
};

static int max77541_adc_read_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OFFSET:
		return max77541_adc_offset(indio_dev, chan, val, val2);

	case IIO_CHAN_INFO_SCALE:
		return max77541_adc_scale(indio_dev, chan, val, val2);

	case IIO_CHAN_INFO_RAW:
		return max77541_adc_raw(indio_dev, chan, val);

	default:
		return -EINVAL;
	}
}

static const struct iio_info max77541_adc_info = {
	.read_raw = max77541_adc_read_raw,
};

static int max77541_adc_probe(struct platform_device *pdev)
{
	struct max77541_adc_iio *info;
	struct iio_dev *indio_dev;
	struct max77541_dev *max77541;
	int ret;

	max77541 = dev_get_drvdata(pdev->dev.parent);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*info));
	if (!indio_dev)
		return -ENOMEM;

	info = iio_priv(indio_dev);

	info->regmap = max77541->regmap;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->name = platform_get_device_id(pdev)->name;
	indio_dev->info = &max77541_adc_info;
	indio_dev->channels = max77541_adc_channels;
	indio_dev->num_channels = ARRAY_SIZE(max77541_adc_channels);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
			"could not register the iio/adc device\n");

	return ret;
}

static const struct platform_device_id max77541_adc_platform_id[] = {
	{ "max77541-adc", MAX77541, },
	{  /* sentinel */  }
};
MODULE_DEVICE_TABLE(platform, max77541_adc_platform_id);

static const struct of_device_id max77541_adc_of_id[] = {
	{
		.compatible = "adi,max77541-adc",
		.data = (void *)MAX77541,
	},
	{  /* sentinel */  }
};
MODULE_DEVICE_TABLE(of, max77541_adc_of_id);

static struct platform_driver max77541_adc_driver = {
	.driver = {
		.name = "max77541-adc",
		.of_match_table = max77541_adc_of_id,
	},
	.probe = max77541_adc_probe,
	.id_table = max77541_adc_platform_id,
};

module_platform_driver(max77541_adc_driver);

MODULE_AUTHOR("Okan Sahin <Okan.Sahin@analog.com>");
MODULE_DESCRIPTION("MAX77541 ADC driver");
MODULE_LICENSE("GPL");
