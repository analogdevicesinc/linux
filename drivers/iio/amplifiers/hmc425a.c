// SPDX-License-Identifier: GPL-2.0
/*
 * hmc425a and similar Gain Amplifiers
 *
 * Copyright 2019 Analog Devices Inc.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>

#define HMC425A_NR_GPIOS	6

enum hmc425_type {
	ID_HMC425 ,
};

struct hmc425_info {
	int gain_min;
	int gain_max;
};

struct hmc425_state {
	struct regulator *reg;
	struct mutex lock; /* protect sensor state */
	struct hmc425_info *info;
	struct gpio_descs *gpios;
	enum hmc425_type type;
	u32 value;
};

static struct hmc425_info hmc425_infos[] = {
	[ID_HMC425] = {
		.gain_min = -31500,
		.gain_max = 0,
	},
};

static int hmc425_write(struct iio_dev *indio_dev, u32 value)
{
	struct hmc425_state *st = iio_priv(indio_dev);
	int i, values[HMC425A_NR_GPIOS];

	for (i = 0; i < st->gpios->ndescs; i++)
		values[i] = (value >> i) & 1;

	gpiod_set_array_value_cansleep(st->gpios->ndescs, st->gpios->desc,
				       values);

	return 0;
}

static int hmc425_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long m)
{
	struct hmc425_state *st = iio_priv(indio_dev);
	int ret;
	int code, gain = 0;

	mutex_lock(&st->lock);
	switch (m) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		code = st->value;

		switch (st->type) {
		case ID_HMC425:
			gain = ~code * -500;
			break;
		}

		/* Values in dB */
		*val = gain / 1000;
		*val2 = (gain % 1000) * 1000;

		ret = IIO_VAL_INT_PLUS_MICRO_DB;
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
};

static int hmc425_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	struct hmc425_state *st = iio_priv(indio_dev);
	struct hmc425_info *inf = st->info;
	int code = 0, gain;
	int ret;

	/* Values in dB */
	if (val < 0)
		gain = (val * 1000) - (val2 / 1000);
	else
		gain = (val * 1000) + (val2 / 1000);

	if (gain > inf->gain_max || gain < inf->gain_min)
		return -EINVAL;

	switch (st->type) {
	case ID_HMC425:
		code = ~((abs(gain) / 500) & 0x3F);
		break;
	}

	mutex_lock(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		st->value = code;

		ret = hmc425_write(indio_dev, st->value);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&st->lock);

	return ret;
}

static const struct iio_info hmc425_info = {
	.read_raw = &hmc425_read_raw,
	.write_raw = &hmc425_write_raw,
};

#define hmc425_CHAN(_channel)                                          \
{                                                                      \
	.type = IIO_VOLTAGE, .output = 1, .indexed = 1,                \
	.channel = _channel,                                           \
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),         \
}

static const struct iio_chan_spec hmc425_channels[] = {
	hmc425_CHAN(0),
};

/* Match table for of_platform binding */
static const struct of_device_id hmc425_of_match[] = {
	{ .compatible = "adi,hmc425a", .data = (void *) ID_HMC425 },
	{},
};
MODULE_DEVICE_TABLE(of, hmc425_of_match);

static int hmc425_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	const struct of_device_id *id;
	struct hmc425_state *st;
	struct device_node *np = pdev->dev.of_node;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->gpios = devm_gpiod_get_array(&pdev->dev, "ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpios)) {
		ret = PTR_ERR(st->gpios);
		if (ret != -EPROBE_DEFER)
			dev_err(&pdev->dev, "failed to get gpios\n");
		return ret;
	}

	if (st->gpios->ndescs != HMC425A_NR_GPIOS) {
		dev_err(&pdev->dev, "%d GPIOs needed to operate\n",
			HMC425A_NR_GPIOS);
		return -ENODEV;
	}

	st->reg = devm_regulator_get(&pdev->dev, "vcc");
	if (!IS_ERR(st->reg)) {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
	}

	platform_set_drvdata(pdev, indio_dev);
	mutex_init(&st->lock);

	id = of_match_device(hmc425_of_match, &pdev->dev);
	if (!id) {
		ret = -ENODEV;
		goto error_disable_reg;
	}

	st->type = (enum hmc425_type)id->data;

	switch (st->type) {
	case ID_HMC425:
		indio_dev->channels = hmc425_channels;
		indio_dev->num_channels = ARRAY_SIZE(hmc425_channels);
		st->value = 0x3F;
		break;
	default:
		dev_err(&pdev->dev, "Invalid device ID\n");
		ret = -EINVAL;
		goto error_disable_reg;
	}

	st->info = &hmc425_infos[st->type];
	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->info = &hmc425_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	if (!IS_ERR(st->reg))
		regulator_disable(st->reg);

	return ret;
}

static int hmc425_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct hmc425_state *st = iio_priv(indio_dev);
	struct regulator *reg = st->reg;

	iio_device_unregister(indio_dev);

	if (!IS_ERR(reg))
		regulator_disable(reg);

	return 0;
}

static struct platform_driver hmc425_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = hmc425_of_match,
	},
	.probe = hmc425_probe,
	.remove = hmc425_remove,
};
module_platform_driver(hmc425_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION(
	"Analog Devices HMC425A and similar GPIO control Gain Amplifiers");
MODULE_LICENSE("GPL v2");
