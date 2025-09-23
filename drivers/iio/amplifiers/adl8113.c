// SPDX-License-Identifier: GPL-2.0
/*
 * ADL8113 Low Noise Amplifier with integrated bypass switches
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/array_size.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>

enum adl8113_signal_path {
	ADL8113_INTERNAL_AMP,
	ADL8113_INTERNAL_BYPASS,
	ADL8113_EXTERNAL_A,
	ADL8113_EXTERNAL_B,
};

struct adl8113_gain_config {
	enum adl8113_signal_path path;
	int gain_db;
	int va;
	int vb;
};

struct adl8113_state {
	struct gpio_descs *gpios;
	struct adl8113_gain_config *gain_configs;
	int num_gain_configs;
	enum adl8113_signal_path current_path;
};

static const char * const adl8113_supply_names[] = {
	"vdd1",
	"vss2",
	"vdd2"
};

static int adl8113_set_path(struct adl8113_state *st,
			    enum adl8113_signal_path path)
{
	DECLARE_BITMAP(values, 2);
	int ret;

	bitmap_zero(values, 2);

	/* Determine GPIO values based on signal path */
	switch (path) {
	case ADL8113_INTERNAL_AMP:
		/* va=0, vb=0 - already zero */
		break;
	case ADL8113_INTERNAL_BYPASS:
		/* va=1, vb=1 */
		__set_bit(0, values);
		__set_bit(1, values);
		break;
	case ADL8113_EXTERNAL_A:
		/* va=0, vb=1 */
		__set_bit(1, values);
		break;
	case ADL8113_EXTERNAL_B:
		/* va=1, vb=0 */
		__set_bit(0, values);
		break;
	default:
		return -EINVAL;
	}

	ret = gpiod_set_array_value_cansleep(st->gpios->ndescs, st->gpios->desc,
					     st->gpios->info, values);
	if (ret)
		return ret;

	st->current_path = path;
	return 0;
}

static int adl8113_find_gain_config(struct adl8113_state *st, int gain_db)
{
	int i;

	for (i = 0; i < st->num_gain_configs; i++) {
		if (st->gain_configs[i].gain_db == gain_db)
			return i;
	}
	return -EINVAL;
}

static const struct iio_chan_spec adl8113_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
	},
};

static int adl8113_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adl8113_state *st = iio_priv(indio_dev);
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		/* Find current gain configuration */
		for (i = 0; i < st->num_gain_configs; i++) {
			if (st->gain_configs[i].path == st->current_path) {
				*val = st->gain_configs[i].gain_db;
				*val2 = 0;
				return IIO_VAL_INT_PLUS_MICRO_DB;
			}
		}
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static int adl8113_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adl8113_state *st = iio_priv(indio_dev);
	int config_idx;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val2 != 0)
			return -EINVAL;

		config_idx = adl8113_find_gain_config(st, val);
		if (config_idx < 0)
			return config_idx;

		return adl8113_set_path(st, st->gain_configs[config_idx].path);
	default:
		return -EINVAL;
	}
}

static const struct iio_info adl8113_info = {
	.read_raw = adl8113_read_raw,
	.write_raw = adl8113_write_raw,
};

static int adl8113_init_gain_configs(struct device *dev, struct adl8113_state *st)
{
	int external_a_gain, external_b_gain, i = 0, j;

	/*
	 * Allocate for all 4 possible paths:
	 * - Internal amp and bypass (always present)
	 * - External bypass A and B (optional, or INT_MIN for testing)
	 */
	st->gain_configs = devm_kcalloc(dev, 4,
					sizeof(*st->gain_configs), GFP_KERNEL);
	if (!st->gain_configs)
		return -ENOMEM;

	/* Always include internal amplifier (14dB) */
	st->gain_configs[i].path = ADL8113_INTERNAL_AMP;
	st->gain_configs[i].gain_db = 14;
	i++;

	/* Always include internal bypass (-2dB insertion loss) */
	st->gain_configs[i].path = ADL8113_INTERNAL_BYPASS;
	st->gain_configs[i].gain_db = -2;
	i++;

	/* Add external bypass A if configured */
	if (!device_property_read_u32(dev, "adi,external-bypass-a-gain-db",
				      &external_a_gain)) {
		st->gain_configs[i].path = ADL8113_EXTERNAL_A;
		st->gain_configs[i].gain_db = external_a_gain;
		i++;
	}

	/* Add external bypass B if configured */
	if (!device_property_read_u32(dev, "adi,external-bypass-b-gain-db",
				      &external_b_gain)) {
		st->gain_configs[i].path = ADL8113_EXTERNAL_B;
		st->gain_configs[i].gain_db = external_b_gain;
		i++;
	}

	/*
	 * If there's a free external bypass path, add one with INT_MIN gain
	 * to represent "nothing connected" for testing purposes
	 */
	if (!device_property_present(dev, "adi,external-bypass-a-gain-db")) {
		st->gain_configs[i].path = ADL8113_EXTERNAL_A;
		st->gain_configs[i].gain_db = INT_MIN;
		i++;
	} else if (!device_property_present(dev, "adi,external-bypass-b-gain-db")) {
		st->gain_configs[i].path = ADL8113_EXTERNAL_B;
		st->gain_configs[i].gain_db = INT_MIN;
		i++;
	}

	st->num_gain_configs = i;

	/* Check for duplicate gain values */
	for (i = 0; i < st->num_gain_configs - 1; i++) {
		for (j = i + 1; j < st->num_gain_configs; j++) {
			if (st->gain_configs[i].gain_db == st->gain_configs[j].gain_db)
				return dev_err_probe(dev, -EINVAL,
						     "Duplicate gain values not allowed: %d dB\n",
						     st->gain_configs[i].gain_db);
		}
	}

	return 0;
}

static int adl8113_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adl8113_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->gpios = devm_gpiod_get_array(dev, "ctrl", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpios))
		return dev_err_probe(dev, PTR_ERR(st->gpios),
				     "failed to get control GPIOs\n");

	if (st->gpios->ndescs != 2)
		return dev_err_probe(dev, -EINVAL,
				     "expected 2 control GPIOs, got %d\n",
				     st->gpios->ndescs);

	ret = devm_regulator_bulk_get_enable(dev,
					     ARRAY_SIZE(adl8113_supply_names),
					     adl8113_supply_names);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to get and enable supplies\n");

	/* Initialize gain configurations from devicetree */
	ret = adl8113_init_gain_configs(dev, st);
	if (ret)
		return ret;

	/* Initialize to internal amplifier path (14dB) */
	ret = adl8113_set_path(st, ADL8113_INTERNAL_AMP);
	if (ret)
		return ret;

	indio_dev->info = &adl8113_info;
	indio_dev->name = "adl8113";
	indio_dev->channels = adl8113_channels;
	indio_dev->num_channels = ARRAY_SIZE(adl8113_channels);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id adl8113_of_match[] = {
	{ .compatible = "adi,adl8113" },
	{ }
};
MODULE_DEVICE_TABLE(of, adl8113_of_match);

static struct platform_driver adl8113_driver = {
	.driver = {
		.name = "adl8113",
		.of_match_table = adl8113_of_match,
	},
	.probe = adl8113_probe,
};

module_platform_driver(adl8113_driver);

MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADL8113 Low Noise Amplifier");
MODULE_LICENSE("GPL");
