// SPDX-License-Identifier: GPL-2.0
/*
 * max31827.c - Support for Maxim Low-Power Switch
 *
 * Copyright (c) 2023 Daniel Matyas <daniel.matyas@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/of.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>

#define MAX31827_T_REG				0x0
#define MAX31827_CONFIGURATION_REG              0x2
#define MAX31827_TH_REG                         0x4
#define MAX31827_TL_REG                         0x6
#define MAX31827_TH_HYST_REG                    0x8
#define MAX31827_TL_HYST_REG                    0xA

#define MAX31827_CONFIGURATION_1SHOT_MASK	BIT(0)
#define MAX31827_CONFIGURATION_CNV_RATE_MASK    GENMASK(3, 1)
#define MAX31827_CONFIGURATION_RESOL_MASK       GENMASK(7, 6)
#define MAX31827_CONFIGURATION_U_TEMP_STAT_MASK BIT(14)
#define MAX31827_CONFIGURATION_O_TEMP_STAT_MASK BIT(15)

#define MAX31827_CNV_SHUTDOWN			0x0
#define MAX31827_CNV_1_DIV_64_HZ		0x1
#define MAX31827_CNV_1_DIV_32_HZ		0x2
#define MAX31827_CNV_1_DIV_16_HZ		0x3
#define MAX31827_CNV_1_DIV_4_HZ			0x4
#define MAX31827_CNV_1_HZ			0x5
#define MAX31827_CNV_4_HZ			0x6
#define MAX31827_CNV_8_HZ			0x7

#define MAX31827_1SHOT_EN(x)			((x) ? BIT(0) : 0)

struct max31827_state {
	struct regmap *regmap;
	struct i2c_client *client;
};

static const struct regmap_config max31827_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xA,
};

static umode_t max31827_is_visible(const void *state,
				   enum hwmon_sensor_types type,
				   u32 attr, int channel)
{
	if (type == hwmon_temp) {
		switch (attr) {
		case hwmon_temp_enable:
		case hwmon_temp_max:
		case hwmon_temp_min:
		case hwmon_temp_max_hyst:
		case hwmon_temp_min_hyst:
			return 0644;
		case hwmon_temp_input:
			return 0444;
		}
	}

	return 0;
}

static int max31827_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct max31827_state *st;
	unsigned int uval;
	int ret;

	st = dev_get_drvdata(dev);
	if (IS_ERR(st))
		return PTR_ERR(st);

	if (type != hwmon_temp)
		return -EOPNOTSUPP;

	switch (attr) {
	case hwmon_temp_enable:
		ret = regmap_read(st->regmap, MAX31827_CONFIGURATION_REG, &uval);
		uval  = FIELD_GET(MAX31827_CONFIGURATION_1SHOT_MASK, uval);
		break;

	case hwmon_temp_input:
		ret = regmap_read(st->regmap, MAX31827_T_REG, &uval);
		break;

	case hwmon_temp_max:
		ret = regmap_read(st->regmap, MAX31827_TH_REG, &uval);
		break;

	case hwmon_temp_max_hyst:
		ret = regmap_read(st->regmap, MAX31827_TH_HYST_REG, &uval);
		break;
	case hwmon_temp_min:
		ret = regmap_read(st->regmap, MAX31827_TL_REG, &uval);
		break;

	case hwmon_temp_min_hyst:
		ret = regmap_read(st->regmap, MAX31827_TL_HYST_REG, &uval);
		break;

	default:
		ret = -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	*val = uval;

	return 0;
}

static int max31827_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct max31827_state *st = dev_get_drvdata(dev);

	if (IS_ERR(st))
		return PTR_ERR(st);

	switch (attr) {
	case hwmon_temp_enable:
		if (val >> 1)
			return -EOPNOTSUPP;

		return regmap_update_bits(st->regmap, MAX31827_CONFIGURATION_REG,
					  MAX31827_CONFIGURATION_1SHOT_MASK,
					  MAX31827_1SHOT_EN(val));

	case hwmon_temp_max:
		return regmap_write(st->regmap, MAX31827_TH_REG, val);

	case hwmon_temp_max_hyst:
		return regmap_write(st->regmap, MAX31827_TH_HYST_REG, val);

	case hwmon_temp_min:
		return regmap_write(st->regmap, MAX31827_TL_REG, val);

	case hwmon_temp_min_hyst:
		return regmap_write(st->regmap, MAX31827_TL_HYST_REG, val);
	}

	return -EOPNOTSUPP;
}

static int max31827_init_client(struct max31827_state *st)
{
	return regmap_update_bits(st->regmap, MAX31827_CONFIGURATION_REG,
				 MAX31827_CONFIGURATION_CNV_RATE_MASK |
				 MAX31827_CONFIGURATION_1SHOT_MASK,
				 MAX31827_1SHOT_EN(1));
}

static const struct hwmon_channel_info *max31827_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_ENABLE | HWMON_T_INPUT | HWMON_T_MIN |
			   HWMON_T_MIN_HYST | HWMON_T_MAX | HWMON_T_MAX_HYST),
	NULL,
};

static const struct hwmon_ops max31827_hwmon_ops = {
	.is_visible = max31827_is_visible,
	.read = max31827_read,
	.write = max31827_write,
};

static const struct hwmon_chip_info max31827_chip_info = {
	.ops = &max31827_hwmon_ops,
	.info = max31827_info,
};

static int max31827_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct max31827_state *st;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	st = devm_kzalloc(dev, sizeof(struct max31827_state), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;

	st->regmap = devm_regmap_init_i2c(client, &max31827_regmap);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to allocate regmap: %ld\n",
				     PTR_ERR(st->regmap));

	ret = max31827_init_client(st);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, st,
							 &max31827_chip_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id max31827_i2c_ids[] = {
	{ .name = "max31827" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max31827_i2c_ids);

static const struct of_device_id max31827_of_match[] = {
	{ .compatible = "max31827" },
	{ }
};
MODULE_DEVICE_TABLE(of, max31827_of_match);

static struct i2c_driver max31827_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "max31827",
		.of_match_table = max31827_of_match,
	},
	.probe_new = max31827_probe,
	.id_table = max31827_i2c_ids,
};
module_i2c_driver(max31827_driver);

MODULE_AUTHOR("Daniel Matyas <daniel.matyas@analog.com>");
MODULE_DESCRIPTION("Maxim MAX31827 low-power temperature switch driver");
MODULE_LICENSE("GPL");
