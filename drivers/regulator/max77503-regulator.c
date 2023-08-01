// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2023 Analog Devices, Inc.
 * ADI driver for the MAX77503 Buck Converter
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/util_macros.h>

#define MAX77503_REG_CFG		0x00
#define MAX77503_REG_VOUT		0x01

#define MAX77503_BIT_EN			BIT(0)
#define MAX77503_BIT_CURR_LIM		BIT(3)
#define MAX77503_BIT_ADEN		BIT(6)

#define MAX77503_BITS_SOFT_START	GENMASK(5, 4)
#define MAX77503_BITS_MX_VOUT		GENMASK(7, 0)

#define MAX77503_AD_ENABLED		0x1
#define MAX77503_AD_DISABLED		0x0

struct max77503_dev {
	struct device *dev;
	struct device_node *of_node;
	struct regulator_desc desc;
	struct regulator_dev *rdev;
	struct regmap *regmap;
};

static const struct regmap_config max77503_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 8,
	.max_register		= 0x2,
};

static const struct regulator_ops max77503_buck_ops = {
		.list_voltage = regulator_list_voltage_linear_range,
		.map_voltage = regulator_map_voltage_ascend,
		.is_enabled = regulator_is_enabled_regmap,
		.enable = regulator_enable_regmap,
		.disable = regulator_disable_regmap,
		.get_voltage_sel = regulator_get_voltage_sel_regmap,
		.set_voltage_sel = regulator_set_voltage_sel_regmap,
		.get_current_limit = regulator_get_current_limit_regmap,
		.set_current_limit = regulator_set_current_limit_regmap,
		.set_active_discharge = regulator_set_active_discharge_regmap,
		.set_soft_start = regulator_set_soft_start_regmap,
};

static const struct linear_range max77503_buck_ranges[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x54, 50000)
};

static const unsigned int max77503_current_limit_table[] = {
	500000, 2000000
};

static const struct regulator_desc max77503_regulators_desc = {
		.name = "max77503",
		.enable_reg = MAX77503_REG_CFG,
		.enable_mask = MAX77503_BIT_EN,
		.ops = &max77503_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = max77503_buck_ranges,
		.n_linear_ranges = ARRAY_SIZE(max77503_buck_ranges),
		.vsel_reg = MAX77503_REG_VOUT,
		.vsel_mask = MAX77503_BITS_MX_VOUT,
		.soft_start_reg = MAX77503_REG_CFG,
		.soft_start_mask = MAX77503_BITS_SOFT_START,
		.active_discharge_reg = MAX77503_REG_CFG,
		.active_discharge_mask = MAX77503_BIT_ADEN,
		.active_discharge_off = MAX77503_AD_DISABLED,
		.active_discharge_on = MAX77503_AD_ENABLED,
		.csel_reg = MAX77503_REG_CFG,
		.csel_mask = MAX77503_BIT_CURR_LIM,
		.curr_table = max77503_current_limit_table,
		.n_current_limits = ARRAY_SIZE(max77503_current_limit_table),
		.owner = THIS_MODULE,
};

static int max77503_regulator_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = {};
	struct max77503_dev *max77503;
	int ret;

	max77503 = devm_kzalloc(dev, sizeof(*max77503), GFP_KERNEL);

	if (!max77503)
		return -ENOMEM;

	max77503->regmap = devm_regmap_init_i2c(client, &max77503_regmap_config);
	if (IS_ERR(max77503->regmap))
		return PTR_ERR(max77503->regmap);

	max77503->dev = dev;
	max77503->of_node = dev->of_node;
	max77503->desc = max77503_regulators_desc;

	config.dev = max77503->dev;
	config.of_node = max77503->of_node;
	config.driver_data = max77503;
	config.regmap = max77503->regmap;

	max77503->rdev = devm_regulator_register(max77503->dev, &max77503->desc, &config);
	if (IS_ERR(max77503->rdev))
		dev_err(max77503->dev, "Failed to register regulator\n");

	return ret;
}

static const struct of_device_id of_max77503_match_tbl[] = {
	{ .compatible = "adi,max77503", },
	{ },
};

MODULE_DEVICE_TABLE(of, of_max77503_match_tbl);

static const struct i2c_device_id max77503_regulator_id[] = {
	{ "max77503", 0 },
	{  /* sentinel */  }
};

MODULE_DEVICE_TABLE(i2c, max77503_regulator_id);

static struct i2c_driver max77503_regulator_driver = {
	.driver = {
		.name = "max77503",
		.of_match_table = of_max77503_match_tbl
	},
	.probe_new = max77503_regulator_probe,
	.id_table = max77503_regulator_id,
};

module_i2c_driver(max77503_regulator_driver);

MODULE_AUTHOR("Gokhan Celik <Gokhan.Celik@analog.com>");
MODULE_DESCRIPTION("MAX77503 regulator driver");
MODULE_LICENSE("GPL");
