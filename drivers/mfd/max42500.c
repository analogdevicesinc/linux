// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX42500 - MFD Power System Monitor
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/mfd/max42500.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

static const struct mfd_cell max42500_subdev[] = {
	MFD_CELL_NAME("max42500-hwmon"),
	MFD_CELL_NAME("max42500-wdt"),
};

static const struct regmap_config max42500_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= MAX42500_REG_CID,
};

static int max42500_probe(struct i2c_client *i2c)
{
	struct regmap *map;

	map = devm_regmap_init_i2c(i2c, &max42500_regmap_config);
	if (IS_ERR(map))
		return PTR_ERR(map);

	return devm_mfd_add_devices(&i2c->dev, PLATFORM_DEVID_AUTO,
				    max42500_subdev, ARRAY_SIZE(max42500_subdev),
				    NULL, 0, NULL);
}

static const struct of_device_id max42500_of_match[] = {
	{ .compatible = "adi,max42500" },
	{ }
};
MODULE_DEVICE_TABLE(of, max42500_of_match);

static struct i2c_driver max42500_driver = {
	.driver = {
		.name = "max42500",
		.of_match_table = max42500_of_match,
	},
	.probe = max42500_probe,
};
module_i2c_driver(max42500_driver);

MODULE_AUTHOR("Kent Libetario <kent.libetario@analog.com>");
MODULE_DESCRIPTION("MFD driver for MAX42500");
MODULE_LICENSE("GPL");
