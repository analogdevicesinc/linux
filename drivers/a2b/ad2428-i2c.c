// SPDX-License-Identifier: GPL-2.0-only

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/a2b/a2b.h>
#include "ad2428.h"

static const struct mfd_cell ad2428_cells[] = {
	{
		.of_compatible = "adi,ad2428w-gpio",
		.name = "ad242x-gpio",
	},
	{
		.of_compatible = "adi,ad2428w-clk",
		.name = "ad242x-clk",
	},
	{
		.of_compatible = "adi,ad2428w-codec",
		.name = "ad242x-codec",
	},
};

static int ad2428_main_probe(struct i2c_client *i2c)
{
	struct a2b_mainnode *node;

	node = devm_a2b_mainnode_register(i2c, &ad2428_regmap_config,
					  ad2428_cells,
					  ARRAY_SIZE(ad2428_cells));
	if (IS_ERR(node))
		return PTR_ERR(node);

	return 0;
}

static const struct of_device_id ad2428_main_of_match[] = {
	{ .compatible = "adi,ad2428w" },
	{}
};
MODULE_DEVICE_TABLE(of, ad2428_main_of_match);

static const struct i2c_device_id ad2428_main_i2c_id[] = { { "ad2428", 0 },
							   {} };
MODULE_DEVICE_TABLE(i2c, ad2428_main_i2c_id);

static struct i2c_driver ad2428_main_i2c_driver = {
	.driver	= {
		.name = "ad2428-main",
		.of_match_table	= ad2428_main_of_match,
	},
	.probe_new = ad2428_main_probe,
	.id_table = ad2428_main_i2c_id,
};
module_i2c_driver(ad2428_main_i2c_driver);

MODULE_DESCRIPTION("AD2428 mainnode driver");
MODULE_LICENSE("GPL v2");
