// SPDX-License-Identifier: GPL-2.0-only

#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/module.h>

#include <linux/a2b/a2b.h>
#include "ad2428.h"

static const struct mfd_cell ad2428_cells[] = {
	{
		.of_compatible = "adi,ad2428w-i2c",
		.name = "ad242x-i2c",
	},
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
	{
		.of_compatible = "adi,ad2428w-mailbox",
		.name = "a2b-mailbox",
	},
};

static const struct a2b_device_id_table ad2428_a2b_id_table[] = {
	{
		.vendor = 0xad,
		.product = 0x20,
		.product_mask = 0xf0,
	},
	{}
};

static struct a2b_subnode_driver ad2428_a2b_driver = {
	.driver = { .name = "ad2428" },
	.id_table = ad2428_a2b_id_table,
	.regmap_config = &ad2428_regmap_config,
	.cells = ad2428_cells,
	.num_cells = ARRAY_SIZE(ad2428_cells),
};
module_a2b_driver(ad2428_a2b_driver);

MODULE_DESCRIPTION("AD2428 subnode driver");
MODULE_LICENSE("GPL v2");
