// SPDX-License-Identifier: GPL-2.0
/*
 * NXP P3T1085 Temperature Sensor Driver
 *
 * Copyright 2024 NXP
 */
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/slab.h>
#include <linux/regmap.h>

#include "p3t1085.h"

static const struct i3c_device_id p3t1085_i3c_ids[] = {
	I3C_DEVICE(0x011B, 0x1529, (void *)P3T1085_ID),
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(i3c, p3t1085_i3c_ids);

static int p3t1085_i3c_probe(struct i3c_device *i3cdev)
{
	struct regmap_config p3t1085_i3c_regmap_config = {
		.reg_bits = 8,
		.val_bits = 16,
	};
	const struct i3c_device_id *id = i3c_device_match_id(i3cdev,
							    p3t1085_i3c_ids);
	struct regmap *regmap;

	regmap = devm_regmap_init_i3c(i3cdev, &p3t1085_i3c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i3cdev->dev, "Failed to register i3c regmap %ld\n", PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return p3t1085_probe(&i3cdev->dev, 0, (uintptr_t)id->data, regmap);
}

static struct i3c_driver p3t1085_driver = {
	.driver = {
		.name = "p3t1085_i3c",
	},
	.probe = p3t1085_i3c_probe,
	.id_table = p3t1085_i3c_ids,
};
module_i3c_driver(p3t1085_driver);

MODULE_AUTHOR("Xiaoning Wang <xiaoning.wang@nxp.com>");
MODULE_DESCRIPTION("NXP p3t1085 i3c driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_P3T1085);
