// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADPD188 I2C driver
 *
 * Copyright 2021 Analog Devices Inc.
 *
 * 7-bit I2C slave address: 0x64
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "adpd188.h"

static const struct regmap_range wr_no_range[] = {
	regmap_reg_range(0x3, 0x3),
	regmap_reg_range(0x8, 0x8),
	regmap_reg_range(0xA, 0xA),
	regmap_reg_range(0xC, 0xC),
	regmap_reg_range(0xE, 0xE),
	regmap_reg_range(0x13, 0x13),
	regmap_reg_range(0x26, 0x2F),
	regmap_reg_range(0x32, 0x33),
	regmap_reg_range(0x3A, 0x3A),
	regmap_reg_range(0x3D, 0x3D),
	regmap_reg_range(0x40, 0x41),
	regmap_reg_range(0x46, 0x4A),
	regmap_reg_range(0x4C, 0x4C),
	regmap_reg_range(0x4E, 0x4E),
	regmap_reg_range(0x51, 0x53),
	regmap_reg_range(0x56, 0x57),
	regmap_reg_range(0x5B, 0x5D),
	regmap_reg_range(0x60, 0x7F)
};

static struct regmap_access_table write_tab = {
	.no_ranges = wr_no_range,
	.n_no_ranges = 18
};

static const struct regmap_range rd_no_range[] = {
	regmap_reg_range(0x3, 0x3),
	regmap_reg_range(0xC, 0xC),
	regmap_reg_range(0xE, 0xE),
	regmap_reg_range(0x13, 0x13),
	regmap_reg_range(0x26, 0x2F),
	regmap_reg_range(0x32, 0x33),
	regmap_reg_range(0x3A, 0x3A),
	regmap_reg_range(0x3D, 0x3D),
	regmap_reg_range(0x40, 0x41),
	regmap_reg_range(0x46, 0x4A),
	regmap_reg_range(0x4C, 0x4C),
	regmap_reg_range(0x4E, 0x4E),
	regmap_reg_range(0x51, 0x53),
	regmap_reg_range(0x56, 0x57),
	regmap_reg_range(0x5B, 0x5D),
	regmap_reg_range(0x61, 0x63),
	regmap_reg_range(0x6C, 0x6F)
};

static struct regmap_access_table read_tab = {
	.no_ranges = rd_no_range,
	.n_no_ranges = ARRAY_SIZE(rd_no_range)
};

const struct regmap_config adpd188_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x7F,
	.wr_table = &write_tab,
	.rd_table = &read_tab
};

static int adpd188_i2c_probe(struct i2c_client *i2c)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(i2c, &adpd188_i2c_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&i2c->dev, "Error initializing i2c regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	return adpd188_core_probe(&i2c->dev, regmap, i2c->name, i2c->irq);
}

static const struct i2c_device_id adpd188_i2c_id[] = {
	{ "adpd188", ADPD188 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adpd188_i2c_id);

static const struct of_device_id adpd188_of_match[] = {
	{ .compatible = "adi,adpd188" },
	{},
};
MODULE_DEVICE_TABLE(of, adpd188_of_match);

static struct i2c_driver adpd188_i2c_driver = {
	.driver = {
			.name = "adpd188_i2c",
			.of_match_table = of_match_ptr(adpd188_of_match),
		},
	.probe_new = adpd188_i2c_probe,
	.id_table = adpd188_i2c_id,
};
module_i2c_driver(adpd188_i2c_driver);

MODULE_AUTHOR("Andrei Drimbarean <andrei.drimbarean@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADPD188 I2C driver");
MODULE_LICENSE("GPL v2");

