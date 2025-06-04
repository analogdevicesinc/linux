// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices LTC4283 I2C Negative Voltage Hot Swap Controller
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/i2c.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/regmap.h>

static const struct mfd_cell ltc4283_cells[] = {
	MFD_CELL_OF("ltc4283-hwmon", NULL, NULL, 0, 0, "adi,ltc4283-hwmon"),
	MFD_CELL_OF("ltc4283-gpio", NULL, NULL, 0, 0, "adi,ltc4283-gpio"),
};

static const struct regmap_config ltc4283_regmap_config = {

};

static int ltc4283_probe(struct i2c_client *client)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(client, &ltc4283_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	return devm_mfd_add_devices(&client->dev, PLATFORM_DEVID_AUTO,
				    ltc4283_cells, ARRAY_SIZE(ltc4283_cells) - 1,
				    NULL, 0, NULL);
}

static const struct of_device_id ltc4283_of_match[] = {
	{ .compatible = "adi,ltc4283" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc4283_of_match);

static const struct i2c_device_id ltc4283_i2c_id[] = {
	{ "ltc4283" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc4283_i2c_id);

static struct i2c_driver ltc4283_driver = {
	.driver = {
		.name = "ltc4283",
		.of_match_table = ltc4283_of_match,
	},
	.probe = ltc4283_probe,
	.id_table = ltc4283_i2c_id,
};
module_i2c_driver(ltc4283_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("LTC4283 MFD I2C driver");
MODULE_LICENSE("GPL v2");
