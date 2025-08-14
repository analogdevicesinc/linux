// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2025 Analog Devices, Inc.
 * ADI regulator driver for MAX77675.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regulator/driver.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#define MAX77675_NUM_REGULATORS 4

struct max77675_regulator {
	struct device *dev;
	struct regulator_dev *rdev[MAX77675_NUM_REGULATORS];
	struct regmap *regmap;
};

static const struct regulator_ops max77675_regulator_ops = {
	.enable     = regulator_enable_regmap,
	.disable    = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static const struct regulator_desc max77675_regulators[] = {
	{
		.name = "simo0",
		.id = 0,
		.ops = &max77675_regulator_ops,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
		.n_voltages = 64,
		.min_uV = 500000,
		.uV_step = 50000,
		.vsel_reg = 0x10,   // example value : SIMO0 VOUT
		.vsel_mask = 0x3F,
		.enable_reg = 0x20, // example value :  SIMO0 ENABLE
		.enable_mask = 0x01,
	},
};

static int max77675_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct max77675_regulator *priv;
	int i, ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &some_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(&client->dev, PTR_ERR(priv->regmap), "regmap init failed");

	for (i = 0; i < MAX77675_NUM_REGULATORS; i++) {
		struct regulator_config config = {};
		config.dev = &client->dev;
		config.regmap = priv->regmap;

		priv->rdev[i] = devm_regulator_register(&client->dev,
				&max77675_regulators[i], &config);
		if (IS_ERR(priv->rdev[i])) {
			dev_err(&client->dev, "Failed to register regulator %d\n", i);
			return PTR_ERR(priv->rdev[i]);
		}
	}

	dev_info(&client->dev, "MAX77675 regulator driver loaded\n");
	return 0;
}

static const struct of_device_id max77675_of_match[] = {
	{ .compatible = "maxim,max77675", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77675_of_match);

static const struct i2c_device_id max77675_id[] = {
	{ "max77675", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max77675_id);

static struct i2c_driver max77675_driver = {
	.driver = {
		.name = "max77675",
		.of_match_table = max77675_of_match,
	},
	.probe = max77675_probe,
	.id_table = max77675_id,
};

module_i2c_driver(max77675_driver);

MODULE_AUTHOR("joan.na@analog.com");
MODULE_DESCRIPTION("MAX77675 Regulator Driver");
MODULE_LICENSE("GPL");

