/*
 * ak4458-i2c.c  --  AK4458 DAC - I2C
 *
 * Copyright 2017 NXP
 *
 * Author: Mihai Serban <mihai.serban@nxp.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/pm_runtime.h>

#include "ak4458.h"

static int ak4458_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct regmap *regmap;
	int ret;

	regmap = devm_regmap_init_i2c(i2c, &ak4458_i2c_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	ret = ak4458_probe(&i2c->dev, regmap);
	if (ret)
		return ret;

	pm_runtime_enable(&i2c->dev);

	return 0;
}

static int ak4458_i2c_remove(struct i2c_client *i2c)
{
	ak4458_remove(&i2c->dev);
	pm_runtime_disable(&i2c->dev);

	return 0;
}

static const struct i2c_device_id ak4458_i2c_id[] = {
	{ "ak4458", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ak4458_i2c_id);

static const struct of_device_id ak4458_of_match[] = {
	{ .compatible = "asahi-kasei,ak4458", },
	{ },
};
MODULE_DEVICE_TABLE(of, ak4458_of_match);

static struct i2c_driver ak4458_i2c_driver = {
	.driver = {
		.name = "ak4458",
		.pm = &ak4458_pm,
		.of_match_table = ak4458_of_match,
	},
	.probe = ak4458_i2c_probe,
	.remove = ak4458_i2c_remove,
	.id_table = ak4458_i2c_id
};

module_i2c_driver(ak4458_i2c_driver);

MODULE_DESCRIPTION("ASoC AK4458 driver - I2C");
MODULE_AUTHOR("Mihai Serban <mihai.serban@nxp.com>");
MODULE_LICENSE("GPL");
