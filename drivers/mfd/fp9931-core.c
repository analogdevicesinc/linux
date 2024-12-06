/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Fitipower FP9931 PMIC core driver
 *
 * Copyright 2021 NXP
 */

#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/mfd/fp9931.h>
#include <linux/module.h>

static const unsigned short normal_i2c[] = {
	0x18, I2C_CLIENT_END
};

static struct mfd_cell fp9931_devs[] = {
	{ .name = "fp9931-pmic",  },
	{ .name = "fp9931-hwmon", },
};

int fp9931_reg_read(struct i2c_client *client, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0) {
		dev_err(&client->dev, "Unable to read reg %d : %d\n",
			reg, ret);
		return ret;
	}

	*val = ret;

	return 0;
}
EXPORT_SYMBOL(fp9931_reg_read);

int fp9931_reg_write(struct i2c_client *client, int reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		dev_err(&client->dev, "Unable to write to reg %d with %u : %d\n",
			reg, val, ret);

	return ret;
}
EXPORT_SYMBOL(fp9931_reg_write);

static int fp9931_detect(struct i2c_client *client,
			 struct i2c_board_info *info)
{
	int ret;
	u8 tmst_value;
	s8 temperature;
	struct i2c_adapter *adapter = client->adapter;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -ENODEV;

	ret = fp9931_reg_read(client, FP9931_TMST_VALUE, &tmst_value);
	if (ret)
		return ret;
	temperature = tmst_value;

	dev_info(&client->dev,
		 "detect temperature is : %d Celsius Degree\n",
		 temperature);

	return 0;
}

static int fp9931_probe(struct i2c_client *client)
{
	int ret;
	struct fp9931 *fp9931;
	struct device *dev = &client->dev;

	fp9931 = devm_kzalloc(dev, sizeof(*fp9931), GFP_KERNEL);
	if (!fp9931)
		return -ENOMEM;

	fp9931->dev = dev;
	fp9931->client = client;

	i2c_set_clientdata(client, fp9931);

	/* call detect here, since the .detect function of
	 * i2c_driver won't be called if the driver's class
	 * and adapter's class do not match.
	 */
	ret = fp9931_detect(client, NULL);
	if (ret)
		return ret;

	fp9931->pdata = devm_kzalloc(dev, sizeof(*fp9931->pdata), GFP_KERNEL);
	if (!fp9931->pdata)
		return -ENOMEM;

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE, fp9931_devs,
				   ARRAY_SIZE(fp9931_devs), NULL, 0, NULL);
	if (ret) {
		dev_err(dev, "Failed to add pmic subdevices: %d\n", ret);
		return ret;
	}

	return 0;
}

static void fp9931_remove(struct i2c_client *i2c)
{
	/* nothing needs to be done */
}

static const struct i2c_device_id fp9931_id[] = {
	{ "fp9931", 0 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(i2c, fp9931_id);

static const struct of_device_id fp9931_dt_ids[] = {
	{
		.compatible = "fitipower,fp9931",
	},
	{	/* sentinel */	}
};
MODULE_DEVICE_TABLE(of, fp9931_dt_ids);

static struct i2c_driver fp9931_driver = {
	.driver = {
		.name  = "fp9931",
		.owner = THIS_MODULE,
		.of_match_table = fp9931_dt_ids,
	},
	.probe  = fp9931_probe,
	.remove = fp9931_remove,
	.id_table = fp9931_id,
	.detect = fp9931_detect,
	.address_list = normal_i2c,
};

static int __init fp9931_init(void)
{
	return i2c_add_driver(&fp9931_driver);
}

static void fp9931_exit(void)
{
	i2c_del_driver(&fp9931_driver);
}

subsys_initcall(fp9931_init);
module_exit(fp9931_exit);

MODULE_DESCRIPTION("PF9931 PMIC core driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
