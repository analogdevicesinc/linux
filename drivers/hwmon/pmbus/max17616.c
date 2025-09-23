// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware monitoring driver for Analog Devices MAX17616/MAX17616A
 *
 * Copyright (C) 2025 Analog Devices, Inc.
 */

#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>

#include "pmbus.h"

static struct pmbus_driver_info max17616_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = direct,
	.m[PSC_VOLTAGE_IN] = 512,
	.b[PSC_VOLTAGE_IN] = -18,
	.R[PSC_VOLTAGE_IN] = -1,

	.format[PSC_VOLTAGE_OUT] = direct,
	.m[PSC_VOLTAGE_OUT] = 512,
	.b[PSC_VOLTAGE_OUT] = -18,
	.R[PSC_VOLTAGE_OUT] = -1,

	.format[PSC_CURRENT_OUT] = direct,
	.m[PSC_CURRENT_OUT] = 5845,
	.b[PSC_CURRENT_OUT] = 80,
	.R[PSC_CURRENT_OUT] = -1,

	.format[PSC_TEMPERATURE] = direct,
	.m[PSC_TEMPERATURE] = 71,
	.b[PSC_TEMPERATURE] = 19653,
	.R[PSC_TEMPERATURE] = -1,

	.func[0] =  PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT | PMBUS_HAVE_IOUT |
		    PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_VOUT |
		    PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
		    PMBUS_HAVE_STATUS_TEMP,
};

static int max17616_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	u8 buf[I2C_SMBUS_BLOCK_MAX];
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -ENODEV;

	ret = i2c_smbus_read_i2c_block_data(client, PMBUS_MFR_MODEL, sizeof(buf), buf);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to read MFR_MODEL\n");

	if ((strncmp(buf + 1, "MAX17616", 8) && strncmp(buf + 1, "MAX17616A", 9)))
		return dev_err_probe(dev, -ENODEV, "Unsupported device\n");

	return pmbus_do_probe(client, &max17616_info);
}

static const struct i2c_device_id max17616_id[] = {
	{ "max17616" },
	{ "max17616a" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max17616_id);

static const struct of_device_id __maybe_unused max17616_of_match[] = {
	{ .compatible = "adi,max17616" },
	{ .compatible = "adi,max17616a" },
	{ }
};
MODULE_DEVICE_TABLE(of, max17616_of_match);

static struct i2c_driver max17616_driver = {
	.driver = {
		.name = "max17616",
		.of_match_table = of_match_ptr(max17616_of_match),
	},
	.probe = max17616_probe,
	.id_table = max17616_id,
};
module_i2c_driver(max17616_driver);

MODULE_AUTHOR("Kim Seer Paller <kim.seer.paller@analog.com>");
MODULE_DESCRIPTION("PMBus driver for Analog Devices MAX17616/MAX17616A");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
