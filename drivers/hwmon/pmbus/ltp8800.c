// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware monitoring driver for Analog Devices LTP8800
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include "pmbus.h"

static const struct regulator_desc __maybe_unused ltp8800_reg_desc[] = {
	PMBUS_REGULATOR("vout", 0),
};

static struct pmbus_driver_info ltp8800_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT |
		   PMBUS_HAVE_IIN | PMBUS_HAVE_IOUT |
		   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP |
		   PMBUS_HAVE_POUT,
#if IS_ENABLED(CONFIG_SENSORS_LTP8800_REGULATOR)
	.num_regulators = 1,
	.reg_desc = ltp8800_reg_desc,
#endif
};

/* 6.1 requires i2c_device_id */
static int ltp8800_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -ENODEV;

	return pmbus_do_probe(client, &ltp8800_info);
}

static const struct i2c_device_id ltp8800_id[] = {
	{"ltp8800-1a", 0},
	{"ltp8800-4a", 0},
	{"ltp8800-2", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, ltp8800_id);

static const struct of_device_id ltp8800_of_match[] = {
	{ .compatible = "adi,ltp8800-1a"},
	{ .compatible = "adi,ltp8800-4a"},
	{ .compatible = "adi,ltp8800-2"},
	{}
};
MODULE_DEVICE_TABLE(of, ltp8800_of_match);

static struct i2c_driver ltp8800_driver = {
	.driver = {
		.name = "ltp8800",
		.of_match_table = ltp8800_of_match,
	},
	.probe = ltp8800_probe,
	.id_table = ltp8800_id,
};
module_i2c_driver(ltp8800_driver);

MODULE_AUTHOR("Cedric Encarnacion <cedricjustine.encarnacion@analog.com>");
MODULE_DESCRIPTION("Analog Devices LTP8800 HWMON PMBus Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
