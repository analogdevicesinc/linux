// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware monitoring driver for Analog Devices ADP1050
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */
#include <linux/bits.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>

#include "pmbus.h"

enum ADP1050_type {
	ADP1050,
	ADP1051,
	ADP1055,
};

static struct pmbus_driver_info adp1050_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT
		   | PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT
		   | PMBUS_HAVE_IIN | PMBUS_HAVE_TEMP
		   | PMBUS_HAVE_STATUS_TEMP,
};

static struct pmbus_driver_info adp1051_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_IIN | PMBUS_HAVE_VOUT
		   | PMBUS_HAVE_IOUT | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_VOUT
		   | PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT
		   | PMBUS_HAVE_STATUS_TEMP,
};

static struct pmbus_driver_info adp1055_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_IIN | PMBUS_HAVE_VOUT
		   | PMBUS_HAVE_IOUT | PMBUS_HAVE_TEMP2 | PMBUS_HAVE_TEMP3
		   | PMBUS_HAVE_POUT | PMBUS_HAVE_STATUS_VOUT
		   | PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT
		   | PMBUS_HAVE_STATUS_TEMP,
};

static int adp1050_probe(struct i2c_client *client)
{
	enum ADP1050_type type;

	type = (enum ADP1050_type)(uintptr_t)device_get_match_data(&client->dev);

	switch (type) {
	case ADP1050:
		return pmbus_do_probe(client, &adp1050_info);
	case ADP1051:
		return pmbus_do_probe(client, &adp1051_info);
	case ADP1055:
		return pmbus_do_probe(client, &adp1055_info);
	default:
		return -EINVAL;
	}
}

static const struct i2c_device_id adp1050_id[] = {
	{"adp1050", ADP1050},
	{"adp1051", ADP1051},
	{"adp1055", ADP1055},
	{}
};

MODULE_DEVICE_TABLE(i2c, adp1050_id);

static const struct of_device_id adp1050_of_match[] = {
	{ .compatible = "adi,adp1050", .data = (void *)ADP1050},
	{ .compatible = "adi,adp1051", .data = (void *)ADP1051},
	{ .compatible = "adi,adp1055", .data = (void *)ADP1055},
	{}
};
MODULE_DEVICE_TABLE(of, adp1050_of_match);

static struct i2c_driver adp1050_driver = {
	.driver = {
		.name = "adp1050",
		.of_match_table = adp1050_of_match,
	},
	.probe = adp1050_probe,
	.id_table = adp1050_id,
};
module_i2c_driver(adp1050_driver);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADP1050 HWMON PMBus Driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
