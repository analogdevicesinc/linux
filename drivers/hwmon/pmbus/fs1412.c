// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Hardware monitoring driver for the TDK FS1412 and FS1525 uPOL regulators
 *
 * Copyright 2026 Nexthop Systems.
 */

#include <linux/array_size.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pmbus.h>
#include <linux/string.h>

#include "pmbus.h"

#define FS1412_MFR_ID		"KDT"
#define FS1412_MFR_ID_LEN	(sizeof(FS1412_MFR_ID) - 1)

/*
 * OT_WARN_LIMIT is not in the FS1412 command table. The register answers,
 * but the part has no warning threshold and reports OT_WARNING at room
 * temperature, so keep the core from exposing either.
 */
static int fs1412_read_word_data(struct i2c_client *client, int page,
				 int phase, int reg)
{
	if (reg == PMBUS_OT_WARN_LIMIT)
		return -ENXIO;

	return -ENODATA;
}

static int fs1412_write_word_data(struct i2c_client *client, int page,
				  int reg, u16 word)
{
	if (reg == PMBUS_OT_WARN_LIMIT)
		return -ENXIO;

	return -ENODATA;
}

/*
 * The FS1525 compares READ_VOUT, the output voltage, against its warning and
 * under-voltage limits, but its over-voltage fault is an analog comparison at
 * the feedback pin. With an external divider VOUT_OV_FAULT_LIMIT would read in
 * feedback-pin volts, below in2_input, and the part cannot report the divider,
 * so do not expose it.
 */
static int fs1525_read_word_data(struct i2c_client *client, int page,
				 int phase, int reg)
{
	if (reg == PMBUS_VOUT_OV_FAULT_LIMIT)
		return -ENXIO;

	return -ENODATA;
}

static int fs1525_write_word_data(struct i2c_client *client, int page,
				  int reg, u16 word)
{
	if (reg == PMBUS_VOUT_OV_FAULT_LIMIT)
		return -ENXIO;

	return -ENODATA;
}

enum chips { fs1412, fs1525 };

/* IC_DEVICE_ID, a one byte block */
static const u8 fs1412_device_id[] = {
	[fs1412] = 0x0c,
	[fs1525] = 0x19,
};

static struct pmbus_driver_info fs1412_info[] = {
	[fs1412] = {
		.pages = 1,
		.format[PSC_VOLTAGE_IN] = linear,
		.format[PSC_VOLTAGE_OUT] = linear,
		.format[PSC_TEMPERATURE] = linear,
		.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT |
			   PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
			   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP,
		.read_word_data = fs1412_read_word_data,
		.write_word_data = fs1412_write_word_data,
	},
	[fs1525] = {
		.pages = 1,
		.format[PSC_VOLTAGE_IN] = linear,
		.format[PSC_VOLTAGE_OUT] = linear,
		.format[PSC_CURRENT_OUT] = linear,
		.format[PSC_TEMPERATURE] = linear,
		.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_STATUS_INPUT |
			   PMBUS_HAVE_VOUT | PMBUS_HAVE_STATUS_VOUT |
			   PMBUS_HAVE_IOUT | PMBUS_HAVE_STATUS_IOUT |
			   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_TEMP,
		.read_word_data = fs1525_read_word_data,
		.write_word_data = fs1525_write_word_data,
	},
};

static const struct i2c_device_id fs1412_id[] = {
	{ .name = "fs1412", .driver_data = fs1412 },
	{ .name = "fs1525", .driver_data = fs1525 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fs1412_id);

static int fs1412_probe(struct i2c_client *client)
{
	char buf[I2C_SMBUS_BLOCK_MAX + 1] = { 0 };
	struct device *dev = &client->dev;
	enum chips chip, detected;
	int ret;

	ret = i2c_smbus_read_block_data(client, PMBUS_MFR_ID, buf);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to read PMBUS_MFR_ID\n");

	if (ret != FS1412_MFR_ID_LEN ||
	    strncmp(buf, FS1412_MFR_ID, FS1412_MFR_ID_LEN))
		return dev_err_probe(dev, -ENODEV,
				     "Unsupported Manufacturer ID '%s'\n", buf);

	ret = i2c_smbus_read_block_data(client, PMBUS_IC_DEVICE_ID, buf);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to read PMBUS_IC_DEVICE_ID\n");
	if (ret != 1)
		return dev_err_probe(dev, -ENODEV,
				     "Unexpected IC_DEVICE_ID length %d\n", ret);

	for (detected = 0; detected < ARRAY_SIZE(fs1412_device_id); detected++)
		if (fs1412_device_id[detected] == (u8)buf[0])
			break;
	if (detected == ARRAY_SIZE(fs1412_device_id))
		return dev_err_probe(dev, -ENODEV,
				     "Unsupported IC_DEVICE_ID 0x%02x\n",
				     (u8)buf[0]);

	chip = (uintptr_t)i2c_get_match_data(client);
	if (chip != detected)
		dev_warn(dev, "Device mismatch: Configured %s, detected %s\n",
			 client->name, fs1412_id[detected].name);

	return pmbus_do_probe(client, &fs1412_info[detected]);
}

static const struct of_device_id fs1412_of_match[] = {
	{ .compatible = "tdk,fs1412", .data = (void *)fs1412 },
	{ .compatible = "tdk,fs1525", .data = (void *)fs1525 },
	{ }
};
MODULE_DEVICE_TABLE(of, fs1412_of_match);

static struct i2c_driver fs1412_driver = {
	.driver = {
		.name = "fs1412",
		.of_match_table = fs1412_of_match,
	},
	.probe = fs1412_probe,
	.id_table = fs1412_id,
};

module_i2c_driver(fs1412_driver);

MODULE_AUTHOR("Abdurrahman Hussain <abdurrahman@nexthop.ai>");
MODULE_DESCRIPTION("PMBus driver for the TDK FS1412 and FS1525 uPOL regulators");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("PMBUS");
