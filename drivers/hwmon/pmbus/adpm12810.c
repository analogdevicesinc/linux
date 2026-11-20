// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware monitoring driver for Analog Devices ADPM12810
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 */
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pmbus.h>
#include <linux/string.h>
#include "pmbus.h"

#define ADPM12810_MODEL_PREFIX		"ADPM12810"

/*
 * The module implements only the commands below. Everything else, including
 * the virtual registers the PMBus core probes for, is reported as unsupported
 * so that no unsupported command is ever sent to the module.
 */
static int adpm12810_read_word_data(struct i2c_client *client, int page,
				    int phase, int reg)
{
	switch (reg) {
	case PMBUS_VOUT_OV_FAULT_LIMIT:
	case PMBUS_VOUT_OV_WARN_LIMIT:
	case PMBUS_VOUT_UV_WARN_LIMIT:
	case PMBUS_VOUT_UV_FAULT_LIMIT:
	case PMBUS_IOUT_OC_FAULT_LIMIT:
	case PMBUS_IOUT_OC_WARN_LIMIT:
	case PMBUS_OT_FAULT_LIMIT:
	case PMBUS_OT_WARN_LIMIT:
	case PMBUS_VIN_OV_FAULT_LIMIT:
	case PMBUS_VIN_OV_WARN_LIMIT:
	case PMBUS_VIN_UV_WARN_LIMIT:
	case PMBUS_VIN_UV_FAULT_LIMIT:
	case PMBUS_STATUS_WORD:
	case PMBUS_READ_VIN:
	case PMBUS_READ_IIN:
	case PMBUS_READ_VOUT:
	case PMBUS_READ_IOUT:
	case PMBUS_READ_TEMPERATURE_1:
	case PMBUS_READ_POUT:
	case PMBUS_READ_PIN:
		return -ENODATA;
	default:
		return -ENXIO;
	}
}

static int adpm12810_read_byte_data(struct i2c_client *client, int page,
				    int reg)
{
	switch (reg) {
	case PMBUS_OPERATION:
	case PMBUS_ON_OFF_CONFIG:
	case PMBUS_VOUT_MODE:
	case PMBUS_STATUS_VOUT:
	case PMBUS_STATUS_IOUT:
	case PMBUS_STATUS_INPUT:
	case PMBUS_STATUS_TEMPERATURE:
	case PMBUS_STATUS_CML:
	case PMBUS_REVISION:
		return -ENODATA;
	default:
		return -ENXIO;
	}
}

/*
 * Output overvoltage and overcurrent protection shut the module down in latch
 * mode, and clearing faults may restart a latched-off output. The driver
 * therefore never clears faults on its own: CLEAR_FAULTS requests from the
 * PMBus core complete without bus traffic.
 */
static int adpm12810_write_byte(struct i2c_client *client, int page, u8 value)
{
	if (value == PMBUS_CLEAR_FAULTS)
		return 0;

	return -ENXIO;
}

/*
 * The status registers are read-only and are cleared only by CLEAR_FAULTS.
 * Status bit write-backs from the PMBus core complete without bus traffic,
 * so the alarm attributes reflect the module's latched status.
 */
static int adpm12810_write_byte_data(struct i2c_client *client, int page,
				     int reg, u8 byte)
{
	if (reg >= PMBUS_STATUS_BYTE && reg <= PMBUS_STATUS_FAN_34)
		return 0;

	return -ENXIO;
}

/* The limit registers are read-only and no other word command is writable */
static int adpm12810_write_word_data(struct i2c_client *client, int page,
				     int reg, u16 word)
{
	return -ENXIO;
}

static struct pmbus_driver_info adpm12810_info = {
	.pages = 1,
	.format[PSC_VOLTAGE_IN] = linear,
	.format[PSC_VOLTAGE_OUT] = linear,
	.format[PSC_CURRENT_IN] = linear,
	.format[PSC_CURRENT_OUT] = linear,
	.format[PSC_POWER] = linear,
	.format[PSC_TEMPERATURE] = linear,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_IIN | PMBUS_HAVE_PIN |
		   PMBUS_HAVE_VOUT | PMBUS_HAVE_IOUT | PMBUS_HAVE_POUT |
		   PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_VOUT |
		   PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
		   PMBUS_HAVE_STATUS_TEMP,
	.read_word_data = adpm12810_read_word_data,
	.read_byte_data = adpm12810_read_byte_data,
	.write_byte = adpm12810_write_byte,
	.write_byte_data = adpm12810_write_byte_data,
	.write_word_data = adpm12810_write_word_data,
};

/*
 * CAPABILITY and WRITE_PROTECT are not implemented and the limit registers are
 * read-only. Register support is decided by the read hooks above, so the
 * STATUS_CML check is skipped: faults are never cleared by the driver, and a
 * latched communication error must not hide attributes.
 */
static struct pmbus_platform_data adpm12810_pdata = {
	.flags = PMBUS_NO_CAPABILITY | PMBUS_NO_WRITE_PROTECT |
		 PMBUS_WRITE_PROTECTED | PMBUS_SKIP_STATUS_CHECK,
};

static int adpm12810_read_model(struct i2c_client *client, char *buf)
{
	int ret;

	if (i2c_check_functionality(client->adapter,
				    I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return i2c_smbus_read_block_data(client, PMBUS_MFR_MODEL, buf);

	/* Reads the length byte followed by the data bytes */
	ret = i2c_smbus_read_i2c_block_data(client, PMBUS_MFR_MODEL,
					    I2C_SMBUS_BLOCK_MAX, buf);
	if (ret < 0)
		return ret;
	if (ret < 1 || buf[0] > ret - 1)
		return -EIO;

	ret = buf[0];
	memmove(buf, buf + 1, ret);

	return ret;
}

static int adpm12810_probe(struct i2c_client *client)
{
	char buf[I2C_SMBUS_BLOCK_MAX + 1];
	struct device *dev = &client->dev;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_WRITE_BYTE |
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA) &&
	    !i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK))
		return -ENODEV;

	ret = adpm12810_read_model(client, buf);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to read MFR_MODEL\n");

	if (ret < (int)strlen(ADPM12810_MODEL_PREFIX) ||
	    strncmp(buf, ADPM12810_MODEL_PREFIX, strlen(ADPM12810_MODEL_PREFIX)))
		return dev_err_probe(dev, -ENODEV, "Unsupported device: %*pE\n",
				     ret, buf);

	dev->platform_data = &adpm12810_pdata;

	return pmbus_do_probe(client, &adpm12810_info);
}

static const struct i2c_device_id adpm12810_id[] = {
	{ "adpm12810", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, adpm12810_id);

static const struct of_device_id adpm12810_of_match[] = {
	{ .compatible = "adi,adpm12810" },
	{}
};
MODULE_DEVICE_TABLE(of, adpm12810_of_match);

static struct i2c_driver adpm12810_driver = {
	.driver = {
		.name = "adpm12810",
		.of_match_table = adpm12810_of_match,
	},
	.probe = adpm12810_probe,
	.id_table = adpm12810_id,
};
module_i2c_driver(adpm12810_driver);

MODULE_AUTHOR("Carlos Jones Jr <carlosjr.jones@analog.com>");
MODULE_DESCRIPTION("PMBus driver for Analog Devices ADPM12810");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
