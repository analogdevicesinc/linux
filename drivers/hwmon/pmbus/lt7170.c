// SPDX-License-Identifier: GPL-2.0
/*
 * Hardware monitoring driver for Analog Devices LT7170
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */
#include <linux/bits.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include "pmbus.h"

#define LT7170_NUM_PAGES	1

#define MFR_READ_EXTVCC		0xcd
#define MFR_READ_ITH		0xce
#define MFR_CONFIG_ALL_LT7170	0xd1
#define MFR_IOUT_PEAK		0xd7
#define MFR_ADC_CONTROL_LT7170 0xd8

#define MFR_DEBUG_TELEMETRY	BIT(0)

#define MFR_VOUT_PEAK		0xdd
#define MFR_VIN_PEAK		0xde
#define MFR_TEMPERATURE_1_PEAK	0xdf
#define MFR_CLEAR_PEAKS		0xe3

static int lt7170_read_word_data(struct i2c_client *client, int page, int phase, int reg)
{
	int ret;

	switch (reg) {
	case PMBUS_VIRT_READ_VMON:
		if (page == 0)
			ret = pmbus_read_word_data(client, page, phase, MFR_READ_ITH);
		else
			ret = pmbus_read_word_data(client, 0, phase, MFR_READ_EXTVCC);
		break;
	case PMBUS_VIRT_READ_IOUT_MAX:
		ret = pmbus_read_word_data(client, page, phase, MFR_IOUT_PEAK);
		break;
	case PMBUS_VIRT_READ_VOUT_MAX:
		ret = pmbus_read_word_data(client, page, phase, MFR_VOUT_PEAK);
		break;
	case PMBUS_VIRT_READ_VIN_MAX:
		ret = pmbus_read_word_data(client, page, phase, MFR_VIN_PEAK);
		break;
	case PMBUS_VIRT_READ_TEMP_MAX:
		ret = pmbus_read_word_data(client, page, phase, MFR_TEMPERATURE_1_PEAK);
		break;
	case PMBUS_VIRT_RESET_VIN_HISTORY:
		ret = (page == 0) ? 0 : -ENODATA;
		break;
	default:
		ret = -ENODATA;
		break;
	}
	return ret;
}

static int lt7170_write_word_data(struct i2c_client *client, int page, int reg, u16 word)
{
	int ret;

	switch (reg) {
	case PMBUS_VIRT_RESET_VIN_HISTORY:
		ret = pmbus_write_byte(client, 0, MFR_CLEAR_PEAKS);
		break;
	default:
		ret = -ENODATA;
		break;
	}
	return ret;
}

static struct pmbus_driver_info lt7170_info = {
	.pages = LT7170_NUM_PAGES,
	.format[PSC_VOLTAGE_IN] = ieee754,
	.format[PSC_VOLTAGE_OUT] = ieee754,
	.format[PSC_CURRENT_OUT] = ieee754,
	.format[PSC_TEMPERATURE] = ieee754,
	.func[0] = PMBUS_HAVE_VIN | PMBUS_HAVE_VOUT |
	  PMBUS_HAVE_IOUT | PMBUS_HAVE_TEMP | PMBUS_HAVE_STATUS_VOUT |
	  PMBUS_HAVE_STATUS_IOUT | PMBUS_HAVE_STATUS_INPUT |
	  PMBUS_HAVE_STATUS_TEMP,
	.read_word_data = lt7170_read_word_data,
	.write_word_data = lt7170_write_word_data,
};

static int lt7170_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct pmbus_driver_info *info;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA |
				     I2C_FUNC_SMBUS_READ_BLOCK_DATA))
		return -ENODEV;

	info = devm_kmemdup(dev, &lt7170_info,
			    sizeof(struct pmbus_driver_info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	/* Enable VMON output if configured */
	ret = i2c_smbus_read_byte_data(client, MFR_ADC_CONTROL_LT7170);
	if (ret < 0)
		return ret;
	if (ret & MFR_DEBUG_TELEMETRY) {
		info->pages = 1;
		info->func[0] |= PMBUS_HAVE_VMON;
	}

	return pmbus_do_probe(client, &lt7170_info);
}

static const struct i2c_device_id lt7170_id[] = {
	{"lt7170", 0},
	{"lt7170-1", 0},
	{"lt7171", 0},
	{"lt7171-1", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, lt7170_id);

static const struct of_device_id lt7170_of_match[] = {
	{ .compatible = "adi,lt7170" },
	{ .compatible = "adi,lt7170-1" },
	{ .compatible = "adi,lt7171" },
	{ .compatible = "adi,lt7171-1" },
	{}
};
MODULE_DEVICE_TABLE(of, lt7170_of_match);

static struct i2c_driver lt7170_driver = {
	.driver = {
	.name = "lt7170",
	.of_match_table = of_match_ptr(lt7170_of_match),
	},
	.probe = lt7170_probe,
	.id_table = lt7170_id,
};

module_i2c_driver(lt7170_driver);

MODULE_AUTHOR("Cherrence Sarip <cherrence.sarip@analog.com>");
MODULE_DESCRIPTION("PMBus driver for Analog Devices LT7170");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(PMBUS);
