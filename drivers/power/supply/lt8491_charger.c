// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LT8491 Battery Charger
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/power_supply.h>

#define LT8491_TELE_TBAT_REG 0x0
#define LT8491_TELE_POUT_REG 0x2
#define LT8491_TELE_PIN_REG  0x4
#define LT8491_TELE_EFF_REG  0x6
#define LT8491_TELE_IOUT_REG 0x8
#define LT8491_TELE_IIN_REG  0xA
#define LT8491_TELE_VBAT_REG 0xC
#define LT8491_TELE_VIN_REG  0xE
#define LT8491_TELE_VINR_REG 0x10
#define LT8491_STAT_CHARGER_REG 0x12
#define LT8491_STAT_CHRG_FAULTS_REG 0x19
#define LT8491_CTRL_UPDATE_TELEM_REG 0x26

#define LT8491_CFG_RSENSE1_REG 0x28
#define LT8491_CFG_RIMON_OUT_REG 0x2A
#define LT8491_CFG_RSENSE2_REG 0x2C
#define LT8491_CFG_RDACO_REG 0x2E
#define LT8491_CFG_RFBOUT1_REG 0x30
#define LT8491_CFG_RFBOUT2_REG 0x32
#define LT8491_CFG_RDACI_REG 0x34
#define LT8491_CFG_RFBIN2_REG 0x36
#define LT8491_CFG_RFBIN1_REG 0x38
#define LT8491_CFG_TBAT_MIN_REG 0x40
#define LT8491_CFG_TBAT_MAX_REG 0x41
#define LT8491_MFR_DATA1_LSB_REG 0x5C

#define LT8491_TELEM_ACTIVE_MASK BIT(6)
#define LT8491_CHARGING_MASK BIT(2)
#define LT8491_BAT_DISCON_FLT_MASK BIT(3)

#define LT8491_UPDATE_TELEM_CMD 0xAA

#define LT8491_MFR_DATA_LEN 0x3

struct lt8491_info {
	struct i2c_client *client;
	struct power_supply *psp;
	char serial_number[16];
	/* protect against device accesses */
	struct mutex lock;
};

static int lt8491_read_serial_number(struct lt8491_info *info)
{
	int i, ret;
	u32 serial_number[LT8491_MFR_DATA_LEN];

	for (i = 0; i < LT8491_MFR_DATA_LEN; i++) {
		serial_number[i] = i2c_smbus_read_word_data(info->client, LT8491_MFR_DATA1_LSB_REG + i * 2);
		if (serial_number[i] < 0)
			return serial_number[i];
	}

	ret = sprintf(info->serial_number, "%04x%04x%04x", serial_number[0], serial_number[1], serial_number[2]);
	if (ret < 0)
		return ret;

	return 0;
}

static int lt8491_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct lt8491_info *info = power_supply_get_drvdata(psy);
	s16 ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = i2c_smbus_read_byte_data(info->client, LT8491_STAT_CHARGER_REG);
		if (ret < 0)
			return ret;

		val->intval = FIELD_GET(LT8491_CHARGING_MASK, ret) ?
					POWER_SUPPLY_STATUS_CHARGING :
					POWER_SUPPLY_STATUS_NOT_CHARGING;

		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		mutex_lock(&info->lock);

		ret = i2c_smbus_read_byte_data(info->client, LT8491_STAT_CHARGER_REG);
		if (ret < 0)
			goto unlock;

		if (!FIELD_GET(LT8491_TELEM_ACTIVE_MASK, ret)) {
			ret = i2c_smbus_write_byte_data(info->client,
						LT8491_CTRL_UPDATE_TELEM_REG,
						LT8491_UPDATE_TELEM_CMD);
			if (ret)
				goto unlock;
		}

		ret = i2c_smbus_read_word_data(info->client, LT8491_TELE_VBAT_REG);
		if (ret < 0)
			goto unlock;

		mutex_unlock(&info->lock);

		val->intval = ret * 10000;

		return 0;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		mutex_lock(&info->lock);

		ret = i2c_smbus_read_byte_data(info->client, LT8491_STAT_CHARGER_REG);
		if (ret < 0)
			goto unlock;

		if (!FIELD_GET(LT8491_TELEM_ACTIVE_MASK, ret)) {
			ret = i2c_smbus_write_byte_data(info->client,
						LT8491_CTRL_UPDATE_TELEM_REG,
						LT8491_UPDATE_TELEM_CMD);
			if (ret)
				goto unlock;
		}

		ret = i2c_smbus_read_word_data(info->client, LT8491_TELE_IOUT_REG);
		if (ret < 0)
			goto unlock;

		mutex_unlock(&info->lock);

		val->intval = ret;

		return 0;
	case POWER_SUPPLY_PROP_POWER_NOW:
		mutex_lock(&info->lock);

		ret = i2c_smbus_read_byte_data(info->client, LT8491_STAT_CHARGER_REG);
		if (ret < 0)
			goto unlock;

		if (!FIELD_GET(LT8491_TELEM_ACTIVE_MASK, ret)) {
			ret = i2c_smbus_write_byte_data(info->client,
						LT8491_CTRL_UPDATE_TELEM_REG,
						LT8491_UPDATE_TELEM_CMD);
			if (ret)
				goto unlock;
		}

		ret = i2c_smbus_read_word_data(info->client, LT8491_TELE_POUT_REG);
		if (ret < 0)
			goto unlock;

		mutex_unlock(&info->lock);

		val->intval = ret * 10000;

		return 0;
	case POWER_SUPPLY_PROP_TEMP:
		mutex_lock(&info->lock);

		ret = i2c_smbus_read_byte_data(info->client, LT8491_STAT_CHARGER_REG);
		if (ret < 0)
			goto unlock;

		if (!FIELD_GET(LT8491_TELEM_ACTIVE_MASK, ret)) {
			ret = i2c_smbus_write_byte_data(info->client,
						LT8491_CTRL_UPDATE_TELEM_REG,
						LT8491_UPDATE_TELEM_CMD);
			if (ret)
				goto unlock;
		}

		ret = i2c_smbus_read_word_data(info->client, LT8491_TELE_TBAT_REG);
		if (ret < 0)
			goto unlock;

		mutex_unlock(&info->lock);

		val->intval = sign_extend32(ret, 15);

		return 0;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = i2c_smbus_read_byte_data(info->client, LT8491_CFG_TBAT_MIN_REG);
		if (ret < 0)
			return ret;

		val->intval = sign_extend32(ret, 7) * 10;

		return 0;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = i2c_smbus_read_byte_data(info->client, LT8491_CFG_TBAT_MAX_REG);
		if (ret < 0)
			return ret;

		val->intval = sign_extend32(ret, 7) * 10;

		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "lt8491";

		return 0;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Analog Devices";

		return 0;
	case POWER_SUPPLY_PROP_SERIAL_NUMBER:
		val->strval = info->serial_number;

		return 0;
	default:
		return -EINVAL;
	}

unlock:
	mutex_unlock(&info->lock);
	return ret;
}

static int lt8491_set_property(struct power_supply *psy,
				enum power_supply_property psp,
				const union power_supply_propval *val)
{
	struct lt8491_info *info = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		return i2c_smbus_write_byte_data(info->client,
						 LT8491_CFG_TBAT_MIN_REG,
						 val->intval / 10);
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		return i2c_smbus_write_byte_data(info->client,
						 LT8491_CFG_TBAT_MAX_REG,
						 val->intval / 10);
	default:
		return -EINVAL;
	}
}

static int lt8491_property_is_writeable(struct power_supply *psy,
					enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property lt8491_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_POWER_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

static const struct power_supply_desc lt8491_desc = {
	.name		= "lt8491",
	.type		= POWER_SUPPLY_TYPE_BATTERY,
	.properties	= lt8491_properties,
	.num_properties	= ARRAY_SIZE(lt8491_properties),
	.get_property	= lt8491_get_property,
	.set_property	= lt8491_set_property,
	.property_is_writeable = lt8491_property_is_writeable,
};

static int lt8491_configure_resistor(struct lt8491_info *info,
				     const char *propname, int divider,
				     unsigned int reg)
{
	struct device *dev = &info->client->dev;
	int ret;
	u32 val;

	ret = device_property_read_u32(dev, propname, &val);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Missing %s property.\n", propname);

	return i2c_smbus_write_word_data(info->client, reg, val / divider);
}

static int lt8491_configure_telemetry(struct lt8491_info *info)
{
	int ret;

	ret = lt8491_configure_resistor(info, "adi,rsense1-micro-ohms", 10,
					LT8491_CFG_RSENSE1_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rimon-out-ohms", 10,
					LT8491_CFG_RIMON_OUT_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rsense2-micro-ohms", 10,
					LT8491_CFG_RSENSE2_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rdaco-ohms", 10,
					LT8491_CFG_RDACO_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rfbout1-ohms", 100,
					LT8491_CFG_RFBOUT1_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rfbout2-ohms", 10,
					LT8491_CFG_RFBOUT2_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rdaci-ohms", 10,
					LT8491_CFG_RDACI_REG);
	if (ret)
		return ret;

	ret = lt8491_configure_resistor(info, "adi,rfbin2-ohms", 10,
					LT8491_CFG_RFBIN2_REG);
	if (ret)
		return ret;

	return lt8491_configure_resistor(info, "adi,rfbin1-ohms", 100,
					 LT8491_CFG_RFBIN1_REG);
}

static int lt8491_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct lt8491_info *info;
	struct power_supply_config psy_cfg = {};
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EOPNOTSUPP;

	info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	info->client = client;
	psy_cfg.drv_data = info;

	mutex_init(&info->lock);

	ret = lt8491_read_serial_number(info);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Can't read serial. Hardware error.\n");

	ret = lt8491_configure_telemetry(info);
	if (ret)
		return ret;

	info->psp = power_supply_register(dev, &lt8491_desc, &psy_cfg);
	if (IS_ERR(info->psp))
		return dev_err_probe(dev, PTR_ERR(info->psp),
				     "Failed to register power supply.\n");

	return 0;
}

static const struct i2c_device_id lt8491_id[] = {
	{ "lt8491", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lt8491_id);

static const struct of_device_id lt8491_of_match[] = {
	{ .compatible = "adi,lt8491" },
	{ }
};
MODULE_DEVICE_TABLE(of, lt8491_of_match);

static struct i2c_driver lt8491_driver = {
	.driver = {
		.name = "lt8491",
		.of_match_table = lt8491_of_match,
	},
	.probe_new = lt8491_probe,
	.id_table = lt8491_id,
};
module_i2c_driver(lt8491_driver);

MODULE_AUTHOR("John Erasmus Mari Geronimo <johnerasmusmari.geronimo@analog.com");
MODULE_DESCRIPTION("LT8491 battery charger");
MODULE_LICENSE("GPL");
