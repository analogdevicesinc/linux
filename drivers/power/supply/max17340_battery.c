// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices MAX17340 Battery (Fuel Gauge) Driver
 *
 * Copyright (C) 2024 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/mfd/max17340.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

/* VEMPTY register fields */
#define MAX17340_VEMPTY_VE_MASK		GENMASK(15, 7)
#define MAX17340_VEMPTY_VR_MASK		GENMASK(6, 0)

/* REPSOC register fields */
#define MAX17340_REPSOC_PERCENT_MASK	GENMASK(15, 8)

/* TALRTTH register fields */
#define MAX17340_TALRTTH_MIN_MASK	GENMASK(7, 0)
#define MAX17340_TALRTTH_MAX_MASK	GENMASK(15, 8)

struct max17340_battery {
	struct device *dev;
	struct max17340_dev *mfd;
	struct power_supply *psy;
};

static int max17340_get_status(struct max17340_battery *bat, int *status)
{
	unsigned int chgstat, protstatus;
	int ret;

	ret = regmap_read(bat->mfd->regmap, MAX17340_PROTSTATUS, &protstatus);
	if (ret)
		return ret;

	ret = regmap_read(bat->mfd->regmap, MAX17340_CHGSTAT, &chgstat);
	if (ret)
		return ret;

	if (protstatus & MAX17340_PROT_FULL) {
		*status = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	if (chgstat & (MAX17340_CHGSTAT_CC | MAX17340_CHGSTAT_CV |
		       MAX17340_CHGSTAT_PREQ)) {
		*status = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (protstatus & (MAX17340_PROT_BLOCKCHG | MAX17340_PROT_TOOHOTC |
			  MAX17340_PROT_TOOCOLDC)) {
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	}

	*status = POWER_SUPPLY_STATUS_DISCHARGING;
	return 0;
}

static int max17340_get_charge_type(struct max17340_battery *bat, int *type)
{
	unsigned int chgstat;
	int ret;

	ret = regmap_read(bat->mfd->regmap, MAX17340_CHGSTAT, &chgstat);
	if (ret)
		return ret;

	if (chgstat & MAX17340_CHGSTAT_PREQ) {
		*type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		return 0;
	}

	if (chgstat & (MAX17340_CHGSTAT_CC | MAX17340_CHGSTAT_CV)) {
		*type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		return 0;
	}

	*type = POWER_SUPPLY_CHARGE_TYPE_NONE;
	return 0;
}

static int max17340_get_health(struct max17340_battery *bat, int *health)
{
	unsigned int protstatus;
	int ret;

	ret = regmap_read(bat->mfd->regmap, MAX17340_PROTSTATUS, &protstatus);
	if (ret)
		return ret;

	if (protstatus & MAX17340_PROT_OVP) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	if (protstatus & MAX17340_PROT_UVP) {
		*health = POWER_SUPPLY_HEALTH_DEAD;
		return 0;
	}

	if (protstatus & (MAX17340_PROT_TOOHOTC | MAX17340_PROT_TOOHOTD |
			  MAX17340_PROT_DIEHOT)) {
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
		return 0;
	}

	if (protstatus & MAX17340_PROT_TOOCOLDC) {
		*health = POWER_SUPPLY_HEALTH_COLD;
		return 0;
	}

	if (protstatus & (MAX17340_PROT_OCCP | MAX17340_PROT_ODCP)) {
		*health = POWER_SUPPLY_HEALTH_OVERCURRENT;
		return 0;
	}

	*health = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int max17340_battery_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max17340_battery *bat = power_supply_get_drvdata(psy);
	struct regmap *regmap = bat->mfd->regmap;
	struct regmap *regmap_nvm = bat->mfd->regmap_nvm;
	u32 rsense = bat->mfd->rsense;
	unsigned int reg;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return max17340_get_status(bat, &val->intval);

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return max17340_get_charge_type(bat, &val->intval);

	case POWER_SUPPLY_PROP_HEALTH:
		return max17340_get_health(bat, &val->intval);

	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(regmap, MAX17340_STATUS, &reg);
		if (ret)
			return ret;
		val->intval = !(reg & MAX17340_STATUS_BST);
		return 0;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;

	case POWER_SUPPLY_PROP_CYCLE_COUNT:
		ret = regmap_read(regmap, MAX17340_CYCLES, &reg);
		if (ret)
			return ret;
		val->intval = reg / 4;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = regmap_read(regmap, MAX17340_VCELL, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uv(reg);
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(regmap, MAX17340_AVGVCELL, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uv(reg);
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		ret = regmap_read(regmap_nvm, MAX17340_NVM_DESIGNVOLTAGE, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uv(reg);
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = regmap_read(regmap_nvm, MAX17340_NVM_VEMPTY, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(MAX17340_VEMPTY_VE_MASK, reg) * 10000;
		return 0;

	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = regmap_read(regmap, MAX17340_CURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_ua(rsense, (s16)reg);
		return 0;

	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = regmap_read(regmap, MAX17340_AVGCURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_ua(rsense, (s16)reg);
		return 0;

	case POWER_SUPPLY_PROP_CAPACITY:
		ret = regmap_read(regmap, MAX17340_REPSOC, &reg);
		if (ret)
			return ret;
		val->intval = FIELD_GET(MAX17340_REPSOC_PERCENT_MASK, reg);
		return 0;

	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		ret = regmap_read(regmap, MAX17340_REPSOC, &reg);
		if (ret)
			return ret;
		reg = FIELD_GET(MAX17340_REPSOC_PERCENT_MASK, reg);
		if (reg >= 100)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		else if (reg >= 80)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
		else if (reg >= 20)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
		else if (reg >= 5)
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
		else
			val->intval = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = regmap_read(regmap, MAX17340_FULLCAPREP, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uah(rsense, reg);
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = regmap_read(regmap, MAX17340_DESIGNCAP, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uah(rsense, reg);
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = regmap_read(regmap, MAX17340_REPCAP, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uah(rsense, reg);
		return 0;

	case POWER_SUPPLY_PROP_TEMP:
		ret = regmap_read(regmap, MAX17340_TEMP, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_decidegc((s16)reg);
		return 0;

	case POWER_SUPPLY_PROP_TEMP_MIN:
		ret = regmap_read(regmap, MAX17340_TALRTTH, &reg);
		if (ret)
			return ret;
		val->intval = (s8)FIELD_GET(MAX17340_TALRTTH_MIN_MASK, reg) * 10;
		return 0;

	case POWER_SUPPLY_PROP_TEMP_MAX:
		ret = regmap_read(regmap, MAX17340_TALRTTH, &reg);
		if (ret)
			return ret;
		val->intval = (s8)FIELD_GET(MAX17340_TALRTTH_MAX_MASK, reg) * 10;
		return 0;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = regmap_read(regmap, MAX17340_TTE, &reg);
		if (ret)
			return ret;
		if (reg == 0xFFFF) {
			val->intval = -1;
			return 0;
		}
		val->intval = max17340_raw_to_sec(reg);
		return 0;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		ret = regmap_read(regmap, MAX17340_TTF, &reg);
		if (ret)
			return ret;
		if (reg == 0xFFFF) {
			val->intval = -1;
			return 0;
		}
		val->intval = max17340_raw_to_sec(reg);
		return 0;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "MAX17340";
		return 0;

	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Analog Devices";
		return 0;

	default:
		return -EINVAL;
	}
}

static int max17340_battery_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max17340_battery *bat = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_MIN:
		return regmap_update_bits(bat->mfd->regmap, MAX17340_TALRTTH,
					  MAX17340_TALRTTH_MIN_MASK,
					  FIELD_PREP(MAX17340_TALRTTH_MIN_MASK,
						     val->intval / 10));

	case POWER_SUPPLY_PROP_TEMP_MAX:
		return regmap_update_bits(bat->mfd->regmap, MAX17340_TALRTTH,
					  MAX17340_TALRTTH_MAX_MASK,
					  FIELD_PREP(MAX17340_TALRTTH_MAX_MASK,
						     val->intval / 10));

	default:
		return -EINVAL;
	}
}

static int max17340_battery_prop_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_MIN:
	case POWER_SUPPLY_PROP_TEMP_MAX:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property max17340_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CYCLE_COUNT,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct power_supply_desc max17340_battery_desc = {
	.name = "max17340-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = max17340_battery_props,
	.num_properties = ARRAY_SIZE(max17340_battery_props),
	.get_property = max17340_battery_get_property,
	.set_property = max17340_battery_set_property,
	.property_is_writeable = max17340_battery_prop_writeable,
};

static int max17340_battery_probe(struct platform_device *pdev)
{
	struct max17340_dev *mfd = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct max17340_battery *bat;

	if (!mfd)
		return -ENODEV;

	bat = devm_kzalloc(&pdev->dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	bat->dev = &pdev->dev;
	bat->mfd = mfd;
	platform_set_drvdata(pdev, bat);

	psy_cfg.drv_data = bat;
	psy_cfg.of_node = pdev->dev.of_node;

	bat->psy = devm_power_supply_register(&pdev->dev, &max17340_battery_desc,
					      &psy_cfg);
	if (IS_ERR(bat->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(bat->psy),
				     "Failed to register power supply\n");

	/* Store reference in MFD for IRQ handler notification */
	mfd->battery = bat->psy;

	return 0;
}

static const struct platform_device_id max17340_battery_id[] = {
	{ "max17340-battery", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17340_battery_id);

static struct platform_driver max17340_battery_driver = {
	.driver = {
		.name = "max17340-battery",
	},
	.probe = max17340_battery_probe,
	.id_table = max17340_battery_id,
};
module_platform_driver(max17340_battery_driver);

MODULE_DESCRIPTION("Analog Devices MAX17340 Battery Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joan Na <joan.na@analog.com>");
