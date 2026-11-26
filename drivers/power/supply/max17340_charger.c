// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices MAX17340 Charger Driver
 *
 * Copyright (C) 2024-2026 Analog Devices, Inc.
 */

#include <linux/mfd/max17340.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

struct max17340_charger {
	struct device *dev;
	struct max17340_dev *mfd;
	struct power_supply *psy;
};

static int max17340_charger_get_health(struct max17340_charger *chg, int *health)
{
	unsigned int protstatus;
	int ret;

	ret = regmap_read(chg->mfd->regmap, MAX17340_PROTSTATUS, &protstatus);
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

	*health = POWER_SUPPLY_HEALTH_GOOD;
	return 0;
}

static int max17340_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max17340_charger *chg = power_supply_get_drvdata(psy);
	struct regmap *regmap = chg->mfd->regmap;
	u32 rsense = chg->mfd->rsense;
	unsigned int reg, chgstat, protstatus;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(regmap, MAX17340_CHGSTAT, &chgstat);
		if (ret)
			return ret;
		val->intval = !!(chgstat & (MAX17340_CHGSTAT_CC |
					    MAX17340_CHGSTAT_CV |
					    MAX17340_CHGSTAT_PREQ));
		return 0;

	case POWER_SUPPLY_PROP_STATUS:
		ret = regmap_read(regmap, MAX17340_PROTSTATUS, &protstatus);
		if (ret)
			return ret;
		ret = regmap_read(regmap, MAX17340_CHGSTAT, &chgstat);
		if (ret)
			return ret;

		if (protstatus & MAX17340_PROT_FULL)
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (chgstat & (MAX17340_CHGSTAT_CC | MAX17340_CHGSTAT_CV))
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (protstatus & MAX17340_PROT_BLOCKCHG)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(regmap, MAX17340_CHGSTAT, &chgstat);
		if (ret)
			return ret;

		if (chgstat & MAX17340_CHGSTAT_PREQ)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (chgstat & (MAX17340_CHGSTAT_CC | MAX17340_CHGSTAT_CV))
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;

	case POWER_SUPPLY_PROP_HEALTH:
		return max17340_charger_get_health(chg, &val->intval);

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(regmap, MAX17340_CHARGINGCURRENT, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_ua(rsense, reg);
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(regmap, MAX17340_CHARGINGVOLTAGE, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_uv(reg);
		return 0;

	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = regmap_read(regmap, MAX17340_ICHGTERM, &reg);
		if (ret)
			return ret;
		val->intval = max17340_raw_to_ua(rsense, reg);
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

static enum power_supply_property max17340_charger_props[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static const struct power_supply_desc max17340_charger_desc = {
	.name = "max17340-charger",
	.type = POWER_SUPPLY_TYPE_MAINS,
	.properties = max17340_charger_props,
	.num_properties = ARRAY_SIZE(max17340_charger_props),
	.get_property = max17340_charger_get_property,
};

static int max17340_charger_probe(struct platform_device *pdev)
{
	struct max17340_dev *mfd = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct max17340_charger *chg;

	if (!mfd)
		return -ENODEV;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->mfd = mfd;
	platform_set_drvdata(pdev, chg);

	psy_cfg.drv_data = chg;
	psy_cfg.of_node = pdev->dev.of_node;

	chg->psy = devm_power_supply_register(&pdev->dev, &max17340_charger_desc,
					      &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->psy),
				     "Failed to register power supply\n");

	/* Store reference in MFD for IRQ handler notification */
	mfd->charger = chg->psy;

	return 0;
}

static const struct platform_device_id max17340_charger_id[] = {
	{ "max17340-charger", },
	{ }
};
MODULE_DEVICE_TABLE(platform, max17340_charger_id);

static struct platform_driver max17340_charger_driver = {
	.driver = {
		.name = "max17340-charger",
	},
	.probe = max17340_charger_probe,
	.id_table = max17340_charger_id,
};
module_platform_driver(max17340_charger_driver);

MODULE_DESCRIPTION("Analog Devices MAX17340 Charger Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joan Na <joan.na@analog.com>");
