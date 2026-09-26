// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Analog Devices MAX20357 Li-ion charger driver
 *
 * The MAX20357 is the slave (earbud) PMIC with an integrated charger.
 * Charge parameters are obtained from a monitored-battery DT phandle.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>

#include "../../mfd/maxim/max2035x/max2035x.h"
#include "../../mfd/maxim/max2035x/max2035x_registers.h"

struct max20357_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
};

static int max20357_charger_get_status(struct max20357_charger *chg,
				       union power_supply_propval *val)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(chg->regmap, MAX20357_REG_STATUS1, &reg_val);
	if (ret)
		return ret;

	switch (reg_val & MAX20357_STATUS1_CHGSTAT_MASK) {
	case 0x0:
	case 0x1:
		val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 0x2:
	case 0x3:
	case 0x4:
	case 0x5:
	case 0x6:
		val->intval = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 0x7:
		val->intval = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	}

	return 0;
}

static int max20357_charger_get_charge_type(struct max20357_charger *chg,
					    union power_supply_propval *val)
{
	unsigned int reg_val;
	int ret;

	ret = regmap_read(chg->regmap, MAX20357_REG_STATUS1, &reg_val);
	if (ret)
		return ret;

	switch (reg_val & MAX20357_STATUS1_CHGSTAT_MASK) {
	case 0x2:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case 0x3:
	case 0x4:
	case 0x5:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		break;
	case 0x6:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_STANDARD;
		break;
	default:
		val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	return 0;
}

static int max20357_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max20357_charger *chg = power_supply_get_drvdata(psy);
	unsigned int reg_val;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return max20357_charger_get_status(chg, val);

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		return max20357_charger_get_charge_type(chg, val);

	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = 1;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = regmap_read(chg->regmap, MAX20357_REG_CHG_CUR0, &reg_val);
		if (ret)
			return ret;
		val->intval = (reg_val & MAX20357_CHG_CUR0_CC1IFCHG_MASK) * 7500;
		return 0;

	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = regmap_read(chg->regmap, MAX20357_REG_CHG_CNTL1, &reg_val);
		if (ret)
			return ret;
		val->intval = 3600000 + (reg_val & MAX20357_CHG_CNTL1_BATREG_MASK) * 25000;
		return 0;

	default:
		return -EINVAL;
	}
}

static const enum power_supply_property max20357_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
};

/* desc is allocated in probe to allow per-instance naming */

static int max20357_charger_configure(struct max20357_charger *chg,
				      struct power_supply_battery_info *info)
{
	/* Enable step charging and auto stop/restart */
	regmap_update_bits(chg->regmap, MAX20357_REG_CHG_CNTL0,
			   MAX20357_CHG_CNTL0_CHG_AUTOSTOP_BIT |
			   MAX20357_CHG_CNTL0_CHG_AUTORESTA_BIT |
			   MAX20357_CHG_CNTL0_CC1_ENABLE_BIT,
			   MAX20357_CHG_CNTL0_CHG_AUTOSTOP_BIT |
			   MAX20357_CHG_CNTL0_CHG_AUTORESTA_BIT |
			   MAX20357_CHG_CNTL0_CC1_ENABLE_BIT);

	/* Enable CC tracking for clean PLC line */
	regmap_update_bits(chg->regmap, MAX20357_REG_CHG_CTR1,
			   MAX20357_CHG_CTR1_CHG_CC_TRK_BIT,
			   MAX20357_CHG_CTR1_CHG_CC_TRK_BIT);

	/* Enable JEITA thermistor monitoring (continuous when PLC present) */
	regmap_update_bits(chg->regmap, MAX20357_REG_THM_CFG7,
			   MAX20357_THM_CFG7_THMEN_MASK, 0x2);

	return 0;
}

static int max20357_charger_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_battery_info *bat_info;
	struct power_supply_config psy_cfg = {};
	struct max20357_charger *chg;
	int ret;

	chg = devm_kzalloc(&pdev->dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	struct power_supply_desc *desc;

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	chg->dev = &pdev->dev;
	chg->regmap = chip->regmap;

	desc->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s-charger",
				    dev_name(&pdev->dev));
	if (!desc->name)
		return -ENOMEM;

	desc->type = POWER_SUPPLY_TYPE_MAINS;
	desc->properties = max20357_charger_props;
	desc->num_properties = ARRAY_SIZE(max20357_charger_props);
	desc->get_property = max20357_charger_get_property;

	psy_cfg.drv_data = chg;
	psy_cfg.fwnode = dev_fwnode(chip->dev);

	chg->psy = devm_power_supply_register(&pdev->dev, desc, &psy_cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(chg->psy),
				     "Failed to register power supply\n");

	ret = power_supply_get_battery_info(chg->psy, &bat_info);
	if (!ret) {
		max20357_charger_configure(chg, bat_info);
		power_supply_put_battery_info(chg->psy, bat_info);
	}

	/* Enable charger */
	regmap_update_bits(chg->regmap, MAX20357_REG_CHG_CNTL0,
			   MAX20357_CHG_CNTL0_CHG_EN_BIT,
			   MAX20357_CHG_CNTL0_CHG_EN_BIT);

	platform_set_drvdata(pdev, chg);

	dev_info(&pdev->dev, "MAX20357 charger driver probed\n");

	return 0;
}

static const struct platform_device_id max20357_charger_id[] = {
	{ "max20357-charger" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max20357_charger_id);

static struct platform_driver max20357_charger_driver = {
	.driver = {
		.name = "max20357-charger",
	},
	.probe = max20357_charger_probe,
	.id_table = max20357_charger_id,
};
module_platform_driver(max20357_charger_driver);

MODULE_DESCRIPTION("Analog Devices MAX20357 charger driver");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
