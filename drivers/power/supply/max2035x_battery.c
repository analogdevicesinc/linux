// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Battery power supply driver for Analog Devices MAX20355/MAX20357 PMICs
 * Reads fuel gauge data from PLC mirror registers in the main I2C block.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/math64.h>

#include "../../mfd/maxim/max2035x/max2035x.h"
#include "../../mfd/maxim/max2035x/max2035x_registers.h"

struct max2035x_bat_regs {
	u8 soc_byte1;
	u8 soc_byte0;
	u8 vcell_byte1;
	u8 vcell_byte0;
	u8 tte_byte1;
	u8 tte_byte0;
	u8 avgvcell_byte1;
	u8 avgvcell_byte0;
	u8 ttf_byte1;
	u8 ttf_byte0;
	u8 ready_reg;
};

static const struct max2035x_bat_regs max20355_bat_regs = {
	.soc_byte1     = MAX20355_REG_SOC_BYTE_1,
	.soc_byte0     = MAX20355_REG_SOC_BYTE_0,
	.vcell_byte1   = MAX20355_REG_VCELL_BYTE_1,
	.vcell_byte0   = MAX20355_REG_VCELL_BYTE_0,
	.tte_byte1     = MAX20355_REG_TTE_BYTE_1,
	.tte_byte0     = MAX20355_REG_TTE_BYTE_0,
	.avgvcell_byte1 = MAX20355_REG_AVGVCELL_BYTE_1,
	.avgvcell_byte0 = MAX20355_REG_AVGVCELL_BYTE_0,
	.ttf_byte1     = MAX20355_REG_TTF_BYTE_1,
	.ttf_byte0     = MAX20355_REG_TTF_BYTE_0,
	.ready_reg     = MAX20355_REG_READY_REG,
};

static const struct max2035x_bat_regs max20357_bat_regs = {
	.soc_byte1     = MAX20357_REG_SOC_BYTE_1,
	.soc_byte0     = MAX20357_REG_SOC_BYTE_0,
	.vcell_byte1   = MAX20357_REG_VCELL_BYTE_1,
	.vcell_byte0   = MAX20357_REG_VCELL_BYTE_0,
	.tte_byte1     = MAX20357_REG_TTE_BYTE_1,
	.tte_byte0     = MAX20357_REG_TTE_BYTE_0,
	.avgvcell_byte1 = MAX20357_REG_AVGVCELL_BYTE_1,
	.avgvcell_byte0 = MAX20357_REG_AVGVCELL_BYTE_0,
	.ttf_byte1     = MAX20357_REG_TTF_BYTE_1,
	.ttf_byte0     = MAX20357_REG_TTF_BYTE_0,
	.ready_reg     = MAX20357_REG_READY_REG,
};

struct max2035x_battery {
	struct device *dev;
	struct regmap *regmap;
	const struct max2035x_bat_regs *regs;
};

static int max2035x_bat_read_word(struct max2035x_battery *bat,
				  u8 reg_hi, u8 reg_lo, u16 *val)
{
	unsigned int hi, lo;
	int ret;

	ret = regmap_read(bat->regmap, reg_hi, &hi);
	if (ret)
		return ret;

	ret = regmap_read(bat->regmap, reg_lo, &lo);
	if (ret)
		return ret;

	*val = (hi << 8) | lo;
	return 0;
}

static int max2035x_bat_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct max2035x_battery *bat = power_supply_get_drvdata(psy);
	const struct max2035x_bat_regs *regs = bat->regs;
	u16 raw;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = max2035x_bat_read_word(bat, regs->soc_byte1,
					     regs->soc_byte0, &raw);
		if (ret)
			return ret;
		val->intval = raw >> 8;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = max2035x_bat_read_word(bat, regs->vcell_byte1,
					     regs->vcell_byte0, &raw);
		if (ret)
			return ret;
		val->intval = (int)div_s64((s64)raw * 78125, 1000);
		return 0;

	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW:
		ret = max2035x_bat_read_word(bat, regs->tte_byte1,
					     regs->tte_byte0, &raw);
		if (ret)
			return ret;
		val->intval = (raw * 5625) / 1000;
		return 0;

	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = max2035x_bat_read_word(bat, regs->avgvcell_byte1,
					     regs->avgvcell_byte0, &raw);
		if (ret)
			return ret;
		val->intval = (int)div_s64((s64)raw * 78125, 1000);
		return 0;

	case POWER_SUPPLY_PROP_TIME_TO_FULL_NOW:
		ret = max2035x_bat_read_word(bat, regs->ttf_byte1,
					     regs->ttf_byte0, &raw);
		if (ret)
			return ret;
		val->intval = (raw * 5625) / 1000;
		return 0;

	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;

	default:
		return -EINVAL;
	}
}

static enum power_supply_property max2035x_bat_properties[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,
	POWER_SUPPLY_PROP_PRESENT,
};

static int max2035x_battery_probe(struct platform_device *pdev)
{
	struct max2035x *chip = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config psy_cfg = {};
	struct power_supply_desc *desc;
	struct max2035x_battery *bat;

	bat = devm_kzalloc(&pdev->dev, sizeof(*bat), GFP_KERNEL);
	if (!bat)
		return -ENOMEM;

	desc = devm_kzalloc(&pdev->dev, sizeof(*desc), GFP_KERNEL);
	if (!desc)
		return -ENOMEM;

	bat->dev = &pdev->dev;
	bat->regmap = chip->regmap;
	bat->regs = (chip->type == MAX20355) ?
		    &max20355_bat_regs : &max20357_bat_regs;

	desc->name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "%s-battery",
				    dev_name(&pdev->dev));
	if (!desc->name)
		return -ENOMEM;

	desc->type = POWER_SUPPLY_TYPE_BATTERY;
	desc->get_property = max2035x_bat_get_property;
	desc->properties = max2035x_bat_properties;
	desc->num_properties = ARRAY_SIZE(max2035x_bat_properties);

	psy_cfg.drv_data = bat;
	psy_cfg.fwnode = dev_fwnode(&pdev->dev);

	return PTR_ERR_OR_ZERO(devm_power_supply_register(&pdev->dev,
							   desc, &psy_cfg));
}

static const struct platform_device_id max2035x_battery_id[] = {
	{ "max20355-battery" },
	{ "max20357-battery" },
	{ }
};
MODULE_DEVICE_TABLE(platform, max2035x_battery_id);

static struct platform_driver max2035x_battery_driver = {
	.probe = max2035x_battery_probe,
	.driver = {
		.name = "max2035x-battery",
	},
	.id_table = max2035x_battery_id,
};
module_platform_driver(max2035x_battery_driver);

MODULE_DESCRIPTION("Battery driver for Analog Devices MAX20355/MAX20357");
MODULE_AUTHOR("Judy Na <judy.na@analog.com>");
MODULE_LICENSE("GPL");
