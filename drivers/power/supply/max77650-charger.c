// SPDX-License-Identifier: GPL-2.0
//
// Copyright (C) 2018 BayLibre SAS
// Author: Bartosz Golaszewski <bgolaszewski@baylibre.com>
//
// Battery charger driver for MAXIM 77650/77651 charger/power-supply.

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77650.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>

#define MAX77650_CHARGER_ENABLED		BIT(0)
#define MAX77650_CHARGER_DISABLED		0x00
#define MAX77650_CHARGER_CHG_EN_MASK		BIT(0)

#define MAX77650_CHG_DETAILS_MASK		GENMASK(7, 4)
#define MAX77650_CHG_DETAILS_BITS(_reg) \
		(((_reg) & MAX77650_CHG_DETAILS_MASK) >> 4)

/* Charger is OFF. */
#define MAX77650_CHG_OFF			0x00
/* Charger is in prequalification mode. */
#define MAX77650_CHG_PREQ			0x01
/* Charger is in fast-charge constant current mode. */
#define MAX77650_CHG_ON_CURR			0x02
/* Charger is in JEITA modified fast-charge constant-current mode. */
#define MAX77650_CHG_ON_CURR_JEITA		0x03
/* Charger is in fast-charge constant-voltage mode. */
#define MAX77650_CHG_ON_VOLT			0x04
/* Charger is in JEITA modified fast-charge constant-voltage mode. */
#define MAX77650_CHG_ON_VOLT_JEITA		0x05
/* Charger is in top-off mode. */
#define MAX77650_CHG_ON_TOPOFF			0x06
/* Charger is in JEITA modified top-off mode. */
#define MAX77650_CHG_ON_TOPOFF_JEITA		0x07
/* Charger is done. */
#define MAX77650_CHG_DONE			0x08
/* Charger is JEITA modified done. */
#define MAX77650_CHG_DONE_JEITA			0x09
/* Charger is suspended due to a prequalification timer fault. */
#define MAX77650_CHG_SUSP_PREQ_TIM_FAULT	0x0a
/* Charger is suspended due to a fast-charge timer fault. */
#define MAX77650_CHG_SUSP_FAST_CHG_TIM_FAULT	0x0b
/* Charger is suspended due to a battery temperature fault. */
#define MAX77650_CHG_SUSP_BATT_TEMP_FAULT	0x0c

#define MAX77650_CHGIN_DETAILS_MASK		GENMASK(3, 2)
#define MAX77650_CHGIN_DETAILS_BITS(_reg) \
		(((_reg) & MAX77650_CHGIN_DETAILS_MASK) >> 2)

#define MAX77650_CHGIN_UNDERVOLTAGE_LOCKOUT	0x00
#define MAX77650_CHGIN_OVERVOLTAGE_LOCKOUT	0x01
#define MAX77650_CHGIN_OKAY			0x11

#define MAX77650_CHARGER_CHG_MASK	BIT(1)
#define MAX77650_CHARGER_CHG_CHARGING(_reg) \
		(((_reg) & MAX77650_CHARGER_CHG_MASK) > 1)

#define MAX77650_CHARGER_VCHGIN_MIN_MASK	0xc0
#define MAX77650_CHARGER_VCHGIN_MIN_SHIFT(_val)	((_val) << 5)

#define MAX77650_CHARGER_ICHGIN_LIM_MASK	0x1c
#define MAX77650_CHARGER_ICHGIN_LIM_SHIFT(_val)	((_val) << 2)

/* Default value for topoff timer, in minutes */
#define MAX77650_CHARGER_TTOPOFF_DEFAULT	30
#define MAX77650_CHARGER_TTOPOFF_STEP		5
#define MAX77650_CHARGER_TTOPOFF_MASK		GENMASK(2, 0)

/* Default value for fast charge timer, in hours */
#define MAX77650_CHARGER_TFAST_CHG_DEFAULT	5
#define MAX77650_CHARGER_TFAST_CHG_MASK	GENMASK(1, 0)

/* Default value for charge current, in microamps*/
#define MAX77650_CHARGER_CHG_CC_DEFAULT		15000
#define MAX77650_CHARGER_CHG_CC_MASK		GENMASK(7, 2)

#define MAX77650_CHARGER_CURRENT_MAX		300000
#define MAX77650_CHARGER_CURRENT_MIN		7500
#define MAX77650_CHARGER_CURRENT_STEP		7500

enum charger_chip_id {
	MAX77650,
	MAX77654,
	MAX77658,
	MAX77659,
	NUM_CHIP_TYPES,
};

struct max77650_charger_data {
	struct regmap *map;
	struct device *dev;
	enum charger_chip_id id;

	unsigned int fast_charge_current_ua;
};

static enum power_supply_property max77650_charger_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_HEALTH,
};

static int max77650_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return 1;
	default:
		return 0;
	}
}

struct chip_info {
	u8 reg_cnfg_chg_b;
	u8 reg_cnfg_chg_c;
	u8 reg_cnfg_chg_e;
};

static const struct chip_info chip[NUM_CHIP_TYPES] = {
	[MAX77650] = {
		.reg_cnfg_chg_b = 0x19,
		.reg_cnfg_chg_c = 0x1A,
		.reg_cnfg_chg_e = 0x1C,
	},
	[MAX77654] = {
		.reg_cnfg_chg_b = 0x21,
		.reg_cnfg_chg_c = 0x22,
		.reg_cnfg_chg_e = 0x24,
	},
	[MAX77658] = {
		.reg_cnfg_chg_b = 0x21,
		.reg_cnfg_chg_c = 0x22,
		.reg_cnfg_chg_e = 0x24,
	},
	[MAX77659] = {
		.reg_cnfg_chg_b = 0x21,
		.reg_cnfg_chg_c = 0x22,
		.reg_cnfg_chg_e = 0x24,
	},
};

static const unsigned int max77650_charger_vchgin_min_table[] = {
	4000000, 4100000, 4200000, 4300000, 4400000, 4500000, 4600000, 4700000
};

static const unsigned int ichgin_lim_table[3][5] = {
	[MAX77650] = {  95000, 190000, 285000, 380000, 475000 },
	[MAX77654] = {  95000, 190000, 285000, 380000, 475000 },
	[MAX77658] = {  475000, 380000, 285000, 190000, 95000 },
};

static int max77650_charger_set_vchgin_min(struct max77650_charger_data *chg,
					   unsigned int val)
{
	int i, rv;

	for (i = 0; i < ARRAY_SIZE(max77650_charger_vchgin_min_table); i++) {
		if (val == max77650_charger_vchgin_min_table[i]) {
			rv = regmap_update_bits(chg->map,
					chip[chg->id].reg_cnfg_chg_b,
					MAX77650_CHARGER_VCHGIN_MIN_MASK,
					MAX77650_CHARGER_VCHGIN_MIN_SHIFT(i));
			if (rv)
				return rv;

			return 0;
		}
	}

	return -EINVAL;
}

static int max77650_charger_get_vchgin_min(struct max77650_charger_data *chg,
					   unsigned int *val)
{
	unsigned int reg_data = 0;
	int ret;

	ret = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg_data);
	if (ret)
		return ret;

	reg_data = FIELD_GET(MAX77650_CHARGER_VCHGIN_MIN_MASK, reg_data);

	*val = max77650_charger_vchgin_min_table[reg_data];

	return 0;
}

static int max77650_charger_set_ichgin_lim(struct max77650_charger_data *chg,
					   unsigned int val)
{
	int i, rv;

	for (i = 0; i < ARRAY_SIZE(ichgin_lim_table[chg->id]); i++) {
		if (val == ichgin_lim_table[chg->id][i]) {
			rv = regmap_update_bits(chg->map,
					chip[chg->id].reg_cnfg_chg_b,
					MAX77650_CHARGER_ICHGIN_LIM_MASK,
					MAX77650_CHARGER_ICHGIN_LIM_SHIFT(i));
			if (rv)
				return rv;

			return 0;
		}
	}

	return -EINVAL;
}

static int max77650_charger_get_ichgin_lim(struct max77650_charger_data *chg,
					   unsigned int *val)
{
	unsigned int reg_data = 0;
	int ret;

	ret = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg_data);
	if (ret)
		return ret;

	reg_data = FIELD_GET(MAX77650_CHARGER_ICHGIN_LIM_MASK, reg_data);

	*val = ichgin_lim_table[chg->id][reg_data];

	return 0;
}

static int max77650_set_charge_current(struct max77650_charger_data *chg,
				       int fast_charge_current_ua)
{
	unsigned int reg_data;

	fast_charge_current_ua = clamp_val(fast_charge_current_ua,
					   MAX77650_CHARGER_CURRENT_MIN,
					   MAX77650_CHARGER_CURRENT_MAX);
	reg_data = fast_charge_current_ua / MAX77650_CHARGER_CURRENT_STEP - 1;
	reg_data = FIELD_PREP(MAX77650_CHARGER_CHG_CC_MASK, reg_data);

	return regmap_update_bits(chg->map, chip[chg->id].reg_cnfg_chg_e,
				  MAX77650_CHARGER_CHG_CC_MASK, reg_data);
}

static int max77650_get_charge_current(struct max77650_charger_data *chg,
				       int *get_current)
{
	unsigned int reg_data, current_val;
	int ret;

	ret = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_e, &reg_data);
	if (ret)
		return ret;

	reg_data = FIELD_GET(MAX77650_CHARGER_CHG_CC_MASK, reg_data);
	current_val = (reg_data + 1) * MAX77650_CHARGER_CURRENT_STEP;

	*get_current = clamp_val(current_val, MAX77650_CHARGER_CURRENT_MIN,
				 MAX77650_CHARGER_CURRENT_MAX);

	return 0;
}

static ssize_t device_attr_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count,
				 int (*fn)(struct max77650_charger_data *,
					   unsigned long))
{
	struct max77650_charger_data *chg = dev_get_drvdata(dev);
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return ret;

	ret = fn(chg, val);
	if (ret)
		return ret;

	return count;
}

static int max77650_set_topoff_timer(struct max77650_charger_data *chg,
				     unsigned long minutes)
{
	unsigned int reg_data;

	minutes = clamp_val(minutes, 0, MAX77650_CHARGER_TTOPOFF_MASK
			    * MAX77650_CHARGER_TTOPOFF_STEP);
	reg_data = FIELD_PREP(MAX77650_CHARGER_TTOPOFF_MASK,
			      minutes / MAX77650_CHARGER_TTOPOFF_STEP);

	return regmap_update_bits(chg->map, chip[chg->id].reg_cnfg_chg_c,
				  MAX77650_CHARGER_TTOPOFF_MASK, reg_data);
}

static ssize_t topoff_timer_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	return device_attr_store(dev, attr, buf, count,
				 max77650_set_topoff_timer);
}

static ssize_t topoff_timer_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct max77650_charger_data *chg = dev_get_drvdata(dev);
	unsigned int reg_data, minutes;
	int ret;

	ret = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_c,
			  &reg_data);
	if (ret)
		return ret;

	minutes = FIELD_GET(MAX77650_CHARGER_TTOPOFF_MASK, reg_data)
		  * MAX77650_CHARGER_TTOPOFF_STEP;

	return scnprintf(buf, PAGE_SIZE, "%u\n", minutes);
}

static int max77650_set_fast_charge_timer(struct max77650_charger_data *chg,
					  unsigned long hours)
{
	unsigned int val;

	if (hours == 0 || hours > 7)
		val = 0x00;
	else if (hours < 3)
		val = 0x01;
	else
		val = DIV_ROUND_UP(hours - 1, 2);

	return regmap_update_bits(chg->map, chip[chg->id].reg_cnfg_chg_e,
				 MAX77650_CHARGER_TFAST_CHG_MASK, val);
}

static ssize_t fast_charge_timer_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	return device_attr_store(dev, attr, buf, count,
				 max77650_set_fast_charge_timer);
}

static ssize_t fast_charge_timer_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct max77650_charger_data *chg = dev_get_drvdata(dev);
	unsigned int reg_data, hours;
	int ret;

	ret = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_e,
			  &reg_data);
	if (ret)
		return ret;

	reg_data = FIELD_GET(MAX77650_CHARGER_TFAST_CHG_MASK, reg_data);

	if (reg_data == 0)
		hours = 0;
	else
		hours = reg_data * 2 + 1;

	return scnprintf(buf, PAGE_SIZE, "%u\n", hours);
}

static DEVICE_ATTR_RW(fast_charge_timer);
static DEVICE_ATTR_RW(topoff_timer);

static int max77650_charger_enable(struct max77650_charger_data *chg)
{
	int rv;

	rv = regmap_update_bits(chg->map,
				chip[chg->id].reg_cnfg_chg_b,
				MAX77650_CHARGER_CHG_EN_MASK,
				MAX77650_CHARGER_ENABLED);
	if (rv)
		dev_err(chg->dev, "unable to enable the charger: %d\n", rv);

	return rv;
}

static int max77650_charger_disable(struct max77650_charger_data *chg)
{
	int rv;

	rv = regmap_update_bits(chg->map,
				chip[chg->id].reg_cnfg_chg_b,
				MAX77650_CHARGER_CHG_EN_MASK,
				MAX77650_CHARGER_DISABLED);
	if (rv)
		dev_err(chg->dev, "unable to disable the charger: %d\n", rv);

	return rv;
}

static irqreturn_t max77650_charger_check_status(int irq, void *data)
{
	struct max77650_charger_data *chg = data;
	int rv, reg;

	rv = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg);
	if (rv) {
		dev_err(chg->dev,
			"unable to read the charger status: %d\n", rv);
		return IRQ_HANDLED;
	}

	switch (MAX77650_CHGIN_DETAILS_BITS(reg)) {
	case MAX77650_CHGIN_UNDERVOLTAGE_LOCKOUT:
		dev_err(chg->dev, "undervoltage lockout detected, disabling charger\n");
		max77650_charger_disable(chg);
		break;
	case MAX77650_CHGIN_OVERVOLTAGE_LOCKOUT:
		dev_err(chg->dev, "overvoltage lockout detected, disabling charger\n");
		max77650_charger_disable(chg);
		break;
	case MAX77650_CHGIN_OKAY:
		max77650_charger_enable(chg);
		break;
	default:
		/* May be 0x10 - debouncing */
		break;
	}

	return IRQ_HANDLED;
}

static int max77650_charger_get_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 union power_supply_propval *val)
{
	struct max77650_charger_data *chg = power_supply_get_drvdata(psy);
	int rv, reg;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		rv = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg);
		if (rv)
			return rv;

		if (MAX77650_CHARGER_CHG_CHARGING(reg)) {
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		}

		switch (MAX77650_CHG_DETAILS_BITS(reg)) {
		case MAX77650_CHG_OFF:
		case MAX77650_CHG_SUSP_PREQ_TIM_FAULT:
		case MAX77650_CHG_SUSP_FAST_CHG_TIM_FAULT:
		case MAX77650_CHG_SUSP_BATT_TEMP_FAULT:
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
			break;
		case MAX77650_CHG_PREQ:
		case MAX77650_CHG_ON_CURR:
		case MAX77650_CHG_ON_CURR_JEITA:
		case MAX77650_CHG_ON_VOLT:
		case MAX77650_CHG_ON_VOLT_JEITA:
		case MAX77650_CHG_ON_TOPOFF:
		case MAX77650_CHG_ON_TOPOFF_JEITA:
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
			break;
		case MAX77650_CHG_DONE:
			val->intval = POWER_SUPPLY_STATUS_FULL;
			break;
		default:
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		rv = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg);
		if (rv)
			return rv;

		val->intval = MAX77650_CHARGER_CHG_CHARGING(reg);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		rv = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg);
		if (rv)
			return rv;

		if (!MAX77650_CHARGER_CHG_CHARGING(reg)) {
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}

		switch (MAX77650_CHG_DETAILS_BITS(reg)) {
		case MAX77650_CHG_PREQ:
		case MAX77650_CHG_ON_CURR:
		case MAX77650_CHG_ON_CURR_JEITA:
		case MAX77650_CHG_ON_VOLT:
		case MAX77650_CHG_ON_VOLT_JEITA:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		case MAX77650_CHG_ON_TOPOFF:
		case MAX77650_CHG_ON_TOPOFF_JEITA:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		rv = regmap_read(chg->map, chip[chg->id].reg_cnfg_chg_b, &reg);
		if (rv)
			return rv;

		if (!MAX77650_CHARGER_CHG_CHARGING(reg)) {
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}

		switch (MAX77650_CHG_DETAILS_BITS(reg)) {
		case MAX77650_CHG_PREQ:
		case MAX77650_CHG_ON_CURR:
		case MAX77650_CHG_ON_CURR_JEITA:
		case MAX77650_CHG_ON_VOLT:
		case MAX77650_CHG_ON_VOLT_JEITA:
		case MAX77650_CHG_ON_TOPOFF:
		case MAX77650_CHG_ON_TOPOFF_JEITA:
		case MAX77650_CHG_DONE:
		case MAX77650_CHG_DONE_JEITA:
		val->intval = POWER_SUPPLY_HEALTH_GOOD;
			break;
		case MAX77650_CHG_SUSP_PREQ_TIM_FAULT:
		val->intval = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
			break;
		case MAX77650_CHG_SUSP_BATT_TEMP_FAULT:
			val->intval = POWER_SUPPLY_HEALTH_HOT;
		break;
		default:
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		}
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return max77650_get_charge_current(chg, &val->intval);
	default:
		return -EINVAL;
	}

	return 0;
}

static int max77650_charger_set_property(struct power_supply *psy,
					 enum power_supply_property psp,
					 const union power_supply_propval *val)
{
	struct max77650_charger_data *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_update_bits(chg->map,
					 chip[chg->id].reg_cnfg_chg_b,
					 MAX77650_CHARGER_ENABLED,
					 !!val->intval);
		if (ret)
			return ret;

		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		/* val->intval - uA */
		ret = max77650_set_charge_current(chg, val->intval);
		if (ret)
			return ret;

		chg->fast_charge_current_ua = val->intval;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	return ret;
}

static const struct power_supply_desc max77650_battery_desc = {
	.name		= "max77650",
	.type		= POWER_SUPPLY_TYPE_USB,
	.get_property	= max77650_charger_get_property,
	.set_property	= max77650_charger_set_property,
	.properties	= max77650_charger_properties,
	.num_properties	= ARRAY_SIZE(max77650_charger_properties),
	.property_is_writeable = max77650_property_is_writeable,
};

static int max77650_charger_probe(struct platform_device *pdev)
{
	struct power_supply_config pscfg = {};
	struct max77650_charger_data *chg;
	struct power_supply *battery;
	struct device *dev, *parent;
	int rv, chg_irq, chgin_irq;
	unsigned int prop;

	dev = &pdev->dev;
	parent = dev->parent;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	platform_set_drvdata(pdev, chg);

	chg->map = dev_get_regmap(parent, NULL);
	if (!chg->map)
		return -ENODEV;

	chg->dev = dev;

	pscfg.of_node = dev->of_node;
	pscfg.drv_data = chg;

	chg->id = (enum charger_chip_id)(uintptr_t)device_get_match_data(dev);

	chg_irq = platform_get_irq_byname(pdev, "CHG");
	if (chg_irq < 0)
		return chg_irq;

	chgin_irq = platform_get_irq_byname(pdev, "CHGIN");
	if (chgin_irq < 0)
		return chgin_irq;

	rv = devm_request_any_context_irq(dev, chg_irq,
					  max77650_charger_check_status,
					  IRQF_ONESHOT, "chg", chg);
	if (rv < 0)
		return rv;

	rv = devm_request_any_context_irq(dev, chgin_irq,
					  max77650_charger_check_status,
					  IRQF_ONESHOT, "chgin", chg);
	if (rv < 0)
		return rv;

	battery = devm_power_supply_register(dev,
					     &max77650_battery_desc, &pscfg);
	if (IS_ERR(battery))
		return PTR_ERR(battery);

	if (chg->id != MAX77659) {
		rv = of_property_read_u32(dev->of_node,
					  "input-current-limit-microamp",
					  &prop);
		if (rv == 0) {
			rv = max77650_charger_set_ichgin_lim(chg, prop);
			if (rv)
				return rv;
		}

		rv = of_property_read_u32(dev->of_node,
					  "input-voltage-min-microvolt", &prop);
		if (rv == 0) {
			rv = max77650_charger_set_vchgin_min(chg, prop);
			if (rv)
				return rv;
		}
	}

	rv = max77650_set_fast_charge_timer(chg,
					    MAX77650_CHARGER_TFAST_CHG_DEFAULT);
	if (rv)
		return dev_err_probe(dev, rv,
				     "unable to set fast charge timer\n");

	rv = max77650_set_topoff_timer(chg, MAX77650_CHARGER_TTOPOFF_DEFAULT);
	if (rv)
		return dev_err_probe(dev, rv,
				     "unable to set topoff timer\n");

	rv = device_create_file(dev, &dev_attr_fast_charge_timer);
	if (rv)
		return dev_err_probe(dev, rv,
				     "unable to create fast charge timer sysfs entry\n");

	rv = device_create_file(dev, &dev_attr_topoff_timer);
	if (rv)
		return dev_err_probe(dev, rv,
				     "unable to create topoff timer sysfs entry\n");

	return max77650_charger_enable(chg);
}

static int max77650_charger_remove(struct platform_device *pdev)
{
	struct max77650_charger_data *chg = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_topoff_timer);
	device_remove_file(&pdev->dev, &dev_attr_fast_charge_timer);

	return max77650_charger_disable(chg);
}

static const struct of_device_id max77650_charger_of_match[] = {
	{ .compatible = "maxim,max77650-charger", (void *)MAX77650 },
	{ .compatible = "adi,max77654-charger", (void *)MAX77654 },
	{ .compatible = "adi,max77658-charger", (void *)MAX77658 },
	{ .compatible = "adi,max77659-charger", (void *)MAX77659 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, max77650_charger_of_match);

static struct platform_driver max77650_charger_driver = {
	.driver = {
		.name = "max77650-charger",
		.of_match_table = max77650_charger_of_match,
	},
	.probe = max77650_charger_probe,
	.remove = max77650_charger_remove,
};

module_platform_driver(max77650_charger_driver);

MODULE_DESCRIPTION("MAXIM 77650/77651/77654/77658/77659 charger driver");
MODULE_AUTHOR("Bartosz Golaszewski <bgolaszewski@baylibre.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:max77650-charger");

