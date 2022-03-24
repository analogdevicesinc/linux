// SPDX-License-Identifier: GPL-2.0-or-later

#include <asm/unaligned.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/max77658.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/power/max77658-battery.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/version.h>

static char *batt_supplied_to[] = {
	MAX77658_FUELGAUGE_NAME,
};

static void max77658_fg_get_soc(struct max77658_fg_dev *max77658_fg)
{
	unsigned int soc;
	int ret;

	ret = regmap_read(max77658_fg->regmap, REG_REPSOC, &soc);
	if (ret < 0) {
		dev_err(max77658_fg->dev, "Error in reading register REPSOC\n");
		return;
	}

	max77658_fg->soc = (u16)(soc >> 8);

	if (max77658_fg->soc > MAX77658_BATTERY_FULL) {
		max77658_fg->soc = MAX77658_BATTERY_FULL;
		max77658_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		max77658_fg->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (max77658_fg->soc < MAX77658_BATTERY_CRITICAL) {
		max77658_fg->health = POWER_SUPPLY_HEALTH_DEAD;
		max77658_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else if (max77658_fg->soc < MAX77658_BATTERY_LOW) {
		max77658_fg->health = POWER_SUPPLY_HEALTH_DEAD;
		max77658_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	} else {
		max77658_fg->health = POWER_SUPPLY_HEALTH_GOOD;
		max77658_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static void max77658_fg_work(struct work_struct *work)
{
	struct max77658_fg_dev *max77658_fg;
	unsigned int val;
	int ret;

	max77658_fg = container_of(work, struct max77658_fg_dev, work.work);

	mutex_lock(&max77658_fg->lock);

	ret = regmap_read(max77658_fg->regmap, REG_VCELL, &val);
	if (ret < 0)
		dev_err(max77658_fg->dev, "Error in reading register\n");
	else
		max77658_fg->vcell = (u16)val * 625 / 8; /* 78.125uV per bit */

	max77658_fg_get_soc(max77658_fg);

	if (max77658_fg->vcell != max77658_fg->lasttime_vcell ||
	    max77658_fg->soc != max77658_fg->lasttime_soc) {

		max77658_fg->lasttime_vcell = max77658_fg->vcell;
		max77658_fg->lasttime_soc = max77658_fg->soc;

		power_supply_changed(max77658_fg->battery);
	}

	mutex_unlock(&max77658_fg->lock);

	schedule_delayed_work(&max77658_fg->work, MAX77658_FG_DELAY);
}

static enum power_supply_property max77658_fg_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,
	POWER_SUPPLY_PROP_TIME_TO_FULL_AVG,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_POWER_AVG
};

static int max77658_property_is_writeable(struct power_supply *psy,
					  enum power_supply_property psp)
{
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
	case POWER_SUPPLY_PROP_TEMP_MAX:
	case POWER_SUPPLY_PROP_TEMP_MIN:
		return 1;
	default:
		return 0;
	}

	return ret;
}

static int max77658_fg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77658_fg_dev *max77658_fg = power_supply_get_drvdata(psy);
	unsigned int reg_val;
	int ret = 0;

	mutex_lock(&max77658_fg->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = max77658_fg->soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = max77658_fg->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = max77658_fg->capacity_level;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = max77658_fg->vcell;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = regmap_read(max77658_fg->regmap, REG_AVGVCELL, &reg_val);
		if (ret)
			goto out;
		val->intval = (u16)reg_val * 625 / 8; /* 78.125uV per bit */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		ret = regmap_read(max77658_fg->regmap, REG_MAXMINVOLT, &reg_val);
		if (ret)
			goto out;

		val->intval = reg_val >> 8;
		val->intval *= 20; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN:
		ret = regmap_read(max77658_fg->regmap, REG_MAXMINVOLT, &reg_val);
		if (ret)
			goto out;

		val->intval = reg_val & 0xFF;
		val->intval *= 20; /* Units of LSB = 20mV */
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		ret = regmap_read(max77658_fg->regmap, REG_VEMPTY, &reg_val);
		if (ret)
			goto out;

		val->intval = reg_val >> 7;
		val->intval *= 10; /* Units of LSB = 10mV */
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = regmap_read(max77658_fg->regmap, REG_SALRTTH, &reg_val);
		if (ret)
			goto out;

		val->intval = reg_val & 0xFF;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = regmap_read(max77658_fg->regmap, REG_SALRTTH, &reg_val);
		if (ret)
			goto out;

		val->intval = (reg_val >> 8) & 0xFF;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = regmap_read(max77658_fg->regmap, REG_TEMP, &reg_val);
		if (ret)
			goto out;

		val->intval = sign_extend32(reg_val, 15);
		/* The value is converted into centigrade scale */
		/* Units of LSB = 1 / 256 degree Celsius */
		val->intval = val->intval >> 8;
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_read(max77658_fg->regmap, REG_TALRTTH, &reg_val);
		if (ret)
			goto out;

		val->intval = sign_extend32(reg_val & 0xff, 7);
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_read(max77658_fg->regmap, REG_TALRTTH, &reg_val);
		if (ret)
			goto out;

		val->intval = sign_extend32(reg_val >> 8, 7);
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = regmap_read(max77658_fg->regmap, REG_CURRENT, &reg_val);
		if (ret)
			goto out;

		/* 33.487uA per bit */
		val->intval = sign_extend32(reg_val, 15) * 33487 / 1000;
		/* Convert to milliamps */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = regmap_read(max77658_fg->regmap, REG_AVGCURRENT, &reg_val);
		if (ret)
			goto out;

		/* 33.487uA per bit */
		val->intval = sign_extend32(reg_val, 15) * 33487 / 1000;
		/* Convert to milliamps */
		val->intval /= 1000;
		break;
	case POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG:
		ret = regmap_read(max77658_fg->regmap, REG_TTE, &reg_val);
		if (ret)
			goto out;

		val->intval = (reg_val * 45) >> 3; /* TTE LSB: 5.625 sec */
		break;
	case POWER_SUPPLY_PROP_TIME_TO_FULL_AVG:
		ret = regmap_read(max77658_fg->regmap, REG_TTF, &reg_val);
		if (ret)
			goto out;

		val->intval = (reg_val * 45) >> 3; /* TTF LSB: 5.625 sec */
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		ret = regmap_read(max77658_fg->regmap, REG_MAXMINTEMP, &reg_val);
		if (ret)
			goto out;

		val->intval = sign_extend32(reg_val & 0xff, 7);
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		ret = regmap_read(max77658_fg->regmap, REG_MAXMINTEMP, &reg_val);
		if (ret)
			goto out;

		val->intval = sign_extend32(reg_val >> 8, 7);
		break;
	case POWER_SUPPLY_PROP_POWER_AVG:
		ret = regmap_read(max77658_fg->regmap, REG_AVGPOWER, &reg_val);
		if (ret)
			goto out;

		val->intval = (reg_val * 171); /* REG_POWER LSB: 171uW sec */
		break;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		ret = regmap_read(max77658_fg->regmap, REG_MAXMINCURR, &reg_val);
		if (ret)
			goto out;

		val->intval = sign_extend32(reg_val >> 8, 7);
		val->intval = val->intval * MAX77658_IALRTTH_RESOLUTION;
		break;
	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&max77658_fg->lock);
	return ret;
}

static int max77658_fg_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct max77658_fg_dev *max77658_fg = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&max77658_fg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		ret = regmap_update_bits(max77658_fg->regmap,
					 REG_TALRTTH,
					 BIT_TALRTTH_TMIN,
					 val->intval);
		if (ret)
			dev_err(max77658_fg->dev, "Error in setting TALRTTH\n");
		break;
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		ret = regmap_update_bits(max77658_fg->regmap,
					 REG_TALRTTH,
					 BIT_TALRTTH_TMAX,
					 val->intval);
		if (ret)
			dev_err(max77658_fg->dev, "Error in setting TALRTTH\n");
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		ret = regmap_update_bits(max77658_fg->regmap,
					 REG_SALRTTH,
					 BIT_SALRTTH_SMIN,
					 val->intval);
		if (ret)
			dev_err(max77658_fg->dev, "Error in setting CML\n");
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MAX:
		ret = regmap_update_bits(max77658_fg->regmap,
					 REG_SALRTTH,
					 BIT_SALRTTH_SMAX,
					 val->intval);
		if (ret)
			dev_err(max77658_fg->dev, "Error in setting CML\n");
		break;
	case POWER_SUPPLY_PROP_TEMP_MIN:
		ret = regmap_update_bits(max77658_fg->regmap,
					 REG_MAXMINTEMP,
					 BIT_MAXMINTEMP_MIN,
					 val->intval);
		if (ret)
			dev_err(max77658_fg->dev, "Error in setting TEMP_MIN\n");
		break;
	case POWER_SUPPLY_PROP_TEMP_MAX:
		ret = regmap_update_bits(max77658_fg->regmap,
					 REG_MAXMINTEMP,
					 BIT_MAXMINTEMP_MAX,
					 val->intval);
		if (ret)
			dev_err(max77658_fg->dev, "Error in setting TEMP_MAX\n");
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&max77658_fg->lock);
	return ret;
}

static int max77658_set_alert_thresholds(struct max77658_fg_dev *max77658_fg)
{
	unsigned int val;
	int ret;

	/* Set VAlrtTh */
	val = (max77658_fg->volt_min / 20);
	val |= ((max77658_fg->volt_max / 20) << 8);
	ret = regmap_write(max77658_fg->regmap, REG_VALRTTH, val);
	if (ret)
		return dev_err_probe(max77658_fg->dev, ret,
				     "Error in writing register VALRTTH\n");

	/* Set TAlrtTh */
	val = max77658_fg->temp_min & 0xFF;
	val |= ((max77658_fg->temp_max & 0xFF) << 8);
	ret = regmap_write(max77658_fg->regmap, REG_TALRTTH, val);
	if (ret)
		return dev_err_probe(max77658_fg->dev, ret,
				     "Error in writing register TALRTTH\n");

	/* Set SAlrtTh */
	val = max77658_fg->soc_min;
	val |= (max77658_fg->soc_max << 8);
	ret = regmap_write(max77658_fg->regmap, REG_SALRTTH, val);
	if (ret)
		return dev_err_probe(max77658_fg->dev, ret,
				     "Error in writing register SALRTTH\n");

	/* Set IAlrtTh */
	val = (max77658_fg->curr_min / MAX77658_IALRTTH_RESOLUTION) & 0xFF;
	val |= (((max77658_fg->curr_max / MAX77658_IALRTTH_RESOLUTION)  & 0xFF) << 8);
	return regmap_write(max77658_fg->regmap, REG_IALRTTH, val);
}

static int max77658_fg_initialize(struct max77658_fg_dev *max77658_fg)
{
	int ret;
	unsigned int reg;

	/* Optional step - alert threshold initialization */
	ret = max77658_set_alert_thresholds(max77658_fg);
	if (ret)
		return dev_err_probe(max77658_fg->dev, ret,
				     "Error in setting alert thresholds\n");

	/* Clear Status.POR */
	ret = regmap_read(max77658_fg->regmap, REG_STATUS, &reg);
	if (ret)
		return dev_err_probe(max77658_fg->dev, ret,
				     "Error in reading register STATUS\n");

	ret = regmap_write(max77658_fg->regmap, REG_STATUS, reg & ~BIT_STATUS_POR);
	if (ret)
		return dev_err_probe(max77658_fg->dev, ret,
				     "Error in writing register STATUS\n");

	schedule_delayed_work(&max77658_fg->work, MAX77658_FG_DELAY);
	return 0;
}

static int max77658_fg_parse_dt(struct max77658_fg_dev *max77658_fg)
{
	struct device *dev = max77658_fg->dev;
	struct device_node *np = of_find_node_by_name(NULL, "battery");
	int ret;

	/* FG */
	if (!np)
		return dev_err_probe(dev, -ENODEV, "Failed to find device_node\n");

	ret = of_property_read_u32(np, "talrt-min", &max77658_fg->temp_min);
	if (ret)
		max77658_fg->temp_min = -128; /* DegreeC */

	ret = of_property_read_u32(np, "talrt-max", &max77658_fg->temp_max);
	if (ret)
		max77658_fg->temp_max = 127; /* DegreeC */

	ret = of_property_read_u32(np, "valrt-min", &max77658_fg->volt_min);
	if (ret)
		max77658_fg->volt_min = 0; /* mV */

	ret = of_property_read_u32(np, "valrt-max", &max77658_fg->volt_max);
	if (ret)
		max77658_fg->volt_max = 5100; /* mV */

	ret = of_property_read_u32(np, "salrt-min", &max77658_fg->soc_min);
	if (ret)
		max77658_fg->soc_min = 0; /* Percent */

	ret = of_property_read_u32(np, "salrt-max", &max77658_fg->soc_max);
	if (ret)
		max77658_fg->soc_max = 255; /* Percent */

	ret = of_property_read_u32(np, "ialrt-min", &max77658_fg->curr_min);
	if (ret)
		max77658_fg->curr_min = MAX77658_IALRTTH_RESOLUTION * (-128); /* uA */

	ret = of_property_read_u32(np, "ialrt-max", &max77658_fg->curr_max);
	if (ret)
		max77658_fg->curr_max = MAX77658_IALRTTH_RESOLUTION * 127; /* uA */

	return 0;
}

static int max77658_fg_probe(struct platform_device *pdev)
{
	struct max77658_dev *max77658 = dev_get_drvdata(pdev->dev.parent);
	struct max77658_fg_dev *fg;
	unsigned int version;
	int ret = 0;
	struct power_supply_config fg_cfg = {};

	fg = devm_kzalloc(&pdev->dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return dev_err_probe(&pdev->dev, -ENOMEM,
				     "Failed to allocate memory for fuel gauge\n");

	mutex_init(&fg->lock);

	fg->dev = &pdev->dev;
	fg->regmap = max77658->regmap_fuel;

	ret = max77658_fg_parse_dt(fg);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Error in parsing DT\n");

	ret = regmap_read(fg->regmap, REG_DEVNAME, &version);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Error in getting version\n");

	dev_dbg(&pdev->dev, "MAX77658 Fuelgauge Ver : 0x%x\n", version);

	fg->psy_batt_d.name = MAX77658_FUELGAUGE_NAME;
	fg->psy_batt_d.type = POWER_SUPPLY_TYPE_BATTERY;
	fg->psy_batt_d.get_property = max77658_fg_get_property;
	fg->psy_batt_d.set_property = max77658_fg_set_property;
	fg->psy_batt_d.properties = max77658_fg_battery_props;
	fg->psy_batt_d.property_is_writeable = max77658_property_is_writeable;
	fg->psy_batt_d.num_properties = ARRAY_SIZE(max77658_fg_battery_props);
	fg_cfg.drv_data = fg;
	fg_cfg.supplied_to = batt_supplied_to;
	fg_cfg.of_node = pdev->dev.of_node;
	fg_cfg.num_supplicants = ARRAY_SIZE(batt_supplied_to);

	fg->battery = devm_power_supply_register(&pdev->dev, &fg->psy_batt_d, &fg_cfg);
	if (IS_ERR(fg->battery))
		return dev_err_probe(&pdev->dev, PTR_ERR(fg->battery),
				     "Failed to register battery\n");

	INIT_DELAYED_WORK(&fg->work, max77658_fg_work);

	return max77658_fg_initialize(fg);
}

static int max77658_fg_remove(struct platform_device *pdev)
{
	struct max77658_fg_dev *fg = platform_get_drvdata(pdev);

	cancel_delayed_work_sync(&fg->work);

	return 0;
}

static const struct platform_device_id max77658_fg_id[] = {
	{ MAX77658_FUELGAUGE_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, max77658_fg_id);

static struct platform_driver max77658_fg_driver = {
	.driver = {
		.name = MAX77658_FUELGAUGE_NAME,
	},
	.probe = max77658_fg_probe,
	.remove = max77658_fg_remove,
	.id_table = max77658_fg_id,
};

module_platform_driver(max77658_fg_driver);

MODULE_DESCRIPTION("MAX77658 Fuel Gauge Driver");
MODULE_AUTHOR("Nurettin.Bolucu@analog.com ");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
