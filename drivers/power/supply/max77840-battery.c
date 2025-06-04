// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 Maxim Integrated Products, Inc.
 * Copyright (C) 2024 Analog Devices, Inc.
 *
 * Author : Analog Devices <joan.na@analog.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/version.h>
#include <asm/unaligned.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77840.h>
#include <linux/power/max77840-battery.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#ifdef __TEST_DEVICE_NODE__
#include <linux/string.h>
#include <linux/sysfs.h>
#endif

#define MAX77840_FG_DELAY		1000
#define MAX77840_BATTERY_FULL	100
#define MAX77840_BATTERY_LOW	15

#define MAX77840_VERSION_NO	0x20B0

static char *batt_supplied_to[] = {
	"max77840-fuelgauge",
};

struct max77840_chip {
	struct device           *dev;
	struct max77840_dev     *max77840;
	struct regmap           *regmap;

	int                     fg_irq;
	struct delayed_work     work;
	struct power_supply     *battery;
	struct power_supply_desc psy_batt_d;

	struct mutex lock; /* mutext */

	/* alert */
	int alert_threshold;

	/* State Of Connect */
	int ac_online;
	int usb_online;

	/* battery voltage */
	int vcell;

	/* battery capacity */
	int soc;

	/* State Of Charge */
	int status;

	/* battery health */
	int health;

	/* battery capacity */
	int capacity_level;

	int lasttime_vcell;
	int lasttime_soc;
	int lasttime_status;

	struct max77840_fg_platform_data  *pdata;
};

static int max77840_fg_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct max77840_chip *chip = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = chip->status;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		val->intval = chip->vcell;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = chip->soc;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = chip->health;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = chip->capacity_level;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void max77840_fg_get_vcell(struct max77840_chip *max77840_fg)
{
	u16 vcell;
	int rc;

	rc = max77840_fg_read(max77840_fg->regmap, REG_VCELL, &vcell);
	if (rc < 0) {
		dev_err(max77840_fg->dev, "%s: err %d\n", __func__, rc);
	} else {
		pr_info("%s: vcell raw value(0x%4x)\n", __func__, vcell);
		max77840_fg->vcell = (vcell >> 3) * 625;
	}
}

static void max77840_fg_get_soc(struct max77840_chip *max77840_fg)
{
	u16 soc;
	int rc;

	rc = max77840_fg_read(max77840_fg->regmap, REG_VFSOC, &soc);
	pr_info("%s fg read REG_VFSOC %d, rc=%d", __func__, soc, rc);

	if (rc < 0)
		dev_err(max77840_fg->dev, "%s: err %d\n", __func__, rc);
	else
		max77840_fg->soc = (u16)soc >> 8;

	if (max77840_fg->soc > MAX77840_BATTERY_FULL) {
		max77840_fg->soc = MAX77840_BATTERY_FULL;
		max77840_fg->status = POWER_SUPPLY_STATUS_FULL;
		max77840_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_FULL;
		max77840_fg->health = POWER_SUPPLY_HEALTH_GOOD;
	} else if (max77840_fg->soc < MAX77840_BATTERY_LOW) {
		max77840_fg->health = POWER_SUPPLY_HEALTH_DEAD;
		max77840_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
	} else {
		max77840_fg->health = POWER_SUPPLY_HEALTH_GOOD;
		max77840_fg->capacity_level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	}
}

static u16 max77840_fg_get_version(struct max77840_chip *max77840_fg)
{
	u16 version;
	int rc;

	rc = max77840_fg_read(max77840_fg->regmap, REG_VERSION, &version);
	if (rc < 0)
		dev_err(max77840_fg->dev, "%s: err %d\n", __func__, rc);

	return version;
}

static bool max77840_fg_check_status(struct max77840_chip *max77840_fg)
{
	u16 data;
	bool ret = false;
	int fg_write_rt;

	/* check if Smn was generated */
	if (max77840_fg_read(max77840_fg->regmap, REG_STATUS, &data) < 0)
		return ret;

	pr_info("%s: status_reg(0x%4x)\n", __func__, data);

	/* minimum SOC threshold exceeded. */
	if (data & BIT_SMN)
		ret = true;

	/* check 1% SOC change happened */
	if (data & BIT_DSOCI) {
		max77840_fg_get_vcell(max77840_fg);
		max77840_fg_get_soc(max77840_fg);
		power_supply_changed(max77840_fg->battery);

		pr_info("soc changed, SOC=%d, VCELL=%d\n", max77840_fg->soc, max77840_fg->vcell);
	}

	/* clear status reg */
	if (!ret) {
		data = data & 0x007F;
		fg_write_rt = max77840_fg_write(max77840_fg->regmap, REG_STATUS, data);
		if (fg_write_rt < 0)
			return ret;
	}

	return ret;
}

static void max77840_fg_work(struct work_struct *work)
{
	struct max77840_chip *chip;

	chip = container_of(work, struct max77840_chip, work.work);

	max77840_fg_get_vcell(chip);
	max77840_fg_get_soc(chip);

	if (chip->vcell != chip->lasttime_vcell ||
	    chip->soc != chip->lasttime_soc ||
	    chip->status != chip->lasttime_status) {
		chip->lasttime_vcell = chip->vcell;
		chip->lasttime_soc = chip->soc;

		power_supply_changed(chip->battery);
	}
	schedule_delayed_work(&chip->work, MAX77840_FG_DELAY);
}

static irqreturn_t max77840_fg_irq_thread(int irq, void *irq_data)
{
	struct max77840_chip *fuelgauge = irq_data;
	bool fuel_alerted;

	if (fuelgauge->pdata->soc_alert_threshold >= 0) {
		fuel_alerted = max77840_fg_check_status(fuelgauge);
		pr_info("%s: Fuel-alert %salerted!\n",
			__func__, fuel_alerted ? "" : "NOT ");

		/* schedule_delayed_work(&fuelgauge->work, 0); */
	}

	return IRQ_HANDLED;
}

static enum power_supply_property max77840_fg_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

static int max77840_fg_initialize(struct max77840_chip *chip)
{
	u16 config, val;
	u8 data[2];
	int ret;

	max77840_fg_get_vcell(chip);
	pr_info("<%s> vcell %d\n", __func__, chip->vcell);

	max77840_fg_get_soc(chip);
	pr_info("<%s> soc %d\n", __func__, chip->soc);

	/* 1. set fuel gauge alert configuration */
	/* SALRT Threshold setting */
	data[0] = chip->pdata->soc_alert_threshold;
	data[1] = 0xff;
	val = (data[1] << 8) | data[0];
	max77840_fg_write(chip->regmap, REG_SALRT_TH, val);

	/* VALRT Threshold setting */
	data[0] = 0x00;
	data[1] = 0xff;
	val = (data[1] << 8) | data[0];
	max77840_fg_write(chip->regmap, REG_VALRT_TH, val);

	/* TALRT Threshold setting */
	data[0] = 0x80;
	data[1] = 0x7f;
	val = (data[1] << 8) | data[0];
	max77840_fg_write(chip->regmap, REG_TALRT_TH, val);

	ret = max77840_fg_read(chip->regmap, REG_CONFIG, &val);
	if (ret < 0)
		return ret;

	/*Enable Alert (Aen = 1) */
	config = val | (0x01 << 2);
	ret = max77840_fg_write(chip->regmap, REG_CONFIG, config);
	if (ret < 0)
		return ret;

	/* 2. set SOC 1% change alert */
	ret = max77840_fg_read(chip->regmap, REG_CONFIG2, &val);
	if (ret < 0)
		return ret;
	config = val | BIT_DSOCEN;
	ret = max77840_fg_write(chip->regmap, REG_CONFIG2, config);
	if (ret < 0)
		return ret;

	return 0;
}

#ifdef CONFIG_OF
static int max77840_fg_parse_dt(struct max77840_chip *fuelgauge)
{
	struct device *dev = fuelgauge->dev;
	struct device_node *np = of_find_node_by_name(NULL, "fuelgauge");
	struct max77840_fg_platform_data *pdata;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata))
		return -ENOMEM;

	/* reset, irq gpio info */
	if (!np) {
		pr_err("%s np NULL\n", __func__);
		return -EINVAL;
	}
	ret |= of_property_read_u32(np, "fuel_alert_soc", &pdata->soc_alert_threshold);
	if (ret < 0)
		pr_err("%s error reading pdata->fuel_alert_soc %d\n", __func__, ret);

	if (ret < 0)
		return ret;
	fuelgauge->pdata = pdata;
	return 0;
}
#endif

#ifdef __TEST_DEVICE_NODE__
static struct max77840_chip *test_if;
static ssize_t max77840_test_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int ret, reg;
	unsigned long strval;
	u16 val;
	char *args[3];
	char *sep = ",\t";
	char *buff = (char  *)buf;
	struct regmap *if_regmap = test_if->regmap;

	args[0] = strsep(&buff, sep);
	if (!args[0])
		return -2;

	// register read
	args[1] = strsep(&buff, sep);
	if (strncmp("read", args[0], 4) == 0) {
		ret = kstrtoul(args[1], 0, &strval);
		reg = (int)strval;
		ret = max77840_fg_read(if_regmap, reg, &val);
		if (ret < 0)
			dev_err(test_if->dev, "failed to read i2c: %d @ function %s\n",
				ret, __func__);
		else
			dev_err(test_if->dev, "read [0x%x] = 0x%x\n", reg, val);
		return size;
	}
	// register write
	else if (strncmp("write", args[0], 5) == 0) {
		ret = kstrtoul(args[1], 0, &strval);
		reg = (int)strval;
		args[2] = strsep(&buff, sep);
		ret = kstrtoul(args[2], 0, &strval);
		val = (int)strval;
		ret = max77840_fg_write(if_regmap, reg, val);
		if (ret < 0)
			dev_err(test_if->dev, "failed to write i2c: %d @ function %s\n",
				ret, __func__);
		else
			dev_err(test_if->dev, "write [0x%x] = 0x%x\n", reg, val);
		return size;
	}

	dev_err(test_if->dev, "Command not supported.\n");

	return 0;
}

static struct device_attribute max77840_attribute = {
		.attr = {
			.name = "max77840_test_node",
			.mode = 0x0666,
			},
		.show = NULL,
		.store = max77840_test_store,
};

static int max77840_test_node(struct max77840_chip *pchip)
{
	int ret;

	ret = sysfs_create_file(&pchip->battery->dev.kobj, &max77840_attribute.attr);
	test_if = pchip;
	return ret;
}
#else
static int max77840_test_node(struct max77840_chip *pchip)
{
	return 0;
}
#endif

static int max77840_fg_probe(struct platform_device *pdev)
{
	struct max77840_dev *max77840 = dev_get_drvdata(pdev->dev.parent);
	struct max77840_fg_platform_data *pdata =
		dev_get_platdata(max77840->dev);
	struct max77840_chip *chip;
	u16 version;
	int ret = 0;
	u8 val;
	struct power_supply_config fg_cfg;

	pr_info("%s: MAX77840 Fuelgauge Driver Loading\n", __func__);

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		pdata = ERR_PTR(-ENOMEM);
		return -ENOMEM;
	}

	mutex_init(&chip->lock);

	chip->dev = &pdev->dev;
	chip->pdata = pdata;
	chip->regmap = max77840->regmap_fuel;

#if defined(CONFIG_OF)
	ret = max77840_fg_parse_dt(chip);
	if (ret < 0)
		pr_err("%s not found fuelgauge dt! ret[%d]\n", __func__, ret);
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	version = max77840_fg_get_version(chip);
	dev_info(&pdev->dev, "MAX77840 Fuelgauge Ver 0x%x\n", version);
	if (version != MAX77840_VERSION_NO) {
		ret = -ENODEV;
		goto error;
	}
	chip->psy_batt_d.name		= "battery";
	chip->psy_batt_d.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->psy_batt_d.get_property	= max77840_fg_get_property;
	chip->psy_batt_d.properties	= max77840_fg_battery_props;
	chip->psy_batt_d.num_properties = ARRAY_SIZE(max77840_fg_battery_props);

	fg_cfg.drv_data        = chip;
	fg_cfg.supplied_to     = batt_supplied_to;
	fg_cfg.of_node	       = max77840->dev->of_node;
	fg_cfg.num_supplicants = ARRAY_SIZE(batt_supplied_to);

	chip->battery =
		devm_power_supply_register(max77840->dev, &chip->psy_batt_d, &fg_cfg);
		if (IS_ERR(chip->battery)) {
			pr_err("Couldn't register battery rc=%ld\n", PTR_ERR(chip->battery));
			goto error;
		}

	chip->fg_irq = regmap_irq_get_virq(max77840->irqc_intsrc, MAX77840_FG_INT);
	dev_info(&pdev->dev, "MAX77840 Fuel-Gauge irq %d\n", chip->fg_irq);

	if (chip->fg_irq > 0) {
		INIT_DELAYED_WORK(&chip->work, max77840_fg_work);
		ret = request_threaded_irq(chip->fg_irq, NULL,
					   max77840_fg_irq_thread,
					   IRQF_TRIGGER_FALLING, "fuelgauge-irq", chip);
		if (ret) {
			pr_err("%s: Failed to Request IRQ\n", __func__);
			goto error1;
		}
	}

	ret = max77840_fg_initialize(chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "Error: Initializing fuel-gauge\n");
		goto error2;
	}

	max77840_read(max77840->regmap_pmic, REG_INTSRCMASK, &val);
	pr_info("<%s> intsrc_mask %Xh\n", pdev->name, val);

	max77840_test_node(chip);
	return 0;

error2:
	if (chip->fg_irq)
		free_irq(chip->fg_irq, chip);
error1:
	power_supply_unregister(chip->battery);
error:
	kfree(chip);
	return ret;
}

static int max77840_fg_remove(struct platform_device *pdev)
{
	struct max77840_chip *chip = platform_get_drvdata(pdev);

	power_supply_unregister(chip->battery);
	cancel_delayed_work(&chip->work);
	kfree(chip);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77840_fg_suspend(struct device *dev)
{
	struct max77840_chip *chip = dev_get_drvdata(dev);

	cancel_delayed_work(&chip->work);
	return 0;
}

static int max77840_fg_resume(struct device *dev)
{
	struct max77840_chip *chip = dev_get_drvdata(dev);

	schedule_delayed_work(&chip->work, MAX77840_FG_DELAY);
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77840_fg_pm_ops, max77840_fg_suspend,
		max77840_fg_resume);

#ifdef CONFIG_OF
static const struct of_device_id max77840_fg_dt_ids[] = {
	{ .compatible = "maxim,max77840-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77840_fg_dt_ids);
#endif /* CONFIG_OF */

static struct platform_driver max77840_fg_driver = {
	.driver = {
		.name = MAX77840_FUELGAUGE_NAME,
		.owner = THIS_MODULE,
		.pm = &max77840_fg_pm_ops,
#ifdef CONFIG_OF
		.of_match_table	= max77840_fg_dt_ids,
#endif /* CONFIG_OF */
	},
	.probe	= max77840_fg_probe,
	.remove	= max77840_fg_remove,
};

static int __init max77840_fg_init(void)
{
	return platform_driver_register(&max77840_fg_driver);
}

static void __exit max77840_fg_exit(void)
{
	platform_driver_unregister(&max77840_fg_driver);
}

module_init(max77840_fg_init);
module_exit(max77840_fg_exit);

MODULE_DESCRIPTION("MAX77840 Fuel Gauge Driver");
MODULE_AUTHOR("joan.na@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
