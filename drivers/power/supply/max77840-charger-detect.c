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
#include <linux/bitops.h>

/* for Regmap */
#include <linux/regmap.h>

/* for Device Tree */
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mfd/max77840.h>
#include <linux/power/max77840-charger-detect.h>

#ifdef __TEST_DEVICE_NODE__
#include <linux/string.h>
#include <linux/sysfs.h>
#endif

#define M2SH	__ffs

#define IRQ_WORK_DELAY              0

static char *chgdet_supplied_to[] = {
	"max77840-charger-detect",
};

#define __lock(_me)    mutex_lock(&(_me)->lock)
#define __unlock(_me)  mutex_unlock(&(_me)->lock)

static const struct max77840_chg_type_props {
	enum power_supply_type type;
} chg_type_props[] = {
	{ POWER_SUPPLY_TYPE_UNKNOWN },
	{ POWER_SUPPLY_TYPE_USB },
	{ POWER_SUPPLY_TYPE_USB_CDP },
	{ POWER_SUPPLY_TYPE_USB_DCP },
	{ POWER_SUPPLY_TYPE_USB_DCP },
	{ POWER_SUPPLY_TYPE_USB_DCP },
	{ POWER_SUPPLY_TYPE_USB_DCP },
	{ POWER_SUPPLY_TYPE_USB },
};

struct max77840_chgdet_data {
	struct device                          *dev;
	struct max77840_dev                    *max77840;
	struct regmap                          *regmap;
	struct power_supply                    *psy_chgdet;
	struct power_supply_desc               psy_chgdet_d;

	int                                    chgtyp_irq;

	int                                    status;

	int                                    irq;
	int                                    irq_mask;
	struct delayed_work                    irq_work;

	/* mutex */
	struct mutex                           lock;

	struct max77840_chgdet_platform_data  *pdata;
};

static enum power_supply_property max77840_chgdet_props[] = {
	POWER_SUPPLY_PROP_CURRENT_MAX
};

static void max77840_chgdet_initialize(struct max77840_chgdet_data *chgdet);

static inline int max77840_ffs(unsigned int x)
{
	return x ? __ffs(x) : 0;
}

/* charger detect API function */
static int max77840_chgdet_get_input_current(struct max77840_chgdet_data *chgdet)
{
	u8 reg_data = 0;
	int steps[3] = { 0, 33, 67 };	/* mA */
	int get_current, quotient, remainder;

	pr_info("max77840 charger detect get charger current");

	max77840_read(chgdet->regmap, REG_CHGDET_CHGIN_ILIM, &reg_data);

	quotient = reg_data / 3;
	remainder = reg_data % 3;

	if ((reg_data & BIT_CHGIN_ILIM) < 3)
		get_current = 100;	/* 100mA */
	else if ((reg_data & BIT_CHGIN_ILIM) > 0x78)
		get_current = 4000;	/* 4000mA */
	else
		get_current = quotient * 100 + steps[remainder];

	return get_current;
}

/* interrupt handler and workqueu*/
static void max77840_do_irq(struct max77840_chgdet_data *chgdet, int irq)
{
	u8 val;

	switch (irq) {
	case CHGDET_INT_CHGTYPE_I:
		val = (chgdet->status) & BIT_CHGTYPE;
		if (val < CHGDET_CHGTYP_CHARGER_LAST)
			chgdet->psy_chgdet_d.type = chg_type_props[val].type;
		else
			chgdet->psy_chgdet_d.type = POWER_SUPPLY_TYPE_UNKNOWN;
		pr_debug("CHGDET_INT_CHGTYP: ChgTyp = %02Xh\n", val);
		break;

	case CHGDET_INT_CHGDETRUN_I:
		val = ((chgdet->status & BIT_CHGDETRUN) >> max77840_ffs(BIT_CHGDETRUN));
		pr_debug("CHGDET_INT_CHGDETRUN: ChgDetRun = %02Xh\n", val);
		break;

	case CHGDET_INT_DCDTMR_I:
		val = ((chgdet->status & BIT_DCDTMR) >> max77840_ffs(BIT_DCDTMR));
		pr_debug("CHGDET_INT_DCDTMR: DCDTmr = %02Xh\n", val);
		break;

	case CHGDET_INT_DXOVP_I:
		val = ((chgdet->status & BIT_DXOVP) >> max77840_ffs(BIT_DXOVP));
		pr_debug("CHGDET_INT_DXOVP: DxOVP = %02Xh\n", val);
		break;

	case CHGDET_INT_VDNMON_I:
		val = ((chgdet->status & BIT_VDNMON) >> max77840_ffs(BIT_VDNMON));
		pr_debug("CHGDET_INT_VDNMON: VDNMon = %02Xh\n", val);
		break;

	default:
		break;
	}
	/* notify psy changed */
	power_supply_changed(chgdet->psy_chgdet);
}

static irqreturn_t max77840_chgdet_chgin_isr(int irq, void *data)
{
	struct max77840_chgdet_data *me = data;
	u8 reg_data;

	me->irq = irq;

	max77840_read(me->regmap, REG_CHGDET_STATUS, &reg_data);
	me->status = reg_data;

	schedule_delayed_work(&me->irq_work, IRQ_WORK_DELAY);
	return IRQ_HANDLED;
}

static void max77840_chgdet_irq_work(struct work_struct *work)
{
	struct max77840_chgdet_data *me =
		container_of(work, struct max77840_chgdet_data, irq_work.work);
	u8 irq;

	__lock(me);

	irq = me->irq - me->chgtyp_irq;

	max77840_do_irq(me, irq);

	__unlock(me);
}

static void max77840_chgdet_initialize(struct max77840_chgdet_data *chgdet)
{
	int rc;
	u8 val;

	/* interrupt mask - if you want to enable some bits, you should clear them */
	val  = 0;
	val |= BIT_CHGTYPE_I;
	val |= BIT_CHGDETRUN_I;
	val |= BIT_DCDTMR_I;
	val |= BIT_DXOVP_I;
	val |= BIT_VDNMON_I;

	rc = max77840_write(chgdet->regmap, REG_CHGDET_INT_MASK, val);
	if (rc < 0) {
		pr_err("CHGDET_INT_MASK write error [%d]\n", rc);
		goto out;
	}

out:
	return;
}

static int max77840_chgdet_get_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct max77840_chgdet_data *chgdet =
		power_supply_get_drvdata(psy);
	int rc = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		val->intval = max77840_chgdet_get_input_current(chgdet);
		break;

	default:
		rc = -EINVAL;
		goto out;
	}

out:
	pr_info("%s: <get_property> psp %d val %d [%d]\n",
		__func__, psp, val->intval, rc);
	return rc;
}

#ifdef CONFIG_OF
static int max77840_chgdet_parse_dt(struct max77840_chgdet_data *chgdet)
{
	struct device *dev = chgdet->dev;
	struct device_node *np = of_find_node_by_name(NULL, "charger-detect");
	struct max77840_chgdet_platform_data *pdata;

	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata))
		return -ENOMEM;

	pdata->input_current_limit = 500; /* 500mA */
	ret |= of_property_read_u32(np, "input_current_limit", pdata->input_current_limit);
	pr_debug("property:INPUT_CURRENT_LIMIT %umA\n", pdata->input_current_limit);

	if (ret < 0)
		return ret;
	chgdet->pdata = pdata;
	return 0;
}
#endif

#ifdef __TEST_DEVICE_NODE__
static struct max77840_chgdet_data *test_if;
static ssize_t max77840_test_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t size)
{
	int ret, reg;
	unsigned long strval;
	u8 rval;
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
		ret = max77840_read(if_regmap, reg, &rval);
		if (ret < 0)
			dev_err(test_if->dev, "failed to read i2c: %d @ function %s\n",
				ret, __func__);
		else
			dev_err(test_if->dev, "read [0x%x] = 0x%x\n", reg, rval);
		return size;
	}
	// register write
	else if (strncmp("write", args[0], 5) == 0) {
		ret = kstrtoul(args[1], 0, &strval);
		reg = (int)strval;
		args[2] = strsep(&buff, sep);
		ret = kstrtoul(args[2], 0, &strval);
		rval = (int)strval;
		ret = max77840_write(if_regmap, reg, rval);
		if (ret < 0)
			dev_err(test_if->dev, "failed to write i2c: %d @ function %s\n",
				ret, __func__);
		else
			dev_err(test_if->dev, "write [0x%x] = 0x%x\n", reg, rval);
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

static int max77840_test_node(struct max77840_chgdet_data *chgdet)
{
	int ret;

	ret = sysfs_create_file(&chgdet->psy_chgdet->dev.kobj, &max77840_attribute.attr);
	test_if = chgdet;
	return ret;
}
#else
static int max77840_test_node(struct max77840_chgdet_data *chgdet)
{
	return 0;
}
#endif

static int max77840_chgdet_probe(struct platform_device *pdev)
{
	struct max77840_dev *max77840 = dev_get_drvdata(pdev->dev.parent);
	struct max77840_chgdet_platform_data *pdata;
	struct max77840_chgdet_data *chgdet;
	int ret = 0;
	u8 val = 0;
	struct power_supply_config chgdet_cfg;

	pr_info("%s: Max77840 Charger Detect Driver Loading\n", __func__);

	chgdet = kzalloc(sizeof(*chgdet), GFP_KERNEL);
	if (!chgdet)
		return -ENOMEM;

	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata)) {
		pr_err("%s: out of memory\n", __func__);
		pdata = ERR_PTR(-ENOMEM);
		return -ENOMEM;
	}

	mutex_init(&chgdet->lock);

	chgdet->dev = &pdev->dev;
	chgdet->regmap = max77840->regmap_chg_det;
	chgdet->pdata = pdata;

#if defined(CONFIG_OF)
	ret = max77840_chgdet_parse_dt(chgdet);
	if (ret < 0)
		pr_err("%s not found charger dt! ret[%d]\n", __func__, ret);
#else
	pdata = dev_get_platdata(&pdev->dev);
#endif

	platform_set_drvdata(pdev, chgdet);
	chgdet->psy_chgdet_d.name		= "max77840-charger-detect";
	chgdet->psy_chgdet_d.type		= POWER_SUPPLY_TYPE_UNKNOWN;
	chgdet->psy_chgdet_d.get_property	= max77840_chgdet_get_property;
	chgdet->psy_chgdet_d.properties         = max77840_chgdet_props;
	chgdet->psy_chgdet_d.num_properties	= ARRAY_SIZE(max77840_chgdet_props);
	chgdet_cfg.drv_data = chgdet;
	chgdet_cfg.supplied_to = chgdet_supplied_to;
	chgdet_cfg.of_node = max77840->dev->of_node;
	chgdet_cfg.num_supplicants = ARRAY_SIZE(chgdet_supplied_to);

	max77840_chgdet_initialize(chgdet);

	chgdet->psy_chgdet = devm_power_supply_register(max77840->dev,
							&chgdet->psy_chgdet_d,
							&chgdet_cfg);
	if (IS_ERR(chgdet->psy_chgdet)) {
		pr_err("Couldn't register charger detect rc=%ld\n", PTR_ERR(chgdet->psy_chgdet));
		goto err_power_supply_register;
	}

	INIT_DELAYED_WORK(&chgdet->irq_work, max77840_chgdet_irq_work);

	chgdet->chgtyp_irq = regmap_irq_get_virq(max77840->irqc_chgdet, CHGDET_IRQ_CHGTYPE_I);

	ret = request_threaded_irq(chgdet->chgtyp_irq, NULL,
				   max77840_chgdet_chgin_isr,
				   IRQF_TRIGGER_FALLING,
				   "charger-detect-chgtyp",
				   chgdet);
	if (ret) {
		pr_err("%s: Failed to Request CHGTYP IRQ\n", __func__);
		goto err_chgtyp_irq;
	}

	max77840_read(chgdet->regmap, REG_CHGDET_INT, &val);
	max77840_read(chgdet->regmap, REG_CHGDET_INT_MASK, &val);

	pr_info("%s: Max77840 Charger Detect Driver Loaded\n", __func__);
	max77840_test_node(chgdet);

	return 0;

err_chgtyp_irq:
	power_supply_unregister(chgdet->psy_chgdet);
err_power_supply_register:
	kfree(chgdet);

	return ret;
}

static int max77840_chgdet_remove(struct platform_device *pdev)
{
	struct max77840_chgdet_data *chgdet =
		platform_get_drvdata(pdev);

	free_irq(chgdet->chgtyp_irq, NULL);
	power_supply_unregister(chgdet->psy_chgdet);

	kfree(chgdet);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int max77840_chgdet_suspend(struct device *dev)
{
	return 0;
}

static int max77840_chgdet_resume(struct device *dev)
{
	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(max77840_chgdet_pm_ops, max77840_chgdet_suspend,
		max77840_chgdet_resume);

#ifdef CONFIG_OF
static const struct of_device_id max77840_chgdet_dt_ids[] = {
	{ .compatible = "maxim,max77840-charger-detect" },
	{ }
};
MODULE_DEVICE_TABLE(of, max77840_chgdet_dt_ids);
#endif

static struct platform_driver max77840_chgdet_driver = {
	.driver = {
		.name = MAX77840_CHARGER_DETECT_NAME,
		.owner = THIS_MODULE,
		.pm = &max77840_chgdet_pm_ops,
#ifdef CONFIG_OF
		.of_match_table = max77840_chgdet_dt_ids,
#endif
	},
	.probe = max77840_chgdet_probe,
	.remove = max77840_chgdet_remove,
};

static int __init max77840_chgdet_init(void)
{
	return platform_driver_register(&max77840_chgdet_driver);
}

static void __exit max77840_chgdet_exit(void)
{
	platform_driver_unregister(&max77840_chgdet_driver);
}

module_init(max77840_chgdet_init);
module_exit(max77840_chgdet_exit);

MODULE_DESCRIPTION("MAX77840 Charger Detect Driver");
MODULE_AUTHOR("joan.na@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
