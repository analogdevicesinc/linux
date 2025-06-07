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

#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/version.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regmap.h>
#include <linux/mfd/max77840.h>
#include <linux/regulator/max77840-regulator.h>
#include <linux/module.h>
#include <linux/regulator/of_regulator.h>
#include <linux/of.h>

#ifdef __TEST_DEVICE_NODE__
#include <linux/string.h>
#include <linux/sysfs.h>
#endif

#define M2SH	__CONST_FFS

/* Register */
/* Safeout */
#define REG_SAFEOUTCTRL		0xC6
#define BIT_SAFEOUT1		BITS(1, 0)
#define BIT_ACTDISSAFEO1	BIT(4)
#define BIT_ENSAFEOUT1		BIT(6)

struct max77840_data {
	struct device *dev;
	struct max77840_dev *iodev;
	struct regulator_dev *rdev;
};

static unsigned int max77840_safeout_volt_table[] = {
	4850000, 4900000, 4950000, 3300000,
};

static const struct regulator_ops max77840_safeout_ops = {
	.list_voltage		= regulator_list_voltage_table,
	.map_voltage		= regulator_map_voltage_ascend,
	.is_enabled		= regulator_is_enabled_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
};

static struct regulator_desc max77840_safeout_desc = {
	.name           = "SAFEOUT1",
	.id             = MAX77840_SAFEOUT1,
	.ops            = &max77840_safeout_ops,
	.type           = REGULATOR_VOLTAGE,
	.owner          = THIS_MODULE,
	.n_voltages     = ARRAY_SIZE(max77840_safeout_volt_table),
	.volt_table     = max77840_safeout_volt_table,
	.vsel_reg       = REG_SAFEOUTCTRL,
	.vsel_mask      = BIT_SAFEOUT1,
	.enable_reg     = REG_SAFEOUTCTRL,
	.enable_mask    = BIT_ENSAFEOUT1,
};

#ifdef CONFIG_OF
	static struct max77840_regulator_platform_data
*max77840_regulator_parse_dt(struct device *dev)
{
	struct device_node *np = of_find_node_by_name(NULL, "regulator");
	struct max77840_regulator_platform_data *pdata;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (unlikely(!pdata))
		return ERR_PTR(-ENOMEM);

	if (!np) {
		pr_err("%s np NULL\n", __func__);
		pdata = ERR_PTR(-EINVAL);
		goto out;
	}

	if (!of_node_cmp(np->name, max77840_safeout_desc.name))
		pdata->initdata = of_get_regulator_init_data(dev, np, &max77840_safeout_desc);

	pdata->of_node = np;
	of_node_put(np);

out:
	return pdata;
}
#endif

#ifdef __TEST_DEVICE_NODE__
static struct max77840_data *test_if;
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
	struct regmap *if_regmap = test_if->iodev->regmap_pmic;

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
		.name = "max77840_test_if",
		.mode = 0x0666,
		},
	.show = NULL,
	.store = max77840_test_store,
};

static int max77840_test_node(struct max77840_data *max77840)
{
	int ret;

	ret = sysfs_create_file(&max77840->dev->kobj, &max77840_attribute.attr);
	test_if = max77840;
	return ret;
}
#else
static int max77840_test_node(struct max77840_data *pmic)
{
	return 0;
}
#endif

static int max77840_regulator_probe(struct platform_device *pdev)
{
	struct max77840_dev *iodev = dev_get_drvdata(pdev->dev.parent);
	struct max77840_regulator_platform_data *pdata = dev_get_platdata(&pdev->dev);
	struct regulator_dev *rdev;
	struct max77840_data *max77840;
	struct regmap *regmap;
	struct regulator_config config;
	int ret, size;

	pr_info("%s: Max77840 Regulator Driver Loading\n", __func__);

#ifdef CONFIG_OF
	pdata = max77840_regulator_parse_dt(&pdev->dev);
#endif
	if (IS_ERR(pdata)) {
		pr_info("[%s:%d] !pdata\n", __FILE__, __LINE__);
		dev_err(pdev->dev.parent, "No platform init data supplied.\n");
		return PTR_ERR(pdata);
	}

	max77840 = kzalloc(sizeof(*max77840), GFP_KERNEL);
	if (!max77840)
		return -ENOMEM;

	size = sizeof(struct regulator_dev  *);
	max77840->rdev = kzalloc(size, GFP_KERNEL);
	if (!max77840->rdev) {
		pr_info("[%s:%d] if (!max77840->rdev)\n", __FILE__, __LINE__);
		kfree(max77840);
		return -ENOMEM;
	}

	rdev = max77840->rdev;
	max77840->dev = &pdev->dev;
	max77840->iodev = iodev;
	platform_set_drvdata(pdev, max77840);
	regmap = max77840->iodev->regmap_pmic;

	pr_info("[%s:%d] for regulator\n",
		__FILE__,
		__LINE__);

	config.dev = &pdev->dev;
	config.driver_data = max77840;
	config.init_data = pdata->initdata;
	config.of_node = pdata->of_node;
	config.regmap = regmap;
	rdev = devm_regulator_register(&pdev->dev, &max77840_safeout_desc, &config);

	if (IS_ERR(rdev)) {
		ret = PTR_ERR(rdev);
		dev_err(max77840->dev, "regulator init failed!\n");
		rdev = NULL;
		goto err;
	}

	max77840_test_node(max77840);
	return 0;
err:
	pr_info("[%s:%d] err:\n", __FILE__, __LINE__);
	pr_info("[%s:%d] err_alloc\n", __FILE__, __LINE__);
	kfree(max77840->rdev);
	kfree(max77840);

	return ret;
}

static int max77840_regulator_remove(struct platform_device *pdev)
{
	struct max77840_data *max77840 = platform_get_drvdata(pdev);
	struct regulator_dev *rdev = max77840->rdev;

	regulator_unregister(rdev);
	kfree(max77840->rdev);
	kfree(max77840);

	return 0;
}

static const struct platform_device_id max77840_regulator_id[] = {
	{"max77840-regulator", 0},
	{},
};

MODULE_DEVICE_TABLE(platform, max77840_regulator_id);

static struct platform_driver max77840_regulator_driver = {
	.driver = {
		.name = MAX77840_REGULATOR_NAME,
	},
	.probe = max77840_regulator_probe,
	.remove = max77840_regulator_remove,
	.id_table = max77840_regulator_id,
};

static int __init max77840_regulator_init(void)
{
	return platform_driver_register(&max77840_regulator_driver);
}

static void __exit max77840_regulator_exit(void)
{
	platform_driver_unregister(&max77840_regulator_driver);
}

subsys_initcall(max77840_regulator_init);
module_exit(max77840_regulator_exit);

MODULE_DESCRIPTION("MAXIM 77840 Regulator Driver");
MODULE_AUTHOR("joan.na@analog.com");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
