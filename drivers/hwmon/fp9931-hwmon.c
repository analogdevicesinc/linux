/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Fitipower FP9931 PMIC temperature monitor driver
 *
 * Copyright 2021 NXP
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/mfd/fp9931.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct fp9931_hwmon {
	struct device *dev;
	struct device *hwmon_dev;
	struct fp9931 *fp9931;
	struct regulator *en_ts;
};

static ssize_t fp9931_hwmon_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf);

static SENSOR_DEVICE_ATTR_RO(temp1_input, fp9931_hwmon_temp, 0);

static struct attribute *fp9931_hwmon_attrs[] = {
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(fp9931_hwmon);

static ssize_t fp9931_hwmon_temp_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;
	u8 tmst_value;
	struct fp9931_hwmon *hwmon = dev_get_drvdata(dev);

	ret = fp9931_reg_read(hwmon->fp9931->client,
			      FP9931_TMST_VALUE,
			      &tmst_value);
	if (ret)
		return ret;

	return sprintf(buf, "%d\n", (s32)((s8)tmst_value));
}

static int fp9931_hwmon_probe(struct platform_device *pdev)
{
	int ret;
	struct fp9931_hwmon *hwmon;

	hwmon = devm_kzalloc(&pdev->dev, sizeof(*hwmon), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;
	hwmon->dev = &pdev->dev;

	hwmon->en_ts = devm_regulator_get(pdev->dev.parent, "en-ts");
	if (IS_ERR(hwmon->en_ts)) {
		ret = PTR_ERR(hwmon->en_ts);
		dev_err(&pdev->dev, "failed to get en-ts regulator : %d\n", ret);

		return ret;
	}

	hwmon->hwmon_dev = devm_hwmon_device_register_with_groups(hwmon->dev,
							"fp9931_hwmon", hwmon,
							fp9931_hwmon_groups);
	if (IS_ERR(hwmon->hwmon_dev)) {
		ret = PTR_ERR(hwmon->hwmon_dev);
		dev_err(hwmon->dev, "failed to register hwmon for fp9931 : %d\n", ret);

		return ret;
	}

	hwmon->fp9931 = dev_get_drvdata(pdev->dev.parent);
	WARN_ON(IS_ERR_OR_NULL(hwmon->fp9931));
	platform_set_drvdata(pdev, hwmon);

	return regulator_enable(hwmon->en_ts);
}

static int fp9931_hwmon_remove(struct platform_device *pdev)
{
	struct fp9931_hwmon *hwmon = platform_get_drvdata(pdev);

	return regulator_disable(hwmon->en_ts);
}

static const struct platform_device_id fp9931_hwmon_id[] = {
	{ "fp9931-hwmon", 0 },
	{ /* sentinel */    },
};

static struct platform_driver fp9931_hwmon_driver = {
	.probe	= fp9931_hwmon_probe,
	.remove	= fp9931_hwmon_remove,
	.id_table = fp9931_hwmon_id,
	.driver	= {
		.name = "fp9931_hwmon",
	},
};

module_platform_driver(fp9931_hwmon_driver);

MODULE_DESCRIPTION("FP9931 hardware monitor driver");
MODULE_AUTHOR("Fancy Fang <chen.fang@nxp.com>");
MODULE_LICENSE("GPL");
