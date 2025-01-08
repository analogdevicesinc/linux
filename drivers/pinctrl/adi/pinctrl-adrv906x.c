// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2022 ~ 2024, Analog Devices Incorporated, All Rights Reserved
 *
 * The sysfs file I/O for user space application to set/get configs such as:
 *  -config:[all, 6:0]
 *      --drive_strength=config[3:0]
 *      --schmitt_trig_enable=config[4]
 *      --pin_pull_enablement=config[5]
 *      --pin_pull_up_enable=config[6]
 *  -mux_sel
 *
 * Location example('20218000.pinctrl' is the dev_name of this driver):
 * /sys/devices/platform/20218000.pinctrl/control/
 * |-- pin
 * |-- config
 * |-- drive_strength
 * |-- mux_sel
 * |-- pin_pull_enablement
 * |-- pin_pull_up_enable
 * `-- schmitt_trig_enable
 *
 *  Usage examples:
 *  1) Commands to set pin 5'  drive_strength to 7:
 *      echo 5 > ./pin
 *      echo 7 > ./drive_strength
 *  2) Commands to get pin 5'  drive_strength:
 *      echo 5 > ./pin (this can be omitted when following a set command)
 *      cat ./drive_strength
 *      Result will be showed on the console.
 */

#include <dt-bindings/pinctrl/pinctrl-adi-adrv906x-io-pad.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinctrl.h>
#include "pinctrl-adi.h"
#include "pinctrl-adrv906x-init-tbl.h"
#include "../core.h"

struct adrv906x_pinctrl_driver {
	struct device *dev;
	int config;
	int drive_strength;
	int schmitt_trig_enable;
	int pin_pull_enablement;
	int pin_pull_up_enable;
	int mux_sel;
};

typedef enum {
	CONFIG			= 0,
	DRIVE_STRENGTH		= 1,
	SCHMITT_TRIG_ENABLE	= 2,
	PIN_PULL_ENABLEMENT	= 3,
	PIN_PULL_UP_ENABLE	= 4,
	MUX_SEL			= 5
} store_type_t;

struct adrv906x_pinconf_state {
	struct pinctrl *p;
	struct pinctrl_state *s;
};

static int pin;

static int adrv906x_pinctrl_get_config_direct(const char *dev_name, unsigned pin, unsigned long *configs, size_t nconfigs)
{
	const struct pinconf_ops *ops;
	int ret;

	struct pinctrl_dev *pctldev = get_pinctrl_dev_from_devname(dev_name);

	if (!pctldev)
		return -EPROBE_DEFER;

	mutex_lock(&pctldev->mutex);
	ops = pctldev->desc->confops;
	if (!ops || !ops->pin_config_set) {
		mutex_unlock(&pctldev->mutex);
		return -ENOTSUPP;
	}

	ret = ops->pin_config_get(pctldev, pin, configs);
	mutex_unlock(&pctldev->mutex);

	return ret;
}

static int adrv906x_pinctrl_set_config_direct(const char *dev_name, unsigned pin, unsigned long *configs, size_t nconfigs)
{
	const struct pinconf_ops *ops;
	int ret;

	struct pinctrl_dev *pctldev = get_pinctrl_dev_from_devname(dev_name);

	if (!pctldev)
		return -EPROBE_DEFER;

	mutex_lock(&pctldev->mutex);
	ops = pctldev->desc->confops;
	if (!ops || !ops->pin_config_set) {
		mutex_unlock(&pctldev->mutex);
		return -ENOTSUPP;
	}

	ret = ops->pin_config_set(pctldev, pin, configs, nconfigs);
	mutex_unlock(&pctldev->mutex);

	return ret;
}

static ssize_t adrv906x_pinctrl_common_store(struct device *dev, const char *buf, size_t count, store_type_t st)
{
	int result = 0, scan_count = 0, config_val;
	struct adi_pin_mio conf = { 0 };

	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	scan_count = sscanf(buf, "%d", &config_val);
	if (scan_count == 1) {
		conf.input_pin = pin;
		result = adrv906x_pinctrl_get_config_direct(dev_name(dev), pin, &conf.config, 1);
		if (result) {
			pr_err("getting config failed\n");
			return -EIO;
		}

		switch (st) {
		case CONFIG:
			conf.config = (unsigned long)config_val;
			break;
		case DRIVE_STRENGTH:
			conf.config &= (~ADI_CONFIG_DRIVE_STRENGTH_MASK);
			conf.config |= config_val & ADI_CONFIG_DRIVE_STRENGTH_MASK;
			break;
		case SCHMITT_TRIG_ENABLE:
			conf.config &= (~ADI_CONFIG_SCHMITT_TRIG_ENABLE_MASK);
			if (config_val & 0x1)
				conf.config |= ADI_CONFIG_SCHMITT_TRIG_ENABLE_MASK;
			break;
		case PIN_PULL_ENABLEMENT:
			conf.config &= (~ADI_CONFIG_PULL_UP_DOWN_ENABLEMENT_MASK);
			if (config_val & 0x1)
				conf.config |= ADI_CONFIG_PULL_UP_DOWN_ENABLEMENT_MASK;
			break;
		case PIN_PULL_UP_ENABLE:
			conf.config &= (~ADI_CONFIG_PULLUP_ENABLE_MASK);
			if (config_val & 0x1)
				conf.config |= ADI_CONFIG_PULLUP_ENABLE_MASK;
			break;
		case MUX_SEL:
			conf.mux_sel = config_val & ADI_CONFIG_MUX_SEL_MASK;
			break;
		}

		result = adrv906x_pinctrl_set_config_direct(dev_name(dev), pin, (unsigned long *)&conf, 1);
		if (result == 0) {
			switch (st) {
			case CONFIG:
				adrv906x_pinctrl_drv->config = config_val;
				break;
			case DRIVE_STRENGTH:
				adrv906x_pinctrl_drv->drive_strength = config_val & ADI_CONFIG_DRIVE_STRENGTH_MASK;
				break;
			case SCHMITT_TRIG_ENABLE:
				adrv906x_pinctrl_drv->schmitt_trig_enable = config_val & 0x1;
				break;
			case PIN_PULL_ENABLEMENT:
				adrv906x_pinctrl_drv->pin_pull_enablement = config_val & 0x1;
				break;
			case PIN_PULL_UP_ENABLE:
				adrv906x_pinctrl_drv->pin_pull_up_enable = config_val & 0x1;
				break;
			case MUX_SEL:
				adrv906x_pinctrl_drv->mux_sel = config_val & ADI_CONFIG_MUX_SEL_MASK;
				break;
			}
			return count;
		} else {
			pr_err("Set pinconfig failed!\n");
			return -EIO;
		}
	} else {
		pr_err("invalid input\n");
		return -EINVAL;
	}
}

static ssize_t adrv906x_pinctrl_drive_strength_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);
	struct adi_pin_mio conf = { 0 };
	int result = 0;

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	conf.input_pin = pin;
	result = adrv906x_pinctrl_get_config_direct(dev_name(dev), pin, &conf.config, 1);
	if (result == 0)
		return snprintf(buf, sizeof(buf) - 1, "%d\n", (unsigned int)(conf.config & ADI_CONFIG_DRIVE_STRENGTH_MASK));
	else
		return -EIO;
}

static ssize_t adrv906x_pinctrl_drive_strength_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, DRIVE_STRENGTH);
}

static ssize_t adrv906x_pinctrl_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return snprintf(buf, sizeof(buf) - 1, "%d\n", pin);
}

static ssize_t adrv906x_pinctrl_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int tmp_pin = 0, result = 0;

	result = sscanf(buf, "%du", &tmp_pin);
	if (result == 1) {
		if (tmp_pin >= 0 && tmp_pin <= ADI_ADRV906X_PIN_COUNT) {
			pin = tmp_pin;
		} else {
			pr_err("Failed in %s, setting for pin = %d is out of range!\n", __func__, tmp_pin);
			return -EINVAL;
		}
		return count;
	} else {
		return -EIO;
	}
}

#define ADRV906X_DEVICE_ATTR(_name) \
	DEVICE_ATTR(_name, S_IRUGO | S_IWUSR, adrv906x_pinctrl_ ## _name ## _show, adrv906x_pinctrl_ ## _name ## _store)
static ADRV906X_DEVICE_ATTR(drive_strength);
static ADRV906X_DEVICE_ATTR(pin);

#ifdef ADRV906X_PINCTRL_MORE_CONFIGS_OPT_IN
static ADRV906X_DEVICE_ATTR(config);
static ADRV906X_DEVICE_ATTR(schmitt_trig_enable);
static ADRV906X_DEVICE_ATTR(pin_pull_enablement);
static ADRV906X_DEVICE_ATTR(pin_pull_up_enable);
static ADRV906X_DEVICE_ATTR(mux_sel);

static ssize_t adrv906x_pinctrl_config_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	return snprintf(buf, sizeof(buf) - 1, "0x%x\n", adrv906x_pinctrl_drv->config);
}

static ssize_t adrv906x_pinctrl_config_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, CONFIG);
}

static ssize_t adrv906x_pinctrl_schmitt_trig_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	return snprintf(buf, sizeof(buf) - 1, "0x%x\n", adrv906x_pinctrl_drv->schmitt_trig_enable);
}

static ssize_t adrv906x_pinctrl_schmitt_trig_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, SCHMITT_TRIG_ENABLE);
}

static ssize_t adrv906x_pinctrl_pin_pull_enablement_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	return snprintf(buf, sizeof(buf) - 1, "0x%x\n", adrv906x_pinctrl_drv->pin_pull_enablement);
}

static ssize_t adrv906x_pinctrl_pin_pull_enablement_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, PIN_PULL_ENABLEMENT);
}

static ssize_t adrv906x_pinctrl_pin_pull_up_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	return snprintf(buf, sizeof(buf) - 1, "0x%x\n", adrv906x_pinctrl_drv->pin_pull_up_enable);
}

static ssize_t adrv906x_pinctrl_pin_pull_up_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, PIN_PULL_UP_ENABLE);
}

static ssize_t adrv906x_pinctrl_mux_sel_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	return snprintf(buf, sizeof(buf) - 1, "0x%x\n", adrv906x_pinctrl_drv->mux_sel);
}

static ssize_t adrv906x_pinctrl_mux_sel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, MUX_SEL);
}
#endif

static struct attribute *adrv906x_pinctrl_attrs[] = {
	&dev_attr_drive_strength.attr,
	&dev_attr_pin.attr,
#ifdef ADRV906X_PINCTRL_MORE_CONFIGS_OPT_IN
	&dev_attr_config.attr,
	&dev_attr_schmitt_trig_enable.attr,
	&dev_attr_pin_pull_enablement.attr,
	&dev_attr_pin_pull_up_enable.attr,
	&dev_attr_mux_sel.attr,
#endif
	NULL
};

static struct attribute_group adrv906x_pinctrl_group = {
	.name	= "control",
	.attrs	= adrv906x_pinctrl_attrs,
};

static struct attribute_group *adrv906x_pinctrl_groups[] = {
	&adrv906x_pinctrl_group,
	NULL
};

static const struct of_device_id adi_adrv906x_pinctrl_of_match[] = {
	{ .compatible = "adi,adrv906x-pinctrl", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, adi_adrv906x_pinctrl_of_match);

static struct adi_pinctrl_soc_info adi_adrv906x_pinctrl_info = {
	.pins			= adi_adrv906x_pinctrl_pads,
	.npins			= ARRAY_SIZE(adi_adrv906x_pinctrl_pads),
	.adi_pinconf_get	= adi_pinconf_get_smc,
	.adi_pinconf_set	= adi_pinconf_set_smc,
	.adi_pinctrl_parse_pin	= NULL,
};

static int adi_adrv906x_pinctrl_probe(struct platform_device *pdev)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv;
	int ret;

	ret = adi_pinctrl_probe(pdev, &adi_adrv906x_pinctrl_info);
	if (ret)
		return ret;

	adrv906x_pinctrl_drv = devm_kzalloc(&pdev->dev, sizeof(*adrv906x_pinctrl_drv), GFP_KERNEL);
	adrv906x_pinctrl_drv->dev = &pdev->dev;
	platform_set_drvdata(pdev, adrv906x_pinctrl_drv);

	ret = sysfs_create_groups(&pdev->dev.kobj, (const struct attribute_group **)adrv906x_pinctrl_groups);
	if (ret) {
		dev_err(&pdev->dev, "sysfs creation failed\n");
		return ret;
	}

	return ret;
}

static int adi_adrv906x_pinctrl_remove(struct platform_device *pdev)
{
	sysfs_remove_groups(&pdev->dev.kobj, (const struct attribute_group **)adrv906x_pinctrl_groups);
	return 0;
}

static struct platform_driver adi_adrv906x_pinctrl_driver = {
	.driver				={
		.name			= "adrv906x-pinctrl",
		.of_match_table		= of_match_ptr(adi_adrv906x_pinctrl_of_match),
		.suppress_bind_attrs	= true,
	},
	.probe				= adi_adrv906x_pinctrl_probe,
	.remove				= adi_adrv906x_pinctrl_remove,
};

static int __init adi_adrv906x_pinctrl_init(void)
{
	pin = 0;
	return platform_driver_register(&adi_adrv906x_pinctrl_driver);
}
arch_initcall(adi_adrv906x_pinctrl_init);

MODULE_AUTHOR("Howard Massey <Howard.Massey@analog.com>");
MODULE_DESCRIPTION("ADI ADRV906X pinctrl driver");
MODULE_LICENSE("GPL v2");
