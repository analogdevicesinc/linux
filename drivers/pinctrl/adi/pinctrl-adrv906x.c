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
 * |-- config
 * |-- drive_strength
 * |-- mux_sel
 * |-- pin_pull_enablement
 * |-- pin_pull_up_enable
 * `-- schmitt_trig_enable
 *
 *  Usage examples:
 *  1) To set the config:
 *      Command: echo 1=5 > ./config
 *      Set pin 1's config value to 5.
 *  2) To get the config:
 *      Command: echo 2 > ./config
 *      Get pin 2's config value(result will be showed on the console.)
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
	int result = 0, scan_count = 0, pin, config_val;
	struct adi_pin_mio conf = { 0 };

	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	scan_count = sscanf(buf, "%d=%d", &pin, &config_val);
	if (scan_count == 1 || scan_count == 2) {
		if (pin < 0 || pin >= ADI_ADRV906X_PIN_COUNT) {
			pr_err("Pin number out of range.\n");
			return -EIO;
		}

		conf.input_pin = pin;
		result = adrv906x_pinctrl_get_config_direct(dev_name(dev), pin, &conf.config, 1);
		if (result == 0) {
			switch (st) {
			case CONFIG:
				pr_info("config read back for pin %d is 0x%lx\n", conf.input_pin, conf.config);
				break;
			case DRIVE_STRENGTH:
				pr_info("drive_strength read back for pin %d is 0x%lx\n", conf.input_pin, (unsigned long)(conf.config & ADI_CONFIG_DRIVE_STRENGTH_MASK));
				break;
			case SCHMITT_TRIG_ENABLE:
				pr_info("schmitt_trig_enable read back for pin %d is 0x%x\n", conf.input_pin, (conf.config & ADI_CONFIG_SCHMITT_TRIG_ENABLE_MASK) ? 1 : 0);
				break;
			case PIN_PULL_ENABLEMENT:
				pr_info("pin_pull_enablement read back for pin %d is 0x%x\n", conf.input_pin, (conf.config & ADI_CONFIG_PULL_UP_DOWN_ENABLEMENT_MASK) ? 1 : 0);
				break;
			case PIN_PULL_UP_ENABLE:
				pr_info("pin_pull_up_enable read back for pin %d is 0x%x\n", conf.input_pin, (conf.config & ADI_CONFIG_PULLUP_ENABLE_MASK) ? 1 : 0);
				break;
			case MUX_SEL:
				pr_info("mux_sel read back for pin %d is 0x%lx\n", conf.input_pin, (unsigned long)(conf.mux_sel & ADI_CONFIG_MUX_SEL_MASK));
				break;
			}
		} else {
			pr_err("getting config failed\n");
			return -EIO;
		}
	} else {
		pr_err("invalid input\n");
		return -EIO;
	}

	if (scan_count == 2) {
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
				pr_info("set config val 0x%x for pin %d succeeded\n", adrv906x_pinctrl_drv->config, conf.input_pin);
				break;
			case DRIVE_STRENGTH:
				adrv906x_pinctrl_drv->drive_strength = config_val & ADI_CONFIG_DRIVE_STRENGTH_MASK;
				pr_info("set drive_strength val 0x%x for pin %d succeeded\n", adrv906x_pinctrl_drv->drive_strength, conf.input_pin);
				break;
			case SCHMITT_TRIG_ENABLE:
				adrv906x_pinctrl_drv->schmitt_trig_enable = config_val & 0x1;
				pr_info("set schmitt_trig_enable val 0x%x for pin %d succeeded\n", adrv906x_pinctrl_drv->schmitt_trig_enable, conf.input_pin);
				break;
			case PIN_PULL_ENABLEMENT:
				adrv906x_pinctrl_drv->pin_pull_enablement = config_val & 0x1;
				pr_info("set pin_pull_enablement val 0x%x for pin %d succeeded\n", adrv906x_pinctrl_drv->pin_pull_enablement, conf.input_pin);
				break;
			case PIN_PULL_UP_ENABLE:
				adrv906x_pinctrl_drv->pin_pull_up_enable = config_val & 0x1;
				pr_info("set pin_pull_up_enable val 0x%x for pin %d succeeded\n", adrv906x_pinctrl_drv->pin_pull_up_enable, conf.input_pin);
				break;
			case MUX_SEL:
				adrv906x_pinctrl_drv->mux_sel = config_val & ADI_CONFIG_MUX_SEL_MASK;
				pr_info("set mux_sel val 0x%x for pin %d succeeded\n", adrv906x_pinctrl_drv->mux_sel, conf.input_pin);
				break;
			}
			return count;
		} else {
			pr_err("Set pinconfig failed!\n");
			return -EIO;
		}
	} else {
		return count;
	}
}

static ssize_t adrv906x_pinctrl_drive_strength_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct adrv906x_pinctrl_driver *adrv906x_pinctrl_drv = dev_get_drvdata(dev);

	if (!adrv906x_pinctrl_drv)
		return -EIO;

	return sprintf(buf, "0x%x\n", adrv906x_pinctrl_drv->drive_strength);
}

static ssize_t adrv906x_pinctrl_drive_strength_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, DRIVE_STRENGTH);
}

#define ADRV906X_DEVICE_ATTR(_name) \
	DEVICE_ATTR(_name, S_IRUGO | S_IWUSR, adrv906x_pinctrl_ ## _name ## _show, adrv906x_pinctrl_ ## _name ## _store)
static ADRV906X_DEVICE_ATTR(drive_strength);

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

	return sprintf(buf, "0x%x\n", adrv906x_pinctrl_drv->config);
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

	return sprintf(buf, "0x%x\n", adrv906x_pinctrl_drv->schmitt_trig_enable);
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

	return sprintf(buf, "0x%x\n", adrv906x_pinctrl_drv->pin_pull_enablement);
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

	return sprintf(buf, "0x%x\n", adrv906x_pinctrl_drv->pin_pull_up_enable);
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

	return sprintf(buf, "0x%x\n", adrv906x_pinctrl_drv->mux_sel);
}

static ssize_t adrv906x_pinctrl_mux_sel_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return adrv906x_pinctrl_common_store(dev, buf, count, MUX_SEL);
}
#endif

static struct attribute *adrv906x_pinctrl_attrs[] = {
	&dev_attr_drive_strength.attr,
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
	return platform_driver_register(&adi_adrv906x_pinctrl_driver);
}
arch_initcall(adi_adrv906x_pinctrl_init);

MODULE_AUTHOR("Howard Massey <Howard.Massey@analog.com>");
MODULE_DESCRIPTION("ADI ADRV906X pinctrl driver");
MODULE_LICENSE("GPL v2");
