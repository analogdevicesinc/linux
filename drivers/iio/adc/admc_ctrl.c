/*
 * Analog Devices MC-Controller Module
 *
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include "cf_axi_adc.h"

#define MC_REG_VERSION			0x00
#define MC_REG_ID			0x04
#define MC_REG_SCRATCH			0x08
#define MC_REG_START_SPEED		0x0C
#define MC_REG_CONTROL			0x10
#define MC_REG_REFERENCE_SPEED		0x14
#define MC_REG_KP			0x18
#define MC_REG_KI			0x1C
#define MC_REG_KD			0x20
#define MC_REG_PWM_OPEN			0x30
#define MC_REG_PWM_BREAK		0x34
#define MC_REG_STATUS			0x38
#define MC_REG_ERR			0x3C

#define MC_CONTROL_RUN(x)		(((x) & 0x1) << 0)
#define MC_CONTROL_RESET_OVR_CURR(x)	(((x) & 0x1) << 1)
#define MC_CONTROL_BREAK(x)		(((x) & 0x1) << 2)
#define MC_CONTROL_DIRECTION(x)		(((x) & 0x1) << 3)
#define MC_CONTROL_DELTA(x)		(((x) & 0x1) << 4)
#define MC_CONTROL_SENSORS(x)		(((x) & 0x3) << 8)
#define MC_CONTROL_MATLAB(x)		(((x) & 0x1) << 12)
#define MC_CONTROL_CALIB_ADC(x)		(((x) & 0x1) << 16)
#define MC_CONTROL_GPO(x)		(((x) & 0x7FF) << 20)

static const char mc_ctrl_sensors[3][8] = {"hall", "bemf", "resolver"};

static inline void mc_ctrl_write(struct axiadc_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int mc_ctrl_read(struct axiadc_state *st,
	unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int mc_ctrl_reg_access(struct iio_dev *indio_dev,
	unsigned reg, unsigned writeval, unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		mc_ctrl_write(st, reg & 0xFFFF, writeval);
	else
		*readval = mc_ctrl_read(st, reg & 0xFFFF);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

enum mc_ctrl_iio_dev_attr {
	MC_RUN,
	MC_RESET_OVR_CURR,
	MC_BREAK,
	MC_DIRECTION,
	MC_DELTA,
	MC_SENSORS_AVAIL,
	MC_SENSORS,
	MC_MATLAB,
	MC_CALIB_ADC,
	MC_GPO,
	MC_REF_SPEED,
	MC_KP,
	MC_KI,
	MC_KD,
	MC_PWM,
	MC_PWM_BREAK,
	MC_STATUS,
};

static ssize_t mc_ctrl_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_state *st = iio_priv(indio_dev);
	int ret = 0;
	bool setting;
	u32 reg_val;
	u32 setting2;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case MC_RUN:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_RUN(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_RESET_OVR_CURR:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_RESET_OVR_CURR(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_BREAK:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_BREAK(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_DIRECTION:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_DIRECTION(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_DELTA:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_DELTA(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_SENSORS_AVAIL:
		ret = sprintf(buf, "%s\n", "hall bemf resolver");
		break;
	case MC_SENSORS:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting2 = (reg_val & MC_CONTROL_SENSORS(-1)) >> 8;
		ret = sprintf(buf, "%s\n", mc_ctrl_sensors[setting2]);
		break;
	case MC_MATLAB:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_MATLAB(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_CALIB_ADC:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_CALIB_ADC(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_GPO:
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		setting2 = (reg_val & MC_CONTROL_GPO(-1)) >> 20;
		ret = sprintf(buf, "%u\n", setting2);
		break;
	case MC_REF_SPEED:
		reg_val = mc_ctrl_read(st, MC_REG_REFERENCE_SPEED);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_KI:
		reg_val = mc_ctrl_read(st, MC_REG_KI);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_KP:
		reg_val = mc_ctrl_read(st, MC_REG_KP);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_KD:
		reg_val = mc_ctrl_read(st, MC_REG_KD);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_PWM:
		reg_val = mc_ctrl_read(st, MC_REG_PWM_OPEN);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_PWM_BREAK:
		reg_val = mc_ctrl_read(st, MC_REG_PWM_BREAK);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_STATUS:
		reg_val = mc_ctrl_read(st, MC_REG_STATUS);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t mc_ctrl_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_state *st = iio_priv(indio_dev);
	int ret = 0;
	bool setting;
	u32 reg_val;
	u32 setting2;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case MC_RUN:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_RUN(-1);
		reg_val |= MC_CONTROL_RUN(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_RESET_OVR_CURR:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_RESET_OVR_CURR(-1);
		reg_val |= MC_CONTROL_RESET_OVR_CURR(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_BREAK:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_BREAK(-1);
		reg_val |= MC_CONTROL_BREAK(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_DIRECTION:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_DIRECTION(-1);
		reg_val |= MC_CONTROL_DIRECTION(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_DELTA:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_DELTA(-1);
		reg_val |= MC_CONTROL_DELTA(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_SENSORS:
		if (sysfs_streq(buf, "hall"))
			setting2 = 0;
		else if (sysfs_streq(buf, "bemf"))
			setting2 = 1;
		else if (sysfs_streq(buf, "resolver"))
			setting2 = 2;
		else
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_SENSORS(-1);
		reg_val |= MC_CONTROL_SENSORS(setting2);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_MATLAB:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_MATLAB(-1);
		reg_val |= MC_CONTROL_MATLAB(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_CALIB_ADC:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_CALIB_ADC(-1);
		reg_val |= MC_CONTROL_CALIB_ADC(setting);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_GPO:
		ret = kstrtou32(buf, 10, &setting2);
		if (ret < 0)
			break;
		reg_val = mc_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_GPO(-1);
		reg_val |= MC_CONTROL_GPO(setting2);
		mc_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_REF_SPEED:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_REFERENCE_SPEED, reg_val);
		break;
	case MC_KI:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_KI, reg_val);
		break;
	case MC_KP:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_KP, reg_val);
		break;
	case MC_KD:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_KD, reg_val);
		break;
	case MC_PWM:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_PWM_OPEN, reg_val);
		break;
	case MC_PWM_BREAK:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_PWM_BREAK, reg_val);
		break;
	case MC_STATUS:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_ctrl_write(st, MC_REG_STATUS, reg_val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(mc_ctrl_run, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_RUN);

static IIO_DEVICE_ATTR(mc_ctrl_reset_overcurrent, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_RESET_OVR_CURR);

static IIO_DEVICE_ATTR(mc_ctrl_break, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_BREAK);

static IIO_DEVICE_ATTR(mc_ctrl_direction, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_DIRECTION);

static IIO_DEVICE_ATTR(mc_ctrl_delta, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_DELTA);

static IIO_DEVICE_ATTR(mc_ctrl_sensors_available, S_IRUGO,
			mc_ctrl_show,
			NULL,
			MC_SENSORS_AVAIL);

static IIO_DEVICE_ATTR(mc_ctrl_sensors, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_SENSORS);

static IIO_DEVICE_ATTR(mc_ctrl_matlab, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_MATLAB);

static IIO_DEVICE_ATTR(mc_ctrl_calibrate_adc, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_CALIB_ADC);

static IIO_DEVICE_ATTR(mc_ctrl_gpo, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_GPO);

static IIO_DEVICE_ATTR(mc_ctrl_ref_speed, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_REF_SPEED);

static IIO_DEVICE_ATTR(mc_ctrl_ki, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_KI);

static IIO_DEVICE_ATTR(mc_ctrl_kp, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_KP);

static IIO_DEVICE_ATTR(mc_ctrl_kd, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_KD);

static IIO_DEVICE_ATTR(mc_ctrl_pwm, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_PWM);

static IIO_DEVICE_ATTR(mc_ctrl_pwm_break, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_PWM_BREAK);

static IIO_DEVICE_ATTR(mc_ctrl_status, S_IRUGO | S_IWUSR,
			mc_ctrl_show,
			mc_ctrl_store,
			MC_STATUS);

static struct attribute *mc_ctrl_attributes[] = {
	&iio_dev_attr_mc_ctrl_run.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_reset_overcurrent.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_break.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_direction.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_delta.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_sensors_available.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_sensors.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_matlab.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_calibrate_adc.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_gpo.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_ref_speed.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_pwm.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_ki.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_kp.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_kd.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_pwm_break.dev_attr.attr,
	&iio_dev_attr_mc_ctrl_status.dev_attr.attr,
	NULL,
};

static const struct attribute_group mc_ctrl_attribute_group = {
	.attrs = mc_ctrl_attributes,
};

static const struct iio_info mc_ctrl_info = {
	.driver_module = THIS_MODULE,
	.debugfs_reg_access = &mc_ctrl_reg_access,
	.attrs = &mc_ctrl_attribute_group,
};

static int mc_ctrl_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &mc_ctrl_info;

	/* Reset all HDL Cores */
	mc_ctrl_write(st, ADI_REG_RSTN, 0);
	mc_ctrl_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);

	ret = iio_device_register(indio_dev);

	return ret;
}

static int mc_ctrl_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);

	return 0;
}

static const struct of_device_id mc_ctrl_of_match[] = {
	{ .compatible = "xlnx,axi-ad-mc-ctrl-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, mc_ctrl_of_match);

static struct platform_driver mc_ctrl_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = mc_ctrl_of_match,
	},
	.probe	  = mc_ctrl_probe,
	.remove	 = mc_ctrl_remove,
};

module_platform_driver(mc_ctrl_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-Controller");
MODULE_LICENSE("GPL v2");
