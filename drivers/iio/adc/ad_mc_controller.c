/*
 * Analog Devices MC-Controller Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/dmaengine.h>
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"

#define MC_REG_VERSION			0x00
#define MC_REG_ID			0x04
#define MC_REG_SCRATCH			0x08
#define MC_REG_ENCODER_ZERO_OFFSET	0x0C
#define MC_REG_CONTROL			0x10
#define MC_REG_COMMAND			0x14
#define MC_REG_VELOCITY_P_GAIN		0x18
#define MC_REG_VELOCITY_I_GAIN		0x1C
#define MC_REG_CURRENT_P_GAIN		0x20
#define MC_REG_CURRENT_I_GAIN		0x24
#define MC_REG_OPEN_LOOP_BIAS		0x28
#define MC_REG_OPEN_LOOP_SCALAR		0x2C
#define MC_REG_PWM_OPEN			0x30
#define MC_REG_PWM_BREAK		0x34
#define MC_REG_STATUS			0x38
#define MC_REG_ERROR			0x3C

#define MC_CONTROL_RUN(x)		(((x) & 0x1) << 0)
#define MC_CONTROL_RESET_OVR_CURR(x)	(((x) & 0x1) << 1)
#define MC_CONTROL_BREAK(x)		(((x) & 0x1) << 2)
#define MC_CONTROL_DELTA(x)		(((x) & 0x1) << 4)
#define MC_CONTROL_SENSORS(x)		(((x) & 0x3) << 8)
#define MC_CONTROL_MATLAB(x)		(((x) & 0x1) << 12)
#define MC_CONTROL_CONTROLLER_MODE(x)	(((x) & 0x3) << 16)

static const char mc_adv_ctrl_sensors[3][8] = {"hall", "bemf", "resolver"};
static const char mc_adv_ctrl_controller_mode[4][26] = {"error", "stand-by",
		"velocity-control", "open-loop-velocity-control"};

static inline void mc_adv_ctrl_write(struct axiadc_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int mc_adv_ctrl_read(struct axiadc_state *st,
	unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int mc_adv_ctrl_reg_access(struct iio_dev *indio_dev,
	unsigned reg, unsigned writeval, unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		mc_adv_ctrl_write(st, reg & 0xFFFF, writeval);
	else
		*readval = mc_adv_ctrl_read(st, reg & 0xFFFF);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

enum mc_adv_ctrl_iio_dev_attr {
	MC_ENCODER_ZERO_OFFSET,
	MC_RUN,
	MC_RESET_OVR_CURR,
	MC_BREAK,
	MC_DELTA,
	MC_SENSORS_AVAIL,
	MC_SENSORS,
	MC_MATLAB,
	MC_CONTROLLER_MODE_AVAIL,
	MC_CONTROLLER_MODE,
	MC_VELOCITY_P_GAIN,
	MC_VELOCITY_I_GAIN,
	MC_CURRENT_P_GAIN,
	MC_CURRENT_I_GAIN,
	MC_OPEN_LOOP_BIAS,
	MC_OPEN_LOOP_SCALAR,
	MC_COMMAND,
	MC_PWM,
	MC_PWM_BREAK,
	MC_STATUS,
	MC_ERROR,
};

static ssize_t mc_adv_ctrl_show(struct device *dev,
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
	case MC_ENCODER_ZERO_OFFSET:
		reg_val = mc_adv_ctrl_read(st, MC_REG_ENCODER_ZERO_OFFSET);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_RUN:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_RUN(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_RESET_OVR_CURR:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_RESET_OVR_CURR(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_BREAK:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_BREAK(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_DELTA:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_DELTA(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_SENSORS_AVAIL:
		ret = sprintf(buf, "%s\n", "hall bemf resolver");
		break;
	case MC_SENSORS:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting2 = (reg_val & MC_CONTROL_SENSORS(-1)) >> 8;
		ret = sprintf(buf, "%s\n", mc_adv_ctrl_sensors[setting2]);
		break;
	case MC_MATLAB:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting = (reg_val & MC_CONTROL_MATLAB(-1));
		ret = sprintf(buf, "%u\n", setting);
		break;
	case MC_CONTROLLER_MODE_AVAIL:
		ret = sprintf(buf, "%s\n", "error stand-by velocity-control open-loop-velocity-control");
		break;
	case MC_CONTROLLER_MODE:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		setting2 = (reg_val & MC_CONTROL_CONTROLLER_MODE(-1)) >> 16;
		ret = sprintf(buf, "%s\n", mc_adv_ctrl_controller_mode[setting2]);
		break;
	case MC_VELOCITY_P_GAIN:
		reg_val = mc_adv_ctrl_read(st, MC_REG_VELOCITY_P_GAIN);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_VELOCITY_I_GAIN:
		reg_val = mc_adv_ctrl_read(st, MC_REG_VELOCITY_I_GAIN);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_CURRENT_P_GAIN:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CURRENT_P_GAIN);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_CURRENT_I_GAIN:
		reg_val = mc_adv_ctrl_read(st, MC_REG_CURRENT_I_GAIN);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_OPEN_LOOP_BIAS:
		reg_val = mc_adv_ctrl_read(st, MC_REG_OPEN_LOOP_BIAS);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_OPEN_LOOP_SCALAR:
		reg_val = mc_adv_ctrl_read(st, MC_REG_OPEN_LOOP_SCALAR);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_COMMAND:
		reg_val = mc_adv_ctrl_read(st, MC_REG_COMMAND);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_PWM:
		reg_val = mc_adv_ctrl_read(st, MC_REG_PWM_OPEN);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_PWM_BREAK:
		reg_val = mc_adv_ctrl_read(st, MC_REG_PWM_BREAK);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_STATUS:
		reg_val = mc_adv_ctrl_read(st, MC_REG_STATUS);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	case MC_ERROR:
		reg_val = mc_adv_ctrl_read(st, MC_REG_ERROR);
		ret = sprintf(buf, "%d\n", reg_val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t mc_adv_ctrl_store(struct device *dev,
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
	case MC_ENCODER_ZERO_OFFSET:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_ENCODER_ZERO_OFFSET, reg_val);
		break;
	case MC_RUN:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_RUN(-1);
		reg_val |= MC_CONTROL_RUN(setting);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_RESET_OVR_CURR:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_RESET_OVR_CURR(-1);
		reg_val |= MC_CONTROL_RESET_OVR_CURR(setting);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_BREAK:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_BREAK(-1);
		reg_val |= MC_CONTROL_BREAK(setting);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_DELTA:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_DELTA(-1);
		reg_val |= MC_CONTROL_DELTA(setting);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
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
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_SENSORS(-1);
		reg_val |= MC_CONTROL_SENSORS(setting2);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_MATLAB:
		ret = strtobool(buf, &setting);
		if (ret < 0)
			break;
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_MATLAB(-1);
		reg_val |= MC_CONTROL_MATLAB(setting);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_CONTROLLER_MODE:
		if (sysfs_streq(buf, "error"))
			setting2 = 0;
		else if (sysfs_streq(buf, "stand-by"))
			setting2 = 1;
		else if (sysfs_streq(buf, "velocity-control"))
			setting2 = 2;
		else if (sysfs_streq(buf, "open-loop-velocity-control"))
			setting2 = 3;
		else
			break;
		reg_val = mc_adv_ctrl_read(st, MC_REG_CONTROL);
		reg_val &= ~MC_CONTROL_CONTROLLER_MODE(-1);
		reg_val |= MC_CONTROL_CONTROLLER_MODE(setting2);
		mc_adv_ctrl_write(st, MC_REG_CONTROL, reg_val);
		break;
	case MC_VELOCITY_P_GAIN:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_VELOCITY_P_GAIN, reg_val);
		break;
	case MC_VELOCITY_I_GAIN:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_VELOCITY_I_GAIN, reg_val);
		break;
	case MC_CURRENT_P_GAIN:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_CURRENT_P_GAIN, reg_val);
		break;
	case MC_CURRENT_I_GAIN:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_CURRENT_I_GAIN, reg_val);
		break;
	case MC_OPEN_LOOP_BIAS:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_OPEN_LOOP_BIAS, reg_val);
		break;
	case MC_OPEN_LOOP_SCALAR:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_OPEN_LOOP_SCALAR, reg_val);
		break;
	case MC_COMMAND:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_COMMAND, reg_val);
		break;
	case MC_PWM:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_PWM_OPEN, reg_val);
		break;
	case MC_PWM_BREAK:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_PWM_BREAK, reg_val);
		break;
	case MC_STATUS:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_STATUS, reg_val);
		break;
	case MC_ERROR:
		ret = kstrtou32(buf, 10, &reg_val);
		if (ret < 0)
			break;
		mc_adv_ctrl_write(st, MC_REG_ERROR, reg_val);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(mc_adv_ctrl_encoder_zero_offset, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_ENCODER_ZERO_OFFSET);

static IIO_DEVICE_ATTR(mc_adv_ctrl_run, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_RUN);

static IIO_DEVICE_ATTR(mc_adv_ctrl_reset_overcurrent, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_RESET_OVR_CURR);

static IIO_DEVICE_ATTR(mc_adv_ctrl_break, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_BREAK);

static IIO_DEVICE_ATTR(mc_adv_ctrl_delta, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_DELTA);

static IIO_DEVICE_ATTR(mc_adv_ctrl_sensors_available, S_IRUGO,
			mc_adv_ctrl_show,
			NULL,
			MC_SENSORS_AVAIL);

static IIO_DEVICE_ATTR(mc_adv_ctrl_sensors, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_SENSORS);

static IIO_DEVICE_ATTR(mc_adv_ctrl_matlab, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_MATLAB);

static IIO_DEVICE_ATTR(mc_adv_ctrl_controller_mode_available, S_IRUGO,
			mc_adv_ctrl_show,
			NULL,
			MC_CONTROLLER_MODE_AVAIL);

static IIO_DEVICE_ATTR(mc_adv_ctrl_controller_mode, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_CONTROLLER_MODE);

static IIO_DEVICE_ATTR(mc_adv_ctrl_velocity_p_gain, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_VELOCITY_P_GAIN);

static IIO_DEVICE_ATTR(mc_adv_ctrl_velocity_i_gain, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_VELOCITY_I_GAIN);

static IIO_DEVICE_ATTR(mc_adv_ctrl_current_p_gain, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_CURRENT_P_GAIN);

static IIO_DEVICE_ATTR(mc_adv_ctrl_current_i_gain, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_CURRENT_I_GAIN);

static IIO_DEVICE_ATTR(mc_adv_ctrl_open_loop_bias, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_OPEN_LOOP_BIAS);

static IIO_DEVICE_ATTR(mc_adv_ctrl_open_loop_scalar, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_OPEN_LOOP_SCALAR);

static IIO_DEVICE_ATTR(mc_adv_ctrl_command, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_COMMAND);

static IIO_DEVICE_ATTR(mc_adv_ctrl_pwm, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_PWM);

static IIO_DEVICE_ATTR(mc_adv_ctrl_pwm_break, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_PWM_BREAK);

static IIO_DEVICE_ATTR(mc_adv_ctrl_status, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_STATUS);

static IIO_DEVICE_ATTR(mc_adv_ctrl_error, S_IRUGO | S_IWUSR,
			mc_adv_ctrl_show,
			mc_adv_ctrl_store,
			MC_ERROR);

static struct attribute *mc_adv_ctrl_attributes[] = {
	&iio_dev_attr_mc_adv_ctrl_encoder_zero_offset.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_run.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_reset_overcurrent.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_break.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_delta.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_sensors_available.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_sensors.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_matlab.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_controller_mode_available.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_controller_mode.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_velocity_p_gain.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_velocity_i_gain.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_current_p_gain.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_current_i_gain.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_open_loop_bias.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_open_loop_scalar.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_command.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_pwm.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_pwm_break.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_status.dev_attr.attr,
	&iio_dev_attr_mc_adv_ctrl_error.dev_attr.attr,
	NULL,
};

static const struct attribute_group mc_adv_ctrl_attribute_group = {
	.attrs = mc_adv_ctrl_attributes,
};

static int mc_adv_ctrl_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = mc_adv_ctrl_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;

		mc_adv_ctrl_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static const struct iio_info mc_adv_ctrl_info = {
	.driver_module = THIS_MODULE,
	.debugfs_reg_access = &mc_adv_ctrl_reg_access,
	.attrs = &mc_adv_ctrl_attribute_group,
	.update_scan_mode = &mc_adv_ctrl_update_scan_mode,
};

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)		  \
	{ .type = IIO_VOLTAGE,					  \
	  .indexed = 1,						 \
	  .channel = _chan,					 \
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

static int mc_adv_ctrl_probe(struct platform_device *pdev)
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
	indio_dev->info = &mc_adv_ctrl_info;

	/* Reset all HDL Cores */
	mc_adv_ctrl_write(st, ADI_REG_RSTN, 0);
	mc_adv_ctrl_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);

	st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 'u');
	st->channels[1] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(1, 1, 16, 'u');

	indio_dev->channels = st->channels;
	indio_dev->num_channels = 2;
	indio_dev->masklength = 2;

	axiadc_configure_ring(indio_dev, "ad-mc-adv-ctrl-dma");

	ret = iio_buffer_register(indio_dev, indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto err_unconfigure_ring;

	*indio_dev->buffer->scan_mask = (1UL << 2) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_iio_buffer_unregister;

	return 0;

err_iio_buffer_unregister:
	iio_buffer_unregister(indio_dev);
err_unconfigure_ring:
	axiadc_unconfigure_ring(indio_dev);

	return ret;
}

static int mc_adv_ctrl_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	axiadc_unconfigure_ring(indio_dev);

	return 0;
}

static const struct of_device_id mc_adv_ctrl_of_match[] = {
	{ .compatible = "xlnx,axi-ad-mc-adv-ctrl-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, mc_adv_ctrl_of_match);

static struct platform_driver mc_adv_ctrl_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = mc_adv_ctrl_of_match,
	},
	.probe	  = mc_adv_ctrl_probe,
	.remove	 = mc_adv_ctrl_remove,
};

module_platform_driver(mc_adv_ctrl_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-Advanced-Controller");
MODULE_LICENSE("GPL v2");
