// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for sensors and fan control on Siemens SIMATIC IPC embedded controllers.
 *
 * Due to historical reasons the ACPI description for the ec doesn't
 * follow the standard ACPI descriptions for fans and hwmon sensors (using
 * the generic drivers/acpi/ec.c driver).
 *
 * This driver binds to Siemens SIMATIC IPC embedded controller sensor devices.
 *
 * Copyright (c) Siemens AG, 2025-2026
 *
 * Authors:
 *  Gao Nian <nian.gao@siemens.com>
 *  Benedikt Niedermayr <benedikt.niedermayr@siemens.com>
 */
#include <linux/acpi.h>
#include <linux/ctype.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/unaligned.h>

/*
 * The ACPI ID domain "110A" belongs to Siemens
 * https://admin.pci-ids.ucw.cz/read/PC/110a
 */
#define SIEMENS_FAN_HID		"110A5002"
#define SIEMENS_HWMON_HID	"110A5004"

/*
 * All temperature sensors are located in an ACPI device with SIEMENS_HWMON_HID.
 */
#define TEMP_REG_CTR_STATUS		0	/* u8 */
#define TEMP_REG_VALUE			1	/* char */
#define TEMP_REG_ALARM_LOWLIMIT		2	/* char */
#define TEMP_REG_ALARM_HIGHLIMIT	3	/* char */
#define TEMP_REG_CRITICAL		4	/* char */

#define SIZE_OF_TEMP_REG		5

#define TEMP_MAX_CHANNELS		8

#define CRITICAL_SUPPORT		BIT(5)
#define ALARM_SUPPORT			BIT(4)

/*
 * Each fan device is a separate ACPI device with the same SIEMENS_FAN_HID.
 */
#define FAN_REG_CTR_STATUS		0	/* u8 */
#define FAN_REG_SPEED			1	/* u16 */
#define FAN_REG_MODE			3	/* u8 */
#define FAN_REG_DUTY_LEVEL		4	/* u8 */

/*
 * CONTROL MODE:
 * 0: Full speed
 * 1: Manual fan speed control enabled
 * 2: Automatic fan speed control enabled
 */
#define CONTROL_MODE_MASK		(BIT(1) | BIT(0))
#define CONTROL_MODE_GET(mode)		((mode) & CONTROL_MODE_MASK)

/*
 * The number of temperature sensor's channels comes from ACPI as it may be different for
 * different projects. Channels and limits that are not present are hidden by is_visible().
 */
struct simatic_ipc_ec_temp {
	u8 base;
	u8 count;
	u8 status[TEMP_MAX_CHANNELS];
};

struct simatic_ipc_ec_fan {
	u8 base;
	const char *label;
};

struct simatic_ipc_ec_sensor {
	struct simatic_ipc_ec_temp *temp;	/* Optional */
	struct simatic_ipc_ec_fan *fan;	/* Optional */
};

static inline u8 temp_reg(u8 base, u8 ch, u8 offset)
{
	return base + ch * SIZE_OF_TEMP_REG + offset;
}

static inline u8 fan_reg(u8 base, u8 offset)
{
	return base + offset;
}

static umode_t simatic_ipc_ec_temp_is_visible(const struct simatic_ipc_ec_temp *temp,
					      u32 attr, int channel)
{
	if (channel >= temp->count)
		return 0;

	switch (attr) {
	case hwmon_temp_input:
		return 0444;
	case hwmon_temp_max:
	case hwmon_temp_min:
		if (temp->status[channel] & ALARM_SUPPORT)
			return 0644;
		break;
	case hwmon_temp_crit:
		if (temp->status[channel] & CRITICAL_SUPPORT)
			return 0644;
		break;
	default:
		break;
	}

	return 0;
}

static int simatic_ipc_ec_temp_read(const struct simatic_ipc_ec_temp *temp, u32 attr,
				    int channel, long *val)
{
	u8 value;
	int ret;
	u8 reg;

	switch (attr) {
	case hwmon_temp_input:
		reg = temp_reg(temp->base, channel, TEMP_REG_VALUE);
		break;
	case hwmon_temp_max:
		reg = temp_reg(temp->base, channel, TEMP_REG_ALARM_HIGHLIMIT);
		break;
	case hwmon_temp_min:
		reg = temp_reg(temp->base, channel, TEMP_REG_ALARM_LOWLIMIT);
		break;
	case hwmon_temp_crit:
		reg = temp_reg(temp->base, channel, TEMP_REG_CRITICAL);
		break;
	default:
		return -EOPNOTSUPP;
	}

	ret = ec_read(reg, &value);
	if (ret < 0)
		return ret;

	/* Signed degree Celsius to millidegree Celsius */
	*val = (s8)value * 1000;

	return 0;
}

static int simatic_ipc_ec_temp_write(const struct simatic_ipc_ec_temp *temp, u32 attr,
				     int channel, long val)
{
	s8 value;
	u8 reg;

	switch (attr) {
	case hwmon_temp_max:
		reg = temp_reg(temp->base, channel, TEMP_REG_ALARM_HIGHLIMIT);
		break;
	case hwmon_temp_min:
		reg = temp_reg(temp->base, channel, TEMP_REG_ALARM_LOWLIMIT);
		break;
	case hwmon_temp_crit:
		reg = temp_reg(temp->base, channel, TEMP_REG_CRITICAL);
		break;
	default:
		return -EOPNOTSUPP;
	}

	/* Millidegree Celsius to signed degree Celsius */
	val = clamp_val(val, S8_MIN * 1000L, S8_MAX * 1000L);
	value = DIV_ROUND_CLOSEST(val, 1000);

	return ec_write(reg, (u8)value);
}

static umode_t simatic_ipc_ec_fan_is_visible(const struct simatic_ipc_ec_fan *fan,
					     enum hwmon_sensor_types type, u32 attr,
					     int channel)
{
	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
		case hwmon_fan_label:
			return 0444;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
		case hwmon_pwm_enable:
			return 0644;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int simatic_ipc_ec_read_block(u8 addr, u8 *buf, u8 n)
{
	int ret;
	u8 i;

	for (i = 0; i < n; i++) {
		ret = ec_read(addr + i, &buf[i]);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int simatic_ipc_ec_fan_read(struct simatic_ipc_ec_fan *fan, enum hwmon_sensor_types type,
				   u32 attr, int channel, long *val)
{
	u8 buf[2];
	u8 value;
	int ret;

	switch (type) {
	case hwmon_fan:
		switch (attr) {
		case hwmon_fan_input:
			ret = simatic_ipc_ec_read_block(fan_reg(fan->base, FAN_REG_SPEED),
							buf, sizeof(buf));
			if (ret < 0)
				return ret;
			*val = get_unaligned_le16(buf);
			return 0;
		default:
			break;
		}
		break;
	case hwmon_pwm:
		switch (attr) {
		case hwmon_pwm_input:
			ret = ec_read(fan_reg(fan->base, FAN_REG_DUTY_LEVEL), &value);
			if (ret < 0)
				return ret;
			*val = value;
			return 0;
		case hwmon_pwm_enable:
			ret = ec_read(fan_reg(fan->base, FAN_REG_MODE), &value);
			if (ret < 0)
				return ret;
			*val = CONTROL_MODE_GET(value);
			return 0;
		default:
			break;
		}
		break;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int simatic_ipc_ec_fan_read_string(struct simatic_ipc_ec_fan *fan, u32 attr,
					  const char **str)
{
	switch (attr) {
	case hwmon_fan_label:
		*str = fan->label;
		return 0;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int simatic_ipc_ec_fan_set_control_mode(struct simatic_ipc_ec_fan *fan, u8 ctrl_mode)
{
	int ret;
	u8 mode;

	ret = ec_read(fan_reg(fan->base, FAN_REG_MODE), &mode);
	if (ret < 0)
		return ret;

	mode &= ~CONTROL_MODE_MASK;
	mode |= ctrl_mode;

	ret = ec_write(fan_reg(fan->base, FAN_REG_MODE), mode);
	if (ret < 0)
		return ret;

	return 0;
}

static int simatic_ipc_ec_fan_write(struct simatic_ipc_ec_fan *fan, u32 attr, int channel,
				    long val)
{
	switch (attr) {
	case hwmon_pwm_input:
		if (val < 0 || val > 0xFF)
			return -EINVAL;
		return ec_write(fan_reg(fan->base, FAN_REG_DUTY_LEVEL), (u8)val);
	case hwmon_pwm_enable:
		/* 0: full speed, 1: manual control, 2: auto-control */
		if (val < 0 || val > 2)
			return -EINVAL;
		return simatic_ipc_ec_fan_set_control_mode(fan, val);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static umode_t simatic_ipc_ec_is_visible(const void *drvdata, enum hwmon_sensor_types type,
					 u32 attr, int channel)
{
	const struct simatic_ipc_ec_sensor *data = drvdata;
	umode_t ret = 0;

	switch (type) {
	case hwmon_temp:
		if (data->temp)
			ret = simatic_ipc_ec_temp_is_visible(data->temp, attr, channel);
		break;
	case hwmon_fan:
	case hwmon_pwm:
		if (data->fan)
			ret = simatic_ipc_ec_fan_is_visible(data->fan, type, attr, channel);
		break;
	default:
		break;
	}

	return ret;
}

static int simatic_ipc_ec_read(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long *val)
{
	struct simatic_ipc_ec_sensor *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		return simatic_ipc_ec_temp_read(data->temp, attr, channel, val);
	case hwmon_fan:
	case hwmon_pwm:
		return simatic_ipc_ec_fan_read(data->fan, type, attr, channel, val);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int simatic_ipc_ec_read_string(struct device *dev, enum hwmon_sensor_types type,
				      u32 attr, int channel, const char **str)
{
	struct simatic_ipc_ec_sensor *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_fan:
	case hwmon_pwm:
		return simatic_ipc_ec_fan_read_string(data->fan, attr, str);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int simatic_ipc_ec_write(struct device *dev, enum hwmon_sensor_types type,
				u32 attr, int channel, long val)
{
	struct simatic_ipc_ec_sensor *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		return simatic_ipc_ec_temp_write(data->temp, attr, channel, val);
	case hwmon_fan:
	case hwmon_pwm:
		return simatic_ipc_ec_fan_write(data->fan, attr, channel, val);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops simatic_ipc_ec_ops = {
	.is_visible = simatic_ipc_ec_is_visible,
	.read = simatic_ipc_ec_read,
	.read_string = simatic_ipc_ec_read_string,
	.write = simatic_ipc_ec_write,
};

#define SIMATIC_IPC_EC_TEMP_CONFIG \
	(HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_CRIT)

/* One entry per channel, up to TEMP_MAX_CHANNELS */
static const struct hwmon_channel_info * const simatic_ipc_ec_hwmon_ch_info[] = {
	HWMON_CHANNEL_INFO(temp,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG,
			   SIMATIC_IPC_EC_TEMP_CONFIG),
	NULL
};

static const struct hwmon_channel_info * const simatic_ipc_ec_fan_ch_info[] = {
	HWMON_CHANNEL_INFO(fan, HWMON_F_INPUT | HWMON_F_LABEL),
	HWMON_CHANNEL_INFO(pwm, HWMON_PWM_INPUT | HWMON_PWM_ENABLE),
	NULL
};

static const struct hwmon_chip_info simatic_ipc_ec_hwmon_info = {
	.ops = &simatic_ipc_ec_ops,
	.info = simatic_ipc_ec_hwmon_ch_info,
};

static const struct hwmon_chip_info simatic_ipc_ec_fan_info = {
	.ops = &simatic_ipc_ec_ops,
	.info = simatic_ipc_ec_fan_ch_info,
};

static int simatic_ipc_ec_hwmon_probe(struct device *dev, struct simatic_ipc_ec_sensor *data)
{
	struct simatic_ipc_ec_temp *temp;
	struct device *hwmon_dev;
	u8 temp_count;
	u8 temp_base;
	int ret;
	u8 ch;

	ret = device_property_read_u8(dev, "TempRegOffset", &temp_base);
	if (ret < 0)
		return ret;

	ret = device_property_read_u8(dev, "TempCount", &temp_count);
	if (ret < 0)
		return ret;

	if (!temp_count || temp_count > TEMP_MAX_CHANNELS ||
	    temp_base + temp_count * SIZE_OF_TEMP_REG > 256)
		return -EINVAL;

	temp = devm_kzalloc(dev, sizeof(*temp), GFP_KERNEL);
	if (!temp)
		return -ENOMEM;

	temp->base = temp_base;
	temp->count = temp_count;

	data->temp = temp;

	for (ch = 0; ch < temp_count; ch++) {
		ret = ec_read(temp_reg(temp_base, ch, TEMP_REG_CTR_STATUS), &temp->status[ch]);
		if (ret < 0)
			return ret;
	}

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "simatic_ipc_ec_hwmon",
							 data, &simatic_ipc_ec_hwmon_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static int simatic_ipc_ec_fan_probe(struct device *dev, struct simatic_ipc_ec_sensor *data)
{
	struct device *hwmon_dev;
	struct simatic_ipc_ec_fan *fan;
	const char *label, *fan_name = NULL;
	char *chip_name, *p;
	u8 fan_base;
	int ret;

	ret = device_property_read_u8(dev, "RegOffset", &fan_base);
	if (ret < 0)
		return ret;

	ret = device_property_read_string(dev, "Label", &label);
	if (ret < 0)
		return ret;

	device_property_read_string(dev, "FanName", &fan_name);

	fan = devm_kzalloc(dev, sizeof(*fan), GFP_KERNEL);
	if (!fan)
		return -ENOMEM;

	fan->base = fan_base;
	fan->label = fan_name ? fan_name : label;

	chip_name = devm_kasprintf(dev, GFP_KERNEL, "simatic_ipc_ec_%s", fan->label);
	if (!chip_name)
		return -ENOMEM;

	chip_name = devm_hwmon_sanitize_name(dev, chip_name);
	if (IS_ERR(chip_name))
		return PTR_ERR(chip_name);

	for (p = chip_name; *p; p++)
		*p = tolower(*p);

	data->fan = fan;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, chip_name, data,
							 &simatic_ipc_ec_fan_info,
							 NULL);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static int simatic_ipc_ec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct simatic_ipc_ec_sensor *data;
	const char *hid;
	int ret;

	/* The same HID sensor device may not be on the EC. */
	ret = device_property_match_string(dev, "ChipType", "ec");
	if (ret < 0)
		return -ENODEV;

	hid = acpi_device_hid(to_acpi_device_node(dev->fwnode));

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	if (!strcmp(hid, SIEMENS_HWMON_HID))
		ret = simatic_ipc_ec_hwmon_probe(dev, data);
	else if (!strcmp(hid, SIEMENS_FAN_HID))
		ret = simatic_ipc_ec_fan_probe(dev, data);
	else
		ret = -EINVAL;

	return ret;
}

static const struct acpi_device_id simatic_ipc_ec_sensors_ids[] = {
	{SIEMENS_HWMON_HID},
	{SIEMENS_FAN_HID},
	{}
};

MODULE_DEVICE_TABLE(acpi, simatic_ipc_ec_sensors_ids);

static struct platform_driver simatic_ipc_ec_sensors_driver = {
	.probe = simatic_ipc_ec_probe,
	.driver = {
		.name = KBUILD_MODNAME,
		.acpi_match_table = simatic_ipc_ec_sensors_ids
	},
};

module_platform_driver(simatic_ipc_ec_sensors_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Gao Nian <nian.gao@siemens.com>");
MODULE_DESCRIPTION("Siemens SIMATIC IPC embedded controller sensors driver");
