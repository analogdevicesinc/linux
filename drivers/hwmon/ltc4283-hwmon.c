// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices LTC4283 I2C Negative Voltage Hot Swap Controller (HWMON)
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include "asm-generic/int-ll64.h"
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h> // devres.h when upstreaming
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>
#include <linux/math.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#define LTC4283_ADC_ALM_LOG_1		0x05
#define LTC4283_ADC_ALM_LOG_2		0x06
#define LTC4283_ADC_ALM_LOG_3		0x07
#define LTC4283_ADC_ALM_LOG_4		0x08
#define LTC4283_ADC_ALM_LOG_5		0x09
#define LTC4283_ADC_SELECT(c)		(0x13 + (c) / 8)
#define   LTC4283_ADC_SELECT_MASK(c)	BIT((c) % 8)
#define LTC4283_VPWR_MIN_TH		0x1d
#define LTC4283_VPWR_MAX_TH		0x1e
#define LTC4283_ADC_2_MIN_TH(c)		(0x21 + (c) * 2)
#define LTC4283_ADC_2_MAX_TH(c)		(0x22 + (c) * 2)
#define LTC4283_VPWR			0x44
#define LTC4283_VPWR_MIN		0x45
#define LTC4283_VPWR_MAX		0x46
#define LTC4283_POWER			0x47
#define LTC4283_POWER_MIN		0x48
#define LTC4283_POWER_MAX		0x49
/* get channels from ADC 2 */
#define LTC4283_ADC_2(c)		(0x4a + (c) * 3)
#define LTC4283_ADC_2_MIN(c)		(0x4b + (c) * 3)
#define LTC4283_ADC_2_MAX(c)		(0x4c + (c) * 3)
#define LTC4283_ADC2_FS_mV		2048

/* voltage channels */
enum {
	LTC4283_HWMON_VPWR,
	LTC4283_HWMON_ADI_1,
	LTC4283_HWMON_ADI_2,
	LTC4283_HWMON_ADI_3,
	LTC4283_HWMON_ADI_4,
	LTC4283_HWMON_ADIO_1,
	LTC4283_HWMON_ADIO_2,
	LTC4283_HWMON_ADIO_3,
	LTC4283_HWMON_ADIO_4,
	LTC4283_HWMON_DRAIN,
	LTC4283_HWMON_DRNS,
};

struct ltc4283_hwmon_in_regs {
	u32 in_input;
	u32 in_highest;
	u32 in_lowest;
	u32 in_max_alarm;
	u32 in_max_bit;
	u32 in_min_alarm;
	u32 in_min_bit;
};

struct ltc4283_hwmon {
	struct regmap *map;
	/* lock to protect concurrent device accesses and shared data */
	struct mutex lock;
	struct ltc4283_hwmon_in_regs in_regs[11];
};

static int ltc4283_hwmon_read_voltage_word(const struct ltc4283_hwmon *st,
					   u32 reg, u32 fs, long *val)
{
	__be16 in;
	int ret;

	ret = regmap_bulk_read(st->map, reg, &in, sizeof(in));
	if (ret)
		return ret;

	*val = DIV_ROUND_CLOSEST(be16_to_cpu(in) * fs, BIT(16));
	return 0;
}

static int ltc4283_hwmon_read_voltage_byte(const struct ltc4283_hwmon *st,
					   u32 reg, u32 fs, long *val)
{
	int ret;
	u32 in;

	ret = regmap_read(st->map, reg, &in);
	if (ret)
		return ret;

	*val = DIV_ROUND_CLOSEST(in * fs, BIT(8));
	return 0;
}

static int ltc428e_hwmon_read_alarm(const struct ltc4283_hwmon *st, u32 reg,
				    u32 mask, long *val)
{
	u32 alarm;
	int ret;

	ret = regmap_read(st->map, reg, &alarm);
	if (ret)
		return ret;

	*val = !!(alarm & mask);

	/* if not status/fault logs, clear the alarm after reading it */
	//if (reg != LTC4282_STATUS_LSB && reg != LTC4282_FAULT_LOG)
	//	return regmap_clear_bits(st->map, reg, mask);

	return 0;
}

static int ltc4283_hwmon_read_in_alarm(const struct ltc4283_hwmon *st,
				       u32 channel, bool max_alm, long *val)
{
	if (channel == LTC4283_HWMON_VPWR)
		return ltc428e_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_1,
						BIT(2 + max_alm), val);

	if (channel >= LTC4283_HWMON_ADI_1 && channel <= LTC4283_HWMON_ADI_4) {
		u32 bit = (channel - LTC4283_HWMON_ADI_1) * 2;
		/*
		 * Lower channels go to higher bits. We also want to go +1 down
		 * in the min_alarm case.
		 */
		return ltc428e_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_2,
						BIT(7 - bit - !max_alm), val);
	}

	if (channel >= LTC4283_HWMON_ADIO_1 && channel <= LTC4283_HWMON_ADIO_4) {
		u32 bit = (channel - LTC4283_HWMON_ADIO_1) * 2;

		return ltc428e_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_3,
						BIT(7 - bit - !max_alm), val);
	}

	if (channel == LTC4283_HWMON_DRNS)
		return ltc428e_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_4,
						BIT(6 + max_alm), val);

	return ltc428e_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_4,
					BIT(4 + max_alm), val);
}

static int ltc4283_hwmon_read_in_en(const struct ltc4283_hwmon *st, u32 channel,
				    long *val)
{
	unsigned int reg_val, bit;
	int ret;

	ret = regmap_read(st->map, LTC4283_ADC_SELECT(channel - 1), &reg_val);
	if (ret)
		return ret;

	bit = LTC4283_ADC_SELECT_MASK(channel - LTC4283_HWMON_ADI_1);
	if (channel > LTC4283_HWMON_DRAIN)
		/* account for two reserved fields after DRAIN */
		bit += 2;

	*val = !!(reg_val & bit);

	return 0;
}

static int ltc4283_hwmon_read_in(struct ltc4283_hwmon *st, u32 attr,
				 u32 channel, long *val)
{
	u32 reg;

	switch (attr) {
	case hwmon_in_input:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_read_voltage_word(st, LTC4283_VPWR,
							       2048, val);

		reg = LTC4283_ADC_2(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_read_voltage_word(st, reg, 2048, val);
	case hwmon_in_highest:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_read_voltage_word(st,
							       LTC4283_VPWR_MAX,
							       2048, val);

		reg = LTC4283_ADC_2_MAX(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_read_voltage_word(st, reg, 2048, val);
	case hwmon_in_lowest:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_read_voltage_word(st,
							       LTC4283_VPWR_MIN,
							       2048, val);

		reg = LTC4283_ADC_2_MIN(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_read_voltage_word(st, reg, 2048, val);
	case hwmon_in_max:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_read_voltage_byte(st,
							       LTC4283_VPWR_MAX_TH,
							       2048, val);

		reg = LTC4283_ADC_2_MAX_TH(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_read_voltage_byte(st, reg, 2048, val);
	case hwmon_in_min:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_read_voltage_byte(st,
							       LTC4283_VPWR_MIN_TH,
							       2048, val);

		reg = LTC4283_ADC_2_MIN_TH(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_read_voltage_byte(st, reg, 2048, val);
	case hwmon_in_max_alarm:
		return ltc4283_hwmon_read_in_alarm(st, channel, true, val);
	case hwmon_in_min_alarm:
		return ltc4283_hwmon_read_in_alarm(st, channel, true, val);
	case hwmon_in_enable:
		return ltc4283_hwmon_read_in_en(st, channel, val);
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int ltc4283_hwmon_read_curr(struct ltc4283_hwmon *st, u32 attr,
				   long *val)
{
	return 0;
}

static int ltc4283_hwmon_read_power(struct ltc4283_hwmon *st, u32 attr,
				    long *val)
{
	return 0;
}

static int ltc4283_hwmon_read(struct device *dev, enum hwmon_sensor_types type,
			      u32 attr, int channel, long *val)
{
	struct ltc4283_hwmon *st = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_in:
		return ltc4283_hwmon_read_in(st, attr, channel, val);
	case hwmon_curr:
		return ltc4283_hwmon_read_curr(st, attr, val);
	case hwmon_power:
		return ltc4283_hwmon_read_power(st, attr, val);
	case hwmon_energy:
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc4283_hwmon_write_power(struct ltc4283_hwmon *st, u32 attr,
				     long val)
{
	return 0;
}

static int ltc4283_hwmon_write_in(struct ltc4283_hwmon *st, u32 attr, long val,
				  int channel)
{
	return 0;
}

static int ltc4283_hwmon_write_curr(struct ltc4283_hwmon *st, u32 attr,
				    long val)
{
	return 0;
}

static int ltc4283_hwmon_energy_enable_set(struct ltc4283_hwmon *st, long val)
{
	return 0;
}

static int ltc4283_hwmon_write(struct device *dev, enum hwmon_sensor_types type,
			       u32 attr, int channel, long val)
{
	struct ltc4283_hwmon *st = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_power:
		return ltc4283_hwmon_write_power(st, attr, val);
	case hwmon_in:
		return ltc4283_hwmon_write_in(st, attr, val, channel);
	case hwmon_curr:
		return ltc4283_hwmon_write_curr(st, attr, val);
	case hwmon_energy:
		return ltc4283_hwmon_energy_enable_set(st, val);
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t ltc4283_hwmon_in_is_visible(const struct ltc4283_hwmon *st,
					   u32 attr)
{
	switch (attr) {
	case hwmon_in_input:
	case hwmon_in_highest:
	case hwmon_in_lowest:
	case hwmon_in_max_alarm:
	case hwmon_in_min_alarm:
	case hwmon_in_label:
	case hwmon_in_lcrit_alarm:
	case hwmon_in_crit_alarm:
	case hwmon_in_fault:
		return 0444;
	case hwmon_in_max:
	case hwmon_in_min:
	case hwmon_in_enable:
	case hwmon_in_reset_history:
		return 0644;
	default:
		return 0;
	}
}

static umode_t ltc4283_hwmon_curr_is_visible(u32 attr)
{
	switch (attr) {
	case hwmon_curr_input:
	case hwmon_curr_highest:
	case hwmon_curr_lowest:
	case hwmon_curr_max_alarm:
	case hwmon_curr_min_alarm:
	case hwmon_curr_crit_alarm:
	case hwmon_curr_label:
		return 0444;
	case hwmon_curr_max:
	case hwmon_curr_min:
	case hwmon_curr_reset_history:
		return 0644;
	default:
		return 0;
	}
}

static umode_t ltc4283_hwmon_power_is_visible(u32 attr)
{
	switch (attr) {
	case hwmon_power_input:
	case hwmon_power_input_highest:
	case hwmon_power_input_lowest:
	case hwmon_power_label:
	case hwmon_power_max_alarm:
	case hwmon_power_min_alarm:
		return 0444;
	case hwmon_power_max:
	case hwmon_power_min:
	case hwmon_power_reset_history:
		return 0644;
	default:
		return 0;
	}
}

static umode_t ltc4283_hwmon_is_visible(const void *data,
					enum hwmon_sensor_types type,
					u32 attr, int channel)
{
	switch (type) {
	case hwmon_in:
		return ltc4283_hwmon_in_is_visible(data, attr);
	case hwmon_curr:
		return ltc4283_hwmon_curr_is_visible(attr);
	case hwmon_power:
		return ltc4283_hwmon_power_is_visible(attr);
	case hwmon_energy:
		/* hwmon_energy_enable */
		return 0644;
	default:
		return 0;
	}
}

static const char * const ltc4283_hwmon_in_strs[] = {
	"VIN", "VPWR", "VADI1", "VADI2", "VADI3", "VADI4", "VADIO1", "VADIO2",
	"VADIO3", "VADIO4", "DRAIN", "DRNS"
};

static int ltc4283_hwmon_read_labels(struct device *dev,
				     enum hwmon_sensor_types type,
				     u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_in:
		*str = ltc4283_hwmon_in_strs[channel];
		return 0;
	case hwmon_curr:
		*str = "ISENSE";
		return 0;
	case hwmon_power:
		*str = "Power";
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static ssize_t ltc4283_hwmon_energy_show(struct device *dev,
					 struct device_attribute *da, char *buf)
{
	return sysfs_emit(buf, "%llu\n", 0ULL);
}

static int ltc4283_hwmon_setup(struct ltc4283_hwmon *st, struct device *dev)
{
	return 0;
}

static const struct hwmon_channel_info * const ltc4283_hwmon_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_LCRIT_ALARM | HWMON_I_CRIT_ALARM |
			   HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_MAX_ALARM | HWMON_I_ENABLE |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_FAULT | HWMON_I_ENABLE | HWMON_I_LABEL,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MAX | HWMON_I_MIN | HWMON_I_MIN_ALARM |
			   HWMON_I_RESET_HISTORY | HWMON_I_MAX_ALARM |
			   HWMON_I_ENABLE | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_MAX | HWMON_C_MIN | HWMON_C_MIN_ALARM |
			   HWMON_C_MAX_ALARM | HWMON_C_CRIT_ALARM |
			   HWMON_C_RESET_HISTORY | HWMON_C_LABEL),
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_INPUT_LOWEST |
			   HWMON_P_INPUT_HIGHEST | HWMON_P_MAX | HWMON_P_MIN |
			   HWMON_P_MAX_ALARM | HWMON_P_MIN_ALARM |
			   HWMON_P_RESET_HISTORY | HWMON_P_LABEL),
	HWMON_CHANNEL_INFO(energy,
			   HWMON_E_ENABLE | HWMON_E_LABEL),
	NULL
};

static const struct hwmon_ops ltc4283_hwmon_ops = {
	.read = ltc4283_hwmon_read,
	.write = ltc4283_hwmon_write,
	.is_visible = ltc4283_hwmon_is_visible,
	.read_string = ltc4283_hwmon_read_labels,
};

static const struct hwmon_chip_info ltc4283_hwmon_chip_info = {
	.ops = &ltc4283_hwmon_ops,
	.info = ltc4283_hwmon_info,
};

/* energy attributes are 6bytes wide so we need u64 */
static SENSOR_DEVICE_ATTR_RO(energy1_input, ltc4283_hwmon_energy, 0);

static struct attribute *ltc4283_hwmon_attrs[] = {
	&sensor_dev_attr_energy1_input.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(ltc4283_hwmon);

static void ltc4282_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static void ltc4283_debugfs_init(struct ltc4283_hwmon *st, struct device *dev,
				 const struct device *hwmon)
{
	const char *debugfs_name;
	struct dentry *dentry;
	int ret;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_name = devm_kasprintf(dev, GFP_KERNEL, "ltc4282-%s",
				      dev_name(hwmon));
	if (!debugfs_name)
		return;

	dentry = debugfs_create_dir(debugfs_name, NULL);
	if (IS_ERR(dentry))
		return;

	ret = devm_add_action_or_reset(dev, ltc4282_debugfs_remove, dentry);
	if (ret)
		return;
}

static int ltc4283_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev, *hwmon;
	struct ltc4283_hwmon *st;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->map = dev_get_regmap(dev->parent, NULL);
	if (!st->map)
		return dev_err_probe(dev, -ENODEV,
				     "Failed to get parent regmap\n");

	ret = ltc4283_hwmon_setup(st, dev);
	if (ret)
		return ret;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	hwmon = devm_hwmon_device_register_with_info(dev, "ltc4283-hwmon", st,
						     &ltc4283_hwmon_chip_info,
						     ltc4283_hwmon_groups);

	if (IS_ERR(hwmon))
		return PTR_ERR(hwmon);

	ltc4283_debugfs_init(st, dev, hwmon);

	return 0;
}

static const struct of_device_id ltc4283_of_match[] = {
	{ .compatible = "adi,ltc4283-hwmon" },
	{ }
};

static struct platform_driver ltc4283_hwmon_driver = {
	.driver	= {
		.name = "ltc4283-hwmon",
		.of_match_table = ltc4283_of_match,
	},
	.probe = ltc4283_hwmon_probe,
};
module_platform_driver(ltc4283_hwmon_driver);

MODULE_AUTHOR("Nuno Sá <nuno.sa@analog.com>");
MODULE_DESCRIPTION("HWMON LTC4283 How Swap Controller driver");
MODULE_LICENSE("GPL");
