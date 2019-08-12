// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LTC2947 high precision power and energy monitor
 *
 * Copyright 2019 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>

#include "ltc2947.h"

/* register's */
#define LTC2947_REG_PAGE_CTRL		0xFF
#define LTC2947_REG_CTRL		0xF0
#define LTC2947_REG_TBCTL		0xE9
#define LTC2947_CONT_MODE_MASK		BIT(3)
#define LTC2947_CONT_MODE(x)		FIELD_PREP(LTC2947_CONT_MODE_MASK, x)
#define LTC2947_PRE_MASK		GENMASK(2, 0)
#define LTC2947_PRE(x)			FIELD_PREP(LTC2947_PRE_MASK, x)
#define LTC2947_DIV_MASK		GENMASK(7, 3)
#define LTC2947_DIV(x)			FIELD_PREP(LTC2947_DIV_MASK, x)
#define LTC2947_SHUTDOWN_MASK		BIT(0)
#define LTC2947_REG_ACCUM_POL		0xE1
#define LTC2947_ACCUM_POL_1_MASK	GENMASK(1, 0)
#define LTC2947_ACCUM_POL_1(x)		FIELD_PREP(LTC2947_ACCUM_POL_1_MASK, x)
#define LTC2947_ACCUM_POL_2_MASK	GENMASK(3, 2)
#define LTC2947_ACCUM_POL_2(x)		FIELD_PREP(LTC2947_ACCUM_POL_2_MASK, x)
#define LTC2947_REG_ACCUM_DEADBAND	0xE4
/* 200Khz */
#define LTC2947_CLK_MIN			200000
/* 25Mhz */
#define LTC2947_CLK_MAX			25000000
#define PAGE0				0
#define PAGE1				1
/* Voltage registers */
#define LTC2947_REG_VOLTAGE		0xA0
#define LTC2947_REG_VOLTAGE_MAX		0x50
#define LTC2947_REG_VOLTAGE_MIN		0x52
#define LTC2947_REG_VOLTAGE_THRE_H	0x90
#define LTC2947_REG_VOLTAGE_THRE_L	0x92
#define LTC2947_REG_DVCC		0xA4
#define LTC2947_REG_DVCC_MAX		0x58
#define LTC2947_REG_DVCC_MIN		0x5A
#define LTC2947_REG_DVCC_THRE_H		0x98
#define LTC2947_REG_DVCC_THRE_L		0x9A
#define LTC2947_VOLTAGE_GEN_CHAN	0
#define LTC2947_VOLTAGE_DVCC_CHAN	1
/* Current registers */
#define LTC2947_REG_CURRENT		0x90
#define LTC2947_REG_CURRENT_MAX		0x40
#define LTC2947_REG_CURRENT_MIN		0x42
#define LTC2947_REG_CURRENT_THRE_H	0x80
#define LTC2947_REG_CURRENT_THRE_L	0x82
/* Power registers */
#define LTC2947_REG_POWER		0x93
#define LTC2947_REG_POWER_MAX		0x44
#define LTC2947_REG_POWER_MIN		0x46
#define LTC2947_REG_POWER_THRE_H	0x84
#define LTC2947_REG_POWER_THRE_L	0x86
/* Temperature registers */
#define LTC2947_REG_TEMP		0xA2
#define LTC2947_REG_TEMP_MAX		0x54
#define LTC2947_REG_TEMP_MIN		0x56
#define LTC2947_REG_TEMP_THRE_H		0x94
#define LTC2947_REG_TEMP_THRE_L		0x96
/* Energy registers */
#define LTC2947_REG_ENERGY1		0x06
#define LTC2947_REG_ENERGY1_THRE_H	0x10
#define LTC2947_REG_ENERGY1_THRE_L	0x16
#define LTC2947_REG_ENERGY2		0x16
#define LTC2947_REG_ENERGY2_THRE_H	0x30
#define LTC2947_REG_ENERGY2_THRE_L	0x36
#define ENERGY_MIN			0xFFFF800000000000LL
#define ENERGY_MAX			0x00007FFFFFFFFFFFLL
/* Status/Alarm/Overflow registers */
#define LTC2947_REG_STATUS		0x80
#define LTC2947_REG_STATVT		0x81
#define LTC2947_REG_STATIP		0x82
#define LTC2947_REG_STATC		0x83
#define LTC2947_REG_STATE		0x84
#define LTC2947_REG_STATCEOF		0x85
#define LTC2947_REG_STATVDVCC		0x87

#define LTC2947_ALERTS_SIZE	(LTC2947_REG_STATVDVCC - LTC2947_REG_STATUS)
#define LTC2947_UPDATE_VAL_MASK		BIT(4)
#define LTC2947_MAX_VOLTAGE_MASK	BIT(0)
#define LTC2947_MIN_VOLTAGE_MASK	BIT(1)
#define LTC2947_MAX_CURRENT_MASK	BIT(0)
#define LTC2947_MIN_CURRENT_MASK	BIT(1)
#define LTC2947_MAX_POWER_MASK		BIT(2)
#define LTC2947_MIN_POWER_MASK		BIT(3)
#define LTC2947_MAX_TEMP_MASK		BIT(2)
#define LTC2947_MIN_TEMP_MASK		BIT(3)
#define LTC2947_SINGLE_SHOT_MASK	BIT(2)
#define LTC2947_MAX_ENERGY1_MASK	BIT(0)
#define LTC2947_MIN_ENERGY1_MASK	BIT(1)
#define LTC2947_MAX_ENERGY2_MASK	BIT(2)
#define LTC2947_MIN_ENERGY2_MASK	BIT(3)
#define LTC2947_MIN_ENERGY1_O_MASK	BIT(4)
#define LTC2947_MIN_ENERGY2_O_MASK	BIT(5)
#define LTC2947_ADCERR_MASK		BIT(5)
/*
 * For accumulated values there's a fault if the ADC conversions are invalid
 * (ADCERR) or if there is an overflow of the internal timebase register
 * (which indicates invalid TBCTL configuration).
 */
#define LTC2947_ENERGY_FAULT_MASK	GENMASK(6, 5)

struct ltc2947_data {
	struct regmap *map;
	struct device *dev;
	/*
	 * The mutex is needed because the device has 2 memory pages. When
	 * reading/writing the correct page needs to be set so that, the
	 * complete sequence select_page->read/write needs to be protected.
	 */
	struct mutex lock;
	u32 lsb_energy;
	bool reset;
};
/* used for raw sysfs entries */
enum {
	LTC2947_POWER_INPUT,
	LTC2947_POWER_THRE_L,
	LTC2947_POWER_THRE_H,
	LTC2947_POWER_HIGHEST,
	LTC2947_POWER_LOWEST,
	LTC2947_POWER_MIN_ALARM,
	LTC2947_ENERGY1_INPUT,
	LTC2947_ENERGY1_THRE_H,
	LTC2947_ENERGY1_THRE_L,
	LTC2947_ENERGY1_MAX_ALARM,
	LTC2947_ENERGY1_MIN_ALARM,
	LTC2947_ENERGY2_INPUT,
	LTC2947_ENERGY2_THRE_H,
	LTC2947_ENERGY2_THRE_L,
	LTC2947_ENERGY2_MAX_ALARM,
	LTC2947_ENERGY2_MIN_ALARM,
	LTC2947_ENERGY1_OVERF_ALARM,
	LTC2947_ENERGY2_OVERF_ALARM,
	LTC2947_FAULT = LTC2947_ADCERR_MASK,
	LTC2947_ENERGY_FAULT = LTC2947_ENERGY_FAULT_MASK,
};

static int __ltc2947_val_read16(const struct ltc2947_data *st, const u8 reg,
				u64 *val)
{
	__be16 __val = 0;
	int ret;

	ret = regmap_bulk_read(st->map, reg, &__val, 2);
	if (ret)
		return ret;

	*val = be16_to_cpu(__val);

	return 0;
}

static int __ltc2947_val_read24(const struct ltc2947_data *st, const u8 reg,
				u64 *val)
{
	__be32 __val = 0;
	int ret;

	ret = regmap_bulk_read(st->map, reg, &__val, 3);
	if (ret)
		return ret;

	*val = be32_to_cpu(__val) >> 8;

	return 0;
}

static int __ltc2947_val_read64(const struct ltc2947_data *st, const u8 reg,
				u64 *val)
{
	__be64 __val = 0;
	int ret;

	ret = regmap_bulk_read(st->map, reg, &__val, 6);
	if (ret)
		return ret;

	*val = be64_to_cpu(__val) >> 16;

	return 0;
}

static int ltc2947_val_read(struct ltc2947_data *st, const u8 reg,
			    const u8 page, const size_t size, s64 *val)
{
	int ret;
	u64 __val = 0;

	mutex_lock(&st->lock);

	if (st->reset) {
		mutex_unlock(&st->lock);
		return -EPERM;
	}

	ret = regmap_write(st->map, LTC2947_REG_PAGE_CTRL, page);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	dev_dbg(st->dev, "Read val, reg:%02X, p:%d sz:%02X\n", reg, page,
								size);
	switch (size) {
	case 2:
		ret = __ltc2947_val_read16(st, reg, &__val);
		break;
	case 3:
		ret = __ltc2947_val_read24(st, reg, &__val);
		break;
	case 6:
		ret = __ltc2947_val_read64(st, reg, &__val);
		break;
	default:
		dev_err(st->dev, "Invalid size(%d) to read", size);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	if (ret)
		return ret;

	*val = sign_extend64(__val, (8 * size) - 1);

	dev_dbg(st->dev, "Got s:%lld, u:%016llX\n", *val, __val);

	return 0;
}

static int __ltc2947_val_write64(const struct ltc2947_data *st, const u8 reg,
				 const u64 val)
{
	__be64 __val;

	__val = cpu_to_be64(val << 16);
	return regmap_bulk_write(st->map, reg, &__val, 6);
}

static int __ltc2947_val_write16(const struct ltc2947_data *st, const u8 reg,
				 const u16 val)
{
	__be16 __val;

	__val = cpu_to_be16(val);
	return regmap_bulk_write(st->map, reg, &__val, 2);
}

static int ltc2947_val_write(struct ltc2947_data *st, const u8 reg,
			     const u8 page, const size_t size, const u64 val)
{
	int ret;

	mutex_lock(&st->lock);
	/*
	 * Do not allow channel readings if device is in sleep state.
	 * A read/write on the spi/i2c bus would bring the device prematurely
	 * out of sleep.
	 */
	if (st->reset) {
		mutex_unlock(&st->lock);
		return -EPERM;
	}
	/* set device on correct page */
	ret = regmap_write(st->map, LTC2947_REG_PAGE_CTRL, page);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	dev_dbg(st->dev, "Write val, r:%02X, p:%d, sz:%02X, val:%016llX\n",
		reg, page, size, val);

	switch (size) {
	case 2:
		ret = __ltc2947_val_write16(st, reg, val);
		break;
	case 6:
		ret = __ltc2947_val_write64(st, reg, val);
		break;
	default:
		dev_err(st->dev, "Invalid size(%d) to write", size);
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static int ltc2947_reset_history(struct ltc2947_data *st, const u8 reg_h,
				 const u8 reg_l)
{
	int ret;
	/*
	 * let's reset the tracking register's. Tracking register's have all
	 * 2 bytes size
	 */
	ret = ltc2947_val_write(st, reg_h, PAGE0, 2, 0x8000U);
	if (ret)
		return ret;

	return ltc2947_val_write(st, reg_l, PAGE0, 2, 0x7FFFU);
}

static int ltc2947_alarm_read(struct ltc2947_data *st, const u8 reg,
			      const u32 mask, long *val)
{
	u8 offset = reg - LTC2947_REG_STATUS;
	/* +1 to include status reg */
	char alarms[LTC2947_ALERTS_SIZE + 1];
	int ret = 0;

	memset(alarms, 0, sizeof(alarms));

	mutex_lock(&st->lock);

	if (st->reset) {
		ret = -EPERM;
		goto unlock;
	}

	ret = regmap_write(st->map, LTC2947_REG_PAGE_CTRL, PAGE0);
	if (ret)
		goto unlock;

	dev_dbg(st->dev, "Read alarm, reg:%02X, mask:%02X\n", reg, mask);
	/*
	 * As stated in the datasheet, when Threshold and Overflow registers
	 * are used, the status and all alert registers must be read in one
	 * multi-byte transaction.
	 */
	ret = regmap_bulk_read(st->map, LTC2947_REG_STATUS, alarms,
			       sizeof(alarms));
	if (ret)
		goto unlock;

	/* get the alarm */
	*val = !!(alarms[offset] & mask);
unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static ssize_t ltc2947_set_value(struct device *dev,
				 struct device_attribute *da,
				 const char *buf, size_t count)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int ret;
	u8 reg, page = PAGE1;
	s64 val = 0;

	ret = kstrtoll(buf, 10, &val);
	if (ret) {
		dev_err(st->dev, "Failed to convert the value\n");
		return ret;
	}

	switch (attr->index) {
	case LTC2947_POWER_THRE_H:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);
		ret = ltc2947_val_write(st, LTC2947_REG_POWER_THRE_H, PAGE1, 2,
					div_s64(val, 200000));
		return ret ? ret : count;
	case LTC2947_POWER_THRE_L:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);
		ret = ltc2947_val_write(st, LTC2947_REG_POWER_THRE_L, PAGE1, 2,
					div_s64(val, 200000));
		return ret ? ret : count;
	case LTC2947_ENERGY1_THRE_H:
		reg = LTC2947_REG_ENERGY1_THRE_H;
		break;
	case LTC2947_ENERGY1_THRE_L:
		reg = LTC2947_REG_ENERGY1_THRE_L;
		break;
	case LTC2947_ENERGY2_THRE_H:
		reg = LTC2947_REG_ENERGY2_THRE_H;
		break;
	case LTC2947_ENERGY2_THRE_L:
		reg = LTC2947_REG_ENERGY2_THRE_L;
		break;
	case LTC2947_ENERGY1_INPUT:
		reg = LTC2947_REG_ENERGY1;
		page = PAGE0;
		break;
	case LTC2947_ENERGY2_INPUT:
		reg = LTC2947_REG_ENERGY2;
		page = PAGE0;
		break;
	default:
		return -ENOTSUPP;
	}

	val = clamp_val(val, ENERGY_MIN, ENERGY_MAX);
	/* we are losing the fractional part here... */
	val = div_s64(val * 1000, st->lsb_energy);

	ret = ltc2947_val_write(st, reg, page, 6, val);

	return ret ? ret : count;
}

static ssize_t ltc2947_show_value(struct device *dev,
				  struct device_attribute *da, char *buf)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	int ret;
	s64 val = 0;

	switch (attr->index) {
	case LTC2947_POWER_INPUT:
		ret = ltc2947_val_read(st, LTC2947_REG_POWER, PAGE0, 3, &val);
		return ret ? ret : sprintf(buf, "%lld\n", val * 50000);
	case LTC2947_POWER_THRE_H:
		ret = ltc2947_val_read(st, LTC2947_REG_POWER_THRE_H, PAGE1, 2,
				       &val);
		return ret ? ret : sprintf(buf, "%lld\n", val * 200000);
	case LTC2947_POWER_THRE_L:
		ret = ltc2947_val_read(st, LTC2947_REG_POWER_THRE_L, PAGE1, 2,
				       &val);
		return ret ? ret : sprintf(buf, "%lld\n", val * 200000);
	case LTC2947_POWER_HIGHEST:
		ret = ltc2947_val_read(st, LTC2947_REG_POWER_MAX, PAGE0, 2,
				       &val);
		return ret ? ret : sprintf(buf, "%lld\n", val * 200000);
	case LTC2947_POWER_LOWEST:
		ret = ltc2947_val_read(st, LTC2947_REG_POWER_MIN, PAGE0, 2,
				       &val);
		return ret ? ret : sprintf(buf, "%lld\n", val * 200000);
	case LTC2947_ENERGY1_THRE_H:
		ret = ltc2947_val_read(st, LTC2947_REG_ENERGY1_THRE_H, PAGE1, 6,
				       &val);
		break;
	case LTC2947_ENERGY1_THRE_L:
		ret = ltc2947_val_read(st, LTC2947_REG_ENERGY1_THRE_L, PAGE1, 6,
				       &val);
		break;
	case LTC2947_ENERGY2_THRE_H:
		ret = ltc2947_val_read(st, LTC2947_REG_ENERGY2_THRE_H, PAGE1, 6,
				       &val);
		break;
	case LTC2947_ENERGY2_THRE_L:
		ret = ltc2947_val_read(st, LTC2947_REG_ENERGY2_THRE_L, PAGE1, 6,
				       &val);
		break;
	case LTC2947_ENERGY1_INPUT:
		ret = ltc2947_val_read(st, LTC2947_REG_ENERGY1, PAGE0, 6, &val);
		break;
	case LTC2947_ENERGY2_INPUT:
		ret = ltc2947_val_read(st, LTC2947_REG_ENERGY2, PAGE0, 6, &val);
		break;
	default:
		return -EINVAL;
	}

	/* if we got here, must be an energy reading... */
	if (ret)
		return ret;

	/* value in microJoule. st->lsb_energy was multiplied by 10E9 */
	val = div_s64(val * st->lsb_energy, 1000);

	return sprintf(buf, "%lld\n", val);
}

static ssize_t ltc2947_show_alert(struct device *dev,
				  struct device_attribute *da, char *buf)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	long alert;
	int ret = 0;

	switch (attr->index) {
	case LTC2947_POWER_MIN_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATIP,
					 LTC2947_MIN_POWER_MASK, &alert);
		break;
	case LTC2947_ENERGY1_MAX_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATE,
					 LTC2947_MAX_ENERGY1_MASK, &alert);
		break;
	case LTC2947_ENERGY1_MIN_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATE,
					 LTC2947_MIN_ENERGY1_MASK, &alert);
		break;
	case LTC2947_ENERGY2_MAX_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATE,
					 LTC2947_MAX_ENERGY2_MASK, &alert);
		break;
	case LTC2947_ENERGY2_MIN_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATE,
					 LTC2947_MIN_ENERGY2_MASK, &alert);
		break;
	case LTC2947_ENERGY1_OVERF_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATCEOF,
					 LTC2947_MIN_ENERGY1_O_MASK, &alert);
		break;
	case LTC2947_ENERGY2_OVERF_ALARM:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATCEOF,
					 LTC2947_MIN_ENERGY2_O_MASK, &alert);
		break;
	case LTC2947_FAULT:
	case LTC2947_ENERGY_FAULT:
		ret = ltc2947_alarm_read(st, LTC2947_REG_STATUS, attr->index,
					 &alert);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : sprintf(buf, "%li\n", alert);
}

static int ltc2947_read_temp(struct device *dev, const u32 attr, long *val)
{
	int ret;
	struct ltc2947_data *st = dev_get_drvdata(dev);
	s64 __val = 0;

	switch (attr) {
	case hwmon_temp_input:
		ret = ltc2947_val_read(st, LTC2947_REG_TEMP, PAGE0, 2, &__val);
		break;
	case hwmon_temp_highest:
		ret = ltc2947_val_read(st, LTC2947_REG_TEMP_MAX, PAGE0, 2,
				       &__val);
		break;
	case hwmon_temp_lowest:
		ret = ltc2947_val_read(st, LTC2947_REG_TEMP_MIN, PAGE0, 2,
				       &__val);
		break;
	case hwmon_temp_max_alarm:
		return ltc2947_alarm_read(st, LTC2947_REG_STATVT,
					  LTC2947_MAX_TEMP_MASK, val);
	case hwmon_temp_min_alarm:
		return	ltc2947_alarm_read(st, LTC2947_REG_STATVT,
					   LTC2947_MIN_TEMP_MASK, val);
	case hwmon_temp_max:
		ret = ltc2947_val_read(st, LTC2947_REG_TEMP_THRE_H, PAGE1, 2,
				       &__val);
		break;
	case hwmon_temp_min:
		ret = ltc2947_val_read(st, LTC2947_REG_TEMP_THRE_L, PAGE1, 2,
				       &__val);
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	/* in milidegrees celcius, temp is given by: */
	*val = (__val * 204) + 550;

	return 0;
}

static int ltc2947_read_power(struct device *dev, const u32 attr, long *val)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_power_max_alarm:
		return ltc2947_alarm_read(st, LTC2947_REG_STATIP,
					  LTC2947_MAX_POWER_MASK, val);
	default:
		return -EOPNOTSUPP;
	}

	return 0;
}

static int ltc2947_read_curr(struct device *dev, const u32 attr, long *val)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	int ret;
	u8 lsb = 12; /* in mA */
	s64 __val = 0;

	switch (attr) {
	case hwmon_curr_input:
		ret = ltc2947_val_read(st, LTC2947_REG_CURRENT, PAGE0, 3,
				       &__val);
		lsb = 3;
		break;
	case hwmon_curr_highest:
		ret = ltc2947_val_read(st, LTC2947_REG_CURRENT_MAX, PAGE0, 2,
				       &__val);
		break;
	case hwmon_curr_lowest:
		ret = ltc2947_val_read(st, LTC2947_REG_CURRENT_MIN, PAGE0, 2,
				       &__val);
		break;
	case hwmon_curr_max_alarm:
		return ltc2947_alarm_read(st, LTC2947_REG_STATIP,
					  LTC2947_MAX_CURRENT_MASK, val);
	case hwmon_curr_min_alarm:
		return ltc2947_alarm_read(st, LTC2947_REG_STATIP,
					  LTC2947_MIN_CURRENT_MASK, val);
	case hwmon_curr_max:
		ret = ltc2947_val_read(st, LTC2947_REG_CURRENT_THRE_H, PAGE1, 2,
				       &__val);
		break;
	case hwmon_curr_min:
		ret = ltc2947_val_read(st, LTC2947_REG_CURRENT_THRE_L, PAGE1, 2,
				       &__val);
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	*val = __val * lsb;

	return 0;
}

static int ltc2947_read_in(struct device *dev, const u32 attr, long *val,
			   const int channel)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	int ret;
	u8 lsb = 2; /* in mV */
	s64 __val = 0;

	if (channel < 0 || channel > LTC2947_VOLTAGE_DVCC_CHAN) {
		dev_err(st->dev, "Invalid chan%d for voltage", channel);
		return -EINVAL;
	}

	switch (attr) {
	case hwmon_in_input:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN) {
			ret = ltc2947_val_read(st, LTC2947_REG_DVCC, PAGE0, 2,
					       &__val);
			lsb = 145;
		} else {
			ret = ltc2947_val_read(st, LTC2947_REG_VOLTAGE, PAGE0,
					       2, &__val);
		}
		break;
	case hwmon_in_highest:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN) {
			ret = ltc2947_val_read(st, LTC2947_REG_DVCC_MAX, PAGE0,
					       2, &__val);
			lsb = 145;
		} else {
			ret = ltc2947_val_read(st, LTC2947_REG_VOLTAGE_MAX,
					       PAGE0, 2, &__val);
		}
		break;
	case hwmon_in_lowest:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN) {
			ret = ltc2947_val_read(st, LTC2947_REG_DVCC_MIN, PAGE0,
					       2, &__val);
			lsb = 145;
		} else {
			ret = ltc2947_val_read(st, LTC2947_REG_VOLTAGE_MIN,
					       PAGE0, 2, &__val);
		}
		break;
	case hwmon_in_max_alarm:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN)
			return ltc2947_alarm_read(st, LTC2947_REG_STATVDVCC,
						  LTC2947_MAX_VOLTAGE_MASK,
						  val);
		else
			return ltc2947_alarm_read(st, LTC2947_REG_STATVT,
						  LTC2947_MAX_VOLTAGE_MASK,
						  val);
	case hwmon_in_min_alarm:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN)
			return ltc2947_alarm_read(st, LTC2947_REG_STATVDVCC,
						  LTC2947_MIN_VOLTAGE_MASK,
						  val);
		else
			return ltc2947_alarm_read(st, LTC2947_REG_STATVT,
						  LTC2947_MIN_VOLTAGE_MASK,
						  val);
	case hwmon_in_max:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN) {
			ret = ltc2947_val_read(st, LTC2947_REG_DVCC_THRE_H,
					       PAGE1, 2, &__val);
			lsb = 145;
		} else {
			ret = ltc2947_val_read(st, LTC2947_REG_VOLTAGE_THRE_H,
					       PAGE1, 2, &__val);
		}
		break;
	case hwmon_in_min:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN) {
			ret = ltc2947_val_read(st, LTC2947_REG_DVCC_THRE_L,
					       PAGE1, 2, &__val);
			lsb = 145;
		} else {
			ret = ltc2947_val_read(st, LTC2947_REG_VOLTAGE_THRE_L,
					       PAGE1, 2, &__val);
		}
		break;
	default:
		return -EOPNOTSUPP;
	}

	if (ret)
		return ret;

	*val = __val * lsb;

	return 0;
}

static int ltc2947_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	switch (type) {
	case hwmon_in:
		return ltc2947_read_in(dev, attr, val, channel);
	case hwmon_curr:
		return ltc2947_read_curr(dev, attr, val);
	case hwmon_power:
		return ltc2947_read_power(dev, attr, val);
	case hwmon_temp:
		return ltc2947_read_temp(dev, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc2947_write_temp(struct device *dev, const u32 attr,
			      long val)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_temp_reset_history:
		if (val != 1)
			return -EINVAL;
		return ltc2947_reset_history(st, LTC2947_REG_TEMP_MAX,
					     LTC2947_REG_TEMP_MIN);
	case hwmon_temp_max:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);
		return ltc2947_val_write(st, LTC2947_REG_TEMP_THRE_H, PAGE1, 2,
					DIV_ROUND_CLOSEST(val - 550, 204));
	case hwmon_temp_min:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);
		return ltc2947_val_write(st, LTC2947_REG_TEMP_THRE_L, PAGE1, 2,
					DIV_ROUND_CLOSEST(val - 550, 204));
	default:
		return -ENOTSUPP;
	}
}

static int ltc2947_write_power(struct device *dev, const u32 attr,
			       long val)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_power_reset_history:
		if (val != 1)
			return -EINVAL;
		return ltc2947_reset_history(st, LTC2947_REG_POWER_MAX,
					     LTC2947_REG_POWER_MIN);
	default:
		return -ENOTSUPP;
	}
}

static int ltc2947_write_curr(struct device *dev, const u32 attr,
			      long val)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);

	switch (attr) {
	case hwmon_curr_reset_history:
		if (val != 1)
			return -EINVAL;
		return ltc2947_reset_history(st, LTC2947_REG_CURRENT_MAX,
					     LTC2947_REG_CURRENT_MIN);
	case hwmon_curr_max:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);
		return ltc2947_val_write(st, LTC2947_REG_CURRENT_THRE_H, PAGE1,
					 2, DIV_ROUND_CLOSEST(val, 12));
	case hwmon_curr_min:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);
		return ltc2947_val_write(st, LTC2947_REG_CURRENT_THRE_L, PAGE1,
					 2, DIV_ROUND_CLOSEST(val, 12));
	default:
		return -ENOTSUPP;
	}
}

static int ltc2947_write_in(struct device *dev, const u32 attr, long val,
			    const int channel)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);

	if (channel > LTC2947_VOLTAGE_DVCC_CHAN) {
		dev_err(st->dev, "Invalid chan%d for voltage", channel);
		return -EINVAL;
	}

	switch (attr) {
	case hwmon_in_reset_history:
		if (val != 1)
			return -EINVAL;

		if (channel == LTC2947_VOLTAGE_DVCC_CHAN)
			return ltc2947_reset_history(st, LTC2947_REG_DVCC_MAX,
						     LTC2947_REG_DVCC_MIN);
		else
			return ltc2947_reset_history(st,
						     LTC2947_REG_VOLTAGE_MAX,
						     LTC2947_REG_VOLTAGE_MIN);
	case hwmon_in_max:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);

		if (channel == LTC2947_VOLTAGE_DVCC_CHAN)
			return ltc2947_val_write(st, LTC2947_REG_DVCC_THRE_H,
						 PAGE1, 2,
						 DIV_ROUND_CLOSEST(val, 145));
		else
			return ltc2947_val_write(st, LTC2947_REG_VOLTAGE_THRE_H,
						 PAGE1, 2,
						 DIV_ROUND_CLOSEST(val, 2));
	case hwmon_in_min:
		val = clamp_val(val, SHRT_MIN, SHRT_MAX);

		if (channel == LTC2947_VOLTAGE_DVCC_CHAN)
			return ltc2947_val_write(st, LTC2947_REG_DVCC_THRE_L,
						 PAGE1, 2,
						 DIV_ROUND_CLOSEST(val, 145));
		else
			return ltc2947_val_write(st, LTC2947_REG_VOLTAGE_THRE_L,
						 PAGE1, 2,
						 DIV_ROUND_CLOSEST(val, 2));
	default:
		return -ENOTSUPP;
	}
}

static int ltc2947_write(struct device *dev,
			 enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	switch (type) {
	case hwmon_in:
		return ltc2947_write_in(dev, attr, val, channel);
	case hwmon_curr:
		return ltc2947_write_curr(dev, attr, val);
	case hwmon_power:
		return ltc2947_write_power(dev, attr, val);
	case hwmon_temp:
		return ltc2947_write_temp(dev, attr, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc2947_read_labels(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_in:
		if (channel == LTC2947_VOLTAGE_DVCC_CHAN)
			*str = "DVCC";
		else
			*str = "VP-VM";
		return 0;
	case hwmon_curr:
		*str = "IP-IM";
		return 0;
	case hwmon_temp:
		*str = "Ambient";
		return 0;
	case hwmon_power:
		*str = "Power";
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc2947_in_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_in_input:
	case hwmon_in_highest:
	case hwmon_in_lowest:
	case hwmon_in_max_alarm:
	case hwmon_in_min_alarm:
	case hwmon_in_label:
		return 0444;
	case hwmon_in_reset_history:
		return 0200;
	case hwmon_in_max:
	case hwmon_in_min:
		return 0644;
	default:
		return 0;
	}
}

static int ltc2947_curr_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_curr_input:
	case hwmon_curr_highest:
	case hwmon_curr_lowest:
	case hwmon_curr_max_alarm:
	case hwmon_curr_min_alarm:
	case hwmon_curr_label:
		return 0444;
	case hwmon_curr_reset_history:
		return 0200;
	case hwmon_curr_max:
	case hwmon_curr_min:
		return 0644;
	default:
		return 0;
	}
}

static int ltc2947_power_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_power_label:
	case hwmon_power_max_alarm:
		return 0444;
	case hwmon_power_reset_history:
		return 0200;
	default:
		return 0;
	}
}

static int ltc2947_temp_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_temp_input:
	case hwmon_temp_highest:
	case hwmon_temp_lowest:
	case hwmon_temp_max_alarm:
	case hwmon_temp_min_alarm:
	case hwmon_temp_label:
		return 0444;
	case hwmon_temp_reset_history:
		return 0200;
	case hwmon_temp_max:
	case hwmon_temp_min:
		return 0644;
	default:
		return 0;
	}
}

static umode_t ltc2947_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_in:
		return ltc2947_in_is_visible(attr);
	case hwmon_curr:
		return ltc2947_curr_is_visible(attr);
	case hwmon_power:
		return ltc2947_power_is_visible(attr);
	case hwmon_temp:
		return ltc2947_temp_is_visible(attr);
	default:
		return 0;
	}
}

static const u32 ltc2947_temp_config[] = {
	HWMON_T_INPUT | HWMON_T_LOWEST | HWMON_T_HIGHEST | HWMON_T_MAX |
	HWMON_T_MIN | HWMON_T_RESET_HISTORY | HWMON_T_MIN_ALARM |
	HWMON_T_MAX_ALARM | HWMON_T_LABEL,
	0
};

static const struct hwmon_channel_info ltc2947_temp = {
	.type = hwmon_temp,
	.config = ltc2947_temp_config,
};

/* all the other properties require u64 */
static const u32 ltc2947_power_config[] = {
	HWMON_P_RESET_HISTORY | HWMON_P_MAX_ALARM | HWMON_P_LABEL,
	0
};

static const struct hwmon_channel_info ltc2947_power = {
	.type = hwmon_power,
	.config = ltc2947_power_config,
};

static const u32 ltc2947_curr_config[] = {
	HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST | HWMON_C_MAX |
	HWMON_C_MIN | HWMON_C_RESET_HISTORY | HWMON_C_MIN_ALARM |
	HWMON_C_MAX_ALARM | HWMON_C_LABEL,
	0
};

static const struct hwmon_channel_info ltc2947_curr = {
	.type = hwmon_curr,
	.config = ltc2947_curr_config,
};

static const u32 ltc2947_in_config[] = {
	HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST | HWMON_I_MAX |
	HWMON_I_MIN | HWMON_I_RESET_HISTORY | HWMON_I_MIN_ALARM |
	HWMON_I_MAX_ALARM | HWMON_I_LABEL,
	HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST | HWMON_I_LABEL |
	HWMON_I_MAX | HWMON_I_MIN | HWMON_I_RESET_HISTORY | HWMON_I_MIN_ALARM |
	HWMON_I_MAX_ALARM,
	0
};

static const struct hwmon_channel_info ltc2947_in = {
	.type = hwmon_in,
	.config = ltc2947_in_config,
};

static const struct hwmon_channel_info *ltc2947_info[] = {
	&ltc2947_in,
	&ltc2947_curr,
	&ltc2947_power,
	&ltc2947_temp,
	NULL
};

static const struct hwmon_ops ltc2947_hwmon_ops = {
	.is_visible = ltc2947_is_visible,
	.read = ltc2947_read,
	.write = ltc2947_write,
	.read_string = ltc2947_read_labels,
};

static const struct hwmon_chip_info ltc2947_chip_info = {
	.ops = &ltc2947_hwmon_ops,
	.info = ltc2947_info,
};

static SENSOR_DEVICE_ATTR(test1, 0644, ltc2947_show_value,
			  ltc2947_set_value, 0xFF);

/* power attributes */
static SENSOR_DEVICE_ATTR(power1_input, 0444, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_POWER_INPUT);
static SENSOR_DEVICE_ATTR(power1_max, 0644, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_POWER_THRE_H);
static SENSOR_DEVICE_ATTR(power1_min, 0644, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_POWER_THRE_L);
static SENSOR_DEVICE_ATTR(power1_input_highest, 0444, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_POWER_HIGHEST);
static SENSOR_DEVICE_ATTR(power1_input_lowest, 0444, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_POWER_LOWEST);
static SENSOR_DEVICE_ATTR(power1_min_alarm, 0444, ltc2947_show_alert, NULL,
			  LTC2947_POWER_MIN_ALARM);
/* energy attributes */
static SENSOR_DEVICE_ATTR(energy1_input, 0444, ltc2947_show_value, NULL,
			  LTC2947_ENERGY1_INPUT);
static SENSOR_DEVICE_ATTR(energy1_max, 0644, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_ENERGY1_THRE_H);
static SENSOR_DEVICE_ATTR(energy1_min, 0644, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_ENERGY1_THRE_L);
static SENSOR_DEVICE_ATTR(energy1_max_alarm, 0444, ltc2947_show_alert, NULL,
			  LTC2947_ENERGY1_MAX_ALARM);
static SENSOR_DEVICE_ATTR(energy1_min_alarm, 0444, ltc2947_show_alert, NULL,
			  LTC2947_ENERGY1_MIN_ALARM);
static SENSOR_DEVICE_ATTR(energy2_input, 0444, ltc2947_show_value, NULL,
			  LTC2947_ENERGY2_INPUT);
static SENSOR_DEVICE_ATTR(energy2_max, 0644, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_ENERGY2_THRE_H);
static SENSOR_DEVICE_ATTR(energy2_min, 0644, ltc2947_show_value,
			  ltc2947_set_value, LTC2947_ENERGY2_THRE_L);
static SENSOR_DEVICE_ATTR(energy2_max_alarm, 0444, ltc2947_show_alert, NULL,
			  LTC2947_ENERGY2_MAX_ALARM);
static SENSOR_DEVICE_ATTR(energy2_min_alarm, 0444, ltc2947_show_alert, NULL,
			  LTC2947_ENERGY2_MIN_ALARM);
/*
 *
 */
static SENSOR_DEVICE_ATTR(energy1_overflow_alarm, 0444, ltc2947_show_alert,
			  NULL, LTC2947_ENERGY1_OVERF_ALARM);
static SENSOR_DEVICE_ATTR(energy2_overflow_alarm, 0444, ltc2947_show_alert,
			  NULL, LTC2947_ENERGY2_OVERF_ALARM);

/*
 * Fault attributes indicate that the readings in the respective channel are
 * not to be trusted
 */
static SENSOR_DEVICE_ATTR(energy1_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_ENERGY_FAULT);
static SENSOR_DEVICE_ATTR(energy2_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_ENERGY_FAULT);
static SENSOR_DEVICE_ATTR(in0_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_FAULT);
static SENSOR_DEVICE_ATTR(in1_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_FAULT);
static SENSOR_DEVICE_ATTR(curr1_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_FAULT);
static SENSOR_DEVICE_ATTR(temp1_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_FAULT);
static SENSOR_DEVICE_ATTR(power1_fault, 0444, ltc2947_show_alert,
			  NULL, LTC2947_FAULT);

static struct attribute *ltc2947_attrs[] = {
	&sensor_dev_attr_test1.dev_attr.attr,
	&sensor_dev_attr_in0_fault.dev_attr.attr,
	&sensor_dev_attr_in1_fault.dev_attr.attr,
	&sensor_dev_attr_curr1_fault.dev_attr.attr,
	&sensor_dev_attr_temp1_fault.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_power1_max.dev_attr.attr,
	&sensor_dev_attr_power1_min.dev_attr.attr,
	&sensor_dev_attr_power1_input_highest.dev_attr.attr,
	&sensor_dev_attr_power1_input_lowest.dev_attr.attr,
	&sensor_dev_attr_power1_min_alarm.dev_attr.attr,
	&sensor_dev_attr_power1_fault.dev_attr.attr,
	&sensor_dev_attr_energy1_input.dev_attr.attr,
	&sensor_dev_attr_energy1_max.dev_attr.attr,
	&sensor_dev_attr_energy1_min.dev_attr.attr,
	&sensor_dev_attr_energy1_max_alarm.dev_attr.attr,
	&sensor_dev_attr_energy1_min_alarm.dev_attr.attr,
	&sensor_dev_attr_energy2_input.dev_attr.attr,
	&sensor_dev_attr_energy2_max.dev_attr.attr,
	&sensor_dev_attr_energy2_min.dev_attr.attr,
	&sensor_dev_attr_energy2_max_alarm.dev_attr.attr,
	&sensor_dev_attr_energy2_min_alarm.dev_attr.attr,
	&sensor_dev_attr_energy1_overflow_alarm.dev_attr.attr,
	&sensor_dev_attr_energy2_overflow_alarm.dev_attr.attr,
	&sensor_dev_attr_energy1_fault.dev_attr.attr,
	&sensor_dev_attr_energy2_fault.dev_attr.attr,
	NULL,
};
ATTRIBUTE_GROUPS(ltc2947);

static void ltc2947_clk_disable(void *data)
{
	struct clk *extclk = data;

	clk_disable_unprepare(extclk);
}

static int ltc2947_setup(struct ltc2947_data *st)
{
	int ret;
	struct clk *extclk;
	u32 dummy, deadband;
	u32 accum[2];
	/* clear status register by reading it */
	ret = regmap_read(st->map, LTC2947_REG_STATUS, &dummy);
	if (ret)
		return ret;

	/* check external clock presence */
	extclk = devm_clk_get(st->dev, NULL);
	if (!IS_ERR(extclk)) {
		unsigned long rate_hz;
		u8 pre = 0, div, tbctl;
		u64 aux;

		/* let's calculate and set the right valus in TBCTL */
		rate_hz = clk_get_rate(extclk);
		if (rate_hz < LTC2947_CLK_MIN || rate_hz > LTC2947_CLK_MAX) {
			dev_err(st->dev, "Invalid rate:%lu for external clock",
				rate_hz);
			return -EINVAL;
		}

		ret = clk_prepare_enable(extclk);
		if (ret)
			return ret;

		ret = devm_add_action_or_reset(st->dev, ltc2947_clk_disable,
					       extclk);
		if (ret)
			return ret;
		/* as in table 1 of the datasheet */
		if (rate_hz >= LTC2947_CLK_MIN && rate_hz <= 1000000)
			pre = 0;
		else if (rate_hz > 1000000 && rate_hz <= 2000000)
			pre = 1;
		else if (rate_hz > 2000000 && rate_hz <= 4000000)
			pre = 2;
		else if (rate_hz > 4000000 && rate_hz <= 8000000)
			pre = 3;
		else if (rate_hz > 8000000 && rate_hz <= 16000000)
			pre = 4;
		else if (rate_hz > 16000000 && rate_hz <= LTC2947_CLK_MAX)
			pre = 5;
		/*
		 * Div is given by:
		 *	floor(fref / (2^PRE * 32768))
		 */
		div = rate_hz / ((1 << pre) * 32768);
		tbctl = LTC2947_PRE(pre) | LTC2947_DIV(div);

		ret = regmap_write(st->map, LTC2947_REG_TBCTL, tbctl);
		if (ret)
			return ret;
		/*
		 * The energy lsb is given by (in W*s):
		 *      06416 * (1/fref) * 2^PRE * (DIV + 1)
		 * The value is multiplied by 10E9
		 */
		aux = (div + 1) * ((1 << pre) * 641600000ULL);
		st->lsb_energy = DIV_ROUND_CLOSEST_ULL(aux, rate_hz);
	} else {
		/* 19.89E-6 * 10E9 */
		st->lsb_energy = 19890;
	}
	ret = of_property_read_u32_array(st->dev->of_node,
					 "adi,accumulator-ctl-pol", accum,
					  ARRAY_SIZE(accum));
	if (!ret) {
		u32 accum_reg = LTC2947_ACCUM_POL_1(accum[0]) |
				LTC2947_ACCUM_POL_2(accum[1]);

		ret = regmap_write(st->map, LTC2947_REG_ACCUM_POL, accum_reg);
		if (ret)
			return ret;
	}
	ret = of_property_read_u32(st->dev->of_node,
				   "adi,accumulation-deadband-microamp",
				   &deadband);
	if (!ret) {
		/* the LSB is the same as the current, so 3mA */
		ret = regmap_write(st->map, LTC2947_REG_ACCUM_DEADBAND,
				   deadband/(1000 * 3));
		if (ret)
			return ret;
	}

	/* set continuos mode */
	return regmap_update_bits(st->map, LTC2947_REG_CTRL,
				  LTC2947_CONT_MODE_MASK, LTC2947_CONT_MODE(1));
}

int ltc2947_core_probe(struct regmap *map, const char *name)
{
	struct ltc2947_data *st;
	struct device *dev = regmap_get_device(map);
	struct device *hwmon;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->map = map;
	st->dev = dev;
	dev_set_drvdata(dev, st);
	mutex_init(&st->lock);

	ret = ltc2947_setup(st);
	if (ret)
		return ret;

	hwmon = devm_hwmon_device_register_with_info(dev, name, st,
						     &ltc2947_chip_info,
						     ltc2947_groups);
	return PTR_ERR_OR_ZERO(hwmon);
}
EXPORT_SYMBOL_GPL(ltc2947_core_probe);

static int __maybe_unused ltc2947_resume(struct device *dev)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	u32 ctrl = 0;
	int ret;

	mutex_lock(&st->lock);
	/* dummy read to wake the device */
	ret = regmap_read(st->map, LTC2947_REG_CTRL, &ctrl);
	if (ret)
		goto unlock;

	/*
	 * Wait for the device. It takes 100ms to wake up so, 10ms extra
	 * should be enough.
	 */
	msleep(110);
	ret = regmap_read(st->map, LTC2947_REG_CTRL, &ctrl);
	if (ret)
		goto unlock;
	/* ctrl should be 0 */
	if (ctrl != 0) {
		dev_err(st->dev, "Device failed to wake up, ctl:%02X\n", ctrl);
		ret = -ETIMEDOUT;
		goto unlock;
	}

	st->reset = false;
	/* set continuous mode */
	ret = regmap_update_bits(st->map, LTC2947_REG_CTRL,
				 LTC2947_CONT_MODE_MASK, LTC2947_CONT_MODE(1));
unlock:
	mutex_unlock(&st->lock);
	return ret;
}

static int __maybe_unused ltc2947_suspend(struct device *dev)
{
	struct ltc2947_data *st = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_update_bits(st->map, LTC2947_REG_CTRL,
				 LTC2947_SHUTDOWN_MASK, 1);
	if (ret)
		goto unlock;

	st->reset = true;
unlock:
	mutex_unlock(&st->lock);
	return ret;
}

SIMPLE_DEV_PM_OPS(ltc2947_pm_ops, ltc2947_suspend, ltc2947_resume);
EXPORT_SYMBOL_GPL(ltc2947_pm_ops);

const struct of_device_id ltc2947_of_match[] = {
	{ .compatible = "adi,ltc2947" },
	{}
};
EXPORT_SYMBOL_GPL(ltc2947_of_match);
MODULE_DEVICE_TABLE(of, ltc2947_of_match);
