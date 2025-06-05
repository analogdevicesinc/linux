// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices LTC4283 I2C Negative Voltage Hot Swap Controller (HWMON)
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h> // devres.h when upstreaming
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mod_devicetable.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/units.h>

#define LTC4283_FAULT_STATUS		0x03
#define   LTC4283_OV_MASK		BIT(0)
#define   LTC4283_UV_MASK		BIT(1)
#define   LTC4283_OC_MASK		BIT(2)
#define   LTC4283_FET_BAD_MASK		BIT(3)
#define   LTC4283_FET_SHORT_MASK	BIT(6)
#define LTC4283_FAULT_LOG		0x04
#define LTC4283_ADC_ALM_LOG_1		0x05
#define   LTC4283_POWER_LOW_ALM		BIT(0)
#define   LTC4283_POWER_HIGH_ALM	BIT(1)
#define   LTC4283_SENSE_LOW_ALM		BIT(4)
#define   LTC4283_SENSE_HIGH_ALM	BIT(5)
#define LTC4283_ADC_ALM_LOG_2		0x06
#define LTC4283_ADC_ALM_LOG_3		0x07
#define LTC4283_ADC_ALM_LOG_4		0x08
#define LTC4283_ADC_ALM_LOG_5		0x09
#define LTC4283_ADC_SELECT(c)		(0x13 + (c) / 8)
#define   LTC4283_ADC_SELECT_MASK(c)	BIT((c) % 8)
#define LTC4283_SENSE_MIN_TH		0x1b
#define LTC4283_SENSE_MAX_TH		0x1c
#define LTC4283_VPWR_MIN_TH		0x1d
#define LTC4283_VPWR_MAX_TH		0x1e
#define LTC4283_POWER_MIN_TH		0x1f
#define LTC4283_POWER_MAX_TH		0x20
#define LTC4283_ADC_2_MIN_TH(c)		(0x21 + (c) * 2)
#define LTC4283_ADC_2_MAX_TH(c)		(0x22 + (c) * 2)
#define LTC4283_SENSE			0x41
#define LTC4283_SENSE_MIN		0x42
#define LTC4283_SENSE_MAX		0x43
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
#define LTC4283_ENERGY			0x7a
/* also applies for differential channels */
#define LTC4283_ADC1_FS_uV		32768
#define LTC4283_ADC2_FS_mV		2048
#define LTC4283_TCONV_uS		64103

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

struct ltc4283_hwmon {
	struct regmap *map;
	/* lock to protect concurrent device accesses and shared data */
	struct mutex lock;
	u32 vsense_max;
	u32 rsense;
	bool energy_en;
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

static int ltc4283_hwmon_read_alarm(struct ltc4283_hwmon *st, u32 reg,
				    u32 mask, long *val)
{
	u32 alarm;
	int ret;

	guard(mutex)(&st->lock);
	ret = regmap_read(st->map, reg, &alarm);
	if (ret)
		return ret;

	*val = !!(alarm & mask);

	/* if not status/fault logs, clear the alarm after reading it */
	if (reg != LTC4283_FAULT_STATUS && reg != LTC4283_FAULT_LOG)
		return regmap_clear_bits(st->map, reg, mask);

	return 0;
}

static int ltc4283_hwmon_read_in_alarm(struct ltc4283_hwmon *st, u32 channel,
				       bool max_alm, long *val)
{
	if (channel == LTC4283_HWMON_VPWR)
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_1,
						BIT(2 + max_alm), val);

	if (channel >= LTC4283_HWMON_ADI_1 && channel <= LTC4283_HWMON_ADI_4) {
		u32 bit = (channel - LTC4283_HWMON_ADI_1) * 2;
		/*
		 * Lower channels go to higher bits. We also want to go +1 down
		 * in the min_alarm case.
		 */
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_2,
						BIT(7 - bit - !max_alm), val);
	}

	if (channel >= LTC4283_HWMON_ADIO_1 && channel <= LTC4283_HWMON_ADIO_4) {
		u32 bit = (channel - LTC4283_HWMON_ADIO_1) * 2;

		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_3,
						BIT(7 - bit - !max_alm), val);
	}

	if (channel == LTC4283_HWMON_DRNS)
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_4,
						BIT(6 + max_alm), val);

	return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_4,
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
	case hwmon_in_crit_alarm:
	return ltc4283_hwmon_read_alarm(st, LTC4283_FAULT_STATUS,
					LTC4283_OV_MASK, val);
	case hwmon_in_lcrit_alarm:
	return ltc4283_hwmon_read_alarm(st, LTC4283_FAULT_STATUS,
					LTC4283_UV_MASK, val);
	case hwmon_in_fault:
		/*
		 * We report failure if we detect either a fer_bad or a
		 * fet_short in the status register. Should we also consider
		 * external failure as DRAIN fault?
		 */
		return ltc4283_hwmon_read_alarm(st, LTC4283_FAULT_STATUS,
						LTC4283_FET_BAD_MASK | LTC4283_FET_BAD_MASK, val);
	case hwmon_in_enable:
		return ltc4283_hwmon_read_in_en(st, channel, val);
	default:
		return -EOPNOTSUPP;
	}
	return 0;
}

static int ltc4283_read_current_word(const struct ltc4283_hwmon *st, u32 reg,
				     long *val)
{
	u64 temp = DECA * LTC4283_ADC1_FS_uV * MICRO;
	__be16 curr;
	int ret;

	ret = regmap_bulk_read(st->map, reg, &curr, sizeof(curr));
	if (ret)
		return ret;

	*val = DIV64_U64_ROUND_CLOSEST(be16_to_cpu(curr) * temp,
				       BIT_ULL(16) * st->rsense);

	return 0;
}

static int ltc4283_read_current_byte(const struct ltc4283_hwmon *st, u32 reg,
				     long *val)
{
	u32 curr;
	int ret;

	ret = regmap_read(st->map, reg, &curr);
	if (ret)
		return ret;

	*val = DIV_ROUND_CLOSEST_ULL(curr * LTC4283_ADC1_FS_uV * DECA * MICRO,
				     BIT(8) * st->rsense);

	return 0;
}

static int ltc4283_hwmon_read_curr(struct ltc4283_hwmon *st, u32 attr,
				   long *val)
{
	switch (attr) {
	case hwmon_curr_input:
		return ltc4283_read_current_word(st, LTC4283_SENSE, val);
	case hwmon_curr_highest:
		return ltc4283_read_current_word(st, LTC4283_SENSE_MAX, val);
	case hwmon_curr_lowest:
		return ltc4283_read_current_word(st, LTC4283_SENSE_MIN, val);
	case hwmon_curr_max:
		return ltc4283_read_current_byte(st, LTC4283_SENSE_MAX_TH, val);
	case hwmon_curr_min:
		return ltc4283_read_current_byte(st, LTC4283_SENSE_MIN_TH, val);
	case hwmon_curr_max_alarm:
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_1,
						LTC4283_SENSE_HIGH_ALM, val);
	case hwmon_curr_min_alarm:
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_1,
						LTC4283_SENSE_LOW_ALM, val);
	case hwmon_curr_crit_alarm:
		return ltc4283_hwmon_read_alarm(st, LTC4283_FAULT_STATUS,
						LTC4283_OC_MASK, val);
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc4283_hwmon_read_power_word(const struct ltc4283_hwmon *st,
					 u32 reg, long *val)
{
	u64 temp = LTC4283_ADC1_FS_uV * LTC4283_ADC2_FS_mV * DECA * MICRO;
	__be16 raw;
	int ret;

	ret = regmap_bulk_read(st->map, reg, &raw, sizeof(raw));
	if (ret)
		return ret;

	/*
	 * Power is given by:
	 *     P = CODE(16b) * 32.768mV * 2.048V / (2^16 * Rsense)
	 */
	if (check_mul_overflow(temp, be16_to_cpu(raw), &temp)) {
		temp = DIV64_U64_ROUND_CLOSEST(temp, BIT_ULL(16) * st->rsense);
		*val = temp * be16_to_cpu(raw);
		return 0;
	}

	*val = DIV64_U64_ROUND_CLOSEST(temp, BIT_ULL(16) * st->rsense);

	return 0;
}

static int ltc4283_hwmon_read_power_byte(const struct ltc4283_hwmon *st,
					 u32 reg, long *val)
{
	u64 temp = LTC4283_ADC1_FS_uV * LTC4283_ADC2_FS_mV * DECA;
	u32 power;
	int ret;

	ret = regmap_read(st->map, reg, &power);
	if (ret)
		return ret;

	*val = DIV64_U64_ROUND_CLOSEST(power * temp * MICRO,
				       BIT_ULL(8) * st->rsense);

	return 0;
}

static int ltc4283_hwmon_read_power(struct ltc4283_hwmon *st, u32 attr,
				    long *val)
{
	switch (attr) {
	case hwmon_power_input:
		return ltc4283_hwmon_read_power_word(st, LTC4283_POWER, val);
	case hwmon_power_input_highest:
		return ltc4283_hwmon_read_power_word(st, LTC4283_POWER_MAX, val);
	case hwmon_power_input_lowest:
		return ltc4283_hwmon_read_power_word(st, LTC4283_POWER_MIN, val);
	case hwmon_power_max_alarm:
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_1,
						LTC4283_POWER_HIGH_ALM, val);
	case hwmon_power_min_alarm:
		return ltc4283_hwmon_read_alarm(st, LTC4283_ADC_ALM_LOG_1,
						LTC4283_POWER_LOW_ALM, val);
	case hwmon_power_max:
		return ltc4283_hwmon_read_power_byte(st, LTC4283_POWER_MAX_TH,
						     val);
	case hwmon_power_min:
		return ltc4283_hwmon_read_power_byte(st, LTC4283_POWER_MIN_TH,
						     val);
	default:
		return -EOPNOTSUPP;
	}
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
		scoped_guard(mutex, &st->lock) {
			*val = st->energy_en;
		}
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc4283_hwmon_write_power(struct ltc4283_hwmon *st, u32 attr,
				     long val)
{
	switch (attr) {
	case hwmon_power_max:
	case hwmon_power_min:
	case hwmon_power_reset_history:
	default:
		return -EOPNOTSUPP;
}

static int ltc4283_hwmon_write_in_history(const struct ltc4282_state *st,
					  u32 reg, long lowest, long highest,
					  u32 fs)
{
	__be16 __raw;
	u16 tmp;

	guard(mutex)(&st->lock);

	tmp = DIV_ROUND_CLOSEST(BIT(16) * lowest, fs);
	if (tmp == BIT(16))
		tmp = U16_MAX;

	__raw = cpu_to_be16(tmp);

	ret = regmap_bulk_write(st->map, reg, &__raw, sizeof(__raw));
	if (ret)
		return ret;

	/* Make sure it's really zero for all */
	return regmap_bulk_write(st->map, reg + 1,  0, sizeof(__raw));
}

static int ltc4283_hwmon_write_in_byte(const struct ltc4283_hwmon *st,
				       u32 reg, u32 fs, long val)
{
	u32 __raw;

	val = clamp_val(val, 0, fs);

	__raw = DIV_ROUND_CLOSEST(val * BIT(16), fs);
	if (__raw == BIT(16))
		__raw = U16_MAX;

	return regmap_write(st->map, reg, __raw)
}

static int ltc4283_hwmon_write_in_en(const struct ltc4283_hwmon *st,
				     u32 channel, bool en)
{
	unsigned int bit;

	bit = LTC4283_ADC_SELECT_MASK(channel - LTC4283_HWMON_ADI_1);
	if (channel > LTC4283_HWMON_DRAIN)
		/* account for two reserved fields after DRAIN */
		bit += 2;

	/* I'll need a non constant version of field_prep(). Or, create
	 * a static const map of masks with FIELD_PREP_CONST(). Not sure
	 * if it's worth it.
	 */
	return regmap_update_bits(st->map, LTC4283_ADC_SELECT(channel - 1),
				  bit, FIELD_PREP(bit, en));
}

static int ltc4283_hwmon_write_in(struct ltc4283_hwmon *st, u32 attr, long val,
				  int channel)
{
	u32 reg;

	switch (attr) {
	case hwmon_in_max:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_write_voltage_byte(st,
								LTC4283_VPWR_MAX_TH,
								LTC4283_ADC2_FS_uV, val);

		reg = LTC4283_ADC_2_MAX_TH(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_write_voltage_byte(st, reg, 2048, val);
	case hwmon_in_min:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_write_voltage_byte(st,
								LTC4283_VPWR_MIN_TH,
								LTC4283_ADC2_FS_uV, val);

		reg = LTC4283_ADC_2_MIN_TH(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_write_voltage_byte(st, reg, 2048, val);
	case hwmon_in_reset_history:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_write_in_history(st,
							      LTC4283_VPWR_MIN,
							      LTC4283_ADC2_FS_uV, 0,
							      LTC4283_ADC2_FS_uV);

		reg = LTC4283_ADC_2_MIN(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_write_in_history(st, reg,
						      LTC4283_ADC2_FS_uV, 0),
						      LTC4283_ADC2_FS_uV);
	case hwmon_in_enable:
		return ltc4283_hwmon_write_in_en(st, channel, !!val);
	default:
		return -EOPNOTSUPP;
}

static int ltc4283_hwmon_reset_curr_hist(const struct ltc4283_hwmon *st,
					 u32 reg)
{
	u64 temp = DECA * LTC4283_ADC1_FS_uV * MICRO;
	__be16 reg_val;
	u16 tmp;

	return regmap_bulk_write(st->map, LTC4283_SENSE_MAX, 0,
				 sizeof(reg_val));
}

static int ltc4283_hwmon_write_curr_byte(const struct ltc4283_hwmon *st,
					 u32 reg, long val)
{
	u64 temp = (u64)LTC4283_ADC1_FS_uV * DECA * MICRO;
	u32 reg_val;

	reg_val = DIV64_U64_ROUND_CLOSEST(val * BIT_ULL(8) * st->rsense, temp);
	return regmap_write(st->map, reg, reg_val)
}

static int ltc4283_hwmon_write_curr(struct ltc4283_hwmon *st, u32 attr,
				    long val)
{
	switch (attr) {
	case hwmon_curr_max:
		return ltc4283_hwmon_write_in_history(st, LTC4283_SENSE_MIN_TH,
						      val);
	case hwmon_curr_min:
		return ltc4283_hwmon_write_in_history(st, LTC4283_SENSE_MIN_TH,
						      val);
	case hwmon_curr_reset_history:
		return ltc4283_hwmon_write_in_history(st, LTC4283_SENSE_MIN,
						      st->vsense_max 0,
						      LTC4283_ADC1_FS_uV);
		/* Should I also reset OC fault log?? */
	default:
		return -EOPNOTSUPP;
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
	u64 temp = LTC4283_ADC1_FS_uV * LTC4283_ADC2_FS_mV * DECA, energy;
	struct ltc4283_hwmon *st = dev_get_drvdata(dev);
	__be64 raw;
	int ret;

	ret = regmap_bulk_read(st->map, LTC4283_ENERGY, &raw, 6);
	if (ret)
		return ret;

	energy =  be64_to_cpu(raw) >> 16;
	/*
	 * The formula for energy is given by:
	 *	E = CODE(48b) * 32.768mV * 2.048V * Tconv / 2^24 * Rsense
	 *
	 * As Rsense can have tenths of micro-ohm resolution, we need to
	 * multiply by DECA to get microjoule.
	 */
	if (check_mul_overflow(temp * LTC4283_TCONV_uS, energy, &temp)) {
		/*
		 * We multiply again by 1000 to make sure that we don't get 0
		 * in the following division which could happen for big rsense
		 * values. OTOH, we then divide energy first by 1000 so that
		 * we do not overflow u64 again for very small rsense values.
		 */
		temp = DIV64_U64_ROUND_CLOSEST(temp * LTC4283_TCONV_uS * MILLI,
					       BIT_ULL(24) * st->rsense);
		energy = DIV_ROUND_CLOSEST_ULL(energy, MILLI) * temp;
	} else {
		energy = DIV64_U64_ROUND_CLOSEST(temp, BIT_ULL(24) * st->rsense);
	}

	return sysfs_emit(buf, "%llu\n", energy);
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
