// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices LTC4283 I2C Negative Voltage Hot Swap Controller (HWMON)
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h> // devres.h when upstreaming
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/mfd/ltc4283.h>
#include <linux/minmax.h>
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
#define   LTC4283_OV_FAULT_MASK		BIT(0)
#define   LTC4283_UV_FAULT_MASK		BIT(1)
#define   LTC4283_OC_FAULT_MASK		BIT(2)
#define LTC4283_ADC_ALM_LOG_1		0x05
#define   LTC4283_POWER_LOW_ALM		BIT(0)
#define   LTC4283_POWER_HIGH_ALM	BIT(1)
#define   LTC4283_SENSE_LOW_ALM		BIT(4)
#define   LTC4283_SENSE_HIGH_ALM	BIT(5)
#define LTC4283_ADC_ALM_LOG_2		0x06
#define LTC4283_ADC_ALM_LOG_3		0x07
#define LTC4283_ADC_ALM_LOG_4		0x08
#define LTC4283_ADC_ALM_LOG_5		0x09
#define LTC4283_CONTROL_1		0x0a
#define   LTC4283_PIGIO2_ACLB_MASK	BIT(2)
#define   LTC4283_PWRGD_RST_CTRL_MASK	BIT(3)
#define   LTC4283_FET_BAD_OFF_MASK	BIT(4)
#define   LTC4283_THERM_TMR_MASK	BIT(5)
#define   LTC4283_DVDT_MASK		BIT(6)
#define LTC4283_CONTROL_2		0x0b
#define   LTC4283_OV_RETRY_MASK		BIT(0)
#define   LTC4283_UV_RETRY_MASK		BIT(1)
#define   LTC4283_OC_RETRY_MASK		GENMASK(3, 2)
#define   LTC4283_FET_BAD_RETRY_MASK	GENMASK(5, 4)
#define   LTC4283_EXT_FAULT_RETRY_MASK	BIT(7)
#define LTC4283_CONFIG_1		0x0d
#define   LTC4283_FB_MASK		GENMASK(3, 2)
#define   LTC4283_ILIM_MASK		GENMASK(7, 4)
#define LTC4283_CONFIG_2		0x0e
#define   LTC4283_COOLING_DL_MASK	GENMASK(3, 0)
#define   LTC4283_FTBD_DL_MASK		GENMASK(5, 4)
#define LTC4283_CONFIG_3		0x0f
#define   LTC4283_VPWR_DRNS_MASK	BIT(6)
#define   LTC4283_EXTFLT_TURN_OFF_MASK	BIT(7)
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
#define LTC4283_METER_CONTROL		0x84
#define   LTC4283_INTEGRATE_I_MASK	BIT(0)
#define   LTC4283_METER_HALT_MASK	BIT(6)
#define LTC4283_FAULT_LOG_CTRL		0x90
#define   LTC4283_FAULT_LOG_EN_MASK	BIT(7)

/* also applies for differential channels */
#define LTC4283_ADC1_FS_uV		32768
#define LTC4283_ADC2_FS_mV		2048
#define LTC4283_TCONV_uS		64103
#define LTC4283_VILIM_MIN_mV		15
#define LTC4283_VILIM_MAX_mV		30
#define LTC4283_VILIM_RANGE	\
	(LTC4283_VILIM_MAX_mV - LTC4283_VILIM_MIN_mV + 1)

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
	unsigned long *gpio_mask;
	/* lock to protect concurrent device accesses and shared data */
	struct mutex lock;
	unsigned long ch_enable_mask;
	/* in microwatt */
	long power_max;
	/* in millivolt */
	u32 vsense_max;
	/* in tenths of microohm*/
	u32 rsense;
	bool energy_en;
	bool ext_fault;
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
		scoped_guard(mutex, &st->lock) {
			*val = test_bit(channel, &st->ch_enable_mask);
		}
		return 0;
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

static int ltc4282_write_power_byte(const struct ltc4283_hwmon *st, u32 reg,
				    long val)
{
	u64 temp = (u64)LTC4283_ADC1_FS_uV * LTC4283_ADC2_FS_mV * DECA * MICRO;
	u32 __raw;

	/* can we have negative powers?! */
	if (val > st->power_max)
		val = st->power_max;

	__raw = DIV64_U64_ROUND_CLOSEST(val * BIT_ULL(8) * st->rsense, temp);

	return regmap_write(st->map, reg, __raw);
}

static int ltc4283_hwmon_write_power_word(const struct ltc4283_hwmon *st,
					  u32 reg, long val)
{
	u64 temp = st->rsense * BIT_ULL(16), temp_2;
	__be16 __raw;
	u16 code;

	if (check_mul_overflow(val, temp, &temp_2)) {
		temp = DIV_ROUND_CLOSEST_ULL(temp, DECA * MICRO);
		code = DIV_ROUND_CLOSEST_ULL(temp * val, LTC4283_ADC1_FS_uV * LTC4283_ADC2_FS_mV);
	} else {
		temp = (u64)DECA * MICRO * LTC4283_ADC1_FS_uV * LTC4283_ADC2_FS_mV;
		code = DIV64_U64_ROUND_CLOSEST(temp_2, temp);
	}

	__raw = cpu_to_be16(code);
	return regmap_bulk_write(st->map, reg, &__raw, sizeof(__raw));
}

static int ltc4283_hwmon_reset_power_hist(struct ltc4283_hwmon *st)
{
	int ret;

	guard(mutex)(&st->lock);

	/* reset the power history */
	ret = ltc4283_hwmon_write_power_word(st, LTC4283_POWER_MIN,
					     st->power_max);
	if (ret)
		return ret;

	ret = ltc4283_hwmon_write_power_word(st, LTC4283_POWER_MAX, 0);
	if (ret)
		return ret;

	/* clear possible power faults?!! */
	return 0;
}

static int ltc4283_hwmon_write_power(struct ltc4283_hwmon *st, u32 attr,
				     long val)
{
	switch (attr) {
	case hwmon_power_max:
		return ltc4282_write_power_byte(st, LTC4283_POWER_MAX_TH, val);
	case hwmon_power_min:
		return ltc4282_write_power_byte(st, LTC4283_POWER_MIN_TH, val);
	case hwmon_power_reset_history:
		return ltc4283_hwmon_reset_power_hist(st);
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc4283_hwmon_write_in_history(struct ltc4283_hwmon *st,
					  u32 reg, long lowest, long highest,
					  u32 fs)
{
	__be16 __raw;
	u16 tmp;
	int ret;

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

	return regmap_write(st->map, reg, __raw);
}

static int ltc4283_hwmon_write_in_en(struct ltc4283_hwmon *st, u32 channel,
				     bool en)
{
	unsigned int bit;
	int ret;

	bit = LTC4283_ADC_SELECT_MASK(channel - LTC4283_HWMON_ADI_1);
	if (channel > LTC4283_HWMON_DRAIN)
		/* account for two reserved fields after DRAIN */
		bit += 2;

	guard(mutex)(&st->lock);
	ret = regmap_update_bits(st->map, LTC4283_ADC_SELECT(channel - 1),
				 bit, field_prep(bit, en));
	if (ret)
		return ret;

	set_bit(channel, &st->ch_enable_mask);
	return 0;
}

static int ltc4283_hwmon_write_in(struct ltc4283_hwmon *st, u32 attr, long val,
				  int channel)
{
	u32 reg;

	switch (attr) {
	case hwmon_in_max:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_write_in_byte(st,
							   LTC4283_VPWR_MAX_TH,
							   LTC4283_ADC2_FS_mV,
							   val);

		reg = LTC4283_ADC_2_MAX_TH(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_write_in_byte(st, reg, LTC4283_ADC2_FS_mV,
						   val);
	case hwmon_in_min:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_write_in_byte(st,
							   LTC4283_VPWR_MIN_TH,
							   LTC4283_ADC2_FS_mV,
							   val);

		reg = LTC4283_ADC_2_MIN_TH(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_write_in_byte(st, reg, LTC4283_ADC2_FS_mV,
						   val);
	case hwmon_in_reset_history:
		if (channel == LTC4283_HWMON_VPWR)
			return ltc4283_hwmon_write_in_history(st,
							      LTC4283_VPWR_MIN,
							      LTC4283_ADC2_FS_mV, 0,
							      LTC4283_ADC2_FS_mV);

		reg = LTC4283_ADC_2_MIN(channel - LTC4283_HWMON_ADI_1);
		return ltc4283_hwmon_write_in_history(st, reg,
						      LTC4283_ADC2_FS_mV, 0,
						      LTC4283_ADC2_FS_mV);
	case hwmon_in_enable:
		return ltc4283_hwmon_write_in_en(st, channel, !!val);
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc4283_hwmon_write_curr_byte(const struct ltc4283_hwmon *st,
					 u32 reg, long val)
{
	u64 temp = (u64)LTC4283_ADC1_FS_uV * DECA * MICRO;
	u32 reg_val;

	reg_val = DIV64_U64_ROUND_CLOSEST(val * BIT_ULL(8) * st->rsense, temp);
	return regmap_write(st->map, reg, reg_val);
}

static int ltc4283_hwmon_write_curr_history(struct ltc4283_hwmon *st)
{
	int ret;

	guard(mutex)(&st->lock);

	/* reset the current history */
	ret = ltc4283_hwmon_write_in_history(st, LTC4283_SENSE_MIN,
					     st->vsense_max, 0,
					     LTC4283_ADC1_FS_uV);
	if (ret)
		return ret;

	/* now, let's also clear possible overcurrent fault logs */
	return regmap_clear_bits(st->map, LTC4283_FAULT_LOG,
				 LTC4283_OC_FAULT_MASK);
}

static int ltc4283_hwmon_write_curr(struct ltc4283_hwmon *st, u32 attr,
				    long val)
{
	switch (attr) {
	case hwmon_curr_max:
		return ltc4283_hwmon_write_curr_byte(st, LTC4283_SENSE_MIN_TH,
						     val);
	case hwmon_curr_min:
		return ltc4283_hwmon_write_curr_byte(st, LTC4283_SENSE_MIN_TH,
						     val);
	case hwmon_curr_reset_history:
		return ltc4283_hwmon_write_curr_history(st);
	default:
		return -EOPNOTSUPP;
	}
}

static int ltc4283_hwmon_energy_enable_set(struct ltc4283_hwmon *st, long val)
{
	int ret;

	guard(mutex)(&st->lock);

	/* setting the bit halts the meter */
	val = !!val;
	ret = regmap_update_bits(st->map, LTC4283_METER_CONTROL,
				 LTC4283_METER_HALT_MASK,
				 FIELD_PREP(LTC4283_METER_HALT_MASK, !val));
	if (ret)
		return ret;

	st->energy_en = val;

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
					   u32 attr, int channel)
{
	/* if ADIO is set as a GPIO, don´t make it visible */
	if (channel >= LTC4283_HWMON_ADIO_1 && channel <= LTC4283_HWMON_ADIO_4) {
		/* ADIOX pins come at LTC4283_ADIOX_START_NR in the gpio mask */
		channel -= LTC4283_HWMON_ADIO_1 + LTC4283_ADIOX_START_NR;
		if (test_bit(channel, st->gpio_mask))
			return 0;
	}

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
		return ltc4283_hwmon_in_is_visible(data, attr, channel);
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

/*
 * Set max limits for ISENSE and Power as that depends on the max voltage on
 * rsense that is defined in ILIM_ADJUST. This is specially important for power
 * because for some rsense and vfsout values, if we allow the default raw 255
 * value, that would overflow long in 32bit archs when reading back the max
 * power limit.
 */
static int ltc4283_hwmon_set_max_limits(struct ltc4283_hwmon *st)
{
	u32 temp = st->vsense_max * DECA * MICRO;
	int ret;

	ret = ltc4283_hwmon_write_in_byte(st, LTC4283_SENSE_MAX,
					  LTC4283_ADC1_FS_uV,
					  st->vsense_max * MILLI);
	if (ret)
		return ret;

	/* Power is given by ISENSE * Vout. */
	st->power_max = DIV_ROUND_CLOSEST(temp, st->rsense) * LTC4283_ADC2_FS_mV;
	return ltc4282_write_power_byte(st, LTC4283_POWER_MAX, st->power_max);
}

static int ltc4283_hwmon_set_array_prop(const struct ltc4283_hwmon *st,
					struct device *dev, const char *prop,
					const u32 *vals, u32 n_vals, u32 reg,
					u32 mask)
{
	u32 prop_val;
	int ret;
	u32 i;

	ret = device_property_read_u32(dev, prop, &prop_val);
	if (ret)
		return 0;

	for (i = 0; i < n_vals; i++) {
		if (prop_val != vals[i])
			continue;

		return regmap_update_bits(st->map, reg, mask,
					  field_prep(mask, i));
	}

	return dev_err_probe(dev, -EINVAL,
			     "Invalid %s property value %u, expected one of: %*ph\n",
			     prop, prop_val, n_vals, vals);
}

static int ltc4283_hwmon_get_defaults(struct ltc4283_hwmon *st)
{
	u32 reg_val, ilm_adjust;
	int ret;

	ret = regmap_read(st->map, LTC4283_METER_CONTROL, &reg_val);
	if (ret)
		return ret;

	st->energy_en = !FIELD_GET(LTC4283_METER_HALT_MASK, reg_val);

	ret = regmap_read(st->map, LTC4283_CONFIG_1, &reg_val);
	if (ret)
		return ret;

	ilm_adjust = FIELD_GET(LTC4283_ILIM_MASK, reg_val);
	st->vsense_max = LTC4283_VILIM_MIN_mV + ilm_adjust;

	return 0;
}

static const char * const ltc4283_pgio_funcs[][3] = {
	{ "inverted_power_good", "power_good" },
	{ "inverted_power_good", "power_good", "active_current_limit" },
	{ "inverted_power_good_input", "power_good_input" },
	{ "inverted_external_fault", "external_fault"}
};

static const char * const ltc4283_pgio_props[] = {
	"adi,pgio1-func", "adi,pgio2-func", "adi,pgio3-func", "adi,pgio4-func"
};

static int ltc4283_pgio_config(struct ltc4283_hwmon *st, struct device *dev)
{
	u32 pgio;
	int ret;

	for (pgio = 0; pgio < ARRAY_SIZE(ltc4283_pgio_props); pgio++) {
		ret = device_property_match_property_string(dev, ltc4283_pgio_props[pgio],
							    ltc4283_pgio_funcs[pgio],
							    ARRAY_SIZE(ltc4283_pgio_funcs[pgio]));
		if (ret < 0)
			continue;
		if (test_bit(pgio, st->gpio_mask))
			return dev_err_probe(dev, -EINVAL,
					     "PGIO%u already configured as GPIO\n",
					     pgio + 1);

		/* pgio2 and it's configured as active current limiting */
		if (pgio == 1 && ret == 2) {
			ret = regmap_set_bits(st->map, LTC4283_CONTROL_1,
					      LTC4283_PIGIO2_ACLB_MASK);
			if (ret)
				return ret;

			continue;
		}

		ret = regmap_update_bits(st->map, LTC4283_PGIO_CONFIG,
					 LTC4283_PGIO_CFG_MASK(pgio),
					 field_prep(LTC4283_PGIO_CFG_MASK(pgio), ret));
		if (ret)
			return ret;

		/* if pgio4, set ext_fault flag */
		if (pgio == 3)
			st->ext_fault = true;
	}

	return 0;
}

static const char * const ltc4283_oc_fet_retry[] = {
	"latch-off", "1", "7", "unlimited"
};

static const u32 ltc4283_fb_factor[] = {
	100, 50, 20, 10
};

static const u32 ltc4283_cooling_dl[] = {
	512, 1002, 2005, 4100, 8190, 16400, 32800, 65600
};

static const u32 ltc4283_fet_bad_delay[] = {
	256, 512, 1002, 2005
};

static int ltc4283_hwmon_setup(struct ltc4283_hwmon *st, struct device *dev)
{
	int ret;

	/* The part has an eeprom so let's get the needed defaults from it */
	ret = ltc4283_hwmon_get_defaults(st);
	if (ret)
		return ret;

	ret = device_property_read_u32(dev, "adi,rsense-nano-ohms",
				       &st->rsense);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to read adi,rsense-nano-ohms\n");
	if (st->rsense < CENTI)
		return dev_err_probe(dev, -EINVAL,
				     "adi,rsense-nano-ohms too small (< %lu)\n",
				     CENTI);

	/*
	 * The resolution for rsense is tenths of micro (eg: 62.5 uOhm) which
	 * means we need nano in the bindings. However, to make things easier to
	 * handle (with respect to overflows) we divide it by 100 as we don't
	 * really need the last two digits.
	 */
	st->rsense /= CENTI;
	ret = device_property_read_u32(dev, "adi,current-limit-sense-millivolt",
				       &st->vsense_max);
	if (!ret) {
		u32 reg_val;

		if (!in_range(st->vsense_max, LTC4283_VILIM_MIN_mV,
			      LTC4283_VILIM_RANGE)) {
			return dev_err_probe(dev, -EINVAL,
					     "adi,current-limit-sense-millivolt (%u) out of range [%u %u]\n",
					     st->vsense_max, LTC4283_VILIM_MIN_mV,
					     LTC4283_VILIM_MAX_mV);
		}

		reg_val = FIELD_PREP(LTC4283_ILIM_MASK,
				     st->vsense_max - LTC4283_VILIM_MIN_mV);
		ret = regmap_update_bits(st->map, LTC4283_CONFIG_1,
					 LTC4283_ILIM_MASK, reg_val);
		if (ret)
			return ret;
	}

	ret = ltc4283_hwmon_set_array_prop(st, dev, "adi,current-limit-foldback-factor",
					   ltc4283_fb_factor, ARRAY_SIZE(ltc4283_fb_factor),
					   LTC4283_CONFIG_1, LTC4283_FB_MASK);
	if (ret)
		return ret;

	ret = ltc4283_hwmon_set_array_prop(st, dev, "adi,cooling-delay-ms",
					   ltc4283_cooling_dl, ARRAY_SIZE(ltc4283_cooling_dl),
					   LTC4283_CONFIG_2, LTC4283_COOLING_DL_MASK);
	if (ret)
		return ret;

	ret = ltc4283_hwmon_set_array_prop(st, dev, "adi,fet-bad-timer-delay-ms",
					   ltc4283_fet_bad_delay, ARRAY_SIZE(ltc4283_fet_bad_delay),
					   LTC4283_CONFIG_2, LTC4283_FTBD_DL_MASK);
	if (ret)
		return ret;

	ret = ltc4283_hwmon_set_max_limits(st);
	if (ret)
		return ret;

	ret = ltc4283_pgio_config(st, dev);
	if (ret)
		return ret;

	if (device_property_read_bool(dev, "adi,power-good-reset-on-fet")) {
		ret = regmap_clear_bits(st->map, LTC4283_CONTROL_1,
					LTC4283_PWRGD_RST_CTRL_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,fet-turn-off-disable")) {
		ret = regmap_clear_bits(st->map, LTC4283_CONTROL_1,
					LTC4283_FET_BAD_OFF_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,tmr-pull-down-disable")) {
		ret = regmap_set_bits(st->map, LTC4283_CONTROL_1,
				      LTC4283_THERM_TMR_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,dvdt-inrush-control-disable")) {
		ret = regmap_clear_bits(st->map, LTC4283_CONTROL_1,
					LTC4283_DVDT_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,undervoltage-retry-disable")) {
		ret = regmap_clear_bits(st->map, LTC4283_CONTROL_2,
					LTC4283_UV_RETRY_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,overvoltage-retry-disable")) {
		ret = regmap_clear_bits(st->map, LTC4283_CONTROL_2,
					LTC4283_OV_RETRY_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,undervoltage-retry-disable")) {
		ret = regmap_clear_bits(st->map, LTC4283_CONTROL_2,
					LTC4283_UV_RETRY_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,external-fault-retry-enable")) {
		if (!st->ext_fault)
			return dev_err_probe(dev, -EINVAL,
					     "adi,external-fault-retry-enable set but PGIO4 not configured\n");
		ret = regmap_set_bits(st->map, LTC4283_CONTROL_2,
				      LTC4283_EXT_FAULT_RETRY_MASK);
		if (ret)
			return ret;
	}

	ret = device_property_match_property_string(dev, "adi,overcurrent-retries",
						    ltc4283_oc_fet_retry,
						    ARRAY_SIZE(ltc4283_oc_fet_retry));
	/* We still wanna catch when an invalid string is given */
	if (ret == -ENODATA)
		return dev_err_probe(dev, ret,
				     "adi,overcurrent-retries invalid value\n");
	if (ret >= 0) {
		ret = regmap_update_bits(st->map, LTC4283_CONTROL_2,
					 LTC4283_OC_RETRY_MASK,
					 FIELD_PREP(LTC4283_OC_RETRY_MASK, ret));
		if (ret)
			return ret;
	}

	ret = device_property_match_property_string(dev, "adi,fet-bad-retries",
						    ltc4283_oc_fet_retry,
						    ARRAY_SIZE(ltc4283_oc_fet_retry));
	if (ret == -ENODATA)
		return dev_err_probe(dev, ret,
				     "adi,fet-bad-retries invalid value\n");
	if (ret >= 0) {
		ret = regmap_update_bits(st->map, LTC4283_CONTROL_2,
					 LTC4283_FET_BAD_RETRY_MASK,
					 FIELD_PREP(LTC4283_FET_BAD_RETRY_MASK, ret));
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,external-fault-fet-off-enable")) {
		if (!st->ext_fault)
			return dev_err_probe(dev, -EINVAL,
					     "adi,external-fault-fet-off-enable set but PGIO4 not configured\n");
		ret = regmap_set_bits(st->map, LTC4283_CONFIG_3,
				      LTC4283_EXTFLT_TURN_OFF_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(dev, "adi,vpower-drns-enable")) {
		ret = regmap_set_bits(st->map, LTC4283_CONFIG_3,
				      LTC4283_VPWR_DRNS_MASK);
		if (ret)
			return ret;
	}

	/* Make sure the ADC has 12bit resolution since we're assuming that */
	ret = regmap_update_bits(st->map, LTC4283_PGIO_CONFIG_2,
				 LTC4283_ADC_MASK,
				 FIELD_PREP(LTC4283_ADC_MASK, 3));
	if (ret)
		return ret;

	/*
	 * Make sure we are integrating power as we only support reporting
	 * consumed energy
	 */
	return regmap_clear_bits(st->map, LTC4283_METER_CONTROL,
				 LTC4283_INTEGRATE_I_MASK);
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

static int ltc4283_hwmon_show_fault_log(void *arg, u64 *val, u32 mask)
{
	struct ltc4283_hwmon *st = arg;
	long alarm;
	int ret;

	ret = ltc4283_hwmon_read_alarm(st, LTC4283_FAULT_LOG, mask, &alarm);
	if (ret)
		return ret;

	*val = alarm;

	return 0;
}

static int ltc4283_hwmon_show_curr1_crit_fault_log(void *arg, u64 *val)
{
	return ltc4283_hwmon_show_fault_log(arg, val, LTC4283_OC_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4283_curr1_crit_fault_log,
			 ltc4283_hwmon_show_curr1_crit_fault_log, NULL, "%llu\n");

static int ltc4283_hwmon_show_in0_lcrit_fault_log(void *arg, u64 *val)
{
	return ltc4283_hwmon_show_fault_log(arg, val, LTC4283_UV_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4283_in0_lcrit_fault_log,
			 ltc4283_hwmon_show_in0_lcrit_fault_log, NULL, "%llu\n");

static int ltc4283_hwmon_show_in0_crit_fault_log(void *arg, u64 *val)
{
	return ltc4283_hwmon_show_fault_log(arg, val, LTC4283_OV_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4283_in0_crit_fault_log,
			 ltc4283_hwmon_show_in0_crit_fault_log, NULL, "%llu\n");

static void ltc4283_debugfs_init(struct ltc4283_hwmon *st, struct device *dev,
				 const struct device *hwmon)
{
	const char *debugfs_name;
	struct dentry *dentry;
	int ret;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_name = devm_kasprintf(dev, GFP_KERNEL, "ltc4283-%s",
				      dev_name(hwmon));
	if (!debugfs_name)
		return;

	dentry = debugfs_create_dir(debugfs_name, NULL);
	if (IS_ERR(dentry))
		return;

	ret = devm_add_action_or_reset(dev, ltc4282_debugfs_remove, dentry);
	if (ret)
		return;

	debugfs_create_file_unsafe("in0_crit_fault_log", 0400, dentry, st,
				   &ltc4283_in0_crit_fault_log);
	debugfs_create_file_unsafe("in0_lcrit_fault_log", 0400, dentry, st,
				   &ltc4283_in0_lcrit_fault_log);
	debugfs_create_file_unsafe("curr1_crit_fault_log", 0400, dentry, st,
				   &ltc4283_curr1_crit_fault_log);

	/* Missing FET/EXTERNAL faults and clarify VOUT/PGI*/
}

static int ltc4283_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev, *hwmon;
	struct ltc4283_hwmon *st;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->gpio_mask = dev_get_drvdata(dev->parent);
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
