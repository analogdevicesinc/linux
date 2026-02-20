// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LTC4284 High Power Negative Voltage Hot Swap Controller
 * with Energy Monitor
 *
 * Copyright 2026 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/property.h>

/* Register addresses */
#define LTC4284_SYSTEM_STATUS		0x00
  #define LTC4284_FET_ON_STATUS_MASK	BIT(0)
  #define LTC4284_GATE1_HIGH_MASK	BIT(1)
  #define LTC4284_GATE2_HIGH_MASK	BIT(2)
  #define LTC4284_PG_STATUS_MASK	BIT(3)
  #define LTC4284_PGIO1_STATUS_MASK	BIT(4)
  #define LTC4284_PGIO2_STATUS_MASK	BIT(5)
  #define LTC4284_PGIO3_STATUS_MASK	BIT(6)
  #define LTC4284_PGIO4_STATUS_MASK	BIT(7)

#define LTC4284_ADC_STATUS		0x01
#define LTC4284_INPUT_STATUS		0x02
#define LTC4284_FAULT_STATUS		0x03
  #define LTC4284_OC1_FAULT_MASK	BIT(0)
  #define LTC4284_OC2_FAULT_MASK	BIT(1)
  #define LTC4284_UV_FAULT_MASK		BIT(2)
  #define LTC4284_OV_FAULT_MASK		BIT(3)
  #define LTC4284_PGI_FAULT_MASK	BIT(4)
  #define LTC4284_VOUT_LOW_FAULT_MASK	BIT(5)
  #define LTC4284_EXT_FAULT_MASK	BIT(6)
  #define LTC4284_FET_BAD_FAULT_MASK	BIT(7)

#define LTC4284_FAULT			0x04
#define LTC4284_ADC_ALARM_LOG		0x05  /* 5-byte register 0x05-0x09 */

#define LTC4284_CONTROL			0x0A  /* 2-byte register 0x0A-0x0B */
  #define LTC4284_ON_MASK		BIT(0)
  #define LTC4284_OC1_RETRY_MASK	BIT(1)
  #define LTC4284_OC2_RETRY_MASK	BIT(2)
  #define LTC4284_UV_RETRY_MASK		BIT(3)
  #define LTC4284_OV_RETRY_MASK		BIT(4)
  #define LTC4284_PGI_RETRY_MASK	BIT(5)
  #define LTC4284_EXT_FAULT_RETRY_MASK	BIT(6)
  #define LTC4284_FET_BAD_RETRY_MASK	BIT(7)
  /* Second byte (0x0B) */
  #define LTC4284_FAULT_LOG_EN_MASK	BIT(8)

#define LTC4284_RESERVED_0C		0x0C

#define LTC4284_CONFIG			0x0D  /* 3-byte register 0x0D-0x0F */
  #define LTC4284_ILIM_MASK		GENMASK(3, 0)  /* Current limit */
  #define LTC4284_MODE_MASK		GENMASK(5, 4)  /* Operation mode */
  #define LTC4284_FB_MASK		GENMASK(7, 6)  /* Current foldback */
  /* Second byte (0x0E) */
  #define LTC4284_COOLING_DL_MASK	GENMASK(11, 9) /* Cooling delay */
  #define LTC4284_FET_BAD_DL_MASK	GENMASK(13, 12) /* FET bad delay */
  /* Third byte (0x0F) */
  #define LTC4284_DVDT_MASK		BIT(16)        /* dV/dt control */

#define LTC4284_PGIO_CONFIG		0x10  /* 2-byte register 0x10-0x11 */
#define LTC4284_ADIO_CONFIG		0x12
#define LTC4284_ADC_SELECT		0x13  /* 2-byte register 0x13-0x14 */

#define LTC4284_FAULT_ALERT		0x15
#define LTC4284_ADC_ALERT		0x16  /* 5-byte register 0x16-0x1A */

/* ADC alarm thresholds */
#define LTC4284_SENSE_MIN_TH		0x1B
#define LTC4284_SENSE_MAX_TH		0x1C
#define LTC4284_VPWR_MIN_TH		0x1D
#define LTC4284_VPWR_MAX_TH		0x1E
#define LTC4284_POWER_MIN_TH		0x1F
#define LTC4284_POWER_MAX_TH		0x20

/* ADC measurement registers - all 2-byte */
#define LTC4284_SENSE			0x41
#define LTC4284_SENSE_MIN		0x42
#define LTC4284_SENSE_MAX		0x43
#define LTC4284_VPWR			0x44
#define LTC4284_VPWR_MIN		0x45
#define LTC4284_VPWR_MAX		0x46
#define LTC4284_POWER			0x47
#define LTC4284_POWER_MIN		0x48
#define LTC4284_POWER_MAX		0x49
#define LTC4284_ENERGY			0x4A  /* 6-byte register */

#define LTC4284_SENSE1			0x68
#define LTC4284_SENSE1_MIN		0x69
#define LTC4284_SENSE1_MAX		0x6A
#define LTC4284_SENSE2			0x6B
#define LTC4284_SENSE2_MIN		0x6C
#define LTC4284_SENSE2_MAX		0x6D

#define LTC4284_ADIN12			0x6E
#define LTC4284_ADIN12_MIN		0x6F
#define LTC4284_ADIN12_MAX		0x70
#define LTC4284_ADIN34			0x71
#define LTC4284_ADIN34_MIN		0x72
#define LTC4284_ADIN34_MAX		0x73

/* Constants */
#define LTC4284_ILIM_MIN	15000   /* 15mV minimum current limit */
#define LTC4284_ILIM_MAX	30000   /* 30mV maximum current limit */
#define LTC4284_ILIM_STEP	1000    /* 1mV step */

#define LTC4284_VIN_MODES	5
#define LTC4284_VIN_3V3		3300000
#define LTC4284_VIN_5V		5000000
#define LTC4284_VIN_12V		12000000
#define LTC4284_VIN_24V		24000000
#define LTC4284_VIN_48V		48000000

enum ltc4284_operation_mode {
	LTC4284_MODE_SINGLE = 0,
	LTC4284_MODE_PARALLEL = 1,
	LTC4284_MODE_STAGED_HIGH_STRESS = 2,
	LTC4284_MODE_STAGED_LOW_STRESS = 3,
};

enum ltc4284_channels {
	LTC4284_CHAN_SENSE,
	LTC4284_CHAN_SENSE1,
	LTC4284_CHAN_SENSE2,
	LTC4284_CHAN_VPWR,
	LTC4284_CHAN_POWER,
	LTC4284_CHAN_ADIN12,
	LTC4284_CHAN_ADIN34,
};

struct ltc4284_state {
	struct regmap *regmap;
	struct mutex lock;
	u32 rsense1;		/* nano-ohms */
	u32 rsense2;		/* nano-ohms */
	u32 vin_mode;		/* microvolts */
	u32 current_limit;	/* microvolts */
	enum ltc4284_operation_mode op_mode;
};

static const u32 ltc4284_vin_modes[] = {
	LTC4284_VIN_3V3,
	LTC4284_VIN_5V,
	LTC4284_VIN_12V,
	LTC4284_VIN_24V,
	LTC4284_VIN_48V,
};

static const struct regmap_config ltc4284_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0xFF,
};

static int ltc4284_read_word_data(struct ltc4284_state *st, u8 reg, u16 *val)
{
	__be16 tmp;
	int ret;

	ret = regmap_bulk_read(st->regmap, reg, &tmp, sizeof(tmp));
	if (ret)
		return ret;

	*val = be16_to_cpu(tmp);
	return 0;
}

static int ltc4284_write_word_data(struct ltc4284_state *st, u8 reg, u16 val)
{
	__be16 tmp = cpu_to_be16(val);

	return regmap_bulk_write(st->regmap, reg, &tmp, sizeof(tmp));
}

static umode_t ltc4284_is_visible(const void *data, enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	const struct ltc4284_state *st = data;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
		case hwmon_in_lowest:
		case hwmon_in_highest:
		case hwmon_in_label:
			return 0444;
		case hwmon_in_min:
		case hwmon_in_max:
			/* Only VPWR channel supports thresholds */
			if (channel == LTC4284_CHAN_VPWR)
				return 0644;
			return 0;
		case hwmon_in_reset_history:
			return 0200;
		}
		break;
	case hwmon_curr:
		if (!st->rsense1 && channel == LTC4284_CHAN_SENSE1)
			return 0;
		if (!st->rsense2 && channel == LTC4284_CHAN_SENSE2)
			return 0;

		switch (attr) {
		case hwmon_curr_input:
		case hwmon_curr_lowest:
		case hwmon_curr_highest:
		case hwmon_curr_label:
			return 0444;
		case hwmon_curr_min:
		case hwmon_curr_max:
			/* Only main SENSE channel supports thresholds */
			if (channel == LTC4284_CHAN_SENSE)
				return 0644;
			return 0;
		case hwmon_curr_reset_history:
			return 0200;
		}
		break;
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
		case hwmon_power_input_lowest:
		case hwmon_power_input_highest:
		case hwmon_power_label:
			return 0444;
		case hwmon_power_min:
		case hwmon_power_max:
			return 0644;
		case hwmon_power_reset_history:
			return 0200;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int ltc4284_read_voltage(struct ltc4284_state *st, u8 reg, long *val)
{
	u16 raw;
	int ret;
	u32 scale;

	ret = ltc4284_read_word_data(st, reg, &raw);
	if (ret)
		return ret;

	switch (st->vin_mode) {
	case LTC4284_VIN_3V3:
		scale = 3300;
		break;
	case LTC4284_VIN_5V:
		scale = 5000;
		break;
	case LTC4284_VIN_12V:
		scale = 12000;
		break;
	case LTC4284_VIN_24V:
		scale = 24000;
		break;
	case LTC4284_VIN_48V:
		scale = 48000;
		break;
	default:
		return -EINVAL;
	}

	*val = DIV_ROUND_CLOSEST_ULL((u64)raw * scale, 65536);

	return 0;
}

static int ltc4284_read_current(struct ltc4284_state *st, u8 reg, u32 rsense, long *val)
{
	u16 raw;
	int ret;
	u64 tmp;

	if (!rsense)
		return -EINVAL;

	ret = ltc4284_read_word_data(st, reg, &raw);
	if (ret)
		return ret;

	/*
	 * Units: (µV / nΩ) = kA, so multiply by 1000 to get mA.
	 * Multiply before division to preserve precision.
	 */
	tmp = mul_u64_u32_div(raw, st->current_limit, rsense);
	*val = DIV_ROUND_CLOSEST_ULL(tmp * 1000, 65536);

	return 0;
}

static int ltc4284_read_power(struct ltc4284_state *st, u8 reg, long *val)
{
	u16 raw;
	int ret;
	u64 tmp;

	ret = ltc4284_read_word_data(st, reg, &raw);
	if (ret)
		return ret;

	/* Full scale: 32.768mV × 2.048V = 67.108864mW = 1024 µW/LSB */
	tmp = (u64)raw * 1024;
	*val = tmp;

	return 0;
}

static int ltc4284_write_voltage(struct ltc4284_state *st, u8 reg, long val, u32 scale)
{
	u16 raw;

	if (val < 0)
		val = 0;
	else if (val > scale)
		val = scale;

	raw = DIV_ROUND_CLOSEST_ULL((u64)val * 65536, scale);

	return ltc4284_write_word_data(st, reg, raw);
}

static int ltc4284_write_current(struct ltc4284_state *st, u8 reg, long val, u32 rsense)
{
	u16 raw;
	u64 vsense_nv;

	if (!rsense)
		return -EINVAL;

	if (val < 0)
		val = 0;

	/*
	 * mA × nΩ gives voltage in our integer units.
	 * Scale by current_limit (µV): µV = 1000 nV.
	 */
	vsense_nv = (u64)val * rsense;
	raw = DIV_ROUND_CLOSEST_ULL(vsense_nv * 65536,
				    (u64)st->current_limit * 1000);

	return ltc4284_write_word_data(st, reg, raw);
}

static int ltc4284_write_power(struct ltc4284_state *st, u8 reg, long val)
{
	u16 raw;

	if (val < 0)
		val = 0;

	raw = DIV_ROUND_CLOSEST_ULL((u64)val, 1024);

	return ltc4284_write_word_data(st, reg, raw);
}


static int ltc4284_reset_history(struct ltc4284_state *st, enum hwmon_sensor_types type,
				 int channel)
{
	u8 reg_min, reg_max;
	u16 reset_val = (type == hwmon_energy) ? 0 : 0xFFFF;
	int ret = 0;

	guard(mutex)(&st->lock);

	switch (type) {
	case hwmon_in:
		switch (channel) {
		case LTC4284_CHAN_VPWR:
			reg_min = LTC4284_VPWR_MIN;
			reg_max = LTC4284_VPWR_MAX;
			break;
		case LTC4284_CHAN_ADIN12:
			reg_min = LTC4284_ADIN12_MIN;
			reg_max = LTC4284_ADIN12_MAX;
			break;
		case LTC4284_CHAN_ADIN34:
			reg_min = LTC4284_ADIN34_MIN;
			reg_max = LTC4284_ADIN34_MAX;
			break;
		default:
			return -EINVAL;
		}
		break;
	case hwmon_curr:
		switch (channel) {
		case LTC4284_CHAN_SENSE:
			reg_min = LTC4284_SENSE_MIN;
			reg_max = LTC4284_SENSE_MAX;
			break;
		case LTC4284_CHAN_SENSE1:
			reg_min = LTC4284_SENSE1_MIN;
			reg_max = LTC4284_SENSE1_MAX;
			break;
		case LTC4284_CHAN_SENSE2:
			reg_min = LTC4284_SENSE2_MIN;
			reg_max = LTC4284_SENSE2_MAX;
			break;
		default:
			return -EINVAL;
		}
		break;
	case hwmon_power:
		reg_min = LTC4284_POWER_MIN;
		reg_max = LTC4284_POWER_MAX;
		break;
	default:
		return -EINVAL;
	}

	/* Reset min/max registers */
	ret = ltc4284_write_word_data(st, reg_min, reset_val);
	if (ret)
		return ret;

	ret = ltc4284_write_word_data(st, reg_max, 0);
	return ret;
}

static int ltc4284_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct ltc4284_state *st = dev_get_drvdata(dev);
	u8 reg;
	u32 rsense;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				reg = LTC4284_VPWR;
				break;
			case LTC4284_CHAN_ADIN12:
				reg = LTC4284_ADIN12;
				break;
			case LTC4284_CHAN_ADIN34:
				reg = LTC4284_ADIN34;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_voltage(st, reg, val);
		case hwmon_in_lowest:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				reg = LTC4284_VPWR_MIN;
				break;
			case LTC4284_CHAN_ADIN12:
				reg = LTC4284_ADIN12_MIN;
				break;
			case LTC4284_CHAN_ADIN34:
				reg = LTC4284_ADIN34_MIN;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_voltage(st, reg, val);
		case hwmon_in_highest:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				reg = LTC4284_VPWR_MAX;
				break;
			case LTC4284_CHAN_ADIN12:
				reg = LTC4284_ADIN12_MAX;
				break;
			case LTC4284_CHAN_ADIN34:
				reg = LTC4284_ADIN34_MAX;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_voltage(st, reg, val);
		case hwmon_in_min:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				reg = LTC4284_VPWR_MIN_TH;
				break;
			default:
				return -EOPNOTSUPP;
			}
			return ltc4284_read_voltage(st, reg, val);
		case hwmon_in_max:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				reg = LTC4284_VPWR_MAX_TH;
				break;
			default:
				return -EOPNOTSUPP;
			}
			return ltc4284_read_voltage(st, reg, val);
		}
		break;
	case hwmon_curr:
		switch (channel) {
		case LTC4284_CHAN_SENSE:
			rsense = st->rsense1; /* Use rsense1 for main sense */
			break;
		case LTC4284_CHAN_SENSE1:
			rsense = st->rsense1;
			break;
		case LTC4284_CHAN_SENSE2:
			rsense = st->rsense2;
			break;
		default:
			return -EINVAL;
		}

		switch (attr) {
		case hwmon_curr_input:
			switch (channel) {
			case LTC4284_CHAN_SENSE:
				reg = LTC4284_SENSE;
				break;
			case LTC4284_CHAN_SENSE1:
				reg = LTC4284_SENSE1;
				break;
			case LTC4284_CHAN_SENSE2:
				reg = LTC4284_SENSE2;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_current(st, reg, rsense, val);
		case hwmon_curr_lowest:
			switch (channel) {
			case LTC4284_CHAN_SENSE:
				reg = LTC4284_SENSE_MIN;
				break;
			case LTC4284_CHAN_SENSE1:
				reg = LTC4284_SENSE1_MIN;
				break;
			case LTC4284_CHAN_SENSE2:
				reg = LTC4284_SENSE2_MIN;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_current(st, reg, rsense, val);
		case hwmon_curr_highest:
			switch (channel) {
			case LTC4284_CHAN_SENSE:
				reg = LTC4284_SENSE_MAX;
				break;
			case LTC4284_CHAN_SENSE1:
				reg = LTC4284_SENSE1_MAX;
				break;
			case LTC4284_CHAN_SENSE2:
				reg = LTC4284_SENSE2_MAX;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_current(st, reg, rsense, val);
		case hwmon_curr_min:
			if (channel != LTC4284_CHAN_SENSE)
				return -EOPNOTSUPP;
			reg = LTC4284_SENSE_MIN_TH;
			return ltc4284_read_current(st, reg, rsense, val);
		case hwmon_curr_max:
			if (channel != LTC4284_CHAN_SENSE)
				return -EOPNOTSUPP;
			reg = LTC4284_SENSE_MAX_TH;
			return ltc4284_read_current(st, reg, rsense, val);
		}
		break;
	case hwmon_power:
		switch (attr) {
		case hwmon_power_input:
			reg = LTC4284_POWER;
			break;
		case hwmon_power_input_lowest:
			reg = LTC4284_POWER_MIN;
			break;
		case hwmon_power_input_highest:
			reg = LTC4284_POWER_MAX;
			break;
		case hwmon_power_min:
			reg = LTC4284_POWER_MIN_TH;
			break;
		case hwmon_power_max:
			reg = LTC4284_POWER_MAX_TH;
			break;
		default:
			return -EINVAL;
		}
		return ltc4284_read_power(st, reg, val);
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int ltc4284_write(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long val)
{
	struct ltc4284_state *st = dev_get_drvdata(dev);
	u8 reg;
	u32 rsense, scale;

	switch (type) {
	case hwmon_in:
		if (attr == hwmon_in_reset_history)
			return ltc4284_reset_history(st, type, channel);

		switch (channel) {
		case LTC4284_CHAN_VPWR:
			scale = st->vin_mode / 1000;	/* Convert to mV */
			break;
		default:
			return -EOPNOTSUPP;
		}

		switch (attr) {
		case hwmon_in_min:
			reg = LTC4284_VPWR_MIN_TH;
			break;
		case hwmon_in_max:
			reg = LTC4284_VPWR_MAX_TH;
			break;
		default:
			return -EOPNOTSUPP;
		}
		return ltc4284_write_voltage(st, reg, val, scale);

	case hwmon_curr:
		if (attr == hwmon_curr_reset_history)
			return ltc4284_reset_history(st, type, channel);

		if (channel != LTC4284_CHAN_SENSE)
			return -EOPNOTSUPP;

		rsense = st->rsense1;
		switch (attr) {
		case hwmon_curr_min:
			reg = LTC4284_SENSE_MIN_TH;
			break;
		case hwmon_curr_max:
			reg = LTC4284_SENSE_MAX_TH;
			break;
		default:
			return -EOPNOTSUPP;
		}
		return ltc4284_write_current(st, reg, val, rsense);

	case hwmon_power:
		if (attr == hwmon_power_reset_history)
			return ltc4284_reset_history(st, type, channel);

		switch (attr) {
		case hwmon_power_min:
			reg = LTC4284_POWER_MIN_TH;
			break;
		case hwmon_power_max:
			reg = LTC4284_POWER_MAX_TH;
			break;
		default:
			return -EOPNOTSUPP;
		}
		return ltc4284_write_power(st, reg, val);

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_channel_info * const ltc4284_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* VPWR */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL,  /* ADIN12 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL), /* ADIN34 */
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_MIN | HWMON_C_MAX | HWMON_C_RESET_HISTORY |
			   HWMON_C_LABEL,  /* SENSE */
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_RESET_HISTORY | HWMON_C_LABEL,  /* SENSE1 */
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_RESET_HISTORY | HWMON_C_LABEL), /* SENSE2 */
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_INPUT_LOWEST |
			   HWMON_P_INPUT_HIGHEST | HWMON_P_MIN | HWMON_P_MAX |
			   HWMON_P_RESET_HISTORY | HWMON_P_LABEL),
	NULL
};

static const char * const ltc4284_in_labels[] = {
	"VPWR", "ADIN12", "ADIN34"
};

static const char * const ltc4284_curr_labels[] = {
	"SENSE", "SENSE1", "SENSE2"
};

static int ltc4284_read_labels(struct device *dev,
			       enum hwmon_sensor_types type,
			       u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_in:
		if (channel < ARRAY_SIZE(ltc4284_in_labels)) {
			*str = ltc4284_in_labels[channel];
			return 0;
		}
		break;
	case hwmon_curr:
		if (channel < ARRAY_SIZE(ltc4284_curr_labels)) {
			*str = ltc4284_curr_labels[channel];
			return 0;
		}
		break;
	case hwmon_power:
		*str = "Power";
		return 0;
	default:
		break;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops ltc4284_hwmon_ops = {
	.is_visible = ltc4284_is_visible,
	.read = ltc4284_read,
	.write = ltc4284_write,
	.read_string = ltc4284_read_labels,
};

static const struct hwmon_chip_info ltc4284_chip_info = {
	.ops = &ltc4284_hwmon_ops,
	.info = ltc4284_info,
};

static int ltc4284_init(struct ltc4284_state *st)
{
	int ret;
	u32 val;

	/* Configure current limit */
	if (st->current_limit < LTC4284_ILIM_MIN ||
	    st->current_limit > LTC4284_ILIM_MAX) {
		dev_err(regmap_get_device(st->regmap),
			"Invalid current limit %u µV (min: %u, max: %u)\n",
			st->current_limit, LTC4284_ILIM_MIN, LTC4284_ILIM_MAX);
		return -EINVAL;
	}

	/* Set current limit - convert from µV to register value */
	val = (st->current_limit - LTC4284_ILIM_MIN) / LTC4284_ILIM_STEP;
	ret = regmap_update_bits(st->regmap, LTC4284_CONFIG,
				 LTC4284_ILIM_MASK, val);
	if (ret)
		return ret;

	/* Set operation mode */
	ret = regmap_update_bits(st->regmap, LTC4284_CONFIG,
				 LTC4284_MODE_MASK,
				 FIELD_PREP(LTC4284_MODE_MASK, st->op_mode));
	if (ret)
		return ret;

	/* Enable fault logging by default */
	ret = regmap_update_bits(st->regmap, LTC4284_CONTROL,
				 LTC4284_FAULT_LOG_EN_MASK,
				 LTC4284_FAULT_LOG_EN_MASK);
	return ret;
}

static int ltc4284_parse_dt(struct ltc4284_state *st, struct device *dev)
{
	u32 val;
	const char *mode_str;
	int ret, i;

	/* Required: first current sense resistor */
	ret = device_property_read_u32(dev, "adi,rsense1-nano-ohms", &st->rsense1);
	if (ret) {
		dev_err(dev, "Missing required adi,rsense1-nano-ohms property\n");
		return ret;
	}

	/* Optional: second current sense resistor */
	ret = device_property_read_u32(dev, "adi,rsense2-nano-ohms",
				       &st->rsense2);
	if (ret)
		st->rsense2 = 0; /* Not used */

	/* VIN mode - default to 48V for -48V systems */
	ret = device_property_read_u32(dev, "adi,vin-mode-microvolt", &val);
	if (ret)
		val = LTC4284_VIN_48V;

	for (i = 0; i < ARRAY_SIZE(ltc4284_vin_modes); i++) {
		if (ltc4284_vin_modes[i] == val) {
			st->vin_mode = val;
			break;
		}
	}
	if (i == ARRAY_SIZE(ltc4284_vin_modes)) {
		dev_err(dev, "Invalid vin-mode-microvolt %u\n", val);
		return -EINVAL;
	}

	/* Current limit - default to 25mV */
	ret = device_property_read_u32(dev,
				       "adi,current-limit-sense-microvolt",
				       &val);
	if (ret)
		val = 25000;
	st->current_limit = val;

	/* Operation mode - default to parallel */
	ret = device_property_read_string(dev, "adi,operation-mode", &mode_str);
	if (ret) {
		st->op_mode = LTC4284_MODE_PARALLEL;
	} else {
		if (strcmp(mode_str, "single") == 0)
			st->op_mode = LTC4284_MODE_SINGLE;
		else if (strcmp(mode_str, "parallel") == 0)
			st->op_mode = LTC4284_MODE_PARALLEL;
		else if (strcmp(mode_str, "staged-high-stress") == 0)
			st->op_mode = LTC4284_MODE_STAGED_HIGH_STRESS;
		else if (strcmp(mode_str, "staged-low-stress") == 0)
			st->op_mode = LTC4284_MODE_STAGED_LOW_STRESS;
		else {
			dev_err(dev, "Invalid operation-mode: %s\n", mode_str);
			return -EINVAL;
		}
	}

	return 0;
}

static int ltc4284_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ltc4284_state *st;
	struct device *hwmon_dev;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	mutex_init(&st->lock);

	st->regmap = devm_regmap_init_i2c(client, &ltc4284_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	ret = ltc4284_parse_dt(st, dev);
	if (ret)
		return ret;

	ret = ltc4284_init(st);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize device\n");

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "ltc4284", st,
							 &ltc4284_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	return 0;
}

static const struct of_device_id ltc4284_of_match[] = {
	{ .compatible = "adi,ltc4284" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc4284_of_match);

static const struct i2c_device_id ltc4284_id[] = {
	{ "ltc4284", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc4284_id);

static struct i2c_driver ltc4284_driver = {
	.driver = {
		.name = "ltc4284",
		.of_match_table = ltc4284_of_match,
	},
	.probe = ltc4284_probe,
	.id_table = ltc4284_id,
};

module_i2c_driver(ltc4284_driver);

MODULE_AUTHOR("Carlos Jones Jr <carlosjr.jones@analog.com>");
MODULE_DESCRIPTION("LTC4284 High Power Negative Voltage Hot Swap Controller");
MODULE_LICENSE("GPL");
