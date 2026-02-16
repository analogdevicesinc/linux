// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices LTC4284 High Power Negative Voltage Hot Swap Controller
 * with Energy Monitor
 *
 * Copyright 2026 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/property.h>
#include <linux/units.h>

#define LTC4284_SYSTEM_STATUS		0x00
  #define LTC4284_MODE1_MASK		BIT(0)
  #define LTC4284_PG_STATUS_MASK	BIT(1)
  #define LTC4284_EEPROM_BUSY_MASK	BIT(2)
  #define LTC4284_TMR_LOW_MASK		BIT(3)
  #define LTC4284_GATE1_HIGH_MASK	BIT(4)
  #define LTC4284_GATE2_HIGH_MASK	BIT(5)
  #define LTC4284_EN_MASK		BIT(6)
  #define LTC4284_FET_ON_STATUS_MASK	BIT(7)

#define LTC4284_ADC_STATUS		0x01
#define LTC4284_INPUT_STATUS		0x02
#define LTC4284_FAULT_STATUS		0x03
  #define LTC4284_OV_STATUS_MASK	BIT(0)
  #define LTC4284_UV_STATUS_MASK	BIT(1)
  #define LTC4284_OC_STATUS_MASK	BIT(2)
  #define LTC4284_FET_BAD_STATUS_MASK	BIT(3)
  #define LTC4284_PGI_STATUS_MASK	BIT(4)
  #define LTC4284_VOUT_LOW_STATUS_MASK	BIT(5)
  #define LTC4284_FET_SHORT_STATUS_MASK	BIT(6)
  #define LTC4284_EXT_FAULT_STATUS_MASK	BIT(7)

#define LTC4284_FAULT			0x04
  #define LTC4284_OV_FAULT_MASK		BIT(0)
  #define LTC4284_UV_FAULT_MASK		BIT(1)
  #define LTC4284_OC_FAULT_MASK		BIT(2)
  #define LTC4284_FET_BAD_FAULT_MASK	BIT(3)
  #define LTC4284_PGI_FAULT_MASK	BIT(4)
  #define LTC4284_POWER_FAILED_MASK	BIT(5)
  #define LTC4284_FET_SHORT_FAULT_MASK	BIT(6)
  #define LTC4284_EXT_FAULT_MASK	BIT(7)
/* 5-byte register 0x05-0x09 */
#define LTC4284_ADC_ALARM_LOG		0x05

/* 2-byte register 0x0A-0x0B */
#define LTC4284_CONTROL			0x0A
  /* First byte (0x0A) - CONTROL_1 */
  #define LTC4284_PAGE_RW_EN_MASK	BIT(0)
  #define LTC4284_MASS_WRITE_EN_MASK	BIT(1)
  #define LTC4284_PGIO2_ACLB_MASK	BIT(2)
  #define LTC4284_PWRGD_RESET_MASK	BIT(3)
  #define LTC4284_FET_BAD_TURN_OFF_MASK	BIT(4)
  #define LTC4284_THERM_TMR_MASK	BIT(5)
  #define LTC4284_DVDT_MASK		BIT(6)
  #define LTC4284_ON_MASK		BIT(7)
  /* Second byte (0x0B) - CONTROL_2 */
  #define LTC4284_OV_RETRY_MASK		BIT(8)
  #define LTC4284_UV_RETRY_MASK		BIT(9)
  #define LTC4284_OC_RETRY_MASK		GENMASK(11, 10)
  #define LTC4284_FET_BAD_RETRY_MASK	GENMASK(13, 12)
  #define LTC4284_PGI_RETRY_MASK	BIT(14)
  #define LTC4284_EXT_FAULT_RETRY_MASK	BIT(15)

/* 3-byte register 0x0D-0x0F */
#define LTC4284_CONFIG			0x0D
  /* First byte (0x0D) - CONFIG_1 */
  #define LTC4284_LPFB_MASK		BIT(0)
  #define LTC4284_FB_DIS_MASK		BIT(1)
  #define LTC4284_FB_MASK		GENMASK(3, 2)	/* Current foldback */
  #define LTC4284_ILIM_MASK		GENMASK(7, 4)	/* Current limit */
  /* Second byte (0x0E) - CONFIG_2 */
  #define LTC4284_PORB_MASK		BIT(8)
  #define LTC4284_COOLING_DL_MASK	GENMASK(11, 9)	/* Cooling delay */
  #define LTC4284_FET_BAD_DL_MASK	GENMASK(13, 12) /* FET bad delay */
  #define LTC4284_VDTH_MASK		GENMASK(15, 14)
  /* Third byte (0x0F) - CONFIG_3 */
  #define LTC4284_INTEGRATE_I_MASK	BIT(16)
  #define LTC4284_METER_OVERFLOW_ALERT_MASK BIT(17)
  #define LTC4284_TICK_OVERFLOW_ALERT_MASK BIT(18)
  #define LTC4284_BC_MASK		GENMASK(20, 19)
  #define LTC4284_FAST_I2C_EN_MASK	BIT(21)
  #define LTC4284_VPWR_SELECT_MASK	BIT(22)
  #define LTC4284_EXTFLT_TURN_OFF_MASK	BIT(23)

#define LTC4284_PGIO_CONFIG		0x10	/* 2-byte register 0x10-0x11 */
#define LTC4284_ADIO_CONFIG		0x12
#define LTC4284_ADC_SELECT		0x13	/* 2-byte register 0x13-0x14 */

#define LTC4284_FAULT_ALERT		0x15
#define LTC4284_ADC_ALERT		0x16	/* 5-byte register 0x16-0x1A */

/* ADC alarm thresholds */
#define LTC4284_SENSE_MIN_TH		0x1B
#define LTC4284_SENSE_MAX_TH		0x1C
#define LTC4284_VPWR_MIN_TH		0x1D
#define LTC4284_VPWR_MAX_TH		0x1E
#define LTC4284_POWER_MIN_TH		0x1F
#define LTC4284_POWER_MAX_TH		0x20

/* Threshold registers - all 8-bit */
#define LTC4284_ADIN1_MIN_TH		0x21
#define LTC4284_ADIN1_MAX_TH		0x22
#define LTC4284_ADIN2_MIN_TH		0x23
#define LTC4284_ADIN2_MAX_TH		0x24
#define LTC4284_ADIN3_MIN_TH		0x25
#define LTC4284_ADIN3_MAX_TH		0x26
#define LTC4284_ADIN4_MIN_TH		0x27
#define LTC4284_ADIN4_MAX_TH		0x28
#define LTC4284_ADIO1_MIN_TH		0x29
#define LTC4284_ADIO1_MAX_TH		0x2A
#define LTC4284_ADIO2_MIN_TH		0x2B
#define LTC4284_ADIO2_MAX_TH		0x2C
#define LTC4284_ADIO3_MIN_TH		0x2D
#define LTC4284_ADIO3_MAX_TH		0x2E
#define LTC4284_ADIO4_MIN_TH		0x2F
#define LTC4284_ADIO4_MAX_TH		0x30
#define LTC4284_SENSE1_MIN_TH		0x35
#define LTC4284_SENSE1_MAX_TH		0x36
#define LTC4284_SENSE2_MIN_TH		0x37
#define LTC4284_SENSE2_MAX_TH		0x38
#define LTC4284_ADIN12_MIN_TH		0x39
#define LTC4284_ADIN12_MAX_TH		0x3A
#define LTC4284_ADIN34_MIN_TH		0x3B
#define LTC4284_ADIN34_MAX_TH		0x3C
#define LTC4284_ADIO12_MIN_TH		0x3D
#define LTC4284_ADIO12_MAX_TH		0x3E
#define LTC4284_ADIO34_MIN_TH		0x3F
#define LTC4284_ADIO34_MAX_TH		0x40

/* ADC measurement registers - all 2-bytes */
#define LTC4284_SENSE			0x41
#define LTC4284_SENSE_MIN		0x42
#define LTC4284_SENSE_MAX		0x43
#define LTC4284_VPWR			0x44
#define LTC4284_VPWR_MIN		0x45
#define LTC4284_VPWR_MAX		0x46
#define LTC4284_POWER			0x47
#define LTC4284_POWER_MIN		0x48
#define LTC4284_POWER_MAX		0x49
#define LTC4284_ADIN1			0x4A
#define LTC4284_ADIN1_MIN		0x4B
#define LTC4284_ADIN1_MAX		0x4C
#define LTC4284_ADIN2			0x4D
#define LTC4284_ADIN2_MIN		0x4E
#define LTC4284_ADIN2_MAX		0x4F
#define LTC4284_ADIN3			0x50
#define LTC4284_ADIN3_MIN		0x51
#define LTC4284_ADIN3_MAX		0x52
#define LTC4284_ADIN4			0x53
#define LTC4284_ADIN4_MIN		0x54
#define LTC4284_ADIN4_MAX		0x55
#define LTC4284_ADIO1			0x56
#define LTC4284_ADIO1_MIN		0x57
#define LTC4284_ADIO1_MAX		0x58
#define LTC4284_ADIO2			0x59
#define LTC4284_ADIO2_MIN		0x5A
#define LTC4284_ADIO2_MAX		0x5B
#define LTC4284_ADIO3			0x5C
#define LTC4284_ADIO3_MIN		0x5D
#define LTC4284_ADIO3_MAX		0x5E
#define LTC4284_ADIO4			0x5F
#define LTC4284_ADIO4_MIN		0x60
#define LTC4284_ADIO4_MAX		0x61
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
#define LTC4284_ADIO12			0x74
#define LTC4284_ADIO12_MIN		0x75
#define LTC4284_ADIO12_MAX		0x76
#define LTC4284_ADIO34			0x77
#define LTC4284_ADIO34_MIN		0x78
#define LTC4284_ADIO34_MAX		0x79

/* 6-byte register */
#define LTC4284_ENERGY			0x7A

/* Constants */
#define LTC4284_ILIM_MIN	15000   /* 15mV minimum current sense threshold */
#define LTC4284_ILIM_MAX	30000   /* 30mV maximum current sense threshold */
#define LTC4284_ILIM_STEP	1000    /* 1mV step */

/* ADC full-scale voltages from datasheet */
#define LTC4284_FS_SENSE_UV	32768  /* 32.768mV full-scale for SENSE (uv) */
#define LTC4284_FS_AUX		2048   /* 2.048V full-scale for ADIN/ADIO (mv) */

/* 13 Voltage channel indices (hwmon_in) */
enum ltc4284_in_channels {
	LTC4284_CHAN_VPWR = 0,
	LTC4284_CHAN_ADIN12,
	LTC4284_CHAN_ADIN34,
	LTC4284_CHAN_ADIN1,
	LTC4284_CHAN_ADIN2,
	LTC4284_CHAN_ADIN3,
	LTC4284_CHAN_ADIN4,
	LTC4284_CHAN_ADIO12,
	LTC4284_CHAN_ADIO34,
	LTC4284_CHAN_ADIO1,
	LTC4284_CHAN_ADIO2,
	LTC4284_CHAN_ADIO3,
	LTC4284_CHAN_ADIO4,
};

/* 3 Current channel indices (hwmon_curr) */
enum ltc4284_curr_channels {
	LTC4284_CHAN_SENSE = 0,
	LTC4284_CHAN_SENSE1,
	LTC4284_CHAN_SENSE2,
};

struct ltc4284_state {
	struct regmap *regmap;
	/* Protects access to device registers */
	struct mutex lock;
	u32 rsense1;		/* nano-ohms */
	u32 rsense2;		/* nano-ohms */
	u32 current_limit;	/* microvolts */
	/* All voltage scales are in millivolts */
	u32 scale_vpwr;		/* VPWR input scale */
	u32 scale_adin[4];	/* ADIN1-4 scales */
	u32 scale_adio[4];	/* ADIO1-4 scales */
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

static int ltc4284_write_energy(struct ltc4284_state *st, u64 val)
{
	u8 buf[6];

	/* Energy register is 48-bit big-endian */
	buf[0] = (val >> 40) & 0xFF;
	buf[1] = (val >> 32) & 0xFF;
	buf[2] = (val >> 24) & 0xFF;
	buf[3] = (val >> 16) & 0xFF;
	buf[4] = (val >> 8) & 0xFF;
	buf[5] = val & 0xFF;

	return regmap_bulk_write(st->regmap, LTC4284_ENERGY, buf, sizeof(buf));
}

static int ltc4284_read_alarm(struct ltc4284_state *st, u8 reg, u32 mask, long *val)
{
	unsigned int alarm;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, reg, &alarm);
	if (ret)
		return ret;

	*val = !!(alarm & mask);
	return 0;
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
			return 0644;
		case hwmon_in_crit_alarm:
		case hwmon_in_lcrit_alarm:
			/* Only VPWR has OV/UV alarms */
			if (channel == LTC4284_CHAN_VPWR)
				return 0444;
			return 0;
		case hwmon_in_reset_history:
			return 0200;
		default:
			return 0;
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
			return 0644;
		case hwmon_curr_crit_alarm:
			/* Only main SENSE has OC alarm */
			if (channel == LTC4284_CHAN_SENSE)
				return 0444;
			return 0;
		case hwmon_curr_reset_history:
			return 0200;
		default:
			return 0;
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
		default:
			return 0;
		}
		break;
	case hwmon_energy:
		switch (attr) {
		case hwmon_energy_input:
		case hwmon_energy_label:
			return 0444;
		case hwmon_energy_enable:
			return 0200;
		default:
			return 0;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int ltc4284_get_voltage_scale(struct ltc4284_state *st, int channel, u32 *scale)
{
	switch (channel) {
	case LTC4284_CHAN_VPWR:
		*scale = st->scale_vpwr;
		break;
	case LTC4284_CHAN_ADIN1:
		*scale = st->scale_adin[0];
		break;
	case LTC4284_CHAN_ADIN2:
		*scale = st->scale_adin[1];
		break;
	case LTC4284_CHAN_ADIN3:
		*scale = st->scale_adin[2];
		break;
	case LTC4284_CHAN_ADIN4:
		*scale = st->scale_adin[3];
		break;
	case LTC4284_CHAN_ADIN12:
		/* Differential: 32.768mV full-scale */
		*scale = LTC4284_FS_SENSE_UV;
		break;
	case LTC4284_CHAN_ADIN34:
		/* Differential: 32.768mV full-scale */
		*scale = LTC4284_FS_SENSE_UV;
		break;
	case LTC4284_CHAN_ADIO1:
		*scale = st->scale_adio[0];
		break;
	case LTC4284_CHAN_ADIO2:
		*scale = st->scale_adio[1];
		break;
	case LTC4284_CHAN_ADIO3:
		*scale = st->scale_adio[2];
		break;
	case LTC4284_CHAN_ADIO4:
		*scale = st->scale_adio[3];
		break;
	case LTC4284_CHAN_ADIO12:
		/* Differential: 32.768mV full-scale */
		*scale = LTC4284_FS_SENSE_UV;
		break;
	case LTC4284_CHAN_ADIO34:
		/* Differential: 32.768mV full-scale */
		*scale = LTC4284_FS_SENSE_UV;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int ltc4284_read_voltage(struct ltc4284_state *st, u8 reg, int channel, long *val)
{
	unsigned int reg_val;
	u16 raw;
	int ret;
	u32 scale;

	ret = ltc4284_get_voltage_scale(st, channel, &scale);
	if (ret)
		return ret;

	/* Threshold registers are 8-bit (0x1B-0x40), measurement registers are 16-bit */
	if (reg >= LTC4284_SENSE_MIN_TH && reg <= LTC4284_ADIO34_MAX_TH) {
		ret = regmap_read(st->regmap, reg, &reg_val);
		if (ret)
			return ret;
		/* Convert 8-bit to 16-bit equivalent */
		raw = reg_val * 255;
	} else {
		ret = ltc4284_read_word_data(st, reg, &raw);
		if (ret)
			return ret;
	}

	/* Differential channels uses microvolts */
	if (channel == LTC4284_CHAN_ADIN12 || channel == LTC4284_CHAN_ADIN34 ||
	    channel == LTC4284_CHAN_ADIO12 || channel == LTC4284_CHAN_ADIO34)
		*val = DIV_ROUND_CLOSEST_ULL((u64)raw * scale, 65536ULL * MILLI);
	else
		*val = DIV_ROUND_CLOSEST_ULL((u64)raw * scale, 65536);

	return 0;
}

static int ltc4284_read_current(struct ltc4284_state *st, u8 reg, u32 rsense, long *val)
{
	unsigned int reg_val;
	u16 raw;
	int ret;

	if (!rsense)
		return -EINVAL;

	/* Threshold registers are 8-bit */
	if (reg == LTC4284_SENSE_MIN_TH || reg == LTC4284_SENSE_MAX_TH ||
	    reg == LTC4284_SENSE1_MIN_TH || reg == LTC4284_SENSE1_MAX_TH ||
	    reg == LTC4284_SENSE2_MIN_TH || reg == LTC4284_SENSE2_MAX_TH) {
		ret = regmap_read(st->regmap, reg, &reg_val);
		if (ret)
			return ret;
		/* Convert 8-bit to 16-bit equivalent */
		raw = reg_val * 255;
	} else {
		ret = ltc4284_read_word_data(st, reg, &raw);
		if (ret)
			return ret;
	}

	/*
	 * Current (mA) = Voltage (µV) / Resistance (nΩ) × 1,000,000
	 *              = (raw / 65536) × 32768µV / rsense × MEGA
	 * SENSE ADC has fixed 32.768mV full-scale (not configurable by current_limit)
	 * current_limit only sets the overcurrent protection threshold
	 */
	*val = DIV64_U64_ROUND_CLOSEST((u64)raw * LTC4284_FS_SENSE_UV * MEGA,
				       (u64)rsense * 65536);
	return 0;
}

static int ltc4284_read_power(struct ltc4284_state *st, u8 reg, long *val)
{
	unsigned int reg_val;
	u16 raw;
	int ret;
	u64 tmp;

	/*
	 * Chip measures: P_chip = V_sense × V_vpwr_at_pin
	 * Full scale at chip pins: 32.768mV × 2.048V = 67.108864mW
	 * This gives: 1024 nW/LSB @ 1Ω sense resistor (16bits)
	 * or 1024 * 255 = 261120 nW/LSB @ 1Ω sense resistor (8bits)
	 *
	 * Actual power accounts for external VPWR divider:
	 * P_actual = P_chip × (scale_vpwr / LTC4284_FS_AUX) / Rsense
	 *
	 * Example: With 40:1 divider and 1Ω sense resistor:
	 *   P_actual = raw × 1024nW × 40
	 */

	/* Threshold registers are 8-bit, measurement registers are 16-bit */
	if (reg == LTC4284_POWER_MIN_TH || reg == LTC4284_POWER_MAX_TH) {
		ret = regmap_read(st->regmap, reg, &reg_val);
		if (ret)
			return ret;
		 /* Convert to 16-bit equivalent */
		raw = reg_val * 255;
	} else {
		ret = ltc4284_read_word_data(st, reg, &raw);
		if (ret)
			return ret;
	}

	/* Base power: raw × 1024nW × (1Ω / Rsense), convert to µW */
	tmp = mul_u64_u32_div((u64)raw * 1024, MEGA, st->rsense1);

	/* Scale by VPWR divider ratio */
	*val = DIV_ROUND_CLOSEST_ULL(tmp * st->scale_vpwr, LTC4284_FS_AUX);

	return 0;
}

static int ltc4284_read_energy(struct ltc4284_state *st, long *val)
{
	u64 raw, tmp;
	__be64 energy_raw;
	int ret;

	/* Read 48-bit energy register */
	ret = regmap_bulk_read(st->regmap, LTC4284_ENERGY, &energy_raw, 6);
	if (ret)
		return ret;

	raw = be64_to_cpu(energy_raw) >> 16;

	/*
	 * From datasheet: E = CODE × 32.768mV × 2.048V × tCONV / 2^24 / RSENSE
	 *
	 * Where:
	 * - CODE: 48-bit energy accumulator value
	 * - 32.768mV: SENSE full-scale voltage
	 * - 2.048V: VPWR full-scale voltage (at chip pin)
	 * - tCONV: ADC conversion period = 1/fCONV = 1/15.6 Hz ≈ 64.1ms
	 * - 2^24: scaling factor = 16777216
	 * - RSENSE: sense resistor
	 *
	 * Simplifying with tCONV = 1/15.6 s:
	 * E [J] = CODE × 0.032768 × 2.048 × (1/15.6) / 16777216 / RSENSE_ohms
	 * E [J] = CODE × 0.067108864 / 15.6 / 16777216 / RSENSE_ohms
	 * E [J] = CODE × 0.00430313 / 16777216 / RSENSE_ohms
	 * E [J] = CODE × 2.565e-10 / RSENSE_ohms
	 *
	 * Converting to µJ and RSENSE in nΩ:
	 * E [µJ] = CODE × 2.565e-10 × 1e6 × 1e9 / RSENSE_nΩ
	 * E [µJ] = CODE × 256500 / RSENSE_nΩ
	 *
	 * With VPWR divider scaling:
	 * E_actual = E_chip × (scale_vpwr / LTC4284_FS_AUX)
	 *
	 * To avoid overflow, calculate in steps:
	 * tmp = CODE × 256500 / RSENSE_nΩ
	 * val = tmp × scale_vpwr / 2048
	 */

	/* Calculate: CODE × 256500 / RSENSE [µJ at chip] */
	tmp = mul_u64_u32_div(raw, 256500, st->rsense1);

	/* Scale by VPWR divider ratio */
	*val = DIV_ROUND_CLOSEST_ULL(tmp * st->scale_vpwr, LTC4284_FS_AUX);

	return 0;
}

static int ltc4284_write_voltage(struct ltc4284_state *st, u8 reg, int channel, long val)
{
	u16 raw;
	u32 scale;
	int ret;

	ret = ltc4284_get_voltage_scale(st, channel, &scale);
	if (ret)
		return ret;

	/* Differential channels: scale is in µV, val is in mV, convert to match units */
	if (channel == LTC4284_CHAN_ADIN12 || channel == LTC4284_CHAN_ADIN34 ||
	    channel == LTC4284_CHAN_ADIO12 || channel == LTC4284_CHAN_ADIO34) {
		val = clamp_val(val, 0, scale / MILLI);
		raw = DIV_ROUND_CLOSEST_ULL((u64)val * 255 * MILLI, scale);
	} else {
		val = clamp_val(val, 0, scale);
		raw = DIV_ROUND_CLOSEST_ULL((u64)val * 255, scale);
	}

	/* Threshold registers are 8-bit */
	return regmap_write(st->regmap, reg, raw & 0xFF);
}

static int ltc4284_write_current(struct ltc4284_state *st, u8 reg, long val, u32 rsense)
{
	u16 raw;

	if (!rsense)
		return -EINVAL;

	val = clamp_val(val, 0, LONG_MAX);

	/*
	 * mA × nΩ gives voltage in picovolts (pV).
	 * raw = (val_mA × rsense_nΩ × 255) / (Full_Scale_µV × MEGA)
	 */
	raw = DIV64_U64_ROUND_CLOSEST((u64)val * rsense * 255,
				      (u64)LTC4284_FS_SENSE_UV * MEGA);

	/* All current threshold registers are 8-bit */
	return regmap_write(st->regmap, reg, raw & 0xFF);
}

static int ltc4284_write_power(struct ltc4284_state *st, u8 reg, long val)
{
	u16 raw;
	u64 tmp;

	val = clamp_val(val, 0, LONG_MAX);

	tmp = mul_u64_u32_div((u64)val * st->rsense1, LTC4284_FS_AUX,
			      st->scale_vpwr);

	/* All power threshold registers are 8-bit: LSB = 255 × 1024 nW */
	raw = DIV64_U64_ROUND_CLOSEST(tmp, 261120ULL * MEGA);

	return regmap_write(st->regmap, reg, raw & 0xFF);
}

static int ltc4284_reset_history(struct ltc4284_state *st, enum hwmon_sensor_types type,
				 int channel)
{
	u8 reg_min, reg_max;
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
		case LTC4284_CHAN_ADIN1:
			reg_min = LTC4284_ADIN1_MIN;
			reg_max = LTC4284_ADIN1_MAX;
			break;
		case LTC4284_CHAN_ADIN2:
			reg_min = LTC4284_ADIN2_MIN;
			reg_max = LTC4284_ADIN2_MAX;
			break;
		case LTC4284_CHAN_ADIN3:
			reg_min = LTC4284_ADIN3_MIN;
			reg_max = LTC4284_ADIN3_MAX;
			break;
		case LTC4284_CHAN_ADIN4:
			reg_min = LTC4284_ADIN4_MIN;
			reg_max = LTC4284_ADIN4_MAX;
			break;
		case LTC4284_CHAN_ADIO12:
			reg_min = LTC4284_ADIO12_MIN;
			reg_max = LTC4284_ADIO12_MAX;
			break;
		case LTC4284_CHAN_ADIO34:
			reg_min = LTC4284_ADIO34_MIN;
			reg_max = LTC4284_ADIO34_MAX;
			break;
		case LTC4284_CHAN_ADIO1:
			reg_min = LTC4284_ADIO1_MIN;
			reg_max = LTC4284_ADIO1_MAX;
			break;
		case LTC4284_CHAN_ADIO2:
			reg_min = LTC4284_ADIO2_MIN;
			reg_max = LTC4284_ADIO2_MAX;
			break;
		case LTC4284_CHAN_ADIO3:
			reg_min = LTC4284_ADIO3_MIN;
			reg_max = LTC4284_ADIO3_MAX;
			break;
		case LTC4284_CHAN_ADIO4:
			reg_min = LTC4284_ADIO4_MIN;
			reg_max = LTC4284_ADIO4_MAX;
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
	case hwmon_energy:
		/* Energy register is 48-bit, reset to 0 */
		return ltc4284_write_energy(st, 0);
	default:
		return -EINVAL;
	}

	/* Reset min/max registers */
	ret = ltc4284_write_word_data(st, reg_min, 0xFFFF);
	if (ret)
		return ret;

	return ltc4284_write_word_data(st, reg_max, 0);
}

static int ltc4284_get_threshold_reg(enum hwmon_sensor_types type, u32 attr,
				      int channel)
{
	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_min:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				return LTC4284_VPWR_MIN_TH;
			case LTC4284_CHAN_ADIN12:
				return LTC4284_ADIN12_MIN_TH;
			case LTC4284_CHAN_ADIN34:
				return LTC4284_ADIN34_MIN_TH;
			case LTC4284_CHAN_ADIN1:
				return LTC4284_ADIN1_MIN_TH;
			case LTC4284_CHAN_ADIN2:
				return LTC4284_ADIN2_MIN_TH;
			case LTC4284_CHAN_ADIN3:
				return LTC4284_ADIN3_MIN_TH;
			case LTC4284_CHAN_ADIN4:
				return LTC4284_ADIN4_MIN_TH;
			case LTC4284_CHAN_ADIO12:
				return LTC4284_ADIO12_MIN_TH;
			case LTC4284_CHAN_ADIO34:
				return LTC4284_ADIO34_MIN_TH;
			case LTC4284_CHAN_ADIO1:
				return LTC4284_ADIO1_MIN_TH;
			case LTC4284_CHAN_ADIO2:
				return LTC4284_ADIO2_MIN_TH;
			case LTC4284_CHAN_ADIO3:
				return LTC4284_ADIO3_MIN_TH;
			case LTC4284_CHAN_ADIO4:
				return LTC4284_ADIO4_MIN_TH;
			default:
				return -EOPNOTSUPP;
			}
		case hwmon_in_max:
			switch (channel) {
			case LTC4284_CHAN_VPWR:
				return LTC4284_VPWR_MAX_TH;
			case LTC4284_CHAN_ADIN12:
				return LTC4284_ADIN12_MAX_TH;
			case LTC4284_CHAN_ADIN34:
				return LTC4284_ADIN34_MAX_TH;
			case LTC4284_CHAN_ADIN1:
				return LTC4284_ADIN1_MAX_TH;
			case LTC4284_CHAN_ADIN2:
				return LTC4284_ADIN2_MAX_TH;
			case LTC4284_CHAN_ADIN3:
				return LTC4284_ADIN3_MAX_TH;
			case LTC4284_CHAN_ADIN4:
				return LTC4284_ADIN4_MAX_TH;
			case LTC4284_CHAN_ADIO12:
				return LTC4284_ADIO12_MAX_TH;
			case LTC4284_CHAN_ADIO34:
				return LTC4284_ADIO34_MAX_TH;
			case LTC4284_CHAN_ADIO1:
				return LTC4284_ADIO1_MAX_TH;
			case LTC4284_CHAN_ADIO2:
				return LTC4284_ADIO2_MAX_TH;
			case LTC4284_CHAN_ADIO3:
				return LTC4284_ADIO3_MAX_TH;
			case LTC4284_CHAN_ADIO4:
				return LTC4284_ADIO4_MAX_TH;
			default:
				return -EOPNOTSUPP;
			}
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_min:
			switch (channel) {
			case LTC4284_CHAN_SENSE:
				return LTC4284_SENSE_MIN_TH;
			case LTC4284_CHAN_SENSE1:
				return LTC4284_SENSE1_MIN_TH;
			case LTC4284_CHAN_SENSE2:
				return LTC4284_SENSE2_MIN_TH;
			default:
				return -EOPNOTSUPP;
			}
		case hwmon_curr_max:
			switch (channel) {
			case LTC4284_CHAN_SENSE:
				return LTC4284_SENSE_MAX_TH;
			case LTC4284_CHAN_SENSE1:
				return LTC4284_SENSE1_MAX_TH;
			case LTC4284_CHAN_SENSE2:
				return LTC4284_SENSE2_MAX_TH;
			default:
				return -EOPNOTSUPP;
			}
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_power:
		switch (attr) {
		case hwmon_power_min:
			return LTC4284_POWER_MIN_TH;
		case hwmon_power_max:
			return LTC4284_POWER_MAX_TH;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
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
			case LTC4284_CHAN_ADIN1:
				reg = LTC4284_ADIN1;
				break;
			case LTC4284_CHAN_ADIN2:
				reg = LTC4284_ADIN2;
				break;
			case LTC4284_CHAN_ADIN3:
				reg = LTC4284_ADIN3;
				break;
			case LTC4284_CHAN_ADIN4:
				reg = LTC4284_ADIN4;
				break;
			case LTC4284_CHAN_ADIO12:
				reg = LTC4284_ADIO12;
				break;
			case LTC4284_CHAN_ADIO34:
				reg = LTC4284_ADIO34;
				break;
			case LTC4284_CHAN_ADIO1:
				reg = LTC4284_ADIO1;
				break;
			case LTC4284_CHAN_ADIO2:
				reg = LTC4284_ADIO2;
				break;
			case LTC4284_CHAN_ADIO3:
				reg = LTC4284_ADIO3;
				break;
			case LTC4284_CHAN_ADIO4:
				reg = LTC4284_ADIO4;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_voltage(st, reg, channel, val);
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
			case LTC4284_CHAN_ADIN1:
				reg = LTC4284_ADIN1_MIN;
				break;
			case LTC4284_CHAN_ADIN2:
				reg = LTC4284_ADIN2_MIN;
				break;
			case LTC4284_CHAN_ADIN3:
				reg = LTC4284_ADIN3_MIN;
				break;
			case LTC4284_CHAN_ADIN4:
				reg = LTC4284_ADIN4_MIN;
				break;
			case LTC4284_CHAN_ADIO12:
				reg = LTC4284_ADIO12_MIN;
				break;
			case LTC4284_CHAN_ADIO34:
				reg = LTC4284_ADIO34_MIN;
				break;
			case LTC4284_CHAN_ADIO1:
				reg = LTC4284_ADIO1_MIN;
				break;
			case LTC4284_CHAN_ADIO2:
				reg = LTC4284_ADIO2_MIN;
				break;
			case LTC4284_CHAN_ADIO3:
				reg = LTC4284_ADIO3_MIN;
				break;
			case LTC4284_CHAN_ADIO4:
				reg = LTC4284_ADIO4_MIN;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_voltage(st, reg, channel, val);
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
			case LTC4284_CHAN_ADIN1:
				reg = LTC4284_ADIN1_MAX;
				break;
			case LTC4284_CHAN_ADIN2:
				reg = LTC4284_ADIN2_MAX;
				break;
			case LTC4284_CHAN_ADIN3:
				reg = LTC4284_ADIN3_MAX;
				break;
			case LTC4284_CHAN_ADIN4:
				reg = LTC4284_ADIN4_MAX;
				break;
			case LTC4284_CHAN_ADIO12:
				reg = LTC4284_ADIO12_MAX;
				break;
			case LTC4284_CHAN_ADIO34:
				reg = LTC4284_ADIO34_MAX;
				break;
			case LTC4284_CHAN_ADIO1:
				reg = LTC4284_ADIO1_MAX;
				break;
			case LTC4284_CHAN_ADIO2:
				reg = LTC4284_ADIO2_MAX;
				break;
			case LTC4284_CHAN_ADIO3:
				reg = LTC4284_ADIO3_MAX;
				break;
			case LTC4284_CHAN_ADIO4:
				reg = LTC4284_ADIO4_MAX;
				break;
			default:
				return -EINVAL;
			}
			return ltc4284_read_voltage(st, reg, channel, val);
		case hwmon_in_min:
		case hwmon_in_max:
			reg = ltc4284_get_threshold_reg(type, attr, channel);
			if (reg < 0)
				return reg;
			return ltc4284_read_voltage(st, reg, channel, val);
		case hwmon_in_crit_alarm:
			if (channel == LTC4284_CHAN_VPWR)
				return ltc4284_read_alarm(st, LTC4284_FAULT_STATUS,
							  LTC4284_OV_STATUS_MASK, val);
			return -EOPNOTSUPP;
		case hwmon_in_lcrit_alarm:
			if (channel == LTC4284_CHAN_VPWR)
				return ltc4284_read_alarm(st, LTC4284_FAULT_STATUS,
							  LTC4284_UV_STATUS_MASK, val);
			return -EOPNOTSUPP;
		default:
			return -EOPNOTSUPP;
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
		case hwmon_curr_max:
			reg = ltc4284_get_threshold_reg(type, attr, channel);
			if (reg < 0)
				return reg;
			return ltc4284_read_current(st, reg, rsense, val);

		case hwmon_curr_crit_alarm:
			if (channel == LTC4284_CHAN_SENSE)
				return ltc4284_read_alarm(st, LTC4284_FAULT_STATUS,
							  LTC4284_OC_STATUS_MASK, val);
			return -EOPNOTSUPP;
		default:
			return -EOPNOTSUPP;
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
		case hwmon_power_max:
			reg = ltc4284_get_threshold_reg(type, attr, channel);
			if (reg < 0)
				return reg;
			return ltc4284_read_power(st, reg, val);
		default:
			return -EINVAL;
		}
		return ltc4284_read_power(st, reg, val);
	case hwmon_energy:
		if (attr == hwmon_energy_input)
			return ltc4284_read_energy(st, val);
		break;
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
	u32 rsense;

	switch (type) {
	case hwmon_in:
		if (attr == hwmon_in_reset_history)
			return ltc4284_reset_history(st, type, channel);

		reg = ltc4284_get_threshold_reg(type, attr, channel);
		if (reg < 0)
			return reg;
		return ltc4284_write_voltage(st, reg, channel, val);

	case hwmon_curr:
		if (attr == hwmon_curr_reset_history)
			return ltc4284_reset_history(st, type, channel);

		switch (channel) {
		case LTC4284_CHAN_SENSE:
			rsense = st->rsense1;
			break;
		case LTC4284_CHAN_SENSE1:
			rsense = st->rsense1;
			break;
		case LTC4284_CHAN_SENSE2:
			rsense = st->rsense2;
			break;
		default:
			return -EOPNOTSUPP;
		}

		reg = ltc4284_get_threshold_reg(type, attr, channel);
		if (reg < 0)
			return reg;
		return ltc4284_write_current(st, reg, val, rsense);

	case hwmon_power:
		if (attr == hwmon_power_reset_history)
			return ltc4284_reset_history(st, type, channel);

		reg = ltc4284_get_threshold_reg(type, attr, channel);
		if (reg < 0)
			return reg;
		return ltc4284_write_power(st, reg, val);

	case hwmon_energy:
		if (attr == hwmon_energy_enable)
			return ltc4284_reset_history(st, type, channel);
		break;

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_channel_info * const ltc4284_info[] = {
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_CRIT_ALARM |
			   HWMON_I_LCRIT_ALARM | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* VPWR */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIN12 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIN34 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIN1 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIN2 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIN3 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIN4 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIO12 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIO34 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIO1 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIO2 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL,  /* ADIO3 */
			   HWMON_I_INPUT | HWMON_I_LOWEST | HWMON_I_HIGHEST |
			   HWMON_I_MIN | HWMON_I_MAX | HWMON_I_RESET_HISTORY |
			   HWMON_I_LABEL), /* ADIO4 */
	HWMON_CHANNEL_INFO(curr,
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_MIN | HWMON_C_MAX | HWMON_C_CRIT_ALARM |
			   HWMON_C_RESET_HISTORY | HWMON_C_LABEL,  /* SENSE */
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_MIN | HWMON_C_MAX |
			   HWMON_C_RESET_HISTORY | HWMON_C_LABEL,  /* SENSE1 */
			   HWMON_C_INPUT | HWMON_C_LOWEST | HWMON_C_HIGHEST |
			   HWMON_C_MIN | HWMON_C_MAX |
			   HWMON_C_RESET_HISTORY | HWMON_C_LABEL), /* SENSE2 */
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_INPUT | HWMON_P_INPUT_LOWEST |
			   HWMON_P_INPUT_HIGHEST | HWMON_P_MIN | HWMON_P_MAX |
			   HWMON_P_RESET_HISTORY | HWMON_P_LABEL),
	HWMON_CHANNEL_INFO(energy,
			   HWMON_E_INPUT | HWMON_E_ENABLE | HWMON_E_LABEL),
	NULL
};

static const char * const ltc4284_in_labels[] = {
	"VPWR",
	"ADIN12",
	"ADIN34",
	"ADIN1",
	"ADIN2",
	"ADIN3",
	"ADIN4",
	"ADIO12",
	"ADIO34",
	"ADIO1",
	"ADIO2",
	"ADIO3",
	"ADIO4",
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
	case hwmon_energy:
		*str = "Energy";
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

/* Debugfs fault log attributes */
static int ltc4284_show_fault_log(void *arg, u64 *val, u32 mask)
{
	struct ltc4284_state *st = arg;
	long alarm;
	int ret;

	ret = ltc4284_read_alarm(st, LTC4284_FAULT, mask, &alarm);
	if (ret)
		return ret;

	*val = alarm;
	return 0;
}

static int ltc4284_show_ov_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_OV_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_ov_fault_log,
			 ltc4284_show_ov_fault_log, NULL, "%llu\n");

static int ltc4284_show_uv_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_UV_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_uv_fault_log,
			 ltc4284_show_uv_fault_log, NULL, "%llu\n");

static int ltc4284_show_oc_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_OC_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_oc_fault_log,
			 ltc4284_show_oc_fault_log, NULL, "%llu\n");

static int ltc4284_show_fet_bad_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_FET_BAD_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_fet_bad_fault_log,
			 ltc4284_show_fet_bad_fault_log, NULL, "%llu\n");

static int ltc4284_show_pgi_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_PGI_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_pgi_fault_log,
			 ltc4284_show_pgi_fault_log, NULL, "%llu\n");

static int ltc4284_show_power_failed_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_POWER_FAILED_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_power_failed_log,
			 ltc4284_show_power_failed_log, NULL, "%llu\n");

static int ltc4284_show_fet_short_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_FET_SHORT_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_fet_short_fault_log,
			 ltc4284_show_fet_short_fault_log, NULL, "%llu\n");

static int ltc4284_show_ext_fault_log(void *arg, u64 *val)
{
	return ltc4284_show_fault_log(arg, val, LTC4284_EXT_FAULT_MASK);
}
DEFINE_DEBUGFS_ATTRIBUTE(ltc4284_ext_fault_log,
			 ltc4284_show_ext_fault_log, NULL, "%llu\n");

static void ltc4284_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static void ltc4284_debugfs_init(struct ltc4284_state *st,
				 struct i2c_client *i2c,
				 const struct device *hwmon)
{
	const char *debugfs_name;
	struct dentry *dentry;
	int ret;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_name = devm_kasprintf(&i2c->dev, GFP_KERNEL, "ltc4284-%s",
				      dev_name(hwmon));
	if (!debugfs_name)
		return;

	dentry = debugfs_create_dir(debugfs_name, NULL);
	if (IS_ERR(dentry))
		return;

	ret = devm_add_action_or_reset(&i2c->dev, ltc4284_debugfs_remove,
				       dentry);
	if (ret)
		return;

	debugfs_create_file_unsafe("in0_ov_fault_log", 0400, dentry, st,
				   &ltc4284_ov_fault_log);
	debugfs_create_file_unsafe("in0_uv_fault_log", 0400, dentry, st,
				   &ltc4284_uv_fault_log);
	debugfs_create_file_unsafe("curr1_oc_fault_log", 0400, dentry, st,
				   &ltc4284_oc_fault_log);
	debugfs_create_file_unsafe("fet_bad_fault_log", 0400, dentry, st,
				   &ltc4284_fet_bad_fault_log);
	debugfs_create_file_unsafe("pgi_fault_log", 0400, dentry, st,
				   &ltc4284_pgi_fault_log);
	debugfs_create_file_unsafe("power_failed_log", 0400, dentry, st,
				   &ltc4284_power_failed_log);
	debugfs_create_file_unsafe("fet_short_fault_log", 0400, dentry, st,
				   &ltc4284_fet_short_fault_log);
	debugfs_create_file_unsafe("ext_fault_log", 0400, dentry, st,
				   &ltc4284_ext_fault_log);
}

static int ltc4284_read_current_limit(struct ltc4284_state *st)
{
	unsigned int reg_val;
	int ret, ilim_code;

	/* Read CONFIG_1 register (0x0D) to get ILIM value from EEPROM */
	ret = regmap_read(st->regmap, LTC4284_CONFIG, &reg_val);
	if (ret)
		return ret;

	/* Extract ILIM field (bits 7:4 of CONFIG_1) */
	ilim_code = FIELD_GET(LTC4284_ILIM_MASK, reg_val);

	/* Convert ILIM code to microvolts: 15mV + (code * 1mV) */
	st->current_limit = LTC4284_ILIM_MIN + (ilim_code * LTC4284_ILIM_STEP);

	return 0;
}

static int ltc4284_parse_dt(struct ltc4284_state *st, struct device *dev)
{
	u32 val;
	int ret, i;
	char prop_name[32];

	/* Read EEPROM configuration for default current limit */

	ret = ltc4284_read_current_limit(st);
	if (ret)
		return ret;

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

	/* Check device tree current limit, if not provided use EEPROM value */
	ret = device_property_read_u32(dev, "adi,current-limit-sense-microvolt",
				       &st->current_limit);
	if (!ret) {
		/* Device tree override provided - validate and apply */
		if (st->current_limit < LTC4284_ILIM_MIN ||
		    st->current_limit > LTC4284_ILIM_MAX) {
			dev_err(dev, "Invalid current limit %u µV (min: %u, max: %u)\n",
				st->current_limit, LTC4284_ILIM_MIN, LTC4284_ILIM_MAX);
			return -EINVAL;
		}

		val = (st->current_limit - LTC4284_ILIM_MIN) / LTC4284_ILIM_STEP;
		ret = regmap_update_bits(st->regmap, LTC4284_CONFIG, LTC4284_ILIM_MASK,
					 FIELD_PREP(LTC4284_ILIM_MASK, val));
		if (ret)
			return ret;
	}

	/* VPWR voltage scale - default to chip full-scale (2.048V) */
	ret = device_property_read_u32(dev, "adi,vpwr-millivolt", &st->scale_vpwr);
	if (ret)
		st->scale_vpwr = LTC4284_FS_AUX;

	/* ADIN1-4 ADIO1-4 voltage scales - default to chip full-scale (2.048V) */
	for (i = 0; i < 4; i++) {
		snprintf(prop_name, sizeof(prop_name), "adi,adin%d-millivolt", i + 1);
		ret = device_property_read_u32(dev, prop_name, &st->scale_adin[i]);
		if (ret)
			st->scale_adin[i] = LTC4284_FS_AUX;

		snprintf(prop_name, sizeof(prop_name), "adi,adio%d-millivolt", i + 1);
		ret = device_property_read_u32(dev, prop_name, &st->scale_adio[i]);
		if (ret)
			st->scale_adio[i] = LTC4284_FS_AUX;
	}

	/* NOTE: Operation mode is configured via hardware MODE pin, not device tree */

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
	if (ret < 0)
		return ret;

	/* Enable all ADC selectable channels */
	ret = ltc4284_write_word_data(st, LTC4284_ADC_SELECT, 0xFFFF);
	if (ret)
		return ret;

	/* NOTE: Operation mode is configured via hardware MODE pin, not register */
	/* NOTE: No fault logging enable bit exists in CONTROL register */

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "ltc4284", st,
							 &ltc4284_chip_info,
							 NULL);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	ltc4284_debugfs_init(st, client, hwmon_dev);

	return 0;
}

static const struct of_device_id ltc4284_of_match[] = {
	{ .compatible = "adi,ltc4284" },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc4284_of_match);

static const struct i2c_device_id ltc4284_id[] = {
	{ "ltc4284" },
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

MODULE_AUTHOR("Alexis Czezar Torreno <alexisczezar.torreno@analog.com>");
MODULE_DESCRIPTION("LTC4284 High Power Negative Voltage Hot Swap Controller");
MODULE_LICENSE("GPL");
