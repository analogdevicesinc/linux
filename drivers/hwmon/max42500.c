// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX42500 - Industrial Power System Monitor
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/io.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#define MAX42500_REG_ID		0x00
#define MAX42500_REG_CONFIG1		0x01
#define MAX42500_REG_CONFIG2		0x02
#define MAX42500_REG_VMON		0x03
#define MAX42500_REG_RSTMAP		0x04
#define MAX42500_REG_STATOV		0x05
#define MAX42500_REG_STATUV		0x06
#define MAX42500_REG_STATOFF		0x07
#define MAX42500_REG_VIN1		0x08
#define MAX42500_REG_VIN2		0x09
#define MAX42500_REG_VIN3		0x0A
#define MAX42500_REG_VIN4		0x0B
#define MAX42500_REG_VIN5		0x0C
#define MAX42500_REG_VINO6		0x0D
#define MAX42500_REG_VINU6		0x0E
#define MAX42500_REG_VINO7		0x0F
#define MAX42500_REG_VINU7		0x10
#define MAX42500_REG_OVUV1		0x11
#define MAX42500_REG_OVUV2		0x12
#define MAX42500_REG_OVUV3		0x13
#define MAX42500_REG_OVUV4		0x14
#define MAX42500_REG_OVUV5		0x15
#define MAX42500_REG_FPSSTAT1		0x16
#define MAX42500_REG_FPSCFG1		0x17
#define MAX42500_REG_UTIME1		0x18
#define MAX42500_REG_UTIME2		0x19
#define MAX42500_REG_UTIME3		0x1A
#define MAX42500_REG_UTIME4		0x1B
#define MAX42500_REG_UTIME5		0x1C
#define MAX42500_REG_UTIME6		0x1D
#define MAX42500_REG_UTIME7		0x1E
#define MAX42500_REG_DTIME1		0x1F
#define MAX42500_REG_DTIME2		0x20
#define MAX42500_REG_DTIME3		0x21
#define MAX42500_REG_DTIME4		0x22
#define MAX42500_REG_DTIME5		0x23
#define MAX42500_REG_DTIME6		0x24
#define MAX42500_REG_DTIME7		0x25
#define MAX42500_REG_WDSTAT		0x26
#define MAX42500_REG_WDCDIV		0x27
#define MAX42500_REG_WDCFG1		0x28
#define MAX42500_REG_WDCFG2		0x29
#define MAX42500_REG_WDKEY		0x2A
#define MAX42500_REG_WDLOCK		0x2B
#define MAX42500_REG_RSTCTRL		0x2C
#define MAX42500_REG_CID		0x2D

enum max42500_dev_part {
	MAX42500_DEV_PART_CHIP,
	MAX42500_DEV_PART_VMON,
	MAX42500_DEV_PART_FPSR,
	MAX42500_DEV_PART_WDOG
};

enum max42500_wd_mode {
	MAX42500_WD_MODE_CH_RESP,
	MAX42500_WD_MODE_SIMPLE,
	MAX42500_WD_MODE_MAX
};

enum max42500_attr_chip {
	MAX42500_ATTR_CHIP_LABEL,
	MAX42500_ATTR_CHIP_NAME,
	MAX42500_ATTR_CHIP_RELOAD_OTP_ENABLE,
	MAX42500_ATTR_CHIP_BIST_RESET_ENABLE,
	MAX42500_ATTR_CHIP_PEC_ENABLE,
	MAX42500_ATTR_CHIP_CONFIG_STATUS,
	MAX42500_ATTR_CHIP_MAX
};

static const u8 max42500_reg_chip[] = { MAX42500_REG_ID,
					MAX42500_REG_CID,
					MAX42500_REG_CONFIG1,
					MAX42500_REG_CONFIG1,
					MAX42500_REG_CONFIG1,
					MAX42500_REG_CONFIG2
					};

enum max42500_attr_vmon {
	MAX42500_ATTR_VMON_IN1_ENABLE,
	MAX42500_ATTR_VMON_IN2_ENABLE,
	MAX42500_ATTR_VMON_IN3_ENABLE,
	MAX42500_ATTR_VMON_IN4_ENABLE,
	MAX42500_ATTR_VMON_IN5_ENABLE,
	MAX42500_ATTR_VMON_IN6_ENABLE,
	MAX42500_ATTR_VMON_IN7_ENABLE,
	MAX42500_ATTR_VMON_PDN_ENABLE,
	MAX42500_ATTR_VMON_IN1_RESET_ENABLE,
	MAX42500_ATTR_VMON_IN2_RESET_ENABLE,
	MAX42500_ATTR_VMON_IN3_RESET_ENABLE,
	MAX42500_ATTR_VMON_IN4_RESET_ENABLE,
	MAX42500_ATTR_VMON_IN5_RESET_ENABLE,
	MAX42500_ATTR_VMON_IN6_RESET_ENABLE,
	MAX42500_ATTR_VMON_IN7_RESET_ENABLE,
	MAX42500_ATTR_VMON_PAR_ENABLE,
	MAX42500_ATTR_VMON_VMON_STATUS_OV,
	MAX42500_ATTR_VMON_VMON_STATUS_UV,
	MAX42500_ATTR_VMON_VMON_STATUS_OFF,
	MAX42500_ATTR_VMON_IN1_NOMINAL,
	MAX42500_ATTR_VMON_IN2_NOMINAL,
	MAX42500_ATTR_VMON_IN3_NOMINAL,
	MAX42500_ATTR_VMON_IN4_NOMINAL,
	MAX42500_ATTR_VMON_IN5_NOMINAL,
	MAX42500_ATTR_VMON_IN6_CRIT,
	MAX42500_ATTR_VMON_IN6_LCRIT,
	MAX42500_ATTR_VMON_IN7_CRIT,
	MAX42500_ATTR_VMON_IN7_LCRIT,
	MAX42500_ATTR_VMON_IN1_OV_THRESH,
	MAX42500_ATTR_VMON_IN1_UV_THRESH,
	MAX42500_ATTR_VMON_IN2_OV_THRESH,
	MAX42500_ATTR_VMON_IN2_UV_THRESH,
	MAX42500_ATTR_VMON_IN3_OV_THRESH,
	MAX42500_ATTR_VMON_IN3_UV_THRESH,
	MAX42500_ATTR_VMON_IN4_OV_THRESH,
	MAX42500_ATTR_VMON_IN4_UV_THRESH,
	MAX42500_ATTR_VMON_IN5_OV_THRESH,
	MAX42500_ATTR_VMON_IN5_UV_THRESH,
	MAX42500_ATTR_VMON_MAX
};

static const u8 max42500_reg_vmon[] = { MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_VMON,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_RSTMAP,
					MAX42500_REG_STATOV,
					MAX42500_REG_STATUV,
					MAX42500_REG_STATOFF,
					MAX42500_REG_VIN1,
					MAX42500_REG_VIN2,
					MAX42500_REG_VIN3,
					MAX42500_REG_VIN4,
					MAX42500_REG_VIN5,
					MAX42500_REG_VINO6,
					MAX42500_REG_VINU6,
					MAX42500_REG_VINO7,
					MAX42500_REG_VINU7,
					MAX42500_REG_OVUV1,
					MAX42500_REG_OVUV1,
					MAX42500_REG_OVUV2,
					MAX42500_REG_OVUV2,
					MAX42500_REG_OVUV3,
					MAX42500_REG_OVUV3,
					MAX42500_REG_OVUV4,
					MAX42500_REG_OVUV4,
					MAX42500_REG_OVUV5,
					MAX42500_REG_OVUV5
					};

enum max42500_attr_fpsr {
	MAX42500_ATTR_FPSR_FPS_STATUS,
	MAX42500_ATTR_FPSR_POWERUP_SEQ_CAPTURE_FINISH,
	MAX42500_ATTR_FPSR_POWERDN_SEQ_CAPTURE_FINISH,
	MAX42500_ATTR_FPSR_POWERUP_NO_INTERRUPT_ENABLE,
	MAX42500_ATTR_FPSR_POWERDN_NO_INTERRUPT_ENABLE,
	MAX42500_ATTR_FPSR_FPS_EN1_START_TIMER_ENABLE,
	MAX42500_ATTR_FPSR_FPS_CLOCK_DIVIDER,
	MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MAX,
	MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MIN,
	MAX42500_ATTR_FPSR_MAX
};

static const u8 max42500_reg_fpsr[] = { MAX42500_REG_FPSSTAT1,
					MAX42500_REG_FPSCFG1,
					MAX42500_REG_FPSCFG1,
					MAX42500_REG_FPSCFG1,
					MAX42500_REG_FPSCFG1,
					MAX42500_REG_FPSCFG1,
					MAX42500_REG_FPSCFG1,
					MAX42500_REG_UTIME1,
					MAX42500_REG_UTIME2,
					MAX42500_REG_UTIME3,
					MAX42500_REG_UTIME4,
					MAX42500_REG_UTIME5,
					MAX42500_REG_UTIME6,
					MAX42500_REG_UTIME7,
					MAX42500_REG_DTIME1,
					MAX42500_REG_DTIME2,
					MAX42500_REG_DTIME3,
					MAX42500_REG_DTIME4,
					MAX42500_REG_DTIME5,
					MAX42500_REG_DTIME6,
					MAX42500_REG_DTIME7
					};

enum max42500_attr_wdog {
	MAX42500_ATTR_WDOG_WD_STATUS,
	MAX42500_ATTR_WDOG_WD_SIMPLE_MODE_ENABLE,
	MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER,
	MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL,
	MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL,
	MAX42500_ATTR_WDOG_WD_ENABLE,
	MAX42500_ATTR_WDOG_WD_FIRST_UPDATE_INTERVAL,
	MAX42500_ATTR_WDOG_WD_KEY,
	MAX42500_ATTR_WDOG_WD_LOCK_ENABLE,
	MAX42500_ATTR_WDOG_WD_RESET_TWO_COUNT_ENABLE,
	MAX42500_ATTR_WDOG_WD_RESET_HOLD_INTERVAL,
	MAX42500_ATTR_WDOG_MAX
};

static const u8 max42500_reg_wdog[] = { MAX42500_REG_WDSTAT,
					MAX42500_REG_WDCDIV,
					MAX42500_REG_WDCDIV,
					MAX42500_REG_WDCFG1,
					MAX42500_REG_WDCFG1,
					MAX42500_REG_WDCFG2,
					MAX42500_REG_WDCFG2,
					MAX42500_REG_WDKEY,
					MAX42500_REG_WDLOCK,
					MAX42500_REG_RSTCTRL,
					MAX42500_REG_RSTCTRL
					};

struct max42500_state {
	struct i2c_client	*client;
	struct regmap		*regmap;
	struct mutex		lock; /* Protects access during data transfer */
	struct gpio_chip	gc;
	struct gpio_desc	**gpios;
	int			num_gpios;
	u8			chip[MAX42500_ATTR_CHIP_MAX];
	u8			vmon[MAX42500_ATTR_VMON_MAX];
	u8			fpsr[MAX42500_ATTR_FPSR_MAX];
	u8			wdog[MAX42500_ATTR_WDOG_MAX];
};

/* VMON Register to Nominal voltage (uV) Conversion */
static long convert_vin_nominal_from_reg(u8 vmon)
{
	return 500000 + (12500 * vmon);
}

/* VMON Register to Nominal voltage (mV) Conversion */
static long convert_vin5_nominal_from_reg(u8 vmon)
{
	return 500 + (20 * vmon);
}

/* VMON Register to Critical voltage (mV) Conversion */
static long convert_vin_critical_from_reg(u8 vmon)
{
	return 500 + (5 * vmon);
}

/* VMON Register to OV threshold (100,000%) Conversion */
static long convert_ov_threshold_from_reg(u8 vmon)
{
	return 102500 + (500 * vmon);
}

/* VMON Register to UV threshold (100,000%) Conversion */
static long convert_uv_threshold_from_reg(u8 vmon)
{
	return 97500 - (500 * vmon);
}

/* FPSR Register to Timestamp (usec) Conversion */
static long convert_timestamp_from_reg(u8 fpsr, u8 clk_div)
{
	return (fpsr - 1) * 25 * (1 << clk_div);
}

/* Watchdog Register to Clock divider (usec) Conversion */
static long convert_wd_clkdiv_from_reg(u8 wdog)
{
	return (wdog + 1) * 200;
}

/* Watchdog Register to Window open/close time (usec) Conversion */
static long convert_wd_openclose_from_reg(u8 wdog, u8 clk_div)
{
	return ((wdog + 1) << 3) * clk_div;
}

/* Watchdog Register to First power-up time (usec) Conversion */
static long convert_wd_firstup_from_reg(u8 wdog, u8 wclo, u8 wopn)
{
	return (wclo + wopn) * ((wdog << 1) + 1);
}

/* Watchdog Register to Reset hold time (usec) Conversion */
static long convert_wd_holdreset_from_reg(u8 wdog)
{
	return wdog ?  (1 << (wdog + 3)) : 0;
}

/* VMON Nominal voltage (uV) to Register Conversion */
static long convert_vin_nominal_to_reg(long value)
{
	return (value - 500000) / 12500;
}

/* VMON Nominal voltage (mV) to Register Conversion */
static long convert_vin5_nominal_to_reg(long value)
{
	return (value - 500) / 20;
}

/* VMON Critical voltage (mV) to Register Conversion */
static long convert_vin_critical_to_reg(long value)
{
	return (value - 500) / 5;		/* mV */
}

/* VMON OV threshold (100,000%) to Register Conversion */
static long convert_ov_threshold_to_reg(long value)
{
	return (value - 102500) / 500;
}

/* VMON UV threshold (100,000%) to Register Conversion */
static long convert_uv_threshold_to_reg(long value)
{
	return (97500 - value) / 500;
}

/* FPSR Timestamp (usec) to Register Conversion */
static long convert_timestamp_to_reg(u8 value, u8 clk_div)
{
	return value / (25 * (1 << clk_div)) + 1;
}

/* Watchdog Clock divider time (usec) to Register Conversion */
static long convert_wd_clkdiv_to_reg(long value)
{
	return (value - 200) / 200;
}

/* Watchdog Window open/close time (usec) to Register Conversion */
static long convert_wd_openclose_to_reg(long value, u8 clk_div)
{
	return value / (clk_div << 3);
}

/* Watchdog First power-up time (usec) to Register Conversion */
static long convert_wd_firstup_to_reg(long value, u8 wclo, u8 wopn)
{
	return ((value / (wclo + wopn)) - 1) >> 1;
}

/* Watchdog Reset hold time (usec) to Register Conversion */
static long convert_wd_holdreset_to_reg(long value)
{
	/* Check values if divisible by 8. */
	if (value % 8)
		return -EINVAL;

	/* Valid are 0, 8, 16 and 32 only. */
	value = clamp_val(value, 0, 32);
	if (value == 0)
		return 0;

	/* Test for valid bits 1, 2 or 4. */
	if (((value >> 3) ^ 3))
		return (value >> 4) + 1;

	return -EINVAL;
}

/* Watchdog Key Challenge-Response Computation */
static long max42500_prep_watchdog_key(enum max42500_wd_mode mode, u8 key)
{
	u8 lfsr;

	if (mode == MAX42500_WD_MODE_SIMPLE)
		return key;

	if (mode != MAX42500_WD_MODE_CH_RESP)
		return -EINVAL;

	/* Linear-Feedback Shift Register (LFSR) Polynomial Equation */
	lfsr = ((key >> 7) ^ (key >> 5) ^ (key >> 4) ^ (key >> 3)) & 1;

	return (key << 1) | lfsr;
}

static int max42500_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0)
		return ret;

	*val = ret;

	return 0;
}

static int max42500_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct device *dev = context;
	struct i2c_client *i2c = to_i2c_client(dev);

	return i2c_smbus_write_byte_data(i2c, reg, val);
}

static int max42500_get_updates(struct max42500_state *st,
				enum max42500_dev_part part, u8 attr)
{
	unsigned int reg_cmd, reg_val;
	long value;
	int ret;

	mutex_lock(&st->lock);
	switch (part) {
	case MAX42500_DEV_PART_CHIP:
		reg_cmd = max42500_reg_chip[attr];
		break;
	case MAX42500_DEV_PART_VMON:
		reg_cmd = max42500_reg_vmon[attr];
		break;
	case MAX42500_DEV_PART_FPSR:
		reg_cmd = max42500_reg_fpsr[attr];
		break;
	case MAX42500_DEV_PART_WDOG:
		reg_cmd = max42500_reg_wdog[attr];
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_read(st->regmap, reg_cmd, &reg_val);
	if (ret)
		goto exit;

	switch (part) {
	case MAX42500_DEV_PART_CHIP:
		switch (attr) {
		case MAX42500_ATTR_CHIP_LABEL: /* ID */
		case MAX42500_ATTR_CHIP_NAME: /* CID */
		case MAX42500_ATTR_CHIP_CONFIG_STATUS: /* CFG2 */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		case MAX42500_ATTR_CHIP_PEC_ENABLE: /* PECE */
			value = FIELD_GET(BIT(0), reg_val);
			break;
		case MAX42500_ATTR_CHIP_BIST_RESET_ENABLE: /* MBST */
			value = FIELD_GET(BIT(1), reg_val);
			break;
		case MAX42500_ATTR_CHIP_RELOAD_OTP_ENABLE: /* RR */
			value = FIELD_GET(BIT(2), reg_val);
			break;
		default:
			ret = -EINVAL;
			goto exit;
		}

		st->chip[attr] = FIELD_GET(GENMASK(7, 0), value);
		break;
	case MAX42500_DEV_PART_VMON:
		switch (attr) {
		case MAX42500_ATTR_VMON_IN1_ENABLE: /* VIN1 */
		case MAX42500_ATTR_VMON_IN1_RESET_ENABLE: /* RST1 */
			value = FIELD_GET(BIT(0), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN2_ENABLE: /* VIN2 */
		case MAX42500_ATTR_VMON_IN2_RESET_ENABLE: /* RST2 */
			value = FIELD_GET(BIT(1), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN3_ENABLE: /* VIN3 */
		case MAX42500_ATTR_VMON_IN3_RESET_ENABLE: /* RST3 */
			value = FIELD_GET(BIT(2), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN4_ENABLE: /* VIN4 */
		case MAX42500_ATTR_VMON_IN4_RESET_ENABLE: /* RST4 */
			value = FIELD_GET(BIT(3), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN5_ENABLE: /* VIN5 */
		case MAX42500_ATTR_VMON_IN5_RESET_ENABLE: /* RST5 */
			value = FIELD_GET(BIT(4), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN6_ENABLE: /* VIN6 */
		case MAX42500_ATTR_VMON_IN6_RESET_ENABLE: /*RST6 */
			value = FIELD_GET(BIT(5), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN7_ENABLE: /* VIN7 */
		case MAX42500_ATTR_VMON_IN7_RESET_ENABLE: /* RST7 */
			value = FIELD_GET(BIT(6), reg_val);
			break;
		case MAX42500_ATTR_VMON_PDN_ENABLE: /* VMPD */
		case MAX42500_ATTR_VMON_PAR_ENABLE: /* PARM */
			value = FIELD_GET(BIT(7), reg_val);
			break;
		case MAX42500_ATTR_VMON_VMON_STATUS_OV: /* STATOV */
		case MAX42500_ATTR_VMON_VMON_STATUS_UV: /* STATUV */
		case MAX42500_ATTR_VMON_VMON_STATUS_OFF: /* STATOFF */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN1_NOMINAL: /* VIN1 */
		case MAX42500_ATTR_VMON_IN2_NOMINAL: /* VIN2 */
		case MAX42500_ATTR_VMON_IN3_NOMINAL: /* VIN3 */
		case MAX42500_ATTR_VMON_IN4_NOMINAL: /* VIN4 */
		case MAX42500_ATTR_VMON_IN5_NOMINAL: /* VIN5 */
		case MAX42500_ATTR_VMON_IN6_CRIT: /* VINO6 */
		case MAX42500_ATTR_VMON_IN6_LCRIT: /* VINU6 */
		case MAX42500_ATTR_VMON_IN7_CRIT: /* VINO7 */
		case MAX42500_ATTR_VMON_IN7_LCRIT: /* VINU7 */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN1_OV_THRESH: /* VINO1 */
		case MAX42500_ATTR_VMON_IN2_OV_THRESH: /* VINO2 */
		case MAX42500_ATTR_VMON_IN3_OV_THRESH: /* VINO3 */
		case MAX42500_ATTR_VMON_IN4_OV_THRESH: /* VINO4 */
		case MAX42500_ATTR_VMON_IN5_OV_THRESH: /* VINO5 */
			value = FIELD_GET(GENMASK(7, 4), reg_val);
			break;
		case MAX42500_ATTR_VMON_IN1_UV_THRESH: /* VINU1 */
		case MAX42500_ATTR_VMON_IN2_UV_THRESH: /* VINU2 */
		case MAX42500_ATTR_VMON_IN3_UV_THRESH: /* VINU3 */
		case MAX42500_ATTR_VMON_IN4_UV_THRESH: /* VINU4 */
		case MAX42500_ATTR_VMON_IN5_UV_THRESH: /* VINU5 */
			value = FIELD_GET(GENMASK(3, 0), reg_val);
			break;
		default:
			ret = -EINVAL;
			goto exit;
		}

		st->vmon[attr] = FIELD_GET(GENMASK(7, 0), value);
		break;
	case MAX42500_DEV_PART_FPSR:
		switch (attr) {
		case MAX42500_ATTR_FPSR_FPS_STATUS: /* FPSSTAT1 */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		case MAX42500_ATTR_FPSR_FPS_CLOCK_DIVIDER: /* FDIV */
			value = FIELD_GET(GENMASK(2, 0), reg_val);
			break;
		case MAX42500_ATTR_FPSR_FPS_EN1_START_TIMER_ENABLE: /* FPSEN1 */
			value = FIELD_GET(BIT(3), reg_val);
			break;
		case MAX42500_ATTR_FPSR_POWERDN_NO_INTERRUPT_ENABLE: /* DVALM */
			value = FIELD_GET(BIT(4), reg_val);
			break;
		case MAX42500_ATTR_FPSR_POWERUP_NO_INTERRUPT_ENABLE: /* UVALM */
			value = FIELD_GET(BIT(5), reg_val);
			break;
		case MAX42500_ATTR_FPSR_POWERDN_SEQ_CAPTURE_FINISH: /* DVAL */
			value = FIELD_GET(BIT(6), reg_val);
			break;
		case MAX42500_ATTR_FPSR_POWERUP_SEQ_CAPTURE_FINISH: /* UVAL */
			value = FIELD_GET(BIT(7), reg_val);
			break;
		case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MAX: /* UTIM1 */
		case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MAX: /* UTIM2 */
		case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MAX: /* UTIM3 */
		case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MAX: /* UTIM4 */
		case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MAX: /* UTIM5 */
		case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MAX: /* UTIM6 */
		case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MAX: /* UTIM7 */
		case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MIN: /* DTIM1 */
		case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MIN: /* DTIM2 */
		case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MIN: /* DTIM3 */
		case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MIN: /* DTIM4 */
		case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MIN: /* DTIM5 */
		case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MIN: /* DTIM6 */
		case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MIN: /* DTIM7 */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		default:
			ret = -EINVAL;
			goto exit;
		}

		st->fpsr[attr] = FIELD_GET(GENMASK(7, 0), value);
		break;
	case MAX42500_DEV_PART_WDOG:
		switch (attr) {
		case MAX42500_ATTR_WDOG_WD_STATUS: /* WDSTAT */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
			value = FIELD_GET(GENMASK(5, 0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_SIMPLE_MODE_ENABLE: /* SWW */
			value = FIELD_GET(BIT(6), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
			value = FIELD_GET(GENMASK(3, 0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
			value = FIELD_GET(GENMASK(7, 4), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
			value = FIELD_GET(GENMASK(2, 0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_ENABLE: /* WDEN */
			value = FIELD_GET(BIT(3), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_KEY: /* WDKEY */
			value = FIELD_GET(GENMASK(7, 0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_LOCK_ENABLE: /* WDLOCK */
			value = FIELD_GET(BIT(0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
			value = FIELD_GET(GENMASK(1, 0), reg_val);
			break;
		case MAX42500_ATTR_WDOG_WD_RESET_TWO_COUNT_ENABLE: /* MR1 */
			value = FIELD_GET(BIT(2), reg_val);
			break;
		default:
			ret = -EINVAL;
			goto exit;
		}

		st->wdog[attr] = FIELD_GET(GENMASK(7, 0), value);
		break;
	default:
		ret = -EINVAL;
	}
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int max42500_chip_set_update(struct max42500_state *st, long value,
				    u8 attr)
{
	struct i2c_client *client = st->client;
	bool pec_flag = false;
	unsigned int reg_val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, max42500_reg_chip[attr], &reg_val);
	if (ret)
		goto exit;

	switch (attr) {
	case MAX42500_ATTR_CHIP_LABEL: /* ID */
	case MAX42500_ATTR_CHIP_NAME: /* CID */
	case MAX42500_ATTR_CHIP_CONFIG_STATUS: /* CFG2 */
		/* The register is read-only but can get updates. */
		st->chip[attr] = FIELD_GET(GENMASK(7, 0), reg_val);
		goto exit;
	case MAX42500_ATTR_CHIP_PEC_ENABLE: /* PECE */
		reg_val |= FIELD_PREP(BIT(0), value);
		pec_flag = true;
		break;
	case MAX42500_ATTR_CHIP_BIST_RESET_ENABLE: /* MBST */
		reg_val |= FIELD_PREP(BIT(1), value);
		break;
	case MAX42500_ATTR_CHIP_RELOAD_OTP_ENABLE: /* RR */
		reg_val |= FIELD_PREP(BIT(2), value);
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_write(st->regmap, max42500_reg_chip[attr], reg_val);
	if (ret)
		goto exit;

	/* Set the PEC of the device first then set in here after. */
	if (pec_flag)
		client->flags |= FIELD_PREP(BIT(I2C_CLIENT_PEC), value);

	st->chip[attr] = FIELD_GET(GENMASK(7, 0), value);
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int max42500_vmon_set_update(struct max42500_state *st, long value,
				    u8 attr)
{
	unsigned int reg_val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, max42500_reg_vmon[attr], &reg_val);
	if (ret)
		goto exit;

	switch (attr) {
	case MAX42500_ATTR_VMON_IN1_ENABLE: /* VIN1 */
	case MAX42500_ATTR_VMON_IN1_RESET_ENABLE: /* RST1 */
		reg_val |= FIELD_PREP(BIT(0), value);
		break;
	case MAX42500_ATTR_VMON_IN2_ENABLE: /* VIN2 */
	case MAX42500_ATTR_VMON_IN2_RESET_ENABLE: /* RST2 */
		reg_val |= FIELD_PREP(BIT(1), value);
		break;
	case MAX42500_ATTR_VMON_IN3_ENABLE: /* VIN3 */
	case MAX42500_ATTR_VMON_IN3_RESET_ENABLE: /* RST3 */
		reg_val |= FIELD_PREP(BIT(2), value);
		break;
	case MAX42500_ATTR_VMON_IN4_ENABLE: /* VIN4 */
	case MAX42500_ATTR_VMON_IN4_RESET_ENABLE: /* RST4 */
		reg_val |= FIELD_PREP(BIT(3), value);
		break;
	case MAX42500_ATTR_VMON_IN5_ENABLE: /* VIN5 */
	case MAX42500_ATTR_VMON_IN5_RESET_ENABLE: /* RST5 */
		reg_val |= FIELD_PREP(BIT(4), value);
		break;
	case MAX42500_ATTR_VMON_IN6_ENABLE: /* VIN6 */
	case MAX42500_ATTR_VMON_IN6_RESET_ENABLE: /*RST6 */
		reg_val |= FIELD_PREP(BIT(5), value);
		break;
	case MAX42500_ATTR_VMON_IN7_ENABLE: /* VIN7 */
	case MAX42500_ATTR_VMON_IN7_RESET_ENABLE: /* RST7 */
		reg_val |= FIELD_PREP(BIT(6), value);
		break;
	case MAX42500_ATTR_VMON_PDN_ENABLE: /* VMPD */
	case MAX42500_ATTR_VMON_PAR_ENABLE: /* PARM */
		reg_val |= FIELD_PREP(BIT(7), value);
		break;
	case MAX42500_ATTR_VMON_VMON_STATUS_OV: /* STATOV */
	case MAX42500_ATTR_VMON_VMON_STATUS_UV: /* STATUV */
	case MAX42500_ATTR_VMON_VMON_STATUS_OFF: /* STATOFF */
		/* The register is read-only but can get updates. */
		st->vmon[attr] = FIELD_GET(GENMASK(7, 0), reg_val);
		goto exit;
	case MAX42500_ATTR_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_ATTR_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_ATTR_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_ATTR_VMON_IN4_NOMINAL: /* VIN4 */
	case MAX42500_ATTR_VMON_IN5_NOMINAL: /* VIN5 */
	case MAX42500_ATTR_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_ATTR_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_ATTR_VMON_IN7_CRIT: /* VINO7 */
	case MAX42500_ATTR_VMON_IN7_LCRIT: /* VINU7 */
		reg_val |= FIELD_PREP(GENMASK(7, 0), value);
		break;
	case MAX42500_ATTR_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_ATTR_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_ATTR_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_ATTR_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_ATTR_VMON_IN5_OV_THRESH: /* VINO5 */
		reg_val |= FIELD_PREP(GENMASK(7, 4), value);
		break;
	case MAX42500_ATTR_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_ATTR_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_ATTR_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_ATTR_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_ATTR_VMON_IN5_UV_THRESH: /* VINU5 */
		reg_val |= FIELD_PREP(GENMASK(3, 0), value);
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_write(st->regmap, max42500_reg_vmon[attr], reg_val);
	if (ret)
		goto exit;

	st->vmon[attr] = FIELD_GET(GENMASK(7, 0), value);
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int max42500_fpsr_set_update(struct max42500_state *st, long value,
				    u8 attr)
{
	unsigned int reg_val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, max42500_reg_fpsr[attr], &reg_val);
	if (ret)
		goto exit;

	switch (attr) {
	case MAX42500_ATTR_FPSR_FPS_STATUS: /* FPSSTAT1 */
		/* The register is read-only but can get updates. */
		st->fpsr[attr] = FIELD_GET(GENMASK(7, 0), reg_val);
		goto exit;
	case MAX42500_ATTR_FPSR_FPS_CLOCK_DIVIDER: /* FDIV */
		reg_val |= FIELD_PREP(GENMASK(2, 0), value);
		break;
	case MAX42500_ATTR_FPSR_FPS_EN1_START_TIMER_ENABLE: /* FPSEN1 */
		reg_val |= FIELD_PREP(BIT(3), value);
		break;
	case MAX42500_ATTR_FPSR_POWERDN_NO_INTERRUPT_ENABLE: /* DVALM */
		reg_val |= FIELD_PREP(BIT(4), value);
		break;
	case MAX42500_ATTR_FPSR_POWERUP_NO_INTERRUPT_ENABLE: /* UVALM */
		reg_val |= FIELD_PREP(BIT(5), value);
		break;
	case MAX42500_ATTR_FPSR_POWERDN_SEQ_CAPTURE_FINISH: /* DVAL */
		reg_val |= FIELD_PREP(BIT(6), value);
		break;
	case MAX42500_ATTR_FPSR_POWERUP_SEQ_CAPTURE_FINISH: /* UVAL */
		reg_val |= FIELD_PREP(BIT(7), value);
		break;
	case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MAX: /* UTIM1 */
	case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MAX: /* UTIM2 */
	case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MAX: /* UTIM3 */
	case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MAX: /* UTIM4 */
	case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MAX: /* UTIM5 */
	case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MAX: /* UTIM6 */
	case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MAX: /* UTIM7 */
	case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MIN: /* DTIM1 */
	case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MIN: /* DTIM2 */
	case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MIN: /* DTIM3 */
	case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MIN: /* DTIM4 */
	case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MIN: /* DTIM5 */
	case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MIN: /* DTIM6 */
	case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MIN: /* DTIM7 */
		/* The register is read-only but can get updates. */
		st->fpsr[attr] = FIELD_GET(GENMASK(7, 0), reg_val);
		goto exit;
	default:
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_write(st->regmap, max42500_reg_fpsr[attr], reg_val);
	if (ret)
		goto exit;

	st->fpsr[attr] = FIELD_GET(GENMASK(7, 0), value);
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int max42500_wdog_set_update(struct max42500_state *st, long value,
				    u8 attr)
{
	unsigned int reg_val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, max42500_reg_wdog[attr], &reg_val);
	if (ret)
		goto exit;

	switch (attr) {
	case MAX42500_ATTR_WDOG_WD_STATUS: /* WDSTAT */
		/* The register is read-only but can get updates. */
		st->wdog[attr] = FIELD_GET(GENMASK(7, 0), reg_val);
		goto exit;
	case MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		reg_val |= FIELD_PREP(GENMASK(5, 0), value);
		break;
	case MAX42500_ATTR_WDOG_WD_SIMPLE_MODE_ENABLE: /* SWW */
		reg_val |= FIELD_PREP(BIT(6), value);
		break;
	case MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
		reg_val |= FIELD_PREP(GENMASK(3, 0), value);
		break;
	case MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		reg_val |= FIELD_PREP(GENMASK(7, 4), value);
		break;
	case MAX42500_ATTR_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		reg_val |= FIELD_PREP(GENMASK(2, 0), value);
		break;
	case MAX42500_ATTR_WDOG_WD_ENABLE: /* WDEN */
		reg_val |= FIELD_PREP(BIT(3), value);
		break;
	case MAX42500_ATTR_WDOG_WD_KEY: /* WDKEY */
		reg_val |= FIELD_PREP(GENMASK(7, 0), value);
		break;
	case MAX42500_ATTR_WDOG_WD_LOCK_ENABLE: /* WDLOCK */
		reg_val |= FIELD_PREP(BIT(0), value);
		break;
	case MAX42500_ATTR_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		reg_val |= FIELD_PREP(GENMASK(1, 0), value);
		break;
	case MAX42500_ATTR_WDOG_WD_RESET_TWO_COUNT_ENABLE: /* MR1 */
		reg_val |= FIELD_PREP(BIT(2), value);
		break;
	default:
		ret = -EINVAL;
		goto exit;
	}

	ret = regmap_write(st->regmap, max42500_reg_wdog[attr], reg_val);
	if (ret)
		goto exit;

	st->wdog[attr] = FIELD_GET(GENMASK(7, 0), value);
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static ssize_t chip_show(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	int ret;

	ret = max42500_get_updates(st, MAX42500_DEV_PART_CHIP, attr->index);
	if (ret < 0)
		return ret;

	switch (attr->index) {
	case MAX42500_ATTR_CHIP_LABEL: /* ID */
	case MAX42500_ATTR_CHIP_NAME: /* CID */
	case MAX42500_ATTR_CHIP_CONFIG_STATUS: /* CFG2 */
		/* Show the device ID and status in hexadecimals. */
		ret = sprintf(buf, "%x\n", st->chip[attr->index]);
		break;
	default:
		/* Show enabled and disabled bits from registers. */
		ret = sprintf(buf, "%d\n", st->chip[attr->index]);
		break;
	}

	return ret;
}

static ssize_t vmon_show(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	long value, vmon;
	int ret;

	ret = max42500_get_updates(st, MAX42500_DEV_PART_VMON, attr->index);
	if (ret < 0)
		goto error;

	vmon = st->vmon[attr->index];

	switch (attr->index) {
	case MAX42500_ATTR_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_ATTR_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_ATTR_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_ATTR_VMON_IN4_NOMINAL: /* VIN4 */
		value = convert_vin_nominal_from_reg(vmon);
		break;
	case MAX42500_ATTR_VMON_IN5_NOMINAL: /* VIN5 */
		value = convert_vin5_nominal_from_reg(vmon);
		break;
	case MAX42500_ATTR_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_ATTR_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_ATTR_VMON_IN7_CRIT: /* VINO7 */
	case MAX42500_ATTR_VMON_IN7_LCRIT: /* VINU7 */
		value = convert_vin_critical_from_reg(vmon);
		break;
	case MAX42500_ATTR_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_ATTR_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_ATTR_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_ATTR_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_ATTR_VMON_IN5_OV_THRESH: /* VINO5 */
		value = convert_ov_threshold_from_reg(vmon);
		break;
	case MAX42500_ATTR_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_ATTR_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_ATTR_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_ATTR_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_ATTR_VMON_IN5_UV_THRESH: /* VINU5 */
		value = convert_uv_threshold_from_reg(vmon);
		break;
	default:
		value = vmon;
		goto exit;
	}
	if (value < 0) {
		ret = value;
		goto error;
	}

	/* Show voltage (uV, mV, %) converted values from register. */
	return sprintf(buf, "%ld\n", value);
exit:
	/* Show enabled/disabled bits and status in hexadecimals. */
	return sprintf(buf, "%x\n", (unsigned int)value);
error:
	return ret;
}

static ssize_t fpsr_show(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	const u8 clk_div = MAX42500_ATTR_FPSR_FPS_CLOCK_DIVIDER;
	long value, fpsr;
	int ret;

	ret = max42500_get_updates(st, MAX42500_DEV_PART_FPSR, attr->index);
	if (ret < 0)
		goto error;

	fpsr = st->fpsr[attr->index];

	switch (attr->index) {
	case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MAX: /* UTIM1 */
	case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MAX: /* UTIM2 */
	case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MAX: /* UTIM3 */
	case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MAX: /* UTIM4 */
	case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MAX: /* UTIM5 */
	case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MAX: /* UTIM6 */
	case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MAX: /* UTIM7 */
	case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MIN: /* DTIM1 */
	case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MIN: /* DTIM2 */
	case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MIN: /* DTIM3 */
	case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MIN: /* DTIM4 */
	case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MIN: /* DTIM5 */
	case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MIN: /* DTIM6 */
	case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MIN: /* DTIM7 */
		/* Get the latest FPS clock divider for the timestamp */
		ret = max42500_get_updates(st, MAX42500_DEV_PART_FPSR, clk_div);
		if (ret < 0)
			goto error;

		/* Clear UVAL/DVAL bits and trigger EN0/EN1 to start timer */
		value = convert_timestamp_from_reg(fpsr, st->fpsr[clk_div]);
		break;
	default:
		value = fpsr;
		goto exit;
	}
	if (value < 0) {
		ret = value;
		goto error;
	}

	/* Show the time (us and ms) converted values from register. */
	return sprintf(buf, "%ld\n", value);
exit:
	/* Show enabled/disabled bits and status in hexadecimals. */
	return sprintf(buf, "%x\n", (unsigned int)value);
error:
	return ret;
}

static ssize_t wdog_show(struct device *dev, struct device_attribute *devattr,
			 char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	const u8 clk_div = MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER;
	const u8 opn_win = MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL;
	const u8 clo_win = MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL;
	long value, wdog;
	int ret;

	ret = max42500_get_updates(st, MAX42500_DEV_PART_WDOG, attr->index);
	if (ret < 0)
		goto error;

	wdog = st->wdog[attr->index];

	switch (attr->index) {
	case MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		value = convert_wd_clkdiv_from_reg(wdog);
		break;
	case MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
	case MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		ret = max42500_get_updates(st, MAX42500_DEV_PART_WDOG, clk_div);
		if (ret < 0)
			goto error;

		value = convert_wd_openclose_from_reg(wdog, st->wdog[clk_div]);
		break;
	case MAX42500_ATTR_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		ret = max42500_get_updates(st, MAX42500_DEV_PART_WDOG, clo_win);
		if (ret < 0)
			goto error;

		ret = max42500_get_updates(st, MAX42500_DEV_PART_WDOG, opn_win);
		if (ret < 0)
			goto error;

		value = convert_wd_firstup_from_reg(wdog, st->wdog[clo_win],
						    st->wdog[opn_win]);
		break;
	case MAX42500_ATTR_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		value = convert_wd_holdreset_from_reg(wdog);
		break;
	default:
		value = wdog;
		goto exit;
	}
	if (value < 0) {
		ret = value;
		goto error;
	}

	/* Show the time (us and ms) converted values from register. */
	return sprintf(buf, "%ld\n", value);
exit:
	/* Show enabled/disabled bits and status in hexadecimals. */
	return sprintf(buf, "%x\n", (unsigned int)value);
error:
	return ret;
}

static ssize_t gpio_show(struct device *dev, struct device_attribute *attr,
			 char *buf)
{
	struct max42500_state *st = dev_get_drvdata(dev);
	int idx, len = 0;

	for (idx = 0; idx < st->num_gpios; idx++) {
		len += sprintf(buf + len, "%d: %d\n", idx,
			       gpiod_get_value(st->gpios[idx]));
	}

	return len;
}

static ssize_t chip_store(struct device *dev,
			  struct device_attribute *devattr, const char *buf,
			  size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	long chip;
	int ret;

	ret = kstrtol(buf, 10, &chip);
	if (ret < 0)
		return ret;

	ret = max42500_chip_set_update(st, chip, attr->index);
	if (ret < 0)
		return ret;

	return count;
}

static ssize_t vmon_store(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	long value, vmon;
	int ret;

	ret = kstrtol(buf, 10, &value);
	if (ret < 0)
		goto error;

	switch (attr->index) {
	case MAX42500_ATTR_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_ATTR_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_ATTR_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_ATTR_VMON_IN4_NOMINAL: /* VIN4 */
		vmon = convert_vin_nominal_to_reg(value);
		break;
	case MAX42500_ATTR_VMON_IN5_NOMINAL: /* VIN5 */
		vmon = convert_vin5_nominal_to_reg(value);
		break;
	case MAX42500_ATTR_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_ATTR_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_ATTR_VMON_IN7_CRIT: /* VINO7 */
	case MAX42500_ATTR_VMON_IN7_LCRIT: /* VINU7 */
		vmon = convert_vin_critical_to_reg(value);
		break;
	case MAX42500_ATTR_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_ATTR_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_ATTR_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_ATTR_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_ATTR_VMON_IN5_OV_THRESH: /* VINO5 */
		vmon = convert_ov_threshold_to_reg(value);
		break;
	case MAX42500_ATTR_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_ATTR_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_ATTR_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_ATTR_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_ATTR_VMON_IN5_UV_THRESH: /* VINU5 */
		vmon = convert_uv_threshold_to_reg(value);
		break;
	default:
		vmon = value;
		break;
	}
	if (vmon < 0) {
		ret = vmon;
		goto error;
	}

	ret = max42500_vmon_set_update(st, vmon, attr->index);
	if (ret < 0)
		goto error;

	return count;
error:
	return ret;
}

static ssize_t fpsr_store(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	const u8 clk_div = MAX42500_ATTR_FPSR_FPS_CLOCK_DIVIDER;
	long value, fpsr;
	int ret;

	ret = kstrtol(buf, 10, &value);
	if (ret < 0)
		goto error;

	switch (attr->index) {
	case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MAX: /* UTIM1 */
	case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MAX: /* UTIM2 */
	case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MAX: /* UTIM3 */
	case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MAX: /* UTIM4 */
	case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MAX: /* UTIM5 */
	case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MAX: /* UTIM6 */
	case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MAX: /* UTIM7 */
	case MAX42500_ATTR_FPSR_POWER1_AVERAGE_INTERVAL_MIN: /* DTIM1 */
	case MAX42500_ATTR_FPSR_POWER2_AVERAGE_INTERVAL_MIN: /* DTIM2 */
	case MAX42500_ATTR_FPSR_POWER3_AVERAGE_INTERVAL_MIN: /* DTIM3 */
	case MAX42500_ATTR_FPSR_POWER4_AVERAGE_INTERVAL_MIN: /* DTIM4 */
	case MAX42500_ATTR_FPSR_POWER5_AVERAGE_INTERVAL_MIN: /* DTIM5 */
	case MAX42500_ATTR_FPSR_POWER6_AVERAGE_INTERVAL_MIN: /* DTIM6 */
	case MAX42500_ATTR_FPSR_POWER7_AVERAGE_INTERVAL_MIN: /* DTIM7 */
		fpsr = convert_timestamp_to_reg(value, st->fpsr[clk_div]);
		break;
	default:
		fpsr = value;
		break;
	}
	if (fpsr < 0) {
		ret = fpsr;
		goto error;
	}

	ret = max42500_fpsr_set_update(st, fpsr, attr->index);
	if (ret < 0)
		goto error;

	return count;
error:
	return ret;
}

static ssize_t wdog_store(struct device *dev, struct device_attribute *devattr,
			  const char *buf, size_t count)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(devattr);
	struct max42500_state *st = dev_get_drvdata(dev);
	const u8 clk_div = MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER;
	const u8 win_mode = MAX42500_ATTR_WDOG_WD_SIMPLE_MODE_ENABLE;
	const u8 opn_win = MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL;
	const u8 clo_win = MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL;
	long value, wdog;
	int ret;

	if (attr->index != MAX42500_ATTR_WDOG_WD_KEY) {
		ret = kstrtol(buf, 10, &value);
		if (ret < 0)
			goto error;
	}

	switch (attr->index) {
	case MAX42500_ATTR_WDOG_WD_CLOCK_DIVIDER: /* WDIV */
		wdog = convert_wd_clkdiv_to_reg(value);
		break;
	case MAX42500_ATTR_WDOG_WD_OPEN_WINDOW_INTERVAL: /* WDOPEN */
	case MAX42500_ATTR_WDOG_WD_CLOSE_WINDOW_INTERVAL: /* WDCLOSE */
		wdog = convert_wd_openclose_to_reg(value, st->fpsr[clk_div]);
		break;
	case MAX42500_ATTR_WDOG_WD_FIRST_UPDATE_INTERVAL: /* 1UD */
		wdog = convert_wd_firstup_to_reg(value, st->fpsr[clo_win],
						 st->fpsr[opn_win]);
		break;
	case MAX42500_ATTR_WDOG_WD_RESET_HOLD_INTERVAL: /* RHLD */
		/* Valid values are 0, 8, 16, 32 and truncates larger values */
		wdog = convert_wd_holdreset_to_reg(value);
		break;
	case MAX42500_ATTR_WDOG_WD_KEY: /* WDKEY */
		/* User cannot input watchdog key but can trigger key update  */
		wdog = max42500_prep_watchdog_key(st->wdog[win_mode],
						  st->wdog[attr->index]);
		break;
	default:
		wdog = value;
		break;
	}
	if (wdog < 0) {
		ret = wdog;
		goto error;
	}

	ret = max42500_wdog_set_update(st, wdog, attr->index);
	if (ret < 0)
		goto error;

	return count;
error:
	return ret;
}

static ssize_t gpio_store(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct max42500_state *data = dev_get_drvdata(dev);
	int gpio_num, value;

	if (sscanf(buf, "%d %d", &gpio_num, &value) == 2) {
		if (gpio_num >= 0 && gpio_num < data->num_gpios)
			gpiod_set_value(data->gpios[gpio_num], value);
	}

	return count;
}

static SENSOR_DEVICE_ATTR_RO(label, chip, 0);
static SENSOR_DEVICE_ATTR_RO(name, chip, 1);
static SENSOR_DEVICE_ATTR_RW(reload_otp_enable, chip, 2);
static SENSOR_DEVICE_ATTR_RW(bist_reset_enable, chip, 3);
static SENSOR_DEVICE_ATTR_RW(pec_enable, chip, 4);
static SENSOR_DEVICE_ATTR_RO(config_status, chip, 5);

static SENSOR_DEVICE_ATTR_RW(in1_enable, vmon, 0);
static SENSOR_DEVICE_ATTR_RW(in2_enable, vmon, 1);
static SENSOR_DEVICE_ATTR_RW(in3_enable, vmon, 2);
static SENSOR_DEVICE_ATTR_RW(in4_enable, vmon, 3);
static SENSOR_DEVICE_ATTR_RW(in5_enable, vmon, 4);
static SENSOR_DEVICE_ATTR_RW(in6_enable, vmon, 5);
static SENSOR_DEVICE_ATTR_RW(in7_enable, vmon, 6);
static SENSOR_DEVICE_ATTR_RW(pdn_enable, vmon, 7);
static SENSOR_DEVICE_ATTR_RW(in1_reset_enable, vmon, 8);
static SENSOR_DEVICE_ATTR_RW(in2_reset_enable, vmon, 9);
static SENSOR_DEVICE_ATTR_RW(in3_reset_enable, vmon, 10);
static SENSOR_DEVICE_ATTR_RW(in4_reset_enable, vmon, 11);
static SENSOR_DEVICE_ATTR_RW(in5_reset_enable, vmon, 12);
static SENSOR_DEVICE_ATTR_RW(in6_reset_enable, vmon, 13);
static SENSOR_DEVICE_ATTR_RW(in7_reset_enable, vmon, 14);
static SENSOR_DEVICE_ATTR_RW(par_reset_enable, vmon, 15);
static SENSOR_DEVICE_ATTR_RO(in_status_ov, vmon, 16);
static SENSOR_DEVICE_ATTR_RO(in_status_uv, vmon, 17);
static SENSOR_DEVICE_ATTR_RO(in_status_off, vmon, 18);
static SENSOR_DEVICE_ATTR_RW(in1_nominal, vmon, 19);
static SENSOR_DEVICE_ATTR_RW(in2_nominal, vmon, 20);
static SENSOR_DEVICE_ATTR_RW(in3_nominal, vmon, 21);
static SENSOR_DEVICE_ATTR_RW(in4_nominal, vmon, 22);
static SENSOR_DEVICE_ATTR_RW(in5_nominal, vmon, 23);
static SENSOR_DEVICE_ATTR_RW(in6_crit, vmon, 24);
static SENSOR_DEVICE_ATTR_RW(in6_lcrit, vmon, 25);
static SENSOR_DEVICE_ATTR_RW(in7_crit, vmon, 26);
static SENSOR_DEVICE_ATTR_RW(in7_lcrit, vmon, 27);
static SENSOR_DEVICE_ATTR_RW(in1_ov_thresh, vmon, 28);
static SENSOR_DEVICE_ATTR_RW(in1_uv_thresh, vmon, 29);
static SENSOR_DEVICE_ATTR_RW(in2_ov_thresh, vmon, 30);
static SENSOR_DEVICE_ATTR_RW(in2_uv_thresh, vmon, 31);
static SENSOR_DEVICE_ATTR_RW(in3_ov_thresh, vmon, 32);
static SENSOR_DEVICE_ATTR_RW(in3_uv_thresh, vmon, 33);
static SENSOR_DEVICE_ATTR_RW(in4_ov_thresh, vmon, 34);
static SENSOR_DEVICE_ATTR_RW(in4_uv_thresh, vmon, 35);
static SENSOR_DEVICE_ATTR_RW(in5_ov_thresh, vmon, 36);
static SENSOR_DEVICE_ATTR_RW(in5_uv_thresh, vmon, 37);

static SENSOR_DEVICE_ATTR_RO(fpsr_status, fpsr, 0);
static SENSOR_DEVICE_ATTR_RW(powerup_seq_capture_finish, fpsr, 1);
static SENSOR_DEVICE_ATTR_RW(powerdn_seq_capture_finish, fpsr, 2);
static SENSOR_DEVICE_ATTR_RW(powerup_no_interrupt_enable, fpsr, 3);
static SENSOR_DEVICE_ATTR_RW(powerdn_no_interrupt_enable, fpsr, 4);
static SENSOR_DEVICE_ATTR_RW(fps_en1_start_timer_enable, fpsr, 5);
static SENSOR_DEVICE_ATTR_RW(fps_clock_divider, fpsr, 6);
static SENSOR_DEVICE_ATTR_RO(power1_average_interval_max, fpsr, 7);
static SENSOR_DEVICE_ATTR_RO(power2_average_interval_max, fpsr, 8);
static SENSOR_DEVICE_ATTR_RO(power3_average_interval_max, fpsr, 9);
static SENSOR_DEVICE_ATTR_RO(power4_average_interval_max, fpsr, 10);
static SENSOR_DEVICE_ATTR_RO(power5_average_interval_max, fpsr, 11);
static SENSOR_DEVICE_ATTR_RO(power6_average_interval_max, fpsr, 12);
static SENSOR_DEVICE_ATTR_RO(power7_average_interval_max, fpsr, 13);
static SENSOR_DEVICE_ATTR_RO(power1_average_interval_min, fpsr, 14);
static SENSOR_DEVICE_ATTR_RO(power2_average_interval_min, fpsr, 15);
static SENSOR_DEVICE_ATTR_RO(power3_average_interval_min, fpsr, 16);
static SENSOR_DEVICE_ATTR_RO(power4_average_interval_min, fpsr, 17);
static SENSOR_DEVICE_ATTR_RO(power5_average_interval_min, fpsr, 18);
static SENSOR_DEVICE_ATTR_RO(power6_average_interval_min, fpsr, 19);
static SENSOR_DEVICE_ATTR_RO(power7_average_interval_min, fpsr, 20);

static SENSOR_DEVICE_ATTR_RO(wd_status, wdog, 0);
static SENSOR_DEVICE_ATTR_RW(wd_simple_mode_enable, wdog, 1);
static SENSOR_DEVICE_ATTR_RW(wd_clock_divider, wdog, 2);
static SENSOR_DEVICE_ATTR_RW(wd_open_window_interval, wdog, 3);
static SENSOR_DEVICE_ATTR_RW(wd_close_window_interval, wdog, 4);
static SENSOR_DEVICE_ATTR_RW(wd_enable, wdog, 5);
static SENSOR_DEVICE_ATTR_RW(wd_first_update_interval, wdog, 6);
static SENSOR_DEVICE_ATTR_RW(wd_key, wdog, 7);
static SENSOR_DEVICE_ATTR_RW(wd_lock_enable, wdog, 8);
static SENSOR_DEVICE_ATTR_RW(wd_reset_two_count_enable, wdog, 9);
static SENSOR_DEVICE_ATTR_RW(wd_reset_hold_interval, wdog, 10);

static SENSOR_DEVICE_ATTR_RW(en, gpio, 0);

static struct attribute *max42500_attrs[] = {
	&sensor_dev_attr_label.dev_attr.attr,
	&sensor_dev_attr_name.dev_attr.attr,
	&sensor_dev_attr_reload_otp_enable.dev_attr.attr,
	&sensor_dev_attr_bist_reset_enable.dev_attr.attr,
	&sensor_dev_attr_pec_enable.dev_attr.attr,
	&sensor_dev_attr_config_status.dev_attr.attr,
	&sensor_dev_attr_in1_enable.dev_attr.attr,
	&sensor_dev_attr_in2_enable.dev_attr.attr,
	&sensor_dev_attr_in3_enable.dev_attr.attr,
	&sensor_dev_attr_in4_enable.dev_attr.attr,
	&sensor_dev_attr_in5_enable.dev_attr.attr,
	&sensor_dev_attr_in6_enable.dev_attr.attr,
	&sensor_dev_attr_in7_enable.dev_attr.attr,
	&sensor_dev_attr_pdn_enable.dev_attr.attr,
	&sensor_dev_attr_in1_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in2_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in3_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in4_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in5_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in6_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in7_reset_enable.dev_attr.attr,
	&sensor_dev_attr_par_reset_enable.dev_attr.attr,
	&sensor_dev_attr_in_status_ov.dev_attr.attr,
	&sensor_dev_attr_in_status_uv.dev_attr.attr,
	&sensor_dev_attr_in_status_off.dev_attr.attr,
	&sensor_dev_attr_in1_nominal.dev_attr.attr,
	&sensor_dev_attr_in2_nominal.dev_attr.attr,
	&sensor_dev_attr_in3_nominal.dev_attr.attr,
	&sensor_dev_attr_in4_nominal.dev_attr.attr,
	&sensor_dev_attr_in5_nominal.dev_attr.attr,
	&sensor_dev_attr_in6_crit.dev_attr.attr,
	&sensor_dev_attr_in6_lcrit.dev_attr.attr,
	&sensor_dev_attr_in7_crit.dev_attr.attr,
	&sensor_dev_attr_in7_lcrit.dev_attr.attr,
	&sensor_dev_attr_in1_ov_thresh.dev_attr.attr,
	&sensor_dev_attr_in1_uv_thresh.dev_attr.attr,
	&sensor_dev_attr_in2_ov_thresh.dev_attr.attr,
	&sensor_dev_attr_in2_uv_thresh.dev_attr.attr,
	&sensor_dev_attr_in3_ov_thresh.dev_attr.attr,
	&sensor_dev_attr_in3_uv_thresh.dev_attr.attr,
	&sensor_dev_attr_in4_ov_thresh.dev_attr.attr,
	&sensor_dev_attr_in4_uv_thresh.dev_attr.attr,
	&sensor_dev_attr_in5_ov_thresh.dev_attr.attr,
	&sensor_dev_attr_in5_uv_thresh.dev_attr.attr,
	&sensor_dev_attr_fpsr_status.dev_attr.attr,
	&sensor_dev_attr_powerup_seq_capture_finish.dev_attr.attr,
	&sensor_dev_attr_powerdn_seq_capture_finish.dev_attr.attr,
	&sensor_dev_attr_powerup_no_interrupt_enable.dev_attr.attr,
	&sensor_dev_attr_powerdn_no_interrupt_enable.dev_attr.attr,
	&sensor_dev_attr_fps_en1_start_timer_enable.dev_attr.attr,
	&sensor_dev_attr_fps_clock_divider.dev_attr.attr,
	&sensor_dev_attr_power1_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power2_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power3_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power4_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power5_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power6_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power7_average_interval_max.dev_attr.attr,
	&sensor_dev_attr_power1_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_power2_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_power3_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_power4_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_power5_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_power6_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_power7_average_interval_min.dev_attr.attr,
	&sensor_dev_attr_wd_status.dev_attr.attr,
	&sensor_dev_attr_wd_simple_mode_enable.dev_attr.attr,
	&sensor_dev_attr_wd_clock_divider.dev_attr.attr,
	&sensor_dev_attr_wd_open_window_interval.dev_attr.attr,
	&sensor_dev_attr_wd_close_window_interval.dev_attr.attr,
	&sensor_dev_attr_wd_enable.dev_attr.attr,
	&sensor_dev_attr_wd_first_update_interval.dev_attr.attr,
	&sensor_dev_attr_wd_key.dev_attr.attr,
	&sensor_dev_attr_wd_lock_enable.dev_attr.attr,
	&sensor_dev_attr_wd_reset_two_count_enable.dev_attr.attr,
	&sensor_dev_attr_wd_reset_hold_interval.dev_attr.attr,
	&sensor_dev_attr_en.dev_attr.attr,
	NULL
};
ATTRIBUTE_GROUPS(max42500);

static const struct regmap_bus max42500_regmap_bus = {
	.reg_read = max42500_reg_read,
	.reg_write = max42500_reg_write,
};

static const struct regmap_config max42500_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX42500_REG_CID,
};

static int max42500_probe(struct i2c_client *client)
{
	struct max42500_state *st;
	struct device *hwmon_dev;
	struct regmap *regmap;
	struct device *dev = &client->dev;
	const struct attribute_group **attr_groups = max42500_groups;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	i2c_set_clientdata(client, st);

	st->client = client;
	regmap = devm_regmap_init(dev, &max42500_regmap_bus, client,
				  &max42500_regmap_config);
	if (IS_ERR(regmap))
		return PTR_ERR(regmap);

	st->regmap = regmap;

	st->num_gpios = gpiod_count(dev, "gpios");
	if (st->num_gpios < 0)
		return st->num_gpios;

	st->gpios = devm_kcalloc(dev, st->num_gpios, sizeof(*st->gpios),
				   GFP_KERNEL);
	if (!st->gpios)
		return -ENOMEM;

	for (int i = 0; i < st->num_gpios; i++) {
		st->gpios[i] = devm_gpiod_get_index(dev, "gpios", i,
						    GPIOD_OUT_HIGH);
		if (IS_ERR(st->gpios[i]))
			return PTR_ERR(st->gpios[i]);
	}

	st->gc.label = dev_name(dev);
	st->gc.parent = dev;
	st->gc.owner = THIS_MODULE;
	st->gc.base = -1;
	st->gc.ngpio = st->num_gpios;
	st->gc.can_sleep = true;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
							   client->name,
							   st, attr_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct of_device_id max42500_of_match[] = {
	{ .compatible = "adi,max42500" },
	{ }
};
MODULE_DEVICE_TABLE(of, max42500_of_match);

static const struct i2c_device_id max42500_id[] = {
	{ "max42500", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max42500_id);

static struct i2c_driver max42500_driver = {
	.driver = {
		.name	= "max42500",
		.of_match_table = max42500_of_match,
	},
	.probe		= max42500_probe,
	.id_table	= max42500_id,
};

module_i2c_driver(max42500_driver);

MODULE_AUTHOR("Kent Libetario <kent.libetario@analog.com>");
MODULE_DESCRIPTION("Hwmon driver for MAX42500");
MODULE_LICENSE("GPL");
