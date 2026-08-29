// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * MAX42500 - Industrial Power System Monitor
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/debugfs.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/mfd/max42500.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

enum MAX42500_chip {
	MAX42500_CHIP_LABEL,
	MAX42500_CHIP_NAME,
	MAX42500_CHIP_RELOAD_OTP_ENABLE,
	MAX42500_CHIP_BIST_RESET_ENABLE,
	MAX42500_CHIP_PEC_ENABLE,
	MAX42500_CHIP_CONFIG_STATUS,
	MAX42500_CHIP_MAX
};

static const u8 max42500_reg_chip[] = {
	MAX42500_REG_ID,
	MAX42500_REG_CID,
	MAX42500_REG_CONFIG1,
	MAX42500_REG_CONFIG1,
	MAX42500_REG_CONFIG1,
	MAX42500_REG_CONFIG2
};

enum MAX42500_vmon {
	MAX42500_VMON_IN1_ENABLE,
	MAX42500_VMON_IN2_ENABLE,
	MAX42500_VMON_IN3_ENABLE,
	MAX42500_VMON_IN4_ENABLE,
	MAX42500_VMON_IN5_ENABLE,
	MAX42500_VMON_IN6_ENABLE,
	MAX42500_VMON_IN7_ENABLE,
	MAX42500_VMON_PDN_ENABLE,
	MAX42500_VMON_IN1_RESET_ENABLE,
	MAX42500_VMON_IN2_RESET_ENABLE,
	MAX42500_VMON_IN3_RESET_ENABLE,
	MAX42500_VMON_IN4_RESET_ENABLE,
	MAX42500_VMON_IN5_RESET_ENABLE,
	MAX42500_VMON_IN6_RESET_ENABLE,
	MAX42500_VMON_IN7_RESET_ENABLE,
	MAX42500_VMON_PAR_ENABLE,
	MAX42500_VMON_OV_STATUS,
	MAX42500_VMON_UV_STATUS,
	MAX42500_VMON_OFF_STATUS,
	MAX42500_VMON_IN1_NOMINAL,
	MAX42500_VMON_IN2_NOMINAL,
	MAX42500_VMON_IN3_NOMINAL,
	MAX42500_VMON_IN4_NOMINAL,
	MAX42500_VMON_IN5_NOMINAL,
	MAX42500_VMON_IN1_OV_THRESH,
	MAX42500_VMON_IN2_OV_THRESH,
	MAX42500_VMON_IN3_OV_THRESH,
	MAX42500_VMON_IN4_OV_THRESH,
	MAX42500_VMON_IN5_OV_THRESH,
	MAX42500_VMON_IN6_CRIT,
	MAX42500_VMON_IN7_CRIT,
	MAX42500_VMON_IN1_UV_THRESH,
	MAX42500_VMON_IN2_UV_THRESH,
	MAX42500_VMON_IN3_UV_THRESH,
	MAX42500_VMON_IN4_UV_THRESH,
	MAX42500_VMON_IN5_UV_THRESH,
	MAX42500_VMON_IN6_LCRIT,
	MAX42500_VMON_IN7_LCRIT,
	MAX42500_VMON_MAX
};

static const u8 max42500_reg_vmon[] = {
	MAX42500_REG_VMON,
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
	MAX42500_REG_OVUV1,
	MAX42500_REG_OVUV2,
	MAX42500_REG_OVUV3,
	MAX42500_REG_OVUV4,
	MAX42500_REG_OVUV5,
	MAX42500_REG_VINO6,
	MAX42500_REG_VINO7,
	MAX42500_REG_OVUV1,
	MAX42500_REG_OVUV2,
	MAX42500_REG_OVUV3,
	MAX42500_REG_OVUV4,
	MAX42500_REG_OVUV5,
	MAX42500_REG_VINU6,
	MAX42500_REG_VINU7
};

enum max42500_fpsr {
	MAX42500_FPSR_FPS_STATUS,
	MAX42500_FPSR_POWERUP_SEQ_CAPTURE_FINISH,
	MAX42500_FPSR_POWERDN_SEQ_CAPTURE_FINISH,
	MAX42500_FPSR_POWERUP_NO_INTERRUPT_ENABLE,
	MAX42500_FPSR_POWERDN_NO_INTERRUPT_ENABLE,
	MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE,
	MAX42500_FPSR_FPS_CLOCK_DIVIDER,
	MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER2_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER3_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER4_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER5_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER6_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER7_AVERAGE_INTERVAL_MAX,
	MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_POWER2_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_POWER3_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_POWER4_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_POWER5_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_POWER6_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_POWER7_AVERAGE_INTERVAL_MIN,
	MAX42500_FPSR_MAX
};

static const u8 max42500_reg_fpsr[] = {
	MAX42500_REG_FPSTAT1,
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

struct max42500_state {
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex lock; /* Protects access during data transfer */
	struct gpio_desc *power_gpio;
	struct gpio_desc *sleep_gpio;
	u8 chip[MAX42500_CHIP_MAX];
	u8 vmon[MAX42500_VMON_MAX];
	u8 fpsr[MAX42500_FPSR_MAX];
};

static int max42500_chip_get_update(struct max42500_state *st,
				    enum MAX42500_chip index)
{
	unsigned int reg_val;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, max42500_reg_chip[index], &reg_val);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_CHIP_LABEL: /* ID */
		st->chip[index] = MAX42500_CHIP_ID(reg_val);
		return 0;
	case MAX42500_CHIP_NAME: /* CID */
		st->chip[index] = MAX42500_CHIP_CID(reg_val);
		return 0;
	case MAX42500_CHIP_CONFIG_STATUS: /* CFG2 */
		st->chip[index] = MAX42500_CHIP_CFG2(reg_val);
		return 0;
	case MAX42500_CHIP_PEC_ENABLE: /* PECE */
		st->chip[index] = MAX42500_CHIP_PECE(reg_val);
		return 0;
	case MAX42500_CHIP_BIST_RESET_ENABLE: /* MBST */
		st->chip[index] = MAX42500_CHIP_MBST(reg_val);
		return 0;
	case MAX42500_CHIP_RELOAD_OTP_ENABLE: /* RR */
		st->chip[index] = MAX42500_CHIP_RR(reg_val);
		return 0;
	default:
		return -EINVAL;
	}
}

static int max42500_vmon_get_update(struct max42500_state *st,
				    enum MAX42500_vmon index)
{
	unsigned int reg_val;
	int ret = 0;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, max42500_reg_vmon[index], &reg_val);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_VMON_IN1_ENABLE: /* VIN1 */
		st->vmon[index] = MAX42500_VMON_VM1(reg_val);
		return 0;
	case MAX42500_VMON_IN1_RESET_ENABLE: /* RST1 */
		st->vmon[index] = MAX42500_VMON_RST1(reg_val);
		return 0;
	case MAX42500_VMON_IN2_ENABLE: /* VIN2 */
		st->vmon[index] = MAX42500_VMON_VM2(reg_val);
		return 0;
	case MAX42500_VMON_IN2_RESET_ENABLE: /* RST2 */
		st->vmon[index] = MAX42500_VMON_RST2(reg_val);
		return 0;
	case MAX42500_VMON_IN3_ENABLE: /* VIN3 */
		st->vmon[index] = MAX42500_VMON_VM3(reg_val);
		return 0;
	case MAX42500_VMON_IN3_RESET_ENABLE: /* RST3 */
		st->vmon[index] = MAX42500_VMON_RST3(reg_val);
		return 0;
	case MAX42500_VMON_IN4_ENABLE: /* VIN4 */
		st->vmon[index] = MAX42500_VMON_VM4(reg_val);
		return 0;
	case MAX42500_VMON_IN4_RESET_ENABLE: /* RST4 */
		st->vmon[index] = MAX42500_VMON_RST4(reg_val);
		return 0;
	case MAX42500_VMON_IN5_ENABLE: /* VIN5 */
		st->vmon[index] = MAX42500_VMON_VM5(reg_val);
		return 0;
	case MAX42500_VMON_IN5_RESET_ENABLE: /* RST5 */
		st->vmon[index] = MAX42500_VMON_RST5(reg_val);
		return 0;
	case MAX42500_VMON_IN6_ENABLE: /* VIN6 */
		st->vmon[index] = MAX42500_VMON_VM6(reg_val);
		return 0;
	case MAX42500_VMON_IN6_RESET_ENABLE: /*RST6 */
		st->vmon[index] = MAX42500_VMON_RST6(reg_val);
		return 0;
	case MAX42500_VMON_IN7_ENABLE: /* VIN7 */
		st->vmon[index] = MAX42500_VMON_VM7(reg_val);
		return 0;
	case MAX42500_VMON_IN7_RESET_ENABLE: /* RST7 */
		st->vmon[index] = MAX42500_VMON_RST7(reg_val);
		return 0;
	case MAX42500_VMON_PDN_ENABLE: /* VMPD */
		st->vmon[index] = MAX42500_VMON_VMPD(reg_val);
		return 0;
	case MAX42500_VMON_PAR_ENABLE: /* PARM */
		st->vmon[index] = MAX42500_VMON_PARM(reg_val);
		return 0;
	case MAX42500_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_VMON_IN4_NOMINAL: /* VIN4 */
	case MAX42500_VMON_IN5_NOMINAL: /* VIN5 */
		st->vmon[index] = MAX42500_VMON_1VIN5(reg_val);
		return 0;
	case MAX42500_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_VMON_IN5_UV_THRESH: /* VINU5 */
		st->vmon[index] = MAX42500_VMON_1VINU5(reg_val);
		return 0;
	case MAX42500_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_VMON_IN7_LCRIT: /* VINU7 */
		st->vmon[index] = MAX42500_VMON_6VINU7(reg_val);
		return 0;
	case MAX42500_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_VMON_IN5_OV_THRESH: /* VINO5 */
		st->vmon[index] = MAX42500_VMON_1VINO5(reg_val);
		return 0;
	case MAX42500_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_VMON_IN7_CRIT: /* VINO7 */
		st->vmon[index] = MAX42500_VMON_6VINO7(reg_val);
		return 0;
	case MAX42500_VMON_OFF_STATUS: /* STATOFF */
		st->vmon[index] = MAX42500_VMON_STATOFF(reg_val);
		return 0;
	case MAX42500_VMON_UV_STATUS: /* STATUV */
		st->vmon[index] = MAX42500_VMON_STATUV(reg_val);
		return 0;
	case MAX42500_VMON_OV_STATUS: /* STATOV */
		st->vmon[index] = MAX42500_VMON_STATOV(reg_val);
		return 0;
	default:
		return -EINVAL;
	}
}

static int max42500_fpsr_get_update(struct max42500_state *st,
				    enum max42500_fpsr index)
{
	unsigned int reg_val;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, max42500_reg_fpsr[index], &reg_val);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_FPSR_FPS_STATUS: /* FPSTAT1 */
		st->fpsr[index] = MAX42500_FPSR_FPSTAT1(reg_val);
		return 0;
	case MAX42500_FPSR_FPS_CLOCK_DIVIDER: /* FDIV */
		st->fpsr[index] = MAX42500_FPSR_FDIV(reg_val);
		return 0;
	case MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE: /* FPSEN1 */
		st->fpsr[index] = MAX42500_FPSR_FPSEN1(reg_val);
		return 0;
	case MAX42500_FPSR_POWERDN_NO_INTERRUPT_ENABLE: /* DVALM */
		st->fpsr[index] = MAX42500_FPSR_DVALM(reg_val);
		return 0;
	case MAX42500_FPSR_POWERUP_NO_INTERRUPT_ENABLE: /* UVALM */
		st->fpsr[index] = MAX42500_FPSR_UVALM(reg_val);
		return 0;
	case MAX42500_FPSR_POWERDN_SEQ_CAPTURE_FINISH: /* DVAL */
		st->fpsr[index] = MAX42500_FPSR_DVAL(reg_val);
		return 0;
	case MAX42500_FPSR_POWERUP_SEQ_CAPTURE_FINISH: /* UVAL */
		st->fpsr[index] = MAX42500_FPSR_UVAL(reg_val);
		return 0;
	case MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MAX: /* UTIM1 */
	case MAX42500_FPSR_POWER2_AVERAGE_INTERVAL_MAX: /* UTIM2 */
	case MAX42500_FPSR_POWER3_AVERAGE_INTERVAL_MAX: /* UTIM3 */
	case MAX42500_FPSR_POWER4_AVERAGE_INTERVAL_MAX: /* UTIM4 */
	case MAX42500_FPSR_POWER5_AVERAGE_INTERVAL_MAX: /* UTIM5 */
	case MAX42500_FPSR_POWER6_AVERAGE_INTERVAL_MAX: /* UTIM6 */
	case MAX42500_FPSR_POWER7_AVERAGE_INTERVAL_MAX: /* UTIM7 */
		st->fpsr[index] = MAX42500_FPSR_1UTIM7(reg_val);
		return 0;
	case MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MIN: /* DTIM1 */
	case MAX42500_FPSR_POWER2_AVERAGE_INTERVAL_MIN: /* DTIM2 */
	case MAX42500_FPSR_POWER3_AVERAGE_INTERVAL_MIN: /* DTIM3 */
	case MAX42500_FPSR_POWER4_AVERAGE_INTERVAL_MIN: /* DTIM4 */
	case MAX42500_FPSR_POWER5_AVERAGE_INTERVAL_MIN: /* DTIM5 */
	case MAX42500_FPSR_POWER6_AVERAGE_INTERVAL_MIN: /* DTIM6 */
	case MAX42500_FPSR_POWER7_AVERAGE_INTERVAL_MIN: /* DTIM7 */
		st->fpsr[index] = MAX42500_FPSR_1DTIM7(reg_val);
		return 0;
	default:
		return -EINVAL;
	}
}

static int max42500_chip_set_update(struct max42500_state *st, long value,
				    enum MAX42500_chip index)
{
	bool pec_flag = false;
	unsigned int reg_mask;
	int ret;

	guard(mutex)(&st->lock);

	switch (index) {
	case MAX42500_CHIP_PEC_ENABLE: /* PECE */
		reg_mask = MAX42500_CHIP_PECE_MASK;
		pec_flag = true;
		break;
	case MAX42500_CHIP_BIST_RESET_ENABLE: /* MBST */
		reg_mask = MAX42500_CHIP_MBST_MASK;
		break;
	case MAX42500_CHIP_RELOAD_OTP_ENABLE: /* RR */
		reg_mask = MAX42500_CHIP_RR_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(st->regmap, max42500_reg_chip[index],
				 reg_mask, value);
	if (ret)
		return ret;

	/* Set the PEC of the device first then set in here after. */
	if (pec_flag)
		st->client->flags |= FIELD_PREP(BIT(I2C_CLIENT_PEC), value);

	/* Update after successful register write */
	st->chip[index] = FIELD_GET(GENMASK(7, 0), value);

	return 0;
}

static int max42500_vmon_set_update(struct max42500_state *st, long value,
				    enum MAX42500_vmon index)
{
	unsigned int reg_mask;
	int ret;

	guard(mutex)(&st->lock);

	switch (index) {
	case MAX42500_VMON_IN1_ENABLE: /* VIN1 */
		reg_mask = MAX42500_VMON_VM1_MASK;
		break;
	case MAX42500_VMON_IN1_RESET_ENABLE: /* RST1 */
		reg_mask = MAX42500_VMON_RST1_MASK;
		break;
	case MAX42500_VMON_IN2_ENABLE: /* VIN2 */
		reg_mask = MAX42500_VMON_VM2_MASK;
		break;
	case MAX42500_VMON_IN2_RESET_ENABLE: /* RST2 */
		reg_mask = MAX42500_VMON_RST2_MASK;
		break;
	case MAX42500_VMON_IN3_ENABLE: /* VIN3 */
		reg_mask = MAX42500_VMON_VM3_MASK;
		break;
	case MAX42500_VMON_IN3_RESET_ENABLE: /* RST3 */
		reg_mask = MAX42500_VMON_RST3_MASK;
		break;
	case MAX42500_VMON_IN4_ENABLE: /* VIN4 */
		reg_mask = MAX42500_VMON_VM4_MASK;
		break;
	case MAX42500_VMON_IN4_RESET_ENABLE: /* RST4 */
		reg_mask = MAX42500_VMON_RST4_MASK;
		break;
	case MAX42500_VMON_IN5_ENABLE: /* VIN5 */
		reg_mask = MAX42500_VMON_VM5_MASK;
		break;
	case MAX42500_VMON_IN5_RESET_ENABLE: /* RST5 */
		reg_mask = MAX42500_VMON_RST5_MASK;
		break;
	case MAX42500_VMON_IN6_ENABLE: /* VIN6 */
		reg_mask = MAX42500_VMON_VM6_MASK;
		break;
	case MAX42500_VMON_IN6_RESET_ENABLE: /*RST6 */
		reg_mask = MAX42500_VMON_RST6_MASK;
		break;
	case MAX42500_VMON_IN7_ENABLE: /* VIN7 */
		reg_mask = MAX42500_VMON_VM7_MASK;
		break;
	case MAX42500_VMON_IN7_RESET_ENABLE: /* RST7 */
		reg_mask = MAX42500_VMON_RST7_MASK;
		break;
	case MAX42500_VMON_PDN_ENABLE: /* VMPD */
		reg_mask = MAX42500_VMON_VMPD_MASK;
		break;
	case MAX42500_VMON_PAR_ENABLE: /* PARM */
		reg_mask = MAX42500_VMON_PARM_MASK;
		break;
	case MAX42500_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_VMON_IN4_NOMINAL: /* VIN4 */
	case MAX42500_VMON_IN5_NOMINAL: /* VIN5 */
		reg_mask = MAX42500_VMON_1VIN5_MASK;
		break;
	case MAX42500_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_VMON_IN5_UV_THRESH: /* VINU5 */
		reg_mask = MAX42500_VMON_1VINU5_MASK;
		break;
	case MAX42500_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_VMON_IN7_LCRIT: /* VINU7 */
		reg_mask = MAX42500_VMON_6VINU7_MASK;
		break;
	case MAX42500_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_VMON_IN5_OV_THRESH: /* VINO5 */
		reg_mask = MAX42500_VMON_1VINO5_MASK;
		break;
	case MAX42500_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_VMON_IN7_CRIT: /* VINO7 */
		reg_mask = MAX42500_VMON_6VINO7_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(st->regmap, max42500_reg_vmon[index],
				 reg_mask, value);
	if (ret)
		return ret;

	/* Update after successful register write */
	st->vmon[index] = FIELD_GET(GENMASK(7, 0), value);

	return 0;
}

static int max42500_fpsr_set_update(struct max42500_state *st, long value,
				    enum max42500_fpsr index)
{
	unsigned int reg_mask;
	int ret;

	guard(mutex)(&st->lock);

	switch (index) {
	case MAX42500_FPSR_FPS_CLOCK_DIVIDER: /* FDIV */
		reg_mask = MAX42500_FPSR_FDIV_MASK;
		break;
	case MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE: /* FPSEN1 */
		reg_mask = MAX42500_FPSR_FPSEN1_MASK;
		break;
	case MAX42500_FPSR_POWERDN_NO_INTERRUPT_ENABLE: /* DVALM */
		reg_mask = MAX42500_FPSR_DVALM_MASK;
		break;
	case MAX42500_FPSR_POWERUP_NO_INTERRUPT_ENABLE: /* UVALM */
		reg_mask = MAX42500_FPSR_UVALM_MASK;
		break;
	case MAX42500_FPSR_POWERDN_SEQ_CAPTURE_FINISH: /* DVAL */
		reg_mask = MAX42500_FPSR_DVAL_MASK;
		break;
	case MAX42500_FPSR_POWERUP_SEQ_CAPTURE_FINISH: /* UVAL */
		reg_mask = MAX42500_FPSR_UVAL_MASK;
		break;
	default:
		return -EINVAL;
	}

	ret = regmap_update_bits(st->regmap, max42500_reg_fpsr[index],
				 reg_mask, value);
	if (ret)
		return ret;

	/* Update after successful register write */
	st->fpsr[index] = FIELD_GET(GENMASK(7, 0), value);

	return 0;
}

static int max42500_vmon_read(struct max42500_state *st, long *value,
			      enum MAX42500_vmon index)
{
	int ret;

	ret = max42500_vmon_get_update(st, index);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_VMON_IN4_NOMINAL: /* VIN4 */
		*value = 500000 + (12500 * st->vmon[index]);
		return 0;
	case MAX42500_VMON_IN5_NOMINAL: /* VIN5 */
		*value = 500000 + (20000 * st->vmon[index]);
		return 0;
	case MAX42500_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_VMON_IN7_CRIT: /* VINO7 */
	case MAX42500_VMON_IN7_LCRIT: /* VINU7 */
		*value = 500000 + (5000 * st->vmon[index]);
		return 0;
	case MAX42500_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_VMON_IN5_OV_THRESH: /* VINO5 */
		*value = 102500 + (500 * st->vmon[index]);
		return 0;
	case MAX42500_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_VMON_IN5_UV_THRESH: /* VINU5 */
		*value = 97500 - (500 * st->vmon[index]);
		return 0;
	default:
		*value = st->vmon[index];
		return 0;
	}
}

static int max42500_fpsr_read(struct max42500_state *st, long *value,
			      enum max42500_fpsr index)
{
	u8 clk_div = MAX42500_FPSR_FPS_CLOCK_DIVIDER;
	int ret;

	ret = max42500_fpsr_get_update(st, index);
	if (ret)
		return ret;

	switch (index) {
	case MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MAX: /* UTIM1 */
	case MAX42500_FPSR_POWER2_AVERAGE_INTERVAL_MAX: /* UTIM2 */
	case MAX42500_FPSR_POWER3_AVERAGE_INTERVAL_MAX: /* UTIM3 */
	case MAX42500_FPSR_POWER4_AVERAGE_INTERVAL_MAX: /* UTIM4 */
	case MAX42500_FPSR_POWER5_AVERAGE_INTERVAL_MAX: /* UTIM5 */
	case MAX42500_FPSR_POWER6_AVERAGE_INTERVAL_MAX: /* UTIM6 */
	case MAX42500_FPSR_POWER7_AVERAGE_INTERVAL_MAX: /* UTIM7 */
	case MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MIN: /* DTIM1 */
	case MAX42500_FPSR_POWER2_AVERAGE_INTERVAL_MIN: /* DTIM2 */
	case MAX42500_FPSR_POWER3_AVERAGE_INTERVAL_MIN: /* DTIM3 */
	case MAX42500_FPSR_POWER4_AVERAGE_INTERVAL_MIN: /* DTIM4 */
	case MAX42500_FPSR_POWER5_AVERAGE_INTERVAL_MIN: /* DTIM5 */
	case MAX42500_FPSR_POWER6_AVERAGE_INTERVAL_MIN: /* DTIM6 */
	case MAX42500_FPSR_POWER7_AVERAGE_INTERVAL_MIN: /* DTIM7 */
		/* Get the latest FPS clock divider for the timestamp */
		ret = max42500_fpsr_get_update(st, clk_div);
		if (ret)
			return ret;

		*value = (st->fpsr[index] - 1) * 25 * (1 << st->fpsr[clk_div]);
		return 0;
	default:
		*value = st->fpsr[index];
		return 0;
	}
}

static int max42500_vmon_write(struct max42500_state *st, long value,
			       enum MAX42500_vmon index)
{
	long vmon;

	switch (index) {
	case MAX42500_VMON_IN1_NOMINAL: /* VIN1 */
	case MAX42500_VMON_IN2_NOMINAL: /* VIN2 */
	case MAX42500_VMON_IN3_NOMINAL: /* VIN3 */
	case MAX42500_VMON_IN4_NOMINAL: /* VIN4 */
		vmon = (value - 500000) / 12500;
		return max42500_vmon_set_update(st, vmon, index);
	case MAX42500_VMON_IN5_NOMINAL: /* VIN5 */
		vmon = (value - 500000) / 20000;
		return max42500_vmon_set_update(st, vmon, index);
	case MAX42500_VMON_IN6_CRIT: /* VINO6 */
	case MAX42500_VMON_IN6_LCRIT: /* VINU6 */
	case MAX42500_VMON_IN7_CRIT: /* VINO7 */
	case MAX42500_VMON_IN7_LCRIT: /* VINU7 */
		vmon = (value - 500000) / 5000;
		return max42500_vmon_set_update(st, vmon, index);
	case MAX42500_VMON_IN1_OV_THRESH: /* VINO1 */
	case MAX42500_VMON_IN2_OV_THRESH: /* VINO2 */
	case MAX42500_VMON_IN3_OV_THRESH: /* VINO3 */
	case MAX42500_VMON_IN4_OV_THRESH: /* VINO4 */
	case MAX42500_VMON_IN5_OV_THRESH: /* VINO5 */
		vmon = (value - 102500) / 500;
		return max42500_vmon_set_update(st, vmon, index);
	case MAX42500_VMON_IN1_UV_THRESH: /* VINU1 */
	case MAX42500_VMON_IN2_UV_THRESH: /* VINU2 */
	case MAX42500_VMON_IN3_UV_THRESH: /* VINU3 */
	case MAX42500_VMON_IN4_UV_THRESH: /* VINU4 */
	case MAX42500_VMON_IN5_UV_THRESH: /* VINU5 */
		vmon = (97500 - value) / 500;
		return max42500_vmon_set_update(st, vmon, index);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read_chip(struct max42500_state *st, const u32 attr,
			      long *val)
{
	int ret;

	switch (attr) {
	case hwmon_chip_pec:
		ret = max42500_chip_get_update(st, MAX42500_CHIP_PEC_ENABLE);
		if (ret)
			return ret;

		*val = st->chip[MAX42500_CHIP_PEC_ENABLE];
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read_in(struct max42500_state *st, const u32 attr,
			    long *val, const int channel)
{
	int index;

	switch (attr) {
	case hwmon_in_enable:
		index = MAX42500_VMON_IN1_ENABLE + channel;
		return max42500_vmon_read(st, val, index);
	case hwmon_in_lcrit:
		index = MAX42500_VMON_IN1_UV_THRESH + channel;
		return max42500_vmon_read(st, val, index);
	case hwmon_in_crit:
		index = MAX42500_VMON_IN1_OV_THRESH + channel;
		return max42500_vmon_read(st, val, index);
	case hwmon_in_min:
		index = MAX42500_VMON_IN1_NOMINAL + channel;
		return max42500_vmon_read(st, val, index);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read_power(struct max42500_state *st, const u32 attr,
			       long *val, const int channel)
{
	int index;

	switch (attr) {
	case hwmon_power_enable:
		return max42500_vmon_read(st, val, MAX42500_VMON_OFF_STATUS);
	case hwmon_power_lcrit_alarm:
		return max42500_vmon_read(st, val, MAX42500_VMON_UV_STATUS);
	case hwmon_power_crit_alarm:
		return max42500_vmon_read(st, val, MAX42500_VMON_OV_STATUS);
	case hwmon_power_average_interval_max:
		index = MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MAX + channel;
		return max42500_fpsr_read(st, val, index);
	case hwmon_power_average_interval_min:
		index = MAX42500_FPSR_POWER1_AVERAGE_INTERVAL_MIN + channel;
		return max42500_fpsr_read(st, val, index);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_chip:
		return max42500_read_chip(st, attr, val);
	case hwmon_in:
		return max42500_read_in(st, attr, val, channel);
	case hwmon_power:
		return max42500_read_power(st, attr, val, channel);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_write_chip(struct max42500_state *st, const u32 attr,
			       long val)
{
	switch (attr) {
	case hwmon_chip_pec:
		return max42500_chip_set_update(st, val,
						MAX42500_CHIP_PEC_ENABLE);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_write_in(struct max42500_state *st, const u32 attr,
			     long val, const int channel)
{
	int index;

	switch (attr) {
	case hwmon_in_enable:
		index = MAX42500_VMON_IN1_ENABLE + channel;
		return max42500_vmon_write(st, val, index);
	case hwmon_in_lcrit:
		index = MAX42500_VMON_IN1_UV_THRESH + channel;
		return max42500_vmon_write(st, val, index);
	case hwmon_in_crit:
		index = MAX42500_VMON_IN1_OV_THRESH + channel;
		return max42500_vmon_write(st, val, index);
	case hwmon_in_min:
		index = MAX42500_VMON_IN1_NOMINAL + channel;
		return max42500_vmon_write(st, val, index);
	case hwmon_in_reset_history:
		index = MAX42500_VMON_IN1_RESET_ENABLE + channel;
		return max42500_vmon_write(st, val, index);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_write(struct device *dev,
			  enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_chip:
		return max42500_write_chip(st, attr, val);
	case hwmon_in:
		return max42500_write_in(st, attr, val, channel);
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_read_labels(struct device *dev,
				enum hwmon_sensor_types type,
				u32 attr, int channel, const char **str)
{
	switch (type) {
	case hwmon_in:
		*str = "VMON";
		return 0;
	case hwmon_power:
		*str = "STATUS";
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int max42500_chip_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_chip_pec:
		return 0644;
	default:
		return 0;
	}
}

static int max42500_in_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_in_label:
		return 0444;
	case hwmon_in_reset_history:
		return 0200;
	case hwmon_in_enable:
	case hwmon_in_lcrit:
	case hwmon_in_crit:
	case hwmon_in_min:
		return 0644;
	default:
		return 0;
	}
}

static int max42500_power_is_visible(const u32 attr)
{
	switch (attr) {
	case hwmon_power_enable:
	case hwmon_power_lcrit_alarm:
	case hwmon_power_crit_alarm:
	case hwmon_power_average_interval_min:
	case hwmon_power_average_interval_max:
		return 0444;
	default:
		return 0;
	}
}

static umode_t max42500_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	switch (type) {
	case hwmon_chip:
		return max42500_chip_is_visible(attr);
	case hwmon_in:
		return max42500_in_is_visible(attr);
	case hwmon_power:
		return max42500_power_is_visible(attr);
	default:
		return 0;
	}
}

static const struct hwmon_channel_info * const max42500_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_PEC),
	HWMON_CHANNEL_INFO(in,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL | HWMON_I_MIN,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL | HWMON_I_MIN,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL | HWMON_I_MIN,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL | HWMON_I_MIN,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL | HWMON_I_MIN,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL,
			   HWMON_I_ENABLE | HWMON_I_LCRIT | HWMON_I_CRIT |
			   HWMON_I_RESET_HISTORY | HWMON_I_LABEL),
	HWMON_CHANNEL_INFO(power,
			   HWMON_P_ENABLE |
			   HWMON_P_LCRIT_ALARM |
			   HWMON_P_CRIT_ALARM |
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX,
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX,
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX,
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX,
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX,
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX,
			   HWMON_P_AVERAGE_INTERVAL_MIN |
			   HWMON_P_AVERAGE_INTERVAL_MAX),
	NULL
};

static const struct hwmon_ops max42500_hwmon_ops = {
	.read = max42500_read,
	.write = max42500_write,
	.is_visible = max42500_is_visible,
	.read_string = max42500_read_labels,
};

static const struct hwmon_chip_info max42500_chip_info = {
	.ops = &max42500_hwmon_ops,
	.info = max42500_info,
};

static int max42500_show_chip_label_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_chip_get_update(st, MAX42500_CHIP_LABEL);
	if (ret)
		return ret;

	*val = st->chip[MAX42500_CHIP_LABEL];

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_chip_label_log,
			 max42500_show_chip_label_log, NULL, "%llu\n");

static int max42500_show_chip_name_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_chip_get_update(st, MAX42500_CHIP_NAME);
	if (ret)
		return ret;

	*val = st->chip[MAX42500_CHIP_NAME];

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_chip_name_log,
			 max42500_show_chip_name_log, NULL, "%llu\n");

static int max42500_show_chip_otp_enable_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_chip_get_update(st, MAX42500_CHIP_RELOAD_OTP_ENABLE);
	if (ret)
		return ret;

	*val = st->chip[MAX42500_CHIP_RELOAD_OTP_ENABLE];

	return 0;
}

static int max42500_store_chip_otp_enable_log(void *arg, u64 val)
{
	return max42500_chip_set_update(arg, val,
					MAX42500_CHIP_RELOAD_OTP_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_chip_otp_enable_log,
			 max42500_show_chip_otp_enable_log,
			 max42500_store_chip_otp_enable_log, "%llu\n");

static int max42500_show_chip_bist_rst_enable_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_chip_get_update(st, MAX42500_CHIP_BIST_RESET_ENABLE);
	if (ret)
		return ret;

	*val = st->chip[MAX42500_CHIP_BIST_RESET_ENABLE];

	return 0;
}

static int max42500_store_chip_bist_rst_enable_log(void *arg, u64 val)
{
	return max42500_chip_set_update(arg, val,
					MAX42500_CHIP_BIST_RESET_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_chip_bist_rst_enable_log,
			 max42500_show_chip_bist_rst_enable_log,
			 max42500_store_chip_bist_rst_enable_log, "%llu\n");

static int max42500_show_chip_config_status_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_chip_get_update(st, MAX42500_CHIP_CONFIG_STATUS);
	if (ret)
		return ret;

	*val = st->chip[MAX42500_CHIP_CONFIG_STATUS];

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_chip_config_status_log,
			 max42500_show_chip_config_status_log, NULL, "%llu\n");

static int max42500_show_fpsr_status_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st, MAX42500_FPSR_FPS_STATUS);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_FPS_STATUS];

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_fpsr_status_log,
			 max42500_show_fpsr_status_log, NULL, "%llu\n");

static int max42500_show_pwrup_capt_finish_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st,
				      MAX42500_FPSR_POWERUP_SEQ_CAPTURE_FINISH);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_POWERUP_SEQ_CAPTURE_FINISH];

	return 0;
}

static int max42500_store_pwrup_capt_finish_log(void *arg, u64 val)
{
	return max42500_fpsr_set_update(arg, val,
				      MAX42500_FPSR_POWERUP_SEQ_CAPTURE_FINISH);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_pwrup_capt_finish_log,
			 max42500_show_pwrup_capt_finish_log,
			 max42500_store_pwrup_capt_finish_log, "%llu\n");

static int max42500_show_pwrup_capt_enable_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st,
				     MAX42500_FPSR_POWERUP_NO_INTERRUPT_ENABLE);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_POWERUP_NO_INTERRUPT_ENABLE];

	return 0;
}

static int max42500_store_pwrup_capt_enable_log(void *arg, u64 val)
{
	return max42500_fpsr_set_update(arg, val,
				     MAX42500_FPSR_POWERUP_NO_INTERRUPT_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_pwrup_capt_enable_log,
			 max42500_show_pwrup_capt_enable_log,
			 max42500_store_pwrup_capt_enable_log, "%llu\n");

static int max42500_show_pwrdn_capt_finish_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st,
				      MAX42500_FPSR_POWERDN_SEQ_CAPTURE_FINISH);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_POWERDN_SEQ_CAPTURE_FINISH];

	return 0;
}

static int max42500_store_pwrdn_capt_finish_log(void *arg, u64 val)
{
	return max42500_fpsr_set_update(arg, val,
				      MAX42500_FPSR_POWERDN_SEQ_CAPTURE_FINISH);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_pwrdn_capt_finish_log,
			 max42500_show_pwrdn_capt_finish_log,
			 max42500_store_pwrdn_capt_finish_log, "%llu\n");

static int max42500_show_fps_clk_div_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st, MAX42500_FPSR_FPS_CLOCK_DIVIDER);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_FPS_CLOCK_DIVIDER];

	return 0;
}

static int max42500_store_fps_clk_div_log(void *arg, u64 val)
{
	return max42500_fpsr_set_update(arg, val,
					MAX42500_FPSR_FPS_CLOCK_DIVIDER);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_fps_clk_div_log,
			 max42500_show_fps_clk_div_log,
			 max42500_store_fps_clk_div_log, "%llu\n");

static int max42500_show_pwrdn_capt_enable_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st,
				     MAX42500_FPSR_POWERDN_NO_INTERRUPT_ENABLE);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_POWERDN_NO_INTERRUPT_ENABLE];

	return 0;
}

static int max42500_store_pwrdn_capt_enable_log(void *arg, u64 val)
{
	return max42500_fpsr_set_update(arg, val,
				     MAX42500_FPSR_POWERDN_NO_INTERRUPT_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_pwrdn_capt_enable_log,
			 max42500_show_pwrdn_capt_enable_log,
			 max42500_store_pwrdn_capt_enable_log, "%llu\n");

static int max42500_show_fps_en1_timer_enable_log(void *arg, u64 *val)
{
	struct max42500_state *st = arg;
	int ret;

	ret = max42500_fpsr_get_update(st,
				      MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE);
	if (ret)
		return ret;

	*val = st->fpsr[MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE];

	return 0;
}

static int max42500_store_fps_en1_timer_enable_log(void *arg, u64 val)
{
	return max42500_fpsr_set_update(arg, val,
				      MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE);
}
DEFINE_DEBUGFS_ATTRIBUTE(max42500_fps_en1_timer_enable_log,
			 max42500_show_fps_en1_timer_enable_log,
			 max42500_store_fps_en1_timer_enable_log, "%llu\n");

static void max42500_hwmon_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static void max42500_hwmon_debugfs_init(struct max42500_state *st,
					struct platform_device *pdev,
					const struct device *hwmon)
{
	const char *debugfs_name;
	struct dentry *dentry;
	int ret;

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_name = devm_kasprintf(&pdev->dev, GFP_KERNEL, "max42500-%s",
				      dev_name(hwmon));
	if (!debugfs_name)
		return;

	dentry = debugfs_create_dir(debugfs_name, NULL);
	if (IS_ERR(dentry))
		return;

	ret = devm_add_action_or_reset(&pdev->dev,
				       max42500_hwmon_debugfs_remove,
				       dentry);
	if (ret)
		return;

	/* Chip registers */
	debugfs_create_file_unsafe("chip_label_log", 0400, dentry, st,
				   &max42500_chip_label_log);
	debugfs_create_file_unsafe("chip_name_log", 0400, dentry, st,
				   &max42500_chip_name_log);
	debugfs_create_file_unsafe("chip_otp_enable_log", 0644, dentry, st,
				   &max42500_chip_otp_enable_log);
	debugfs_create_file_unsafe("chip_bist_rst_enable_log", 0644, dentry, st,
				   &max42500_chip_bist_rst_enable_log);
	debugfs_create_file_unsafe("chip_config_status_log", 0400, dentry, st,
				   &max42500_chip_config_status_log);

	/* FPSR registers */
	debugfs_create_file_unsafe("fpsr_status_log", 0400, dentry, st,
				   &max42500_fpsr_status_log);
	debugfs_create_file_unsafe("pwrup_capt_finish_log", 0644, dentry, st,
				   &max42500_pwrup_capt_finish_log);
	debugfs_create_file_unsafe("pwrdn_capt_finish_log", 0644, dentry, st,
				   &max42500_pwrdn_capt_finish_log);
	debugfs_create_file_unsafe("pwrup_capt_enable_log", 0644, dentry, st,
				   &max42500_pwrup_capt_enable_log);
	debugfs_create_file_unsafe("pwrdn_capt_enable", 0644, dentry, st,
				   &max42500_pwrdn_capt_enable_log);
	debugfs_create_file_unsafe("fps_clk_div_log", 0644, dentry, st,
				   &max42500_fps_clk_div_log);
	debugfs_create_file_unsafe("fps_en1_timer_enable_log", 0644, dentry, st,
				   &max42500_fps_en1_timer_enable_log);
}

static int max42500_hwmon_probe(struct platform_device *pdev)
{
	struct device *parent_dev = pdev->dev.parent;
	struct regmap *regmap = dev_get_drvdata(parent_dev);
	struct i2c_client *client = to_i2c_client(parent_dev);
	struct device *dev = &pdev->dev;
	struct device *hwmon_dev;
	struct max42500_state *st;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	platform_set_drvdata(pdev, st);

	st->client = client;
	st->regmap = regmap;

	device_set_of_node_from_dev(dev, dev->parent);

	st->power_gpio = devm_gpiod_get_optional(dev, "poweroff-gpios",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->power_gpio))
		return PTR_ERR(st->power_gpio);

	st->sleep_gpio = devm_gpiod_get_optional(dev, "sleepoff-gpios",
						 GPIOD_OUT_HIGH);
	if (IS_ERR(st->sleep_gpio))
		return PTR_ERR(st->sleep_gpio);

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	hwmon_dev = devm_hwmon_device_register_with_info(dev, "max42500",
							st, &max42500_chip_info,
							NULL);

	max42500_hwmon_debugfs_init(st, pdev, hwmon_dev);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static int max42500_resume(struct device *dev)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	gpiod_set_value(st->power_gpio, GPIOD_OUT_HIGH);

	gpiod_set_value(st->sleep_gpio,
			!st->fpsr[MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE]);

	return 0;
}

static int max42500_suspend(struct device *dev)
{
	struct max42500_state *st = dev_get_drvdata(dev);

	gpiod_set_value(st->power_gpio, GPIOD_OUT_LOW);

	gpiod_set_value(st->sleep_gpio,
			!!st->fpsr[MAX42500_FPSR_FPS_EN1_START_TIMER_ENABLE]);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(max42500_pm_ops, max42500_suspend,
				max42500_resume);

static const struct platform_device_id max42500_hwmon_id_table[] = {
	{ "max42500-hwmon" },
	{  }
};
MODULE_DEVICE_TABLE(platform, max42500_hwmon_id_table);

static struct platform_driver max42500_hwmon_driver = {
	.driver = {
		.name	= "max42500-hwmon",
		.pm = pm_sleep_ptr(&max42500_pm_ops),
	},
	.probe = max42500_hwmon_probe,
	.id_table = max42500_hwmon_id_table,
};
module_platform_driver(max42500_hwmon_driver);

MODULE_AUTHOR("Kent Libetario <kent.libetario@analog.com>");
MODULE_DESCRIPTION("Hwmon driver for MAX42500");
MODULE_LICENSE("GPL");
