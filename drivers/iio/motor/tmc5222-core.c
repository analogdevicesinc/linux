// SPDX-License-Identifier: GPL-2.0
/*
 * TMC5222/TMC5221 Stepper Motor Controller Driver - Core
 *
 * Copyright (C) 2025 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/bitfield.h>
#include <linux/regmap.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/of.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/debugfs.h>
#include <linux/math64.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/events.h>

#include "tmc5222.h"

/* Private data structure */
struct tmc5222_priv {
	struct regmap *regmap;
	struct gpio_desc *drv_en_gpio;
	struct gpio_desc *gpio_sleep;
	struct mutex lock;

	uint32_t clock;
	bool enabled;

	/* Interrupts */
	int irq_stallguard;
	int irq_pos_reached;

	/* Event enable flags */
	bool stallguard_event_enabled;
	bool pos_reached_event_enabled;

	/* Current control */
	u8 current_run;
	u8 current_hold;

	/* Microstep and mode */
	u8 microstep_res;
	u32 step_angle_millidegrees; /* Step angle in millidegrees (e.g., 1800 for 1.8°) */

	/* Chopper configuration */
	u8 toff;
	u8 tbl;
	u8 hstrt_tfd210;
	u8 hend_offset;
	bool vhighchm;
	bool vhighfs;

	/* PWM configuration */
	bool pwm_dis_reg_stat;
	bool pwm_meas_sd_enable;
	enum tmc5222_pwm_freewheel pwm_freewheel;
	enum tmc5222_pwm_freq pwm_freq;

	/* StallGuard configuration */
	u8 sgt; /* StallGuard threshold, if > 0 then StallGuard is active */
	u16 sgt4; /* StallGuard4 threshold (9-bit, 0-511) */
	bool sg_stop;

	/* Delay settings */
	u8 irundelay;
	u8 iholddelay;

	/* Ramp mode (internal use only) */
	enum tmc5222_rampmode rampmode;

	/* Cached sysfs attributes */
	u8 fs_gain;
	u8 fs_sense;
	u8 global_scaler_a;
	u8 global_scaler_b;
	u32 tpowerdown;
	u32 tpwmthrs;
	u32 tcoolthrs;
	u32 thigh;
	u32 vstart;
	u32 a1;
	u32 v1;
	u32 a2;
	u32 v2;
	u32 amax;
	u32 vmax;
	u32 dmax;
	u32 d2;
	u32 d1;
	u32 vstop;
	s32 xtarget;
};

/* Register access helpers */
static inline int tmc5222_read_reg(struct tmc5222_priv *priv, unsigned int reg,
				   unsigned int *val)
{
	return regmap_read(priv->regmap, reg, val);
}

static inline int tmc5222_write_reg(struct tmc5222_priv *priv, unsigned int reg,
				    unsigned int val)
{
	return regmap_write(priv->regmap, reg, val);
}

static inline int tmc5222_update_bits(struct tmc5222_priv *priv, unsigned int reg,
				      unsigned int mask, unsigned int val)
{
	return regmap_update_bits(priv->regmap, reg, mask, val);
}

static int tmc5222_clear_stall(struct iio_dev *indio_dev)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);

	return tmc5222_update_bits(priv, TMC5222_RAMP_STAT, TMC5222_EVENT_STOP_SG_MASK, TMC5222_EVENT_STOP_SG_MASK);
}

static int tmc5222_clear_pos_reached(struct iio_dev *indio_dev)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);

	return tmc5222_update_bits(priv, TMC5222_RAMP_STAT, TMC5222_EVENT_POS_REACH_MASK, TMC5222_EVENT_POS_REACH_MASK);
}

static int tmc5222_set_position(struct iio_dev *indio_dev, s32 position)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	int ret;

	ret = tmc5222_clear_stall(indio_dev);
	if (ret)
		return ret;

	/* Warn about zero motion parameters (except VSTART, VSTOP, and DMAX) */
	if (priv->a1 == 0)
		dev_warn(&indio_dev->dev, "a1 is 0, motor may not move properly\n");
	if (priv->a2 == 0)
		dev_warn(&indio_dev->dev, "a2 is 0, motor may not move properly\n");
	if (priv->amax == 0)
		dev_warn(&indio_dev->dev, "amax is 0, motor may not accelerate properly\n");
	if (priv->v1 == 0)
		dev_warn(&indio_dev->dev, "v1 is 0, motor may not move properly\n");
	if (priv->v2 == 0)
		dev_warn(&indio_dev->dev, "v2 is 0, motor may not move properly\n");
	if (priv->vmax == 0)
		dev_warn(&indio_dev->dev, "vmax is 0, motor may not move properly\n");
	if (priv->d1 == 0)
		dev_warn(&indio_dev->dev, "d1 is 0, motor may not move properly\n");
	if (priv->d2 == 0)
		dev_warn(&indio_dev->dev, "d2 is 0, motor may not move properly\n");

	/* Update motion parameters */
	ret = tmc5222_write_reg(priv, TMC5222_VSTART, priv->vstart);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_A1, priv->a1);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_V1, priv->v1);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_A2, priv->a2);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_V2, priv->v2);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_AMAX, priv->amax);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_VMAX, priv->vmax);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_DMAX, priv->dmax);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_D2, priv->d2);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_D1, priv->d1);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_VSTOP, priv->vstop);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_RAMPMODE, TMC5222_RAMPMODE_POSITION);
	if (ret)
		return ret;

	ret = tmc5222_write_reg(priv, TMC5222_XTARGET, position);
	if (ret)
		return ret;

	return 0;
}

/* Set velocity (internal helper) */
static int tmc5222_set_velocity(struct iio_dev *indio_dev, s32 velocity)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = tmc5222_clear_stall(indio_dev);
	if (ret)
		return ret;

	ret = tmc5222_read_reg(priv, TMC5222_AMAX, &reg_val);
	if (ret)
		return ret;

	/* If AMAX register is 0, write the cached value */
	if (reg_val == 0) {
		ret = tmc5222_write_reg(priv, TMC5222_AMAX, priv->amax);
		if (ret)
			return ret;
	}

	if (velocity < 0) {
		ret = tmc5222_write_reg(priv, TMC5222_VMAX, (-1) * velocity);
		if (ret)
			return ret;

		ret = tmc5222_write_reg(priv, TMC5222_RAMPMODE,
					TMC5222_RAMPMODE_NEGATIVE_VELOCITY);			
		
	}
	else {
		ret = tmc5222_write_reg(priv, TMC5222_VMAX, velocity);
		if (ret)
			return ret;

		ret = tmc5222_write_reg(priv, TMC5222_RAMPMODE,
					TMC5222_RAMPMODE_POSITIVE_VELOCITY);
	}

	return ret;
}

/* Enable/disable motor via GPIO */
static int tmc5222_set_enable(struct iio_dev *indio_dev, bool enable)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	int ret = 0;

	if (enable) {
		/* SLEEP stays HIGH (chip always awake for I2C) */
		/* Enable driver outputs */
		gpiod_set_value_cansleep(priv->drv_en_gpio, 1);
		msleep(10);

		ret = tmc5222_set_position(indio_dev, priv->xtarget);
		if (ret)
			return ret;

		priv->enabled = true;
	} else {
		/* Disable driver outputs (SLEEP stays HIGH) */
		gpiod_set_value_cansleep(priv->drv_en_gpio, 0);

		priv->enabled = false;
	}

	return 0;
}

/* Debugfs attributes */
static int drv_status_show(struct seq_file *s, void *data)
{
	struct iio_dev *indio_dev = s->private;
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	unsigned int status;
	int ret;

	mutex_lock(&priv->lock);
	ret = tmc5222_read_reg(priv, TMC5222_DRV_STATUS, &status);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	seq_printf(s, "status: 0x%x\n", status);
	seq_printf(s, "standstill: %d\n", !!(status & TMC5222_DRV_STATUS_STST));
	seq_printf(s, "open_load_b: %d\n", !!(status & TMC5222_DRV_STATUS_OLB));
	seq_printf(s, "open_load_a: %d\n", !!(status & TMC5222_DRV_STATUS_OLA));
	seq_printf(s, "short_to_ground_b: %d\n", !!(status & TMC5222_DRV_STATUS_S2GB));
	seq_printf(s, "short_to_ground_a: %d\n", !!(status & TMC5222_DRV_STATUS_S2GA));
	seq_printf(s, "overtemperature_warning: %d\n", !!(status & TMC5222_DRV_STATUS_OTPW));
	seq_printf(s, "overtemperature: %d\n", !!(status & TMC5222_DRV_STATUS_OT));
	seq_printf(s, "stallguard: %d\n", !!(status & TMC5222_DRV_STATUS_STALLGUARD));
	seq_printf(s, "fullstep_active: %d\n", !!(status & TMC5222_DRV_STATUS_FSACTIVE));
	seq_printf(s, "stealthchop_active: %d\n", !!(status & TMC5222_DRV_STATUS_STEALTHCHOP));
	seq_printf(s, "short_to_supply_b: %d\n", !!(status & TMC5222_DRV_STATUS_S2VSB));
	seq_printf(s, "short_to_supply_a: %d\n", !!(status & TMC5222_DRV_STATUS_S2VSA));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(drv_status);

static int general_status_show(struct seq_file *s, void *data)
{
	struct iio_dev *indio_dev = s->private;
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	unsigned int status;
	int ret;

	mutex_lock(&priv->lock);
	ret = tmc5222_read_reg(priv, TMC5222_GSTAT, &status);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	seq_printf(s, "0x%08x\n", status);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(general_status);

/* sc2_vel_thrs attribute (StealthChop2 velocity threshold, 0 = disable>)
 * When > 0, SC2 will be enabled
 */
static ssize_t sc2_vel_thrs_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	u64 numerator, denominator;
	u32 sc2_vel_thrs;

	if (priv->tpwmthrs == 0)
		return sysfs_emit(buf, "StealthChop2 is disabled.\n");

	numerator = (u64)priv->clock * priv->step_angle_millidegrees;
	denominator = (u64)priv->tpwmthrs * 1000 * (256 >> priv->microstep_res);
	sc2_vel_thrs = (u32)div64_u64(numerator, denominator);

	return sysfs_emit(buf, "%u\n", sc2_vel_thrs);
}

static ssize_t sc2_vel_thrs_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	u64 numerator, denominator;
	u32 sc2_vel_thrs;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val == 0) {
		mutex_lock(&priv->lock);
		ret = tmc5222_update_bits(priv, TMC5222_GCONF, TMC5222_EN_PWM_MODE_MASK, 0);
		
		ret = tmc5222_write_reg(priv, TMC5222_TPWMTHRS, 0);

		priv->tpwmthrs = 0;
		mutex_unlock(&priv->lock);
		if (ret)
			return ret;
	}

	mutex_lock(&priv->lock);
	/*
	 * Convert velocity threshold (rad/s) to TPWMTHRS register value.
	 * TPWMTHRS = clock * step_angle_millidegrees / (velocity * 1000 * microsteps_per_fullstep)
	 * where microsteps_per_fullstep = 256 >> microstep_res
	 */
	numerator = (u64)priv->clock * priv->step_angle_millidegrees;
	denominator = (u64)val * 1000 * (256 >> priv->microstep_res);
	sc2_vel_thrs = (u32)div64_u64(numerator, denominator);

	ret = tmc5222_update_bits(priv, TMC5222_GCONF, TMC5222_EN_PWM_MODE_MASK, TMC5222_EN_PWM_MODE_MASK);
	if (ret) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = tmc5222_write_reg(priv, TMC5222_TPWMTHRS, sc2_vel_thrs);

	priv->tpwmthrs = sc2_vel_thrs;
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return count;
}

/* cool_vel_thrs attribute - coolstep velocity threshold */
static ssize_t cool_vel_thrs_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	u64 numerator, denominator;
	u32 cool_vel_thrs;

	if (priv->tcoolthrs == 0)
		return sysfs_emit(buf, "StallGuard is disabled.\n");

	numerator = (u64)priv->clock * priv->step_angle_millidegrees;
	denominator = (u64)priv->tcoolthrs * 1000 * (256 >> priv->microstep_res);
	cool_vel_thrs = (u32)div64_u64(numerator, denominator);

	return sysfs_emit(buf, "%u\n", cool_vel_thrs);
}

static ssize_t cool_vel_thrs_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	u64 numerator, denominator;
	u32 cool_vel_thrs;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val == 0) {
		mutex_lock(&priv->lock);
		
		priv->tcoolthrs = 0;
		ret = tmc5222_write_reg(priv, TMC5222_TCOOLTHRS, 0);
		mutex_unlock(&priv->lock);
		if (ret)
			return ret;
	}

	mutex_lock(&priv->lock);
	/*
	 * Convert velocity threshold (rad/s) to TCOOLTHRS register value.
	 * TCOOLTHRS = clock * step_angle_millidegrees / (velocity * 1000 * microsteps_per_fullstep)
	 * where microsteps_per_fullstep = 256 >> microstep_res
	 */
	numerator = (u64)priv->clock * priv->step_angle_millidegrees;
	denominator = (u64)val * 1000 * (256 >> priv->microstep_res);
	cool_vel_thrs = (u32)div64_u64(numerator, denominator);
	priv->tcoolthrs = cool_vel_thrs;
	ret = tmc5222_write_reg(priv, TMC5222_TCOOLTHRS, cool_vel_thrs);
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return count;
}

/* fullstep_vel_thrs attribute - fullstep velocity threshold (0 = disable) */
static ssize_t fullstep_vel_thrs_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	u64 numerator, denominator;
	u32 fullstep_vel_thrs;

	if (priv->thigh == 0)
		return sysfs_emit(buf, "FullStep is disabled.\n");

	numerator = (u64)priv->clock * priv->step_angle_millidegrees;
	denominator = (u64)priv->thigh * 1000 * (256 >> priv->microstep_res);
	fullstep_vel_thrs = (u32)div64_u64(numerator, denominator);

	return sysfs_emit(buf, "%u\n", fullstep_vel_thrs);
}

static ssize_t fullstep_vel_thrs_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	u64 numerator, denominator;
	u32 fullstep_vel_thrs;
	unsigned int val;
	int ret;

	ret = kstrtouint(buf, 0, &val);
	if (ret)
		return ret;

	if (val == 0) {
		mutex_lock(&priv->lock);
		ret = tmc5222_update_bits(priv, TMC5222_CHOPCONF, TMC5222_VHIGHFS_MASK, 0);
		
		ret = tmc5222_write_reg(priv, TMC5222_THIGH, 0);

		priv->thigh = 0;
		mutex_unlock(&priv->lock);
		if (ret)
			return ret;
	}

	mutex_lock(&priv->lock);
	/*
	 * Convert velocity threshold (rad/s) to THIGH register value.
	 * THIGH = clock * step_angle_millidegrees / (velocity * 1000 * microsteps_per_fullstep)
	 * where microsteps_per_fullstep = 256 >> microstep_res
	 */
	numerator = (u64)priv->clock * priv->step_angle_millidegrees;
	denominator = (u64)val * 1000 * (256 >> priv->microstep_res);
	fullstep_vel_thrs = (u32)div64_u64(numerator, denominator);

	ret = tmc5222_update_bits(priv, TMC5222_CHOPCONF, TMC5222_VHIGHFS_MASK, TMC5222_VHIGHFS_MASK);
	if (ret) {
		mutex_unlock(&priv->lock);
		return ret;
	}

	ret = tmc5222_write_reg(priv, TMC5222_THIGH, fullstep_vel_thrs);

	priv->thigh = fullstep_vel_thrs;
	mutex_unlock(&priv->lock);

	if (ret)
		return ret;

	return count;
}

/* Device attributes - sysfs only */
static DEVICE_ATTR_RW(sc2_vel_thrs);
static DEVICE_ATTR_RW(cool_vel_thrs);
static DEVICE_ATTR_RW(fullstep_vel_thrs);

static struct attribute *tmc5222_attrs[] = {
	&dev_attr_sc2_vel_thrs.attr,
	&dev_attr_cool_vel_thrs.attr,
	&dev_attr_fullstep_vel_thrs.attr,
	NULL,
};

static const struct attribute_group tmc5222_attr_group = {
	.attrs = tmc5222_attrs,
};

/* IIO event specifications */
static const struct iio_event_spec tmc5222_velocity_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

static const struct iio_event_spec tmc5222_position_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH_ADAPTIVE,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_CHANGE,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_ENABLE),
	},
};

/* IIO read_raw function */
static int tmc5222_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	unsigned int actual, reg;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		*val = priv->enabled;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ANGL_VEL) {
			reg = TMC5222_VACTUAL;
		} else if (chan->type == IIO_ACCEL) {
			reg = TMC5222_AACTUAL;
		} else if (chan->type == IIO_ANGL) {
			reg = TMC5222_XACTUAL;
		} else {
			return -EINVAL;
		}
		/* Read absolute position from XACTUAL register */
		mutex_lock(&priv->lock);
		ret = tmc5222_read_reg(priv, reg, &actual);
		mutex_unlock(&priv->lock);
		if (ret)
			return ret;
		*val = (s32)actual;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ANGL_VEL) {
			/*
			 * Velocity scale: convert VACTUAL (usteps/t) to degrees/s
			 *
			 * TMC5222 velocity formula (empirically verified):
			 *   velocity[usteps/s] = VACTUAL × fCLK / (2^24 × 10)
			 *
			 * Converting to degrees/s:
			 *   degrees/s = VACTUAL × fCLK × step_angle / (2^24 × 10 × 1000 × usteps_per_fullstep)
			 *
			 * Rearranging for IIO_VAL_FRACTIONAL_LOG2 (val / 2^val2):
			 *   scale = (fCLK / 10000) × step_angle_millidegrees / 2^(32 - microstep_res)
			 */
			*val = (priv->clock / 10000) * priv->step_angle_millidegrees;
			*val2 = 32 - priv->microstep_res;
			return IIO_VAL_FRACTIONAL_LOG2;
		} else if (chan->type == IIO_ACCEL) {
			/*
			 * Acceleration scale: convert AACTUAL (usteps/t^2) to degrees/s^2
			 *
			 * TMC5222 acceleration formula (based on velocity timing with 2^48):
			 *   acceleration[usteps/s^2] = AACTUAL × fCLK^2 / 2^48
			 *
			 * Converting to degrees/s^2:
			 *   deg/s^2 = AACTUAL × fCLK^2 × step_angle / (2^48 × usteps_per_fullstep)
			 *           = AACTUAL × fCLK^2 × step_angle_millidegrees / (1000 × 2^48 × usteps)
			 *
			 * Rearranging for IIO_VAL_FRACTIONAL_LOG2 (val / 2^val2):
			 *   scale = ((fCLK^2 / 1000) × step_angle_millidegrees >> 40) / 2^(16 - microstep_res)
			 *
			 * Note: Divide clock_sq by 1000 first to avoid u64 overflow.
			 */
			u64 clock_sq = (u64)priv->clock * priv->clock;
			u64 numerator = div_u64(clock_sq, 1000) * priv->step_angle_millidegrees;

			*val = (int)(numerator >> 40);
			*val2 = 16 - priv->microstep_res;
			return IIO_VAL_FRACTIONAL_LOG2;
		} else if (chan->type == IIO_ANGL) {
			/* Position scale: convert XACTUAL (microsteps) to degrees
			 * XACTUAL is in microsteps
			 * degrees = (XACTUAL × step_angle_millidegrees) / (1000 × microsteps_per_full_step)
			 * microsteps_per_full_step = 256 >> microstep_res
			 */
			*val = priv->step_angle_millidegrees;
			*val2 = 1000 << (8 - priv->microstep_res);
			return IIO_VAL_FRACTIONAL;
		} else {
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

/* IIO write_raw function */
static int tmc5222_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	int ret;

	mutex_lock(&priv->lock);

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		priv->enabled = val;
		ret = tmc5222_set_enable(indio_dev, priv->enabled);
		break;
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_ANGL_VEL) {
			/* VMAX is 23-bit signed: -2^22 to +(2^22 - 1) */
			if (val < -4194304 || val > 4194303) {
				ret = -EINVAL;
				break;
			}
			ret = tmc5222_set_velocity(indio_dev, val);
		} else if (chan->type == IIO_ACCEL) {
			ret = tmc5222_write_reg(priv, TMC5222_AMAX, val);
		} else if (chan->type == IIO_ANGL) {
			ret = tmc5222_set_position(indio_dev, val);
		} else {
			ret = -EINVAL;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&priv->lock);
	return ret;
}

/* IIO register access for debugging */
static int tmc5222_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);

	if (readval)
		return tmc5222_read_reg(priv, reg, readval);

	return tmc5222_write_reg(priv, reg, writeval);
}

/* IIO event configuration */
static int tmc5222_read_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);

	if (type == IIO_EV_TYPE_THRESH_ADAPTIVE) {
		if (chan->type == IIO_ANGL_VEL || chan->type == IIO_ANGL) {
			/* StallGuard event - available on both angular velocity and angle channels */
			return priv->stallguard_event_enabled;
		}
	} else if (type == IIO_EV_TYPE_CHANGE && chan->type == IIO_ANGL) {
		/* Position reached event - only on angle channel */
		return priv->pos_reached_event_enabled;
	}

	return -EINVAL;
}

static int tmc5222_write_event_config(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      int state)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	int ret;

	if (type == IIO_EV_TYPE_THRESH_ADAPTIVE) {
		if (chan->type == IIO_ANGL_VEL || chan->type == IIO_ANGL) {
			/* StallGuard event - available on both angular velocity and angle channels */
			if (priv->irq_stallguard < 0)
				return -ENODEV;

			mutex_lock(&priv->lock);

			if (state) {
				unsigned int tstep;

				/* Read current TSTEP to verify stallGuard will be active */
				ret = tmc5222_read_reg(priv, TMC5222_TSTEP, &tstep);
				if (ret) {
					mutex_unlock(&priv->lock);
					return ret;
				}

				/* Warn if motor is too slow for current TCOOLTHRS setting
				 * StallGuard is active when TSTEP <= TCOOLTHRS
				 * (lower TSTEP = higher velocity)
				 */
				if (tstep > priv->tcoolthrs && priv->tcoolthrs > 0) {
					dev_warn(&indio_dev->dev, "TSTEP(%u) > TCOOLTHRS(%u): motor too slow, stallGuard inactive. Increase velocity or increase TCOOLTHRS.\n",
						 tstep, priv->tcoolthrs);
				}

				/* Clear any pending stall flags before enabling to prevent
				 * false stall detection
				 */
				ret = tmc5222_update_bits(priv, TMC5222_RAMP_STAT,
							  TMC5222_EVENT_STOP_SG_MASK,
							  TMC5222_EVENT_STOP_SG_MASK);
				if (ret) {
					mutex_unlock(&priv->lock);
					return ret;
				}
			}

			/* Update sg_stop hardware bit to match event state */
			priv->sg_stop = !!state;
			ret = tmc5222_update_bits(priv, TMC5222_SW_MODE,
						  TMC5222_SG_STOP_MASK,
						  state ? TMC5222_SG_STOP_MASK : 0);
			if (ret) {
				mutex_unlock(&priv->lock);
				return ret;
			}

			priv->stallguard_event_enabled = !!state;

			mutex_unlock(&priv->lock);

			if (state)
				enable_irq(priv->irq_stallguard);
			else
				disable_irq(priv->irq_stallguard);
		} else {
			return -EINVAL;
		}
	} else if (type == IIO_EV_TYPE_CHANGE && chan->type == IIO_ANGL) {
		/* Position reached event - only on angle channel */
		if (priv->irq_pos_reached < 0)
			return -ENODEV;

		priv->pos_reached_event_enabled = !!state;
		if (state)
			enable_irq(priv->irq_pos_reached);
		else
			disable_irq(priv->irq_pos_reached);
	} else {
		return -EINVAL;
	}

	return 0;
}

static const struct iio_info tmc5222_info = {
	.read_raw = &tmc5222_read_raw,
	.write_raw = &tmc5222_write_raw,
	.debugfs_reg_access = &tmc5222_reg_access,
	.read_event_config = &tmc5222_read_event_config,
	.write_event_config = &tmc5222_write_event_config,
	.attrs = &tmc5222_attr_group,
};

/* IIO channels definition */
static const struct iio_chan_spec tmc5222_channels[] = {
	{
		.type = IIO_ANGL_VEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 1,
		.channel = 0,
		.event_spec = tmc5222_velocity_events,
		.num_event_specs = ARRAY_SIZE(tmc5222_velocity_events),
	}, {
		.type = IIO_ACCEL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 1,
		.channel = 1,
	}, {
		.type = IIO_ANGL,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_ENABLE),
		.output = 1,
		.channel = 2,
		.event_spec = tmc5222_position_events,
		.num_event_specs = ARRAY_SIZE(tmc5222_position_events),
	}
};

/* Interrupt handler for stall guard event */
static irqreturn_t tmc5222_irq_stallguard(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	enum iio_chan_type type;
	unsigned int ramp_stat;
	unsigned int val;
	int ret;

	/* Read RAMP_STAT to check if stall guard event occurred */
	ret = tmc5222_read_reg(priv, TMC5222_RAMP_STAT, &ramp_stat);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to read RAMP_STAT in stallguard IRQ: %d\n", ret);
		return IRQ_NONE;
	}

	/* Read ramp mode to determine which channel to report on */
	ret = tmc5222_read_reg(priv, TMC5222_RAMPMODE, &val);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to read RAMPMODE in stallguard IRQ: %d\n", ret);
		return IRQ_NONE;
	}

	/* Report on appropriate channel based on mode:
	 * Position mode (0) -> report on IIO_ANGL channel
	 * Velocity mode (1/2) -> report on IIO_ANGL_VEL channel */
	type = (val == TMC5222_RAMPMODE_POSITION) ? IIO_ANGL : IIO_ANGL_VEL;

	if (ramp_stat & TMC5222_EVENT_STOP_SG_MASK) {
		s64 timestamp = iio_get_time_ns(indio_dev);

		dev_info(&indio_dev->dev, "Stall guard event detected! RAMP_STAT: 0x%08x\n", ramp_stat);

		/* Push IIO event to userspace */
		if (priv->stallguard_event_enabled) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(type, 0,
							    IIO_EV_TYPE_THRESH_ADAPTIVE,
							    IIO_EV_DIR_EITHER),
				       timestamp);
		}

		/* Clear the event by writing 1 to the bit */
		tmc5222_clear_stall(indio_dev);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

/* Interrupt handler for position reached event */
static irqreturn_t tmc5222_irq_pos_reached(int irq, void *dev_id)
{
	struct iio_dev *indio_dev = dev_id;
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	unsigned int ramp_stat;
	int ret;

	/* Read RAMP_STAT to check if position reached event occurred */
	ret = tmc5222_read_reg(priv, TMC5222_RAMP_STAT, &ramp_stat);
	if (ret) {
		dev_err(&indio_dev->dev, "Failed to read RAMP_STAT in pos_reached IRQ: %d\n", ret);
		return IRQ_NONE;
	}

	if (ramp_stat & TMC5222_EVENT_POS_REACH_MASK) {
		s64 timestamp = iio_get_time_ns(indio_dev);

		dev_info(&indio_dev->dev, "Position reached event detected! RAMP_STAT: 0x%08x\n", ramp_stat);

		/* Push IIO event to userspace on IIO_ANGL channel */
		if (priv->pos_reached_event_enabled) {
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_ANGL, 2,
							    IIO_EV_TYPE_CHANGE,
							    IIO_EV_DIR_EITHER),
				       timestamp);
		}

		/* Clear the event by writing 1 to the bit */
		tmc5222_clear_pos_reached(indio_dev);

		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static int tmc5222_setup(struct iio_dev *indio_dev)
{
	struct tmc5222_priv *priv = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	/* FS Sense & FS Gain - use values from DT */
	val = FIELD_PREP(TMC5222_FS_SENSE_MASK, priv->fs_sense) |
	      FIELD_PREP(TMC5222_FS_GAIN_MASK, priv->fs_gain);
	ret = tmc5222_write_reg(priv, TMC5222_DRVCONF, val);
	if (ret)
		return ret;

	/* Global Scaler - use values from DT */
	val = FIELD_PREP(TMC5222_GLOBALSCALER_A_MASK, priv->global_scaler_a) |
	      FIELD_PREP(TMC5222_GLOBALSCALER_B_MASK, priv->global_scaler_b);
	ret = tmc5222_write_reg(priv, TMC5222_GLOBALSCALER, val);
	if (ret)
		return ret;

	/* Current control - use values from DT */
	val = (priv->irundelay << 24) | (priv->iholddelay << 16) | (priv->current_run << 8) | (priv->current_hold);
	ret = tmc5222_write_reg(priv, TMC5222_IHOLD_IRUN, val);
	if (ret)
		return ret;

	/* Tpowerdown - use value from DT */
	ret = tmc5222_write_reg(priv, TMC5222_TPOWERDOWN, priv->tpowerdown);
	if (ret)
		return ret;

	/* Chopper configuration - use values from DT */
	val = FIELD_PREP(TMC5222_TOFF_MASK, priv->toff);
	ret = tmc5222_update_bits(priv, TMC5222_CHOPCONF, TMC5222_TOFF_MASK, val);
	if (ret)
		return ret;

	val = FIELD_PREP(TMC5222_TBL_MASK, priv->tbl);
	ret = tmc5222_update_bits(priv, TMC5222_CHOPCONF, TMC5222_TBL_MASK, val);
	if (ret)
		return ret;

	val = FIELD_PREP(TMC5222_CHOPCONF_MRES_MASK, priv->microstep_res);
	ret = tmc5222_update_bits(priv, TMC5222_CHOPCONF, TMC5222_CHOPCONF_MRES_MASK, val);
	if (ret)
		return ret;

	/* PWM configuration - use values from DT TODO */
	val = FIELD_PREP(TMC5222_PWMCONF_FREEWHEEL_MASK, priv->pwm_freewheel);
	ret = tmc5222_update_bits(priv, TMC5222_PWMCONF, TMC5222_PWMCONF_FREEWHEEL_MASK, val);
	if (ret)
		return ret;
	
	val = FIELD_PREP(TMC5222_PWMCONF_FREQ_MASK, priv->pwm_freq);
	ret = tmc5222_update_bits(priv, TMC5222_PWMCONF, TMC5222_PWMCONF_FREQ_MASK, val);
	if (ret)
		return ret;

	/* StallGuard2 threshold - use value from DT */
	ret = tmc5222_update_bits(priv, TMC5222_COOLCONF,
				  TMC5222_SGT_MASK,
				  FIELD_PREP(TMC5222_SGT_MASK, priv->sgt));
	if (ret)
		return ret;

	/* StallGuard4 threshold - use value from DT */
	ret = tmc5222_update_bits(priv, TMC5222_SG4_THRS,
				  TMC5222_SG4_THRS_MASK,
				  FIELD_PREP(TMC5222_SG4_THRS_MASK, priv->sgt4));
	if (ret)
		return ret;

	/* Enable DIAG0 and DIAG1 as active low push-pull. */
	val = TMC5222_DIAG1_INVPP_MASK | TMC5222_DIAG1_NOD_PP_MASK;
	val |= TMC5222_DIAG0_INVPP_MASK | TMC5222_DIAG0_NOD_PP_MASK;
	ret = tmc5222_write_reg(priv, TMC5222_DIAG_CONF, val);
	if (ret)
		return ret;

	/* Configure DIAG0 as STALL_EVENT and DIAG1 as POS_REACHED event.*/
	ret = tmc5222_update_bits(priv, TMC5222_DIAG_CONF, TMC5222_DIAG1_POS_REACH_MASK, TMC5222_DIAG1_POS_REACH_MASK);
	if (ret)
		return ret;

	ret = tmc5222_update_bits(priv, TMC5222_DIAG_CONF, TMC5222_DIAG0_STALL_MASK, TMC5222_DIAG0_STALL_MASK);
	if (ret)
		return ret;

	return tmc5222_set_enable(indio_dev, priv->enabled);
}

int tmc5222_probe_common(struct device *dev, struct regmap *regmap)
{
	struct iio_dev *indio_dev;
	struct tmc5222_priv *priv;
	unsigned int val;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);

	priv->regmap = regmap;
	mutex_init(&priv->lock);

	/* Get sleep GPIO (mandatory) - Initialize HIGH to keep chip awake */
	priv->gpio_sleep = devm_gpiod_get(dev, "sleep", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpio_sleep)) {
		ret = PTR_ERR(priv->gpio_sleep);
		dev_err(dev, "Failed to get sleep GPIO: %d\n", ret);
		return ret;
	}

	/* Reset sequence using sleep GPIO - toggle LOW then back HIGH */
	gpiod_set_value_cansleep(priv->gpio_sleep, 0);
	msleep(1);
	gpiod_set_value_cansleep(priv->gpio_sleep, 1);
	msleep(10); /* Wait for chip to wake up fully */

	ret = tmc5222_update_bits(priv, TMC5222_CHOPCONF, TMC5222_DRV_EN_SW_MASK, TMC5222_DRV_EN_SW_MASK);
	if (ret)
		return ret;

	/* Get driver enable GPIO (mandatory) */
	priv->drv_en_gpio = devm_gpiod_get(dev, "drv-en", GPIOD_OUT_LOW);
	if (IS_ERR(priv->drv_en_gpio)) {
		ret = PTR_ERR(priv->drv_en_gpio);
		dev_err(dev, "Failed to get drv-en GPIO: %d\n", ret);
		return ret;
	}

	/* Initialize default values */
	priv->enabled = true;

	/* StallGuard configuration */
	priv->sg_stop = false; /* True -> StallGuard enabled.*/

	/* Ramp mode (internal use only) */
	priv->rampmode = TMC5222_RAMPMODE_POSITION;

	/* Cached sysfs attributes */
	priv->tpwmthrs = 0;
	priv->tcoolthrs = 0;
	priv->thigh = 0;

	/* Parse device tree properties (optional, use defaults if not specified) */
	ret = device_property_read_u32(dev, "ext-clock", &priv->clock);
	if (ret)
		priv->clock = 0;

	/* Current control */
	ret = device_property_read_u32(dev, "adi,current-run", &val);
	if (ret)
		priv->current_run = 31;
	else
		priv->current_run = val;

	ret = device_property_read_u32(dev, "adi,current-hold", &val);
	if (ret)
		priv->current_hold = 8;
	else
		priv->current_hold = val;

	/* Delay settings */
	ret = device_property_read_u32(dev, "adi,irundelay", &val);
	if (ret)
		priv->irundelay = 4;
	else
		priv->irundelay = val;

	ret = device_property_read_u32(dev, "adi,iholddelay", &val);
	if (ret)
		priv->iholddelay = 1;
	else
		priv->iholddelay = val;

	/* Driver configuration */
	ret = device_property_read_u32(dev, "adi,fs-gain", &val);
	if (ret)
		priv->fs_gain = 1;
	else
		priv->fs_gain = val;

	ret = device_property_read_u32(dev, "adi,fs-sense", &val);
	if (ret)
		priv->fs_sense = 1;
	else
		priv->fs_sense = val;

	ret = device_property_read_u32(dev, "adi,global-scaler", &val);
	if (ret) {
		priv->global_scaler_a = 0;
		priv->global_scaler_b = 0;
	} else {
		priv->global_scaler_a = val;
		priv->global_scaler_b = val;
	}

	ret = device_property_read_u32(dev, "adi,tpowerdown", &priv->tpowerdown);
	if (ret)
		priv->tpowerdown = 10;

	/* Chopper configuration */
	ret = device_property_read_u32(dev, "adi,toff", &val);
	if (ret)
		priv->toff = 2;
	else
		priv->toff = val;

	ret = device_property_read_u32(dev, "adi,tbl", &val);
	if (ret)
		priv->tbl = 2;
	else
		priv->tbl = val;

	/* PWM configuration */
	ret = device_property_read_u32(dev, "adi,pwm-freewheel", &val);
	if (ret)
		priv->pwm_freewheel = TMC5222_FREEWHEEL_NORMAL;
	else
		priv->pwm_freewheel = val;

	ret = device_property_read_u32(dev, "adi,pwm-freq", &val);
	if (ret)
		priv->pwm_freq = TMC5222_PWM_FREQ_2_1024;
	else
		priv->pwm_freq = val;

	/* StallGuard configuration */
	ret = device_property_read_u32(dev, "adi,sgt", &val);
	if (ret)
		priv->sgt = 0; /* Default: 0 (StallGuard2 threshold) */
	else
		priv->sgt = val;

	ret = device_property_read_u32(dev, "adi,sgt4", &val);
	if (ret)
		priv->sgt4 = 0; /* Default: 0 (StallGuard4 threshold) */
	else
		priv->sgt4 = val;

	/* Microstep resolution */
	ret = device_property_read_u32(dev, "adi,microstep-res", &val);
	if (ret)
		priv->microstep_res = TMC5222_MRES_256; /* Default: 256 microsteps */
	else
		priv->microstep_res = val;

	/* Step angle in millidegrees (e.g., 1800 for 1.8°, 900 for 0.9°) */
	ret = device_property_read_u32(dev, "adi,step-angle-millidegrees", &val);
	if (ret)
		priv->step_angle_millidegrees = 1800; /* Default: 1.8° */
	else
		priv->step_angle_millidegrees = val;

	/* Target position - will be used by set_position in setup */
	ret = device_property_read_u32(dev, "adi,xtarget", &val);
	if (ret)
		priv->xtarget = 0; /* Default to 0 position */
	else
		priv->xtarget = val;

	/* Ramp parameters - fixed values used for position control */
	ret = device_property_read_u32(dev, "adi,vstart", &val);
	if (ret)
		priv->vstart = 0;
	else
		priv->vstart = val;

	ret = device_property_read_u32(dev, "adi,a1", &val);
	if (ret)
		priv->a1 = 10000;
	else
		priv->a1 = val;

	ret = device_property_read_u32(dev, "adi,v1", &val);
	if (ret)
		priv->v1 = 100000;
	else
		priv->v1 = val;

	ret = device_property_read_u32(dev, "adi,a2", &val);
	if (ret)
		priv->a2 = 30000;
	else
		priv->a2 = val;

	ret = device_property_read_u32(dev, "adi,v2", &val);
	if (ret)
		priv->v2 = 300000;
	else
		priv->v2 = val;

	ret = device_property_read_u32(dev, "adi,amax", &val);
	if (ret)
		priv->amax = 100000;
	else
		priv->amax = val;

	ret = device_property_read_u32(dev, "adi,vmax", &val);
	if (ret)
		priv->vmax = 500000;
	else
		priv->vmax = val;

	ret = device_property_read_u32(dev, "adi,dmax", &val);
	if (ret)
		priv->dmax = 2000;
	else
		priv->dmax = val;

	ret = device_property_read_u32(dev, "adi,d2", &val);
	if (ret)
		priv->d2 = 1600;
	else
		priv->d2 = val;

	ret = device_property_read_u32(dev, "adi,d1", &val);
	if (ret)
		priv->d1 = 1000;
	else
		priv->d1 = val;

	ret = device_property_read_u32(dev, "adi,vstop", &val);
	if (ret)
		priv->vstop = 10;
	else
		priv->vstop = val;

	/* Read IOIN register to determine clock source */
	ret = tmc5222_read_reg(priv, TMC5222_IOIN, &val);
	if (ret) {
		dev_err(dev, "Failed to read IOIN register: %d\n", ret);
		return ret;
	}

	val = FIELD_GET(TMC5222_IOIN_EXT_CLK_MASK, val);
	switch (val) {
	case 0:
		/* Internal clock */
		priv->clock = TMC5222_INTERNAL_CLOCK_HZ;
		dev_info(dev, "Using internal clock: %u Hz\n", priv->clock);
		break;
	case 1:
		/* External clock */
		if (priv->clock == 0) {
			dev_err(dev, "External clock selected but ext-clock not specified in device tree\n");
			return -EINVAL;
		}
		dev_info(dev, "Using external clock: %u Hz\n", priv->clock);
		break;
	case 3:
		/* Clock quiescent mode */
		priv->clock = TMC5222_QUIESCENT_CLOCK_HZ;
		dev_info(dev, "Using quiescent clock: %u Hz\n", priv->clock);
		break;
	default:
		dev_warn(dev, "Unknown EXT_CLK value: %u, using internal clock\n", val);
		priv->clock = TMC5222_INTERNAL_CLOCK_HZ;
		break;
	}

	ret = tmc5222_setup(indio_dev);
	if (ret) {
		dev_err(dev, "Hardware initialization failed: %d\n", ret);
		return ret;
	}

	indio_dev->name = "tmc5222";
	indio_dev->channels = tmc5222_channels;
	indio_dev->num_channels = ARRAY_SIZE(tmc5222_channels);
	indio_dev->info = &tmc5222_info;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret) {
		dev_err(dev, "Failed to register IIO device: %d\n", ret);
		return ret;
	}

	/* Create debugfs entries */
	debugfs_create_file("drv_status", 0444, iio_get_debugfs_dentry(indio_dev),
			    indio_dev, &drv_status_fops);
	debugfs_create_file("general_status", 0444, iio_get_debugfs_dentry(indio_dev),
			    indio_dev, &general_status_fops);

	/* Initialize event flags */
	priv->stallguard_event_enabled = false;
	priv->pos_reached_event_enabled = false;

	/* Request stallguard interrupt (optional) */
	priv->irq_stallguard = fwnode_irq_get_byname(dev_fwnode(dev), "stallguard");
	if (priv->irq_stallguard > 0) {
		ret = devm_request_threaded_irq(dev, priv->irq_stallguard, NULL,
						tmc5222_irq_stallguard,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
						"tmc5222-stallguard", indio_dev);
		if (ret) {
			dev_err(dev, "Failed to request stallguard IRQ: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Stallguard interrupt configured on IRQ %d (disabled by default)\n", priv->irq_stallguard);
	} else if (priv->irq_stallguard == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else {
		dev_info(dev, "No stallguard interrupt specified\n");
		priv->irq_stallguard = -1;
	}

	/* Request position reached interrupt (optional) */
	priv->irq_pos_reached = fwnode_irq_get_byname(dev_fwnode(dev), "pos_reached");
	if (priv->irq_pos_reached > 0) {
		ret = devm_request_threaded_irq(dev, priv->irq_pos_reached, NULL,
						tmc5222_irq_pos_reached,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
						"tmc5222-pos-reached", indio_dev);
		if (ret) {
			dev_err(dev, "Failed to request position reached IRQ: %d\n", ret);
			return ret;
		}
		dev_info(dev, "Position reached interrupt configured on IRQ %d (disabled by default)\n", priv->irq_pos_reached);
	} else if (priv->irq_pos_reached == -EPROBE_DEFER) {
		return -EPROBE_DEFER;
	} else {
		dev_info(dev, "No position reached interrupt specified\n");
		priv->irq_pos_reached = -1;
	}

	dev_info(dev, "TMC5222 stepper motor controller initialized\n");

	return 0;
}
EXPORT_SYMBOL_GPL(tmc5222_probe_common);

void tmc5222_remove_common(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct tmc5222_priv *priv = iio_priv(indio_dev);

	/* Disable driver */
	gpiod_set_value_cansleep(priv->drv_en_gpio, 0);

	/* Enter sleep mode (LOW = sleep) */
	gpiod_set_value_cansleep(priv->gpio_sleep, 0);

	dev_info(dev, "TMC5222 driver removed\n");
}
EXPORT_SYMBOL_GPL(tmc5222_remove_common);

MODULE_AUTHOR("Radu Sabau <radu.sabau@analog.com>");
MODULE_DESCRIPTION("TMC5222/TMC5221 Stepper Motor Controller Core Driver");
MODULE_LICENSE("GPL");
