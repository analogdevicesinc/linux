// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices MAX30210 I2C Temperature Sensor driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <asm/div64.h>
#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/interrupt.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/stat.h>
#include <linux/string.h>
#include <linux/units.h>

#define MAX30210_STATUS_REG             0x00
#define MAX30210_INT_EN_REG             0x02
#define MAX30210_FIFO_DATA_REG          0x08
#define MAX30210_FIFO_CONF_1_REG        0x09
#define MAX30210_FIFO_CONF_2_REG        0x0A
#define MAX30210_SYS_CONF_REG           0x11
#define MAX30210_PIN_CONF_REG           0x12
#define MAX30210_TEMP_ALM_HI_REG        0x22
#define MAX30210_TEMP_ALM_LO_REG        0x24
#define MAX30210_TEMP_INC_THRESH_REG    0x26
#define MAX30210_TEMP_DEC_THRESH_REG    0x27
#define MAX30210_TEMP_CONF_1_REG        0x28
#define MAX30210_TEMP_CONF_2_REG        0x29
#define MAX30210_TEMP_CONV_REG          0x2A
#define MAX30210_TEMP_DATA_REG          0x2B
#define MAX30210_TEMP_SLOPE_REG         0x2D
#define MAX30210_UNIQUE_ID_REG          0x30
#define MAX30210_PART_ID_REG            0xFF

#define MAX30210_A_FULL_MASK   BIT(7)
#define MAX30210_TEMP_RDY_MASK BIT(6)
#define MAX30210_TEMP_DEC_MASK BIT(5)
#define MAX30210_TEMP_INC_MASK BIT(4)
#define MAX30210_TEMP_LO_MASK  BIT(3)
#define MAX30210_TEMP_HI_MASK  BIT(2)
#define MAX30210_PWR_RDY_MASK  BIT(0)

#define MAX30210_FLUSH_FIFO_MASK BIT(4)

#define MAX30210_EXT_CNV_EN_MASK   BIT(7)
#define MAX30210_EXT_CVT_ICFG_MASK BIT(6)
#define MAX30210_INT_FCFG_MASK     GENMASK(3, 2)
#define MAX30210_INT_OCFG_MASK     GENMASK(1, 0)

#define MAX30210_CHG_DET_EN_MASK      BIT(3)
#define MAX30210_RATE_CHG_FILTER_MASK GENMASK(2, 0)

#define MAX30210_TEMP_PERIOD_MASK GENMASK(3, 0)
#define MAX30210_ALERT_MODE_MASK  BIT(7)

#define MAX30210_AUTO_MASK   BIT(1)
#define MAX30210_CONV_T_MASK BIT(0)

#define MAX30210_PART_ID           0x45
#define MAX30210_FIFO_SIZE         64
#define MAX30210_FIFO_INVAL_DATA   GENMASK(23, 0)
#define MAX30210_WATERMARK_DEFAULT (0x40 - 0x1F)

#define MAX30210_INT_EN(state, mask)     ((state) ? (mask) : 0x0)

#define MAX30210_UNIQUE_ID_LEN 6
#define MAX30210_EXT_CVT_FREQ_MIN 1
#define MAX30210_EXT_CVT_FREQ_MAX 20

enum max30210_events_type {
	MAX30210_DEFAULT_EVENTS,
	MAX30210_ROC_EVENTS,
	MAX30210_COMP_MODE_EVENTS,
	MAX30210_COMP_MODE_ROC_EVENTS,
};

struct max30210_state {
	/*
	 * Prevent simultaneous access to the i2c client.
	 */
	struct mutex lock;
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct gpio_desc *powerdown_gpio;
	struct pwm_device *ext_cvt_pwm;
	bool chg_det_en;
	bool ext_cvt_en;
	bool ext_cvt_pol;
	bool hwfifo_enabled;
	u8 events_type;
	u8 watermark;
};

static const int samp_freq_avail[] = {
	0, 15625,
	0, 31250,
	0, 62500,
	0, 125000,
	0, 250000,
	0, 500000,
	1, 0,
	2, 0,
	4, 0,
	8, 0
};

static const int samp_freq_range[] = {MAX30210_EXT_CVT_FREQ_MIN, 1,
				       MAX30210_EXT_CVT_FREQ_MAX};

static const struct regmap_config max30210_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = MAX30210_PART_ID_REG,
};

static int max30210_read_temp_raw(struct regmap *regmap, unsigned int reg,
				   int *temp)
{
	u8 uval[2] __aligned(IIO_DMA_MINALIGN);
	int ret;

	ret = regmap_bulk_read(regmap, reg, uval, 2);
	if (ret)
		return ret;

	*temp = sign_extend32(get_unaligned_be16(uval), 15);

	return IIO_VAL_INT;
}

static int max30210_read_temp_scaled(struct regmap *regmap, unsigned int reg,
				      int *val, int *val2)
{
	int ret;

	ret = max30210_read_temp_raw(regmap, reg, val);
	if (ret < 0)
		return ret;

	*val *= 5;

	return IIO_VAL_INT;
}

static int max30210_write_temp_raw(struct regmap *regmap, unsigned int reg,
				    int temp)
{
	u8 uval[2] __aligned(IIO_DMA_MINALIGN);

	put_unaligned_be16(temp, uval);

	return regmap_bulk_write(regmap, reg, uval, 2);
}

static int max30210_write_temp_scaled(struct regmap *regmap, unsigned int reg,
				       int val, int val2)
{
	return max30210_write_temp_raw(regmap, reg, val / 5);
}

static void max30210_fifo_read(struct iio_dev *indio_dev)
{
	struct max30210_state *st = iio_priv(indio_dev);
	u8 data[3 * MAX30210_FIFO_SIZE]  __aligned(IIO_DMA_MINALIGN);

	u32 samp;
	int ret, i, j;

	ret = regmap_bulk_read(st->regmap, MAX30210_FIFO_DATA_REG,
			       data, 3 * st->watermark);
	if (ret < 0)
		return dev_err(&indio_dev->dev, "Failed to read from fifo.\n");

	for (i = 0; i < st->watermark; i++) {
		samp = 0;
		for (j = 0; j < 3; j++) {
			samp <<= 8;
			samp |= data[3 * i + j];
		}
		if (samp == MAX30210_FIFO_INVAL_DATA) {
			dev_err(&indio_dev->dev, "Invalid data\n");
			continue;
		}
		iio_push_to_buffers(indio_dev, &samp);
	}
}

static irqreturn_t max30210_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int status;
	int ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->regmap, MAX30210_STATUS_REG, &status);
	if (ret) {
		dev_err(&indio_dev->dev, "Status byte read error\n");
		goto exit_irq;
	}

	if (status & MAX30210_PWR_RDY_MASK) {
		dev_info(&indio_dev->dev, "power-on\n");
		st->watermark = MAX30210_WATERMARK_DEFAULT;
	}

	if (status & MAX30210_A_FULL_MASK)
		max30210_fifo_read(indio_dev);

	if (status & MAX30210_TEMP_HI_MASK)
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_RISING),
			       iio_get_time_ns(indio_dev));

	if (status & MAX30210_TEMP_LO_MASK)
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_FALLING),
			       iio_get_time_ns(indio_dev));

	if (status & MAX30210_TEMP_INC_MASK)
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
						    IIO_EV_TYPE_ROC,
						    IIO_EV_DIR_RISING),
			       iio_get_time_ns(indio_dev));

	if (status & MAX30210_TEMP_DEC_MASK)
		iio_push_event(indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_TEMP, 0,
						    IIO_EV_TYPE_ROC,
						    IIO_EV_DIR_FALLING),
			       iio_get_time_ns(indio_dev));

exit_irq:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int max30210_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				unsigned int writeval, unsigned int *readval)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (!readval)
		return regmap_write(st->regmap, reg, writeval);

	return regmap_read(st->regmap, reg, readval);
}

static int max30210_validate_trigger(struct iio_dev *indio_dev,
				      struct iio_trigger *trig)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (st->trig != trig)
		return -EINVAL;

	return 0;
}

static int max30210_read_event(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				enum iio_event_type type,
				enum iio_event_direction dir,
				enum iio_event_info info, int *val, int *val2)
{
	struct max30210_state *st = iio_priv(indio_dev);
	int ret;

	if (info == IIO_EV_INFO_VALUE) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_read_temp_scaled(st->regmap,
						       MAX30210_TEMP_ALM_HI_REG,
						       val, val2);
			case IIO_EV_TYPE_ROC:
				ret = regmap_read(st->regmap,
						  MAX30210_TEMP_INC_THRESH_REG,
						  val);
				if (ret)
					return ret;

				*val *= 5;

				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		case IIO_EV_DIR_FALLING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_read_temp_scaled(st->regmap,
						       MAX30210_TEMP_ALM_LO_REG,
						       val, val2);
			case IIO_EV_TYPE_ROC:
				ret = regmap_read(st->regmap,
						  MAX30210_TEMP_DEC_THRESH_REG,
						  val);
				if (ret)
					return ret;

				*val *= -5;

				return IIO_VAL_INT;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
	} else if (info == IIO_EV_INFO_HYSTERESIS && dir == IIO_EV_DIR_RISING &&
		   type == IIO_EV_TYPE_THRESH) {
		return max30210_read_temp_scaled(st->regmap,
						 MAX30210_TEMP_ALM_LO_REG, val,
						 val2);
	}

	return -EINVAL;
}

static int max30210_write_event(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 enum iio_event_type type,
				 enum iio_event_direction dir,
				 enum iio_event_info info, int val, int val2)
{
	struct max30210_state *st = iio_priv(indio_dev);

	if (info == IIO_EV_INFO_VALUE) {
		switch (dir) {
		case IIO_EV_DIR_RISING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_write_temp_scaled(st->regmap,
						       MAX30210_TEMP_ALM_HI_REG,
						       val, val2);
			case IIO_EV_TYPE_ROC:
				return regmap_write(st->regmap,
						   MAX30210_TEMP_INC_THRESH_REG,
						   val / 5);
			default:
				return -EINVAL;
			}
			break;
		case IIO_EV_DIR_FALLING:
			switch (type) {
			case IIO_EV_TYPE_THRESH:
				return max30210_write_temp_scaled(st->regmap,
						       MAX30210_TEMP_ALM_LO_REG,
						       val, val2);
			case IIO_EV_TYPE_ROC:
				return regmap_write(st->regmap,
						   MAX30210_TEMP_DEC_THRESH_REG,
						   val / -5);
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}
	} else if (info == IIO_EV_INFO_HYSTERESIS && dir == IIO_EV_DIR_RISING &&
		   type == IIO_EV_TYPE_THRESH) {
		return max30210_write_temp_scaled(st->regmap,
						  MAX30210_TEMP_ALM_LO_REG, val,
						  val2);
	}

	return -EINVAL;
}

static int max30210_read_event_config(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       enum iio_event_type type,
				       enum iio_event_direction dir)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	ret = regmap_read(st->regmap, MAX30210_INT_EN_REG, &val);
	if (ret)
		return ret;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			return FIELD_GET(MAX30210_TEMP_HI_MASK, val);
		case IIO_EV_TYPE_ROC:
			return FIELD_GET(MAX30210_TEMP_INC_MASK, val);
		default:
			return -EINVAL;
		}
		break;
	case IIO_EV_DIR_FALLING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			return FIELD_GET(MAX30210_TEMP_LO_MASK, val);
		case IIO_EV_TYPE_ROC:
			return FIELD_GET(MAX30210_TEMP_DEC_MASK, val);
		default:
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}
}

static int max30210_write_event_config(struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan,
					enum iio_event_type type,
					enum iio_event_direction dir, int state)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int val;

	switch (dir) {
	case IIO_EV_DIR_RISING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			val = MAX30210_INT_EN(state, MAX30210_TEMP_HI_MASK);

			return regmap_update_bits(st->regmap,
						  MAX30210_INT_EN_REG,
						  MAX30210_TEMP_HI_MASK, val);
		case IIO_EV_TYPE_ROC:
			val = MAX30210_INT_EN(state, MAX30210_TEMP_INC_MASK);

			return regmap_update_bits(st->regmap,
						  MAX30210_INT_EN_REG,
						  MAX30210_TEMP_INC_MASK, val);
		default:
			return -EINVAL;
		}
		break;
	case IIO_EV_DIR_FALLING:
		switch (type) {
		case IIO_EV_TYPE_THRESH:
			val = MAX30210_INT_EN(state, MAX30210_TEMP_LO_MASK);

			return regmap_update_bits(st->regmap,
						  MAX30210_INT_EN_REG,
						  MAX30210_TEMP_LO_MASK, val);
		case IIO_EV_TYPE_ROC:
			val = MAX30210_INT_EN(state, MAX30210_TEMP_DEC_MASK);

			return regmap_update_bits(st->regmap,
						  MAX30210_INT_EN_REG,
						  MAX30210_TEMP_DEC_MASK, val);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int max30210_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int uval;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		*val = 5;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->ext_cvt_en) {
			*val2 = pwm_get_period(st->ext_cvt_pwm);
			*val = NANO;
		} else {
			ret = regmap_read(st->regmap, MAX30210_TEMP_CONF_2_REG,
					  &uval);
			if (ret)
				return ret;

			uval = FIELD_GET(MAX30210_TEMP_PERIOD_MASK, uval);

			*val = 8;
			/**
			 * register values 0x9 to 0xF have the same sample rate
			 * of 8Hz
			 */
			*val2 = uval >= 0x9 ? 1 : BIT(0x9 - uval);
		}

		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_RAW:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, MAX30210_TEMP_CONV_REG,
				   MAX30210_CONV_T_MASK);
		if (ret)
			goto err_dmode;

		fsleep(8000);

		ret = max30210_read_temp_raw(st->regmap,
					     MAX30210_TEMP_DATA_REG, val);
err_dmode:
		iio_device_release_direct_mode(indio_dev);
		return ret;
	default:
		return -EINVAL;
	}
}

static int max30210_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type, int *length,
				long mask)
{
	struct max30210_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->ext_cvt_en) {
			*vals = samp_freq_range;
			*type = IIO_VAL_INT;
			*length = ARRAY_SIZE(samp_freq_range);

			return IIO_AVAIL_RANGE;
		}

		*vals = samp_freq_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(samp_freq_avail);

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int max30210_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int val,
			       int val2, long mask)
{
	struct max30210_state *st = iio_priv(indio_dev);
	struct pwm_state state;
	u64 data;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		if (st->ext_cvt_en) {
			if (val < MAX30210_EXT_CVT_FREQ_MIN ||
			    val > MAX30210_EXT_CVT_FREQ_MAX) {
				ret = -EINVAL;
				goto exit_samp;
			}

			data = NANO;
			do_div(data, val);

			/**
			 * Maintain duty cycle of 100% to power up the device
			 */
			pwm_get_state(st->ext_cvt_pwm, &state);
			state.period = data;
			pwm_set_relative_duty_cycle(&state, 1, 1);
			ret = pwm_apply_state(st->ext_cvt_pwm, &state);

			goto exit_samp;
		}

		/**
		 * micro_value = val * 1000000 + val2
		 * reg_value = ((micro_value * 64) / 1000000) - 1
		 */
		data = (val * MICRO + val2) << 6;
		do_div(data, MICRO);

		data = fls_long(data - 1);
		data = FIELD_PREP(MAX30210_TEMP_PERIOD_MASK, data);

		ret = regmap_update_bits(st->regmap, MAX30210_TEMP_CONF_2_REG,
					 MAX30210_TEMP_PERIOD_MASK,
					 (unsigned int)data);
exit_samp:
		iio_device_release_direct_mode(indio_dev);
		return ret;
	default:
		return -EINVAL;
	}
}

static int max30210_write_raw_get_fmt(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_trigger_ops max30210_trigger_ops = {
	.validate_device = &iio_trigger_validate_own_device,
};

static int max30210_set_watermark(struct iio_dev *indio_dev, unsigned int val)
{
	struct max30210_state *st = iio_priv(indio_dev);
	unsigned int reg;
	int ret;

	if (val < 1 || val > MAX30210_FIFO_SIZE)
		return -EINVAL;

	reg = MAX30210_FIFO_SIZE - val;

	ret = regmap_write(st->regmap, MAX30210_FIFO_CONF_1_REG, reg);
	if (ret)
		return ret;

	st->watermark = val;

	return 0;
}

static ssize_t hwfifo_watermark_min_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	return sysfs_emit(buf, "1\n");
}

static ssize_t hwfifo_watermark_max_show(struct device *dev,
					struct device_attribute *devattr,
					char *buf)
{
	return sysfs_emit(buf, "%d\n", MAX30210_FIFO_SIZE);
}

static ssize_t hwfifo_watermark_show(struct device *dev,
				      struct device_attribute *devattr,
				      char *buf)
{
	struct max30210_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%d\n", st->watermark);
}

static ssize_t hwfifo_enabled_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct max30210_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%d\n", st->hwfifo_enabled);
}

static IIO_DEVICE_ATTR_RO(hwfifo_watermark_min, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_watermark_max, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_watermark, 0);
static IIO_DEVICE_ATTR_RO(hwfifo_enabled, 0);

static const struct attribute *max30210_fifo_attributes[] = {
	&iio_dev_attr_hwfifo_watermark_min.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark_max.dev_attr.attr,
	&iio_dev_attr_hwfifo_watermark.dev_attr.attr,
	&iio_dev_attr_hwfifo_enabled.dev_attr.attr,
	NULL,
};

static int max30210_buffer_preenable(struct iio_dev *indio_dev)
{
	struct max30210_state *st = iio_priv(indio_dev);
	struct pwm_state state;
	int ret;

	ret = regmap_update_bits(st->regmap, MAX30210_INT_EN_REG,
				 MAX30210_A_FULL_MASK | MAX30210_TEMP_RDY_MASK,
				 MAX30210_TEMP_RDY_MASK);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, MAX30210_FIFO_CONF_2_REG,
				 MAX30210_FLUSH_FIFO_MASK,
				 MAX30210_FLUSH_FIFO_MASK);
	if (ret)
		return ret;

	if (st->ext_cvt_en) {
		ret = regmap_update_bits(st->regmap, MAX30210_PIN_CONF_REG,
					 MAX30210_EXT_CNV_EN_MASK,
					 MAX30210_EXT_CNV_EN_MASK);
		if (ret)
			return ret;

		/**
		 * Make pin function as EXT CVT pin:
		 * - polarity should be what is set in device tree
		 * - 6us duty cycle
		 */
		pwm_get_state(st->ext_cvt_pwm, &state);
		state.polarity = !st->ext_cvt_pol;
		state.duty_cycle = 6000;

		ret = pwm_apply_state(st->ext_cvt_pwm, &state);
	} else {
		ret = regmap_write(st->regmap, MAX30210_TEMP_CONV_REG,
				   MAX30210_AUTO_MASK | MAX30210_CONV_T_MASK);
	}
	if (ret)
		return ret;

	if (st->chg_det_en) {
		ret = regmap_update_bits(st->regmap, MAX30210_TEMP_CONF_1_REG,
					 MAX30210_CHG_DET_EN_MASK,
					 MAX30210_CHG_DET_EN_MASK);
		if (ret)
			return ret;
	}

	st->hwfifo_enabled = 1;

	return 0;
}

static int max30210_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct max30210_state *st = iio_priv(indio_dev);
	struct pwm_state state;
	int ret;

	ret = regmap_update_bits(st->regmap, MAX30210_INT_EN_REG,
				 MAX30210_A_FULL_MASK | MAX30210_TEMP_RDY_MASK,
				 0x0);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, MAX30210_FIFO_CONF_2_REG,
				 MAX30210_FLUSH_FIFO_MASK,
				 MAX30210_FLUSH_FIFO_MASK);
	if (ret)
		return ret;

	if (st->ext_cvt_en) {
		/**
		 * Make pin function as powerdown pin again (tie to VDD):
		 * - polarity active-high
		 * - 100% duty cycle
		 */
		pwm_get_state(st->ext_cvt_pwm, &state);
		state.polarity = PWM_POLARITY_NORMAL;
		pwm_set_relative_duty_cycle(&state, 1, 1);

		/** Power up device again */
		ret = pwm_apply_state(st->ext_cvt_pwm, &state);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, MAX30210_PIN_CONF_REG,
					 MAX30210_EXT_CNV_EN_MASK, 0x0);
	} else {
		ret = regmap_write(st->regmap, MAX30210_TEMP_CONV_REG, 0x0);
	}
	if (ret)
		return ret;

	if (st->chg_det_en) {
		ret = regmap_update_bits(st->regmap, MAX30210_TEMP_CONF_1_REG,
					 MAX30210_CHG_DET_EN_MASK, 0x0);
		if (ret)
			return ret;
	}

	st->hwfifo_enabled = 0;

	return 0;
}

static const struct iio_buffer_setup_ops max30210_buffer_ops = {
	.preenable = max30210_buffer_preenable,
	.postdisable = max30210_buffer_postdisable,
};

static const struct iio_info max30210_info = {
	.read_raw = max30210_read_raw,
	.read_avail = max30210_read_avail,
	.write_raw = max30210_write_raw,
	.write_raw_get_fmt = max30210_write_raw_get_fmt,
	.hwfifo_set_watermark = max30210_set_watermark,
	.debugfs_reg_access = &max30210_reg_access,
	.validate_trigger = &max30210_validate_trigger,
	.read_event_value = max30210_read_event,
	.write_event_value = max30210_write_event,
	.write_event_config = max30210_write_event_config,
	.read_event_config = max30210_read_event_config,
};

#define MAX30210_IIO_EVENT_TEMP_MAX { \
	.type = IIO_EV_TYPE_THRESH, \
	.dir = IIO_EV_DIR_RISING, \
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | \
			 BIT(IIO_EV_INFO_ENABLE), \
}

#define MAX30210_IIO_EVENT_TEMP_MIN { \
	.type = IIO_EV_TYPE_THRESH, \
	.dir = IIO_EV_DIR_FALLING, \
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | \
			 BIT(IIO_EV_INFO_ENABLE), \
}

#define MAX30210_IIO_EVENT_TEMP_MAX_HYST { \
	.type = IIO_EV_TYPE_THRESH, \
	.dir = IIO_EV_DIR_RISING, \
	.mask_separate = BIT(IIO_EV_INFO_HYSTERESIS), \
}

#define MAX30210_IIO_EVENT_TEMP_ROC_RISING { \
	.type = IIO_EV_TYPE_ROC, \
	.dir = IIO_EV_DIR_RISING, \
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | \
			 BIT(IIO_EV_INFO_ENABLE), \
}

#define MAX30210_IIO_EVENT_TEMP_ROC_FALLING { \
	.type = IIO_EV_TYPE_ROC, \
	.dir = IIO_EV_DIR_FALLING, \
	.mask_separate = BIT(IIO_EV_INFO_VALUE) | \
			 BIT(IIO_EV_INFO_ENABLE), \
}

static const struct iio_event_spec max30210_events_default[] = {
	MAX30210_IIO_EVENT_TEMP_MAX,
	MAX30210_IIO_EVENT_TEMP_MIN,
};

static const struct iio_event_spec max30210_events_roc[] = {
	MAX30210_IIO_EVENT_TEMP_MAX,
	MAX30210_IIO_EVENT_TEMP_MIN,
	MAX30210_IIO_EVENT_TEMP_ROC_RISING,
	MAX30210_IIO_EVENT_TEMP_ROC_FALLING,
};

static const struct iio_event_spec max30210_events_comp_mode[] = {
	MAX30210_IIO_EVENT_TEMP_MAX,
	MAX30210_IIO_EVENT_TEMP_MAX_HYST,
};

static const struct iio_event_spec max30210_events_comp_mode_roc[] = {
	MAX30210_IIO_EVENT_TEMP_MAX,
	MAX30210_IIO_EVENT_TEMP_MAX_HYST,
	MAX30210_IIO_EVENT_TEMP_ROC_RISING,
	MAX30210_IIO_EVENT_TEMP_ROC_FALLING,
};

#define MAX30210_IIO_CHAN(_suffix) \
	static const struct iio_chan_spec max30210_channels_##_suffix = { \
		.type = IIO_TEMP, \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
				      BIT(IIO_CHAN_INFO_SCALE) | \
				      BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.output = 0, \
		.scan_index = 0, \
		.event_spec = max30210_events_##_suffix, \
		.num_event_specs = ARRAY_SIZE(max30210_events_##_suffix), \
		.scan_type = { \
			.sign = 's', \
			.realbits = 16, \
			.storagebits = 32, \
			.shift = 8, \
			.endianness = IIO_BE, \
		}, \
	}

MAX30210_IIO_CHAN(default);
MAX30210_IIO_CHAN(roc);
MAX30210_IIO_CHAN(comp_mode);
MAX30210_IIO_CHAN(comp_mode_roc);

static int max30210_show_serial_number(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct max30210_state *st = iio_priv(indio_dev);

	u8 uval[8] __aligned(IIO_DMA_MINALIGN);
	int ret;

	ret = regmap_bulk_read(st->regmap, MAX30210_UNIQUE_ID_REG, uval,
			       MAX30210_UNIQUE_ID_LEN);
	if (ret)
		return ret;

	*val = get_unaligned_be64(uval);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(max30210_serial_number_fops,
			  max30210_show_serial_number, NULL, "0x%.6llx\n");

static int max30210_show_roc(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct max30210_state *st = iio_priv(indio_dev);

	u8 uval[2] __aligned(IIO_DMA_MINALIGN);
	int ret;

	ret = regmap_bulk_read(st->regmap, MAX30210_TEMP_SLOPE_REG, uval, 2);
	if (ret)
		return ret;

	*val = sign_extend64(get_unaligned_be16(uval), 8) * 5;

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(max30210_roc_fops, max30210_show_roc, NULL, "%lld\n");

static void max30210_debugfs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);
	struct max30210_state *st = iio_priv(indio_dev);

	if (!IS_ENABLED(CONFIG_DEBUG_FS))
		return;

	debugfs_create_file_unsafe("serial_number", 0400, d, indio_dev,
				   &max30210_serial_number_fops);
	if (st->chg_det_en)
		debugfs_create_file_unsafe("roc", 0400, d, indio_dev,
					   &max30210_roc_fops);
}

static int max30210_setup(struct max30210_state *st, struct device *dev,
			   u8 *dir)
{
	struct pwm_state state;
	unsigned int val;
	u32 prop_val;
	int ret;
	bool prop;

	st->ext_cvt_pwm = devm_pwm_get(dev, "ext-cvt-pwm");
	if (!IS_ERR(st->ext_cvt_pwm)) {
		st->ext_cvt_en = true;
		st->ext_cvt_pol = device_property_read_bool(dev,
						"adi,ext-conv-rising-edge");

		ret = pwm_enable(st->ext_cvt_pwm);
		if (ret)
			return ret;

		/**
		 * Power down to reset device (tie to GND)
		 * - polarity = active-high
		 * - 1Hz frequency (default)
		 * - 0% duty cycle
		 */
		pwm_get_state(st->ext_cvt_pwm, &state);
		state.polarity = PWM_POLARITY_NORMAL;
		state.period = NANO;
		pwm_set_relative_duty_cycle(&state, 0, 1);
		ret = pwm_apply_state(st->ext_cvt_pwm, &state);
		if (ret)
			return ret;

		/**
		 * Power up device (tie to VDD)
		 * - 100% duty cycle
		 */
		pwm_get_state(st->ext_cvt_pwm, &state);
		pwm_set_relative_duty_cycle(&state, 1, 1);
		ret = pwm_apply_state(st->ext_cvt_pwm, &state);
		if (ret)
			return ret;
	} else {
		st->ext_cvt_en = false;

		/** Power down to reset device */
		st->powerdown_gpio = devm_gpiod_get_optional(dev, "powerdown",
							     GPIOD_OUT_HIGH);
		if (IS_ERR(st->powerdown_gpio))
			return dev_err_probe(dev, PTR_ERR(st->powerdown_gpio),
					 "Failed to request powerdown GPIO.\n");

		/** Power up device */
		gpiod_set_value(st->powerdown_gpio, 0);
	}

	fsleep(700);

	if (st->ext_cvt_en && st->ext_cvt_pol)
		ret = regmap_update_bits(st->regmap,
					 MAX30210_PIN_CONF_REG,
					 MAX30210_EXT_CVT_ICFG_MASK,
					 MAX30210_EXT_CVT_ICFG_MASK);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, MAX30210_PART_ID_REG, &val);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read Part ID.\n");
	if (val != MAX30210_PART_ID)
		dev_err_probe(dev, -ENODEV, "Part ID mismatch.\n");

	st->chg_det_en = device_property_read_bool(dev, "adi,roc-en");
	if (st->chg_det_en)
		__set_bit(0, (unsigned long *)&st->events_type);

	ret = device_property_read_u32(dev, "adi,roc-len", &prop_val);
	if (!ret) {
		switch (prop_val) {
		case 2:
		case 3:
		case 5:
		case 9:
		case 17:
		case 33:
		case 65:
			prop_val = ilog2(prop_val - 1);
			prop_val = FIELD_PREP(MAX30210_RATE_CHG_FILTER_MASK,
					      prop_val);
			ret = regmap_update_bits(st->regmap,
						 MAX30210_TEMP_CONF_1_REG,
						 MAX30210_RATE_CHG_FILTER_MASK,
						 prop_val);
			if (ret)
				return ret;
			break;
		default:
			return dev_err_probe(dev, -EINVAL,
					     "Invalid adi,roc-len.\n");
		}
	}

	prop = device_property_read_bool(dev, "adi,comp-int");
	if (prop)
		__set_bit(1, (unsigned long *)&st->events_type);
	val = FIELD_PREP(MAX30210_ALERT_MODE_MASK, prop);
	ret = regmap_update_bits(st->regmap, MAX30210_TEMP_CONF_2_REG,
				 MAX30210_ALERT_MODE_MASK, val);
	if (ret)
		return ret;

	/** Clear status byte */
	return regmap_read(st->regmap, MAX30210_STATUS_REG, &val);
}

static int max30210_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct max30210_state *st;
	u8 dir;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable vdd regulator.\n");

	st->regmap = devm_regmap_init_i2c(client, &max30210_regmap);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to allocate regmap.\n");

	ret = max30210_setup(st, dev, &dir);
	if (ret)
		return ret;

	indio_dev->name = "max30210";
	indio_dev->info = &max30210_info;
	switch (st->events_type) {
	case MAX30210_DEFAULT_EVENTS:
		indio_dev->channels = &max30210_channels_default;
		break;
	case MAX30210_ROC_EVENTS:
		indio_dev->channels = &max30210_channels_roc;
		break;
	case MAX30210_COMP_MODE_EVENTS:
		indio_dev->channels = &max30210_channels_comp_mode;
		break;
	case MAX30210_COMP_MODE_ROC_EVENTS:
		indio_dev->channels = &max30210_channels_comp_mode_roc;
		break;
	default:
		indio_dev->channels = &max30210_channels_default;
		break;
	}
	indio_dev->num_channels = 1;
	indio_dev->modes = INDIO_DIRECT_MODE;

	ret = devm_iio_triggered_buffer_setup_ext(dev, indio_dev, NULL,
						  max30210_trigger_handler,
						  IIO_BUFFER_DIRECTION_IN,
						  &max30210_buffer_ops,
						  max30210_fifo_attributes);
	if (ret < 0)
		return ret;

	if (client->irq) {
		st->trig = devm_iio_trigger_alloc(dev, "%s-dev%d",
						  indio_dev->name,
						  iio_device_id(indio_dev));
		if (!st->trig)
			return -ENOMEM;

		st->trig->ops = &max30210_trigger_ops;
		iio_trigger_set_drvdata(st->trig, indio_dev);
		ret = devm_iio_trigger_register(dev, st->trig);
		if (ret)
			return ret;

		indio_dev->trig = st->trig;
		ret = devm_request_threaded_irq(dev, client->irq,
					      iio_trigger_generic_data_rdy_poll,
					      NULL, IRQF_TRIGGER_FALLING,
					      indio_dev->name, st->trig);
		if (ret)
			return ret;
	}

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	max30210_debugfs_init(indio_dev);

	return 0;
}

static const struct i2c_device_id max30210_id[] = {
	{ "max30210", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max30210_id);

static const struct of_device_id max30210_of_match[] = {
	{ .compatible = "adi,max30210" },
	{ }
};
MODULE_DEVICE_TABLE(of, max30210_of_match);

static struct i2c_driver max30210_driver = {
	.driver = {
		.name = "max30210",
		.of_match_table = max30210_of_match,
	},
	.probe_new = max30210_probe,
	.id_table = max30210_id,
};
module_i2c_driver(max30210_driver);

MODULE_AUTHOR("John Erasmus Mari Geronimo <johnerasmusmari.geronimo@analog.com");
MODULE_AUTHOR("Daniel Matyas <daniel.matyas@analog.com>");
MODULE_DESCRIPTION("MAX30210 low-power digital temperature sensor");
MODULE_LICENSE("GPL");
