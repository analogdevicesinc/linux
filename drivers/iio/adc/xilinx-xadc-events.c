/*
 * Xilinx XADC driver
 *
 * Copyright 2013 Analog Devices Inc.
 *  Author: Lars-Peter Clauen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/kernel.h>
#include <linux/iio/iio.h>
#include <linux/iio/events.h>

#include "xilinx-xadc.h"

static void xadc_handle_event(struct iio_dev *indio_dev, unsigned int event)
{
	struct xadc *xadc = iio_priv(indio_dev);
	const struct iio_chan_spec *chan;
	unsigned int offset;
	uint16_t val;
	int ret;

	/* Temperature threshold error, we don't handle this yet */
	if (event == 0)
		return;

	if (event < 4)
		offset = event;
	else
		offset = event + 4;

	if (event < 3)
		chan = &indio_dev->channels[event];
	else if (event == 3)
		chan = &indio_dev->channels[0];
	else
		chan = &indio_dev->channels[event - 1];

	if (event != 3) {
		ret = xadc_read_reg(xadc, chan->address, &val);
		if (ret)
			return;

		if ((xadc->threshold_state & BIT(offset)) &&
			val >= xadc->threshold[offset]) {
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(chan->type, chan->channel,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
				iio_get_time_ns());
		}

		if ((xadc->threshold_state & BIT(offset + 4))
			&& val <= xadc->threshold[offset + 4]) {
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(chan->type, chan->channel,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_FALLING),
				iio_get_time_ns());
		}

	} else {
			iio_push_event(indio_dev,
				IIO_UNMOD_EVENT_CODE(chan->type, chan->channel,
					IIO_EV_TYPE_THRESH, IIO_EV_DIR_RISING),
				iio_get_time_ns());
	}

}

void xadc_handle_events(struct iio_dev *indio_dev, unsigned long events)
{
	unsigned int i;

	for_each_set_bit(i, &events, 8)
		xadc_handle_event(indio_dev, i);
}

static unsigned int xadc_get_active_alarms(struct xadc *xadc)
{
	unsigned int alarm;
	unsigned int state = xadc->threshold_state;

	alarm = ((state & 0x00f0) >> 4) | (state & 0x000f);
	alarm |= ((state & 0xf000) >> 8) | ((state & 0x0f00) >> 4);

	return alarm;
}

static unsigned xadc_get_threshold_offset(const struct iio_chan_spec *chan,
	enum iio_event_type type, enum iio_event_direction dir)
{
	unsigned int offset;

	if (chan->type == IIO_TEMP) {
		offset = 3;
	} else {
		if (chan->channel < 2)
			offset = chan->channel + 1;
		else
			offset = chan->channel + 6;
	}

	if (dir == IIO_EV_DIR_FALLING)
		offset += 4;

	return offset;
}

static int xadc_write_event_threshold(struct xadc *xadc,
	const struct iio_chan_spec *chan, unsigned int offset,
	unsigned int val)
{
	int ret;

	if (offset == 3) {
		/*
		 * According to the datasheet we need to set the lower 4 bits to
		 * 0x3, otherwise 125 degree celsius will be used as the threshold.
		 */
		val = (val & ~0xf) | 0x3;
	}

	ret = _xadc_write_reg(xadc, XADC_REG_THRESHOLD(offset), val);
	if (ret)
		return ret;

	if (chan->type == IIO_TEMP) {
		ret = _xadc_write_reg(xadc, XADC_REG_THRESHOLD(offset + 4),
			xadc->threshold[offset + 4]);
	}
	return ret;
}

int xadc_read_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir)
{
	unsigned int offset = xadc_get_threshold_offset(chan, type, dir);
	struct xadc *xadc = iio_priv(indio_dev);

	return (bool)(xadc->threshold_state & BIT(offset));
}

int xadc_write_event_config(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, int state)
{
	unsigned int offset = xadc_get_threshold_offset(chan, type, dir);
	struct xadc *xadc = iio_priv(indio_dev);
	unsigned int alarm;
	uint16_t val, cfg, old_cfg;
	int ret;

	mutex_lock(&xadc->mutex);

	if (!state) {
		/*
		 * Unfortunately there is only one alarm interrupt signal per
		 * channel and it is triggered by both low and high threshold
		 * events. So if we enable a threshold set the limit to the
		 * min/max value to avoid accidentally triggering the threshold.
		 */
		if (dir == IIO_EV_DIR_FALLING)
			val = 0;
		else
			val = 0xffff;
	} else {
		val = xadc->threshold[offset];
	}

	ret = xadc_write_event_threshold(xadc, chan, offset, val);
	if (ret)
		goto err_out;

	if (state)
		xadc->threshold_state |= BIT(offset);
	else
		xadc->threshold_state &= ~BIT(offset);

	alarm = xadc_get_active_alarms(xadc);

	xadc->ops->update_alarm(xadc, alarm);

	ret = _xadc_read_reg(xadc, XADC_REG_CONF1, &cfg);
	if (ret)
		goto err_out;
	old_cfg = cfg;
	cfg |= XADC_CONF1_ALARM_MASK;
	cfg &= ~((alarm & 0xf0) << 4); /* bram, pint, paux, ddr */
	cfg &= ~((alarm & 0x08) >> 3); /* ot */
	cfg &= ~((alarm & 0x07) << 1); /* temp, vccint, vccaux */
	if (old_cfg != cfg)
		ret = _xadc_write_reg(xadc, XADC_REG_CONF1, cfg);

err_out:
	mutex_unlock(&xadc->mutex);

	return ret;
}

int xadc_read_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info,
	int *val, int *val2)
{
	unsigned int offset = xadc_get_threshold_offset(chan, type, dir);
	struct xadc *xadc = iio_priv(indio_dev);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		*val = xadc->threshold[offset] >> 4;
		break;
	case IIO_EV_INFO_HYSTERESIS:
		*val = xadc->temp_hysteresis >> 4;
		break;
	default:
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

int xadc_write_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info,
	int val, int val2)
{
	unsigned int offset = xadc_get_threshold_offset(chan, type, dir);
	struct xadc *xadc = iio_priv(indio_dev);
	int ret = 0;

	val <<= 4;

	if (val < 0 || val > 0xffff)
		return -EINVAL;

	mutex_lock(&xadc->mutex);

	switch (info) {
	case IIO_EV_INFO_VALUE:
		xadc->threshold[offset] = val;
		break;
	case IIO_EV_INFO_HYSTERESIS:
		xadc->temp_hysteresis = val;
		break;
	default:
		return -EINVAL;
	}

	if (chan->type == IIO_TEMP) {
		if (xadc->threshold[offset] < xadc->temp_hysteresis) {
			xadc->threshold[offset + 4] = 0;
		} else {
			xadc->threshold[offset + 4] = xadc->threshold[offset] -
					xadc->temp_hysteresis;
		}
	}

	if (xadc->threshold_state & BIT(offset))
		ret = xadc_write_event_threshold(xadc, chan, offset, val);
	mutex_unlock(&xadc->mutex);

	return ret;
}
