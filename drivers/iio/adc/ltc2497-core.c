// SPDX-License-Identifier: GPL-2.0-only
/*
 * ltc2497-core.c - Common code for Analog Devices/Linear Technology
 * LTC2496 and LTC2497 ADCs
 *
 * Copyright (C) 2017 Analog Devices Inc.
 */

#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/device/devres.h>
#include <linux/gfp.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include "ltc2497.h"

#define LTC2497_SGL			BIT(4)
#define LTC2497_DIFF			0
#define LTC2497_SIGN			BIT(3)

/*
 * Output-rate modes as [rejection][{1x, 2x}], each a (Hz, microHz) pair taken
 * as 1 / t_CONV typ for that combination.  The advertised sampling_frequency
 * and the conversion-time budget are two views of the same mode, so the rows
 * here are kept in lock-step with ltc2497core_conv_time_ms_tbl.
 */
static const int ltc2497core_samp_freq_avail[][4] = {
	[LTC2499_REJECT_50_60]	= { 6, 800000, 13, 600000 },
	[LTC2499_REJECT_50]	= { 6, 240000, 12, 450000 },
	[LTC2499_REJECT_60]	= { 7, 490000, 14, 950000 },
};

static const unsigned int ltc2497core_conv_time_ms_tbl[][2] = {
	[LTC2499_REJECT_50_60]	= { LTC2497_CONV_TIME_50_60_1X_MS,
				    LTC2499_CONV_TIME_50_60_2X_MS },
	[LTC2499_REJECT_50]	= { LTC2499_CONV_TIME_50_1X_MS,
				    LTC2499_CONV_TIME_50_2X_MS },
	[LTC2499_REJECT_60]	= { LTC2499_CONV_TIME_60_1X_MS,
				    LTC2499_CONV_TIME_60_2X_MS },
};

static unsigned int ltc2497core_conv_time_ms(struct ltc2497core_driverdata *ddata,
					     u8 address)
{
	/*
	 * SPD is ignored by the part during a temperature measurement: it
	 * always converts at 1x, but the rejection selection still applies, so
	 * budget the 1x cell of the current rejection row regardless of the
	 * selected voltage-channel mode.
	 */
	if (address == LTC2497_TEMP_ADDR)
		return ltc2497core_conv_time_ms_tbl[ddata->rejection][0];

	return ltc2497core_conv_time_ms_tbl[ddata->rejection][ddata->speed_2x];
}

static int ltc2497core_wait_conv(struct ltc2497core_driverdata *ddata,
				 unsigned int conv_time_ms)
{
	s64 time_elapsed;

	time_elapsed = ktime_ms_delta(ktime_get(), ddata->time_prev);

	if (time_elapsed < conv_time_ms) {
		/* delay if conversion time not passed
		 * since last read or write
		 */
		if (msleep_interruptible(conv_time_ms - time_elapsed))
			return -ERESTARTSYS;

		return 0;
	}

	if (time_elapsed - conv_time_ms <= 0) {
		/* We're in automatic mode -
		 * so the last reading is still not outdated
		 */
		return 0;
	}

	return 1;
}

static int ltc2497core_read(struct ltc2497core_driverdata *ddata, u8 address, int *val)
{
	unsigned int conv_time_ms = ltc2497core_conv_time_ms(ddata, address);
	int ret;

	/*
	 * Wait for the conversion currently in flight, whose duration was fixed
	 * by the mode active when it was started (ddata->conv_time_prev).  This
	 * can be longer than the freshly selected mode's time - e.g. a 1x
	 * conversion is still running when the first 2x read arrives after a
	 * sampling_frequency change - and reprogramming the device before it
	 * finishes would be NACKed (-EIO).
	 */
	ret = ltc2497core_wait_conv(ddata, ddata->conv_time_prev);
	if (ret < 0)
		return ret;

	if (ret || ddata->addr_prev != address) {
		ret = ddata->result_and_measure(ddata, address, NULL);
		if (ret < 0)
			return ret;
		ddata->addr_prev = address;

		/*
		 * The reprogram above starts a conversion in the new mode.
		 * Record its start time and duration before sleeping, so that if
		 * msleep_interruptible() is interrupted the conversion state is
		 * already consistent: the next retry then waits only the time
		 * remaining from the real start instead of from a stale
		 * time_prev, which would let it reprogram/read too early.
		 */
		ddata->time_prev = ktime_get();
		ddata->conv_time_prev = conv_time_ms;
		if (msleep_interruptible(conv_time_ms))
			return -ERESTARTSYS;
	}

	ret = ddata->result_and_measure(ddata, address, val);
	if (ret < 0)
		return ret;

	ddata->time_prev = ktime_get();
	/* The read above auto-starts the next conversion in the current mode. */
	ddata->conv_time_prev = conv_time_ms;

	return ret;
}

static int ltc2497core_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&ddata->lock);
		ret = ltc2497core_read(ddata, chan->address, val);
		mutex_unlock(&ddata->lock);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(ddata->ref);
		if (ret < 0)
			return ret;

		switch (chan->type) {
		case IIO_TEMP:
			/*
			 * raw is normalised to 2^(resolution + 1), i.e.
			 * raw = 2 * DATAOUT24, so the PTAT scale (datasheet
			 * Vref / 1570 per Kelvin) doubles its denominator and,
			 * in m°C, becomes Vref_uV / 3140000.
			 */
			*val = ret;
			*val2 = 3140000;
			return IIO_VAL_FRACTIONAL;
		case IIO_VOLTAGE:
			*val = ret / (MICRO / MILLI);
			*val2 = ddata->chip_info->resolution + 1;
			return IIO_VAL_FRACTIONAL_LOG2;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_TEMP:
			ret = regulator_get_voltage(ddata->ref);
			if (ret < 0)
				return ret;
			if (ret == 0)
				return -EINVAL;
			/*
			 * 0 °C == 273.15 K must map to raw + offset such that
			 * (raw + offset) * scale == 0 m°C, i.e.
			 *   offset = -273150 / scale
			 *          = -273150 * 3140000 / Vref_uV
			 * Computed in 64-bit to avoid overflow.
			 */
			*val = div_s64(ABSOLUTE_ZERO_MILLICELSIUS * 3140000LL, ret);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		/*
		 * Only advertised on the voltage channels of parts with a speed
		 * mode; the sampling frequency is a property of the selected 1x/2x
		 * mode and rejection mode, not of an individual conversion.
		 */
		mutex_lock(&ddata->lock);
		*val = ltc2497core_samp_freq_avail[ddata->rejection][ddata->speed_2x * 2];
		*val2 = ltc2497core_samp_freq_avail[ddata->rejection][ddata->speed_2x * 2 + 1];
		mutex_unlock(&ddata->lock);

		return IIO_VAL_INT_PLUS_MICRO;

	default:
		return -EINVAL;
	}
}

static int ltc2497core_read_avail(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  const int **vals, int *type, int *length,
				  long mask)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		guard(mutex)(&ddata->lock);

		*vals = ltc2497core_samp_freq_avail[ddata->rejection];
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(ltc2497core_samp_freq_avail[0]);
		return IIO_AVAIL_LIST;
	}
	default:
		return -EINVAL;
	}
}

static int ltc2497core_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);
	const int *avail;
	unsigned int i;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ: {
		guard(mutex)(&ddata->lock);

		/*
		 * Match the (val, val2) pair against the rates advertised for
		 * the rejection mode currently selected.
		 */
		avail = ltc2497core_samp_freq_avail[ddata->rejection];
		for (i = 0; i < ARRAY_SIZE(ltc2497core_samp_freq_avail[0]); i += 2) {
			if (val == avail[i] && val2 == avail[i + 1])
				break;
		}
		if (i == ARRAY_SIZE(ltc2497core_samp_freq_avail[0]))
			return -EINVAL;

		ddata->speed_2x = i / 2;
		/*
		 * The new speed only takes effect once the second command byte
		 * is reprogrammed, so force the next read to reprogram rather
		 * than reuse the value already latched for this address.
		 * LTC2497_CONFIG_DEFAULT is not a valid channel/temperature
		 * address, so it is a safe re-arm sentinel (as used at probe).
		 *
		 * A conversion started under the old speed may still be in
		 * flight; its own duration (conv_time_prev), not the new mode's,
		 * still gates the next reprogram, so the timing state is left
		 * untouched here.
		 */
		ddata->addr_prev = LTC2497_CONFIG_DEFAULT;

		return 0;
	}
	default:
		return -EINVAL;
	}
}

static const char * const ltc2497core_filter_modes_str[] = {
	[LTC2499_REJECT_50_60]	= "50_60",
	[LTC2499_REJECT_50]	= "50",
	[LTC2499_REJECT_60]	= "60",
};

static int ltc2497core_set_filter_mode(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan,
				       unsigned int val)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);

	guard(mutex)(&ddata->lock);

	ddata->rejection = val;
	/*
	 * The new rejection mode only takes effect once the second command byte
	 * is reprogrammed, so force the next read to reprogram rather than reuse
	 * the value already latched for this address.  LTC2497_CONFIG_DEFAULT is
	 * not a valid channel/temperature address, so it is a safe re-arm
	 * sentinel (as used at probe).
	 *
	 * A conversion started under the old rejection mode may still be in
	 * flight; its own duration (conv_time_prev), not the new mode's, still
	 * gates the next reprogram, so the timing state is left untouched here.
	 */
	ddata->addr_prev = LTC2497_CONFIG_DEFAULT;

	return 0;
}

static int ltc2497core_get_filter_mode(struct iio_dev *indio_dev,
				       const struct iio_chan_spec *chan)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);

	guard(mutex)(&ddata->lock);

	return ddata->rejection;
}

static const struct iio_enum ltc2497core_filter_mode_enum = {
	.items = ltc2497core_filter_modes_str,
	.num_items = ARRAY_SIZE(ltc2497core_filter_modes_str),
	.set = ltc2497core_set_filter_mode,
	.get = ltc2497core_get_filter_mode,
};

/*
 * Rejection applies to voltage and temperature alike, so the attribute is
 * device-wide.
 */
static const struct iio_chan_spec_ext_info ltc2497core_ext_info[] = {
	IIO_ENUM("filter_mode", IIO_SHARED_BY_ALL, &ltc2497core_filter_mode_enum),
	IIO_ENUM_AVAILABLE("filter_mode", IIO_SHARED_BY_ALL,
			   &ltc2497core_filter_mode_enum),
	{ }
};

#define LTC2497_CHAN(_chan, _addr, _ds_name, _extra_mask) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_chan), \
	.address = (_addr | (_chan / 2) | ((_chan & 1) ? LTC2497_SIGN : 0)), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | (_extra_mask), \
	.info_mask_shared_by_type_available = (_extra_mask), \
	.datasheet_name = (_ds_name), \
}

#define LTC2497_CHAN_DIFF(_chan, _addr, _extra_mask) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_chan) * 2 + ((_addr) & LTC2497_SIGN ? 1 : 0), \
	.channel2 = (_chan) * 2 + ((_addr) & LTC2497_SIGN ? 0 : 1),\
	.address = (_addr | _chan), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) | (_extra_mask), \
	.info_mask_shared_by_type_available = (_extra_mask), \
	.differential = 1, \
}

/*
 * SPD is ignored by the part during a temperature measurement, so the
 * temperature channel carries no sampling-frequency attribute.
 */
#define LTC2497_TEMP_CHANNEL { \
	.type = IIO_TEMP, \
	.address = LTC2497_TEMP_ADDR, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			      BIT(IIO_CHAN_INFO_SCALE) | \
			      BIT(IIO_CHAN_INFO_OFFSET), \
}

/*
 * Parts with a speed mode advertise sampling_frequency on their voltage
 * channels; the two tables are otherwise identical.
 */
#define LTC2497_DEFINE_CHANNELS(_name, _extra_mask)                     \
static const struct iio_chan_spec _name[] = {                           \
	LTC2497_CHAN(0, LTC2497_SGL, "CH0", _extra_mask),               \
	LTC2497_CHAN(1, LTC2497_SGL, "CH1", _extra_mask),               \
	LTC2497_CHAN(2, LTC2497_SGL, "CH2", _extra_mask),               \
	LTC2497_CHAN(3, LTC2497_SGL, "CH3", _extra_mask),               \
	LTC2497_CHAN(4, LTC2497_SGL, "CH4", _extra_mask),               \
	LTC2497_CHAN(5, LTC2497_SGL, "CH5", _extra_mask),               \
	LTC2497_CHAN(6, LTC2497_SGL, "CH6", _extra_mask),               \
	LTC2497_CHAN(7, LTC2497_SGL, "CH7", _extra_mask),               \
	LTC2497_CHAN(8, LTC2497_SGL, "CH8", _extra_mask),               \
	LTC2497_CHAN(9, LTC2497_SGL, "CH9", _extra_mask),               \
	LTC2497_CHAN(10, LTC2497_SGL, "CH10", _extra_mask),             \
	LTC2497_CHAN(11, LTC2497_SGL, "CH11", _extra_mask),             \
	LTC2497_CHAN(12, LTC2497_SGL, "CH12", _extra_mask),             \
	LTC2497_CHAN(13, LTC2497_SGL, "CH13", _extra_mask),             \
	LTC2497_CHAN(14, LTC2497_SGL, "CH14", _extra_mask),             \
	LTC2497_CHAN(15, LTC2497_SGL, "CH15", _extra_mask),             \
	LTC2497_CHAN_DIFF(0, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(1, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(2, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(3, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(4, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(5, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(6, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(7, LTC2497_DIFF, _extra_mask),                \
	LTC2497_CHAN_DIFF(0, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(1, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(2, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(3, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(4, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(5, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(6, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_CHAN_DIFF(7, LTC2497_DIFF | LTC2497_SIGN, _extra_mask), \
	LTC2497_TEMP_CHANNEL,                                           \
}

LTC2497_DEFINE_CHANNELS(ltc2497core_channel, 0);
LTC2497_DEFINE_CHANNELS(ltc2497core_channel_samp_freq, BIT(IIO_CHAN_INFO_SAMP_FREQ));

static const struct iio_chan_spec ltc2493_channel[] = {
	LTC2497_CHAN(0, LTC2497_SGL, "CH0", BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN(1, LTC2497_SGL, "CH1", BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN(2, LTC2497_SGL, "CH2", BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN(3, LTC2497_SGL, "CH3", BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN_DIFF(0, LTC2497_DIFF, BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN_DIFF(1, LTC2497_DIFF, BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN_DIFF(0, LTC2497_DIFF | LTC2497_SIGN,
			  BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_CHAN_DIFF(1, LTC2497_DIFF | LTC2497_SIGN,
			  BIT(IIO_CHAN_INFO_SAMP_FREQ)),
	LTC2497_TEMP_CHANNEL,
};

/*
 * The temperature channel is the last entry of each table, so a part without
 * the sensor stops one short of it.
 */
const struct ltc2497_channel_set ltc2497_chanset_ltc2497 = {
	.channels	= ltc2497core_channel,
	.num_channels	= ARRAY_SIZE(ltc2497core_channel) - 1,
};
EXPORT_SYMBOL_NS_GPL(ltc2497_chanset_ltc2497, "LTC2497");

const struct ltc2497_channel_set ltc2497_chanset_ltc2499 = {
	.channels	= ltc2497core_channel_samp_freq,
	.num_channels	= ARRAY_SIZE(ltc2497core_channel_samp_freq),
};
EXPORT_SYMBOL_NS_GPL(ltc2497_chanset_ltc2499, "LTC2497");

const struct ltc2497_channel_set ltc2497_chanset_ltc2493 = {
	.channels	= ltc2493_channel,
	.num_channels	= ARRAY_SIZE(ltc2493_channel),
};
EXPORT_SYMBOL_NS_GPL(ltc2497_chanset_ltc2493, "LTC2497");

static const struct iio_info ltc2497core_info = {
	.read_raw = ltc2497core_read_raw,
	.read_avail = ltc2497core_read_avail,
	.write_raw = ltc2497core_write_raw,
};

int ltc2497core_probe(struct device *dev, struct iio_dev *indio_dev)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);
	struct iio_map *plat_data = dev_get_platdata(dev);
	int ret;

	/*
	 * Keep using dev_name() for the iio_dev's name on some of the parts,
	 * since updating it would result in a ABI breakage.
	 */
	if (ddata->chip_info->name)
		indio_dev->name = ddata->chip_info->name;
	else
		indio_dev->name = dev_name(dev);

	indio_dev->info = &ltc2497core_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ddata->chip_info->channel_set->channels;
	indio_dev->num_channels = ddata->chip_info->channel_set->num_channels;

	if (ddata->chip_info->has_rejection_mode) {
		struct iio_chan_spec *channels;

		channels = devm_kmemdup(dev, indio_dev->channels,
					indio_dev->num_channels * sizeof(*channels),
					GFP_KERNEL);
		if (!channels)
			return -ENOMEM;

		for (unsigned int i = 0; i < indio_dev->num_channels; i++)
			channels[i].ext_info = ltc2497core_ext_info;

		indio_dev->channels = channels;
	}

	ret = ddata->result_and_measure(ddata, LTC2497_CONFIG_DEFAULT, NULL);
	if (ret < 0)
		return ret;

	ddata->ref = devm_regulator_get(dev, "vref");
	if (IS_ERR(ddata->ref))
		return dev_err_probe(dev, PTR_ERR(ddata->ref),
				     "Failed to get vref regulator\n");

	ret = regulator_enable(ddata->ref);
	if (ret < 0) {
		dev_err(dev, "Failed to enable vref regulator: %pe\n",
			ERR_PTR(ret));
		return ret;
	}

	ret = iio_map_array_register(indio_dev, plat_data);
	if (ret) {
		dev_err(&indio_dev->dev, "iio map err: %d\n", ret);
		goto err_regulator_disable;
	}

	ddata->addr_prev = LTC2497_CONFIG_DEFAULT;
	ddata->time_prev = ktime_get();
	/*
	 * Power-on default is simultaneous 50/60Hz rejection at 1x; a conversion
	 * is already in flight.
	 */
	ddata->conv_time_prev = LTC2497_CONV_TIME_50_60_1X_MS;

	mutex_init(&ddata->lock);

	ret = iio_device_register(indio_dev);
	if (ret < 0)
		goto err_array_unregister;

	return 0;

err_array_unregister:
	iio_map_array_unregister(indio_dev);

err_regulator_disable:
	regulator_disable(ddata->ref);

	return ret;
}
EXPORT_SYMBOL_NS(ltc2497core_probe, "LTC2497");

void ltc2497core_remove(struct iio_dev *indio_dev)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	iio_map_array_unregister(indio_dev);

	regulator_disable(ddata->ref);
}
EXPORT_SYMBOL_NS(ltc2497core_remove, "LTC2497");

MODULE_DESCRIPTION("common code for LTC2496/LTC2497 drivers");
MODULE_LICENSE("GPL v2");
