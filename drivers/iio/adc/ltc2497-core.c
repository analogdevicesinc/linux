// SPDX-License-Identifier: GPL-2.0-only
/*
 * ltc2497-core.c - Common code for Analog Devices/Linear Technology
 * LTC2496 and LTC2497 ADCs
 *
 * Copyright (C) 2017 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>

#include "ltc2497.h"

#define LTC2497_SGL			BIT(4)
#define LTC2497_DIFF			0
#define LTC2497_SIGN			BIT(3)

/*
 * Output-rate modes, indexed by ltc2497core_driverdata.sped_2x
 * (0 = 1x, the power-on default; 1 = 2x, LTC2499 only).  The advertised
 * sampling_frequency and the conversion-time budget are two views of the same
 * mode, so they are kept in lock-step here and can never drift apart.  Only the
 * two simultaneous 50/60Hz rejection rates are reachable today; adding FA/FB
 * rejection selection later turns this into a [rejection][speed] lookup without
 * changing any caller.
 */
static const int ltc2497core_samp_freq_avail[] = {
	6, 800000,	/* 1x: ~6.8 Hz  (1 / t_CONV_1 typ 146.9ms) */
	13, 600000,	/* 2x: ~13.6 Hz (1 / t_CONV_2 typ  73.6ms) */
};

static const unsigned int ltc2497core_conv_time_ms_tbl[] = {
	LTC2497_CONV_TIME_1X_MS,		/* 1x */
	LTC2499_CONV_TIME_2X_MS,		/* 2x */
};

static unsigned int ltc2497core_conv_time_ms(struct ltc2497core_driverdata *ddata,
					     u8 address)
{
	/*
	 * SPD is ignored by the part during a temperature measurement: it
	 * always converts at 1x, so budget the 1x time regardless of the
	 * selected voltage-channel mode.
	 */
	if (address == LTC2497_TEMP_ADDR)
		return ltc2497core_conv_time_ms_tbl[0];

	return ltc2497core_conv_time_ms_tbl[ddata->sped_2x];
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

		/* The reprogram above starts a conversion in the new mode. */
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
			 * Vref / 1570 per kelvin) doubles its denominator and,
			 * in m°C, becomes Vref_uV / 3140000.
			 */
			*val = ret;
			*val2 = 3140000;
			return IIO_VAL_FRACTIONAL;
		case IIO_VOLTAGE:
			*val = ret / 1000;
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
			/*
			 * 0 °C == 273.15 K must map to raw + offset such that
			 * (raw + offset) * scale == 0 m°C, i.e.
			 *   offset = -273150 / scale
			 *          = -273150 * 3140000 / Vref_uV
			 * Computed in 64-bit to avoid overflow.
			 */
			*val = div_s64(-273150LL * 3140000, ret);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_SAMP_FREQ:
		/*
		 * Only advertised on the voltage channels of parts with a speed
		 * mode; the sampling frequency is a property of the selected 1x/2x
		 * mode, not of an individual conversion.
		 */
		mutex_lock(&ddata->lock);
		*val = ltc2497core_samp_freq_avail[ddata->sped_2x * 2];
		*val2 = ltc2497core_samp_freq_avail[ddata->sped_2x * 2 + 1];
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
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ltc2497core_samp_freq_avail;
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = ARRAY_SIZE(ltc2497core_samp_freq_avail);
		return IIO_AVAIL_LIST;

	default:
		return -EINVAL;
	}
}

static int ltc2497core_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);
	bool sped_2x;
	int i;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		/* Match the (val, val2) pair against the advertised rates. */
		for (i = 0; i < ARRAY_SIZE(ltc2497core_samp_freq_avail); i += 2) {
			if (val == ltc2497core_samp_freq_avail[i] &&
			    val2 == ltc2497core_samp_freq_avail[i + 1])
				break;
		}
		if (i >= ARRAY_SIZE(ltc2497core_samp_freq_avail))
			return -EINVAL;

		sped_2x = i / 2;

		mutex_lock(&ddata->lock);
		ddata->sped_2x = sped_2x;
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
		mutex_unlock(&ddata->lock);

		return 0;

	default:
		return -EINVAL;
	}
}

#define LTC2497_CHAN(_chan, _addr, _ds_name) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_chan), \
	.address = (_addr | (_chan / 2) | ((_chan & 1) ? LTC2497_SIGN : 0)), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.datasheet_name = (_ds_name), \
}

#define LTC2497_CHAN_DIFF(_chan, _addr) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_chan) * 2 + ((_addr) & LTC2497_SIGN ? 1 : 0), \
	.channel2 = (_chan) * 2 + ((_addr) & LTC2497_SIGN ? 0 : 1),\
	.address = (_addr | _chan), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.differential = 1, \
}

#define LTC2497_TEMP_CHANNEL { \
	.type = IIO_TEMP, \
	.address = LTC2497_TEMP_ADDR, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
			      BIT(IIO_CHAN_INFO_SCALE) | \
			      BIT(IIO_CHAN_INFO_OFFSET), \
}

static const struct iio_chan_spec ltc2497core_channel[] = {
	LTC2497_CHAN(0, LTC2497_SGL, "CH0"),
	LTC2497_CHAN(1, LTC2497_SGL, "CH1"),
	LTC2497_CHAN(2, LTC2497_SGL, "CH2"),
	LTC2497_CHAN(3, LTC2497_SGL, "CH3"),
	LTC2497_CHAN(4, LTC2497_SGL, "CH4"),
	LTC2497_CHAN(5, LTC2497_SGL, "CH5"),
	LTC2497_CHAN(6, LTC2497_SGL, "CH6"),
	LTC2497_CHAN(7, LTC2497_SGL, "CH7"),
	LTC2497_CHAN(8, LTC2497_SGL, "CH8"),
	LTC2497_CHAN(9, LTC2497_SGL, "CH9"),
	LTC2497_CHAN(10, LTC2497_SGL, "CH10"),
	LTC2497_CHAN(11, LTC2497_SGL, "CH11"),
	LTC2497_CHAN(12, LTC2497_SGL, "CH12"),
	LTC2497_CHAN(13, LTC2497_SGL, "CH13"),
	LTC2497_CHAN(14, LTC2497_SGL, "CH14"),
	LTC2497_CHAN(15, LTC2497_SGL, "CH15"),
	LTC2497_CHAN_DIFF(0, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(1, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(2, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(3, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(4, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(5, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(6, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(7, LTC2497_DIFF),
	LTC2497_CHAN_DIFF(0, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(1, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(2, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(3, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(4, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(5, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(6, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_CHAN_DIFF(7, LTC2497_DIFF | LTC2497_SIGN),
	LTC2497_TEMP_CHANNEL,
};

static const struct iio_info ltc2497core_info = {
	.read_raw = ltc2497core_read_raw,
	.read_avail = ltc2497core_read_avail,
	.write_raw = ltc2497core_write_raw,
};

int ltc2497core_probe(struct device *dev, struct iio_dev *indio_dev)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);
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
	indio_dev->channels = ltc2497core_channel;
	indio_dev->num_channels = ARRAY_SIZE(ltc2497core_channel);
	/* Only the ltc2499 has a temperature channel; it is the last entry. */
	if (!ddata->chip_info->has_temp)
		indio_dev->num_channels--;

	/*
	 * Parts with a speed mode expose in_voltage_sampling_frequency /
	 * _available on the voltage channels only.  SPD is ignored during a
	 * temperature measurement, so the temperature channel deliberately
	 * carries no SAMP_FREQ attribute.  Patch a private copy of the shared
	 * channel array so parts without a speed mode stay untouched.
	 */
	if (ddata->chip_info->has_speed_mode) {
		struct iio_chan_spec *channels;
		unsigned int i;

		channels = devm_kmemdup(dev, ltc2497core_channel,
					sizeof(ltc2497core_channel), GFP_KERNEL);
		if (!channels)
			return -ENOMEM;

		for (i = 0; i < indio_dev->num_channels; i++) {
			if (channels[i].type != IIO_VOLTAGE)
				continue;
			channels[i].info_mask_shared_by_type |=
				BIT(IIO_CHAN_INFO_SAMP_FREQ);
			channels[i].info_mask_shared_by_type_available |=
				BIT(IIO_CHAN_INFO_SAMP_FREQ);
		}

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

	if (dev->platform_data) {
		struct iio_map *plat_data;

		plat_data = (struct iio_map *)dev->platform_data;

		ret = iio_map_array_register(indio_dev, plat_data);
		if (ret) {
			dev_err(&indio_dev->dev, "iio map err: %d\n", ret);
			goto err_regulator_disable;
		}
	}

	ddata->addr_prev = LTC2497_CONFIG_DEFAULT;
	ddata->time_prev = ktime_get();
	/* Power-on default mode is 1x; a conversion is already in flight. */
	ddata->conv_time_prev = LTC2497_CONV_TIME_1X_MS;

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
EXPORT_SYMBOL_NS(ltc2497core_probe, LTC2497);

void ltc2497core_remove(struct iio_dev *indio_dev)
{
	struct ltc2497core_driverdata *ddata = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	iio_map_array_unregister(indio_dev);

	regulator_disable(ddata->ref);
}
EXPORT_SYMBOL_NS(ltc2497core_remove, LTC2497);

MODULE_DESCRIPTION("common code for LTC2496/LTC2497 drivers");
MODULE_LICENSE("GPL v2");
