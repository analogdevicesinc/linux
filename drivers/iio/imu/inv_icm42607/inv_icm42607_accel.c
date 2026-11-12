// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2026 InvenSense, Inc.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/device/devres.h>
#include <linux/err.h>
#include <linux/iio/iio.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/units.h>

#include "inv_icm42607.h"
#include "inv_icm42607_temp.h"

#define INV_ICM42607_ACCEL_CHAN(_modifier, _index)				\
{										\
	.type = IIO_ACCEL,							\
	.modified = 1,								\
	.channel2 = _modifier,							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |				\
			      BIT(IIO_CHAN_INFO_CALIBBIAS),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SCALE) |	\
					      BIT(IIO_CHAN_INFO_CALIBBIAS),	\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = _index,							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = 16,							\
		.storagebits = 16,						\
		.endianness = IIO_BE,						\
	},									\
	.ext_info = inv_icm42607_accel_ext_infos,				\
}

enum inv_icm42607_accel_scan {
	INV_ICM42607_ACCEL_SCAN_X,
	INV_ICM42607_ACCEL_SCAN_Y,
	INV_ICM42607_ACCEL_SCAN_Z,
	INV_ICM42607_ACCEL_SCAN_TEMP,
};

static const struct iio_chan_spec_ext_info inv_icm42607_accel_ext_infos[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_ALL, inv_icm42607_get_mount_matrix),
	{ }
};

static const struct iio_chan_spec inv_icm42607_accel_channels[] = {
	INV_ICM42607_ACCEL_CHAN(IIO_MOD_X, INV_ICM42607_ACCEL_SCAN_X),
	INV_ICM42607_ACCEL_CHAN(IIO_MOD_Y, INV_ICM42607_ACCEL_SCAN_Y),
	INV_ICM42607_ACCEL_CHAN(IIO_MOD_Z, INV_ICM42607_ACCEL_SCAN_Z),
	INV_ICM42607_TEMP_CHAN(INV_ICM42607_ACCEL_SCAN_TEMP),
};

/*
 * Calibration bias values, IIO range format int + micro.
 * Value is limited to +/-1g coded on 12 bits signed. Step is 0.5mg.
 */
static const int inv_icm42607_accel_calibbias[] = {
	-10, 42010, /* Min : -2^11 * 0.0005 * 9.80665	   = -10.042010 m/s²	*/
	  0,  4903, /* Step: 0.0005 * 9.80665		   = 0.004903 m/s²	*/
	 10, 37106, /* Max : (2^11 - 1) * 0.0005 * 9.80665 = 10.037106 m/s²	*/
};

static const int inv_icm42607_accel_scale_nano[][2] = {
	[INV_ICM42607_ACCEL_FS_16G] = { 0, 4788403 },
	[INV_ICM42607_ACCEL_FS_8G] = { 0, 2394202 },
	[INV_ICM42607_ACCEL_FS_4G] = { 0, 1197101 },
	[INV_ICM42607_ACCEL_FS_2G] = { 0, 598550 },
};

static int inv_icm42607_accel_read_scale(struct iio_dev *indio_dev,
					 int *val, int *val2)
{
	struct inv_icm42607_state *st = iio_device_get_drvdata(indio_dev);
	unsigned int idx;

	guard(mutex)(&st->lock);

	idx = st->conf.accel.fs;

	*val = inv_icm42607_accel_scale_nano[idx][0];
	*val2 = inv_icm42607_accel_scale_nano[idx][1];
	return IIO_VAL_INT_PLUS_NANO;
}

static int inv_icm42607_accel_write_scale(struct iio_dev *indio_dev,
					  int val, int val2)
{
	struct inv_icm42607_sensor_conf conf = INV_ICM42607_SENSOR_CONF_INIT;
	struct inv_icm42607_state *st = iio_device_get_drvdata(indio_dev);
	size_t scales_len = ARRAY_SIZE(inv_icm42607_accel_scale_nano);
	struct device *dev = regmap_get_device(st->map);
	unsigned int idx;
	int ret;

	for (idx = 0; idx < scales_len; idx++) {
		if (val == inv_icm42607_accel_scale_nano[idx][0] &&
		    val2 == inv_icm42607_accel_scale_nano[idx][1])
			break;
	}
	if (idx == scales_len)
		return -EINVAL;

	conf.fs = idx;

	PM_RUNTIME_ACQUIRE_AUTOSUSPEND(dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	return inv_icm42607_set_sensor_conf(st, &conf, IIO_ACCEL);
}

/* IIO format int + micro , values 0-4 reserved. */
static const int inv_icm42607_accel_odr[][2] = {
	[INV_ICM42607_ODR_1600HZ] = { 1600, 0 },
	[INV_ICM42607_ODR_800HZ] = { 800, 0 },
	[INV_ICM42607_ODR_400HZ] = { 400, 0 },
	[INV_ICM42607_ODR_200HZ] = { 200, 0 },
	[INV_ICM42607_ODR_100HZ] = { 100, 0 },
	[INV_ICM42607_ODR_50HZ] = { 50, 0 },
	[INV_ICM42607_ODR_25HZ] = { 25, 0 },
	[INV_ICM42607_ODR_12_5HZ] = { 12, 500000 },
	[INV_ICM42607_ODR_6_25HZ_LP] = { 6, 250000 },
	[INV_ICM42607_ODR_3_125HZ_LP] = { 3, 125000 },
	[INV_ICM42607_ODR_1_5625HZ_LP] = { 1, 562500 },
};

static int inv_icm42607_accel_read_odr(struct inv_icm42607_state *st,
				       int *val, int *val2)
{
	unsigned int odr;
	unsigned int i;

	guard(mutex)(&st->lock);

	odr = st->conf.accel.odr;

	for (i = INV_ICM42607_ODR_1600HZ; i < ARRAY_SIZE(inv_icm42607_accel_odr); i++) {
		if (i == odr)
			break;
	}
	if (i == ARRAY_SIZE(inv_icm42607_accel_odr))
		return -EINVAL;

	*val = inv_icm42607_accel_odr[i][0];
	*val2 = inv_icm42607_accel_odr[i][1];

	return IIO_VAL_INT_PLUS_MICRO;
}

static int inv_icm42607_accel_write_odr(struct iio_dev *indio_dev,
					int val, int val2)
{
	struct inv_icm42607_sensor_conf conf = INV_ICM42607_SENSOR_CONF_INIT;
	struct inv_icm42607_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	unsigned int idx;
	int ret;

	for (idx = INV_ICM42607_ODR_1600HZ;
	     idx < ARRAY_SIZE(inv_icm42607_accel_odr); idx++) {
		if (val == inv_icm42607_accel_odr[idx][0] &&
		    val2 == inv_icm42607_accel_odr[idx][1])
			break;
	}
	if (idx == ARRAY_SIZE(inv_icm42607_accel_odr))
		return -EINVAL;

	conf.odr = idx;

	PM_RUNTIME_ACQUIRE_AUTOSUSPEND(dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	return inv_icm42607_set_sensor_conf(st, &conf, IIO_ACCEL);
}

static int inv_icm42607_accel_read_offset(struct inv_icm42607_state *st,
		struct iio_chan_spec const *chan, int *val, int *val2)
{
	struct device *dev = regmap_get_device(st->map);
	unsigned int msb_reg, lsb_reg, nib, byte;
	int offset;
	s64 val64;
	s32 bias;
	int ret;
	u32 raw;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	switch (chan->channel2) {
	case IIO_MOD_X:
		msb_reg = INV_ICM42607_REG_OFFSET_USER4;
		lsb_reg = INV_ICM42607_REG_OFFSET_USER5;
		break;
	case IIO_MOD_Y:
		msb_reg = INV_ICM42607_REG_OFFSET_USER7;
		lsb_reg = INV_ICM42607_REG_OFFSET_USER6;
		break;
	case IIO_MOD_Z:
		msb_reg = INV_ICM42607_REG_OFFSET_USER7;
		lsb_reg = INV_ICM42607_REG_OFFSET_USER8;
		break;
	default:
		return -EINVAL;
	}

	PM_RUNTIME_ACQUIRE_AUTOSUSPEND(dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	ret = regmap_read(st->map, msb_reg, &nib);
	if (ret)
		return ret;

	ret = regmap_read(st->map, lsb_reg, &byte);
	if (ret)
		return ret;

	/* 12 bits signed value */
	if (chan->channel2 == IIO_MOD_Y)
		raw = FIELD_PREP(INV_ICM42607_OFFSET_MSB,
				 FIELD_GET(INV_ICM42607_OFFSET_USER_NIBBLE_LO, nib));
	else
		raw = FIELD_PREP(INV_ICM42607_OFFSET_MSB,
				 FIELD_GET(INV_ICM42607_OFFSET_USER_NIBBLE_HI, nib));

	raw |= FIELD_PREP(INV_ICM42607_OFFSET_LSB, byte);
	offset = sign_extend32(raw, 11);

	/*
	 * Convert raw offset to g then to m/s²
	 * 12 bits signed raw step 0.5mg to g: 5 / 10000
	 * g to m/s²: 9.806650
	 * Result in micro (1000000)
	 * (offset * 5 * 9.806650 * 1000000) / 10000
	 */
	val64 = (s64)offset * 5LL * 9806650LL;
	/* For rounding, add + or - divisor (10000) divided by 2 */
	bias = DIV_S64_ROUND_CLOSEST(val64, 10000L);
	*val = bias / (long)MEGA;
	*val2 = bias % (long)MEGA;

	return IIO_VAL_INT_PLUS_MICRO;
}

static int inv_icm42607_accel_write_offset(struct iio_dev *indio_dev,
					   struct iio_chan_spec const *chan,
					   int val, int val2)
{
	struct inv_icm42607_state *st = iio_device_get_drvdata(indio_dev);
	struct device *dev = regmap_get_device(st->map);
	s32 min, max;
	u8 msb, lsb;
	s16 offset;
	s64 val64;
	int ret;

	if (chan->type != IIO_ACCEL)
		return -EINVAL;

	/* inv_icm42607_accel_calibbias: min - step - max in micro */
	min = inv_icm42607_accel_calibbias[0] * (long)MEGA -
	      inv_icm42607_accel_calibbias[1];
	max = inv_icm42607_accel_calibbias[4] * (long)MEGA +
	      inv_icm42607_accel_calibbias[5];

	val64 = val * (s64)MEGA;
	if (val >= 0)
		val64 += val2;
	else
		val64 -= val2;

	if (val64 < min || val64 > max)
		return -EINVAL;

	/*
	 * Convert m/s² to g then to raw value
	 * m/s² to g: 1 / 9.806650
	 * g to raw 12 bits signed, step 0.5mg: 10000 / 5
	 * val in micro (1000000)
	 * val * 10000 / (9.806650 * 1000000 * 5)
	 */
	val64 *= 10000LL;
	offset = DIV_S64_ROUND_CLOSEST(val64, 9806650 * 5);

	/* Value is limited to 12 bits signed, return -EINVAL if out of range */
	if (offset < -2048 || offset > 2047)
		return -EINVAL;

	msb = FIELD_GET(INV_ICM42607_OFFSET_MSB, offset);
	lsb = FIELD_GET(INV_ICM42607_OFFSET_LSB, offset);

	PM_RUNTIME_ACQUIRE_AUTOSUSPEND(dev, pm);
	ret = PM_RUNTIME_ACQUIRE_ERR(&pm);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (chan->channel2) {
	case IIO_MOD_X:
		/* OFFSET_USER4 upper nibble is shared. */
		ret = regmap_update_bits(st->map, INV_ICM42607_REG_OFFSET_USER4,
					 INV_ICM42607_OFFSET_USER_NIBBLE_HI,
					 FIELD_PREP(INV_ICM42607_OFFSET_USER_NIBBLE_HI, msb));
		if (ret)
			return ret;

		return regmap_write(st->map, INV_ICM42607_REG_OFFSET_USER5, lsb);
	case IIO_MOD_Y:
		/* OFFSET_USER7 lower nibble is shared. */
		ret = regmap_update_bits(st->map, INV_ICM42607_REG_OFFSET_USER7,
					 INV_ICM42607_OFFSET_USER_NIBBLE_LO,
					 FIELD_PREP(INV_ICM42607_OFFSET_USER_NIBBLE_LO, msb));
		if (ret)
			return ret;

		return regmap_write(st->map, INV_ICM42607_REG_OFFSET_USER6, lsb);
	case IIO_MOD_Z:
		/* OFFSET_USER7 upper nibble is shared. */
		ret = regmap_update_bits(st->map, INV_ICM42607_REG_OFFSET_USER7,
					 INV_ICM42607_OFFSET_USER_NIBBLE_HI,
					 FIELD_PREP(INV_ICM42607_OFFSET_USER_NIBBLE_HI, msb));
		if (ret)
			return ret;

		return regmap_write(st->map, INV_ICM42607_REG_OFFSET_USER8, lsb);
	default:
		return -EINVAL;
	}
}

static int inv_icm42607_accel_read_raw(struct iio_dev *indio_dev,
				       struct iio_chan_spec const *chan,
				       int *val, int *val2, long mask)
{
	struct inv_icm42607_state *st = iio_device_get_drvdata(indio_dev);
	s16 data;
	int ret;

	switch (chan->type) {
	case IIO_ACCEL:
		break;
	case IIO_TEMP:
		if (mask != IIO_CHAN_INFO_SAMP_FREQ)
			return inv_icm42607_temp_read_raw(indio_dev, chan,
							  val, val2, mask);
		break;
	default:
		return -EINVAL;
	}

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = inv_icm42607_read_sensor(indio_dev, chan, &data);
		if (ret)
			return ret;
		*val = data;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return inv_icm42607_accel_read_scale(indio_dev, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return inv_icm42607_accel_read_odr(st, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return inv_icm42607_accel_read_offset(st, chan, val, val2);
	default:
		return -EINVAL;
	}
}

static int inv_icm42607_accel_read_avail(struct iio_dev *indio_dev,
					 struct iio_chan_spec const *chan,
					 const int **vals,
					 int *type, int *length, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		*vals = (const int *)inv_icm42607_accel_scale_nano;
		*type = IIO_VAL_INT_PLUS_NANO;
		*length = ARRAY_SIZE(inv_icm42607_accel_scale_nano) * 2;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = (const int *)inv_icm42607_accel_odr[INV_ICM42607_ODR_1600HZ];
		*type = IIO_VAL_INT_PLUS_MICRO;
		*length = (ARRAY_SIZE(inv_icm42607_accel_odr) -
			   INV_ICM42607_ODR_1600HZ) * 2;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		*vals = inv_icm42607_accel_calibbias;
		*type = IIO_VAL_INT_PLUS_MICRO;
		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int inv_icm42607_accel_write_raw(struct iio_dev *indio_dev,
					struct iio_chan_spec const *chan,
					int val, int val2, long mask)
{
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		ret = inv_icm42607_accel_write_scale(indio_dev, val, val2);
		return ret;
	case IIO_CHAN_INFO_SAMP_FREQ:
		return inv_icm42607_accel_write_odr(indio_dev, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return inv_icm42607_accel_write_offset(indio_dev, chan, val, val2);
	default:
		return -EINVAL;
	}
}

static int inv_icm42607_accel_write_raw_get_fmt(struct iio_dev *indio_dev,
						struct iio_chan_spec const *chan,
						long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (chan->type != IIO_ACCEL)
			return -EINVAL;
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_CALIBBIAS:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info inv_icm42607_accel_info = {
	.read_raw = inv_icm42607_accel_read_raw,
	.read_avail = inv_icm42607_accel_read_avail,
	.write_raw = inv_icm42607_accel_write_raw,
	.write_raw_get_fmt = inv_icm42607_accel_write_raw_get_fmt,
};

struct iio_dev *inv_icm42607_accel_init(struct inv_icm42607_state *st)
{
	struct device *dev = regmap_get_device(st->map);
	struct inv_icm42607_sensor_state *accel_st;
	struct iio_dev *indio_dev;
	const char *name;
	int ret;

	name = devm_kasprintf(dev, GFP_KERNEL, "%s-accel", st->hw->name);
	if (!name)
		return ERR_PTR(-ENOMEM);

	indio_dev = devm_iio_device_alloc(dev, sizeof(*accel_st));
	if (!indio_dev)
		return ERR_PTR(-ENOMEM);

	accel_st = iio_priv(indio_dev);
	accel_st->power_mode = INV_ICM42607_SENSOR_MODE_LOW_NOISE;
	accel_st->filter = INV_ICM42607_FILTER_BW_73HZ;

	indio_dev->name = name;
	indio_dev->info = &inv_icm42607_accel_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = inv_icm42607_accel_channels;
	indio_dev->num_channels = ARRAY_SIZE(inv_icm42607_accel_channels);
	iio_device_set_drvdata(indio_dev, st);

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ERR_PTR(ret);

	return indio_dev;
}
