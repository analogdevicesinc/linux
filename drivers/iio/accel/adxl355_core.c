// SPDX-License-Identifier: GPL-2.0-only
/*
 * ADXL355 3-Axis Digital Accelerometer IIO core driver
 *
 * Copyright (c) 2021 Puranjay Mohan <puranjay12@gmail.com>
 *
 * Datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/adxl354_adxl355.pdf
 */

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/limits.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include "adxl355.h"

const struct regmap_range adxl355_read_reg_range[] = {
	regmap_reg_range(ADXL355_DEVID_AD, ADXL355_FIFO_DATA),
	regmap_reg_range(ADXL355_OFFSET_X_H, ADXL355_SELF_TEST)
};
EXPORT_SYMBOL_GPL(adxl355_read_reg_range);

const struct regmap_access_table adxl355_readable_regs_tbl = {
	.yes_ranges = adxl355_read_reg_range,
	.n_yes_ranges = ARRAY_SIZE(adxl355_read_reg_range),
};
EXPORT_SYMBOL_GPL(adxl355_readable_regs_tbl);

const struct regmap_range adxl355_write_reg_range[] = {
	regmap_reg_range(ADXL355_OFFSET_X_H, ADXL355_RESET)
};
EXPORT_SYMBOL_GPL(adxl355_write_reg_range);

const struct regmap_access_table adxl355_writeable_regs_tbl = {
	.yes_ranges = adxl355_write_reg_range,
	.n_yes_ranges = ARRAY_SIZE(adxl355_write_reg_range),
};
EXPORT_SYMBOL_GPL(adxl355_writeable_regs_tbl);

enum adxl355_op_mode {
	ADXL355_MEASUREMENT,
	ADXL355_STANDBY,
	ADXL355_TEMP_OFF
};

enum adxl355_odr {
	ADXL355_ODR_4000HZ,
	ADXL355_ODR_2000HZ,
	ADXL355_ODR_1000HZ,
	ADXL355_ODR_500HZ,
	ADXL355_ODR_250HZ,
	ADXL355_ODR_125HZ,
	ADXL355_ODR_62_5HZ,
	ADXL355_ODR_31_25HZ,
	ADXL355_ODR_15_625HZ,
	ADXL355_ODR_7_813HZ,
	ADXL355_ODR_3_906HZ
};

enum adxl355_hpf_3db {
	ADXL355_HPF_OFF,
	ADXL355_HPF_24_7,
	ADXL355_HPF_6_2084,
	ADXL355_HPF_1_5545,
	ADXL355_HPF_0_3862,
	ADXL355_HPF_0_0954,
	ADXL355_HPF_0_0238
};

static const int adxl355_odr_table[][2] = {
	[0] = {4000, 0},
	[1] = {2000, 0},
	[2] = {1000, 0},
	[3] = {500, 0},
	[4] = {250, 0},
	[5] = {125, 0},
	[6] = {62, 500000},
	[7] = {31, 250000},
	[8] = {15, 625000},
	[9] = {7, 813000},
	[10] = {3, 906000}
};

static int adxl355_hpf_3db_table[7][2] = {0};

static const int adxl355_hpf_3db_multipliers[] = {
	0,
	247000,
	62084,
	15545,
	3862,
	954,
	238
};

struct adxl355_data {
	struct regmap *regmap;
	struct device *dev;
	struct mutex lock; /* lock to protect op_mode */
	enum adxl355_op_mode op_mode;
	enum adxl355_odr odr;
	enum adxl355_hpf_3db hpf_3db;
	int x_calibbias;
	int y_calibbias;
	int z_calibbias;
};

static int adxl355_set_op_mode(struct adxl355_data *data,
			       enum adxl355_op_mode op_mode)
{
	int ret;

	if (data->op_mode == op_mode)
		return 0;

	ret = regmap_update_bits(data->regmap, ADXL355_POWER_CTL,
				 ADXL355_POWER_CTL_MODE_MSK, op_mode);
	if (ret < 0)
		return ret;

	data->op_mode = op_mode;

	return ret;
}

static void adxl355_fill_3db_frequency_table(struct adxl355_data *data)
{
	int i;
	u64 rem;
	u64 div;
	u32 multiplier;
	u64 odr = mul_u64_u32_shr(adxl355_odr_table[data->odr][0], 1000000, 0) +
					adxl355_odr_table[data->odr][1];

	for (i = 0; i < ARRAY_SIZE(adxl355_hpf_3db_multipliers); i++) {
		multiplier = adxl355_hpf_3db_multipliers[i];
		div = div64_u64_rem(mul_u64_u32_shr(odr, multiplier, 0),
				    100000000000000UL, &rem);

		adxl355_hpf_3db_table[i][0] = div;
		adxl355_hpf_3db_table[i][1] = div_u64(rem, 100000000);
	}
}

static int adxl355_setup(struct adxl355_data *data)
{
	unsigned int regval;
	int ret;

	ret = regmap_read(data->regmap, ADXL355_DEVID_AD, &regval);
	if (ret < 0)
		return ret;

	if (regval != ADXL355_DEVID_AD_VAL) {
		dev_err(data->dev, "Invalid ADI ID 0x%02x\n", regval);
		return -ENODEV;
	}

	ret = regmap_read(data->regmap, ADXL355_DEVID_MST, &regval);
	if (ret < 0)
		return ret;

	if (regval != ADXL355_DEVID_MST_VAL) {
		dev_err(data->dev, "Invalid MEMS ID 0x%02x\n", regval);
		return -ENODEV;
	}

	ret = regmap_read(data->regmap, ADXL355_PARTID, &regval);
	if (ret < 0)
		return ret;

	if (regval != ADXL355_PARTID_VAL) {
		dev_err(data->dev, "Invalid DEV ID 0x%02x\n", regval);
		return -ENODEV;
	}

	/*
	 * Perform a software reset to make sure the device is in a consistent
	 * state after start up.
	 */
	ret = regmap_write(data->regmap, ADXL355_RESET, ADXL355_RESET_CODE);
	if (ret < 0)
		return ret;

	adxl355_fill_3db_frequency_table(data);

	return adxl355_set_op_mode(data, ADXL355_MEASUREMENT);
}

static int adxl355_get_temp_data(struct adxl355_data *data,
				 u8 addr, __be16 *out)
{
	return regmap_bulk_read(data->regmap, addr, out, sizeof(*out));
}

static int adxl355_read_axis(struct adxl355_data *data, u8 addr)
{
	__be32 regval;
	int ret;

	ret = regmap_bulk_read(data->regmap, addr, &regval, 3);
	if (ret < 0)
		return ret;

	return be32_to_cpu(regval) >> 8;
}

static int adxl355_find_match(const int (*freq_tbl)[2], const int n,
			      const int val, const int val2)
{
	int i;

	for (i = 0; i < n; i++) {
		if (freq_tbl[i][0] == val && freq_tbl[i][1] == val2)
			return i;
	}

	return -EINVAL;
}

static int adxl355_set_odr(struct adxl355_data *data,
			   enum adxl355_odr odr)
{
	int ret = 0;

	mutex_lock(&data->lock);

	if (data->odr == odr)
		goto out_unlock;

	ret = adxl355_set_op_mode(data, ADXL355_STANDBY);
	if (ret < 0)
		goto out_unlock;

	ret = regmap_update_bits(data->regmap, ADXL355_FILTER,
				 ADXL355_FILTER_ODR_MSK,
				 ADXL355_FILTER_ODR_MODE(odr));
	if (!ret) {
		data->odr = odr;
		adxl355_fill_3db_frequency_table(data);
	}

out_unlock:
	ret = adxl355_set_op_mode(data, ADXL355_MEASUREMENT);
	mutex_unlock(&data->lock);
	return ret;
}

static int adxl355_set_hpf_3db(struct adxl355_data *data,
			       enum adxl355_hpf_3db hpf)
{
	int ret = 0;

	mutex_lock(&data->lock);

	if (data->hpf_3db == hpf)
		goto out_unlock;

	ret = adxl355_set_op_mode(data, ADXL355_STANDBY);
	if (ret < 0)
		goto out_unlock;

	ret = regmap_update_bits(data->regmap, ADXL355_FILTER,
				 ADXL355_FILTER_HPF_MSK,
				 ADXL355_FILTER_HPF_MODE(hpf));
	if (!ret)
		data->hpf_3db = hpf;

out_unlock:
	ret = adxl355_set_op_mode(data, ADXL355_MEASUREMENT);
	mutex_unlock(&data->lock);
	return ret;
}

static int adxl355_set_calibbias(struct adxl355_data *data,
				 int scan_index, int calibbias)
{
	int ret = 0;
	__be16 reg = cpu_to_be16(calibbias);

	mutex_lock(&data->lock);

	ret = adxl355_set_op_mode(data, ADXL355_STANDBY);
	if (ret < 0)
		goto out_unlock;

	switch (scan_index) {
	case 0:
		ret = regmap_bulk_write(data->regmap, ADXL355_OFFSET_X_H,
					&reg, 2);
		if (ret < 0)
			goto out_unlock;
		data->x_calibbias = calibbias;
		break;
	case 1:
		ret = regmap_bulk_write(data->regmap, ADXL355_OFFSET_Y_H,
					&reg, 2);
		if (ret < 0)
			goto out_unlock;
		data->y_calibbias = calibbias;
		break;
	case 2:
		ret = regmap_bulk_write(data->regmap, ADXL355_OFFSET_Z_H,
					&reg, 2);
		if (ret < 0)
			goto out_unlock;
		data->z_calibbias = calibbias;
		break;
	default:
		ret = -EINVAL;
		break;
	}

out_unlock:
	ret = adxl355_set_op_mode(data, ADXL355_MEASUREMENT);
	mutex_unlock(&data->lock);
	return ret;
}

static int adxl355_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct adxl355_data *data = iio_priv(indio_dev);
	int ret;
	__be16 out;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_TEMP:
			ret = adxl355_get_temp_data(data, chan->address, &out);
			if (ret < 0)
				break;
			*val = be16_to_cpu(out);
			ret = IIO_VAL_INT;
			break;

		case IIO_ACCEL:
			ret = adxl355_read_axis(data, chan->address);
			if (ret < 0)
				break;
			*val = sign_extend32(ret >> (chan->scan_type.shift),
					     chan->scan_type.realbits - 1);
			ret = IIO_VAL_INT;
			break;

		default:
			ret = -EINVAL;
			break;
		}

		return ret;
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_TEMP:
			*val = TEMP_SCALE_VAL;
			*val2 = TEMP_SCALE_VAL2;
			return IIO_VAL_INT_PLUS_MICRO;
		case IIO_ACCEL:
			*val = 0;
			*val2 = ADXL355_NSCALE;
			return IIO_VAL_INT_PLUS_NANO;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		*val = TEMP_OFFSET_VAL;
		*val2 = TEMP_OFFSET_VAL2;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (chan->scan_index == 0)
			*val = data->x_calibbias;
		else if (chan->scan_index == 1)
			*val = data->y_calibbias;
		else
			*val = data->z_calibbias;
		*val = sign_extend32(*val, 15);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adxl355_odr_table[data->odr][0];
		*val2 = adxl355_odr_table[data->odr][1];
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		*val = adxl355_hpf_3db_table[data->hpf_3db][0];
		*val2 = adxl355_hpf_3db_table[data->hpf_3db][1];
		return IIO_VAL_INT_PLUS_MICRO;
	}
	return -EINVAL;
}

static int adxl355_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct adxl355_data *data = iio_priv(indio_dev);
	int odr_idx, hpf_idx, calibbias, ret;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		odr_idx = adxl355_find_match(adxl355_odr_table,
					     ARRAY_SIZE(adxl355_odr_table),
					     val, val2);
		if (odr_idx < 0)
			return odr_idx;

		return adxl355_set_odr(data, odr_idx);
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		hpf_idx = adxl355_find_match(adxl355_hpf_3db_table,
					     ARRAY_SIZE(adxl355_hpf_3db_table),
					     val, val2);
		if (hpf_idx < 0)
			return hpf_idx;

		return adxl355_set_hpf_3db(data, hpf_idx);
	case IIO_CHAN_INFO_CALIBBIAS:
		calibbias = clamp_t(int, val, S16_MIN, S16_MAX);
		ret = adxl355_set_calibbias(data, chan->scan_index, calibbias);

		return ret;
	default:
		return -EINVAL;
	}
}

static int adxl355_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length,
			      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = (const int *)adxl355_odr_table;
		*type = IIO_VAL_INT_PLUS_MICRO;
		/* Values are stored in a 2D matrix */
		*length = ARRAY_SIZE(adxl355_odr_table) * 2;

		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		*vals = (const int *)adxl355_hpf_3db_table;
		*type = IIO_VAL_INT_PLUS_MICRO;
		/* Values are stored in a 2D matrix */
		*length = ARRAY_SIZE(adxl355_hpf_3db_table) * 2;

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static const unsigned long adxl355_avail_scan_masks[] = {
	GENMASK(3, 0),
	0
};

static const struct iio_info adxl355_info = {
	.read_raw	= adxl355_read_raw,
	.write_raw	= adxl355_write_raw,
	.read_avail	= &adxl355_read_avail
};

#define ADXL355_ACCEL_CHANNEL(index, reg, axis) {			\
	.type = IIO_ACCEL,						\
	.address = reg,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##axis,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |			\
			      BIT(IIO_CHAN_INFO_CALIBBIAS),		\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ) |	\
		BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY),	\
	.info_mask_shared_by_type_available =				\
		BIT(IIO_CHAN_INFO_SAMP_FREQ) |				\
		BIT(IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY),	\
	.scan_index = index,						\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 20,						\
		.storagebits = 32,					\
		.shift = 4,						\
		.endianness = IIO_BE,					\
	}								\
}

static const struct iio_chan_spec adxl355_channels[] = {
	ADXL355_ACCEL_CHANNEL(0, ADXL355_XDATA3, X),
	ADXL355_ACCEL_CHANNEL(1, ADXL355_YDATA3, Y),
	ADXL355_ACCEL_CHANNEL(2, ADXL355_ZDATA3, Z),
	{
		.type = IIO_TEMP,
		.address = ADXL355_TEMP2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE) |
				      BIT(IIO_CHAN_INFO_OFFSET),
		.scan_index = 3,
		.scan_type = {
			.sign = 's',
			.realbits = 12,
			.storagebits = 16,
			.endianness = IIO_BE,
		},
	}
};

int adxl355_core_probe(struct device *dev, struct regmap *regmap,
		       const char *name)
{
	struct adxl355_data *data;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);
	data->regmap = regmap;
	data->dev = dev;
	data->op_mode = ADXL355_STANDBY;
	mutex_init(&data->lock);

	indio_dev->dev.parent = dev;
	indio_dev->name = name;
	indio_dev->info = &adxl355_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = adxl355_channels;
	indio_dev->num_channels = ARRAY_SIZE(adxl355_channels);
	indio_dev->available_scan_masks = adxl355_avail_scan_masks;

	ret = adxl355_setup(data);
	if (ret < 0) {
		dev_err(dev, "ADXL355 setup failed\n");
		return ret;
	}

	return devm_iio_device_register(dev, indio_dev);
}
EXPORT_SYMBOL_GPL(adxl355_core_probe);

MODULE_AUTHOR("Puranjay Mohan <puranjay12@gmail.com>");
MODULE_DESCRIPTION("ADXL355 3-Axis Digital Accelerometer core driver");
MODULE_LICENSE("GPL v2");
