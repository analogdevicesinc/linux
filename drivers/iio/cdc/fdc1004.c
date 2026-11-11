// SPDX-License-Identifier: GPL-2.0+
/*
 * Texas Instruments FDC1004 capacitive sensor driver
 *
 * Copyright (c) 2026 Victor Chernyakin <chernyakin.victor.j@outlook.com>
 *
 * Datasheet: https://www.ti.com/lit/gpn/FDC1004
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/types.h>

#include <linux/iio/iio.h>

#define FDC1004_REG_MEASn_MSB(n)	(2 * (n))
#define FDC1004_REG_MEASn_LSB(n)	(2 * (n) + 1)
#define FDC1004_REG_CONF_MEASn(n)	(0x08 + (n))
#define FDC1004_REG_FDC_CONF		0x0C
#define FDC1004_REG_OFFSET_CAL_CINn(n)	(0x0D + (n))
#define FDC1004_REG_GAIN_CAL_CINn(n)	(0x11 + (n))

#define FDC1004_CONF_MEAS_CHA_MASK	GENMASK(15, 13)
#define FDC1004_CONF_MEAS_CHB_MASK	GENMASK(12, 10)
#define FDC1004_CONF_MEAS_CHB_CAPDAC	0b100
#define FDC1004_CONF_MEAS_CAPDAC_MASK	GENMASK(9, 5)

#define FDC1004_FDC_CONF_DONE_n_MASK(n)	BIT(3 - (n))
#define FDC1004_FDC_CONF_MEAS_n_MASK(n)	BIT(7 - (n))
#define FDC1004_FDC_CONF_RATE_MASK	GENMASK(11, 10)
#define FDC1004_FDC_CONF_RST_MASK	BIT(15)

static const int fdc1004_samp_freqs[] = { 100, 200, 400 };

/*
 * MEASn_{MSB,LSB}: 24 bits, signed. Format: IIO_VAL_DECIMAL64_YOCTO.
 * 1 step = 1/(2^19) pF = 1/(1000 * 2^19) nF = 0.000000001907348632812500 nF.
 */
static const s64 fdc1004_scale = 1907348632812500;

/*
 * CAPDAC field: 5 bits, unsigned. Format: IIO_VAL_INT.
 * 1 step = 3.125 pF = 3.125 / 1/(2^19) raw value steps = 1638400 raw value steps.
 */
static const int fdc1004_offset[] = {
	0x00 * 1638400, /* min */
	0x01 * 1638400, /* step */
	0x1F * 1638400, /* max */
};

/*
 * OFFSET_CAL_CINn: 16 bits, signed. Format: IIO_VAL_DECIMAL64_FEMTO.
 * 1 step = 1/(2^11) pF = 1/(1000 * 2^11) nF = 0.000000488281250 nF.
 * Negative values are used to cancel out any parasitic capacitance
 * of the sensor's external circuitry.
 */
static const s64 fdc1004_calibbias[] = {
	-0x8000 * 488281250LL, /* min */
	+0x0001 * 488281250LL, /* step */
	+0x7FFF * 488281250LL, /* max */
};

/*
 * GAIN_CAL_CINn: 16 bits, unsigned. Format: IIO_VAL_DECIMAL64_FEMTO.
 * 1 step = 1/(2^14) pF/pF = 0.000061035156250 pF/pF.
 */
static const s64 fdc1004_calibscale[] = {
	0x0000 * 61035156250LL, /* min */
	0x0001 * 61035156250LL, /* step */
	0xFFFF * 61035156250LL, /* max */
};

static const struct regmap_range fdc1004_readable_reg_ranges[] = {
	regmap_reg_range(FDC1004_REG_MEASn_MSB(0), FDC1004_REG_GAIN_CAL_CINn(3)),
};

static const struct regmap_access_table fdc1004_readable_reg_table = {
	.yes_ranges = fdc1004_readable_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(fdc1004_readable_reg_ranges),
};

static const struct regmap_range fdc1004_writeable_reg_ranges[] = {
	regmap_reg_range(FDC1004_REG_CONF_MEASn(0), FDC1004_REG_GAIN_CAL_CINn(3)),
};

static const struct regmap_access_table fdc1004_writeable_reg_table = {
	.yes_ranges = fdc1004_writeable_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(fdc1004_writeable_reg_ranges),
};

static const struct regmap_range fdc1004_volatile_reg_ranges[] = {
	regmap_reg_range(FDC1004_REG_MEASn_MSB(0), FDC1004_REG_MEASn_LSB(3)),
	regmap_reg_range(FDC1004_REG_FDC_CONF, FDC1004_REG_FDC_CONF),
};

static const struct regmap_access_table fdc1004_volatile_reg_table = {
	.yes_ranges = fdc1004_volatile_reg_ranges,
	.n_yes_ranges = ARRAY_SIZE(fdc1004_volatile_reg_ranges),
};

static const struct regmap_config fdc1004_regmap_config = {
	.name = "fdc1004",
	.reg_bits = 8,
	.val_bits = 16,
	/*
	 * The sensor's pointer register doesn't autoincrement, so bulk reads
	 * must be split up into individual reads.
	 */
	.use_single_read = true,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.max_register = FDC1004_REG_GAIN_CAL_CINn(3),
	.rd_table = &fdc1004_readable_reg_table,
	.wr_table = &fdc1004_writeable_reg_table,
	.volatile_table = &fdc1004_volatile_reg_table,
	.cache_type = REGCACHE_MAPLE,
};

struct fdc1004_data {
	struct regmap *regmap;

	/*
	 * Serializes accesses to the chip's measurement and config
	 * registers, and to the other struct members below.
	 */
	struct mutex lock;

	u8 offsets[4];
	u8 samp_freq_index;
};

static int fdc1004_soft_reset(struct fdc1004_data *data)
{
	struct regmap *map = data->regmap;
	unsigned int reg_val;
	int ret;

	ret = regmap_write(map, FDC1004_REG_FDC_CONF, FDC1004_FDC_CONF_RST_MASK);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(map, FDC1004_REG_FDC_CONF, reg_val,
					!(reg_val & FDC1004_FDC_CONF_RST_MASK),
					1 * USEC_PER_MSEC, 200 * USEC_PER_MSEC);
}

static int fdc1004_read_raw(struct iio_dev *indio_dev,
			    const struct iio_chan_spec *chan,
			    int *val, int *val2, long mask)
{
	struct fdc1004_data *data = iio_priv(indio_dev);
	struct regmap *map = data->regmap;
	unsigned int reg_val;
	u16 reg_vals[2];
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		iio_val_s64_decompose(fdc1004_scale, val, val2);
		return IIO_VAL_DECIMAL64_YOCTO;
	case IIO_CHAN_INFO_RAW: {
		guard(mutex)(&data->lock);

		/* Configure the measurement we want. */
		reg_val = FIELD_PREP(FDC1004_CONF_MEAS_CHA_MASK, chan->channel) |
			  FIELD_PREP(FDC1004_CONF_MEAS_CHB_MASK, chan->channel2) |
			  FIELD_PREP(FDC1004_CONF_MEAS_CAPDAC_MASK,
				     data->offsets[chan->channel]);
		ret = regmap_write(map, FDC1004_REG_CONF_MEASn(0), reg_val);
		if (ret)
			return ret;

		/*
		 * Start the measurement. The "+ 1" is because the chip
		 * wants a 1-indexed sampling frequency.
		 */
		reg_val = FDC1004_FDC_CONF_MEAS_n_MASK(0) |
			  FIELD_PREP(FDC1004_FDC_CONF_RATE_MASK,
				     data->samp_freq_index + 1);
		ret = regmap_write(map, FDC1004_REG_FDC_CONF, reg_val);
		if (ret)
			return ret;

		/*
		 * Wait for the measurement to complete.
		 * The slowest sampling frequency the chip supports is 100 SPS,
		 * i.e. 1 sample every 10 ms, so our timeout can't be shorter
		 * than that. Go with 40 ms to be safe.
		 */
		ret = regmap_read_poll_timeout(map, FDC1004_REG_FDC_CONF, reg_val,
					       reg_val & FDC1004_FDC_CONF_DONE_n_MASK(0),
					       1 * USEC_PER_MSEC, 40 * USEC_PER_MSEC);
		if (ret)
			return ret;

		/*
		 * Now read the two registers making up the measurement.
		 * The datasheet says to read MSB first, LSB second.
		 */
		ret = regmap_bulk_read(map, FDC1004_REG_MEASn_MSB(0),
				       reg_vals, ARRAY_SIZE(reg_vals));
		if (ret)
			return ret;

		*val = (sign_extend32(reg_vals[0], 15) << 8) | (reg_vals[1] >> 8);
		return IIO_VAL_INT;
	}
	case IIO_CHAN_INFO_SAMP_FREQ: {
		guard(mutex)(&data->lock);

		*val = fdc1004_samp_freqs[data->samp_freq_index];
		return IIO_VAL_INT;
	}
	case IIO_CHAN_INFO_OFFSET: {
		guard(mutex)(&data->lock);

		*val = data->offsets[chan->channel] * fdc1004_offset[1];
		return IIO_VAL_INT;
	}
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = regmap_read(map, FDC1004_REG_OFFSET_CAL_CINn(chan->channel),
				  &reg_val);
		if (ret)
			return ret;

		iio_val_s64_decompose(sign_extend64(reg_val, 15) *
				      fdc1004_calibbias[1], val, val2);
		return IIO_VAL_DECIMAL64_FEMTO;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = regmap_read(map, FDC1004_REG_GAIN_CAL_CINn(chan->channel),
				  &reg_val);
		if (ret)
			return ret;

		iio_val_s64_decompose(reg_val * fdc1004_calibscale[1], val, val2);
		return IIO_VAL_DECIMAL64_FEMTO;
	default:
		return -EINVAL;
	}
}

static const int fdc1004_calibbias_decomposed[] = {
	lower_32_bits(fdc1004_calibbias[0]), upper_32_bits(fdc1004_calibbias[0]),
	lower_32_bits(fdc1004_calibbias[1]), upper_32_bits(fdc1004_calibbias[1]),
	lower_32_bits(fdc1004_calibbias[2]), upper_32_bits(fdc1004_calibbias[2]),
};

static const int fdc1004_calibscale_decomposed[] = {
	lower_32_bits(fdc1004_calibscale[0]), upper_32_bits(fdc1004_calibscale[0]),
	lower_32_bits(fdc1004_calibscale[1]), upper_32_bits(fdc1004_calibscale[1]),
	lower_32_bits(fdc1004_calibscale[2]), upper_32_bits(fdc1004_calibscale[2]),
};

static int fdc1004_read_avail(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      const int **vals, int *type, int *length,
			      long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = fdc1004_samp_freqs;
		*length = ARRAY_SIZE(fdc1004_samp_freqs);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_OFFSET:
		*vals = fdc1004_offset;
		*length = ARRAY_SIZE(fdc1004_offset);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_CALIBBIAS:
		*vals = fdc1004_calibbias_decomposed;
		*length = ARRAY_SIZE(fdc1004_calibbias_decomposed);
		*type = IIO_VAL_DECIMAL64_FEMTO;
		return IIO_AVAIL_RANGE;
	case IIO_CHAN_INFO_CALIBSCALE:
		*vals = fdc1004_calibscale_decomposed;
		*length = ARRAY_SIZE(fdc1004_calibscale_decomposed);
		*type = IIO_VAL_DECIMAL64_FEMTO;
		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int fdc1004_write_raw_get_fmt(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
	case IIO_CHAN_INFO_OFFSET:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
	case IIO_CHAN_INFO_CALIBSCALE:
		return IIO_VAL_DECIMAL64_FEMTO;
	default:
		return -EINVAL;
	}
}

static int fdc1004_write_raw(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     int val, int val2, long mask)
{
	struct fdc1004_data *data = iio_priv(indio_dev);
	struct regmap *map = data->regmap;
	s64 val64;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		for (unsigned int i = 0; i < ARRAY_SIZE(fdc1004_samp_freqs); i++) {
			if (fdc1004_samp_freqs[i] == val) {
				guard(mutex)(&data->lock);

				data->samp_freq_index = i;
				return 0;
			}
		}

		return -EINVAL;
	case IIO_CHAN_INFO_OFFSET: {
		if (val < fdc1004_offset[0] || val > fdc1004_offset[2])
			return -EINVAL;

		guard(mutex)(&data->lock);

		data->offsets[chan->channel] = DIV_ROUND_CLOSEST(val, fdc1004_offset[1]);
		return 0;
	}
	case IIO_CHAN_INFO_CALIBBIAS:
		val64 = iio_val_s64_compose(val, val2);
		if (val64 < fdc1004_calibbias[0] || val64 > fdc1004_calibbias[2])
			return -EINVAL;

		return regmap_write(map, FDC1004_REG_OFFSET_CAL_CINn(chan->channel),
				    DIV_S64_ROUND_CLOSEST(val64, fdc1004_calibbias[1]));
	case IIO_CHAN_INFO_CALIBSCALE:
		val64 = iio_val_s64_compose(val, val2);
		if (val64 < fdc1004_calibscale[0] || val64 > fdc1004_calibscale[2])
			return -EINVAL;

		return regmap_write(map, FDC1004_REG_GAIN_CAL_CINn(chan->channel),
				    DIV64_U64_ROUND_CLOSEST(val64, fdc1004_calibscale[1]));
	default:
		return -EINVAL;
	}
}

static int fdc1004_debugfs_reg_access(struct iio_dev *indio_dev,
				      unsigned int reg, unsigned int writeval,
				      unsigned int *readval)
{
	struct fdc1004_data *data = iio_priv(indio_dev);

	guard(mutex)(&data->lock);

	if (readval)
		return regmap_read(data->regmap, reg, readval);

	return regmap_write(data->regmap, reg, writeval);
}

#define FDC1004_SINGLE_ENDED_CAPACITANCE_CHANNEL(_channel)		 \
	{								 \
		.type = IIO_CAPACITANCE,				 \
		.indexed = true,					 \
		.channel = _channel,					 \
		.channel2 = FDC1004_CONF_MEAS_CHB_CAPDAC,		 \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		 \
				      BIT(IIO_CHAN_INFO_OFFSET) |	 \
				      BIT(IIO_CHAN_INFO_CALIBBIAS) |	 \
				      BIT(IIO_CHAN_INFO_CALIBSCALE),	 \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	 \
		.info_mask_shared_by_type_available =			 \
			BIT(IIO_CHAN_INFO_OFFSET) |			 \
			BIT(IIO_CHAN_INFO_CALIBBIAS) |			 \
			BIT(IIO_CHAN_INFO_CALIBSCALE),			 \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.info_mask_shared_by_all_available =			 \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			 \
	}

#define FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(_chan1, _chan2)	 \
	{								 \
		.type = IIO_CAPACITANCE,				 \
		.indexed = true,					 \
		.differential = true,					 \
		.channel = _chan1,					 \
		.channel2 = _chan2,					 \
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		 \
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	 \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
		.info_mask_shared_by_all_available =			 \
			BIT(IIO_CHAN_INFO_SAMP_FREQ),			 \
	}

static const struct iio_chan_spec fdc1004_channels[] = {
	FDC1004_SINGLE_ENDED_CAPACITANCE_CHANNEL(0),
	FDC1004_SINGLE_ENDED_CAPACITANCE_CHANNEL(1),
	FDC1004_SINGLE_ENDED_CAPACITANCE_CHANNEL(2),
	FDC1004_SINGLE_ENDED_CAPACITANCE_CHANNEL(3),

	/* The sensor only supports differential measurements where CHA < CHB. */
	FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(0, 1),
	FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(0, 2),
	FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(0, 3),

	FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(1, 2),
	FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(1, 3),

	FDC1004_DIFFERENTIAL_CAPACITANCE_CHANNEL(2, 3),
};

static const struct iio_info fdc1004_info = {
	.read_raw = fdc1004_read_raw,
	.read_avail = fdc1004_read_avail,
	.write_raw = fdc1004_write_raw,
	.write_raw_get_fmt = fdc1004_write_raw_get_fmt,
	.debugfs_reg_access = fdc1004_debugfs_reg_access,
};

static int fdc1004_probe(struct i2c_client *client)
{
	struct iio_dev *indio_dev;
	struct fdc1004_data *data;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*data));
	if (!indio_dev)
		return -ENOMEM;

	data = iio_priv(indio_dev);

	data->regmap = devm_regmap_init_i2c(client, &fdc1004_regmap_config);
	if (IS_ERR(data->regmap))
		return PTR_ERR(data->regmap);

	ret = devm_mutex_init(&client->dev, &data->lock);
	if (ret)
		return ret;

	ret = devm_regulator_get_enable(&client->dev, "vdd");
	if (ret)
		return ret;

	ret = fdc1004_soft_reset(data);
	if (ret)
		return ret;

	indio_dev->name = "fdc1004";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &fdc1004_info;
	indio_dev->channels = fdc1004_channels;
	indio_dev->num_channels = ARRAY_SIZE(fdc1004_channels);

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id fdc1004_id[] = {
	{ .name = "fdc1004" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, fdc1004_id);

static const struct of_device_id fdc1004_of_match[] = {
	{ .compatible = "ti,fdc1004" },
	{ }
};
MODULE_DEVICE_TABLE(of, fdc1004_of_match);

static struct i2c_driver fdc1004_driver = {
	.driver = {
		.name = "fdc1004",
		.of_match_table = fdc1004_of_match,
	},
	.probe = fdc1004_probe,
	.id_table = fdc1004_id,
};
module_i2c_driver(fdc1004_driver);

MODULE_AUTHOR("Victor Chernyakin <chernyakin.victor.j@outlook.com>");
MODULE_DESCRIPTION("Texas Instruments FDC1004 capacitive sensor driver");
MODULE_LICENSE("GPL");
