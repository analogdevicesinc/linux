// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices AD485x DAS driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/delay.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/units.h>
#include <linux/iio/backend.h>
#include <linux/iio/iio.h>
#include <linux/spi/spi.h>

#define AD485X_REG_INTERFACE_CONFIG_A	0x00
#define AD485X_REG_INTERFACE_CONFIG_B	0x01
#define AD485X_REG_PRODUCT_ID_L		0x04
#define AD485X_REG_PRODUCT_ID_H		0x05
#define AD485X_REG_DEVICE_CTRL		0x25
#define AD485X_REG_PACKET		0x26

#define AD485X_REG_CH_CONFIG_BASE	0x2A
#define AD485X_REG_CHX_SOFTSPAN(ch)	((0x12 * (ch)) + AD485X_REG_CH_CONFIG_BASE)
#define AD485X_REG_CHX_OFFSET(ch)	(AD485X_REG_CHX_SOFTSPAN(ch) + 0x01)
#define AD485X_REG_CHX_OFFSET_LSB(ch)	AD485X_REG_CHX_OFFSET(ch)
#define AD485X_REG_CHX_OFFSET_MID(ch)	(AD485X_REG_CHX_OFFSET_LSB(ch) + 0x01)
#define AD485X_REG_CHX_OFFSET_MSB(ch)	(AD485X_REG_CHX_OFFSET_MID(ch) + 0x01)
#define AD485X_REG_CHX_GAIN(ch)		(AD485X_REG_CHX_OFFSET(ch) + 0x03)
#define AD485X_REG_CHX_GAIN_LSB(ch)	AD485X_REG_CHX_GAIN(ch)
#define AD485X_REG_CHX_GAIN_MSB(ch)	(AD485X_REG_CHX_GAIN(ch) + 0x01)
#define AD485X_REG_CHX_PHASE(ch)	(AD485X_REG_CHX_GAIN(ch) + 0x02)
#define AD485X_REG_CHX_PHASE_LSB(ch)	AD485X_REG_CHX_PHASE(ch)
#define AD485X_REG_CHX_PHASE_MSB(ch)	(AD485X_REG_CHX_PHASE_LSB(ch) + 0x01)

#define AD485X_REG_TESTPAT_0(c)		(0x38 + (c) * 0x12)
#define AD485X_REG_TESTPAT_1(c)		(0x39 + (c) * 0x12)
#define AD485X_REG_TESTPAT_2(c)		(0x3A + (c) * 0x12)
#define AD485X_REG_TESTPAT_3(c)		(0x3B + (c) * 0x12)

#define AD485X_SW_RESET			(BIT(7) | BIT(0))
#define AD485X_SDO_ENABLE		BIT(4)
#define AD485X_SINGLE_INSTRUCTION	BIT(7)
#define AD485X_ECHO_CLOCK_MODE		BIT(0)

#define AD485X_PACKET_FORMAT_0		0
#define AD485X_PACKET_FORMAT_1		1
#define AD485X_PACKET_FORMAT_MASK	GENMASK(1, 0)
#define AD485X_OS_EN			BIT(7)

#define AD485X_TEST_PAT			BIT(2)

#define AD4858_PACKET_SIZE_20		0
#define AD4858_PACKET_SIZE_24		1
#define AD4858_PACKET_SIZE_32		2

#define AD4857_PACKET_SIZE_16		0
#define AD4857_PACKET_SIZE_24		1

#define AD485X_TESTPAT_0_DEFAULT	0x2A
#define AD485X_TESTPAT_1_DEFAULT	0x3C
#define AD485X_TESTPAT_2_DEFAULT	0xCE
#define AD485X_TESTPAT_3_DEFAULT(c)	(0x0A + (0x10 * (c)))

#define AD485X_MAX_LANES		8
#define AD485X_MAX_IODELAY		32

#define AD485X_T_CNVH_NS		40

enum {
	ID_AD4858 = 0,
	ID_AD4857,
	ID_AD4856,
	ID_AD4855,
	ID_AD4854,
	ID_AD4853,
	ID_AD4852,
	ID_AD4851,
	ID_AD4858I,
};

struct ad485x_chip_info {
	const char		*name;
	unsigned int		product_id;
	const struct		iio_chan_spec *channels;
	unsigned int		num_channels;
	unsigned long		throughput;
	unsigned int		resolution;
};

struct ad485x_state {
	struct spi_device		*spi;
	struct pwm_device		*cnv;
	struct iio_backend		*back;
	struct regmap			*regmap;
	const struct ad485x_chip_info	*info;
	unsigned long			sampling_freq;
};

static int ad485x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad485x_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad485x_set_sampling_freq(struct ad485x_state *st, unsigned int freq)
{
	struct pwm_state cnv_state = {
		.duty_cycle = AD485X_T_CNVH_NS,
		.enabled = true,
	};
	int ret;

	if (freq > st->info->throughput)
		freq = st->info->throughput;

	cnv_state.period = DIV_ROUND_CLOSEST_ULL(1000000000, freq);

	ret = pwm_apply_state(st->cnv, &cnv_state);
	if (ret)
		return ret;

	st->sampling_freq = freq;

	return 0;
}

static int ad485x_setup(struct ad485x_state *st)
{
	unsigned int product_id;
	int ret;

	ret = ad485x_set_sampling_freq(st, 1000000);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD485X_REG_INTERFACE_CONFIG_A,
			   AD485X_SW_RESET);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD485X_REG_INTERFACE_CONFIG_B,
			   AD485X_SINGLE_INSTRUCTION);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD485X_REG_INTERFACE_CONFIG_A,
			   AD485X_SDO_ENABLE);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD485X_REG_PRODUCT_ID_L, &product_id);
	if (ret)
		return ret;

	if (product_id != st->info->product_id) {
		dev_warn(&st->spi->dev, "Unknown product ID: 0x%02X\n",
			 product_id);
	}

	ret = regmap_write(st->regmap, AD485X_REG_DEVICE_CTRL,
			   AD485X_ECHO_CLOCK_MODE);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD485X_REG_PACKET, 0);
}

static int ad485x_find_opt(bool *field, u32 size, u32 *ret_start)
{
	int i, cnt = 0, max_cnt = 0, start, max_start = 0;

	for (i = 0, start = -1; i < size; i++) {
		if (field[i] == 0) {
			if (start == -1)
				start = i;
			cnt++;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
			start = -1;
			cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}

	*ret_start = max_start;

	return max_cnt;
}

static int ad485x_calibrate(struct ad485x_state *st)
{
	int opt_delay, lane_num, delay, i, s, c;
	enum iio_backend_interface_type interface_type;
	bool pn_status[AD485X_MAX_LANES][AD485X_MAX_IODELAY];
	int ret;

	ret = iio_backend_interface_type_get(st->back, &interface_type);
	if (ret)
		return ret;
	if (interface_type == IIO_BACKEND_CMOS)
		lane_num = st->info->num_channels;
	else
		lane_num = 1;

	if (st->info->resolution == 16) {
		ret = iio_backend_packet_size_set(st->back, AD4857_PACKET_SIZE_24);
		if (ret)
			return ret;
		ret = regmap_write(st->regmap, AD485X_REG_PACKET,
				   AD485X_TEST_PAT | AD4857_PACKET_SIZE_24);
		if (ret)
			return ret;
	} else {
		ret = iio_backend_packet_size_set(st->back, AD4858_PACKET_SIZE_32);
		if (ret)
			return ret;
		ret = regmap_write(st->regmap, AD485X_REG_PACKET,
				   AD485X_TEST_PAT | AD4858_PACKET_SIZE_32);
		if (ret)
			return ret;
	}

	for (i = 0; i < st->info->num_channels; i++) {
		ret = regmap_write(st->regmap, AD485X_REG_TESTPAT_0(i),
				   AD485X_TESTPAT_0_DEFAULT);
		if (ret)
			return ret;
		ret = regmap_write(st->regmap, AD485X_REG_TESTPAT_1(i),
				   AD485X_TESTPAT_1_DEFAULT);
		if (ret)
			return ret;
		ret = regmap_write(st->regmap, AD485X_REG_TESTPAT_2(i),
				   AD485X_TESTPAT_2_DEFAULT);
		if (ret)
			return ret;
		ret = regmap_write(st->regmap, AD485X_REG_TESTPAT_3(i),
				   AD485X_TESTPAT_3_DEFAULT(i));
		if (ret)
			return ret;
		ret = iio_backend_chan_enable(st->back, i);
		if (ret)
			return ret;
	}

	for (i = 0; i < lane_num; i++) {
		for (delay = 0; delay < AD485X_MAX_IODELAY; delay++) {
			ret = iio_backend_iodelay_set(st->back, i, delay);
			if (ret)
				return ret;
			ret = iio_backend_chan_status(st->back, i,
						      &pn_status[i][delay]);
			if (ret)
				return ret;
		}
	}

	for (i = 0; i < lane_num; i++) {
		c = ad485x_find_opt(&pn_status[i][0], AD485X_MAX_IODELAY, &s);
		opt_delay = s + c / 2;
		ret = iio_backend_iodelay_set(st->back, i, opt_delay);
		if (ret)
			return ret;
	}

	for (i = 0; i < st->info->num_channels; i++) {
		ret = iio_backend_chan_disable(st->back, i);
		if (ret)
			return ret;
	}

	ret = iio_backend_packet_size_set(st->back, 0);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD485X_REG_PACKET, 0);
}

static const char * const ad485x_softspan[] = {
	[0] = "0-2.5",
	[1] = "M2.5-2.5",
	[2] = "0-5",
	[3] = "M5-5",
	[4] = "0-6.25",
	[5] = "M6.25-6.25",
	[6] = "0-10",
	[7] = "M10-10",
	[8] = "0-12.5",
	[9] = "M12.5-12.5",
	[10] = "0-20",
	[11] = "M20-20",
	[12] = "0-25",
	[13] = "M25-25",
	[14] = "0-40",
	[15] = "M40-40",
};

static const long ad485x_softspan_range_mv[] = {
	[0] = 2500,   /* 0-2.5 */
	[1] = 5000,   /* M2.5-2.5 */
	[2] = 5000,   /* 0-5 */
	[3] = 10000,  /* M5-5 */
	[4] = 6250,   /* 0-6.25 */
	[5] = 12500,  /* M6.25-6.25 */
	[6] = 10000,  /* 0-10 */
	[7] = 20000,  /* M10-10 */
	[8] = 12500,  /* 0-12.5 */
	[9] = 25000,  /* M12.5-12.5 */
	[10] = 20000, /* 0-20 */
	[11] = 40000, /* M20-20 */
	[12] = 25000, /* 0-25 */
	[13] = 50000, /* M25-25 */
	[14] = 40000, /* 0-40 */
	[15] = 80000, /* M40-40 */
};

static int ad485x_softspan_write(struct iio_dev *indio_dev,
				 const struct iio_chan_spec *chan,
				 unsigned int item)
{
	struct ad485x_state *st = iio_priv(indio_dev);

	return regmap_write(st->regmap, AD485X_REG_CHX_SOFTSPAN(chan->channel),
			    item);
}

static int ad485x_softspan_read(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct ad485x_state *st = iio_priv(indio_dev);
	unsigned int softspan;
	int ret;

	ret = regmap_read(st->regmap, AD485X_REG_CHX_SOFTSPAN(chan->channel),
			  &softspan);
	if (ret)
		return ret;

	return softspan;
}

static const struct iio_enum ad485x_softspan_enum = {
	.items = ad485x_softspan,
	.num_items = ARRAY_SIZE(ad485x_softspan),
	.set = ad485x_softspan_write,
	.get = ad485x_softspan_read,
};

static const char *const ad4858_packet_fmts[] = {
	"20-bit", "24-bit", "32-bit"
};

static const char *const ad4857_packet_fmts[] = {
	"16-bit", "24-bit"
};

static int ad485x_set_packet_format(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int format)
{
	struct ad485x_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_backend_packet_size_set(st->back, format);
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, AD485X_REG_PACKET,
				  AD485X_PACKET_FORMAT_MASK, format);
}

static int ad485x_get_packet_format(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad485x_state *st = iio_priv(indio_dev);
	unsigned int format;
	int ret;

	ret = regmap_read(st->regmap, AD485X_REG_PACKET, &format);
	if (ret)
		return ret;

	format &= AD485X_PACKET_FORMAT_MASK;

	return format;
}

static const struct iio_enum ad4858_packet_fmt = {
	.items = ad4858_packet_fmts,
	.num_items = ARRAY_SIZE(ad4858_packet_fmts),
	.set = ad485x_set_packet_format,
	.get = ad485x_get_packet_format,
};

static const struct iio_enum ad4857_packet_fmt = {
	.items = ad4857_packet_fmts,
	.num_items = ARRAY_SIZE(ad4857_packet_fmts),
	.set = ad485x_set_packet_format,
	.get = ad485x_get_packet_format,
};

static int ad485x_get_calibscale(struct ad485x_state *st, int ch, int *val,
				 int *val2)
{
	unsigned int reg_val;
	int gain;
	int ret;

	ret = regmap_read(st->regmap, AD485X_REG_CHX_GAIN_MSB(ch),
			  &reg_val);
	if (ret < 0)
		return ret;

	gain = (reg_val & 0xFF) << 8;
	ret = regmap_read(st->regmap, AD485X_REG_CHX_GAIN_LSB(ch),
			  &reg_val);
	if (ret < 0)
		return ret;

	gain |= reg_val & 0xFF;

	*val = gain;
	*val2 = 32768;

	return IIO_VAL_FRACTIONAL;
}

static int ad485x_set_calibscale(struct ad485x_state *st, int ch, int val,
				 int val2)
{
	unsigned long long gain;
	unsigned int reg_val;
	int ret;

	gain = val * 1000000 + val2;
	gain = gain * 32768;
	gain = DIV_U64_ROUND_CLOSEST(gain, 1000000);

	reg_val = gain;

	ret = regmap_write(st->regmap, AD485X_REG_CHX_GAIN_MSB(ch),
			   reg_val >> 8);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, AD485X_REG_CHX_GAIN_LSB(ch),
			    reg_val & 0xFF);
}

static int ad485x_get_calibbias(struct ad485x_state *st, int ch, int *val,
				int *val2)
{
	unsigned int lsb, mid, msb;
	int ret;

	ret = regmap_read(st->regmap, AD485X_REG_CHX_OFFSET_MSB(ch),
			  &msb);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, AD485X_REG_CHX_OFFSET_MID(ch),
			  &mid);
	if (ret < 0)
		return ret;

	ret = regmap_read(st->regmap, AD485X_REG_CHX_OFFSET_LSB(ch),
			  &lsb);
	if (ret < 0)
		return ret;

	if (st->info->resolution == 16) {
		*val = msb << 8;
		*val |= mid;
		if (*val & 0x8000)
			*val |= ~((1 << 16) - 1);
	} else {
		*val = msb << 12;
		*val |= mid << 4;
		*val |= lsb >> 4;
		if (*val & 0x80000)
			*val |= ~((1 << 20) - 1);
	}

	return IIO_VAL_INT;
}

static int ad485x_set_calibbias(struct ad485x_state *st, int ch, int val,
				int val2)
{
	unsigned int lsb, mid, msb;
	int ret;

	if (st->info->resolution == 16) {
		lsb = 0;
		mid = val & 0xFF;
		msb = (val >> 8) & 0xFF;
	} else {
		lsb = (val << 4) & 0xFF;
		mid = (val >> 4) & 0xFF;
		msb = (val >> 12) & 0xFF;
	}

	ret = regmap_write(st->regmap, AD485X_REG_CHX_OFFSET_LSB(ch), lsb);
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, AD485X_REG_CHX_OFFSET_MID(ch), mid);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, AD485X_REG_CHX_OFFSET_MSB(ch), msb);
}

static int ad485x_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad485x_state *st = iio_priv(indio_dev);
	unsigned int softspan;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad485x_get_calibscale(st, chan->channel, val, val2);
	case IIO_CHAN_INFO_SCALE:
		ret = regmap_read(st->regmap,
				  AD485X_REG_CHX_SOFTSPAN(chan->channel),
				  &softspan);
		if (ret)
			return ret;
		*val = ad485x_softspan_range_mv[softspan];
		*val2 = (1 << chan->scan_type.realbits) * 1000;
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad485x_get_calibbias(st, chan->channel, val, val2);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad485x_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad485x_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad485x_set_sampling_freq(st, val);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad485x_set_calibscale(st, chan->channel, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad485x_set_calibbias(st, chan->channel, val, val2);
	default:
		return -EINVAL;
	}
}

static int ad485x_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad485x_state *st = iio_priv(indio_dev);
	unsigned int c;
	int ret;

	for (c = 0; c < st->info->num_channels; c++) {
		if (test_bit(c, scan_mask))
			ret = iio_backend_chan_enable(st->back, c);
		else
			ret = iio_backend_chan_disable(st->back, c);
		if (ret)
			return ret;
	}

	return 0;
}

#define AD485X_IIO_CHANNEL(index, real, storage, info)		\
{								\
	.type = IIO_VOLTAGE,					\
	.ext_info = info,					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |	\
		BIT(IIO_CHAN_INFO_CALIBBIAS) |			\
		BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
	.address = index,					\
	.indexed = 1,						\
	.channel = index,					\
	.scan_index = index,					\
	.scan_type = {						\
		.sign = 's',					\
		.realbits = real,				\
		.storagebits = storage,				\
	},							\
}

static struct iio_chan_spec_ext_info ad4858_ext_info[] = {
	IIO_ENUM("softspan", IIO_SEPARATE, &ad485x_softspan_enum),
	IIO_ENUM_AVAILABLE("softspan", IIO_SHARED_BY_TYPE,
			   &ad485x_softspan_enum),
	IIO_ENUM("packet_format", IIO_SHARED_BY_ALL, &ad4858_packet_fmt),
	IIO_ENUM_AVAILABLE("packet_format",
			   IIO_SHARED_BY_ALL, &ad4858_packet_fmt),
	{},
};

static struct iio_chan_spec_ext_info ad4857_ext_info[] = {
	IIO_ENUM("softspan", IIO_SEPARATE, &ad485x_softspan_enum),
	IIO_ENUM_AVAILABLE("softspan", IIO_SHARED_BY_TYPE,
			   &ad485x_softspan_enum),
	IIO_ENUM("packet_format", IIO_SHARED_BY_ALL, &ad4857_packet_fmt),
	IIO_ENUM_AVAILABLE("packet_format",
			   IIO_SHARED_BY_ALL, &ad4857_packet_fmt),
	{},
};

static const struct iio_chan_spec ad4858_channels[] = {
	AD485X_IIO_CHANNEL(0, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(1, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(2, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(3, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(4, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(5, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(6, 20, 32, ad4858_ext_info),
	AD485X_IIO_CHANNEL(7, 20, 32, ad4858_ext_info),
};

static const struct iio_chan_spec ad4857_channels[] = {
	AD485X_IIO_CHANNEL(0, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(1, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(2, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(3, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(4, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(5, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(6, 16, 16, ad4857_ext_info),
	AD485X_IIO_CHANNEL(7, 16, 16, ad4857_ext_info),
};

static const struct ad485x_chip_info ad4858_info = {
	.name = "ad4858",
	.product_id = 0x60,
	.channels = ad4858_channels,
	.num_channels = ARRAY_SIZE(ad4858_channels),
	.throughput = 1 * MEGA,
	.resolution = 20,
};

static const struct ad485x_chip_info ad4857_info = {
	.name = "ad4857",
	.product_id = 0x61,
	.channels = ad4857_channels,
	.num_channels = ARRAY_SIZE(ad4857_channels),
	.throughput = 1 * MEGA,
	.resolution = 16,
};

static const struct ad485x_chip_info ad4856_info = {
	.name = "ad4856",
	.product_id = 0x62,
	.channels = ad4858_channels,
	.num_channels = ARRAY_SIZE(ad4858_channels),
	.throughput = 250 * KILO,
	.resolution = 20,
	.resolution = 16,
};

static const struct ad485x_chip_info ad4855_info = {
	.name = "ad4855",
	.product_id = 0x63,
	.channels = ad4857_channels,
	.num_channels = ARRAY_SIZE(ad4857_channels),
	.throughput = 250 * KILO,
	.resolution = 16,
};

static const struct ad485x_chip_info ad4854_info = {
	.name = "ad4854",
	.product_id = 0x64,
	.channels = ad4858_channels,
	.num_channels = ARRAY_SIZE(ad4858_channels),
	.throughput = 1 * MEGA,
	.resolution = 20,
};

static const struct ad485x_chip_info ad4853_info = {
	.name = "ad4853",
	.product_id = 0x65,
	.channels = ad4857_channels,
	.num_channels = ARRAY_SIZE(ad4857_channels),
	.throughput = 1 * MEGA,
	.resolution = 16,
};

static const struct ad485x_chip_info ad4852_info = {
	.name = "ad4852",
	.product_id = 0x66,
	.channels = ad4858_channels,
	.num_channels = ARRAY_SIZE(ad4858_channels),
	.throughput = 250 * KILO,
	.resolution = 20,
};

static const struct ad485x_chip_info ad4851_info = {
	.name = "ad4851",
	.product_id = 0x67,
	.channels = ad4857_channels,
	.num_channels = ARRAY_SIZE(ad4857_channels),
	.throughput = 250 * KILO,
	.resolution = 16,
};

static const struct ad485x_chip_info ad4858i_info = {
	.name = "ad4858i",
	.product_id = 0x6f,
	.channels = ad4858_channels,
	.num_channels = ARRAY_SIZE(ad4858_channels),
	.throughput = 1000000,
	.resolution = 20,
};

static const struct iio_info ad485x_info = {
	.debugfs_reg_access = ad485x_reg_access,
	.read_raw = ad485x_read_raw,
	.write_raw = ad485x_write_raw,
	.update_scan_mode = ad485x_update_scan_mode,
};

static const struct regmap_config regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int ad485x_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad485x_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	st->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(st->cnv))
		return PTR_ERR(st->cnv);

	st->info = spi_get_device_match_data(spi);
	if (!st->info)
		return -ENODEV;

	st->regmap = devm_regmap_init_spi(spi, &regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = ad485x_setup(st);
	if (ret < 0)
		return ret;

	indio_dev->name = st->info->name;
	indio_dev->channels = st->info->channels;
	indio_dev->num_channels = st->info->num_channels;
	indio_dev->info = &ad485x_info;

	st->back = devm_iio_backend_get(&spi->dev, NULL);
	if (IS_ERR(st->back))
		return PTR_ERR(st->back);

	ret = devm_iio_backend_request_buffer(&spi->dev, st->back, indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_backend_enable(&spi->dev, st->back);
	if (ret)
		return ret;

	ret = ad485x_calibrate(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad485x_of_match[] = {
	{ .compatible = "adi,ad4858", .data = &ad4858_info, },
	{ .compatible = "adi,ad4857", .data = &ad4857_info, },
	{ .compatible = "adi,ad4856", .data = &ad4856_info,  },
	{ .compatible = "adi,ad4855", .data = &ad4855_info,  },
	{ .compatible = "adi,ad4854", .data = &ad4854_info,  },
	{ .compatible = "adi,ad4853", .data = &ad4853_info,  },
	{ .compatible = "adi,ad4852", .data = &ad4852_info,  },
	{ .compatible = "adi,ad4851", .data = &ad4851_info,  },
	{ .compatible = "adi,ad4858i", .data = &ad4858i_info,  },
	{}
};

static const struct spi_device_id ad7949_spi_id[] = {
	{ "ad4858", (kernel_ulong_t)&ad4858_info },
	{ "ad4857", (kernel_ulong_t)&ad4857_info },
	{ "ad4856", (kernel_ulong_t)&ad4856_info },
	{ "ad4855", (kernel_ulong_t)&ad4855_info },
	{ "ad4854", (kernel_ulong_t)&ad4854_info },
	{ "ad4853", (kernel_ulong_t)&ad4853_info },
	{ "ad4852", (kernel_ulong_t)&ad4852_info },
	{ "ad4851", (kernel_ulong_t)&ad4851_info },
	{ "ad4858i", (kernel_ulong_t)&ad4858i_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad7949_spi_id);

static struct spi_driver ad485x_driver = {
	.probe = ad485x_probe,
	.driver = {
		.name   = "ad485x",
		.of_match_table = ad485x_of_match,
	},
};
module_spi_driver(ad485x_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_AUTHOR("Antoniu Miclaus <antoniu.miclaus@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD485x DAS driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_BACKEND);
