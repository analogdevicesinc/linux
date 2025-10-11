// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD4062 I3C ADC driver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i3c/device.h>
#include <linux/i3c/master.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/unaligned.h>

#define AD4062_REG_INTERFACE_CONFIG_A			0x00
#define AD4062_REG_DEVICE_CONFIG			0x02
#define     AD4062_REG_DEVICE_CONFIG_POWER_MODE_MSK	GENMASK(1, 0)
#define     AD4062_REG_DEVICE_CONFIG_LOW_POWER_MODE	3
#define AD4062_REG_PROD_ID_1				0x05
#define AD4062_REG_DEVICE_GRADE				0x06
#define AD4062_REG_SCRATCH_PAD				0x0A
#define AD4062_REG_VENDOR_H				0x0D
#define AD4062_REG_STREAM_MODE				0x0E
#define AD4062_REG_INTERFACE_STATUS			0x11
#define     AD4062_REG_INTERFACE_STATUS_NOT_RDY		BIT(7)
#define AD4062_REG_MODE_SET				0x20
#define     AD4062_REG_MODE_SET_ENTER_ADC		BIT(0)
#define AD4062_REG_ADC_MODES				0x21
#define     AD4062_REG_ADC_MODES_MODE_MSK		GENMASK(1, 0)
#define     AD4062_REG_ADC_MODES_DATA_FORMAT		BIT(7)
#define AD4062_REG_ADC_CONFIG				0x22
#define     AD4062_REG_ADC_CONFIG_REF_EN_MSK		BIT(5)
#define     AD4062_REG_ADC_CONFIG_SCALE_EN_MSK		BIT(4)
#define AD4062_REG_AVG_CONFIG				0x23
#define AD4062_REG_GP_CONF				0x24
#define     AD4062_REG_GP_CONF_MODE_MSK_0		GENMASK(2, 0)
#define     AD4062_REG_GP_CONF_MODE_MSK_1		GENMASK(6, 4)
#define AD4062_REG_INTR_CONF				0x25
#define     AD4062_REG_INTR_CONF_EN_MSK_0		GENMASK(1, 0)
#define     AD4062_REG_INTR_CONF_EN_MSK_1		GENMASK(5, 4)
#define AD4062_REG_TIMER_CONFIG				0x27
#define     AD4062_REG_TIMER_CONFIG_FS_MASK		GENMASK(7, 4)
#define     AD4062_REG_TIMER_CONFIG_300KSPS		0x2
#define AD4062_REG_MAX_LIMIT				0x29
#define AD4062_REG_MIN_LIMIT				0x2B
#define AD4062_REG_MAX_HYST				0x2C
#define AD4062_REG_MIN_HYST				0x2D
#define AD4062_REG_MON_VAL				0x2F
#define AD4062_REG_ADC_IBI_EN				0x31
#define AD4062_REG_ADC_IBI_EN_CONV_TRIGGER		BIT(2)
#define AD4062_REG_ADC_IBI_EN_MAX			BIT(1)
#define AD4062_REG_ADC_IBI_EN_MIN			BIT(0)
#define AD4062_REG_FUSE_CRC				0x40
#define AD4062_REG_DEVICE_STATUS			0x41
#define     AD4062_REG_DEVICE_STATUS_DEVICE_RDY		BIT(7)
#define     AD4062_REG_DEVICE_STATUS_DEVICE_RESET	BIT(6)
#define AD4062_REG_MIN_SAMPLE				0x45
#define AD4062_REG_IBI_STATUS				0x48
#define AD4062_REG_CONV_READ_LSB			0x50
#define AD4062_REG_CONV_READ				0x53
#define AD4062_REG_CONV_TRIGGER				0x59
#define AD4062_REG_CONV_AUTO				0x61
#define AD4062_MAX_REG					0x61

#define AD4062_I3C_VENDOR	0x0177

#define AD4050_MAX_AVG		0x7
#define AD4062_MAX_AVG		0xB
#define AD4062_MAX_RATE(x)	((x) == AD4062_2MSPS ? 2000000 : 500000)
#define AD4062_FS_OFFSET(g)	((g) == AD4062_2MSPS ? 0 : 2)
#define AD4062_FS(g)		(&ad4062_conversion_freqs[AD4062_FS_OFFSET(g)])
#define AD4062_FS_LEN(g)	(ARRAY_SIZE(ad4062_conversion_freqs) - (AD4062_FS_OFFSET(g)))
#define AD4062_MON_VAL_MAX_GAIN		1999970
#define AD4062_MON_VAL_MIDDLE_POINT	0x8000
#define AD4062_T_CNVH_NS	10
#define AD4062_VIO_3V3		3300000
#define AD4062_SPI_MAX_ADC_XFER_SPEED(x)	((x) >= AD4062_VIO_3V3 ? 83333333 : 58823529)
#define AD4062_SPI_MAX_REG_XFER_SPEED		16000000

enum ad4062_grade {
	AD4062_2MSPS,
};

enum ad4062_operation_mode {
	AD4062_SAMPLE_MODE = 0,
	AD4062_BURST_AVERAGING_MODE = 1,
	AD4062_MONITOR_MODE = 3,
};

enum ad4062_gp_mode {
	AD4062_GP_DISABLED,
	AD4062_GP_INTR,
	AD4062_GP_DRDY,
};

enum ad4062_interrupt_en {
	AD4062_INTR_EN_NEITHER,
	AD4062_INTR_EN_MIN,
	AD4062_INTR_EN_MAX,
	AD4062_INTR_EN_EITHER,
};

struct ad4062_chip_info {
	const struct iio_chan_spec channels[1];
	const char *name;
	u16 prod_id;
	u8 max_avg;
	u8 grade;
};

enum {
	AD4062_SCAN_TYPE_SAMPLE,
	AD4062_SCAN_TYPE_BURST_AVG,
};

static const struct iio_scan_type ad4062_scan_type_12_s[] = {
	[AD4062_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
	[AD4062_SCAN_TYPE_BURST_AVG] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
};

static const struct iio_scan_type ad4062_scan_type_16_s[] = {
	[AD4062_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
	[AD4062_SCAN_TYPE_BURST_AVG] = {
		.sign = 's',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_BE,
	},
};

struct ad4062_state {
	const struct ad4062_chip_info *chip;
	const struct ad4062_bus_ops *ops;
	enum ad4062_operation_mode mode;
	struct completion completion;
	struct iio_trigger *trigger;
	struct iio_dev *indio_dev;
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	u16 sampling_frequency;
	int vref_uv;
	u8 raw[4] __aligned(IIO_DMA_MINALIGN);
};

static const struct regmap_range ad4062_regmap_rd_ranges[] = {
	regmap_reg_range(AD4062_REG_INTERFACE_CONFIG_A, AD4062_REG_DEVICE_GRADE),
	regmap_reg_range(AD4062_REG_SCRATCH_PAD, AD4062_REG_INTERFACE_STATUS),
	regmap_reg_range(AD4062_REG_MODE_SET, AD4062_REG_ADC_IBI_EN),
	regmap_reg_range(AD4062_REG_FUSE_CRC, AD4062_REG_IBI_STATUS),
	regmap_reg_range(AD4062_REG_CONV_READ_LSB, AD4062_REG_CONV_AUTO),
};

static const struct regmap_access_table ad4062_regmap_rd_table = {
	.yes_ranges = ad4062_regmap_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad4062_regmap_rd_ranges),
};

static const struct regmap_range ad4062_regmap_wr_ranges[] = {
	regmap_reg_range(AD4062_REG_INTERFACE_CONFIG_A, AD4062_REG_DEVICE_CONFIG),
	regmap_reg_range(AD4062_REG_SCRATCH_PAD, AD4062_REG_SCRATCH_PAD),
	regmap_reg_range(AD4062_REG_STREAM_MODE, AD4062_REG_INTERFACE_STATUS),
	regmap_reg_range(AD4062_REG_MODE_SET, AD4062_REG_ADC_IBI_EN),
	regmap_reg_range(AD4062_REG_FUSE_CRC, AD4062_REG_DEVICE_STATUS),
};

static const struct regmap_access_table ad4062_regmap_wr_table = {
	.yes_ranges = ad4062_regmap_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad4062_regmap_wr_ranges),
};

static const char *const ad4062_conversion_freqs[] = {
	"2000000", "1000000", "300000", "100000",	/*  0 -  3 */
	"33300", "10000", "3000", "500",		/*  4 -  7 */
	"333", "250", "200", "166",			/*  8 - 11 */
	"140", "124", "111",				/* 12 - 15 */
};

static int ad4062_conversion_frequency_set(struct ad4062_state *st, u8 val)
{
	val += AD4062_FS_OFFSET(st->chip->grade);
	return regmap_write(st->regmap, AD4062_REG_TIMER_CONFIG,
			    FIELD_PREP(AD4062_REG_TIMER_CONFIG_FS_MASK, val));
}

static int ad4062_sampling_frequency_get(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	return st->sampling_frequency - AD4062_FS_OFFSET(st->chip->grade);
}

static int ad4062_sampling_frequency_set(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan,
					     unsigned int val)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	val += AD4062_FS_OFFSET(st->chip->grade);
	st->sampling_frequency = val;

	return 0;
}

static const struct iio_enum AD4062_2MSPS_conversion_freq_enum = {
	.items = AD4062_FS(AD4062_2MSPS),
	.num_items = AD4062_FS_LEN(AD4062_2MSPS),
	.set = ad4062_sampling_frequency_set,
	.get = ad4062_sampling_frequency_get,
};

#define AD4062_EXT_INFO(grade)						\
static struct iio_chan_spec_ext_info grade##_ext_info[] = {		\
	IIO_ENUM("sampling_frequency", IIO_SHARED_BY_ALL,		\
		 &grade##_conversion_freq_enum),			\
	IIO_ENUM_AVAILABLE("sampling_frequency", IIO_SHARED_BY_ALL,	\
			   &grade##_conversion_freq_enum),		\
	{ }								\
}

AD4062_EXT_INFO(AD4062_2MSPS);

#define AD4062_CHAN(bits, grade) {							\
	.type = IIO_VOLTAGE,								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW) |				\
				    BIT(IIO_CHAN_INFO_SCALE) |				\
				    BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
				    BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),	\
	.indexed = 1,									\
	.channel = 0,									\
	.has_ext_scan_type = 1,								\
	.ext_scan_type = ad4062_scan_type_##bits##_s,					\
	.num_ext_scan_type = ARRAY_SIZE(ad4062_scan_type_##bits##_s),			\
	.ext_info = grade##_ext_info,							\
}

static const struct ad4062_chip_info ad4060_chip_info = {
	.name = "ad4060",
	.channels = { AD4062_CHAN(12, AD4062_2MSPS) },
	.prod_id = 0x7A,
	.max_avg = AD4050_MAX_AVG,
	.grade = AD4062_2MSPS,
};

static const struct ad4062_chip_info ad4062_chip_info = {
	.name = "ad4062",
	.channels = { AD4062_CHAN(16, AD4062_2MSPS) },
	.prod_id = 0x7C,
	.max_avg = AD4062_MAX_AVG,
	.grade = AD4062_2MSPS,
};

static int ad4062_set_oversampling_ratio(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 unsigned int val)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (val < 1 || val > BIT(st->chip->max_avg + 1))
		return -EINVAL;

	/* 1 disables oversampling */
	if (val == 1) {
		st->mode = AD4062_SAMPLE_MODE;
	} else {
		val = ilog2(val);
		st->mode = AD4062_BURST_AVERAGING_MODE;
		ret = regmap_write(st->regmap, AD4062_REG_AVG_CONFIG, val - 1);
		if (ret)
			return ret;
	}

	return 0;
}

static int ad4062_get_oversampling_ratio(struct ad4062_state *st,
					 unsigned int *val)
{
	int ret, buf;

	if (st->mode == AD4062_SAMPLE_MODE) {
		*val = 1;
		return 0;
	}

	ret = regmap_read(st->regmap, AD4062_REG_AVG_CONFIG, &buf);
	if (ret)
		return ret;

	*val = BIT(buf + 1);

	return 0;
}

static int ad4062_check_ids(struct ad4062_state *st)
{
	int ret;
	u16 val;

	ret = regmap_bulk_read(st->regmap, AD4062_REG_PROD_ID_1, &st->raw, 2);
	if (ret)
		return ret;

	val = get_unaligned_be16(st->raw);
	if (val != st->chip->prod_id)
		dev_warn(&st->i3cdev->dev,
			 "Production ID x%x does not match known values", val);

	ret = regmap_bulk_read(st->regmap, AD4062_REG_VENDOR_H, &st->raw, 2);
	if (ret)
		return ret;

	val = get_unaligned_be16(st->raw);
	if (val != AD4062_I3C_VENDOR) {
		dev_err(&st->i3cdev->dev,
			"Vendor ID x%x does not match expected value\n", val);
		return -ENODEV;
	}

	return 0;
}

static int ad4062_set_operation_mode(struct ad4062_state *st,
				     enum ad4062_operation_mode mode)
{
	int ret;

	if (mode == AD4062_BURST_AVERAGING_MODE) {
		ret = ad4062_conversion_frequency_set(st, st->sampling_frequency);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, AD4062_REG_ADC_MODES,
				 AD4062_REG_ADC_MODES_MODE_MSK, mode);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD4062_REG_MODE_SET,
			    AD4062_REG_MODE_SET_ENTER_ADC);
}

static int ad4062_soft_reset(struct ad4062_state *st)
{
	u8 val = 0x81;
	int ret;

	ret = regmap_write(st->regmap, AD4062_REG_INTERFACE_CONFIG_A, val);
	if (ret)
		return ret;

	/* Wait AD4062 treset time */
	fsleep(5000);

	return 0;
}

static int ad4062_setup(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			const bool *ref_sel)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	int ret;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	u8 val = FIELD_PREP(AD4062_REG_GP_CONF_MODE_MSK_0, AD4062_GP_INTR) |
		 FIELD_PREP(AD4062_REG_GP_CONF_MODE_MSK_1, AD4062_GP_DRDY);

	ret = regmap_update_bits(st->regmap, AD4062_REG_GP_CONF,
				 AD4062_REG_GP_CONF_MODE_MSK_1 | AD4062_REG_GP_CONF_MODE_MSK_0,
				 val);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4062_REG_INTR_CONF_EN_MSK_0, (AD4062_INTR_EN_EITHER)) |
	      FIELD_PREP(AD4062_REG_INTR_CONF_EN_MSK_1, (AD4062_INTR_EN_NEITHER));

	ret = regmap_update_bits(st->regmap, AD4062_REG_ADC_MODES,
				 AD4062_REG_ADC_CONFIG_REF_EN_MSK,
				 FIELD_PREP(AD4062_REG_ADC_CONFIG_REF_EN_MSK,
					    *ref_sel));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4062_REG_DEVICE_STATUS,
			   AD4062_REG_DEVICE_STATUS_DEVICE_RESET);
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, AD4062_REG_INTR_CONF,
				  AD4062_REG_INTR_CONF_EN_MSK_0 | AD4062_REG_INTR_CONF_EN_MSK_1,
				  val);
}

static irqreturn_t ad4062_irq_handler_drdy(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ad4062_state *st = iio_priv(indio_dev);

	complete(&st->completion);

	return IRQ_HANDLED;
}

static void ad4062_ibi_handler(struct i3c_device *i3cdev,
			       const struct i3c_ibi_payload *payload)
{
	struct ad4062_state *st = i3cdev_get_drvdata(i3cdev);

	complete(&st->completion);
}

static int ad4062_request_ibi(struct i3c_device *i3cdev)
{
	const struct i3c_ibi_setup ibireq = {
		.max_payload_len = 1,
		.num_slots = 1,
		.handler = ad4062_ibi_handler,
	};
	int ret;

	ret = i3c_device_request_ibi(i3cdev, &ibireq);
	if (ret)
		return ret;

	ret = i3c_device_enable_ibi(i3cdev);
	if (ret)
		goto err_enable_ibi;
	return 0;

err_enable_ibi:
	i3c_device_free_ibi(i3cdev);
	return ret;
}

static int ad4062_request_irq(struct iio_dev *indio_dev)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	struct device *dev = &st->i3cdev->dev;
	int ret;

	ret = fwnode_irq_get_byname(dev_fwnode(&st->i3cdev->dev), "gp1");
	if (ret >= 0) {
		ret = devm_request_threaded_irq(dev, ret, NULL,
						 ad4062_irq_handler_drdy,
						 IRQF_ONESHOT, indio_dev->name,
						 indio_dev);
	} else if (ret != -EPROBE_DEFER) {
		ret = regmap_update_bits(st->regmap, AD4062_REG_ADC_IBI_EN,
					 AD4062_REG_ADC_IBI_EN_CONV_TRIGGER,
					 AD4062_REG_ADC_IBI_EN_CONV_TRIGGER);
	}

	return ret;
}

static const int ad4062_oversampling_avail[] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096,
};

static int ad4062_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *len, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*vals = ad4062_oversampling_avail;
		*len = ARRAY_SIZE(ad4062_oversampling_avail);
		*type = IIO_VAL_INT;

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad4062_get_chan_scale(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;

	scan_type = iio_get_current_scan_type(indio_dev, st->chip->channels);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	*val = (st->vref_uv * 2) / MILLI;

	*val2 = scan_type->realbits;

	return IIO_VAL_FRACTIONAL_LOG2;
}

static int ad4062_get_chan_calibscale(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val, int *val2)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	u16 gain;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD4062_REG_MON_VAL, &st->raw, 2);
	if (ret)
		return ret;

	gain = get_unaligned_be16(&st->raw);

	/* From datasheet: code out = code in × mon_val/0x8000 */
	*val = gain / AD4062_MON_VAL_MIDDLE_POINT;
	*val2 = mul_u64_u32_div(gain % AD4062_MON_VAL_MIDDLE_POINT, NANO,
				AD4062_MON_VAL_MIDDLE_POINT);

	return IIO_VAL_INT_PLUS_NANO;
}

static int ad4062_set_chan_calibscale(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int gain_int, int gain_frac)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	u64 gain;
	int ret;

	if (gain_int < 0 || gain_frac < 0)
		return -EINVAL;

	gain = mul_u32_u32(gain_int, MICRO) + gain_frac;

	if (gain > AD4062_MON_VAL_MAX_GAIN)
		return -EINVAL;

	put_unaligned_be16(DIV_ROUND_CLOSEST_ULL(gain * AD4062_MON_VAL_MIDDLE_POINT,
						 MICRO),
			   &st->raw);

	ret = regmap_bulk_write(st->regmap, AD4062_REG_MON_VAL, &st->raw, 2);
	if (ret)
		return ret;

	/* Enable scale if gain is not one. */
	return regmap_update_bits(st->regmap, AD4062_REG_ADC_MODES,
				  AD4062_REG_ADC_CONFIG_SCALE_EN_MSK,
				  FIELD_PREP(AD4062_REG_ADC_CONFIG_SCALE_EN_MSK,
					     !(gain_int == 1 && gain_frac == 0)));
}

static int __ad4062_read_chan_raw(struct ad4062_state *st, int *val)
{
	struct i3c_device *i3cdev = st->i3cdev;
	u8 addr = AD4062_REG_CONV_TRIGGER;
	struct i3c_priv_xfer t[2] = {
		{
			.data.out = &addr,
			.len = 1,
			.rnw = false,
		},
		{
			.data.in = &st->raw,
			.len = 4,
			.rnw = true,
		}
	};
	int ret;

	reinit_completion(&st->completion);
	/* Change address pointer to trigger conversion */
	ret = i3c_device_do_priv_xfers(i3cdev, &t[0], 1);
	if (ret)
		return ret;
	/*
	 * Single sample read should be used only for oversampling and
	 * sampling frequency pairs that take less than 1 sec.
	 */
	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
	if (!ret)
		return -ETIMEDOUT;

	ret = i3c_device_do_priv_xfers(i3cdev, &t[1], 1);
	if (ret)
		return ret;
	*val = get_unaligned_be32(st->raw);
	return ret;
}

static int ad4062_read_chan_raw(struct iio_dev *indio_dev, int *val)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	ret = pm_runtime_resume_and_get(&st->i3cdev->dev);
	if (ret)
		return ret;

	ret = ad4062_set_operation_mode(st, st->mode);
	if (ret)
		goto out_error;

	ret = __ad4062_read_chan_raw(st, val);

out_error:
	pm_runtime_put_autosuspend(&st->i3cdev->dev);
	return ret;
}

static int ad4062_read_raw_dispatch(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, int *val,
				    int *val2, long info)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4062_read_chan_raw(indio_dev, val);

	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4062_get_chan_calibscale(indio_dev, chan, val, val2);

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4062_get_oversampling_ratio(st, val);

	default:
		return -EINVAL;
	}
}

static int ad4062_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	int ret;

	if (info == IIO_CHAN_INFO_SCALE)
		return ad4062_get_chan_scale(indio_dev, chan, val, val2);

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = ad4062_read_raw_dispatch(indio_dev, chan, val, val2, info);

	iio_device_release_direct(indio_dev);
	return ret ? ret : IIO_VAL_INT;
}

static int ad4062_write_raw_dispatch(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int val,
				     int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4062_set_oversampling_ratio(indio_dev, chan, val);

	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4062_set_chan_calibscale(indio_dev, chan, val, val2);

	default:
		return -EINVAL;
	}
};

static int ad4062_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = ad4062_write_raw_dispatch(indio_dev, chan, val, val2, info);

	iio_device_release_direct(indio_dev);
	return ret;
}

static int ad4062_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				     unsigned int writeval, unsigned int *readval)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	if (readval)
		ret = regmap_read(st->regmap, reg, readval);
	else
		ret = regmap_write(st->regmap, reg, writeval);

	iio_device_release_direct(indio_dev);
	return ret;
}

static int ad4062_get_current_scan_type(const struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	return st->mode == AD4062_BURST_AVERAGING_MODE ?
			   AD4062_SCAN_TYPE_BURST_AVG :
			   AD4062_SCAN_TYPE_SAMPLE;
}

static const struct iio_info ad4062_info = {
	.read_raw = ad4062_read_raw,
	.write_raw = ad4062_write_raw,
	.read_avail = ad4062_read_avail,
	.get_current_scan_type = &ad4062_get_current_scan_type,
	.debugfs_reg_access = &ad4062_debugfs_reg_access,
};

static const struct regmap_config ad4062_regmap_config = {
	.name = "ad4062",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AD4062_MAX_REG,
	.rd_table = &ad4062_regmap_rd_table,
	.wr_table = &ad4062_regmap_wr_table,
	.can_sleep = true,
};

static int ad4062_regulators_get(struct ad4062_state *st, bool *ref_sel)
{
	struct device *dev = &st->i3cdev->dev;
	int uv;

	uv = devm_regulator_get_enable_read_voltage(dev, "vio");
	if (uv < 0)
		return dev_err_probe(dev, uv,
				     "Failed to enable and read vio voltage\n");

	uv = devm_regulator_get_enable_read_voltage(dev, "vdd");
	if (uv < 0)
		return dev_err_probe(dev, uv,
				     "Failed to enable vdd regulator\n");

	st->vref_uv = devm_regulator_get_enable_read_voltage(dev, "ref");
	*ref_sel = st->vref_uv == -ENODEV;
	if (st->vref_uv == -ENODEV)
		st->vref_uv = uv;
	else if (st->vref_uv < 0)
		return dev_err_probe(dev, st->vref_uv,
				     "Failed to enable and read ref voltage\n");
	return 0;
}

static const struct i3c_device_id ad4062_id_table[] = {
	I3C_DEVICE(AD4062_I3C_VENDOR, ad4060_chip_info.prod_id, &ad4060_chip_info),
	I3C_DEVICE(AD4062_I3C_VENDOR, ad4062_chip_info.prod_id, &ad4062_chip_info),
	{}
};
MODULE_DEVICE_TABLE(i3c, ad4062_id_table);

static int ad4062_probe(struct i3c_device *i3cdev)
{
	const struct i3c_device_id *id = i3c_device_match_id(i3cdev, ad4062_id_table);
	const struct ad4062_chip_info *chip = id->data;
	struct device *dev = &i3cdev->dev;
	struct iio_dev *indio_dev;
	struct ad4062_state *st;
	bool ref_sel;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->i3cdev = i3cdev;
	i3cdev_set_drvdata(i3cdev, st);
	init_completion(&st->completion);

	ret = ad4062_regulators_get(st, &ref_sel);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init_i3c(i3cdev, &ad4062_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	st->mode = AD4062_SAMPLE_MODE;
	st->chip = chip;
	st->sampling_frequency = AD4062_FS_OFFSET(st->chip->grade);
	st->indio_dev = indio_dev;

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 1;
	indio_dev->info = &ad4062_info;
	indio_dev->name = chip->name;
	indio_dev->channels = chip->channels;

	ret = ad4062_soft_reset(st);
	if (ret)
		return dev_err_probe(dev, ret, "AD4062 failed to soft reset\n");

	ret = ad4062_check_ids(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "AD4062 fields assertions failed\n");

	ret = ad4062_setup(indio_dev, indio_dev->channels, &ref_sel);
	if (ret)
		return ret;

	ret = ad4062_request_irq(indio_dev);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable pm_runtime\n");

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = ad4062_request_ibi(i3cdev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request i3c ibi\n");

	return devm_iio_device_register(dev, indio_dev);
}

static void ad4062_remove(struct i3c_device *i3cdev)
{
	i3c_device_disable_ibi(i3cdev);
	i3c_device_free_ibi(i3cdev);
}

static int ad4062_runtime_suspend(struct device *dev)
{
	struct ad4062_state *st = dev_get_drvdata(dev);

	return regmap_write(st->regmap, AD4062_REG_DEVICE_CONFIG,
			    FIELD_PREP(AD4062_REG_DEVICE_CONFIG_POWER_MODE_MSK,
				       AD4062_REG_DEVICE_CONFIG_LOW_POWER_MODE));
}

static int ad4062_runtime_resume(struct device *dev)
{
	struct ad4062_state *st = dev_get_drvdata(dev);
	int ret;

	ret = regmap_clear_bits(st->regmap, AD4062_REG_DEVICE_CONFIG,
				AD4062_REG_DEVICE_CONFIG_POWER_MODE_MSK);

	fsleep(4000);
	return ret;
}

static const struct dev_pm_ops ad4062_pm_ops = {
	SET_RUNTIME_PM_OPS(ad4062_runtime_suspend, ad4062_runtime_resume, NULL)
};

static struct i3c_driver ad4062_driver = {
	.driver = {
		.name = "ad4062",
		.pm = pm_ptr(&ad4062_pm_ops),
	},
	.probe = ad4062_probe,
	.remove = ad4062_remove,
	.id_table = ad4062_id_table,
};
module_i3c_driver(ad4062_driver);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4062");
MODULE_LICENSE("GPL");
