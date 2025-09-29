// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD4062 I3C ADC driver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include "linux/i3c/master.h"
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i3c/device.h>
#include <linux/iio/buffer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
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
#include <dt-bindings/iio/adc/adi,ad4052.h>

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
	struct work_struct trig_conv;
	struct completion completion;
	struct iio_trigger *trigger;
	struct iio_dev *indio_dev;
	struct i3c_device *i3cdev;
	struct regmap *regmap;
	u16 oversampling_frequency;
	u16 events_frequency;
	bool wait_event;
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

static const struct iio_event_spec ad4062_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_HYSTERESIS),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_HYSTERESIS),
	},
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

static int ad4062_oversampling_frequency_get(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	return st->oversampling_frequency - AD4062_FS_OFFSET(st->chip->grade);
}

static int ad4062_oversampling_frequency_set(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan,
					     unsigned int val)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	val += AD4062_FS_OFFSET(st->chip->grade);
	st->oversampling_frequency = val;

	return 0;
}

static const struct iio_enum AD4062_2MSPS_conversion_freq_enum = {
	.items = AD4062_FS(AD4062_2MSPS),
	.num_items = AD4062_FS_LEN(AD4062_2MSPS),
	.set = ad4062_oversampling_frequency_set,
	.get = ad4062_oversampling_frequency_get,
};

#define AD4062_EXT_INFO(grade)						\
static struct iio_chan_spec_ext_info grade##_ext_info[] = {		\
	IIO_ENUM("oversampling_frequency", IIO_SHARED_BY_ALL,		\
		 &grade##_conversion_freq_enum),			\
	IIO_ENUM_AVAILABLE("oversampling_frequency", IIO_SHARED_BY_ALL,	\
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
	.event_spec = ad4062_events,							\
	.num_event_specs = ARRAY_SIZE(ad4062_events),					\
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

static ssize_t ad4062_events_frequency_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct ad4062_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%s\n", ad4062_conversion_freqs[st->events_frequency]);
}

static ssize_t ad4062_events_frequency_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf,
					     size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	ret = __sysfs_match_string(AD4062_FS(st->chip->grade),
				   AD4062_FS_LEN(st->chip->grade), buf);
	if (ret < 0)
		goto out_release;

	st->events_frequency = ret;

out_release:
	iio_device_release_direct(indio_dev);
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(sampling_frequency, 0644,
		       ad4062_events_frequency_show,
		       ad4062_events_frequency_store, 0);

static ssize_t sampling_frequency_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct ad4062_state *st = iio_priv(dev_to_iio_dev(dev));
	int ret = 0;

	for (u8 i = AD4062_FS_OFFSET(st->chip->grade);
	     i < AD4062_FS_LEN(st->chip->grade); i++)
		ret += sysfs_emit_at(buf, ret, "%s ", ad4062_conversion_freqs[i]);

	ret += sysfs_emit_at(buf, ret, "\n");
	return ret;
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static struct attribute *ad4062_event_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad4062_event_attribute_group = {
	.attrs = ad4062_event_attributes,
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
		ret = ad4062_conversion_frequency_set(st, st->oversampling_frequency);
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

	ret = regmap_write(st->regmap, AD4062_REG_INTERFACE_CONFIG_A,
			   val);
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

static irqreturn_t ad4062_irq_handler_thresh(int irq, void *private)
{
	struct iio_dev *indio_dev = private;

	iio_push_event(indio_dev,
		       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, 0,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_EITHER),
		       iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static irqreturn_t ad4062_irq_handler_drdy(int irq, void *private)
{
	struct ad4062_state *st = private;
	struct iio_dev *indio_dev = st->indio_dev;

	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trigger);
	else
		complete(&st->completion);

	return IRQ_HANDLED;
}

static void ad4062_ibi_handler(struct i3c_device *i3cdev,
			       const struct i3c_ibi_payload *payload)
{
	struct ad4062_state *st = i3cdev_get_drvdata(i3cdev);

	if (st->wait_event) {
		iio_push_event(st->indio_dev,
			       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, 0,
						    IIO_EV_TYPE_THRESH,
						    IIO_EV_DIR_EITHER),
			       iio_get_time_ns(st->indio_dev));
	} else {
		if (iio_buffer_enabled(st->indio_dev))
			iio_trigger_poll_nested(st->trigger);
		else
			complete(&st->completion);
	}
}

static void ad4062_trigger_work(struct work_struct *work)
{
	struct ad4062_state *st = container_of(work, struct ad4062_state,
					       trig_conv);
	u8 addr = AD4062_REG_CONV_TRIGGER;
	int ret;

	struct i3c_priv_xfer xfer_read = {
		.data.in = &st->raw,
		.len = 4,
		.rnw = true,
	};
	struct i3c_priv_xfer xfer_trig = {
		.data.out = &addr,
		.len = 1,
		.rnw = false,
	};

	ret = i3c_device_do_priv_xfers(st->i3cdev, &xfer_read, 1);
	if (ret)
		return;
	iio_push_to_buffers_with_timestamp(st->indio_dev, &st->raw,
					   iio_get_time_ns(st->indio_dev));

	i3c_device_do_priv_xfers(st->i3cdev, &xfer_trig, 1);
}

static irqreturn_t ad4062_trigger_handler(int irq, void *private)
{
	struct iio_poll_func *pf = private;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4062_state *st = iio_priv(indio_dev);

	iio_trigger_notify_done(indio_dev->trig);
	schedule_work(&st->trig_conv);
	return IRQ_HANDLED;
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

	ret = fwnode_irq_get_byname(dev_fwnode(&st->i3cdev->dev), "gp0");
	if (ret >= 0) {
		ret = devm_request_threaded_irq(dev, ret, NULL,
						ad4062_irq_handler_thresh,
						IRQF_ONESHOT, indio_dev->name,
						indio_dev);
		if (ret)
			return ret;
	} else if (ret != -EPROBE_DEFER) {
		ret = regmap_update_bits(st->regmap, AD4062_REG_ADC_IBI_EN,
					 AD4062_REG_ADC_IBI_EN_MAX | AD4062_REG_ADC_IBI_EN_MIN,
					 AD4062_REG_ADC_IBI_EN_MAX | AD4062_REG_ADC_IBI_EN_MIN);
		if (ret)
			return ret;
	} else {
		return ret;
	}

	ret = fwnode_irq_get_byname(dev_fwnode(&st->i3cdev->dev), "gp1");
	if (ret >= 0) {
		ret = devm_request_threaded_irq(dev, ret,
						ad4062_irq_handler_drdy,
						NULL, IRQF_ONESHOT,
						indio_dev->name, st);
	} else if (ret != -EPROBE_DEFER) {
		ret = regmap_update_bits(st->regmap, AD4062_REG_ADC_IBI_EN,
					 AD4062_REG_ADC_IBI_EN_CONV_TRIGGER,
					 AD4062_REG_ADC_IBI_EN_CONV_TRIGGER);
	}

	return ret;
}

static const struct iio_trigger_ops ad4062_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};


static int ad4062_request_trigger(struct iio_dev *indio_dev)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	struct device *dev = &st->i3cdev->dev;
	int ret;

	st->trigger = devm_iio_trigger_alloc(dev, "%s-dev%d",
					     indio_dev->name,
					     iio_device_id(indio_dev));
	if (!st->trigger)
		return -ENOMEM;

	st->trigger->ops = &ad4062_trigger_ops;

	ret = devm_iio_trigger_register(dev, st->trigger);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trigger);
	return 0;
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
				 int *val,
				 int *val2)
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
				      int *val,
				      int *val2)
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
				      int gain_int,
				      int gain_frac)
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

	// Enable scale if gain is not one.
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
	pm_runtime_mark_last_busy(&st->i3cdev->dev);
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
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (info == IIO_CHAN_INFO_SCALE)
		return ad4062_get_chan_scale(indio_dev, chan, val, val2);

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	ret = ad4062_read_raw_dispatch(indio_dev, chan, val, val2, info);

out_release:
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
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	ret = ad4062_write_raw_dispatch(indio_dev, chan, val, val2, info);

out_release:
	iio_device_release_direct(indio_dev);
	return ret;
}

static int ad4062_monitor_mode_enable(struct ad4062_state *st)
{
	int ret;

	ret = pm_runtime_resume_and_get(&st->i3cdev->dev);
	if (ret)
		return ret;

	ret = ad4062_conversion_frequency_set(st, st->events_frequency);
	if (ret)
		goto out_error;

	ret = ad4062_set_operation_mode(st, AD4062_MONITOR_MODE);
	if (ret)
		goto out_error;

	return ret;
out_error:
	pm_runtime_mark_last_busy(&st->i3cdev->dev);
	pm_runtime_put_autosuspend(&st->i3cdev->dev);
	return ret;
}

static int ad4062_monitor_mode_disable(struct ad4062_state *st)
{
	pm_runtime_mark_last_busy(&st->i3cdev->dev);
	pm_runtime_put_autosuspend(&st->i3cdev->dev);
	return 0;
}

static int ad4062_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	return st->wait_event;
}

static int ad4062_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event == state) {
		ret = 0;
		goto out_release;
	}

	if (state)
		ret = ad4062_monitor_mode_enable(st);
	else
		ret = ad4062_monitor_mode_disable(st);

	if (!ret)
		st->wait_event = state;

out_release:
	iio_device_release_direct(indio_dev);
	return ret;
}

static int __ad4062_read_event_info_value(struct ad4062_state *st,
					   enum iio_event_direction dir, int *val)
{
	int ret;
	u8 reg;

	if (dir == IIO_EV_DIR_RISING)
		reg = AD4062_REG_MAX_LIMIT;
	else
		reg = AD4062_REG_MIN_LIMIT;

	ret = regmap_bulk_read(st->regmap, reg, &st->raw, 2);
	if (ret)
		return ret;

	*val = sign_extend32(get_unaligned_be16(st->raw), 11);

	return 0;
}

static int __ad4062_read_event_info_hysteresis(struct ad4062_state *st,
						enum iio_event_direction dir, int *val)
{
	u8 reg;

	if (dir == IIO_EV_DIR_RISING)
		reg = AD4062_REG_MAX_HYST;
	else
		reg = AD4062_REG_MIN_HYST;
	return regmap_read(st->regmap, reg, val);
}

static int ad4062_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	switch (info) {
	case IIO_EV_INFO_VALUE:
		ret = __ad4062_read_event_info_value(st, dir, val);
		break;
	case IIO_EV_INFO_HYSTERESIS:
		ret = __ad4062_read_event_info_hysteresis(st, dir, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

out_release:
	iio_device_release_direct(indio_dev);
	return ret ? ret : IIO_VAL_INT;
}

static int __ad4062_write_event_info_value(struct ad4062_state *st,
					   enum iio_event_direction dir, int val)
{
	u8 reg;

	if (val > 2047 || val < -2048)
		return -EINVAL;
	if (dir == IIO_EV_DIR_RISING)
		reg = AD4062_REG_MAX_LIMIT;
	else
		reg = AD4062_REG_MIN_LIMIT;
	put_unaligned_be16(val, &st->raw);

	return regmap_bulk_write(st->regmap, reg, &st->raw, 2);
}

static int __ad4062_write_event_info_hysteresis(struct ad4062_state *st,
						enum iio_event_direction dir, int val)
{
	u8 reg;

	if (val >= BIT(7))
		return -EINVAL;
	if (dir == IIO_EV_DIR_RISING)
		reg = AD4062_REG_MAX_HYST;
	else
		reg = AD4062_REG_MIN_HYST;

	return regmap_write(st->regmap, reg, val);
}

static int ad4062_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val,
				    int val2)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	switch (type) {
	case IIO_EV_TYPE_THRESH:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			ret = __ad4062_write_event_info_value(st, dir, val);
			break;
		case IIO_EV_INFO_HYSTERESIS:
			ret = __ad4062_write_event_info_hysteresis(st, dir, val);
			break;
		default:
			ret = -EINVAL;
			break;
		}
		break;
	default:
		ret = -EINVAL;
		break;
	}

out_release:
	iio_device_release_direct(indio_dev);
	return ret;
}

static int ad4062_triggered_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	u8 addr = AD4062_REG_CONV_TRIGGER;
	int ret;

	if (st->wait_event)
		return -EBUSY;

	ret = pm_runtime_resume_and_get(&st->i3cdev->dev);
	if (ret)
		return ret;

	ret = ad4062_set_operation_mode(st, st->mode);
	if (ret)
		goto out_mode_error;

	/* Trigger first conversion */
	struct i3c_priv_xfer t = {
		.data.out = &addr,
		.len = 1,
		.rnw = false,
	};

	ret = i3c_device_do_priv_xfers(st->i3cdev, &t, 1);
	if (ret)
		goto out_mode_error;
	return 0;

out_mode_error:
	pm_runtime_mark_last_busy(&st->i3cdev->dev);
	pm_runtime_put_autosuspend(&st->i3cdev->dev);

	return ret;
}

static int ad4062_triggered_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4062_state *st = iio_priv(indio_dev);

	pm_runtime_mark_last_busy(&st->i3cdev->dev);
	pm_runtime_put_autosuspend(&st->i3cdev->dev);
	return 0;
}

static const struct iio_buffer_setup_ops ad4062_triggered_buffer_setup_ops = {
	.postenable = &ad4062_triggered_buffer_postenable,
	.predisable = &ad4062_triggered_buffer_predisable,
};

static int ad4062_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				     unsigned int writeval, unsigned int *readval)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	if (readval)
		ret = regmap_read(st->regmap, reg, readval);
	else
		ret = regmap_write(st->regmap, reg, writeval);

out_release:
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
	.read_event_config = &ad4062_read_event_config,
	.write_event_config = &ad4062_write_event_config,
	.read_event_value = &ad4062_read_event_value,
	.write_event_value = &ad4062_write_event_value,
	.event_attrs = &ad4062_event_attribute_group,
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

static int __ad4062_validate_trigger_sources(struct of_phandle_args *trigger_sources)
{
	switch (trigger_sources->args[1]) {
	case AD4052_TRIGGER_PIN_GP0:
		return trigger_sources->args[0] == AD4052_TRIGGER_EVENT_EITHER_THRESH ?
		       0 : -EINVAL;
	case AD4052_TRIGGER_PIN_GP1:
		return trigger_sources->args[0] == AD4052_TRIGGER_EVENT_DATA_READY ?
		       0 : -EINVAL;
	default:
		return -EINVAL;
	}
}

static int ad4062_validate_trigger_sources(struct iio_dev *indio_dev)
{
	struct ad4062_state *st = iio_priv(indio_dev);
	struct of_phandle_args trigger_sources;
	struct device_node *np;
	int ret;

	np = st->i3cdev->dev.of_node;
	for (u8 i = 0; i < 2; i++) {
		ret = of_parse_phandle_with_args(np, "trigger-sources",
						 "#trigger-source-cells", i,
						 &trigger_sources);
		if (ret == -ENOENT)
			return 0;
		else if (ret)
			return ret;

		ret = __ad4062_validate_trigger_sources(&trigger_sources);
		of_node_put(trigger_sources.np);
		if (ret)
			return ret;
	}

	return ret;
}

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
		return dev_err_probe(dev,  PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	st->mode = AD4062_SAMPLE_MODE;
	st->wait_event = false;
	st->chip = chip;
	st->oversampling_frequency = AD4062_FS_OFFSET(st->chip->grade);
	st->events_frequency = AD4062_FS_OFFSET(st->chip->grade);
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

	ret = ad4062_validate_trigger_sources(indio_dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to validate trigger sources\n");

	ret = ad4062_request_trigger(indio_dev);
	if (ret)
		return ret;

	ret = devm_iio_triggered_buffer_setup(&i3cdev->dev, indio_dev,
					      &iio_pollfunc_store_time,
					      &ad4062_trigger_handler,
					      &ad4062_triggered_buffer_setup_ops);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable pm_runtime\n");

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = ad4062_request_ibi(i3cdev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to request i3c ibi\n");

	INIT_WORK(&st->trig_conv, ad4062_trigger_work);

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
