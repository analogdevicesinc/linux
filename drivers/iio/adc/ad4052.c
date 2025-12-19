// SPDX-License-Identifier: GPL-2.0ad4052
/*
 * Analog Devices AD4052 SPI ADC driver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/offload/consumer.h>
#include <linux/spi/offload/provider.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/unaligned.h>
#include <dt-bindings/iio/adc/adi,ad4052.h>

#define AD4052_REG_INTERFACE_CONFIG_A			0x00
#define AD4052_REG_DEVICE_CONFIG			0x02
#define     AD4052_REG_DEVICE_CONFIG_POWER_MODE_MSK	GENMASK(1, 0)
#define     AD4052_REG_DEVICE_CONFIG_LOW_POWER_MODE	3
#define AD4052_REG_PROD_ID_1				0x05
#define AD4052_REG_DEVICE_GRADE				0x06
#define AD4052_REG_SCRATCH_PAD				0x0A
#define AD4052_REG_VENDOR_H				0x0D
#define AD4052_REG_STREAM_MODE				0x0E
#define AD4052_REG_INTERFACE_STATUS			0x11
#define     AD4052_REG_INTERFACE_STATUS_NOT_RDY		BIT(7)
#define AD4052_REG_MODE_SET				0x20
#define     AD4052_REG_MODE_SET_ENTER_ADC		BIT(0)
#define AD4052_REG_ADC_MODES				0x21
#define     AD4052_REG_ADC_MODES_MODE_MSK		GENMASK(1, 0)
#define     AD4052_REG_ADC_MODES_DATA_FORMAT		BIT(7)
#define AD4052_REG_ADC_CONFIG				0x22
#define     AD4052_REG_ADC_CONFIG_REF_EN_MSK		BIT(5)
#define     AD4052_REG_ADC_CONFIG_SCALE_EN_MSK		BIT(4)
#define AD4052_REG_AVG_CONFIG				0x23
#define AD4052_REG_GP_CONF				0x24
#define     AD4052_REG_GP_CONF_MODE_MSK_0		GENMASK(2, 0)
#define     AD4052_REG_GP_CONF_MODE_MSK_1		GENMASK(6, 4)
#define AD4052_REG_INTR_CONF				0x25
#define     AD4052_REG_INTR_CONF_EN_MSK_0		GENMASK(1, 0)
#define     AD4052_REG_INTR_CONF_EN_MSK_1		GENMASK(5, 4)
#define AD4052_REG_TIMER_CONFIG				0x27
#define     AD4052_REG_TIMER_CONFIG_FS_MASK		GENMASK(7, 4)
#define     AD4052_REG_TIMER_CONFIG_300KSPS		0x2
#define AD4052_REG_MAX_LIMIT				0x29
#define AD4052_REG_MIN_LIMIT				0x2B
#define AD4052_REG_MAX_HYST				0x2C
#define AD4052_REG_MIN_HYST				0x2D
#define AD4052_REG_MON_VAL				0x2F
#define AD4052_REG_FUSE_CRC				0x40
#define AD4052_REG_DEVICE_STATUS			0x41
#define     AD4052_REG_DEVICE_STATUS_MIN_FLAG		BIT(2)
#define     AD4052_REG_DEVICE_STATUS_MAX_FLAG		BIT(3)
#define     AD4052_REG_DEVICE_STATUS_DEVICE_RDY		BIT(7)
#define     AD4052_REG_DEVICE_STATUS_DEVICE_RESET	BIT(6)
#define AD4052_REG_MIN_SAMPLE				0x45
#define AD4052_MAX_REG					0x45

#define AD4052_SPI_VENDOR	0x0456

#define AD4050_MAX_AVG		0x7
#define AD4052_MAX_AVG		0xB
#define AD4052_MAX_RATE(x)	((x) == AD4052_500KSPS ? 500000 : 2000000)
#define AD4052_FS_OFFSET(g)	((g) == AD4052_500KSPS ? 2 : 0)
#define AD4052_FS(g)		(&ad4052_conversion_freqs[AD4052_FS_OFFSET(g)])
#define AD4052_FS_LEN(g)	(ARRAY_SIZE(ad4052_conversion_freqs) - (AD4052_FS_OFFSET(g)))
#define AD4052_MON_VAL_MAX_GAIN		1999970
#define AD4052_MON_VAL_MIDDLE_POINT	0x8000
#define AD4052_T_CNVH_NS	10
#define AD4052_VIO_3V3		3300000
#define AD4052_SPI_MAX_ADC_XFER_SPEED(x)	((x) >= AD4052_VIO_3V3 ? 83333333 : 58823529)
#define AD4052_SPI_MAX_REG_XFER_SPEED		16000000

enum ad4052_grade {
	AD4052_2MSPS,
	AD4052_500KSPS,
};

enum ad4052_operation_mode {
	AD4052_SAMPLE_MODE = 0,
	AD4052_BURST_AVERAGING_MODE = 1,
	AD4052_MONITOR_MODE = 3,
};

enum ad4052_gp_mode {
	AD4052_GP_DISABLED,
	AD4052_GP_INTR,
	AD4052_GP_DRDY,
};

enum ad4052_interrupt_en {
	AD4052_INTR_EN_NEITHER,
	AD4052_INTR_EN_MIN,
	AD4052_INTR_EN_MAX,
	AD4052_INTR_EN_EITHER,
};

struct ad4052_chip_info {
	const struct iio_chan_spec channels;
	const struct iio_chan_spec offload_channels;
	const char *name;
	u16 prod_id;
	u8 max_avg;
	u8 grade;
};

enum {
	AD4052_SCAN_TYPE_SAMPLE,
	AD4052_SCAN_TYPE_BURST_AVG,
};

static const struct iio_scan_type ad4052_scan_type_12_s[] = {
	[AD4052_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.endianness = IIO_CPU,
	},
	[AD4052_SCAN_TYPE_BURST_AVG] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.endianness = IIO_CPU,
	},
};

static const struct iio_scan_type ad4052_scan_type_16_s[] = {
	[AD4052_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 32,
		.endianness = IIO_CPU,
	},
	[AD4052_SCAN_TYPE_BURST_AVG] = {
		.sign = 's',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_CPU,
	},
};

struct ad4052_state {
	const struct ad4052_bus_ops *ops;
	const struct ad4052_chip_info *chip;
	enum ad4052_operation_mode mode;
	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_device *spi;
	struct spi_transfer offload_xfer;
	struct spi_message offload_msg;
	struct pwm_device *cnv_pwm;
	struct pwm_state pwm_st;
	struct spi_transfer xfer;
	struct gpio_desc *cnv_gp;
	struct completion completion;
	struct regmap *regmap;
	u16 oversampling_frequency;
	u16 events_frequency;
	bool wait_event;
	int drdy_irq;
	int vio_uv;
	int vref_uv;
	u8 raw[4] __aligned(IIO_DMA_MINALIGN);
	u8 reg_tx[4];
	u8 reg_rx[4];
	u8 buf_reset_pattern[18];
};

static int ad4052_spi_read(void *context, const void *reg, size_t reg_size,
			   void *val, size_t val_size)
{
	int ret;
	struct ad4052_state *st = context;
	struct spi_transfer xfer = {
		.tx_buf = st->reg_tx,
		.rx_buf = st->reg_rx,
		.len = reg_size + val_size,
		.speed_hz = AD4052_SPI_MAX_REG_XFER_SPEED,
	};

	if (xfer.len > sizeof(st->reg_tx) ||
	    xfer.len > sizeof(st->reg_rx))
		return  -EINVAL;

	memset(st->reg_tx, 0, sizeof(st->reg_tx));
	memcpy(st->reg_tx, reg, reg_size);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(val, &st->reg_rx[reg_size], val_size);

	return 0;
}

static int ad4052_spi_write(void *context, const void *data, size_t count)
{
	struct ad4052_state *st = context;
	struct spi_transfer xfer = {
		.tx_buf = st->reg_tx,
		.len = count,
		.speed_hz = AD4052_SPI_MAX_REG_XFER_SPEED,
	};

	if (count > sizeof(st->reg_tx))
		return  -EINVAL;

	memcpy(st->reg_tx, data, count);

	return spi_sync_transfer(st->spi, &xfer, 1);
}

/* To limit the configuration mode access speed */
static const struct regmap_bus ad4052_regmap_bus = {
	.read = ad4052_spi_read,
	.write = ad4052_spi_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_range ad4052_regmap_rd_ranges[] = {
	regmap_reg_range(AD4052_REG_INTERFACE_CONFIG_A, AD4052_REG_DEVICE_GRADE),
	regmap_reg_range(AD4052_REG_SCRATCH_PAD, AD4052_REG_INTERFACE_STATUS),
	regmap_reg_range(AD4052_REG_MODE_SET, AD4052_REG_MON_VAL),
	regmap_reg_range(AD4052_REG_FUSE_CRC, AD4052_REG_MIN_SAMPLE),
};

static const struct regmap_access_table ad4052_regmap_rd_table = {
	.yes_ranges = ad4052_regmap_rd_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad4052_regmap_rd_ranges),
};

static const struct regmap_range ad4052_regmap_wr_ranges[] = {
	regmap_reg_range(AD4052_REG_INTERFACE_CONFIG_A, AD4052_REG_DEVICE_CONFIG),
	regmap_reg_range(AD4052_REG_SCRATCH_PAD, AD4052_REG_SCRATCH_PAD),
	regmap_reg_range(AD4052_REG_STREAM_MODE, AD4052_REG_INTERFACE_STATUS),
	regmap_reg_range(AD4052_REG_MODE_SET, AD4052_REG_MON_VAL),
	regmap_reg_range(AD4052_REG_FUSE_CRC, AD4052_REG_DEVICE_STATUS),
};

static const struct regmap_access_table ad4052_regmap_wr_table = {
	.yes_ranges = ad4052_regmap_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad4052_regmap_wr_ranges),
};

static const struct iio_event_spec ad4052_events[] = {
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

static const char *const ad4052_conversion_freqs[] = {
	"2000000", "1000000", "300000", "100000",	/*  0 -  3 */
	"33300", "10000", "3000", "500",		/*  4 -  7 */
	"333", "250", "200", "166",			/*  8 - 11 */
	"140", "124", "111",				/* 12 - 15 */
};

static int ad4052_conversion_frequency_set(struct ad4052_state *st, u8 val)
{
	val += AD4052_FS_OFFSET(st->chip->grade);
	return regmap_write(st->regmap, AD4052_REG_TIMER_CONFIG,
			    FIELD_PREP(AD4052_REG_TIMER_CONFIG_FS_MASK, val));
}

static int ad4052_oversampling_frequency_get(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	return st->oversampling_frequency - AD4052_FS_OFFSET(st->chip->grade);
}

static int ad4052_oversampling_frequency_set(struct iio_dev *indio_dev,
					     const struct iio_chan_spec *chan,
					     unsigned int val)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	val += AD4052_FS_OFFSET(st->chip->grade);
	st->oversampling_frequency = val;

	return 0;
}

static const struct iio_enum AD4052_500KSPS_conversion_freq_enum = {
	.items = AD4052_FS(AD4052_500KSPS),
	.num_items = AD4052_FS_LEN(AD4052_500KSPS),
	.set = ad4052_oversampling_frequency_set,
	.get = ad4052_oversampling_frequency_get,
};

static const struct iio_enum AD4052_2MSPS_conversion_freq_enum = {
	.items = AD4052_FS(AD4052_2MSPS),
	.num_items = AD4052_FS_LEN(AD4052_2MSPS),
	.set = ad4052_oversampling_frequency_set,
	.get = ad4052_oversampling_frequency_get,
};

#define AD4052_EXT_INFO(grade)						\
static struct iio_chan_spec_ext_info grade##_ext_info[] = {		\
	IIO_ENUM("oversampling_frequency", IIO_SHARED_BY_ALL,		\
		 &grade##_conversion_freq_enum),			\
	IIO_ENUM_AVAILABLE("oversampling_frequency", IIO_SHARED_BY_ALL,	\
			   &grade##_conversion_freq_enum),		\
	{ }								\
}

AD4052_EXT_INFO(AD4052_2MSPS);
AD4052_EXT_INFO(AD4052_500KSPS);

#define AD4052_CHAN(bits, grade) {							\
	.type = IIO_VOLTAGE,								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW) |				\
				    BIT(IIO_CHAN_INFO_SCALE) |				\
				    BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
				    BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),	\
	.indexed = 1,									\
	.channel = 0,									\
	.event_spec = ad4052_events,							\
	.num_event_specs = ARRAY_SIZE(ad4052_events),					\
	.has_ext_scan_type = 1,								\
	.ext_scan_type = ad4052_scan_type_##bits##_s,					\
	.num_ext_scan_type = ARRAY_SIZE(ad4052_scan_type_##bits##_s),			\
	.ext_info = grade##_ext_info,							\
}

#define AD4052_OFFLOAD_CHAN(bits, grade) {						\
	.type = IIO_VOLTAGE,								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW) |				\
				    BIT(IIO_CHAN_INFO_SCALE) |				\
				    BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
				    BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),	\
	.indexed = 1,									\
	.channel = 0,									\
	.event_spec = ad4052_events,							\
	.num_event_specs = ARRAY_SIZE(ad4052_events),					\
	.has_ext_scan_type = 1,								\
	.ext_scan_type = ad4052_scan_type_##bits##_s,					\
	.num_ext_scan_type = ARRAY_SIZE(ad4052_scan_type_##bits##_s),			\
	.ext_info = grade##_ext_info,							\
}

static const struct ad4052_chip_info ad4050_chip_info = {
	.name = "ad4050",
	.channels = AD4052_CHAN(12, AD4052_2MSPS),
	.offload_channels = AD4052_OFFLOAD_CHAN(12, AD4052_2MSPS),
	.prod_id = 0x70,
	.max_avg = AD4050_MAX_AVG,
	.grade = AD4052_2MSPS,
};

static const struct ad4052_chip_info ad4052_chip_info = {
	.name = "ad4052",
	.channels = AD4052_CHAN(16, AD4052_2MSPS),
	.offload_channels = AD4052_OFFLOAD_CHAN(16, AD4052_2MSPS),
	.prod_id = 0x72,
	.max_avg = AD4052_MAX_AVG,
	.grade = AD4052_2MSPS,
};

static const struct ad4052_chip_info ad4056_chip_info = {
	.name = "ad4056",
	.channels = AD4052_CHAN(12, AD4052_500KSPS),
	.offload_channels = AD4052_OFFLOAD_CHAN(12, AD4052_500KSPS),
	.prod_id = 0x76,
	.max_avg = AD4050_MAX_AVG,
	.grade = AD4052_500KSPS,
};

static const struct ad4052_chip_info ad4058_chip_info = {
	.name = "ad4058",
	.channels = AD4052_CHAN(16, AD4052_500KSPS),
	.offload_channels = AD4052_OFFLOAD_CHAN(16, AD4052_500KSPS),
	.prod_id = 0x78,
	.max_avg = AD4052_MAX_AVG,
	.grade = AD4052_500KSPS,
};

static ssize_t ad4052_events_frequency_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct ad4052_state *st = iio_priv(dev_to_iio_dev(dev));

	return sysfs_emit(buf, "%s\n", ad4052_conversion_freqs[st->events_frequency]);
}

static ssize_t ad4052_events_frequency_store(struct device *dev,
					     struct device_attribute *attr,
					     const char *buf,
					     size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	ret = __sysfs_match_string(AD4052_FS(st->chip->grade),
				   AD4052_FS_LEN(st->chip->grade), buf);
	if (ret < 0)
		goto out_release;

	st->events_frequency = ret;

out_release:
	iio_device_release_direct(indio_dev);
	return ret ? ret : len;
}

static IIO_DEVICE_ATTR(sampling_frequency, 0644,
		       ad4052_events_frequency_show,
		       ad4052_events_frequency_store, 0);

static ssize_t sampling_frequency_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct ad4052_state *st = iio_priv(dev_to_iio_dev(dev));
	int ret = 0;

	for (u8 i = AD4052_FS_OFFSET(st->chip->grade);
	     i < AD4052_FS_LEN(st->chip->grade); i++)
		ret += sysfs_emit_at(buf, ret, "%s ", ad4052_conversion_freqs[i]);

	ret += sysfs_emit_at(buf, ret, "\n");
	return ret;
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static struct attribute *ad4052_event_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad4052_event_attribute_group = {
	.attrs = ad4052_event_attributes,
};

static int ad4052_update_xfer_raw(struct iio_dev *indio_dev,
				   struct iio_chan_spec const *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	struct spi_transfer *xfer = &st->xfer;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	xfer->rx_buf = st->raw;
	xfer->bits_per_word = scan_type->realbits;
	xfer->len = scan_type->realbits == 24 ? 4 : 2;
	xfer->speed_hz = AD4052_SPI_MAX_ADC_XFER_SPEED(st->vio_uv);

	return 0;
}

static int ad4052_update_xfer_offload(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	struct spi_transfer *xfer = &st->offload_xfer;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	xfer->bits_per_word = scan_type->realbits;
	xfer->offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
	xfer->len = spi_bpw_to_bytes(scan_type->realbits);
	xfer->speed_hz = AD4052_SPI_MAX_ADC_XFER_SPEED(st->vio_uv);

	spi_message_init_with_transfers(&st->offload_msg, &st->offload_xfer, 1);
	st->offload_msg.offload = st->offload;

	return 0;
}

static int ad4052_set_oversampling_ratio(struct iio_dev *indio_dev,
					 const struct iio_chan_spec *chan,
					 unsigned int val)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	if (val < 1 || val > BIT(st->chip->max_avg + 1))
		return -EINVAL;

	/* 1 disables oversampling */
	if (val == 1) {
		st->mode = AD4052_SAMPLE_MODE;
	} else {
		val = ilog2(val);
		st->mode = AD4052_BURST_AVERAGING_MODE;
		ret = regmap_write(st->regmap, AD4052_REG_AVG_CONFIG, val - 1);
		if (ret)
			return ret;
	}

	return ad4052_update_xfer_raw(indio_dev, chan);
}

static int ad4052_get_oversampling_ratio(struct ad4052_state *st,
					 unsigned int *val)
{
	int ret, buf;

	if (st->mode == AD4052_SAMPLE_MODE) {
		*val = 1;
		return 0;
	}

	ret = regmap_read(st->regmap, AD4052_REG_AVG_CONFIG, &buf);
	if (ret)
		return ret;

	*val = BIT(buf + 1);

	return 0;
}

static int ad4052_check_ids(struct ad4052_state *st)
{
	int ret;
	u16 val;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_PROD_ID_1, &st->raw, 2);
	if (ret)
		return ret;

	val = get_unaligned_be16(st->raw);
	if (val != st->chip->prod_id)
		dev_warn(&st->spi->dev,
			 "Production ID x%x does not match known values", val);

	ret = regmap_bulk_read(st->regmap, AD4052_REG_VENDOR_H, &st->raw, 2);
	if (ret)
		return ret;

	val = get_unaligned_be16(st->raw);
	if (val != AD4052_SPI_VENDOR)
		return -ENODEV;

	return 0;
}

static int ad4052_exit_command(struct ad4052_state *st)
{
	struct spi_device *spi = st->spi;
	const u8 val = 0xA8;

	return spi_write_then_read(spi, &val, 1, NULL, 0);
}

static int ad4052_set_operation_mode(struct ad4052_state *st,
				     enum ad4052_operation_mode mode)
{
	int ret;

	if (mode == AD4052_BURST_AVERAGING_MODE) {
		ret = ad4052_conversion_frequency_set(st, st->oversampling_frequency);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, AD4052_REG_ADC_MODES,
				 AD4052_REG_ADC_MODES_MODE_MSK, mode);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD4052_REG_MODE_SET,
			    AD4052_REG_MODE_SET_ENTER_ADC);
}

static int ad4052_set_sampling_freq(struct ad4052_state *st, unsigned int freq)
{
	const u32 start = 1;

	if (!in_range(freq, start, AD4052_MAX_RATE(st->chip->grade)))
		return -EINVAL;

	st->pwm_st.period = DIV_ROUND_UP_ULL(NSEC_PER_SEC, freq);
	return pwm_apply_might_sleep(st->cnv_pwm, &st->pwm_st);
}

static int ad4052_soft_reset(struct ad4052_state *st)
{
	int ret;

	memset(st->buf_reset_pattern, 0xFF, sizeof(st->buf_reset_pattern));
	st->buf_reset_pattern[5] = 0xFE;
	st->buf_reset_pattern[11] = 0xFE;
	st->buf_reset_pattern[17] = 0xFE;

	ret = spi_write(st->spi, st->buf_reset_pattern,
			sizeof(st->buf_reset_pattern));
	if (ret)
		return ret;

	/* Wait AD4052 reset delay */
	fsleep(5000);

	return 0;
}

static int ad4052_setup(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			const bool *ref_sel, const int drdy_gp)
{
	u8 gp0_role = !drdy_gp ? AD4052_GP_DRDY : AD4052_GP_INTR;
	u8 gp1_role =  drdy_gp ? AD4052_GP_DRDY : AD4052_GP_INTR;
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	int ret;
	u8 val;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	val = FIELD_PREP(AD4052_REG_GP_CONF_MODE_MSK_0, gp0_role) |
	      FIELD_PREP(AD4052_REG_GP_CONF_MODE_MSK_1, gp1_role);

	ret = regmap_update_bits(st->regmap, AD4052_REG_GP_CONF,
				 AD4052_REG_GP_CONF_MODE_MSK_1 | AD4052_REG_GP_CONF_MODE_MSK_0,
				 val);
	if (ret)
		return ret;

	if (st->chip->grade == AD4052_500KSPS) {
		ret = regmap_write(st->regmap, AD4052_REG_TIMER_CONFIG,
				   FIELD_PREP(AD4052_REG_TIMER_CONFIG_FS_MASK,
					      AD4052_REG_TIMER_CONFIG_300KSPS));
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, AD4052_REG_ADC_MODES,
				 AD4052_REG_ADC_CONFIG_REF_EN_MSK,
				 FIELD_PREP(AD4052_REG_ADC_CONFIG_REF_EN_MSK,
					    *ref_sel));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4052_REG_DEVICE_STATUS,
			   AD4052_REG_DEVICE_STATUS_DEVICE_RESET);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4052_REG_INTR_CONF_EN_MSK_0, (AD4052_INTR_EN_EITHER)) |
	      FIELD_PREP(AD4052_REG_INTR_CONF_EN_MSK_1, (AD4052_INTR_EN_EITHER));
	return regmap_update_bits(st->regmap, AD4052_REG_INTR_CONF,
				  AD4052_REG_INTR_CONF_EN_MSK_0 | AD4052_REG_INTR_CONF_EN_MSK_1,
				  val);
}

static irqreturn_t ad4052_irq_handler_thresh(int irq, void *private)
{
	struct iio_dev *indio_dev = private;

	iio_push_event(indio_dev,
		       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, 0,
					    IIO_EV_TYPE_THRESH,
					    IIO_EV_DIR_EITHER),
		       iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static irqreturn_t ad4052_irq_handler_drdy(int irq, void *private)
{
	struct ad4052_state *st = private;

	complete(&st->completion);
	return IRQ_HANDLED;
}

static int ad4052_request_irq(struct iio_dev *indio_dev, const int drdy_gp)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	int ret;
	char gp_name[4] = "gp_\n";

	gp_name[3] = drdy_gp ? '1' : '0';
	ret = fwnode_irq_get_byname(dev_fwnode(&st->spi->dev), gp_name);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret >= 0) {
		ret = devm_request_threaded_irq(dev, ret, NULL,
						ad4052_irq_handler_drdy,
						IRQF_ONESHOT, indio_dev->name,
						st);
		if (ret)
			return ret;
		st->drdy_irq = ret;
	}

	gp_name[3] = !drdy_gp ? '0' : '1';
	ret = fwnode_irq_get_byname(dev_fwnode(&st->spi->dev), gp_name);
	if (ret == -EPROBE_DEFER)
		return ret;
	if (ret >= 0) {
		ret = devm_request_threaded_irq(dev, ret, NULL,
						ad4052_irq_handler_thresh,
						IRQF_ONESHOT, indio_dev->name,
						indio_dev);
		if (ret)
			return ret;
	}

	return 0;
}

static const int ad4052_oversampling_avail[] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096,
};

static int ad4052_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *len, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*vals = ad4052_oversampling_avail;
		*len = ARRAY_SIZE(ad4052_oversampling_avail);
		*type = IIO_VAL_INT;

		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int ad4052_get_samp_freq(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val,
				int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct pwm_state pwm_st;
	int ret;

	ret = pwm_get_state_hw(st->cnv_pwm, &pwm_st);
	if (ret)
		return ret;

	if (!pwm_st.enabled)
		pwm_st = st->pwm_st;

	*val = DIV_ROUND_UP_ULL(NSEC_PER_SEC, pwm_st.period);
	return IIO_VAL_INT;
}

static int ad4052_get_chan_scale(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val,
				 int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;

	scan_type = iio_get_current_scan_type(indio_dev, &st->chip->channels);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	*val = (st->vref_uv * 2) / MILLI;

	*val2 = scan_type->realbits;

	return IIO_VAL_FRACTIONAL_LOG2;
}

static int ad4052_get_chan_calibscale(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int *val,
				      int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u16 gain;
	int ret;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_MON_VAL, &st->raw, 2);
	if (ret)
		return ret;

	gain = get_unaligned_be16(&st->raw);

	/* From datasheet: code out = code in × mon_val/0x8000 */
	*val = gain / AD4052_MON_VAL_MIDDLE_POINT;
	*val2 = mul_u64_u32_div(gain % AD4052_MON_VAL_MIDDLE_POINT, NANO,
				AD4052_MON_VAL_MIDDLE_POINT);

	return IIO_VAL_INT_PLUS_NANO;
}

static int ad4052_set_chan_calibscale(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      int gain_int,
				      int gain_frac)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u64 gain;
	int ret;

	if (gain_int < 0 || gain_frac < 0)
		return -EINVAL;

	gain = mul_u32_u32(gain_int, MICRO) + gain_frac;

	if (gain > AD4052_MON_VAL_MAX_GAIN)
		return -EINVAL;

	put_unaligned_be16(DIV_ROUND_CLOSEST_ULL(gain * AD4052_MON_VAL_MIDDLE_POINT,
						 MICRO),
			   &st->raw);

	ret = regmap_bulk_write(st->regmap, AD4052_REG_MON_VAL, &st->raw, 2);
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, AD4052_REG_ADC_MODES,
				  AD4052_REG_ADC_CONFIG_SCALE_EN_MSK,
				  FIELD_PREP(AD4052_REG_ADC_CONFIG_SCALE_EN_MSK,
					     !(gain_int == 1 && gain_frac == 0)));
}

static int __ad4052_read_chan_raw(struct ad4052_state *st, int *val)
{
	struct spi_device *spi = st->spi;
	struct spi_transfer t_cnv = {};
	int ret;

	reinit_completion(&st->completion);

	if (st->cnv_gp) {
		gpiod_set_value_cansleep(st->cnv_gp, 1);
		gpiod_set_value_cansleep(st->cnv_gp, 0);
	} else {
		/* CNV and CS tied together */
		ret = spi_sync_transfer(spi, &t_cnv, 1);
		if (ret)
			return ret;
	}
	/*
	 * Single sample read should be used only for oversampling and
	 * sampling frequency pairs that take less than 1 sec.
	 */
	if (st->drdy_irq) {
		ret = wait_for_completion_timeout(&st->completion,
						  msecs_to_jiffies(1000));
		if (!ret)
			return -ETIMEDOUT;
	}

	ret = spi_sync_transfer(spi, &st->xfer, 1);
	if (ret)
		return ret;

	if (st->xfer.len == 2)
		*val = sign_extend32(*(u16 *)(st->raw), 15);
	else
		*val = sign_extend32(*(u32 *)(st->raw), 23);

	return ret;
}

static int ad4052_read_chan_raw(struct iio_dev *indio_dev, int *val)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	ret = ad4052_set_operation_mode(st, st->mode);
	if (ret)
		goto out_error;

	ret = __ad4052_read_chan_raw(st, val);
	if (ret)
		goto out_error;

	ret = ad4052_exit_command(st);

out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_read_raw_dispatch(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, int *val,
				    int *val2, long info)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4052_read_chan_raw(indio_dev, val);

	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4052_get_chan_calibscale(indio_dev, chan, val, val2);

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4052_get_oversampling_ratio(st, val);

	default:
		return -EINVAL;
	}
}

static int ad4052_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad4052_get_chan_scale(indio_dev, chan, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4052_get_samp_freq(indio_dev, chan, val, val2);
	}

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	ret = ad4052_read_raw_dispatch(indio_dev, chan, val, val2, info);

out_release:
	iio_device_release_direct(indio_dev);
	return ret ? ret : IIO_VAL_INT;
}

static int ad4052_write_raw_dispatch(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int val,
				     int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4052_set_oversampling_ratio(indio_dev, chan, val);

	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4052_set_chan_calibscale(indio_dev, chan, val, val2);

	default:
		return -EINVAL;
	}
};

static int ad4052_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4052_set_sampling_freq(st, val);
	}

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	ret = ad4052_write_raw_dispatch(indio_dev, chan, val, val2, info);

out_release:
	iio_device_release_direct(indio_dev);
	return ret;
}

static int ad4052_monitor_mode_enable(struct ad4052_state *st)
{
	int ret;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	ret = ad4052_conversion_frequency_set(st, st->events_frequency);
	if (ret)
		goto out_error;

	ret = ad4052_set_operation_mode(st, AD4052_MONITOR_MODE);
	if (ret)
		goto out_error;

	return ret;
out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_monitor_mode_disable(struct ad4052_state *st)
{
	int ret;

	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);

	ret = ad4052_exit_command(st);
	if (ret)
		return ret;
	return regmap_write(st->regmap, AD4052_REG_DEVICE_STATUS,
			    AD4052_REG_DEVICE_STATUS_MAX_FLAG |
			    AD4052_REG_DEVICE_STATUS_MIN_FLAG);
}

static int ad4052_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	return st->wait_event;
}

static int ad4052_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     bool state)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret = 0;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;
	if (st->wait_event == state) {
		goto out_release;
	}

	if (state)
		ret = ad4052_monitor_mode_enable(st);
	else
		ret = ad4052_monitor_mode_disable(st);

	if (!ret)
		st->wait_event = state;

out_release:
	iio_device_release_direct(indio_dev);
	return ret;
}

static int ad4052_read_event_info_value(struct ad4052_state *st,
					enum iio_event_direction dir, int *val)
{
	int ret;
	u8 reg;

	if (dir == IIO_EV_DIR_RISING)
		reg = AD4052_REG_MAX_LIMIT;
	else
		reg = AD4052_REG_MIN_LIMIT;

	ret = regmap_bulk_read(st->regmap, reg, &st->raw, 2);
	if (ret)
		return ret;

	*val = sign_extend32(get_unaligned_be16(st->raw), 11);

	return 0;
}

static int ad4052_read_event_info_hysteresis(struct ad4052_state *st,
					     enum iio_event_direction dir, int *val)
{
	u8 reg;

	if (dir == IIO_EV_DIR_RISING)
		reg = AD4052_REG_MAX_HYST;
	else
		reg = AD4052_REG_MIN_HYST;
	return regmap_read(st->regmap, reg, val);
}

static int ad4052_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	if (st->wait_event) {
		ret = -EBUSY;
		goto out_release;
	}

	switch (info) {
	case IIO_EV_INFO_VALUE:
		ret = ad4052_read_event_info_value(st, dir, val);
		break;
	case IIO_EV_INFO_HYSTERESIS:
		ret = ad4052_read_event_info_hysteresis(st, dir, val);
		break;
	default:
		ret = -EINVAL;
		break;
	}

out_release:
	iio_device_release_direct(indio_dev);
	return ret ? ret : IIO_VAL_INT;
}

static int ad4052_write_event_info_value(struct ad4052_state *st,
					 enum iio_event_direction dir, int val)
{
	u8 reg;

	if (val > 2047 || val < -2048)
		return -EINVAL;
	if (dir == IIO_EV_DIR_RISING)
		reg = AD4052_REG_MAX_LIMIT;
	else
		reg = AD4052_REG_MIN_LIMIT;
	put_unaligned_be16(val, &st->raw);

	return regmap_bulk_write(st->regmap, reg, &st->raw, 2);
}

static int ad4052_write_event_info_hysteresis(struct ad4052_state *st,
					      enum iio_event_direction dir, int val)
{
	u8 reg;

	if (val >= BIT(7))
		return -EINVAL;
	if (dir == IIO_EV_DIR_RISING)
		reg = AD4052_REG_MAX_HYST;
	else
		reg = AD4052_REG_MIN_HYST;

	return regmap_write(st->regmap, reg, val);
}

static int ad4052_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val,
				    int val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
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
			ret = ad4052_write_event_info_value(st, dir, val);
			break;
		case IIO_EV_INFO_HYSTERESIS:
			ret = ad4052_write_event_info_hysteresis(st, dir, val);
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

static int ad4052_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct spi_offload_trigger_config config = {
		.type = SPI_OFFLOAD_TRIGGER_DATA_READY,
	};
	int ret;

	if (st->wait_event)
		return -EBUSY;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	ret = ad4052_set_operation_mode(st, st->mode);
	if (ret)
		goto out_mode_error;

	ret = ad4052_update_xfer_offload(indio_dev, indio_dev->channels);
	if (ret)
		goto out_xfer_error;

	ret = spi_optimize_message(st->spi, &st->offload_msg);
	if (ret)
		goto out_xfer_error;

	/* SPI Offload handles the data ready irq */
	if (st->drdy_irq)
		disable_irq(st->drdy_irq);

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger,
					 &config);
	if (ret)
		goto out_offload_error;

	st->pwm_st.enabled = true;
	ret = pwm_apply_might_sleep(st->cnv_pwm, &st->pwm_st);
	if (ret)
		goto out_pwm_error;

	return 0;

out_pwm_error:
	spi_offload_trigger_disable(st->offload, st->offload_trigger);
out_offload_error:
	if (st->drdy_irq)
		enable_irq(st->drdy_irq);
	spi_unoptimize_message(&st->offload_msg);
out_xfer_error:
	ad4052_exit_command(st);
out_mode_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);

	return ret;
}

static int ad4052_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	st->pwm_st.enabled = false;
	pwm_apply_might_sleep(st->cnv_pwm, &st->pwm_st);

	spi_offload_trigger_disable(st->offload, st->offload_trigger);
	spi_unoptimize_message(&st->offload_msg);

	ret = ad4052_exit_command(st);

	if (st->drdy_irq)
		enable_irq(st->drdy_irq);
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);

	return ret;
}

static const struct iio_buffer_setup_ops ad4052_buffer_offload_setup_ops = {
	.postenable = &ad4052_offload_buffer_postenable,
	.predisable = &ad4052_offload_buffer_predisable,
};

static int ad4052_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				     unsigned int writeval, unsigned int *readval)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	else
		return regmap_write(st->regmap, reg, writeval);
}

static int ad4052_get_current_scan_type(const struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	return st->mode == AD4052_BURST_AVERAGING_MODE ?
			   AD4052_SCAN_TYPE_BURST_AVG :
			   AD4052_SCAN_TYPE_SAMPLE;
}

static const struct iio_info ad4052_info = {
	.read_raw = ad4052_read_raw,
	.write_raw = ad4052_write_raw,
	.read_avail = ad4052_read_avail,
	.read_event_config = &ad4052_read_event_config,
	.write_event_config = &ad4052_write_event_config,
	.read_event_value = &ad4052_read_event_value,
	.write_event_value = &ad4052_write_event_value,
	.event_attrs = &ad4052_event_attribute_group,
	.get_current_scan_type = &ad4052_get_current_scan_type,
	.debugfs_reg_access = &ad4052_debugfs_reg_access,
};

static const struct regmap_config ad4052_regmap_config = {
	.name = "ad4052",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AD4052_MAX_REG,
	.rd_table = &ad4052_regmap_rd_table,
	.wr_table = &ad4052_regmap_wr_table,
	.read_flag_mask = BIT(7),
	.can_sleep = true,
};

static const struct spi_offload_config ad4052_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

static void ad4052_pwm_disable(void *pwm)
{
	pwm_disable(pwm);
}

static bool ad4052_offload_trigger_match(struct spi_offload_trigger *trigger,
					 enum spi_offload_trigger_type type,
					 u64 *args, u32 nargs)
{
	return type == SPI_OFFLOAD_TRIGGER_DATA_READY;
}

static const struct spi_offload_trigger_ops ad4052_offload_trigger_ops = {
	.match = ad4052_offload_trigger_match,
};

static int ad4052_request_offload(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct dma_chan *rx_dma;
	struct spi_offload_trigger_info trigger_info = {
		.fwnode = dev_fwnode(dev),
		.ops = &ad4052_offload_trigger_ops,
		.priv = st,
	};
	int ret;

	indio_dev->setup_ops = &ad4052_buffer_offload_setup_ops;

	ret = devm_spi_offload_trigger_register(dev, &trigger_info);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register offload trigger\n");

	st->offload_trigger = devm_spi_offload_trigger_get(dev, st->offload,
							   SPI_OFFLOAD_TRIGGER_DATA_READY);
	if (IS_ERR(st->offload_trigger))
		return PTR_ERR(st->offload_trigger);

	st->cnv_pwm = devm_pwm_get(dev, NULL);
	if (IS_ERR(st->cnv_pwm))
		return dev_err_probe(dev, PTR_ERR(st->cnv_pwm), "failed to get CNV PWM\n");

	pwm_init_state(st->cnv_pwm, &st->pwm_st);

	st->pwm_st.enabled = false;
	st->pwm_st.duty_cycle = AD4052_T_CNVH_NS * 2;
	st->pwm_st.period = DIV_ROUND_UP_ULL(NSEC_PER_SEC, AD4052_MAX_RATE(st->chip->grade));

	ret = pwm_apply_might_sleep(st->cnv_pwm, &st->pwm_st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to apply CNV PWM\n");

	ret = devm_add_action_or_reset(dev, ad4052_pwm_disable, st->cnv_pwm);
	if (ret)
		return ret;

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(rx_dma))
		return PTR_ERR(rx_dma);

	return devm_iio_dmaengine_buffer_setup_with_handle(dev, indio_dev, rx_dma,
							   IIO_BUFFER_DIRECTION_IN);
}

static int ad4052_validate_parent_trigger_sources(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct of_phandle_args trigger_sources;
	struct device_node *np;
	int ret;

	np = of_get_parent(st->spi->dev.of_node);
	if (!np)
		return -ENODEV;

	ret = of_parse_phandle_with_args(np, "trigger-sources",
					 "#trigger-source-cells", 0,
					 &trigger_sources);
	if (ret)
		goto out_error;

	if (trigger_sources.args[0] != AD4052_TRIGGER_EVENT_DATA_READY)
		ret = -EINVAL;
	ret = trigger_sources.args[1];
	if (ret != AD4052_TRIGGER_PIN_GP0 || ret != AD4052_TRIGGER_PIN_GP1)
		ret = -EINVAL;
	of_node_put(trigger_sources.np);
out_error:
	of_node_put(np);
	return ret;
}

static int ad4052_regulators_get(struct ad4052_state *st, bool *ref_sel)
{
	struct device *dev = &st->spi->dev;
	int uv;

	st->vio_uv = devm_regulator_get_enable_read_voltage(dev, "vio");
	if (st->vio_uv < 0)
		return dev_err_probe(dev, st->vio_uv,
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

static void ad4052_monitor_mode_remove(void *ptr)
{
	struct ad4052_state *st = ptr;

	if (st->wait_event)
		ad4052_monitor_mode_disable(st);
}

static int ad4052_probe(struct spi_device *spi)
{
	int ret, drdy_gp = AD4052_TRIGGER_PIN_GP0;
	const struct ad4052_chip_info *chip;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4052_state *st;
	bool ref_sel;

	chip = spi_get_device_match_data(spi);
	if (!chip)
		return dev_err_probe(dev, -ENODEV,
				     "Could not find chip info data\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, st);
	init_completion(&st->completion);

	ret = ad4052_regulators_get(st, &ref_sel);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init(dev, &ad4052_regmap_bus, st,
				      &ad4052_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	st->mode = AD4052_SAMPLE_MODE;
	st->wait_event = false;
	st->chip = chip;
	st->oversampling_frequency = AD4052_FS_OFFSET(st->chip->grade);
	st->events_frequency = AD4052_FS_OFFSET(st->chip->grade);

	st->cnv_gp = devm_gpiod_get_optional(dev, "cnv", GPIOD_OUT_LOW);
	if (IS_ERR(st->cnv_gp))
		return dev_err_probe(dev, PTR_ERR(st->cnv_gp),
				     "Failed to get cnv gpio\n");

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 1;
	indio_dev->info = &ad4052_info;
	indio_dev->name = chip->name;

	st->offload = devm_spi_offload_get(dev, spi, &ad4052_offload_config);
	ret = PTR_ERR_OR_ZERO(st->offload);

	if (ret == -ENODEV) {
		st->offload_trigger = NULL;
		indio_dev->channels = &chip->channels;
	} else if (!ret) {
		indio_dev->channels = &chip->offload_channels;
		ret = ad4052_request_offload(indio_dev);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to configure offload\n");
		drdy_gp = ad4052_validate_parent_trigger_sources(indio_dev);
		if (drdy_gp < 0)
			return dev_err_probe(dev, drdy_gp,
					     "Failed to validate parent trigger sources\n");
	} else {
		return dev_err_probe(dev, ret, "Failed to get offload\n");
	}

	ret = ad4052_soft_reset(st);
	if (ret)
		return dev_err_probe(dev, ret, "AD4052 failed to soft reset\n");

	ret = ad4052_check_ids(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "AD4052 fields assertions failed\n");

	ret = ad4052_setup(indio_dev, indio_dev->channels, &ref_sel, drdy_gp);
	if (ret)
		return ret;

	ret = ad4052_request_irq(indio_dev, drdy_gp);
	if (ret)
		return ret;

	ret = ad4052_update_xfer_raw(indio_dev, indio_dev->channels);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable pm_runtime\n");

	ret = devm_add_action_or_reset(dev, ad4052_monitor_mode_remove, st);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	return devm_iio_device_register(dev, indio_dev);
}

static int ad4052_runtime_suspend(struct device *dev)
{
	struct ad4052_state *st = dev_get_drvdata(dev);

	return regmap_write(st->regmap, AD4052_REG_DEVICE_CONFIG,
			    FIELD_PREP(AD4052_REG_DEVICE_CONFIG_POWER_MODE_MSK,
				       AD4052_REG_DEVICE_CONFIG_LOW_POWER_MODE));
}

static int ad4052_runtime_resume(struct device *dev)
{
	struct ad4052_state *st = dev_get_drvdata(dev);
	int ret;

	ret = regmap_clear_bits(st->regmap, AD4052_REG_DEVICE_CONFIG,
				AD4052_REG_DEVICE_CONFIG_POWER_MODE_MSK);
	if (ret)
		return ret;

	fsleep(4000);
	return ret;
}

static DEFINE_RUNTIME_DEV_PM_OPS(ad4052_pm_ops, ad4052_runtime_suspend,
				 ad4052_runtime_resume, NULL);

static const struct spi_device_id ad4052_id_table[] = {
	{"ad4050", (kernel_ulong_t)&ad4050_chip_info },
	{"ad4052", (kernel_ulong_t)&ad4052_chip_info },
	{"ad4056", (kernel_ulong_t)&ad4056_chip_info },
	{"ad4058", (kernel_ulong_t)&ad4058_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4052_id_table);

static const struct of_device_id ad4052_of_match[] = {
	{ .compatible = "adi,ad4050", .data = &ad4050_chip_info },
	{ .compatible = "adi,ad4052", .data = &ad4052_chip_info },
	{ .compatible = "adi,ad4056", .data = &ad4056_chip_info },
	{ .compatible = "adi,ad4058", .data = &ad4058_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ad4052_of_match);

static struct spi_driver ad4052_driver = {
	.driver = {
		.name = "ad4052",
		.of_match_table = ad4052_of_match,
		.pm = pm_ptr(&ad4052_pm_ops),
	},
	.probe = ad4052_probe,
	.id_table = ad4052_id_table,
};
module_spi_driver(ad4052_driver);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4052");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_DMAENGINE_BUFFER");
