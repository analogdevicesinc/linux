// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD4052 SPI ADC driver
 *
 * Copyright 2025 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine-ex.h>
#include <linux/types.h>
#include <linux/units.h>
#include <linux/util_macros.h>

#define AD4052_REG_INTERFACE_CONFIG_A	0x00
#define AD4052_REG_DEVICE_CONFIG	0x02
#define AD4052_REG_PROD_ID_1		0x05
#define AD4052_REG_DEVICE_GRADE		0x06
#define AD4052_REG_SCRATCH_PAD		0x0A
#define AD4052_REG_VENDOR_H		0x0D
#define AD4052_REG_STREAM_MODE		0x0E
#define AD4052_REG_INTERFACE_STATUS	0x11
#define AD4052_REG_MODE_SET		0x20
#define AD4052_REG_ADC_MODES		0x21
#define AD4052_REG_AVG_CONFIG		0x23
#define AD4052_REG_GP_CONFIG		0x24
#define AD4052_REG_INTR_CONFIG		0x25
#define AD4052_REG_TIMER_CONFIG		0x27
#define AD4052_REG_MAX_LIMIT		0x29
#define AD4052_REG_MIN_LIMIT		0x2B
#define AD4052_REG_MAX_HYST		0x2C
#define AD4052_REG_MIN_HYST		0x2D
#define AD4052_REG_MON_VAL		0x2F
#define AD4052_REG_FUSE_CRC		0x40
#define AD4052_REG_DEVICE_STATUS	0x41
#define AD4052_REG_MIN_SAMPLE		0x45
#define AD4052_MAX_REG			0x45
/* GP_CONFIG */
#define AD4052_GP_MODE_MSK(x)		(GENMASK(2, 0) << (x) * 4)
#define AD4052_GP_MODE(x, y)		FIELD_PREP(AD4052_GP_MODE_MSK(x), (y))
/* INTR_CONFIG */
#define AD4052_INTR_EN_MSK(x)		(GENMASK(1, 0) << (x) * 4)
#define AD4052_INTR_EN(x, y)		FIELD_PREP(AD4052_INTR_EN_MSK(x), (y))
/* ADC_MODES */
#define AD4052_DATA_FORMAT		BIT(7)
/* DEVICE_CONFIG */
#define AD4052_POWER_MODE_MSK		GENMASK(1, 0)
#define AD4052_LOW_POWER_MODE		3
/* DEVICE_STATUS */
#define AD4052_DEVICE_RESET		BIT(6)
#define AD4052_THRESH_OVERRUN		BIT(4)
#define AD4052_MAX_FLAG			BIT(3)
#define AD4052_MIN_FLAG			BIT(2)
#define AD4052_EVENT_CLEAR		(AD4052_THRESH_OVERRUN | AD4052_MAX_FLAG | AD4052_MIN_FLAG)
/* TIMER_CONFIG */
#define AD4052_FS_MASK			GENMASK(7, 4)
#define AD4052_300KSPS			0x2

#define AD4052_SPI_VENDOR		0x0456

#define AD4050_MAX_AVG			0x7
#define AD4052_MAX_AVG			0xB
#define AD4052_CHECK_OVERSAMPLING(x, y)	({typeof(y) y_ = (y); \
					  ((y_) < 0 || (y_) > BIT((x) + 1)); })
#define AD4052_MAX_RATE(x)		((x) == AD4052_500KSPS ? 500000 : 2000000)
#define AD4052_CHECK_RATE(x, y)		({typeof(y) y_ = (y);				\
					  ((y_) > AD4052_MAX_RATE(x) || (y_) <= 0); })
#define AD4052_FS_OFFSET(g)		((g) == AD4052_500KSPS ? 2 : 0)
#define AD4052_FS(g)			(&ad4052_sample_rates[AD4052_FS_OFFSET(g)])
#define AD4052_FS_LEN(g)		(ARRAY_SIZE(ad4052_sample_rates) - (AD4052_FS_OFFSET(g)))

enum ad4052_device_type {
	ID_AD4050,
	ID_AD4056,
	ID_AD4052,
	ID_AD4058,
};

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
	const struct iio_chan_spec channels[1];
	const char *name;
	u16 prod_id;
	u8 max_avg;
	u8 grade;
};

enum {
	AD4052_SCAN_TYPE_SAMPLE,
	AD4052_SCAN_TYPE_BURST_AVG,
};

/* The family sign-extend bytes  */
static const struct iio_scan_type ad4052_scan_type_12_s[] = {
	[AD4052_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 16,
		.endianness = IIO_CPU,
	},
	[AD4052_SCAN_TYPE_BURST_AVG] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 16,
		.endianness = IIO_CPU,
	},
};

static const struct iio_scan_type ad4052_scan_type_16_s[] = {
	[AD4052_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 16,
		.storagebits = 16,
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
	struct spi_device *spi;
	struct spi_transfer xfer;
	struct spi_message msg;
	struct pwm_device *cnv_pwm;
	struct pwm_waveform cnv_wf;
	struct gpio_desc *cnv_gp;
	struct completion completion;
	struct regmap *regmap;
	bool wait_event;
	int gp1_irq;
	u8 data_format;
	union {
		__be16 d16;
		__be32 d32;
	} __aligned(IIO_DMA_MINALIGN);
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
		.mask_shared_by_all = BIT(IIO_EV_INFO_ENABLE)
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_HYSTERESIS)
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_shared_by_all = BIT(IIO_EV_INFO_VALUE) |
				      BIT(IIO_EV_INFO_HYSTERESIS)
	}
};

static const int ad4052_sample_rate_avail[] = {
	2000000, 1000000, 300000, 100000, 33300,
	10000, 3000, 500, 333, 250, 200,
	166, 140, 125, 111
};

static const char *const ad4052_sample_rates[] = {
	"2000000", "1000000", "300000", "100000", "33300",
	"10000", "3000", "500", "333", "250", "200",
	"166", "140", "124", "111",
};

static ssize_t ad4052_sample_rate_get(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		struct ad4052_state *st = iio_priv(indio_dev);
		int ret, val;

		ret = regmap_read(st->regmap, AD4052_REG_TIMER_CONFIG, &val);
		if (ret)
			return ret;

		val = FIELD_GET(AD4052_FS_MASK, val);

		return val - AD4052_FS_OFFSET(st->chip->grade);
	}
	unreachable();
}

static ssize_t ad4052_sample_rate_set(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int val)
{
	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		struct ad4052_state *st = iio_priv(indio_dev);

		val += AD4052_FS_OFFSET(st->chip->grade);
		val = FIELD_PREP(AD4052_FS_MASK, val);
		return regmap_write(st->regmap, AD4052_REG_TIMER_CONFIG, val);
	}
	unreachable();
}

static const struct iio_enum AD4052_500KSPS_sample_rate_enum = {
	.items = AD4052_FS(AD4052_500KSPS),
	.num_items = AD4052_FS_LEN(AD4052_500KSPS),
	.set = ad4052_sample_rate_set,
	.get = ad4052_sample_rate_get,
};

static const struct iio_enum AD4052_2MSPS_sample_rate_enum = {
	.items = AD4052_FS(AD4052_2MSPS),
	.num_items = AD4052_FS_LEN(AD4052_2MSPS),
	.set = ad4052_sample_rate_set,
	.get = ad4052_sample_rate_get,
};

#define AD4052_EXT_INFO(grade)								\
static struct iio_chan_spec_ext_info grade##_ext_info[] = {				\
	IIO_ENUM("sample_rate", IIO_SHARED_BY_ALL, &grade##_sample_rate_enum),		\
	IIO_ENUM_AVAILABLE("sample_rate", IIO_SHARED_BY_ALL, &grade##_sample_rate_enum),\
	{}										\
}

AD4052_EXT_INFO(AD4052_2MSPS);
AD4052_EXT_INFO(AD4052_500KSPS);

#define AD4052_CHAN(bits, grade) {							\
	.type = IIO_VOLTAGE,								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW) |				\
				    BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |		\
				    BIT(IIO_CHAN_INFO_SAMP_FREQ),			\
	.info_mask_shared_by_type_available =  BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),	\
	.indexed = 1,									\
	.channel = 0,									\
	.event_spec = ad4052_events,							\
	.num_event_specs = ARRAY_SIZE(ad4052_events),					\
	.has_ext_scan_type = 1,								\
	.ext_scan_type = ad4052_scan_type_##bits##_s,					\
	.num_ext_scan_type = ARRAY_SIZE(ad4052_scan_type_##bits##_s),			\
	.ext_info = grade##_ext_info,							\
}

const struct ad4052_chip_info ad4050_chip_info = {
	.name = "ad4050",
	.channels = { AD4052_CHAN(12, AD4052_2MSPS) },
	.prod_id = 0x70,
	.max_avg = AD4050_MAX_AVG,
	.grade = AD4052_2MSPS,
};

const struct ad4052_chip_info ad4052_chip_info = {
	.name = "ad4052",
	.channels = { AD4052_CHAN(16, AD4052_2MSPS) },
	.prod_id = 0x72,
	.max_avg = AD4052_MAX_AVG,
	.grade = AD4052_2MSPS,
};

const struct ad4052_chip_info ad4056_chip_info = {
	.name = "ad4056",
	.channels = { AD4052_CHAN(12, AD4052_500KSPS) },
	.prod_id = 0x70,
	.max_avg = AD4050_MAX_AVG,
	.grade = AD4052_500KSPS,
};

const struct ad4052_chip_info ad4058_chip_info = {
	.name = "ad4058",
	.channels = { AD4052_CHAN(16, AD4052_500KSPS) },
	.prod_id = 0x72,
	.max_avg = AD4052_MAX_AVG,
	.grade = AD4052_500KSPS,
};

static int ad4052_set_oversampling_ratio(struct ad4052_state *st,
					 unsigned int val)
{
	int ret = 0;

	if (AD4052_CHECK_OVERSAMPLING(st->chip->max_avg, val))
		return -EINVAL;

	/* 0 or 1 disables oversampling */
	if (val == 0 || val == 1) {
		st->mode = AD4052_SAMPLE_MODE;
	} else {
		val = ilog2(val);
		st->mode = AD4052_BURST_AVERAGING_MODE;
		ret = regmap_write(st->regmap, AD4052_REG_AVG_CONFIG, val - 1);
	}

	return ret;
}

static int ad4052_get_oversampling_ratio(struct ad4052_state *st,
					 unsigned int *val)
{
	int ret;

	if (st->mode == AD4052_SAMPLE_MODE) {
		*val = 0;
		return 0;
	}

	ret = regmap_read(st->regmap, AD4052_REG_AVG_CONFIG, val);
	if (ret)
		return ret;

	*val = BIT(*val + 1);

	return 0;
}

static int ad4052_assert(struct ad4052_state *st)
{
	int ret;
	u16 val;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_PROD_ID_1, &st->d16, 2);
	if (ret)
		return ret;

	val = be16_to_cpu(st->d16);
	if (val != st->chip->prod_id)
		return -ENODEV;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_VENDOR_H, &st->d16, 2);
	if (ret)
		return ret;

	val = be16_to_cpu(st->d16);
	if (val != AD4052_SPI_VENDOR)
		return -ENODEV;

	return 0;
}

static int ad4052_exit_command(struct ad4052_state *st)
{
	struct spi_device *spi = st->spi;
	const u8 val = 0xA8;

	return spi_write(spi, &val, 1);
}

static int ad4052_set_operation_mode(struct ad4052_state *st, enum ad4052_operation_mode mode)
{
	u8 val = st->data_format | mode;
	int ret;

	ret = regmap_write(st->regmap, AD4052_REG_ADC_MODES, val);
	if (ret)
		return ret;

	val = BIT(0);
	return regmap_write(st->regmap, AD4052_REG_MODE_SET, val);
}

static int __ad4052_set_sampling_freq(struct ad4052_state *st, unsigned int freq)
{
	int ret;
	struct pwm_waveform cnv_wf = {
		.duty_length_ns = 10,
	};

	cnv_wf.period_length_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq);
	ret = pwm_round_waveform_might_sleep(st->cnv_pwm, &cnv_wf);
	if (ret)
		return ret;

	st->cnv_wf = cnv_wf;

	return ret;
}

static int ad4052_config(struct ad4052_state *st)
{
	struct device *dev = &st->spi->dev;

	st->xfer.speed_hz = st->spi->max_speed_hz;
	st->xfer.rx_buf = &st->d32;

	spi_message_init_with_transfers(&st->msg, &st->xfer, 1);

	st->cnv_pwm = devm_pwm_get(dev, "cnv");
	if (IS_ERR(st->cnv_pwm))
		return dev_err_probe(dev, PTR_ERR(st->cnv_pwm),
				     "Failed to get cnv pwm\n");
	pwm_disable(st->cnv_pwm);

	st->cnv_gp = devm_gpiod_get_optional(dev, "cnv",
					     GPIOD_OUT_LOW);
	if (IS_ERR(st->cnv_gp))
		return dev_err_probe(dev, PTR_ERR(st->cnv_gp),
				    "Failed to get cnv gpio\n");

	return __ad4052_set_sampling_freq(st, AD4052_MAX_RATE(st->chip->grade));
}

static int ad4052_soft_reset(struct spi_device *spi)
{
	int ret;
	u8 buf[18] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
		      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE,
		      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

	ret = spi_write(spi, buf, sizeof(buf));
	if (ret)
		return ret;

	/* Wait AD4052 treset time */
	fsleep(5000);

	return 0;
}

static int ad4052_set_non_defaults(struct ad4052_state *st)
{
	struct iio_chan_spec chan = st->chip->channels[0];
	u8 val = AD4052_GP_MODE(1, AD4052_GP_DRDY) |
		 AD4052_GP_MODE(0, AD4052_GP_INTR);
	int ret;

	ret = regmap_update_bits(st->regmap, AD4052_REG_GP_CONFIG,
				 AD4052_GP_MODE_MSK(1) | AD4052_GP_MODE_MSK(0),
				 val);
	if (ret)
		return ret;

	val = AD4052_INTR_EN(0, AD4052_INTR_EN_EITHER) |
	      AD4052_INTR_EN(1, AD4052_INTR_EN_NEITHER);

	ret = regmap_update_bits(st->regmap, AD4052_REG_INTR_CONFIG,
				 AD4052_INTR_EN_MSK(0) | AD4052_INTR_EN_MSK(1),
				 val);
	if (ret)
		return ret;

	val = 0;
	if (chan.scan_type.sign == 's')
		val |= AD4052_DATA_FORMAT;

	st->data_format = val;

	if (st->chip->grade == AD4052_500KSPS) {
		ret = regmap_write(st->regmap, AD4052_REG_TIMER_CONFIG,
				   FIELD_PREP(AD4052_FS_MASK, AD4052_300KSPS));
		if (ret)
			return ret;
	}

	return regmap_write(st->regmap, AD4052_REG_ADC_MODES, val);
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

static int ad4052_request_irq(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct gpio_desc *gpio;
	int irq, ret = 0;

	gpio = devm_gpiod_get_optional(dev, "gp0", GPIOD_IN);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	if (gpio) {
		irq = gpiod_to_irq(gpio);

		if (irq < 0)
			return irq;

		ret = devm_request_threaded_irq(dev,
						irq, NULL, ad4052_irq_handler_thresh,
						IRQF_TRIGGER_RISING | IRQF_ONESHOT,
						indio_dev->name, indio_dev);
		if (ret)
			return ret;
	}

	gpio = devm_gpiod_get(dev, "gp1", GPIOD_IN);
	if (IS_ERR(gpio))
		return PTR_ERR(gpio);

	irq = gpiod_to_irq(gpio);

	if (irq < 0)
		return irq;

	st->gp1_irq = irq;
	ret = devm_request_threaded_irq(dev,
					irq, NULL, ad4052_irq_handler_drdy,
					IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
					indio_dev->name, st);
	return ret;
}

static const int ad4052_oversampling_avail[] = {
	0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096
};

static int ad4052_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, const int **vals,
			     int *type, int *len, long mask)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

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
	unreachable();
}

static ssize_t ad4052_get_sampling_freq(const struct ad4052_state *st, int *val)
{
	*val = DIV_ROUND_CLOSEST_ULL(NANO, st->cnv_wf.period_length_ns);

	return 0;
}

static ssize_t ad4052_set_sampling_freq(struct ad4052_state *st, unsigned int val)
{
	int ret;

	if (AD4052_CHECK_RATE(st->chip->grade, val))
		return -EINVAL;

	ret = __ad4052_set_sampling_freq(st, val);

	return ret;
}

static int __ad4052_read_chan_raw(struct ad4052_state *st, int *val)
{
	struct spi_device *spi = st->spi;
	int ret;
	struct spi_transfer t_cnv = {
		.len = 0
	};

	reinit_completion(&st->completion);

	if (st->cnv_gp) {
		gpiod_set_value_cansleep(st->cnv_gp, 1);
		gpiod_set_value_cansleep(st->cnv_gp, 0);
	} else {
		ret = spi_sync_transfer(spi, &t_cnv, 1);
		if (ret)
			return ret;
	}
	/*
	 * Single sample read should be used only for oversampling and
	 * sampling frequency pairs that take less than 1 sec.
	 */
	ret = wait_for_completion_timeout(&st->completion,
					  msecs_to_jiffies(1000));
	if (!ret)
		return -ETIMEDOUT;

	ret = spi_sync(spi, &st->msg);
	if (ret)
		return ret;

	if (st->xfer.len == 2) {
		*val = be16_to_cpu(st->d16);
		if (st->data_format & AD4052_DATA_FORMAT)
			*val = sign_extend32(*val, 15);
	} else {
		*val = be32_to_cpu(st->d32) >> 8;
		if (st->data_format & AD4052_DATA_FORMAT)
			*val = sign_extend32(*val, 23);
	}

	return ret;
}

static void ad4052_update_xfers(struct ad4052_state *st,
				const struct iio_scan_type *scan_type)
{
	struct spi_transfer *xfer = &st->xfer;

	xfer->bits_per_word = scan_type->realbits;
	xfer->len = BITS_TO_BYTES(scan_type->storagebits);
}

static int ad4052_read_chan_raw(struct iio_dev *indio_dev, int *val,
				const struct iio_scan_type *scan_type)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	ret = ad4052_set_operation_mode(st, st->mode);
	if (ret)
		goto out_error;

	ad4052_update_xfers(st, scan_type);

	ret = __ad4052_read_chan_raw(st, val);
	if (ret)
		goto out_error;

	ret = ad4052_exit_command(st);

out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	int ret;

	scan_type = iio_get_current_scan_type(indio_dev, chan);

	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			ret = ad4052_read_chan_raw(indio_dev, val, scan_type);
			if (ret < 0)
				return ret;

			return IIO_VAL_INT;
		case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
			ret = ad4052_get_oversampling_ratio(st, val);
			if (ret)
				return ret;

			return IIO_VAL_INT;
		case IIO_CHAN_INFO_SAMP_FREQ:
			ret = ad4052_get_sampling_freq(st, val);
			if (ret < 0)
				return ret;

			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	}
	unreachable();
}

static int ad4052_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

		switch (info) {
		case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
			return ad4052_set_oversampling_ratio(st, val);
		case IIO_CHAN_INFO_SAMP_FREQ:
			return ad4052_set_sampling_freq(st, val);
		default:
			return -EINVAL;
		}
	}
	unreachable();
}

static int ad4052_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int state;
	int ret;

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

		ret = regmap_read(st->regmap, AD4052_REG_GP_CONFIG, &state);
		if (!ret)
			ret = state & AD4052_GP_MODE_MSK(0);
		return ret;
	}
	unreachable();
}

static int ad4052_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (st->wait_event == state)
		goto out_release;

	if (state) {
		ret = pm_runtime_resume_and_get(&st->spi->dev);
		if (ret)
			goto out_release;

		ret = ad4052_set_operation_mode(st, AD4052_MONITOR_MODE);
		if (ret)
			goto out_err_suspend;
	} else {
		pm_runtime_mark_last_busy(&st->spi->dev);
		pm_runtime_put_autosuspend(&st->spi->dev);

		ret = ad4052_exit_command(st);
	}
	st->wait_event = state;
	iio_device_release_direct_mode(indio_dev);
	return ret;

out_err_suspend:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);

out_release:
	iio_device_release_direct_mode(indio_dev);
	return ret;
}

static int ad4052_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val, int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u8 size = 1;
	int ret;
	u8 reg;

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

		switch (info) {
		case IIO_EV_INFO_VALUE:
			if (dir == IIO_EV_DIR_RISING)
				reg = AD4052_REG_MAX_LIMIT;
			else
				reg = AD4052_REG_MIN_LIMIT;
			size++;
			break;
		case IIO_EV_INFO_HYSTERESIS:
			if (dir == IIO_EV_DIR_RISING)
				reg = AD4052_REG_MAX_HYST;
			else
				reg = AD4052_REG_MIN_HYST;
			break;
		default:
			return -EINVAL;
		}

		ret = regmap_bulk_read(st->regmap, reg, &st->d32, size);
		if (ret)
			return ret;

		if (reg == AD4052_REG_MAX_LIMIT || reg == AD4052_REG_MIN_LIMIT) {
			*val = be16_to_cpu(st->d16);
			if (st->data_format & AD4052_DATA_FORMAT)
				*val = sign_extend32(*val, 11);
		} else {
			*val = st->d32;
		}
		return IIO_VAL_INT;
	}
	unreachable();
}

static int ad4052_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val, int val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	u8 size = 1;
	int ret;
	u8 reg;

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

		st->d16 = cpu_to_be16(val);

		switch (type) {
		case IIO_EV_TYPE_THRESH:
			switch (info) {
			case IIO_EV_INFO_VALUE:
				if (st->data_format & AD4052_DATA_FORMAT) {
					if (val > 2047 || val < -2048)
						return -EINVAL;
				} else if (val > 4095 || val < 0) {
					return -EINVAL;
				}
				if (dir == IIO_EV_DIR_RISING)
					reg = AD4052_REG_MAX_LIMIT;
				else
					reg = AD4052_REG_MIN_LIMIT;
				size++;
				break;
			case IIO_EV_INFO_HYSTERESIS:
				if (val & BIT(7))
					return -EINVAL;
				if (dir == IIO_EV_DIR_RISING)
					reg = AD4052_REG_MAX_HYST;
				else
					reg = AD4052_REG_MIN_HYST;
				st->d16 >>= 8;
				break;
			default:
				return -EINVAL;
			}
			break;
		default:
			return -EINVAL;
		}

		ret = regmap_bulk_write(st->regmap, reg, &st->d16, size);

		return ret;
	}
	unreachable();
}

static int ad4052_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	struct spi_device *spi = st->spi;
	int ret;

	if (st->wait_event)
		return -EBUSY;

	scan_type = iio_get_current_scan_type(indio_dev, &indio_dev->channels[0]);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	ret = ad4052_set_operation_mode(st, st->mode);
	if (ret)
		goto out_error;

	ad4052_update_xfers(st, scan_type);

	disable_irq(st->gp1_irq);

	spi_engine_ex_offload_load_msg(spi, &st->msg);
	spi_engine_ex_offload_enable(spi, true);

	ret = pwm_set_waveform_might_sleep(st->cnv_pwm, &st->cnv_wf, false);
	if (ret)
		goto out_pwm_error;

	return 0;
out_pwm_error:
	if (st->gp1_irq)
		enable_irq(st->gp1_irq);
out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	struct spi_device *spi = st->spi;
	int ret;

	spi_engine_ex_offload_enable(spi, false);

	pwm_disable(st->cnv_pwm);

	enable_irq(st->gp1_irq);

	ret = ad4052_exit_command(st);

	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static const struct iio_buffer_setup_ops ad4052_buffer_setup_ops = {
	.preenable = &ad4052_buffer_preenable,
	.postdisable = &ad4052_buffer_postdisable,
};

static int ad4052_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				     unsigned int writeval, unsigned int *readval)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	iio_device_claim_direct_scoped(return -EBUSY, indio_dev) {
		if (st->wait_event)
			return -EBUSY;

		if (readval)
			return regmap_read(st->regmap, reg, readval);

		return regmap_write(st->regmap, reg, writeval);
	}
	unreachable();
}

static int ad4052_get_current_scan_type(const struct iio_dev *indio_dev,
					const struct iio_chan_spec *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);

	return st->mode == AD4052_BURST_AVERAGING_MODE ?
			   AD4052_SCAN_TYPE_BURST_AVG : AD4052_SCAN_TYPE_SAMPLE;
}

static const struct iio_info ad4052_info = {
	.read_raw = ad4052_read_raw,
	.write_raw = ad4052_write_raw,
	.read_avail = ad4052_read_avail,
	.read_event_config = &ad4052_read_event_config,
	.write_event_config = &ad4052_write_event_config,
	.read_event_value = &ad4052_read_event_value,
	.write_event_value = &ad4052_write_event_value,
	.get_current_scan_type = &ad4052_get_current_scan_type,
	.debugfs_reg_access = &ad4052_debugfs_reg_access,
};

static const struct regmap_config ad4052_regmap_config = {
	.name = "ad4062",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AD4052_MAX_REG,
	.read_flag_mask = BIT(7),
	.can_sleep = true,
};

static int ad4052_probe(struct spi_device *spi)
{
	const struct ad4052_chip_info *chip;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4052_state *st;
	int ret;
	u8 buf;

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

	st->regmap = devm_regmap_init_spi(spi, &ad4052_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&spi->dev,  PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	st->mode = AD4052_SAMPLE_MODE;
	st->wait_event = false;
	st->chip = chip;

	ret = ad4052_config(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Resources configuration failed\n");

	indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
	indio_dev->channels = chip->channels;
	indio_dev->num_channels = 1;
	indio_dev->setup_ops = &ad4052_buffer_setup_ops;
	indio_dev->info = &ad4052_info;
	indio_dev->name = chip->name;

	ret = devm_spi_optimize_message(&st->spi->dev, st->spi, &st->msg);
	if (ret)
		return ret;

	ret = devm_iio_dmaengine_buffer_setup_ext(dev, indio_dev, "rx",
						  IIO_BUFFER_DIRECTION_IN,
						  NULL,
						  NULL);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get DMA buffer\n");

	ret = ad4052_soft_reset(spi);
	if (ret)
		return dev_err_probe(dev, ret,
				     "AD4052 failed to soft reset\n");

	ret = ad4052_assert(st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "AD4052 fields assertions failed\n");

	ret = ad4052_set_non_defaults(st);
	if (ret)
		return ret;

	buf = AD4052_DEVICE_RESET;
	ret = regmap_write(st->regmap, AD4052_REG_DEVICE_STATUS, buf);
	if (ret)
		return ret;

	ret = ad4052_request_irq(indio_dev);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to enable pm_runtime\n");

	return devm_iio_device_register(dev, indio_dev);
}

static int ad4052_runtime_suspend(struct device *dev)
{
	u8 val = FIELD_PREP(AD4052_POWER_MODE_MSK, AD4052_LOW_POWER_MODE);
	struct ad4052_state *st = dev_get_drvdata(dev);

	return regmap_write(st->regmap, AD4052_REG_DEVICE_CONFIG, val);
}

static int ad4052_runtime_resume(struct device *dev)
{
	struct ad4052_state *st = dev_get_drvdata(dev);
	u8 val = FIELD_PREP(AD4052_POWER_MODE_MSK, 0);
	int ret;

	ret = regmap_write(st->regmap, AD4052_REG_DEVICE_CONFIG, val);
	if (ret)
		return ret;

	fsleep(2000);
	return 0;
}

static const struct dev_pm_ops ad4052_pm_ops = {
	SET_RUNTIME_PM_OPS(ad4052_runtime_suspend, ad4052_runtime_resume, NULL)
};

static const struct spi_device_id ad4052_id_table[] = {
	{"ad4050", (kernel_ulong_t)&ad4050_chip_info },
	{"ad4052", (kernel_ulong_t)&ad4052_chip_info },
	{"ad4056", (kernel_ulong_t)&ad4056_chip_info },
	{"ad4058", (kernel_ulong_t)&ad4058_chip_info },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4052_id_table);

static const struct of_device_id ad4052_of_match[] = {
	{ .compatible = "adi,ad4050", .data = &ad4050_chip_info },
	{ .compatible = "adi,ad4052", .data = &ad4052_chip_info },
	{ .compatible = "adi,ad4056", .data = &ad4056_chip_info },
	{ .compatible = "adi,ad4058", .data = &ad4058_chip_info },
	{}
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
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
