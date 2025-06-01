// SPDX-License-Identifier: GPL-2.0
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
#include <linux/iio/iio.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
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
#define AD4052_REG_ADC_CONFIG				0x22
#define     AD4052_REG_ADC_CONFIG_REF_EN_MSK		BIT(5)
#define     AD4052_REG_ADC_CONFIG_SCALE_EN_MSK		BIT(4)
#define AD4052_REG_AVG_CONFIG				0x23
#define AD4052_REG_GP_CONF				0x24
#define     AD4052_REG_GP_CONF_MODE_MSK_1		GENMASK(6, 4)
#define AD4052_REG_INTR_CONF				0x25
#define     AD4052_REG_INTR_CONF_EN_MSK_1		GENMASK(5, 4)
#define AD4052_REG_TIMER_CONFIG				0x27
#define     AD4052_REG_TIMER_CONFIG_FS_MASK		GENMASK(7, 4)
#define     AD4052_REG_TIMER_CONFIG_300KSPS		0x2
#define AD4052_REG_MON_VAL				0x2F
#define AD4052_REG_FUSE_CRC				0x40
#define AD4052_REG_DEVICE_STATUS			0x41
#define     AD4052_REG_DEVICE_STATUS_DEVICE_RESET	BIT(6)
#define AD4052_REG_MIN_SAMPLE				0x45
#define AD4052_MAX_REG					AD4052_REG_MIN_SAMPLE

#define AD4052_MON_VAL_MIDDLE_POINT	0x8000

#define AD4052_SPI_VENDOR	0x0456

#define AD4052_2MSPS		0
#define AD4052_500KSPS		1

#define AD4052_SOFT_RESET	0x81

#define AD4052_MAX_RATE(x)	((x) == AD4052_500KSPS ? 500000 : 2000000)
#define AD4052_FS_OFFSET(g)	((g) == AD4052_500KSPS ? 2 : 0)
#define AD4052_FS(g)		(&ad4052_conversion_freqs[AD4052_FS_OFFSET(g)])
#define AD4052_FS_LEN(g)	(ARRAY_SIZE(ad4052_conversion_freqs) - (AD4052_FS_OFFSET(g)))

#define AD4052_GP_DRDY		0x2

#define AD4052_TCONV_NS		270
#define AD4052_T_CNVH_NS	10
#define AD4052_SPI_MAX_ADC_XFER_SPEED(x)	((x) >= 3300000 ? 83333333 : 58823529)
#define AD4052_SPI_MAX_REG_XFER_SPEED		16000000

enum ad4052_operation_mode {
	AD4052_SAMPLE_MODE = 0x0,
	AD4052_BURST_AVERAGING_MODE = 0x1,
	AD4052_MONITOR_MODE = 0x3,
};

struct ad4052_chip_info {
	const struct iio_chan_spec channels[1];
	const char *name;
	u16 prod_id;
	u16 avg_max;
	u8 grade;
};

enum {
	AD4052_SCAN_TYPE_SAMPLE,
	AD4052_SCAN_TYPE_BURST_AVG,
};

static const struct iio_scan_type ad4052_scan_type_12_s[] = {
	[AD4052_SCAN_TYPE_SAMPLE] = {
		.sign = 's',
		.realbits = 12,
		.storagebits = 32,
		.endianness = IIO_CPU,
	},
	[AD4052_SCAN_TYPE_BURST_AVG] = {
		.sign = 's',
		.realbits = 14,
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
		.realbits = 20,
		.storagebits = 32,
		.endianness = IIO_CPU,
	},
};

static const unsigned int ad4052_conversion_freqs[] = {
	2000000, 1000000, 300000, 100000,	/*  0 -  3 */
	33300, 10000, 3000, 500,		/*  4 -  7 */
	333, 250, 200, 166,			/*  8 - 11 */
	140, 124, 111,				/* 12 - 15 */
};

struct ad4052_state {
	const struct ad4052_chip_info *chip;
	enum ad4052_operation_mode mode;
	struct spi_device *spi;
	struct spi_transfer xfer;
	struct gpio_desc *cnv_gp;
	struct completion completion;
	struct regmap *regmap;
	int drdy_irq;
	int vio_uV;
	int vref_uV;
	u8 oversamp_ratio;
	u8 reg_tx[4];
	u8 reg_rx[4];
	union {
		__be32 be32;
		__be16 be16;
		u8 bytes[4];
	} buf __aligned(IIO_DMA_MINALIGN);
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


#define AD4052_CHAN(bits) {								\
	.type = IIO_VOLTAGE,								\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_RAW) |				\
				    BIT(IIO_CHAN_INFO_SCALE) |				\
				    BIT(IIO_CHAN_INFO_CALIBSCALE) |			\
				    BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),		\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO),	\
	.indexed = 1,									\
	.channel = 0,									\
	.has_ext_scan_type = 1,								\
	.ext_scan_type = ad4052_scan_type_##bits##_s,					\
	.num_ext_scan_type = ARRAY_SIZE(ad4052_scan_type_##bits##_s),			\
}

static const struct ad4052_chip_info ad4050_chip_info = {
	.name = "ad4050",
	.channels = { AD4052_CHAN(12) },
	.prod_id = 0x70,
	.avg_max = 256,
	.grade = AD4052_2MSPS,
};

static const struct ad4052_chip_info ad4052_chip_info = {
	.name = "ad4052",
	.channels = { AD4052_CHAN(16) },
	.prod_id = 0x72,
	.avg_max = 4096,
	.grade = AD4052_2MSPS,
};

static const struct ad4052_chip_info ad4056_chip_info = {
	.name = "ad4056",
	.channels = { AD4052_CHAN(12) },
	.prod_id = 0x76,
	.avg_max = 256,
	.grade = AD4052_500KSPS,
};

static const struct ad4052_chip_info ad4058_chip_info = {
	.name = "ad4058",
	.channels = { AD4052_CHAN(16) },
	.prod_id = 0x78,
	.avg_max = 4096,
	.grade = AD4052_500KSPS,
};

static int ad4052_set_oversampling_ratio(struct ad4052_state *st, int val, int val2)
{
	const u32 _max = st->chip->avg_max;
	const u32 _min = 1;
	int ret;

	if (!in_range(val, _min, _max) || val2 != 0)
		return -EINVAL;

	/* 1 disables oversampling */
	val = ilog2(val);
	if (val == 0) {
		st->mode = AD4052_SAMPLE_MODE;
	} else {
		st->mode = AD4052_BURST_AVERAGING_MODE;
		ret = regmap_write(st->regmap, AD4052_REG_AVG_CONFIG, val - 1);
		if (ret)
			return ret;
	}
	st->oversamp_ratio = val;

	return 0;
}

static int ad4052_get_oversampling_ratio(struct ad4052_state *st, int *val)
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


static int ad4052_update_xfer_raw(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	struct spi_transfer *xfer = &st->xfer;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	xfer->rx_buf = st->buf.bytes;
	xfer->bits_per_word = roundup_pow_of_two(scan_type->realbits); /* + SE_BITS */
	xfer->len = spi_bpw_to_bytes(scan_type->realbits);
	xfer->speed_hz = AD4052_SPI_MAX_ADC_XFER_SPEED(st->vio_uV);

	return 0;
}

static int ad4052_check_ids(struct ad4052_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;
	u16 val;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_PROD_ID_1,
			       &st->buf.be16, sizeof(st->buf.be16));
	if (ret)
		return ret;

	val = be16_to_cpu(st->buf.be16);
	if (val != st->chip->prod_id)
		dev_warn(dev, "Production ID x%x does not match expected value", val);

	ret = regmap_bulk_read(st->regmap, AD4052_REG_VENDOR_H,
			       &st->buf.be16, sizeof(st->buf.be16));
	if (ret)
		return ret;

	val = be16_to_cpu(st->buf.be16);
	if (val != AD4052_SPI_VENDOR) {
		dev_err(dev, "Vendor ID x%x does not match expected value\n", val);
		return -ENODEV;
	}

	return 0;
}

static int ad4052_set_operation_mode(struct ad4052_state *st,
				     enum ad4052_operation_mode mode)
{
	int ret;

	ret = regmap_update_bits(st->regmap, AD4052_REG_ADC_MODES,
				 AD4052_REG_ADC_MODES_MODE_MSK, mode);
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD4052_REG_MODE_SET,
			    AD4052_REG_MODE_SET_ENTER_ADC);
}

static int ad4052_soft_reset(struct ad4052_state *st)
{
	u8 val = AD4052_SOFT_RESET;
	int ret;

	ret = regmap_write(st->regmap, AD4052_REG_INTERFACE_CONFIG_A, val);
	if (ret)
		return ret;

	/* Wait AD4052 treset time, datasheet p8 */
	ndelay(60);

	return 0;
}

static int ad4052_setup(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			const bool *ref_sel)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;
	int ret;

	scan_type = iio_get_current_scan_type(indio_dev, chan);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	ret = regmap_update_bits(st->regmap, AD4052_REG_GP_CONF,
				 AD4052_REG_GP_CONF_MODE_MSK_1,
				 FIELD_PREP(AD4052_REG_GP_CONF_MODE_MSK_1,
					    AD4052_GP_DRDY));
	if (ret)
		return ret;

	if (st->chip->grade == AD4052_500KSPS) {
		ret = regmap_write(st->regmap, AD4052_REG_TIMER_CONFIG,
				   FIELD_PREP(AD4052_REG_TIMER_CONFIG_FS_MASK,
					      AD4052_REG_TIMER_CONFIG_300KSPS));
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(st->regmap, AD4052_REG_ADC_CONFIG,
				 AD4052_REG_ADC_CONFIG_REF_EN_MSK,
				 FIELD_PREP(AD4052_REG_ADC_CONFIG_REF_EN_MSK,
					    *ref_sel));
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4052_REG_DEVICE_STATUS,
			   AD4052_REG_DEVICE_STATUS_DEVICE_RESET);

	st->buf.be16 = cpu_to_be16(AD4052_MON_VAL_MIDDLE_POINT);
	ret = regmap_bulk_write(st->regmap, AD4052_REG_MON_VAL,
				&st->buf.be16, sizeof(st->buf.be16));
	if (ret)
		return ret;

	return regmap_write(st->regmap, AD4052_REG_INTERFACE_STATUS,
			    AD4052_REG_INTERFACE_STATUS_NOT_RDY);
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
	int ret;

	ret = fwnode_irq_get_byname(dev_fwnode(&st->spi->dev), "gp1");
	if (ret == -EPROBE_DEFER)
		return ret;

	st->drdy_irq = ret;
	return devm_request_threaded_irq(dev, st->drdy_irq,
					ad4052_irq_handler_drdy,
					NULL, IRQF_ONESHOT, indio_dev->name,
					st);
}

static const int ad4052_oversampling_avail[] = {
	1, 2, 4, 8, 16, 32, 64, 128,		/*  0 -  7 */
	256, 512, 1024, 2048, 4096,		/*  8 - 12 */
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

static int ad4052_get_chan_scale(struct iio_dev *indio_dev, int *val, int *val2)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	const struct iio_scan_type *scan_type;

	/*
	 * In burst averaging mode the averaging filter accumulates resulting
	 * in a sample with increased precision.
	 */
	scan_type = iio_get_current_scan_type(indio_dev, st->chip->channels);
	if (IS_ERR(scan_type))
		return PTR_ERR(scan_type);

	*val = (st->vref_uV * 2) / (MICRO / MILLI); /* signed */
	*val2 = scan_type->realbits - 1;

	return IIO_VAL_FRACTIONAL_LOG2;
}

static int ad4052_get_chan_calibscale(struct ad4052_state *st, int *val, int *val2)
{
	int ret;

	ret = regmap_bulk_read(st->regmap, AD4052_REG_MON_VAL,
			       &st->buf.be16, sizeof(st->buf.be16));
	if (ret)
		return ret;

	/* From datasheet: code out = code in × mon_val/0x8000 */
	*val = be16_to_cpu(st->buf.be16) * 2;
	*val2 = 16;

	return IIO_VAL_FRACTIONAL_LOG2;
}

static int ad4052_set_chan_calibscale(struct ad4052_state *st, int gain_int,
				      int gain_frac)
{
	/* Divide numerator and denumerator by known great common divider */
	const u32 mon_val = AD4052_MON_VAL_MIDDLE_POINT / 64;
	const u32 micro = MICRO / 64;
	const u32 gain_fp = gain_int * MICRO + gain_frac;
	const u32 reg_val = DIV_ROUND_CLOSEST(gain_fp * mon_val, micro);
	int ret;

	/* Checks if the gain is in range and the value fits the field */
	if (gain_int < 0 || gain_int > 1 || reg_val > BIT(16) - 1)
		return -EINVAL;

	st->buf.be16 = cpu_to_be16(reg_val);
	ret = regmap_bulk_write(st->regmap, AD4052_REG_MON_VAL,
				&st->buf.be16, sizeof(st->buf.be16));
	if (ret)
		return ret;

	/* Enable scale if gain is not equal to one */
	return regmap_update_bits(st->regmap, AD4052_REG_ADC_CONFIG,
				  AD4052_REG_ADC_CONFIG_SCALE_EN_MSK,
				  FIELD_PREP(AD4052_REG_ADC_CONFIG_SCALE_EN_MSK,
					     !(gain_int == 1 && gain_frac == 0)));
}

static int ad4052_exit_command(struct ad4052_state *st)
{
	struct spi_device *spi = st->spi;
	const u8 val = 0xA8;

	return spi_write_then_read(spi, &val, 1, NULL, 0);
}

static int __ad4052_read_chan_raw(struct ad4052_state *st, int *val)
{
	struct spi_device *spi = st->spi;
	struct spi_transfer t_cnv = {};
	int ret;

	ret = ad4052_set_operation_mode(st, st->mode);
	if (ret)
		return ret;

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
		*val = sign_extend32(st->buf.be16, 15);
	else
		*val = sign_extend32(st->buf.be32, 23);

	return ad4052_exit_command(st);
}

static int ad4052_read_chan_raw(struct ad4052_state *st, int *val)
{
	int ret;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret)
		return ret;

	ret = __ad4052_read_chan_raw(st, val);

	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4052_read_raw_dispatch(struct ad4052_state *st,
				    int *val, int *val2, long info)
{

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4052_read_chan_raw(st, val);

	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4052_get_chan_calibscale(st, val, val2);

	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4052_get_oversampling_ratio(st, val);

	default:
		return -EINVAL;
	}
}

static int ad4052_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad4052_get_chan_scale(indio_dev, val, val2);
	}

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = ad4052_read_raw_dispatch(st, val, val2, info);
	iio_device_release_direct(indio_dev);
	return ret ?: IIO_VAL_INT;
}

static int ad4052_write_raw_dispatch(struct ad4052_state *st, int val, int val2,
				     long info)
{
	switch (info) {
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4052_set_oversampling_ratio(st, val, val2);

	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4052_set_chan_calibscale(st, val, val2);

	default:
		return -EINVAL;
	}
}

static int ad4052_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	struct ad4052_state *st = iio_priv(indio_dev);
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = ad4052_write_raw_dispatch(st, val, val2, info);
	iio_device_release_direct(indio_dev);
	return ret;
}


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
	.get_current_scan_type = ad4052_get_current_scan_type,
	.debugfs_reg_access = ad4052_debugfs_reg_access,
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

static int ad4052_regulators_get(struct ad4052_state *st, bool *ref_sel)
{
	struct device *dev = &st->spi->dev;
	int ret;

	st->vio_uV = devm_regulator_get_enable_read_voltage(dev, "vio");
	if (st->vio_uV < 0)
		return dev_err_probe(dev, st->vio_uV,
				     "Failed to enable and read vio voltage\n");

	st->vref_uV = devm_regulator_get_enable_read_voltage(dev, "ref");
	*ref_sel = st->vref_uV == -ENODEV;
	if (st->vref_uV < 0 && !*ref_sel)
		return dev_err_probe(dev, st->vref_uV,
				     "Failed to enable and read ref voltage\n");

	if (*ref_sel) {
		st->vref_uV = devm_regulator_get_enable_read_voltage(dev, "vdd");
		if (st->vref_uV < 0)
			return dev_err_probe(dev, st->vref_uV,
					     "Failed to enable and read vdd voltage\n");
	} else {
		ret = devm_regulator_get_enable(dev, "vdd");
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to enable vdd regulator\n");
	}

	return 0;
}

static int ad4052_probe(struct spi_device *spi)
{
	int ret;
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
	st->chip = chip;

	st->oversamp_ratio = 0;
	st->cnv_gp = devm_gpiod_get_optional(dev, "cnv", GPIOD_OUT_LOW);
	if (IS_ERR(st->cnv_gp))
		return dev_err_probe(dev, PTR_ERR(st->cnv_gp),
				     "Failed to get cnv gpio\n");

	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 1;
	indio_dev->info = &ad4052_info;
	indio_dev->name = chip->name;

	ret = ad4052_soft_reset(st);
	if (ret)
		return dev_err_probe(dev, ret, "AD4052 failed to soft reset\n");

	ret = ad4052_check_ids(st);
	if (ret)
		return ret;

	ret = ad4052_setup(indio_dev, indio_dev->channels, &ref_sel);
	if (ret)
		return ret;

	ret = ad4052_request_irq(indio_dev);
	if (ret)
		return ret;

	ret = ad4052_update_xfer_raw(indio_dev, indio_dev->channels);
	if (ret)
		return ret;

	pm_runtime_set_active(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable pm_runtime\n");

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

	/* Wait device functional blocks to power up */
	fsleep(3 * USEC_PER_MSEC);
	return 0;
}

static DEFINE_RUNTIME_DEV_PM_OPS(ad4052_pm_ops,
				 ad4052_runtime_suspend, ad4052_runtime_resume, NULL);

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
