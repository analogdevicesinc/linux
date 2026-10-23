// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2026 Analog Devices, Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/crc8.h>
#include <linux/delay.h>
#include <linux/dev_printk.h>
#include <linux/err.h>
#include <linux/export.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/mux/consumer.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/time64.h>
#include <linux/types.h>
#include <linux/unaligned.h>
#include <linux/units.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>

#define AD4134_RESET_TIME_US			10

#define AD4134_REG_READ_MASK			BIT(7)
#define AD4134_SPI_MAX_XFER_LEN			3

#define AD4134_EXT_CLOCK_MHZ			(48 * HZ_PER_MHZ)

#define AD4134_NUM_CHANNELS			4
#define AD4134_CHAN_PRECISION_BITS		24

#define AD4134_IFACE_CONFIG_A_REG		0x00
#define AD4134_IFACE_CONFIG_B_REG		0x01
#define AD4134_IFACE_CONFIG_B_SINGLE_INSTR	BIT(7)

#define AD4134_DEVICE_CONFIG_REG		0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK	BIT(0)
#define AD4134_POWER_MODE_HIGH_PERF		0x1

#define AD4134_SILICON_REV_REG			0x07
#define AD4134_SCRATCH_PAD_REG			0x0A
#define AD4134_STREAM_MODE_REG			0x0E
#define AD4134_SDO_PIN_SRC_SEL_REG		0x10
#define AD4134_SDO_PIN_SRC_SEL_SDO_SEL_MASK	BIT(2)

#define AD4134_DATA_PACKET_CONFIG_REG		0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK	GENMASK(5, 4)
#define AD4134_DATA_PACKET_24BIT_FRAME		0x2

#define AD4134_DIG_IF_CFG_REG			0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK		GENMASK(1, 0)
#define AD4134_DATA_FORMAT_SINGLE_CH_MODE	0x0

#define AD4134_PW_DOWN_CTRL_REG			0x13
#define AD4134_DEVICE_STATUS_REG		0x15
#define AD4134_ODR_VAL_INT_LSB_REG		0x16
#define AD4134_CHAN_DIG_FILTER_SEL_REG		0x1E
#define AD4134_CHAN_DIG_FILTER_SEL_CH_MASK(ch)	(GENMASK(1, 0) << 2 * (ch))

#define AD4134_CH3_OFFSET_MSB_REG		0x3E
#define AD4134_AIN_OR_ERROR_REG			0x48

#define AD4134_SDO_INPUT			0
#define AD4134_DOUT0_INPUT			1

/*
 * AD4134 register map ends at address 0x48 and there is no register for
 * retrieving ADC sample data. Though, to make use of Linux regmap API both
 * for register access and sample read, we define one virtual register for each
 * ADC channel. AD4134_CH_VREG(x) maps a channel number to it's virtual register
 * address while AD4134_VREG_CH(x) tells which channel given the address.
 */
#define AD4134_CH_VREG(x)			((x) + 0x50)
#define AD4134_VREG_CH(x)			((x) - 0x50)

#define AD4134_SPI_CRC_POLYNOM			0x07
#define AD4134_SPI_CRC_INIT_VALUE		0xA5
static unsigned char ad4134_spi_crc_table[CRC8_TABLE_SIZE];

enum ad4134_spi_mode {
	AD4134_SPI_MODE_NO_CS, /* datasheet calls this "minimum I/O mode" */
	AD4134_SPI_MODE_4_WIRE,
};

/* maps adi,spi-mode property value to enum */
static const char * const ad4134_spi_modes[] = {
	[AD4134_SPI_MODE_NO_CS] = "no-cs",
	[AD4134_SPI_MODE_4_WIRE] = "4-wire",
};

enum ad4134_filter_type {
	AD4134_WIDEBAND,
	AD4134_SINC6,
	AD4134_SINC3,
	AD4134_SINC3_REJ60,
};

static const char * const ad4134_filt_names[] = {
	[AD4134_WIDEBAND] = "wideband",
	[AD4134_SINC6] = "sinc6",
	[AD4134_SINC3] = "sinc3",
	[AD4134_SINC3_REJ60] = "sinc3+rej60",
};

struct ad4134_state {
	struct spi_device *spi;
	struct regmap *regmap;
	unsigned long sys_clk_hz;
	struct gpio_desc *odr_gpio;
	/*
	 * Synchronize access to device state fields and ensure atomicity of
	 * consecutive register access operations.
	 */
	struct mutex sync_lock;
	struct mux_state *mux_st[2]; /* For external multiplexer control */
	enum ad4134_spi_mode spi_mode;
	int refin_mv;
	bool crc_en;
	/*
	 * DMA (thus cache coherency maintenance) requires the transfer buffers
	 * to live in their own cache lines.
	 */
	u8 rx_buf[AD4134_SPI_MAX_XFER_LEN * AD4134_NUM_CHANNELS] __aligned(IIO_DMA_MINALIGN);
	u8 tx_buf[AD4134_SPI_MAX_XFER_LEN];
};

static int ad4134_set_filter_type(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan,
				  unsigned int val)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int mask, reg_val;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	guard(mutex)(&st->sync_lock);

	mask = AD4134_CHAN_DIG_FILTER_SEL_CH_MASK(chan->channel);
	reg_val = field_prep(mask, val);
	return regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
				  mask, reg_val);
}

static int ad4134_get_filter_type(struct iio_dev *indio_dev,
				  struct iio_chan_spec const *chan)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int mask, reg_val;
	int ret;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	guard(mutex)(&st->sync_lock);

	ret = regmap_read(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG, &reg_val);
	if (ret)
		return ret;

	mask = AD4134_CHAN_DIG_FILTER_SEL_CH_MASK(chan->channel);
	return field_get(mask, reg_val);
}

static const struct iio_enum ad4134_filter_type_enum = {
	.items = ad4134_filt_names,
	.num_items = ARRAY_SIZE(ad4134_filt_names),
	.get = ad4134_get_filter_type,
	.set = ad4134_set_filter_type,
};

static const struct iio_chan_spec_ext_info ad4134_filter_type_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SEPARATE, &ad4134_filter_type_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SEPARATE, &ad4134_filter_type_enum),
	{ }
};

#define AD4134_CHANNEL(_index) {						\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.ext_info = ad4134_filter_type_ext_info,				\
	.scan_index = (_index),							\
	.scan_type = {								\
		.format = IIO_SCAN_FORMAT_SIGNED_INT,				\
		.realbits = AD4134_CHAN_PRECISION_BITS,				\
		.storagebits = 32,						\
		.endianness = IIO_CPU,						\
	},									\
}

static const struct iio_chan_spec ad4134_chan_set[] = {
	AD4134_CHANNEL(0),
	AD4134_CHANNEL(1),
	AD4134_CHANNEL(2),
	AD4134_CHANNEL(3),
	IIO_CHAN_SOFT_TIMESTAMP(4),
};

static const struct regmap_range ad4134_regmap_rd_range[] = {
	regmap_reg_range(AD4134_IFACE_CONFIG_A_REG, AD4134_SILICON_REV_REG),
	regmap_reg_range(AD4134_SCRATCH_PAD_REG, AD4134_PW_DOWN_CTRL_REG),
	regmap_reg_range(AD4134_DEVICE_STATUS_REG, AD4134_AIN_OR_ERROR_REG),
	regmap_reg_range(AD4134_CH_VREG(0), AD4134_CH_VREG(AD4134_NUM_CHANNELS - 1)),
};

static const struct regmap_range ad4134_regmap_wr_range[] = {
	regmap_reg_range(AD4134_IFACE_CONFIG_A_REG, AD4134_DEVICE_CONFIG_REG),
	regmap_reg_range(AD4134_SCRATCH_PAD_REG, AD4134_SCRATCH_PAD_REG),
	regmap_reg_range(AD4134_STREAM_MODE_REG, AD4134_PW_DOWN_CTRL_REG),
	regmap_reg_range(AD4134_ODR_VAL_INT_LSB_REG, AD4134_CH3_OFFSET_MSB_REG),
};

static const struct regmap_access_table ad4134_regmap_rd_table = {
	.yes_ranges = ad4134_regmap_rd_range,
	.n_yes_ranges = ARRAY_SIZE(ad4134_regmap_rd_range),
};

static const struct regmap_access_table ad4134_regmap_wr_table = {
	.yes_ranges = ad4134_regmap_wr_range,
	.n_yes_ranges = ARRAY_SIZE(ad4134_regmap_wr_range),
};

/*
 * When operating in modes other than minimum I/O mode, AD4134 register data on
 * the SDO line is sampled on SCLK rising edge (SPI mode 0). In addition to
 * that, when AD4134 SDO and DOUT0 pins are multiplexed by hardware OUTSIDE
 * AD4134 SILICON, the multiplexer state must be set to route SDO to the SPI
 * controller. With both SPI mode and multiplexer state properly set, register
 * access runs as usual. Successful calls to this function must be balanced with
 * calls to ad4134_release_reg_access() so to release the mux state lock. See
 * AD4134 IIO documentation for details.
 */
static int ad4134_claim_reg_access(struct ad4134_state *st)
{
	unsigned int spi_mode = st->spi->mode & SPI_MODE_X_MASK;
	int ret;

	if (st->spi_mode != AD4134_SPI_MODE_NO_CS && spi_mode != SPI_MODE_0) {
		st->spi->mode &= ~SPI_MODE_X_MASK;
		st->spi->mode |= SPI_MODE_0;
		ret = spi_setup(st->spi);
		if (ret) {
			st->spi->mode &= ~SPI_MODE_X_MASK;
			st->spi->mode |= spi_mode;
			return ret;
		}
	}

	if (st->spi_mode != AD4134_SPI_MODE_NO_CS && st->mux_st[AD4134_SDO_INPUT])
		return mux_state_select(st->mux_st[AD4134_SDO_INPUT]);

	return 0;
}

/*
 * Must be balanced with successful calls to ad4134_claim_reg_access().
 */
static int ad4134_release_reg_access(struct ad4134_state *st)
{
	if (st->spi_mode != AD4134_SPI_MODE_NO_CS && st->mux_st[AD4134_SDO_INPUT])
		return mux_state_deselect(st->mux_st[AD4134_SDO_INPUT]);

	return 0;
}

DEFINE_GUARD(__ad4134_reg_access_mode, struct ad4134_state *,
	     ad4134_claim_reg_access(_T), ad4134_release_reg_access(_T));
DEFINE_GUARD_COND(__ad4134_reg_access_mode, _try, ad4134_claim_reg_access(_T));

/*
 * When operating in modes other than minimum I/O mode, AD4134 ADC sample data
 * on the DOUT lines is sampled on SCLK falling edge (SPI mode 1). In addition
 * to that, when AD4134 SDO and DOUT0 pins are multiplexed by hardware OUTSIDE
 * AD4134 SILICON, the multiplexer state must be set to route DOUT0 to the SPI
 * controller. With both SPI mode and multiplexer state properly set, ADC
 * samples can be read normally. Successful call to this function must be paired
 * with a call to ad4134_release_sample_access().
 */
static int ad4134_claim_sample_access(struct ad4134_state *st)
{
	unsigned int spi_mode = st->spi->mode & SPI_MODE_X_MASK;
	int ret;

	if (st->spi_mode != AD4134_SPI_MODE_NO_CS && spi_mode != SPI_MODE_1) {
		st->spi->mode &= ~SPI_MODE_X_MASK;
		st->spi->mode |= SPI_MODE_1;
		ret = spi_setup(st->spi);
		if (ret) {
			st->spi->mode &= ~SPI_MODE_X_MASK;
			st->spi->mode |= spi_mode;
			return ret;
		}
	}

	if (st->spi_mode != AD4134_SPI_MODE_NO_CS && st->mux_st[AD4134_DOUT0_INPUT])
		return mux_state_select(st->mux_st[AD4134_DOUT0_INPUT]);

	return 0;
}

/*
 * Must be balanced with successful calls to ad4134_claim_sample_access().
 */
static int ad4134_release_sample_access(struct ad4134_state *st)
{
	if (st->spi_mode != AD4134_SPI_MODE_NO_CS && st->mux_st[AD4134_DOUT0_INPUT])
		return mux_state_deselect(st->mux_st[AD4134_DOUT0_INPUT]);

	return 0;
}

DEFINE_GUARD(__ad4134_sample_access_mode, struct ad4134_state *,
	     ad4134_claim_sample_access(_T), ad4134_release_sample_access(_T));
DEFINE_GUARD_COND(__ad4134_sample_access_mode, _try, ad4134_claim_sample_access(_T));

static int ad4134_calc_spi_crc(u8 inst, u8 data)
{
	u8 buf[] = { inst, data };

	return crc8(ad4134_spi_crc_table, buf, ARRAY_SIZE(buf),
		    AD4134_SPI_CRC_INIT_VALUE);
}

static void ad4134_prepare_spi_tx_buf(u8 inst, u8 data, u8 *buf)
{
	buf[0] = inst;
	buf[1] = data;
	buf[2] = ad4134_calc_spi_crc(inst, data);
}

static int ad4134_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4134_state *st = context;
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
		.len = st->crc_en ? AD4134_SPI_MAX_XFER_LEN : 2,
	};
	int ret;

	ACQUIRE(__ad4134_reg_access_mode, claim)(st);
	if (ACQUIRE_ERR(__ad4134_reg_access_mode, &claim))
		return -EBUSY;

	ad4134_prepare_spi_tx_buf(reg, val, st->tx_buf);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	if (st->crc_en && st->rx_buf[2] != st->tx_buf[2])
		dev_dbg(&st->spi->dev, "reg write CRC check failed\n");

	return 0;
}

static int ad4134_data_read(struct ad4134_state *st, unsigned int reg,
			    unsigned int *val)
{
	unsigned int i;
	u32 sample = 0;
	int ret;

	ACQUIRE(__ad4134_sample_access_mode, claim)(st);
	if (ACQUIRE_ERR(__ad4134_sample_access_mode, &claim))
		return -EBUSY;

	/*
	 * To be able to read data from all 4 channels through a single line, we
	 * set DOUTx output format to 0 in the digital interface config register
	 * (0x12). With that, data from all four channels is serialized and
	 * output on DOUT0. During the probe, we also set SDO_PIN_SRC_SEL in
	 * DEVICE_CONFIG_1 register to duplicate DOUT0 on the SDO pin. Combined,
	 * those configurations enable ADC data read through a conventional SPI
	 * interface. Now we read data from all channels but keep only the bits
	 * from the requested one.
	 */
	for (i = 0; i < AD4134_NUM_CHANNELS; i++) {
		ret = spi_write_then_read(st->spi, NULL, 0, st->rx_buf,
					  BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS));
		if (ret)
			return ret;

		/*
		 * AD4134 has a built-in feature that flags when data transfers
		 * don't run enough clock cycles to read the entire data frame.
		 * Clock out data from all channels to avoid that.
		 */
		if (i == AD4134_VREG_CH(reg))
			sample = get_unaligned_be24(st->rx_buf);
	}
	*val = sign_extend32(sample, AD4134_CHAN_PRECISION_BITS - 1);

	return 0;
}

static int ad4134_register_read(struct ad4134_state *st, unsigned int reg,
				unsigned int *val)
{
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
		.len = st->crc_en ? AD4134_SPI_MAX_XFER_LEN : 2,
	};
	unsigned int inst;
	int ret;

	ACQUIRE(__ad4134_reg_access_mode, claim)(st);
	if (ACQUIRE_ERR(__ad4134_reg_access_mode, &claim))
		return -EBUSY;

	inst = AD4134_REG_READ_MASK | reg;
	ad4134_prepare_spi_tx_buf(inst, 0, st->tx_buf);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	*val = st->rx_buf[1];

	/* Check CRC */
	if (st->crc_en && st->rx_buf[2] != st->tx_buf[2])
		dev_dbg(&st->spi->dev, "reg read CRC check failed\n");

	return 0;
}

static int ad4134_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4134_state *st = context;

	if (reg >= AD4134_CH_VREG(0))
		return ad4134_data_read(st, reg, val);

	return ad4134_register_read(st, reg, val);
}

static const struct regmap_config ad4134_regmap_config = {
	.reg_read = ad4134_reg_read,
	.reg_write = ad4134_reg_write,
	.rd_table = &ad4134_regmap_rd_table,
	.wr_table = &ad4134_regmap_wr_table,
	.max_register = AD4134_CH_VREG(AD4134_NUM_CHANNELS - 1),
};

static irqreturn_t ad4134_trigger_handler(int irq, void *p)
{
	IIO_DECLARE_BUFFER_WITH_TS(u32, channels, AD4134_NUM_CHANNELS) = { };
	unsigned int chan_bytes = BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS);
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4134_state *st = iio_priv(indio_dev);
	struct spi_transfer xfer = {
		.rx_buf = st->rx_buf,
		.cs_off = true,
		.len = chan_bytes * AD4134_NUM_CHANNELS,
	};
	unsigned int ch, i = 0;
	int ret;

	gpiod_set_value_cansleep(st->odr_gpio, 1);
	fsleep(1);
	gpiod_set_value_cansleep(st->odr_gpio, 0);

	/* Run enough DCLK cycles to match the data frame length. */
	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		goto out;

	iio_for_each_active_channel(indio_dev, ch)
		channels[i++] = get_unaligned_be24(&st->rx_buf[ch * chan_bytes]);

	iio_push_to_buffers_with_ts(indio_dev, channels, sizeof(channels),
				    pf->timestamp);

out:
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int ad4134_buffer_preenable(struct iio_dev *indio_dev)
{
	return ad4134_claim_sample_access(iio_priv(indio_dev));
}

static int ad4134_buffer_postdisable(struct iio_dev *indio_dev)
{
	return ad4134_release_sample_access(iio_priv(indio_dev));
}

static const struct iio_buffer_setup_ops ad4134_buffer_setup_ops = {
	.preenable = &ad4134_buffer_preenable,
	.postdisable = &ad4134_buffer_postdisable,
};

static int ad4134_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW: {
		IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
		if (IIO_DEV_ACQUIRE_FAILED(claim))
			return -EBUSY;

		guard(mutex)(&st->sync_lock);

		gpiod_set_value_cansleep(st->odr_gpio, 1);
		/*
		 * For slave mode gated DCLK (data sheet page 11), the minimum
		 * ODR high time is 3 * tDIGCLK. The internal digital clock
		 * period is tDIGCLK = 1/fDIGCLK = 2/fSYSCLK.
		 * The System clock frequency (fSYSCLK) is typically 48 MHz.
		 * Thus, ODR high time = 3 * (2 / (48 * HZ_PER_MHZ))
		 * ODR high time = 0.000000125 s = 125 ns
		 * 1 micro second should be more than enough. Not worth it
		 * tweaking for shorter dealy since this is not a fast data path.
		 */
		fsleep(1);
		gpiod_set_value_cansleep(st->odr_gpio, 0);
		ret = regmap_read(st->regmap, AD4134_CH_VREG(chan->channel), val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	}
	case IIO_CHAN_INFO_SCALE:
		*val = st->refin_mv;
		*val2 = AD4134_CHAN_PRECISION_BITS - 1;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad4134_debugfs_reg_access(struct iio_dev *indio_dev,
				     unsigned int reg, unsigned int writeval,
				     unsigned int *readval)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	guard(mutex)(&st->sync_lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4134_min_io_mode_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	st->crc_en = true; /* In minimum I/O mode CRC cannot be disabled */
	ret = regmap_update_bits(st->regmap, AD4134_DIG_IF_CFG_REG,
				 AD4134_DIF_IF_CFG_FORMAT_MASK,
				 FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					    AD4134_DATA_FORMAT_SINGLE_CH_MODE));
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set single channel mode\n");

	ret = regmap_set_bits(st->regmap, AD4134_SDO_PIN_SRC_SEL_REG,
			      AD4134_SDO_PIN_SRC_SEL_SDO_SEL_MASK);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set SDO source selection\n");

	return regmap_set_bits(st->regmap, AD4134_IFACE_CONFIG_B_REG,
			       AD4134_IFACE_CONFIG_B_SINGLE_INSTR);
}

static const struct iio_info ad4134_info = {
	.read_raw = ad4134_read_raw,
	.debugfs_reg_access = ad4134_debugfs_reg_access,
};

static const char * const ad4143_required_regulators[] = {
	"avdd5", "dvdd5", "iovdd",
};

static const char * const ad4143_optional_regulators[] = {
	"avdd1v8", "dvdd1v8", "clkvdd",
};

static int ad4134_regulator_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ad4143_required_regulators),
					     ad4143_required_regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable power supplies\n");

	/* Required regulator that we need to read the voltage */
	ret = devm_regulator_get_enable_read_voltage(dev, "refin");
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get REFIN voltage.\n");

	st->refin_mv = ret / (MICRO / MILLI);

	ret = devm_regulator_get_enable_optional(dev, "ldoin");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to enable ldoin supply\n");

	/* If ldoin was provided, then use the use the internal LDO regulators */
	if (ret == 0)
		return 0;

	/*
	 * If ldoin is not provided, then avdd1v8, dvdd1v8, and clkvdd are
	 * required.
	 */
	ret = devm_regulator_bulk_get_enable(dev, ARRAY_SIZE(ad4143_optional_regulators),
					     ad4143_optional_regulators);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable 1V8 power supplies\n");

	return 0;
}

static int ad4134_clock_select(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct clk *xtal_clk, *clkin_clk;

	/*
	 * AD4134 requires one external clock source and only one external clock
	 * source can be provided at a time. Try to get a crystal provided clock.
	 * If that fails, try to get a CMOS clock.
	 */
	xtal_clk = devm_clk_get_optional_enabled(dev, "xtal");
	if (!xtal_clk)
		xtal_clk = devm_clk_get_optional_enabled(dev, "xtal");
	if (IS_ERR(xtal_clk))
		return dev_err_probe(dev, PTR_ERR(xtal_clk),
				     "failed to get xtal\n");

	clkin_clk = devm_clk_get_optional_enabled(dev, "clkin");
	if (!clkin_clk)
		clkin_clk = devm_clk_get_optional_enabled(dev, "clkin");
	if (IS_ERR(clkin_clk))
		return dev_err_probe(dev, PTR_ERR(clkin_clk),
				     "failed to get clkin\n");

	st->sys_clk_hz = clk_get_rate(xtal_clk) | clk_get_rate(clkin_clk);
	if (st->sys_clk_hz != AD4134_EXT_CLOCK_MHZ)
		dev_warn(dev, "invalid external clock frequency %lu\n",
			 st->sys_clk_hz);

	return 0;
}

static int ad4134_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct reset_control *rst;
	struct iio_dev *indio_dev;
	struct mux_state *mux_st;
	struct ad4134_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	indio_dev->name = "ad4134";
	indio_dev->channels = ad4134_chan_set;
	indio_dev->num_channels = ARRAY_SIZE(ad4134_chan_set);
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4134_info;

	ret = ad4134_regulator_setup(st);
	if (ret)
		return ret;

	ret = ad4134_clock_select(st);
	if (ret)
		return ret;

	rst = devm_reset_control_get_optional_exclusive_deasserted(dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(dev, PTR_ERR(rst),
				     "failed to get and deassert reset\n");

	fsleep(AD4134_RESET_TIME_US);
	crc8_populate_msb(ad4134_spi_crc_table, AD4134_SPI_CRC_POLYNOM);

	ret = devm_mutex_init(dev, &st->sync_lock);
	if (ret)
		return ret;

	st->regmap = devm_regmap_init(dev, NULL, st, &ad4134_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "failed to initialize regmap");

	if (device_property_present(dev, "adi,spi-mode")) {
		ret = device_property_match_property_string(dev, "adi,spi-mode",
							    ad4134_spi_modes,
							    ARRAY_SIZE(ad4134_spi_modes));
		if (ret < 0)
			return dev_err_probe(dev, ret,
					     "getting adi,spi-mode property failed\n");

		st->spi_mode = ret;
	} else {
		/* Default to no-cs mode if adi,spi-mode is not specified */
		st->spi_mode = AD4134_SPI_MODE_NO_CS;
	}

	if (st->spi_mode == AD4134_SPI_MODE_NO_CS) {
		/*
		 * The support hardware for AD4134 may have a multiplexer for
		 * selecting between AD4134 SDO and AD4134 DOUT0. If that mux
		 * is set but the user still wants to run AD4134 in minimum I/O
		 * mode, then DOUT0 is not used and the multiplexer OUTSIDE OF
		 * AD4134 SILICON must be set to select AD4134 SDO. See AD4134
		 * IIO documentation for details.
		 */
		mux_st = devm_mux_state_get_optional_selected(dev, "reg_access");
		if (IS_ERR(mux_st))
			return dev_err_probe(dev, PTR_ERR(mux_st),
					     "failed to get reg_access mux-state\n");

		st->mux_st[AD4134_SDO_INPUT] = mux_st;
		ret = ad4134_min_io_mode_setup(st);
		if (ret)
			return dev_err_probe(dev, ret, "failed to setup minimum I/O mode\n");
	} else {
		/*
		 * This controls a multiplexer OUTSIDE OF AD4134 SILICON.
		 * See AD4134 IIO documentation for details.
		 */
		mux_st = devm_mux_state_get(dev, "reg_access");
		if (IS_ERR(mux_st))
			return dev_err_probe(dev, PTR_ERR(mux_st),
					     "failed to get reg_access mux-state\n");

		st->mux_st[AD4134_SDO_INPUT] = mux_st;

		mux_st = devm_mux_state_get(dev, "data_read");
		st->mux_st[AD4134_DOUT0_INPUT] = devm_mux_state_get(dev, "data_read");
		if (IS_ERR(mux_st))
			return dev_err_probe(dev, PTR_ERR(mux_st),
					     "failed to get data_read mux-state\n");

		st->mux_st[AD4134_DOUT0_INPUT] = mux_st;
		indio_dev->setup_ops = &ad4134_buffer_setup_ops;
	}

	st->odr_gpio = devm_gpiod_get(dev, "odr", GPIOD_OUT_LOW);
	if (IS_ERR(st->odr_gpio))
		return dev_err_probe(dev, PTR_ERR(st->odr_gpio),
				     "failed to get ODR GPIO\n");

	ret = devm_iio_triggered_buffer_setup(dev, indio_dev,
					      iio_pollfunc_store_time,
					      ad4134_trigger_handler,
					      NULL);
	if (ret)
		return ret;

	/* Bump precision to 24-bit */
	ret = regmap_update_bits(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
				 AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
				 FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					    AD4134_DATA_PACKET_24BIT_FRAME));
	if (ret)
		return ret;

	/* Set high performance power mode */
	ret = regmap_update_bits(st->regmap, AD4134_DEVICE_CONFIG_REG,
				 AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
				 FIELD_PREP(AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
					    AD4134_POWER_MODE_HIGH_PERF));
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id ad4134_id[] = {
	{ .name = "ad4134" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4134_id);

static const struct of_device_id ad4134_of_match[] = {
	{ .compatible = "adi,ad4134" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_of_match);

static struct spi_driver ad4134_driver = {
	.driver = {
		.name = "ad4134",
		.of_match_table = ad4134_of_match,
	},
	.probe = ad4134_probe,
	.id_table = ad4134_id,
};
module_spi_driver(ad4134_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4134 SPI driver");
MODULE_LICENSE("GPL");
