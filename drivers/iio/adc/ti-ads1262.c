// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Texas Instruments ADS1262 ADC driver
 *
 * Copyright (C) 2026 Kurt Borja <kuurtb@gmail.com>
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/lockdep.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/units.h>

#include <asm/byteorder.h>

#include <linux/iio/iio.h>

#define ADS1262_OPCODE_NOP			0x00
#define ADS1262_OPCODE_RESET			0x06
#define ADS1262_OPCODE_START1			0x08
#define ADS1262_OPCODE_STOP1			0x0A
#define ADS1262_OPCODE_START2			0x0C
#define ADS1262_OPCODE_STOP2			0x0E
#define ADS1262_OPCODE_RDATA1			0x12
#define ADS1262_OPCODE_RDATA2			0x14
#define ADS1262_OPCODE_SYOCAL1			0x16
#define ADS1262_OPCODE_SYGCAL1			0x17
#define ADS1262_OPCODE_SFOCAL1			0x19
#define ADS1262_OPCODE_SYOCAL2			0x1B
#define ADS1262_OPCODE_SYGCAL2			0x1C
#define ADS1262_OPCODE_SFOCAL2			0x1E
#define ADS1262_OPCODE_RREG			0x20
#define ADS1262_OPCODE_WREG			0x40

#define ADS1262_ID_REG				0x00
#define   ADS1262_DEV_ID_MASK			GENMASK(7, 5)
#define     ADS1262_DEV_ID			  0
#define     ADS1263_DEV_ID			  1
#define   ADS1262_REV_ID_MASK			GENMASK(4, 0)

#define ADS1262_POWER_REG			0x01
#define   ADS1262_POWER_RESET_MASK		BIT(4)
#define   ADS1262_POWER_VBIAS_MASK		BIT(1)
#define   ADS1262_POWER_INTREF_MASK		BIT(0)

#define ADS1262_INTERFACE_REG			0x02
#define   ADS1262_INTERFACE_TIMEOUT_MASK	BIT(3)
#define   ADS1262_INTERFACE_STATUS_MASK		BIT(2)
#define   ADS1262_INTERFACE_CRC_MASK		GENMASK(1, 0)

#define ADS1262_MODE0_REG			0x03
#define   ADS1262_MODE0_REFREV_MASK		BIT(7)
#define   ADS1262_MODE0_RUNMODE_MASK		BIT(6)
#define     ADS1262_RUNMODE_CONTINUOUS		  0
#define     ADS1262_RUNMODE_PULSE		  1
#define   ADS1262_MODE0_IDAC_CHOP_MASK		BIT(5)
#define   ADS1262_MODE0_INPUT_CHOP_MASK		BIT(4)
#define   ADS1262_MODE0_DELAY_MASK		GENMASK(3, 0)

#define ADS1262_MODE1_REG			0x04
#define   ADS1262_MODE1_FILTER_MASK		GENMASK(7, 5)
#define     ADS1262_FILTER_FIR			  4
#define   ADS1262_MODE1_SBADC_MASK		BIT(4)
#define   ADS1262_MODE1_SBPOL_MASK		BIT(3)
#define   ADS1262_MODE1_SBMAG_MASK		GENMASK(2, 0)

#define ADS1262_MODE2_REG			0x05
#define   ADS1262_MODE2_BYPASS_MASK		BIT(7)
#define   ADS1262_MODE2_GAIN_MASK		GENMASK(6, 4)
#define   ADS1262_MODE2_DR_MASK			GENMASK(3, 0)
#define     ADS1262_DR_2_5_SPS			  0
#define     ADS1262_DR_5_SPS			  1
#define     ADS1262_DR_10_SPS			  2
#define     ADS1262_DR_16_6_SPS			  3
#define     ADS1262_DR_20_SPS			  4
#define     ADS1262_DR_50_SPS			  5
#define     ADS1262_DR_60_SPS			  6
#define     ADS1262_DR_100_SPS			  7
#define     ADS1262_DR_400_SPS			  8
#define     ADS1262_DR_1200_SPS			  9
#define     ADS1262_DR_2400_SPS			  10
#define     ADS1262_DR_4800_SPS			  11
#define     ADS1262_DR_7200_SPS			  12
#define     ADS1262_DR_14400_SPS		  13
#define     ADS1262_DR_19200_SPS		  14
#define     ADS1262_DR_38400_SPS		  15
#define     ADS1262_DR_COUNT			  (ADS1262_DR_38400_SPS + 1)

#define ADS1262_INPMUX_REG			0x06
#define   ADS1262_INPMUX_MUXP_MASK		GENMASK(7, 4)
#define   ADS1262_INPMUX_MUXN_MASK		GENMASK(3, 0)
#define     ADS1262_INPMUX_AIN1			  1
#define     ADS1262_INPMUX_AINCOM		  10
#define     ADS1262_INPMUX_TEMP			  11
#define     ADS1262_INPMUX_AVDD			  12
#define     ADS1262_INPMUX_DVDD			  13
#define     ADS1262_INPMUX_TDAC			  14
#define     ADS1262_INPMUX_FLOAT		  15

#define ADS1262_OFCAL0_REG			0x07
#define ADS1262_OFCAL1_REG			0x08
#define ADS1262_OFCAL2_REG			0x09
#define ADS1262_FSCAL0_REG			0x0A
#define ADS1262_FSCAL1_REG			0x0B
#define ADS1262_FSCAL2_REG			0x0C

#define ADS1262_IDACMUX_REG			0x0D
#define   ADS1262_IDACMUX_MUX2_MASK		GENMASK(7, 4)
#define   ADS1262_IDACMUX_MUX1_MASK		GENMASK(3, 0)
#define     ADS1262_IDACMUX_NO_CONN		0b1011

#define ADS1262_IDACMAG_REG			0x0E
#define   ADS1262_IDACMAG_MAG2_MASK		GENMASK(7, 4)
#define   ADS1262_IDACMAG_MAG1_MASK		GENMASK(3, 0)

#define ADS1262_REFMUX_REG			0x0F
#define   ADS1262_REFMUX_RMUXP_MASK		GENMASK(5, 3)
#define   ADS1262_REFMUX_RMUXN_MASK		GENMASK(2, 0)

#define ADS1262_TDACP_REG			0x10
#define   ADS1262_TDACP_OUTP_MASK		BIT(7)
#define   ADS1262_TDACP_MAGP_MASK		GENMASK(4, 0)

#define ADS1262_TDACN_REG			0x11
#define   ADS1262_TDACN_OUTN_MASK		BIT(7)
#define   ADS1262_TDACN_MAGN_MASK		GENMASK(4, 0)

#define ADS1262_GPIOCON_REG			0x12
#define ADS1262_GPIODIR_REG			0x13
#define ADS1262_GPIODAT_REG			0x14

#define ADS1262_ADC2CFG_REG			0x15
#define   ADS1262_ADC2CFG_DR2_MASK		GENMASK(7, 6)
#define   ADS1262_ADC2CFG_REF2_MASK		GENMASK(5, 3)
#define   ADS1262_ADC2CFG_GAIN2_MASK		GENMASK(2, 0)

#define ADS1262_ADC2MUX_REG			0x16
#define   ADS1262_ADC2MUX_MUXP2_MASK		GENMASK(7, 4)
#define   ADS1262_ADC2MUX_MUXN2_MASK		GENMASK(3, 0)

#define ADS1262_ADC2OFC0_REG			0x17
#define ADS1262_ADC2OFC1_REG			0x18
#define ADS1262_ADC2FSC0_REG			0x19
#define ADS1262_ADC2FSC1_REG			0x1A

#define ADS1262_REG_COUNT			(ADS1262_ADC2FSC1_REG + 1)

/*
 * The power transition timing requirement is 65536 clock cycles, at the minimum
 * clock frequency this is 65536 microseconds.
 */
#define ADS1262_POWER_TRANS_US			65536

#define ADS1262_NOMINAL_CLK_RATE_HZ		7372800
#define ADS1262_MODULATOR_DIV			8

#define ADS1262_FW_CHANNEL_COUNT		16
#define ADS1262_MON_CHANNEL_COUNT		4
#define ADS1262_REGMAP_WRITE_SZ			8

#define ADS1262_ADC1_RESOLUTION			32

struct ads1262_chip_info {
	const char *name;
};

struct ads1262_channel {
	u8 data_rate;
	u8 filter;
};

struct ads1262 {
	struct spi_device *spi;
	struct regmap *regmap;
	struct gpio_desc *start_gpiod;
	size_t num_channels;
	struct ads1262_channel *channels __counted_by_ptr(num_channels);
	/* protects concurrent SPI transfers */
	struct mutex xfer_lock;
	/* protects channel state */
	struct mutex chan_lock;
	struct completion drdy;
	unsigned long clk_rate_hz;
	int sampling_freq_table[ADS1262_DR_COUNT][2];
	int sampling_freq_fir[ADS1262_DR_16_6_SPS + 1][2];
};

static const struct iio_chan_spec ads1262_monitor_chan_specs[] = {
	{
		.type = IIO_TEMP,
		.channel = ADS1262_INPMUX_TEMP,
		.channel2 = ADS1262_INPMUX_TEMP,
		.scan_type = {
			.format = IIO_SCAN_FORMAT_SIGNED_INT,
			.realbits = ADS1262_ADC1_RESOLUTION,
			.storagebits = 32,
			.endianness = IIO_BE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = ADS1262_INPMUX_AVDD,
		.channel2 = ADS1262_INPMUX_AVDD,
		.indexed = 1,
		.scan_type = {
			.format = IIO_SCAN_FORMAT_SIGNED_INT,
			.realbits = ADS1262_ADC1_RESOLUTION,
			.storagebits = 32,
			.endianness = IIO_BE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = ADS1262_INPMUX_DVDD,
		.channel2 = ADS1262_INPMUX_DVDD,
		.indexed = 1,
		.scan_type = {
			.format = IIO_SCAN_FORMAT_SIGNED_INT,
			.realbits = ADS1262_ADC1_RESOLUTION,
			.storagebits = 32,
			.endianness = IIO_BE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_VOLTAGE,
		.channel = ADS1262_INPMUX_TDAC,
		.channel2 = ADS1262_INPMUX_TDAC,
		.indexed = 1,
		.differential = 1,
		.scan_type = {
			.format = IIO_SCAN_FORMAT_SIGNED_INT,
			.realbits = ADS1262_ADC1_RESOLUTION,
			.storagebits = 32,
			.endianness = IIO_BE,
		},
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
		.info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
};

#define ads1262_default_channel ((struct ads1262_channel) {	\
	.data_rate = ADS1262_DR_20_SPS,				\
	.filter = ADS1262_FILTER_FIR,				\
})

static const u32 ads1262_mod_decimator_a[] = {
	[ADS1262_DR_2_5_SPS]	= 64,
	[ADS1262_DR_5_SPS]	= 64,
	[ADS1262_DR_10_SPS]	= 64,
	[ADS1262_DR_16_6_SPS]	= 64,
	[ADS1262_DR_20_SPS]	= 64,
	[ADS1262_DR_50_SPS]	= 64,
	[ADS1262_DR_60_SPS]	= 64,
	[ADS1262_DR_100_SPS]	= 64,
	[ADS1262_DR_400_SPS]	= 64,
	[ADS1262_DR_1200_SPS]	= 64,
	[ADS1262_DR_2400_SPS]	= 64,
	[ADS1262_DR_4800_SPS]	= 64,
	[ADS1262_DR_7200_SPS]	= 64,
	[ADS1262_DR_14400_SPS]	= 64,
	[ADS1262_DR_19200_SPS]	= 48,
	[ADS1262_DR_38400_SPS]	= 24,
};

static const u32 ads1262_mod_decimator_b[] = {
	[ADS1262_DR_2_5_SPS]	= 5760,
	[ADS1262_DR_5_SPS]	= 2880,
	[ADS1262_DR_10_SPS]	= 1440,
	[ADS1262_DR_16_6_SPS]	= 864,
	[ADS1262_DR_20_SPS]	= 720,
	[ADS1262_DR_50_SPS]	= 288,
	[ADS1262_DR_60_SPS]	= 240,
	[ADS1262_DR_100_SPS]	= 144,
	[ADS1262_DR_400_SPS]	= 36,
	[ADS1262_DR_1200_SPS]	= 12,
	[ADS1262_DR_2400_SPS]	= 6,
	[ADS1262_DR_4800_SPS]	= 3,
	[ADS1262_DR_7200_SPS]	= 2,
	[ADS1262_DR_14400_SPS]	= 1,
	[ADS1262_DR_19200_SPS]	= 1,
	[ADS1262_DR_38400_SPS]	= 1,
};

static int ads1262_find_two(const int (*array)[2], size_t num_elements, int val,
			    int val2)
{
	for (unsigned int i = 0; i < num_elements; i++) {
		if (val == array[i][0] && val2 == array[i][1])
			return i;
	}

	return -EINVAL;
}

static int ads1262_dev_send_cmd(struct ads1262 *st, u8 opcode)
{
	guard(mutex)(&st->xfer_lock);

	return spi_write_then_read(st->spi, &opcode, sizeof(opcode), NULL, 0);
}

static int ads1262_dev_read_by_cmd(struct ads1262 *st, u8 cmd, __be32 *val)
{
	guard(mutex)(&st->xfer_lock);

	return spi_write_then_read(st->spi, &cmd, sizeof(cmd), val, sizeof(*val));
}

static int ads1262_dev_reset(struct ads1262 *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *reset_gpiod;
	int ret;

	reset_gpiod = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpiod))
		return dev_err_probe(dev, PTR_ERR(reset_gpiod),
				     "failed to get reset GPIO\n");

	if (reset_gpiod) {
		/*
		 * Wait a power transition cycle to ensure we are in a
		 * powered-off state after acquiring the RESET GPIO.
		 */
		fsleep(ADS1262_POWER_TRANS_US);

		ret = gpiod_set_value_cansleep(reset_gpiod, 0);
		if (ret)
			return ret;

		fsleep(ADS1262_POWER_TRANS_US);
	} else {
		ret = ads1262_dev_send_cmd(st, ADS1262_OPCODE_RESET);
		if (ret)
			return ret;
		/*
		 * The RESET timing requirement is 8 clock cycles, at the
		 * minimum clock rate this is 8 microseconds.
		 */
		fsleep(8);
	}

	return 0;
}

static int ads1262_dev_start(struct ads1262 *st)
{
	int ret;

	if (st->start_gpiod)
		ret = gpiod_set_value_cansleep(st->start_gpiod, 1);
	else
		ret = ads1262_dev_send_cmd(st, ADS1262_OPCODE_START1);

	return ret;
}

static int ads1262_dev_stop(struct ads1262 *st)
{
	int ret;

	if (st->start_gpiod)
		ret = gpiod_set_value_cansleep(st->start_gpiod, 0);
	else
		ret = ads1262_dev_send_cmd(st, ADS1262_OPCODE_STOP1);

	return ret;
}

static int ads1262_dev_start_one(struct ads1262 *st)
{
	int ret;

	ret = ads1262_dev_start(st);
	if (ret)
		return ret;

	if (st->start_gpiod) {
		/*
		 * The START pulse timing requirement is 4 clock cycles, at the
		 * minimum clock rate this is 4 microseconds.
		 */
		fsleep(4);
		return ads1262_dev_stop(st);
	}

	return 0;
}

static int ads1262_wait_for_conversion(struct ads1262 *st)
{
	u64 max_lat_ms;
	long ret;

	/*
	 * The first conversion latency is affected by the channel's data rate,
	 * filter, the configurable conversion delay and whether chop mode
	 * and/or IDAC rotation mode are enabled.
	 *
	 * The worst possible latency is calculated by taking the lowest data
	 * rate (2.5 SPS) and the sinc4 filter. This gives a latency of 1600 ms
	 * (Table 9-13). Then we scale it by the actual clock rate and multiply
	 * by 4 to account for chop and IDAC rotation modes (Equation 20).
	 */
	max_lat_ms = 4 * div_u64(1600ULL * ADS1262_NOMINAL_CLK_RATE_HZ, st->clk_rate_hz);
	ret = wait_for_completion_interruptible_timeout(&st->drdy,
							msecs_to_jiffies(max_lat_ms));
	if (ret < 0)
		return ret;
	if (ret == 0)
		return -ETIMEDOUT;

	return 0;
}

static int ads1262_channel_enable(struct ads1262 *st,
				  const struct iio_chan_spec *spec)
{
	struct ads1262_channel *chan = &st->channels[spec->scan_index];
	int ret;

	guard(mutex)(&st->xfer_lock);
	guard(mutex)(&st->chan_lock);

	ret = regmap_update_bits(st->regmap, ADS1262_MODE2_REG,
				 ADS1262_MODE2_DR_MASK,
				 FIELD_PREP(ADS1262_MODE2_DR_MASK, chan->data_rate));
	if (ret)
		return ret;

	return regmap_update_bits(st->regmap, ADS1262_INPMUX_REG,
				  ADS1262_INPMUX_MUXN_MASK |
				  ADS1262_INPMUX_MUXP_MASK,
				  FIELD_PREP(ADS1262_INPMUX_MUXN_MASK, spec->channel2) |
				  FIELD_PREP(ADS1262_INPMUX_MUXP_MASK, spec->channel));
}

static int ads1262_set_runmode(struct ads1262 *st, u8 runmode)
{
	guard(mutex)(&st->xfer_lock);

	return regmap_update_bits(st->regmap, ADS1262_MODE0_REG,
				  ADS1262_MODE0_RUNMODE_MASK,
				  FIELD_PREP(ADS1262_MODE0_RUNMODE_MASK, runmode));
}

static int ads1262_channel_read(struct iio_dev *indio_dev,
				const struct iio_chan_spec *spec, __be32 *val)
{
	struct ads1262 *st = iio_priv(indio_dev);
	int ret;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	ret = ads1262_set_runmode(st, ADS1262_RUNMODE_PULSE);
	if (ret)
		return ret;

	ret = ads1262_channel_enable(st, spec);
	if (ret)
		return ret;

	reinit_completion(&st->drdy);

	ret = ads1262_dev_start_one(st);
	if (ret)
		return ret;

	ret = ads1262_wait_for_conversion(st);
	if (ret)
		return ret;

	return ads1262_dev_read_by_cmd(st, ADS1262_OPCODE_RDATA1, val);
}

static int ads1262_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int *val,
			    int *val2, long mask)
{
	struct ads1262 *st = iio_priv(indio_dev);
	struct ads1262_channel *chan_data = &st->channels[chan->scan_index];
	__be32 raw;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = ads1262_channel_read(indio_dev, chan, &raw);
		if (ret)
			return ret;
		*val = be32_to_cpu(raw);

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SAMP_FREQ: {
		guard(mutex)(&st->chan_lock);

		*val = st->sampling_freq_table[chan_data->data_rate][0];
		*val2 = st->sampling_freq_table[chan_data->data_rate][1];

		return IIO_VAL_INT_PLUS_MICRO;
	}

	default:
		return -EOPNOTSUPP;
	}
}

static int ads1262_read_avail(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, const int **vals,
			      int *type, int *length, long mask)
{
	struct ads1262 *st = iio_priv(indio_dev);
	struct ads1262_channel *chan_data = &st->channels[chan->scan_index];

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*type = IIO_VAL_INT_PLUS_MICRO;

		switch (chan_data->filter) {
		case ADS1262_FILTER_FIR:
			*vals = (const int *)st->sampling_freq_fir;
			*length = ARRAY_SIZE(st->sampling_freq_fir) * 2;
			return IIO_AVAIL_LIST;
		default:
			return -EOPNOTSUPP;
		}

	default:
		return -EOPNOTSUPP;
	}
}

static int ads1262_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan, int val,
			     int val2, long mask)
{
	struct ads1262 *st = iio_priv(indio_dev);
	struct ads1262_channel *chan_data = &st->channels[chan->scan_index];
	int ret;

	IIO_DEV_ACQUIRE_DIRECT_MODE(indio_dev, claim);
	if (IIO_DEV_ACQUIRE_FAILED(claim))
		return -EBUSY;

	guard(mutex)(&st->chan_lock);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		switch (chan_data->filter) {
		case ADS1262_FILTER_FIR:
			ret = ads1262_find_two(st->sampling_freq_fir,
					       ARRAY_SIZE(st->sampling_freq_fir),
					       val, val2);
			if (ret < 0)
				return ret;
			break;
		default:
			return -EINVAL;
		}

		ret = ads1262_find_two(st->sampling_freq_table,
				       ARRAY_SIZE(st->sampling_freq_table),
				       val, val2);
		if (ret < 0)
			return ret;

		chan_data->data_rate = ret;

		return 0;

	default:
		return -EOPNOTSUPP;
	}
}

static int ads1262_debugfs_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				      unsigned int writeval, unsigned int *readval)
{
	struct ads1262 *st = iio_priv(indio_dev);

	guard(mutex)(&st->xfer_lock);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ads1262_fwnode_xlate(struct iio_dev *indio_dev,
				const struct fwnode_reference_args *iiospec)
{
	if (iiospec->nargs != 3)
		return -EINVAL;

	/* REVISIT: the auxiliary ADC (ADC2) is currently not supported */
	if (iiospec->args[0] != 1)
		return -EINVAL;

	for (unsigned int i = 0; i < indio_dev->num_channels; i++) {
		if (indio_dev->channels[i].channel == iiospec->args[1] &&
		    indio_dev->channels[i].channel2 == iiospec->args[2])
			return i;
	}

	return -EINVAL;
}

static const struct iio_info ads1262_iio_info = {
	.read_raw = ads1262_read_raw,
	.read_avail = ads1262_read_avail,
	.write_raw = ads1262_write_raw,
	.debugfs_reg_access = ads1262_debugfs_reg_access,
	.fwnode_xlate = ads1262_fwnode_xlate,
};

static irqreturn_t ads1262_irq_handler(int irq, void *dev_id)
{
	struct ads1262 *st = dev_id;

	complete(&st->drdy);

	return IRQ_HANDLED;
}

static int ads1262_dev_configure(struct ads1262 *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	ret = ads1262_dev_reset(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset device\n");

	guard(mutex)(&st->xfer_lock);

	ret = regmap_clear_bits(st->regmap, ADS1262_POWER_REG,
				ADS1262_POWER_RESET_MASK);
	if (ret)
		return ret;

	ret = regmap_set_bits(st->regmap, ADS1262_POWER_REG,
			      ADS1262_POWER_INTREF_MASK);
	if (ret)
		return ret;

	return regmap_clear_bits(st->regmap, ADS1262_INTERFACE_REG,
				 ADS1262_INTERFACE_STATUS_MASK |
				 ADS1262_INTERFACE_CRC_MASK);
}

static bool ads1262_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADS1262_ID_REG ... ADS1262_ADC2FSC1_REG:
		return true;
	default:
		return false;
	}
}

static bool ads1262_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADS1262_POWER_REG ... ADS1262_ADC2FSC1_REG:
		return true;
	default:
		return false;
	}
}

static bool ads1262_volatile_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case ADS1262_POWER_REG:
	case ADS1262_OFCAL0_REG ... ADS1262_FSCAL2_REG:
	case ADS1262_GPIODAT_REG:
	case ADS1262_ADC2OFC0_REG ... ADS1262_ADC2FSC1_REG:
		return true;
	default:
		return false;
	}
}

static const struct reg_default ads1262_reg_defaults[] = {
	{
		.reg = ADS1262_INTERFACE_REG,
		.def = FIELD_PREP_CONST(ADS1262_INTERFACE_STATUS_MASK, 1)
		     | FIELD_PREP_CONST(ADS1262_INTERFACE_CRC_MASK, 1)
	},
	{
		.reg = ADS1262_MODE0_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_MODE1_REG,
		.def = FIELD_PREP_CONST(ADS1262_MODE1_FILTER_MASK, ADS1262_FILTER_FIR)
	},
	{
		.reg = ADS1262_MODE2_REG,
		.def = FIELD_PREP_CONST(ADS1262_MODE2_DR_MASK, ADS1262_DR_20_SPS)
	},
	{
		.reg = ADS1262_INPMUX_REG,
		.def = FIELD_PREP_CONST(ADS1262_INPMUX_MUXN_MASK, ADS1262_INPMUX_AIN1)
	},
	{
		.reg = ADS1262_IDACMUX_REG,
		.def = FIELD_PREP_CONST(ADS1262_IDACMUX_MUX2_MASK, ADS1262_IDACMUX_NO_CONN)
		     | FIELD_PREP_CONST(ADS1262_IDACMUX_MUX1_MASK, ADS1262_IDACMUX_NO_CONN)
	},
	{
		.reg = ADS1262_IDACMAG_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_REFMUX_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_TDACP_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_TDACN_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_GPIOCON_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_GPIODIR_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_ADC2CFG_REG,
		.def = 0x00
	},
	{
		.reg = ADS1262_ADC2MUX_REG,
		.def = FIELD_PREP_CONST(ADS1262_ADC2MUX_MUXN2_MASK, ADS1262_INPMUX_AIN1)
	},
};

static const struct regmap_config ads1262_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.writeable_reg = ads1262_writeable_reg,
	.readable_reg = ads1262_readable_reg,
	.volatile_reg = ads1262_volatile_reg,
	.reg_defaults = ads1262_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(ads1262_reg_defaults),
	.max_register = ADS1262_ADC2FSC1_REG,
	.can_sleep = true,
	.cache_type = REGCACHE_MAPLE,
};

static int ads1262_regmap_read(void *context, const void *reg_buf,
			       size_t reg_size, void *val_buf, size_t val_size)
{
	struct ads1262 *st = context;
	u8 tx[2];

	lockdep_assert_held(&st->xfer_lock);

	/*
	 * The register read operation uses a two byte command header followed
	 * by the register data:
	 *
	 *	byte 0:   RREG opcode | register address
	 *	byte 1:   number of registers to transfer, minus one
	 *	byte 2..: register data
	 */
	tx[0] = *(u8 *)reg_buf | ADS1262_OPCODE_RREG;
	tx[1] = val_size - 1;

	return spi_write_then_read(st->spi, tx, sizeof(tx), val_buf, val_size);
}

static int ads1262_regmap_gather_write(void *context, const void *reg_buf,
				       size_t reg_size, const void *val_buf,
				       size_t val_size)
{
	struct ads1262 *st = context;
	u8 tx[ADS1262_REGMAP_WRITE_SZ + 2];

	lockdep_assert_held(&st->xfer_lock);

	/*
	 * The register write operation uses a two byte command header followed
	 * by the register data:
	 *
	 *	byte 0:   WREG opcode | register address
	 *	byte 1:   number of registers to transfer, minus one
	 *	byte 2..: register data
	 */
	tx[0] = *(u8 *)reg_buf | ADS1262_OPCODE_WREG;
	tx[1] = val_size - 1;
	memcpy(&tx[2], val_buf, val_size);

	return spi_write_then_read(st->spi, tx, 2 + val_size, NULL, 0);
}

static int ads1262_regmap_write(void *context, const void *data, size_t count)
{
	return ads1262_regmap_gather_write(context, data, 1, data + 1, count - 1);
}

static const struct regmap_bus ads1262_regmap_bus = {
	.read = ads1262_regmap_read,
	.gather_write = ads1262_regmap_gather_write,
	.write = ads1262_regmap_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
	.max_raw_write = ADS1262_REGMAP_WRITE_SZ,
};

static void ads1262_populate_samp_freqs(struct ads1262 *st)
{
	u64 freq_Hz, freq_uHz;
	u32 freq_rem;

	for (unsigned int i = 0; i < ARRAY_SIZE(st->sampling_freq_table); i++) {
		freq_uHz = div_u64(mul_u32_u32(st->clk_rate_hz, MICRO),
				   ADS1262_MODULATOR_DIV *
				   ads1262_mod_decimator_a[i] *
				   ads1262_mod_decimator_b[i]);
		freq_Hz = div_u64_rem(freq_uHz, MICRO, &freq_rem);

		st->sampling_freq_table[i][0] = (int)freq_Hz;
		st->sampling_freq_table[i][1] = (int)freq_rem;
	}

	for (unsigned int i = 0; i < ARRAY_SIZE(st->sampling_freq_fir); i++) {
		/*
		 * The FIR filter does not support the 16.6 SPS data rate so it
		 * gets mapped to 20 SPS instead.
		 */
		switch (i) {
		case ADS1262_DR_2_5_SPS ... ADS1262_DR_10_SPS:
			st->sampling_freq_fir[i][0] = st->sampling_freq_table[i][0];
			st->sampling_freq_fir[i][1] = st->sampling_freq_table[i][1];
			break;
		case ADS1262_DR_16_6_SPS:
			st->sampling_freq_fir[i][0] = st->sampling_freq_table[ADS1262_DR_20_SPS][0];
			st->sampling_freq_fir[i][1] = st->sampling_freq_table[ADS1262_DR_20_SPS][1];
			break;
		}
	}
}

static int ads1262_populate_tables(struct iio_dev *indio_dev)
{
	struct ads1262 *st = iio_priv(indio_dev);

	ads1262_populate_samp_freqs(st);

	return 0;
}

static int ads1262_parse_channel_node(struct ads1262 *st,
				      struct iio_chan_spec *spec,
				      struct fwnode_handle *node)
{
	struct device *dev = &st->spi->dev;
	u32 pins[2];
	int ret;

	if (fwnode_property_present(node, "single-channel")) {
		ret = fwnode_property_read_u32(node, "single-channel", &pins[0]);
		if (ret)
			return dev_err_probe(dev, ret, "%pfwP: failed to read single-channel\n",
					     node);

		pins[1] = ADS1262_INPMUX_AINCOM;

		if (fwnode_property_present(node, "common-mode-channel")) {
			ret = fwnode_property_read_u32(node, "common-mode-channel", &pins[1]);
			if (ret)
				return dev_err_probe(dev, ret,
						     "%pfwP: failed to read common-mode-channel\n",
						     node);
		}
	} else if (fwnode_property_present(node, "diff-channels")) {
		ret = fwnode_property_read_u32_array(node, "diff-channels", pins,
						     ARRAY_SIZE(pins));
		if (ret)
			return dev_err_probe(dev, ret, "%pfwP: failed to read diff-channels\n",
					     node);

		spec->differential = true;
	} else {
		return dev_err_probe(dev, -EINVAL,
				     "%pfwP: one of single-channel or diff-channels is required\n",
				     node);
	}

	if (pins[0] > ADS1262_INPMUX_AINCOM || pins[1] > ADS1262_INPMUX_AINCOM)
		return dev_err_probe(dev, -EINVAL, "%pfwP: input channels not in range\n", node);

	spec->channel = pins[0];
	spec->channel2 = pins[1];

	return 0;
}

static int ads1262_parse_channels(struct iio_dev *indio_dev)
{
	struct ads1262 *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *chan_specs;
	unsigned int num_fw_channels, num_specs;
	unsigned int i;
	u32 reg;
	int ret;

	num_fw_channels = device_get_named_child_node_count(dev, "channel");
	if (num_fw_channels > ADS1262_FW_CHANNEL_COUNT)
		return dev_err_probe(dev, -EINVAL, "too many channels\n");

	/* Account for the monitor channels and timestamp */
	num_specs = num_fw_channels + ADS1262_MON_CHANNEL_COUNT + 1;
	chan_specs = devm_kcalloc(dev, num_specs, sizeof(*chan_specs), GFP_KERNEL);
	if (!chan_specs)
		return -ENOMEM;

	st->num_channels = num_fw_channels + ADS1262_MON_CHANNEL_COUNT;
	st->channels = devm_kcalloc(dev, st->num_channels, sizeof(*st->channels),
				    GFP_KERNEL);
	if (!st->channels)
		return -ENOMEM;

	i = 0;
	device_for_each_named_child_node_scoped(dev, node, "channel") {
		struct iio_chan_spec *spec = &chan_specs[i];
		struct ads1262_channel *chan = &st->channels[i];

		*chan = ads1262_default_channel;

		ret = ads1262_parse_channel_node(st, spec, node);
		if (ret)
			return ret;

		ret = fwnode_property_read_u32(node, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret, "%pfwP: failed to read reg\n",
					     node);

		spec->type = IIO_VOLTAGE;
		spec->indexed = true;
		spec->scan_index = i;
		spec->address = reg;
		spec->scan_type = (struct iio_scan_type) {
			.format = IIO_SCAN_FORMAT_SIGNED_INT,
			.realbits = ADS1262_ADC1_RESOLUTION,
			.storagebits = 32,
			.endianness = IIO_BE,
		};
		spec->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					   BIT(IIO_CHAN_INFO_SAMP_FREQ);
		spec->info_mask_separate_available = BIT(IIO_CHAN_INFO_SAMP_FREQ);

		i++;
	}

	memcpy(&chan_specs[i], ads1262_monitor_chan_specs,
	       sizeof(ads1262_monitor_chan_specs));

	for (unsigned int mon = 0; mon < ADS1262_MON_CHANNEL_COUNT; mon++) {
		st->channels[i] = ads1262_default_channel;
		chan_specs[i].scan_index = i;
		i++;
	}

	chan_specs[i] = IIO_CHAN_SOFT_TIMESTAMP(i);
	i++;

	indio_dev->channels = chan_specs;
	indio_dev->num_channels = i;

	return 0;
}

static int ads1262_supplies_setup(struct ads1262 *st)
{
	struct device *dev = &st->spi->dev;
	int ret;

	ret = devm_regulator_get_enable(dev, "dvdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to get dvdd regulator\n");

	ret = devm_regulator_get_enable(dev, "avdd");
	if (ret < 0)
		return dev_err_probe(dev, ret, "failed to get avdd regulator\n");

	ret = devm_regulator_get_enable_optional(dev, "avss");
	if (ret < 0 && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to get avss regulator\n");

	fsleep(ADS1262_POWER_TRANS_US);

	return 0;
}

static int ads1262_spi_probe(struct spi_device *spi)
{
	const struct ads1262_chip_info *info;
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ads1262 *st;
	unsigned long rate;
	struct clk *clk;
	int irq;
	int ret;

	info = spi_get_device_match_data(spi);
	if (!info)
		return -ENODATA;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;
	indio_dev->name = info->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ads1262_iio_info;

	st = iio_priv(indio_dev);
	st->spi = spi;
	init_completion(&st->drdy);

	ret = devm_mutex_init(dev, &st->chan_lock);
	if (ret)
		return ret;
	ret = devm_mutex_init(dev, &st->xfer_lock);
	if (ret)
		return ret;

	ret = ads1262_parse_channels(indio_dev);
	if (ret)
		return ret;

	ret = ads1262_supplies_setup(st);
	if (ret)
		return ret;

	clk = devm_clk_get_optional_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to get external clock\n");

	rate = clk_get_rate(clk);
	if (clk && !rate)
		return dev_err_probe(dev, -EINVAL, "failed to get clock rate\n");
	st->clk_rate_hz = rate ?: ADS1262_NOMINAL_CLK_RATE_HZ;

	st->start_gpiod = devm_gpiod_get_optional(dev, "start", GPIOD_OUT_LOW);
	if (IS_ERR(st->start_gpiod))
		return dev_err_probe(dev, PTR_ERR(st->start_gpiod),
				     "failed to get start GPIO\n");

	st->regmap = devm_regmap_init(dev, &ads1262_regmap_bus, st,
				      &ads1262_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = ads1262_dev_configure(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure device\n");

	ret = ads1262_populate_tables(indio_dev);
	if (ret)
		return ret;

	/*
	 * REVISIT: This chip has software polling capabilities, which could be
	 * used to stop depending on the DRDY signal.
	 *
	 * Additionally, the MISO pin also can be used as a DRDY IRQ, in which
	 * case the interrupt would be named 'doutdrdy', but requires extra
	 * timing and synchronization considerations to be reliable.
	 */
	irq = fwnode_irq_get_byname(dev_fwnode(dev), "drdy");
	if (irq < 0)
		return dev_err_probe(dev, irq,
				     "the 'drdy' IRQ is currently required for operation\n");

	ret = devm_request_irq(dev, irq, ads1262_irq_handler, IRQF_NO_THREAD,
			       indio_dev->name, st);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct ads1262_chip_info ads1262_chip_info = {
	.name = "ads1262",
};

static const struct of_device_id ads1262_of_match[] = {
	{ .compatible = "ti,ads1262", .data = &ads1262_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(of, ads1262_of_match);

static const struct spi_device_id ads1262_spi_match[] = {
	{ .name = "ads1262", .driver_data = (kernel_ulong_t)&ads1262_chip_info },
	{ }
};
MODULE_DEVICE_TABLE(spi, ads1262_spi_match);

static struct spi_driver ads1262_spi_driver = {
	.driver = {
		.name = "ads1262",
		.of_match_table = ads1262_of_match,
	},
	.probe = ads1262_spi_probe,
	.id_table = ads1262_spi_match,
};
module_spi_driver(ads1262_spi_driver);

MODULE_DESCRIPTION("Texas Instruments ADS1262 ADC driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kurt Borja <kuurtb@gmail.com>");
