// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD4630 SPI ADC driver
 *
 * Copyright 2022 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/limits.h>
#include <linux/kconfig.h>
#include <linux/kernel.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/spi/offload/consumer.h>
#include <linux/util_macros.h>
#include <linux/units.h>
#include <linux/types.h>

#define AD4630_REG_INTERFACE_CONFIG_A	0x00
#define AD4630_REG_INTERFACE_CONFIG_B	0x01
#define AD4630_REG_DEVICE_CONFIG	0x02
#define AD4630_REG_CHIP_TYPE		0x03
#define AD4630_REG_PRODUCT_ID_L		0x04
#define AD4630_REG_PRODUCT_ID_H		0x05
#define AD4630_REG_CHIP_GRADE		0x06
#define AD4630_REG_SCRATCH_PAD		0x0A
#define AD4630_REG_SPI_REVISION		0x0B
#define AD4630_REG_VENDOR_L		0x0C
#define AD4630_REG_VENDOR_H		0x0D
#define AD4630_REG_STREAM_MODE		0x0E
#define AD4630_REG_EXIT_CFG_MODE	0x14
#define AD4630_REG_AVG			0x15
#define AD4630_REG_OFFSET_X0_0		0x16
#define AD4630_REG_OFFSET_X0_1		0x17
#define AD4630_REG_OFFSET_X0_2		0x18
#define AD4630_REG_OFFSET_X1_0		0x19
#define AD4630_REG_OFFSET_X1_1		0x1A
#define AD4630_REG_OFFSET_X1_2		0x1B
#define AD4630_REG_GAIN_X0_LSB		0x1C
#define AD4630_REG_GAIN_X0_MSB		0x1D
#define AD4630_REG_GAIN_X1_LSB		0x1E
#define AD4630_REG_GAIN_X1_MSB		0x1F
#define AD4630_REG_MODES		0x20
#define AD4630_REG_OSCILATOR		0x21
#define AD4630_REG_IO			0x22
#define AD4630_REG_PAT0			0x23
#define AD4630_REG_PAT1			0x24
#define AD4630_REG_PAT2			0x25
#define AD4630_REG_PAT3			0x26
#define AD4630_REG_DIG_DIAG		0x34
#define AD4630_REG_DIG_ERR		0x35
/* INTERFACE_CONFIG_A */
#define AD4630_SW_RESET			(BIT(0) | BIT(7))
/* CHIP GRADE */
#define AD4630_MSK_CHIP_GRADE		GENMASK(7, 3)
/* MODES */
#define AD4630_LANE_MODE_MSK		GENMASK(7, 6)
#define AD4630_CLK_MODE_MSK		GENMASK(5, 4)
#define AD4630_DATA_RATE_MODE_MSK	BIT(3)
#define AD4630_OUT_DATA_MODE_MSK	GENMASK(2, 0)
/* AVG */
#define AD4630_REG_AVG_MASK_AVG_SYNC	BIT(7)
#define AD4630_AVG_AVG_VAL		GENMASK(4, 0)
/* OFFSET */
#define AD4630_REG_CHAN_OFFSET(ch)	(AD4630_REG_OFFSET_X0_2 + 3 * (ch))
/* HARDWARE_GAIN */
#define AD4630_REG_CHAN_GAIN(ch)	(AD4630_REG_GAIN_X0_MSB + 2 * (ch))
#define AD4630_GAIN_MAX			1999970
#define ADAQ4224_GAIN_MAX_NANO		6670000000
/* POWER MODE*/
#define AD4630_POWER_MODE_MSK		GENMASK(1, 0)
#define AD4630_LOW_POWER_MODE		3
/* SPI transfer */
#define AD4630_SPI_REG_ACCESS_SPEED	40000000UL
#define AD4630_SPI_SAMPLING_SPEED	80000000UL
/* sequence starting with "1 0 1" to enable reg access */
#define AD4630_REG_ACCESS		0x2000
/* Sampling timing */
#define AD4630_MAX_RATE_1_LANE		1750000
#define AD4630_MAX_RATE			2000000
#define AD4630_TCNV_HIGH_NS		10
/* Datasheet says 9.8ns, so use the closest integer value */
#define AD4630_TQUIET_CNV_DELAY_NS	10

#define AD4630_MAX_CHANNEL_NR		3
#define AD4630_VREF_MIN			(4096 * MILLI)
#define AD4630_VREF_MAX			(5000 * MILLI)

#define AD4630_CHAN_INFO_NONE		0

#define ADAQ4224_PGA_PINS		2
#define ADAQ4224_PGA_1_BITMAP		0
#define ADAQ4224_PGA_2_BITMAP		BIT(0)
#define ADAQ4224_PGA_3_BITMAP		BIT(1)
#define ADAQ4224_PGA_4_BITMAP		GENMASK(1, 0)

enum {
	AD4630_ONE_LANE_PER_CH,
	AD4630_TWO_LANES_PER_CH,
	AD4630_FOUR_LANES_PER_CH,
	AD4630_SHARED_TWO_CH,
};

enum {
	AD4630_16_DIFF = 0x00,
	AD4630_24_DIFF = 0x00,
	AD4630_16_DIFF_8_COM = 0x01,
	AD4630_24_DIFF_8_COM = 0x02,
	AD4630_30_AVERAGED_DIFF = 0x03,
	AD4630_32_PATTERN = 0x04
};

enum {
	AD4630_SPI_COMPATIBLE_MODE,
	AD4630_ECHO_CLOCK_MODE,
	AD4630_CLOCK_HOST_MODE,
};

enum {
	ID_AD4030_24,
	ID_AD4032_24,
	ID_AD4630_16,
	ID_AD4632_16,
	ID_AD4630_24,
	ID_AD4632_24,
	ID_ADAQ4216,
	ID_ADAQ4220,
	ID_ADAQ4224,
};

enum {
	AD4630_033_GAIN = 0,
	AD4630_056_GAIN = 1,
	AD4630_222_GAIN = 2,
	AD4630_667_GAIN = 3,
	AD4630_MAX_PGA,
};

/*
 * Gains computed as fractions of 1000 so they can be expressed by integers.
 */
static const int ad4630_gains[4] = {
	330, 560, 2220, 6670
};

static const int ad4630_average_modes[] = {
	1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384,
	32768, 65536
};

/*
 * Gains stored and computed as fractions to avoid introducing rounding erros.
 */
static const int ad4630_gains_frac[4][2] = {
	[AD4630_033_GAIN] = { 1, 3 },
	[AD4630_056_GAIN] = { 5, 9 },
	[AD4630_222_GAIN] = { 20, 9 },
	[AD4630_667_GAIN] = { 20, 3 },
};

struct ad4630_out_mode {
	const struct iio_chan_spec channels[AD4630_MAX_CHANNEL_NR];
	u32 data_width;
};

struct ad4630_chip_info {
	const unsigned long *available_masks;
	const struct ad4630_out_mode *modes;
	const char *name;
	unsigned long out_modes_mask;
	int min_offset;
	int max_offset;
	u16 base_word_len;
	u8 grade;
	u8 n_channels;
	bool has_pga;
};

struct ad4630_state {
	const struct ad4630_chip_info *chip;
	struct regulator_bulk_data regulators[3];
	struct pwm_device *conv_trigger;
	struct pwm_waveform conv_wf;
	struct gpio_descs *pga_gpios;
	struct spi_device *spi;
	struct regmap *regmap;

	int vref;
	int vio;
	int pga_idx;
	int scale_tbl[ARRAY_SIZE(ad4630_gains)][2];
	unsigned int out_data;
	unsigned int max_rate;

	/* offload sampling spi message */
	struct spi_transfer offload_xfer;
	struct spi_message offload_msg;
	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_offload_trigger_config offload_trigger_config;
	bool test_pattern_en;
	u8 bits_per_word;
	u8 pattern_bits_per_word;

	u8 tx_data[6] __aligned(ARCH_KMALLOC_MINALIGN);
	u8 rx_data[6];
};

static int ad4630_spi_read(void *context, const void *reg, size_t reg_size,
			   void *val, size_t val_size)
{
	struct ad4630_state *st = context;
	struct spi_transfer xfer = {
		.speed_hz = AD4630_SPI_REG_ACCESS_SPEED,
		.tx_buf = st->tx_data,
		.rx_buf = st->rx_data,
		.len = reg_size + val_size,
	};
	int ret;

	memcpy(st->tx_data, reg, reg_size);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	memcpy(val, &st->rx_data[2], val_size);

	return ret;
}

static int ad4630_spi_write(void *context, const void *data, size_t count)
{
	const struct ad4630_state *st = context;

	return spi_write(st->spi, data, count);
}

static int ad4630_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	const struct ad4630_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static void ad4630_get_sampling_freq(const struct ad4630_state *st, int *freq)
{
	*freq = DIV_ROUND_CLOSEST_ULL(NANO, st->conv_wf.period_length_ns);
}

static int ad4630_get_chan_gain(struct iio_dev *indio_dev, int ch, int *val)
{
	const struct ad4630_state *st = iio_priv(indio_dev);
	__be16 gain;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_bulk_read(st->regmap, AD4630_REG_CHAN_GAIN(ch), &gain, 2);
	if (ret)
		goto out_error;

	*val = DIV_ROUND_CLOSEST_ULL(be16_to_cpu(gain) * 1000000ULL, 0x8000);

out_error:
	iio_device_release_direct_mode(indio_dev);
	return ret;
}

static int ad4630_get_chan_offset(struct iio_dev *indio_dev, int ch, int *val)
{
	const struct ad4630_state *st = iio_priv(indio_dev);
	__be32 offset;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_bulk_read(st->regmap, AD4630_REG_CHAN_OFFSET(ch),
			       &offset, 3);
	if (ret)
		goto out_error;

	*val = be32_to_cpu(offset) >> 8;

	/* For 16bit chips, the third byte is RESERVED. We can read it but it's a don't care...*/
	if (st->chip->base_word_len == 16)
		*val = *val >> 8;

	*val = sign_extend32(*val, st->chip->base_word_len - 1);
out_error:
	iio_device_release_direct_mode(indio_dev);
	return ret;
}

static int ad4630_get_avg_frame_len(struct iio_dev *dev, unsigned int *avg_len)
{
	struct ad4630_state *st = iio_priv(dev);
	unsigned int  val;
	int ret;

	ret = iio_device_claim_direct_mode(dev);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4630_REG_AVG, &val);
	if (ret)
		goto out;

	*avg_len = 1 << FIELD_GET(AD4630_AVG_AVG_VAL, val);
out:
	iio_device_release_direct_mode(dev);

	return 0;
}

static int ad4630_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long info)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	unsigned int temp;
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ad4630_get_sampling_freq(st, val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (st->chip->has_pga) {
			*val = st->scale_tbl[st->pga_idx][0];
			*val2 = st->scale_tbl[st->pga_idx][1];
			return IIO_VAL_INT_PLUS_NANO;
		}
		*val = (st->vref * 2) / 1000;
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = ad4630_get_chan_gain(indio_dev, chan->channel, &temp);
		if (ret)
			return ret;

		*val = temp / 1000000;
		*val2 = temp % 1000000;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = ad4630_get_chan_offset(indio_dev, chan->channel, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		ret = ad4630_get_avg_frame_len(indio_dev, &temp);
		if (ret)
			return ret;

		*val = temp;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4630_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	struct ad4630_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (int *)st->scale_tbl;
		*length = ARRAY_SIZE(ad4630_gains) * 2;
		*type = IIO_VAL_INT_PLUS_NANO;
		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*vals = ad4630_average_modes;
		*type = IIO_VAL_INT;
		*length = ARRAY_SIZE(ad4630_average_modes);
		return IIO_AVAIL_LIST;
	default:
		return -EINVAL;
	}
}

static int __ad4630_set_sampling_freq(struct ad4630_state *st, unsigned int freq)
{
	struct spi_offload_trigger_config *config = &st->offload_trigger_config;
	struct pwm_waveform conv_wf = { };
	u64 offload_period_ns;
	u64 offload_offset_ns;
	u32 mode;
	int ret;
	u64 target = AD4630_TCNV_HIGH_NS;

	conv_wf.period_length_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, freq);
	/*
	 * The datasheet lists a minimum time of 9.8 ns, but no maximum. If the
	 * rounded PWM's value is less than 10, increase the target value by 10
	 * and attempt to round the waveform again, until the value is at least
	 * 10 ns. Use a separate variable to represent the target in case the
	 * rounding is severe enough to keep putting the first few results under
	 * the minimum 10ns condition checked by the while loop.
	 */
	do {
		conv_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(st->conv_trigger, &conv_wf);
		if (ret)
			return ret;
		target += 10;
	} while (conv_wf.duty_length_ns < 10);

	offload_period_ns = conv_wf.period_length_ns;

	ret = regmap_read(st->regmap, AD4630_REG_MODES, &mode);
	if (ret)
		return ret;
	if (FIELD_GET(AD4630_OUT_DATA_MODE_MSK, mode) == AD4630_30_AVERAGED_DIFF) {
		u32 avg;

		ret = regmap_read(st->regmap, AD4630_REG_AVG, &avg);
		if (ret)
			return ret;

		offload_period_ns <<= FIELD_GET(AD4630_AVG_AVG_VAL, avg);
	}

	config->periodic.frequency_hz =  DIV_ROUND_UP_ULL(NSEC_PER_SEC,
			offload_period_ns);

	/*
	 * The hardware does the capture on zone 2 (when spi trigger PWM
	 * is used). This means that the spi trigger signal should happen at
	 * tsync + tquiet_con_delay being tsync the conversion signal period
	 * and tquiet_con_delay 9.8ns. Hence set the PWM phase accordingly.
	 *
	 * The PWM waveform API only supports nanosecond resolution right now,
	 * so round this setting to the closest available value.
	 */
	offload_offset_ns = AD4630_TQUIET_CNV_DELAY_NS;
	do {
		config->periodic.offset_ns = offload_offset_ns;
		ret = spi_offload_trigger_validate(st->offload_trigger, config);
		if (ret)
			return ret;
		offload_offset_ns += 10;

	} while (config->periodic.offset_ns < AD4630_TQUIET_CNV_DELAY_NS);

	st->conv_wf = conv_wf;

	return 0;
}

static int ad4630_set_sampling_freq(struct iio_dev *indio_dev, unsigned int freq)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	int ret;

	if (!freq || freq > st->max_rate)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = __ad4630_set_sampling_freq(st, freq);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4630_set_chan_offset(struct iio_dev *indio_dev, int ch, int offset)
{
	const struct ad4630_state *st = iio_priv(indio_dev);
	__be32 val;
	int ret;

	if (offset < st->chip->min_offset || offset > st->chip->max_offset)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (st->chip->base_word_len == 16)
		val = cpu_to_be32(offset << 16);
	else
		val = cpu_to_be32(offset << 8);

	ret = regmap_bulk_write(st->regmap, AD4630_REG_CHAN_OFFSET(ch),
				&val, 3);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static void ad4630_fill_scale_tbl(struct ad4630_state *st)
{
	int val, val2, tmp0, tmp1, i;
	u64 tmp2;

	val2 = st->chip->modes[st->out_data].channels->scan_type.realbits;
	for (i = 0; i < ARRAY_SIZE(ad4630_gains); i++) {
		val = (st->vref * 2) / 1000;
		/* Multiply by MILLI here to avoid losing precision */
		val = mult_frac(val, ad4630_gains_frac[i][1] * MILLI,
				ad4630_gains_frac[i][0]);
		/* Would multiply by NANO here but we already multiplied by MILLI */
		tmp2 = shift_right((u64)val * MICRO, val2);
		tmp0 = (int)div_s64_rem(tmp2, NANO, &tmp1);
		st->scale_tbl[i][0] = tmp0; /* Integer part */
		st->scale_tbl[i][1] = abs(tmp1); /* Fractional part */
	}
}

static int ad4630_calc_pga_gain(int gain_int, int gain_fract, int vref,
				int precision)
{
	u64 gain_nano, tmp;
	int gain_idx;

	gain_nano = gain_int * NANO + gain_fract;

	if (gain_nano < 0 || gain_nano > ADAQ4224_GAIN_MAX_NANO)
		return -EINVAL;

	tmp = DIV_ROUND_CLOSEST_ULL(gain_nano << precision, NANO);
	gain_nano = DIV_ROUND_CLOSEST_ULL(vref * 2, tmp);
	gain_idx = find_closest(gain_nano, ad4630_gains,
				ARRAY_SIZE(ad4630_gains));

	return gain_idx;
}

static int ad4630_set_pga_gain(struct iio_dev *indio_dev, int gain_idx)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	DECLARE_BITMAP(values, ADAQ4224_PGA_PINS);
	int ret;

	/* Set appropriate status for A0, A1 pins according to requested gain */
	switch (gain_idx) {
	case 0:
		values[0] = ADAQ4224_PGA_1_BITMAP;
		break;
	case 1:
		values[0] = ADAQ4224_PGA_2_BITMAP;
		break;
	case 2:
		values[0] = ADAQ4224_PGA_3_BITMAP;
		break;
	case 3:
		values[0] = ADAQ4224_PGA_4_BITMAP;
		break;
	default:
		return -EINVAL;
	}

	ret = gpiod_set_array_value_cansleep(ADAQ4224_PGA_PINS,
					     st->pga_gpios->desc,
					     st->pga_gpios->info, values);
	if (!ret)
		st->pga_idx = gain_idx;

	return ret;
}

static int ad4630_set_chan_gain(struct iio_dev *indio_dev, int ch,
				int gain_int, int gain_frac)
{
	const struct ad4630_state *st = iio_priv(indio_dev);
	__be16 val;
	u64 gain;
	int ret;

	gain = gain_int * MICRO + gain_frac;

	if (gain < 0 || gain > AD4630_GAIN_MAX)
		return -EINVAL;

	gain = DIV_ROUND_CLOSEST_ULL(gain * 0x8000, 1000000);

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	val = cpu_to_be16(gain);
	ret = regmap_bulk_write(st->regmap, AD4630_REG_CHAN_GAIN(ch), &val, 2);
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4630_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT_PLUS_NANO;
	default:
		return IIO_VAL_INT_PLUS_MICRO;
	}

	return -EINVAL;
}

static int ad4630_set_avg_frame_len(struct iio_dev *dev,
				    unsigned int avg_val)
{
	struct ad4630_state *st = iio_priv(dev);
	unsigned int avg_log2 = ilog2(avg_val);
	unsigned int last_avg_idx = ARRAY_SIZE(ad4630_average_modes) - 1;
	int ret, freq;

	if (avg_val < 0 || avg_val > ad4630_average_modes[last_avg_idx])
		return -EINVAL;

	ret = iio_device_claim_direct_mode(dev);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4630_REG_AVG,
		AD4630_REG_AVG_MASK_AVG_SYNC |
		FIELD_PREP(AD4630_AVG_AVG_VAL, avg_log2));
	if (ret)
		goto out_error;

	/*re-evaluate fetch trigger*/
	ad4630_get_sampling_freq(st, &freq);
	ret = __ad4630_set_sampling_freq(st, freq);
out_error:
	iio_device_release_direct_mode(dev);

	return ret;
}

static int ad4630_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	int gain_idx;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4630_set_sampling_freq(indio_dev, val);
	case IIO_CHAN_INFO_SCALE:
		gain_idx = ad4630_calc_pga_gain(val, val2, st->vref,
						chan->scan_type.realbits);
		return ad4630_set_pga_gain(indio_dev, gain_idx);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4630_set_chan_gain(indio_dev, chan->channel, val,
					    val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad4630_set_chan_offset(indio_dev, chan->channel, val);
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		return ad4630_set_avg_frame_len(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ad4630_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	int ret, read_ret;
	u32 dummy;

	ret = pm_runtime_resume_and_get(&st->spi->dev);
	if (ret < 0)
		return ret;

	if (st->test_pattern_en)
		st->offload_xfer.bits_per_word  = st->pattern_bits_per_word;
	else
		st->offload_xfer.bits_per_word  = st->bits_per_word;

	ret = regmap_write(st->regmap, AD4630_REG_EXIT_CFG_MODE, BIT(0));
	if (ret)
		goto out_error;

	st->offload_msg.offload = st->offload;
	ret = spi_optimize_message(st->spi, &st->offload_msg);
	if (ret < 0)
		goto out_reset_mode;

	spi_bus_lock(st->spi->controller);

	ret = pwm_set_waveform_might_sleep(st->conv_trigger, &st->conv_wf, false);
	if (ret)
		goto out_unlock;

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger,
		&st->offload_trigger_config);
	if (ret)
		goto out_pwm_disable;
	return 0;
out_pwm_disable:
	pwm_disable(st->conv_trigger);
out_unlock:
	spi_bus_unlock(st->spi->controller);
	spi_unoptimize_message(&st->offload_msg);
out_reset_mode:
	/* read this to reenter register configuration mode */
	read_ret = regmap_read(st->regmap, AD4630_REG_ACCESS, &dummy);
	if (read_ret)
		dev_warn(&st->spi->dev, "couldn't reenter register configuration mode\n");
out_error:
	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

static int ad4630_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	u32 dummy;
	int ret;

	pwm_disable(st->conv_trigger);

	spi_offload_trigger_disable(st->offload, st->offload_trigger);
	spi_bus_unlock(st->spi->controller);

	spi_unoptimize_message(&st->offload_msg);
	ret = regmap_read(st->regmap, AD4630_REG_ACCESS, &dummy);
	if (ret)
		dev_warn(&st->spi->dev, "couldn't reenter register configuration mode\n");

	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);
	return ret;
}

#define AD4630_CHAN(_idx, _msk_avail, _storage, _real, _shift, _msk_type) {	\
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |		\
			BIT(IIO_CHAN_INFO_CALIBBIAS),			\
	.info_mask_separate_available = _msk_avail,			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.info_mask_shared_by_all_available =				\
				BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
	.info_mask_shared_by_type = _msk_type |				\
				BIT(IIO_CHAN_INFO_SCALE),		\
	.info_mask_shared_by_type_available = _msk_type,		\
	.type = IIO_VOLTAGE,						\
	.indexed = 1,							\
	.channel = _idx,						\
	.scan_index = _idx,						\
	.scan_type = {							\
		.sign = 's',						\
		.storagebits = _storage,				\
		.realbits = _real,					\
		.shift = _shift,					\
	},								\
}

/*
 * We need the sample size to be 64 bytes when both channels are enabled as the
 * HW will always fill in the DMA bus which is 64bits. If we had just 16 bits
 * of storage (for example on the AD4630_16_DIFF mode for ad4630-16 ), we would
 * have a sample  size of 32bits having the 16MSb set to 0 (in theory channel_2
 * data) as the HW fills in the channel_1 sample to get the 32bits per channel
 * storage.
 */
static const struct ad4630_out_mode ad4030_24_modes[] = {
	[AD4630_24_DIFF] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 24, 0, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 16, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_24_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 24, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 32,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
		},
		.data_width = 32,
	}
};

static const struct ad4630_out_mode ad4630_16_modes[] = {
	[AD4630_16_DIFF] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 16, 0, AD4630_CHAN_INFO_NONE),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 16, 0, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 16,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 16, 8, AD4630_CHAN_INFO_NONE),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 16, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
		},
		.data_width = 32,
	}
};

static const struct ad4630_out_mode ad4630_24_modes[] = {
	[AD4630_24_DIFF] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 24, 0, AD4630_CHAN_INFO_NONE),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 24, 0, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 16, 8, AD4630_CHAN_INFO_NONE),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 16, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_24_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 24, 8, AD4630_CHAN_INFO_NONE),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 24, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 32,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, AD4630_CHAN_INFO_NONE, 32, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
			AD4630_CHAN(1, AD4630_CHAN_INFO_NONE, 32, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
		},
		.data_width = 32,
	}
};

static const struct ad4630_out_mode adaq4216_modes[] = {
	[AD4630_16_DIFF] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 16, 0, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 16,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 16, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
		},
		.data_width = 32,
	}
};

static const struct ad4630_out_mode adaq4220_modes[] = {
	[AD4630_16_DIFF] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 20, 0, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 20,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 16, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
		},
		.data_width = 32,
	}
};

static const struct ad4630_out_mode adaq4224_modes[] = {
	[AD4630_24_DIFF] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 24, 0, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_16_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 16, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 24,
	},
	[AD4630_24_DIFF_8_COM] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 24, 8, AD4630_CHAN_INFO_NONE),
		},
		.data_width = 32,
	},
	[AD4630_30_AVERAGED_DIFF] = {
		.channels = {
			AD4630_CHAN(0, BIT(IIO_CHAN_INFO_SCALE), 64, 30, 2,
				BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO)),
		},
		.data_width = 32,
	}
};

/* all channels must be enabled */
static const unsigned long ad4630_channel_masks[] = {
	GENMASK(1, 0),
	0,
};

static const unsigned long ad4030_channel_masks[] = {
	BIT(0),
	0,
};

/* test pattern is treated as an additional channel */
static const struct ad4630_chip_info ad4630_chip_info[] = {
	[ID_AD4030_24] = {
		.available_masks = ad4030_channel_masks,
		.modes = ad4030_24_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "ad4030-24",
		.grade = 0x10,
		.min_offset = (int)BIT(23) * -1,
		.max_offset = BIT(23) - 1,
		.base_word_len = 24,
		.n_channels = 1,
	},
	[ID_AD4032_24] = {
		.available_masks = ad4030_channel_masks,
		.modes = ad4030_24_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "ad4032-24",
		.grade = 0x12,
		.min_offset = (int)BIT(23) * -1,
		.max_offset = BIT(23) - 1,
		.base_word_len = 24,
		.n_channels = 1,
	},
	[ID_AD4630_16] = {
		.available_masks = ad4630_channel_masks,
		.modes = ad4630_16_modes,
		.out_modes_mask = BIT(3) | GENMASK(1, 0),
		.name = "ad4630-16",
		.grade = 0x03,
		.min_offset = (int)BIT(15) * -1,
		.max_offset = BIT(15) - 1,
		.base_word_len = 16,
		.n_channels = 2,
	},
	[ID_AD4632_16] = {
		.available_masks = ad4630_channel_masks,
		.modes = ad4630_16_modes,
		.out_modes_mask = BIT(3) | GENMASK(1, 0),
		.name = "ad4632-16",
		.grade = 0x05,
		.min_offset = (int)BIT(15) * -1,
		.max_offset = BIT(15) - 1,
		.base_word_len = 16,
		.n_channels = 2,
	},
	[ID_AD4630_24] = {
		.available_masks = ad4630_channel_masks,
		.modes = ad4630_24_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "ad4630-24",
		.min_offset = (int)BIT(23) * -1,
		.max_offset = BIT(23) - 1,
		.base_word_len = 24,
		.n_channels = 2,
	},
	[ID_AD4632_24] = {
		.available_masks = ad4630_channel_masks,
		.modes = ad4630_24_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "ad4632-24",
		.grade = 0x2,
		.min_offset = (int)BIT(23) * -1,
		.max_offset = BIT(23) - 1,
		.base_word_len = 24,
		.n_channels = 2,
	},
	[ID_ADAQ4216] = {
		.available_masks = ad4030_channel_masks,
		.modes = adaq4216_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "adaq4216",
		.grade = 0x1E,
		.min_offset = (int)BIT(15) * -1,
		.max_offset = BIT(15) - 1,
		.base_word_len = 16,
		.has_pga = true,
		.n_channels = 1,
	},
	[ID_ADAQ4220] = {
		.available_masks = ad4030_channel_masks,
		.modes = adaq4220_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "adaq4220",
		.grade = 0x1D,
		.min_offset = (int)BIT(19) * -1,
		.max_offset = BIT(19) - 1,
		.base_word_len = 20,
		.has_pga = true,
		.n_channels = 1,
	},
	[ID_ADAQ4224] = {
		.available_masks = ad4030_channel_masks,
		.modes = adaq4224_modes,
		.out_modes_mask = GENMASK(3, 0),
		.name = "adaq4224",
		.grade = 0x1C,
		.min_offset = (int)BIT(23) * -1,
		.max_offset = BIT(23) - 1,
		.base_word_len = 24,
		.has_pga = true,
		.n_channels = 1,
	}
};

static void ad4630_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ad4630_pm_disable(void *data)
{
	/* based in upstream pm_runtime_disable_action() */
	pm_runtime_dont_use_autosuspend(data);
	pm_runtime_disable(data);
}

static void ad4630_disable_regulators(void *data)
{
	struct ad4630_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

static int ad4630_regulators_get(struct ad4630_state *st)
{
	struct device *dev = &st->spi->dev;
	struct regulator *ref;
	int ret;

	st->regulators[0].supply = "vdd";
	st->regulators[1].supply = "vdd_1_8";
	st->regulators[2].supply = "vio";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad4630_disable_regulators, st);
	if (ret)
		return ret;

	st->vio = regulator_get_voltage(st->regulators[2].consumer);

	ref = devm_regulator_get_optional(dev, "vref");
	if (IS_ERR(ref)) {
		if (PTR_ERR(ref) != -ENODEV)
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vref regulator");

		/* if not using optional REF, the internal REFIN must be used */
		ref = devm_regulator_get(dev, "vrefin");
		if (IS_ERR(ref))
			return dev_err_probe(dev, PTR_ERR(ref),
					     "Failed to get vrefin regulator");
	}

	ret = regulator_enable(ref);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to enable specified ref supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(dev, ad4630_regulator_disable, ref);
	if (ret)
		return ret;

	st->vref = regulator_get_voltage(ref);
	if (st->vref < AD4630_VREF_MIN || st->vref > AD4630_VREF_MAX)
		return dev_err_probe(dev, -EINVAL, "vref(%d) must be under [%lu %lu]\n",
				     st->vref, AD4630_VREF_MIN, AD4630_VREF_MAX);

	return 0;
}

static int ad4630_reset(const struct ad4630_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *reset;
	u32 dummy;
	int ret;

	reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get reset gpio");

	if (reset) {
		gpiod_set_value_cansleep(reset, 0);
	} else {
		/* Guarantee that we can access the registers */
		ret = regmap_read(st->regmap, AD4630_REG_ACCESS, &dummy);
		if (ret)
			return ret;

		ret = regmap_write(st->regmap, AD4630_REG_INTERFACE_CONFIG_A,
				   AD4630_SW_RESET);
		if (ret)
			return ret;
	}

	fsleep(750);

	/* after reset, default is conversion mode... change to reg access */
	return regmap_read(st->regmap, AD4630_REG_ACCESS, &dummy);
}

static int ad4630_pwm_get(struct ad4630_state *st)
{
	struct device *dev = &st->spi->dev;

	st->conv_trigger = devm_pwm_get(dev, "cnv");
	if (IS_ERR(st->conv_trigger))
		return dev_err_probe(dev, PTR_ERR(st->conv_trigger),
				     "Failed to get cnv pwm\n");

	/*
	 * Preemptively disable the PWM, since we only want to enable it with
	 * the buffer
	 */
	pwm_disable(st->conv_trigger);

	return __ad4630_set_sampling_freq(st, st->max_rate);
}

static void ad4630_prepare_spi_sampling_msg(struct ad4630_state *st,
					    u32 clk_mode, u32 lane_mode,
					    bool data_rate)
{
	const struct ad4630_out_mode *out_mode = &st->chip->modes[st->out_data];
	int data_width = out_mode->data_width;
	st->offload_xfer.speed_hz = AD4630_SPI_SAMPLING_SPEED;

	/*
	 * In host mode, for a 16-bit data-word, the device adds an additional
	 * eight clock pulses for a total of 24 clock pulses.
	 */
	if (clk_mode == AD4630_CLOCK_HOST_MODE && data_width == 16)
		data_width = 24;

	if (lane_mode == AD4630_SHARED_TWO_CH) {
		/*
		 * This means all channels on 1 lane.
		 */
		st->bits_per_word = data_width * st->chip->n_channels;
		st->pattern_bits_per_word = 32 * st->chip->n_channels;
	} else {
		st->bits_per_word  = data_width / (1 << lane_mode);
		st->pattern_bits_per_word  = 32 / (1 << lane_mode);
	}

	if (data_rate) {
		st->bits_per_word  /= 2;
		st->pattern_bits_per_word  /= 2;
	}

	st->offload_xfer.bits_per_word = st->bits_per_word;
	st->offload_xfer.len = roundup_pow_of_two(BITS_TO_BYTES(st->bits_per_word));
	st->offload_xfer.offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
	spi_message_init_with_transfers(&st->offload_msg, &st->offload_xfer, 1);
}

static int ad4630_config(struct ad4630_state *st)
{
	u32 clock_mode = 0, lane_mode = 0, reg_modes = 0, grade;
	bool data_rate;
	struct device *dev = &st->spi->dev;
	int ret;

	ret = regmap_read(st->regmap, AD4630_REG_CHIP_GRADE, &grade);
	if (ret)
		return ret;

	if (st->chip->grade != FIELD_GET(AD4630_MSK_CHIP_GRADE, grade))
		dev_warn(dev, "Unknown grade(%u). Expected(%u)\n", grade,
			 st->chip->grade);

	ret = device_property_read_u32(dev, "adi,lane-mode", &lane_mode);
	if (!ret) {
		if (lane_mode > AD4630_SHARED_TWO_CH)
			return dev_err_probe(dev, -EINVAL, "Invalid lane mode(%u)\n", lane_mode);
		if (lane_mode == AD4630_SHARED_TWO_CH && st->chip->n_channels == 1)
			return dev_err_probe(dev, -EINVAL,
					     "Interleaved lanes not valid for devices with one channel\n");

		reg_modes = FIELD_PREP(AD4630_LANE_MODE_MSK, lane_mode);
	}

	ret = device_property_read_u32(dev, "adi,clock-mode", &clock_mode);
	if (!ret) {
		if (clock_mode > AD4630_CLOCK_HOST_MODE)
			return dev_err_probe(dev, -EINVAL, "Invalid clock mode(%u)\n",
					     clock_mode);

		reg_modes |= FIELD_PREP(AD4630_CLK_MODE_MSK, clock_mode);
	}

	data_rate = device_property_read_bool(dev, "adi,dual-data-rate");
	if (data_rate && clock_mode == AD4630_SPI_COMPATIBLE_MODE)
		return dev_err_probe(dev, -EINVAL, "DDR not allowed when using SPI clock mode");

	reg_modes |= FIELD_PREP(AD4630_DATA_RATE_MODE_MSK, data_rate);

	ret = device_property_read_u32(dev, "adi,out-data-mode", &st->out_data);
	if (!ret) {
		if (st->out_data > AD4630_30_AVERAGED_DIFF ||
		    !test_bit(st->out_data, &st->chip->out_modes_mask))
			return dev_err_probe(dev, -EINVAL, "Invalid out data mode(%u)\n",
					     st->out_data);

		reg_modes |= FIELD_PREP(AD4630_OUT_DATA_MODE_MSK, st->out_data);
	}

	if (st->vio < 1400000) {
		/*
		 * for VIO levels below 1.4 V, the IO2X bit in the output driver
		 * register must be set to 1.
		 */
		ret = regmap_set_bits(st->regmap, AD4630_REG_IO, BIT(0));
		if (ret)
			return ret;
	}

	ret = regmap_write(st->regmap, AD4630_REG_MODES, reg_modes);
	if (ret)
		return ret;

	st->max_rate = AD4630_MAX_RATE;

	ad4630_prepare_spi_sampling_msg(st, clock_mode, lane_mode, data_rate);

	return 0;
}

static const struct iio_buffer_setup_ops ad4630_buffer_setup_ops = {
	.postenable = &ad4630_buffer_postenable,
	.predisable = &ad4630_buffer_predisable,
};

static const struct iio_info ad4630_info = {
	.read_raw = &ad4630_read_raw,
	.read_avail = &ad4630_read_avail,
	.write_raw = &ad4630_write_raw,
	.write_raw_get_fmt = &ad4630_write_raw_get_fmt,
	.debugfs_reg_access = &ad4630_reg_access,
};

static const struct regmap_bus ad4630_regmap_bus = {
	.read = ad4630_spi_read,
	.write = ad4630_spi_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config ad4630_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

static int ad4630_set_test_pattern(void *arg, u64 val)
{
	struct iio_dev *indio_dev = arg;
	struct ad4630_state *st = iio_priv(indio_dev);
	__be32 pattern;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (val > U32_MAX) {
		iio_device_release_direct_mode(indio_dev);
		return -EINVAL;
	}

	pattern = cpu_to_be32(val);
	ret = regmap_bulk_write(st->regmap, AD4630_REG_PAT3, &pattern, 4);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4630_show_test_pattern(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad4630_state *st = iio_priv(indio_dev);
	__be32 pattern;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_bulk_read(st->regmap, AD4630_REG_PAT3, &pattern, 4);
	iio_device_release_direct_mode(indio_dev);
	if (ret)
		return ret;

	*val =  __be32_to_cpu(pattern);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ad4630_test_pattern_fops, ad4630_show_test_pattern,
			 ad4630_set_test_pattern, "%llu\n");

static int ad4630_set_test_pattern_en(void *arg, u64 val)
{
	struct iio_dev *indio_dev = arg;
	struct ad4630_state *st = iio_priv(indio_dev);
	u32 mode;
	int ret, freq;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	if (val)
		mode = FIELD_PREP(AD4630_OUT_DATA_MODE_MSK, AD4630_32_PATTERN);
	else
		mode = FIELD_PREP(AD4630_OUT_DATA_MODE_MSK, st->out_data);

	ret = regmap_update_bits(st->regmap, AD4630_REG_MODES,
				 AD4630_OUT_DATA_MODE_MSK, mode);
	if (ret)
		goto out;

	/*
	 * in average mode the fetch trigger might not follow cnv and needs
	 * to be re-evaluated when switching on/off test mode since there is
	 * no avereging in test mode.
	 */
	if (st->out_data == AD4630_30_AVERAGED_DIFF) {
		ad4630_get_sampling_freq(st, &freq);
		ret = __ad4630_set_sampling_freq(st, freq);
	}
out:
	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4630_show_test_pattern_en(void *arg, u64 *val)
{
	struct iio_dev *indio_dev = arg;
	struct ad4630_state *st = iio_priv(indio_dev);
	u32 mode;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD4630_REG_MODES, &mode);
	iio_device_release_direct_mode(indio_dev);
	if (ret)
		return ret;

	mode = FIELD_GET(AD4630_OUT_DATA_MODE_MSK, mode);
	*val = mode == AD4630_32_PATTERN;

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(ad4630_test_pattern_en_fops, ad4630_show_test_pattern_en,
			 ad4630_set_test_pattern_en, "%llu\n");

static void ad4630_debugs_init(struct iio_dev *indio_dev)
{
	struct dentry *d = iio_get_debugfs_dentry(indio_dev);

	debugfs_create_file_unsafe("test_pattern", 0600, d,
				   indio_dev, &ad4630_test_pattern_fops);
	debugfs_create_file_unsafe("test_pattern_enable", 0600, d,
				   indio_dev, &ad4630_test_pattern_en_fops);
}

static const struct spi_offload_config ad4630_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
		SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

static int ad4630_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct dma_chan *rx_dma;
	struct ad4630_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	spi_set_drvdata(spi, st);

	st->chip = spi_get_device_match_data(spi);
	if (!st->chip)
		return dev_err_probe(dev, -ENODEV,
				     "Could not find chip info data\n");

	st->offload = devm_spi_offload_get(&spi->dev, spi, &ad4630_offload_config);
	if (IS_ERR(st->offload))
		return dev_err_probe(&spi->dev, PTR_ERR(st->offload), "failed to get offload\n");

	/* ad4630 could use the busy signal as trigger, this scenario was not yet tested */
	st->offload_trigger = devm_spi_offload_trigger_get(dev,
			st->offload, SPI_OFFLOAD_TRIGGER_PERIODIC);
	if (IS_ERR(st->offload_trigger))
		return dev_err_probe(dev, PTR_ERR(st->offload_trigger),
					"failed to get offload trigger\n");

	st->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;

	st->regmap = devm_regmap_init(&spi->dev, &ad4630_regmap_bus, st,
				      &ad4630_regmap_config);
	if (IS_ERR(st->regmap))
		dev_err_probe(&spi->dev,  PTR_ERR(st->regmap),
			      "Failed to initialize regmap\n");

	ret = ad4630_regulators_get(st);
	if (ret)
		return ret;

	st->pga_gpios = devm_gpiod_get_array_optional(&spi->dev, "adi,pga",
						      GPIOD_OUT_LOW);

	if (IS_ERR(st->pga_gpios))
		dev_err_probe(&spi->dev, PTR_ERR(st->pga_gpios),
			      "Failed to get PGA GPIOs\n");

	ret = ad4630_reset(st);
	if (ret)
		return ret;

	ret = ad4630_config(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Config failed: %d\n", ret);

	if (st->pga_gpios) {
		ad4630_fill_scale_tbl(st);
		ad4630_set_pga_gain(indio_dev, 0);
	}

	/*
	 * Due to a hardware bug in some chips when using average mode zero
	 * (no averaging), set default averaging mode to 2 samples.
	 */
	ret = regmap_write(st->regmap, AD4630_REG_AVG, 0x01);
	if (ret)
		return ret;

	ret = ad4630_pwm_get(st);
	if (ret)
		return dev_err_probe(&spi->dev, ret,
				     "Failed to get PWM: %d\n", ret);

	indio_dev->name = st->chip->name;
	indio_dev->info = &ad4630_info;
	indio_dev->channels = st->chip->modes[st->out_data].channels;
	indio_dev->num_channels = st->chip->n_channels;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->available_scan_masks = st->chip->available_masks;
	indio_dev->setup_ops = &ad4630_buffer_setup_ops;

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(dev, PTR_ERR(rx_dma),
			"failed to get offload RX DMA\n");

	ret = devm_iio_dmaengine_buffer_setup_with_handle(dev, indio_dev,
		rx_dma, IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get DMA buffer\n");

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = devm_add_action_or_reset(dev, ad4630_pm_disable, &spi->dev);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		ad4630_debugs_init(indio_dev);

	return 0;
}

static int ad4630_runtime_suspend(struct device *dev)
{
	u32 val = FIELD_PREP(AD4630_POWER_MODE_MSK, AD4630_LOW_POWER_MODE);
	struct ad4630_state *st = dev_get_drvdata(dev);

	return regmap_write(st->regmap, AD4630_REG_DEVICE_CONFIG, val);
}

static int ad4630_runtime_resume(struct device *dev)
{
	struct ad4630_state *st = dev_get_drvdata(dev);
	int ret;

	ret = regmap_write(st->regmap, AD4630_REG_DEVICE_CONFIG,
			   FIELD_PREP(AD4630_POWER_MODE_MSK, 0));
	if (ret)
		return ret;

	fsleep(30);

	return 0;
}

static const struct dev_pm_ops ad4630_pm_ops = {
	SET_RUNTIME_PM_OPS(ad4630_runtime_suspend, ad4630_runtime_resume, NULL)
};

static const struct spi_device_id ad4630_id_table[] = {
	{ "ad4030-24", (kernel_ulong_t)&ad4630_chip_info[ID_AD4030_24] },
	{ "ad4032-24", (kernel_ulong_t)&ad4630_chip_info[ID_AD4032_24] },
	{ "ad4630-16", (kernel_ulong_t)&ad4630_chip_info[ID_AD4630_16] },
	{ "ad4632-16", (kernel_ulong_t)&ad4630_chip_info[ID_AD4632_16] },
	{ "ad4630-24", (kernel_ulong_t)&ad4630_chip_info[ID_AD4630_24] },
	{ "ad4632-24", (kernel_ulong_t)&ad4630_chip_info[ID_AD4632_24] },
	{ "adaq4216", (kernel_ulong_t)&ad4630_chip_info[ID_ADAQ4216] },
	{ "adaq4220", (kernel_ulong_t)&ad4630_chip_info[ID_ADAQ4220] },
	{ "adaq4224", (kernel_ulong_t)&ad4630_chip_info[ID_ADAQ4224] },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4630_id_table);

static const struct of_device_id ad4630_of_match[] = {
	{ .compatible = "adi,ad4030-24", .data = &ad4630_chip_info[ID_AD4030_24] },
	{ .compatible = "adi,ad4032-24", .data = &ad4630_chip_info[ID_AD4032_24] },
	{ .compatible = "adi,ad4630-16", .data = &ad4630_chip_info[ID_AD4630_16] },
	{ .compatible = "adi,ad4632-16", .data = &ad4630_chip_info[ID_AD4632_16] },
	{ .compatible = "adi,ad4630-24", .data = &ad4630_chip_info[ID_AD4630_24] },
	{ .compatible = "adi,ad4632-24", .data = &ad4630_chip_info[ID_AD4632_24] },
	{ .compatible = "adi,adaq4216", .data = &ad4630_chip_info[ID_ADAQ4216] },
	{ .compatible = "adi,adaq4220", .data = &ad4630_chip_info[ID_ADAQ4220] },
	{ .compatible = "adi,adaq4224", .data = &ad4630_chip_info[ID_ADAQ4224] },
	{}
};
MODULE_DEVICE_TABLE(of, ad4630_of_match);

static struct spi_driver ad4630_driver = {
	.driver = {
		.name = "ad4630",
		.of_match_table = ad4630_of_match,
		.pm = pm_ptr(&ad4630_pm_ops),
	},
	.probe = ad4630_probe,
	.id_table = ad4630_id_table,
};
module_spi_driver(ad4630_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_AUTHOR("Liviu Adace <liviu.adace@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4630 and ADAQ4224 ADC family driver");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
