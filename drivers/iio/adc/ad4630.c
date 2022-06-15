// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices AD4630 SPI ADC driver
 *
 * Copyright 2022 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/pwm.h>
#include <linux/sysfs.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>

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
#define AD4630_REG_OFFSET_BASE		0x16
#define AD4630_REG_OFFSET_X0_0		0x16
#define AD4630_REG_OFFSET_X0_1		0x17
#define AD4630_REG_OFFSET_X0_2		0x18
#define AD4630_REG_OFFSET_X1_0		0x19
#define AD4630_REG_OFFSET_X1_1		0x1A
#define AD4630_REG_OFFSET_X1_2		0x1B
#define AD4630_REG_GAIN_BASE		0x1C
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
/* MODES */
#define AD4630_LANE_MODE_MSK		GENMASK(7, 6)
#define AD4630_CLK_MODE_MSK		GENMASK(5, 4)
#define AD4630_DATA_RATE_MODE_MSK	BIT(3)
#define AD4630_OUT_DATA_MODE_MSK	GENMASK(2, 0)
/* EXIT_CFG_MD */
#define AD4630_EXIT_CFG_MODE		BIT(0)
/* AVG */
#define AD4630_AVG_FILTER_RESET		BIT(7)
#define AD4630_AVG_LEN_DEFAULT		0x06
/* OFFSET */
#define AD4630_REG_CHAN_OFFSET(ch, pos)	\
	(AD4630_REG_OFFSET_BASE + (3 * (ch)) + (pos))
#define AD4630_OFFSET(val, pos)	\
	(FIELD_GET(GENMASK(8 * (pos + 1), 8 * pos), val))
#define AD4630_OFFSET_GET(h, m, l) (FIELD_PREP(GENMASK(23, 16), h) |	\
				    FIELD_PREP(GENMASK(15, 8), m) | (l))
/* HARDWARE_GAIN */
#define AD4630_REG_CHAN_GAIN(ch, pos)	\
	(AD4630_REG_GAIN_BASE + (2 * (ch)) + (pos))
#define AD4630_GAIN(val, pos) \
	(FIELD_GET(GENMASK(8 * (pos + 1), 8 * pos), val))
#define AD4630_GAIN_GET(h, l) (FIELD_PREP(GENMASK(15, 8), h) | l)
/* SPI transfer */
#define AD4630_CONFIG_TIMING		0x2000
#define AD4630_REG_READ_DUMMY		0x00
#define AD4630_REG_WRITE_MASK(x)	((x) & 0x7FFF)
#define AD4630_REG_READ_MASK(x)		((x) | BIT(15))
#define AD4630_SPI_REG_ACCESS_SPEED	40000000UL
#define AD4630_SPI_SAMPLING_SPEED	80000000UL
#define AD4630_SPI_WIDTH(mode, width)	((width) >> ((mode) >> 6))
/* Sampling timing */
#define AD4630_HZ_TO_PICOSEC(f) DIV_ROUND_CLOSEST_ULL(1000000000000ULL, f)
#define AD4630_T_CONV_HI_PS		10000
#define AD4630_MAX_RATE_1_LANE		1750000
#define AD4630_MAX_RATE			2000000
#define AD4630_24_CH_MASK_SEPARATE(_realbits)				       \
	(((_realbits) == 8 || (_realbits) == 32) ? 0 :			       \
	(BIT(IIO_CHAN_INFO_HARDWAREGAIN) | BIT(IIO_CHAN_INFO_OFFSET)))

#define AD4630_CHAN(_name, _idx, _sidx, _storagebits, _realbits, _shift)       \
	{								       \
		.type = IIO_VOLTAGE,					       \
		.info_mask_separate = AD4630_24_CH_MASK_SEPARATE(_realbits),   \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) |      \
					   BIT(IIO_CHAN_INFO_SCALE),	       \
		.indexed = 1,						       \
		.extend_name = _name#_sidx,				       \
		.channel = _idx,					       \
		.scan_index = _sidx,					       \
		.ext_info = ad4630_ext_info,				       \
		.scan_type = {						       \
			.sign = 's',					       \
			.storagebits = _storagebits,			       \
			.realbits = _realbits,				       \
			.shift = _shift,				       \
		},							       \
	}

enum ad4630_id {
	ID_AD4030_24,
	ID_AD4630_16,
	ID_AD4630_24,
	ID_AD463X = 0xFF
};

enum ad4630_lane_mode {
	AD4630_ONE_LANE_PER_CH = 0x00,
	AD4630_TWO_LANES_PER_CH = BIT(6),
	AD4630_FOUR_LANES_PER_CH = BIT(7),
	AD4630_SHARED_TWO_CH = (BIT(6) | BIT(7)),
};

enum ad4630_clock_mode {
	AD4630_SPI_COMPATIBLE_MODE = 0x00,
	AD4630_ECHO_CLOCK_MODE = BIT(4),
	AD4630_CLOCK_MASTER_MODE = BIT(5),
};

enum ad4630_data_rate_mode {
	AD4630_SINGLE_DATA_RATE = 0x00,
	AD4630_DUAL_DATA_RATE = BIT(3),
};

enum ad4630_out_data_mode {
	AD4630_16_DIFF = 0x00,
	AD4630_24_DIFF = 0x00,
	AD4630_16_DIFF_8_COM = 0x01,
	AD4630_24_DIFF_8_COM = 0x02,
	AD4630_30_AVERAGED_DIFF = 0x03,
	AD4630_32_PATTERN = 0x04
};

enum ad4630_power_mode {
	AD4630_NORMAL_OPERATING_MODE = 0,
	AD4630_LOW_POWER_MODE = (BIT(0) | BIT(1)),
};

static const unsigned char ad4630_chip_grades[] = {
	[ID_AD4030_24] = 0x81,
	[ID_AD4630_16] = 0x19,
	[ID_AD4630_24] = 0x00,
};

static const char * const ad4630_lane_mdoes[] = {
	[AD4630_SHARED_TWO_CH] = "one-lane-shared",
	[AD4630_ONE_LANE_PER_CH] = "one-lane-per-ch",
	[AD4630_TWO_LANES_PER_CH] = "two-lanes-per-ch",
	[AD4630_FOUR_LANES_PER_CH] = "four-lanes-per-ch",
};

static const char * const ad4630_clock_mdoes[] = {
	[AD4630_SPI_COMPATIBLE_MODE] = "spi-compatible",
	[AD4630_ECHO_CLOCK_MODE] = "echo-clock",
	[AD4630_CLOCK_MASTER_MODE] = "clock-master",
};

static const char * const ad4630_data_rates[] = {
	[AD4630_SINGLE_DATA_RATE] = "single",
	[AD4630_DUAL_DATA_RATE] = "dual"
};

static const char *ad4630_out_data_modes[3][5] = {
	[ID_AD4630_16] = {
		[AD4630_16_DIFF] = "16diff",
		[AD4630_16_DIFF_8_COM] = "16diff-8com",
		[AD4630_30_AVERAGED_DIFF] = "30diff-avg",
		[AD4630_32_PATTERN] = "32pat",
	},
	[ID_AD4630_24] = {
		[AD4630_24_DIFF] = "24diff",
		[AD4630_16_DIFF_8_COM] = "16diff-8com",
		[AD4630_24_DIFF_8_COM] = "24diff-8com",
		[AD4630_30_AVERAGED_DIFF] = "30diff-avg",
		[AD4630_32_PATTERN] = "32pat",
	},
	[ID_AD4030_24] = {
		[AD4630_24_DIFF] = "24diff",
		[AD4630_16_DIFF_8_COM] = "16diff-8com",
		[AD4630_24_DIFF_8_COM] = "24diff-8com",
		[AD4630_30_AVERAGED_DIFF] = "30diff-avg",
		[AD4630_32_PATTERN] = "32pat",
	},
};

static const unsigned int ad4630_data_widths[3][5] = {
	[ID_AD4630_16] = {
		[AD4630_16_DIFF] = 16,
		[AD4630_16_DIFF_8_COM] = 24,
		[AD4630_30_AVERAGED_DIFF] = 32,
		[AD4630_32_PATTERN] = 32
	},
	[ID_AD4630_24] = {
		[AD4630_24_DIFF] = 24,
		[AD4630_16_DIFF_8_COM] = 24,
		[AD4630_24_DIFF_8_COM] = 32,
		[AD4630_30_AVERAGED_DIFF] = 32,
		[AD4630_32_PATTERN] = 32
	},
	[ID_AD4030_24] = {
		[AD4630_24_DIFF] = 24,
		[AD4630_16_DIFF_8_COM] = 24,
		[AD4630_24_DIFF_8_COM] = 32,
		[AD4630_30_AVERAGED_DIFF] = 32,
		[AD4630_32_PATTERN] = 32
	},
};

static const char *const ad4630_power_modes[] = {
	[AD4630_LOW_POWER_MODE] = "low_power_mode",
	[AD4630_NORMAL_OPERATING_MODE] = "normal_operating_mode",
};

static const char *const ad4630_average_modes[] = {
	"OFF", "2",    "4",    "8",    "16",   "32",	"64",	 "128",	  "256",
	"512", "1024", "2048", "4096", "8192", "16384", "32768", "65536",
};

struct ad4630_phy_config {
	enum ad4630_data_rate_mode data_rate_mode;
	enum ad4630_clock_mode clock_mode;
	enum ad4630_lane_mode lane_mode;
	unsigned int out_data_mode;
	unsigned int device_id;
};

struct ad4630_state {
	struct pwm_device *conversion_trigger;
	struct pwm_device *spi_engine_trigger;
	struct ad4630_phy_config phy;
	struct gpio_desc *gpio_reset;
	struct clk *trigger_clock;
	struct spi_device *spi;
	struct regulator *vref;
	/* Lock for attribute ops */
	struct mutex lock;

	unsigned int sampling_frequency;
	int num_avg_samples;
	int resolution;

	union {
		unsigned char buff[3];
		struct {
			unsigned short reg_addr;
			unsigned char reg_data;
		};
	} adc_data[2] __aligned(ARCH_KMALLOC_MINALIGN);
};

static int ad4630_spi_read_reg(struct ad4630_state *st, unsigned int reg_addr,
			       unsigned int *reg_data)
{
	struct spi_transfer xfer = {
		.speed_hz = AD4630_SPI_REG_ACCESS_SPEED,
		.tx_buf = st->adc_data[0].buff,
		.rx_buf = st->adc_data[1].buff,
		.bits_per_word = 8,
		.len = 3,
	};
	int ret;

	st->adc_data[0].reg_addr = cpu_to_be16(AD4630_REG_READ_MASK(reg_addr));
	st->adc_data[0].reg_data = AD4630_REG_READ_DUMMY;

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret)
		return ret;

	*reg_data = st->adc_data[1].reg_data;

	return ret;
}

static int ad4630_spi_write_reg(struct ad4630_state *st, unsigned int reg_addr,
				unsigned int reg_data)
{
	struct spi_transfer xfer = {
		.speed_hz = AD4630_SPI_REG_ACCESS_SPEED,
		.tx_buf = st->adc_data[0].buff,
		.bits_per_word = 8,
		.len = 3,
	};

	st->adc_data[0].reg_addr = cpu_to_be16(AD4630_REG_WRITE_MASK(reg_addr));
	st->adc_data[0].reg_data = reg_data;

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int ad4630_spi_write_reg_masked(struct ad4630_state *st,
				       unsigned int reg_addr,
				       unsigned int mask, unsigned int reg_data)
{
	unsigned int temp;
	int ret;

	ret = ad4630_spi_read_reg(st, reg_addr, &temp);
	if (ret)
		return ret;

	return ad4630_spi_write_reg(st, reg_addr, ((temp & ~mask) | reg_data));
}

static int ad4630_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval)
		ret = ad4630_spi_read_reg(st, reg, readval);
	else
		ret = ad4630_spi_write_reg(st, reg, writeval);

	mutex_unlock(&st->lock);

	return ret;
}

static int ad4630_set_reg_access(struct ad4630_state *st, bool state)
{
	unsigned int dummy;

	if (state)
		/* Send a sequence starting with "1 0 1" to enable reg access */
		return ad4630_spi_read_reg(st, AD4630_CONFIG_TIMING, &dummy);
	else
		return ad4630_spi_write_reg(st, AD4630_REG_EXIT_CFG_MODE,
					    AD4630_EXIT_CFG_MODE);
}

static int ad4630_set_sampling_freq(struct ad4630_state *st, unsigned int freq)
{
	struct pwm_state conversion_state = {
		.duty_cycle = AD4630_T_CONV_HI_PS,
		.time_unit = PWM_UNIT_PSEC,
		.enabled = true,
	}, spi_trigger_state = {
		.duty_cycle = AD4630_T_CONV_HI_PS,
		.phase = AD4630_T_CONV_HI_PS,
		.time_unit = PWM_UNIT_PSEC,
		.enabled = true,
	};
	unsigned long long target, period_ps, max_rate = AD4630_MAX_RATE;
	unsigned long clk_rate;
	int ret;

	if (st->phy.lane_mode == AD4630_ONE_LANE_PER_CH &&
	    st->phy.data_rate_mode == AD4630_SINGLE_DATA_RATE &&
	    ad4630_data_widths[st->phy.device_id][st->phy.out_data_mode] == 32)
		max_rate = AD4630_MAX_RATE_1_LANE;

	freq = clamp_t(unsigned int, freq, 0, max_rate);
	clk_rate = clk_get_rate(st->trigger_clock);
	target = DIV_ROUND_CLOSEST_ULL(clk_rate, freq);
	period_ps = DIV_ROUND_CLOSEST_ULL(1000000000000ULL, clk_rate);
	conversion_state.period = target * period_ps;
	spi_trigger_state.period = target * period_ps;
	ret = pwm_apply_state(st->conversion_trigger, &conversion_state);
	if (ret)
		return ret;

	if (st->phy.out_data_mode == AD4630_30_AVERAGED_DIFF)
		spi_trigger_state.period *= st->num_avg_samples;

	ret = pwm_apply_state(st->spi_engine_trigger, &spi_trigger_state);
	if (ret)
		return ret;

	st->sampling_frequency = DIV_ROUND_CLOSEST_ULL(clk_rate, target);

	return ret;
}

static int ad4630_set_avg_frame_len(struct iio_dev *dev,
				    const struct iio_chan_spec *chan,
				    unsigned int avg_len)
{
	int ret;
	struct ad4630_state *st = iio_priv(dev);

	if (st->phy.out_data_mode != AD4630_30_AVERAGED_DIFF || avg_len == 0)
		return -EINVAL;

	ret = iio_device_claim_direct_mode(dev);
		if (ret)
			return ret;

	st->num_avg_samples = 1 << avg_len;
	ret = ad4630_set_sampling_freq(st, st->sampling_frequency);
	if (ret)
		goto avg_release;

	ret = ad4630_spi_write_reg(st, AD4630_REG_AVG, avg_len);

avg_release:
	iio_device_release_direct_mode(dev);

	return ret;
}

static int ad4630_get_avg_frame_len(struct iio_dev *dev,
				    const struct iio_chan_spec *chan)
{
	struct ad4630_state *st = iio_priv(dev);
	unsigned int avg_len;
	int ret;

	if (st->phy.out_data_mode != AD4630_30_AVERAGED_DIFF)
		return 0;

	ret = iio_device_claim_direct_mode(dev);
	if (ret)
		return ret;

	ret = ad4630_spi_read_reg(st, AD4630_REG_AVG, &avg_len);
	iio_device_release_direct_mode(dev);
	if (ret)
		return ret;

	return avg_len;
}

static int ad4630_phy_init(struct ad4630_state *st)
{
	int ret;

	ret = ad4630_spi_write_reg_masked(st, AD4630_REG_MODES,
					  AD4630_LANE_MODE_MSK,
					  st->phy.lane_mode);
	if (ret)
		return ret;

	ret = ad4630_spi_write_reg_masked(st, AD4630_REG_MODES,
					  AD4630_CLK_MODE_MSK,
					  st->phy.clock_mode);
	if (ret)
		return ret;

	ret = ad4630_spi_write_reg_masked(st, AD4630_REG_MODES,
					  AD4630_DATA_RATE_MODE_MSK,
					  st->phy.data_rate_mode);
	if (ret)
		return ret;

	ret = ad4630_spi_write_reg_masked(st, AD4630_REG_MODES,
					  AD4630_OUT_DATA_MODE_MSK,
					  st->phy.out_data_mode);
	if (ret)
		return ret;

	if (st->phy.out_data_mode == AD4630_30_AVERAGED_DIFF) {
		ret = ad4630_spi_write_reg(st, AD4630_REG_AVG,
					   AD4630_AVG_LEN_DEFAULT);
		if (ret)
			return ret;

		st->num_avg_samples = 64;
	}

	return ad4630_set_sampling_freq(st, 1000000);
}

static int ad4630_get_string_index(const char *string,
				   const char *const *string_array,
				   size_t array_size, unsigned int *idx)
{
	int i;

	for (i = 0; i < array_size; i++) {
		if (string_array[i]) {
			if (!strcmp(string, string_array[i])) {
				*idx = i;
				return 0;
			}
		}
	}

	return -EINVAL;
}

static int ad4630_set_chan_offset(struct ad4630_state *st, int ch_idx,
				  int offset)
{
	int val, ret, idx = 0;

	/* The address for the 16-bit version device is different*/
	if (st->resolution == 16)
		idx = 1;

	val = clamp_t(int, offset, BIT(st->resolution - 1) * -1,
		      BIT(st->resolution - 1) - 1);
	mutex_lock(&st->lock);
	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_OFFSET(ch_idx, idx),
				   AD4630_OFFSET(val, 0));
	if (ret)
		goto err_set_chan_offset;

	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_OFFSET(ch_idx, idx + 1),
				   AD4630_OFFSET(val, 1));
	if (ret || st->resolution <= 16)
		goto err_set_chan_offset;

	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_OFFSET(ch_idx, idx + 2),
				   AD4630_OFFSET(val, 2));
	if (ret)
		goto err_set_chan_offset;

err_set_chan_offset:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4630_get_chan_offset(struct ad4630_state *st, int ch_idx,
				  int *val)
{
	unsigned int high, mid, low;
	int ret, idx = 0;

	/* The address for the 16-bit version device is different*/
	if (st->resolution == 16)
		idx = 1;

	mutex_lock(&st->lock);
	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_OFFSET(ch_idx, idx),
				  &low);
	if (ret)
		goto err_get_chan_offset;

	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_OFFSET(ch_idx, idx + 1),
				  &mid);
	if (ret)
		goto err_get_chan_offset;

	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_OFFSET(ch_idx, idx + 2),
				  &high);
	if (ret)
		goto err_get_chan_offset;

	if (st->resolution > 16)
		*val = AD4630_OFFSET_GET(high, mid, low);
	else
		*val = AD4630_OFFSET_GET(0, mid, low);

err_get_chan_offset:
	mutex_unlock(&st->lock);

	*val = sign_extend32(*val, st->resolution - 1);

	return ret;
}

static int ad4630_set_chan_gain(struct ad4630_state *st, int ch_idx,
				int gain_int, int gain_frac)
{
	unsigned int gain;
	int ret;

	if (gain_int < 0)
		return -EINVAL;

	gain = gain_int * 10000 + gain_frac / 100;
	gain = clamp_t(unsigned int, gain, 0, 19999);
	gain = DIV_ROUND_UP(gain * 0x8000, 10000);
	mutex_lock(&st->lock);
	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_GAIN(ch_idx, 0),
				   AD4630_GAIN(gain, 0));
	if (ret)
		goto err_set_chan_gain;

	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_GAIN(ch_idx, 1),
				   AD4630_GAIN(gain, 1));
	if (ret)
		goto err_set_chan_gain;

err_set_chan_gain:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4630_get_chan_gain(struct ad4630_state *st, int ch_idx,
				int *val)
{
	unsigned int high, low;
	int ret;

	mutex_lock(&st->lock);
	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_GAIN(ch_idx, 0), &low);
	if (ret)
		goto err_get_chan_gain;

	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_GAIN(ch_idx, 1), &high);
	if (ret)
		goto err_get_chan_gain;

	*val = AD4630_GAIN_GET(high, low) * 10000 / 0x8000;

err_get_chan_gain:
	mutex_unlock(&st->lock);

	return ret;
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
		*val = st->sampling_frequency;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		temp = regulator_get_voltage(st->vref);
		if (temp < 0)
			return temp;

		*val = (temp * 2) / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad4630_get_chan_gain(st, chan->scan_index, &temp);
		iio_device_release_direct_mode(indio_dev);
		if (ret)
			return ret;

		*val = temp / 10000;
		*val2 = (temp - (*val * 10000)) * 100;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_OFFSET:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;

		ret = ad4630_get_chan_offset(st, chan->scan_index, val);
		iio_device_release_direct_mode(indio_dev);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4630_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4630_set_sampling_freq(st, val);
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = ad4630_set_chan_gain(st, chan->scan_index, val, val2);
		iio_device_release_direct_mode(indio_dev);

		return ret;
	case IIO_CHAN_INFO_OFFSET:
		ret = iio_device_claim_direct_mode(indio_dev);
		if (ret)
			return ret;
		ret = ad4630_set_chan_offset(st, chan->scan_index, val);
		iio_device_release_direct_mode(indio_dev);

		return ret;
	default:
		return -EINVAL;
	}
}

static int ad4630_dma_buffer_submit_block(struct iio_dma_buffer_queue *queue,
					  struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static int ad4630_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	unsigned int spi_rx_data[4];
	struct spi_transfer xfer = {
		/* If tx_buf is empty, the spi engine won't write on SDO_FIFO */
		.tx_buf = NULL,
		.rx_buf = spi_rx_data,
		.len = 1,
		.speed_hz = AD4630_SPI_SAMPLING_SPEED,
	};
	struct spi_message msg;
	int width;
	int ret;

	pm_runtime_get_sync(&st->spi->dev);
	width = ad4630_data_widths[st->phy.device_id][st->phy.out_data_mode];
	if (!width)
		width = st->resolution;

	xfer.bits_per_word =  AD4630_SPI_WIDTH(st->phy.lane_mode, width);
	if (st->phy.data_rate_mode == AD4630_DUAL_DATA_RATE)
		xfer.bits_per_word /= 2;

	ret = ad4630_set_reg_access(st, false);
	if (ret)
		return ret;

	spi_bus_lock(st->spi->master);
	spi_message_init_with_transfers(&msg, &xfer, 1);
	spi_engine_offload_load_msg(st->spi, &msg);
	spi_engine_offload_enable(st->spi, true);

	return ret;
}

static int ad4630_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	int ret;

	spi_engine_offload_enable(st->spi, false);
	spi_bus_unlock(st->spi->master);

	ret = ad4630_set_reg_access(st, true);
	if (ret < 0)
		return ret;

	pm_runtime_mark_last_busy(&st->spi->dev);
	pm_runtime_put_autosuspend(&st->spi->dev);

	return 0;
}

static void ad4630_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void ad4630_regulator_disable(void *data)
{
	regulator_disable(data);
}

static void ad4630_cnv_diasble(void *data)
{
	pwm_disable(data);
}

static void ad4630_avg_trigger_disable(void *data)
{
	pwm_disable(data);
}

static void ad4630_pm_disable(void *data)
{
	pm_runtime_disable(data);
	pm_runtime_set_suspended(data);
	pm_runtime_put_noidle(data);
}

static const struct iio_enum ad4630_avg_frame_len_enum = {
	.items = ad4630_average_modes,
	.num_items = ARRAY_SIZE(ad4630_average_modes),
	.set = ad4630_set_avg_frame_len,
	.get = ad4630_get_avg_frame_len,
};
static struct iio_chan_spec_ext_info ad4630_ext_info[] = {
	IIO_ENUM("sample_averaging", IIO_SHARED_BY_ALL,
		 &ad4630_avg_frame_len_enum),
	IIO_ENUM_AVAILABLE_SHARED("sample_averaging", IIO_SHARED_BY_ALL,
				  &ad4630_avg_frame_len_enum),
	{},
};

static int __maybe_unused ad4630_runtime_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad4630_state *st = iio_priv(indio_dev);

	return ad4630_spi_write_reg(st, AD4630_REG_DEVICE_CONFIG,
				    AD4630_LOW_POWER_MODE);
}

static int __maybe_unused ad4630_runtime_resume(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad4630_state *st = iio_priv(indio_dev);

	return ad4630_spi_write_reg(st, AD4630_REG_DEVICE_CONFIG,
				    AD4630_NORMAL_OPERATING_MODE);
}

static const struct dev_pm_ops ad4630_pm_ops = {
	SET_RUNTIME_PM_OPS(ad4630_runtime_suspend, ad4630_runtime_resume, NULL)
};

struct ad464x_chan_info {
	struct iio_chan_spec channels[5][4];
	int channels_number;
	int resolution;
};

static const struct ad464x_chan_info ad4630_ch_info[] = {
	[ID_AD4030_24] = {
		.channels = {
			[AD4630_24_DIFF] = {
				AD4630_CHAN("differential", 0, 0, 64, 24, 0),
			},
			[AD4630_16_DIFF_8_COM] = {
				AD4630_CHAN("differential",   0, 0, 64, 16, 8),
				AD4630_CHAN("common_voltage", 1, 0, 64, 8,  0),
			},
			[AD4630_24_DIFF_8_COM] = {
				AD4630_CHAN("differential",   0, 0, 64, 24, 8),
				AD4630_CHAN("common_voltage", 1, 0, 64, 8,  0),
			},
			[AD4630_30_AVERAGED_DIFF] = {
				AD4630_CHAN("differential", 0, 0, 64, 30, 2),
			},
			[AD4630_32_PATTERN] = {
				AD4630_CHAN("pattern", 0, 0, 64, 32, 0),
			},
		},
		.channels_number = 1,
		.resolution = 24,
	},
	[ID_AD4630_16] = {
		.channels = {
			[AD4630_16_DIFF] = {
				AD4630_CHAN("differential",   0, 0, 32, 16, 0),
				AD4630_CHAN("differential",   1, 1, 32, 16, 0),
			},
			[AD4630_16_DIFF_8_COM] = {
				AD4630_CHAN("differential",   0, 0, 32, 16, 8),
				AD4630_CHAN("common_voltage", 1, 0, 32, 8,  0),
				AD4630_CHAN("differential",   2, 1, 32, 16, 8),
				AD4630_CHAN("common_voltage", 3, 1, 32, 8,  0),
			},
			[AD4630_24_DIFF_8_COM] = {},
			[AD4630_30_AVERAGED_DIFF] = {
				AD4630_CHAN("differential", 0, 0, 32, 30, 2),
				AD4630_CHAN("differential", 1, 1, 32, 30, 2),
			},
			[AD4630_32_PATTERN] = {
				AD4630_CHAN("pattern", 0, 0, 32, 32, 0),
				AD4630_CHAN("pattern", 1, 1, 32, 32, 0),
			},
		},
		.channels_number = 2,
		.resolution = 16,
	},
	[ID_AD4630_24] = {
		.channels = {
			[AD4630_24_DIFF] = {
				AD4630_CHAN("differential", 0, 0, 32, 24, 0),
				AD4630_CHAN("differential", 1, 1, 32, 24, 0),
			},
			[AD4630_16_DIFF_8_COM] = {
				AD4630_CHAN("differential",   0, 0, 32, 16, 8),
				AD4630_CHAN("common_voltage", 1, 0, 32, 8,  0),
				AD4630_CHAN("differential",   2, 1, 32, 16, 8),
				AD4630_CHAN("common_voltage", 3, 1, 32, 8,  0),
			},
			[AD4630_24_DIFF_8_COM] = {
				AD4630_CHAN("differential",   0, 0, 32, 24, 8),
				AD4630_CHAN("common_voltage", 1, 0, 32, 8,  0),
				AD4630_CHAN("differential",   2, 1, 32, 24, 8),
				AD4630_CHAN("common_voltage", 3, 1, 32, 8,  0),
			},
			[AD4630_30_AVERAGED_DIFF] = {
				AD4630_CHAN("differential", 0, 0, 32, 30, 2),
				AD4630_CHAN("differential", 1, 1, 32, 30, 2),
			},
			[AD4630_32_PATTERN] = {
				AD4630_CHAN("pattern", 0, 0, 32, 32, 0),
				AD4630_CHAN("pattern", 1, 1, 32, 32, 0),
			},
		},
		.channels_number = 2,
		.resolution = 24,
	},
};

static const struct iio_dma_buffer_ops ad4630_dma_buffer_ops = {
	.submit = ad4630_dma_buffer_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static const struct iio_buffer_setup_ops ad4630_buffer_setup_ops = {
	.preenable = &ad4630_buffer_preenable,
	.postdisable = &ad4630_buffer_postdisable,
};

static const struct spi_device_id ad4630_id_table[] = {
	{ "ad4030-24", ID_AD4030_24 },
	{ "ad4630-16", ID_AD4630_16 },
	{ "ad4630-24", ID_AD4630_24 },
	{ "ad463x",    ID_AD463X },
	{}
};
MODULE_DEVICE_TABLE(spi, ad4630_id_table);

static const struct of_device_id ad4630_of_match[] = {
	{ .compatible = "adi,ad4630-24" },
	{ .compatible = "adi,ad4630-16" },
	{ .compatible = "adi,ad4030-24" },
	{ .compatible = "adi,ad463x" },
	{},
};
MODULE_DEVICE_TABLE(of, ad4630_of_match);

static const struct iio_info ad4630_info = {
	.read_raw = &ad4630_read_raw,
	.write_raw = &ad4630_write_raw,
	.debugfs_reg_access = &ad4630_reg_access,
};

static int ad4630_parse_fw(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	const char *data_rate_mode;
	const char *out_data_mode;
	const char *clock_mode;
	const char *lane_mode;
	int ret, grade, i = 0;

	if (st->phy.device_id == ID_AD463X) {
		ret = ad4630_spi_read_reg(st, AD4630_REG_CHIP_GRADE, &grade);
		if (ret)
			return ret;

		while (ad4630_chip_grades[i] != grade &&
		       i++ < ARRAY_SIZE(ad4630_id_table)) {
		};

		if (ad4630_chip_grades[i] != grade)
			return -EINVAL;

		st->phy.device_id = ad4630_id_table[i].driver_data;
	}

	ret = device_property_read_string(dev, "adi,lane-mode",
					  &lane_mode);
	if (ret)
		return ret;

	ret = ad4630_get_string_index(lane_mode, ad4630_lane_mdoes,
				      ARRAY_SIZE(ad4630_lane_mdoes),
				      &st->phy.lane_mode);
	if (ret)
		return ret;

	ret = device_property_read_string(dev, "adi,clock-mode",
					  &clock_mode);
	if (ret)
		return ret;

	ret = ad4630_get_string_index(clock_mode, ad4630_clock_mdoes,
				      ARRAY_SIZE(ad4630_clock_mdoes),
				      &st->phy.clock_mode);
	if (ret)
		return ret;

	ret = device_property_read_string(dev, "adi,data-rate-mode",
					  &data_rate_mode);
	if (ret)
		return ret;

	ret = ad4630_get_string_index(data_rate_mode, ad4630_data_rates,
				      ARRAY_SIZE(ad4630_data_rates),
				      &st->phy.data_rate_mode);
	if (ret)
		return ret;

	ret = device_property_read_string(dev, "adi,out-data-mode",
					  &out_data_mode);
	if (!ret) {
		ret = ad4630_get_string_index(out_data_mode,
			ad4630_out_data_modes[st->phy.device_id],
			ARRAY_SIZE(ad4630_out_data_modes[st->phy.device_id]),
			&st->phy.out_data_mode);
		if (ret)
			return ret;
	}

	indio_dev->name = ad4630_id_table[i].name;
	indio_dev->channels = ad4630_ch_info[i].channels[st->phy.out_data_mode];
	/* If the output data mode is not supported by this device */
	if (!indio_dev->channels->ext_info)
		return -ENODEV;

	indio_dev->num_channels = ad4630_ch_info[i].channels_number;
	if (st->phy.out_data_mode == AD4630_16_DIFF_8_COM ||
	    st->phy.out_data_mode == AD4630_24_DIFF_8_COM)
		indio_dev->num_channels *= 2;
	st->resolution = ad4630_ch_info[i].resolution;

	return 0;
}

static int ad4630_probe(struct spi_device *spi)
{
	struct gpio_desc *gpio_reset;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad4630_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;
	st->trigger_clock = devm_clk_get(&spi->dev, "trigger_clock");
	if (IS_ERR(st->trigger_clock))
		return PTR_ERR(st->trigger_clock);

	ret = clk_prepare_enable(st->trigger_clock);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad4630_clk_disable,
				       st->trigger_clock);
	if (ret)
		return ret;

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable specified vref supply\n");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad4630_regulator_disable,
				       st->vref);
	if (ret)
		return ret;

	st->conversion_trigger = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(st->conversion_trigger))
		return PTR_ERR(st->conversion_trigger);

	ret = devm_add_action_or_reset(&spi->dev, ad4630_cnv_diasble,
				       st->conversion_trigger);
	if (ret < 0)
		return ret;

	st->spi_engine_trigger = devm_pwm_get(&spi->dev, "spi_trigger");
	if (IS_ERR(st->spi_engine_trigger))
		return PTR_ERR(st->spi_engine_trigger);

	ret = devm_add_action_or_reset(&spi->dev, ad4630_avg_trigger_disable,
				       st->spi_engine_trigger);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad4630_pm_disable, &spi->dev);
	if (ret < 0)
		return ret;

	gpio_reset = devm_gpiod_get_optional(&st->spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		gpiod_set_value_cansleep(gpio_reset, 0);
		usleep_range(50, 65);
	}

	ad4630_set_reg_access(st, true);
	if (!gpio_reset) {
		ret = ad4630_spi_write_reg(st, AD4630_REG_INTERFACE_CONFIG_A,
					   AD4630_SW_RESET);
		if (ret)
			return ret;

		usleep_range(50, 65);
	}

	st->phy.device_id = spi_get_device_id(st->spi)->driver_data;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad4630_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad4630_buffer_setup_ops;
	spi_set_drvdata(spi, indio_dev);
	mutex_init(&st->lock);
	pm_runtime_set_autosuspend_delay(&spi->dev, 1000);
	pm_runtime_use_autosuspend(&spi->dev);
	pm_runtime_set_active(&spi->dev);
	if (ret)
		return ret;

	pm_runtime_enable(&spi->dev);
	pm_runtime_idle(&spi->dev);

	ret = ad4630_parse_fw(indio_dev);
	if (ret < 0)
		return ret;

	ret = ad4630_phy_init(st);
	if (ret)
		return ret;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						 "rx",
						 &ad4630_dma_buffer_ops,
						 indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		return ret;

	pm_runtime_mark_last_busy(&spi->dev);
	pm_runtime_put_autosuspend(&spi->dev);

	return 0;
}

static struct spi_driver ad4630_driver = {
	.driver = {
		.name = "ad4630",
		.of_match_table = ad4630_of_match,
		.pm = &ad4630_pm_ops,
	},
	.probe = ad4630_probe,
	.id_table = ad4630_id_table,
};
module_spi_driver(ad4630_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4630 ADC family driver");
MODULE_LICENSE("GPL v2");
