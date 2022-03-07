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
#include <linux/property.h>
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
/* HARDWARE_GAIN */
#define AD4630_REG_CHAN_GAIN(ch, pos)	\
	(AD4630_REG_GAIN_BASE + (2 * (ch)) + (pos))
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

#define AD4630_24_CHANNEL(_name, _idx, _sidx, _storagebits, _realbits, _shift) \
	{								       \
		.type = IIO_VOLTAGE,					       \
		.info_mask_separate = AD4630_24_CH_MASK_SEPARATE(_realbits),   \
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),       \
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

static const char *const ad4630_lane_mdoes[] = {
	[AD4630_SHARED_TWO_CH] = "one-lane-shared",
	[AD4630_ONE_LANE_PER_CH] = "one-lane-per-ch",
	[AD4630_TWO_LANES_PER_CH] = "two-lanes-per-ch",
	[AD4630_FOUR_LANES_PER_CH] = "four-lanes-per-ch",
};

static const char *const ad4630_clock_mdoes[] = {
	[AD4630_SPI_COMPATIBLE_MODE] = "spi-compatible",
	[AD4630_ECHO_CLOCK_MODE] = "echo-clock",
	[AD4630_CLOCK_MASTER_MODE] = "clock-master",
};

static const char *const ad4630_data_rates[] = {
	[AD4630_SINGLE_DATA_RATE] = "single",
	[AD4630_DUAL_DATA_RATE] = "dual"
};

static const char *const ad4630_out_data_modes[] = {
	[AD4630_24_DIFF] = "24diff",
	[AD4630_16_DIFF_8_COM] = "16diff-8com",
	[AD4630_24_DIFF_8_COM] = "24diff-8com",
	[AD4630_30_AVERAGED_DIFF] = "30diff-avg",
	[AD4630_32_PATTERN] = "32pat"
};

static const unsigned int ad4630_data_widths[] = {
	[AD4630_16_DIFF_8_COM] = 24,
	[AD4630_24_DIFF] = 24,
	[AD4630_24_DIFF_8_COM] = 32,
	[AD4630_30_AVERAGED_DIFF] = 32,
	[AD4630_32_PATTERN] = 32
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
	enum ad4630_out_data_mode out_data_mode;
	enum ad4630_clock_mode clock_mode;
	enum ad4630_lane_mode lane_mode;
	int num_avg_samples;
};

struct ad4630_state {
	struct pwm_device *conversion_trigger;
	struct pwm_device *spi_engine_trigger;
	struct ad4630_phy_config phy;
	struct gpio_desc *gpio_reset;
	struct clk *trigger_clock;
	struct spi_device *spi;
	/* Lock for attribute ops */
	struct mutex lock;

	unsigned int sampling_frequency;
	bool buffer_enabled;

	union {
		unsigned char buff[3];
		struct {
			unsigned short reg_addr;
			unsigned char reg_data;
		};
	} adc_data[2] ____cacheline_aligned;
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

	*reg_data = (unsigned int)st->adc_data[1].reg_data;

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
	int ret;

	if (state) {
		/* Send a sequence starting with "1 0 1" to enable reg access */
		ret = ad4630_spi_read_reg(st, AD4630_CONFIG_TIMING, &dummy);
		if (ret)
			return ret;
	} else {
		ret = ad4630_spi_write_reg(st, AD4630_REG_EXIT_CFG_MODE,
					   AD4630_EXIT_CFG_MODE);
		if (ret)
			return ret;
	}

	st->buffer_enabled = !state;

	return 0;
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
	    ad4630_data_widths[st->phy.out_data_mode] == 32)
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
		spi_trigger_state.period *= st->phy.num_avg_samples;

	ret = pwm_apply_state(st->spi_engine_trigger, &spi_trigger_state);
	if (ret)
		return ret;

	st->sampling_frequency = (int)DIV_ROUND_CLOSEST_ULL(clk_rate, target);

	return ret;
}

static int ad4630_set_avg_frame_len(struct iio_dev *dev,
				    const struct iio_chan_spec *chan,
				    unsigned int avg_len)
{
	int ret;
	struct ad4630_state *st = iio_priv(dev);

	if (st->buffer_enabled)
		return -EBUSY;

	if (st->phy.out_data_mode != AD4630_30_AVERAGED_DIFF || avg_len == 0)
		return -EINVAL;

	st->phy.num_avg_samples = 1 << avg_len;
	ret = ad4630_set_sampling_freq(st, st->sampling_frequency);
	if (ret)
		return ret;

	return ad4630_spi_write_reg(st, AD4630_REG_AVG, avg_len);
}

static int ad4630_get_avg_frame_len(struct iio_dev *dev,
				    const struct iio_chan_spec *chan)
{
	struct ad4630_state *st = iio_priv(dev);
	unsigned int avg_len;
	int ret;

	if (st->buffer_enabled)
		return -EBUSY;

	if (st->phy.out_data_mode != AD4630_30_AVERAGED_DIFF)
		return 0;

	ret = ad4630_spi_read_reg(st, AD4630_REG_AVG, &avg_len);
	if (ret)
		return ret;

	return avg_len;
}

static int ad4630_set_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan,
			       unsigned int mode)
{
	struct ad4630_state *st = iio_priv(dev);

	return ad4630_spi_write_reg(st, AD4630_REG_DEVICE_CONFIG, mode);
}

static int ad4630_get_pwr_mode(struct iio_dev *dev,
			       const struct iio_chan_spec *chan)
{
	struct ad4630_state *st = iio_priv(dev);
	unsigned int mode;
	int ret;

	ret = ad4630_spi_read_reg(st, AD4630_REG_DEVICE_CONFIG, &mode);
	if (ret)
		return ret;

	return mode;
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

		st->phy.num_avg_samples = 64;
	}

	return 0;
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

static int ad4630_parse_dt(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	struct fwnode_handle *fwnode;
	const char *data_rate_mode;
	const char *out_data_mode;
	const char *clock_mode;
	const char *lane_mode;
	int ret;

	fwnode = dev_fwnode(indio_dev->dev.parent);
	ret = fwnode_property_read_string(fwnode, "adi,lane-mode",
					  &lane_mode);
	if (ret)
		return ret;

	ret = fwnode_property_read_string(fwnode, "adi,clock-mode",
					  &clock_mode);
	if (ret)
		return ret;

	ret = fwnode_property_read_string(fwnode, "adi,data-rate-mode",
					  &data_rate_mode);
	if (ret)
		return ret;

	ret = fwnode_property_read_string(fwnode, "adi,out-data-mode",
					  &out_data_mode);
	if (ret)
		return ret;

	ret = ad4630_get_string_index(lane_mode, ad4630_lane_mdoes,
				      ARRAY_SIZE(ad4630_lane_mdoes),
				      &st->phy.lane_mode);
	if (ret)
		return ret;

	ret = ad4630_get_string_index(clock_mode, ad4630_clock_mdoes,
				      ARRAY_SIZE(ad4630_clock_mdoes),
				      &st->phy.clock_mode);
	if (ret)
		return ret;

	ret = ad4630_get_string_index(data_rate_mode, ad4630_data_rates,
				      ARRAY_SIZE(ad4630_data_rates),
				      &st->phy.data_rate_mode);
	if (ret)
		return ret;

	return ad4630_get_string_index(out_data_mode, ad4630_out_data_modes,
				       ARRAY_SIZE(ad4630_out_data_modes),
				       &st->phy.out_data_mode);
}

static int ad4630_set_chan_offset(struct ad4630_state *st, int chan_idx,
				  int offset)
{
	int ret;

	if (abs(offset) > 0x7FFFFF)
		return -EINVAL;

	mutex_lock(&st->lock);
	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_OFFSET(chan_idx, 0),
				   offset);
	if (ret)
		goto err_set_chan_offset;

	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_OFFSET(chan_idx, 1),
				   offset >> 8);
	if (ret)
		goto err_set_chan_offset;

	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_OFFSET(chan_idx, 2),
				   offset >> 16);
	if (ret)
		goto err_set_chan_offset;

err_set_chan_offset:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4630_get_chan_offset(struct ad4630_state *st, int chan_idx,
				  int *val)
{
	unsigned int reg;
	int offset;
	int ret;

	mutex_lock(&st->lock);
	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_OFFSET(chan_idx, 0),
				  &reg);
	if (ret)
		goto err_get_chan_offset;

	offset = reg;
	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_OFFSET(chan_idx, 1),
				  &reg);
	if (ret)
		goto err_get_chan_offset;

	offset |= reg << 8;
	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_OFFSET(chan_idx, 2),
				  &reg);
	if (ret)
		goto err_get_chan_offset;

	*val = offset | (reg << 16);
err_get_chan_offset:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4630_set_chan_gain(struct ad4630_state *st, int chan_idx,
				int gain_int, int gain_frac)
{
	unsigned int gain;
	int ret;

	if (gain_int < 0)
		return -EINVAL;

	gain = ((gain_int * 10000) + (gain_frac / 100));
	gain = clamp_t(unsigned int, gain, 0, 19999);
	gain = DIV_ROUND_UP(gain * 0x8000, 10000);
	mutex_lock(&st->lock);
	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_GAIN(chan_idx, 0),
				   gain);
	if (ret)
		goto err_set_chan_gain;

	ret = ad4630_spi_write_reg(st, AD4630_REG_CHAN_GAIN(chan_idx, 1),
				   gain >> 8);
	if (ret)
		goto err_set_chan_gain;

err_set_chan_gain:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4630_get_chan_gain(struct ad4630_state *st, int chan_idx,
				int *val)
{
	unsigned int gain, reg;
	int ret;

	mutex_lock(&st->lock);
	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_GAIN(chan_idx, 0), &gain);
	if (ret)
		goto err_get_chan_gain;

	ret = ad4630_spi_read_reg(st, AD4630_REG_CHAN_GAIN(chan_idx, 1), &reg);
	if (ret)
		goto err_get_chan_gain;

	*val = gain | (reg << 8);
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

	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = ad4630_get_chan_gain(st, chan->scan_index, &temp);
		if (ret)
			return ret;

		temp *= 10000;
		temp /= 0x8000;
		*val = temp / 10000;
		*val2 = (temp - (*val * 10000)) * 100;

		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_OFFSET:
		ret = ad4630_get_chan_offset(st, chan->scan_index, val);
		if (ret)
			return ret;

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ad4630_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val,
			    int val2, long info)
{
	struct ad4630_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4630_set_sampling_freq(st, val);

	case IIO_CHAN_INFO_HARDWAREGAIN:
		return ad4630_set_chan_gain(st, chan->scan_index, val, val2);

	case IIO_CHAN_INFO_OFFSET:
		return ad4630_set_chan_offset(st, chan->scan_index, val);
	}

	return -EINVAL;
}

static int ad4630_setup(struct ad4630_state *st)
{
	int ret;

	gpiod_set_value_cansleep(st->gpio_reset, 0);
	usleep_range(50, 65);
	ret = ad4630_set_reg_access(st, true);
	if (ret)
		return ret;

	ret = ad4630_phy_init(st);
	if (ret)
		return ret;

	ret = ad4630_set_chan_gain(st, 0, 1, 0);
	if (ret)
		return ret;

	ret = ad4630_set_chan_gain(st, 1, 1, 0);
	if (ret)
		return ret;

	ret = ad4630_set_chan_offset(st, 0, 0);
	if (ret)
		return ret;

	ret = ad4630_set_chan_offset(st, 1, 0);
	if (ret)
		return ret;

	return ad4630_set_sampling_freq(st, 1000000);
}

static int ad4630_dma_buffer_submit_block(struct iio_dma_buffer_queue *queue,
					  struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static int ad4630_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad4630_state *st = iio_priv(indio_dev);
	struct spi_message msg;
	unsigned int data[4];
	int ret;

	struct spi_transfer xfer = {
		.len = 1,
		.tx_buf = NULL,
		.rx_buf = data,
		.speed_hz = AD4630_SPI_SAMPLING_SPEED,
	};
	ret = ad4630_set_reg_access(st, false);
	if (ret)
		return ret;

	xfer.bits_per_word =
		AD4630_SPI_WIDTH(st->phy.lane_mode,
				 ad4630_data_widths[st->phy.out_data_mode]);
	if (st->phy.data_rate_mode == AD4630_DUAL_DATA_RATE)
		xfer.bits_per_word /= 2;
	ret = ad4630_set_reg_access(st, false);
	if (ret)
		return ret;

	spi_bus_lock(st->spi->master);
	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);
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
	if (ret)
		return ret;

	return ret;
}

static void ad4630_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static void ad4630_cnv_diasble(void *data)
{
	pwm_disable(data);
}

static void ad4630_avg_trigger_disable(void *data)
{
	pwm_disable(data);
}

static const struct iio_enum ad4630_avg_frame_len_enum = {
	.items = ad4630_average_modes,
	.num_items = ARRAY_SIZE(ad4630_average_modes),
	.set = ad4630_set_avg_frame_len,
	.get = ad4630_get_avg_frame_len,
};

static const struct iio_enum ad4630_power_mode_enum = {
	.items = ad4630_power_modes,
	.num_items = ARRAY_SIZE(ad4630_power_modes),
	.set = ad4630_set_pwr_mode,
	.get = ad4630_get_pwr_mode,
};

static struct iio_chan_spec_ext_info ad4630_ext_info[] = {
	IIO_ENUM("sample_averaging", IIO_SHARED_BY_ALL,
		 &ad4630_avg_frame_len_enum),
	IIO_ENUM_AVAILABLE_SHARED("sample_averaging", IIO_SHARED_BY_ALL,
				  &ad4630_avg_frame_len_enum),
	IIO_ENUM("operating_mode", IIO_SHARED_BY_ALL, &ad4630_power_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("operating_mode", IIO_SHARED_BY_ALL,
				  &ad4630_power_mode_enum),
	{},
};

struct ad464x_chan_info {
	struct iio_chan_spec channels[5][4];
	int channels_number;
};

static const struct ad464x_chan_info ad4630_ch_info[] = {
[ID_AD4030_24] = {
	.channels = {
		[AD4630_16_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential",   0, 0, 64, 16, 8),
			AD4630_24_CHANNEL("common_voltage", 1, 0, 64, 8,  0),
		},
		[AD4630_24_DIFF] = {
			AD4630_24_CHANNEL("differential", 0, 0, 64, 24, 0),
		},
		[AD4630_24_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential",   0, 0, 64, 24, 8),
			AD4630_24_CHANNEL("common_voltage", 1, 0, 64, 8,  0),
		},
		[AD4630_30_AVERAGED_DIFF] = {
			AD4630_24_CHANNEL("differential", 0, 0, 64, 30, 2),
		},
		[AD4630_32_PATTERN] = {
			AD4630_24_CHANNEL("pattern", 0, 0, 64, 32, 0),
		},
	},
	.channels_number = 1
},
[ID_AD4630_16] = {
	.channels = {
		[AD4630_16_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential",   0, 0, 32, 16, 8),
			AD4630_24_CHANNEL("common_voltage", 1, 0, 32, 8,  0),
			AD4630_24_CHANNEL("differential",   2, 1, 32, 16, 8),
			AD4630_24_CHANNEL("common_voltage", 3, 1, 32, 8,  0),
		},
		[AD4630_24_DIFF] = {},
		[AD4630_24_DIFF_8_COM] = {},
		[AD4630_30_AVERAGED_DIFF] = {
			AD4630_24_CHANNEL("differential", 0, 0, 32, 30, 2),
			AD4630_24_CHANNEL("differential", 1, 1, 32, 30, 2),
		},
		[AD4630_32_PATTERN] = {
			AD4630_24_CHANNEL("pattern", 0, 0, 32, 32, 0),
			AD4630_24_CHANNEL("pattern", 1, 1, 32, 32, 0),
		},
	},
	.channels_number = 2
},
[ID_AD4630_24] = {
	.channels = {
		[AD4630_16_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential",   0, 0, 32, 16, 8),
			AD4630_24_CHANNEL("common_voltage", 1, 0, 32, 8,  0),
			AD4630_24_CHANNEL("differential",   2, 1, 32, 16, 8),
			AD4630_24_CHANNEL("common_voltage", 3, 1, 32, 8,  0),
		},
		[AD4630_24_DIFF] = {
			AD4630_24_CHANNEL("differential", 0, 0, 32, 24, 0),
			AD4630_24_CHANNEL("differential", 1, 1, 32, 24, 0),
		},
		[AD4630_24_DIFF_8_COM] = {
			AD4630_24_CHANNEL("differential",   0, 0, 32, 24, 8),
			AD4630_24_CHANNEL("common_voltage", 1, 0, 32, 8,  0),
			AD4630_24_CHANNEL("differential",   2, 1, 32, 24, 8),
			AD4630_24_CHANNEL("common_voltage", 3, 1, 32, 8,  0),
		},
		[AD4630_30_AVERAGED_DIFF] = {
			AD4630_24_CHANNEL("differential", 0, 0, 32, 30, 2),
			AD4630_24_CHANNEL("differential", 1, 1, 32, 30, 2),
		},
		[AD4630_32_PATTERN] = {
			AD4630_24_CHANNEL("pattern", 0, 0, 32, 32, 0),
			AD4630_24_CHANNEL("pattern", 1, 1, 32, 32, 0),
		},
	},
	.channels_number = 2
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

static int ad4630_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	struct ad4630_state *st;
	unsigned int device_id;
	int ret, i;

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

	st->gpio_reset = devm_gpiod_get(&st->spi->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_reset))
		return PTR_ERR(st->gpio_reset);

	device_id = spi_get_device_id(st->spi)->driver_data;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad4630_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad4630_buffer_setup_ops;
	mutex_init(&st->lock);
	ret = ad4630_parse_dt(indio_dev);
	if (ret) {
		dev_err(&spi->dev, "%s invalid devicetree configuration\n",
			indio_dev->name);
		return -EINVAL;
	}
	ret = ad4630_setup(st);
	if (ret) {
		dev_err(&spi->dev, "%s setup failed\n", indio_dev->name);
		return -ENOEXEC;
	}
	if (device_id == ID_AD463X) {
		ret = ad4630_spi_read_reg(st, AD4630_REG_CHIP_GRADE,
					  &device_id);
		if (ret)
			return ret;

		for (i = 0; i < ARRAY_SIZE(ad4630_id_table); i++) {
			if (ad4630_chip_grades[i] == device_id) {
				indio_dev->name = ad4630_id_table[i].name;
				break;
			}
		}
	}
	indio_dev->channels =
		ad4630_ch_info[i].channels[st->phy.out_data_mode];
	/* If the output data mode is not supported by this device */
	if (!indio_dev->channels)
		return -EINVAL;

	indio_dev->num_channels = ad4630_ch_info[i].channels_number;
	if (st->phy.out_data_mode == AD4630_16_DIFF_8_COM ||
	    st->phy.out_data_mode == AD4630_24_DIFF_8_COM)
		indio_dev->num_channels *= 2;
	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent,
						 "rx",
						 &ad4630_dma_buffer_ops,
						 indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static struct spi_driver ad4630_driver = {
	.driver = {
		.name = "ad4630",
		.of_match_table = ad4630_of_match,
	},
	.probe = ad4630_probe,
	.id_table = ad4630_id_table,
};
module_spi_driver(ad4630_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4630 ADC family driver");
MODULE_LICENSE("GPL v2");
