// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2022 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/device.h>
#include <linux/dmaengine.h>
#include <linux/gpio/driver.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pwm.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/legacy-spi-engine.h>
#include <linux/units.h>
#include <linux/err.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>

#include <linux/iio/sysfs.h>

#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/spi/offload/consumer.h>
#include <linux/spi/offload/types.h>

#define AD4134_DCLK_RISING_OFFSET_NS		8
#define AD4134_MIN_ODR_FREQ_HZ			10
#define AD4134_MAX_ODR_FREQ_HZ			(1496 * HZ_PER_KHZ)

#define AD4134_SPI_MAX_XFER_LEN			3
#define AD4134_NUM_CHANNELS			4
#define AD4134_CHAN_PRECISION_BITS		24

#define AD4134_NAME				"ad4134"

#define AD4134_IF_CONFIG_B_REG				0x01
#define AD4134_IF_CONFIG_B_SINGLE_INSTR			BIT(7)
#define AD4134_IF_CONFIG_B_MASTER_SLAVE_RD_CTRL		BIT(5)
#define AD4134_IF_CONFIG_B_RESET			BIT(1)

#define AD4134_DEVICE_CONFIG_REG		0x02
#define AD4134_DEVICE_CONFIG_POWER_MODE_MASK	BIT(0)
#define AD4134_POWER_MODE_HIGH_PERF		0b1

#define AD4134_DATA_PACKET_CONFIG_REG		0x11
#define AD4134_DATA_PACKET_CONFIG_FRAME_MASK	GENMASK(5, 4)
#define AD4134_DATA_PACKET_16BIT_FRAME		0x0
#define AD4134_DATA_PACKET_16BIT_CRC6_FRAME	0x1
#define AD4134_DATA_PACKET_24BIT_FRAME		0x2
#define AD4134_DATA_PACKET_24BIT_CRC6_FRAME	0x3

#define AD4134_DIG_IF_CFG_REG			0x12
#define AD4134_DIF_IF_CFG_FORMAT_MASK		GENMASK(1, 0)
#define AD4134_DATA_FORMAT_QUAD_CH_PARALLEL	0b10

#define AD4134_CHAN_DIG_FILTER_SEL_REG			0x1E
#define AD4134_CHAN_DIG_FILTER_SEL_MASK			GENMASK(7, 0)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH0	GENMASK(1, 0)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH1	GENMASK(3, 2)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH2	GENMASK(5, 4)
#define AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH3	GENMASK(7, 6)

#define AD4134_GPIO_INPUT(x)			0x00
#define AD4134_GPIO_OUTPUT(x)			BIT(x)
#define AD4134_GPIO_DIR_CONTROL			0x20
#define AD4134_GPIO_DATA			0x21

#define AD4134_SINC6_FILTER			0b01010101

#define AD4134_ODR_MIN				10
#define AD4134_ODR_MAX				1496000
#define AD4134_ODR_DEFAULT			1496000

#define AD4134_RESET_TIME_US			10000000

enum {
	ODR_SET_FREQ,
};

enum ad4134_regulators {
	AD4134_AVDD5_REGULATOR,
	AD4134_AVDD1V8_REGULATOR,
	AD4134_IOVDD_REGULATOR,
	AD4134_REFIN_REGULATOR,
	AD4134_NUM_REGULATORS
};

/* maps adi,adc-frame property value to enum */
static const char * const ad4134_frame_config[] = {
	[AD4134_DATA_PACKET_16BIT_FRAME] = "16-bit",
	[AD4134_DATA_PACKET_16BIT_CRC6_FRAME] = "16-bit+CRC",
	[AD4134_DATA_PACKET_24BIT_FRAME] = "24-bit",
	[AD4134_DATA_PACKET_24BIT_CRC6_FRAME] = "24-bit+CRC",
};

enum ad7134_flt_type {
	WIDEBAND,
	SINC6,
	SINC3,
	SINC3_REJECTION
};

static const char * const ad7134_filter_enum[] = {
	[WIDEBAND] = "WIDEBAND",
	[SINC6] = "SINC6",
	[SINC3] = "SINC3",
	[SINC3_REJECTION] = "SINC3_REJECTION",
};

#define AD4134_CHANNEL(_index, _realbits, _storebits, _ext_info) {		\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ) |		\
				    BIT(IIO_CHAN_INFO_SCALE),			\
	.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.realbits = (_realbits),					\
		.storagebits = 32,						\
		.shift = ((_storebits) - (_realbits))				\
	},									\
	.ext_info = _ext_info,							\
}

static ssize_t ad7134_set_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan,
			       const char *buf, size_t len);
static ssize_t ad7134_get_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf);
static int ad7134_set_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan,
			      unsigned int filter);
static int ad7134_get_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan);
static ssize_t ad7134_ext_info_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len);
static ssize_t ad7134_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf);

static const struct iio_enum ad7134_flt_type_iio_enum = {
	.items = ad7134_filter_enum,
	.num_items = ARRAY_SIZE(ad7134_filter_enum),
	.set = ad7134_set_dig_fil,
	.get = ad7134_get_dig_fil,
};

static struct iio_chan_spec_ext_info ad7134_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SHARED_BY_ALL, &ad7134_flt_type_iio_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SHARED_BY_ALL, &ad7134_flt_type_iio_enum),
	{
	 .name = "odr_set_freq",
	 .read = ad7134_ext_info_read,
	 .write = ad7134_ext_info_write,
	 .shared =  IIO_SHARED_BY_ALL,
	 .private = ODR_SET_FREQ,
	},
	{
	 .name = "ad4134_sync",
	 .write = ad7134_set_sync,
	 .read = ad7134_get_sync,
	 .shared = IIO_SHARED_BY_ALL,
	 },
	{ },
};

#define AD4134_CHAN_SET(_realbits, _storebits) {				\
	AD4134_CHANNEL(0, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(1, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(2, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(3, _realbits, _storebits, ad7134_ext_info),		\
}

#define AD4134_DUO_CHAN_SET(_realbits, _storebits) {				\
	AD4134_CHANNEL(0, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(1, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(2, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(3, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(4, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(5, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(6, _realbits, _storebits, ad7134_ext_info),		\
	AD4134_CHANNEL(7, _realbits, _storebits, ad7134_ext_info),		\
}

static const struct iio_chan_spec ad4134_16_chan_set[] = AD4134_CHAN_SET(16, 16);
static const struct iio_chan_spec ad4134_16CRC_chan_set[] = AD4134_CHAN_SET(16, 24);
static const struct iio_chan_spec ad4134_24_chan_set[] = AD4134_CHAN_SET(24, 24);
static const struct iio_chan_spec ad4134_24CRC_chan_set[] = AD4134_CHAN_SET(24, 32);

static const struct iio_chan_spec ad4134_16_duo_chan_set[] = AD4134_DUO_CHAN_SET(16, 16);
static const struct iio_chan_spec ad4134_16CRC_duo_chan_set[] = AD4134_DUO_CHAN_SET(16, 24);
static const struct iio_chan_spec ad4134_24_duo_chan_set[] = AD4134_DUO_CHAN_SET(24, 24);
static const struct iio_chan_spec ad4134_24CRC_duo_chan_set[] = AD4134_DUO_CHAN_SET(24, 32);

static const unsigned long ad4134_channel_masks[] = {
	GENMASK(ARRAY_SIZE(ad4134_16_chan_set) - 1, 0),
	0,
};

static const unsigned long ad4134_duo_channel_masks[] = {
	GENMASK(ARRAY_SIZE(ad4134_16_duo_chan_set) - 1, 0),
	0,
};

struct ad4134_state {
	struct fwnode_handle		*spi_engine_fwnode;
	struct regmap			*regmap;
	struct spi_device		*spi;
	struct spi_device		*spi_engine;

	unsigned long sys_clk_hz;

	struct spi_transfer xfers[AD4134_NUM_CHANNELS];
	struct spi_message msg;

	struct spi_offload *offload;
	struct spi_offload_trigger *offload_trigger;
	struct spi_offload_trigger_config offload_trigger_config;
	struct pwm_device *odr_trigger;
	struct pwm_waveform odr_wf;
	unsigned int odr_hz;

	struct regulator_bulk_data	regulators[AD4134_NUM_REGULATORS];

	/*
	 * Synchronize access to members the of driver state, and ensure
	 * atomicity of consecutive regmap operations.
	 */
	struct mutex			lock;

	struct spi_message		buf_read_msg;
	struct gpio_desc		*cs_gpio;
	struct gpio_chip		gpiochip;

	unsigned int			odr;
	unsigned int			filter_type;
	unsigned long			sys_clk_rate;
	int				refin_mv;
	int				output_frame;
	u8 num_dout_lines;

};

static ssize_t ad7134_get_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan, char *buf)
{
	return sprintf(buf, "enable\n");
}

static ssize_t ad7134_set_sync(struct iio_dev *indio_dev, uintptr_t private,
			       const struct iio_chan_spec *chan,
			       const char *buf, size_t len)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	gpiod_set_value_cansleep(st->cs_gpio, 1);
	ret = regmap_update_bits(st->regmap, AD4134_IF_CONFIG_B_REG,
				 (AD4134_IF_CONFIG_B_RESET |  AD4134_IF_CONFIG_B_SINGLE_INSTR),
				 (AD4134_IF_CONFIG_B_RESET |  AD4134_IF_CONFIG_B_SINGLE_INSTR));
	if (ret)
		return ret;
	gpiod_set_value_cansleep(st->cs_gpio, 0);

	return ret ? ret : len;
}

static int ad7134_set_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan,
			      unsigned int filter)
{
	struct ad4134_state *st = iio_priv(dev);
	int ret;

	st->filter_type = filter;
	gpiod_set_value_cansleep(st->cs_gpio, 1);

	ret = regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
				 AD4134_CHAN_DIG_FILTER_SEL_MASK,
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH0, filter) |
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH1, filter) |
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH2, filter) |
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH3, filter));

	gpiod_set_value_cansleep(st->cs_gpio, 0);

	if (ret)
		return ret;
	return 0;
}

static int ad7134_get_dig_fil(struct iio_dev *dev,
			      const struct iio_chan_spec *chan)
{
	struct ad4134_state *st = iio_priv(dev);
	int ret;
	unsigned int readval;

	ret = regmap_read(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG, &readval);
	if (ret)
		return ret;

	return FIELD_GET(AD4134_CHAN_DIG_FILTER_SEL_FRAME_MASK_CH0, readval);
}

static ssize_t ad7134_ext_info_read(struct iio_dev *indio_dev,
				    uintptr_t private,
				    const struct iio_chan_spec *chan, char *buf)
{
	int ret = -EINVAL;
	long long val;
	struct ad4134_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);

	switch (private) {
	case ODR_SET_FREQ:
		val = st->odr;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&st->lock);

	return sprintf(buf, "%lld\n", val);
}

static int ad4134_samp_freq_avail[] = { AD4134_ODR_MIN, 1, AD4134_ODR_MAX };

/*
 * Hardcoded 32-bit storagebits because the currently available HDL only
 * supports that.
 */
#define AD4134_OFFLOAD_CHANNEL(_index) {					\
	.type = IIO_VOLTAGE,							\
	.indexed = 1,								\
	.channel = (_index),							\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),				\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),			\
	.scan_index = (_index),							\
	.scan_type = {								\
		.sign = 's',							\
		.storagebits = 32,						\
		.realbits = AD4134_CHAN_PRECISION_BITS,				\
		.endianness = IIO_CPU,						\
	},									\
}

/*
 * It's not possible for software to record when offloaded SPI transfers run so
 * no additional timestamp channel is added.
 */
static const struct iio_chan_spec ad4134_offload_chan_set[] = {
	AD4134_OFFLOAD_CHANNEL(0),
	AD4134_OFFLOAD_CHANNEL(1),
	AD4134_OFFLOAD_CHANNEL(2),
	AD4134_OFFLOAD_CHANNEL(3),
};

/* The chip converts and outputs all 4 channels on each sample request */
static const unsigned long ad4134_offload_scan_masks[] = {
	GENMASK(3, 0),
	0
};

static const struct spi_offload_config ad4134_offload_config = {
	.capability_flags = SPI_OFFLOAD_CAP_TRIGGER |
			    SPI_OFFLOAD_CAP_RX_STREAM_DMA,
};

static int ad4134_update_conversion_rate(struct ad4134_state *st,
					 unsigned int freq_hz)
{
	struct spi_offload_trigger_config *config = &st->offload_trigger_config;
	struct pwm_waveform odr_wf = { };
	u64 offload_period_ns;
	u64 offload_offset_ns;
	u64 odr_high_time_ns;
	unsigned int odr_hz;
	u64 target = 0;
	int ret;

	/*
	 * Every ODR pulse causes each of the 4 ADCs within the AD4134 chip to
	 * take a sample simultaneously. The peripheral then outputs the data
	 * from all those channels over one, two, or four data output lanes. If
	 * the controller can fetch data from multiple lanes, the throughput is
	 * increased proportionally to the number of data lanes in use.
	 * Conversely, when multiple data lanes are enabled, the requested
	 * sampling frequency can be reached with slower ODR frequencies.
	 */
	odr_hz = freq_hz / st->num_dout_lines;
	if (odr_hz < AD4134_MIN_ODR_FREQ_HZ || odr_hz > AD4134_MAX_ODR_FREQ_HZ)
		return -EINVAL;

	odr_wf.period_length_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC, odr_hz);
	/*
	 * For an arbitrary system clock (fSYSCLK), we have a minimum ODR high
	 * time of 6/fSYSCLK derived from the device clock and data interface
	 * timing (with Gated DCLK) specifications. Set the PWM duty cycle to
	 * keep ODR up for at least that long. If the rounded PWM's value is
	 * less than the minimum required, increase the target value by 10 and
	 * attempt to round the waveform again, until the minimum is reached.
	 */
	odr_high_time_ns = div64_ul(6ULL * NANO, st->sys_clk_hz);
	do {
		odr_wf.duty_length_ns = target;
		ret = pwm_round_waveform_might_sleep(st->odr_trigger, &odr_wf);
		if (ret)
			return ret;
		target += 10;
	} while (odr_wf.duty_length_ns < odr_high_time_ns);

	if (!in_range(odr_wf.period_length_ns, 2 * odr_high_time_ns, UINT_MAX))
		return -EINVAL;

	/*
	 * The controller fetches one sample per active lane each time the
	 * offload module is triggered. If multiple data lanes are enabled, the
	 * offload trigger frequency can be proportionally slower.
	 */
	offload_period_ns = DIV_ROUND_CLOSEST(NSEC_PER_SEC,
					      odr_hz * st->num_dout_lines);

	config->periodic.frequency_hz = DIV_ROUND_UP_ULL(NSEC_PER_SEC,
							 offload_period_ns);

	/*
	 * For gated DCLK, the minimum required time between ODR rising edge
	 * and DCLK rising edge is the sum of ODR high time and ODR falling
	 * edge to DCLK rising edge time. Delay the offload trigger for at least
	 * that amount of time so the ADC sample data will be available when the
	 * SPI transfer begin.
	 */
	offload_offset_ns = odr_high_time_ns + AD4134_DCLK_RISING_OFFSET_NS;
	do {
		config->periodic.offset_ns = offload_offset_ns;
		ret = spi_offload_trigger_validate(st->offload_trigger, config);
		if (ret)
			return ret;

		offload_offset_ns += 10;
	} while (config->periodic.offset_ns < odr_high_time_ns +
					      AD4134_DCLK_RISING_OFFSET_NS);

	st->odr_wf = odr_wf;
	st->odr_hz = DIV_ROUND_CLOSEST_ULL(NSEC_PER_SEC, odr_wf.period_length_ns);

	return 0;
}

static ssize_t sampling_frequency_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct ad4134_state *st = iio_priv(dev_to_iio_dev(dev));

	/*
	 * If the controller can fetch data from multiple lanes, the throughput
	 * is increased proportionally to the number of data lanes in use.
	 */
	return sysfs_emit(buf, "%u\n", st->odr_hz * st->num_dout_lines);
}

static ssize_t sampling_frequency_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int val;
	int ret;

	if (!iio_device_claim_direct(indio_dev))
		return -EBUSY;

	ret = kstrtouint(buf, 10, &val);
	if (ret)
		goto out_store;

	ret = ad4134_update_conversion_rate(st, val);

out_store:
	iio_device_release_direct(indio_dev);
	return ret ?: len;
}

static IIO_DEVICE_ATTR_RW(sampling_frequency, 0);

static ssize_t sampling_frequency_available_show(struct device *dev,
						 struct device_attribute *attr,
						 char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad4134_state *st = iio_priv(indio_dev);

	return sysfs_emit(buf, "[%u %u %lu]\n",
			  AD4134_MIN_ODR_FREQ_HZ * st->num_dout_lines, 1,
			  AD4134_MAX_ODR_FREQ_HZ * st->num_dout_lines);
}

static IIO_DEVICE_ATTR_RO(sampling_frequency_available, 0);

static struct attribute *ad4134_offload_attributes[] = {
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL,
};

const struct attribute_group ad4134_offload_attribute_group = {
	.attrs = ad4134_offload_attributes,
};

static void ad4134_prepare_offload_msg(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	unsigned int i;

	for (i = 0; i < AD4134_NUM_CHANNELS; i++) {
		st->xfers[i].bits_per_word = AD4134_CHAN_PRECISION_BITS;
		st->xfers[i].len = roundup_pow_of_two(BITS_TO_BYTES(AD4134_CHAN_PRECISION_BITS));
		st->xfers[i].offload_flags = SPI_OFFLOAD_XFER_RX_STREAM;
	}

	spi_message_init_with_transfers(&st->msg, st->xfers, AD4134_NUM_CHANNELS);
}

static int ad4134_offload_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ad4134_prepare_offload_msg(indio_dev);
	st->msg.offload = st->offload;
	ret = spi_optimize_message(st->spi, &st->msg);
	if (ret)
		return ret;

	ret = pwm_set_waveform_might_sleep(st->odr_trigger, &st->odr_wf, false);
	if (ret)
		goto out_unoptimize;

	ret = spi_offload_trigger_enable(st->offload, st->offload_trigger,
					 &st->offload_trigger_config);
	if (ret)
		goto out_pwm_disable;

	return 0;

out_pwm_disable:
	pwm_disable(st->odr_trigger);
out_unoptimize:
	spi_unoptimize_message(&st->msg);

	return 0;
}

static int ad4134_offload_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	spi_offload_trigger_disable(st->offload, st->offload_trigger);

	pwm_disable(st->odr_trigger);

	spi_unoptimize_message(&st->msg);

	return 0;
}

static const struct iio_buffer_setup_ops ad4134_offload_buffer_setup_ops = {
	.postenable = &ad4134_offload_buffer_postenable,
	.predisable = &ad4134_offload_buffer_predisable,
};

static int ad4134_spi_offload_setup(struct iio_dev *indio_dev,
				    struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct dma_chan *rx_dma;


	st->offload_trigger = devm_spi_offload_trigger_get(dev, st->offload,
							   SPI_OFFLOAD_TRIGGER_PERIODIC);
	if (IS_ERR(st->offload_trigger))
		return dev_err_probe(dev, PTR_ERR(st->offload_trigger),
				     "failed to get offload trigger\n");

	st->offload_trigger_config.type = SPI_OFFLOAD_TRIGGER_PERIODIC;

	rx_dma = devm_spi_offload_rx_stream_request_dma_chan(dev, st->offload);
	if (IS_ERR(rx_dma))
		return dev_err_probe(dev, PTR_ERR(rx_dma),
				     "failed to get offload RX DMA\n");

	return devm_iio_dmaengine_buffer_setup_with_handle(dev, indio_dev, rx_dma,
							   IIO_BUFFER_DIRECTION_IN);
}

static int ad4134_pwm_get(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;

	st->odr_trigger = devm_pwm_get(dev, NULL);
	if (IS_ERR(st->odr_trigger))
		return dev_err_probe(dev, PTR_ERR(st->odr_trigger),
				     "failed to get ODR PWM\n");

	pwm_disable(st->odr_trigger);

	return 0;
}

static int ad4134_offload_buffer_setup(struct iio_dev *indio_dev, struct spi_device *spi)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	struct device *dev = &spi->dev;
	int ret;

	st->offload = devm_spi_offload_get(dev, spi, &ad4134_offload_config);
	ret = PTR_ERR_OR_ZERO(st->offload);
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "failed to get offload\n");


	ret = ad4134_spi_offload_setup(indio_dev, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to setup SPI offload\n");

	ret = ad4134_pwm_get(st);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get PWM: %d\n", ret);

	/*
	 * Start with a slower sampling rate so there is some room for
	 * adjusting the sampling frequency without hitting the maximum
	 * conversion rate.
	 */
	st->odr_hz = AD4134_MAX_ODR_FREQ_HZ >> 4;
	ret = ad4134_update_conversion_rate(st, st->odr_hz);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set offload samp freq\n");

	return 0;
}

static int ad4134_set_odr(struct iio_dev *indio_dev, unsigned int odr)
{
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	if (IS_ERR(st->odr_trigger))
		return 0;

	if (IS_ERR(st->offload_trigger))
		return 0;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);

	ret = ad4134_update_conversion_rate(st, odr);

	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static ssize_t ad7134_ext_info_write(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	int ret = -EINVAL;
	long long readin;
	struct ad4134_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);

	switch (private) {
	case ODR_SET_FREQ:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;
		readin = clamp_t(long long, readin, 0, 1500000);
		ret = ad4134_update_conversion_rate(st, readin);
	break;

	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&st->lock);

	return ret ? ret : len;
}

static int ad4134_input_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_update_bits(st->regmap, AD4134_GPIO_DIR_CONTROL,
				 BIT(offset), AD4134_GPIO_INPUT(offset));

	mutex_unlock(&st->lock);

	return ret;
}

static int ad4134_output_gpio(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);

	ret = regmap_update_bits(st->regmap, AD4134_GPIO_DIR_CONTROL,
				 BIT(offset), AD4134_GPIO_OUTPUT(offset));
	if (ret < 0)
		goto out;

	ret = regmap_update_bits(st->regmap, AD4134_GPIO_DATA, BIT(offset),
				 (value << offset));
out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4134_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, AD4134_GPIO_DIR_CONTROL, &val);
	if (ret < 0)
		goto out;

	ret = regmap_read(st->regmap, AD4134_GPIO_DATA, &val);
	if (ret < 0)
		goto out;

	ret = !!(val & BIT(offset));

out:
	mutex_unlock(&st->lock);

	return ret;
}

static void ad4134_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad4134_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, AD4134_GPIO_DIR_CONTROL, &val);
	if (ret < 0)
		goto out;

	if (val & BIT(offset))
		regmap_update_bits(st->regmap, AD4134_GPIO_DATA, BIT(offset),
				   (value << offset));

out:
	mutex_unlock(&st->lock);
}

static int ad4134_gpio_setup(struct ad4134_state *st)
{
	st->gpiochip.label = "ad4134";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 8;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad4134_input_gpio;
	st->gpiochip.direction_output = ad4134_output_gpio;
	st->gpiochip.get = ad4134_get_gpio;
	st->gpiochip.set = ad4134_set_gpio;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static int ad4134_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*val = st->refin_mv;
		*val2 = chan->scan_type.realbits - 1;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		*val = st->odr;
		mutex_unlock(&st->lock);

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4134_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*vals = ad4134_samp_freq_avail;
		*type = IIO_VAL_INT;

		return IIO_AVAIL_RANGE;
	default:
		return -EINVAL;
	}
}

static int ad4134_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4134_set_odr(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static int ad4134_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4134_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info ad4134_info = {
	.read_raw = ad4134_read_raw,
	.read_avail = ad4134_read_avail,
	.write_raw = ad4134_write_raw,
	.attrs = &ad4134_offload_attribute_group,
	.debugfs_reg_access = ad4134_reg_access,
};

static int ad4134_get_ADC_count(struct ad4134_state *st)
{
	struct device *controller_dev = &st->spi->controller->dev;
	unsigned int adc_count = 0;

	device_for_each_child_node_scoped(controller_dev, child) {
		if (fwnode_property_match_string(child, "compatible",
						 "adi,ad4134") >= 0)
			adc_count++;
		if (fwnode_property_match_string(child, "compatible",
						 "adi,ad7134") >= 0)
			adc_count++;
	}
	return adc_count;
}

static void ad4134_disable_regulators(void *data)
{
	struct ad4134_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}


static int ad4134_setup(struct ad4134_state *st)
{
	struct device *dev = &st->spi->dev;
	struct gpio_desc *reset_gpio;
	struct clk *clk;
	int ret;

	clk = devm_clk_get_enabled(dev, "cnv_ext_clk");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to find SYS clock\n");

	st->sys_clk_rate = clk_get_rate(clk);
	if (!st->sys_clk_rate)
		return dev_err_probe(dev, -EINVAL, "Failed to get SYS clock rate\n");
	st->sys_clk_hz = st->sys_clk_rate;

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = regulator_get_voltage(st->regulators[AD4134_REFIN_REGULATOR].consumer);
	if (ret < 0)
		return ret;

	st->refin_mv = ret / 1000;

	ret = devm_add_action_or_reset(dev, ad4134_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Failed to find reset GPIO\n");

	st->cs_gpio = devm_gpiod_get_optional(dev, "cs", GPIOD_OUT_LOW);
	if (IS_ERR(st->cs_gpio))
		return dev_err_probe(dev, PTR_ERR(st->cs_gpio),
				     "Failed to find cs-gpio\n");

	fsleep(AD4134_RESET_TIME_US);

	gpiod_set_value_cansleep(reset_gpio, 0);


	ret = regmap_update_bits(st->regmap, AD4134_DATA_PACKET_CONFIG_REG,
				 AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
				 FIELD_PREP(AD4134_DATA_PACKET_CONFIG_FRAME_MASK,
					    st->output_frame));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4134_DIG_IF_CFG_REG,
				 AD4134_DIF_IF_CFG_FORMAT_MASK,
				 FIELD_PREP(AD4134_DIF_IF_CFG_FORMAT_MASK,
					    AD4134_DATA_FORMAT_QUAD_CH_PARALLEL));
	if (ret)
		return ret;

	 ret = regmap_update_bits(st->regmap, AD4134_DEVICE_CONFIG_REG,
				  AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
				  FIELD_PREP(AD4134_DEVICE_CONFIG_POWER_MODE_MASK,
					     AD4134_POWER_MODE_HIGH_PERF));
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4134_CHAN_DIG_FILTER_SEL_REG,
				 AD4134_CHAN_DIG_FILTER_SEL_MASK,
				 FIELD_PREP(AD4134_CHAN_DIG_FILTER_SEL_MASK,
					    AD4134_SINC6_FILTER));

	if (ret)
		return ret;

	return 0;
}

static const struct regmap_config ad4134_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static inline int ad4134_spi_engine_compare_fwnode(struct device *dev, void *data)
{
	struct fwnode_handle *fwnode = data;

	return device_match_fwnode(dev, fwnode);
}

static inline void ad4134_spi_engine_release_fwnode(struct device *dev, void *data)
{
	struct fwnode_handle *fwnode = data;

	fwnode_handle_put(fwnode);
}

static int ad4134_bind(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct ad4134_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad4134_offload_buffer_setup(indio_dev, st->spi);
	if (ret)
		return ret;

	ret = component_bind_all(dev, st);
	if (ret)
		return ret;

	return iio_device_register(indio_dev);
}

static void ad4134_unbind(struct device *dev)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);

	iio_device_unregister(indio_dev);

	component_unbind_all(dev, NULL);
}

static const struct component_master_ops ad4134_comp_ops = {
	.bind = ad4134_bind,
	.unbind = ad4134_unbind,
};

static int ad4134_probe(struct spi_device *spi)
{
	struct component_match *match = NULL;
	struct device *dev = &spi->dev;
	struct fwnode_handle *fwnode = dev_fwnode(dev);
	struct iio_dev *indio_dev;
	struct ad4134_state *st;
	bool ad4134_duo;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mutex_init(&st->lock);
	st->spi = spi;

	dev_set_drvdata(dev, indio_dev);

	st->regulators[AD4134_AVDD5_REGULATOR].supply = "avdd5";
	st->regulators[AD4134_AVDD1V8_REGULATOR].supply = "avdd1v8";
	st->regulators[AD4134_IOVDD_REGULATOR].supply = "iovdd";
	st->regulators[AD4134_REFIN_REGULATOR].supply = "refin";

	ad4134_duo = ad4134_get_ADC_count(st) == 2;

	st->output_frame = AD4134_DATA_PACKET_24BIT_FRAME;
	ret = device_property_match_property_string(dev, "adi,adc-frame",
						    ad4134_frame_config,
						    ARRAY_SIZE(ad4134_frame_config));
	if (ret < 0)
		dev_warn(dev, "Failed to get adi,adc-frame property: %d\n", ret);
	else
		st->output_frame = ret;

	switch (st->output_frame) {
	case AD4134_DATA_PACKET_16BIT_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_16_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_16_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	case AD4134_DATA_PACKET_16BIT_CRC6_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_16CRC_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16CRC_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_16CRC_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_16CRC_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	case AD4134_DATA_PACKET_24BIT_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_24_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_24_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	case AD4134_DATA_PACKET_24BIT_CRC6_FRAME:
		if (ad4134_duo) {
			indio_dev->channels = ad4134_24CRC_duo_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24CRC_duo_chan_set);
			indio_dev->available_scan_masks = ad4134_duo_channel_masks;
		} else {
			indio_dev->channels = ad4134_24CRC_chan_set;
			indio_dev->num_channels = ARRAY_SIZE(ad4134_24CRC_chan_set);
			indio_dev->available_scan_masks = ad4134_channel_masks;
		}
		break;
	default:
		return dev_err_probe(dev, -EINVAL,
				     "Failed to config ADC frame\n");
	}

	st->regmap = devm_regmap_init_spi(spi, &ad4134_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	/* The HDL is hardconded/configured to read from all 4 DOUT lines. */
	st->num_dout_lines = 4;

	ret = ad4134_setup(st);
	if (ret)
		return ret;

	if (device_property_present(&st->spi->dev, "gpio-controller")) {
		ret = ad4134_gpio_setup(st);
		if (ret < 0)
			return dev_err_probe(&spi->dev, ret,
					     "Failed to setup GPIOs\n");
	}

	indio_dev->name = spi->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4134_info;

	if (!device_property_present(&st->spi->dev, "adi,spi-engine")) {
		indio_dev->channels = 0;
		indio_dev->num_channels = 0;
		indio_dev->available_scan_masks = 0;
		return devm_iio_device_register(dev, indio_dev);
	}

	indio_dev->setup_ops = &ad4134_offload_buffer_setup_ops;

	st->spi_engine_fwnode = fwnode_find_reference(fwnode, "adi,spi-engine", 0);
	if (IS_ERR(st->spi_engine_fwnode))
		return dev_err_probe(dev, PTR_ERR(st->spi_engine_fwnode),
				     "Failed to find SPI engine node\n");

	component_match_add_release(dev, &match, ad4134_spi_engine_release_fwnode,
				    ad4134_spi_engine_compare_fwnode,
				    st->spi_engine_fwnode);

	return component_master_add_with_match(dev, &ad4134_comp_ops, match);
}

static void ad4134_remove(struct spi_device *spi)
{
	component_master_del(&spi->dev, &ad4134_comp_ops);
}

static const struct spi_device_id ad4134_id[] = {
	{ "ad4134", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_id);

static const struct of_device_id ad4134_of_match[] = {
	{
		.compatible = "adi,ad4134",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_of_match);

static struct spi_driver ad4134_driver = {
	.driver = {
		.name = AD4134_NAME,
		.of_match_table = ad4134_of_match,
	},
	.probe = ad4134_probe,
	.remove = ad4134_remove,
	.id_table = ad4134_id,
};

static int ad4134_spi_engine_bind(struct device *dev, struct device *master,
				  void *data)
{
	struct ad4134_state *st = data;

	st->spi_engine = to_spi_device(dev);

	return 0;
}

static const struct component_ops ad4134_spi_engine_ops = {
	.bind   = ad4134_spi_engine_bind,
};

static int ad4134_spi_engine_probe(struct spi_device *spi)
{
	return component_add(&spi->dev, &ad4134_spi_engine_ops);
}

static void ad4134_spi_engine_remove(struct spi_device *spi)
{
	component_del(&spi->dev, &ad4134_spi_engine_ops);
}

static const struct spi_device_id ad4134_spi_engine_id[] = {
	{ "ad4134-spi-engine", 0 },
	{ },
};
MODULE_DEVICE_TABLE(spi, ad4134_spi_engine_id);

static const struct of_device_id ad4134_spi_engine_of_match[] = {
	{
		.compatible = "adi,ad4134-spi-engine",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4134_spi_engine_of_match);

static struct spi_driver ad4134_spi_engine_driver = {
	.driver = {
		.name = "ad4134-spi-engine",
		.of_match_table = ad4134_spi_engine_of_match,
	},
	.probe = ad4134_spi_engine_probe,
	.remove = ad4134_spi_engine_remove,
	.id_table = ad4134_spi_engine_id,
};

static int __init ad4134_init(void)
{
	int ret;

	ret = spi_register_driver(&ad4134_driver);
	if (ret)
		return ret;

	ret = spi_register_driver(&ad4134_spi_engine_driver);
	if (ret) {
		spi_unregister_driver(&ad4134_driver);
		return ret;
	}

	return 0;
}
module_init(ad4134_init);

static void __exit ad4134_exit(void)
{
	spi_unregister_driver(&ad4134_spi_engine_driver);
	spi_unregister_driver(&ad4134_driver);
}
module_exit(ad4134_exit);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4134 SPI driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
