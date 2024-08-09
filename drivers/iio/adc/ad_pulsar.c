// SPDX-License-Identifier: GPL-2.0+
/*
 * Analog Devices PulSAR ADC family driver
 *
 * Copyright 2022 Analog Devices Inc.
 */

#include <asm/unaligned.h>
#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define AD_PULSAR_REG_CONFIG		0x00

#define AD7682_NUM_TEMP_CHANNELS	1

#define AD7682_REF_INT_2_5V		0x0
#define AD7682_REF_INT_4V		0x1
#define AD7682_REF_EXT_1		0x2
#define AD7682_REF_EXT_2		0x3
#define AD7682_REF_EXT_3		0x6
#define AD7682_REF_EXT_4		0x7
#define AD7682_REG_CONFIG		AD_PULSAR_REG_CONFIG
#define AD7682_CONFIG_RESET		GENMASK(15, 0)
#define AD7682_CFG_MSK			BIT(13)
#define AD7682_POLARITY_MSK		BIT(12)
#define AD7682_PAIR_MSK			BIT(11)
#define AD7682_REF_MSK			BIT(10)
#define AD7682_SEL_MSK			GENMASK(9, 7)
#define AD7682_BW_MSK			BIT(6)
#define AD7682_REFBUF_MSK		GENMASK(5, 3)
#define AD7682_SEQ_MSK			GENMASK(2, 1)
#define AD7682_READBACK_MSK		BIT(0)
#define AD7682_UPDATE_CFG		AD7682_CFG_MSK
#define AD7682_NO_READBACK		AD7682_READBACK_MSK
#define AD7682_CH_POLARITY(x)		FIELD_PREP(AD7682_POLARITY_MSK, x)
#define AD7682_CH_TYPE(x)		FIELD_PREP(AD7682_PAIR_MSK, x)
#define AD7682_CH_REF(x)		FIELD_PREP(AD7682_REF_MSK, x)
#define AD7682_SEQ_SCAN(x)		FIELD_PREP(AD7682_SEQ_MSK, x)
#define AD7682_REFBUF_SEL(x)		FIELD_PREP(AD7682_REFBUF_MSK, x)
#define AD7682_BW_SEL(x)		FIELD_PREP(AD7682_BW_MSK, x)
#define AD7682_SEL_CH(x)		FIELD_PREP(AD7682_SEL_MSK, x)
#define AD7682_GET_BW(x)		FIELD_GET(AD7682_BW_MSK, x)

#define AD7682_CH_TEMP_SENSOR		(AD7682_REFBUF_SEL(AD7682_REF_INT_4V) |\
	AD7682_UPDATE_CFG | AD7682_CH_TYPE(SINGLE_ENDED) | AD7682_CH_REF(GND) |\
	AD7682_CH_POLARITY(BIPOLAR))

#define AD7682_SEQ_EN_CHANNEL(i)	(AD7682_UPDATE_CFG |		       \
	AD7682_CH_POLARITY(UNIPOLAR) | AD7682_CH_TYPE(SINGLE_ENDED) |	       \
	AD7682_CH_REF(COM) | AD7682_SEL_CH(i) | AD7682_BW_SEL(FULL_BW) |       \
	AD7682_REFBUF_SEL(AD7682_REF_INT_4V) | AD7682_SEQ_SCAN(DISABLED) |     \
	AD7682_NO_READBACK)

#define AD7682_DISABLE_SEQ		(AD7682_UPDATE_CFG |		       \
	AD7682_CH_POLARITY(UNIPOLAR) | AD7682_CH_TYPE(SINGLE_ENDED) |	       \
	AD7682_CH_REF(GND) | AD7682_SEL_CH(7) | AD7682_BW_SEL(FULL_BW) |       \
	AD7682_REFBUF_SEL(AD7682_REF_EXT_1) | AD7682_SEQ_SCAN(DISABLED) |      \
	AD7682_NO_READBACK)

enum ad_pulsar_input_type {
	DIFFERENTIAL,
	SINGLE_ENDED,
};

enum ad_pulsar_input_polarity {
	BIPOLAR,
	UNIPOLAR
};

enum ad_pulsar_ch_ref {
	COM,
	GND
};

enum ad_pulsar_sequencer_scan {
	DISABLED,
};

enum ad_pulsar_filter_bw {
	QUARTER_BW,
	FULL_BW
};

static unsigned int ad_pulsar_filter_freq[] = {
	[QUARTER_BW] = 425000,
	[FULL_BW] = 1700000
};

struct ad_pulsar_chip_info {
	enum ad_pulsar_input_type input_type;
	const char *name;
	int num_channels;
	int resolution;
	int sclk_rate;
	int max_rate;
	bool has_filter:1;
	bool cfg_register:1;
};

static const struct ad_pulsar_chip_info ad7988_5_chip_info = {
	.name = "ad7988-5",
	.input_type = SINGLE_ENDED,
	.max_rate = 500000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 80000000
};

static const struct ad_pulsar_chip_info ad7988_1_chip_info = {
	.name = "ad7988-1",
	.input_type = SINGLE_ENDED,
	.max_rate = 100000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 80000000
};

static const struct ad_pulsar_chip_info ad7984_chip_info = {
	.name = "ad7984",
	.input_type = DIFFERENTIAL,
	.max_rate = 1333333,
	.resolution = 18,
	.num_channels = 1,
	.sclk_rate = 80000000
};

static const struct ad_pulsar_chip_info ad7983_chip_info = {
	.name = "ad7983",
	.input_type = SINGLE_ENDED,
	.max_rate = 1333333,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 80000000
};

static const struct ad_pulsar_chip_info ad7982_chip_info = {
	.name = "ad7982",
	.input_type = DIFFERENTIAL,
	.max_rate = 1000000,
	.resolution = 18,
	.num_channels = 1,
	.sclk_rate = 80000000
};

static const struct ad_pulsar_chip_info ad7980_chip_info = {
	.name = "ad7980",
	.input_type = SINGLE_ENDED,
	.max_rate = 1000000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 80000000
};

static const struct ad_pulsar_chip_info ad7949_chip_info = {
	.name = "ad7949",
	.input_type = SINGLE_ENDED,
	.max_rate = 250000,
	.resolution = 14,
	.num_channels = 8 + AD7682_NUM_TEMP_CHANNELS,
	.sclk_rate = 40000000,
	.has_filter = true,
	.cfg_register = true
};

static const struct ad_pulsar_chip_info ad7946_chip_info = {
	.name = "ad7946",
	.input_type = SINGLE_ENDED,
	.max_rate = 500000,
	.resolution = 14,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7942_chip_info = {
	.name = "ad7942",
	.input_type = SINGLE_ENDED,
	.max_rate = 250000,
	.resolution = 14,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7699_chip_info = {
	.name = "ad7699",
	.input_type = SINGLE_ENDED,
	.max_rate = 500000,
	.resolution = 16,
	.num_channels = 8 + AD7682_NUM_TEMP_CHANNELS,
	.sclk_rate = 40000000,
	.has_filter = true,
	.cfg_register = true
};

static const struct ad_pulsar_chip_info ad7694_chip_info = {
	.name = "ad7694",
	.input_type = DIFFERENTIAL,
	.max_rate = 250000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7693_chip_info = {
	.name = "ad7693",
	.input_type = DIFFERENTIAL,
	.max_rate = 500000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7691_chip_info = {
	.name = "ad7691",
	.input_type = DIFFERENTIAL,
	.max_rate = 250000,
	.resolution = 18,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7690_chip_info = {
	.name = "ad7690",
	.input_type = DIFFERENTIAL,
	.max_rate = 400000,
	.resolution = 18,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7689_chip_info = {
	.name = "ad7689",
	.input_type = SINGLE_ENDED,
	.max_rate = 250000,
	.resolution = 16,
	.num_channels = 8 + AD7682_NUM_TEMP_CHANNELS,
	.sclk_rate = 40000000,
	.has_filter = true,
	.cfg_register = true
};

static const struct ad_pulsar_chip_info ad7688_chip_info = {
	.name = "ad7688",
	.input_type = DIFFERENTIAL,
	.max_rate = 500000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7687_chip_info = {
	.name = "ad7687",
	.input_type = DIFFERENTIAL,
	.max_rate = 250000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7686_chip_info = {
	.name = "ad7686",
	.input_type = SINGLE_ENDED,
	.max_rate = 500000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7685_chip_info = {
	.name = "ad7685",
	.input_type = SINGLE_ENDED,
	.max_rate = 250000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7684_chip_info = {
	.name = "ad7684",
	.input_type = DIFFERENTIAL,
	.max_rate = 100000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7683_chip_info = {
	.name = "ad7683",
	.input_type = SINGLE_ENDED,
	.max_rate = 100000,
	.resolution = 16,
	.num_channels = 1,
	.sclk_rate = 40000000
};

static const struct ad_pulsar_chip_info ad7682_chip_info = {
	.name = "ad7682",
	.input_type = SINGLE_ENDED,
	.max_rate = 250000,
	.resolution = 16,
	.num_channels = 4 + AD7682_NUM_TEMP_CHANNELS,
	.sclk_rate = 40000000,
	.has_filter = true,
	.cfg_register = true
};

struct ad_pulsar_adc {
	const struct ad_pulsar_chip_info *info;
	struct iio_chan_spec *channels;
	struct spi_transfer *seq_xfer;
	unsigned int cfg;
	unsigned long ref_clk_rate;
	struct pwm_device *cnv;
	struct spi_device *spi;
	struct regulator *vref;
	unsigned int *cfg_reg;
	int spi_speed_hz;
	int samp_freq;
	int device_id;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	u8 spi_rx_data[4] __aligned(IIO_DMA_MINALIGN);
	u8 spi_tx_data[4];
};

static const struct iio_chan_spec ad_pulsar_chan_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE),
	.scan_type.sign = 'u',
	.scan_type.storagebits = 32,
};

static const struct iio_chan_spec ad_pulsar_chan_template_bw = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),
	.info_mask_shared_by_all_available = BIT(IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY),
	.scan_type.sign = 'u',
	.scan_type.storagebits = 32,
};

static int ad_pulsar_reg_write(struct ad_pulsar_adc *adc, unsigned int reg,
			       unsigned int val)
{
	struct spi_transfer xfer = {
		.bits_per_word = adc->info->resolution,
		.speed_hz = adc->info->sclk_rate,
		.len = 4,
	};

	adc->cfg = val;
	put_unaligned_be16(val << 2, adc->spi_tx_data);
	xfer.tx_buf = adc->spi_tx_data;

	return spi_sync_transfer(adc->spi, &xfer, 1);
}

static int ad_pulsar_read_channel(struct ad_pulsar_adc *adc, unsigned int reg,
				  unsigned int *val)
{
	struct spi_transfer xfer = {
		.bits_per_word = adc->info->resolution,
		.speed_hz = adc->info->sclk_rate,
		.len = 4,
	};
	int ret;

	adc->cfg = reg;
	put_unaligned_be16(reg << 2, adc->spi_tx_data);
	if (adc->info->cfg_register)
		xfer.tx_buf = adc->spi_tx_data;
	xfer.rx_buf = adc->spi_rx_data;

	ret = spi_sync_transfer(adc->spi, &xfer, 1);
	if (ret)
		return ret;

	*val = get_unaligned_le32(adc->spi_rx_data);

	return ret;
}

static int ad_pulsar_reg_access(struct iio_dev *indio_dev, unsigned int reg,
				unsigned int writeval, unsigned int *readval)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);

	if (readval) {
		*readval = adc->cfg;
		return 0;
	}

	return ad_pulsar_reg_write(adc, reg, writeval);
}

static int ad_pulsar_set_samp_freq(struct ad_pulsar_adc *adc, int freq)
{
	unsigned long long ref_clk_period_ns;
	struct pwm_state cnv_state;
	int ret;

	/*
	 * The objective here is to configure the PWM such that we don't have
	 * more than $freq periods per second and duty_cycle and phase should be
	 * their minimal positive value.
	 */
	freq = clamp(freq, 1, adc->info->max_rate);
	ref_clk_period_ns = DIV_ROUND_UP(NSEC_PER_SEC, adc->ref_clk_rate);

	cnv_state = (struct pwm_state){
		.period = DIV_ROUND_UP(NSEC_PER_SEC, freq),
		.duty_cycle = ref_clk_period_ns,
		.phase = ref_clk_period_ns,
		.enabled = true,
	};

	ret = pwm_apply_state(adc->cnv, &cnv_state);
	if (ret)
		return ret;

	adc->samp_freq = freq;

	return ret;
}

static int ad_pulsar_set_lpf(struct ad_pulsar_adc *adc, int index,
			     unsigned int val)
{
	enum ad_pulsar_filter_bw filter;

	if (val == ad_pulsar_filter_freq[QUARTER_BW])
		filter  = QUARTER_BW;
	else if (val == ad_pulsar_filter_freq[FULL_BW])
		filter  = FULL_BW;
	else
		return -EINVAL;

	if (adc->info->has_filter) {
		adc->cfg_reg[index] &= ~AD7682_BW_MSK;
		adc->cfg_reg[index] |= AD7682_BW_SEL(filter);
	}

	return 0;
}

static int ad_pulsar_get_lpf(struct ad_pulsar_adc *adc, int index, int *val)
{
	if (!adc->info->has_filter)
		return -EINVAL;

	*val = ad_pulsar_filter_freq[AD7682_GET_BW(adc->cfg_reg[index])];

	return 0;
}

static int ad_pulsar_read_raw(struct iio_dev *indio_dev,
			      const struct iio_chan_spec *chan,
			      int *val, int *val2, long info)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		/*
		 * Conversion requires 2 acquisitions for some ADCs (AD7682
		 * Datasheet page 31).
		 */
		if (adc->info->cfg_register) {
			ret = ad_pulsar_read_channel(adc, chan->address, val);
			if (ret)
				return ret;

			ret = ad_pulsar_read_channel(adc, chan->address, val);
			if (ret)
				return ret;
		}

		ret = ad_pulsar_read_channel(adc, chan->address, val);
		if (ret)
			return ret;

		if (chan->differential)
			*val = sign_extend32(*val, adc->info->resolution - 1);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = adc->samp_freq;
		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			ret = regulator_get_voltage(adc->vref);
			if (ret < 0)
				return ret;
			*val = ret / 1000;
			*val2 = adc->info->resolution;

			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_TEMP:
			*val = 8933;
			*val2 = 1000;

			return IIO_VAL_FRACTIONAL;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = ad_pulsar_get_lpf(adc, chan->scan_index, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ad_pulsar_write_raw(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       int val, int val2, long info)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad_pulsar_set_samp_freq(adc, val);
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		ret = ad_pulsar_set_lpf(adc, chan->scan_index, val);
		if (ret)
			return ret;

		return ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
					   adc->cfg_reg[chan->scan_index]);

	default:
		return -EINVAL;
	}
}

static int ad_pulsar_read_avail(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				const int **vals, int *type, int *length,
				long info)
{
	switch (info) {
	case IIO_CHAN_INFO_LOW_PASS_FILTER_3DB_FREQUENCY:
		*vals = ad_pulsar_filter_freq;
		*length = ARRAY_SIZE(ad_pulsar_filter_freq);
		*type = IIO_VAL_INT;
		return IIO_AVAIL_LIST;

	default:
		return -EINVAL;
	}
}

static int ad_pulsar_buffer(struct iio_dev *indio_dev,
			    struct spi_message *msg)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	u8 ch, next_active_ch, first, second, last, active_ch[8];
	unsigned int num_en_ch;
	u8 next_in_sequ = 1;
	u8 i = 0;
	int ret;

	num_en_ch = bitmap_weight(indio_dev->active_scan_mask,
				  adc->info->num_channels);

	last = find_last_bit(indio_dev->active_scan_mask,
			     indio_dev->masklength);

	first = find_first_bit(indio_dev->active_scan_mask,
			       indio_dev->masklength);
	if (num_en_ch > 1) {
		second = find_next_bit(indio_dev->active_scan_mask,
				       indio_dev->masklength,
				       first + 1);
	}

	spi_message_init(msg);

	for_each_set_bit(ch, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		active_ch[i] = ch;
		i++;
	}

	/*
	 * Conversion requires 2 acquisitions for some ADCs (AD7682
	 * Datasheet page 31)
	 */
	for (i = 0; i < num_en_ch; i++) {
		ch = active_ch[i];
		/*
		 * Configurations need to be sent prior to the acquisition.
		 */
		if (num_en_ch > 1) {
			if (next_in_sequ >= num_en_ch)
				next_in_sequ = 0;
			next_active_ch = active_ch[next_in_sequ];
			adc->seq_xfer[ch].tx_buf = &adc->cfg_reg[next_active_ch];
			next_in_sequ++;
		}

		adc->seq_xfer[ch].cs_change = 1;
		adc->seq_xfer[ch].word_delay.value = 2;
		adc->seq_xfer[ch].word_delay.unit = SPI_DELAY_UNIT_USECS;

		if (ch == last)
			adc->seq_xfer[ch].cs_change = 0;

		spi_message_add_tail(&adc->seq_xfer[ch], msg);
	}
	/*
	 * The first two configurations need to be sent before populating the
	 * buffer.
	 */
	ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
				  adc->cfg_reg[first]);
	if (ret)
		return ret;

	if (num_en_ch > 1)
		return ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
					  adc->cfg_reg[second]);

	return ad_pulsar_reg_write(adc, AD7682_REG_CONFIG,
				   adc->cfg_reg[first]);
}

static int ad_pulsar_buffer_preenable(struct iio_dev *indio_dev)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	struct spi_message msg;
	int ret;

	ret = ad_pulsar_buffer(indio_dev, &msg);

	if (ret)
		return ret;

	spi_bus_lock(adc->spi->master);
	ret = spi_engine_offload_load_msg(adc->spi, &msg);
	if (ret)
		return ret;

	spi_engine_offload_enable(adc->spi, true);

	return 0;
}

static int ad_pulsar_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	int ret;

	spi_engine_offload_enable(adc->spi, false);
	spi_bus_unlock(adc->spi->master);

	ret = ad_pulsar_reg_write(adc, AD7682_REG_CONFIG, AD7682_DISABLE_SEQ);
	if (ret)
		return ret;

	return ad_pulsar_reg_write(adc, AD7682_REG_CONFIG, AD7682_DISABLE_SEQ);
}

static const struct iio_buffer_setup_ops ad_pulsar_buffer_ops = {
	.preenable = &ad_pulsar_buffer_preenable,
	.postdisable = &ad_pulsar_buffer_postdisable,
};

static const struct iio_info ad_pulsar_iio_info = {
	.read_raw = &ad_pulsar_read_raw,
	.write_raw = &ad_pulsar_write_raw,
	.read_avail = &ad_pulsar_read_avail,
	.debugfs_reg_access = &ad_pulsar_reg_access,
};

static int ad_pulsar_setup_channel(struct ad_pulsar_adc *adc,
				   struct fwnode_handle *child)
{
	unsigned int dummy, in[2] = {};
	u32 chan_index;
	int ret;

	ret = fwnode_property_read_u32(child, "reg", &chan_index);
	if (ret)
		return ret;

	if (fwnode_property_present(child, "diff-channels")) {
		ret = fwnode_property_read_u32_array(child,
						     "diff-channels",
						     in, 2);
		if (ret)
			return ret;

		if (in[0] > 7 || in[1] > 7 || in[0] % 2 != 0 ||
		    (in[0] % 2 == 0 && in[1] != in[0] + 1))
			return -EINVAL;

		adc->cfg_reg[chan_index] = AD7682_SEQ_EN_CHANNEL(in[0]);
		adc->cfg_reg[chan_index] &= ~AD7682_PAIR_MSK;
		adc->cfg_reg[chan_index] |= AD7682_CH_TYPE(DIFFERENTIAL);
		adc->channels[chan_index].differential = 1;
		adc->channels[chan_index].scan_type.sign = 's';
		adc->channels[chan_index].channel2 = in[1];
	} else if (fwnode_property_read_bool(child, "adi,temp-sensor")) {
		adc->cfg_reg[chan_index] = AD7682_CH_TEMP_SENSOR;
		adc->channels[chan_index].type = IIO_TEMP;
		adc->channels[chan_index].indexed = 0;
	} else {
		ret = fwnode_property_read_u32(child,
					       "adi,single-channel",
					       &in[0]);
		if (ret)
			return ret;

		adc->cfg_reg[chan_index] = AD7682_SEQ_EN_CHANNEL(in[0]);
		if (in[0] > adc->info->num_channels)
			return -EINVAL;
	}

	if (fwnode_property_read_bool(child, "bipolar")) {
		adc->channels[chan_index].scan_type.sign = 's';
		adc->cfg_reg[chan_index] &= ~AD7682_POLARITY_MSK;
		adc->cfg_reg[chan_index] |= AD7682_CH_POLARITY(BIPOLAR);
	}

	adc->channels[chan_index].channel = in[0];
	adc->channels[chan_index].scan_index = chan_index;
	adc->channels[chan_index].address = adc->cfg_reg[chan_index];

	if (adc->info->cfg_register)
		adc->seq_xfer[chan_index].tx_buf = &adc->cfg_reg[chan_index];
	adc->seq_xfer[chan_index].rx_buf = &dummy;
	adc->seq_xfer[chan_index].len = 1;
	adc->seq_xfer[chan_index].bits_per_word = adc->info->resolution;
	adc->seq_xfer[chan_index].speed_hz = adc->info->sclk_rate;

	return 0;
}

static int ad_pulsar_parse_channels(struct iio_dev *indio_dev)
{
	struct ad_pulsar_adc *adc = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct fwnode_handle *child;
	int num_ch, ret, i;

	num_ch = device_get_child_node_count(dev);
	if (!num_ch || num_ch > adc->info->num_channels)
		return -EINVAL;

	adc->channels = devm_kcalloc(dev, num_ch, sizeof(struct iio_chan_spec),
				     GFP_KERNEL);
	if (!adc->channels)
		return -ENOMEM;

	for (i = 0; i < num_ch; i++) {
		if (adc->info->has_filter)
			adc->channels[i] = ad_pulsar_chan_template_bw;
		else
			adc->channels[i] = ad_pulsar_chan_template;
		adc->channels[i].channel = i;
		adc->channels[i].scan_index = i;
		adc->channels[i].scan_type.realbits = adc->info->resolution;
	}

	indio_dev->channels = adc->channels;
	indio_dev->num_channels = num_ch;

	adc->cfg_reg = devm_kcalloc(dev, num_ch, sizeof(unsigned short),
				    GFP_KERNEL);
	if (!adc->cfg_reg)
		return -ENOMEM;

	adc->seq_xfer = devm_kcalloc(dev, num_ch, sizeof(struct spi_transfer),
				     GFP_KERNEL);
	if (!adc->seq_xfer)
		return -ENOMEM;

	device_for_each_child_node(dev, child) {
		ret = ad_pulsar_setup_channel(adc, child);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}
	}

	return 0;
}

static void ad_pulsar_reg_disable(void *data)
{
	regulator_disable(data);
}

static void ad_pulsar_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static void ad_pulsar_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ad_pulsar_probe(struct spi_device *spi)
{
	struct ad_pulsar_adc *adc;
	struct iio_dev *indio_dev;
	struct clk *ref_clk;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->spi = spi;

	adc->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(adc->vref))
		return PTR_ERR(adc->vref);

	ret = regulator_enable(adc->vref);
	if (ret) {
		dev_err(&spi->dev, "Failed to enable VREF regulator");
		return ret;
	}

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_reg_disable,
				       adc->vref);
	if (ret)
		return ret;

	ref_clk = devm_clk_get(&spi->dev, "ref_clk");
	if (IS_ERR(ref_clk))
		return PTR_ERR(ref_clk);

	ret = clk_prepare_enable(ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_clk_disable,
				       ref_clk);
	if (ret)
		return ret;

	adc->ref_clk_rate = clk_get_rate(ref_clk);

	adc->cnv = devm_pwm_get(&spi->dev, "cnv");
	if (IS_ERR(adc->cnv))
		return PTR_ERR(adc->cnv);

	ret = devm_add_action_or_reset(&spi->dev, ad_pulsar_pwm_diasble,
				       adc->cnv);
	if (ret)
		return ret;

	adc->info = device_get_match_data(&spi->dev);
	if (!adc->info) {
		adc->info = (struct ad_pulsar_chip_info *)
				spi_get_device_id(spi)->driver_data;
		if (!adc->info)
			return -EINVAL;
	}

	ret = ad_pulsar_parse_channels(indio_dev);
	if (ret)
		return ret;

	indio_dev->name = adc->info->name;
	indio_dev->info = &ad_pulsar_iio_info;
	indio_dev->modes = INDIO_BUFFER_HARDWARE | INDIO_DIRECT_MODE;
	indio_dev->setup_ops = &ad_pulsar_buffer_ops;

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
					      indio_dev, "rx",
					      IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return ret;

	ret = ad_pulsar_set_samp_freq(adc, adc->info->max_rate);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct of_device_id ad_pulsar_of_match[] = {
	{ .compatible = "adi,pulsar,ad7988-5", .data = &ad7988_5_chip_info },
	{ .compatible = "adi,pulsar,ad7988-1", .data = &ad7988_1_chip_info },
	{ .compatible = "adi,pulsar,ad7984", .data = &ad7984_chip_info },
	{ .compatible = "adi,pulsar,ad7983", .data = &ad7983_chip_info },
	{ .compatible = "adi,pulsar,ad7982", .data = &ad7982_chip_info },
	{ .compatible = "adi,pulsar,ad7980", .data = &ad7980_chip_info },
	{ .compatible = "adi,pulsar,ad7949", .data = &ad7949_chip_info },
	{ .compatible = "adi,pulsar,ad7946", .data = &ad7946_chip_info },
	{ .compatible = "adi,pulsar,ad7942", .data = &ad7942_chip_info },
	{ .compatible = "adi,pulsar,ad7699", .data = &ad7699_chip_info },
	{ .compatible = "adi,pulsar,ad7694", .data = &ad7694_chip_info },
	{ .compatible = "adi,pulsar,ad7693", .data = &ad7693_chip_info },
	{ .compatible = "adi,pulsar,ad7691", .data = &ad7691_chip_info },
	{ .compatible = "adi,pulsar,ad7690", .data = &ad7690_chip_info },
	{ .compatible = "adi,pulsar,ad7689", .data = &ad7689_chip_info },
	{ .compatible = "adi,pulsar,ad7688", .data = &ad7688_chip_info },
	{ .compatible = "adi,pulsar,ad7687", .data = &ad7687_chip_info },
	{ .compatible = "adi,pulsar,ad7686", .data = &ad7686_chip_info },
	{ .compatible = "adi,pulsar,ad7685", .data = &ad7685_chip_info },
	{ .compatible = "adi,pulsar,ad7684", .data = &ad7684_chip_info },
	{ .compatible = "adi,pulsar,ad7683", .data = &ad7683_chip_info },
	{ .compatible = "adi,pulsar,ad7682", .data = &ad7682_chip_info },
	{ },
};
MODULE_DEVICE_TABLE(of, ad_pulsar_of_match);

static const struct spi_device_id ad_pulsar_spi_id[] = {
	{ "pulsar,ad7988-5", (kernel_ulong_t)&ad7988_5_chip_info },
	{ "pulsar,ad7988-1", (kernel_ulong_t)&ad7988_1_chip_info },
	{ "pulsar,ad7984", (kernel_ulong_t)&ad7984_chip_info },
	{ "pulsar,ad7983", (kernel_ulong_t)&ad7983_chip_info },
	{ "pulsar,ad7982", (kernel_ulong_t)&ad7982_chip_info },
	{ "pulsar,ad7980", (kernel_ulong_t)&ad7980_chip_info },
	{ "pulsar,ad7949", (kernel_ulong_t)&ad7949_chip_info },
	{ "pulsar,ad7946", (kernel_ulong_t)&ad7946_chip_info },
	{ "pulsar,ad7942", (kernel_ulong_t)&ad7942_chip_info },
	{ "pulsar,ad7699", (kernel_ulong_t)&ad7699_chip_info },
	{ "pulsar,ad7694", (kernel_ulong_t)&ad7694_chip_info },
	{ "pulsar,ad7693", (kernel_ulong_t)&ad7693_chip_info },
	{ "pulsar,ad7691", (kernel_ulong_t)&ad7691_chip_info },
	{ "pulsar,ad7690", (kernel_ulong_t)&ad7690_chip_info },
	{ "pulsar,ad7689", (kernel_ulong_t)&ad7689_chip_info },
	{ "pulsar,ad7688", (kernel_ulong_t)&ad7688_chip_info },
	{ "pulsar,ad7687", (kernel_ulong_t)&ad7687_chip_info },
	{ "pulsar,ad7686", (kernel_ulong_t)&ad7686_chip_info },
	{ "pulsar,ad7685", (kernel_ulong_t)&ad7685_chip_info },
	{ "pulsar,ad7684", (kernel_ulong_t)&ad7684_chip_info },
	{ "pulsar,ad7683", (kernel_ulong_t)&ad7683_chip_info },
	{ "pulsar,ad7682", (kernel_ulong_t)&ad7682_chip_info },
	{ }

};
MODULE_DEVICE_TABLE(spi, ad_pulsar_spi_id);

static struct spi_driver ad_pulsar_driver = {
	.driver = {
		.name = "pulsar_adc",
		.of_match_table = ad_pulsar_of_match,
	},
	.probe = ad_pulsar_probe,
	.id_table = ad_pulsar_spi_id,
};
module_spi_driver(ad_pulsar_driver);

MODULE_AUTHOR("Sergiu Cuciurean <sergiu.cuciurean@analog.com>");
MODULE_DESCRIPTION("Analog Devices PulSAR ADC family driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
