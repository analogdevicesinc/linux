// SPDX-License-Identifier: GPL-2.0+
/*
 * AD7768 Analog to digital converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/gpio/driver.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "cf_axi_adc.h"

/* AD7768 registers definition */
#define AD7768_CH_MODE				0x01
#define AD7768_POWER_MODE			0x04
#define AD7768_DATA_CONTROL			0x06
#define AD7768_INTERFACE_CFG			0x07

#define AD7768_REG_GPIO_CONTROL			0x0E

/* AD7768_REG_GPIO_CONTROL */
#define AD7768_GPIO_UGPIO_ENABLE		BIT(7)

#define AD7768_GPIO_INPUT(x)			0x00
#define AD7768_GPIO_OUTPUT(x)			BIT(x)

#define AD7768_REG_GPIO_WRITE			0x0F
#define AD7768_REG_GPIO_READ			0x10

/* AD7768_CH_MODE */
#define AD7768_CH_MODE_FILTER_TYPE_MSK		BIT(3)
#define AD7768_CH_MODE_FILTER_TYPE_MODE(x)	(((x) & 0x1) << 3)
#define AD7768_CH_MODE_GET_FILTER_TYPE(x)	(((x) >> 3) & 0x1)
#define AD7768_CH_MODE_DEC_RATE_MSK		GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE_MODE(x)		(((x) & 0x7) << 0)

/* AD7768_POWER_MODE */
#define AD7768_POWER_MODE_POWER_MODE_MSK	GENMASK(5, 4)
#define AD7768_POWER_MODE_POWER_MODE(x)		(((x) & 0x3) << 4)
#define AD7768_POWER_MODE_GET_POWER_MODE(x)	(((x) >> 4) & 0x3)
#define AD7768_POWER_MODE_MCLK_DIV_MSK		GENMASK(1, 0)
#define AD7768_POWER_MODE_MCLK_DIV_MODE(x)	(((x) & 0x3) << 0)
#define ad7768_map_power_mode_to_regval(x)	((x) ? ((x) + 1) : 0)
#define ad7768_map_regval_to_power_mode(x)	((x) ? ((x) - 1) : 0)

/* AD7768_DATA_CONTROL */
#define AD7768_DATA_CONTROL_SPI_RESET_MSK	GENMASK(1, 0)
#define AD7768_DATA_CONTROL_SPI_RESET_1		0x03
#define AD7768_DATA_CONTROL_SPI_RESET_2		0x02
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK	BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC		BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR	0

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK	GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)	(4 - ffs(x))
#define AD7768_MAX_DCLK_DIV			8

#define AD7768_INTERFACE_CFG_CRC_SELECT_MSK	GENMASK(3, 2)
/* only 4 samples CRC calculation support exists */
#define AD7768_INTERFACE_CFG_CRC_SELECT		0x01

#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

#define AD7768_OUTPUT_MODE_TWOS_COMPLEMENT	0x01

#define SAMPLE_SIZE				32
#define MAX_FREQ_PER_MODE			6


enum ad7768_power_modes {
	AD7768_LOW_POWER_MODE,
	AD7768_MEDIAN_MODE,
	AD7768_FAST_MODE,
	AD7768_NUM_POWER_MODES
};

struct ad7768_freq_config {
	unsigned int freq;
	unsigned int dec_rate;
};

struct ad7768_avail_freq {
	unsigned int n_freqs;
	struct ad7768_freq_config freq_cfg[MAX_FREQ_PER_MODE];
};

struct ad7768_state {
	struct spi_device *spi;
	struct mutex lock;
	struct regulator *vref;
	struct clk *mclk;
	struct gpio_chip gpiochip;
	unsigned int datalines;
	unsigned int sampling_freq;
	enum ad7768_power_modes power_mode;
	const struct axiadc_chip_info *chip_info;
	struct ad7768_avail_freq avail_freq[AD7768_NUM_POWER_MODES];
	__be16 d16;
};

enum ad7768_device_ids {
	ID_AD7768,
	ID_AD7768_4
};

static const int ad7768_dec_rate[6] = {
	32, 64, 128, 256, 512, 1024
};

static const int ad7768_mclk_div[3] = {
	32, 8, 4
};

static const unsigned int ad7768_available_datalines[] = {
	1, 2, 8
};

static const unsigned int ad7768_4_available_datalines[] = {
	1, 4
};

static bool ad7768_has_axi_adc(struct device *dev)
{
	return device_property_present(dev, "spibus-connected");
}

static struct ad7768_state *ad7768_get_data(struct iio_dev *indio_dev)
{
	struct axiadc_converter *conv;

	if (ad7768_has_axi_adc(&indio_dev->dev)) {
		/* AXI ADC*/
		conv = iio_device_get_drvdata(indio_dev);
		return conv->phy;
	} else {
		return iio_priv(indio_dev);
	}
}

static int ad7768_spi_reg_read(struct ad7768_state *st, unsigned int addr,
			       unsigned int *val)
{
	struct spi_transfer t[] = {
		{
			.tx_buf = &st->d16,
			.len = 2,
			.cs_change = 1,
		}, {
			.rx_buf = &st->d16,
			.len = 2,
		},
	};
	int ret;

	st->d16 = cpu_to_be16((AD7768_WR_FLAG_MSK(addr) << 8));

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret < 0)
		return ret;

	*val = be16_to_cpu(st->d16);

	return ret;
}

static int ad7768_spi_reg_write(struct ad7768_state *st,
				unsigned int addr,
				unsigned int val)
{
	st->d16 = cpu_to_be16(((addr & 0x7F) << 8) | val);

	return spi_write(st->spi, &st->d16, sizeof(st->d16));
}

static int ad7768_spi_write_mask(struct ad7768_state *st,
				 unsigned int addr,
				 unsigned long int mask,
				 unsigned int val)
{
	unsigned int regval;
	int ret;

	ret = ad7768_spi_reg_read(st, addr, &regval);
	if (ret < 0)
		return ret;

	regval &= ~mask;
	regval |= val;

	return ad7768_spi_reg_write(st, addr, regval);
}

static int ad7768_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad7768_state *st = ad7768_get_data(indio_dev);
	int ret;

	mutex_lock(&st->lock);
	if (readval) {
		ret = ad7768_spi_reg_read(st, reg, readval);
		if (ret < 0)
			goto exit;
		ret = 0;
	} else {
		ret = ad7768_spi_reg_write(st, reg, writeval);
	}
exit:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_sync(struct ad7768_state *st)
{
	int ret;

	ret = ad7768_spi_write_mask(st, AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC_CLEAR);
	if (ret < 0)
		return ret;

	return ad7768_spi_write_mask(st,  AD7768_DATA_CONTROL,
				    AD7768_DATA_CONTROL_SPI_SYNC_MSK,
				    AD7768_DATA_CONTROL_SPI_SYNC);
}

static int ad7768_set_clk_divs(struct ad7768_state *st,
			       unsigned int freq)
{
	unsigned int mclk, dclk, dclk_div, i;
	struct ad7768_freq_config f_cfg;
	unsigned int chan_per_doutx;
	int ret = 0;

	mclk = clk_get_rate(st->mclk);

	chan_per_doutx = st->chip_info->num_channels / st->datalines;

	for (i = 0; i < st->avail_freq[st->power_mode].n_freqs; i++) {
		f_cfg = st->avail_freq[st->power_mode].freq_cfg[i];
		if (freq == f_cfg.freq)
			break;
	}
	if (i == st->avail_freq[st->power_mode].n_freqs)
		return -EINVAL;

	dclk = f_cfg.freq * SAMPLE_SIZE * chan_per_doutx;
	if (dclk > mclk)
		return -EINVAL;

	/* Set dclk_div to the nearest power of 2 less than the original value */
	dclk_div = DIV_ROUND_CLOSEST_ULL(mclk, dclk);
	if (dclk_div > AD7768_MAX_DCLK_DIV)
		dclk_div = AD7768_MAX_DCLK_DIV;
	else if (hweight32(dclk_div) != 1)
		dclk_div = 1 << (fls(dclk_div) - 1);

	ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
			AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
			AD7768_INTERFACE_CFG_DCLK_DIV_MODE(dclk_div));
	if (ret < 0)
		return ret;

	return ad7768_spi_write_mask(st, AD7768_CH_MODE,
				     AD7768_CH_MODE_DEC_RATE_MSK,
				     AD7768_CH_MODE_DEC_RATE_MODE(f_cfg.dec_rate));
}

static int ad7768_set_sampling_freq(struct iio_dev *dev,
				    unsigned int freq)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	int ret = 0;

	if (!freq)
		return -EINVAL;

	mutex_lock(&st->lock);

	ret = ad7768_set_clk_divs(st, freq);
	if (ret < 0)
		goto freq_err;

	st->sampling_freq = freq;

freq_err:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_set_power_mode(struct iio_dev *dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	struct ad7768_avail_freq avail_freq;
	int max_mode_freq;
	unsigned int regval;
	int ret;

	st->power_mode = mode;

	regval = ad7768_map_power_mode_to_regval(mode);
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_POWER_MODE_MSK,
				    AD7768_POWER_MODE_POWER_MODE(regval));
	if (ret < 0)
		return ret;

	/* The values for the powermode correspond for mclk div */
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_MCLK_DIV_MSK,
				    AD7768_POWER_MODE_MCLK_DIV_MODE(regval));
	if (ret < 0)
		return ret;

	/* Set the max freq of the selected power mode */
	avail_freq = st->avail_freq[mode];
	max_mode_freq = avail_freq.freq_cfg[avail_freq.n_freqs - 1].freq;
	ret = ad7768_set_sampling_freq(dev, max_mode_freq);
	if (ret < 0)
		return ret;

	ret = ad7768_sync(st);
	if (ret < 0)
		return ret;

	return ret;
}

static int ad7768_get_power_mode(struct iio_dev *dev,
				 const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int regval, power_mode;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_POWER_MODE, &regval);
	if (ret < 0)
		return ret;

	power_mode = AD7768_POWER_MODE_GET_POWER_MODE(regval);
	st->power_mode = ad7768_map_regval_to_power_mode(power_mode);

	return st->power_mode;
}

static const char * const ad7768_power_mode_iio_enum[] = {
	[AD7768_LOW_POWER_MODE] = "LOW_POWER_MODE",
	[AD7768_MEDIAN_MODE] = "MEDIAN_MODE",
	[AD7768_FAST_MODE] = "FAST_MODE"
};

static const struct iio_enum ad7768_power_mode_enum = {
	.items = ad7768_power_mode_iio_enum,
	.num_items = ARRAY_SIZE(ad7768_power_mode_iio_enum),
	.set = ad7768_set_power_mode,
	.get = ad7768_get_power_mode,
};
static const char * const ad7768_filter_type_enum[] = {
	"WIDEBAND",
	"SINC5"
};

static int ad7768_set_filter_type(struct iio_dev *dev,
				  const struct iio_chan_spec *chan,
				  unsigned int filter)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	int ret;

	ret = ad7768_spi_write_mask(st, AD7768_CH_MODE,
				    AD7768_CH_MODE_FILTER_TYPE_MSK,
				    AD7768_CH_MODE_FILTER_TYPE_MODE(filter));
	if (ret < 0)
		return ret;

	return ad7768_sync(st);
}

static int ad7768_get_filter_type(struct iio_dev *dev,
				  const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int filter;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_CH_MODE, &filter);
	if (ret < 0)
		return ret;

	return AD7768_CH_MODE_GET_FILTER_TYPE(filter);
}

static const struct iio_enum ad7768_filter_type_iio_enum = {
	.items = ad7768_filter_type_enum,
	.num_items = ARRAY_SIZE(ad7768_filter_type_enum),
	.set = ad7768_set_filter_type,
	.get = ad7768_get_filter_type
};

static void ad7768_set_available_sampl_freq(struct ad7768_state *st)
{
	unsigned int mode;
	unsigned int dec;
	unsigned int mclk = clk_get_rate(st->mclk);
	struct ad7768_avail_freq *avail_freq;

	for (mode = 0; mode < AD7768_NUM_POWER_MODES; mode++) {
		avail_freq = &st->avail_freq[mode];
		for (dec = ARRAY_SIZE(ad7768_dec_rate); dec > 0; dec--) {
			struct ad7768_freq_config freq_cfg;

			freq_cfg.dec_rate = dec - 1;
			freq_cfg.freq = mclk / (ad7768_dec_rate[dec - 1] *
					ad7768_mclk_div[mode]);

			avail_freq->freq_cfg[avail_freq->n_freqs++] = freq_cfg;
		}
	}

	/* The max frequency is not supported in one data line configuration */
	if (st->datalines == 1)
		st->avail_freq[AD7768_FAST_MODE].n_freqs--;
}

static ssize_t ad7768_attr_sampl_freq_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7768_state *st;
	int i, len = 0, size;

	st = ad7768_get_data(indio_dev);
	size = st->avail_freq[st->power_mode].n_freqs;

	for (i = 0; i < size; i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			st->avail_freq[st->power_mode].freq_cfg[i].freq);
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEV_ATTR_SAMP_FREQ_AVAIL(ad7768_attr_sampl_freq_avail);

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7768_state *st = ad7768_get_data(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		*val = 2 * (ret / 1000);
		*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad7768_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7768_set_sampling_freq(indio_dev, val);
	default:
		return -EINVAL;
	}
}

static struct iio_chan_spec_ext_info ad7768_ext_info[] = {
	IIO_ENUM("power_mode",
		 IIO_SHARED_BY_ALL,
		 &ad7768_power_mode_enum),
	IIO_ENUM_AVAILABLE_SHARED("power_mode",
				  IIO_SHARED_BY_ALL,
				  &ad7768_power_mode_enum),
	IIO_ENUM("filter_type",
		 IIO_SHARED_BY_ALL,
		 &ad7768_filter_type_iio_enum),
	IIO_ENUM_AVAILABLE_SHARED("filter_type",
				  IIO_SHARED_BY_ALL,
				  &ad7768_filter_type_iio_enum),
	{ },

};

static struct attribute *ad7768_attributes[] = {
	&iio_dev_attr_sampling_frequency_available.dev_attr.attr,
	NULL
};

static const struct attribute_group ad7768_group = {
	.attrs = ad7768_attributes,
};

static const struct iio_info ad7768_info = {
	.attrs = &ad7768_group,
	.read_raw = &ad7768_read_raw,
	.write_raw = &ad7768_write_raw,
	.debugfs_reg_access = &ad7768_reg_access,
};

#define AD7768_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.address = index,					\
		.indexed = 1,						\
		.channel = index,					\
		.scan_index = index,					\
		.ext_info = ad7768_ext_info,				\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = 24,					\
			.storagebits = 32,				\
		},							\
	}

static const struct axiadc_chip_info ad7768_conv_chip_info = {
	.id = ID_AD7768,
	.name = "ad7768_axi_adc",
	.num_channels = 8,
	.channel[0] = AD7768_CHAN(0),
	.channel[1] = AD7768_CHAN(1),
	.channel[2] = AD7768_CHAN(2),
	.channel[3] = AD7768_CHAN(3),
	.channel[4] = AD7768_CHAN(4),
	.channel[5] = AD7768_CHAN(5),
	.channel[6] = AD7768_CHAN(6),
	.channel[7] = AD7768_CHAN(7),
};

static const struct axiadc_chip_info ad7768_4_conv_chip_info = {
	.id = ID_AD7768_4,
	.name = "ad7768_4_axi_adc",
	.num_channels = 4,
	.channel[0] = AD7768_CHAN(0),
	.channel[1] = AD7768_CHAN(1),
	.channel[2] = AD7768_CHAN(2),
	.channel[3] = AD7768_CHAN(3),
};

static const unsigned long ad7768_available_scan_masks[]  = { 0xFF, 0x00 };

static int hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	block->block.bytes_used = block->block.size;

	return iio_dmaengine_buffer_submit_block(queue, block, DMA_DEV_TO_MEM);
}

static const struct iio_dma_buffer_ops dma_buffer_ops = {
	.submit = hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static void ad7768_reg_disable(void *data)
{
	struct regulator *reg = data;

	regulator_disable(reg);
}

static void ad7768_clk_disable(void *data)
{
	struct clk *clk = data;

	clk_disable_unprepare(clk);
}

static int ad7768_datalines_from_dt(struct ad7768_state *st)
{
	const unsigned int *available_datalines;
	unsigned int i, len;
	int ret;

	st->datalines = 1;
	ret = device_property_read_u32(&st->spi->dev, "adi,data-lines",
				       &st->datalines);
	if (ret < 0)
		return (ret != -EINVAL) ? ret : 0;

	switch (st->chip_info->id) {
	case ID_AD7768:
		available_datalines = ad7768_available_datalines;
		len = ARRAY_SIZE(ad7768_available_datalines);
		break;
	case ID_AD7768_4:
		available_datalines = ad7768_4_available_datalines;
		len = ARRAY_SIZE(ad7768_4_available_datalines);
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < len; i++) {
		if (available_datalines[i] == st->datalines)
			return 0;
	}

	return -EINVAL;
}

static int ad7768_post_setup(struct iio_dev *indio_dev)
{
	struct axiadc_state *axiadc_st = iio_priv(indio_dev);
	struct axiadc_converter *conv = iio_device_get_drvdata(indio_dev);
	struct ad7768_state *st = conv->phy;

	axiadc_write(axiadc_st, ADI_REG_CNTRL_3, ADI_CRC_EN);
	axiadc_write(axiadc_st, ADI_REG_CNTRL, ADI_NUM_LANES(st->datalines));

	return 0;
}

static int ad7768_register_axi_adc(struct ad7768_state *st)
{
	struct axiadc_converter	*conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->clk = st->mclk;
	conv->chip_info = st->chip_info;
	conv->adc_output_mode = AD7768_OUTPUT_MODE_TWOS_COMPLEMENT;
	conv->reg_access = &ad7768_reg_access;
	conv->write_raw = &ad7768_write_raw;
	conv->read_raw = &ad7768_read_raw;
	conv->post_setup = &ad7768_post_setup;
	conv->attrs = &ad7768_group;
	conv->phy = st;
	/* Without this, the axi_adc won't find the converter data */
	spi_set_drvdata(st->spi, conv);

	return 0;
}

static int ad7768_register(struct ad7768_state *st, struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;

	indio_dev->dev.parent = &st->spi->dev;
	indio_dev->name = "ad7768";
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->num_channels = st->chip_info->num_channels;
	indio_dev->info = &ad7768_info;
	indio_dev->available_scan_masks = ad7768_available_scan_masks;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						&dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static int ad7768_input_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_write_mask(st,
				    AD7768_REG_GPIO_CONTROL,
				    BIT(offset),
				    AD7768_GPIO_INPUT(offset));
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_output_gpio(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_write_mask(st,
				    AD7768_REG_GPIO_CONTROL,
				    BIT(offset),
				    AD7768_GPIO_OUTPUT(offset));
	if (ret < 0)
		goto out;

	ret = ad7768_spi_write_mask(st,
				    AD7768_REG_GPIO_WRITE,
				    BIT(offset),
				    (value << offset));
out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &val);
	if (ret < 0)
		goto out;

	if (val & BIT(offset))
		ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_WRITE, &val);
	else
		ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_READ, &val);
	if (ret < 0)
		goto out;

	ret = !!(val & BIT(offset));

out:
	mutex_unlock(&st->lock);

	return ret;
}

static void ad7768_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad7768_state *st = gpiochip_get_data(chip);
	unsigned int val;
	int ret;

	mutex_lock(&st->lock);
	ret = ad7768_spi_reg_read(st, AD7768_REG_GPIO_CONTROL, &val);
	if (ret < 0)
		goto out;

	if (val & BIT(offset))
		ad7768_spi_write_mask(st,
				      AD7768_REG_GPIO_WRITE,
				      BIT(offset),
				      (value << offset));

out:
	mutex_unlock(&st->lock);
}

int ad7768_gpio_setup(struct ad7768_state *st)
{
	int ret;

	ret = ad7768_spi_reg_write(st,
				   AD7768_REG_GPIO_CONTROL,
				   AD7768_GPIO_UGPIO_ENABLE);
	if (ret < 0)
		return ret;

	st->gpiochip.label = "ad7768";
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 5;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad7768_input_gpio;
	st->gpiochip.direction_output = ad7768_output_gpio;
	st->gpiochip.get = ad7768_get_gpio;
	st->gpiochip.set = ad7768_set_gpio;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static int ad7768_probe(struct spi_device *spi)
{
	struct gpio_desc *gpio_reset;
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->chip_info = device_get_match_data(&spi->dev);
	if (!st->chip_info) {
		st->chip_info = (void *)spi_get_device_id(spi)->driver_data;
		if (!st->chip_info)
			return PTR_ERR(st->chip_info);
	}

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);
	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad7768_reg_disable, st->vref);
	if (ret)
		return ret;

	st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);
	ret = clk_prepare_enable(st->mclk);
	if (ret < 0)
		return ret;
	ret = devm_add_action_or_reset(&spi->dev, ad7768_clk_disable, st->mclk);
	if (ret)
		return ret;

	st->spi = spi;

	/* get out of reset state */
	gpio_reset = devm_gpiod_get_optional(&spi->dev, "reset",
					     GPIOD_OUT_HIGH);
	if (IS_ERR(gpio_reset))
		return PTR_ERR(gpio_reset);

	if (gpio_reset) {
		fsleep(2);
		gpiod_set_value_cansleep(gpio_reset, 0);
		fsleep(1660);
	}

	ret = ad7768_datalines_from_dt(st);
	if (ret < 0)
		return ret;

	ad7768_set_available_sampl_freq(st);

	ad7768_set_power_mode(indio_dev, NULL, AD7768_FAST_MODE);
	if (ret < 0)
		return ret;

	ret =  ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
				     AD7768_INTERFACE_CFG_CRC_SELECT_MSK,
				     AD7768_INTERFACE_CFG_CRC_SELECT);
	if (ret < 0)
		return ret;

	ret = ad7768_gpio_setup(st);
	if (ret < 0)
		return ret;

	mutex_init(&st->lock);

	/*  If there is a reference to a dma channel, the device is not using
	 *  the axi adc
	 */
	if (device_property_present(&spi->dev, "dmas"))
		ret = ad7768_register(st, indio_dev);
	else
		ret = ad7768_register_axi_adc(st);

	return ret;
}

static const struct spi_device_id ad7768_id[] = {
	{"ad7768", (kernel_ulong_t)&ad7768_conv_chip_info},
	{"ad7768-4", (kernel_ulong_t)&ad7768_4_conv_chip_info},
	{},
};
MODULE_DEVICE_TABLE(spi, ad7768_id);

static const struct of_device_id ad7768_of_match[]  = {
	{ .compatible = "adi,ad7768", .data = &ad7768_conv_chip_info },
	{ .compatible = "adi,ad7768-4", .data = &ad7768_4_conv_chip_info },
	{},
};
MODULE_DEVICE_TABLE(of, ad7768_of_match);

static struct spi_driver ad7768_driver = {
	.driver = {
		.name	= "ad7768",
		.of_match_table = ad7768_of_match,
	},
	.probe		= ad7768_probe,
	.id_table	= ad7768_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 ADC");
MODULE_LICENSE("GPL v2");
