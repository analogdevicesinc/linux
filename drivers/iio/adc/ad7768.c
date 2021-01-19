// SPDX-License-Identifier: GPL-2.0+
/*
 * AD7768 Analog to digital converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
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

/* AD7768_DATA_CONTROL */
#define AD7768_DATA_CONTROL_SPI_RESET_MSK	GENMASK(1, 0)
#define AD7768_DATA_CONTROL_SPI_RESET_1		0x03
#define AD7768_DATA_CONTROL_SPI_RESET_2		0x02
#define AD7768_DATA_CONTROL_SPI_SYNC_MSK	BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC		BIT(7)
#define AD7768_DATA_CONTROL_SPI_SYNC_CLEAR	0

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK	GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)	(((x) & 0x3) << 0)

#define AD7768_MAX_SAMP_FREQ	256000
#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

#define AD7768_OUTPUT_MODE_TWOS_COMPLEMENT	0x01
#define AD7768_CONFIGS_PER_MODE			0x06
#define AD7768_NUM_CONFIGS			0x04
#define AD7768_MAX_RATE				(AD7768_CONFIGS_PER_MODE - 1)
#define AD7768_MIN_RATE				0

enum ad7768_power_modes {
	AD7768_LOW_POWER_MODE,
	AD7768_MEDIAN_MODE = 2,
	AD7768_FAST_MODE
};

struct ad7768_state {
	struct spi_device *spi;
	struct mutex lock;
	struct regulator *vref;
	struct clk *mclk;
	unsigned int sampling_freq;
	enum ad7768_power_modes power_mode;
	__be16 d16;
};

unsigned int ad7768_mclk_divs[] = {
	[AD7768_LOW_POWER_MODE] = 32,
	[AD7768_MEDIAN_MODE] = 8,
	[AD7768_FAST_MODE] = 4
};

int ad7768_sampling_rates[AD7768_NUM_CONFIGS][AD7768_CONFIGS_PER_MODE] = {
	[AD7768_LOW_POWER_MODE] = { 1000, 2000, 4000, 8000, 16000, 32000 },
	[AD7768_MEDIAN_MODE] = { 4000, 8000, 16000, 32000, 64000, 128000 },
	[AD7768_FAST_MODE] = { 8000, 16000, 32000, 64000, 128000, 256000 }
};

static const unsigned int ad7768_sampl_freq_avail[9] = {
	1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000
};

enum ad7768_device_ids {
	ID_AD7768
};

static const int ad7768_dec_rate[6] = {
	32, 64, 128, 256, 512, 1024
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
			       unsigned int mclk_div,
			       unsigned int freq)
{
	unsigned int mclk, dclk_div, dec, div;
	unsigned int result = 0;
	int ret = 0;

	mclk = clk_get_rate(st->mclk);

	for (dclk_div = 0; dclk_div < 4 ; dclk_div++) {
		for (dec = 0; dec < ARRAY_SIZE(ad7768_dec_rate); dec++) {
			div = mclk_div *
			      (1 <<  (3 - dclk_div)) *
			      ad7768_dec_rate[dec];

			result = DIV_ROUND_CLOSEST_ULL(mclk, div);
			if (freq == result)
				break;
		}
	}
	if (freq != result)
		return -EINVAL;

	ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
			AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
			AD7768_INTERFACE_CFG_DCLK_DIV_MODE(3 - dclk_div));
	if (ret < 0)
		return ret;

	return ad7768_spi_write_mask(st, AD7768_CH_MODE,
				     AD7768_CH_MODE_DEC_RATE_MSK,
				     AD7768_CH_MODE_DEC_RATE_MODE(dec));
}

static int ad7768_set_power_mode(struct iio_dev *dev,
				 const struct iio_chan_spec *chan,
				 unsigned int mode)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int regval;
	int ret;

	/* Check if this mode supports the current sampling rate */
	if (st->sampling_freq > ad7768_sampling_rates[mode][AD7768_MAX_RATE] ||
	    st->sampling_freq < ad7768_sampling_rates[mode][AD7768_MIN_RATE])
		return -EINVAL;

	regval = AD7768_POWER_MODE_POWER_MODE(mode);
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_POWER_MODE_MSK,
				    regval);
	if (ret < 0)
		return ret;
	/* The values for the powermode correspond for mclk div */
	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    AD7768_POWER_MODE_MCLK_DIV_MSK,
				    AD7768_POWER_MODE_MCLK_DIV_MODE(mode));
	if (ret < 0)
		return ret;

	ret = ad7768_set_clk_divs(st, ad7768_mclk_divs[mode],
				st->sampling_freq);
	if (ret < 0)
		return ret;

	ret = ad7768_sync(st);
	if (ret < 0)
		return ret;

	st->power_mode = mode;

	return ret;
}

static int ad7768_get_power_mode(struct iio_dev *dev,
				 const struct iio_chan_spec *chan)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	unsigned int regval;
	int ret;

	ret = ad7768_spi_reg_read(st, AD7768_POWER_MODE, &regval);
	if (ret < 0)
		return ret;

	st->power_mode = AD7768_POWER_MODE_GET_POWER_MODE(regval);

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

static int ad7768_set_sampling_freq(struct iio_dev *dev,
				    unsigned int freq)
{
	struct ad7768_state *st = ad7768_get_data(dev);
	int power_mode = -1;
	unsigned int i, j;
	int ret = 0;

	if (!freq)
		return -EINVAL;

	mutex_lock(&st->lock);

	for (i = 0; i < AD7768_NUM_CONFIGS; i++) {
		for (j = 0; j < AD7768_CONFIGS_PER_MODE; j++) {
			if (freq == ad7768_sampling_rates[i][j]) {
				power_mode = i;
				break;
			}
		}
	}

	if (power_mode == -1) {
		ret = -EINVAL;
		goto freq_err;
	}

	ret = ad7768_set_clk_divs(st, ad7768_mclk_divs[power_mode], freq);
	if (ret < 0)
		goto freq_err;

	st->sampling_freq = freq;

	ret = ad7768_set_power_mode(dev, NULL, power_mode);
	if (ret < 0)
		goto freq_err;

freq_err:
	mutex_unlock(&st->lock);

	return ret;
}

static ssize_t ad7768_attr_sampl_freq_avail(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct ad7768_state *st;
	int i, len = 0;

	st = ad7768_get_data(indio_dev);
	for (i = 0; i < ARRAY_SIZE(ad7768_sampl_freq_avail); i++)
		len += scnprintf(buf + len, PAGE_SIZE - len, "%d ",
			ad7768_sampl_freq_avail[i]);
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

#define DECLARE_AD7768_CHANNELS(name)	\
static struct iio_chan_spec name[] = {	\
		AD7768_CHAN(0), \
		AD7768_CHAN(1), \
		AD7768_CHAN(2), \
		AD7768_CHAN(3), \
		AD7768_CHAN(4), \
		AD7768_CHAN(5), \
		AD7768_CHAN(6), \
		AD7768_CHAN(7), \
}

DECLARE_AD7768_CHANNELS(ad7768_channels);

static const struct axiadc_chip_info conv_chip_info = {
	.name = "ad7768_axi_adc",
	.max_rate = 256000000UL,
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

static int ad7768_register_axi_adc(struct ad7768_state *st)
{
	struct axiadc_converter	*conv;

	conv = devm_kzalloc(&st->spi->dev, sizeof(*conv), GFP_KERNEL);
	if (conv == NULL)
		return -ENOMEM;

	conv->spi = st->spi;
	conv->clk = st->mclk;
	conv->chip_info = &conv_chip_info;
	conv->adc_output_mode = AD7768_OUTPUT_MODE_TWOS_COMPLEMENT;
	conv->reg_access = &ad7768_reg_access;
	conv->write_raw = &ad7768_write_raw;
	conv->read_raw = &ad7768_read_raw;
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
	indio_dev->channels = ad7768_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7768_channels);
	indio_dev->info = &ad7768_info;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						&dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	return devm_iio_device_register(&st->spi->dev, indio_dev);
}

static int ad7768_probe(struct spi_device *spi)
{
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

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

	ret = ad7768_set_sampling_freq(indio_dev, AD7768_MAX_SAMP_FREQ);
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
	{"ad7768", ID_AD7768},
	{}
};
MODULE_DEVICE_TABLE(spi, ad7768_id);

static struct spi_driver ad7768_driver = {
	.driver = {
		.name	= "ad7768",
	},
	.probe		= ad7768_probe,
	.id_table	= ad7768_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 ADC");
MODULE_LICENSE("GPL v2");
