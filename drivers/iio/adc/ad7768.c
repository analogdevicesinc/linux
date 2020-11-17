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
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

/* AD7768 registers definition */
#define AD7768_CH_MODE				0x01
#define AD7768_POWER_MODE			0x04
#define AD7768_INTERFACE_CFG			0x07

/* AD7768_CH_MODE */
#define AD7768_CH_MODE_FILTER_TYPE_MSK		BIT(3)
#define AD7768_CH_MODE_FILTER_TYPE_MODE(x)	(((x) & 0x1) << 3)
#define AD7768_CH_MODE_DEC_RATE_MSK		GENMASK(2, 0)
#define AD7768_CH_MODE_DEC_RATE_MODE(x)		(((x) & 0x7) << 0)

/* AD7768_POWER_MODE */
#define AD7768_POWER_MODE_POWER_MODE_MSK	GENMASK(5, 4)
#define AD7768_POWER_MODE_POWER_MODE(x)		(((x) & 0x3) << 4)
#define AD7768_POWER_MODE_MCLK_DIV_MSK		GENMASK(1, 0)
#define AD7768_POWER_MODE_MCLK_DIV_MODE(x)	(((x) & 0x3) << 0)

/* AD7768_INTERFACE_CFG */
#define AD7768_INTERFACE_CFG_DCLK_DIV_MSK	GENMASK(1, 0)
#define AD7768_INTERFACE_CFG_DCLK_DIV_MODE(x)	(((x) & 0x3) << 0)

#define AD7768_MAX_SAMP_FREQ	256000
#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

struct ad7768_state {
	struct spi_device *spi;
	struct mutex lock;
	struct regulator *vref;
	struct clk *mclk;
	unsigned int sampling_freq;
	__be16 d16;
};

enum ad7768_device_ids {
	ID_AD7768
};

enum ad7768_mclk_div {
	AD7768_MCLK_DIV_32,
	AD7768_MCLK_DIV_8 = 2,
	AD7768_MCLK_DIV_4
};

enum ad7768_dclk_div {
	AD7768_DCLK_DIV_8,
	AD7768_DCLK_DIV_4,
	AD7768_DCLK_DIV_2,
	AD7768_DCLK_DIV_1
};

struct ad7768_div_settings {
	unsigned int mclk_div;
	unsigned int dclk_div;
	unsigned int clk_div;
};

static const struct ad7768_div_settings ad7768_div_set[] = {
	{ AD7768_MCLK_DIV_4, AD7768_DCLK_DIV_1, 4 },
	{ AD7768_MCLK_DIV_4, AD7768_DCLK_DIV_2, 8 },
	{ AD7768_MCLK_DIV_8, AD7768_DCLK_DIV_2, 16 },
	{ AD7768_MCLK_DIV_8, AD7768_DCLK_DIV_4, 32 },
	{ AD7768_MCLK_DIV_8, AD7768_DCLK_DIV_8, 64 },
	{ AD7768_MCLK_DIV_32, AD7768_DCLK_DIV_4, 128 },
	{ AD7768_MCLK_DIV_32, AD7768_DCLK_DIV_8, 256 }
};

static const int ad7768_dec_rate[6] = {
	32, 64, 128, 256, 512, 1024
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
	struct ad7768_state *st = iio_priv(indio_dev);
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

static int ad7768_set_samp_freq(struct ad7768_state *st,
				enum ad7768_mclk_div mclk_div,
				enum ad7768_dclk_div dclk_div,
				unsigned int dec_rate_index)
{
	unsigned int power_mode;
	unsigned long int power_mask;
	int ret;

	mutex_lock(&st->lock);
	power_mode = AD7768_POWER_MODE_POWER_MODE(mclk_div) |
		     AD7768_POWER_MODE_MCLK_DIV_MODE(mclk_div);
	power_mask = AD7768_POWER_MODE_POWER_MODE_MSK |
		     AD7768_POWER_MODE_MCLK_DIV_MSK;

	ret = ad7768_spi_write_mask(st, AD7768_POWER_MODE,
				    power_mask, power_mode);
	if (ret < 0)
		goto err_unlock;

	ret = ad7768_spi_write_mask(st, AD7768_INTERFACE_CFG,
			AD7768_INTERFACE_CFG_DCLK_DIV_MSK,
			AD7768_INTERFACE_CFG_DCLK_DIV_MODE(dclk_div));
	if (ret < 0)
		goto err_unlock;

	ret = ad7768_spi_write_mask(st, AD7768_CH_MODE,
			AD7768_CH_MODE_DEC_RATE_MSK,
			AD7768_CH_MODE_DEC_RATE_MODE(dec_rate_index));
err_unlock:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad7768_samp_freq_config(struct ad7768_state *st,
				   unsigned int freq)
{
	unsigned int res, dclk, calc_freq, diff_new;
	unsigned int div_set_idx, dec_rate_idx, mclk_freq;
	int diff_old, i, j, ret;

	diff_old = S32_MAX;
	div_set_idx = 0;
	dec_rate_idx = 0;
	calc_freq = AD7768_MAX_SAMP_FREQ;
	mclk_freq = clk_get_rate(st->mclk);

	/*
	 * Sampling freq is defined by MCLK, MCLK_DIV, DCLK_DIV and DEC_RATE:
	 * samp_freq = MCLK / (MCLK_DIV * DCLK_DIV * DEC_RATE)
	 */
	for (i = 0; i < ARRAY_SIZE(ad7768_div_set); i++) {
		if (calc_freq <= freq)
			break;
		/* DCLK = MCLK / (MCLK_DIV * DCLK_DIV) */
		dclk = DIV_ROUND_CLOSEST(mclk_freq, ad7768_div_set[i].clk_div);
		/*
		 * Go through each decimation rate and determine a valid value
		 * for the sampling frequency which is closest to the desired
		 * sampling frequency
		 */
		for (j = 0; j < ARRAY_SIZE(ad7768_dec_rate); j++) {
			res = DIV_ROUND_CLOSEST(dclk, ad7768_dec_rate[j]);
			diff_new = abs(freq - res);
			if (diff_new < diff_old) {
				diff_old = diff_new;
				div_set_idx = j;
				dec_rate_idx = i;
				calc_freq = res;
			}
		}
	}

	ret = ad7768_set_samp_freq(st, ad7768_div_set[div_set_idx].mclk_div,
				   ad7768_div_set[div_set_idx].dclk_div,
				   dec_rate_idx);
	if (ret < 0)
		return ret;

	st->sampling_freq = calc_freq;

	return ret;
}

static int ad7768_read_raw(struct iio_dev *indio_dev,
			   const struct iio_chan_spec *chan,
			   int *val, int *val2, long info)
{
	struct ad7768_state *st = iio_priv(indio_dev);
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
	struct ad7768_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad7768_samp_freq_config(st, val);
	default:
		return -EINVAL;
	}
}

static const struct iio_info ad7768_info = {
	.read_raw = &ad7768_read_raw,
	.write_raw = &ad7768_write_raw,
	.debugfs_reg_access = &ad7768_reg_access,
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

static int ad7768_probe(struct spi_device *spi)
{
	struct ad7768_state *st;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	st->mclk = devm_clk_get(&spi->dev, "mclk");
	if (IS_ERR(st->mclk))
		return PTR_ERR(st->mclk);

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	mutex_init(&st->lock);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = ad7768_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7768_channels);
	indio_dev->info = &ad7768_info;

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
						 &dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad7768_reg_disable, st->vref);
	if (ret)
		return ret;

	ret = clk_prepare_enable(st->mclk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&spi->dev, ad7768_clk_disable, st->mclk);
	if (ret)
		return ret;

	ret = ad7768_samp_freq_config(st, AD7768_MAX_SAMP_FREQ);
	if (ret < 0)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
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
