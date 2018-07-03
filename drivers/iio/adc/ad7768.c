// SPDX-License-Identifier: GPL-2.0+
/*
 * AD7768 Analog to digital converters driver
 *
 * Copyright 2018 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AD7768_WR_FLAG_MSK(x)	(0x80 | ((x) & 0x7F))

struct ad7768_state {
	struct spi_device *spi;
	struct mutex lock;
	struct regulator *vref;
	__be16 d16;
};

enum ad7768_device_ids {
	ID_AD7768
};

#define AD7768_CHAN(index)						\
	{								\
		.type = IIO_VOLTAGE,					\
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
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

static int ad7768_spi_reg_read(struct ad7768_state *st, unsigned int addr)
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

	return be16_to_cpu(st->d16);
}

static int ad7768_spi_reg_write(struct ad7768_state *st,
				unsigned int addr,
				unsigned int val)
{
	st->d16 = cpu_to_be16(((addr & 0x7F) << 8) | val);

	return spi_write(st->spi, &st->d16, sizeof(st->d16));
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
		ret = ad7768_spi_reg_read(st, reg);
		if (ret < 0)
			goto exit;

		*readval = ret;
		ret = 0;
	} else {
		ret = ad7768_spi_reg_write(st, reg, writeval);
	}
exit:
	mutex_unlock(&st->lock);

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
	}

	return -EINVAL;
}

static const struct iio_info ad7768_info = {
	.read_raw = &ad7768_read_raw,
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

	spi_set_drvdata(spi, indio_dev);

	st->spi = spi;

	mutex_init(&st->lock);

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->channels = ad7768_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad7768_channels);
	indio_dev->info = &ad7768_info;

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
					    &dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		goto error_disable_reg;

	return 0;

error_disable_reg:
	regulator_disable(st->vref);

	return ret;
}

static int ad7768_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad7768_state *st = iio_priv(indio_dev);

	return regulator_disable(st->vref);
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
	.remove		= ad7768_remove,
	.id_table	= ad7768_id,
};
module_spi_driver(ad7768_driver);

MODULE_AUTHOR("Stefan Popa <stefan.popa@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD7768 ADC");
MODULE_LICENSE("GPL v2");
