// SPDX-License-Identifier: GPL-2.0+
/*
 * AD400x SPI ADC driver
 *
 * Copyright 2018 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/sysfs.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#define AD400X_READ_COMMAND	0x54
#define AD400X_WRITE_COMMAND	0x14
#define AD400X_RESERVED_MSK	0xE0

#define AD400X_TURBO_MODE(x)	FIELD_PREP(BIT_MASK(1), x)
#define AD400X_HIGH_Z_MODE(x)	FIELD_PREP(BIT_MASK(2), x)

enum ad400x_ids {
	ID_AD4003,
	ID_AD4007,
	ID_AD4011,
	ID_AD4020,
};

struct ad400x_state {
	struct spi_device *spi;
	struct regulator *vref;

	bool bus_locked;

	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;

	bool turbo_mode;
	bool high_z_mode;

	unsigned int num_bits;

	/*
	 * DMA (thus cache coherency maintenance) requires the
	 * transfer buffers to live in their own cache lines.
	 */
	uint8_t	data[4] ____cacheline_aligned;
};

static int ad400x_write_reg(struct ad400x_state *st, uint8_t val)
{
	struct spi_transfer t = {
		.tx_buf		= st->data,
		.len		= 4,
		.bits_per_word	= st->num_bits,
	};
	struct spi_message m;

	st->data[0] = AD400X_WRITE_COMMAND;
	st->data[1] = val;

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	if (st->bus_locked)
		return spi_sync_locked(st->spi, &m);

	return spi_sync(st->spi, &m);
}

static int ad400x_read_reg(struct ad400x_state *st, unsigned int *val)
{
	struct spi_message m;
	struct spi_transfer t = {0};
	int ret;

	st->data[0] = AD400X_READ_COMMAND;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 2;
	t.bits_per_word = 16; /* reg reads are only 16 clocks pulses */

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->bus_locked)
		ret = spi_sync_locked(st->spi, &m);
	else
		ret = spi_sync(st->spi, &m);

	if (ret < 0)
		return ret;

	*val = st->data[0];

	return ret;
}

static int ad400x_read_sample(struct ad400x_state *st, uint32_t *val)
{
	struct spi_message m;
	struct spi_transfer t = {0};
	uint8_t input[4] = {0};
	int ret;

	t.rx_buf = input;
	t.len = 4;
	t.bits_per_word = st->num_bits;

	spi_message_init_with_transfers(&m, &t, 1);

	if (st->bus_locked)
		ret = spi_sync_locked(st->spi, &m);
	else
		ret = spi_sync(st->spi, &m);

	if (ret < 0)
		return ret;

	*val = (input[2] << 16) | (input[1] << 8) | input[0];

	return ret;
}

static int ad400x_set_mode(struct ad400x_state *st)
{
	uint8_t mode;
	int ret;

	mode = AD400X_TURBO_MODE(st->turbo_mode) |
		AD400X_HIGH_Z_MODE(st->high_z_mode);

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = ad400x_write_reg(st, mode);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);

	return ret;
}

#define AD400X_CHANNEL(real_bits)					\
	{								\
		.type = IIO_VOLTAGE,					\
		.indexed = 1,						\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			BIT(IIO_CHAN_INFO_SCALE) |			\
			BIT(IIO_CHAN_INFO_OFFSET),			\
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),\
		.scan_type = {						\
			.sign = 's',					\
			.realbits = real_bits,				\
			.storagebits = 32,				\
		},							\
	}								\

static const struct iio_chan_spec ad400x_channels[] = {
	AD400X_CHANNEL(18),
};

static const struct iio_chan_spec ad4020_channel[] = {
	AD400X_CHANNEL(20),
};

static int ad400x_single_conversion(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	unsigned int sample, raw_sample;
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = ad400x_read_sample(st, &raw_sample);
	iio_device_release_direct_mode(indio_dev);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);

	if (ret)
		return ret;

	sample = raw_sample >> chan->scan_type.shift;
	sample &= (1 << chan->scan_type.realbits) - 1;

	*val = sample;

	return IIO_VAL_INT;
}

static int ad400x_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad400x_single_conversion(indio_dev, chan, val);
	case IIO_CHAN_INFO_SCALE:
		ret = regulator_get_voltage(st->vref);
		if (ret < 0)
			return ret;

		*val = ret / 1000;
		*val2 = chan->scan_type.realbits;

		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = 1800000;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_OFFSET:
		*val = -(1 << chan->scan_type.realbits);

		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

static int ad400x_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&indio_dev->mlock);
	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	if (readval)
		ret = ad400x_read_reg(st, readval);
	else
		ret = ad400x_write_reg(st, writeval);

	st->bus_locked = false;
	spi_bus_unlock(st->spi->master);
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const struct iio_info ad400x_info = {
	.read_raw = &ad400x_read_raw,
	.debugfs_reg_access = &ad400x_reg_access,
};

static const struct spi_device_id ad400x_id[] = {
	{"ad4003", ID_AD4003},
	{"ad4007", ID_AD4007},
	{"ad4011", ID_AD4011},
	{"ad4020", ID_AD4020},
	{}
};
MODULE_DEVICE_TABLE(spi, ad400x_id);

static int ad400x_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad400x_state *st = iio_priv(indio_dev);
	int ret;

	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = (void *)-1;
	st->spi_transfer.len = 4;
	st->spi_transfer.bits_per_word = st->num_bits;

	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);

	spi_bus_lock(st->spi->master);
	st->bus_locked = true;

	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret < 0)
		return ret;

	spi_engine_offload_enable(st->spi, true);

	return 0;
}

static int ad400x_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct ad400x_state *st = iio_priv(indio_dev);

	spi_engine_offload_enable(st->spi, false);

	st->bus_locked = false;
	return spi_bus_unlock(st->spi->master);
}

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

static const struct iio_buffer_setup_ops ad400x_buffer_setup_ops = {
	.postenable = &ad400x_buffer_postenable,
	.postdisable = &ad400x_buffer_postdisable,
};

static int ad400x_probe(struct spi_device *spi)
{
	struct ad400x_state *st;
	struct iio_dev *indio_dev;
	struct iio_buffer *buffer;
	int ret, dev_id;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	dev_id = spi_get_device_id(spi)->driver_data;
	st = iio_priv(indio_dev);
	st->spi = spi;

	st->vref = devm_regulator_get(&spi->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret < 0)
		return ret;

	indio_dev->dev.parent = &spi->dev;
	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &ad400x_buffer_setup_ops;
	indio_dev->info = &ad400x_info;

	if (dev_id == ID_AD4020)
		indio_dev->channels = ad4020_channel;
	else
		indio_dev->channels = ad400x_channels;

	indio_dev->num_channels = 1;
	st->num_bits = indio_dev->channels->scan_type.realbits;

	/* Set turbo mode */
	st->turbo_mode = true;
	ret = ad400x_set_mode(st);
	if (ret < 0)
		return ret;

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "rx",
					    &dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	ret = devm_iio_device_register(&spi->dev, indio_dev);
	if (ret < 0)
		goto error;

	return 0;

error:
	iio_dmaengine_buffer_free(indio_dev->buffer);
	regulator_disable(st->vref);

	return ret;
}

static int ad400x_remove(struct spi_device *spi)
{
	struct iio_dev *indio_dev = spi_get_drvdata(spi);
	struct ad400x_state *st = iio_priv(indio_dev);

	iio_dmaengine_buffer_free(indio_dev->buffer);
	regulator_disable(st->vref);

	return 0;
}

static const struct of_device_id ad400x_of_match[] = {
	{ .compatible = "adi,ad4003" },
	{ .compatible = "adi,ad4007" },
	{ .compatible = "adi,ad4011" },
	{ .compatible = "adi,ad4020" },
	{ },
};
MODULE_DEVICE_TABLE(of, ad400x_of_match);

static struct spi_driver ad400x_driver = {
	.driver = {
		.name   = "ad400x",
		.of_match_table = ad400x_of_match,
	},
	.probe          = ad400x_probe,
	.remove		= ad400x_remove,
	.id_table       = ad400x_id,
};
module_spi_driver(ad400x_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD400x ADC driver");
MODULE_LICENSE("GPL v2");
