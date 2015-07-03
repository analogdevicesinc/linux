/*
 * ADI-AIM ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/spinlock.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/dma-buffer.h>
#include <linux/iio/dmaengine.h>
#include "ad7606.h"

static int ad7606_hw_submit_block(void *data, struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = data;
	struct ad7606_state *st = iio_priv(indio_dev);

	block->block.bytes_used = block->block.size;

	iio_dmaengine_buffer_submit_block(block, DMA_FROM_DEVICE);

	return 0;
}

static const struct iio_dma_buffer_ops ad7606_dma_buffer_ops = {
	.submit_block = ad7606_hw_submit_block,
};

int ad7606_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (dma_name == NULL)
		dma_name = "rx";

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
			&ad7606_dma_buffer_ops, indio_dev);
	if (IS_ERR_OR_NULL(buffer))
		return -ENOMEM;

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}
EXPORT_SYMBOL_GPL(ad7606_configure_ring_stream);

void ad7606_unconfigure_ring_stream(struct iio_dev *indio_dev)
{
	iio_dmaengine_buffer_free(indio_dev->buffer);
}
EXPORT_SYMBOL_GPL(ad7606_unconfigure_ring_stream);
