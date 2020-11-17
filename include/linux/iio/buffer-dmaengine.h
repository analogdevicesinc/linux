/*
 * Copyright 2014-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2 or later.
 */

#ifndef __IIO_DMAENGINE_H__
#define __IIO_DMAENGINE_H__

struct device;
struct iio_buffer;
struct iio_dma_buffer_ops;
struct iio_dma_buffer_block;
struct iio_dma_buffer_queue;

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block, int direction);
void iio_dmaengine_buffer_abort(struct iio_dma_buffer_queue *queue);

struct iio_buffer *iio_dmaengine_buffer_alloc(struct device *dev,
	const char *channel, const struct iio_dma_buffer_ops *ops, void *data);
void iio_dmaengine_buffer_free(struct iio_buffer *buffer);

struct iio_buffer *devm_iio_dmaengine_buffer_alloc(struct device *dev,
						   const char *channel,
						   const struct iio_dma_buffer_ops *ops,
						   void *data);

#endif
