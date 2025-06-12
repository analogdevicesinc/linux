/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2014-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 */

#ifndef __IIO_DMAENGINE_H__
#define __IIO_DMAENGINE_H__

#include <linux/iio/buffer.h>

struct iio_dev;
struct device;
struct iio_buffer;
struct iio_dma_buffer_ops;
struct iio_dma_buffer_block;
struct iio_dma_buffer_queue;

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block);
void iio_dmaengine_buffer_abort(struct iio_dma_buffer_queue *queue);

struct iio_buffer *iio_dmaengine_buffer_alloc(struct device *dev,
					      const char *channel,
					      const struct iio_dma_buffer_ops *ops,
					      void *data);

int devm_iio_dmaengine_buffer_setup(struct device *dev,
				    struct iio_dev *indio_dev,
				    const char *channel,
				    enum iio_buffer_direction dir);

#endif
