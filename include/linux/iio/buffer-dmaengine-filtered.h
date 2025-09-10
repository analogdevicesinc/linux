/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2025 Analog Devices Inc.
 * Author: Marcelo Schmitt <marcelo.schmitt@analog.com>
 * Based on buffer-dmaengine.h.
 */

#ifndef __IIO_DMAENGINE_FILTERED_H__
#define __IIO_DMAENGINE_FILTERED_H__

#include <linux/iio/buffer.h>

struct iio_dev;
struct device;
struct dma_chan;
struct iio_buffer;
struct iio_dma_buffer_ops;
struct iio_dma_buffer_block;
struct iio_dma_buffer_queue;

int iio_dmaengine_filtered_buffer_submit_block(struct iio_dma_buffer_queue *queue,
					       struct iio_dma_buffer_block *block);

struct iio_buffer *iio_dmaengine_filtered_buffer_setup_ext(struct device *dev,
							   struct iio_dev *indio_dev,
							   const char *channel,
							   enum iio_buffer_direction dir);

int devm_iio_dmaengine_filtered_buffer_setup_with_handle(struct device *dev,
							 struct iio_dev *indio_dev,
							 struct dma_chan *chan,
							 enum iio_buffer_direction dir);

#endif
