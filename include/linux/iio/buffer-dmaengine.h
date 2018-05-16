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
struct dma_chan;
struct iio_buffer;
struct iio_dma_buffer_ops;
struct iio_dma_buffer_block;
struct iio_dma_buffer_queue;

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block);
void iio_dmaengine_buffer_abort(struct iio_dma_buffer_queue *queue);

void iio_dmaengine_buffer_teardown(struct iio_buffer *buffer);
struct iio_buffer *iio_dmaengine_buffer_setup_ext(struct device *dev,
						  struct iio_dev *indio_dev,
						  const char *channel,
						  enum iio_buffer_direction dir);

#define iio_dmaengine_buffer_setup(dev, indio_dev, channel)	\
	iio_dmaengine_buffer_setup_ext(dev, indio_dev, channel,	\
				       IIO_BUFFER_DIRECTION_IN)

int devm_iio_dmaengine_buffer_setup_ext(struct device *dev,
					struct iio_dev *indio_dev,
					const char *channel,
					enum iio_buffer_direction dir);
int devm_iio_dmaengine_buffer_setup_with_handle(struct device *dev,
						struct iio_dev *indio_dev,
						struct dma_chan *chan,
						enum iio_buffer_direction dir);

#define devm_iio_dmaengine_buffer_setup(dev, indio_dev, channel)	\
	devm_iio_dmaengine_buffer_setup_ext(dev, indio_dev, channel,	\
					    IIO_BUFFER_DIRECTION_IN)

int devm_iio_dmaengine_buffer_setup_with_ops(struct device *dev,
					     struct iio_dev *indio_dev,
					     const char *channel,
					     enum iio_buffer_direction dir,
					     const struct iio_dma_buffer_ops *ops,
					     void *data);

#endif
