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

#include "../iio.h"
#include "../sysfs.h"
#include "../buffer.h"
#include "../ring_hw.h"
#include "cf_ad9467.h"

static int aim_read_first_n_hw_rb(struct iio_buffer *r,
				      size_t count, char __user *buf)
{
	struct iio_hw_buffer *hw_ring = iio_to_hw_buf(r);
	struct iio_dev *indio_dev = hw_ring->private;
	struct aim_state *st = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	unsigned rcount;
	int ret;

	mutex_lock(&st->lock);
	if (count == 0) {
		ret = -EINVAL;
		goto error_ret;
	}

	if (count % 8)
		rcount = (count + 8) & 0xFFFFFFF8;
	else
		rcount = count;

	st->buf_virt = dma_alloc_coherent(indio_dev->dev.parent,
					  PAGE_ALIGN(rcount), &st->buf_phys,
					  GFP_KERNEL);
	if (st->buf_virt == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	desc = dmaengine_prep_slave_single(st->rx_chan, st->buf_phys, rcount,
					   DMA_FROM_DEVICE, DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(indio_dev->dev.parent,
			"Failed to allocate a dma descriptor\n");
		ret = -ENOMEM;
		goto error_free;
	}

	desc->callback = (dma_async_tx_callback) complete;
	desc->callback_param = &st->dma_complete;

	cookie = dmaengine_submit(desc);
	if (cookie < 0) {
		dev_err(indio_dev->dev.parent,
			"Failed to submit a dma transfer\n");
		ret = cookie;
		goto error_free;
	}

	dma_async_issue_pending(st->rx_chan);

	aim_write(st, AD9467_PCORE_DMA_CTRL, 0);
	aim_read(st, AD9467_PCORE_DMA_STAT);
	aim_write(st, AD9467_PCORE_DMA_CTRL,
		  AD9647_DMA_CAP_EN | AD9647_DMA_CNT((rcount / 8) - 1));

	ret = wait_for_completion_interruptible_timeout(&st->dma_complete,
							2 * HZ);
	if (ret == 0) {
		ret = -ETIMEDOUT;
		goto error_free;
	} else if (ret < 0) {
		goto error_free;
	}

	if (copy_to_user(buf, st->buf_virt, count))
		ret = -EFAULT;

error_free:
	dma_free_coherent(indio_dev->dev.parent, PAGE_ALIGN(rcount),
			  st->buf_virt, st->buf_phys);
	r->stufftoread = 0;
error_ret:
	mutex_unlock(&st->lock);

	return ret ? ret : count;
}

static int aim_ring_get_length(struct iio_buffer *r)
{
	struct iio_hw_buffer *hw_ring = iio_to_hw_buf(r);
	struct iio_dev *indio_dev = hw_ring->private;
	struct aim_state *st = iio_priv(indio_dev);

	return st->ring_lenght;
}

static int aim_ring_set_length(struct iio_buffer *r, int lenght)
{
	struct iio_hw_buffer *hw_ring = iio_to_hw_buf(r);
	struct aim_state *st = iio_priv(hw_ring->private);

	st->ring_lenght = lenght;

	return 0;
}

static int aim_ring_get_bytes_per_datum(struct iio_buffer *r)
{
	struct iio_hw_buffer *hw_ring = iio_to_hw_buf(r);
	struct aim_state *st = iio_priv(hw_ring->private);

	return st->bytes_per_datum;
}

static IIO_BUFFER_ENABLE_ATTR;
static IIO_BUFFER_LENGTH_ATTR;

static struct attribute *aim_ring_attributes[] = {
	&dev_attr_length.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group aim_ring_attr = {
	.attrs = aim_ring_attributes,
	.name = "buffer",
};

static struct iio_buffer *aim_rb_allocate(struct iio_dev *indio_dev)
{
	struct iio_buffer *buf;
	struct iio_hw_buffer *ring;

	ring = kzalloc(sizeof *ring, GFP_KERNEL);
	if (!ring)
		return NULL;

	ring->private = indio_dev;
	buf = &ring->buf;
	buf->attrs = &aim_ring_attr;
	iio_buffer_init(buf);

	return buf;
}

static inline void aim_rb_free(struct iio_buffer *r)
{
	kfree(iio_to_hw_buf(r));
}

static const struct iio_buffer_access_funcs aim_ring_access_funcs = {
	.read_first_n = &aim_read_first_n_hw_rb,
	.get_length = &aim_ring_get_length,
	.set_length = &aim_ring_set_length,
	.get_bytes_per_datum = &aim_ring_get_bytes_per_datum,
};

int aim_configure_ring(struct iio_dev *indio_dev)
{
	indio_dev->buffer = aim_rb_allocate(indio_dev);
	if (indio_dev->buffer == NULL)
		return -ENOMEM;

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->buffer->access = &aim_ring_access_funcs;

	return 0;
}

void aim_unconfigure_ring(struct iio_dev *indio_dev)
{
	aim_rb_free(indio_dev->buffer);
}
