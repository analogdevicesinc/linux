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

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include "../../staging/iio/ring_hw.h"
#include "cf_axi_adc.h"

#define AXIADC_MAX_DMA_SIZE		(4 * 1024 * 1024) /* Randomly picked */

struct axiadc_buf {
	struct iio_buffer buffer;
	void *buf_virt;
	dma_addr_t buf_phys;
	size_t read_offs;
	int compl_stat;
	struct completion complete;
	struct dma_chan *chan;
	unsigned int rcount;
	struct mutex lock;
	struct iio_dev *indio_dev;
};

static struct axiadc_buf *iio_buffer_to_axiadc_buf(struct iio_buffer *buf)
{
	return container_of(buf, struct axiadc_buf, buffer);
}

static int axiadc_read_first_n_hw_rb(struct iio_buffer *r,
				     size_t count, char __user *buf)
{
	struct axiadc_buf *axiadc_buf = iio_buffer_to_axiadc_buf(r);
	struct iio_dev *indio_dev = axiadc_buf->indio_dev;
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned stat, dma_stat;
	int ret = 0;

	mutex_lock(&axiadc_buf->lock);

	if (!axiadc_buf->read_offs) {
		ret = wait_for_completion_interruptible_timeout(
			    &axiadc_buf->complete, 4 * HZ);
		stat = axiadc_read(st, ADI_REG_STATUS);
		dma_stat = axiadc_read(st, ADI_REG_DMA_STATUS);

		if (axiadc_buf->compl_stat < 0) {
			ret = axiadc_buf->compl_stat;
			goto error_ret;
		} else if (ret == 0) {
			ret = -ETIMEDOUT;
			dev_err(indio_dev->dev.parent,
				"timeout: DMA_STAT 0x%X, ADC_STAT 0x%X\n",
				dma_stat, stat);
			goto error_ret;
		} else if (ret < 0) {
			goto error_ret;
		}

		if ((stat & ADI_MUX_OVER_RANGE) /*|| !(stat & ADI_STATUS)*/ || dma_stat)
			dev_warn(indio_dev->dev.parent,
				"STATUS: DMA_STAT 0x%X, ADC_STAT 0x%X\n",
				dma_stat, stat);

	}

	count = min(count, r->length - axiadc_buf->read_offs);

	if (copy_to_user(buf, axiadc_buf->buf_virt + axiadc_buf->read_offs,
		count))
		ret = -EFAULT;

	axiadc_buf->read_offs += count;

error_ret:
	r->stufftoread = count != 0;
	mutex_unlock(&axiadc_buf->lock);

	return ret < 0 ? ret : count;
}

static int axiadc_ring_set_length(struct iio_buffer *r, int length)
{
	r->length = length;

	return 0;
}

static int axiadc_ring_set_bytes_per_datum(struct iio_buffer *r, size_t bpd)
{
	r->bytes_per_datum = bpd;
	return 0;
}

static void axiadc_ring_release(struct iio_buffer *r)
{
	struct axiadc_buf *axiadc_buf = iio_buffer_to_axiadc_buf(r);
	kfree(axiadc_buf);
}

static const struct iio_buffer_access_funcs axiadc_ring_access_funcs = {
	.read = &axiadc_read_first_n_hw_rb,
	.set_length = &axiadc_ring_set_length,
	.set_bytes_per_datum = &axiadc_ring_set_bytes_per_datum,
	.release = axiadc_ring_release,

	.modes = INDIO_BUFFER_HARDWARE,
};

static void axiadc_hw_transfer_done(void *data)
{
	struct axiadc_buf *axiadc_buf = data;

	axiadc_buf->buffer.stufftoread = 1;
	wake_up_interruptible_poll(&axiadc_buf->buffer.pollq,
		POLLIN | POLLRDNORM);
	complete(&axiadc_buf->complete);
}

static int __axiadc_hw_ring_state_set(struct iio_dev *indio_dev, bool state)
{
	struct axiadc_buf *axiadc_buf = iio_buffer_to_axiadc_buf(indio_dev->buffer);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	dma_cookie_t cookie;
	int ret = 0;

	if (!state) {
		if (!completion_done(&axiadc_buf->complete)) {
			axiadc_buf->compl_stat = -EPERM;
			dmaengine_terminate_all(axiadc_buf->chan);
			complete(&axiadc_buf->complete);
		}

		return 0;
	}

	axiadc_buf->compl_stat = 0;
	if (indio_dev->buffer->length == 0) {
		ret = -EINVAL;
		goto error_ret;
	}

	axiadc_buf->rcount = ALIGN(indio_dev->buffer->length, 8);
	if (axiadc_buf->rcount > AXIADC_MAX_DMA_SIZE) {
		ret = -EINVAL;
		goto error_ret;
	}

	desc = dmaengine_prep_slave_single(axiadc_buf->chan,
		axiadc_buf->buf_phys, axiadc_buf->rcount, DMA_FROM_DEVICE,
		DMA_PREP_INTERRUPT);
	if (!desc) {
		dev_err(indio_dev->dev.parent,
			"Failed to allocate a dma descriptor\n");
		ret = -ENOMEM;
		goto error_ret;
	}

	desc->callback = axiadc_hw_transfer_done;
	desc->callback_param = axiadc_buf;

	cookie = dmaengine_submit(desc);
	if (cookie < 0) {
		dev_err(indio_dev->dev.parent,
			"Failed to submit a dma transfer\n");
		ret = cookie;
		goto error_ret;
	}
	reinit_completion(&axiadc_buf->complete);
	axiadc_buf->read_offs = 0;
	dma_async_issue_pending(axiadc_buf->chan);

	axiadc_write(st, ADI_REG_DMA_CNTRL, 0);
	axiadc_write(st, ADI_REG_STATUS, ~0);
	axiadc_write(st, ADI_REG_DMA_STATUS, ~0);
	axiadc_write(st, ADI_REG_DMA_COUNT, axiadc_buf->rcount);
	axiadc_write(st, ADI_REG_DMA_CNTRL, ADI_DMA_START);

	return 0;
error_ret:
	return ret;
}

static int axiadc_hw_ring_preenable(struct iio_dev *indio_dev)
{
	return __axiadc_hw_ring_state_set(indio_dev, 1);
}

static int axiadc_hw_ring_postdisable(struct iio_dev *indio_dev)
{
	return __axiadc_hw_ring_state_set(indio_dev, 0);
}

static const struct iio_buffer_setup_ops axiadc_ring_setup_ops = {
	.preenable = &axiadc_hw_ring_preenable,
	.postdisable = &axiadc_hw_ring_postdisable,
};

int axiadc_configure_ring(struct iio_dev *indio_dev, const char *dma_name)
{
	struct axiadc_buf *axiadc_buf;
	int ret;

	if (!dma_name)
		dma_name = "rx";

	axiadc_buf = kzalloc(sizeof(*axiadc_buf), GFP_KERNEL);
	if (!axiadc_buf)
		return -ENOMEM;

	axiadc_buf->chan = dma_request_slave_channel(indio_dev->dev.parent,
		dma_name);
	if (!axiadc_buf->chan) {
		ret = -EPROBE_DEFER;
		goto err_free;
	}

	init_completion(&axiadc_buf->complete);
	mutex_init(&axiadc_buf->lock);
	axiadc_buf->indio_dev = indio_dev;
	iio_buffer_init(&axiadc_buf->buffer);

	iio_device_attach_buffer(indio_dev, &axiadc_buf->buffer);
	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->buffer->access = &axiadc_ring_access_funcs;
	indio_dev->setup_ops = &axiadc_ring_setup_ops;

	axiadc_buf->buf_virt = dma_alloc_coherent(indio_dev->dev.parent,
					  PAGE_ALIGN(AXIADC_MAX_DMA_SIZE),
					  &axiadc_buf->buf_phys,
					  GFP_KERNEL);
	if (!axiadc_buf->buf_virt) {
		dev_err(indio_dev->dev.parent,
			"Failed to allocate a dma memory\n");
		ret = -ENOMEM;
		goto err_release_dma;
	}

	return 0;
err_release_dma:
	dma_release_channel(axiadc_buf->chan);
err_free:
	kfree(axiadc_buf);

	return ret;
}
EXPORT_SYMBOL_GPL(axiadc_configure_ring);

void axiadc_unconfigure_ring(struct iio_dev *indio_dev)
{
	struct axiadc_buf *axiadc_buf = iio_buffer_to_axiadc_buf(indio_dev->buffer);

	dma_release_channel(axiadc_buf->chan);
	dma_free_coherent(indio_dev->dev.parent,
		PAGE_ALIGN(AXIADC_MAX_DMA_SIZE), axiadc_buf->buf_virt,
		axiadc_buf->buf_phys);
	iio_buffer_put(&axiadc_buf->buffer);
}
EXPORT_SYMBOL_GPL(axiadc_unconfigure_ring);
