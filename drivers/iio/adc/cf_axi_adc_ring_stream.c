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
#include "cf_axi_adc.h"

static DEFINE_SPINLOCK(block_list_lock);

static void axiadc_hw_block_done(void *data)
{
	struct iio_dma_buffer_block *block = data;
	unsigned long flags;

	spin_lock_irqsave(&block_list_lock, flags);
	list_del(&block->head);
	spin_unlock_irqrestore(&block_list_lock, flags);
	iio_dma_buffer_block_done(block);
}

static int axiadc_hw_submit_block(void *data, struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = data;
	struct axiadc_state *st = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	unsigned int count;
	dma_cookie_t cookie;

	count = round_down(block->block.size, st->dma_align);
	if (count > st->max_count)
		count = st->max_count;
	block->block.bytes_used = count;

	spin_lock_irq(&block_list_lock);
	list_add_tail(&block->head, &st->block_list);
	spin_unlock_irq(&block_list_lock);

	desc = dmaengine_prep_slave_single(st->rx_chan, block->phys_addr, count,
					   DMA_FROM_DEVICE, DMA_PREP_INTERRUPT);
	if (!desc)
		return -ENOMEM;

	desc->callback = axiadc_hw_block_done;
	desc->callback_param = block;

	cookie = dmaengine_submit(desc);
	if (cookie < 0)
		return cookie;

	dma_async_issue_pending(st->rx_chan);

	axiadc_write(st, ADI_REG_STATUS, ~0);
	axiadc_write(st, ADI_REG_DMA_STATUS, ~0);

	if (!st->has_fifo_interface) {
		axiadc_write(st, ADI_REG_DMA_CNTRL, 0);
		axiadc_write(st, ADI_REG_DMA_COUNT, count);
		axiadc_write(st, ADI_REG_DMA_CNTRL, ADI_DMA_START | 2);
	}

	return 0;
}

static const struct iio_dma_buffer_ops axiadc_dma_buffer_ops = {
	.submit_block = axiadc_hw_submit_block,
};

static int axiadc_hw_ring_predisable(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct iio_dma_buffer_block *block, *_block;
	LIST_HEAD(block_list);

	dmaengine_terminate_all(st->rx_chan);

	spin_lock_irq(&block_list_lock);
	list_splice_tail_init(&st->block_list, &block_list);
	spin_unlock_irq(&block_list_lock);

	list_for_each_entry_safe(block, _block, &block_list, head) {
		list_del(&block->head);
		iio_dma_buffer_block_done(block);
	}

	return 0;
}

static const struct iio_buffer_setup_ops axiadc_ring_setup_ops = {
	.predisable = &axiadc_hw_ring_predisable,
};

int axiadc_configure_ring_stream(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	indio_dev->buffer = iio_dmabuf_allocate(indio_dev->dev.parent,
			&axiadc_dma_buffer_ops, indio_dev);
	if (indio_dev->buffer == NULL)
		return -ENOMEM;

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &axiadc_ring_setup_ops;

	INIT_LIST_HEAD(&st->block_list);

	return 0;
}

void axiadc_unconfigure_ring_stream(struct iio_dev *indio_dev)
{
	iio_buffer_put(indio_dev->buffer);
}

