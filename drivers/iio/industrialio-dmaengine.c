/*
 * Copyright 2014 Analog Devices Inc.
 *
 * Licensed under the GPL-2 or later.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/dmaengine.h>
#include <linux/dma-mapping.h>
#include <linux/spinlock.h>
#include <linux/err.h>

#include <linux/iio/iio.h>
#include <linux/iio/buffer.h>
#include <linux/iio/dma-buffer.h>
#include <linux/iio/dmaengine.h>

struct dmaengine_buffer {
	struct dma_chan *chan;
	struct list_head active;
	struct iio_dma_buffer_queue queue;
	unsigned int align;
};

static struct dmaengine_buffer *iio_buffer_to_dmaengine_buffer(
		struct iio_buffer *buffer)
{
	return container_of(buffer, struct dmaengine_buffer, queue.buffer);
}

static void dmaengine_buffer_block_done(void *data)
{
	struct iio_dma_buffer_block *block = data;
	unsigned long flags;

	spin_lock_irqsave(&block->queue->list_lock, flags);
	list_del(&block->head);
	spin_unlock_irqrestore(&block->queue->list_lock, flags);
	iio_dma_buffer_block_done(block);
}

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_block *block,
	int direction)
{
	struct dmaengine_buffer *dmaengine_buffer;
	struct dma_async_tx_descriptor *desc;
	unsigned int max_size;
	dma_cookie_t cookie;

	dmaengine_buffer = iio_buffer_to_dmaengine_buffer(&block->queue->buffer);

	max_size = dma_get_max_seg_size(dmaengine_buffer->chan->device->dev);
	if (block->block.bytes_used > max_size)
		block->block.bytes_used = max_size;

	block->block.bytes_used = rounddown(block->block.bytes_used,
			dmaengine_buffer->align);
	if (block->block.bytes_used == 0) {
		iio_dma_buffer_block_done(block);
		return 0;
	}

	if (block->block.flags & IIO_BUFFER_BLOCK_FLAG_CYCLIC) {
		desc = dmaengine_prep_dma_cyclic(dmaengine_buffer->chan,
			block->phys_addr, block->block.bytes_used,
			block->block.bytes_used, direction, 0);
		if (!desc)
			return -ENOMEM;
	} else {
		desc = dmaengine_prep_slave_single(dmaengine_buffer->chan,
			block->phys_addr, block->block.bytes_used, direction,
			DMA_PREP_INTERRUPT);
		if (!desc)
			return -ENOMEM;

		desc->callback = dmaengine_buffer_block_done;
		desc->callback_param = block;
	}

	spin_lock_irq(&dmaengine_buffer->queue.list_lock);
	list_add_tail(&block->head, &dmaengine_buffer->active);
	spin_unlock_irq(&dmaengine_buffer->queue.list_lock);

	cookie = dmaengine_submit(desc);
	if (cookie < 0)
		return cookie;

	dma_async_issue_pending(dmaengine_buffer->chan);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dmaengine_buffer_submit_block);

static int dmaengine_buffer_disable(struct iio_buffer *buf,
	struct iio_dev *indio_dev)
{
	struct dmaengine_buffer *dmaengine_buffer;
	struct iio_dma_buffer_block *block, *_block;
	LIST_HEAD(block_list);

	dmaengine_buffer = iio_buffer_to_dmaengine_buffer(buf);

	dmaengine_terminate_all(dmaengine_buffer->chan);

	spin_lock_irq(&dmaengine_buffer->queue.list_lock);
	list_splice_tail_init(&dmaengine_buffer->active, &block_list);
	spin_unlock_irq(&dmaengine_buffer->queue.list_lock);

	list_for_each_entry_safe(block, _block, &block_list, head) {
		list_del(&block->head);
		iio_dma_buffer_block_done(block);
	}

	return iio_dma_buffer_disable(buf, indio_dev);
}

static void dmaengine_buffer_release(struct iio_buffer *buf)
{
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(buf);

	kfree(dmaengine_buffer);
}

static const struct iio_buffer_access_funcs dmaengine_buffer_ops = {
	.read = iio_dma_buffer_read,
	.write = iio_dma_buffer_write,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.set_length = iio_dma_buffer_set_length,
	.enable = iio_dma_buffer_enable,
	.disable = dmaengine_buffer_disable,
	.data_available = iio_dma_buffer_data_available,
	.space_available = iio_dma_buffer_space_available,
	.release = dmaengine_buffer_release,

	.alloc_blocks = iio_dma_buffer_alloc_blocks,
	.free_blocks = iio_dma_buffer_free_blocks,
	.query_block = iio_dma_buffer_query_block,
	.enqueue_block = iio_dma_buffer_enqueue_block,
	.dequeue_block = iio_dma_buffer_dequeue_block,
	.mmap = iio_dma_buffer_mmap,

	.modes = INDIO_BUFFER_HARDWARE,
};

struct iio_buffer *iio_dmaengine_buffer_alloc(struct device *dev,
	const char *channel, const struct iio_dma_buffer_ops *ops, void *data)
{
	struct dmaengine_buffer *dmaengine_buffer;
	struct dma_slave_caps caps;
	unsigned int width, src_width, dest_width;
	int ret;

	dmaengine_buffer = kzalloc(sizeof(*dmaengine_buffer), GFP_KERNEL);
	if (!dmaengine_buffer)
		return ERR_PTR(-ENOMEM);

	dmaengine_buffer->chan = dma_request_slave_channel(dev, channel);
	if (!dmaengine_buffer->chan) {
		ret = -EPROBE_DEFER;
		goto err_free;
	}

	ret = dma_get_slave_caps(dmaengine_buffer->chan, &caps);
	if (ret == 0) {
		/* Needs to be aligned to the maximum of the minimums */
		if (caps.src_addr_widths)
			src_width = __ffs(caps.src_addr_widths);
		else
			src_width = 1;
		if (caps.dst_addr_widths)
			dest_width = __ffs(caps.dst_addr_widths);
		else
			dest_width = 1;
		width = max(src_width, dest_width);
	} else {
		width = 1;
	}

	if (!width) { /* FIXME */
		pr_warn("%s:%d width %d (DMA width >= 256-bits ?)\n",
			__func__,__LINE__, width);
		width = 32;
	}

	dmaengine_buffer->align = width;

	INIT_LIST_HEAD(&dmaengine_buffer->active);
	iio_dmabuf_init(&dmaengine_buffer->queue, dev, ops, data);

	dmaengine_buffer->queue.buffer.access = &dmaengine_buffer_ops;

	return &dmaengine_buffer->queue.buffer;

err_free:
	kfree(dmaengine_buffer);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL(iio_dmaengine_buffer_alloc);

void iio_dmaengine_buffer_free(struct iio_buffer *buffer)
{
	struct dmaengine_buffer *dmaengine_buffer =
		iio_buffer_to_dmaengine_buffer(buffer);

	iio_dmabuf_exit(&dmaengine_buffer->queue);
	dma_release_channel(dmaengine_buffer->chan);

	iio_buffer_put(buffer);
}
EXPORT_SYMBOL_GPL(iio_dmaengine_buffer_free);
