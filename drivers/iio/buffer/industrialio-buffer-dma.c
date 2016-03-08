/*
 * Copyright 2013 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/iio/buffer.h>
#include <linux/iio/dma-buffer.h>
#include <linux/dma-mapping.h>
#include <linux/sizes.h>

static void iio_buffer_block_release(struct kref *kref)
{
	struct iio_dma_buffer_block *block = container_of(kref,
		struct iio_dma_buffer_block, kref);

	BUG_ON(block->state != IIO_BLOCK_STATE_DEAD);

	dma_free_coherent(block->queue->dev, PAGE_ALIGN(block->block.size),
					block->vaddr, block->phys_addr);

	iio_buffer_put(&block->queue->buffer);
	kfree(block);
}

static void iio_buffer_block_get(struct iio_dma_buffer_block *block)
{
	kref_get(&block->kref);
}

static void iio_buffer_block_put(struct iio_dma_buffer_block *block)
{
	kref_put(&block->kref, iio_buffer_block_release);
}

/*
 * dma_free_coherent can sleep, hence we need to take some special care to be
 * able to drop a reference from an atomic context.
 */
static LIST_HEAD(iio_dmabuf_dead_blocks);
static DEFINE_SPINLOCK(iio_dmabuf_dead_blocks_lock);

static void iio_dmabuf_cleanup_worker(struct work_struct *work)
{
	struct iio_dma_buffer_block *block, *_block;
	LIST_HEAD(block_list);

	spin_lock_irq(&iio_dmabuf_dead_blocks_lock);
	list_splice_tail_init(&iio_dmabuf_dead_blocks, &block_list);
	spin_unlock_irq(&iio_dmabuf_dead_blocks_lock);

	list_for_each_entry_safe(block, _block, &block_list, head)
		iio_buffer_block_release(&block->kref);
}
static DECLARE_WORK(iio_dmabuf_cleanup_work, iio_dmabuf_cleanup_worker);

static void iio_buffer_block_release_atomic(struct kref *kref)
{
	struct iio_dma_buffer_block *block;
	unsigned long flags;

	block = container_of(kref, struct iio_dma_buffer_block, kref);

	spin_lock_irqsave(&iio_dmabuf_dead_blocks_lock, flags);
	list_add_tail(&block->head, &iio_dmabuf_dead_blocks);
	spin_unlock_irqrestore(&iio_dmabuf_dead_blocks_lock, flags);

	schedule_work(&iio_dmabuf_cleanup_work);
}

static void iio_buffer_block_put_atomic(struct iio_dma_buffer_block *block)
{
	kref_put(&block->kref, iio_buffer_block_release_atomic);
}

static struct iio_dma_buffer_queue *iio_buffer_to_queue(struct iio_buffer *buf)
{
	return container_of(buf, struct iio_dma_buffer_queue, buffer);
}

static struct iio_dma_buffer_block *iio_dma_buffer_alloc_block(
	struct iio_dma_buffer_queue *queue, size_t size)
{
	struct iio_dma_buffer_block *block;

	block = kzalloc(sizeof(*block), GFP_KERNEL);
	if (!block)
		return NULL;

	block->vaddr = dma_alloc_coherent(queue->dev, PAGE_ALIGN(size),
		&block->phys_addr, GFP_KERNEL);
	if (!block->vaddr) {
		kfree(block);
		return NULL;
	}
	block->block.size = size;
	block->state = IIO_BLOCK_STATE_DEQUEUED;
	block->queue = queue;
	INIT_LIST_HEAD(&block->head);
	kref_init(&block->kref);

	iio_buffer_get(&queue->buffer);

	return block;
}

int iio_dma_buffer_alloc_blocks(struct iio_buffer *buffer,
	struct iio_buffer_block_alloc_req *req)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block **blocks;
	unsigned int num_blocks;
	unsigned int i;
	int ret = 0;

	mutex_lock(&queue->lock);

	if (queue->fileio.block) {
		ret = -EBUSY;
		goto err_unlock;
	}

	/* 64 blocks ought to be enough for anybody ;) */
	if (req->count > 64 - queue->num_blocks)
		req->count = 64 - queue->num_blocks;
	if (req->size > SZ_16M)
		req->size = SZ_16M;

	req->id = queue->num_blocks;

	if (req->count == 0 || req->size == 0) {
		ret = 0;
		goto err_unlock;
	}

	num_blocks = req->count + queue->num_blocks;

	blocks = krealloc(queue->blocks, sizeof(*blocks) * num_blocks,
			GFP_KERNEL);
	if (!blocks) {
		ret = -ENOMEM;
		goto err_unlock;
	}

	for (i = queue->num_blocks; i < num_blocks; i++) {
		blocks[i] = iio_dma_buffer_alloc_block(queue, req->size);
		if (!blocks[i])
			break;
		blocks[i]->block.id = i;
		blocks[i]->block.data.offset = queue->max_offset;
		queue->max_offset += PAGE_ALIGN(req->size);
	}

	req->count = i - queue->num_blocks;
	queue->num_blocks = i;
	queue->blocks = blocks;

err_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_alloc_blocks);

int iio_dma_buffer_free_blocks(struct iio_buffer *buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	unsigned int i;

	mutex_lock(&queue->lock);

	if (!queue->num_blocks)
		goto out_unlock;

	spin_lock_irq(&queue->list_lock);
	INIT_LIST_HEAD(&queue->incoming);
	INIT_LIST_HEAD(&queue->outgoing);

	for (i = 0; i < queue->num_blocks; i++)
		queue->blocks[i]->state = IIO_BLOCK_STATE_DEAD;
	spin_unlock_irq(&queue->list_lock);

	for (i = 0; i < queue->num_blocks; i++)
		iio_buffer_block_put(queue->blocks[i]);

	kfree(queue->blocks);
	queue->blocks = NULL;
	queue->num_blocks = 0;
	queue->max_offset = 0;

out_unlock:

	mutex_unlock(&queue->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_free_blocks);

void iio_dma_buffer_block_done(struct iio_dma_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = block->queue;
	unsigned long flags;

	spin_lock_irqsave(&queue->list_lock, flags);
	/*
	 * The buffer has already been freed by the application, just drop the
	 * reference
	 */
	if (block->state != IIO_BLOCK_STATE_DEAD) {
		block->state = IIO_BLOCK_STATE_DONE;
		list_add_tail(&block->head, &queue->outgoing);
	}
	spin_unlock_irqrestore(&queue->list_lock, flags);

	iio_buffer_block_put_atomic(block);
	wake_up_interruptible_poll(&queue->buffer.pollq, POLLIN | POLLRDNORM);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_block_done);

static int iio_dma_buffer_fileio_alloc(struct iio_dma_buffer_queue *queue,
	struct iio_dev *indio_dev)
{
	size_t size = queue->buffer.bytes_per_datum * queue->buffer.length;
	struct iio_dma_buffer_block *block;

	block = iio_dma_buffer_alloc_block(queue, size);
	if (!block)
		return -ENOMEM;

	queue->fileio.block = block;
	queue->fileio.pos = 0;

	if (indio_dev->direction == IIO_DEVICE_DIRECTION_IN)
		list_add_tail(&block->head, &queue->incoming);

	return 0;
}

static void iio_dma_buffer_fileio_free(struct iio_dma_buffer_queue *queue)
{
	spin_lock_irq(&queue->list_lock);
	queue->fileio.block->state = IIO_BLOCK_STATE_DEAD;
	INIT_LIST_HEAD(&queue->incoming);
	INIT_LIST_HEAD(&queue->outgoing);
	spin_unlock_irq(&queue->list_lock);
	iio_buffer_block_put(queue->fileio.block);
	queue->fileio.block = NULL;
}

static void iio_dma_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	/*
	 * If the hardware has already been removed we put the block into
	 * limbo. It will neither be on the incoming nor outgoing list, nor will
	 * it ever complete. It will just wait to be freed eventually.
	 */
	if (queue->ops) {
		block->state = IIO_BLOCK_STATE_ACTIVE;
		iio_buffer_block_get(block);
		queue->ops->submit_block(queue->driver_data, block);
	}
}

int iio_dma_buffer_enable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *block, *_block;

	mutex_lock(&queue->lock);
	queue->active = true;

	/**
	 * If no buffer blocks are allocated when we start streaming go into
	 * fileio mode.
	 */
	if (!queue->num_blocks)
		iio_dma_buffer_fileio_alloc(queue, indio_dev);

	list_for_each_entry_safe(block, _block, &queue->incoming, head) {
		list_del(&block->head);
		iio_dma_buffer_submit_block(queue, block);
	}

	mutex_unlock(&queue->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_enable);

int iio_dma_buffer_disable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);

	mutex_lock(&queue->lock);

	if (queue->fileio.block)
		iio_dma_buffer_fileio_free(queue);

	queue->active = false;
	mutex_unlock(&queue->lock);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_disable);

static void iio_dma_buffer_enqueue(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	if (queue->active) {
		iio_dma_buffer_submit_block(queue, block);
	} else {
		block->state = IIO_BLOCK_STATE_QUEUED;
		list_add_tail(&block->head, &queue->incoming);
	}
}

static void iio_dma_buffer_dequeue(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block **block)
{
	spin_lock_irq(&queue->list_lock);
	if (list_empty(&queue->outgoing)) {
		*block = NULL;
	} else {
		*block = list_first_entry(&queue->outgoing,
				struct iio_dma_buffer_block, head);
		list_del(&(*block)->head);
		(*block)->state = IIO_BLOCK_STATE_DEQUEUED;
	}
	spin_unlock_irq(&queue->list_lock);
}

int iio_dma_buffer_query_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	int ret = 0;

	mutex_lock(&queue->lock);

	if (block->id >= queue->num_blocks) {
		ret = -EINVAL;
		goto out_unlock;
	}

	*block = queue->blocks[block->id]->block;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_query_block);

int iio_dma_buffer_enqueue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *dma_block;
	int ret = 0;

	mutex_lock(&queue->lock);

	if (block->id >= queue->num_blocks) {
		ret = -EINVAL;
		goto out_unlock;
	}

	dma_block = queue->blocks[block->id];
	dma_block->block.bytes_used = block->bytes_used;
	dma_block->block.flags = block->flags;

	switch (dma_block->state) {
	case IIO_BLOCK_STATE_DONE:
		list_del_init(&dma_block->head);
		break;
	case IIO_BLOCK_STATE_QUEUED:
		/* Nothing to do */
		goto out_unlock;
	case IIO_BLOCK_STATE_DEQUEUED:
		break;
	default:
		ret = -EBUSY;
		goto out_unlock;
	}

	iio_dma_buffer_enqueue(queue, dma_block);

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_enqueue_block);

int iio_dma_buffer_dequeue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *dma_block = NULL;
	int ret = 0;

	mutex_lock(&queue->lock);

	iio_dma_buffer_dequeue(queue, &dma_block);

	if (!dma_block) {
		ret = -EAGAIN;
		goto out_unlock;
	}

	*block = dma_block->block;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_dequeue_block);

int iio_dma_buffer_read(struct iio_buffer *buf, size_t n,
	char __user *user_buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	struct iio_dma_buffer_block *block;
	int ret;

	if (n < buf->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&queue->lock);

	if (!queue->fileio.block) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (queue->fileio.block->state != IIO_BLOCK_STATE_DEQUEUED) {
		iio_dma_buffer_dequeue(queue, &block);
		if (block == NULL) {
			ret = 0;
			goto out_unlock;
		}
		queue->fileio.pos = 0;
	} else {
		block = queue->fileio.block;
	}

	block = queue->fileio.block;

	n = rounddown(n, buf->bytes_per_datum);
	if (n > block->block.bytes_used - queue->fileio.pos)
		n = block->block.bytes_used - queue->fileio.pos;

	if (copy_to_user(user_buffer, block->vaddr + queue->fileio.pos, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	queue->fileio.pos += n;

	if (queue->fileio.pos == block->block.bytes_used)
		iio_dma_buffer_enqueue(queue, block);

	ret = n;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_read);

int iio_dma_buffer_write(struct iio_buffer *buf, size_t n,
	const char __user *user_buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	struct iio_dma_buffer_block *block;
	int ret;

	if (n < buf->bytes_per_datum)
		return -EINVAL;

	mutex_lock(&queue->lock);

	if (!queue->fileio.block) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (queue->fileio.block->state != IIO_BLOCK_STATE_DEQUEUED) {
		iio_dma_buffer_dequeue(queue, &block);
		if (block == NULL) {
			ret = 0;
			goto out_unlock;
		}
		queue->fileio.pos = 0;
	} else {
		block = queue->fileio.block;
	}

	block = queue->fileio.block;

	n = ALIGN(n, buf->bytes_per_datum);
	if (n > block->block.size - queue->fileio.pos)
		n = block->block.size - queue->fileio.pos;

	if (copy_from_user(block->vaddr + queue->fileio.pos, user_buffer, n)) {
		ret = -EFAULT;
		goto out_unlock;
	}

	queue->fileio.pos += n;

	if (queue->fileio.pos == block->block.size) {
		block->block.bytes_used = block->block.size;
		iio_dma_buffer_enqueue(queue, block);
	}

	ret = n;

out_unlock:
	mutex_unlock(&queue->lock);

	return ret;

}
EXPORT_SYMBOL_GPL(iio_dma_buffer_write);

size_t iio_dma_buffer_data_available(struct iio_buffer *buf)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	struct iio_dma_buffer_block *block;
	size_t data_available = 0;

	/*
	 * For counting the available bytes we'll use the size of the block not
	 * the number of actual bytes available in the block. Otherwise it is
	 * possible that we end up with a value that is lower than the watermark
	 * but wont increase since all blocks are in use.
	 */

	mutex_lock(&queue->lock);
	if (queue->fileio.block)
		data_available += queue->fileio.block->block.size;

	spin_lock_irq(&queue->list_lock);
	list_for_each_entry(block, &queue->outgoing, head)
		data_available += block->block.size;
	spin_unlock_irq(&queue->list_lock);
	mutex_unlock(&queue->lock);

	return data_available;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_data_available);

bool iio_dma_buffer_space_available(struct iio_buffer *buf)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buf);
	bool space_available = false;

	mutex_lock(&queue->lock);
	if (queue->fileio.block &&
		queue->fileio.block->state == IIO_BLOCK_STATE_DEQUEUED)
		space_available = true;
	spin_lock_irq(&queue->list_lock);
	space_available |= !list_empty(&queue->outgoing);
	spin_unlock_irq(&queue->list_lock);
	mutex_unlock(&queue->lock);

	return space_available;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_space_available);

static void iio_dma_buffer_mmap_open(struct vm_area_struct *area)
{
	struct iio_dma_buffer_block *block = area->vm_private_data;
	iio_buffer_block_get(block);
}

static void iio_dma_buffer_mmap_close(struct vm_area_struct *area)
{
	struct iio_dma_buffer_block *block = area->vm_private_data;
	iio_buffer_block_put(block);
}

static const struct vm_operations_struct iio_dma_buffer_vm_ops = {
	.open = iio_dma_buffer_mmap_open,
	.close = iio_dma_buffer_mmap_close,
};

int iio_dma_buffer_mmap(struct iio_buffer *buffer,
	struct vm_area_struct *vma)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);
	struct iio_dma_buffer_block *block = NULL;
	size_t vm_offset;
	unsigned int i;

	vm_offset = vma->vm_pgoff << PAGE_SHIFT;

	for (i = 0; i < queue->num_blocks; i++) {
		if (queue->blocks[i]->block.data.offset == vm_offset) {
			block = queue->blocks[i];
			break;
		}
	}

	if (block == NULL)
		return -EINVAL;

	if (PAGE_ALIGN(block->block.size) < vma->vm_end - vma->vm_start)
		return -EINVAL;

	vma->vm_pgoff = 0;

	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_ops = &iio_dma_buffer_vm_ops;
	vma->vm_private_data = block;

	vma->vm_ops->open(vma);

	return dma_mmap_coherent(queue->dev, vma, block->vaddr,
		block->phys_addr, vma->vm_end - vma->vm_start);
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_mmap);

int iio_dma_buffer_set_bytes_per_datum(struct iio_buffer *buf, size_t bpd)
{
	buf->bytes_per_datum = bpd;

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_set_bytes_per_datum);

int iio_dma_buffer_set_length(struct iio_buffer *buf, int length)
{
	/* Avoid an invalid state */
	if (length < 2)
		length = 2;
	buf->length = length;

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dma_buffer_set_length);

static void iio_dma_buffer_release(struct iio_buffer *buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);

	BUG_ON(queue->num_blocks != 0);
	BUG_ON(queue->fileio.block != NULL);

	kfree(queue);
}

static const struct iio_buffer_access_funcs dmabuf_ops = {
	.read = iio_dma_buffer_read,
	.write = iio_dma_buffer_write,
	.set_bytes_per_datum = iio_dma_buffer_set_bytes_per_datum,
	.set_length = iio_dma_buffer_set_length,
	.enable = iio_dma_buffer_enable,
	.disable = iio_dma_buffer_disable,
	.data_available = iio_dma_buffer_data_available,
	.space_available = iio_dma_buffer_space_available,
	.release = iio_dma_buffer_release,

	.alloc_blocks = iio_dma_buffer_alloc_blocks,
	.free_blocks = iio_dma_buffer_free_blocks,
	.query_block = iio_dma_buffer_query_block,
	.enqueue_block = iio_dma_buffer_enqueue_block,
	.dequeue_block = iio_dma_buffer_dequeue_block,
	.mmap = iio_dma_buffer_mmap,

	.modes = INDIO_BUFFER_HARDWARE,
};

static u64 dmamask = DMA_BIT_MASK(64);

int iio_dmabuf_init(struct iio_dma_buffer_queue *queue,
	struct device *dma_dev, const struct iio_dma_buffer_ops *ops,
	void *driver_data)
{
	iio_buffer_init(&queue->buffer);
	queue->buffer.access = &dmabuf_ops;
	queue->buffer.length = PAGE_SIZE;
	queue->dev = dma_dev;
	queue->ops = ops;
	queue->driver_data = driver_data;

	INIT_LIST_HEAD(&queue->incoming);
	INIT_LIST_HEAD(&queue->outgoing);

	mutex_init(&queue->lock);
	spin_lock_init(&queue->list_lock);

	if (!queue->dev->dma_mask)
		queue->dev->dma_mask = &dmamask;
	if (!queue->dev->coherent_dma_mask)
		queue->dev->coherent_dma_mask = DMA_BIT_MASK(64);

	return 0;
}
EXPORT_SYMBOL_GPL(iio_dmabuf_init);

void iio_dmabuf_exit(struct iio_dma_buffer_queue *queue)
{
	mutex_lock(&queue->lock);
	queue->ops = NULL;
	mutex_unlock(&queue->lock);
}
EXPORT_SYMBOL_GPL(iio_dmabuf_exit);

struct iio_buffer *iio_dmabuf_allocate(struct device *dma_dev,
	const struct iio_dma_buffer_ops *ops, void *driver_data)
{
	struct iio_dma_buffer_queue *queue;

	queue = kzalloc(sizeof(*queue), GFP_KERNEL);
	if (!queue)
		return NULL;

	iio_dmabuf_init(queue, dma_dev, ops, driver_data);

	return &queue->buffer;
}
EXPORT_SYMBOL_GPL(iio_dmabuf_allocate);

void iio_dmabuf_free(struct iio_buffer *buffer)
{
	struct iio_dma_buffer_queue *queue = iio_buffer_to_queue(buffer);

	iio_dmabuf_exit(queue);
	iio_buffer_put(buffer);
}
EXPORT_SYMBOL_GPL(iio_dmabuf_free);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("DMA buffer for the IIO framework");
MODULE_LICENSE("GPL");
