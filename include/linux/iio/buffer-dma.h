/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright 2013-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 */

#ifndef __INDUSTRIALIO_DMA_BUFFER_H__
#define __INDUSTRIALIO_DMA_BUFFER_H__

#include <linux/atomic.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/spinlock.h>
#include <linux/mutex.h>
#include <linux/iio/buffer_impl.h>

struct iio_dma_buffer_queue;
struct iio_dma_buffer_ops;
struct device;
struct dma_buf_attachment;
struct dma_fence;
struct sg_table;

/**
 * enum iio_block_state - State of a struct iio_dma_buffer_block
 * @IIO_BLOCK_STATE_QUEUED: Block is on the incoming queue
 * @IIO_BLOCK_STATE_ACTIVE: Block is currently being processed by the DMA
 * @IIO_BLOCK_STATE_DONE: Block is on the outgoing queue
 * @IIO_BLOCK_STATE_DEAD: Block has been marked as to be freed
 */
enum iio_block_state {
#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
	IIO_BLOCK_STATE_DEQUEUED,
#endif
	IIO_BLOCK_STATE_QUEUED,
	IIO_BLOCK_STATE_ACTIVE,
	IIO_BLOCK_STATE_DONE,
	IIO_BLOCK_STATE_DEAD,
};

/**
 * struct iio_dma_buffer_block - IIO buffer block
 * @head: List head
 * @size: Total size of the block in bytes
 * @bytes_used: Number of bytes that contain valid data
 * @vaddr: Virutal address of the blocks memory
 * @phys_addr: Physical address of the blocks memory
 * @queue: Parent DMA buffer queue
 * @kref: kref used to manage the lifetime of block
 * @state: Current state of the block
 * @cyclic: True if this is a cyclic buffer
 * @fileio: True if this buffer is used for fileio mode
 * @sg_table: DMA table for the transfer when transferring a DMABUF
 * @fence: DMA fence to be signaled when a DMABUF transfer is complete
 */
struct iio_dma_buffer_block {
	/* May only be accessed by the owner of the block */
	struct list_head head;
	size_t bytes_used;
#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
	struct iio_buffer_block block;
#endif

	/*
	 * Set during allocation, constant thereafter. May be accessed read-only
	 * by anybody holding a reference to the block.
	 */
	void *vaddr;
	dma_addr_t phys_addr;
	size_t size;
	struct iio_dma_buffer_queue *queue;

	/* Must not be accessed outside the core. */
	struct kref kref;
	/*
	 * Must not be accessed outside the core. Access needs to hold
	 * queue->list_lock if the block is not owned by the core.
	 */
	enum iio_block_state state;

	bool cyclic;
	bool fileio;

	struct sg_table *sg_table;
	struct dma_fence *fence;
};

/**
 * struct iio_dma_buffer_queue_fileio - FileIO state for the DMA buffer
 * @blocks: Buffer blocks used for fileio
 * @active_block: Block being used in read()
 * @pos: Read offset in the active block
 * @block_size: Size of each block
 * @next_dequeue: index of next block that will be dequeued
 * @enabled: Whether the buffer is operating in fileio mode
 */
struct iio_dma_buffer_queue_fileio {
	struct iio_dma_buffer_block *blocks[2];
	struct iio_dma_buffer_block *active_block;
	size_t pos;
	size_t block_size;

	unsigned int next_dequeue;
	bool enabled;
};

/**
 * struct iio_dma_buffer_queue - DMA buffer base structure
 * @buffer: IIO buffer base structure
 * @dev: Parent device
 * @ops: DMA buffer callbacks
 * @lock: Protects the incoming list, active and the fields in the fileio
 *   substruct
 * @list_lock: Protects lists that contain blocks which can be modified in
 *   atomic context as well as blocks on those lists. This is the outgoing queue
 *   list and typically also a list of active blocks in the part that handles
 *   the DMA controller
 * @incoming: List of buffers on the incoming queue
 * @active: Whether the buffer is currently active
 * @num_dmabufs: Total number of DMABUFs attached to this queue
 * @fileio: FileIO state
 */
struct iio_dma_buffer_queue {
	struct iio_buffer buffer;
	struct device *dev;
	const struct iio_dma_buffer_ops *ops;

	struct mutex lock;
	spinlock_t list_lock;
	struct list_head incoming;
#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
	struct list_head outgoing;
#endif

	bool active;
	atomic_t num_dmabufs;

	void *driver_data;

#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
	unsigned int num_blocks;
	struct iio_dma_buffer_block **blocks;
	unsigned int max_offset;
#endif

	struct iio_dma_buffer_queue_fileio fileio;
};

/**
 * struct iio_dma_buffer_ops - DMA buffer callback operations
 * @submit: Called when a block is submitted to the DMA controller
 * @abort: Should abort all pending transfers
 */
struct iio_dma_buffer_ops {
	int (*submit)(struct iio_dma_buffer_queue *queue,
		struct iio_dma_buffer_block *block);
	void (*abort)(struct iio_dma_buffer_queue *queue);
};

void iio_dma_buffer_block_done(struct iio_dma_buffer_block *block);
void iio_dma_buffer_block_list_abort(struct iio_dma_buffer_queue *queue,
	struct list_head *list);

int iio_dma_buffer_enable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev);
int iio_dma_buffer_disable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev);
int iio_dma_buffer_read(struct iio_buffer *buffer, size_t n,
	char __user *user_buffer);
int iio_dma_buffer_write(struct iio_buffer *buffer, size_t n,
			 const char __user *user_buffer);
size_t iio_dma_buffer_usage(struct iio_buffer *buffer);
int iio_dma_buffer_set_bytes_per_datum(struct iio_buffer *buffer, size_t bpd);
int iio_dma_buffer_set_length(struct iio_buffer *buffer, unsigned int length);
int iio_dma_buffer_request_update(struct iio_buffer *buffer);

int iio_dma_buffer_init(struct iio_dma_buffer_queue *queue,
	struct device *dma_dev, const struct iio_dma_buffer_ops *ops,
	void *driver_data);
void iio_dma_buffer_exit(struct iio_dma_buffer_queue *queue);
void iio_dma_buffer_release(struct iio_dma_buffer_queue *queue);
#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
int iio_dma_buffer_alloc_blocks(struct iio_buffer *buffer,
	struct iio_buffer_block_alloc_req *req);
int iio_dma_buffer_free_blocks(struct iio_buffer *buffer);
int iio_dma_buffer_query_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block);
int iio_dma_buffer_enqueue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block);
int iio_dma_buffer_dequeue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block);
int iio_dma_buffer_mmap(struct iio_buffer *buffer,
	struct vm_area_struct *vma);
#else

struct iio_dma_buffer_block *
iio_dma_buffer_attach_dmabuf(struct iio_buffer *buffer,
			     struct dma_buf_attachment *attach);
void iio_dma_buffer_detach_dmabuf(struct iio_buffer *buffer,
				  struct iio_dma_buffer_block *block);
int iio_dma_buffer_enqueue_dmabuf(struct iio_buffer *buffer,
				  struct iio_dma_buffer_block *block,
				  struct dma_fence *fence,
				  struct sg_table *sgt,
				  size_t size, bool cyclic);
void iio_dma_buffer_lock_queue(struct iio_buffer *buffer);
void iio_dma_buffer_unlock_queue(struct iio_buffer *buffer);
#endif
#endif
