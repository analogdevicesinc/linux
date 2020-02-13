/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * Copyright (c) 2020 Analog Devices Inc.
 */
#ifndef __INDUSTRIALIO_DMA_BUFFER_IMPL_H__
#define __INDUSTRIALIO_DMA_BUFFER_IMPL_H_

#include <linux/iio/buffer_impl.h>
#include <linux/list.h>
#include <linux/kref.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>

/**
 * enum iio_block_state - State of a struct iio_dma_buffer_block
 * @IIO_BLOCK_STATE_DEQUEUED: Block is not queued
 * @IIO_BLOCK_STATE_QUEUED: Block is on the incoming queue
 * @IIO_BLOCK_STATE_ACTIVE: Block is currently being processed by the DMA
 * @IIO_BLOCK_STATE_DONE: Block is on the outgoing queue
 * @IIO_BLOCK_STATE_DEAD: Block has been marked as to be freed
 */
enum iio_block_state {
	IIO_BLOCK_STATE_DEQUEUED,
	IIO_BLOCK_STATE_QUEUED,
	IIO_BLOCK_STATE_ACTIVE,
	IIO_BLOCK_STATE_DONE,
	IIO_BLOCK_STATE_DEAD,
};

/**
 * struct iio_dma_buffer_block - IIO buffer block
 * @head: List head
 * @block: IIO buffer block
 * @vaddr: Virtual address of the blocks memory
 * @phys_addr: Physical address of the blocks memory
 * @queue: Parent DMA buffer queue
 * @kref: kref used to manage the lifetime of block
 * @state: Current state of the block
 */
struct iio_dma_buffer_block {
	/* May only be accessed by the owner of the block */
	struct list_head head;
	struct iio_buffer_block block;
	/*
	 * Set during allocation, constant thereafter. May be accessed read-only
	 * by anybody holding a reference to the block.
	 */
	void *vaddr;
	dma_addr_t phys_addr;
	struct iio_dma_buffer_queue *queue;

	/* Must not be accessed outside the core. */
	struct kref kref;
	/*
	 * Must not be accessed outside the core. Access needs to hold
	 * queue->list_lock if the block is not owned by the core.
	 */
	enum iio_block_state state;
};

/**
 * struct iio_dma_buffer_queue_fileio - FileIO state for the DMA buffer
 * @blocks: Buffer blocks used for fileio
 * @active_block: Block being used in read()
 * @pos: Read offset in the active block
 * @block_size: Size of each block
 */
struct iio_dma_buffer_queue_fileio {
	struct iio_dma_buffer_block *blocks[2];
	struct iio_dma_buffer_block *active_block;
	size_t pos;
	size_t block_size;
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
 * @outgoing: List of buffers on the outgoing queue
 * @active: Whether the buffer is currently active
 * @driver_data: Driver private data
 * @poll_wakup_flags: Wake up flags to be used with the buffer wait queue
 * @num_blocks: Number of IIO buffer blocks in the buffer
 * @blocks: Group of IIO buffer blocks
 * @max_offset: Max offset on the DMA buffer until where a block can be found
 * @fileio: FileIO state
 */
struct iio_dma_buffer_queue {
	struct iio_buffer buffer;
	struct device *dev;
	const struct iio_dma_buffer_ops *ops;
	/*
	 * Protects the incoming list, active and the fields in the fileio
	 * substruct.
	 */
	struct mutex lock;
	/*
	 * Protects lists that contain blocks which can be modified in
	 * atomic context as well as blocks on those lists. This is the outgoing
	 * queue list and typically also a list of active blocks in the part
	 * that handles the DMA controller
	 */
	spinlock_t list_lock;
	struct list_head incoming;
	struct list_head outgoing;

	u8 active;

	void *driver_data;

	unsigned int poll_wakup_flags;

	unsigned int num_blocks;
	struct iio_dma_buffer_block **blocks;
	unsigned int max_offset;

	struct iio_dma_buffer_queue_fileio fileio;
};
#endif
