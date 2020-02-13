/*
 * Copyright 2013-2015 Analog Devices Inc.
 *  Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

#ifndef __INDUSTRIALIO_DMA_BUFFER_H__
#define __INDUSTRIALIO_DMA_BUFFER_H__

struct iio_dev;
struct iio_buffer;
struct iio_dma_buffer_queue;
struct device;
struct iio_dma_buffer_block;
struct iio_buffer_block;
struct iio_buffer_block_alloc_req;

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
size_t iio_dma_buffer_data_available(struct iio_buffer *buffer);
int iio_dma_buffer_set_bytes_per_datum(struct iio_buffer *buffer, size_t bpd);
int iio_dma_buffer_set_length(struct iio_buffer *buffer, unsigned int length);
int iio_dma_buffer_request_update(struct iio_buffer *buffer);

int iio_dma_buffer_init(struct iio_dma_buffer_queue *queue,
	struct device *dma_dev, const struct iio_dma_buffer_ops *ops,
	void *driver_data);
void iio_dma_buffer_exit(struct iio_dma_buffer_queue *queue);
void iio_dma_buffer_release(struct iio_dma_buffer_queue *queue);

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
int iio_dma_buffer_write(struct iio_buffer *buf, size_t n,
	const char __user *user_buffer);
bool iio_dma_buffer_space_available(struct iio_buffer *buf);
void *iio_dma_buffer_get_drvdata(const struct iio_dma_buffer_queue *queue);
#endif
