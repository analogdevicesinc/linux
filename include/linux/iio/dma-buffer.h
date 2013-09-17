#ifndef __INDUSTRIALIO_DMABUF_H__
#define __INDUSTRIALIO_DMABUF_H__

#include <linux/kref.h>

struct iio_dma_buffer_queue;

enum iio_block_state {
	IIO_BLOCK_STATE_DEQUEUED,
	IIO_BLOCK_STATE_QUEUED,
	IIO_BLOCK_STATE_ACTIVE,
	IIO_BLOCK_STATE_DONE,
	IIO_BLOCK_STATE_DEAD,
};

struct iio_dma_buffer_block {
	struct iio_buffer_block block;
	void *vaddr;
	dma_addr_t phys_addr;
	struct list_head head;
	struct iio_dma_buffer_queue *queue;
	enum iio_block_state state;
	struct kref kref;
};

struct iio_dma_buffer_ops {
	int (*submit_block)(void *data, struct iio_dma_buffer_block *block);
};

void iio_dma_buffer_block_done(struct iio_dma_buffer_block *block);
struct iio_buffer *iio_dmabuf_allocate(struct device *dma_dev,
	const struct iio_dma_buffer_ops *ops, void *driver_data);
void iio_dmabuf_free(struct iio_buffer *r);

#endif
