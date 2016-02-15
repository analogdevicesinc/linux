#ifndef __INDUSTRIALIO_DMABUF_H__
#define __INDUSTRIALIO_DMABUF_H__

#include <linux/kref.h>

struct iio_dma_buffer_queue {
	struct iio_buffer buffer;
	struct device *dev;
	const struct iio_dma_buffer_ops *ops;
	struct mutex lock;
	spinlock_t list_lock;
	struct list_head incoming;
	struct list_head outgoing;
	bool active;

	void *driver_data;

	unsigned int num_blocks;
	struct iio_dma_buffer_block **blocks;
	unsigned int max_offset;

	struct {
		size_t pos;
		struct iio_dma_buffer_block *block;
	} fileio;
};

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

int iio_dma_buffer_alloc_blocks(struct iio_buffer *buffer,
	struct iio_buffer_block_alloc_req *req);
int iio_dma_buffer_free_blocks(struct iio_buffer *buffer);
int iio_dma_buffer_enable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev);
int iio_dma_buffer_disable(struct iio_buffer *buffer,
	struct iio_dev *indio_dev);
int iio_dma_buffer_query_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block);
int iio_dma_buffer_enqueue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block);
int iio_dma_buffer_dequeue_block(struct iio_buffer *buffer,
	struct iio_buffer_block *block);
int iio_dma_buffer_read(struct iio_buffer *buf, size_t n,
	char __user *user_buffer);
int iio_dma_buffer_write(struct iio_buffer *buf, size_t n,
	const char __user *user_buffer);
size_t iio_dma_buffer_data_available(struct iio_buffer *buf);
bool iio_dma_buffer_space_available(struct iio_buffer *buf);
int iio_dma_buffer_mmap(struct iio_buffer *buffer,
	struct vm_area_struct *vma);
int iio_dma_buffer_set_bytes_per_datum(struct iio_buffer *buf, size_t bpd);
int iio_dma_buffer_get_length(struct iio_buffer *buf);
int iio_dma_buffer_set_length(struct iio_buffer *buf, int length);
int iio_dmabuf_init(struct iio_dma_buffer_queue *queue,
	struct device *dma_dev, const struct iio_dma_buffer_ops *ops,
	void *driver_data);
void iio_dmabuf_exit(struct iio_dma_buffer_queue *queue);

#endif
