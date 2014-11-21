#ifndef __IIO_DMAENGINE_H__
#define __IIO_DMAENGINE_H__

struct dma_chan;
struct iio_dma_buffer_block;
struct iio_dma_buffer_ops;

int iio_dmaengine_buffer_submit_block(struct iio_dma_buffer_block *block, int direction);

struct iio_buffer *iio_dmaengine_buffer_alloc(struct device *dev,
	const char *channel, const struct iio_dma_buffer_ops *ops, void *data);
void iio_dmaengine_buffer_free(struct iio_buffer *buffer);

#endif
