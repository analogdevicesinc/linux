#include <linux/err.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/driver.h>
#include <linux/iio/consumer.h>
#include <linux/iio/hw_consumer.h>
#include <linux/iio/buffer_impl.h>

struct iio_hw_consumer {
	struct list_head buffers;
	struct iio_channel *channels;
};

struct hw_consumer_buffer {
	struct list_head head;
	struct iio_dev *indio_dev;
	struct iio_buffer buffer;
};

static struct hw_consumer_buffer *iio_buffer_to_hw_consumer_buffer(
	struct iio_buffer *buffer)
{
	return container_of(buffer, struct hw_consumer_buffer, buffer);
}

static void iio_hw_buf_release(struct iio_buffer *buffer)
{
	struct hw_consumer_buffer *hw_buf =
		iio_buffer_to_hw_consumer_buffer(buffer);
	iio_buffer_free_scanmask(buffer);
	kfree(hw_buf);
}

static const struct iio_buffer_access_funcs iio_hw_buf_access = {
	.release = &iio_hw_buf_release,
	.modes = INDIO_BUFFER_HARDWARE,
};

static struct hw_consumer_buffer *iio_hw_consumer_get_buffer(
	struct iio_hw_consumer *hwc, struct iio_dev *indio_dev)
{
	struct hw_consumer_buffer *buf;
	int ret;

	list_for_each_entry(buf, &hwc->buffers, head) {
		if (buf->indio_dev == indio_dev)
			return buf;
	}

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return NULL;

	buf->buffer.access = &iio_hw_buf_access;
	buf->indio_dev = indio_dev;

	iio_buffer_init(&buf->buffer);

	ret = iio_buffer_alloc_scanmask(&buf->buffer, indio_dev);
	if (ret)
		goto err_free_buf;

	list_add_tail(&buf->head, &hwc->buffers);

	return buf;

err_free_buf:
	kfree(buf);
	return NULL;
}

struct iio_hw_consumer *iio_hw_consumer_alloc(struct device *dev)
{
	struct hw_consumer_buffer *buf;
	struct iio_hw_consumer *hwc;
	struct iio_channel *chan;
	int ret;

	hwc = kzalloc(sizeof(*hwc), GFP_KERNEL);
	if (!hwc)
		return ERR_PTR(-ENOMEM);

	INIT_LIST_HEAD(&hwc->buffers);

	hwc->channels = iio_channel_get_all(dev);
	if (IS_ERR(hwc->channels)) {
		ret = PTR_ERR(hwc->channels);
		goto err_free_hwc;
	}

	chan = &hwc->channels[0];
	while (chan->indio_dev) {
		buf = iio_hw_consumer_get_buffer(hwc, chan->indio_dev);
		if (buf == NULL) {
			ret = -ENOMEM;
			goto err_put_buffers;
		}
		iio_buffer_channel_enable(&buf->buffer, chan);
		chan++;
	}

	return hwc;

err_put_buffers:
	list_for_each_entry(buf, &hwc->buffers, head)
		iio_buffer_put(&buf->buffer);
	iio_channel_release_all(hwc->channels);
err_free_hwc:
	kfree(hwc);
	return ERR_PTR(ret);
}
EXPORT_SYMBOL_GPL(iio_hw_consumer_alloc);

void iio_hw_consumer_free(struct iio_hw_consumer *hwc)
{
	struct hw_consumer_buffer *buf;

	iio_channel_release_all(hwc->channels);
	list_for_each_entry(buf, &hwc->buffers, head)
		iio_buffer_put(&buf->buffer);
	kfree(hwc);
}
EXPORT_SYMBOL_GPL(iio_hw_consumer_free);

int iio_hw_consumer_enable(struct iio_hw_consumer *hwc)
{
	struct hw_consumer_buffer *buf;
	int ret;

	list_for_each_entry(buf, &hwc->buffers, head) {
		ret = iio_update_buffers(buf->indio_dev, &buf->buffer, NULL);
		if (ret)
			goto err_disable_buffers;
	}

	return 0;

err_disable_buffers:
	list_for_each_entry_continue_reverse(buf, &hwc->buffers, head)
		iio_update_buffers(buf->indio_dev, NULL, &buf->buffer);
	return ret;
}
EXPORT_SYMBOL_GPL(iio_hw_consumer_enable);

void iio_hw_consumer_disable(struct iio_hw_consumer *hwc)
{
	struct hw_consumer_buffer *buf;

	list_for_each_entry(buf, &hwc->buffers, head)
		iio_update_buffers(buf->indio_dev, NULL, &buf->buffer);
}
EXPORT_SYMBOL_GPL(iio_hw_consumer_disable);
