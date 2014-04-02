/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/uaccess.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "../../staging/iio/ring_hw.h"
#include "cf_axi_dds.h"

#include <linux/amba/xilinx_dma.h>

#define VDMA_MAX_HSIZE	0xFFFF
#define VDMA_MAX_VSIZE	0xFFF

struct dds_buffer {
	struct iio_buffer iio_buffer;
	unsigned int length;
	struct dma_chan *chan;

	void *buf_virt;
	dma_addr_t buf_phys;

	struct iio_dev *indio_dev;
	struct xilinx_dma_config dma_config;
};

static struct dds_buffer *iio_buffer_to_dds_buffer(struct iio_buffer *buf)
{
	return container_of(buf, struct dds_buffer, iio_buffer);
}

static int dds_buffer_enable(struct iio_buffer *buf, struct iio_dev *indio_dev)
{
	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(buf);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	unsigned int x, cnt;

	if (!st->has_fifo_interface) {
		cnt = dds_buffer->length;
		x = 1;

		do {
			if ((cnt <= VDMA_MAX_HSIZE) && ((cnt % 8) == 0))
				break;

			cnt = dds_buffer->length / ++x;

		} while (x < VDMA_MAX_VSIZE);

		if (x > VDMA_MAX_VSIZE || cnt * x != dds_buffer->length || ((cnt % 8) != 0)) {
			dev_err(indio_dev->dev.parent, "Buffer size doesn't fit VDMA\n");
			return -EINVAL;
		}

		dds_buffer->dma_config.vsize = x;
		dds_buffer->dma_config.stride = dds_buffer->dma_config.hsize = cnt;

		dmaengine_device_control(dds_buffer->chan, DMA_SLAVE_CONFIG,
			(unsigned long)&dds_buffer->dma_config);
	}

	desc = dmaengine_prep_slave_single(dds_buffer->chan,
		dds_buffer->buf_phys, dds_buffer->length, DMA_MEM_TO_DEV, 0);
	if (!desc) {
		dev_err(indio_dev->dev.parent, "Failed to prepare DMA descriptor\n");
		return -ENOMEM;
	}

	dmaengine_submit(desc);
	dma_async_issue_pending(dds_buffer->chan);

	return 0;
}

static int dds_buffer_disable(struct iio_buffer *buf, struct iio_dev *indio_dev)
{

	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(buf);

	dmaengine_terminate_all(dds_buffer->chan);

	return 0;
}

static int dds_buffer_state_set(struct iio_dev *indio_dev, bool state)
{
	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(indio_dev->buffer);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned tmp_reg;

#if 0
	tmp_reg = dds_read(st, CF_AXI_DDS_DMA_STAT);
	if (tmp_reg & (CF_AXI_DDS_DMA_STAT_OVF | CF_AXI_DDS_DMA_STAT_UNF))
		dev_warn(indio_dev->dev.parent, "VDMA Status: %s %s\n",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "overflow" : "",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "underflow" : "");
#endif
	tmp_reg = dds_read(st, ADI_REG_CNTRL_2);
	tmp_reg &= ~ADI_DATA_SEL(~0);

	if (!state) {
//		cf_axi_dds_stop(st);
		tmp_reg |= DATA_SEL_DDS;
		dds_write(st, ADI_REG_CNTRL_2, tmp_reg);
		return 0;
	}

	tmp_reg |= DATA_SEL_DMA;

	dds_write(st, ADI_REG_CNTRL_1, 0);
	st->enable = false;

	if (!st->has_fifo_interface)
		dds_write(st, ADI_REG_VDMA_FRMCNT, dds_buffer->length);

	dds_write(st, ADI_REG_CNTRL_2, tmp_reg);
	dds_write(st, ADI_REG_VDMA_STATUS, ADI_VDMA_OVF | ADI_VDMA_UNF);
	dds_write(st, ADI_REG_CNTRL_1, ADI_ENABLE);
	st->enable = true;

	return 0;
}

static int dds_buffer_write(struct iio_buffer *buf, size_t count,
	const char __user *user_buffer)
{
	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(buf);
	bool enabled = iio_buffer_enabled(dds_buffer->indio_dev);
	int ret = 0;

	if (PAGE_ALIGN(count) > AXIDDS_MAX_DMA_SIZE || count % 8)
		return -EINVAL;

	if (enabled)
		dds_buffer_disable(buf, dds_buffer->indio_dev);

	if (copy_from_user(dds_buffer->buf_virt, user_buffer, count))
		ret = -EFAULT;

	dds_buffer->length = count;

	if (enabled) {
		dds_buffer_state_set(dds_buffer->indio_dev, true);
		dds_buffer_enable(buf, dds_buffer->indio_dev);
	}

	return ret < 0 ? ret : count;
}

static int dds_buffer_get_length(struct iio_buffer *buf)
{
	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(buf);

	return dds_buffer->length;
}

static int dds_buffer_set_length(struct iio_buffer *buf, int length)
{
	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(buf);

	dds_buffer->length = length;

	return 0;
}

static int dds_buffer_get_bytes_per_datum(struct iio_buffer *buf)
{
	return buf->bytes_per_datum;
}

static IIO_BUFFER_ENABLE_ATTR;
static IIO_BUFFER_LENGTH_ATTR;

static struct attribute *dds_buffer_attributes[] = {
	&dev_attr_length.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group dds_buffer_attrs = {
	.attrs = dds_buffer_attributes,
	.name = "buffer",
};

static void dds_buffer_release(struct iio_buffer *buf)
{
	struct dds_buffer *dds_buffer = iio_buffer_to_dds_buffer(buf);
	kfree(dds_buffer);
}

static const struct iio_buffer_access_funcs dds_buffer_access_funcs = {
	.write = &dds_buffer_write,
	.get_length = &dds_buffer_get_length,
	.set_length = &dds_buffer_set_length,
	.get_bytes_per_datum = &dds_buffer_get_bytes_per_datum,
	.enable = dds_buffer_enable,
	.disable = dds_buffer_disable,
	.release = dds_buffer_release,
};

static int dds_buffer_preenable(struct iio_dev *indio_dev)
{
	return dds_buffer_state_set(indio_dev, 1);
}

static int dds_buffer_postdisable(struct iio_dev *indio_dev)
{
	return dds_buffer_state_set(indio_dev, 0);
}

static const struct iio_buffer_setup_ops dds_buffer_setup_ops = {
	.preenable = &dds_buffer_preenable,
	.postdisable = &dds_buffer_postdisable,
};

int cf_axi_dds_configure_buffer(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct device *dev = indio_dev->dev.parent;
	struct dds_buffer *dds_buffer;
	int ret;

	if (st->dp_disable)
		return 0;

	dds_buffer = kzalloc(sizeof(*dds_buffer), GFP_KERNEL);
	if (!dds_buffer)
		return -ENOMEM;

	dds_buffer->iio_buffer.attrs = &dds_buffer_attrs;
	dds_buffer->iio_buffer.access = &dds_buffer_access_funcs;
	iio_buffer_init(&dds_buffer->iio_buffer);
	dds_buffer->indio_dev = indio_dev;

	dds_buffer->chan = dma_request_slave_channel(dev, "tx");
	if (!dds_buffer->chan) {
		dev_err(dev, "failed to find DMA device\n");
		ret = -ENODEV;
		goto err_free;
	}

	dds_buffer->buf_virt = dma_alloc_coherent(indio_dev->dev.parent,
		PAGE_ALIGN(AXIDDS_MAX_DMA_SIZE), &dds_buffer->buf_phys,
		GFP_KERNEL);
	if (dds_buffer->buf_virt == NULL) {
		ret = -ENOMEM;
		goto err_release_dma;
	}

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	indio_dev->setup_ops = &dds_buffer_setup_ops;

	iio_device_attach_buffer(indio_dev, &dds_buffer->iio_buffer);

	return 0;
err_release_dma:
	dma_release_channel(dds_buffer->chan);
err_free:
	kfree(dds_buffer);

	return ret;
}

void cf_axi_dds_unconfigure_buffer(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct dds_buffer *dds_buffer;

	if (st->dp_disable)
		return;

	dds_buffer = iio_buffer_to_dds_buffer(indio_dev->buffer);

	dma_release_channel(dds_buffer->chan);

	dma_free_coherent(indio_dev->dev.parent,
		PAGE_ALIGN(AXIDDS_MAX_DMA_SIZE), dds_buffer->buf_virt,
		dds_buffer->buf_phys);
	iio_buffer_put(indio_dev->buffer);
}
