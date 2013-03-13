/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/spi/spi.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "../../staging/iio/ring_hw.h"
#include "cf_axi_dds.h"

#define VDMA_MAX_HSIZE	0xFFFF
#define VDMA_MAX_VSIZE	0xFFF


static int __cf_axi_dds_hw_buffer_state_set(struct iio_dev *indio_dev, bool state);

static int cf_axi_dds_write_buffer(struct iio_buffer *r,
				      size_t count, const char __user *buf)
{
	struct iio_hw_buffer *hw_buffer = iio_to_hw_buf(r);
	struct iio_dev *indio_dev = hw_buffer->private;
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	int ret = 0;

	if (PAGE_ALIGN(count) > AXIDDS_MAX_DMA_SIZE || count % 8) {
		ret = -EINVAL;
		goto error_ret;
	}

	mutex_lock(&indio_dev->mlock);

	if (copy_from_user (st->buf_virt, buf, count))
		ret = -EFAULT;

	st->txcount = count;
	st->buffer_length = count / 4;

	if (iio_buffer_enabled(indio_dev)) {
		dmaengine_terminate_all(st->tx_chan);
		__cf_axi_dds_hw_buffer_state_set(indio_dev, 1);
	}

	mutex_unlock(&indio_dev->mlock);

error_ret:

	return ret < 0 ? ret : count;
}

static int cf_axi_dds_buffer_get_length(struct iio_buffer *r)
{
	struct iio_hw_buffer *hw_buffer = iio_to_hw_buf(r);
	struct iio_dev *indio_dev = hw_buffer->private;
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	return st->buffer_length;
}

static int cf_axi_dds_buffer_set_length(struct iio_buffer *r, int length)
{
	struct iio_hw_buffer *hw_buffer = iio_to_hw_buf(r);
	struct cf_axi_dds_state *st = iio_priv(hw_buffer->private);

	st->buffer_length = length;

	return 0;
}

static int cf_axi_dds_buffer_get_bytes_per_datum(struct iio_buffer *r)
{
	return r->bytes_per_datum;
}

static IIO_BUFFER_ENABLE_ATTR;
static IIO_BUFFER_LENGTH_ATTR;

static struct attribute *cf_axi_dds_buffer_attributes[] = {
	&dev_attr_length.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group cf_axi_dds_buffer_attr = {
	.attrs = cf_axi_dds_buffer_attributes,
	.name = "buffer",
};

static struct iio_buffer *cf_axi_dds_rb_allocate(struct iio_dev *indio_dev)
{
	struct iio_buffer *buf;
	struct iio_hw_buffer *ring;

	ring = kzalloc(sizeof *ring, GFP_KERNEL);
	if (!ring)
		return NULL;

	ring->private = indio_dev;
	buf = &ring->buf;
	buf->attrs = &cf_axi_dds_buffer_attr;
	iio_buffer_init(buf);

	return buf;
}

static inline void cf_axi_dds_rb_free(struct iio_buffer *r)
{
	kfree(iio_to_hw_buf(r));
}

static const struct iio_buffer_access_funcs cf_axi_dds_buffer_access_funcs = {
	.write = &cf_axi_dds_write_buffer,
	.get_length = &cf_axi_dds_buffer_get_length,
	.set_length = &cf_axi_dds_buffer_set_length,
	.get_bytes_per_datum = &cf_axi_dds_buffer_get_bytes_per_datum,
};

static int __cf_axi_dds_hw_buffer_state_set(struct iio_dev *indio_dev, bool state)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct dma_async_tx_descriptor *desc;
	unsigned tmp_reg, x, cnt;

#if 0
	tmp_reg = dds_read(st, CF_AXI_DDS_DMA_STAT);
	if (tmp_reg & (CF_AXI_DDS_DMA_STAT_OVF | CF_AXI_DDS_DMA_STAT_UNF))
		dev_warn(indio_dev->dev.parent, "VDMA Status: %s %s\n",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "overflow" : "",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "underflow" : "");
#endif
	tmp_reg = dds_read(st, CF_AXI_DDS_CTRL);

	if (!state) {
		cf_axi_dds_stop(st);
		tmp_reg = CF_AXI_DDS_CTRL_DATA_EN | CF_AXI_DDS_CTRL_DDS_CLK_EN_V2;
		dds_write(st, CF_AXI_DDS_CTRL, tmp_reg);
		dmaengine_terminate_all(st->tx_chan);
		return 0;
	}

	if (st->txcount == 0) {
		return -EINVAL;
	}

	tmp_reg = CF_AXI_DDS_CTRL_DDS_SEL |
		CF_AXI_DDS_CTRL_DATA_EN |
		CF_AXI_DDS_CTRL_DDS_CLK_EN_V2 |
		st->ddr_dds_interp_en;

	cnt = st->txcount;
	x = 1;

	do {
		if (cnt > VDMA_MAX_HSIZE)
			cnt /= ++x;

		if ((cnt <= VDMA_MAX_HSIZE) && ((cnt % 8) == 0))
			break;
	} while (x < VDMA_MAX_VSIZE);

	if (x > VDMA_MAX_VSIZE || cnt * x != st->txcount || ((cnt % 8) != 0))
		return -EINVAL;

	cf_axi_dds_stop(st);

	st->dma_config.vsize = x;
	st->dma_config.stride = st->dma_config.hsize = cnt;

	dmaengine_device_control(st->tx_chan, DMA_SLAVE_CONFIG,
		(unsigned long)&st->dma_config);

	dds_write(st, CF_AXI_DDS_DMA_FRAMECNT, st->txcount / 8);

	desc = dmaengine_prep_slave_single(st->tx_chan,
		st->buf_phys,
		st->txcount,
		DMA_MEM_TO_DEV, 0);
	if (!desc) {
		dev_err(indio_dev->dev.parent, "Failed to prepare DMA descriptor\n");
		return -ENOMEM;
	} else {
		dmaengine_submit(desc);
		dma_async_issue_pending(st->tx_chan);
	}

	dds_write(st, CF_AXI_DDS_CTRL, tmp_reg);
	dds_write(st, CF_AXI_DDS_DMA_STAT,
		  CF_AXI_DDS_DMA_STAT_OVF |
		  CF_AXI_DDS_DMA_STAT_UNF);

	return 0;

}

static int cf_axi_dds_hw_buffer_preenable(struct iio_dev *indio_dev)
{
	return __cf_axi_dds_hw_buffer_state_set(indio_dev, 1);
}

static int cf_axi_dds_hw_buffer_postdisable(struct iio_dev *indio_dev)
{
	return __cf_axi_dds_hw_buffer_state_set(indio_dev, 0);
}

static const struct iio_buffer_setup_ops cf_axi_dds_buffer_setup_ops = {
	.preenable = &cf_axi_dds_hw_buffer_preenable,
	.postdisable = &cf_axi_dds_hw_buffer_postdisable,
};

int cf_axi_dds_configure_buffer(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	indio_dev->buffer = cf_axi_dds_rb_allocate(indio_dev);
	if (indio_dev->buffer == NULL)
		return -ENOMEM;

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->buffer->direction = IIO_BUFFER_DIRECTION_OUT;
	indio_dev->buffer->access = &cf_axi_dds_buffer_access_funcs;
	indio_dev->setup_ops = &cf_axi_dds_buffer_setup_ops;

	st->buf_virt = dma_alloc_coherent(indio_dev->dev.parent,
					  PAGE_ALIGN(AXIDDS_MAX_DMA_SIZE), &st->buf_phys,
					  GFP_KERNEL);
	if (st->buf_virt == NULL) {
		dev_err(indio_dev->dev.parent,
			"Failed to allocate a dma memory\n");
		return -ENOMEM;
	}

	return 0;
}

void cf_axi_dds_unconfigure_buffer(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	dma_free_coherent(indio_dev->dev.parent, PAGE_ALIGN(AXIDDS_MAX_DMA_SIZE),
			  st->buf_virt, st->buf_phys);
	cf_axi_dds_rb_free(indio_dev->buffer);
}
