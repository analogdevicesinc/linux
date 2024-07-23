/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
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
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "cf_axi_dds.h"

static int dds_buffer_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct cf_axi_dds_state *st = iio_priv(queue->driver_data);
	bool enable_fifo = false;
	bool oneshot = true;

#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
	if (block->block.bytes_used) {
#else
	if (block->bytes_used) {
#endif
		if (cf_axi_dds_dma_fifo_en(st)) {
			enable_fifo = true;
#ifdef CONFIG_IIO_DMA_BUF_MMAP_LEGACY
			if (block->block.flags & IIO_BUFFER_BLOCK_FLAG_CYCLIC) {
				block->block.flags &= ~IIO_BUFFER_BLOCK_FLAG_CYCLIC;
#else
			if (block->cyclic) {
				block->cyclic = false;
#endif
				oneshot = false;
			}

			cf_axi_dds_pl_ddr_fifo_ctrl_oneshot(st, oneshot);
		}

		cf_axi_dds_pl_ddr_fifo_ctrl(st, enable_fifo);
	}

	return iio_dmaengine_buffer_submit_block(queue, block);
}

static int dds_buffer_state_set(struct iio_dev *indio_dev, bool state)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

#if 0
	tmp_reg = dds_read(st, CF_AXI_DDS_DMA_STAT);
	if (tmp_reg & (CF_AXI_DDS_DMA_STAT_OVF | CF_AXI_DDS_DMA_STAT_UNF))
		dev_warn(indio_dev->dev.parent, "VDMA Status: %s %s\n",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "overflow" : "",
			 (tmp_reg & CF_AXI_DDS_DMA_STAT_OVF) ? "underflow" : "");
#endif

	if (!state)
		return cf_axi_dds_datasel(st, -1, DATA_SEL_DDS);

	dds_write(st, ADI_REG_VDMA_STATUS, ADI_VDMA_OVF | ADI_VDMA_UNF);

	cf_axi_dds_start_sync(st, 1);

	return 0;
}

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

static const struct iio_dma_buffer_ops dds_buffer_dma_buffer_ops = {
	.submit = dds_buffer_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

int cf_axi_dds_configure_buffer(struct iio_dev *indio_dev)
{
	return devm_iio_dmaengine_buffer_setup_with_ops(indio_dev->dev.parent,
						   indio_dev, "tx",
						   IIO_BUFFER_DIRECTION_OUT,
						   &dds_buffer_dma_buffer_ops, indio_dev);
}
EXPORT_SYMBOL_GPL(cf_axi_dds_configure_buffer);
