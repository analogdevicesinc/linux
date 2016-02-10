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
#include <linux/iio/dma-buffer.h>
#include <linux/iio/dmaengine.h>

#include "cf_axi_dds.h"

static int dds_buffer_submit_block(void *data, struct iio_dma_buffer_block *block)
{
	return iio_dmaengine_buffer_submit_block(block, DMA_TO_DEVICE);
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

	cf_axi_dds_stop(st);
	st->enable = false;

	dds_write(st, ADI_REG_VDMA_STATUS, ADI_VDMA_OVF | ADI_VDMA_UNF);

	st->enable = true;
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
	.submit_block = dds_buffer_submit_block,
};

int cf_axi_dds_configure_buffer(struct iio_dev *indio_dev)
{
	struct iio_buffer *buffer;

	buffer = iio_dmaengine_buffer_alloc(indio_dev->dev.parent, "tx",
			&dds_buffer_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	iio_device_attach_buffer(indio_dev, buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	indio_dev->direction = IIO_DEVICE_DIRECTION_OUT;
	indio_dev->setup_ops = &dds_buffer_setup_ops;

	return 0;
}
EXPORT_SYMBOL_GPL(cf_axi_dds_configure_buffer);

void cf_axi_dds_unconfigure_buffer(struct iio_dev *indio_dev)
{
	iio_dmaengine_buffer_free(indio_dev->buffer);
}
EXPORT_SYMBOL_GPL(cf_axi_dds_unconfigure_buffer);
