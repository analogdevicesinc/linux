/*
 * Analog Devices MC-Speed Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/wait.h>
#include <linux/spi/spi.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/clk.h>

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"

#define ID_AD_MC_SPEED   1

static int axiadc_reg_access(struct iio_dev *indio_dev,
			     unsigned reg, unsigned writeval,
			     unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		axiadc_write(st, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(st, reg & 0xFFFF);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
		const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;

		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static const struct iio_info axiadc_info = {
	.driver_module = THIS_MODULE,
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
};

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)		  \
	{ .type = IIO_VOLTAGE,					  \
	  .indexed = 1,						 \
	  .channel = _chan,					 \
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 32, 0)}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD_MC_SPEED] = {
	.name = "AD-MC-SPEED",
	.max_rate = 1000000UL,
	.num_channels = 1,
	.channel[0] = AIM_CHAN_NOCALIB(0, 0, 32, 'u'),
	},
};

static int axiadc_of_probe(struct platform_device *op)
{
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	struct axiadc_state *st;
	struct resource r_mem; /* IO mem resources */
	struct axiadc_converter *conv;
	resource_size_t remap_size, phys_addr;
	unsigned i, cnt;
	int ret;

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "Invalid address\n");
		return ret;
	}

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	dev_set_drvdata(dev, indio_dev);
	mutex_init(&st->lock);

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}

	st->rx_chan = dma_request_slave_channel(&op->dev, "ad-mc-speed-dma");
	if (!st->rx_chan) {
		ret = -EPROBE_DEFER;
		dev_err(dev, "Failed to find rx dma device\n");
		goto failed2;
	}

	conv = kmalloc(sizeof(struct axiadc_converter), GFP_KERNEL);

	conv->chip_info = &axiadc_chip_info_tbl[ID_AD_MC_SPEED];

	/* Reset all HDL Cores */
	axiadc_write(st, ADI_REG_RSTN, 0);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);
	st->max_count = AXIADC_MAX_DMA_SIZE;

	st->dma_align = ADI_DMA_BUSWIDTH(axiadc_read(st, ADI_REG_DMA_BUSWIDTH));

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	for (i = 0, cnt = 0; i < conv->chip_info->num_channels; i++)
		st->channels[cnt++] = conv->chip_info->channel[i];

	indio_dev->channels = st->channels;
	indio_dev->num_channels = cnt;
	indio_dev->masklength = cnt;

	st->iio_info = axiadc_info;
	indio_dev->info = &st->iio_info;

	init_completion(&st->dma_complete);

	axiadc_configure_ring(indio_dev);

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto failed3;

	*indio_dev->buffer->scan_mask =
			(1UL << conv->chip_info->num_channels) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed3;

	dev_info(dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, DMA-%d probed ADC %s as %s\n",
		 st->pcore_version,
		 (unsigned long long)phys_addr, st->regs,
		 st->rx_chan->chan_id, conv->chip_info->name,
		 axiadc_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER");

	return 0;

failed3:
	axiadc_unconfigure_ring(indio_dev);
	dma_release_channel(st->rx_chan);
failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	iio_device_free(indio_dev);
	dev_set_drvdata(dev, NULL);

	return ret;
}

static int axiadc_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	axiadc_unconfigure_ring(indio_dev);

	dma_release_channel(st->rx_chan);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	iio_device_free(indio_dev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible = "xlnx,axi-ad-mc-speed-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

static struct platform_driver axiadc_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe	  = axiadc_of_probe,
	.remove	 = axiadc_of_remove,
};

module_platform_driver(axiadc_of_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-Speed");
MODULE_LICENSE("GPL v2");
