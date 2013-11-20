/*
 * Analog Devices ADC Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/dmaengine.h>

#include <linux/of_device.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"

static inline void adc_write(struct axiadc_state *st,
	unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int adc_read(struct axiadc_state *st,
	unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int adc_reg_access(struct iio_dev *indio_dev,
	unsigned reg, unsigned writeval, unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL)
		adc_write(st, reg & 0xFFFF, writeval);
	else
		*readval = adc_read(st, reg & 0xFFFF);
	mutex_unlock(&indio_dev->mlock);

	return 0;
}

static const struct iio_info adc_info = {
	.debugfs_reg_access = &adc_reg_access,
};

// Modifed to add real bits, shift, etc
#define AIM_CHAN_NOCALIB(_chan, _si, _real_bits, _storage_bits, _shift, _sign)		  \
	{ .type = IIO_VOLTAGE,					  \
	  .indexed = 1,						 \
	  .channel = _chan,					 \
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _real_bits, _storage_bits, _shift)}

static int adc_parse_dt_string(struct device_node *np, const char **name_pointer)
{	
	return(of_property_read_string(np, "adc-name-id", name_pointer));
} 

static int adc_of_probe(struct platform_device *op)
{
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	struct axiadc_state *st;
	struct resource r_mem;
	resource_size_t remap_size, phys_addr;
	const char *adc_name;
	int ret;
	
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
		goto failed1;
	}

	st->rx_chan = dma_request_slave_channel(&op->dev,
			"ad-adc-dma");
	if (!st->rx_chan) {
		ret = -EPROBE_DEFER;
		dev_err(dev, "Failed to find rx dma device\n");
		goto failed1;
	}

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adc_info;

	/* Reset all HDL Cores */
	adc_write(st, ADI_REG_RSTN, 0);
	adc_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = adc_read(st, ADI_REG_VERSION);
	st->max_count = AXIADC_MAX_DMA_SIZE;
	st->dma_align = ADI_DMA_BUSWIDTH(adc_read(st,
				ADI_REG_DMA_BUSWIDTH));
	
	// Read adc type from device tree
	ret = adc_parse_dt_string(op->dev.of_node, &adc_name);
	
	if(ret == (-EINVAL))
	{
		dev_err(dev, "Failed to select ADC. Property not found in devicetree.\n");
		goto failed1;
	}
	if(ret == (-ENODATA))
	{
		dev_err(dev, "Failed to select ADC. Property found in devicetree, but has no value\n");
		goto failed1;
	}
	if(ret == (-EILSEQ))
	{
		dev_err(dev, "Failed to select ADC. Property found but noy NULL terminated\n");
		goto failed1;
	}
	
	ret = 1;
	while(ret != 0)
	{
		ret = strcmp(adc_name, "ad7476a");
		if(ret == 0)
		{
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 32, 0, 'u');
			st->channels[1] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(1, 1, 16, 32, 16, 'u');
			indio_dev->num_channels = 2;
			indio_dev->masklength = 2;
			break;
		}

		ret = strcmp(adc_name, "ad7091r"); 
		if(ret == 0)
		{
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 32, 16, 'u');
			indio_dev->num_channels = 1;
			indio_dev->masklength = 1;
			break;
		}
		
		ret = strcmp(adc_name, "ad7780");
		if(ret == 0)
		{
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 24, 32, 8, 'u'); 
			indio_dev->num_channels = 1;
			indio_dev->masklength = 1;		  
			break;	 
		} 
		
		ret = strcmp(adc_name, "ad7980");
		if(ret == 0)
		{
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 32, 16, 'u'); 
			indio_dev->num_channels = 1;
			indio_dev->masklength = 1;		  
			break;	 
		} 		
		
		dev_err(dev, "ADC not found in supported drivers list\n");  
		goto failed1;
	}

	indio_dev->channels = st->channels;


	init_completion(&st->dma_complete);

	axiadc_configure_ring(indio_dev);

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto failed2;

	*indio_dev->buffer->scan_mask = (1UL << 2) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed2;

	return 0;

failed2:
	axiadc_unconfigure_ring(indio_dev);
	dma_release_channel(st->rx_chan);
failed1:
	release_mem_region(phys_addr, remap_size);

	return -1;
}

static int adc_of_remove(struct platform_device *op)
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

static const struct of_device_id adc_of_match[] = {
	{ .compatible = "xlnx,axi-ad-adc-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adc_of_match);

static struct platform_driver adc_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = adc_of_match,
	},
	.probe	  = adc_of_probe,
	.remove	 = adc_of_remove,
};

module_platform_driver(adc_of_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADC");
MODULE_LICENSE("GPL v2");
