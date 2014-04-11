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
#include <linux/platform_device.h>
#include <linux/of.h>

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

static int adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	const char *adc_name;
	int ret;
	
	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &adc_info;

	/* Reset all HDL Cores */
	adc_write(st, ADI_REG_RSTN, 0);
	adc_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = adc_read(st, ADI_REG_VERSION);
	
	// Read adc type from device tree
	ret = adc_parse_dt_string(pdev->dev.of_node, &adc_name);
	switch(ret) {
	case -EINVAL:
		dev_err(&pdev->dev, "Failed to select ADC. Property not found in devicetree.\n");
		return ret;
	case -ENODATA:
		dev_err(&pdev->dev, "Failed to select ADC. Property found in devicetree, but has no value\n");
		return ret;
	case -EILSEQ:
		dev_err(&pdev->dev, "Failed to select ADC. Property found but noy NULL terminated\n");
		return ret;
	default:
		break;
	}
	
	ret = 1;
	while(ret != 0) {
		ret = strcmp(adc_name, "ad7476a");
		if(ret == 0) {
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 32, 0, 'u');
			st->channels[1] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(1, 1, 16, 32, 16, 'u');
			indio_dev->num_channels = 2;
			indio_dev->masklength = 2;
			break;
		}

		ret = strcmp(adc_name, "ad7091r"); 
		if(ret == 0) {
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 32, 16, 'u');
			indio_dev->num_channels = 1;
			indio_dev->masklength = 1;
			break;
		}
		
		ret = strcmp(adc_name, "ad7780");
		if(ret == 0) {
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 24, 32, 8, 'u'); 
			indio_dev->num_channels = 1;
			indio_dev->masklength = 1;		  
			break;	 
		} 
		
		ret = strcmp(adc_name, "ad7980");
		if(ret == 0) {
			st->channels[0] = (struct iio_chan_spec)AIM_CHAN_NOCALIB(0, 0, 16, 32, 16, 'u'); 
			indio_dev->num_channels = 1;
			indio_dev->masklength = 1;		  
			break;	 
		} 		
		
		dev_err(&pdev->dev, "ADC not found in supported drivers list\n");  
		return ret;
	}

	indio_dev->channels = st->channels;

	st->streaming_dma = of_property_read_bool(pdev->dev.of_node,
                        "adi,streaming-dma");

	if (st->streaming_dma)
			axiadc_configure_ring_stream(indio_dev, "ad-adc-dma");
	else
			axiadc_configure_ring(indio_dev, "ad-adc-dma");

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto err_unconfigure_ring;

	*indio_dev->buffer->scan_mask = (1UL << 2) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_iio_unregister_buffer;

	return 0;

err_iio_unregister_buffer:
	iio_buffer_unregister(indio_dev);
err_unconfigure_ring:
	if (st->streaming_dma)
		axiadc_unconfigure_ring_stream(indio_dev);
	else
		axiadc_unconfigure_ring(indio_dev);

	return ret;
}

static int adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	axiadc_unconfigure_ring(indio_dev);

	return 0;
} 

static const struct of_device_id adc_of_match[] = {
	{ .compatible = "xlnx,axi-ad-adc-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adc_of_match);

static struct platform_driver adc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = adc_of_match,
	},
	.probe	  = adc_probe,
	.remove	 = adc_remove,
};

module_platform_driver(adc_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADC");
MODULE_LICENSE("GPL v2");
