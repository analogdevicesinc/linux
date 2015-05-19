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

/*
 * Don't use this for new devices, this is only in here for backwards
 * compatibility and might be dropped at some point
 */
struct adc_legacy_chip_info {
	const char *name;
	struct iio_chan_spec channels[2];
	unsigned int num_channels;
};

static const struct adc_legacy_chip_info adc_legacy_chip_info_table[] = {
	{
		.name = "ad7476a",
		.num_channels = 2,
		.channels = {
			AIM_CHAN_NOCALIB(0, 0, 16, 32, 0, 'u'),
			AIM_CHAN_NOCALIB(1, 1, 16, 32, 16, 'u'),
		},
	}, {
		.name = "ad7091r",
		.num_channels = 1,
		.channels = {
			AIM_CHAN_NOCALIB(0, 0, 16, 32, 16, 'u'),
		},
	}, {
		.name = "ad7780",
		.num_channels = 1,
		.channels = {
			AIM_CHAN_NOCALIB(0, 0, 24, 32, 8, 'u'),
		},
	}, {
		.name = "ad7980",
		.num_channels = 1,
		.channels = {
			AIM_CHAN_NOCALIB(0, 0, 16, 32, 16, 'u'),
		},
	}
};

static int adc_legacy_probe(struct platform_device *pdev,
	struct iio_dev *indio_dev)
{
	const struct adc_legacy_chip_info *info = NULL;
	struct axiadc_state *st = iio_priv(indio_dev);
	struct device_node *np = pdev->dev.of_node;
	const char *adc_name;
	unsigned int i;
	int ret;

	ret = of_property_read_string(np, "adc-name-id", &adc_name);
	switch (ret) {
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

	for (i = 0; i < ARRAY_SIZE(adc_legacy_chip_info_table); i++) {
		if (strcmp(adc_legacy_chip_info_table[i].name, adc_name) == 0) {
			info = &adc_legacy_chip_info_table[i];
			break;
		}
	}

	if (info == NULL) {
		dev_err(&pdev->dev, "ADC not found in supported drivers list\n");
		return -ENODEV;
	}

	indio_dev->channels = info->channels;
	indio_dev->num_channels = info->num_channels;
	st->streaming_dma = of_property_read_bool(np, "adi,streaming-dma");

	if (st->streaming_dma)
		axiadc_configure_ring_stream(indio_dev, "ad-adc-dma");
	else
		axiadc_configure_ring(indio_dev, "ad-adc-dma");

	return 0;
}

static int adc_probe(struct platform_device *pdev)
{
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
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

	ret = adc_legacy_probe(pdev, indio_dev);
	if (ret)
		return ret;

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto err_unconfigure_ring;

	*indio_dev->buffer->scan_mask = (1UL << 2) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	return 0;

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
