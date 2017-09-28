/*
 * Analog Devices MC-ADC Module
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
#include <linux/platform_device.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"

#define ID_AD_MC_ADC   1

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
	  .scan_type = {				\
		.sign = _sign,				\
		.realbits = _bits,			\
		.storagebits = 16,			\
		.shift = 0,				\
	  },						\
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD_MC_ADC] = {
		.name = "AD-MC-ADC",
		.max_rate = 1000000UL,
		.num_channels = 3,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 16, 'u'),
			AIM_CHAN_NOCALIB(1, 1, 16, 'u'),
			AIM_CHAN_NOCALIB(2, 2, 16, 'u'),
		},
	},
};

static int axiadc_probe(struct platform_device *pdev)
{
	const struct axiadc_chip_info *chip_info;
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

	chip_info = &axiadc_chip_info_tbl[ID_AD_MC_ADC];

	platform_set_drvdata(pdev, indio_dev);

	/* Reset all HDL Cores */
	axiadc_write(st, ADI_REG_RSTN, 0);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip_info->channel;
	indio_dev->num_channels = chip_info->num_channels;

	st->iio_info = axiadc_info;
	indio_dev->info = &st->iio_info;

	ret = axiadc_configure_ring_stream(indio_dev, "ad-mc-adc-dma");
	if (ret < 0)
		return ret;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	*indio_dev->buffer->scan_mask = (1UL << indio_dev->num_channels) - 1;

	dev_info(&pdev->dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, probed ADC %s as %s\n",
		 st->pcore_version,
		 (unsigned long long)mem->start, st->regs, chip_info->name,
		 axiadc_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER");

	return 0;

err_unconfigure_ring:
	axiadc_unconfigure_ring_stream(indio_dev);

	return ret;
}

static int axiadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);

	iio_device_unregister(indio_dev);
	axiadc_unconfigure_ring_stream(indio_dev);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible = "xlnx,axi-ad-mc-adc-1.00.a", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

static struct platform_driver axiadc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe	  = axiadc_probe,
	.remove	 = axiadc_remove,
};

module_platform_driver(axiadc_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-ADC");
MODULE_LICENSE("GPL v2");
