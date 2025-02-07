/*
 * Analog Devices MC-ADC Module
 *
 * Copyright 2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

/**
 * Note:
 * This driver is an old copy from the cf_axi_adc/axi-adc driver.
 * And some things were common with that driver. The cf_axi_adc/axi-adc
 * driver is a more complete implementation, while this one is just caring
 * about Motor Control.
 * The code duplication [here] is intentional, as we try to cleanup the
 * AXI ADC and decouple it from this driver.
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

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include <linux/fpga/adi-axi-common.h>

/* ADC Common */
#define ADI_REG_RSTN			0x0040
#define ADI_RSTN			(1 << 0)

#define ADI_REG_STATUS			0x005C
#define ADI_REG_DMA_STATUS		0x0088

/* ADC Channel */
#define ADI_REG_CHAN_CNTRL(c)		(0x0400 + (c) * 0x40)
#define ADI_ENABLE			(1 << 0)

#define ID_AD_MC_ADC   1
#define ID_AD_MC_ADC_RAMP 1
#define ID_AD_MC_ADC_SHA3 2

struct axiadc_chip_info {
	char				*name;
	unsigned			num_channels;
	const unsigned long		*scan_masks;
	unsigned int			max_rate;
	struct iio_chan_spec		channel[3];
};

struct axiadc_state {
	struct iio_info			iio_info;
	/* protect against device accesses */
	struct mutex			lock;
	void __iomem			*regs;
	unsigned int			pcore_version;
};

static inline void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}

static inline unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}

static int axiadc_reg_access(struct iio_dev *indio_dev,
			     unsigned reg, unsigned writeval,
			     unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	mutex_lock(&st->lock);
	if (readval == NULL)
		axiadc_write(st, reg & 0xFFFF, writeval);
	else
		*readval = axiadc_read(st, reg & 0xFFFF);
	mutex_unlock(&st->lock);

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
		.storagebits = _bits,			\
		.shift = 0,				\
	  },						\
	}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD_MC_ADC_RAMP] = {
		.name = "AD-MC-ADC-RAMP",
		.max_rate = 1000000UL,
		.num_channels = 1,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 64, 'u'),
		},
	},
	[ID_AD_MC_ADC_SHA3] = {
		.name = "AD-MC-ADC-SHA3",
		.max_rate = 1000000UL,
		.num_channels = 1,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 512, 'u'),
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

	//mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	//st->regs = devm_ioremap_resource(&pdev->dev, mem);
	//if (IS_ERR(st->regs))
	//	return PTR_ERR(st->regs);

	if (strcmp(pdev->dev.of_node->name, "sha3-reader") == 0) {
		dev_info(&pdev->dev, "sha3-reader");
		chip_info = &axiadc_chip_info_tbl[ID_AD_MC_ADC_SHA3];
	} else {
		dev_info(&pdev->dev, "ramp-reader");
		chip_info = &axiadc_chip_info_tbl[ID_AD_MC_ADC_RAMP];
	}

	platform_set_drvdata(pdev, indio_dev);

	/* Reset all HDL Cores */
	//axiadc_write(st, ADI_REG_RSTN, 0);
	//axiadc_write(st, ADI_REG_RSTN, ADI_RSTN);

	//st->pcore_version = axiadc_read(st, ADI_AXI_REG_VERSION);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip_info->channel;
	indio_dev->num_channels = chip_info->num_channels;

	st->iio_info = axiadc_info;
	indio_dev->info = &st->iio_info;

	ret = devm_iio_dmaengine_buffer_setup(&pdev->dev, indio_dev, "ad-mc-adc-dma");
	if (ret < 0)
		return ret;

	mutex_init(&st->lock);

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	//dev_info(&pdev->dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p, probed ADC %s as %s\n",
	//	 st->pcore_version,
	//	 (unsigned long long)mem->start, st->regs, chip_info->name,
	//	 axiadc_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER");

	dev_info(&pdev->dev, "ADI AIM probed ADC %s\n", chip_info->name);

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
};

module_platform_driver(axiadc_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
