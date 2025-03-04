/*
 * Analog Devices SHA3 IP Module
 *
 * Copyright 2025 Analog Devices Inc.
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
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/dma-direction.h>
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/consumer.h>
#include <linux/iio/hw-consumer.h>

#include <linux/fpga/adi-axi-common.h>

struct sha3_chip_info {
	unsigned			num_channels;
	unsigned int			max_rate;
	struct iio_chan_spec		channel[3];
};

enum ad_sha3_buffer {
	AD_SHA3_RAMP,
	AD_SHA3_DATA,
};

static const char * const sha_dma_names[] = {
	[AD_SHA3_RAMP] = "ramp-dma",
	[AD_SHA3_DATA] = "sha3-dma",
};

static const char * const reset_gpio_names[] = {
	[AD_SHA3_RAMP] = "ramp-reset",
	[AD_SHA3_DATA] = "sha-reset",
};

static const char * const device_names[] = {
	[AD_SHA3_RAMP] = "ramp-reader",
	[AD_SHA3_DATA] = "sha3-reader",
};

struct sha3_state {
	struct iio_dev *ch_indio_dev[ARRAY_SIZE(device_names)];
	struct iio_hw_consumer *ramp_hw_cons;
	struct iio_channel *ramp_channel;
	struct gpio_desc *ramp_reset_gpio;
	struct gpio_desc *sha3_reset_gpio;
};

static int sha3_reg_access(struct iio_dev *indio_dev, unsigned reg,
			     unsigned writeval, unsigned *readval)
{
	dev_info(indio_dev->dev.parent, "%s\n", __func__);
	return 0;
}

static const struct iio_info sha3_info = {
	.debugfs_reg_access = &sha3_reg_access,
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

static const struct sha3_chip_info sha3_chip_info_tbl[] = {
	[AD_SHA3_RAMP] = {
		.max_rate = 1000000UL,
		.num_channels = 1,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 64, 'u'),
		},
	},
	[AD_SHA3_DATA] = {
		.max_rate = 1000000UL,
		.num_channels = 1,
		.channel = {
			AIM_CHAN_NOCALIB(0, 0, 512, 'u'),
		},
	},
};

static int sha3_probe(struct platform_device *pdev)
{
	const struct sha3_chip_info *chip_info;
	struct iio_dev *indio_dev;
	struct sha3_state *st;
	int ret;

	dev_info(&pdev->dev, "Probing SHA3 driver\n");

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	if (strcmp(pdev->dev.of_node->name, "sha3-reader") == 0) {
		dev_info(&pdev->dev, "sha3-reader");
		//st->regs = 0x64a00000;
		chip_info = &sha3_chip_info_tbl[AD_SHA3_DATA];

	} else {
		dev_info(&pdev->dev, "ramp-reader");
		//st->regs = 0x65a00000;
		chip_info = &sha3_chip_info_tbl[AD_SHA3_RAMP];
	}

	platform_set_drvdata(pdev, indio_dev);

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	indio_dev->channels = chip_info->channel;
	indio_dev->num_channels = chip_info->num_channels;

	indio_dev->info = &sha3_info;

	ret = devm_iio_dmaengine_buffer_setup(&pdev->dev, indio_dev, "sha3-dma");
	if (ret < 0)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "ADI AIM probed ADC\n");

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id sha3_of_match[] = {
	{ .compatible = "adi,sha3", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, sha3_of_match);

static struct platform_driver sha3_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = sha3_of_match,
	},
	.probe	  = sha3_probe,
};

module_platform_driver(sha3_driver);

MODULE_AUTHOR("Marcelo Schmitt <marcelo.schmitt@analog.com>");
MODULE_DESCRIPTION("Analog Devices MC-ADC");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
