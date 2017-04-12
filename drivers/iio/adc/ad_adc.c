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

#include <linux/clk.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/hw_consumer.h>

#include "cf_axi_adc.h"

struct adc_chip_info {
	bool has_frontend;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	unsigned int ctrl_flags;
};

#define CN0363_CHANNEL(_address, _type, _ch, _mod, _rb) { \
	.type = _type, \
	.indexed = 1, \
	.channel = _ch, \
	.modified = (_mod == 0) ? 0 : 1, \
	.channel2 = _mod, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.address = _address, \
	.scan_index = _address, \
	.scan_type = { \
		.sign = (_type == IIO_ANGL) ? 'u' : 's', \
		.realbits = _rb, \
		.storagebits = 32, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec cn0363_channels[] = {
	CN0363_CHANNEL(0, IIO_ANGL, 0, 0, 32),
	CN0363_CHANNEL(1, IIO_VOLTAGE, 0, 0, 24),
	CN0363_CHANNEL(2, IIO_VOLTAGE, 1, 0, 32),
	CN0363_CHANNEL(3, IIO_VOLTAGE, 2, IIO_MOD_I, 32),
	CN0363_CHANNEL(4, IIO_VOLTAGE, 2, IIO_MOD_Q, 32),
	CN0363_CHANNEL(5, IIO_VOLTAGE, 3, IIO_MOD_I, 32),
	CN0363_CHANNEL(6, IIO_VOLTAGE, 3, IIO_MOD_Q, 32),
	CN0363_CHANNEL(7, IIO_ANGL, 1, 0, 32),
	CN0363_CHANNEL(8, IIO_VOLTAGE, 4, 0, 24),
	CN0363_CHANNEL(9, IIO_VOLTAGE, 5, 0, 32),
	CN0363_CHANNEL(10, IIO_VOLTAGE, 6, IIO_MOD_I, 32),
	CN0363_CHANNEL(11, IIO_VOLTAGE, 6, IIO_MOD_Q, 32),
	CN0363_CHANNEL(12, IIO_VOLTAGE, 7, IIO_MOD_I, 32),
	CN0363_CHANNEL(13, IIO_VOLTAGE, 7, IIO_MOD_Q, 32),
};

static const struct adc_chip_info cn0363_chip_info = {
	.has_frontend = true,
	.channels = cn0363_channels,
	.num_channels = ARRAY_SIZE(cn0363_channels),
};

#define AD9361_OBS_RX_CHANNEL(_ch, _mod, _si) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _ch, \
	.modified = (_mod == 0) ? 0 : 1, \
	.channel2 = _mod, \
	.address = _ch, \
	.scan_index = _si, \
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_type = { \
		.sign = 's', \
		.realbits = 16, \
		.storagebits = 16, \
		.shift = 0, \
		.endianness = IIO_LE, \
	}, \
}

static const struct iio_chan_spec ad9371_obs_rx_channels[] = {
	AD9361_OBS_RX_CHANNEL(0, IIO_MOD_I, 0),
	AD9361_OBS_RX_CHANNEL(0, IIO_MOD_Q, 1),
};

static const struct adc_chip_info ad9371_obs_rx_chip_info = {
	.channels = ad9371_obs_rx_channels,
	.num_channels = ARRAY_SIZE(ad9371_obs_rx_channels),
	.ctrl_flags = ADI_FORMAT_SIGNEXT | ADI_FORMAT_ENABLE,
};

static int axiadc_hw_consumer_postenable(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	return iio_hw_consumer_enable(st->frontend);
}

static int axiadc_hw_consumer_predisable(struct iio_dev *indio_dev)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_hw_consumer_disable(st->frontend);
	return 0;
}

static const struct iio_buffer_setup_ops axiadc_hw_consumer_setup_ops = {
	.postenable = &axiadc_hw_consumer_postenable,
	.predisable = &axiadc_hw_consumer_predisable,
};

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

		ctrl |= st->adc_def_output_mode;

		axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static int axiadc_read_raw(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, int *val, int *val2, long info)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (IS_ERR(st->clk)) {
			return -ENODEV;
		} else {
			*val = clk_get_rate(st->clk);
		}
		return IIO_VAL_INT;
	default:
		break;
	}

	return -EINVAL;
}

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
	.read_raw = axiadc_read_raw,
	.driver_module = THIS_MODULE,
	.debugfs_reg_access = &adc_reg_access,
	.update_scan_mode = axiadc_update_scan_mode,
};

static const struct of_device_id adc_of_match[] = {
	{ .compatible = "adi,cn0363-adc-1.00.a", .data = &cn0363_chip_info },
	{ .compatible = "adi,axi-ad9371-obs-1.0", .data = &ad9371_obs_rx_chip_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, adc_of_match);

static int adc_probe(struct platform_device *pdev)
{
	const struct adc_chip_info *info;
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	int ret;

	id = of_match_node(adc_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;
	info = id->data;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->clk = devm_clk_get(&pdev->dev, "sampl_clk");
	if (IS_ERR(st->clk)) {
		return PTR_ERR(st->clk);
	} else {
		ret = clk_prepare_enable(st->clk);
		if (ret)
			return ret;
	}

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

	if (info->has_frontend) {
		st->frontend = iio_hw_consumer_alloc(&pdev->dev);
		if (IS_ERR(st->frontend))
				return PTR_ERR(st->frontend);
		indio_dev->setup_ops = &axiadc_hw_consumer_setup_ops;

	}

	indio_dev->channels = info->channels;
	indio_dev->num_channels = info->num_channels;
	ret = axiadc_configure_ring_stream(indio_dev, "rx");
	if (ret)
		goto err_free_frontend;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	return 0;

err_unconfigure_ring:
	axiadc_unconfigure_ring_stream(indio_dev);
err_free_frontend:
	if (st->frontend)
		iio_hw_consumer_free(st->frontend);

	return ret;
}

static int adc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	axiadc_unconfigure_ring_stream(indio_dev);
	if (st->frontend)
		iio_hw_consumer_free(st->frontend);

	if (!IS_ERR(st->clk))
		clk_disable_unprepare(st->clk);

	return 0;
}

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
