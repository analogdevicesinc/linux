/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2013 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/io.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/spi/spi.h>
#include <asm/div64.h>

#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_dds.h"
#include "ad9122.h"

#define DRIVER_NAME		"cf_axi_dds"

struct dds_spidev {
	struct device_node *of_nspi;
	struct device *dev_spi;
};

static int cf_axi_dds_sync_frame(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int stat;
	static int retry = 0;

	mdelay(10); /* Wait until clocks are stable */

	dds_write(st, ADI_REG_FRAME, 0);
	dds_write(st, ADI_REG_FRAME, ADI_FRAME);

	if (conv->get_fifo_status) {
		/* Check FIFO status */
		stat = conv->get_fifo_status(conv);
		if (stat) {
			if (retry++ > 10) {
				dev_warn(indio_dev->dev.parent,
					 "FRAME/FIFO Reset Retry cnt\n");
				return -EIO;
			}
			cf_axi_dds_sync_frame(indio_dev);
		}
	}

	retry = 0;
	return 0;
}

static void cf_axi_dds_set_sed_pattern(struct iio_dev *indio_dev, unsigned chan,
				      unsigned pat1, unsigned pat2)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned ctrl;

	dds_write(st, ADI_REG_CHAN_CNTRL_5(chan),
		  ADI_TO_DDS_PATT_1(pat1) | ADI_DDS_PATT_2(pat2));

	dds_write(st, ADI_REG_CNTRL_1, 0);

	ctrl = dds_read(st, ADI_REG_CNTRL_2) & ~ADI_DATA_SEL(~0);
	dds_write(st, ADI_REG_CNTRL_2, ctrl | ADI_DATA_SEL(DATA_SEL_SED) | ADI_DATA_FORMAT);

	dds_write(st, ADI_REG_CNTRL_1, ADI_ENABLE);
}

static void cf_axi_dds_stop(struct cf_axi_dds_state *st) {
	dds_write(st, ADI_REG_CNTRL_1, 0);
}

static int cf_axi_dds_default_setup(struct cf_axi_dds_state *st, u32 chan,
				    u32 phase, u32 freq, u32 scale) {

	unsigned long long val64;
	u32 val;

	val64 = (u64) freq * 0xFFFFULL;
	do_div(val64, st->dac_clk);
	val = ADI_DDS_INCR(val64) | 1;

	val64 = (u64) phase * 0x10000ULL + (360000 / 2);
	do_div(val64, 360000);
	val |= ADI_DDS_INIT(val64);

	dds_write(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan), ADI_DDS_SCALE(scale));
	dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan), val);

	return 0;
}

static int cf_axi_dds_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long long val64;
	unsigned reg;
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (m) {
	case 0:
		if (!chan->output) {
			ret = -EINVAL;
			break;
		}
		*val = !!(dds_read(st, ADI_REG_CNTRL_1) & ADI_ENABLE);

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		reg = ADI_TO_DDS_SCALE(dds_read(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel)));
		if (!reg) {
			*val = 1;
			*val2 = 0;
		} else {
			*val = 0;
			*val2 = 1000000 >> reg;
		}
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_FREQUENCY:
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		val64 = (u64)ADI_TO_DDS_INCR(reg) * (u64)st->dac_clk;
		do_div(val64, 0xFFFF);
		*val = val64;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		val64 = (u64)ADI_TO_DDS_INIT(reg) * 360000ULL + (0x10000 / 2);
		do_div(val64, 0x10000);
		*val = val64;
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->get_data_clk) {
			ret = -ENODEV;
			break;
		}
		*val = st->dac_clk = conv->get_data_clk(conv);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	default:
		ret = conv->read_raw(indio_dev, chan, val, val2, m);
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int cf_axi_dds_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	unsigned long long val64;
	unsigned reg, ctrl_reg;
	int i, ret = 0;

	mutex_lock(&indio_dev->mlock);
	ctrl_reg = dds_read(st, ADI_REG_CNTRL_1);

	switch (mask) {
	case 0:
		if (!chan->output) {
			ret = -EINVAL;
			break;
		}

		if (val)
			ctrl_reg |= ADI_ENABLE;
		else
			ctrl_reg &= ~ADI_ENABLE;

		dds_write(st, ADI_REG_CNTRL_1, ctrl_reg);
		break;
	case IIO_CHAN_INFO_SCALE:
		if (val == 1) {
			i = 0;
		} else {
			for (i = 1; i < 16; i++)
				if (val2 == (1000000 >> i))
					break;
		}
		cf_axi_dds_stop(st);
		dds_write(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel), ADI_DDS_SCALE(i));
		dds_write(st, ADI_REG_CNTRL_1, ctrl_reg);
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (!chan->output) {
			st->dac_clk = val;
			break;
		}
		if (val > (st->dac_clk / 2)) {
			ret = -EINVAL;
			break;
		}
		cf_axi_dds_stop(st);
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		reg &= ~ADI_DDS_INCR(~0);
		val64 = (u64) val * 0xFFFFULL;
		do_div(val64, st->dac_clk);
		reg |= ADI_DDS_INCR(val64) | 1;
		dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel), reg);
		dds_write(st, ADI_REG_CNTRL_1, ctrl_reg);
		break;
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > 360000) {
			ret = -EINVAL;
			break;
		}

		if (val == 360000)
			val = 0;

		dds_write(st, ADI_REG_CNTRL_1, 0);
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		reg &= ~ADI_DDS_INIT(~0);
		val64 = (u64) val * 0x10000ULL + (360000 / 2);
		do_div(val64, 360000);
		reg |= ADI_DDS_INIT(val64);
		dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel), reg);
		dds_write(st, ADI_REG_CNTRL_1, ctrl_reg);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->write_raw) {
			ret = -ENODEV;
			break;
		}

		reg = dds_read(st, ADI_REG_CNTRL_2);
		cf_axi_dds_stop(st);
		conv->write_raw(indio_dev, chan, val, val2, mask);
		dds_write(st, ADI_REG_CNTRL_2, reg);
		st->dac_clk = conv->get_data_clk(conv);
		dds_write(st, ADI_REG_CNTRL_1, ctrl_reg);
		ret = cf_axi_dds_sync_frame(indio_dev);
		break;
	default:
		ret = conv->write_raw(indio_dev, chan, val, val2, mask);
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int cf_axi_dds_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv = iio_device_get_drvdata(indio_dev);
	int ret;

	if (IS_ERR(conv))
		return PTR_ERR(conv);

	if ((reg & ~DEBUGFS_DRA_PCORE_REG_MAGIC) > 0xFF)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		if (reg & DEBUGFS_DRA_PCORE_REG_MAGIC) {
			dds_write(st, reg & 0xFFFF, writeval);
			ret = 0;
		} else {
			ret = conv->write(conv->spi, reg, writeval & 0xFF);
		}
	} else {
		if (reg & DEBUGFS_DRA_PCORE_REG_MAGIC) {
			ret = dds_read(st, reg & 0xFFFF);
		} else {
			ret = conv->read(conv->spi, reg);
			if (ret < 0)
				goto out_unlock;
		}
		*readval = ret;
		ret = 0;

	}

out_unlock:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static const char * const cf_axi_dds_scale[] = {
	"1.000000", "0.500000", "0.250000", "0.125000",
	"0.062500", "0.031250", "0.015625", "0.007812",
	"0.003906", "0.001953", "0.000976", "0.000488",
	"0.000244", "0.000122", "0.000061", "0.000030"
};

static const struct iio_enum cf_axi_dds_scale_available = {
	.items = cf_axi_dds_scale,
	.num_items = ARRAY_SIZE(cf_axi_dds_scale),
};

static const struct iio_chan_spec_ext_info cf_axi_dds_ext_info[] = {
	IIO_ENUM_AVAILABLE("scale", &cf_axi_dds_scale_available),
	{ },
};

#define CF_AXI_DDS_CHAN(_chan, _address, _extend_name)			\
	{ .type = IIO_ALTVOLTAGE,					\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
		       BIT(IIO_CHAN_INFO_SCALE) |			\
	  	       BIT(IIO_CHAN_INFO_PHASE) |			\
		       BIT(IIO_CHAN_INFO_FREQUENCY),			\
	  .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ),	\
	  .address = _address,						\
	  .output = 1,							\
	  .extend_name = _extend_name,					\
	  .ext_info = cf_axi_dds_ext_info,				\
	  }

#define CF_AXI_DDS_CHAN_BUF(_chan)					\
	{ .type = IIO_ALTVOLTAGE,					\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .output = 1,							\
	  .scan_index = _chan,						\
	  .scan_type = IIO_ST('s', 16, 16, 0),				\
}

static const struct cf_axi_dds_chip_info cf_axi_dds_chip_info_tbl[] = {
	[ID_AD9122] = {
		.name = "AD9122",
		.channel[1] = CF_AXI_DDS_CHAN(0, 0, "1A"),
		.channel[2] = CF_AXI_DDS_CHAN(1, 0, "1B"),
		.channel[3] = CF_AXI_DDS_CHAN(2, 0, "2A"),
		.channel[4] = CF_AXI_DDS_CHAN(3, 0, "2B"),
		.channel[0] = {
			.type = IIO_TEMP,
			.indexed = 1,
			.channel = 0,
			.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED) |
					      BIT(IIO_CHAN_INFO_CALIBBIAS),
		},
		.buf_channel[0] = CF_AXI_DDS_CHAN_BUF(0),
		.buf_channel[1] = CF_AXI_DDS_CHAN_BUF(1),
		.num_channels = 5,
		.num_buf_channels = 2,
		.num_dp_disable_channels = 1,
	},
	[ID_AD9739A] = {
		.name = "AD9739A",
		.channel[0] = CF_AXI_DDS_CHAN(0, 0, "1A"),
		.channel[1] = CF_AXI_DDS_CHAN(1, 0, "1B"),
		.buf_channel[0] = CF_AXI_DDS_CHAN_BUF(0),
		.num_channels = 2,
		.num_buf_channels = 1,
	},
};

static const struct iio_info cf_axi_dds_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
};

static int dds_attach_spi_client(struct device *dev, void *data)
{
	struct dds_spidev *dds_spidev = data;

	if ((dds_spidev->of_nspi == dev->of_node) && dev->driver) {
		dds_spidev->dev_spi = dev;
		return 1;
	}

	return 0;
}

struct axidds_core_info {
	unsigned int version;
	bool has_fifo_interface;
};

static const struct axidds_core_info ad9122_6_00_a_info = {
	.version = PCORE_VERSION(6, 0, 'a'),
	.has_fifo_interface = true,
};

/* Match table for of_platform binding */
static const struct of_device_id cf_axi_dds_of_match[] = {
	{ .compatible = "xlnx,cf-ad9122-core-1.00.a", },
	{ .compatible = "adi,axi-ad9122-6.00.a", .data = &ad9122_6_00_a_info},
	{ .compatible = "xlnx,cf-ad9739a-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9122x2-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9122-core-2.00.a", },
	{ .compatible = "xlnx,axi-dac-4d-2c-1.00.a", },
{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

static int cf_axi_dds_of_probe(struct platform_device *op)
{
	unsigned int expected_version, version;
	const struct axidds_core_info *info;
	const struct of_device_id *id;
	struct cf_axi_dds_state *st;
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	resource_size_t remap_size, phys_addr;
	struct dds_spidev dds_spidev;
	struct cf_axi_converter *conv;
	int ret;

	id = of_match_device(cf_axi_dds_of_match, &op->dev);
	if (!id)
		return -ENODEV;

	info = id->data;

	dev_dbg(dev, "Device Tree Probing \'%s\'\n",
			op->dev.of_node->name);

	/* Defer driver probe until matching spi
	 * converter driver is registered
	 */
	dds_spidev.of_nspi = of_parse_phandle(op->dev.of_node,
						 "spibus-connected", 0);
	if (!dds_spidev.of_nspi) {
		dev_err(&op->dev, "could not find spi node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&spi_bus_type, NULL, &dds_spidev,
			       dds_attach_spi_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(dds_spidev.dev_spi->driver->owner))
		return -ENODEV;
	get_device(dds_spidev.dev_spi);

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev_spi = dds_spidev.dev_spi;

	if (info)
		st->has_fifo_interface = info->has_fifo_interface;

	dev_set_drvdata(dev, indio_dev);

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &st->r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		goto failed1;
	}

	phys_addr = st->r_mem.start;
	remap_size = resource_size(&st->r_mem);
	if (!request_mem_region(phys_addr, remap_size, DRIVER_NAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	/* Fill in configuration data and add them to the list */
	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}

	version = dds_read(st, ADI_REG_VERSION);
	st->dp_disable = dds_read(st, ADI_REG_DAC_DP_DISABLE);

	if (info)
		expected_version = info->version;
	else
		expected_version = PCORE_VERSION(4, 0, 'a');

	if (PCORE_VERSION_MAJOR(version) !=
		PCORE_VERSION_MAJOR(expected_version)) {
		dev_err(&op->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			PCORE_VERSION_MAJOR(expected_version),
			PCORE_VERSION_MINOR(expected_version),
			PCORE_VERSION_LETTER(expected_version),
			PCORE_VERSION_MAJOR(version),
			PCORE_VERSION_MINOR(version),
			PCORE_VERSION_LETTER(version));
		ret = -ENODEV;
		goto failed2;
	}

	conv = to_converter(st->dev_spi);
	if (IS_ERR(conv)) {
		ret = PTR_ERR(conv);
		goto failed3;
	}

	iio_device_set_drvdata(indio_dev, conv);
	conv->indio_dev = indio_dev;
	conv->pcore_sync = cf_axi_dds_sync_frame;
	conv->pcore_set_sed_pattern = cf_axi_dds_set_sed_pattern;

	st->dac_clk = conv->get_data_clk(conv);
	st->chip_info = &cf_axi_dds_chip_info_tbl[conv->id];

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = (st->dp_disable ?
		st->chip_info->num_dp_disable_channels :
		st->chip_info->num_channels);

	st->iio_info = cf_axi_dds_info;
	st->iio_info.attrs = conv->attrs;
	indio_dev->info = &st->iio_info;

	dds_write(st, ADI_REG_RSTN, 0x0);
	dds_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

	dds_write(st, ADI_REG_RATECNTRL, ADI_RATE(1));

	ret = conv->setup(conv);
	if (ret < 0)
		goto failed3;

	dds_write(st, ADI_REG_CNTRL_1, 0);
	dds_write(st, ADI_REG_CNTRL_2,  ADI_DATA_SEL(DATA_SEL_DDS) | ADI_DATA_FORMAT);

	if (!st->dp_disable) {
		cf_axi_dds_default_setup(st, 0, 90000, 40000000, 2);
		cf_axi_dds_default_setup(st, 1, 90000, 40000000, 2);

		if (st->chip_info->num_channels >= 4) {
			cf_axi_dds_default_setup(st, 2, 0, 40000000, 2);
			cf_axi_dds_default_setup(st, 3, 0, 40000000, 2);
		}
	}

	dds_write(st, ADI_REG_CNTRL_1, ADI_ENABLE);
	cf_axi_dds_sync_frame(indio_dev);

	if (!st->dp_disable) {
		st->tx_chan = dma_request_slave_channel(&op->dev, "tx");
		if (!st->tx_chan) {
			dev_err(dev, "failed to find vdma device\n");
			goto failed3;
		}

		cf_axi_dds_configure_buffer(indio_dev);

		ret = iio_buffer_register(indio_dev,
					st->chip_info->buf_channel, 2);
		if (ret)
			goto failed3;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed3;

	dev_info(dev, "Analog Devices CF_AXI_DDS_DDS %s (0x%X) at 0x%08llX mapped"
		" to 0x%p, probed DDS %s\n",
		dds_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER",
		version,
		 (unsigned long long)phys_addr, st->regs, st->chip_info->name);

	return 0;		/* success */

failed3:
	iounmap(st->regs);
failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);
	dev_set_drvdata(dev, NULL);
	iio_device_free(indio_dev);

	return ret;
}

static int cf_axi_dds_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);
	iounmap(st->regs);
	release_mem_region(st->r_mem.start, resource_size(&st->r_mem));

	if (!st->dp_disable) {
		cf_axi_dds_unconfigure_buffer(indio_dev);
		dma_release_channel(st->tx_chan);
	}

	iio_device_free(indio_dev);
	dev_set_drvdata(dev, NULL);

	return 0;
}

static struct platform_driver cf_axi_dds_of_driver = {
	.driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = cf_axi_dds_of_match,
	},
	.probe		= cf_axi_dds_of_probe,
	.remove		= cf_axi_dds_of_remove,
};

module_platform_driver(cf_axi_dds_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices DDS");
MODULE_LICENSE("GPL v2");
