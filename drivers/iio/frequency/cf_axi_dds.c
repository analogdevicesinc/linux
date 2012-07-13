/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012 Analog Devices Inc.
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
#include <linux/dma-mapping.h>
#include <linux/spi/spi.h>
#include <asm/div64.h>

#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#define CF_AXI_DDS_WAVDATA
#include "cf_axi_dds.h"

#define DRIVER_NAME		"cf_axi_dds"

struct dds_spidev {
	struct device_node *of_nspi;
	struct device *dev_spi;
};

static void cf_axi_dds_sync_frame(struct cf_axi_dds_state *st) {
	dds_write(st, CF_AXI_DDS_DDS_FRAME, 0);
	dds_write(st, CF_AXI_DDS_DDS_FRAME, CF_AXI_DDS_DDS_FRAME_SYNC);
}

static void cf_axi_dds_stop(struct cf_axi_dds_state *st) {
	dds_write(st, CF_AXI_DDS_DDS_CTRL, (st->vers_id > 1) ?
	CF_AXI_DDS_DDS_CTRL_DDS_CLK_EN_V2 : CF_AXI_DDS_DDS_CTRL_DDS_CLK_EN_V1);
}

static u32 cf_axi_dds_calc(u32 phase, u32 freq, u32 dac_clk) {

	unsigned long long val64;
	u32 val;

	val64 = (u64) freq * 0xFFFFULL;
	do_div(val64, dac_clk);
	val = ((val64 & 0xFFFF) | 1);

	val64 = (u64) phase * 0xFFFFULL;
	do_div(val64, 360000);
	val |= val64 << 16;

	return val;
}

static void cf_axi_dds_mem_init(struct cf_axi_dds_state *st) {

	u32 i, sample, addr, data;

	sample = 0;
	addr = 0;

	for (i = 0; i < ARRAY_SIZE(cf_axi_dds_ia_data); i++) {
		data = (sample << 24) | (addr << 16);
		dds_write(st, CF_AXI_DDS_DDS_MEM_CTRL, ((0 << 26) | data | cf_axi_dds_ia_data[i]));
		dds_write(st, CF_AXI_DDS_DDS_MEM_CTRL, ((1 << 26) | data | cf_axi_dds_ib_data[i]));
		dds_write(st, CF_AXI_DDS_DDS_MEM_CTRL, ((2 << 26) | data | cf_axi_dds_qa_data[i]));
		dds_write(st, CF_AXI_DDS_DDS_MEM_CTRL, ((3 << 26) | data | cf_axi_dds_qb_data[i]));
		sample++;
		if (sample >= 3) {
		 	addr++;
		 	sample = 0;
		}
	}
}

static const int cf_axi_dds_scale_table[16] = {
	10000, 5000, 2500, 1250, 625, 313, 156,
};

static int cf_axi_dds_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned long long val64;
	unsigned reg;

	switch (m) {
	case 0:
		if (!chan->output) {
			return -EINVAL;
		}
		reg = dds_read(st, CF_AXI_DDS_DDS_CTRL);
		if (st->vers_id > 1) {
			if (reg & 0x2)
				*val = 1;
			else
				*val = 0;

		} else {
			if (reg & (1 << (chan->channel * 2)))
				*val = 1;
			else
				*val = 0;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		reg = dds_read(st, CF_AXI_DDS_DDS_SCALE);
		reg = (reg >> (chan->channel * 4)) & 0xF;
		if (!reg) {
			*val = 1;
			*val2 = 0;
		} else {
			*val = 0;
			*val2 = 1000000 >> reg;
		}
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_FREQUENCY:
		if (!chan->output) {
			*val = st->dac_clk;
			return IIO_VAL_INT;
		}
		reg = dds_read(st, chan->address);
		val64 = (u64)(reg & 0xFFFF) * (u64)st->dac_clk;
		do_div(val64, 0xFFFF);
		*val = val64;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		reg = dds_read(st, chan->address);
		val64 = (u64)(reg >> 16) * 360000ULL;
		do_div(val64, 0xFFFF);
		*val = val64;
		return IIO_VAL_INT;
	}
	return -EINVAL;
}

static int cf_axi_dds_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned long long val64;
	unsigned reg, ctrl_reg;
	int i;

	ctrl_reg = dds_read(st, CF_AXI_DDS_DDS_CTRL);

	switch (mask) {
	case 0:
		if (!chan->output) {
			return -EINVAL;
		}

		if (st->vers_id > 1) {
			if (val)
				ctrl_reg |= 0x3;
			else
				ctrl_reg &= ~(0x2);
		} else {
			if (val)
				ctrl_reg |= 1 << (chan->channel * 2);
			else
				ctrl_reg &= ~(1 << (chan->channel * 2));
		}

		dds_write(st, CF_AXI_DDS_DDS_CTRL, ctrl_reg);
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
		reg = dds_read(st, CF_AXI_DDS_DDS_SCALE);

		reg &= ~(0xF << (chan->channel * 4));
		reg |= (i << (chan->channel * 4));
		dds_write(st, CF_AXI_DDS_DDS_SCALE, reg);
		dds_write(st, CF_AXI_DDS_DDS_CTRL, ctrl_reg);
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (!chan->output) {
			st->dac_clk = val;
			break;
		}
		if (val > (st->dac_clk / 2))
			return -EINVAL;
		cf_axi_dds_stop(st);
		reg = dds_read(st, chan->address);
		reg &= 0xFFFF0000;
		val64 = (u64) val * 0xFFFFULL;
		do_div(val64, st->dac_clk);
		reg |= (val64 & 0xFFFF) | 1;
		dds_write(st, chan->address, reg);
		dds_write(st, CF_AXI_DDS_DDS_CTRL, ctrl_reg);
		break;
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > 360000)
			return -EINVAL;
		cf_axi_dds_stop(st);
		reg = dds_read(st, chan->address);
		reg &= 0x0000FFFF;
		val64 = (u64) val * 0xFFFFULL;
		do_div(val64, 360000);
		reg |= val64 << 16;
		dds_write(st, chan->address, reg);
		dds_write(st, CF_AXI_DDS_DDS_CTRL, ctrl_reg);
		break;
	default:
		return -EINVAL;
	}

	cf_axi_dds_sync_frame(st);

	return 0;
}

static int cf_axi_dds_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	int ret;

	if (IS_ERR(conv))
		return PTR_ERR(conv);

	if (reg > 0xFF)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		ret = conv->write(conv->spi, reg, writeval & 0xFF);
	} else {
		ret = conv->read(conv->spi, reg);
		if (ret < 0)
			return ret;
		*readval = ret;
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_CONST_ATTR(out_altvoltage_scale_available,
		"1.000000 0.500000 0.250000 0.125000 ...");

static struct attribute *cf_axi_dds_attributes[] = {
	&iio_const_attr_out_altvoltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group cf_axi_dds_attribute_group = {
	.attrs = cf_axi_dds_attributes,
};

#define CF_AXI_DDS_DDS_CHAN(_chan, _address, _extend_name)			\
	{ .type = IIO_ALTVOLTAGE,					\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_RAW_SEPARATE_BIT |			\
		       IIO_CHAN_INFO_SCALE_SEPARATE_BIT |		\
	  	       IIO_CHAN_INFO_PHASE_SEPARATE_BIT |		\
		       IIO_CHAN_INFO_FREQUENCY_SEPARATE_BIT,		\
	  .address = _address,						\
	  .output = 1,							\
	  .extend_name = _extend_name,					\
	  }

#define CF_AXI_DDS_DDS_CHAN_CLK_IN(_chan, _extend_name)			\
	{ .type = IIO_ALTVOLTAGE,					\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_FREQUENCY_SEPARATE_BIT,		\
	  .extend_name = _extend_name,					\
	  }

static const struct cf_axi_dds_chip_info cf_axi_dds_chip_info_tbl[] = {
	[ID_AD9122] = {
		.name = "AD9122",
		.channel[0] = CF_AXI_DDS_DDS_CHAN(0, CF_AXI_DDS_DDS_1A_OUTPUT_CTRL, "1A"),
		.channel[1] = CF_AXI_DDS_DDS_CHAN(1, CF_AXI_DDS_DDS_1B_OUTPUT_CTRL, "1B"),
		.channel[2] = CF_AXI_DDS_DDS_CHAN(2, CF_AXI_DDS_DDS_2A_OUTPUT_CTRL, "2A"),
		.channel[3] = CF_AXI_DDS_DDS_CHAN(3, CF_AXI_DDS_DDS_2B_OUTPUT_CTRL, "2B"),
		.channel[4] = CF_AXI_DDS_DDS_CHAN_CLK_IN(0, "DAC_CLK"),
	},
	[ID_AD9739A] = {
		.name = "AD9739A",
		.channel[0] = CF_AXI_DDS_DDS_CHAN(0, CF_AXI_DDS_DDS_1A_OUTPUT_CTRL, "1A"),
		.channel[1] = CF_AXI_DDS_DDS_CHAN(1, CF_AXI_DDS_DDS_1B_OUTPUT_CTRL, "1B"),
		.channel[2] = CF_AXI_DDS_DDS_CHAN(2, CF_AXI_DDS_DDS_2A_OUTPUT_CTRL, "2A"),
		.channel[3] = CF_AXI_DDS_DDS_CHAN(3, CF_AXI_DDS_DDS_2B_OUTPUT_CTRL, "2B"),
		.channel[4] = CF_AXI_DDS_DDS_CHAN_CLK_IN(0, "DAC_CLK"),
	},
};

static const struct iio_info cf_axi_dds_info = {
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
	.attrs = &cf_axi_dds_attribute_group,
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

/* Match table for of_platform binding */
static const struct of_device_id cf_axi_dds_of_match[] __devinitconst = {
	{ .compatible = "xlnx,cf-ad9122-core-1.00.a", .data = (void*) 1},
	{ .compatible = "xlnx,cf-ad9739a-core-1.00.a", .data = (void*) 1},
	{ .compatible = "xlnx,cf-ad9122x2-core-1.00.a", .data = (void*) 1},
	{ .compatible = "xlnx,cf-ad9122-core-2.00.a", .data = (void*) 2},
	{ .compatible = "xlnx,axi-dac-4d-2c-1.00.a", .data = (void*) 2},
{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

static int __devinit cf_axi_dds_of_probe(struct platform_device *op)
{
	struct cf_axi_dds_state *st;
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	resource_size_t remap_size, phys_addr;
	struct dds_spidev dds_spidev;
	struct cf_axi_dds_converter *conv;
	int ret;

	const struct of_device_id *of_id =
			of_match_device(cf_axi_dds_of_match, &op->dev);

	dev_info(dev, "Device Tree Probing \'%s\'\n",
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

	dev_set_drvdata(dev, indio_dev);

	if (of_id && of_id->data)
		st->vers_id = (unsigned) of_id->data;
	else
		goto failed1;

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

	ret = of_property_read_u32(op->dev.of_node,
				   "dac-sample-frequency",
				   &st->dac_clk);
	if (ret)
		st->dac_clk = 491520000;

	conv = to_converter(st->dev_spi);
	if (IS_ERR(conv)) {
		ret = PTR_ERR(conv);
		goto failed3;
	}

	ret = conv->setup(conv->spi, 0);
	if (ret) {
		dev_err(&op->dev, "failed to setup spi device\n");
		goto failed3;
	}

	st->chip_info = &cf_axi_dds_chip_info_tbl[conv->id];

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 5;
	indio_dev->info = &cf_axi_dds_info;

	cf_axi_dds_mem_init(st);

	dds_write(st, CF_AXI_DDS_DDS_CTRL, 0x0);
	dds_write(st, CF_AXI_DDS_DDS_SCALE, 0x1111); /* divide by 4 */
	dds_write(st, CF_AXI_DDS_DDS_1A_OUTPUT_CTRL,
		  cf_axi_dds_calc(90000, 40000000, st->dac_clk));
	dds_write(st, CF_AXI_DDS_DDS_1B_OUTPUT_CTRL,
		  cf_axi_dds_calc(90000, 40000000, st->dac_clk));
	dds_write(st, CF_AXI_DDS_DDS_2A_OUTPUT_CTRL,
		  cf_axi_dds_calc( 0, 40000000, st->dac_clk));
	dds_write(st, CF_AXI_DDS_DDS_2B_OUTPUT_CTRL,
		  cf_axi_dds_calc( 0, 40000000, st->dac_clk));

	if (st->vers_id > 1)
		dds_write(st, CF_AXI_DDS_DDS_CTRL, 0x3); /* clk, dds enable & ddsx select */
	else
		dds_write(st, CF_AXI_DDS_DDS_CTRL, 0x1ff); /* clk, dds enable & ddsx select */

	cf_axi_dds_sync_frame(st);

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed3;

	dev_info(dev, "Analog Devices CF_AXI_DDS_DDS %s (0x%X) at 0x%08llX mapped"
		" to 0x%p, probed DDS %s\n",
		(dds_read(st, CF_AXI_DDS_DDS_PCORE_IDENT) &
		CF_AXI_DDS_DDS_PCORE_IDENT_SLAVE) ? "SLAVE" : "MASTER",
		dds_read(st, CF_AXI_DDS_DDS_VERSION_ID),
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

static int __devexit cf_axi_dds_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);
	iounmap(st->regs);
	release_mem_region(st->r_mem.start, resource_size(&st->r_mem));

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
	.remove		= __devexit_p(cf_axi_dds_of_remove),
};

module_platform_driver(cf_axi_dds_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices DDS");
MODULE_LICENSE("GPL v2");
