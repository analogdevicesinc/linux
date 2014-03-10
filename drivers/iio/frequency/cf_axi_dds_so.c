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

#include <linux/clk.h>
#include <linux/clkdev.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_dds.h"
#include "ad9122.h"

#define DRIVER_NAME		"cf_axi_dds_so"

struct axidds_core_info {
	unsigned int version;
	bool has_fifo_interface;
	const struct cf_axi_dds_chip_info *chip_info;
};

static int cf_axi_dds_sync_frame(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	dds_write(st, ADI_REG_FRAME, 0);
	dds_write(st, ADI_REG_FRAME, ADI_FRAME);

	return 0;
}

static void cf_axi_dds_stop(struct cf_axi_dds_state *st) {
	dds_write(st, ADI_REG_CNTRL_1, 0);
}


static int cf_axi_dds_rate_change(struct notifier_block *nb,
	unsigned long flags, void *data)
{
	struct clk_notifier_data *cnd = data;
	struct cf_axi_dds_state *st =
		container_of(nb, struct cf_axi_dds_state, clk_nb);
	unsigned reg, i;
	unsigned long long val64;

	/* Temp Workaround: stop PCORE while we reset the sink */
	if (flags == PRE_RATE_CHANGE && cnd->new_rate == -EINVAL)
		cf_axi_dds_stop(st);

	st->dac_clk = cnd->new_rate;

	if (flags == POST_RATE_CHANGE) {
		st->dac_clk = cnd->new_rate;
		cf_axi_dds_stop(st);

		for (i = 0; i < st->chip_info->num_channels; i++) {
			reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(i));
			reg &= ~ADI_DDS_INCR(~0);
			val64 = (u64) st->cached_freq[i] * 0xFFFFULL;
			do_div(val64, st->dac_clk);
			reg |= ADI_DDS_INCR(val64) | 1;
			dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(i), reg);
		}
		dds_write(st, ADI_REG_CNTRL_1, st->enable ? ADI_ENABLE : 0);
		cf_axi_dds_sync_frame(st->indio_dev);
	}

	return NOTIFY_OK;
}

static int cf_axi_dds_default_setup(struct cf_axi_dds_state *st, u32 chan,
				    u32 phase, u32 freq, u32 scale) {

	unsigned long long val64;
	u32 val;

	st->cached_freq[chan] = freq;

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
		*val = st->dac_clk = clk_get_rate(st->clk);
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	default:
		ret = -EINVAL;
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

		st->enable = !!val;
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
		dds_write(st, ADI_REG_CNTRL_1, 0);
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

		st->cached_freq[chan->channel] = val;
		dds_write(st, ADI_REG_CNTRL_1, 0);
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
// 	case IIO_CHAN_INFO_SAMP_FREQ:
//
// 		dds_write(st, ADI_REG_CNTRL_1, 0);
// 		/* FIXME */
// 		ret = cf_axi_dds_sync_frame(indio_dev);
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int cf_axi_dds_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	int ret;


	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		dds_write(st, reg & 0xFFFF, writeval);
	} else {
		ret = dds_read(st, reg & 0xFFFF);
		*readval = ret;
	}
	mutex_unlock(&indio_dev->mlock);

	return 0;
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
	[0] = {
		.name = "AD9361",
		.channel[0] = CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		.channel[1] = CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		.channel[2] = CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		.channel[3] = CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
		.channel[4] = CF_AXI_DDS_CHAN(4, 0, "TX2_I_F1"),
		.channel[5] = CF_AXI_DDS_CHAN(5, 0, "TX2_I_F2"),
		.channel[6] = CF_AXI_DDS_CHAN(6, 0, "TX2_Q_F1"),
		.channel[7] = CF_AXI_DDS_CHAN(7, 0, "TX2_Q_F2"),
		.buf_channel[0] = CF_AXI_DDS_CHAN_BUF(0),
		.buf_channel[1] = CF_AXI_DDS_CHAN_BUF(1),
		.num_channels = 8,
		.num_buf_channels = 2,
	},
};

struct axidds_core_info ad9361_1_00_a_info = {
	.version = PCORE_VERSION(4, 0, 'a'),
	.chip_info = cf_axi_dds_chip_info_tbl,
};

struct axidds_core_info ad9361_6_00_a_info = {
	.version = PCORE_VERSION(6, 0, 'a'),
	.has_fifo_interface = true,
	.chip_info = cf_axi_dds_chip_info_tbl,
};

static const struct iio_info cf_axi_dds_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
};


/* Match table for of_platform binding */
static const struct of_device_id cf_axi_dds_of_match[] = {
	{ .compatible = "xlnx,axi-ad9361-dds-1.00.a", .data = &ad9361_1_00_a_info },
	{ .compatible = "adi,axi-ad9361-dds-6.00.a", .data = &ad9361_6_00_a_info },
{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

static int cf_axi_dds_of_probe(struct platform_device *op)
{
	const struct of_device_id *id;
	const struct axidds_core_info *info;
	struct cf_axi_dds_state *st;
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	resource_size_t remap_size, phys_addr;
	struct clk *clk = NULL;
	unsigned int version;
	int ret;
	u32 ctrl_2, rate;

	id = of_match_device(cf_axi_dds_of_match, &op->dev);
	if (!id)
		return -ENODEV;

	info = id->data;
	if (!info)
		return -ENODEV;

	dev_dbg(dev, "Device Tree Probing \'%s\'\n",
			op->dev.of_node->name);

	clk = devm_clk_get(dev, "sampl_clk");
	if (IS_ERR(clk)) {
		return -EPROBE_DEFER;
	}

	ret = clk_prepare_enable(clk);
	if (ret < 0)
		return ret;

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->clk = clk;
	st->indio_dev = indio_dev;
	st->chip_info = info->chip_info;
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

	if (PCORE_VERSION_MAJOR(version) !=
		PCORE_VERSION_MAJOR(info->version)) {
		dev_err(&op->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			PCORE_VERSION_MAJOR(info->version),
			PCORE_VERSION_MINOR(info->version),
			PCORE_VERSION_LETTER(info->version),
			PCORE_VERSION_MAJOR(version),
			PCORE_VERSION_MINOR(version),
			PCORE_VERSION_LETTER(version));
		ret = -ENODEV;
		goto failed2;
	}

	st->dac_clk = clk_get_rate(clk);

	st->clk_nb.notifier_call = cf_axi_dds_rate_change;
	clk_notifier_register(clk, &st->clk_nb);

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = (st->dp_disable ? 0 :
		st->chip_info->num_channels);

	st->iio_info = cf_axi_dds_info;
	indio_dev->info = &st->iio_info;

	ctrl_2 = (of_property_read_bool(op->dev.of_node,
			"adi,axi-dds-parity-enable") ? ADI_PAR_ENB : 0) |
			(of_property_read_bool(op->dev.of_node,
			"adi,axi-dds-parity-type-odd") ? ADI_PAR_TYPE : 0) |
			(of_property_read_bool(op->dev.of_node,
			"adi,axi-dds-1-rf-channel") ? ADI_R1_MODE : 0);

	rate = 3;
	of_property_read_u32(op->dev.of_node, "adi,axi-dds-rate", &rate);

	dds_write(st, ADI_REG_RSTN, 0x0);
	dds_write(st, ADI_REG_RSTN, ADI_RSTN);

	dds_write(st, ADI_REG_RATECNTRL, ADI_RATE(rate));

	dds_write(st, ADI_REG_CNTRL_1, 0);
	dds_write(st, ADI_REG_CNTRL_2,  ctrl_2 | ADI_DATA_SEL(DATA_SEL_DDS));

	if (!st->dp_disable) {
		cf_axi_dds_default_setup(st, 0, 90000, 1000000, 4);
		cf_axi_dds_default_setup(st, 1, 90000, 1000000, 4);

		if (st->chip_info->num_channels > 2) {
			cf_axi_dds_default_setup(st, 2, 0, 1000000, 4);
			cf_axi_dds_default_setup(st, 3, 0, 1000000, 4);
		}

		if (st->chip_info->num_channels >= 8) {
			cf_axi_dds_default_setup(st, 4, 90000, 1000000, 4);
			cf_axi_dds_default_setup(st, 5, 90000, 1000000, 4);
			cf_axi_dds_default_setup(st, 6, 0, 1000000, 4);
			cf_axi_dds_default_setup(st, 7, 0, 1000000, 4);

		}
	}
	st->enable = true;
	dds_write(st, ADI_REG_CNTRL_1, ADI_ENABLE);

	cf_axi_dds_sync_frame(indio_dev);

	if (!st->dp_disable) {
		st->tx_chan = of_dma_request_slave_channel(op->dev.of_node, "tx");
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
