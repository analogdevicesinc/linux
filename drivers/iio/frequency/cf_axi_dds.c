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
#include <linux/clk.h>

#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_dds.h"
#include "ad9122.h"

static int cf_axi_dds_sync_frame(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv;
	int stat;
	static int retry = 0;

	mdelay(10); /* Wait until clocks are stable */

	dds_write(st, ADI_REG_FRAME, 0);
	dds_write(st, ADI_REG_FRAME, ADI_FRAME);

	if (st->standalone)
		return 0;

	conv = to_converter(st->dev_spi);
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

	return 0;
}

static void cf_axi_dds_stop(struct cf_axi_dds_state *st)
{
	dds_write(st, ADI_REG_CNTRL_1, 0);
}

static int cf_axi_dds_rate_change(struct notifier_block *nb,
	unsigned long flags, void *data)
{
	struct clk_notifier_data *cnd = data;
	struct cf_axi_dds_state *st =
		container_of(nb, struct cf_axi_dds_state, clk_nb);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	unsigned reg, i;
	unsigned long long val64;

	/* Temp Workaround: stop PCORE while we reset the sink */
	if (flags == PRE_RATE_CHANGE && cnd->new_rate == -EINVAL)
		cf_axi_dds_stop(st);

	st->dac_clk = cnd->new_rate;

	if (flags == POST_RATE_CHANGE) {
		st->dac_clk = cnd->new_rate;
		cf_axi_dds_stop(st);

		for (i = 0; i < st->chip_info->num_dds_channels; i++) {
			reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(i));
			reg &= ~ADI_DDS_INCR(~0);
			val64 = (u64) st->cached_freq[i] * 0xFFFFULL;
			do_div(val64, st->dac_clk);
			reg |= ADI_DDS_INCR(val64) | 1;
			dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(i), reg);
		}
		dds_write(st, ADI_REG_CNTRL_1, st->enable ? ADI_ENABLE : 0);
		cf_axi_dds_sync_frame(indio_dev);
	}

	return NOTIFY_OK;
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
	struct cf_axi_converter *conv;
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
		if (st->standalone) {
			*val = st->dac_clk = clk_get_rate(st->clk);
		} else {
			conv = to_converter(st->dev_spi);
			if (!conv->get_data_clk) {
				ret = -ENODEV;
				break;
			}
			*val = st->dac_clk = conv->get_data_clk(conv);
		}
		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	default:
		if (!st->standalone) {
			conv = to_converter(st->dev_spi);
			ret = conv->read_raw(indio_dev, chan, val, val2, m);
		} else {
			ret = -EINVAL;
		}
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
	struct cf_axi_converter *conv = to_converter(st->dev_spi);
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

		st->cached_freq[chan->channel] = val;
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
		if (!conv) {
			ret = -EINVAL;
			break;
		}
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
		if (conv)
			ret = conv->write_raw(indio_dev, chan, val, val2, mask);
		else
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
	struct cf_axi_converter *conv = to_converter(st->dev_spi);
	int ret;

	if ((reg & ~DEBUGFS_DRA_PCORE_REG_MAGIC) > 0xFF)
		return -EINVAL;

	mutex_lock(&indio_dev->mlock);
	if (readval == NULL) {
		if ((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) ||
			st->standalone) {
			dds_write(st, reg & 0xFFFF, writeval);
			ret = 0;
		} else {
			if (IS_ERR(conv))
				ret  = PTR_ERR(conv);
			else
				ret = conv->write(conv->spi, reg, writeval & 0xFF);
		}
	} else {
		if ((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) ||
			st->standalone) {
			ret = dds_read(st, reg & 0xFFFF);
		} else {
			if (IS_ERR(conv))
				ret  = PTR_ERR(conv);
			else
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

#define CF_AXI_DDS_CHAN(_chan, _address, _extend_name) { \
	.type = IIO_ALTVOLTAGE,	\
	.indexed = 1, \
	.channel = _chan, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | \
		BIT(IIO_CHAN_INFO_SCALE) | \
		BIT(IIO_CHAN_INFO_PHASE) | \
		BIT(IIO_CHAN_INFO_FREQUENCY), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.address = _address, \
	.output = 1, \
	.extend_name = _extend_name, \
	.ext_info = cf_axi_dds_ext_info, \
	.scan_index = -1, \
}

#define CF_AXI_DDS_CHAN_BUF(_chan) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _chan, \
	.output = 1, \
	.scan_index = _chan, \
	.scan_type = IIO_ST('s', 16, 16, 0), \
}

static const struct cf_axi_dds_chip_info cf_axi_dds_chip_info_tbl[] = {
	[ID_AD9122] = {
		.name = "AAD9122",
		.channel = {
			{
				.type = IIO_TEMP,
				.indexed = 1,
				.channel = 0,
				.info_mask_separate =
					BIT(IIO_CHAN_INFO_PROCESSED) |
					BIT(IIO_CHAN_INFO_CALIBBIAS),
			},
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN_BUF(1),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
			CF_AXI_DDS_CHAN(2, 0, "2A"),
			CF_AXI_DDS_CHAN(3, 0, "2B"),
		},
		.num_channels = 7,
		.num_dp_disable_channels = 3,
		.num_dds_channels = 4,
	},
	[ID_AD9739A] = {
		.name = "AD9739A",
		.channel = {
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
		},
		.num_channels = 3,
		.num_dds_channels = 2,
	},
};

static const struct cf_axi_dds_chip_info cf_axi_dds_chip_info_ad9361 = {
	.name = "AD9361",
	.channel = {
		CF_AXI_DDS_CHAN_BUF(0),
		CF_AXI_DDS_CHAN_BUF(1),
		CF_AXI_DDS_CHAN_BUF(2),
		CF_AXI_DDS_CHAN_BUF(3),
		CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
		CF_AXI_DDS_CHAN(4, 0, "TX2_I_F1"),
		CF_AXI_DDS_CHAN(5, 0, "TX2_I_F2"),
		CF_AXI_DDS_CHAN(6, 0, "TX2_Q_F1"),
		CF_AXI_DDS_CHAN(7, 0, "TX2_Q_F2"),
	},
	.num_channels = 12,
	.num_dds_channels = 8,
};

static const struct iio_info cf_axi_dds_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
};

static int dds_converter_match(struct device *dev, void *data)
{
	return dev->driver && dev->of_node == data;
}

static struct device *dds_converter_find(struct device *dev)
{
	struct device_node *conv_of;
	struct device *conv_dev;

	conv_of = of_parse_phandle(dev->of_node, "spibus-connected", 0);
	if (!conv_of)
		return ERR_PTR(-ENODEV);

	conv_dev = bus_find_device(&spi_bus_type, NULL, conv_of, dds_converter_match);
	of_node_put(conv_of);
	if (!conv_dev)
		return ERR_PTR(-EPROBE_DEFER);

	return conv_dev;
}

static void dds_converter_put(struct device *conv_dev)
{
	put_device(conv_dev);
}

struct axidds_core_info {
	unsigned int version;
	bool has_fifo_interface;
	bool standalone;
	const struct cf_axi_dds_chip_info *chip_info;
	unsigned int data_format;
	unsigned int rate;
};

static const struct axidds_core_info ad9122_6_00_a_info = {
	.version = PCORE_VERSION(6, 0, 'a'),
	.has_fifo_interface = true,
	.rate = 1,
	.data_format = ADI_DATA_FORMAT,
};

static const struct axidds_core_info ad9361_1_00_a_info = {
	.version = PCORE_VERSION(4, 0, 'a'),
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9361,
};

static const struct axidds_core_info ad9361_6_00_a_info = {
	.version = PCORE_VERSION(6, 0, 'a'),
	.has_fifo_interface = true,
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9361,
};

static const struct of_device_id cf_axi_dds_of_match[] = {
	{ .compatible = "xlnx,cf-ad9122-core-1.00.a", },
	{ .compatible = "adi,axi-ad9122-6.00.a", .data = &ad9122_6_00_a_info},
	{ .compatible = "xlnx,cf-ad9739a-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9122x2-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9122-core-2.00.a", },
	{ .compatible = "xlnx,axi-dac-4d-2c-1.00.a", },
	{
	    .compatible = "xlnx,axi-ad9361-dds-1.00.a",
	    .data = &ad9361_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9361-dds-6.00.a",
	    .data = &ad9361_6_00_a_info,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

static int cf_axi_dds_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	unsigned int expected_version, version;
	struct cf_axi_converter *conv = NULL;
	const struct axidds_core_info *info;
	const struct of_device_id *id;
	struct cf_axi_dds_state *st;
	struct iio_dev *indio_dev;
	struct resource *res;
	unsigned int ctrl_2;
	unsigned int rate;
	int ret;

	id = of_match_device(cf_axi_dds_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	info = id->data;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = iio_device_alloc(sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!st->regs) {
		ret = -ENOMEM;
		goto err_iio_device_free;
	}

	if (info && info->standalone) {
		st->clk = devm_clk_get(&pdev->dev, "sampl_clk");
		if (IS_ERR(st->clk)) {
			ret = PTR_ERR(st->clk);
			goto err_iio_device_free;
		}

		ret = clk_prepare_enable(st->clk);
		if (ret < 0)
			goto err_iio_device_free;

		st->dac_clk = clk_get_rate(st->clk);

		st->clk_nb.notifier_call = cf_axi_dds_rate_change;
		clk_notifier_register(st->clk, &st->clk_nb);

		st->chip_info = info->chip_info;
	} else {
		st->dev_spi = dds_converter_find(&pdev->dev);
		if (IS_ERR(st->dev_spi)) {
			ret = PTR_ERR(st->dev_spi);
			goto err_iio_device_free;
		}

		conv = to_converter(st->dev_spi);
		if (IS_ERR(conv)) {
			ret = PTR_ERR(conv);
			goto err_converter_put;
		}

		iio_device_set_drvdata(indio_dev, conv);
		conv->indio_dev = indio_dev;
		conv->pcore_sync = cf_axi_dds_sync_frame;
		conv->pcore_set_sed_pattern = cf_axi_dds_set_sed_pattern;

		st->dac_clk = conv->get_data_clk(conv);

		st->chip_info = &cf_axi_dds_chip_info_tbl[conv->id];
	}

	if (info)
		st->has_fifo_interface = info->has_fifo_interface;
	if (info)
		st->standalone = info->standalone;

	version = dds_read(st, ADI_REG_VERSION);
	st->dp_disable = dds_read(st, ADI_REG_DAC_DP_DISABLE);

	if (info)
		expected_version = info->version;
	else
		expected_version = PCORE_VERSION(4, 0, 'a');

	if (PCORE_VERSION_MAJOR(version) !=
		PCORE_VERSION_MAJOR(expected_version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			PCORE_VERSION_MAJOR(expected_version),
			PCORE_VERSION_MINOR(expected_version),
			PCORE_VERSION_LETTER(expected_version),
			PCORE_VERSION_MAJOR(version),
			PCORE_VERSION_MINOR(version),
			PCORE_VERSION_LETTER(version));
		ret = -ENODEV;
		goto err_converter_put;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = np->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = (st->dp_disable ?
		st->chip_info->num_dp_disable_channels :
		st->chip_info->num_channels);

	st->iio_info = cf_axi_dds_info;
	if (conv)
		st->iio_info.attrs = conv->attrs;
	indio_dev->info = &st->iio_info;

	dds_write(st, ADI_REG_RSTN, 0x0);
	dds_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

	if (info)
		rate = info->rate;
	else
		rate = 1;

	dds_write(st, ADI_REG_RATECNTRL, ADI_RATE(rate));

	if (conv) {
		ret = conv->setup(conv);
		if (ret < 0)
			goto err_converter_put;
	}

	ctrl_2 = 0;
	if (of_property_read_bool(np, "adi,axi-dds-parity-enable"))
		ctrl_2 |= ADI_PAR_ENB;
	if (of_property_read_bool(np, "adi,axi-dds-parity-type-odd"))
		ctrl_2 |= ADI_PAR_TYPE;
	if (of_property_read_bool(np, "adi,axi-dds-1-rf-channel"))
		ctrl_2 |= ADI_R1_MODE;

	if (info)
		ctrl_2 |= info->data_format;
	else
		ctrl_2 |= ADI_DATA_FORMAT;

	dds_write(st, ADI_REG_CNTRL_1, 0);
	dds_write(st, ADI_REG_CNTRL_2,  ctrl_2 | ADI_DATA_SEL(DATA_SEL_DDS));

	if (!st->dp_disable) {
		cf_axi_dds_default_setup(st, 0, 90000, 40000000, 2);
		cf_axi_dds_default_setup(st, 1, 90000, 40000000, 2);

		if (st->chip_info->num_dds_channels >= 4) {
			cf_axi_dds_default_setup(st, 2, 0, 40000000, 2);
			cf_axi_dds_default_setup(st, 3, 0, 40000000, 2);
		}

		if (st->chip_info->num_dds_channels >= 8) {
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
		ret = cf_axi_dds_configure_buffer(indio_dev);
		if (ret)
			goto err_converter_put;

		ret = iio_buffer_register(indio_dev, st->chip_info->channel,
			st->chip_info->num_channels);
		if (ret)
			goto err_unconfigure_buffer;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_buffer;

	dev_info(&pdev->dev, "Analog Devices CF_AXI_DDS_DDS %s (0x%X) at 0x%08llX mapped"
		" to 0x%p, probed DDS %s\n",
		dds_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER",
		version,
		 (unsigned long long)res->start, st->regs, st->chip_info->name);

	platform_set_drvdata(pdev, indio_dev);

	return 0;

err_unconfigure_buffer:
	cf_axi_dds_unconfigure_buffer(indio_dev);
err_converter_put:
	if (st->dev_spi)
		dds_converter_put(st->dev_spi);
	if (st->clk)
		clk_disable_unprepare(st->clk);
err_iio_device_free:
	iio_device_free(indio_dev);

	return ret;
}

static int cf_axi_dds_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	cf_axi_dds_unconfigure_buffer(indio_dev);
	if (st->dev_spi)
		dds_converter_put(st->dev_spi);
	if (st->clk)
		clk_disable_unprepare(st->clk);
	iio_device_free(indio_dev);

	return 0;
}

static struct platform_driver cf_axi_dds_driver = {
	.driver = {
		.name = "cf_axi_dds",
		.owner = THIS_MODULE,
		.of_match_table = cf_axi_dds_of_match,
	},
	.probe		= cf_axi_dds_probe,
	.remove		= cf_axi_dds_remove,
};
module_platform_driver(cf_axi_dds_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices DDS");
MODULE_LICENSE("GPL v2");
