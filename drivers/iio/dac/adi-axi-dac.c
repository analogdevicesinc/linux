// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Generic AXI DAC IP core
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_dac_ip
 *
 * Copyright 2012-2021 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dmaengine.h>

#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/dac/adi-axi-dac.h>

/**
 * Register definitions:
 *   https://wiki.analog.com/resources/fpga/docs/axi_dac_ip#register_map
 */

/* DAC controls */

#define ADI_AXI_REG_RSTN			0x0040
#define   ADI_AXI_REG_RSTN_CE_N			BIT(2)
#define   ADI_AXI_REG_RSTN_MMCM_RSTN		BIT(1)
#define   ADI_AXI_REG_RSTN_RSTN			BIT(0)

#define ADI_AXI_REG_CTRL1			0x0044
#define   ADI_AXI_REG_CTRL1_SYNC		BIT(0)

#define ADI_AXI_REG_CTRL2			0x0048
#define   ADI_AXI_REG_CTRL2_PAR_TYPE		BIT(7)
#define   ADI_AXI_REG_CTRL2_PAR_EN		BIT(6)
#define   ADI_AXI_REG_CTRL2_R1_MODE		BIT(5)
#define   ADI_AXI_REG_CTRL2_FMT_SIGNEXT		BIT(4)

#define ADI_AXI_REG_CTRL2_DEFAULTS		\
	(ADI_AXI_REG_CTRL2_FMT_SIGNEXT)

#define ADI_AXI_REG_RATE_CTRL			0x004C
#define   ADI_AXI_REG_RATE_CTRL_MSK		GENMASK(7, 0)
#define   ADI_AXI_REG_RATE_CTRL_SET(x)		FIELD_PREP(ADI_AXI_REG_RATE_CTRL_MSK, x)
#define   ADI_AXI_REG_RATE_CTRL_GET(x)		FIELD_GET(ADI_AXI_REG_RATE_CTRL_MSK, x)

#define ADI_AXI_REG_UI_STATUS			0x0088
#define   ADI_AXI_REG_UI_STATUS_OVF		BIT(1)
#define   ADI_AXI_REG_UI_STATUS_UNF		BIT(0)

/* DAC Channel controls */

#define ADI_AXI_REG_CHAN_CNTRL_1(c)		(0x0400 + (c) * 0x40)
#define   ADI_AXI_REG_SCALE_1_MSK		GENMASK(15, 0)
#define   ADI_AXI_REG_SCALE_1_SET(x)		FIELD_PREP(ADI_AXI_REG_SCALE_1_MSK, x)
#define   ADI_AXI_REG_SCALE_1_GET(x)		FIELD_GET(ADI_AXI_REG_SCALE_1_MSK, x)

#define ADI_AXI_REG_CHAN_CNTRL_2(c)		(0x0404 + (c) * 0x40)
#define   ADI_AXI_REG_DDS_INIT_MSK		GENMASK(31, 16)
#define   ADI_AXI_REG_DDS_INIT_SET(x)		FIELD_PREP(ADI_AXI_REG_DDS_INIT_MSK, x)
#define   ADI_AXI_REG_DDS_INIT_GET(x)		FIELD_GET(ADI_AXI_REG_DDS_INIT_MSK, x)
#define   ADI_AXI_REG_DDS_INCR_MSK		GENMASK(15, 0)
#define   ADI_AXI_REG_DDS_INCR_SET(x)		FIELD_PREP(ADI_AXI_REG_DDS_INCR_MSK, x)
#define   ADI_AXI_REG_DDS_INCR_GET(x)		FIELD_GET(ADI_AXI_REG_DDS_INCR_MSK, x)

#define ADI_AXI_REG_CHAN_DATA_SEL(c)		(0x0418 + (c) * 0x40)
#define   ADI_AXI_REG_DATA_SEL_MSK		GENMASK(3, 0)
#define   ADI_AXI_REG_DATA_SEL_SET(x)		FIELD_PREP(ADI_AXI_REG_DATA_SEL_MSK, x)
#define   ADI_AXI_REG_DATA_SEL_GET(x)		FIELD_GET(ADI_AXI_REG_DATA_SEL_MSK, x)

enum adi_axi_dac_data_sel {
	AXI_DAC_DATA_SEL_DDS,
	AXI_DAC_DATA_SEL_SED,
	AXI_DAC_DATA_SEL_DMA,
	AXI_DAC_DATA_SEL_ZERO,	/* OUTPUT 0 */
	AXI_DAC_DATA_SEL_PN7,
	AXI_DAC_DATA_SEL_PN15,
	AXI_DAC_DATA_SEL_PN23,
	AXI_DAC_DATA_SEL_PN31,
	AXI_DAC_DATA_SEL_LB,	/* loopback data (ADC) */
	AXI_DAC_DATA_SEL_PNXX,	/* (Device specific) */
};

#define ADI_AXI_REG_CHAN_IQCOR_CTRL(c)		(0x041C + (c) * 0x40)
#define   ADI_AXI_REG_IQCOR_COEFF_1_MSK		GENMASK(31, 16)
#define   ADI_AXI_REG_IQCOR_COEFF_1_SET(x)	FIELD_PREP(ADI_AXI_REG_IQCOR_COEFF_1_MSK, x)
#define   ADI_AXI_REG_IQCOR_COEFF_1_GET(x)	FIELD_GET(ADI_AXI_REG_IQCOR_COEFF_1_MSK, x)
#define   ADI_AXI_REG_IQCOR_COEFF_2_MSK		GENMASK(15, 0)
#define   ADI_AXI_REG_IQCOR_COEFF_2_SET(x)	FIELD_PREP(ADI_AXI_REG_IQCOR_COEFF_2_MSK, x)
#define   ADI_AXI_REG_IQCOR_COEFF_2_GET(x)	FIELD_GET(ADI_AXI_REG_IQCOR_COEFF_2_MSK, x)

/* IQ correction format is 1.1.14 (sign, integer and fractional bits) */
#define IQCOR_INT_1				0x4000UL
#define IQCOR_SIGN_BIT				BIT(15)

#define ADI_AXI_IQCOR_COEFF_1_INT_1		ADI_AXI_REG_IQCOR_COEFF_1_SET(IQCOR_INT_1)
#define ADI_AXI_IQCOR_COEFF_2_INT_1		ADI_AXI_REG_IQCOR_COEFF_2_SET(IQCOR_INT_1)

#define ADI_AXI_DEFAULT_SCALE			0x1000	/* 0.250 */
#define ADI_AXI_DEFAULT_FREQUENCY		40000000

struct adi_axi_dac_core_info {
	unsigned int				version;
};

struct adi_axi_dac_state {
	struct mutex				lock;

	struct adi_axi_dac_client		*client;
	void __iomem				*regs;
};

struct adi_axi_dac_client {
	struct list_head			entry;
	struct adi_axi_dac_conv			conv;
	struct adi_axi_dac_state		*state;
	struct device				*dev;
	const struct adi_axi_dac_core_info	*info;
};

static LIST_HEAD(registered_clients);
static DEFINE_MUTEX(registered_clients_lock);

static void adi_axi_dac_data_sel_set(struct adi_axi_dac_state *st, int chan,
				     enum adi_axi_dac_data_sel sel);

static struct adi_axi_dac_client *conv_to_client(struct adi_axi_dac_conv *conv)
{
	return container_of(conv, struct adi_axi_dac_client, conv);
}

void *adi_axi_dac_conv_priv(struct adi_axi_dac_conv *conv)
{
	struct adi_axi_dac_client *cl = conv_to_client(conv);

	return (char *)cl + ALIGN(sizeof(struct adi_axi_dac_client), IIO_ALIGN);
}
EXPORT_SYMBOL_GPL(adi_axi_dac_conv_priv);

static void adi_axi_dac_write(struct adi_axi_dac_state *st,
			      unsigned int reg,
			      unsigned int val)
{
	iowrite32(val, st->regs + reg);
}

static unsigned int adi_axi_dac_read(struct adi_axi_dac_state *st,
				     unsigned int reg)
{
	return ioread32(st->regs + reg);
}

static int adi_axi_dac_buffer_state_set(struct iio_dev *indio_dev, bool en)
{
	struct adi_axi_dac_state *st = iio_priv(indio_dev);

	if (!en) {
		adi_axi_dac_data_sel_set(st, -1, AXI_DAC_DATA_SEL_DDS);
		return 0;
	}

	adi_axi_dac_write(st, ADI_AXI_REG_UI_STATUS,
			  ADI_AXI_REG_UI_STATUS_OVF |
			  ADI_AXI_REG_UI_STATUS_UNF);

	adi_axi_dac_write(st, ADI_AXI_REG_CTRL1, ADI_AXI_REG_CTRL1_SYNC);

	return 0;
}

static int adi_axi_dac_buffer_preenable(struct iio_dev *indio_dev)
{
	return adi_axi_dac_buffer_state_set(indio_dev, true);
}

static int adi_axi_dac_buffer_postdisable(struct iio_dev *indio_dev)
{
	return adi_axi_dac_buffer_state_set(indio_dev, false);
}

static const struct iio_buffer_setup_ops adi_axi_dac_buffer_setup_ops = {
	.preenable = &adi_axi_dac_buffer_preenable,
	.postdisable = &adi_axi_dac_buffer_postdisable,
};

static int adi_axi_dac_config_dma_buffer(struct device *dev,
					 struct iio_dev *indio_dev)
{
	enum iio_buffer_direction dir = IIO_BUFFER_DIRECTION_OUT;
	const char *dma_name;

	if (!device_property_present(dev, "dmas"))
		return 0;

	if (device_property_read_string(dev, "dma-names", &dma_name))
		dma_name = "tx";

	return devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
					       indio_dev, dir, dma_name,
					       NULL, NULL);
}

static int adi_axi_dac_read_raw(struct iio_dev *indio_dev,
				struct iio_chan_spec const *chan,
				int *val, int *val2, long mask)
{
	struct adi_axi_dac_state *st = iio_priv(indio_dev);
	struct adi_axi_dac_conv *conv = &st->client->conv;

	if (!conv->read_raw)
		return -EOPNOTSUPP;

	return conv->read_raw(conv, chan, val, val2, mask);
}

static int adi_axi_dac_write_raw(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int val, int val2, long mask)
{
	struct adi_axi_dac_state *st = iio_priv(indio_dev);
	struct adi_axi_dac_conv *conv = &st->client->conv;

	if (!conv->write_raw)
		return -EOPNOTSUPP;

	return conv->write_raw(conv, chan, val, val2, mask);
}

static void adi_axi_dac_data_sel_set(struct adi_axi_dac_state *st, int chan,
				     enum adi_axi_dac_data_sel sel)
{
	struct adi_axi_dac_conv *conv = &st->client->conv;
	int i;

	if (chan > -1) {
		adi_axi_dac_write(st, ADI_AXI_REG_CHAN_DATA_SEL(chan), sel);
		return;
	}

	/* negative channel, means all channels */
	for (i = 0; i < conv->chip_info->num_channels; i++)
		adi_axi_dac_write(st, ADI_AXI_REG_CHAN_DATA_SEL(i), sel);
}

static int adi_axi_dac_update_scan_mode(struct iio_dev *indio_dev,
					const unsigned long *scan_mask)
{
	struct adi_axi_dac_state *st = iio_priv(indio_dev);
	struct adi_axi_dac_conv *conv = &st->client->conv;
	unsigned int i, sel;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		if (test_bit(i, scan_mask))
			sel = AXI_DAC_DATA_SEL_DMA;
		else
			sel = AXI_DAC_DATA_SEL_DDS;

		adi_axi_dac_write(st, ADI_AXI_REG_CHAN_DATA_SEL(i), sel);
	}

	return 0;
}

static struct adi_axi_dac_conv *adi_axi_dac_conv_register(struct device *dev,
							  int sizeof_priv)
{
	struct adi_axi_dac_client *cl;
	size_t alloc_size;

	alloc_size = ALIGN(sizeof(struct adi_axi_dac_client), IIO_ALIGN);
	if (sizeof_priv)
		alloc_size += ALIGN(sizeof_priv, IIO_ALIGN);

	cl = kzalloc(alloc_size, GFP_KERNEL);
	if (!cl)
		return ERR_PTR(-ENOMEM);

	mutex_lock(&registered_clients_lock);

	cl->dev = get_device(dev);

	list_add_tail(&cl->entry, &registered_clients);

	mutex_unlock(&registered_clients_lock);

	return &cl->conv;
}

static void adi_axi_dac_conv_unregister(struct adi_axi_dac_conv *conv)
{
	struct adi_axi_dac_client *cl = conv_to_client(conv);

	mutex_lock(&registered_clients_lock);

	list_del(&cl->entry);
	put_device(cl->dev);

	mutex_unlock(&registered_clients_lock);

	kfree(cl);
}

static void devm_adi_axi_dac_conv_release(struct device *dev, void *res)
{
	adi_axi_dac_conv_unregister(*(struct adi_axi_dac_conv **)res);
}

struct adi_axi_dac_conv *devm_adi_axi_dac_conv_register(struct device *dev,
							int sizeof_priv)
{
	struct adi_axi_dac_conv **ptr, *conv;

	ptr = devres_alloc(devm_adi_axi_dac_conv_release, sizeof(*ptr),
			   GFP_KERNEL);
	if (!ptr)
		return ERR_PTR(-ENOMEM);

	conv = adi_axi_dac_conv_register(dev, sizeof_priv);
	if (IS_ERR(conv)) {
		devres_free(ptr);
		return ERR_CAST(conv);
	}

	*ptr = conv;
	devres_add(dev, ptr);

	return conv;
}
EXPORT_SYMBOL_GPL(devm_adi_axi_dac_conv_register);

static const char * const axi_dac_default_scales[] = {
	"1.000000", "0.500000", "0.250000", "0.125000",
	"0.062500", "0.031250", "0.015625", "0.007812",
	"0.003906", "0.001953", "0.000976", "0.000488",
	"0.000244", "0.000122", "0.000061", "0.000030"
};

static ssize_t out_voltage_scale_available_show(struct device *dev,
						struct device_attribute *attr,
						 char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(axi_dac_default_scales); i++) {
		const char *scale = axi_dac_default_scales[i];

		len += scnprintf(buf + len, PAGE_SIZE - len, "%s ", scale);
	}
	buf[len - 1] = '\n';

	return len;
}

static IIO_DEVICE_ATTR_RO(out_voltage_scale_available, 0);

static struct attribute *adi_axi_dac_attributes[] = {
	&iio_dev_attr_out_voltage_scale_available.dev_attr.attr,
	NULL
};

static const struct attribute_group adi_axi_dac_attribute_group = {
	.attrs = adi_axi_dac_attributes,
};

static const struct iio_info adi_axi_dac_info = {
	.read_raw = &adi_axi_dac_read_raw,
	.write_raw = &adi_axi_dac_write_raw,
	.attrs = &adi_axi_dac_attribute_group,
	.update_scan_mode = &adi_axi_dac_update_scan_mode,
};

static const struct adi_axi_dac_core_info adi_axi_dac_9_0_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
};

struct adi_axi_dac_client *adi_axi_dac_attach_client(struct device *dev)
{
	const struct adi_axi_dac_core_info *info;
	struct adi_axi_dac_client *cl;
	struct device_node *cln;

	info = of_device_get_match_data(dev);
	if (!info)
		return ERR_PTR(-ENODEV);

	cln = of_parse_phandle(dev->of_node, "adi,dac-dev", 0);
	if (!cln) {
		dev_err(dev, "No 'adi,dac-dev' node defined\n");
		return ERR_PTR(-ENODEV);
	}

	mutex_lock(&registered_clients_lock);

	list_for_each_entry(cl, &registered_clients, entry) {
		if (!cl->dev)
			continue;

		if (cl->dev->of_node != cln)
			continue;

		if (!try_module_get(cl->dev->driver->owner)) {
			mutex_unlock(&registered_clients_lock);
			return ERR_PTR(-ENODEV);
		}

		get_device(cl->dev);
		cl->info = info;
		mutex_unlock(&registered_clients_lock);
		return cl;
	}

	mutex_unlock(&registered_clients_lock);

	return ERR_PTR(-EPROBE_DEFER);
}

static void adi_axi_dac_setup_dac_chan(struct adi_axi_dac_state *st,
				       unsigned long long dac_rate,
				       unsigned int chan,
				       unsigned int phase,
				       unsigned int freq,
				       unsigned int scale)
{
	u64 val64;
	u32 val;

	val64 = (u64) freq * 0xFFFFULL;
	val64 = div64_u64(val64, dac_rate);
	val = ADI_AXI_REG_DDS_INCR_SET(val64) | 1;

	val64 = (u64) phase * 0x10000ULL + (360000 / 2);
	do_div(val64, 360000);
	val |= ADI_AXI_REG_DDS_INIT_SET(val64);

	adi_axi_dac_write(st, ADI_AXI_REG_CHAN_CNTRL_1(chan),
			  ADI_AXI_REG_SCALE_1_SET(scale));
	adi_axi_dac_write(st, ADI_AXI_REG_CHAN_CNTRL_2(chan), val);
}

static int adi_axi_dac_setup_channels(struct device *dev,
				      struct adi_axi_dac_state *st)
{
	struct adi_axi_dac_conv *conv = conv = &st->client->conv;
	unsigned long long dac_rate;
	unsigned int val, phase;
	int i;

	dac_rate = conv->get_rate(conv);
	if (dac_rate == 0) {
		dev_err(dev, "DAC clock rate is 0\n");
		return -EINVAL;
	}

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		const struct iio_chan_spec *chan;

		chan = &conv->chip_info->channels[i];

		if (!chan->modified)
			continue;

		if (chan->channel2 == IIO_MOD_Q) {
			phase = 0;
			val = ADI_AXI_IQCOR_COEFF_2_INT_1;
		} else {
			phase = 90000;
			val = ADI_AXI_IQCOR_COEFF_1_INT_1;
		}

		adi_axi_dac_setup_dac_chan(st, dac_rate, i, phase,
					   ADI_AXI_DEFAULT_FREQUENCY,
					   ADI_AXI_DEFAULT_SCALE);

		adi_axi_dac_write(st, ADI_AXI_REG_CHAN_IQCOR_CTRL(i), val);
	}

	return 0;
}

static int adi_axi_dac_setup(struct device *dev, struct adi_axi_dac_state *st)
{
	struct adi_axi_dac_conv *conv = conv = &st->client->conv;
	int ret;

	adi_axi_dac_write(st, ADI_AXI_REG_RATE_CTRL, 1);
	
	if (conv->preenable_setup) {
		ret = conv->preenable_setup(conv);
		if (ret)
			return ret;
	}

	adi_axi_dac_write(st, ADI_AXI_REG_CTRL2, ADI_AXI_REG_CTRL2_DEFAULTS);

	adi_axi_dac_data_sel_set(st, -1, AXI_DAC_DATA_SEL_DDS);

	return adi_axi_dac_setup_channels(dev, st);
}

static void axi_dac_reset(struct adi_axi_dac_state *st)
{
	adi_axi_dac_write(st, ADI_AXI_REG_RSTN, 0);
	mdelay(10);
	adi_axi_dac_write(st, ADI_AXI_REG_RSTN, ADI_AXI_REG_RSTN_MMCM_RSTN);
	mdelay(10);
	adi_axi_dac_write(st, ADI_AXI_REG_RSTN,
			  ADI_AXI_REG_RSTN_RSTN | ADI_AXI_REG_RSTN_MMCM_RSTN);
}


static void adi_axi_dac_cleanup(void *data)
{
	struct adi_axi_dac_client *cl = data;

	put_device(cl->dev);
	module_put(cl->dev->driver->owner);
}

static int adi_axi_dac_probe(struct platform_device *pdev)
{
	struct adi_axi_dac_conv *conv = NULL;
	struct iio_dev *indio_dev;
	struct adi_axi_dac_client *cl;
	struct adi_axi_dac_state *st;
	unsigned int ver;
	int ret;

	cl = adi_axi_dac_attach_client(&pdev->dev);
	if (IS_ERR(cl))
		return PTR_ERR(cl);

	ret = devm_add_action_or_reset(&pdev->dev, adi_axi_dac_cleanup, cl);
	if (ret)
		return ret;

	if (!cl->conv.get_rate) {
		dev_err(&pdev->dev,
			"Client device '%s' has no 'get_rate' cb\n",
			dev_name(cl->dev));
		return -ENODEV;
	}

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->client = cl;
	cl->state = st;
	mutex_init(&st->lock);

	st->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	conv = &st->client->conv;

	axi_dac_reset(st);

	ver = adi_axi_dac_read(st, ADI_AXI_REG_VERSION);

	if (cl->info->version > ver) {
		dev_err(&pdev->dev,
			"IP core version is too old. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(cl->info->version),
			ADI_AXI_PCORE_VER_MINOR(cl->info->version),
			ADI_AXI_PCORE_VER_PATCH(cl->info->version),
			ADI_AXI_PCORE_VER_MAJOR(ver),
			ADI_AXI_PCORE_VER_MINOR(ver),
			ADI_AXI_PCORE_VER_PATCH(ver));
		return -ENODEV;
	}

	indio_dev->info = &adi_axi_dac_info;
	indio_dev->name = "adi-axi-dac";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = conv->chip_info->num_channels;
	indio_dev->channels = conv->chip_info->channels;

	ret = adi_axi_dac_config_dma_buffer(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	ret = adi_axi_dac_setup(&pdev->dev, st);
	if (ret)
		return ret;

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "AXI DAC IP core (%d.%.2d.%c) probed\n",
		 ADI_AXI_PCORE_VER_MAJOR(ver),
		 ADI_AXI_PCORE_VER_MINOR(ver),
		 ADI_AXI_PCORE_VER_PATCH(ver));

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id adi_axi_dac_of_match[] = {
	{ .compatible = "adi,axi-dac-9.0.a", .data = &adi_axi_dac_9_0_a_info },
	{ /* end of list */ }
};
MODULE_DEVICE_TABLE(of, adi_axi_dac_of_match);

static struct platform_driver adi_axi_dac_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = adi_axi_dac_of_match,
	},
	.probe = adi_axi_dac_probe,
};
module_platform_driver(adi_axi_dac_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices Generic AXI DAC IP core driver");
MODULE_LICENSE("GPL v2");
