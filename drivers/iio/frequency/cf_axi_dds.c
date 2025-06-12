/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/gpio/consumer.h>
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
#include <linux/debugfs.h>
#include <linux/uaccess.h>
#include <linux/gpio/consumer.h>

#include <linux/of_device.h>
#include <linux/of_dma.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/adi-common.h>

#include "cf_axi_dds.h"
#include "ad9122.h"
#include "../../misc/adi-axi-data-offload.h"

static const unsigned int interpolation_factors_available[] = {1, 8};

static const char * const dds_extend_names_complex[] = {
	"TX1_I_F1", "TX1_I_F2", "TX1_Q_F1", "TX1_Q_F2",
	"TX2_I_F1", "TX2_I_F2", "TX2_Q_F1", "TX2_Q_F2",
	"TX3_I_F1", "TX3_I_F2", "TX3_Q_F1", "TX3_Q_F2",
	"TX4_I_F1", "TX4_I_F2", "TX4_Q_F1", "TX4_Q_F2",
	"TX5_I_F1", "TX5_I_F2", "TX5_Q_F1", "TX5_Q_F2",
	"TX6_I_F1", "TX6_I_F2", "TX6_Q_F1", "TX6_Q_F2",
	"TX7_I_F1", "TX7_I_F2", "TX7_Q_F1", "TX7_Q_F2",
	"TX8_I_F1", "TX8_I_F2", "TX8_Q_F1", "TX8_Q_F2",
	"TX9_I_F1", "TX9_I_F2", "TX9_Q_F1", "TX9_Q_F2",
	"TX10_I_F1", "TX10_I_F2", "TX10_Q_F1", "TX10_Q_F2",
	"TX11_I_F1", "TX11_I_F2", "TX11_Q_F1", "TX11_Q_F2",
	"TX12_I_F1", "TX12_I_F2", "TX12_Q_F1", "TX12_Q_F2",
	"TX13_I_F1", "TX13_I_F2", "TX13_Q_F1", "TX13_Q_F2",
	"TX14_I_F1", "TX14_I_F2", "TX14_Q_F1", "TX14_Q_F2",
	"TX15_I_F1", "TX15_I_F2", "TX15_Q_F1", "TX15_Q_F2",
	"TX16_I_F1", "TX16_I_F2", "TX16_Q_F1", "TX16_Q_F2",
	"TX17_I_F1", "TX17_I_F2", "TX17_Q_F1", "TX17_Q_F2",
	"TX18_I_F1", "TX18_I_F2", "TX18_Q_F1", "TX18_Q_F2",
	"TX19_I_F1", "TX19_I_F2", "TX19_Q_F1", "TX19_Q_F2",
	"TX20_I_F1", "TX20_I_F2", "TX20_Q_F1", "TX20_Q_F2",
	"TX21_I_F1", "TX21_I_F2", "TX21_Q_F1", "TX21_Q_F2",
	"TX22_I_F1", "TX22_I_F2", "TX22_Q_F1", "TX22_Q_F2",
	"TX23_I_F1", "TX23_I_F2", "TX23_Q_F1", "TX23_Q_F2",
	"TX24_I_F1", "TX24_I_F2", "TX24_Q_F1", "TX24_Q_F2",
	"TX25_I_F1", "TX25_I_F2", "TX25_Q_F1", "TX25_Q_F2",
	"TX26_I_F1", "TX26_I_F2", "TX26_Q_F1", "TX26_Q_F2",
	"TX27_I_F1", "TX27_I_F2", "TX27_Q_F1", "TX27_Q_F2",
	"TX28_I_F1", "TX28_I_F2", "TX28_Q_F1", "TX28_Q_F2",
	"TX29_I_F1", "TX29_I_F2", "TX29_Q_F1", "TX29_Q_F2",
	"TX30_I_F1", "TX30_I_F2", "TX30_Q_F1", "TX30_Q_F2",
	"TX31_I_F1", "TX31_I_F2", "TX31_Q_F1", "TX31_Q_F2",
	"TX32_I_F1", "TX32_I_F2", "TX32_Q_F1", "TX32_Q_F2",
};

static const char * const dds_extend_names[] = {
	"1A", "1B", "2A", "2B",
	"3A", "3B", "4A", "4B",
	"5A", "5B", "6A", "6B",
	"7A", "7B", "8A", "8B",
	"9A", "9B", "10A", "10B",
	"11A", "11B", "12A", "12B",
	"13A", "13B", "14A", "14B",
	"15A", "15B", "16A", "16B",
	"17A", "17B", "18A", "18B",
	"19A", "19B", "20A", "20B",
	"21A", "21B", "22A", "22B",
	"23A", "23B", "24A", "24B",
	"25A", "25B", "26A", "26B",
	"27A", "27B", "28A", "28B",
	"29A", "29B", "30A", "30B",
	"31A", "31B", "32A", "32B",
};

struct cf_axi_dds_state {
	struct device			*dev_spi;
	struct axi_data_offload_state	*data_offload;
	struct clk			*clk;
	struct clock_scale		clkscale;
	struct cf_axi_dds_chip_info	*chip_info;
	struct gpio_desc		*plddrbypass_gpio;
	struct gpio_desc		*interpolation_gpio;
	struct jesd204_dev 		*jdev;
	/*
	 * Lock used when in standalone mode.
	 */
	struct mutex			lock;
	bool				standalone;
	bool				dp_disable;
	bool				enable;
	bool				pl_dma_fifo_en;
	enum fifo_ctrl			dma_fifo_ctrl_bypass;
	bool				dma_fifo_ctrl_oneshot;
	bool				issue_sync_en;
	bool				ext_sync_avail;

	struct iio_info			iio_info;
	struct iio_dev			*indio_dev;
	size_t				regs_size;
	void __iomem			*regs;
	void __iomem			*slave_regs;
	void __iomem			*master_regs;
	u64				dac_clk;
	unsigned int			ddr_dds_interp_en;
	unsigned int			cached_freq[AXIDDS_MAX_NUM_DDS_CHAN];
	unsigned int			version;
	unsigned int			have_slave_channels;
	unsigned int			interpolation_factor;
	struct notifier_block		clk_nb;
	struct cf_axi_dds_chip_info	chip_info_generated;
};

#define cf_axi_dds_lock(st) {							\
	struct cf_axi_dds_state *__st = st;					\
										\
	if (__st->standalone) {							\
		mutex_lock(&__st->lock);					\
	} else {								\
		struct cf_axi_converter *conv = to_converter(__st->dev_spi);	\
										\
		mutex_lock(&conv->lock);					\
	}									\
}

#define cf_axi_dds_unlock(st) {								\
	struct cf_axi_dds_state *__st = st;					\
										\
	if (__st->standalone) {							\
		mutex_unlock(&__st->lock);					\
	} else {								\
		struct cf_axi_converter *conv = to_converter(__st->dev_spi);	\
										\
		mutex_unlock(&conv->lock);					\
	}									\
}

bool cf_axi_dds_dma_fifo_en(struct cf_axi_dds_state *st)
{
	return st->pl_dma_fifo_en;
}
EXPORT_SYMBOL(cf_axi_dds_dma_fifo_en);

void dds_write(struct cf_axi_dds_state *st,
	       unsigned int reg, unsigned int val)
{
	iowrite32(val, st->regs + reg);
}
EXPORT_SYMBOL(dds_write);

int dds_read(struct cf_axi_dds_state *st, unsigned int reg)
{
	return ioread32(st->regs + reg);
}
EXPORT_SYMBOL(dds_read);

void dds_slave_write(struct cf_axi_dds_state *st,
		     unsigned int reg, unsigned int val)
{
	iowrite32(val, st->slave_regs + reg);
}
EXPORT_SYMBOL(dds_slave_write);

unsigned int dds_slave_read(struct cf_axi_dds_state *st, unsigned int reg)
{
	return ioread32(st->slave_regs + reg);
}
EXPORT_SYMBOL(dds_slave_read);

void dds_master_write(struct cf_axi_dds_state *st,
		      unsigned int reg, unsigned int val)
{
	if (st->master_regs)
		iowrite32(val, st->master_regs + reg);
}
EXPORT_SYMBOL(dds_master_write);

static int cf_axi_dds_to_signed_mag_fmt(int val, int val2, unsigned int *res)
{
	unsigned int i;
	u64 val64;

	/*  format is 1.1.14 (sign, integer and fractional bits) */
	switch (val) {
	case 1:
		i = 0x4000;
		break;
	case -1:
		i = 0xC000;
		break;
	case 0:
		i = 0;
		if (val2 < 0) {
			i = 0x8000;
			val2 *= -1;
		}
		break;
	default:
		pr_err("%s: Invalid Value\n", __func__);
		return -EINVAL;
	}

	val64 = (unsigned long long)val2 * 0x4000UL + (1000000UL / 2);
		do_div(val64, 1000000UL);

	*res = i | val64;

	return 0;
}

static int cf_axi_dds_signed_mag_fmt_to_iio(unsigned int val, int *r_val,
	int *r_val2)
{
	u64 val64;
	int sign;

	if (val & 0x8000)
		sign = -1;
	else
		sign = 1;

	if (val & 0x4000)
		*r_val = 1 * sign;
	else
		*r_val = 0;

	val &= ~0xC000;

	val64 = val * 1000000ULL + (0x4000 / 2);
	do_div(val64, 0x4000);

	if (*r_val == 0)
		*r_val2 = val64 * sign;
	else
		*r_val2 = val64;

	return IIO_VAL_INT_PLUS_MICRO;
}

int cf_axi_dds_pl_ddr_fifo_ctrl_oneshot(struct cf_axi_dds_state *st, bool enable)
{
	int ret;

	if (!st->data_offload)
		return -ENODEV;

	if (st->dma_fifo_ctrl_oneshot == enable)
		return 0;

	ret = axi_data_offload_ctrl_oneshot(st->data_offload, enable);
	if (!ret)
		st->dma_fifo_ctrl_oneshot = enable;

	return ret;
}

int cf_axi_dds_pl_ddr_fifo_ctrl(struct cf_axi_dds_state *st, bool enable)
{
	enum fifo_ctrl mode;
	int ret;

	mode = (enable ? FIFO_ENABLE : FIFO_DISABLE);

	if (st->data_offload) {
		if (st->dma_fifo_ctrl_bypass == mode)
			return 0;

		ret = axi_data_offload_ctrl_bypass(st->data_offload, !enable);

		if (!ret)
			st->dma_fifo_ctrl_bypass = mode;

		return ret;
	}

	if (!st->plddrbypass_gpio)
		return -ENODEV;

	if (st->dma_fifo_ctrl_bypass == mode)
		return 0;

	ret = gpiod_direction_output(st->plddrbypass_gpio, !enable);
	if (ret == 0)
		st->dma_fifo_ctrl_bypass = mode;

	return ret;
}

static int cf_axi_get_parent_sampling_frequency(struct cf_axi_dds_state *st,
						u64 *freq)
{
	struct cf_axi_converter *conv;

	if (st->standalone) {
		*freq = st->dac_clk = clk_get_rate_scaled(st->clk, &st->clkscale);
	} else {
		conv = to_converter(st->dev_spi);
		if (!conv->get_data_clk)
			return -ENODEV;

		*freq = st->dac_clk = conv->get_data_clk(conv);
	}

	return 0;
}


void __cf_axi_dds_datasel(struct cf_axi_dds_state *st,
	int channel, enum dds_data_select sel)
{
	unsigned int val;

	if ((unsigned int)channel > (st->have_slave_channels - 1)) {
		val = dds_slave_read(st,
			ADI_REG_CHAN_CNTRL_7(channel -
			st->have_slave_channels));

		val &= ~ADI_DAC_DDS_SEL(~0);
		val |= ADI_DAC_DDS_SEL(sel);

		dds_slave_write(st,
			ADI_REG_CHAN_CNTRL_7(channel -
			st->have_slave_channels), val);
	} else {
		val = dds_read(st, ADI_REG_CHAN_CNTRL_7(channel));

		val &= ~ADI_DAC_DDS_SEL(~0);
		val |= ADI_DAC_DDS_SEL(sel);

		dds_write(st, ADI_REG_CHAN_CNTRL_7(channel), val);
	}
}

int cf_axi_dds_datasel(struct cf_axi_dds_state *st,
			       int channel, enum dds_data_select sel)
{
	unsigned int i;

	if (channel < 0) /* ALL */
		for (i = 0; i < st->chip_info->num_buf_channels; i++)
			__cf_axi_dds_datasel(st, i, sel);
	else
		__cf_axi_dds_datasel(st, channel, sel);

	return 0;
}
EXPORT_SYMBOL_GPL(cf_axi_dds_datasel);

static enum dds_data_select cf_axi_dds_get_datasel(struct cf_axi_dds_state *st,
			       int channel)
{
	if (channel < 0)
		channel = 0;

	if ((unsigned int)channel > (st->have_slave_channels - 1))
		return ADI_TO_DAC_DDS_SEL(dds_slave_read(st,
			ADI_REG_CHAN_CNTRL_7(channel -
			st->have_slave_channels)));

	return ADI_TO_DAC_DDS_SEL(dds_read(st, ADI_REG_CHAN_CNTRL_7(channel)));
}

static int cf_axi_dds_sync_frame(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv;
	int stat;
	static int retry;

	msleep(10); /* Wait until clocks are stable */

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

void cf_axi_dds_start_sync(struct cf_axi_dds_state *st, int sync_dma)
{
	if (sync_dma && !st->issue_sync_en)
		return;

	dds_write(st, ADI_REG_CNTRL_1, ADI_SYNC);
	dds_master_write(st, ADI_REG_CNTRL_1, ADI_SYNC);
}
EXPORT_SYMBOL_GPL(cf_axi_dds_start_sync);

static int cf_axi_dds_rate_change(struct notifier_block *nb,
	unsigned long flags, void *data)
{
	struct clk_notifier_data *cnd = data;
	struct cf_axi_dds_state *st =
		container_of(nb, struct cf_axi_dds_state, clk_nb);
	unsigned int reg, i;
	unsigned long long val64;

	st->dac_clk = cnd->new_rate;

	if (flags == POST_RATE_CHANGE) {
		st->dac_clk = cnd->new_rate;

		for (i = 0; i < st->chip_info->num_dds_channels; i++) {
			reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(i));
			reg &= ~ADI_DDS_INCR(~0);
			val64 = (u64) st->cached_freq[i] * 0xFFFFULL;
			val64 = div64_u64(val64, st->dac_clk);
			reg |= ADI_DDS_INCR(val64) | 1;
			dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(i), reg);
		}
		cf_axi_dds_start_sync(st, 0);
		cf_axi_dds_sync_frame(st->indio_dev);
	}

	return NOTIFY_OK;
}

static void cf_axi_dds_set_sed_pattern(struct iio_dev *indio_dev,
				       unsigned int chan, unsigned int pat1,
				       unsigned int pat2)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned int ctrl;

	dds_write(st, ADI_REG_CHAN_CNTRL_5(chan),
		ADI_TO_DDS_PATT_1(pat1) | ADI_DDS_PATT_2(pat2));

	cf_axi_dds_datasel(st, -1, DATA_SEL_SED);

	ctrl = dds_read(st, ADI_REG_CNTRL_2);
	dds_write(st, ADI_REG_CNTRL_2, ctrl | ADI_DATA_FORMAT);

	cf_axi_dds_start_sync(st, 0);
}

static int cf_axi_dds_default_setup(struct cf_axi_dds_state *st, u32 chan,
				    u32 phase, u32 freq, u32 scale)
{

	unsigned long long val64;
	u32 val;

	st->cached_freq[chan] = freq;

	val64 = (u64) freq * 0xFFFFULL;
	val64 = div64_u64(val64, st->dac_clk);
	val = ADI_DDS_INCR(val64) | 1;

	val64 = (u64) phase * 0x10000ULL + (360000 / 2);
	do_div(val64, 360000);
	val |= ADI_DDS_INIT(val64);

	dds_write(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan), ADI_DDS_SCALE(scale));
	dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan), val);

	return 0;
}

static int cf_axi_interpolation_set(struct cf_axi_dds_state *st,
				    unsigned int interpolation_factor)
{
	u32 reg;
	int ret = 0;

	switch (interpolation_factor) {
	case 1:
	case 8:
		reg = dds_read(st, ADI_REG_DAC_GP_CONTROL);

		if (st->interpolation_factor == 8)
			reg |= BIT(0);
		else
			reg &= ~BIT(0);

		if (st->interpolation_gpio)
			gpiod_set_value(st->interpolation_gpio,
					reg & BIT(0));
		else
			dds_write(st, ADI_REG_DAC_GP_CONTROL, reg);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static ssize_t cf_axi_interpolation_store(struct cf_axi_dds_state *st,
					  unsigned long frequency)
{
	unsigned long val;
	u64 parent;
	int i, ret;

	if (!frequency)
		return -EINVAL;

	ret = cf_axi_get_parent_sampling_frequency(st, &parent);
	if (ret < 0)
		return ret;

	val = DIV_ROUND_CLOSEST_ULL(parent, frequency);

	for (i = 0; i < ARRAY_SIZE(interpolation_factors_available); i++) {
		if (val == interpolation_factors_available[i]) {
			st->interpolation_factor = val;
			return cf_axi_interpolation_set(st, val);
		}
	}

	return -EINVAL;
}

static ssize_t cf_axi_sampling_frequency_available(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	u64 freq;
	int i, ret;

	if (!st->interpolation_factor)
		return -ENODEV;

	cf_axi_dds_lock(st);
	ret = cf_axi_get_parent_sampling_frequency(st, &freq);
	if (ret < 0) {
		cf_axi_dds_unlock(st);
		return ret;
	}

	for (i = 0, ret = 0; i < ARRAY_SIZE(interpolation_factors_available); i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%lld ",
				div_u64(freq, interpolation_factors_available[i]));

	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "\n");

	cf_axi_dds_unlock(st);

	return ret;
}


static const char * const axidds_sync_ctrls[] = {
	"arm", "disarm", "trigger_manual",
};

static ssize_t axidds_sync_start_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	int ret;

	ret = sysfs_match_string(axidds_sync_ctrls, buf);
	if (ret < 0)
		return ret;

	cf_axi_dds_lock(st);
	if (st->ext_sync_avail) {
		switch (ret) {
		case 0:
			dds_write(st, ADI_REG_CNTRL_1, ADI_EXT_SYNC_ARM);
			break;
		case 1:
			dds_write(st, ADI_REG_CNTRL_1, ADI_EXT_SYNC_DISARM);
			break;
		case 2:
			dds_write(st, ADI_REG_CNTRL_1, ADI_MANUAL_SYNC_REQUEST);
			break;
		default:
			ret = -EINVAL;
		}
	} else if (ret == 0) {
		cf_axi_dds_start_sync(st, 0);
	}
	cf_axi_dds_unlock(st);

	return ret < 0 ? ret : len;
}

static ssize_t axidds_sync_start_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	u32 reg;

	switch ((u32)this_attr->address) {
	case 0:
		reg = dds_read(st, ADI_REG_SYNC_STATUS);

		return sprintf(buf, "%s\n", reg & ADI_ADC_SYNC_STATUS ?
			axidds_sync_ctrls[0] : axidds_sync_ctrls[1]);
	case 1:
		if (st->ext_sync_avail)
			return sprintf(buf, "arm disarm trigger_manual\n");
		else
			return sprintf(buf, "arm\n");
	default:
		return -EINVAL;
	}

	return -EINVAL;
}

static IIO_DEVICE_ATTR(sync_start_enable, 0644,
		       axidds_sync_start_show,
		       axidds_sync_start_store,
		       0);

static IIO_DEVICE_ATTR(sync_start_enable_available, 0444,
		       axidds_sync_start_show,
		       NULL,
		       1);

static IIO_DEVICE_ATTR(out_voltage_sampling_frequency_available, 0444,
		       cf_axi_sampling_frequency_available,
		       NULL,
		       0);

static struct attribute *cf_axi_attributes[] = {
	&iio_dev_attr_sync_start_enable.dev_attr.attr,
	&iio_dev_attr_sync_start_enable_available.dev_attr.attr,
	&iio_dev_attr_out_voltage_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group cf_axi_int_attribute_group = {
	.attrs = cf_axi_attributes,
};

static int cf_axi_dds_reg_index(struct iio_chan_spec const *chan)
{
	if (chan->modified)
		switch (chan->channel2) {
		case IIO_MOD_I:
			return chan->channel * 2;
		case IIO_MOD_Q:
			return chan->channel * 2 + 1;
		default:
			return chan->channel;
		}

	return chan->channel;
}

static int cf_axi_dds_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv;
	unsigned long long val64, freq;
	unsigned int reg, channel, phase = 0;
	int ret;

	cf_axi_dds_lock(st);

	switch (m) {
	case 0:
		if (!chan->output) {
			ret = -EINVAL;
			break;
		}
		*val = st->enable;

		cf_axi_dds_unlock(st);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			if (!st->standalone) {
				conv = to_converter(st->dev_spi);
				if (!conv->read_raw) {
					ret = -ENODEV;
				} else {
					ret = conv->read_raw(indio_dev, chan,
							     val, val2, m);
				}
				cf_axi_dds_unlock(st);
				return ret;
			}
		}

		reg = ADI_TO_DDS_SCALE(dds_read(st,
			ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel)));
		cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
		cf_axi_dds_unlock(st);
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_FREQUENCY:
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		val64 = (u64)ADI_TO_DDS_INCR(reg) * (u64)st->dac_clk;
		do_div(val64, 0xFFFF);
		*val = val64;
		cf_axi_dds_unlock(st);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_PHASE:
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		val64 = (u64)ADI_TO_DDS_INIT(reg) * 360000ULL + (0x10000 / 2);
		do_div(val64, 0x10000);
		*val = val64;
		cf_axi_dds_unlock(st);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = cf_axi_get_parent_sampling_frequency(st, &freq);
		if (ret < 0)
			break;

		if (chan->type == IIO_VOLTAGE && st->interpolation_factor)
			do_div(freq, st->interpolation_factor);

		*val = lower_32_bits(freq);
		*val2 = upper_32_bits(freq);

		cf_axi_dds_unlock(st);
		return IIO_VAL_INT_64;
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		fallthrough;
	case IIO_CHAN_INFO_CALIBSCALE:
		channel = cf_axi_dds_reg_index(chan);

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_8(channel));
		/*  format is 1.1.14 (sign, integer and fractional bits) */

		if (!((phase + channel) % 2))
			reg = ADI_TO_IQCOR_COEFF_1(reg);
		else
			reg = ADI_TO_IQCOR_COEFF_2(reg);

		cf_axi_dds_unlock(st);
		return cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
	default:
		if (!st->standalone) {
			conv = to_converter(st->dev_spi);
			if (!conv->read_raw) {
				ret = -ENODEV;
			} else {
				ret = conv->read_raw(indio_dev, chan,
						     val, val2, m);
			}
		} else {
			ret = -EINVAL;
		}
	}

	cf_axi_dds_unlock(st);

	return ret;
}

static int cf_axi_dds_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv;
	unsigned long long val64;
	unsigned int reg, i, channel, phase = 0;
	int ret = 0;

	if (st->dev_spi)
		conv = to_converter(st->dev_spi);
	else
		conv = ERR_PTR(-ENODEV);

	cf_axi_dds_lock(st);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!chan->output) {
			ret = -EINVAL;
			break;
		}

		st->enable = !!val;
		cf_axi_dds_start_sync(st, 0);
		cf_axi_dds_datasel(st, -1,
			st->enable ? DATA_SEL_DDS : DATA_SEL_ZERO);

		break;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			if (!st->standalone) {
				if (IS_ERR(conv)) {
					ret = PTR_ERR(conv);
				} else if (!conv->write_raw) {
					ret = -ENODEV;
				} else {
					ret = conv->write_raw(indio_dev,
						chan, val, val2, mask);
				}
				cf_axi_dds_unlock(st);
				return ret;
			}
		}

		/*  format is 1.1.14 (sign, integer and fractional bits) */
		switch (val) {
		case 1:
			i = 0x4000;
			break;
		case -1:
			i = 0xC000;
			break;
		case 0:
			i = 0;
			if (val2 < 0) {
				i = 0x8000;
				val2 *= -1;
			}
			break;
		default:
			ret = -EINVAL;
			goto err_unlock;
		}

		val64 = (unsigned long long)val2 * 0x4000UL + (1000000UL / 2);
		do_div(val64, 1000000UL);
		i |= val64;
		dds_write(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel),
			  ADI_DDS_SCALE(i));
		cf_axi_dds_start_sync(st, 0);
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

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		reg &= ~ADI_DDS_INCR(~0);
		val64 = (u64) val * 0xFFFFULL;
		val64 = div64_u64(val64, st->dac_clk);
		reg |= ADI_DDS_INCR(val64) | 1;
		dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel), reg);

		val64 = (u64)ADI_TO_DDS_INCR(reg) * (u64)st->dac_clk;
		do_div(val64, 0xFFFF);
		st->cached_freq[chan->channel] = val64;

		cf_axi_dds_start_sync(st, 0);
		break;
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > 360000) {
			ret = -EINVAL;
			break;
		}

		if (val == 360000)
			val = 0;

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		reg &= ~ADI_DDS_INIT(~0);
		val64 = (u64) val * 0x10000ULL + (360000 / 2);
		do_div(val64, 360000);
		reg |= ADI_DDS_INIT(val64);
		dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel), reg);
		cf_axi_dds_start_sync(st, 0);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (chan->type == IIO_VOLTAGE && st->interpolation_factor) {
			ret = cf_axi_interpolation_store(st, val);
			if (!ret)
				break;

		}

		if (IS_ERR(conv)) {
			ret = -EINVAL;
			break;
		}
		if (!conv->write_raw) {
			ret = -ENODEV;
			break;
		}

		reg = dds_read(st, ADI_REG_CNTRL_2);
		i = cf_axi_dds_get_datasel(st, -1);
		conv->write_raw(indio_dev, chan, val, val2, mask);
		dds_write(st, ADI_REG_CNTRL_2, reg);
		cf_axi_dds_datasel(st, -1, i);
		st->dac_clk = conv->get_data_clk(conv);
		cf_axi_dds_start_sync(st, 0);
		ret = cf_axi_dds_sync_frame(indio_dev);
		break;
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		fallthrough;
	case IIO_CHAN_INFO_CALIBSCALE:
		channel = cf_axi_dds_reg_index(chan);

		ret = cf_axi_dds_to_signed_mag_fmt(val, val2, &i);
		if (ret < 0)
			break;

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_8(channel));

		if (!((channel + phase) % 2)) {
			reg &= ~ADI_IQCOR_COEFF_1(~0);
			reg |= ADI_IQCOR_COEFF_1(i);
		} else {
			reg &= ~ADI_IQCOR_COEFF_2(~0);
			reg |= ADI_IQCOR_COEFF_2(i);
		}

		dds_write(st, ADI_REG_CHAN_CNTRL_8(channel), reg);
		dds_write(st, ADI_REG_CHAN_CNTRL_6(channel),
			  ADI_IQCOR_ENB);
		break;
	default:
		if (IS_ERR(conv))
			ret = PTR_ERR(conv);
		else if (!conv->write_raw)
			ret = -ENODEV;
		else
			ret = conv->write_raw(indio_dev, chan, val, val2, mask);
	}

err_unlock:
	cf_axi_dds_unlock(st);

	return ret;
}

static int cf_axi_dds_reg_access(struct iio_dev *indio_dev,
				 unsigned int reg, unsigned int writeval,
				 unsigned int *readval)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv = ERR_PTR(-ENODEV);
	int ret;

	if ((reg & ~DEBUGFS_DRA_PCORE_REG_MAGIC) > 0xFFFF)
		return -EINVAL;

	/* Check that the register is in range and aligned */
	if (((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) || st->standalone) &&
	    ((reg & 0xffff) >= st->regs_size || (reg & 0x3)))
		return -EINVAL;

	if (st->dev_spi)
		conv = to_converter(st->dev_spi);

	cf_axi_dds_lock(st);
	if (readval == NULL) {
		if ((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) ||
			st->standalone) {
			dds_write(st, reg & 0xFFFF, writeval);
			ret = 0;
		} else {
			if (IS_ERR(conv))
				ret  = PTR_ERR(conv);
			else if (!conv->write)
				ret = -ENODEV;
			else
				ret = conv->write(conv->spi,
						  reg, writeval & 0xFF);
		}
	} else {
		if ((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) ||
			st->standalone) {
			ret = dds_read(st, reg & 0xFFFF);
		} else {
			if (IS_ERR(conv))
				ret  = PTR_ERR(conv);
			else if (!conv->read)
				ret = -ENODEV;
			else
				ret = conv->read(conv->spi, reg);
			if (ret < 0)
				goto out_unlock;
		}
		*readval = ret;
		ret = 0;

	}

out_unlock:
	cf_axi_dds_unlock(st);

	return ret;
}

static int cf_axi_dds_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned int i, sel;

	for (i = 0; i < indio_dev->masklength; i++) {

		if (test_bit(i, scan_mask))
			sel = DATA_SEL_DMA;
		else
			sel = DATA_SEL_DDS;

		cf_axi_dds_datasel(st, i, sel);
	}

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
	IIO_ENUM_AVAILABLE("scale", IIO_SHARED_BY_TYPE, &cf_axi_dds_scale_available),
	{ },
};

static void cf_axi_dds_update_chan_spec(struct cf_axi_dds_state *st,
			struct iio_chan_spec *channels, unsigned int num)
{
	unsigned int i;

	for (i = 0; i < num; i++) {
		if (channels[i].type == IIO_ALTVOLTAGE)
			channels[i].ext_info = NULL;
	}
}


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
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) | \
		BIT(IIO_CHAN_INFO_CALIBPHASE), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.output = 1, \
	.scan_index = _chan, \
	.scan_type = { \
		.sign = 's', \
		.storagebits = 16, \
		.realbits = 16, \
		.shift = 0, \
	} \
}

#define CF_AXI_DDS_CHAN_BUF_MOD(_chan, _mod, _si) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.modified = 1, \
	.channel = _chan, \
	.channel2 = _mod, \
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.output = 1, \
	.scan_index = _si, \
	.scan_type = { \
		.sign = 's', \
		.storagebits = 16, \
		.realbits = 16, \
		.shift = 0, \
	} \
}

#define CF_AXI_DDS_CHAN_BUF_NO_CALIB(_chan) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _chan, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.output = 1, \
	.scan_index = _chan, \
	.scan_type = { \
		.sign = 's', \
		.storagebits = 16, \
		.realbits = 16, \
		.shift = 0, \
	} \
}

#define CF_AXI_DDS_CHAN_BUF_VIRT(_chan) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = _chan, \
	.output = 1, \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SAMP_FREQ), \
	.scan_index = _chan, \
	.scan_type = { \
		.sign = 's', \
		.storagebits = 16, \
		.realbits = 16, \
		.shift = 0, \
	} \
}

static const unsigned long ad9361_2x2_available_scan_masks[] = {
	0x01, 0x02, 0x04, 0x08, 0x03, 0x0C, /* 1 & 2 chan */
	0x10, 0x20, 0x40, 0x80, 0x30, 0xC0, /* 1 & 2 chan */
	0x33, 0xCC, 0xC3, 0x3C, 0x0F, 0xF0, /* 4 chan */
	0xFF,				   /* 8 chan */
	0x00,
};

static const unsigned long ad9009_4x2_available_scan_masks[] = {
	0x0001, 0x0002, 0x0004, 0x0008, 0x0003, 0x000C, /* 1 & 2 chan */
	0x0010, 0x0020, 0x0040, 0x0080, 0x0030, 0x00C0, /* 1 & 2 chan */
	0x0100, 0x0200, 0x0400, 0x0800, 0x0300, 0x0C00, /* 1 & 2 chan */
	0x1000, 0x2000, 0x4000, 0x8000, 0x3000, 0xC000, /* 1 & 2 chan */
	0x0033, 0x00CC, 0x00C3, 0x003C, 0x000F, 0x00F0, /* 4 chan */
	0x3300, 0xCC00, 0xC300, 0x3C00, 0x0F00, 0xF000, /* 4 chan */
	0x0330, 0x0CC0, 0x0C30, 0x03C0, 0x00F0, 0x0F00, /* 4 chan */
	0x3300, 0xCC00, 0xC300, 0x3C00, 0x0F00, 0xF000, /* 4 chan */
	0x00FF, 0xFF00, 0x0FF0,	0x3333, 0xCCCC, 0xF0F0, /* 8 chan */
	0x0F0F,						/* 8 chan */
	0xFFFF,						/* 16 chan */
	0x00,
};

static const unsigned long ad9361_available_scan_masks[] = {
	0x01, 0x02, 0x04, 0x08, 0x03, 0x0C, 0x0F,
	0x00,
};

static const unsigned long adrv9002_rx2tx2_available_scan_masks[] = {
	0x01, 0x02, 0x04, 0x08, 0x03, 0x0C, 0x0F,
	0x00,
};

static const unsigned long adrv9002_available_scan_masks[] = {
	0x01, 0x02, 0x03, 0x00
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_tbl[] = {
	[ID_AD9122] = {
		.name = "AD9122",
		.channel = {
			{
				.type = IIO_TEMP,
				.indexed = 1,
				.channel = 0,
				.scan_index = -1,
				.info_mask_separate =
					BIT(IIO_CHAN_INFO_PROCESSED) |
					BIT(IIO_CHAN_INFO_CALIBBIAS),
			},
			CF_AXI_DDS_CHAN_BUF_NO_CALIB(0),
			CF_AXI_DDS_CHAN_BUF_NO_CALIB(1),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
			CF_AXI_DDS_CHAN(2, 0, "2A"),
			CF_AXI_DDS_CHAN(3, 0, "2B"),
		},
		.num_channels = 7,
		.num_dp_disable_channels = 3,
		.num_dds_channels = 4,
		.num_buf_channels = 2,
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
		.num_buf_channels = 1,

	},
	[ID_AD9783] = {
		.name = "AD9783",
		.channel = {
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
		},
		.num_channels = 3,
		.num_dds_channels = 2,
		.num_buf_channels = 1,

	},
	[ID_AD9136] = {
		.name = "AD9136",
		.channel = {
			{
				.type = IIO_TEMP,
				.indexed = 1,
				.channel = 0,
				.scan_index = -1,
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
		.num_buf_channels = 2,
	},
	[ID_AD9144] = {
		.name = "AD9144",
		.channel = {
			{
				.type = IIO_TEMP,
				.indexed = 1,
				.channel = 0,
				.scan_index = -1,
				.info_mask_separate =
					BIT(IIO_CHAN_INFO_PROCESSED) |
					BIT(IIO_CHAN_INFO_CALIBBIAS),
			},
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN_BUF(1),
			CF_AXI_DDS_CHAN_BUF(2),
			CF_AXI_DDS_CHAN_BUF(3),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
			CF_AXI_DDS_CHAN(2, 0, "2A"),
			CF_AXI_DDS_CHAN(3, 0, "2B"),
			CF_AXI_DDS_CHAN(4, 0, "3A"),
			CF_AXI_DDS_CHAN(5, 0, "3B"),
			CF_AXI_DDS_CHAN(6, 0, "4A"),
			CF_AXI_DDS_CHAN(7, 0, "4B"),
		},
		.num_channels = 13,
		.num_dp_disable_channels = 5,
		.num_dds_channels = 8,
		.num_buf_channels = 4,
	},
	[ID_AD9154] = {
		.name = "AD9154",
		.channel = {
			{
				.type = IIO_TEMP,
				.indexed = 1,
				.channel = 0,
				.scan_index = -1,
				.info_mask_separate =
					BIT(IIO_CHAN_INFO_PROCESSED) |
					BIT(IIO_CHAN_INFO_CALIBBIAS),
			},
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN_BUF(1),
			CF_AXI_DDS_CHAN_BUF(2),
			CF_AXI_DDS_CHAN_BUF(3),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
			CF_AXI_DDS_CHAN(2, 0, "2A"),
			CF_AXI_DDS_CHAN(3, 0, "2B"),
			CF_AXI_DDS_CHAN(4, 0, "3A"),
			CF_AXI_DDS_CHAN(5, 0, "3B"),
			CF_AXI_DDS_CHAN(6, 0, "4A"),
			CF_AXI_DDS_CHAN(7, 0, "4B"),
		},
		.num_channels = 13,
		.num_dp_disable_channels = 5,
		.num_dds_channels = 8,
		.num_buf_channels = 4,
	},
	[ID_AD9152] = {
		.name = "AD9152",
		.channel = {
			{
				.type = IIO_TEMP,
				.indexed = 1,
				.channel = 0,
				.scan_index = -1,
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
		.num_buf_channels = 2,
	},
	[ID_AD9162] = {
		.name = "AD9162",
		.channel = {
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
		},
		.num_channels = 3,
		.num_dds_channels = 2,
		.num_buf_channels = 1,
	},
	[ID_AD9162_COMPLEX] = {
		.name = "AD9162",
		.channel = {
			CF_AXI_DDS_CHAN_BUF_MOD(0, IIO_MOD_I, 0),
			CF_AXI_DDS_CHAN_BUF_MOD(0, IIO_MOD_Q, 1),
			CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
			CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
			CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
			CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
		},
		.num_channels = 6,
		.num_dp_disable_channels = 2,
		.num_dds_channels = 4,
		.num_buf_channels = 2,
	},
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_ad9361 = {
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
	.num_buf_channels = 4,
	.scan_masks = ad9361_available_scan_masks,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_ad9371 = {
	.name = "AD9371",
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
	.num_buf_channels = 4,
	.scan_masks = ad9361_available_scan_masks,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_adrv9009_x2 = {
	.name = "ADRV9009-X2",
	.channel = {
		CF_AXI_DDS_CHAN_BUF(0),
		CF_AXI_DDS_CHAN_BUF(1),
		CF_AXI_DDS_CHAN_BUF(2),
		CF_AXI_DDS_CHAN_BUF(3),
		CF_AXI_DDS_CHAN_BUF(4),
		CF_AXI_DDS_CHAN_BUF(5),
		CF_AXI_DDS_CHAN_BUF(6),
		CF_AXI_DDS_CHAN_BUF(7),
		CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
		CF_AXI_DDS_CHAN(4, 0, "TX2_I_F1"),
		CF_AXI_DDS_CHAN(5, 0, "TX2_I_F2"),
		CF_AXI_DDS_CHAN(6, 0, "TX2_Q_F1"),
		CF_AXI_DDS_CHAN(7, 0, "TX2_Q_F2"),
		CF_AXI_DDS_CHAN(8, 0, "TX3_I_F1"),
		CF_AXI_DDS_CHAN(9, 0, "TX3_I_F2"),
		CF_AXI_DDS_CHAN(10, 0, "TX3_Q_F1"),
		CF_AXI_DDS_CHAN(11, 0, "TX3_Q_F2"),
		CF_AXI_DDS_CHAN(12, 0, "TX4_I_F1"),
		CF_AXI_DDS_CHAN(13, 0, "TX4_I_F2"),
		CF_AXI_DDS_CHAN(14, 0, "TX4_Q_F1"),
		CF_AXI_DDS_CHAN(15, 0, "TX4_Q_F2"),
	},
	.num_channels = 24,
	.num_dds_channels = 16,
	.num_buf_channels = 8,
	.scan_masks = ad9361_2x2_available_scan_masks,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_adrv9009_x4 = {
	.name = "ADRV9009-X4",
	.channel = {
		CF_AXI_DDS_CHAN_BUF(0),
		CF_AXI_DDS_CHAN_BUF(1),
		CF_AXI_DDS_CHAN_BUF(2),
		CF_AXI_DDS_CHAN_BUF(3),
		CF_AXI_DDS_CHAN_BUF(4),
		CF_AXI_DDS_CHAN_BUF(5),
		CF_AXI_DDS_CHAN_BUF(6),
		CF_AXI_DDS_CHAN_BUF(7),
		CF_AXI_DDS_CHAN_BUF(8),
		CF_AXI_DDS_CHAN_BUF(9),
		CF_AXI_DDS_CHAN_BUF(10),
		CF_AXI_DDS_CHAN_BUF(11),
		CF_AXI_DDS_CHAN_BUF(12),
		CF_AXI_DDS_CHAN_BUF(13),
		CF_AXI_DDS_CHAN_BUF(14),
		CF_AXI_DDS_CHAN_BUF(15),
		CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
		CF_AXI_DDS_CHAN(4, 0, "TX2_I_F1"),
		CF_AXI_DDS_CHAN(5, 0, "TX2_I_F2"),
		CF_AXI_DDS_CHAN(6, 0, "TX2_Q_F1"),
		CF_AXI_DDS_CHAN(7, 0, "TX2_Q_F2"),
		CF_AXI_DDS_CHAN(8, 0, "TX3_I_F1"),
		CF_AXI_DDS_CHAN(9, 0, "TX3_I_F2"),
		CF_AXI_DDS_CHAN(10, 0, "TX3_Q_F1"),
		CF_AXI_DDS_CHAN(11, 0, "TX3_Q_F2"),
		CF_AXI_DDS_CHAN(12, 0, "TX4_I_F1"),
		CF_AXI_DDS_CHAN(13, 0, "TX4_I_F2"),
		CF_AXI_DDS_CHAN(14, 0, "TX4_Q_F1"),
		CF_AXI_DDS_CHAN(15, 0, "TX4_Q_F2"),
		CF_AXI_DDS_CHAN(16, 0, "TX5_I_F1"),
		CF_AXI_DDS_CHAN(17, 0, "TX5_I_F2"),
		CF_AXI_DDS_CHAN(18, 0, "TX5_Q_F1"),
		CF_AXI_DDS_CHAN(19, 0, "TX5_Q_F2"),
		CF_AXI_DDS_CHAN(20, 0, "TX6_I_F1"),
		CF_AXI_DDS_CHAN(21, 0, "TX6_I_F2"),
		CF_AXI_DDS_CHAN(22, 0, "TX6_Q_F1"),
		CF_AXI_DDS_CHAN(23, 0, "TX6_Q_F2"),
		CF_AXI_DDS_CHAN(24, 0, "TX7_I_F1"),
		CF_AXI_DDS_CHAN(25, 0, "TX7_I_F2"),
		CF_AXI_DDS_CHAN(26, 0, "TX7_Q_F1"),
		CF_AXI_DDS_CHAN(27, 0, "TX7_Q_F2"),
		CF_AXI_DDS_CHAN(28, 0, "TX8_I_F1"),
		CF_AXI_DDS_CHAN(29, 0, "TX8_I_F2"),
		CF_AXI_DDS_CHAN(30, 0, "TX8_Q_F1"),
		CF_AXI_DDS_CHAN(31, 0, "TX8_Q_F2"),
	},
	.num_channels = 48,
	.num_dds_channels = 32,
	.num_buf_channels = 16,
	.scan_masks = ad9009_4x2_available_scan_masks,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_ad9364 = {
	.name = "AD9364",
	.channel = {
		CF_AXI_DDS_CHAN_BUF(0),
		CF_AXI_DDS_CHAN_BUF(1),
		CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
	},
	.num_channels = 6,
	.num_dds_channels = 4,
	.num_buf_channels = 2,

};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_ad9361x2 = {
	.name = "AD9361",
	.channel = {
		CF_AXI_DDS_CHAN_BUF(0),
		CF_AXI_DDS_CHAN_BUF(1),
		CF_AXI_DDS_CHAN_BUF(2),
		CF_AXI_DDS_CHAN_BUF(3),
		CF_AXI_DDS_CHAN_BUF_VIRT(4),
		CF_AXI_DDS_CHAN_BUF_VIRT(5),
		CF_AXI_DDS_CHAN_BUF_VIRT(6),
		CF_AXI_DDS_CHAN_BUF_VIRT(7),
		CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
		CF_AXI_DDS_CHAN(4, 0, "TX2_I_F1"),
		CF_AXI_DDS_CHAN(5, 0, "TX2_I_F2"),
		CF_AXI_DDS_CHAN(6, 0, "TX2_Q_F1"),
		CF_AXI_DDS_CHAN(7, 0, "TX2_Q_F2"),
	},
	.num_channels = 16,
	.num_dds_channels = 8,
	.num_buf_channels = 8,
	.num_shadow_slave_channels = 4,
	.scan_masks = ad9361_2x2_available_scan_masks,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_ad9936 = {
	.name = "AD9963",
	.channel = {
		CF_AXI_DDS_CHAN(0, 0, "1A"),
		CF_AXI_DDS_CHAN(1, 0, "1B"),
		CF_AXI_DDS_CHAN(2, 0, "2A"),
		CF_AXI_DDS_CHAN(3, 0, "2B"),
	},
	.num_channels = 4,
	.num_dds_channels = 4,
	.num_buf_channels = 0,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_adrv9002_rx2tx2 = {
	.name = "ADRV9002",
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
	.num_buf_channels = 4,
	.scan_masks = adrv9002_rx2tx2_available_scan_masks,
};

static struct cf_axi_dds_chip_info cf_axi_dds_chip_info_adrv9002 = {
	.name = "ADRV9002",
	.channel = {
		CF_AXI_DDS_CHAN_BUF(0),
		CF_AXI_DDS_CHAN_BUF(1),
		CF_AXI_DDS_CHAN(0, 0, "TX1_I_F1"),
		CF_AXI_DDS_CHAN(1, 0, "TX1_I_F2"),
		CF_AXI_DDS_CHAN(2, 0, "TX1_Q_F1"),
		CF_AXI_DDS_CHAN(3, 0, "TX1_Q_F2"),
	},
	.num_channels = 6,
	.num_dds_channels = 4,
	.num_buf_channels = 2,
	.scan_masks = adrv9002_available_scan_masks,
};

static const struct iio_info cf_axi_dds_info = {
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
	.update_scan_mode = &cf_axi_dds_update_scan_mode,
};

static int cf_axi_dds_debugfs_fifo_en_get(void *data, u64 *val)
{
	struct iio_dev *indio_dev = data;
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	*val = st->pl_dma_fifo_en;
	return 0;
}

static int cf_axi_dds_debugfs_fifo_en_set(void *data, u64 val)
{
	struct iio_dev *indio_dev = data;
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	st->pl_dma_fifo_en = !!val;

	return cf_axi_dds_pl_ddr_fifo_ctrl(st, st->pl_dma_fifo_en);
}

DEFINE_DEBUGFS_ATTRIBUTE(cf_axi_dds_debugfs_fifo_en_fops,
			 cf_axi_dds_debugfs_fifo_en_get,
			 cf_axi_dds_debugfs_fifo_en_set,
			 "%llu\n");

static int dds_converter_match(struct device *dev, const void *data)
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

	conv_dev = bus_find_device(&spi_bus_type, NULL,
				   conv_of, dds_converter_match);
	of_node_put(conv_of);
	if (!conv_dev)
		return ERR_PTR(-EPROBE_DEFER);

	return conv_dev;
}

static void dds_converter_put(void *conv_dev)
{
	put_device(conv_dev);
}

static ssize_t cf_axi_dds_ext_info_read(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	long long val;
	unsigned int index;
	int ret = 0;

	cf_axi_dds_lock(st);

	switch (private) {
	case CHANNEL_XBAR:
		index = cf_axi_dds_reg_index(chan);
		val = dds_read(st, ADI_REG_CHAN_CNTRL_7(index));

		if (val & ADI_DAC_ENABLE_MASK)
			val = ADI_TO_DAC_SRC_CH_SEL(val);
		else
			val = index;
		break;
	default:
		ret = -EINVAL;
	}

	cf_axi_dds_unlock(st);

	if (ret == 0)
		ret = sprintf(buf, "%lld\n", val);

	return ret;
}

static ssize_t cf_axi_dds_ext_info_write(struct iio_dev *indio_dev,
				uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	long long readin;
	unsigned int index, val;
	int ret;

	cf_axi_dds_lock(st);

	switch (private) {
	case CHANNEL_XBAR:
		ret = kstrtoll(buf, 10, &readin);
		if (ret)
			goto out;

		if (readin >= st->chip_info->num_buf_channels) {
			ret = -EINVAL;
			goto out;
		}

		index = cf_axi_dds_reg_index(chan);

		val = dds_read(st, ADI_REG_CHAN_CNTRL_7(index));

		val &= ADI_DAC_DDS_SEL(~0);
		val |= ADI_DAC_SRC_CH_SEL(readin);

		if (readin != index)
			val |= ADI_DAC_ENABLE_MASK;

		dds_write(st, ADI_REG_CHAN_CNTRL_7(index), val);

		break;
	default:
		ret = -EINVAL;
	}

out:
	cf_axi_dds_unlock(st);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info axidds_ext_info[] = {
	{
		.name = "channel_crossbar_select",
		.read = cf_axi_dds_ext_info_read,
		.write = cf_axi_dds_ext_info_write,
		.shared = IIO_SEPARATE,
		.private = CHANNEL_XBAR,
	},
	{},
};

static int cf_axi_dds_jesd204_link_supported(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	u32 i, d1, d2, num, multi_device_link;
	bool failed, last;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__, lnk->link_id, jesd204_state_op_reason_str(reason));

	num = ADI_JESD204_TPL_TO_PROFILE_NUM(dds_read(st, ADI_JESD204_REG_TPL_STATUS));

	for (i = 0; i < num; i++) {
		last = (i == (num - 1));
		failed = false;

		dds_write(st, ADI_JESD204_REG_TPL_CNTRL, ADI_JESD204_PROFILE_SEL(i));
		d1 = dds_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_1);
		d2 = dds_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_2);

		if ((ADI_JESD204_TPL_TO_L(d1) / lnk->num_lanes) ==
			(ADI_JESD204_TPL_TO_M(d1) / lnk->num_converters))
			multi_device_link = ADI_JESD204_TPL_TO_L(d1) / lnk->num_lanes;
		else
			multi_device_link = 1;

		if (ADI_JESD204_TPL_TO_L(d1) != lnk->num_lanes * multi_device_link) {
			if (last)
				dev_warn(dev, "profile%u:link_num%u param L mismatch %u!=%u*%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_L(d1), lnk->num_lanes,
					multi_device_link);
			failed = true;
		}

		if (ADI_JESD204_TPL_TO_M(d1) != lnk->num_converters * multi_device_link) {
			if (last)
				dev_warn(dev, "profile%u:link_num%u param M mismatch %u!=%u*%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_M(d1), lnk->num_converters,
					multi_device_link);
			failed = true;
		}

		if (lnk->samples_per_conv_frame && ADI_JESD204_TPL_TO_S(d1) != lnk->samples_per_conv_frame) {
			if (last)
				dev_warn(dev, "profile%u:link_num%u param S mismatch %u!=%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_S(d1), lnk->samples_per_conv_frame);
			failed = true;
		}

		if (ADI_JESD204_TPL_TO_F(d1) != lnk->octets_per_frame) {
			if (last)
				dev_warn(dev, "profile%u:link_num%u param F mismatch %u!=%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_F(d1), lnk->octets_per_frame);
			failed = true;
		}

		if (ADI_JESD204_TPL_TO_NP(d2) != lnk->bits_per_sample) {
			if (last)
				dev_warn(dev, "profile%u:link_num%u param NP mismatch %u!=%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_NP(d2), lnk->bits_per_sample);
			failed = true;
		}

		if (!failed)
			return JESD204_STATE_CHANGE_DONE;
	}

	dev_err(dev, "JESD param mismatch between TPL and Link configuration !\n");

	return JESD204_STATE_CHANGE_DONE;
}

static int cf_axi_dds_jesd204_post_running_stage(struct jesd204_dev *jdev,
					       enum jesd204_state_op_reason reason)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	if (reason == JESD204_STATE_OP_REASON_INIT) {
		cf_axi_dds_start_sync(st, 0);
		return JESD204_STATE_CHANGE_DONE;
	}

	return JESD204_STATE_CHANGE_DONE;

}

static const struct jesd204_dev_data jesd204_cf_axi_dds_init = {
	.state_ops = {
		[JESD204_OP_LINK_SUPPORTED] = {
			.per_link = cf_axi_dds_jesd204_link_supported,
		},
		[JESD204_OP_OPT_POST_RUNNING_STAGE] = {
			.per_device = cf_axi_dds_jesd204_post_running_stage,
			.mode = JESD204_STATE_OP_MODE_PER_DEVICE,
		},
	},
};

struct axidds_core_info {
	unsigned int version;
	bool standalone;
	bool rate_format_skip_en;
	bool complex_modified;
	bool issue_sync_en;
	struct cf_axi_dds_chip_info *chip_info;
	unsigned int data_format;
	unsigned int rate;
	long info_mask_separate;
	const char *name;
	struct iio_chan_spec device_channels[AXIDDS_MAX_NUM_CHANNELS];
	unsigned int num_device_channels;
};

static int cf_axi_dds_setup_chip_info_tbl(struct cf_axi_dds_state *st,
	const struct axidds_core_info *info)
{
	u32 i, c, reg, m, n, np;

	reg = dds_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_1);
	m = ADI_JESD204_TPL_TO_M(reg);

	if (m == 0 || m > ARRAY_SIZE(st->chip_info_generated.channel))
		return -EINVAL;

	reg = dds_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_2);
	n = ADI_JESD204_TPL_TO_N(reg);
	np = roundup_pow_of_two(ADI_JESD204_TPL_TO_NP(reg));

	reg = dds_read(st, ADI_REG_CONFIG);

	for (c = 0, i = 0; i < m; i++, c++) {
		st->chip_info_generated.channel[c].type = IIO_VOLTAGE;
		st->chip_info_generated.channel[c].output = 1;
		st->chip_info_generated.channel[c].indexed = 1;
		st->chip_info_generated.channel[c].modified =
			info->complex_modified ? 1 : 0;
		st->chip_info_generated.channel[c].channel =
			info->complex_modified ? i / 2 : i;
		st->chip_info_generated.channel[c].channel2 =
			(i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		st->chip_info_generated.channel[c].scan_index = i;
		st->chip_info_generated.channel[c].info_mask_shared_by_type =
			BIT(IIO_CHAN_INFO_SAMP_FREQ);

		if (!(reg & ADI_IQCORRECTION_DISABLE))
			st->chip_info_generated.channel[c].info_mask_separate =
			BIT(IIO_CHAN_INFO_CALIBSCALE) |
			BIT(IIO_CHAN_INFO_CALIBPHASE);

		if (reg & ADI_XBAR_ENABLE)
			st->chip_info_generated.channel[c].ext_info =
			axidds_ext_info;

		st->chip_info_generated.channel[c].info_mask_separate |=
			info->info_mask_separate;

		st->chip_info_generated.channel[c].scan_type.realbits = n;
		st->chip_info_generated.channel[c].scan_type.storagebits = np;
		st->chip_info_generated.channel[c].scan_type.shift = np - n;
		st->chip_info_generated.channel[c].scan_type.sign = 's';
	}

	if (!(reg & ADI_DDS_DISABLE)) {
		for (i = 0; i < 2 * m; i++, c++) {
			if (c > ARRAY_SIZE(st->chip_info_generated.channel))
				return -EINVAL;
			st->chip_info_generated.channel[c].type =
				IIO_ALTVOLTAGE;
			st->chip_info_generated.channel[c].output = 1;
			st->chip_info_generated.channel[c].indexed = 1;
			st->chip_info_generated.channel[c].channel = i;
			st->chip_info_generated.channel[c].scan_index = -1;
			st->chip_info_generated.channel
				[c].info_mask_shared_by_type =
				BIT(IIO_CHAN_INFO_SAMP_FREQ);
			st->chip_info_generated.channel[c].info_mask_separate =
				BIT(IIO_CHAN_INFO_RAW) |
				BIT(IIO_CHAN_INFO_SCALE) |
				BIT(IIO_CHAN_INFO_PHASE) |
				BIT(IIO_CHAN_INFO_FREQUENCY);

			st->chip_info_generated.channel[c].ext_info =
				cf_axi_dds_ext_info;
			if (info->complex_modified) {
				if (i < ARRAY_SIZE(dds_extend_names_complex))
					st->chip_info_generated.channel[c].extend_name =
						dds_extend_names_complex[i];
			} else {
				if (i < ARRAY_SIZE(dds_extend_names))
					st->chip_info_generated.channel[c].extend_name =
						dds_extend_names[i];
			}
		}
	}

	st->chip_info_generated.num_dds_channels = i;

	for (i = 0; i < info->num_device_channels; i++, c++) {
		if (c > ARRAY_SIZE(st->chip_info_generated.channel))
			return -EINVAL;
		memcpy(&st->chip_info_generated.channel[c],
			&info->device_channels[i],
			sizeof(info->device_channels[0]));
	}

	st->chip_info_generated.num_channels = c;
	st->chip_info_generated.num_dp_disable_channels = m;
	st->chip_info_generated.num_buf_channels = m;
	st->chip_info_generated.name = info->name;

	return 0;
}

static const struct axidds_core_info ad9122_6_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.rate = 1,
	.data_format = ADI_DATA_FORMAT,
	.issue_sync_en = 1,
};

static const struct axidds_core_info ad9361_6_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate_format_skip_en = true, /* Set by the ad936x_conv driver */
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9361,
};

static const struct axidds_core_info ad9364_6_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate_format_skip_en = true, /* Set by the ad936x_conv driver */
	.rate = 1,
	.chip_info = &cf_axi_dds_chip_info_ad9364,
};

static const struct axidds_core_info ad9361x2_6_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate_format_skip_en = true, /* Set by the ad936x_conv driver */
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9361x2,
	.issue_sync_en = 1,
};

static const struct axidds_core_info ad9144_7_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.rate = 1,
	.issue_sync_en = 1,
	.complex_modified = false,
	.device_channels = {{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.scan_index = -1,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_CALIBBIAS),
	}},
	.num_device_channels = 1,
};

static const struct axidds_core_info ad9739a_8_00_b_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'b'),
	.rate = 1,
	.data_format = ADI_DATA_FORMAT,
	.issue_sync_en = 1,
};

static const struct axidds_core_info ad9783_1_0_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'b'),
	.rate = 1,
};

static const struct axidds_core_info ad9371_6_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9371,
};

static const struct axidds_core_info adrv9009_x2_9_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_adrv9009_x2,
};

static const struct axidds_core_info adrv9009_x4_9_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_adrv9009_x4,
};

static const struct axidds_core_info ad9162_1_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.rate = 1,
};

static const struct axidds_core_info ad9963_1_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.standalone = true,
	.rate = 0,
	.chip_info = &cf_axi_dds_chip_info_ad9936,
};

static const struct axidds_core_info ad9081_1_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 1, 'b'),
	.name = "AD9081",
	.standalone = true,
	.complex_modified = true,
};

static const struct axidds_core_info ad9081_1_00_a_real_info = {
	.version = ADI_AXI_PCORE_VER(9, 1, 'b'),
	.name = "AD9081",
	.standalone = true,
	.complex_modified = false,
};

static const struct axidds_core_info ad9172_1_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 1, 'b'),
	.name = "AD917x",
	.complex_modified = true,
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),
};

static const struct axidds_core_info adrv9002_9_01_b_info = {
	.version = ADI_AXI_PCORE_VER(9, 1, 'b'),
	.standalone = true,
	.rate_format_skip_en = true, /* Set by the ad9002_conv driver */
	.chip_info = &cf_axi_dds_chip_info_adrv9002,
};

static const struct axidds_core_info adrv9002_rx2tx2_9_01_b_info = {
	.version = ADI_AXI_PCORE_VER(9, 1, 'b'),
	.standalone = true,
	.rate_format_skip_en = true, /* Set by the ad9002_conv driver */
	.chip_info = &cf_axi_dds_chip_info_adrv9002_rx2tx2,
};

static const struct axidds_core_info adrv9025_1_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 1, 'b'),
	.name = "ADRV9025",
	.standalone = true,
	.complex_modified = true,
};

/* Match table for of_platform binding */
static const struct of_device_id cf_axi_dds_of_match[] = {
	{ .compatible = "adi,axi-ad9122-6.00.a", .data = &ad9122_6_00_a_info},
	{ .compatible = "adi,axi-ad9136-1.0", .data = &ad9144_7_00_a_info },
	{ .compatible = "adi,axi-ad9144-1.0", .data = &ad9144_7_00_a_info, },
	{ .compatible = "adi,axi-ad9154-1.0", .data = &ad9144_7_00_a_info, },
	{ .compatible = "adi,axi-ad9739a-8.00.b", .data = &ad9739a_8_00_b_info},
	{
	    .compatible = "adi,axi-ad9361x2-dds-6.00.a",
	    .data = &ad9361x2_6_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9361-dds-6.00.a",
	    .data = &ad9361_6_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9364-dds-6.00.a",
	    .data = &ad9364_6_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9371-tx-1.0",
	    .data = &ad9371_6_00_a_info,
	}, {
	    .compatible = "adi,axi-adrv9009-tx-1.0",
	    .data = &ad9371_6_00_a_info,
	}, {
	    .compatible = "adi,axi-adrv9009-x2-tx-1.0",
	    .data = &adrv9009_x2_9_00_a_info,
	}, {
            .compatible = "adi,axi-adrv9009-x4-tx-1.0",
            .data = &adrv9009_x4_9_00_a_info,
        }, {
	    .compatible = "adi,axi-ad9162-1.0",
	    .data = &ad9162_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9783-1.0",
	    .data = &ad9783_1_0_info,
	}, {
	    .compatible = "adi,axi-ad9963-dds-1.00.a",
	    .data = &ad9963_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9172-1.0",
	    .data = &ad9172_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9081-tx-1.0",
	    .data = &ad9081_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9081-tx-1.0-real",
	    .data = &ad9081_1_00_a_real_info,
	}, {
	    .compatible = "adi,axi-adrv9002-tx-1.0",
	    .data = &adrv9002_9_01_b_info
	},{
	    .compatible = "adi,axi-adrv9002-rx2tx2-1.0",
	    .data = &adrv9002_rx2tx2_9_01_b_info
	},{
	    .compatible = "adi,axi-adrv9025-tx-1.0",
	    .data = &adrv9025_1_00_a_info,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

int cf_axi_append_attrs(struct iio_dev *indio_dev,
	const struct attribute_group *add_group, unsigned int skip_cnt)
{
	size_t old_cnt = 0, add_cnt = 0, new_cnt;
	struct attribute **attrs;
	struct attribute_group *group;
	struct iio_info	*iio_info = (struct iio_info *) indio_dev->info;

	if (!add_group)
		return -EINVAL;

	if (indio_dev->info->attrs) {
		attrs = indio_dev->info->attrs->attrs;
		while (*attrs++ != NULL)
			old_cnt++;
	} else if (!skip_cnt) {
		iio_info->attrs = add_group;

		return 0;
	}

	if (add_group->attrs) {
		attrs = add_group->attrs;
		while (*attrs++ != NULL)
			add_cnt++;
	}

	if (skip_cnt > add_cnt)
		return -EINVAL;

	add_cnt -= skip_cnt;
	new_cnt = old_cnt + add_cnt + 1;
	attrs = devm_kcalloc(indio_dev->dev.parent, new_cnt,
		sizeof(*attrs), GFP_KERNEL);
	if (!attrs)
		return -ENOMEM;

	group = devm_kzalloc(indio_dev->dev.parent,
		sizeof(*group), GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	if (old_cnt)
		memcpy(attrs, indio_dev->info->attrs->attrs,
			old_cnt * sizeof(*attrs));
	memcpy(attrs + old_cnt, add_group->attrs, add_cnt * sizeof(*attrs));
	attrs[new_cnt - 1] = NULL;
	group->attrs = attrs;

	iio_info->attrs = group;

	return 0;
}

static void dds_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

static void dds_clk_notifier_unreg(void *data)
{
	struct cf_axi_dds_state *st = data;

	clk_notifier_unregister(st->clk, &st->clk_nb);
}

static int cf_axi_dds_probe(struct platform_device *pdev)
{

	struct device_node *np = pdev->dev.of_node;
	struct cf_axi_converter *conv = NULL;
	const struct axidds_core_info *info;
	const struct of_device_id *id;
	struct cf_axi_dds_state *st;
	struct iio_dev *indio_dev;
	struct resource *res;
	unsigned int ctrl_2, config;
	unsigned int rate;
	unsigned int drp_status;
	int timeout = 100;
	int ret;

	id = of_match_device(cf_axi_dds_of_match, &pdev->dev);
	if (!id || !id->data)
		return -ENODEV;

	info = id->data;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
			np->name);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->indio_dev = indio_dev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs_size = resource_size(res);
	st->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!st->regs)
		return -ENOMEM;

	st->jdev = devm_jesd204_dev_register(&pdev->dev, &jesd204_cf_axi_dds_init);
	if (IS_ERR(st->jdev))
		return PTR_ERR(st->jdev);

	st->data_offload = devm_axi_data_offload_get_optional(&pdev->dev);
	if (IS_ERR(st->data_offload))
		return PTR_ERR(st->data_offload);

	if (info->standalone) {
		st->clk = devm_clk_get(&pdev->dev, "sampl_clk");
		if (IS_ERR(st->clk))
			return PTR_ERR(st->clk);

		of_clk_get_scale(np, "sampl_clk", &st->clkscale);

		ret = clk_prepare_enable(st->clk);
		if (ret < 0)
			return ret;

		ret = devm_add_action_or_reset(&pdev->dev, dds_clk_disable, st->clk);
		if (ret)
			return ret;

		st->dac_clk = clk_get_rate_scaled(st->clk, &st->clkscale);

		st->clk_nb.notifier_call = cf_axi_dds_rate_change;
		clk_notifier_register(st->clk, &st->clk_nb);

		ret = devm_add_action_or_reset(&pdev->dev, dds_clk_notifier_unreg, st);
		if (ret)
			return ret;

		if (info->chip_info) {
			st->chip_info = info->chip_info;
		} else {
			ret = cf_axi_dds_setup_chip_info_tbl(st, info);
			if (ret) {
				dev_err(&pdev->dev,
					"Invalid number of converters identified");
				return ret;
			}

			st->chip_info = &st->chip_info_generated;
		}

		mutex_init(&st->lock);
	} else {
		st->dev_spi = dds_converter_find(&pdev->dev);
		if (IS_ERR(st->dev_spi))
			return PTR_ERR(st->dev_spi);

		ret = devm_add_action_or_reset(&pdev->dev, dds_converter_put, st->dev_spi);
		if (ret)
			return ret;

		conv = to_converter(st->dev_spi);
		if (IS_ERR(conv))
			return PTR_ERR(conv);

		iio_device_set_drvdata(indio_dev, conv);
		conv->indio_dev = indio_dev;
		conv->pcore_sync = cf_axi_dds_sync_frame;
		conv->pcore_set_sed_pattern = cf_axi_dds_set_sed_pattern;
		mutex_init(&conv->lock);

		st->dac_clk = conv->get_data_clk(conv);

		if (conv->id == ID_AUTO_SYNTH_PARAM) {
			ret = cf_axi_dds_setup_chip_info_tbl(st, info);
			if (ret) {
				dev_err(&pdev->dev,
					"Invalid number of converters identified");
				return ret;
			}

			st->chip_info = &st->chip_info_generated;
		} else {
			st->chip_info = &cf_axi_dds_chip_info_tbl[conv->id];
		}
	}

	/*
	 * Sanity check that we did not got 0. Otherwise this will lead to a div by 0 exception.
	 * We will try EPROBE_DEFER as a last resort. Might be that the converter is still
	 * busy...
	 */
	if (!st->dac_clk) {
		dev_err(&pdev->dev, "Cannot have dac_clk=0. Deferring probe...\n");
		return -EPROBE_DEFER;
	}

	st->issue_sync_en = info->issue_sync_en;
	st->standalone = info->standalone;
	st->version = dds_read(st, ADI_AXI_REG_VERSION);

	config = dds_read(st, ADI_REG_CONFIG);
	st->ext_sync_avail = !!(config & ADI_EXT_SYNC);
	st->dp_disable = false; /* FIXME: resolve later which reg & bit to read for this */

	if (ADI_AXI_PCORE_VER_MAJOR(st->version) >
		ADI_AXI_PCORE_VER_MAJOR(info->version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(info->version),
			ADI_AXI_PCORE_VER_MINOR(info->version),
			ADI_AXI_PCORE_VER_PATCH(info->version),
			ADI_AXI_PCORE_VER_MAJOR(st->version),
			ADI_AXI_PCORE_VER_MINOR(st->version),
			ADI_AXI_PCORE_VER_PATCH(st->version));
		return -ENODEV;
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
	dds_write(st, ADI_REG_RSTN, ADI_MMCM_RSTN);

	do {
		drp_status = dds_read(st, ADI_REG_DRP_STATUS);
		if (drp_status & ADI_DRP_LOCKED)
			break;
		msleep(1);
	} while (timeout--);

	if (timeout == -1) {
		dev_err(&pdev->dev, "DRP unlocked.\n");
		return -ETIMEDOUT;
	}

	dds_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

	if (info)
		rate = info->rate;
	else
		rate = 1;

	if (info && !info->rate_format_skip_en)
		dds_write(st, ADI_REG_RATECNTRL, ADI_RATE(rate));

	if (conv && conv->setup) {
		ret = conv->setup(conv);
		if (ret < 0)
			return ret;
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

	if (info && !info->rate_format_skip_en)
		dds_write(st, ADI_REG_CNTRL_2, ctrl_2);

	cf_axi_dds_datasel(st, -1, DATA_SEL_DDS);

	if (!st->dp_disable) {
		u32 scale, frequency, phase, i, skip_attrib = 0;

		scale = 0x1000; /* 0.250 */
		frequency = 40000000;

		of_property_read_u32(np, "adi,axi-dds-default-scale", &scale);
		of_property_read_u32(np, "adi,axi-dds-default-frequency",
				     &frequency);

		for (i = 0; i < st->chip_info->num_dds_channels; i += 2) {
			if ((i / 2) % 2)
				phase = 0;
			else
				phase = 90000;

			cf_axi_dds_default_setup(st, i, phase,
						 frequency, scale);
			cf_axi_dds_default_setup(st, i + 1, phase,
						 frequency, scale);
		}

		for (i = 0; i < st->chip_info->num_buf_channels; i++)
			if (i % 2)
				dds_write(st, ADI_REG_CHAN_CNTRL_8(i),
					ADI_IQCOR_COEFF_2(0x4000) |
					ADI_IQCOR_COEFF_1(0));
			else
				dds_write(st, ADI_REG_CHAN_CNTRL_8(i),
					ADI_IQCOR_COEFF_2(0) |
					ADI_IQCOR_COEFF_1(0x4000));

		cf_axi_dds_update_chan_spec(st, st->chip_info->channel,
				st->chip_info->num_channels);

		if (of_property_read_bool(np,
			"adi,axi-interpolation-core-available")) {
			st->interpolation_factor = 1;

			st->interpolation_gpio = devm_gpiod_get_optional(&pdev->dev,
							      "interpolation",
							      GPIOD_OUT_LOW);
			if (IS_ERR(st->interpolation_gpio))
				dev_err(&pdev->dev, "interpolation gpio error\n");
		} else {
			skip_attrib = 1;
		}

		ret = cf_axi_append_attrs(indio_dev,
			&cf_axi_int_attribute_group, skip_attrib);
		if (ret) {
			dev_err(&pdev->dev,
				"Failed to add sysfs attributes (%d)\n", ret);
			return ret;
		}

	}

	st->enable = true;
	cf_axi_dds_start_sync(st, 0);
	cf_axi_dds_sync_frame(indio_dev);

	if (!st->dp_disable && !dds_read(st, ADI_AXI_REG_ID)) {

		if (st->chip_info->num_shadow_slave_channels) {
			u32 regs[2];

			ret = of_property_read_u32_array(pdev->dev.of_node,
					"slavecore-reg", regs,
					ARRAY_SIZE(regs));
			if (!ret) {
				st->slave_regs = ioremap(regs[0], regs[1]);
				if (st->slave_regs)
					st->have_slave_channels =
						st->chip_info->num_shadow_slave_channels;

			}
		}

		st->pl_dma_fifo_en =
			of_property_read_bool(np, "adi,axi-pl-fifo-enable");

		if (of_find_property(np, "dmas", NULL)) {
			ret = cf_axi_dds_configure_buffer(indio_dev);
			if (ret)
				return ret;

			indio_dev->available_scan_masks =
				st->chip_info->scan_masks;
		}

	} else if (dds_read(st, ADI_AXI_REG_ID)) {
		u32 regs[2];

		ret = of_property_read_u32_array(pdev->dev.of_node,
				"mastercore-reg", regs, ARRAY_SIZE(regs));
		if (!ret)
			st->master_regs = ioremap(regs[0], regs[1]);
	}

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	st->plddrbypass_gpio = devm_gpiod_get_optional(&pdev->dev,
			"plddrbypass", GPIOD_ASIS);
	if ((st->plddrbypass_gpio || st->data_offload)
		&& iio_get_debugfs_dentry(indio_dev))
		debugfs_create_file_unsafe("pl_ddr_fifo_enable", 0644,
				iio_get_debugfs_dentry(indio_dev),
				indio_dev, &cf_axi_dds_debugfs_fifo_en_fops);

	platform_set_drvdata(pdev, indio_dev);

	if (st->jdev) {
		ret = devm_jesd204_fsm_start(&pdev->dev, st->jdev, JESD204_LINKS_ALL);
		if (ret)
			return ret;
	}

	dev_info(&pdev->dev,
		 "Analog Devices CF_AXI_DDS_DDS %s (%d.%.2d.%c) at 0x%08llX mapped to 0x%p, probed DDS %s\n",
		 dds_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER",
		 ADI_AXI_PCORE_VER_MAJOR(st->version),
		 ADI_AXI_PCORE_VER_MINOR(st->version),
		 ADI_AXI_PCORE_VER_PATCH(st->version),
		 (unsigned long long)res->start, st->regs, st->chip_info->name);

	return 0;
}

static struct platform_driver cf_axi_dds_driver = {
	.driver = {
		.name = "cf_axi_dds",
		.owner = THIS_MODULE,
		.of_match_table = cf_axi_dds_of_match,
	},
	.probe		= cf_axi_dds_probe,
};
module_platform_driver(cf_axi_dds_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices DDS");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
