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

#include "cf_axi_dds.h"
#include "ad9122.h"

static const unsigned int interpolation_factors_available[] = {1, 8};

static const char * const dds_extend_names[] = {
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

struct cf_axi_dds_state {
	struct device			*dev_spi;
	struct clk			*clk;
	struct cf_axi_dds_chip_info	*chip_info;
	struct gpio_desc		*plddrbypass_gpio;
	struct gpio_desc		*interpolation_gpio;

	bool				standalone;
	bool				dp_disable;
	bool				enable;
	bool				pl_dma_fifo_en;
	enum fifo_ctrl			gpio_dma_fifo_ctrl;

	struct iio_info			iio_info;
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

int cf_axi_dds_pl_ddr_fifo_ctrl(struct cf_axi_dds_state *st, bool enable)
{
	enum fifo_ctrl mode;
	int ret;

	if (IS_ERR(st->plddrbypass_gpio))
		return -ENODEV;

	mode = (enable ? FIFO_ENABLE : FIFO_DISABLE);

	if (st->gpio_dma_fifo_ctrl == mode)
		return 0;

	ret = gpiod_direction_output(st->plddrbypass_gpio, !enable);
	if (ret == 0)
		st->gpio_dma_fifo_ctrl = mode;

	return ret;
}

static int cf_axi_get_parent_sampling_frequency(struct cf_axi_dds_state *st,
						unsigned long *freq)
{
	struct cf_axi_converter *conv;

	if (st->standalone) {
		*freq = st->dac_clk = clk_get_rate(st->clk);
	} else {
		conv = to_converter(st->dev_spi);
		if (!conv->get_data_clk)
			return -ENODEV;

		*freq = st->dac_clk = conv->get_data_clk(conv);
	}

	return 0;
}

int cf_axi_dds_datasel(struct cf_axi_dds_state *st,
			       int channel, enum dds_data_select sel)
{
	if (channel < 0) { /* ALL */
		unsigned int i;

		for (i = 0; i < st->chip_info->num_buf_channels; i++) {
			if (i > (st->have_slave_channels - 1))
				dds_slave_write(st,
					ADI_REG_CHAN_CNTRL_7(i -
					st->have_slave_channels), sel);
			else
				dds_write(st,
				ADI_REG_CHAN_CNTRL_7(i), sel);
		}
	} else {
		if ((unsigned int)channel > (st->have_slave_channels - 1))
			dds_slave_write(st,
				ADI_REG_CHAN_CNTRL_7(channel -
				st->have_slave_channels), sel);
		else
			dds_write(st, ADI_REG_CHAN_CNTRL_7(channel),
				  sel);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cf_axi_dds_datasel);

static enum dds_data_select cf_axi_dds_get_datasel(struct cf_axi_dds_state *st,
			       int channel)
{
	if (channel < 0)
		channel = 0;

	if ((unsigned int)channel > (st->have_slave_channels - 1))
		return dds_slave_read(st,
			ADI_REG_CHAN_CNTRL_7(channel -
			st->have_slave_channels));

	return dds_read(st, ADI_REG_CHAN_CNTRL_7(channel));
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

void cf_axi_dds_start_sync(struct cf_axi_dds_state *st)
{
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
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
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
		cf_axi_dds_start_sync(st);
		cf_axi_dds_sync_frame(indio_dev);
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

	cf_axi_dds_start_sync(st);
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
	unsigned long parent, val;
	int i, ret;

	if (!frequency)
		return -EINVAL;

	ret = cf_axi_get_parent_sampling_frequency(st, &parent);
	if (ret < 0)
		return ret;

	val = DIV_ROUND_CLOSEST(parent, frequency);

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
	unsigned long freq;
	int i, ret;

	if (!st->interpolation_factor)
		return -ENODEV;

	mutex_lock(&indio_dev->mlock);
	ret = cf_axi_get_parent_sampling_frequency(st, &freq);
	if (ret < 0) {
		mutex_unlock(&indio_dev->mlock);
		return ret;
	}

	for (i = 0, ret = 0; i < ARRAY_SIZE(interpolation_factors_available); i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%ld ",
				freq / interpolation_factors_available[i]);

	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "\n");

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(out_voltage_sampling_frequency_available, S_IRUGO,
		       cf_axi_sampling_frequency_available,
		       NULL,
		       0);

static struct attribute *cf_axi_attributes[] = {
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
	unsigned long long val64;
	unsigned long freq;
	unsigned int reg, channel, phase = 0;
	int ret;

	mutex_lock(&indio_dev->mlock);

	switch (m) {
	case 0:
		if (!chan->output) {
			ret = -EINVAL;
			break;
		}
		*val = st->enable;

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			if (!st->standalone) {
				conv = to_converter(st->dev_spi);
				ret = conv->read_raw(indio_dev,
						     chan, val, val2, m);
				mutex_unlock(&indio_dev->mlock);
				return ret;
			}
		}

		reg = ADI_TO_DDS_SCALE(dds_read(st,
			ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel)));
		cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
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
		ret = cf_axi_get_parent_sampling_frequency(st, &freq);
		if (ret < 0)
			break;

		if (chan->type == IIO_VOLTAGE && st->interpolation_factor)
			freq /= st->interpolation_factor;

		*val = freq;

		mutex_unlock(&indio_dev->mlock);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		/* fall-through */
	case IIO_CHAN_INFO_CALIBSCALE:
		channel = cf_axi_dds_reg_index(chan);

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_8(channel));
		/*  format is 1.1.14 (sign, integer and fractional bits) */

		if (!((phase + channel) % 2))
			reg = ADI_TO_IQCOR_COEFF_1(reg);
		else
			reg = ADI_TO_IQCOR_COEFF_2(reg);

		mutex_unlock(&indio_dev->mlock);
		return cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
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
	struct cf_axi_converter *conv;
	unsigned long long val64;
	unsigned int reg, i, channel, phase = 0;
	int ret = 0;

	if (st->dev_spi)
		conv = to_converter(st->dev_spi);
	else
		conv = ERR_PTR(-ENODEV);

	mutex_lock(&indio_dev->mlock);

	switch (mask) {
	case 0:
		if (!chan->output) {
			ret = -EINVAL;
			break;
		}

		st->enable = !!val;
		cf_axi_dds_start_sync(st);
		cf_axi_dds_datasel(st, -1,
			st->enable ? DATA_SEL_DDS : DATA_SEL_ZERO);

		break;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			if (!st->standalone) {
				if (!IS_ERR(conv))
					ret = conv->write_raw(indio_dev,
						chan, val, val2, mask);
				mutex_unlock(&indio_dev->mlock);
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
		cf_axi_dds_start_sync(st);
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

		cf_axi_dds_start_sync(st);
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
		cf_axi_dds_start_sync(st);
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
		cf_axi_dds_start_sync(st);
		ret = cf_axi_dds_sync_frame(indio_dev);
		break;
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		/* fall-through */
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
		if (!IS_ERR(conv))
			ret = conv->write_raw(indio_dev, chan, val, val2, mask);
		else
			ret = -EINVAL;

	}

err_unlock:
	mutex_unlock(&indio_dev->mlock);

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
	IIO_ENUM_AVAILABLE("scale", &cf_axi_dds_scale_available),
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

static const unsigned long ad9361_available_scan_masks[] = {
	0x01, 0x02, 0x04, 0x08, 0x03, 0x0C, 0x0F,
	0x00,
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
			CF_AXI_DDS_CHAN_BUF(0),
			CF_AXI_DDS_CHAN_BUF(1),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
		},
		.num_channels = 4,
		.num_dds_channels = 2,
		.num_buf_channels = 2,
	},
	[ID_AD9172_M2] = {
		.name = "AD9172",
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
	[ID_AD9172_M4] = {
		.name = "AD9172",
		.channel = {
			CF_AXI_DDS_CHAN_BUF_MOD(0, IIO_MOD_I, 0),
			CF_AXI_DDS_CHAN_BUF_MOD(0, IIO_MOD_Q, 1),
			CF_AXI_DDS_CHAN_BUF_MOD(1, IIO_MOD_I, 2),
			CF_AXI_DDS_CHAN_BUF_MOD(1, IIO_MOD_Q, 3),
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
		.num_dp_disable_channels = 4,
		.num_dds_channels = 8,
		.num_buf_channels = 4,
	},
	[ID_AD9172_M6] = {
		.name = "AD9172",
		.channel = {
			CF_AXI_DDS_CHAN_BUF_MOD(0, IIO_MOD_I, 0),
			CF_AXI_DDS_CHAN_BUF_MOD(0, IIO_MOD_Q, 1),
			CF_AXI_DDS_CHAN_BUF_MOD(1, IIO_MOD_I, 2),
			CF_AXI_DDS_CHAN_BUF_MOD(1, IIO_MOD_Q, 3),
			CF_AXI_DDS_CHAN_BUF_MOD(2, IIO_MOD_I, 4),
			CF_AXI_DDS_CHAN_BUF_MOD(2, IIO_MOD_Q, 5),
			CF_AXI_DDS_CHAN(0, 0, "1A"),
			CF_AXI_DDS_CHAN(1, 0, "1B"),
			CF_AXI_DDS_CHAN(2, 0, "2A"),
			CF_AXI_DDS_CHAN(3, 0, "2B"),
			CF_AXI_DDS_CHAN(4, 0, "3A"),
			CF_AXI_DDS_CHAN(5, 0, "3B"),
			CF_AXI_DDS_CHAN(6, 0, "4A"),
			CF_AXI_DDS_CHAN(7, 0, "4B"),
			CF_AXI_DDS_CHAN(8, 0, "5A"),
			CF_AXI_DDS_CHAN(9, 0, "5B"),
			CF_AXI_DDS_CHAN(10, 0, "6A"),
			CF_AXI_DDS_CHAN(11, 0, "6B"),
		},
		.num_channels = 18,
		.num_dp_disable_channels = 6,
		.num_dds_channels = 12,
		.num_buf_channels = 6,
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

static const struct iio_info cf_axi_dds_info = {
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
	.update_scan_mode = &cf_axi_dds_update_scan_mode,
};

static ssize_t cf_axi_dds_debugfs_read(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	char buf[80];

	ssize_t len = sprintf(buf, "%d\n", st->pl_dma_fifo_en);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t cf_axi_dds_debugfs_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	char buf[80], *p = buf;
	int ret;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(p, userbuf, count))
		return -EFAULT;

	p[count] = 0;

	ret = strtobool(p, &st->pl_dma_fifo_en);
	if (ret < 0)
		return -EINVAL;

	ret = cf_axi_dds_pl_ddr_fifo_ctrl(st, st->pl_dma_fifo_en);
	if (ret)
		return ret;

	return count;
}

static const struct file_operations cf_axi_dds_debugfs_fops = {
	.open = simple_open,
	.read = cf_axi_dds_debugfs_read,
	.write = cf_axi_dds_debugfs_write,
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

	conv_dev = bus_find_device(&spi_bus_type, NULL,
				   conv_of, dds_converter_match);
	of_node_put(conv_of);
	if (!conv_dev)
		return ERR_PTR(-EPROBE_DEFER);

	return conv_dev;
}

static void dds_converter_put(struct device *conv_dev)
{
	put_device(conv_dev);
}

static int cf_axi_dds_setup_chip_info_tbl(struct cf_axi_dds_state *st,
					  const char *name, bool complex)
{
	u32 i, c, reg, m, n, np;

	reg = dds_read(st, ADI_REG_TPL_DESCRIPTOR_1);
	m = ADI_TO_JESD_M(reg);

	if (m == 0 || m > ARRAY_SIZE(st->chip_info_generated.channel))
		return -EINVAL;

	reg = dds_read(st, ADI_REG_TPL_DESCRIPTOR_2);
	n = ADI_TO_JESD_N(reg);
	np = ADI_TO_JESD_NP(reg);

	reg = dds_read(st, ADI_REG_CONFIG);

	for (c = 0, i = 0; i < m; i++, c++) {
		st->chip_info_generated.channel[c].type = IIO_VOLTAGE;
		st->chip_info_generated.channel[c].output = 1;
		st->chip_info_generated.channel[c].indexed = 1;
		st->chip_info_generated.channel[c].modified = complex ? 1 : 0;
		st->chip_info_generated.channel[c].channel =
			complex ? i / 2 : i;
		st->chip_info_generated.channel[c].channel2 =
			(i & 1) ? IIO_MOD_Q : IIO_MOD_I;
		st->chip_info_generated.channel[c].scan_index = i;
		st->chip_info_generated.channel[c].info_mask_shared_by_type =
		BIT(IIO_CHAN_INFO_SAMP_FREQ);

		if (!(reg & ADI_IQCORRECTION_DISABLE))
			st->chip_info_generated.channel[c].info_mask_separate =
			BIT(IIO_CHAN_INFO_CALIBSCALE) |
			BIT(IIO_CHAN_INFO_CALIBPHASE);

		st->chip_info_generated.channel[c].scan_type.realbits = n;
		st->chip_info_generated.channel[c].scan_type.storagebits = np;
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
			if (i < ARRAY_SIZE(dds_extend_names))
				st->chip_info_generated.channel[
					c].extend_name = dds_extend_names[i];
		}
	}

	st->chip_info_generated.num_channels = c;
	st->chip_info_generated.num_dp_disable_channels = m;
	st->chip_info_generated.num_dds_channels = i;
	st->chip_info_generated.num_buf_channels = m;
	st->chip_info_generated.name = name;

	return 0;
}

struct axidds_core_info {
	unsigned int version;
	bool standalone;
	bool rate_format_skip_en;
	bool complex_modified;
	struct cf_axi_dds_chip_info *chip_info;
	unsigned int data_format;
	unsigned int rate;
	const char *name;
};

static const struct axidds_core_info ad9122_6_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.rate = 1,
	.data_format = ADI_DATA_FORMAT,
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
};

static const struct axidds_core_info ad9144_7_00_a_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'a'),
	.rate = 1,
};

static const struct axidds_core_info ad9739a_8_00_b_info = {
	.version = ADI_AXI_PCORE_VER(9, 0, 'b'),
	.rate = 1,
	.data_format = ADI_DATA_FORMAT,
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
	    .compatible = "adi,axi-ad9162-1.0",
	    .data = &ad9162_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9963-dds-1.00.a",
	    .data = &ad9963_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9172-1.0",
	    .data = &ad9162_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9081-tx-1.0",
	    .data = &ad9081_1_00_a_info,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

static int cf_axi_dds_probe(struct platform_device *pdev)
{

	struct device_node *np = pdev->dev.of_node;
	struct cf_axi_converter *conv = NULL;
	const struct axidds_core_info *info;
	const struct of_device_id *id;
	struct cf_axi_dds_state *st;
	struct iio_dev *indio_dev;
	struct resource *res;
	unsigned int ctrl_2;
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

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs_size = resource_size(res);
	st->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (!st->regs)
		return -ENOMEM;

	if (info->standalone) {
		st->clk = devm_clk_get(&pdev->dev, "sampl_clk");
		if (IS_ERR(st->clk))
			return PTR_ERR(st->clk);

		ret = clk_prepare_enable(st->clk);
		if (ret < 0)
			return ret;

		st->dac_clk = clk_get_rate(st->clk);

		st->clk_nb.notifier_call = cf_axi_dds_rate_change;
		clk_notifier_register(st->clk, &st->clk_nb);

		if (info->chip_info) {
			st->chip_info = info->chip_info;
		} else {
			ret = cf_axi_dds_setup_chip_info_tbl(st, info->name,
					info->complex_modified);
			if (ret) {
				dev_err(&pdev->dev,
					"Invalid number of converters identified");
				return ret;
			}

			st->chip_info = &st->chip_info_generated;
		}
	} else {
		st->dev_spi = dds_converter_find(&pdev->dev);
		if (IS_ERR(st->dev_spi))
			return PTR_ERR(st->dev_spi);

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

	st->standalone = info->standalone;
	st->version = dds_read(st, ADI_AXI_REG_VERSION);
	st->dp_disable = dds_read(st, ADI_REG_DAC_DP_DISABLE);

	if (ADI_AXI_PCORE_VER_MAJOR(st->version) >
		ADI_AXI_PCORE_VER_MAJOR(info->version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(info->version),
			ADI_AXI_PCORE_VER_MINOR(info->version),
			ADI_AXI_PCORE_VER_PATCH(info->version),
			ADI_AXI_PCORE_VER_MAJOR(st->version),
			ADI_AXI_PCORE_VER_MINOR(st->version),
			ADI_AXI_PCORE_VER_PATCH(st->version));
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
	dds_write(st, ADI_REG_RSTN, ADI_MMCM_RSTN);

	do {
		drp_status = dds_read(st, ADI_REG_DRP_STATUS);
		if (drp_status & ADI_DRP_LOCKED)
			break;
		msleep(1);
	} while (timeout--);

	if (timeout == -1) {
		dev_err(&pdev->dev, "DRP unlocked.\n");
		ret = -ETIMEDOUT;
		goto err_converter_put;
	}

	dds_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

	if (info)
		rate = info->rate;
	else
		rate = 1;

	if (info && !info->rate_format_skip_en)
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

	if (info && !info->rate_format_skip_en)
		dds_write(st, ADI_REG_CNTRL_2, ctrl_2);

	cf_axi_dds_datasel(st, -1, DATA_SEL_DDS);

	if (!st->dp_disable) {
		u32 scale, frequency, phase, i;

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
			WARN_ON(st->iio_info.attrs != NULL);
			st->iio_info.attrs = &cf_axi_int_attribute_group;

			st->interpolation_gpio = devm_gpiod_get_optional(&pdev->dev,
							      "interpolation",
							      GPIOD_OUT_LOW);
			if (IS_ERR(st->interpolation_gpio))
				dev_err(&pdev->dev, "interpolation gpio error\n");
		}
	}

	st->enable = true;
	cf_axi_dds_start_sync(st);
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
				goto err_converter_put;

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

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_buffer;

	dev_info(&pdev->dev, "Analog Devices CF_AXI_DDS_DDS %s (%d.%.2d.%c) at 0x%08llX mapped"
		" to 0x%p, probed DDS %s\n",
		dds_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER",
		ADI_AXI_PCORE_VER_MAJOR(st->version),
		ADI_AXI_PCORE_VER_MINOR(st->version),
		ADI_AXI_PCORE_VER_PATCH(st->version),
		(unsigned long long)res->start, st->regs, st->chip_info->name);

	st->plddrbypass_gpio = devm_gpiod_get(&pdev->dev, "plddrbypass", GPIOD_ASIS);
	if (!IS_ERR(st->plddrbypass_gpio) && iio_get_debugfs_dentry(indio_dev))
		debugfs_create_file("pl_ddr_fifo_enable", 0644,
				    iio_get_debugfs_dentry(indio_dev),
				    indio_dev, &cf_axi_dds_debugfs_fops);

	platform_set_drvdata(pdev, indio_dev);

	return 0;

err_unconfigure_buffer:
	cf_axi_dds_unconfigure_buffer(indio_dev);
err_converter_put:
	if (st->dev_spi)
		dds_converter_put(st->dev_spi);
	if (st->clk) {
		clk_notifier_unregister(st->clk, &st->clk_nb);
		clk_disable_unprepare(st->clk);
	}

	return ret;
}

static int cf_axi_dds_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);

	if (!st->dp_disable && !dds_read(st, ADI_AXI_REG_ID) &&
		of_find_property(pdev->dev.of_node, "dmas", NULL))
		cf_axi_dds_unconfigure_buffer(indio_dev);

	if (st->dev_spi)
		dds_converter_put(st->dev_spi);
	if (st->clk) {
		clk_notifier_unregister(st->clk, &st->clk_nb);
		clk_disable_unprepare(st->clk);
	}

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
