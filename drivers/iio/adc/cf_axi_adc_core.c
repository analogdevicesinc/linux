/*
 * AXI_ADC ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
 */

#include <linux/gpio/consumer.h>
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
#include <linux/iio/buffer_impl.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include <linux/jesd204/jesd204.h>
#include <linux/jesd204/adi-common.h>

#include "cf_axi_adc.h"

const unsigned int decimation_factors_available[] = {1, 8};

struct axiadc_core_info {
	unsigned int version;
};

struct axiadc_state {
	struct device 			*dev_spi;
	struct iio_info			iio_info;
	struct clk 			*clk;
	struct gpio_desc		*gpio_decimation;
	struct jesd204_dev 		*jdev;
	size_t				regs_size;
	void __iomem			*regs;
	void __iomem			*slave_regs;
	unsigned int			max_usr_channel;
	unsigned int			id;
	unsigned int			pcore_version;
	unsigned int			decimation_factor;
	unsigned long long		adc_clk;
	unsigned int			have_slave_channels;
	bool				additional_channel;
	bool				dp_disable;
	bool				ext_sync_avail;

	struct iio_chan_spec		channels[AXIADC_MAX_CHANNEL];
};

struct axiadc_converter *to_converter(struct device *dev)
{
	struct axiadc_converter *conv = spi_get_drvdata(to_spi_device(dev));

	if (conv)
		return conv;

	return ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(to_converter);

void axiadc_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->regs + reg);
}
EXPORT_SYMBOL_GPL(axiadc_write);

unsigned int axiadc_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->regs + reg);
}
EXPORT_SYMBOL_GPL(axiadc_read);

void axiadc_slave_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	iowrite32(val, st->slave_regs + reg);
}
EXPORT_SYMBOL_GPL(axiadc_slave_write);

unsigned int axiadc_slave_read(struct axiadc_state *st, unsigned reg)
{
	return ioread32(st->slave_regs + reg);
}
EXPORT_SYMBOL_GPL(axiadc_slave_read);

void axiadc_idelay_set(struct axiadc_state *st, unsigned lane, unsigned val)
{
	axiadc_write(st, ADI_REG_DELAY(lane), val);
}
EXPORT_SYMBOL_GPL(axiadc_idelay_set);

static int axiadc_hw_submit_block(struct iio_dma_buffer_queue *queue,
	struct iio_dma_buffer_block *block)
{
	struct iio_dev *indio_dev = queue->driver_data;
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_dmaengine_buffer_submit_block(queue, block);

	axiadc_write(st, ADI_REG_STATUS, ~0);
	axiadc_write(st, ADI_REG_DMA_STATUS, ~0);

	return 0;
}

static const struct iio_dma_buffer_ops axiadc_dma_buffer_ops = {
	.submit = axiadc_hw_submit_block,
	.abort = iio_dmaengine_buffer_abort,
};

static int axiadc_configure_ring_stream(struct iio_dev *indio_dev,
	const char *dma_name)
{
	struct iio_buffer *buffer;

	if (dma_name == NULL)
		dma_name = "rx";

	buffer = devm_iio_dmaengine_buffer_alloc(indio_dev->dev.parent, dma_name,
						 &axiadc_dma_buffer_ops, indio_dev);
	if (IS_ERR(buffer))
		return PTR_ERR(buffer);

	indio_dev->modes |= INDIO_BUFFER_HARDWARE;
	iio_device_attach_buffer(indio_dev, buffer);

	return 0;
}

static int axiadc_chan_to_regoffset(struct iio_chan_spec const *chan)
{
	if (chan->modified)
		return chan->scan_index;

	return chan->channel;
}

int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	unsigned reg;

	reg = axiadc_read(st, ADI_REG_CHAN_CNTRL_3(channel));
	reg &= ~ADI_ADC_PN_SEL(~0);
	reg |= ADI_ADC_PN_SEL(sel);
	axiadc_write(st, ADI_REG_CHAN_CNTRL_3(channel), reg);

	return 0;
}
EXPORT_SYMBOL_GPL(axiadc_set_pnsel);

enum adc_pn_sel axiadc_get_pnsel(struct axiadc_state *st,
			       int channel, const char **name)
{
	unsigned val;
	const char *ident[] = {"PN9", "PN23A", "UNDEF", "UNDEF",
			"PN7", "PN15", "PN23", "PN31", "UNDEF", "PN_CUSTOM"};

	val = ADI_TO_ADC_PN_SEL(axiadc_read(st, ADI_REG_CHAN_CNTRL_3(channel)));

	if (name) {
		if (val >= ARRAY_SIZE(ident))
			*name = "UNDEF";
		else
			*name = ident[val];
	}

	return val;
}

static unsigned int axiadc_num_phys_channels(struct axiadc_state *st)
{
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	return conv->chip_info->num_channels - st->have_slave_channels;
}

static ssize_t axiadc_debugfs_pncheck_read(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct axiadc_state *st = iio_priv(indio_dev);
	char buf[1000];
	const char *pn_name;
	ssize_t len = 0;
	unsigned stat, i;

	for (i = 0; i < axiadc_num_phys_channels(st); i++) {
		stat = axiadc_read(st, ADI_REG_CHAN_STATUS(i));
		axiadc_get_pnsel(st, i, &pn_name);

		len += sprintf(buf + len, "CH%d : %s : %s %s\n", i, pn_name,
			(stat & ADI_PN_OOS) ? "Out of Sync :" : "In Sync :",
			(stat & (ADI_PN_ERR | ADI_PN_OOS)) ? "PN Error" : "OK");
		axiadc_write(st, ADI_REG_CHAN_STATUS(i), ~0);
		if (len > 955)
			return -ENOMEM;
	}

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t axiadc_debugfs_pncheck_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	enum adc_pn_sel mode;
	unsigned int i;
	char buf[80], *p = buf;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(p, userbuf, count))
		return -EFAULT;

	p[count] = 0;

	if (sysfs_streq(p, "PN9"))
		mode = ADC_PN9;
	else if (sysfs_streq(p, "PN23"))
		mode = ADC_PN23A;
	else
		mode = ADC_PN_OFF;

	mutex_lock(&conv->lock);

	for (i = 0; i < axiadc_num_phys_channels(st); i++) {
		if (conv->set_pnsel)
			conv->set_pnsel(indio_dev, i, mode);

		if (mode != ADC_PN_OFF)
			axiadc_set_pnsel(st, i, mode);
	}

	mdelay(1);

	for (i = 0; i < axiadc_num_phys_channels(st); i++)
		axiadc_write(st, ADI_REG_CHAN_STATUS(i), ~0);

	mutex_unlock(&conv->lock);

	return count;
}

static const struct file_operations axiadc_debugfs_pncheck_fops = {
	.open = simple_open,
	.read = axiadc_debugfs_pncheck_read,
	.write = axiadc_debugfs_pncheck_write,
};

static int axiadc_reg_access(struct iio_dev *indio_dev,
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	int ret;

	/* Check that the register is in range and aligned */
	if ((reg & DEBUGFS_DRA_PCORE_REG_MAGIC) &&
	    ((reg & 0xffff) >= st->regs_size || (reg & 0x3)))
		return -EINVAL;

	mutex_lock(&conv->lock);

	if (!(reg & DEBUGFS_DRA_PCORE_REG_MAGIC)) {
		struct axiadc_converter *conv = to_converter(st->dev_spi);
		if (IS_ERR(conv))
			ret = PTR_ERR(conv);
		else if (!conv->reg_access)
			ret = -ENODEV;
		else
			ret = conv->reg_access(indio_dev, reg, writeval, readval);
	} else {
		if (readval == NULL)
			axiadc_write(st, reg & 0xFFFF, writeval);
		else
			*readval = axiadc_read(st, reg & 0xFFFF);
		ret = 0;
	}
	mutex_unlock(&conv->lock);

	return 0;
}

static int axiadc_decimation_set(struct axiadc_state *st,
				 unsigned int decimation_factor)
{
	int ret = 0;
	u32 reg;

	switch (decimation_factor) {
		case 1:
		case 8:
			reg = axiadc_read(st, ADI_REG_GP_CONTROL);

			if (decimation_factor == 8)
				reg |= BIT(0);
			else
				reg &= ~BIT(0);

			if (st->gpio_decimation)
				gpiod_set_value(st->gpio_decimation,
						reg & BIT(0));
			else
				axiadc_write(st, ADI_REG_GP_CONTROL, reg);
			st->decimation_factor = decimation_factor;
			break;
		default:
			ret = -EINVAL;
	}

	return ret;
}

static int axiadc_get_parent_sampling_frequency(struct axiadc_state *st, unsigned long *freq)
{
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	int ret = 0;

	if (conv->clk)
		*freq = clk_get_rate(conv->clk);
	else
		ret = -ENODEV;

	return ret;
}

static ssize_t axiadc_decimation_store(struct axiadc_state *st,
				       unsigned long frequency)
{
	unsigned long parent, val;
	int i, ret;

	if (!frequency)
		return -EINVAL;

	ret = axiadc_get_parent_sampling_frequency(st, &parent);
	if (ret < 0)
		return ret;

	val = DIV_ROUND_CLOSEST(parent, frequency);

	for (i = 0; i < ARRAY_SIZE(decimation_factors_available); i++) {
		if (val == decimation_factors_available[i])
			return axiadc_decimation_set(st, val);
	}

	return -EINVAL;
}

static ssize_t axiadc_sampling_frequency_available(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	unsigned long freq;
	int i, ret;

	if (!st->decimation_factor)
		return -ENODEV;

	mutex_lock(&conv->lock);
	ret = axiadc_get_parent_sampling_frequency(st, &freq);
	if (ret < 0) {
		mutex_unlock(&conv->lock);
		return ret;
	}

	for (ret = 0, i = 0; i < ARRAY_SIZE(decimation_factors_available); i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%ld ",
				freq / decimation_factors_available[i]);

	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "\n");

	mutex_unlock(&conv->lock);

	return ret;
}

static const char * const axiadc_sync_ctrls[] = {
	"arm", "disarm", "trigger_manual",
};

static ssize_t axiadc_sync_start_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	int ret;

	ret = sysfs_match_string(axiadc_sync_ctrls, buf);
	if (ret < 0)
		return ret;

	mutex_lock(&conv->lock);
	if (st->ext_sync_avail) {
		switch (ret) {
		case 0:
			axiadc_write(st, ADI_REG_CNTRL_2, ADI_EXT_SYNC_ARM);
			break;
		case 1:
			axiadc_write(st, ADI_REG_CNTRL_2, ADI_EXT_SYNC_DISARM);
			break;
		case 2:
			axiadc_write(st, ADI_REG_CNTRL_2, ADI_MANUAL_SYNC_REQUEST);
			break;
		default:
			ret = -EINVAL;
		}
	} else if (ret == 0) {
		u32 reg;

		reg = axiadc_read(st, ADI_REG_CNTRL);
		axiadc_write(st, ADI_REG_CNTRL, reg | ADI_SYNC);
	}
	mutex_unlock(&conv->lock);

	return ret < 0 ? ret : len;
}

static ssize_t axiadc_sync_start_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct axiadc_state *st = iio_priv(indio_dev);
	u32 reg;

	switch ((u32)this_attr->address) {
	case 0:
		reg = axiadc_read(st, ADI_REG_SYNC_STATUS);

		return sprintf(buf, "%s\n", reg & ADI_ADC_SYNC_STATUS ?
			axiadc_sync_ctrls[0] : axiadc_sync_ctrls[1]);
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
		       axiadc_sync_start_show,
		       axiadc_sync_start_store,
		       0);

static IIO_DEVICE_ATTR(sync_start_enable_available, 0444,
		       axiadc_sync_start_show,
		       NULL,
		       1);

static IIO_DEVICE_ATTR(in_voltage_sampling_frequency_available, S_IRUGO,
		       axiadc_sampling_frequency_available,
		       NULL,
		       0);

static struct attribute *axiadc_attributes[] = {
	&iio_dev_attr_sync_start_enable.dev_attr.attr,
	&iio_dev_attr_sync_start_enable_available.dev_attr.attr,
	&iio_dev_attr_in_voltage_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group axiadc_dec_attribute_group = {
	.attrs = axiadc_attributes,
};

static int axiadc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	int ret, sign;
	unsigned tmp, phase = 0, channel;
	unsigned long long llval;

	channel = axiadc_chan_to_regoffset(chan);

	switch (m) {
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		fallthrough;
	case IIO_CHAN_INFO_CALIBSCALE:
		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_2(channel));
		/*  format is 1.1.14 (sign, integer and fractional bits) */

		if (!((phase + channel) % 2)) {
			tmp = ADI_TO_IQCOR_COEFF_1(tmp);
		} else {
			tmp = ADI_TO_IQCOR_COEFF_2(tmp);
		}

		if (tmp & 0x8000)
			sign = -1;
		else
			sign = 1;

		if (tmp & 0x4000)
			*val = 1 * sign;
		else
			*val = 0;

		tmp &= ~0xC000;

		llval = tmp * 1000000ULL + (0x4000 / 2);
		do_div(llval, 0x4000);
		if (*val == 0)
			*val2 = llval * sign;
		else
			*val2 = llval;

		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_1(channel));
		*val = (short)ADI_TO_DCFILT_OFFSET(tmp);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		/*
		 * approx: F_cut = C * Fsample / (2 * pi)
		 */

		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL(channel));
		if (!(tmp & ADI_DCFILT_ENB)) {
			*val = 0;
			return IIO_VAL_INT;
		}

		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_1(channel));
		llval = ADI_TO_DCFILT_COEFF(tmp) * (unsigned long long)conv->adc_clk;
		do_div(llval, 102944); /* 2 * pi * 0x4000 */
		*val = llval;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val2 = 0;
		ret = conv->read_raw(indio_dev, chan, val, val2, m);
		llval = (((u64)*val2) << 32) | (u32)*val;

		if (ret < 0 || !llval) {
			tmp = ADI_TO_CLK_FREQ(axiadc_read(st, ADI_REG_CLK_FREQ));
			llval = tmp * 100000000ULL /* FIXME */ * ADI_TO_CLK_RATIO(axiadc_read(st, ADI_REG_CLK_RATIO));
			llval = llval >> 16;
		}

		if (chan->extend_name && !strcmp(chan->extend_name, "user_logic")) {
			tmp = axiadc_read(st,
				ADI_REG_CHAN_USR_CNTRL_2(channel));

			llval = ADI_TO_USR_DECIMATION_M(tmp) * conv->adc_clk;
			do_div(llval, ADI_TO_USR_DECIMATION_N(tmp));
		}

		if (st->decimation_factor)
			do_div(llval, st->decimation_factor);

		*val = lower_32_bits(llval);
		*val2 = upper_32_bits(llval);

		return IIO_VAL_INT_64;
	default:
		return conv->read_raw(indio_dev, chan, val, val2, m);

	}

	return -EINVAL;
}

static int axiadc_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	unsigned fract, tmp, phase = 0, channel;
	unsigned long long llval;

	channel = axiadc_chan_to_regoffset(chan);

	switch (mask) {
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
		fallthrough;
	case IIO_CHAN_INFO_CALIBSCALE:
		/*  format is 1.1.14 (sign, integer and fractional bits) */
		switch (val) {
		case 1:
			fract = 0x4000;
			break;
		case -1:
			fract = 0xC000;
			break;
		case 0:
			fract = 0;
			if (val2 < 0) {
				fract = 0x8000;
				val2 *= -1;
			}
			break;
		default:
			return -EINVAL;
		}

		llval = (unsigned long long)val2 * 0x4000UL + (1000000UL / 2);
		do_div(llval, 1000000UL);
		fract |= llval;

		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_2(channel));

		if (!((channel + phase) % 2)) {
			tmp &= ~ADI_IQCOR_COEFF_1(~0);
			tmp |= ADI_IQCOR_COEFF_1(fract);
		} else {
			tmp &= ~ADI_IQCOR_COEFF_2(~0);
			tmp |= ADI_IQCOR_COEFF_2(fract);
		}

		axiadc_write(st, ADI_REG_CHAN_CNTRL_2(channel), tmp);

		return 0;

	case IIO_CHAN_INFO_HIGH_PASS_FILTER_3DB_FREQUENCY:
		/* C = 1 – e^(-2 * pi * F_cut / Fsample)
		 * approx: C = 2 * pi * F_cut / Fsample
		 */

		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL(channel));

		if (val == 0 && val2 == 0) {
			tmp &= ~ADI_DCFILT_ENB;
			axiadc_write(st, ADI_REG_CHAN_CNTRL(channel), tmp);
			return 0;
		}

		tmp |= ADI_DCFILT_ENB;

		llval = 102944ULL * val; /* 2 * pi * 0x4000 * val */
		do_div(llval, conv->adc_clk);

		axiadc_write(st, ADI_REG_CHAN_CNTRL_1(channel),
			     ADI_DCFILT_COEFF(clamp_t(unsigned short, llval, 1, 0x4000)));
		axiadc_write(st, ADI_REG_CHAN_CNTRL(channel), tmp);

		return 0;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axiadc_read(st, ADI_REG_CHAN_CNTRL_1(channel));
		tmp &= ~ADI_DCFILT_OFFSET(~0);
		tmp |= ADI_DCFILT_OFFSET((short)val);

		axiadc_write(st, ADI_REG_CHAN_CNTRL_1(channel), tmp);
		return 0;

	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->decimation_factor)
			return axiadc_decimation_store(st, val);

		return conv->write_raw(indio_dev, chan, val, val2, mask);

	default:
		return conv->write_raw(indio_dev, chan, val, val2, mask);
	}
}

static int axiadc_read_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int *val,
	int *val2)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (conv->read_event_value)
		return conv->read_event_value(indio_dev, chan,
				type, dir, info, val, val2);
	else
		return -ENOSYS;
}

static int axiadc_write_event_value(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, enum iio_event_type type,
	enum iio_event_direction dir, enum iio_event_info info, int val,
	int val2)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (conv->write_event_value)
		return conv->write_event_value(indio_dev, chan,
				type, dir, info, val, val2);
	else
		return -ENOSYS;
}

static int axiadc_read_event_config(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (conv->read_event_config)
		return conv->read_event_config(indio_dev, chan, type, dir);
	else
		return -ENOSYS;
}

static int axiadc_write_event_config(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     int state)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (conv->write_event_config)
		return conv->write_event_config(indio_dev,
				chan, type, dir, state);
	else
		return -ENOSYS;
}

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, ctrl;

	for (i = 0; i < indio_dev->masklength; i++) {
		if (i > (st->have_slave_channels - 1))
			ctrl = axiadc_slave_read(st,
				ADI_REG_CHAN_CNTRL(i - st->have_slave_channels));
		else
			ctrl = axiadc_read(st, ADI_REG_CHAN_CNTRL(i));

		if (test_bit(i, scan_mask))
			ctrl |= ADI_ENABLE;
		else
			ctrl &= ~ADI_ENABLE;


		if (i > (st->have_slave_channels - 1))
			axiadc_slave_write(st,
				ADI_REG_CHAN_CNTRL(i - st->have_slave_channels),
				ctrl);
		else
			axiadc_write(st, ADI_REG_CHAN_CNTRL(i), ctrl);
	}

	return 0;
}

static int axiadc_channel_setup(struct iio_dev *indio_dev,
				const struct iio_chan_spec *adc_channels,
				unsigned adc_chan_num)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned i, cnt, usr_ctrl;

	st->max_usr_channel = ADI_USR_CHANMAX(axiadc_read(st, ADI_REG_USR_CNTRL_1));
	st->max_usr_channel = 0; /* FIXME */

	for (i = 0, cnt = 0; i < adc_chan_num; i++)
		st->channels[cnt++] = adc_channels[i];

	if (st->additional_channel && cnt < AXIADC_MAX_CHANNEL) {
		st->channels[cnt] = adc_channels[0];
		st->channels[cnt].channel = cnt;
		st->channels[cnt].scan_index = cnt;
		cnt++;
	}

	for (i = 0; i < st->max_usr_channel; i++) {
		usr_ctrl = axiadc_read(st, ADI_REG_CHAN_USR_CNTRL_1(cnt));
		st->channels[cnt].type = IIO_VOLTAGE;
		st->channels[cnt].indexed = 1,
		st->channels[cnt].channel = cnt;
		st->channels[cnt].scan_index = cnt;
		st->channels[cnt].info_mask_separate = BIT(IIO_CHAN_INFO_SAMP_FREQ);
		st->channels[cnt].extend_name = "user_logic";
		st->channels[cnt].scan_type.sign = (usr_ctrl & ADI_USR_DATATYPE_SIGNED) ? 's' : 'u';
		st->channels[cnt].scan_type.realbits = ADI_TO_USR_DATATYPE_BITS(usr_ctrl);
		st->channels[cnt].scan_type.storagebits = ADI_TO_USR_DATATYPE_TOTAL_BITS(usr_ctrl);
		st->channels[cnt].scan_type.shift = ADI_TO_USR_DATATYPE_SHIFT(usr_ctrl);
		st->channels[cnt].scan_type.endianness = (usr_ctrl & ADI_USR_DATATYPE_BE) ? IIO_BE : IIO_LE;
		cnt++;
	}

	indio_dev->channels = st->channels;
	indio_dev->num_channels = cnt;

	return 0;
}

static struct iio_info axiadc_info = {
	.read_raw = &axiadc_read_raw,
	.write_raw = &axiadc_write_raw,
	.read_event_value = &axiadc_read_event_value,
	.write_event_value = &axiadc_write_event_value,
	.read_event_config = &axiadc_read_event_config,
	.write_event_config = &axiadc_write_event_config,
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
};

struct axiadc_spidev {
	struct device_node *of_nspi;
	struct device *dev_spi;
	struct module *owner;
};

static int axiadc_attach_spi_client(struct device *dev, void *data)
{
	struct axiadc_spidev *axiadc_spidev = data;
	int ret = 0;

	device_lock(dev);
	if ((axiadc_spidev->of_nspi == dev->of_node) && dev->driver) {
		axiadc_spidev->dev_spi = dev;
		axiadc_spidev->owner = dev->driver->owner;
		ret = 1;
	}
	device_unlock(dev);

	return ret;
}

static int axiadc_jesd204_link_supported(struct jesd204_dev *jdev,
		enum jesd204_state_op_reason reason,
		struct jesd204_link *lnk)
{
	struct device *dev = jesd204_dev_to_device(jdev);
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	u32 i, d1, d2, num, multi_device_link;
	bool failed, last;

	if (reason != JESD204_STATE_OP_REASON_INIT)
		return JESD204_STATE_CHANGE_DONE;

	dev_dbg(dev, "%s:%d link_num %u reason %s\n", __func__, __LINE__,
		lnk->link_id, jesd204_state_op_reason_str(reason));

	num = ADI_JESD204_TPL_TO_PROFILE_NUM(axiadc_read(st, ADI_JESD204_REG_TPL_STATUS));

	for (i = 0; i < num; i++) {
		last = (i == (num - 1));
		failed = false;

		axiadc_write(st, ADI_JESD204_REG_TPL_CNTRL, ADI_JESD204_PROFILE_SEL(i));
		d1 = axiadc_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_1);
		d2 = axiadc_read(st, ADI_JESD204_REG_TPL_DESCRIPTOR_2);

		if ((ADI_JESD204_TPL_TO_L(d1) / lnk->num_lanes) ==
			(ADI_JESD204_TPL_TO_M(d1) / lnk->num_converters))
			multi_device_link = ADI_JESD204_TPL_TO_L(d1) / lnk->num_lanes;
		else
			multi_device_link = 1;

		if (ADI_JESD204_TPL_TO_L(d1) != lnk->num_lanes * multi_device_link) {
			if (last)
				dev_warn(dev,
					"profile%u:link_num%u param L mismatch %u!=%u*%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_L(d1), lnk->num_lanes,
					multi_device_link);
			failed = true;
		}

		if (ADI_JESD204_TPL_TO_M(d1) != lnk->num_converters * multi_device_link) {
			if (last)
				dev_warn(dev,
					"profile%u:link_num%u param M mismatch %u!=%u*%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_M(d1),
					lnk->num_converters, multi_device_link);
			failed = true;
		}

		if (lnk->samples_per_conv_frame && ADI_JESD204_TPL_TO_S(d1) !=
			lnk->samples_per_conv_frame) {
			if (last)
				dev_warn(dev,
					"profile%u:link_num%u param S mismatch %u!=%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_S(d1),
					lnk->samples_per_conv_frame);
			failed = true;
		}

		if (ADI_JESD204_TPL_TO_F(d1) != lnk->octets_per_frame) {
			if (last)
				dev_warn(dev,
					"profile%u:link_num%u param F mismatch %u!=%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_F(d1),
					lnk->octets_per_frame);
			failed = true;
		}

		if (ADI_JESD204_TPL_TO_NP(d2) != lnk->bits_per_sample) {
			if (last)
				dev_warn(dev,
					"profile%u:link_num%u param NP mismatch %u!=%u\n",
					i, lnk->link_id, ADI_JESD204_TPL_TO_NP(d2),
					lnk->bits_per_sample);
			failed = true;
		}

		if (!failed)
			return JESD204_STATE_CHANGE_DONE;
	}

	dev_err(dev, "JESD param mismatch between TPL and Link configuration !\n");

	return JESD204_STATE_CHANGE_DONE;
}

static const struct jesd204_dev_data jesd204_axiadc_init = {
	.state_ops = {
		[JESD204_OP_LINK_SUPPORTED] = {
			.per_link = axiadc_jesd204_link_supported,
		},
	},
};

static const struct axiadc_core_info axi_adc_10_0_a_info = {
	.version = ADI_AXI_PCORE_VER(10, 0, 'a'),
};

static const struct axiadc_core_info axi_adc_10_1_b_info = {
	.version = ADI_AXI_PCORE_VER(10, 1, 'b'),
};

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible =	"xlnx,axi-ad9234-1.00.a", .data = &axi_adc_10_0_a_info },
	{ .compatible =	"xlnx,axi-ad9250-1.00.a", .data = &axi_adc_10_0_a_info },
	{ .compatible =	"xlnx,axi-ad9434-1.00.a", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9643-6.00.a", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9361-6.00.a", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9680-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9694-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9625-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad6676-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9371-rx-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9684-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-adrv9009-rx-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9208-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9213-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-ad9081-rx-1.0", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-adc-10.0.a", .data = &axi_adc_10_0_a_info },
	{ .compatible = "adi,axi-adrv9002-rx-1.0", .data = &axi_adc_10_1_b_info},
	{ .compatible = "adi,axi-ad9083-rx-1.0", .data = &axi_adc_10_0_a_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

int axiadc_append_attrs(struct iio_dev *indio_dev,
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

static void axiadc_release_converter(void *conv)
{
	struct axiadc_spidev *axiadc_spidev = conv;

	put_device(axiadc_spidev->dev_spi);
	module_put(axiadc_spidev->owner);
}

/**
 * axiadc_of_probe - probe method for the AIM device.
 * @of_dev:	pointer to OF device structure
 * @match:	pointer to the structure used for matching a device
 *
 * This function probes the AIM device in the device tree.
 * It initializes the driver data structure and the hardware.
 * It returns 0, if the driver is bound to the AIM device, or a negative
 * value if there is an error.
 */
static int axiadc_probe(struct platform_device *pdev)
{
	const struct axiadc_core_info *info;
	const struct of_device_id *id;
	struct iio_dev *indio_dev;
	struct axiadc_state *st;
	struct resource *mem;
	struct axiadc_spidev *axiadc_spidev;
	struct axiadc_converter *conv;
	struct device_link *link;
	unsigned int config, skip = 1;
	int ret;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
		 pdev->dev.of_node->name);

	id = of_match_node(axiadc_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	info = id->data;

	axiadc_spidev = devm_kzalloc(&pdev->dev, sizeof(*axiadc_spidev), GFP_KERNEL);
	if (!axiadc_spidev)
		return -ENOMEM;

	/* Defer driver probe until matching spi
	 * converter driver is registered
	 */
	axiadc_spidev->of_nspi = of_parse_phandle(pdev->dev.of_node,
						  "spibus-connected", 0);
	if (!axiadc_spidev->of_nspi) {
		dev_err(&pdev->dev, "could not find spi node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&spi_bus_type, NULL, axiadc_spidev,
			       axiadc_attach_spi_client);
	of_node_put(axiadc_spidev->of_nspi);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(axiadc_spidev->owner))
		return -ENODEV;

	get_device(axiadc_spidev->dev_spi);

	link = device_link_add(&pdev->dev, axiadc_spidev->dev_spi,
			       DL_FLAG_AUTOREMOVE_SUPPLIER);
	if (!link)
		dev_warn(&pdev->dev, "failed to create device link to %s\n",
			dev_name(axiadc_spidev->dev_spi));

	ret = devm_add_action_or_reset(&pdev->dev, axiadc_release_converter, axiadc_spidev);
	if (ret)
		return ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->jdev = devm_jesd204_dev_register(&pdev->dev, &jesd204_axiadc_init);
	if (IS_ERR(st->jdev))
		return PTR_ERR(st->jdev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs_size = resource_size(mem);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->dev_spi = axiadc_spidev->dev_spi;

	platform_set_drvdata(pdev, indio_dev);

	config = axiadc_read(st, ADI_REG_CONFIG);
	st->ext_sync_avail = !!(config & ADI_EXT_SYNC);
	st->dp_disable = false; /* FIXME: resolve later which reg & bit to read for this */

	conv = to_converter(st->dev_spi);
	if (IS_ERR(conv)) {
		dev_err(&pdev->dev, "Failed to get converter device: %d\n",
				(int)PTR_ERR(conv));
		return PTR_ERR(conv);
	}

	iio_device_set_drvdata(indio_dev, conv);
	conv->indio_dev = indio_dev;
	mutex_init(&conv->lock);

	if (conv->chip_info->num_shadow_slave_channels) {
		u32 regs[2];
		ret = of_property_read_u32_array(pdev->dev.of_node,
				"slavecore-reg", regs, ARRAY_SIZE(regs));
		if (!ret) {
			st->slave_regs = ioremap(regs[0], regs[1]);
			if (st->slave_regs)
				st->have_slave_channels = conv->chip_info->
					num_shadow_slave_channels;

		}
	}

	st->additional_channel = of_property_read_bool(pdev->dev.of_node,
		"adi,axi-additional-channel-available");

	/* Reset all HDL Cores */
	axiadc_write(st, ADI_REG_RSTN, 0);
	mdelay(10);
	axiadc_write(st, ADI_REG_RSTN, ADI_MMCM_RSTN);
	mdelay(10);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

	st->pcore_version = axiadc_read(st, ADI_AXI_REG_VERSION);

	if (ADI_AXI_PCORE_VER_MAJOR(st->pcore_version) >
		ADI_AXI_PCORE_VER_MAJOR(info->version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(info->version),
			ADI_AXI_PCORE_VER_MINOR(info->version),
			ADI_AXI_PCORE_VER_PATCH(info->version),
			ADI_AXI_PCORE_VER_MAJOR(st->pcore_version),
			ADI_AXI_PCORE_VER_MINOR(st->pcore_version),
			ADI_AXI_PCORE_VER_PATCH(st->pcore_version));
		return -ENODEV;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = conv->chip_info->scan_masks;

	axiadc_channel_setup(indio_dev, conv->chip_info->channel,
			     st->dp_disable ? 0 : conv->chip_info->num_channels);

	/* only have labels if really supported */
	axiadc_info.read_label = conv->read_label;
	st->iio_info = axiadc_info;
	st->iio_info.attrs = conv->attrs;
	indio_dev->info = &st->iio_info;

	if (conv->post_setup) {
		ret = conv->post_setup(indio_dev);
		if (ret < 0)
			return ret;
	}

	if (!st->dp_disable && !axiadc_read(st, ADI_AXI_REG_ID) &&
		of_find_property(pdev->dev.of_node, "dmas", NULL)) {
		ret = axiadc_configure_ring_stream(indio_dev, NULL);
		if (ret < 0)
			return ret;
	}

	if (!st->dp_disable && of_property_read_bool(pdev->dev.of_node,
		"adi,axi-decimation-core-available")) {
		st->decimation_factor = 1;
		st->gpio_decimation = devm_gpiod_get_optional(&pdev->dev,
							      "decimation",
							      GPIOD_OUT_LOW);
		if (IS_ERR(st->gpio_decimation))
			dev_err(&pdev->dev, "decimation gpio error\n");
		skip = 0;
	}

	ret = axiadc_append_attrs(indio_dev,
		&axiadc_dec_attribute_group, skip);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to add sysfs attributes (%d)\n", ret);
		return ret;
	}

	ret = devm_iio_device_register(&pdev->dev, indio_dev);
	if (ret)
		return ret;

	if (iio_get_debugfs_dentry(indio_dev))
		debugfs_create_file("pseudorandom_err_check", 0644,
					iio_get_debugfs_dentry(indio_dev),
					indio_dev, &axiadc_debugfs_pncheck_fops);

	if (conv->post_iio_register) {
		ret = conv->post_iio_register(indio_dev);
		if (ret < 0)
			dev_err(&pdev->dev,
				"post_iio_register callback failed (%d)", ret);
	}

	ret = jesd204_fsm_start(st->jdev, JESD204_LINKS_ALL);
	if (ret)
		return ret;

	dev_info(&pdev->dev,
		 "ADI AIM (%d.%.2d.%c) at 0x%08llX mapped to 0x%p probed ADC %s as %s\n",
		 ADI_AXI_PCORE_VER_MAJOR(st->pcore_version),
		 ADI_AXI_PCORE_VER_MINOR(st->pcore_version),
		 ADI_AXI_PCORE_VER_PATCH(st->pcore_version),
		 (unsigned long long)mem->start, st->regs,
		 conv->chip_info->name,
		 axiadc_read(st, ADI_AXI_REG_ID) ? "SLAVE" : "MASTER");

	return 0;
}

static struct platform_driver axiadc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe		= axiadc_probe,
};

module_platform_driver(axiadc_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices ADI-AIM");
MODULE_LICENSE("GPL v2");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
