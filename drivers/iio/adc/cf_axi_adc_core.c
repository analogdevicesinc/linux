/*
 * AXI_ADC ADI ADC Interface Module
 *
 * Copyright 2012 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * http://wiki.analog.com/resources/fpga/xilinx/fmc/ad9467
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

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"

const unsigned int decimation_factors_available[] = {1, 8};

struct axiadc_core_info {
	unsigned int version;
};

static int axiadc_chan_to_regoffset(struct iio_chan_spec const *chan)
{
	if (chan->modified)
		return chan->scan_index;

	return chan->channel;
}

int axiadc_set_pnsel(struct axiadc_state *st, int channel, enum adc_pn_sel sel)
{
	unsigned reg;

	if (PCORE_VERSION_MAJOR(st->pcore_version) > 7) {
		reg = axiadc_read(st, ADI_REG_CHAN_CNTRL_3(channel));
		reg &= ~ADI_ADC_PN_SEL(~0);
		reg |= ADI_ADC_PN_SEL(sel);
		axiadc_write(st, ADI_REG_CHAN_CNTRL_3(channel), reg);
	} else {
		reg = axiadc_read(st, ADI_REG_CHAN_CNTRL(channel));

		if (sel == ADC_PN_CUSTOM) {
			reg |= ADI_PN_SEL;
		} else if (sel == ADC_PN9) {
			reg &= ~ADI_PN23_TYPE;
			reg &= ~ADI_PN_SEL;
		} else {
			reg |= ADI_PN23_TYPE;
			reg &= ~ADI_PN_SEL;
		}

		axiadc_write(st, ADI_REG_CHAN_CNTRL(channel), reg);
	}

	return 0;
}
EXPORT_SYMBOL_GPL(axiadc_set_pnsel);

enum adc_pn_sel axiadc_get_pnsel(struct axiadc_state *st,
			       int channel, const char **name)
{
	unsigned val;

	if (PCORE_VERSION_MAJOR(st->pcore_version) > 7) {
		const char *ident[] = {"PN9", "PN23A", "UNDEF", "UNDEF",
				"PN7", "PN15", "PN23", "PN31", "UNDEF", "PN_CUSTOM"};

		val = ADI_TO_ADC_PN_SEL(axiadc_read(st, ADI_REG_CHAN_CNTRL_3(channel)));

		if (name)
			*name = ident[val];

		return val;
	} else {
		val = axiadc_read(st, ADI_REG_CHAN_CNTRL(channel));;

		if (name) {
			if (val & ADI_PN_SEL)
				*name = "PN_CUSTOM";
			else if (val & ADI_PN23_TYPE)
				*name = "PN23";
			else
				*name = "PN9";
		}
		return val & (ADI_PN23_TYPE | ADI_PN_SEL);
	}
}

static void axiadc_toggle_scale_offset_en(struct axiadc_state *st)
{
	return;
}

static ssize_t axiadc_debugfs_pncheck_read(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	char buf[1000];
	const char *pn_name;
	ssize_t len = 0;
	unsigned stat, i;

	for (i = 0; i < conv->chip_info->num_channels; i++) {
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

	mutex_lock(&indio_dev->mlock);

	for (i = 0; i < conv->chip_info->num_channels; i++) {
		if (conv->set_pnsel)
			conv->set_pnsel(indio_dev, i, mode);

		if (mode != ADC_PN_OFF)
			axiadc_set_pnsel(st, i, mode);
	}

	mdelay(1);

	for (i = 0; i < conv->chip_info->num_channels; i++)
		axiadc_write(st, ADI_REG_CHAN_STATUS(i), ~0);

	mutex_unlock(&indio_dev->mlock);

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
	int ret;

	mutex_lock(&indio_dev->mlock);

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
	mutex_unlock(&indio_dev->mlock);

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

			if (st->decimation_factor == 8)
				reg |= BIT(0);
			else
				reg &= ~BIT(0);

			axiadc_write(st, ADI_REG_GP_CONTROL, reg);
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
		if (val == decimation_factors_available[i]) {
			st->decimation_factor = val;
			return axiadc_decimation_set(st, val);
		}
	}

	return -EINVAL;
}

static ssize_t axiadc_sampling_frequency_available(struct device *dev,
						   struct device_attribute *attr,
						   char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned long freq;
	int i, ret;

	if (!st->decimation_factor)
		return -ENODEV;

	mutex_lock(&indio_dev->mlock);
	ret = axiadc_get_parent_sampling_frequency(st, &freq);
	if (ret < 0) {
		mutex_unlock(&indio_dev->mlock);
		return ret;
	}

	for (ret = 0, i = 0; i < ARRAY_SIZE(decimation_factors_available); i++)
		ret += snprintf(buf + ret, PAGE_SIZE - ret, "%ld ",
				freq / decimation_factors_available[i]);

	ret += snprintf(&buf[ret], PAGE_SIZE - ret, "\n");

	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(in_voltage_sampling_frequency_available, S_IRUGO,
		       axiadc_sampling_frequency_available,
		       NULL,
		       0);

static struct attribute *axiadc_attributes[] = {
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
		ret = conv->read_raw(indio_dev, chan, val, val2, m);
		if (ret < 0 || !*val) {
			tmp = ADI_TO_CLK_FREQ(axiadc_read(st, ADI_REG_CLK_FREQ));
			llval = tmp * 100000000ULL /* FIXME */ * ADI_TO_CLK_RATIO(axiadc_read(st, ADI_REG_CLK_RATIO));
			*val = llval >> 16;
		}

		if (chan->extend_name) {
			tmp = axiadc_read(st,
				ADI_REG_CHAN_USR_CNTRL_2(channel));

			llval = ADI_TO_USR_DECIMATION_M(tmp) * conv->adc_clk;
			do_div(llval, ADI_TO_USR_DECIMATION_N(tmp));
			*val = llval;
		}

		if (st->decimation_factor)
			*val /= st->decimation_factor;

		return IIO_VAL_INT;
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

		axiadc_toggle_scale_offset_en(st);

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
		axiadc_toggle_scale_offset_en(st);
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
	indio_dev->masklength = cnt;

	return 0;
}

static const struct iio_info axiadc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &axiadc_read_raw,
	.write_raw = &axiadc_write_raw,
	.read_event_value = &axiadc_read_event_value,
	.write_event_value = &axiadc_write_event_value,
	.read_event_config = &axiadc_read_event_config,
	.write_event_config = &axiadc_write_event_config,
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
};

static int axiadc_attach_spi_client(struct device *dev, void *data)
{
	struct axiadc_spidev *axiadc_spidev = data;
	int ret = 0;

	device_lock(dev);
	if ((axiadc_spidev->of_nspi == dev->of_node) && dev->driver) {
		axiadc_spidev->dev_spi = dev;
		ret = 1;
	}
	device_unlock(dev);

	return ret;
}

static const struct axiadc_core_info ad9467_core_1_00_a_info = {
	.version = PCORE_VERSION(10, 0, 'a'),
};

static const struct axiadc_core_info ad9361_6_00_a_info = {
	.version = PCORE_VERSION(10, 0, 'a'),
};

static const struct axiadc_core_info ad9643_6_00_a_info = {
	.version = PCORE_VERSION(10, 0, 'a'),
};

static const struct axiadc_core_info ad9680_6_00_a_info = {
	.version = PCORE_VERSION(10, 0, 'a'),
};

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] = {
	{ .compatible = "xlnx,cf-ad9467-core-1.00.a", .data = &ad9467_core_1_00_a_info },
	{ .compatible =	"xlnx,axi-ad9234-1.00.a", .data = &ad9680_6_00_a_info },
	{ .compatible =	"xlnx,axi-ad9250-1.00.a", .data = &ad9680_6_00_a_info },
	{ .compatible =	"xlnx,axi-ad9434-1.00.a", .data = &ad9680_6_00_a_info },
	{ .compatible = "adi,axi-ad9643-6.00.a", .data = &ad9643_6_00_a_info },
	{ .compatible = "adi,axi-ad9361-6.00.a", .data = &ad9361_6_00_a_info },
	{ .compatible = "adi,axi-ad9680-1.0", .data = &ad9680_6_00_a_info },
	{ .compatible = "adi,axi-ad9625-1.0", .data = &ad9680_6_00_a_info },
	{ .compatible = "adi,axi-ad6676-1.0", .data = &ad9680_6_00_a_info },
	{ .compatible = "adi,axi-ad9371-rx-1.0", .data = &ad9361_6_00_a_info },
	{ .compatible = "adi,axi-ad9684-1.0", .data = &ad9680_6_00_a_info },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

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
	struct axiadc_spidev axiadc_spidev;
	struct axiadc_converter *conv;
	int ret;

	dev_dbg(&pdev->dev, "Device Tree Probing \'%s\'\n",
		 pdev->dev.of_node->name);

	id = of_match_node(axiadc_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	info = id->data;

	/* Defer driver probe until matching spi
	 * converter driver is registered
	 */
	axiadc_spidev.of_nspi = of_parse_phandle(pdev->dev.of_node,
						 "spibus-connected", 0);
	if (!axiadc_spidev.of_nspi) {
		dev_err(&pdev->dev, "could not find spi node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&spi_bus_type, NULL, &axiadc_spidev,
			       axiadc_attach_spi_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(axiadc_spidev.dev_spi->driver->owner))
		return -ENODEV;

	get_device(axiadc_spidev.dev_spi);

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*st));
	if (indio_dev == NULL) {
		ret = -ENOMEM;
		goto err_put_converter;
	}

	st = iio_priv(indio_dev);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	st->regs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(st->regs))
		return PTR_ERR(st->regs);

	st->dev_spi = axiadc_spidev.dev_spi;

	platform_set_drvdata(pdev, indio_dev);

	st->dp_disable = axiadc_read(st, ADI_REG_ADC_DP_DISABLE);

	conv = to_converter(st->dev_spi);
	if (IS_ERR(conv)) {
		dev_err(&pdev->dev, "Failed to get converter device: %d\n",
				(int)PTR_ERR(conv));
		return PTR_ERR(conv);
	}

	iio_device_set_drvdata(indio_dev, conv);
	conv->indio_dev = indio_dev;

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

	/* Reset all HDL Cores */
	axiadc_write(st, ADI_REG_RSTN, 0);
	mdelay(10);
	axiadc_write(st, ADI_REG_RSTN, ADI_MMCM_RSTN);
	mdelay(10);
	axiadc_write(st, ADI_REG_RSTN, ADI_RSTN | ADI_MMCM_RSTN);

	st->pcore_version = axiadc_read(st, ADI_REG_VERSION);

	if (PCORE_VERSION_MAJOR(st->pcore_version) >
		PCORE_VERSION_MAJOR(info->version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			PCORE_VERSION_MAJOR(info->version),
			PCORE_VERSION_MINOR(info->version),
			PCORE_VERSION_LETTER(info->version),
			PCORE_VERSION_MAJOR(st->pcore_version),
			PCORE_VERSION_MINOR(st->pcore_version),
			PCORE_VERSION_LETTER(st->pcore_version));
		ret = -ENODEV;
		goto err_put_converter;
	}

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = pdev->dev.of_node->name;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->available_scan_masks = conv->chip_info->scan_masks;

	axiadc_channel_setup(indio_dev, conv->chip_info->channel,
			     st->dp_disable ? 0 : conv->chip_info->num_channels);

	st->iio_info = axiadc_info;
	st->iio_info.attrs = conv->attrs;
	indio_dev->info = &st->iio_info;

	ret = conv->post_setup(indio_dev);
	if (ret < 0)
		goto err_put_converter;

	if (!st->dp_disable && !axiadc_read(st, ADI_REG_ID)) {

		ret = axiadc_configure_ring_stream(indio_dev, NULL);
		if (ret < 0)
			goto err_put_converter;
	}

	if (!st->dp_disable && of_property_read_bool(pdev->dev.of_node, "adi,axi-decimation-core-available")) {
		st->decimation_factor = 1;
		WARN_ON(st->iio_info.attrs != NULL);
		st->iio_info.attrs = &axiadc_dec_attribute_group;
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_ring;

	if (indio_dev->buffer && indio_dev->buffer->scan_mask)
		*indio_dev->buffer->scan_mask =
			(1UL << conv->chip_info->num_channels) - 1;

	dev_info(&pdev->dev, "ADI AIM (%d.%.2d.%c) at 0x%08llX mapped to 0x%p,"
		 " probed ADC %s as %s\n",
		PCORE_VERSION_MAJOR(st->pcore_version),
		PCORE_VERSION_MINOR(st->pcore_version),
		PCORE_VERSION_LETTER(st->pcore_version),
		 (unsigned long long)mem->start, st->regs,
		 conv->chip_info->name,
		 axiadc_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER");

	if (iio_get_debugfs_dentry(indio_dev))
		debugfs_create_file("pseudorandom_err_check", 0644,
					iio_get_debugfs_dentry(indio_dev),
					indio_dev, &axiadc_debugfs_pncheck_fops);

	return 0;

err_unconfigure_ring:
	if (!st->dp_disable)
			axiadc_unconfigure_ring_stream(indio_dev);
err_put_converter:
	put_device(axiadc_spidev.dev_spi);
	module_put(axiadc_spidev.dev_spi->driver->owner);

	return ret;
}

/**
 * axiadc_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int axiadc_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	if (!st->dp_disable)
		axiadc_unconfigure_ring_stream(indio_dev);
	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);

	return 0;
}

static struct platform_driver axiadc_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe		= axiadc_probe,
	.remove		= axiadc_remove,
};

module_platform_driver(axiadc_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices ADI-AIM");
MODULE_LICENSE("GPL v2");
