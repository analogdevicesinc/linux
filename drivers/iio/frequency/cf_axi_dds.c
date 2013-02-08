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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/spi/spi.h>
#include <asm/div64.h>

#include <linux/of_device.h>
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

struct axi_dds_dma_params {
	struct device_node *of_node;
	int chan_id;
};

static bool cf_axi_dds_dma_filter(struct dma_chan *chan, void *param)
{
	struct axi_dds_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

static void cf_axi_dds_sync_frame(struct iio_dev *indio_dev)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	unsigned stat;
	static int retry = 0;

	mdelay(10); /* Wait until clocks are stable */

	dds_write(st, CF_AXI_DDS_FRAME, 0);
	dds_write(st, CF_AXI_DDS_FRAME, CF_AXI_DDS_FRAME_SYNC);

	/* Check FIFO status */
	stat = conv->read(conv->spi, AD9122_REG_FIFO_STATUS_1);
	if (stat & (AD9122_FIFO_STATUS_1_FIFO_WARNING_1 |
		AD9122_FIFO_STATUS_1_FIFO_WARNING_2)) {
		if (retry++ > 3) {
			dev_warn(indio_dev->dev.parent, "FRAME/FIFO Reset Retry cnt\n");
			return;
		}

		cf_axi_dds_sync_frame(indio_dev);
	}

	retry = 0;
}

void cf_axi_dds_stop(struct cf_axi_dds_state *st) {
	dds_write(st, CF_AXI_DDS_CTRL, (st->vers_id > 1) ?
	CF_AXI_DDS_CTRL_DDS_CLK_EN_V2 : CF_AXI_DDS_CTRL_DDS_CLK_EN_V1);
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

struct cf_axi_dds_sed {
	unsigned short i0;
	unsigned short q0;
	unsigned short i1;
	unsigned short q1;
};

static struct cf_axi_dds_sed dac_sed_pattern[5] = {
	{
		.i0 = 0x5555,
		.q0 = 0xAAAA,
		.i1 = 0xAAAA,
		.q1 = 0x5555,
	},
	{
		.i0 = 0,
		.q0 = 0,
		.i1 = 0xFFFF,
		.q1 = 0xFFFF,
	},
	{
		.i0 = 0,
		.q0 = 0,
		.i1 = 0,
		.q1 = 0,
	},
	{
		.i0 = 0xFFFF,
		.q0 = 0xFFFF,
		.i1 = 0xFFFF,
		.q1 = 0xFFFF,
	},
	{
		.i0 = 0x1248,
		.q0 = 0xEDC7,
		.i1 = 0xEDC7,
		.q1 = 0x1248,
	}
};

static int cf_axi_dds_find_dci(unsigned long *err_field, unsigned entries)
{
	int dci, cnt, start, max_start, max_cnt;
	char str[33];
	int ret;

	for(dci = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		dci < entries; dci++) {
		if (test_bit(dci, err_field) == 0) {
			if (start == -1)
				start = dci;
			cnt++;
			str[dci] = 'o';
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
			start = -1;
			cnt = 0;
			str[dci] = '-';
		}
	}
	str[dci] = 0;

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}


	ret = max_start + ((max_cnt - 1) / 2);

	str[ret] = '|';

	printk("%s DCI %d\n",str, ret);

	return ret;
}

static int cf_axi_dds_tune_dci(struct cf_axi_dds_state *st)
{
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	unsigned reg, err_mask, pwr;
	int i = 0, dci;
	unsigned long err_bfield = 0;

	pwr = conv->read(conv->spi, AD9122_REG_POWER_CTRL);
	conv->write(conv->spi, AD9122_REG_POWER_CTRL, pwr |
			AD9122_POWER_CTRL_PD_I_DAC |
			AD9122_POWER_CTRL_PD_Q_DAC);

	for (dci = 0; dci < 4; dci++) {
		conv->write(conv->spi, AD9122_REG_DCI_DELAY, dci);
		for (i = 0; i < ARRAY_SIZE(dac_sed_pattern); i++) {

			conv->write(conv->spi, AD9122_REG_SED_CTRL, 0);

			dds_write(st, CF_AXI_DDS_PAT_DATA1,
				  (dac_sed_pattern[i].i1 << 16) |
				  dac_sed_pattern[i].i0);
			dds_write(st, CF_AXI_DDS_PAT_DATA2,
				  (dac_sed_pattern[i].q1 << 16) |
				  dac_sed_pattern[i].q0);

			dds_write(st, CF_AXI_DDS_CTRL, 0);
			dds_write(st, CF_AXI_DDS_CTRL, CF_AXI_DDS_CTRL_DDS_CLK_EN_V2 |
				 CF_AXI_DDS_CTRL_PATTERN_EN);
			dds_write(st, CF_AXI_DDS_CTRL, CF_AXI_DDS_CTRL_DDS_CLK_EN_V2 |
				 CF_AXI_DDS_CTRL_PATTERN_EN | CF_AXI_DDS_CTRL_DATA_EN);

			conv->write(conv->spi, AD9122_REG_COMPARE_I0_LSBS,
				dac_sed_pattern[i].i0 & 0xFF);
			conv->write(conv->spi, AD9122_REG_COMPARE_I0_MSBS,
				dac_sed_pattern[i].i0 >> 8);

			conv->write(conv->spi, AD9122_REG_COMPARE_Q0_LSBS,
				dac_sed_pattern[i].q0 & 0xFF);
			conv->write(conv->spi, AD9122_REG_COMPARE_Q0_MSBS,
				dac_sed_pattern[i].q0 >> 8);

			conv->write(conv->spi, AD9122_REG_COMPARE_I1_LSBS,
				dac_sed_pattern[i].i1 & 0xFF);
			conv->write(conv->spi, AD9122_REG_COMPARE_I1_MSBS,
				dac_sed_pattern[i].i1 >> 8);

			conv->write(conv->spi, AD9122_REG_COMPARE_Q1_LSBS,
				dac_sed_pattern[i].q1 & 0xFF);
			conv->write(conv->spi, AD9122_REG_COMPARE_Q1_MSBS,
				dac_sed_pattern[i].q1 >> 8);


			conv->write(conv->spi, AD9122_REG_SED_CTRL,
				    AD9122_SED_CTRL_SED_COMPARE_EN);

 			conv->write(conv->spi, AD9122_REG_EVENT_FLAG_2,
 				    AD9122_EVENT_FLAG_2_AED_COMPARE_PASS |
				    AD9122_EVENT_FLAG_2_AED_COMPARE_FAIL |
				    AD9122_EVENT_FLAG_2_SED_COMPARE_FAIL);

			conv->write(conv->spi, AD9122_REG_SED_CTRL,
				    AD9122_SED_CTRL_SED_COMPARE_EN);

			msleep(100);
			reg = conv->read(conv->spi, AD9122_REG_SED_CTRL);
			err_mask = conv->read(conv->spi, AD9122_REG_SED_I_LSBS);
			err_mask |= conv->read(conv->spi, AD9122_REG_SED_I_MSBS);
			err_mask |= conv->read(conv->spi, AD9122_REG_SED_Q_LSBS);
			err_mask |= conv->read(conv->spi, AD9122_REG_SED_Q_MSBS);

			if (err_mask || (reg & AD9122_SED_CTRL_SAMPLE_ERR_DETECTED))
				set_bit(dci, &err_bfield);
		}
	}

	conv->write(conv->spi, AD9122_REG_DCI_DELAY,
		    cf_axi_dds_find_dci(&err_bfield, 4));
	conv->write(conv->spi, AD9122_REG_SED_CTRL, 0);
	conv->write(conv->spi, AD9122_REG_POWER_CTRL, pwr);

	dds_write(st, CF_AXI_DDS_CTRL, 0);

	return 0;
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
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	unsigned long long val64;
	unsigned reg;

	switch (m) {
	case 0:
		if (!chan->output) {
			return -EINVAL;
		}
		reg = dds_read(st, CF_AXI_DDS_CTRL);
		if (st->vers_id > 1) {
			if (reg & CF_AXI_DDS_CTRL_DATA_EN)
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
		reg = dds_read(st, CF_AXI_DDS_SCALE);
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
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->get_data_clk)
			return -ENODEV;

		*val = st->dac_clk = conv->get_data_clk(conv);
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
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	unsigned long long val64;
	unsigned reg, ctrl_reg;
	int i, ret;

	ctrl_reg = dds_read(st, CF_AXI_DDS_CTRL);

	switch (mask) {
	case 0:
		if (!chan->output) {
			return -EINVAL;
		}

		if (st->vers_id > 1) {
			if (val)
				ctrl_reg |= (CF_AXI_DDS_CTRL_DATA_EN |
					    CF_AXI_DDS_CTRL_DDS_CLK_EN_V2);
			else
				ctrl_reg &= ~(CF_AXI_DDS_CTRL_DATA_EN);
		} else {
			if (val)
				ctrl_reg |= 1 << (chan->channel * 2);
			else
				ctrl_reg &= ~(1 << (chan->channel * 2));
		}

		dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
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
		reg = dds_read(st, CF_AXI_DDS_SCALE);

		reg &= ~(0xF << (chan->channel * 4));
		reg |= (i << (chan->channel * 4));
		dds_write(st, CF_AXI_DDS_SCALE, reg);
		dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
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
		dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
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
		dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->set_data_clk)
			return -ENODEV;

		cf_axi_dds_stop(st);
		ret = conv->set_data_clk(conv, val);
		if (ret < 0) {
			dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
			return ret;
		}

		if (val != st->dac_clk) {
			cf_axi_dds_tune_dci(st);
		}

		st->dac_clk = conv->get_data_clk(conv);
		dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
		cf_axi_dds_sync_frame(indio_dev);

		break;
	default:
		return -EINVAL;
	}
//	cf_axi_dds_sync_frame(st);

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
				return ret;
		}
		*readval = ret;
		ret = 0;

	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad9122_dds_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	long readin;
	int ret;

	ret = kstrtol(buf, 10, &readin);
	if (ret)
		return ret;

	mutex_lock(&indio_dev->mlock);

	switch ((u32)this_attr->address) {
	case AD9122_REG_I_DAC_OFFSET_MSB:
	case AD9122_REG_Q_DAC_OFFSET_MSB:
		if (readin < 0 || readin > 0xFFFF) {
			ret = -EINVAL;
			goto out;
		}
		break;
	case AD9122_REG_I_PHA_ADJ_MSB:
	case AD9122_REG_Q_PHA_ADJ_MSB:
		if (readin < -512 || readin > 511) {
			ret = -EINVAL;
			goto out;
		}
		break;
	default:
		if (readin < 0 || readin > 0x3FF) {
			ret = -EINVAL;
			goto out;
		}
		break;
	}

	ret = conv->write(conv->spi, (u32)this_attr->address, readin >> 8);
	if (ret < 0)
		goto out;

	ret = conv->write(conv->spi, (u32)this_attr->address - 1, readin & 0xFF);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9122_dds_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int ret = 0;
	unsigned val;

	mutex_lock(&indio_dev->mlock);
	ret = conv->read(conv->spi, (u32)this_attr->address);
	if (ret < 0)
		goto out;
	val = ret << 8;

	ret = conv->read(conv->spi, (u32)this_attr->address - 1);
	if (ret < 0)
		goto out;
	val |= ret & 0xFF;

	switch ((u32)this_attr->address) {
	case AD9122_REG_I_PHA_ADJ_MSB:
	case AD9122_REG_Q_PHA_ADJ_MSB:
		val = sign_extend32(val, 9);
		break;
	}

	ret = sprintf(buf, "%d\n", val);
out:
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static ssize_t ad9122_dds_interpolation_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	long readin;
	unsigned ctrl_reg;
	int ret;

	ret = kstrtol(buf, 10, &readin);
	if (ret)
		return ret;

	if (!conv->set_interpol)
		return -ENODEV;

	mutex_lock(&indio_dev->mlock);

	ctrl_reg = dds_read(st, CF_AXI_DDS_CTRL);
	cf_axi_dds_stop(st);

	switch ((u32)this_attr->address) {
	case 0:

		ret = conv->set_interpol(conv, readin,
					conv->fcenter_shift, 0);
		break;
	case 1:
		ret = conv->set_interpol(conv, conv->interp_factor,
					readin, 0);
		break;
	default:
		ret = -EINVAL;
	}

	dds_write(st, CF_AXI_DDS_CTRL, ctrl_reg);
	cf_axi_dds_sync_frame(indio_dev);

	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static ssize_t ad9122_dds_interpolation_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	struct cf_axi_dds_converter *conv = to_converter(st->dev_spi);
	int ret = 0;

	mutex_lock(&indio_dev->mlock);
	switch ((u32)this_attr->address) {
	case 0:
		ret = sprintf(buf, "%d\n", conv->interp_factor);
		break;
	case 1:
		ret = sprintf(buf, "%d\n", conv->fcenter_shift);
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static IIO_DEVICE_ATTR(out_voltage0_phase, S_IRUGO | S_IWUSR,
 			ad9122_dds_show,
 			ad9122_dds_store,
 			AD9122_REG_I_PHA_ADJ_MSB);

static IIO_DEVICE_ATTR(out_voltage1_phase, S_IRUGO | S_IWUSR,
 			ad9122_dds_show,
 			ad9122_dds_store,
 			AD9122_REG_Q_PHA_ADJ_MSB);

static IIO_DEVICE_ATTR(out_voltage0_calibbias, S_IRUGO | S_IWUSR,
 			ad9122_dds_show,
 			ad9122_dds_store,
 			AD9122_REG_I_DAC_OFFSET_MSB);

static IIO_DEVICE_ATTR(out_voltage1_calibbias, S_IRUGO | S_IWUSR,
 			ad9122_dds_show,
 			ad9122_dds_store,
 			AD9122_REG_Q_DAC_OFFSET_MSB);

static IIO_DEVICE_ATTR(out_voltage0_calibscale, S_IRUGO | S_IWUSR,
 			ad9122_dds_show,
 			ad9122_dds_store,
 			AD9122_REG_I_DAC_CTRL);

static IIO_DEVICE_ATTR(out_voltage1_calibscale, S_IRUGO | S_IWUSR,
 			ad9122_dds_show,
 			ad9122_dds_store,
 			AD9122_REG_Q_DAC_CTRL);

static IIO_DEVICE_ATTR(out_altvoltage_interpolation, S_IRUGO | S_IWUSR,
			ad9122_dds_interpolation_show,
			ad9122_dds_interpolation_store,
			0);

static IIO_DEVICE_ATTR(out_altvoltage_interpolation_center_shift, S_IRUGO | S_IWUSR,
			ad9122_dds_interpolation_show,
			ad9122_dds_interpolation_store,
			1);

static IIO_CONST_ATTR(out_altvoltage_scale_available,
		"1.000000 0.500000 0.250000 0.125000 ...");

static IIO_CONST_ATTR(out_altvoltage_interpolation_available,
		"1 2 4 8");

static struct attribute *cf_axi_dds_attributes[] = {
	&iio_dev_attr_out_voltage0_phase.dev_attr.attr, /* I */
	&iio_dev_attr_out_voltage0_calibscale.dev_attr.attr,
	&iio_dev_attr_out_voltage0_calibbias.dev_attr.attr,
	&iio_dev_attr_out_voltage1_phase.dev_attr.attr, /* Q */
	&iio_dev_attr_out_voltage1_calibscale.dev_attr.attr,
	&iio_dev_attr_out_voltage1_calibbias.dev_attr.attr,
	&iio_const_attr_out_altvoltage_scale_available.dev_attr.attr,
	&iio_dev_attr_out_altvoltage_interpolation.dev_attr.attr,
	&iio_dev_attr_out_altvoltage_interpolation_center_shift.dev_attr.attr,
	&iio_const_attr_out_altvoltage_interpolation_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group cf_axi_dds_attribute_group = {
	.attrs = cf_axi_dds_attributes,
};

#define CF_AXI_DDS_CHAN(_chan, _address, _extend_name)			\
	{ .type = IIO_ALTVOLTAGE,					\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_RAW_SEPARATE_BIT |			\
		       IIO_CHAN_INFO_SCALE_SEPARATE_BIT |		\
	  	       IIO_CHAN_INFO_PHASE_SEPARATE_BIT |		\
		       IIO_CHAN_INFO_FREQUENCY_SEPARATE_BIT |		\
		       IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT,		\
	  .address = _address,						\
	  .output = 1,							\
	  .extend_name = _extend_name,					\
	  }

#define CF_AXI_DDS_CHAN_BUF(_chan)					\
	{ .type = IIO_ALTVOLTAGE,					\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = 0,						\
	  .output = 1,							\
	  .scan_index = _chan,						\
	  .scan_type = IIO_ST('s', 16, 16, 0),				\
}

static const struct cf_axi_dds_chip_info cf_axi_dds_chip_info_tbl[] = {
	[ID_AD9122] = {
		.name = "AD9122",
		.channel[0] = CF_AXI_DDS_CHAN(0, CF_AXI_DDS_1A_OUTPUT_CTRL, "1A"),
		.channel[1] = CF_AXI_DDS_CHAN(1, CF_AXI_DDS_1B_OUTPUT_CTRL, "1B"),
		.channel[2] = CF_AXI_DDS_CHAN(2, CF_AXI_DDS_2A_OUTPUT_CTRL, "2A"),
		.channel[3] = CF_AXI_DDS_CHAN(3, CF_AXI_DDS_2B_OUTPUT_CTRL, "2B"),
		.buf_channel[0] = CF_AXI_DDS_CHAN_BUF(0),
		.buf_channel[1] = CF_AXI_DDS_CHAN_BUF(1),
	},
	[ID_AD9739A] = {
		.name = "AD9739A",
		.channel[0] = CF_AXI_DDS_CHAN(0, CF_AXI_DDS_1A_OUTPUT_CTRL, "1A"),
		.channel[1] = CF_AXI_DDS_CHAN(1, CF_AXI_DDS_1B_OUTPUT_CTRL, "1B"),
		.channel[2] = CF_AXI_DDS_CHAN(2, CF_AXI_DDS_2A_OUTPUT_CTRL, "2A"),
		.channel[3] = CF_AXI_DDS_CHAN(3, CF_AXI_DDS_2B_OUTPUT_CTRL, "2B"),
		.buf_channel[0] = CF_AXI_DDS_CHAN_BUF(0),
		.buf_channel[1] = CF_AXI_DDS_CHAN_BUF(1),
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
	struct axi_dds_dma_params dma_params;
	struct of_phandle_args dma_spec;
	dma_cap_mask_t mask;
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

	conv = to_converter(st->dev_spi);
	if (IS_ERR(conv)) {
		ret = PTR_ERR(conv);
		goto failed3;
	}

	st->dac_clk = conv->get_data_clk(conv);
	st->chip_info = &cf_axi_dds_chip_info_tbl[conv->id];

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = 4;
	indio_dev->info = &cf_axi_dds_info;

	cf_axi_dds_tune_dci(st);

	dds_write(st, CF_AXI_DDS_INTERPOL_CTRL, 0x2aaa5555); /* Lin. Interp. */

	dds_write(st, CF_AXI_DDS_CTRL, 0x0);
	dds_write(st, CF_AXI_DDS_SCALE, 0x1111); /* divide by 4 */
	dds_write(st, CF_AXI_DDS_1A_OUTPUT_CTRL,
		  cf_axi_dds_calc(90000, 40000000, st->dac_clk));
	dds_write(st, CF_AXI_DDS_1B_OUTPUT_CTRL,
		  cf_axi_dds_calc(90000, 40000000, st->dac_clk));
	dds_write(st, CF_AXI_DDS_2A_OUTPUT_CTRL,
		  cf_axi_dds_calc(0, 40000000, st->dac_clk));
	dds_write(st, CF_AXI_DDS_2B_OUTPUT_CTRL,
		  cf_axi_dds_calc(0, 40000000, st->dac_clk));

	if (st->vers_id > 1)
		dds_write(st, CF_AXI_DDS_CTRL, CF_AXI_DDS_CTRL_DATA_EN |
			  CF_AXI_DDS_CTRL_DDS_CLK_EN_V2); /* clk, dds enable & ddsx select */
	else
		dds_write(st, CF_AXI_DDS_CTRL, 0x1ff); /* clk, dds enable & ddsx select */

	cf_axi_dds_sync_frame(indio_dev);

	ret = of_parse_phandle_with_args(op->dev.of_node, "dma-request",
					 "#dma-cells", 0, &dma_spec);
	if (ret) {
		dev_warn(dev, "Couldn't parse dma-request\n");
		goto skip_writebuf;
	}

	dma_params.of_node = dma_spec.np;
	dma_params.chan_id = dma_spec.args[0];

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

	st->tx_chan = dma_request_channel(mask, cf_axi_dds_dma_filter, &dma_params);
	if (!st->tx_chan) {
		dev_err(dev, "failed to find vdma device\n");
		goto failed3;
	}

	cf_axi_dds_configure_buffer(indio_dev);

	ret = iio_buffer_register(indio_dev,
				  st->chip_info->buf_channel, 2);
	if (ret)
		goto failed3;

skip_writebuf:
	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed3;

	dev_info(dev, "Analog Devices CF_AXI_DDS_DDS %s (0x%X) at 0x%08llX mapped"
		" to 0x%p, probed DDS %s\n",
		(dds_read(st, CF_AXI_DDS_PCORE_IDENT) &
		CF_AXI_DDS_PCORE_IDENT_SLAVE) ? "SLAVE" : "MASTER",
		dds_read(st, CF_AXI_DDS_VERSION_ID),
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

	if (st->tx_chan == NULL) {
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
	.remove		= __devexit_p(cf_axi_dds_of_remove),
};

module_platform_driver(cf_axi_dds_of_driver);

MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices DDS");
MODULE_LICENSE("GPL v2");
