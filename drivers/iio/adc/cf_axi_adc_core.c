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

#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>

#include "cf_axi_adc.h"
#define DCO_DEBUG

static int axiadc_spi_read(struct axiadc_state *st, unsigned reg)
{
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (IS_ERR(conv))
		return PTR_ERR(conv);

	return conv->read(conv->spi, reg);
}

static int axiadc_spi_write(struct axiadc_state *st, unsigned reg, unsigned val)
{
	struct axiadc_converter *conv = to_converter(st->dev_spi);

	if (IS_ERR(conv))
		return PTR_ERR(conv);

	return conv->write(conv->spi, reg, val);
}

static int axiadc_testmode_set(struct iio_dev *indio_dev,
			     unsigned chan_mask, unsigned mode)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	switch (mode) {
	case TESTMODE_PN23_SEQ:
	case TESTMODE_PN9_SEQ:
	case TESTMODE_ALT_CHECKERBOARD:
		axiadc_write(st, AXIADC_PCORE_ADC_CTRL, 0);
		axiadc_spi_write(st, ADC_REG_OUTPUT_MODE,
			      st->adc_def_output_mode &
			      ~OUTPUT_MODE_TWOS_COMPLEMENT);
		break;
	default:
		axiadc_write(st, AXIADC_PCORE_ADC_CTRL,
		  (st->id == CHIPID_AD9643) ? AXIADC_SIGNEXTEND : 0);
		axiadc_spi_write(st, ADC_REG_OUTPUT_MODE, st->adc_def_output_mode);
	};

	axiadc_spi_write(st, ADC_REG_CHAN_INDEX, chan_mask);
	axiadc_spi_write(st, ADC_REG_TEST_IO, mode);
	axiadc_spi_write(st, ADC_REG_CHAN_INDEX, 0x3);
	axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);

	if (chan_mask & BIT(0))
		st->testmode[0] = mode;
	if (chan_mask & BIT(1))
		st->testmode[1] = mode;

	return 0;
}

static void axiadc_toggle_scale_offset_en(struct axiadc_state *st)
{
	unsigned val = axiadc_read(st, AXIADC_PCORE_ADC_CTRL);
	val &= ~AXIADC_SCALE_OFFSET_EN;
	axiadc_write(st, AXIADC_PCORE_ADC_CTRL, val);
	axiadc_write(st, AXIADC_PCORE_ADC_CTRL, val | AXIADC_SCALE_OFFSET_EN);
}

static int axiadc_debugfs_open(struct inode *inode, struct file *file)
{
	if (inode->i_private)
		file->private_data = inode->i_private;

	return 0;
}

static int axiadc_dco_calibrate(struct iio_dev *indio_dev, unsigned chan)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	int dco, cnt, start, max_start, max_cnt;
	unsigned stat, inv_range = 0, tm_mask, err_mask, dco_en = 0;
	unsigned char err_field[66];

	switch (st->id) {
	case CHIPID_AD9250:
		return 0;
	case CHIPID_AD9265:
		dco_en = 0;
		break;
	default:
		dco_en = DCO_DELAY_ENABLE;
	}

restart:
	axiadc_spi_write(st, ADC_REG_OUTPUT_PHASE, OUTPUT_EVEN_ODD_MODE_EN |
			(inv_range ? INVERT_DCO_CLK : 0));

	if (chan == 2) {
		axiadc_testmode_set(indio_dev, 0x2, TESTMODE_PN23_SEQ);
		tm_mask = AXIADC_PN23_1_EN | AXIADC_PN9_0_EN;
		err_mask = AXIADC_PCORE_ADC_STAT_PN_ERR0 |
			AXIADC_PCORE_ADC_STAT_PN_ERR1 |
			AXIADC_PCORE_ADC_STAT_PN_OOS0 |
			AXIADC_PCORE_ADC_STAT_PN_OOS1;
	} else {
		tm_mask = AXIADC_PN9_0_EN;
		err_mask = AXIADC_PCORE_ADC_STAT_PN_ERR | AXIADC_PCORE_ADC_STAT_PN_OOS;
	}

	axiadc_testmode_set(indio_dev, 0x1, TESTMODE_PN9_SEQ);
	axiadc_write(st, AXIADC_PCORE_PN_ERR_CTRL, tm_mask);

	for(dco = 0; dco <= 32; dco++) {
		axiadc_spi_write(st, ADC_REG_OUTPUT_DELAY,
			      dco > 0 ? ((dco - 1) | dco_en) : 0);
		axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		axiadc_write(st, AXIADC_PCORE_ADC_STAT,
			     AXIADC_PCORE_ADC_STAT_MASK);

		mdelay(1);
		stat = axiadc_read(st, AXIADC_PCORE_ADC_STAT);
		err_field[dco + (inv_range * 33)] = !!(stat & err_mask);
	}

	for(dco = 0, cnt = 0, max_cnt = 0, start = -1, max_start = 0;
		dco <= (32 + (inv_range * 33)); dco++) {
		if (err_field[dco] == 0) {
			if (start == -1)
				start = dco;
			cnt++;
		} else {
			if (cnt > max_cnt) {
				max_cnt = cnt;
				max_start = start;
			}
			start = -1;
			cnt = 0;
		}
	}

	if (cnt > max_cnt) {
		max_cnt = cnt;
		max_start = start;
	}

	if ((inv_range == 0) && ((max_cnt < 3) || (err_field[32] == 0))) {
		inv_range = 1;
		goto restart;
	}

	dco = max_start + (max_cnt / 2);

#ifdef DCO_DEBUG
	for(cnt = 0; cnt <= (32  + (inv_range * 33)); cnt++)
		if (cnt == dco)
			printk("|");
		else
			printk("%c", err_field[cnt] ? '-' : 'o');
#endif
	if (dco > 32) {
		dco -= 33;
		axiadc_spi_write(st, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN | INVERT_DCO_CLK);
		cnt = 1;
	} else {
		axiadc_spi_write(st, ADC_REG_OUTPUT_PHASE,
				 OUTPUT_EVEN_ODD_MODE_EN);
		cnt = 0;
	}

#ifdef DCO_DEBUG
	printk(" %s DCO 0x%X CLK %lu Hz\n", cnt ? "INVERT" : "",
	       dco > 0 ? ((dco - 1) | dco_en) : 0, st->adc_clk);
#endif

	axiadc_testmode_set(indio_dev, 0x3, TESTMODE_OFF);
	axiadc_spi_write(st, ADC_REG_OUTPUT_DELAY,
		      dco > 0 ? ((dco - 1) | dco_en) : 0);
	axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);

	return 0;
}

static ssize_t axiadc_debugfs_pncheck_read(struct file *file, char __user *userbuf,
			      size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct axiadc_state *st = iio_priv(indio_dev);
	char buf[80];
	ssize_t len;
	unsigned stat;

	stat = axiadc_read(st, AXIADC_PCORE_ADC_STAT);

	switch (st->id) {
	case CHIPID_AD9467:
		len = sprintf(buf, "%s %s\n", (stat & AXIADC_PCORE_ADC_STAT_PN_OOS) ?
			"Out of Sync :" : "In Sync :",
			(stat & AXIADC_PCORE_ADC_STAT_PN_ERR) ?
			"PN Error" : "No Error");
		break;
	case CHIPID_AD9643:
		len = sprintf(buf, "CH0 %s %s\nCH1 %s %s\n", (stat & AXIADC_PCORE_ADC_STAT_PN_OOS0) ?
			"Out of Sync :" : "In Sync :",
			(stat & AXIADC_PCORE_ADC_STAT_PN_ERR0) ?
			"PN Error" : "No Error",
			(stat & AXIADC_PCORE_ADC_STAT_PN_OOS1) ?
			"Out of Sync :" : "In Sync :",
			(stat & AXIADC_PCORE_ADC_STAT_PN_ERR1) ?
			"PN Error" : "No Error");
		break;
	default:
		len = 0;
	}

	axiadc_write(st, AXIADC_PCORE_ADC_STAT, AXIADC_PCORE_ADC_STAT_MASK);

	return simple_read_from_buffer(userbuf, count, ppos, buf, len);
}

static ssize_t axiadc_debugfs_pncheck_write(struct file *file,
		     const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct iio_dev *indio_dev = file->private_data;
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned mode = TESTMODE_OFF;
	char buf[80], *p = buf;

	count = min_t(size_t, count, (sizeof(buf)-1));
	if (copy_from_user(p, userbuf, count))
		return -EFAULT;

	p[count] = 0;

	if (sysfs_streq(p, "PN9"))
		mode = TESTMODE_PN9_SEQ;
	else if (sysfs_streq(p, "PN23"))
		mode = TESTMODE_PN23_SEQ;
	else if (sysfs_streq(p, "CALIB"))
		axiadc_dco_calibrate(indio_dev, st->chip_info->num_channels);
	else
		mode = TESTMODE_OFF;

	mutex_lock(&indio_dev->mlock);
	axiadc_testmode_set(indio_dev, 0x3, mode);

	axiadc_write(st, AXIADC_PCORE_PN_ERR_CTRL, (mode == TESTMODE_PN23_SEQ) ?
		  AXIADC_PN23_EN : AXIADC_PN9_EN);

	mdelay(1); /* FIXME */

	axiadc_write(st, AXIADC_PCORE_ADC_STAT, AXIADC_PCORE_ADC_STAT_MASK);
	mutex_unlock(&indio_dev->mlock);

	return count;
}

static const struct file_operations axiadc_debugfs_pncheck_fops = {
	.open = axiadc_debugfs_open,
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
	if (readval == NULL) {
		if (reg & DEBUGFS_DRA_PCORE_REG_MAGIC) {
			axiadc_write(st, reg & 0xFFFF, writeval);
			ret = 0;
		} else {
			ret = axiadc_spi_write(st, reg, writeval);
			axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		}
	} else {
		if (reg & DEBUGFS_DRA_PCORE_REG_MAGIC) {
			*readval = axiadc_read(st, reg & 0xFFFF);
		} else {
			ret = axiadc_spi_read(st, reg);
			if (ret < 0)
				return ret;
			*readval = ret;
		}
		ret = 0;
	}
	mutex_unlock(&indio_dev->mlock);

	return ret;
}

static int axiadc_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long m)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	struct axiadc_converter *conv = to_converter(st->dev_spi);
	int i;
	unsigned vref_val, tmp, mask;
	unsigned long long llval;

	switch (m) {
	case IIO_CHAN_INFO_SCALE:
		vref_val = axiadc_spi_read(st, ADC_REG_VREF);

		switch (st->id) {
		case CHIPID_AD9467:
			mask = AD9467_REG_VREF_MASK;
			break;
		case CHIPID_AD9643:
			mask = AD9643_REG_VREF_MASK;
			break;
		case CHIPID_AD9250:
			mask = AD9250_REG_VREF_MASK;
			break;
		case CHIPID_AD9265:
			mask = AD9265_REG_VREF_MASK;
			break;
		default:
			mask = 0xFFFF;
		}

		vref_val &= mask;

		for (i = 0; i < st->chip_info->num_scales; i++)
			if (vref_val == st->chip_info->scale_table[i][1])
				break;

		*val =  0;
		*val2 = st->chip_info->scale_table[i][0];

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_CALIBSCALE:
		tmp = axiadc_read(st, (chan->channel == 0) ?
			AXIADC_PCORE_CA_OFFS_SCALE :
			AXIADC_PCORE_CB_OFFS_SCALE);

		tmp = AXIADC_SCALE(tmp);
		if (tmp & 0x8000)
			*val = 1;
		else
			*val = 0;

		tmp &= ~0x8000;

		llval = tmp * 1000000ULL;
		do_div(llval, 0x8000);
		*val2 = llval;

		return IIO_VAL_INT_PLUS_MICRO;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axiadc_read(st, (chan->channel == 0) ? AXIADC_PCORE_CA_OFFS_SCALE : AXIADC_PCORE_CB_OFFS_SCALE);
		tmp >>= 16;
		*val = sign_extend32(tmp, 14);

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		*val = st->adc_clk = clk_get_rate(conv->clk);

		if (chan->extend_name) {
			unsigned chan_sel, cached_chan_sel;
			cached_chan_sel = chan_sel = axiadc_read(st,
					AXIADC_PCORE_DMA_CHAN_SEL);
			chan_sel &= ~AXIADC_PCORE_DMA_CHAN_USRL_SEL(0xF);
			chan_sel |= AXIADC_PCORE_DMA_CHAN_USRL_SEL(
				chan->channel - st->chip_info->num_channels);
			axiadc_write(st, AXIADC_PCORE_DMA_CHAN_SEL, chan_sel);
			tmp = axiadc_read(st, AXIADC_PCORE_USRL_DECIM);
			axiadc_write(st, AXIADC_PCORE_DMA_CHAN_SEL,
				     cached_chan_sel);
			llval = AXIADC_PCORE_USRL_DECIM_NUM(tmp) *
				(unsigned long long)st->adc_clk;
			do_div(llval, AXIADC_PCORE_USRL_DECIM_DEN(tmp));
			*val = llval;
		}
		return IIO_VAL_INT;
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
	unsigned fract, tmp;
	unsigned long long llval;
	long r_clk;
	int i, ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (val != 0)
			return -EINVAL;

		for (i = 0; i < st->chip_info->num_scales; i++)
			if (val2 == st->chip_info->scale_table[i][0]) {
				axiadc_spi_write(st, ADC_REG_VREF,
					st->chip_info->scale_table[i][1]);
				axiadc_spi_write(st, ADC_REG_TRANSFER,
					      TRANSFER_SYNC);
				return 0;
			}

		return -EINVAL;
	case IIO_CHAN_INFO_CALIBSCALE:
		if (val == 0)
			fract = 0;
		else if (val == 1)
			fract = 0x8000;
		else
			return -EINVAL;

		llval = (unsigned long long)val2 * 0x8000UL;
		do_div(llval, 1000000);
		fract |= llval;
		tmp = axiadc_read(st, (chan->channel == 0) ? AXIADC_PCORE_CA_OFFS_SCALE : AXIADC_PCORE_CB_OFFS_SCALE);
		tmp &= ~AXIADC_SCALE(~0);
		tmp |= AXIADC_SCALE(fract);
		axiadc_write(st, (chan->channel == 0) ?
			AXIADC_PCORE_CA_OFFS_SCALE :
			AXIADC_PCORE_CB_OFFS_SCALE, tmp);
		axiadc_toggle_scale_offset_en(st);

		return 0;

	case IIO_CHAN_INFO_CALIBBIAS:
		tmp = axiadc_read(st, (chan->channel == 0) ? AXIADC_PCORE_CA_OFFS_SCALE : AXIADC_PCORE_CB_OFFS_SCALE);
		tmp &= ~AXIADC_OFFSET(~0);
		tmp |= AXIADC_OFFSET((short)val);

		axiadc_write(st, (chan->channel == 0) ?
			AXIADC_PCORE_CA_OFFS_SCALE :
			AXIADC_PCORE_CB_OFFS_SCALE, tmp);
		axiadc_toggle_scale_offset_en(st);
		return 0;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!conv->clk)
			return -ENODEV;

		if (chan->extend_name)
			return -ENODEV;

		r_clk = clk_round_rate(conv->clk, val);
		if (r_clk < 0 || r_clk > st->chip_info->max_rate) {
			dev_warn(&conv->spi->dev,
				"Error setting ADC sample rate %ld", r_clk);
			return -EINVAL;
		}

		ret = clk_set_rate(conv->clk, r_clk);
		if (ret < 0)
			return ret;

		if (st->adc_clk != r_clk) {
			st->adc_clk = r_clk;
			ret = axiadc_dco_calibrate(indio_dev,
						   st->chip_info->num_channels);
		}

		return 0;
	default:
		return -EINVAL;
	}
}

static ssize_t axiadc_show_scale_available(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct axiadc_state *st = iio_priv(indio_dev);
	int i, len = 0;

	for (i = 0; i < st->chip_info->num_scales; i++)
		len += sprintf(buf + len, "0.%06u ",
			       st->chip_info->scale_table[i][0]);

	len += sprintf(buf + len, "\n");

	return len;
}

static IIO_DEVICE_ATTR(in_voltage_scale_available, S_IRUGO,
		       axiadc_show_scale_available, NULL, 0);

static struct attribute *axiadc_attributes[] = {
	&iio_dev_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group axiadc_attribute_group = {
	.attrs = axiadc_attributes,
};

static const int ad9265_scale_table[][2] = {
	{19073, 0x00}, {22888, 0x40}, {26703, 0x80}, {30518, 0xC0},
};

static const int ad9467_scale_table[][2] = {
	{30517, 0}, {32043, 6}, {33569, 7},
	{35095, 8}, {36621, 9}, {38146, 10},
};

static const int ad9643_scale_table[][2] = {
	{31738, 0xF}, {31403, 0xE}, {31067, 0xD}, {30731, 0xC}, {30396, 0xB},
	{30060, 0xA}, {29724, 0x9}, {29388, 0x8}, {29053, 0x7}, {28717, 0x6},
	{28381, 0x5}, {28046, 0x4}, {27710, 0x3}, {27374, 0x2}, {27039, 0x1},
	{26703, 0x0}, {26367, 0x1F}, {26031, 0x1E}, {25696, 0x1D},
	{25360, 0x1C}, {25024, 0x1B}, {24689, 0x1A}, {24353, 0x19},
	{24017, 0x18}, {23682, 0x17}, {23346, 0x16}, {23010, 0x15},
	{22675, 0x14}, {22339, 0x13}, {22003, 0x12}, {21667, 0x11},
	{21332, 0x10},
};

static int axiadc_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct axiadc_state *st = iio_priv(indio_dev);
	unsigned mask;

	if (indio_dev->num_channels == 1)
		return 0;

	switch (st->id) {
	case CHIPID_AD9250:
		if (*scan_mask != (BIT(0) | BIT(1)))
			return -EINVAL;
		break;
	default:
		mask = *scan_mask;

		if (st->have_user_logic)
			mask = (*scan_mask & (BIT(0) | BIT(1))) |
			AXIADC_PCORE_DMA_CHAN_USRL_SEL(ffs(*scan_mask >> 2));

		axiadc_write(st, AXIADC_PCORE_DMA_CHAN_SEL, mask);
	};

	return 0;
}

static const char testmodes[][16] = {
	[TESTMODE_OFF]			= "off",
	[TESTMODE_MIDSCALE_SHORT]	= "midscale_short",
	[TESTMODE_POS_FULLSCALE]	= "pos_fullscale",
	[TESTMODE_NEG_FULLSCALE]	= "neg_fullscale",
	[TESTMODE_ALT_CHECKERBOARD]	= "checkerboard",
	[TESTMODE_PN23_SEQ]		= "pn_long",
	[TESTMODE_PN9_SEQ]		= "pn_short",
	[TESTMODE_ONE_ZERO_TOGGLE]	= "one_zero_toggle",
};

static ssize_t axiadc_testmode_mode_available(struct iio_dev *indio_dev,
				   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	size_t len = 0;
	int i;

	for (i = 0; i < ARRAY_SIZE(testmodes); ++i) {
		len += sprintf(buf + len, "%s ", testmodes[i]);
	}
	len += sprintf(buf + len, "\n");
	return len;
}

static ssize_t axiadc_testmode_read(struct iio_dev *indio_dev,
				 uintptr_t private,
				 const struct iio_chan_spec *chan, char *buf)
{
	struct axiadc_state *st = iio_priv(indio_dev);

	return sprintf(buf, "%s\n",
		testmodes[st->testmode[chan->channel]]);
}

static ssize_t axiadc_testmode_write(struct iio_dev *indio_dev,
				  uintptr_t private,
				  const struct iio_chan_spec *chan,
				  const char *buf, size_t len)
{
	unsigned int mode, i;
	int ret;

	mode = 0;

	for (i = 0; i < ARRAY_SIZE(testmodes); ++i) {
		if (sysfs_streq(buf, testmodes[i])) {
			mode = i;
			break;
		}
	}

	mutex_lock(&indio_dev->mlock);
	ret = axiadc_testmode_set(indio_dev, 1 << chan->channel, mode);
	mutex_unlock(&indio_dev->mlock);

	return ret ? ret : len;
}

static struct iio_chan_spec_ext_info axiadc_ext_info[] = {
	{
		.name = "test_mode",
		.read = axiadc_testmode_read,
		.write = axiadc_testmode_write,
	},
	{
		.name = "test_mode_available",
		.read = axiadc_testmode_mode_available,
		.shared = true,
	},
	{ },
};

#define AIM_CHAN(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT | 		\
			IIO_CHAN_INFO_CALIBSCALE_SEPARATE_BIT |		\
			IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |		\
			IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT,		\
			.ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

#define AIM_CHAN_UL(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_SAMP_FREQ_SEPARATE_BIT,		\
	  .extend_name = "user_logic",					\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

#define AIM_CHAN_NOCALIB(_chan, _si, _bits, _sign)			\
	{ .type = IIO_VOLTAGE,						\
	  .indexed = 1,							\
	  .channel = _chan,						\
	  .info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT |	 		\
			IIO_CHAN_INFO_SAMP_FREQ_SHARED_BIT,		\
	  .ext_info = axiadc_ext_info,			\
	  .scan_index = _si,						\
	  .scan_type =  IIO_ST(_sign, _bits, 16, 0)}

static const struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	[ID_AD9467] = {
		.name = "AD9467",
		.max_rate = 250000000,
		.scale_table = ad9467_scale_table,
		.num_scales = ARRAY_SIZE(ad9467_scale_table),
		.num_channels = 1,
		.channel[0] = AIM_CHAN(0, 0, 16, 's'),
		.channel[1] = AIM_CHAN_UL(1, 1, 16, 's'),
		.channel[2] = AIM_CHAN_UL(2, 2, 16, 's'),
		.channel[3] = AIM_CHAN_UL(3, 3, 16, 's'),
		.channel[4] = AIM_CHAN_UL(4, 4, 16, 's'),
		.channel[5] = AIM_CHAN_UL(5, 5, 16, 's'),
		.channel[6] = AIM_CHAN_UL(6, 6, 16, 's'),
		.channel[7] = AIM_CHAN_UL(7, 7, 16, 's'),
		.channel[8] = AIM_CHAN_UL(8, 8, 16, 's'),
		.channel[9] = AIM_CHAN_UL(9, 9, 16, 's'),
		.channel[10] = AIM_CHAN_UL(10, 10, 16, 's'),
		.channel[11] = AIM_CHAN_UL(11, 11, 16, 's'),
		.channel[12] = AIM_CHAN_UL(12, 12, 16, 's'),
		.channel[13] = AIM_CHAN_UL(13, 13, 16, 's'),
		.channel[14] = AIM_CHAN_UL(14, 14, 16, 's'),
		.channel[15] = AIM_CHAN_UL(15, 15, 16, 's'),
	},
	[ID_AD9643] = {
		.name = "AD9643",
		.max_rate = 250000000,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.num_channels = 2,
		.channel[0] = AIM_CHAN(0, 0, 14, 'u'),
		.channel[1] = AIM_CHAN(1, 1, 14, 'u'),
		.channel[2] = AIM_CHAN_UL(2, 2, 14, 'u'),
		.channel[3] = AIM_CHAN_UL(3, 3, 14, 'u'),
		.channel[4] = AIM_CHAN_UL(4, 4, 14, 'u'),
		.channel[5] = AIM_CHAN_UL(5, 5, 14, 'u'),
		.channel[6] = AIM_CHAN_UL(6, 6, 14, 'u'),
		.channel[7] = AIM_CHAN_UL(7, 7, 14, 'u'),
		.channel[8] = AIM_CHAN_UL(8, 8, 14, 'u'),
		.channel[9] = AIM_CHAN_UL(9, 9, 14, 'u'),
		.channel[10] = AIM_CHAN_UL(10, 10, 14, 'u'),
		.channel[11] = AIM_CHAN_UL(11, 11, 14, 'u'),
		.channel[12] = AIM_CHAN_UL(12, 12, 14, 'u'),
		.channel[13] = AIM_CHAN_UL(13, 13, 14, 'u'),
		.channel[14] = AIM_CHAN_UL(14, 14, 14, 'u'),
		.channel[15] = AIM_CHAN_UL(15, 15, 14, 'u'),
		.channel[16] = AIM_CHAN_UL(16, 16, 14, 'u'),
	},
	[ID_AD9250] = {
		.name = "AD9250",
		.max_rate = 250000000,
		.scale_table = ad9643_scale_table,
		.num_scales = ARRAY_SIZE(ad9643_scale_table),
		.num_channels = 2,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 14, 'u'),
		.channel[1] = AIM_CHAN_NOCALIB(1, 1, 14, 'u'),
		.channel[2] = AIM_CHAN_UL(2, 2, 14, 'u'),
		.channel[3] = AIM_CHAN_UL(3, 3, 14, 'u'),
		.channel[4] = AIM_CHAN_UL(4, 4, 14, 'u'),
		.channel[5] = AIM_CHAN_UL(5, 5, 14, 'u'),
		.channel[6] = AIM_CHAN_UL(6, 6, 14, 'u'),
		.channel[7] = AIM_CHAN_UL(7, 7, 14, 'u'),
		.channel[8] = AIM_CHAN_UL(8, 8, 14, 'u'),
		.channel[9] = AIM_CHAN_UL(9, 9, 14, 'u'),
		.channel[10] = AIM_CHAN_UL(10, 10, 14, 'u'),
		.channel[11] = AIM_CHAN_UL(11, 11, 14, 'u'),
		.channel[12] = AIM_CHAN_UL(12, 12, 14, 'u'),
		.channel[13] = AIM_CHAN_UL(13, 13, 14, 'u'),
		.channel[14] = AIM_CHAN_UL(14, 14, 14, 'u'),
		.channel[15] = AIM_CHAN_UL(15, 15, 14, 'u'),
		.channel[16] = AIM_CHAN_UL(16, 16, 14, 'u'),
	},
	[ID_AD9265] = {
		.name = "AD9265",
		.max_rate = 125000000,
		.scale_table = ad9265_scale_table,
		.num_scales = ARRAY_SIZE(ad9265_scale_table),
		.num_channels = 1,
		.channel[0] = AIM_CHAN_NOCALIB(0, 0, 16, 's'),
		.channel[1] = AIM_CHAN_UL(1, 1, 16, 's'),
		.channel[2] = AIM_CHAN_UL(2, 2, 16, 's'),
		.channel[3] = AIM_CHAN_UL(3, 3, 16, 's'),
		.channel[4] = AIM_CHAN_UL(4, 4, 16, 's'),
		.channel[5] = AIM_CHAN_UL(5, 5, 16, 's'),
		.channel[6] = AIM_CHAN_UL(6, 6, 16, 's'),
		.channel[7] = AIM_CHAN_UL(7, 7, 16, 's'),
		.channel[8] = AIM_CHAN_UL(8, 8, 16, 's'),
		.channel[9] = AIM_CHAN_UL(9, 9, 16, 's'),
		.channel[10] = AIM_CHAN_UL(10, 10, 16, 's'),
		.channel[11] = AIM_CHAN_UL(11, 11, 16, 's'),
		.channel[12] = AIM_CHAN_UL(12, 12, 16, 's'),
		.channel[13] = AIM_CHAN_UL(13, 13, 16, 's'),
		.channel[14] = AIM_CHAN_UL(14, 14, 16, 's'),
		.channel[15] = AIM_CHAN_UL(15, 15, 16, 's'),
	},
};

static const struct iio_info axiadc_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &axiadc_read_raw,
	.write_raw = &axiadc_write_raw,
	.attrs = &axiadc_attribute_group,
	.debugfs_reg_access = &axiadc_reg_access,
	.update_scan_mode = &axiadc_update_scan_mode,
};

struct axiadc_dma_params {
	struct device_node *of_node;
	int chan_id;
};

static bool axiadc_dma_filter(struct dma_chan *chan, void *param)
{
	struct axiadc_dma_params *p = param;

	return chan->device->dev->of_node == p->of_node &&
		chan->chan_id == p->chan_id;
}

static int axiadc_attach_spi_client(struct device *dev, void *data)
{
	struct axiadc_spidev *axiadc_spidev = data;

	if ((axiadc_spidev->of_nspi == dev->of_node) && dev->driver) {
		axiadc_spidev->dev_spi = dev;
		return 1;
	}

	return 0;
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
static int __devinit axiadc_of_probe(struct platform_device *op)
{
	struct iio_dev *indio_dev;
	struct device *dev = &op->dev;
	struct axiadc_state *st;
	struct resource r_mem; /* IO mem resources */
	struct axiadc_dma_params dma_params;
	struct of_phandle_args dma_spec;
	struct axiadc_spidev axiadc_spidev;
	dma_cap_mask_t mask;
	resource_size_t remap_size, phys_addr;
	unsigned chan_inc;
	int ret;

	dev_info(dev, "Device Tree Probing \'%s\'\n",
		 op->dev.of_node->name);

	/* Defer driver probe until matching spi
	 * converter driver is registered
	 */
	axiadc_spidev.of_nspi = of_parse_phandle(op->dev.of_node,
						 "spibus-connected", 0);
	if (!axiadc_spidev.of_nspi) {
		dev_err(&op->dev, "could not find spi node\n");
		return -ENODEV;
	}

	ret = bus_for_each_dev(&spi_bus_type, NULL, &axiadc_spidev,
			       axiadc_attach_spi_client);
	if (ret == 0)
		return -EPROBE_DEFER;

	if (!try_module_get(axiadc_spidev.dev_spi->driver->owner))
		return -ENODEV;

	get_device(axiadc_spidev.dev_spi);

	/* Get iospace for the device */
	ret = of_address_to_resource(op->dev.of_node, 0, &r_mem);
	if (ret) {
		dev_err(dev, "invalid address\n");
		return ret;
	}

	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->dev_spi = axiadc_spidev.dev_spi;

	dev_set_drvdata(dev, indio_dev);
	mutex_init(&st->lock);

	phys_addr = r_mem.start;
	remap_size = resource_size(&r_mem);
	if (!request_mem_region(phys_addr, remap_size, KBUILD_MODNAME)) {
		dev_err(dev, "Couldn't lock memory region at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EBUSY;
		goto failed1;
	}

	st->regs = ioremap(phys_addr, remap_size);
	if (st->regs == NULL) {
		dev_err(dev, "Couldn't ioremap memory at 0x%08llX\n",
			(unsigned long long)phys_addr);
		ret = -EFAULT;
		goto failed2;
	}
	/* Get dma channel for the device */
	ret = of_parse_phandle_with_args(op->dev.of_node, "dma-request",
					 "#dma-cells", 0, &dma_spec);
	if (ret) {
		dev_err(dev, "Couldn't parse dma-request\n");
		goto failed2;
	}

	dma_params.of_node = dma_spec.np;
	dma_params.chan_id = dma_spec.args[0];

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE | DMA_PRIVATE, mask);

	st->rx_chan = dma_request_channel(mask, axiadc_dma_filter, &dma_params);
	if (!st->rx_chan) {
		ret = -EPROBE_DEFER;
		dev_err(dev, "failed to find rx dma device\n");
		goto failed2;
	}

	if (to_converter(st->dev_spi)->clk)
		st->adc_clk = clk_get_rate(to_converter(st->dev_spi)->clk);

	/* Probe device */
	st->id = axiadc_spi_read(st, ADC_REG_CHIP_ID);

	switch (st->id) {
	case CHIPID_AD9467:
		st->chip_info = &axiadc_chip_info_tbl[ID_AD9467];
		st->adc_def_output_mode = AD9467_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		axiadc_write(st, AXIADC_PCORE_ADC_CTRL, AXIADC_SCALE_OFFSET_EN);
		axiadc_write(st, AXIADC_PCORE_CA_OFFS_SCALE,
			     AXIADC_OFFSET(0) | AXIADC_SCALE(0x8000));
		axiadc_spi_write(st, ADC_REG_OUTPUT_MODE, st->adc_def_output_mode);
		axiadc_spi_write(st, ADC_REG_TEST_IO, TESTMODE_OFF);
		axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		axiadc_dco_calibrate(indio_dev, st->chip_info->num_channels);
		break;
	case CHIPID_AD9643:
		st->chip_info = &axiadc_chip_info_tbl[ID_AD9643];
		st->adc_def_output_mode = AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		axiadc_spi_write(st, ADC_REG_OUTPUT_PHASE, OUTPUT_EVEN_ODD_MODE_EN);
		axiadc_write(st, AXIADC_PCORE_ADC_CTRL, AXIADC_SIGNEXTEND | AXIADC_SCALE_OFFSET_EN);
		axiadc_write(st, AXIADC_PCORE_CA_OFFS_SCALE,
			     AXIADC_OFFSET(0) | AXIADC_SCALE(0x8000));
		axiadc_write(st, AXIADC_PCORE_CB_OFFS_SCALE,
			     AXIADC_OFFSET(0) | AXIADC_SCALE(0x8000));
		axiadc_spi_write(st, ADC_REG_OUTPUT_MODE, st->adc_def_output_mode);
		axiadc_spi_write(st, ADC_REG_TEST_IO, TESTMODE_OFF);
		axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		axiadc_dco_calibrate(indio_dev, st->chip_info->num_channels);

		break;
	case CHIPID_AD9250:
		st->chip_info = &axiadc_chip_info_tbl[ID_AD9250];
		st->adc_def_output_mode = AD9643_DEF_OUTPUT_MODE | OUTPUT_MODE_OFFSET_BINARY;
		axiadc_spi_write(st, ADC_REG_OUTPUT_MODE, st->adc_def_output_mode);
		axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		break;
	case CHIPID_AD9265:
		st->chip_info = &axiadc_chip_info_tbl[ID_AD9265];
		st->adc_def_output_mode = AD9265_DEF_OUTPUT_MODE | OUTPUT_MODE_TWOS_COMPLEMENT;
		axiadc_spi_write(st, ADC_REG_OUTPUT_MODE, st->adc_def_output_mode);
		axiadc_spi_write(st, ADC_REG_TEST_IO, TESTMODE_OFF);
		axiadc_spi_write(st, ADC_REG_TRANSFER, TRANSFER_SYNC);
		axiadc_dco_calibrate(indio_dev, st->chip_info->num_channels);
		break;
	default:
		dev_err(dev, "Unrecognized CHIP_ID 0x%X\n", st->id);
		ret = -ENODEV;
		goto failed3;
	}

	st->pcore_version = axiadc_read(st, AXIADC_PCORE_VERSION);

	if (st->pcore_version > AXIADC_PCORE_VERSION_IS(1, 0, 'a'))
		st->max_count = AXIADC_MAX_DMA_SIZE;
	else
		st->max_count = AXIADC_MAX_PCORE_TSIZE;


	st->have_user_logic = axiadc_read(st, AXIADC_PCORE_USRL_MAXCH);

	indio_dev->dev.parent = dev;
	indio_dev->name = op->dev.of_node->name;
	indio_dev->channels = st->chip_info->channel;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->num_channels = st->chip_info->num_channels + st->have_user_logic;
	indio_dev->masklength = indio_dev->num_channels;

	indio_dev->info = &axiadc_info;

	init_completion(&st->dma_complete);

	axiadc_configure_ring(indio_dev);

	ret = iio_buffer_register(indio_dev,
				  indio_dev->channels,
				  indio_dev->num_channels);
	if (ret)
		goto failed4;

	*indio_dev->buffer->scan_mask = (1UL << st->chip_info->num_channels) - 1;

	ret = iio_device_register(indio_dev);
	if (ret)
		goto failed4;

	dev_info(dev, "ADI AIM (0x%X) at 0x%08llX mapped to 0x%p,"
		 " DMA-%d probed ADC %s as %s\n",
		 st->pcore_version,
		 (unsigned long long)phys_addr, st->regs,
		 st->rx_chan->chan_id, st->chip_info->name,
		 (axiadc_read(st, AXIADC_PCORE_IDENT) &
		AXIADC_PCORE_IDENT_SLAVE) ? "SLAVE" : "MASTER");

	if (iio_get_debugfs_dentry(indio_dev))
		debugfs_create_file("pseudorandom_err_check", 0644,
					iio_get_debugfs_dentry(indio_dev),
					indio_dev, &axiadc_debugfs_pncheck_fops);

	return 0;

failed4:
	axiadc_unconfigure_ring(indio_dev);
failed3:
	dma_release_channel(st->rx_chan);
failed2:
	release_mem_region(phys_addr, remap_size);
failed1:
	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);
	iio_device_free(indio_dev);
	dev_set_drvdata(dev, NULL);

	return ret;
}

/**
 * axiadc_of_remove - unbinds the driver from the AIM device.
 * @of_dev:	pointer to OF device structure
 *
 * This function is called if a device is physically removed from the system or
 * if the driver module is being unloaded. It frees any resources allocated to
 * the device.
 */
static int __devexit axiadc_of_remove(struct platform_device *op)
{
	struct device *dev = &op->dev;
	struct resource r_mem; /* IO mem resources */
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct axiadc_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	iio_buffer_unregister(indio_dev);
	axiadc_unconfigure_ring(indio_dev);

	dma_release_channel(st->rx_chan);

	put_device(st->dev_spi);
	module_put(st->dev_spi->driver->owner);

	iounmap(st->regs);

	/* Get iospace of the device */
	if (of_address_to_resource(op->dev.of_node, 0, &r_mem))
		dev_err(dev, "invalid address\n");
	else
		release_mem_region(r_mem.start, resource_size(&r_mem));

	iio_device_free(indio_dev);

	dev_set_drvdata(dev, NULL);

	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id axiadc_of_match[] __devinitconst = {
	{ .compatible = "xlnx,cf-ad9467-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9643-core-1.00.a", },
	{ .compatible = "xlnx,axi-adc-2c-1.00.a", },
	{ .compatible =	"xlnx,axi-adc-1c-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9250-1.00.a", },
	{ .compatible =	"xlnx,axi-ad9265-1.00.a", },
{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, axiadc_of_match);

static struct platform_driver axiadc_of_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.of_match_table = axiadc_of_match,
	},
	.probe		= axiadc_of_probe,
	.remove		= __devexit_p(axiadc_of_remove),
};

module_platform_driver(axiadc_of_driver);

MODULE_AUTHOR("Michael Hennerich <hennerich@blackfin.uclinux.org>");
MODULE_DESCRIPTION("Analog Devices ADI-AIM");
MODULE_LICENSE("GPL v2");
