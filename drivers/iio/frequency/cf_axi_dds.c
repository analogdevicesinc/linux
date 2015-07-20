/*
 * DDS PCORE/COREFPGA Module
 *
 * Copyright 2012-2014 Analog Devices Inc.
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

unsigned cf_axi_dds_to_signed_mag_fmt(int val, int val2)
{
	unsigned i;
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
	}

	val64 = (unsigned long long)val2 * 0x4000UL + (1000000UL / 2);
		do_div(val64, 1000000UL);

	return i | val64;
}

int cf_axi_dds_signed_mag_fmt_to_iio(unsigned val, int *r_val, int *r_val2)
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

#ifdef CF_AXI_DDS_HAVE_TWOS_FMT
unsigned cf_axi_dds_to_twos_fmt(int val, int val2)
{
	s64 sval64;
	if (val < 0)
		val2 = -val2;
	sval64 = ((val * 1000000ULL + (long long)val2) * 0x4000);
	return div_s64(sval64, 1000000UL);
}

int cf_axi_dds_twos_fmt_to_iio(s16 val, int *r_val, int *r_val2)
{
	*r_val = val;
	*r_val2 = 14;

	return IIO_VAL_FRACTIONAL_LOG2;
}
#endif

int cf_axi_dds_datasel(struct cf_axi_dds_state *st,
			       int channel, enum dds_data_select sel)
{
	if (PCORE_VERSION_MAJOR(st->version) > 7) {
		if (channel < 0) { /* ALL */
			unsigned i;
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
			if ((unsigned)channel > (st->have_slave_channels - 1))
				dds_slave_write(st,
					ADI_REG_CHAN_CNTRL_7(channel -
					st->have_slave_channels), sel);
			else
				dds_write(st, ADI_REG_CHAN_CNTRL_7(channel),
					  sel);
		}
	} else {
		unsigned reg;

		switch(sel) {
		case DATA_SEL_DDS:
		case DATA_SEL_SED:
		case DATA_SEL_DMA:
			reg = dds_read(st, ADI_REG_CNTRL_2) & ~ADI_DATA_SEL(~0);
			dds_write(st, ADI_REG_CNTRL_2, reg | ADI_DATA_SEL(sel));
			break;
		default:
			return -EINVAL;
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(cf_axi_dds_datasel);

static enum dds_data_select cf_axi_dds_get_datasel(struct cf_axi_dds_state *st,
			       int channel)
{
	if (PCORE_VERSION_MAJOR(st->version) > 7) {
		if (channel < 0)
			channel = 0;

		if ((unsigned)channel > (st->have_slave_channels - 1))
			return dds_slave_read(st,
				ADI_REG_CHAN_CNTRL_7(channel - st->have_slave_channels));

		return dds_read(st, ADI_REG_CHAN_CNTRL_7(channel));
	} else {
		return ADI_TO_DATA_SEL(dds_read(st, ADI_REG_CNTRL_2));
	}
}

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

void cf_axi_dds_stop(struct cf_axi_dds_state *st)
{
	if (PCORE_VERSION_MAJOR(st->version) < 8)
		dds_write(st, ADI_REG_CNTRL_1, 0);
}
EXPORT_SYMBOL_GPL(cf_axi_dds_stop);

void cf_axi_dds_start_sync(struct cf_axi_dds_state *st, bool force_on)
{
	if (PCORE_VERSION_MAJOR(st->version) < 8) {
		dds_write(st, ADI_REG_CNTRL_1, (st->enable || force_on) ? ADI_ENABLE : 0);
	} else {
		dds_write(st, ADI_REG_CNTRL_1, ADI_SYNC);
		dds_master_write(st, ADI_REG_CNTRL_1, ADI_SYNC);
	}
}
EXPORT_SYMBOL_GPL(cf_axi_dds_start_sync);

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
		cf_axi_dds_start_sync(st, 0);
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

	cf_axi_dds_stop(st);

	cf_axi_dds_datasel(st, -1, DATA_SEL_SED);

	ctrl = dds_read(st, ADI_REG_CNTRL_2);
	dds_write(st, ADI_REG_CNTRL_2, ctrl | ADI_DATA_FORMAT);

	cf_axi_dds_start_sync(st, 1);
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

	if (PCORE_VERSION_MAJOR(st->version) > 7) {
		if (chan % 2)
			dds_write(st, ADI_REG_CHAN_CNTRL_8(chan),
				ADI_IQCOR_COEFF_2(0x4000) |
				ADI_IQCOR_COEFF_1(0));
		else
			dds_write(st, ADI_REG_CHAN_CNTRL_8(chan),
				ADI_IQCOR_COEFF_2(0) |
				ADI_IQCOR_COEFF_1(0x4000));
	}

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
	unsigned reg, phase = 0;
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
		reg = ADI_TO_DDS_SCALE(dds_read(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel)));
		if (PCORE_VERSION_MAJOR(st->version) > 6) {
			cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
		} else {
			if (!reg) {
				*val = 1;
				*val2 = 0;
			} else {
				*val = 0;
				*val2 = 1000000 >> reg;
			}
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
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
	case IIO_CHAN_INFO_CALIBSCALE:

		if (PCORE_VERSION_MAJOR(st->version) < 8) {
			ret = -ENODEV;
			break;
		}

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_8(chan->channel));
		/*  format is 1.1.14 (sign, integer and fractional bits) */

		if (!((phase + chan->channel) % 2)) {
			reg = ADI_TO_IQCOR_COEFF_1(reg);
		} else {
			reg = ADI_TO_IQCOR_COEFF_2(reg);
		}

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
	unsigned reg, i, phase = 0;
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
		cf_axi_dds_start_sync(st, 0);
		cf_axi_dds_datasel(st, -1, st->enable ? DATA_SEL_DDS : DATA_SEL_ZERO);

		break;
	case IIO_CHAN_INFO_SCALE:
		if (PCORE_VERSION_MAJOR(st->version) > 6) {
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
		} else {
			if (val == 1) {
				i = 0;
			} else {
				for (i = 1; i < 16; i++)
					if (val2 == (1000000 >> i))
						break;
			}
		}
		cf_axi_dds_stop(st);
		dds_write(st, ADI_REG_CHAN_CNTRL_1_IIOCHAN(chan->channel), ADI_DDS_SCALE(i));
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

		st->cached_freq[chan->channel] = val;
		cf_axi_dds_stop(st);
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		reg &= ~ADI_DDS_INCR(~0);
		val64 = (u64) val * 0xFFFFULL;
		do_div(val64, st->dac_clk);
		reg |= ADI_DDS_INCR(val64) | 1;
		dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel), reg);
		cf_axi_dds_start_sync(st, 0);
		break;
	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > 360000) {
			ret = -EINVAL;
			break;
		}

		if (val == 360000)
			val = 0;

		cf_axi_dds_stop(st);
		reg = dds_read(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel));
		reg &= ~ADI_DDS_INIT(~0);
		val64 = (u64) val * 0x10000ULL + (360000 / 2);
		do_div(val64, 360000);
		reg |= ADI_DDS_INIT(val64);
		dds_write(st, ADI_REG_CHAN_CNTRL_2_IIOCHAN(chan->channel), reg);
		cf_axi_dds_start_sync(st, 0);
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
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
		cf_axi_dds_stop(st);
		conv->write_raw(indio_dev, chan, val, val2, mask);
		dds_write(st, ADI_REG_CNTRL_2, reg);
		cf_axi_dds_datasel(st, -1, i);
		st->dac_clk = conv->get_data_clk(conv);
		cf_axi_dds_start_sync(st, 0);
		ret = cf_axi_dds_sync_frame(indio_dev);
		break;
	case IIO_CHAN_INFO_CALIBPHASE:
		phase = 1;
	case IIO_CHAN_INFO_CALIBSCALE:

		if (PCORE_VERSION_MAJOR(st->version) < 7) {
			ret = -ENODEV;
			break;
		}

		i = cf_axi_dds_to_signed_mag_fmt(val, val2);

		reg = dds_read(st, ADI_REG_CHAN_CNTRL_8(chan->channel));

		if (!((chan->channel + phase) % 2)) {
			reg &= ~ADI_IQCOR_COEFF_1(~0);
			reg |= ADI_IQCOR_COEFF_1(i);
		} else {
			reg &= ~ADI_IQCOR_COEFF_2(~0);
			reg |= ADI_IQCOR_COEFF_2(i);
		}

		dds_write(st, ADI_REG_CHAN_CNTRL_8(chan->channel), reg);
		dds_write(st, ADI_REG_CHAN_CNTRL_6(chan->channel), ADI_IQCOR_ENB);
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
			      unsigned reg, unsigned writeval,
			      unsigned *readval)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	struct cf_axi_converter *conv = ERR_PTR(-ENODEV);
	int ret;

	if ((reg & ~DEBUGFS_DRA_PCORE_REG_MAGIC) > 0xFFFF)
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

static int cf_axi_dds_update_scan_mode(struct iio_dev *indio_dev,
	const unsigned long *scan_mask)
{
	struct cf_axi_dds_state *st = iio_priv(indio_dev);
	unsigned i, sel;

	for (i = 0; i < indio_dev->masklength; i++) {

		if (test_bit(i, scan_mask)) {
			sel = DATA_SEL_DMA;
		} else {
			sel = DATA_SEL_DDS;
		}

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
			struct iio_chan_spec *channels, unsigned num)
{

	if (PCORE_VERSION_MAJOR(st->version) > 6) {
		int i;
		for (i = 0; i < num; i++) {
			if (channels[i].type == IIO_ALTVOLTAGE)
				channels[i].ext_info = NULL;
		}
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
	.output = 1, \
	.scan_index = _chan, \
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

static const struct iio_info cf_axi_dds_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &cf_axi_dds_read_raw,
	.write_raw = &cf_axi_dds_write_raw,
	.debugfs_reg_access = &cf_axi_dds_reg_access,
	.update_scan_mode = &cf_axi_dds_update_scan_mode,
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
	struct cf_axi_dds_chip_info *chip_info;
	unsigned int data_format;
	unsigned int rate;
};

static const struct axidds_core_info ad9122_6_00_a_info = {
	.version = PCORE_VERSION(8, 0, 'a'),
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
	.version = PCORE_VERSION(8, 0, 'a'),
	.has_fifo_interface = true,
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9361,
};

static const struct axidds_core_info ad9364_6_00_a_info = {
	.version = PCORE_VERSION(8, 0, 'a'),
	.has_fifo_interface = true,
	.standalone = true,
	.rate = 1,
	.chip_info = &cf_axi_dds_chip_info_ad9364,
};

static const struct axidds_core_info ad9361x2_6_00_a_info = {
	.version = PCORE_VERSION(8, 0, 'a'),
	.has_fifo_interface = true,
	.standalone = true,
	.rate = 3,
	.chip_info = &cf_axi_dds_chip_info_ad9361x2,
};

static const struct axidds_core_info ad9144_7_00_a_info = {
	.version = PCORE_VERSION(8, 0, 'a'),
	.has_fifo_interface = true,
	.rate = 1,
};

static const struct axidds_core_info ad9739a_8_00_b_info = {
	.version = PCORE_VERSION(8, 0, 'b'),
	.has_fifo_interface = true,
	.rate = 1,
	.data_format = ADI_DATA_FORMAT,
};

/* Match table for of_platform binding */
static const struct of_device_id cf_axi_dds_of_match[] = {
	{ .compatible = "xlnx,cf-ad9122-core-1.00.a", },
	{ .compatible = "adi,axi-ad9122-6.00.a", .data = &ad9122_6_00_a_info},
	{ .compatible = "adi,axi-ad9144-1.0", .data = &ad9144_7_00_a_info},
	{ .compatible = "xlnx,cf-ad9739a-core-1.00.a", },
	{ .compatible = "adi,axi-ad9739a-8.00.b", .data = &ad9739a_8_00_b_info},
	{ .compatible = "xlnx,cf-ad9122x2-core-1.00.a", },
	{ .compatible = "xlnx,cf-ad9122-core-2.00.a", },
	{ .compatible = "xlnx,axi-dac-4d-2c-1.00.a", },
	{
	    .compatible = "xlnx,axi-ad9361-dds-1.00.a",
	    .data = &ad9361_1_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9361x2-dds-6.00.a",
	    .data = &ad9361x2_6_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9361-dds-6.00.a",
	    .data = &ad9361_6_00_a_info,
	}, {
	    .compatible = "adi,axi-ad9364-dds-6.00.a",
	    .data = &ad9364_6_00_a_info,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, cf_axi_dds_of_match);

static int cf_axi_dds_probe(struct platform_device *pdev)
{

	struct device_node *np = pdev->dev.of_node;
	unsigned int expected_version;
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


	if (info) {
		st->has_fifo_interface = info->has_fifo_interface;
		st->standalone = info->standalone;
	}

	st->version = dds_read(st, ADI_REG_VERSION);
	st->dp_disable = dds_read(st, ADI_REG_DAC_DP_DISABLE);

	if (info)
		expected_version = info->version;
	else
		expected_version = PCORE_VERSION(4, 0, 'a');

	if (PCORE_VERSION_MAJOR(st->version) >
		PCORE_VERSION_MAJOR(expected_version)) {
		dev_err(&pdev->dev, "Major version mismatch between PCORE and driver. Driver expected %d.%.2d.%c, PCORE reported %d.%.2d.%c\n",
			PCORE_VERSION_MAJOR(expected_version),
			PCORE_VERSION_MINOR(expected_version),
			PCORE_VERSION_LETTER(expected_version),
			PCORE_VERSION_MAJOR(st->version),
			PCORE_VERSION_MINOR(st->version),
			PCORE_VERSION_LETTER(st->version));
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

	cf_axi_dds_stop(st);
	dds_write(st, ADI_REG_CNTRL_2, ctrl_2);

	cf_axi_dds_datasel(st, -1, DATA_SEL_DDS);

	if (!st->dp_disable) {
		unsigned scale;
		if (PCORE_VERSION_MAJOR(st->version) > 6)
			scale = 0x1000; /* 0.250 */
		else
			scale = 2; /* 0.250 */

		of_property_read_u32(np, "adi,axi-dds-default-scale", &scale);

		cf_axi_dds_default_setup(st, 0, 90000, 40000000, scale);
		cf_axi_dds_default_setup(st, 1, 90000, 40000000, scale);


		if (st->chip_info->num_dds_channels >= 4) {
			cf_axi_dds_default_setup(st, 2, 0, 40000000, scale);
			cf_axi_dds_default_setup(st, 3, 0, 40000000, scale);
		}

		if (st->chip_info->num_dds_channels >= 8) {
			cf_axi_dds_default_setup(st, 4, 90000, 1000000, scale);
			cf_axi_dds_default_setup(st, 5, 90000, 1000000, scale);
			cf_axi_dds_default_setup(st, 6, 0, 1000000, scale);
			cf_axi_dds_default_setup(st, 7, 0, 1000000, scale);
		}

		cf_axi_dds_update_chan_spec(st, st->chip_info->channel,
				st->chip_info->num_channels);

	}

	st->enable = true;
	cf_axi_dds_start_sync(st, 0);
	cf_axi_dds_sync_frame(indio_dev);

	if (!st->dp_disable && !dds_read(st, ADI_REG_ID)) {

		if (st->chip_info->num_shadow_slave_channels) {
			u32 regs[2];
			ret = of_property_read_u32_array(pdev->dev.of_node,
					"slavecore-reg", regs, ARRAY_SIZE(regs));
			if (!ret) {
				st->slave_regs = ioremap(regs[0], regs[1]);
				if (st->slave_regs)
					st->have_slave_channels = st->chip_info->
						num_shadow_slave_channels;

			}
		}

		ret = cf_axi_dds_configure_buffer(indio_dev);
		if (ret)
			goto err_converter_put;

		indio_dev->available_scan_masks = st->chip_info->scan_masks;

	} else if (dds_read(st, ADI_REG_ID)){
		u32 regs[2];
		ret = of_property_read_u32_array(pdev->dev.of_node,
				"mastercore-reg", regs, ARRAY_SIZE(regs));
		if (!ret) {
			st->master_regs = ioremap(regs[0], regs[1]);
		}
	}

	ret = iio_device_register(indio_dev);
	if (ret)
		goto err_unconfigure_buffer;

	dev_info(&pdev->dev, "Analog Devices CF_AXI_DDS_DDS %s (%d.%.2d.%c) at 0x%08llX mapped"
		" to 0x%p, probed DDS %s\n",
		dds_read(st, ADI_REG_ID) ? "SLAVE" : "MASTER",
		PCORE_VERSION_MAJOR(st->version),
		PCORE_VERSION_MINOR(st->version),
		PCORE_VERSION_LETTER(st->version),
		(unsigned long long)res->start, st->regs, st->chip_info->name);

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
err_iio_device_free:
	iio_device_free(indio_dev);

	return ret;
}

static int cf_axi_dds_remove(struct platform_device *pdev)
{
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct cf_axi_dds_state *st = iio_priv(indio_dev);

	iio_device_unregister(indio_dev);
	cf_axi_dds_unconfigure_buffer(indio_dev);
	if (st->dev_spi)
		dds_converter_put(st->dev_spi);
	if (st->clk) {
		clk_notifier_unregister(st->clk, &st->clk_nb);
		clk_disable_unprepare(st->clk);
	}
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
