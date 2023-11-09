/*
 * Copyright 2016 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/dmaengine.h>
#include <linux/bitfield.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>

#include "cf_axi_dds.h"

#define M2K_DAC_REG_VERSION	0x00
#define M2K_DAC_REG_OSR(x)	(0x40 + (x) * 0x8)
#define M2K_DAC_REG_FILTER(x)	(0x44 + (x) * 0x8)
#define M2K_DAC_REG_FLAGS	0x50
#define M2K_DAC_REG_CORRECTION_ENABLE 0x54
#define M2K_DAC_REG_CORRECTION_COEFFICIENT(x) (0x58 + (x) * 4)
#define M2K_DAC_REG_INSTRUMENT_TRIGGER 0x60
#define M2K_DAC_REG_RAW_PATTERN 0x64

#define M2K_DAC_DMA_SYNC_BIT	BIT(0)
#define M2K_DAC_SYNC_START_BIT	BIT(1)
#define M2K_DAC_FLAGS_DMA_FLUSH_BIT		BIT(3)
#define M2K_DAC_FLAGS_RAW_ENABLE_CHAN_A_BIT	BIT(4)
#define M2K_DAC_FLAGS_RAW_ENABLE_CHAN_B_BIT	BIT(5)
#define M2K_DAC_FLAGS_RAW_IDLE_BIT(chan_num)	(!(chan_num) ? M2K_DAC_FLAGS_RAW_ENABLE_CHAN_A_BIT :\
							     M2K_DAC_FLAGS_RAW_ENABLE_CHAN_B_BIT)

#define M2K_DAC_FLAGS_DMA_FLUSH(x)		FIELD_PREP(M2K_DAC_FLAGS_DMA_FLUSH_BIT, x)
#define M2K_DAC_FLAGS_RAW_IDLE(x, chan_num)	(!(chan_num) ? FIELD_PREP(M2K_DAC_FLAGS_RAW_ENABLE_CHAN_A_BIT, x) :\
							     FIELD_PREP(M2K_DAC_FLAGS_RAW_ENABLE_CHAN_B_BIT, x))

#define M2K_DAC_TRIGGER_CONDITION_MASK(x)	(0x155 << x)
#define M2K_DAC_TRIGGER_SOURCE_MASK		GENMASK(19, 16)
#define M2K_DAC_TRIGGER_EXT_SOURCE		GENMASK(17, 16)
#define M2K_DAC_REG_RAW_PATTERN_CHAN_A_MASK	GENMASK(15, 0)
#define M2K_DAC_REG_RAW_PATTERN_CHAN_B_MASK	GENMASK(31, 16)
#define M2K_DAC_RAW_PATTERN_MASK(chan_num)	(!(chan_num) ? M2K_DAC_REG_RAW_PATTERN_CHAN_A_MASK :\
							     M2K_DAC_REG_RAW_PATTERN_CHAN_B_MASK)
#define M2K_DAC_RAW_PATTERN(x, chan_num)	(!(chan_num) ? FIELD_PREP(M2K_DAC_REG_RAW_PATTERN_CHAN_A_MASK, x) :\
							     FIELD_PREP(M2K_DAC_REG_RAW_PATTERN_CHAN_B_MASK, x))

struct m2k_dac {
	void __iomem *regs;
	struct clk *clk;

	struct cf_axi_dds_state *dds;

	struct mutex lock;

	struct iio_dev *ch_indio_dev[2];
};

struct m2k_dac_ch {
	struct m2k_dac *dac;
	unsigned int num;
	unsigned char raw_idle_enable;
};

static unsigned int cf_axi_dds_to_signed_mag_fmt(int val, int val2)
{
	unsigned int i;
	u64 val64;

	if (val > 1) {
		val = 1;
		val2 = 999999;
	} else if (val < -1) {
		val = -1;
		val2 = 999999;
	}

	/*  format is 1.1.14 (sign, integer and fractional bits) */
	switch (val) {
	case 1:
		i = 0x4000;
		break;
	case -1:
		i = 0xC000;
		break;
	default: /* val = 0 */
		i = 0;
		if (val2 < 0) {
			i = 0x8000;
			val2 *= -1;
		}
		break;
	}

	val64 = (unsigned long long)val2 * 0x4000UL + (1000000UL / 2);
		do_div(val64, 1000000UL);

	if (i & 0x4000 && val64 == 0x4000)
		val64 = 0x3fff;

	return i | val64;
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

static int m2k_dac_reg_update(struct iio_dev *indio_dev, unsigned int reg,
			      unsigned int writeval, const unsigned int mask)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	unsigned int regval;

	reg &= 0xffff;

	regval = ioread32(m2k_dac->regs + reg);
	regval &= ~mask;
	writeval &= mask;
	iowrite32(writeval | regval, m2k_dac->regs + reg);

	return 0;
}

static int m2k_dac_ch_read_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int *val, int *val2, long info)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	unsigned int reg;
	int ret;

	mutex_lock(&m2k_dac->lock);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		if (ch->num == 0)
			*val = FIELD_GET(M2K_DAC_REG_RAW_PATTERN_CHAN_A_MASK,
					 ioread32(m2k_dac->regs + M2K_DAC_REG_RAW_PATTERN));
		else
			*val = FIELD_GET(M2K_DAC_REG_RAW_PATTERN_CHAN_B_MASK,
					 ioread32(m2k_dac->regs + M2K_DAC_REG_RAW_PATTERN));
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = clk_get_rate(m2k_dac->clk);
		reg = ioread32(m2k_dac->regs + M2K_DAC_REG_FILTER(ch->num));
		switch (reg) {
		case 1:
			*val /= 10;
			break;
		case 2:
			*val /= 100;
			break;
		case 3:
			*val /= 1000;
			break;
		case 6:
			*val /= 10000;
			break;
		case 7:
			*val /= 100000;
			break;
		default:
			break;
		}
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		*val = ioread32(m2k_dac->regs + M2K_DAC_REG_OSR(ch->num)) + 1;
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		reg = ioread32(m2k_dac->regs +
			       M2K_DAC_REG_CORRECTION_COEFFICIENT(ch->num));
		ret = cf_axi_dds_signed_mag_fmt_to_iio(reg, val, val2);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&m2k_dac->lock);

	return ret;
}

static int m2k_dac_ch_write_raw(struct iio_dev *indio_dev,
	struct iio_chan_spec const *chan, int val, int val2, long info)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	unsigned long rate;
	unsigned int reg;
	int ret = 0;

	mutex_lock(&m2k_dac->lock);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		m2k_dac_reg_update(indio_dev, M2K_DAC_REG_RAW_PATTERN,
				   M2K_DAC_RAW_PATTERN(val, ch->num),
				   M2K_DAC_RAW_PATTERN_MASK(ch->num));
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		rate = clk_get_rate(m2k_dac->clk);
		if (val >= rate)
			reg = 0;
		else if (val >= rate / 10)
			reg = 1;
		else if (val >= rate / 100)
			reg = 2;
		else if (val >= rate / 1000)
			reg = 3;
		else if (val >= rate / 10000)
			reg = 6;
		else
			reg = 7;
		iowrite32(reg, m2k_dac->regs + M2K_DAC_REG_FILTER(ch->num));
		break;
	case IIO_CHAN_INFO_OVERSAMPLING_RATIO:
		if (val <= 0) {
			ret = -EINVAL;
			break;
		}
		iowrite32(val - 1, m2k_dac->regs + M2K_DAC_REG_OSR(ch->num));
		break;
	case IIO_CHAN_INFO_CALIBSCALE:
		reg = ioread32(m2k_dac->regs + M2K_DAC_REG_CORRECTION_ENABLE);
		if (val == 1 && val2 == 0)
			reg &= ~BIT(ch->num);
		else
			reg |= BIT(ch->num);
		iowrite32(reg, m2k_dac->regs + M2K_DAC_REG_CORRECTION_ENABLE);

		reg = cf_axi_dds_to_signed_mag_fmt(val, val2);
		iowrite32(reg, m2k_dac->regs +
			  M2K_DAC_REG_CORRECTION_COEFFICIENT(ch->num));
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&m2k_dac->lock);

	return ret;
}

static int m2k_dac_reg_access(struct iio_dev *indio_dev, unsigned int reg,
	unsigned int writeval, unsigned int *readval)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;

	reg &= 0xffff;

	if (readval == NULL)
		iowrite32(writeval, m2k_dac->regs + reg);
	else
		*readval = ioread32(m2k_dac->regs + reg);

	return 0;
}

static ssize_t m2k_dac_read_dma_sync(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	unsigned int val;

	val = ioread32(m2k_dac->regs + M2K_DAC_REG_FLAGS);
	val &= 1;

	return sprintf(buf, "%d\n", val);
}

static ssize_t m2k_dac_write_dma_sync(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	bool val;
	int ret;

	ret = strtobool(buf, &val);
	if (ret < 0)
		return ret;

	m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS, val,
			   M2K_DAC_DMA_SYNC_BIT);

	return len;
}

static ssize_t m2k_dac_read_dma_sync_start(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	unsigned int val;

	val = ioread32(m2k_dac->regs + M2K_DAC_REG_FLAGS);
	val &= M2K_DAC_SYNC_START_BIT;

	return sprintf(buf, "%d\n", val >> 1);
}

static ssize_t m2k_dac_write_dma_sync_start(struct device *dev,
					    struct device_attribute *attr,
					    const char *buf, size_t len)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	bool val;
	int ret;

	ret = strtobool(buf, &val);
	if (ret < 0)
		return ret;

	m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS,
			   val << 1,
			   M2K_DAC_SYNC_START_BIT);

	return len;
}

static IIO_DEVICE_ATTR(dma_sync, 0664,
	m2k_dac_read_dma_sync, m2k_dac_write_dma_sync, 0);

static IIO_DEVICE_ATTR(dma_sync_start, 0664,
	m2k_dac_read_dma_sync_start, m2k_dac_write_dma_sync_start, 0);

static struct attribute *m2k_dac_attributes[] = {
	&iio_dev_attr_dma_sync.dev_attr.attr,
	&iio_dev_attr_dma_sync_start.dev_attr.attr,
	NULL
};

static const struct attribute_group m2k_dac_attribute_group = {
	.attrs = m2k_dac_attributes,
};

static const char * const m2k_dac_samp_freq_available[] = {
	"750",
	"7500",
	"75000",
	"750000",
	"7500000",
	"75000000"
};

static const struct iio_enum m2k_dac_samp_freq_available_enum = {
	.items = m2k_dac_samp_freq_available,
	.num_items = ARRAY_SIZE(m2k_dac_samp_freq_available),
};

static int m2k_dac_set_trig_condition(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	int trig_src;

	mutex_lock(&m2k_dac->lock);

	trig_src = ioread32(m2k_dac->regs + M2K_DAC_REG_INSTRUMENT_TRIGGER);
	trig_src = FIELD_GET(M2K_DAC_TRIGGER_EXT_SOURCE, trig_src);

	if (trig_src) {
		trig_src -= 1;

		if (val)
			/* Subtract 1 because of the none item. Multiply with 2
			 * and shift with trig_src due to channel selection
			 */
			val = BIT(2 * (val - 1)) << trig_src;

		m2k_dac_reg_update(indio_dev, M2K_DAC_REG_INSTRUMENT_TRIGGER,
				val, M2K_DAC_TRIGGER_CONDITION_MASK(trig_src));
	}

	mutex_unlock(&m2k_dac->lock);

	return 0;
}

static int m2k_dac_get_trig_condition(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	int val, trig_src;

	trig_src = val =
		ioread32(m2k_dac->regs + M2K_DAC_REG_INSTRUMENT_TRIGGER);
	trig_src = FIELD_GET(M2K_DAC_TRIGGER_EXT_SOURCE, trig_src);

	if (trig_src) {
		trig_src -= 1;
		val &= M2K_DAC_TRIGGER_CONDITION_MASK(trig_src);
		return ((fls(val >> trig_src) + 1) / 2);
	}

	return 0;
}

static const char * const m2k_dac_trig_condition_items[] = {
	"none",
	"level-low",
	"level-high",
	"edge-any",
	"edge-rising",
	"edge-falling",
};

static const struct iio_enum m2k_dac_trig_condition_enum = {
	.items = m2k_dac_trig_condition_items,
	.num_items = ARRAY_SIZE(m2k_dac_trig_condition_items),
	.set = m2k_dac_set_trig_condition,
	.get = m2k_dac_get_trig_condition,
};

static int m2k_dac_set_trig_src(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;

	mutex_lock(&m2k_dac->lock);

	/* HDL required reset */
	m2k_dac_reg_update(indio_dev, M2K_DAC_REG_INSTRUMENT_TRIGGER, 0x0,
			   M2K_DAC_TRIGGER_SOURCE_MASK);

	val = BIT(val) << 15;
	m2k_dac_reg_update(indio_dev, M2K_DAC_REG_INSTRUMENT_TRIGGER, val,
			   M2K_DAC_TRIGGER_SOURCE_MASK);

	mutex_unlock(&m2k_dac->lock);

	return 0;
}

static int m2k_dac_get_trig_src(struct iio_dev *indio_dev,
	const struct iio_chan_spec *chan)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;
	int val;

	val = ioread32(m2k_dac->regs + M2K_DAC_REG_INSTRUMENT_TRIGGER);

	if (val & M2K_DAC_TRIGGER_SOURCE_MASK)
		return fls(val) - 16;

	return 0;
}

static const char * const m2k_dac_trig_src_items[] = {
	"none",
	"trigger-i_0",
	"trigger-i_1",
	"trigger-adc",
	"trigger-la",
};

static const struct iio_enum m2k_dac_trig_src_enum = {
	.items = m2k_dac_trig_src_items,
	.num_items = ARRAY_SIZE(m2k_dac_trig_src_items),
	.set = m2k_dac_set_trig_src,
	.get = m2k_dac_get_trig_src,
};

static int m2k_dac_get_raw_enable(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);

	return ch->raw_idle_enable;
}

static int m2k_dac_set_raw_enable(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan, unsigned int val)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	int ret;

	ch->raw_idle_enable = val;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return 0;

	m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS,
			   M2K_DAC_FLAGS_RAW_IDLE(val, ch->num),
			   M2K_DAC_FLAGS_RAW_IDLE_BIT(ch->num));

	iio_device_release_direct_mode(indio_dev);

	return 0;
}

static const char * const m2k_dac_raw_enable_items[] = {
	"disabled",
	"enabled",
};

static const struct iio_enum m2k_dac_raw_enable_enum = {
	.items = m2k_dac_raw_enable_items,
	.num_items = ARRAY_SIZE(m2k_dac_raw_enable_items),
	.set = m2k_dac_set_raw_enable,
	.get = m2k_dac_get_raw_enable,
};

static const struct iio_chan_spec_ext_info m2k_dac_ext_info[] = {
	IIO_ENUM_AVAILABLE("sampling_frequency", IIO_SHARED_BY_ALL,
			   &m2k_dac_samp_freq_available_enum),
	IIO_ENUM("trigger_src", IIO_SHARED_BY_ALL, &m2k_dac_trig_src_enum),
	IIO_ENUM_AVAILABLE("trigger_src", IIO_SHARED_BY_ALL, &m2k_dac_trig_src_enum),
	IIO_ENUM("trigger_condition", IIO_SHARED_BY_ALL,
		 &m2k_dac_trig_condition_enum),
	IIO_ENUM_AVAILABLE("trigger_condition", IIO_SHARED_BY_ALL,
			   &m2k_dac_trig_condition_enum),
	IIO_ENUM_AVAILABLE("raw_enable", IIO_SEPARATE, &m2k_dac_raw_enable_enum),
	IIO_ENUM("raw_enable", IIO_SEPARATE, &m2k_dac_raw_enable_enum),
	{ },
};

static const struct iio_chan_spec m2k_dac_channel_info = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ) |
		BIT(IIO_CHAN_INFO_OVERSAMPLING_RATIO) |
		BIT(IIO_CHAN_INFO_CALIBSCALE),
	.output = 1,
	.scan_index = 0,
	.ext_info = m2k_dac_ext_info,
	.scan_type = {
		.sign = 's',
		.storagebits = 16,
		.realbits = 16,
		.shift = 0,
	},
};

static int m2k_dac_buffer_preenable(struct iio_dev *indio_dev)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;

	if (iio_buffer_enabled(m2k_dac->ch_indio_dev[0]) !=
	    iio_buffer_enabled(m2k_dac->ch_indio_dev[1])) {
		m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS, 0,
				   M2K_DAC_SYNC_START_BIT);
	}
	m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS,
			   M2K_DAC_FLAGS_RAW_IDLE(0, ch->num), M2K_DAC_FLAGS_RAW_IDLE_BIT(ch->num));
	return 0;
}

/*
 * The raw flag must be enabled prior to the destruction of the iio_buffer due
 *  to the specifics of the HDL DMA implementation. If the flag is enabled
 *  post-destruction, it could result in improper data flushing by the DMA.
 *
 * Consequently, upon the next buffer enablement, a spike will occur, displaying
 *  the last sample from the previous buffer.
 */
static int m2k_dac_buffer_predisable(struct iio_dev *indio_dev)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);

	if (ch->raw_idle_enable)
		m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS,
				   M2K_DAC_FLAGS_RAW_IDLE(1, ch->num),
				   M2K_DAC_FLAGS_RAW_IDLE_BIT(ch->num));

	return 0;
}

static int m2k_dac_buffer_postdisable(struct iio_dev *indio_dev)
{
	struct m2k_dac_ch *ch = iio_priv(indio_dev);
	struct m2k_dac *m2k_dac = ch->dac;

	if (iio_buffer_enabled(m2k_dac->ch_indio_dev[0]) !=
	    iio_buffer_enabled(m2k_dac->ch_indio_dev[1])) {
		m2k_dac_reg_update(indio_dev, M2K_DAC_REG_FLAGS, 0,
				   M2K_DAC_SYNC_START_BIT);
	}

	return 0;
}

static const struct iio_buffer_setup_ops m2k_dac_buffer_setup_ops = {
	.preenable = m2k_dac_buffer_preenable,
	.predisable = m2k_dac_buffer_predisable,
	.postdisable = m2k_dac_buffer_postdisable,
};

static const struct iio_info m2k_dac_ch_info = {
	.read_raw = m2k_dac_ch_read_raw,
	.write_raw = m2k_dac_ch_write_raw,
	.debugfs_reg_access = m2k_dac_reg_access,
	.attrs = &m2k_dac_attribute_group,
};

static const char * const m2k_dac_ch_dev_names[] = {
	"m2k-dac-a",
	"m2k-dac-b",
};

static const char * const m2k_dac_ch_dma_names[] = {
	"tx0", "tx1"
};

static int m2k_dac_alloc_channel(struct platform_device *pdev,
	struct m2k_dac *m2k_dac, unsigned int num)
{
	struct iio_dev *indio_dev;
	struct m2k_dac_ch *ch;
	int ret;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*ch));
	if (!indio_dev)
		return -ENOMEM;

	ch = iio_priv(indio_dev);
	ch->num = num;
	ch->dac = m2k_dac;
	ch->raw_idle_enable = 0;

	indio_dev->dev.parent = &pdev->dev;
	indio_dev->name = m2k_dac_ch_dev_names[num];
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_HARDWARE;
	indio_dev->setup_ops = &m2k_dac_buffer_setup_ops;
	indio_dev->info = &m2k_dac_ch_info;
	indio_dev->channels = &m2k_dac_channel_info;
	indio_dev->num_channels = 1;

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent, indio_dev,
					      m2k_dac_ch_dma_names[num],
					      IIO_BUFFER_DIRECTION_OUT);
	if (ret)
		return ret;

	m2k_dac->ch_indio_dev[num] = indio_dev;

	return 0;
}

static int dds_dev_match(struct device *dev, const void *data)
{
	int ret;

	if (dev->of_node != data)
		return false;

	/* Make sure probe() has finished running */
	device_lock(dev);
	ret = dev->driver != NULL;
	device_unlock(dev);

	return ret;
}

static int m2k_dac_attach_dds(struct platform_device *pdev,
	struct m2k_dac *m2k_dac)
{
	struct device_node *dds_of;
	struct device *dds_dev;
	struct iio_dev *dds_indio_dev;

	dds_of = of_parse_phandle(pdev->dev.of_node, "dds-connected", 0);
	if (!dds_of)
		return -ENODEV;

	dds_dev = bus_find_device(&platform_bus_type, NULL, dds_of,
				  dds_dev_match);
	of_node_put(dds_of);
	if (!dds_dev)
		return -EPROBE_DEFER;

	dds_indio_dev = dev_get_drvdata(dds_dev);

	/* This device better not unregister while we still use it */
	put_device(dds_dev);

	if (!dds_indio_dev)
		return -EPROBE_DEFER;

	m2k_dac->dds = iio_priv(dds_indio_dev);

	return 0;
}

static void m2k_clk_disable(void *clk)
{
	clk_disable_unprepare(clk);
}

static int m2k_dac_probe(struct platform_device *pdev)
{
	struct m2k_dac *m2k_dac;
	struct resource *res;
	unsigned int reg;
	int ret;

	m2k_dac = devm_kzalloc(&pdev->dev, sizeof(*m2k_dac), GFP_KERNEL);
	if (!m2k_dac)
		return -ENOMEM;

	mutex_init(&m2k_dac->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	m2k_dac->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(m2k_dac->regs))
		return PTR_ERR(m2k_dac->regs);

	m2k_dac->clk = devm_clk_get(&pdev->dev, "dac_clk");
	if (IS_ERR(m2k_dac->clk))
		return PTR_ERR(m2k_dac->clk);

	ret = clk_prepare_enable(m2k_dac->clk);
	if (ret < 0)
		return ret;

	ret = devm_add_action_or_reset(&pdev->dev, m2k_clk_disable, m2k_dac->clk);
	if (ret)
		return ret;

	reg = cf_axi_dds_to_signed_mag_fmt(1, 0);
	iowrite32(reg, m2k_dac->regs + M2K_DAC_REG_CORRECTION_COEFFICIENT(0));
	iowrite32(reg, m2k_dac->regs + M2K_DAC_REG_CORRECTION_COEFFICIENT(1));

	ret = m2k_dac_attach_dds(pdev, m2k_dac);
	if (ret)
		return ret;

	ret = m2k_dac_alloc_channel(pdev, m2k_dac, 0);
	if (ret)
		return ret;

	ret = m2k_dac_alloc_channel(pdev, m2k_dac, 1);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, m2k_dac);

	cf_axi_dds_datasel(m2k_dac->dds, 0, DATA_SEL_DMA);
	cf_axi_dds_datasel(m2k_dac->dds, 1, DATA_SEL_DMA);

	ret = devm_iio_device_register(&pdev->dev, m2k_dac->ch_indio_dev[0]);
	if (ret)
		return ret;

	return devm_iio_device_register(&pdev->dev, m2k_dac->ch_indio_dev[1]);
}

static const struct of_device_id m2k_dac_of_match[] = {
	{
		.compatible = "adi,m2k-dac-1.00.a",
		.data = NULL,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, m2k_dac_match);

static struct platform_driver m2k_dac_driver = {
	.driver = {
		.name = "m2k_dac",
		.owner = THIS_MODULE,
		.of_match_table = m2k_dac_of_match,
	},
	.probe = m2k_dac_probe,
};
module_platform_driver(m2k_dac_driver);

MODULE_AUTHOR("Lars-Peter Clausen <lars@metafoo.de>");
MODULE_DESCRIPTION("Analog Devices M2K DAC driver");
MODULE_LICENSE("GPL v2");
