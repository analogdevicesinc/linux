// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (C) 2023 Analog Devices, Inc.
 * Author: Ana-Maria Cusco <ana-maria.cusco@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine-ex.h>
#include <linux/units.h>

#include <asm/div64.h>
#include <asm/unaligned.h>

#include <linux/iio/buffer.h>
#include <linux/iio/buffer-dma.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/iio.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/iio/trigger_consumer.h>
#include "ad4170.h"

struct ad4170_slot_info {
	struct ad4170_setup		setup;
	unsigned int			enabled_channels;
	unsigned int			channels;
};

struct ad4170_chan_info {
	struct ad4170_current_source current_source[AD4170_NUM_CURRENT_SOURCE];
	unsigned int nr;
	int slot;
	u32 scale_tbl[10][2];
	bool enabled;
};

struct ad4170_state {
	struct regmap *regmap;
	bool spi_is_dma_mapped;
	struct spi_device *spi;
	struct clk *mclk;
	struct regulator_bulk_data regulators[4];
	struct mutex lock; /* Protect filter, PGA, GPIO, and channel config */
	struct ad4170_chan_info *chan_info;
	struct ad4170_slot_info slots_info[AD4170_NUM_SETUPS];
	unsigned int num_channels;
	u32 fclk;
	u8 reg_read_tx_buf[2];
	u8 reg_read_rx_buf[4];
	u8 reg_write_tx_buf[6];
	u32 vbias_pins[AD4170_MAX_ANALOG_PINS];
	u32 num_vbias_pins;
	struct gpio_desc *dig_aux1_gpio;
	struct gpio_desc *dig_aux2_gpio;
	struct gpio_desc *sync_gpio;
	struct completion completion;
	struct ad4170_config cfg;
	struct iio_trigger *trig;
	u32 data[AD4170_NUM_CHANNELS];
	struct gpio_chip gpiochip;
	bool pdsw0;
	bool pdsw1;
	u32 chop_adc;

	struct spi_transfer xfer;
	struct spi_message msg;
	unsigned int rx_data[2] __aligned(IIO_DMA_MINALIGN);
	unsigned int tx_data[2];
};

static const unsigned int ad4170_iexc_chop_tbl[AD4170_IEXC_CHOP_MAX] = {
	[AD4170_CHOP_IEXC_OFF] = AD4170_MISC_CHOP_IEXC_OFF,
	[AD4170_CHOP_IEXC_AB] = AD4170_MISC_CHOP_IEXC_AB,
	[AD4170_CHOP_IEXC_CD] = AD4170_MISC_CHOP_IEXC_CD,
	[AD4170_CHOP_IEXC_ABCD] = AD4170_MISC_CHOP_IEXC_ABCD,
};

static const unsigned int ad4170_iout_current_ua_tbl[AD4170_I_OUT_MAX] = {
	[AD4170_I_OUT_0UA] = 0,
	[AD4170_I_OUT_10UA] = 10,
	[AD4170_I_OUT_50UA] = 50,
	[AD4170_I_OUT_100UA] = 100,
	[AD4170_I_OUT_250UA] = 250,
	[AD4170_I_OUT_500UA] = 500,
	[AD4170_I_OUT_1000UA] = 1000,
	[AD4170_I_OUT_1500UA] = 1500,
};

static const unsigned int ad4170_burnout_current_na_tbl[AD4170_BURNOUT_MAX] = {
	[AD4170_BURNOUT_OFF] = 0,
	[AD4170_BURNOUT_100NA] = 100,
	[AD4170_BURNOUT_2000NA] = 2000,
	[AD4170_BURNOUT_10000NA] = 10000,
};

static const char * const ad4170_filter_modes_str[] = {
	[AD4170_FILT_SINC5_AVG] = "sinc5+avg",
	[AD4170_FILT_SINC5] = "sinc5",
	[AD4170_FILT_SINC3] = "sinc3",
};

struct ad4170_filter_config {
	enum ad4170_filter_type		filter_type;
	unsigned int			odr_div;
	unsigned int			fs_max;
	unsigned int			fs_min;
	unsigned int shift;
	enum iio_available_type		samp_freq_avail_type;
	int				samp_freq_avail_len;
	int				samp_freq_avail[3][2];
};

/*
 * For the moment this structure uses the internal 16MHz clk, needs to be
 * adapted to external clk source.
 */
#define AD4170_ODR_CONFIG(_filter_type, _odr_div, _fs_min, _fs_max, _shift)	\
{									\
		.filter_type = (_filter_type),				\
		.odr_div = (_odr_div),					\
		.fs_min = (_fs_min),					\
		.fs_max = (_fs_max),					\
		.shift = (_shift),					\
		.samp_freq_avail_type = IIO_AVAIL_RANGE,		\
		.samp_freq_avail_len = 3,				\
		.samp_freq_avail = {					\
			{ AD4170_INT_FREQ_16MHZ, (_odr_div) * (_fs_max) >> _shift},		\
			{ AD4170_INT_FREQ_16MHZ, (_odr_div) * (_fs_max / 2) >> _shift},		\
			{ AD4170_INT_FREQ_16MHZ, (_odr_div) * (_fs_min) >> _shift},		\
		},							\
}

static const struct ad4170_filter_config ad4170_filter_configs[] = {
	[AD4170_FILT_SINC5_AVG] = AD4170_ODR_CONFIG(AD4170_FILT_SINC5_AVG, 128, 4,  65532, 2),
	[AD4170_FILT_SINC5] = AD4170_ODR_CONFIG(AD4170_FILT_SINC5, 32, 1,  256, 0),
	[AD4170_FILT_SINC3] = AD4170_ODR_CONFIG(AD4170_FILT_SINC3, 32, 4,  65532, 0),

};

static int ad4170_get_reg_size(struct ad4170_state *st, unsigned int reg,
			       unsigned int *size)
{
	if (reg >= ARRAY_SIZE(ad4170_reg_size))
		return -EINVAL;

	if (!ad4170_reg_size[reg])
		return -EINVAL;

	*size = ad4170_reg_size[reg];

	return 0;
}

static int ad4170_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad4170_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int ad4170_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct ad4170_state *st = context;
	unsigned int size, addr;
	int ret;

	ret = ad4170_get_reg_size(st, reg, &size);
	if (ret)
		return ret;

	addr = reg + size - 1;
	put_unaligned_be16(addr, &st->reg_write_tx_buf[0]);

	switch (size) {
	case 4:
		put_unaligned_be32(val, &st->reg_write_tx_buf[2]);
		break;
	case 3:
		put_unaligned_be24(val, &st->reg_write_tx_buf[2]);
		break;
	case 2:
		put_unaligned_be16(val, &st->reg_write_tx_buf[2]);
		break;
	case 1:
		st->reg_write_tx_buf[2] = val;
		break;
	default:
		return -EINVAL;
	}

	return spi_write(st->spi, st->reg_write_tx_buf, size + 2);
}

static int ad4170_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct ad4170_state *st = context;
	struct spi_transfer t[] = {
		{
			.tx_buf = st->reg_read_tx_buf,
			.len = ARRAY_SIZE(st->reg_read_tx_buf),
		},
		{
			.rx_buf = st->reg_read_rx_buf,
		},
	};
	unsigned int size, addr;
	int ret;

	ret = ad4170_get_reg_size(st, reg, &size);
	if (ret)
		return ret;

	addr = reg + size - 1;
	put_unaligned_be16(AD4170_READ_MASK | addr, &st->reg_read_tx_buf[0]);

	t[1].len = size;

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (ret)
		return ret;

	switch (size) {
	case 4:
		*val = get_unaligned_be32(st->reg_read_rx_buf);
		break;
	case 3:
		*val = get_unaligned_be24(st->reg_read_rx_buf);
		break;
	case 2:
		*val = get_unaligned_be16(st->reg_read_rx_buf);
		break;
	case 1:
		*val = st->reg_read_rx_buf[0];
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct regmap_config ad4170_regmap_config = {
	.reg_bits = 14,
	.val_bits = 32,
	.reg_format_endian = REGMAP_ENDIAN_BIG,
	.val_format_endian = REGMAP_ENDIAN_BIG,
	.reg_read = ad4170_reg_read,
	.reg_write = ad4170_reg_write,
};

/* 8 possible setups (slots) (0-7)*/
static int ad4170_write_slot_setup(struct ad4170_state *st,
				   unsigned int slot,
				   struct ad4170_setup *setup)
{
	unsigned int val;
	int ret;

	val = FIELD_PREP(AD4170_ADC_SETUPS_MISC_CHOP_IEXC_MSK, setup->misc.chop_iexc) |
	      FIELD_PREP(AD4170_ADC_SETUPS_MISC_CHOP_ADC_MSK, setup->misc.chop_adc) |
	      FIELD_PREP(AD4170_ADC_SETUPS_MISC_BURNOUT_MSK, setup->misc.burnout);

	ret = regmap_write(st->regmap, AD4170_MISC_REG(slot), val);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4170_ADC_SETUPS_AFE_REF_BUF_M_MSK, setup->afe.ref_buf_m) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_REF_BUF_P_MSK, setup->afe.ref_buf_p) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_REF_SELECT_MSK, setup->afe.ref_select) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_BIPOLAR_MSK, setup->afe.bipolar) |
	      FIELD_PREP(AD4170_ADC_SETUPS_AFE_PGA_GAIN_MSK, setup->afe.pga_gain);

	ret = regmap_write(st->regmap, AD4170_AFE_REG(slot), val);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4170_ADC_SETUPS_POST_FILTER_SEL_MSK, setup->filter.post_filter_sel) |
	      FIELD_PREP(AD4170_ADC_SETUPS_FILTER_TYPE_MSK, setup->filter.filter_type);

	ret = regmap_write(st->regmap, AD4170_FILTER_REG(slot), val);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4170_FILTER_FS_REG(slot), setup->filter_fs);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4170_OFFSET_REG(slot), setup->offset);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4170_GAIN_REG(slot), setup->gain);
	if (ret)
		return ret;

	memcpy(&st->slots_info[slot].setup, setup, sizeof(*setup));

	return 0;
}

static int ad4170_write_channel_setup(struct ad4170_state *st,
				      unsigned int channel)
{
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	int slot;
	int ret;

	slot = channel;
	setup->afe.ref_buf_m = AD4170_REF_BUF_PRE;
	setup->afe.ref_buf_p = AD4170_REF_BUF_PRE;
	setup->filter.post_filter_sel = AD4170_POST_FILTER_NONE;
	setup->gain = AD4170_DEFAULT_ADC_GAIN_COEF;

	chan_info->slot = slot;

	ret = ad4170_write_slot_setup(st, slot, setup);
	if (ret)
		return ret;

	/* Hardcode default setup for channel x and write it */
	ret = regmap_update_bits(st->regmap, AD4170_CHANNEL_SETUP_REG(channel),
				 AD4170_CHANNEL_SETUPN_SETUP_N_MSK,
				 FIELD_PREP(AD4170_CHANNEL_SETUPN_SETUP_N_MSK, slot));
	if (ret)
		return ret;

	return 0;
}

static void ad4170_freq_to_fs(enum ad4170_filter_type filter_type,
			      int val, int val2, unsigned int *fs)
{
	const struct ad4170_filter_config *filter_config =
		&ad4170_filter_configs[filter_type];
	u64 dividend, divisor;
	int temp;

	dividend = (u64)AD4170_INT_FREQ_16MHZ * MICRO;
	divisor = filter_config->odr_div * ((u64)val * MICRO + val2);

	temp = DIV64_U64_ROUND_CLOSEST(dividend, divisor);
	temp <<= filter_config->shift;

	if (temp < filter_config->fs_min)
		temp = filter_config->fs_min;
	else if (temp > filter_config->fs_max)
		temp = filter_config->fs_max;

	*fs = temp;
}

static void ad4170_fs_to_freq(enum ad4170_filter_type filter_type,
			      unsigned int fs, int *val, int *val2)
{
	const struct ad4170_filter_config *filter_config =
		&ad4170_filter_configs[filter_type];
	unsigned int dividend, divisor;
	u64 temp;

	dividend = AD4170_INT_FREQ_16MHZ;
	divisor = (fs >> filter_config->shift) * filter_config->odr_div;

	temp = div_u64((u64)dividend, divisor);
	*val = div_u64_rem(temp, 1UL, val2);
}

static int ad4170_set_filter_type(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan,
				  unsigned int val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel = chan->scan_index;
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	enum ad4170_filter_type old_filter_type;
	int freq_val, freq_val2;
	unsigned int old_fs;
	int ret = 0;

	mutex_lock(&st->lock);
	if (setup->filter.filter_type == val)
		goto out;

	old_fs = setup->filter_fs;
	old_filter_type = setup->filter.filter_type;
	/*
	 * When switching between filter modes, try to match the ODR as
	 * close as possible. To do this, convert the current FS into ODR
	 * using the old filter mode, then convert it back into FS using
	 * the new filter mode.
	 */
	ad4170_fs_to_freq(setup->filter.filter_type, setup->filter_fs,
			  &freq_val, &freq_val2);

	ad4170_freq_to_fs(val, freq_val, freq_val2, &setup->filter_fs);

	setup->filter.filter_type = val;

	ret = ad4170_write_channel_setup(st, channel);
	if (ret) {
		setup->filter_fs = old_fs;
		setup->filter.filter_type = old_filter_type;
	}

 out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad4170_get_filter_type(struct iio_dev *indio_dev,
				  const struct iio_chan_spec *chan)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel = chan->scan_index;
	struct ad4170_chan_info *chan_info = &st->chan_info[chan->address];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	enum ad4170_filter_type filter_type;

	mutex_lock(&st->lock);
	filter_type = setup->filter.filter_type;
	mutex_unlock(&st->lock);

	return filter_type;
}

static const struct iio_enum ad4170_filter_type_enum = {
	.items = ad4170_filter_modes_str,
	.num_items = ARRAY_SIZE(ad4170_filter_modes_str),
	.set = ad4170_set_filter_type,
	.get = ad4170_get_filter_type,
};

static const struct iio_chan_spec_ext_info ad4170_filter_type_ext_info[] = {
	IIO_ENUM("filter_type", IIO_SEPARATE, &ad4170_filter_type_enum),
	IIO_ENUM_AVAILABLE("filter_type", IIO_SHARED_BY_TYPE, &ad4170_filter_type_enum),
	{ }
};

static const struct iio_chan_spec ad4170_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.differential = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
			      BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_OFFSET) |
			      BIT(IIO_CHAN_INFO_CALIBSCALE) |
			      BIT(IIO_CHAN_INFO_CALIBBIAS) |
			      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate_available = BIT(IIO_CHAN_INFO_SCALE) |
					BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.ext_info = ad4170_filter_type_ext_info,
	.scan_type = {
		.sign = 'u',
		.realbits = 24,
		.storagebits = 32,
		.endianness = IIO_LE,
	},
};

static int _ad4170_find_table_index(const unsigned int *tbl, size_t len,
				    unsigned int val)
{
	unsigned int i;

	for (i = 0; i < len; i++)
		if (tbl[i] == val)
			return i;

	return -EINVAL;
}

#define ad4170_find_table_index(table, val) \
	_ad4170_find_table_index(table, ARRAY_SIZE(table), val)

static int ad4170_set_mode(struct ad4170_state *st, enum ad4170_mode mode)
{
	return regmap_update_bits(st->regmap, AD4170_ADC_CTRL_REG,
				  AD4170_REG_CTRL_MODE_MSK,
				  FIELD_PREP(AD4170_REG_CTRL_MODE_MSK, mode));
}

static int ad4170_set_channel_enable(struct ad4170_state *st,
				     unsigned int channel, bool status)
{
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_slot_info *slot_info;
	int ret;

	if (chan_info->enabled == status)
		return 0;

	slot_info = &st->slots_info[chan_info->slot];

	ret = regmap_update_bits(st->regmap, AD4170_CHANNEL_EN_REG,
				 AD4170_CHANNEL_EN(channel),
				 status ? AD4170_CHANNEL_EN(channel) : 0);
	if (ret)
		return ret;

	slot_info->enabled_channels += status ? 1 : -1;
	chan_info->enabled = status;
	return 0;
}

static int _ad4170_read_sample(struct iio_dev *indio_dev, unsigned int channel,
			       int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	int ret;

	ret = ad4170_set_channel_enable(st, channel, true);
	if (ret)
		return ret;

	if (!st->spi_is_dma_mapped)
		reinit_completion(&st->completion);

	ret = ad4170_set_mode(st, AD4170_MODE_SINGLE);
	if (ret)
		return ret;

	if (!st->spi_is_dma_mapped) {
		ret = wait_for_completion_timeout(&st->completion, HZ);
		if (!ret)
			goto out;
	}

	//ret = regmap_read(st->regmap, AD4170_DATA_PER_CHANNEL_REG(channel), val);
	ret = regmap_read(st->regmap, AD4170_DATA_24b_REG, val);
	if (ret)
		return ret;

	if (setup->afe.bipolar)
		*val = sign_extend32(*val, ad4170_channel_template.scan_type.realbits - 1);
out:
	ret = ad4170_set_channel_enable(st, channel, false);
	if (ret)
		return ret;

	return IIO_VAL_INT;
}

static int ad4170_read_sample(struct iio_dev *indio_dev, unsigned int channel,
			      int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	ret = iio_device_claim_direct_mode(indio_dev);
	if (ret)
		return ret;

	mutex_lock(&st->lock);
	ret = _ad4170_read_sample(indio_dev, channel, val);
	mutex_unlock(&st->lock);

	iio_device_release_direct_mode(indio_dev);

	return ret;
}

static int ad4170_get_ref_voltage(struct ad4170_state *st,
				  enum ad4170_ref_select ref_sel)
{
	switch (ref_sel) {
	case AD4170_REFIN_REFIN1:
		return regulator_get_voltage(st->regulators[2].consumer);
	case AD4170_REFIN_REFIN2:
		return regulator_get_voltage(st->regulators[3].consumer);
	case AD4170_REFIN_AVDD:
		return regulator_get_voltage(st->regulators[0].consumer);
	case AD4170_REFIN_REFOUT:
		return AD4170_INT_REF_2_5V;
	default:
		return -EINVAL;
	}
}

static void ad4170_channel_scale(struct iio_dev *indio_dev,
				 struct iio_chan_spec const *chan,
				 int *val, int *val2)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct ad4170_chan_info *chan_info = &st->chan_info[chan->address];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	int ch_resolution = chan->scan_type.realbits - setup->afe.bipolar;
	int pga_gain;

	*val = ad4170_get_ref_voltage(st, setup->afe.ref_select) / MILLI;
	pga_gain = setup->afe.pga_gain & 0x7;
	if (setup->afe.pga_gain & 0x8) /* handle cases pga_gain = 8 and 9 */
		pga_gain--;

	*val2 = ch_resolution + pga_gain;
}

static int ad4170_get_offset(struct iio_dev *indio_dev, int addr, int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	ret = regmap_read(st->regmap, AD4170_OFFSET_REG(addr), val);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad4170_get_gain(struct iio_dev *indio_dev, int addr, int *val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	ret = regmap_read(st->regmap, AD4170_GAIN_REG(addr), val);
	if (ret < 0)
		return ret;

	return 0;
}

static int ad4170_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel = chan->scan_index;
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		return ad4170_read_sample(indio_dev, channel, val);
	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&st->lock);
		ad4170_channel_scale(indio_dev, chan, val, val2);
		mutex_unlock(&st->lock);
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		*val = 0;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		ad4170_fs_to_freq(setup->filter.filter_type, setup->filter_fs,
				  val, val2);
		mutex_unlock(&st->lock);

		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_CALIBBIAS:
		ad4170_get_offset(indio_dev, channel, val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		ad4170_get_gain(indio_dev, channel, val);
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static void ad4170_fill_scale_tbl(struct iio_dev *indio_dev, int channel)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	const struct iio_chan_spec *chan = &indio_dev->channels[channel];
	int ch_resolution = chan->scan_type.realbits - setup->afe.bipolar;
	unsigned int ref_select_uv;
	int pga;

	ref_select_uv = ad4170_get_ref_voltage(st, setup->afe.ref_select);

	for (pga = 0; pga < AD4170_PGA_GAIN_MAX; pga++) {
		u64 nv;
		unsigned int lshift, rshift;

		/* Keep ref in uV before right shigt to preserve scale precision */
		nv = (u64)ref_select_uv * NANO;
		lshift = (pga >> 3 & 1);  /* handle cases 8 and 9 */
		rshift = ch_resolution + (pga & 0x7) - lshift;
		chan_info->scale_tbl[pga][0] = 0;
		chan_info->scale_tbl[pga][1] = div_u64(nv >> rshift, MILLI);
	}
}

static int ad4170_read_avail(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     const int **vals, int *type, int *length,
			     long info)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel = chan->scan_index;
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	const struct ad4170_filter_config *filter_config;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		*vals = (int *)chan_info->scale_tbl;
		*length = ARRAY_SIZE(chan_info->scale_tbl) * 2;
		*type = IIO_VAL_INT_PLUS_NANO;

		return IIO_AVAIL_LIST;
	case IIO_CHAN_INFO_SAMP_FREQ:
		mutex_lock(&st->lock);
		filter_config = &ad4170_filter_configs[setup->filter.filter_type];
		mutex_unlock(&st->lock);

		*vals = (int *)filter_config->samp_freq_avail;
		*length = filter_config->samp_freq_avail_len;
		*type = IIO_VAL_FRACTIONAL;

		return filter_config->samp_freq_avail_type;
	default:
		return -EINVAL;
	}
}

static int ad4170_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long info)
{
	switch (info) {
	case IIO_CHAN_INFO_SCALE:
	case IIO_CHAN_INFO_SAMP_FREQ:
		return IIO_VAL_INT_PLUS_NANO;
	case IIO_CHAN_INFO_CALIBBIAS:
	case IIO_CHAN_INFO_CALIBSCALE:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad4170_set_channel_pga(struct iio_dev *indio_dev,
				  struct ad4170_state *st, unsigned int channel,
				  int val, int val2)
{
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	unsigned int pga, old_pga;
	int ret = 0;

	for (pga = 0; pga < AD4170_PGA_GAIN_MAX; pga++) {
		if (val == chan_info->scale_tbl[pga][0] &&
		    val2 == chan_info->scale_tbl[pga][1])
			break;
	}

	if (pga == AD4170_PGA_GAIN_MAX)
		return -EINVAL;

	mutex_lock(&st->lock);
	if (pga == setup->afe.pga_gain)
		goto out;

	old_pga = setup->afe.pga_gain;
	setup->afe.pga_gain = pga;

	ret = ad4170_write_channel_setup(st, channel);
	if (ret)
		setup->afe.pga_gain = old_pga;

out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad4170_set_channel_freq(struct ad4170_state *st,
				   unsigned int channel, int val, int val2)
{
	struct ad4170_chan_info *chan_info = &st->chan_info[channel];
	struct ad4170_setup *setup = &st->slots_info[chan_info->slot].setup;
	unsigned int fs, old_fs;
	int ret = 0;

	mutex_lock(&st->lock);
	old_fs = setup->filter_fs;

	ad4170_freq_to_fs(setup->filter.filter_type, val, val2, &fs);

	if (fs == setup->filter_fs)
		goto out;

	setup->filter_fs = fs;

	ret = ad4170_write_channel_setup(st, channel);
	if (ret)
		setup->filter_fs = old_fs;

out:
	mutex_unlock(&st->lock);
	return ret;
}

static int ad4170_set_gain(struct iio_dev *indio_dev, int addr, int val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	/*
	 * When writing to the GAIN registers, the ADC must be placed in
	 * standby mode or idle mode.
	 */
	ret = ad4170_set_mode(st, AD4170_MODE_IDLE);
	if (ret)
		return ret;

	val &= AD4170_GAIN_MSK;
	return regmap_write(st->regmap, AD4170_GAIN_REG(addr), val);
}

static int ad4170_set_offset(struct iio_dev *indio_dev, int addr, int val)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	/*
	 * When writing to the OFFSET_X registers, the ADC must be placed in
	 * standby mode or idle mode.
	 */
	ret = ad4170_set_mode(st, AD4170_MODE_IDLE);
	if (ret)
		return ret;

	val &= AD4170_OFFSET_MSK;
	return regmap_write(st->regmap, AD4170_OFFSET_REG(addr), val);
}

static int ad4170_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel = chan->scan_index;

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		return ad4170_set_channel_pga(indio_dev, st, channel, val, val2);
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4170_set_channel_freq(st, channel, val, val2);
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad4170_set_offset(indio_dev, channel, val);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4170_set_gain(indio_dev, channel, val);
	default:
		return -EINVAL;
	}
}

static int ad4170_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int channel;
	int ret;

	mutex_lock(&st->lock);

	for_each_set_bit(channel, scan_mask, indio_dev->num_channels) {
		ret = ad4170_set_channel_enable(st, channel, true);
		if (ret)
			goto out;
	}
out:
	mutex_unlock(&st->lock);

	return 0;
}

static const struct iio_info ad4170_info = {
	.read_raw = ad4170_read_raw,
	.read_avail = ad4170_read_avail,
	.write_raw = ad4170_write_raw,
	.write_raw_get_fmt = ad4170_write_raw_get_fmt,
	.update_scan_mode = ad4170_update_scan_mode,
	.debugfs_reg_access = ad4170_reg_access,
};

static int ad4170_soft_reset(struct ad4170_state *st)
{
	int ret;
	unsigned int reg = AD4170_INTERFACE_CONFIG_A_REG;

	ret = regmap_write(st->regmap, reg, AD4170_SW_RESET_MSK);
	if (ret)
		return ret;

	/*
	 * AD4170-4 requires a minimum of 1 ms between any reset event and a
	 * register read/write transaction.
	 */
	fsleep(AD4170_RESET_SLEEP_US);

	return 0;
}

static void ad4170_clk_disable_unprepare(void *clk)
{
	clk_disable_unprepare(clk);
}

static inline bool ad4170_valid_external_frequency(u32 freq)
{
	return (freq >= AD4170_EXT_FREQ_MHZ_MIN &&
		freq <= AD4170_EXT_FREQ_MHZ_MAX);
}

static int ad4170_of_clock_select(struct ad4170_state *st)
{
	struct device_node *np = st->spi->dev.of_node;
	unsigned int clocksel;

	clocksel = AD4170_INTERNAL_OSC;

	/* use internal clock */
	if (PTR_ERR(st->mclk) == -ENOENT) {
		if (of_property_read_bool(np, "adi,int-clock-output-enable"))
			clocksel = AD4170_INTERNAL_OSC_OUTPUT;
	} else {
		if (of_property_read_bool(np, "adi,clock-xtal"))
			clocksel = AD4170_EXTERNAL_XTAL;
		else
			clocksel = AD4170_EXTERNAL_OSC;
	}
	return clocksel;
}

static int ad4170_parse_digif_fw(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	int ret;

	/*
	 * Optional adi,dig-aux1 defaults to 0, DIG_AUX1 pin disabled.
	 */
	st->cfg.pin_muxing.dig_aux1_ctrl = AD4170_DIG_AUX1_DISABLED;
	ret = device_property_read_u8(dev, "adi,dig-aux1",
				      &st->cfg.pin_muxing.dig_aux1_ctrl);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to read adi,dig-aux1 property\n");

	if (st->cfg.pin_muxing.dig_aux1_ctrl < AD4170_DIG_AUX1_DISABLED ||
	    st->cfg.pin_muxing.dig_aux1_ctrl > AD4170_DIG_AUX1_SYNC)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid adi,dig-aux1 value: %u\n",
				     st->cfg.pin_muxing.dig_aux1_ctrl);

	/*
	 * Optional adi,dig-aux2 defaults to 0, DIG_AUX2 pin disabled.
	 */
	st->cfg.pin_muxing.dig_aux2_ctrl = AD4170_DIG_AUX2_DISABLED;
	ret = device_property_read_u8(dev, "adi,dig-aux2",
				      &st->cfg.pin_muxing.dig_aux2_ctrl);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to read adi,dig-aux2 property\n");

	if (st->cfg.pin_muxing.dig_aux2_ctrl < AD4170_DIG_AUX2_DISABLED ||
	    st->cfg.pin_muxing.dig_aux2_ctrl > AD4170_DIG_AUX2_SYNC)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid adi,dig-aux2 value: %u\n",
				     st->cfg.pin_muxing.dig_aux2_ctrl);

	/*
	 * Optional adi,sync-option defaults to 1, standard sync functionality.
	 */
	st->cfg.pin_muxing.sync_ctrl = AD4170_SYNC_STANDARD;
	ret = device_property_read_u8(dev, "adi,sync-option",
				      &st->cfg.pin_muxing.sync_ctrl);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Failed to read adi,sync-option property\n");

	if (st->cfg.pin_muxing.sync_ctrl < AD4170_SYNC_DISABLED ||
	    st->cfg.pin_muxing.sync_ctrl > AD4170_SYNC_ALTERNATE)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid adi,sync-option value: %u\n",
				     st->cfg.pin_muxing.sync_ctrl);

	return 0;
}

static int ad4170_parse_fw_setup(struct ad4170_state *st,
				 struct fwnode_handle *child,
				 struct ad4170_setup *setup)
{
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	tmp = 0;
	fwnode_property_read_u32(child, "adi,chop-adc", &tmp);
	setup->misc.chop_adc = tmp;

	st->chop_adc = tmp > st->chop_adc ? tmp : st->chop_adc;

	tmp = 0;
	fwnode_property_read_u32(child, "adi,burnout-current-nanoamp", &tmp);
	ret = ad4170_find_table_index(ad4170_burnout_current_na_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid burnout current %unA\n", tmp);
	setup->misc.burnout = ret;

	setup->afe.ref_buf_p = fwnode_property_read_bool(child, "adi,buffered-positive");
	setup->afe.ref_buf_m = fwnode_property_read_bool(child, "adi,buffered-negative");

	setup->afe.ref_select = AD4170_REFIN_REFOUT;
	fwnode_property_read_u32(child, "adi,reference-select",
				 &setup->afe.ref_select);
	if (setup->afe.ref_select >= AD4170_REFIN_MAX)
		return dev_err_probe(dev, -EINVAL,
				     "Invalid reference selected %u\n",
				     setup->afe.ref_select);

	ret = ad4170_get_ref_voltage(st, setup->afe.ref_select);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Cannot use reference %u\n",
				     setup->afe.ref_select);
	return 0;
}

static int ad4170_parse_fw_channel_type(struct device *dev,
					struct fwnode_handle *child,
					struct iio_chan_spec *chan)
{
	u32 pins[2];
	u32 iout;
	int ret;

	/*
	 * May be one of differential channel, single-ended channel, or current
	 * channel.
	 */
	ret = fwnode_property_read_u32_array(child, "diff-channels", pins,
					     ARRAY_SIZE(pins));
	if (!ret) {
		chan->differential = true;
		chan->channel = pins[0];
		chan->channel2 = pins[1];
		return 0;
	}
	ret = fwnode_property_read_u32(child, "single-channel", &pins[0]);
	if (!ret) {
		chan->differential = false;
		chan->channel = pins[0];

		ret = fwnode_property_read_u32(child, "common-mode-channel", &pins[1]);
		if (ret)
			return dev_err_probe(dev, ret,
				"common-mode-channel must be defined for single-ended channels.\n");
		chan->channel2 = pins[1];
		return 0;
	}
	ret = fwnode_property_read_u32(child, "adi,current-out", &iout);
	if (!ret) {
		chan->type = IIO_CURRENT;
		chan->differential = false;
		chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
					   BIT(IIO_CHAN_INFO_SCALE),
		chan->info_mask_separate_available = BIT(IIO_CHAN_INFO_RAW),
		chan->output = true;


		ret = fwnode_property_read_u32(child, "adi,current-out-pin", &pins[0]);
		if (ret)
			return dev_err_probe(dev, ret,
				"Must define adi,current-out-pin if channel has adi,current-out.\n");
		chan->channel = pins[0];
		return 0;
	}
	return dev_err_probe(dev, ret,
		"Channel must define one of diff-channels, single-channel, or adi,current-out.\n");
}

static int ad4170_parse_fw_channel(struct iio_dev *indio_dev,
				   struct iio_chan_spec *chan_array,
				   struct fwnode_handle *child)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	unsigned int index, setup_slot = 0;
	struct device *dev = &st->spi->dev;
	struct ad4170_chan_info *chan_info;
	struct ad4170_setup *setup;
	struct iio_chan_spec *chan;
	int ret;

	ret = fwnode_property_read_u32(child, "reg", &index);
	if (ret)
		return ret;

	if (index >= indio_dev->num_channels)
		return dev_err_probe(dev, -EINVAL,
				     "Channel idx greater than no of channels\n");

	chan = chan_array + index;
	chan_info = &st->chan_info[index];

	*chan = ad4170_channel_template;
	chan->address = index;
	chan->scan_index = index;

	chan_info->slot = AD4170_INVALID_SLOT;
	ret = fwnode_property_read_u32(child, "adi,config-setup-slot", &setup_slot);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Channel idx greater than no of channels\n");

	chan_info->slot = setup_slot;
	ret = ad4170_parse_fw_channel_type(dev, child, chan);
	if (ret < 0)
		return ret;

	setup = &st->slots_info[chan_info->slot].setup;
	setup->filter.filter_type = AD4170_FILT_SINC5_AVG;
	setup->filter_fs = 0x4;

	setup->afe.bipolar = fwnode_property_read_bool(child, "bipolar");
	if (setup->afe.bipolar)
		chan->scan_type.sign = 's';


	ret = ad4170_parse_fw_setup(st, child, setup);
	if (ret)
		return ret;

	return 0;
}

static int ad4170_parse_fw_children(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	struct fwnode_handle *child;
	struct ad4170_chan_info *chan_info;
	struct iio_chan_spec *chan_array;
	unsigned int num_channels;
	int ret;

	num_channels = device_get_child_node_count(dev);
	if (!num_channels)
		return dev_err_probe(&indio_dev->dev, -ENODEV,
				     "no channels defined\n");

	indio_dev->num_channels = num_channels;

	chan_array = devm_kcalloc(dev, num_channels, sizeof(*chan_array),
				  GFP_KERNEL);
	if (!chan_array)
		return -ENOMEM;

	chan_info = devm_kcalloc(dev, num_channels,
				 sizeof(*chan_info), GFP_KERNEL);
	if (!chan_info)
		return -ENOMEM;

	st->chan_info = chan_info;

	device_for_each_child_node(dev, child) {
		ret = ad4170_parse_fw_channel(indio_dev, chan_array, child);
		if (ret) {
			fwnode_handle_put(child);
			return ret;
		}
	}

	indio_dev->channels = chan_array;
	return 0;
}

static int ad4170_parse_fw(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	int ret, tmp;

	st->mclk = devm_clk_get(dev, "mclk");
	if (IS_ERR(st->mclk) && PTR_ERR(st->mclk) != -ENOENT)
		return dev_err_probe(dev, PTR_ERR(st->mclk),
				     "Failed to get mclk\n");

	st->cfg.clock_ctrl.clocksel = ad4170_of_clock_select(st);
	ret = ad4170_parse_digif_fw(indio_dev);
	if (ret < 0)
		return ret;

	st->pdsw0 = fwnode_property_read_bool(dev->fwnode, "adi,gpio0-power-down-switch");
	st->pdsw1 = fwnode_property_read_bool(dev->fwnode, "adi,gpio1-power-down-switch");

	ret = device_property_count_u32(dev, "adi,vbias-pins");
	if (ret > 0) {
		if (ret > AD4170_MAX_ANALOG_PINS)
			return dev_err_probe(dev, -EINVAL,
					     "Too many vbias pins %u\n", ret);

		st->num_vbias_pins = ret;

		ret = device_property_read_u32_array(dev, "adi,vbias-pins",
						     st->vbias_pins,
						     st->num_vbias_pins);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to read vbias pins\n");
	}

	st->cfg.current_source[0].i_out_pin = AD4170_I_OUT_AIN0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-pin-0",
				 &st->cfg.current_source[0].i_out_pin);

	st->cfg.current_source[1].i_out_pin = AD4170_I_OUT_AIN0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-pin-1",
				 &st->cfg.current_source[1].i_out_pin);

	st->cfg.current_source[2].i_out_pin = AD4170_I_OUT_AIN0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-pin-1",
				 &st->cfg.current_source[2].i_out_pin);

	st->cfg.current_source[3].i_out_pin = AD4170_I_OUT_AIN0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-pin-1",
				 &st->cfg.current_source[3].i_out_pin);

	tmp = 0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-current-0-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	st->cfg.current_source[0].i_out_val = ret;

	tmp = 0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-current-1-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	st->cfg.current_source[1].i_out_val = ret;
	tmp = 0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-current-2-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	st->cfg.current_source[2].i_out_val = ret;

	tmp = 0;
	fwnode_property_read_u32(dev->fwnode, "adi,excitation-current-3-microamp", &tmp);
	ret = ad4170_find_table_index(ad4170_iout_current_ua_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid excitation current %uuA\n", tmp);
	st->cfg.current_source[3].i_out_val = ret;

	ret = ad4170_parse_fw_children(indio_dev);
	if (ret)
		return ret;

	tmp = 0;
	fwnode_property_read_u32(dev->fwnode, "adi,chop-iexc", &tmp);
	ret = ad4170_find_table_index(ad4170_iexc_chop_tbl, tmp);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Invalid adi,chop-iexc config: %u\n", tmp);

	/* Set excitation current chop config to first channel setup config */
	st->slots_info[indio_dev->channels[0].address].setup.misc.chop_iexc = tmp;
	return 0;
}

static void ad4170_disable_regulators(void *data)
{
	struct ad4170_state *st = data;

	regulator_bulk_disable(ARRAY_SIZE(st->regulators), st->regulators);
}

static int ad4170_setup(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	struct device *dev = &st->spi->dev;
	unsigned int i, val;
	int ret;

	st->fclk = AD4170_INT_FREQ_16MHZ;
	if (st->cfg.clock_ctrl.clocksel == AD4170_EXTERNAL_OSC ||
	    st->cfg.clock_ctrl.clocksel == AD4170_EXTERNAL_XTAL) {
		ret = clk_prepare_enable(st->mclk);
		if (ret)
			return ret;

		st->fclk = clk_get_rate(st->mclk);
		if (!ad4170_valid_external_frequency(st->fclk)) {
			dev_warn(dev, "Invalid external clock frequency %u\n",
				 st->fclk);
			return -EINVAL;
		}
	}

	ret = devm_add_action_or_reset(dev, ad4170_clk_disable_unprepare,
				       st->mclk);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4170_PIN_MUXING_DIG_AUX1_CTRL_MSK,
			 st->cfg.pin_muxing.dig_aux1_ctrl);
	val |= FIELD_PREP(AD4170_PIN_MUXING_DIG_AUX2_CTRL_MSK,
			  st->cfg.pin_muxing.dig_aux2_ctrl);
	val |= FIELD_PREP(AD4170_PIN_MUXING_SYNC_CTRL_MSK,
			  st->cfg.pin_muxing.sync_ctrl);

	ret = regmap_write(st->regmap, AD4170_PIN_MUXING_REG, val);
	if (ret)
		return ret;

	val = FIELD_PREP(AD4170_POWER_DOWN_SW_PDSW0_MSK, st->pdsw0) |
	      FIELD_PREP(AD4170_POWER_DOWN_SW_PDSW1_MSK, st->pdsw1);

	ret = regmap_write(st->regmap, AD4170_POWER_DOWN_SW_REG, val);
	if (ret)
		return ret;

	/* Put ADC in IDLE mode */
	ret = ad4170_set_mode(st, AD4170_MODE_IDLE);
	if (ret)
		return ret;

	/* Setup channels. */
	for (i = 0; i < indio_dev->num_channels; i++) {
		struct iio_chan_spec const *chan = &indio_dev->channels[i];
		unsigned int val;

		ret = ad4170_write_channel_setup(st, i);
		if (ret)
			return ret;

		val = FIELD_PREP(AD4170_CHANNEL_MAPN_AINP_MSK, chan->channel) |
		      FIELD_PREP(AD4170_CHANNEL_MAPN_AINM_MSK, chan->channel2);

		ret = regmap_write(st->regmap, AD4170_CHANNEL_MAP_REG(i), val);
		if (ret)
			return ret;

		//ad4170_set_channel_freq(st, i, 9615, 0);
		ad4170_set_channel_freq(st, i, 125000, 0);
		ad4170_fill_scale_tbl(indio_dev, i);
	}

	val = 0;
	for (i = 0; i < st->num_vbias_pins; i++)
		val |= BIT(st->vbias_pins[i]);

	ret = regmap_write(st->regmap, AD4170_V_BIAS_REG, val);
	if (ret)
		return ret;

	for (i = 0; i < AD4170_NUM_CURRENT_SOURCE; i++) {
		val = FIELD_PREP(AD4170_CURRENT_SOURCE_I_OUT_PIN_MSK,
				 st->cfg.current_source[i].i_out_pin) |
		      FIELD_PREP(AD4170_CURRENT_SOURCE_I_OUT_VAL_MSK,
				 st->cfg.current_source[i].i_out_val);

		ret = regmap_write(st->regmap, AD4170_CURRENT_SOURCE_REG(i), val);
		if (ret)
			return ret;
	}

	ret = regmap_write(st->regmap, AD4170_CHANNEL_EN_REG, 0);
	if (ret)
		return ret;

	ret = regmap_update_bits(st->regmap, AD4170_ADC_CTRL_REG,
				 AD4170_ADC_CTRL_MULTI_DATA_REG_SEL_MSK,
				 AD4170_ADC_CTRL_MULTI_DATA_REG_SEL_MSK);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD4170_STATUS_REG, 0xffff);
	if (ret)
		return ret;

	return 0;
}

static const struct iio_trigger_ops ad4170_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static irqreturn_t ad4170_interrupt(int irq, void *dev_id)
{
	/* Top half of the interrupt, cannot sleep, should return asap*/
	struct iio_dev *indio_dev = dev_id;
	struct ad4170_state *st = iio_priv(indio_dev);

	/* Acknowledge the interrupt and call trig handler*/
	if (iio_buffer_enabled(indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->completion);

	return IRQ_HANDLED;
};

static void ad4170_prepare_message(struct ad4170_state *st)
{
	/* Read from AD4170_DATA_24b_REG top address which is 0x1E */
	st->tx_data[0] = (AD4170_READ_MASK | 0x1E) << 16;

	st->xfer.len = BITS_TO_BYTES(ad4170_channel_template.scan_type.storagebits);
	st->xfer.bits_per_word = 32;
	st->xfer.tx_buf = st->tx_data;
	st->xfer.rx_buf = st->rx_data;

	spi_message_init_with_transfers(&st->msg, &st->xfer, 1);
}

static int ad4170_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	ret = ad4170_set_mode(st, AD4170_MODE_CONT);
	if (ret)
		goto out;

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4170_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret, i;

	for (i = 0; i < indio_dev->num_channels; i++) {
		ret = ad4170_set_channel_enable(st, i, false);
		if (ret)
			goto out;
	}

	ret = ad4170_set_mode(st, AD4170_MODE_IDLE);
	if (ret)
		return ret;
out:
	return ret;
}

static const struct iio_buffer_setup_ops ad4170_buffer_ops = {
	.postenable = ad4170_buffer_postenable,
	.predisable = ad4170_buffer_predisable,
};

static int ad4170_hw_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	mutex_lock(&st->lock);

	ret = ad4170_set_mode(st, AD4170_MODE_CONT);
	if (ret)
		goto out;

	ret = spi_optimize_message(st->spi, &st->msg);
	if (ret < 0)
		goto out;

	spi_bus_lock(st->spi->master);
	ret = spi_engine_ex_offload_load_msg(st->spi, &st->msg);
	if (ret < 0)
		return ret;

	spi_engine_ex_offload_enable(st->spi, true);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4170_hw_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret, i;

	spi_engine_ex_offload_enable(st->spi, false);
	spi_bus_unlock(st->spi->master);
	spi_unoptimize_message(&st->msg);

	for (i = 0; i < indio_dev->num_channels; i++) {
		ret = ad4170_set_channel_enable(st, i, false);
		if (ret)
			goto out;
	}

	ret = ad4170_set_mode(st, AD4170_MODE_IDLE);
	if (ret)
		return ret;
out:
	return ret;
}

static const struct iio_buffer_setup_ops ad4170_hw_buffer_ops = {
	.postenable = ad4170_hw_buffer_postenable,
	.predisable = ad4170_hw_buffer_predisable,
};

static irqreturn_t ad4170_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret, i = 0;
	int scan_index;

	mutex_lock(&st->lock);
	for_each_set_bit(scan_index, indio_dev->active_scan_mask,
			 indio_dev->masklength) {
		/* Read register data */
		ret = regmap_read(st->regmap, AD4170_DATA_24b_REG, &st->data[i]);
		if (ret)
			goto out;
		i++;
	}

	iio_push_to_buffers_with_timestamp(indio_dev, &st->data,
					   iio_get_time_ns(indio_dev));
out:
	mutex_unlock(&st->lock);
	iio_trigger_notify_done(indio_dev->trig);
	return IRQ_HANDLED;
}

static int ad4170_triggered_buffer_alloc(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);
	int ret;

	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	st->trig = devm_iio_trigger_alloc(indio_dev->dev.parent, "%s-dev%d",
					  indio_dev->name, iio_device_id(indio_dev));
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ad4170_trigger_ops;
	st->trig->dev.parent = indio_dev->dev.parent;

	iio_trigger_set_drvdata(st->trig, indio_dev);
	ret = devm_iio_trigger_register(indio_dev->dev.parent, st->trig);
	if (ret)
		return ret;

	indio_dev->trig = iio_trigger_get(st->trig);

	init_completion(&st->completion);

	ret = request_irq(st->spi->irq,
			  &ad4170_interrupt,
			  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			  indio_dev->name, indio_dev);
	if (ret)
		return ret;

	return devm_iio_triggered_buffer_setup(indio_dev->dev.parent, indio_dev,
					       &iio_pollfunc_store_time,
					       &ad4170_trigger_handler,
					       &ad4170_buffer_ops);
}

static int ad4170_hardware_buffer_alloc(struct iio_dev *indio_dev)
{
	struct ad4170_state *st = iio_priv(indio_dev);

	ad4170_prepare_message(st);
	indio_dev->setup_ops = &ad4170_hw_buffer_ops;
	return devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent,
					       indio_dev, "rx",
					       IIO_BUFFER_DIRECTION_IN);
}

static int ad4170_input_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4170_state *st = gpiochip_get_data(chip);
	unsigned int mask;
	int ret;

	mutex_lock(&st->lock);
	mask = AD4170_GPIO_MODE_REG_MSK << 2 * offset;
	ret = regmap_update_bits(st->regmap, AD4170_GPIO_MODE_REG, mask,
				 (AD4170_GPIO_INPUT << 2 * offset));
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4170_output_gpio(struct gpio_chip *chip,
			      unsigned int offset, int value)
{
	struct ad4170_state *st = gpiochip_get_data(chip);
	unsigned int mask;
	int ret, val;

	mutex_lock(&st->lock);
	mask = AD4170_GPIO_MODE_REG_MSK << 2 * offset;
	ret = regmap_update_bits(st->regmap,
				 AD4170_GPIO_MODE_REG,
				 mask,
				 (AD4170_GPIO_OUTPUT << 2 * offset));
	if (ret < 0)
		goto out;
	ret = regmap_read(st->regmap, AD4170_GPIO_MODE_REG, &val);
	if (ret < 0)
		goto out;

	ret = regmap_update_bits(st->regmap,
				 AD4170_OUTPUT_DATA_REG,
				 BIT(offset),
				 (value << offset));
out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad4170_get_gpio(struct gpio_chip *chip, unsigned int offset)
{
	struct ad4170_state *st = gpiochip_get_data(chip);
	unsigned int val, mask;
	int ret;

	mutex_lock(&st->lock);
	ret = regmap_read(st->regmap, AD4170_GPIO_MODE_REG, &val);
	if (ret < 0)
		goto out;

	mask = AD4170_GPIO_MODE_REG_MSK << 2 * offset;
	switch (val & mask) {
	case AD4170_GPIO_INPUT:
		ret = regmap_read(st->regmap, AD4170_INPUT_DATA_REG, &val);
		break;
	case AD4170_GPIO_OUTPUT:
		ret = regmap_read(st->regmap, AD4170_OUTPUT_DATA_REG, &val);
		break;
	default:
		ret = -EINVAL;
	}

	if (ret < 0)
		goto out;

	ret = !!(val & BIT(offset));

out:
	mutex_unlock(&st->lock);

	return ret;
}

static void ad4170_set_gpio(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct ad4170_state *st = gpiochip_get_data(chip);
	unsigned int val, mask;
	int ret;

	mutex_lock(&st->lock);
	mask = AD4170_GPIO_MODE_REG_MSK << 2 * offset;
	ret = regmap_read(st->regmap, AD4170_GPIO_MODE_REG, &val);
	if (ret < 0)
		goto out;

	if (((val & mask) >> 2 * offset) == AD4170_GPIO_OUTPUT)
		regmap_update_bits(st->regmap, AD4170_OUTPUT_DATA_REG,
				   BIT(offset), (value << offset));
out:
	mutex_unlock(&st->lock);
}

static int ad4170_gpio_setup(struct ad4170_state *st)
{
	unsigned long valid_mask = 0x0;
	int i = 0;

	st->gpiochip.owner = THIS_MODULE;
	st->gpiochip.label = AD4170_NAME;
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = 4;
	st->gpiochip.parent = &st->spi->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.direction_input = ad4170_input_gpio;
	st->gpiochip.direction_output = ad4170_output_gpio;
	st->gpiochip.get = ad4170_get_gpio;
	st->gpiochip.set = ad4170_set_gpio;

	for (i = 0; i < 4; i++)
		__assign_bit(i, &valid_mask, true);

	if (st->pdsw0)
		__assign_bit(0, &valid_mask, false);

	if (st->pdsw1)
		__assign_bit(1,  &valid_mask, false);

	if (st->chop_adc == AD4170_CHOP_ACX_4PIN)
		valid_mask = 0x0;

	if (st->chop_adc == AD4170_CHOP_ACX_2PIN) {
		__assign_bit(2,  &valid_mask, false);
		__assign_bit(3,  &valid_mask, false);
	}
	st->gpiochip.valid_mask = &valid_mask;

	return devm_gpiochip_add_data(&st->spi->dev, &st->gpiochip, st);
}

static int ad4170_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad4170_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	mutex_init(&st->lock);
	st->spi_is_dma_mapped = spi_engine_ex_offload_supported(spi);
	st->spi = spi;

	indio_dev->name = AD4170_NAME;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &ad4170_info;

	st->regmap = devm_regmap_init(dev, NULL, st, &ad4170_regmap_config);

	st->regulators[0].supply = "avdd";
	st->regulators[1].supply = "iovdd";
	st->regulators[2].supply = "refin1";
	st->regulators[3].supply = "refin2";

	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(st->regulators),
				      st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulators\n");

	ret = regulator_bulk_enable(ARRAY_SIZE(st->regulators), st->regulators);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable regulators\n");

	ret = devm_add_action_or_reset(dev, ad4170_disable_regulators, st);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to add regulators disable action\n");

	ret = ad4170_soft_reset(st);
	if (ret)
		return ret;

	ret = ad4170_parse_fw(indio_dev);
	if (ret)
		return ret;

	ret = ad4170_setup(indio_dev);
	if (ret)
		return ret;

	ret = ad4170_gpio_setup(st);
	if (ret)
		return ret;

	if (st->spi_is_dma_mapped)
		ret = ad4170_hardware_buffer_alloc(indio_dev);
	else
		ret = ad4170_triggered_buffer_alloc(indio_dev);
	if (ret)
		return ret;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad4170_of_match[] = {
	{
		.compatible = "adi,ad4170",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, ad4170_of_match);

static struct spi_driver ad4170_driver = {
	.driver = {
		.name = AD4170_NAME,
		.of_match_table = ad4170_of_match,
	},
	.probe = ad4170_probe,
};
module_spi_driver(ad4170_driver);

MODULE_AUTHOR("Ana-Maria Cusco <ana-maria.cuso@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4170 SPI driver");
MODULE_LICENSE("GPL");
