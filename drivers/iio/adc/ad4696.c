// SPDX-License-Identifier: GPL-2.0+
/*
 * AD4696 SPI ADC driver
 *
 * Copyright 2024 Analog Devices Inc.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/pwm.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-engine.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/units.h>

/* AD4696 used registers */
#define AD4696_REG_IF_CONFIG_A			0x000
#define AD4696_REG_IF_CONFIG_B			0x001
#define AD4696_REG_IF_CONFIG_C			0x010
#define AD4696_REG_SETUP			0x020
#define AD4696_REG_REF_CTRL			0x021
#define AD4696_REG_SEQ_CTRL			0x022
#define AD4696_REG_GP_MODE			0x027
#define AD4696_REG_TEMP_CTRL			0x029

#define AD4696_REG_OFFSET_IN(x)			(((x) << 1) | 0x0A0)
#define AD4696_REG_GAIN_IN(x)			(((x) << 1) | 0x0C0)

/* Set instuction mode command */
#define AD4696_INST_MODE_MASK			(0x01 << 7)
#define AD4696_SET_INST_MODE(x)			((x) << 7)

/* Software reset command */
#define AD4696_SW_RST_MASK			(0x01 << 7 | 0x01)
#define AD4696_SW_RST_CMD			(0x01 << 7 | 0x01)

/* AD4696_REG_SETUP */
#define AD4696_SETUP_IF_MODE_MASK		(0x01 << 2)
#define AD4696_SETUP_IF_MODE_CONV		(0x01 << 2)

/* AD4696_REG_REF_CTRL */
#define AD4696_REG_REF_VREF_SET_MASK		(0x07 << 2)
#define AD4696_REG_REF_VREF_SET(x)		(((x) & 0x07) << 2)

/* AD4696_REG_GP_MODE */
#define AD4696_GP_MODE_BUSY_GP_EN_MASK		(0x01 << 1)
#define AD4696_GP_MODE_BUSY_GP_EN(x)		(((x) & 0x01) << 1)
#define AD4696_GP_MODE_BUSY_GP_SEL_MASK		(0x01 << 5)
#define AD4696_GP_MODE_BUSY_GP_SEL(x)		(((x) & 0x01) << 5)

/* AD4696_REG_SEQ_CTRL */
#define AD4696_SEQ_CTRL_NUM_SLOTS_AS(x)		(((x) & 0x7f) << 0)

/* AD4696_REG_TEMP_CTRL */
#define AD4696_REG_TEMP_CTRL_TEMP_EN(x)		(((x) & 0x01) << 0)

/* AD4696_REG_AS_SLOT */
#define AD4696_REG_AS_SLOT(x)			(((x) & 0x7F) | 0x100)
#define AD4696_REG_AS_SLOT_INX(x)		(((x) & 0x0f) << 0)

/* AD4696_REG_IF_CONFIG_C */
#define AD4696_REG_IF_CONFIG_C_MB_STRICT_MASK	(0x01 << 5)
#define AD4696_REG_IF_CONFIG_C_MB_STRICT(x)	(((x) & 0x01) << 5)

/* AD4696_REG_CONFIG_INn */
#define AD4696_REG_CONFIG_IN(x)			(((x) & 0x0F) | 0x30)
#define AD4696_REG_CONFIG_IN_OSR_MASK		(0x03 << 0)
#define AD4696_REG_CONFIG_IN_OSR(x)		(((x) & 0x03) << 0)
#define AD4696_REG_CONFIG_IN_PAIR_MASK		(0x03 << 4)
#define AD4696_REG_CONFIG_IN_PAIR(x)		(((x) & 0x03) << 4)
#define AD4696_REG_CONFIG_IN_MODE_MASK		(0x01 << 6)
#define AD4696_REG_CONFIG_IN_MODE(x)		(((x) & 0x01) << 6)

enum ad4696_ids {
	ID_AD4695,
	ID_AD4696,
	ID_AD4697,
	ID_AD4698,
};

enum ad4696_instr_mode {
	AD4696_STREAM_INSTR_MODE,
	AD4696_SINGLE_INSTR_MODE,
};

enum ad4696_reg_access {
	AD4696_BYTE_ACCESS,
	AD4696_WORD_ACCESS,
};

enum ad4696_osr_ratios {
	AD4696_OSR_1,
	AD4696_OSR_4,
	AD4696_OSR_16,
	AD4696_OSR_64
};

enum ad4696_in_pair {
	AD4696_WITH_REFGND,
	AD4696_WITH_COM,
	AD4696_EVEN_ODD,
};

enum ad4696_busy_gp_sel {
	AD4696_BUSY_GP0 = 0,
	AD4696_BUSY_GP3 = 1,
};

enum ad4696_mode {
	AD4696_REG_CONFIG_MODE = 0,
	AD4696_CONV_MODE = 1,
	AD4696_AUTOCYCLE_MODE = 2,
};

struct ad4696_chip_info {
	const char	*name;
	enum ad4696_ids dev_id;
	int		max_sample_rate;
	u8		max_num_channels;
};

struct ad4696_channel_config {
	bool td_en;
	bool bipolar;
	unsigned int cfg_slot;
	enum ad4696_in_pair pair_select;
	bool highz_en;
	enum ad4696_osr_ratios osr_select;
};

struct ad4696_state {
	struct device *dev;
	struct clk *ref_clk;
	struct pwm_device *cnv;
	struct spi_device *spi;
	struct regulator *vref;
	struct ad4696_channel_config *channels_cfg;
	const struct ad4696_chip_info *chip_info;
	struct spi_message spi_msg;
	struct spi_transfer spi_transfer;
	enum ad4696_mode mode;
	unsigned long ref_clk_rate;
	unsigned int vref_mv;
	int sampling_freq;
	u8 num_channels;
	u8	data[3] ____cacheline_aligned;
	u32 conv_data[17] ____cacheline_aligned;
};

static int ad4696_set_sampling_freq(struct ad4696_state *st, int freq)
{
	int ret;
	struct pwm_state cnv_state;
	unsigned long long target, ref_clk_period_ps;

	if (freq > st->chip_info->max_sample_rate)
		return -EINVAL;

	target = DIV_ROUND_CLOSEST_ULL(st->ref_clk_rate, freq);
	ref_clk_period_ps = DIV_ROUND_CLOSEST_ULL(PICO, st->ref_clk_rate);
	cnv_state.period = ref_clk_period_ps * target;
	cnv_state.duty_cycle = cnv_state.period / 2;
	cnv_state.time_unit = PWM_UNIT_PSEC;
	cnv_state.enabled = true;

	ret = pwm_apply_state(st->cnv, &cnv_state);
	if (ret)
		return ret;

	st->sampling_freq = DIV_ROUND_CLOSEST_ULL(st->ref_clk_rate, target);

	return 0;
}

static void ad4696_disable_cnv(struct ad4696_state *st)
{
	pwm_disable(st->cnv);
}

static int ad4696_enable_cnv(struct ad4696_state *st)
{
	return ad4696_set_sampling_freq(st, st->sampling_freq);
}

static int ad4696_spi_reg_write(struct ad4696_state *st, u16 reg_addr,
				u8 reg_data)
{
	struct spi_transfer t = {0};

	if (st->mode != AD4696_REG_CONFIG_MODE)
		return -EBUSY;

	st->data[0] = ((reg_addr >> 8) & 0x7F);
	st->data[1] = 0xFF & reg_addr;
	st->data[2] = reg_data;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 3;
	t.bits_per_word = 8;

	return spi_sync_transfer(st->spi, &t, 1);
}

static int ad4696_spi_exit_conv(struct ad4696_state *st)
{
	struct spi_transfer t = {0};

	/* Write Register Configuration Mode Command */
	st->data[0] = 0x50;
	st->data[1] = 0x00;
	st->data[2] = 0x00;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 3;
	t.bits_per_word = 8;

	return spi_sync_transfer(st->spi, &t, 1);
}

static int ad4696_spi_reg_read(struct ad4696_state *st, u16 reg_addr,
			       u8 *reg_data)
{
	struct spi_transfer t = {0};
	int ret;

	if (st->mode != AD4696_REG_CONFIG_MODE)
		return -EBUSY;

	st->data[0] = (1 << 7) | ((reg_addr >> 8) & 0x7F);
	st->data[1] = 0xFF & reg_addr;
	st->data[2] = 0xFF;

	t.rx_buf = st->data;
	t.tx_buf = st->data;
	t.len = 3;
	t.bits_per_word = 8;

	ret = spi_sync_transfer(st->spi, &t, 1);
	if (ret)
		return ret;

	*reg_data = st->data[2];

	return 0;
}

static int ad4696_spi_write_mask(struct ad4696_state *st, u16 reg_addr,
				 u8 mask, u8 data)
{
	u8 reg_data;
	int ret;

	ret = ad4696_spi_reg_read(st, reg_addr, &reg_data);
	if (ret)
		return ret;

	reg_data &= ~mask;
	reg_data |= data;

	return ad4696_spi_reg_write(st, reg_addr, reg_data);
}

static int ad4696_set_reg_access_mode(struct ad4696_state *st,
				      enum ad4696_reg_access access)
{
	return ad4696_spi_write_mask(st, AD4696_REG_IF_CONFIG_C,
				     AD4696_REG_IF_CONFIG_C_MB_STRICT_MASK,
				     AD4696_REG_IF_CONFIG_C_MB_STRICT(access));
}

static int ad4696_update_channels(struct ad4696_state *st,
				  const unsigned long *mask, bool temp_en)
{
	int ret;
	int bit, slot_count = 0;

	/* handle temperature channel */
	ret = ad4696_spi_reg_write(st,
				   AD4696_REG_TEMP_CTRL,
				   AD4696_REG_TEMP_CTRL_TEMP_EN(temp_en));
	if (ret)
		return ret;

	for_each_set_bit(bit, mask, 32) {
		if (bit != (st->num_channels - 1)) {
			ret = ad4696_spi_reg_write(st,
						   AD4696_REG_AS_SLOT(slot_count),
						   AD4696_REG_AS_SLOT_INX(bit));
			if (ret)
				return ret;
			slot_count++;
		}
	}

	/* The number of slots is the value of NUM_SLOTS_AS register + 1 */
	slot_count--;

	return ad4696_spi_reg_write(st,
				   AD4696_REG_SEQ_CTRL,
				   AD4696_SEQ_CTRL_NUM_SLOTS_AS(slot_count));
}

static int ad4696_set_busy(struct ad4696_state *st,
			   enum ad4696_busy_gp_sel gp_sel)
{
	int ret;

	ret = ad4696_spi_write_mask(st,
				    AD4696_REG_GP_MODE,
				    AD4696_GP_MODE_BUSY_GP_EN_MASK,
				    AD4696_GP_MODE_BUSY_GP_EN(1));
	if (ret)
		return ret;

	return ad4696_spi_write_mask(st,
				     AD4696_REG_GP_MODE,
				     AD4696_GP_MODE_BUSY_GP_SEL_MASK,
				     AD4696_GP_MODE_BUSY_GP_SEL(gp_sel));
}

static int ad4696_enter_conversion_mode(struct ad4696_state *st)
{
	return ad4696_spi_write_mask(st,
				     AD4696_REG_SETUP,
				     AD4696_SETUP_IF_MODE_MASK,
				     AD4696_SETUP_IF_MODE_CONV);
}

static int ad4696_set_gain(struct ad4696_state *st, int chan_idx, u16 gain)
{
	int ret;

	if (chan_idx >= st->chip_info->max_num_channels)
		return -EINVAL;

	/* Update gain bits 7-0 */
	ret = ad4696_spi_reg_write(st,
				   AD4696_REG_GAIN_IN(chan_idx),
				   gain & 0xFF);
	if (ret)
		return ret;

	/* Update gain bits 15-8 */
	return ad4696_spi_reg_write(st,
				    AD4696_REG_GAIN_IN(chan_idx) + 1,
				    (gain >> 8) & 0xFF);
}

static int ad4696_get_gain(struct ad4696_state *st, int chan_idx, u16 *gain)
{
	int ret;
	u8 lsb, msb;

	if (chan_idx >= st->chip_info->max_num_channels)
		return -EINVAL;

	/* Read gain bits 7-0 */
	ret = ad4696_spi_reg_read(st,
				  AD4696_REG_GAIN_IN(chan_idx),
				  &lsb);
	if (ret)
		return ret;

	/* Read gain bits 15-8 */
	ret = ad4696_spi_reg_read(st,
				  AD4696_REG_GAIN_IN(chan_idx) + 1,
				  &msb);
	if (ret)
		return ret;

	*gain = msb << 8 | lsb;

	return 0;
}

static int ad4696_set_offset(struct ad4696_state *st, int chan_idx, u16 offset)
{
	int ret;

	if (chan_idx >= st->chip_info->max_num_channels)
		return -EINVAL;

	/* Update offset bits 7-0 */
	ret = ad4696_spi_reg_write(st,
				   AD4696_REG_OFFSET_IN(chan_idx),
				   offset & 0xFF);
	if (ret)
		return ret;

	/* Update offset bits 15-8 */
	return ad4696_spi_reg_write(st,
				    AD4696_REG_OFFSET_IN(chan_idx) + 1,
				    (offset >> 8) & 0xFF);
}

static int ad4696_get_offset(struct ad4696_state *st, int chan_idx, u16 *offset)
{
	int ret;
	u8 lsb, msb;

	if (chan_idx >= st->chip_info->max_num_channels)
		return -EINVAL;

	/* Get offset bits 7-0 */
	ret = ad4696_spi_reg_read(st,
				  AD4696_REG_OFFSET_IN(chan_idx),
				  &lsb);
	if (ret)
		return ret;

	/* Get offset bits 15-8 */
	ret = ad4696_spi_reg_read(st,
				  AD4696_REG_OFFSET_IN(chan_idx) + 1,
				  &msb);
	if (ret)
		return ret;

	*offset = msb << 8 | lsb;

	return 0;
}

static int ad4696_set_ref_voltage(struct ad4696_state *st, int ref_voltage)
{
	u8 reg;

	if (ref_voltage >= 2400 && ref_voltage <= 2750)
		reg = 0;
	else if (ref_voltage > 2750 && ref_voltage <= 3250)
		reg = 1;
	else if (ref_voltage > 3250 && ref_voltage <= 3750)
		reg = 2;
	else if (ref_voltage > 3750 && ref_voltage <= 4500)
		reg = 3;
	else if (ref_voltage > 4500 && ref_voltage <= 5100)
		reg = 4;
	else
		return -EINVAL;

	return ad4696_spi_write_mask(st, AD4696_REG_IF_CONFIG_A,
				     AD4696_REG_REF_VREF_SET_MASK,
				     AD4696_REG_REF_VREF_SET(reg));
}

static int ad4696_reset(struct ad4696_state *st)
{
	int ret;

	ret = ad4696_spi_write_mask(st, AD4696_REG_IF_CONFIG_A,
				    AD4696_SW_RST_MASK,
				    AD4696_SW_RST_CMD);
	if (ret)
		return ret;

	/* Wait 310 microseconds after issuing a software reset */
	usleep_range(310, 320);

	st->mode = AD4696_REG_CONFIG_MODE;
	return 0;
}

static int ad4696_set_instr_mode(struct ad4696_state *st,
				 enum ad4696_instr_mode mode)
{
	return  ad4696_spi_write_mask(st, AD4696_REG_IF_CONFIG_B,
				      AD4696_INST_MODE_MASK,
				      AD4696_SET_INST_MODE(mode));
}

static const struct iio_chan_spec ad4696_channel_template = {
	.type = IIO_VOLTAGE,
	.indexed = 1,
	.channel = 0,
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.info_mask_separate = BIT(IIO_CHAN_INFO_CALIBSCALE) |
			      BIT(IIO_CHAN_INFO_CALIBBIAS) |
			      BIT(IIO_CHAN_INFO_SCALE),
	.scan_type = {
		.sign = 'u',
		.storagebits = 32,
	},
};

static const struct iio_chan_spec ad4696_temp_channel_template = {
	.type = IIO_TEMP,
	.indexed = 1,
	.channel = 0,
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE) |
			      BIT(IIO_CHAN_INFO_OFFSET),
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SAMP_FREQ),
	.scan_type = {
		.sign = 'u',
		.realbits = 16,
		.storagebits = 32,
		.shift = 0,
	},
};

static int ad4696_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val,
			   int *val2,
			   long mask)
{
	struct ad4696_state *st = iio_priv(indio_dev);
	int ret;
	u16 read_val;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		*val = st->sampling_freq;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBSCALE:
		ret = ad4696_get_gain(st, chan->channel, &read_val);
		if (ret)
			return ret;
		*val = read_val;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_CALIBBIAS:
		ret = ad4696_get_offset(st, chan->channel, &read_val);
		if (ret)
			return ret;
		*val = sign_extend32(read_val, 15);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_TEMP) {
			*val = st->vref_mv * (-556);
			*val2 = chan->scan_type.realbits;
			return IIO_VAL_FRACTIONAL_LOG2;
		}

		*val = st->vref_mv;
		if (st->channels_cfg[chan->address].bipolar)
			*val2 = chan->scan_type.realbits - 1;
		else
			*val2 = chan->scan_type.realbits;
		return IIO_VAL_FRACTIONAL_LOG2;
	case IIO_CHAN_INFO_OFFSET:
		if (chan->type == IIO_TEMP) {
			*val = 402778;
			return IIO_VAL_INT;
		}
		fallthrough;
	default:
		return -EINVAL;
	}
}

static int ad4696_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val,
			    int val2,
			    long mask)
{
	struct ad4696_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		return ad4696_set_sampling_freq(st, val);
	case IIO_CHAN_INFO_CALIBSCALE:
		return ad4696_set_gain(st, chan->channel, val);
	case IIO_CHAN_INFO_CALIBBIAS:
		return ad4696_set_offset(st, chan->channel, (u16)val);
	default:
		return -EINVAL;
	}
}

static int ad4696_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad4696_state *st = iio_priv(indio_dev);
	int ret;
	u8 data;

	if (readval) {
		ret = ad4696_spi_reg_read(st, (u16)reg, &data);
		*readval = data;
	} else {
		data = writeval;
		ret = ad4696_spi_reg_write(st, (u16)reg, data);
	}

	return ret;
}

static int ad4696_update_scan_mode(struct iio_dev *indio_dev,
				   const unsigned long *scan_mask)
{
	bool temp_en;
	u8 chan_scan_nb;
	struct ad4696_state *st = iio_priv(indio_dev);

	/*
	 * At least 2 voltage channels have to be enabled, due to  NUM_SLOTS_AS
	 * minimum value in advanced sequencer.
	 */
	temp_en = *scan_mask & BIT(st->num_channels - 1);
	chan_scan_nb = hweight32(*scan_mask);
	if ((temp_en && chan_scan_nb < 3) || (!temp_en && chan_scan_nb < 2))
		return -EINVAL;

	return ad4696_update_channels(st, scan_mask, temp_en);
}

static const struct iio_info ad4696_info = {
	.read_raw = &ad4696_read_raw,
	.write_raw = &ad4696_write_raw,
	.update_scan_mode = &ad4696_update_scan_mode,
	.debugfs_reg_access = &ad4696_reg_access,
};

static void ad4696_regulator_disable(void *reg)
{
	regulator_disable(reg);
}

static void ad4696_clk_disable(void *data)
{
	clk_disable_unprepare(data);
}

static int ad4696_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad4696_state *st = iio_priv(indio_dev);
	int ret;

	ret = ad4696_enter_conversion_mode(st);

	st->mode = AD4696_CONV_MODE;

	memset(&st->spi_transfer, 0, sizeof(st->spi_transfer));
	st->spi_transfer.rx_buf = st->conv_data;
	st->spi_transfer.len = hweight32(*indio_dev->active_scan_mask);
	st->spi_transfer.bits_per_word = 32;

	spi_message_init_with_transfers(&st->spi_msg, &st->spi_transfer, 1);

	spi_bus_lock(st->spi->master);

	ret = spi_engine_offload_load_msg(st->spi, &st->spi_msg);
	if (ret)
		return ret;

	spi_engine_offload_enable(st->spi, true);

	ad4696_enable_cnv(st);

	return 0;
}

static int ad4696_buffer_predisable(struct iio_dev *indio_dev)
{
	int ret;
	struct ad4696_state *st = iio_priv(indio_dev);

	spi_engine_offload_enable(st->spi, false);

	ret = spi_bus_unlock(st->spi->master);
	if (ret)
		return ret;

	ad4696_disable_cnv(st);

	/* Wait for conversion time (415 ns) before sending the exit command. */
	udelay(1);
	ret =  ad4696_spi_exit_conv(st);
	if (ret)
		return ret;

	st->mode = AD4696_REG_CONFIG_MODE;

	return 0;
}

static const struct iio_buffer_setup_ops ad4696_buffer_setup_ops = {
	.postenable = &ad4696_buffer_postenable,
	.predisable = &ad4696_buffer_predisable,
};

static void ad4696_pwm_diasble(void *data)
{
	pwm_disable(data);
}

static int ad4696_write_chn_cfg(struct ad4696_state *st, struct ad4696_channel_config *cfg)
{
	int ret;

	ret = ad4696_spi_write_mask(st, AD4696_REG_CONFIG_IN(cfg->cfg_slot),
				    AD4696_REG_CONFIG_IN_OSR_MASK,
				    AD4696_REG_CONFIG_IN_OSR(cfg->osr_select));
	if (ret)
		return ret;

	ret = ad4696_spi_write_mask(st, AD4696_REG_CONFIG_IN(cfg->cfg_slot),
				    AD4696_REG_CONFIG_IN_MODE_MASK,
				    AD4696_REG_CONFIG_IN_MODE(cfg->bipolar));
	if (ret)
		return ret;

	return ad4696_spi_write_mask(st, AD4696_REG_CONFIG_IN(cfg->cfg_slot),
				     AD4696_REG_CONFIG_IN_PAIR_MASK,
				     AD4696_REG_CONFIG_IN_PAIR(cfg->pair_select));
}

static int ad4696_of_parse_channel_cfg(struct iio_dev *indio_dev,
				       struct device_node *np)
{
	struct ad4696_state *st = iio_priv(indio_dev);
	struct device_node *chan_node, *child;
	struct iio_chan_spec *iio_chan_arr;
	struct ad4696_channel_config *chan_cfg_arr;
	unsigned int channel, chan_idx = 0;
	int ret;

	// chan_node = of_get_child_by_name(np, "channels");
	// if (chan_node)
	st->num_channels = of_get_available_child_count(np);

	if (!st->num_channels) {
		dev_err(indio_dev->dev.parent, "no channel children\n");
		return -ENODEV;
	}

	/* Extra channel for temperature. */
	st->num_channels++;

	iio_chan_arr = devm_kcalloc(indio_dev->dev.parent, st->num_channels,
				    sizeof(*iio_chan_arr), GFP_KERNEL);
	if (!iio_chan_arr)
		return -ENOMEM;

	chan_cfg_arr = devm_kcalloc(indio_dev->dev.parent, st->num_channels,
				    sizeof(*chan_cfg_arr), GFP_KERNEL);
	if (!chan_cfg_arr)
		return -ENOMEM;

	indio_dev->channels = iio_chan_arr;
	indio_dev->num_channels = st->num_channels;
	st->channels_cfg = chan_cfg_arr;

	for_each_available_child_of_node(np, child) {
		struct ad4696_channel_config *chan_cfg = &st->channels_cfg[chan_idx];

		ret = of_property_read_u32(child, "reg", &channel);
		if (ret)
			goto err;

		if (channel >= st->chip_info->max_num_channels) {
			dev_err(indio_dev->dev.parent,
				"Channel id out of range, maximum %d channels allowed.\n",
				st->chip_info->max_num_channels);
			ret = -EINVAL;
			goto err;
		}

		u32 osr;

		ret = of_property_read_u32(child, "adi,osr", &osr);
		if (ret)
			osr = AD4696_OSR_1;
		chan_cfg->osr_select = osr;

		iio_chan_arr[chan_idx] = ad4696_channel_template;
		iio_chan_arr[chan_idx].address = channel;
		iio_chan_arr[chan_idx].scan_index = chan_idx;
		iio_chan_arr[chan_idx].channel = channel;
		iio_chan_arr[chan_idx].scan_type.realbits = 16 + chan_cfg->osr_select;
		iio_chan_arr[chan_idx].scan_type.shift = 16 - chan_cfg->osr_select;

		chan_cfg->bipolar = of_property_read_bool(child, "bipolar");
		if (chan_cfg->bipolar)
			iio_chan_arr[chan_idx].scan_type.sign = 's';

		u32 pair_select;

		ret = of_property_read_u32(child, "adi,pair-select", &pair_select);
		if (ret)
			goto err;
		chan_cfg->pair_select = pair_select;

		if (chan_cfg->bipolar && chan_cfg->pair_select == AD4696_WITH_REFGND) {
			dev_err(indio_dev->dev.parent,
				"Bipolar mode is not available for channels with the REGFND pin pairing assignment selected.\n");
			ret = -EINVAL;
			goto err;
		}

		if (chan_cfg->pair_select == AD4696_EVEN_ODD) {
			if (channel % 2 == 1) {
				dev_err(indio_dev->dev.parent,
					"Channels with EVEN_ODD pin pairing assignment selected must have an even index.\n");
				ret = -EINVAL;
				goto err;
			}
			iio_chan_arr[chan_idx].differential = 1;
			iio_chan_arr[chan_idx].channel2 = channel + 1;
		}

		chan_cfg->cfg_slot = channel;

		chan_idx++;

		ret = ad4696_write_chn_cfg(st, chan_cfg);
		if (ret)
			goto err;
	}

	/* Temperature channel. */
	iio_chan_arr[chan_idx] = ad4696_temp_channel_template;
	iio_chan_arr[chan_idx].address = chan_idx;
	iio_chan_arr[chan_idx].scan_index = chan_idx;
	iio_chan_arr[chan_idx].channel = 0;

	return 0;

err:
	of_node_put(chan_node);

	return ret;
}

static int ad4696_probe(struct spi_device *spi)
{
	struct ad4696_state *st;
	struct iio_dev *indio_dev;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;
	st->dev = &spi->dev;
	st->chip_info = device_get_match_data(&spi->dev);
	if (!st->chip_info) {
		st->chip_info = (const struct ad4696_chip_info *)spi_get_device_id(spi)->driver_data;
		if (!st->chip_info)
			return PTR_ERR(st->chip_info);
	}

	st->vref = devm_regulator_get(st->dev, "vref");
	if (IS_ERR(st->vref))
		return PTR_ERR(st->vref);

	ret = regulator_enable(st->vref);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(st->dev, ad4696_regulator_disable, st->vref);
	if (ret)
		return ret;

	st->ref_clk = devm_clk_get(st->dev, NULL);
	if (IS_ERR(st->ref_clk))
		return PTR_ERR(st->ref_clk);

	ret = clk_prepare_enable(st->ref_clk);
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(st->dev, ad4696_clk_disable, st->ref_clk);
	if (ret)
		return ret;

	st->ref_clk_rate = clk_get_rate(st->ref_clk);

	st->cnv = devm_pwm_get(st->dev, "cnv");
	if (IS_ERR(st->cnv))
		return PTR_ERR(st->cnv);

	ret = devm_add_action_or_reset(st->dev, ad4696_pwm_diasble, st->cnv);
	if (ret)
		return ret;

	indio_dev->name = st->chip_info->name;
	indio_dev->modes = INDIO_BUFFER_HARDWARE;
	indio_dev->info = &ad4696_info;
	indio_dev->setup_ops = &ad4696_buffer_setup_ops;

	ret = ad4696_reset(st);
	if (ret)
		return ret;

	ret = ad4696_set_instr_mode(st, AD4696_SINGLE_INSTR_MODE);
	if (ret)
		return ret;

	ret = ad4696_set_reg_access_mode(st, AD4696_BYTE_ACCESS);
	if (ret)
		return ret;

	ret = ad4696_of_parse_channel_cfg(indio_dev, spi->dev.of_node);
	if (ret)
		return ret;

	ret = ad4696_set_busy(st, AD4696_BUSY_GP0);
	if (ret)
		return ret;

	st->vref_mv = regulator_get_voltage(st->vref) / 1000;
	ad4696_set_ref_voltage(st, st->vref_mv);
	if (ret)
		return ret;

	ret = ad4696_set_sampling_freq(st, st->chip_info->max_sample_rate);
	if (ret)
		return ret;

	ret = devm_iio_dmaengine_buffer_setup(indio_dev->dev.parent, indio_dev,
					      "rx", IIO_BUFFER_DIRECTION_IN);
	if (ret)
		return ret;

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct ad4696_chip_info ad4696_chip_info[] = {
	[ID_AD4695] = {
		.name = "ad4695",
		.dev_id = ID_AD4695,
		.max_sample_rate = 500 * KILO,
		.max_num_channels = 16,
	},
	[ID_AD4696] = {
		.name = "ad4696",
		.max_sample_rate = 1000 * KILO,
		.dev_id = ID_AD4696,
		.max_num_channels = 16,
	},
	[ID_AD4697] = {
		.name = "ad4697",
		.max_sample_rate = 500 * KILO,
		.dev_id = ID_AD4697,
		.max_num_channels = 8,
	},
	[ID_AD4698] = {
		.name = "ad4698",
		.max_sample_rate = 1000 * KILO,
		.dev_id = ID_AD4698,
		.max_num_channels = 8,
	},
};

static const struct spi_device_id ad4696_id[] = {
	{"ad4695", (kernel_ulong_t)&ad4696_chip_info[ID_AD4695]},
	{"ad4696", (kernel_ulong_t)&ad4696_chip_info[ID_AD4696]},
	{"ad4697", (kernel_ulong_t)&ad4696_chip_info[ID_AD4697]},
	{"ad4698", (kernel_ulong_t)&ad4696_chip_info[ID_AD4698]},
	{ }
};
MODULE_DEVICE_TABLE(spi, ad4696_id);

static const struct of_device_id ad4696_of_match[] = {
	{
		.compatible = "adi,ad4695",
		.data = &ad4696_chip_info[ID_AD4695],
	},
	{
		.compatible = "adi,ad4696",
		.data = &ad4696_chip_info[ID_AD4696],
	},
	{
		.compatible = "adi,ad4697",
		.data = &ad4696_chip_info[ID_AD4697],
	},
	{
		.compatible = "adi,ad4698",
		.data = &ad4696_chip_info[ID_AD4698],
	},
	{ },
};
MODULE_DEVICE_TABLE(of, ad4696_of_match);

static struct spi_driver ad4696_driver = {
	.driver = {
		.name   = "ad4696",
		.of_match_table = ad4696_of_match,
	},
	.probe          = ad4696_probe,
	.id_table	= ad4696_id,
};
module_spi_driver(ad4696_driver);

MODULE_AUTHOR("Ramona Gradinariu <ramona.gradinariu@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD4696 ADC driver");
MODULE_LICENSE("GPL");
