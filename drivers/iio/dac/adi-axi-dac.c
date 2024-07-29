// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Generic AXI DAC IP core
 * Link: https://wiki.analog.com/resources/fpga/docs/axi_dac_ip
 *
 * Copyright 2016-2024 Analog Devices Inc.
 */
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/limits.h>
#include <linux/kstrtox.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include <linux/fpga/adi-axi-common.h>
#include <linux/iio/backend.h>
#include <linux/iio/buffer-dmaengine.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>

/*
 * Register definitions:
 *   https://wiki.analog.com/resources/fpga/docs/axi_dac_ip#register_map
 */

/* Base controls */
#define AXI_DAC_REG_CONFIG		0x0c
#define	   AXI_DDS_DISABLE		BIT(6)

 /* DAC controls */
#define AXI_DAC_REG_RSTN		0x0040
#define   AXI_DAC_RSTN_CE_N		BIT(2)
#define   AXI_DAC_RSTN_MMCM_RSTN	BIT(1)
#define   AXI_DAC_RSTN_RSTN		BIT(0)
#define AXI_DAC_REG_CNTRL_1		0x0044
#define   AXI_DAC_EXT_SYNC_ARM		BIT(1)
#define   AXI_DAC_EXT_SYNC_DISARM	BIT(2)
#define   AXI_DAC_SYNC			BIT(0)
#define AXI_DAC_REG_CNTRL_2		0x0048
#define   AXI_DAC_SDR_DDR_N		BIT(16)
#define   AXI_DAC_SYMB_8B		BIT(14)
#define	  ADI_DAC_R1_MODE		BIT(5)
#define   AXI_DAC_UNSIGNED_DATA		BIT(4)
#define AXI_DAC_REG_STATUS_1		0x54
#define AXI_DAC_REG_STATUS_2		0x58
#define AXI_DAC_DRP_STATUS		0x0074
#define   AXI_DAC_DRP_LOCKED		BIT(17)
#define AXI_DAC_CNTRL_DATA_RD		0x0080
#define   AXI_DAC_DATA_RD_8		GENMASK(7, 0)
#define   AXI_DAC_DATA_RD_16		GENMASK(15, 0)
#define AXI_DAC_CNTRL_DATA_WR		0x0084
#define   AXI_DAC_DATA_WR_8		GENMASK(23, 16)
#define   AXI_DAC_DATA_WR_16		GENMASK(23, 8)
#define AXI_DAC_UI_STATUS		0x0088
#define   AXI_DAC_BUSY			BIT(4)
#define AXI_DAC_REG_CUSTOM_CTRL		0x008C
#define   AXI_DAC_ADDRESS		GENMASK(31, 24)
#define   AXI_DAC_SYNCED_TRANSFER	BIT(2)
#define   AXI_DAC_STREAM		BIT(1)
#define   AXI_DAC_TRANSFER_DATA		BIT(0)

#define AXI_DAC_STREAM_ENABLE		(AXI_DAC_TRANSFER_DATA | AXI_DAC_STREAM)

/* DAC Channel controls */
#define AXI_DAC_REG_CHAN_CNTRL_1(c)	(0x0400 + (c) * 0x40)
#define AXI_DAC_REG_CHAN_CNTRL_3(c)	(0x0408 + (c) * 0x40)
#define   AXI_DAC_SCALE_SIGN		BIT(15)
#define   AXI_DAC_SCALE_INT		BIT(14)
#define   AXI_DAC_SCALE			GENMASK(14, 0)
#define AXI_DAC_REG_CHAN_CNTRL_2(c)	(0x0404 + (c) * 0x40)
#define AXI_DAC_REG_CHAN_CNTRL_4(c)	(0x040c + (c) * 0x40)
#define   AXI_DAC_PHASE			GENMASK(31, 16)
#define   AXI_DAC_FREQUENCY		GENMASK(15, 0)
#define AXI_DAC_REG_CHAN_CNTRL_7(c)	(0x0418 + (c) * 0x40)
#define   AXI_DAC_DATA_SEL		GENMASK(3, 0)

#define AXI_DAC_RD_ADDR(x)		(BIT(7) | (x))

/* 360 degrees in rad */
#define AXI_DAC_2_PI_MEGA		6283190

enum {
	AXI_DAC_DATA_INTERNAL_TONE,
	AXI_DAC_DATA_DMA = 2,
	AXI_DAC_DATA_INTERNAL_RAMP_16 = 11,
};

enum {
	AXI_DAC_BUS_TYPE_NONE,
	AXI_DAC_BUS_TYPE_QSPI,
};

struct axi_dac_state {
	struct regmap *regmap;
	struct device *dev;
	/*
	 * lock to protect multiple accesses to the device registers and global
	 * data/variables.
	 */
	struct mutex lock;
	u64 dac_clk;
	u32 reg_config;
	bool int_tone;
	int bus_type;
};

static int axi_dac_enable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);
	unsigned int __val;
	int ret;

	guard(mutex)(&st->lock);
	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_RSTN,
			      AXI_DAC_RSTN_MMCM_RSTN);
	if (ret)
		return ret;
	/*
	 * Make sure the DRP (Dynamic Reconfiguration Port) is locked. Not all
	 * designs really use it but if they don't we still get the lock bit
	 * set. So let's do it all the time so the code is generic.
	 */
	ret = regmap_read_poll_timeout(st->regmap, AXI_DAC_DRP_STATUS, __val,
				       __val & AXI_DAC_DRP_LOCKED, 100, 1000);
	if (ret)
		return ret;

	return regmap_set_bits(st->regmap, AXI_DAC_REG_RSTN,
			       AXI_DAC_RSTN_RSTN | AXI_DAC_RSTN_MMCM_RSTN);
}

static void axi_dac_disable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	guard(mutex)(&st->lock);
	regmap_write(st->regmap, AXI_DAC_REG_RSTN, 0);
}

static struct iio_buffer *axi_dac_request_buffer(struct iio_backend *back,
						 struct iio_dev *indio_dev)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);
	const char *dma_name;

	if (device_property_read_string(st->dev, "dma-names", &dma_name))
		dma_name = "tx";

	return iio_dmaengine_buffer_setup_ext(st->dev, indio_dev, dma_name,
					      IIO_BUFFER_DIRECTION_OUT);
}

static void axi_dac_free_buffer(struct iio_backend *back,
				struct iio_buffer *buffer)
{
	iio_dmaengine_buffer_free(buffer);
}

enum {
	AXI_DAC_FREQ_TONE_1,
	AXI_DAC_FREQ_TONE_2,
	AXI_DAC_SCALE_TONE_1,
	AXI_DAC_SCALE_TONE_2,
	AXI_DAC_PHASE_TONE_1,
	AXI_DAC_PHASE_TONE_2,
};

static int __axi_dac_frequency_get(struct axi_dac_state *st, unsigned int chan,
				   unsigned int tone_2, unsigned int *freq)
{
	u32 reg, raw;
	int ret;

	if (!st->dac_clk) {
		dev_err(st->dev, "Sampling rate is 0...\n");
		return -EINVAL;
	}

	if (tone_2)
		reg = AXI_DAC_REG_CHAN_CNTRL_4(chan);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_2(chan);

	ret = regmap_read(st->regmap, reg, &raw);
	if (ret)
		return ret;

	raw = FIELD_GET(AXI_DAC_FREQUENCY, raw);
	*freq = DIV_ROUND_CLOSEST_ULL(raw * st->dac_clk, BIT(16));

	return 0;
}

static int axi_dac_frequency_get(struct axi_dac_state *st,
				 const struct iio_chan_spec *chan, char *buf,
				 unsigned int tone_2)
{
	unsigned int freq;
	int ret;

	scoped_guard(mutex, &st->lock) {
		ret = __axi_dac_frequency_get(st, chan->channel, tone_2, &freq);
		if (ret)
			return ret;
	}

	return sysfs_emit(buf, "%u\n", freq);
}

static int axi_dac_scale_get(struct axi_dac_state *st,
			     const struct iio_chan_spec *chan, char *buf,
			     unsigned int tone_2)
{
	unsigned int scale, sign;
	int ret, vals[2];
	u32 reg, raw;

	if (tone_2)
		reg = AXI_DAC_REG_CHAN_CNTRL_3(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_1(chan->channel);

	ret = regmap_read(st->regmap, reg, &raw);
	if (ret)
		return ret;

	sign = FIELD_GET(AXI_DAC_SCALE_SIGN, raw);
	raw = FIELD_GET(AXI_DAC_SCALE, raw);
	scale = DIV_ROUND_CLOSEST_ULL((u64)raw * MEGA, AXI_DAC_SCALE_INT);

	vals[0] = scale / MEGA;
	vals[1] = scale % MEGA;

	if (sign) {
		vals[0] *= -1;
		if (!vals[0])
			vals[1] *= -1;
	}

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals),
				vals);
}

static int axi_dac_phase_get(struct axi_dac_state *st,
			     const struct iio_chan_spec *chan, char *buf,
			     unsigned int tone_2)
{
	u32 reg, raw, phase;
	int ret, vals[2];

	if (tone_2)
		reg = AXI_DAC_REG_CHAN_CNTRL_4(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_2(chan->channel);

	ret = regmap_read(st->regmap, reg, &raw);
	if (ret)
		return ret;

	raw = FIELD_GET(AXI_DAC_PHASE, raw);
	phase = DIV_ROUND_CLOSEST_ULL((u64)raw * AXI_DAC_2_PI_MEGA, U16_MAX);

	vals[0] = phase / MEGA;
	vals[1] = phase % MEGA;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals),
				vals);
}

static int __axi_dac_frequency_set(struct axi_dac_state *st, unsigned int chan,
				   u64 sample_rate, unsigned int freq,
				   unsigned int tone_2)
{
	u32 reg;
	u16 raw;
	int ret;

	if (!sample_rate || freq > sample_rate / 2) {
		dev_err(st->dev, "Invalid frequency(%u) dac_clk(%llu)\n",
			freq, sample_rate);
		return -EINVAL;
	}

	if (tone_2)
		reg = AXI_DAC_REG_CHAN_CNTRL_4(chan);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_2(chan);

	raw = DIV64_U64_ROUND_CLOSEST((u64)freq * BIT(16), sample_rate);

	ret = regmap_update_bits(st->regmap,  reg, AXI_DAC_FREQUENCY, raw);
	if (ret)
		return ret;

	/* synchronize channels */
	return regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_1, AXI_DAC_SYNC);
}

static int axi_dac_frequency_set(struct axi_dac_state *st,
				 const struct iio_chan_spec *chan,
				 const char *buf, size_t len, unsigned int tone_2)
{
	unsigned int freq;
	int ret;

	ret = kstrtou32(buf, 10, &freq);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);
	ret = __axi_dac_frequency_set(st, chan->channel, st->dac_clk, freq,
				      tone_2);
	if (ret)
		return ret;

	return len;
}

static int axi_dac_scale_set(struct axi_dac_state *st,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len, unsigned int tone_2)
{
	int integer, frac, scale;
	u32 raw = 0, reg;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &frac);
	if (ret)
		return ret;

	scale = integer * MEGA + frac;
	if (scale <= -2 * (int)MEGA || scale >= 2 * (int)MEGA)
		return -EINVAL;

	/*  format is 1.1.14 (sign, integer and fractional bits) */
	if (scale < 0) {
		raw = FIELD_PREP(AXI_DAC_SCALE_SIGN, 1);
		scale *= -1;
	}

	raw |= div_u64((u64)scale * AXI_DAC_SCALE_INT, MEGA);

	if (tone_2)
		reg = AXI_DAC_REG_CHAN_CNTRL_3(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_1(chan->channel);

	guard(mutex)(&st->lock);
	ret = regmap_write(st->regmap, reg, raw);
	if (ret)
		return ret;

	/* synchronize channels */
	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_1, AXI_DAC_SYNC);
	if (ret)
		return ret;

	return len;
}

static int axi_dac_phase_set(struct axi_dac_state *st,
			     const struct iio_chan_spec *chan,
			     const char *buf, size_t len, unsigned int tone_2)
{
	int integer, frac, phase;
	u32 raw, reg;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &integer, &frac);
	if (ret)
		return ret;

	phase = integer * MEGA + frac;
	if (phase < 0 || phase > AXI_DAC_2_PI_MEGA)
		return -EINVAL;

	raw = DIV_ROUND_CLOSEST_ULL((u64)phase * U16_MAX, AXI_DAC_2_PI_MEGA);

	if (tone_2)
		reg = AXI_DAC_REG_CHAN_CNTRL_4(chan->channel);
	else
		reg = AXI_DAC_REG_CHAN_CNTRL_2(chan->channel);

	guard(mutex)(&st->lock);
	ret = regmap_update_bits(st->regmap, reg, AXI_DAC_PHASE,
				 FIELD_PREP(AXI_DAC_PHASE, raw));
	if (ret)
		return ret;

	/* synchronize channels */
	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_1, AXI_DAC_SYNC);
	if (ret)
		return ret;

	return len;
}

static int axi_dac_ext_info_set(struct iio_backend *back, uintptr_t private,
				const struct iio_chan_spec *chan,
				const char *buf, size_t len)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (private) {
	case AXI_DAC_FREQ_TONE_1:
	case AXI_DAC_FREQ_TONE_2:
		return axi_dac_frequency_set(st, chan, buf, len,
					     private == AXI_DAC_FREQ_TONE_2);
	case AXI_DAC_SCALE_TONE_1:
	case AXI_DAC_SCALE_TONE_2:
		return axi_dac_scale_set(st, chan, buf, len,
					 private == AXI_DAC_SCALE_TONE_2);
	case AXI_DAC_PHASE_TONE_1:
	case AXI_DAC_PHASE_TONE_2:
		return axi_dac_phase_set(st, chan, buf, len,
					 private == AXI_DAC_PHASE_TONE_2);
	default:
		return -EOPNOTSUPP;
	}
}

static int axi_dac_ext_info_get(struct iio_backend *back, uintptr_t private,
				const struct iio_chan_spec *chan, char *buf)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (private) {
	case AXI_DAC_FREQ_TONE_1:
	case AXI_DAC_FREQ_TONE_2:
		return axi_dac_frequency_get(st, chan, buf,
					     private - AXI_DAC_FREQ_TONE_1);
	case AXI_DAC_SCALE_TONE_1:
	case AXI_DAC_SCALE_TONE_2:
		return axi_dac_scale_get(st, chan, buf,
					 private - AXI_DAC_SCALE_TONE_1);
	case AXI_DAC_PHASE_TONE_1:
	case AXI_DAC_PHASE_TONE_2:
		return axi_dac_phase_get(st, chan, buf,
					 private - AXI_DAC_PHASE_TONE_1);
	default:
		return -EOPNOTSUPP;
	}
}

static const struct iio_chan_spec_ext_info axi_dac_ext_info[] = {
	IIO_BACKEND_EX_INFO("frequency0", IIO_SEPARATE, AXI_DAC_FREQ_TONE_1),
	IIO_BACKEND_EX_INFO("frequency1", IIO_SEPARATE, AXI_DAC_FREQ_TONE_2),
	IIO_BACKEND_EX_INFO("scale0", IIO_SEPARATE, AXI_DAC_SCALE_TONE_1),
	IIO_BACKEND_EX_INFO("scale1", IIO_SEPARATE, AXI_DAC_SCALE_TONE_2),
	IIO_BACKEND_EX_INFO("phase0", IIO_SEPARATE, AXI_DAC_PHASE_TONE_1),
	IIO_BACKEND_EX_INFO("phase1", IIO_SEPARATE, AXI_DAC_PHASE_TONE_2),
	{}
};

static int axi_dac_extend_chan(struct iio_backend *back,
			       struct iio_chan_spec *chan)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	if (chan->type != IIO_ALTVOLTAGE)
		return -EINVAL;
	if (st->reg_config & AXI_DDS_DISABLE)
		/* nothing to extend */
		return 0;

	chan->ext_info = axi_dac_ext_info;

	return 0;
}

static int axi_dac_data_source_set(struct iio_backend *back, unsigned int chan,
				   enum iio_backend_data_source data)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (data) {
	case IIO_BACKEND_INTERNAL_CONTINUOUS_WAVE:
		return regmap_update_bits(st->regmap,
					  AXI_DAC_REG_CHAN_CNTRL_7(chan),
					  AXI_DAC_DATA_SEL,
					  AXI_DAC_DATA_INTERNAL_TONE);
	case IIO_BACKEND_EXTERNAL:
		return regmap_update_bits(st->regmap,
					  AXI_DAC_REG_CHAN_CNTRL_7(chan),
					  AXI_DAC_DATA_SEL,
					  AXI_DAC_DATA_DMA);
	case IIO_BACKEND_INTERNAL_RAMP_16:
		return regmap_update_bits(st->regmap,
					  AXI_DAC_REG_CHAN_CNTRL_7(chan),
					  AXI_DAC_DATA_SEL,
					  AXI_DAC_DATA_INTERNAL_RAMP_16);
	default:
		return -EINVAL;
	}
}

static int axi_dac_set_sample_rate(struct iio_backend *back, unsigned int chan,
				   u64 sample_rate)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);
	unsigned int freq;
	int ret, tone;

	if (!sample_rate)
		return -EINVAL;
	if (st->reg_config & AXI_DDS_DISABLE)
		/* sample_rate has no meaning if DDS is disabled */
		return 0;

	guard(mutex)(&st->lock);
	/*
	 * If dac_clk is 0 then this must be the first time we're being notified
	 * about the interface sample rate. Hence, just update our internal
	 * variable and bail... If it's not 0, then we get the current DDS
	 * frequency (for the old rate) and update the registers for the new
	 * sample rate.
	 */
	if (!st->dac_clk) {
		st->dac_clk = sample_rate;
		return 0;
	}

	for (tone = 0; tone <= AXI_DAC_FREQ_TONE_2; tone++) {
		ret = __axi_dac_frequency_get(st, chan, tone, &freq);
		if (ret)
			return ret;

		ret = __axi_dac_frequency_set(st, chan, sample_rate, tone, freq);
		if (ret)
			return ret;
	}

	st->dac_clk = sample_rate;

	return 0;
}

static int axi_dac_reg_access(struct iio_backend *back, unsigned int reg,
			      unsigned int writeval, unsigned int *readval)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static int axi_dac_ext_sync_enable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	return regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_1,
			       AXI_DAC_EXT_SYNC_ARM);
}

static int axi_dac_ext_sync_disable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	return regmap_clear_bits(st->regmap, AXI_DAC_REG_CNTRL_1,
				 AXI_DAC_EXT_SYNC_DISARM);
}

static int axi_dac_ddr_enable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	return regmap_clear_bits(st->regmap, AXI_DAC_REG_CNTRL_2,
				 AXI_DAC_SDR_DDR_N);
}

static int axi_dac_ddr_disable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	return regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_2,
			       AXI_DAC_SDR_DDR_N);
}

static int axi_dac_buffer_enable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	return regmap_set_bits(st->regmap, AXI_DAC_REG_CUSTOM_CTRL,
			       AXI_DAC_STREAM_ENABLE);
}

static int axi_dac_buffer_disable(struct iio_backend *back)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	return regmap_clear_bits(st->regmap, AXI_DAC_REG_CUSTOM_CTRL,
				 AXI_DAC_STREAM_ENABLE);
}

static int axi_dac_data_transfer_addr(struct iio_backend *back, u32 address)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	/*
	 * Sample register address, when the DAC is configured, or stream
	 * start address when the FSM is in stream state.
	 */
	return regmap_update_bits(st->regmap, AXI_DAC_REG_CUSTOM_CTRL,
				  AXI_DAC_ADDRESS,
				  FIELD_PREP(AXI_DAC_ADDRESS, address));
}

static int axi_dac_data_format_set(struct iio_backend *back, unsigned int ch,
				   const struct iio_backend_data_fmt *data)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	if (data->type == IIO_BACKEND_DATA_UNSIGNED)
		return regmap_clear_bits(st->regmap, AXI_DAC_REG_CNTRL_2,
					 AXI_DAC_UNSIGNED_DATA);

	return -EINVAL;
}

static int axi_dac_read_raw(struct iio_backend *back,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	switch (mask) {
	case IIO_CHAN_INFO_FREQUENCY:
		*val = clk_get_rate(devm_clk_get(st->dev, 0));

		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int axi_dac_bus_reg_write(struct iio_backend *back,
				 u32 reg, void *val, size_t size)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	if (!st->bus_type)
		return -EOPNOTSUPP;

	if (st->bus_type == AXI_DAC_BUS_TYPE_QSPI) {
		int ret;
		u32 ival;

		if (size != 1 && size != 2)
			return -EINVAL;

		switch (size) {
		case 1:
			ival = FIELD_PREP(AXI_DAC_DATA_WR_8, *(u8 *)val);
			break;
		case 2:
			ival =  FIELD_PREP(AXI_DAC_DATA_WR_16, *(u16 *)val);
			break;
		default:
			return  -EINVAL;
		}

		ret = regmap_write(st->regmap, AXI_DAC_CNTRL_DATA_WR, ival);
		if (ret)
			return ret;

		/*
		 * Both REG_CNTRL_2 and AXI_DAC_CNTRL_DATA_WR need to know
		 * the data size. So keeping data size control here only,
		 * since data size is mandatory for to the current transfer.
		 * DDR state handled separately by specific backend calls,
		 * generally all raw register writes are SDR.
		 */
		if (size == 1)
			ret = regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_2,
					      AXI_DAC_SYMB_8B);
		else
			ret = regmap_clear_bits(st->regmap, AXI_DAC_REG_CNTRL_2,
						AXI_DAC_SYMB_8B);
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, AXI_DAC_REG_CUSTOM_CTRL,
					 AXI_DAC_ADDRESS,
					 FIELD_PREP(AXI_DAC_ADDRESS, reg));
		if (ret)
			return ret;

		ret = regmap_update_bits(st->regmap, AXI_DAC_REG_CUSTOM_CTRL,
					 AXI_DAC_TRANSFER_DATA,
					 AXI_DAC_TRANSFER_DATA);
		if (ret)
			return ret;

		ret = regmap_read_poll_timeout(st->regmap,
					       AXI_DAC_REG_CUSTOM_CTRL, ival,
					       ival & AXI_DAC_TRANSFER_DATA,
					       10, 100 * KILO);
		if (ret)
			return ret;

		return regmap_clear_bits(st->regmap, AXI_DAC_REG_CUSTOM_CTRL,
					  AXI_DAC_TRANSFER_DATA);
	}

	return -EINVAL;
}

static int axi_dac_bus_reg_read(struct iio_backend *back,
				u32 reg, void *val, size_t size)
{
	struct axi_dac_state *st = iio_backend_get_priv(back);

	if (!st->bus_type)
		return -EOPNOTSUPP;

	if (st->bus_type == AXI_DAC_BUS_TYPE_QSPI) {
		int ret;
		u32 bval;

		if (size != 1 && size != 2)
			return -EINVAL;

		bval = 0;
		ret = axi_dac_bus_reg_write(back,
					    AXI_DAC_RD_ADDR(reg), &bval, size);
		if (ret)
			return ret;

		ret = regmap_read_poll_timeout(st->regmap, AXI_DAC_UI_STATUS,
					       bval, bval != AXI_DAC_BUSY,
					       10, 100);
		if (ret)
			return ret;

		return regmap_read(st->regmap, AXI_DAC_CNTRL_DATA_RD, val);
	}

	return -EINVAL;
}

static const struct iio_backend_ops axi_dac_generic_ops = {
	.enable = axi_dac_enable,
	.disable = axi_dac_disable,
	.read_raw = axi_dac_read_raw,
	.request_buffer = axi_dac_request_buffer,
	.free_buffer = axi_dac_free_buffer,
	.extend_chan_spec = axi_dac_extend_chan,
	.ext_info_set = axi_dac_ext_info_set,
	.ext_info_get = axi_dac_ext_info_get,
	.data_source_set = axi_dac_data_source_set,
	.set_sample_rate = axi_dac_set_sample_rate,
	.ext_sync_enable = axi_dac_ext_sync_enable,
	.ext_sync_disable = axi_dac_ext_sync_disable,
	.ddr_enable = axi_dac_ddr_enable,
	.ddr_disable = axi_dac_ddr_disable,
	.buffer_enable = axi_dac_buffer_enable,
	.buffer_disable = axi_dac_buffer_disable,
	.data_format_set = axi_dac_data_format_set,
	.data_transfer_addr = axi_dac_data_transfer_addr,
	.bus_reg_read = axi_dac_bus_reg_read,
	.bus_reg_write = axi_dac_bus_reg_write,
	.debugfs_reg_access = iio_backend_debugfs_ptr(axi_dac_reg_access),
};

static const struct iio_backend_info axi_dac_generic = {
	.name = "axi-dac",
	.ops = &axi_dac_generic_ops,
};

static const struct regmap_config axi_dac_regmap_config = {
	.val_bits = 32,
	.reg_bits = 32,
	.reg_stride = 4,
	.max_register = 0x0800,
};

static int axi_dac_probe(struct platform_device *pdev)
{
	const unsigned int *expected_ver;
	struct axi_dac_state *st;
	void __iomem *base;
	unsigned int ver;
	struct clk *clk;
	int ret;

	st = devm_kzalloc(&pdev->dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	expected_ver = device_get_match_data(&pdev->dev);
	if (!expected_ver)
		return -ENODEV;

	clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(clk),
				     "failed to get clock\n");

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	st->dev = &pdev->dev;
	st->regmap = devm_regmap_init_mmio(&pdev->dev, base,
					   &axi_dac_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(&pdev->dev, PTR_ERR(st->regmap),
				     "failed to init register map\n");

	device_property_read_u32(st->dev, "bus-type", &st->bus_type);

	/*
	 * Force disable the core. Up to the frontend to enable us. And we can
	 * still read/write registers...
	 */
	ret = regmap_write(st->regmap, AXI_DAC_REG_RSTN, 0);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, ADI_AXI_REG_VERSION, &ver);
	if (ret)
		return ret;

	if (ADI_AXI_PCORE_VER_MAJOR(ver) != ADI_AXI_PCORE_VER_MAJOR(*expected_ver)) {
		dev_err(&pdev->dev,
			"Major version mismatch. Expected %d.%.2d.%c, Reported %d.%.2d.%c\n",
			ADI_AXI_PCORE_VER_MAJOR(*expected_ver),
			ADI_AXI_PCORE_VER_MINOR(*expected_ver),
			ADI_AXI_PCORE_VER_PATCH(*expected_ver),
			ADI_AXI_PCORE_VER_MAJOR(ver),
			ADI_AXI_PCORE_VER_MINOR(ver),
			ADI_AXI_PCORE_VER_PATCH(ver));
		return -ENODEV;
	}

	/* Let's get the core read only configuration */
	ret = regmap_read(st->regmap, AXI_DAC_REG_CONFIG, &st->reg_config);
	if (ret)
		return ret;

	/*
	 * In some designs, setting the R1_MODE bit to 0 (which is the default
	 * value) causes all channels of the frontend to be routed to the same
	 * DMA (so they are sampled together). This is for things like
	 * Multiple-Input and Multiple-Output (MIMO). As most of the times we
	 * want independent channels let's override the core's default value and
	 * set the R1_MODE bit.
	 */
	ret = regmap_set_bits(st->regmap, AXI_DAC_REG_CNTRL_2, ADI_DAC_R1_MODE);
	if (ret)
		return ret;

	mutex_init(&st->lock);
	ret = devm_iio_backend_register(&pdev->dev, &axi_dac_generic, st);
	if (ret)
		return dev_err_probe(&pdev->dev, ret,
				     "failed to register iio backend\n");

	dev_info(&pdev->dev, "AXI DAC IP core (%d.%.2d.%c) probed\n",
		 ADI_AXI_PCORE_VER_MAJOR(ver),
		 ADI_AXI_PCORE_VER_MINOR(ver),
		 ADI_AXI_PCORE_VER_PATCH(ver));

	return 0;
}

static unsigned int axi_dac_9_1_b_info = ADI_AXI_PCORE_VER(9, 1, 'b');

static const struct of_device_id axi_dac_of_match[] = {
	{ .compatible = "adi,axi-dac-9.1.b", .data = &axi_dac_9_1_b_info },
	{}
};
MODULE_DEVICE_TABLE(of, axi_dac_of_match);

static struct platform_driver axi_dac_driver = {
	.driver = {
		.name = "adi-axi-dac",
		.of_match_table = axi_dac_of_match,
	},
	.probe = axi_dac_probe,
};
module_platform_driver(axi_dac_driver);

MODULE_AUTHOR("Nuno Sa <nuno.sa@analog.com>");
MODULE_DESCRIPTION("Analog Devices Generic AXI DAC IP core driver");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_DMAENGINE_BUFFER);
MODULE_IMPORT_NS(IIO_BACKEND);
