// SPDX-License-Identifier: GPL-2.0
/*
 * ADAR1000 4-Channel, X Band and Ku Band Beamformer
 *
 * Copyright 2020 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/kernel.h>
#include <linux/firmware.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/debugfs.h>

#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADAR1000_INTERFACE_CFG_A	0x00
#define ADAR1000_INTERFACE_CFG_B	0x01
#define ADAR1000_DEV_CFG		0x02
#define ADAR1000_CHIP_TYPE		0x03
#define ADAR1000_PRODUCT_ID_H		0x04
#define ADAR1000_PRODUCT_ID_L		0x05
#define ADAR1000_SCRATCH_PAD		0x0A
#define ADAR1000_SPI_REV		0x0B
#define ADAR1000_VENDOR_ID_H		0x0C
#define ADAR1000_VENDOR_ID_L		0x0D
#define ADAR1000_TRANSFER_REG		0x0F
#define ADAR1000_CH_RX_GAIN(x)		(0x10 + (x))
#define ADAR1000_CH_RX_PHASE_I(x)	(0x14 + 2 * (x))
#define ADAR1000_CH_RX_PHASE_Q(x)	(0x15 + 2 * (x))
#define ADAR1000_CH_TX_GAIN(x)		(0x1C + (x))
#define ADAR1000_CH_TX_PHASE_I(x)	(0x20 + 2 * (x))
#define ADAR1000_CH_TX_PHASE_Q(x)	(0x21 + 2 * (x))
#define ADAR1000_LD_WRK_REGS		0x28
#define ADAR1000_CH_PA_BIAS_ON(x)	(0x29 + (x))
#define ADAR1000_LNA_BIAS_ON		0x2D
#define ADAR1000_RX_ENABLES		0x2E
#define ADAR1000_TX_ENABLES		0x2F
#define ADAR1000_MISC_ENABLES		0x30
#define ADAR1000_SW_CTRL		0x31
#define ADAR1000_ADC_CTRL		0x32
#define ADAR1000_ADC_OUTPUT		0x33
#define ADAR1000_BIAS_CURRENT_RX_LNA	0x34
#define ADAR1000_BIAS_CURRENT_RX	0x35
#define ADAR1000_BIAS_CURRENT_TX	0x36
#define ADAR1000_BIAS_CURRENT_TX_DRV	0x37
#define ADAR1000_MEM_CTRL		0x38
#define ADAR1000_RX_CHX_MEM		0x39
#define ADAR1000_TX_CHX_MEM		0x3A
#define ADAR1000_RX_CH_MEM(x)		(0x3D + (x))
#define ADAR1000_TX_CH_MEM(x)		(0x41 + (x))
#define ADAR1000_REV_ID			0x45
#define ADAR1000_CH_PA_BIAS_OFF(x)	(0x46 + (x))
#define ADAR1000_LNA_BIAS_OFF		0x4A
#define ADAR1000_TX_TO_RX_DELAY_CTRL	0x4B
#define ADAR1000_RX_TO_TX_DELAY_CTRL	0x4C
#define ADAR1000_TX_BEAM_STEP_START	0x4D
#define ADAR1000_TX_BEAM_STEP_STOP	0x4E
#define ADAR1000_RX_BEAM_STEP_START	0x4F
#define ADAR1000_RX_BEAM_STEP_STOP	0x50
#define ADAR1000_RX_BIAS_RAM_CTL	0x51
#define ADAR1000_TX_BIAS_RAM_CTL	0x52
#define ADAR1000_LDO_TRIM_CTL_0		0x400
#define ADAR1000_LDO_TRIM_CTL_1		0x401

/* ADAR1000_INTERFACE_CFG_A */
#define ADAR1000_SOFTRESET		BIT(7)
#define ADAR1000_LSB_FIRST		BIT(6)
#define ADAR1000_ADDR_ASCN		BIT(5)
#define ADAR1000_SDOACTIVE		BIT(4)

/* ADAR1000_ADC_CTRL */
#define ADAR1000_ADC_CLKFREQ_SEL	BIT(7)
#define ADAR1000_AC_EN			BIT(6)
#define ADAR1000_CLK_EN			BIT(5)
#define ADAR1000_ST_CONV		BIT(4)
#define ADAR1000_MUX_SEL_MSK		GENMASK(3, 1)
#define ADAR1000_MUX_SEL(x)		FIELD_PREP(ADAR1000_MUX_SEL_MSK, x)
#define ADAR1000_ADC_EOC		BIT(0)

/* ADAR1000_CH_GAIN */
#define ADAR1000_CH_ATTN		BIT(7)
#define ADAR1000_VGA_GAIN_MSK		GENMASK(6, 0)
#define ADAR1000_VGA_GAIN(x)		FIELD_PREP(ADAR1000_VGA_GAIN_MSK, x)

/* ADAR1000_CH polarity and phase for I and Q */
#define ADAR1000_VM_POL			BIT(5)
#define ADAR1000_VM_GAIN_MSK		GENMASK(4, 0)
#define ADAR1000_VM_GAIN(x)		FIELD_PREP(ADAR1000_VM_GAIN_MSK, x)

/* ADAR1000 LDO trim select */
#define ADAR1000_LDO_TRIM_SEL_MSK	GENMASK(1, 0)
#define ADAR1000_LDO_TRIM_SEL(x)	FIELD_PREP(ADAR1000_LDO_TRIM_SEL_MSK, x)

/* ADAR1000_MEM_CTRL */
#define ADAR1000_SCAN_MODE_EN		BIT(7)
#define ADAR1000_BEAM_RAM_BYPASS	BIT(6)
#define ADAR1000_BIAS_RAM_BYPASS	BIT(5)
#define ADAR1000_TX_BEAM_STEP_EN	BIT(3)
#define ADAR1000_RX_BEAM_STEP_EN	BIT(2)
#define ADAR1000_TX_CHX_RAM_BYPASS	BIT(1)
#define ADAR1000_RX_CHX_RAM_BYPASS	BIT(0)

/* ADAR1000_SW_CTRL */
#define ADAR1000_SW_DRV_TR_STATE	BIT(7)
#define ADAR1000_TX_EN			BIT(6)
#define ADAR1000_RX_EN			BIT(5)
#define ADAR1000_SW_DRV_EN_TR		BIT(4)
#define ADAR1000_SW_DRV_EN_POL		BIT(3)
#define ADAR1000_TR_SOURCE		BIT(2)
#define ADAR1000_TR_SPI			BIT(1)
#define ADAR1000_POL			BIT(0)

/* RX_ENABLES/TX_ENABLES */
#define ADAR1000_CH1_EN			BIT(6)
#define ADAR1000_CH2_EN			BIT(5)
#define ADAR1000_CH3_EN			BIT(4)
#define ADAR1000_CH4_EN			BIT(3)
#define ADAR1000_LNA_EN			BIT(2)
#define ADAR1000_VM_EN			BIT(1)
#define ADAR1000_VGA_EN			BIT(0)

/* ADAR1000_LD_WRK_REGS */
#define ADAR1000_LDTX_OVERRIDE		BIT(1)
#define ADAR1000_LDRX_OVERRIDE		BIT(0)

#define ADAR1000_SPI_ADDR_MSK		GENMASK(14, 13)
#define ADAR1000_SPI_ADDR(x)		FIELD_PREP(ADAR1000_SPI_ADDR_MSK, x)

/* SPI write all */
#define ADAR1000_SPI_WR_ALL		0x800

#define ADAR1000_MAX_DEV		4

struct adar1000_phase {
	u32 val;
	u32 val2;
	u8 vm_gain_i;
	u8 vm_gain_q;
};

struct adar1000_state {
	struct spi_device	*spi;
	struct regmap		*regmap;
	u16			dev_addr;

	int			tx_phase[4][4];
	int			rx_phase[4][4];
	struct adar1000_phase	*pt_info;
	unsigned int		pt_size;
	struct bin_attribute	bin_pt;
	char			*bin_attr_buf;

};

static const struct regmap_config adar1000_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.read_flag_mask = BIT(7),
};

/* Phase values Table 13, 14, 15, 16 page 34 of the datasheet - these values
 * keep the vector modulator constant.
 */
static const struct adar1000_phase adar1000_phase_values[] = {
	{0, 0, 0x3f, 0x20}, {2, 812, 0x3f, 0x21}, {5, 625, 0x3f, 0x23},
	{8, 437, 0x3F, 0x24}, {11, 250, 0x3F, 0x26}, {14, 62, 0x3E, 0x27},
	{16, 875, 0x3E, 0x28}, {19, 687, 0x3D, 0x2A}, {22, 500, 0x3D, 0x2B},
	{25, 312, 0x3C, 0x2D}, {28, 125, 0x3C, 0x2E}, {30, 937, 0x3B, 0x2F},
	{33, 750, 0x3A, 0x30}, {36, 562, 0x39, 0x31}, {39, 375, 0x38, 0x33},
	{42, 187, 0x37, 0x34}, {45, 0, 0x36, 0x35}, {47, 812, 0x35, 0x36},
	{50, 625, 0x34, 0x37}, {53, 437, 0x33, 0x38}, {56, 250, 0x32, 0x38},
	{59, 62, 0x30, 0x39}, {61, 875, 0x2F, 0x3}, {64, 687, 0x2E, 0x3A},
	{67, 500, 0x2C, 0x3B}, {70, 312, 0x2B, 0x3C}, {73, 125, 0x2A, 0x3C},
	{75, 937, 0x28, 0x3C}, {78, 750, 0x27, 0x3D}, {81, 562, 0x25, 0x3D},
	{84, 375, 0x24, 0x3D}, {87, 187, 0x22, 0x3D}, {90, 0, 0x21, 0x3D},
	{92, 812, 0x01, 0x3D}, {95, 625, 0x03, 0x3D}, {98, 437, 0x04, 0x3D},
	{101, 250, 0x06, 0x3D}, {104, 62, 0x07, 0x3C}, {106, 875, 0x08, 0x3C},
	{109, 687, 0x0A, 0x3C}, {112, 500, 0x0B, 0x3B}, {115, 312, 0x0D, 0x3A},
	{118, 125, 0x0E, 0x3A}, {120, 937, 0x0F, 0x39}, {123, 750, 0x11, 0x38},
	{126, 562, 0x12, 0x38}, {129, 375, 0x13, 0x37}, {132, 187, 0x14, 0x36},
	{135, 0, 0x16, 0x35}, {137, 812, 0x17, 0x34}, {140, 625, 0x18, 0x33},
	{143, 437, 0x19, 0x31}, {146, 250, 0x19, 0x30}, {149, 62, 0x1A, 0x2F},
	{151, 875, 0x1B, 0x2E}, {154, 687, 0x1C, 0x2D}, {157, 500, 0x1C, 0x2B},
	{160, 312, 0x1D, 0x2A}, {163, 125, 0x1E, 0x28}, {165, 937, 0x1E, 0x27},
	{168, 750, 0x1E, 0x26}, {171, 562, 0x1F, 0x24}, {174, 375, 0x1F, 0x23},
	{177, 187, 0x1F, 0x21}, {180, 0, 0x1F, 0x20}, {182, 812, 0x1F, 0x01},
	{185, 625, 0x1F, 0x03}, {188, 437, 0x1F, 0x04}, {191, 250, 0x1F, 0x06},
	{194, 62, 0x1E, 0x07}, {196, 875, 0x1E, 0x08}, {199, 687, 0x1D, 0x0A},
	{202, 500, 0x1D, 0x0B}, {205, 312, 0x1C, 0x0D}, {208, 125, 0x1C, 0x0E},
	{210, 937, 0x1B, 0x0F}, {213, 750, 0x1A, 0x10}, {216, 562, 0x19, 0x11},
	{219, 375, 0x18, 0x13}, {222, 187, 0x17, 0x14}, {225, 0, 0x16, 0x15},
	{227, 812, 0x15, 0x16}, {230, 625, 0x14, 0x17}, {233, 437, 0x13, 0x18},
	{236, 250, 0x12, 0x18}, {239, 62, 0x10, 0x19}, {241, 875, 0x0F, 0x1A},
	{244, 687, 0x0E, 0x1A}, {247, 500, 0x0C, 0x1B}, {250, 312, 0x0B, 0x1C},
	{253, 125, 0x0A, 0x1C}, {255, 937, 0x08, 0x1C}, {258, 750, 0x07, 0x1D},
	{261, 562, 0x05, 0x1D}, {264, 375, 0x04, 0x1D}, {267, 187, 0x02, 0x1D},
	{270, 0, 0x01, 0x1D}, {272, 812, 0x21, 0x1D}, {275, 625, 0x23, 0x1D},
	{278, 437, 0x24, 0x1D}, {281, 250, 0x26, 0x1D}, {284, 62, 0x27, 0x1C},
	{286, 875, 0x28, 0x1C}, {289, 687, 0x2A, 0x1C}, {292, 500, 0x2B, 0x1B},
	{295, 312, 0x2D, 0x1A}, {298, 125, 0x2E, 0x1A}, {300, 937, 0x2F, 0x19},
	{303, 750, 0x31, 0x18}, {306, 562, 0x32, 0x18}, {309, 375, 0x33, 0x17},
	{312, 187, 0x34, 0x16}, {315, 0, 0x36, 0x15}, {317, 812, 0x37, 0x14},
	{320, 625, 0x38, 0x13}, {323, 437, 0x39, 0x11}, {326, 250, 0x39, 0x10},
	{329, 62, 0x3A, 0x0F}, {331, 875, 0x3B, 0x0E}, {334, 687, 0x3C, 0x0D},
	{337, 500, 0x3C, 0x0B}, {340, 312, 0x3D, 0x0A}, {343, 125, 0x3E, 0x08},
	{345, 937, 0x3E, 0x07}, {348, 750, 0x3E, 0x06}, {351, 562, 0x3F, 0x04},
	{354, 375, 0x3F, 0x03}, {357, 187, 0x3F, 0x01}
};

static int adar1000_reg_access(struct iio_dev *indio_dev,
			       u32 reg, u32 writeval,
			       u32 *readval)
{
	struct adar1000_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, st->dev_addr | reg, readval);
	else
		return regmap_write(st->regmap, st->dev_addr | reg, writeval);
}

#define ADAR1000_RX_CHANNEL(_num)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE),			\
}

#define ADAR1000_TX_CHANNEL(_num)				\
{								\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) | \
		BIT(IIO_CHAN_INFO_PHASE),			\
}

#define ADAR1000_TEMP_CHANNEL(_num)				\
{	.type = IIO_TEMP,					\
	.indexed = 1,						\
	.channel = (_num),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),	\
}

static const struct iio_chan_spec adar1000_chan[] = {
	ADAR1000_TEMP_CHANNEL(0),
	ADAR1000_RX_CHANNEL(0),
	ADAR1000_RX_CHANNEL(1),
	ADAR1000_RX_CHANNEL(2),
	ADAR1000_RX_CHANNEL(3),
	ADAR1000_TX_CHANNEL(0),
	ADAR1000_TX_CHANNEL(1),
	ADAR1000_TX_CHANNEL(2),
	ADAR1000_TX_CHANNEL(3),
};

static int adar1000_get_atten(struct adar1000_state *st, u32 ch_num, u8 output)
{
	u32 val, reg;
	int ret;
	u32 code;

	if (output)
		reg = ADAR1000_CH_TX_GAIN(ch_num);
	else
		reg = ADAR1000_CH_RX_GAIN(ch_num);

	ret = regmap_read(st->regmap, st->dev_addr | reg, &val);
	if (ret < 0)
		return ret;

	val &= ~ADAR1000_CH_ATTN;
	if (val & ADAR1000_CH_ATTN)
		code = val * 125 + 16000;
	else
		code = val * 125;

	return code;
}

static int adar1000_set_atten(struct adar1000_state *st, u32 atten_mdb,
			      u32 ch_num, u8 output)
{
	u32 temp = 0, code, reg;

	/* The values for this are explained at page 33 of the datasheet */
	if (atten_mdb > 31000)
		return -EINVAL;

	if (atten_mdb > 16000) {
		atten_mdb -= 16000; /* will enable step attenuator */
		temp |= ADAR1000_CH_ATTN;
	}

	code = atten_mdb / 125; /* translate from mdB to codes  */
	temp |= ADAR1000_VGA_GAIN(code);

	if (output)
		reg = ADAR1000_CH_TX_GAIN(ch_num);
	else
		reg = ADAR1000_CH_RX_GAIN(ch_num);

	return regmap_write(st->regmap, st->dev_addr | reg, temp);
}

static int adar1000_read_adc(struct adar1000_state *st, u8 adc_ch, s32 *adc_data)
{
	u32 adc_ctrl;
	int ret;

	/* Select ADC channel */
	ret = regmap_update_bits(st->regmap, st->dev_addr | ADAR1000_ADC_CTRL,
				 ADAR1000_MUX_SEL_MSK,
				 ADAR1000_MUX_SEL(adc_ch));
	if (ret < 0)
		return ret;

	/* Start ADC conversion */
	ret = regmap_update_bits(st->regmap, st->dev_addr | ADAR1000_ADC_CTRL,
				 ADAR1000_ST_CONV, ADAR1000_ST_CONV);
	if (ret < 0)
		return ret;

	do {
		ret = regmap_read(st->regmap, st->dev_addr | ADAR1000_ADC_CTRL,
				  &adc_ctrl);
		if (ret < 0)
			return ret;
	} while (adc_ctrl & ADAR1000_ADC_EOC);

	/* Read ADC sample */
	return regmap_read(st->regmap, st->dev_addr | ADAR1000_ADC_OUTPUT,
			   adc_data);
}

static int adar1000_set_phase(struct adar1000_state *st, u8 ch_num, u8 output,
			      u32 val, u32 val2)
{
	int ret, i, prev, next, value;
	u32 vm_gain_i, vm_gain_q;
	u16 reg_i, reg_q;

	for (i = 0; i < st->pt_size - 1; i++) {
		if (st->pt_info[i].val > val)
			break;
	}

	prev = st->pt_info[i - 1].val * 1000 +
		st->pt_info[i - 1].val2;
	next = st->pt_info[i].val * 1000 +
		st->pt_info[i].val2;
	value = val * 1000 + val2;

	if (prev > next) {
		vm_gain_i = st->pt_info[i - 1].vm_gain_i;
		vm_gain_q = st->pt_info[i - 1].vm_gain_q;
		value = prev;
	} else {
		vm_gain_i = st->pt_info[i].vm_gain_i;
		vm_gain_q = st->pt_info[i].vm_gain_q;
		value = next;
	}

	if (output) {
		reg_i = st->dev_addr | ADAR1000_CH_TX_PHASE_I(ch_num);
		reg_q = st->dev_addr | ADAR1000_CH_TX_PHASE_Q(ch_num);
		st->tx_phase[st->dev_addr][ch_num] = value;
	} else {
		reg_i = st->dev_addr | ADAR1000_CH_RX_PHASE_I(ch_num);
		reg_q = st->dev_addr | ADAR1000_CH_RX_PHASE_Q(ch_num);
		st->rx_phase[st->dev_addr][ch_num] = value;
	}

	ret = regmap_write(st->regmap, reg_i, vm_gain_i);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, reg_q, vm_gain_q);
}

static int adar1000_get_phase(struct adar1000_state *st, u8 ch_num, u8 output,
			      u32 *val, u32 *val2)
{
	if (output) {
		*val = st->tx_phase[st->dev_addr][ch_num] / 1000;
		*val2 = (st->tx_phase[st->dev_addr][ch_num] % 1000) * 1000;
	} else {
		*val = st->rx_phase[st->dev_addr][ch_num] / 1000;
		*val2 = (st->rx_phase[st->dev_addr][ch_num] % 1000) * 1000;
	}

	return IIO_VAL_INT_PLUS_MICRO;
}

static int adar1000_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long m)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;
	s32 adc_data;

	switch (m) {
	case IIO_CHAN_INFO_PROCESSED:
		ret = adar1000_read_adc(st, 0x04, &adc_data);
		if (ret < 0)
			return ret;

		*val2 = adc_data * 800;

		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_HARDWAREGAIN:
		ret = adar1000_get_atten(st, chan->channel, chan->output);
		if (ret < 0)
			return ret;

		*val = -1 * (ret / 1000);
		*val2 = (ret % 1000) * 1000;

		if (!*val)
			*val2 *= -1;

		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return adar1000_get_phase(st, chan->channel, chan->output,
					  val, val2);
	default:
		return -EINVAL;
	}
};

static int adar1000_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	u32 code;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		if (val > 0 || (val == 0 && val2 > 0)) {
			ret = -EINVAL;
			return ret;
		}

		code = (abs(val) * 1000) + (abs(val2) / 1000);

		return adar1000_set_atten(st, code, chan->channel,
					   chan->output);
	case IIO_CHAN_INFO_PHASE:
		return adar1000_set_phase(st, chan->channel, chan->output,
					  val, val2);
	default:
		return -EINVAL;
	};
}

static int adar1000_write_raw_get_fmt(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan,
				      long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		return IIO_VAL_INT_PLUS_MICRO_DB;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT_PLUS_MICRO;
	default:
		return -EINVAL;
	}
}

static const struct iio_info adar1000_info = {
	.read_raw = &adar1000_read_raw,
	.write_raw = &adar1000_write_raw,
	.write_raw_get_fmt = &adar1000_write_raw_get_fmt,
	.debugfs_reg_access = &adar1000_reg_access,
};

static void adar1000_free_pt(struct adar1000_state *st,
			     struct adar1000_phase *table)
{
	if (!table || table == adar1000_phase_values)
		return;

	devm_kfree(&st->spi->dev, table);
}

static struct adar1000_phase *adar1000_parse_pt(struct adar1000_state *st,
						char *data, u32 size)
{
	struct adar1000_phase *table;
	unsigned int model, count;
	bool header_found;
	int i = 0, ret = 0;
	char *line, *ptr = data;

	header_found = false;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size)
			break;

		if (line[0] == '#') /* skip comment lines */
			continue;

		if (!header_found) {

			ret = sscanf(line, " <phasetable ADAR%i table entries=%i>",
				     &model, &count);

			if (ret == 2) {
				if (!(model == 1000)) {
					ret = -EINVAL;
					goto out;
				}
				table = devm_kzalloc(&st->spi->dev,
					sizeof(struct adar1000_phase) * count,
					GFP_KERNEL);
				if (!table) {
					ret = -ENOMEM;
					goto out;
				}

				header_found = true;
				i = 0;

				continue;
			} else {
				header_found = false;
			}
		}

		if (header_found) {
			int a, b, c, d;

			ret = sscanf(line, " %i.%i,%i,%i", &a, &b, &c, &d);
			if (ret == 4) {
				if (i >= count)
					goto out;

				table[i].val = a;
				table[i].val2 = b;
				table[i].vm_gain_i = c;
				table[i].vm_gain_q = d;
				i++;
				continue;
			} else if (strstr(line, "</phasetable>")) {
				goto done;
			} else {
				dev_err(&st->spi->dev,
					"ERROR: Malformed phase table");
				goto out_free_table;
			}
		}
	}

done:
	if (header_found) {
		st->pt_size = count;
		return table;
	} else {
		return ERR_PTR(ret);
	}

out_free_table:
	adar1000_free_pt(st, table);

out:
	return ERR_PTR(ret);
}

static int adar1000_request_pt(struct adar1000_state *st)
{

	struct iio_dev *indio_dev;
	const struct firmware *fw;
	struct adar1000_phase *phase_table;
	const char *name;
	char *cpy;
	int ret;

	indio_dev = iio_priv_to_dev(st);

	if (of_property_read_string(indio_dev->dev.of_node,
				    "adi,phasetable-name", &name))
		return -ENOENT;

	ret = request_firmware(&fw, name, &st->spi->dev);
	if (ret) {
		dev_err(&st->spi->dev,
			"request_firmware(%s) failed with %i\n", name, ret);
		return ret;
	}

	cpy = devm_kzalloc(&st->spi->dev, fw->size, GFP_KERNEL);
	if (!cpy)
		goto out;

	memcpy(cpy, fw->data, fw->size);

	phase_table = adar1000_parse_pt(st, cpy, fw->size);
	if (IS_ERR_OR_NULL(phase_table)) {
		ret = PTR_ERR(phase_table);
		goto out;
	}

	adar1000_free_pt(st, st->pt_info);

	st->pt_info = phase_table;

out:
	release_firmware(fw);

	return ret;
}

static ssize_t adar1000_pt_bin_write(struct file *filp, struct kobject *kobj,
				     struct bin_attribute *bin_attr,
				     char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adar1000_state *st = iio_priv(indio_dev);
	struct adar1000_phase *table;

	if (off == 0) {
		if (!st->bin_attr_buf) {
			st->bin_attr_buf = devm_kzalloc(&st->spi->dev,
							bin_attr->size,
							GFP_KERNEL);
			if (!st->bin_attr_buf)
				return -ENOMEM;
		} else {
			memset(st->bin_attr_buf, 0, bin_attr->size);
		}
	}

	memcpy(st->bin_attr_buf + off, buf, count);

	if (strnstr(st->bin_attr_buf, "</phasetable>", off + count) == NULL)
		return count;

	table = adar1000_parse_pt(st, st->bin_attr_buf, off + count);
	if (IS_ERR_OR_NULL(table))
		return PTR_ERR(table);

	adar1000_free_pt(st, st->pt_info);

	st->pt_info = table;

	return count;
}

static ssize_t adar1000_pt_bin_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr,
				    char *buf, loff_t off, size_t count)
{

	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret, i, len = 0;
	char *tab;

	tab = kzalloc(count, GFP_KERNEL);
	if (!tab)
		return -ENOMEM;

	len += snprintf(tab + len, count - len,
			"<phasetable ADAR%i table entries=%i>\n", 1000,
			st->pt_size);

	for (i = 0; i < st->pt_size; i++)
		len += snprintf(tab + len, count - len,
			"%d.%03d, 0x%.2X, 0x%.2X\n",
			st->pt_info[i].val,
			st->pt_info[i].val2,
			st->pt_info[i].vm_gain_i,
			st->pt_info[i].vm_gain_q);

	len += snprintf(tab + len, count - len, "</phasetable>\n");

	ret = memory_read_from_buffer(buf, count, &off, tab, len);

	kfree(tab);

	return ret;
}

static int adar1000_setup(struct iio_dev *indio_dev)
{
	struct adar1000_state *st = iio_priv(indio_dev);
	int ret;

	/* Load phase values */
	ret = adar1000_request_pt(st);
	if (ret < 0) {
		st->pt_info = (struct adar1000_phase *)&adar1000_phase_values[0];
		st->pt_size = ARRAY_SIZE(adar1000_phase_values);
	}

	/* Reset device */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_INTERFACE_CFG_A,
			   ADAR1000_SOFTRESET |
			   ADAR1000_ADDR_ASCN |
			   ADAR1000_SDOACTIVE);
	if (ret < 0)
		return ret;

	/* Adjust LDOs */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_LDO_TRIM_CTL_1,
			   ADAR1000_LDO_TRIM_SEL(2));
	if (ret < 0)
		return ret;

	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_LDO_TRIM_CTL_0, 0x55);
	if (ret < 0)
		return ret;

	/* Select SPI for channel settings */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_MEM_CTRL,
			   ADAR1000_BEAM_RAM_BYPASS |
			   ADAR1000_BIAS_RAM_BYPASS |
			   ADAR1000_TX_CHX_RAM_BYPASS |
			   ADAR1000_RX_CHX_RAM_BYPASS);
	if (ret < 0)
		return ret;

	/* Select TR input */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_SW_CTRL,
			   ADAR1000_SW_DRV_EN_TR |
			   ADAR1000_SW_DRV_EN_POL |
			   ADAR1000_TR_SOURCE);
	if (ret < 0)
		return ret;

	/* Setup ADC operation */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_ADC_CTRL, ADAR1000_AC_EN |
			   ADAR1000_CLK_EN | ADAR1000_MUX_SEL(0x05));
	if (ret < 0)
		return ret;

	/* Enable Channels */
	ret = regmap_write(st->regmap, st->dev_addr |
			   ADAR1000_RX_ENABLES, ADAR1000_CH1_EN |
			   ADAR1000_CH2_EN | ADAR1000_CH3_EN |
			   ADAR1000_CH4_EN | ADAR1000_LNA_EN |
			   ADAR1000_VM_EN | ADAR1000_VGA_EN);
	if (ret < 0)
		return ret;

	return regmap_write(st->regmap, st->dev_addr |
			    ADAR1000_TX_ENABLES, ADAR1000_CH1_EN |
			    ADAR1000_CH2_EN | ADAR1000_CH3_EN |
			    ADAR1000_CH4_EN | ADAR1000_LNA_EN |
			    ADAR1000_VM_EN | ADAR1000_VGA_EN);
}

static int adar1000_probe(struct spi_device *spi)
{
	struct iio_dev **indio_dev;
	struct adar1000_state **st;
	struct device_node *child, *np = spi->dev.of_node;
	struct regmap *regmap;
	int ret, cnt = 0, num_dev;
	u32 tmp;

	num_dev = of_get_available_child_count(np);
	if (num_dev < 1 || num_dev > ADAR1000_MAX_DEV) {
		dev_err(&spi->dev, "Number of devices is incorrect (%d)\n",
			num_dev);
		return -ENODEV;
	}

	indio_dev = devm_kzalloc(&spi->dev, num_dev, sizeof(indio_dev));
	if (!indio_dev)
		return -ENOMEM;

	st = devm_kzalloc(&spi->dev, num_dev, sizeof(st));
	if (!st)
		return -ENOMEM;

	regmap = devm_regmap_init_spi(spi, &adar1000_regmap_config);
	if (IS_ERR(regmap)) {
		dev_err(&spi->dev, "Error initializing spi regmap: %ld\n",
			PTR_ERR(regmap));
		return PTR_ERR(regmap);
	}

	for_each_available_child_of_node(np, child) {
		indio_dev[cnt] = devm_iio_device_alloc(&spi->dev,
						       sizeof(*st));
		if (!indio_dev[cnt])
			return -ENOMEM;

		st[cnt] = iio_priv(indio_dev[cnt]);
		spi_set_drvdata(spi, indio_dev[cnt]);
		st[cnt]->spi = spi;
		st[cnt]->regmap = regmap;

		ret = of_property_read_u32(child, "reg", &tmp);
		if (ret < 0)
			return ret;

		st[cnt]->dev_addr = ADAR1000_SPI_ADDR(tmp);

		sysfs_bin_attr_init(&st[cnt]->bin_pt);
		st[cnt]->bin_pt.attr.name = "phase_table_config";
		st[cnt]->bin_pt.attr.mode = 0644;
		st[cnt]->bin_pt.write = adar1000_pt_bin_write;
		st[cnt]->bin_pt.read = adar1000_pt_bin_read;
		st[cnt]->bin_pt.size = 4096;

		indio_dev[cnt]->dev.parent = &spi->dev;
		indio_dev[cnt]->dev.of_node = child;
		indio_dev[cnt]->name = spi->dev.of_node->name;
		indio_dev[cnt]->info = &adar1000_info;
		indio_dev[cnt]->modes = INDIO_DIRECT_MODE;
		indio_dev[cnt]->channels = adar1000_chan;
		indio_dev[cnt]->num_channels = ARRAY_SIZE(adar1000_chan);

		ret = adar1000_setup(indio_dev[cnt]);
		if (ret < 0) {
			dev_err(&spi->dev, "Setup failed (%d)\n", ret);
			return ret;
		}

		ret = devm_iio_device_register(&spi->dev, indio_dev[cnt]);
		if (ret < 0)
			return ret;

		ret = sysfs_create_bin_file(&indio_dev[cnt]->dev.kobj,
					    &st[cnt]->bin_pt);
		if (ret < 0)
			return ret;

		cnt++;
	}

	return 0;
}

static const struct of_device_id adar1000_of_match[] = {
	{ .compatible = "adi,adar1000"},
	{ }
};
MODULE_DEVICE_TABLE(of, adar1000_of_match);

static struct spi_driver adar1000_driver = {
	.driver = {
		.name	= "adar1000",
		.of_match_table = adar1000_of_match,
	},
	.probe		= adar1000_probe,
};
module_spi_driver(adar1000_driver);

MODULE_AUTHOR("Mircea Caprioru <mircea.caprioru@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADAR1000 Beamformer");
MODULE_LICENSE("GPL v2");
