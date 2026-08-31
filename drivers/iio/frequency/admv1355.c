// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADMV1355 Microwave Upconverter Driver
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/clk/clkscale.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/units.h>

#define ADMV1355_REG_SDO_CTRL		0x000
#define ADMV1355_REG_PRODUCT_ID_LSB	0x004
#define ADMV1355_REG_SCRATCH_PAD	0x00A
#define ADMV1355_REG_NVM_CTRL		0x078
#define ADMV1355_REG_NVM_ADDR		0x07B
#define ADMV1355_REG_NVM_DATA		0x07C
#define ADMV1355_REG_NVM_LOAD		0x07E
#define ADMV1355_REG_RF_HS_PD		0x13E
#define ADMV1355_REG_FILTER_SM_STEP	0x200
#define ADMV1355_REG_FILTER_LUT_EN	0x202
#define ADMV1355_REG_ADC_MUX		0x2A7
#define ADMV1355_REG_ADC_CTRL		0x300
#define ADMV1355_REG_ADC_RESET		0x302
#define ADMV1355_REG_LON_CTRL		0x306
#define ADMV1355_REG_ADC_OUT		0x30D
#define ADMV1355_REG_FILTER_TBL_SEL	0x207
#define ADMV1355_REG_FILTER_LOAD_EN	0x208
#define ADMV1355_REG_GAIN_LOAD_EN	0x281
#define ADMV1355_REG_GAIN_LUT_EN	0x285
#define ADMV1355_REG_GAIN_TBL_BYP	0x28A
#define ADMV1355_REG_DSA_BYPASS		0x28B
#define ADMV1355_REG_GPO_G_BYPASS	0x28C
#define ADMV1355_REG_RF_LPF		0x2A0
#define ADMV1355_REG_RF_HPF		0x2A1
#define ADMV1355_REG_DSA_DIRECT		0x600
#define ADMV1355_REG_GPO_G_DIRECT	0x601
#define ADMV1355_REG_IF_ENABLE		0x602
#define ADMV1355_REG_GPO_F_OE		0x780
#define ADMV1355_REG_GPO_GF_OE		0x781
#define ADMV1355_REG_GPO_F_DIRECT	0x803
#define ADMV1355_REG_LO_X4_FILTER	0x800
#define ADMV1355_REG_LO_X3_FILTER	0x801
#define ADMV1355_REG_LO_X1_FILTER	0x802
#define ADMV1355_REG_LO_TRAP_7_0	0x804
#define ADMV1355_REG_LO_TRAP_9_8	0x805
#define ADMV1355_REG_LO_BAND		0x806
#define ADMV1355_REG_LON_OFFSET_I	0x808
#define ADMV1355_REG_LON_OFFSET_Q	0x809
#define ADMV1355_REG_IF_GAIN		0x80A
#define ADMV1355_REG_RF_FILTER		0x80B
#define ADMV1355_REG_LO_DBL_BAND	0x80C

#define ADMV1355_REG_FILTER_LUTA_BASE	0x900
#define ADMV1355_REG_FILTER_LUTB_BASE	0xAA0
#define ADMV1355_REG_GAIN_LUT_BASE	0xE00

#define ADMV1355_FILTER_LUT_ENTRY_SIZE	13
#define ADMV1355_FILTER_LUT_NUM_ENTRIES	32
#define ADMV1355_FILTER_LUT_CONFIG_SIZE	4096

#define ADMV1355_GAIN_LUT_ENTRY_SIZE	2
#define ADMV1355_GAIN_LUT_NUM_ENTRIES	67
#define ADMV1355_GAIN_LUT_CONFIG_SIZE	2048

/* 0x28A: GAIN_LUT_BYPASS_EN */
#define ADMV1355_GAIN_LUT_BYP_EN_MSK	BIT(0)

/* 0x28B: DSA2_BYPASS_VALUE[7:4], DSA1_BYPASS_VALUE[3:0] */
#define ADMV1355_DSA1_BYPASS_MSK	GENMASK(3, 0)
#define ADMV1355_DSA2_BYPASS_MSK	GENMASK(7, 4)

/* 0x2A0: LPF_SELECT[4], LPF_BYPASS_VALUE[3:0] */
#define ADMV1355_LPF_SELECT_MSK		BIT(4)
#define ADMV1355_LPF_BYPASS_VAL_MSK	GENMASK(3, 0)

/* 0x2A1: HPF_SELECT[7], HPF_BYPASS_VALUE[5:0] */
#define ADMV1355_HPF_SELECT_MSK		BIT(7)
#define ADMV1355_HPF_BYPASS_VAL_MSK	GENMASK(5, 0)

/* 0x600: RF_DSA2_GAIN[7:4], RF_DSA1_GAIN[3:0] */
#define ADMV1355_DSA1_DIRECT_MSK	GENMASK(3, 0)
#define ADMV1355_DSA2_DIRECT_MSK	GENMASK(7, 4)

/* 0x602: IF_HYBRID_EN[3], IF_HIGH_FREQ_EN[2], IF_LOW_FREQ_EN[1], BB_EN[0] */
#define ADMV1355_BB_EN_MSK		BIT(0)
#define ADMV1355_IF_LOW_FREQ_EN_MSK	BIT(1)
#define ADMV1355_IF_HIGH_FREQ_EN_MSK	BIT(2)
#define ADMV1355_IF_HYBRID_EN_MSK	BIT(3)
#define ADMV1355_IF_MODE_MSK		GENMASK(3, 0)

/* 0x805: RF_BAND_SELECT[7], LO_PHASE_Q[6:2], LO_TRAP_FILTER[9:8] at [1:0] */
#define ADMV1355_RF_BAND_805_MSK	BIT(7)
#define ADMV1355_LO_PHASE_Q_MSK	GENMASK(6, 2)

/* 0x806: LO_BAND[7], MIXER_BAND[6], LO_SIDEBAND[5], LO_PHASE_I[4:0] */
#define ADMV1355_LO_BAND_806_MSK	BIT(7)
#define ADMV1355_MIXER_BAND_806_MSK	BIT(6)
#define ADMV1355_LO_SIDEBAND_MSK	BIT(5)
#define ADMV1355_LO_PHASE_I_MSK	GENMASK(4, 0)

/* 0x601: GPO_G_DIRECT[4:0] */
#define ADMV1355_GPO_G_DIRECT_MSK	GENMASK(4, 0)

/* 0x780: GPO_G_OE[7:6], GPO_F_OE[5:0] */
#define ADMV1355_GPO_F_OE_MSK		GENMASK(5, 0)
#define ADMV1355_GPO_G_OE_LO_MSK	GENMASK(7, 6)

/* 0x781: GPO_G_OE[2:0] (bits [4:2] of full GPO_G_OE) */
#define ADMV1355_GPO_G_OE_HI_MSK	GENMASK(2, 0)

/* 0x803: GPO_F_DIRECT[5:0] */
#define ADMV1355_GPO_F_DIRECT_MSK	GENMASK(5, 0)

/* 0x80A: DSAQ[7:4], DSAI[3:0] */
#define ADMV1355_DSAI_MSK		GENMASK(3, 0)
#define ADMV1355_DSAQ_MSK		GENMASK(7, 4)

/* 0x800: LO_X4_FILTER[3:0] */
#define ADMV1355_LO_X4_FILTER_MSK	GENMASK(3, 0)

/* 0x801: LO_X3_FILTER[4:0] */
#define ADMV1355_LO_X3_FILTER_MSK	GENMASK(4, 0)

/* 0x802: LO_X1_FILTER[4:0] */
#define ADMV1355_LO_X1_FILTER_MSK	GENMASK(4, 0)

/* 0x805: LO_TRAP_FILTER[9:8] at [1:0] */
#define ADMV1355_LO_TRAP_9_8_MSK	GENMASK(1, 0)

/* 0x80B: RF_FILTER: HPF_DIRECT[7:4], LPF_DIRECT[3:0] */
#define ADMV1355_DIRECT_LPF_MSK	GENMASK(3, 0)
#define ADMV1355_DIRECT_HPF_MSK	GENMASK(7, 4)

/* 0x28C: GPO_G_BYPASS[4:0] */
#define ADMV1355_GPO_G_BYPASS_MSK	GENMASK(4, 0)

#define ADMV1355_SCRATCH_TEST_VAL	0xA5

static const char * const admv1355_rf_band_items[] = {
	"low_band",
	"high_band",
};

static const char * const admv1355_mixer_sideband_items[] = {
	"LSB",
	"USB",
};

static const char * const admv1355_if_mode_items[] = {
	"baseband",
	"complex_if_low",
	"complex_if_high",
	"if_hybrid",
};

static const char * const admv1355_dsa_4bit_items[] = {
	"0dB", "-1dB", "-2dB", "-3dB", "-4dB", "-5dB", "-6dB", "-7dB",
	"-8dB", "-9dB", "-10dB", "-11dB", "-12dB", "-13dB", "-14dB", "-15dB"
};

static const char * const admv1355_dsa_iq_items[] = {
	"0dB", "-100mdB", "-200mdB", "-300mdB",
	"-400mdB", "-500mdB", "-600mdB", "-700mdB",
	"-800mdB", "-900mdB", "-1000mdB", "-1100mdB",
	"-1200mdB", "-1300mdB", "-1400mdB", "-1500mdB"
};

static const char * const admv1355_lo_x4_filter_items[] = {
	"10GHz_14GHz",
	"14GHz_18GHz",
	"ge_18GHz",
	"lt_10GHz",
};
static const u8 admv1355_lo_x4_filter_reg_values[] = {
	0x00, 0x05, 0x07, 0x08,
};

static const char * const admv1355_lo_x3_filter_items[] = {
	"ge_24GHz",
	"21GHz_24GHz",
	"19GHz_21GHz",
	"17GHz_19GHz",
	"15GHz_17GHz",
	"12GHz_15GHz",
	"lt_12GHz",
};
static const u8 admv1355_lo_x3_filter_reg_values[] = {
	0x00, 0x02, 0x03, 0x04, 0x05, 0x07, 0x1C,
};

static const char * const admv1355_lo_x1_filter_items[] = {
	"ge_21GHz",
	"17GHz_21GHz",
	"15GHz_17GHz",
	"12GHz_15GHz",
	"lt_12GHz",
};
static const u8 admv1355_lo_x1_filter_reg_values[] = {
	0x00, 0x01, 0x05, 0x0A, 0x1F,
};

enum admv1355_ext_info {
	ADMV1355_RF_LPF_BYPASS_EN,
	ADMV1355_RF_LPF_BYPASS_VAL,
	ADMV1355_RF_HPF_BYPASS_EN,
	ADMV1355_RF_HPF_BYPASS_VAL,
	ADMV1355_LO_PHASE_I,
	ADMV1355_LO_PHASE_Q,
	ADMV1355_LO_LON_OFFSET_I,
	ADMV1355_LO_LON_OFFSET_Q,
	ADMV1355_RF_DIRECT_LPF_VAL,
	ADMV1355_RF_DIRECT_HPF_VAL,
	ADMV1355_LO_TRAP,
	ADMV1355_LO_DBL_BAND,
};

enum admv1355_dev_attr_id {
	ADMV1355_DEV_ATTR_FILTER_LUT_EN,
	ADMV1355_DEV_ATTR_FILTER_LOAD_EN,
	ADMV1355_DEV_ATTR_GAIN_LUT_EN,
	ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN,
	ADMV1355_DEV_ATTR_GAIN_LOAD_EN,
	ADMV1355_DEV_ATTR_FILTER_TABLE_SEL,
	ADMV1355_DEV_ATTR_DIRECT_GPO_G,
	ADMV1355_DEV_ATTR_BYPASS_GPO_G,
	ADMV1355_DEV_ATTR_DIRECT_GPO_F,
	ADMV1355_DEV_ATTR_GPO_F_OE,
	ADMV1355_DEV_ATTR_GPO_G_OE,
	ADMV1355_DEV_ATTR_FILTER_SM_STEP,
};

struct admv1355_priv {
	struct spi_device	*spi;
	struct gpio_desc	*cen_gpio;
	struct gpio_desc	*reset_gpio;
	struct regmap		*regmap;
	struct clk		*lo_input;
	struct clock_scale	clkscale;
	struct notifier_block	nb;
	struct mutex		lock;
};

/*
 * Custom SPI bus: the ADMV1355 SPI interface requires a single full-duplex
 * 3-byte transfer (like the ADMV1320). The default regmap SPI bus uses
 * spi_write_then_read() which splits the transaction into separate write
 * and read phases, causing the read data pipeline to return stale values.
 *
 * Frame format (3 bytes):
 *   Byte 0: [R/W_n][0][0][0][A11:A8]  — bit 7 = read flag
 *   Byte 1: [A7:A0]
 *   Byte 2: [D7:D0]  — write data (TX) or read data (RX)
 */
static int admv1355_spi_reg_read(void *context, unsigned int reg,
				 unsigned int *val)
{
	struct spi_device *spi = context;
	struct spi_transfer t = {};
	u8 tx_buf[3], rx_buf[3];
	int ret;

	tx_buf[0] = BIT(7) | ((reg >> 8) & 0x0F);
	tx_buf[1] = reg & 0xFF;
	tx_buf[2] = 0x00;

	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = 3;

	ret = spi_sync_transfer(spi, &t, 1);
	if (!ret)
		*val = rx_buf[2];

	return ret;
}

static int admv1355_spi_reg_write(void *context, unsigned int reg,
				  unsigned int val)
{
	struct spi_device *spi = context;
	u8 tx_buf[3];

	tx_buf[0] = (reg >> 8) & 0x0F;
	tx_buf[1] = reg & 0xFF;
	tx_buf[2] = val & 0xFF;

	return spi_write(spi, tx_buf, 3);
}

static int admv1355_spi_read(struct admv1355_priv *priv, unsigned int reg,
			     unsigned int *val)
{
	int ret;

	ret = regmap_read(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->spi->dev, "%s: REG 0x%03X read failed (%d)\n",
			__func__, reg, ret);
	else
		dev_dbg(&priv->spi->dev, "%s: REG 0x%03X VAL 0x%02X\n",
			__func__, reg, *val);

	return ret;
}

static int admv1355_spi_write(struct admv1355_priv *priv, unsigned int reg,
			      unsigned int val)
{
	int ret;

	ret = regmap_write(priv->regmap, reg, val);
	if (ret)
		dev_err(&priv->spi->dev, "%s: REG 0x%03X VAL 0x%02X write failed (%d)\n",
			__func__, reg, val, ret);
	else
		dev_dbg(&priv->spi->dev, "%s: REG 0x%03X VAL 0x%02X\n",
			__func__, reg, val);

	return ret;
}

static int admv1355_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			       unsigned int write_val, unsigned int *read_val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);

	guard(mutex)(&priv->lock);

	if (read_val)
		return admv1355_spi_read(priv, reg, read_val);

	return admv1355_spi_write(priv, reg, write_val);
}

static int admv1355_read_temp(struct admv1355_priv *priv, int *val)
{
	unsigned int adc_code;
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_ADC_RESET, 0x02);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_ADC_CTRL, 0x09);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LON_CTRL, 0x02);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HS_PD, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_ADC_MUX, 0x01);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_ADC_RESET, 0x06);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = admv1355_spi_read(priv, ADMV1355_REG_ADC_OUT, &adc_code);
	if (ret)
		return ret;

	/* TC (milli-degrees C) = -(5/6) * ADC_CODE + 173 (datasheet Eq. 3) */
	*val = -5 * (int)adc_code * 1000 / 6 + 173000;

	/* Restore power-down register to init state */
	admv1355_spi_write(priv, ADMV1355_REG_RF_HS_PD, 0x02);
	admv1355_spi_write(priv, ADMV1355_REG_ADC_CTRL, 0x00);

	return 0;
}

static int admv1355_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	int ret;

	if (chan->type != IIO_TEMP)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		guard(mutex)(&priv->lock);
		ret = admv1355_read_temp(priv, val);
		return ret ? ret : IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

/*
 * Macro to generate get/set/iio_enum for simple register-mask-enum patterns
 */
#define ADMV1355_REG_ENUM(_name, _reg, _mask, _items)				\
static int admv1355_get_##_name(struct iio_dev *indio_dev,			\
				const struct iio_chan_spec *chan)			\
{										\
	struct admv1355_priv *priv = iio_priv(indio_dev);			\
	unsigned int data;							\
	int ret;								\
										\
	guard(mutex)(&priv->lock);						\
	ret = regmap_read(priv->regmap, _reg, &data);				\
	return ret ? ret : FIELD_GET(_mask, data);				\
}										\
										\
static int admv1355_set_##_name(struct iio_dev *indio_dev,			\
				const struct iio_chan_spec *chan, u32 val)	\
{										\
	struct admv1355_priv *priv = iio_priv(indio_dev);			\
										\
	guard(mutex)(&priv->lock);						\
	return regmap_update_bits(priv->regmap, _reg, _mask,			\
				  FIELD_PREP(_mask, val));			\
}										\
										\
static const struct iio_enum admv1355_##_name##_enum = {			\
	.items = _items,							\
	.num_items = ARRAY_SIZE(_items),					\
	.get = admv1355_get_##_name,						\
	.set = admv1355_set_##_name,						\
}

/* RF DSA enums */
ADMV1355_REG_ENUM(rf_direct_dsa1, ADMV1355_REG_DSA_DIRECT,
		  ADMV1355_DSA1_DIRECT_MSK, admv1355_dsa_4bit_items);
ADMV1355_REG_ENUM(rf_direct_dsa2, ADMV1355_REG_DSA_DIRECT,
		  ADMV1355_DSA2_DIRECT_MSK, admv1355_dsa_4bit_items);
ADMV1355_REG_ENUM(rf_bypass_dsa1, ADMV1355_REG_DSA_BYPASS,
		  ADMV1355_DSA1_BYPASS_MSK, admv1355_dsa_4bit_items);
ADMV1355_REG_ENUM(rf_bypass_dsa2, ADMV1355_REG_DSA_BYPASS,
		  ADMV1355_DSA2_BYPASS_MSK, admv1355_dsa_4bit_items);

/* Mixer sideband enum */
ADMV1355_REG_ENUM(mixer_sideband, ADMV1355_REG_LO_BAND,
		  ADMV1355_LO_SIDEBAND_MSK, admv1355_mixer_sideband_items);

/* IF fine-adjust DSA enums */
ADMV1355_REG_ENUM(dsai, ADMV1355_REG_IF_GAIN,
		  ADMV1355_DSAI_MSK, admv1355_dsa_iq_items);
ADMV1355_REG_ENUM(dsaq, ADMV1355_REG_IF_GAIN,
		  ADMV1355_DSAQ_MSK, admv1355_dsa_iq_items);

/*
 * LO filter enums — non-contiguous register codes require custom get/set
 * with a register-value lookup table.
 */
#define ADMV1355_FILTER_ENUM(_name, _reg, _mask, _items, _reg_values)	\
static int admv1355_get_##_name(struct iio_dev *indio_dev,		\
				const struct iio_chan_spec *chan)		\
{									\
	struct admv1355_priv *priv = iio_priv(indio_dev);		\
	unsigned int data, reg_val;					\
	int ret, i;							\
									\
	guard(mutex)(&priv->lock);					\
	ret = regmap_read(priv->regmap, _reg, &data);			\
	if (ret)							\
		return ret;						\
	reg_val = FIELD_GET(_mask, data);				\
	for (i = 0; i < ARRAY_SIZE(_reg_values); i++) {			\
		if (_reg_values[i] == reg_val)				\
			return i;					\
	}								\
	return -EINVAL;							\
}									\
									\
static int admv1355_set_##_name(struct iio_dev *indio_dev,		\
				const struct iio_chan_spec *chan,		\
				unsigned int val)			\
{									\
	struct admv1355_priv *priv = iio_priv(indio_dev);		\
									\
	if (val >= ARRAY_SIZE(_reg_values))				\
		return -EINVAL;						\
	guard(mutex)(&priv->lock);					\
	return regmap_update_bits(priv->regmap, _reg, _mask,		\
				  FIELD_PREP(_mask, _reg_values[val]));	\
}									\
									\
static const struct iio_enum admv1355_##_name##_enum = {		\
	.items = _items,						\
	.num_items = ARRAY_SIZE(_items),				\
	.get = admv1355_get_##_name,					\
	.set = admv1355_set_##_name,					\
}

ADMV1355_FILTER_ENUM(lo_x4_filter, ADMV1355_REG_LO_X4_FILTER,
		     ADMV1355_LO_X4_FILTER_MSK,
		     admv1355_lo_x4_filter_items,
		     admv1355_lo_x4_filter_reg_values);
ADMV1355_FILTER_ENUM(lo_x3_filter, ADMV1355_REG_LO_X3_FILTER,
		     ADMV1355_LO_X3_FILTER_MSK,
		     admv1355_lo_x3_filter_items,
		     admv1355_lo_x3_filter_reg_values);
ADMV1355_FILTER_ENUM(lo_x1_filter, ADMV1355_REG_LO_X1_FILTER,
		     ADMV1355_LO_X1_FILTER_MSK,
		     admv1355_lo_x1_filter_items,
		     admv1355_lo_x1_filter_reg_values);

/* RF band select — controls bits in two registers (0x805[7] and 0x806[6]) */
static int admv1355_get_rf_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	guard(mutex)(&priv->lock);
	ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_9_8, &data);
	return ret ? ret : FIELD_GET(ADMV1355_RF_BAND_805_MSK, data);
}

static int admv1355_set_rf_band(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	int ret;

	guard(mutex)(&priv->lock);
	ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				 ADMV1355_RF_BAND_805_MSK,
				 FIELD_PREP(ADMV1355_RF_BAND_805_MSK, val));
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, ADMV1355_REG_LO_BAND,
				  ADMV1355_MIXER_BAND_806_MSK,
				  FIELD_PREP(ADMV1355_MIXER_BAND_806_MSK, val));
}

static const struct iio_enum admv1355_rf_band_enum = {
	.items = admv1355_rf_band_items,
	.num_items = ARRAY_SIZE(admv1355_rf_band_items),
	.get = admv1355_get_rf_band,
	.set = admv1355_set_rf_band,
};

/* IF mode select — maps enum index to individual enable bits in 0x602 */
static const u8 admv1355_if_mode_reg_values[] = {
	0x01, /* baseband */
	0x02, /* complex_if_low */
	0x04, /* complex_if_high */
	0x08, /* if_hybrid */
};

static int admv1355_get_if_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data;
	int ret, i;

	guard(mutex)(&priv->lock);
	ret = regmap_read(priv->regmap, ADMV1355_REG_IF_ENABLE, &data);
	if (ret)
		return ret;

	data &= ADMV1355_IF_MODE_MSK;
	for (i = 0; i < ARRAY_SIZE(admv1355_if_mode_reg_values); i++) {
		if (admv1355_if_mode_reg_values[i] == data)
			return i;
	}

	return -EINVAL;
}

static int admv1355_set_if_mode(struct iio_dev *indio_dev,
				const struct iio_chan_spec *chan,
				unsigned int val)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);

	if (val >= ARRAY_SIZE(admv1355_if_mode_reg_values))
		return -EINVAL;

	guard(mutex)(&priv->lock);
	return regmap_update_bits(priv->regmap, ADMV1355_REG_IF_ENABLE,
				  ADMV1355_IF_MODE_MSK,
				  admv1355_if_mode_reg_values[val]);
}

static const struct iio_enum admv1355_if_mode_enum = {
	.items = admv1355_if_mode_items,
	.num_items = ARRAY_SIZE(admv1355_if_mode_items),
	.get = admv1355_get_if_mode,
	.set = admv1355_set_if_mode,
};

static ssize_t admv1355_ext_info_read(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      char *buf)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int val, data;
	int ret;

	guard(mutex)(&priv->lock);

	switch (private) {
	case ADMV1355_RF_LPF_BYPASS_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_LPF, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  FIELD_GET(ADMV1355_LPF_SELECT_MSK, data) ?
				  "true" : "false");
	case ADMV1355_RF_LPF_BYPASS_VAL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_LPF, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_LPF_BYPASS_VAL_MSK, data);
		break;
	case ADMV1355_RF_HPF_BYPASS_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_HPF, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  FIELD_GET(ADMV1355_HPF_SELECT_MSK, data) ?
				  "true" : "false");
	case ADMV1355_RF_HPF_BYPASS_VAL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_HPF, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_HPF_BYPASS_VAL_MSK, data);
		break;
	case ADMV1355_LO_PHASE_I:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_BAND, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_LO_PHASE_I_MSK, data);
		break;
	case ADMV1355_LO_PHASE_Q:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				  &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_LO_PHASE_Q_MSK, data);
		break;
	case ADMV1355_LO_LON_OFFSET_I:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LON_OFFSET_I,
				  &data);
		if (ret)
			return ret;
		val = data;
		break;
	case ADMV1355_LO_LON_OFFSET_Q:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LON_OFFSET_Q,
				  &data);
		if (ret)
			return ret;
		val = data;
		break;
	case ADMV1355_RF_DIRECT_LPF_VAL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_FILTER, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_DIRECT_LPF_MSK, data);
		break;
	case ADMV1355_RF_DIRECT_HPF_VAL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_RF_FILTER, &data);
		if (ret)
			return ret;
		val = FIELD_GET(ADMV1355_DIRECT_HPF_MSK, data);
		break;
	case ADMV1355_LO_TRAP: {
		unsigned int lo, hi;

		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_7_0, &lo);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_9_8, &hi);
		if (ret)
			return ret;
		val = lo | (FIELD_GET(ADMV1355_LO_TRAP_9_8_MSK, hi) << 8);
		break;
	}
	case ADMV1355_LO_DBL_BAND:
		ret = regmap_read(priv->regmap, ADMV1355_REG_LO_DBL_BAND,
				  &data);
		if (ret)
			return ret;
		val = data;
		break;
	default:
		return -EINVAL;
	}

	return sysfs_emit(buf, "%u\n", val);
}

static ssize_t admv1355_ext_info_write(struct iio_dev *indio_dev,
				       uintptr_t private,
				       const struct iio_chan_spec *chan,
				       const char *buf, size_t len)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int val;
	bool bval;
	int ret;

	guard(mutex)(&priv->lock);

	switch (private) {
	case ADMV1355_RF_LPF_BYPASS_EN:
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_LPF,
					 ADMV1355_LPF_SELECT_MSK,
					 bval ? ADMV1355_LPF_SELECT_MSK : 0);
		break;
	case ADMV1355_RF_LPF_BYPASS_VAL:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 15)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_LPF,
					 ADMV1355_LPF_BYPASS_VAL_MSK,
					 FIELD_PREP(ADMV1355_LPF_BYPASS_VAL_MSK,
						    val));
		break;
	case ADMV1355_RF_HPF_BYPASS_EN:
		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_HPF,
					 ADMV1355_HPF_SELECT_MSK,
					 bval ? ADMV1355_HPF_SELECT_MSK : 0);
		break;
	case ADMV1355_RF_HPF_BYPASS_VAL:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 63)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_HPF,
					 ADMV1355_HPF_BYPASS_VAL_MSK,
					 FIELD_PREP(ADMV1355_HPF_BYPASS_VAL_MSK,
						    val));
		break;
	case ADMV1355_LO_PHASE_I:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_BAND,
					 ADMV1355_LO_PHASE_I_MSK,
					 FIELD_PREP(ADMV1355_LO_PHASE_I_MSK,
						    val));
		break;
	case ADMV1355_LO_PHASE_Q:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
					 ADMV1355_LO_PHASE_Q_MSK,
					 FIELD_PREP(ADMV1355_LO_PHASE_Q_MSK,
						    val));
		break;
	case ADMV1355_LO_LON_OFFSET_I:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 255)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_LON_OFFSET_I,
				   val);
		break;
	case ADMV1355_LO_LON_OFFSET_Q:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 255)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_LON_OFFSET_Q,
				   val);
		break;
	case ADMV1355_RF_DIRECT_LPF_VAL:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 15)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_FILTER,
					 ADMV1355_DIRECT_LPF_MSK,
					 FIELD_PREP(ADMV1355_DIRECT_LPF_MSK,
						    val));
		break;
	case ADMV1355_RF_DIRECT_HPF_VAL:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 15)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_RF_FILTER,
					 ADMV1355_DIRECT_HPF_MSK,
					 FIELD_PREP(ADMV1355_DIRECT_HPF_MSK,
						    val));
		break;
	case ADMV1355_LO_TRAP:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 1023)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_LO_TRAP_7_0,
				   val & 0xFF);
		if (ret)
			break;
		ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
					 ADMV1355_LO_TRAP_9_8_MSK,
					 FIELD_PREP(ADMV1355_LO_TRAP_9_8_MSK,
						    (val >> 8) & 0x03));
		break;
	case ADMV1355_LO_DBL_BAND:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 255)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_LO_DBL_BAND, val);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

#define ADMV1355_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = admv1355_ext_info_read, \
	.write = admv1355_ext_info_write, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info admv1355_rf_ext_info[] = {
	IIO_ENUM("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	IIO_ENUM_AVAILABLE("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	IIO_ENUM("direct_dsa1_gain", IIO_SEPARATE,
		 &admv1355_rf_direct_dsa1_enum),
	IIO_ENUM_AVAILABLE("direct_dsa1_gain", IIO_SEPARATE,
			   &admv1355_rf_direct_dsa1_enum),
	IIO_ENUM("direct_dsa2_gain", IIO_SEPARATE,
		 &admv1355_rf_direct_dsa2_enum),
	IIO_ENUM_AVAILABLE("direct_dsa2_gain", IIO_SEPARATE,
			   &admv1355_rf_direct_dsa2_enum),
	IIO_ENUM("bypass_dsa1_gain", IIO_SEPARATE,
		 &admv1355_rf_bypass_dsa1_enum),
	IIO_ENUM_AVAILABLE("bypass_dsa1_gain", IIO_SEPARATE,
			   &admv1355_rf_bypass_dsa1_enum),
	IIO_ENUM("bypass_dsa2_gain", IIO_SEPARATE,
		 &admv1355_rf_bypass_dsa2_enum),
	IIO_ENUM_AVAILABLE("bypass_dsa2_gain", IIO_SEPARATE,
			   &admv1355_rf_bypass_dsa2_enum),
	ADMV1355_EXT_INFO("direct_lpf_val", ADMV1355_RF_DIRECT_LPF_VAL),
	ADMV1355_EXT_INFO("direct_hpf_val", ADMV1355_RF_DIRECT_HPF_VAL),
	ADMV1355_EXT_INFO("bypass_lpf_en", ADMV1355_RF_LPF_BYPASS_EN),
	ADMV1355_EXT_INFO("bypass_lpf_val", ADMV1355_RF_LPF_BYPASS_VAL),
	ADMV1355_EXT_INFO("bypass_hpf_en", ADMV1355_RF_HPF_BYPASS_EN),
	ADMV1355_EXT_INFO("bypass_hpf_val", ADMV1355_RF_HPF_BYPASS_VAL),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_if_ext_info[] = {
	IIO_ENUM("mode", IIO_SEPARATE, &admv1355_if_mode_enum),
	IIO_ENUM_AVAILABLE("mode", IIO_SEPARATE, &admv1355_if_mode_enum),
	{ }
};

static const struct iio_chan_spec_ext_info admv1355_lo_ext_info[] = {
	IIO_ENUM("sideband", IIO_SEPARATE, &admv1355_mixer_sideband_enum),
	IIO_ENUM_AVAILABLE("sideband", IIO_SEPARATE,
			   &admv1355_mixer_sideband_enum),
	IIO_ENUM("dsai_0p1db", IIO_SEPARATE, &admv1355_dsai_enum),
	IIO_ENUM_AVAILABLE("dsai_0p1db", IIO_SEPARATE, &admv1355_dsai_enum),
	IIO_ENUM("dsaq_0p1db", IIO_SEPARATE, &admv1355_dsaq_enum),
	IIO_ENUM_AVAILABLE("dsaq_0p1db", IIO_SEPARATE, &admv1355_dsaq_enum),
	ADMV1355_EXT_INFO("direct_i_phase_val", ADMV1355_LO_PHASE_I),
	ADMV1355_EXT_INFO("direct_q_phase_val", ADMV1355_LO_PHASE_Q),
	ADMV1355_EXT_INFO("direct_lon_offset_i", ADMV1355_LO_LON_OFFSET_I),
	ADMV1355_EXT_INFO("direct_lon_offset_q", ADMV1355_LO_LON_OFFSET_Q),
	IIO_ENUM("lo_x4_filter", IIO_SEPARATE, &admv1355_lo_x4_filter_enum),
	IIO_ENUM_AVAILABLE("lo_x4_filter", IIO_SEPARATE,
			   &admv1355_lo_x4_filter_enum),
	IIO_ENUM("lo_x3_filter", IIO_SEPARATE, &admv1355_lo_x3_filter_enum),
	IIO_ENUM_AVAILABLE("lo_x3_filter", IIO_SEPARATE,
			   &admv1355_lo_x3_filter_enum),
	IIO_ENUM("lo_x1_filter", IIO_SEPARATE, &admv1355_lo_x1_filter_enum),
	IIO_ENUM_AVAILABLE("lo_x1_filter", IIO_SEPARATE,
			   &admv1355_lo_x1_filter_enum),
	ADMV1355_EXT_INFO("lo_trap", ADMV1355_LO_TRAP),
	ADMV1355_EXT_INFO("lo_dbl_band", ADMV1355_LO_DBL_BAND),
	{ }
};

/* Device-level attributes */
struct admv1355_attribute {
	enum admv1355_dev_attr_id id;
	struct device_attribute attr;
};

#define to_admv1355_attribute(x) container_of(x, struct admv1355_attribute, attr)

static ssize_t admv1355_dev_attr_show(struct device *dev,
				      struct device_attribute *dev_attr,
				      char *buf)
{
	const struct admv1355_attribute *attr = to_admv1355_attribute(dev_attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data;
	int ret;

	guard(mutex)(&priv->lock);

	switch (attr->id) {
	case ADMV1355_DEV_ATTR_FILTER_LUT_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_FILTER_LUT_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_FILTER_LOAD_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_FILTER_LOAD_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_GAIN_LUT_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GAIN_LUT_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GAIN_TBL_BYP,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & ADMV1355_GAIN_LUT_BYP_EN_MSK) ?
				  "true" : "false");
	case ADMV1355_DEV_ATTR_GAIN_LOAD_EN:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GAIN_LOAD_EN,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%s\n",
				  (data & BIT(0)) ? "true" : "false");
	case ADMV1355_DEV_ATTR_FILTER_TABLE_SEL:
		ret = regmap_read(priv->regmap, ADMV1355_REG_FILTER_TBL_SEL,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%c\n", (data & BIT(0)) ? 'B' : 'A');
	case ADMV1355_DEV_ATTR_DIRECT_GPO_G:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GPO_G_DIRECT,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n",
				  FIELD_GET(ADMV1355_GPO_G_DIRECT_MSK, data));
	case ADMV1355_DEV_ATTR_BYPASS_GPO_G:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GPO_G_BYPASS,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n",
				  FIELD_GET(ADMV1355_GPO_G_BYPASS_MSK, data));
	case ADMV1355_DEV_ATTR_DIRECT_GPO_F:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GPO_F_DIRECT,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n",
				  FIELD_GET(ADMV1355_GPO_F_DIRECT_MSK, data));
	case ADMV1355_DEV_ATTR_GPO_F_OE:
		ret = regmap_read(priv->regmap, ADMV1355_REG_GPO_F_OE, &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n",
				  FIELD_GET(ADMV1355_GPO_F_OE_MSK, data));
	case ADMV1355_DEV_ATTR_GPO_G_OE: {
		unsigned int lo, hi;

		ret = regmap_read(priv->regmap, ADMV1355_REG_GPO_F_OE, &lo);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, ADMV1355_REG_GPO_GF_OE, &hi);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%lu\n",
				  (FIELD_GET(ADMV1355_GPO_G_OE_LO_MSK, lo)) |
				  (FIELD_GET(ADMV1355_GPO_G_OE_HI_MSK, hi)
				   << 2));
	}
	case ADMV1355_DEV_ATTR_FILTER_SM_STEP:
		ret = regmap_read(priv->regmap, ADMV1355_REG_FILTER_SM_STEP,
				  &data);
		if (ret)
			return ret;
		return sysfs_emit(buf, "%u\n", data & 0xFF);
	default:
		return -EINVAL;
	}
}

static ssize_t admv1355_dev_attr_store(struct device *dev,
				       struct device_attribute *dev_attr,
				       const char *buf, size_t len)
{
	const struct admv1355_attribute *attr = to_admv1355_attribute(dev_attr);
	struct iio_dev *indio_dev = dev_to_iio_dev(dev);
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int val;
	bool bval;
	int ret;

	guard(mutex)(&priv->lock);

	switch (attr->id) {
	case ADMV1355_DEV_ATTR_FILTER_LUT_EN:
	case ADMV1355_DEV_ATTR_FILTER_LOAD_EN:
	case ADMV1355_DEV_ATTR_GAIN_LUT_EN:
	case ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN:
	case ADMV1355_DEV_ATTR_GAIN_LOAD_EN: {
		static const struct {
			unsigned int reg;
			unsigned int mask;
		} bool_map[] = {
			[ADMV1355_DEV_ATTR_FILTER_LUT_EN] = {
				ADMV1355_REG_FILTER_LUT_EN, BIT(0) },
			[ADMV1355_DEV_ATTR_FILTER_LOAD_EN] = {
				ADMV1355_REG_FILTER_LOAD_EN, BIT(0) },
			[ADMV1355_DEV_ATTR_GAIN_LUT_EN] = {
				ADMV1355_REG_GAIN_LUT_EN, BIT(0) },
			[ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN] = {
				ADMV1355_REG_GAIN_TBL_BYP,
				ADMV1355_GAIN_LUT_BYP_EN_MSK },
			[ADMV1355_DEV_ATTR_GAIN_LOAD_EN] = {
				ADMV1355_REG_GAIN_LOAD_EN, BIT(0) },
		};

		ret = kstrtobool(buf, &bval);
		if (ret)
			return ret;
		ret = regmap_update_bits(priv->regmap,
					 bool_map[attr->id].reg,
					 bool_map[attr->id].mask,
					 bval ? bool_map[attr->id].mask : 0);
		break;
	}
	case ADMV1355_DEV_ATTR_FILTER_TABLE_SEL:
		if (sysfs_streq(buf, "A"))
			val = 0;
		else if (sysfs_streq(buf, "B"))
			val = 1;
		else
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_FILTER_TBL_SEL,
					 BIT(0), val);
		break;
	case ADMV1355_DEV_ATTR_DIRECT_GPO_G:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GPO_G_DIRECT,
					 ADMV1355_GPO_G_DIRECT_MSK,
					 FIELD_PREP(ADMV1355_GPO_G_DIRECT_MSK,
						    val));
		break;
	case ADMV1355_DEV_ATTR_BYPASS_GPO_G:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GPO_G_BYPASS,
					 ADMV1355_GPO_G_BYPASS_MSK,
					 FIELD_PREP(ADMV1355_GPO_G_BYPASS_MSK,
						    val));
		break;
	case ADMV1355_DEV_ATTR_DIRECT_GPO_F:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 63)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GPO_F_DIRECT,
					 ADMV1355_GPO_F_DIRECT_MSK,
					 FIELD_PREP(ADMV1355_GPO_F_DIRECT_MSK,
						    val));
		break;
	case ADMV1355_DEV_ATTR_GPO_F_OE:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 63)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GPO_F_OE,
					 ADMV1355_GPO_F_OE_MSK,
					 FIELD_PREP(ADMV1355_GPO_F_OE_MSK,
						    val));
		break;
	case ADMV1355_DEV_ATTR_GPO_G_OE:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 31)
			return -EINVAL;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GPO_F_OE,
					 ADMV1355_GPO_G_OE_LO_MSK,
					 FIELD_PREP(ADMV1355_GPO_G_OE_LO_MSK,
						    val & 0x3));
		if (ret)
			break;
		ret = regmap_update_bits(priv->regmap,
					 ADMV1355_REG_GPO_GF_OE,
					 ADMV1355_GPO_G_OE_HI_MSK,
					 FIELD_PREP(ADMV1355_GPO_G_OE_HI_MSK,
						    (val >> 2) & 0x7));
		break;
	case ADMV1355_DEV_ATTR_FILTER_SM_STEP:
		ret = kstrtouint(buf, 0, &val);
		if (ret)
			return ret;
		if (val > 255)
			return -EINVAL;
		ret = regmap_write(priv->regmap, ADMV1355_REG_FILTER_SM_STEP,
				   val);
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

#define ADMV1355_ATTR(_name, _id, _mode) \
	struct admv1355_attribute dev_attr_##_name = { \
		.attr = __ATTR(_name, _mode, admv1355_dev_attr_show, \
			       admv1355_dev_attr_store), \
		.id = _id, \
	}

static ADMV1355_ATTR(filter_lut_en, ADMV1355_DEV_ATTR_FILTER_LUT_EN, 0644);
static ADMV1355_ATTR(filter_load_en, ADMV1355_DEV_ATTR_FILTER_LOAD_EN, 0644);
static ADMV1355_ATTR(filter_table_sel, ADMV1355_DEV_ATTR_FILTER_TABLE_SEL,
		     0644);
static ADMV1355_ATTR(gain_lut_en, ADMV1355_DEV_ATTR_GAIN_LUT_EN, 0644);
static ADMV1355_ATTR(gain_lut_bypass_en, ADMV1355_DEV_ATTR_GAIN_LUT_BYPASS_EN,
		     0644);
static ADMV1355_ATTR(gain_load_en, ADMV1355_DEV_ATTR_GAIN_LOAD_EN, 0644);
static ADMV1355_ATTR(direct_gpo_g, ADMV1355_DEV_ATTR_DIRECT_GPO_G, 0644);
static ADMV1355_ATTR(bypass_gpo_g, ADMV1355_DEV_ATTR_BYPASS_GPO_G, 0644);
static ADMV1355_ATTR(direct_gpo_f, ADMV1355_DEV_ATTR_DIRECT_GPO_F, 0644);
static ADMV1355_ATTR(gpo_f_oe, ADMV1355_DEV_ATTR_GPO_F_OE, 0644);
static ADMV1355_ATTR(gpo_g_oe, ADMV1355_DEV_ATTR_GPO_G_OE, 0644);
static ADMV1355_ATTR(filter_sm_step, ADMV1355_DEV_ATTR_FILTER_SM_STEP, 0644);

/*
 * Filter LUT binary attributes
 *
 * Each entry is 13 registers matching the direct register layout (0x800-0x80C):
 *   +0: LO_X4_FILTER[3:0]
 *   +1: LO_X3_FILTER[4:0]
 *   +2: LO_X1_FILTER[4:0]
 *   +3: GPO_F[5:0]
 *   +4: LO_TRAP[7:0]
 *   +5: RF_BAND[7] | LO_PHASE_Q[6:2] | LO_TRAP[9:8]
 *   +6: LO_BAND[7] | MIXER_BAND[6] | LO_SIDEBAND[5] | LO_PHASE_I[4:0]
 *   +7: (undocumented, always 0)
 *   +8: LON_OFFSET_I[7:0]
 *   +9: LON_OFFSET_Q[7:0]
 *  +10: DSAQ[7:4] | DSAI[3:0]
 *  +11: HPF[7:4] | LPF[3:0]
 *  +12: LO_DOUBLER_BAND[7:0]
 *
 * Text format per line (bitfield with enumerations):
 *   FLUT_<page>_<idx> RF (<band>,<lpf>,<hpf>) IF (<dsai>,<dsaq>) \
 *     LO (<x4>,<x3>,<x1>,<trap>,<lo_band>,<mixer_band>,<sideband>, \
 *         <phase_i>,<phase_q>,<lon_i>,<lon_q>,<dbl_band>,<gpo_f>)
 *
 * Enum fields: band={low_band,high_band}, dsai/dsaq={0dB,...,-1500mdB},
 *              sideband={LSB,USB}. All others are numeric.
 * Lines starting with '#' are comments.
 */
static int admv1355_find_enum_item(const char *str,
				   const char * const *items, int num_items)
{
	int i;

	for (i = 0; i < num_items; i++) {
		if (strcmp(str, items[i]) == 0)
			return i;
	}

	return -EINVAL;
}

static const char *admv1355_filter_reg_to_name(u8 reg_val,
					       const u8 *reg_values,
					       const char * const *items,
					       int num_items)
{
	int i;

	for (i = 0; i < num_items; i++) {
		if (reg_values[i] == reg_val)
			return items[i];
	}

	return NULL;
}

static int admv1355_filter_name_to_reg(const char *name,
				       const u8 *reg_values,
				       const char * const *items,
				       int num_items)
{
	int i;

	for (i = 0; i < num_items; i++) {
		if (strcmp(name, items[i]) == 0)
			return reg_values[i];
	}

	return -EINVAL;
}

static int admv1355_parse_filter_lut(struct admv1355_priv *priv, char *data,
				     size_t size, unsigned int base_reg)
{
	char *line, *ptr = data;
	char band_str[16], dsai_str[16], dsaq_str[16], sb_str[4];
	char x4_str[16], x3_str[16], x1_str[16];
	unsigned int idx, entry_base;
	unsigned int lpf, hpf, gpo_f;
	unsigned int lo_band, mixer_band, phase_i, phase_q;
	unsigned int lon_i, lon_q, dbl_band;
	int band_idx, dsai_idx, dsaq_idx, sideband;
	int x4_reg, x3_reg, x1_reg;
	unsigned int trap;
	int ret;

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= data + size)
			break;
		if (line[0] == '\0' || line[0] == '#')
			continue;

		if (strncmp(line, "FLUT_", 5) != 0 || line[6] != '_')
			return -EINVAL;

		ret = sscanf(line + 7,
			     "%u RF (%15[^,],%u,%u) "
			     "IF (%15[^,],%15[^)]) "
			     "LO (%15[^,],%15[^,],%15[^,],%u,%u,%u,%3[^,],%u,%u,%u,%u,%u,%u)",
			     &idx,
			     band_str, &lpf, &hpf,
			     dsai_str, dsaq_str,
			     x4_str, x3_str, x1_str,
			     &trap, &lo_band, &mixer_band,
			     sb_str, &phase_i, &phase_q,
			     &lon_i, &lon_q, &dbl_band, &gpo_f);
		if (ret != 19)
			return -EINVAL;

		if (idx >= ADMV1355_FILTER_LUT_NUM_ENTRIES)
			return -EINVAL;

		band_idx = admv1355_find_enum_item(band_str,
				admv1355_rf_band_items,
				ARRAY_SIZE(admv1355_rf_band_items));
		dsai_idx = admv1355_find_enum_item(dsai_str,
				admv1355_dsa_iq_items,
				ARRAY_SIZE(admv1355_dsa_iq_items));
		dsaq_idx = admv1355_find_enum_item(dsaq_str,
				admv1355_dsa_iq_items,
				ARRAY_SIZE(admv1355_dsa_iq_items));
		sideband = admv1355_find_enum_item(sb_str,
				admv1355_mixer_sideband_items,
				ARRAY_SIZE(admv1355_mixer_sideband_items));
		x4_reg = admv1355_filter_name_to_reg(x4_str,
				admv1355_lo_x4_filter_reg_values,
				admv1355_lo_x4_filter_items,
				ARRAY_SIZE(admv1355_lo_x4_filter_items));
		x3_reg = admv1355_filter_name_to_reg(x3_str,
				admv1355_lo_x3_filter_reg_values,
				admv1355_lo_x3_filter_items,
				ARRAY_SIZE(admv1355_lo_x3_filter_items));
		x1_reg = admv1355_filter_name_to_reg(x1_str,
				admv1355_lo_x1_filter_reg_values,
				admv1355_lo_x1_filter_items,
				ARRAY_SIZE(admv1355_lo_x1_filter_items));

		if (band_idx < 0 || dsai_idx < 0 || dsaq_idx < 0 ||
		    sideband < 0 || x4_reg < 0 || x3_reg < 0 || x1_reg < 0 ||
		    lpf > 15 || hpf > 15 || trap > 1023 || gpo_f > 63 ||
		    lo_band > 1 || mixer_band > 1 || phase_i > 31 ||
		    phase_q > 31 || lon_i > 255 || lon_q > 255 ||
		    dbl_band > 255)
			return -EINVAL;

		entry_base = base_reg + idx * ADMV1355_FILTER_LUT_ENTRY_SIZE;

		/* +0: LO_X4_FILTER[3:0] */
		ret = regmap_write(priv->regmap, entry_base + 0,
				   FIELD_PREP(ADMV1355_LO_X4_FILTER_MSK, x4_reg));
		if (ret)
			return ret;

		/* +1: LO_X3_FILTER[4:0] */
		ret = regmap_write(priv->regmap, entry_base + 1,
				   FIELD_PREP(ADMV1355_LO_X3_FILTER_MSK, x3_reg));
		if (ret)
			return ret;

		/* +2: LO_X1_FILTER[4:0] */
		ret = regmap_write(priv->regmap, entry_base + 2,
				   FIELD_PREP(ADMV1355_LO_X1_FILTER_MSK, x1_reg));
		if (ret)
			return ret;

		/* +3: GPO_F[5:0] */
		ret = regmap_write(priv->regmap, entry_base + 3,
				   FIELD_PREP(ADMV1355_GPO_F_DIRECT_MSK, gpo_f));
		if (ret)
			return ret;

		/* +4: LO_TRAP[7:0] */
		ret = regmap_write(priv->regmap, entry_base + 4,
				   trap & 0xFF);
		if (ret)
			return ret;

		/* +5: RF_BAND[7] | LO_PHASE_Q[6:2] | LO_TRAP[9:8] */
		ret = regmap_write(priv->regmap, entry_base + 5,
				   FIELD_PREP(ADMV1355_RF_BAND_805_MSK, band_idx) |
				   FIELD_PREP(ADMV1355_LO_PHASE_Q_MSK, phase_q) |
				   FIELD_PREP(ADMV1355_LO_TRAP_9_8_MSK,
					      (trap >> 8) & 0x03));
		if (ret)
			return ret;

		/* +6: LO_BAND[7] | MIXER_BAND[6] | LO_SIDEBAND[5] | LO_PHASE_I[4:0] */
		ret = regmap_write(priv->regmap, entry_base + 6,
				   FIELD_PREP(ADMV1355_LO_BAND_806_MSK, lo_band) |
				   FIELD_PREP(ADMV1355_MIXER_BAND_806_MSK, mixer_band) |
				   FIELD_PREP(ADMV1355_LO_SIDEBAND_MSK, sideband) |
				   FIELD_PREP(ADMV1355_LO_PHASE_I_MSK, phase_i));
		if (ret)
			return ret;

		/* +7: undocumented register, write 0 */
		ret = regmap_write(priv->regmap, entry_base + 7, 0x00);
		if (ret)
			return ret;

		/* +8: LON_OFFSET_I[7:0] */
		ret = regmap_write(priv->regmap, entry_base + 8, lon_i);
		if (ret)
			return ret;

		/* +9: LON_OFFSET_Q[7:0] */
		ret = regmap_write(priv->regmap, entry_base + 9, lon_q);
		if (ret)
			return ret;

		/* +10: DSAQ[7:4] | DSAI[3:0] */
		ret = regmap_write(priv->regmap, entry_base + 10,
				   FIELD_PREP(ADMV1355_DSAQ_MSK, dsaq_idx) |
				   FIELD_PREP(ADMV1355_DSAI_MSK, dsai_idx));
		if (ret)
			return ret;

		/* +11: HPF[7:4] | LPF[3:0] */
		ret = regmap_write(priv->regmap, entry_base + 11,
				   FIELD_PREP(ADMV1355_DIRECT_HPF_MSK, hpf) |
				   FIELD_PREP(ADMV1355_DIRECT_LPF_MSK, lpf));
		if (ret)
			return ret;

		/* +12: LO_DOUBLER_BAND[7:0] */
		ret = regmap_write(priv->regmap, entry_base + 12, dbl_band);
		if (ret)
			return ret;
	}

	return 0;
}

static ssize_t admv1355_filter_lut_write(struct file *filp,
					 struct kobject *kobj,
					 struct bin_attribute *bin_attr,
					 char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int base_reg;
	int ret;

	if (strcmp(bin_attr->attr.name, "filter_table_config_A") == 0)
		base_reg = ADMV1355_REG_FILTER_LUTA_BASE;
	else
		base_reg = ADMV1355_REG_FILTER_LUTB_BASE;

	guard(mutex)(&priv->lock);

	ret = admv1355_parse_filter_lut(priv, buf, count, base_reg);
	if (ret)
		return ret;

	return count;
}

static ssize_t admv1355_filter_lut_read(struct file *filp,
					struct kobject *kobj,
					struct bin_attribute *bin_attr,
					char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int base_reg, entry_base;
	unsigned int r0, r1, r2, r3, r4, r5, r6, r8, r9, r10, r11, r12;
	unsigned int rf_band, sideband, dsai_val, dsaq_val, trap;
	int ret, len = 0;
	unsigned int i;
	char page;

	if (off)
		return 0;

	if (strcmp(bin_attr->attr.name, "filter_table_config_A") == 0) {
		base_reg = ADMV1355_REG_FILTER_LUTA_BASE;
		page = 'A';
	} else {
		base_reg = ADMV1355_REG_FILTER_LUTB_BASE;
		page = 'B';
	}

	guard(mutex)(&priv->lock);

	len += scnprintf(buf + len, count - len,
		"# RF (band,lpf,hpf) IF (dsai,dsaq) "
		"LO (x4,x3,x1,trap,lo_band,mixer_band,sideband,"
		"phase_i,phase_q,lon_i,lon_q,dbl_band,gpo_f)\n");

	for (i = 0; i < ADMV1355_FILTER_LUT_NUM_ENTRIES; i++) {
		entry_base = base_reg + i * ADMV1355_FILTER_LUT_ENTRY_SIZE;

		ret = regmap_read(priv->regmap, entry_base + 0, &r0);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 1, &r1);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 2, &r2);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 3, &r3);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 4, &r4);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 5, &r5);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 6, &r6);
		if (ret)
			return ret;
		/* +7 is undocumented — skip */
		ret = regmap_read(priv->regmap, entry_base + 8, &r8);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 9, &r9);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 10, &r10);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 11, &r11);
		if (ret)
			return ret;
		ret = regmap_read(priv->regmap, entry_base + 12, &r12);
		if (ret)
			return ret;

		rf_band = FIELD_GET(ADMV1355_RF_BAND_805_MSK, r5);
		if (rf_band >= ARRAY_SIZE(admv1355_rf_band_items))
			rf_band = 0;

		sideband = FIELD_GET(ADMV1355_LO_SIDEBAND_MSK, r6);

		dsai_val = FIELD_GET(ADMV1355_DSAI_MSK, r10);
		if (dsai_val >= ARRAY_SIZE(admv1355_dsa_iq_items))
			dsai_val = 0;
		dsaq_val = FIELD_GET(ADMV1355_DSAQ_MSK, r10);
		if (dsaq_val >= ARRAY_SIZE(admv1355_dsa_iq_items))
			dsaq_val = 0;

		trap = r4 | (FIELD_GET(ADMV1355_LO_TRAP_9_8_MSK, r5) << 8);

		{
			const char *x4_name, *x3_name, *x1_name;

			x4_name = admv1355_filter_reg_to_name(
				FIELD_GET(ADMV1355_LO_X4_FILTER_MSK, r0),
				admv1355_lo_x4_filter_reg_values,
				admv1355_lo_x4_filter_items,
				ARRAY_SIZE(admv1355_lo_x4_filter_items));
			x3_name = admv1355_filter_reg_to_name(
				FIELD_GET(ADMV1355_LO_X3_FILTER_MSK, r1),
				admv1355_lo_x3_filter_reg_values,
				admv1355_lo_x3_filter_items,
				ARRAY_SIZE(admv1355_lo_x3_filter_items));
			x1_name = admv1355_filter_reg_to_name(
				FIELD_GET(ADMV1355_LO_X1_FILTER_MSK, r2),
				admv1355_lo_x1_filter_reg_values,
				admv1355_lo_x1_filter_items,
				ARRAY_SIZE(admv1355_lo_x1_filter_items));

			len += scnprintf(buf + len, count - len,
				"FLUT_%c_%u RF (%s,%lu,%lu) "
				"IF (%s,%s) "
				"LO (%s,%s,%s,%u,%lu,%lu,%s,%lu,%lu,%u,%u,%u,%lu)\n",
				page, i,
				admv1355_rf_band_items[rf_band],
				FIELD_GET(ADMV1355_DIRECT_LPF_MSK, r11),
				FIELD_GET(ADMV1355_DIRECT_HPF_MSK, r11),
				admv1355_dsa_iq_items[dsai_val],
				admv1355_dsa_iq_items[dsaq_val],
				x4_name ? x4_name : "unknown",
				x3_name ? x3_name : "unknown",
				x1_name ? x1_name : "unknown",
				trap,
				FIELD_GET(ADMV1355_LO_BAND_806_MSK, r6),
				FIELD_GET(ADMV1355_MIXER_BAND_806_MSK, r6),
				admv1355_mixer_sideband_items[sideband],
				FIELD_GET(ADMV1355_LO_PHASE_I_MSK, r6),
				FIELD_GET(ADMV1355_LO_PHASE_Q_MSK, r5),
				r8, r9, r12,
				FIELD_GET(ADMV1355_GPO_F_DIRECT_MSK, r3));
		}
	}

	return len;
}

static struct bin_attribute bin_attr_filter_table_config_A = {
	.attr = { .name = "filter_table_config_A", .mode = 0644 },
	.read = admv1355_filter_lut_read,
	.write = admv1355_filter_lut_write,
	.size = ADMV1355_FILTER_LUT_CONFIG_SIZE,
};

static struct bin_attribute bin_attr_filter_table_config_B = {
	.attr = { .name = "filter_table_config_B", .mode = 0644 },
	.read = admv1355_filter_lut_read,
	.write = admv1355_filter_lut_write,
	.size = ADMV1355_FILTER_LUT_CONFIG_SIZE,
};

/*
 * Gain LUT binary attribute
 *
 * Each entry is 2 registers:
 *   +0: DSA2[7:4] | DSA1[3:0]
 *   +1: GPO_G[4:0]
 *
 * Text format per line:
 *   GLUT_<idx> RF (<dsa1>,<dsa2>) <gpo_g>
 */
static ssize_t admv1355_gain_lut_write(struct file *filp,
				       struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct admv1355_priv *priv = iio_priv(indio_dev);
	char dsa1_str[32], dsa2_str[32];
	unsigned int idx, entry_base, gpo_g;
	int dsa1_idx, dsa2_idx;
	char *line, *ptr = buf;
	int ret;

	guard(mutex)(&priv->lock);

	while ((line = strsep(&ptr, "\n"))) {
		if (line >= buf + count)
			break;
		if (line[0] == '\0' || line[0] == '#')
			continue;

		ret = sscanf(line, "GLUT_%u RF (%31[^,],%31[^)]) %u",
			     &idx, dsa1_str, dsa2_str, &gpo_g);
		if (ret != 4)
			return -EINVAL;

		if (idx >= ADMV1355_GAIN_LUT_NUM_ENTRIES)
			return -EINVAL;

		dsa1_idx = admv1355_find_enum_item(dsa1_str,
				admv1355_dsa_4bit_items,
				ARRAY_SIZE(admv1355_dsa_4bit_items));
		dsa2_idx = admv1355_find_enum_item(dsa2_str,
				admv1355_dsa_4bit_items,
				ARRAY_SIZE(admv1355_dsa_4bit_items));

		if (dsa1_idx < 0 || dsa2_idx < 0 || gpo_g > 31)
			return -EINVAL;

		entry_base = ADMV1355_REG_GAIN_LUT_BASE +
			     idx * ADMV1355_GAIN_LUT_ENTRY_SIZE;

		ret = regmap_write(priv->regmap, entry_base,
				   FIELD_PREP(ADMV1355_DSA2_DIRECT_MSK, dsa2_idx) |
				   FIELD_PREP(ADMV1355_DSA1_DIRECT_MSK, dsa1_idx));
		if (ret)
			return ret;

		ret = regmap_write(priv->regmap, entry_base + 1,
				   gpo_g & 0x1F);
		if (ret)
			return ret;
	}

	return count;
}

static ssize_t admv1355_gain_lut_read(struct file *filp,
				      struct kobject *kobj,
				      struct bin_attribute *bin_attr,
				      char *buf, loff_t off, size_t count)
{
	struct iio_dev *indio_dev = dev_to_iio_dev(kobj_to_dev(kobj));
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int entry_base, r0, r1;
	unsigned int dsa1_val, dsa2_val;
	int ret, len = 0;
	unsigned int i;

	if (off)
		return 0;

	guard(mutex)(&priv->lock);

	len += scnprintf(buf + len, count - len,
		"# RF (dsa1,dsa2) gpo_g\n");

	for (i = 0; i < ADMV1355_GAIN_LUT_NUM_ENTRIES; i++) {
		entry_base = ADMV1355_REG_GAIN_LUT_BASE +
			     i * ADMV1355_GAIN_LUT_ENTRY_SIZE;

		ret = regmap_read(priv->regmap, entry_base, &r0);
		if (ret)
			return ret;

		ret = regmap_read(priv->regmap, entry_base + 1, &r1);
		if (ret)
			return ret;

		dsa1_val = FIELD_GET(ADMV1355_DSA1_DIRECT_MSK, r0);
		if (dsa1_val >= ARRAY_SIZE(admv1355_dsa_4bit_items))
			dsa1_val = 0;
		dsa2_val = FIELD_GET(ADMV1355_DSA2_DIRECT_MSK, r0);
		if (dsa2_val >= ARRAY_SIZE(admv1355_dsa_4bit_items))
			dsa2_val = 0;

		len += scnprintf(buf + len, count - len,
			"GLUT_%u RF (%s,%s) %lu\n",
			i,
			admv1355_dsa_4bit_items[dsa1_val],
			admv1355_dsa_4bit_items[dsa2_val],
			FIELD_GET(ADMV1355_GPO_G_DIRECT_MSK, r1));
	}

	return len;
}

static struct bin_attribute bin_attr_gain_table_config = {
	.attr = { .name = "gain_table_config", .mode = 0644 },
	.read = admv1355_gain_lut_read,
	.write = admv1355_gain_lut_write,
	.size = ADMV1355_GAIN_LUT_CONFIG_SIZE,
};

static struct bin_attribute *admv1355_bin_attributes[] = {
	&bin_attr_filter_table_config_A,
	&bin_attr_filter_table_config_B,
	&bin_attr_gain_table_config,
	NULL,
};

static struct attribute *admv1355_attributes[] = {
	&dev_attr_filter_lut_en.attr.attr,
	&dev_attr_filter_load_en.attr.attr,
	&dev_attr_filter_table_sel.attr.attr,
	&dev_attr_gain_lut_en.attr.attr,
	&dev_attr_gain_lut_bypass_en.attr.attr,
	&dev_attr_gain_load_en.attr.attr,
	&dev_attr_direct_gpo_g.attr.attr,
	&dev_attr_bypass_gpo_g.attr.attr,
	&dev_attr_direct_gpo_f.attr.attr,
	&dev_attr_gpo_f_oe.attr.attr,
	&dev_attr_gpo_g_oe.attr.attr,
	&dev_attr_filter_sm_step.attr.attr,
	NULL,
};

static const struct attribute_group admv1355_attribute_group = {
	.attrs = admv1355_attributes,
	.bin_attrs = admv1355_bin_attributes,
};

static const struct iio_chan_spec admv1355_channels[] = {
	/* IF input channel */
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 0,
		.extend_name = "if",
		.ext_info = admv1355_if_ext_info,
	},
	/* RF output channel */
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.extend_name = "rf",
		.ext_info = admv1355_rf_ext_info,
	},
	/* LO channel */
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 2,
		.extend_name = "lo",
		.ext_info = admv1355_lo_ext_info,
	},
	/* Temperature sensor */
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static const struct iio_info admv1355_info = {
	.read_raw = &admv1355_read_raw,
	.debugfs_reg_access = &admv1355_reg_access,
	.attrs = &admv1355_attribute_group,
};

static int admv1355_set_filters(struct admv1355_priv *priv, u64 rate_hz)
{
	struct device *dev = &priv->spi->dev;
	u64 rate_khz = div_u64(rate_hz, 1000);
	u8 lo_trap_filter_7_0;
	u8 lo_trap_filter_9_8;
	u8 lo_doubler_band;
	u16 lo_trap_filter;
	u8 lo_x4_filter;
	u8 lo_x3_filter;
	u8 lo_x1_filter;
	u8 lo_band;
	int ret;

	if (rate_khz < (10000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x08;
		lo_x3_filter = 0x1C;
		lo_x1_filter = 0x1F;
		lo_trap_filter = 0x318;
		lo_doubler_band = 0x1F;
	} else if (rate_khz < (12000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x00;
		lo_x3_filter = 0x1C;
		lo_x1_filter = 0x1F;
		lo_trap_filter = 0x318;
		lo_doubler_band = 0x1D;
	} else if (rate_khz < (14000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x00;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x3AA;
		lo_doubler_band = 0x04;
	} else if (rate_khz < (14500 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x3AA;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (15000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x07;
		lo_x1_filter = 0x0A;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (17000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x05;
		lo_x1_filter = 0x05;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (18000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x05;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x08;
	} else if (rate_khz < (18500 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A3;
		lo_doubler_band = 0x80;
	} else if (rate_khz < (19000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x04;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A7;
		lo_doubler_band = 0x80;
	} else if (rate_khz < (21000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x03;
		lo_x1_filter = 0x01;
		lo_trap_filter = 0x1A7;
		lo_doubler_band = 0x80;
	} else if (rate_khz < (24000 * HZ_PER_KHZ)) {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x02;
		lo_x1_filter = 0x00;
		lo_trap_filter = 0x1E7;
		lo_doubler_band = 0x80;
	} else {
		lo_x4_filter = 0x07;
		lo_x3_filter = 0x00;
		lo_x1_filter = 0x00;
		lo_trap_filter = 0x1E7;
		lo_doubler_band = 0xC0;
	}

	lo_trap_filter_7_0 = lo_trap_filter & 0xff;
	lo_trap_filter_9_8 = (lo_trap_filter >> 8) & 0x03;
	lo_band = (rate_khz >= (18000 * HZ_PER_KHZ)) ? 1 : 0;

	dev_dbg(dev, "%s: LO rate %llu Hz (%llu kHz), x4=0x%02X x3=0x%02X x1=0x%02X trap=0x%03X dbl=0x%02X band=%u\n",
		__func__, rate_hz, rate_khz, lo_x4_filter, lo_x3_filter,
		lo_x1_filter, lo_trap_filter, lo_doubler_band, lo_band);

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X4_FILTER, lo_x4_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X3_FILTER, lo_x3_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X1_FILTER, lo_x1_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_TRAP_7_0, lo_trap_filter_7_0);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				 0x03, lo_trap_filter_9_8);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_DBL_BAND, lo_doubler_band);
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, ADMV1355_REG_LO_BAND,
				  ADMV1355_LO_BAND_806_MSK,
				  FIELD_PREP(ADMV1355_LO_BAND_806_MSK,
					     lo_band));
}

static int admv1355_freq_change(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct admv1355_priv *priv = container_of(nb, struct admv1355_priv, nb);
	u64 rate;
	int ret;

	if (action != POST_RATE_CHANGE)
		return NOTIFY_OK;

	guard(mutex)(&priv->lock);

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);
	ret = admv1355_set_filters(priv, rate);
	if (ret)
		dev_err(&priv->spi->dev, "%s: failed for LO rate %llu (%d)\n",
			__func__, rate, ret);

	return notifier_from_errno(ret);
}

static int admv1355_spi_verify(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	unsigned int scratch_rd;
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_SCRATCH_PAD,
				 ADMV1355_SCRATCH_TEST_VAL);
	if (ret)
		return ret;

	ret = admv1355_spi_read(priv, ADMV1355_REG_SCRATCH_PAD, &scratch_rd);
	if (ret)
		return ret;

	if (scratch_rd != ADMV1355_SCRATCH_TEST_VAL) {
		dev_err(dev, "%s: mismatch: wrote 0x%02X, read 0x%02X\n",
			__func__, ADMV1355_SCRATCH_TEST_VAL, scratch_rd);
		return -EIO;
	}

	dev_dbg(dev, "SPI verify OK (scratchpad 0x%02X)\n",
		 ADMV1355_SCRATCH_TEST_VAL);

	return 0;
}

static int admv1355_nvm_load(struct admv1355_priv *priv)
{
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_CTRL, 0x08);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_LOAD, 0x40);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_LOAD, 0x54);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	return admv1355_spi_write(priv, ADMV1355_REG_NVM_CTRL, 0x09);
}

static int admv1355_nvm_init(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	static const u8 shadow_regs[] = { 0x33, 0x34, 0x37, 0x38 };
	unsigned int addr_rd, data_rd;
	int ret, i;

	ret = admv1355_nvm_load(priv);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(shadow_regs); i++) {
		ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_ADDR,
					 shadow_regs[i]);
		if (ret)
			return ret;

		ret = admv1355_spi_read(priv, ADMV1355_REG_NVM_ADDR, &addr_rd);
		if (ret)
			return ret;

		if (addr_rd != shadow_regs[i]) {
			dev_err(dev, "%s: NVM addr 0x%02X readback mismatch: 0x%02X\n",
				__func__, shadow_regs[i], addr_rd);
			return -EIO;
		}

		ret = admv1355_spi_read(priv, ADMV1355_REG_NVM_DATA, &data_rd);
		if (ret)
			return ret;

		if (data_rd == 0) {
			dev_err(dev, "%s: NVM shadow 0x%02X still zero after load\n",
				__func__, shadow_regs[i]);
			return -EIO;
		}
	}

	dev_dbg(dev, "NVM loaded and verified\n");

	return 0;
}

static int admv1355_setup(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	unsigned int product_id;
	int ret;

	/* Soft reset to ensure known state (self-clearing) */
	ret = admv1355_spi_write(priv, ADMV1355_REG_SDO_CTRL, 0x81);
	if (ret)
		return ret;

	usleep_range(1000, 2000);

	/* Enable SDO for 4-wire SPI */
	ret = admv1355_spi_write(priv, ADMV1355_REG_SDO_CTRL, 0x18);
	if (ret)
		return ret;

	ret = admv1355_spi_read(priv, ADMV1355_REG_PRODUCT_ID_LSB, &product_id);
	if (ret)
		return ret;

	dev_dbg(dev, "PRODUCT_ID: 0x%02X\n", product_id);

	ret = admv1355_spi_verify(priv);
	if (ret)
		return ret;

	ret = admv1355_nvm_init(priv);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HS_PD, 0x02);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_FILTER_LUT_EN, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_FILTER_LOAD_EN, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_GAIN_TBL_BYP, 0x01);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_DSA_BYPASS, 0x00);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_LPF, 0x10);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HPF, 0xBF);
	if (ret)
		return ret;

	return admv1355_spi_write(priv, ADMV1355_REG_GPO_G_DIRECT, 0x08);
}

static const struct regmap_config admv1355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_read = admv1355_spi_reg_read,
	.reg_write = admv1355_spi_reg_write,
};

static int admv1355_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct admv1355_priv *priv;
	u64 rate;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*priv));
	if (!indio_dev)
		return -ENOMEM;

	priv = iio_priv(indio_dev);
	priv->spi = spi;

	dev_dbg(dev, "probe start\n");

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						    GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset gpio\n");

	priv->cen_gpio = devm_gpiod_get_optional(dev, "chip-enable",
						  GPIOD_OUT_LOW);
	if (IS_ERR(priv->cen_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->cen_gpio),
				     "failed to get chip-enable gpio\n");

	ret = devm_regulator_get_enable_optional(dev, "vcc");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret,
				     "failed to enable vcc supply\n");
	if (ret != -ENODEV)
		msleep(100);


	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		usleep_range(1000, 2000);
		dev_dbg(dev, "reset deasserted\n");
	}

	priv->regmap = devm_regmap_init(dev, NULL, spi,
				       &admv1355_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	priv->lo_input = devm_clk_get_enabled(dev, "lo-input");
	if (IS_ERR(priv->lo_input))
		return dev_err_probe(dev, PTR_ERR(priv->lo_input),
				     "failed to get the LO input clock\n");

	ret = of_clk_get_scale(spi->dev.of_node, NULL, &priv->clkscale);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get clock scale\n");

	priv->nb.notifier_call = admv1355_freq_change;
	ret = devm_clk_notifier_register(dev, priv->lo_input, &priv->nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to register clock notifier\n");

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	ret = admv1355_setup(priv);
	if (ret)
		return ret;

	rate = clk_get_rate_scaled(priv->lo_input, &priv->clkscale);
	ret = admv1355_set_filters(priv, rate);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set filters for LO rate %llu\n",
				     rate);

	if (priv->cen_gpio) {
		gpiod_set_value_cansleep(priv->cen_gpio, 1);
		dev_dbg(dev, "chip-enable asserted\n");
	}

	indio_dev->name = spi->dev.of_node->name;
	indio_dev->info = &admv1355_info;
	indio_dev->channels = admv1355_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv1355_channels);

	dev_info(dev, "successfully initialized, LO rate %llu Hz\n", rate);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct spi_device_id admv1355_id[] = {
	{ "admv1355", 0 },
	{}
};
MODULE_DEVICE_TABLE(spi, admv1355_id);

static const struct of_device_id admv1355_of_match[] = {
	{ .compatible = "adi,admv1355" },
	{}
};
MODULE_DEVICE_TABLE(of, admv1355_of_match);

static struct spi_driver admv1355_driver = {
	.driver = {
		.name = "admv1355",
		.of_match_table = admv1355_of_match,
	},
	.probe = admv1355_probe,
	.id_table = admv1355_id,
};
module_spi_driver(admv1355_driver);

MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV1355 Microwave Upconverter Driver");
MODULE_LICENSE("GPL v2");
