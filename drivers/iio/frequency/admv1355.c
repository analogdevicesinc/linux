// SPDX-License-Identifier: GPL-2.0
/*
 * ADMV1355 17.7 GHz to 55 GHz Wideband Microwave Upconverter
 *
 * Copyright (C) 2026 Analog Devices, Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
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
#define ADMV1355_REG_FILTER_LUT_EN	0x202
#define ADMV1355_REG_FILTER_LOAD_EN	0x208
#define ADMV1355_REG_GAIN_TBL_BYP	0x28A
#define ADMV1355_REG_RF_LPF		0x2A0
#define ADMV1355_REG_RF_HPF		0x2A1
#define ADMV1355_REG_ADC_MUX		0x2A7
#define ADMV1355_REG_ADC_CTRL		0x300
#define ADMV1355_REG_ADC_RESET		0x302
#define ADMV1355_REG_LON_CTRL		0x306
#define ADMV1355_REG_ADC_OUT		0x30D
#define ADMV1355_REG_DSA_DIRECT		0x600
#define ADMV1355_REG_GPO_G_DIRECT	0x601
#define ADMV1355_REG_IF_ENABLE		0x602
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

#define ADMV1355_DSA1_DIRECT_MSK	GENMASK(3, 0)
#define ADMV1355_DSA2_DIRECT_MSK	GENMASK(7, 4)

#define ADMV1355_IF_MODE_MSK		GENMASK(3, 0)

#define ADMV1355_RF_BAND_805_MSK	BIT(7)
#define ADMV1355_LO_PHASE_Q_MSK		GENMASK(6, 2)
#define ADMV1355_LO_TRAP_9_8_MSK	GENMASK(1, 0)

#define ADMV1355_LO_BAND_806_MSK	BIT(7)
#define ADMV1355_MIXER_BAND_806_MSK	BIT(6)
#define ADMV1355_LO_SIDEBAND_MSK	BIT(5)
#define ADMV1355_LO_PHASE_I_MSK		GENMASK(4, 0)

#define ADMV1355_DSAI_MSK		GENMASK(3, 0)
#define ADMV1355_DSAQ_MSK		GENMASK(7, 4)

#define ADMV1355_LO_X4_FILTER_MSK	GENMASK(3, 0)
#define ADMV1355_LO_X3_FILTER_MSK	GENMASK(4, 0)
#define ADMV1355_LO_X1_FILTER_MSK	GENMASK(4, 0)

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

enum admv1355_chan_addr {
	ADMV1355_CH_RF,
	ADMV1355_CH_DSA1,
	ADMV1355_CH_DSA2,
	ADMV1355_CH_IF,
	ADMV1355_CH_LO,
	ADMV1355_CH_LO_I,
	ADMV1355_CH_LO_Q,
};

static const char * const admv1355_chan_labels[] = {
	[ADMV1355_CH_RF] = "rf",
	[ADMV1355_CH_DSA1] = "dsa1",
	[ADMV1355_CH_DSA2] = "dsa2",
	[ADMV1355_CH_IF] = "if",
	[ADMV1355_CH_LO] = "lo",
	[ADMV1355_CH_LO_I] = "lo_i",
	[ADMV1355_CH_LO_Q] = "lo_q",
};

struct admv1355_priv {
	struct spi_device	*spi;
	struct gpio_desc	*cen_gpio;
	struct gpio_desc	*reset_gpio;
	struct regmap		*regmap;
	struct clk		*lo_input;
	struct notifier_block	nb;
	struct mutex		lock;
};

/*
 * The ADMV1355 uses a 3-byte full-duplex SPI frame. The default regmap SPI
 * bus uses spi_write_then_read() which splits the transaction and causes
 * stale read data.
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
	return regmap_read(priv->regmap, reg, val);
}

static int admv1355_spi_write(struct admv1355_priv *priv, unsigned int reg,
			      unsigned int val)
{
	return regmap_write(priv->regmap, reg, val);
}

static const struct regmap_config admv1355_regmap_config = {
	.reg_bits = 16,
	.val_bits = 8,
	.reg_read = admv1355_spi_reg_read,
	.reg_write = admv1355_spi_reg_write,
};

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

	fsleep(1000);

	ret = admv1355_spi_read(priv, ADMV1355_REG_ADC_OUT, &adc_code);
	if (ret)
		return ret;

	/* Datasheet Eq. 3: T(mC) = -(5/6) * code * 1000 + 173000 */
	*val = -5 * (int)adc_code * 1000 / 6 + 173000;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HS_PD, 0x02);
	if (ret)
		return ret;

	return admv1355_spi_write(priv, ADMV1355_REG_ADC_CTRL, 0x00);
}

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
	.num_items = ARRAY_SIZE(_items),						\
	.get = admv1355_get_##_name,						\
	.set = admv1355_set_##_name,						\
}

ADMV1355_REG_ENUM(mixer_sideband, ADMV1355_REG_LO_BAND,
		  ADMV1355_LO_SIDEBAND_MSK, admv1355_mixer_sideband_items);

/* RF band select — sets bits in two registers (0x805[7] and 0x806[6]) */
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

/* IF mode select — non-contiguous register values */
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

static const struct iio_chan_spec_ext_info admv1355_rf_ext_info[] = {
	IIO_ENUM("band", IIO_SEPARATE, &admv1355_rf_band_enum),
	IIO_ENUM_AVAILABLE("band", IIO_SEPARATE, &admv1355_rf_band_enum),
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
	{ }
};

static const struct iio_chan_spec admv1355_channels[] = {
	/* RF output — parent for DSA1/DSA2 gain channels */
	[ADMV1355_CH_RF] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 0,
		.ext_info = admv1355_rf_ext_info,
		.address = ADMV1355_CH_RF,
	},
	/* DSA1 — direct path attenuator, 0 to -15 dB in 1 dB steps */
	[ADMV1355_CH_DSA1] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.parent = &admv1355_channels[ADMV1355_CH_RF],
		.address = ADMV1355_CH_DSA1,
	},
	/* DSA2 — direct path attenuator, 0 to -15 dB in 1 dB steps */
	[ADMV1355_CH_DSA2] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = 2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN),
		.parent = &admv1355_channels[ADMV1355_CH_RF],
		.address = ADMV1355_CH_DSA2,
	},
	/* IF input */
	[ADMV1355_CH_IF] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 0,
		.ext_info = admv1355_if_ext_info,
		.address = ADMV1355_CH_IF,
	},
	/* LO — parent for I/Q path channels */
	[ADMV1355_CH_LO] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 1,
		.ext_info = admv1355_lo_ext_info,
		.address = ADMV1355_CH_LO,
	},
	/* LO I-path — phase, gain adjust, LO null offset */
	[ADMV1355_CH_LO_I] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 2,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS),
		.parent = &admv1355_channels[ADMV1355_CH_LO],
		.address = ADMV1355_CH_LO_I,
	},
	/* LO Q-path — phase, gain adjust, LO null offset */
	[ADMV1355_CH_LO_Q] = {
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 0,
		.channel = 3,
		.info_mask_separate = BIT(IIO_CHAN_INFO_HARDWAREGAIN) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_CALIBBIAS),
		.parent = &admv1355_channels[ADMV1355_CH_LO],
		.address = ADMV1355_CH_LO_Q,
	},
	/* On-chip temperature sensor */
	{
		.type = IIO_TEMP,
		.indexed = 1,
		.channel = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
	},
};

static int admv1355_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	unsigned int data, reg_val;
	int ret;

	guard(mutex)(&priv->lock);

	switch (mask) {
	case IIO_CHAN_INFO_PROCESSED:
		if (chan->type != IIO_TEMP)
			return -EINVAL;
		ret = admv1355_read_temp(priv, val);
		return ret ? ret : IIO_VAL_INT;

	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (chan->address) {
		case ADMV1355_CH_DSA1:
			ret = regmap_read(priv->regmap, ADMV1355_REG_DSA_DIRECT,
					  &data);
			if (ret)
				return ret;
			*val = -(int)FIELD_GET(ADMV1355_DSA1_DIRECT_MSK, data);
			return IIO_VAL_INT;
		case ADMV1355_CH_DSA2:
			ret = regmap_read(priv->regmap, ADMV1355_REG_DSA_DIRECT,
					  &data);
			if (ret)
				return ret;
			*val = -(int)FIELD_GET(ADMV1355_DSA2_DIRECT_MSK, data);
			return IIO_VAL_INT;
		case ADMV1355_CH_LO_I:
			ret = regmap_read(priv->regmap, ADMV1355_REG_IF_GAIN,
					  &data);
			if (ret)
				return ret;
			reg_val = FIELD_GET(ADMV1355_DSAI_MSK, data);
			*val = -(int)(reg_val / 10);
			*val2 = -(int)(reg_val % 10) * 100000;
			return IIO_VAL_INT_PLUS_MICRO;
		case ADMV1355_CH_LO_Q:
			ret = regmap_read(priv->regmap, ADMV1355_REG_IF_GAIN,
					  &data);
			if (ret)
				return ret;
			reg_val = FIELD_GET(ADMV1355_DSAQ_MSK, data);
			*val = -(int)(reg_val / 10);
			*val2 = -(int)(reg_val % 10) * 100000;
			return IIO_VAL_INT_PLUS_MICRO;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PHASE:
		switch (chan->address) {
		case ADMV1355_CH_LO_I:
			ret = regmap_read(priv->regmap, ADMV1355_REG_LO_BAND,
					  &data);
			if (ret)
				return ret;
			*val = FIELD_GET(ADMV1355_LO_PHASE_I_MSK, data);
			return IIO_VAL_INT;
		case ADMV1355_CH_LO_Q:
			ret = regmap_read(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
					  &data);
			if (ret)
				return ret;
			*val = FIELD_GET(ADMV1355_LO_PHASE_Q_MSK, data);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		switch (chan->address) {
		case ADMV1355_CH_LO_I:
			ret = regmap_read(priv->regmap,
					  ADMV1355_REG_LON_OFFSET_I, &data);
			if (ret)
				return ret;
			*val = data;
			return IIO_VAL_INT;
		case ADMV1355_CH_LO_Q:
			ret = regmap_read(priv->regmap,
					  ADMV1355_REG_LON_OFFSET_Q, &data);
			if (ret)
				return ret;
			*val = data;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int admv1355_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct admv1355_priv *priv = iio_priv(indio_dev);
	int reg_val;

	guard(mutex)(&priv->lock);

	switch (mask) {
	case IIO_CHAN_INFO_HARDWAREGAIN:
		switch (chan->address) {
		case ADMV1355_CH_DSA1:
			if (val2 != 0 || val > 0 || val < -15)
				return -EINVAL;
			return regmap_update_bits(priv->regmap,
						 ADMV1355_REG_DSA_DIRECT,
						 ADMV1355_DSA1_DIRECT_MSK,
						 FIELD_PREP(ADMV1355_DSA1_DIRECT_MSK,
							    -val));
		case ADMV1355_CH_DSA2:
			if (val2 != 0 || val > 0 || val < -15)
				return -EINVAL;
			return regmap_update_bits(priv->regmap,
						 ADMV1355_REG_DSA_DIRECT,
						 ADMV1355_DSA2_DIRECT_MSK,
						 FIELD_PREP(ADMV1355_DSA2_DIRECT_MSK,
							    -val));
		case ADMV1355_CH_LO_I:
			if (val > 0 || (val == 0 && val2 > 0))
				return -EINVAL;
			reg_val = -val * 10 + abs(val2) / 100000;
			if (reg_val > 15)
				return -EINVAL;
			return regmap_update_bits(priv->regmap,
						 ADMV1355_REG_IF_GAIN,
						 ADMV1355_DSAI_MSK,
						 FIELD_PREP(ADMV1355_DSAI_MSK,
							    reg_val));
		case ADMV1355_CH_LO_Q:
			if (val > 0 || (val == 0 && val2 > 0))
				return -EINVAL;
			reg_val = -val * 10 + abs(val2) / 100000;
			if (reg_val > 15)
				return -EINVAL;
			return regmap_update_bits(priv->regmap,
						 ADMV1355_REG_IF_GAIN,
						 ADMV1355_DSAQ_MSK,
						 FIELD_PREP(ADMV1355_DSAQ_MSK,
							    reg_val));
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_PHASE:
		if (val < 0 || val > 31)
			return -EINVAL;
		switch (chan->address) {
		case ADMV1355_CH_LO_I:
			return regmap_update_bits(priv->regmap,
						 ADMV1355_REG_LO_BAND,
						 ADMV1355_LO_PHASE_I_MSK,
						 FIELD_PREP(ADMV1355_LO_PHASE_I_MSK,
							    val));
		case ADMV1355_CH_LO_Q:
			return regmap_update_bits(priv->regmap,
						 ADMV1355_REG_LO_TRAP_9_8,
						 ADMV1355_LO_PHASE_Q_MSK,
						 FIELD_PREP(ADMV1355_LO_PHASE_Q_MSK,
							    val));
		default:
			return -EINVAL;
		}

	case IIO_CHAN_INFO_CALIBBIAS:
		if (val < 0 || val > 255)
			return -EINVAL;
		switch (chan->address) {
		case ADMV1355_CH_LO_I:
			return regmap_write(priv->regmap,
					   ADMV1355_REG_LON_OFFSET_I, val);
		case ADMV1355_CH_LO_Q:
			return regmap_write(priv->regmap,
					   ADMV1355_REG_LON_OFFSET_Q, val);
		default:
			return -EINVAL;
		}

	default:
		return -EINVAL;
	}
}

static int admv1355_read_label(struct iio_dev *indio_dev,
			       const struct iio_chan_spec *chan,
			       char *label)
{
	if (chan->type == IIO_TEMP)
		return sprintf(label, "temp\n");

	return sprintf(label, "%s\n", admv1355_chan_labels[chan->address]);
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

static const struct iio_info admv1355_info = {
	.read_raw = admv1355_read_raw,
	.write_raw = admv1355_write_raw,
	.read_label = admv1355_read_label,
	.debugfs_reg_access = admv1355_reg_access,
};

static int admv1355_set_filters(struct admv1355_priv *priv, u64 rate_hz)
{
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

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X4_FILTER, lo_x4_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X3_FILTER, lo_x3_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_X1_FILTER, lo_x1_filter);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_TRAP_7_0,
				 lo_trap_filter_7_0);
	if (ret)
		return ret;

	ret = regmap_update_bits(priv->regmap, ADMV1355_REG_LO_TRAP_9_8,
				 ADMV1355_LO_TRAP_9_8_MSK, lo_trap_filter_9_8);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_LO_DBL_BAND,
				 lo_doubler_band);
	if (ret)
		return ret;

	return regmap_update_bits(priv->regmap, ADMV1355_REG_LO_BAND,
				  ADMV1355_LO_BAND_806_MSK,
				  FIELD_PREP(ADMV1355_LO_BAND_806_MSK, lo_band));
}

static int admv1355_freq_change(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct admv1355_priv *priv = container_of(nb, struct admv1355_priv, nb);
	int ret;

	if (action != POST_RATE_CHANGE)
		return NOTIFY_OK;

	guard(mutex)(&priv->lock);

	ret = admv1355_set_filters(priv, clk_get_rate(priv->lo_input));
	if (ret)
		dev_err(&priv->spi->dev, "failed to update LO filters (%d)\n",
			ret);

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
		dev_err(dev, "SPI scratchpad mismatch: wrote 0x%02X, read 0x%02X\n",
			ADMV1355_SCRATCH_TEST_VAL, scratch_rd);
		return -EIO;
	}

	return 0;
}

static int admv1355_nvm_load(struct admv1355_priv *priv)
{
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_CTRL, 0x08);
	if (ret)
		return ret;

	fsleep(1000);

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_LOAD, 0x40);
	if (ret)
		return ret;

	fsleep(1000);

	ret = admv1355_spi_write(priv, ADMV1355_REG_NVM_LOAD, 0x54);
	if (ret)
		return ret;

	fsleep(1000);

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
			dev_err(dev, "NVM addr 0x%02X readback mismatch: 0x%02X\n",
				shadow_regs[i], addr_rd);
			return -EIO;
		}

		ret = admv1355_spi_read(priv, ADMV1355_REG_NVM_DATA, &data_rd);
		if (ret)
			return ret;

		if (data_rd == 0) {
			dev_err(dev, "NVM shadow 0x%02X still zero after load\n",
				shadow_regs[i]);
			return -EIO;
		}
	}

	return 0;
}

static int admv1355_setup(struct admv1355_priv *priv)
{
	struct device *dev = &priv->spi->dev;
	unsigned int product_id;
	int ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_SDO_CTRL, 0x81);
	if (ret)
		return ret;

	fsleep(1000);

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

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_LPF, 0x10);
	if (ret)
		return ret;

	ret = admv1355_spi_write(priv, ADMV1355_REG_RF_HPF, 0xBF);
	if (ret)
		return ret;

	return admv1355_spi_write(priv, ADMV1355_REG_GPO_G_DIRECT, 0x08);
}

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

	priv->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(priv->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->reset_gpio),
				     "failed to get reset gpio\n");

	priv->cen_gpio = devm_gpiod_get_optional(dev, "chip-enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->cen_gpio))
		return dev_err_probe(dev, PTR_ERR(priv->cen_gpio),
				     "failed to get chip-enable gpio\n");

	ret = devm_regulator_get_enable(dev, "vcc");
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable vcc supply\n");

	msleep(100);

	if (priv->cen_gpio)
		gpiod_set_value_cansleep(priv->cen_gpio, 1);

	if (priv->reset_gpio) {
		gpiod_set_value_cansleep(priv->reset_gpio, 0);
		fsleep(1000);
	}

	priv->regmap = devm_regmap_init(dev, NULL, spi, &admv1355_regmap_config);
	if (IS_ERR(priv->regmap))
		return dev_err_probe(dev, PTR_ERR(priv->regmap),
				     "failed to init regmap\n");

	priv->lo_input = devm_clk_get_enabled(dev, "lo-input");
	if (IS_ERR(priv->lo_input))
		return dev_err_probe(dev, PTR_ERR(priv->lo_input),
				     "failed to get the LO input clock\n");

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

	rate = clk_get_rate(priv->lo_input);
	ret = admv1355_set_filters(priv, rate);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to set filters for LO rate %llu\n",
				     rate);

	indio_dev->name = "admv1355";
	indio_dev->info = &admv1355_info;
	indio_dev->channels = admv1355_channels;
	indio_dev->num_channels = ARRAY_SIZE(admv1355_channels);

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

MODULE_AUTHOR("George Mois <george.mois@analog.com>");
MODULE_AUTHOR("Dragos Bogdan <dragos.bogdan@analog.com>");
MODULE_DESCRIPTION("ADMV1355 Microwave Upconverter Driver");
MODULE_LICENSE("GPL");
