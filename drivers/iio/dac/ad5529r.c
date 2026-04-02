// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5529R Digital-to-Analog Converter Driver
 * 16-Channel, 12/16-Bit, 40V High Voltage Precision DAC
 *
 * Copyright 2026 Analog Devices Inc.
 * Author: Janani Sunil <janani.sunil@analog.com>
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/spi/spi.h>
#include <linux/errno.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/delay.h>
#include <linux/regulator/consumer.h>

/* Register Map */
#define AD5529R_REG_INTERFACE_CONFIG_A		0x00
#define AD5529R_REG_INTERFACE_CONFIG_B		0x01
#define AD5529R_REG_DEVICE_CONFIG		0x02
#define AD5529R_REG_CHIP_TYPE			0x03
#define AD5529R_REG_PRODUCT_ID_L		0x04
#define AD5529R_REG_PRODUCT_ID_H		0x05
#define AD5529R_REG_CHIP_GRADE			0x06
#define AD5529R_REG_SCRATCH_PAD			0x0A
#define AD5529R_REG_SPI_REVISION		0x0B
#define AD5529R_REG_VENDOR_L			0x0C
#define AD5529R_REG_VENDOR_H			0x0D
#define AD5529R_REG_STREAM_MODE			0x0E
#define AD5529R_REG_TRANSFER_CONFIG		0x0F
#define AD5529R_REG_INTERFACE_CONFIG_C		0x10
#define AD5529R_REG_INTERFACE_STATUS_A		0x11

/* Configuration registers */
#define AD5529R_REG_MULTI_DAC_CH_SEL		(0x14 + 1)
#define AD5529R_REG_LDAC_SYNC_ASYNC		(0x16 + 1)
#define AD5529R_REG_LDAC_HW_SW			(0x18 + 1)

/* Hardware LDAC source and edge select registers (per channel, 16-bit) */
#define AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE	(0x1A + 1)
#define AD5529R_REG_LDAC_HW_SRC_EDGE_SEL(ch)	\
	(AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE + (ch) * 2)

/* Output configuration */
#define AD5529R_REG_OUT_OPERATING_MODE		(0x3A + 1)
#define AD5529R_REG_OUT_RANGE_BASE		(0x3C + 1)
#define AD5529R_REG_OUT_RANGE(ch)		(AD5529R_REG_OUT_RANGE_BASE + (ch) * 2)

/* Calibration registers */
#define AD5529R_REG_CAL_GAIN_BASE		(0x5C + 1)
#define AD5529R_REG_CAL_GAIN(ch)		(AD5529R_REG_CAL_GAIN_BASE + (ch) * 2)

#define AD5529R_REG_CAL_OFFSET_BASE		(0x7C + 1)
#define AD5529R_REG_CAL_OFFSET(ch)		(AD5529R_REG_CAL_OFFSET_BASE + (ch) * 2)

/* Function generator registers */
#define AD5529R_REG_FUNC_EN			(0x9C + 1)
#define AD5529R_REG_FUNC_MODE_SEL_BASE		(0x9E + 1)
#define AD5529R_REG_FUNC_MODE_SEL(ch)		\
	(AD5529R_REG_FUNC_MODE_SEL_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_DAC_INPUT_B_BASE	(0xBE + 1)
#define AD5529R_REG_FUNC_DAC_INPUT_B(ch)	\
	(AD5529R_REG_FUNC_DAC_INPUT_B_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_DITHER_PERIOD_BASE	(0xDE + 1)
#define AD5529R_REG_FUNC_DITHER_PERIOD(ch)	\
	(AD5529R_REG_FUNC_DITHER_PERIOD_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_DITHER_PHASE_BASE	(0xFE + 1)
#define AD5529R_REG_FUNC_DITHER_PHASE(ch)	\
	(AD5529R_REG_FUNC_DITHER_PHASE_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_RAMP_STEP_BASE		(0x11E + 1)
#define AD5529R_REG_FUNC_RAMP_STEP(ch)		\
	(AD5529R_REG_FUNC_RAMP_STEP_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_INT_EN			(0x13E + 1)

/* Multiplexer and main DAC registers */
#define AD5529R_REG_MUX_OUT_SEL			(0x140 + 1)
#define AD5529R_REG_MULTI_DAC_SW_LDAC		(0x142 + 1)
#define AD5529R_REG_MULTI_DAC_INPUT_A		(0x144 + 1)
#define AD5529R_REG_DAC_SW_LDAC			(0x146 + 1)

#define AD5529R_REG_DAC_INPUT_A_BASE		(0x148 + 1)
#define AD5529R_REG_DAC_INPUT_A(ch)		(AD5529R_REG_DAC_INPUT_A_BASE + (ch) * 2)

/* Status and readback registers */
#define AD5529R_REG_FUNC_INT_STAT		(0x168 + 1)
#define AD5529R_REG_DAC_DATA_READBACK_BASE	(0x16A + 1)
#define AD5529R_REG_DAC_DATA_READBACK(ch)	\
	(AD5529R_REG_DAC_DATA_READBACK_BASE + (ch) * 2)

/* Temperature sensor registers */
#define AD5529R_REG_TSENS_EN			(0x18A + 1)
#define AD5529R_REG_TSENS_ALERT_FLAG		(0x18C + 1)
#define AD5529R_REG_TSENS_SHTD_FLAG		(0x18E + 1)
#define AD5529R_REG_TSENS_ALERT_STAT		(0x190 + 1)
#define AD5529R_REG_TSENS_SHTD_STAT		(0x192 + 1)
#define AD5529R_REG_ALARMB_TSENS_EN		(0x194 + 1)
#define AD5529R_REG_ALARMB_TSENS_SEL		(0x196 + 1)
#define AD5529R_REG_TSENS_SHTD_EN_CH		(0x198 + 1)
#define AD5529R_REG_DAC_DIS_DEGLITCH_CH		(0x19A + 1)
#define AD5529R_REG_DAC_INT_EN			(0x19C + 1)
#define AD5529R_REG_ALL_FUNC_INT_STAT		(0x19E + 1)
#define AD5529R_REG_FUNC_BUSY			(0x1A0 + 1)
#define AD5529R_REG_REF_SRC_SEL			(0x1A2 + 1)
#define AD5529R_REG_INIT_CRC_ERR_STAT		(0x1A4 + 1)

/* Hotpath registers for multi-device support */
#define AD5529R_REG_MULTI_DAC_HOTPATH_SW_LDAC		(0x1A8 + 1)
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_0	(0x1AA + 1)
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_1	(0x1AC + 1)
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_2	(0x1AE + 1)
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_3	(0x1B0 + 1)
#define AD5529R_REG_DAC_HOTPATH_SW_LDAC			(0x1B2 + 1)

/* Hotpath per-channel DAC input registers for each die */
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE	(0x1B4 + 1)
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE + (ch) * 2)

#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE	(0x1D4 + 1)
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE + (ch) * 2)

#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE	(0x1F4 + 1)
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE + (ch) * 2)

#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE	(0x214 + 1)
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE + (ch) * 2)

#define   AD5529R_INTERFACE_CONFIG_A_SW_RESET	(BIT(7) | BIT(0))
#define   AD5529R_INTERFACE_CONFIG_A_ADDR_ASCENSION	BIT(5)
#define   AD5529R_INTERFACE_CONFIG_A_SDO_ENABLE	BIT(4)
#define   AD5529R_INTERFACE_CONFIG_A_DEFAULT	0x10
#define   AD5529R_NUM_CHANNELS			16
#define   AD5529R_MAX_CHANNEL_INDEX		(AD5529R_NUM_CHANNELS - 1)
#define   AD5529R_MAX_REGISTER			(0x232 + 1)
#define   AD5529R_8BIT_REG_MAX			0x13
#define   AD5529R_ADDR(reg_addr)		((reg_addr) & 0xFFF)
#define   AD5529R_RESET_PULSE_US		1000
#define   AD5529R_RESET_DELAY_US		10000
#define   AD5529R_SPI_BUF_SIZE			4
#define   AD5529R_NUM_SUPPLIES			4
#define   AD5529R_SPI_READ_FLAG			0x80

/* Device identification values */
#define   AD5529R_PRODUCT_ID_16BIT		0x4A
#define   AD5529R_PRODUCT_ID_12BIT		0x49

struct ad5529r_model_data {
	const char *model_name;
	unsigned int resolution;
	const struct iio_chan_spec *channels;
};

#define AD5529R_DAC_CHANNEL(chan, bits) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |		\
			      BIT(IIO_CHAN_INFO_SCALE),		\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (bits),				\
		.storagebits = 16,				\
	},							\
}

static const char * const ad5529r_supply_names[AD5529R_NUM_SUPPLIES] = {
	"vdd",
	"avdd",
	"hvdd",
	"hvss",
};

static const struct iio_chan_spec ad5529r_channels_16bit[] = {
	AD5529R_DAC_CHANNEL(0, 16),
	AD5529R_DAC_CHANNEL(1, 16),
	AD5529R_DAC_CHANNEL(2, 16),
	AD5529R_DAC_CHANNEL(3, 16),
	AD5529R_DAC_CHANNEL(4, 16),
	AD5529R_DAC_CHANNEL(5, 16),
	AD5529R_DAC_CHANNEL(6, 16),
	AD5529R_DAC_CHANNEL(7, 16),
	AD5529R_DAC_CHANNEL(8, 16),
	AD5529R_DAC_CHANNEL(9, 16),
	AD5529R_DAC_CHANNEL(10, 16),
	AD5529R_DAC_CHANNEL(11, 16),
	AD5529R_DAC_CHANNEL(12, 16),
	AD5529R_DAC_CHANNEL(13, 16),
	AD5529R_DAC_CHANNEL(14, 16),
	AD5529R_DAC_CHANNEL(15, 16),
};

static const struct iio_chan_spec ad5529r_channels_12bit[] = {
	AD5529R_DAC_CHANNEL(0, 12),
	AD5529R_DAC_CHANNEL(1, 12),
	AD5529R_DAC_CHANNEL(2, 12),
	AD5529R_DAC_CHANNEL(3, 12),
	AD5529R_DAC_CHANNEL(4, 12),
	AD5529R_DAC_CHANNEL(5, 12),
	AD5529R_DAC_CHANNEL(6, 12),
	AD5529R_DAC_CHANNEL(7, 12),
	AD5529R_DAC_CHANNEL(8, 12),
	AD5529R_DAC_CHANNEL(9, 12),
	AD5529R_DAC_CHANNEL(10, 12),
	AD5529R_DAC_CHANNEL(11, 12),
	AD5529R_DAC_CHANNEL(12, 12),
	AD5529R_DAC_CHANNEL(13, 12),
	AD5529R_DAC_CHANNEL(14, 12),
	AD5529R_DAC_CHANNEL(15, 12),
};

static const struct ad5529r_model_data ad5529r_16bit_model_data = {
	.model_name = "ad5529r-16",
	.resolution = 16,
	.channels = ad5529r_channels_16bit,
};

static const struct ad5529r_model_data ad5529r_12bit_model_data = {
	.model_name = "ad5529r-12",
	.resolution = 12,
	.channels = ad5529r_channels_12bit,
};

struct ad5529r_state {
	struct spi_device *spi;
	const struct ad5529r_model_data *model_data;
	struct regmap *regmap_8bit;
	struct regmap *regmap_16bit;
};

static const struct regmap_range ad5529r_8bit_readable_ranges[] = {
	regmap_reg_range(AD5529R_REG_INTERFACE_CONFIG_A, AD5529R_REG_CHIP_GRADE),
	regmap_reg_range(AD5529R_REG_SCRATCH_PAD, AD5529R_REG_VENDOR_H),
	regmap_reg_range(AD5529R_REG_STREAM_MODE, AD5529R_REG_INTERFACE_STATUS_A),
};

static const struct regmap_range ad5529r_16bit_readable_ranges[] = {
	regmap_reg_range(AD5529R_REG_MULTI_DAC_CH_SEL, AD5529R_REG_LDAC_HW_SW),
	regmap_reg_range(AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE,
			 AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_OUT_OPERATING_MODE, AD5529R_REG_OUT_OPERATING_MODE),
	regmap_reg_range(AD5529R_REG_OUT_RANGE_BASE,
			 AD5529R_REG_OUT_RANGE_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_CAL_GAIN_BASE,
			 AD5529R_REG_CAL_GAIN_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_CAL_OFFSET_BASE,
			 AD5529R_REG_CAL_OFFSET_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_EN, AD5529R_REG_FUNC_EN),
	regmap_reg_range(AD5529R_REG_FUNC_MODE_SEL_BASE,
			 AD5529R_REG_FUNC_MODE_SEL_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_DAC_INPUT_B_BASE,
			 AD5529R_REG_FUNC_DAC_INPUT_B_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_DITHER_PERIOD_BASE,
			 AD5529R_REG_FUNC_DITHER_PERIOD_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_DITHER_PHASE_BASE,
			 AD5529R_REG_FUNC_DITHER_PHASE_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_RAMP_STEP_BASE,
			 AD5529R_REG_FUNC_RAMP_STEP_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_INT_EN, AD5529R_REG_DAC_SW_LDAC),
	regmap_reg_range(AD5529R_REG_DAC_INPUT_A_BASE,
			 AD5529R_REG_DAC_INPUT_A_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_FUNC_INT_STAT, AD5529R_REG_FUNC_INT_STAT),
	regmap_reg_range(AD5529R_REG_DAC_DATA_READBACK_BASE,
			 AD5529R_REG_DAC_DATA_READBACK_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_TSENS_EN, AD5529R_REG_INIT_CRC_ERR_STAT),
	regmap_reg_range(AD5529R_REG_MULTI_DAC_HOTPATH_SW_LDAC, AD5529R_REG_DAC_HOTPATH_SW_LDAC),
	regmap_reg_range(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE,
			 AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE +
			 AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE,
			 AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE +
			 AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE,
			 AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE +
			 AD5529R_MAX_CHANNEL_INDEX * 2),
	regmap_reg_range(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE,
			 AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE +
			 AD5529R_MAX_CHANNEL_INDEX * 2),
};

static const struct regmap_access_table ad5529r_8bit_readable_table = {
	.yes_ranges = ad5529r_8bit_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad5529r_8bit_readable_ranges),
};

static const struct regmap_access_table ad5529r_16bit_readable_table = {
	.yes_ranges = ad5529r_16bit_readable_ranges,
	.n_yes_ranges = ARRAY_SIZE(ad5529r_16bit_readable_ranges),
};

static const struct regmap_range ad5529r_8bit_read_only_ranges[] = {
	regmap_reg_range(AD5529R_REG_CHIP_TYPE, AD5529R_REG_CHIP_GRADE),
	regmap_reg_range(AD5529R_REG_SPI_REVISION, AD5529R_REG_VENDOR_H),
	regmap_reg_range(AD5529R_REG_DEVICE_CONFIG, AD5529R_REG_DEVICE_CONFIG),
};

static const struct regmap_range ad5529r_16bit_read_only_ranges[] = {
	regmap_reg_range(AD5529R_REG_TSENS_ALERT_FLAG, AD5529R_REG_TSENS_SHTD_STAT),
	regmap_reg_range(AD5529R_REG_ALL_FUNC_INT_STAT, AD5529R_REG_FUNC_BUSY),
	regmap_reg_range(AD5529R_REG_INIT_CRC_ERR_STAT, AD5529R_REG_INIT_CRC_ERR_STAT),
	regmap_reg_range(AD5529R_REG_DAC_DATA_READBACK_BASE,
			 AD5529R_REG_DAC_DATA_READBACK_BASE + AD5529R_MAX_CHANNEL_INDEX * 2),
};

static const struct regmap_access_table ad5529r_8bit_writeable_table = {
	.no_ranges = ad5529r_8bit_read_only_ranges,
	.n_no_ranges = ARRAY_SIZE(ad5529r_8bit_read_only_ranges),
};

static const struct regmap_access_table ad5529r_16bit_writeable_table = {
	.no_ranges = ad5529r_16bit_read_only_ranges,
	.n_no_ranges = ARRAY_SIZE(ad5529r_16bit_read_only_ranges),
};

static const struct regmap_config ad5529r_regmap_8bit_config = {
	.name = "ad5529r-8bit",
	.reg_bits = 16,
	.val_bits = 8,
	.max_register = AD5529R_8BIT_REG_MAX,
	.read_flag_mask = AD5529R_SPI_READ_FLAG,
	.rd_table = &ad5529r_8bit_readable_table,
	.wr_table = &ad5529r_8bit_writeable_table,
};

static const struct regmap_config ad5529r_regmap_16bit_config = {
	.name = "ad5529r-16bit",
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = AD5529R_MAX_REGISTER,
	.read_flag_mask = AD5529R_SPI_READ_FLAG,
	.rd_table = &ad5529r_16bit_readable_table,
	.wr_table = &ad5529r_16bit_writeable_table,
};

static struct regmap *ad5529r_get_regmap(struct ad5529r_state *st, unsigned int reg)
{
	if (reg <= AD5529R_8BIT_REG_MAX)
		return st->regmap_8bit;

	return st->regmap_16bit;
}

static int ad5529r_debugfs_reg_read(struct ad5529r_state *st, unsigned int reg,
				    unsigned int *val)
{
	return regmap_read(ad5529r_get_regmap(st, reg), reg, val);
}

static int ad5529r_debugfs_reg_write(struct ad5529r_state *st, unsigned int reg,
				     unsigned int val)
{
	return regmap_write(ad5529r_get_regmap(st, reg), reg, val);
}

static int ad5529r_detect_device(struct ad5529r_state *st)
{
	unsigned int product_id;
	int ret;

	ret = regmap_read(st->regmap_8bit, AD5529R_REG_PRODUCT_ID_L, &product_id);
	if (ret)
		return ret;

	switch (product_id) {
	case AD5529R_PRODUCT_ID_16BIT:
		st->model_data = &ad5529r_16bit_model_data;
		break;
	case AD5529R_PRODUCT_ID_12BIT:
		st->model_data = &ad5529r_12bit_model_data;
		break;
	default:
		dev_err(&st->spi->dev, "Unknown product ID: 0x%02X\n", product_id);
		return -ENODEV;
	}

	dev_dbg(&st->spi->dev, "Detected %s variant (Product ID: 0x%02X)\n",
		st->model_data->model_name, product_id);

	return 0;
}

static int ad5529r_reset(struct ad5529r_state *st)
{
	struct reset_control *rst;
	int ret;

	rst = devm_reset_control_get_optional_exclusive(&st->spi->dev, NULL);
	if (IS_ERR(rst))
		return dev_err_probe(&st->spi->dev, PTR_ERR(rst),
				     "Failed to get reset control\n");

	if (rst) {
		ret = reset_control_deassert(rst);
		if (ret)
			return ret;
	} else {
		ret = regmap_write(st->regmap_8bit, AD5529R_REG_INTERFACE_CONFIG_A,
				   AD5529R_INTERFACE_CONFIG_A_SW_RESET);
		if (ret)
			return ret;
	}

	fsleep(AD5529R_RESET_DELAY_US);

	return regmap_write(st->regmap_8bit, AD5529R_REG_INTERFACE_CONFIG_A,
			   AD5529R_INTERFACE_CONFIG_A_DEFAULT);
}

static int ad5529r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ad5529r_state *st = iio_priv(indio_dev);
	unsigned int reg_addr;
	unsigned int reg_val_h;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		reg_addr = AD5529R_REG_DAC_INPUT_A(chan->channel);
		ret = regmap_read(st->regmap_16bit, reg_addr, &reg_val_h);
		if (ret)
			return ret;

		*val = reg_val_h;

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/*
		 * Using default 0-5V range: VOUTn = A × D/2^N + B
		 * where A = 5V, B = 0V, D = digital code, N = resolution
		 * Scale = 5V / 2^resolution
		 */
		*val = 5;
		*val2 = st->model_data->resolution;

		return IIO_VAL_FRACTIONAL_LOG2;
	default:
		return -EINVAL;
	}
}

static int ad5529r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ad5529r_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > GENMASK(st->model_data->resolution - 1, 0))
			return -EINVAL;

		return regmap_write(st->regmap_16bit, AD5529R_REG_DAC_INPUT_A(chan->channel), val);
	default:
		return -EINVAL;
	}
}

static int ad5529r_reg_access(struct iio_dev *indio_dev,
			      unsigned int reg,
			      unsigned int writeval,
			      unsigned int *readval)
{
	struct ad5529r_state *st = iio_priv(indio_dev);

	if (!readval)
		return ad5529r_debugfs_reg_write(st, reg, writeval);

	return ad5529r_debugfs_reg_read(st, reg, readval);
}

static const struct iio_info ad5529r_info = {
	.read_raw = ad5529r_read_raw,
	.write_raw = ad5529r_write_raw,
	.debugfs_reg_access = ad5529r_reg_access,
};

static int ad5529r_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct iio_dev *indio_dev;
	struct ad5529r_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;

	ret = devm_regulator_bulk_get_enable(dev, AD5529R_NUM_SUPPLIES,
					     ad5529r_supply_names);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get and enable regulators\n");

	st->regmap_8bit = devm_regmap_init_spi(spi, &ad5529r_regmap_8bit_config);
	if (IS_ERR(st->regmap_8bit))
		return dev_err_probe(dev, PTR_ERR(st->regmap_8bit),
				     "Failed to initialize 8-bit regmap\n");

	st->regmap_16bit = devm_regmap_init_spi(spi, &ad5529r_regmap_16bit_config);
	if (IS_ERR(st->regmap_16bit))
		return dev_err_probe(dev, PTR_ERR(st->regmap_16bit),
				     "Failed to initialize 16-bit regmap\n");

	ret = ad5529r_reset(st);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to reset device\n");

	ret = ad5529r_detect_device(st);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to detect device variant\n");

	indio_dev->name = st->model_data->model_name;
	indio_dev->info = &ad5529r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->model_data->channels;
	indio_dev->num_channels = AD5529R_NUM_CHANNELS;

	return devm_iio_device_register(dev, indio_dev);
}

static const struct of_device_id ad5529r_of_match[] = {
	{ .compatible = "adi,ad5529r" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5529r_of_match);

static const struct spi_device_id ad5529r_id[] = {
	{ "ad5529r" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ad5529r_id);

static struct spi_driver ad5529r_driver = {
	.driver = {
		.name = "ad5529r",
		.of_match_table = ad5529r_of_match,
	},
	.probe = ad5529r_probe,
	.id_table = ad5529r_id,
};
module_spi_driver(ad5529r_driver);

MODULE_AUTHOR("Janani Sunil <janani.sunil@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD5529R 12/16-bit DAC driver");
MODULE_LICENSE("GPL");
