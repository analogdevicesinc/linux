// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD5529R Digital-to-Analog Converter Driver
 * 16-Channel, 12/16-Bit, 40V High Voltage Precision DAC
 *
 * Copyright 2026 Analog Devices Inc.
 * Author: Janani Sunil <janani.sunil@analog.com>
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/iio/iio.h>
#include <linux/regmap.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include <linux/unaligned.h>
#include <linux/mutex.h>
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
#define AD5529R_REG_MULTI_DAC_CH_SEL		0x14
#define AD5529R_REG_LDAC_SYNC_ASYNC		0x16
#define AD5529R_REG_LDAC_HW_SW			0x18

/* Hardware LDAC source and edge select registers (per channel, 16-bit) */
#define AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE	0x1A
#define AD5529R_REG_LDAC_HW_SRC_EDGE_SEL(ch)	\
	(AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE + (ch) * 2)

/* Output configuration */
#define AD5529R_REG_OUT_OPERATING_MODE		0x3A
#define AD5529R_REG_OUT_RANGE_BASE		0x3C
#define AD5529R_REG_OUT_RANGE(ch)		(AD5529R_REG_OUT_RANGE_BASE + (ch) * 2)

/* Calibration registers */
#define AD5529R_REG_CAL_GAIN_BASE		0x5C
#define AD5529R_REG_CAL_GAIN(ch)		(AD5529R_REG_CAL_GAIN_BASE + (ch) * 2)

#define AD5529R_REG_CAL_OFFSET_BASE		0x7C
#define AD5529R_REG_CAL_OFFSET(ch)		(AD5529R_REG_CAL_OFFSET_BASE + (ch) * 2)

/* Function generator registers */
#define AD5529R_REG_FUNC_EN			0x9C
#define AD5529R_REG_FUNC_MODE_SEL_BASE		0x9E
#define AD5529R_REG_FUNC_MODE_SEL(ch)		\
	(AD5529R_REG_FUNC_MODE_SEL_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_DAC_INPUT_B_BASE	0xBE
#define AD5529R_REG_FUNC_DAC_INPUT_B(ch)	\
	(AD5529R_REG_FUNC_DAC_INPUT_B_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_DITHER_PERIOD_BASE	0xDE
#define AD5529R_REG_FUNC_DITHER_PERIOD(ch)	\
	(AD5529R_REG_FUNC_DITHER_PERIOD_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_DITHER_PHASE_BASE	0xFE
#define AD5529R_REG_FUNC_DITHER_PHASE(ch)	\
	(AD5529R_REG_FUNC_DITHER_PHASE_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_RAMP_STEP_BASE		0x11E
#define AD5529R_REG_FUNC_RAMP_STEP(ch)		\
	(AD5529R_REG_FUNC_RAMP_STEP_BASE + (ch) * 2)

#define AD5529R_REG_FUNC_INT_EN			0x13E

/* Multiplexer and main DAC registers */
#define AD5529R_REG_MUX_OUT_SEL			0x140
#define AD5529R_REG_MULTI_DAC_SW_LDAC		0x142
#define AD5529R_REG_MULTI_DAC_INPUT_A		0x144
#define AD5529R_REG_DAC_SW_LDAC			0x146

#define AD5529R_REG_DAC_INPUT_A_BASE		0x148
#define AD5529R_REG_DAC_INPUT_A(ch)		(AD5529R_REG_DAC_INPUT_A_BASE + (ch) * 2)

/* Status and readback registers */
#define AD5529R_REG_FUNC_INT_STAT		0x168
#define AD5529R_REG_DAC_DATA_READBACK_BASE	0x16A
#define AD5529R_REG_DAC_DATA_READBACK(ch)	\
	(AD5529R_REG_DAC_DATA_READBACK_BASE + (ch) * 2)

/* Temperature sensor registers */
#define AD5529R_REG_TSENS_EN			0x18A
#define AD5529R_REG_TSENS_ALERT_FLAG		0x18C
#define AD5529R_REG_TSENS_SHTD_FLAG		0x18E
#define AD5529R_REG_TSENS_ALERT_STAT		0x190
#define AD5529R_REG_TSENS_SHTD_STAT		0x192
#define AD5529R_REG_ALARMB_TSENS_EN		0x194
#define AD5529R_REG_ALARMB_TSENS_SEL		0x196
#define AD5529R_REG_TSENS_SHTD_EN_CH		0x198
#define AD5529R_REG_DAC_DIS_DEGLITCH_CH		0x19A
#define AD5529R_REG_DAC_INT_EN			0x19C
#define AD5529R_REG_ALL_FUNC_INT_STAT		0x19E
#define AD5529R_REG_FUNC_BUSY			0x1A0
#define AD5529R_REG_REF_SRC_SEL			0x1A2
#define AD5529R_REG_INIT_CRC_ERR_STAT		0x1A4

/* Hotpath registers for multi-device support */
#define AD5529R_REG_MULTI_DAC_HOTPATH_SW_LDAC		0x1A8
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_0	0x1AA
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_1	0x1AC
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_2	0x1AE
#define AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_3	0x1B0
#define AD5529R_REG_DAC_HOTPATH_SW_LDAC			0x1B2

/* Hotpath per-channel DAC input registers for each die */
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE	0x1B4
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE + (ch) * 2)

#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE	0x1D4
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE + (ch) * 2)

#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE	0x1F4
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE + (ch) * 2)

#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE	0x214
#define AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3(ch)	\
	(AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE + (ch) * 2)

#define   AD5529R_INSTR_RW_BIT			BIT(15)
#define   AD5529R_INSTR_ADDR_MASK		GENMASK(11, 0)
#define   AD5529R_DAC_DATA_MASK			GENMASK(15, 0)
#define   AD5529R_INTERFACE_CONFIG_A_SW_RESET	(BIT(7) | BIT(0))
#define   AD5529R_INTERFACE_CONFIG_A_ADDR_ASCENSION	BIT(5)
#define   AD5529R_INTERFACE_CONFIG_A_SDO_ENABLE	BIT(4)
#define   AD5529R_INTERFACE_CONFIG_A_DEFAULT	0x10
#define   AD5529R_NUM_CHANNELS			16
#define   AD5529R_MAX_CHANNEL_INDEX		(AD5529R_NUM_CHANNELS - 1)
#define   AD5529R_MAX_REGISTER			0x232
#define   AD5529R_LEN(reg_addr)			(((reg_addr) >= 0x14) ? 2 : 1)
#define   AD5529R_ADDR(reg_addr)		((reg_addr) & 0xFFF)
#define   AD5529R_RESET_PULSE_MS		1
#define   AD5529R_RESET_DELAY_MS		10
#define   AD5529R_SPI_BUF_SIZE		4
enum ad5529r_type {
	AD5529R_16BIT,
	AD5529R_12BIT,
};

struct ad5529r_model_data {
	const char *model_name;
	enum ad5529r_type type;
	unsigned int num_hw_channels;
	unsigned int resolution;
	const struct iio_chan_spec *channels;
};

#define AD5529R_DAC_CHANNEL(chan, bits) {		\
	.type = IIO_VOLTAGE,					\
	.indexed = 1,						\
	.output = 1,						\
	.channel = (chan),					\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		\
	.scan_type = {						\
		.sign = 'u',					\
		.realbits = (bits),				\
		.storagebits = 16,				\
	},							\
}

#define AD5529R_NUM_SUPPLIES	4

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
	.model_name = "ad5529r-16bit",
	.type = AD5529R_16BIT,
	.num_hw_channels = 16,
	.resolution = 16,
	.channels = ad5529r_channels_16bit,
};

static const struct ad5529r_model_data ad5529r_12bit_model_data = {
	.model_name = "ad5529r-12bit",
	.type = AD5529R_12BIT,
	.num_hw_channels = 16,
	.resolution = 12,
	.channels = ad5529r_channels_12bit,
};

struct ad5529r_state {
	struct spi_device *spi;
	const struct ad5529r_model_data *model_data;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct mutex lock;	/* Protect SPI operations and driver state */
};

static inline u16 ad5529r_get_dac_mask(struct ad5529r_state *st)
{
	return GENMASK(st->model_data->resolution - 1, 0);
}

static int ad5529r_spi_read(struct ad5529r_state *st, u16 reg_addr,
			     void *data, int len)
{
	struct spi_transfer xfer;
	struct spi_message msg;
	u8 tx_buf[AD5529R_SPI_BUF_SIZE];
	u8 rx_buf[AD5529R_SPI_BUF_SIZE];
	u16 addr;
	int ret;

	if (!st || !data)
		return -EINVAL;

	if (len < 1 || len > 2)
		return -EINVAL;

	if (reg_addr > AD5529R_MAX_REGISTER)
		return -EINVAL;

	mutex_lock(&st->lock);

	xfer = (struct spi_transfer)
{
		.tx_buf = tx_buf,
		.rx_buf = rx_buf,
		.len = 2 + len,
	};

	/* Always use decrement mode: for 16-bit access, start at reg_addr + 1 */
	if (len == 2)
		addr = AD5529R_ADDR(reg_addr) + 1;
	else
		addr = AD5529R_ADDR(reg_addr);

	tx_buf[0] = 0x80 | (addr >> 8);
	tx_buf[1] = addr & 0xFF;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(st->spi, &msg);
	if (ret) {
		mutex_unlock(&st->lock);
		return ret;
	}

	if (len == 1) {
		*(u8 *)data = rx_buf[2];
	} else {
		/* Decrement mode: data comes back as big-endian */
		*(u16 *)data = get_unaligned_be16(&rx_buf[2]);
	}

	mutex_unlock(&st->lock);
	return 0;
}

static int ad5529r_spi_write(struct ad5529r_state *st, u16 reg_addr,
			      u16 reg_data, int len)
{
	struct spi_transfer xfer;
	struct spi_message msg;
	u8 tx_buf[AD5529R_SPI_BUF_SIZE];
	u16 addr;
	int ret;

	if (!st)
		return -EINVAL;

	if (len < 1 || len > 2)
		return -EINVAL;

	if (reg_addr > AD5529R_MAX_REGISTER)
		return -EINVAL;

	mutex_lock(&st->lock);

	xfer = (struct spi_transfer)
{
		.tx_buf = tx_buf,
		.len = 2 + len,
	};

	/* Always use decrement mode: for 16-bit access, start at reg_addr + 1 */
	if (len == 2)
		addr = AD5529R_ADDR(reg_addr) + 1;
	else
		addr = AD5529R_ADDR(reg_addr);

	tx_buf[0] = (addr >> 8) & 0xFF;
	tx_buf[1] = addr & 0xFF;

	if (len == 1) {
		tx_buf[2] = reg_data & 0xFF;
	} else {
		/* Decrement mode: send data as big-endian */
		put_unaligned_be16(reg_data, &tx_buf[2]);
	}

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(st->spi, &msg);
	mutex_unlock(&st->lock);

	return ret;
}

static int ad5529r_regmap_read(void *context, const void *reg_buf,
				size_t reg_size, void *val_buf,
				size_t val_size)
{
	struct ad5529r_state *st = context;
	u8 reg_len, data8;
	u16 reg, data16;
	int ret;

	if (!st || !reg_buf || !val_buf)
		return -EINVAL;

	if (reg_size != 2 || val_size != 2)
		return -EINVAL;

	reg = get_unaligned_be16(reg_buf);

	if (reg > AD5529R_MAX_REGISTER)
		return -EINVAL;

	reg_len = AD5529R_LEN(reg);

	if (reg_len == 1) {
		ret = ad5529r_spi_read(st, reg, &data8, 1);
		if (ret)
			return ret;

		put_unaligned_be16((u16)data8, val_buf);
	} else {
		ret = ad5529r_spi_read(st, reg, &data16, 2);
		if (ret)
			return ret;

		put_unaligned_be16(data16, val_buf);
	}

	return 0;
}

static int ad5529r_regmap_write(void *context, const void *data, size_t count)
{
	struct ad5529r_state *st = context;
	u16 reg, val;
	u8 reg_len;
	int ret;

	if (!st || !data)
		return -EINVAL;

	if (count != 4)
		return -EINVAL;

	reg = get_unaligned_be16(data);
	val = get_unaligned_be16((u8 *)data + 2);

	if (reg > AD5529R_MAX_REGISTER)
		return -EINVAL;

	reg_len = AD5529R_LEN(reg);

	if (reg_len == 1)
		ret = ad5529r_spi_write(st, reg, val & 0xFF, 1);
	else
		ret = ad5529r_spi_write(st, reg, val, 2);

	return ret;
}

static const struct regmap_bus ad5529r_regmap_bus = {
	.read = ad5529r_regmap_read,
	.write = ad5529r_regmap_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static bool ad5529r_reg_readable(struct device *dev, unsigned int reg)
{
	if (!dev || reg > AD5529R_MAX_REGISTER)
		return false;

	switch (reg) {
	case AD5529R_REG_INTERFACE_CONFIG_A:
	case AD5529R_REG_INTERFACE_CONFIG_B:
	case AD5529R_REG_DEVICE_CONFIG:
	case AD5529R_REG_CHIP_TYPE:
	case AD5529R_REG_PRODUCT_ID_L:
	case AD5529R_REG_PRODUCT_ID_H:
	case AD5529R_REG_CHIP_GRADE:
	case AD5529R_REG_SCRATCH_PAD:
	case AD5529R_REG_SPI_REVISION:
	case AD5529R_REG_VENDOR_L:
	case AD5529R_REG_VENDOR_H:
	case AD5529R_REG_STREAM_MODE:
	case AD5529R_REG_TRANSFER_CONFIG:
	case AD5529R_REG_INTERFACE_CONFIG_C:
	case AD5529R_REG_INTERFACE_STATUS_A:
		return true;
	case AD5529R_REG_MULTI_DAC_CH_SEL:
	case AD5529R_REG_LDAC_SYNC_ASYNC:
	case AD5529R_REG_LDAC_HW_SW:
		return true;
	case AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE ...
	     (AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_OUT_OPERATING_MODE:
		return true;
	case AD5529R_REG_OUT_RANGE_BASE ...
	     (AD5529R_REG_OUT_RANGE_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_CAL_GAIN_BASE ...
	     (AD5529R_REG_CAL_GAIN_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_CAL_OFFSET_BASE ...
	     (AD5529R_REG_CAL_OFFSET_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_EN:
		return true;
	case AD5529R_REG_FUNC_MODE_SEL_BASE ...
	     (AD5529R_REG_FUNC_MODE_SEL_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_DAC_INPUT_B_BASE ...
	     (AD5529R_REG_FUNC_DAC_INPUT_B_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_DITHER_PERIOD_BASE ...
	     (AD5529R_REG_FUNC_DITHER_PERIOD_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_DITHER_PHASE_BASE ...
	     (AD5529R_REG_FUNC_DITHER_PHASE_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_RAMP_STEP_BASE ...
	     (AD5529R_REG_FUNC_RAMP_STEP_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_INT_EN:
		return true;
	case AD5529R_REG_MUX_OUT_SEL:
	case AD5529R_REG_MULTI_DAC_SW_LDAC:
	case AD5529R_REG_MULTI_DAC_INPUT_A:
	case AD5529R_REG_DAC_SW_LDAC:
		return true;
	case AD5529R_REG_DAC_INPUT_A_BASE ...
	     (AD5529R_REG_DAC_INPUT_A_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_INT_STAT:
		return true;
	case AD5529R_REG_DAC_DATA_READBACK_BASE ...
	     (AD5529R_REG_DAC_DATA_READBACK_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_TSENS_EN:
	case AD5529R_REG_TSENS_ALERT_FLAG:
	case AD5529R_REG_TSENS_SHTD_FLAG:
	case AD5529R_REG_TSENS_ALERT_STAT:
	case AD5529R_REG_TSENS_SHTD_STAT:
	case AD5529R_REG_ALARMB_TSENS_EN:
	case AD5529R_REG_ALARMB_TSENS_SEL:
	case AD5529R_REG_TSENS_SHTD_EN_CH:
	case AD5529R_REG_DAC_DIS_DEGLITCH_CH:
	case AD5529R_REG_DAC_INT_EN:
	case AD5529R_REG_ALL_FUNC_INT_STAT:
	case AD5529R_REG_FUNC_BUSY:
	case AD5529R_REG_REF_SRC_SEL:
	case AD5529R_REG_INIT_CRC_ERR_STAT:
		return true;
	case AD5529R_REG_MULTI_DAC_HOTPATH_SW_LDAC:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_0:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_1:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_2:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_3:
	case AD5529R_REG_DAC_HOTPATH_SW_LDAC:
		return true;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	default:
		return false;
	}
}

static bool ad5529r_reg_writeable(struct device *dev, unsigned int reg)
{
	if (!dev || reg > AD5529R_MAX_REGISTER)
		return false;

	switch (reg) {
	case AD5529R_REG_CHIP_TYPE:
	case AD5529R_REG_PRODUCT_ID_L:
	case AD5529R_REG_PRODUCT_ID_H:
	case AD5529R_REG_CHIP_GRADE:
	case AD5529R_REG_SPI_REVISION:
	case AD5529R_REG_VENDOR_L:
	case AD5529R_REG_VENDOR_H:
	case AD5529R_REG_DEVICE_CONFIG:
		return false;
	case AD5529R_REG_TSENS_ALERT_FLAG:
	case AD5529R_REG_TSENS_SHTD_FLAG:
	case AD5529R_REG_TSENS_ALERT_STAT:
	case AD5529R_REG_TSENS_SHTD_STAT:
	case AD5529R_REG_ALL_FUNC_INT_STAT:
	case AD5529R_REG_FUNC_BUSY:
	case AD5529R_REG_INIT_CRC_ERR_STAT:
		return false;
	case AD5529R_REG_DAC_DATA_READBACK_BASE ...
	     (AD5529R_REG_DAC_DATA_READBACK_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		return false;
	case AD5529R_REG_INTERFACE_CONFIG_A:
	case AD5529R_REG_INTERFACE_CONFIG_B:
	case AD5529R_REG_SCRATCH_PAD:
	case AD5529R_REG_STREAM_MODE:
	case AD5529R_REG_TRANSFER_CONFIG:
	case AD5529R_REG_INTERFACE_CONFIG_C:
	case AD5529R_REG_INTERFACE_STATUS_A:
		return true;
	case AD5529R_REG_MULTI_DAC_CH_SEL:
	case AD5529R_REG_LDAC_SYNC_ASYNC:
	case AD5529R_REG_LDAC_HW_SW:
		return true;
	case AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE ...
	     (AD5529R_REG_LDAC_HW_SRC_EDGE_SEL_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_OUT_OPERATING_MODE:
		return true;
	case AD5529R_REG_OUT_RANGE_BASE ...
	     (AD5529R_REG_OUT_RANGE_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_CAL_GAIN_BASE ...
	     (AD5529R_REG_CAL_GAIN_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_CAL_OFFSET_BASE ...
	     (AD5529R_REG_CAL_OFFSET_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_EN:
		return true;
	case AD5529R_REG_FUNC_MODE_SEL_BASE ...
	     (AD5529R_REG_FUNC_MODE_SEL_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_DAC_INPUT_B_BASE ...
	     (AD5529R_REG_FUNC_DAC_INPUT_B_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_DITHER_PERIOD_BASE ...
	     (AD5529R_REG_FUNC_DITHER_PERIOD_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_DITHER_PHASE_BASE ...
	     (AD5529R_REG_FUNC_DITHER_PHASE_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_RAMP_STEP_BASE ...
	     (AD5529R_REG_FUNC_RAMP_STEP_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_FUNC_INT_EN:
	case AD5529R_REG_FUNC_INT_STAT:
		return true;
	case AD5529R_REG_MUX_OUT_SEL:
	case AD5529R_REG_MULTI_DAC_SW_LDAC:
	case AD5529R_REG_MULTI_DAC_INPUT_A:
	case AD5529R_REG_DAC_SW_LDAC:
		return true;
	case AD5529R_REG_DAC_INPUT_A_BASE ...
	     (AD5529R_REG_DAC_INPUT_A_BASE + AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_TSENS_EN:
	case AD5529R_REG_ALARMB_TSENS_EN:
	case AD5529R_REG_ALARMB_TSENS_SEL:
	case AD5529R_REG_TSENS_SHTD_EN_CH:
	case AD5529R_REG_DAC_DIS_DEGLITCH_CH:
	case AD5529R_REG_DAC_INT_EN:
	case AD5529R_REG_REF_SRC_SEL:
		return true;
	case AD5529R_REG_MULTI_DAC_HOTPATH_SW_LDAC:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_0:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_1:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_2:
	case AD5529R_REG_MULTI_DAC_HOTPATH_INPUT_A_DIE_3:
	case AD5529R_REG_DAC_HOTPATH_SW_LDAC:
		return true;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_0_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_1_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_2_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	case AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE ...
	     (AD5529R_REG_DAC_HOTPATH_INPUT_A_DIE_3_BASE +
	      AD5529R_MAX_CHANNEL_INDEX * 2):
		if (!(reg & 1))
			return true;
		return false;
	default:
		return false;
	}
}

static const struct regmap_config ad5529r_regmap_config = {
	.reg_bits = 16,
	.val_bits = 16,
	.max_register = AD5529R_MAX_REGISTER,
	.readable_reg = ad5529r_reg_readable,
	.writeable_reg = ad5529r_reg_writeable,
};

static int ad5529r_reset(struct ad5529r_state *st)
{
	int ret;

	if (!st)
		return -EINVAL;

	if (st->reset_gpio) {
		gpiod_set_value_cansleep(st->reset_gpio, 0);
		msleep(AD5529R_RESET_PULSE_MS);
		gpiod_set_value_cansleep(st->reset_gpio, 1);
		msleep(AD5529R_RESET_DELAY_MS);
	} else {
		ret = regmap_write(st->regmap, AD5529R_REG_INTERFACE_CONFIG_A,
				   AD5529R_INTERFACE_CONFIG_A_SW_RESET);
		if (ret)
			return ret;

		msleep(AD5529R_RESET_DELAY_MS);

		ret = regmap_write(st->regmap, AD5529R_REG_INTERFACE_CONFIG_A,
				   AD5529R_INTERFACE_CONFIG_A_DEFAULT);
		if (ret)
			return ret;
	}

	return 0;
}

static int ad5529r_read_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int *val, int *val2, long mask)
{
	struct ad5529r_state *st = iio_priv(indio_dev);
	unsigned int reg_val_h;
	int ret;

	if (!indio_dev || !chan || !val)
		return -EINVAL;

	if (chan->channel > AD5529R_MAX_CHANNEL_INDEX)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(st->regmap, AD5529R_REG_DAC_INPUT_A(chan->channel),
				  &reg_val_h);
		if (ret)
			return ret;

		*val = reg_val_h & ad5529r_get_dac_mask(st);

		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int ad5529r_write_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int val, int val2, long mask)
{
	struct ad5529r_state *st = iio_priv(indio_dev);

	if (!indio_dev || !chan)
		return -EINVAL;

	if (chan->channel > AD5529R_MAX_CHANNEL_INDEX)
		return -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (val < 0 || val > ad5529r_get_dac_mask(st))
			return -EINVAL;

		return regmap_write(st->regmap, AD5529R_REG_DAC_INPUT_A(chan->channel),
				    val & ad5529r_get_dac_mask(st));

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

	if (!indio_dev || reg > AD5529R_MAX_REGISTER)
		return -EINVAL;

	if (!readval)
		return regmap_write(st->regmap, reg, writeval);
	else
		return regmap_read(st->regmap, reg, readval);
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

	if (!spi)
		return -EINVAL;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);

	st->spi = spi;
	st->model_data = device_get_match_data(dev);
	if (!st->model_data)
		st->model_data = &ad5529r_16bit_model_data; /* Default to 16-bit */

	/* Validate model data */
	if (!st->model_data || !st->model_data->channels ||
	    st->model_data->num_hw_channels != AD5529R_NUM_CHANNELS ||
	    (st->model_data->resolution != 12 && st->model_data->resolution != 16))
		return -EINVAL;

	ret = devm_mutex_init(dev, &st->lock);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_enable(dev, AD5529R_NUM_SUPPLIES,
					     ad5529r_supply_names);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get and enable regulators\n");

	st->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(st->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(st->reset_gpio),
				     "Failed to get reset GPIO\n");

	st->regmap = devm_regmap_init(dev, &ad5529r_regmap_bus, st,
				      &ad5529r_regmap_config);
	if (IS_ERR(st->regmap))
		return dev_err_probe(dev, PTR_ERR(st->regmap),
				     "Failed to initialize regmap\n");

	ret = ad5529r_reset(st);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to reset device\n");

	indio_dev->name = st->model_data->model_name;
	indio_dev->info = &ad5529r_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = st->model_data->channels;
	indio_dev->num_channels = st->model_data->num_hw_channels;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register IIO device\n");

	dev_info(dev, "%s DAC probed successfully (%d channels)\n",
		 st->model_data->model_name, st->model_data->num_hw_channels);

	return 0;
}

static const struct of_device_id ad5529r_of_match[] = {
	{ .compatible = "adi,ad5529r", .data = &ad5529r_16bit_model_data },
	{ .compatible = "adi,ad5529r-12bit", .data = &ad5529r_12bit_model_data },
	{ }
};
MODULE_DEVICE_TABLE(of, ad5529r_of_match);

static const struct spi_device_id ad5529r_id[] = {
	{ "ad5529r", (kernel_ulong_t)&ad5529r_16bit_model_data },
	{ "ad5529r-12bit", (kernel_ulong_t)&ad5529r_12bit_model_data },
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
