// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * max22007.c - MAX22007 DAC driver
 *
 * Driver for Analog Devices MAX22007 Digital to Analog Converter.
 *
 * Copyright (c) 2025
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/crc8.h>
#include <linux/dev_printk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/iio.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define MAX22007_NUM_CHANNELS				4
#define MAX22007_REV_ID_REG				0x00
#define MAX22007_STAT_INTR_REG				0x01
#define MAX22007_INTERRUPT_EN_REG			0x02
#define MAX22007_CONFIG_REG				0x03
#define MAX22007_CONTROL_REG				0x04
#define MAX22007_CHANNEL_MODE_REG			0x05
#define MAX22007_SOFT_RESET_REG				0x06
#define MAX22007_DAC_CHANNEL_REG(ch)			(0x07 + (ch))
#define MAX22007_GPIO_CTRL_REG				0x0B
#define MAX22007_GPIO_DATA_REG				0x0C
#define MAX22007_GPI_EDGE_INT_CTRL_REG			0x0D
#define MAX22007_GPI_INT_STATUS_REG			0x0E

/* Channel mask definitions */
#define     MAX22007_CH_MODE_CH_MASK(channel)		BIT(12 + (channel))
#define     MAX22007_CH_PWR_CH_MASK(channel)		BIT(8 + (channel))
#define     MAX22007_DAC_LATCH_MODE_MASK(channel)	BIT(12 + (channel))
#define     MAX22007_LDAC_UPDATE_MASK(channel)		BIT(12 + (channel))
#define     MAX22007_SW_RST_MASK			BIT(8)
#define     MAX22007_SW_CLR_MASK			BIT(12)
#define     MAX22007_SOFT_RESET_BITS_MASK		(MAX22007_SW_RST_MASK | \
	    MAX22007_SW_CLR_MASK)
#define     MAX22007_DAC_DATA_MASK			GENMASK(15, 4)
#define     MAX22007_DAC_MAX_RAW			GENMASK(11, 0)
#define     MAX22007_CRC8_POLYNOMIAL			0x8C
#define     MAX22007_CRC_EN_MASK			BIT(0)
#define     MAX22007_REG_ADDR_MASK			GENMASK(7, 1)
#define     MAX22007_RW_MASK				BIT(0)

/* Field value preparation macros with masking */
#define     MAX22007_CH_PWR_VAL(channel, val)	(((val) & 0x1) << (8 + (channel)))
#define     MAX22007_CH_MODE_VAL(channel, val)	(((val) & 0x1) << (12 + (channel)))
#define     MAX22007_DAC_LATCH_MODE_VAL(channel, val)	(((val) & 0x1) << (12 + (channel)))

static u8 max22007_crc8_table[256];

enum max22007_channel_mode {
	MAX22007_VOLTAGE_MODE = 0,
	MAX22007_CURRENT_MODE = 1,
};

enum max22007_dac_latch_mode {
	LDAC_CONTROL = 0,
	TRANSPARENT_LATCH = 1
};

enum max22007_channel_power {
	MAX22007_CH_POWER_OFF = 0,
	MAX22007_CH_POWER_ON = 1,
};

static const char * const max22007_power_modes[] = {
	[MAX22007_CH_POWER_OFF] = "off",
	[MAX22007_CH_POWER_ON] = "on",
};

struct max22007_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct iio_chan_spec *iio_chan;
	bool crc_en;
	u8 tx_buf[4] __aligned(IIO_DMA_MINALIGN);
	u8 rx_buf[4];
};

static ssize_t max22007_write_ldac_update(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf, size_t len);

static int max22007_power_mode_read(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan);

static int max22007_power_mode_write(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int item);

static const struct iio_enum max22007_power_mode_enum = {
	.items = max22007_power_modes,
	.num_items = ARRAY_SIZE(max22007_power_modes),
	.set = max22007_power_mode_write,
	.get = max22007_power_mode_read,
};

static const struct iio_chan_spec_ext_info max22007_ext_info[] = {
	{
		.name = "ldac_update",
		.write = max22007_write_ldac_update,
		.shared = IIO_SEPARATE,
	},
	IIO_ENUM("powermode", IIO_SEPARATE, &max22007_power_mode_enum),
	IIO_ENUM_AVAILABLE("powermode", IIO_SEPARATE, &max22007_power_mode_enum),
	{ },
};

static int max22007_spi_reg_read(struct max22007_state *st, unsigned int reg, unsigned int *val)
{
	u8 calculated_crc, received_crc;
	u8 crc_data[3];
	int ret;
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
		.len = 3,
	};

	memset(st->tx_buf, 0, sizeof(st->tx_buf));
	memset(st->rx_buf, 0, sizeof(st->rx_buf));

	if (st->crc_en)
		xfer.len += 1;

	st->tx_buf[0] = FIELD_PREP(MAX22007_REG_ADDR_MASK, reg) |
			FIELD_PREP(MAX22007_RW_MASK, 1);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "SPI transfer failed: %d\n", ret);
		return ret;
	}

	if (st->crc_en) {
		crc_data[0] = st->tx_buf[0];
		crc_data[1] = st->rx_buf[1];
		crc_data[2] = st->rx_buf[2];

		calculated_crc = crc8(max22007_crc8_table, crc_data, 3, 0x00);
		received_crc = st->rx_buf[3];

		if (calculated_crc != received_crc) {
			dev_warn(&st->spi->dev, "CRC mismatch on read register %02x:\n", reg);
			return -EIO;
		}
	}

	/* Ignore the dummy byte 0 */
	*val = get_unaligned_be16(&st->rx_buf[1]);

	return 0;
}

static int max22007_spi_reg_write(struct max22007_state *st, unsigned int reg, unsigned int val)
{
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->tx_buf,
		.len = 3,
	};

	memset(st->tx_buf, 0, sizeof(st->tx_buf));

	if (st->crc_en)
		xfer.len += 1;

	st->tx_buf[0] = FIELD_PREP(MAX22007_REG_ADDR_MASK, reg) |
			FIELD_PREP(MAX22007_RW_MASK, 0);
	st->tx_buf[1] = (val >> 8) & 0xFF;
	st->tx_buf[2] = val;
	if (st->crc_en)
		st->tx_buf[3] = crc8(max22007_crc8_table, st->tx_buf, 3, 0x00);

	return spi_sync_transfer(st->spi, &xfer, 1);
}

static int max22007_spi_read(void *context, const void *reg, size_t reg_size,
			     void *val, size_t val_size)
{
	struct max22007_state *st = context;
	unsigned int reg_addr = *(u8 *)reg;
	unsigned int read_val;
	int ret;

	ret = max22007_spi_reg_read(st, reg_addr, &read_val);
	if (ret)
		return ret;

	/* Store as big endian */
	put_unaligned_be16(read_val, val);
	return 0;
}

static int max22007_spi_write(void *context, const void *data, size_t count)
{
	struct max22007_state *st = context;
	u8 reg;
	u16 val;

	if (count != 3)
		return -EINVAL;

	reg = *(u8 *)data;
	val = get_unaligned_be16((u8 *)data + 1);

	return max22007_spi_reg_write(st, reg, val);
}

static bool max22007_reg_readable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX22007_REV_ID_REG:
	case MAX22007_STAT_INTR_REG:
	case MAX22007_CONFIG_REG:
	case MAX22007_CONTROL_REG:
	case MAX22007_CHANNEL_MODE_REG:
	case MAX22007_SOFT_RESET_REG:
	case MAX22007_GPIO_CTRL_REG:
	case MAX22007_GPIO_DATA_REG:
	case MAX22007_GPI_EDGE_INT_CTRL_REG:
	case MAX22007_GPI_INT_STATUS_REG:
		return true;
	case MAX22007_DAC_CHANNEL_REG(0) ... MAX22007_DAC_CHANNEL_REG(MAX22007_NUM_CHANNELS - 1):
		return true;
	default:
		return false;
	}
}

static bool max22007_reg_writable(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case MAX22007_CONFIG_REG:
	case MAX22007_CONTROL_REG:
	case MAX22007_CHANNEL_MODE_REG:
	case MAX22007_SOFT_RESET_REG:
	case MAX22007_GPIO_CTRL_REG:
	case MAX22007_GPIO_DATA_REG:
	case MAX22007_GPI_EDGE_INT_CTRL_REG:
		return true;
	case MAX22007_DAC_CHANNEL_REG(0) ... MAX22007_DAC_CHANNEL_REG(MAX22007_NUM_CHANNELS - 1):
		return true;
	default:
		return false;
	}
}

static const struct regmap_bus max22007_regmap_bus = {
	.read = max22007_spi_read,
	.write = max22007_spi_write,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config max22007_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.readable_reg = max22007_reg_readable,
	.writeable_reg = max22007_reg_writable,
	.max_register = 0x0E,
};

static int max22007_write_channel_data(struct max22007_state *state, unsigned int channel,
				       unsigned int data)
{
	unsigned int reg_val;

	if (data > MAX22007_DAC_MAX_RAW)
		return -EINVAL;

	reg_val = FIELD_PREP(MAX22007_DAC_DATA_MASK, data);

	return regmap_write(state->regmap, MAX22007_DAC_CHANNEL_REG(channel), reg_val);
}

static int max22007_read_channel_data(struct max22007_state *state, unsigned int channel,
				      unsigned int *data)
{
	int ret;
	unsigned int reg_val;

	ret = regmap_read(state->regmap, MAX22007_DAC_CHANNEL_REG(channel), &reg_val);
	if (ret)
		return ret;

	*data = FIELD_GET(MAX22007_DAC_DATA_MASK, reg_val);

	return 0;
}

static int max22007_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max22007_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = max22007_read_channel_data(st, chan->channel, &reg_val);
		if (ret)
			return ret;
		*val = reg_val;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			*val = 5 * 2500;  /* 5 * Vref(2.5V) in mV */
			*val2 = 4096 * 1000;  /* 12-bit DAC resolution * 1000 to convert mV to V */
		} else {
			*val = 2500;  /* Vref in mV */
			*val2 = 2 * 50 * 4096 * 1000;  /* 2 * Rsense * 4096 * 1000 to convert mA to A */
		}
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int max22007_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct max22007_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return max22007_write_channel_data(st, chan->channel, val);
	default:
		return -EINVAL;
	}
}

static int max22007_ldac_update_channel(struct max22007_state *state, unsigned int channel)
{
	if (channel >= MAX22007_NUM_CHANNELS)
		return -EINVAL;

	return regmap_write(state->regmap, MAX22007_CONTROL_REG,
			    MAX22007_LDAC_UPDATE_MASK(channel));
}

static ssize_t max22007_write_ldac_update(struct iio_dev *indio_dev,
					  uintptr_t private,
					  const struct iio_chan_spec *chan,
					  const char *buf, size_t len)
{
	struct max22007_state *st = iio_priv(indio_dev);
	bool update;
	int ret;

	ret = kstrtobool(buf, &update);
	if (ret)
		return ret;

	if (!update)
		return len;

	ret = max22007_ldac_update_channel(st, chan->channel);
	if (ret)
		return ret;

	return len;
}

static const struct iio_info max22007_info = {
	.read_raw = max22007_read_raw,
	.write_raw = max22007_write_raw,
};

static int max22007_set_power_mode(struct max22007_state *state, unsigned int channel,
				   enum max22007_channel_power power_mode)
{
	return regmap_update_bits(state->regmap, MAX22007_CHANNEL_MODE_REG,
				  MAX22007_CH_PWR_CH_MASK(channel),
				  MAX22007_CH_PWR_VAL(channel, power_mode));
}

static int max22007_set_channel_mode(struct max22007_state *state, unsigned int channel,
				     enum max22007_channel_mode channel_mode)
{
	return regmap_update_bits(state->regmap, MAX22007_CHANNEL_MODE_REG,
				  MAX22007_CH_MODE_CH_MASK(channel),
				  MAX22007_CH_MODE_VAL(channel, channel_mode));
}

static int max22007_set_dac_latch_mode(struct max22007_state *state, unsigned int channel,
				       enum max22007_dac_latch_mode latch_mode)
{
	return regmap_update_bits(state->regmap, MAX22007_CONFIG_REG,
				  MAX22007_DAC_LATCH_MODE_MASK(channel),
				  MAX22007_DAC_LATCH_MODE_VAL(channel, latch_mode));
}

static int max22007_configure_crc(struct max22007_state *state)
{
	bool crc;
	int ret;

	crc = !device_property_read_bool(&state->spi->dev, "adi,crc-disable");

	ret = regmap_update_bits(state->regmap, MAX22007_CONFIG_REG,
				 MAX22007_CRC_EN_MASK,
				 crc ? MAX22007_CRC_EN_MASK : 0);
	if (ret)
		return ret;

	state->crc_en = crc;

	return 0;
}

static int max22007_power_mode_read(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct max22007_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	int ret;

	ret = regmap_read(st->regmap, MAX22007_CHANNEL_MODE_REG, &reg_val);
	if (ret)
		return ret;

	if (reg_val & MAX22007_CH_PWR_CH_MASK(chan->channel))
		return MAX22007_CH_POWER_ON;
	else
		return MAX22007_CH_POWER_OFF;
}

static int max22007_power_mode_write(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     unsigned int item)
{
	struct max22007_state *st = iio_priv(indio_dev);

	return max22007_set_power_mode(st, chan->channel, item);
}

static int max22007_parse_channel_cfg(struct max22007_state *st, u8 *num_channels)
{
	static const char * const latch_mode_strings[] = { "ldac-control", "transparent" };
	static const char * const mode_strings[] = { "voltage", "current" };
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *iio_chan;
	int ret, num_chan = 0, i = 0;
	int latch_mode_idx, mode_idx;
	u32 reg;

	num_chan = device_get_child_node_count(dev);
	if (!num_chan)
		return dev_err_probe(dev, -ENODEV, "no channels configured\n");

	st->iio_chan = devm_kcalloc(dev, num_chan, sizeof(*st->iio_chan), GFP_KERNEL);
	if (!st->iio_chan)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret,
				"failed to read reg property of %pfwP\n", child);

		if (reg >= MAX22007_NUM_CHANNELS)
			return dev_err_probe(dev, -EINVAL,
				"reg out of range in %pfwP\n", child);

		iio_chan = &st->iio_chan[i];

		iio_chan->type = IIO_VOLTAGE;
		iio_chan->output = 1;
		iio_chan->indexed = 1;
		iio_chan->channel = reg;
		iio_chan->info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE);
		iio_chan->ext_info = max22007_ext_info;
		i++;

		latch_mode_idx = fwnode_property_match_property_string(child, "adi,dac-latch-mode",
								       latch_mode_strings,
								       ARRAY_SIZE(latch_mode_strings));
		if (latch_mode_idx < 0 && latch_mode_idx != -EINVAL)
			return dev_err_probe(dev, latch_mode_idx,
				"failed to read adi,dac-latch-mode of %pfwP\n", child);
		if (latch_mode_idx >= 0) {
			ret = max22007_set_dac_latch_mode(st, reg, latch_mode_idx);
			if (ret)
				return ret;
		}

		mode_idx = fwnode_property_match_property_string(child, "adi,mode", mode_strings,
								 ARRAY_SIZE(mode_strings));
		if (mode_idx < 0 && mode_idx != -EINVAL)
			return dev_err_probe(dev, mode_idx,
				"failed to read adi,mode of %pfwP\n", child);
		if (mode_idx >= 0) {
			ret = max22007_set_channel_mode(st, reg, mode_idx);
			if (ret)
				return ret;

			if (mode_idx == MAX22007_CURRENT_MODE)
				iio_chan->type = IIO_CURRENT;
			else
				iio_chan->type = IIO_VOLTAGE;
		}
	}

	*num_channels = num_chan;

	return 0;
}

static int max22007_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct max22007_state *state;
	struct gpio_desc *reset_gpio;
	u8 num_channels;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*state));
	if (!indio_dev)
		return -ENOMEM;

	state = iio_priv(indio_dev);
	state->spi = spi;
	state->crc_en = true;

	crc8_populate_lsb(max22007_crc8_table, MAX22007_CRC8_POLYNOMIAL);

	state->regmap = devm_regmap_init(&spi->dev, &max22007_regmap_bus, state,
					 &max22007_regmap_config);
	if (IS_ERR(state->regmap)) {
		dev_err(&spi->dev, "Failed to initialize regmap\n");
		return PTR_ERR(state->regmap);
	}

	reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(&spi->dev, PTR_ERR(reset_gpio),
				     "Failed to get reset GPIO\n");

	if (reset_gpio) {
		ret = gpiod_direction_output(reset_gpio, 1);
		if (ret) {
			dev_err(&spi->dev, "Failed to set GPIO as output: %d\n", ret);
			return ret;
		}
		gpiod_set_value_cansleep(reset_gpio, 0);
	} else {
		ret = max22007_spi_reg_write(state, MAX22007_SOFT_RESET_REG,
					     MAX22007_SOFT_RESET_BITS_MASK);
		if (ret)
			return ret;
	}

	ret = max22007_configure_crc(state);
	if (ret)
		return ret;

	ret = max22007_parse_channel_cfg(state, &num_channels);
	if (ret)
		return ret;

	indio_dev->info = &max22007_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = state->iio_chan;
	indio_dev->num_channels = num_channels;
	indio_dev->name = "max22007";

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id max22007_id[] = {
	{"max22007"},
	{ }
};
MODULE_DEVICE_TABLE(spi, max22007_id);

static const struct of_device_id max22007_of_match[] = {
	{ .compatible = "adi,max22007" },
	{ }
};
MODULE_DEVICE_TABLE(of, max22007_of_match);

static struct spi_driver max22007_driver = {
	.driver = {
		.name = "max22007",
		.of_match_table = max22007_of_match,
	},
	.probe = max22007_probe,
	.id_table = max22007_id,
};
module_spi_driver(max22007_driver);

MODULE_AUTHOR("Janani Sunil <janani.sunil@analog.com>");
MODULE_DESCRIPTION("Analog Devices MAX22007 DAC");
MODULE_LICENSE("GPL");
