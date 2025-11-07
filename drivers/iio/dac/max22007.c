// SPDX-License-Identifier: GPL-2.0
/*
 * max22007.c - MAX22007 DAC driver
 *
 * Driver for Analog Devices MAX22007 Digital to Analog Converter.
 *
 * Copyright (c) 2025 Analog Devices Inc.
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
#define     MAX22007_RW_MASK				BIT(0)
#define     MAX22007_CRC_OVERHEAD			1

/* Field value preparation macros with masking */
#define     MAX22007_CH_PWR_VAL(channel, val)	(((val) & 0x1) << (8 + (channel)))
#define     MAX22007_CH_MODE_VAL(channel, val)	(((val) & 0x1) << (12 + (channel)))
#define     MAX22007_DAC_LATCH_MODE_VAL(channel, val)	(((val) & 0x1) << (12 + (channel)))

static u8 max22007_crc8_table[256];

enum max22007_channel_mode {
	MAX22007_VOLTAGE_MODE,
	MAX22007_CURRENT_MODE
};

enum max22007_channel_power {
	MAX22007_CH_POWER_OFF,
	MAX22007_CH_POWER_ON,
};

struct max22007_state {
	struct spi_device *spi;
	struct regmap *regmap;
	struct iio_chan_spec *iio_chan;
	u8 tx_buf[4] __aligned(IIO_DMA_MINALIGN);
	u8 rx_buf[4];
};

static int max22007_spi_read(void *context, const void *reg, size_t reg_size,
			     void *val, size_t val_size)
{
	struct max22007_state *st = context;
	u8 calculated_crc, received_crc;
	u8 crc_data[3];
	int ret;
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
	};

	xfer.len = reg_size + val_size + MAX22007_CRC_OVERHEAD;

	memcpy(st->tx_buf, reg, reg_size);

	ret = spi_sync_transfer(st->spi, &xfer, 1);
	if (ret) {
		dev_err(&st->spi->dev, "SPI transfer failed: %d\n", ret);
		return ret;
	}

	crc_data[0] = st->tx_buf[0];
	crc_data[1] = st->rx_buf[1];
	crc_data[2] = st->rx_buf[2];

	calculated_crc = crc8(max22007_crc8_table, crc_data, 3, 0x00);
	received_crc = st->rx_buf[3];

	if (calculated_crc != received_crc) {
		dev_err(&st->spi->dev, "CRC mismatch on read register %02x:\n", *(u8 *)reg);
		return -EIO;
	}

	/* Ignore the dummy byte 0 */
	memcpy(val, &st->rx_buf[1], val_size);

	return 0;
}

static int max22007_spi_write(void *context, const void *data, size_t count)
{
	struct max22007_state *st = context;
	struct spi_transfer xfer = {
		.tx_buf = st->tx_buf,
		.rx_buf = st->rx_buf,
	};

	memset(st->tx_buf, 0, sizeof(st->tx_buf));

	xfer.len = count + MAX22007_CRC_OVERHEAD;

	memcpy(st->tx_buf, data, count);
	st->tx_buf[count] = crc8(max22007_crc8_table, st->tx_buf,
				 sizeof(st->tx_buf) - 1, 0x00);

	return spi_sync_transfer(st->spi, &xfer, 1);
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
	.read_flag_mask = MAX22007_RW_MASK,
	.reg_format_endian_default = REGMAP_ENDIAN_BIG,
	.val_format_endian_default = REGMAP_ENDIAN_BIG,
};

static const struct regmap_config max22007_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.reg_shift = -1,
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
				      int *data)
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
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = max22007_read_channel_data(st, chan->channel, val);
		if (ret)
			return ret;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_VOLTAGE) {
			*val = 5 * 2500;  /* 5 * Vref(2.5V) in mV */
			*val2 = 12;  /* 12-bit DAC resolution (2^12) */
		} else {
			*val = 25;  /* Vref / (2 * Rsense) = 2500mV / 100 */
			*val2 = 12;  /* 12-bit DAC resolution (2^12) */
		}
		return IIO_VAL_FRACTIONAL_LOG2;
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

static const struct iio_info max22007_info = {
	.read_raw = max22007_read_raw,
	.write_raw = max22007_write_raw,
};

static ssize_t max22007_read_dac_powerdown(struct iio_dev *indio_dev,
					   uintptr_t private,
					   const struct iio_chan_spec *chan,
					   char *buf)
{
	struct max22007_state *st = iio_priv(indio_dev);
	unsigned int reg_val;
	bool powerdown;
	int ret;

	ret = regmap_read(st->regmap, MAX22007_CHANNEL_MODE_REG, &reg_val);
	if (ret)
		return ret;

	powerdown = !(reg_val & MAX22007_CH_PWR_CH_MASK(chan->channel));

	return sysfs_emit(buf, "%d\n", powerdown);
}

static ssize_t max22007_write_dac_powerdown(struct iio_dev *indio_dev,
					    uintptr_t private,
					    const struct iio_chan_spec *chan,
					    const char *buf, size_t len)
{
	struct max22007_state *st = iio_priv(indio_dev);
	bool powerdown;
	int ret;

	ret = kstrtobool(buf, &powerdown);
	if (ret)
		return ret;

	if (powerdown)
		ret = regmap_update_bits(st->regmap, MAX22007_CHANNEL_MODE_REG,
					 MAX22007_CH_PWR_CH_MASK(chan->channel),
					 MAX22007_CH_PWR_VAL(chan->channel, MAX22007_CH_POWER_OFF));
	else
		ret = regmap_update_bits(st->regmap, MAX22007_CHANNEL_MODE_REG,
					 MAX22007_CH_PWR_CH_MASK(chan->channel),
					 MAX22007_CH_PWR_VAL(chan->channel, MAX22007_CH_POWER_ON));
	if (ret)
		return ret;

	return len;
}

static const struct iio_chan_spec_ext_info max22007_ext_info[] = {
	{
		.name = "powerdown",
		.read = max22007_read_dac_powerdown,
		.write = max22007_write_dac_powerdown,
		.shared = IIO_SEPARATE,
	},
	{ },
};

static const struct iio_chan_spec max22007_channel_template = {
	.output = 1,
	.indexed = 1,
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_SCALE),
	.ext_info = max22007_ext_info,
};

static int max22007_parse_channel_cfg(struct max22007_state *st, u8 *num_channels)
{
	struct device *dev = &st->spi->dev;
	struct iio_chan_spec *iio_chan;
	int ret, num_chan = 0, i = 0;
	u32 reg;

	num_chan = device_get_child_node_count(dev);
	if (!num_chan)
		return dev_err_probe(dev, -ENODEV, "no channels configured\n");

	st->iio_chan = devm_kcalloc(dev, num_chan, sizeof(*st->iio_chan), GFP_KERNEL);
	if (!st->iio_chan)
		return -ENOMEM;

	device_for_each_child_node_scoped(dev, child) {
		const char *channel_type_str;
		enum max22007_channel_mode mode;

		ret = fwnode_property_read_u32(child, "reg", &reg);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to read reg property of %pfwP\n", child);

		if (reg >= MAX22007_NUM_CHANNELS)
			return dev_err_probe(dev, -EINVAL,
					     "reg out of range in %pfwP\n", child);

		iio_chan = &st->iio_chan[i];

		*iio_chan = max22007_channel_template;
		iio_chan->channel = reg;

		ret = fwnode_property_read_string(child, "adi,type", &channel_type_str);
		if (ret)
			return dev_err_probe(dev, ret,
					     "missing adi,type property for %pfwP\n", child);

		if (strcmp(channel_type_str, "current") == 0) {
			mode = MAX22007_CURRENT_MODE;
			iio_chan->type = IIO_CURRENT;
		} else if (strcmp(channel_type_str, "voltage") == 0) {
			mode = MAX22007_VOLTAGE_MODE;
			iio_chan->type = IIO_VOLTAGE;
		} else {
			return dev_err_probe(dev, -EINVAL,
					     "invalid adi,type '%s' for %pfwP\n",
					     channel_type_str, child);
		}

		ret = regmap_update_bits(st->regmap, MAX22007_CHANNEL_MODE_REG,
					 MAX22007_CH_MODE_CH_MASK(reg),
					 MAX22007_CH_MODE_VAL(reg, mode));
		if (ret)
			return ret;

		/* Set DAC to transparent mode (immediate update) */
		ret = regmap_update_bits(st->regmap, MAX22007_CONFIG_REG,
					 MAX22007_DAC_LATCH_MODE_MASK(reg),
					 MAX22007_DAC_LATCH_MODE_VAL(reg, 1));
		if (ret)
			return ret;

		i++;
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

	crc8_populate_lsb(max22007_crc8_table, MAX22007_CRC8_POLYNOMIAL);

	state->regmap = devm_regmap_init(&spi->dev, &max22007_regmap_bus, state,
					 &max22007_regmap_config);
	if (IS_ERR(state->regmap))
		return dev_err_probe(&spi->dev, PTR_ERR(state->regmap),
				     "Failed to initialize regmap\n");

	reset_gpio = devm_gpiod_get_optional(&spi->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(&spi->dev, PTR_ERR(reset_gpio),
				     "Failed to get reset GPIO\n");

	if (reset_gpio) {
		gpiod_set_value_cansleep(reset_gpio, 0);
	} else {
		ret = regmap_write(state->regmap, MAX22007_SOFT_RESET_REG,
				   MAX22007_SOFT_RESET_BITS_MASK);
		if (ret)
			return ret;
	}

	ret = regmap_update_bits(state->regmap, MAX22007_CONFIG_REG,
				 MAX22007_CRC_EN_MASK,
				 MAX22007_CRC_EN_MASK);
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
	{ "max22007" },
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
