// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2021 Analog Devices, Inc.
 * Author: Cosmin Tanislav <cosmin.tanislav@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/driver.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>

#include <dt-bindings/iio/addac/adi,ad74413r.h>

#define AD74413R_CRC_POLYNOMIAL	0x7
DECLARE_CRC8_TABLE(ad74413r_crc8_table);

#define AD74413R_CHANNEL_MAX	4

struct ad74413r_config {
	bool hart_support;
};

struct ad74413r_channel_config {
	u32 gpo_config;
	u32 func;
	bool initialized;
};

struct ad74413r_state {
	struct ad74413r_channel_config	channel_configs[AD74413R_CHANNEL_MAX];
	struct gpio_chip		gpiochip;
	struct mutex			lock;
	struct completion		adc_data_completion;

	struct spi_device		*spi;
	struct iio_dev			*indio_dev;
	struct regulator		*refin_reg;
	struct regmap			*regmap;
	struct device			*dev;
	struct iio_trigger		*trig;
	const struct ad74413r_config	*config;

	u32				rsense_resistance_ohms;

	size_t				adc_active_channels;
	struct spi_message		adc_samples_msg;
	struct spi_transfer		adc_samples_xfer[AD74413R_CHANNEL_MAX + 1];
	struct {
		__be32 rx_buf[AD74413R_CHANNEL_MAX];
		s64 timestamp;
	} adc_samples ____cacheline_aligned;
	__be32				adc_samples_tx_buf[AD74413R_CHANNEL_MAX];
	__be32				regmap_tx_buf;
	__be32				regmap_rx_buf;
};

#define AD74413R_REG_NOP		0x00

#define AD74413R_REG_CH_FUNC_SETUP_X(x)	(0x01 + (x))
#define AD74413R_CH_FUNC_SETUP_MASK	GENMASK(3, 0)

#define AD74413R_REG_ADC_CONFIG_X(x)		(0x05 + (x))
#define AD74413R_ADC_CONFIG_RANGE_MASK		GENMASK(7, 5)
#define AD74413R_ADC_CONFIG_REJECTION_MASK	GENMASK(4, 3)
#define AD74413R_ADC_RANGE_10V			0b000
#define AD74413R_ADC_RANGE_2P5V_EXT_POW		0b001
#define AD74413R_ADC_RANGE_2P5V_INT_POW		0b010
#define AD74413R_ADC_RANGE_5V_BI_DIR		0b011
#define AD74413R_ADC_REJECTION_50_60		0b00
#define AD74413R_ADC_REJECTION_NONE		0b01
#define AD74413R_ADC_REJECTION_50_60_HART	0b10
#define AD74413R_ADC_REJECTION_HART		0b11

#define AD74413R_REG_DAC_CODE_X(x)	(0x16 + (x))
#define AD74413R_DAC_CODE_MAX		((1 << 13) - 1)

#define AD74413R_REG_GPO_PAR_DATA		0x0d
#define AD74413R_REG_GPO_CONFIG_X(x)		(0x0e + (x))
#define AD74413R_GPO_CONFIG_GPO_DATA_MASK	BIT(3)
#define AD74413R_GPO_CONFIG_DATA_LOW		0
#define AD74413R_GPO_CONFIG_DATA_HIGH		AD74413R_GPO_CONFIG_GPO_DATA_MASK
#define AD74413R_GPO_CONFIG_GPO_SELECT_MASK	GENMASK(2, 0)

#define AD74413R_REG_ADC_CONV_CTRL	0x23
#define AD74413R_CONV_SEQ_MASK		GENMASK(9, 8)
#define AD74413R_CONV_SEQ_ON		0b00
#define AD74413R_CONV_SEQ_SINGLE	0b01
#define AD74413R_CONV_SEQ_CONTINUOUS	0b10
#define AD74413R_CONV_SEQ_OFF		0b11
#define AD74413R_CH_EN_MASK(x)		BIT(x)
#define AD74413R_CH_EN_SHIFT(x)		x

#define AD74413R_REG_DIN_COMP_OUT		0x25
#define AD74413R_DIN_COMP_OUT_MASK_X(x)		BIT(x)
#define AD74413R_DIN_COMP_OUT_SHIFT_X(x)	x

#define AD74413R_REG_ADC_RESULT_X(x)	(0x26 + (x))
#define AD74413R_ADC_RESULT_MAX		((1 << 16) - 1)

#define AD74413R_REG_READ_SELECT	0x41

#define AD74413R_REG_CMD_KEY		0x44
#define AD74413R_CMD_KEY_LDAC		0x953a

#define AD74413R_REG_SILLICON_REV		0x46
#define AD74413R_SILLICON_REV_ID_MASK		GENMASK(7, 0)
#define AD74413R_SILLICON_REV_ID_EXPECTED	0x8

#define AD74413R_ADC_DATA_TIMEOUT	1000

static const int ad74413r_adc_sampling_rates[] = {
	20,
	4800,
};

static const int ad74413r_adc_sampling_rates_hart[] = {
	10,
	20,
	1200,
	4800,
};

static int ad74413r_crc(u8 *buf)
{
	return crc8(ad74413r_crc8_table, buf, 3, 0);
}

static void ad74413r_format_reg_write(unsigned int reg, unsigned int val, u8 *buf)
{
	buf[0] = reg & 0xff;
	buf[1] = (val >> 8) & 0xff;
	buf[2] = val & 0xff;
	buf[3] = ad74413r_crc(buf);
}

static int ad74413r_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ad74413r_state *st = spi_get_drvdata(spi);
	u8 *buf = (u8 *)&st->regmap_tx_buf;

	ad74413r_format_reg_write(reg, val, buf);

	return spi_write(spi, buf, 4);
}

static int ad74413r_crc_check(struct ad74413r_state *st, u8 *buf)
{
	u8 expected_crc = ad74413r_crc(buf);

	if (buf[3] != expected_crc) {
		dev_err(st->dev, "Bad CRC, data: %02x%02x%02x, expected: %02x, received: %02x\n",
				buf[0], buf[1], buf[2], expected_crc, buf[3]);
		return -EINVAL;
	}

	return 0;
}

static int ad74413r_reg_read_raw(void *context, unsigned int reg, u8 *buf,
				 unsigned int count)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ad74413r_state *st = spi_get_drvdata(spi);
	unsigned int i;
	int ret;

	ret = ad74413r_reg_write(context, AD74413R_REG_READ_SELECT, reg);
	if (ret) {
		dev_err(dev, "Failed to write read select register: %d\n", ret);
		return ret;
	}

	for (i = 0; i < count; i++, buf += 4) {
		ret = spi_read(spi, buf, 4);
		if (ret)
			return ret;

		ret = ad74413r_crc_check(st, buf);
		if (ret)
			return ret;
	}

	return 0;
}

static int ad74413r_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct device *dev = context;
	struct spi_device *spi = to_spi_device(dev);
	struct ad74413r_state *st = spi_get_drvdata(spi);
	u8 *buf = (u8 *)&st->regmap_rx_buf;
	int ret;

	ret = ad74413r_reg_read_raw(context, reg, buf, 1);
	if (ret)
		return ret;

	*val = (buf[1] << 8) | buf[2];

	return 0;
}

const struct regmap_config ad74413r_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.reg_read = ad74413r_reg_read,
	.reg_write = ad74413r_reg_write,
};

static int ad74413r_set_gpo_mode(struct ad74413r_state *st, unsigned int offset, u8 mode)
{
	if (mode < GPO_CONFIG_MIN || mode > GPO_CONFIG_MAX) {
		dev_err(st->dev, "Invalid gpo config select mode\n");
		return -EINVAL;
	}

	return regmap_update_bits(st->regmap, AD74413R_REG_GPO_CONFIG_X(offset),
				  AD74413R_GPO_CONFIG_GPO_SELECT_MASK, mode);
}

static void ad74413r_gpio_set(struct gpio_chip *chip, unsigned int offset, int val)
{
	struct ad74413r_state *st = gpiochip_get_data(chip);
	struct ad74413r_channel_config *channel_config = &st->channel_configs[offset];
	int ret;

	if (channel_config->gpo_config != GPO_CONFIG_LOGIC) {
		dev_err(st->dev, "Cannot set gpo %u, not in logic mode", offset);
		return;
	}

	ret = ad74413r_set_gpo_mode(st, offset, GPO_CONFIG_LOGIC);
	if (ret)
		return;

	regmap_update_bits(st->regmap, AD74413R_REG_GPO_CONFIG_X(offset),
			   AD74413R_GPO_CONFIG_GPO_DATA_MASK,
			   val ? AD74413R_GPO_CONFIG_DATA_HIGH
			       : AD74413R_GPO_CONFIG_DATA_LOW);
}

static void ad74413r_gpio_set_multiple(struct gpio_chip *chip, unsigned long *mask,
				       unsigned long *bits)
{
	struct ad74413r_state *st = gpiochip_get_data(chip);
	unsigned int offset = 0;
	int ret;

	for_each_set_bit_from(offset, mask, AD74413R_CHANNEL_MAX) {
		struct ad74413r_channel_config *channel_config = &st->channel_configs[offset];

		if (channel_config->gpo_config != GPO_CONFIG_LOGIC) {
			dev_err(st->dev, "Cannot set gpo %u, not in logic mode\n", offset);
			continue;
		}

		ret = ad74413r_set_gpo_mode(st, offset, GPO_CONFIG_LOGIC_PARALLEL);
		if (ret)
			return;
	}

	regmap_update_bits(st->regmap, AD74413R_REG_GPO_PAR_DATA, *mask, *bits);
}

static int ad74413r_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct ad74413r_state *st = gpiochip_get_data(chip);
	struct ad74413r_channel_config *channel_config = &st->channel_configs[offset];
	unsigned int status;
	int ret;

	if (channel_config->gpo_config != GPO_CONFIG_DEBOUNCED_COMPARATOR) {
		dev_err(st->dev, "Cannot get gpo %u, not in comparator mode\n", offset);
		return -EINVAL;
	}

	ret = regmap_read(st->regmap, AD74413R_REG_DIN_COMP_OUT, &status);
	if (ret)
		return ret;

	status &= AD74413R_DIN_COMP_OUT_MASK_X(offset);
	status >>= AD74413R_DIN_COMP_OUT_SHIFT_X(offset);

	return status;
}

static int ad74413r_set_channel_dac_code(struct ad74413r_state *st, unsigned int channel,
					 uint16_t dac_code)
{
	int ret;

	if (dac_code > AD74413R_DAC_CODE_MAX) {
		dev_err(st->dev, "Invalid dac code %d\n", dac_code);
		return -EINVAL;
	}

	ret = regmap_write(st->regmap, AD74413R_REG_DAC_CODE_X(channel), dac_code);
	if (ret)
		return ret;

	ret = regmap_write(st->regmap, AD74413R_REG_CMD_KEY, AD74413R_CMD_KEY_LDAC);
	if (ret)
		return ret;

	return 0;
}

static int ad74413r_channel_set_function(struct ad74413r_state *st, unsigned int channel, u8 func)
{
	if (func < CH_FUNC_MIN || func > CH_FUNC_MAX) {
		dev_err(st->dev, "Invalid channel function %d\n", func);
		return -EINVAL;
	}

	if (!st->config->hart_support && (func == CH_FUNC_CURRENT_INPUT_EXT_POWER_HART
		|| func == CH_FUNC_CURRENT_INPUT_LOOP_POWER_HART)) {
		dev_err(st->dev, "HART function not supported %d\n", func);
		return -EINVAL;
	}

	return regmap_update_bits(st->regmap, AD74413R_REG_CH_FUNC_SETUP_X(channel),
				  AD74413R_CH_FUNC_SETUP_MASK, func);
}

static int ad74413r_set_adc_conv_seq(struct ad74413r_state *st, unsigned int status)
{
	int ret;

	/*
	 * These bits do not clear when a conversion completes.
	 * To enable a subsequent conversion, repeat the write to enable the conversion.
	 */
	ret = regmap_write_bits(st->regmap, AD74413R_REG_ADC_CONV_CTRL,
				AD74413R_CONV_SEQ_MASK,
				FIELD_PREP(AD74413R_CONV_SEQ_MASK, status));
	if (ret)
		return ret;

	// Wait 100us before starting conversions
	usleep_range(100, 120);

	return 0;
}

static int ad74413r_set_adc_channel_enable(struct ad74413r_state *st, unsigned int channel,
					   bool status)
{
	unsigned int val = status << AD74413R_CH_EN_SHIFT(channel);

	return regmap_update_bits(st->regmap, AD74413R_REG_ADC_CONV_CTRL,
				  AD74413R_CH_EN_MASK(channel), val);
}

static int ad74413r_get_adc_range(struct ad74413r_state *st, unsigned int channel,
				  unsigned int *val)
{
	int ret;

	ret = regmap_read(st->regmap, AD74413R_REG_ADC_CONFIG_X(channel), val);
	if (ret)
		return ret;

	*val = FIELD_GET(AD74413R_ADC_CONFIG_RANGE_MASK, *val);

	return 0;
}

static int ad74413r_get_adc_rejection(struct ad74413r_state *st, unsigned int channel,
				      unsigned int *val)
{
	int ret;

	ret = regmap_read(st->regmap, AD74413R_REG_ADC_CONFIG_X(channel), val);
	if (ret)
		return ret;

	*val = FIELD_GET(AD74413R_ADC_CONFIG_REJECTION_MASK, *val);

	return 0;
}

static int ad74413r_set_adc_rejection(struct ad74413r_state *st, unsigned int channel,
				      unsigned int val)
{
	if (!st->config->hart_support && (val == AD74413R_ADC_REJECTION_50_60_HART
		|| val == AD74413R_ADC_REJECTION_HART)) {
		dev_err(st->dev, "HART rate not supported %d\n", val);
		return -EINVAL;
	}

	return regmap_update_bits(st->regmap, AD74413R_REG_ADC_CONFIG_X(channel),
				  AD74413R_ADC_CONFIG_REJECTION_MASK,
				  FIELD_PREP(AD74413R_ADC_CONFIG_REJECTION_MASK, val));
}

static int ad74413r_get_adc_rate(struct ad74413r_state *st, unsigned int channel, int *val)
{
	unsigned int rej;
	int ret;

	ret = ad74413r_get_adc_rejection(st, channel, &rej);
	if (ret)
		return ret;

	switch (rej) {
	case AD74413R_ADC_REJECTION_50_60:
		*val = 20;
		break;
	case AD74413R_ADC_REJECTION_NONE:
		*val = 4800;
		break;
	case AD74413R_ADC_REJECTION_50_60_HART:
		*val = 10;
		break;
	case AD74413R_ADC_REJECTION_HART:
		*val = 1200;
		break;
	default:
		dev_err(st->dev, "Channel %u ADC rejection invalid\n", channel);
		return -EINVAL;
	}

	return IIO_VAL_INT;
}

static int ad74413r_set_adc_rate(struct ad74413r_state *st, unsigned int channel, int val)
{
	unsigned int rej;

	switch (val) {
	case 20:
		rej = AD74413R_ADC_REJECTION_50_60;
		break;
	case 4800:
		rej = AD74413R_ADC_REJECTION_NONE;
		break;
	case 10:
		rej = AD74413R_ADC_REJECTION_50_60_HART;
		break;
	case 1200:
		rej = AD74413R_ADC_REJECTION_HART;
		break;
	default:
		dev_err(st->dev, "Channel %u ADC rate invalid\n", channel);
		return -EINVAL;
	}

	return ad74413r_set_adc_rejection(st, channel, rej);
}

static int ad74413r_get_adc_voltage_range(struct ad74413r_state *st, unsigned int channel,
					  int *val)
{
	unsigned int range;
	int ret;

	ret = ad74413r_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	switch (range) {
	case AD74413R_ADC_RANGE_10V:
		*val = 10000;
		break;
	case AD74413R_ADC_RANGE_2P5V_EXT_POW:
	case AD74413R_ADC_RANGE_2P5V_INT_POW:
		*val = 2500;
		break;
	case AD74413R_ADC_RANGE_5V_BI_DIR:
		*val = 5000;
		break;
	default:
		dev_err(st->dev, "Channel %u ADC range invalid\n", channel);
		return -EINVAL;
	}

	return 0;
}

static int ad74413r_get_adc_voltage_offset(struct ad74413r_state *st, unsigned int channel,
					   int *val)
{
	unsigned int range;
	int ret;

	ret = ad74413r_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	switch (range) {
	case AD74413R_ADC_RANGE_10V:
	case AD74413R_ADC_RANGE_2P5V_EXT_POW:
		*val = 0;
		break;
	case AD74413R_ADC_RANGE_2P5V_INT_POW:
	case AD74413R_ADC_RANGE_5V_BI_DIR:
		*val = -2500;
		break;
	default:
		dev_err(st->dev, "Channel %u ADC range invalid\n", channel);
		return -EINVAL;
	}

	return 0;
}

static int ad74413r_get_input_voltage_scale(struct ad74413r_state *st, unsigned int channel,
					    int *val, int *val2)
{
	int ret;

	ret = ad74413r_get_adc_voltage_range(st, channel, val);
	if (ret)
		return ret;

	*val2 = AD74413R_ADC_RESULT_MAX;

	return IIO_VAL_FRACTIONAL;
}

static int ad74413r_get_input_voltage_offset(struct ad74413r_state *st, unsigned int channel,
					     int *val, int *val2)
{
	unsigned int range;
	int ret;

	ret = ad74413r_get_adc_range(st, channel, &range);
	if (ret)
		return ret;

	switch (range) {
	case AD74413R_ADC_RANGE_10V:
	case AD74413R_ADC_RANGE_2P5V_EXT_POW:
		*val = 0;
		break;
	case AD74413R_ADC_RANGE_2P5V_INT_POW:
		*val = -AD74413R_ADC_RESULT_MAX;
		break;
	case AD74413R_ADC_RANGE_5V_BI_DIR:
		*val = -AD74413R_ADC_RESULT_MAX / 2;
		break;
	default:
		dev_err(st->dev, "Channel %u ADC range invalid\n", channel);
		return -EINVAL;
	}

	*val2 = 0;

	return IIO_VAL_INT;
}

static int ad74413r_get_input_current_scale(struct ad74413r_state *st, unsigned int channel,
					       int *val, int *val2)
{
	int ret;

	ret = ad74413r_get_adc_voltage_range(st, channel, val);
	if (ret)
		return ret;

	*val2 = AD74413R_ADC_RESULT_MAX * st->rsense_resistance_ohms;

	return IIO_VAL_FRACTIONAL;
}

static int ad74413_get_input_current_offset(struct ad74413r_state *st, unsigned int channel,
					    int *val, int *val2)
{
	int range;
	int offset;
	int ret;

	ret = ad74413r_get_adc_voltage_range(st, channel, &range);
	if (ret)
		return ret;

	ret = ad74413r_get_adc_voltage_offset(st, channel, &offset);
	if (ret)
		return ret;

	*val = offset * AD74413R_ADC_RESULT_MAX / range;
	*val2 = 0;

	return IIO_VAL_INT;
}

static int ad74413r_get_output_voltage_scale(struct ad74413r_state *st,
					     int *val, int *val2)
{
	*val = 11000;
	*val2 = AD74413R_DAC_CODE_MAX;

	return IIO_VAL_FRACTIONAL;
}

static int ad74413r_get_output_current_scale(struct ad74413r_state *st, int *val, int *val2)
{
	*val = regulator_get_voltage(st->refin_reg);
	*val2 = st->rsense_resistance_ohms * AD74413R_DAC_CODE_MAX * 1000;

	return IIO_VAL_FRACTIONAL;
}

static irqreturn_t ad74413r_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct ad74413r_state *st = iio_priv(indio_dev);
	unsigned int i;
	int ret;

	mutex_lock(&st->lock);

	ret = spi_sync(st->spi, &st->adc_samples_msg);
	if (ret)
		goto out;

	for (i = 0; i < st->adc_active_channels; i++)
		ad74413r_crc_check(st, (u8 *)&st->adc_samples.rx_buf[i]);

	iio_push_to_buffers_with_timestamp(indio_dev, &st->adc_samples,
					   iio_get_time_ns(indio_dev));

out:
	iio_trigger_notify_done(indio_dev->trig);

	mutex_unlock(&st->lock);

	return IRQ_HANDLED;
}

static irqreturn_t ad74413r_adc_data_interrupt(int irq, void *data)
{
	struct ad74413r_state *st = data;

	if (iio_buffer_enabled(st->indio_dev))
		iio_trigger_poll(st->trig);
	else
		complete(&st->adc_data_completion);

	return IRQ_HANDLED;
}

static int ad74413r_get_single_adc_result(struct ad74413r_state *st, unsigned int channel,
					  int *val)
{
	unsigned int uval;
	int ret;

	ret = iio_device_claim_direct_mode(st->indio_dev);
	if (ret)
		return ret;

	reinit_completion(&st->adc_data_completion);

	ret = ad74413r_set_adc_channel_enable(st, channel, true);
	if (ret)
		goto out;

	ret = ad74413r_set_adc_conv_seq(st, AD74413R_CONV_SEQ_SINGLE);
	if (ret)
		goto out;

	ret = wait_for_completion_timeout(&st->adc_data_completion,
					  msecs_to_jiffies(AD74413R_ADC_DATA_TIMEOUT));
	if (!ret) {
		ret = -ETIMEDOUT;
		goto out;
	}

	ret = regmap_read(st->regmap, AD74413R_REG_ADC_RESULT_X(channel), &uval);
	if (ret)
		goto out;

	ret = ad74413r_set_adc_conv_seq(st, AD74413R_CONV_SEQ_OFF);
	if (ret)
		goto out;

	ret = ad74413r_set_adc_channel_enable(st, channel, false);
	if (ret)
		goto out;

	*val = uval;
	ret = IIO_VAL_INT;

out:
	iio_device_release_direct_mode(st->indio_dev);

	return ret;
}

static int ad74413r_get_single_resistance_result(struct ad74413r_state *st, unsigned int channel,
						 int *val)
{
	unsigned int uval;
	int ret;

	ret = ad74413r_get_single_adc_result(st, channel, &uval);
	if (ret < 0)
		return ret;

	if (uval == AD74413R_ADC_RESULT_MAX)
		*val = 0;
	else
		*val = DIV_ROUND_CLOSEST(uval * 2100, AD74413R_ADC_RESULT_MAX - uval);

	return ret;
}

static int ad74413r_update_scan_mode(struct iio_dev *indio_dev,
				     const unsigned long *active_scan_mask)
{
	struct ad74413r_state *st = iio_priv(indio_dev);
	struct spi_transfer *xfer;
	unsigned int channel;
	int transfer_index = 0;
	int ret;

	mutex_lock(&st->lock);

	spi_message_init(&st->adc_samples_msg);
	st->adc_active_channels = 0;

	for (channel = 0; channel < AD74413R_CHANNEL_MAX; channel++) {
		bool status = test_bit(channel, active_scan_mask);
		u8 *tx_buf;

		ret = ad74413r_set_adc_channel_enable(st, channel, status);
		if (ret)
			goto out;

		if (!status)
			continue;

		st->adc_active_channels++;

		xfer = &st->adc_samples_xfer[transfer_index];

		if (transfer_index == 0)
			xfer->rx_buf = NULL;
		else
			xfer->rx_buf = &st->adc_samples.rx_buf[transfer_index - 1];

		tx_buf = (u8 *)&st->adc_samples_tx_buf[transfer_index];

		xfer->tx_buf = tx_buf;
		xfer->len = 4;
		xfer->cs_change = 1;

		ad74413r_format_reg_write(AD74413R_REG_READ_SELECT,
					  AD74413R_REG_ADC_RESULT_X(channel),
					  tx_buf);

		spi_message_add_tail(xfer, &st->adc_samples_msg);

		transfer_index++;
	}

	if (transfer_index == 0)
		goto out;

	xfer = &st->adc_samples_xfer[transfer_index];

	xfer->tx_buf = NULL;
	xfer->rx_buf = &st->adc_samples.rx_buf[transfer_index - 1];
	xfer->len = 4;
	xfer->cs_change = 0;

	spi_message_add_tail(xfer, &st->adc_samples_msg);

out:
	mutex_unlock(&st->lock);

	return ret;
}

static int ad74413r_buffer_postenable(struct iio_dev *indio_dev)
{
	struct ad74413r_state *st = iio_priv(indio_dev);

	return ad74413r_set_adc_conv_seq(st, AD74413R_CONV_SEQ_CONTINUOUS);
}

static int ad74413r_buffer_predisable(struct iio_dev *indio_dev)
{
	struct ad74413r_state *st = iio_priv(indio_dev);

	return ad74413r_set_adc_conv_seq(st, AD74413R_CONV_SEQ_OFF);
}

static int ad74413r_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			     int *val, int *val2, long info)
{
	struct ad74413r_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);

	switch (info) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ALTVOLTAGE:
			if (chan->output)
				ret = ad74413r_get_output_voltage_scale(st, val, val2);
			else
				ret = ad74413r_get_input_voltage_scale(st, chan->channel,
								       val, val2);
			break;

		case IIO_CURRENT:
			if (chan->output)
				ret = ad74413r_get_output_current_scale(st, val, val2);
			else
				ret = ad74413r_get_input_current_scale(st, chan->channel,
									  val, val2);
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_ALTVOLTAGE:
			if (!chan->output)
				ret = ad74413r_get_input_voltage_offset(st, chan->channel,
									val, val2);
			break;
		case IIO_CURRENT:
			if (!chan->output)
				ret = ad74413_get_input_current_offset(st, chan->channel,
								       val, val2);
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_RAW:
		if (!chan->output)
			ret = ad74413r_get_single_adc_result(st, chan->channel, val);
		break;
	case IIO_CHAN_INFO_PROCESSED:
		switch (chan->type) {
		case IIO_RESISTANCE:
			if (!chan->output)
				ret = ad74413r_get_single_resistance_result(st, chan->channel,
									    val);
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!chan->output)
			ret = ad74413r_get_adc_rate(st, chan->channel, val);
		break;
	default:
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static int ad74413r_write_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			      int val, int val2, long info)
{
	struct ad74413r_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	mutex_lock(&st->lock);

	switch (info) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_ALTVOLTAGE:
		case IIO_CURRENT:
			ret = ad74413r_set_channel_dac_code(st, chan->channel, val);
			break;
		default:
			break;
		}
		break;
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (!chan->output)
			ret = ad74413r_set_adc_rate(st, chan->channel, val);
		break;
	default:
		break;
	}

	mutex_unlock(&st->lock);

	return ret;
}

static int ad74413r_read_avail(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			      const int **vals, int *type, int *length, long info)
{
	struct ad74413r_state *st = iio_priv(indio_dev);
	int ret = -EINVAL;

	switch (info) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		if (st->config->hart_support) {
			*vals = ad74413r_adc_sampling_rates_hart;
			*length = ARRAY_SIZE(ad74413r_adc_sampling_rates_hart);
		} else {
			*vals = ad74413r_adc_sampling_rates;
			*length = ARRAY_SIZE(ad74413r_adc_sampling_rates);
		}
		*type = IIO_VAL_INT;
		ret = IIO_AVAIL_LIST;
		break;
	default:
		break;
	}

	return ret;
}

static const struct iio_buffer_setup_ops ad74413r_buffer_ops = {
	.postenable = &ad74413r_buffer_postenable,
	.predisable = &ad74413r_buffer_predisable,
};

static const struct iio_trigger_ops ad74413r_trigger_ops = {
	.validate_device = iio_trigger_validate_own_device,
};

static const struct iio_info ad74413r_info = {
	.read_raw = &ad74413r_read_raw,
	.write_raw = &ad74413r_write_raw,
	.read_avail = &ad74413r_read_avail,
	.update_scan_mode = &ad74413r_update_scan_mode,
};

#define AD74413R_CHANNEL(_type, _output, extra_mask_separate)		\
		.type = _type,						\
		.indexed = 1,						\
		.output = _output,					\
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) | extra_mask_separate

#define AD74413R_DAC_CHANNEL(_type, extra_mask_separate)		\
	{								\
		AD74413R_CHANNEL(_type, 1, extra_mask_separate),	\
	}

#define AD74413R_ADC_CHANNEL(_type, extra_mask_separate)					\
	{											\
		AD74413R_CHANNEL(_type, 0, extra_mask_separate | BIT(IIO_CHAN_INFO_SAMP_FREQ)),	\
		.info_mask_shared_by_type_available = BIT(IIO_CHAN_INFO_SAMP_FREQ),		\
		.scan_type = {									\
			.sign = 'u',								\
			.realbits = 16,								\
			.storagebits = 32,							\
			.shift = 8,								\
			.endianness = IIO_BE,							\
		},										\
	}

#define AD74413R_ADC_VOLTAGE_CHANNEL					\
	AD74413R_ADC_CHANNEL(IIO_ALTVOLTAGE, BIT(IIO_CHAN_INFO_SCALE)	\
			     | BIT(IIO_CHAN_INFO_OFFSET))

#define AD74413R_ADC_CURRENT_CHANNEL					\
	AD74413R_ADC_CHANNEL(IIO_CURRENT,  BIT(IIO_CHAN_INFO_SCALE)	\
			     | BIT(IIO_CHAN_INFO_OFFSET))

static struct iio_chan_spec ad74413r_high_impedance_channels[] = {
	AD74413R_ADC_VOLTAGE_CHANNEL,
};

static struct iio_chan_spec ad74413r_voltage_output_channels[] = {
	AD74413R_DAC_CHANNEL(IIO_ALTVOLTAGE, BIT(IIO_CHAN_INFO_SCALE)),
	AD74413R_ADC_CURRENT_CHANNEL,
};

static struct iio_chan_spec ad74413r_current_output_channels[] = {
	AD74413R_DAC_CHANNEL(IIO_CURRENT, BIT(IIO_CHAN_INFO_SCALE)),
	AD74413R_ADC_VOLTAGE_CHANNEL,
};

static struct iio_chan_spec ad74413r_voltage_input_channels[] = {
	AD74413R_ADC_VOLTAGE_CHANNEL,
};

static struct iio_chan_spec ad74413r_current_input_channels[] = {
	AD74413R_ADC_CURRENT_CHANNEL,
};

static struct iio_chan_spec ad74413r_resistance_input_channels[] = {
	AD74413R_ADC_CHANNEL(IIO_RESISTANCE, BIT(IIO_CHAN_INFO_PROCESSED)),
};

static struct iio_chan_spec ad74413r_digital_input_channels[] = {
	AD74413R_ADC_VOLTAGE_CHANNEL,
};

static int ad74413r_of_parse_channel_config(struct ad74413r_state *st,
					    struct device_node *channel_node)
{
	struct ad74413r_channel_config *channel_config;
	u32 index;
	int ret;

	ret = of_property_read_u32(channel_node, "reg", &index);
	if (ret) {
		dev_err(st->dev, "Failed to read channel reg: %d\n", ret);
		return ret;
	}

	if (index > AD74413R_CHANNEL_MAX) {
		dev_err(st->dev, "Channel index %u is too large\n", index);
		return -EINVAL;
	}

	channel_config = &st->channel_configs[index];
	if (channel_config->initialized) {
		dev_err(st->dev, "Channel %u has already been initialized\n", index);
		return -EINVAL;
	}

	channel_config->func = CH_FUNC_HIGH_IMPEDANCE;
	of_property_read_u32(channel_node, "adi,ch-func", &channel_config->func);

	channel_config->gpo_config = GPO_CONFIG_100K_PULL_DOWN;
	of_property_read_u32(channel_node, "adi,gpo-config", &channel_config->gpo_config);

	if (channel_config->gpo_config == GPO_CONFIG_LOGIC_PARALLEL) {
		dev_err(st->dev, "GPO config logic parallel is unsupported\n");
		return -EINVAL;
	}

	ret = ad74413r_channel_set_function(st, index, channel_config->func);
	if (ret)
		return ret;

	ret = ad74413r_set_gpo_mode(st, index, channel_config->gpo_config);
	if (ret)
		return ret;

	channel_config->initialized = true;

	return 0;
}

static int ad74413r_parse_channel_configs(struct ad74413r_state *st)
{
	struct device_node *channel_node = NULL;
	int ret;

	for_each_available_child_of_node(st->dev->of_node, channel_node) {
		if (!of_node_name_eq(channel_node, "channel"))
			continue;

		ret = ad74413r_of_parse_channel_config(st, channel_node);
		if (ret) {
			dev_err(st->dev, "Failed to parse channel %s config: %d\n",
				channel_node->name, ret);
			goto put_channel_node;
		}
	}

put_channel_node:
	of_node_put(channel_node);

	return ret;
}

static int ad74413r_get_iio_channels(struct ad74413r_state *st, u8 func,
				     struct iio_chan_spec **iio_channels, unsigned int *count)
{
	switch (func) {
	case CH_FUNC_HIGH_IMPEDANCE:
		*iio_channels = ad74413r_high_impedance_channels;
		*count = ARRAY_SIZE(ad74413r_high_impedance_channels);
		break;
	case CH_FUNC_VOLTAGE_OUTPUT:
		*iio_channels = ad74413r_voltage_output_channels;
		*count = ARRAY_SIZE(ad74413r_voltage_output_channels);
		break;
	case CH_FUNC_CURRENT_OUTPUT:
		*iio_channels = ad74413r_current_output_channels;
		*count = ARRAY_SIZE(ad74413r_current_output_channels);
		break;
	case CH_FUNC_VOLTAGE_INPUT:
		*iio_channels = ad74413r_voltage_input_channels;
		*count = ARRAY_SIZE(ad74413r_voltage_input_channels);
		break;
	case CH_FUNC_CURRENT_INPUT_EXT_POWER:
	case CH_FUNC_CURRENT_INPUT_LOOP_POWER:
	case CH_FUNC_CURRENT_INPUT_EXT_POWER_HART:
	case CH_FUNC_CURRENT_INPUT_LOOP_POWER_HART:
		*iio_channels = ad74413r_current_input_channels;
		*count = ARRAY_SIZE(ad74413r_current_input_channels);
		break;
	case CH_FUNC_RESISTANCE_INPUT:
		*iio_channels = ad74413r_resistance_input_channels;
		*count = ARRAY_SIZE(ad74413r_resistance_input_channels);
		break;
	case CH_FUNC_DIGITAL_INPUT_LOGIC:
	case CH_FUNC_DIGITAL_INPUT_LOOP_POWER:
		*iio_channels = ad74413r_digital_input_channels;
		*count = ARRAY_SIZE(ad74413r_digital_input_channels);
		break;
	default:
		dev_err(st->dev, "Unhandled channel config function %d\n", func);
		return -EINVAL;
	}

	return 0;
}

static int ad74413r_count_iio_channels(struct ad74413r_state *st)
{
	unsigned int index;
	int ret;

	st->indio_dev->num_channels = 0;

	for (index = 0; index < AD74413R_CHANNEL_MAX; index++) {
		struct ad74413r_channel_config *channel_config = &st->channel_configs[index];
		struct iio_chan_spec *local_channels;
		unsigned int num_local_channels;

		ret = ad74413r_get_iio_channels(st, channel_config->func, &local_channels,
						&num_local_channels);
		if (ret)
			return ret;

		st->indio_dev->num_channels += num_local_channels;
	}

	return 0;
}

static int ad74413r_setup_iio_channels(struct ad74413r_state *st)
{
	struct iio_chan_spec *channels;
	unsigned int index;
	int ret;

	channels = devm_kcalloc(st->dev, sizeof(*channels),
				st->indio_dev->num_channels, GFP_KERNEL);
	if (!channels)
		return -ENOMEM;

	st->indio_dev->channels = channels;

	for (index = 0; index < AD74413R_CHANNEL_MAX; index++) {
		struct ad74413r_channel_config *channel_config = &st->channel_configs[index];
		struct iio_chan_spec *local_channels;
		unsigned int num_local_channels;
		unsigned int chan_index;

		ret = ad74413r_get_iio_channels(st, channel_config->func, &local_channels,
						&num_local_channels);
		if (ret)
			return ret;

		memcpy(channels, local_channels, num_local_channels * sizeof(*local_channels));

		for (chan_index = 0; chan_index < num_local_channels; chan_index++) {
			struct iio_chan_spec *local_channel = &channels[chan_index];

			local_channel->channel = index;
			if (local_channel->output)
				local_channel->scan_index = -1;
			else
				local_channel->scan_index = index;

		}

		channels += num_local_channels;
	}

	return 0;
}

static void ad74413r_regulator_disable(void *regulator)
{
	regulator_disable(regulator);
}

static int ad74413r_probe(struct spi_device *spi)
{
	struct ad74413r_state *st;
	struct iio_dev *indio_dev;
	unsigned int sillicon_rev_id;
	const char *name;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (indio_dev == NULL)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	spi_set_drvdata(spi, st);

	st->spi = spi;
	st->dev = &spi->dev;
	st->config = of_device_get_match_data(&spi->dev);
	st->indio_dev = indio_dev;
	mutex_init(&st->lock);
	init_completion(&st->adc_data_completion);

	name = dev_name(st->dev);

	st->regmap = devm_regmap_init(st->dev, NULL, &spi->dev, &ad74413r_regmap_config);
	if (IS_ERR(st->regmap)) {
		ret = PTR_ERR(st->regmap);
		dev_err(st->dev, "Failed to create regmap: %d\n", ret);
		return ret;
	}

	ret = regmap_read(st->regmap, AD74413R_REG_SILLICON_REV, &sillicon_rev_id);
	if (ret) {
		dev_err(st->dev, "Failed to read sillicon rev: %d\n", ret);
		return ret;
	}

	sillicon_rev_id = FIELD_GET(AD74413R_SILLICON_REV_ID_MASK, sillicon_rev_id);
	if (sillicon_rev_id != AD74413R_SILLICON_REV_ID_EXPECTED) {
		dev_err(st->dev, "Read sillicon rev id %d does not match expected rev id %d\n",
			sillicon_rev_id, AD74413R_SILLICON_REV_ID_EXPECTED);
		return -EINVAL;
	}

	st->refin_reg = devm_regulator_get(st->dev, "refin");
	if (IS_ERR(st->refin_reg)) {
		dev_err(st->dev, "Failed to get refin regulator: %d\n", ret);
		return PTR_ERR(st->refin_reg);
	}

	ret = regulator_enable(st->refin_reg);
	if (ret) {
		dev_err(st->dev, "Failed to enable refin regulator: %d\n", ret);
		return ret;
	}

	ret = devm_add_action_or_reset(st->dev, ad74413r_regulator_disable,
				       st->refin_reg);
	if (ret) {
		dev_err(st->dev, "Failed to add refin regulator disable action: %d\n", ret);
		return ret;
	}

	ret = of_property_read_u32(st->dev->of_node, "adi,rsense-resistance-ohms",
				   &st->rsense_resistance_ohms);
	if (ret) {
		dev_err(st->dev, "Failed to get rsense resistance: %d\n", ret);
		return ret;
	}

	st->gpiochip.label = name;
	st->gpiochip.base = -1;
	st->gpiochip.ngpio = AD74413R_CHANNEL_MAX;
	st->gpiochip.parent = st->dev;
	st->gpiochip.can_sleep = true;
	st->gpiochip.set = ad74413r_gpio_set;
	st->gpiochip.set_multiple = ad74413r_gpio_set_multiple;
	st->gpiochip.owner = THIS_MODULE;
	st->gpiochip.get = ad74413r_gpio_get;

	ret = devm_gpiochip_add_data(st->dev, &st->gpiochip, st);
	if (ret) {
		dev_err(st->dev, "Failed to add gpio chip: %d\n", ret);
		return ret;
	}

	st->trig = devm_iio_trigger_alloc(st->dev, "%s-dev%d", name, indio_dev->id);
	if (!st->trig)
		return -ENOMEM;

	st->trig->ops = &ad74413r_trigger_ops;
	st->trig->dev.parent = st->dev;
	iio_trigger_set_drvdata(st->trig, st);

	ret = devm_iio_trigger_register(st->dev, st->trig);
	if (ret)
		return ret;

	indio_dev->dev.parent = st->dev;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	indio_dev->info = &ad74413r_info;
	indio_dev->trig = iio_trigger_get(st->trig);

	ret = ad74413r_parse_channel_configs(st);
	if (ret)
		return ret;

	ret = ad74413r_count_iio_channels(st);
	if (ret)
		return ret;

	ret = ad74413r_setup_iio_channels(st);
	if (ret) {
		dev_err(st->dev, "Failed to setup iio channels: %d\n", ret);
		return ret;
	}

	ret = ad74413r_set_adc_conv_seq(st, AD74413R_CONV_SEQ_OFF);
	if (ret)
		return ret;

	ret = devm_request_irq(st->dev, spi->irq, ad74413r_adc_data_interrupt,
			       IRQF_TRIGGER_FALLING, name, st);
	if (ret) {
		dev_err(st->dev, "Failed to request threaded irq: %d\n", ret);
		return ret;
	}

	ret = devm_iio_triggered_buffer_setup(st->dev, indio_dev, &iio_pollfunc_store_time,
					      &ad74413r_trigger_handler, &ad74413r_buffer_ops);
	if (ret) {
		dev_err(st->dev, "Failed to setup triggered buffer: %d\n", ret);
		return ret;
	}

	return devm_iio_device_register(st->dev, indio_dev);
}

static int ad74413r_unregister_driver(struct spi_driver *spi)
{
	spi_unregister_driver(spi);

	return 0;
}

static int __init ad74413r_register_driver(struct spi_driver *spi)
{
	crc8_populate_msb(ad74413r_crc8_table, AD74413R_CRC_POLYNOMIAL);

	return spi_register_driver(spi);
}

static const struct ad74413r_config ad74412r_config_data = {
	.hart_support = false,
};

static const struct ad74413r_config ad74413r_config_data = {
	.hart_support = true,
};

static const struct of_device_id ad74413r_dt_id[] = {
	{
		.compatible = "adi,ad74412r",
		.data = &ad74412r_config_data,
	},
	{
		.compatible = "adi,ad74413r",
		.data = &ad74413r_config_data,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ad74413r_dt_id);

static struct spi_driver ad74413r_driver = {
	.driver = {
		   .name = "ad74413r",
		   .of_match_table = ad74413r_dt_id,
	},
	.probe = ad74413r_probe,
};

module_driver(ad74413r_driver, ad74413r_register_driver, ad74413r_unregister_driver);

MODULE_AUTHOR("Cosmin Tanislav <cosmin.tanislav@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD74413R ADDAC");
MODULE_LICENSE("GPL v2");
