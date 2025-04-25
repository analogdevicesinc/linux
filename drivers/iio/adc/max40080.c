// SPDX-License-Identifier: GPL-2.0+
/*
 * MAX40080 Digital Current-Sense Amplifier driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/units.h>

#include <linux/iio/iio.h>

#define MAX40080_REG_CFG		0x00
#define  MAX40080_MODE_MSK		GENMASK(2, 0)
#define  MAX40080_I2C_TO_MSK		BIT(3)
#define  MAX40080_ALERT_MSK		BIT(4)
#define  MAX40080_PEC_EN_MSK		BIT(5)
#define  MAX40080_RANGE_MSK		BIT(6)
#define  MAX40080_HS_MSK		BIT(7)
#define  MAX40080_ADC_RATE_MSK		GENMASK(11, 8)
#define  MAX40080_FILTER_MSK		GENMASK(14, 12)

#define MAX40080_REG_STATUS		0x02
#define  MAX40080_WAKE_UP_MSK		BIT(0)
#define  MAX40080_CNV_RDY_MSK		BIT(1)
#define  MAX40080_I_OVERFLOW_MSK	BIT(2)
#define  MAX40080_V_OVERFLOW_MSK	BIT(3)
#define  MAX40080_V_UNDERFLOW_MSK	BIT(4)
#define  MAX40080_I2C_TO_STATUS_MSK	BIT(5)
#define  MAX40080_FIFO_ALARM_MSK	BIT(6)
#define  MAX40080_FIFO_OVERFLOW_MSK	BIT(7)
#define  MAX40080_FIFO_CNT_MSK		GENMASK(13, 8)

#define MAX40080_REG_TH_OVER_I		0x04

#define MAX40080_REG_TH_OVER_V		0x05

#define MAX40080_REG_TH_UNDER_V		0x06

#define MAX40080_REG_WAKE_UP_I		0x07

#define MAX40080_REG_MAX_PEAK_I		0x08
#define  MAX40080_STORE_IV_MSK		GENMASK(1, 0)
#define  MAX40080_OVERFLOW_WARN_MSK	GENMASK(13, 8)
#define  MAX40080_ROLL_OVER_MSK		BIT(14)
#define  MAX40080_FLUSH_MSK		BIT(15)

#define MAX40080_REG_FIFO_CFG		0x0A

#define MAX40080_REG_I			0x0C
#define  MAX40080_I_MAG_MSK		GENMASK(11, 0)
#define  MAX40080_I_SIGN_MSK		BIT(12)
#define  MAX40080_I_SIGN_ETX_MSK	GENMASK(14, 13)
#define  MAX40080_I_VALID_MSK		BIT(15)

#define MAX40080_REG_V			0x0E
#define  MAX40080_V_MAG_MSK		GENMASK(11, 0)
#define  MAX40080_V_SIGN_MSK		BIT(12)
#define  MAX40080_V_SIGN_ETX_MSK	GENMASK(14, 13)
#define  MAX40080_V_VALID_MSK		BIT(15)

#define MAX40080_REG_IV			0x10
#define  MAX40080_IV_I_MAG_MSK		GENMASK(11, 0)
#define  MAX40080_IV_I_SIGN_MSK		GENMASK(14, 12)
#define  MAX40080_IV_V_MAG_MSK		GENMASK(27, 16)
#define  MAX40080_IV_V_SIGN_MSK		GENMASK(30, 28)
#define  MAX40080_IV_VALID_MSK		BIT(31)

#define MAX40080_REG_INT_EN		0x14
#define  MAX40080_WAKE_UP_EN_MSK	BIT(0)
#define  MAX40080_CNV_RDY_EN_MSK	BIT(1)
#define  MAX40080_I_OVERFLOW_EN_MSK	BIT(2)
#define  MAX40080_V_OVERFLOW_EN_MSK	BIT(3)
#define  MAX40080_V_UNDERFLOW_EN_MSK	BIT(4)
#define  MAX40080_I2C_TO_EN_MSK		BIT(5)
#define  MAX40080_FIFO_ALARM_EN_MSK	BIT(6)
#define  MAX40080_FIFO_OVERFLOW_EN_MSK	BIT(7)

#define	MAX40080_STDBY_MODE		0x00
#define	MAX40080_LP_MODE		0x01
#define	MAX40080_SGL_CNV_MODE		0x02
#define	MAX40080_ACTIVE_MODE		0x03
#define	MAX40080_4SPS_MODE		0x04
#define	MAX40080_1SPS_MODE		0x05
#define	MAX40080_0_25SPS_MODE		0x06
#define	MAX40080_0_0625SPS_MODE		0x07

#define MAX40080_SR_15_KSPS		0x00
#define MAX40080_SR_23_45_KSPS		0x02
#define MAX40080_SR_30_KSPS		0x03
#define MAX40080_SR_37_5_KSPS		0x04
#define MAX40080_SR_47_1_KSPS		0x05
#define MAX40080_SR_60_KSPS		0x06
#define MAX40080_SR_93_5_KSPS		0x07
#define MAX40080_SR_120_KSPS		0x08
#define MAX40080_SR_150_KSPS		0x09
#define MAX40080_SR_234_5_KSPS		0x0A
#define MAX40080_SR_375_KSPS		0x0B
#define MAX40080_SR_468_5_KSPS		0x0C
#define MAX40080_SR_750_KSPS		0x0D
#define MAX40080_SR_1000_KSPS		0x0E
#define MAX40080_SR_0_5_KSPS		0x0F

#define MAX40080_FTR_NO_AVG		0x00
#define MAX40080_FTR_8_AVG		0x01
#define MAX40080_FTR_16_AVG		0x02
#define MAX40080_FTR_32_AVG		0x03
#define MAX40080_FTR_64_AVG		0x04
#define MAX40080_FTR_128_AVG		0x05	

#define MAX40080_STORE_I_ONLY		0x00
#define MAX40080_STORE_V_ONLY		0x01
#define MAX40080_STORE_I_V		0x02

#define MAX40080_ADC_RES		4095
#define MAX40080_INTER_VREF_MV		1250
#define MAX40080_V_BUFF_GAIN		30
#define MAX40080_CSA_50MV_GAIN		25
#define MAX40080_CSA_10MV_GAIN		125

struct max40080_state {
	struct i2c_client *client;
};

static int max40080_get_current(struct max40080_state *st, int *val)
{
	u16 _current;
	u8 valid;
	int tmp;

	tmp = i2c_smbus_read_word_data(st->client, MAX40080_REG_I);
	if (tmp < 0)
		return tmp;

	dev_info(&st->client->dev, "current reg val = 0x%X \n", tmp);

	valid = FIELD_GET(MAX40080_I_VALID_MSK, tmp);
	if (!valid) {
		dev_err(&st->client->dev, "Invalid current data\n");
		return -EINVAL;
	}

	_current = FIELD_GET(MAX40080_I_MAG_MSK, tmp);
	if (FIELD_GET(MAX40080_I_SIGN_MSK, tmp))
		*val = _current * -1;
	else
		*val = _current;
	
	return 0;
}

static int max40080_get_voltage(struct max40080_state *st, int *val)
{
	u8 valid;
	int tmp;

	tmp = i2c_smbus_read_word_data(st->client, MAX40080_REG_V);
	if (tmp < 0)
		return tmp;

	dev_info(&st->client->dev, "voltage reg val = 0x%X \n", tmp);

	valid = FIELD_GET(MAX40080_V_VALID_MSK, tmp);
	if (!valid) {
		dev_err(&st->client->dev, "Invalid voltage data\n");
		return -EINVAL;
	}

	*val = FIELD_GET(MAX40080_V_MAG_MSK, tmp);

	return 0;
}

static int max40080_get_csa_gain(struct max40080_state *st, u8 *val)
{
	u8 range;
	u16 tmp;

	tmp = i2c_smbus_read_word_data(st->client, MAX40080_REG_CFG);
	if (tmp < 0)
		return tmp;

	range = FIELD_GET(MAX40080_RANGE_MSK, tmp);
	if (range)
		*val = MAX40080_CSA_10MV_GAIN;
	else
		*val = MAX40080_CSA_50MV_GAIN;

	return 0;
}

static int max40080_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long mask)
{
	struct max40080_state *st = iio_priv(indio_dev);
	u8 gain;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (chan->type == IIO_CURRENT) {
			ret = max40080_get_current(st, val);
			if (ret)
				return ret;
		} else if (chan->type == IIO_VOLTAGE) {
			ret = max40080_get_voltage(st, val);
			if (ret)
				return ret;
		}

		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_CURRENT) {
			ret = max40080_get_csa_gain(st, &gain);
			if (ret)
				return ret;
			
			*val = MAX40080_INTER_VREF_MV * gain;
			*val2 = MAX40080_ADC_RES * MILLI;
		}
		else if (chan->type == IIO_VOLTAGE) {
			*val = MAX40080_INTER_VREF_MV * MAX40080_V_BUFF_GAIN;
			*val2 = MAX40080_ADC_RES * MILLI;
		}
		return IIO_VAL_FRACTIONAL;
	default:
		return -EINVAL;
	}
}

static int max40080_reg_access(struct iio_dev *indio_dev,
			       unsigned int reg,
			       unsigned int write_val,
			       unsigned int *read_val)
{
	struct max40080_state *st = iio_priv(indio_dev);

	if (read_val) {
		*read_val = i2c_smbus_read_word_data(st->client, reg);
		if (*read_val < 0)
			return *read_val;
		return 0;
	}

	return i2c_smbus_write_word_data(st->client, reg, write_val);
}

static const struct iio_info max40080_info = {
	.read_raw = max40080_read_raw,
	.debugfs_reg_access = &max40080_reg_access,
};

static const struct iio_chan_spec max40080_channels[] = {
	{
		.type = IIO_CURRENT,
		.indexed = 1,
		.channel = 0,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 1,
		.output = 1,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int max40080_init(struct max40080_state *st)
{
	u16 tmp = 0;
	int ret;

	tmp = FIELD_PREP(MAX40080_MODE_MSK, MAX40080_ACTIVE_MODE) |
	      FIELD_PREP(MAX40080_I2C_TO_MSK, 1) |
	      FIELD_PREP(MAX40080_ALERT_MSK, 0) |
	      FIELD_PREP(MAX40080_PEC_EN_MSK, 1) |
	      FIELD_PREP(MAX40080_RANGE_MSK, 0) |
	      FIELD_PREP(MAX40080_HS_MSK, 0) |
	      FIELD_PREP(MAX40080_ADC_RATE_MSK, MAX40080_SR_15_KSPS) |
	      FIELD_PREP(MAX40080_FILTER_MSK, MAX40080_FTR_NO_AVG);

	ret = i2c_smbus_write_word_data(st->client, MAX40080_REG_CFG, tmp);
	if (ret)
		return ret;

	tmp = FIELD_PREP(MAX40080_STORE_IV_MSK, MAX40080_STORE_I_V) |
	      FIELD_PREP(MAX40080_OVERFLOW_WARN_MSK, 0X34) |
	      FIELD_PREP(MAX40080_ROLL_OVER_MSK, 1);

	ret = i2c_smbus_write_word_data(st->client, MAX40080_REG_FIFO_CFG, tmp);
	if (ret)
		return ret;

	return 0;
}

static int max40080_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct iio_dev *indio_dev;
	struct max40080_state *st;
	int ret;

	dev_info(dev, "Probing\n");
	client->flags |= I2C_CLIENT_PEC;

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct iio_dev));
	if (!indio_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, indio_dev);
	
	st = iio_priv(indio_dev);
	st->client = client;

	indio_dev->name = "max40080";
	indio_dev->info = &max40080_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = max40080_channels;
	indio_dev->num_channels = ARRAY_SIZE(max40080_channels);

	ret = max40080_init(st);
	if (ret)
		return ret;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return ret;

	return 0;
}


static const struct i2c_device_id max40080_i2c_ids[] = {
	{"adi,max40080", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, max40080_i2c_ids);

static const struct of_device_id max40080_device_match[] = {
	{ .compatible = "adi,max40080" },
	{}
};
MODULE_DEVICE_TABLE(of, max40080_device_match);

static struct i2c_driver max40080_driver = {
	.driver = {
		.name = "max40080",
		.of_match_table = max40080_device_match,
	},
	.probe = max40080_probe,
};
module_i2c_driver(max40080_driver)

MODULE_AUTHOR("Ciprian Hegbeli <ciprian.hegbeli@analog.com>");
MODULE_DESCRIPTION("Analog Devices MAX40080 driver");
MODULE_LICENSE("GPL");