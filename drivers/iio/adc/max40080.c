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

struct max40080_state {
	struct i2c_client *client;
	struct regmap *regmap;
};

static const struct regmap_config max40080_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int max40080_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val,
			     int *val2,
			     long mask)
{
	struct max40080_state *st = iio_priv(indio_dev);

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_FRACTIONAL_LOG2;
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

	if (read_val)
		return regmap_bulk_read(st->regmap, reg, read_val, 2);

	return regmap_bulk_write(st->regmap, reg, &write_val, 2);
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
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_VOLTAGE,
		.indexed = 1,
		.channel = 1,
		.output = 0,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
};

static int max40080_init(struct max40080_state *st)
{
	u16 tmp = 0;
	int ret;

	tmp = FIELD_PREP(MAX40080_MODE_MSK, MAX40080_ACTIVE_MODE) |
	      FIELD_PREP(MAX40080_I2C_TO_MSK, 0) |
	      FIELD_PREP(MAX40080_ALERT_MSK, 0) |
	      FIELD_PREP(MAX40080_PEC_EN_MSK, 0) |
	      FIELD_PREP(MAX40080_RANGE_MSK, 0) |
	      FIELD_PREP(MAX40080_HS_MSK, 0) |
	      FIELD_PREP(MAX40080_ADC_RATE_MSK, MAX40080_SR_15_KSPS) |
	      FIELD_PREP(MAX40080_FILTER_MSK, MAX40080_FTR_NO_AVG);

	dev_info(&st->client->dev, "reg 0x%X val 0x%X \n", MAX40080_REG_CFG, tmp);

	ret = regmap_bulk_write(st->regmap, MAX40080_REG_CFG, &tmp, 2);
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
	struct regmap *regmap;
	int ret;

	dev_info(dev, "Probing\n");

	indio_dev = devm_iio_device_alloc(dev, sizeof(struct iio_dev));
	if (!indio_dev)
		return -ENOMEM;

	i2c_set_clientdata(client, indio_dev);
	
	regmap = devm_regmap_init_i2c(client, &max40080_regmap_config);
	if (IS_ERR(regmap))
	    return PTR_ERR(regmap);
	
	st = iio_priv(indio_dev);
	st->client = client;
	st->regmap = regmap;

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