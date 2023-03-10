// SPDX-License-Identifier: GPL-2.0
/*
 * max31827.c - Support for Maxim Low-Power Switch
 *
 * Copyright (c) 2023 Daniel Matyas <daniel.matyas@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/regmap.h>

#include <linux/iio/events.h>
#include <linux/iio/iio.h>

#define MAX31827_T_REG				0x0
#define MAX31827_CONFIGURATION_REG              0x2
#define MAX31827_TH_REG                         0x4
#define MAX31827_TL_REG                         0x6
#define MAX31827_TH_HYST_REG                    0x8
#define MAX31827_TL_HYST_REG                    0xA

#define MAX31827_CONFIGURATION_1SHOT_MASK	BIT(0)
#define MAX31827_CONFIGURATION_CNV_RATE_MASK    GENMASK(3, 1)
#define MAX31827_CONFIGURATION_RESOL_MASK       GENMASK(7, 6)
#define MAX31827_CONFIGURATION_U_TEMP_STAT_MASK BIT(14)
#define MAX31827_CONFIGURATION_O_TEMP_STAT_MASK BIT(15)

#define MAX31827_CNV_SHUTDOWN			0x0
#define MAX31827_CNV_1_DIV_64_HZ		0x1
#define MAX31827_CNV_1_DIV_32_HZ		0x2
#define MAX31827_CNV_1_DIV_16_HZ		0x3
#define MAX31827_CNV_1_DIV_4_HZ			0x4
#define MAX31827_CNV_1_HZ			0x5
#define MAX31827_CNV_4_HZ			0x6
#define MAX31827_CNV_8_HZ			0x7

#define MAX31827_1SHOT_EN(x)			((x) ? BIT(0) : 0)

struct max31827_state {
	struct regmap *regmap;
	struct i2c_client *client;
};

static const struct regmap_config max31827_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0xA,
};

static irqreturn_t max31827_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct max31827_state *st = iio_priv(indio_dev);
	int u_temp_stat, o_temp_stat;
	int ret;
	int reg;

	ret = regmap_read(st->regmap, MAX31827_CONFIGURATION_REG, &reg);
	if (ret < 0)
		return IRQ_NONE;

	u_temp_stat = FIELD_GET(MAX31827_CONFIGURATION_U_TEMP_STAT_MASK, reg);
	o_temp_stat = FIELD_GET(MAX31827_CONFIGURATION_O_TEMP_STAT_MASK, reg);
	if (!(u_temp_stat | o_temp_stat))
		return IRQ_NONE;

	if (u_temp_stat)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
						  IIO_MOD_TEMP_OBJECT,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_FALLING),
			       iio_get_time_ns(indio_dev));

	if (o_temp_stat)
		iio_push_event(indio_dev,
			       IIO_MOD_EVENT_CODE(IIO_TEMP, 0,
						  IIO_MOD_TEMP_OBJECT,
						  IIO_EV_TYPE_THRESH,
						  IIO_EV_DIR_RISING),
			       iio_get_time_ns(indio_dev));

	return IRQ_HANDLED;
}

static int max31827_read_event_value(struct iio_dev *indio_dev,
				     const struct iio_chan_spec *chan,
				     enum iio_event_type type,
				     enum iio_event_direction dir,
				     enum iio_event_info info,
				     int *val, int *val2)
{
	struct max31827_state *st = iio_priv(indio_dev);
	int ret;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			ret = regmap_read(st->regmap, MAX31827_TL_REG, val);
			break;
		case IIO_EV_DIR_RISING:
			ret = regmap_read(st->regmap, MAX31827_TH_REG, val);
			break;
		default:
			return -EINVAL;
		}
		break;

	case IIO_EV_INFO_HYSTERESIS:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			ret = regmap_read(st->regmap, MAX31827_TL_HYST_REG, val);
			break;
		case IIO_EV_DIR_RISING:
			ret = regmap_read(st->regmap, MAX31827_TH_HYST_REG, val);
			break;
		default:
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;
	return IIO_VAL_INT;
}

static int max31827_write_event_value(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      enum iio_event_type type,
				      enum iio_event_direction dir,
				      enum iio_event_info info,
				      int val, int val2)
{
	struct max31827_state *st = iio_priv(indio_dev);
	int ret;

	ret = regmap_update_bits(st->regmap,
				 MAX31827_CONFIGURATION_REG,
				 MAX31827_CONFIGURATION_1SHOT_MASK |
				 MAX31827_CONFIGURATION_CNV_RATE_MASK, 0x0);
	if (ret < 0)
		return ret;

	switch (info) {
	case IIO_EV_INFO_VALUE:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			ret = regmap_write(st->regmap, MAX31827_TL_REG, val);
			break;
		case IIO_EV_DIR_RISING:
			ret = regmap_write(st->regmap, MAX31827_TH_REG, val);
			break;
		default:
			return -EINVAL;
		}
		break;

	case IIO_EV_INFO_HYSTERESIS:
		switch (dir) {
		case IIO_EV_DIR_FALLING:
			ret = regmap_write(st->regmap, MAX31827_TL_HYST_REG, val);
			break;
		case IIO_EV_DIR_RISING:
			ret = regmap_write(st->regmap, MAX31827_TH_HYST_REG, val);
			break;
		default:
			return -EINVAL;
		}
		break;

	default:
		return -EINVAL;
	}

	if (ret < 0)
		return ret;
	return 0;
}

static int max31827_read_raw(struct iio_dev *indio_dev,
			     struct iio_chan_spec const *chan,
			     int *val, int *val2, long mask)
{
	struct max31827_state *st = iio_priv(indio_dev);
	int ret;
	int cfg;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_read(st->regmap, MAX31827_CONFIGURATION_REG, val);
		if (ret < 0)
			return ret;

		*val = FIELD_GET(MAX31827_CONFIGURATION_1SHOT_MASK, *val);
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_RAW:
		ret = regmap_read(st->regmap, MAX31827_T_REG, val);
		if (ret < 0)
			return ret;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		*val = 1;
		*val2 = 16;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_SAMP_FREQ:
		ret = regmap_read(st->regmap, MAX31827_CONFIGURATION_REG, &cfg);
		if (ret < 0)
			return ret;
		
		cfg = FIELD_GET(MAX31827_CONFIGURATION_CNV_RATE_MASK, cfg);
		switch (cfg) {
		case MAX31827_CNV_SHUTDOWN:
			*val = 0;
			break;
		case MAX31827_CNV_1_DIV_64_HZ:
			*val = 1;
			*val2 = 64;
			break;
		case MAX31827_CNV_1_DIV_32_HZ:
			*val = 1;
			*val2 = 32;
			break;
		case MAX31827_CNV_1_DIV_16_HZ:
			*val = 1;
			*val2 = 16;
			break;
		case MAX31827_CNV_1_DIV_4_HZ:
			*val = 1;
			*val2 = 4;
			break;
		case MAX31827_CNV_1_HZ:
			*val = 1;
			*val2 = 1;
			break;
		case MAX31827_CNV_4_HZ:
			*val = 4;
			*val2 = 1;
			break;
		case MAX31827_CNV_8_HZ:
			*val = 8;
			*val2 = 1;
			break;
		default:
			return -EINVAL;
		}

		return IIO_VAL_FRACTIONAL;
	}

	return -EINVAL;
}

static int max31827_write_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int val, int val2, long mask)
{
	struct max31827_state *st = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		ret = regmap_update_bits(st->regmap, MAX31827_CONFIGURATION_REG,
					 MAX31827_CONFIGURATION_1SHOT_MASK |
					 MAX31827_CONFIGURATION_CNV_RATE_MASK,
					 MAX31827_1SHOT_EN(val));
		if (ret < 0)
			return ret;
		return 0;

	case IIO_CHAN_INFO_SAMP_FREQ:
		val = FIELD_PREP(MAX31827_CONFIGURATION_CNV_RATE_MASK, val);

		ret = regmap_update_bits(st->regmap,
					 MAX31827_CONFIGURATION_REG,
					 MAX31827_CONFIGURATION_CNV_RATE_MASK,
					 val);
		if (ret < 0)
			return ret;

		return 0;
	}

	return -EINVAL;
}

static int max31827_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			       unsigned int writeval, unsigned int *readval)
{
	struct max31827_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);

	return regmap_write(st->regmap, reg, writeval);
}

static const struct iio_info max31827_info = {
	.read_raw = &max31827_read_raw,
	.write_raw = &max31827_write_raw,
	.debugfs_reg_access = &max31827_reg_access,
	.read_event_value = &max31827_read_event_value,
	.write_event_value = &max31827_write_event_value,
};

static const struct iio_event_spec max31827_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
			.dir = IIO_EV_DIR_RISING,
			.mask_separate = BIT(IIO_EV_INFO_VALUE) |
					 BIT(IIO_EV_INFO_HYSTERESIS),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
			.dir = IIO_EV_DIR_FALLING,
			.mask_separate = BIT(IIO_EV_INFO_VALUE) |
					 BIT(IIO_EV_INFO_HYSTERESIS),
	},
};

static const struct iio_chan_spec max31827_channels[] = {
	{
		.type = IIO_TEMP,
		.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_RAW) |
					   BIT(IIO_CHAN_INFO_SAMP_FREQ) |
					   BIT(IIO_CHAN_INFO_ENABLE) |
					   BIT(IIO_CHAN_INFO_SCALE),
		.output = 0,
		.event_spec = max31827_events,
		.num_event_specs = ARRAY_SIZE(max31827_events),
	},
};

static int max31827_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct iio_dev *indio_dev;
	struct max31827_state *st;
	struct regmap *regmap;
	int ret;

	dev_info(&client->dev, "Entered probe function of max31827\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	i2c_set_clientdata(client, indio_dev);
	st->client = client;

	regmap = devm_regmap_init_i2c(client, &max31827_regmap);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&client->dev, "Failed to allocate regmap: %d\n", ret);
		return ret;
	}
	st->regmap = regmap;

	indio_dev->name = "max31827";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &max31827_info;

	indio_dev->channels = max31827_channels;
	indio_dev->num_channels = ARRAY_SIZE(max31827_channels);

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(&client->dev, client->irq,
						NULL,
						&max31827_event_handler,
						IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING,
						id->name,
						indio_dev);
	if (ret)
		return ret;
	}

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct i2c_device_id max31827_i2c_ids[] = {
	{ .name = "max31827", },
	{}
};
MODULE_DEVICE_TABLE(i2c, max31827_i2c_ids);

static const struct of_device_id max31827_of_match[] = {
	{ .compatible = "max31827", },
	{}
};
MODULE_DEVICE_TABLE(of, max31827_of_match);

static struct i2c_driver max31827_driver = {
	.driver = {
		.name = "max31827",
		.of_match_table = max31827_of_match,
	},
	.probe = max31827_probe,
	.id_table = max31827_i2c_ids,
};
module_i2c_driver(max31827_driver);

MODULE_AUTHOR("Daniel Matyas <daniel.matyas@analog.com>");
MODULE_DESCRIPTION("Maxim MAX31827 low-power temperature switch driver");
MODULE_LICENSE("GPL");
