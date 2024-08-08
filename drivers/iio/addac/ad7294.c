// SPDX-License-Identifier: GPL-2.0
/*
 * AD7294/AD7294-2 I2C driver
 * Datasheet:
 *   https://www.analog.com/media/en/technical-documentation/data-sheets/AD7294.pdf
 *
 * Copyright (c) 2024 Analog Devices Inc.
 * Author: Anshul Dalal <anshulusr@gmail.com>
 */

#include <linux/i2c.h>
#include <linux/iio/iio.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#define AD7294_REG_CMD		     0x00
#define AD7294_REG_RESULT	     0x01
#define AD7294_REG_DAC(x)	     ((x) + 0x01)
#define AD7294_REG_PWDN		     0x0A

#define AD7294_TEMP_VALUE_MASK	     GENMASK(10, 0)
#define AD7294_ADC_VALUE_MASK	     GENMASK(11, 0)
#define AD7294_ADC_EXTERNAL_REF_MASK BIT(5)
#define AD7294_DAC_EXTERNAL_REF_MASK BIT(4)

#define AD7294_ADC_INTERNAL_VREF_MV  2500
#define AD7294_DAC_INTERNAL_VREF_MV  2500
#define AD7294_RESOLUTION	     12

#define AD7294_DAC_CHAN(_chan_id) {                           \
	.type = IIO_VOLTAGE,                                  \
	.channel = _chan_id,                                  \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.indexed = 1,                                         \
	.output = 1,                                          \
}

#define AD7294_ADC_CHAN(_type, _chan_id) {                    \
	.type = _type,                                        \
	.channel = _chan_id,                                  \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.indexed = 1,                                         \
	.output = 0,                                          \
}

#define AD7294_TEMP_CHAN(_chan_id) {                          \
	.type = IIO_TEMP,                                     \
	.channel = _chan_id,                                  \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.indexed = 1,                                         \
	.output = 0,                                          \
}

enum ad7294_temp_chan {
	TSENSE_1 = 0x02,
	TSENSE_2,
	TSENSE_INTERNAL,
};

static const char *const ad7294_power_supplies[] = {
	"vdrive",
	"avdd",
};

static bool ad7294_readable_reg(struct device *dev, unsigned int reg)
{
	return reg != AD7294_REG_CMD;
};

static int ad7294_reg_size(unsigned int reg)
{
	switch (reg) {
	case AD7294_REG_CMD:
	case AD7294_VOLTAGE_STATUS:
	case AD7294_CURRENT_STATUS:
	case AD7294_TEMP_STATUS:
	case AD7294_REG_PWDN:
		return 1;
	default:
		return 2;
	}
};

struct ad7294_state {
	struct mutex lock;
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct regulator *adc_vref_reg;
	struct regulator *dac_vref_reg;
	u16 dac_value[2];
};

static int ad7294_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	int ret;
	struct i2c_client *client = context;
	unsigned char buffer[3] = { reg };

	int reg_size = ad7294_reg_size(reg);

	ret = i2c_master_send(client, buffer, 1);
	if (ret < 0)
		return ret;

	ret = i2c_master_recv(client, buffer + 1, reg_size);
	if (ret < 0)
		return ret;

	dev_dbg(&client->dev, "Read [%x, %x] from reg:0x%x, size: %d",
		 buffer[1], buffer[2], reg, reg_size);
	if (reg_size == 1) {
		*val = buffer[1];
	} else {
		*val = buffer[1] << 8 | buffer[2];
	}

	return 0;
};

static int ad7294_reg_write(void *context, unsigned int reg, unsigned int val)
{
	int ret;
	struct i2c_client *client = context;
	unsigned char buffer[3] = { reg };

	int reg_size = ad7294_reg_size(reg);
	dev_dbg(&client->dev, "Write [%x] to reg: %x, size: %d", val, reg,
		 reg_size);

	if (reg_size == 1) {
		/* Only take LSB of the data when writing to 1 byte reg */
		buffer[1] = val & 0xff;
	} else {
		buffer[1] = val >> 8;
		buffer[2] = val & 0xff;
	}

	ret = i2c_master_send(client, buffer, reg_size + 1);
	if (ret < 0)
		return ret;

	return 0;
};

static const struct regmap_config ad7294_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x27,
	.reg_read = ad7294_reg_read,
	.reg_write = ad7294_reg_write,
	.readable_reg = ad7294_readable_reg,
};

static const struct iio_chan_spec ad7294_chan_spec[] = {
	AD7294_DAC_CHAN(0),
	AD7294_DAC_CHAN(1),
	AD7294_DAC_CHAN(2),
	AD7294_DAC_CHAN(3),
	AD7294_ADC_CHAN(IIO_VOLTAGE, 0),
	AD7294_ADC_CHAN(IIO_VOLTAGE, 1),
	AD7294_ADC_CHAN(IIO_VOLTAGE, 2),
	AD7294_ADC_CHAN(IIO_VOLTAGE, 3),
	AD7294_ADC_CHAN(IIO_CURRENT, 4),
	AD7294_ADC_CHAN(IIO_CURRENT, 5),
	AD7294_TEMP_CHAN(TSENSE_1),
	AD7294_TEMP_CHAN(TSENSE_2),
	AD7294_TEMP_CHAN(TSENSE_INTERNAL),
};

static int ad7294_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct ad7294_state *st = iio_priv(indio_dev);
	int ret, temperature;
	unsigned int regval;

	guard(mutex)(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_CURRENT:
			goto adc_read;
		case IIO_VOLTAGE:
			if (chan->output) {
				*val = st->dac_value[chan->channel];
				return IIO_VAL_INT;
			}
adc_read:
			ret = regmap_write(st->regmap, AD7294_REG_CMD,
					   BIT(chan->channel));
			if (ret)
				return ret;
			ret = regmap_read(st->regmap, AD7294_REG_RESULT,
					  &regval);
			if (ret)
				return ret;
			*val = regval & AD7294_ADC_VALUE_MASK;
			return IIO_VAL_INT;
		case IIO_TEMP:
			ret = regmap_read(st->regmap,
					  chan->channel + AD7294_REG_TEMP_BASE,
					  &regval);
			if (ret)
				return ret;
			regval &= AD7294_TEMP_VALUE_MASK;
			/* Raw data is read in 11-bit Two's completement format
			 * Reference: Datasheet Page#29
			*/
			temperature = regval & GENMASK(9, 0);
			if (regval & BIT(9))
				temperature -= (1 << 10);
			*val = temperature;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			if (chan->output) {
				if (st->dac_vref_reg) {
					ret = regulator_get_voltage(
						st->dac_vref_reg);
					if (ret < 0)
						return ret;
					*val = ret / MILLI;
				} else {
					*val = AD7294_DAC_INTERNAL_VREF_MV;
				}
			} else {
				if (st->adc_vref_reg) {
					ret = regulator_get_voltage(
						st->adc_vref_reg);
					if (ret < 0)
						return ret;
					*val = ret / MILLI;
				} else {
					*val = AD7294_ADC_INTERNAL_VREF_MV;
				}
			}
			*val2 = AD7294_RESOLUTION;
			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_TEMP:
			/* Data resolution is 0.25 degree Celsius */
			*val = 250;
			return IIO_VAL_INT;
		case IIO_CURRENT:
			/* TODO */
		default:
			return -EINVAL;
		}
	}
	return -EINVAL;
}

static int ad7294_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	int ret;
	struct ad7294_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		if (!chan->output)
			return -EINVAL;
		if (val < 0 || val >= BIT(AD7294_RESOLUTION) || val2)
			return -EINVAL;
		ret = regmap_write(st->regmap, AD7294_REG_DAC(chan->channel),
				   val);
		if (ret)
			return ret;
		st->dac_value[chan->channel] = val;
		return 0;
	}

	return -EINVAL;
}

static int ad7294_reg_access(struct iio_dev *indio_dev, unsigned reg,
			     unsigned writeval, unsigned *readval)
{
	struct ad7294_state *st = iio_priv(indio_dev);
	if (readval)
		return regmap_read(st->regmap, reg, readval);
	return regmap_write(st->regmap, reg, writeval);
}

struct iio_info ad7294_info = {
	.read_raw = ad7294_read_raw,
	.write_raw = ad7294_write_raw,
	.debugfs_reg_access = ad7294_reg_access,
};

static void ad7294_reg_disable(void *data)
{
	regulator_disable(data);
}

static int ad7294_init(struct ad7294_state *st)
{
	int ret;
	int pwdn_config, config_reg;
	struct i2c_client *i2c = st->i2c;

	mutex_init(&st->lock);

	st->regmap =
		devm_regmap_init(&i2c->dev, NULL, i2c, &ad7294_regmap_config);
	if (IS_ERR(st->regmap))
		return PTR_ERR(st->regmap);

	ret = regmap_read(st->regmap, AD7294_REG_PWDN, &pwdn_config);
	if (ret)
		return ret;

	ret = regmap_read(st->regmap, AD7294_REG_CONFIG, &config_reg);
	if (ret)
		return ret;

	ret = devm_regulator_bulk_get_enable(&i2c->dev,
					     ARRAY_SIZE(ad7294_power_supplies),
					     ad7294_power_supplies);
	if (ret)
		return dev_err_probe(&i2c->dev, ret,
				     "Failed to enable power supplies\n");

	st->adc_vref_reg = devm_regulator_get_optional(&i2c->dev, "adc-vref");
	if (IS_ERR(st->adc_vref_reg)) {
		ret = PTR_ERR(st->adc_vref_reg);
		if (ret != -ENODEV)
			return ret;
		dev_info(&i2c->dev,
			 "ADC Vref not found, using internal reference");
		pwdn_config &= ~AD7294_ADC_EXTERNAL_REF_MASK;
		st->adc_vref_reg = NULL;
	} else {
		ret = regulator_enable(st->adc_vref_reg);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(&i2c->dev, ad7294_reg_disable,
					       st->adc_vref_reg);
		if (ret)
			return ret;
		pwdn_config |= AD7294_ADC_EXTERNAL_REF_MASK;
	}

	st->dac_vref_reg = devm_regulator_get_optional(&i2c->dev, "dac-vref");
	if (IS_ERR(st->dac_vref_reg)) {
		ret = PTR_ERR(st->dac_vref_reg);
		if (ret != -ENODEV)
			return ret;
		dev_info(&i2c->dev,
			 "DAC Vref not found, using internal reference");
		pwdn_config &= ~AD7294_DAC_EXTERNAL_REF_MASK;
		st->dac_vref_reg = NULL;
	} else {
		ret = regulator_enable(st->dac_vref_reg);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(&i2c->dev, ad7294_reg_disable,
					       st->dac_vref_reg);
		if (ret)
			return ret;
		pwdn_config |= AD7294_DAC_EXTERNAL_REF_MASK;
	}

	ret = regmap_write(st->regmap, AD7294_REG_PWDN, pwdn_config);
	if (ret)
		return ret;
	ret = regmap_write(st->regmap, AD7294_REG_CONFIG, config_reg);
	if (ret)
		return ret;

	return 0;
};

static int ad7294_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int ret;
	struct iio_dev *indio_dev;
	struct ad7294_state *st;

	indio_dev = devm_iio_device_alloc(&client->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	indio_dev->name = "ad7294";
	indio_dev->info = &ad7294_info;
	indio_dev->channels = ad7294_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(ad7294_chan_spec);

	st = iio_priv(indio_dev);
	st->i2c = client;

	ret = ad7294_init(st);
	if (ret)
		return ret;

	return devm_iio_device_register(&client->dev, indio_dev);
}

static const struct of_device_id ad7294_of_table[] = {
	{ .compatible = "adi,ad7294" },
	{ .compatible = "adi,ad7294-2" },
	{ /* Sentinel */ },
};

static struct i2c_driver ad7294_driver = {
	.driver = {
		.name = "ad7294",
		.of_match_table = ad7294_of_table,
	},
	.probe = ad7294_probe,
};

module_i2c_driver(ad7294_driver);

MODULE_AUTHOR("Anshul Dalal <anshulusr@gmail.com>");
MODULE_DESCRIPTION("Analog Devices AD7294/AD7294-2 ADDAC");
MODULE_LICENSE("GPL");
