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
#include <linux/iio/events.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#define AD7294_REG_CMD			 0x00
#define AD7294_REG_RESULT		 0x01
#define AD7294_REG_TEMP_BASE		 0x02
#define AD7294_REG_DAC(x)		 ((x) + 0x01)
#define AD7294_VOLTAGE_STATUS		 0x05
#define AD7294_CURRENT_STATUS		 0x06
#define AD7294_TEMP_STATUS		 0x07
#define AD7294_REG_CONFIG		 0x09
#define AD7294_REG_PWDN			 0x0A

#define AD7294_REG_VOLTAGE_DATA_LOW(x)	 ((x) * 3 + 0x0B)
#define AD7294_REG_VOLTAGE_DATA_HIGH(x)	 ((x) * 3 + 0x0C)
#define AD7294_REG_VOLTAGE_HYSTERESIS(x) ((x) * 3 + 0x0D)

#define AD7294_REG_CURRENT_DATA_LOW(x)	 ((x) * 3 + 0x17)
#define AD7294_REG_CURRENT_DATA_HIGH(x)	 ((x) * 3 + 0x18)
#define AD7294_REG_CURRENT_HYSTERESIS(x) ((x) * 3 + 0x19)

#define AD7294_REG_TEMP_DATA_LOW(x)	 ((x) * 3 + 0x1D)
#define AD7294_REG_TEMP_DATA_HIGH(x)	 ((x) * 3 + 0x1E)
#define AD7294_REG_TEMP_HYSTERESIS(x)	 ((x) * 3 + 0x1F)

#define AD7294_TEMP_VALUE_MASK		 GENMASK(10, 0)
#define AD7294_ADC_VALUE_MASK		 GENMASK(11, 0)
#define AD7294_ADC_EXTERNAL_REF_MASK	 BIT(5)
#define AD7294_DAC_EXTERNAL_REF_MASK	 BIT(4)
#define AD7294_DIFF_V3_V2		 BIT(1)
#define AD7294_DIFF_V1_V0		 BIT(0)
#define AD7294_ALERT_PIN		 BIT(2)
#define AD7294_ALERT_LOW(x)		 BIT((x) * 2)
#define AD7294_ALERT_HIGH(x)		 BIT((x) * 2 + 1)

#define AD7294_ADC_INTERNAL_VREF_MV	 2500
#define AD7294_DAC_INTERNAL_VREF_MV	 2500
#define AD7294_RESOLUTION		 12
#define AD7294_VOLTAGE_CHANNEL_COUNT	 4
#define AD7294_CURRENT_CHANNEL_COUNT	 2
#define AD7294_TEMP_CHANNEL_COUNT	 3

struct ad7294_state {
	/* Protects device raw read write operations */
	struct mutex lock;
	struct regmap *regmap;
	struct i2c_client *i2c;
	struct regulator *adc_vref_reg;
	struct regulator *dac_vref_reg;
	u32 shunt_ohms[2];
	u16 dac_value[2];
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

static int ad7294_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct i2c_client *client = context;
	int reg_size = ad7294_reg_size(reg);
	unsigned char buffer[3] = { reg };
	int ret;

	ret = i2c_master_send(client, buffer, 1);
	if (ret < 0)
		return ret;

	ret = i2c_master_recv(client, buffer + 1, reg_size);
	if (ret < 0)
		return ret;

	if (reg_size == 1)
		*val = buffer[1];
	else
		*val = buffer[1] << 8 | buffer[2];
	return 0;
};

static int ad7294_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct i2c_client *client = context;
	int reg_size = ad7294_reg_size(reg);
	unsigned char buffer[3] = { reg };
	int ret;

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

static bool ad7294_readable_reg(struct device *dev, unsigned int reg)
{
	return reg != AD7294_REG_CMD;
};

static const struct regmap_config ad7294_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = 0x27,
	.reg_read = ad7294_reg_read,
	.reg_write = ad7294_reg_write,
	.readable_reg = ad7294_readable_reg,
};

static const struct iio_event_spec ad7294_events[] = {
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_RISING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_FALLING,
		.mask_separate = BIT(IIO_EV_INFO_VALUE),
	},
	{
		.type = IIO_EV_TYPE_THRESH,
		.dir = IIO_EV_DIR_EITHER,
		.mask_separate = BIT(IIO_EV_INFO_HYSTERESIS),
	},
};

// clang-format off
#define AD7294_DAC_CHAN(_chan_id) {                           \
	.type = IIO_VOLTAGE,                                  \
	.channel = _chan_id,                                  \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.indexed = 1,                                         \
	.output = 1,                                          \
}

#define AD7294_VOLTAGE_CHAN(_type) {                          \
	.type = _type,                                        \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.indexed = 1,                                         \
	.output = 0,                                          \
	.event_spec = ad7294_events,                          \
	.num_event_specs = ARRAY_SIZE(ad7294_events),         \
}

#define AD7294_CURRENT_CHAN(_type) {                          \
	.type = _type,                                        \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW)          \
				| BIT(IIO_CHAN_INFO_SCALE),   \
	.indexed = 1,                                         \
	.output = 0,                                          \
	.event_spec = ad7294_events,                          \
	.num_event_specs = ARRAY_SIZE(ad7294_events),         \
}

#define AD7294_TEMP_CHAN(_chan_id) {                          \
	.type = IIO_TEMP,                                     \
	.channel = _chan_id,                                  \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),         \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
	.indexed = 1,                                         \
	.output = 0,                                          \
	.event_spec = ad7294_events,                          \
	.num_event_specs = ARRAY_SIZE(ad7294_events),         \
}
// clang-format on

enum ad7294_temp_chan {
	TSENSE_1,
	TSENSE_2,
	TSENSE_INTERNAL,
};

static const struct iio_chan_spec ad7294_chan_spec[] = {
	AD7294_DAC_CHAN(0),
	AD7294_DAC_CHAN(1),
	AD7294_DAC_CHAN(2),
	AD7294_DAC_CHAN(3),
	AD7294_VOLTAGE_CHAN(0),
	AD7294_VOLTAGE_CHAN(1),
	AD7294_VOLTAGE_CHAN(2),
	AD7294_VOLTAGE_CHAN(3),
	AD7294_CURRENT_CHAN(0),
	AD7294_CURRENT_CHAN(1),
	AD7294_TEMP_CHAN(TSENSE_1),
	AD7294_TEMP_CHAN(TSENSE_2),
	AD7294_TEMP_CHAN(TSENSE_INTERNAL),
};

static const char *const ad7294_power_supplies[] = {
	"vdrive",
	"avdd",
};

static irqreturn_t ad7294_event_handler(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	s64 timestamp = iio_get_time_ns(indio_dev);
	struct ad7294_state *st = iio_priv(indio_dev);
	unsigned int voltage_status, temp_status, current_status;
	int i;

	if (regmap_read(st->regmap, AD7294_VOLTAGE_STATUS, &voltage_status))
		return IRQ_HANDLED;

	if (regmap_read(st->regmap, AD7294_CURRENT_STATUS, &current_status))
		return IRQ_HANDLED;

	if (regmap_read(st->regmap, AD7294_TEMP_STATUS, &temp_status))
		return IRQ_HANDLED;

	if (!(voltage_status || current_status || temp_status))
		return IRQ_HANDLED;

	for (i = 0; i < AD7294_VOLTAGE_CHANNEL_COUNT; i++) {
		if (voltage_status & AD7294_ALERT_LOW(i))
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, i,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
		if (voltage_status & AD7294_ALERT_HIGH(i))
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_VOLTAGE, i,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
	}

	for (i = 0; i < AD7294_CURRENT_CHANNEL_COUNT; i++) {
		if (current_status & AD7294_ALERT_LOW(i))
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT, i,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
		if (current_status & AD7294_ALERT_HIGH(i))
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_CURRENT, i,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
	}

	for (i = 0; i < AD7294_TEMP_CHANNEL_COUNT; i++) {
		if (temp_status & AD7294_ALERT_LOW(i))
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_TEMP, i,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_FALLING),
				       timestamp);
		if (temp_status & AD7294_ALERT_HIGH(i))
			iio_push_event(indio_dev,
				       IIO_UNMOD_EVENT_CODE(IIO_TEMP, i,
							    IIO_EV_TYPE_THRESH,
							    IIO_EV_DIR_RISING),
				       timestamp);
	}

	return IRQ_HANDLED;
}

static int ad7294_adc_read(struct ad7294_state *st, int channel, int reg_base,
			   int *val)
{
	int ret;

	ret = regmap_write(st->regmap, AD7294_REG_CMD, BIT(channel + reg_base));
	if (ret)
		return ret;
	ret = regmap_read(st->regmap, AD7294_REG_RESULT, val);
	if (ret)
		return ret;
	*val &= AD7294_ADC_VALUE_MASK;

	return 0;
}

static int ad7294_voltage_scale(struct iio_chan_spec const *chan,
				struct ad7294_state *st, int *val)
{
	int ret;

	if (chan->output) {
		if (st->dac_vref_reg) {
			ret = regulator_get_voltage(st->dac_vref_reg);
			if (ret < 0)
				return ret;
			*val = ret / MILLI;
		} else {
			*val = AD7294_DAC_INTERNAL_VREF_MV;
		}
	} else {
		if (st->adc_vref_reg) {
			ret = regulator_get_voltage(st->adc_vref_reg);
			if (ret < 0)
				return ret;
			*val = ret / MILLI;
		} else {
			*val = AD7294_ADC_INTERNAL_VREF_MV;
		}
	}
	return 0;
}

static int ad7294_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct ad7294_state *st = iio_priv(indio_dev);
	unsigned int regval;
	int ret;

	guard(mutex)(&st->lock);
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		switch (chan->type) {
		case IIO_CURRENT:
			ret = ad7294_adc_read(st, chan->channel,
					      AD7294_VOLTAGE_CHANNEL_COUNT,
					      val);
			if (ret)
				return ret;
			return IIO_VAL_INT;
		case IIO_VOLTAGE:
			if (chan->output) {
				*val = st->dac_value[chan->channel];
				return IIO_VAL_INT;
			}
			ret = ad7294_adc_read(st, chan->channel, 0, val);
			if (ret)
				return ret;
			return IIO_VAL_INT;
		case IIO_TEMP:
			ret = regmap_read(st->regmap,
					  chan->channel + AD7294_REG_TEMP_BASE,
					  &regval);
			if (ret)
				return ret;
			regval &= AD7294_TEMP_VALUE_MASK;
			*val = sign_extend32(regval, 11);
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_VOLTAGE:
			ret = ad7294_voltage_scale(chan, st, val);
			if (ret)
				return ret;
			*val2 = AD7294_RESOLUTION;
			return IIO_VAL_FRACTIONAL_LOG2;
		case IIO_TEMP:
			/* Data resolution is 0.25 degree Celsius */
			*val = 250;
			return IIO_VAL_INT;
		case IIO_CURRENT:
			/* Current(in mA) =
			 *   ADC_READING * 100 / Shunt resistance (in ohm)
			 */
			*val = 100;
			*val2 = st->shunt_ohms[chan->channel & 1];
			return IIO_VAL_FRACTIONAL;
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

static int ad7294_reg_access(struct iio_dev *indio_dev, unsigned int reg,
			     unsigned int writeval, unsigned int *readval)
{
	struct ad7294_state *st = iio_priv(indio_dev);

	if (readval)
		return regmap_read(st->regmap, reg, readval);
	return regmap_write(st->regmap, reg, writeval);
}

static unsigned int ad7294_threshold_reg(const struct iio_chan_spec *chan,
					 enum iio_event_direction dir,
					 enum iio_event_info info)
{
	switch (chan->type) {
	case IIO_VOLTAGE:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			if (dir == IIO_EV_DIR_FALLING)
				return AD7294_REG_VOLTAGE_DATA_LOW(
					chan->channel);
			else
				return AD7294_REG_VOLTAGE_DATA_HIGH(
					chan->channel);
		case IIO_EV_INFO_HYSTERESIS:
			return AD7294_REG_VOLTAGE_HYSTERESIS(chan->channel);
		default:
			return 0;
		}
	case IIO_CURRENT:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			if (dir == IIO_EV_DIR_FALLING)
				return AD7294_REG_CURRENT_DATA_LOW(
					chan->channel);
			else
				return AD7294_REG_CURRENT_DATA_HIGH(
					chan->channel);
		case IIO_EV_INFO_HYSTERESIS:
			return AD7294_REG_CURRENT_HYSTERESIS(chan->channel);
		default:
			return 0;
		}
	case IIO_TEMP:
		switch (info) {
		case IIO_EV_INFO_VALUE:
			if (dir == IIO_EV_DIR_FALLING)
				return AD7294_REG_CURRENT_DATA_LOW(
					chan->channel);
			else
				return AD7294_REG_CURRENT_DATA_HIGH(
					chan->channel);
		case IIO_EV_INFO_HYSTERESIS:
			return AD7294_REG_CURRENT_HYSTERESIS(chan->channel);
		default:
			return 0;
		}
	default:
		return 0;
	}

	return 0;
}

static int ad7294_read_event_value(struct iio_dev *indio_dev,
				   const struct iio_chan_spec *chan,
				   enum iio_event_type type,
				   enum iio_event_direction dir,
				   enum iio_event_info info, int *val,
				   int *val2)
{
	int ret;
	unsigned int readval;
	struct ad7294_state *st = iio_priv(indio_dev);

	ret = regmap_read(st->regmap, ad7294_threshold_reg(chan, dir, info),
			  &readval);
	if (ret)
		return ret;

	if (chan->type == IIO_TEMP)
		*val = sign_extend32(readval, 11);
	else
		*val = readval & AD7294_ADC_VALUE_MASK;

	return IIO_VAL_INT;
}

static int ad7294_write_event_value(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    enum iio_event_type type,
				    enum iio_event_direction dir,
				    enum iio_event_info info, int val, int val2)
{
	struct ad7294_state *st = iio_priv(indio_dev);

	return regmap_write(st->regmap, ad7294_threshold_reg(chan, dir, info),
			    val);
}

struct iio_info ad7294_info = {
	.read_raw = ad7294_read_raw,
	.write_raw = ad7294_write_raw,
	.debugfs_reg_access = ad7294_reg_access,
	.read_event_value = &ad7294_read_event_value,
	.write_event_value = &ad7294_write_event_value,
};

static void ad7294_reg_disable(void *data)
{
	regulator_disable(data);
}

static const bool ad7294_diff_channels_valid(int chan1, int chan2)
{
	switch (chan1) {
	case 0:
		return chan2 == 1;
	case 1:
		return chan2 == 0;
	case 2:
		return chan2 == 3;
	case 3:
		return chan2 == 2;
	default:
		return false;
	}
}

static int ad7294_init(struct iio_dev *indio_dev, struct ad7294_state *st)
{
	int ret, diff_channels[2];
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

	ret = device_property_read_u32_array(&i2c->dev, "shunt-resistor-ohms",
					     st->shunt_ohms, 2);
	if (ret) {
		dev_err(&i2c->dev, "Failed to read shunt resistor values");
		return ret;
	}

	if (device_property_present(&i2c->dev, "diff-channels")) {
		ret = device_property_read_u32_array(&i2c->dev, "diff-channels",
						     diff_channels,
						     ARRAY_SIZE(diff_channels));
		if (ret) {
			dev_err(&i2c->dev,
				"Failed to get differential channels");
			return ret;
		}
		if (!ad7294_diff_channels_valid(diff_channels[0],
						diff_channels[1])) {
			dev_err(&i2c->dev, "Invalid differential channels");
			return -EINVAL;
		}

		/* TODO: Set differential to 1 for the corresponding channel */
		if (diff_channels[0] > 1)
			config_reg |= AD7294_DIFF_V3_V2;
		else
			config_reg |= AD7294_DIFF_V1_V0;
	}

	if (i2c->irq > 0) {
		ret = devm_request_threaded_irq(&i2c->dev, i2c->irq, NULL,
						ad7294_event_handler,
						IRQF_TRIGGER_LOW | IRQF_ONESHOT,
						"ad7294", indio_dev);
		if (ret)
			return ret;

		config_reg |= AD7294_ALERT_PIN;
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
	/* TODO:
	Memcpy ad7284_chan_spec to some writable space and store pointer in indio_dev->channels
	*/
	indio_dev->channels = ad7294_chan_spec;
	indio_dev->num_channels = ARRAY_SIZE(ad7294_chan_spec);

	st = iio_priv(indio_dev);
	st->i2c = client;

	ret = ad7294_init(indio_dev, st);
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
