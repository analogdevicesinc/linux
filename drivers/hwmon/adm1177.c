// SPDX-License-Identifier: GPL-2.0
/*
 * ADM1177 Hot Swap Controller and Digital Power Monitor with Soft Start Pin
 *
 * Copyright 2015-2019 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/regulator/consumer.h>

/*  Command Byte Operations */
#define ADM1177_CMD_V_CONT	BIT(0)
#define ADM1177_CMD_V_ONCE	BIT(1)
#define ADM1177_CMD_I_CONT	BIT(2)
#define ADM1177_CMD_I_ONCE	BIT(3)
#define ADM1177_CMD_VRANGE	BIT(4)
#define ADM1177_CMD_STATUS_RD	BIT(6)

/* Extended Register */
#define ADM1177_REG_ALERT_EN	1
#define ADM1177_REG_ALERT_TH	2
#define ADM1177_REG_CONTROL	3

/* ADM1177_REG_ALERT_EN */
#define ADM1177_EN_ADC_OC1	BIT(0)
#define ADM1177_EN_ADC_OC4	BIT(1)
#define ADM1177_EN_HS_ALERT	BIT(2)
#define ADM1177_EN_OFF_ALERT	BIT(3)
#define ADM1177_CLEAR		BIT(4)

/* ADM1177_REG_CONTROL */
#define ADM1177_SWOFF		BIT(0)

#define ADM1177_BITS		12

/**
 * struct adm1177_state - driver instance specific data
 * @client		pointer to i2c client
 * @reg			regulator info for the the power supply of the device
 * @command		internal control register
 * @r_sense_uohm	current sense resistor value
 * @alert_threshold_ua	current limit for shutdown
 * @vrange_high		internal voltage divider
 */
struct adm1177_state {
	struct i2c_client	*client;
	struct regulator	*reg;
	u8			command;
	u32			r_sense_uohm;
	u32			alert_threshold_ua;
	bool			vrange_high;
};

static int adm1177_read_raw(struct adm1177_state *st, u8 num, u8 *data)
{
	struct i2c_client *client = st->client;

	return i2c_master_recv(client, data, num);
}

static int adm1177_write_cmd(struct adm1177_state *st, u8 cmd)
{
	st->command = cmd;
	return i2c_smbus_write_byte(st->client, cmd);
}

static int adm1177_read(struct device *dev, enum hwmon_sensor_types type,
			u32 attr, int channel, long *val)
{
	struct adm1177_state *st = dev_get_drvdata(dev);
	u8 data[3];
	long dummy;
	int ret;

	switch (type) {
	case hwmon_curr:
		ret = adm1177_read_raw(st, 3, data);
		if (ret < 0)
			return ret;
		dummy = (data[1] << 4) | (data[2] & 0xF);
		/*
		 * convert in milliamperes
		 * ((105.84mV / 4096) x raw) / senseResistor(ohm)
		 */
		*val = div_u64((105840000ull * dummy), 4096 * st->r_sense_uohm);
		return 0;
	case hwmon_in:
		ret = adm1177_read_raw(st, 3, data);
		if (ret < 0)
			return ret;
		dummy = (data[0] << 4) | (data[2] >> 4);
		/*
		 * convert in millivolts based on resistor devision
		 * (V_fullscale / 4096) * raw
		 */
		if (st->command & ADM1177_CMD_VRANGE)
			*val = 6650;
		else
			*val = 26350;

		*val = ((*val * dummy) / 4096);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static umode_t adm1177_is_visible(const void *data,
				  enum hwmon_sensor_types type,
				  u32 attr, int channel)
{
	const struct adm1177_state *st = data;

	switch (type) {
	case hwmon_in:
		switch (attr) {
		case hwmon_in_input:
			return 0444;
		}
		break;
	case hwmon_curr:
		switch (attr) {
		case hwmon_curr_input:
			if (st->r_sense_uohm)
				return 0444;
			return 0;
		}
		break;
	default:
		break;
	}
	return 0;
}

static const u32 adm1177_curr_config[] = {
	HWMON_C_INPUT,
	0
};

static const struct hwmon_channel_info adm1177_curr = {
	.type = hwmon_curr,
	.config = adm1177_curr_config,
};

static const u32 adm1177_in_config[] = {
	HWMON_I_INPUT,
	0
};

static const struct hwmon_channel_info adm1177_in = {
	.type = hwmon_in,
	.config = adm1177_in_config,
};

static const struct hwmon_channel_info *adm1177_info[] = {
	&adm1177_curr,
	&adm1177_in,
	NULL
};

static const struct hwmon_ops adm1177_hwmon_ops = {
	.is_visible = adm1177_is_visible,
	.read = adm1177_read,
};

static const struct hwmon_chip_info adm1177_chip_info = {
	.ops = &adm1177_hwmon_ops,
	.info = adm1177_info,
};

static void adm1177_remove(void *data)
{
	struct adm1177_state *st = data;

	regulator_disable(st->reg);
}

static int adm1177_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device *hwmon_dev;
	struct adm1177_state *st;
	int ret;

	st = devm_kzalloc(dev, sizeof(*st), GFP_KERNEL);
	if (!st)
		return -ENOMEM;

	st->client = client;

	st->reg = devm_regulator_get_optional(&client->dev, "vref");
	if (IS_ERR(st->reg)) {
		if (PTR_ERR(st->reg) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		st->reg = NULL;
	} else {
		ret = regulator_enable(st->reg);
		if (ret)
			return ret;
		ret = devm_add_action_or_reset(&client->dev, adm1177_remove,
					       st);
		if (ret)
			return ret;
	}

	if (device_property_read_u32(dev, "adi,r-sense-micro-ohms",
				     &st->r_sense_uohm))
		st->r_sense_uohm = 0;
	if (device_property_read_u32(dev, "adi,shutdown-threshold-microamp",
				     &st->alert_threshold_ua))
		st->alert_threshold_ua = 0;
	st->vrange_high = device_property_read_bool(dev,
						    "adi,vrange-high-enable");
	if (st->alert_threshold_ua) {
		u64 val;

		val = (0xFFUL * st->alert_threshold_ua * st->r_sense_uohm);
		val = div_u64(val, 105840000U);
		if (val > 0xFF) {
			dev_warn(&client->dev,
				 "Requested shutdown current %d uA above limit\n",
				 st->alert_threshold_ua);

			val = 0xFF;
		}
		i2c_smbus_write_byte_data(st->client, ADM1177_REG_ALERT_TH,
					  val);
	}

	ret = adm1177_write_cmd(st, ADM1177_CMD_V_CONT |
				    ADM1177_CMD_I_CONT |
				    (st->vrange_high ? 0 : ADM1177_CMD_VRANGE));
	if (ret)
		return ret;

	hwmon_dev =
		devm_hwmon_device_register_with_info(dev, client->name, st,
						     &adm1177_chip_info, NULL);
	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id adm1177_id[] = {
	{"adm1177", 0},
	{}
};
MODULE_DEVICE_TABLE(i2c, adm1177_id);

static const struct of_device_id adm1177_dt_ids[] = {
	{ .compatible = "adi,adm1177" },
	{},
};
MODULE_DEVICE_TABLE(of, adm1177_dt_ids);

static struct i2c_driver adm1177_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "adm1177",
		.of_match_table = adm1177_dt_ids,
	},
	.probe = adm1177_probe,
	.id_table = adm1177_id,
};
module_i2c_driver(adm1177_driver);

MODULE_AUTHOR("Beniamin Bia <beniamin.bia@analog.com>");
MODULE_AUTHOR("Michael Hennerich <michael.hennerich@analog.com>");
MODULE_DESCRIPTION("Analog Devices ADM1177 ADC driver");
MODULE_LICENSE("GPL v2");
