// SPDX-License-Identifier: GPL-2.0
/*
 * Analog Devices MAX31875 I2C Temperature Sensor driver
 *
 * Copyright 2024 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/units.h>

#define MAX31875_TEMP_REG	0x0
#define MAX31875_CONF_REG	0x1
#define MAX31875_T_HYST_REG	0x2
#define MAX31875_T_OS_REG	0x3

#define MAX31875_OVER_TEMP_STAT_MASK	BIT(15)
#define MAX31875_FAULT_QUEUE_MASK	GENMASK(12, 11)
#define MAX31875_COMP_INT_MASK		BIT(9)
#define MAX31875_SHUT_DOWN_MASK	BIT(8)
#define MAX31875_DATA_FORMAT_MASK	BIT(7)
#define MAX31875_RESOLUTION_MASK	GENMASK(6, 5)
#define MAX31875_TIMEOUT_MASK		BIT(4)
#define MAX31875_PEC_MASK		BIT(3)
#define MAX31875_CONV_RATE_MASK	GENMASK(2, 1)
#define MAX31875_ONESHOT_MASK		BIT(0)

#define MAX31875_FAULT_QUEUE(x)	FIELD_PREP(MAX31875_FAULT_QUEUE_MASK, (x) >> 1)
#define MAX31875_COMP_INT(x)		FIELD_PREP(MAX31875_COMP_INT_MASK, x)
#define MAX31875_SHUT_DOWN(x)		FIELD_PREP(MAX31875_SHUT_DOWN_MASK, x)
#define MAX31875_DATA_FORMAT(x)	FIELD_PREP(MAX31875_DATA_FORMAT_MASK, x)
#define MAX31875_RESOLUTION(x)		FIELD_PREP(MAX31875_RESOLUTION_MASK, x)
#define MAX31875_TIMEOUT(x)		FIELD_PREP(MAX31875_TIMEOUT_MASK, x)
#define MAX31875_ONESHOT(x)		FIELD_PREP(MAX31875_ONESHOT_MASK, x)

#define MAX31875_TEMP_OS_DEFAULT	(80 * MILLI)
#define MAX31875_TEMP_HYST_DEFAULT	(75 * MILLI)

enum max31875_conv {
	MAX31875_CONV_0_25_HZ,
	MAX31875_CONV_1_HZ,
	MAX31875_CONV_4_HZ,
	MAX31875_CONV_8_HZ,
};

static const u16 max31875_conversions[] = {
	[MAX31875_CONV_0_25_HZ] = 4000,
	[MAX31875_CONV_1_HZ] = 1000,
	[MAX31875_CONV_4_HZ] = 250,
	[MAX31875_CONV_8_HZ] = 125,
};

enum max31875_res {
	MAX31875_RES_8_BIT,
	MAX31875_RES_9_BIT,
	MAX31875_RES_10_BIT,
	MAX31875_RES_12_BIT,
	MAX31875_RES_EXTENDED,
};

static const u16 max31875_resolutions[] = {
	[MAX31875_RES_8_BIT] = 1000,
	[MAX31875_RES_9_BIT] = 500,
	[MAX31875_RES_10_BIT] = 250,
	[MAX31875_RES_12_BIT] = 62,
	[MAX31875_RES_EXTENDED] = 62,
};

static const u32 max31875_conv_times[] = {
	[MAX31875_RES_8_BIT] = 8750,
	[MAX31875_RES_9_BIT] = 17500,
	[MAX31875_RES_10_BIT] = 35000,
	[MAX31875_RES_12_BIT] = 140000,
	[MAX31875_RES_EXTENDED] = 280000,
};

struct max31875_data {
	/**
	 * Prevent simultaneous access to the i2c client.
	 */
	struct mutex lock;
	struct regmap *regmap;
	bool enable;
	u8 resolution;
	u16 update_interval;
	long temp_os;
	long temp_hyst;
};

static int shutdown_write(struct max31875_data *data, unsigned int reg,
			  unsigned int mask, unsigned int val)
{
	int ret;

	/**
	 * Before the Max Temperature, Max Temperature Hyteresis, Format
	 * and Resolution bits from Configuration register are changed over I2C,
	 * the part must be in shutdown mode.
	 *
	 * Mutex is used to ensure, that some other process doesn't change the
	 * configuration register.
	 */
	guard(mutex)(&data->lock);

	if (!data->enable) {
		if (!mask)
			ret = regmap_write(data->regmap, reg, val);
		else
			ret = regmap_update_bits(data->regmap, reg, mask, val);
		return ret;
	}

	ret = regmap_update_bits(data->regmap, MAX31875_CONF_REG,
				 MAX31875_SHUT_DOWN_MASK,
				 MAX31875_SHUT_DOWN(1));
	if (ret)
		return ret;

	if (!mask)
		ret = regmap_write(data->regmap, reg, val);
	else
		ret = regmap_update_bits(data->regmap, reg, mask, val);
	if (ret)
		return ret;

	return regmap_update_bits(data->regmap, MAX31875_CONF_REG,
				  MAX31875_SHUT_DOWN_MASK,
				  MAX31875_SHUT_DOWN(0));
}

static int max31875_update_bits(struct max31875_data *data, bool shutdown,
				 const u16 max31875_list[], size_t size,
				 const u16 val, const u16 mask, int *idx)
{
	int res = 0;
	int ret;

	/**
	 * Convert the desired value into register
	 * bits.
	 *
	 * This was inspired by lm73 driver.
	 */
	while (res < size && val < max31875_list[res])
		res++;

	if (res == size)
		res--;

	if (shutdown)
		ret = shutdown_write(data, MAX31875_CONF_REG, mask,
				     FIELD_PREP(mask, res));
	else
		ret = regmap_update_bits(data->regmap, MAX31875_CONF_REG, mask,
					 FIELD_PREP(mask, res));

	if (!ret && idx)
		*idx = res;

	return ret;
}

static inline void max31875_convert_raw_to_temp(u8 resolution, unsigned int raw,
						 long *val)
{
	bool extend = resolution == max31875_resolutions[MAX31875_RES_EXTENDED];

	*val = (sign_extend64(raw, 15) * MILLI) >> (extend ? 7 : 8);
}

static int max31875_read_temp(struct max31875_data *data, bool is_temp_input,
			       u8 reg, long *val)
{
	int ret;
	unsigned int uval;

	if (is_temp_input) {
		if (!data->enable) {
			ret = regmap_update_bits(data->regmap,
						 MAX31875_CONF_REG,
						 MAX31875_ONESHOT_MASK,
						 MAX31875_ONESHOT(1));
			if (ret)
				return ret;

			fsleep(max31875_conv_times[data->resolution]);
		}

		/**
		 * The conversion time for 12-bit resolution is 140ms and for
		 * 13-bit resolution is 280ms. So, 15ms and 155ms are added
		 * when the update interval is set to 125ms.
		 */
		if (data->update_interval ==
		    max31875_conversions[MAX31875_CONV_8_HZ]) {
			if (data->resolution == MAX31875_RES_12_BIT)
				fsleep(15000);
			else if (data->resolution == MAX31875_RES_EXTENDED)
				fsleep(155000);
		}
	}

	ret = regmap_read(data->regmap, reg, &uval);
	if (ret)
		return ret;

	max31875_convert_raw_to_temp(data->resolution, uval, val);

	return 0;
}

static inline void max31875_convert_temp_to_raw(u8 resolution, long *val)
{
	bool extend = resolution == max31875_resolutions[MAX31875_RES_EXTENDED];
	bool sign = *val < 0;

	*val *= sign ? -1 : 1;

	*val = ((*val << (extend ? 7 : 8)) / MILLI) * (sign ? -1 : 1);
}

static int max31875_write_temp(struct max31875_data *data, u8 reg, long val)
{
	max31875_convert_temp_to_raw(data->resolution, &val);

	return shutdown_write(data, reg, 0, (unsigned int)val);
}

static int max31875_init_client(struct max31875_data *data, struct device *dev)
{
	unsigned int res = 0;
	u8 propval;

	data->enable = true;

	res |= MAX31875_RESOLUTION(MAX31875_RES_10_BIT);

	res |= MAX31875_COMP_INT(device_property_read_bool(dev, "adi,comp-int"));

	res |= MAX31875_TIMEOUT(!device_property_read_bool(dev, "adi,timeout-enable"));

	if (!device_property_read_u8(dev, "adi,fault-q", &propval)) {
		/**
		 * Convert the desired fault queue into register bits.
		 */
		propval >>= 1;

		if (propval > 0x3) {
			dev_err(dev, "Invalid data in adi,fault-q\n");
			return -EINVAL;
		}

		res |= MAX31875_FAULT_QUEUE(propval);
	}

	return regmap_write(data->regmap, MAX31875_CONF_REG, res);
}

static ssize_t temp1_resolution_show(struct device *dev,
				     struct device_attribute *devattr,
				     char *buf)
{
	struct max31875_data *data = dev_get_drvdata(dev);
	unsigned int uval;
	int ret;

	ret = regmap_read(data->regmap, MAX31875_CONF_REG, &uval);
	if (ret)
		return ret;

	uval = FIELD_GET(MAX31875_DATA_FORMAT_MASK | MAX31875_RESOLUTION_MASK,
			 uval);

	uval = min(uval, 0x4U);

	return sysfs_emit(buf, "%u\n", max31875_resolutions[uval]);
}

static ssize_t temp1_resolution_store(struct device *dev,
				      struct device_attribute *devattr,
				      const char *buf, size_t count)
{
	struct max31875_data *data = dev_get_drvdata(dev);
	int ret;
	unsigned int uval;

	ret = kstrtouint(buf, 10, &uval);
	if (ret)
		return ret;

	ret = max31875_update_bits(data, true, max31875_resolutions,
				   ARRAY_SIZE(max31875_resolutions), uval,
				   MAX31875_DATA_FORMAT_MASK |
				   MAX31875_RESOLUTION_MASK,
				   (int *)&data->resolution);

	return ret ? ret : count;
}

static DEVICE_ATTR_RW(temp1_resolution);

static struct attribute *max31875_attrs[] = {
	&dev_attr_temp1_resolution.attr,
	NULL
};
ATTRIBUTE_GROUPS(max31875);

static umode_t max31875_is_visible(const void *_data,
				    enum hwmon_sensor_types type, u32 attr,
				    int channel)
{
	if (type == hwmon_chip && attr == hwmon_chip_update_interval) {
		return 0644;
	} else if (type == hwmon_temp) {
		if (attr == hwmon_temp_input || attr == hwmon_temp_max_alarm)
			return 0444;
		else if (attr == hwmon_temp_enable || attr == hwmon_temp_max ||
			 attr == hwmon_temp_max_hyst)
			return 0644;
	}

	return 0;
}

static int max31875_read(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long *val)
{
	int ret;
	struct max31875_data *data = dev_get_drvdata(dev);
	unsigned int uval;

	if (type == hwmon_chip && attr == hwmon_chip_update_interval) {
		ret = regmap_read(data->regmap, MAX31875_CONF_REG, &uval);
		if (ret)
			return ret;

		uval = FIELD_GET(MAX31875_CONV_RATE_MASK, uval);

		*val = max31875_conversions[uval];

		return 0;
	} else if (type == hwmon_temp) {
		if (attr == hwmon_temp_enable) {
			guard(mutex)(&data->lock);

			ret = regmap_read(data->regmap, MAX31875_CONF_REG,
					  &uval);
			if (ret)
				return ret;

			*val = !FIELD_GET(MAX31875_SHUT_DOWN_MASK, uval);

			return 0;
		} else if (attr == hwmon_temp_input) {
			guard(mutex)(&data->lock);

			return max31875_read_temp(data, true, MAX31875_TEMP_REG,
						  val);
		} else if (attr == hwmon_temp_max) {
			return max31875_read_temp(data, false,
						  MAX31875_T_OS_REG, val);
		} else if (attr == hwmon_temp_max_hyst) {
			return max31875_read_temp(data, false,
						  MAX31875_T_HYST_REG, val);
		} else if (attr == hwmon_temp_max_alarm) {
			ret = regmap_read(data->regmap, MAX31875_CONF_REG,
					  &uval);
			if (ret)
				return ret;

			*val = FIELD_GET(MAX31875_OVER_TEMP_STAT_MASK, uval);

			return 0;
		}
	}

	return -EOPNOTSUPP;
}

static int max31875_write(struct device *dev, enum hwmon_sensor_types type,
			   u32 attr, int channel, long val)
{
	struct max31875_data *data = dev_get_drvdata(dev);
	int ret;

	if (type == hwmon_chip && attr == hwmon_chip_update_interval) {
		ret = max31875_update_bits(data, false, max31875_conversions,
					   ARRAY_SIZE(max31875_conversions),
					   (const u16)val,
					   MAX31875_CONV_RATE_MASK, NULL);

		if (!ret)
			data->update_interval = (u16)val;

		return ret;
	} else if (type == hwmon_temp) {
		if (attr == hwmon_temp_enable) {
			if (val >> 1)
				return -EOPNOTSUPP;

			guard(mutex)(&data->lock);

			ret = regmap_update_bits(data->regmap,
						 MAX31875_CONF_REG,
						 MAX31875_SHUT_DOWN_MASK,
						 MAX31875_SHUT_DOWN(!val));
			if (!ret)
				data->enable = val;

			return ret;
		} else if (attr == hwmon_temp_max) {
			/**
			 * The temperature hysteresis must be less than or
			 * equal to the temperature over limit.
			 *
			 * So, if the temperature over limit is less than the
			 * temperature hysteresis, the temperature hysteresis is
			 * set to the temperature over limit.
			 */
			if (val < data->temp_hyst) {
				data->temp_hyst = val;
				ret = max31875_write_temp(data,
							  MAX31875_T_HYST_REG,
							  val);
				if (ret)
					return ret;
			}

			ret = max31875_write_temp(data, MAX31875_T_OS_REG,
						   val);
			if (!ret)
				data->temp_os = val;

			return ret;
		} else if (attr == hwmon_temp_max_hyst) {
			/**
			 * The temperature hysteresis must be less than or
			 * equal to the temperature over limit.
			 *
			 * So, if the temperature hysteresis is greater than the
			 * temperature over limit, the temperature hyteresis is
			 * set to the temperature over limit.
			 */
			if (val > data->temp_os)
				val = data->temp_os;

			ret = max31875_write_temp(data, MAX31875_T_HYST_REG,
						   val);
			if (!ret)
				data->temp_hyst = val;

			return ret;
		}
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops max31875_hwmon_ops = {
	.is_visible = max31875_is_visible,
	.read = max31875_read,
	.write = max31875_write,
};

static const struct hwmon_channel_info *max31875_hwmon_channel_info[] = {
	HWMON_CHANNEL_INFO(temp, HWMON_T_ENABLE | HWMON_T_INPUT | HWMON_T_MAX |
			   HWMON_T_MAX_HYST | HWMON_T_MAX_ALARM),
	HWMON_CHANNEL_INFO(chip, HWMON_C_UPDATE_INTERVAL),
	NULL,
};

static const struct hwmon_chip_info max31875_chip_info = {
	.ops = &max31875_hwmon_ops,
	.info = max31875_hwmon_channel_info,
};

static const struct regmap_config max31875_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = MAX31875_T_OS_REG,
};

static int max31875_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max31875_data *data;
	struct device *hwmon_dev;
	int ret;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EOPNOTSUPP;

	data = devm_kzalloc(dev, sizeof(struct max31875_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	mutex_init(&data->lock);

	ret = devm_regulator_get_enable(dev, "vref");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable regulator\n");

	data->temp_os = MAX31875_TEMP_OS_DEFAULT;
	data->temp_hyst = MAX31875_TEMP_HYST_DEFAULT;

	data->regmap = devm_regmap_init_i2c(client, &max31875_regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap),
				     "Failed to allocate regmap.\n");

	ret = max31875_init_client(data, dev);
	if (ret)
		return ret;

	i2c_set_clientdata(client, data);

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &max31875_chip_info,
							 max31875_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id max31875_id[] = {
	{ "max31875", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, max31875_id);

static const struct of_device_id max31875_of_match[] = {
	{ .compatible = "adi,max31875" },
	{ },
};
MODULE_DEVICE_TABLE(of, max31875_of_match);

static struct i2c_driver max31875_driver = {
	.driver = {
		.name = "max31875",
		.of_match_table = max31875_of_match,
	},
	.probe_new = max31875_probe,
	.id_table = max31875_id,
};
module_i2c_driver(max31875_driver)

MODULE_AUTHOR("John Erasmus Mari Geronimo <johnerasmusmari.geronimo@analog.com");
MODULE_DESCRIPTION("MAX31875 low-power I2C temperature sensor");
MODULE_LICENSE("GPL");
