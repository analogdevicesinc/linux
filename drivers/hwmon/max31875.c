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

#define MAX31875_FAULT_QUEUE(x)		FIELD_PREP(MAX31875_FAULT_QUEUE_MASK, x)
#define MAX31875_COMP_INT(x)		FIELD_PREP(MAX31875_COMP_INT_MASK, x)
#define MAX31875_SHUT_DOWN(x)		FIELD_PREP(MAX31875_SHUT_DOWN_MASK, x)
#define MAX31875_DATA_FORMAT(x)		FIELD_PREP(MAX31875_DATA_FORMAT_MASK, x)
#define MAX31875_RESOLUTION(x)		FIELD_PREP(MAX31875_RESOLUTION_MASK, x)
#define MAX31875_TIMEOUT(x)		FIELD_PREP(MAX31875_TIMEOUT_MASK, x)
#define MAX31875_ONESHOT(x)		FIELD_PREP(MAX31875_ONESHOT_MASK, x)

#define MAX31875_TEMP_OS_DEFAULT	(80 * MILLI)
#define MAX31875_TEMP_HYST_DEFAULT	(75 * MILLI)

#define MAX31875_8_BIT_CNV_TIME		8750
#define MAX31875_9_BIT_CNV_TIME		17500
#define MAX31875_10_BIT_CNV_TIME	35000
#define MAX31875_12_BIT_CNV_TIME	140000
#define MAX31875_EXTENDED_CNV_TIME	280000

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
};

static const u16 max31875_resolutions[] = {
	[MAX31875_RES_8_BIT] = 1000,
	[MAX31875_RES_9_BIT] = 500,
	[MAX31875_RES_10_BIT] = 250,
	[MAX31875_RES_12_BIT] = 62,
};

static const u32 max31875_conv_times[] = {
	[MAX31875_RES_8_BIT] = MAX31875_8_BIT_CNV_TIME,
	[MAX31875_RES_9_BIT] = MAX31875_9_BIT_CNV_TIME,
	[MAX31875_RES_10_BIT] = MAX31875_10_BIT_CNV_TIME,
	[MAX31875_RES_12_BIT] = MAX31875_12_BIT_CNV_TIME,
};

struct max31875_data {
	/**
	 * Prevent simultaneous access to the i2c client.
	 */
	struct mutex lock;
	struct regmap *regmap;
	bool enable;
	bool extend_fmt;
	u8 bit_reso;
	u16 update_interval;
	long temp_os;
	long temp_hyst;
};

static int max31875_shutdown_write(struct max31875_data *data, unsigned int reg,
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
				const u16 val, const u16 mask, u8 *idx)
{
	u8 res = 0;
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
		ret = max31875_shutdown_write(data, MAX31875_CONF_REG, mask,
					      FIELD_PREP(mask, res));
	else
		ret = regmap_update_bits(data->regmap, MAX31875_CONF_REG, mask,
					 FIELD_PREP(mask, res));

	if (ret)
		return ret;

	if (idx)
		*idx = res;

	return 0;
}

static inline void max31875_convert_raw_to_temp(bool extend_fmt, unsigned int raw,
						long *val)
{
	/**
	 * In the extended format, the MSB is increased from 64C to 128C.
	 */
	s32 temp = (sign_extend32(raw, 15) * (long)MILLI) >> (extend_fmt ? 7 : 8);

	*val = temp;
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

			if (data->extend_fmt)
				fsleep(MAX31875_EXTENDED_CNV_TIME);
			else
				fsleep(max31875_conv_times[data->bit_reso]);
		}

		/**
		 * The conversion time for 12-bit resolution is 140ms and for
		 * 13-bit resolution (extended) is 280ms. So, 15ms and 155ms are
		 * added when the update interval is set to 125ms.
		 */
		if (data->update_interval == max31875_conversions[MAX31875_CONV_8_HZ]) {
			if (data->extend_fmt)
				fsleep(155000);
			else if (data->bit_reso == MAX31875_RES_12_BIT)
				fsleep(15000);
		}
	}

	ret = regmap_read(data->regmap, reg, &uval);
	if (ret)
		return ret;

	max31875_convert_raw_to_temp(data->extend_fmt, uval, val);

	return 0;
}

static inline void max31875_convert_temp_to_raw(bool extend_fmt, long *val)
{
	/**
	 * In the extended format, the MSB is increased from 64C to 128C.
	 */
	*val = (*val << (extend_fmt ? 7 : 8)) / (long)MILLI;
}

static int max31875_write_temp(struct max31875_data *data, u8 reg, long val)
{
	max31875_convert_temp_to_raw(data->extend_fmt, &val);

	return max31875_shutdown_write(data, reg, 0, val);
}

static int max31875_init_client(struct max31875_data *data, struct device *dev)
{
	unsigned int res = 0;
	int ret;
	u32 propval;

	data->enable = true;

	res |= MAX31875_RESOLUTION(MAX31875_RES_10_BIT);

	if (device_property_read_bool(dev, "adi,comp-int"))
		res |= MAX31875_COMP_INT(1);

	if (device_property_read_bool(dev, "adi,timeout-disable"))
		res |= MAX31875_TIMEOUT(1);

	if (!device_property_read_u32(dev, "adi,fault-q", &propval)) {
		/**
		 * Convert the desired fault queue into register bits.
		 */
		switch (propval) {
		case 1:
		case 2:
		case 4:
		case 6:
			res |= MAX31875_FAULT_QUEUE(propval >> 1);
			break;
		default:
			return dev_err_probe(dev, -EINVAL,
					     "Invalid data in adi,fault-q\n");
		}
	}

	if (device_property_read_bool(dev, "adi,extended-format")) {
		data->extend_fmt = true;
		res |= MAX31875_DATA_FORMAT(1);

		ret = max31875_write_temp(data, MAX31875_T_OS_REG,
					  MAX31875_TEMP_OS_DEFAULT);
		if (ret)
			return ret;

		ret = max31875_write_temp(data, MAX31875_T_HYST_REG,
					  MAX31875_TEMP_HYST_DEFAULT);
		if (ret)
			return ret;
	} else {
		data->extend_fmt = false;
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

	/**
	 * The resolution of extended format is the same as the 12-bit
	 * resolution.
	 */
	if (data->extend_fmt)
		return sysfs_emit(buf, "%u\n",
				  max31875_resolutions[MAX31875_RES_12_BIT]);

	ret = regmap_read(data->regmap, MAX31875_CONF_REG, &uval);
	if (ret)
		return ret;

	uval = FIELD_GET(MAX31875_RESOLUTION_MASK, uval);

	if (uval >= ARRAY_SIZE(max31875_resolutions))
		return -EIO;

	return sysfs_emit(buf, "%u\n", max31875_resolutions[uval]);
}

static ssize_t temp1_resolution_store(struct device *dev,
				      struct device_attribute *devattr,
				      const char *buf, size_t count)
{
	struct max31875_data *data = dev_get_drvdata(dev);
	int ret;
	unsigned int uval;

	if (data->extend_fmt)
		return -EOPNOTSUPP;

	ret = kstrtouint(buf, 10, &uval);
	if (ret)
		return ret;

	ret = max31875_update_bits(data, true, max31875_resolutions,
				   ARRAY_SIZE(max31875_resolutions), uval,
				   MAX31875_RESOLUTION_MASK,
				   &data->bit_reso);

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
	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			return 0644;
		default:
			return 0;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_max_alarm:
			return 0444;
		case hwmon_temp_enable:
		case hwmon_temp_max:
		case hwmon_temp_max_hyst:
			return 0644;
		default:
			return 0;
		}
	default:
		return 0;
	}
}

static int max31875_read(struct device *dev, enum hwmon_sensor_types type,
			 u32 attr, int channel, long *val)
{
	int ret;
	struct max31875_data *data = dev_get_drvdata(dev);
	unsigned int uval;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			ret = regmap_read(data->regmap, MAX31875_CONF_REG,
					  &uval);
			if (ret)
				return ret;

			uval = FIELD_GET(MAX31875_CONV_RATE_MASK, uval);

			*val = max31875_conversions[uval];

			return 0;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			scoped_guard(mutex, &data->lock) {
				ret = regmap_read(data->regmap,
						  MAX31875_CONF_REG, &uval);
				if (ret)
					return ret;

				*val = !FIELD_GET(MAX31875_SHUT_DOWN_MASK,
						  uval);
			}

			return 0;
		case hwmon_temp_input:
			scoped_guard(mutex, &data->lock)
				ret = max31875_read_temp(data, true,
							 MAX31875_TEMP_REG,
							 val);

			return ret;
		case hwmon_temp_max:
			return max31875_read_temp(data, false,
						  MAX31875_T_OS_REG, val);
		case hwmon_temp_max_hyst:
			return max31875_read_temp(data, false,
						  MAX31875_T_HYST_REG, val);
		case hwmon_temp_max_alarm:
			ret = regmap_read(data->regmap, MAX31875_CONF_REG,
					  &uval);
			if (ret)
				return ret;

			*val = FIELD_GET(MAX31875_OVER_TEMP_STAT_MASK, uval);

			return 0;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
}

static int max31875_write(struct device *dev, enum hwmon_sensor_types type,
			  u32 attr, int channel, long val)
{
	struct max31875_data *data = dev_get_drvdata(dev);
	int ret;

	switch (type) {
	case hwmon_chip:
		switch (attr) {
		case hwmon_chip_update_interval:
			ret = max31875_update_bits(data, false,
					max31875_conversions,
					ARRAY_SIZE(max31875_conversions),
					(const u16)val, MAX31875_CONV_RATE_MASK,
					NULL);
			if (!ret)
				data->update_interval = (u16)val;

			return ret;
		default:
			return -EOPNOTSUPP;
		}
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_enable:
			if (val >> 1)
				return -EOPNOTSUPP;

			scoped_guard(mutex, &data->lock) {
				ret = regmap_update_bits(data->regmap,
						MAX31875_CONF_REG,
						MAX31875_SHUT_DOWN_MASK,
						MAX31875_SHUT_DOWN(!val));
				if (!ret)
					data->enable = val;
			}
			return ret;
		case hwmon_temp_max:
			/**
			 * The temperature hysteresis must be less than
			 * or equal to the temperature over limit.
			 *
			 * So, if the temperature over limit is less
			 * than the temperature hysteresis, the
			 * temperature hysteresis is set to the
			 * temperature over limit.
			 */
			if (val < data->temp_hyst) {
				data->temp_hyst = val;

				ret = max31875_write_temp(data,
							  MAX31875_T_HYST_REG,
							  val);
				if (ret)
					return ret;
			}

			ret = max31875_write_temp(data, MAX31875_T_OS_REG, val);
			if (!ret)
				data->temp_os = val;

			return ret;
		case hwmon_temp_max_hyst:
			/**
			 * The temperature hysteresis must be less than
			 * or equal to the temperature over limit.
			 *
			 * So, if the temperature hysteresis is greater
			 * than the temperature over limit, the
			 * temperature hyteresis is set to the
			 * temperature over limit.
			 */
			if (val > data->temp_os)
				val = data->temp_os;

			ret = max31875_write_temp(data, MAX31875_T_HYST_REG,
						  val);
			if (!ret)
				data->temp_hyst = val;

			return ret;
		default:
			return -EOPNOTSUPP;
		}
	default:
		return -EOPNOTSUPP;
	}
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

	hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name,
							 data,
							 &max31875_chip_info,
							 max31875_groups);

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

static const struct i2c_device_id max31875_id[] = {
	{ "max31875", },
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
