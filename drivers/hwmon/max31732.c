// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for MAX31732 4-Channel Remote Temperature Sensor
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/property.h>

/* common definitions*/
#define MAX31732_TEMP_OFFSET_BASELINE	0x77
#define MAX31732_TEMP_MIN		(-128000)
#define MAX31732_TEMP_MAX		127937
#define MAX31732_TEMP_MIN_EXT		(MAX31732_TEMP_MIN + MAX31732_EXTENDED_RANGE_OFFSET)
#define MAX31732_TEMP_MAX_EXT		(MAX31732_TEMP_MAX + MAX31732_EXTENDED_RANGE_OFFSET)
#define MAX31732_OFFSET_MIN		(-14875)
#define MAX31732_OFFSET_MAX		17000
#define MAX31732_OFFSET_ZERO		14875
#define MAX31732_SECOND_TEMP_MIN	(-128000)
#define MAX31732_SECOND_TEMP_MAX	127000
#define MAX31732_SECOND_TEMP_MIN_EXT	(MAX31732_SECOND_TEMP_MIN + MAX31732_EXTENDED_RANGE_OFFSET)
#define MAX31732_SECOND_TEMP_MAX_EXT	(MAX31732_SECOND_TEMP_MAX + MAX31732_EXTENDED_RANGE_OFFSET)
#define MAX31732_CUSTOM_OFFSET_RES	125
#define MAX31732_ALL_CHANNEL_MASK	0x1F
#define MAX31732_ALARM_INT_MODE		0
#define MAX31732_ALARM_COMP_MODE	1
#define MAX31732_ALARM_FAULT_QUE	1
#define MAX31732_ALARM_FAULT_QUE_MAX	3
#define MAX31732_TEMP_STEP_MC		1000
#define MAX31732_TEMP_DIVISOR		16
#define MAX31732_EXTENDED_RANGE_OFFSET	64000

/* The MAX31732 registers */
#define MAX31732_REG_TEMP_R		0x02
#define MAX31732_REG_TEMP_L		0x0A
#define MAX31732_REG_PRIM_HIGH_STATUS	0x0C
#define MAX31732_REG_PRIM_LOW_STATUS	0x0D
#define MAX31732_REG_CHANNEL_ENABLE	0x0E
#define MAX31732_CHANNEL_ENABLE_EN_MTP_PU_LOAD	BIT(7)
#define MAX31732_REG_CONF1		0x0F
#define MAX31732_CONF1_STOP			BIT(7)
#define MAX31732_CONF1_SOFT_POR			BIT(6)
#define MAX31732_CONF1_ALARM_MODE		BIT(4)
#define MAX31732_CONF1_ALARM_FAULT_QUEUE	GENMASK(3, 2)
#define MAX31732_CONF1_EXTRANGE			BIT(1)
#define MAX31732_CONF1_ONE_SHOT			BIT(0)
#define MAX31732_REG_CONF2		0x10
#define MAX31732_CONF2_ALARM_MODE		BIT(4)
#define MAX31732_CONF2_ALARM_FAULT_QUEUE	GENMASK(3, 2)
#define MAX31732_CONF2_IGNORE_ISC2GND		BIT(0)
#define MAX31732_REG_CUSTOM_IDEAL_R1	0x11
#define MAX31732_REG_CUST_IDEAL_ENABLE	0x15
#define MAX31732_REG_TEMP_OFFSET	0x16
#define MAX31732_REG_OFFSET_ENABLE	0x17
#define MAX31732_REG_FILTER_ENABLE	0x18
#define MAX31732_REG_BETA_COMP_ENABLE	0x19
#define MAX31732_REG_HIGHEST_TEMP_ENABLE 0x1A
#define MAX31732_REG_ALARM1_MASK	0x1B
#define MAX31732_REG_ALARM2_MASK	0x1C
#define MAX31732_REG_PRIM_HIGH_TEMP_R	0x1D
#define MAX31732_REG_PRIM_HIGH_TEMP_L	0x25
#define MAX31732_REG_PRIM_LOW_TEMP	0x27
#define MAX31732_REG_SECOND_HIGH_TEMP_R	0x29
#define MAX31732_REG_SECOND_HIGH_TEMP_L	0x2D
#define MAX31732_REG_SECOND_LOW_TEMP	0x2E
#define MAX31732_REG_REFERENCE_TEMP_R1	0x2F
#define MAX31732_REG_REFERENCE_TEMP_R2	0x31
#define MAX31732_REG_REFERENCE_TEMP_R3	0x33
#define MAX31732_REG_REFERENCE_TEMP_R4	0x35
#define MAX31732_REG_REFERENCE_TEMP_L	0x37
#define MAX31732_REG_MTP_CONF		0x39
#define MAX31732_MTP_CONF_WR_ENABLE		BIT(7)
#define MAX31732_REG_MTP_CONF2		0x3A
#define MAX31732_MTP_CONF2_STORE		BIT(7)
#define MAX31732_MTP_CONF2_LOAD			BIT(5)
#define MAX31732_MTP_CONF2_MAN_WRITE		BIT(4)
#define MAX31732_MTP_CONF2_I2C_READ		BIT(3)
#define MAX31732_MTP_ADDRESS		0x3B
#define MAX31732_MTP_DIN		0x3C
#define MAX31732_REG_SECOND_HIGH_STATUS	0x42
#define MAX31732_REG_SECOND_LOW_STATUS	0x43
#define MAX31732_REG_TEMP_FAULT		0x44
#define MAX31732_REG_HIGHEST_TEMP	0x45
#define MAX31732_REG_BETA_COMPEN_R1	0x47
#define MAX31732_REG_BETA_COMPEN_R2	0x48
#define MAX31732_REG_BETA_COMPEN_R3	0x49
#define MAX31732_REG_BETA_COMPEN_R4	0x4A
#define MAX31732_BETA_COMPEN_OC			BIT(7)
#define MAX31732_BETA_COMPEN_SC2GND		BIT(6)
#define MAX31732_BETA_COMPEN_SC2VCC		BIT(5)
#define MAX31732_BETA_COMPEN_SC2DXP		BIT(4)
#define MAX31732_BETA_COMPEN_BETA		GENMASK(3, 0)

/* The MAX31732 MTP Registers */
#define MAX31732_MTP_REG_USER_SW_REV	0x80
#define MAX31732_MTP_REG_TEMP_R		0x82
#define MAX31732_MTP_REG_TEMP_L		0x8A
#define MAX31732_MTP_REG_PRIM_HIGH_STATUS 0x8C
#define MAX31732_MTP_REG_PRIM_LOW_STATUS 0x8D

enum max31732_temp_type {
	MAX31732_TEMP,
	MAX31732_PRIM_HIGH,
	MAX31732_SECOND_HIGH
};

enum max31732_channel {
	MAX31732_CHANNEL_LOCAL = 0,
	MAX31732_CHANNEL_REMOTE1,
	MAX31732_CHANNEL_REMOTE2,
	MAX31732_CHANNEL_REMOTE3,
	MAX31732_CHANNEL_REMOTE4
};

struct max31732_data {
	struct i2c_client	*client;
	struct device		*hwmon_dev;
	struct regmap		*regmap;
	s32			irqs[2];
	bool			extended_range;
};

static u32 max31732_get_temp_reg(enum max31732_temp_type temp_type, u32 channel)
{
	switch (temp_type) {
	case MAX31732_PRIM_HIGH:
		if (channel == MAX31732_CHANNEL_LOCAL)
			return MAX31732_REG_PRIM_HIGH_TEMP_L;
		else
			return (MAX31732_REG_PRIM_HIGH_TEMP_R + (channel - 1) * 2);
		break;
	case MAX31732_SECOND_HIGH:
		if (channel == MAX31732_CHANNEL_LOCAL)
			return MAX31732_REG_SECOND_HIGH_TEMP_L;
		else
			return (MAX31732_REG_SECOND_HIGH_TEMP_R + (channel - 1));
		break;
	case MAX31732_TEMP:
	default:
		if (channel == MAX31732_CHANNEL_LOCAL)
			return MAX31732_REG_TEMP_L;
		else
			return (MAX31732_REG_TEMP_R + (channel - 1) * 2);
		break;
	}
}

static const struct regmap_range max31732_regmap_volatile_ranges[] = {
	regmap_reg_range(MAX31732_REG_TEMP_R, MAX31732_REG_PRIM_LOW_STATUS),
	regmap_reg_range(MAX31732_REG_SECOND_HIGH_STATUS,
			 MAX31732_REG_BETA_COMPEN_R4),
	regmap_reg_range(MAX31732_REG_MTP_CONF2, MAX31732_REG_MTP_CONF2),
	regmap_reg_range(MAX31732_MTP_REG_USER_SW_REV, MAX31732_MTP_REG_PRIM_LOW_STATUS),
};

static bool max31732_volatile_reg(struct device *dev, u32 reg)
{
	return regmap_reg_in_ranges(reg, max31732_regmap_volatile_ranges,
				    ARRAY_SIZE(max31732_regmap_volatile_ranges));
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_MAPLE,
	.volatile_reg = max31732_volatile_reg,
};

static inline long max31732_reg_to_mc(s16 temp, bool extended_range)
{
	if (extended_range)
		return DIV_ROUND_CLOSEST((temp / MAX31732_TEMP_DIVISOR) * MAX31732_TEMP_STEP_MC,
					 MAX31732_TEMP_DIVISOR) + MAX31732_EXTENDED_RANGE_OFFSET;

	return DIV_ROUND_CLOSEST((temp / MAX31732_TEMP_DIVISOR) * MAX31732_TEMP_STEP_MC,
				 MAX31732_TEMP_DIVISOR);
}

static int max31732_read(struct device *dev, enum hwmon_sensor_types type, u32 attr, s32 channel,
			 long *val)
{
	struct max31732_data *data = dev_get_drvdata(dev);
	s32 ret;
	u32 reg_val, reg_addr;
	s16 temp_reg_val;
	u8 regs[2];

	if (type != hwmon_temp)
		return -EINVAL;

	switch (attr) {
	case hwmon_temp_input:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_CHANNEL_ENABLE, BIT(channel));
		if (ret < 0)
			return ret;

		if (!ret)
			return -ENODATA;

		reg_addr = max31732_get_temp_reg(MAX31732_TEMP, channel);
		break;
	case hwmon_temp_max:
		reg_addr = max31732_get_temp_reg(MAX31732_PRIM_HIGH, channel);
		break;
	case hwmon_temp_min:
		reg_addr = MAX31732_REG_PRIM_LOW_TEMP;
		break;
	case hwmon_temp_lcrit:
		ret = regmap_read(data->regmap, MAX31732_REG_SECOND_LOW_TEMP, &reg_val);
		if (ret)
			return ret;

		*val = data->extended_range ? (((s8)reg_val * MAX31732_TEMP_STEP_MC) +
					 MAX31732_EXTENDED_RANGE_OFFSET) :
					 ((s8)reg_val * MAX31732_TEMP_STEP_MC);
		return 0;
	case hwmon_temp_crit:
		reg_addr = max31732_get_temp_reg(MAX31732_SECOND_HIGH, channel);
		ret = regmap_read(data->regmap, reg_addr, &reg_val);
		if (ret)
			return ret;

		*val = data->extended_range ? (((s8)reg_val * MAX31732_TEMP_STEP_MC) +
					 MAX31732_EXTENDED_RANGE_OFFSET) :
					 ((s8)reg_val * MAX31732_TEMP_STEP_MC);
		return 0;
	case hwmon_temp_enable:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_CHANNEL_ENABLE, BIT(channel));
		if (ret < 0)
			return ret;

		*val = ret;
		return 0;
	case hwmon_temp_offset:
		if (channel == 0)
			return -EINVAL;

		ret = regmap_test_bits(data->regmap, MAX31732_REG_OFFSET_ENABLE, BIT(channel));
		if (ret < 0)
			return ret;

		if (!ret)
			return 0;

		ret = regmap_read(data->regmap, MAX31732_REG_TEMP_OFFSET, &reg_val);
		if (ret)
			return ret;

		*val = (long)((reg_val - MAX31732_TEMP_OFFSET_BASELINE) *
			      MAX31732_CUSTOM_OFFSET_RES);
		return 0;
	case hwmon_temp_fault:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_TEMP_FAULT, BIT(channel));
		if (ret < 0)
			return ret;

		*val = ret;
		return 0;
	case hwmon_temp_lcrit_alarm:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_SECOND_LOW_STATUS, BIT(channel));
		if (ret < 0)
			return ret;

		*val = ret;
		return 0;
	case hwmon_temp_min_alarm:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_PRIM_LOW_STATUS, BIT(channel));
		if (ret < 0)
			return ret;

		*val = ret;
		return 0;
	case hwmon_temp_max_alarm:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_PRIM_HIGH_STATUS, BIT(channel));
		if (ret < 0)
			return ret;

		*val = ret;
		return 0;
	case hwmon_temp_crit_alarm:
		ret = regmap_test_bits(data->regmap, MAX31732_REG_SECOND_HIGH_STATUS, BIT(channel));
		if (ret < 0)
			return ret;

		*val = ret;
		return 0;
	default:
		return -EINVAL;
	}

	ret = regmap_bulk_read(data->regmap, reg_addr, regs, 2);
	if (ret < 0)
		return ret;

	temp_reg_val = get_unaligned_be16(regs);
	*val = max31732_reg_to_mc(temp_reg_val, data->extended_range);
	return 0;
}

static int max31732_write(struct device *dev, enum hwmon_sensor_types type, u32 attr, s32 channel,
			  long val)
{
	struct max31732_data *data = dev_get_drvdata(dev);
	s32 reg_addr, ret;
	u16 temp_reg_val;
	u8 reg_arr[2];
	long min_temp, max_temp;

	if (type != hwmon_temp)
		return -EINVAL;

	switch (attr) {
	case hwmon_temp_max:
		reg_addr = max31732_get_temp_reg(MAX31732_PRIM_HIGH, channel);
		break;
	case hwmon_temp_min:
		if (channel)
			return -EOPNOTSUPP;
		reg_addr = MAX31732_REG_PRIM_LOW_TEMP;
		break;
	case hwmon_temp_enable:
		if (val == 0) {
			return regmap_clear_bits(data->regmap, MAX31732_REG_CHANNEL_ENABLE,
						 BIT(channel));
		} else if (val == 1) {
			return regmap_set_bits(data->regmap, MAX31732_REG_CHANNEL_ENABLE,
					       BIT(channel));
		} else {
			return -EINVAL;
		}
	case hwmon_temp_offset:
		val = clamp_val(val, MAX31732_OFFSET_MIN, MAX31732_OFFSET_MAX) +
				MAX31732_OFFSET_ZERO;
		val = DIV_ROUND_CLOSEST(val, 125);

		ret = regmap_update_bits(data->regmap, MAX31732_REG_OFFSET_ENABLE, BIT(channel),
			 val == MAX31732_TEMP_OFFSET_BASELINE ? 0 : BIT(channel));

		if (ret)
			return ret;

		return regmap_write(data->regmap, MAX31732_REG_TEMP_OFFSET, val);
	case hwmon_temp_crit:
		min_temp = data->extended_range ? MAX31732_SECOND_TEMP_MIN_EXT : MAX31732_SECOND_TEMP_MIN;
		max_temp = data->extended_range ? MAX31732_SECOND_TEMP_MAX_EXT : MAX31732_SECOND_TEMP_MAX;

		val = clamp_val(val, min_temp, max_temp);

		if (data->extended_range)
			val -= MAX31732_EXTENDED_RANGE_OFFSET;

		val = DIV_ROUND_CLOSEST(val, MAX31732_TEMP_STEP_MC);
		reg_addr = max31732_get_temp_reg(MAX31732_SECOND_HIGH, channel);
		return regmap_write(data->regmap, reg_addr, val);
	case hwmon_temp_lcrit:
		if (channel)
			return -EOPNOTSUPP;

		min_temp = data->extended_range ? MAX31732_SECOND_TEMP_MIN_EXT : MAX31732_SECOND_TEMP_MIN;
		max_temp = data->extended_range ? MAX31732_SECOND_TEMP_MAX_EXT : MAX31732_SECOND_TEMP_MAX;

		val = clamp_val(val, min_temp, max_temp);

		if (data->extended_range)
			val -= MAX31732_EXTENDED_RANGE_OFFSET;

		val = DIV_ROUND_CLOSEST(val, MAX31732_TEMP_STEP_MC);
		return regmap_write(data->regmap, MAX31732_REG_SECOND_LOW_TEMP, val);
	default:
		return -EINVAL;
	}

	min_temp = data->extended_range ? MAX31732_TEMP_MIN_EXT : MAX31732_TEMP_MIN;
	max_temp = data->extended_range ? MAX31732_TEMP_MAX_EXT : MAX31732_TEMP_MAX;

	val = clamp_val(val, min_temp, max_temp);

	if (data->extended_range)
		val -= MAX31732_EXTENDED_RANGE_OFFSET;

	val = DIV_ROUND_CLOSEST(val * MAX31732_TEMP_DIVISOR, MAX31732_TEMP_STEP_MC);
	val *= MAX31732_TEMP_DIVISOR;

	temp_reg_val = (u16)val;
	put_unaligned_be16(temp_reg_val, reg_arr);

	return regmap_bulk_write(data->regmap, reg_addr, reg_arr, sizeof(reg_arr));
}

static umode_t max31732_is_visible(const void *data, enum hwmon_sensor_types type, u32 attr,
				   s32 channel)
{
	switch (type) {
	case hwmon_temp:
		switch (attr) {
		case hwmon_temp_input:
		case hwmon_temp_lcrit_alarm:
		case hwmon_temp_min_alarm:
		case hwmon_temp_max_alarm:
		case hwmon_temp_crit_alarm:
		case hwmon_temp_fault:
			return 0444;
		case hwmon_temp_min:
		case hwmon_temp_lcrit:
			return channel ? 0444 : 0644;
		case hwmon_temp_offset:
		case hwmon_temp_enable:
		case hwmon_temp_max:
		case hwmon_temp_crit:
			return 0644;
		}
		break;
	default:
		break;
	}
	return 0;
}

static void notify_event(struct max31732_data *drvdata, u32 attr, u32 reg_val)
{
	s32 channel;

	for (channel = 0; channel <= MAX31732_CHANNEL_REMOTE4; channel++) {
		if (reg_val & BIT(channel))
			hwmon_notify_event(drvdata->hwmon_dev, hwmon_temp, attr, channel);
	}
}

static irqreturn_t max31732_irq_handler(s32 irq, void *data)
{
	struct device *dev = data;
	struct max31732_data *drvdata = dev_get_drvdata(dev);
	s32 ret;
	u32 reg_val;
	bool reported = false;

	ret = regmap_read(drvdata->regmap, MAX31732_REG_PRIM_HIGH_STATUS, &reg_val);
	if (ret) {
		dev_err_ratelimited(dev, "Failed to read PRIMARY HIGH STATUS register (%pe)\n",
				    ERR_PTR(ret));
		return IRQ_HANDLED;
	}

	if (reg_val != 0) {
		dev_crit_ratelimited(dev, "Primary Overtemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE4)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE3)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE2)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE1)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_LOCAL)));
		notify_event(drvdata, hwmon_temp_max_alarm, reg_val);
		reported = true;
	}

	ret = regmap_read(drvdata->regmap, MAX31732_REG_PRIM_LOW_STATUS, &reg_val);
	if (ret) {
		dev_err_ratelimited(dev, "Failed to read PRIMARY LOW STATUS register (%pe)\n",
				    ERR_PTR(ret));
		return IRQ_HANDLED;
	}

	if (reg_val != 0) {
		dev_crit_ratelimited(dev, "Primary Undertemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE4)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE3)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE2)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE1)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_LOCAL)));
		notify_event(drvdata, hwmon_temp_min_alarm, reg_val);
		reported = true;
	}

	ret = regmap_read(drvdata->regmap, MAX31732_REG_SECOND_HIGH_STATUS, &reg_val);
	if (ret) {
		dev_err_ratelimited(dev, "Failed to read SECONDARY HIGH STATUS register (%pe)\n",
				    ERR_PTR(ret));
		return IRQ_HANDLED;
	}

	if (reg_val != 0) {
		dev_crit_ratelimited(dev, "Secondary Overtemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE4)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE3)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE2)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE1)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_LOCAL)));
		notify_event(drvdata, hwmon_temp_crit_alarm, reg_val);
		reported = true;
	}

	ret = regmap_read(drvdata->regmap, MAX31732_REG_SECOND_LOW_STATUS, &reg_val);
	if (ret) {
		dev_err_ratelimited(dev, "Failed to read SECONDARY LOW STATUS register (%pe)\n",
				    ERR_PTR(ret));
		return IRQ_HANDLED;
	}

	if (reg_val != 0) {
		dev_crit_ratelimited(dev, "Secondary Undertemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE4)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE3)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE2)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_REMOTE1)),
				     !!(reg_val & BIT(MAX31732_CHANNEL_LOCAL)));
		notify_event(drvdata, hwmon_temp_lcrit_alarm, reg_val);
		reported = true;
	}

	if (!reported) {
		if (irq == drvdata->irqs[0])
			dev_err_ratelimited(dev, "ALARM1 interrupt received but status registers not set.\n");
		else if (irq == drvdata->irqs[1])
			dev_err_ratelimited(dev, "ALARM2 interrupt received but status registers not set.\n");
		else
			dev_err_ratelimited(dev, "Undefined interrupt source.\n");
	}

	return IRQ_HANDLED;
}

static const struct hwmon_channel_info *max31732_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX |
			   HWMON_T_LCRIT | HWMON_T_CRIT | HWMON_T_ENABLE |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM |
			   HWMON_T_CRIT_ALARM | HWMON_T_LCRIT_ALARM,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT | HWMON_T_OFFSET | HWMON_T_ENABLE | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM | HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT | HWMON_T_OFFSET | HWMON_T_ENABLE | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM | HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT | HWMON_T_OFFSET | HWMON_T_ENABLE | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM | HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT | HWMON_T_OFFSET | HWMON_T_ENABLE | HWMON_T_MIN_ALARM |
			   HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM | HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT
			   ),
	NULL
};

static const struct hwmon_ops max31732_hwmon_ops = {
	.is_visible = max31732_is_visible,
	.read = max31732_read,
	.write = max31732_write,
};

static const struct hwmon_chip_info max31732_chip_info = {
	.ops = &max31732_hwmon_ops,
	.info = max31732_info,
};

static int max31732_parse_alarms(struct device *dev, struct max31732_data *data)
{
	s32 ret;
	u32 alarm_que;

	ret = regmap_update_bits(data->regmap, MAX31732_REG_CONF1, MAX31732_CONF1_ALARM_MODE,
		device_property_read_bool(dev, "adi,alarm1-interrupt-mode") ? 0 : MAX31732_CONF1_ALARM_MODE);

	if (ret)
		return ret;

	ret = regmap_update_bits(data->regmap, MAX31732_REG_CONF2, MAX31732_CONF2_ALARM_MODE,
		device_property_read_bool(dev, "adi,alarm2-interrupt-mode") ? 0 : MAX31732_CONF2_ALARM_MODE);

	if (ret)
		return ret;

	alarm_que = MAX31732_ALARM_FAULT_QUE;
	device_property_read_u32(dev, "adi,alarm1-fault-queue", &alarm_que);

	if ((alarm_que / 2) <= MAX31732_ALARM_FAULT_QUE_MAX) {
		ret = regmap_write_bits(data->regmap, MAX31732_REG_CONF1,
					MAX31732_CONF1_ALARM_FAULT_QUEUE,
					FIELD_PREP(MAX31732_CONF1_ALARM_FAULT_QUEUE,
						   (alarm_que / 2)));
		if (ret)
			return ret;
	} else {
		return dev_err_probe(dev, -EINVAL, "Invalid adi,alarm1-fault-queue.\n");
	}

	alarm_que = MAX31732_ALARM_FAULT_QUE;
	device_property_read_u32(dev, "adi,alarm2-fault-queue", &alarm_que);

	if ((alarm_que / 2) <= MAX31732_ALARM_FAULT_QUE_MAX) {
		ret = regmap_write_bits(data->regmap, MAX31732_REG_CONF2,
					MAX31732_CONF2_ALARM_FAULT_QUEUE,
					FIELD_PREP(MAX31732_CONF2_ALARM_FAULT_QUEUE,
						   (alarm_que / 2)));
	} else {
		return dev_err_probe(dev, -EINVAL, "Invalid adi,alarm2-fault-queue.\n");
	}

	return ret;
}

static int max31732_parse_dt_config(struct device *dev, struct max31732_data *data)
{
	s32 ret;
	u32 val, i;
	u32 filter_channels = 0;
	u32 beta_comp_channels = 0;
	u32 highest_temp_channels = 0;

	/* Configure ignore short-circuit to ground */
	ret = regmap_update_bits(data->regmap, MAX31732_REG_CONF2,
				 MAX31732_CONF2_IGNORE_ISC2GND,
				 device_property_read_bool(dev, "adi,ignore-isc2gnd") ?
				 MAX31732_CONF2_IGNORE_ISC2GND : 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure ignore-isc2gnd\n");

	/* Configure custom ideality for remote channels */
	for (i = 0; i < 4; i++) {
		char prop_name[40];
		u32 ideality_factor;

		snprintf(prop_name, sizeof(prop_name), "adi,custom-ideality-factor-r%d", i + 1);
		if (!device_property_read_u32(dev, prop_name, &ideality_factor)) {
			if (ideality_factor > 0 && ideality_factor <= 255) {
				/* Write custom ideality code */
				ret = regmap_write(data->regmap,
						   MAX31732_REG_CUSTOM_IDEAL_R1 + i,
						   ideality_factor);
				if (ret)
					return dev_err_probe(dev, ret,
						"failed to set custom ideality for R%d\n", i + 1);

				/* Enable custom ideality for this channel */
				ret = regmap_set_bits(data->regmap,
						      MAX31732_REG_CUST_IDEAL_ENABLE,
						      BIT(i + 1));
				if (ret)
					return dev_err_probe(dev, ret,
						"failed to enable custom ideality for R%d\n", i + 1);
			} else {
				return dev_err_probe(dev, -EINVAL,
					"Invalid custom ideality factor for R%d (must be 1-255)\n", i + 1);
			}
		} else {
			/* Property not specified, disable custom ideality for this channel */
			ret = regmap_clear_bits(data->regmap,
						MAX31732_REG_CUST_IDEAL_ENABLE,
						BIT(i + 1));
			if (ret)
				return dev_err_probe(dev, ret,
					"failed to disable custom ideality for R%d\n", i + 1);
		}
	}

	/* Configure filter enable for remote channels (bit mask) */
	if (!device_property_read_u32(dev, "adi,filter-channels", &filter_channels)) {
		/* Mask out local channel (bit 0) and invalid channels */
		filter_channels &= 0x1E; /* Only remote channels 1-4 */
	}
	ret = regmap_write(data->regmap, MAX31732_REG_FILTER_ENABLE, filter_channels);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure filter channels\n");

	/* Configure beta compensation enable for remote channels (bit mask) */
	if (!device_property_read_u32(dev, "adi,beta-compensation-channels",
				       &beta_comp_channels)) {
		/* Mask out local channel (bit 0) and invalid channels */
		beta_comp_channels &= 0x1E; /* Only remote channels 1-4 */
	}
	ret = regmap_write(data->regmap, MAX31732_REG_BETA_COMP_ENABLE, beta_comp_channels);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure beta compensation channels\n");

	/* Configure highest temperature tracking enable (bit mask) */
	if (!device_property_read_u32(dev, "adi,highest-temp-channels",
				       &highest_temp_channels)) {
		highest_temp_channels &= MAX31732_ALL_CHANNEL_MASK;
	}
	ret = regmap_write(data->regmap, MAX31732_REG_HIGHEST_TEMP_ENABLE, highest_temp_channels);
	if (ret)
		return dev_err_probe(dev, ret, "failed to configure highest temp channels\n");

	/* Configure reference temperatures if provided */
	/* Local reference temperature */
	if (!device_property_read_u32(dev, "adi,reference-temp-local", &val)) {
		s16 temp_reg;
		u8 regs[2];

		val = clamp_val((s32)val,
				data->extended_range ? MAX31732_TEMP_MIN_EXT : MAX31732_TEMP_MIN,
				data->extended_range ? MAX31732_TEMP_MAX_EXT : MAX31732_TEMP_MAX);

		if (data->extended_range)
			val -= MAX31732_EXTENDED_RANGE_OFFSET;

		val = DIV_ROUND_CLOSEST((s32)val * MAX31732_TEMP_DIVISOR, MAX31732_TEMP_STEP_MC);
		val *= MAX31732_TEMP_DIVISOR;

		temp_reg = (s16)val;
		put_unaligned_be16(temp_reg, regs);

		ret = regmap_bulk_write(data->regmap, MAX31732_REG_REFERENCE_TEMP_L,
					regs, sizeof(regs));
		if (ret)
			return dev_err_probe(dev, ret,
				"failed to set local reference temperature\n");
	}

	/* Remote reference temperatures */
	for (i = 0; i < 4; i++) {
		char prop_name[32];

		snprintf(prop_name, sizeof(prop_name), "adi,reference-temp-remote%d", i + 1);
		if (!device_property_read_u32(dev, prop_name, &val)) {
			s16 temp_reg;
			u8 regs[2];
			u32 reg_addr = MAX31732_REG_REFERENCE_TEMP_R1 + (i * 2);

			val = clamp_val((s32)val,
					data->extended_range ? MAX31732_TEMP_MIN_EXT : MAX31732_TEMP_MIN,
					data->extended_range ? MAX31732_TEMP_MAX_EXT : MAX31732_TEMP_MAX);

			if (data->extended_range)
				val -= MAX31732_EXTENDED_RANGE_OFFSET;

			val = DIV_ROUND_CLOSEST((s32)val * MAX31732_TEMP_DIVISOR,
						MAX31732_TEMP_STEP_MC);
			val *= MAX31732_TEMP_DIVISOR;

			temp_reg = (s16)val;
			put_unaligned_be16(temp_reg, regs);

			ret = regmap_bulk_write(data->regmap, reg_addr, regs, sizeof(regs));
			if (ret)
				return dev_err_probe(dev, ret,
					"failed to set remote%d reference temperature\n", i + 1);
		}
	}

	return 0;
}

static int max31732_setup_irq(struct device *dev,
			      struct regmap *regmap,
			      const char *irqname,
			      int *irq,
			      unsigned int mask_reg,
			      unsigned int alarm_mask)
{
	int irq_num, ret;

	irq_num = fwnode_irq_get_byname(dev_fwnode(dev), irqname);
	if (irq_num == -EPROBE_DEFER)
		return dev_err_probe(dev, irq_num, "IRQ probe defer\n");

	if (irq_num == -ENOENT || irq_num == -ENXIO)
		return regmap_set_bits(regmap, mask_reg, MAX31732_ALL_CHANNEL_MASK);

	if (irq_num < 0)
		return 0;

	ret = regmap_update_bits(regmap, mask_reg, MAX31732_ALL_CHANNEL_MASK, alarm_mask);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set alarm mask\n");

	*irq = irq_num;

	ret = devm_request_threaded_irq(dev, irq_num, NULL,
					max31732_irq_handler,
					IRQF_ONESHOT,
					dev_name(dev), dev);
	if (ret)
		return dev_err_probe(dev, ret, "cannot request irq\n");

	return 0;
}

static int max31732_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max31732_data *data;
	s32 ret;
	u32 reg_val;
	u32 alarm1_mask = MAX31732_ALL_CHANNEL_MASK;
	u32 alarm2_mask = MAX31732_ALL_CHANNEL_MASK;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	data->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap), "regmap init failed\n");

	ret = regmap_read(data->regmap, MAX31732_REG_CHANNEL_ENABLE, &reg_val);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read channel enable register\n");

	ret = regmap_update_bits(data->regmap, MAX31732_REG_CONF1, MAX31732_CONF1_STOP, reg_val == 0 ? MAX31732_CONF1_STOP : 0);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set STOP bit per channel enable reg\n");

	ret = max31732_parse_alarms(dev, data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to parse alarms\n");

	dev_set_drvdata(dev, data);

	/* Set extended range in hardware and cache it */
	data->extended_range = device_property_read_bool(dev, "adi,extended-range");
	ret = regmap_update_bits(data->regmap, MAX31732_REG_CONF1, MAX31732_CONF1_EXTRANGE,
		data->extended_range ? MAX31732_CONF1_EXTRANGE : 0);

	if (ret)
		return dev_err_probe(dev, ret, "failed to set extended range\n");

	/* Parse and apply device tree configuration */
	ret = max31732_parse_dt_config(dev, data);
	if (ret)
		return ret;

	if (!device_property_read_u32(dev, "adi,alarm1-mask", &alarm1_mask)) {
		if (alarm1_mask & ~MAX31732_ALL_CHANNEL_MASK) {
			dev_warn(dev, "Invalid alarm1-mask=0x%x, applying 0x%x\n", alarm1_mask,
				 (alarm1_mask & MAX31732_ALL_CHANNEL_MASK));
			alarm1_mask &= MAX31732_ALL_CHANNEL_MASK;
		}
	}

	if (!device_property_read_u32(dev, "adi,alarm2-mask", &alarm2_mask)) {
		if (alarm2_mask & ~MAX31732_ALL_CHANNEL_MASK) {
			dev_warn(dev, "Invalid alarm2-mask=0x%x, applying 0x%x\n", alarm2_mask,
				 (alarm2_mask & MAX31732_ALL_CHANNEL_MASK));
			alarm2_mask &= MAX31732_ALL_CHANNEL_MASK;
		}
	}

	data->hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, data,
							       &max31732_chip_info, NULL);

	if (IS_ERR(data->hwmon_dev))
		return dev_err_probe(dev, PTR_ERR(data->hwmon_dev),
				     "failed to register hwmon device\n");

	/* Clear any pending status registers before setting up interrupts */
	regmap_read(data->regmap, MAX31732_REG_PRIM_HIGH_STATUS, &reg_val);
	regmap_read(data->regmap, MAX31732_REG_PRIM_LOW_STATUS, &reg_val);
	regmap_read(data->regmap, MAX31732_REG_SECOND_HIGH_STATUS, &reg_val);
	regmap_read(data->regmap, MAX31732_REG_SECOND_LOW_STATUS, &reg_val);

	/* Setup interrupts after hwmon device is registered and status cleared */
	ret = max31732_setup_irq(dev, data->regmap, "ALARM1", &data->irqs[0],
				 MAX31732_REG_ALARM1_MASK, alarm1_mask);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup ALARM1 irq\n");

	ret = max31732_setup_irq(dev, data->regmap, "ALARM2", &data->irqs[1],
				 MAX31732_REG_ALARM2_MASK, alarm2_mask);
	if (ret)
		return dev_err_probe(dev, ret, "failed to setup ALARM2 irq\n");

	return 0;
}

static const struct i2c_device_id max31732_ids[] = {
	{ "max31732" },
	{ }
};

MODULE_DEVICE_TABLE(i2c, max31732_ids);

static const struct of_device_id __maybe_unused max31732_of_match[] = {
	{ .compatible = "adi,max31732", },
	{ }
};

MODULE_DEVICE_TABLE(of, max31732_of_match);

static int max31732_suspend(struct device *dev)
{
	struct max31732_data *data = dev_get_drvdata(dev);

	return regmap_set_bits(data->regmap, MAX31732_REG_CONF1, MAX31732_CONF1_STOP);
}

static int max31732_resume(struct device *dev)
{
	struct max31732_data *data = dev_get_drvdata(dev);

	return regmap_clear_bits(data->regmap, MAX31732_REG_CONF1, MAX31732_CONF1_STOP);
}

static DEFINE_SIMPLE_DEV_PM_OPS(max31732_pm_ops, max31732_suspend, max31732_resume);

static struct i2c_driver max31732_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "max31732",
		.of_match_table = max31732_of_match,
		.pm	= pm_sleep_ptr(&max31732_pm_ops),
	},
	.probe	= max31732_probe,
	.id_table	= max31732_ids,
};

module_i2c_driver(max31732_driver);

MODULE_AUTHOR("Sinan Divarci <sinan.divarci@analog.com>");
MODULE_DESCRIPTION("MAX31732 driver");
MODULE_LICENSE("GPL");
