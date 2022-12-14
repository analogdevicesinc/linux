// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for MAX31732 4-Channel Remote Temperature Sensor
 */

#include <linux/bitfield.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/regmap.h>

/* common definitions*/
#define MAX3173X_STOP			BIT(7)
#define MAX3173X_ALARM_MODE		BIT(4)
#define MAX3173X_ALARM_FAULT_QUEUE_MASK	GENMASK(3, 2)
#define MAX3173X_EXTRANGE		BIT(1)
#define MAX3173X_TEMP_OFFSET_BASELINE	0x77
#define MAX3173X_TEMP_MIN		(-128000)
#define MAX3173X_TEMP_MAX		127937
#define MAX3173X_OFFSET_MIN		(-14875)
#define MAX3173X_OFFSET_MAX		17000
#define MAX3173X_OFFSET_ZERO		14875
#define MAX31732_SECOND_TEMP_MIN	(-128000)
#define MAX31732_SECOND_TEMP_MAX	127000
#define MAX31732_CUSTOM_OFFSET_RES	125
#define MAX31732_ALL_CHANNEL_MASK	0x1F
#define MAX31732_ALARM_INT_MODE		0
#define MAX31732_ALARM_COMP_MODE	1
#define MAX31732_ALARM_FAULT_QUE	1
#define MAX31732_ALARM_FAULT_QUE_MAX	3

/* The MAX31732 registers */
#define MAX31732_REG_TEMP_R		0x02
#define MAX31732_REG_TEMP_L		0x0A
#define MAX31732_REG_PRIM_HIGH_STATUS	0x0C
#define MAX31732_REG_PRIM_LOW_STATUS	0x0D
#define MAX31732_REG_CHANNEL_ENABLE	0x0E
#define MAX31732_REG_CONF1		0x0F
#define MAX31732_REG_CONF2		0x10
#define MAX31732_REG_TEMP_OFFSET	0x16
#define MAX31732_REG_OFFSET_ENABLE	0x17
#define MAX31732_REG_ALARM1_MASK	0x1B
#define MAX31732_REG_ALARM2_MASK	0x1C
#define MAX31732_REG_PRIM_HIGH_TEMP_R	0x1D
#define MAX31732_REG_PRIM_HIGH_TEMP_L	0x25
#define MAX31732_REG_PRIM_LOW_TEMP	0x27
#define MAX31732_REG_SECOND_HIGH_TEMP_R	0x29
#define MAX31732_REG_SECOND_HIGH_TEMP_L	0x2D
#define MAX31732_REG_SECOND_LOW_TEMP	0x2E
#define MAX31732_REG_SECOND_HIGH_STATUS	0x42
#define MAX31732_REG_SECOND_LOW_STATUS	0x43
#define MAX31732_REG_TEMP_FAULT		0x44

enum max31732_temp_type {
	MAX31732_TEMP,
	MAX31732_PRIM_HIGH,
	MAX31732_SECOND_HIGH
};

struct max31732_data {
	struct i2c_client	*client;
	struct device		*hwmon_dev;
	struct regmap		*regmap;
	s32			irqs[2];
};

static u32 max31732_get_temp_reg(enum max31732_temp_type temp_type, u32 channel)
{
	switch (temp_type) {
	case MAX31732_PRIM_HIGH:
		if (channel == 0)
			return MAX31732_REG_PRIM_HIGH_TEMP_L;
		else
			return (MAX31732_REG_PRIM_HIGH_TEMP_R + (channel - 1) * 2);
		break;
	case MAX31732_SECOND_HIGH:
		if (channel == 0)
			return MAX31732_REG_SECOND_HIGH_TEMP_L;
		else
			return (MAX31732_REG_SECOND_HIGH_TEMP_R + (channel - 1));
		break;
	case MAX31732_TEMP:
	default:
		if (channel == 0)
			return MAX31732_REG_TEMP_L;
		else
			return (MAX31732_REG_TEMP_R + (channel - 1) * 2);
		break;
	}
}

static bool max31732_volatile_reg(struct device *dev, u32 reg)
{
	if (reg >= MAX31732_REG_TEMP_R && reg <= MAX31732_REG_PRIM_LOW_STATUS)
		return true;

	if (reg == MAX31732_REG_SECOND_HIGH_STATUS || reg == MAX31732_REG_SECOND_LOW_STATUS)
		return true;

	if (reg == MAX31732_REG_TEMP_FAULT)
		return true;

	return false;
}

static const struct regmap_config regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.volatile_reg = max31732_volatile_reg,
};

static inline long max31732_reg_to_mc(s16 temp)
{
	return DIV_ROUND_CLOSEST((temp / 16) * 1000, 16);
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

		*val = reg_val * 1000;
		return 0;
	case hwmon_temp_crit:
		reg_addr = max31732_get_temp_reg(MAX31732_SECOND_HIGH, channel);
		ret = regmap_read(data->regmap, reg_addr, &reg_val);
		if (ret)
			return ret;

		*val = reg_val * 1000;
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

		*val = (reg_val - MAX3173X_TEMP_OFFSET_BASELINE) * MAX31732_CUSTOM_OFFSET_RES;
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

	ret = regmap_bulk_read(data->regmap, reg_addr, &regs, 2);
	if (ret < 0)
		return ret;

	temp_reg_val = regs[1] | regs[0] << 8;
	*val = max31732_reg_to_mc(temp_reg_val);
	return 0;
}

static int max31732_write(struct device *dev, enum hwmon_sensor_types type, u32 attr, s32 channel,
			  long val)
{
	struct max31732_data *data = dev_get_drvdata(dev);
	s32 reg_addr, ret;
	u16 temp_reg_val;

	if (type != hwmon_temp)
		return -EINVAL;

	switch (attr) {
	case hwmon_temp_max:
		reg_addr = max31732_get_temp_reg(MAX31732_PRIM_HIGH, channel);
		break;
	case hwmon_temp_min:
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
		val = clamp_val(val, MAX3173X_OFFSET_MIN, MAX3173X_OFFSET_MAX) +
				MAX3173X_OFFSET_ZERO;
		val = DIV_ROUND_CLOSEST(val, 125);

		if (val == MAX3173X_TEMP_OFFSET_BASELINE) {
			ret = regmap_clear_bits(data->regmap, MAX31732_REG_OFFSET_ENABLE,
						BIT(channel));
		} else {
			ret = regmap_set_bits(data->regmap, MAX31732_REG_OFFSET_ENABLE,
					      BIT(channel));
		}
		if (ret)
			return ret;

		return regmap_write(data->regmap, MAX31732_REG_TEMP_OFFSET, val);
	case hwmon_temp_crit:
		val = clamp_val(val, MAX31732_SECOND_TEMP_MIN, MAX31732_SECOND_TEMP_MAX);
		val = DIV_ROUND_CLOSEST(val, 1000);
		reg_addr = max31732_get_temp_reg(MAX31732_SECOND_HIGH, channel);
		return regmap_write(data->regmap, reg_addr, val);
	case hwmon_temp_lcrit:
		val = clamp_val(val, MAX31732_SECOND_TEMP_MIN, MAX31732_SECOND_TEMP_MAX);
		val = DIV_ROUND_CLOSEST(val, 1000);
		return regmap_write(data->regmap, MAX31732_REG_SECOND_LOW_TEMP, val);
	default:
		return -EINVAL;
	}

	val = clamp_val(val, MAX3173X_TEMP_MIN, MAX3173X_TEMP_MAX);
	val = DIV_ROUND_CLOSEST(val << 4, 1000) << 4;

	temp_reg_val = (u16)val;
	temp_reg_val = swab16(temp_reg_val);

	return regmap_bulk_write(data->regmap, reg_addr, &temp_reg_val, sizeof(temp_reg_val));
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

static irqreturn_t max31732_irq_handler(s32 irq, void *data)
{
	struct device *dev = data;
	struct max31732_data *drvdata = dev_get_drvdata(dev);
	s32 ret;
	u32 reg_val;
	bool reported = false;

	ret = regmap_read(drvdata->regmap, MAX31732_REG_PRIM_HIGH_STATUS, &reg_val);
	if (ret)
		return ret;

	if (reg_val != 0) {
		dev_crit(dev, "Primary Overtemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
			 !!(reg_val & BIT(4)), !!(reg_val & BIT(3)), !!(reg_val & BIT(2)),
			 !!(reg_val & BIT(1)), !!(reg_val & BIT(0)));
		hwmon_notify_event(drvdata->hwmon_dev, hwmon_temp, hwmon_temp_max_alarm, 0);
		reported = true;
	}

	ret = regmap_read(drvdata->regmap, MAX31732_REG_PRIM_LOW_STATUS, &reg_val);
	if (ret)
		return ret;

	if (reg_val != 0) {
		dev_crit(dev, "Primary Undertemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
			 !!(reg_val & BIT(4)), !!(reg_val & BIT(3)), !!(reg_val & BIT(2)),
			 !!(reg_val & BIT(1)), !!(reg_val & BIT(0)));
		hwmon_notify_event(drvdata->hwmon_dev, hwmon_temp, hwmon_temp_min_alarm, 0);
		reported = true;
	}

	ret = regmap_read(drvdata->regmap, MAX31732_REG_SECOND_HIGH_STATUS, &reg_val);
	if (ret)
		return ret;

	if (reg_val != 0) {
		dev_crit(dev, "Secondary Overtemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
			 !!(reg_val & BIT(4)), !!(reg_val & BIT(3)), !!(reg_val & BIT(2)),
			 !!(reg_val & BIT(1)), !!(reg_val & BIT(0)));
		hwmon_notify_event(drvdata->hwmon_dev, hwmon_temp, hwmon_temp_crit_alarm, 0);
		reported = true;
	}

	ret = regmap_read(drvdata->regmap, MAX31732_REG_SECOND_LOW_STATUS, &reg_val);
	if (ret)
		return ret;

	if (reg_val != 0) {
		dev_crit(dev, "Secondary Undertemperature Alarm, R4:%d R3:%d R2:%d R1:%d L:%d.\n",
			 !!(reg_val & BIT(4)), !!(reg_val & BIT(3)), !!(reg_val & BIT(2)),
			 !!(reg_val & BIT(1)), !!(reg_val & BIT(0)));
		hwmon_notify_event(drvdata->hwmon_dev, hwmon_temp, hwmon_temp_lcrit_alarm, 0);
		reported = true;
	}

	if (!reported) {
		if (irq == drvdata->irqs[0])
			dev_err(dev, "ALARM1 interrupt received but status registers not set.\n");
		else if (irq == drvdata->irqs[1])
			dev_err(dev, "ALARM2 interrupt received but status registers not set.\n");
		else
			dev_err(dev, "Undefined interrupt source.\n");
	}

	return IRQ_HANDLED;
}

static const struct hwmon_channel_info *max31732_info[] = {
	HWMON_CHANNEL_INFO(chip,
			   HWMON_C_REGISTER_TZ),
	HWMON_CHANNEL_INFO(temp,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT |
			   HWMON_T_ENABLE |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM |
			   HWMON_T_LCRIT_ALARM,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT |
			   HWMON_T_OFFSET | HWMON_T_ENABLE |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM |
			   HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT |
			   HWMON_T_OFFSET | HWMON_T_ENABLE |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM |
			   HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT |
			   HWMON_T_OFFSET | HWMON_T_ENABLE |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM |
			   HWMON_T_LCRIT_ALARM |
			   HWMON_T_FAULT,
			   HWMON_T_INPUT | HWMON_T_MIN | HWMON_T_MAX | HWMON_T_LCRIT |
			   HWMON_T_CRIT |
			   HWMON_T_OFFSET | HWMON_T_ENABLE |
			   HWMON_T_MIN_ALARM | HWMON_T_MAX_ALARM | HWMON_T_CRIT_ALARM |
			   HWMON_T_LCRIT_ALARM |
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

	if (fwnode_property_read_bool(dev_fwnode(dev), "adi,alarm1-interrupt-mode"))
		ret = regmap_clear_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_ALARM_MODE);
	else
		ret = regmap_set_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_ALARM_MODE);

	if (ret)
		return ret;

	if (fwnode_property_read_bool(dev_fwnode(dev), "adi,alarm2-interrupt-mode"))
		ret = regmap_clear_bits(data->regmap, MAX31732_REG_CONF2, MAX3173X_ALARM_MODE);
	else
		ret = regmap_set_bits(data->regmap, MAX31732_REG_CONF2, MAX3173X_ALARM_MODE);

	if (ret)
		return ret;

	alarm_que = MAX31732_ALARM_FAULT_QUE;
	fwnode_property_read_u32(dev_fwnode(dev), "adi,alarm1-fault-queue", &alarm_que);

	if ((alarm_que / 2) <= MAX31732_ALARM_FAULT_QUE_MAX) {
		ret = regmap_write_bits(data->regmap, MAX31732_REG_CONF1,
					MAX3173X_ALARM_FAULT_QUEUE_MASK,
					FIELD_PREP(MAX3173X_ALARM_FAULT_QUEUE_MASK,
						   (alarm_que / 2)));
		if (ret)
			return ret;
	} else {
		return dev_err_probe(dev, -EINVAL, "Invalid adi,alarm1-fault-queue.\n");
	}

	alarm_que = MAX31732_ALARM_FAULT_QUE;
	fwnode_property_read_u32(dev_fwnode(dev), "adi,alarm2-fault-queue", &alarm_que);

	if ((alarm_que / 2) <= MAX31732_ALARM_FAULT_QUE_MAX) {
		ret = regmap_write_bits(data->regmap, MAX31732_REG_CONF2,
					MAX3173X_ALARM_FAULT_QUEUE_MASK,
					FIELD_PREP(MAX3173X_ALARM_FAULT_QUEUE_MASK,
						   (alarm_que / 2)));
	} else {
		return dev_err_probe(dev, -EINVAL, "Invalid adi,alarm2-fault-queue.\n");
	}

	return ret;
}

static int max31732_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max31732_data *data;
	s32 ret;
	u32 reg_val;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;

	data->regmap = devm_regmap_init_i2c(client, &regmap_config);
	if (IS_ERR(data->regmap))
		return dev_err_probe(dev, PTR_ERR(data->regmap), "regmap init failed\n");

	ret = regmap_read(data->regmap, MAX31732_REG_CHANNEL_ENABLE, &reg_val);
	if (ret)
		return ret;

	if (reg_val == 0)
		ret = regmap_set_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_STOP);
	else
		ret = regmap_clear_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_STOP);

	if (ret)
		return ret;

	ret = regmap_clear_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_EXTRANGE);
	if (ret)
		return ret;

	ret = max31732_parse_alarms(dev, data);
	if (ret)
		return ret;

	dev_set_drvdata(dev, data);

	data->irqs[0] = fwnode_irq_get_byname(dev_fwnode(dev), "ALARM1");
	if (data->irqs[0] > 0) {
		ret = devm_request_threaded_irq(dev, data->irqs[0], NULL, max31732_irq_handler,
						IRQF_ONESHOT, client->name, dev);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request irq\n");

		ret = regmap_set_bits(data->regmap, MAX31732_REG_ALARM1_MASK,
				      MAX31732_ALL_CHANNEL_MASK);
		if (ret)
			return ret;
	} else {
		ret = regmap_clear_bits(data->regmap, MAX31732_REG_ALARM1_MASK,
					MAX31732_ALL_CHANNEL_MASK);
		if (ret)
			return ret;
	}

	data->irqs[1] = fwnode_irq_get_byname(dev_fwnode(dev), "ALARM2");
	if (data->irqs[1] > 0) {
		ret = devm_request_threaded_irq(dev, data->irqs[1], NULL, max31732_irq_handler,
						IRQF_ONESHOT, client->name, dev);
		if (ret)
			return dev_err_probe(dev, ret, "cannot request irq\n");

		ret = regmap_set_bits(data->regmap, MAX31732_REG_ALARM2_MASK,
				      MAX31732_ALL_CHANNEL_MASK);
		if (ret)
			return ret;
	} else {
		ret = regmap_clear_bits(data->regmap, MAX31732_REG_ALARM2_MASK,
					MAX31732_ALL_CHANNEL_MASK);
		if (ret)
			return ret;
	}

	data->hwmon_dev = devm_hwmon_device_register_with_info(dev, client->name, data,
							       &max31732_chip_info, NULL);

	return PTR_ERR_OR_ZERO(data->hwmon_dev);
}

static const struct i2c_device_id max31732_ids[] = {
	{ "max31732" },
	{ },
};

MODULE_DEVICE_TABLE(i2c, max31732_ids);

static const struct of_device_id __maybe_unused max31732_of_match[] = {
	{ .compatible = "adi,max31732", },
	{ },
};

MODULE_DEVICE_TABLE(of, max31732_of_match);

static int __maybe_unused max31732_suspend(struct device *dev)
{
	struct max31732_data *data = dev_get_drvdata(dev);

	return regmap_set_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_STOP);
}

static int __maybe_unused max31732_resume(struct device *dev)
{
	struct max31732_data *data = dev_get_drvdata(dev);

	return regmap_clear_bits(data->regmap, MAX31732_REG_CONF1, MAX3173X_STOP);
}

static SIMPLE_DEV_PM_OPS(max31732_pm_ops, max31732_suspend, max31732_resume);

static struct i2c_driver max31732_driver = {
	.class		= I2C_CLASS_HWMON,
	.driver = {
		.name	= "max31732-driver",
		.of_match_table = of_match_ptr(max31732_of_match),
		.pm	= &max31732_pm_ops,
	},
	.probe_new	= max31732_probe,
	.id_table	= max31732_ids,
};

module_i2c_driver(max31732_driver);

MODULE_AUTHOR("Sinan Divarci <sinan.divarci@analog.com>");
MODULE_DESCRIPTION("MAX31732 driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
