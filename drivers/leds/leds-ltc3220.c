// SPDX-License-Identifier: GPL-2.0
/*
 * LTC3220 18-Channel LED Driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Edelweise Escala <edelweise.escala@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/types.h>

/* LTC3220 Registers */
#define LTC3220_COMMAND				0x00
#define LTC3220_ULED01				0x01
#define LTC3220_ULED02				0x02
#define LTC3220_ULED03				0x03
#define LTC3220_ULED04				0x04
#define LTC3220_ULED05				0x05
#define LTC3220_ULED06				0x06
#define LTC3220_ULED07				0x07
#define LTC3220_ULED08				0x08
#define LTC3220_ULED09				0x09
#define LTC3220_ULED10				0x0A
#define LTC3220_ULED11				0x0B
#define LTC3220_ULED12				0x0C
#define LTC3220_ULED13				0x0D
#define LTC3220_ULED14				0x0E
#define LTC3220_ULED15				0x0F
#define LTC3220_ULED16				0x10
#define LTC3220_ULED17				0x11
#define LTC3220_ULED18				0x12
#define LTC3220_GRAD_BLINK			0x13

#define LTC3220_GRAD_COUNT_UP		BIT(0)
#define LTC3220_COMMAND_QUICK_WRITE	BIT(0)
#define LTC3220_COMMAND_SHUTDOWN	BIT(3)

#define LTC3220_LED_CURRENT_MASK	GENMASK(5, 0)
#define LTC3220_LED_MODE_MASK		GENMASK(7, 6)
#define LTC3220_BLINK_MASK			GENMASK(4, 3)
#define LTC3220_GRADATION_MASK		GENMASK(2, 1)
#define LTC3220_CPO_COMMAND_MASK	GENMASK(2, 1)

#define LTC3220_NUM_LEDS 18

struct ltc3220_command_cfg {
	bool is_quick_write;
	bool is_shutdown;
	u8 force_cpo_mode;
};

struct ltc3220_uled_cfg {
	u8 led_index;
	u8 reg_value;
	struct led_classdev led_cdev;
};

struct ltc3220_grad_cfg {
	bool is_increasing;
	u8 gradation_period_ms;
};

struct ltc3220_state {
	u8 blink_mode;
	struct i2c_client	*client;
	struct ltc3220_command_cfg command_cfg;
	struct ltc3220_uled_cfg uled_cfg[LTC3220_NUM_LEDS];
	struct ltc3220_grad_cfg grad_cfg;
};

static int ltc3220_set_command(struct ltc3220_state *ltc3220_state,
			       bool is_shutdown)
{
	u8 reg_val;
	int ret;
	struct i2c_client *client = ltc3220_state->client;

	reg_val = FIELD_PREP(LTC3220_COMMAND_SHUTDOWN, is_shutdown);
	reg_val |= FIELD_PREP(LTC3220_CPO_COMMAND_MASK,
			      ltc3220_state->command_cfg.force_cpo_mode);
	reg_val |= FIELD_PREP(LTC3220_COMMAND_QUICK_WRITE,
			      ltc3220_state->command_cfg.is_quick_write);
	ret = i2c_smbus_write_byte_data(client, LTC3220_COMMAND, reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "Set command fail\n");
		return ret;
	}

	ltc3220_state->command_cfg.is_shutdown = is_shutdown;

	return 0;
}

static int ltc3220_set_led_data(struct led_classdev *led_cdev,
						    enum led_brightness brightness)
{
	int ret;
	int i = 0;
	struct ltc3220_state *ltc3220_state;
	struct ltc3220_uled_cfg *uled_cfg;
	struct device *parent_dev;
	struct i2c_client *client;

	if (!led_cdev->dev)
		return -ENODEV;

	parent_dev = led_cdev->dev->parent;
	if (!parent_dev)
		return -ENODEV;

	if (!parent_dev || strcmp(parent_dev->bus->name, "i2c") != 0)
		return -ENODEV;

	client = to_i2c_client(parent_dev);
	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);
	ltc3220_state = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(client,
			LTC3220_ULED01 + uled_cfg->led_index, brightness);
	if (ret < 0)
		return ret;

	if (ltc3220_state->command_cfg.is_quick_write && uled_cfg->led_index == 0) {
		for (i = 0; i < LTC3220_NUM_LEDS; i++)
			ltc3220_state->uled_cfg[i].reg_value = brightness;
	} else {
		uled_cfg->reg_value = brightness;
	}
	return 0;
}

static enum led_brightness ltc3220_get_led_data(struct led_classdev *led_cdev)
{
	struct ltc3220_uled_cfg *uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);

	return uled_cfg->reg_value;
}

static int ltc3220_set_blink_and_gradation(struct ltc3220_state *ltc3220_state,
		    u8 blink_cfg, u8 gradation_period_ms, bool is_increasing)
{
	u8 reg_val;
	int ret;
	struct i2c_client *client = ltc3220_state->client;

	reg_val = FIELD_PREP(LTC3220_BLINK_MASK, blink_cfg);
	reg_val |= FIELD_PREP(LTC3220_GRADATION_MASK, gradation_period_ms);
	reg_val |= FIELD_PREP(LTC3220_GRAD_COUNT_UP, is_increasing);

	ret = i2c_smbus_write_byte_data(client, LTC3220_GRAD_BLINK, reg_val);
	if (ret < 0) {
		dev_err(&client->dev, "Set blinking and gradation fail\n");
		return ret;
	}

	return 0;
}

/*
 * LTC3220 has a global blink configuration that affects all LEDs.
 * This implementation allows per-LED blink requests, but the blink timing
 * will be shared across all LEDs. The delay values are mapped to the
 * hardware's discrete blink rates.
 */
static int ltc3220_blink_set(struct led_classdev *led_cdev,
			     unsigned long *delay_on,
			     unsigned long *delay_off)
{
	struct ltc3220_state *ltc3220_state;
	struct device *parent_dev;
	struct i2c_client *client;
	u8 blink_mode;
	unsigned long period;
	int ret;

	if (!led_cdev->dev || !led_cdev->dev->parent)
		return -ENODEV;

	parent_dev = led_cdev->dev->parent;
	client = to_i2c_client(parent_dev);
	ltc3220_state = i2c_get_clientdata(client);

	if (*delay_on == 0 && *delay_off == 0) {
		blink_mode = 1;
		*delay_on = 500;
		*delay_off = 500;
	} else {
		period = *delay_on + *delay_off;

		if (period <= 750) {
			blink_mode = 0;
			*delay_on = 250;
			*delay_off = 250;
		} else if (period <= 1500) {
			blink_mode = 1;
			*delay_on = 500;
			*delay_off = 500;
		} else if (period <= 3000) {
			blink_mode = 2;
			*delay_on = 1000;
			*delay_off = 1000;
		} else {
			blink_mode = 3;
			*delay_on = 2000;
			*delay_off = 2000;
		}
	}

	ret = ltc3220_set_blink_and_gradation(ltc3220_state,
					       blink_mode,
					       ltc3220_state->grad_cfg.gradation_period_ms,
					       ltc3220_state->grad_cfg.is_increasing);
	if (ret < 0)
		return ret;

	ltc3220_state->blink_mode = blink_mode;

	return 0;
}

static ssize_t ltc3220_gradation_period_ms_show(struct device *dev,
								 struct device_attribute *attr, char *buf)
{
	struct ltc3220_state *ltc3220_state = dev_get_drvdata(dev);
	u16 period_ms;

	switch (ltc3220_state->grad_cfg.gradation_period_ms) {
	case 0:
		period_ms = 0;
		break;
	case 1:
		period_ms = 313;
		break;
	case 2:
		period_ms = 625;
		break;
	case 3:
		period_ms = 1250;
		break;
	default:
		period_ms = 0;
		break;
	}

	return sysfs_emit(buf, "%u\n", period_ms);
}

static ssize_t ltc3220_gradation_period_ms_store(struct device *dev,
					  struct device_attribute *dev_attr,
					  const char *buf, size_t size)
{
	u8 hw_value;
	int ret;
	struct ltc3220_state *ltc3220_state = dev_get_drvdata(dev);
	u16 period_ms;

	ret = kstrtou16(buf, 10, &period_ms);
	if (ret)
		return ret;

	if (period_ms > 1250)
		period_ms = 1250;

	if (period_ms == 0)
		hw_value = 0;
	else if (period_ms <= 313)
		hw_value = 1;
	else if (period_ms <= 625)
		hw_value = 2;
	else
		hw_value = 3;

	ret = ltc3220_set_blink_and_gradation(ltc3220_state,
					       ltc3220_state->blink_mode,
					       hw_value,
					       ltc3220_state->grad_cfg.is_increasing);
	if (ret < 0)
		return ret;

	ltc3220_state->grad_cfg.gradation_period_ms = hw_value;

	return size;
}
static DEVICE_ATTR(gradation_period_ms, 0644,
		ltc3220_gradation_period_ms_show,
		ltc3220_gradation_period_ms_store);

static ssize_t ltc3220_gradation_mode_show(struct device *dev,
					    struct device_attribute *attr,
					    char *buf)
{
	struct ltc3220_state *ltc3220_state = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%s\n",
			  ltc3220_state->grad_cfg.is_increasing ? "ramp_up" : "ramp_down");
}

static ssize_t ltc3220_gradation_mode_store(struct device *dev,
					     struct device_attribute *dev_attr,
					     const char *buf, size_t size)
{
	bool is_increasing;
	int ret;
	struct ltc3220_state *ltc3220_state = dev_get_drvdata(dev);

	if (sysfs_streq(buf, "ramp_up")) {
		is_increasing = true;
	} else if (sysfs_streq(buf, "ramp_down")) {
		is_increasing = false;
	} else {
		dev_err(&ltc3220_state->client->dev,
			"Invalid gradation mode, must be 'ramp_up' or 'ramp_down'\n");
		return -EINVAL;
	}

	ret = ltc3220_set_blink_and_gradation(ltc3220_state,
					       ltc3220_state->blink_mode,
					       ltc3220_state->grad_cfg.gradation_period_ms,
					       is_increasing);
	if (ret < 0)
		return ret;

	ltc3220_state->grad_cfg.is_increasing = is_increasing;

	return size;
}
static DEVICE_ATTR(gradation_mode, 0644,
		   ltc3220_gradation_mode_show,
		   ltc3220_gradation_mode_store);

static ssize_t ltc3220_gradation_mode_available_show(struct device *dev,
						      struct device_attribute *attr,
						      char *buf)
{
	return sysfs_emit(buf, "ramp_up ramp_down\n");
}
static DEVICE_ATTR(gradation_mode_available, 0444,
		   ltc3220_gradation_mode_available_show,
		   NULL);

static struct attribute *device_cfg_attrs[] = {
	&dev_attr_gradation_period_ms.attr,
	&dev_attr_gradation_mode.attr,
	&dev_attr_gradation_mode_available.attr,
	NULL
};

static const struct attribute_group device_cfg_group = {
	.attrs = device_cfg_attrs,
};

static void ltc3220_reset_gpio_action(void *data)
{
	struct gpio_desc *reset_gpio = data;

	gpiod_set_value_cansleep(reset_gpio, 0);
}

static int ltc3220_reset(struct ltc3220_state *ltc3220_state, struct i2c_client *client)
{
	int i;
	int ret;
	struct gpio_desc *reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);

	if (IS_ERR(reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(reset_gpio),
							"Failed to set reset GPIO\n");

	if (reset_gpio) {
		gpiod_set_value_cansleep(reset_gpio, 0);
		fsleep(20);
		gpiod_set_value_cansleep(reset_gpio, 1);

		ret = devm_add_action_or_reset(&client->dev, ltc3220_reset_gpio_action, reset_gpio);
		if (ret)
			return ret;

	} else {
		ret = ltc3220_set_command(ltc3220_state, 0);
		if (ret < 0)
			return ret;

		for (i = 0; i < LTC3220_NUM_LEDS; i++) {
			ret = i2c_smbus_write_byte_data(client, LTC3220_ULED01 + i, 0);
			if (ret < 0)
				return ret;
		}

		ret = ltc3220_set_blink_and_gradation(ltc3220_state, 0, 0, 0);
		if (ret < 0)
			return ret;

	}

	return 0;
}

static void ltc3220_remove(struct i2c_client *client)
{
	struct ltc3220_state *ltc3220_state = i2c_get_clientdata(client);

	ltc3220_set_command(ltc3220_state, true);
}

static int ltc3220_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3220_state *ltc3220_state = i2c_get_clientdata(client);

	return ltc3220_set_command(ltc3220_state, true);
}

static int ltc3220_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3220_state *ltc3220_state = i2c_get_clientdata(client);

	return ltc3220_set_command(ltc3220_state, false);
}

static DEFINE_SIMPLE_DEV_PM_OPS(ltc3220_pm_ops, ltc3220_suspend, ltc3220_resume);

static int ltc3220_probe(struct i2c_client *client)
{
	u8 i = 0;
	int ret;
	struct ltc3220_state *ltc3220_state;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "SMBUS Byte Data not Supported\n");
		return -EIO;
	}

	ltc3220_state = devm_kzalloc(&client->dev, sizeof(*ltc3220_state), GFP_KERNEL);
	if (!ltc3220_state)
		return -ENOMEM;

	ltc3220_state->client = client;
	i2c_set_clientdata(client, ltc3220_state);

	ret = ltc3220_reset(ltc3220_state, client);
	if (ret)
		return ret;

	ret = device_property_read_u8(&client->dev, "adi,force-cpo-mode",
				      &ltc3220_state->command_cfg.force_cpo_mode);
	if (ret)
		ltc3220_state->command_cfg.force_cpo_mode = 0;

	if (ltc3220_state->command_cfg.force_cpo_mode > 3) {
		dev_err(&client->dev,
		    "Invalid adi,force-cpo-mode %u, must be 0-3\n",
		    ltc3220_state->command_cfg.force_cpo_mode);
		return -EINVAL;
	}

	ltc3220_state->command_cfg.is_quick_write =
		device_property_read_bool(&client->dev, "adi,quick-write-enable");

	ret = ltc3220_set_command(ltc3220_state, 0);
	if (ret < 0)
		return ret;

	device_for_each_child_node_scoped(&client->dev, child) {
		u32 source;
		struct led_init_data init_data = {};
		char label_buffer[32];

		ret = fwnode_property_read_u32(child, "reg", &source);
		if (ret != 0 || !source || source > LTC3220_NUM_LEDS) {
			dev_err(&client->dev, "Couldn't read LED address: %d\n",
				ret);
			return -EINVAL;
		}

		/* LED node reg/index/address goes from 1 to 18 */
		i = source - 1;
		init_data.fwnode = child;
		init_data.devicename = "ltc3220";
		snprintf(label_buffer, sizeof(label_buffer), ":led%d", i);
		init_data.default_label = label_buffer;

		ltc3220_state->uled_cfg[i].led_index = i;
		ltc3220_state->uled_cfg[i].reg_value = 0;
		ltc3220_state->uled_cfg[i].led_cdev.brightness_set_blocking = ltc3220_set_led_data;
		ltc3220_state->uled_cfg[i].led_cdev.brightness_get = ltc3220_get_led_data;
		ltc3220_state->uled_cfg[i].led_cdev.blink_set = ltc3220_blink_set;

		ret = devm_led_classdev_register_ext(&client->dev,
							   &ltc3220_state->uled_cfg[i].led_cdev,
							   &init_data);
		if (ret) {
			dev_err(&client->dev, "Fail LED class register\n");
			return ret;
		}
	}

	ret = devm_device_add_group(&client->dev, &device_cfg_group);
	if (ret) {
		dev_err(&client->dev, "Failed to create sysfs group\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id ltc3220_of_match[] = {
	{ .compatible = "adi,ltc3220", },
	{ .compatible = "adi,ltc3220-1", },
	{ },
};
MODULE_DEVICE_TABLE(of, ltc3220_of_match);

static struct i2c_driver ltc3220_led_driver = {
	.driver		= {
		.name	= "ltc3220",
		.of_match_table = ltc3220_of_match,
		.pm	= pm_sleep_ptr(&ltc3220_pm_ops),
	},
	.probe		= ltc3220_probe,
	.remove		= ltc3220_remove,
};

module_i2c_driver(ltc3220_led_driver);

MODULE_AUTHOR("Edelweise Escala <edelweise.escala@analog.com>");
MODULE_DESCRIPTION("LED driver for LTC3220 controllers");
MODULE_LICENSE("GPL");
