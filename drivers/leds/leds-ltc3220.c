// SPDX-License-Identifier: GPL-2.0
/*
 * LTC3220 18-Channel LED Driver
 *
 * Copyright 2026 Analog Devices Inc.
 *
 * Author: Edelweise Escala <edelweise.escala@analog.com>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/types.h>

/* LTC3220 Registers */
#define LTC3220_COMMAND				0x00
#define LTC3220_ULED(x)				(0x01 + (x))
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

static const char * const ltc3220_cpo_levels[] = { "0", "1.5", "2", "1" };

struct ltc3220_command_cfg {
	bool quick_write;
	bool is_shutdown;
	u8 force_cpo_level;
};

struct ltc3220_uled_cfg {
	struct ltc3220_state *ltc3220_state;
	struct led_classdev led_cdev;
	u8 reg_value;
	u8 led_index;
};

struct ltc3220_grad_cfg {
	bool is_increasing;
	u8 gradation_period_ms;
};

struct ltc3220_state {
	struct ltc3220_command_cfg command_cfg;
	struct ltc3220_uled_cfg uled_cfg[LTC3220_NUM_LEDS];
	struct ltc3220_grad_cfg grad_cfg;
	struct i2c_client *client;
	u8 blink_mode;
};

static int ltc3220_set_command(struct ltc3220_state *ltc3220_state)
{
	struct i2c_client *client = ltc3220_state->client;
	u8 reg_val;

	reg_val = FIELD_PREP(LTC3220_COMMAND_SHUTDOWN, ltc3220_state->command_cfg.is_shutdown);
	reg_val |= FIELD_PREP(LTC3220_CPO_COMMAND_MASK,
				 ltc3220_state->command_cfg.force_cpo_level);
	reg_val |= FIELD_PREP(LTC3220_COMMAND_QUICK_WRITE,
				 ltc3220_state->command_cfg.quick_write);

	return i2c_smbus_write_byte_data(client, LTC3220_COMMAND, reg_val);
}

static int ltc3220_shutdown(struct ltc3220_state *ltc3220_state)
{
	struct i2c_client *client = ltc3220_state->client;
	u8 reg_val;
	int ret;

	reg_val = FIELD_PREP(LTC3220_COMMAND_SHUTDOWN, 1);
	reg_val |= FIELD_PREP(LTC3220_CPO_COMMAND_MASK,
				 ltc3220_state->command_cfg.force_cpo_level);

	ret = i2c_smbus_write_byte_data(client, LTC3220_COMMAND, reg_val);
	if (ret == 0)
		ltc3220_state->command_cfg.is_shutdown = true;

	return ret;
}

static int ltc3220_resume_from_shutdown(struct ltc3220_state *ltc3220_state)
{
	int ret;

	ltc3220_state->command_cfg.is_shutdown = false;
	ret = ltc3220_set_command(ltc3220_state);
	if (ret < 0)
		ltc3220_state->command_cfg.is_shutdown = true;

	return ret;
}

/*
 * Set LED brightness and mode.
 * The brightness value determines both the LED current and operating mode:
 * 0-63:    Normal mode - LED current from 0-63 (off to full brightness)
 * 64-127:  Blink mode - LED blinks with current level (brightness - 64)
 * 128-191: Gradation mode - LED gradually changes brightness (brightness - 128)
 * 192-255: GPO mode - LED operates as general purpose output (brightness - 192)
 */
static int ltc3220_set_led_data(struct led_classdev *led_cdev,
				enum led_brightness brightness)
{
	struct ltc3220_state *ltc3220_state;
	struct ltc3220_uled_cfg *uled_cfg;
	int ret;
	int i;

	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);
	ltc3220_state = uled_cfg->ltc3220_state;

	ret = i2c_smbus_write_byte_data(ltc3220_state->client,
			LTC3220_ULED(uled_cfg->led_index), brightness);
	if (ret < 0)
		return ret;

	uled_cfg->reg_value = brightness;

	/*
	 * When quick-write is enabled, writing to LED 1 updates all
	 * LEDs simultaneously via quick-write mode. Update cached values for
	 * all LEDs to reflect the synchronized state.
	 */
	if (ltc3220_state->command_cfg.quick_write && uled_cfg->led_index == 0) {
		for (i = 0; i < LTC3220_NUM_LEDS; i++)
			ltc3220_state->uled_cfg[i].reg_value = brightness;
	}

	return 0;
}

static enum led_brightness ltc3220_get_led_data(struct led_classdev *led_cdev)
{
	struct ltc3220_uled_cfg *uled_cfg;

	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);

	return uled_cfg->reg_value;
}

static int ltc3220_set_blink_and_gradation(struct ltc3220_state *ltc3220_state,
		    u8 blink_cfg, u8 gradation_period_ms, bool is_increasing)
{
	struct i2c_client *client = ltc3220_state->client;
	u8 reg_val;

	reg_val = FIELD_PREP(LTC3220_BLINK_MASK, blink_cfg);
	reg_val |= FIELD_PREP(LTC3220_GRADATION_MASK, gradation_period_ms);
	reg_val |= FIELD_PREP(LTC3220_GRAD_COUNT_UP, is_increasing);

	return i2c_smbus_write_byte_data(client, LTC3220_GRAD_BLINK, reg_val);
}

/*
 * LTC3220 pattern support for hardware-assisted breathing/gradation.
 * The hardware supports 3 gradation ramp time 240ms, 480ms, 960ms)
 * and can ramp up or down.
 *
 * Pattern array interpretation:
 *   pattern[0].brightness = start brightness (0-63)
 *   pattern[0].delta_t = ramp time in milliseconds
 *   pattern[1].brightness = end brightness (0-63)
 *   pattern[1].delta_t = (optional, can be 0 or same as pattern[0].delta_t)
 */
static int ltc3220_pattern_set(struct led_classdev *led_cdev,
			       struct led_pattern *pattern,
			       u32 len, int repeat)
{
	struct ltc3220_state *ltc3220_state;
	struct ltc3220_uled_cfg *uled_cfg;
	u8 gradation_period;
	u8 start_brightness;
	u8 end_brightness;
	bool is_increasing;
	int ret;

	if (len != 2)
		return -EINVAL;

	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);
	ltc3220_state = uled_cfg->ltc3220_state;

	start_brightness = pattern[0].brightness & LTC3220_LED_CURRENT_MASK;
	end_brightness = pattern[1].brightness & LTC3220_LED_CURRENT_MASK;

	is_increasing = end_brightness > start_brightness;

	if (pattern[0].delta_t == 0)
		gradation_period = 0;
	else if (pattern[0].delta_t <= 240)
		gradation_period = 1;
	else if (pattern[0].delta_t <= 480)
		gradation_period = 2;
	else
		gradation_period = 3;

	ret = ltc3220_set_blink_and_gradation(ltc3220_state,
					       ltc3220_state->blink_mode,
					       gradation_period,
					       is_increasing);
	if (ret < 0)
		return ret;

	ltc3220_state->grad_cfg.gradation_period_ms = gradation_period;
	ltc3220_state->grad_cfg.is_increasing = is_increasing;

	ret = ltc3220_set_led_data(led_cdev, start_brightness);
	if (ret < 0)
		return ret;

	return ltc3220_set_led_data(led_cdev, 128 + end_brightness);
}

static int ltc3220_pattern_clear(struct led_classdev *led_cdev)
{
	struct ltc3220_state *ltc3220_state;
	struct ltc3220_uled_cfg *uled_cfg;
	int ret;

	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);
	ltc3220_state = uled_cfg->ltc3220_state;

	ret = ltc3220_set_blink_and_gradation(ltc3220_state,
					       ltc3220_state->blink_mode,
					       0, false);
	if (ret < 0)
		return ret;

	ltc3220_state->grad_cfg.gradation_period_ms = 0;
	ltc3220_state->grad_cfg.is_increasing = false;

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
	struct ltc3220_uled_cfg *uled_cfg;
	unsigned long period;
	u8 blink_mode;
	int ret;

	uled_cfg = container_of(led_cdev, struct ltc3220_uled_cfg, led_cdev);
	ltc3220_state = uled_cfg->ltc3220_state;

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

	ret = ltc3220_set_blink_and_gradation(ltc3220_state, blink_mode,
					       ltc3220_state->grad_cfg.gradation_period_ms,
					       ltc3220_state->grad_cfg.is_increasing);
	if (ret < 0)
		return ret;

	ltc3220_state->blink_mode = blink_mode;

	return 0;
}

static void ltc3220_reset_gpio_action(void *data)
{
	struct gpio_desc *reset_gpio = data;

	gpiod_set_value_cansleep(reset_gpio, 1);
}

static int ltc3220_reset(struct ltc3220_state *ltc3220_state, struct i2c_client *client)
{
	struct gpio_desc *reset_gpio;
	int ret;
	int i;

	reset_gpio = devm_gpiod_get_optional(&client->dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(reset_gpio),
							"Failed to set reset GPIO\n");

	if (reset_gpio) {
		gpiod_set_value_cansleep(reset_gpio, 0);

		ret = devm_add_action_or_reset(&client->dev, ltc3220_reset_gpio_action, reset_gpio);
		if (ret)
			return ret;

	} else {
		ret = ltc3220_set_command(ltc3220_state);
		if (ret < 0)
			return ret;

		for (i = 0; i < LTC3220_NUM_LEDS; i++) {
			ret = i2c_smbus_write_byte_data(client, LTC3220_ULED(i), 0);
			if (ret < 0)
				return ret;
		}

		ret = ltc3220_set_blink_and_gradation(ltc3220_state, 0, 0, 0);
		if (ret < 0)
			return ret;
	}

	return 0;
}

static int ltc3220_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3220_state *ltc3220_state = i2c_get_clientdata(client);

	return ltc3220_shutdown(ltc3220_state);
}

static int ltc3220_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct ltc3220_state *ltc3220_state = i2c_get_clientdata(client);

	return ltc3220_resume_from_shutdown(ltc3220_state);
}

static SIMPLE_DEV_PM_OPS(ltc3220_pm_ops, ltc3220_suspend, ltc3220_resume);

static int ltc3220_probe(struct i2c_client *client)
{
	struct ltc3220_state *ltc3220_state;
	u8 i = 0;
	int ret;

	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(&client->dev, -EIO,
				     "SMBUS Byte Data not Supported\n");

	ltc3220_state = devm_kzalloc(&client->dev, sizeof(*ltc3220_state), GFP_KERNEL);
	if (!ltc3220_state)
		return -ENOMEM;

	ltc3220_state->client = client;
	i2c_set_clientdata(client, ltc3220_state);

	if (device_property_read_bool(&client->dev, "adi,quick-write"))
		ltc3220_state->command_cfg.quick_write = true;

	ret = ltc3220_reset(ltc3220_state, client);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "Failed to reset device\n");

	ret = device_property_match_property_string(&client->dev, "adi,force-cpo-level",
					   ltc3220_cpo_levels, ARRAY_SIZE(ltc3220_cpo_levels));
	if (ret >= 0)
		ltc3220_state->command_cfg.force_cpo_level = ret;

	ret = ltc3220_set_command(ltc3220_state);
	if (ret < 0)
		return dev_err_probe(&client->dev, ret,
				     "Failed to set command\n");

	device_for_each_child_node_scoped(&client->dev, child) {
		struct led_init_data init_data = {};
		struct ltc3220_uled_cfg *led;
		u32 source;

		ret = fwnode_property_read_u32(child, "reg", &source);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "Couldn't read LED address\n");

		if (!source || source > LTC3220_NUM_LEDS)
			return dev_err_probe(&client->dev, -EINVAL,
					     "LED address out of range\n");

		init_data.fwnode = child;
		init_data.devicename = "ltc3220";
		init_data.devname_mandatory = true;

		/* LED node reg/index/address goes from 1 to 18 */
		i = source - 1;
		led = &ltc3220_state->uled_cfg[i];
		led->led_index = i;
		led->reg_value = 0;
		led->ltc3220_state = ltc3220_state;
		led->led_cdev.brightness_set_blocking = ltc3220_set_led_data;
		led->led_cdev.brightness_get = ltc3220_get_led_data;
		led->led_cdev.max_brightness = 255;
		led->led_cdev.blink_set = ltc3220_blink_set;
		led->led_cdev.pattern_set = ltc3220_pattern_set;
		led->led_cdev.pattern_clear = ltc3220_pattern_clear;

		ret = devm_led_classdev_register_ext(&client->dev,
						      &led->led_cdev,
						      &init_data);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "Failed to register LED class device\n");
	}

	return 0;
}

static const struct of_device_id ltc3220_of_match[] = {
	{ .compatible = "adi,ltc3220", },
	{ .compatible = "adi,ltc3220-1", },
	{ }
};
MODULE_DEVICE_TABLE(of, ltc3220_of_match);

static struct i2c_driver ltc3220_led_driver = {
	.driver = {
		.name = "ltc3220",
		.of_match_table = ltc3220_of_match,
		.pm	= pm_sleep_ptr(&ltc3220_pm_ops),
	},
	.probe = ltc3220_probe,
};
module_i2c_driver(ltc3220_led_driver);

MODULE_AUTHOR("Edelweise Escala <edelweise.escala@analog.com>");
MODULE_DESCRIPTION("LED driver for LTC3220 controllers");
MODULE_LICENSE("GPL");
