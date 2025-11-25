// SPDX-License-Identifier: GPL-2.0
/*
 * LED driver for Analog Devices LTC3208 Multi-Display Driver
 *
 * Copyright 2025 Analog Devices Inc.
 *
 * Author: Jan Carlo Roleda <jancarlo.roleda@analog.com>
 */
#include <linux/bitfield.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#define LTC3208_DEVICE_NAME "ltc3208"

#define LTC3208_SET_HIGH_BYTE_DATA(x)	FIELD_PREP(GENMASK(7, 4), (x))

/* Registers */
#define LTC3208_REG_A_GNRD	0x1 /* Green (High half-byte) and Red (Low half-byte) current DAC*/
#define LTC3208_REG_B_AXBL	0x2 /* AUX (High half-byte) and Blue (Low half-byte) current DAC*/
#define LTC3208_REG_C_MAIN	0x3 /* Main current DAC */
#define LTC3208_REG_D_SUB	0x4 /* Sub current DAC */
#define LTC3208_REG_E_AUX	0x5 /* AUX DAC Select */
#define LTC3208_REG_F_CAM	0x6 /* CAM (High half-byte and Low half-byte) current DAC*/
#define LTC3208_REG_G_OPT	0x7 /* Device Options */

/* Device Options register */
#define LTC3208_OPT_CPO_MASK	GENMASK(7, 6)
#define LTC3208_OPT_DIS_RGBDROP	BIT(3)
#define LTC3208_OPT_DIS_CAMHILO	BIT(2)
#define LTC3208_OPT_EN_RGBS		BIT(1)

/* Auxiliary DAC select masks */
#define LTC3208_AUX1_MASK	GENMASK(1, 0)
#define LTC3208_AUX2_MASK	GENMASK(3, 2)
#define LTC3208_AUX3_MASK	GENMASK(5, 4)
#define LTC3208_AUX4_MASK	GENMASK(7, 6)

#define LTC3208_MAX_BRIGHTNESS_4BIT 0xF
#define LTC3208_MAX_BRIGHTNESS_8BIT 0xFF

#define LTC3208_NUM_LED_GRPS	8
#define LTC3208_NUM_AUX_LEDS	3

enum ltc3208_aux_channel {
	LTC3208_AUX_CHAN_AUX = 0,
	LTC3208_AUX_CHAN_MAIN,
	LTC3208_AUX_CHAN_SUB,
	LTC3208_AUX_CHAN_CAM
};

enum ltc3208_channel {
	LTC3208_CHAN_MAIN = 0,
	LTC3208_CHAN_SUB,
	LTC3208_CHAN_AUX,
	LTC3208_CHAN_CAML,
	LTC3208_CHAN_CAMH,
	LTC3208_CHAN_RED,
	LTC3208_CHAN_BLUE,
	LTC3208_CHAN_GREEN
};

enum ltc3208_cpo_mode {
	LTC3208_CPO_AUTO = 0,
	LTC3208_CPO_1P5X,
	LTC3208_CPO_2X
};

static const char * const ltc3208_channel_labels[] = {
	"main", "sub", "aux", "cam_lo", "cam_hi", "red", "blue", "green"
};

static const char * const ltc3208_dt_aux_channels[] = {
	"adi,aux1-channel", "adi,aux2-channel",
	"adi,aux3-channel", "adi,aux4-channel"
};

struct ltc3208_led {
	struct led_classdev cdev;
	struct i2c_client *client;
	enum ltc3208_channel channel;
};

/*
 * as device is write-only the device configs must be stored in the driver to
 * allow atomic operations in registers that control multiple channels/options
 */
struct ltc3208_chip_data {
	struct ltc3208_led *leds;
};

struct ltc3208_dev {
	struct i2c_client *client;
	struct regmap *map;
	struct ltc3208_chip_data chip_data;
};

static const struct regmap_config ltc3208_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ltc3208_led_set_brightness(struct led_classdev *led_cdev,
	enum led_brightness brightness)
{
	struct ltc3208_led *led = container_of(led_cdev, struct ltc3208_led, cdev);
	struct i2c_client *client = led->client;
	struct ltc3208_dev *dev = i2c_get_clientdata(client);
	struct regmap *map = dev->map;
	int ret;
	u8 current_level = (u8)brightness;
	u32 reg = 0;

	if (current_level > led->cdev.max_brightness) {
		dev_err(&client->dev, "Brightness set is too High\n");
		return -EINVAL;
	}

	/*
	 * For registers with 4-bit splits (CAM, AUX/BLUE, GREEN/RED), the other
	 * half of the byte will be retrieved from the stored DAC value before
	 * updating the register.
	 */
	switch (led->channel) {
	case LTC3208_CHAN_MAIN:
		reg = LTC3208_REG_C_MAIN;
		break;
	case LTC3208_CHAN_SUB:
		reg = LTC3208_REG_D_SUB;
		break;

	case LTC3208_CHAN_AUX:
		/* combine both low and high halves of byte */
		current_level = LTC3208_SET_HIGH_BYTE_DATA(current_level);
		current_level |= dev->chip_data.leds[LTC3208_CHAN_BLUE].cdev.brightness;
		reg = LTC3208_REG_B_AXBL;
		break;

	case LTC3208_CHAN_BLUE:
		/* apply high bits stored in other led */
		current_level |= LTC3208_SET_HIGH_BYTE_DATA(
			dev->chip_data.leds[LTC3208_CHAN_AUX].cdev.brightness);
		reg = LTC3208_REG_B_AXBL;
		break;

	case LTC3208_CHAN_CAMH:
		current_level = LTC3208_SET_HIGH_BYTE_DATA(current_level);
		current_level |= dev->chip_data.leds[LTC3208_CHAN_CAML].cdev.brightness;
		reg = LTC3208_REG_F_CAM;
		break;

	case LTC3208_CHAN_CAML:
		current_level |= LTC3208_SET_HIGH_BYTE_DATA(
			dev->chip_data.leds[LTC3208_CHAN_CAMH].cdev.brightness);
		reg = LTC3208_REG_F_CAM;
		break;

	case LTC3208_CHAN_GREEN:
		current_level = LTC3208_SET_HIGH_BYTE_DATA(current_level);
		current_level |= dev->chip_data.leds[LTC3208_CHAN_RED].cdev.brightness;
		reg = LTC3208_REG_A_GNRD;
		break;
	case LTC3208_CHAN_RED:
		current_level |= LTC3208_SET_HIGH_BYTE_DATA(
			dev->chip_data.leds[LTC3208_CHAN_GREEN].cdev.brightness);
		reg = LTC3208_REG_A_GNRD;
		break;
	}

	ret = regmap_write(map, reg, current_level);
	if (ret) {
		dev_err(&client->dev, "Error Writing brightness to register %u\n", reg);
		return ret;
	}

	return 0;
}

static int ltc3208_update_options(struct ltc3208_dev *dev,
	bool is_sub, bool is_cam_hi, bool is_rgb_drop,
	enum ltc3208_cpo_mode cpo_mode)
{
	struct regmap *map = dev->map;
	int ret;
	u8 val = 0;

	val |= is_sub ? LTC3208_OPT_EN_RGBS : 0;
	val |= is_cam_hi ? LTC3208_OPT_DIS_CAMHILO : 0;
	val |= is_rgb_drop ? LTC3208_OPT_DIS_RGBDROP : 0;
	val |= FIELD_PREP(LTC3208_OPT_CPO_MASK, cpo_mode);

	ret = regmap_write(map, LTC3208_REG_G_OPT, val);
	if (ret) {
		dev_err(&dev->client->dev,
			"Error writing options to device: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ltc3208_update_aux_dac(struct ltc3208_dev *dev,
	enum ltc3208_aux_channel aux_1, enum ltc3208_aux_channel aux_2,
	enum ltc3208_aux_channel aux_3, enum ltc3208_aux_channel aux_4)
{
	struct regmap *map = dev->map;
	int ret;
	u8 val = 0;

	val = FIELD_PREP(LTC3208_AUX1_MASK, aux_1) |
		  FIELD_PREP(LTC3208_AUX2_MASK, aux_2) |
		  FIELD_PREP(LTC3208_AUX3_MASK, aux_3) |
		  FIELD_PREP(LTC3208_AUX4_MASK, aux_4);

	ret = regmap_write(map, LTC3208_REG_E_AUX, val);
	if (ret) {
		dev_err(&dev->client->dev,
			"Error writing AUX DAC selection to device: %d\n", ret);
		return ret;
	}

	return 0;
}

static int ltc3208_probe(struct i2c_client *client)
{
	struct ltc3208_dev *data;
	struct regmap *map;
	struct ltc3208_led *leds;
	enum ltc3208_aux_channel aux_channels[LTC3208_NUM_AUX_LEDS];
	enum ltc3208_cpo_mode cpo_mode;
	int ret, i;
	u32 val;
	const char *str_val;
	bool en_rgbs, dis_camhl, dropdis_rgb_aux4;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return dev_err_probe(&client->dev, -EIO,
			"SMBUS Byte Data not Supported\n");

	map = devm_regmap_init_i2c(client, &ltc3208_regmap_cfg);
	if (IS_ERR(map))
		return dev_err_probe(&client->dev, PTR_ERR(map),
		"Failed to initialize regmap\n");

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return dev_err_probe(&client->dev, -ENOMEM,
			"No memory for device data\n");

	leds = devm_kcalloc(&client->dev,
					   LTC3208_NUM_LED_GRPS,
					   sizeof(struct ltc3208_led),
					   GFP_KERNEL);

	data->client = client;
	data->map = map;

	/* initialize options from devicetree */
	dis_camhl = device_property_read_bool(&client->dev,
		"adi,disable-camhl-pin");
	en_rgbs = device_property_read_bool(&client->dev, "adi,select-rgb-sub-en");
	dropdis_rgb_aux4 = device_property_read_bool(&client->dev,
		"adi,disable-rgb-aux4-dropout");

	cpo_mode = LTC3208_CPO_AUTO;
	if (!device_property_read_u32(&client->dev, "adi,force-cpo-level", &val)) {
		if (val > 2)
			return dev_err_probe(&client->dev, -EINVAL,
				"invalid adi,force-cpo-level value: %u\n", val);

		cpo_mode = (enum ltc3208_cpo_mode)val;
	}

	ret = ltc3208_update_options(data, en_rgbs, dis_camhl, dropdis_rgb_aux4,
		cpo_mode);
	if (ret)
		return dev_err_probe(&client->dev, ret,
			"error writing to options register: %d\n", ret);

	/* initialize aux channel configurations from devicetree */
	for (i = 0; i < LTC3208_NUM_AUX_LEDS; i++) {
		ret = device_property_match_string(&client->dev,
										  ltc3208_dt_aux_channels[i],
										  str_val);
			if (ret == -ENXIO) {
				/* use default value if unspecified on devicetree */
				aux_channels[i] = LTC3208_AUX_CHAN_AUX;
				continue;
			} else {
				return dev_err_probe(&client->dev, -EINVAL,
					"invalid channel name: %s\n", str_val);
			}

			if (strcmp(str_val, "aux") == 0)
				aux_channels[i] = LTC3208_AUX_CHAN_AUX;
			else if (strcmp(str_val, "main") == 0)
				aux_channels[i] = LTC3208_AUX_CHAN_MAIN;
			else if (strcmp(str_val, "sub") == 0)
				aux_channels[i] = LTC3208_AUX_CHAN_SUB;
			else if (strcmp(str_val, "cam") == 0)
				aux_channels[i] = LTC3208_AUX_CHAN_CAM;
			else
				return dev_err_probe(&client->dev, ret,
					"invalid channel name: %s\n", str_val);
	}

	ret = ltc3208_update_aux_dac(data, aux_channels[0], aux_channels[1],
								 aux_channels[2], aux_channels[3]);
	if (ret)
		return dev_err_probe(&client->dev, ret,
			"error writing to aux %u channel register: %d\n", i, ret);

	i2c_set_clientdata(client, data);

	device_for_each_child_node_scoped(&client->dev, child) {
		struct ltc3208_led *led;
		struct led_init_data init_data = {};

		ret = fwnode_property_read_u32(child, "reg", &val);
		if (ret || val >= LTC3208_NUM_LED_GRPS)
			return dev_err_probe(&client->dev, -EINVAL,
				"Invalid reg property for LED\n");

		led = &leds[val];
		led->client = client;
		led->channel = val;
		led->cdev.brightness_set_blocking = ltc3208_led_set_brightness;
		led->cdev.max_brightness = LTC3208_MAX_BRIGHTNESS_4BIT;
		if (val == LTC3208_CHAN_MAIN || val == LTC3208_CHAN_SUB)
			led->cdev.max_brightness = LTC3208_MAX_BRIGHTNESS_8BIT;

		led->cdev.name = devm_kasprintf(&client->dev, GFP_KERNEL, "ltc3208:%s",
			ltc3208_channel_labels[val]);
		if (!led->cdev.name)
			return dev_err_probe(&client->dev, -ENOMEM,
				"No memory for LED name");

		init_data.fwnode = child;

		ret = devm_led_classdev_register_ext(&client->dev, &led->cdev,
			&init_data);
		if (ret)
			return dev_err_probe(&client->dev, ret,
				"Failed to register LED %u\n", val);
	}

	data->chip_data.leds = leds;

	return 0;
}

static const struct of_device_id ltc3208_match_table[] = {
	{.compatible = "adi,ltc3208"},
	{ }
};
MODULE_DEVICE_TABLE(of, ltc3208_match_table);

static const struct i2c_device_id ltc3208_idtable[] = {
	{ LTC3208_DEVICE_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc3208_idtable);

static struct i2c_driver ltc3208_driver = {
	.driver = {
		.name = LTC3208_DEVICE_NAME,
		.of_match_table = ltc3208_match_table,
	},
	.id_table = ltc3208_idtable,
	.probe = ltc3208_probe,
};
module_i2c_driver(ltc3208_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jan Carlo Roleda <jancarlo.roleda@analog.com>");
MODULE_DESCRIPTION("LTC3208 LED Driver");
