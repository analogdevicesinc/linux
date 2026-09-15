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
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/leds.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/types.h>
#include <linux/workqueue.h>

#define LTC3208_SET_HIGH_BYTE_DATA(x)	FIELD_PREP(GENMASK(7, 4), (x))

/* Registers */
#define LTC3208_REG_A_GRNRED	0x1 /* Green (High half-byte) and Red (Low half-byte) current DAC*/
#define LTC3208_REG_B_AUXBLU	0x2 /* AUX (High half-byte) and Blue (Low half-byte) current DAC*/
#define LTC3208_REG_C_MAIN	0x3 /* Main current DAC */
#define LTC3208_REG_D_SUB	0x4 /* Sub current DAC */
#define LTC3208_REG_E_AUX	0x5 /* AUX DAC Select */
#define LTC3208_REG_F_CAM	0x6 /* CAM (High half-byte and Low half-byte) current DAC*/
#define LTC3208_REG_G_OPT	0x7 /* Device Options */

/* Device Options register */
#define LTC3208_OPT_CPO_MASK	GENMASK(7, 6)
#define LTC3208_OPT_DIS_RGBDROP	BIT(3)
#define LTC3208_OPT_DIS_CAMHILO	BIT(2)
#define LTC3208_OPT_EN_RGBS	BIT(1)

/* Auxiliary DAC select masks */
#define LTC3208_AUX1_MASK	GENMASK(1, 0)
#define LTC3208_AUX2_MASK	GENMASK(3, 2)
#define LTC3208_AUX3_MASK	GENMASK(5, 4)
#define LTC3208_AUX4_MASK	GENMASK(7, 6)

#define LTC3208_MAX_BRIGHTNESS_4BIT 0xF
#define LTC3208_MAX_BRIGHTNESS_8BIT 0xFF

#define LTC3208_NUM_LED_GRPS	8
#define LTC3208_NUM_AUX_LEDS	4

#define LTC3208_NUM_AUX_OPT	4
#define LTC3208_MAX_CPO_OPT	3

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

static const char * const ltc3208_aux_opt[] = {
	"aux", "main", "sub", "cam"
};

static const char * const ltc3208_cpo_opt[] = {
	"1", "1.5", "2"
};

struct ltc3208_led {
	struct led_classdev cdev;
	struct i2c_client *client;
	enum ltc3208_channel channel;
};

struct ltc3208_dev {
	struct i2c_client *client;
	struct regmap *map;
	struct ltc3208_led *leds;
};

static const struct regmap_config ltc3208_regmap_cfg = {
	.reg_bits = 8,
	.val_bits = 8,
};

static int ltc3208_led_set_brightness(struct led_classdev *led_cdev,
				      enum led_brightness brightness)
{
	struct ltc3208_led *led = container_of(led_cdev,
					struct ltc3208_led, cdev);
	struct i2c_client *client = led->client;
	struct ltc3208_dev *dev = i2c_get_clientdata(client);
	struct regmap *map = dev->map;
	u8 current_level = brightness;

	/*
	 * For registers with 4-bit splits (CAM, AUX/BLUE, GREEN/RED), the other
	 * half of the byte will be retrieved from the stored DAC value before
	 * updating the register.
	 */
	switch (led->channel) {
	case LTC3208_CHAN_MAIN:
		return regmap_write(map, LTC3208_REG_C_MAIN, current_level);
	case LTC3208_CHAN_SUB:
		return regmap_write(map, LTC3208_REG_D_SUB, current_level);
	case LTC3208_CHAN_AUX:
		/* combine both low and high halves of byte */
		current_level = LTC3208_SET_HIGH_BYTE_DATA(current_level);
		current_level |= dev->leds[LTC3208_CHAN_BLUE].cdev.brightness;
		return regmap_write(map, LTC3208_REG_B_AUXBLU, current_level);
	case LTC3208_CHAN_BLUE:
		/* apply high bits stored in other led */
		current_level |= LTC3208_SET_HIGH_BYTE_DATA(
			dev->leds[LTC3208_CHAN_AUX].cdev.brightness);
		return regmap_write(map, LTC3208_REG_B_AUXBLU, current_level);
	case LTC3208_CHAN_CAMH:
		current_level = LTC3208_SET_HIGH_BYTE_DATA(current_level);
		current_level |= dev->leds[LTC3208_CHAN_CAML].cdev.brightness;
		return regmap_write(map, LTC3208_REG_F_CAM, current_level);
	case LTC3208_CHAN_CAML:
		current_level |= LTC3208_SET_HIGH_BYTE_DATA(
			dev->leds[LTC3208_CHAN_CAMH].cdev.brightness);
		return regmap_write(map, LTC3208_REG_F_CAM, current_level);
	case LTC3208_CHAN_GREEN:
		current_level = LTC3208_SET_HIGH_BYTE_DATA(current_level);
		current_level |= dev->leds[LTC3208_CHAN_RED].cdev.brightness;
		return regmap_write(map, LTC3208_REG_A_GRNRED, current_level);
	case LTC3208_CHAN_RED:
		current_level |= LTC3208_SET_HIGH_BYTE_DATA(
			dev->leds[LTC3208_CHAN_GREEN].cdev.brightness);
		return regmap_write(map, LTC3208_REG_A_GRNRED, current_level);
	default:
		dev_err(&client->dev, "Invalid LED Channel\n");
		return -EINVAL;
	}
}

static int ltc3208_update_options(struct ltc3208_dev *dev,
				  bool is_sub, bool is_cam_hi, bool is_rgb_drop,
				  enum ltc3208_cpo_mode cpo_mode)
{
	struct regmap *map = dev->map;
	u8 val =	FIELD_PREP(LTC3208_OPT_EN_RGBS, is_sub) |
			FIELD_PREP(LTC3208_OPT_DIS_CAMHILO, is_cam_hi) |
			FIELD_PREP(LTC3208_OPT_DIS_RGBDROP, is_rgb_drop) |
			FIELD_PREP(LTC3208_OPT_CPO_MASK, cpo_mode);

	return regmap_write(map, LTC3208_REG_G_OPT, val);
}

static int ltc3208_update_aux_dac(struct ltc3208_dev *dev,
	enum ltc3208_aux_channel aux_1, enum ltc3208_aux_channel aux_2,
	enum ltc3208_aux_channel aux_3, enum ltc3208_aux_channel aux_4)
{
	struct regmap *map = dev->map;
	u8 val =	FIELD_PREP(LTC3208_AUX1_MASK, aux_1) |
			FIELD_PREP(LTC3208_AUX2_MASK, aux_2) |
			FIELD_PREP(LTC3208_AUX3_MASK, aux_3) |
			FIELD_PREP(LTC3208_AUX4_MASK, aux_4);

	return regmap_write(map, LTC3208_REG_E_AUX, val);
}

static int ltc3208_probe(struct i2c_client *client)
{
	enum ltc3208_aux_channel aux_channels[LTC3208_NUM_AUX_LEDS];
	enum ltc3208_cpo_mode cpo_mode = LTC3208_CPO_AUTO;
	struct ltc3208_dev *data;
	struct ltc3208_led *leds;
	struct regmap *map;
	int ret, i;
	u32 val;
	bool dropdis_rgb_aux4;
	bool dis_camhl;
	bool en_rgbs;

	map = devm_regmap_init_i2c(client, &ltc3208_regmap_cfg);
	if (IS_ERR(map))
		return dev_err_probe(&client->dev, PTR_ERR(map),
				     "Failed to initialize regmap\n");

	data = devm_kzalloc(&client->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	leds = devm_kcalloc(&client->dev, LTC3208_NUM_LED_GRPS,
			    sizeof(struct ltc3208_led), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	data->client = client;
	data->map = map;

	/* initialize options from devicetree */
	dis_camhl = device_property_read_bool(&client->dev,
					      "adi,disable-camhl-pin");
	en_rgbs = device_property_read_bool(&client->dev,
					    "adi,cfg-enrgbs-pin");
	dropdis_rgb_aux4 = device_property_read_bool(&client->dev,
						     "adi,disable-rgb-aux4-dropout");

	ret = device_property_match_property_string(&client->dev,
						    "adi,force-cpo-level",
						    ltc3208_cpo_opt,
						    LTC3208_MAX_CPO_OPT);
	if (ret < 0)
		return dev_err_probe(&client->dev, ret,
				     "Failed getting force-cpo-level.\n");
	else
		cpo_mode = ret;

	ret = ltc3208_update_options(data, en_rgbs, dis_camhl,
				     dropdis_rgb_aux4, cpo_mode);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "error writing to options register\n");

	/* initialize aux channel configurations from devicetree */
	for (i = 0; i <= LTC3208_NUM_AUX_LEDS; i++) {
		ret = device_property_match_property_string(&client->dev,
							    ltc3208_dt_aux_channels[i],
							    ltc3208_aux_opt,
							    LTC3208_NUM_AUX_OPT);
		/* use default value if absent in devicetree */
		if (ret == -EINVAL)
			aux_channels[i] = LTC3208_AUX_CHAN_AUX;
		else if (ret >= 0)
			aux_channels[i] = ret;
		else
			return dev_err_probe(&client->dev, ret,
					     "Failed getting aux-channel.\n");
	}

	ret = ltc3208_update_aux_dac(data, aux_channels[0], aux_channels[1],
				     aux_channels[2], aux_channels[3]);
	if (ret)
		return dev_err_probe(&client->dev, ret,
				     "error writing to aux %u channel register.\n", i);

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

		init_data.fwnode = child;

		ret = devm_led_classdev_register_ext(&client->dev, &led->cdev,
			&init_data);
		if (ret)
			return dev_err_probe(&client->dev, ret,
					     "Failed to register LED %u\n", val);
	}

	data->leds = leds;

	return 0;
}

static const struct of_device_id ltc3208_match_table[] = {
	{.compatible = "adi,ltc3208"},
	{ }
};
MODULE_DEVICE_TABLE(of, ltc3208_match_table);

static const struct i2c_device_id ltc3208_idtable[] = {
	{ "ltc3208" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ltc3208_idtable);

static struct i2c_driver ltc3208_driver = {
	.driver = {
		.name = "ltc3208",
		.of_match_table = ltc3208_match_table,
	},
	.id_table = ltc3208_idtable,
	.probe = ltc3208_probe,
};
module_i2c_driver(ltc3208_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Jan Carlo Roleda <jancarlo.roleda@analog.com>");
MODULE_DESCRIPTION("LTC3208 LED Driver");
