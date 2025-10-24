// SPDX-License-Identifier: GPL-2.0+
/*
 *  HID driver for gaming keys on Logitech gaming keyboards (such as the G15)
 *
 *  Copyright (c) 2019 Hans de Goede <hdegoede@redhat.com>
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/leds.h>
#include <linux/led-class-multicolor.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/wait.h>
#include <dt-bindings/leds/common.h>

#include "hid-ids.h"

#define LG_G15_TRANSFER_BUF_SIZE	20

#define LG_G15_FEATURE_REPORT		0x02

#define LG_G510_FEATURE_M_KEYS_LEDS	0x04
#define LG_G510_FEATURE_BACKLIGHT_RGB	0x05
#define LG_G510_FEATURE_POWER_ON_RGB	0x06

#define LG_G510_INPUT_MACRO_KEYS	0x03
#define LG_G510_INPUT_KBD_BACKLIGHT	0x04

#define LG_G13_INPUT_REPORT		0x01
#define LG_G13_FEATURE_M_KEYS_LEDS	0x05
#define LG_G13_FEATURE_BACKLIGHT_RGB	0x07
#define LG_G13_BACKLIGHT_HW_ON_BIT	23

/**
 * g13_input_report.keybits[] is not 32-bit aligned, so we can't use the bitops macros.
 *
 * @ary: Pointer to array of u8s
 * @b: Bit index into ary, LSB first.  Not range checked.
 */
#define TEST_BIT(ary, b)	((1 << ((b) & 7)) & (ary)[(b) >> 3])

enum lg_g15_model {
	LG_G13,
	LG_G15,
	LG_G15_V2,
	LG_G510,
	LG_G510_USB_AUDIO,
	LG_Z10,
};

enum lg_g15_led_type {
	LG_G15_KBD_BRIGHTNESS,
	LG_G15_LCD_BRIGHTNESS,
	LG_G15_BRIGHTNESS_MAX,
	LG_G15_MACRO_PRESET1 = 2,
	LG_G15_MACRO_PRESET2,
	LG_G15_MACRO_PRESET3,
	LG_G15_MACRO_RECORD,
	LG_G15_LED_MAX
};

struct g13_input_report {
	u8 report_id;	/* Report ID is always set to 1. */
	u8 joy_x, joy_y;
	u8 keybits[5];
};

struct lg_g15_led {
	union {
		struct led_classdev cdev;
		struct led_classdev_mc mcdev;
	};
	enum led_brightness brightness;
	enum lg_g15_led_type led;
	/* Used to store initial color intensities before subled_info is allocated */
	u8 red, green, blue;
};

struct lg_g15_data {
	/* Must be first for proper dma alignment */
	u8 transfer_buf[LG_G15_TRANSFER_BUF_SIZE];
	/* Protects the transfer_buf and led brightness */
	struct mutex mutex;
	struct work_struct work;
	struct input_dev *input;
	struct input_dev *input_js; /* Separate joystick device for G13. */
	struct hid_device *hdev;
	enum lg_g15_model model;
	struct lg_g15_led leds[LG_G15_LED_MAX];
	bool game_mode_enabled;
	bool backlight_disabled;	/* true == HW backlight toggled *OFF* */
};

/********* G13 LED functions ***********/
/*
 * G13 retains no state across power cycles, and always powers up with the backlight on,
 * color #5AFF6E, all macro key LEDs off.
 */
static int lg_g13_get_leds_state(struct lg_g15_data *g15)
{
	u8 * const tbuf = g15->transfer_buf;
	int ret, high;

	/* RGB backlight. */
	ret = hid_hw_raw_request(g15->hdev, LG_G13_FEATURE_BACKLIGHT_RGB,
				 tbuf, 5,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret != 5) {
		hid_err(g15->hdev, "Error getting backlight brightness: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	/* Normalize RGB intensities against the highest component. */
	high = max3(tbuf[1], tbuf[2], tbuf[3]);
	if (high) {
		g15->leds[LG_G15_KBD_BRIGHTNESS].red =
			DIV_ROUND_CLOSEST(tbuf[1] * 255, high);
		g15->leds[LG_G15_KBD_BRIGHTNESS].green =
			DIV_ROUND_CLOSEST(tbuf[2] * 255, high);
		g15->leds[LG_G15_KBD_BRIGHTNESS].blue =
			DIV_ROUND_CLOSEST(tbuf[3] * 255, high);
		g15->leds[LG_G15_KBD_BRIGHTNESS].brightness = high;
	} else {
		g15->leds[LG_G15_KBD_BRIGHTNESS].red        = 255;
		g15->leds[LG_G15_KBD_BRIGHTNESS].green      = 255;
		g15->leds[LG_G15_KBD_BRIGHTNESS].blue       = 255;
		g15->leds[LG_G15_KBD_BRIGHTNESS].brightness = 0;
	}

	/* Macro LEDs. */
	ret = hid_hw_raw_request(g15->hdev, LG_G13_FEATURE_M_KEYS_LEDS,
				 tbuf, 5,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret != 5) {
		hid_err(g15->hdev, "Error getting macro LED brightness: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	for (int i = LG_G15_MACRO_PRESET1; i < LG_G15_LED_MAX; ++i)
		g15->leds[i].brightness = !!(tbuf[1] & (1 << (i - LG_G15_MACRO_PRESET1)));

	/*
	 * Bit 23 of g13_input_report.keybits[] contains the backlight's
	 * current HW toggle state.  Retrieve it from the device.
	 */
	ret = hid_hw_raw_request(g15->hdev, LG_G13_INPUT_REPORT,
				 tbuf, sizeof(struct g13_input_report),
				 HID_INPUT_REPORT, HID_REQ_GET_REPORT);
	if (ret != sizeof(struct g13_input_report)) {
		hid_err(g15->hdev, "Error getting backlight on/off state: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}
	g15->backlight_disabled =
		!TEST_BIT(((struct g13_input_report *) tbuf)->keybits,
			  LG_G13_BACKLIGHT_HW_ON_BIT);

	return 0;
}

static int lg_g13_kbd_led_write(struct lg_g15_data *g15,
				struct lg_g15_led *g15_led,
				enum led_brightness brightness)
{
	struct mc_subled const * const subleds = g15_led->mcdev.subled_info;
	u8 * const tbuf = g15->transfer_buf;
	int ret;

	guard(mutex)(&g15->mutex);

	led_mc_calc_color_components(&g15_led->mcdev, brightness);

	tbuf[0] = 5;
	tbuf[1] = subleds[0].brightness;
	tbuf[2] = subleds[1].brightness;
	tbuf[3] = subleds[2].brightness;
	tbuf[4] = 0;

	ret = hid_hw_raw_request(g15->hdev, LG_G13_FEATURE_BACKLIGHT_RGB,
				 tbuf, 5,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret != 5) {
		hid_err(g15->hdev, "Error setting backlight brightness: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	g15_led->brightness = brightness;
	return 0;
}

static int lg_g13_kbd_led_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(led_cdev);
	struct lg_g15_led *g15_led =
		container_of(mc, struct lg_g15_led, mcdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);

	/* Ignore LED off on unregister / keyboard unplug */
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	return lg_g13_kbd_led_write(g15, g15_led, brightness);
}

static enum led_brightness lg_g13_kbd_led_get(struct led_classdev *led_cdev)
{
	struct led_classdev_mc const * const mc = lcdev_to_mccdev(led_cdev);
	struct lg_g15_led const *g15_led =
		container_of(mc, struct lg_g15_led, mcdev);

	return g15_led->brightness;
}

static int lg_g13_mkey_led_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct lg_g15_led *g15_led =
		container_of(led_cdev, struct lg_g15_led, cdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);
	int i, ret;
	u8 * const tbuf = g15->transfer_buf;
	u8 val, mask = 0;

	/* Ignore LED off on unregister / keyboard unplug */
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	guard(mutex)(&g15->mutex);

	for (i = LG_G15_MACRO_PRESET1; i < LG_G15_LED_MAX; ++i) {
		if (i == g15_led->led)
			val = brightness;
		else
			val = g15->leds[i].brightness;

		if (val)
			mask |= 1 << (i - LG_G15_MACRO_PRESET1);
	}

	tbuf[0] = 5;
	tbuf[1] = mask;
	tbuf[2] = 0;
	tbuf[3] = 0;
	tbuf[4] = 0;

	ret = hid_hw_raw_request(g15->hdev, LG_G13_FEATURE_M_KEYS_LEDS,
				 tbuf, 5,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret != 5) {
		hid_err(g15->hdev, "Error setting LED brightness: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	g15_led->brightness = brightness;
	return 0;
}

static enum led_brightness lg_g13_mkey_led_get(struct led_classdev *led_cdev)
{
	/*
	 * G13 doesn't change macro key LEDs behind our back, so they're
	 * whatever we last set them to.
	 */
	struct lg_g15_led *g15_led =
		container_of(led_cdev, struct lg_g15_led, cdev);

	return g15_led->brightness;
}

/******** G15 and G15 v2 LED functions ********/

static int lg_g15_update_led_brightness(struct lg_g15_data *g15)
{
	int ret;

	ret = hid_hw_raw_request(g15->hdev, LG_G15_FEATURE_REPORT,
				 g15->transfer_buf, 4,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret != 4) {
		hid_err(g15->hdev, "Error getting LED brightness: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	g15->leds[LG_G15_KBD_BRIGHTNESS].brightness = g15->transfer_buf[1];
	g15->leds[LG_G15_LCD_BRIGHTNESS].brightness = g15->transfer_buf[2];

	g15->leds[LG_G15_MACRO_PRESET1].brightness =
		!(g15->transfer_buf[3] & 0x01);
	g15->leds[LG_G15_MACRO_PRESET2].brightness =
		!(g15->transfer_buf[3] & 0x02);
	g15->leds[LG_G15_MACRO_PRESET3].brightness =
		!(g15->transfer_buf[3] & 0x04);
	g15->leds[LG_G15_MACRO_RECORD].brightness =
		!(g15->transfer_buf[3] & 0x08);

	return 0;
}

static enum led_brightness lg_g15_led_get(struct led_classdev *led_cdev)
{
	struct lg_g15_led *g15_led =
		container_of(led_cdev, struct lg_g15_led, cdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);
	enum led_brightness brightness;

	mutex_lock(&g15->mutex);
	lg_g15_update_led_brightness(g15);
	brightness = g15->leds[g15_led->led].brightness;
	mutex_unlock(&g15->mutex);

	return brightness;
}

static int lg_g15_led_set(struct led_classdev *led_cdev,
			  enum led_brightness brightness)
{
	struct lg_g15_led *g15_led =
		container_of(led_cdev, struct lg_g15_led, cdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);
	u8 val, mask = 0;
	int i, ret;

	/* Ignore LED off on unregister / keyboard unplug */
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	mutex_lock(&g15->mutex);

	g15->transfer_buf[0] = LG_G15_FEATURE_REPORT;
	g15->transfer_buf[3] = 0;

	if (g15_led->led < LG_G15_BRIGHTNESS_MAX) {
		g15->transfer_buf[1] = g15_led->led + 1;
		g15->transfer_buf[2] = brightness << (g15_led->led * 4);
	} else {
		for (i = LG_G15_MACRO_PRESET1; i < LG_G15_LED_MAX; i++) {
			if (i == g15_led->led)
				val = brightness;
			else
				val = g15->leds[i].brightness;

			if (val)
				mask |= 1 << (i - LG_G15_MACRO_PRESET1);
		}

		g15->transfer_buf[1] = 0x04;
		g15->transfer_buf[2] = ~mask;
	}

	ret = hid_hw_raw_request(g15->hdev, LG_G15_FEATURE_REPORT,
				 g15->transfer_buf, 4,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret == 4) {
		/* Success */
		g15_led->brightness = brightness;
		ret = 0;
	} else {
		hid_err(g15->hdev, "Error setting LED brightness: %d\n", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	mutex_unlock(&g15->mutex);

	return ret;
}

static void lg_g15_leds_changed_work(struct work_struct *work)
{
	struct lg_g15_data *g15 = container_of(work, struct lg_g15_data, work);
	enum led_brightness old_brightness[LG_G15_BRIGHTNESS_MAX];
	enum led_brightness brightness[LG_G15_BRIGHTNESS_MAX];
	int i, ret;

	mutex_lock(&g15->mutex);
	for (i = 0; i < LG_G15_BRIGHTNESS_MAX; i++)
		old_brightness[i] = g15->leds[i].brightness;

	ret = lg_g15_update_led_brightness(g15);

	for (i = 0; i < LG_G15_BRIGHTNESS_MAX; i++)
		brightness[i] = g15->leds[i].brightness;
	mutex_unlock(&g15->mutex);

	if (ret)
		return;

	for (i = 0; i < LG_G15_BRIGHTNESS_MAX; i++) {
		if (brightness[i] == old_brightness[i])
			continue;

		led_classdev_notify_brightness_hw_changed(&g15->leds[i].cdev,
							  brightness[i]);
	}
}

/******** G510 LED functions ********/

static int lg_g510_get_initial_led_brightness(struct lg_g15_data *g15, int i)
{
	int ret, high;

	ret = hid_hw_raw_request(g15->hdev, LG_G510_FEATURE_BACKLIGHT_RGB + i,
				 g15->transfer_buf, 4,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret != 4) {
		hid_err(g15->hdev, "Error getting LED brightness: %d\n", ret);
		return (ret < 0) ? ret : -EIO;
	}

	high = max3(g15->transfer_buf[1], g15->transfer_buf[2],
		    g15->transfer_buf[3]);

	if (high) {
		g15->leds[i].red =
			DIV_ROUND_CLOSEST(g15->transfer_buf[1] * 255, high);
		g15->leds[i].green =
			DIV_ROUND_CLOSEST(g15->transfer_buf[2] * 255, high);
		g15->leds[i].blue =
			DIV_ROUND_CLOSEST(g15->transfer_buf[3] * 255, high);
		g15->leds[i].brightness = high;
	} else {
		g15->leds[i].red   = 255;
		g15->leds[i].green = 255;
		g15->leds[i].blue  = 255;
		g15->leds[i].brightness = 0;
	}

	if (i)
		return 0;

	ret = hid_hw_raw_request(g15->hdev, LG_G510_INPUT_KBD_BACKLIGHT,
				 g15->transfer_buf, 2,
				 HID_INPUT_REPORT, HID_REQ_GET_REPORT);
	if (ret != 2) {
		/* This can happen when a KVM switch is used, so only warn. */
		hid_warn(g15->hdev, "Error getting backlight state: %d\n", ret);
		return 0;
	}

	g15->backlight_disabled = g15->transfer_buf[1] & 0x04;

	return 0;
}

/* Must be called with g15->mutex locked */
static int lg_g510_kbd_led_write(struct lg_g15_data *g15,
				 struct lg_g15_led *g15_led,
				 enum led_brightness brightness)
{
	struct mc_subled *subleds = g15_led->mcdev.subled_info;
	int ret;

	led_mc_calc_color_components(&g15_led->mcdev, brightness);

	g15->transfer_buf[0] = 5 + g15_led->led;
	g15->transfer_buf[1] = subleds[0].brightness;
	g15->transfer_buf[2] = subleds[1].brightness;
	g15->transfer_buf[3] = subleds[2].brightness;

	ret = hid_hw_raw_request(g15->hdev,
				 LG_G510_FEATURE_BACKLIGHT_RGB + g15_led->led,
				 g15->transfer_buf, 4,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret == 4) {
		/* Success */
		g15_led->brightness = brightness;
		ret = 0;
	} else {
		hid_err(g15->hdev, "Error setting LED brightness: %d\n", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	return ret;
}

static int lg_g510_kbd_led_set(struct led_classdev *led_cdev,
			       enum led_brightness brightness)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(led_cdev);
	struct lg_g15_led *g15_led =
		container_of(mc, struct lg_g15_led, mcdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);
	int ret;

	/* Ignore LED off on unregister / keyboard unplug */
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	mutex_lock(&g15->mutex);
	ret = lg_g510_kbd_led_write(g15, g15_led, brightness);
	mutex_unlock(&g15->mutex);

	return ret;
}

static enum led_brightness lg_g510_kbd_led_get(struct led_classdev *led_cdev)
{
	struct led_classdev_mc *mc = lcdev_to_mccdev(led_cdev);
	struct lg_g15_led *g15_led =
		container_of(mc, struct lg_g15_led, mcdev);

	return g15_led->brightness;
}

static void lg_g510_leds_sync_work(struct work_struct *work)
{
	struct lg_g15_data *g15 = container_of(work, struct lg_g15_data, work);
	struct lg_g15_led *g15_led = &g15->leds[LG_G15_KBD_BRIGHTNESS];

	mutex_lock(&g15->mutex);
	lg_g510_kbd_led_write(g15, g15_led, g15_led->brightness);
	mutex_unlock(&g15->mutex);
}

static int lg_g510_update_mkey_led_brightness(struct lg_g15_data *g15)
{
	int ret;

	ret = hid_hw_raw_request(g15->hdev, LG_G510_FEATURE_M_KEYS_LEDS,
				 g15->transfer_buf, 2,
				 HID_FEATURE_REPORT, HID_REQ_GET_REPORT);
	if (ret != 2) {
		hid_err(g15->hdev, "Error getting LED brightness: %d\n", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	g15->leds[LG_G15_MACRO_PRESET1].brightness =
		!!(g15->transfer_buf[1] & 0x80);
	g15->leds[LG_G15_MACRO_PRESET2].brightness =
		!!(g15->transfer_buf[1] & 0x40);
	g15->leds[LG_G15_MACRO_PRESET3].brightness =
		!!(g15->transfer_buf[1] & 0x20);
	g15->leds[LG_G15_MACRO_RECORD].brightness =
		!!(g15->transfer_buf[1] & 0x10);

	return 0;
}

static enum led_brightness lg_g510_mkey_led_get(struct led_classdev *led_cdev)
{
	struct lg_g15_led *g15_led =
		container_of(led_cdev, struct lg_g15_led, cdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);
	enum led_brightness brightness;

	mutex_lock(&g15->mutex);
	lg_g510_update_mkey_led_brightness(g15);
	brightness = g15->leds[g15_led->led].brightness;
	mutex_unlock(&g15->mutex);

	return brightness;
}

static int lg_g510_mkey_led_set(struct led_classdev *led_cdev,
				enum led_brightness brightness)
{
	struct lg_g15_led *g15_led =
		container_of(led_cdev, struct lg_g15_led, cdev);
	struct lg_g15_data *g15 = dev_get_drvdata(led_cdev->dev->parent);
	u8 val, mask = 0;
	int i, ret;

	/* Ignore LED off on unregister / keyboard unplug */
	if (led_cdev->flags & LED_UNREGISTERING)
		return 0;

	mutex_lock(&g15->mutex);

	for (i = LG_G15_MACRO_PRESET1; i < LG_G15_LED_MAX; i++) {
		if (i == g15_led->led)
			val = brightness;
		else
			val = g15->leds[i].brightness;

		if (val)
			mask |= 0x80 >> (i - LG_G15_MACRO_PRESET1);
	}

	g15->transfer_buf[0] = LG_G510_FEATURE_M_KEYS_LEDS;
	g15->transfer_buf[1] = mask;

	ret = hid_hw_raw_request(g15->hdev, LG_G510_FEATURE_M_KEYS_LEDS,
				 g15->transfer_buf, 2,
				 HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	if (ret == 2) {
		/* Success */
		g15_led->brightness = brightness;
		ret = 0;
	} else {
		hid_err(g15->hdev, "Error setting LED brightness: %d\n", ret);
		ret = (ret < 0) ? ret : -EIO;
	}

	mutex_unlock(&g15->mutex);

	return ret;
}

/******** Generic LED functions ********/
static int lg_g15_get_initial_led_brightness(struct lg_g15_data *g15)
{
	int ret;

	switch (g15->model) {
	case LG_G13:
		return lg_g13_get_leds_state(g15);
	case LG_G15:
	case LG_G15_V2:
		return lg_g15_update_led_brightness(g15);
	case LG_G510:
	case LG_G510_USB_AUDIO:
		ret = lg_g510_get_initial_led_brightness(g15, 0);
		if (ret)
			return ret;

		ret = lg_g510_get_initial_led_brightness(g15, 1);
		if (ret)
			return ret;

		return lg_g510_update_mkey_led_brightness(g15);
	case LG_Z10:
		/*
		 * Getting the LCD backlight brightness is not supported.
		 * Reading Feature(2) fails with -EPIPE and this crashes
		 * the LCD and touch keys part of the speakers.
		 */
		return 0;
	}
	return -EINVAL; /* Never reached */
}

/******** Input functions ********/

/* Table mapping keybits[] bit positions to event codes. */
/* Note: Indices are discontinuous to aid readability. */
static const u16 g13_keys_for_bits[] = {
	/* Main keypad - keys G1 - G22 */
	[0] = KEY_MACRO1,
	[1] = KEY_MACRO2,
	[2] = KEY_MACRO3,
	[3] = KEY_MACRO4,
	[4] = KEY_MACRO5,
	[5] = KEY_MACRO6,
	[6] = KEY_MACRO7,
	[7] = KEY_MACRO8,
	[8] = KEY_MACRO9,
	[9] = KEY_MACRO10,
	[10] = KEY_MACRO11,
	[11] = KEY_MACRO12,
	[12] = KEY_MACRO13,
	[13] = KEY_MACRO14,
	[14] = KEY_MACRO15,
	[15] = KEY_MACRO16,
	[16] = KEY_MACRO17,
	[17] = KEY_MACRO18,
	[18] = KEY_MACRO19,
	[19] = KEY_MACRO20,
	[20] = KEY_MACRO21,
	[21] = KEY_MACRO22,

	/* LCD menu buttons. */
	[24] = KEY_KBD_LCD_MENU5,	/* "Next page" button */
	[25] = KEY_KBD_LCD_MENU1,	/* Left-most */
	[26] = KEY_KBD_LCD_MENU2,
	[27] = KEY_KBD_LCD_MENU3,
	[28] = KEY_KBD_LCD_MENU4,	/* Right-most */

	/* Macro preset and record buttons; have red LEDs under them. */
	[29] = KEY_MACRO_PRESET1,
	[30] = KEY_MACRO_PRESET2,
	[31] = KEY_MACRO_PRESET3,
	[32] = KEY_MACRO_RECORD_START,

	/* 33-35 handled by joystick device. */

	/* Backlight toggle. */
	[37] = KEY_LIGHTS_TOGGLE,
};

#define G13_JS_KEYBITS_OFFSET	33

static const u16 g13_keys_for_bits_js[] = {
	/* Joystick buttons */
	/* These keybits are at bit indices 33, 34, and 35. */
	BTN_BASE,	/* Left side */
	BTN_BASE2,	/* Bottom side */
	BTN_THUMB,	/* Stick depress */
};

static int lg_g13_event(struct lg_g15_data *g15, u8 const *data)
{
	struct g13_input_report const * const rep = (struct g13_input_report *) data;
	int i, val;
	bool backlight_disabled;

	/*
	 * Main macropad and menu keys.
	 * Emit key events defined for each bit position.
	 */
	for (i = 0; i < ARRAY_SIZE(g13_keys_for_bits); ++i) {
		if (g13_keys_for_bits[i]) {
			val = TEST_BIT(rep->keybits, i);
			input_report_key(g15->input, g13_keys_for_bits[i], val);
		}
	}
	input_sync(g15->input);

	/*
	 * Joystick.
	 * Emit button and deflection events.
	 */
	for (i = 0; i < ARRAY_SIZE(g13_keys_for_bits_js); ++i) {
		val = TEST_BIT(rep->keybits, i + G13_JS_KEYBITS_OFFSET);
		input_report_key(g15->input_js, g13_keys_for_bits_js[i], val);
	}
	input_report_abs(g15->input_js, ABS_X, rep->joy_x);
	input_report_abs(g15->input_js, ABS_Y, rep->joy_y);
	input_sync(g15->input_js);

	/*
	 * Bit 23 of keybits[] reports the current backlight on/off state.  If
	 * it has changed from the last cached value, apply an update.
	 */
	backlight_disabled = !TEST_BIT(rep->keybits, LG_G13_BACKLIGHT_HW_ON_BIT);
	if (backlight_disabled ^ g15->backlight_disabled) {
		led_classdev_notify_brightness_hw_changed(
			&g15->leds[LG_G15_KBD_BRIGHTNESS].mcdev.led_cdev,
			backlight_disabled
			? 0 : g15->leds[LG_G15_KBD_BRIGHTNESS].brightness);
		g15->backlight_disabled = backlight_disabled;
	}

	return 0;
}

/* On the G15 Mark I Logitech has been quite creative with which bit is what */
static void lg_g15_handle_lcd_menu_keys(struct lg_g15_data *g15, u8 *data)
{
	int i, val;

	/* Most left (round/display) button below the LCD */
	input_report_key(g15->input, KEY_KBD_LCD_MENU1, data[8] & 0x80);
	/* 4 other buttons below the LCD */
	for (i = 0; i < 4; i++) {
		val = data[i + 2] & 0x80;
		input_report_key(g15->input, KEY_KBD_LCD_MENU2 + i, val);
	}
}

static int lg_g15_event(struct lg_g15_data *g15, u8 *data)
{
	int i, val;

	/* G1 - G6 */
	for (i = 0; i < 6; i++) {
		val = data[i + 1] & (1 << i);
		input_report_key(g15->input, KEY_MACRO1 + i, val);
	}
	/* G7 - G12 */
	for (i = 0; i < 6; i++) {
		val = data[i + 2] & (1 << i);
		input_report_key(g15->input, KEY_MACRO7 + i, val);
	}
	/* G13 - G17 */
	for (i = 0; i < 5; i++) {
		val = data[i + 1] & (4 << i);
		input_report_key(g15->input, KEY_MACRO13 + i, val);
	}
	/* G18 */
	input_report_key(g15->input, KEY_MACRO18, data[8] & 0x40);

	/* M1 - M3 */
	for (i = 0; i < 3; i++) {
		val = data[i + 6] & (1 << i);
		input_report_key(g15->input, KEY_MACRO_PRESET1 + i, val);
	}
	/* MR */
	input_report_key(g15->input, KEY_MACRO_RECORD_START, data[7] & 0x40);

	lg_g15_handle_lcd_menu_keys(g15, data);

	/* Backlight cycle button pressed? */
	if (data[1] & 0x80)
		schedule_work(&g15->work);

	input_sync(g15->input);
	return 0;
}

static int lg_g15_v2_event(struct lg_g15_data *g15, u8 *data)
{
	int i, val;

	/* G1 - G6 */
	for (i = 0; i < 6; i++) {
		val = data[1] & (1 << i);
		input_report_key(g15->input, KEY_MACRO1 + i, val);
	}

	/* M1 - M3 + MR */
	input_report_key(g15->input, KEY_MACRO_PRESET1, data[1] & 0x40);
	input_report_key(g15->input, KEY_MACRO_PRESET2, data[1] & 0x80);
	input_report_key(g15->input, KEY_MACRO_PRESET3, data[2] & 0x20);
	input_report_key(g15->input, KEY_MACRO_RECORD_START, data[2] & 0x40);

	/* Round button to the left of the LCD */
	input_report_key(g15->input, KEY_KBD_LCD_MENU1, data[2] & 0x80);
	/* 4 buttons below the LCD */
	for (i = 0; i < 4; i++) {
		val = data[2] & (2 << i);
		input_report_key(g15->input, KEY_KBD_LCD_MENU2 + i, val);
	}

	/* Backlight cycle button pressed? */
	if (data[2] & 0x01)
		schedule_work(&g15->work);

	input_sync(g15->input);
	return 0;
}

static int lg_g510_event(struct lg_g15_data *g15, u8 *data)
{
	bool game_mode_enabled;
	int i, val;

	/* G1 - G18 */
	for (i = 0; i < 18; i++) {
		val = data[i / 8 + 1] & (1 << (i % 8));
		input_report_key(g15->input, KEY_MACRO1 + i, val);
	}

	/* Game mode on/off slider */
	game_mode_enabled = data[3] & 0x04;
	if (game_mode_enabled != g15->game_mode_enabled) {
		if (game_mode_enabled)
			hid_info(g15->hdev, "Game Mode enabled, Windows (super) key is disabled\n");
		else
			hid_info(g15->hdev, "Game Mode disabled\n");
		g15->game_mode_enabled = game_mode_enabled;
	}

	/* M1 - M3 */
	for (i = 0; i < 3; i++) {
		val = data[3] & (0x10 << i);
		input_report_key(g15->input, KEY_MACRO_PRESET1 + i, val);
	}
	/* MR */
	input_report_key(g15->input, KEY_MACRO_RECORD_START, data[3] & 0x80);

	/* LCD menu keys */
	for (i = 0; i < 5; i++) {
		val = data[4] & (1 << i);
		input_report_key(g15->input, KEY_KBD_LCD_MENU1 + i, val);
	}

	/* Headphone Mute */
	input_report_key(g15->input, KEY_MUTE, data[4] & 0x20);
	/* Microphone Mute */
	input_report_key(g15->input, KEY_F20, data[4] & 0x40);

	input_sync(g15->input);
	return 0;
}

static int lg_g510_leds_event(struct lg_g15_data *g15, u8 *data)
{
	struct lg_g15_led *g15_led = &g15->leds[LG_G15_KBD_BRIGHTNESS];
	bool backlight_disabled;

	backlight_disabled = data[1] & 0x04;
	if (backlight_disabled == g15->backlight_disabled)
		return 0;

	led_classdev_notify_brightness_hw_changed(
		&g15_led->mcdev.led_cdev,
		backlight_disabled ? 0 : g15_led->brightness);

	g15->backlight_disabled = backlight_disabled;

	/*
	 * The G510 ignores backlight updates when the backlight is turned off
	 * through the light toggle button on the keyboard, to work around this
	 * we queue a workitem to sync values when the backlight is turned on.
	 */
	if (!backlight_disabled)
		schedule_work(&g15->work);

	return 0;
}

static int lg_g15_raw_event(struct hid_device *hdev, struct hid_report *report,
			    u8 *data, int size)
{
	struct lg_g15_data *g15 = hid_get_drvdata(hdev);

	if (!g15)
		return 0;

	switch (g15->model) {
	case LG_G13:
		if (data[0] == 0x01 && size == sizeof(struct g13_input_report))
			return lg_g13_event(g15, data);
		break;
	case LG_G15:
		if (data[0] == 0x02 && size == 9)
			return lg_g15_event(g15, data);
		break;
	case LG_G15_V2:
		if (data[0] == 0x02 && size == 5)
			return lg_g15_v2_event(g15, data);
		break;
	case LG_Z10:
		if (data[0] == 0x02 && size == 9) {
			lg_g15_handle_lcd_menu_keys(g15, data);
			input_sync(g15->input);
		}
		break;
	case LG_G510:
	case LG_G510_USB_AUDIO:
		if (data[0] == LG_G510_INPUT_MACRO_KEYS && size == 5)
			return lg_g510_event(g15, data);
		if (data[0] == LG_G510_INPUT_KBD_BACKLIGHT && size == 2)
			return lg_g510_leds_event(g15, data);
		break;
	}

	return 0;
}

static int lg_g15_input_open(struct input_dev *dev)
{
	struct hid_device *hdev = input_get_drvdata(dev);

	return hid_hw_open(hdev);
}

static void lg_g15_input_close(struct input_dev *dev)
{
	struct hid_device *hdev = input_get_drvdata(dev);

	hid_hw_close(hdev);
}

static void lg_g15_setup_led_rgb(struct lg_g15_data *g15, int index)
{
	int i;
	struct mc_subled *subled_info;
	struct lg_g15_led * const gled = &g15->leds[index];

	if (g15->model == LG_G13) {
		gled->mcdev.led_cdev.brightness_set_blocking =
			lg_g13_kbd_led_set;
		gled->mcdev.led_cdev.brightness_get =
			lg_g13_kbd_led_get;
		gled->mcdev.led_cdev.flags = LED_BRIGHT_HW_CHANGED;
	} else {
		gled->mcdev.led_cdev.brightness_set_blocking =
			lg_g510_kbd_led_set;
		gled->mcdev.led_cdev.brightness_get =
			lg_g510_kbd_led_get;
		if (index == LG_G15_KBD_BRIGHTNESS)
			g15->leds[index].mcdev.led_cdev.flags = LED_BRIGHT_HW_CHANGED;
	}
	gled->mcdev.led_cdev.max_brightness = 255;
	gled->mcdev.num_colors = 3;

	subled_info = devm_kcalloc(&g15->hdev->dev, 3, sizeof(*subled_info), GFP_KERNEL);
	if (!subled_info)
		return;

	for (i = 0; i < 3; i++) {
		switch (i + 1) {
		case LED_COLOR_ID_RED:
			subled_info[i].color_index = LED_COLOR_ID_RED;
			subled_info[i].intensity = gled->red;
			break;
		case LED_COLOR_ID_GREEN:
			subled_info[i].color_index = LED_COLOR_ID_GREEN;
			subled_info[i].intensity = gled->green;
			break;
		case LED_COLOR_ID_BLUE:
			subled_info[i].color_index = LED_COLOR_ID_BLUE;
			subled_info[i].intensity = gled->blue;
			break;
		}
		subled_info[i].channel = i;
	}
	gled->mcdev.subled_info = subled_info;
}

static int lg_g15_register_led(struct lg_g15_data *g15, int i, const char *name)
{
	int ret;

	g15->leds[i].led = i;
	g15->leds[i].cdev.name = name;

	switch (g15->model) {
	case LG_G13:
		if (i < LG_G15_BRIGHTNESS_MAX) {
			/* RGB backlight. */
			lg_g15_setup_led_rgb(g15, i);
			ret = devm_led_classdev_multicolor_register_ext(&g15->hdev->dev,
									&g15->leds[i].mcdev,
									NULL);
		} else {
			/* Macro keys */
			g15->leds[i].cdev.brightness_set_blocking = lg_g13_mkey_led_set;
			g15->leds[i].cdev.brightness_get = lg_g13_mkey_led_get;
			g15->leds[i].cdev.max_brightness = 1;

			ret = devm_led_classdev_register(&g15->hdev->dev,
							 &g15->leds[i].cdev);
		}
		break;
	case LG_G15:
	case LG_G15_V2:
		g15->leds[i].cdev.brightness_get = lg_g15_led_get;
		fallthrough;
	case LG_Z10:
		g15->leds[i].cdev.brightness_set_blocking = lg_g15_led_set;
		if (i < LG_G15_BRIGHTNESS_MAX) {
			g15->leds[i].cdev.flags = LED_BRIGHT_HW_CHANGED;
			g15->leds[i].cdev.max_brightness = 2;
		} else {
			g15->leds[i].cdev.max_brightness = 1;
		}
		ret = devm_led_classdev_register(&g15->hdev->dev, &g15->leds[i].cdev);
		break;
	case LG_G510:
	case LG_G510_USB_AUDIO:
		switch (i) {
		case LG_G15_LCD_BRIGHTNESS:
			/*
			 * The G510 does not have a separate LCD brightness,
			 * but it does have a separate power-on (reset) value.
			 */
			g15->leds[i].cdev.name = "g15::power_on_backlight_val";
			fallthrough;
		case LG_G15_KBD_BRIGHTNESS:
			/* register multicolor LED */
			lg_g15_setup_led_rgb(g15, i);
			ret = devm_led_classdev_multicolor_register_ext(&g15->hdev->dev,
									&g15->leds[i].mcdev,
									NULL);
			break;
		default:
			g15->leds[i].cdev.brightness_set_blocking =
				lg_g510_mkey_led_set;
			g15->leds[i].cdev.brightness_get =
				lg_g510_mkey_led_get;
			g15->leds[i].cdev.max_brightness = 1;
			ret = devm_led_classdev_register(&g15->hdev->dev, &g15->leds[i].cdev);
		}
		break;
	}

	return ret;
}

/* Common input device init code shared between keyboards and Z-10 speaker handling */
static void lg_g15_init_input_dev_core(struct hid_device *hdev, struct input_dev *input,
				       char const *name)
{
	input->name = name;
	input->phys = hdev->phys;
	input->uniq = hdev->uniq;
	input->id.bustype = hdev->bus;
	input->id.vendor  = hdev->vendor;
	input->id.product = hdev->product;
	input->id.version = hdev->version;
	input->dev.parent = &hdev->dev;
	input->open = lg_g15_input_open;
	input->close = lg_g15_input_close;
}

static void lg_g15_init_input_dev(struct hid_device *hdev, struct input_dev *input,
				  const char *name)
{
	int i;

	lg_g15_init_input_dev_core(hdev, input, name);

	/* Keys below the LCD, intended for controlling a menu on the LCD */
	for (i = 0; i < 5; i++)
		input_set_capability(input, EV_KEY, KEY_KBD_LCD_MENU1 + i);
}

static void lg_g13_init_input_dev(struct hid_device *hdev,
				  struct input_dev *input, const char *name,
				  struct input_dev *input_js, const char *name_js)
{
	/* Macropad. */
	lg_g15_init_input_dev_core(hdev, input, name);
	for (int i = 0; i < ARRAY_SIZE(g13_keys_for_bits); ++i) {
		if (g13_keys_for_bits[i])
			input_set_capability(input, EV_KEY, g13_keys_for_bits[i]);
	}

	/* OBTW, we're a joystick, too... */
	lg_g15_init_input_dev_core(hdev, input_js, name_js);
	for (int i = 0; i < ARRAY_SIZE(g13_keys_for_bits_js); ++i)
		input_set_capability(input_js, EV_KEY, g13_keys_for_bits_js[i]);

	input_set_capability(input_js, EV_ABS, ABS_X);
	input_set_abs_params(input_js, ABS_X, 0, 255, 0, 0);
	input_set_capability(input_js, EV_ABS, ABS_Y);
	input_set_abs_params(input_js, ABS_Y, 0, 255, 0, 0);
}

static int lg_g15_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	static const char * const led_names[] = {
		"g15::kbd_backlight",
		"g15::lcd_backlight",
		"g15::macro_preset1",
		"g15::macro_preset2",
		"g15::macro_preset3",
		"g15::macro_record",
	};
	u8 gkeys_settings_output_report = 0;
	u8 gkeys_settings_feature_report = 0;
	struct hid_report_enum *rep_enum;
	unsigned int connect_mask = 0;
	bool has_ff000000 = false;
	struct lg_g15_data *g15;
	struct input_dev *input, *input_js;
	struct hid_report *rep;
	int ret, i, gkeys = 0;

	hdev->quirks |= HID_QUIRK_INPUT_PER_APP;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	/*
	 * Some models have multiple interfaces, we want the interface with
	 * the f000.0000 application input report.
	 */
	rep_enum = &hdev->report_enum[HID_INPUT_REPORT];
	list_for_each_entry(rep, &rep_enum->report_list, list) {
		if (rep->application == 0xff000000)
			has_ff000000 = true;
	}
	if (!has_ff000000)
		return hid_hw_start(hdev, HID_CONNECT_DEFAULT);

	g15 = devm_kzalloc(&hdev->dev, sizeof(*g15), GFP_KERNEL);
	if (!g15)
		return -ENOMEM;

	mutex_init(&g15->mutex);

	input = devm_input_allocate_device(&hdev->dev);
	if (!input)
		return -ENOMEM;

	g15->hdev = hdev;
	g15->model = id->driver_data;
	g15->input = input;
	input_set_drvdata(input, hdev);
	hid_set_drvdata(hdev, (void *)g15);

	switch (g15->model) {
	case LG_G13:
		/*
		 * The G13 has an analog thumbstick with nearby buttons.  Some
		 * libraries and applications are known to ignore devices that
		 * don't "look like" a joystick, and a device with two ABS axes
		 * and 25+ macro keys would confuse them.
		 *
		 * Create an additional input device dedicated to appear as a
		 * simplified joystick (two ABS axes, three BTN buttons).
		 */
		input_js = devm_input_allocate_device(&hdev->dev);
		if (!input_js)
			return -ENOMEM;
		g15->input_js = input_js;
		input_set_drvdata(input_js, hdev);

		connect_mask = HID_CONNECT_HIDRAW;
		gkeys = 25;
		break;
	case LG_G15:
		INIT_WORK(&g15->work, lg_g15_leds_changed_work);
		/*
		 * The G15 and G15 v2 use a separate usb-device (on a builtin
		 * hub) which emulates a keyboard for the F1 - F12 emulation
		 * on the G-keys, which we disable, rendering the emulated kbd
		 * non-functional, so we do not let hid-input connect.
		 */
		connect_mask = HID_CONNECT_HIDRAW;
		gkeys_settings_output_report = 0x02;
		gkeys = 18;
		break;
	case LG_G15_V2:
		INIT_WORK(&g15->work, lg_g15_leds_changed_work);
		connect_mask = HID_CONNECT_HIDRAW;
		gkeys_settings_output_report = 0x02;
		gkeys = 6;
		break;
	case LG_G510:
	case LG_G510_USB_AUDIO:
		INIT_WORK(&g15->work, lg_g510_leds_sync_work);
		connect_mask = HID_CONNECT_HIDINPUT | HID_CONNECT_HIDRAW;
		gkeys_settings_feature_report = 0x01;
		gkeys = 18;
		break;
	case LG_Z10:
		connect_mask = HID_CONNECT_HIDRAW;
		break;
	}

	ret = hid_hw_start(hdev, connect_mask);
	if (ret)
		return ret;

	/* Tell the keyboard to stop sending F1-F12 + 1-6 for G1 - G18 */
	if (gkeys_settings_output_report) {
		g15->transfer_buf[0] = gkeys_settings_output_report;
		memset(g15->transfer_buf + 1, 0, gkeys);
		/*
		 * The kbd ignores our output report if we do not queue
		 * an URB on the USB input endpoint first...
		 */
		ret = hid_hw_open(hdev);
		if (ret)
			goto error_hw_stop;
		ret = hid_hw_output_report(hdev, g15->transfer_buf, gkeys + 1);
		hid_hw_close(hdev);
	}

	if (gkeys_settings_feature_report) {
		g15->transfer_buf[0] = gkeys_settings_feature_report;
		memset(g15->transfer_buf + 1, 0, gkeys);
		ret = hid_hw_raw_request(g15->hdev,
				gkeys_settings_feature_report,
				g15->transfer_buf, gkeys + 1,
				HID_FEATURE_REPORT, HID_REQ_SET_REPORT);
	}

	if (ret < 0) {
		hid_err(hdev, "Error %d disabling keyboard emulation for the G-keys, falling back to generic hid-input driver\n",
			ret);
		hid_set_drvdata(hdev, NULL);
		return 0;
	}

	/* Get initial brightness levels */
	ret = lg_g15_get_initial_led_brightness(g15);
	if (ret)
		goto error_hw_stop;

	if (g15->model == LG_Z10) {
		lg_g15_init_input_dev(hdev, g15->input, "Logitech Z-10 LCD Menu Keys");
		ret = input_register_device(g15->input);
		if (ret)
			goto error_hw_stop;

		ret = lg_g15_register_led(g15, 1, "z-10::lcd_backlight");
		if (ret)
			goto error_hw_stop;

		return 0; /* All done */
	} else if (g15->model == LG_G13) {
		static char const * const g13_led_names[] = {
			/* Backlight is shared between LCD and keys. */
			"g13:rgb:kbd_backlight",
			NULL,	/* Keep in sync with led_type enum */
			"g13:red:macro_preset_1",
			"g13:red:macro_preset_2",
			"g13:red:macro_preset_3",
			"g13:red:macro_record",
		};
		lg_g13_init_input_dev(hdev,
				      input, "Logitech G13 Gaming Keypad",
				      input_js, "Logitech G13 Thumbstick");
		ret = input_register_device(input);
		if (ret)
			goto error_hw_stop;
		ret = input_register_device(input_js);
		if (ret)
			goto error_hw_stop;

		for (i = 0; i < ARRAY_SIZE(g13_led_names); ++i) {
			if (g13_led_names[i]) {
				ret = lg_g15_register_led(g15, i, g13_led_names[i]);
				if (ret)
					goto error_hw_stop;
			}
		}
		led_classdev_notify_brightness_hw_changed(
			&g15->leds[LG_G15_KBD_BRIGHTNESS].mcdev.led_cdev,
			g15->backlight_disabled
			? 0 : g15->leds[LG_G15_KBD_BRIGHTNESS].brightness);
		return 0;
	}

	/* Setup and register input device */
	lg_g15_init_input_dev(hdev, input, "Logitech Gaming Keyboard Gaming Keys");

	/* G-keys */
	for (i = 0; i < gkeys; i++)
		input_set_capability(input, EV_KEY, KEY_MACRO1 + i);

	/* M1 - M3 and MR keys */
	for (i = 0; i < 3; i++)
		input_set_capability(input, EV_KEY, KEY_MACRO_PRESET1 + i);
	input_set_capability(input, EV_KEY, KEY_MACRO_RECORD_START);

	/*
	 * On the G510 only report headphone and mic mute keys when *not* using
	 * the builtin USB audio device. When the builtin audio is used these
	 * keys directly toggle mute (and the LEDs) on/off.
	 */
	if (g15->model == LG_G510) {
		input_set_capability(input, EV_KEY, KEY_MUTE);
		/* Userspace expects F20 for micmute */
		input_set_capability(input, EV_KEY, KEY_F20);
	}

	ret = input_register_device(input);
	if (ret)
		goto error_hw_stop;

	/* Register LED devices */
	for (i = 0; i < LG_G15_LED_MAX; i++) {
		ret = lg_g15_register_led(g15, i, led_names[i]);
		if (ret)
			goto error_hw_stop;
	}

	return 0;

error_hw_stop:
	hid_hw_stop(hdev);
	return ret;
}

static const struct hid_device_id lg_g15_devices[] = {
	/*
	 * The G13 is a macropad-only device with an LCD, LED backlighing,
	 * and joystick.
	 */
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
			 USB_DEVICE_ID_LOGITECH_G13),
		.driver_data = LG_G13 },
	/* The G11 is a G15 without the LCD, treat it as a G15 */
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
		USB_DEVICE_ID_LOGITECH_G11),
		.driver_data = LG_G15 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
			 USB_DEVICE_ID_LOGITECH_G15_LCD),
		.driver_data = LG_G15 },
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
			 USB_DEVICE_ID_LOGITECH_G15_V2_LCD),
		.driver_data = LG_G15_V2 },
	/* G510 without a headset plugged in */
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
			 USB_DEVICE_ID_LOGITECH_G510),
		.driver_data = LG_G510 },
	/* G510 with headset plugged in / with extra USB audio interface */
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
			 USB_DEVICE_ID_LOGITECH_G510_USB_AUDIO),
		.driver_data = LG_G510_USB_AUDIO },
	/* Z-10 speakers */
	{ HID_USB_DEVICE(USB_VENDOR_ID_LOGITECH,
			 USB_DEVICE_ID_LOGITECH_Z_10_SPK),
		.driver_data = LG_Z10 },
	{ }
};
MODULE_DEVICE_TABLE(hid, lg_g15_devices);

static struct hid_driver lg_g15_driver = {
	.name			= "lg-g15",
	.id_table		= lg_g15_devices,
	.raw_event		= lg_g15_raw_event,
	.probe			= lg_g15_probe,
};
module_hid_driver(lg_g15_driver);

MODULE_AUTHOR("Hans de Goede <hdegoede@redhat.com>");
MODULE_DESCRIPTION("HID driver for gaming keys on Logitech gaming keyboards");
MODULE_LICENSE("GPL");
