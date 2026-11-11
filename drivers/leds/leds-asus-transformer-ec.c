// SPDX-License-Identifier: GPL-2.0-only

#include <linux/err.h>
#include <linux/leds.h>
#include <linux/mfd/asus-transformer-ec.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

enum {
	ASUSEC_LED_AMBER,
	ASUSEC_LED_GREEN,
	ASUSEC_LED_MAX
};

struct asus_ec_led_config {
	const char *name;
	unsigned int color;
	u64 ctrl_bit;
};

struct asus_ec_led {
	const struct asusec_core *ec;
	struct led_classdev cdev;
	u64 ctrl_bit;
};

static const struct asus_ec_led_config asus_ec_leds[] = {
	[ASUSEC_LED_AMBER] = {
		.name = "amber",
		.color = LED_COLOR_ID_AMBER,
		.ctrl_bit = ASUSEC_CTL_LED_AMBER,
	},
	[ASUSEC_LED_GREEN] = {
		.name = "green",
		.color = LED_COLOR_ID_GREEN,
		.ctrl_bit = ASUSEC_CTL_LED_GREEN,
	},
};

static enum led_brightness asus_ec_led_get_brightness(struct led_classdev *cdev)
{
	struct asus_ec_led *led = container_of(cdev, struct asus_ec_led, cdev);
	const struct asusec_core *ec = led->ec;
	u64 ctl;
	int ret;

	ret = asus_dockram_access_ctl(ec->dockram, &ctl, 0, 0);
	if (ret)
		return LED_OFF;

	return ctl & led->ctrl_bit ? LED_ON : LED_OFF;
}

static int asus_ec_led_set_brightness(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct asus_ec_led *led = container_of(cdev, struct asus_ec_led, cdev);
	const struct asusec_core *ec = led->ec;

	if (brightness)
		return asus_dockram_access_ctl(ec->dockram, NULL, led->ctrl_bit, led->ctrl_bit);

	return asus_dockram_access_ctl(ec->dockram, NULL, led->ctrl_bit, 0);
}

static int asus_ec_led_probe(struct platform_device *pdev)
{
	const struct asusec_core *ec = dev_get_drvdata(pdev->dev.parent);
	struct device *dev = &pdev->dev;
	struct asus_ec_led *leds;
	int ret;

	leds = devm_kcalloc(dev, ASUSEC_LED_MAX, sizeof(*leds), GFP_KERNEL);
	if (!leds)
		return -ENOMEM;

	for (int i = 0; i < ASUSEC_LED_MAX; i++) {
		const struct asus_ec_led_config *cfg = &asus_ec_leds[i];
		struct asus_ec_led *led = &leds[i];

		led->cdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s:%s", ec->name, cfg->name);
		if (!led->cdev.name)
			return -ENOMEM;

		led->cdev.max_brightness = 1;
		led->cdev.color = cfg->color;
		led->cdev.flags = LED_CORE_SUSPENDRESUME | LED_RETAIN_AT_SHUTDOWN;
		led->cdev.brightness_get = asus_ec_led_get_brightness;
		led->cdev.brightness_set_blocking = asus_ec_led_set_brightness;

		led->ec = ec;
		led->ctrl_bit = cfg->ctrl_bit;

		ret = devm_led_classdev_register(dev, &led->cdev);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to register %s LED\n", cfg->name);
	}

	return 0;
}

static struct platform_driver asus_ec_led_driver = {
	.driver = {
		.name = "asus-transformer-ec-led",
	},
	.probe = asus_ec_led_probe,
};
module_platform_driver(asus_ec_led_driver);

MODULE_ALIAS("platform:asus-transformer-ec-led");
MODULE_AUTHOR("Michał Mirosław <mirq-linux@rere.qmqm.pl>");
MODULE_AUTHOR("Svyatoslav Ryhel <clamor95@gmail.com>");
MODULE_DESCRIPTION("ASUS Transformer's charging LED driver");
MODULE_LICENSE("GPL");
