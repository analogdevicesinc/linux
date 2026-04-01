// SPDX-License-Identifier: GPL-2.0
/* Author: Hans de Goede <hdegoede@redhat.com> */

#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/leds.h>
#include <linux/platform_data/x86/int3472.h>

static int int3472_pled_set(struct led_classdev *led_cdev, enum led_brightness brightness)
{
	struct int3472_pled *led = container_of(led_cdev, struct int3472_pled, classdev);

	gpiod_set_value_cansleep(led->gpio, brightness);
	return 0;
}

int skl_int3472_register_pled(struct int3472_discrete_device *int3472, struct gpio_desc *gpio)
{
	struct int3472_pled *led = &int3472->pled;
	char *p;
	int ret;

	if (led->classdev.dev)
		return -EBUSY;

	led->gpio = gpio;

	/* Generate the name, replacing the ':' in the ACPI devname with '_' */
	snprintf(led->name, sizeof(led->name),
		 "%s::privacy_led", acpi_dev_name(int3472->sensor));
	p = strchr(led->name, ':');
	if (p)
		*p = '_';

	led->classdev.name = led->name;
	led->classdev.max_brightness = 1;
	led->classdev.brightness_set_blocking = int3472_pled_set;

	ret = led_classdev_register(int3472->dev, &led->classdev);
	if (ret)
		return ret;

	led->lookup.provider = led->name;
	led->lookup.dev_id = int3472->sensor_name;
	led->lookup.con_id = "privacy";
	led_add_lookup(&led->lookup);

	return 0;
}

void skl_int3472_unregister_pled(struct int3472_discrete_device *int3472)
{
	struct int3472_pled *led = &int3472->pled;

	if (IS_ERR_OR_NULL(led->classdev.dev))
		return;

	led_remove_lookup(&led->lookup);
	led_classdev_unregister(&led->classdev);
	gpiod_put(led->gpio);
}
