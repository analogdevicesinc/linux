// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADSP PORT gpio driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/device.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/soc/adi/adsp-gpio-port.h>
#include "gpiolib.h"

static int adsp_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DIR_CLEAR);
	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_INEN_SET);
	return 0;
}

static int adsp_gpio_direction_output(struct gpio_chip *chip, unsigned int offset,
	int value)
{
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	/*
	 * For open drain ports, they've already been configured by pinctrl and
	 * we should not modify their output characteristics
	 */
	if (port->open_drain & BIT(offset))
		return 0;

	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_INEN_CLEAR);

	if (value)
		__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_SET);
	else
		__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_CLEAR);

	__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DIR_SET);
	return 0;
}

static void adsp_gpio_set_value(struct gpio_chip *chip, unsigned int offset, int value)
{
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	/*
	 * For open drain ports, set as input if driving a 1, set as output
	 * if driving a 0
	 */
	if (port->open_drain & BIT(offset)) {
		if (value) {
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DIR_CLEAR);
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_INEN_SET);
		} else {
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_INEN_CLEAR);
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_CLEAR);
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DIR_SET);
		}
	} else {
		if (value)
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_SET);
		else
			__adsp_gpio_writew(port, BIT(offset), ADSP_PORT_REG_DATA_CLEAR);
	}
}

static int adsp_gpio_get_value(struct gpio_chip *chip, unsigned int offset)
{
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);

	return !!(__adsp_gpio_readw(port, ADSP_PORT_REG_DATA) & BIT(offset));
}

static int adsp_gpio_to_irq(struct gpio_chip *chip, unsigned int offset)
{
	struct adsp_gpio_port *port = to_adsp_gpio_port(chip);
	irq_hw_number_t irq = offset + port->irq_offset;
	int map = irq_find_mapping(port->irq_domain, irq);

	if (!map)
		map = irq_create_mapping(port->irq_domain, irq);

	return map;
}

static int adsp_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adsp_gpio_port *gpio;
	struct resource *res;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	gpio->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(gpio->regs))
		return PTR_ERR(gpio->regs);

	gpio->dev = dev;
	platform_set_drvdata(pdev, gpio);

	ret = adsp_attach_pint_to_gpio(gpio);
	if (ret) {
		if (ret == -EPROBE_DEFER)
			return ret;
		dev_err(dev, "Failed to attach to pint node!\n");
		return ret;
	}

	spin_lock_init(&gpio->lock);

	gpio->gpio.label = "adsp-gpio";
	gpio->gpio.direction_input = adsp_gpio_direction_input;
	gpio->gpio.direction_output = adsp_gpio_direction_output;
	gpio->gpio.get = adsp_gpio_get_value;
	gpio->gpio.set = adsp_gpio_set_value;
	gpio->gpio.to_irq = adsp_gpio_to_irq;
	gpio->gpio.request = gpiochip_generic_request;
	gpio->gpio.free = gpiochip_generic_free;
	gpio->gpio.ngpio = ADSP_PORT_NGPIO;
	gpio->gpio.parent = dev;
	gpio->gpio.base = -1;
	ret = devm_gpiochip_add_data(dev, &gpio->gpio, gpio);
	return ret;
}

static const struct of_device_id adsp_gpio_of_match[] = {
	{ .compatible = "adi,adsp-port-gpio", },
	{ },
};
MODULE_DEVICE_TABLE(of, adsp_gpio_of_match);

static struct platform_driver adsp_gpio_driver = {
	.driver = {
		.name = "adsp-port-gpio",
		.of_match_table = adsp_gpio_of_match,
	},
	.probe = adsp_gpio_probe,
};

static int __init adsp_gpio_init(void)
{
	return platform_driver_register(&adsp_gpio_driver);
}

arch_initcall(adsp_gpio_init);
