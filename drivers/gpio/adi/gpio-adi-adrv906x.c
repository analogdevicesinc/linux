// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/bits.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include <dt-bindings/interrupt-controller/arm-gic.h>

#include "gpio-adi-smc.h"

/* The adrv906x SoC utilizes GPIO controllers which support up to 32 GPIO's per instance */
#define ADI_ADRV906X_MAX_GPIO_PER_INST (32U)

#define ADI_ADRV906X_GPIO_NUM_INSTS       (4U)

#define ADI_ADRV906X_MAX_GPIOS (ADI_ADRV906X_MAX_GPIO_PER_INST * ADI_ADRV906X_GPIO_NUM_INSTS)

#define ADI_ADRV906X_GET_GPIO_INST_NUM(pin)               (pin / ADI_ADRV906X_MAX_GPIO_PER_INST)
#define ADI_ADRV906X_GET_GPIO_INST_BIT_MASK(pin)  (0x1 << (pin % ADI_ADRV906X_MAX_GPIO_PER_INST))

#define ADI_ADRV906X_GPIO_INST_0_OFFSET   (0x800U)
#define ADI_ADRV906X_GPIO_INST_1_OFFSET   (0x900U)
#define ADI_ADRV906X_GPIO_INST_2_OFFSET   (0xA00U)
#define ADI_ADRV906X_GPIO_INST_3_OFFSET   (0xB00U)

#define ADI_ADRV906X_GPIO_WRITE_OFFSET    (0x000U)
#define ADI_ADRV906X_GPIO_CLEAR_OFFSET    (0x004U)
#define ADI_ADRV906X_GPIO_SET_OFFSET              (0x008U)
#define ADI_ADRV906X_GPIO_TOGGLE_OFFSET   (0x00AU)
#define ADI_ADRV906X_GPIO_READ_OFFSET             (0x010U)

#define ADI_ADRV906X_GPIO_DIR_CONTROL_BASE        (0xB14U)

#define GPIO_LINE_DIRECTION_IN  1
#define GPIO_LINE_DIRECTION_OUT 0

#define GPIO_DIR_CONTROL_OE_BIT_MASK    (0x1U)
#define GPIO_DIR_CONTROL_IE_BIT_MASK    (0x2U)

#define GPIO_PIN_STATE_LOW      0
#define GPIO_PIN_STATE_HIGH     1

static uint32_t adrv906x_gpio_inst_base_addr[ADI_ADRV906X_GPIO_NUM_INSTS] = {
	ADI_ADRV906X_GPIO_INST_0_OFFSET,
	ADI_ADRV906X_GPIO_INST_1_OFFSET,
	ADI_ADRV906X_GPIO_INST_2_OFFSET,
	ADI_ADRV906X_GPIO_INST_3_OFFSET
};

/**
 * struct adi_adrv906x_platform_data -  adi adrv906x soc gpio platform data structure
 * @label:	string to store in gpio->label
 * @ngpio:	max number of gpio pins
 */
struct adi_adrv906x_platform_data {
	const char *label;
};

/**
 * struct adi_adrv906x_gpio - gpio device private data structure
 * @chip:	instance of the gpio_chip
 * @regmap:	base address of the GPIO device
 */
struct adi_adrv906x_gpio {
	struct gpio_chip chip;
	void __iomem *base_addr;
	const struct adi_adrv906x_platform_data *p_data;
	uint32_t pintmux_addr;

	// For interrupt-controller
	struct irq_chip irq;
	struct fwnode_handle *fwnode;
};

/**
 * adi_adrv906x_gpio_get_direction - Read the direction of the specified GPIO pin
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 *
 * This function returns the direction of the specified GPIO.
 *
 * Return: GPIO_LINE_DIRECTION_OUT or GPIO_LINE_DIRECTION_IN
 */
static int adi_adrv906x_gpio_get_direction(struct gpio_chip *chip, unsigned int pin)
{
	uint32_t reg;
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(chip);

	reg = ioread32(gpio->base_addr + ADI_ADRV906X_GPIO_DIR_CONTROL_BASE + (pin * sizeof(uint32_t)));

	if (reg & GPIO_DIR_CONTROL_OE_BIT_MASK)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

/**
 * adi_adrv906x_gpio_direction_input - Set the direction of the specified GPIO pin as input
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 *
 * This sets the direction of the gpio pin as input.
 *
 * Return: 0 always
 */
static int adi_adrv906x_gpio_direction_input(struct gpio_chip *chip, unsigned int pin)
{
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(chip);

	iowrite32(GPIO_DIR_CONTROL_IE_BIT_MASK, gpio->base_addr + ADI_ADRV906X_GPIO_DIR_CONTROL_BASE + (pin * sizeof(uint32_t)));

	return 0;
}

/**
 * adi_adrv906x_gpio_get() - Get the state of the specified pin of GPIO device
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 *
 * This function reads the state of the specified pin of the GPIO device.
 *
 * Return: 0 if the pin is low, 1 if pin is high.
 */
static int adi_adrv906x_gpio_get(struct gpio_chip *chip, unsigned int pin)
{
	uint32_t reg;
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(chip);
	uint32_t gpio_inst_number = ADI_ADRV906X_GET_GPIO_INST_NUM(pin);
	uint32_t gpio_inst_bit_mask = ADI_ADRV906X_GET_GPIO_INST_BIT_MASK(pin);

	/* if pin is input, read value from the read register, else write register */
	int ret = adi_adrv906x_gpio_get_direction(chip, pin);

	if (ret == GPIO_LINE_DIRECTION_IN)
		reg = ioread32(gpio->base_addr + adrv906x_gpio_inst_base_addr[gpio_inst_number] + ADI_ADRV906X_GPIO_READ_OFFSET);
	else
		reg = ioread32(gpio->base_addr + adrv906x_gpio_inst_base_addr[gpio_inst_number] + ADI_ADRV906X_GPIO_WRITE_OFFSET);

	if (gpio_inst_bit_mask & reg)
		return GPIO_PIN_STATE_HIGH;

	return GPIO_PIN_STATE_LOW;
}

/**
 * adi_adrv906x_gpio_set - Modify the state of the pin with specified value
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 * @state:	value used to modify the state of the specified pin
 *
 * This function calculates the register offset based on the given pin number and sets the state of a
 * gpio pin to the specified value. The state is either 0 or non-zero.
 */
static void adi_adrv906x_gpio_set(struct gpio_chip *chip, unsigned int pin, int state)
{
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(chip);
	uint32_t gpio_inst_number = ADI_ADRV906X_GET_GPIO_INST_NUM(pin);
	uint32_t gpio_inst_bit_mask = ADI_ADRV906X_GET_GPIO_INST_BIT_MASK(pin);

	if (state)
		iowrite32(gpio_inst_bit_mask, gpio->base_addr + adrv906x_gpio_inst_base_addr[gpio_inst_number] + ADI_ADRV906X_GPIO_SET_OFFSET);
	else
		iowrite32(gpio_inst_bit_mask, gpio->base_addr + adrv906x_gpio_inst_base_addr[gpio_inst_number] + ADI_ADRV906X_GPIO_CLEAR_OFFSET);

	return;
}

/**
 * adi_adrv906x_gpio_direction_output - Set the direction of the specified GPIO pin as output
 * @chip:	gpio_chip instance to be worked on
 * @pin:	gpio pin number within the device
 * @state:	value to be written to specified pin
 *
 * This function sets the direction of specified GPIO pin as output, configures
 * the Output Enable register for the pin and uses adi_adrv906x_gpio_set to set
 * the state of the pin to the value specified.
 *
 * Return: 0 always
 */
static int adi_adrv906x_gpio_direction_output(struct gpio_chip *chip,
					      unsigned int pin, int state)
{
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(chip);

	/*
	 * Write the specified value to the output pin
	 */
	adi_adrv906x_gpio_set(chip, pin, state);

	/*
	 * Setup pin as output
	 */
	iowrite32(GPIO_DIR_CONTROL_OE_BIT_MASK, gpio->base_addr + ADI_ADRV906X_GPIO_DIR_CONTROL_BASE + (pin * sizeof(uint32_t)));

	return 0;
}

/**
 * adi_adrv906x_gpio_irq_convert_to_supported_type
 * - GIC supports LEVEL_HIGH and EDGE_RISING interrupts.
 * - Our trasmutter can manage both edge and level interrupts, and allows inverting the polarity of both types.
 * - We here adapt the requested type to match those supported by GIC.
 * - Note that IRQ_TYPE_NONE is used to skip the configuration of the hardware in some drivers, we transform it to level high.
 * @requested_type: IRQ type requested bo be configured
 * @supported_type: IRQ type to be used
 * @polarity:       polarity for the supported_type IRQ
 * Returns 0 on success
 */
static int adi_adrv906x_gpio_irq_convert_to_supported_type(unsigned int requested_type, unsigned int *supported_type, bool *polarity)
{
	bool pol;

	if (!supported_type) return -EINVAL;

	switch (requested_type) {
	case IRQ_TYPE_EDGE_RISING:
	case IRQ_TYPE_LEVEL_HIGH:
		*supported_type = requested_type;
		pol = true;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		*supported_type = IRQ_TYPE_EDGE_RISING;
		pol = false;
		break;
	case IRQ_TYPE_LEVEL_LOW:
		*supported_type = IRQ_TYPE_LEVEL_HIGH;
		pol = false;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		printk(KERN_INFO "adi-adrv906x-gpio: requested IRQ_TYPE_EDGE_BOTH. Forced to IRQ_TYPE_EDGE_RISING\n");
		*supported_type = IRQ_TYPE_EDGE_RISING;
		pol = true;
		break;
	case IRQ_TYPE_NONE:
		printk(KERN_INFO "adi-adrv906x-gpio: requested IRQ_TYPE_NONE. Forced to IRQ_TYPE_LEVEL_HIGH\n");
		*supported_type = IRQ_TYPE_LEVEL_HIGH;
		pol = true;
		break;
	default:
		printk(KERN_ERR "adi-adrv906x-gpio: Unknown requested irq type: %i\n", requested_type);
		return -EINVAL;
	}

	if (polarity) *polarity = pol;

	return 0;
}

/**
 * adi_adrv906x_gpio_child_to_parent_hwirq - Look up the parent hardware irq from a child hardware irq
 * @gc:          gpio_chip pointer
 * @child:       GPIO index 0..ngpio-1
 * @child_type:  IRQ type (such as IRQ_TYPE_*)
 * @parent:      returned parent IRQ number
 * @parent_type: returned parent IRQ type (such as IRQ_TYPE_*)
 */
static int adi_adrv906x_gpio_child_to_parent_hwirq(struct gpio_chip *gc,
						   unsigned int child,
						   unsigned int child_type,
						   unsigned int *parent,
						   unsigned int *parent_type)
{
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int type;
	bool polarity;
	unsigned int irq_num;

	if (adi_adrv906x_gpio_irq_convert_to_supported_type(child_type, &type, &polarity) != 0) {
		printk(KERN_ERR "adi-adrv906x-gpio: Cannot find supported irq type for requested type %u)\n", child_type);
		return -EINVAL;
	}

	if (adi_adrv906x_pintmux_map(child, polarity, &irq_num, gpio->pintmux_addr)) {
		*parent = irq_num;
		*parent_type = type;
		return 0;
	}
	printk(KERN_ERR "adi-adrv906x-gpio: pintmux_map failed (child irq number: %u)\n", child);

	return -EINVAL;
}

/**
 * adi_adrv906x_gpio_populate_parent_alloc_arg - Allocates and populates the specific struct for the parent's IRQ domain
 * @chip:         gpio_chip pointer
 * @parent_hwirq: parent IRQ number
 * @parent_type:  IRQ type (such as IRQ_TYPE_*)
 */
static void *adi_adrv906x_gpio_populate_parent_alloc_arg(struct gpio_chip *chip,
							 unsigned int parent_hwirq,
							 unsigned int parent_type)
{
	struct irq_fwspec *fwspec;

	fwspec = kmalloc(sizeof(*fwspec), GFP_KERNEL);
	if (!fwspec)
		return NULL;

	fwspec->fwnode = chip->irq.parent_domain->fwnode;
	fwspec->param_count = 3;
	fwspec->param[0] = GIC_SPI;
	fwspec->param[1] = parent_hwirq;
	fwspec->param[2] = parent_type;

	return fwspec;
}

/**
 * adi_adrv906x_gpio_irq_mask - Masks the IRQ
 * @d: irq_data pointer
 */
static void adi_adrv906x_gpio_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	irq_chip_mask_parent(d);
	gpiochip_disable_irq(gc, irqd_to_hwirq(d));
}

/**
 * adi_adrv906x_gpio_irq_unmask - Unmasks the IRQ
 * @d: irq_data pointer
 */
static void adi_adrv906x_gpio_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	gpiochip_enable_irq(gc, irqd_to_hwirq(d));
	irq_chip_unmask_parent(d);
}

/**
 * adi_adrv906x_gpio_irq_set_type - Sets the parent's IRQ type
 * @d:    irq_data pointer
 * @type: IRQ type (such as IRQ_TYPE_*)
 */
static int adi_adrv906x_gpio_irq_set_type(struct irq_data *d, unsigned int type)
{
	unsigned int supported_type;

	if (adi_adrv906x_gpio_irq_convert_to_supported_type(type, &supported_type, NULL) != 0) {
		printk(KERN_ERR "adi-adrv906x-gpio: Cannot find supported irq type for requested type %u)\n", type);
		return -EINVAL;
	}

	return irq_chip_set_type_parent(d, supported_type);
}

/**
 * adi_adrv906x_gpio_irq_shutdown - Shutdowns the IRQ, unmapping the gpio from the tf-a pintmux service
 * @d: irq_data pointer
 */
static void adi_adrv906x_gpio_irq_shutdown(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct adi_adrv906x_gpio *gpio = gpiochip_get_data(gc);
	unsigned int gpionum = d->hwirq;

	gpiochip_unlock_as_irq(gc, gpionum);
	if (!adi_adrv906x_pintmux_unmap(gpionum, gpio->pintmux_addr))
		printk(KERN_ERR "adi-adrv906x-gpio: pintmux_unmap failed (child irq number: %u)\n", gpionum);
}

static const struct adi_adrv906x_platform_data adi_adrv906x_gpio_def = {
	.label	= "adi_adrv906x_gpio",
};

static const struct of_device_id adi_adrv906x_gpio_ids[] = {
	{ .compatible = "adi,adrv906x-gpio", .data = &adi_adrv906x_gpio_def },
	{},
};

static int adi_adrv906x_gpio_probe(struct platform_device *pdev)
{
	int ret;
	struct adi_adrv906x_gpio *gpio;
	struct gpio_chip *chip;
	const struct of_device_id *match;
	uint32_t ngpio = 0U;

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	match = of_match_node(adi_adrv906x_gpio_ids, pdev->dev.of_node);
	if (!match) {
		dev_err(&pdev->dev, "of_match_node() failed\n");
		return -EINVAL;
	}

	if (of_property_read_u32(pdev->dev.of_node, "ngpios", &ngpio))
		return -EINVAL;

	if (ngpio > ADI_ADRV906X_MAX_GPIOS) {
		dev_err(&pdev->dev, "ngpios exceeded maximum, failed\n");
		return -EINVAL;
	}

	gpio->p_data = match->data;
	platform_set_drvdata(pdev, gpio);
	if (of_property_read_u32(pdev->dev.of_node, "pintmux", &gpio->pintmux_addr))
		return -EINVAL;
	gpio->base_addr = devm_platform_ioremap_resource(pdev, 0);

	if (IS_ERR(gpio->base_addr))
		return PTR_ERR(gpio->base_addr);

	chip = &gpio->chip;
	chip->label = gpio->p_data->label;
	chip->parent = &pdev->dev;
	chip->owner = THIS_MODULE;
	chip->get_direction = adi_adrv906x_gpio_get_direction;
	chip->direction_input = adi_adrv906x_gpio_direction_input;
	chip->direction_output = adi_adrv906x_gpio_direction_output;
	chip->get = adi_adrv906x_gpio_get;
	chip->set = adi_adrv906x_gpio_set;
	chip->base = -1;
	chip->ngpio = (uint16_t)ngpio;
	chip->can_sleep = 0;

	if (of_property_read_bool(pdev->dev.of_node, "interrupt-controller")) {
		struct gpio_irq_chip *girq;
		struct device_node *node = pdev->dev.of_node;
		struct device_node *irq_parent;
		struct irq_domain *parent;

		irq_parent = of_irq_find_parent(node);
		if (!irq_parent) {
			dev_err(&pdev->dev, "no IRQ parent node\n");
			return -ENODEV;
		}
		parent = irq_find_host(irq_parent);
		of_node_put(irq_parent);
		if (!parent) {
			dev_err(&pdev->dev, "no IRQ parent domain\n");
			return -ENODEV;
		}

		gpio->irq.name = "adi-adrv906x-gpio";
		gpio->irq.irq_mask = adi_adrv906x_gpio_irq_mask;
		gpio->irq.irq_unmask = adi_adrv906x_gpio_irq_unmask;
		gpio->irq.irq_set_type = adi_adrv906x_gpio_irq_set_type;
		gpio->irq.irq_shutdown = adi_adrv906x_gpio_irq_shutdown;
		gpio->irq.irq_eoi = irq_chip_eoi_parent;

		girq = &gpio->chip.irq;
		girq->chip = &gpio->irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_bad_irq;
		girq->fwnode = of_node_to_fwnode(node);
		girq->parent_domain = parent;
		girq->child_to_parent_hwirq = adi_adrv906x_gpio_child_to_parent_hwirq;
		girq->populate_parent_alloc_arg = adi_adrv906x_gpio_populate_parent_alloc_arg;
	}

	/* report a bug if gpio chip registration fails */
	ret = devm_gpiochip_add_data(&pdev->dev, &gpio->chip, gpio);
	if (ret) {
		dev_err(&pdev->dev, "Failed to add gpio chip\n");
		return -EINVAL;
	}

	printk(KERN_INFO "adi_adrv906x_gpio_probe :: SUCCESS \n");

	return 0;
}

MODULE_DEVICE_TABLE(of, adi_adrv906x_gpio_ids);

static struct platform_driver adi_adrv906x_gpio_driver = {
	.driver			= {
		.name		= "adi-adrv906x-gpio",
		.of_match_table = of_match_ptr(adi_adrv906x_gpio_ids)
	},
	.probe			= adi_adrv906x_gpio_probe,
};

module_platform_driver(adi_adrv906x_gpio_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ADI ADRV906X GPIO driver");
MODULE_AUTHOR("Analog Devices Inc.");
