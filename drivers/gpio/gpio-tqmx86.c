// SPDX-License-Identifier: GPL-2.0
/*
 * TQ-Systems TQMx86 PLD GPIO driver
 *
 * Based on vendor driver by:
 *   Vadim V.Vlasov <vvlasov@dev.rtsoft.ru>
 */

#include <linux/bitmap.h>
#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#define TQMX86_NGPIO	8
#define TQMX86_NGPO	4	/* 0-3 - output */
#define TQMX86_NGPI	4	/* 4-7 - input */
#define TQMX86_DIR_INPUT_MASK	0xf0	/* 0-3 - output, 4-7 - input */

#define TQMX86_GPIODD	0	/* GPIO Data Direction Register */
#define TQMX86_GPIOD	1	/* GPIO Data Register */
#define TQMX86_GPIIC	3	/* GPI Interrupt Configuration Register */
#define TQMX86_GPIIS	4	/* GPI Interrupt Status Register */

#define TQMX86_GPII_NONE	0
#define TQMX86_GPII_FALLING	BIT(0)
#define TQMX86_GPII_RISING	BIT(1)
/* Stored in irq_type as a trigger type, but not actually valid as a register
 * value, so the name doesn't use "GPII"
 */
#define TQMX86_INT_BOTH		(BIT(0) | BIT(1))
#define TQMX86_GPII_MASK	(BIT(0) | BIT(1))
#define TQMX86_GPII_BITS	2
/* Stored in irq_type with GPII bits */
#define TQMX86_INT_UNMASKED	BIT(2)

struct tqmx86_gpio_data {
	struct gpio_chip	chip;
	void __iomem		*io_base;
	int			irq;
	/* Lock must be held for accessing output and irq_type fields */
	raw_spinlock_t		spinlock;
	DECLARE_BITMAP(output, TQMX86_NGPIO);
	u8			irq_type[TQMX86_NGPI];
};

static u8 tqmx86_gpio_read(struct tqmx86_gpio_data *gd, unsigned int reg)
{
	return ioread8(gd->io_base + reg);
}

static void tqmx86_gpio_write(struct tqmx86_gpio_data *gd, u8 val,
			      unsigned int reg)
{
	iowrite8(val, gd->io_base + reg);
}

static int tqmx86_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);

	return !!(tqmx86_gpio_read(gpio, TQMX86_GPIOD) & BIT(offset));
}

static void tqmx86_gpio_set(struct gpio_chip *chip, unsigned int offset,
			    int value)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	__assign_bit(offset, gpio->output, value);
	tqmx86_gpio_write(gpio, bitmap_get_value8(gpio->output, 0), TQMX86_GPIOD);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);
}

static int tqmx86_gpio_direction_input(struct gpio_chip *chip,
				       unsigned int offset)
{
	/* Direction cannot be changed. Validate is an input. */
	if (BIT(offset) & TQMX86_DIR_INPUT_MASK)
		return 0;
	else
		return -EINVAL;
}

static int tqmx86_gpio_direction_output(struct gpio_chip *chip,
					unsigned int offset,
					int value)
{
	/* Direction cannot be changed, validate is an output */
	if (BIT(offset) & TQMX86_DIR_INPUT_MASK)
		return -EINVAL;

	tqmx86_gpio_set(chip, offset, value);
	return 0;
}

static int tqmx86_gpio_get_direction(struct gpio_chip *chip,
				     unsigned int offset)
{
	if (TQMX86_DIR_INPUT_MASK & BIT(offset))
		return GPIO_LINE_DIRECTION_IN;

	return GPIO_LINE_DIRECTION_OUT;
}

static void tqmx86_gpio_irq_config(struct tqmx86_gpio_data *gpio, int offset)
	__must_hold(&gpio->spinlock)
{
	u8 type = TQMX86_GPII_NONE, gpiic;

	if (gpio->irq_type[offset] & TQMX86_INT_UNMASKED) {
		type = gpio->irq_type[offset] & TQMX86_GPII_MASK;

		if (type == TQMX86_INT_BOTH)
			type = tqmx86_gpio_get(&gpio->chip, offset + TQMX86_NGPO)
				? TQMX86_GPII_FALLING
				: TQMX86_GPII_RISING;
	}

	gpiic = tqmx86_gpio_read(gpio, TQMX86_GPIIC);
	gpiic &= ~(TQMX86_GPII_MASK << (offset * TQMX86_GPII_BITS));
	gpiic |= type << (offset * TQMX86_GPII_BITS);
	tqmx86_gpio_write(gpio, gpiic, TQMX86_GPIIC);
}

static void tqmx86_gpio_irq_mask(struct irq_data *data)
{
	unsigned int offset = (data->hwirq - TQMX86_NGPO);
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(
		irq_data_get_irq_chip_data(data));
	unsigned long flags;

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	gpio->irq_type[offset] &= ~TQMX86_INT_UNMASKED;
	tqmx86_gpio_irq_config(gpio, offset);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);

	gpiochip_disable_irq(&gpio->chip, irqd_to_hwirq(data));
}

static void tqmx86_gpio_irq_unmask(struct irq_data *data)
{
	unsigned int offset = (data->hwirq - TQMX86_NGPO);
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(
		irq_data_get_irq_chip_data(data));
	unsigned long flags;

	gpiochip_enable_irq(&gpio->chip, irqd_to_hwirq(data));

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	gpio->irq_type[offset] |= TQMX86_INT_UNMASKED;
	tqmx86_gpio_irq_config(gpio, offset);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);
}

static int tqmx86_gpio_irq_set_type(struct irq_data *data, unsigned int type)
{
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(
		irq_data_get_irq_chip_data(data));
	unsigned int offset = (data->hwirq - TQMX86_NGPO);
	unsigned int edge_type = type & IRQF_TRIGGER_MASK;
	unsigned long flags;
	u8 new_type;

	switch (edge_type) {
	case IRQ_TYPE_EDGE_RISING:
		new_type = TQMX86_GPII_RISING;
		break;
	case IRQ_TYPE_EDGE_FALLING:
		new_type = TQMX86_GPII_FALLING;
		break;
	case IRQ_TYPE_EDGE_BOTH:
		new_type = TQMX86_INT_BOTH;
		break;
	default:
		return -EINVAL; /* not supported */
	}

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	gpio->irq_type[offset] &= ~TQMX86_GPII_MASK;
	gpio->irq_type[offset] |= new_type;
	tqmx86_gpio_irq_config(gpio, offset);
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);

	return 0;
}

static void tqmx86_gpio_irq_handler(struct irq_desc *desc)
{
	struct gpio_chip *chip = irq_desc_get_handler_data(desc);
	struct tqmx86_gpio_data *gpio = gpiochip_get_data(chip);
	struct irq_chip *irq_chip = irq_desc_get_chip(desc);
	unsigned long irq_bits, flags;
	int i;
	u8 irq_status;

	chained_irq_enter(irq_chip, desc);

	irq_status = tqmx86_gpio_read(gpio, TQMX86_GPIIS);
	tqmx86_gpio_write(gpio, irq_status, TQMX86_GPIIS);

	irq_bits = irq_status;

	raw_spin_lock_irqsave(&gpio->spinlock, flags);
	for_each_set_bit(i, &irq_bits, TQMX86_NGPI) {
		/*
		 * Edge-both triggers are implemented by flipping the edge
		 * trigger after each interrupt, as the controller only supports
		 * either rising or falling edge triggers, but not both.
		 *
		 * Internally, the TQMx86 GPIO controller has separate status
		 * registers for rising and falling edge interrupts. GPIIC
		 * configures which bits from which register are visible in the
		 * interrupt status register GPIIS and defines what triggers the
		 * parent IRQ line. Writing to GPIIS always clears both rising
		 * and falling interrupt flags internally, regardless of the
		 * currently configured trigger.
		 *
		 * In consequence, we can cleanly implement the edge-both
		 * trigger in software by first clearing the interrupt and then
		 * setting the new trigger based on the current GPIO input in
		 * tqmx86_gpio_irq_config() - even if an edge arrives between
		 * reading the input and setting the trigger, we will have a new
		 * interrupt pending.
		 */
		if ((gpio->irq_type[i] & TQMX86_GPII_MASK) == TQMX86_INT_BOTH)
			tqmx86_gpio_irq_config(gpio, i);
	}
	raw_spin_unlock_irqrestore(&gpio->spinlock, flags);

	for_each_set_bit(i, &irq_bits, TQMX86_NGPI)
		generic_handle_domain_irq(gpio->chip.irq.domain,
					  i + TQMX86_NGPO);

	chained_irq_exit(irq_chip, desc);
}

/* Minimal runtime PM is needed by the IRQ subsystem */
static int __maybe_unused tqmx86_gpio_runtime_suspend(struct device *dev)
{
	return 0;
}

static int __maybe_unused tqmx86_gpio_runtime_resume(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops tqmx86_gpio_dev_pm_ops = {
	SET_RUNTIME_PM_OPS(tqmx86_gpio_runtime_suspend,
			   tqmx86_gpio_runtime_resume, NULL)
};

static void tqmx86_init_irq_valid_mask(struct gpio_chip *chip,
				       unsigned long *valid_mask,
				       unsigned int ngpios)
{
	/* Only GPIOs 4-7 are valid for interrupts. Clear the others */
	clear_bit(0, valid_mask);
	clear_bit(1, valid_mask);
	clear_bit(2, valid_mask);
	clear_bit(3, valid_mask);
}

static void tqmx86_gpio_irq_print_chip(struct irq_data *d, struct seq_file *p)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);

	seq_printf(p, gc->label);
}

static const struct irq_chip tqmx86_gpio_irq_chip = {
	.irq_mask = tqmx86_gpio_irq_mask,
	.irq_unmask = tqmx86_gpio_irq_unmask,
	.irq_set_type = tqmx86_gpio_irq_set_type,
	.irq_print_chip = tqmx86_gpio_irq_print_chip,
	.flags = IRQCHIP_IMMUTABLE,
	GPIOCHIP_IRQ_RESOURCE_HELPERS,
};

static int tqmx86_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tqmx86_gpio_data *gpio;
	struct gpio_chip *chip;
	struct gpio_irq_chip *girq;
	void __iomem *io_base;
	struct resource *res;
	int ret, irq;

	irq = platform_get_irq_optional(pdev, 0);
	if (irq < 0 && irq != -ENXIO)
		return irq;

	res = platform_get_resource(pdev, IORESOURCE_IO, 0);
	if (!res) {
		dev_err(&pdev->dev, "Cannot get I/O\n");
		return -ENODEV;
	}

	io_base = devm_ioport_map(&pdev->dev, res->start, resource_size(res));
	if (!io_base)
		return -ENOMEM;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	raw_spin_lock_init(&gpio->spinlock);
	gpio->io_base = io_base;

	tqmx86_gpio_write(gpio, (u8)~TQMX86_DIR_INPUT_MASK, TQMX86_GPIODD);

	/*
	 * Reading the previous output state is not possible with TQMx86 hardware.
	 * Initialize all outputs to 0 to have a defined state that matches the
	 * shadow register.
	 */
	tqmx86_gpio_write(gpio, 0, TQMX86_GPIOD);

	chip = &gpio->chip;
	chip->label = "gpio-tqmx86";
	chip->owner = THIS_MODULE;
	chip->can_sleep = false;
	chip->base = -1;
	chip->direction_input = tqmx86_gpio_direction_input;
	chip->direction_output = tqmx86_gpio_direction_output;
	chip->get_direction = tqmx86_gpio_get_direction;
	chip->get = tqmx86_gpio_get;
	chip->set = tqmx86_gpio_set;
	chip->ngpio = TQMX86_NGPIO;
	chip->parent = pdev->dev.parent;

	pm_runtime_enable(&pdev->dev);

	if (irq > 0) {
		u8 irq_status;

		/* Mask all interrupts */
		tqmx86_gpio_write(gpio, 0, TQMX86_GPIIC);

		/* Clear all pending interrupts */
		irq_status = tqmx86_gpio_read(gpio, TQMX86_GPIIS);
		tqmx86_gpio_write(gpio, irq_status, TQMX86_GPIIS);

		girq = &chip->irq;
		gpio_irq_chip_set_chip(girq, &tqmx86_gpio_irq_chip);
		girq->parent_handler = tqmx86_gpio_irq_handler;
		girq->num_parents = 1;
		girq->parents = devm_kcalloc(&pdev->dev, 1,
					     sizeof(*girq->parents),
					     GFP_KERNEL);
		if (!girq->parents) {
			ret = -ENOMEM;
			goto out_pm_dis;
		}
		girq->parents[0] = irq;
		girq->default_type = IRQ_TYPE_NONE;
		girq->handler = handle_simple_irq;
		girq->init_valid_mask = tqmx86_init_irq_valid_mask;

		irq_domain_set_pm_device(girq->domain, dev);
	}

	ret = devm_gpiochip_add_data(dev, chip, gpio);
	if (ret) {
		dev_err(dev, "Could not register GPIO chip\n");
		goto out_pm_dis;
	}

	dev_info(dev, "GPIO functionality initialized with %d pins\n",
		 chip->ngpio);

	return 0;

out_pm_dis:
	pm_runtime_disable(&pdev->dev);

	return ret;
}

static struct platform_driver tqmx86_gpio_driver = {
	.driver = {
		.name = "tqmx86-gpio",
		.pm = &tqmx86_gpio_dev_pm_ops,
	},
	.probe		= tqmx86_gpio_probe,
};

module_platform_driver(tqmx86_gpio_driver);

MODULE_DESCRIPTION("TQMx86 PLD GPIO Driver");
MODULE_AUTHOR("Andrew Lunn <andrew@lunn.ch>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:tqmx86-gpio");
