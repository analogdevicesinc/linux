// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * ADSP PINT PORT driver.
 *
 * The default mapping is used for all PINTs, refer to the HRM to identify
 * PORT mapping to PINTs. For example, PINT0 has PORT B (0-15) and PORT A
 * (16-31).
 *
 * Copyright (C) 2022, Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 */

#include <linux/bitops.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdesc.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/soc/adi/adsp-gpio-port.h>

#define ADSP_PINT_IRQS 32

/* Register offsets in a single PINT */
#define ADSP_PINT_REG_MASK_SET			0x00
#define ADSP_PINT_REG_MASK_CLEAR		0x04
#define ADSP_PINT_REG_REQUEST			0x08
#define ADSP_PINT_REG_ASSIGN			0x0c
#define ADSP_PINT_REG_EDGE_SET			0x10
#define ADSP_PINT_REG_EDGE_CLEAR		0x14
#define ADSP_PINT_REG_INVERT_SET		0x18
#define ADSP_PINT_REG_INVERT_CLEAR		0x1c
#define ADSP_PINT_REG_PINSTATE			0x20
#define ADSP_PINT_REG_LATCH				0x24

struct adsp_pint {
	struct irq_chip chip;
	void __iomem *regs;
	struct irq_domain *domain;
	unsigned int irq;
};

static struct adsp_pint *to_adsp_pint(struct irq_chip *chip)
{
	return container_of(chip, struct adsp_pint, chip);
}

/**
 * Each gpio device should be connected to one of the two valid pints with an
 * indicator of which half it is connected to:
 *
 * pint0 {
 *   ...
 * };
 * gpa {
 *   adi,pint = <&pint0 1>;
 * };
 * gpb {
 *   adi,pint = <&pint0 0>;
 * };
 *
 * This relies on the default configuration of the hardware, which we do not
 * expose an interface to change.
 */
int adsp_attach_pint_to_gpio(struct adsp_gpio_port *port)
{
	struct platform_device *pint_pdev;
	struct device_node *pint_node;
	struct adsp_pint *pint;
	struct of_phandle_args args;
	int ret;

	ret = of_parse_phandle_with_fixed_args(port->dev->of_node, "adi,pint", 1, 0,
		&args);
	if (ret) {
		dev_err(port->dev, "Missing or invalid adi,pint connection for %pOFn; "
			"attach a pint instance with one argument for port assignment\n",
			port->dev->of_node);
		return ret;
	}

	pint_node = args.np;

	pint_pdev = of_find_device_by_node(pint_node);
	if (!pint_pdev) {
		ret = -EPROBE_DEFER;
		goto cleanup;
	}

	pint = dev_get_drvdata(&pint_pdev->dev);
	if (!pint) {
		ret = -EPROBE_DEFER;
		goto cleanup;
	}

	port->irq_domain = pint->domain;

	if (args.args[0])
		port->irq_offset = 16;
	else
		port->irq_offset = 0;

cleanup:
	of_node_put(pint_node);
	return ret;
}

static void adsp_pint_dispatch_irq(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct adsp_pint *pint = to_adsp_pint(chip);
	unsigned int type = irqd_get_trigger_type(&desc->irq_data);
	u32 pos = BIT(desc->irq_data.hwirq);

	/* for both edge interrupt, toggle invert bit to catch next edge */
	if (type == IRQ_TYPE_EDGE_BOTH) {
		u32 invert = readl(pint->regs + ADSP_PINT_REG_INVERT_SET) & pos;

		if (invert)
			writel(pos, pint->regs + ADSP_PINT_REG_INVERT_CLEAR);
		else
			writel(pos, pint->regs + ADSP_PINT_REG_INVERT_SET);
	}

	writel(pos, pint->regs + ADSP_PINT_REG_REQUEST);

	/* either edge is set */
	if (type & IRQ_TYPE_EDGE_BOTH)
		handle_edge_irq(desc);
	else
		handle_level_irq(desc);
}

static int adsp_pint_irq_map(struct irq_domain *domain, unsigned int irq,
	irq_hw_number_t hwirq)
{
	struct adsp_pint *pint = domain->host_data;

	irq_set_chip_data(irq, pint);
	irq_set_chip_and_handler(irq, &pint->chip, adsp_pint_dispatch_irq);
	return 0;
}

static const struct irq_domain_ops adsp_irq_domain_ops = {
	.map = adsp_pint_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

/**
 * This handles the GIC interrupt associated with this PINT being activated.
 * It chains the interrupt associated with a particular pin
 */
static void adsp_pint_irq_handler(struct irq_desc *desc)
{
	struct adsp_pint *pint = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long req;
	int pos;

	chained_irq_enter(chip, desc);

	req = readl(pint->regs + ADSP_PINT_REG_REQUEST);

	for_each_set_bit(pos, &req, 32) {
		unsigned int virq = irq_find_mapping(pint->domain, pos);

		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(chip, desc);
}

static void adsp_pint_irq_ack(struct irq_data *d)
{
	/* this is required for edge type irqs unconditionally */
}

static void adsp_pint_irq_mask(struct irq_data *d)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);

	writel(BIT(d->hwirq), pint->regs + ADSP_PINT_REG_MASK_CLEAR);
}

static void adsp_pint_irq_unmask(struct irq_data *d)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);

	writel(BIT(d->hwirq), pint->regs + ADSP_PINT_REG_MASK_SET);
}

static int adsp_pint_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct adsp_pint *pint = irq_data_get_irq_chip_data(d);
	unsigned int pos = BIT(d->hwirq);

	switch (type) {
	case IRQ_TYPE_PROBE:
		type = IRQ_TYPE_EDGE_BOTH;
		fallthrough;
	case IRQ_TYPE_EDGE_BOTH:
		/* start by looking for rising edge */
		writel(pos, pint->regs + ADSP_PINT_REG_INVERT_CLEAR);
		writel(pos, pint->regs + ADSP_PINT_REG_EDGE_SET);
		break;

	case IRQ_TYPE_EDGE_FALLING:
		writel(pos, pint->regs + ADSP_PINT_REG_INVERT_SET);
		writel(pos, pint->regs + ADSP_PINT_REG_EDGE_SET);
		break;

	case IRQ_TYPE_EDGE_RISING:
		writel(pos, pint->regs + ADSP_PINT_REG_INVERT_CLEAR);
		writel(pos, pint->regs + ADSP_PINT_REG_EDGE_SET);
		break;

	case IRQ_TYPE_LEVEL_HIGH:
		writel(pos, pint->regs + ADSP_PINT_REG_INVERT_CLEAR);
		writel(pos, pint->regs + ADSP_PINT_REG_EDGE_CLEAR);
		break;

	case IRQ_TYPE_LEVEL_LOW:
		writel(pos, pint->regs + ADSP_PINT_REG_INVERT_SET);
		writel(pos, pint->regs + ADSP_PINT_REG_EDGE_CLEAR);
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static int adsp_pint_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adsp_pint *pint;
	struct resource *res;

	pint = devm_kzalloc(dev, sizeof(*pint), GFP_KERNEL);
	if (!pint)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	pint->regs = devm_ioremap_resource(dev, res);
	if (IS_ERR(pint->regs)) {
		dev_err(dev, "Could not find address range for interrupt controller\n");
		return PTR_ERR(pint->regs);
	}

	pint->chip.name = "adsp-pint";
	pint->chip.irq_ack = adsp_pint_irq_ack;
	pint->chip.irq_mask = adsp_pint_irq_mask;
	pint->chip.irq_unmask = adsp_pint_irq_unmask;
	pint->chip.irq_set_type = adsp_pint_irq_set_type;
	// @todo potentially only SEC supports wake options, not gic

	// @todo determine if we actually need a raw spinlock

	pint->domain = irq_domain_add_linear(np, ADSP_PINT_IRQS,
		&adsp_irq_domain_ops, pint);
	if (!pint->domain) {
		dev_err(dev, "Could not create irq domain\n");
		return -EINVAL;
	}

	pint->irq = platform_get_irq(pdev, 0);
	if (!pint->irq) {
		dev_err(dev, "Could not find parent interrupt for port\n");
		return -EINVAL;
	}

	irq_set_chained_handler_and_data(pint->irq, adsp_pint_irq_handler, pint);
	platform_set_drvdata(pdev, pint);

	return 0;
}

static void adsp_pint_remove(struct platform_device *pdev)
{
	struct adsp_pint *pint = platform_get_drvdata(pdev);

	irq_set_chained_handler_and_data(pint->irq, NULL, NULL);
	irq_domain_remove(pint->domain);
}

static const struct of_device_id adsp_pint_of_match[] = {
	{ .compatible = "adi,adsp-pint" },
	{ }
};
MODULE_DEVICE_TABLE(of, adsp_pint_of_match);

static struct platform_driver adsp_pint_driver = {
	.driver = {
		.name = "adsp-port-pint",
		.of_match_table = adsp_pint_of_match,
	},
	.probe = adsp_pint_probe,
	.remove = adsp_pint_remove,
};

static int __init adsp_pint_init(void)
{
	return platform_driver_register(&adsp_pint_driver);
}

arch_initcall(adsp_pint_init);
