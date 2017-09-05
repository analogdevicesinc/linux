/*
 * Copyright 2017 NXP
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/irqdomain.h>
#include <linux/of_platform.h>
#include <linux/spinlock.h>

#define CHANCSR(n)	(0x0 + 0x40 * n)
#define CHANVEC(n)	(0x4 + 0x40 * n)
#define CHANIER(n)	(0x10 + (0x40 * n))
#define CHANIPR(n)	(0x20 + (0x40 * n))

struct intmux_irqchip_data {
	int chanidx;
	int irq;
	struct irq_domain *domain;
	unsigned int irqstat;
};


struct intmux_data {
	spinlock_t lock;
	struct platform_device	*pdev;
	void __iomem *regs;
	struct clk *ipg_clk;
	int channum;
	struct intmux_irqchip_data irqchip_data[];
};

static void imx_intmux_irq_unmask(struct irq_data *d)
{
	struct intmux_irqchip_data *irqchip_data = d->chip_data;
	u32 idx = irqchip_data->chanidx;
	struct intmux_data *intmux_data = container_of(irqchip_data, struct intmux_data, irqchip_data[idx]);
	void __iomem *reg;
	u32 val;

	spin_lock(&intmux_data->lock);
	reg = intmux_data->regs + CHANIER(idx);
	val = readl_relaxed(reg);
	val |= 1 << d->hwirq;
	writel_relaxed(val, reg);
	spin_unlock(&intmux_data->lock);
}

static void imx_intmux_irq_mask(struct irq_data *d)
{
	struct intmux_irqchip_data *irqchip_data = d->chip_data;
	u32 idx = irqchip_data->chanidx;
	struct intmux_data *intmux_data = container_of(irqchip_data, struct intmux_data, irqchip_data[idx]);
	void __iomem *reg;
	u32 val;

	spin_lock(&intmux_data->lock);
	reg = intmux_data->regs + CHANIER(idx);
	val = readl_relaxed(reg);
	val &= ~(1 << d->hwirq);
	writel_relaxed(val, reg);
	spin_unlock(&intmux_data->lock);
}

static void imx_intmux_irq_ack(struct irq_data *d)
{
	/* the irqchip has no ack */
}

static struct irq_chip imx_intmux_irq_chip = {
	.name		= "intmux",
	.irq_eoi	= irq_chip_eoi_parent,
	.irq_mask	= imx_intmux_irq_mask,
	.irq_unmask	= imx_intmux_irq_unmask,
	.irq_ack	= imx_intmux_irq_ack,
};

static int imx_intmux_irq_map(struct irq_domain *h, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, h->host_data);
	irq_set_chip_and_handler(irq, &imx_intmux_irq_chip, handle_edge_irq);

	return 0;
}

static const struct irq_domain_ops imx_intmux_domain_ops = {
	.map		= imx_intmux_irq_map,
	.xlate		= irq_domain_xlate_twocell,
};

static void imx_intmux_update_irqstat(struct intmux_irqchip_data *irqchip_data)
{
	int i = irqchip_data->chanidx;
	struct intmux_data *intmux_data = container_of(irqchip_data, struct intmux_data, irqchip_data[i]);

	irqchip_data->irqstat = readl_relaxed(intmux_data->regs + CHANIPR(i));
}

static void imx_intmux_irq_handler(struct irq_desc *desc)
{
	struct intmux_irqchip_data *irqchip_data = irq_desc_get_handler_data(desc);
	int pos, virq;

	chained_irq_enter(irq_desc_get_chip(desc), desc);

	imx_intmux_update_irqstat(irqchip_data);

	for_each_set_bit(pos, (unsigned long *)&irqchip_data->irqstat, 32) {
		virq = irq_find_mapping(irqchip_data->domain, pos);
		if (virq)
			generic_handle_irq(virq);
	}

	chained_irq_exit(irq_desc_get_chip(desc), desc);
}

static int imx_intmux_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct intmux_data *intmux_data;
	struct resource *res;
	int i;
	int channum;
	int ret;

	ret = of_property_read_u32(np, "nxp,intmux_chans", &channum);
	if (ret)
		channum = 1;

	intmux_data = devm_kzalloc(&pdev->dev, sizeof(*intmux_data) +
				     channum *
				     sizeof(intmux_data->irqchip_data[0]),
				     GFP_KERNEL);
	if (!intmux_data)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	intmux_data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(intmux_data->regs)) {
		dev_err(&pdev->dev, "failed to initialize reg\n");
		return PTR_ERR(intmux_data->regs);
	}

	intmux_data->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(intmux_data->ipg_clk)) {
		ret = PTR_ERR(intmux_data->ipg_clk);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}

	intmux_data->channum = channum;
	intmux_data->pdev = pdev;
	spin_lock_init(&intmux_data->lock);

	ret = clk_prepare_enable(intmux_data->ipg_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable ipg clk: %d\n", ret);
		return ret;
	}

	for (i = 0; i < channum; i++) {
		intmux_data->irqchip_data[i].chanidx = i;
		intmux_data->irqchip_data[i].irq = platform_get_irq(pdev, i);
		if (intmux_data->irqchip_data[i].irq <= 0) {
			dev_err(&pdev->dev, "failed to get irq\n");
			return -ENODEV;
		}

		intmux_data->irqchip_data[i].domain = irq_domain_add_linear(np,
						 32,
						 &imx_intmux_domain_ops,
						 &intmux_data->irqchip_data[i]);
		if (!intmux_data->irqchip_data[i].domain) {
			dev_err(&intmux_data->pdev->dev,
				"failed to create IRQ domain\n");
			return -ENOMEM;
		}

		irq_set_chained_handler_and_data(intmux_data->irqchip_data[i].irq,
					 imx_intmux_irq_handler,
					 &intmux_data->irqchip_data[i]);
	}

	platform_set_drvdata(pdev, intmux_data);

	return 0;
}

static int imx_intmux_remove(struct platform_device *pdev)
{
	struct intmux_data *intmux_data = platform_get_drvdata(pdev);
	int i;

	for (i = 0; i < intmux_data->channum; i++) {
		irq_set_chained_handler_and_data(intmux_data->irqchip_data[i].irq, NULL, NULL);

		irq_domain_remove(intmux_data->irqchip_data[i].domain);
	}

	platform_set_drvdata(pdev, NULL);
	clk_disable_unprepare(intmux_data->ipg_clk);

	return 0;
}

static const struct of_device_id imx_intmux_id[] = {
	{ .compatible = "nxp,imx-intmux", },
	{},
};

static struct platform_driver imx_intmux_driver = {
	.driver = {
		.name = "imx-intmux",
		.of_match_table = imx_intmux_id,
	},
	.probe = imx_intmux_probe,
	.remove = imx_intmux_remove,
};

static int __init irq_imx_intmux_init(void)
{
	return platform_driver_register(&imx_intmux_driver);
}
arch_initcall(irq_imx_intmux_init);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX8 irq steering driver");
MODULE_LICENSE("GPL v2");
