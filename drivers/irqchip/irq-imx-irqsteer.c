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

#define CHANREG_OFF	(irqsteer_data->channum * 4)
#define CHANCTRL	0x0
#define CHANMASK(n)	(0x4 + 0x4 * (n))
#define CHANSET(n)	(0x4 + (0x4 * (n)) + CHANREG_OFF)
#define CHANSTATUS(n)	(0x4 + (0x4 * (n)) + (CHANREG_OFF * 2))
#define CHAN_MINTDIS	(0x4 + (CHANREG_OFF * 3))
#define CHAN_MASTRSTAT	(CHAN_MINTDIS + 0x4)

struct irqsteer_irqchip_data {
	spinlock_t lock;
	struct platform_device	*pdev;
	void __iomem *regs;
	struct clk *ipg_clk;
	int irq;
	int channum;
	int endian;	/* 0: littel endian; 1: big endian */
	struct irq_domain *domain;
	unsigned int irqstat[];
};

static void imx_irqsteer_irq_unmask(struct irq_data *d)
{
	struct irqsteer_irqchip_data *irqsteer_data = d->chip_data;
	void __iomem *reg;
	u32 val, idx;

	spin_lock(&irqsteer_data->lock);
	idx = irqsteer_data->endian ? (irqsteer_data->channum - d->hwirq / 32 - 1) :
				      d->hwirq / 32;
	reg = irqsteer_data->regs + CHANMASK(idx);
	val = readl_relaxed(reg);
	val |= 1 << (d->hwirq % 32);
	writel_relaxed(val, reg);
	spin_unlock(&irqsteer_data->lock);
}

static void imx_irqsteer_irq_mask(struct irq_data *d)
{
	struct irqsteer_irqchip_data *irqsteer_data = d->chip_data;
	void __iomem *reg;
	u32 val, idx;

	spin_lock(&irqsteer_data->lock);
	idx = d->hwirq / 32;
	reg = irqsteer_data->regs + CHANMASK(idx);
	val = readl_relaxed(reg);
	val &= ~(1 << (d->hwirq % 32));
	writel_relaxed(val, reg);
	spin_unlock(&irqsteer_data->lock);
}

static void imx_irqsteer_irq_ack(struct irq_data *d)
{
	/* the irqchip has no ack */
}

static struct irq_chip imx_irqsteer_irq_chip = {
	.name		= "irqsteer",
	.irq_eoi	= irq_chip_eoi_parent,
	.irq_mask	= imx_irqsteer_irq_mask,
	.irq_unmask	= imx_irqsteer_irq_unmask,
	.irq_ack	= imx_irqsteer_irq_ack,
};

static int imx_irqsteer_irq_map(struct irq_domain *h, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_data(irq, h->host_data);
	irq_set_chip_and_handler(irq, &imx_irqsteer_irq_chip, handle_edge_irq);

	return 0;
}

static const struct irq_domain_ops imx_irqsteer_domain_ops = {
	.map		= imx_irqsteer_irq_map,
	.xlate		= irq_domain_xlate_twocell,
};

static void imx_irqsteer_init(struct irqsteer_irqchip_data *irqsteer_data)
{
	/* enable channel 1 in default */
	writel_relaxed(1, irqsteer_data->regs + CHANCTRL);
}

static void imx_irqsteer_update_irqstat(struct irqsteer_irqchip_data *irqsteer_data)
{
	int i;

	/*
	 * From irq steering doc, there have one mapping:
	 * word[0] bit 31 -> irq 31 ...  word[0] bit 0 -> irq 0
	 * word[1] bit 31 -> irq 63 ...  word[1] bit 0 -> irq 32
	 * ......
	 * word[15] bit 31 -> irq 511 ...  word[15] bit 0 -> irq 480
	 */
	for (i = 0; i < irqsteer_data->channum; i++)
		irqsteer_data->irqstat[i] = readl_relaxed(irqsteer_data->regs +
						CHANSTATUS(irqsteer_data->endian ?
							   (irqsteer_data->channum - i - 1) :
							   i));
}

static void imx_irqsteer_irq_handler(struct irq_desc *desc)
{
	struct irqsteer_irqchip_data *irqsteer_data = irq_desc_get_handler_data(desc);
	unsigned long val;
	int irqnum;
	int pos, virq;

	chained_irq_enter(irq_desc_get_chip(desc), desc);

	val = readl_relaxed(irqsteer_data->regs + CHAN_MASTRSTAT);
	if (!val)
		goto out;

	imx_irqsteer_update_irqstat(irqsteer_data);

	irqnum = irqsteer_data->channum * 32;
	for_each_set_bit(pos, (unsigned long *)irqsteer_data->irqstat, irqnum) {
		virq = irq_find_mapping(irqsteer_data->domain, pos);
		if (virq)
			generic_handle_irq(virq);
	}

out:
	chained_irq_exit(irq_desc_get_chip(desc), desc);
}

static int imx_irqsteer_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct irqsteer_irqchip_data *irqsteer_data;
	struct resource *res;
	int channum, endian;
	int ret;

	ret = of_property_read_u32(np, "nxp,irqsteer_chans", &channum);
	if (ret)
		channum = 1;

	ret = of_property_read_u32(np, "nxp,endian", &endian);
	if (ret)
		/* default is LSB */
		endian = 0;

	irqsteer_data = devm_kzalloc(&pdev->dev, sizeof(*irqsteer_data) +
				     channum *
				     sizeof(irqsteer_data->irqstat[0]),
				     GFP_KERNEL);
	if (!irqsteer_data)
		return -ENOMEM;


	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irqsteer_data->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(irqsteer_data->regs)) {
		dev_err(&pdev->dev, "failed to initialize reg\n");
		return PTR_ERR(irqsteer_data->regs);
	}

	irqsteer_data->irq = platform_get_irq(pdev, 0);
	if (irqsteer_data->irq <= 0) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return -ENODEV;
	}

	irqsteer_data->ipg_clk = devm_clk_get(&pdev->dev, "ipg");
	if (IS_ERR(irqsteer_data->ipg_clk)) {
		ret = PTR_ERR(irqsteer_data->ipg_clk);
		dev_err(&pdev->dev, "failed to get ipg clk: %d\n", ret);
		return ret;
	}

	irqsteer_data->channum = channum;
	irqsteer_data->endian  = endian;
	irqsteer_data->pdev = pdev;
	spin_lock_init(&irqsteer_data->lock);

	ret = clk_prepare_enable(irqsteer_data->ipg_clk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable ipg clk: %d\n", ret);
		return ret;
	}

	imx_irqsteer_init(irqsteer_data);

	irqsteer_data->domain = irq_domain_add_linear(np,
						 irqsteer_data->channum * 32,
						 &imx_irqsteer_domain_ops,
						 irqsteer_data);
	if (!irqsteer_data->domain) {
		dev_err(&irqsteer_data->pdev->dev,
			"failed to create IRQ domain\n");
		return -ENOMEM;
	}

	irq_set_chained_handler_and_data(irqsteer_data->irq,
					 imx_irqsteer_irq_handler,
					 irqsteer_data);

	platform_set_drvdata(pdev, irqsteer_data);

	return 0;
}

static int imx_irqsteer_remove(struct platform_device *pdev)
{
	struct irqsteer_irqchip_data *irqsteer_data = platform_get_drvdata(pdev);

	irq_set_chained_handler_and_data(irqsteer_data->irq, NULL, NULL);

	irq_domain_remove(irqsteer_data->domain);

	platform_set_drvdata(pdev, NULL);
	clk_disable_unprepare(irqsteer_data->ipg_clk);

	return 0;
}

static const struct of_device_id imx_irqsteer_id[] = {
	{ .compatible = "nxp,imx-irqsteer", },
	{},
};

static struct platform_driver imx_irqsteer_driver = {
	.driver = {
		.name = "imx-irqsteer",
		.of_match_table = imx_irqsteer_id,
	},
	.probe = imx_irqsteer_probe,
	.remove = imx_irqsteer_remove,
};

static int __init irq_imx_irqsteer_init(void)
{
	return platform_driver_register(&imx_irqsteer_driver);
}
arch_initcall(irq_imx_irqsteer_init);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("NXP i.MX8 irq steering driver");
MODULE_LICENSE("GPL v2");
