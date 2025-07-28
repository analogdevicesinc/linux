/*
 * Copyright (C) 2007-2013 Michal Simek <monstr@monstr.eu>
 * Copyright (C) 2012-2013 Xilinx, Inc.
 * Copyright (C) 2007-2009 PetaLogix
 * Copyright (C) 2006 Atmark Techno, Inc.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/clk.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/jump_label.h>
#include <linux/bug.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>

/* No one else should require these constants, so define them locally here. */
#define ISR 0x00			/* Interrupt Status Register */
#define IPR 0x04			/* Interrupt Pending Register */
#define IER 0x08			/* Interrupt Enable Register */
#define IAR 0x0c			/* Interrupt Acknowledge Register */
#define SIE 0x10			/* Set Interrupt Enable bits */
#define CIE 0x14			/* Clear Interrupt Enable bits */
#define IVR 0x18			/* Interrupt Vector Register */
#define MER 0x1c			/* Master Enable Register */

#define MER_ME (1<<0)
#define MER_HIE (1<<1)

#define SPURIOUS_IRQ	(-1U)

static DEFINE_STATIC_KEY_FALSE(xintc_is_be);

struct xintc_irq_chip {
	void		__iomem *base;
	struct		irq_domain *root_domain;
	u32		intr_mask;
	u32		nr_irq;
#ifdef CONFIG_IRQCHIP_XILINX_INTC_MODULE_SUPPORT_EXPERIMENTAL
	int				irq;
#endif
};

static struct xintc_irq_chip *primary_intc;

static void xintc_write(struct xintc_irq_chip *irqc, int reg, u32 data)
{
	if (static_branch_unlikely(&xintc_is_be))
		iowrite32be(data, irqc->base + reg);
	else
		iowrite32(data, irqc->base + reg);
}

static u32 xintc_read(struct xintc_irq_chip *irqc, int reg)
{
	if (static_branch_unlikely(&xintc_is_be))
		return ioread32be(irqc->base + reg);
	else
		return ioread32(irqc->base + reg);
}

static void intc_enable_or_unmask(struct irq_data *d)
{
	struct xintc_irq_chip *irqc = irq_data_get_irq_chip_data(d);
	unsigned long mask = BIT(d->hwirq);

	pr_debug("irq-xilinx: enable_or_unmask: %ld\n", d->hwirq);

	/* ack level irqs because they can't be acked during
	 * ack function since the handle_level_irq function
	 * acks the irq before calling the interrupt handler
	 */
	if (irqd_is_level_type(d))
		xintc_write(irqc, IAR, mask);

	xintc_write(irqc, SIE, mask);
}

static void intc_disable_or_mask(struct irq_data *d)
{
	struct xintc_irq_chip *irqc = irq_data_get_irq_chip_data(d);

	pr_debug("irq-xilinx: disable: %ld\n", d->hwirq);
	xintc_write(irqc, CIE, BIT(d->hwirq));
}

static void intc_ack(struct irq_data *d)
{
	struct xintc_irq_chip *irqc = irq_data_get_irq_chip_data(d);

	pr_debug("irq-xilinx: ack: %ld\n", d->hwirq);
	xintc_write(irqc, IAR, BIT(d->hwirq));
}

static void intc_mask_ack(struct irq_data *d)
{
	struct xintc_irq_chip *irqc = irq_data_get_irq_chip_data(d);
	unsigned long mask = BIT(d->hwirq);

	pr_debug("irq-xilinx: disable_and_ack: %ld\n", d->hwirq);
	xintc_write(irqc, CIE, mask);
	xintc_write(irqc, IAR, mask);
}

static struct irq_chip intc_dev = {
	.name = "Xilinx INTC",
	.irq_unmask = intc_enable_or_unmask,
	.irq_mask = intc_disable_or_mask,
	.irq_ack = intc_ack,
	.irq_mask_ack = intc_mask_ack,
};

static int xintc_map(struct irq_domain *d, unsigned int irq, irq_hw_number_t hw)
{
	struct xintc_irq_chip *irqc = d->host_data;

	if (irqc->intr_mask & BIT(hw)) {
		irq_set_chip_and_handler_name(irq, &intc_dev,
					      handle_edge_irq, "edge");
		irq_clear_status_flags(irq, IRQ_LEVEL);
	} else {
		irq_set_chip_and_handler_name(irq, &intc_dev,
					      handle_level_irq, "level");
		irq_set_status_flags(irq, IRQ_LEVEL);
	}
	irq_set_chip_data(irq, irqc);
	return 0;
}

static const struct irq_domain_ops xintc_irq_domain_ops = {
	.xlate = irq_domain_xlate_onetwocell,
	.map = xintc_map,
};

static void xil_intc_irq_handler(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct xintc_irq_chip *irqc;

	irqc = irq_data_get_irq_handler_data(&desc->irq_data);
	chained_irq_enter(chip, desc);
	do {
		u32 hwirq = xintc_read(irqc, IVR);

		if (hwirq == -1U)
			break;

		generic_handle_domain_irq(irqc->root_domain, hwirq);
	} while (true);
	chained_irq_exit(chip, desc);
}

static void xil_intc_handle_irq(struct pt_regs *regs)
{
	u32 hwirq;

	do {
		hwirq = xintc_read(primary_intc, IVR);
		if (unlikely(hwirq == SPURIOUS_IRQ))
			break;

		generic_handle_domain_irq(primary_intc->root_domain, hwirq);
	} while (true);
}

#ifndef CONFIG_IRQCHIP_XILINX_INTC_MODULE_SUPPORT_EXPERIMENTAL
static int __init xilinx_intc_of_init(struct device_node *intc,
					     struct device_node *parent)
#else
static int xilinx_intc_of_init(struct device_node *intc,
			       struct device_node *parent)
#endif
{
	struct xintc_irq_chip *irqc;
	int ret, irq;

	if (parent) {
		struct platform_device *pdev;
		struct clk *clkin;

		pdev = of_find_device_by_node(intc);
		if (!pdev)
			return -ENODEV;

		clkin = devm_clk_get_optional_enabled(&pdev->dev, NULL);
		if (IS_ERR(clkin)) {
			platform_device_put(pdev);
			return dev_err_probe(&pdev->dev, PTR_ERR(clkin),
					     "Failed to get and enable clock from Device Tree\n");
		}

		irqc = devm_kzalloc(&pdev->dev, sizeof(*irqc), GFP_KERNEL);
		if (!irqc)
			return -ENOMEM;

		irqc->base = devm_of_iomap(&pdev->dev, intc, 0, NULL);
		if (IS_ERR(irqc->base))
			return PTR_ERR(irqc->base);
	} else {
		irqc = kzalloc(sizeof(*irqc), GFP_KERNEL);
		if (!irqc)
			return -ENOMEM;

		irqc->base = of_iomap(intc, 0);
		BUG_ON(!irqc->base);
	}

	ret = of_property_read_u32(intc, "xlnx,num-intr-inputs", &irqc->nr_irq);
	if (ret < 0) {
		pr_err("irq-xilinx: unable to read xlnx,num-intr-inputs\n");
		goto error;
	}

	ret = of_property_read_u32(intc, "xlnx,kind-of-intr", &irqc->intr_mask);
	if (ret < 0) {
		pr_warn("irq-xilinx: unable to read xlnx,kind-of-intr\n");
		irqc->intr_mask = 0;
	}

	if ((u64)irqc->intr_mask >> irqc->nr_irq)
		pr_warn("irq-xilinx: mismatch in kind-of-intr param\n");

	pr_info("irq-xilinx: %pOF: num_irq=%d, edge=0x%x\n",
		intc, irqc->nr_irq, irqc->intr_mask);


	/*
	 * Disable all external interrupts until they are
	 * explicitly requested.
	 */
	xintc_write(irqc, IER, 0);

	/* Acknowledge any pending interrupts just in case. */
	xintc_write(irqc, IAR, 0xffffffff);

	/* Turn on the Master Enable. */
	xintc_write(irqc, MER, MER_HIE | MER_ME);
	if (xintc_read(irqc, MER) != (MER_HIE | MER_ME)) {
		static_branch_enable(&xintc_is_be);
		xintc_write(irqc, MER, MER_HIE | MER_ME);
	}

	irqc->root_domain = irq_domain_add_linear(intc, irqc->nr_irq,
						  &xintc_irq_domain_ops, irqc);
	if (!irqc->root_domain) {
		pr_err("irq-xilinx: Unable to create IRQ domain\n");
		ret = -EINVAL;
		goto error;
	}

	if (parent) {
		irq = irq_of_parse_and_map(intc, 0);
#ifdef CONFIG_IRQCHIP_XILINX_INTC_MODULE_SUPPORT_EXPERIMENTAL
		irqc->irq = irq;
		intc->data = irqc;
#endif
		if (irq) {
			irq_set_chained_handler_and_data(irq,
							 xil_intc_irq_handler,
							 irqc);
		} else {
			pr_err("irq-xilinx: interrupts property not in DT\n");
			ret = -EINVAL;
			goto error;
		}
	} else {
		primary_intc = irqc;
		irq_set_default_host(primary_intc->root_domain);
		set_handle_irq(xil_intc_handle_irq);
	}

	return 0;

error:
	if (!parent) {
		iounmap(irqc->base);
		kfree(irqc);
	}

	return ret;
}

#ifdef CONFIG_IRQCHIP_XILINX_INTC_MODULE_SUPPORT_EXPERIMENTAL

#define INTC_WARN "INTC module will be removed from the Linux platform " \
		  "framework with modules still using it. This can cause " \
		  "unpredictable behavior"

static int xilinx_intc_of_remove(struct device_node *intc,
				 struct device_node *parent)
{
	int irq;
	struct xintc_irq_chip *irqc;

	BUG_ON(!parent);

	irqc = intc->data;
	irq = irqc->irq;

	irq_set_chained_handler_and_data(irq, NULL, NULL);

	if (irqc->root_domain) {
		unsigned int tempirq;
		unsigned int i;

		for (i = 0; i < irqc->root_domain->mapcount; i++) {
			struct irq_desc *desc;
			tempirq = irq_find_mapping(irqc->root_domain, i);
			if (tempirq) {
				desc = irq_to_desc(tempirq);
				if (desc && desc->action) {
					pr_warn("%s\n", INTC_WARN);
					return 0;
				}
			}
		}
		irq_dispose_mapping(irq);
		irq_domain_remove(irqc->root_domain);
	}

	/*
	 * Disable all external interrupts until they are
	 * explicity requested.
	 */
	xintc_write(irqc, IER, 0);
	/* Acknowledge any pending interrupts just in case. */
	xintc_write(irqc, IAR, 0xffffffff);
	/* Turn off the Master Enable. */
	xintc_write(irqc, MER, 0x0);

	return 0;
}

static struct irqc_init_remove_funps intc_funps = {
	.irqchip_initp = xilinx_intc_of_init,
	.irqchip_removep = xilinx_intc_of_remove,
};

IRQCHIP_PLATFORM_DRIVER_BEGIN(xilinx_intc_xps)
IRQCHIP_MATCH("xlnx,xps-intc-1.00.a", &intc_funps)
IRQCHIP_PLATFORM_DRIVER_END(xilinx_intc_xps)

IRQCHIP_PLATFORM_DRIVER_BEGIN(xilinx_intc_opb)
IRQCHIP_MATCH("xlnx,opb-intc-1.00.c", &intc_funps)
IRQCHIP_PLATFORM_DRIVER_END(xilinx_intc_opb)
#else
IRQCHIP_DECLARE(xilinx_intc_xps, "xlnx,xps-intc-1.00.a", xilinx_intc_of_init);
IRQCHIP_DECLARE(xilinx_intc_opb, "xlnx,opb-intc-1.00.c", xilinx_intc_of_init);
#endif
