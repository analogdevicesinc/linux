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

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/irqdomain.h>
#include <linux/irq.h>
#include <linux/irqchip.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/bug.h>
#include <linux/slab.h>

#define DRIVER_NAME "xilinx_intc"

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

#define MAX_IRQ 32

struct xilinx_intc_info {
	u32 intr_mask;
	u32 nr_irq;
	int irq_parent;
	struct irq_domain *domain;
	struct device *dev;
	void __iomem *baseaddr;
	unsigned int irq_base;
};

#define info_to_pdev(info)	container_of(info->dev, struct platform_device, dev)
#define info_to_of(info) (info->dev.of_node)

static unsigned int (*read_fn)(void __iomem *);
static void (*write_fn)(u32, void __iomem *);

static void intc_write32(u32 val, void __iomem *addr)
{
	iowrite32(val, addr);
}

static unsigned int intc_read32(void __iomem *addr)
{
	return ioread32(addr);
}

static void intc_write32_be(u32 val, void __iomem *addr)
{
	iowrite32be(val, addr);
}

static unsigned int intc_read32_be(void __iomem *addr)
{
	return ioread32be(addr);
}

static void intc_enable_or_unmask(struct irq_data *d)
{
	unsigned long mask = 1 << d->hwirq;
	struct xilinx_intc_info *info = d->chip_data;

	dev_dbg(info->dev, "enable_or_unmask: %ld\n", d->hwirq);

	/* ack level irqs because they can't be acked during
	 * ack function since the handle_level_irq function
	 * acks the irq before calling the interrupt handler
	 */
	if (irqd_is_level_type(d))
		write_fn(mask, info->baseaddr + IAR);

	write_fn(mask, info->baseaddr + SIE);
}

static void intc_disable_or_mask(struct irq_data *d)
{
	struct xilinx_intc_info *info = d->chip_data;

	dev_dbg(info->dev, "disable: %ld\n", d->hwirq);
	write_fn(1 << d->hwirq, info->baseaddr + CIE);
}

static void intc_ack(struct irq_data *d)
{
	struct xilinx_intc_info *info = d->chip_data;

	dev_dbg(info->dev, "ack: %ld\n", d->hwirq);
	write_fn(1 << d->hwirq, info->baseaddr + IAR);
}

static void intc_mask_ack(struct irq_data *d)
{
	unsigned long mask = 1 << d->hwirq;
	struct xilinx_intc_info *info = d->chip_data;

	dev_dbg(info->dev, "disable_and_ack: %ld\n", d->hwirq);
	write_fn(mask, info->baseaddr + CIE);
	write_fn(mask, info->baseaddr + IAR);
}

static void xilinx_intc_irq_handler(struct irq_desc *desc)
{

	struct xilinx_intc_info *info = irq_desc_get_handler_data(desc);
	struct irq_chip *irqchip = irq_desc_get_chip(desc);
	unsigned int hwirq, irq = -1;

	chained_irq_enter(irqchip, desc);

	while(1){
		hwirq = read_fn(info->baseaddr + IVR);

		if (hwirq != -1U){
			irq = irq_find_mapping(info->domain, hwirq);
			generic_handle_irq(irq);
		} else {
			break;
		}
	}

	dev_dbg(info->dev, "get_irq: hwirq=%d, irq=%d\n", hwirq, irq);

	chained_irq_exit(irqchip, desc);

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
	struct xilinx_intc_info *info = d->host_data;

	if (info->intr_mask & (1 << hw)) {
		irq_set_chip_and_handler_name(irq, &intc_dev,
						handle_edge_irq, "edge");
		irq_clear_status_flags(irq, IRQ_LEVEL);
	} else {
		irq_set_chip_and_handler_name(irq, &intc_dev,
						handle_level_irq, "level");
		irq_set_status_flags(irq, IRQ_LEVEL);
	}
	return 0;
}

static const struct irq_domain_ops xintc_irq_domain_ops = {
	.xlate = irq_domain_xlate_onecell,
	.map = xintc_map,
};

static int xilinx_intc_probe(struct platform_device *pdev)
{
	struct device_node *intc =  pdev->dev.of_node;
	struct xilinx_intc_info *info;
	struct resource res;
	unsigned int irq, irq_base;
	int ret;

	info = devm_kzalloc(&pdev->dev, sizeof(struct xilinx_intc_info), GFP_KERNEL);
	if(!info)
		return -ENOMEM;

	if(of_address_to_resource(intc, 0, &res))
		return -ENODEV;
	info->baseaddr = devm_ioremap(&pdev->dev, res.start, resource_size(&res));

	info->dev = &pdev->dev;

	ret = of_property_read_u32(intc, "xlnx,num-intr-inputs", &info->nr_irq);
	if (ret < 0) {
		dev_err(&pdev->dev,"unable to read xlnx,num-intr-inputs\n");
		return ret;
	}

	ret = of_property_read_u32(intc, "xlnx,kind-of-intr", &info->intr_mask);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to read xlnx,kind-of-intr\n");
		return ret;
	}

	if ((info->intr_mask >> (info->nr_irq)))
		dev_warn(&pdev->dev, "mismatch in kind-of-intr param: %X\n", (info->intr_mask >> (info->nr_irq)));

	dev_info(&pdev->dev, "num_irq=%d, edge=0x%x\n", info->nr_irq, info->intr_mask);

	info->irq_parent = platform_get_irq(pdev, 0);
	if(info->irq_parent < 0)
		return info->irq_parent;

	dev_info(&pdev->dev, "Got parent IRQ: %d\n", info->irq_parent);

	write_fn = intc_write32;
	read_fn = intc_read32;

	/*
	 * Disable all external interrupts until they are
	 * explicity requested.
	 */
	write_fn(0, info->baseaddr + IER);

	/* Acknowledge any pending interrupts just in case. */
	write_fn(0xffffffff, info->baseaddr + IAR);

	/* Turn on the Master Enable. */
	write_fn(MER_HIE | MER_ME, info->baseaddr + MER);
	if (!(read_fn(info->baseaddr + MER) & (MER_HIE | MER_ME))) {
		write_fn = intc_write32_be;
		read_fn = intc_read32_be;
		write_fn(MER_HIE | MER_ME, info->baseaddr + MER);
	}

	info->domain = irq_domain_add_linear(intc, info->nr_irq, &xintc_irq_domain_ops,
							info);
	if(!info->domain){
		dev_err(&pdev->dev, "Failed to add IRQ domain\n");
		return -ENODEV;
	}

	for (irq = 0; irq < info->nr_irq; irq++) {
		irq_base = irq_create_mapping(info->domain, irq);
		irq_set_chip_data(irq_base, info);
		irq_set_parent(irq_base, info->irq_parent);
		if (irq == 0)
			info->irq_base = irq_base;
	}

	irq_set_chained_handler_and_data(info->irq_parent,
							 xilinx_intc_irq_handler, info);

    if (intc->data == NULL)
    	intc->data = info;

	return 0;
}

static int xilinx_intc_remove(struct platform_device *pdev)
{
	struct device_node *intc =  pdev->dev.of_node;
	struct xilinx_intc_info *info = intc->data;
	unsigned int irq;

	write_fn(0x0, info->baseaddr + MER);

	irq_set_chained_handler(info->irq_parent, NULL);
	irq_set_handler_data(info->irq_parent, NULL);

	for (irq = 0; irq < info->nr_irq; irq++){
		irq_dispose_mapping(
				irq_find_mapping(info->domain, irq));
	}

	irq_domain_remove(info->domain);

	return 0;
}

static const struct of_device_id xilinx_intc_of_match[] = {
    { .compatible = "xlnx,xps-intc-1.00.a",},
    {},

};
MODULE_DEVICE_TABLE(of, xilinx_intc_of_match);

static struct platform_driver xilinx_intc_driver = {
    .driver = {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = xilinx_intc_of_match,
		},
    .probe = xilinx_intc_probe,
    .remove = xilinx_intc_remove,
};

module_platform_driver(xilinx_intc_driver);

MODULE_AUTHOR("Xilinx, Inc");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_NAME ": XILINX AXI INTC driver");
MODULE_ALIAS(DRIVER_NAME);
