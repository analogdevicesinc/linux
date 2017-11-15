// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2013 Freescale Semiconductor, Inc.
 * Copyright 2018 NXP
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/io.h>
#include <linux/uio_driver.h>
#include <linux/slab.h>
#include <linux/list.h>
#include "regs.h"
#include "fsl_jr_uio.h"

static const char jr_uio_version[] = "fsl JR UIO driver v1.0";

#define NAME_LENGTH 30
#define JR_INDEX_OFFSET 12

static const char uio_device_name[] = "fsl-jr";
static LIST_HEAD(jr_list);

struct jr_uio_info {
	atomic_t ref; /* exclusive, only one open() at a time */
	struct uio_info uio;
	char name[NAME_LENGTH];
};

struct jr_dev {
	u32 revision;
	u32 index;
	u32 irq;
	struct caam_job_ring __iomem *global_regs;
	struct device *dev;
	struct resource *res;
	struct jr_uio_info info;
	struct list_head node;
	struct list_head jr_list;
};

static int jr_uio_open(struct uio_info *info, struct inode *inode)
{
	struct jr_uio_info *uio_info = container_of(info,
					struct jr_uio_info, uio);

	if (!atomic_dec_and_test(&uio_info->ref)) {
		pr_err("%s: failing non-exclusive open()\n", uio_info->name);
		atomic_inc(&uio_info->ref);
		return -EBUSY;
	}

	return 0;
}

static int jr_uio_release(struct uio_info *info, struct inode *inode)
{
	struct jr_uio_info *uio_info = container_of(info,
					struct jr_uio_info, uio);
	atomic_inc(&uio_info->ref);

	return 0;
}

static irqreturn_t jr_uio_irq_handler(int irq, struct uio_info *dev_info)
{
	struct jr_dev *jrdev = dev_info->priv;
	u32 irqstate;

	irqstate = rd_reg32(&jrdev->global_regs->jrintstatus);

	if (!irqstate)
		return IRQ_NONE;

	if (irqstate & JRINT_JR_ERROR)
		dev_info(jrdev->dev, "uio job ring error - irqstate: %08x\n",
			 irqstate);

	/*mask valid interrupts */
	clrsetbits_32(&jrdev->global_regs->rconfig_lo, 0, JRCFG_IMSK);

	/* Have valid interrupt at this point, just ACK and trigger */
	wr_reg32(&jrdev->global_regs->jrintstatus, irqstate);

	return IRQ_HANDLED;
}

static int jr_uio_irqcontrol(struct uio_info *dev_info, int irqon)
{
	struct jr_dev *jrdev = dev_info->priv;

	switch (irqon) {
	case SEC_UIO_SIMULATE_IRQ_CMD:
		uio_event_notify(dev_info);
		break;
	case SEC_UIO_ENABLE_IRQ_CMD:
		/* Enable Job Ring interrupt */
		clrsetbits_32(&jrdev->global_regs->rconfig_lo, JRCFG_IMSK, 0);
		break;
	case SEC_UIO_DISABLE_IRQ_CMD:
		/* Disable Job Ring interrupt */
		clrsetbits_32(&jrdev->global_regs->rconfig_lo, 0, JRCFG_IMSK);
		break;
	default:
		break;
	}
	return 0;
}

static int __init jr_uio_init(struct jr_dev *uio_dev)
{
	int ret;
	struct jr_uio_info *info;

	info = &uio_dev->info;
	atomic_set(&info->ref, 1);
	info->uio.version = jr_uio_version;
	info->uio.name = uio_dev->info.name;
	info->uio.mem[0].name = "JR config space";
	info->uio.mem[0].addr = uio_dev->res->start;
	info->uio.mem[0].size = resource_size(uio_dev->res);
	info->uio.mem[0].internal_addr = uio_dev->global_regs;
	info->uio.mem[0].memtype = UIO_MEM_PHYS;
	info->uio.irq = uio_dev->irq;
	info->uio.irq_flags = IRQF_SHARED;
	info->uio.handler = jr_uio_irq_handler;
	info->uio.irqcontrol = jr_uio_irqcontrol;
	info->uio.open = jr_uio_open;
	info->uio.release = jr_uio_release;
	info->uio.priv = uio_dev;

	ret = uio_register_device(uio_dev->dev, &info->uio);
	if (ret) {
		dev_err(uio_dev->dev, "jr_uio: UIO registration failed\n");
		return ret;
	}

	return 0;
}

static const struct of_device_id jr_ids[] = {
	{ .compatible = "fsl,sec-v4.0-job-ring", },
	{ .compatible = "fsl,sec-v4.4-job-ring", },
	{ .compatible = "fsl,sec-v5.0-job-ring", },
	{ .compatible = "fsl,sec-v6.0-job-ring", },
	{},
};

static int fsl_jr_probe(struct platform_device *dev)
{
	struct jr_dev *jr_dev;
	struct device_node *jr_node;
	int ret, count = 0;
	struct list_head *p;

	jr_node = dev->dev.of_node;
	if (!jr_node) {
		dev_err(&dev->dev, "Device OF-Node is NULL\n");
		return -EFAULT;
	}

	jr_dev = devm_kzalloc(&dev->dev, sizeof(*jr_dev), GFP_KERNEL);
	if (!jr_dev)
		return -ENOMEM;

	/* Creat name and index */
	list_for_each(p, &jr_list) {
		count++;
	}
	jr_dev->index = count;

	snprintf(jr_dev->info.name, sizeof(jr_dev->info.name) - 1,
		 "%s%d", uio_device_name, jr_dev->index);

	jr_dev->dev = &dev->dev;
	platform_set_drvdata(dev, jr_dev);

	jr_dev->res = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (unlikely(!jr_dev->res)) {
		dev_err(jr_dev->dev, "platform_get_resource() failed\n");
		ret = -ENOMEM;
		goto abort;
	}

	jr_dev->global_regs =
		devm_ioremap(&dev->dev, jr_dev->res->start,
			     resource_size(jr_dev->res));
	if (unlikely(jr_dev->global_regs == 0)) {
		dev_err(jr_dev->dev, "devm_ioremap failed\n");
		ret = -EIO;
		goto abort;
	}
	jr_dev->irq = irq_of_parse_and_map(jr_node, 0);
	dev_dbg(jr_dev->dev, "errirq: %d\n", jr_dev->irq);

	/* Register UIO */
	ret = jr_uio_init(jr_dev);
	if (ret) {
		dev_err(&dev->dev, "UIO init Failed\n");
		goto abort;
	}

	list_add_tail(&jr_dev->node, &jr_list);

	dev_info(jr_dev->dev, "UIO device full name %s initialized\n",
		 jr_dev->info.name);

	return 0;

abort:
	return ret;
}

static int fsl_jr_remove(struct platform_device *dev)
{
	struct jr_dev *jr_dev = platform_get_drvdata(dev);

	if (!jr_dev)
		return 0;

	list_del(&jr_dev->node);
	uio_unregister_device(&jr_dev->info.uio);

	return 0;
}

MODULE_DEVICE_TABLE(of, jr_ids);

static struct platform_driver fsl_jr_driver = {
	.driver = {
		.name = "fsl-jr-uio",
		.of_match_table = jr_ids,
	},
	.probe = fsl_jr_probe,
	.remove = fsl_jr_remove,
};

module_platform_driver(fsl_jr_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("NXP");
MODULE_DESCRIPTION("FSL SEC UIO Driver");
