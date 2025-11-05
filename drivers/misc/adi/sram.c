// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * SRAM driver for ADI processor on-chip memory
 *
 * (C) Copyright 2025 - Analog Devices, Inc.
 *
 * Authors:
 *    Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 *    Greg Malysa <greg.malysa@timesys.com>
 */

#include <linux/module.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_irq.h>
#include <linux/genalloc.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <linux/platform_device.h>

#include "sram.h"

static struct device *sram_dev;
static void __iomem *l2_ctl_vaddr;

static irqreturn_t sram_ecc_err(int irq, void *dev_id)
{
	int status;

	pr_err("SRAM ECC error happened\n");
	status = readl(l2_ctl_vaddr + L2CTL0_STAT_OFFSET);
	pr_err("status 0x%x ctl %x\n", status, readl(l2_ctl_vaddr));

	if (status & 0x1)
		pr_err("Core channel error type:0x%x, addr:0x%x\n",
			readl(l2_ctl_vaddr + L2CTL0_ET0_OFFSET),
			readl(l2_ctl_vaddr + L2CTL0_EADDR0_OFFSET));
	if (status & 0x2)
		pr_err("System channel error type:0x%x, addr:0x%x\n",
			readl(l2_ctl_vaddr + L2CTL0_ET1_OFFSET),
			readl(l2_ctl_vaddr + L2CTL0_EADDR1_OFFSET));

	status = status >> 8;
	if (status)
		pr_err("SRAM Bank%d error, addr:0x%x\n", status,
			readl(l2_ctl_vaddr + L2CTL0_ERRADDR0_OFFSET + status));
	panic("Can't recover from the SRAM ECC error.");

	return IRQ_HANDLED;
}

static int adi_sram_show(struct seq_file *s, void *data)
{
	struct device_node *sram_node;
	struct gen_pool *sram_pool = NULL;
	size_t pool_size = 0, avail = 0, used = 0;
	int index, count = 0;
	const __be32 *sram_pbase;

	count = of_count_phandle_with_args(sram_dev->of_node, "adi,sram", NULL);
	if (!count) {
		pr_err("no adi,sram phandle defined in sram controller\n");
		return -ENODEV;
	}

	for (index = 0; index < count; index++) {
		/* Get the name and addr of the sram pool */
		sram_node = of_parse_phandle(sram_dev->of_node, "adi,sram", index);
		if (!sram_node) {
			pr_err("Unable to parse phandle\n");
			return -ENODEV;
		}

		sram_pbase = of_get_address(sram_node, 0, NULL, NULL);
		if (!sram_pbase) {
			pr_err("Unable to get phandle address\n");
			return -ENODEV;
		}
		seq_printf(s, "%s@%x:\n",
			sram_node->name, be32_to_cpu(*sram_pbase));

		/* Get the sram pool info */
		sram_pool = of_gen_pool_get(sram_dev->of_node, "adi,sram", index);
		if (!sram_pool) {
			pr_err("sram_pool not available\n");
			of_node_put(sram_node);
			return -ENOMEM;
		}

		/* Calculate the sram total/available/used size */
		pool_size = gen_pool_size(sram_pool);
		avail = gen_pool_avail(sram_pool);
		used = pool_size - avail;
		seq_printf(s, "\tTotal size: %lu B\n\tUsed sram: %lu B\n\tAvail sram: %lu B\n",
			pool_size, used, avail);

		of_node_put(sram_node);
	}

	return 0;
}

static int adi_sram_open(struct inode *inode, struct file *file)
{
	return single_open(file, adi_sram_show, NULL);
}

static const struct proc_ops adi_sram_ops = {
	.proc_open		= adi_sram_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release	= single_release,
};

static const struct of_device_id adi_sram_of_match[] = {
	{ .compatible = "adi,sram-controller" },
	{ },
};
MODULE_DEVICE_TABLE(of, adi_sram_of_match);

static int adi_sram_probe(struct platform_device *pdev)
{
	int ret = 0;
	int irq;
	struct proc_dir_entry *d;
	struct device *dev = &pdev->dev;
	struct resource *res;

	sram_dev = &pdev->dev;

	/* create sram proc show data */
	d = proc_create("sraminfo", 0, NULL, &adi_sram_ops);
	if (!d) {
		dev_err(dev, "Cannot create proc sraminfo entry\n");
		return -ENOMEM;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot find L2 CTL address (reg property in device tree)\n");
		ret = -ENOENT;
		goto free_proc;
	}

	l2_ctl_vaddr = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(l2_ctl_vaddr)) {
		dev_err(dev, "Cannot map L2 control address\n");
		ret = -ENOENT;
		goto free_proc;
	}

	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		dev_err(dev, "invalid irq from dts node\n");
		ret = -ENOENT;
		goto free_proc;
	}

	ret = devm_request_threaded_irq(dev, irq, sram_ecc_err, NULL,
		0, "sram-ecc-err", dev);
	if (unlikely(ret < 0)) {
		dev_err(dev, "Fail to request SRAM ECC error interrupt.ret:%d\n", ret);
		ret = -ENOENT;
		goto free_proc;
	}

	/* clear all status bits */
	writel(readl(l2_ctl_vaddr + L2CTL0_STAT_OFFSET),
				(l2_ctl_vaddr + L2CTL0_STAT_OFFSET));

	return 0;

free_proc:
	remove_proc_entry("sraminfo", NULL);
	return ret;
}

static void adi_sram_remove(struct platform_device *pdev)
{
	remove_proc_entry("sraminfo", NULL);
}

static struct platform_driver adi_sram_driver = {
	.probe = adi_sram_probe,
	.remove = adi_sram_remove,
	.driver = {
		.name	= "sram_controller",
		.of_match_table = of_match_ptr(adi_sram_of_match),
	},
};

module_platform_driver(adi_sram_driver);

MODULE_DESCRIPTION("ADI on-chip sram Controller Driver");
MODULE_LICENSE("GPL");
