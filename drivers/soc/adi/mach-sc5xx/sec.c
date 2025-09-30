// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * sc59x SEC
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/printk.h>
#include <linux/spinlock.h>

#include <linux/soc/adi/rcu.h>
#include "sec.h"

struct adi_sec {
	void __iomem *ioaddr;
	struct adi_rcu *rcu;
	struct device *dev;
	int cores;
	spinlock_t lock;
};

void adi_sec_writel(u32 val, struct adi_sec *rcu, int offset)
{
	writel(val, rcu->ioaddr + offset);
}

u32 adi_sec_readl(struct adi_sec *rcu, int offset)
{
	return readl(rcu->ioaddr + offset);
}

void sec_raise_irq(struct adi_sec *sec, unsigned int irq)
{
	unsigned long flags;
	unsigned int sid = irq - 32;

	spin_lock_irqsave(&sec->lock, flags);
	adi_sec_writel(sid, sec, ADI_SEC_REG_RAISE);
	spin_unlock_irqrestore(&sec->lock, flags);
}

EXPORT_SYMBOL(sec_raise_irq);

void sec_enable_ssi(struct adi_sec *sec, unsigned int sid, bool fault,
		    bool source)
{
	unsigned long flags;
	u32 val;
	u32 offset;

	offset = ADI_SEC_REG_SCTL_BASE + 8 * sid;

	spin_lock_irqsave(&sec->lock, flags);
	val = adi_sec_readl(sec, offset);

	if (fault)
		val |= ADI_SEC_SCTL_FAULT_EN;
	else
		val |= ADI_SEC_SCTL_INT_EN;

	if (source)
		val |= ADI_SEC_SCTL_SRC_EN;

	adi_sec_writel(val, sec, offset);
	spin_unlock_irqrestore(&sec->lock, flags);
}

EXPORT_SYMBOL(sec_enable_ssi);

void sec_enable_sci(struct adi_sec *sec, unsigned int coreid)
{
	unsigned long flags;
	u32 val;
	u32 offset;

	if (coreid == 0 || coreid > sec->cores) {
		dev_err(sec->dev, "Invalid core ID given to %s: %d\n",
			__func__, coreid);
		return;
	}

	offset = ADI_SEC_REG_CCTL_BASE + coreid * ADI_SEC_CCTL_SIZE;

	spin_lock_irqsave(&sec->lock, flags);
	val = adi_sec_readl(sec, offset);
	val |= ADI_SEC_CCTL_EN;
	adi_sec_writel(val, sec, offset);
	spin_unlock_irqrestore(&sec->lock, flags);
}

EXPORT_SYMBOL(sec_enable_sci);

void sec_set_ssi_coreid(struct adi_sec *sec, unsigned int sid,
			unsigned int coreid)
{
	unsigned long flags;
	u32 val;
	u32 offset;

	if (coreid == 0 || coreid > sec->cores) {
		dev_err(sec->dev, "Invalid core ID given to %s: %d\n",
			__func__, coreid);
		return;
	}

	offset = ADI_SEC_REG_SCTL_BASE + 8 * sid;

	spin_lock_irqsave(&sec->lock, flags);
	val = adi_sec_readl(sec, offset);
	val &= ~ADI_SEC_SCTL_CTG;
	val |= ((coreid << 24) & ADI_SEC_SCTL_CTG);
	adi_sec_writel(val, sec, offset);
	spin_unlock_irqrestore(&sec->lock, flags);
}

EXPORT_SYMBOL(sec_set_ssi_coreid);

struct adi_sec *get_adi_sec_from_node(struct device *dev)
{
	struct platform_device *sec_pdev;
	struct device_node *sec_node;
	struct adi_sec *ret = NULL;

	sec_node = of_parse_phandle(dev->of_node, "adi,sec", 0);
	if (!sec_node) {
		dev_err(dev, "Missing adi,sec phandle in device tree\n");
		return ERR_PTR(-ENODEV);
	}

	sec_pdev = of_find_device_by_node(sec_node);
	if (!sec_pdev) {
		ret = ERR_PTR(-EPROBE_DEFER);
		goto cleanup;
	}

	ret = dev_get_drvdata(&sec_pdev->dev);

cleanup:
	of_node_put(sec_node);
	return ret;
}

EXPORT_SYMBOL(get_adi_sec_from_node);

void put_adi_sec(struct adi_sec *sec)
{
	put_device(sec->dev);
}

EXPORT_SYMBOL(put_adi_sec);

static int adi_sec_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_sec *adi_sec;
	struct adi_rcu *adi_rcu;
	struct resource *res;
	void __iomem *base;
	int cores;
	int ret = 0;

	adi_sec = devm_kzalloc(dev, sizeof(*adi_sec), GFP_KERNEL);
	if (!adi_sec)
		return -ENOMEM;

	spin_lock_init(&adi_sec->lock);
	dev_set_drvdata(dev, adi_sec);

	adi_rcu = get_adi_rcu_from_node(dev);
	if (IS_ERR(adi_rcu))
		return PTR_ERR(adi_rcu);

	adi_sec->dev = dev;
	adi_sec->rcu = adi_rcu;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		ret = -ENOENT;
		goto free_rcu;
	}

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(base)) {
		ret = PTR_ERR(base);
		goto free_rcu;
	}

	adi_rcu_set_sec(adi_rcu, adi_sec);

	if (of_property_read_u32(np, "adi,sharc-cores", &adi_sec->cores)) {
		dev_warn(dev,
			 "Missing property adi,sharc-cores, default to 0\n");
		adi_sec->cores = 0;
	}

	adi_sec->ioaddr = base;

	/* Disable SYSCD_RESETb and clear RCU reset status */
	adi_rcu_writel(0x00, adi_rcu, ADI_RCU_REG_CTL);
	adi_rcu_writel(0x0f, adi_rcu, ADI_RCU_REG_STAT);

	/* Reset SEC */
	adi_sec_writel(0x02, adi_sec, ADI_SEC_REG_GCTL);
	adi_sec_writel(0x02, adi_sec, ADI_SEC_REG_FCTL);

	/* Initialize each core */
	for (cores = 0; cores < adi_sec->cores; ++cores) {
		adi_sec_writel(0x02, adi_sec,
			       ADI_SEC_REG_CCTL_BASE + (cores +
							1) *
			       ADI_SEC_CCTL_SIZE);
	}
	udelay(100);

	/* Enable SEC fault event */
	adi_sec_writel(0x01, adi_sec, ADI_SEC_REG_GCTL);

	/* ANOMALY 36100004 spurious external fault event occurs when FCTL is
	 * re-programmed when active fault is not cleared
	 */
	adi_sec_writel(0xc0, adi_sec, ADI_SEC_REG_FCTL);
	adi_sec_writel(0xc1, adi_sec, ADI_SEC_REG_FCTL);

	/* Enable SYSCD_RESETb input */
	adi_rcu_writel(0x100, adi_rcu, ADI_RCU_REG_CTL);

#ifdef CONFIG_ADI_WATCHDOG
	/* @todo verify sec watchdog event number, make device tree based */
	sec_enable_ssi(adi_sec, 3, true, true);
#endif

	return 0;

free_rcu:
	put_adi_rcu(adi_rcu);
	return ret;
}

static void adi_sec_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_sec *adi_sec;

	adi_sec = dev_get_drvdata(dev);
	put_adi_rcu(adi_sec->rcu);
}

static const struct of_device_id adi_sec_match[] = {
	{.compatible = "adi,system-event-controller" },
	{ }
};

MODULE_DEVICE_TABLE(of, adi_sec_match);

static struct platform_driver adi_sec_driver = {
	.probe = adi_sec_probe,
	.remove = adi_sec_remove,
	.driver = {
		   .name = "adi-system-event-controller",
		   .of_match_table = of_match_ptr(adi_sec_match)
		    },
};

module_platform_driver(adi_sec_driver);

MODULE_DESCRIPTION("System Event Controller for ADI SC5xx SoCs");
MODULE_LICENSE("GPL v2");
