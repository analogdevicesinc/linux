// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Analog Devices Reset Control Unit
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Author: Greg Malysa <greg.malysa@timesys.com>
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/types.h>

#include <linux/soc/adi/rcu.h>
#include "sec.h"

#define ADI_RCU_REBOOT_PRIORITY		255
#define ADI_RCU_CORE_INIT_TIMEOUT	msecs_to_jiffies(2000)

struct adi_rcu {
	struct notifier_block reboot_notifier;
	void __iomem *ioaddr;
	struct device *dev;
	struct adi_sec *sec;
	int sharc_min_coreid;
	int sharc_max_coreid;
};

static struct adi_rcu *to_adi_rcu(struct notifier_block *nb)
{
	return container_of(nb, struct adi_rcu, reboot_notifier);
}

// RCU memory accessors for other drivers that need it
u32 adi_rcu_readl(struct adi_rcu *rcu, int offset)
{
	return readl(rcu->ioaddr + offset);
}

void adi_rcu_writel(u32 val, struct adi_rcu *rcu, int offset)
{
	writel(val, rcu->ioaddr + offset);
}
EXPORT_SYMBOL(adi_rcu_writel);

void adi_rcu_msg_set(struct adi_rcu *rcu, u32 bits)
{
	writel(bits, rcu->ioaddr + ADI_RCU_REG_MSG_SET);
}

void adi_rcu_msg_clear(struct adi_rcu *rcu, u32 bits)
{
	writel(bits, rcu->ioaddr + ADI_RCU_REG_MSG_CLR);
}

// Device tree interface for other modules that need RCU access
struct adi_rcu *get_adi_rcu_from_node(struct device *dev)
{
	struct platform_device *rcu_pdev;
	struct device_node *rcu_node;
	struct adi_rcu *ret = NULL;

	rcu_node = of_parse_phandle(dev->of_node, "adi,rcu", 0);
	if (!rcu_node) {
		dev_err(dev, "Missing adi,rcu phandle in device tree\n");
		return ERR_PTR(-ENODEV);
	}

	rcu_pdev = of_find_device_by_node(rcu_node);
	if (!rcu_pdev) {
		ret = ERR_PTR(-EPROBE_DEFER);
		goto cleanup;
	}

	ret = dev_get_drvdata(&rcu_pdev->dev);
	if (!ret)
		ret = ERR_PTR(-EPROBE_DEFER);

cleanup:
	of_node_put(rcu_node);
	return ret;
}
EXPORT_SYMBOL(get_adi_rcu_from_node);

void put_adi_rcu(struct adi_rcu *rcu)
{
	put_device(rcu->dev);
}
EXPORT_SYMBOL(put_adi_rcu);

void adi_rcu_set_sec(struct adi_rcu *rcu, struct adi_sec *sec)
{
	rcu->sec = sec;
}

// API for other drivers to interact with RCU
int adi_rcu_check_coreid_valid(struct adi_rcu *rcu, int coreid)
{
	if (coreid < rcu->sharc_min_coreid || coreid > rcu->sharc_max_coreid)
		return -EINVAL;
	return 0;
}
EXPORT_SYMBOL(adi_rcu_check_coreid_valid);

int adi_rcu_reset_core(struct adi_rcu *rcu, int coreid)
{
	u32 val;
	int ret;

	ret = adi_rcu_check_coreid_valid(rcu, coreid);
	if (ret)
		return ret;

	// First put core in reset.
    // Clear CRSTAT bit for given coreid.
	adi_rcu_writel(1 << coreid, rcu, ADI_RCU_REG_CRSTAT);

	// Set SIDIS to disable the system interface
	val = adi_rcu_readl(rcu, ADI_RCU_REG_SIDIS);
	adi_rcu_writel(val | (1 << (coreid-1)), rcu, ADI_RCU_REG_SIDIS);

	// Wait for access to coreX have been disabled and all the pending
	// transactions have completed
	udelay(50);

	// Set CRCTL bit to put core in reset
	val = adi_rcu_readl(rcu, ADI_RCU_REG_CRCTL);
	adi_rcu_writel(val | (1 << coreid), rcu, ADI_RCU_REG_CRCTL);

	// Poll until Core is in reset
	while (!(adi_rcu_readl(rcu, ADI_RCU_REG_CRSTAT) & (1 << coreid)))
		;

	// Clear SIDIS to reenable the system interface
	val = adi_rcu_readl(rcu, ADI_RCU_REG_SIDIS);
	adi_rcu_writel(val & ~(1 << (coreid-1)), rcu, ADI_RCU_REG_SIDIS);

	udelay(50);

	// Take Core out of reset
	val = adi_rcu_readl(rcu, ADI_RCU_REG_CRCTL);
	adi_rcu_writel(val & ~(1 << coreid), rcu, ADI_RCU_REG_CRCTL);

	// Wait for done
	udelay(50);

	return 0;
}
EXPORT_SYMBOL(adi_rcu_reset_core);

int adi_rcu_start_core(struct adi_rcu *rcu, int coreid)
{
	int ret;

	ret = adi_rcu_check_coreid_valid(rcu, coreid);
	if (ret)
		return ret;

	// Clear the IDLE bit when start the SHARC core
	adi_rcu_msg_clear(rcu, RCU0_MSG_C0IDLE << coreid);

	// Notify CCES
	adi_rcu_msg_set(rcu, RCU0_MSG_C1ACTIVATE << (coreid-1));

	return 0;
}
EXPORT_SYMBOL(adi_rcu_start_core);

int adi_rcu_is_core_idle(struct adi_rcu *rcu, int coreid)
{
	int ret = adi_rcu_check_coreid_valid(rcu, coreid);

	if (ret)
		return ret;
	return !!(adi_rcu_readl(rcu, ADI_RCU_REG_MSG) & (RCU0_MSG_C0IDLE << coreid));
}
EXPORT_SYMBOL(adi_rcu_is_core_idle);

int adi_rcu_stop_core(struct adi_rcu *rcu, int coreid, int coreirq)
{
	unsigned long timeout = jiffies + ADI_RCU_CORE_INIT_TIMEOUT;
	bool is_timeout = true;
	int ret;

	ret = adi_rcu_check_coreid_valid(rcu, coreid);
	if (ret)
		return ret;

	if (adi_rcu_readl(rcu, ADI_RCU_REG_CRCTL) & (1 << coreid))
		return 0;

	// Check the IDLE bit in RCU_MSG register
	if (adi_rcu_is_core_idle(rcu, coreid) == 0) {
		// Set core reset request bit in RCU_MSG bit(12:14)
		adi_rcu_msg_set(rcu, RCU0_MSG_CRR0 << coreid);

		// Raise SOFT IRQ through SEC
		// DSP enter into ISR to release interrupts used by DSP program
		sec_set_ssi_coreid(rcu->sec, coreirq, coreid);
		sec_enable_ssi(rcu->sec, coreirq, false, true);
		sec_enable_sci(rcu->sec, coreid);
		sec_raise_irq(rcu->sec, coreirq);
	}

	// Wait until the specific core enter into IDLE bit(8:10)
	// DSP should set the IDLE bit to 1 manully in ISR
	do {
		if (adi_rcu_is_core_idle(rcu, coreid)) {
			is_timeout = false;
			break;
		}
	} while (time_before(jiffies, timeout));

	if (is_timeout)
		dev_warn(rcu->dev, "Timeout waiting for remote core %d to IDLE!\n", coreid);

	// Clear core reset request bit in RCU_MSG bit(12:14)
	adi_rcu_msg_clear(rcu, RCU0_MSG_CRR0 << coreid);

	// Clear Activate bit when stop SHARC core
	adi_rcu_msg_clear(rcu, RCU0_MSG_C1ACTIVATE << (coreid-1));
	return 0;
}
EXPORT_SYMBOL(adi_rcu_stop_core);

static int adi_rcu_reboot(struct notifier_block *nb, unsigned long mode, void *cmd)
{
	struct adi_rcu *adi_rcu = to_adi_rcu(nb);
	u32 val;

	dev_info(adi_rcu->dev, "Reboot requested\n");

	val = adi_rcu_readl(adi_rcu, ADI_RCU_REG_CTL);
	adi_rcu_writel(val | ADI_RCU_CTL_SYSRST, adi_rcu, ADI_RCU_REG_CTL);

	dev_err(adi_rcu->dev, "Unable to reboot via RCU\n");
	return NOTIFY_DONE;
}

static int adi_rcu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct adi_rcu *adi_rcu = NULL;
	struct resource *res;
	void __iomem *base;
	int ret;

	adi_rcu = devm_kzalloc(dev, sizeof(*adi_rcu), GFP_KERNEL);
	if (!adi_rcu)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot get RCU base address\n");
		return -ENODEV;
	}

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR(base)) {
		dev_err(dev, "Cannot map RCU base address\n");
		return PTR_ERR(base);
	}

	if (of_property_read_u32(np, "adi,sharc-min", &adi_rcu->sharc_min_coreid))
		adi_rcu->sharc_min_coreid = 1;

	if (of_property_read_u32(np, "adi,sharc-max", &adi_rcu->sharc_max_coreid))
		adi_rcu->sharc_max_coreid = 2;

	adi_rcu->ioaddr = base;
	adi_rcu->dev = dev;
	adi_rcu->reboot_notifier.priority = ADI_RCU_REBOOT_PRIORITY;
	adi_rcu->reboot_notifier.notifier_call = adi_rcu_reboot;
	if (of_property_read_bool(np, "adi,enable-reboot")) {
		ret = register_restart_handler(&adi_rcu->reboot_notifier);
		if (ret) {
			dev_err(dev, "Unable to register restart handler: %d\n", ret);
			return ret;
		}
	}

	dev_set_drvdata(dev, adi_rcu);

	return 0;
}

static int adi_rcu_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_rcu *adi_rcu = dev_get_drvdata(dev);

	unregister_restart_handler(&adi_rcu->reboot_notifier);
	return 0;
}

static const struct of_device_id adi_rcu_match[] = {
	{ .compatible = "adi,reset-controller" },
	{}
};
MODULE_DEVICE_TABLE(of, adi_rcu_match);

static struct platform_driver adi_rcu_driver = {
	.probe = adi_rcu_probe,
	.remove = adi_rcu_remove,
	.driver = {
		.name = "ADI Reset Control Unit",
		.of_match_table = of_match_ptr(adi_rcu_match),
	},
};
module_platform_driver(adi_rcu_driver);

MODULE_DESCRIPTION("Analog Devices RCU driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Malysa <greg.malysa@timesys.com>");
