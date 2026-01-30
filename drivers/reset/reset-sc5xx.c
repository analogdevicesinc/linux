// SPDX-License-Identifier: GPL-2.0-only
/*
 * Driver for Analog Devices Reset Control Unit
 *
 * (C) Copyright 2022-2024 - Analog Devices, Inc.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/reboot.h>
#include <linux/types.h>

#include <linux/soc/adi/rcu.h>

#define ADI_RCU_REBOOT_PRIORITY		255
#define ADI_RCU_CORE_INIT_TIMEOUT	msecs_to_jiffies(2000)

struct adi_rcu {
	struct notifier_block reboot_notifier;
	void __iomem *ioaddr;
	struct device *dev;
};

static struct adi_rcu *to_adi_rcu(const struct notifier_block *nb)
{
	return container_of(nb, struct adi_rcu, reboot_notifier);
}

/*
 * RCU memory accessors for other drivers that need it
 */
static u32 adi_rcu_readl(struct adi_rcu *rcu, int offset)
{
	return readl(rcu->ioaddr + offset);
}

static void adi_rcu_writel(u32 val, struct adi_rcu *rcu, int offset)
{
	writel(val, rcu->ioaddr + offset);
}

static int adi_rcu_reboot(struct notifier_block *nb, unsigned long mode,
			  void *cmd)
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
	struct adi_rcu *adi_rcu = NULL;
	void __iomem *base;
	int ret;

	adi_rcu = devm_kzalloc(dev, sizeof(*adi_rcu), GFP_KERNEL);
	if (!adi_rcu)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		dev_err(dev, "Cannot map RCU base address\n");
		return PTR_ERR(base);
	}

	adi_rcu->ioaddr = base;
	adi_rcu->dev = dev;
	adi_rcu->reboot_notifier.priority = ADI_RCU_REBOOT_PRIORITY;
	adi_rcu->reboot_notifier.notifier_call = adi_rcu_reboot;
	ret = devm_register_reboot_notifier(dev, &adi_rcu->reboot_notifier);
	if (ret) {
		dev_err(dev,
			"Unable to register restart handler: %d\n",
			ret);
		return ret;
	}

	dev_set_drvdata(dev, adi_rcu);

	return 0;
}

static const struct of_device_id adi_rcu_match[] = {
	{.compatible = "adi,sc5xx-reset" },
	{ }
};

MODULE_DEVICE_TABLE(of, adi_rcu_match);

static struct platform_driver adi_rcu_driver = {
	.probe = adi_rcu_probe,
	.driver = {
		.name = "ADI Reset Control Unit",
		.of_match_table = of_match_ptr(adi_rcu_match),
	},
};

module_platform_driver(adi_rcu_driver);

MODULE_DESCRIPTION("Analog Devices RCU driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Greg Malysa <malysagreg@gmail.com>");
