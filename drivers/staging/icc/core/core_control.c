// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ADSP-SC5xx Core Control Driver
 *
 * (C) Copyright 2022 - Analog Devices, Inc.
 *
 * Written and/or maintained by Timesys Corporation
 *
 * Contact: Nathan Barrett-Morrison <nathan.morrison@timesys.com>
 * Contact: Greg Malysa <greg.malysa@timesys.com>
 *
 */

#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/soc/adi/hardware.h>
#include <linux/io.h>
#include <linux/soc/adi/icc.h>

#include "../include/icc.h"

#define VALID_CORE_MIN			1
#define VALID_CORE_MAX			2

struct core_control {
	struct miscdevice mdev;
	struct rcu_reg __iomem *rcu_base;
	void __iomem *icc_base;
};

static void init_icc_l2_queue(struct core_control *cctl, unsigned int coreid)
{
	struct core_control *core_ctrl = cctl;

	switch (coreid) {
	case CCTRL_CORE1:
		/*
		 * To start Core1, all of the shared memory that is related with
		 * Core1 should be zeroed as below:
		 *
		 * Core1 - Core0 shared mem: Base core_ctrl->icc_base
		 *                           Size MSGQ_SIZE
		 * Core1 - Core2 shared mem: Base core_ctrl->icc_base + MSGQ_SIZE
		 *                           Size MSGQ_SIZE
		 */
		memset(core_ctrl->icc_base, 0, 2 * MSGQ_SIZE);
		break;

	case CCTRL_CORE2:
		/*
		 * To start Core2, all of the shared memory that is related with
		 * Core2 should be zeroed as bleow:
		 *
		 * Core2 - Core0 shared mem: Base core_ctrl->icc_base + 2*MSGQ_SIZE
		 *                           Size MSGQ_SIZE
		 * Core2 - Core1 shared mem: Base core_ctrl->icc_base + MSGQ_SIZE
		 *                           Size MSGQ_SIZE
		 */
		memset(core_ctrl->icc_base + MSGQ_SIZE, 0, 2 * MSGQ_SIZE);
		break;

	default:
		pr_err("%s: invalid Core ID:%d\n", __func__, coreid);
		break;
	}
}

void adi_core_start(struct core_control *cctl, unsigned int coreid)
{
	struct core_control *cct = cctl;
	struct rcu_reg __iomem *rcu_base = cct->rcu_base;

	if ((coreid < VALID_CORE_MIN) || (coreid > VALID_CORE_MAX)) {
		pr_err(" %s: invalid Core ID:%d\n", __func__, coreid);
		return;
	}

	/* This fun is used to init the L2 shared mem for the corresponding core */
	init_icc_l2_queue(cct, coreid);

	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, &rcu_base->reg_rcu_crstat);

	if (!(readl(&rcu_base->reg_rcu_crctl) & 1 << coreid)) {
		/* disable the system interface */
		writel(readl(&rcu_base->reg_rcu_sidis) | 1 << (coreid - 1),
		       &rcu_base->reg_rcu_sidis);

		/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
		/* while(readl(&rcu_base->reg_rcu_sistat) & 1 << (coreid-1)!=1); */

		usleep_range(50, 100);

		/* put core in reset */
		writel(readl(&rcu_base->reg_rcu_crctl) | 1 << coreid,
		       &rcu_base->reg_rcu_crctl);

		/* reenable the system interface */
		writel(readl(&rcu_base->reg_rcu_sidis) & ~(1 << (coreid - 1)),
		       &rcu_base->reg_rcu_sidis);

		/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
		/* while(readl(&rcu_base->reg_rcu_sistat) & 1 << (coreid-1)!=0); */

		usleep_range(50, 100);
	}

	/* move core out of reset */
	writel(readl(&rcu_base->reg_rcu_crctl) & ~(1 << coreid),
	       &rcu_base->reg_rcu_crctl);
	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, &rcu_base->reg_rcu_crstat);
	/* notify CCES */
	writel(1 << (18 + coreid), &rcu_base->reg_rcu_msg_set);
}

void adi_core_stop(struct core_control *cctl, unsigned int coreid)
{
	struct core_control *cct = cctl;
	struct rcu_reg __iomem *rcu_base = cct->rcu_base;

	if ((coreid < VALID_CORE_MIN) || (coreid > VALID_CORE_MAX)) {
		pr_err(" %s: invalid Core ID:%d\n", __func__, coreid);
		return;
	}

	if (readl(&rcu_base->reg_rcu_crctl) & 1 << coreid)
		return;

	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, &rcu_base->reg_rcu_crstat);

	/* disable the system interface */
	writel(readl(&rcu_base->reg_rcu_sidis) | 1 << (coreid - 1),
	       &rcu_base->reg_rcu_sidis);

	/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
	/* while (readl(&rcu_base->reg_rcu_sistat) & 1 << (coreid - 1) != 1); */

	usleep_range(50, 100);

	/* put core in reset */
	writel(readl(&rcu_base->reg_rcu_crctl) | 1 << coreid,
	       &rcu_base->reg_rcu_crctl);

	/* reenable the system interface */
	writel(readl(&rcu_base->reg_rcu_sidis) & ~(1 << (coreid - 1)),
	       &rcu_base->reg_rcu_sidis);

	/* Anomaly 36-10-0005: SISTAT doesn't acknowledge set status */
	/* while (readl(&rcu_base->reg_rcu_sistat) & 1 << (coreid - 1) != 0); */

	usleep_range(50, 100);

	/* clear CRSTAT bit for given coreid */
	writel(1 << coreid, &rcu_base->reg_rcu_crstat);
}

void adi_set_svect(struct core_control *cctl,
		   unsigned int coreid, unsigned int svect)
{
	struct core_control *cct = cctl;
	struct rcu_reg __iomem *rcu_base = cct->rcu_base;

	if ((coreid < VALID_CORE_MIN) || (coreid > VALID_CORE_MAX)) {
		pr_err(" %s: invalid Core ID:%d\n", __func__, coreid);
		return;
	}

	if (svect && (coreid == 1))
		writel(svect, &rcu_base->reg_rcu_svect1);
	else if (svect && (coreid == 2))
		writel(svect, &rcu_base->reg_rcu_svect2);
}

static long adi_core_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct core_control *cct = filp->private_data;
	int ret = 0;

	switch (cmd) {
	case CMD_SET_SVECT1:
		adi_set_svect(cct, 1, arg);
		break;

	case CMD_SET_SVECT2:
		adi_set_svect(cct, 2, arg);
		break;

	case CMD_CORE_START:
		adi_core_start(cct, arg);
		break;

	case CMD_CORE_STOP:
		adi_core_stop(cct, arg);
		break;

	default:
		pr_err("%s: invalid CMD\n", __func__);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int adi_core_open(struct inode *inode, struct file *filp)
{
	struct miscdevice *misdev = filp->private_data;
	struct core_control *cct = container_of(misdev, struct core_control, mdev);

	filp->private_data = cct;

	return 0;
}

static const struct file_operations core_fops = {
		.owner          = THIS_MODULE,
		.unlocked_ioctl = adi_core_ioctl,
		.open			= adi_core_open,
		.llseek         = noop_llseek,
};

#ifdef CONFIG_OF
static const struct of_device_id core_ctrl_of_match[] = {
	{ .compatible = "adi,core_ctrl" },
	{},
};

MODULE_DEVICE_TABLE(of, core_ctrl_of_match);
#endif

static int adi_core_ctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *match;
	struct core_control *core_ctrl = NULL;
	struct resource *res;
	struct device_node *icc_node;
	const __be32 *icc_mem;
	phys_addr_t icc_mem_addr;
	u64 icc_mem_size;
	int ret;

	/* malloc the core control device */
	if (core_ctrl) {
		dev_err(&pdev->dev, "Can't register more than one core_ctrl device.\n");
		return -ENOENT;
	}

	core_ctrl = devm_kzalloc(dev, sizeof(*core_ctrl), GFP_KERNEL);
	if (!core_ctrl)
		return -ENOMEM;

	/* get the device information */
	if (!np) {
		dev_err(dev, "No platform device\n");
		return -ENODEV;
	}

	match = of_match_device(of_match_ptr(core_ctrl_of_match), dev);
	if (!match) {
		dev_err(dev, "Of match device error\n");
		return -ENODEV;
	}

	/* get the rcu (reset core unit) control register address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Cannot get IORESOURCE_MEM for core ctrl\n");
		return -ENOENT;
	}

	core_ctrl->rcu_base = devm_ioremap(dev, res->start, resource_size(res));
	if (IS_ERR((void *)core_ctrl->rcu_base)) {
		dev_err(dev, "Cannot map RCU register mem\n");
		return PTR_ERR((void *)core_ctrl->rcu_base);
	}

	/* get and init the icc l2 shared memory address refer to the icc node */
	icc_node = of_parse_phandle(dev->of_node, "adi,icc_mem", 0);
	if (!icc_node) {
		dev_err(dev, "Cannot get the device node of icc memory\n");
		return -ENODEV;
	}

	icc_mem = of_get_address(icc_node, 0, &icc_mem_size, NULL);
	if (!icc_mem) {
		dev_err(dev, "Cannot get icc memory address\n");
		return -EFAULT;
	}

	icc_mem_addr = of_translate_address(icc_node, icc_mem);
	if (icc_mem_addr == OF_BAD_ADDR) {
		dev_err(&pdev->dev, "Bad icc memory address\n");
		return -EFAULT;
	}

	core_ctrl->icc_base = devm_ioremap(dev, icc_mem_addr,
					   (u32)icc_mem_size);
	if (IS_ERR((void *)core_ctrl->icc_base)) {
		dev_err(dev, "Cannot map ICC memory\n");
		return PTR_ERR((void *)core_ctrl->icc_base);
	}

	memset(core_ctrl->icc_base, 0, (u32)icc_mem_size);

	/* filled in the misc device */
	core_ctrl->mdev.minor = MISC_DYNAMIC_MINOR,
	core_ctrl->mdev.name  = "corectrl",
	core_ctrl->mdev.fops  = &core_fops,

	ret = misc_register(&core_ctrl->mdev);
	if (ret) {
		dev_err(dev, "Cannot register core control miscdev\n");
		return ret;
	}

	platform_set_drvdata(pdev, core_ctrl);
	dev_info(dev, "initialized\n");

	return 0;
}

static int adi_core_ctrl_remove(struct platform_device *pdev)
{
	struct core_control *core_ctrl = platform_get_drvdata(pdev);

	if (core_ctrl) {
		misc_deregister(&core_ctrl->mdev);

		if (core_ctrl->icc_base)
			devm_iounmap(&pdev->dev, core_ctrl->icc_base);

		if (core_ctrl->rcu_base)
			devm_iounmap(&pdev->dev, core_ctrl->rcu_base);

		kfree(core_ctrl);
	}

	return 0;
}

static struct platform_driver adi_core_control = {
	.probe     = adi_core_ctrl_probe,
	.remove    = adi_core_ctrl_remove,
	.driver    = {
		.name  = "adi,corectrl",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(core_ctrl_of_match),
#endif
	},
};

module_platform_driver(adi_core_control);

MODULE_DESCRIPTION("ADI SC5xx Core Control Support");
MODULE_LICENSE("GPL v2");
