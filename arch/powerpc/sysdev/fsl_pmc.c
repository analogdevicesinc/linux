// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Suspend/resume support
 *
 * Copyright 2009  MontaVista Software, Inc.
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 */

#include <linux/init.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/suspend.h>
#include <linux/delay.h>
#include <linux/mod_devicetable.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <asm/cacheflush.h>

#include <sysdev/fsl_soc.h>
#include <asm/switch_to.h>
#include <asm/fsl_pm.h>

struct pmc_regs {
	__be32 devdisr;
	__be32 devdisr2;
	__be32 res1;
	__be32 res2;
	__be32 powmgtcsr;
#define POWMGTCSR_SLP		0x00020000
#define POWMGTCSR_DPSLP		0x00100000
#define POWMGTCSR_LOSSLESS	0x00400000
	__be32 res3[2];
	__be32 pmcdr;
};

static struct pmc_regs __iomem *pmc_regs;
static unsigned int pmc_flag;

#define PMC_SLEEP	0x1
#define PMC_DEEP_SLEEP	0x2
#define PMC_LOSSLESS	0x4

/**
 * mpc85xx_pmc_set_wake - enable devices as wakeup event source
 * @dev: a device affected
 * @enable: True to enable event generation; false to disable
 *
 * This enables the device as a wakeup event source, or disables it.
 *
 * RETURN VALUE:
 * 0 is returned on success.
 * -EINVAL is returned if device is not supposed to wake up the system.
 * -ENODEV is returned if PMC is unavailable.
 * Error code depending on the platform is returned if both the platform and
 * the native mechanism fail to enable the generation of wake-up events
 */
int mpc85xx_pmc_set_wake(struct device *dev, bool enable)
{
	int ret = 0;
	struct device_node *clk_np;
	const u32 *prop;
	u32 pmcdr_mask;

	if (!pmc_regs) {
		dev_err(dev, "%s: PMC is unavailable\n", __func__);
		return -ENODEV;
	}

	if (enable && !device_may_wakeup(dev))
		return -EINVAL;

	clk_np = of_parse_phandle(dev->of_node, "fsl,pmc-handle", 0);
	if (!clk_np)
		return -EINVAL;

	prop = of_get_property(clk_np, "fsl,pmcdr-mask", NULL);
	if (!prop) {
		ret = -EINVAL;
		goto out;
	}
	pmcdr_mask = be32_to_cpup(prop);

	if (enable)
		/* clear to enable clock in low power mode */
		clrbits32(&pmc_regs->pmcdr, pmcdr_mask);
	else
		setbits32(&pmc_regs->pmcdr, pmcdr_mask);

out:
	of_node_put(clk_np);
	return ret;
}
EXPORT_SYMBOL_GPL(mpc85xx_pmc_set_wake);

/**
 * mpc85xx_pmc_set_lossless_ethernet - enable lossless ethernet
 * in (deep) sleep mode
 * @enable: True to enable event generation; false to disable
 */
void mpc85xx_pmc_set_lossless_ethernet(int enable)
{
	if (pmc_flag & PMC_LOSSLESS) {
		if (enable)
			setbits32(&pmc_regs->powmgtcsr,	POWMGTCSR_LOSSLESS);
		else
			clrbits32(&pmc_regs->powmgtcsr, POWMGTCSR_LOSSLESS);
	}
}
EXPORT_SYMBOL_GPL(mpc85xx_pmc_set_lossless_ethernet);

static int pmc_suspend_enter(suspend_state_t state)
{
	int ret = 0;
	int result;

	switch (state) {
#ifdef CONFIG_PPC_85xx
	case PM_SUSPEND_MEM:
#ifdef CONFIG_SPE
		enable_kernel_spe();
#endif
#ifdef CONFIG_PPC_FPU
		enable_kernel_fp();
#endif

		pr_debug("%s: Entering deep sleep\n", __func__);

		local_irq_disable();
		mpc85xx_enter_deep_sleep(get_immrbase(), POWMGTCSR_DPSLP);

		pr_debug("%s: Resumed from deep sleep\n", __func__);
		break;
#endif

	case PM_SUSPEND_STANDBY:
		local_irq_disable();
		flush_dcache_L1();

		setbits32(&pmc_regs->powmgtcsr, POWMGTCSR_SLP);
		/* At this point, the CPU is asleep. */

		/* Upon resume, wait for SLP bit to be clear. */
		result = spin_event_timeout(
			(in_be32(&pmc_regs->powmgtcsr) & POWMGTCSR_SLP) == 0,
			10000, 10);
		if (!result) {
			pr_err("%s: timeout waiting for SLP bit "
				"to be cleared\n", __func__);
			ret = -ETIMEDOUT;
		}
		break;

	default:
		ret = -EINVAL;

	}
	return ret;
}

static int pmc_suspend_valid(suspend_state_t state)
{
	set_pm_suspend_state(state);

	if (((pmc_flag & PMC_SLEEP) && (state == PM_SUSPEND_STANDBY)) ||
	    ((pmc_flag & PMC_DEEP_SLEEP) && (state == PM_SUSPEND_MEM)))
		return 1;

	set_pm_suspend_state(PM_SUSPEND_ON);
	return 0;
}

static void pmc_suspend_end(void)
{
	set_pm_suspend_state(PM_SUSPEND_ON);
}

static const struct platform_suspend_ops pmc_suspend_ops = {
	.valid = pmc_suspend_valid,
	.enter = pmc_suspend_enter,
	.end = pmc_suspend_end,
};

static int pmc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;

	pmc_regs = of_iomap(np, 0);
	if (!pmc_regs)
		return -ENOMEM;

	pmc_flag = PMC_SLEEP;
	if (of_device_is_compatible(np, "fsl,mpc8536-pmc"))
		pmc_flag |= PMC_DEEP_SLEEP;

	if (of_device_is_compatible(np, "fsl,p1022-pmc"))
		pmc_flag |= PMC_DEEP_SLEEP | PMC_LOSSLESS;

	suspend_set_ops(&pmc_suspend_ops);
	set_pm_suspend_state(PM_SUSPEND_ON);

	pr_info("Freescale PMC driver\n");
	return 0;
}

static const struct of_device_id pmc_ids[] = {
	{ .compatible = "fsl,mpc8548-pmc", },
	{ .compatible = "fsl,mpc8641d-pmc", },
	{ },
};

static struct platform_driver pmc_driver = {
	.driver = {
		.name = "fsl-pmc",
		.of_match_table = pmc_ids,
	},
	.probe = pmc_probe,
};

builtin_platform_driver(pmc_driver);
