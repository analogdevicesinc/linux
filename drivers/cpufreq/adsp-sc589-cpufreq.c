// SPDX-License-Identifier: GPL-2.0-only
/*
 * Analog Devices Inc. ADSP-SC589 CPUFreq Driver
 * 
 * Copyright (C) 2026 Analog Devices Inc.
 *
 * Author: Qasim Ijaz <Qasim.Ijaz@analog.com>
 */

#include <linux/module.h>
#include <linux/cpufreq.h>
#include <linux/init.h>
#include <linux/of_address.h>
#include <linux/iopoll.h>

#define CGU0_STAT_OFFSET 0x08
#define CGU0_DIV_OFFSET	 0x0C

#define CGU0_CLKSALGN	BIT(3) 
#define CGU0_CSEL_MASK	0x1F
#define CGU0_UPDT	BIT(30)

#define STAT_POLL_SLEEP   10
#define STAT_POLL_TIMEOUT 10000 

void __iomem *cgu0_ctl;

static struct cpufreq_frequency_table sc589_freq_table[] = {
	{ .driver_data = 0, .frequency = 450000 },
	{ .driver_data = 1, .frequency = 225000 },
	{ .frequency = CPUFREQ_TABLE_END },
};

static int sc589_init(struct cpufreq_policy *policy)
{
	policy->freq_table = sc589_freq_table;
	policy->cur = 450000;
	return 0;
}

static int sc589_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	return 0;
}

static unsigned int sc589_get(unsigned int cpu)
{
	return 450000;
}

static int sc589_wait_clock_align(void)
{
	u32 value;
	return readl_poll_timeout(cgu0_ctl + CGU0_STAT_OFFSET,
			   value, 
			   !(value & CGU0_CLKSALGN),
			   STAT_POLL_SLEEP,
			   STAT_POLL_TIMEOUT);
}

static int sc589_set_divider(u32 div)
{
	u32 csel_value;

	if (sc589_wait_clock_align()) {
		pr_err("timeout while waiting for clock alignment.\n");
		return -ETIMEDOUT;
	}

	csel_value = readl(cgu0_ctl + CGU0_DIV_OFFSET);
	csel_value &= ~CGU0_CSEL_MASK;
	csel_value |= div;
	csel_value |= CGU0_UPDT;
	writel(csel_value, cgu0_ctl + CGU0_DIV_OFFSET);

	return 0;
}

static int map_cgu_from_dt(void)
{
	struct device_node *dn = of_find_compatible_node(NULL, NULL, "adi,sc58x-clocks");
	if (!dn) {
		pr_err("failed to find clock node in device tree.\n");
		return -ENODEV;
	}

	cgu0_ctl = of_iomap(dn, 0);
	of_node_put(dn);

	if (!cgu0_ctl) {
		pr_err("failed to map CGU0 control register.\n");
		return -ENOMEM;
	}

	return 0;
}

static struct cpufreq_driver sc589_cpufreq_driver = {
	.name = "sc589-cpufreq",
	.init = sc589_init,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sc589_target_index,
	.get = sc589_get,
};

static int __init sc589_cpufreq_init(void)
{
	pr_info("sc589-cpufreq: loading...\n");
	map_cgu_from_dt();
	return cpufreq_register_driver(&sc589_cpufreq_driver);
}
module_init(sc589_cpufreq_init);

static void __exit sc589_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&sc589_cpufreq_driver);
	if (cgu0_ctl)
		iounmap(cgu0_ctl);
	pr_info("sc589-cpufreq: exit...\n");
}
module_exit(sc589_cpufreq_exit);

MODULE_AUTHOR("Qasim Ijaz <Qasim.Ijaz@analog.com>");
MODULE_DESCRIPTION("CPUFreq driver for Analog Devices ADSP-SC589");
MODULE_LICENSE("GPL");
