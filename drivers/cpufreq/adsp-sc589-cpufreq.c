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

#define STAT_POLL_SLEEP    10
#define STAT_POLL_TIMEOUT  10000 

#define CGU0_STAT_OFFSET   0x08
#define CGU0_DIV_OFFSET	   0x0C
#define CGU0_CSEL_MASK	   0x1F

#define CGU0_CLKSALGN	   BIT(3) 
#define CGU0_UPDT	   BIT(30)

#define TRANSITION_LATENCY 50000 /* nanoseconds, TODO: refine with timing/testing */

static void __iomem *cgu0_ctl;
static u32 sys_clkin_khz;

static struct cpufreq_frequency_table sc589_freq_table[] = {
	{ .driver_data = 0, .frequency = 450000 },
	{ .driver_data = 1, .frequency = 225000 },
	{ .frequency = CPUFREQ_TABLE_END },
};

static int sc589_init(struct cpufreq_policy *policy)
{
       	//policy->policy = 
	//policy->governors = 	
	//policy->cpus = 
	policy->cpuinfo.transition_latency = TRANSITION_LATENCY;
	//policy->cpuinfo.min_freq =  
	//policy->cpuinfo.max_freq = 
	//policy->min = 
       	//policy->max = 
	policy->freq_table = sc589_freq_table;
	policy->cur = 450000;
	return 0;
}

static int sc589_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	return 0;
}

// CCLK frequency = (SYS_CLKIN frequency / (DF+1)) * MSEL / CGU_DIV.CSEL

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
		pr_err("sc589-cpufreq: timeout while waiting for clock alignment.\n");
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
	struct device_node *clk_np;
	struct device_node *clkin_np;
	int ret;

	clk_np = of_find_compatible_node(NULL, NULL, "adi,sc58x-clocks");
	if (!clk_np) {
		pr_err("sc589-cpufreq: failed to find clock node in device tree.\n");
		return -ENODEV;
	}

	clkin_np = of_parse_phandle(clk_np, "clocks", 0);
	if (!clkin_np) {
		pr_err("sc589-cpufreq: failed to find sys_clkin0 phandle.\n");
		ret = -ENODEV;
		goto out_put_clk;
	}

	ret = of_property_read_u32(clkin_np, "clock-frequency", &sys_clkin_khz);
	if (ret) {
		pr_err("sc589-cpufreq: failed to read sys_clkin0 frequency.\n");
		goto out_put_clkin;
	}
	sys_clkin_khz /= 1000;

	cgu0_ctl = of_iomap(clk_np, 0);
	if (!cgu0_ctl) {
		pr_err("sc589-cpufreq: failed to map CGU0 control register.\n");
		ret = -ENOMEM;
		goto out_put_clkin;
	}

	ret = 0;

out_put_clkin:
	of_node_put(clkin_np);
out_put_clk:
	of_node_put(clk_np);
	return ret;
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
	int ret;

	pr_info("sc589-cpufreq: loading...\n");
	ret = map_cgu_from_dt();
	if (ret)
		return ret;

	sc589_set_divider(8);
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
