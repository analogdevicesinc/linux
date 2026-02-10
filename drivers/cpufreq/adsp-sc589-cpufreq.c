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

#define CGU0_DIV_REG 0x3108D00C
#define GCU0_CONTROL_REG 0x3108D000
#define CGU0_STAT_REG 0x3108D008

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

static struct cpufreq_driver sc589_cpufreq_driver = {
	.name = "sc589-cpufreq",
	.init = sc589_init,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = sc589_target_index,
	.get = sc589_get,
};

static int __init sc589_cpufreq_init(void)
{
	pr_info("sc589-cpufreq loading...\n");
	return cpufreq_register_driver(&sc589_cpufreq_driver);
}
module_init(sc589_cpufreq_init);

static void __exit sc589_cpufreq_exit(void)
{
	pr_info("sc589-cpufreq exit...\n");
	cpufreq_unregister_driver(&sc589_cpufreq_driver);
}
module_exit(sc589_cpufreq_exit);


MODULE_AUTHOR("Qasim Ijaz <Qasim.Ijaz@analog.com>");
MODULE_DESCRIPTION("CPUFreq driver for Analog Devices ADSP-SC589");
MODULE_LICENSE("GPL");
