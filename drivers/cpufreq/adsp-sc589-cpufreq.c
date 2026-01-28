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

static struct cpufreq_frequency_table sc589_freq_table[] = {
	{.flags = 0, .driver_data = 0, .frequency = 0},
	{.flags = 0, .driver_data = 0, .frequency = 0},
	{.flags = 0, .driver_data = 0, .frequency = CPUFREQ_TABLE_END},
};


static int sc589_init(struct cpufreq_policy *policy)
{
        return 0;
}

static int sc589_target_index(struct cpufreq_policy *policy, unsigned int index)
{
	return 0;
}

static unsigned int sc589_get(unsigned int cpu)
{
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
	return cpufreq_register_driver(&sc589_cpufreq_driver);
}
module_init(sc589_cpufreq_init);

static void __exit sc589_cpufreq_exit(void)
{
	cpufreq_unregister_driver(&sc589_cpufreq_driver);
}
module_exit(sc589_cpufreq_exit);


MODULE_AUTHOR("Qasim Ijaz <Qasim.Ijaz@analog.com>");
MODULE_DESCRIPTION("CPUFreq driver for Analog Devices ADSP-SC589");
MODULE_LICENSE("GPL");
