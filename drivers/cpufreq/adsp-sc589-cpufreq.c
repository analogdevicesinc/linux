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

static int __init sc589_cpufreq_init(void)
{

	return 0;
}
module_init(sc589_cpufreq_init);

static void __exit sc589_cpufreq_exit(void)
{

	return;
}
module_exit(sc589_cpufreq_exit);

MODULE_AUTHOR("Qasim Ijaz <Qasim.Ijaz@analog.com>");
MODULE_DESCRIPTION("CPUFreq driver for Analog Devices ADSP-SC589");
MODULE_LICENSE("GPL");
