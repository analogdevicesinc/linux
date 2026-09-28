// SPDX-License-Identifier: GPL-2.0-only
/*
 * Steal time governor driver periodically computes steal time.
 * Based on the thresholds it either reduce/increase the preferred
 * CPUs which can be used by the workload to avoid vCPU preemption
 * to an extent possible in paravirtualized environment.
 *
 * Available with CONFIG_STEAL_GOVERNOR
 *
 * Copyright (C) 2026 IBM
 * Author: Shrikanth Hegde <sshegde@linux.ibm.com>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpuhplock.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/kconfig.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#ifdef CONFIG_XEN
#include <xen/xen.h>
#endif

#if !IS_ENABLED(CONFIG_PREFERRED_CPU)
#error "Steal Governor requires CONFIG_PREFERRED_CPU"
#endif

struct steal_governor {
	ktime_t			time;
	u64			steal;
	unsigned long		delay;
	unsigned int		interval_ms;
	unsigned int		high_threshold;
	unsigned int		low_threshold;
	struct delayed_work	work;
};

static struct steal_governor sg_ctx;

static void restore_preferred_to_active(void)
{
	int cpu;

	guard(cpus_read_lock)();
	for_each_cpu(cpu, cpu_active_mask)
		set_cpu_preferred(cpu, true);
}

static int __init steal_governor_init(void)
{
#ifdef CONFIG_XEN
	if (xen_initial_domain()) {
		pr_err("Cannot load in Xen Dom0 (Host OS). Driver is for guests only.\n");
		return -ENODEV;
	}
#endif

	pr_info("enabled\n");
	return 0;
}

static void __exit steal_governor_exit(void)
{
	restore_preferred_to_active();
	pr_info("disabled\n");
}

module_init(steal_governor_init);
module_exit(steal_governor_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("IBM Corporation");
MODULE_DESCRIPTION("Virtualization Steal Time Governor");
