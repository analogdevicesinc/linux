// SPDX-License-Identifier: GPL-2.0-only
/* Copyright(c) 2026 Intel Corporation. */

/*
 * Intel driver to reset bitfix filters when they overflow.
 *
 * Each bitfix filter has limited slots to track corrected errors.
 * These slots can be filled by transient corrected errors (e.g.,
 * bit flips from particle strikes) that don't represent permanent
 * hardware defects.
 *
 * When the filter overflows (yellow status), reset it to reclaim
 * slots occupied by transient corrected errors. If the overflow
 * repeats frequently after reset, it indicates persistent hardware
 * defects that need attention: the system should be scheduled for
 * servicing.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufeature.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>

#include <asm/cpufeatures.h>
#include <asm/mce.h>
#include <asm/msr.h>
#include <asm/msr-index.h>

static int __init bff_init(void)
{
	u64 core_caps, mcg_cap;

	if (!cpu_feature_enabled(X86_FEATURE_MCA))
		return -ENODEV;
	rdmsrq(MSR_IA32_MCG_CAP, mcg_cap);
	if (!(mcg_cap & MCG_TES_P))
		return -ENODEV;

	if (!cpu_feature_enabled(X86_FEATURE_CORE_CAPABILITIES))
		return -ENODEV;
	rdmsrq(MSR_IA32_CORE_CAPS, core_caps);
	if (!(core_caps & MSR_IA32_CORE_CAPS_BFF_RESET))
		return -ENODEV;

	return 0;
}

module_init(bff_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tony Luck");
MODULE_DESCRIPTION("Intel bitfix filter reset driver");
