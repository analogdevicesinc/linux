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
#include <linux/device-id/x86_cpu.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/printk.h>
#include <linux/types.h>

#include <asm/cpu_device_id.h>
#include <asm/cpufeatures.h>
#include <asm/intel-family.h>
#include <asm/mce.h>
#include <asm/msr.h>
#include <asm/msr-index.h>

/* Machine check bank hardware unit types */
enum bff_bank_type {
	BFF_BANK_NONE = 0,
	BFF_BANK_DCU,
	BFF_BANK_DTLB,
	BFF_BANK_MLC,
	BFF_BANK_CCF,
	BFF_BANK_HSF,
	BFF_BANK_IOCACHE,
};

/*
 * Mapping from machine check bank numbers on Diamond Rapids CPU, that are
 * supported by a bitfix filter, to hardware unit type.
 */
static const enum bff_bank_type dmr_mcbanks[MAX_NR_BANKS] = {
	[1]	= BFF_BANK_DCU,
	[2]	= BFF_BANK_DTLB,
	[3]	= BFF_BANK_MLC,
	[6]	= BFF_BANK_CCF,
	[13]	= BFF_BANK_HSF,
	[17]	= BFF_BANK_IOCACHE
};

static const struct x86_cpu_id bff_cpu_ids[] __initconst = {
	X86_MATCH_VFM(INTEL_DIAMONDRAPIDS_X, dmr_mcbanks),
	{}
};
MODULE_DEVICE_TABLE(x86cpu, bff_cpu_ids);

static const enum bff_bank_type *bff_bank_types;

static int __init bff_init(void)
{
	const struct x86_cpu_id *m;
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

	m = x86_match_cpu(bff_cpu_ids);
	if (!m) {
		pr_info("CPU model not supported by the bitfix filter reset driver\n");
		return -ENODEV;
	}

	bff_bank_types = (const enum bff_bank_type *)m->driver_data;

	return 0;
}

module_init(bff_init);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tony Luck");
MODULE_DESCRIPTION("Intel bitfix filter reset driver");
