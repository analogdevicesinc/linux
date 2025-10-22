/* SPDX-License-Identifier: GPL-2.0 */
#ifndef LINUX_CRASH_RESERVE_H
#define LINUX_CRASH_RESERVE_H

#include <linux/linkage.h>
#include <linux/elfcore.h>
#include <linux/elf.h>
#ifdef CONFIG_ARCH_HAS_GENERIC_CRASHKERNEL_RESERVATION
#include <asm/crash_reserve.h>
#endif

/* Location of a reserved region to hold the crash kernel.
 */
extern struct resource crashk_res;
extern struct resource crashk_low_res;
extern struct range crashk_cma_ranges[];
#if defined(CONFIG_CMA) && defined(CONFIG_ARCH_HAS_GENERIC_CRASHKERNEL_RESERVATION)
#define CRASHKERNEL_CMA
#define CRASHKERNEL_CMA_RANGES_MAX 4
extern int crashk_cma_cnt;
#else
#define crashk_cma_cnt 0
#define CRASHKERNEL_CMA_RANGES_MAX 0
#endif


int __init parse_crashkernel(char *cmdline, unsigned long long system_ram,
		unsigned long long *crash_size, unsigned long long *crash_base,
		unsigned long long *low_size, unsigned long long *cma_size,
		bool *high);

void __init reserve_crashkernel_cma(unsigned long long cma_size);

#ifdef CONFIG_ARCH_HAS_GENERIC_CRASHKERNEL_RESERVATION
#ifndef arch_add_crash_res_to_iomem
static inline bool arch_add_crash_res_to_iomem(void)
{
	return true;
}
#endif
#ifndef DEFAULT_CRASH_KERNEL_LOW_SIZE
#define DEFAULT_CRASH_KERNEL_LOW_SIZE	(128UL << 20)
#endif
#ifndef CRASH_ALIGN
#define CRASH_ALIGN			SZ_2M
#endif
#ifndef CRASH_ADDR_LOW_MAX
#define CRASH_ADDR_LOW_MAX		SZ_4G
#endif
#ifndef CRASH_ADDR_HIGH_MAX
#define CRASH_ADDR_HIGH_MAX		memblock_end_of_DRAM()
#endif

void __init reserve_crashkernel_generic(unsigned long long crash_size,
					unsigned long long crash_base,
					unsigned long long crash_low_size,
					bool high);
#else
static inline void __init reserve_crashkernel_generic(
		unsigned long long crash_size,
		unsigned long long crash_base,
		unsigned long long crash_low_size,
		bool high)
{}
#endif
#endif /* LINUX_CRASH_RESERVE_H */
