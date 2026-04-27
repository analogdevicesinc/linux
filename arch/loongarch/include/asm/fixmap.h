/* SPDX-License-Identifier: GPL-2.0 */
/*
 * fixmap.h: compile-time virtual memory allocation
 *
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 */

#ifndef _ASM_FIXMAP_H
#define _ASM_FIXMAP_H

#ifdef CONFIG_HIGHMEM
#include <linux/threads.h>
#include <asm/kmap_size.h>
#endif

#define NR_FIX_BTMAPS 64

enum fixed_addresses {
	FIX_HOLE,
#ifdef CONFIG_HIGHMEM
	FIX_KMAP_BEGIN,
	FIX_KMAP_END = FIX_KMAP_BEGIN + (KM_MAX_IDX * NR_CPUS) - 1,
#endif
	FIX_EARLYCON_MEM_BASE,
	__end_of_fixed_addresses
};

#define FIXADDR_SIZE	(__end_of_fixed_addresses << PAGE_SHIFT)
#define FIXADDR_START	(FIXADDR_TOP - FIXADDR_SIZE)
#define FIXMAP_PAGE_IO	PAGE_KERNEL_SUC

extern void __set_fixmap(enum fixed_addresses idx,
			 phys_addr_t phys, pgprot_t flags);

#include <asm-generic/fixmap.h>

/*
 * Called from pagetable_init()
 */
extern void fixrange_init(unsigned long start, unsigned long end, pgd_t *pgd_base);

#endif
