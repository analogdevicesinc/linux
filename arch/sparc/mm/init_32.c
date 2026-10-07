// SPDX-License-Identifier: GPL-2.0
/*
 *  linux/arch/sparc/mm/init.c
 *
 *  Copyright (C) 1995 David S. Miller (davem@caip.rutgers.edu)
 *  Copyright (C) 1995 Eddie C. Dost (ecd@skynet.be)
 *  Copyright (C) 1998 Jakub Jelinek (jj@sunsite.mff.cuni.cz)
 *  Copyright (C) 2000 Anton Blanchard (anton@samba.org)
 */

#include <linux/module.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/ptrace.h>
#include <linux/mman.h>
#include <linux/mm.h>
#include <linux/swap.h>
#include <linux/initrd.h>
#include <linux/init.h>
#include <linux/highmem.h>
#include <linux/memblock.h>
#include <linux/pagemap.h>
#include <linux/gfp.h>

#include <asm/sections.h>
#include <asm/page.h>
#include <asm/vaddrs.h>
#include <asm/setup.h>
#include <asm/tlb.h>
#include <asm/prom.h>
#include <asm/leon.h>

#include "mm_32.h"

struct sparc_phys_banks sp_banks[SPARC_PHYS_BANKS+1];

/* Initial ramdisk setup */
extern unsigned int sparc_ramdisk_image;
extern unsigned int sparc_ramdisk_size;

void __init find_ramdisk(unsigned long end_of_phys_memory)
{
#ifdef CONFIG_BLK_DEV_INITRD
	unsigned long size;

	/* Now have to check initial ramdisk, so that it won't pass
	 * the end of memory
	 */
	if (sparc_ramdisk_image) {
		if (sparc_ramdisk_image >= (unsigned long)&_end - 2 * PAGE_SIZE)
			sparc_ramdisk_image -= KERNBASE;
		initrd_start = sparc_ramdisk_image + phys_base;
		initrd_end = initrd_start + sparc_ramdisk_size;
		if (initrd_end > end_of_phys_memory) {
			printk(KERN_CRIT "initrd extends beyond end of memory "
			       "(0x%016lx > 0x%016lx)\ndisabling initrd\n",
			       initrd_end, end_of_phys_memory);
			initrd_start = 0;
		} else {
			/* Reserve the initrd image area. */
			size = initrd_end - initrd_start;
			memblock_reserve(initrd_start, size);

			initrd_start = (initrd_start - phys_base) + PAGE_OFFSET;
			initrd_end = (initrd_end - phys_base) + PAGE_OFFSET;
		}
	}
#endif
}

/*
 * paging_init() sets up the page tables: We call the MMU specific
 * init routine based upon the Sun model type on the Sparc.
 *
 */
void __init paging_init(void)
{
	srmmu_paging_init();
	prom_build_devicetree();
	of_fill_in_cpu_data();
	device_scan();
}

void __init arch_mm_preinit(void)
{
	if (PKMAP_BASE+LAST_PKMAP*PAGE_SIZE >= FIXADDR_START) {
		prom_printf("BUG: fixmap and pkmap areas overlap\n");
		prom_printf("pkbase: 0x%lx pkend: 0x%lx fixstart 0x%lx\n",
		       PKMAP_BASE,
		       (unsigned long)PKMAP_BASE+LAST_PKMAP*PAGE_SIZE,
		       FIXADDR_START);
		prom_printf("Please mail sparclinux@vger.kernel.org.\n");
		prom_halt();
	}
}

void sparc_flush_page_to_ram(struct page *page)
{
	unsigned long vaddr = (unsigned long)page_address(page);

	__flush_page_to_ram(vaddr);
}
EXPORT_SYMBOL(sparc_flush_page_to_ram);

void sparc_flush_folio_to_ram(struct folio *folio)
{
	unsigned long vaddr = (unsigned long)folio_address(folio);
	unsigned int i, nr = folio_nr_pages(folio);

	for (i = 0; i < nr; i++)
		__flush_page_to_ram(vaddr + i * PAGE_SIZE);
}
EXPORT_SYMBOL(sparc_flush_folio_to_ram);

static const pgprot_t protection_map[16] = {
	[VM_NONE]					= PAGE_NONE,
	[VM_READ]					= PAGE_READONLY,
	[VM_WRITE]					= PAGE_COPY,
	[VM_WRITE | VM_READ]				= PAGE_COPY,
	[VM_EXEC]					= PAGE_READONLY,
	[VM_EXEC | VM_READ]				= PAGE_READONLY,
	[VM_EXEC | VM_WRITE]				= PAGE_COPY,
	[VM_EXEC | VM_WRITE | VM_READ]			= PAGE_COPY,
	[VM_SHARED]					= PAGE_NONE,
	[VM_SHARED | VM_READ]				= PAGE_READONLY,
	[VM_SHARED | VM_WRITE]				= PAGE_SHARED,
	[VM_SHARED | VM_WRITE | VM_READ]		= PAGE_SHARED,
	[VM_SHARED | VM_EXEC]				= PAGE_READONLY,
	[VM_SHARED | VM_EXEC | VM_READ]			= PAGE_READONLY,
	[VM_SHARED | VM_EXEC | VM_WRITE]		= PAGE_SHARED,
	[VM_SHARED | VM_EXEC | VM_WRITE | VM_READ]	= PAGE_SHARED
};
DECLARE_VM_GET_PAGE_PROT
