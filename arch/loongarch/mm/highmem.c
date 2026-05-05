// SPDX-License-Identifier: GPL-2.0
#include <linux/init.h>
#include <linux/export.h>
#include <linux/highmem.h>
#include <asm/fixmap.h>
#include <asm/tlbflush.h>

void kmap_flush_tlb(unsigned long addr)
{
	flush_tlb_one(addr);
}
EXPORT_SYMBOL(kmap_flush_tlb);
