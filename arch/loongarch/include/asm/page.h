/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2020-2022 Loongson Technology Corporation Limited
 */
#ifndef _ASM_PAGE_H
#define _ASM_PAGE_H

#include <linux/const.h>
#include <asm/addrspace.h>

/*
 * PAGE_SHIFT determines the page size
 */
#define PAGE_SHIFT	CONFIG_PAGE_SHIFT
#define PAGE_SIZE	(_AC(1, UL) << PAGE_SHIFT)
#define PAGE_MASK	(~(PAGE_SIZE - 1))

#define HPAGE_SHIFT	(PAGE_SHIFT + PAGE_SHIFT - 3)
#define HPAGE_SIZE	(_AC(1, UL) << HPAGE_SHIFT)
#define HPAGE_MASK	(~(HPAGE_SIZE - 1))
#define HUGETLB_PAGE_ORDER	(HPAGE_SHIFT - PAGE_SHIFT)

#ifndef __ASSEMBLY__

#include <linux/kernel.h>
#include <linux/pfn.h>

/*
 * It's normally defined only for FLATMEM config but it's
 * used in our early mem init code for all memory models.
 * So always define it.
 */
#define ARCH_PFN_OFFSET	PFN_UP(PHYS_OFFSET)

extern void clear_page(void *page);
extern void copy_page(void *to, void *from);

#define clear_user_page(page, vaddr, pg)	clear_page(page)
#define copy_user_page(to, from, vaddr, pg)	copy_page(to, from)

extern unsigned long shm_align_mask;

struct page;
struct vm_area_struct;
void copy_user_highpage(struct page *to, struct page *from,
	      unsigned long vaddr, struct vm_area_struct *vma);

#define __HAVE_ARCH_COPY_USER_HIGHPAGE

typedef struct { unsigned long pte; } pte_t;
#define pte_val(x)	((x).pte)
#define __pte(x)	((pte_t) { (x) })
typedef struct page *pgtable_t;

typedef struct { unsigned long pgd; } pgd_t;
#define pgd_val(x)	((x).pgd)
#define __pgd(x)	((pgd_t) { (x) })

/*
 * Manipulate page protection bits
 */
typedef struct { unsigned long pgprot; } pgprot_t;
#define pgprot_val(x)	((x).pgprot)
#define __pgprot(x)	((pgprot_t) { (x) })
#define pte_pgprot(x)	__pgprot(pte_val(x) & ~_PFN_MASK)

#define ptep_buddy(x)	((pte_t *)((unsigned long)(x) ^ sizeof(pte_t)))

/*
 * __pa()/__va() should be used only during mem init.
 */
#define __pa(x)		PHYSADDR(x)
#define __va(x)		((void *)((unsigned long)(x) + PAGE_OFFSET - PHYS_OFFSET))

#define pfn_to_kaddr(pfn)	__va((pfn) << PAGE_SHIFT)
#define sym_to_pfn(x)		__phys_to_pfn(__pa_symbol(x))

struct page *dmw_virt_to_page(unsigned long kaddr);
struct page *tlb_virt_to_page(unsigned long kaddr);

#define pfn_to_phys(pfn)	__pfn_to_phys(pfn)
#define phys_to_pfn(paddr)	__phys_to_pfn(paddr)

#define page_to_phys(page)	pfn_to_phys(page_to_pfn(page))
#define phys_to_page(paddr)	pfn_to_page(phys_to_pfn(paddr))

#ifndef CONFIG_KFENCE

#define page_to_virt(page)	__va(page_to_phys(page))
#define virt_to_page(kaddr)	phys_to_page(__pa(kaddr))

#else

#define WANT_PAGE_VIRTUAL

#define page_to_virt(page)								\
({											\
	extern char *__kfence_pool;							\
	(__kfence_pool == NULL) ? __va(page_to_phys(page)) : page_address(page);	\
})

#define virt_to_page(kaddr)								\
({											\
	(likely((unsigned long)kaddr < vm_map_base)) ?					\
	dmw_virt_to_page((unsigned long)kaddr) : tlb_virt_to_page((unsigned long)kaddr);\
})

#endif

#define pfn_to_virt(pfn)	page_to_virt(pfn_to_page(pfn))
#define virt_to_pfn(kaddr)	page_to_pfn(virt_to_page(kaddr))

extern int __virt_addr_valid(volatile void *kaddr);
#define virt_addr_valid(kaddr)	__virt_addr_valid((volatile void *)(kaddr))

#define VM_DATA_DEFAULT_FLAGS \
	(VM_READ | VM_WRITE | \
	 ((current->personality & READ_IMPLIES_EXEC) ? VM_EXEC : 0) | \
	 VM_MAYREAD | VM_MAYWRITE | VM_MAYEXEC)

#include <asm-generic/memory_model.h>
#include <asm-generic/getorder.h>

#endif /* !__ASSEMBLY__ */

#endif /* _ASM_PAGE_H */
