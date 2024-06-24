/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_HUGE_MM_H
#define _LINUX_HUGE_MM_H

#include <linux/sched/coredump.h>
#include <linux/mm_types.h>

#include <linux/fs.h> /* only for vma_is_dax() */

vm_fault_t do_huge_pmd_anonymous_page(struct vm_fault *vmf);
int copy_huge_pmd(struct mm_struct *dst_mm, struct mm_struct *src_mm,
		  pmd_t *dst_pmd, pmd_t *src_pmd, unsigned long addr,
		  struct vm_area_struct *dst_vma, struct vm_area_struct *src_vma);
void huge_pmd_set_accessed(struct vm_fault *vmf);
int copy_huge_pud(struct mm_struct *dst_mm, struct mm_struct *src_mm,
		  pud_t *dst_pud, pud_t *src_pud, unsigned long addr,
		  struct vm_area_struct *vma);

#ifdef CONFIG_HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD
void huge_pud_set_accessed(struct vm_fault *vmf, pud_t orig_pud);
#else
static inline void huge_pud_set_accessed(struct vm_fault *vmf, pud_t orig_pud)
{
}
#endif

vm_fault_t do_huge_pmd_wp_page(struct vm_fault *vmf);
bool madvise_free_huge_pmd(struct mmu_gather *tlb, struct vm_area_struct *vma,
			   pmd_t *pmd, unsigned long addr, unsigned long next);
int zap_huge_pmd(struct mmu_gather *tlb, struct vm_area_struct *vma, pmd_t *pmd,
		 unsigned long addr);
int zap_huge_pud(struct mmu_gather *tlb, struct vm_area_struct *vma, pud_t *pud,
		 unsigned long addr);
bool move_huge_pmd(struct vm_area_struct *vma, unsigned long old_addr,
		   unsigned long new_addr, pmd_t *old_pmd, pmd_t *new_pmd);
int change_huge_pmd(struct mmu_gather *tlb, struct vm_area_struct *vma,
		    pmd_t *pmd, unsigned long addr, pgprot_t newprot,
		    unsigned long cp_flags);

vm_fault_t vmf_insert_pfn_pmd(struct vm_fault *vmf, pfn_t pfn, bool write);
vm_fault_t vmf_insert_pfn_pud(struct vm_fault *vmf, pfn_t pfn, bool write);

enum transparent_hugepage_flag {
	TRANSPARENT_HUGEPAGE_UNSUPPORTED,
	TRANSPARENT_HUGEPAGE_FLAG,
	TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG,
	TRANSPARENT_HUGEPAGE_DEFRAG_DIRECT_FLAG,
	TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_FLAG,
	TRANSPARENT_HUGEPAGE_DEFRAG_KSWAPD_OR_MADV_FLAG,
	TRANSPARENT_HUGEPAGE_DEFRAG_REQ_MADV_FLAG,
	TRANSPARENT_HUGEPAGE_DEFRAG_KHUGEPAGED_FLAG,
	TRANSPARENT_HUGEPAGE_USE_ZERO_PAGE_FLAG,
};

struct kobject;
struct kobj_attribute;

ssize_t single_hugepage_flag_store(struct kobject *kobj,
				   struct kobj_attribute *attr,
				   const char *buf, size_t count,
				   enum transparent_hugepage_flag flag);
ssize_t single_hugepage_flag_show(struct kobject *kobj,
				  struct kobj_attribute *attr, char *buf,
				  enum transparent_hugepage_flag flag);
extern struct kobj_attribute shmem_enabled_attr;

/*
 * Mask of all large folio orders supported for anonymous THP; all orders up to
 * and including PMD_ORDER, except order-0 (which is not "huge") and order-1
 * (which is a limitation of the THP implementation).
 */
#define THP_ORDERS_ALL_ANON	((BIT(PMD_ORDER + 1) - 1) & ~(BIT(0) | BIT(1)))

/*
 * Mask of all large folio orders supported for file THP.
 */
#define THP_ORDERS_ALL_FILE	(BIT(PMD_ORDER) | BIT(PUD_ORDER))

/*
 * Mask of all large folio orders supported for THP.
 */
#define THP_ORDERS_ALL		(THP_ORDERS_ALL_ANON | THP_ORDERS_ALL_FILE)

#define TVA_SMAPS		(1 << 0)	/* Will be used for procfs */
#define TVA_IN_PF		(1 << 1)	/* Page fault handler */
#define TVA_ENFORCE_SYSFS	(1 << 2)	/* Obey sysfs configuration */

#define thp_vma_allowable_order(vma, vm_flags, tva_flags, order) \
	(!!thp_vma_allowable_orders(vma, vm_flags, tva_flags, BIT(order)))

#ifdef CONFIG_PGTABLE_HAS_HUGE_LEAVES
#define HPAGE_PMD_SHIFT PMD_SHIFT
#define HPAGE_PUD_SHIFT PUD_SHIFT
#else
#define HPAGE_PMD_SHIFT ({ BUILD_BUG(); 0; })
#define HPAGE_PUD_SHIFT ({ BUILD_BUG(); 0; })
#endif

#define HPAGE_PMD_ORDER (HPAGE_PMD_SHIFT-PAGE_SHIFT)
#define HPAGE_PMD_NR (1<<HPAGE_PMD_ORDER)
#define HPAGE_PMD_MASK	(~(HPAGE_PMD_SIZE - 1))
#define HPAGE_PMD_SIZE	((1UL) << HPAGE_PMD_SHIFT)

#define HPAGE_PUD_ORDER (HPAGE_PUD_SHIFT-PAGE_SHIFT)
#define HPAGE_PUD_NR (1<<HPAGE_PUD_ORDER)
#define HPAGE_PUD_MASK	(~(HPAGE_PUD_SIZE - 1))
#define HPAGE_PUD_SIZE	((1UL) << HPAGE_PUD_SHIFT)

#ifdef CONFIG_TRANSPARENT_HUGEPAGE

extern unsigned long transparent_hugepage_flags;
extern unsigned long huge_anon_orders_always;
extern unsigned long huge_anon_orders_madvise;
extern unsigned long huge_anon_orders_inherit;

static inline bool hugepage_global_enabled(void)
{
	return transparent_hugepage_flags &
			((1<<TRANSPARENT_HUGEPAGE_FLAG) |
			(1<<TRANSPARENT_HUGEPAGE_REQ_MADV_FLAG));
}

static inline bool hugepage_global_always(void)
{
	return transparent_hugepage_flags &
			(1<<TRANSPARENT_HUGEPAGE_FLAG);
}

static inline bool hugepage_flags_enabled(void)
{
	/*
	 * We cover both the anon and the file-backed case here; we must return
	 * true if globally enabled, even when all anon sizes are set to never.
	 * So we don't need to look at huge_anon_orders_inherit.
	 */
	return hugepage_global_enabled() ||
	       huge_anon_orders_always ||
	       huge_anon_orders_madvise;
}

static inline int highest_order(unsigned long orders)
{
	return fls_long(orders) - 1;
}

static inline int next_order(unsigned long *orders, int prev)
{
	*orders &= ~BIT(prev);
	return highest_order(*orders);
}

/*
 * Do the below checks:
 *   - For file vma, check if the linear page offset of vma is
 *     order-aligned within the file.  The hugepage is
 *     guaranteed to be order-aligned within the file, but we must
 *     check that the order-aligned addresses in the VMA map to
 *     order-aligned offsets within the file, else the hugepage will
 *     not be mappable.
 *   - For all vmas, check if the haddr is in an aligned hugepage
 *     area.
 */
static inline bool thp_vma_suitable_order(struct vm_area_struct *vma,
		unsigned long addr, int order)
{
	unsigned long hpage_size = PAGE_SIZE << order;
	unsigned long haddr;

	/* Don't have to check pgoff for anonymous vma */
	if (!vma_is_anonymous(vma)) {
		if (!IS_ALIGNED((vma->vm_start >> PAGE_SHIFT) - vma->vm_pgoff,
				hpage_size >> PAGE_SHIFT))
			return false;
	}

	haddr = ALIGN_DOWN(addr, hpage_size);

	if (haddr < vma->vm_start || haddr + hpage_size > vma->vm_end)
		return false;
	return true;
}

/*
 * Filter the bitfield of input orders to the ones suitable for use in the vma.
 * See thp_vma_suitable_order().
 * All orders that pass the checks are returned as a bitfield.
 */
static inline unsigned long thp_vma_suitable_orders(struct vm_area_struct *vma,
		unsigned long addr, unsigned long orders)
{
	int order;

	/*
	 * Iterate over orders, highest to lowest, removing orders that don't
	 * meet alignment requirements from the set. Exit loop at first order
	 * that meets requirements, since all lower orders must also meet
	 * requirements.
	 */

	order = highest_order(orders);

	while (orders) {
		if (thp_vma_suitable_order(vma, addr, order))
			break;
		order = next_order(&orders, order);
	}

	return orders;
}

static inline bool file_thp_enabled(struct vm_area_struct *vma)
{
	struct inode *inode;

	if (!vma->vm_file)
		return false;

	inode = vma->vm_file->f_inode;

	return (IS_ENABLED(CONFIG_READ_ONLY_THP_FOR_FS)) &&
	       !inode_is_open_for_write(inode) && S_ISREG(inode->i_mode);
}

unsigned long __thp_vma_allowable_orders(struct vm_area_struct *vma,
					 unsigned long vm_flags,
					 unsigned long tva_flags,
					 unsigned long orders);

/**
 * thp_vma_allowable_orders - determine hugepage orders that are allowed for vma
 * @vma:  the vm area to check
 * @vm_flags: use these vm_flags instead of vma->vm_flags
 * @tva_flags: Which TVA flags to honour
 * @orders: bitfield of all orders to consider
 *
 * Calculates the intersection of the requested hugepage orders and the allowed
 * hugepage orders for the provided vma. Permitted orders are encoded as a set
 * bit at the corresponding bit position (bit-2 corresponds to order-2, bit-3
 * corresponds to order-3, etc). Order-0 is never considered a hugepage order.
 *
 * Return: bitfield of orders allowed for hugepage in the vma. 0 if no hugepage
 * orders are allowed.
 */
static inline
unsigned long thp_vma_allowable_orders(struct vm_area_struct *vma,
				       unsigned long vm_flags,
				       unsigned long tva_flags,
				       unsigned long orders)
{
	/* Optimization to check if required orders are enabled early. */
	if ((tva_flags & TVA_ENFORCE_SYSFS) && vma_is_anonymous(vma)) {
		unsigned long mask = READ_ONCE(huge_anon_orders_always);

		if (vm_flags & VM_HUGEPAGE)
			mask |= READ_ONCE(huge_anon_orders_madvise);
		if (hugepage_global_always() ||
		    ((vm_flags & VM_HUGEPAGE) && hugepage_global_enabled()))
			mask |= READ_ONCE(huge_anon_orders_inherit);

		orders &= mask;
		if (!orders)
			return 0;
	}

	return __thp_vma_allowable_orders(vma, vm_flags, tva_flags, orders);
}

enum mthp_stat_item {
	MTHP_STAT_ANON_FAULT_ALLOC,
	MTHP_STAT_ANON_FAULT_FALLBACK,
	MTHP_STAT_ANON_FAULT_FALLBACK_CHARGE,
	MTHP_STAT_SWPOUT,
	MTHP_STAT_SWPOUT_FALLBACK,
	__MTHP_STAT_COUNT
};

struct mthp_stat {
	unsigned long stats[ilog2(MAX_PTRS_PER_PTE) + 1][__MTHP_STAT_COUNT];
};

#ifdef CONFIG_SYSFS
DECLARE_PER_CPU(struct mthp_stat, mthp_stats);

static inline void count_mthp_stat(int order, enum mthp_stat_item item)
{
	if (order <= 0 || order > PMD_ORDER)
		return;

	this_cpu_inc(mthp_stats.stats[order][item]);
}
#else
static inline void count_mthp_stat(int order, enum mthp_stat_item item)
{
}
#endif

#define transparent_hugepage_use_zero_page()				\
	(transparent_hugepage_flags &					\
	 (1<<TRANSPARENT_HUGEPAGE_USE_ZERO_PAGE_FLAG))

unsigned long thp_get_unmapped_area(struct file *filp, unsigned long addr,
		unsigned long len, unsigned long pgoff, unsigned long flags);
unsigned long thp_get_unmapped_area_vmflags(struct file *filp, unsigned long addr,
		unsigned long len, unsigned long pgoff, unsigned long flags,
		vm_flags_t vm_flags);

bool can_split_folio(struct folio *folio, int *pextra_pins);
int split_huge_page_to_list_to_order(struct page *page, struct list_head *list,
		unsigned int new_order);
static inline int split_huge_page(struct page *page)
{
	return split_huge_page_to_list_to_order(page, NULL, 0);
}
void deferred_split_folio(struct folio *folio);

void __split_huge_pmd(struct vm_area_struct *vma, pmd_t *pmd,
		unsigned long address, bool freeze, struct folio *folio);

#define split_huge_pmd(__vma, __pmd, __address)				\
	do {								\
		pmd_t *____pmd = (__pmd);				\
		if (is_swap_pmd(*____pmd) || pmd_trans_huge(*____pmd)	\
					|| pmd_devmap(*____pmd))	\
			__split_huge_pmd(__vma, __pmd, __address,	\
						false, NULL);		\
	}  while (0)


void split_huge_pmd_address(struct vm_area_struct *vma, unsigned long address,
		bool freeze, struct folio *folio);

void __split_huge_pud(struct vm_area_struct *vma, pud_t *pud,
		unsigned long address);

#define split_huge_pud(__vma, __pud, __address)				\
	do {								\
		pud_t *____pud = (__pud);				\
		if (pud_trans_huge(*____pud)				\
					|| pud_devmap(*____pud))	\
			__split_huge_pud(__vma, __pud, __address);	\
	}  while (0)

int hugepage_madvise(struct vm_area_struct *vma, unsigned long *vm_flags,
		     int advice);
int madvise_collapse(struct vm_area_struct *vma,
		     struct vm_area_struct **prev,
		     unsigned long start, unsigned long end);
void vma_adjust_trans_huge(struct vm_area_struct *vma, unsigned long start,
			   unsigned long end, long adjust_next);
spinlock_t *__pmd_trans_huge_lock(pmd_t *pmd, struct vm_area_struct *vma);
spinlock_t *__pud_trans_huge_lock(pud_t *pud, struct vm_area_struct *vma);

static inline int is_swap_pmd(pmd_t pmd)
{
	return !pmd_none(pmd) && !pmd_present(pmd);
}

/* mmap_lock must be held on entry */
static inline spinlock_t *pmd_trans_huge_lock(pmd_t *pmd,
		struct vm_area_struct *vma)
{
	if (is_swap_pmd(*pmd) || pmd_trans_huge(*pmd) || pmd_devmap(*pmd))
		return __pmd_trans_huge_lock(pmd, vma);
	else
		return NULL;
}
static inline spinlock_t *pud_trans_huge_lock(pud_t *pud,
		struct vm_area_struct *vma)
{
	if (pud_trans_huge(*pud) || pud_devmap(*pud))
		return __pud_trans_huge_lock(pud, vma);
	else
		return NULL;
}

/**
 * folio_test_pmd_mappable - Can we map this folio with a PMD?
 * @folio: The folio to test
 */
static inline bool folio_test_pmd_mappable(struct folio *folio)
{
	return folio_order(folio) >= HPAGE_PMD_ORDER;
}

struct page *follow_devmap_pmd(struct vm_area_struct *vma, unsigned long addr,
		pmd_t *pmd, int flags, struct dev_pagemap **pgmap);

vm_fault_t do_huge_pmd_numa_page(struct vm_fault *vmf);

extern struct folio *huge_zero_folio;
extern unsigned long huge_zero_pfn;

static inline bool is_huge_zero_folio(const struct folio *folio)
{
	return READ_ONCE(huge_zero_folio) == folio;
}

static inline bool is_huge_zero_pmd(pmd_t pmd)
{
	return pmd_present(pmd) && READ_ONCE(huge_zero_pfn) == pmd_pfn(pmd);
}

static inline bool is_huge_zero_pud(pud_t pud)
{
	return false;
}

struct folio *mm_get_huge_zero_folio(struct mm_struct *mm);
void mm_put_huge_zero_folio(struct mm_struct *mm);

#define mk_huge_pmd(page, prot) pmd_mkhuge(mk_pmd(page, prot))

static inline bool thp_migration_supported(void)
{
	return IS_ENABLED(CONFIG_ARCH_ENABLE_THP_MIGRATION);
}

#else /* CONFIG_TRANSPARENT_HUGEPAGE */

static inline bool folio_test_pmd_mappable(struct folio *folio)
{
	return false;
}

static inline bool thp_vma_suitable_order(struct vm_area_struct *vma,
		unsigned long addr, int order)
{
	return false;
}

static inline unsigned long thp_vma_suitable_orders(struct vm_area_struct *vma,
		unsigned long addr, unsigned long orders)
{
	return 0;
}

static inline unsigned long thp_vma_allowable_orders(struct vm_area_struct *vma,
					unsigned long vm_flags,
					unsigned long tva_flags,
					unsigned long orders)
{
	return 0;
}

#define transparent_hugepage_flags 0UL

#define thp_get_unmapped_area	NULL

static inline unsigned long
thp_get_unmapped_area_vmflags(struct file *filp, unsigned long addr,
			      unsigned long len, unsigned long pgoff,
			      unsigned long flags, vm_flags_t vm_flags)
{
	return 0;
}

static inline bool
can_split_folio(struct folio *folio, int *pextra_pins)
{
	return false;
}
static inline int
split_huge_page_to_list_to_order(struct page *page, struct list_head *list,
		unsigned int new_order)
{
	return 0;
}
static inline int split_huge_page(struct page *page)
{
	return 0;
}
static inline void deferred_split_folio(struct folio *folio) {}
#define split_huge_pmd(__vma, __pmd, __address)	\
	do { } while (0)

static inline void __split_huge_pmd(struct vm_area_struct *vma, pmd_t *pmd,
		unsigned long address, bool freeze, struct folio *folio) {}
static inline void split_huge_pmd_address(struct vm_area_struct *vma,
		unsigned long address, bool freeze, struct folio *folio) {}

#define split_huge_pud(__vma, __pmd, __address)	\
	do { } while (0)

static inline int hugepage_madvise(struct vm_area_struct *vma,
				   unsigned long *vm_flags, int advice)
{
	return -EINVAL;
}

static inline int madvise_collapse(struct vm_area_struct *vma,
				   struct vm_area_struct **prev,
				   unsigned long start, unsigned long end)
{
	return -EINVAL;
}

static inline void vma_adjust_trans_huge(struct vm_area_struct *vma,
					 unsigned long start,
					 unsigned long end,
					 long adjust_next)
{
}
static inline int is_swap_pmd(pmd_t pmd)
{
	return 0;
}
static inline spinlock_t *pmd_trans_huge_lock(pmd_t *pmd,
		struct vm_area_struct *vma)
{
	return NULL;
}
static inline spinlock_t *pud_trans_huge_lock(pud_t *pud,
		struct vm_area_struct *vma)
{
	return NULL;
}

static inline vm_fault_t do_huge_pmd_numa_page(struct vm_fault *vmf)
{
	return 0;
}

static inline bool is_huge_zero_folio(const struct folio *folio)
{
	return false;
}

static inline bool is_huge_zero_pmd(pmd_t pmd)
{
	return false;
}

static inline bool is_huge_zero_pud(pud_t pud)
{
	return false;
}

static inline void mm_put_huge_zero_folio(struct mm_struct *mm)
{
	return;
}

static inline struct page *follow_devmap_pmd(struct vm_area_struct *vma,
	unsigned long addr, pmd_t *pmd, int flags, struct dev_pagemap **pgmap)
{
	return NULL;
}

static inline bool thp_migration_supported(void)
{
	return false;
}
#endif /* CONFIG_TRANSPARENT_HUGEPAGE */

static inline int split_folio_to_list_to_order(struct folio *folio,
		struct list_head *list, int new_order)
{
	return split_huge_page_to_list_to_order(&folio->page, list, new_order);
}

static inline int split_folio_to_order(struct folio *folio, int new_order)
{
	return split_folio_to_list_to_order(folio, NULL, new_order);
}

#define split_folio_to_list(f, l) split_folio_to_list_to_order(f, l, 0)
#define split_folio(f) split_folio_to_order(f, 0)

#endif /* _LINUX_HUGE_MM_H */
