/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_HUGE_MM_H
#define _LINUX_HUGE_MM_H

#include <linux/mm_types.h>

#include <linux/fs.h> /* only for vma_is_dax() */
#include <linux/kobject.h>

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

vm_fault_t vmf_insert_pfn_pmd(struct vm_fault *vmf, unsigned long pfn,
			      bool write);
vm_fault_t vmf_insert_pfn_pud(struct vm_fault *vmf, unsigned long pfn,
			      bool write);
vm_fault_t vmf_insert_folio_pmd(struct vm_fault *vmf, struct folio *folio,
				bool write);
vm_fault_t vmf_insert_folio_pud(struct vm_fault *vmf, struct folio *folio,
				bool write);

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
extern struct kobj_attribute thpsize_shmem_enabled_attr;

/*
 * Mask of all large folio orders supported for anonymous THP; all orders up to
 * and including PMD_ORDER, except order-0 (which is not "huge") and order-1
 * (which is a limitation of the THP implementation).
 */
#define THP_ORDERS_ALL_ANON	((BIT(PMD_ORDER + 1) - 1) & ~(BIT(0) | BIT(1)))

/*
 * Mask of all large folio orders supported for file THP. Folios in a DAX
 * file is never split and the MAX_PAGECACHE_ORDER limit does not apply to
 * it.  Same to PFNMAPs where there's neither page* nor pagecache.
 */
#define THP_ORDERS_ALL_SPECIAL		\
	(BIT(PMD_ORDER) | BIT(PUD_ORDER))
#define THP_ORDERS_ALL_FILE_DEFAULT	\
	((BIT(MAX_PAGECACHE_ORDER + 1) - 1) & ~BIT(0))

/*
 * Mask of all large folio orders supported for THP.
 */
#define THP_ORDERS_ALL	\
	(THP_ORDERS_ALL_ANON | THP_ORDERS_ALL_SPECIAL | THP_ORDERS_ALL_FILE_DEFAULT)

enum tva_type {
	TVA_SMAPS,		/* Exposing "THPeligible:" in smaps. */
	TVA_PAGEFAULT,		/* Serving a page fault. */
	TVA_KHUGEPAGED,		/* Khugepaged collapse. */
	TVA_FORCED_COLLAPSE,	/* Forced collapse (e.g. MADV_COLLAPSE). */
};

#define thp_vma_allowable_order(vma, vm_flags, type, order) \
	(!!thp_vma_allowable_orders(vma, vm_flags, type, BIT(order)))

#define split_folio(f) split_folio_to_list(f, NULL)

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

enum mthp_stat_item {
	MTHP_STAT_ANON_FAULT_ALLOC,
	MTHP_STAT_ANON_FAULT_FALLBACK,
	MTHP_STAT_ANON_FAULT_FALLBACK_CHARGE,
	MTHP_STAT_ZSWPOUT,
	MTHP_STAT_SWPIN,
	MTHP_STAT_SWPIN_FALLBACK,
	MTHP_STAT_SWPIN_FALLBACK_CHARGE,
	MTHP_STAT_SWPOUT,
	MTHP_STAT_SWPOUT_FALLBACK,
	MTHP_STAT_SHMEM_ALLOC,
	MTHP_STAT_SHMEM_FALLBACK,
	MTHP_STAT_SHMEM_FALLBACK_CHARGE,
	MTHP_STAT_SPLIT,
	MTHP_STAT_SPLIT_FAILED,
	MTHP_STAT_SPLIT_DEFERRED,
	MTHP_STAT_NR_ANON,
	MTHP_STAT_NR_ANON_PARTIALLY_MAPPED,
	__MTHP_STAT_COUNT
};

#if defined(CONFIG_TRANSPARENT_HUGEPAGE) && defined(CONFIG_SYSFS)
struct mthp_stat {
	unsigned long stats[ilog2(MAX_PTRS_PER_PTE) + 1][__MTHP_STAT_COUNT];
};

DECLARE_PER_CPU(struct mthp_stat, mthp_stats);

static inline void mod_mthp_stat(int order, enum mthp_stat_item item, int delta)
{
	if (order <= 0 || order > PMD_ORDER)
		return;

	this_cpu_add(mthp_stats.stats[order][item], delta);
}

static inline void count_mthp_stat(int order, enum mthp_stat_item item)
{
	mod_mthp_stat(order, item, 1);
}

#else
static inline void mod_mthp_stat(int order, enum mthp_stat_item item, int delta)
{
}

static inline void count_mthp_stat(int order, enum mthp_stat_item item)
{
}
#endif

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

unsigned long __thp_vma_allowable_orders(struct vm_area_struct *vma,
					 vm_flags_t vm_flags,
					 enum tva_type type,
					 unsigned long orders);

/**
 * thp_vma_allowable_orders - determine hugepage orders that are allowed for vma
 * @vma:  the vm area to check
 * @vm_flags: use these vm_flags instead of vma->vm_flags
 * @type: TVA type
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
				       vm_flags_t vm_flags,
				       enum tva_type type,
				       unsigned long orders)
{
	/*
	 * Optimization to check if required orders are enabled early. Only
	 * forced collapse ignores sysfs configs.
	 */
	if (type != TVA_FORCED_COLLAPSE && vma_is_anonymous(vma)) {
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

	return __thp_vma_allowable_orders(vma, vm_flags, type, orders);
}

struct thpsize {
	struct kobject kobj;
	struct list_head node;
	int order;
};

#define to_thpsize(kobj) container_of(kobj, struct thpsize, kobj)

#define transparent_hugepage_use_zero_page()				\
	(transparent_hugepage_flags &					\
	 (1<<TRANSPARENT_HUGEPAGE_USE_ZERO_PAGE_FLAG))

/*
 * Check whether THPs are explicitly disabled for this VMA, for example,
 * through madvise or prctl.
 */
static inline bool vma_thp_disabled(struct vm_area_struct *vma,
		vm_flags_t vm_flags, bool forced_collapse)
{
	/* Are THPs disabled for this VMA? */
	if (vm_flags & VM_NOHUGEPAGE)
		return true;
	/* Are THPs disabled for all VMAs in the whole process? */
	if (mm_flags_test(MMF_DISABLE_THP_COMPLETELY, vma->vm_mm))
		return true;
	/*
	 * Are THPs disabled only for VMAs where we didn't get an explicit
	 * advise to use them?
	 */
	if (vm_flags & VM_HUGEPAGE)
		return false;
	/*
	 * Forcing a collapse (e.g., madv_collapse), is a clear advice to
	 * use THPs.
	 */
	if (forced_collapse)
		return false;
	return mm_flags_test(MMF_DISABLE_THP_EXCEPT_ADVISED, vma->vm_mm);
}

static inline bool thp_disabled_by_hw(void)
{
	/* If the hardware/firmware marked hugepage support disabled. */
	return transparent_hugepage_flags & (1 << TRANSPARENT_HUGEPAGE_UNSUPPORTED);
}

unsigned long thp_get_unmapped_area(struct file *filp, unsigned long addr,
		unsigned long len, unsigned long pgoff, unsigned long flags);
unsigned long thp_get_unmapped_area_vmflags(struct file *filp, unsigned long addr,
		unsigned long len, unsigned long pgoff, unsigned long flags,
		vm_flags_t vm_flags);

bool can_split_folio(struct folio *folio, int caller_pins, int *pextra_pins);
int split_huge_page_to_list_to_order(struct page *page, struct list_head *list,
		unsigned int new_order);
int min_order_for_split(struct folio *folio);
int split_folio_to_list(struct folio *folio, struct list_head *list);
bool uniform_split_supported(struct folio *folio, unsigned int new_order,
		bool warns);
bool non_uniform_split_supported(struct folio *folio, unsigned int new_order,
		bool warns);
int folio_split(struct folio *folio, unsigned int new_order, struct page *page,
		struct list_head *list);
/*
 * try_folio_split_to_order - try to split a @folio at @page to @new_order using
 * non uniform split.
 * @folio: folio to be split
 * @page: split to @new_order at the given page
 * @new_order: the target split order
 *
 * Try to split a @folio at @page using non uniform split to @new_order, if
 * non uniform split is not supported, fall back to uniform split. After-split
 * folios are put back to LRU list. Use min_order_for_split() to get the lower
 * bound of @new_order.
 *
 * Return: 0: split is successful, otherwise split failed.
 */
static inline int try_folio_split_to_order(struct folio *folio,
		struct page *page, unsigned int new_order)
{
	if (!non_uniform_split_supported(folio, new_order, /* warns= */ false))
		return split_huge_page_to_list_to_order(&folio->page, NULL,
				new_order);
	return folio_split(folio, new_order, page, NULL);
}
static inline int split_huge_page(struct page *page)
{
	return split_huge_page_to_list_to_order(page, NULL, 0);
}
void deferred_split_folio(struct folio *folio, bool partially_mapped);

void __split_huge_pmd(struct vm_area_struct *vma, pmd_t *pmd,
		unsigned long address, bool freeze);

#define split_huge_pmd(__vma, __pmd, __address)				\
	do {								\
		pmd_t *____pmd = (__pmd);				\
		if (is_swap_pmd(*____pmd) || pmd_trans_huge(*____pmd))	\
			__split_huge_pmd(__vma, __pmd, __address,	\
					 false);			\
	}  while (0)

void split_huge_pmd_address(struct vm_area_struct *vma, unsigned long address,
		bool freeze);

void __split_huge_pud(struct vm_area_struct *vma, pud_t *pud,
		unsigned long address);

#ifdef CONFIG_HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD
int change_huge_pud(struct mmu_gather *tlb, struct vm_area_struct *vma,
		    pud_t *pudp, unsigned long addr, pgprot_t newprot,
		    unsigned long cp_flags);
#else
static inline int
change_huge_pud(struct mmu_gather *tlb, struct vm_area_struct *vma,
		pud_t *pudp, unsigned long addr, pgprot_t newprot,
		unsigned long cp_flags) { return 0; }
#endif

#define split_huge_pud(__vma, __pud, __address)				\
	do {								\
		pud_t *____pud = (__pud);				\
		if (pud_trans_huge(*____pud))				\
			__split_huge_pud(__vma, __pud, __address);	\
	}  while (0)

int hugepage_madvise(struct vm_area_struct *vma, vm_flags_t *vm_flags,
		     int advice);
int madvise_collapse(struct vm_area_struct *vma, unsigned long start,
		     unsigned long end, bool *lock_dropped);
void vma_adjust_trans_huge(struct vm_area_struct *vma, unsigned long start,
			   unsigned long end, struct vm_area_struct *next);
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
	if (is_swap_pmd(*pmd) || pmd_trans_huge(*pmd))
		return __pmd_trans_huge_lock(pmd, vma);
	else
		return NULL;
}
static inline spinlock_t *pud_trans_huge_lock(pud_t *pud,
		struct vm_area_struct *vma)
{
	if (pud_trans_huge(*pud))
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

vm_fault_t do_huge_pmd_numa_page(struct vm_fault *vmf);

extern struct folio *huge_zero_folio;
extern unsigned long huge_zero_pfn;

static inline bool is_huge_zero_folio(const struct folio *folio)
{
	VM_WARN_ON_ONCE(!folio);

	return READ_ONCE(huge_zero_folio) == folio;
}

static inline bool is_huge_zero_pfn(unsigned long pfn)
{
	return READ_ONCE(huge_zero_pfn) == (pfn & ~(HPAGE_PMD_NR - 1));
}

static inline bool is_huge_zero_pmd(pmd_t pmd)
{
	return pmd_present(pmd) && is_huge_zero_pfn(pmd_pfn(pmd));
}

struct folio *mm_get_huge_zero_folio(struct mm_struct *mm);
void mm_put_huge_zero_folio(struct mm_struct *mm);

static inline struct folio *get_persistent_huge_zero_folio(void)
{
	if (!IS_ENABLED(CONFIG_PERSISTENT_HUGE_ZERO_FOLIO))
		return NULL;

	if (unlikely(!huge_zero_folio))
		return NULL;

	return huge_zero_folio;
}

static inline bool thp_migration_supported(void)
{
	return IS_ENABLED(CONFIG_ARCH_ENABLE_THP_MIGRATION);
}

void split_huge_pmd_locked(struct vm_area_struct *vma, unsigned long address,
			   pmd_t *pmd, bool freeze);
bool unmap_huge_pmd_locked(struct vm_area_struct *vma, unsigned long addr,
			   pmd_t *pmdp, struct folio *folio);

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
					vm_flags_t vm_flags,
					enum tva_type type,
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
can_split_folio(struct folio *folio, int caller_pins, int *pextra_pins)
{
	return false;
}
static inline int
split_huge_page_to_list_to_order(struct page *page, struct list_head *list,
		unsigned int new_order)
{
	VM_WARN_ON_ONCE_PAGE(1, page);
	return -EINVAL;
}
static inline int split_huge_page(struct page *page)
{
	VM_WARN_ON_ONCE_PAGE(1, page);
	return -EINVAL;
}

static inline int min_order_for_split(struct folio *folio)
{
	VM_WARN_ON_ONCE_FOLIO(1, folio);
	return -EINVAL;
}

static inline int split_folio_to_list(struct folio *folio, struct list_head *list)
{
	VM_WARN_ON_ONCE_FOLIO(1, folio);
	return -EINVAL;
}

static inline int try_folio_split_to_order(struct folio *folio,
		struct page *page, unsigned int new_order)
{
	VM_WARN_ON_ONCE_FOLIO(1, folio);
	return -EINVAL;
}

static inline void deferred_split_folio(struct folio *folio, bool partially_mapped) {}
#define split_huge_pmd(__vma, __pmd, __address)	\
	do { } while (0)

static inline void __split_huge_pmd(struct vm_area_struct *vma, pmd_t *pmd,
		unsigned long address, bool freeze) {}
static inline void split_huge_pmd_address(struct vm_area_struct *vma,
		unsigned long address, bool freeze) {}
static inline void split_huge_pmd_locked(struct vm_area_struct *vma,
					 unsigned long address, pmd_t *pmd,
					 bool freeze) {}

static inline bool unmap_huge_pmd_locked(struct vm_area_struct *vma,
					 unsigned long addr, pmd_t *pmdp,
					 struct folio *folio)
{
	return false;
}

#define split_huge_pud(__vma, __pmd, __address)	\
	do { } while (0)

static inline int hugepage_madvise(struct vm_area_struct *vma,
				   vm_flags_t *vm_flags, int advice)
{
	return -EINVAL;
}

static inline int madvise_collapse(struct vm_area_struct *vma,
				   unsigned long start,
				   unsigned long end, bool *lock_dropped)
{
	return -EINVAL;
}

static inline void vma_adjust_trans_huge(struct vm_area_struct *vma,
					 unsigned long start,
					 unsigned long end,
					 struct vm_area_struct *next)
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

static inline bool is_huge_zero_pfn(unsigned long pfn)
{
	return false;
}

static inline bool is_huge_zero_pmd(pmd_t pmd)
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

static inline int highest_order(unsigned long orders)
{
	return 0;
}

static inline int next_order(unsigned long *orders, int prev)
{
	return 0;
}

static inline void __split_huge_pud(struct vm_area_struct *vma, pud_t *pud,
				    unsigned long address)
{
}

static inline int change_huge_pud(struct mmu_gather *tlb,
				  struct vm_area_struct *vma, pud_t *pudp,
				  unsigned long addr, pgprot_t newprot,
				  unsigned long cp_flags)
{
	return 0;
}

static inline struct folio *get_persistent_huge_zero_folio(void)
{
	return NULL;
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

/**
 * largest_zero_folio - Get the largest zero size folio available
 *
 * This function shall be used when mm_get_huge_zero_folio() cannot be
 * used as there is no appropriate mm lifetime to tie the huge zero folio
 * from the caller.
 *
 * Deduce the size of the folio with folio_size instead of assuming the
 * folio size.
 *
 * Return: pointer to PMD sized zero folio if CONFIG_PERSISTENT_HUGE_ZERO_FOLIO
 * is enabled or a single page sized zero folio
 */
static inline struct folio *largest_zero_folio(void)
{
	struct folio *folio = get_persistent_huge_zero_folio();

	if (folio)
		return folio;

	return page_folio(ZERO_PAGE(0));
}
#endif /* _LINUX_HUGE_MM_H */
