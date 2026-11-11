// SPDX-License-Identifier: GPL-2.0
/*
 * HugeTLB Vmemmap Optimization (HVO)
 *
 * Copyright (c) 2020, ByteDance. All rights reserved.
 *
 *     Author: Muchun Song <songmuchun@bytedance.com>
 *
 * See Documentation/mm/vmemmap_dedup.rst
 */
#define pr_fmt(fmt)	"HugeTLB: " fmt

#include <linux/pgtable.h>
#include <linux/moduleparam.h>
#include <linux/mmdebug.h>
#include <linux/pagewalk.h>
#include <linux/pgalloc.h>
#include <linux/vmemmap-optimization.h>

#include <asm/tlbflush.h>
#include "hugetlb_vmemmap.h"

/**
 * struct vmemmap_remap_walk - walk vmemmap page table
 *
 * @remap_pte:		called for each lowest-level entry (PTE).
 * @nr_walked:		the number of walked pte.
 * @vmemmap_pages:	list supplying replacement pages for the remap and
 *			receiving displaced backing pages for the caller to free.
 * @flags:		used to modify behavior in vmemmap page table walking
 *			operations.
 */
struct vmemmap_remap_walk {
	int			(*remap_pte)(pte_t *pte, unsigned long addr,
					     struct vmemmap_remap_walk *walk);
	unsigned long		nr_walked;
	struct list_head	*vmemmap_pages;

/* Defer safely batchable TLB flushes to the caller. */
#define VMEMMAP_DEFER_TLB_FLUSH	BIT(0)
	unsigned long		flags;
};

static int vmemmap_split_pmd(pmd_t *pmd, struct page *head, unsigned long start,
			     struct vmemmap_remap_walk *walk)
{
	pmd_t __pmd;
	int i;
	unsigned long addr = start;
	pte_t *pgtable;

	pgtable = pte_alloc_one_kernel(&init_mm);
	if (!pgtable)
		return -ENOMEM;

	pmd_populate_kernel(&init_mm, &__pmd, pgtable);

	for (i = 0; i < PTRS_PER_PTE; i++, addr += PAGE_SIZE) {
		pte_t entry, *pte;
		pgprot_t pgprot = PAGE_KERNEL;

		entry = mk_pte(head + i, pgprot);
		pte = pte_offset_kernel(&__pmd, addr);
		set_pte_at(&init_mm, addr, pte, entry);
	}

	spin_lock(&init_mm.page_table_lock);
	if (likely(pmd_leaf(*pmd))) {
		/*
		 * Higher order allocations from buddy allocator must be able to
		 * be treated as independent small pages (as they can be freed
		 * individually).
		 */
		if (!PageReserved(head))
			split_page(head, get_order(PMD_SIZE));

		/* Make pte visible before pmd. See comment in pmd_install(). */
		smp_wmb();
		pmd_populate_kernel(&init_mm, pmd, pgtable);
		/*
		 * Modern MMUs allow overlapping TLB entries with different mapping
		 * granularities only if they map the same physical address. So
		 * remapping must not proceed without first flushing the TLB.
		 */
		if (walk->remap_pte || !(walk->flags & VMEMMAP_DEFER_TLB_FLUSH))
			flush_tlb_kernel_range(start, start + PMD_SIZE);
	} else {
		pte_free_kernel(&init_mm, pgtable);
	}
	spin_unlock(&init_mm.page_table_lock);

	return 0;
}

static int vmemmap_pmd_entry(pmd_t *pmd, unsigned long addr,
			     unsigned long next, struct mm_walk *walk)
{
	int ret = 0;
	struct page *head;
	struct vmemmap_remap_walk *vmemmap_walk = walk->private;

	/* Only splitting, not remapping the vmemmap pages. */
	if (!vmemmap_walk->remap_pte)
		walk->action = ACTION_CONTINUE;

	spin_lock(&init_mm.page_table_lock);
	head = pmd_leaf(*pmd) ? pmd_page(*pmd) : NULL;
	/*
	 * Due to HugeTLB alignment requirements and the vmemmap
	 * pages being at the start of the hotplugged memory
	 * region in memory_hotplug.memmap_on_memory case. Checking
	 * the vmemmap page associated with the first vmemmap page
	 * if it is self-hosted is sufficient.
	 *
	 * [                  hotplugged memory                  ]
	 * [        section        ][...][        section        ]
	 * [ vmemmap ][              usable memory               ]
	 *   ^  | ^                        |
	 *   +--+ |                        |
	 *        +------------------------+
	 */
	if (IS_ENABLED(CONFIG_MEMORY_HOTPLUG) && unlikely(!vmemmap_walk->nr_walked)) {
		struct page *page = head ? head + pte_index(addr) :
				    pte_page(ptep_get(pte_offset_kernel(pmd, addr)));

		if (PageVmemmapSelfHosted(page))
			ret = -ENOTSUPP;
	}
	spin_unlock(&init_mm.page_table_lock);
	if (!head || ret)
		return ret;

	return vmemmap_split_pmd(pmd, head, addr & PMD_MASK, vmemmap_walk);
}

static int vmemmap_pte_entry(pte_t *pte, unsigned long addr,
			     unsigned long next, struct mm_walk *walk)
{
	int ret;
	struct vmemmap_remap_walk *vmemmap_walk = walk->private;

	ret = vmemmap_walk->remap_pte(pte, addr, vmemmap_walk);
	if (!ret)
		vmemmap_walk->nr_walked++;

	return ret;
}

static const struct mm_walk_ops vmemmap_remap_ops = {
	.pmd_entry	= vmemmap_pmd_entry,
	.pte_entry	= vmemmap_pte_entry,
};

static int vmemmap_remap_range(unsigned long start, unsigned long end,
			       struct vmemmap_remap_walk *walk)
{
	int ret;

	VM_BUG_ON(!PAGE_ALIGNED(start | end));

	mmap_read_lock(&init_mm);
	ret = walk_kernel_page_table_range(start, end, &vmemmap_remap_ops,
				    NULL, walk);
	mmap_read_unlock(&init_mm);
	if (ret)
		return ret;

	if (walk->remap_pte && !(walk->flags & VMEMMAP_DEFER_TLB_FLUSH))
		flush_tlb_kernel_range(start, end);

	return 0;
}

/*
 * Free a vmemmap page. A vmemmap page can be allocated from the memblock
 * allocator or buddy allocator. If the PG_reserved flag is set, it means
 * that it allocated from the memblock allocator, just free it via the
 * free_reserved_page(). Otherwise, use __free_page().
 */
static inline void free_vmemmap_page(struct page *page)
{
	if (PageReserved(page)) {
		memmap_boot_pages_add(-1);
		free_reserved_page(page);
	} else {
		memmap_pages_add(-1);
		__free_page(page);
	}
}

/* Free a list of the vmemmap pages */
static void free_vmemmap_page_list(struct list_head *list)
{
	struct page *page, *next;

	list_for_each_entry_safe(page, next, list, lru)
		free_vmemmap_page(page);
}

static int alloc_vmemmap_page_list(struct list_head *head,
		unsigned int nr_pages, int nid, gfp_t gfp_mask)
{
	struct page *page, *next;
	LIST_HEAD(new_pages);

	for (unsigned int i = 0; i < nr_pages; i++) {
		page = alloc_pages_node(nid, gfp_mask | __GFP_RETRY_MAYFAIL, 0);
		if (!page)
			goto free_pages;
		list_add(&page->lru, &new_pages);
	}
	list_splice(&new_pages, head);
	memmap_pages_add(nr_pages);

	return 0;
free_pages:
	list_for_each_entry_safe(page, next, &new_pages, lru)
		__free_page(page);
	return -ENOMEM;
}

static struct page *vmemmap_dup_page_from_list(const struct page *src,
		struct list_head *vmemmap_pages)
{
	struct page *dst = list_first_entry(vmemmap_pages, struct page, lru);

	list_del(&dst->lru);
	copy_page(page_address(dst), page_address(src));

	/* Ensure updates to @dst page are visible before set_pte_at(). */
	smp_wmb();

	return dst;
}

static inline unsigned long folio_vmemmap_size(const struct folio *folio)
{
	return folio_nr_pages(folio) * sizeof(struct page);
}

static inline struct page *folio_vmemmap_shared_tail_page(const struct folio *folio)
{
	return vmemmap_shared_tail_page(folio_order(folio), folio_zone(folio));
}

static int vmemmap_optimize_pte(pte_t *pte, unsigned long addr,
		struct vmemmap_remap_walk *walk)
{
	const struct page *page = (const struct page *)addr;
	const struct folio *folio = page_folio(page);
	struct page *src = pte_page(ptep_get(pte)), *dst;

	if (likely(vmemmap_optimizable_pfn(page_to_pfn(page), folio_order(folio)))) {
		dst = folio_vmemmap_shared_tail_page(folio);
		if (!dst)
			return -ENOMEM;
	} else {
		dst = vmemmap_dup_page_from_list(src, walk->vmemmap_pages);
	}

	list_add(&src->lru, walk->vmemmap_pages);
	/*
	 * Map the shared tail page writable intentionally. A lockless get that
	 * read a non-zero refcount before the remap can still do its cmpxchg
	 * after it. The refcount is zero, so the cmpxchg fails and leaves the
	 * page unchanged.
	 */
	set_pte_at(&init_mm, addr, pte, mk_pte(dst, PAGE_KERNEL));

	return 0;
}

static int vmemmap_restore_pte(pte_t *pte, unsigned long addr,
		struct vmemmap_remap_walk *walk)
{
	const struct page *page = (const struct page *)addr;
	const struct folio *folio = page_folio(page);
	struct page *src = pte_page(ptep_get(pte)), *dst;

	if (unlikely(!vmemmap_optimizable_pfn(page_to_pfn(page), folio_order(folio))))
		return 0;

	VM_WARN_ON_ONCE(folio_vmemmap_shared_tail_page(folio) != src);
	dst = vmemmap_dup_page_from_list(src, walk->vmemmap_pages);
	set_pte_at(&init_mm, addr, pte, mk_pte(dst, PAGE_KERNEL));

	return 0;
}

static inline int vmemmap_restore_range(unsigned long start, unsigned long end,
		struct list_head *vmemmap_pages, unsigned long flags)
{
	struct vmemmap_remap_walk walk = {
		.remap_pte	= vmemmap_restore_pte,
		.vmemmap_pages	= vmemmap_pages,
		.flags		= flags,
	};

	return vmemmap_remap_range(start, end, &walk);
}

/**
 * vmemmap_pteify_folio - convert the vmemmap backing of a folio to PTE mappings
 * @folio:	folio whose vmemmap range should be converted to PTE mappings.
 * @flags:	vmemmap remap control flags passed to vmemmap_remap_walk.
 *
 * Convert mappings covering @folio's vmemmap into PTE mappings. If TLB flushing
 * is skipped, the caller must flush TLBs after all pteify operations are complete
 * before remapping the folio's vmemmap pages.
 *
 * Return: %0 on success, negative error code otherwise.
 */
static int vmemmap_pteify_folio(const struct folio *folio, unsigned long flags)
{
	unsigned long start = (unsigned long)folio_page(folio, 0);
	unsigned long end = start + folio_vmemmap_size(folio);
	struct vmemmap_remap_walk walk = {
		.flags		= flags,
	};

	return vmemmap_remap_range(start, end, &walk);
}

/**
 * vmemmap_optimize_folio - optimize the vmemmap backing of a folio
 * @folio:	folio whose vmemmap pages should be optimized.
 * @vmemmap_pages: list holding replacement pages for the remap and displaced
 *		backing pages for the caller to free.
 * @flags:	vmemmap remap control flags passed to vmemmap_remap_walk.
 *
 * Remap @folio's vmemmap so optimizable entries point at the shared tail page
 * while non-optimizable entries are copied into replacement pages. Caller is
 * responsible for freeing the pages left on the @vmemmap_pages list, even on
 * failure.
 *
 * Return: %0 on success, negative error code otherwise.
 */
static int vmemmap_optimize_folio(const struct folio *folio,
		struct list_head *vmemmap_pages, unsigned long flags)
{
	unsigned long start = (unsigned long)folio_page(folio, 0);
	unsigned long end = start + folio_vmemmap_size(folio);
	gfp_t gfp_mask = GFP_KERNEL | __GFP_NOWARN;
	int ret, nid = folio_nid(folio);
	struct vmemmap_remap_walk walk = {
		.remap_pte	= vmemmap_optimize_pte,
		.vmemmap_pages	= vmemmap_pages,
		.flags		= flags,
	};

	VM_WARN_ON_ONCE_FOLIO(folio_ref_count(folio), folio);

	if (!vmemmap_optimizable_order(folio_order(folio)))
		return -EOPNOTSUPP;

	ret = alloc_vmemmap_page_list(vmemmap_pages, VMEMMAP_OPTIMIZATION_PAGES,
				      nid, gfp_mask);
	if (ret)
		return ret;

	ret = vmemmap_remap_range(start, end, &walk);
	if (!ret || !walk.nr_walked)
		return ret;

	end = start + walk.nr_walked * PAGE_SIZE;
	vmemmap_restore_range(start, end, vmemmap_pages, 0);

	return ret;
}

/**
 * vmemmap_restore_folio - restore the vmemmap backing of a folio
 * @folio:	folio whose vmemmap pages should be restored.
 * @flags:	vmemmap remap control flags passed to vmemmap_remap_walk.
 *
 * Remap @folio's vmemmap so optimizable entries are restored from the shared
 * tail page back to distinct backing pages allocated for @folio, while
 * non-optimizable entries remain unchanged.
 *
 * Return: %0 on success, negative error code otherwise.
 */
static int vmemmap_restore_folio(const struct folio *folio, unsigned long flags)
{
	unsigned long start = (unsigned long)folio_page(folio, 0);
	unsigned long end = start + folio_vmemmap_size(folio);
	int ret, nid = folio_nid(folio);
	LIST_HEAD(vmemmap_pages);

	start += VMEMMAP_OPTIMIZATION_SIZE;
	ret = alloc_vmemmap_page_list(&vmemmap_pages, (end - start) / PAGE_SIZE,
				      nid, GFP_KERNEL);
	if (ret)
		return ret;

	return vmemmap_restore_range(start, end, &vmemmap_pages, flags);
}

static bool vmemmap_optimize_enabled = IS_ENABLED(CONFIG_HUGETLB_PAGE_OPTIMIZE_VMEMMAP_DEFAULT_ON);
static int __init hugetlb_vmemmap_optimize_param(char *buf)
{
	return kstrtobool(buf, &vmemmap_optimize_enabled);
}
early_param("hugetlb_free_vmemmap", hugetlb_vmemmap_optimize_param);

static int __hugetlb_vmemmap_restore_folio(const struct hstate *h,
					   struct folio *folio, unsigned long flags)
{
	int ret;

	VM_WARN_ON_ONCE_FOLIO(!folio_test_hugetlb(folio), folio);
	VM_WARN_ON_ONCE_FOLIO(folio_ref_count(folio), folio);

	if (!folio_test_hugetlb_vmemmap_optimized(folio))
		return 0;

	ret = vmemmap_restore_folio(folio, flags);
	if (!ret)
		folio_clear_hugetlb_vmemmap_optimized(folio);

	return ret;
}

/**
 * hugetlb_vmemmap_restore_folio - restore previously optimized (by
 *				hugetlb_vmemmap_optimize_folio()) vmemmap pages which
 *				will be reallocated and remapped.
 * @h:		struct hstate.
 * @folio:     the folio whose vmemmap pages will be restored.
 *
 * Return: %0 if @folio's vmemmap pages have been reallocated and remapped,
 * negative error code otherwise.
 */
int hugetlb_vmemmap_restore_folio(const struct hstate *h, struct folio *folio)
{
	return __hugetlb_vmemmap_restore_folio(h, folio, 0);
}

/**
 * hugetlb_vmemmap_restore_folios - restore vmemmap for every folio on the list.
 * @h:			hstate.
 * @folio_list:		list of folios.
 * @non_hvo_folios:	Output list of folios for which vmemmap exists.
 *
 * Return: number of folios for which vmemmap was restored, or an error code
 *		if an error was encountered restoring vmemmap for a folio.
 *		Folios that have vmemmap are moved to the non_hvo_folios
 *		list.  Processing of entries stops when the first error is
 *		encountered. The folio that experienced the error and all
 *		non-processed folios will remain on folio_list.
 */
long hugetlb_vmemmap_restore_folios(const struct hstate *h,
					struct list_head *folio_list,
					struct list_head *non_hvo_folios)
{
	struct folio *folio, *t_folio;
	long restored = 0;
	long ret = 0;
	unsigned long flags = VMEMMAP_DEFER_TLB_FLUSH;

	list_for_each_entry_safe(folio, t_folio, folio_list, lru) {
		if (folio_test_hugetlb_vmemmap_optimized(folio)) {
			ret = __hugetlb_vmemmap_restore_folio(h, folio, flags);
			if (ret)
				break;
			restored++;
		}

		/* Add non-optimized folios to output list */
		list_move(&folio->lru, non_hvo_folios);
	}

	if (restored)
		flush_tlb_all();
	if (!ret)
		ret = restored;
	return ret;
}

/* Return true iff a HugeTLB whose vmemmap should and can be optimized. */
static bool vmemmap_should_optimize_folio(const struct hstate *h, struct folio *folio)
{
	if (folio_test_hugetlb_vmemmap_optimized(folio))
		return false;

	if (!READ_ONCE(vmemmap_optimize_enabled))
		return false;

	if (!hugetlb_vmemmap_optimizable(h))
		return false;

	return true;
}

static int __hugetlb_vmemmap_optimize_folio(const struct hstate *h,
					    struct folio *folio,
					    struct list_head *vmemmap_pages,
					    unsigned long flags)
{
	int ret;

	VM_WARN_ON_ONCE_FOLIO(!folio_test_hugetlb(folio), folio);

	if (!vmemmap_should_optimize_folio(h, folio))
		return 0;

	/*
	 * Very Subtle
	 * If VMEMMAP_DEFER_TLB_FLUSH is set, TLB flushing is not performed
	 * immediately after remapping.  As a result, subsequent accesses
	 * and modifications to struct pages associated with the hugetlb
	 * page could be to the OLD struct pages.  Set the vmemmap optimized
	 * flag here so that it is copied to the new head page.  This keeps
	 * the old and new struct pages in sync.
	 * If there is an error during optimization, we will immediately FLUSH
	 * the TLB and clear the flag below.
	 */
	folio_set_hugetlb_vmemmap_optimized(folio);
	ret = vmemmap_optimize_folio(folio, vmemmap_pages, flags);
	if (ret)
		folio_clear_hugetlb_vmemmap_optimized(folio);

	return ret;
}

/**
 * hugetlb_vmemmap_optimize_folio - optimize @folio's vmemmap pages.
 * @h:		struct hstate.
 * @folio:     the folio whose vmemmap pages will be optimized.
 *
 * This function only tries to optimize @folio's vmemmap pages and does not
 * guarantee that the optimization will succeed after it returns. The caller
 * can use folio_test_hugetlb_vmemmap_optimized(@folio) to detect if @folio's
 * vmemmap pages have been optimized.
 */
void hugetlb_vmemmap_optimize_folio(const struct hstate *h, struct folio *folio)
{
	LIST_HEAD(vmemmap_pages);

	__hugetlb_vmemmap_optimize_folio(h, folio, &vmemmap_pages, 0);
	free_vmemmap_page_list(&vmemmap_pages);
}

static int hugetlb_vmemmap_pteify_folio(const struct hstate *h, struct folio *folio,
		unsigned long flags)
{
	if (!vmemmap_should_optimize_folio(h, folio))
		return 0;

	return vmemmap_pteify_folio(folio, flags);
}

void hugetlb_vmemmap_optimize_folios(struct hstate *h, struct list_head *folio_list)
{
	struct folio *folio;
	unsigned long nr_to_optimize = 0;
	LIST_HEAD(vmemmap_pages);
	unsigned long flags = VMEMMAP_DEFER_TLB_FLUSH;

	if (list_empty(folio_list))
		return;

	list_for_each_entry(folio, folio_list, lru) {
		int ret;

		/*
		 * Bootmem gigantic folios may already be marked optimized when
		 * their vmemmap layout was prepared earlier, so skip them here.
		 */
		if (folio_test_hugetlb_vmemmap_optimized(folio))
			continue;

		nr_to_optimize++;

		ret = hugetlb_vmemmap_pteify_folio(h, folio, flags);

		/*
		 * Splitting the PMD requires allocating a page, thus let's fail
		 * early once we encounter the first OOM. No point in retrying
		 * as it can be dynamically done on remap with the memory
		 * we get back from the vmemmap deduplication.
		 */
		if (ret == -ENOMEM)
			break;
	}

	if (!nr_to_optimize)
		return;

	flush_tlb_all();

	list_for_each_entry(folio, folio_list, lru) {
		int ret;

		ret = __hugetlb_vmemmap_optimize_folio(h, folio, &vmemmap_pages, flags);

		/*
		 * Pages to be freed may have been accumulated.  If we
		 * encounter an ENOMEM,  free what we have and try again.
		 * This can occur in the case that both splitting fails
		 * halfway and head page allocation also failed. In this
		 * case __hugetlb_vmemmap_optimize_folio() would free memory
		 * allowing more vmemmap remaps to occur.
		 */
		if (ret == -ENOMEM && !list_empty(&vmemmap_pages)) {
			flush_tlb_all();
			free_vmemmap_page_list(&vmemmap_pages);
			INIT_LIST_HEAD(&vmemmap_pages);
			__hugetlb_vmemmap_optimize_folio(h, folio, &vmemmap_pages, flags);
		}
	}

	flush_tlb_all();
	free_vmemmap_page_list(&vmemmap_pages);
}

void __init hugetlb_vmemmap_optimize_bootmem_page(unsigned long pfn, unsigned int order)
{
	if (!READ_ONCE(vmemmap_optimize_enabled))
		return;

	section_set_compound_order_range(pfn, 1UL << order, order);
}

static const struct ctl_table hugetlb_vmemmap_sysctls[] = {
	{
		.procname	= "hugetlb_optimize_vmemmap",
		.data		= &vmemmap_optimize_enabled,
		.maxlen		= sizeof(vmemmap_optimize_enabled),
		.mode		= 0644,
		.proc_handler	= proc_dobool,
	},
};

static int __init hugetlb_vmemmap_init(void)
{
	const struct hstate *h;

	/* VMEMMAP_OPTIMIZATION_NR_STRUCT_PAGES should cover all used struct pages */
	BUILD_BUG_ON(__NR_USED_SUBPAGE > VMEMMAP_OPTIMIZATION_NR_STRUCT_PAGES);

	for_each_hstate(h) {
		if (hugetlb_vmemmap_optimizable(h)) {
			register_sysctl_init("vm", hugetlb_vmemmap_sysctls);
			break;
		}
	}
	return 0;
}
late_initcall(hugetlb_vmemmap_init);
