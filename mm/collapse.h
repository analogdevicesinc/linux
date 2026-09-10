/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __MM_COLLAPSE_H
#define __MM_COLLAPSE_H

#include <linux/mm.h>
#include <linux/nodemask.h>
#include <linux/pgtable.h>
#include <linux/types.h>

#define COLLAPSE_MAX_PTES_LIMIT		(HPAGE_PMD_NR - 1)
#define COLLAPSE_MIN_MTHP_ORDER		2

enum scan_result {
	SCAN_FAIL,
	SCAN_SUCCEED,
	SCAN_NO_PTE_TABLE,
	SCAN_PMD_MAPPED,
	SCAN_EXCEED_NONE_PTE,
	SCAN_EXCEED_SWAP_PTE,
	SCAN_EXCEED_SHARED_PTE,
	SCAN_PTE_NON_PRESENT,
	SCAN_PTE_UFFD,
	SCAN_PTE_MAPPED_HUGEPAGE,
	SCAN_LACK_REFERENCED_PAGE,
	SCAN_PAGE_NULL,
	SCAN_SCAN_ABORT,
	SCAN_PAGE_COUNT,
	SCAN_PAGE_LRU,
	SCAN_PAGE_LOCK,
	SCAN_PAGE_ANON,
	SCAN_PAGE_LAZYFREE,
	SCAN_PAGE_COMPOUND,
	SCAN_ANY_PROCESS,
	SCAN_VMA_NULL,
	SCAN_VMA_CHECK,
	SCAN_ADDRESS_RANGE,
	SCAN_DEL_PAGE_LRU,
	SCAN_ALLOC_HUGE_PAGE_FAIL,
	SCAN_CGROUP_CHARGE_FAIL,
	SCAN_TRUNCATED,
	SCAN_PAGE_HAS_PRIVATE,
	SCAN_STORE_FAILED,
	SCAN_COPY_MC,
	SCAN_PAGE_FILLED,
	SCAN_PAGE_DIRTY_OR_WRITEBACK,
};

/* What a collapse is allowed to do, decided by the caller that asks for it */
struct collapse_policy {
	/* Limits, stated per PMD; HPAGE_PMD_NR means "no limit" */
	unsigned int max_ptes_none;
	unsigned int max_ptes_swap;
	unsigned int max_ptes_shared;

	/* Take no swapped-out or shared PTE into a sub-PMD collapse */
	bool strict_sub_pmd;

	/* Leave clean lazyfree folios to reclaim rather than collapse them */
	bool skip_lazyfree;

	/* Refuse a range with no sign of use */
	bool require_referenced;

	/* Map the PMD over a file collapse instead of leaving it to a fault */
	bool install_pmd;

	/* Write dirty pages back and retry once instead of refusing them */
	bool writeback_dirty;

	/* How hard to try for a destination folio */
	gfp_t gfp;

	/* Which VMAs are eligible, as thp_vma_allowable_orders() spells it */
	enum tva_type tva_type;
};

struct collapse_control {
	struct collapse_policy policy;

	/* Num pages scanned per node */
	u32 node_load[MAX_NUMNODES];

	/* Num pages scanned (see khugepaged_pages_to_scan) */
	unsigned int progress;

	/* nodemask for allocation fallback */
	nodemask_t alloc_nmask;

	/* Each bit marks a PTE the scan accepted as a collapse source */
	DECLARE_BITMAP(eligible_ptes, MAX_PTRS_PER_PTE);

	/*
	 * What a scan found and the run after it needs.  Live only between the
	 * two, and read by nobody else.
	 *
	 * The file side takes a reference while it still has the VMA, since a
	 * file collapse works on the page cache and never sees one; the run is
	 * what gives it back.  A scan that found the PMD folio already in the
	 * cache leaves only the PTE table to retract.
	 */
	unsigned long scan_orders;
	int scan_referenced;
	int scan_unmapped;
	struct file *scan_file;
	pgoff_t scan_pgoff;
	bool scan_retract_only;
};

/* Which orders a VMA may collapse to, zero when it may not collapse at all */
unsigned long collapse_possible_orders(struct vm_area_struct *vma,
		vm_flags_t vm_flags, enum tva_type tva_flags);

/*
 * A caller states what it allows in cc->policy and then hands over one PTE
 * table's worth of a VMA at a time:
 *
 *     collapse_control_init(cc)         once, before the first table
 *     collapse_scan_pmd(vma, addr, ...) per table
 *     collapse_run_pmd(mm, addr, cc)    when a scan found work
 *     collapse_control_release(cc)      once, when done with the control
 *
 * The caller holds mmap_lock for reading over the scan and passes an address
 * within @vma, aligned to the PTE table the scan is to judge.
 *
 * The scan returns with that lock still held.  It only reads, and almost every
 * table it is offered has nothing to collapse, so a caller walks a whole VMA
 * under the one lock it took to get there.  SCAN_SUCCEED means there is
 * something to collapse; anything else is why there is not.
 *
 * The run is called without the lock and returns without it, taking what it
 * needs in between: what it does -- allocate, isolate, copy, flush -- is slow
 * enough that a writer would wait behind it.  The caller gives the lock up
 * first, and with it @vma and anything derived under it, so a caller carrying
 * on has to look up again with collapse_vma_revalidate().  The run revalidates
 * for itself rather than trusting what the scan saw.
 *
 * A scan that found something has to be run: the file side takes a reference on
 * the file while it still has the VMA to take it from, and the run is what
 * gives it back.
 */
void collapse_control_init(struct collapse_control *cc);
void collapse_control_release(struct collapse_control *cc);
enum scan_result collapse_scan_pmd(struct vm_area_struct *vma,
		unsigned long addr, struct collapse_control *cc,
		unsigned long orders);
enum scan_result collapse_run_pmd(struct mm_struct *mm, unsigned long addr,
		struct collapse_control *cc);
enum scan_result collapse_vma_revalidate(struct mm_struct *mm,
		unsigned long address, bool expect_anon,
		struct vm_area_struct **vmap, struct collapse_control *cc,
		unsigned int order);

#endif	/* __MM_COLLAPSE_H */
