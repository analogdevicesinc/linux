// SPDX-License-Identifier: GPL-2.0
/*
 *	linux/mm/madvise.c
 *
 * Copyright (C) 1999  Linus Torvalds
 * Copyright (C) 2002  Christoph Hellwig
 */

#include <linux/mman.h>
#include <linux/pagemap.h>
#include <linux/syscalls.h>
#include <linux/mempolicy.h>
#include <linux/page-isolation.h>
#include <linux/page_idle.h>
#include <linux/userfaultfd_k.h>
#include <linux/hugetlb.h>
#include <linux/falloc.h>
#include <linux/fadvise.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/mm_inline.h>
#include <linux/mmu_context.h>
#include <linux/string.h>
#include <linux/uio.h>
#include <linux/ksm.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/blk_plug.h>
#include <linux/backing-dev.h>
#include <linux/pagewalk.h>
#include <linux/swap.h>
#include <linux/leafops.h>
#include <linux/shmem_fs.h>
#include <linux/mmu_notifier.h>
#include <linux/swap_ops.h>

#include <asm/tlb.h>

#include "internal.h"
#include "swap.h"

#define __MADV_SET_ANON_VMA_NAME (-1)

/*
 * Maximum number of attempts we make to install guard pages before we give up
 * and return -ERESTARTNOINTR to have userspace try again.
 */
#define MAX_MADVISE_GUARD_RETRIES 3

struct madvise_walk_private {
	struct mmu_gather *tlb;
	bool pageout;
};

enum madvise_lock_mode {
	MADVISE_NO_LOCK,
	MADVISE_MMAP_READ_LOCK,
	MADVISE_MMAP_WRITE_LOCK,
	MADVISE_VMA_READ_LOCK,
};

struct madvise_behavior_range {
	unsigned long start;
	unsigned long end;
};

struct madvise_behavior {
	struct mm_struct *mm;
	int behavior;
	struct mmu_gather *tlb;
	enum madvise_lock_mode lock_mode;
	struct anon_vma_name *anon_name;

	/*
	 * The range over which the behaviour is currently being applied. If
	 * traversing multiple VMAs, this is updated for each.
	 */
	struct madvise_behavior_range range;
	/* The VMA and VMA preceding it (if applicable) currently targeted. */
	struct vm_area_struct *prev;
	struct vm_area_struct *vma;
	bool lock_dropped;
};

#ifdef CONFIG_ANON_VMA_NAME
static int madvise_walk_vmas(struct madvise_behavior *madv_behavior);

struct anon_vma_name *anon_vma_name_alloc(const char *name)
{
	struct anon_vma_name *anon_name;
	size_t count;

	/* Add 1 for NUL terminator at the end of the anon_name->name */
	count = strlen(name) + 1;
	anon_name = kmalloc_flex(*anon_name, name, count);
	if (anon_name) {
		kref_init(&anon_name->kref);
		memcpy(anon_name->name, name, count);
	}

	return anon_name;
}

void anon_vma_name_free(struct kref *kref)
{
	struct anon_vma_name *anon_name =
			container_of(kref, struct anon_vma_name, kref);
	kfree(anon_name);
}

struct anon_vma_name *anon_vma_name(struct vm_area_struct *vma)
{
	vma_assert_stabilised(vma);
	return vma->anon_name;
}

/* mmap_lock should be write-locked */
static int replace_anon_vma_name(struct vm_area_struct *vma,
				 struct anon_vma_name *anon_name)
{
	struct anon_vma_name *orig_name = anon_vma_name(vma);

	if (!anon_name) {
		vma->anon_name = NULL;
		anon_vma_name_put(orig_name);
		return 0;
	}

	if (anon_vma_name_eq(orig_name, anon_name))
		return 0;

	vma->anon_name = anon_vma_name_reuse(anon_name);
	anon_vma_name_put(orig_name);

	return 0;
}
#else /* CONFIG_ANON_VMA_NAME */
static int replace_anon_vma_name(struct vm_area_struct *vma,
				 struct anon_vma_name *anon_name)
{
	if (anon_name)
		return -EINVAL;

	return 0;
}
#endif /* CONFIG_ANON_VMA_NAME */
/*
 * Update the vm_flags or anon_name on region of a vma, splitting it or merging
 * it as necessary. Must be called with mmap_lock held for writing.
 */
static int madvise_update_vma(vm_flags_t new_flags,
		struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;
	vma_flags_t new_vma_flags = legacy_to_vma_flags(new_flags);
	struct madvise_behavior_range *range = &madv_behavior->range;
	struct anon_vma_name *anon_name = madv_behavior->anon_name;
	bool set_new_anon_name = madv_behavior->behavior == __MADV_SET_ANON_VMA_NAME;
	VMA_ITERATOR(vmi, madv_behavior->mm, range->start);

	if (vma_flags_same_mask(&vma->flags, new_vma_flags) &&
	    (!set_new_anon_name ||
	     anon_vma_name_eq(anon_vma_name(vma), anon_name)))
		return 0;

	if (set_new_anon_name)
		vma = vma_modify_name(&vmi, madv_behavior->prev, vma,
			range->start, range->end, anon_name);
	else
		vma = vma_modify_flags(&vmi, madv_behavior->prev, vma,
			range->start, range->end, &new_vma_flags);

	if (IS_ERR(vma))
		return PTR_ERR(vma);

	madv_behavior->vma = vma;

	/* vm_flags is protected by the mmap_lock held in write mode. */
	vma_start_write(vma);
	vma->flags = new_vma_flags;
	/*
	 * If the vma become good for khugepaged to scan,
	 * register it here without waiting a page fault that
	 * may not happen any time soon.
	 */
	if (vma_flags_test(&new_vma_flags, VMA_HUGEPAGE_BIT))
		khugepaged_enter_vma(vma, vma_flags_to_legacy(new_vma_flags));

	if (set_new_anon_name)
		return replace_anon_vma_name(vma, anon_name);

	return 0;
}

#ifdef CONFIG_SWAP
static int swapin_walk_pmd_entry(pmd_t *pmd, unsigned long start,
		unsigned long end, struct mm_walk *walk)
{
	struct vm_area_struct *vma = walk->private;
	struct swap_io_ctx ctx = {};
	pte_t *ptep = NULL;
	spinlock_t *ptl;
	unsigned long addr;

	for (addr = start; addr < end; addr += PAGE_SIZE) {
		pte_t pte;
		softleaf_t entry;
		struct folio *folio;

		if (!ptep++) {
			ptep = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
			if (!ptep)
				break;
		}

		pte = ptep_get(ptep);
		entry = softleaf_from_pte(pte);
		if (unlikely(!softleaf_is_swap(entry)))
			continue;

		pte_unmap_unlock(ptep, ptl);
		ptep = NULL;

		folio = read_swap_cache_async(&ctx, entry, GFP_HIGHUSER_MOVABLE,
					vma, addr);
		if (folio)
			folio_put(folio);
	}

	if (ptep)
		pte_unmap_unlock(ptep, ptl);
	swap_read_submit(&ctx);
	cond_resched();

	return 0;
}

static const struct mm_walk_ops swapin_walk_ops = {
	.pmd_entry		= swapin_walk_pmd_entry,
	.walk_lock		= PGWALK_RDLOCK,
};

static void shmem_swapin_range(struct vm_area_struct *vma,
		unsigned long start, unsigned long end,
		struct address_space *mapping)
{
	XA_STATE(xas, &mapping->i_pages, linear_page_index(vma, start));
	pgoff_t end_index = linear_page_index(vma, end) - 1;
	struct folio *folio;
	struct swap_io_ctx ctx = {};

	rcu_read_lock();
	xas_for_each(&xas, folio, end_index) {
		unsigned long addr;
		swp_entry_t entry;

		if (!xa_is_value(folio))
			continue;
		entry = radix_to_swp_entry(folio);
		/* There might be swapin error entries in shmem mapping. */
		if (!softleaf_is_swap(entry))
			continue;

		addr = vma->vm_start +
			((xas.xa_index - vma_start_pgoff(vma)) << PAGE_SHIFT);
		xas_pause(&xas);
		rcu_read_unlock();

		folio = read_swap_cache_async(&ctx, entry,
				mapping_gfp_mask(mapping), vma, addr);
		if (folio)
			folio_put(folio);

		rcu_read_lock();
	}
	rcu_read_unlock();
	swap_read_submit(&ctx);
}
#endif		/* CONFIG_SWAP */

static void mark_mmap_lock_dropped(struct madvise_behavior *madv_behavior)
{
	VM_WARN_ON_ONCE(madv_behavior->lock_mode == MADVISE_VMA_READ_LOCK);
	madv_behavior->lock_dropped = true;
}

/*
 * Schedule all required I/O operations.  Do not wait for completion.
 */
static long madvise_willneed(struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;
	struct mm_struct *mm = madv_behavior->mm;
	struct file *file = vma->vm_file;
	unsigned long start = madv_behavior->range.start;
	unsigned long end = madv_behavior->range.end;
	loff_t offset;

#ifdef CONFIG_SWAP
	if (vma_is_cow_mapping(vma) && vma->anon_vma) {
		walk_page_range_vma(vma, start, end, &swapin_walk_ops, vma);
		lru_add_drain(); /* Push any new pages onto the LRU now */
	}
	if (!file)
		return 0;

	if (shmem_mapping(file->f_mapping)) {
		shmem_swapin_range(vma, start, end, file->f_mapping);
		lru_add_drain(); /* Push any new pages onto the LRU now */
		return 0;
	}
#else
	if (!file)
		return -EBADF;
#endif

	if (IS_DAX(file_inode(file))) {
		/* no bad return value, but ignore advice */
		return 0;
	}

	/*
	 * Filesystem's fadvise may need to take various locks.  We need to
	 * explicitly grab a reference because the vma (and hence the
	 * vma's reference to the file) can go away as soon as we drop
	 * mmap_lock.
	 */
	mark_mmap_lock_dropped(madv_behavior);
	get_file(file);
	offset = (loff_t)(start - vma->vm_start)
			+ ((loff_t)vma_start_pgoff(vma) << PAGE_SHIFT);
	mmap_read_unlock(mm);
	vfs_fadvise(file, offset, end - start, POSIX_FADV_WILLNEED);
	fput(file);
	mmap_read_lock(mm);
	return 0;
}

static inline bool can_do_file_pageout(struct vm_area_struct *vma)
{
	if (!vma->vm_file)
		return false;
	/*
	 * paging out pagecache only for non-anonymous mappings that correspond
	 * to the files the calling process could (if tried) open for writing;
	 * otherwise we'd be including shared non-exclusive mappings, which
	 * opens a side channel.
	 */
	return file_owner_or_capable(vma->vm_file) ||
	       file_permission(vma->vm_file, MAY_WRITE) == 0;
}

static inline int madvise_folio_pte_batch(unsigned long addr, unsigned long end,
					  struct folio *folio, pte_t *ptep,
					  pte_t *ptentp)
{
	int max_nr = (end - addr) / PAGE_SIZE;

	return folio_pte_batch_flags(folio, NULL, ptep, ptentp, max_nr,
				     FPB_MERGE_YOUNG_DIRTY);
}

static int madvise_cold_or_pageout_pte_range(pmd_t *pmd,
				unsigned long addr, unsigned long end,
				struct mm_walk *walk)
{
	struct madvise_walk_private *private = walk->private;
	struct mmu_gather *tlb = private->tlb;
	bool pageout = private->pageout;
	struct mm_struct *mm = tlb->mm;
	struct vm_area_struct *vma = walk->vma;
	pte_t *start_pte, *pte, ptent;
	spinlock_t *ptl;
	struct folio *folio = NULL;
	LIST_HEAD(folio_list);
	bool pageout_anon_only_filter;
	unsigned int batch_count = 0;
	int nr;

	if (fatal_signal_pending(current))
		return -EINTR;

	pageout_anon_only_filter = pageout && !vma_is_anonymous(vma) &&
					!can_do_file_pageout(vma);

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	if (pmd_trans_huge(*pmd)) {
		pmd_t orig_pmd;
		unsigned long next = pmd_addr_end(addr, end);

		tlb_change_page_size(tlb, HPAGE_PMD_SIZE);
		ptl = pmd_trans_huge_lock(pmd, vma);
		if (!ptl)
			return 0;

		orig_pmd = *pmd;
		if (unlikely(!pmd_present(orig_pmd))) {
			VM_WARN_ON_ONCE(!pmd_is_migration_entry(orig_pmd) &&
					!pmd_is_device_private_entry(orig_pmd));
			goto huge_unlock;
		}

		folio = vm_normal_folio_pmd(vma, addr, orig_pmd);
		if (!folio)
			goto huge_unlock;

		if (folio_is_zone_device(folio))
			goto huge_unlock;

		/* Do not interfere with other mappings of this folio */
		if (folio_maybe_mapped_shared(folio))
			goto huge_unlock;

		if (pageout_anon_only_filter && !folio_test_anon(folio))
			goto huge_unlock;

		if (next - addr != HPAGE_PMD_SIZE) {
			int err;

			if (!folio_trylock(folio))
				goto huge_unlock;
			folio_get(folio);
			spin_unlock(ptl);
			err = split_folio(folio);
			folio_unlock(folio);
			folio_put(folio);
			if (!err)
				goto regular_folio;
			return 0;
		}

		if (!pageout && pmd_young(orig_pmd)) {
			pmdp_test_and_clear_young(vma, addr, pmd);
			tlb_remove_pmd_tlb_entry(tlb, pmd, addr);
		}

		folio_clear_referenced(folio);
		folio_test_clear_young(folio);
		if (folio_test_active(folio))
			folio_set_workingset(folio);
		if (pageout) {
			if (folio_isolate_lru(folio)) {
				if (folio_test_unevictable(folio))
					folio_putback_lru(folio);
				else
					list_add(&folio->lru, &folio_list);
			}
		} else
			folio_deactivate(folio);
huge_unlock:
		spin_unlock(ptl);
		if (pageout)
			reclaim_pages(&folio_list);
		cond_resched();
		return 0;
	}

regular_folio:
#endif
	tlb_change_page_size(tlb, PAGE_SIZE);
restart:
	start_pte = pte = pte_offset_map_lock(vma->vm_mm, pmd, addr, &ptl);
	if (!start_pte)
		goto out;
	flush_tlb_batched_pending(mm);
	lazy_mmu_mode_enable();
	for (; addr < end; pte += nr, addr += nr * PAGE_SIZE) {
		nr = 1;
		ptent = ptep_get(pte);

		if (++batch_count == SWAP_CLUSTER_MAX) {
			batch_count = 0;
			if (need_resched()) {
				lazy_mmu_mode_disable();
				pte_unmap_unlock(start_pte, ptl);
				cond_resched();
				goto restart;
			}
		}

		if (pte_none(ptent))
			continue;

		if (!pte_present(ptent))
			continue;

		folio = vm_normal_folio(vma, addr, ptent);
		if (!folio || folio_is_zone_device(folio))
			continue;

		/*
		 * If we encounter a large folio, only split it if it is not
		 * fully mapped within the range we are operating on. Otherwise
		 * leave it as is so that it can be swapped out whole. If we
		 * fail to split a folio, leave it in place and advance to the
		 * next pte in the range.
		 */
		if (folio_test_large(folio)) {
			nr = madvise_folio_pte_batch(addr, end, folio, pte, &ptent);
			if (nr < folio_nr_pages(folio)) {
				int err;

				if (folio_maybe_mapped_shared(folio))
					continue;
				if (pageout_anon_only_filter && !folio_test_anon(folio))
					continue;
				if (!folio_trylock(folio))
					continue;
				folio_get(folio);
				lazy_mmu_mode_disable();
				pte_unmap_unlock(start_pte, ptl);
				start_pte = NULL;
				err = split_folio(folio);
				folio_unlock(folio);
				folio_put(folio);
				start_pte = pte =
					pte_offset_map_lock(mm, pmd, addr, &ptl);
				if (!start_pte)
					break;
				flush_tlb_batched_pending(mm);
				lazy_mmu_mode_enable();
				if (!err)
					nr = 0;
				continue;
			}
		}

		/*
		 * Do not interfere with other mappings of this folio and
		 * non-LRU folio. If we have a large folio at this point, we
		 * know it is fully mapped so if its mapcount is the same as its
		 * number of pages, it must be exclusive.
		 */
		if (!folio_test_lru(folio) ||
		    folio_mapcount(folio) != folio_nr_pages(folio))
			continue;

		if (pageout_anon_only_filter && !folio_test_anon(folio))
			continue;

		if (!pageout && pte_young(ptent)) {
			clear_young_dirty_ptes(vma, addr, pte, nr,
					       CYDP_CLEAR_YOUNG);
			tlb_remove_tlb_entries(tlb, pte, nr, addr);
		}

		/*
		 * We are deactivating a folio for accelerating reclaiming.
		 * VM couldn't reclaim the folio unless we clear PG_young.
		 * As a side effect, it makes confuse idle-page tracking
		 * because they will miss recent referenced history.
		 */
		folio_clear_referenced(folio);
		folio_test_clear_young(folio);
		if (folio_test_active(folio))
			folio_set_workingset(folio);
		if (pageout) {
			if (folio_isolate_lru(folio)) {
				if (folio_test_unevictable(folio))
					folio_putback_lru(folio);
				else
					list_add(&folio->lru, &folio_list);
			}
		} else
			folio_deactivate(folio);
	}

out:
	if (start_pte) {
		lazy_mmu_mode_disable();
		pte_unmap_unlock(start_pte, ptl);
	}
	if (pageout)
		reclaim_pages(&folio_list);
	cond_resched();

	return 0;
}

static const struct mm_walk_ops cold_walk_ops = {
	.pmd_entry = madvise_cold_or_pageout_pte_range,
	.walk_lock = PGWALK_RDLOCK,
};

static void madvise_cold_page_range(struct mmu_gather *tlb,
		struct madvise_behavior *madv_behavior)

{
	struct vm_area_struct *vma = madv_behavior->vma;
	struct madvise_behavior_range *range = &madv_behavior->range;
	struct madvise_walk_private walk_private = {
		.pageout = false,
		.tlb = tlb,
	};

	tlb_start_vma(tlb, vma);
	walk_page_range_vma(vma, range->start, range->end, &cold_walk_ops,
			&walk_private);
	tlb_end_vma(tlb, vma);
}

static inline bool can_madv_lru_vma(struct vm_area_struct *vma)
{
	return !(vma->vm_flags & (VM_LOCKED|VM_PFNMAP|VM_HUGETLB));
}

static long madvise_cold(struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;
	struct mmu_gather tlb;

	if (!can_madv_lru_vma(vma))
		return -EINVAL;

	lru_add_drain();
	tlb_gather_mmu(&tlb, madv_behavior->mm);
	madvise_cold_page_range(&tlb, madv_behavior);
	tlb_finish_mmu(&tlb);

	return 0;
}

static void madvise_pageout_page_range(struct mmu_gather *tlb,
		struct vm_area_struct *vma,
		struct madvise_behavior_range *range)
{
	struct madvise_walk_private walk_private = {
		.pageout = true,
		.tlb = tlb,
	};

	tlb_start_vma(tlb, vma);
	walk_page_range_vma(vma, range->start, range->end, &cold_walk_ops,
			    &walk_private);
	tlb_end_vma(tlb, vma);
}

static long madvise_pageout(struct madvise_behavior *madv_behavior)
{
	struct mmu_gather tlb;
	struct vm_area_struct *vma = madv_behavior->vma;

	if (!can_madv_lru_vma(vma))
		return -EINVAL;

	/*
	 * If the VMA belongs to a private file mapping, there can be private
	 * dirty pages which can be paged out if even this process is neither
	 * owner nor write capable of the file. We allow private file mappings
	 * further to pageout dirty anon pages.
	 */
	if (!vma_is_anonymous(vma) && (!can_do_file_pageout(vma) &&
				(vma->vm_flags & VM_MAYSHARE)))
		return 0;

	lru_add_drain();
	tlb_gather_mmu(&tlb, madv_behavior->mm);
	madvise_pageout_page_range(&tlb, vma, &madv_behavior->range);
	tlb_finish_mmu(&tlb);

	return 0;
}

static int madvise_free_pte_range(pmd_t *pmd, unsigned long addr,
				unsigned long end, struct mm_walk *walk)

{
	const cydp_t cydp_flags = CYDP_CLEAR_YOUNG | CYDP_CLEAR_DIRTY;
	struct mmu_gather *tlb = walk->private;
	struct mm_struct *mm = tlb->mm;
	struct vm_area_struct *vma = walk->vma;
	spinlock_t *ptl;
	pte_t *start_pte, *pte, ptent;
	struct folio *folio;
	int nr_swap = 0;
	unsigned long next;
	int nr, max_nr;

	next = pmd_addr_end(addr, end);
	if (pmd_trans_huge(*pmd))
		if (madvise_free_huge_pmd(tlb, vma, pmd, addr, next))
			return 0;

	tlb_change_page_size(tlb, PAGE_SIZE);
	start_pte = pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
	if (!start_pte)
		return 0;
	flush_tlb_batched_pending(mm);
	lazy_mmu_mode_enable();
	for (; addr != end; pte += nr, addr += PAGE_SIZE * nr) {
		nr = 1;
		ptent = ptep_get(pte);

		if (pte_none(ptent))
			continue;
		/*
		 * If the pte has swp_entry, just clear page table to
		 * prevent swap-in which is more expensive rather than
		 * (page allocation + zeroing).
		 */
		if (!pte_present(ptent)) {
			softleaf_t entry = softleaf_from_pte(ptent);

			if (softleaf_is_swap(entry)) {
				max_nr = (end - addr) / PAGE_SIZE;
				nr = swap_pte_batch(pte, max_nr, ptent);
				nr_swap -= nr;
				swap_put_entries_direct(entry, nr);
				clear_nonpresent_ptes(mm, addr, pte, nr);
			} else if (softleaf_is_hwpoison(entry) ||
				   softleaf_is_poison_marker(entry)) {
				pte_clear(mm, addr, pte);
			}
			continue;
		}

		folio = vm_normal_folio(vma, addr, ptent);
		if (!folio || folio_is_zone_device(folio))
			continue;

		/*
		 * If we encounter a large folio, only split it if it is not
		 * fully mapped within the range we are operating on. Otherwise
		 * leave it as is so that it can be marked as lazyfree. If we
		 * fail to split a folio, leave it in place and advance to the
		 * next pte in the range.
		 */
		if (folio_test_large(folio)) {
			nr = madvise_folio_pte_batch(addr, end, folio, pte, &ptent);
			if (nr < folio_nr_pages(folio)) {
				int err;

				if (folio_maybe_mapped_shared(folio))
					continue;
				if (!folio_trylock(folio))
					continue;
				folio_get(folio);
				lazy_mmu_mode_disable();
				pte_unmap_unlock(start_pte, ptl);
				start_pte = NULL;
				err = split_folio(folio);
				folio_unlock(folio);
				folio_put(folio);
				pte = pte_offset_map_lock(mm, pmd, addr, &ptl);
				start_pte = pte;
				if (!start_pte)
					break;
				flush_tlb_batched_pending(mm);
				lazy_mmu_mode_enable();
				if (!err)
					nr = 0;
				continue;
			}
		}

		if (folio_test_swapcache(folio) || folio_test_dirty(folio)) {
			if (!folio_trylock(folio))
				continue;
			/*
			 * If we have a large folio at this point, we know it is
			 * fully mapped so if its mapcount is the same as its
			 * number of pages, it must be exclusive.
			 */
			if (folio_mapcount(folio) != folio_nr_pages(folio)) {
				folio_unlock(folio);
				continue;
			}

			if (folio_test_swapcache(folio) &&
			    !folio_free_swap(folio)) {
				folio_unlock(folio);
				continue;
			}

			folio_clear_dirty(folio);
			folio_unlock(folio);
		}

		if (pte_young(ptent) || pte_dirty(ptent)) {
			clear_young_dirty_ptes(vma, addr, pte, nr, cydp_flags);
			tlb_remove_tlb_entries(tlb, pte, nr, addr);
		}
		folio_mark_lazyfree(folio);
	}

	if (nr_swap)
		add_mm_counter(mm, MM_SWAPENTS, nr_swap);
	if (start_pte) {
		lazy_mmu_mode_disable();
		pte_unmap_unlock(start_pte, ptl);
	}
	cond_resched();

	return 0;
}

static inline enum page_walk_lock get_walk_lock(enum madvise_lock_mode mode)
{
	switch (mode) {
	case MADVISE_VMA_READ_LOCK:
		return PGWALK_VMA_RDLOCK_VERIFY;
	case MADVISE_MMAP_READ_LOCK:
		return PGWALK_RDLOCK;
	default:
		/* Other modes don't require fixing up the walk_lock */
		WARN_ON_ONCE(1);
		return PGWALK_RDLOCK;
	}
}

static int madvise_free_single_vma(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;
	struct vm_area_struct *vma = madv_behavior->vma;
	struct mmu_notifier_range range = {
		.start = madv_behavior->range.start,
		.end = madv_behavior->range.end,
	};
	struct mmu_gather *tlb = madv_behavior->tlb;
	struct mm_walk_ops walk_ops = {
		.pmd_entry		= madvise_free_pte_range,
	};

	/* MADV_FREE works for only anon vma at the moment */
	if (!vma_is_anonymous(vma))
		return -EINVAL;

	mmu_notifier_range_init(&range, MMU_NOTIFY_CLEAR, 0, mm,
				range.start, range.end);

	lru_add_drain();
	update_hiwater_rss(mm);

	mmu_notifier_invalidate_range_start(&range);
	tlb_start_vma(tlb, vma);
	walk_ops.walk_lock = get_walk_lock(madv_behavior->lock_mode);
	walk_page_range_vma(vma, range.start, range.end,
			&walk_ops, tlb);
	tlb_end_vma(tlb, vma);
	mmu_notifier_invalidate_range_end(&range);
	return 0;
}

/*
 * Application no longer needs these pages.  If the pages are dirty,
 * it's OK to just throw them away.  The app will be more careful about
 * data it wants to keep.  Be sure to free swap resources too.  The
 * zap_vma_range call sets things up for shrink_active_list to actually
 * free these pages later if no one else has touched them in the meantime,
 * although we could add these pages to a global reuse list for
 * shrink_active_list to pick up before reclaiming other pages.
 *
 * NB: This interface discards data rather than pushes it out to swap,
 * as some implementations do.  This has performance implications for
 * applications like large transactional databases which want to discard
 * pages in anonymous maps after committing to backing store the data
 * that was kept in them.  There is no reason to write this data out to
 * the swap area if the application is discarding it.
 *
 * An interface that causes the system to free clean pages and flush
 * dirty pages is already available as msync(MS_INVALIDATE).
 */
static long madvise_dontneed_single_vma(struct madvise_behavior *madv_behavior)

{
	struct madvise_behavior_range *range = &madv_behavior->range;
	struct zap_details details = {
		.reclaim_pt = true,
	};

	zap_vma_range_batched(madv_behavior->tlb, madv_behavior->vma,
			      range->start, range->end - range->start, &details);
	return 0;
}

static
bool madvise_dontneed_free_valid_vma(struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;
	int behavior = madv_behavior->behavior;
	struct madvise_behavior_range *range = &madv_behavior->range;

	if (!vma_is_hugetlb(vma)) {
		unsigned int forbidden = VM_PFNMAP;

		if (behavior != MADV_DONTNEED_LOCKED)
			forbidden |= VM_LOCKED;

		return !(vma->vm_flags & forbidden);
	}

	if (behavior != MADV_DONTNEED && behavior != MADV_DONTNEED_LOCKED)
		return false;
	if (range->start & ~huge_page_mask(hstate_vma(vma)))
		return false;

	/*
	 * Madvise callers expect the length to be rounded up to PAGE_SIZE
	 * boundaries, and may be unaware that this VMA uses huge pages.
	 * Avoid unexpected data loss by rounding down the number of
	 * huge pages freed.
	 */
	range->end = ALIGN_DOWN(range->end, huge_page_size(hstate_vma(vma)));

	return true;
}

static long madvise_dontneed_free(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;
	struct madvise_behavior_range *range = &madv_behavior->range;
	int behavior = madv_behavior->behavior;

	if (!madvise_dontneed_free_valid_vma(madv_behavior))
		return -EINVAL;

	if (range->start == range->end)
		return 0;

	if (!userfaultfd_remove(madv_behavior->vma, range->start, range->end)) {
		struct vm_area_struct *vma;

		mark_mmap_lock_dropped(madv_behavior);
		mmap_read_lock(mm);
		madv_behavior->vma = vma = vma_lookup(mm, range->start);
		if (!vma)
			return -ENOMEM;
		/*
		 * Potential end adjustment for hugetlb vma is OK as
		 * the check below keeps end within vma.
		 */
		if (!madvise_dontneed_free_valid_vma(madv_behavior))
			return -EINVAL;
		if (range->end > vma->vm_end) {
			/*
			 * Don't fail if end > vma->vm_end. If the old
			 * vma was split while the mmap_lock was
			 * released the effect of the concurrent
			 * operation may not cause madvise() to
			 * have an undefined result. There may be an
			 * adjacent next vma that we'll walk
			 * next. userfaultfd_remove() will generate an
			 * UFFD_EVENT_REMOVE repetition on the
			 * end-vma->vm_end range, but the manager can
			 * handle a repetition fine.
			 */
			range->end = vma->vm_end;
		}
		/*
		 * If the memory region between start and end was
		 * originally backed by 4kB pages and then remapped to
		 * be backed by hugepages while mmap_lock was dropped,
		 * the adjustment for hugetlb vma above may have rounded
		 * end down to the start address.
		 */
		if (range->start == range->end)
			return 0;
		VM_WARN_ON(range->start > range->end);
	}

	if (behavior == MADV_DONTNEED || behavior == MADV_DONTNEED_LOCKED)
		return madvise_dontneed_single_vma(madv_behavior);
	else if (behavior == MADV_FREE)
		return madvise_free_single_vma(madv_behavior);
	else
		return -EINVAL;
}

static long madvise_populate(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;
	const bool write = madv_behavior->behavior == MADV_POPULATE_WRITE;
	int locked = 1;
	unsigned long start = madv_behavior->range.start;
	unsigned long end = madv_behavior->range.end;
	long pages;

	while (start < end) {
		/* Populate (prefault) page tables readable/writable. */
		pages = faultin_page_range(mm, start, end, write, &locked);
		if (!locked) {
			mmap_read_lock(mm);
			locked = 1;
		}
		if (pages < 0) {
			switch (pages) {
			case -EINTR:
				return -EINTR;
			case -EINVAL: /* Incompatible mappings / permissions. */
				return -EINVAL;
			case -EHWPOISON:
				return -EHWPOISON;
			case -EFAULT: /* VM_FAULT_SIGBUS or VM_FAULT_SIGSEGV */
				return -EFAULT;
			default:
				pr_warn_once("%s: unhandled return value: %ld\n",
					     __func__, pages);
				fallthrough;
			case -ENOMEM: /* No VMA or out of memory. */
				return -ENOMEM;
			}
		}
		start += pages * PAGE_SIZE;
	}
	return 0;
}

/*
 * Application wants to free up the pages and associated backing store.
 * This is effectively punching a hole into the middle of a file.
 */
static long madvise_remove(struct madvise_behavior *madv_behavior)
{
	loff_t offset;
	int error;
	struct file *f;
	struct mm_struct *mm = madv_behavior->mm;
	struct vm_area_struct *vma = madv_behavior->vma;
	unsigned long start = madv_behavior->range.start;
	unsigned long end = madv_behavior->range.end;

	mark_mmap_lock_dropped(madv_behavior);

	if (vma->vm_flags & VM_LOCKED)
		return -EINVAL;

	f = vma->vm_file;

	if (!f || !f->f_mapping || !f->f_mapping->host) {
			return -EINVAL;
	}

	if (!vma_is_shared_maywrite(vma))
		return -EACCES;

	offset = (loff_t)(start - vma->vm_start)
			+ ((loff_t)vma_start_pgoff(vma) << PAGE_SHIFT);

	/*
	 * Filesystem's fallocate may need to take i_rwsem.  We need to
	 * explicitly grab a reference because the vma (and hence the
	 * vma's reference to the file) can go away as soon as we drop
	 * mmap_lock.
	 */
	get_file(f);
	if (userfaultfd_remove(vma, start, end)) {
		/* mmap_lock was not released by userfaultfd_remove() */
		mmap_read_unlock(mm);
	}
	error = vfs_fallocate(f,
				FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE,
				offset, end - start);
	fput(f);
	mmap_read_lock(mm);
	return error;
}

static bool is_valid_guard_vma(const struct vm_area_struct *vma,
			       bool allow_locked)
{
	/*
	 * A user could lock after setting a guard range but that's fine as
	 * they'd not be able to fault in. The issue arises when we try to zap
	 * existing locked VMAs. We don't want to do that.
	 */
	if (!allow_locked && vma_test(vma, VMA_LOCKED_BIT))
		return false;
	/*
	 * Guard regions require a VMA whose page tables are managed solely by
	 * core mm, so the VMA must be mm-backed (excluding hugetlb, in which
	 * guard markers cannot be installed).
	 */
	if (!vma_is_mm_backed(vma) || vma_is_hugetlb(vma))
		return false;

	return true;
}

static bool is_guard_pte_marker(pte_t ptent)
{
	const softleaf_t entry = softleaf_from_pte(ptent);

	return softleaf_is_guard_marker(entry);
}

static int guard_install_pud_entry(pud_t *pud, unsigned long addr,
				   unsigned long next, struct mm_walk *walk)
{
	pud_t pudval = pudp_get(pud);

	/* If huge return >0 so we abort the operation + zap. */
	return pud_trans_huge(pudval);
}

static int guard_install_pmd_entry(pmd_t *pmd, unsigned long addr,
				   unsigned long next, struct mm_walk *walk)
{
	pmd_t pmdval = pmdp_get(pmd);

	/* If huge return >0 so we abort the operation + zap. */
	return pmd_trans_huge(pmdval);
}

static int guard_install_pte_entry(pte_t *pte, unsigned long addr,
				   unsigned long next, struct mm_walk *walk)
{
	pte_t pteval = ptep_get(pte);
	unsigned long *nr_pages = (unsigned long *)walk->private;

	/* If there is already a guard page marker, we have nothing to do. */
	if (is_guard_pte_marker(pteval)) {
		(*nr_pages)++;

		return 0;
	}

	/* If populated return >0 so we abort the operation + zap. */
	return 1;
}

static int guard_install_set_pte(unsigned long addr, unsigned long next,
				 pte_t *ptep, struct mm_walk *walk)
{
	unsigned long *nr_pages = (unsigned long *)walk->private;

	/* Simply install a PTE marker, this causes segfault on access. */
	*ptep = make_pte_marker(PTE_MARKER_GUARD);
	(*nr_pages)++;

	return 0;
}

static long madvise_guard_install(struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;
	struct madvise_behavior_range *range = &madv_behavior->range;
	struct mm_walk_ops walk_ops = {
		.pud_entry	= guard_install_pud_entry,
		.pmd_entry	= guard_install_pmd_entry,
		.pte_entry	= guard_install_pte_entry,
		.install_pte	= guard_install_set_pte,
		.walk_lock	= get_walk_lock(madv_behavior->lock_mode),
	};
	long err;
	int i;

	if (!is_valid_guard_vma(vma, /* allow_locked = */false))
		return -EINVAL;

	/*
	 * Set atomically under read lock. All pertinent readers will need to
	 * acquire an mmap/VMA write lock to read it. All remaining readers may
	 * or may not see the flag set, but we don't care.
	 */
	vma_set_atomic_flag(vma, VMA_MAYBE_GUARD_BIT);

	/*
	 * If anonymous and we are establishing page tables the VMA ought to
	 * have an anon rmap associated with it.
	 *
	 * We will hold an mmap read lock if this is necessary, this is checked
	 * as part of the VMA lock logic.
	 */
	if (vma_is_anonymous(vma)) {
		VM_WARN_ON_ONCE(!vma_has_anon_rmap(vma) &&
				madv_behavior->lock_mode != MADVISE_MMAP_READ_LOCK);

		err = anon_vma_prepare(vma);
		if (err)
			return err;
	}

	/*
	 * Optimistically try to install the guard marker pages first. If any
	 * non-guard pages or THP huge pages are encountered, give up and zap
	 * the range before trying again.
	 *
	 * We try a few times before giving up and releasing back to userland to
	 * loop around, releasing locks in the process to avoid contention.
	 *
	 * This would only happen due to races with e.g. page faults or
	 * khugepaged.
	 *
	 * In most cases we should simply install the guard markers immediately
	 * with no zap or looping.
	 */
	for (i = 0; i < MAX_MADVISE_GUARD_RETRIES; i++) {
		unsigned long nr_pages = 0;

		/* Returns < 0 on error, == 0 if success, > 0 if zap needed. */
		if (madv_behavior->lock_mode == MADVISE_VMA_READ_LOCK)
			err = walk_page_range_vma_unsafe(madv_behavior->vma,
					range->start, range->end, &walk_ops,
					&nr_pages);
		else
			err = walk_page_range_mm_unsafe(vma->vm_mm, range->start,
					range->end, &walk_ops, &nr_pages);
		if (err < 0)
			return err;

		if (err == 0) {
			unsigned long nr_expected_pages =
				PHYS_PFN(range->end - range->start);

			VM_WARN_ON(nr_pages != nr_expected_pages);
			return 0;
		}

		/*
		 * OK some of the range have non-guard pages mapped, zap
		 * them. This leaves existing guard pages in place.
		 */
		zap_vma_range(vma, range->start, range->end - range->start);
	}

	/*
	 * We were unable to install the guard pages, return to userspace and
	 * immediately retry, relieving lock contention.
	 */
	return restart_syscall();
}

static int guard_remove_pud_entry(pud_t *pud, unsigned long addr,
				  unsigned long next, struct mm_walk *walk)
{
	pud_t pudval = pudp_get(pud);

	/* If huge, cannot have guard pages present, so no-op - skip. */
	if (pud_trans_huge(pudval))
		walk->action = ACTION_CONTINUE;

	return 0;
}

static int guard_remove_pmd_entry(pmd_t *pmd, unsigned long addr,
				  unsigned long next, struct mm_walk *walk)
{
	pmd_t pmdval = pmdp_get(pmd);

	/* If huge, cannot have guard pages present, so no-op - skip. */
	if (pmd_trans_huge(pmdval))
		walk->action = ACTION_CONTINUE;

	return 0;
}

static int guard_remove_pte_entry(pte_t *pte, unsigned long addr,
				  unsigned long next, struct mm_walk *walk)
{
	pte_t ptent = ptep_get(pte);

	if (is_guard_pte_marker(ptent)) {
		/* Simply clear the PTE marker. */
		pte_clear(walk->mm, addr, pte);
		update_mmu_cache(walk->vma, addr, pte);
	}

	return 0;
}

static long madvise_guard_remove(struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;
	struct madvise_behavior_range *range = &madv_behavior->range;
	struct mm_walk_ops wallk_ops = {
		.pud_entry = guard_remove_pud_entry,
		.pmd_entry = guard_remove_pmd_entry,
		.pte_entry = guard_remove_pte_entry,
		.walk_lock = get_walk_lock(madv_behavior->lock_mode),
	};

	/*
	 * We're ok with removing guards in mlock()'d ranges, as this is a
	 * non-destructive action.
	 */
	if (!is_valid_guard_vma(vma, /* allow_locked = */true))
		return -EINVAL;

	return walk_page_range_vma(vma, range->start, range->end,
				   &wallk_ops, NULL);
}

#ifdef CONFIG_64BIT
/* Does the madvise operation result in discarding of mapped data? */
static bool is_discard(int behavior)
{
	switch (behavior) {
	case MADV_FREE:
	case MADV_DONTNEED:
	case MADV_DONTNEED_LOCKED:
	case MADV_REMOVE:
	case MADV_DONTFORK:
	case MADV_WIPEONFORK:
	case MADV_GUARD_INSTALL:
		return true;
	}

	return false;
}

/*
 * We are restricted from madvise()'ing mseal()'d VMAs only in very particular
 * circumstances - discarding of data from read-only anonymous SEALED mappings.
 *
 * This is because users cannot trivially discard data from these VMAs, and may
 * only do so via an appropriate madvise() call.
 */
static bool can_madvise_modify(struct madvise_behavior *madv_behavior)
{
	struct vm_area_struct *vma = madv_behavior->vma;

	/* If the VMA isn't sealed we're good. */
	if (!vma_is_sealed(vma))
		return true;

	/* For a sealed VMA, we only care about discard operations. */
	if (!is_discard(madv_behavior->behavior))
		return true;

	/*
	 * We explicitly permit all file-backed mappings, whether MAP_SHARED or
	 * MAP_PRIVATE.
	 *
	 * The latter causes some complications. Because now, one can mmap()
	 * read/write a MAP_PRIVATE mapping, write to it, then mprotect()
	 * read-only, mseal() and a discard will be permitted.
	 *
	 * However, in order to avoid issues with potential use of madvise(...,
	 * MADV_DONTNEED) of mseal()'d .text mappings we, for the time being,
	 * permit this.
	 */
	if (!vma_is_anonymous(vma))
		return true;

	/* If the user could write to the mapping anyway, then this is fine. */
	if ((vma->vm_flags & VM_WRITE) &&
	    arch_vma_access_permitted(vma, /* write= */ true,
			/* execute= */ false, /* foreign= */ false))
		return true;

	/* Otherwise, we are not permitted to perform this operation. */
	return false;
}
#else
static bool can_madvise_modify(struct madvise_behavior *madv_behavior)
{
	return true;
}
#endif

/*
 * Apply an madvise behavior to a region of a vma.  madvise_update_vma
 * will handle splitting a vm area into separate areas, each area with its own
 * behavior.
 */
static int madvise_vma_behavior(struct madvise_behavior *madv_behavior)
{
	int behavior = madv_behavior->behavior;
	struct vm_area_struct *vma = madv_behavior->vma;
	vm_flags_t new_flags = vma->vm_flags;
	struct madvise_behavior_range *range = &madv_behavior->range;
	int error;

	if (unlikely(!can_madvise_modify(madv_behavior)))
		return -EPERM;

	switch (behavior) {
	case MADV_REMOVE:
		return madvise_remove(madv_behavior);
	case MADV_WILLNEED:
		return madvise_willneed(madv_behavior);
	case MADV_COLD:
		return madvise_cold(madv_behavior);
	case MADV_PAGEOUT:
		return madvise_pageout(madv_behavior);
	case MADV_FREE:
	case MADV_DONTNEED:
	case MADV_DONTNEED_LOCKED:
		return madvise_dontneed_free(madv_behavior);
	case MADV_COLLAPSE:
		return madvise_collapse(vma, range->start, range->end,
			&madv_behavior->lock_dropped);
	case MADV_GUARD_INSTALL:
		return madvise_guard_install(madv_behavior);
	case MADV_GUARD_REMOVE:
		return madvise_guard_remove(madv_behavior);

	/* The below behaviours update VMAs via madvise_update_vma(). */

	case MADV_NORMAL:
		new_flags = new_flags & ~VM_RAND_READ & ~VM_SEQ_READ;
		break;
	case MADV_SEQUENTIAL:
		new_flags = (new_flags & ~VM_RAND_READ) | VM_SEQ_READ;
		break;
	case MADV_RANDOM:
		new_flags = (new_flags & ~VM_SEQ_READ) | VM_RAND_READ;
		break;
	case MADV_DONTFORK:
		new_flags |= VM_DONTCOPY;
		break;
	case MADV_DOFORK:
		if (!vma_is_mm_backed(vma) || vma_is_hugetlb(vma))
			return -EINVAL;
		new_flags &= ~VM_DONTCOPY;
		break;
	case MADV_WIPEONFORK:
		/* MADV_WIPEONFORK is only supported on anonymous memory. */
		if (vma->vm_file || new_flags & VM_SHARED)
			return -EINVAL;
		new_flags |= VM_WIPEONFORK;
		break;
	case MADV_KEEPONFORK:
		if (new_flags & VM_DROPPABLE)
			return -EINVAL;
		new_flags &= ~VM_WIPEONFORK;
		break;
	case MADV_DONTDUMP:
		new_flags |= VM_DONTDUMP;
		break;
	case MADV_DODUMP:
		/* Only mm-backed memory can be meaningfully dumped. */
		if (!vma_is_mm_backed(vma))
			return -EINVAL;
		/* Core mm may discard droppable memory at any time. */
		if (new_flags & VM_DROPPABLE)
			return -EINVAL;
		new_flags &= ~VM_DONTDUMP;
		break;
	case MADV_MERGEABLE:
	case MADV_UNMERGEABLE:
		error = ksm_madvise(vma, range->start, range->end,
				behavior, &new_flags);
		if (error)
			goto out;
		break;
	case MADV_HUGEPAGE:
	case MADV_NOHUGEPAGE:
		error = hugepage_madvise(vma, &new_flags, behavior);
		if (error)
			goto out;
		break;
	case __MADV_SET_ANON_VMA_NAME:
		/* Only anonymous mappings can be named */
		if (vma->vm_file && !vma_is_anon_shmem(vma))
			return -EBADF;
		break;
	}

	/* This is a write operation.*/
	VM_WARN_ON_ONCE(madv_behavior->lock_mode != MADVISE_MMAP_WRITE_LOCK);

	error = madvise_update_vma(new_flags, madv_behavior);
out:
	/*
	 * madvise() returns EAGAIN if kernel resources, such as
	 * slab, are temporarily unavailable.
	 */
	if (error == -ENOMEM)
		error = -EAGAIN;
	return error;
}

#ifdef CONFIG_MEMORY_FAILURE
/*
 * Error injection support for memory error handling.
 */
static int madvise_inject_error(struct madvise_behavior *madv_behavior)
{
	unsigned long size;
	unsigned long start = madv_behavior->range.start;
	unsigned long end = madv_behavior->range.end;

	if (!capable(CAP_SYS_ADMIN))
		return -EPERM;

	for (; start < end; start += size) {
		unsigned long pfn;
		struct page *page;
		int ret;

		ret = get_user_pages_fast(start, 1, 0, &page);
		if (ret != 1)
			return ret;
		pfn = page_to_pfn(page);

		/*
		 * When soft offlining hugepages, after migrating the page
		 * we dissolve it, therefore in the second loop "page" will
		 * no longer be a compound page.
		 */
		size = page_size(compound_head(page));

		if (madv_behavior->behavior == MADV_SOFT_OFFLINE) {
			pr_info("Soft offlining pfn %#lx at process virtual address %#lx\n",
				 pfn, start);
			ret = soft_offline_page(pfn, MF_COUNT_INCREASED);
		} else {
			pr_info("Injecting memory failure for pfn %#lx at process virtual address %#lx\n",
				 pfn, start);
			ret = memory_failure(pfn, MF_ACTION_REQUIRED | MF_COUNT_INCREASED | MF_SW_SIMULATED);
			if (ret == -EOPNOTSUPP)
				ret = 0;
		}

		if (ret)
			return ret;
	}

	return 0;
}

static bool is_memory_failure(struct madvise_behavior *madv_behavior)
{
	switch (madv_behavior->behavior) {
	case MADV_HWPOISON:
	case MADV_SOFT_OFFLINE:
		return true;
	default:
		return false;
	}
}

#else

static int madvise_inject_error(struct madvise_behavior *madv_behavior)
{
	return 0;
}

static bool is_memory_failure(struct madvise_behavior *madv_behavior)
{
	return false;
}

#endif	/* CONFIG_MEMORY_FAILURE */

static bool
madvise_behavior_valid(int behavior)
{
	switch (behavior) {
	case MADV_DOFORK:
	case MADV_DONTFORK:
	case MADV_NORMAL:
	case MADV_SEQUENTIAL:
	case MADV_RANDOM:
	case MADV_REMOVE:
	case MADV_WILLNEED:
	case MADV_DONTNEED:
	case MADV_DONTNEED_LOCKED:
	case MADV_FREE:
	case MADV_COLD:
	case MADV_PAGEOUT:
	case MADV_POPULATE_READ:
	case MADV_POPULATE_WRITE:
#ifdef CONFIG_KSM
	case MADV_MERGEABLE:
	case MADV_UNMERGEABLE:
#endif
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	case MADV_HUGEPAGE:
	case MADV_NOHUGEPAGE:
	case MADV_COLLAPSE:
#endif
	case MADV_DONTDUMP:
	case MADV_DODUMP:
	case MADV_WIPEONFORK:
	case MADV_KEEPONFORK:
	case MADV_GUARD_INSTALL:
	case MADV_GUARD_REMOVE:
#ifdef CONFIG_MEMORY_FAILURE
	case MADV_SOFT_OFFLINE:
	case MADV_HWPOISON:
#endif
		return true;

	default:
		return false;
	}
}

/* Can we invoke process_madvise() on a remote mm for the specified behavior? */
static bool process_madvise_remote_valid(int behavior)
{
	switch (behavior) {
	case MADV_COLD:
	case MADV_PAGEOUT:
	case MADV_WILLNEED:
	case MADV_COLLAPSE:
		return true;
	default:
		return false;
	}
}

/* Does this operation invoke anon_vma_prepare()? */
static bool prepares_anon_vma(int behavior)
{
	switch (behavior) {
	case MADV_GUARD_INSTALL:
		return true;
	default:
		return false;
	}
}

/*
 * We have acquired a VMA read lock, is the VMA valid to be madvise'd under VMA
 * read lock only now we have a VMA to examine?
 */
static bool is_vma_lock_sufficient(struct vm_area_struct *vma,
		struct madvise_behavior *madv_behavior)
{
	/* Must span only a single VMA.*/
	if (madv_behavior->range.end > vma->vm_end)
		return false;
	/* Remote processes unsupported. */
	if (current->mm != vma->vm_mm)
		return false;
	/* Userfaultfd unsupported. */
	if (userfaultfd_armed(vma))
		return false;
	/*
	 * anon_vma_prepare() explicitly requires an mmap lock for
	 * serialisation, so we cannot use a VMA lock in this case.
	 *
	 * Note we might race with the anon rmap being assigned, however this
	 * makes this check overly paranoid which is safe.
	 */
	if (vma_is_anonymous(vma) &&
	    prepares_anon_vma(madv_behavior->behavior) && !vma_has_anon_rmap(vma))
		return false;

	return true;
}

/*
 * Try to acquire a VMA read lock if possible.
 *
 * We only support this lock over a single VMA, which the input range must
 * span either partially or fully.
 *
 * This function always returns with an appropriate lock held. If a VMA read
 * lock could be acquired, we return true and set madv_behavior state
 * accordingly.
 *
 * If a VMA read lock could not be acquired, we return false and expect caller to
 * fallback to mmap lock behaviour.
 */
static bool try_vma_read_lock(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;
	struct vm_area_struct *vma;

	vma = lock_vma_under_rcu(mm, madv_behavior->range.start);
	if (!vma)
		goto take_mmap_read_lock;

	if (!is_vma_lock_sufficient(vma, madv_behavior)) {
		vma_end_read(vma);
		goto take_mmap_read_lock;
	}

	madv_behavior->vma = vma;
	return true;

take_mmap_read_lock:
	mmap_read_lock(mm);
	madv_behavior->lock_mode = MADVISE_MMAP_READ_LOCK;
	return false;
}

/*
 * Walk the vmas in range [start,end), and call the madvise_vma_behavior
 * function on each one.  The function will get start and end parameters that
 * cover the overlap between the current vma and the original range.  Any
 * unmapped regions in the original range will result in this function returning
 * -ENOMEM while still calling the madvise_vma_behavior function on all of the
 * existing vmas in the range.  Must be called with the mmap_lock held for
 * reading or writing.
 */
static
int madvise_walk_vmas(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;
	struct madvise_behavior_range *range = &madv_behavior->range;
	/* range is updated to span each VMA, so store end of entire range. */
	unsigned long last_end = range->end;
	int unmapped_error = 0;
	int error;
	struct vm_area_struct *prev, *vma;

	/*
	 * If VMA read lock is supported, apply madvise to a single VMA
	 * tentatively, avoiding walking VMAs.
	 */
	if (madv_behavior->lock_mode == MADVISE_VMA_READ_LOCK &&
	    try_vma_read_lock(madv_behavior)) {
		error = madvise_vma_behavior(madv_behavior);
		vma_end_read(madv_behavior->vma);
		return error;
	}

	vma = find_vma_prev(mm, range->start, &prev);
	if (vma && range->start > vma->vm_start)
		prev = vma;

	for (;;) {
		/* Still start < end. */
		if (!vma)
			return -ENOMEM;

		/* Here start < (last_end|vma->vm_end). */
		if (range->start < vma->vm_start) {
			/*
			 * This indicates a gap between VMAs in the input
			 * range. This does not cause the operation to abort,
			 * rather we simply return -ENOMEM to indicate that this
			 * has happened, but carry on.
			 */
			unmapped_error = -ENOMEM;
			range->start = vma->vm_start;
			if (range->start >= last_end)
				break;
		}

		/* Here vma->vm_start <= range->start < (last_end|vma->vm_end) */
		range->end = min(vma->vm_end, last_end);

		/* Here vma->vm_start <= range->start < range->end <= (last_end|vma->vm_end). */
		madv_behavior->prev = prev;
		madv_behavior->vma = vma;
		error = madvise_vma_behavior(madv_behavior);
		if (error)
			return error;
		if (madv_behavior->lock_dropped) {
			/* We dropped the mmap lock, we can't ref the VMA. */
			prev = NULL;
			vma = NULL;
			madv_behavior->lock_dropped = false;
		} else {
			vma = madv_behavior->vma;
			prev = vma;
		}

		if (vma && range->end < vma->vm_end)
			range->end = vma->vm_end;
		if (range->end >= last_end)
			break;

		vma = find_vma(mm, vma ? vma->vm_end : range->end);
		range->start = range->end;
	}

	return unmapped_error;
}

/*
 * Any behaviour which results in changes to the vma->vm_flags needs to
 * take mmap_lock for writing. Others, which simply traverse vmas, need
 * to only take it for reading.
 */
static enum madvise_lock_mode get_lock_mode(struct madvise_behavior *madv_behavior)
{
	if (is_memory_failure(madv_behavior))
		return MADVISE_NO_LOCK;

	switch (madv_behavior->behavior) {
	case MADV_REMOVE:
	case MADV_WILLNEED:
	case MADV_COLD:
	case MADV_PAGEOUT:
	case MADV_POPULATE_READ:
	case MADV_POPULATE_WRITE:
	case MADV_COLLAPSE:
		return MADVISE_MMAP_READ_LOCK;
	case MADV_GUARD_INSTALL:
	case MADV_GUARD_REMOVE:
	case MADV_DONTNEED:
	case MADV_DONTNEED_LOCKED:
	case MADV_FREE:
		return MADVISE_VMA_READ_LOCK;
	default:
		return MADVISE_MMAP_WRITE_LOCK;
	}
}

static int madvise_lock(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;
	enum madvise_lock_mode lock_mode = get_lock_mode(madv_behavior);

	switch (lock_mode) {
	case MADVISE_NO_LOCK:
		break;
	case MADVISE_MMAP_WRITE_LOCK:
		if (mmap_write_lock_killable(mm))
			return -EINTR;
		break;
	case MADVISE_MMAP_READ_LOCK:
		mmap_read_lock(mm);
		break;
	case MADVISE_VMA_READ_LOCK:
		/* We will acquire the lock per-VMA in madvise_walk_vmas(). */
		break;
	}

	madv_behavior->lock_mode = lock_mode;
	return 0;
}

static void madvise_unlock(struct madvise_behavior *madv_behavior)
{
	struct mm_struct *mm = madv_behavior->mm;

	switch (madv_behavior->lock_mode) {
	case  MADVISE_NO_LOCK:
		return;
	case MADVISE_MMAP_WRITE_LOCK:
		mmap_write_unlock(mm);
		break;
	case MADVISE_MMAP_READ_LOCK:
		mmap_read_unlock(mm);
		break;
	case MADVISE_VMA_READ_LOCK:
		/* We will drop the lock per-VMA in madvise_walk_vmas(). */
		break;
	}

	madv_behavior->lock_mode = MADVISE_NO_LOCK;
}

static bool madvise_batch_tlb_flush(int behavior)
{
	switch (behavior) {
	case MADV_DONTNEED:
	case MADV_DONTNEED_LOCKED:
	case MADV_FREE:
		return true;
	default:
		return false;
	}
}

static void madvise_init_tlb(struct madvise_behavior *madv_behavior)
{
	if (madvise_batch_tlb_flush(madv_behavior->behavior))
		tlb_gather_mmu(madv_behavior->tlb, madv_behavior->mm);
}

static void madvise_finish_tlb(struct madvise_behavior *madv_behavior)
{
	if (madvise_batch_tlb_flush(madv_behavior->behavior))
		tlb_finish_mmu(madv_behavior->tlb);
}

/**
 * check_input_range() - Check if the requested range is valid.
 * @start:	Start address of madvise-requested address range.
 * @len_in:	Length of madvise-requested address range.
 *
 * Returns: 0 if the input range is valid, otherwise an error code.
 */
static int check_input_range(unsigned long start, size_t len_in)
{
	size_t len;

	if (!PAGE_ALIGNED(start))
		return -EINVAL;
	len = PAGE_ALIGN(len_in);

	/* Check to see whether len was rounded up from small -ve to zero */
	if (len_in && !len)
		return -EINVAL;

	if (start + len < start)
		return -EINVAL;

	return 0;
}

static bool is_madvise_populate(struct madvise_behavior *madv_behavior)
{
	switch (madv_behavior->behavior) {
	case MADV_POPULATE_READ:
	case MADV_POPULATE_WRITE:
		return true;
	default:
		return false;
	}
}

/*
 * untagged_addr_remote() assumes mmap_lock is already held. On
 * architectures like x86 and RISC-V, tagging is tricky because each
 * mm may have a different tagging mask. However, we might only hold
 * the per-VMA lock (currently only local processes are supported),
 * so untagged_addr is used to avoid the mmap_lock assertion for
 * local processes.
 */
static inline unsigned long get_untagged_addr(struct mm_struct *mm,
		unsigned long start)
{
	return current->mm == mm ? untagged_addr(start) :
				   untagged_addr_remote(mm, start);
}

static int madvise_do_behavior(unsigned long start, size_t len_in,
		struct madvise_behavior *madv_behavior)
{
	struct blk_plug plug;
	int error;
	struct madvise_behavior_range *range = &madv_behavior->range;

	if (is_memory_failure(madv_behavior)) {
		range->start = start;
		range->end = start + len_in;
		return madvise_inject_error(madv_behavior);
	}

	range->start = get_untagged_addr(madv_behavior->mm, start);
	range->end = range->start + PAGE_ALIGN(len_in);

	blk_start_plug(&plug);
	if (is_madvise_populate(madv_behavior))
		error = madvise_populate(madv_behavior);
	else
		error = madvise_walk_vmas(madv_behavior);
	blk_finish_plug(&plug);
	return error;
}

/*
 * The madvise(2) system call.
 *
 * Applications can use madvise() to advise the kernel how it should
 * handle paging I/O in this VM area.  The idea is to help the kernel
 * use appropriate read-ahead and caching techniques.  The information
 * provided is advisory only, and can be safely disregarded by the
 * kernel without affecting the correct operation of the application.
 *
 * behavior values:
 *  MADV_NORMAL - the default behavior is to read clusters.  This
 *		results in some read-ahead and read-behind.
 *  MADV_RANDOM - the system should read the minimum amount of data
 *		on any access, since it is unlikely that the appli-
 *		cation will need more than what it asks for.
 *  MADV_SEQUENTIAL - pages in the given range will probably be accessed
 *		once, so they can be aggressively read ahead, and
 *		can be freed soon after they are accessed.
 *  MADV_WILLNEED - the application is notifying the system to read
 *		some pages ahead.
 *  MADV_DONTNEED - the application is finished with the given range,
 *		so the kernel can free resources associated with it.
 *  MADV_FREE - the application marks pages in the given range as lazy free,
 *		where actual purges are postponed until memory pressure happens.
 *  MADV_REMOVE - the application wants to free up the given range of
 *		pages and associated backing store.
 *  MADV_DONTFORK - omit this area from child's address space when forking:
 *		typically, to avoid COWing pages pinned by get_user_pages().
 *  MADV_DOFORK - cancel MADV_DONTFORK: no longer omit this area when forking.
 *  MADV_WIPEONFORK - present the child process with zero-filled memory in this
 *              range after a fork.
 *  MADV_KEEPONFORK - undo the effect of MADV_WIPEONFORK
 *  MADV_HWPOISON - trigger memory error handler as if the given memory range
 *		were corrupted by unrecoverable hardware memory failure.
 *  MADV_SOFT_OFFLINE - try to soft-offline the given range of memory.
 *  MADV_MERGEABLE - the application recommends that KSM try to merge pages in
 *		this area with pages of identical content from other such areas.
 *  MADV_UNMERGEABLE- cancel MADV_MERGEABLE: no longer merge pages with others.
 *  MADV_HUGEPAGE - the application wants to back the given range by transparent
 *		huge pages in the future. Existing pages might be coalesced and
 *		new pages might be allocated as THP.
 *  MADV_NOHUGEPAGE - mark the given range as not worth being backed by
 *		transparent huge pages so the existing pages will not be
 *		coalesced into THP and new pages will not be allocated as THP.
 *  MADV_COLLAPSE - synchronously coalesce pages into new THP.
 *  MADV_DONTDUMP - the application wants to prevent pages in the given range
 *		from being included in its core dump.
 *  MADV_DODUMP - cancel MADV_DONTDUMP: no longer exclude from core dump.
 *  MADV_COLD - the application is not expected to use this memory soon,
 *		deactivate pages in this range so that they can be reclaimed
 *		easily if memory pressure happens.
 *  MADV_PAGEOUT - the application is not expected to use this memory soon,
 *		page out the pages in this range immediately.
 *  MADV_POPULATE_READ - populate (prefault) page tables readable by
 *		triggering read faults if required
 *  MADV_POPULATE_WRITE - populate (prefault) page tables writable by
 *		triggering write faults if required
 *
 * return values:
 *  zero    - success
 *  -EINVAL - start + len < 0, start is not page-aligned,
 *		"behavior" is not a valid value, or application
 *		is attempting to release locked or shared pages,
 *		or the specified address range includes file, Huge TLB,
 *		MAP_SHARED or VMPFNMAP range.
 *  -ENOMEM - addresses in the specified range are not currently
 *		mapped, or are outside the AS of the process.
 *  -EIO    - an I/O error occurred while paging in data.
 *  -EBADF  - map exists, but area maps something that isn't a file.
 *  -EAGAIN - a kernel resource was temporarily unavailable.
 *  -EPERM  - memory is sealed.
 */
int do_madvise(struct mm_struct *mm, unsigned long start, size_t len_in, int behavior)
{
	int error;
	struct mmu_gather tlb;
	struct madvise_behavior madv_behavior = {
		.mm = mm,
		.behavior = behavior,
		.tlb = &tlb,
	};

	if (!madvise_behavior_valid(behavior))
		return -EINVAL;

	error = check_input_range(start, len_in);
	if (error || !len_in)
		return error;

	error = madvise_lock(&madv_behavior);
	if (error)
		return error;
	madvise_init_tlb(&madv_behavior);
	error = madvise_do_behavior(start, len_in, &madv_behavior);
	madvise_finish_tlb(&madv_behavior);
	madvise_unlock(&madv_behavior);

	return error;
}

/**
 * sys_madvise - Give advice about use of memory
 * @start: Starting virtual address of the range to advise on
 * @len_in: Length of the range in bytes
 * @behavior: Advice (a MADV_* constant) the kernel should apply to the range
 *
 * long-desc: Provides the kernel with advice or directions about the address
 *   range starting at start and extending for len_in bytes. The advice is
 *   selected by behavior, which is one of the MADV_* constants defined in
 *   <sys/mman.h>. The behaviors fall into three groups. The hint group
 *   updates VMA flags (MADV_NORMAL, MADV_RANDOM, MADV_SEQUENTIAL,
 *   MADV_DONTFORK, MADV_DOFORK, MADV_DONTDUMP, MADV_DODUMP, MADV_WIPEONFORK,
 *   MADV_KEEPONFORK, MADV_MERGEABLE, MADV_UNMERGEABLE, MADV_HUGEPAGE,
 *   MADV_NOHUGEPAGE). The immediate-action group performs work synchronously
 *   while preserving page contents (MADV_WILLNEED, MADV_COLD, MADV_PAGEOUT,
 *   MADV_POPULATE_READ, MADV_POPULATE_WRITE, MADV_COLLAPSE, MADV_GUARD_REMOVE,
 *   MADV_SOFT_OFFLINE). The destructive group discards, replaces or
 *   invalidates page contents (MADV_DONTNEED, MADV_DONTNEED_LOCKED, MADV_FREE,
 *   MADV_REMOVE, MADV_GUARD_INSTALL, MADV_HWPOISON). MADV_GUARD_INSTALL
 *   belongs to the destructive group because it zaps any existing pages in
 *   the range before installing PTE guard markers.
 *
 *   start must be page-aligned; len_in is rounded up to the next page
 *   boundary internally. Once those validation checks pass, a zero-length
 *   range succeeds without performing work. The kernel rejects ranges that
 *   wrap (start + PAGE_ALIGN(len_in) < start) and ranges where len_in is
 *   non-zero but rounds up to zero. Address tagging bits are stripped
 *   from start before VMA lookup for every behavior except MADV_HWPOISON
 *   and MADV_SOFT_OFFLINE, which receive the raw start value because they
 *   bypass the VMA walk entirely.
 *
 *   The kernel return value reports whether any error condition was
 *   encountered, not whether the requested work was performed. The
 *   relationship between the return code and the work done varies by
 *   handler:
 *
 *     - Hint behaviors set or clear VMA flags; how the flag is used later
 *       depends on the behavior. MADV_DONTFORK / MADV_DOFORK set or clear
 *       VM_DONTCOPY, which dup_mmap() honors, so the child does not get the
 *       mapping. MADV_MERGEABLE / MADV_UNMERGEABLE control whether KSM scans
 *       the VMA. MADV_NOHUGEPAGE blocks the fault-time, MADV_COLLAPSE and
 *       khugepaged THP paths for the VMA. MADV_HUGEPAGE makes the VMA
 *       eligible for THP when the transparent_hugepage mode is "madvise" and
 *       increases defrag effort; it does not force allocation, which still
 *       depends on the global mode, VMA suitability, defrag GFP policy, and
 *       allocation or memcg-charge success. MADV_WIPEONFORK does not wipe
 *       pages at fork time; the child VMA's pages are not copied, and the
 *       child sees zero-filled pages when it first touches them.
 *       MADV_KEEPONFORK clears VM_WIPEONFORK. MADV_DONTDUMP / MADV_DODUMP set
 *       or clear VM_DONTDUMP, although always_dump_vma() still includes gate,
 *       vm_ops-named or arch-named VMAs in a core dump. MADV_NORMAL /
 *       MADV_RANDOM / MADV_SEQUENTIAL set or clear VM_RAND_READ / VM_SEQ_READ,
 *       which only steer read-ahead.
 *
 *     - Walk-and-skip handlers (MADV_COLD, MADV_PAGEOUT, MADV_FREE,
 *       MADV_GUARD_REMOVE) traverse the range and silently skip pages or
 *       PMDs that fail per-page preconditions (absent, special, device,
 *       shared, non-LRU, unsplittable, locked, etc.), returning 0 even
 *       when most or all pages were skipped.
 *
 *     - Bulk-backend handlers delegate the requested range to a single
 *       backend call: MADV_DONTNEED and MADV_DONTNEED_LOCKED to
 *       zap_vma_range_batched(), MADV_REMOVE to vfs_fallocate(),
 *       MADV_WILLNEED on regular files to vfs_fadvise(). The backend's
 *       return is propagated for MADV_REMOVE and discarded for
 *       MADV_WILLNEED; DAX files short-circuit MADV_WILLNEED entirely.
 *
 *     - Stop-on-error handlers (MADV_POPULATE_READ, MADV_POPULATE_WRITE,
 *       MADV_SOFT_OFFLINE) walk the range but surface the first per-page
 *       failure as an errno (-EHWPOISON, -EFAULT, -ENOMEM, ...) rather
 *       than skipping silently.
 *
 *     - Hybrid handlers combine modes: MADV_WILLNEED walks for anonymous
 *       and shmem ranges but bulk-calls vfs_fadvise() for regular files;
 *       MADV_COLLAPSE walks PMD-by-PMD and tracks the last scan failure
 *       so transient skips coexist with terminal errors;
 *       MADV_GUARD_INSTALL walks to install markers and re-walks after
 *       zap_vma_range() to clear pre-existing pages, retrying up to
 *       MAX_MADVISE_GUARD_RETRIES; MADV_HWPOISON walks pages but folds
 *       memory_failure()'s -EOPNOTSUPP back to 0.
 *
 *   Applications that need to know whether a specific page was acted on
 *   must verify the result through other means (e.g. /proc/[pid]/smaps,
 *   page faults, read-after-write).
 *
 *   On success, madvise() returns 0; unlike read(2) and write(2) it has no
 *   notion of partial completion at the syscall boundary. When the range
 *   spans multiple VMAs, the kernel applies the advice to each in turn. For
 *   the behaviors that walk VMAs (all except MADV_POPULATE_*, MADV_HWPOISON
 *   and MADV_SOFT_OFFLINE), an unmapped gap inside the range causes the call
 *   to return -ENOMEM after processing the mapped portions, rather than
 *   aborting at the gap, although the walk stops at the first per-VMA error.
 *   MADV_POPULATE_* stops at the first failure, and MADV_HWPOISON and
 *   MADV_SOFT_OFFLINE bypass the walk, so an unmapped address gives -EFAULT.
 *
 *   POSIX defines posix_madvise(3) for a portable subset (POSIX_MADV_NORMAL,
 *   _RANDOM, _SEQUENTIAL, _WILLNEED, _DONTNEED). Linux MADV_DONTNEED is
 *   destructive: it discards the contents of the affected anonymous pages and
 *   subsequent reads return zero. POSIX permits but does not require
 *   destruction, so portable code that needs the POSIX semantics should use
 *   posix_madvise(3) instead.
 *
 * contexts: process, sleepable
 *
 * param: start
 *   type: uint, input
 *   constraint-type: page_aligned
 *   cdesc: Starting virtual address of the range. Must be aligned to
 *     PAGE_SIZE. An unaligned start always returns -EINVAL, even when
 *     len_in is zero. Address tag bits, where supported by the architecture,
 *     are cleared via untagged_addr() before the range is interpreted, with
 *     the exception of MADV_HWPOISON and MADV_SOFT_OFFLINE, which receive
 *     the raw start value because they bypass the VMA walk.
 *
 * param: len_in
 *   type: uint, input
 *   cdesc: Length of the range in bytes. Internally rounded up to a multiple
 *     of PAGE_SIZE. A len_in of 0 is accepted and the call is a no-op that
 *     returns 0. A non-zero len_in that rounds up to 0 (i.e. wraps around)
 *     returns -EINVAL, as does a range whose end (start + PAGE_ALIGN(len_in))
 *     would wrap below start.
 *
 * param: behavior
 *   type: int, input
 *   cdesc: One of the MADV_* constants from <sys/mman.h>. See the long
 *     description above for the full list and the three semantic groups
 *     (hint, immediate-action, destructive). Behaviors gated by Kconfig
 *     (KSM, transparent hugepage, memory failure) return -EINVAL when the
 *     underlying support is disabled. A few architectures (notably alpha)
 *     renumber values; portable code should always use the symbolic names.
 *
 * return:
 *   type: int
 *   check-type: exact
 *   success: 0
 *   desc: On success, returns 0. On error, returns a negative error code.
 *     There is no partial-success indication; either the entire processed
 *     range succeeded, or an error is returned and an unspecified prefix of
 *     the range may have been advised.
 *
 * error: EINVAL, Invalid argument
 *   desc: Invalid input (unknown or Kconfig-disabled MADV_*, unaligned start,
 *     range wrap, non-zero len_in rounding up to zero) or a VMA filter.
 *     DONTNEED/FREE reject VM_PFNMAP, VM_LOCKED (not DONTNEED_LOCKED) and
 *     misaligned hugetlb; FREE non-anonymous; WIPEONFORK file or shared; REMOVE
 *     VM_LOCKED or no file; COLD/PAGEOUT LOCKED/PFNMAP/HUGETLB; DOFORK
 *     VM_SPECIAL; KEEPONFORK VM_DROPPABLE; DODUMP SPECIAL/DROPPABLE; GUARD_*
 *     SPECIAL/HUGETLB (INSTALL also LOCKED); COLLAPSE if not possible;
 *     POPULATE_* on bad permissions.
 *
 * error: ENOMEM, Cannot allocate memory
 *   desc: For VMA-walking behaviors, a gap between mapped VMAs inside the range
 *     gives -ENOMEM after the mapped subranges have been processed, unless a
 *     per-VMA error ends the walk first. MADV_POPULATE_* stops at the first
 *     failure and returns -ENOMEM when the region has no VMA or
 *     faultin_page_range() exhausts memory. MADV_COLLAPSE returns -ENOMEM when
 *     its struct collapse_control cannot be allocated, and when
 *     madvise_collapse_errno() maps SCAN_ALLOC_HUGE_PAGE_FAIL (no hugepage
 *     available) to it.
 *
 * error: EAGAIN, Resource temporarily unavailable
 *   desc: For the VMA-flag-mutating behaviors, an internal -ENOMEM from VMA
 *     splitting is translated to -EAGAIN before being returned to userspace,
 *     advising the caller that a transient kernel resource shortage
 *     prevented the update. Also returned by MADV_COLLAPSE via
 *     madvise_collapse_errno() for transient scan failures (folio lock
 *     contention, LRU isolation failure, dirty/writeback) where retrying
 *     the call may succeed.
 *
 * error: EIO, Input/output error
 *   desc: For MADV_REMOVE, an I/O error from the underlying filesystem's
 *     FALLOC_FL_PUNCH_HOLE handler is propagated back as -EIO, and
 *     MADV_SOFT_OFFLINE returns -EIO when soft_offline_page() cannot handle
 *     the page. MADV_HWPOISON reports an unhandled page as -EBUSY; -EIO can
 *     reach it only from a ZONE_DEVICE pagemap's ->memory_failure()
 *     callback. MADV_WILLNEED and MADV_PAGEOUT do not surface filesystem or
 *     device I/O errors: vfs_fadvise() returns are discarded by
 *     madvise_willneed() and the pageout walk is invoked through a void
 *     helper, so transient I/O failures during read-ahead or page-out are
 *     silently dropped.
 *
 * error: EBADF, Bad file descriptor
 *   desc: Returned by MADV_WILLNEED when applied to a non-file-backed VMA
 *     and the kernel was built without CONFIG_SWAP, so there is neither a
 *     file to read-ahead from nor a swap device to fault from.
 *
 * error: EACCES, Permission denied
 *   desc: Returned by MADV_REMOVE when the target VMA fails
 *     vma_is_shared_maywrite(), which needs both VM_SHARED and VM_MAYWRITE.
 *     Private file mappings and shared mappings of files not opened for
 *     writing are refused, while a PROT_READ MAP_SHARED mapping of a file
 *     opened O_RDWR is accepted. Punching a hole through a refused mapping
 *     would either be invisible to other mappers or bypass file write
 *     permission.
 *
 * error: EPERM, Operation not permitted
 *   desc: Returned in two situations. First, MADV_HWPOISON and
 *     MADV_SOFT_OFFLINE require CAP_SYS_ADMIN; the inject-error handler
 *     refuses non-privileged callers. Second, on 64-bit kernels, a discard
 *     operation (MADV_FREE, MADV_DONTNEED, MADV_DONTNEED_LOCKED, MADV_REMOVE,
 *     MADV_DONTFORK, MADV_WIPEONFORK, MADV_GUARD_INSTALL) is refused on a
 *     read-only anonymous VMA that has been sealed with mseal(2), to prevent
 *     bypassing the seal by discarding mapped data.
 *
 * error: EINTR, Interrupted system call
 *   desc: Returned when a fatal signal is delivered while the call is
 *     waiting to acquire the mmap write lock for a VMA-flag-mutating
 *     behavior (mmap_write_lock_killable() returns -EINTR), or when
 *     MADV_POPULATE_READ/MADV_POPULATE_WRITE is interrupted while faulting
 *     in pages (faultin_page_range() returns -EINTR). Only a fatal signal
 *     interrupts these waits, so the task is being killed and user space
 *     does not normally see the error.
 *
 * error: EHWPOISON, Memory page has hardware error
 *   desc: MADV_POPULATE_READ or MADV_POPULATE_WRITE encountered a page that
 *     has been marked as containing a hardware-detected memory error and
 *     could not be faulted in. MADV_HWPOISON also returns it when
 *     memory_failure() finds the page already poisoned.
 *
 * error: EFAULT, Bad address
 *   desc: MADV_POPULATE_READ or MADV_POPULATE_WRITE attempted to fault in a
 *     page whose mapping raised VM_FAULT_SIGBUS or VM_FAULT_SIGSEGV (for
 *     example, a file-backed page beyond the end of the file).
 *     MADV_HWPOISON and MADV_SOFT_OFFLINE return a get_user_pages_fast()
 *     failure, typically -EFAULT for an unmapped address.
 *
 * error: EBUSY, Device or resource busy
 *   desc: Returned by MADV_COLLAPSE via madvise_collapse_errno() in two
 *     specific scan-failure modes: SCAN_CGROUP_CHARGE_FAIL (the new
 *     hugepage cannot be charged to the memory cgroup) and
 *     SCAN_EXCEED_NONE_PTE (too many absent PTEs in the candidate range
 *     for a synchronous collapse). Other transient collapse failures are
 *     reported as -EAGAIN; non-transient ones as -EINVAL. MADV_HWPOISON and
 *     MADV_SOFT_OFFLINE also return -EBUSY when the page cannot be handled.
 *
 * error: EOPNOTSUPP, Operation not supported
 *   desc: MADV_REMOVE propagates vfs_fallocate() errors verbatim, so a
 *     filesystem without FALLOC_FL_PUNCH_HOLE support fails with -EOPNOTSUPP
 *     (other propagated errors include -EPERM, -ETXTBSY and -ENOSPC).
 *     MADV_SOFT_OFFLINE also returns it when soft offlining is disabled via
 *     /proc/sys/vm/enable_soft_offline or the event is filtered by
 *     hwpoison_filter(); MADV_HWPOISON folds that case to 0.
 *
 * lock: mm->mmap_lock (read mode)
 *   type: semaphore
 *   acquired: yes
 *   released: yes
 *   desc: Taken for read for MADV_REMOVE, MADV_WILLNEED, MADV_COLD,
 *     MADV_PAGEOUT, MADV_COLLAPSE, MADV_POPULATE_READ and
 *     MADV_POPULATE_WRITE, and as the fallback when the per-VMA lock path
 *     declines. It is dropped and retaken around vfs_fadvise() (WILLNEED on
 *     regular files), vfs_fallocate() and userfaultfd_remove() (REMOVE, and
 *     DONTNEED or FREE with UFFD_FEATURE_EVENT_REMOVE), the anon and file
 *     collapse paths (COLLAPSE) and inside faultin_page_range()
 *     (POPULATE_*), so the VMA must be looked up again afterwards.
 *
 * lock: mm->mmap_lock (write mode; killable)
 *   type: semaphore
 *   acquired: yes
 *   released: yes
 *   desc: Acquired in killable write mode for behaviors that modify
 *     vma->vm_flags or split/merge VMAs (MADV_NORMAL, MADV_RANDOM,
 *     MADV_SEQUENTIAL, MADV_DONTFORK, MADV_DOFORK, MADV_DONTDUMP, MADV_DODUMP,
 *     MADV_WIPEONFORK, MADV_KEEPONFORK, MADV_MERGEABLE, MADV_UNMERGEABLE,
 *     MADV_HUGEPAGE, MADV_NOHUGEPAGE). If the acquisition is killed by a
 *     fatal signal, the syscall returns -EINTR before any VMA is touched.
 *
 * lock: per-VMA read lock (vma->vm_refcnt)
 *   type: custom
 *   acquired: yes
 *   released: yes
 *   desc: Tried first for MADV_DONTNEED, MADV_DONTNEED_LOCKED, MADV_FREE,
 *     MADV_GUARD_INSTALL and MADV_GUARD_REMOVE via lock_vma_under_rcu(). The
 *     lock is a reference on the VMA (vm_refcnt), not an rwsem. The per-VMA
 *     path is taken only when the requested range fits within a single VMA,
 *     the target mm is the caller's mm, the VMA is not armed with
 *     userfaultfd, and, for MADV_GUARD_INSTALL on an anonymous VMA, an
 *     anon_vma is already attached. Otherwise the code falls back to the
 *     mmap read lock above.
 *
 * lock: mmu_gather TLB batch
 *   type: custom
 *   acquired: yes
 *   released: yes
 *   desc: For MADV_DONTNEED, MADV_DONTNEED_LOCKED and MADV_FREE a single
 *     tlb_gather_mmu() / tlb_finish_mmu() pair (madvise_init_tlb() and
 *     madvise_finish_tlb()) wraps the whole syscall, batching TLB invalidation
 *     across all VMAs in the range. MADV_COLD and MADV_PAGEOUT build a
 *     short-lived gather inside the handler. MADV_GUARD_INSTALL builds a
 *     transient gather via zap_vma_range() each time the retry loop clears
 *     pre-existing pages, and none if the range is already empty.
 *     MADV_GUARD_REMOVE never gathers.
 *
 * lock: mmu_notifier invalidate range
 *   type: custom
 *   acquired: yes
 *   released: yes
 *   desc: All zap-based paths -- MADV_DONTNEED, MADV_DONTNEED_LOCKED, the
 *     zap branch of MADV_GUARD_INSTALL via zap_vma_range(), and
 *     MADV_FREE's own walk -- bracket their work with
 *     mmu_notifier_invalidate_range_start()/_end() so secondary MMUs (KVM,
 *     IOMMU SVA, etc.) observe the page clearing.
 *
 * signal: Any fatal signal
 *   direction: receive
 *   action: return
 *   condition: Acquiring the mmap write lock or faulting in pages for
 *     MADV_POPULATE_*
 *   desc: A pending fatal signal aborts mmap_write_lock_killable() (used by
 *     the VMA-flag-mutating behaviors) and faultin_page_range() (used by
 *     MADV_POPULATE_READ and MADV_POPULATE_WRITE), in both cases surfacing as
 *     -EINTR. Only a fatal signal interrupts these waits, so the task is
 *     being killed and user space does not normally see the error.
 *   errno: -EINTR
 *   timing: during
 *   restartable: no
 *
 * side-effect: modify_state
 *   target: vma->vm_flags
 *   condition: Hint-group behaviors (MADV_NORMAL, MADV_RANDOM, MADV_SEQUENTIAL,
 *     MADV_DONTFORK, MADV_DOFORK, MADV_DONTDUMP, MADV_DODUMP, MADV_WIPEONFORK,
 *     MADV_KEEPONFORK, MADV_MERGEABLE, MADV_UNMERGEABLE, MADV_HUGEPAGE,
 *     MADV_NOHUGEPAGE)
 *   desc: Sets or clears VM_RAND_READ, VM_SEQ_READ, VM_DONTCOPY,
 *     VM_DONTDUMP, VM_WIPEONFORK, VM_MERGEABLE, VM_HUGEPAGE or VM_NOHUGEPAGE
 *     on the affected VMAs, splitting or merging VMAs as needed. Reversible
 *     with the inverse advice (MADV_DOFORK undoes MADV_DONTFORK), except
 *     that the inverse call's VMA filter still applies (DOFORK rejects
 *     VM_SPECIAL, DODUMP rejects non-hugetlb VM_SPECIAL or VM_DROPPABLE,
 *     KEEPONFORK rejects VM_DROPPABLE).
 *   reversible: yes
 *
 * side-effect: free_memory | modify_state | irreversible
 *   target: page tables and resident pages within the range
 *   condition: MADV_DONTNEED, MADV_DONTNEED_LOCKED, MADV_FREE
 *   desc: MADV_DONTNEED zaps PTEs, releasing the pages or swap slots so the
 *     next access faults in zero-filled anonymous pages or re-reads the
 *     file. MADV_DONTNEED_LOCKED is identical but tolerates VM_LOCKED.
 *     MADV_FREE marks anonymous pages lazy-freeable; clean pages may be
 *     reclaimed under memory pressure and a write before reclaim cancels
 *     the lazy free. Discarded data cannot be recovered.
 *   reversible: no
 *
 * side-effect: filesystem | irreversible
 *   target: backing file (FALLOC_FL_PUNCH_HOLE)
 *   condition: MADV_REMOVE
 *   desc: Calls vfs_fallocate(FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE) on
 *     the backing file, deallocating the corresponding file blocks. The hole
 *     is visible to all mappers of the file and to read(2)/write(2)
 *     callers; subsequent reads return zero. Filesystem freeze protection,
 *     i_rwsem and any quota/space accounting are taken by the underlying
 *     fallocate path.
 *   reversible: no
 *
 * side-effect: modify_state | schedule
 *   target: LRU lists and page reclaim
 *   condition: MADV_COLD, MADV_PAGEOUT
 *   desc: MADV_COLD deactivates the affected pages, moving them to the
 *     inactive LRU and clearing PG_referenced/PG_young so they are reclaimed
 *     sooner under pressure. MADV_PAGEOUT additionally calls reclaim_pages()
 *     to write dirty pages out and drop clean ones synchronously. Page data
 *     is preserved (rereads will fault in the same content), but the I/O and
 *     LRU bookkeeping cannot be undone.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: page tables (faultin)
 *   condition: MADV_POPULATE_READ, MADV_POPULATE_WRITE
 *   desc: Walks the requested range with faultin_page_range(), populating
 *     PTEs by triggering read or write faults so subsequent accesses do not
 *     fault. Equivalent to touching every page in the range while suppressing
 *     SIGBUS/SIGSEGV through the syscall return value. Allocations made by
 *     faultin are not undone on partial failure.
 *   reversible: no
 *
 * side-effect: modify_state | schedule
 *   target: transparent hugepage layout
 *   condition: MADV_COLLAPSE
 *   desc: Synchronously coalesces base pages in the range into a PMD-sized
 *     transparent hugepage when the mapping permits. Performs the same page
 *     migration and zeroing that khugepaged would do asynchronously; the
 *     range's data is preserved across the collapse.
 *   reversible: no
 *
 * side-effect: free_memory | modify_state | irreversible
 *   target: PTE marker (PTE_MARKER_GUARD)
 *   condition: MADV_GUARD_INSTALL, MADV_GUARD_REMOVE
 *   desc: MADV_GUARD_INSTALL installs PTE_MARKER_GUARD entries that cause
 *     subsequent accesses to deliver SIGSEGV without consuming physical
 *     memory; existing pages already mapped in the range are zapped via
 *     zap_vma_range() before the markers are installed, so any
 *     prior contents are lost. MADV_GUARD_REMOVE clears the markers but
 *     does not (and cannot) restore zapped data.
 *   reversible: no
 *
 * side-effect: hardware | irreversible
 *   target: physical page (memory_failure)
 *   condition: MADV_HWPOISON
 *   desc: MADV_HWPOISON marks the affected pages as containing an
 *     unrecoverable hardware error using the same machine-check path that
 *     real ECC failures take. This affects physical memory bookkeeping
 *     kernel-wide, and madvise() has no inverse. The poison can be cleared
 *     through unpoison_memory() (the hwpoison-inject debugfs interface).
 *     Intended for testing the memory-failure pipeline; restricted to
 *     CAP_SYS_ADMIN.
 *   reversible: no
 *
 * side-effect: hardware
 *   target: physical page (soft_offline_page)
 *   condition: MADV_SOFT_OFFLINE
 *   desc: Migrates the contents off the affected pages and removes them from
 *     the buddy allocator. Page contents are preserved. madvise() has no
 *     inverse. The pages can be brought back through unpoison_memory() (the
 *     hwpoison-inject debugfs interface). Intended for testing the
 *     memory-failure pipeline; restricted to CAP_SYS_ADMIN.
 *   reversible: no
 *
 * side-effect: modify_state
 *   target: KSM merge state (vm_flags & VM_MERGEABLE)
 *   condition: MADV_MERGEABLE, MADV_UNMERGEABLE
 *   desc: Toggles VM_MERGEABLE, the VMA's eligibility for the kernel
 *     same-page merger. MADV_MERGEABLE lets ksmd later replace identical
 *     anonymous pages with shared, write-protected copies and is silently
 *     ignored on KSM-incompatible VMAs. MADV_UNMERGEABLE synchronously
 *     calls break_ksm(), which faults each KSM page in the range back to an
 *     exclusive copy, before clearing VM_MERGEABLE. An -ENOMEM from either
 *     advice is returned as -EAGAIN. The flag toggle is reversible by
 *     issuing the inverse advice.
 *   reversible: yes
 *
 * side-effect: modify_state
 *   target: userfaultfd event queue
 *   condition: MADV_DONTNEED, MADV_DONTNEED_LOCKED, MADV_FREE, MADV_REMOVE
 *     on a VMA whose userfaultfd context negotiated UFFD_FEATURE_EVENT_REMOVE
 *   desc: Generates a UFFD_EVENT_REMOVE notification covering the discarded
 *     range so userfaultfd monitors observing the mapping see the
 *     invalidation. The event is queued before the discard takes effect; the
 *     monitor cannot veto it.
 *   reversible: no
 *
 * capability: CAP_SYS_ADMIN
 *   type: perform_operation
 *   allows: Inject memory errors via MADV_HWPOISON or MADV_SOFT_OFFLINE
 *   without: Both behaviors return -EPERM
 *   condition: Checked at entry to madvise_inject_error() before any pages
 *     are looked up
 *
 * constraint: Page-aligned start
 *   desc: start must lie on a page boundary; otherwise the call returns
 *     -EINVAL before any VMA is consulted.
 *   expr: (start & (PAGE_SIZE - 1)) == 0
 *
 * constraint: Length rounded up to PAGE_SIZE
 *   desc: The effective range length is PAGE_ALIGN(len_in). A non-zero len_in
 *     that overflows during rounding, or a (start, end) range that wraps,
 *     is rejected with -EINVAL.
 *   expr: end = start + PAGE_ALIGN(len_in); end >= start
 *
 * constraint: Behavior must be supported
 *   desc: behavior must be one of the MADV_* values listed under the
 *     behavior parameter. Behaviors gated by Kconfig (KSM, THP, memory
 *     failure) are rejected with -EINVAL when the corresponding option is
 *     disabled in the running kernel.
 *
 * constraint: mseal-protected discards
 *   desc: On 64-bit kernels, a discard operation (FREE, DONTNEED,
 *     DONTNEED_LOCKED, REMOVE, DONTFORK, WIPEONFORK, GUARD_INSTALL) against
 *     a sealed anonymous VMA is rejected unless the mapping is currently
 *     writable -- both VM_WRITE in vm_flags and arch_vma_access_permitted()
 *     allowing write -- so that mseal(2) cannot be bypassed by instructing
 *     the kernel to throw the data away. File-backed sealed VMAs and
 *     writable sealed VMAs are not subject to this restriction.
 *   expr: !is_discard(behavior) || !vma_is_sealed(vma) ||
 *     !vma_is_anonymous(vma) || ((vma->vm_flags & VM_WRITE) &&
 *     arch_vma_access_permitted(vma, true, false, false))
 *
 * constraint: MADV_FREE requires anonymous mappings
 *   desc: MADV_FREE is defined only over anonymous mappings; the handler
 *     requires vma_is_anonymous() (no vm_ops) and rejects file-backed VMAs,
 *     including shared anonymous (shmem) VMAs, with -EINVAL.
 *   expr: vma_is_anonymous(vma)
 *
 * constraint: MADV_WIPEONFORK requires private anonymous mappings
 *   desc: MADV_WIPEONFORK rejects file-backed mappings and shared anonymous
 *     mappings; only MAP_PRIVATE anonymous VMAs accept it. Both rejections
 *     surface as -EINVAL.
 *   expr: !vma->vm_file && !(vma->vm_flags & VM_SHARED)
 *
 * constraint: MADV_REMOVE requires a shared file mapping that may be written
 *   desc: MADV_REMOVE rejects VM_LOCKED VMAs and VMAs without an associated
 *     file/mapping/host inode with -EINVAL. It rejects VMAs failing
 *     vma_is_shared_maywrite() (VM_SHARED and VM_MAYWRITE) with -EACCES,
 *     which covers private file mappings and shared mappings of files not
 *     opened for writing, but not a PROT_READ MAP_SHARED mapping of a file
 *     opened O_RDWR.
 *   expr: !(vma->vm_flags & VM_LOCKED) && vma->vm_file &&
 *     vma->vm_file->f_mapping && vma->vm_file->f_mapping->host &&
 *     vma_is_shared_maywrite(vma)
 *
 * constraint: MADV_COLD / MADV_PAGEOUT VMA filter
 *   desc: Both behaviors require LRU-managed pages; they reject VMAs that
 *     are mlocked, raw-PFN or hugetlb.
 *   expr: !(vma->vm_flags & (VM_LOCKED | VM_PFNMAP | VM_HUGETLB))
 *
 * examples: madvise(p, len, MADV_SEQUENTIAL);  // set VM_SEQ_READ on the VMA
 *   madvise(p, len, MADV_POPULATE_WRITE);  // prefault writable PTEs
 *   madvise(p, len, MADV_DONTNEED);        // discard anonymous pages
 *   madvise(p, len, MADV_GUARD_INSTALL);   // install SIGSEGV guard pages
 *
 * notes: Behavior introduction history (mainline): MADV_FREE in 4.5,
 *   MADV_WIPEONFORK / MADV_KEEPONFORK in 4.14, MADV_COLD / MADV_PAGEOUT in
 *   5.4, MADV_POPULATE_READ / MADV_POPULATE_WRITE in 5.14,
 *   MADV_DONTNEED_LOCKED in 5.18, MADV_COLLAPSE in 6.1, MADV_GUARD_INSTALL /
 *   MADV_GUARD_REMOVE in 6.13. Code that wants to remain portable to older
 *   kernels must handle -EINVAL gracefully and fall back.
 *
 *   process_madvise(2) applies the same advice values to another process
 *   identified by a pidfd. When the target mm is the caller's own (the
 *   pidfd refers to the caller), any locally-supported MADV_* value is
 *   accepted. When the target is a different mm, the behavior must be in
 *   the non-destructive remote subset (MADV_COLD, MADV_PAGEOUT,
 *   MADV_WILLNEED, MADV_COLLAPSE) or the call returns -EINVAL, and the
 *   caller must hold CAP_SYS_NICE.
 *
 *   MADV_PAGEOUT on a non-anonymous VM_MAYSHARE mapping is a silent no-op
 *   returning 0 unless can_do_file_pageout() holds, meaning the caller owns
 *   the file or is capable over it (file_owner_or_capable(), which honors
 *   the mount idmap) or may write it (file_permission(MAY_WRITE)). On a
 *   private file mapping that fails the same test, only anonymous pages are
 *   paged out. MADV_COLD has no such filter.
 *
 *   MADV_GUARD_INSTALL retries up to MAX_MADVISE_GUARD_RETRIES (3) times
 *   when it loses races with concurrent faulting or khugepaged. If those
 *   retries are exhausted the handler returns -ERESTARTNOINTR via
 *   restart_syscall(), which sets TIF_SIGPENDING so the return path runs
 *   signal handling. madvise() is then transparently re-executed with the
 *   same arguments, after any handler for a pending signal has run, unless
 *   a fatal signal terminates the task first. The caller never observes an
 *   errno from the restart itself and the call appears to make eventual
 *   forward progress.
 *
 *   anon_vma_prepare() failures inside MADV_GUARD_INSTALL bypass the
 *   ENOMEM-to-EAGAIN translation that applies to the VMA-flag-mutating
 *   behaviors and surface as -ENOMEM directly.
 *
 *   Architecture note: alpha defines MADV_DONTNEED as 6 (not 4) and reserves
 *   MADV_SPACEAVAIL=5; portable code must use the symbolic names from
 *   <sys/mman.h>.
 */
SYSCALL_DEFINE3(madvise, unsigned long, start, size_t, len_in, int, behavior)
{
	return do_madvise(current->mm, start, len_in, behavior);
}

/* Perform an madvise operation over a vector of addresses and lengths. */
static ssize_t vector_madvise(struct mm_struct *mm, struct iov_iter *iter,
			      int behavior)
{
	ssize_t ret = 0;
	size_t total_len;
	struct mmu_gather tlb;
	struct madvise_behavior madv_behavior = {
		.mm = mm,
		.behavior = behavior,
		.tlb = &tlb,
	};

	total_len = iov_iter_count(iter);

	ret = madvise_lock(&madv_behavior);
	if (ret)
		return ret;
	madvise_init_tlb(&madv_behavior);

	while (iov_iter_count(iter)) {
		unsigned long start = (unsigned long)iter_iov_addr(iter);
		size_t len_in = iter_iov_len(iter);
		int error;

		error = check_input_range(start, len_in);
		if (error || !len_in)
			ret = error;
		else
			ret = madvise_do_behavior(start, len_in, &madv_behavior);
		/*
		 * An madvise operation is attempting to restart the syscall,
		 * but we cannot proceed as it would not be correct to repeat
		 * the operation in aggregate, and would be surprising to the
		 * user.
		 *
		 * We drop and reacquire locks so it is safe to just loop and
		 * try again. We check for fatal signals in case we need exit
		 * early anyway.
		 */
		if (ret == -ERESTARTNOINTR) {
			if (fatal_signal_pending(current)) {
				ret = -EINTR;
				break;
			}

			/* Drop and reacquire lock to unwind race. */
			madvise_finish_tlb(&madv_behavior);
			madvise_unlock(&madv_behavior);
			ret = madvise_lock(&madv_behavior);
			if (ret)
				goto out;
			madvise_init_tlb(&madv_behavior);
			continue;
		}
		if (ret < 0)
			break;
		iov_iter_advance(iter, iter_iov_len(iter));
	}
	madvise_finish_tlb(&madv_behavior);
	madvise_unlock(&madv_behavior);

out:
	ret = (total_len - iov_iter_count(iter)) ? : ret;

	return ret;
}

SYSCALL_DEFINE5(process_madvise, int, pidfd, const struct iovec __user *, vec,
		size_t, vlen, int, behavior, unsigned int, flags)
{
	ssize_t ret;
	struct iovec iovstack[UIO_FASTIOV];
	struct iovec *iov = iovstack;
	struct iov_iter iter;
	struct task_struct *task;
	struct mm_struct *mm;
	unsigned int f_flags;

	if (flags != 0) {
		ret = -EINVAL;
		goto out;
	}

	ret = import_iovec(ITER_DEST, vec, vlen, ARRAY_SIZE(iovstack), &iov, &iter);
	if (ret < 0)
		goto out;

	task = pidfd_get_task(pidfd, &f_flags);
	if (IS_ERR(task)) {
		ret = PTR_ERR(task);
		goto free_iov;
	}

	/* Require PTRACE_MODE_READ to avoid leaking ASLR metadata. */
	mm = mm_access(task, PTRACE_MODE_READ_FSCREDS);
	if (IS_ERR(mm)) {
		ret = PTR_ERR(mm);
		goto release_task;
	}

	if (!madvise_behavior_valid(behavior)) {
		ret = -EINVAL;
		goto release_mm;
	}

	/*
	 * We need only perform this check if we are attempting to manipulate a
	 * remote process's address space.
	 */
	if (mm != current->mm && !process_madvise_remote_valid(behavior)) {
		ret = -EINVAL;
		goto release_mm;
	}

	/*
	 * Require CAP_SYS_NICE for influencing process performance. Note that
	 * only non-destructive hints are currently supported for remote
	 * processes.
	 */
	if (mm != current->mm && !capable(CAP_SYS_NICE)) {
		ret = -EPERM;
		goto release_mm;
	}

	ret = vector_madvise(mm, &iter, behavior);

release_mm:
	mmput(mm);
release_task:
	put_task_struct(task);
free_iov:
	kfree(iov);
out:
	return ret;
}

#ifdef CONFIG_ANON_VMA_NAME

#define ANON_VMA_NAME_MAX_LEN		80
#define ANON_VMA_NAME_INVALID_CHARS	"\\`$[]"

static inline bool is_valid_name_char(char ch)
{
	/* printable ascii characters, excluding ANON_VMA_NAME_INVALID_CHARS */
	return ch > 0x1f && ch < 0x7f &&
		!strchr(ANON_VMA_NAME_INVALID_CHARS, ch);
}

static int madvise_set_anon_name(struct mm_struct *mm, unsigned long start,
		unsigned long len_in, struct anon_vma_name *anon_name)
{
	unsigned long end;
	unsigned long len;
	int error;
	struct madvise_behavior madv_behavior = {
		.mm = mm,
		.behavior = __MADV_SET_ANON_VMA_NAME,
		.anon_name = anon_name,
	};

	if (start & ~PAGE_MASK)
		return -EINVAL;
	len = (len_in + ~PAGE_MASK) & PAGE_MASK;

	/* Check to see whether len was rounded up from small -ve to zero */
	if (len_in && !len)
		return -EINVAL;

	end = start + len;
	if (end < start)
		return -EINVAL;

	if (end == start)
		return 0;

	madv_behavior.range.start = start;
	madv_behavior.range.end = end;

	error = madvise_lock(&madv_behavior);
	if (error)
		return error;
	error = madvise_walk_vmas(&madv_behavior);
	madvise_unlock(&madv_behavior);

	return error;
}

int set_anon_vma_name(unsigned long addr, unsigned long size,
		      const char __user *uname)
{
	struct anon_vma_name *anon_name = NULL;
	struct mm_struct *mm = current->mm;
	int error;

	if (uname) {
		char *name, *pch;

		name = strndup_user(uname, ANON_VMA_NAME_MAX_LEN);
		if (IS_ERR(name))
			return PTR_ERR(name);

		for (pch = name; *pch != '\0'; pch++) {
			if (!is_valid_name_char(*pch)) {
				kfree(name);
				return -EINVAL;
			}
		}
		/* anon_vma has its own copy */
		anon_name = anon_vma_name_alloc(name);
		kfree(name);
		if (!anon_name)
			return -ENOMEM;
	}

	error = madvise_set_anon_name(mm, addr, size, anon_name);
	anon_vma_name_put(anon_name);

	return error;
}
#endif
