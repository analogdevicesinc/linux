// SPDX-License-Identifier: GPL-2.0
/*
 * DAMON Primitives for The Physical Address Space
 *
 * Author: SeongJae Park <sj@kernel.org>
 */

#define pr_fmt(fmt) "damon-pa: " fmt

#include <linux/mmu_notifier.h>
#include <linux/page_idle.h>
#include <linux/pagemap.h>
#include <linux/rmap.h>
#include <linux/swap.h>

#include "../internal.h"
#include "ops-common.h"

static bool damon_folio_mkold_one(struct folio *folio,
		struct vm_area_struct *vma, unsigned long addr, void *arg)
{
	DEFINE_FOLIO_VMA_WALK(pvmw, folio, vma, addr, 0);

	while (page_vma_mapped_walk(&pvmw)) {
		addr = pvmw.address;
		if (pvmw.pte)
			damon_ptep_mkold(pvmw.pte, vma, addr);
		else
			damon_pmdp_mkold(pvmw.pmd, vma, addr);
	}
	return true;
}

static void damon_folio_mkold(struct folio *folio)
{
	struct rmap_walk_control rwc = {
		.rmap_one = damon_folio_mkold_one,
		.anon_lock = folio_lock_anon_vma_read,
	};
	bool need_lock;

	if (!folio_mapped(folio) || !folio_raw_mapping(folio)) {
		folio_set_idle(folio);
		return;
	}

	need_lock = !folio_test_anon(folio) || folio_test_ksm(folio);
	if (need_lock && !folio_trylock(folio))
		return;

	rmap_walk(folio, &rwc);

	if (need_lock)
		folio_unlock(folio);

}

static void damon_pa_mkold(unsigned long paddr)
{
	struct folio *folio = damon_get_folio(PHYS_PFN(paddr));

	if (!folio)
		return;

	damon_folio_mkold(folio);
	folio_put(folio);
}

static void __damon_pa_prepare_access_check(struct damon_region *r)
{
	r->sampling_addr = damon_rand(r->ar.start, r->ar.end);

	damon_pa_mkold(r->sampling_addr);
}

static void damon_pa_prepare_access_checks(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct damon_region *r;

	damon_for_each_target(t, ctx) {
		damon_for_each_region(r, t)
			__damon_pa_prepare_access_check(r);
	}
}

static bool damon_folio_young_one(struct folio *folio,
		struct vm_area_struct *vma, unsigned long addr, void *arg)
{
	bool *accessed = arg;
	DEFINE_FOLIO_VMA_WALK(pvmw, folio, vma, addr, 0);

	*accessed = false;
	while (page_vma_mapped_walk(&pvmw)) {
		addr = pvmw.address;
		if (pvmw.pte) {
			*accessed = pte_young(ptep_get(pvmw.pte)) ||
				!folio_test_idle(folio) ||
				mmu_notifier_test_young(vma->vm_mm, addr);
		} else {
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
			*accessed = pmd_young(pmdp_get(pvmw.pmd)) ||
				!folio_test_idle(folio) ||
				mmu_notifier_test_young(vma->vm_mm, addr);
#else
			WARN_ON_ONCE(1);
#endif	/* CONFIG_TRANSPARENT_HUGEPAGE */
		}
		if (*accessed) {
			page_vma_mapped_walk_done(&pvmw);
			break;
		}
	}

	/* If accessed, stop walking */
	return *accessed == false;
}

static bool damon_folio_young(struct folio *folio)
{
	bool accessed = false;
	struct rmap_walk_control rwc = {
		.arg = &accessed,
		.rmap_one = damon_folio_young_one,
		.anon_lock = folio_lock_anon_vma_read,
	};
	bool need_lock;

	if (!folio_mapped(folio) || !folio_raw_mapping(folio)) {
		if (folio_test_idle(folio))
			return false;
		else
			return true;
	}

	need_lock = !folio_test_anon(folio) || folio_test_ksm(folio);
	if (need_lock && !folio_trylock(folio))
		return false;

	rmap_walk(folio, &rwc);

	if (need_lock)
		folio_unlock(folio);

	return accessed;
}

static bool damon_pa_young(unsigned long paddr, unsigned long *folio_sz)
{
	struct folio *folio = damon_get_folio(PHYS_PFN(paddr));
	bool accessed;

	if (!folio)
		return false;

	accessed = damon_folio_young(folio);
	*folio_sz = folio_size(folio);
	folio_put(folio);
	return accessed;
}

static void __damon_pa_check_access(struct damon_region *r,
		struct damon_attrs *attrs)
{
	static unsigned long last_addr;
	static unsigned long last_folio_sz = PAGE_SIZE;
	static bool last_accessed;

	/* If the region is in the last checked page, reuse the result */
	if (ALIGN_DOWN(last_addr, last_folio_sz) ==
				ALIGN_DOWN(r->sampling_addr, last_folio_sz)) {
		damon_update_region_access_rate(r, last_accessed, attrs);
		return;
	}

	last_accessed = damon_pa_young(r->sampling_addr, &last_folio_sz);
	damon_update_region_access_rate(r, last_accessed, attrs);

	last_addr = r->sampling_addr;
}

static unsigned int damon_pa_check_accesses(struct damon_ctx *ctx)
{
	struct damon_target *t;
	struct damon_region *r;
	unsigned int max_nr_accesses = 0;

	damon_for_each_target(t, ctx) {
		damon_for_each_region(r, t) {
			__damon_pa_check_access(r, &ctx->attrs);
			max_nr_accesses = max(r->nr_accesses, max_nr_accesses);
		}
	}

	return max_nr_accesses;
}

static bool __damos_pa_filter_out(struct damos_filter *filter,
		struct folio *folio)
{
	bool matched = false;
	struct mem_cgroup *memcg;

	switch (filter->type) {
	case DAMOS_FILTER_TYPE_ANON:
		matched = folio_test_anon(folio);
		break;
	case DAMOS_FILTER_TYPE_MEMCG:
		rcu_read_lock();
		memcg = folio_memcg_check(folio);
		if (!memcg)
			matched = false;
		else
			matched = filter->memcg_id == mem_cgroup_id(memcg);
		rcu_read_unlock();
		break;
	case DAMOS_FILTER_TYPE_YOUNG:
		matched = damon_folio_young(folio);
		if (matched)
			damon_folio_mkold(folio);
		break;
	default:
		break;
	}

	return matched == filter->matching;
}

/*
 * damos_pa_filter_out - Return true if the page should be filtered out.
 */
static bool damos_pa_filter_out(struct damos *scheme, struct folio *folio)
{
	struct damos_filter *filter;

	damos_for_each_filter(filter, scheme) {
		if (__damos_pa_filter_out(filter, folio))
			return true;
	}
	return false;
}

static unsigned long damon_pa_pageout(struct damon_region *r, struct damos *s)
{
	unsigned long addr, applied;
	LIST_HEAD(folio_list);
	bool install_young_filter = true;
	struct damos_filter *filter;

	/* check access in page level again by default */
	damos_for_each_filter(filter, s) {
		if (filter->type == DAMOS_FILTER_TYPE_YOUNG) {
			install_young_filter = false;
			break;
		}
	}
	if (install_young_filter) {
		filter = damos_new_filter(DAMOS_FILTER_TYPE_YOUNG, true);
		if (!filter)
			return 0;
		damos_add_filter(s, filter);
	}

	for (addr = r->ar.start; addr < r->ar.end; addr += PAGE_SIZE) {
		struct folio *folio = damon_get_folio(PHYS_PFN(addr));

		if (!folio)
			continue;

		if (damos_pa_filter_out(s, folio))
			goto put_folio;

		folio_clear_referenced(folio);
		folio_test_clear_young(folio);
		if (!folio_isolate_lru(folio))
			goto put_folio;
		if (folio_test_unevictable(folio))
			folio_putback_lru(folio);
		else
			list_add(&folio->lru, &folio_list);
put_folio:
		folio_put(folio);
	}
	if (install_young_filter)
		damos_destroy_filter(filter);
	applied = reclaim_pages(&folio_list);
	cond_resched();
	return applied * PAGE_SIZE;
}

static inline unsigned long damon_pa_mark_accessed_or_deactivate(
		struct damon_region *r, struct damos *s, bool mark_accessed)
{
	unsigned long addr, applied = 0;

	for (addr = r->ar.start; addr < r->ar.end; addr += PAGE_SIZE) {
		struct folio *folio = damon_get_folio(PHYS_PFN(addr));

		if (!folio)
			continue;

		if (damos_pa_filter_out(s, folio))
			goto put_folio;

		if (mark_accessed)
			folio_mark_accessed(folio);
		else
			folio_deactivate(folio);
		applied += folio_nr_pages(folio);
put_folio:
		folio_put(folio);
	}
	return applied * PAGE_SIZE;
}

static unsigned long damon_pa_mark_accessed(struct damon_region *r,
	struct damos *s)
{
	return damon_pa_mark_accessed_or_deactivate(r, s, true);
}

static unsigned long damon_pa_deactivate_pages(struct damon_region *r,
	struct damos *s)
{
	return damon_pa_mark_accessed_or_deactivate(r, s, false);
}

static unsigned long damon_pa_apply_scheme(struct damon_ctx *ctx,
		struct damon_target *t, struct damon_region *r,
		struct damos *scheme)
{
	switch (scheme->action) {
	case DAMOS_PAGEOUT:
		return damon_pa_pageout(r, scheme);
	case DAMOS_LRU_PRIO:
		return damon_pa_mark_accessed(r, scheme);
	case DAMOS_LRU_DEPRIO:
		return damon_pa_deactivate_pages(r, scheme);
	case DAMOS_STAT:
		break;
	default:
		/* DAMOS actions that not yet supported by 'paddr'. */
		break;
	}
	return 0;
}

static int damon_pa_scheme_score(struct damon_ctx *context,
		struct damon_target *t, struct damon_region *r,
		struct damos *scheme)
{
	switch (scheme->action) {
	case DAMOS_PAGEOUT:
		return damon_cold_score(context, r, scheme);
	case DAMOS_LRU_PRIO:
		return damon_hot_score(context, r, scheme);
	case DAMOS_LRU_DEPRIO:
		return damon_cold_score(context, r, scheme);
	default:
		break;
	}

	return DAMOS_MAX_SCORE;
}

static int __init damon_pa_initcall(void)
{
	struct damon_operations ops = {
		.id = DAMON_OPS_PADDR,
		.init = NULL,
		.update = NULL,
		.prepare_access_checks = damon_pa_prepare_access_checks,
		.check_accesses = damon_pa_check_accesses,
		.reset_aggregated = NULL,
		.target_valid = NULL,
		.cleanup = NULL,
		.apply_scheme = damon_pa_apply_scheme,
		.get_scheme_score = damon_pa_scheme_score,
	};

	return damon_register_ops(&ops);
};

subsys_initcall(damon_pa_initcall);
