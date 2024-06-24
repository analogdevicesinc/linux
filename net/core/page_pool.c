/* SPDX-License-Identifier: GPL-2.0
 *
 * page_pool.c
 *	Author:	Jesper Dangaard Brouer <netoptimizer@brouer.com>
 *	Copyright (C) 2016 Red Hat, Inc.
 */

#include <linux/error-injection.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/device.h>

#include <net/page_pool/helpers.h>
#include <net/xdp.h>

#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/page-flags.h>
#include <linux/mm.h> /* for put_page() */
#include <linux/poison.h>
#include <linux/ethtool.h>
#include <linux/netdevice.h>

#include <trace/events/page_pool.h>

#include "page_pool_priv.h"

#define DEFER_TIME (msecs_to_jiffies(1000))
#define DEFER_WARN_INTERVAL (60 * HZ)

#define BIAS_MAX	(LONG_MAX >> 1)

#ifdef CONFIG_PAGE_POOL_STATS
static DEFINE_PER_CPU(struct page_pool_recycle_stats, pp_system_recycle_stats);

/* alloc_stat_inc is intended to be used in softirq context */
#define alloc_stat_inc(pool, __stat)	(pool->alloc_stats.__stat++)
/* recycle_stat_inc is safe to use when preemption is possible. */
#define recycle_stat_inc(pool, __stat)							\
	do {										\
		struct page_pool_recycle_stats __percpu *s = pool->recycle_stats;	\
		this_cpu_inc(s->__stat);						\
	} while (0)

#define recycle_stat_add(pool, __stat, val)						\
	do {										\
		struct page_pool_recycle_stats __percpu *s = pool->recycle_stats;	\
		this_cpu_add(s->__stat, val);						\
	} while (0)

static const char pp_stats[][ETH_GSTRING_LEN] = {
	"rx_pp_alloc_fast",
	"rx_pp_alloc_slow",
	"rx_pp_alloc_slow_ho",
	"rx_pp_alloc_empty",
	"rx_pp_alloc_refill",
	"rx_pp_alloc_waive",
	"rx_pp_recycle_cached",
	"rx_pp_recycle_cache_full",
	"rx_pp_recycle_ring",
	"rx_pp_recycle_ring_full",
	"rx_pp_recycle_released_ref",
};

/**
 * page_pool_get_stats() - fetch page pool stats
 * @pool:	pool from which page was allocated
 * @stats:	struct page_pool_stats to fill in
 *
 * Retrieve statistics about the page_pool. This API is only available
 * if the kernel has been configured with ``CONFIG_PAGE_POOL_STATS=y``.
 * A pointer to a caller allocated struct page_pool_stats structure
 * is passed to this API which is filled in. The caller can then report
 * those stats to the user (perhaps via ethtool, debugfs, etc.).
 */
bool page_pool_get_stats(const struct page_pool *pool,
			 struct page_pool_stats *stats)
{
	int cpu = 0;

	if (!stats)
		return false;

	/* The caller is responsible to initialize stats. */
	stats->alloc_stats.fast += pool->alloc_stats.fast;
	stats->alloc_stats.slow += pool->alloc_stats.slow;
	stats->alloc_stats.slow_high_order += pool->alloc_stats.slow_high_order;
	stats->alloc_stats.empty += pool->alloc_stats.empty;
	stats->alloc_stats.refill += pool->alloc_stats.refill;
	stats->alloc_stats.waive += pool->alloc_stats.waive;

	for_each_possible_cpu(cpu) {
		const struct page_pool_recycle_stats *pcpu =
			per_cpu_ptr(pool->recycle_stats, cpu);

		stats->recycle_stats.cached += pcpu->cached;
		stats->recycle_stats.cache_full += pcpu->cache_full;
		stats->recycle_stats.ring += pcpu->ring;
		stats->recycle_stats.ring_full += pcpu->ring_full;
		stats->recycle_stats.released_refcnt += pcpu->released_refcnt;
	}

	return true;
}
EXPORT_SYMBOL(page_pool_get_stats);

u8 *page_pool_ethtool_stats_get_strings(u8 *data)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(pp_stats); i++) {
		memcpy(data, pp_stats[i], ETH_GSTRING_LEN);
		data += ETH_GSTRING_LEN;
	}

	return data;
}
EXPORT_SYMBOL(page_pool_ethtool_stats_get_strings);

int page_pool_ethtool_stats_get_count(void)
{
	return ARRAY_SIZE(pp_stats);
}
EXPORT_SYMBOL(page_pool_ethtool_stats_get_count);

u64 *page_pool_ethtool_stats_get(u64 *data, const void *stats)
{
	const struct page_pool_stats *pool_stats = stats;

	*data++ = pool_stats->alloc_stats.fast;
	*data++ = pool_stats->alloc_stats.slow;
	*data++ = pool_stats->alloc_stats.slow_high_order;
	*data++ = pool_stats->alloc_stats.empty;
	*data++ = pool_stats->alloc_stats.refill;
	*data++ = pool_stats->alloc_stats.waive;
	*data++ = pool_stats->recycle_stats.cached;
	*data++ = pool_stats->recycle_stats.cache_full;
	*data++ = pool_stats->recycle_stats.ring;
	*data++ = pool_stats->recycle_stats.ring_full;
	*data++ = pool_stats->recycle_stats.released_refcnt;

	return data;
}
EXPORT_SYMBOL(page_pool_ethtool_stats_get);

#else
#define alloc_stat_inc(pool, __stat)
#define recycle_stat_inc(pool, __stat)
#define recycle_stat_add(pool, __stat, val)
#endif

static bool page_pool_producer_lock(struct page_pool *pool)
	__acquires(&pool->ring.producer_lock)
{
	bool in_softirq = in_softirq();

	if (in_softirq)
		spin_lock(&pool->ring.producer_lock);
	else
		spin_lock_bh(&pool->ring.producer_lock);

	return in_softirq;
}

static void page_pool_producer_unlock(struct page_pool *pool,
				      bool in_softirq)
	__releases(&pool->ring.producer_lock)
{
	if (in_softirq)
		spin_unlock(&pool->ring.producer_lock);
	else
		spin_unlock_bh(&pool->ring.producer_lock);
}

static void page_pool_struct_check(void)
{
	CACHELINE_ASSERT_GROUP_MEMBER(struct page_pool, frag, frag_users);
	CACHELINE_ASSERT_GROUP_MEMBER(struct page_pool, frag, frag_page);
	CACHELINE_ASSERT_GROUP_MEMBER(struct page_pool, frag, frag_offset);
	CACHELINE_ASSERT_GROUP_SIZE(struct page_pool, frag, 4 * sizeof(long));
}

static int page_pool_init(struct page_pool *pool,
			  const struct page_pool_params *params,
			  int cpuid)
{
	unsigned int ring_qsize = 1024; /* Default */

	page_pool_struct_check();

	memcpy(&pool->p, &params->fast, sizeof(pool->p));
	memcpy(&pool->slow, &params->slow, sizeof(pool->slow));

	pool->cpuid = cpuid;

	/* Validate only known flags were used */
	if (pool->slow.flags & ~PP_FLAG_ALL)
		return -EINVAL;

	if (pool->p.pool_size)
		ring_qsize = pool->p.pool_size;

	/* Sanity limit mem that can be pinned down */
	if (ring_qsize > 32768)
		return -E2BIG;

	/* DMA direction is either DMA_FROM_DEVICE or DMA_BIDIRECTIONAL.
	 * DMA_BIDIRECTIONAL is for allowing page used for DMA sending,
	 * which is the XDP_TX use-case.
	 */
	if (pool->slow.flags & PP_FLAG_DMA_MAP) {
		if ((pool->p.dma_dir != DMA_FROM_DEVICE) &&
		    (pool->p.dma_dir != DMA_BIDIRECTIONAL))
			return -EINVAL;

		pool->dma_map = true;
	}

	if (pool->slow.flags & PP_FLAG_DMA_SYNC_DEV) {
		/* In order to request DMA-sync-for-device the page
		 * needs to be mapped
		 */
		if (!(pool->slow.flags & PP_FLAG_DMA_MAP))
			return -EINVAL;

		if (!pool->p.max_len)
			return -EINVAL;

		pool->dma_sync = true;

		/* pool->p.offset has to be set according to the address
		 * offset used by the DMA engine to start copying rx data
		 */
	}

	pool->has_init_callback = !!pool->slow.init_callback;

#ifdef CONFIG_PAGE_POOL_STATS
	if (!(pool->slow.flags & PP_FLAG_SYSTEM_POOL)) {
		pool->recycle_stats = alloc_percpu(struct page_pool_recycle_stats);
		if (!pool->recycle_stats)
			return -ENOMEM;
	} else {
		/* For system page pool instance we use a singular stats object
		 * instead of allocating a separate percpu variable for each
		 * (also percpu) page pool instance.
		 */
		pool->recycle_stats = &pp_system_recycle_stats;
		pool->system = true;
	}
#endif

	if (ptr_ring_init(&pool->ring, ring_qsize, GFP_KERNEL) < 0) {
#ifdef CONFIG_PAGE_POOL_STATS
		if (!pool->system)
			free_percpu(pool->recycle_stats);
#endif
		return -ENOMEM;
	}

	atomic_set(&pool->pages_state_release_cnt, 0);

	/* Driver calling page_pool_create() also call page_pool_destroy() */
	refcount_set(&pool->user_cnt, 1);

	if (pool->dma_map)
		get_device(pool->p.dev);

	return 0;
}

static void page_pool_uninit(struct page_pool *pool)
{
	ptr_ring_cleanup(&pool->ring, NULL);

	if (pool->dma_map)
		put_device(pool->p.dev);

#ifdef CONFIG_PAGE_POOL_STATS
	if (!pool->system)
		free_percpu(pool->recycle_stats);
#endif
}

/**
 * page_pool_create_percpu() - create a page pool for a given cpu.
 * @params: parameters, see struct page_pool_params
 * @cpuid: cpu identifier
 */
struct page_pool *
page_pool_create_percpu(const struct page_pool_params *params, int cpuid)
{
	struct page_pool *pool;
	int err;

	pool = kzalloc_node(sizeof(*pool), GFP_KERNEL, params->nid);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	err = page_pool_init(pool, params, cpuid);
	if (err < 0)
		goto err_free;

	err = page_pool_list(pool);
	if (err)
		goto err_uninit;

	return pool;

err_uninit:
	page_pool_uninit(pool);
err_free:
	pr_warn("%s() gave up with errno %d\n", __func__, err);
	kfree(pool);
	return ERR_PTR(err);
}
EXPORT_SYMBOL(page_pool_create_percpu);

/**
 * page_pool_create() - create a page pool
 * @params: parameters, see struct page_pool_params
 */
struct page_pool *page_pool_create(const struct page_pool_params *params)
{
	return page_pool_create_percpu(params, -1);
}
EXPORT_SYMBOL(page_pool_create);

static void page_pool_return_page(struct page_pool *pool, struct page *page);

noinline
static struct page *page_pool_refill_alloc_cache(struct page_pool *pool)
{
	struct ptr_ring *r = &pool->ring;
	struct page *page;
	int pref_nid; /* preferred NUMA node */

	/* Quicker fallback, avoid locks when ring is empty */
	if (__ptr_ring_empty(r)) {
		alloc_stat_inc(pool, empty);
		return NULL;
	}

	/* Softirq guarantee CPU and thus NUMA node is stable. This,
	 * assumes CPU refilling driver RX-ring will also run RX-NAPI.
	 */
#ifdef CONFIG_NUMA
	pref_nid = (pool->p.nid == NUMA_NO_NODE) ? numa_mem_id() : pool->p.nid;
#else
	/* Ignore pool->p.nid setting if !CONFIG_NUMA, helps compiler */
	pref_nid = numa_mem_id(); /* will be zero like page_to_nid() */
#endif

	/* Refill alloc array, but only if NUMA match */
	do {
		page = __ptr_ring_consume(r);
		if (unlikely(!page))
			break;

		if (likely(page_to_nid(page) == pref_nid)) {
			pool->alloc.cache[pool->alloc.count++] = page;
		} else {
			/* NUMA mismatch;
			 * (1) release 1 page to page-allocator and
			 * (2) break out to fallthrough to alloc_pages_node.
			 * This limit stress on page buddy alloactor.
			 */
			page_pool_return_page(pool, page);
			alloc_stat_inc(pool, waive);
			page = NULL;
			break;
		}
	} while (pool->alloc.count < PP_ALLOC_CACHE_REFILL);

	/* Return last page */
	if (likely(pool->alloc.count > 0)) {
		page = pool->alloc.cache[--pool->alloc.count];
		alloc_stat_inc(pool, refill);
	}

	return page;
}

/* fast path */
static struct page *__page_pool_get_cached(struct page_pool *pool)
{
	struct page *page;

	/* Caller MUST guarantee safe non-concurrent access, e.g. softirq */
	if (likely(pool->alloc.count)) {
		/* Fast-path */
		page = pool->alloc.cache[--pool->alloc.count];
		alloc_stat_inc(pool, fast);
	} else {
		page = page_pool_refill_alloc_cache(pool);
	}

	return page;
}

static void __page_pool_dma_sync_for_device(const struct page_pool *pool,
					    const struct page *page,
					    u32 dma_sync_size)
{
#if defined(CONFIG_HAS_DMA) && defined(CONFIG_DMA_NEED_SYNC)
	dma_addr_t dma_addr = page_pool_get_dma_addr(page);

	dma_sync_size = min(dma_sync_size, pool->p.max_len);
	__dma_sync_single_for_device(pool->p.dev, dma_addr + pool->p.offset,
				     dma_sync_size, pool->p.dma_dir);
#endif
}

static __always_inline void
page_pool_dma_sync_for_device(const struct page_pool *pool,
			      const struct page *page,
			      u32 dma_sync_size)
{
	if (pool->dma_sync && dma_dev_need_sync(pool->p.dev))
		__page_pool_dma_sync_for_device(pool, page, dma_sync_size);
}

static bool page_pool_dma_map(struct page_pool *pool, struct page *page)
{
	dma_addr_t dma;

	/* Setup DMA mapping: use 'struct page' area for storing DMA-addr
	 * since dma_addr_t can be either 32 or 64 bits and does not always fit
	 * into page private data (i.e 32bit cpu with 64bit DMA caps)
	 * This mapping is kept for lifetime of page, until leaving pool.
	 */
	dma = dma_map_page_attrs(pool->p.dev, page, 0,
				 (PAGE_SIZE << pool->p.order),
				 pool->p.dma_dir, DMA_ATTR_SKIP_CPU_SYNC |
						  DMA_ATTR_WEAK_ORDERING);
	if (dma_mapping_error(pool->p.dev, dma))
		return false;

	if (page_pool_set_dma_addr(page, dma))
		goto unmap_failed;

	page_pool_dma_sync_for_device(pool, page, pool->p.max_len);

	return true;

unmap_failed:
	WARN_ON_ONCE("unexpected DMA address, please report to netdev@");
	dma_unmap_page_attrs(pool->p.dev, dma,
			     PAGE_SIZE << pool->p.order, pool->p.dma_dir,
			     DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING);
	return false;
}

static void page_pool_set_pp_info(struct page_pool *pool,
				  struct page *page)
{
	page->pp = pool;
	page->pp_magic |= PP_SIGNATURE;

	/* Ensuring all pages have been split into one fragment initially:
	 * page_pool_set_pp_info() is only called once for every page when it
	 * is allocated from the page allocator and page_pool_fragment_page()
	 * is dirtying the same cache line as the page->pp_magic above, so
	 * the overhead is negligible.
	 */
	page_pool_fragment_page(page, 1);
	if (pool->has_init_callback)
		pool->slow.init_callback(page, pool->slow.init_arg);
}

static void page_pool_clear_pp_info(struct page *page)
{
	page->pp_magic = 0;
	page->pp = NULL;
}

static struct page *__page_pool_alloc_page_order(struct page_pool *pool,
						 gfp_t gfp)
{
	struct page *page;

	gfp |= __GFP_COMP;
	page = alloc_pages_node(pool->p.nid, gfp, pool->p.order);
	if (unlikely(!page))
		return NULL;

	if (pool->dma_map && unlikely(!page_pool_dma_map(pool, page))) {
		put_page(page);
		return NULL;
	}

	alloc_stat_inc(pool, slow_high_order);
	page_pool_set_pp_info(pool, page);

	/* Track how many pages are held 'in-flight' */
	pool->pages_state_hold_cnt++;
	trace_page_pool_state_hold(pool, page, pool->pages_state_hold_cnt);
	return page;
}

/* slow path */
noinline
static struct page *__page_pool_alloc_pages_slow(struct page_pool *pool,
						 gfp_t gfp)
{
	const int bulk = PP_ALLOC_CACHE_REFILL;
	unsigned int pp_order = pool->p.order;
	bool dma_map = pool->dma_map;
	struct page *page;
	int i, nr_pages;

	/* Don't support bulk alloc for high-order pages */
	if (unlikely(pp_order))
		return __page_pool_alloc_page_order(pool, gfp);

	/* Unnecessary as alloc cache is empty, but guarantees zero count */
	if (unlikely(pool->alloc.count > 0))
		return pool->alloc.cache[--pool->alloc.count];

	/* Mark empty alloc.cache slots "empty" for alloc_pages_bulk_array */
	memset(&pool->alloc.cache, 0, sizeof(void *) * bulk);

	nr_pages = alloc_pages_bulk_array_node(gfp, pool->p.nid, bulk,
					       pool->alloc.cache);
	if (unlikely(!nr_pages))
		return NULL;

	/* Pages have been filled into alloc.cache array, but count is zero and
	 * page element have not been (possibly) DMA mapped.
	 */
	for (i = 0; i < nr_pages; i++) {
		page = pool->alloc.cache[i];
		if (dma_map && unlikely(!page_pool_dma_map(pool, page))) {
			put_page(page);
			continue;
		}

		page_pool_set_pp_info(pool, page);
		pool->alloc.cache[pool->alloc.count++] = page;
		/* Track how many pages are held 'in-flight' */
		pool->pages_state_hold_cnt++;
		trace_page_pool_state_hold(pool, page,
					   pool->pages_state_hold_cnt);
	}

	/* Return last page */
	if (likely(pool->alloc.count > 0)) {
		page = pool->alloc.cache[--pool->alloc.count];
		alloc_stat_inc(pool, slow);
	} else {
		page = NULL;
	}

	/* When page just alloc'ed is should/must have refcnt 1. */
	return page;
}

/* For using page_pool replace: alloc_pages() API calls, but provide
 * synchronization guarantee for allocation side.
 */
struct page *page_pool_alloc_pages(struct page_pool *pool, gfp_t gfp)
{
	struct page *page;

	/* Fast-path: Get a page from cache */
	page = __page_pool_get_cached(pool);
	if (page)
		return page;

	/* Slow-path: cache empty, do real allocation */
	page = __page_pool_alloc_pages_slow(pool, gfp);
	return page;
}
EXPORT_SYMBOL(page_pool_alloc_pages);
ALLOW_ERROR_INJECTION(page_pool_alloc_pages, NULL);

/* Calculate distance between two u32 values, valid if distance is below 2^(31)
 *  https://en.wikipedia.org/wiki/Serial_number_arithmetic#General_Solution
 */
#define _distance(a, b)	(s32)((a) - (b))

s32 page_pool_inflight(const struct page_pool *pool, bool strict)
{
	u32 release_cnt = atomic_read(&pool->pages_state_release_cnt);
	u32 hold_cnt = READ_ONCE(pool->pages_state_hold_cnt);
	s32 inflight;

	inflight = _distance(hold_cnt, release_cnt);

	if (strict) {
		trace_page_pool_release(pool, inflight, hold_cnt, release_cnt);
		WARN(inflight < 0, "Negative(%d) inflight packet-pages",
		     inflight);
	} else {
		inflight = max(0, inflight);
	}

	return inflight;
}

static __always_inline
void __page_pool_release_page_dma(struct page_pool *pool, struct page *page)
{
	dma_addr_t dma;

	if (!pool->dma_map)
		/* Always account for inflight pages, even if we didn't
		 * map them
		 */
		return;

	dma = page_pool_get_dma_addr(page);

	/* When page is unmapped, it cannot be returned to our pool */
	dma_unmap_page_attrs(pool->p.dev, dma,
			     PAGE_SIZE << pool->p.order, pool->p.dma_dir,
			     DMA_ATTR_SKIP_CPU_SYNC | DMA_ATTR_WEAK_ORDERING);
	page_pool_set_dma_addr(page, 0);
}

/* Disconnects a page (from a page_pool).  API users can have a need
 * to disconnect a page (from a page_pool), to allow it to be used as
 * a regular page (that will eventually be returned to the normal
 * page-allocator via put_page).
 */
void page_pool_return_page(struct page_pool *pool, struct page *page)
{
	int count;

	__page_pool_release_page_dma(pool, page);

	page_pool_clear_pp_info(page);

	/* This may be the last page returned, releasing the pool, so
	 * it is not safe to reference pool afterwards.
	 */
	count = atomic_inc_return_relaxed(&pool->pages_state_release_cnt);
	trace_page_pool_state_release(pool, page, count);

	put_page(page);
	/* An optimization would be to call __free_pages(page, pool->p.order)
	 * knowing page is not part of page-cache (thus avoiding a
	 * __page_cache_release() call).
	 */
}

static bool page_pool_recycle_in_ring(struct page_pool *pool, struct page *page)
{
	int ret;
	/* BH protection not needed if current is softirq */
	if (in_softirq())
		ret = ptr_ring_produce(&pool->ring, page);
	else
		ret = ptr_ring_produce_bh(&pool->ring, page);

	if (!ret) {
		recycle_stat_inc(pool, ring);
		return true;
	}

	return false;
}

/* Only allow direct recycling in special circumstances, into the
 * alloc side cache.  E.g. during RX-NAPI processing for XDP_DROP use-case.
 *
 * Caller must provide appropriate safe context.
 */
static bool page_pool_recycle_in_cache(struct page *page,
				       struct page_pool *pool)
{
	if (unlikely(pool->alloc.count == PP_ALLOC_CACHE_SIZE)) {
		recycle_stat_inc(pool, cache_full);
		return false;
	}

	/* Caller MUST have verified/know (page_ref_count(page) == 1) */
	pool->alloc.cache[pool->alloc.count++] = page;
	recycle_stat_inc(pool, cached);
	return true;
}

static bool __page_pool_page_can_be_recycled(const struct page *page)
{
	return page_ref_count(page) == 1 && !page_is_pfmemalloc(page);
}

/* If the page refcnt == 1, this will try to recycle the page.
 * If pool->dma_sync is set, we'll try to sync the DMA area for
 * the configured size min(dma_sync_size, pool->max_len).
 * If the page refcnt != 1, then the page will be returned to memory
 * subsystem.
 */
static __always_inline struct page *
__page_pool_put_page(struct page_pool *pool, struct page *page,
		     unsigned int dma_sync_size, bool allow_direct)
{
	lockdep_assert_no_hardirq();

	/* This allocator is optimized for the XDP mode that uses
	 * one-frame-per-page, but have fallbacks that act like the
	 * regular page allocator APIs.
	 *
	 * refcnt == 1 means page_pool owns page, and can recycle it.
	 *
	 * page is NOT reusable when allocated when system is under
	 * some pressure. (page_is_pfmemalloc)
	 */
	if (likely(__page_pool_page_can_be_recycled(page))) {
		/* Read barrier done in page_ref_count / READ_ONCE */

		page_pool_dma_sync_for_device(pool, page, dma_sync_size);

		if (allow_direct && page_pool_recycle_in_cache(page, pool))
			return NULL;

		/* Page found as candidate for recycling */
		return page;
	}
	/* Fallback/non-XDP mode: API user have elevated refcnt.
	 *
	 * Many drivers split up the page into fragments, and some
	 * want to keep doing this to save memory and do refcnt based
	 * recycling. Support this use case too, to ease drivers
	 * switching between XDP/non-XDP.
	 *
	 * In-case page_pool maintains the DMA mapping, API user must
	 * call page_pool_put_page once.  In this elevated refcnt
	 * case, the DMA is unmapped/released, as driver is likely
	 * doing refcnt based recycle tricks, meaning another process
	 * will be invoking put_page.
	 */
	recycle_stat_inc(pool, released_refcnt);
	page_pool_return_page(pool, page);

	return NULL;
}

static bool page_pool_napi_local(const struct page_pool *pool)
{
	const struct napi_struct *napi;
	u32 cpuid;

	if (unlikely(!in_softirq()))
		return false;

	/* Allow direct recycle if we have reasons to believe that we are
	 * in the same context as the consumer would run, so there's
	 * no possible race.
	 * __page_pool_put_page() makes sure we're not in hardirq context
	 * and interrupts are enabled prior to accessing the cache.
	 */
	cpuid = smp_processor_id();
	if (READ_ONCE(pool->cpuid) == cpuid)
		return true;

	napi = READ_ONCE(pool->p.napi);

	return napi && READ_ONCE(napi->list_owner) == cpuid;
}

void page_pool_put_unrefed_page(struct page_pool *pool, struct page *page,
				unsigned int dma_sync_size, bool allow_direct)
{
	if (!allow_direct)
		allow_direct = page_pool_napi_local(pool);

	page = __page_pool_put_page(pool, page, dma_sync_size, allow_direct);
	if (page && !page_pool_recycle_in_ring(pool, page)) {
		/* Cache full, fallback to free pages */
		recycle_stat_inc(pool, ring_full);
		page_pool_return_page(pool, page);
	}
}
EXPORT_SYMBOL(page_pool_put_unrefed_page);

/**
 * page_pool_put_page_bulk() - release references on multiple pages
 * @pool:	pool from which pages were allocated
 * @data:	array holding page pointers
 * @count:	number of pages in @data
 *
 * Tries to refill a number of pages into the ptr_ring cache holding ptr_ring
 * producer lock. If the ptr_ring is full, page_pool_put_page_bulk()
 * will release leftover pages to the page allocator.
 * page_pool_put_page_bulk() is suitable to be run inside the driver NAPI tx
 * completion loop for the XDP_REDIRECT use case.
 *
 * Please note the caller must not use data area after running
 * page_pool_put_page_bulk(), as this function overwrites it.
 */
void page_pool_put_page_bulk(struct page_pool *pool, void **data,
			     int count)
{
	int i, bulk_len = 0;
	bool allow_direct;
	bool in_softirq;

	allow_direct = page_pool_napi_local(pool);

	for (i = 0; i < count; i++) {
		struct page *page = virt_to_head_page(data[i]);

		/* It is not the last user for the page frag case */
		if (!page_pool_is_last_ref(page))
			continue;

		page = __page_pool_put_page(pool, page, -1, allow_direct);
		/* Approved for bulk recycling in ptr_ring cache */
		if (page)
			data[bulk_len++] = page;
	}

	if (!bulk_len)
		return;

	/* Bulk producer into ptr_ring page_pool cache */
	in_softirq = page_pool_producer_lock(pool);
	for (i = 0; i < bulk_len; i++) {
		if (__ptr_ring_produce(&pool->ring, data[i])) {
			/* ring full */
			recycle_stat_inc(pool, ring_full);
			break;
		}
	}
	recycle_stat_add(pool, ring, i);
	page_pool_producer_unlock(pool, in_softirq);

	/* Hopefully all pages was return into ptr_ring */
	if (likely(i == bulk_len))
		return;

	/* ptr_ring cache full, free remaining pages outside producer lock
	 * since put_page() with refcnt == 1 can be an expensive operation
	 */
	for (; i < bulk_len; i++)
		page_pool_return_page(pool, data[i]);
}
EXPORT_SYMBOL(page_pool_put_page_bulk);

static struct page *page_pool_drain_frag(struct page_pool *pool,
					 struct page *page)
{
	long drain_count = BIAS_MAX - pool->frag_users;

	/* Some user is still using the page frag */
	if (likely(page_pool_unref_page(page, drain_count)))
		return NULL;

	if (__page_pool_page_can_be_recycled(page)) {
		page_pool_dma_sync_for_device(pool, page, -1);
		return page;
	}

	page_pool_return_page(pool, page);
	return NULL;
}

static void page_pool_free_frag(struct page_pool *pool)
{
	long drain_count = BIAS_MAX - pool->frag_users;
	struct page *page = pool->frag_page;

	pool->frag_page = NULL;

	if (!page || page_pool_unref_page(page, drain_count))
		return;

	page_pool_return_page(pool, page);
}

struct page *page_pool_alloc_frag(struct page_pool *pool,
				  unsigned int *offset,
				  unsigned int size, gfp_t gfp)
{
	unsigned int max_size = PAGE_SIZE << pool->p.order;
	struct page *page = pool->frag_page;

	if (WARN_ON(size > max_size))
		return NULL;

	size = ALIGN(size, dma_get_cache_alignment());
	*offset = pool->frag_offset;

	if (page && *offset + size > max_size) {
		page = page_pool_drain_frag(pool, page);
		if (page) {
			alloc_stat_inc(pool, fast);
			goto frag_reset;
		}
	}

	if (!page) {
		page = page_pool_alloc_pages(pool, gfp);
		if (unlikely(!page)) {
			pool->frag_page = NULL;
			return NULL;
		}

		pool->frag_page = page;

frag_reset:
		pool->frag_users = 1;
		*offset = 0;
		pool->frag_offset = size;
		page_pool_fragment_page(page, BIAS_MAX);
		return page;
	}

	pool->frag_users++;
	pool->frag_offset = *offset + size;
	alloc_stat_inc(pool, fast);
	return page;
}
EXPORT_SYMBOL(page_pool_alloc_frag);

static void page_pool_empty_ring(struct page_pool *pool)
{
	struct page *page;

	/* Empty recycle ring */
	while ((page = ptr_ring_consume_bh(&pool->ring))) {
		/* Verify the refcnt invariant of cached pages */
		if (!(page_ref_count(page) == 1))
			pr_crit("%s() page_pool refcnt %d violation\n",
				__func__, page_ref_count(page));

		page_pool_return_page(pool, page);
	}
}

static void __page_pool_destroy(struct page_pool *pool)
{
	if (pool->disconnect)
		pool->disconnect(pool);

	page_pool_unlist(pool);
	page_pool_uninit(pool);
	kfree(pool);
}

static void page_pool_empty_alloc_cache_once(struct page_pool *pool)
{
	struct page *page;

	if (pool->destroy_cnt)
		return;

	/* Empty alloc cache, assume caller made sure this is
	 * no-longer in use, and page_pool_alloc_pages() cannot be
	 * call concurrently.
	 */
	while (pool->alloc.count) {
		page = pool->alloc.cache[--pool->alloc.count];
		page_pool_return_page(pool, page);
	}
}

static void page_pool_scrub(struct page_pool *pool)
{
	page_pool_empty_alloc_cache_once(pool);
	pool->destroy_cnt++;

	/* No more consumers should exist, but producers could still
	 * be in-flight.
	 */
	page_pool_empty_ring(pool);
}

static int page_pool_release(struct page_pool *pool)
{
	int inflight;

	page_pool_scrub(pool);
	inflight = page_pool_inflight(pool, true);
	if (!inflight)
		__page_pool_destroy(pool);

	return inflight;
}

static void page_pool_release_retry(struct work_struct *wq)
{
	struct delayed_work *dwq = to_delayed_work(wq);
	struct page_pool *pool = container_of(dwq, typeof(*pool), release_dw);
	void *netdev;
	int inflight;

	inflight = page_pool_release(pool);
	if (!inflight)
		return;

	/* Periodic warning for page pools the user can't see */
	netdev = READ_ONCE(pool->slow.netdev);
	if (time_after_eq(jiffies, pool->defer_warn) &&
	    (!netdev || netdev == NET_PTR_POISON)) {
		int sec = (s32)((u32)jiffies - (u32)pool->defer_start) / HZ;

		pr_warn("%s() stalled pool shutdown: id %u, %d inflight %d sec\n",
			__func__, pool->user.id, inflight, sec);
		pool->defer_warn = jiffies + DEFER_WARN_INTERVAL;
	}

	/* Still not ready to be disconnected, retry later */
	schedule_delayed_work(&pool->release_dw, DEFER_TIME);
}

void page_pool_use_xdp_mem(struct page_pool *pool, void (*disconnect)(void *),
			   const struct xdp_mem_info *mem)
{
	refcount_inc(&pool->user_cnt);
	pool->disconnect = disconnect;
	pool->xdp_mem_id = mem->id;
}

static void page_pool_disable_direct_recycling(struct page_pool *pool)
{
	/* Disable direct recycling based on pool->cpuid.
	 * Paired with READ_ONCE() in page_pool_napi_local().
	 */
	WRITE_ONCE(pool->cpuid, -1);

	if (!pool->p.napi)
		return;

	/* To avoid races with recycling and additional barriers make sure
	 * pool and NAPI are unlinked when NAPI is disabled.
	 */
	WARN_ON(!test_bit(NAPI_STATE_SCHED, &pool->p.napi->state) ||
		READ_ONCE(pool->p.napi->list_owner) != -1);

	WRITE_ONCE(pool->p.napi, NULL);
}

void page_pool_destroy(struct page_pool *pool)
{
	if (!pool)
		return;

	if (!page_pool_put(pool))
		return;

	page_pool_disable_direct_recycling(pool);
	page_pool_free_frag(pool);

	if (!page_pool_release(pool))
		return;

	page_pool_detached(pool);
	pool->defer_start = jiffies;
	pool->defer_warn  = jiffies + DEFER_WARN_INTERVAL;

	INIT_DELAYED_WORK(&pool->release_dw, page_pool_release_retry);
	schedule_delayed_work(&pool->release_dw, DEFER_TIME);
}
EXPORT_SYMBOL(page_pool_destroy);

/* Caller must provide appropriate safe context, e.g. NAPI. */
void page_pool_update_nid(struct page_pool *pool, int new_nid)
{
	struct page *page;

	trace_page_pool_update_nid(pool, new_nid);
	pool->p.nid = new_nid;

	/* Flush pool alloc cache, as refill will check NUMA node */
	while (pool->alloc.count) {
		page = pool->alloc.cache[--pool->alloc.count];
		page_pool_return_page(pool, page);
	}
}
EXPORT_SYMBOL(page_pool_update_nid);
