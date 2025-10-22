// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * zswap.c - zswap driver file
 *
 * zswap is a cache that takes pages that are in the process
 * of being swapped out and attempts to compress and store them in a
 * RAM-based memory pool.  This can result in a significant I/O reduction on
 * the swap device and, in the case where decompressing from RAM is faster
 * than reading from the swap device, can also improve workload performance.
 *
 * Copyright (C) 2012  Seth Jennings <sjenning@linux.vnet.ibm.com>
*/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/highmem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/swap.h>
#include <linux/crypto.h>
#include <linux/scatterlist.h>
#include <linux/mempolicy.h>
#include <linux/mempool.h>
#include <crypto/acompress.h>
#include <linux/zswap.h>
#include <linux/mm_types.h>
#include <linux/page-flags.h>
#include <linux/swapops.h>
#include <linux/writeback.h>
#include <linux/pagemap.h>
#include <linux/workqueue.h>
#include <linux/list_lru.h>
#include <linux/zsmalloc.h>

#include "swap.h"
#include "internal.h"

/*********************************
* statistics
**********************************/
/* The number of pages currently stored in zswap */
atomic_long_t zswap_stored_pages = ATOMIC_LONG_INIT(0);
/* The number of incompressible pages currently stored in zswap */
static atomic_long_t zswap_stored_incompressible_pages = ATOMIC_LONG_INIT(0);

/*
 * The statistics below are not protected from concurrent access for
 * performance reasons so they may not be a 100% accurate.  However,
 * they do provide useful information on roughly how many times a
 * certain event is occurring.
*/

/* Pool limit was hit (see zswap_max_pool_percent) */
static u64 zswap_pool_limit_hit;
/* Pages written back when pool limit was reached */
static u64 zswap_written_back_pages;
/* Store failed due to a reclaim failure after pool limit was reached */
static u64 zswap_reject_reclaim_fail;
/* Store failed due to compression algorithm failure */
static u64 zswap_reject_compress_fail;
/* Compressed page was too big for the allocator to (optimally) store */
static u64 zswap_reject_compress_poor;
/* Load or writeback failed due to decompression failure */
static u64 zswap_decompress_fail;
/* Store failed because underlying allocator could not get memory */
static u64 zswap_reject_alloc_fail;
/* Store failed because the entry metadata could not be allocated (rare) */
static u64 zswap_reject_kmemcache_fail;

/* Shrinker work queue */
static struct workqueue_struct *shrink_wq;
/* Pool limit was hit, we need to calm down */
static bool zswap_pool_reached_full;

/*********************************
* tunables
**********************************/

#define ZSWAP_PARAM_UNSET ""

static int zswap_setup(void);

/* Enable/disable zswap */
static DEFINE_STATIC_KEY_MAYBE(CONFIG_ZSWAP_DEFAULT_ON, zswap_ever_enabled);
static bool zswap_enabled = IS_ENABLED(CONFIG_ZSWAP_DEFAULT_ON);
static int zswap_enabled_param_set(const char *,
				   const struct kernel_param *);
static const struct kernel_param_ops zswap_enabled_param_ops = {
	.set =		zswap_enabled_param_set,
	.get =		param_get_bool,
};
module_param_cb(enabled, &zswap_enabled_param_ops, &zswap_enabled, 0644);

/* Crypto compressor to use */
static char *zswap_compressor = CONFIG_ZSWAP_COMPRESSOR_DEFAULT;
static int zswap_compressor_param_set(const char *,
				      const struct kernel_param *);
static const struct kernel_param_ops zswap_compressor_param_ops = {
	.set =		zswap_compressor_param_set,
	.get =		param_get_charp,
	.free =		param_free_charp,
};
module_param_cb(compressor, &zswap_compressor_param_ops,
		&zswap_compressor, 0644);

/* The maximum percentage of memory that the compressed pool can occupy */
static unsigned int zswap_max_pool_percent = 20;
module_param_named(max_pool_percent, zswap_max_pool_percent, uint, 0644);

/* The threshold for accepting new pages after the max_pool_percent was hit */
static unsigned int zswap_accept_thr_percent = 90; /* of max pool size */
module_param_named(accept_threshold_percent, zswap_accept_thr_percent,
		   uint, 0644);

/* Enable/disable memory pressure-based shrinker. */
static bool zswap_shrinker_enabled = IS_ENABLED(
		CONFIG_ZSWAP_SHRINKER_DEFAULT_ON);
module_param_named(shrinker_enabled, zswap_shrinker_enabled, bool, 0644);

bool zswap_is_enabled(void)
{
	return zswap_enabled;
}

bool zswap_never_enabled(void)
{
	return !static_branch_maybe(CONFIG_ZSWAP_DEFAULT_ON, &zswap_ever_enabled);
}

/*********************************
* data structures
**********************************/

struct crypto_acomp_ctx {
	struct crypto_acomp *acomp;
	struct acomp_req *req;
	struct crypto_wait wait;
	u8 *buffer;
	struct mutex mutex;
	bool is_sleepable;
};

/*
 * The lock ordering is zswap_tree.lock -> zswap_pool.lru_lock.
 * The only case where lru_lock is not acquired while holding tree.lock is
 * when a zswap_entry is taken off the lru for writeback, in that case it
 * needs to be verified that it's still valid in the tree.
 */
struct zswap_pool {
	struct zs_pool *zs_pool;
	struct crypto_acomp_ctx __percpu *acomp_ctx;
	struct percpu_ref ref;
	struct list_head list;
	struct work_struct release_work;
	struct hlist_node node;
	char tfm_name[CRYPTO_MAX_ALG_NAME];
};

/* Global LRU lists shared by all zswap pools. */
static struct list_lru zswap_list_lru;

/* The lock protects zswap_next_shrink updates. */
static DEFINE_SPINLOCK(zswap_shrink_lock);
static struct mem_cgroup *zswap_next_shrink;
static struct work_struct zswap_shrink_work;
static struct shrinker *zswap_shrinker;

/*
 * struct zswap_entry
 *
 * This structure contains the metadata for tracking a single compressed
 * page within zswap.
 *
 * swpentry - associated swap entry, the offset indexes into the red-black tree
 * length - the length in bytes of the compressed page data.  Needed during
 *          decompression.
 * referenced - true if the entry recently entered the zswap pool. Unset by the
 *              writeback logic. The entry is only reclaimed by the writeback
 *              logic if referenced is unset. See comments in the shrinker
 *              section for context.
 * pool - the zswap_pool the entry's data is in
 * handle - zsmalloc allocation handle that stores the compressed page data
 * objcg - the obj_cgroup that the compressed memory is charged to
 * lru - handle to the pool's lru used to evict pages.
 */
struct zswap_entry {
	swp_entry_t swpentry;
	unsigned int length;
	bool referenced;
	struct zswap_pool *pool;
	unsigned long handle;
	struct obj_cgroup *objcg;
	struct list_head lru;
};

static struct xarray *zswap_trees[MAX_SWAPFILES];
static unsigned int nr_zswap_trees[MAX_SWAPFILES];

/* RCU-protected iteration */
static LIST_HEAD(zswap_pools);
/* protects zswap_pools list modification */
static DEFINE_SPINLOCK(zswap_pools_lock);
/* pool counter to provide unique names to zsmalloc */
static atomic_t zswap_pools_count = ATOMIC_INIT(0);

enum zswap_init_type {
	ZSWAP_UNINIT,
	ZSWAP_INIT_SUCCEED,
	ZSWAP_INIT_FAILED
};

static enum zswap_init_type zswap_init_state;

/* used to ensure the integrity of initialization */
static DEFINE_MUTEX(zswap_init_lock);

/* init completed, but couldn't create the initial pool */
static bool zswap_has_pool;

/*********************************
* helpers and fwd declarations
**********************************/

/* One swap address space for each 64M swap space */
#define ZSWAP_ADDRESS_SPACE_SHIFT 14
#define ZSWAP_ADDRESS_SPACE_PAGES (1 << ZSWAP_ADDRESS_SPACE_SHIFT)
static inline struct xarray *swap_zswap_tree(swp_entry_t swp)
{
	return &zswap_trees[swp_type(swp)][swp_offset(swp)
		>> ZSWAP_ADDRESS_SPACE_SHIFT];
}

#define zswap_pool_debug(msg, p)			\
	pr_debug("%s pool %s\n", msg, (p)->tfm_name)

/*********************************
* pool functions
**********************************/
static void __zswap_pool_empty(struct percpu_ref *ref);

static struct zswap_pool *zswap_pool_create(char *compressor)
{
	struct zswap_pool *pool;
	char name[38]; /* 'zswap' + 32 char (max) num + \0 */
	int ret, cpu;

	if (!zswap_has_pool && !strcmp(compressor, ZSWAP_PARAM_UNSET))
		return NULL;

	pool = kzalloc(sizeof(*pool), GFP_KERNEL);
	if (!pool)
		return NULL;

	/* unique name for each pool specifically required by zsmalloc */
	snprintf(name, 38, "zswap%x", atomic_inc_return(&zswap_pools_count));
	pool->zs_pool = zs_create_pool(name);
	if (!pool->zs_pool)
		goto error;

	strscpy(pool->tfm_name, compressor, sizeof(pool->tfm_name));

	pool->acomp_ctx = alloc_percpu(*pool->acomp_ctx);
	if (!pool->acomp_ctx) {
		pr_err("percpu alloc failed\n");
		goto error;
	}

	for_each_possible_cpu(cpu)
		mutex_init(&per_cpu_ptr(pool->acomp_ctx, cpu)->mutex);

	ret = cpuhp_state_add_instance(CPUHP_MM_ZSWP_POOL_PREPARE,
				       &pool->node);
	if (ret)
		goto error;

	/* being the current pool takes 1 ref; this func expects the
	 * caller to always add the new pool as the current pool
	 */
	ret = percpu_ref_init(&pool->ref, __zswap_pool_empty,
			      PERCPU_REF_ALLOW_REINIT, GFP_KERNEL);
	if (ret)
		goto ref_fail;
	INIT_LIST_HEAD(&pool->list);

	zswap_pool_debug("created", pool);

	return pool;

ref_fail:
	cpuhp_state_remove_instance(CPUHP_MM_ZSWP_POOL_PREPARE, &pool->node);
error:
	if (pool->acomp_ctx)
		free_percpu(pool->acomp_ctx);
	if (pool->zs_pool)
		zs_destroy_pool(pool->zs_pool);
	kfree(pool);
	return NULL;
}

static struct zswap_pool *__zswap_pool_create_fallback(void)
{
	if (!crypto_has_acomp(zswap_compressor, 0, 0) &&
	    strcmp(zswap_compressor, CONFIG_ZSWAP_COMPRESSOR_DEFAULT)) {
		pr_err("compressor %s not available, using default %s\n",
		       zswap_compressor, CONFIG_ZSWAP_COMPRESSOR_DEFAULT);
		param_free_charp(&zswap_compressor);
		zswap_compressor = CONFIG_ZSWAP_COMPRESSOR_DEFAULT;
	}

	/* Default compressor should be available. Kconfig bug? */
	if (WARN_ON_ONCE(!crypto_has_acomp(zswap_compressor, 0, 0))) {
		zswap_compressor = ZSWAP_PARAM_UNSET;
		return NULL;
	}

	return zswap_pool_create(zswap_compressor);
}

static void zswap_pool_destroy(struct zswap_pool *pool)
{
	zswap_pool_debug("destroying", pool);

	cpuhp_state_remove_instance(CPUHP_MM_ZSWP_POOL_PREPARE, &pool->node);
	free_percpu(pool->acomp_ctx);

	zs_destroy_pool(pool->zs_pool);
	kfree(pool);
}

static void __zswap_pool_release(struct work_struct *work)
{
	struct zswap_pool *pool = container_of(work, typeof(*pool),
						release_work);

	synchronize_rcu();

	/* nobody should have been able to get a ref... */
	WARN_ON(!percpu_ref_is_zero(&pool->ref));
	percpu_ref_exit(&pool->ref);

	/* pool is now off zswap_pools list and has no references. */
	zswap_pool_destroy(pool);
}

static struct zswap_pool *zswap_pool_current(void);

static void __zswap_pool_empty(struct percpu_ref *ref)
{
	struct zswap_pool *pool;

	pool = container_of(ref, typeof(*pool), ref);

	spin_lock_bh(&zswap_pools_lock);

	WARN_ON(pool == zswap_pool_current());

	list_del_rcu(&pool->list);

	INIT_WORK(&pool->release_work, __zswap_pool_release);
	schedule_work(&pool->release_work);

	spin_unlock_bh(&zswap_pools_lock);
}

static int __must_check zswap_pool_tryget(struct zswap_pool *pool)
{
	if (!pool)
		return 0;

	return percpu_ref_tryget(&pool->ref);
}

/* The caller must already have a reference. */
static void zswap_pool_get(struct zswap_pool *pool)
{
	percpu_ref_get(&pool->ref);
}

static void zswap_pool_put(struct zswap_pool *pool)
{
	percpu_ref_put(&pool->ref);
}

static struct zswap_pool *__zswap_pool_current(void)
{
	struct zswap_pool *pool;

	pool = list_first_or_null_rcu(&zswap_pools, typeof(*pool), list);
	WARN_ONCE(!pool && zswap_has_pool,
		  "%s: no page storage pool!\n", __func__);

	return pool;
}

static struct zswap_pool *zswap_pool_current(void)
{
	assert_spin_locked(&zswap_pools_lock);

	return __zswap_pool_current();
}

static struct zswap_pool *zswap_pool_current_get(void)
{
	struct zswap_pool *pool;

	rcu_read_lock();

	pool = __zswap_pool_current();
	if (!zswap_pool_tryget(pool))
		pool = NULL;

	rcu_read_unlock();

	return pool;
}

/* type and compressor must be null-terminated */
static struct zswap_pool *zswap_pool_find_get(char *compressor)
{
	struct zswap_pool *pool;

	assert_spin_locked(&zswap_pools_lock);

	list_for_each_entry_rcu(pool, &zswap_pools, list) {
		if (strcmp(pool->tfm_name, compressor))
			continue;
		/* if we can't get it, it's about to be destroyed */
		if (!zswap_pool_tryget(pool))
			continue;
		return pool;
	}

	return NULL;
}

static unsigned long zswap_max_pages(void)
{
	return totalram_pages() * zswap_max_pool_percent / 100;
}

static unsigned long zswap_accept_thr_pages(void)
{
	return zswap_max_pages() * zswap_accept_thr_percent / 100;
}

unsigned long zswap_total_pages(void)
{
	struct zswap_pool *pool;
	unsigned long total = 0;

	rcu_read_lock();
	list_for_each_entry_rcu(pool, &zswap_pools, list)
		total += zs_get_total_pages(pool->zs_pool);
	rcu_read_unlock();

	return total;
}

static bool zswap_check_limits(void)
{
	unsigned long cur_pages = zswap_total_pages();
	unsigned long max_pages = zswap_max_pages();

	if (cur_pages >= max_pages) {
		zswap_pool_limit_hit++;
		zswap_pool_reached_full = true;
	} else if (zswap_pool_reached_full &&
		   cur_pages <= zswap_accept_thr_pages()) {
			zswap_pool_reached_full = false;
	}
	return zswap_pool_reached_full;
}

/*********************************
* param callbacks
**********************************/

static int zswap_compressor_param_set(const char *val, const struct kernel_param *kp)
{
	struct zswap_pool *pool, *put_pool = NULL;
	char *s = strstrip((char *)val);
	bool create_pool = false;
	int ret = 0;

	mutex_lock(&zswap_init_lock);
	switch (zswap_init_state) {
	case ZSWAP_UNINIT:
		/* Handled in zswap_setup() */
		ret = param_set_charp(s, kp);
		break;
	case ZSWAP_INIT_SUCCEED:
		if (!zswap_has_pool || strcmp(s, *(char **)kp->arg))
			create_pool = true;
		break;
	case ZSWAP_INIT_FAILED:
		pr_err("can't set param, initialization failed\n");
		ret = -ENODEV;
	}
	mutex_unlock(&zswap_init_lock);

	if (!create_pool)
		return ret;

	if (!crypto_has_acomp(s, 0, 0)) {
		pr_err("compressor %s not available\n", s);
		return -ENOENT;
	}

	spin_lock_bh(&zswap_pools_lock);

	pool = zswap_pool_find_get(s);
	if (pool) {
		zswap_pool_debug("using existing", pool);
		WARN_ON(pool == zswap_pool_current());
		list_del_rcu(&pool->list);
	}

	spin_unlock_bh(&zswap_pools_lock);

	if (!pool)
		pool = zswap_pool_create(s);
	else {
		/*
		 * Restore the initial ref dropped by percpu_ref_kill()
		 * when the pool was decommissioned and switch it again
		 * to percpu mode.
		 */
		percpu_ref_resurrect(&pool->ref);

		/* Drop the ref from zswap_pool_find_get(). */
		zswap_pool_put(pool);
	}

	if (pool)
		ret = param_set_charp(s, kp);
	else
		ret = -EINVAL;

	spin_lock_bh(&zswap_pools_lock);

	if (!ret) {
		put_pool = zswap_pool_current();
		list_add_rcu(&pool->list, &zswap_pools);
		zswap_has_pool = true;
	} else if (pool) {
		/*
		 * Add the possibly pre-existing pool to the end of the pools
		 * list; if it's new (and empty) then it'll be removed and
		 * destroyed by the put after we drop the lock
		 */
		list_add_tail_rcu(&pool->list, &zswap_pools);
		put_pool = pool;
	}

	spin_unlock_bh(&zswap_pools_lock);

	/*
	 * Drop the ref from either the old current pool,
	 * or the new pool we failed to add
	 */
	if (put_pool)
		percpu_ref_kill(&put_pool->ref);

	return ret;
}

static int zswap_enabled_param_set(const char *val,
				   const struct kernel_param *kp)
{
	int ret = -ENODEV;

	/* if this is load-time (pre-init) param setting, only set param. */
	if (system_state != SYSTEM_RUNNING)
		return param_set_bool(val, kp);

	mutex_lock(&zswap_init_lock);
	switch (zswap_init_state) {
	case ZSWAP_UNINIT:
		if (zswap_setup())
			break;
		fallthrough;
	case ZSWAP_INIT_SUCCEED:
		if (!zswap_has_pool)
			pr_err("can't enable, no pool configured\n");
		else
			ret = param_set_bool(val, kp);
		break;
	case ZSWAP_INIT_FAILED:
		pr_err("can't enable, initialization failed\n");
	}
	mutex_unlock(&zswap_init_lock);

	return ret;
}

/*********************************
* lru functions
**********************************/

/* should be called under RCU */
#ifdef CONFIG_MEMCG
static inline struct mem_cgroup *mem_cgroup_from_entry(struct zswap_entry *entry)
{
	return entry->objcg ? obj_cgroup_memcg(entry->objcg) : NULL;
}
#else
static inline struct mem_cgroup *mem_cgroup_from_entry(struct zswap_entry *entry)
{
	return NULL;
}
#endif

static inline int entry_to_nid(struct zswap_entry *entry)
{
	return page_to_nid(virt_to_page(entry));
}

static void zswap_lru_add(struct list_lru *list_lru, struct zswap_entry *entry)
{
	int nid = entry_to_nid(entry);
	struct mem_cgroup *memcg;

	/*
	 * Note that it is safe to use rcu_read_lock() here, even in the face of
	 * concurrent memcg offlining:
	 *
	 * 1. list_lru_add() is called before list_lru_one is dead. The
	 *    new entry will be reparented to memcg's parent's list_lru.
	 * 2. list_lru_add() is called after list_lru_one is dead. The
	 *    new entry will be added directly to memcg's parent's list_lru.
	 *
	 * Similar reasoning holds for list_lru_del().
	 */
	rcu_read_lock();
	memcg = mem_cgroup_from_entry(entry);
	/* will always succeed */
	list_lru_add(list_lru, &entry->lru, nid, memcg);
	rcu_read_unlock();
}

static void zswap_lru_del(struct list_lru *list_lru, struct zswap_entry *entry)
{
	int nid = entry_to_nid(entry);
	struct mem_cgroup *memcg;

	rcu_read_lock();
	memcg = mem_cgroup_from_entry(entry);
	/* will always succeed */
	list_lru_del(list_lru, &entry->lru, nid, memcg);
	rcu_read_unlock();
}

void zswap_lruvec_state_init(struct lruvec *lruvec)
{
	atomic_long_set(&lruvec->zswap_lruvec_state.nr_disk_swapins, 0);
}

void zswap_folio_swapin(struct folio *folio)
{
	struct lruvec *lruvec;

	if (folio) {
		lruvec = folio_lruvec(folio);
		atomic_long_inc(&lruvec->zswap_lruvec_state.nr_disk_swapins);
	}
}

/*
 * This function should be called when a memcg is being offlined.
 *
 * Since the global shrinker shrink_worker() may hold a reference
 * of the memcg, we must check and release the reference in
 * zswap_next_shrink.
 *
 * shrink_worker() must handle the case where this function releases
 * the reference of memcg being shrunk.
 */
void zswap_memcg_offline_cleanup(struct mem_cgroup *memcg)
{
	/* lock out zswap shrinker walking memcg tree */
	spin_lock(&zswap_shrink_lock);
	if (zswap_next_shrink == memcg) {
		do {
			zswap_next_shrink = mem_cgroup_iter(NULL, zswap_next_shrink, NULL);
		} while (zswap_next_shrink && !mem_cgroup_online(zswap_next_shrink));
	}
	spin_unlock(&zswap_shrink_lock);
}

/*********************************
* zswap entry functions
**********************************/
static struct kmem_cache *zswap_entry_cache;

static struct zswap_entry *zswap_entry_cache_alloc(gfp_t gfp, int nid)
{
	struct zswap_entry *entry;
	entry = kmem_cache_alloc_node(zswap_entry_cache, gfp, nid);
	if (!entry)
		return NULL;
	return entry;
}

static void zswap_entry_cache_free(struct zswap_entry *entry)
{
	kmem_cache_free(zswap_entry_cache, entry);
}

/*
 * Carries out the common pattern of freeing an entry's zsmalloc allocation,
 * freeing the entry itself, and decrementing the number of stored pages.
 */
static void zswap_entry_free(struct zswap_entry *entry)
{
	zswap_lru_del(&zswap_list_lru, entry);
	zs_free(entry->pool->zs_pool, entry->handle);
	zswap_pool_put(entry->pool);
	if (entry->objcg) {
		obj_cgroup_uncharge_zswap(entry->objcg, entry->length);
		obj_cgroup_put(entry->objcg);
	}
	if (entry->length == PAGE_SIZE)
		atomic_long_dec(&zswap_stored_incompressible_pages);
	zswap_entry_cache_free(entry);
	atomic_long_dec(&zswap_stored_pages);
}

/*********************************
* compressed storage functions
**********************************/
static int zswap_cpu_comp_prepare(unsigned int cpu, struct hlist_node *node)
{
	struct zswap_pool *pool = hlist_entry(node, struct zswap_pool, node);
	struct crypto_acomp_ctx *acomp_ctx = per_cpu_ptr(pool->acomp_ctx, cpu);
	struct crypto_acomp *acomp = NULL;
	struct acomp_req *req = NULL;
	u8 *buffer = NULL;
	int ret;

	buffer = kmalloc_node(PAGE_SIZE, GFP_KERNEL, cpu_to_node(cpu));
	if (!buffer) {
		ret = -ENOMEM;
		goto fail;
	}

	acomp = crypto_alloc_acomp_node(pool->tfm_name, 0, 0, cpu_to_node(cpu));
	if (IS_ERR(acomp)) {
		pr_err("could not alloc crypto acomp %s : %ld\n",
				pool->tfm_name, PTR_ERR(acomp));
		ret = PTR_ERR(acomp);
		goto fail;
	}

	req = acomp_request_alloc(acomp);
	if (!req) {
		pr_err("could not alloc crypto acomp_request %s\n",
		       pool->tfm_name);
		ret = -ENOMEM;
		goto fail;
	}

	/*
	 * Only hold the mutex after completing allocations, otherwise we may
	 * recurse into zswap through reclaim and attempt to hold the mutex
	 * again resulting in a deadlock.
	 */
	mutex_lock(&acomp_ctx->mutex);
	crypto_init_wait(&acomp_ctx->wait);

	/*
	 * if the backend of acomp is async zip, crypto_req_done() will wakeup
	 * crypto_wait_req(); if the backend of acomp is scomp, the callback
	 * won't be called, crypto_wait_req() will return without blocking.
	 */
	acomp_request_set_callback(req, CRYPTO_TFM_REQ_MAY_BACKLOG,
				   crypto_req_done, &acomp_ctx->wait);

	acomp_ctx->buffer = buffer;
	acomp_ctx->acomp = acomp;
	acomp_ctx->is_sleepable = acomp_is_async(acomp);
	acomp_ctx->req = req;
	mutex_unlock(&acomp_ctx->mutex);
	return 0;

fail:
	if (acomp)
		crypto_free_acomp(acomp);
	kfree(buffer);
	return ret;
}

static int zswap_cpu_comp_dead(unsigned int cpu, struct hlist_node *node)
{
	struct zswap_pool *pool = hlist_entry(node, struct zswap_pool, node);
	struct crypto_acomp_ctx *acomp_ctx = per_cpu_ptr(pool->acomp_ctx, cpu);
	struct acomp_req *req;
	struct crypto_acomp *acomp;
	u8 *buffer;

	if (IS_ERR_OR_NULL(acomp_ctx))
		return 0;

	mutex_lock(&acomp_ctx->mutex);
	req = acomp_ctx->req;
	acomp = acomp_ctx->acomp;
	buffer = acomp_ctx->buffer;
	acomp_ctx->req = NULL;
	acomp_ctx->acomp = NULL;
	acomp_ctx->buffer = NULL;
	mutex_unlock(&acomp_ctx->mutex);

	/*
	 * Do the actual freeing after releasing the mutex to avoid subtle
	 * locking dependencies causing deadlocks.
	 */
	if (!IS_ERR_OR_NULL(req))
		acomp_request_free(req);
	if (!IS_ERR_OR_NULL(acomp))
		crypto_free_acomp(acomp);
	kfree(buffer);

	return 0;
}

static struct crypto_acomp_ctx *acomp_ctx_get_cpu_lock(struct zswap_pool *pool)
{
	struct crypto_acomp_ctx *acomp_ctx;

	for (;;) {
		acomp_ctx = raw_cpu_ptr(pool->acomp_ctx);
		mutex_lock(&acomp_ctx->mutex);
		if (likely(acomp_ctx->req))
			return acomp_ctx;
		/*
		 * It is possible that we were migrated to a different CPU after
		 * getting the per-CPU ctx but before the mutex was acquired. If
		 * the old CPU got offlined, zswap_cpu_comp_dead() could have
		 * already freed ctx->req (among other things) and set it to
		 * NULL. Just try again on the new CPU that we ended up on.
		 */
		mutex_unlock(&acomp_ctx->mutex);
	}
}

static void acomp_ctx_put_unlock(struct crypto_acomp_ctx *acomp_ctx)
{
	mutex_unlock(&acomp_ctx->mutex);
}

static bool zswap_compress(struct page *page, struct zswap_entry *entry,
			   struct zswap_pool *pool)
{
	struct crypto_acomp_ctx *acomp_ctx;
	struct scatterlist input, output;
	int comp_ret = 0, alloc_ret = 0;
	unsigned int dlen = PAGE_SIZE;
	unsigned long handle;
	gfp_t gfp;
	u8 *dst;
	bool mapped = false;

	acomp_ctx = acomp_ctx_get_cpu_lock(pool);
	dst = acomp_ctx->buffer;
	sg_init_table(&input, 1);
	sg_set_page(&input, page, PAGE_SIZE, 0);

	sg_init_one(&output, dst, PAGE_SIZE);
	acomp_request_set_params(acomp_ctx->req, &input, &output, PAGE_SIZE, dlen);

	/*
	 * it maybe looks a little bit silly that we send an asynchronous request,
	 * then wait for its completion synchronously. This makes the process look
	 * synchronous in fact.
	 * Theoretically, acomp supports users send multiple acomp requests in one
	 * acomp instance, then get those requests done simultaneously. but in this
	 * case, zswap actually does store and load page by page, there is no
	 * existing method to send the second page before the first page is done
	 * in one thread doing zwap.
	 * but in different threads running on different cpu, we have different
	 * acomp instance, so multiple threads can do (de)compression in parallel.
	 */
	comp_ret = crypto_wait_req(crypto_acomp_compress(acomp_ctx->req), &acomp_ctx->wait);
	dlen = acomp_ctx->req->dlen;

	/*
	 * If a page cannot be compressed into a size smaller than PAGE_SIZE,
	 * save the content as is without a compression, to keep the LRU order
	 * of writebacks.  If writeback is disabled, reject the page since it
	 * only adds metadata overhead.  swap_writeout() will put the page back
	 * to the active LRU list in the case.
	 */
	if (comp_ret || !dlen || dlen >= PAGE_SIZE) {
		if (!mem_cgroup_zswap_writeback_enabled(
					folio_memcg(page_folio(page)))) {
			comp_ret = comp_ret ? comp_ret : -EINVAL;
			goto unlock;
		}
		comp_ret = 0;
		dlen = PAGE_SIZE;
		dst = kmap_local_page(page);
		mapped = true;
	}

	gfp = GFP_NOWAIT | __GFP_NORETRY | __GFP_HIGHMEM | __GFP_MOVABLE;
	handle = zs_malloc(pool->zs_pool, dlen, gfp, page_to_nid(page));
	if (IS_ERR_VALUE(handle)) {
		alloc_ret = PTR_ERR((void *)handle);
		goto unlock;
	}

	zs_obj_write(pool->zs_pool, handle, dst, dlen);
	entry->handle = handle;
	entry->length = dlen;

unlock:
	if (mapped)
		kunmap_local(dst);
	if (comp_ret == -ENOSPC || alloc_ret == -ENOSPC)
		zswap_reject_compress_poor++;
	else if (comp_ret)
		zswap_reject_compress_fail++;
	else if (alloc_ret)
		zswap_reject_alloc_fail++;

	acomp_ctx_put_unlock(acomp_ctx);
	return comp_ret == 0 && alloc_ret == 0;
}

static bool zswap_decompress(struct zswap_entry *entry, struct folio *folio)
{
	struct zswap_pool *pool = entry->pool;
	struct scatterlist input, output;
	struct crypto_acomp_ctx *acomp_ctx;
	int decomp_ret = 0, dlen = PAGE_SIZE;
	u8 *src, *obj;

	acomp_ctx = acomp_ctx_get_cpu_lock(pool);
	obj = zs_obj_read_begin(pool->zs_pool, entry->handle, acomp_ctx->buffer);

	/* zswap entries of length PAGE_SIZE are not compressed. */
	if (entry->length == PAGE_SIZE) {
		memcpy_to_folio(folio, 0, obj, entry->length);
		goto read_done;
	}

	/*
	 * zs_obj_read_begin() might return a kmap address of highmem when
	 * acomp_ctx->buffer is not used.  However, sg_init_one() does not
	 * handle highmem addresses, so copy the object to acomp_ctx->buffer.
	 */
	if (virt_addr_valid(obj)) {
		src = obj;
	} else {
		WARN_ON_ONCE(obj == acomp_ctx->buffer);
		memcpy(acomp_ctx->buffer, obj, entry->length);
		src = acomp_ctx->buffer;
	}

	sg_init_one(&input, src, entry->length);
	sg_init_table(&output, 1);
	sg_set_folio(&output, folio, PAGE_SIZE, 0);
	acomp_request_set_params(acomp_ctx->req, &input, &output, entry->length, PAGE_SIZE);
	decomp_ret = crypto_wait_req(crypto_acomp_decompress(acomp_ctx->req), &acomp_ctx->wait);
	dlen = acomp_ctx->req->dlen;

read_done:
	zs_obj_read_end(pool->zs_pool, entry->handle, obj);
	acomp_ctx_put_unlock(acomp_ctx);

	if (!decomp_ret && dlen == PAGE_SIZE)
		return true;

	zswap_decompress_fail++;
	pr_alert_ratelimited("Decompression error from zswap (%d:%lu %s %u->%d)\n",
						swp_type(entry->swpentry),
						swp_offset(entry->swpentry),
						entry->pool->tfm_name, entry->length, dlen);
	return false;
}

/*********************************
* writeback code
**********************************/
/*
 * Attempts to free an entry by adding a folio to the swap cache,
 * decompressing the entry data into the folio, and issuing a
 * bio write to write the folio back to the swap device.
 *
 * This can be thought of as a "resumed writeback" of the folio
 * to the swap device.  We are basically resuming the same swap
 * writeback path that was intercepted with the zswap_store()
 * in the first place.  After the folio has been decompressed into
 * the swap cache, the compressed version stored by zswap can be
 * freed.
 */
static int zswap_writeback_entry(struct zswap_entry *entry,
				 swp_entry_t swpentry)
{
	struct xarray *tree;
	pgoff_t offset = swp_offset(swpentry);
	struct folio *folio;
	struct mempolicy *mpol;
	bool folio_was_allocated;
	struct swap_info_struct *si;
	int ret = 0;

	/* try to allocate swap cache folio */
	si = get_swap_device(swpentry);
	if (!si)
		return -EEXIST;

	mpol = get_task_policy(current);
	folio = __read_swap_cache_async(swpentry, GFP_KERNEL, mpol,
			NO_INTERLEAVE_INDEX, &folio_was_allocated, true);
	put_swap_device(si);
	if (!folio)
		return -ENOMEM;

	/*
	 * Found an existing folio, we raced with swapin or concurrent
	 * shrinker. We generally writeback cold folios from zswap, and
	 * swapin means the folio just became hot, so skip this folio.
	 * For unlikely concurrent shrinker case, it will be unlinked
	 * and freed when invalidated by the concurrent shrinker anyway.
	 */
	if (!folio_was_allocated) {
		ret = -EEXIST;
		goto out;
	}

	/*
	 * folio is locked, and the swapcache is now secured against
	 * concurrent swapping to and from the slot, and concurrent
	 * swapoff so we can safely dereference the zswap tree here.
	 * Verify that the swap entry hasn't been invalidated and recycled
	 * behind our backs, to avoid overwriting a new swap folio with
	 * old compressed data. Only when this is successful can the entry
	 * be dereferenced.
	 */
	tree = swap_zswap_tree(swpentry);
	if (entry != xa_load(tree, offset)) {
		ret = -ENOMEM;
		goto out;
	}

	if (!zswap_decompress(entry, folio)) {
		ret = -EIO;
		goto out;
	}

	xa_erase(tree, offset);

	count_vm_event(ZSWPWB);
	if (entry->objcg)
		count_objcg_events(entry->objcg, ZSWPWB, 1);

	zswap_entry_free(entry);

	/* folio is up to date */
	folio_mark_uptodate(folio);

	/* move it to the tail of the inactive list after end_writeback */
	folio_set_reclaim(folio);

	/* start writeback */
	__swap_writepage(folio, NULL);

out:
	if (ret && ret != -EEXIST) {
		swap_cache_del_folio(folio);
		folio_unlock(folio);
	}
	folio_put(folio);
	return ret;
}

/*********************************
* shrinker functions
**********************************/
/*
 * The dynamic shrinker is modulated by the following factors:
 *
 * 1. Each zswap entry has a referenced bit, which the shrinker unsets (giving
 *    the entry a second chance) before rotating it in the LRU list. If the
 *    entry is considered again by the shrinker, with its referenced bit unset,
 *    it is written back. The writeback rate as a result is dynamically
 *    adjusted by the pool activities - if the pool is dominated by new entries
 *    (i.e lots of recent zswapouts), these entries will be protected and
 *    the writeback rate will slow down. On the other hand, if the pool has a
 *    lot of stagnant entries, these entries will be reclaimed immediately,
 *    effectively increasing the writeback rate.
 *
 * 2. Swapins counter: If we observe swapins, it is a sign that we are
 *    overshrinking and should slow down. We maintain a swapins counter, which
 *    is consumed and subtract from the number of eligible objects on the LRU
 *    in zswap_shrinker_count().
 *
 * 3. Compression ratio. The better the workload compresses, the less gains we
 *    can expect from writeback. We scale down the number of objects available
 *    for reclaim by this ratio.
 */
static enum lru_status shrink_memcg_cb(struct list_head *item, struct list_lru_one *l,
				       void *arg)
{
	struct zswap_entry *entry = container_of(item, struct zswap_entry, lru);
	bool *encountered_page_in_swapcache = (bool *)arg;
	swp_entry_t swpentry;
	enum lru_status ret = LRU_REMOVED_RETRY;
	int writeback_result;

	/*
	 * Second chance algorithm: if the entry has its referenced bit set, give it
	 * a second chance. Only clear the referenced bit and rotate it in the
	 * zswap's LRU list.
	 */
	if (entry->referenced) {
		entry->referenced = false;
		return LRU_ROTATE;
	}

	/*
	 * As soon as we drop the LRU lock, the entry can be freed by
	 * a concurrent invalidation. This means the following:
	 *
	 * 1. We extract the swp_entry_t to the stack, allowing
	 *    zswap_writeback_entry() to pin the swap entry and
	 *    then validate the zwap entry against that swap entry's
	 *    tree using pointer value comparison. Only when that
	 *    is successful can the entry be dereferenced.
	 *
	 * 2. Usually, objects are taken off the LRU for reclaim. In
	 *    this case this isn't possible, because if reclaim fails
	 *    for whatever reason, we have no means of knowing if the
	 *    entry is alive to put it back on the LRU.
	 *
	 *    So rotate it before dropping the lock. If the entry is
	 *    written back or invalidated, the free path will unlink
	 *    it. For failures, rotation is the right thing as well.
	 *
	 *    Temporary failures, where the same entry should be tried
	 *    again immediately, almost never happen for this shrinker.
	 *    We don't do any trylocking; -ENOMEM comes closest,
	 *    but that's extremely rare and doesn't happen spuriously
	 *    either. Don't bother distinguishing this case.
	 */
	list_move_tail(item, &l->list);

	/*
	 * Once the lru lock is dropped, the entry might get freed. The
	 * swpentry is copied to the stack, and entry isn't deref'd again
	 * until the entry is verified to still be alive in the tree.
	 */
	swpentry = entry->swpentry;

	/*
	 * It's safe to drop the lock here because we return either
	 * LRU_REMOVED_RETRY, LRU_RETRY or LRU_STOP.
	 */
	spin_unlock(&l->lock);

	writeback_result = zswap_writeback_entry(entry, swpentry);

	if (writeback_result) {
		zswap_reject_reclaim_fail++;
		ret = LRU_RETRY;

		/*
		 * Encountering a page already in swap cache is a sign that we are shrinking
		 * into the warmer region. We should terminate shrinking (if we're in the dynamic
		 * shrinker context).
		 */
		if (writeback_result == -EEXIST && encountered_page_in_swapcache) {
			ret = LRU_STOP;
			*encountered_page_in_swapcache = true;
		}
	} else {
		zswap_written_back_pages++;
	}

	return ret;
}

static unsigned long zswap_shrinker_scan(struct shrinker *shrinker,
		struct shrink_control *sc)
{
	unsigned long shrink_ret;
	bool encountered_page_in_swapcache = false;

	if (!zswap_shrinker_enabled ||
			!mem_cgroup_zswap_writeback_enabled(sc->memcg)) {
		sc->nr_scanned = 0;
		return SHRINK_STOP;
	}

	shrink_ret = list_lru_shrink_walk(&zswap_list_lru, sc, &shrink_memcg_cb,
		&encountered_page_in_swapcache);

	if (encountered_page_in_swapcache)
		return SHRINK_STOP;

	return shrink_ret ? shrink_ret : SHRINK_STOP;
}

static unsigned long zswap_shrinker_count(struct shrinker *shrinker,
		struct shrink_control *sc)
{
	struct mem_cgroup *memcg = sc->memcg;
	struct lruvec *lruvec = mem_cgroup_lruvec(memcg, NODE_DATA(sc->nid));
	atomic_long_t *nr_disk_swapins =
		&lruvec->zswap_lruvec_state.nr_disk_swapins;
	unsigned long nr_backing, nr_stored, nr_freeable, nr_disk_swapins_cur,
		nr_remain;

	if (!zswap_shrinker_enabled || !mem_cgroup_zswap_writeback_enabled(memcg))
		return 0;

	/*
	 * The shrinker resumes swap writeback, which will enter block
	 * and may enter fs. XXX: Harmonize with vmscan.c __GFP_FS
	 * rules (may_enter_fs()), which apply on a per-folio basis.
	 */
	if (!gfp_has_io_fs(sc->gfp_mask))
		return 0;

	/*
	 * For memcg, use the cgroup-wide ZSWAP stats since we don't
	 * have them per-node and thus per-lruvec. Careful if memcg is
	 * runtime-disabled: we can get sc->memcg == NULL, which is ok
	 * for the lruvec, but not for memcg_page_state().
	 *
	 * Without memcg, use the zswap pool-wide metrics.
	 */
	if (!mem_cgroup_disabled()) {
		mem_cgroup_flush_stats(memcg);
		nr_backing = memcg_page_state(memcg, MEMCG_ZSWAP_B) >> PAGE_SHIFT;
		nr_stored = memcg_page_state(memcg, MEMCG_ZSWAPPED);
	} else {
		nr_backing = zswap_total_pages();
		nr_stored = atomic_long_read(&zswap_stored_pages);
	}

	if (!nr_stored)
		return 0;

	nr_freeable = list_lru_shrink_count(&zswap_list_lru, sc);
	if (!nr_freeable)
		return 0;

	/*
	 * Subtract from the lru size the number of pages that are recently swapped
	 * in from disk. The idea is that had we protect the zswap's LRU by this
	 * amount of pages, these disk swapins would not have happened.
	 */
	nr_disk_swapins_cur = atomic_long_read(nr_disk_swapins);
	do {
		if (nr_freeable >= nr_disk_swapins_cur)
			nr_remain = 0;
		else
			nr_remain = nr_disk_swapins_cur - nr_freeable;
	} while (!atomic_long_try_cmpxchg(
		nr_disk_swapins, &nr_disk_swapins_cur, nr_remain));

	nr_freeable -= nr_disk_swapins_cur - nr_remain;
	if (!nr_freeable)
		return 0;

	/*
	 * Scale the number of freeable pages by the memory saving factor.
	 * This ensures that the better zswap compresses memory, the fewer
	 * pages we will evict to swap (as it will otherwise incur IO for
	 * relatively small memory saving).
	 */
	return mult_frac(nr_freeable, nr_backing, nr_stored);
}

static struct shrinker *zswap_alloc_shrinker(void)
{
	struct shrinker *shrinker;

	shrinker =
		shrinker_alloc(SHRINKER_NUMA_AWARE | SHRINKER_MEMCG_AWARE, "mm-zswap");
	if (!shrinker)
		return NULL;

	shrinker->scan_objects = zswap_shrinker_scan;
	shrinker->count_objects = zswap_shrinker_count;
	shrinker->batch = 0;
	shrinker->seeks = DEFAULT_SEEKS;
	return shrinker;
}

static int shrink_memcg(struct mem_cgroup *memcg)
{
	int nid, shrunk = 0, scanned = 0;

	if (!mem_cgroup_zswap_writeback_enabled(memcg))
		return -ENOENT;

	/*
	 * Skip zombies because their LRUs are reparented and we would be
	 * reclaiming from the parent instead of the dead memcg.
	 */
	if (memcg && !mem_cgroup_online(memcg))
		return -ENOENT;

	for_each_node_state(nid, N_NORMAL_MEMORY) {
		unsigned long nr_to_walk = 1;

		shrunk += list_lru_walk_one(&zswap_list_lru, nid, memcg,
					    &shrink_memcg_cb, NULL, &nr_to_walk);
		scanned += 1 - nr_to_walk;
	}

	if (!scanned)
		return -ENOENT;

	return shrunk ? 0 : -EAGAIN;
}

static void shrink_worker(struct work_struct *w)
{
	struct mem_cgroup *memcg;
	int ret, failures = 0, attempts = 0;
	unsigned long thr;

	/* Reclaim down to the accept threshold */
	thr = zswap_accept_thr_pages();

	/*
	 * Global reclaim will select cgroup in a round-robin fashion from all
	 * online memcgs, but memcgs that have no pages in zswap and
	 * writeback-disabled memcgs (memory.zswap.writeback=0) are not
	 * candidates for shrinking.
	 *
	 * Shrinking will be aborted if we encounter the following
	 * MAX_RECLAIM_RETRIES times:
	 * - No writeback-candidate memcgs found in a memcg tree walk.
	 * - Shrinking a writeback-candidate memcg failed.
	 *
	 * We save iteration cursor memcg into zswap_next_shrink,
	 * which can be modified by the offline memcg cleaner
	 * zswap_memcg_offline_cleanup().
	 *
	 * Since the offline cleaner is called only once, we cannot leave an
	 * offline memcg reference in zswap_next_shrink.
	 * We can rely on the cleaner only if we get online memcg under lock.
	 *
	 * If we get an offline memcg, we cannot determine if the cleaner has
	 * already been called or will be called later. We must put back the
	 * reference before returning from this function. Otherwise, the
	 * offline memcg left in zswap_next_shrink will hold the reference
	 * until the next run of shrink_worker().
	 */
	do {
		/*
		 * Start shrinking from the next memcg after zswap_next_shrink.
		 * When the offline cleaner has already advanced the cursor,
		 * advancing the cursor here overlooks one memcg, but this
		 * should be negligibly rare.
		 *
		 * If we get an online memcg, keep the extra reference in case
		 * the original one obtained by mem_cgroup_iter() is dropped by
		 * zswap_memcg_offline_cleanup() while we are shrinking the
		 * memcg.
		 */
		spin_lock(&zswap_shrink_lock);
		do {
			memcg = mem_cgroup_iter(NULL, zswap_next_shrink, NULL);
			zswap_next_shrink = memcg;
		} while (memcg && !mem_cgroup_tryget_online(memcg));
		spin_unlock(&zswap_shrink_lock);

		if (!memcg) {
			/*
			 * Continue shrinking without incrementing failures if
			 * we found candidate memcgs in the last tree walk.
			 */
			if (!attempts && ++failures == MAX_RECLAIM_RETRIES)
				break;

			attempts = 0;
			goto resched;
		}

		ret = shrink_memcg(memcg);
		/* drop the extra reference */
		mem_cgroup_put(memcg);

		/*
		 * There are no writeback-candidate pages in the memcg.
		 * This is not an issue as long as we can find another memcg
		 * with pages in zswap. Skip this without incrementing attempts
		 * and failures.
		 */
		if (ret == -ENOENT)
			continue;
		++attempts;

		if (ret && ++failures == MAX_RECLAIM_RETRIES)
			break;
resched:
		cond_resched();
	} while (zswap_total_pages() > thr);
}

/*********************************
* main API
**********************************/

static bool zswap_store_page(struct page *page,
			     struct obj_cgroup *objcg,
			     struct zswap_pool *pool)
{
	swp_entry_t page_swpentry = page_swap_entry(page);
	struct zswap_entry *entry, *old;

	/* allocate entry */
	entry = zswap_entry_cache_alloc(GFP_KERNEL, page_to_nid(page));
	if (!entry) {
		zswap_reject_kmemcache_fail++;
		return false;
	}

	if (!zswap_compress(page, entry, pool))
		goto compress_failed;

	old = xa_store(swap_zswap_tree(page_swpentry),
		       swp_offset(page_swpentry),
		       entry, GFP_KERNEL);
	if (xa_is_err(old)) {
		int err = xa_err(old);

		WARN_ONCE(err != -ENOMEM, "unexpected xarray error: %d\n", err);
		zswap_reject_alloc_fail++;
		goto store_failed;
	}

	/*
	 * We may have had an existing entry that became stale when
	 * the folio was redirtied and now the new version is being
	 * swapped out. Get rid of the old.
	 */
	if (old)
		zswap_entry_free(old);

	/*
	 * The entry is successfully compressed and stored in the tree, there is
	 * no further possibility of failure. Grab refs to the pool and objcg,
	 * charge zswap memory, and increment zswap_stored_pages.
	 * The opposite actions will be performed by zswap_entry_free()
	 * when the entry is removed from the tree.
	 */
	zswap_pool_get(pool);
	if (objcg) {
		obj_cgroup_get(objcg);
		obj_cgroup_charge_zswap(objcg, entry->length);
	}
	atomic_long_inc(&zswap_stored_pages);
	if (entry->length == PAGE_SIZE)
		atomic_long_inc(&zswap_stored_incompressible_pages);

	/*
	 * We finish initializing the entry while it's already in xarray.
	 * This is safe because:
	 *
	 * 1. Concurrent stores and invalidations are excluded by folio lock.
	 *
	 * 2. Writeback is excluded by the entry not being on the LRU yet.
	 *    The publishing order matters to prevent writeback from seeing
	 *    an incoherent entry.
	 */
	entry->pool = pool;
	entry->swpentry = page_swpentry;
	entry->objcg = objcg;
	entry->referenced = true;
	if (entry->length) {
		INIT_LIST_HEAD(&entry->lru);
		zswap_lru_add(&zswap_list_lru, entry);
	}

	return true;

store_failed:
	zs_free(pool->zs_pool, entry->handle);
compress_failed:
	zswap_entry_cache_free(entry);
	return false;
}

bool zswap_store(struct folio *folio)
{
	long nr_pages = folio_nr_pages(folio);
	swp_entry_t swp = folio->swap;
	struct obj_cgroup *objcg = NULL;
	struct mem_cgroup *memcg = NULL;
	struct zswap_pool *pool;
	bool ret = false;
	long index;

	VM_WARN_ON_ONCE(!folio_test_locked(folio));
	VM_WARN_ON_ONCE(!folio_test_swapcache(folio));

	if (!zswap_enabled)
		goto check_old;

	objcg = get_obj_cgroup_from_folio(folio);
	if (objcg && !obj_cgroup_may_zswap(objcg)) {
		memcg = get_mem_cgroup_from_objcg(objcg);
		if (shrink_memcg(memcg)) {
			mem_cgroup_put(memcg);
			goto put_objcg;
		}
		mem_cgroup_put(memcg);
	}

	if (zswap_check_limits())
		goto put_objcg;

	pool = zswap_pool_current_get();
	if (!pool)
		goto put_objcg;

	if (objcg) {
		memcg = get_mem_cgroup_from_objcg(objcg);
		if (memcg_list_lru_alloc(memcg, &zswap_list_lru, GFP_KERNEL)) {
			mem_cgroup_put(memcg);
			goto put_pool;
		}
		mem_cgroup_put(memcg);
	}

	for (index = 0; index < nr_pages; ++index) {
		struct page *page = folio_page(folio, index);

		if (!zswap_store_page(page, objcg, pool))
			goto put_pool;
	}

	if (objcg)
		count_objcg_events(objcg, ZSWPOUT, nr_pages);

	count_vm_events(ZSWPOUT, nr_pages);

	ret = true;

put_pool:
	zswap_pool_put(pool);
put_objcg:
	obj_cgroup_put(objcg);
	if (!ret && zswap_pool_reached_full)
		queue_work(shrink_wq, &zswap_shrink_work);
check_old:
	/*
	 * If the zswap store fails or zswap is disabled, we must invalidate
	 * the possibly stale entries which were previously stored at the
	 * offsets corresponding to each page of the folio. Otherwise,
	 * writeback could overwrite the new data in the swapfile.
	 */
	if (!ret) {
		unsigned type = swp_type(swp);
		pgoff_t offset = swp_offset(swp);
		struct zswap_entry *entry;
		struct xarray *tree;

		for (index = 0; index < nr_pages; ++index) {
			tree = swap_zswap_tree(swp_entry(type, offset + index));
			entry = xa_erase(tree, offset + index);
			if (entry)
				zswap_entry_free(entry);
		}
	}

	return ret;
}

/**
 * zswap_load() - load a folio from zswap
 * @folio: folio to load
 *
 * Return: 0 on success, with the folio unlocked and marked up-to-date, or one
 * of the following error codes:
 *
 *  -EIO: if the swapped out content was in zswap, but could not be loaded
 *  into the page due to a decompression failure. The folio is unlocked, but
 *  NOT marked up-to-date, so that an IO error is emitted (e.g. do_swap_page()
 *  will SIGBUS).
 *
 *  -EINVAL: if the swapped out content was in zswap, but the page belongs
 *  to a large folio, which is not supported by zswap. The folio is unlocked,
 *  but NOT marked up-to-date, so that an IO error is emitted (e.g.
 *  do_swap_page() will SIGBUS).
 *
 *  -ENOENT: if the swapped out content was not in zswap. The folio remains
 *  locked on return.
 */
int zswap_load(struct folio *folio)
{
	swp_entry_t swp = folio->swap;
	pgoff_t offset = swp_offset(swp);
	bool swapcache = folio_test_swapcache(folio);
	struct xarray *tree = swap_zswap_tree(swp);
	struct zswap_entry *entry;

	VM_WARN_ON_ONCE(!folio_test_locked(folio));

	if (zswap_never_enabled())
		return -ENOENT;

	/*
	 * Large folios should not be swapped in while zswap is being used, as
	 * they are not properly handled. Zswap does not properly load large
	 * folios, and a large folio may only be partially in zswap.
	 */
	if (WARN_ON_ONCE(folio_test_large(folio))) {
		folio_unlock(folio);
		return -EINVAL;
	}

	entry = xa_load(tree, offset);
	if (!entry)
		return -ENOENT;

	if (!zswap_decompress(entry, folio)) {
		folio_unlock(folio);
		return -EIO;
	}

	folio_mark_uptodate(folio);

	count_vm_event(ZSWPIN);
	if (entry->objcg)
		count_objcg_events(entry->objcg, ZSWPIN, 1);

	/*
	 * When reading into the swapcache, invalidate our entry. The
	 * swapcache can be the authoritative owner of the page and
	 * its mappings, and the pressure that results from having two
	 * in-memory copies outweighs any benefits of caching the
	 * compression work.
	 *
	 * (Most swapins go through the swapcache. The notable
	 * exception is the singleton fault on SWP_SYNCHRONOUS_IO
	 * files, which reads into a private page and may free it if
	 * the fault fails. We remain the primary owner of the entry.)
	 */
	if (swapcache) {
		folio_mark_dirty(folio);
		xa_erase(tree, offset);
		zswap_entry_free(entry);
	}

	folio_unlock(folio);
	return 0;
}

void zswap_invalidate(swp_entry_t swp)
{
	pgoff_t offset = swp_offset(swp);
	struct xarray *tree = swap_zswap_tree(swp);
	struct zswap_entry *entry;

	if (xa_empty(tree))
		return;

	entry = xa_erase(tree, offset);
	if (entry)
		zswap_entry_free(entry);
}

int zswap_swapon(int type, unsigned long nr_pages)
{
	struct xarray *trees, *tree;
	unsigned int nr, i;

	nr = DIV_ROUND_UP(nr_pages, ZSWAP_ADDRESS_SPACE_PAGES);
	trees = kvcalloc(nr, sizeof(*tree), GFP_KERNEL);
	if (!trees) {
		pr_err("alloc failed, zswap disabled for swap type %d\n", type);
		return -ENOMEM;
	}

	for (i = 0; i < nr; i++)
		xa_init(trees + i);

	nr_zswap_trees[type] = nr;
	zswap_trees[type] = trees;
	return 0;
}

void zswap_swapoff(int type)
{
	struct xarray *trees = zswap_trees[type];
	unsigned int i;

	if (!trees)
		return;

	/* try_to_unuse() invalidated all the entries already */
	for (i = 0; i < nr_zswap_trees[type]; i++)
		WARN_ON_ONCE(!xa_empty(trees + i));

	kvfree(trees);
	nr_zswap_trees[type] = 0;
	zswap_trees[type] = NULL;
}

/*********************************
* debugfs functions
**********************************/
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>

static struct dentry *zswap_debugfs_root;

static int debugfs_get_total_size(void *data, u64 *val)
{
	*val = zswap_total_pages() * PAGE_SIZE;
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(total_size_fops, debugfs_get_total_size, NULL, "%llu\n");

static int debugfs_get_stored_pages(void *data, u64 *val)
{
	*val = atomic_long_read(&zswap_stored_pages);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(stored_pages_fops, debugfs_get_stored_pages, NULL, "%llu\n");

static int debugfs_get_stored_incompressible_pages(void *data, u64 *val)
{
	*val = atomic_long_read(&zswap_stored_incompressible_pages);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(stored_incompressible_pages_fops,
		debugfs_get_stored_incompressible_pages, NULL, "%llu\n");

static int zswap_debugfs_init(void)
{
	if (!debugfs_initialized())
		return -ENODEV;

	zswap_debugfs_root = debugfs_create_dir("zswap", NULL);

	debugfs_create_u64("pool_limit_hit", 0444,
			   zswap_debugfs_root, &zswap_pool_limit_hit);
	debugfs_create_u64("reject_reclaim_fail", 0444,
			   zswap_debugfs_root, &zswap_reject_reclaim_fail);
	debugfs_create_u64("reject_alloc_fail", 0444,
			   zswap_debugfs_root, &zswap_reject_alloc_fail);
	debugfs_create_u64("reject_kmemcache_fail", 0444,
			   zswap_debugfs_root, &zswap_reject_kmemcache_fail);
	debugfs_create_u64("reject_compress_fail", 0444,
			   zswap_debugfs_root, &zswap_reject_compress_fail);
	debugfs_create_u64("reject_compress_poor", 0444,
			   zswap_debugfs_root, &zswap_reject_compress_poor);
	debugfs_create_u64("decompress_fail", 0444,
			   zswap_debugfs_root, &zswap_decompress_fail);
	debugfs_create_u64("written_back_pages", 0444,
			   zswap_debugfs_root, &zswap_written_back_pages);
	debugfs_create_file("pool_total_size", 0444,
			    zswap_debugfs_root, NULL, &total_size_fops);
	debugfs_create_file("stored_pages", 0444,
			    zswap_debugfs_root, NULL, &stored_pages_fops);
	debugfs_create_file("stored_incompressible_pages", 0444,
			    zswap_debugfs_root, NULL,
			    &stored_incompressible_pages_fops);

	return 0;
}
#else
static int zswap_debugfs_init(void)
{
	return 0;
}
#endif

/*********************************
* module init and exit
**********************************/
static int zswap_setup(void)
{
	struct zswap_pool *pool;
	int ret;

	zswap_entry_cache = KMEM_CACHE(zswap_entry, 0);
	if (!zswap_entry_cache) {
		pr_err("entry cache creation failed\n");
		goto cache_fail;
	}

	ret = cpuhp_setup_state_multi(CPUHP_MM_ZSWP_POOL_PREPARE,
				      "mm/zswap_pool:prepare",
				      zswap_cpu_comp_prepare,
				      zswap_cpu_comp_dead);
	if (ret)
		goto hp_fail;

	shrink_wq = alloc_workqueue("zswap-shrink",
			WQ_UNBOUND|WQ_MEM_RECLAIM, 1);
	if (!shrink_wq)
		goto shrink_wq_fail;

	zswap_shrinker = zswap_alloc_shrinker();
	if (!zswap_shrinker)
		goto shrinker_fail;
	if (list_lru_init_memcg(&zswap_list_lru, zswap_shrinker))
		goto lru_fail;
	shrinker_register(zswap_shrinker);

	INIT_WORK(&zswap_shrink_work, shrink_worker);

	pool = __zswap_pool_create_fallback();
	if (pool) {
		pr_info("loaded using pool %s\n", pool->tfm_name);
		list_add(&pool->list, &zswap_pools);
		zswap_has_pool = true;
		static_branch_enable(&zswap_ever_enabled);
	} else {
		pr_err("pool creation failed\n");
		zswap_enabled = false;
	}

	if (zswap_debugfs_init())
		pr_warn("debugfs initialization failed\n");
	zswap_init_state = ZSWAP_INIT_SUCCEED;
	return 0;

lru_fail:
	shrinker_free(zswap_shrinker);
shrinker_fail:
	destroy_workqueue(shrink_wq);
shrink_wq_fail:
	cpuhp_remove_multi_state(CPUHP_MM_ZSWP_POOL_PREPARE);
hp_fail:
	kmem_cache_destroy(zswap_entry_cache);
cache_fail:
	/* if built-in, we aren't unloaded on failure; don't allow use */
	zswap_init_state = ZSWAP_INIT_FAILED;
	zswap_enabled = false;
	return -ENOMEM;
}

static int __init zswap_init(void)
{
	if (!zswap_enabled)
		return 0;
	return zswap_setup();
}
/* must be late so crypto has time to come up */
late_initcall(zswap_init);

MODULE_AUTHOR("Seth Jennings <sjennings@variantweb.net>");
MODULE_DESCRIPTION("Compressed cache for swap pages");
