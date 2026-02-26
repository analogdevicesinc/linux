// SPDX-License-Identifier: GPL-2.0  OR Linux-OpenIB
/*
 * Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 */

#include <linux/slab.h>
#include <linux/rbtree.h>
#include <linux/sort.h>
#include <linux/spinlock.h>
#include <rdma/ib_verbs.h>

#include "frmr_pools.h"

static int push_handle_to_queue_locked(struct frmr_queue *queue, u32 handle)
{
	u32 tmp = queue->ci % NUM_HANDLES_PER_PAGE;
	struct frmr_handles_page *page;

	if (queue->ci >= queue->num_pages * NUM_HANDLES_PER_PAGE) {
		page = kzalloc_obj(*page, GFP_ATOMIC);
		if (!page)
			return -ENOMEM;
		queue->num_pages++;
		list_add_tail(&page->list, &queue->pages_list);
	} else {
		page = list_last_entry(&queue->pages_list,
				       struct frmr_handles_page, list);
	}

	page->handles[tmp] = handle;
	queue->ci++;
	return 0;
}

static u32 pop_handle_from_queue_locked(struct frmr_queue *queue)
{
	u32 tmp = (queue->ci - 1) % NUM_HANDLES_PER_PAGE;
	struct frmr_handles_page *page;
	u32 handle;

	page = list_last_entry(&queue->pages_list, struct frmr_handles_page,
			       list);
	handle = page->handles[tmp];
	queue->ci--;

	if (!tmp) {
		list_del(&page->list);
		queue->num_pages--;
		kfree(page);
	}

	return handle;
}

static bool pop_frmr_handles_page(struct ib_frmr_pool *pool,
				  struct frmr_queue *queue,
				  struct frmr_handles_page **page, u32 *count)
{
	spin_lock(&pool->lock);
	if (list_empty(&queue->pages_list)) {
		spin_unlock(&pool->lock);
		return false;
	}

	*page = list_first_entry(&queue->pages_list, struct frmr_handles_page,
				 list);
	list_del(&(*page)->list);
	queue->num_pages--;

	/* If this is the last page, count may be less than
	 * NUM_HANDLES_PER_PAGE.
	 */
	if (queue->ci >= NUM_HANDLES_PER_PAGE)
		*count = NUM_HANDLES_PER_PAGE;
	else
		*count = queue->ci;

	queue->ci -= *count;
	spin_unlock(&pool->lock);
	return true;
}

static void destroy_frmr_pool(struct ib_device *device,
			      struct ib_frmr_pool *pool)
{
	struct ib_frmr_pools *pools = device->frmr_pools;
	struct frmr_handles_page *page;
	u32 count;

	while (pop_frmr_handles_page(pool, &pool->queue, &page, &count)) {
		pools->pool_ops->destroy_frmrs(device, page->handles, count);
		kfree(page);
	}

	kfree(pool);
}

/*
 * Initialize the FRMR pools for a device.
 *
 * @device: The device to initialize the FRMR pools for.
 * @pool_ops: The pool operations to use.
 *
 * Returns 0 on success, negative error code on failure.
 */
int ib_frmr_pools_init(struct ib_device *device,
		       const struct ib_frmr_pool_ops *pool_ops)
{
	struct ib_frmr_pools *pools;

	pools = kzalloc_obj(*pools);
	if (!pools)
		return -ENOMEM;

	pools->rb_root = RB_ROOT;
	rwlock_init(&pools->rb_lock);
	pools->pool_ops = pool_ops;

	device->frmr_pools = pools;
	return 0;
}
EXPORT_SYMBOL(ib_frmr_pools_init);

/*
 * Clean up the FRMR pools for a device.
 *
 * @device: The device to clean up the FRMR pools for.
 *
 * Call cleanup only after all FRMR handles have been pushed back to the pool
 * and no other FRMR operations are allowed to run in parallel.
 * Ensuring this allows us to save synchronization overhead in pop and push
 * operations.
 */
void ib_frmr_pools_cleanup(struct ib_device *device)
{
	struct ib_frmr_pools *pools = device->frmr_pools;
	struct ib_frmr_pool *pool, *next;

	if (!pools)
		return;

	rbtree_postorder_for_each_entry_safe(pool, next, &pools->rb_root, node)
		destroy_frmr_pool(device, pool);

	kfree(pools);
	device->frmr_pools = NULL;
}
EXPORT_SYMBOL(ib_frmr_pools_cleanup);

static inline int compare_keys(struct ib_frmr_key *key1,
			       struct ib_frmr_key *key2)
{
	int res;

	res = cmp_int(key1->ats, key2->ats);
	if (res)
		return res;

	res = cmp_int(key1->access_flags, key2->access_flags);
	if (res)
		return res;

	res = cmp_int(key1->vendor_key, key2->vendor_key);
	if (res)
		return res;

	res = cmp_int(key1->kernel_vendor_key, key2->kernel_vendor_key);
	if (res)
		return res;

	/*
	 * allow using handles that support more DMA blocks, up to twice the
	 * requested number
	 */
	res = cmp_int(key1->num_dma_blocks, key2->num_dma_blocks);
	if (res > 0) {
		if (key1->num_dma_blocks - key2->num_dma_blocks <
		    key2->num_dma_blocks)
			return 0;
	}

	return res;
}

static int frmr_pool_cmp_find(const void *key, const struct rb_node *node)
{
	struct ib_frmr_pool *pool = rb_entry(node, struct ib_frmr_pool, node);

	return compare_keys(&pool->key, (struct ib_frmr_key *)key);
}

static int frmr_pool_cmp_add(struct rb_node *new, const struct rb_node *node)
{
	struct ib_frmr_pool *new_pool =
		rb_entry(new, struct ib_frmr_pool, node);
	struct ib_frmr_pool *pool = rb_entry(node, struct ib_frmr_pool, node);

	return compare_keys(&pool->key, &new_pool->key);
}

static struct ib_frmr_pool *ib_frmr_pool_find(struct ib_frmr_pools *pools,
					      struct ib_frmr_key *key)
{
	struct ib_frmr_pool *pool;
	struct rb_node *node;

	/* find operation is done under read lock for performance reasons.
	 * The case of threads failing to find the same pool and creating it
	 * is handled by the create_frmr_pool function.
	 */
	read_lock(&pools->rb_lock);
	node = rb_find(key, &pools->rb_root, frmr_pool_cmp_find);
	pool = rb_entry_safe(node, struct ib_frmr_pool, node);
	read_unlock(&pools->rb_lock);

	return pool;
}

static struct ib_frmr_pool *create_frmr_pool(struct ib_device *device,
					     struct ib_frmr_key *key)
{
	struct ib_frmr_pools *pools = device->frmr_pools;
	struct ib_frmr_pool *pool;
	struct rb_node *existing;

	pool = kzalloc_obj(*pool);
	if (!pool)
		return ERR_PTR(-ENOMEM);

	memcpy(&pool->key, key, sizeof(*key));
	INIT_LIST_HEAD(&pool->queue.pages_list);
	spin_lock_init(&pool->lock);

	write_lock(&pools->rb_lock);
	existing = rb_find_add(&pool->node, &pools->rb_root, frmr_pool_cmp_add);
	write_unlock(&pools->rb_lock);

	/* If a different thread has already created the pool, return it.
	 * The insert operation is done under the write lock so we are sure
	 * that the pool is not inserted twice.
	 */
	if (existing) {
		kfree(pool);
		return rb_entry(existing, struct ib_frmr_pool, node);
	}

	return pool;
}

static int get_frmr_from_pool(struct ib_device *device,
			      struct ib_frmr_pool *pool, struct ib_mr *mr)
{
	struct ib_frmr_pools *pools = device->frmr_pools;
	u32 handle;
	int err;

	spin_lock(&pool->lock);
	if (pool->queue.ci == 0) {
		spin_unlock(&pool->lock);
		err = pools->pool_ops->create_frmrs(device, &pool->key, &handle,
						    1);
		if (err)
			return err;
	} else {
		handle = pop_handle_from_queue_locked(&pool->queue);
		spin_unlock(&pool->lock);
	}

	mr->frmr.pool = pool;
	mr->frmr.handle = handle;

	return 0;
}

/*
 * Pop an FRMR handle from the pool.
 *
 * @device: The device to pop the FRMR handle from.
 * @mr: The MR to pop the FRMR handle from.
 *
 * Returns 0 on success, negative error code on failure.
 */
int ib_frmr_pool_pop(struct ib_device *device, struct ib_mr *mr)
{
	struct ib_frmr_pools *pools = device->frmr_pools;
	struct ib_frmr_pool *pool;

	WARN_ON_ONCE(!device->frmr_pools);
	pool = ib_frmr_pool_find(pools, &mr->frmr.key);
	if (!pool) {
		pool = create_frmr_pool(device, &mr->frmr.key);
		if (IS_ERR(pool))
			return PTR_ERR(pool);
	}

	return get_frmr_from_pool(device, pool, mr);
}
EXPORT_SYMBOL(ib_frmr_pool_pop);

/*
 * Push an FRMR handle back to the pool.
 *
 * @device: The device to push the FRMR handle to.
 * @mr: The MR containing the FRMR handle to push back to the pool.
 *
 * Returns 0 on success, negative error code on failure.
 */
int ib_frmr_pool_push(struct ib_device *device, struct ib_mr *mr)
{
	struct ib_frmr_pool *pool = mr->frmr.pool;
	int ret;

	spin_lock(&pool->lock);
	ret = push_handle_to_queue_locked(&pool->queue, mr->frmr.handle);
	spin_unlock(&pool->lock);

	return ret;
}
EXPORT_SYMBOL(ib_frmr_pool_push);
