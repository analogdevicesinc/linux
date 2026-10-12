/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Common infrastructure for supporing dma-buf in the I/O path.
 *
 * Copyright (C) 2026 Pavel Begunkov <asml.silence@gmail.com>
 */
#include <linux/dma-buf-io.h>
#include <linux/dma-resv.h>

static void dma_buf_io_map_release(struct dma_buf_io_map *map)
{
	struct dma_buf_io_ctx *ctx = map->ctx;

	dma_resv_assert_held(ctx->dmabuf->resv);

	percpu_ref_exit(&map->refs);
	ctx->dev_ops->unmap(ctx, map);
}

static void dma_buf_io_map_refs_cb(struct percpu_ref *ref)
{
	struct dma_buf_io_map *map = container_of(ref, struct dma_buf_io_map, refs);

	complete(&map->drained);
}

static void dma_buf_io_kill_map(struct dma_buf_io_ctx *ctx)
{
	struct dma_buf_io_map *map;

	dma_resv_assert_held(ctx->dmabuf->resv);

	map = rcu_dereference_protected(ctx->map,
				dma_resv_held(ctx->dmabuf->resv));
	if (!map)
		return;

	rcu_assign_pointer(ctx->map, NULL);
	percpu_ref_kill(&map->refs);
	/* make sure the map is not visible via ctx->map */
	synchronize_rcu_expedited();
	wait_for_completion(&map->drained);
	dma_buf_io_map_release(map);
}

int dma_buf_io_init_map(struct dma_buf_io_ctx *ctx, struct dma_buf_io_map *map,
			struct sg_table *sgt)
{
	unsigned seg_shift = ~0U;
	struct scatterlist *sg;
	unsigned long tmp;
	int ret;

	for_each_sgtable_dma_sg(sgt, sg, tmp)
		seg_shift = min(seg_shift, __ffs(sg_dma_len(sg)));

	ret = percpu_ref_init(&map->refs, dma_buf_io_map_refs_cb, 0, GFP_KERNEL);
	if (ret)
		return ret;
	init_completion(&map->drained);
	map->min_seg_shift = seg_shift;
	map->ctx = ctx;
	return 0;
}
EXPORT_SYMBOL_NS_GPL(dma_buf_io_init_map, "DMA_BUF");

static struct dma_buf_io_map *__dma_buf_io_create_map(struct dma_buf_io_ctx *ctx)
{
	struct dma_buf *dmabuf = ctx->dmabuf;
	struct dma_buf_io_map *map;
	long ret;

	dma_resv_assert_held(dmabuf->resv);

	if (ctx->is_dead)
		return ERR_PTR(-ENOENT);

	map = __dma_buf_io_get_map(ctx);
	if (map)
		return map;

	map = ctx->dev_ops->map(ctx);
	if (IS_ERR(map))
		return map;

	if (WARN_ON_ONCE(!map->min_seg_shift))
		return ERR_PTR(-EFAULT);

	ret = dma_resv_wait_timeout(dmabuf->resv, DMA_RESV_USAGE_KERNEL,
				    true, MAX_SCHEDULE_TIMEOUT);
	if (ret <= 0) {
		if (!ret)
			ret = -EAGAIN;
		dma_buf_io_map_release(map);
		return ERR_PTR(ret);
	}

	/* get a reference for the caller */
	percpu_ref_get(&map->refs);
	rcu_assign_pointer(ctx->map, map);
	return map;
}

struct dma_buf_io_map *dma_buf_io_create_map(struct dma_buf_io_ctx *ctx)
{
	struct dma_buf_io_map *map;
	int ret;

	ret = dma_resv_lock_interruptible(ctx->dmabuf->resv, NULL);
	if (ret)
		return ERR_PTR(ret);

	map = __dma_buf_io_create_map(ctx);
	dma_resv_unlock(ctx->dmabuf->resv);
	return map;
}

static void dma_buf_io_ctx_shutdown(struct dma_buf_io_ctx *ctx)
{
	dma_resv_lock(ctx->dmabuf->resv, NULL);
	ctx->is_dead = true;
	dma_buf_io_kill_map(ctx);
	dma_resv_unlock(ctx->dmabuf->resv);
}

int dma_buf_io_ctx_create(struct file *file,
			   struct dma_buf *dmabuf,
			   enum dma_data_direction dir,
			   struct dma_buf_io_ctx **out_ctx)
{
	struct dma_buf_io_ctx *ctx;
	int ret;

	if (!file->f_op->init_dma_buf_io_ctx)
		return -EOPNOTSUPP;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->dir = dir;
	ctx->dmabuf = dmabuf;
	get_dma_buf(dmabuf);

	ret = file->f_op->init_dma_buf_io_ctx(file, ctx);
	if (ret) {
		kfree(ctx);
		dma_buf_put(dmabuf);
		return ret;
	}

	if (WARN_ON_ONCE(!ctx->dev_ops ||
			 !ctx->dev_ops->map ||
			 !ctx->dev_ops->unmap ||
			 !ctx->dev_ops->release))
		return -EINVAL;

	*out_ctx = ctx;
	return 0;
}

void dma_buf_io_ctx_release(struct dma_buf_io_ctx *ctx)
{
	dma_buf_io_ctx_shutdown(ctx);

	if (WARN_ON_ONCE(rcu_dereference_protected(ctx->map, true)))
		return;

	ctx->dev_ops->release(ctx);
	dma_buf_put(ctx->dmabuf);
	kfree(ctx);
}

void dma_buf_io_invalidate_mappings(struct dma_buf_io_ctx *ctx)
{
	dma_buf_io_kill_map(ctx);
}
EXPORT_SYMBOL_NS_GPL(dma_buf_io_invalidate_mappings, "DMA_BUF");

void dma_buf_io_detach(struct dma_buf_io_ctx *ctx)
{
	dma_buf_io_ctx_shutdown(ctx);
}
EXPORT_SYMBOL_NS_GPL(dma_buf_io_detach, "DMA_BUF");
