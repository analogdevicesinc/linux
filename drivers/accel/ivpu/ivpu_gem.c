// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2020-2023 Intel Corporation
 */

#include <linux/dma-buf.h>
#include <linux/highmem.h>
#include <linux/module.h>
#include <linux/set_memory.h>
#include <linux/xarray.h>

#include <drm/drm_cache.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>
#include <drm/drm_utils.h>

#include "ivpu_drv.h"
#include "ivpu_fw.h"
#include "ivpu_gem.h"
#include "ivpu_hw.h"
#include "ivpu_mmu.h"
#include "ivpu_mmu_context.h"

MODULE_IMPORT_NS("DMA_BUF");

static const struct drm_gem_object_funcs ivpu_gem_funcs;

static inline void ivpu_dbg_bo(struct ivpu_device *vdev, struct ivpu_bo *bo, const char *action)
{
	ivpu_dbg(vdev, BO,
		 "%6s: bo %8p size %9zu ctx %d vpu_addr %9llx pages %d sgt %d mmu_mapped %d wc %d imported %d\n",
		 action, bo, ivpu_bo_size(bo), bo->ctx_id, bo->vpu_addr,
		 (bool)bo->base.pages, (bool)bo->base.sgt, bo->mmu_mapped, bo->base.map_wc,
		 (bool)drm_gem_is_imported(&bo->base.base));
}

static inline int ivpu_bo_lock(struct ivpu_bo *bo)
{
	return dma_resv_lock(bo->base.base.resv, NULL);
}

static inline void ivpu_bo_unlock(struct ivpu_bo *bo)
{
	dma_resv_unlock(bo->base.base.resv);
}

static struct sg_table *ivpu_bo_map_attachment(struct ivpu_device *vdev, struct ivpu_bo *bo)
{
	struct sg_table *sgt;

	drm_WARN_ON(&vdev->drm, !bo->base.base.import_attach);

	ivpu_bo_lock(bo);

	sgt = bo->base.sgt;
	if (!sgt) {
		sgt = dma_buf_map_attachment(bo->base.base.import_attach, DMA_BIDIRECTIONAL);
		if (IS_ERR(sgt))
			ivpu_err(vdev, "Failed to map BO in IOMMU: %ld\n", PTR_ERR(sgt));
		else
			bo->base.sgt = sgt;
	}

	ivpu_bo_unlock(bo);

	return sgt;
}

/*
 * ivpu_bo_bind() - pin the backing physical pages and map them to VPU.
 *
 * This function pins physical memory pages, then maps the physical pages
 * to IOMMU address space and finally updates the VPU MMU page tables
 * to allow the VPU to translate VPU address to IOMMU address.
 */
int __must_check ivpu_bo_bind(struct ivpu_bo *bo)
{
	struct ivpu_device *vdev = ivpu_bo_to_vdev(bo);
	struct sg_table *sgt;
	int ret = 0;

	ivpu_dbg_bo(vdev, bo, "bind");

	if (bo->base.base.import_attach)
		sgt = ivpu_bo_map_attachment(vdev, bo);
	else
		sgt = drm_gem_shmem_get_pages_sgt(&bo->base);
	if (IS_ERR(sgt)) {
		ret = PTR_ERR(sgt);
		ivpu_err(vdev, "Failed to map BO in IOMMU: %d\n", ret);
		return ret;
	}

	ivpu_bo_lock(bo);

	if (!bo->mmu_mapped) {
		drm_WARN_ON(&vdev->drm, !bo->ctx);
		ret = ivpu_mmu_context_map_sgt(vdev, bo->ctx, bo->vpu_addr, sgt,
					       ivpu_bo_is_snooped(bo));
		if (ret) {
			ivpu_err(vdev, "Failed to map BO in MMU: %d\n", ret);
			goto unlock;
		}
		bo->mmu_mapped = true;
	}

unlock:
	ivpu_bo_unlock(bo);

	return ret;
}

static int
ivpu_bo_alloc_vpu_addr(struct ivpu_bo *bo, struct ivpu_mmu_context *ctx,
		       const struct ivpu_addr_range *range)
{
	struct ivpu_device *vdev = ivpu_bo_to_vdev(bo);
	int idx, ret;

	if (!drm_dev_enter(&vdev->drm, &idx))
		return -ENODEV;

	ivpu_bo_lock(bo);

	ret = ivpu_mmu_context_insert_node(ctx, range, ivpu_bo_size(bo), &bo->mm_node);
	if (!ret) {
		bo->ctx = ctx;
		bo->ctx_id = ctx->id;
		bo->vpu_addr = bo->mm_node.start;
		ivpu_dbg_bo(vdev, bo, "vaddr");
	} else {
		ivpu_err(vdev, "Failed to add BO to context %u: %d\n", ctx->id, ret);
	}

	ivpu_bo_unlock(bo);

	drm_dev_exit(idx);

	return ret;
}

static void ivpu_bo_unbind_locked(struct ivpu_bo *bo)
{
	struct ivpu_device *vdev = ivpu_bo_to_vdev(bo);

	dma_resv_assert_held(bo->base.base.resv);

	if (bo->mmu_mapped) {
		drm_WARN_ON(&vdev->drm, !bo->ctx);
		drm_WARN_ON(&vdev->drm, !bo->vpu_addr);
		drm_WARN_ON(&vdev->drm, !bo->base.sgt);
		ivpu_mmu_context_unmap_sgt(vdev, bo->ctx, bo->vpu_addr, bo->base.sgt);
		bo->mmu_mapped = false;
	}

	if (bo->ctx) {
		ivpu_mmu_context_remove_node(bo->ctx, &bo->mm_node);
		bo->ctx = NULL;
	}

	if (drm_gem_is_imported(&bo->base.base))
		return;

	if (bo->base.sgt) {
		if (bo->base.base.import_attach) {
			dma_buf_unmap_attachment(bo->base.base.import_attach,
						 bo->base.sgt, DMA_BIDIRECTIONAL);
		} else {
			dma_unmap_sgtable(vdev->drm.dev, bo->base.sgt, DMA_BIDIRECTIONAL, 0);
			sg_free_table(bo->base.sgt);
			kfree(bo->base.sgt);
		}
		bo->base.sgt = NULL;
	}
}

void ivpu_bo_unbind_all_bos_from_context(struct ivpu_device *vdev, struct ivpu_mmu_context *ctx)
{
	struct ivpu_bo *bo;

	if (drm_WARN_ON(&vdev->drm, !ctx))
		return;

	mutex_lock(&vdev->bo_list_lock);
	list_for_each_entry(bo, &vdev->bo_list, bo_list_node) {
		ivpu_bo_lock(bo);
		if (bo->ctx == ctx) {
			ivpu_dbg_bo(vdev, bo, "unbind");
			ivpu_bo_unbind_locked(bo);
		}
		ivpu_bo_unlock(bo);
	}
	mutex_unlock(&vdev->bo_list_lock);
}

struct drm_gem_object *ivpu_gem_create_object(struct drm_device *dev, size_t size)
{
	struct ivpu_bo *bo;

	if (size == 0 || !PAGE_ALIGNED(size))
		return ERR_PTR(-EINVAL);

	bo = kzalloc(sizeof(*bo), GFP_KERNEL);
	if (!bo)
		return ERR_PTR(-ENOMEM);

	bo->base.base.funcs = &ivpu_gem_funcs;
	bo->base.pages_mark_dirty_on_put = true; /* VPU can dirty a BO anytime */

	INIT_LIST_HEAD(&bo->bo_list_node);

	return &bo->base.base;
}

struct drm_gem_object *ivpu_gem_prime_import(struct drm_device *dev,
					     struct dma_buf *dma_buf)
{
	struct ivpu_device *vdev = to_ivpu_device(dev);
	struct device *attach_dev = dev->dev;
	struct dma_buf_attachment *attach;
	struct drm_gem_object *obj;
	struct ivpu_bo *bo;
	int ret;

	attach = dma_buf_attach(dma_buf, attach_dev);
	if (IS_ERR(attach))
		return ERR_CAST(attach);

	get_dma_buf(dma_buf);

	obj = drm_gem_shmem_prime_import_sg_table(dev, attach, NULL);
	if (IS_ERR(obj)) {
		ret = PTR_ERR(obj);
		goto fail_detach;
	}

	obj->import_attach = attach;
	obj->resv = dma_buf->resv;

	bo = to_ivpu_bo(obj);

	mutex_lock(&vdev->bo_list_lock);
	list_add_tail(&bo->bo_list_node, &vdev->bo_list);
	mutex_unlock(&vdev->bo_list_lock);

	ivpu_dbg(vdev, BO, "import: bo %8p size %9zu\n", bo, ivpu_bo_size(bo));

	return obj;

fail_detach:
	dma_buf_detach(dma_buf, attach);
	dma_buf_put(dma_buf);

	return ERR_PTR(ret);
}

static struct ivpu_bo *ivpu_bo_alloc(struct ivpu_device *vdev, u64 size, u32 flags)
{
	struct drm_gem_shmem_object *shmem;
	struct ivpu_bo *bo;

	switch (flags & DRM_IVPU_BO_CACHE_MASK) {
	case DRM_IVPU_BO_CACHED:
	case DRM_IVPU_BO_WC:
		break;
	default:
		return ERR_PTR(-EINVAL);
	}

	shmem = drm_gem_shmem_create(&vdev->drm, size);
	if (IS_ERR(shmem))
		return ERR_CAST(shmem);

	bo = to_ivpu_bo(&shmem->base);
	bo->base.map_wc = flags & DRM_IVPU_BO_WC;
	bo->flags = flags;

	mutex_lock(&vdev->bo_list_lock);
	list_add_tail(&bo->bo_list_node, &vdev->bo_list);
	mutex_unlock(&vdev->bo_list_lock);

	ivpu_dbg(vdev, BO, " alloc: bo %8p size %9llu\n", bo, size);

	return bo;
}

static int ivpu_gem_bo_open(struct drm_gem_object *obj, struct drm_file *file)
{
	struct ivpu_file_priv *file_priv = file->driver_priv;
	struct ivpu_device *vdev = file_priv->vdev;
	struct ivpu_bo *bo = to_ivpu_bo(obj);
	struct ivpu_addr_range *range;

	if (bo->ctx) {
		ivpu_warn(vdev, "Can't add BO to ctx %u: already in ctx %u\n",
			  file_priv->ctx.id, bo->ctx->id);
		return -EALREADY;
	}

	if (bo->flags & DRM_IVPU_BO_SHAVE_MEM)
		range = &vdev->hw->ranges.shave;
	else if (bo->flags & DRM_IVPU_BO_DMA_MEM)
		range = &vdev->hw->ranges.dma;
	else
		range = &vdev->hw->ranges.user;

	return ivpu_bo_alloc_vpu_addr(bo, &file_priv->ctx, range);
}

static void ivpu_gem_bo_free(struct drm_gem_object *obj)
{
	struct ivpu_device *vdev = to_ivpu_device(obj->dev);
	struct ivpu_bo *bo = to_ivpu_bo(obj);

	ivpu_dbg_bo(vdev, bo, "free");

	drm_WARN_ON(&vdev->drm, list_empty(&bo->bo_list_node));

	mutex_lock(&vdev->bo_list_lock);
	list_del(&bo->bo_list_node);
	mutex_unlock(&vdev->bo_list_lock);

	drm_WARN_ON(&vdev->drm, !drm_gem_is_imported(&bo->base.base) &&
		    !dma_resv_test_signaled(obj->resv, DMA_RESV_USAGE_READ));
	drm_WARN_ON(&vdev->drm, ivpu_bo_size(bo) == 0);
	drm_WARN_ON(&vdev->drm, bo->base.vaddr);

	ivpu_bo_lock(bo);
	ivpu_bo_unbind_locked(bo);
	ivpu_bo_unlock(bo);

	drm_WARN_ON(&vdev->drm, bo->mmu_mapped);
	drm_WARN_ON(&vdev->drm, bo->ctx);

	drm_WARN_ON(obj->dev, refcount_read(&bo->base.pages_use_count) > 1);
	drm_WARN_ON(obj->dev, bo->base.base.vma_node.vm_files.rb_node);
	drm_gem_shmem_free(&bo->base);
}

static const struct drm_gem_object_funcs ivpu_gem_funcs = {
	.free = ivpu_gem_bo_free,
	.open = ivpu_gem_bo_open,
	.print_info = drm_gem_shmem_object_print_info,
	.pin = drm_gem_shmem_object_pin,
	.unpin = drm_gem_shmem_object_unpin,
	.get_sg_table = drm_gem_shmem_object_get_sg_table,
	.vmap = drm_gem_shmem_object_vmap,
	.vunmap = drm_gem_shmem_object_vunmap,
	.mmap = drm_gem_shmem_object_mmap,
	.vm_ops = &drm_gem_shmem_vm_ops,
};

int ivpu_bo_create_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct ivpu_file_priv *file_priv = file->driver_priv;
	struct ivpu_device *vdev = file_priv->vdev;
	struct drm_ivpu_bo_create *args = data;
	u64 size = PAGE_ALIGN(args->size);
	struct ivpu_bo *bo;
	int ret;

	if (args->flags & ~DRM_IVPU_BO_FLAGS)
		return -EINVAL;

	if (size == 0)
		return -EINVAL;

	bo = ivpu_bo_alloc(vdev, size, args->flags);
	if (IS_ERR(bo)) {
		ivpu_err(vdev, "Failed to allocate BO: %pe (ctx %u size %llu flags 0x%x)",
			 bo, file_priv->ctx.id, args->size, args->flags);
		return PTR_ERR(bo);
	}

	drm_WARN_ON(&vdev->drm, bo->base.base.handle_count != 0);

	ret = drm_gem_handle_create(file, &bo->base.base, &args->handle);
	if (ret) {
		ivpu_err(vdev, "Failed to create handle for BO: %pe (ctx %u size %llu flags 0x%x)",
			 bo, file_priv->ctx.id, args->size, args->flags);
	} else {
		args->vpu_addr = bo->vpu_addr;
		drm_WARN_ON(&vdev->drm, bo->base.base.handle_count != 1);
	}

	drm_gem_object_put(&bo->base.base);

	return ret;
}

struct ivpu_bo *
ivpu_bo_create(struct ivpu_device *vdev, struct ivpu_mmu_context *ctx,
	       struct ivpu_addr_range *range, u64 size, u32 flags)
{
	struct iosys_map map;
	struct ivpu_bo *bo;
	int ret;

	if (drm_WARN_ON(&vdev->drm, !range))
		return NULL;

	drm_WARN_ON(&vdev->drm, !PAGE_ALIGNED(range->start));
	drm_WARN_ON(&vdev->drm, !PAGE_ALIGNED(range->end));
	drm_WARN_ON(&vdev->drm, !PAGE_ALIGNED(size));

	bo = ivpu_bo_alloc(vdev, size, flags);
	if (IS_ERR(bo)) {
		ivpu_err(vdev, "Failed to allocate BO: %pe (vpu_addr 0x%llx size %llu flags 0x%x)",
			 bo, range->start, size, flags);
		return NULL;
	}

	ret = ivpu_bo_alloc_vpu_addr(bo, ctx, range);
	if (ret)
		goto err_put;

	ret = ivpu_bo_bind(bo);
	if (ret)
		goto err_put;

	if (flags & DRM_IVPU_BO_MAPPABLE) {
		ivpu_bo_lock(bo);
		ret = drm_gem_shmem_vmap_locked(&bo->base, &map);
		ivpu_bo_unlock(bo);

		if (ret)
			goto err_put;
	}

	return bo;

err_put:
	drm_gem_object_put(&bo->base.base);
	return NULL;
}

struct ivpu_bo *ivpu_bo_create_runtime(struct ivpu_device *vdev, u64 addr, u64 size, u32 flags)
{
	struct ivpu_addr_range range;

	if (!ivpu_is_within_range(addr, size, &vdev->hw->ranges.runtime)) {
		ivpu_err(vdev, "Invalid runtime BO address 0x%llx size %llu\n", addr, size);
		return NULL;
	}

	if (ivpu_hw_range_init(vdev, &range, addr, size))
		return NULL;

	return ivpu_bo_create(vdev, &vdev->gctx, &range, size, flags);
}

struct ivpu_bo *ivpu_bo_create_global(struct ivpu_device *vdev, u64 size, u32 flags)
{
	return ivpu_bo_create(vdev, &vdev->gctx, &vdev->hw->ranges.global, size, flags);
}

void ivpu_bo_free(struct ivpu_bo *bo)
{
	struct iosys_map map = IOSYS_MAP_INIT_VADDR(bo->base.vaddr);

	if (bo->flags & DRM_IVPU_BO_MAPPABLE) {
		ivpu_bo_lock(bo);
		drm_gem_shmem_vunmap_locked(&bo->base, &map);
		ivpu_bo_unlock(bo);
	}

	drm_gem_object_put(&bo->base.base);
}

int ivpu_bo_info_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_ivpu_bo_info *args = data;
	struct drm_gem_object *obj;
	struct ivpu_bo *bo;
	int ret = 0;

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj)
		return -ENOENT;

	bo = to_ivpu_bo(obj);

	ivpu_bo_lock(bo);
	args->flags = bo->flags;
	args->mmap_offset = drm_vma_node_offset_addr(&obj->vma_node);
	args->vpu_addr = bo->vpu_addr;
	args->size = obj->size;
	ivpu_bo_unlock(bo);

	drm_gem_object_put(obj);
	return ret;
}

int ivpu_bo_wait_ioctl(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_ivpu_bo_wait *args = data;
	struct drm_gem_object *obj;
	unsigned long timeout;
	long ret;

	timeout = drm_timeout_abs_to_jiffies(args->timeout_ns);

	/* Add 1 jiffy to ensure the wait function never times out before intended timeout_ns */
	timeout += 1;

	obj = drm_gem_object_lookup(file, args->handle);
	if (!obj)
		return -EINVAL;

	ret = dma_resv_wait_timeout(obj->resv, DMA_RESV_USAGE_READ, true, timeout);
	if (ret == 0) {
		ret = -ETIMEDOUT;
	} else if (ret > 0) {
		ret = 0;
		args->job_status = to_ivpu_bo(obj)->job_status;
	}

	drm_gem_object_put(obj);

	return ret;
}

static void ivpu_bo_print_info(struct ivpu_bo *bo, struct drm_printer *p)
{
	ivpu_bo_lock(bo);

	drm_printf(p, "%-9p %-3u 0x%-12llx %-10lu 0x%-8x %-4u",
		   bo, bo->ctx_id, bo->vpu_addr, bo->base.base.size,
		   bo->flags, kref_read(&bo->base.base.refcount));

	if (bo->base.pages)
		drm_printf(p, " has_pages");

	if (bo->mmu_mapped)
		drm_printf(p, " mmu_mapped");

	if (drm_gem_is_imported(&bo->base.base))
		drm_printf(p, " imported");

	drm_printf(p, "\n");

	ivpu_bo_unlock(bo);
}

void ivpu_bo_list(struct drm_device *dev, struct drm_printer *p)
{
	struct ivpu_device *vdev = to_ivpu_device(dev);
	struct ivpu_bo *bo;

	drm_printf(p, "%-9s %-3s %-14s %-10s %-10s %-4s %s\n",
		   "bo", "ctx", "vpu_addr", "size", "flags", "refs", "attribs");

	mutex_lock(&vdev->bo_list_lock);
	list_for_each_entry(bo, &vdev->bo_list, bo_list_node)
		ivpu_bo_print_info(bo, p);
	mutex_unlock(&vdev->bo_list_lock);
}

void ivpu_bo_list_print(struct drm_device *dev)
{
	struct drm_printer p = drm_info_printer(dev->dev);

	ivpu_bo_list(dev, &p);
}
