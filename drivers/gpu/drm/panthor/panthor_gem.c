// SPDX-License-Identifier: GPL-2.0 or MIT
/* Copyright 2019 Linaro, Ltd, Rob Herring <robh@kernel.org> */
/* Copyright 2023 Collabora ltd. */
/* Copyright 2025 Amazon.com, Inc. or its affiliates */

#include <linux/cleanup.h>
#include <linux/dma-buf.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>

#include <drm/drm_debugfs.h>
#include <drm/drm_file.h>
#include <drm/drm_prime.h>
#include <drm/drm_print.h>
#include <drm/panthor_drm.h>

#include "panthor_device.h"
#include "panthor_drv.h"
#include "panthor_fw.h"
#include "panthor_gem.h"
#include "panthor_mmu.h"

void panthor_gem_init(struct panthor_device *ptdev)
{
	int err;

	if (IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE) &&
	    !panthor_transparent_hugepage)
		return;

	err = drm_gem_huge_mnt_create(&ptdev->base, "within_size");
	if (drm_gem_get_huge_mnt(&ptdev->base))
		drm_info(&ptdev->base, "Using Transparent Hugepage\n");
	else if (err)
		drm_warn(&ptdev->base, "Can't use Transparent Hugepage (%d)\n",
			 err);
}

#ifdef CONFIG_DEBUG_FS
static void panthor_gem_debugfs_bo_init(struct panthor_gem_object *bo)
{
	INIT_LIST_HEAD(&bo->debugfs.node);
}

static void panthor_gem_debugfs_bo_add(struct panthor_gem_object *bo)
{
	struct panthor_device *ptdev = container_of(bo->base.dev,
						    struct panthor_device, base);

	bo->debugfs.creator.tgid = current->tgid;
	get_task_comm(bo->debugfs.creator.process_name, current->group_leader);

	mutex_lock(&ptdev->gems.lock);
	list_add_tail(&bo->debugfs.node, &ptdev->gems.node);
	mutex_unlock(&ptdev->gems.lock);
}

static void panthor_gem_debugfs_bo_rm(struct panthor_gem_object *bo)
{
	struct panthor_device *ptdev = container_of(bo->base.dev,
						    struct panthor_device, base);

	if (list_empty(&bo->debugfs.node))
		return;

	mutex_lock(&ptdev->gems.lock);
	list_del_init(&bo->debugfs.node);
	mutex_unlock(&ptdev->gems.lock);
}

static void panthor_gem_debugfs_set_usage_flags(struct panthor_gem_object *bo, u32 usage_flags)
{
	bo->debugfs.flags = usage_flags;
	panthor_gem_debugfs_bo_add(bo);
}
#else
static void panthor_gem_debugfs_bo_rm(struct panthor_gem_object *bo) {}
static void panthor_gem_debugfs_set_usage_flags(struct panthor_gem_object *bo, u32 usage_flags) {}
static void panthor_gem_debugfs_bo_init(struct panthor_gem_object *bo) {}
#endif

static bool
should_map_wc(struct panthor_gem_object *bo)
{
	struct panthor_device *ptdev = container_of(bo->base.dev, struct panthor_device, base);

	/* We can't do uncached mappings if the device is coherent,
	 * because the zeroing done by the shmem layer at page allocation
	 * time happens on a cached mapping which isn't CPU-flushed (at least
	 * not on Arm64 where the flush is deferred to PTE setup time, and
	 * only done conditionally based on the mapping permissions). We can't
	 * rely on dma_map_sgtable()/dma_sync_sgtable_for_xxx() either to flush
	 * those, because they are NOPed if dma_dev_coherent() returns true.
	 *
	 * FIXME: Note that this problem is going to pop up again when we
	 * decide to support mapping buffers with the NO_MMAP flag as
	 * non-shareable (AKA buffers accessed only by the GPU), because we
	 * need the same CPU flush to happen after page allocation, otherwise
	 * there's a risk of data leak or late corruption caused by a dirty
	 * cacheline being evicted. At this point we'll need a way to force
	 * CPU cache maintenance regardless of whether the device is coherent
	 * or not.
	 */
	if (ptdev->coherent)
		return false;

	/* Cached mappings are explicitly requested, so no write-combine. */
	if (bo->flags & DRM_PANTHOR_BO_WB_MMAP)
		return false;

	/* The default is write-combine. */
	return true;
}

static void
panthor_gem_backing_cleanup_locked(struct panthor_gem_object *bo)
{
	dma_resv_assert_held(bo->base.resv);

	if (!bo->backing.pages)
		return;

	drm_gem_put_pages(&bo->base, bo->backing.pages, true, false);
	bo->backing.pages = NULL;
}

static int
panthor_gem_backing_get_pages_locked(struct panthor_gem_object *bo)
{
	struct page **pages;

	dma_resv_assert_held(bo->base.resv);

	if (bo->backing.pages)
		return 0;

	pages = drm_gem_get_pages(&bo->base);
	if (IS_ERR(pages)) {
		drm_dbg_driver(bo->base.dev, "Failed to get pages (%pe)\n", pages);
		return PTR_ERR(pages);
	}

	bo->backing.pages = pages;
	return 0;
}

static int panthor_gem_backing_pin_locked(struct panthor_gem_object *bo)
{
	int ret;

	dma_resv_assert_held(bo->base.resv);
	drm_WARN_ON_ONCE(bo->base.dev, drm_gem_is_imported(&bo->base));

	if (refcount_inc_not_zero(&bo->backing.pin_count))
		return 0;

	ret = panthor_gem_backing_get_pages_locked(bo);
	if (!ret)
		refcount_set(&bo->backing.pin_count, 1);

	return ret;
}

static void panthor_gem_backing_unpin_locked(struct panthor_gem_object *bo)
{
	dma_resv_assert_held(bo->base.resv);
	drm_WARN_ON_ONCE(bo->base.dev, drm_gem_is_imported(&bo->base));

	if (refcount_dec_and_test(&bo->backing.pin_count)) {
		/* We don't release anything when pin_count drops to zero.
		 * Pages stay there until an explicit cleanup is requested.
		 */
	}
}

static void
panthor_gem_dev_map_cleanup_locked(struct panthor_gem_object *bo)
{
	dma_resv_assert_held(bo->base.resv);

	if (!bo->dmap.sgt)
		return;

	dma_unmap_sgtable(drm_dev_dma_dev(bo->base.dev), bo->dmap.sgt, DMA_BIDIRECTIONAL, 0);
	sg_free_table(bo->dmap.sgt);
	kfree(bo->dmap.sgt);
	bo->dmap.sgt = NULL;
}

static struct sg_table *
panthor_gem_dev_map_get_sgt_locked(struct panthor_gem_object *bo)
{
	struct sg_table *sgt;
	int ret;

	dma_resv_assert_held(bo->base.resv);

	if (bo->dmap.sgt)
		return bo->dmap.sgt;

	/* Pages stay around after they've been allocated. At least that stands
	 * until we add a shrinker.
	 */
	ret = panthor_gem_backing_get_pages_locked(bo);
	if (ret)
		return ERR_PTR(ret);

	sgt = drm_prime_pages_to_sg(bo->base.dev, bo->backing.pages,
				    bo->base.size >> PAGE_SHIFT);
	if (IS_ERR(sgt))
		return sgt;

	/* Map the pages for use by the h/w. */
	ret = dma_map_sgtable(drm_dev_dma_dev(bo->base.dev), sgt, DMA_BIDIRECTIONAL, 0);
	if (ret)
		goto err_free_sgt;

	bo->dmap.sgt = sgt;
	return sgt;

err_free_sgt:
	sg_free_table(sgt);
	kfree(sgt);
	return ERR_PTR(ret);
}

struct sg_table *
panthor_gem_get_dev_sgt(struct panthor_gem_object *bo)
{
	struct sg_table *sgt;

	dma_resv_lock(bo->base.resv, NULL);
	sgt = panthor_gem_dev_map_get_sgt_locked(bo);
	dma_resv_unlock(bo->base.resv);

	return sgt;
}

static void
panthor_gem_vmap_cleanup_locked(struct panthor_gem_object *bo)
{
	if (!bo->cmap.vaddr)
		return;

	vunmap(bo->cmap.vaddr);
	bo->cmap.vaddr = NULL;
	panthor_gem_backing_unpin_locked(bo);
}

static int
panthor_gem_prep_for_cpu_map_locked(struct panthor_gem_object *bo)
{
	if (should_map_wc(bo)) {
		struct sg_table *sgt;

		sgt = panthor_gem_dev_map_get_sgt_locked(bo);
		if (IS_ERR(sgt))
			return PTR_ERR(sgt);
	}

	return 0;
}

static void *
panthor_gem_vmap_get_locked(struct panthor_gem_object *bo)
{
	pgprot_t prot = PAGE_KERNEL;
	void *vaddr;
	int ret;

	dma_resv_assert_held(bo->base.resv);

	if (drm_WARN_ON_ONCE(bo->base.dev, drm_gem_is_imported(&bo->base)))
		return ERR_PTR(-EINVAL);

	if (refcount_inc_not_zero(&bo->cmap.vaddr_use_count)) {
		drm_WARN_ON_ONCE(bo->base.dev, !bo->cmap.vaddr);
		return bo->cmap.vaddr;
	}

	ret = panthor_gem_backing_pin_locked(bo);
	if (ret)
		return ERR_PTR(ret);

	ret = panthor_gem_prep_for_cpu_map_locked(bo);
	if (ret)
		goto err_unpin;

	if (should_map_wc(bo))
		prot = pgprot_writecombine(prot);

	vaddr = vmap(bo->backing.pages, bo->base.size >> PAGE_SHIFT, VM_MAP, prot);
	if (!vaddr) {
		ret = -ENOMEM;
		goto err_unpin;
	}

	bo->cmap.vaddr = vaddr;
	refcount_set(&bo->cmap.vaddr_use_count, 1);
	return vaddr;

err_unpin:
	panthor_gem_backing_unpin_locked(bo);
	return ERR_PTR(ret);
}

static void
panthor_gem_vmap_put_locked(struct panthor_gem_object *bo)
{
	dma_resv_assert_held(bo->base.resv);

	if (drm_WARN_ON_ONCE(bo->base.dev, drm_gem_is_imported(&bo->base)))
		return;

	if (refcount_dec_and_test(&bo->cmap.vaddr_use_count))
		panthor_gem_vmap_cleanup_locked(bo);
}

static void panthor_gem_free_object(struct drm_gem_object *obj)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	struct drm_gem_object *vm_root_gem = bo->exclusive_vm_root_gem;

	panthor_gem_debugfs_bo_rm(bo);

	/*
	 * Label might have been allocated with kstrdup_const(),
	 * we need to take that into account when freeing the memory
	 */
	kfree_const(bo->label.str);

	mutex_destroy(&bo->label.lock);

	if (drm_gem_is_imported(obj)) {
		drm_prime_gem_destroy(obj, bo->dmap.sgt);
	} else {
		dma_resv_lock(obj->resv, NULL);
		panthor_gem_vmap_cleanup_locked(bo);
		panthor_gem_dev_map_cleanup_locked(bo);
		panthor_gem_backing_cleanup_locked(bo);
		dma_resv_unlock(obj->resv);
	}

	drm_gem_object_release(obj);

	kfree(bo);
	drm_gem_object_put(vm_root_gem);
}

static struct sg_table *
panthor_gem_prime_map_dma_buf(struct dma_buf_attachment *attach,
			      enum dma_data_direction dir)
{
	struct sg_table *sgt = drm_gem_map_dma_buf(attach, dir);

	if (!IS_ERR(sgt))
		attach->priv = sgt;

	return sgt;
}

static void
panthor_gem_prime_unmap_dma_buf(struct dma_buf_attachment *attach,
				struct sg_table *sgt,
				enum dma_data_direction dir)
{
	attach->priv = NULL;
	drm_gem_unmap_dma_buf(attach, sgt, dir);
}

static int
panthor_gem_prime_begin_cpu_access(struct dma_buf *dma_buf,
				   enum dma_data_direction dir)
{
	struct drm_gem_object *obj = dma_buf->priv;
	struct drm_device *dev = obj->dev;
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	struct dma_buf_attachment *attach;

	dma_resv_lock(obj->resv, NULL);
	if (bo->dmap.sgt)
		dma_sync_sgtable_for_cpu(drm_dev_dma_dev(dev), bo->dmap.sgt, dir);

	if (bo->cmap.vaddr)
		invalidate_kernel_vmap_range(bo->cmap.vaddr, bo->base.size);

	list_for_each_entry(attach, &dma_buf->attachments, node) {
		struct sg_table *sgt = attach->priv;

		if (sgt)
			dma_sync_sgtable_for_cpu(attach->dev, sgt, dir);
	}
	dma_resv_unlock(obj->resv);

	return 0;
}

static int
panthor_gem_prime_end_cpu_access(struct dma_buf *dma_buf,
				 enum dma_data_direction dir)
{
	struct drm_gem_object *obj = dma_buf->priv;
	struct drm_device *dev = obj->dev;
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	struct dma_buf_attachment *attach;

	dma_resv_lock(obj->resv, NULL);
	list_for_each_entry(attach, &dma_buf->attachments, node) {
		struct sg_table *sgt = attach->priv;

		if (sgt)
			dma_sync_sgtable_for_device(attach->dev, sgt, dir);
	}

	if (bo->cmap.vaddr)
		flush_kernel_vmap_range(bo->cmap.vaddr, bo->base.size);

	if (bo->dmap.sgt)
		dma_sync_sgtable_for_device(drm_dev_dma_dev(dev), bo->dmap.sgt, dir);

	dma_resv_unlock(obj->resv);
	return 0;
}

static const struct dma_buf_ops panthor_dma_buf_ops = {
	.attach = drm_gem_map_attach,
	.detach = drm_gem_map_detach,
	.map_dma_buf = panthor_gem_prime_map_dma_buf,
	.unmap_dma_buf = panthor_gem_prime_unmap_dma_buf,
	.release = drm_gem_dmabuf_release,
	.mmap = drm_gem_dmabuf_mmap,
	.vmap = drm_gem_dmabuf_vmap,
	.vunmap = drm_gem_dmabuf_vunmap,
	.begin_cpu_access = panthor_gem_prime_begin_cpu_access,
	.end_cpu_access = panthor_gem_prime_end_cpu_access,
};

static struct dma_buf *
panthor_gem_prime_export(struct drm_gem_object *obj, int flags)
{
	struct drm_device *dev = obj->dev;
	struct dma_buf_export_info exp_info = {
		.exp_name = KBUILD_MODNAME,
		.owner = THIS_MODULE,
		.ops = &panthor_dma_buf_ops,
		.size = obj->size,
		.flags = flags,
		.priv = obj,
		.resv = obj->resv,
	};

	/* We can't export GEMs that have an exclusive VM. */
	if (to_panthor_bo(obj)->exclusive_vm_root_gem)
		return ERR_PTR(-EINVAL);

	return drm_gem_dmabuf_export(dev, &exp_info);
}

struct drm_gem_object *
panthor_gem_prime_import(struct drm_device *dev,
			 struct dma_buf *dma_buf)
{
	struct drm_gem_object *obj = dma_buf->priv;

	if (dma_buf->ops == &panthor_dma_buf_ops && obj->dev == dev) {
		/* Importing dmabuf exported from our own gem increases
		 * refcount on gem itself instead of f_count of dmabuf.
		 */
		drm_gem_object_get(obj);
		return obj;
	}

	return drm_gem_prime_import(dev, dma_buf);
}

static void panthor_gem_print_info(struct drm_printer *p, unsigned int indent,
				   const struct drm_gem_object *obj)
{
	const struct panthor_gem_object *bo = to_panthor_bo(obj);

	if (drm_gem_is_imported(&bo->base))
		return;

	drm_printf_indent(p, indent, "resident=%s\n", str_true_false(bo->backing.pages));
	drm_printf_indent(p, indent, "pages_pin_count=%u\n", refcount_read(&bo->backing.pin_count));
	drm_printf_indent(p, indent, "vmap_use_count=%u\n",
			  refcount_read(&bo->cmap.vaddr_use_count));
	drm_printf_indent(p, indent, "vaddr=%p\n", bo->cmap.vaddr);
}

static int panthor_gem_pin_locked(struct drm_gem_object *obj)
{
	if (!drm_gem_is_imported(obj))
		return panthor_gem_backing_pin_locked(to_panthor_bo(obj));

	return 0;
}

static void panthor_gem_unpin_locked(struct drm_gem_object *obj)
{
	if (!drm_gem_is_imported(obj))
		panthor_gem_backing_unpin_locked(to_panthor_bo(obj));
}

int panthor_gem_pin(struct panthor_gem_object *bo)
{
	int ret = 0;

	if (drm_gem_is_imported(&bo->base))
		return 0;

	if (refcount_inc_not_zero(&bo->backing.pin_count))
		return 0;

	dma_resv_lock(bo->base.resv, NULL);
	ret = panthor_gem_backing_pin_locked(bo);
	dma_resv_unlock(bo->base.resv);

	return ret;
}

void panthor_gem_unpin(struct panthor_gem_object *bo)
{
	if (drm_gem_is_imported(&bo->base))
		return;

	if (refcount_dec_not_one(&bo->backing.pin_count))
		return;

	dma_resv_lock(bo->base.resv, NULL);
	panthor_gem_backing_unpin_locked(bo);
	dma_resv_unlock(bo->base.resv);
}

static struct sg_table *panthor_gem_get_sg_table(struct drm_gem_object *obj)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);

	drm_WARN_ON_ONCE(obj->dev, drm_gem_is_imported(obj));
	drm_WARN_ON_ONCE(obj->dev, !bo->backing.pages);
	drm_WARN_ON_ONCE(obj->dev, !refcount_read(&bo->backing.pin_count));

	return drm_prime_pages_to_sg(obj->dev, bo->backing.pages, obj->size >> PAGE_SHIFT);
}

static int panthor_gem_vmap_locked(struct drm_gem_object *obj,
				   struct iosys_map *map)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	void *vaddr;

	dma_resv_assert_held(obj->resv);

	if (drm_gem_is_imported(obj))
		return dma_buf_vmap(obj->import_attach->dmabuf, map);

	vaddr = panthor_gem_vmap_get_locked(bo);
	if (IS_ERR(vaddr))
		return PTR_ERR(vaddr);

	iosys_map_set_vaddr(map, vaddr);
	return 0;
}

static void panthor_gem_vunmap_locked(struct drm_gem_object *obj,
				      struct iosys_map *map)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);

	dma_resv_assert_held(obj->resv);

	if (drm_gem_is_imported(obj)) {
		dma_buf_vunmap(obj->import_attach->dmabuf, map);
	} else {
		drm_WARN_ON_ONCE(obj->dev, bo->cmap.vaddr != map->vaddr);
		panthor_gem_vmap_put_locked(bo);
	}
}

static int panthor_gem_mmap(struct drm_gem_object *obj, struct vm_area_struct *vma)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	int ret;

	if (drm_gem_is_imported(obj)) {
		/* Reset both vm_ops and vm_private_data, so we don't end up with
		 * vm_ops pointing to our implementation if the dma-buf backend
		 * doesn't set those fields.
		 */
		vma->vm_private_data = NULL;
		vma->vm_ops = NULL;

		ret = dma_buf_mmap(obj->dma_buf, vma, 0);

		/* Drop the reference drm_gem_mmap_obj() acquired.*/
		if (!ret)
			drm_gem_object_put(obj);

		return ret;
	}

	if (is_cow_mapping(vma->vm_flags))
		return -EINVAL;

	vm_flags_set(vma, VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP);
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	if (should_map_wc(bo))
		vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	return 0;
}

static enum drm_gem_object_status panthor_gem_status(struct drm_gem_object *obj)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	enum drm_gem_object_status res = 0;

	if (drm_gem_is_imported(&bo->base) || bo->backing.pages)
		res |= DRM_GEM_OBJECT_RESIDENT;

	return res;
}

static vm_fault_t insert_page(struct vm_fault *vmf, unsigned int order, struct page *page)
{
	if (!order) {
		return vmf_insert_pfn(vmf->vma, vmf->address, page_to_pfn(page));
#ifdef CONFIG_ARCH_SUPPORTS_PMD_PFNMAP
	} else if (order == PMD_ORDER) {
		unsigned long pfn = page_to_pfn(page);
		unsigned long paddr = pfn << PAGE_SHIFT;
		bool aligned = (vmf->address & ~PMD_MASK) == (paddr & ~PMD_MASK);

		if (aligned &&
		    folio_test_pmd_mappable(page_folio(page))) {
			pfn &= PMD_MASK >> PAGE_SHIFT;
			return vmf_insert_pfn_pmd(vmf, pfn, vmf->flags & FAULT_FLAG_WRITE);
		}
#endif
	}

	return VM_FAULT_FALLBACK;
}

static vm_fault_t nonblocking_page_setup(struct vm_fault *vmf,
					 unsigned int order,
					 pgoff_t page_offset)
{
	struct vm_area_struct *vma = vmf->vma;
	struct panthor_gem_object *bo = to_panthor_bo(vma->vm_private_data);
	vm_fault_t ret;

	if (!dma_resv_trylock(bo->base.resv))
		return VM_FAULT_RETRY;

	if (bo->backing.pages)
		ret = insert_page(vmf, order, bo->backing.pages[page_offset]);
	else
		ret = VM_FAULT_RETRY;

	dma_resv_unlock(bo->base.resv);
	return ret;
}

static vm_fault_t blocking_page_setup(struct vm_fault *vmf, unsigned int order,
				      struct panthor_gem_object *bo,
				      pgoff_t page_offset, bool mmap_lock_held)
{
	vm_fault_t ret;
	int err;

	err = dma_resv_lock_interruptible(bo->base.resv, NULL);
	if (err)
		return mmap_lock_held ? VM_FAULT_NOPAGE : VM_FAULT_RETRY;

	err = panthor_gem_backing_get_pages_locked(bo);
	if (!err)
		err = panthor_gem_prep_for_cpu_map_locked(bo);

	if (err) {
		ret = mmap_lock_held ? VM_FAULT_SIGBUS : VM_FAULT_RETRY;
	} else {
		struct page *page = bo->backing.pages[page_offset];

		if (mmap_lock_held)
			ret = insert_page(vmf, order, page);
		else
			ret = VM_FAULT_RETRY;
	}

	dma_resv_unlock(bo->base.resv);

	return ret;
}

static vm_fault_t panthor_gem_any_fault(struct vm_fault *vmf, unsigned int order)
{
	struct vm_area_struct *vma = vmf->vma;
	struct panthor_gem_object *bo = to_panthor_bo(vma->vm_private_data);
	loff_t num_pages = bo->base.size >> PAGE_SHIFT;
	pgoff_t page_offset;
	vm_fault_t ret;

	if (order && order != PMD_ORDER)
		return VM_FAULT_FALLBACK;

	/* Offset to faulty address in the VMA. */
	page_offset = vmf->pgoff - vma->vm_pgoff;
	if (page_offset >= num_pages)
		return VM_FAULT_SIGBUS;

	ret = nonblocking_page_setup(vmf, order, page_offset);
	if (ret != VM_FAULT_RETRY)
		return ret;

	/* Check if we're allowed to retry. */
	if (fault_flag_allow_retry_first(vmf->flags)) {
		/* If we're allowed to retry but not wait here, return
		 * immediately, the wait will be done when the fault
		 * handler is called again, with the mmap_lock held.
		 */
		if (vmf->flags & FAULT_FLAG_RETRY_NOWAIT)
			return VM_FAULT_RETRY;

		/* Wait with the mmap lock released, if we're allowed to. */
		drm_gem_object_get(&bo->base);
		mmap_read_unlock(vmf->vma->vm_mm);
		ret = blocking_page_setup(vmf, order, bo, page_offset, false);
		drm_gem_object_put(&bo->base);
		return ret;
	}

	return blocking_page_setup(vmf, order, bo, page_offset, true);
}

static vm_fault_t panthor_gem_fault(struct vm_fault *vmf)
{
	return panthor_gem_any_fault(vmf, 0);
}

static const struct vm_operations_struct panthor_gem_vm_ops = {
	.fault = panthor_gem_fault,
#ifdef CONFIG_ARCH_SUPPORTS_PMD_PFNMAP
	.huge_fault = panthor_gem_any_fault,
#endif
	.open = drm_gem_vm_open,
	.close = drm_gem_vm_close,
};

static const struct drm_gem_object_funcs panthor_gem_funcs = {
	.free = panthor_gem_free_object,
	.print_info = panthor_gem_print_info,
	.pin = panthor_gem_pin_locked,
	.unpin = panthor_gem_unpin_locked,
	.get_sg_table = panthor_gem_get_sg_table,
	.vmap = panthor_gem_vmap_locked,
	.vunmap = panthor_gem_vunmap_locked,
	.mmap = panthor_gem_mmap,
	.status = panthor_gem_status,
	.export = panthor_gem_prime_export,
	.vm_ops = &panthor_gem_vm_ops,
};

static struct panthor_gem_object *
panthor_gem_alloc_object(u32 flags)
{
	struct panthor_gem_object *bo;

	bo = kzalloc_obj(*bo);
	if (!bo)
		return ERR_PTR(-ENOMEM);

	bo->base.funcs = &panthor_gem_funcs;
	bo->flags = flags;
	mutex_init(&bo->label.lock);
	panthor_gem_debugfs_bo_init(bo);
	return bo;
}

static struct panthor_gem_object *
panthor_gem_create(struct drm_device *dev, size_t size, uint32_t flags,
		   struct panthor_vm *exclusive_vm, u32 usage_flags)
{
	struct panthor_gem_object *bo;
	int ret;

	bo = panthor_gem_alloc_object(flags);
	if (IS_ERR(bo))
		return bo;

	size = PAGE_ALIGN(size);
	ret = drm_gem_object_init(dev, &bo->base, size);
	if (ret)
		goto err_put;

	/* Our buffers are kept pinned, so allocating them
	 * from the MOVABLE zone is a really bad idea, and
	 * conflicts with CMA. See comments above new_inode()
	 * why this is required _and_ expected if you're
	 * going to pin these pages.
	 */
	mapping_set_gfp_mask(bo->base.filp->f_mapping,
			     GFP_HIGHUSER | __GFP_RETRY_MAYFAIL | __GFP_NOWARN);

	ret = drm_gem_create_mmap_offset(&bo->base);
	if (ret)
		goto err_put;

	if (exclusive_vm) {
		bo->exclusive_vm_root_gem = panthor_vm_root_gem(exclusive_vm);
		drm_gem_object_get(bo->exclusive_vm_root_gem);
		bo->base.resv = bo->exclusive_vm_root_gem->resv;
	}

	panthor_gem_debugfs_set_usage_flags(bo, usage_flags);
	return bo;

err_put:
	drm_gem_object_put(&bo->base);
	return ERR_PTR(ret);
}

struct drm_gem_object *
panthor_gem_prime_import_sg_table(struct drm_device *dev,
				  struct dma_buf_attachment *attach,
				  struct sg_table *sgt)
{
	struct panthor_gem_object *bo;
	int ret;

	bo = panthor_gem_alloc_object(0);
	if (IS_ERR(bo))
		return ERR_CAST(bo);

	drm_gem_private_object_init(dev, &bo->base, attach->dmabuf->size);

	ret = drm_gem_create_mmap_offset(&bo->base);
	if (ret)
		goto err_put;

	bo->dmap.sgt = sgt;
	return &bo->base;

err_put:
	drm_gem_object_put(&bo->base);
	return ERR_PTR(ret);
}

/**
 * panthor_gem_create_with_handle() - Create a GEM object and attach it to a handle.
 * @file: DRM file.
 * @ddev: DRM device.
 * @exclusive_vm: Exclusive VM. Not NULL if the GEM object can't be shared.
 * @size: Size of the GEM object to allocate.
 * @flags: Combination of drm_panthor_bo_flags flags.
 * @handle: Pointer holding the handle pointing to the new GEM object.
 *
 * Return: Zero on success
 */
int
panthor_gem_create_with_handle(struct drm_file *file,
			       struct drm_device *ddev,
			       struct panthor_vm *exclusive_vm,
			       u64 *size, u32 flags, u32 *handle)
{
	int ret;
	struct panthor_gem_object *bo;

	bo = panthor_gem_create(ddev, *size, flags, exclusive_vm, 0);
	if (IS_ERR(bo))
		return PTR_ERR(bo);

	/*
	 * Allocate an id of idr table where the obj is registered
	 * and handle has the id what user can see.
	 */
	ret = drm_gem_handle_create(file, &bo->base, handle);
	if (!ret)
		*size = bo->base.size;

	/* drop reference from allocate - handle holds it now. */
	drm_gem_object_put(&bo->base);
	return ret;
}

void
panthor_gem_bo_set_label(struct drm_gem_object *obj, const char *label)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	const char *old_label;

	scoped_guard(mutex, &bo->label.lock) {
		old_label = bo->label.str;
		bo->label.str = label;
	}

	kfree_const(old_label);
}

void
panthor_gem_kernel_bo_set_label(struct panthor_kernel_bo *bo, const char *label)
{
	const char *str;

	/* We should never attempt labelling a UM-exposed GEM object */
	if (drm_WARN_ON(bo->obj->dev, bo->obj->handle_count > 0))
		return;

	if (!label)
		return;

	str = kstrdup_const(label, GFP_KERNEL);
	if (!str) {
		/* Failing to allocate memory for a label isn't a fatal condition */
		drm_warn(bo->obj->dev, "Not enough memory to allocate BO label");
		return;
	}

	panthor_gem_bo_set_label(bo->obj, str);
}

int
panthor_gem_sync(struct drm_gem_object *obj, u32 type,
		 u64 offset, u64 size)
{
	struct panthor_gem_object *bo = to_panthor_bo(obj);
	struct device *dma_dev = drm_dev_dma_dev(bo->base.dev);
	struct sg_table *sgt;
	struct scatterlist *sgl;
	unsigned int count;

	/* Make sure the range is in bounds. */
	if (offset + size < offset || offset + size > bo->base.size)
		return -EINVAL;

	/* Disallow CPU-cache maintenance on imported buffers. */
	if (drm_gem_is_imported(&bo->base))
		return -EINVAL;

	switch (type) {
	case DRM_PANTHOR_BO_SYNC_CPU_CACHE_FLUSH:
	case DRM_PANTHOR_BO_SYNC_CPU_CACHE_FLUSH_AND_INVALIDATE:
		break;

	default:
		return -EINVAL;
	}

	/* Don't bother if it's WC-mapped */
	if (should_map_wc(bo))
		return 0;

	/* Nothing to do if the size is zero. */
	if (size == 0)
		return 0;

	sgt = panthor_gem_get_dev_sgt(bo);
	if (IS_ERR(sgt))
		return PTR_ERR(sgt);

	for_each_sgtable_dma_sg(sgt, sgl, count) {
		if (size == 0)
			break;

		dma_addr_t paddr = sg_dma_address(sgl);
		size_t len = sg_dma_len(sgl);

		if (len <= offset) {
			offset -= len;
			continue;
		}

		paddr += offset;
		len -= offset;
		len = min_t(size_t, len, size);
		size -= len;
		offset = 0;

		/* It's unclear whether dma_sync_xxx() is the right API to do CPU
		 * cache maintenance given an IOMMU can register their own
		 * implementation doing more than just CPU cache flushes/invalidation,
		 * and what we really care about here is CPU caches only, but that's
		 * the best we have that is both arch-agnostic and does at least the
		 * CPU cache maintenance on a <page,offset,size> tuple.
		 *
		 * Also, I wish we could do a single
		 *
		 *      dma_sync_single_for_device(BIDIR)
		 *
		 * and get a flush+invalidate, but that's not how it's implemented
		 * in practice (at least on arm64), so we have to make it
		 *
		 *      dma_sync_single_for_device(TO_DEVICE)
		 *      dma_sync_single_for_cpu(FROM_DEVICE)
		 *
		 * for the flush+invalidate case.
		 */
		dma_sync_single_for_device(dma_dev, paddr, len, DMA_TO_DEVICE);
		if (type == DRM_PANTHOR_BO_SYNC_CPU_CACHE_FLUSH_AND_INVALIDATE)
			dma_sync_single_for_cpu(dma_dev, paddr, len, DMA_FROM_DEVICE);
	}

	return 0;
}

/**
 * panthor_kernel_bo_destroy() - Destroy a kernel buffer object
 * @bo: Kernel buffer object to destroy. If NULL or an ERR_PTR(), the destruction
 * is skipped.
 */
void panthor_kernel_bo_destroy(struct panthor_kernel_bo *bo)
{
	struct panthor_vm *vm;

	if (IS_ERR_OR_NULL(bo))
		return;

	vm = bo->vm;
	panthor_kernel_bo_vunmap(bo);

	drm_WARN_ON(bo->obj->dev,
		    to_panthor_bo(bo->obj)->exclusive_vm_root_gem != panthor_vm_root_gem(vm));
	panthor_vm_unmap_range(vm, bo->va_node.start, bo->va_node.size);
	panthor_vm_free_va(vm, &bo->va_node);
	drm_gem_object_put(bo->obj);
	panthor_vm_put(vm);
	kfree(bo);
}

/**
 * panthor_kernel_bo_create() - Create and map a GEM object to a VM
 * @ptdev: Device.
 * @vm: VM to map the GEM to. If NULL, the kernel object is not GPU mapped.
 * @size: Size of the buffer object.
 * @bo_flags: Combination of drm_panthor_bo_flags flags.
 * @vm_map_flags: Combination of drm_panthor_vm_bind_op_flags (only those
 * that are related to map operations).
 * @gpu_va: GPU address assigned when mapping to the VM.
 * If gpu_va == PANTHOR_VM_KERNEL_AUTO_VA, the virtual address will be
 * automatically allocated.
 * @name: Descriptive label of the BO's contents
 *
 * Return: A valid pointer in case of success, an ERR_PTR() otherwise.
 */
struct panthor_kernel_bo *
panthor_kernel_bo_create(struct panthor_device *ptdev, struct panthor_vm *vm,
			 size_t size, u32 bo_flags, u32 vm_map_flags,
			 u64 gpu_va, const char *name)
{
	struct panthor_kernel_bo *kbo;
	struct panthor_gem_object *bo;
	u32 debug_flags = PANTHOR_DEBUGFS_GEM_USAGE_FLAG_KERNEL;
	int ret;

	if (drm_WARN_ON(&ptdev->base, !vm))
		return ERR_PTR(-EINVAL);

	kbo = kzalloc_obj(*kbo);
	if (!kbo)
		return ERR_PTR(-ENOMEM);

	if (vm == panthor_fw_vm(ptdev))
		debug_flags |= PANTHOR_DEBUGFS_GEM_USAGE_FLAG_FW_MAPPED;

	bo = panthor_gem_create(&ptdev->base, size, bo_flags, vm, debug_flags);
	if (IS_ERR(bo)) {
		ret = PTR_ERR(bo);
		goto err_free_kbo;
	}

	kbo->obj = &bo->base;

	panthor_gem_kernel_bo_set_label(kbo, name);

	/* The system and GPU MMU page size might differ, which becomes a
	 * problem for FW sections that need to be mapped at explicit address
	 * since our PAGE_SIZE alignment might cover a VA range that's
	 * expected to be used for another section.
	 * Make sure we never map more than we need.
	 */
	size = ALIGN(size, panthor_vm_page_size(vm));
	ret = panthor_vm_alloc_va(vm, gpu_va, size, &kbo->va_node);
	if (ret)
		goto err_put_obj;

	ret = panthor_vm_map_bo_range(vm, bo, 0, size, kbo->va_node.start, vm_map_flags);
	if (ret)
		goto err_free_va;

	kbo->vm = panthor_vm_get(vm);
	return kbo;

err_free_va:
	panthor_vm_free_va(vm, &kbo->va_node);

err_put_obj:
	drm_gem_object_put(&bo->base);

err_free_kbo:
	kfree(kbo);
	return ERR_PTR(ret);
}

#ifdef CONFIG_DEBUG_FS
struct gem_size_totals {
	size_t size;
	size_t resident;
	size_t reclaimable;
};

static void panthor_gem_debugfs_print_flag_names(struct seq_file *m)
{
	int len;
	int i;

	static const char * const gem_state_flags_names[] = {
		[PANTHOR_DEBUGFS_GEM_STATE_IMPORTED_BIT] = "imported",
		[PANTHOR_DEBUGFS_GEM_STATE_EXPORTED_BIT] = "exported",
	};

	static const char * const gem_usage_flags_names[] = {
		[PANTHOR_DEBUGFS_GEM_USAGE_KERNEL_BIT] = "kernel",
		[PANTHOR_DEBUGFS_GEM_USAGE_FW_MAPPED_BIT] = "fw-mapped",
	};

	seq_puts(m, "GEM state flags: ");
	for (i = 0, len = ARRAY_SIZE(gem_state_flags_names); i < len; i++) {
		if (!gem_state_flags_names[i])
			continue;
		seq_printf(m, "%s (0x%x)%s", gem_state_flags_names[i],
			   (u32)BIT(i), (i < len - 1) ? ", " : "\n");
	}

	seq_puts(m, "GEM usage flags: ");
	for (i = 0, len = ARRAY_SIZE(gem_usage_flags_names); i < len; i++) {
		if (!gem_usage_flags_names[i])
			continue;
		seq_printf(m, "%s (0x%x)%s", gem_usage_flags_names[i],
			   (u32)BIT(i), (i < len - 1) ? ", " : "\n\n");
	}
}

static void panthor_gem_debugfs_bo_print(struct panthor_gem_object *bo,
					 struct seq_file *m,
					 struct gem_size_totals *totals)
{
	unsigned int refcount = kref_read(&bo->base.refcount);
	char creator_info[32] = {};
	size_t resident_size;
	u32 gem_usage_flags = bo->debugfs.flags;
	u32 gem_state_flags = 0;

	/* Skip BOs being destroyed. */
	if (!refcount)
		return;

	resident_size = bo->backing.pages ? bo->base.size : 0;

	snprintf(creator_info, sizeof(creator_info),
		 "%s/%d", bo->debugfs.creator.process_name, bo->debugfs.creator.tgid);
	seq_printf(m, "%-32s%-16d%-16d%-16zd%-16zd0x%-16lx",
		   creator_info,
		   bo->base.name,
		   refcount,
		   bo->base.size,
		   resident_size,
		   drm_vma_node_start(&bo->base.vma_node));

	if (drm_gem_is_imported(&bo->base))
		gem_state_flags |= PANTHOR_DEBUGFS_GEM_STATE_FLAG_IMPORTED;
	if (bo->base.dma_buf)
		gem_state_flags |= PANTHOR_DEBUGFS_GEM_STATE_FLAG_EXPORTED;

	seq_printf(m, "0x%-8x 0x%-10x", gem_state_flags, gem_usage_flags);

	scoped_guard(mutex, &bo->label.lock) {
		seq_printf(m, "%s\n", bo->label.str ? : "");
	}

	totals->size += bo->base.size;
	totals->resident += resident_size;
}

static void panthor_gem_debugfs_print_bos(struct panthor_device *ptdev,
					  struct seq_file *m)
{
	struct gem_size_totals totals = {0};
	struct panthor_gem_object *bo;

	panthor_gem_debugfs_print_flag_names(m);

	seq_puts(m, "created-by                      global-name     refcount        size            resident-size   file-offset       state      usage       label\n");
	seq_puts(m, "----------------------------------------------------------------------------------------------------------------------------------------------\n");

	scoped_guard(mutex, &ptdev->gems.lock) {
		list_for_each_entry(bo, &ptdev->gems.node, debugfs.node) {
			panthor_gem_debugfs_bo_print(bo, m, &totals);
		}
	}

	seq_puts(m, "==============================================================================================================================================\n");
	seq_printf(m, "Total size: %zd, Total resident: %zd, Total reclaimable: %zd\n",
		   totals.size, totals.resident, totals.reclaimable);
}

static int panthor_gem_show_bos(struct seq_file *m, void *data)
{
	struct drm_info_node *node = m->private;
	struct drm_device *dev = node->minor->dev;
	struct panthor_device *ptdev =
		container_of(dev, struct panthor_device, base);

	panthor_gem_debugfs_print_bos(ptdev, m);

	return 0;
}

static struct drm_info_list panthor_gem_debugfs_list[] = {
	{ "gems", panthor_gem_show_bos, 0, NULL },
};

void panthor_gem_debugfs_init(struct drm_minor *minor)
{
	drm_debugfs_create_files(panthor_gem_debugfs_list,
				 ARRAY_SIZE(panthor_gem_debugfs_list),
				 minor->debugfs_root, minor);
}
#endif
