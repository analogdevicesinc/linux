// SPDX-License-Identifier: MIT
/*
 * Copyright © 2021 Intel Corporation
 */

#include "xe_device.h"

#include <linux/units.h>

#include <drm/drm_aperture.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_client.h>
#include <drm/drm_gem_ttm_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_managed.h>
#include <drm/drm_print.h>
#include <drm/xe_drm.h>

#include "display/xe_display.h"
#include "regs/xe_gt_regs.h"
#include "regs/xe_regs.h"
#include "xe_bo.h"
#include "xe_debugfs.h"
#include "xe_devcoredump.h"
#include "xe_dma_buf.h"
#include "xe_drm_client.h"
#include "xe_drv.h"
#include "xe_exec.h"
#include "xe_exec_queue.h"
#include "xe_ggtt.h"
#include "xe_gsc_proxy.h"
#include "xe_gt.h"
#include "xe_gt_mcr.h"
#include "xe_hwmon.h"
#include "xe_irq.h"
#include "xe_memirq.h"
#include "xe_mmio.h"
#include "xe_module.h"
#include "xe_pat.h"
#include "xe_pcode.h"
#include "xe_pm.h"
#include "xe_query.h"
#include "xe_sriov.h"
#include "xe_tile.h"
#include "xe_ttm_stolen_mgr.h"
#include "xe_ttm_sys_mgr.h"
#include "xe_vm.h"
#include "xe_wait_user_fence.h"

static int xe_file_open(struct drm_device *dev, struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_drm_client *client;
	struct xe_file *xef;
	int ret = -ENOMEM;

	xef = kzalloc(sizeof(*xef), GFP_KERNEL);
	if (!xef)
		return ret;

	client = xe_drm_client_alloc();
	if (!client) {
		kfree(xef);
		return ret;
	}

	xef->drm = file;
	xef->client = client;
	xef->xe = xe;

	mutex_init(&xef->vm.lock);
	xa_init_flags(&xef->vm.xa, XA_FLAGS_ALLOC1);

	mutex_init(&xef->exec_queue.lock);
	xa_init_flags(&xef->exec_queue.xa, XA_FLAGS_ALLOC1);

	spin_lock(&xe->clients.lock);
	xe->clients.count++;
	spin_unlock(&xe->clients.lock);

	file->driver_priv = xef;
	return 0;
}

static void xe_file_close(struct drm_device *dev, struct drm_file *file)
{
	struct xe_device *xe = to_xe_device(dev);
	struct xe_file *xef = file->driver_priv;
	struct xe_vm *vm;
	struct xe_exec_queue *q;
	unsigned long idx;

	mutex_lock(&xef->exec_queue.lock);
	xa_for_each(&xef->exec_queue.xa, idx, q) {
		xe_exec_queue_kill(q);
		xe_exec_queue_put(q);
	}
	mutex_unlock(&xef->exec_queue.lock);
	xa_destroy(&xef->exec_queue.xa);
	mutex_destroy(&xef->exec_queue.lock);
	mutex_lock(&xef->vm.lock);
	xa_for_each(&xef->vm.xa, idx, vm)
		xe_vm_close_and_put(vm);
	mutex_unlock(&xef->vm.lock);
	xa_destroy(&xef->vm.xa);
	mutex_destroy(&xef->vm.lock);

	spin_lock(&xe->clients.lock);
	xe->clients.count--;
	spin_unlock(&xe->clients.lock);

	xe_drm_client_put(xef->client);
	kfree(xef);
}

static const struct drm_ioctl_desc xe_ioctls[] = {
	DRM_IOCTL_DEF_DRV(XE_DEVICE_QUERY, xe_query_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_GEM_CREATE, xe_gem_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_GEM_MMAP_OFFSET, xe_gem_mmap_offset_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_VM_CREATE, xe_vm_create_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_VM_DESTROY, xe_vm_destroy_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_VM_BIND, xe_vm_bind_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_EXEC, xe_exec_ioctl, DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_EXEC_QUEUE_CREATE, xe_exec_queue_create_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_EXEC_QUEUE_DESTROY, xe_exec_queue_destroy_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_EXEC_QUEUE_GET_PROPERTY, xe_exec_queue_get_property_ioctl,
			  DRM_RENDER_ALLOW),
	DRM_IOCTL_DEF_DRV(XE_WAIT_USER_FENCE, xe_wait_user_fence_ioctl,
			  DRM_RENDER_ALLOW),
};

static long xe_drm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = file->private_data;
	struct xe_device *xe = to_xe_device(file_priv->minor->dev);
	long ret;

	ret = xe_pm_runtime_get_ioctl(xe);
	if (ret >= 0)
		ret = drm_ioctl(file, cmd, arg);
	xe_pm_runtime_put(xe);

	return ret;
}

#ifdef CONFIG_COMPAT
static long xe_drm_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = file->private_data;
	struct xe_device *xe = to_xe_device(file_priv->minor->dev);
	long ret;

	ret = xe_pm_runtime_get_ioctl(xe);
	if (ret >= 0)
		ret = drm_compat_ioctl(file, cmd, arg);
	xe_pm_runtime_put(xe);

	return ret;
}
#else
/* similarly to drm_compat_ioctl, let's it be assigned to .compat_ioct unconditionally */
#define xe_drm_compat_ioctl NULL
#endif

static const struct file_operations xe_driver_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release_noglobal,
	.unlocked_ioctl = xe_drm_ioctl,
	.mmap = drm_gem_mmap,
	.poll = drm_poll,
	.read = drm_read,
	.compat_ioctl = xe_drm_compat_ioctl,
	.llseek = noop_llseek,
#ifdef CONFIG_PROC_FS
	.show_fdinfo = drm_show_fdinfo,
#endif
};

static void xe_driver_release(struct drm_device *dev)
{
	struct xe_device *xe = to_xe_device(dev);

	pci_set_drvdata(to_pci_dev(xe->drm.dev), NULL);
}

static struct drm_driver driver = {
	/* Don't use MTRRs here; the Xserver or userspace app should
	 * deal with them for Intel hardware.
	 */
	.driver_features =
	    DRIVER_GEM |
	    DRIVER_RENDER | DRIVER_SYNCOBJ |
	    DRIVER_SYNCOBJ_TIMELINE | DRIVER_GEM_GPUVA,
	.open = xe_file_open,
	.postclose = xe_file_close,

	.gem_prime_import = xe_gem_prime_import,

	.dumb_create = xe_bo_dumb_create,
	.dumb_map_offset = drm_gem_ttm_dumb_map_offset,
#ifdef CONFIG_PROC_FS
	.show_fdinfo = xe_drm_client_fdinfo,
#endif
	.release = &xe_driver_release,

	.ioctls = xe_ioctls,
	.num_ioctls = ARRAY_SIZE(xe_ioctls),
	.fops = &xe_driver_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = DRIVER_MAJOR,
	.minor = DRIVER_MINOR,
	.patchlevel = DRIVER_PATCHLEVEL,
};

static void xe_device_destroy(struct drm_device *dev, void *dummy)
{
	struct xe_device *xe = to_xe_device(dev);

	if (xe->preempt_fence_wq)
		destroy_workqueue(xe->preempt_fence_wq);

	if (xe->ordered_wq)
		destroy_workqueue(xe->ordered_wq);

	if (xe->unordered_wq)
		destroy_workqueue(xe->unordered_wq);

	ttm_device_fini(&xe->ttm);
}

struct xe_device *xe_device_create(struct pci_dev *pdev,
				   const struct pci_device_id *ent)
{
	struct xe_device *xe;
	int err;

	xe_display_driver_set_hooks(&driver);

	err = drm_aperture_remove_conflicting_pci_framebuffers(pdev, &driver);
	if (err)
		return ERR_PTR(err);

	xe = devm_drm_dev_alloc(&pdev->dev, &driver, struct xe_device, drm);
	if (IS_ERR(xe))
		return xe;

	err = ttm_device_init(&xe->ttm, &xe_ttm_funcs, xe->drm.dev,
			      xe->drm.anon_inode->i_mapping,
			      xe->drm.vma_offset_manager, false, false);
	if (WARN_ON(err))
		goto err;

	err = drmm_add_action_or_reset(&xe->drm, xe_device_destroy, NULL);
	if (err)
		goto err;

	xe->info.devid = pdev->device;
	xe->info.revid = pdev->revision;
	xe->info.force_execlist = xe_modparam.force_execlist;

	spin_lock_init(&xe->irq.lock);
	spin_lock_init(&xe->clients.lock);

	init_waitqueue_head(&xe->ufence_wq);

	drmm_mutex_init(&xe->drm, &xe->usm.lock);
	xa_init_flags(&xe->usm.asid_to_vm, XA_FLAGS_ALLOC);

	if (IS_ENABLED(CONFIG_DRM_XE_DEBUG)) {
		/* Trigger a large asid and an early asid wrap. */
		u32 asid;

		BUILD_BUG_ON(XE_MAX_ASID < 2);
		err = xa_alloc_cyclic(&xe->usm.asid_to_vm, &asid, NULL,
				      XA_LIMIT(XE_MAX_ASID - 2, XE_MAX_ASID - 1),
				      &xe->usm.next_asid, GFP_KERNEL);
		drm_WARN_ON(&xe->drm, err);
		if (err >= 0)
			xa_erase(&xe->usm.asid_to_vm, asid);
	}

	spin_lock_init(&xe->pinned.lock);
	INIT_LIST_HEAD(&xe->pinned.kernel_bo_present);
	INIT_LIST_HEAD(&xe->pinned.external_vram);
	INIT_LIST_HEAD(&xe->pinned.evicted);

	xe->preempt_fence_wq = alloc_ordered_workqueue("xe-preempt-fence-wq", 0);
	xe->ordered_wq = alloc_ordered_workqueue("xe-ordered-wq", 0);
	xe->unordered_wq = alloc_workqueue("xe-unordered-wq", 0, 0);
	if (!xe->ordered_wq || !xe->unordered_wq ||
	    !xe->preempt_fence_wq) {
		/*
		 * Cleanup done in xe_device_destroy via
		 * drmm_add_action_or_reset register above
		 */
		drm_err(&xe->drm, "Failed to allocate xe workqueues\n");
		err = -ENOMEM;
		goto err;
	}

	err = xe_display_create(xe);
	if (WARN_ON(err))
		goto err;

	return xe;

err:
	return ERR_PTR(err);
}

/*
 * The driver-initiated FLR is the highest level of reset that we can trigger
 * from within the driver. It is different from the PCI FLR in that it doesn't
 * fully reset the SGUnit and doesn't modify the PCI config space and therefore
 * it doesn't require a re-enumeration of the PCI BARs. However, the
 * driver-initiated FLR does still cause a reset of both GT and display and a
 * memory wipe of local and stolen memory, so recovery would require a full HW
 * re-init and saving/restoring (or re-populating) the wiped memory. Since we
 * perform the FLR as the very last action before releasing access to the HW
 * during the driver release flow, we don't attempt recovery at all, because
 * if/when a new instance of i915 is bound to the device it will do a full
 * re-init anyway.
 */
static void xe_driver_flr(struct xe_device *xe)
{
	const unsigned int flr_timeout = 3 * MICRO; /* specs recommend a 3s wait */
	struct xe_gt *gt = xe_root_mmio_gt(xe);
	int ret;

	if (xe_mmio_read32(gt, GU_CNTL_PROTECTED) & DRIVERINT_FLR_DIS) {
		drm_info_once(&xe->drm, "BIOS Disabled Driver-FLR\n");
		return;
	}

	drm_dbg(&xe->drm, "Triggering Driver-FLR\n");

	/*
	 * Make sure any pending FLR requests have cleared by waiting for the
	 * FLR trigger bit to go to zero. Also clear GU_DEBUG's DRIVERFLR_STATUS
	 * to make sure it's not still set from a prior attempt (it's a write to
	 * clear bit).
	 * Note that we should never be in a situation where a previous attempt
	 * is still pending (unless the HW is totally dead), but better to be
	 * safe in case something unexpected happens
	 */
	ret = xe_mmio_wait32(gt, GU_CNTL, DRIVERFLR, 0, flr_timeout, NULL, false);
	if (ret) {
		drm_err(&xe->drm, "Driver-FLR-prepare wait for ready failed! %d\n", ret);
		return;
	}
	xe_mmio_write32(gt, GU_DEBUG, DRIVERFLR_STATUS);

	/* Trigger the actual Driver-FLR */
	xe_mmio_rmw32(gt, GU_CNTL, 0, DRIVERFLR);

	/* Wait for hardware teardown to complete */
	ret = xe_mmio_wait32(gt, GU_CNTL, DRIVERFLR, 0, flr_timeout, NULL, false);
	if (ret) {
		drm_err(&xe->drm, "Driver-FLR-teardown wait completion failed! %d\n", ret);
		return;
	}

	/* Wait for hardware/firmware re-init to complete */
	ret = xe_mmio_wait32(gt, GU_DEBUG, DRIVERFLR_STATUS, DRIVERFLR_STATUS,
			     flr_timeout, NULL, false);
	if (ret) {
		drm_err(&xe->drm, "Driver-FLR-reinit wait completion failed! %d\n", ret);
		return;
	}

	/* Clear sticky completion status */
	xe_mmio_write32(gt, GU_DEBUG, DRIVERFLR_STATUS);
}

static void xe_driver_flr_fini(struct drm_device *drm, void *arg)
{
	struct xe_device *xe = arg;

	if (xe->needs_flr_on_fini)
		xe_driver_flr(xe);
}

static void xe_device_sanitize(struct drm_device *drm, void *arg)
{
	struct xe_device *xe = arg;
	struct xe_gt *gt;
	u8 id;

	for_each_gt(gt, xe, id)
		xe_gt_sanitize(gt);
}

static int xe_set_dma_info(struct xe_device *xe)
{
	unsigned int mask_size = xe->info.dma_mask_size;
	int err;

	dma_set_max_seg_size(xe->drm.dev, xe_sg_segment_size(xe->drm.dev));

	err = dma_set_mask(xe->drm.dev, DMA_BIT_MASK(mask_size));
	if (err)
		goto mask_err;

	err = dma_set_coherent_mask(xe->drm.dev, DMA_BIT_MASK(mask_size));
	if (err)
		goto mask_err;

	return 0;

mask_err:
	drm_err(&xe->drm, "Can't set DMA mask/consistent mask (%d)\n", err);
	return err;
}

static bool verify_lmem_ready(struct xe_gt *gt)
{
	u32 val = xe_mmio_read32(gt, GU_CNTL) & LMEM_INIT;

	return !!val;
}

static int wait_for_lmem_ready(struct xe_device *xe)
{
	struct xe_gt *gt = xe_root_mmio_gt(xe);
	unsigned long timeout, start;

	if (!IS_DGFX(xe))
		return 0;

	if (IS_SRIOV_VF(xe))
		return 0;

	if (verify_lmem_ready(gt))
		return 0;

	drm_dbg(&xe->drm, "Waiting for lmem initialization\n");

	start = jiffies;
	timeout = start + msecs_to_jiffies(60 * 1000); /* 60 sec! */

	do {
		if (signal_pending(current))
			return -EINTR;

		/*
		 * The boot firmware initializes local memory and
		 * assesses its health. If memory training fails,
		 * the punit will have been instructed to keep the GT powered
		 * down.we won't be able to communicate with it
		 *
		 * If the status check is done before punit updates the register,
		 * it can lead to the system being unusable.
		 * use a timeout and defer the probe to prevent this.
		 */
		if (time_after(jiffies, timeout)) {
			drm_dbg(&xe->drm, "lmem not initialized by firmware\n");
			return -EPROBE_DEFER;
		}

		msleep(20);

	} while (!verify_lmem_ready(gt));

	drm_dbg(&xe->drm, "lmem ready after %ums",
		jiffies_to_msecs(jiffies - start));

	return 0;
}

/**
 * xe_device_probe_early: Device early probe
 * @xe: xe device instance
 *
 * Initialize MMIO resources that don't require any
 * knowledge about tile count. Also initialize pcode and
 * check vram initialization on root tile.
 *
 * Return: 0 on success, error code on failure
 */
int xe_device_probe_early(struct xe_device *xe)
{
	int err;

	err = xe_mmio_init(xe);
	if (err)
		return err;

	xe_sriov_probe_early(xe);

	err = xe_pcode_probe_early(xe);
	if (err)
		return err;

	err = wait_for_lmem_ready(xe);
	if (err)
		return err;

	return 0;
}

static int xe_device_set_has_flat_ccs(struct  xe_device *xe)
{
	u32 reg;
	int err;

	if (GRAPHICS_VER(xe) < 20 || !xe->info.has_flat_ccs)
		return 0;

	struct xe_gt *gt = xe_root_mmio_gt(xe);

	err = xe_force_wake_get(gt_to_fw(gt), XE_FW_GT);
	if (err)
		return err;

	reg = xe_gt_mcr_unicast_read_any(gt, XE2_FLAT_CCS_BASE_RANGE_LOWER);
	xe->info.has_flat_ccs = (reg & XE2_FLAT_CCS_ENABLE);

	if (!xe->info.has_flat_ccs)
		drm_dbg(&xe->drm,
			"Flat CCS has been disabled in bios, May lead to performance impact");

	return xe_force_wake_put(gt_to_fw(gt), XE_FW_GT);
}

int xe_device_probe(struct xe_device *xe)
{
	struct xe_tile *tile;
	struct xe_gt *gt;
	int err;
	u8 last_gt;
	u8 id;

	xe_pat_init_early(xe);

	err = xe_sriov_init(xe);
	if (err)
		return err;

	xe->info.mem_region_mask = 1;
	err = xe_display_init_nommio(xe);
	if (err)
		return err;

	err = xe_set_dma_info(xe);
	if (err)
		return err;

	xe_mmio_probe_tiles(xe);

	xe_ttm_sys_mgr_init(xe);

	for_each_gt(gt, xe, id)
		xe_force_wake_init_gt(gt, gt_to_fw(gt));

	for_each_tile(tile, xe, id) {
		err = xe_ggtt_init_early(tile->mem.ggtt);
		if (err)
			return err;
		if (IS_SRIOV_VF(xe)) {
			err = xe_memirq_init(&tile->sriov.vf.memirq);
			if (err)
				return err;
		}
	}

	for_each_gt(gt, xe, id) {
		err = xe_gt_init_hwconfig(gt);
		if (err)
			return err;
	}

	err = xe_devcoredump_init(xe);
	if (err)
		return err;
	err = drmm_add_action_or_reset(&xe->drm, xe_driver_flr_fini, xe);
	if (err)
		return err;

	for_each_gt(gt, xe, id)
		xe_pcode_init(gt);

	err = xe_display_init_noirq(xe);
	if (err)
		return err;

	err = xe_irq_install(xe);
	if (err)
		goto err;

	for_each_gt(gt, xe, id) {
		err = xe_gt_init_early(gt);
		if (err)
			goto err_irq_shutdown;
	}

	err = xe_device_set_has_flat_ccs(xe);
	if (err)
		goto err_irq_shutdown;

	err = xe_mmio_probe_vram(xe);
	if (err)
		goto err_irq_shutdown;

	for_each_tile(tile, xe, id) {
		err = xe_tile_init_noalloc(tile);
		if (err)
			goto err_irq_shutdown;
	}

	/* Allocate and map stolen after potential VRAM resize */
	xe_ttm_stolen_mgr_init(xe);

	/*
	 * Now that GT is initialized (TTM in particular),
	 * we can try to init display, and inherit the initial fb.
	 * This is the reason the first allocation needs to be done
	 * inside display.
	 */
	err = xe_display_init_noaccel(xe);
	if (err)
		goto err_irq_shutdown;

	for_each_gt(gt, xe, id) {
		last_gt = id;

		err = xe_gt_init(gt);
		if (err)
			goto err_fini_gt;
	}

	xe_heci_gsc_init(xe);

	err = xe_display_init(xe);
	if (err)
		goto err_fini_gt;

	err = drm_dev_register(&xe->drm, 0);
	if (err)
		goto err_fini_display;

	xe_display_register(xe);

	xe_debugfs_register(xe);

	xe_hwmon_register(xe);

	return drmm_add_action_or_reset(&xe->drm, xe_device_sanitize, xe);

err_fini_display:
	xe_display_driver_remove(xe);

err_fini_gt:
	for_each_gt(gt, xe, id) {
		if (id < last_gt)
			xe_gt_remove(gt);
		else
			break;
	}

err_irq_shutdown:
	xe_irq_shutdown(xe);
err:
	xe_display_fini(xe);
	return err;
}

static void xe_device_remove_display(struct xe_device *xe)
{
	xe_display_unregister(xe);

	drm_dev_unplug(&xe->drm);
	xe_display_driver_remove(xe);
}

void xe_device_remove(struct xe_device *xe)
{
	struct xe_gt *gt;
	u8 id;

	xe_device_remove_display(xe);

	xe_display_fini(xe);

	xe_heci_gsc_fini(xe);

	for_each_gt(gt, xe, id)
		xe_gt_remove(gt);

	xe_irq_shutdown(xe);
}

void xe_device_shutdown(struct xe_device *xe)
{
}

void xe_device_wmb(struct xe_device *xe)
{
	struct xe_gt *gt = xe_root_mmio_gt(xe);

	wmb();
	if (IS_DGFX(xe))
		xe_mmio_write32(gt, SOFTWARE_FLAGS_SPR33, 0);
}

u32 xe_device_ccs_bytes(struct xe_device *xe, u64 size)
{
	return xe_device_has_flat_ccs(xe) ?
		DIV_ROUND_UP_ULL(size, NUM_BYTES_PER_CCS_BYTE(xe)) : 0;
}

/**
 * xe_device_assert_mem_access - Inspect the current runtime_pm state.
 * @xe: xe device instance
 *
 * To be used before any kind of memory access. It will splat a debug warning
 * if the device is currently sleeping. But it doesn't guarantee in any way
 * that the device is going to remain awake. Xe PM runtime get and put
 * functions might be added to the outer bound of the memory access, while
 * this check is intended for inner usage to splat some warning if the worst
 * case has just happened.
 */
void xe_device_assert_mem_access(struct xe_device *xe)
{
	xe_assert(xe, !xe_pm_runtime_suspended(xe));
}

void xe_device_snapshot_print(struct xe_device *xe, struct drm_printer *p)
{
	struct xe_gt *gt;
	u8 id;

	drm_printf(p, "PCI ID: 0x%04x\n", xe->info.devid);
	drm_printf(p, "PCI revision: 0x%02x\n", xe->info.revid);

	for_each_gt(gt, xe, id) {
		drm_printf(p, "GT id: %u\n", id);
		drm_printf(p, "\tType: %s\n",
			   gt->info.type == XE_GT_TYPE_MAIN ? "main" : "media");
		drm_printf(p, "\tIP ver: %u.%u.%u\n",
			   REG_FIELD_GET(GMD_ID_ARCH_MASK, gt->info.gmdid),
			   REG_FIELD_GET(GMD_ID_RELEASE_MASK, gt->info.gmdid),
			   REG_FIELD_GET(GMD_ID_REVID, gt->info.gmdid));
		drm_printf(p, "\tCS reference clock: %u\n", gt->info.reference_clock);
	}
}

u64 xe_device_canonicalize_addr(struct xe_device *xe, u64 address)
{
	return sign_extend64(address, xe->info.va_bits - 1);
}

u64 xe_device_uncanonicalize_addr(struct xe_device *xe, u64 address)
{
	return address & GENMASK_ULL(xe->info.va_bits - 1, 0);
}
