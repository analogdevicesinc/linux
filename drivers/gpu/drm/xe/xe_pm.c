// SPDX-License-Identifier: MIT
/*
 * Copyright © 2022 Intel Corporation
 */

#include "xe_pm.h"

#include <linux/pm_runtime.h>

#include <drm/drm_managed.h>
#include <drm/ttm/ttm_placement.h>

#include "display/xe_display.h"
#include "xe_bo.h"
#include "xe_bo_evict.h"
#include "xe_device.h"
#include "xe_device_sysfs.h"
#include "xe_ggtt.h"
#include "xe_gt.h"
#include "xe_guc.h"
#include "xe_irq.h"
#include "xe_pcode.h"
#include "xe_wa.h"

/**
 * DOC: Xe Power Management
 *
 * Xe PM implements the main routines for both system level suspend states and
 * for the opportunistic runtime suspend states.
 *
 * System Level Suspend (S-States) - In general this is OS initiated suspend
 * driven by ACPI for achieving S0ix (a.k.a. S2idle, freeze), S3 (suspend to ram),
 * S4 (disk). The main functions here are `xe_pm_suspend` and `xe_pm_resume`. They
 * are the main point for the suspend to and resume from these states.
 *
 * PCI Device Suspend (D-States) - This is the opportunistic PCIe device low power
 * state D3, controlled by the PCI subsystem and ACPI with the help from the
 * runtime_pm infrastructure.
 * PCI D3 is special and can mean D3hot, where Vcc power is on for keeping memory
 * alive and quicker low latency resume or D3Cold where Vcc power is off for
 * better power savings.
 * The Vcc control of PCI hierarchy can only be controlled at the PCI root port
 * level, while the device driver can be behind multiple bridges/switches and
 * paired with other devices. For this reason, the PCI subsystem cannot perform
 * the transition towards D3Cold. The lowest runtime PM possible from the PCI
 * subsystem is D3hot. Then, if all these paired devices in the same root port
 * are in D3hot, ACPI will assist here and run its own methods (_PR3 and _OFF)
 * to perform the transition from D3hot to D3cold. Xe may disallow this
 * transition by calling pci_d3cold_disable(root_pdev) before going to runtime
 * suspend. It will be based on runtime conditions such as VRAM usage for a
 * quick and low latency resume for instance.
 *
 * Runtime PM - This infrastructure provided by the Linux kernel allows the
 * device drivers to indicate when the can be runtime suspended, so the device
 * could be put at D3 (if supported), or allow deeper package sleep states
 * (PC-states), and/or other low level power states. Xe PM component provides
 * `xe_pm_runtime_suspend` and `xe_pm_runtime_resume` functions that PCI
 * subsystem will call before transition to/from runtime suspend.
 *
 * Also, Xe PM provides get and put functions that Xe driver will use to
 * indicate activity. In order to avoid locking complications with the memory
 * management, whenever possible, these get and put functions needs to be called
 * from the higher/outer levels.
 * The main cases that need to be protected from the outer levels are: IOCTL,
 * sysfs, debugfs, dma-buf sharing, GPU execution.
 *
 * This component is not responsible for GT idleness (RC6) nor GT frequency
 * management (RPS).
 */

#ifdef CONFIG_LOCKDEP
struct lockdep_map xe_pm_runtime_lockdep_map = {
	.name = "xe_pm_runtime_lockdep_map"
};
#endif

/**
 * xe_pm_suspend - Helper for System suspend, i.e. S0->S3 / S0->S2idle
 * @xe: xe device instance
 *
 * Return: 0 on success
 */
int xe_pm_suspend(struct xe_device *xe)
{
	struct xe_gt *gt;
	u8 id;
	int err;

	drm_dbg(&xe->drm, "Suspending device\n");

	for_each_gt(gt, xe, id)
		xe_gt_suspend_prepare(gt);

	/* FIXME: Super racey... */
	err = xe_bo_evict_all(xe);
	if (err)
		goto err;

	xe_display_pm_suspend(xe);

	for_each_gt(gt, xe, id) {
		err = xe_gt_suspend(gt);
		if (err) {
			xe_display_pm_resume(xe);
			goto err;
		}
	}

	xe_irq_suspend(xe);

	xe_display_pm_suspend_late(xe);

	drm_dbg(&xe->drm, "Device suspended\n");
	return 0;
err:
	drm_dbg(&xe->drm, "Device suspend failed %d\n", err);
	return err;
}

/**
 * xe_pm_resume - Helper for System resume S3->S0 / S2idle->S0
 * @xe: xe device instance
 *
 * Return: 0 on success
 */
int xe_pm_resume(struct xe_device *xe)
{
	struct xe_tile *tile;
	struct xe_gt *gt;
	u8 id;
	int err;

	drm_dbg(&xe->drm, "Resuming device\n");

	for_each_tile(tile, xe, id)
		xe_wa_apply_tile_workarounds(tile);

	err = xe_pcode_ready(xe, true);
	if (err)
		return err;

	xe_display_pm_resume_early(xe);

	/*
	 * This only restores pinned memory which is the memory required for the
	 * GT(s) to resume.
	 */
	err = xe_bo_restore_kernel(xe);
	if (err)
		goto err;

	xe_irq_resume(xe);

	xe_display_pm_resume(xe);

	for_each_gt(gt, xe, id)
		xe_gt_resume(gt);

	err = xe_bo_restore_user(xe);
	if (err)
		goto err;

	drm_dbg(&xe->drm, "Device resumed\n");
	return 0;
err:
	drm_dbg(&xe->drm, "Device resume failed %d\n", err);
	return err;
}

static bool xe_pm_pci_d3cold_capable(struct xe_device *xe)
{
	struct pci_dev *pdev = to_pci_dev(xe->drm.dev);
	struct pci_dev *root_pdev;

	root_pdev = pcie_find_root_port(pdev);
	if (!root_pdev)
		return false;

	/* D3Cold requires PME capability */
	if (!pci_pme_capable(root_pdev, PCI_D3cold)) {
		drm_dbg(&xe->drm, "d3cold: PME# not supported\n");
		return false;
	}

	/* D3Cold requires _PR3 power resource */
	if (!pci_pr3_present(root_pdev)) {
		drm_dbg(&xe->drm, "d3cold: ACPI _PR3 not present\n");
		return false;
	}

	return true;
}

static void xe_pm_runtime_init(struct xe_device *xe)
{
	struct device *dev = xe->drm.dev;

	/*
	 * Disable the system suspend direct complete optimization.
	 * We need to ensure that the regular device suspend/resume functions
	 * are called since our runtime_pm cannot guarantee local memory
	 * eviction for d3cold.
	 * TODO: Check HDA audio dependencies claimed by i915, and then enforce
	 *       this option to integrated graphics as well.
	 */
	if (IS_DGFX(xe))
		dev_pm_set_driver_flags(dev, DPM_FLAG_NO_DIRECT_COMPLETE);

	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_set_active(dev);
	pm_runtime_allow(dev);
	pm_runtime_mark_last_busy(dev);
	pm_runtime_put(dev);
}

int xe_pm_init_early(struct xe_device *xe)
{
	int err;

	INIT_LIST_HEAD(&xe->mem_access.vram_userfault.list);

	err = drmm_mutex_init(&xe->drm, &xe->mem_access.vram_userfault.lock);
	if (err)
		return err;

	err = drmm_mutex_init(&xe->drm, &xe->d3cold.lock);
	if (err)
		return err;

	return 0;
}

/**
 * xe_pm_init - Initialize Xe Power Management
 * @xe: xe device instance
 *
 * This component is responsible for System and Device sleep states.
 *
 * Returns 0 for success, negative error code otherwise.
 */
int xe_pm_init(struct xe_device *xe)
{
	int err;

	/* For now suspend/resume is only allowed with GuC */
	if (!xe_device_uc_enabled(xe))
		return 0;

	xe->d3cold.capable = xe_pm_pci_d3cold_capable(xe);

	if (xe->d3cold.capable) {
		err = xe_device_sysfs_init(xe);
		if (err)
			return err;

		err = xe_pm_set_vram_threshold(xe, DEFAULT_VRAM_THRESHOLD);
		if (err)
			return err;
	}

	xe_pm_runtime_init(xe);

	return 0;
}

/**
 * xe_pm_runtime_fini - Finalize Runtime PM
 * @xe: xe device instance
 */
void xe_pm_runtime_fini(struct xe_device *xe)
{
	struct device *dev = xe->drm.dev;

	pm_runtime_get_sync(dev);
	pm_runtime_forbid(dev);
}

static void xe_pm_write_callback_task(struct xe_device *xe,
				      struct task_struct *task)
{
	WRITE_ONCE(xe->pm_callback_task, task);

	/*
	 * Just in case it's somehow possible for our writes to be reordered to
	 * the extent that something else re-uses the task written in
	 * pm_callback_task. For example after returning from the callback, but
	 * before the reordered write that resets pm_callback_task back to NULL.
	 */
	smp_mb(); /* pairs with xe_pm_read_callback_task */
}

struct task_struct *xe_pm_read_callback_task(struct xe_device *xe)
{
	smp_mb(); /* pairs with xe_pm_write_callback_task */

	return READ_ONCE(xe->pm_callback_task);
}

/**
 * xe_pm_runtime_suspended - Check if runtime_pm state is suspended
 * @xe: xe device instance
 *
 * This does not provide any guarantee that the device is going to remain
 * suspended as it might be racing with the runtime state transitions.
 * It can be used only as a non-reliable assertion, to ensure that we are not in
 * the sleep state while trying to access some memory for instance.
 *
 * Returns true if PCI device is suspended, false otherwise.
 */
bool xe_pm_runtime_suspended(struct xe_device *xe)
{
	return pm_runtime_suspended(xe->drm.dev);
}

/**
 * xe_pm_runtime_suspend - Prepare our device for D3hot/D3Cold
 * @xe: xe device instance
 *
 * Returns 0 for success, negative error code otherwise.
 */
int xe_pm_runtime_suspend(struct xe_device *xe)
{
	struct xe_bo *bo, *on;
	struct xe_gt *gt;
	u8 id;
	int err = 0;

	/* Disable access_ongoing asserts and prevent recursive pm calls */
	xe_pm_write_callback_task(xe, current);

	/*
	 * The actual xe_pm_runtime_put() is always async underneath, so
	 * exactly where that is called should makes no difference to us. However
	 * we still need to be very careful with the locks that this callback
	 * acquires and the locks that are acquired and held by any callers of
	 * xe_runtime_pm_get(). We already have the matching annotation
	 * on that side, but we also need it here. For example lockdep should be
	 * able to tell us if the following scenario is in theory possible:
	 *
	 * CPU0                          | CPU1 (kworker)
	 * lock(A)                       |
	 *                               | xe_pm_runtime_suspend()
	 *                               |      lock(A)
	 * xe_pm_runtime_get()           |
	 *
	 * This will clearly deadlock since rpm core needs to wait for
	 * xe_pm_runtime_suspend() to complete, but here we are holding lock(A)
	 * on CPU0 which prevents CPU1 making forward progress.  With the
	 * annotation here and in xe_pm_runtime_get() lockdep will see
	 * the potential lock inversion and give us a nice splat.
	 */
	lock_map_acquire(&xe_pm_runtime_lockdep_map);

	/*
	 * Applying lock for entire list op as xe_ttm_bo_destroy and xe_bo_move_notify
	 * also checks and delets bo entry from user fault list.
	 */
	mutex_lock(&xe->mem_access.vram_userfault.lock);
	list_for_each_entry_safe(bo, on,
				 &xe->mem_access.vram_userfault.list, vram_userfault_link)
		xe_bo_runtime_pm_release_mmap_offset(bo);
	mutex_unlock(&xe->mem_access.vram_userfault.lock);

	if (xe->d3cold.allowed) {
		err = xe_bo_evict_all(xe);
		if (err)
			goto out;
	}

	for_each_gt(gt, xe, id) {
		err = xe_gt_suspend(gt);
		if (err)
			goto out;
	}

	xe_irq_suspend(xe);
out:
	lock_map_release(&xe_pm_runtime_lockdep_map);
	xe_pm_write_callback_task(xe, NULL);
	return err;
}

/**
 * xe_pm_runtime_resume - Waking up from D3hot/D3Cold
 * @xe: xe device instance
 *
 * Returns 0 for success, negative error code otherwise.
 */
int xe_pm_runtime_resume(struct xe_device *xe)
{
	struct xe_gt *gt;
	u8 id;
	int err = 0;

	/* Disable access_ongoing asserts and prevent recursive pm calls */
	xe_pm_write_callback_task(xe, current);

	lock_map_acquire(&xe_pm_runtime_lockdep_map);

	/*
	 * It can be possible that xe has allowed d3cold but other pcie devices
	 * in gfx card soc would have blocked d3cold, therefore card has not
	 * really lost power. Detecting primary Gt power is sufficient.
	 */
	gt = xe_device_get_gt(xe, 0);
	xe->d3cold.power_lost = xe_guc_in_reset(&gt->uc.guc);

	if (xe->d3cold.allowed && xe->d3cold.power_lost) {
		err = xe_pcode_ready(xe, true);
		if (err)
			goto out;

		/*
		 * This only restores pinned memory which is the memory
		 * required for the GT(s) to resume.
		 */
		err = xe_bo_restore_kernel(xe);
		if (err)
			goto out;
	}

	xe_irq_resume(xe);

	for_each_gt(gt, xe, id)
		xe_gt_resume(gt);

	if (xe->d3cold.allowed && xe->d3cold.power_lost) {
		err = xe_bo_restore_user(xe);
		if (err)
			goto out;
	}
out:
	lock_map_release(&xe_pm_runtime_lockdep_map);
	xe_pm_write_callback_task(xe, NULL);
	return err;
}

/*
 * For places where resume is synchronous it can be quite easy to deadlock
 * if we are not careful. Also in practice it might be quite timing
 * sensitive to ever see the 0 -> 1 transition with the callers locks
 * held, so deadlocks might exist but are hard for lockdep to ever see.
 * With this in mind, help lockdep learn about the potentially scary
 * stuff that can happen inside the runtime_resume callback by acquiring
 * a dummy lock (it doesn't protect anything and gets compiled out on
 * non-debug builds).  Lockdep then only needs to see the
 * xe_pm_runtime_lockdep_map -> runtime_resume callback once, and then can
 * hopefully validate all the (callers_locks) -> xe_pm_runtime_lockdep_map.
 * For example if the (callers_locks) are ever grabbed in the
 * runtime_resume callback, lockdep should give us a nice splat.
 */
static void pm_runtime_lockdep_prime(void)
{
	lock_map_acquire(&xe_pm_runtime_lockdep_map);
	lock_map_release(&xe_pm_runtime_lockdep_map);
}

/**
 * xe_pm_runtime_get - Get a runtime_pm reference and resume synchronously
 * @xe: xe device instance
 */
void xe_pm_runtime_get(struct xe_device *xe)
{
	pm_runtime_get_noresume(xe->drm.dev);

	if (xe_pm_read_callback_task(xe) == current)
		return;

	pm_runtime_lockdep_prime();
	pm_runtime_resume(xe->drm.dev);
}

/**
 * xe_pm_runtime_put - Put the runtime_pm reference back and mark as idle
 * @xe: xe device instance
 */
void xe_pm_runtime_put(struct xe_device *xe)
{
	if (xe_pm_read_callback_task(xe) == current) {
		pm_runtime_put_noidle(xe->drm.dev);
	} else {
		pm_runtime_mark_last_busy(xe->drm.dev);
		pm_runtime_put(xe->drm.dev);
	}
}

/**
 * xe_pm_runtime_get_ioctl - Get a runtime_pm reference before ioctl
 * @xe: xe device instance
 *
 * Returns: Any number greater than or equal to 0 for success, negative error
 * code otherwise.
 */
int xe_pm_runtime_get_ioctl(struct xe_device *xe)
{
	if (WARN_ON(xe_pm_read_callback_task(xe) == current))
		return -ELOOP;

	pm_runtime_lockdep_prime();
	return pm_runtime_get_sync(xe->drm.dev);
}

/**
 * xe_pm_runtime_get_if_active - Get a runtime_pm reference if device active
 * @xe: xe device instance
 *
 * Returns: Any number greater than or equal to 0 for success, negative error
 * code otherwise.
 */
int xe_pm_runtime_get_if_active(struct xe_device *xe)
{
	return pm_runtime_get_if_active(xe->drm.dev);
}

/**
 * xe_pm_runtime_get_if_in_use - Get a runtime_pm reference and resume if needed
 * @xe: xe device instance
 *
 * Returns: True if device is awake and the reference was taken, false otherwise.
 */
bool xe_pm_runtime_get_if_in_use(struct xe_device *xe)
{
	if (xe_pm_read_callback_task(xe) == current) {
		/* The device is awake, grab the ref and move on */
		pm_runtime_get_noresume(xe->drm.dev);
		return true;
	}

	return pm_runtime_get_if_in_use(xe->drm.dev) > 0;
}

/**
 * xe_pm_runtime_get_noresume - Bump runtime PM usage counter without resuming
 * @xe: xe device instance
 *
 * This function should be used in inner places where it is surely already
 * protected by outer-bound callers of `xe_pm_runtime_get`.
 * It will warn if not protected.
 * The reference should be put back after this function regardless, since it
 * will always bump the usage counter, regardless.
 */
void xe_pm_runtime_get_noresume(struct xe_device *xe)
{
	bool ref;

	ref = xe_pm_runtime_get_if_in_use(xe);

	if (drm_WARN(&xe->drm, !ref, "Missing outer runtime PM protection\n"))
		pm_runtime_get_noresume(xe->drm.dev);
}

/**
 * xe_pm_runtime_resume_and_get - Resume, then get a runtime_pm ref if awake.
 * @xe: xe device instance
 *
 * Returns: True if device is awake and the reference was taken, false otherwise.
 */
bool xe_pm_runtime_resume_and_get(struct xe_device *xe)
{
	if (xe_pm_read_callback_task(xe) == current) {
		/* The device is awake, grab the ref and move on */
		pm_runtime_get_noresume(xe->drm.dev);
		return true;
	}

	pm_runtime_lockdep_prime();
	return pm_runtime_resume_and_get(xe->drm.dev) >= 0;
}

/**
 * xe_pm_assert_unbounded_bridge - Disable PM on unbounded pcie parent bridge
 * @xe: xe device instance
 */
void xe_pm_assert_unbounded_bridge(struct xe_device *xe)
{
	struct pci_dev *pdev = to_pci_dev(xe->drm.dev);
	struct pci_dev *bridge = pci_upstream_bridge(pdev);

	if (!bridge)
		return;

	if (!bridge->driver) {
		drm_warn(&xe->drm, "unbounded parent pci bridge, device won't support any PM support.\n");
		device_set_pm_not_required(&pdev->dev);
	}
}

/**
 * xe_pm_set_vram_threshold - Set a vram threshold for allowing/blocking D3Cold
 * @xe: xe device instance
 * @threshold: VRAM size in bites for the D3cold threshold
 *
 * Returns 0 for success, negative error code otherwise.
 */
int xe_pm_set_vram_threshold(struct xe_device *xe, u32 threshold)
{
	struct ttm_resource_manager *man;
	u32 vram_total_mb = 0;
	int i;

	for (i = XE_PL_VRAM0; i <= XE_PL_VRAM1; ++i) {
		man = ttm_manager_type(&xe->ttm, i);
		if (man)
			vram_total_mb += DIV_ROUND_UP_ULL(man->size, 1024 * 1024);
	}

	drm_dbg(&xe->drm, "Total vram %u mb\n", vram_total_mb);

	if (threshold > vram_total_mb)
		return -EINVAL;

	mutex_lock(&xe->d3cold.lock);
	xe->d3cold.vram_threshold = threshold;
	mutex_unlock(&xe->d3cold.lock);

	return 0;
}

/**
 * xe_pm_d3cold_allowed_toggle - Check conditions to toggle d3cold.allowed
 * @xe: xe device instance
 *
 * To be called during runtime_pm idle callback.
 * Check for all the D3Cold conditions ahead of runtime suspend.
 */
void xe_pm_d3cold_allowed_toggle(struct xe_device *xe)
{
	struct ttm_resource_manager *man;
	u32 total_vram_used_mb = 0;
	u64 vram_used;
	int i;

	if (!xe->d3cold.capable) {
		xe->d3cold.allowed = false;
		return;
	}

	for (i = XE_PL_VRAM0; i <= XE_PL_VRAM1; ++i) {
		man = ttm_manager_type(&xe->ttm, i);
		if (man) {
			vram_used = ttm_resource_manager_usage(man);
			total_vram_used_mb += DIV_ROUND_UP_ULL(vram_used, 1024 * 1024);
		}
	}

	mutex_lock(&xe->d3cold.lock);

	if (total_vram_used_mb < xe->d3cold.vram_threshold)
		xe->d3cold.allowed = true;
	else
		xe->d3cold.allowed = false;

	mutex_unlock(&xe->d3cold.lock);

	drm_dbg(&xe->drm,
		"d3cold: allowed=%s\n", str_yes_no(xe->d3cold.allowed));
}
