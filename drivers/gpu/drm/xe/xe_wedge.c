// SPDX-License-Identifier: MIT
/*
 * Copyright © 2026 Intel Corporation
 */

#include <drm/drm_drv.h>
#include <drm/drm_managed.h>

#include "xe_defaults.h"
#include "xe_device.h"
#include "xe_gt.h"
#include "xe_gt_printk.h"
#include "xe_guc_ads.h"
#include "xe_log.h"
#include "xe_module.h"
#include "xe_pm.h"
#include "xe_printk.h"
#include "xe_sriov.h"
#include "xe_wedge.h"

/**
 * DOC: Xe Device Wedging
 *
 * Xe driver uses drm device wedged uevent as documented in Documentation/gpu/drm-uapi.rst.
 * When device is in wedged state, every IOCTL will be blocked and GT cannot
 * be used. The conditions under which the driver declares the device wedged
 * depend on the wedge mode configuration (see &enum xe_wedged_mode). The
 * default recovery method for a wedged state is rebind/bus-reset.
 *
 * Another recovery method is vendor-specific. Below are the cases that send
 * ``WEDGED=vendor-specific`` recovery method in drm device wedged uevent.
 *
 * Case: Firmware Flash
 * --------------------
 *
 * Identification Hint
 * +++++++++++++++++++
 *
 * ``WEDGED=vendor-specific`` drm device wedged uevent with
 * :ref:`Runtime Survivability mode <xe-survivability-mode>` is used to notify
 * admin/userspace consumer about the need for a firmware flash.
 *
 * Recovery Procedure
 * ++++++++++++++++++
 *
 * Once ``WEDGED=vendor-specific`` drm device wedged uevent is received, follow
 * the below steps
 *
 * - Check Runtime Survivability mode sysfs.
 *   If enabled, firmware flash is required to recover the device.
 *
 *   /sys/bus/pci/devices/<device>/survivability_mode
 *
 * - Admin/userspace consumer can use firmware flashing tools like fwupd to flash
 *   firmware and restore device to normal operation.
 */

static void wedge_work(struct work_struct *work)
{
	struct xe_device *xe = container_of(work, struct xe_device, wedged.work);

	/* Notify userspace of wedged device */
	drm_dev_wedged_event(&xe->drm, xe->wedged.method, NULL);
}

#define WEDGED_URL	"https://docs.kernel.org/gpu/drm-uapi.html#device-wedging"
#define XE_BUG_URL	"https://gitlab.freedesktop.org/drm/xe/kernel/issues/new"

/**
 * xe_wedge_declare() - Declare device wedged
 * @xe: xe device instance
 * @method: method(s) to be used for recovery
 *
 * This is the final state that can only be cleared with the recovery method
 * specified in the drm wedged uevent.
 *
 * In this state every IOCTL will be blocked so the GT cannot be used.
 * In general it will be called upon any critical error such as gt reset
 * failure or guc loading failure. Userspace will be notified of this state
 * through device wedged uevent.
 *
 * If xe.wedged module parameter is set to 2, this function will be called
 * on every single execution timeout (a.k.a. GPU hang) right after devcoredump
 * snapshot capture. In this mode, GT reset won't be attempted so the state of
 * the issue is preserved for further debugging.
 *
 * Since the wedge handling is done as part of worker thread, this is safe for
 * atomic callers.
 */
void xe_wedge_declare(struct xe_device *xe, unsigned long method)
{
	struct xe_gt *gt;
	u8 id;

	if (xe->wedged.mode == XE_WEDGED_MODE_NEVER) {
		xe_dbg(xe, "Wedged mode is forcibly disabled\n");
		return;
	}

	if (atomic_xchg(&xe->wedged.flag, 1))
		return;

	xe->needs_flr_on_fini = true;
	xe_pm_runtime_get_noresume(xe);

	xe_log_err_fatal(xe, WEDGED, -EIO, "Device declared wedged!\n");
	xe_err_once(xe, "IOCTLs and executions are now blocked!\n"
			"For recovery procedure, refer to %s\n"
			"Please file a _new_ bug report at %s\n",
			WEDGED_URL, XE_BUG_URL);

	for_each_initialized_gt(gt, xe, id)
		xe_gt_wedge_declare(gt);

	/*
	 * XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET is intended for debugging
	 * hangs, so wedge the device with 'none' recovery method and have
	 * it available to the user for debugging.
	 */
	if (xe->wedged.mode == XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET)
		xe->wedged.method = DRM_WEDGE_RECOVERY_NONE;
	else
		xe->wedged.method = method;

	schedule_work(&xe->wedged.work);
}

static const char *wedge_mode_to_string(enum xe_wedged_mode mode)
{
	switch (mode) {
	case XE_WEDGED_MODE_NEVER:
		return "never";
	case XE_WEDGED_MODE_UPON_CRITICAL_ERROR:
		return "upon-critical-error";
	case XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET:
		return "upon-any-hang-no-reset";
	default:
		return "<invalid>";
	}
}

static int wedge_validate_mode(struct xe_device *xe, unsigned int mode)
{
	if (mode > XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET) {
		xe_dbg(xe, "wedged_mode: invalid value (%u)\n", mode);
		return -EINVAL;
	} else if (mode == XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET && (IS_SRIOV_VF(xe) ||
		   (IS_SRIOV_PF(xe) && !IS_ENABLED(CONFIG_DRM_XE_DEBUG)))) {
		xe_dbg(xe, "wedged_mode: (%u) %s mode is not supported for %s\n",
		       mode, wedge_mode_to_string(mode),
		       xe_sriov_mode_to_string(xe_device_sriov_mode(xe)));
		return -EPERM;
	}

	return 0;
}

static bool wedge_mode_needs_policy_update(struct xe_device *xe, enum xe_wedged_mode mode)
{
	if (xe->wedged.inconsistent_reset)
		return true;

	if (xe->wedged.mode == mode)
		return false;

	if (xe->wedged.mode == XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET ||
	    mode == XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET)
		return true;

	return false;
}

static int __wedge_mode_set_reset_policy(struct xe_gt *gt, enum xe_wedged_mode mode)
{
	bool enable_engine_reset;
	int ret;

	enable_engine_reset = (mode != XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET);
	ret = xe_guc_ads_scheduler_policy_toggle_reset(&gt->uc.guc.ads,
						       enable_engine_reset);
	if (ret)
		xe_gt_err(gt, "Failed to update GuC ADS scheduler policy (%pe)\n", ERR_PTR(ret));

	return ret;
}

static int wedge_mode_set_reset_policy(struct xe_device *xe, enum xe_wedged_mode mode)
{
	struct xe_gt *gt;
	int ret;
	u8 id;

	guard(xe_pm_runtime)(xe);
	for_each_gt(gt, xe, id) {
		ret = __wedge_mode_set_reset_policy(gt, mode);
		if (ret) {
			if (id > 0) {
				xe->wedged.inconsistent_reset = true;
				xe_err(xe, "Inconsistent reset policy state between GTs\n");
			}
			return ret;
		}
	}

	xe->wedged.inconsistent_reset = false;

	return 0;
}

/**
 * xe_wedge_set_mode() - Set wedge mode
 * @xe: xe device instance
 * @mode: wedge mode to be set
 *
 * Returns: 0 on success, negative error code otherwise.
 */
int xe_wedge_set_mode(struct xe_device *xe, enum xe_wedged_mode mode)
{
	int ret;

	ret = wedge_validate_mode(xe, mode);
	if (ret)
		return ret;

	if (wedge_mode_needs_policy_update(xe, mode)) {
		ret = wedge_mode_set_reset_policy(xe, mode);
		if (ret)
			return ret;
	}

	xe->wedged.mode = mode;

	return ret;
}

static void xe_wedge_fini(struct drm_device *drm, void *arg)
{
	struct xe_device *xe = arg;

	disable_work_sync(&xe->wedged.work);

	if (atomic_read(&xe->wedged.flag))
		xe_pm_runtime_put(xe);
}

/**
 * xe_wedge_init() - Initialize wedge
 * @xe: xe device instance
 *
 * Returns: 0 on success, negative error code otherwise.
 */
int xe_wedge_init(struct xe_device *xe)
{
	INIT_WORK(&xe->wedged.work, wedge_work);

	xe->wedged.mode = wedge_validate_mode(xe, xe_modparam.wedged_mode) ?
			  XE_DEFAULT_WEDGED_MODE : xe_modparam.wedged_mode;
	xe_dbg(xe, "wedged_mode: setting mode (%u) %s\n",
	       xe->wedged.mode, wedge_mode_to_string(xe->wedged.mode));

	return drmm_add_action_or_reset(&xe->drm, xe_wedge_fini, xe);
}
