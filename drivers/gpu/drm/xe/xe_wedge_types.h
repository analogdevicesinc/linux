/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2026 Intel Corporation
 */

#ifndef _XE_WEDGE_TYPES_H_
#define _XE_WEDGE_TYPES_H_

#include <linux/types.h>
#include <linux/workqueue_types.h>

/**
 * enum xe_wedged_mode - Possible wedge modes
 * @XE_WEDGED_MODE_NEVER: Device will never be declared wedged.
 * @XE_WEDGED_MODE_UPON_CRITICAL_ERROR: Device will be declared wedged only
 *	when critical error occurs like GT reset failure or firmware failure.
 *	This is the default mode.
 * @XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET: Device will be declared wedged on
 *	any hang. In this mode, engine resets are disabled to avoid automatic
 *	recovery attempts. This mode is primarily intended for debugging hangs.
 */
enum xe_wedged_mode {
	XE_WEDGED_MODE_NEVER = 0,
	XE_WEDGED_MODE_UPON_CRITICAL_ERROR = 1,
	XE_WEDGED_MODE_UPON_ANY_HANG_NO_RESET = 2,
};

/**
 * struct xe_wedge - Struct to control Wedged States and mode
 */
struct xe_wedge {
	/** @flag: Xe device faced a critical error and is now blocked. */
	atomic_t flag;
	/** @mode: Mode controlled by kernel parameter and debugfs */
	enum xe_wedged_mode mode;
	/** @method: Recovery method to be sent in the drm device wedged uevent */
	unsigned long method;
	/** @inconsistent_reset: Inconsistent reset policy state between GTs */
	bool inconsistent_reset;
	/** @work: Worker to defer non-atomic calls */
	struct work_struct work;
};

#endif
