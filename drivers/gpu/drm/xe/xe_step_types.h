/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2022 Intel Corporation
 */

#ifndef _XE_STEP_TYPES_H_
#define _XE_STEP_TYPES_H_

#include <linux/types.h>

#include <drm/intel/step.h>

#define xe_step intel_step

struct xe_step_info {
	u8 platform;
	u8 graphics;
	u8 media;
	u8 basedie;
};

#endif
