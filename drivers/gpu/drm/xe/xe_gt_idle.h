/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2023 Intel Corporation
 */

#ifndef _XE_GT_IDLE_H_
#define _XE_GT_IDLE_H_

#include "xe_gt_idle_types.h"

struct xe_gt;

int xe_gt_idle_sysfs_init(struct xe_gt_idle *gtidle);
void xe_gt_idle_enable_c6(struct xe_gt *gt);
void xe_gt_idle_disable_c6(struct xe_gt *gt);

#endif /* _XE_GT_IDLE_H_ */
