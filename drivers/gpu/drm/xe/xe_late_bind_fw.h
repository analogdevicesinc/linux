/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2025 Intel Corporation
 */

#ifndef _XE_LATE_BIND_FW_H_
#define _XE_LATE_BIND_FW_H_

#include <linux/types.h>

struct xe_late_bind;

int xe_late_bind_init(struct xe_late_bind *late_bind);
void xe_late_bind_pm_suspend(struct xe_late_bind *late_bind);
void xe_late_bind_pm_resume(struct xe_late_bind *late_bind);

#endif
