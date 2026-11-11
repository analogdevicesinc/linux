/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2019 Intel Corporation
 */

#ifndef __INTEL_CLOCK_GATING_H__
#define __INTEL_CLOCK_GATING_H__

struct drm_device;

void intel_clock_gating_init(struct drm_device *drm);
void intel_clock_gating_resume(struct drm_device *drm);
void intel_clock_gating_hooks_init(struct drm_device *drm);

#endif /* __INTEL_CLOCK_GATING_H__ */
