/* SPDX-License-Identifier: MIT */
/*
 * Copyright © 2023 Intel Corporation
 */

#ifndef __INTEL_UNCORE_H__
#define __INTEL_UNCORE_H__

#include "i915_reg_defs.h"
#include "xe_device.h"
#include "xe_device_types.h"
#include "xe_mmio.h"

static inline struct intel_uncore *to_intel_uncore(struct drm_device *drm)
{
	return &to_xe_device(drm)->uncore;
}

static inline struct xe_mmio *__compat_uncore_to_mmio(struct intel_uncore *uncore)
{
	struct xe_device *xe = container_of(uncore, struct xe_device, uncore);

	return xe_root_tile_mmio(xe);
}

static inline u32 intel_uncore_read(struct intel_uncore *uncore,
				    i915_reg_t i915_reg)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	return xe_mmio_read32(__compat_uncore_to_mmio(uncore), reg);
}

static inline u8 intel_uncore_read8(struct intel_uncore *uncore,
				    i915_reg_t i915_reg)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	return xe_mmio_read8(__compat_uncore_to_mmio(uncore), reg);
}

static inline void intel_uncore_write8(struct intel_uncore *uncore,
				       i915_reg_t i915_reg, u8 val)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	xe_mmio_write8(__compat_uncore_to_mmio(uncore), reg, val);
}

static inline u16 intel_uncore_read16(struct intel_uncore *uncore,
				      i915_reg_t i915_reg)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	return xe_mmio_read16(__compat_uncore_to_mmio(uncore), reg);
}

static inline void intel_uncore_write(struct intel_uncore *uncore,
				      i915_reg_t i915_reg, u32 val)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	xe_mmio_write32(__compat_uncore_to_mmio(uncore), reg, val);
}

static inline u32 intel_uncore_read_fw(struct intel_uncore *uncore,
				       i915_reg_t i915_reg)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	return xe_mmio_read32(__compat_uncore_to_mmio(uncore), reg);
}

static inline void intel_uncore_write_fw(struct intel_uncore *uncore,
					 i915_reg_t i915_reg, u32 val)
{
	struct xe_reg reg = XE_REG(i915_mmio_reg_offset(i915_reg));

	xe_mmio_write32(__compat_uncore_to_mmio(uncore), reg, val);
}

static inline bool
intel_uncore_arm_unclaimed_mmio_detection(struct intel_uncore *uncore)
{
	return false;
}

#endif /* __INTEL_UNCORE_H__ */
