// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2014-2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include "backend/gpu/mali_kbase_cache_policy_backend.h"
#include <device/mali_kbase_device.h>

void kbase_cache_set_coherency_mode(struct kbase_device *kbdev, u32 mode)
{

	kbdev->current_gpu_coherency_mode = mode;

#if MALI_USE_CSF
	if (kbdev->gpu_props.gpu_id.arch_id >= GPU_ID_ARCH_MAKE(12, 0, 1)) {
		/* AMBA_ENABLE present from 12.0.1 */
		u32 val = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(AMBA_ENABLE));

		val = AMBA_ENABLE_COHERENCY_PROTOCOL_SET(val, mode);
		kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(AMBA_ENABLE), val);
	} else {
		/* Fallback to COHERENCY_ENABLE for older versions */
		kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(COHERENCY_ENABLE), mode);
	}
#else /* MALI_USE_CSF */
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(COHERENCY_ENABLE), mode);
#endif /* MALI_USE_CSF */
}

void kbase_amba_set_shareable_cache_support(struct kbase_device *kbdev)
{
#if MALI_USE_CSF

	/* AMBA registers only present from 12.0.1 */
	if (kbdev->gpu_props.gpu_id.arch_id < GPU_ID_ARCH_MAKE(12, 0, 1))
		return;

	if (kbdev->system_coherency != COHERENCY_NONE) {
		u32 val = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(AMBA_FEATURES));

		if (AMBA_FEATURES_SHAREABLE_CACHE_SUPPORT_GET(val)) {
			val = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(AMBA_ENABLE));
			val = AMBA_ENABLE_SHAREABLE_CACHE_SUPPORT_SET(val, 1);
			kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(AMBA_ENABLE), val);
		}
	}
#endif /* MALI_USE_CSF */
}
