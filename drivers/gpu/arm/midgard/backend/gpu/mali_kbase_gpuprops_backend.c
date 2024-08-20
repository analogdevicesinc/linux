// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2014-2024 ARM Limited. All rights reserved.
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

/*
 * Base kernel property query backend APIs
 */

#include <mali_kbase.h>
#include <device/mali_kbase_device.h>
#include <mali_kbase_hwaccess_gpuprops.h>
#include <mali_kbase_gpuprops_private_types.h>

int kbase_backend_gpuprops_get(struct kbase_device *kbdev, struct kbasep_gpuprops_regdump *regdump)
{
	uint i;

	/* regdump is zero intiialized, individual entries do not need to be explicitly set */
	regdump->gpu_id = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(GPU_ID));

	regdump->shader_present = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(SHADER_PRESENT));
	regdump->tiler_present = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(TILER_PRESENT));
	regdump->l2_present = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(L2_PRESENT));
	if (kbase_reg_is_valid(kbdev, GPU_CONTROL_ENUM(AS_PRESENT)))
		regdump->as_present = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(AS_PRESENT));
	if (kbase_reg_is_valid(kbdev, GPU_CONTROL_ENUM(STACK_PRESENT)))
		regdump->stack_present = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(STACK_PRESENT));

#if !MALI_USE_CSF
	regdump->js_present = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(JS_PRESENT));
	/* Not a valid register on TMIX */

	/* TGOx specific register */
	if (kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_THREAD_TLS_ALLOC))
		regdump->thread_tls_alloc =
			kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_TLS_ALLOC));
#endif /* !MALI_USE_CSF */

	regdump->thread_max_threads = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_THREADS));
	if (kbase_reg_is_valid(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_WORKGROUP_SIZE)))
		regdump->thread_max_workgroup_size =
			kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_WORKGROUP_SIZE));
#if MALI_USE_CSF
#endif /* MALI_USE_CSF */
	regdump->thread_max_barrier_size =
		kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_BARRIER_SIZE));
	regdump->thread_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_FEATURES));

	/* Feature Registers */
	/* AMBA_FEATURES enum is mapped to COHERENCY_FEATURES enum */
	regdump->coherency_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(COHERENCY_FEATURES));

	if (kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_CORE_FEATURES))
		regdump->core_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(CORE_FEATURES));

#if MALI_USE_CSF
	if (kbase_reg_is_valid(kbdev, GPU_CONTROL_ENUM(GPU_FEATURES)))
		regdump->gpu_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(GPU_FEATURES));
#endif /* MALI_USE_CSF */

	regdump->tiler_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(TILER_FEATURES));
	regdump->l2_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(L2_FEATURES));
	regdump->mem_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(MEM_FEATURES));
	regdump->mmu_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(MMU_FEATURES));

#if !MALI_USE_CSF
	for (i = 0; i < GPU_MAX_JOB_SLOTS; i++)
		regdump->js_features[i] = kbase_reg_read32(kbdev, GPU_JS_FEATURES_OFFSET(i));
#endif /* !MALI_USE_CSF */

#if MALI_USE_CSF
#endif /* MALI_USE_CSF */
	{
		for (i = 0; i < BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS; i++)
			regdump->texture_features[i] =
				kbase_reg_read32(kbdev, GPU_TEXTURE_FEATURES_OFFSET(i));
	}

	if (kbase_is_gpu_removed(kbdev))
		return -EIO;
	return 0;
}

int kbase_backend_gpuprops_get_curr_config(struct kbase_device *kbdev,
					   struct kbase_current_config_regdump *curr_config_regdump)
{
	if (WARN_ON(!kbdev) || WARN_ON(!curr_config_regdump))
		return -EINVAL;

	curr_config_regdump->mem_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(MEM_FEATURES));
	curr_config_regdump->l2_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(L2_FEATURES));
	curr_config_regdump->shader_present =
		kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(SHADER_PRESENT));
	curr_config_regdump->l2_present = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(L2_PRESENT));

	if (kbase_is_gpu_removed(kbdev))
		return -EIO;

	return 0;
}

int kbase_backend_gpuprops_get_l2_features(struct kbase_device *kbdev,
					   struct kbasep_gpuprops_regdump *regdump)
{
	if (kbase_hw_has_feature(kbdev, KBASE_HW_FEATURE_L2_CONFIG)) {
		regdump->l2_features = KBASE_REG_READ(kbdev, GPU_CONTROL_ENUM(L2_FEATURES));
		regdump->l2_config = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(L2_CONFIG));

#if MALI_USE_CSF
		if (kbase_hw_has_l2_slice_hash_feature(kbdev)) {
			uint i;
			for (i = 0; i < GPU_L2_SLICE_HASH_COUNT; i++)
				regdump->l2_slice_hash[i] =
					kbase_reg_read32(kbdev, GPU_L2_SLICE_HASH_OFFSET(i));
		}
#endif /* MALI_USE_CSF */

		if (kbase_is_gpu_removed(kbdev))
			return -EIO;
	}

	return 0;
}
