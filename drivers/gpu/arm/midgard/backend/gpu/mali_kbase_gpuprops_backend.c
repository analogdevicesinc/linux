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

/*
 * Base kernel property query backend APIs
 */

#include <mali_kbase.h>
#include <device/mali_kbase_device.h>
#include <backend/gpu/mali_kbase_cache_policy_backend.h>
#include <mali_kbase_hwaccess_gpuprops.h>

int kbase_backend_gpuprops_get(struct kbase_device *kbdev, struct kbase_gpuprops_regdump *regdump)
{
	u64 val64;
	int i;
	struct kbase_gpuprops_regdump registers = { 0 };

	/* Fill regdump with the content of the relevant registers */
	registers.gpu_id = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_ID));

	registers.l2_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(L2_FEATURES));

	registers.tiler_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(TILER_FEATURES));
	registers.mem_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(MEM_FEATURES));
	registers.mmu_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(MMU_FEATURES));
	registers.as_present = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(AS_PRESENT));
#if !MALI_USE_CSF
	registers.js_present = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(JS_PRESENT));
#else /* !MALI_USE_CSF */
	registers.js_present = 0;
#endif /* !MALI_USE_CSF */

	for (i = 0; i < GPU_MAX_JOB_SLOTS; i++)
#if !MALI_USE_CSF
		registers.js_features[i] = kbase_reg_read32(kbdev, GPU_JS_FEATURES_OFFSET(i));
#else /* !MALI_USE_CSF */
		registers.js_features[i] = 0;
#endif /* !MALI_USE_CSF */

	for (i = 0; i < BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS; i++)
		registers.texture_features[i] =
			kbase_reg_read32(kbdev, GPU_TEXTURE_FEATURES_OFFSET(i));

	registers.thread_max_threads =
		kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_THREADS));
	registers.thread_max_workgroup_size =
		kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_WORKGROUP_SIZE));
	registers.thread_max_barrier_size =
		kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_MAX_BARRIER_SIZE));
	registers.thread_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_FEATURES));

	/* TODO: Change GPU Props to handle this as a single 64-bit value */
	val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(SHADER_PRESENT));
	registers.shader_present_lo = (u32)(val64 & 0xFFFFFFFF);
	registers.shader_present_hi = (u32)(val64 >> 32);

	val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(TILER_PRESENT));
	registers.tiler_present_lo = (u32)(val64 & 0xFFFFFFFF);
	registers.tiler_present_hi = (u32)(val64 >> 32);

	val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(L2_PRESENT));
	registers.l2_present_lo = (u32)(val64 & 0xFFFFFFFF);
	registers.l2_present_hi = (u32)(val64 >> 32);

	/* Not a valid register on TMIX */
	if (kbase_reg_is_valid(kbdev, GPU_CONTROL_ENUM(STACK_PRESENT))) {
		val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(STACK_PRESENT));
		registers.stack_present_lo = (u32)(val64 & 0xFFFFFFFF);
		registers.stack_present_hi = (u32)(val64 >> 32);
	}

#if MALI_USE_CSF
	if (registers.gpu_id >= GPU_ID2_PRODUCT_MAKE(11, 8, 5, 2)) {
		val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(GPU_FEATURES));
		registers.gpu_features_lo = (u32)(val64 & 0xFFFFFFFF);
		registers.gpu_features_hi = (u32)(val64 >> 32);
	}

#endif /* MALI_USE_CSF */

	if (!kbase_is_gpu_removed(kbdev)) {
		*regdump = registers;
		return 0;
	} else
		return -EIO;
}

int kbase_backend_gpuprops_get_curr_config(struct kbase_device *kbdev,
					   struct kbase_current_config_regdump *curr_config_regdump)
{
	u64 val64;

	if (WARN_ON(!kbdev) || WARN_ON(!curr_config_regdump))
		return -EINVAL;

	curr_config_regdump->mem_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(MEM_FEATURES));

	val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(SHADER_PRESENT));
	curr_config_regdump->shader_present_lo = (u32)(val64 & 0xFFFFFFFF);
	curr_config_regdump->shader_present_hi = (u32)(val64 >> 32);

	val64 = kbase_reg_read64(kbdev, GPU_CONTROL_ENUM(L2_PRESENT));
	curr_config_regdump->l2_present_lo = (u32)(val64 & 0xFFFFFFFF);
	curr_config_regdump->l2_present_hi = (u32)(val64 >> 32);

	if (kbase_is_gpu_removed(kbdev))
		return -EIO;

	return 0;
}

int kbase_backend_gpuprops_get_features(struct kbase_device *kbdev,
					struct kbase_gpuprops_regdump *regdump)
{
	u32 coherency_features;
	int error = 0;

	coherency_features = kbase_cache_get_coherency_features(kbdev);

	if (kbase_is_gpu_removed(kbdev))
		error = -EIO;

	regdump->coherency_features = coherency_features;

	if (kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_CORE_FEATURES))
		regdump->core_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(CORE_FEATURES));
	else
		regdump->core_features = 0;

	regdump->thread_tls_alloc = 0;
#if !MALI_USE_CSF
	if (kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_THREAD_TLS_ALLOC))
		regdump->thread_tls_alloc =
			kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(THREAD_TLS_ALLOC));
#endif

	return error;
}

int kbase_backend_gpuprops_get_l2_features(struct kbase_device *kbdev,
					   struct kbase_gpuprops_regdump *regdump)
{
	if (kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_L2_CONFIG)) {
		u32 l2_features = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(L2_FEATURES));
		u32 l2_config = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(L2_CONFIG));
		u32 slice_hash[GPU_L2_SLICE_HASH_COUNT] = {
			0,
		};
		int i;

#if MALI_USE_CSF
		if (kbase_hw_has_l2_slice_hash_feature(kbdev)) {
			for (i = 0; i < GPU_L2_SLICE_HASH_COUNT; i++)
				slice_hash[i] =
					kbase_reg_read32(kbdev, GPU_L2_SLICE_HASH_OFFSET(i));
		}
#endif /* MALI_USE_CSF */

		if (kbase_is_gpu_removed(kbdev))
			return -EIO;

		regdump->l2_features = l2_features;
		regdump->l2_config = l2_config;
		for (i = 0; i < GPU_L2_SLICE_HASH_COUNT; i++)
			regdump->l2_slice_hash[i] = slice_hash[i];
	}

	return 0;
}
