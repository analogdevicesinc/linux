/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023-2024 ARM Limited. All rights reserved.
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

#ifndef _MALI_KBASE_GPUPROPS_PRIVATE_TYPES_H_
#define _MALI_KBASE_GPUPROPS_PRIVATE_TYPES_H_

#include <uapi/gpu/arm/midgard/mali_base_kernel.h>

/**
 * struct kbasep_gpuprops_regdump - structure containing raw GPU register values.
 *
 * @shader_present: Shader core present bitmap
 * @tiler_present: Tiler core present bitmap
 * @l2_present: L2 cache present bitmap
 * @stack_present: Core stack present bitmap
 * @core_features: Shader core features
 * @tiler_features: Tiler features
 * @l2_features: Level 2 cache features
 * @mem_features: Memory system features
 * @mmu_features: GPU Memory Management Unit configuration
 * @gpu_features: Supported GPU features
 * @as_present: Address spaces present
 * @js_present: Job slots present
 * @js_features: Job slot features
 * @texture_features: Support flags for texture formats
 * @gpu_id: GPU ID
 * @thread_max_threads: Maximum number of threads per core
 * @thread_max_workgroup_size: Maximum number of threads per workgroup
 * @thread_max_barrier_size: Maximum number of threads per barrier
 * @thread_features: Thread features
 * @coherency_features: Coherency/AMBA features
 * @thread_tls_alloc: Number of threads per core to allocate TLS storage for
 * @l2_config: Level 2 cache configuration
 * @l2_slice_hash: ASN Hash function arguments
 * @base_present: Shader core base present bitmap
 * @neural_present: Neural engine present bitmap
 *
 * This structure is used to store raw GPU register values that will be used as-is
 * or parsed into respective properties.
 */
struct kbasep_gpuprops_regdump {
	u64 shader_present;
	u64 tiler_present;
	u64 l2_present;
	u64 stack_present;
	u64 core_features;
	u64 tiler_features;
	u64 l2_features;
	u64 mem_features;
	u64 mmu_features;
	u64 gpu_features;
	u32 as_present;
	u32 js_present;
	u32 js_features[GPU_MAX_JOB_SLOTS];
	u32 texture_features[BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS];
	u64 gpu_id;
	u32 thread_max_threads;
	u32 thread_max_workgroup_size;
	u32 thread_max_barrier_size;
	u32 thread_features;
	u32 coherency_features;
	u32 thread_tls_alloc;
	u32 l2_config;
	u32 l2_slice_hash[GPU_ASN_HASH_COUNT];
	u64 base_present;
	u64 neural_present;
};

struct kbasep_gpuprops_priv_data {
	struct kbasep_gpuprops_regdump regdump;
};

#endif /* _MALI_KBASE_GPUPROPS_PRIVATE_TYPES_H_ */
