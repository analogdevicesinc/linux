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

#ifndef _UAPI_MALI_GPUPROPS_H_
#define _UAPI_MALI_GPUPROPS_H_

#include <linux/types.h>
#include "mali_base_common_kernel.h"

#define BASE_MAX_COHERENT_GROUPS 16
#define GPU_MAX_JOB_SLOTS 16

/**
 * struct gpu_props_user_data - structure for gpu props user buffer.
 * @core_props:     Core props.
 * @l2_props:       L2 props.
 * @tiler_props:    Tiler props.
 * @thread_props:   Thread props.
 * @raw_props:      Raw register values kept for backwards compatibility. Kbase
 *                  and base should never reference values within this struct.
 * @coherency_info: Coherency information.
 *
 * This structure is used solely for the encoding and decoding of the prop_buffer
 * returned by kbase.
 */
struct gpu_props_user_data {
	struct {
		__u32 product_id;
		__u16 version_status;
		__u16 minor_revision;
		__u16 major_revision;
		__u32 gpu_freq_khz_max;
		__u32 log2_program_counter_size;
		__u32 texture_features[BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS];
		__u64 gpu_available_memory_size;
		__u8 num_exec_engines;
	} core_props;
	struct {
		__u8 log2_line_size;
		__u8 log2_cache_size;
		__u8 num_l2_slices;
	} l2_props;
	struct {
		__u32 bin_size_bytes;
		__u32 max_active_levels;
	} tiler_props;
	struct {
		__u32 max_threads;
		__u32 max_workgroup_size;
		__u32 max_barrier_size;
		__u32 max_registers;
		__u8 max_task_queue;
		__u8 max_thread_group_split;
		__u8 impl_tech;
		__u32 tls_alloc;
	} thread_props;

	/* kept for backward compatibility, should not be used in the future. */
	struct {
		__u64 shader_present;
		__u64 tiler_present;
		__u64 l2_present;
		__u64 stack_present;
		__u64 l2_features;
		__u64 core_features;
		__u64 mem_features;
		__u64 mmu_features;
		__u32 as_present;
		__u32 js_present;
		__u32 js_features[GPU_MAX_JOB_SLOTS];
		__u64 tiler_features;
		__u32 texture_features[BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS];
		__u64 gpu_id;
		__u32 thread_max_threads;
		__u32 thread_max_workgroup_size;
		__u32 thread_max_barrier_size;
		__u32 thread_features;
		__u32 coherency_mode;
		__u32 thread_tls_alloc;
		__u64 gpu_features;
		__u64 base_present;
		__u64 neural_present;
	} raw_props;
	struct {
		__u32 num_groups;
		__u32 num_core_groups;
		__u32 coherency;
		struct {
			__u64 core_mask;
			__u32 num_cores;
		} group[BASE_MAX_COHERENT_GROUPS];
	} coherency_info;
};

#endif /* _UAPI_MALI_GPUPROPS_H_ */
