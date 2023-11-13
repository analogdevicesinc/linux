/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2011-2023 ARM Limited. All rights reserved.
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

/**
 * DOC: Base kernel property query APIs
 */

#ifndef _KBASE_GPUPROPS_TYPES_H_
#define _KBASE_GPUPROPS_TYPES_H_

#include <uapi/gpu/arm/midgard/mali_base_kernel.h>

#define KBASE_GPU_SPEED_MHZ 123
#define KBASE_GPU_PC_SIZE_LOG2 24U

/**
 * struct kbase_current_config_regdump - Register dump for current resources
 *                                       allocated to the GPU.
 * @mem_features: Memory system features. Contains information about the
 *                features of the memory system. Used here to get the L2 slice
 *                count.
 * @l2_features: L2 cache features
 * @shader_present: Shader core present bitmap.
 * @l2_present: L2 cache present bitmap.
 *
 * Register dump structure used to store the resgisters data realated to the
 * current resources allocated to the GPU.
 */
struct kbase_current_config_regdump {
	u64 mem_features;
	u64 l2_features;
	u64 shader_present;
	u64 l2_present;
};

/**
 * struct kbase_gpu_mmu_props - MMU properties
 * @va_bits: Number of bits supported in virtual addresses
 * @pa_bits: Number of bits supported in physical addresses
 */
struct kbase_gpu_mmu_props {
	u8 va_bits;
	u8 pa_bits;
};

/**
 * struct max_config_props - Properties based on the maximum resources
 *                           available.
 * @l2_slices: Maximum number of L2 slices that can be assinged to the GPU
 *             during runtime.
 * @padding:   Padding to a multiple of 64 bits.
 * @core_mask: Largest core mask bitmap that can be assigned to the GPU during
 *             runtime.
 *
 * Properties based on the maximum resources available (not necessarly
 * allocated at that moment). Used to provide the maximum configuration to the
 * userspace allowing the applications to allocate enough resources in case the
 * real allocated resources change.
 */
struct max_config_props {
	u8 l2_slices;
	u8 padding[3];
	u32 core_mask;
};

/**
 * struct curr_config_props - Properties based on the current resources
 *                            allocated to the GPU.
 * @l2_present:     Current L2 present bitmap that is allocated to the GPU.
 * @shader_present: Current shader present bitmap that is allocated to the GPU.
 * @num_cores:      Current number of shader cores allocated to the GPU.
 * @l2_slices:      Current number of L2 slices allocated to the GPU.
 * @update_needed:  Defines if it is necessary to re-read the registers to
 *                  update the current allocated resources.
 * @padding:        Padding to a multiple of 64 bits.
 *
 * Properties based on the current resource available. Used for operations with
 * hardware interactions to avoid using userspace data that can be based on
 * the maximum resource available.
 */
struct curr_config_props {
	u64 l2_present;
	u64 shader_present;
	u16 num_cores;
	u8 l2_slices;
	bool update_needed;
	u8 padding[4];
};

/**
 * struct kbase_gpu_id_props - Properties based on GPU_ID register.
 * @version_status: field indicating the status of the GPU release
 * @version_minor:  minor release version number (p1 in r0p1)
 * @version_major:  major release version number (r0 in r0p1)
 * @product_major:  product identifier
 * @arch_rev:       architecture patch version
 * @arch_minor:     architecture minor revision
 * @arch_major:     architecture major revision
 * @product_id:     arch_major << 24 | arch_minor << 16 | arch_rev << 8 | product_major
 * @product_model:  arch_major << 24 | product_major
 * @version_id:     version_major << 16 | version_minor << 8 | version_status
 * @arch_id:        id composed of arch_major << 16 | arch_minor << 8 | arch_rev
 *
 * Use GPU_ID_PRODUCT_ID_MAKE, GPU_ID_VERSION_MAKE or GPU_ID_ARCH_MAKE to perform
 * comparisons between product_id, version_id or arch_id respectively
 */
struct kbase_gpu_id_props {
	u16 version_status;
	u16 version_minor;
	u16 version_major;
	u16 product_major;
	u16 arch_rev;
	u16 arch_minor;
	u16 arch_major;
	/* Composite ids */
	u32 product_id;
	u32 product_model;
	u32 version_id;
	u32 arch_id;
};

/**
 * struct kbase_gpu_features_props - boolean struct indicating feature support
 *                                   from GPU_FEATURES register.
 * @ray_intersection:  Ray tracing intersection instructions supported
 * @cross_stream_sync: Cross stream sync supported
 *
 * This register is only present on certain CSF GPUs.
 */
struct kbase_gpu_features_props {
	bool ray_intersection;
	bool cross_stream_sync;
};

/**
 * struct kbase_coherent_group_props - Coherency goup properties
 * @core_mask: Coherent group core mask
 * @num_cores: Number of cores in coherent group
 */
struct kbase_coherent_group_props {
	u64 core_mask;
	u16 num_cores;
};

/**
 * struct kbase_coherency_props - Coherency group information
 * @coherent_core_group: Core group is coherent (MEM_FEATURES register)
 * @coherent_super_group: Core supergroup is coherent (MEM_FEATURES register)
 * @group: Descriptors of coherent groups
 *
 * The groups are sorted by core mask. The core masks are non-repeating and do
 * not intersect.
 */
struct kbase_coherency_props {
	bool coherent_core_group;
	bool coherent_super_group;
	struct kbase_coherent_group_props group;
};

/**
 * struct kbase_js_features_props - Boolean struct of fields in JSn_FEATURES register
 * @null: Supports null jobs
 * @write_value: Supports write value jobs
 * @cache_flush: Supports cache flush jobs
 * @compute_shader: Supports compute shader jobs
 * @tiler: Supports tiler jobs
 * @fragment_shader: Supports fragment shader jobs
 */
struct kbase_js_features_props {
	bool null;
	bool write_value;
	bool cache_flush;
	bool compute_shader;
	bool tiler;
	bool fragment_shader;
};

/**
 * struct kbase_gpu_props - parsed gpu properties used by kbase.
 * @shader_present: Shader core present bitmap
 * @stack_present: Core stack present bitmap
 * @tiler_present: Tiler present bitmap
 * @l2_present: L2 cache present bitmap
 * @num_cores: Number of shader cores present
 * @num_core_groups: Number of L2 cache present
 * @num_address_spaces: Number of address spaces
 * @num_job_slots: Number of job slots
 * @coherency_mode: Coherency mode bitmask
 * @gpu_freq_khz_max: Max configured gpu frequency
 * @log2_program_counter_size: Program counter size in log2
 * @log2_line_size: L2 cache line size in log2
 * @num_l2_slices: Number of l2 slices
 * @max_threads: Total number of registers per core
 * @impl_tech: Implementation technology type
 * @js_features: Job slot features
 * @gpu_id: struct kbase_gpu_id_props
 * @gpu_features: struct kbase_gpu_features_props
 * @coherency_info: struct kbase_coherency_props
 * @mmu: MMU props
 * @curr_config: struct curr_config_props current resource available
 * @max_config: struct max_config_props maximum resource available
 * @prop_buffer_size: prop_buffer size
 * @prop_buffer: buffer containing encoded gpu props for userspace
 * @priv_data: private data structure freed after kbase_gpuprops_populate_user_buffer()
 *
 * @note Structure should be kbase specific, it should not contain userspace (e.g. base)
 *       structures nor should it ever contain raw register values unless it is
 *       a bitmask (e.g. shader_present, stack_present).
 */
struct kbase_gpu_props {
	/* kernel-only properties */
	u64 shader_present;
	u64 stack_present;
	u64 tiler_present;
	u64 l2_present;

	u8 num_cores;
	u8 num_core_groups;
	u8 num_address_spaces;
	u8 num_job_slots;

	u32 coherency_mode;
	u32 gpu_freq_khz_max;
	u32 log2_program_counter_size;

	u8 log2_line_size;
	u8 num_l2_slices;

	u32 max_threads;
	u8 impl_tech;

	struct kbase_js_features_props js_features[GPU_MAX_JOB_SLOTS];

	struct kbase_gpu_id_props gpu_id;

	struct kbase_gpu_features_props gpu_features;

	struct kbase_coherency_props coherency_info;

	struct kbase_gpu_mmu_props mmu;

	/* Properties based on the current resource available */
	struct curr_config_props curr_config;

	/* Properties based on the maximum resource available */
	struct max_config_props max_config;

	u32 prop_buffer_size;
	void *prop_buffer;
	void *priv_data;
};

#endif /* _KBASE_GPUPROPS_TYPES_H_ */
