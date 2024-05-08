// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2024 ARM Limited. All rights reserved.
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

#include <mali_kbase.h>
#include "hwcnt/mali_kbase_hwcnt_gpu.h"

#include <linux/err.h>
#include <linux/log2.h>

/** enum enable_map_idx - index into a block enable map that spans multiple u64 array elements
 */
enum enable_map_idx {
	EM_LO,
	EM_HI,
	EM_COUNT,
};

static enum kbase_hwcnt_gpu_v5_block_type kbasep_get_fe_block_type(enum kbase_hwcnt_set counter_set,
								   bool is_csf)
{
	switch (counter_set) {
	case KBASE_HWCNT_SET_PRIMARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE;
	case KBASE_HWCNT_SET_SECONDARY:
		if (is_csf)
			return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE2;
		else
			return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE_UNDEFINED;
	case KBASE_HWCNT_SET_TERTIARY:
		if (is_csf)
			return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE3;
		else
			return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE_UNDEFINED;
	default:
		WARN(true, "Invalid counter set for FE block type: %d", counter_set);
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE_UNDEFINED;
	}
}

static enum kbase_hwcnt_gpu_v5_block_type
kbasep_get_tiler_block_type(enum kbase_hwcnt_set counter_set)
{
	switch (counter_set) {
	case KBASE_HWCNT_SET_PRIMARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER;
	case KBASE_HWCNT_SET_SECONDARY:
	case KBASE_HWCNT_SET_TERTIARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER_UNDEFINED;
	default:
		WARN(true, "Invalid counter set for tiler block type: %d", counter_set);
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER_UNDEFINED;
	}
}

static enum kbase_hwcnt_gpu_v5_block_type kbasep_get_sc_block_type(enum kbase_hwcnt_set counter_set,
								   bool is_csf)
{
	switch (counter_set) {
	case KBASE_HWCNT_SET_PRIMARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC;
	case KBASE_HWCNT_SET_SECONDARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC2;
	case KBASE_HWCNT_SET_TERTIARY:
		if (is_csf)
			return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC3;
		else
			return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC_UNDEFINED;
	default:
		WARN(true, "Invalid counter set for shader core block type: %d", counter_set);
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC_UNDEFINED;
	}
}


static enum kbase_hwcnt_gpu_v5_block_type
kbasep_get_memsys_block_type(enum kbase_hwcnt_set counter_set)
{
	switch (counter_set) {
	case KBASE_HWCNT_SET_PRIMARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS;
	case KBASE_HWCNT_SET_SECONDARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS2;
	case KBASE_HWCNT_SET_TERTIARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS_UNDEFINED;
	default:
		WARN(true, "Invalid counter set for Memsys block type: %d", counter_set);
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS_UNDEFINED;
	}
}

static enum kbase_hwcnt_gpu_v5_block_type kbasep_get_fw_block_type(enum kbase_hwcnt_set counter_set)
{
	switch (counter_set) {
	case KBASE_HWCNT_SET_PRIMARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW;
	case KBASE_HWCNT_SET_SECONDARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW2;
	case KBASE_HWCNT_SET_TERTIARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW3;
	default:
		WARN(true, "Invalid counter set for FW type: %d", counter_set);
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW_UNDEFINED;
	}
}

static enum kbase_hwcnt_gpu_v5_block_type
kbasep_get_csg_block_type(enum kbase_hwcnt_set counter_set)
{
	switch (counter_set) {
	case KBASE_HWCNT_SET_PRIMARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG;
	case KBASE_HWCNT_SET_SECONDARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG2;
	case KBASE_HWCNT_SET_TERTIARY:
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG3;
	default:
		WARN(true, "Invalid counter set for CSG type: %d", counter_set);
		return KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG_UNDEFINED;
	}
}

/**
 * kbasep_hwcnt_backend_gpu_metadata_create() - Create hardware counter metadata
 *                                              for the GPU.
 * @gpu_info:      Non-NULL pointer to hwcnt info for current GPU.
 * @is_csf:        true for CSF GPU, otherwise false.
 * @counter_set:   The performance counter set to use.
 * @metadata:      Non-NULL pointer to where created metadata is stored
 *                 on success.
 *
 * Return: 0 on success, else error code.
 */
static int kbasep_hwcnt_backend_gpu_metadata_create(const struct kbase_hwcnt_gpu_info *gpu_info,
						    const bool is_csf,
						    enum kbase_hwcnt_set counter_set,
						    const struct kbase_hwcnt_metadata **metadata)
{
	struct kbase_hwcnt_description desc;
	struct kbase_hwcnt_block_description blks[KBASE_HWCNT_V5_BLOCK_TYPE_COUNT] = {};
	size_t non_core_block_count;
	size_t core_block_count;
	size_t sc_block_count;
	size_t blk_idx = 0;

	if (WARN_ON(!gpu_info))
		return -EINVAL;

	if (WARN_ON(!metadata))
		return -EINVAL;

	/* Calculate number of block instances that aren't cores */
	non_core_block_count = 2 + gpu_info->l2_count;
	/* Calculate number of block instances that are shader cores */
	sc_block_count = (size_t)fls64(gpu_info->sc_core_mask);
	/* Determine the total number of cores */
	core_block_count = sc_block_count;


	if (gpu_info->has_fw_counters)
		non_core_block_count += 1 + gpu_info->csg_cnt;

	/*
	 * Check we have enough bits to represent the number of cores that
	 * exist in the system. Error-out if there are more blocks than our implementation can
	 * support.
	 */
	if ((core_block_count + non_core_block_count) > KBASE_HWCNT_AVAIL_MASK_BITS)
		return -EINVAL;

	/* The dump starts with, on supporting systems, the FW blocks, and as such,
	 * they should be taken into account first.
	 */
	if (gpu_info->has_fw_counters) {
		blks[blk_idx++] = (struct kbase_hwcnt_block_description){
			.type = kbasep_get_fw_block_type(counter_set),
			.inst_cnt = 1,
			.hdr_cnt = KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
			.ctr_cnt = gpu_info->prfcnt_values_per_block -
				   KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
		};
	}

	/* Some systems may support FW counters but not CSG counters, so the
	 * two are handled differently.
	 */
	if (gpu_info->csg_cnt > 0) {
		blks[blk_idx++] = (struct kbase_hwcnt_block_description){
			.type = kbasep_get_csg_block_type(counter_set),
			.inst_cnt = gpu_info->csg_cnt,
			.hdr_cnt = KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
			.ctr_cnt = gpu_info->prfcnt_values_per_block -
				   KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
		};
	}

	/* One Front End block */
	blks[blk_idx++] = (struct kbase_hwcnt_block_description){
		.type = kbasep_get_fe_block_type(counter_set, is_csf),
		.inst_cnt = 1,
		.hdr_cnt = KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
		.ctr_cnt = gpu_info->prfcnt_values_per_block - KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
	};

	/* One Tiler block */
	blks[blk_idx++] = (struct kbase_hwcnt_block_description){
		.type = kbasep_get_tiler_block_type(counter_set),
		.inst_cnt = 1,
		.hdr_cnt = KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
		.ctr_cnt = gpu_info->prfcnt_values_per_block - KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
	};

	/* l2_count memsys blks */
	blks[blk_idx++] = (struct kbase_hwcnt_block_description){
		.type = kbasep_get_memsys_block_type(counter_set),
		.inst_cnt = gpu_info->l2_count,
		.hdr_cnt = KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
		.ctr_cnt = gpu_info->prfcnt_values_per_block - KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
	};

	/*
	 * There are as many cores in the system as there are bits set in
	 * the core mask. However, the dump buffer memory requirements need to
	 * take into account the fact that the core mask may be non-contiguous.
	 *
	 * For example, a system with a core mask of 0b1011 has the same dump
	 * buffer memory requirements as a system with 0b1111, but requires more
	 * memory than a system with 0b0111. However, core 2 of the system with
	 * 0b1011 doesn't physically exist, and the dump buffer memory that
	 * accounts for that core will never be written to when we do a counter
	 * dump.
	 *
	 * We find the core mask's last set bit to determine the memory
	 * requirements, and embed the core mask into the availability mask so
	 * we can determine later which cores physically exist.
	 */
	blks[blk_idx++] = (struct kbase_hwcnt_block_description){
		.type = kbasep_get_sc_block_type(counter_set, is_csf),
		.inst_cnt = sc_block_count,
		.hdr_cnt = KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
		.ctr_cnt = gpu_info->prfcnt_values_per_block - KBASE_HWCNT_V5_HEADERS_PER_BLOCK,
	};


	/* Currently, we're only handling a maximum of seven blocks, and this needs
	 * to be changed whenever the number of blocks increases
	 */
	BUILD_BUG_ON(KBASE_HWCNT_V5_BLOCK_TYPE_COUNT != 7);

	/* After assembling the block list in the code above, we should not end up with more
	 * elements than KBASE_HWCNT_V5_BLOCK_TYPE_COUNT.
	 */
	WARN_ON(blk_idx > KBASE_HWCNT_V5_BLOCK_TYPE_COUNT);

	desc.blk_cnt = blk_idx;
	desc.blks = blks;
	desc.clk_cnt = gpu_info->clk_cnt;

	/* The JM, Tiler, and L2s are always available, and are before cores */
	kbase_hwcnt_set_avail_mask(&desc.avail_mask, 0, 0);
	kbase_hwcnt_set_avail_mask_bits(&desc.avail_mask, 0, non_core_block_count, U64_MAX);
	kbase_hwcnt_set_avail_mask_bits(&desc.avail_mask, non_core_block_count, sc_block_count,
					gpu_info->sc_core_mask);


	return kbase_hwcnt_metadata_create(&desc, metadata);
}

/**
 * kbasep_hwcnt_backend_jm_dump_bytes() - Get the raw dump buffer size for the
 *                                        GPU.
 * @gpu_info: Non-NULL pointer to hwcnt info for the GPU.
 *
 * Return: Size of buffer the GPU needs to perform a counter dump.
 */
static size_t kbasep_hwcnt_backend_jm_dump_bytes(const struct kbase_hwcnt_gpu_info *gpu_info)
{
	WARN_ON(!gpu_info);

	return (2 + gpu_info->l2_count + (size_t)fls64(gpu_info->sc_core_mask)) *
	       gpu_info->prfcnt_values_per_block * KBASE_HWCNT_VALUE_HW_BYTES;
}

int kbase_hwcnt_jm_metadata_create(const struct kbase_hwcnt_gpu_info *gpu_info,
				   enum kbase_hwcnt_set counter_set,
				   const struct kbase_hwcnt_metadata **out_metadata,
				   size_t *out_dump_bytes)
{
	int errcode;
	const struct kbase_hwcnt_metadata *metadata;
	size_t dump_bytes;

	if (!gpu_info || !out_metadata || !out_dump_bytes)
		return -EINVAL;

	/*
	 * For architectures where a max_config interface is available
	 * from the arbiter, the v5 dump bytes and the metadata v5 are
	 * based on the maximum possible allocation of the HW in the
	 * GPU cause it needs to be prepared for the worst case where
	 * all the available L2 cache and Shader cores are allocated.
	 */
	dump_bytes = kbasep_hwcnt_backend_jm_dump_bytes(gpu_info);
	errcode = kbasep_hwcnt_backend_gpu_metadata_create(gpu_info, false, counter_set, &metadata);
	if (errcode)
		return errcode;

	/*
	 * The physical dump size should be half of dump abstraction size in
	 * metadata since physical HW uses 32-bit per value but metadata
	 * specifies 64-bit per value.
	 */
	if (WARN(dump_bytes * 2 != metadata->dump_buf_bytes,
		 "Dump buffer size expected to be %zu, instead is %zu", dump_bytes * 2,
		 metadata->dump_buf_bytes))
		return -EINVAL;

	*out_metadata = metadata;
	*out_dump_bytes = dump_bytes;

	return 0;
}

int kbase_hwcnt_csf_metadata_create(const struct kbase_hwcnt_gpu_info *gpu_info,
				    enum kbase_hwcnt_set counter_set,
				    const struct kbase_hwcnt_metadata **out_metadata)
{
	int errcode;
	const struct kbase_hwcnt_metadata *metadata;

	if (!gpu_info || !out_metadata)
		return -EINVAL;

	errcode = kbasep_hwcnt_backend_gpu_metadata_create(gpu_info, true, counter_set, &metadata);
	if (errcode)
		return errcode;

	*out_metadata = metadata;

	return 0;
}

bool kbase_hwcnt_is_block_type_shader(const enum kbase_hwcnt_gpu_v5_block_type blk_type)
{
	if (blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC2 ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC3 ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC_UNDEFINED)
		return true;

	return false;
}


bool kbase_hwcnt_is_block_type_memsys(const enum kbase_hwcnt_gpu_v5_block_type blk_type)
{
	if (blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS2 ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS_UNDEFINED)
		return true;

	return false;
}

bool kbase_hwcnt_is_block_type_tiler(const enum kbase_hwcnt_gpu_v5_block_type blk_type)
{
	if (blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER_UNDEFINED)
		return true;

	return false;
}

bool kbase_hwcnt_is_block_type_fe(const enum kbase_hwcnt_gpu_v5_block_type blk_type)
{
	if (blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE2 ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE3 ||
	    blk_type == KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE_UNDEFINED)
		return true;

	return false;
}

int kbase_hwcnt_jm_dump_get(struct kbase_hwcnt_dump_buffer *dst, u64 *src,
			    const struct kbase_hwcnt_enable_map *dst_enable_map, u64 pm_core_mask,
			    u64 debug_core_mask, size_t max_l2_slices,
			    const struct kbase_hwcnt_curr_config *curr_config, bool accumulate)
{
	const struct kbase_hwcnt_metadata *metadata;
	size_t blk, blk_inst;
	const u64 *dump_src = src;
	size_t src_offset = 0;
	u64 core_mask = pm_core_mask;
	u64 shader_present = curr_config->shader_present;

	/* Variables to deal with the current configuration */
	size_t l2_count = 0;

	if (!dst || !src || !dst_enable_map || (dst_enable_map->metadata != dst->metadata))
		return -EINVAL;

	metadata = dst->metadata;

	kbase_hwcnt_metadata_for_each_block(metadata, blk, blk_inst) {
		const size_t hdr_cnt = kbase_hwcnt_metadata_block_headers_count(metadata, blk);
		const size_t ctr_cnt = kbase_hwcnt_metadata_block_counters_count(metadata, blk);
		const u64 blk_type = kbase_hwcnt_metadata_block_type(metadata, blk);
		const bool is_shader_core = kbase_hwcnt_is_block_type_shader(blk_type);
		const bool is_l2_cache = kbase_hwcnt_is_block_type_memsys(blk_type);
		const bool is_undefined = kbase_hwcnt_is_block_type_undefined(blk_type);
		blk_stt_t *dst_blk_stt =
			kbase_hwcnt_dump_buffer_block_state_instance(dst, blk, blk_inst);
		bool hw_res_available = true;

		/*
		 * If l2 blocks is greater than the current allocated number of
		 * L2 slices, there is no hw allocated to that block.
		 */
		if (is_l2_cache) {
			l2_count++;
			if (l2_count > curr_config->num_l2_slices)
				hw_res_available = false;
			else
				hw_res_available = true;
		}
		/*
		 * For the shader cores, the current shader_mask allocated is
		 * always a subgroup of the maximum shader_mask, so after
		 * jumping any L2 cache not available the available shader cores
		 * will always have a matching set of blk instances available to
		 * accumulate them.
		 */
		else
			hw_res_available = true;

		/* Skip block if no values in the destination block are enabled. */
		if (kbase_hwcnt_enable_map_block_enabled(dst_enable_map, blk, blk_inst)) {
			u64 *dst_blk = kbase_hwcnt_dump_buffer_block_instance(dst, blk, blk_inst);
			const u64 *src_blk = dump_src + src_offset;
			bool blk_valid = (!is_undefined && hw_res_available);

			if (blk_valid) {
				bool blk_powered;
				blk_stt_t current_block_state = 0;

				if (!is_shader_core) {
					/* The L2 block must be available at this point, or handled
					 * differently below.
					 * Every partition must have a FE and a tiler, so they
					 * must be implicitly available as part of the current
					 * configuration.
					 */
					blk_powered = true;
					current_block_state |= KBASE_HWCNT_STATE_AVAILABLE;
				} else {
					/* Check the PM core mask to see if the shader core is
					 * powered up.
					 */
					blk_powered = core_mask & 1;

					/* Set availability bits based on whether the core is
					 * present in both the shader_present AND the core
					 * mask in sysFS. The core masks are shifted to the
					 * right at the end of the loop so always check the
					 * rightmost bit.
					 */
					if ((shader_present & debug_core_mask) & 0x1)
						current_block_state |= KBASE_HWCNT_STATE_AVAILABLE;
					else {
						/* If this branch is taken, the shader core may
						 * be:
						 * * in the max configuration, but not enabled
						 * through the sysFS core mask
						 * * in the max configuration, but not in the
						 * current configuration
						 * * physically not present
						 */
						current_block_state |=
							KBASE_HWCNT_STATE_UNAVAILABLE;
					}
				}

				/* Note: KBASE_HWCNT_STATE_OFF for non-shader cores (L2, Tiler, JM)
				 * is handled on this backend's dump_disable function (since
				 * they are considered to always be powered here).
				 */
				current_block_state |= (blk_powered) ? KBASE_HWCNT_STATE_ON :
									     KBASE_HWCNT_STATE_OFF;

				if (accumulate) {
					/* Only update existing counter values if block was powered
					 * and valid
					 */
					if (blk_powered)
						kbase_hwcnt_dump_buffer_block_accumulate(
							dst_blk, src_blk, hdr_cnt, ctr_cnt);

					kbase_hwcnt_block_state_append(dst_blk_stt,
								       current_block_state);
				} else {
					if (blk_powered) {
						kbase_hwcnt_dump_buffer_block_copy(
							dst_blk, src_blk, (hdr_cnt + ctr_cnt));
					} else {
						/* src is garbage, so zero the dst */
						kbase_hwcnt_dump_buffer_block_zero(
							dst_blk, (hdr_cnt + ctr_cnt));
					}

					kbase_hwcnt_block_state_set(dst_blk_stt,
								    current_block_state);
				}
			} else if (is_l2_cache && !is_undefined) {
				/* Defined L2 can only reach here when the partition does not
				 * own it. Check that the L2 count is within the resource
				 * group or whole GPU's max L2 count, and if so,
				 * mark it as unavailable.
				 */
				if (l2_count <= max_l2_slices) {
					kbase_hwcnt_block_state_set(
						dst_blk_stt, KBASE_HWCNT_STATE_OFF |
								     KBASE_HWCNT_STATE_UNAVAILABLE);
				}
				kbase_hwcnt_dump_buffer_block_zero(dst_blk, (hdr_cnt + ctr_cnt));
			} else {
				/* Even though the block is undefined, the user has
				 * enabled counter collection for it. We should not propagate
				 * garbage data, or copy/accumulate the block states.
				 */
				if (accumulate) {
					/* No-op to preserve existing values */
				} else {
					/* src is garbage, so zero the dst and reset block state */
					kbase_hwcnt_dump_buffer_block_zero(dst_blk,
									   (hdr_cnt + ctr_cnt));
					kbase_hwcnt_block_state_set(dst_blk_stt,
								    KBASE_HWCNT_STATE_UNKNOWN);
				}
			}
		}

		/* Just increase the src_offset if the HW is available */
		if (hw_res_available)
			src_offset += (hdr_cnt + ctr_cnt);
		if (is_shader_core) {
			/* Shift each core mask right by 1 */
			core_mask >>= 1;
			debug_core_mask >>= 1;
			shader_present >>= 1;
		}
	}

	return 0;
}

int kbase_hwcnt_csf_dump_get(struct kbase_hwcnt_dump_buffer *dst, u64 *src,
			     blk_stt_t *src_block_stt,
			     const struct kbase_hwcnt_enable_map *dst_enable_map,
			     size_t num_l2_slices, u64 powered_shader_core_mask, bool accumulate)
{
	const struct kbase_hwcnt_metadata *metadata;
	const u64 *dump_src = src;
	size_t src_offset = 0;
	size_t blk, blk_inst;
	size_t blk_inst_count = 0;

	if (!dst || !src || !src_block_stt || !dst_enable_map ||
	    (dst_enable_map->metadata != dst->metadata))
		return -EINVAL;

	metadata = dst->metadata;

	kbase_hwcnt_metadata_for_each_block(metadata, blk, blk_inst) {
		const size_t hdr_cnt = kbase_hwcnt_metadata_block_headers_count(metadata, blk);
		const size_t ctr_cnt = kbase_hwcnt_metadata_block_counters_count(metadata, blk);
		const uint64_t blk_type = kbase_hwcnt_metadata_block_type(metadata, blk);
		const bool is_undefined = kbase_hwcnt_is_block_type_undefined(blk_type);
		blk_stt_t *dst_blk_stt =
			kbase_hwcnt_dump_buffer_block_state_instance(dst, blk, blk_inst);

		/* Skip block if no values in the destination block are enabled. */
		if (kbase_hwcnt_enable_map_block_enabled(dst_enable_map, blk, blk_inst)) {
			u64 *dst_blk = kbase_hwcnt_dump_buffer_block_instance(dst, blk, blk_inst);
			const u64 *src_blk = dump_src + src_offset;

			if (!is_undefined) {
				if (accumulate) {
					kbase_hwcnt_dump_buffer_block_accumulate(dst_blk, src_blk,
										 hdr_cnt, ctr_cnt);
					kbase_hwcnt_block_state_append(
						dst_blk_stt, src_block_stt[blk_inst_count]);
				} else {
					kbase_hwcnt_dump_buffer_block_copy(dst_blk, src_blk,
									   (hdr_cnt + ctr_cnt));
					kbase_hwcnt_block_state_set(dst_blk_stt,
								    src_block_stt[blk_inst_count]);
				}
			} else {
				/* Even though the block might be undefined, the user has enabled
				 * counter collection for it. We should not propagate garbage
				 * data, or copy/accumulate the block states.
				 */
				if (accumulate) {
					/* No-op to preserve existing values */
				} else {
					/* src is garbage, so zero the dst and reset block state */
					kbase_hwcnt_dump_buffer_block_zero(dst_blk,
									   (hdr_cnt + ctr_cnt));
					kbase_hwcnt_block_state_set(dst_blk_stt,
								    KBASE_HWCNT_STATE_UNKNOWN);
				}
			}
		}
		blk_inst_count++;
		src_offset += (hdr_cnt + ctr_cnt);
	}

	return 0;
}

/**
 * kbasep_hwcnt_backend_gpu_block_map_from_physical() - Convert from a physical
 *                                                      block enable map to a
 *                                                      block enable map
 *                                                      abstraction.
 * @phys: Physical 32-bit block enable map
 * @lo:   Non-NULL pointer to where low 64 bits of block enable map abstraction
 *        will be stored.
 * @hi:   Non-NULL pointer to where high 64 bits of block enable map abstraction
 *        will be stored.
 */
static inline void kbasep_hwcnt_backend_gpu_block_map_from_physical(u32 phys, u64 *lo, u64 *hi)
{
	u64 dwords[2] = { 0, 0 };

	size_t dword_idx;

	for (dword_idx = 0; dword_idx < 2; dword_idx++) {
		const u16 packed = phys >> (16 * dword_idx);
		u64 dword = 0;

		size_t hword_bit;

		for (hword_bit = 0; hword_bit < 16; hword_bit++) {
			const size_t dword_bit = hword_bit * 4;
			const u64 mask = (packed >> (hword_bit)) & 0x1;

			dword |= mask << (dword_bit + 0);
			dword |= mask << (dword_bit + 1);
			dword |= mask << (dword_bit + 2);
			dword |= mask << (dword_bit + 3);
		}
		dwords[dword_idx] = dword;
	}
	*lo = dwords[0];
	*hi = dwords[1];
}

void kbase_hwcnt_gpu_enable_map_to_physical(struct kbase_hwcnt_physical_enable_map *dst,
					    const struct kbase_hwcnt_enable_map *src)
{
	const struct kbase_hwcnt_metadata *metadata;
	u64 fe_bm[EM_COUNT] = { 0 };
	u64 shader_bm[EM_COUNT] = { 0 };
	u64 tiler_bm[EM_COUNT] = { 0 };
	u64 mmu_l2_bm[EM_COUNT] = { 0 };
	u64 fw_bm[EM_COUNT] = { 0 };
	u64 csg_bm[EM_COUNT] = { 0 };
	size_t blk, blk_inst;

	if (WARN_ON(!src) || WARN_ON(!dst))
		return;

	metadata = src->metadata;

	kbase_hwcnt_metadata_for_each_block(metadata, blk, blk_inst) {
		const u64 blk_type = kbase_hwcnt_metadata_block_type(metadata, blk);
		const u64 *blk_map = kbase_hwcnt_enable_map_block_instance(src, blk, blk_inst);
		const size_t map_stride =
			kbase_hwcnt_metadata_block_enable_map_stride(metadata, blk);
		size_t map_idx;

		for (map_idx = 0; map_idx < map_stride; ++map_idx) {
			if (WARN_ON(map_idx >= EM_COUNT))
				break;

			switch ((enum kbase_hwcnt_gpu_v5_block_type)blk_type) {
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG_UNDEFINED:
				/* Nothing to do in this case. */
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE3:
				fe_bm[map_idx] |= blk_map[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER:
				tiler_bm[map_idx] |= blk_map[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC3:
				shader_bm[map_idx] |= blk_map[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS2:
				mmu_l2_bm[map_idx] |= blk_map[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW3:
				fw_bm[map_idx] |= blk_map[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG3:
				csg_bm[map_idx] |= blk_map[map_idx];
				break;
			default:
				WARN(true, "Unknown block type %llu", blk_type);
			}
		}
	}

	dst->fe_bm = kbase_hwcnt_backend_gpu_block_map_to_physical(fe_bm[EM_LO], fe_bm[EM_HI]);
	dst->shader_bm =
		kbase_hwcnt_backend_gpu_block_map_to_physical(shader_bm[EM_LO], shader_bm[EM_HI]);
	dst->tiler_bm =
		kbase_hwcnt_backend_gpu_block_map_to_physical(tiler_bm[EM_LO], tiler_bm[EM_HI]);
	dst->mmu_l2_bm =
		kbase_hwcnt_backend_gpu_block_map_to_physical(mmu_l2_bm[EM_LO], mmu_l2_bm[EM_HI]);
	dst->fw_bm = kbase_hwcnt_backend_gpu_block_map_to_physical(fw_bm[EM_LO], fw_bm[EM_HI]);
	dst->csg_bm = kbase_hwcnt_backend_gpu_block_map_to_physical(csg_bm[EM_LO], csg_bm[EM_HI]);
}

void kbase_hwcnt_gpu_set_to_physical(enum kbase_hwcnt_physical_set *dst, enum kbase_hwcnt_set src)
{
	switch (src) {
	case KBASE_HWCNT_SET_PRIMARY:
		*dst = KBASE_HWCNT_PHYSICAL_SET_PRIMARY;
		break;
	case KBASE_HWCNT_SET_SECONDARY:
		*dst = KBASE_HWCNT_PHYSICAL_SET_SECONDARY;
		break;
	case KBASE_HWCNT_SET_TERTIARY:
		*dst = KBASE_HWCNT_PHYSICAL_SET_TERTIARY;
		break;
	default:
		WARN_ON(true);
	}
}

void kbase_hwcnt_gpu_enable_map_from_physical(struct kbase_hwcnt_enable_map *dst,
					      const struct kbase_hwcnt_physical_enable_map *src)
{
	struct kbase_hwcnt_enable_cm cm = {};

	if (WARN_ON(!src) || WARN_ON(!dst))
		return;

	kbasep_hwcnt_backend_gpu_block_map_from_physical(src->fe_bm, &cm.fe_bm[EM_LO],
							 &cm.fe_bm[EM_HI]);
	kbasep_hwcnt_backend_gpu_block_map_from_physical(src->shader_bm, &cm.shader_bm[EM_LO],
							 &cm.shader_bm[EM_HI]);
	kbasep_hwcnt_backend_gpu_block_map_from_physical(src->tiler_bm, &cm.tiler_bm[EM_LO],
							 &cm.tiler_bm[EM_HI]);
	kbasep_hwcnt_backend_gpu_block_map_from_physical(src->mmu_l2_bm, &cm.mmu_l2_bm[EM_LO],
							 &cm.mmu_l2_bm[EM_HI]);
	kbasep_hwcnt_backend_gpu_block_map_from_physical(src->fw_bm, &cm.fw_bm[EM_LO],
							 &cm.fw_bm[EM_HI]);
	kbasep_hwcnt_backend_gpu_block_map_from_physical(src->csg_bm, &cm.csg_bm[EM_LO],
							 &cm.csg_bm[EM_HI]);

	kbase_hwcnt_gpu_enable_map_from_cm(dst, &cm);
}

void kbase_hwcnt_gpu_enable_map_from_cm(struct kbase_hwcnt_enable_map *dst,
					const struct kbase_hwcnt_enable_cm *src)
{
	const struct kbase_hwcnt_metadata *metadata;
	size_t blk, blk_inst;

	if (WARN_ON(!src) || WARN_ON(!dst))
		return;

	metadata = dst->metadata;

	kbase_hwcnt_metadata_for_each_block(metadata, blk, blk_inst) {
		const u64 blk_type = kbase_hwcnt_metadata_block_type(metadata, blk);
		u64 *blk_map = kbase_hwcnt_enable_map_block_instance(dst, blk, blk_inst);
		const size_t map_stride =
			kbase_hwcnt_metadata_block_enable_map_stride(metadata, blk);
		size_t map_idx;

		for (map_idx = 0; map_idx < map_stride; ++map_idx) {
			if (WARN_ON(map_idx >= EM_COUNT))
				break;

			switch ((enum kbase_hwcnt_gpu_v5_block_type)blk_type) {
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW_UNDEFINED:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG_UNDEFINED:
				/* Nothing to do in this case. */
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FE3:
				blk_map[map_idx] = src->fe_bm[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_TILER:
				blk_map[map_idx] = src->tiler_bm[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_SC3:
				blk_map[map_idx] = src->shader_bm[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_MEMSYS2:
				blk_map[map_idx] = src->mmu_l2_bm[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_FW3:
				blk_map[map_idx] = src->fw_bm[map_idx];
				break;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG2:
				fallthrough;
			case KBASE_HWCNT_GPU_V5_BLOCK_TYPE_PERF_CSG3:
				blk_map[map_idx] = src->csg_bm[map_idx];
				break;
			default:
				WARN(true, "Invalid block type %llu", blk_type);
			}
		}
	}
}

void kbase_hwcnt_gpu_patch_dump_headers(struct kbase_hwcnt_dump_buffer *buf,
					const struct kbase_hwcnt_enable_map *enable_map)
{
	const struct kbase_hwcnt_metadata *metadata;
	size_t blk, blk_inst;

	if (WARN_ON(!buf) || WARN_ON(!enable_map) || WARN_ON(buf->metadata != enable_map->metadata))
		return;

	metadata = buf->metadata;

	kbase_hwcnt_metadata_for_each_block(metadata, blk, blk_inst) {
		u64 *buf_blk = kbase_hwcnt_dump_buffer_block_instance(buf, blk, blk_inst);
		const u64 *blk_map =
			kbase_hwcnt_enable_map_block_instance(enable_map, blk, blk_inst);

		const size_t map_stride =
			kbase_hwcnt_metadata_block_enable_map_stride(metadata, blk);
		u64 prfcnt_bm[EM_COUNT] = { 0 };
		u32 prfcnt_en = 0;
		size_t map_idx;

		for (map_idx = 0; map_idx < map_stride; ++map_idx) {
			if (WARN_ON(map_idx >= EM_COUNT))
				break;

			prfcnt_bm[map_idx] = blk_map[map_idx];
		}

		prfcnt_en = kbase_hwcnt_backend_gpu_block_map_to_physical(prfcnt_bm[EM_LO],
									  prfcnt_bm[EM_HI]);

		buf_blk[KBASE_HWCNT_V5_PRFCNT_EN_HEADER] = prfcnt_en;
	}
}
