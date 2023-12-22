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

#ifndef _KBASE_GPUPROPS_H_
#define _KBASE_GPUPROPS_H_

#include <linux/types.h>

/* Forward definition - see mali_kbase.h */
struct kbase_device;
struct max_config_props;
struct curr_config_props;
struct kbase_gpu_id_props;

/**
 * KBASE_UBFX32 - Extracts bits from a 32-bit bitfield.
 * @value:  The value from which to extract bits.
 * @offset: The first bit to extract (0 being the LSB).
 * @size:   The number of bits to extract.
 *
 * Context: @offset + @size <= 32.
 *
 * Return: Bits [@offset, @offset + @size) from @value.
 */
/* from mali_cdsb.h */
#define KBASE_UBFX32(value, offset, size) \
	(((u32)(value) >> (u32)(offset)) & (u32)((1ULL << (u32)(size)) - 1))

/**
 * KBASE_UBFX64 - Extracts bits from a 64-bit bitfield.
 * @value:  The value from which to extract bits.
 * @offset: The first bit to extract (0 being the LSB).
 * @size:   The number of bits to extract.
 *
 * Context: @offset + @size <= 64.
 *
 * Return: Bits [@offset, @offset + @size) from @value.
 */
/* from mali_cdsb.h */
#define KBASE_UBFX64(value, offset, size) \
	(((u64)(value) >> (u32)(offset)) & (u64)((1ULL << (u32)(size)) - 1))

/**
 * kbase_gpuprops_update_composite_ids - update composite ids with new gpu id
 * @props: pointer to GPU_ID property structure
 */
void kbase_gpuprops_update_composite_ids(struct kbase_gpu_id_props *props);

/**
 * kbase_gpuprops_parse_gpu_id - parse fields of GPU_ID
 * @props: pointer to GPU_ID property structure
 * @gpu_id: gpu id register value
 */
void kbase_gpuprops_parse_gpu_id(struct kbase_gpu_id_props *props, u64 gpu_id);

/**
 * kbase_gpuprops_init - Set up Kbase GPU properties.
 * @kbdev: The struct kbase_device structure for the device
 *
 * Set up Kbase GPU properties with information from the GPU registers
 *
 * Return: Zero on success, Linux error code on failuren
 */
int kbase_gpuprops_init(struct kbase_device *kbdev);

/**
 * kbase_gpuprops_term - Terminate Kbase GPU properties.
 * @kbdev: The struct kbase_device structure for the device
 */
void kbase_gpuprops_term(struct kbase_device *kbdev);

/**
 * kbase_gpuprops_update_l2_features - Update GPU property of L2_FEATURES
 * @kbdev:   Device pointer
 *
 * This function updates l2_features and the log2 cache size.
 * The function expects GPU to be powered up and value of pm.active_count
 * to be 1.
 *
 * Return: Zero on success, Linux error code for failure
 */
int kbase_gpuprops_update_l2_features(struct kbase_device *kbdev);

/**
 * kbase_gpuprops_populate_user_buffer - Populate the GPU properties buffer
 * @kbdev: The kbase device
 *
 * Fills prop_buffer with the GPU properties for user space to read.
 *
 * Return: MALI_ERROR_NONE on success. Any other value indicates failure.
 */
int kbase_gpuprops_populate_user_buffer(struct kbase_device *kbdev);

/**
 * kbase_gpuprops_free_user_buffer - Free the GPU properties buffer.
 * @kbdev: kbase device pointer
 *
 * Free the GPU properties buffer allocated from
 * kbase_gpuprops_populate_user_buffer.
 */
void kbase_gpuprops_free_user_buffer(struct kbase_device *kbdev);

/**
 * kbase_device_populate_max_freq - Populate max gpu frequency.
 * @kbdev: kbase device pointer
 *
 * Populate the maximum gpu frequency to be used when devfreq is disabled.
 *
 * Return: 0 on success and non-zero value on failure.
 */
int kbase_device_populate_max_freq(struct kbase_device *kbdev);

/**
 * kbase_gpuprops_set_max_config - Set the max config information
 * @kbdev:       Device pointer
 * @max_config:  Maximum configuration data to be updated
 *
 * This function sets max_config in the kbase_gpu_props.
 */
void kbase_gpuprops_set_max_config(struct kbase_device *kbdev,
				   const struct max_config_props *max_config);

/**
 * kbase_gpuprops_get_curr_config_props - Get the current allocated resources
 * @kbdev: The &struct kbase_device structure for the device
 * @curr_config: The &struct curr_config_props structure to receive the result
 *
 * Fill the &struct curr_config_props structure with values from the GPU
 * configuration registers.
 *
 * Return: Zero on success, Linux error code on failure
 */
int kbase_gpuprops_get_curr_config_props(struct kbase_device *kbdev,
					 struct curr_config_props *const curr_config);

/**
 * kbase_gpuprops_req_curr_config_update - Request Current Config Update
 * @kbdev: The &struct kbase_device structure for the device
 *
 * Requests the current configuration to be updated next time the
 * kbase_gpuprops_get_curr_config_props() is called.
 *
 * Return: Zero on success, Linux error code on failure
 */
int kbase_gpuprops_req_curr_config_update(struct kbase_device *kbdev);

#endif /* _KBASE_GPUPROPS_H_ */
