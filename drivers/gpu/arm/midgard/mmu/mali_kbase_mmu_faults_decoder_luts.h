/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2024 ARM Limited. All rights reserved.
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

#ifndef _MALI_KBASE_MMU_FAULTS_DECODER_LUTS_H_
#define _MALI_KBASE_MMU_FAULTS_DECODER_LUTS_H_

#include <linux/types.h>

/**
 * decode_fault_source_core_id_t_desc() - Get core description of a
 * fault in a human readable format.
 *
 * @idx: Core ID part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: core ID of the fault in human readable format.
 */
const char *decode_fault_source_core_id_t_desc(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_core_id_t_core_type() - Get core type of a
 * fault in a human readable format.
 *
 * @idx: Core ID part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: core type of the fault in human readable format.
 */
const char *decode_fault_source_core_id_t_core_type(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_core_type_t_name() - Get core type name of a
 * fault.
 *
 * @idx: Core type part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: core type short name of the fault.
 */
const char *decode_fault_source_core_type_t_name(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_core_type_t_desc() - Get core type description of a
 * fault.
 *
 * @idx: Core type part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: core type description of the fault.
 */
const char *decode_fault_source_core_type_t_desc(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_shader_r_t() - Get internal requester of a
 * fault in a human readable format.
 *
 * @idx: Internal requester part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: Internal requester of a fault in a human readable format for read
 * operations on a shader core.
 */
const char *decode_fault_source_shader_r_t(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_shader_w_t() - Get internal requester of a
 * fault in a human readable format.
 *
 * @idx: Internal requester part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: Internal requester of a fault in a human readable format for write
 * operations on a shader core.
 */
const char *decode_fault_source_shader_w_t(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_tiler_r_t() - Get internal requester of a
 * fault in a human readable format.
 *
 * @idx: Internal requester part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: Internal requester of a fault in a human readable format for read
 * operations on a tiler core.
 */
const char *decode_fault_source_tiler_r_t(u16 idx, u32 gpu_id);

/**
 * decode_fault_source_tiler_w_t() - Get internal requester of a
 * fault in a human readable format.
 *
 * @idx: Internal requester part of SOURCE_ID field of the fault.
 * @gpu_id: GPU id composed of arch_major << 16 | arch_minor << 8 | arch_rev.
 *
 * Return: Internal requester of a fault in a human readable format for write
 * operations on a tiler core.
 */
const char *decode_fault_source_tiler_w_t(u16 idx, u32 gpu_id);

#endif /* _MALI_KBASE_MMU_FAULTS_DECODER_LUTS_H_ */
