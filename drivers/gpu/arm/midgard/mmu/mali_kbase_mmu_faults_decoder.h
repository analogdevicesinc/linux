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
#ifndef _MALI_KBASE_MMU_FAULTS_DECODER_H_
#define _MALI_KBASE_MMU_FAULTS_DECODER_H_

#include <linux/types.h>
#include <mali_kbase.h>

/* FAULTSTATUS.SOURCE_ID encoding */
#define SOURCE_ID_CORE_ID_SHIFT (9)
#define SOURCE_ID_CORE_ID_MASK (0x7F << SOURCE_ID_CORE_ID_SHIFT)
#define SOURCE_ID_UTLB_ID_SHIFT (8)
#define SOURCE_ID_UTLB_ID_MASK (0x01 << SOURCE_ID_UTLB_ID_SHIFT)
#define SOURCE_ID_CORE_TYPE_SHIFT (12)
#define SOURCE_ID_CORE_TYPE_MASK (0x0F << SOURCE_ID_CORE_TYPE_SHIFT)
#define SOURCE_ID_CORE_INDEX_SHIFT (6)
#define SOURCE_ID_CORE_INDEX_MASK (0x3F << SOURCE_ID_CORE_INDEX_SHIFT)

/**
 * FAULT_SOURCE_ID_CORE_ID_GET() - Get core ID of a fault.
 *
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 *
 * Get core ID part of SOURCE_ID field of FAULTSTATUS (MMU) or
 * GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: core ID of the fault.
 */
#define FAULT_SOURCE_ID_CORE_ID_GET(source_id) \
	((source_id & SOURCE_ID_CORE_ID_MASK) >> SOURCE_ID_CORE_ID_SHIFT)

/**
 * FAULT_SOURCE_ID_UTLB_ID_GET() - Get UTLB ID of a fault.
 *
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 *
 * Get UTLB(micro-TLB) ID part of SOURCE_ID field of FAULTSTATUS (MMU) or
 * GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: UTLB ID of the fault.
 */
#define FAULT_SOURCE_ID_UTLB_ID_GET(source_id) \
	((source_id & SOURCE_ID_UTLB_ID_MASK) >> SOURCE_ID_UTLB_ID_SHIFT)

/**
 * FAULT_SOURCE_ID_CORE_TYPE_GET() - Get core type of a fault.
 *
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 *
 * Get core type part of SOURCE_ID field of FAULTSTATUS (MMU) or
 * GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: core type code of the fault.
 */
#define FAULT_SOURCE_ID_CORE_TYPE_GET(source_id) \
	((source_id & SOURCE_ID_CORE_TYPE_MASK) >> SOURCE_ID_CORE_TYPE_SHIFT)

/**
 * FAULT_SOURCE_ID_CORE_INDEX_GET() - Get core index of a fault.
 *
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 *
 * Get core index part of SOURCE_ID field of FAULTSTATUS (MMU) or
 * GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: core index of the fault.
 */
#define FAULT_SOURCE_ID_CORE_INDEX_GET(source_id) \
	((source_id & SOURCE_ID_CORE_INDEX_MASK) >> SOURCE_ID_CORE_INDEX_SHIFT)

/**
 * fault_source_id_internal_requester_get() - Get internal_requester of a fault.
 *
 * @kbdev: The kbase device structure for the device (must be a valid pointer).
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 *
 * Get internal_requester part of SOURCE_ID field of FAULTSTATUS (MMU) or
 * GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: Internal requester code of the fault.
 */
unsigned int fault_source_id_internal_requester_get(struct kbase_device *kbdev,
						    unsigned int source_id);

/**
 * fault_source_id_internal_requester_get_str() - Get internal_requester of a
 * fault in a human readable format.
 *
 * @kbdev: The kbase device structure for the device (must be a valid pointer).
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 * @access_type: the direction of data transfer that caused the fault (atomic,
 *				 execute, read, write)
 *
 * Get the human readable decoding of internal_requester part of SOURCE_ID field
 * of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: Internal requester of the fault in human readable format.
 */
const char *fault_source_id_internal_requester_get_str(struct kbase_device *kbdev,
						       unsigned int source_id,
						       unsigned int access_type);

/**
 * fault_source_id_core_type_description_get() - Get the core type of
 * a fault in a human readable format.
 *
 * @kbdev: The kbase device structure for the device (must be a valid pointer).
 * @source_id: SOURCE_ID field of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU)
 *			   registers.
 *
 * Get the human readable decoding of core type part of SOURCE_ID field
 * of FAULTSTATUS (MMU) or GPU_FAULTSTATUS (GPU) registers.
 *
 * Return: core type of the fault in human readable format.
 */
const char *fault_source_id_core_type_description_get(struct kbase_device *kbdev,
						      unsigned int source_id);

#endif /* _MALI_KBASE_MMU_FAULTS_DECODER_H_ */
