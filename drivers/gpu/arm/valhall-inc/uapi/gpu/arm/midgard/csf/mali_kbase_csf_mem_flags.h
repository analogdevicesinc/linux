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

#ifndef _UAPI_KBASE_CSF_MEM_FLAGS_H_
#define _UAPI_KBASE_CSF_MEM_FLAGS_H_

/* Memory allocation, access/hint flags & mask specific to CSF GPU.
 *
 * See base_mem_alloc_flags.
 */

/* Must be FIXED memory. */
#define BASE_MEM_FIXED ((base_mem_alloc_flags)1 << 8)

/* CSF event memory
 *
 * If Outer shareable coherence is not specified or not available, then on
 * allocation kbase will automatically use the uncached GPU mapping.
 * There is no need for the client to specify BASE_MEM_UNCACHED_GPU
 * themselves when allocating memory with the BASE_MEM_CSF_EVENT flag.
 *
 * This memory requires a permanent mapping
 *
 * See also kbase_reg_needs_kernel_mapping()
 */
#define BASE_MEM_CSF_EVENT ((base_mem_alloc_flags)1 << 19)

/* Unused bit for CSF, only used in JM for BASE_MEM_TILER_ALIGN_TOP */
#define BASE_MEM_UNUSED_BIT_20 ((base_mem_alloc_flags)1 << 20)

/* Must be FIXABLE memory: its GPU VA will be determined at a later point,
 * at which time it will be at a fixed GPU VA.
 */
#define BASE_MEM_FIXABLE ((base_mem_alloc_flags)1 << 29)

/* A mask of flags that, when provided, cause other flags to be
 * enabled but are not enabled themselves
 */
#define BASE_MEM_FLAGS_ACTION_MODIFIERS (BASE_MEM_COHERENT_SYSTEM_REQUIRED | BASE_MEM_IMPORT_SHARED)

/* A mask of all currently reserved flags */
#define BASE_MEM_FLAGS_RESERVED ((base_mem_alloc_flags)0)

/* Number of bits used as flags for base memory management from kernel-side
 * only (ie BASEP_MEM_* flags), located from 63 bit downwards:
 *   < 63 .. (64 - BASEP_MEM_FLAGS_NR_BITS) >
 */
#define BASEP_MEM_FLAGS_NR_BITS (4)

/* A mask of all bits that are not used by a flag on CSF */
#define BASE_MEM_FLAGS_UNUSED                                                     \
	(BASE_MEM_UNUSED_BIT_5 | BASE_MEM_UNUSED_BIT_7 | BASE_MEM_UNUSED_BIT_20 | \
	 BASE_MEM_UNUSED_BIT_27)

/* Special base mem handles specific to CSF.
 */
#define BASEP_MEM_CSF_USER_REG_PAGE_HANDLE (47ul << LOCAL_PAGE_SHIFT)
#define BASEP_MEM_CSF_USER_IO_PAGES_HANDLE (48ul << LOCAL_PAGE_SHIFT)

#define KBASE_CSF_NUM_USER_IO_PAGES_HANDLE \
	((BASE_MEM_COOKIE_BASE - BASEP_MEM_CSF_USER_IO_PAGES_HANDLE) >> LOCAL_PAGE_SHIFT)

#endif /* _UAPI_KBASE_CSF_MEM_FLAGS_H_ */
