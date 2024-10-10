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

#ifndef _UAPI_KBASE_JM_MEM_FLAGS_H_
#define _UAPI_KBASE_JM_MEM_FLAGS_H_

/* Memory allocation, access/hint flags & mask specific to JM GPU.
 *
 * See base_mem_alloc_flags.
 */

/* Unused bit for JM, only used in CSF for BASE_MEM_FIXED */
#define BASE_MEM_UNUSED_BIT_8 ((base_mem_alloc_flags)1 << 8)

/* Unused bit for JM, only used in CSF for BASE_CSF_EVENT */
#define BASE_MEM_UNUSED_BIT_19 ((base_mem_alloc_flags)1 << 19)

/**
 * BASE_MEM_TILER_ALIGN_TOP - Memory starting from the end of the initial commit is aligned
 * to 'extension' pages, where 'extension' must be a power of 2 and no more than
 * BASE_MEM_TILER_ALIGN_TOP_EXTENSION_MAX_PAGES
 */
#define BASE_MEM_TILER_ALIGN_TOP ((base_mem_alloc_flags)1 << 20)

/* Previously BASEP_MEM_PERFORM_JIT_TRIM, can be reused in the future */
#define BASE_MEM_UNUSED_BIT_29 ((base_mem_alloc_flags)1 << 29)

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
#define BASEP_MEM_FLAGS_NR_BITS (6)

/* A mask of all bits that are not used by a flag on JM */
#define BASE_MEM_FLAGS_UNUSED                                                    \
	(BASE_MEM_UNUSED_BIT_5 | BASE_MEM_UNUSED_BIT_7 | BASE_MEM_UNUSED_BIT_8 | \
	 BASE_MEM_UNUSED_BIT_19 | BASE_MEM_UNUSED_BIT_27 | BASE_MEM_UNUSED_BIT_29)

#endif /* _UAPI_KBASE_JM_MEM_FLAGS_H_ */
