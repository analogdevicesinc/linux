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

#ifndef _UAPI_KBASE_MEM_FLAGS_H_
#define _UAPI_KBASE_MEM_FLAGS_H_

#if MALI_USE_CSF
#include "csf/mali_kbase_csf_mem_flags.h"
#else
#include "jm/mali_kbase_jm_mem_flags.h"
#endif

/* Memory allocation, access/hint flags & mask.
 *
 * See base_mem_alloc_flags.
 */

/* IN */
/* Read access CPU side
 */
#define BASE_MEM_PROT_CPU_RD ((base_mem_alloc_flags)1 << 0)

/* Write access CPU side
 */
#define BASE_MEM_PROT_CPU_WR ((base_mem_alloc_flags)1 << 1)

/* Read access GPU side
 */
#define BASE_MEM_PROT_GPU_RD ((base_mem_alloc_flags)1 << 2)

/* Write access GPU side
 */
#define BASE_MEM_PROT_GPU_WR ((base_mem_alloc_flags)1 << 3)

/* Execute allowed on the GPU side
 */
#define BASE_MEM_PROT_GPU_EX ((base_mem_alloc_flags)1 << 4)

/* Unused bit, previously for BASEP_MEM_PERMANENT_KERNEL_MAPPING
 */
#define BASE_MEM_UNUSED_BIT_5 ((base_mem_alloc_flags)1 << 5)

/* The allocation will completely reside within the same 4GB chunk in the GPU
 * virtual space.
 * Since this flag is primarily required only for the TLS memory which will
 * not be used to contain executable code and also not used for Tiler heap,
 * it can't be used along with BASE_MEM_PROT_GPU_EX and TILER_ALIGN_TOP flags.
 */
#define BASE_MEM_GPU_VA_SAME_4GB_PAGE ((base_mem_alloc_flags)1 << 6)

/* Unused bit, previously for BASEP_MEM_NO_USER_FREE
 */
#define BASE_MEM_UNUSED_BIT_7 ((base_mem_alloc_flags)1 << 7)

/* Grow backing store on GPU Page Fault
 */
#define BASE_MEM_GROW_ON_GPF ((base_mem_alloc_flags)1 << 9)

/* Page coherence Outer shareable, if available
 */
#define BASE_MEM_COHERENT_SYSTEM ((base_mem_alloc_flags)1 << 10)

/* Page coherence Inner shareable
 */
#define BASE_MEM_COHERENT_LOCAL ((base_mem_alloc_flags)1 << 11)

/* IN/OUT */
/* Should be cached on the CPU, returned if actually cached
 */
#define BASE_MEM_CACHED_CPU ((base_mem_alloc_flags)1 << 12)

/* IN/OUT */
/* Must have same VA on both the GPU and the CPU
 */
#define BASE_MEM_SAME_VA ((base_mem_alloc_flags)1 << 13)

/* OUT */
/* Must call mmap to acquire a GPU address for the allocation
 */
#define BASE_MEM_NEED_MMAP ((base_mem_alloc_flags)1 << 14)

/* IN */
/* Page coherence Outer shareable, required.
 */
#define BASE_MEM_COHERENT_SYSTEM_REQUIRED ((base_mem_alloc_flags)1 << 15)

/* Protected memory
 */
#define BASE_MEM_PROTECTED ((base_mem_alloc_flags)1 << 16)

/* Not needed physical memory
 */
#define BASE_MEM_DONT_NEED ((base_mem_alloc_flags)1 << 17)

/* Must use shared CPU/GPU zone (SAME_VA zone) but doesn't require the
 * addresses to be the same
 */
#define BASE_MEM_IMPORT_SHARED ((base_mem_alloc_flags)1 << 18)

/* Should be uncached on the GPU, will work only for GPUs using AARCH64 mmu
 * mode. Some components within the GPU might only be able to access memory
 * that is GPU cacheable. Refer to the specific GPU implementation for more
 * details. The 3 shareability flags will be ignored for GPU uncached memory.
 * If used while importing USER_BUFFER type memory, then the import will fail
 * if the memory is not aligned to GPU and CPU cache line width.
 */
#define BASE_MEM_UNCACHED_GPU ((base_mem_alloc_flags)1 << 21)

/*
 * Bits [22:25] for group_id (0~15).
 *
 * base_mem_group_id_set() should be used to pack a memory group ID into a
 * base_mem_alloc_flags value instead of accessing the bits directly.
 * base_mem_group_id_get() should be used to extract the memory group ID from
 * a base_mem_alloc_flags value.
 */
#define BASEP_MEM_GROUP_ID_SHIFT 22
#define BASE_MEM_GROUP_ID_MASK ((base_mem_alloc_flags)0xF << BASEP_MEM_GROUP_ID_SHIFT)

/* Must do CPU cache maintenance when imported memory is mapped/unmapped
 * on GPU. Currently applicable to dma-buf type only.
 */
#define BASE_MEM_IMPORT_SYNC_ON_MAP_UNMAP ((base_mem_alloc_flags)1 << 26)

/* Unused bit, only previously used in JM for BASEP_MEM_FLAG_MAP_FIXED */
#define BASE_MEM_UNUSED_BIT_27 ((base_mem_alloc_flags)1 << 27)

/* Kernel side cache sync ops required */
#define BASE_MEM_KERNEL_SYNC ((base_mem_alloc_flags)1 << 28)

/* Note that the number of bits used for base_mem_alloc_flags
 * must be less than BASE_MEM_FLAGS_NR_BITS (for both user
 * and kernel-side usage) or allocated from bit 63 downwards
 * (for kernel-only usage) which is controlled by BASEP_MEM_FLAGS_NR_BITS
 */

/* Number of bits used as flags for base memory management from user-side
 * ie BASE_MEM_* flags.
 *
 * Must be kept in sync with the base_mem_alloc_flags flags
 */
#define BASE_MEM_FLAGS_NR_BITS 30

/* A mask of all the flags which are only valid within kbase,
 * and may not be passed to/from user space.
 */
#define BASE_MEM_FLAGS_KERNEL_ONLY \
	(~(((base_mem_alloc_flags)1 << (64 - BASEP_MEM_FLAGS_NR_BITS)) - 1))

/* A mask for all bits that are output from kbase, but never input. */
#define BASE_MEM_FLAGS_OUTPUT_MASK BASE_MEM_NEED_MMAP

/* A mask for all bits that can be input to kbase. */
#define BASE_MEM_FLAGS_INPUT_MASK                                     \
	(((((base_mem_alloc_flags)1 << BASE_MEM_FLAGS_NR_BITS) - 1) | \
	  BASE_MEM_FLAGS_KERNEL_ONLY) &                               \
	 ~BASE_MEM_FLAGS_OUTPUT_MASK)

/* A mask for all input and output bits */
#define BASE_MEM_ALL_FLAGS_MASK (BASE_MEM_FLAGS_INPUT_MASK | BASE_MEM_FLAGS_OUTPUT_MASK)

/* Special base mem handles.
 */
#define BASEP_MEM_INVALID_HANDLE (0ul)
#define BASE_MEM_MMU_DUMP_HANDLE (1ul << LOCAL_PAGE_SHIFT)
#define BASE_MEM_TRACE_BUFFER_HANDLE (2ul << LOCAL_PAGE_SHIFT)
#define BASE_MEM_MAP_TRACKING_HANDLE (3ul << LOCAL_PAGE_SHIFT)
#define BASEP_MEM_WRITE_ALLOC_PAGES_HANDLE (4ul << LOCAL_PAGE_SHIFT)
/* reserved handles ..-47<<PAGE_SHIFT> for future special handles */
#define BASE_MEM_COOKIE_BASE (64ul << LOCAL_PAGE_SHIFT)
#define BASE_MEM_FIRST_FREE_ADDRESS ((BITS_PER_LONG << LOCAL_PAGE_SHIFT) + BASE_MEM_COOKIE_BASE)

#endif /* _UAPI_KBASE_MEM_FLAGS_H_ */
