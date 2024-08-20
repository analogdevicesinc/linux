/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2020-2024 ARM Limited. All rights reserved.
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
 * DOC: Driver Capability Queries.
 */

#ifndef _KBASE_CAPS_H_
#define _KBASE_CAPS_H_

#include <linux/types.h>

/**
 * enum mali_kbase_cap - Enumeration for kbase capability
 *
 * @MALI_KBASE_CAP_SYSTEM_MONITOR: System Monitor
 * @MALI_KBASE_CAP_JIT_PRESSURE_LIMIT: JIT Pressure limit
 * @MALI_KBASE_CAP_QUERY_MEM_DONT_NEED: BASE_MEM_DONT_NEED is queryable
 * @MALI_KBASE_CAP_QUERY_MEM_GROW_ON_GPF: BASE_MEM_GROW_ON_GPF is queryable
 * @MALI_KBASE_CAP_QUERY_MEM_PROTECTED: BASE_MEM_PROTECTED is queryable
 * @MALI_KBASE_CAP_QUERY_MEM_IMPORT_SYNC_ON_MAP_UNMAP: BASE_MEM_IMPORT_SYNC_ON_MAP_UNMAP is
 *                                                     queryable
 * @MALI_KBASE_CAP_QUERY_MEM_KERNEL_SYNC: BASE_MEM_KERNEL_SYNC is queryable
 * @MALI_KBASE_CAP_QUERY_MEM_SAME_VA: BASE_MEM_SAME_VA is queryable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_DONT_NEED: BASE_MEM_DONT_NEED is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_PROTECTED_IN_UNPROTECTED_ALLOCS: BASE_MEM_PROTECTED is not
 *                                                                   allocatable in functions other
 *                                                                   than base_mem_protected
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_5: BASE_MEM_UNUSED_BIT_5 is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_7: BASE_MEM_UNUSED_BIT_7 is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_8: BASE_MEM_UNUSED_BIT_8 is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_19: BASE_MEM_UNUSED_BIT_19 is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_20: BASE_MEM_UNUSED_BIT_20 is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_27: BASE_MEM_UNUSED_BIT_27 is not allocatable
 * @MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_29: BASE_MEM_UNUSED_BIT_29 is not allocatable
 * @MALI_KBASE_NUM_CAPS: Delimiter
 *
 * New enumerator must not be negative and smaller than @MALI_KBASE_NUM_CAPS.
 */
enum mali_kbase_cap {
	MALI_KBASE_CAP_SYSTEM_MONITOR = 0,
	MALI_KBASE_CAP_JIT_PRESSURE_LIMIT,
	MALI_KBASE_CAP_QUERY_MEM_DONT_NEED,
	MALI_KBASE_CAP_QUERY_MEM_GROW_ON_GPF,
	MALI_KBASE_CAP_QUERY_MEM_PROTECTED,
	MALI_KBASE_CAP_QUERY_MEM_IMPORT_SYNC_ON_MAP_UNMAP,
	MALI_KBASE_CAP_QUERY_MEM_KERNEL_SYNC,
	MALI_KBASE_CAP_QUERY_MEM_SAME_VA,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_DONT_NEED,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_PROTECTED_IN_UNPROTECTED_ALLOCS,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_8,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_19,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_20,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_27,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_5,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_7,
	MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_29,
	MALI_KBASE_NUM_CAPS
};

extern bool mali_kbase_supports_cap(unsigned long api_version, enum mali_kbase_cap cap);

static inline bool mali_kbase_supports_system_monitor(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_SYSTEM_MONITOR);
}

static inline bool mali_kbase_supports_jit_pressure_limit(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_JIT_PRESSURE_LIMIT);
}

static inline bool mali_kbase_supports_query_mem_dont_need(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_QUERY_MEM_DONT_NEED);
}

static inline bool mali_kbase_supports_query_mem_grow_on_gpf(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_QUERY_MEM_GROW_ON_GPF);
}

static inline bool mali_kbase_supports_query_mem_protected(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_QUERY_MEM_PROTECTED);
}

static inline bool mali_kbase_supports_query_mem_import_sync_on_map_unmap(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version,
				       MALI_KBASE_CAP_QUERY_MEM_IMPORT_SYNC_ON_MAP_UNMAP);
}

static inline bool mali_kbase_supports_query_mem_kernel_sync(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_QUERY_MEM_KERNEL_SYNC);
}

static inline bool mali_kbase_supports_query_mem_same_va(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_QUERY_MEM_SAME_VA);
}

static inline bool mali_kbase_supports_reject_alloc_mem_dont_need(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_DONT_NEED);
}

static inline bool
mali_kbase_supports_reject_alloc_mem_protected_in_unprotected_allocs(unsigned long api_version)
{
	return mali_kbase_supports_cap(
		api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_PROTECTED_IN_UNPROTECTED_ALLOCS);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_5(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_5);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_7(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_7);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_8(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_8);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_19(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_19);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_20(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_20);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_27(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_27);
}

static inline bool mali_kbase_supports_reject_alloc_mem_unused_bit_29(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_REJECT_ALLOC_MEM_UNUSED_BIT_29);
}

#endif /* __KBASE_CAPS_H_ */
