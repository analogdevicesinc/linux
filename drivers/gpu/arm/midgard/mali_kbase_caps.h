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
 * @MALI_KBASE_CAP_MEM_DONT_NEED: Not needed physical memory
 * @MALI_KBASE_CAP_MEM_GROW_ON_GPF: Memory grow on page fault
 * @MALI_KBASE_CAP_MEM_PROTECTED: Protected memory
 * @MALI_KBASE_CAP_MEM_IMPORT_SYNC_ON_MAP_UNMAP: CPU cache maintenance required when
 *                                               imported GPU memory is mapped/unmapped
 * @MALI_KBASE_CAP_MEM_KERNEL_SYNC: Kernel side cache sync ops required
 * @MALI_KBASE_CAP_MEM_SAME_VA: Same VA on CPU and GPU
 * @MALI_KBASE_NUM_CAPS: Delimiter
 *
 * New enumerator must not be negative and smaller than @MALI_KBASE_NUM_CAPS.
 */
enum mali_kbase_cap {
	MALI_KBASE_CAP_SYSTEM_MONITOR = 0,
	MALI_KBASE_CAP_JIT_PRESSURE_LIMIT,
	MALI_KBASE_CAP_MEM_DONT_NEED,
	MALI_KBASE_CAP_MEM_GROW_ON_GPF,
	MALI_KBASE_CAP_MEM_PROTECTED,
	MALI_KBASE_CAP_MEM_IMPORT_SYNC_ON_MAP_UNMAP,
	MALI_KBASE_CAP_MEM_KERNEL_SYNC,
	MALI_KBASE_CAP_MEM_SAME_VA,
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

static inline bool mali_kbase_supports_mem_dont_need(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_MEM_DONT_NEED);
}

static inline bool mali_kbase_supports_mem_grow_on_gpf(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_MEM_GROW_ON_GPF);
}

static inline bool mali_kbase_supports_mem_protected(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_MEM_PROTECTED);
}

static inline bool mali_kbase_supports_mem_import_sync_on_map_unmap(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_MEM_IMPORT_SYNC_ON_MAP_UNMAP);
}

static inline bool mali_kbase_supports_mem_kernel_sync(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_MEM_KERNEL_SYNC);
}

static inline bool mali_kbase_supports_mem_same_va(unsigned long api_version)
{
	return mali_kbase_supports_cap(api_version, MALI_KBASE_CAP_MEM_SAME_VA);
}

#endif /* __KBASE_CAPS_H_ */
