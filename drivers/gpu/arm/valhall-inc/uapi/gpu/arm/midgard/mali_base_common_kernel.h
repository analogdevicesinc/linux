/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2022-2024 ARM Limited. All rights reserved.
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

#ifndef _UAPI_BASE_COMMON_KERNEL_H_
#define _UAPI_BASE_COMMON_KERNEL_H_

#include <linux/types.h>
#include "mali_kbase_mem_flags.h"

struct base_mem_handle {
	struct {
		__u64 handle;
	} basep;
};

#define BASE_GPU_NUM_TEXTURE_FEATURES_REGISTERS 4

/* Flags to pass to ::base_context_init.
 * Flags can be ORed together to enable multiple things.
 *
 * These share the same space as BASEP_CONTEXT_FLAG_*, and so must
 * not collide with them.
 */
typedef __u32 base_context_create_flags;

/* Flags for base context */

/* No flags set */
#define BASE_CONTEXT_CREATE_FLAG_NONE ((base_context_create_flags)0)

/* Base context is embedded in a cctx object (flag used for CINSTR
 * software counter macros)
 */
#define BASE_CONTEXT_CCTX_EMBEDDED ((base_context_create_flags)1 << 0)

/* Base context is a 'System Monitor' context for Hardware counters.
 *
 * One important side effect of this is that job submission is disabled.
 */
#define BASE_CONTEXT_SYSTEM_MONITOR_SUBMIT_DISABLED ((base_context_create_flags)1 << 1)

/* Bit-shift used to encode a memory group ID in base_context_create_flags
 */
#define BASEP_CONTEXT_MMU_GROUP_ID_SHIFT (3)

/* Bitmask used to encode a memory group ID in base_context_create_flags
 */
#define BASEP_CONTEXT_MMU_GROUP_ID_MASK \
	((base_context_create_flags)0xF << BASEP_CONTEXT_MMU_GROUP_ID_SHIFT)

/* Bitpattern describing the base_context_create_flags that can be
 * passed to the kernel
 */
#define BASEP_CONTEXT_CREATE_KERNEL_FLAGS \
	(BASE_CONTEXT_SYSTEM_MONITOR_SUBMIT_DISABLED | BASEP_CONTEXT_MMU_GROUP_ID_MASK)

/* Flags for base tracepoint
 */

/* Enable additional tracepoints for latency measurements (TL_ATOM_READY,
 * TL_ATOM_DONE, TL_ATOM_PRIO_CHANGE, TL_ATOM_EVENT_POST)
 */
#define BASE_TLSTREAM_ENABLE_LATENCY_TRACEPOINTS (1U << 0)

/* Indicate that job dumping is enabled. This could affect certain timers
 * to account for the performance impact.
 */
#define BASE_TLSTREAM_JOB_DUMPING_ENABLED (1U << 1)

#endif /* _UAPI_BASE_COMMON_KERNEL_H_ */
