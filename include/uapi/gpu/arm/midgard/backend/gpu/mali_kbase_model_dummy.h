/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * (C) COPYRIGHT 2021 ARM Limited. All rights reserved.
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

/*
 * Dummy Model interface
 */

#ifndef _UAPI_KBASE_MODEL_DUMMY_H_
#define _UAPI_KBASE_MODEL_DUMMY_H_

#include <linux/types.h>

#define KBASE_DUMMY_MODEL_COUNTER_HEADER_DWORDS (4)
#define KBASE_DUMMY_MODEL_COUNTER_PER_CORE      (60)
#define KBASE_DUMMY_MODEL_COUNTER_PER_CORE_TYPE  \
		(64*KBASE_DUMMY_MODEL_COUNTER_PER_CORE)
#define KBASE_DUMMY_MODEL_COUNTERS_PER_BIT      (4)
#define KBASE_DUMMY_MODEL_COUNTER_ENABLED(enable_mask, ctr_idx) \
	(enable_mask & (1 << (ctr_idx / KBASE_DUMMY_MODEL_COUNTERS_PER_BIT)))

#define KBASE_DUMMY_MODEL_HEADERS_PER_BLOCK 4
#define KBASE_DUMMY_MODEL_COUNTERS_PER_BLOCK 60
#define KBASE_DUMMY_MODEL_VALUES_PER_BLOCK                                     \
	(KBASE_DUMMY_MODEL_COUNTERS_PER_BLOCK +                                \
	 KBASE_DUMMY_MODEL_HEADERS_PER_BLOCK)
#define KBASE_DUMMY_MODEL_BLOCK_SIZE                                           \
	(KBASE_DUMMY_MODEL_VALUES_PER_BLOCK * sizeof(__u32))
#define KBASE_DUMMY_MODEL_MAX_MEMSYS_BLOCKS      8
#define KBASE_DUMMY_MODEL_MAX_SHADER_CORES       32
#define KBASE_DUMMY_MODEL_MAX_NUM_PERF_BLOCKS    \
	(1 + 1 + KBASE_DUMMY_MODEL_MAX_MEMSYS_BLOCKS + KBASE_DUMMY_MODEL_MAX_SHADER_CORES)
#define KBASE_DUMMY_MODEL_COUNTER_TOTAL          \
	(KBASE_DUMMY_MODEL_MAX_NUM_PERF_BLOCKS * KBASE_DUMMY_MODEL_COUNTER_PER_CORE_TYPE)

#endif /* _UAPI_KBASE_MODEL_DUMMY_H_ */
