/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023 ARM Limited. All rights reserved.
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

#ifndef _MALI_KBASE_REGMAP_LEGACY_JM_H_
#define _MALI_KBASE_REGMAP_LEGACY_JM_H_

#if MALI_USE_CSF && defined(__KERNEL__)
#error "Cannot be compiled with CSF"
#endif

/* GPU control registers */
#define CORE_FEATURES 0x008 /* (RO) Shader Core Features */
#define JS_PRESENT 0x01C /* (RO) Job slots present */
#define LATEST_FLUSH 0x038 /* (RO) Flush ID of latest clean-and-invalidate operation */
#define PRFCNT_BASE_LO 0x060 /* (RW) Performance counter memory region base address, low word */
#define PRFCNT_BASE_HI 0x064 /* (RW) Performance counter memory region base address, high word */
#define PRFCNT_CONFIG 0x068 /* (RW) Performance counter configuration */
#define PRFCNT_JM_EN 0x06C /* (RW) Performance counter enable flags for Job Manager */
#define PRFCNT_SHADER_EN 0x070 /* (RW) Performance counter enable flags for shader cores */
#define PRFCNT_TILER_EN 0x074 /* (RW) Performance counter enable flags for tiler */
#define PRFCNT_MMU_L2_EN 0x07C /* (RW) Performance counter enable flags for MMU/L2 cache */

#define JS0_FEATURES 0x0C0 /* (RO) Features of job slot 0 */
#define JS1_FEATURES 0x0C4 /* (RO) Features of job slot 1 */
#define JS2_FEATURES 0x0C8 /* (RO) Features of job slot 2 */
#define JS3_FEATURES 0x0CC /* (RO) Features of job slot 3 */
#define JS4_FEATURES 0x0D0 /* (RO) Features of job slot 4 */
#define JS5_FEATURES 0x0D4 /* (RO) Features of job slot 5 */
#define JS6_FEATURES 0x0D8 /* (RO) Features of job slot 6 */
#define JS7_FEATURES 0x0DC /* (RO) Features of job slot 7 */
#define JS8_FEATURES 0x0E0 /* (RO) Features of job slot 8 */
#define JS9_FEATURES 0x0E4 /* (RO) Features of job slot 9 */
#define JS10_FEATURES 0x0E8 /* (RO) Features of job slot 10 */
#define JS11_FEATURES 0x0EC /* (RO) Features of job slot 11 */
#define JS12_FEATURES 0x0F0 /* (RO) Features of job slot 12 */
#define JS13_FEATURES 0x0F4 /* (RO) Features of job slot 13 */
#define JS14_FEATURES 0x0F8 /* (RO) Features of job slot 14 */
#define JS15_FEATURES 0x0FC /* (RO) Features of job slot 15 */

#define JS_FEATURES_REG(n) GPU_CONTROL_REG(JS0_FEATURES + ((n) << 2))

#define JM_CONFIG 0xF00 /* (RW) Job manager configuration (implementation-specific) */

/* Job control registers */
/* status==active and _next == busy snapshot from last JOB_IRQ_CLEAR */
#define JOB_IRQ_JS_STATE 0x010
/* cycles to delay delivering an interrupt externally. The JOB_IRQ_STATUS
 * is NOT affected by this, just the delivery of the interrupt.
 */
#define JOB_IRQ_THROTTLE 0x014

#define JOB_SLOT0 0x800 /* Configuration registers for job slot 0 */
#define JOB_SLOT_REG(n, r) (JOB_CONTROL_REG(JOB_SLOT0 + ((n) << 7)) + (r))
#define JOB_SLOT1 0x880 /* Configuration registers for job slot 1 */
#define JOB_SLOT2 0x900 /* Configuration registers for job slot 2 */
#define JOB_SLOT3 0x980 /* Configuration registers for job slot 3 */
#define JOB_SLOT4 0xA00 /* Configuration registers for job slot 4 */
#define JOB_SLOT5 0xA80 /* Configuration registers for job slot 5 */
#define JOB_SLOT6 0xB00 /* Configuration registers for job slot 6 */
#define JOB_SLOT7 0xB80 /* Configuration registers for job slot 7 */
#define JOB_SLOT8 0xC00 /* Configuration registers for job slot 8 */
#define JOB_SLOT9 0xC80 /* Configuration registers for job slot 9 */
#define JOB_SLOT10 0xD00 /* Configuration registers for job slot 10 */
#define JOB_SLOT11 0xD80 /* Configuration registers for job slot 11 */
#define JOB_SLOT12 0xE00 /* Configuration registers for job slot 12 */
#define JOB_SLOT13 0xE80 /* Configuration registers for job slot 13 */
#define JOB_SLOT14 0xF00 /* Configuration registers for job slot 14 */
#define JOB_SLOT15 0xF80 /* Configuration registers for job slot 15 */

/* JM Job control register definitions for mali_kbase_debug_job_fault */
#define JS_HEAD_LO 0x00 /* (RO) Job queue head pointer for job slot n, low word */
#define JS_HEAD_HI 0x04 /* (RO) Job queue head pointer for job slot n, high word */
#define JS_TAIL_LO 0x08 /* (RO) Job queue tail pointer for job slot n, low word */
#define JS_TAIL_HI 0x0C /* (RO) Job queue tail pointer for job slot n, high word */
#define JS_AFFINITY_LO 0x10 /* (RO) Core affinity mask for job slot n, low word */
#define JS_AFFINITY_HI 0x14 /* (RO) Core affinity mask for job slot n, high word */
#define JS_CONFIG 0x18 /* (RO) Configuration settings for job slot n */
#define JS_XAFFINITY 0x1C /* (RO) Extended affinity mask for job slot n*/
#define JS_COMMAND 0x20 /* (WO) Command register for job slot n */
#define JS_STATUS 0x24 /* (RO) Status register for job slot n */
#define JS_HEAD_NEXT_LO 0x40 /* (RW) Next job queue head pointer for job slot n, low word */
#define JS_HEAD_NEXT_HI 0x44 /* (RW) Next job queue head pointer for job slot n, high word */
#define JS_AFFINITY_NEXT_LO 0x50 /* (RW) Next core affinity mask for job slot n, low word */
#define JS_AFFINITY_NEXT_HI 0x54 /* (RW) Next core affinity mask for job slot n, high word */
#define JS_CONFIG_NEXT 0x58 /* (RW) Next configuration settings for job slot n */
#define JS_XAFFINITY_NEXT 0x5C /* (RW) Next extended affinity mask for job slot n */
#define JS_COMMAND_NEXT 0x60 /* (RW) Next command register for job slot n */
#define JS_FLUSH_ID_NEXT 0x70 /* (RW) Next job slot n cache flush ID */

#endif /* _MALI_KBASE_REGMAP_LEGACY_JM_H_ */
