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

#ifndef _MALI_KBASE_REGMAP_LEGACY_H_
#define _MALI_KBASE_REGMAP_LEGACY_H_

#if MALI_USE_CSF
#include "regmap/mali_kbase_regmap_legacy_csf.h"
#else
#include "regmap/mali_kbase_regmap_legacy_jm.h"
#endif

/* Begin Register Offsets */
/* GPU control registers */
#define GPU_CONTROL_BASE 0x0000
#define GPU_CONTROL_REG(r) (GPU_CONTROL_BASE + (r))

#define GPU_ID 0x000 /* (RO) GPU and revision identifier */
#define L2_FEATURES 0x004 /* (RO) Level 2 cache features */
#define TILER_FEATURES 0x00C /* (RO) Tiler Features */
#define MEM_FEATURES 0x010 /* (RO) Memory system features */
#define MMU_FEATURES 0x014 /* (RO) MMU features */
#define AS_PRESENT 0x018 /* (RO) Address space slots present */
#define GPU_IRQ_RAWSTAT 0x020 /* (RW) */
#define GPU_IRQ_CLEAR 0x024 /* (WO) */
#define GPU_IRQ_MASK 0x028 /* (RW) */

#define GPU_IRQ_STATUS 0x02C /* (RO) */
#define GPU_COMMAND 0x030 /* (WO) */

#define GPU_STATUS 0x034 /* (RO) */

#define GPU_FAULTSTATUS 0x03C /* (RO) GPU exception type and fault status */
#define GPU_FAULTADDRESS_LO 0x040 /* (RO) GPU exception fault address, low word */
#define GPU_FAULTADDRESS_HI 0x044 /* (RO) GPU exception fault address, high word */

#define L2_CONFIG 0x048 /* (RW) Level 2 cache configuration */

#define PWR_KEY 0x050 /* (WO) Power manager key register */
#define PWR_OVERRIDE0 0x054 /* (RW) Power manager override settings */
#define PWR_OVERRIDE1 0x058 /* (RW) Power manager override settings */
#define GPU_FEATURES_LO 0x060 /* (RO) GPU features, low word */
#define GPU_FEATURES_HI 0x064 /* (RO) GPU features, high word */
#define PRFCNT_FEATURES 0x068 /* (RO) Performance counter features */
#define TIMESTAMP_OFFSET_LO 0x088 /* (RW) Global time stamp offset, low word */
#define TIMESTAMP_OFFSET_HI 0x08C /* (RW) Global time stamp offset, high word */
#define CYCLE_COUNT_LO 0x090 /* (RO) Cycle counter, low word */
#define CYCLE_COUNT_HI 0x094 /* (RO) Cycle counter, high word */
#define TIMESTAMP_LO 0x098 /* (RO) Global time stamp counter, low word */
#define TIMESTAMP_HI 0x09C /* (RO) Global time stamp counter, high word */

#define THREAD_MAX_THREADS 0x0A0 /* (RO) Maximum number of threads per core */
#define THREAD_MAX_WORKGROUP_SIZE 0x0A4 /* (RO) Maximum workgroup size */
#define THREAD_MAX_BARRIER_SIZE 0x0A8 /* (RO) Maximum threads waiting at a barrier */
#define THREAD_FEATURES 0x0AC /* (RO) Thread features */
#define THREAD_TLS_ALLOC 0x310 /* (RO) Number of threads per core that TLS must be allocated for */

#define TEXTURE_FEATURES_0 0x0B0 /* (RO) Support flags for indexed texture formats 0..31 */
#define TEXTURE_FEATURES_1 0x0B4 /* (RO) Support flags for indexed texture formats 32..63 */
#define TEXTURE_FEATURES_2 0x0B8 /* (RO) Support flags for indexed texture formats 64..95 */
#define TEXTURE_FEATURES_3 0x0BC /* (RO) Support flags for texture order */

#define TEXTURE_FEATURES_REG(n) GPU_CONTROL_REG(TEXTURE_FEATURES_0 + ((n) << 2))

#define GPU_COMMAND_ARG0_LO 0x0D0 /* (RW) Additional parameter 0 for GPU commands, low word */
#define GPU_COMMAND_ARG0_HI 0x0D4 /* (RW) Additional parameter 0 for GPU commands, high word */
#define GPU_COMMAND_ARG1_LO 0x0D8 /* (RW) Additional parameter 1 for GPU commands, low word */
#define GPU_COMMAND_ARG1_HI 0x0DC /* (RW) Additional parameter 1 for GPU commands, high word */

#define SHADER_PRESENT_LO 0x100 /* (RO) Shader core present bitmap, low word */
#define SHADER_PRESENT_HI 0x104 /* (RO) Shader core present bitmap, high word */

#define TILER_PRESENT_LO 0x110 /* (RO) Tiler core present bitmap, low word */
#define TILER_PRESENT_HI 0x114 /* (RO) Tiler core present bitmap, high word */

#define L2_PRESENT_LO 0x120 /* (RO) Level 2 cache present bitmap, low word */
#define L2_PRESENT_HI 0x124 /* (RO) Level 2 cache present bitmap, high word */

#define SHADER_READY_LO 0x140 /* (RO) Shader core ready bitmap, low word */
#define SHADER_READY_HI 0x144 /* (RO) Shader core ready bitmap, high word */

#define TILER_READY_LO 0x150 /* (RO) Tiler core ready bitmap, low word */
#define TILER_READY_HI 0x154 /* (RO) Tiler core ready bitmap, high word */

#define L2_READY_LO 0x160 /* (RO) Level 2 cache ready bitmap, low word */
#define L2_READY_HI 0x164 /* (RO) Level 2 cache ready bitmap, high word */

#define SHADER_PWRON_LO 0x180 /* (WO) Shader core power on bitmap, low word */
#define SHADER_PWRON_HI 0x184 /* (WO) Shader core power on bitmap, high word */

#define SHADER_PWRFEATURES 0x188 /* (RW) Shader core power features */

#define TILER_PWRON_LO 0x190 /* (WO) Tiler core power on bitmap, low word */
#define TILER_PWRON_HI 0x194 /* (WO) Tiler core power on bitmap, high word */

#define L2_PWRON_LO 0x1A0 /* (WO) Level 2 cache power on bitmap, low word */
#define L2_PWRON_HI 0x1A4 /* (WO) Level 2 cache power on bitmap, high word */

#define STACK_PRESENT_LO 0xE00 /* (RO) Core stack present bitmap, low word */
#define STACK_PRESENT_HI 0xE04 /* (RO) Core stack present bitmap, high word */

#define STACK_READY_LO 0xE10 /* (RO) Core stack ready bitmap, low word */
#define STACK_READY_HI 0xE14 /* (RO) Core stack ready bitmap, high word */

#define STACK_PWRON_LO 0xE20 /* (RO) Core stack power on bitmap, low word */
#define STACK_PWRON_HI 0xE24 /* (RO) Core stack power on bitmap, high word */

#define SHADER_PWROFF_LO 0x1C0 /* (WO) Shader core power off bitmap, low word */
#define SHADER_PWROFF_HI 0x1C4 /* (WO) Shader core power off bitmap, high word */

#define TILER_PWROFF_LO 0x1D0 /* (WO) Tiler core power off bitmap, low word */
#define TILER_PWROFF_HI 0x1D4 /* (WO) Tiler core power off bitmap, high word */

#define L2_PWROFF_LO 0x1E0 /* (WO) Level 2 cache power off bitmap, low word */
#define L2_PWROFF_HI 0x1E4 /* (WO) Level 2 cache power off bitmap, high word */

#define STACK_PWROFF_LO 0xE30 /* (RO) Core stack power off bitmap, low word */
#define STACK_PWROFF_HI 0xE34 /* (RO) Core stack power off bitmap, high word */

#define SHADER_PWRTRANS_LO 0x200 /* (RO) Shader core power transition bitmap, low word */
#define SHADER_PWRTRANS_HI 0x204 /* (RO) Shader core power transition bitmap, high word */

#define TILER_PWRTRANS_LO 0x210 /* (RO) Tiler core power transition bitmap, low word */
#define TILER_PWRTRANS_HI 0x214 /* (RO) Tiler core power transition bitmap, high word */

#define L2_PWRTRANS_LO 0x220 /* (RO) Level 2 cache power transition bitmap, low word */
#define L2_PWRTRANS_HI 0x224 /* (RO) Level 2 cache power transition bitmap, high word */

#define L2_SLICE_HASH_0 0x02C0
#define L2_SLICE_HASH(n) (L2_SLICE_HASH_0 + (n)*4)
#define L2_SLICE_HASH_COUNT 3
/* ASN_HASH aliases to L2_SLICE_HASH */
#define ASN_HASH_0 L2_SLICE_HASH_0
#define ASN_HASH(n) L2_SLICE_HASH(n)


#define SYSC_ALLOC0 0x0340 /* (RW) System cache allocation hint from source ID */
#define SYSC_ALLOC(n) (SYSC_ALLOC0 + (n)*4)
#define SYSC_ALLOC_COUNT 8

#define STACK_PWRTRANS_LO 0xE40 /* (RO) Core stack power transition bitmap, low word */
#define STACK_PWRTRANS_HI 0xE44 /* (RO) Core stack power transition bitmap, high word */

#define SHADER_PWRACTIVE_LO 0x240 /* (RO) Shader core active bitmap, low word */
#define SHADER_PWRACTIVE_HI 0x244 /* (RO) Shader core active bitmap, high word */

#define TILER_PWRACTIVE_LO 0x250 /* (RO) Tiler core active bitmap, low word */
#define TILER_PWRACTIVE_HI 0x254 /* (RO) Tiler core active bitmap, high word */

#define L2_PWRACTIVE_LO 0x260 /* (RO) Level 2 cache active bitmap, low word */
#define L2_PWRACTIVE_HI 0x264 /* (RO) Level 2 cache active bitmap, high word */

#define COHERENCY_FEATURES 0x300 /* (RO) Coherency features present */
#define COHERENCY_ENABLE 0x304 /* (RW) Coherency enable */

#define AMBA_FEATURES 0x300 /* (RO) AMBA bus supported features */
#define AMBA_ENABLE 0x304 /* (RW) AMBA features enable */

#define SHADER_CONFIG 0xF04 /* (RW) Shader core configuration (implementation-specific) */
#define TILER_CONFIG 0xF08 /* (RW) Tiler core configuration (implementation-specific) */
#define L2_MMU_CONFIG 0xF0C /* (RW) L2 cache and MMU configuration (implementation-specific) */

/* Job control registers */

#define JOB_CONTROL_BASE 0x1000
#define JOB_CONTROL_REG(r) (JOB_CONTROL_BASE + (r))

#define JOB_IRQ_RAWSTAT 0x000 /* Raw interrupt status register */
#define JOB_IRQ_CLEAR 0x004 /* Interrupt clear register */
#define JOB_IRQ_MASK 0x008 /* Interrupt mask register */
#define JOB_IRQ_STATUS 0x00C /* Interrupt status register */

/* MMU control registers */

#define MMU_CONTROL_BASE 0x2000
#define MMU_CONTROL_REG(r) (MMU_CONTROL_BASE + (r))

#define MMU_IRQ_RAWSTAT 0x000 /* (RW) Raw interrupt status register */
#define MMU_IRQ_CLEAR 0x004 /* (WO) Interrupt clear register */
#define MMU_IRQ_MASK 0x008 /* (RW) Interrupt mask register */
#define MMU_IRQ_STATUS 0x00C /* (RO) Interrupt status register */

#define MMU_AS0 0x400 /* Configuration registers for address space 0 */
#define MMU_AS1 0x440 /* Configuration registers for address space 1 */
#define MMU_AS2 0x480 /* Configuration registers for address space 2 */
#define MMU_AS3 0x4C0 /* Configuration registers for address space 3 */
#define MMU_AS4 0x500 /* Configuration registers for address space 4 */
#define MMU_AS5 0x540 /* Configuration registers for address space 5 */
#define MMU_AS6 0x580 /* Configuration registers for address space 6 */
#define MMU_AS7 0x5C0 /* Configuration registers for address space 7 */
#define MMU_AS8 0x600 /* Configuration registers for address space 8 */
#define MMU_AS9 0x640 /* Configuration registers for address space 9 */
#define MMU_AS10 0x680 /* Configuration registers for address space 10 */
#define MMU_AS11 0x6C0 /* Configuration registers for address space 11 */
#define MMU_AS12 0x700 /* Configuration registers for address space 12 */
#define MMU_AS13 0x740 /* Configuration registers for address space 13 */
#define MMU_AS14 0x780 /* Configuration registers for address space 14 */
#define MMU_AS15 0x7C0 /* Configuration registers for address space 15 */

#define MMU_STAGE1 0x2000
#define MMU_STAGE1_REG(r) (MMU_STAGE1 + (r))
#define MMU_AS_REG(n, r) (MMU_AS0 + ((n) << 6) + (r))

#define AS_TRANSTAB_LO 0x00 /* (RW) Translation Table Base Address for address space n, low word */
#define AS_TRANSTAB_HI 0x04 /* (RW) Translation Table Base Address for address space n, high word */
#define AS_MEMATTR_LO 0x08 /* (RW) Memory attributes for address space n, low word. */
#define AS_MEMATTR_HI 0x0C /* (RW) Memory attributes for address space n, high word. */
#define AS_LOCKADDR_LO 0x10 /* (RW) Lock region address for address space n, low word */
#define AS_LOCKADDR_HI 0x14 /* (RW) Lock region address for address space n, high word */
#define AS_COMMAND 0x18 /* (WO) MMU command register for address space n */
#define AS_FAULTSTATUS 0x1C /* (RO) MMU fault status register for address space n */
#define AS_FAULTADDRESS_LO 0x20 /* (RO) Fault Address for address space n, low word */
#define AS_FAULTADDRESS_HI 0x24 /* (RO) Fault Address for address space n, high word */
#define AS_STATUS 0x28 /* (RO) Status flags for address space n */
#define AS_TRANSCFG_LO 0x30 /* (RW) Translation table configuration for address space n, low word */
#define AS_TRANSCFG_HI \
	0x34 /* (RW) Translation table configuration for address space n, high word */
#define AS_FAULTEXTRA_LO 0x38 /* (RO) Secondary fault address for address space n, low word */
#define AS_FAULTEXTRA_HI 0x3C /* (RO) Secondary fault address for address space n, high word */


#endif /* _MALI_KBASE_REGMAP_LEGACY_H_ */
