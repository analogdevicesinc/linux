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

#ifndef _MALI_KBASE_REGMAP_JM_MACROS_H_
#define _MALI_KBASE_REGMAP_JM_MACROS_H_

#if MALI_USE_CSF
#error "Cannot be compiled with CSF"
#endif

#define ENUM_OFFSET(_index, _base, _next) (_base + _index * (_next - _base))

#define GPU_CONTROL_ENUM(regname) GPU_CONTROL__##regname
#define GPU_TEXTURE_FEATURES_ENUM(n) GPU_CONTROL_ENUM(TEXTURE_FEATURES_##n)
#define GPU_TEXTURE_FEATURES_OFFSET(n) (GPU_TEXTURE_FEATURES_ENUM(0) + n)
#define GPU_JS_FEATURES_ENUM(n) GPU_CONTROL_ENUM(JS##n##_FEATURES)
#define GPU_JS_FEATURES_OFFSET(n) (GPU_JS_FEATURES_ENUM(0) + n)

#define JOB_CONTROL_ENUM(regname) JOB_CONTROL__##regname
#define JOB_SLOT_ENUM(n, regname) JOB_CONTROL_ENUM(JS##n##__##regname)
#define JOB_SLOT_BASE_ENUM(n) JOB_SLOT_ENUM(n, HEAD)
#define JOB_SLOT_OFFSET(n, regname) \
	ENUM_OFFSET(n, JOB_SLOT_ENUM(0, regname), JOB_SLOT_ENUM(1, regname))
#define JOB_SLOT_BASE_OFFSET(n) JOB_SLOT_OFFSET(n, HEAD)

#define MMU_CONTROL_ENUM(regname) MMU_STAGE1__ST1MMU__##regname
#define MMU_AS_ENUM(n, regname) MMU_CONTROL_ENUM(AS##n##__##regname)
#define MMU_AS_BASE_ENUM(n) MMU_AS_ENUM(n, TRANSTAB)
#define MMU_AS_OFFSET(n, regname) ENUM_OFFSET(n, MMU_AS_ENUM(0, regname), MMU_AS_ENUM(1, regname))
#define MMU_AS_BASE_OFFSET(n) MMU_AS_OFFSET(n, TRANSTAB)

/* register value macros */
/* GPU_STATUS values */
#define GPU_STATUS_PRFCNT_ACTIVE (1 << 2) /* Set if the performance counters are active. */
#define GPU_STATUS_CYCLE_COUNT_ACTIVE (1 << 6) /* Set if the cycle counter is active. */

/* PRFCNT_CONFIG register values */
#define PRFCNT_CONFIG_MODE_SHIFT 0 /* Counter mode position. */
#define PRFCNT_CONFIG_AS_SHIFT 4 /* Address space bitmap position. */
#define PRFCNT_CONFIG_SETSELECT_SHIFT 8 /* Set select position. */

/* The performance counters are disabled. */
#define PRFCNT_CONFIG_MODE_OFF 0
/* The performance counters are enabled, but are only written out when a
 * PRFCNT_SAMPLE command is issued using the GPU_COMMAND register.
 */
#define PRFCNT_CONFIG_MODE_MANUAL 1
/* The performance counters are enabled, and are written out each time a tile
 * finishes rendering.
 */
#define PRFCNT_CONFIG_MODE_TILE 2

/*
 * Begin AARCH64 MMU TRANSTAB register values
 */
#define AS_TRANSTAB_BASE_SHIFT GPU_U(4)
#define AS_TRANSTAB_BASE_MASK (GPU_ULL(0xFFFFFFFFFFFFFFF) << AS_TRANSTAB_BASE_SHIFT)
#define AS_TRANSTAB_BASE_GET(reg_val) (((reg_val)&AS_TRANSTAB_BASE_MASK) >> AS_TRANSTAB_BASE_SHIFT)
#define AS_TRANSTAB_BASE_SET(reg_val, value)     \
	(~(~(reg_val) | AS_TRANSTAB_BASE_MASK) | \
	 (((uint64_t)(value) << AS_TRANSTAB_BASE_SHIFT) & AS_TRANSTAB_BASE_MASK))

#define AS_FAULTSTATUS_EXCEPTION_TYPE_OK 0x0
#define AS_FAULTSTATUS_EXCEPTION_TYPE_DONE 0x1
#define AS_FAULTSTATUS_EXCEPTION_TYPE_STOPPED 0x3
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TERMINATED 0x4
#define AS_FAULTSTATUS_EXCEPTION_TYPE_KABOOM 0x5
#define AS_FAULTSTATUS_EXCEPTION_TYPE_EUREKA 0x6
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ACTIVE 0x8
#define AS_FAULTSTATUS_EXCEPTION_TYPE_JOB_CONFIG_FAULT 0x40
#define AS_FAULTSTATUS_EXCEPTION_TYPE_JOB_POWER_FAULT 0x41
#define AS_FAULTSTATUS_EXCEPTION_TYPE_JOB_READ_FAULT 0x42
#define AS_FAULTSTATUS_EXCEPTION_TYPE_JOB_WRITE_FAULT 0x43
#define AS_FAULTSTATUS_EXCEPTION_TYPE_JOB_AFFINITY_FAULT 0x44
#define AS_FAULTSTATUS_EXCEPTION_TYPE_JOB_BUS_FAULT 0x48
#define AS_FAULTSTATUS_EXCEPTION_TYPE_INSTR_INVALID_PC 0x50
#define AS_FAULTSTATUS_EXCEPTION_TYPE_INSTR_INVALID_ENC 0x51
#define AS_FAULTSTATUS_EXCEPTION_TYPE_INSTR_BARRIER_FAULT 0x55
#define AS_FAULTSTATUS_EXCEPTION_TYPE_DATA_INVALID_FAULT 0x58
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TILE_RANGE_FAULT 0x59
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDR_RANGE_FAULT 0x5A
#define AS_FAULTSTATUS_EXCEPTION_TYPE_IMPRECISE_FAULT 0x5B
#define AS_FAULTSTATUS_EXCEPTION_TYPE_OUT_OF_MEMORY 0x60
#define AS_FAULTSTATUS_EXCEPTION_TYPE_UNKNOWN 0x7F
#define AS_FAULTSTATUS_EXCEPTION_TYPE_DELAYED_BUS_FAULT 0x80
#define AS_FAULTSTATUS_EXCEPTION_TYPE_GPU_SHAREABILITY_FAULT 0x88
#define AS_FAULTSTATUS_EXCEPTION_TYPE_SYSTEM_SHAREABILITY_FAULT 0x89
#define AS_FAULTSTATUS_EXCEPTION_TYPE_GPU_CACHEABILITY_FAULT 0x8A
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSLATION_FAULT_0 0xC0
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSLATION_FAULT_1 0xC1
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSLATION_FAULT_2 0xC2
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSLATION_FAULT_3 0xC3
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSLATION_FAULT_4 0xC4
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSLATION_FAULT_IDENTITY 0xC7
#define AS_FAULTSTATUS_EXCEPTION_TYPE_PERMISSION_FAULT_0 0xC8
#define AS_FAULTSTATUS_EXCEPTION_TYPE_PERMISSION_FAULT_1 0xC9
#define AS_FAULTSTATUS_EXCEPTION_TYPE_PERMISSION_FAULT_2 0xCA
#define AS_FAULTSTATUS_EXCEPTION_TYPE_PERMISSION_FAULT_3 0xCB
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSTAB_BUS_FAULT_0 0xD0
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSTAB_BUS_FAULT_1 0xD1
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSTAB_BUS_FAULT_2 0xD2
#define AS_FAULTSTATUS_EXCEPTION_TYPE_TRANSTAB_BUS_FAULT_3 0xD3
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ACCESS_FLAG_0 0xD8
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ACCESS_FLAG_1 0xD9
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ACCESS_FLAG_2 0xDA
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ACCESS_FLAG_3 0xDB
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_IN0 0xE0
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_IN1 0xE1
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_IN2 0xE2
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_IN3 0xE3
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_OUT0 0xE4
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_OUT1 0xE5
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_OUT2 0xE6
#define AS_FAULTSTATUS_EXCEPTION_TYPE_ADDRESS_SIZE_FAULT_OUT3 0xE7
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_FAULT_0 0xE8
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_FAULT_1 0xE9
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_FAULT_2 0xEA
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_FAULT_3 0xEB
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_NONCACHEABLE_0 0xEC
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_NONCACHEABLE_1 0xED
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_NONCACHEABLE_2 0xEE
#define AS_FAULTSTATUS_EXCEPTION_TYPE_MEMORY_ATTRIBUTE_NONCACHEABLE_3 0xEF

/* No JM-specific MMU control registers */
/* No JM-specific MMU address space control registers */

/* JS_COMMAND register commands */
#define JS_COMMAND_NOP 0x00 /* NOP Operation. Writing this value is ignored */
#define JS_COMMAND_START 0x01 /* Start processing a job chain. Writing this value is ignored */
#define JS_COMMAND_SOFT_STOP 0x02 /* Gently stop processing a job chain */
#define JS_COMMAND_HARD_STOP 0x03 /* Rudely stop processing a job chain */
#define JS_COMMAND_SOFT_STOP_0 0x04 /* Execute SOFT_STOP if JOB_CHAIN_FLAG is 0 */
#define JS_COMMAND_HARD_STOP_0 0x05 /* Execute HARD_STOP if JOB_CHAIN_FLAG is 0 */
#define JS_COMMAND_SOFT_STOP_1 0x06 /* Execute SOFT_STOP if JOB_CHAIN_FLAG is 1 */
#define JS_COMMAND_HARD_STOP_1 0x07 /* Execute HARD_STOP if JOB_CHAIN_FLAG is 1 */

#define JS_COMMAND_MASK 0x07 /* Mask of bits currently in use by the HW */

/* Possible values of JS_CONFIG and JS_CONFIG_NEXT registers */
#define JS_CONFIG_START_FLUSH_NO_ACTION (0u << 0)
#define JS_CONFIG_START_FLUSH_CLEAN (1u << 8)
#define JS_CONFIG_START_FLUSH_INV_SHADER_OTHER (2u << 8)
#define JS_CONFIG_START_FLUSH_CLEAN_INVALIDATE (3u << 8)
#define JS_CONFIG_START_MMU (1u << 10)
#define JS_CONFIG_JOB_CHAIN_FLAG (1u << 11)
#define JS_CONFIG_END_FLUSH_NO_ACTION JS_CONFIG_START_FLUSH_NO_ACTION
#define JS_CONFIG_END_FLUSH_CLEAN (1u << 12)
#define JS_CONFIG_END_FLUSH_CLEAN_INVALIDATE (3u << 12)
#define JS_CONFIG_ENABLE_FLUSH_REDUCTION (1u << 14)
#define JS_CONFIG_DISABLE_DESCRIPTOR_WR_BK (1u << 15)
#define JS_CONFIG_THREAD_PRI(n) ((n) << 16)

/* JS_XAFFINITY register values */
#define JS_XAFFINITY_XAFFINITY_ENABLE (1u << 0)
#define JS_XAFFINITY_TILER_ENABLE (1u << 8)
#define JS_XAFFINITY_CACHE_ENABLE (1u << 16)

/* JS_STATUS register values */

/* NOTE: Please keep this values in sync with enum base_jd_event_code in mali_base_kernel.h.
 * The values are separated to avoid dependency of userspace and kernel code.
 */

/* Group of values representing the job status instead of a particular fault */
#define JS_STATUS_NO_EXCEPTION_BASE 0x00
#define JS_STATUS_INTERRUPTED (JS_STATUS_NO_EXCEPTION_BASE + 0x02) /* 0x02 means INTERRUPTED */
#define JS_STATUS_STOPPED (JS_STATUS_NO_EXCEPTION_BASE + 0x03) /* 0x03 means STOPPED */
#define JS_STATUS_TERMINATED (JS_STATUS_NO_EXCEPTION_BASE + 0x04) /* 0x04 means TERMINATED */

/* General fault values */
#define JS_STATUS_FAULT_BASE 0x40
#define JS_STATUS_CONFIG_FAULT (JS_STATUS_FAULT_BASE) /* 0x40 means CONFIG FAULT */
#define JS_STATUS_POWER_FAULT (JS_STATUS_FAULT_BASE + 0x01) /* 0x41 means POWER FAULT */
#define JS_STATUS_READ_FAULT (JS_STATUS_FAULT_BASE + 0x02) /* 0x42 means READ FAULT */
#define JS_STATUS_WRITE_FAULT (JS_STATUS_FAULT_BASE + 0x03) /* 0x43 means WRITE FAULT */
#define JS_STATUS_AFFINITY_FAULT (JS_STATUS_FAULT_BASE + 0x04) /* 0x44 means AFFINITY FAULT */
#define JS_STATUS_BUS_FAULT (JS_STATUS_FAULT_BASE + 0x08) /* 0x48 means BUS FAULT */

/* Instruction or data faults */
#define JS_STATUS_INSTRUCTION_FAULT_BASE 0x50
#define JS_STATUS_INSTR_INVALID_PC \
	(JS_STATUS_INSTRUCTION_FAULT_BASE) /* 0x50 means INSTR INVALID PC */
#define JS_STATUS_INSTR_INVALID_ENC \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x01) /* 0x51 means INSTR INVALID ENC */
#define JS_STATUS_INSTR_TYPE_MISMATCH \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x02) /* 0x52 means INSTR TYPE MISMATCH */
#define JS_STATUS_INSTR_OPERAND_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x03) /* 0x53 means INSTR OPERAND FAULT */
#define JS_STATUS_INSTR_TLS_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x04) /* 0x54 means INSTR TLS FAULT */
#define JS_STATUS_INSTR_BARRIER_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x05) /* 0x55 means INSTR BARRIER FAULT */
#define JS_STATUS_INSTR_ALIGN_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x06) /* 0x56 means INSTR ALIGN FAULT */
/* NOTE: No fault with 0x57 code defined in spec. */
#define JS_STATUS_DATA_INVALID_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x08) /* 0x58 means DATA INVALID FAULT */
#define JS_STATUS_TILE_RANGE_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x09) /* 0x59 means TILE RANGE FAULT */
#define JS_STATUS_ADDRESS_RANGE_FAULT \
	(JS_STATUS_INSTRUCTION_FAULT_BASE + 0x0A) /* 0x5A means ADDRESS RANGE FAULT */

/* Other faults */
#define JS_STATUS_MEMORY_FAULT_BASE 0x60
#define JS_STATUS_OUT_OF_MEMORY (JS_STATUS_MEMORY_FAULT_BASE) /* 0x60 means OUT OF MEMORY */
#define JS_STATUS_UNKNOWN 0x7F /* 0x7F means UNKNOWN */

/* JS<n>_FEATURES register */
#define JS_FEATURE_NULL_JOB (1u << 1)
#define JS_FEATURE_SET_VALUE_JOB (1u << 2)
#define JS_FEATURE_CACHE_FLUSH_JOB (1u << 3)
#define JS_FEATURE_COMPUTE_JOB (1u << 4)
#define JS_FEATURE_VERTEX_JOB (1u << 5)
#define JS_FEATURE_GEOMETRY_JOB (1u << 6)
#define JS_FEATURE_TILER_JOB (1u << 7)
#define JS_FEATURE_FUSED_JOB (1u << 8)
#define JS_FEATURE_FRAGMENT_JOB (1u << 9)

/* JM_CONFIG register */
#define JM_TIMESTAMP_OVERRIDE (1ul << 0)
#define JM_CLOCK_GATE_OVERRIDE (1ul << 1)
#define JM_JOB_THROTTLE_ENABLE (1ul << 2)
#define JM_JOB_THROTTLE_LIMIT_SHIFT (3)
#define JM_MAX_JOB_THROTTLE_LIMIT (0x3F)
#define JM_FORCE_COHERENCY_FEATURES_SHIFT (2)

/* GPU_COMMAND values */
#define GPU_COMMAND_NOP 0x00 /* No operation, nothing happens */
#define GPU_COMMAND_SOFT_RESET \
	0x01 /* Stop all external bus interfaces, and then reset the entire GPU. */
#define GPU_COMMAND_HARD_RESET 0x02 /* Immediately reset the entire GPU. */
#define GPU_COMMAND_PRFCNT_CLEAR \
	0x03 /* Clear all performance counters, setting them all to zero. */
#define GPU_COMMAND_PRFCNT_SAMPLE \
	0x04 /* Sample all performance counters, writing them out to memory */
#define GPU_COMMAND_CYCLE_COUNT_START \
	0x05 /* Starts the cycle counter, and system timestamp propagation */
#define GPU_COMMAND_CYCLE_COUNT_STOP \
	0x06 /* Stops the cycle counter, and system timestamp propagation */
#define GPU_COMMAND_CLEAN_CACHES 0x07 /* Clean all caches */
#define GPU_COMMAND_CLEAN_INV_CACHES 0x08 /* Clean and invalidate all caches */
#define GPU_COMMAND_SET_PROTECTED_MODE 0x09 /* Places the GPU in protected mode */

/* GPU_COMMAND cache flush alias to CSF command payload */
#define GPU_COMMAND_CACHE_CLN_INV_L2 GPU_COMMAND_CLEAN_INV_CACHES
#define GPU_COMMAND_CACHE_CLN_INV_L2_LSC GPU_COMMAND_CLEAN_INV_CACHES
#define GPU_COMMAND_CACHE_CLN_INV_FULL GPU_COMMAND_CLEAN_INV_CACHES
#define GPU_COMMAND_CACHE_CLN_INV_LSC GPU_COMMAND_CLEAN_INV_CACHES

/* Merge cache flush commands */
#define GPU_COMMAND_FLUSH_CACHE_MERGE(cmd1, cmd2) ((cmd1) > (cmd2) ? (cmd1) : (cmd2))

/* IRQ flags */
#define GPU_FAULT (1U << 0) /* A GPU Fault has occurred */
#define MULTIPLE_GPU_FAULTS (1U << 7) /* More than one GPU Fault occurred.  */
#define RESET_COMPLETED (1U << 8) /* Set when a reset has completed.  */
#define POWER_CHANGED_SINGLE \
	(1U << 9) /* Set when a single core has finished powering up or down. */
#define POWER_CHANGED_ALL (1U << 10) /* Set when all cores have finished powering up or down. */
#define PRFCNT_SAMPLE_COMPLETED (1U << 16) /* Set when a performance count sample has completed. */
#define CLEAN_CACHES_COMPLETED (1U << 17) /* Set when a cache clean operation has completed. */
#define FLUSH_PA_RANGE_COMPLETED \
	(1 << 20) /* Set when a physical range cache clean operation has completed. */

/*
 * In Debug build,
 * GPU_IRQ_REG_COMMON | POWER_CHANGED_SINGLE is used to clear and enable
 * interrupts sources of GPU_IRQ by writing it onto GPU_IRQ_CLEAR/MASK registers.
 *
 * In Release build,
 * GPU_IRQ_REG_COMMON is used.
 *
 * Note:
 * CLEAN_CACHES_COMPLETED - Used separately for cache operation.
 */
#define GPU_IRQ_REG_COMMON                                                       \
	(GPU_FAULT | MULTIPLE_GPU_FAULTS | RESET_COMPLETED | POWER_CHANGED_ALL | \
	 PRFCNT_SAMPLE_COMPLETED)

#endif /* _MALI_KBASE_REGMAP_JM_MACROS_H_ */
