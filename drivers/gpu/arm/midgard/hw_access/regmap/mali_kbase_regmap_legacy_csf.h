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

#ifndef _MALI_KBASE_REGMAP_LEGACY_CSF_H_
#define _MALI_KBASE_REGMAP_LEGACY_CSF_H_

#if !MALI_USE_CSF && defined(__KERNEL__)
#error "Cannot be compiled with JM"
#endif
/* GPU control registers */
#define MCU_CONTROL 0x700

/* GPU_CONTROL_MCU base address */
#define GPU_CONTROL_MCU_BASE 0x3000

/* MCU_SUBSYSTEM base address */
#define MCU_SUBSYSTEM_BASE 0x20000

/* IPA control registers */
#define IPA_CONTROL_BASE 0x40000
#define IPA_CONTROL_REG(r) (IPA_CONTROL_BASE + (r))

#define COMMAND 0x000 /* (WO) Command register */
#define STATUS 0x004 /* (RO) Status register */
#define TIMER 0x008 /* (RW) Timer control register */

#define SELECT_CSHW_LO 0x010 /* (RW) Counter select for CS hardware, low word */
#define SELECT_CSHW_HI 0x014 /* (RW) Counter select for CS hardware, high word */
#define SELECT_MEMSYS_LO 0x018 /* (RW) Counter select for Memory system, low word */
#define SELECT_MEMSYS_HI 0x01C /* (RW) Counter select for Memory system, high word */
#define SELECT_TILER_LO 0x020 /* (RW) Counter select for Tiler cores, low word */
#define SELECT_TILER_HI 0x024 /* (RW) Counter select for Tiler cores, high word */
#define SELECT_SHADER_LO 0x028 /* (RW) Counter select for Shader cores, low word */
#define SELECT_SHADER_HI 0x02C /* (RW) Counter select for Shader cores, high word */

/* Accumulated counter values for CS hardware */
#define VALUE_CSHW_BASE 0x100
#define VALUE_CSHW_REG_LO(n) (VALUE_CSHW_BASE + ((n) << 3)) /* (RO) Counter value #n, low word */
#define VALUE_CSHW_REG_HI(n) \
	(VALUE_CSHW_BASE + ((n) << 3) + 4) /* (RO) Counter value #n, high word */

/* Accumulated counter values for memory system */
#define VALUE_MEMSYS_BASE 0x140
#define VALUE_MEMSYS_REG_LO(n) \
	(VALUE_MEMSYS_BASE + ((n) << 3)) /* (RO) Counter value #n, low word */
#define VALUE_MEMSYS_REG_HI(n) \
	(VALUE_MEMSYS_BASE + ((n) << 3) + 4) /* (RO) Counter value #n, high word */

#define VALUE_TILER_BASE 0x180
#define VALUE_TILER_REG_LO(n) (VALUE_TILER_BASE + ((n) << 3)) /* (RO) Counter value #n, low word */
#define VALUE_TILER_REG_HI(n) \
	(VALUE_TILER_BASE + ((n) << 3) + 4) /* (RO) Counter value #n, high word */

#define VALUE_SHADER_BASE 0x1C0
#define VALUE_SHADER_REG_LO(n) \
	(VALUE_SHADER_BASE + ((n) << 3)) /* (RO) Counter value #n, low word */
#define VALUE_SHADER_REG_HI(n) \
	(VALUE_SHADER_BASE + ((n) << 3) + 4) /* (RO) Counter value #n, high word */

/* Configuration bits for the CSF. */
#define CSF_CONFIG 0xF00

/* GPU control registers */
#define CORE_FEATURES 0x008 /* () Shader Core Features */
#define MCU_STATUS 0x704

#endif /* _MALI_KBASE_REGMAP_LEGACY_CSF_H_ */
