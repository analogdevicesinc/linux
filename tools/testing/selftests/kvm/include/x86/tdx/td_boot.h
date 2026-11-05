/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SELFTEST_TDX_TD_BOOT_H
#define SELFTEST_TDX_TD_BOOT_H

/*
 * Layout for boot section (not to scale)
 *
 *                                   GPA
 * _________________________________ 0x1_0000_0000 (4GB)
 * |   Boot code trampoline    |
 * |___________________________|____ 0x0_ffff_fff0: Reset vector (16B below 4GB)
 * |   Boot code               |
 * |___________________________|____ td_boot will be copied here, so that the
 * |                           |     jmp to td_boot is exactly at the reset vector
 * |   Empty space             |
 * |                           |
 * |───────────────────────────|
 * |                           |
 * |                           |
 * |   Boot parameters         |
 * |                           |
 * |                           |
 * |___________________________|____ 0x0_ffff_0000: TD_BOOT_PARAMETERS_GPA
 *
 * TD_BOOT_PARAMETERS_GPA is arbitrarily chosen to
 *
 * + be within the 4GB address space
 * + provide enough contiguous memory for the struct td_boot_parameters such
 *   that there is one struct td_per_vcpu_parameters for KVM_MAX_VCPUS
 */
#define TD_BOOT_PARAMETERS_GPA 0xffff0000

#ifndef __ASSEMBLER__

#include <linux/compiler.h>
#include <linux/types.h>

/*
 * The exact memory layout for LGDT or LIDT instructions.
 */
struct __packed td_boot_parameters_dtr {
	u16 limit;
	u32 base;
};

/*
 * Allows each vCPU to be initialized with different rip and esp.
 */
struct td_per_vcpu_parameters {
	u32 esp_gva;
	u64 guest_code;
};

/*
 * Boot parameters for the TD.
 *
 * Unlike a regular VM, KVM cannot set registers such as esp, eip, etc
 * before boot, so to run selftests, these registers' values have to be
 * initialized by the TD.
 *
 * This struct is loaded in TD private memory at TD_BOOT_PARAMETERS_GPA.
 *
 * The TD boot code will read off parameters from this struct and set up the
 * vCPU for executing selftests.
 */
struct td_boot_parameters {
	u32 cr0;
	u32 cr3;
	u32 cr4;
	struct td_boot_parameters_dtr gdtr;
	struct td_boot_parameters_dtr idtr;
	struct td_per_vcpu_parameters per_vcpu[];
};

extern u8 td_boot[], td_boot_code_end[];

#define TD_BOOT_CODE_SIZE (td_boot_code_end - td_boot)

#endif /* #ifndef __ASSEMBLER__ */

#endif /* SELFTEST_TDX_TD_BOOT_H */
