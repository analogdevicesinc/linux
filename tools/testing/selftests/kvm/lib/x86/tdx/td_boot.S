/* SPDX-License-Identifier: GPL-2.0-only */

#include "tdx/td_boot.h"
#include "tdx/td_boot_offsets.h"
#include "processor_asm.h"

.code32

.globl td_boot
td_boot:
	/*
	 * Paging is turned off.
	 *
	 * Register usage throughout td_boot:
	 * ebx: Address of global boot parameters (Must be preserved)
	 * eax: Address of per-vCPU parameters (Must be preserved)
	 * edi, edx: Temporary scratch registers
	 */
	cli

	movl $TD_BOOT_PARAMETERS_GPA, %ebx

	/*
	 * Find the address of struct td_per_vcpu_parameters for this
	 * vCPU based on esi (TDX spec: initialized with vCPU id). Put
	 * struct address into eax for indirect addressing.
	 */
	movl $SIZEOF_TD_PER_VCPU_PARAMETERS, %eax
	mul %esi
	leal TD_BOOT_PARAMETERS_PER_VCPU(%ebx), %edi
	addl %edi, %eax

	/* Setup stack. */
	movl TD_PER_VCPU_PARAMETERS_ESP_GVA(%eax), %esp

	/* Setup GDT. */
	leal TD_BOOT_PARAMETERS_GDT(%ebx), %edi
	lgdt (%edi)

	/* Setup IDT. */
	leal TD_BOOT_PARAMETERS_IDT(%ebx), %edi
	lidt (%edi)

	/*
	 * Set up control registers (x86 does not support moving memory
	 * directly to a control register).
	 */
	movl TD_BOOT_PARAMETERS_CR4(%ebx), %edi
	movl %edi, %cr4
	movl TD_BOOT_PARAMETERS_CR3(%ebx), %edi
	movl %edi, %cr3
	movl TD_BOOT_PARAMETERS_CR0(%ebx), %edi
	movl %edi, %cr0

	/* Switching to 64bit mode after ljmp and then jump to guest code */
	ljmp $(KERNEL_CS),$1f
1:
	jmp *TD_PER_VCPU_PARAMETERS_GUEST_CODE(%eax)

/* Leave marker so size of td_boot code can be computed. */
.globl td_boot_code_end
td_boot_code_end:

/* Disable executable stack. */
.section .note.GNU-stack,"",%progbits
