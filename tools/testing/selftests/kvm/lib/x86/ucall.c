// SPDX-License-Identifier: GPL-2.0
/*
 * ucall support. A ucall is a "hypercall to userspace".
 *
 * Copyright (C) 2018, Red Hat, Inc.
 */
#include "kvm_util.h"
#include "tdx/tdx.h"
#include "tdx/tdx_util.h"

#define UCALL_PIO_PORT		((u16)0x1000)
#define UCALL_TDX_MAGIC		0xabacadabaULL

static void ucall_tdx_do_ucall(gva_t uc)
{
	__tdvmcall(TDVMCALL_REPORT_FATAL_ERROR, UCALL_TDX_MAGIC, uc, 0, 0);
}

static void *ucall_tdx_get_ucall(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;

	/*
	 * KVM_SYSTEM_EVENT_TDX_FATAL stores R0..R15 in data[0..15].  R12
	 * holds the ucall magic and R13 holds the ucall GVA.
	 */
	if (run->exit_reason == KVM_EXIT_SYSTEM_EVENT &&
	    run->system_event.type == KVM_SYSTEM_EVENT_TDX_FATAL &&
	    run->system_event.data[12] == UCALL_TDX_MAGIC)
		return (void *)(run->system_event.data[13]);

	return NULL;
}

static void ucall_x86_do_ucall(gva_t uc)
{
	asm volatile("in %[port], %%al"
		: : [port] "d" (UCALL_PIO_PORT), "D" (uc) : "rax", "memory");
}

static void *ucall_x86_get_ucall(struct kvm_vcpu *vcpu)
{
	struct kvm_run *run = vcpu->run;

	if (run->exit_reason == KVM_EXIT_IO && run->io.port == UCALL_PIO_PORT) {
		struct kvm_regs regs;

		vcpu_regs_get(vcpu, &regs);
		return (void *)regs.rdi;
	}
	return NULL;
}

static bool ucall_is_tdx_vm;

void ucall_arch_init(struct kvm_vm *vm, gpa_t mmio_gpa)
{
	write_guest_global(vm, ucall_is_tdx_vm, is_tdx_vm(vm));
}

void ucall_arch_do_ucall(gva_t uc)
{
	if (ucall_is_tdx_vm)
		return ucall_tdx_do_ucall(uc);

	return ucall_x86_do_ucall(uc);
}

void *ucall_arch_get_ucall(struct kvm_vcpu *vcpu)
{
	if (is_tdx_vm(vcpu->vm))
		return ucall_tdx_get_ucall(vcpu);

	return ucall_x86_get_ucall(vcpu);
}
