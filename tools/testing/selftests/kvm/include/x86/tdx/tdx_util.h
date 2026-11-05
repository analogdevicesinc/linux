/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SELFTESTS_TDX_TDX_UTIL_H
#define SELFTESTS_TDX_TDX_UTIL_H

#include <inttypes.h>
#include <stdbool.h>

#include "kvm_util.h"

static inline bool is_tdx_vm(struct kvm_vm *vm)
{
	return vm->type == KVM_X86_TDX_VM;
}

/*
 * TDX ioctls
 * Use underscores to avoid collisions with struct member names.
 */
#define __tdx_vm_ioctl(vm, cmd, _flags, arg, hw_err)			\
({									\
	u64 *__hw_err = (hw_err);					\
	int r;								\
									\
	union {								\
		struct kvm_tdx_cmd c;					\
		unsigned long raw;					\
	} tdx_cmd = { .c = {						\
		.id = (cmd),						\
		.flags = (u32)(_flags),					\
		.data = (u64)(arg),					\
	} };								\
									\
	r = __vm_ioctl(vm, KVM_MEMORY_ENCRYPT_OP, &tdx_cmd.raw);	\
	if (__hw_err)							\
		*__hw_err = tdx_cmd.c.hw_error;				\
	r;								\
})

#define tdx_vm_ioctl(vm, cmd, flags, arg)				\
({									\
	u64 hw_error;							\
	int ret = __tdx_vm_ioctl(vm, cmd, flags, arg, &hw_error);	\
									\
	TEST_ASSERT(!ret,						\
		    "%s failed, rc: %d errno: %i (%s) hw_error: 0x%" PRIx64, \
		    #cmd, ret, errno, strerror(errno), hw_error);	\
})

#define __tdx_vcpu_ioctl(vcpu, cmd, _flags, arg)			\
({									\
	union {								\
		struct kvm_tdx_cmd c;					\
		unsigned long raw;					\
	} tdx_cmd = { .c = {						\
		.id = (cmd),						\
		.flags = (u32)(_flags),					\
		.data = (u64)(arg),					\
	} };								\
									\
	__vcpu_ioctl(vcpu, KVM_MEMORY_ENCRYPT_OP, &tdx_cmd.raw);	\
})

#define tdx_vcpu_ioctl(vcpu, cmd, flags, arg)				\
({									\
	int ret = __tdx_vcpu_ioctl(vcpu, cmd, flags, arg);		\
									\
	TEST_ASSERT(!ret, "%s failed, rc: %i errno: %i (%s)",		\
		    #cmd, ret, errno, strerror(errno));			\
})
void tdx_init_vm(struct kvm_vm *vm);
void tdx_vm_setup_boot_code_region(struct kvm_vm *vm);
void tdx_vm_setup_boot_parameters_region(struct kvm_vm *vm, u32 nr_runnable_vcpus);
void tdx_vm_load_common_boot_parameters(struct kvm_vm *vm);
void tdx_vcpu_init(struct kvm_vcpu *vcpu);
void tdx_vcpu_set_entry_point(struct kvm_vcpu *vcpu, void *guest_code);

#endif /* SELFTESTS_TDX_TDX_UTIL_H */
