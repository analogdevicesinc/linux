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

void tdx_init_vm(struct kvm_vm *vm);

#endif /* SELFTESTS_TDX_TDX_UTIL_H */
