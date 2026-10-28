/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_S390_FUTEX_ROBUST_H
#define _ASM_S390_FUTEX_ROBUST_H

#include <asm/ptrace.h>

#define VDSO_FUTEX_ROBUST_POP_GPR	4

static __always_inline void __user *s390_futex_robust_unlock_get_pop(struct pt_regs *regs)
{
	/*
	 * Within critical section of
	 * __vdso_futex_robust_list64_try_unlock(__u32 *lock, __u32 tid, __u64 *pop)
	 *
	 * If condition code is 0, cmpxchg succeeded, and the pending
	 * op pointer in gpr4 needs to be returned for clearing.
	 */
	if (psw_bits(regs->psw).cc != 0)
		return NULL;
	return (void __user *)regs->gprs[VDSO_FUTEX_ROBUST_POP_GPR];
}

#define arch_futex_robust_unlock_get_pop(regs)	s390_futex_robust_unlock_get_pop(regs)

#endif /* _ASM_S390_FUTEX_ROBUST_H */
