/* SPDX-License-Identifier: GPL-2.0-only */
#include <linux/linkage.h>
#include <asm/dwarf.h>

/*
 * __u32 __vdso_futex_robust_list64_try_unlock(__u32 *lock, __u32 tid, __u64 *pop)
 *
 * Returns tid on success, or lock value on failure.
 */
SYM_FUNC_START(__vdso_futex_robust_list64_try_unlock)
	CFI_STARTPROC
	lghi	%r1,0
	cs	%r3,%r1,0(%r2)
__futex_list64_try_unlock_cs_start:
	jnz	__futex_list64_try_unlock_cs_end
__futex_list64_try_unlock_cs_success:
	mvghi	0(%r4),0
__futex_list64_try_unlock_cs_end:
	llgfr	%r2,%r3
	br	%r14
	CFI_ENDPROC
SYM_FUNC_END(__vdso_futex_robust_list64_try_unlock)
