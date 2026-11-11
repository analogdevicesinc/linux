// SPDX-License-Identifier: GPL-2.0
/* Alpha syscall entry and result encoding. */
#include <linux/entry-common.h>
#include <linux/nospec.h>
#include <linux/syscalls.h>

#include <asm/syscall.h>

#include "proto.h"

typedef long (*syscall_fn_t)(unsigned long, unsigned long, unsigned long,
			   unsigned long, unsigned long, unsigned long);

static void alpha_set_syscall_result(struct pt_regs *regs, long ret)
{
	struct thread_info *ti = current_thread_info();
	unsigned long meta = ti->syscall_meta;

	ti->syscall_meta = 0;
	if (meta & (ALPHA_SYSCALL_META_RESTORED | ALPHA_SYSCALL_META_ROLLBACK)) {
		ti->syscall_saved_nr = 0;
		ti->syscall_saved_r19 = 0;
		return;
	}

	/* Entry work may have replaced argument 4 with the a3 error flag. */
	if (!(meta & ALPHA_SYSCALL_META_SKIP))
		ti->syscall_saved_r19 = regs->r19;

	if (meta & ALPHA_SYSCALL_META_SKIP)
		return;

	/* A successful ptrace PEEK can return data that looks like an errno. */
	if (meta & ALPHA_SYSCALL_META_FORCE_SUCCESS) {
		regs->r0 = ret;
		regs->r19 = 0;
	} else if (ret < 0) {
		regs->r0 = -ret;
		regs->r19 = 1;
	} else {
		regs->r0 = ret;
		regs->r19 = 0;
	}
}

/* Called with interrupts disabled and a complete user switch_stack. */
asmlinkage noinstr void alpha_syscall(struct pt_regs *regs)
{
	struct thread_info *ti = current_thread_info();
	unsigned long saved_r19 = regs->r19;
	long nr = regs->r0;
	long ret = -ENOSYS;
	bool permit;

	regs->r1 = nr;
	ti->syscall_meta = 0;
	ti->syscall_saved_nr = nr;
	ti->syscall_saved_r19 = saved_r19;

	if (!(ti->status & TS_SAVED_FP)) {
		ti->status |= TS_SAVED_FP;
		__save_fpu();
	}

	permit = syscall_enter_from_user_mode_randomize_stack(regs, &nr);

	instrumentation_begin();
	if (unlikely(!permit || nr == -1)) {
		/* r19 is still argument 4 unless a hook supplied a result. */
		if (!(ti->syscall_meta & (ALPHA_SYSCALL_META_RESULT |
					 ALPHA_SYSCALL_META_ROLLBACK)))
			syscall_set_return_value(current, regs, -ENOSYS, 0);
		ti->syscall_meta |= ALPHA_SYSCALL_META_SKIP;
	} else if (likely((unsigned long)nr < NR_syscalls)) {
		syscall_fn_t syscall_fn;

		syscall_fn = sys_call_table[array_index_nospec(nr, NR_syscalls)];
		ret = syscall_fn(regs->r16, regs->r17, regs->r18,
				 regs->r19, regs->r20, regs->r21);
	}
	alpha_set_syscall_result(regs, ret);
	instrumentation_end();

	syscall_exit_to_user_mode(regs);
}
