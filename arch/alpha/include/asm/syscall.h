/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_ALPHA_SYSCALL_H
#define _ASM_ALPHA_SYSCALL_H

#include <uapi/linux/audit.h>
#include <linux/audit.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <asm/ptrace.h>

extern void *sys_call_table[];

static inline int syscall_get_arch(struct task_struct *task)
{
	return AUDIT_ARCH_ALPHA;
}

static inline long syscall_get_return_value(struct task_struct *task,
					    struct pt_regs *regs)
{
	return regs->r19 ? -(long)regs->r0 : (long)regs->r0;
}

static inline long syscall_get_error(struct task_struct *task,
				     struct pt_regs *regs)
{
	return regs->r19 ? -(long)regs->r0 : 0;
}


/*
 * The syscall number arrives in r0. Entry saves the original in thread_info
 * for rollback and restart, while tracers can change the number in r1.
 */

static inline long syscall_get_nr(struct task_struct *task,
				struct pt_regs *regs)
{
	return (long)regs->r1;
}

static inline void syscall_set_nr(struct task_struct *task,
				struct pt_regs *regs,
				long nr)
{
	regs->r1 = (unsigned long)nr;
}

/*
 * Syscall arguments:
 *   regs->r16..regs->r21 carry up to 6 syscall arguments on entry.
 *   Note: regs->r19 is also used as "a3" (error flag) on syscall return.
 */

static inline void syscall_get_arguments(struct task_struct *task,
					struct pt_regs *regs,
					unsigned long *args)
{
	args[0] = regs->r16;
	args[1] = regs->r17;
	args[2] = regs->r18;
	args[3] = regs->r19;
	args[4] = regs->r20;
	args[5] = regs->r21;
}

static inline void syscall_set_arguments(struct task_struct *task,
					struct pt_regs *regs,
					const unsigned long *args)
{
	regs->r16 = args[0];
	regs->r17 = args[1];
	regs->r18 = args[2];
	regs->r19 = args[3];
	regs->r20 = args[4];
	regs->r21 = args[5];
}
/*
 * On error, r0 holds a positive errno and a3 is set.
 * On success, r0 holds the result and a3 is clear.
 */

static inline void syscall_set_return_value(struct task_struct *task,
					struct pt_regs *regs,
					int error, long val)
{
	task_thread_info(task)->syscall_meta |= ALPHA_SYSCALL_META_RESULT;
	if (error) {
		/* error is negative errno in this tree */
		regs->r0  = (unsigned long)(-error);  /* positive errno */
		regs->r19 = 1;                        /* a3 = error */
	} else {
		regs->r0  = (unsigned long)val;
		regs->r19 = 0;                        /* a3 = success */
	}
}

/* Preserve the original syscall registers for a skipped syscall. */

static inline void syscall_rollback(struct task_struct *task,
					struct pt_regs *regs)
{
	unsigned long nr = task_thread_info(task)->syscall_saved_nr;

	regs->r0 = nr;
	regs->r1 = nr;
	task_thread_info(task)->syscall_meta |= ALPHA_SYSCALL_META_ROLLBACK;
}

static inline bool arch_syscall_is_vdso_sigreturn(struct pt_regs *regs)
{
	return false;
}
#endif	/* _ASM_ALPHA_SYSCALL_H */
