/* SPDX-License-Identifier: GPL-2.0 */
#ifndef ARCH_ALPHA_ENTRY_COMMON_H
#define ARCH_ALPHA_ENTRY_COMMON_H

#include <asm/stacktrace.h> /* For on_thread_stack() */
#include <asm/syscall.h>
#include <asm/thread_info.h>

#define arch_exit_to_user_mode_work arch_exit_to_user_mode_work

static __always_inline void arch_exit_to_user_mode_work(struct pt_regs *regs,
				unsigned long ti_work)
{
}

#define arch_exit_to_user_mode_prepare arch_exit_to_user_mode_prepare
static inline void arch_exit_to_user_mode_prepare(struct pt_regs *regs,
						 unsigned long ti_work)
{
	/* An interrupt after returning to user mode must not restart a syscall. */
	current_thread_info()->syscall_saved_nr = 0;
}

#endif
