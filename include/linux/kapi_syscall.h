/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * Spec lookup and syscall validation entry points of the kernel API
 * specification framework, kept apart from <linux/kernel_api_spec.h> so that
 * <linux/syscalls.h> can declare them cheaply.
 */

#ifndef _LINUX_KAPI_SYSCALL_H
#define _LINUX_KAPI_SYSCALL_H

#include <linux/types.h>

struct kernel_api_spec;

const struct kernel_api_spec *kapi_get_spec(const char *name);

#ifdef CONFIG_KAPI_RUNTIME_CHECKS
int kapi_validate_syscall_params(const struct kernel_api_spec *spec,
				 const s64 *params, int param_count);
int kapi_validate_syscall_return(const struct kernel_api_spec *spec, s64 retval);
#else
static inline int kapi_validate_syscall_params(const struct kernel_api_spec *spec,
					       const s64 *params, int param_count)
{
	return 0;
}
static inline int kapi_validate_syscall_return(const struct kernel_api_spec *spec, s64 retval)
{
	return 0;
}
#endif

#endif /* _LINUX_KAPI_SYSCALL_H */
