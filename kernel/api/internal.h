/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * Internal declarations shared by the KAPI core and its debugfs
 * interface. Not part of the public kernel API.
 */

#ifndef _KERNEL_API_INTERNAL_H
#define _KERNEL_API_INTERNAL_H

#include <linux/kernel_api_spec.h>

/*
 * Section boundaries for the `.kapi_specs` array. Defined by the
 * linker script in include/asm-generic/vmlinux.lds.h.
 */
extern const struct kernel_api_spec * const __start_kapi_specs[];
extern const struct kernel_api_spec * const __stop_kapi_specs[];

const char *kapi_param_type_to_string(enum kapi_param_type type);
const char *kapi_lock_type_to_string(enum kapi_lock_type type);
const char *kapi_lock_scope_to_string(enum kapi_lock_scope scope);

#endif /* _KERNEL_API_INTERNAL_H */
