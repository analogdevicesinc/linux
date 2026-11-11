/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_GENERIC_PTR_EQ_H
#define __ASM_GENERIC_PTR_EQ_H

#include <linux/types.h>
#include <linux/compiler.h>

static __always_inline
bool ptr_eq(const volatile void *a, const volatile void *b)
{
	OPTIMIZER_HIDE_VAR(a);
	OPTIMIZER_HIDE_VAR(b);
	return a == b;
}

#endif	/* __ASM_GENERIC_PTR_EQ_H */
