/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_PTR_EQ_H
#define _ASM_X86_PTR_EQ_H

#include <linux/types.h>
#include <asm/asm.h>

static __always_inline
bool ptr_eq(const volatile void *a, const volatile void *b)
{
	bool ret;
	asm(__ASM_SIZE(cmp) " %1,%2" : "=@ccz" (ret) : "r" (a), "r" (b));
	return ret;
}

#endif	/* _ASM_X86_PTR_EQ_H */
