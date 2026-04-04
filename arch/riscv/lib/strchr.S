/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * Copyright (C) 2025 Feng Jiang <jiangfeng@kylinos.cn>
 */

#include <linux/linkage.h>
#include <asm/asm.h>

/* char *strchr(const char *s, int c) */
SYM_FUNC_START(strchr)
	/*
	 * Parameters
	 *   a0 - The string to be searched
	 *   a1 - The character to search for
	 *
	 * Returns
	 *   a0 - Address of first occurrence of 'c' or 0
	 *
	 * Clobbers
	 *   t0
	 */
	andi	a1, a1, 0xff
1:
	lbu	t0, 0(a0)
	beq	t0, a1, 2f
	addi	a0, a0, 1
	bnez	t0, 1b
	li	a0, 0
2:
	ret
SYM_FUNC_END(strchr)

SYM_FUNC_ALIAS_WEAK(__pi_strchr, strchr)
EXPORT_SYMBOL(strchr)
