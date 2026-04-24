/* SPDX-License-Identifier: GPL-2.0-only */

/*
 * Copyright (C) 2025 Feng Jiang <jiangfeng@kylinos.cn>
 */

#include <linux/linkage.h>
#include <asm/asm.h>

/* char *strrchr(const char *s, int c) */
SYM_FUNC_START(strrchr)
	/*
	 * Parameters
	 *	a0 - The string to be searched
	 *	a1 - The character to seaerch for
	 *
	 * Returns
	 *	a0 - Address of last occurrence of 'c' or 0
	 *
	 * Clobbers
	 *	t0, t1
	 */
	andi	a1, a1, 0xff
	mv	t1, a0
	li	a0, 0
1:
	lbu	t0, 0(t1)
	bne	t0, a1, 2f
	mv	a0, t1
2:
	addi	t1, t1, 1
	bnez	t0, 1b
	ret
SYM_FUNC_END(strrchr)

SYM_FUNC_ALIAS_WEAK(__pi_strrchr, strrchr)
EXPORT_SYMBOL(strrchr)
