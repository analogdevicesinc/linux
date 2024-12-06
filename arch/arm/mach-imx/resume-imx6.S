/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright 2014 Freescale Semiconductor, Inc.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>
#include <asm/asm-offsets.h>
#include <asm/hardware/cache-l2x0.h>
#include "hardware.h"

.arch armv7-a

/*
 * The following code must assume it is running from physical address
 * where absolute virtual addresses to the data section have to be
 * turned into relative ones.
 */
	.macro is_cortex_a7

	/* Read the primary cpu number is MPIDR */
	mrc     p15, 0, r5, c0, c0, 0
	ldr     r6, =0xfff0
	and     r5, r5, r6
	ldr     r6, =0xc070
	cmp     r5, r6

	.endm

ENTRY(v7_cpu_resume)
	bl	v7_invalidate_l1
	is_cortex_a7
	beq	done
#ifdef CONFIG_CACHE_L2X0
	bl	l2c310_early_resume
#endif
done:
	b	cpu_resume
ENDPROC(v7_cpu_resume)
