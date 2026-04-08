/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _ASM_DELAY_CONST_H
#define _ASM_DELAY_CONST_H

#include <asm/param.h>	/* For HZ */

/* 2**32 / 1000000 (rounded up) */
#define __usecs_to_xloops_mult	0x10C7UL

/* 2**32 / 1000000000 (rounded up) */
#define __nsecs_to_xloops_mult	0x5UL

extern unsigned long loops_per_jiffy;
static inline unsigned long xloops_to_cycles(unsigned long xloops)
{
	return (xloops * loops_per_jiffy * HZ) >> 32;
}

#define USECS_TO_CYCLES(time_usecs) \
	xloops_to_cycles((time_usecs) * __usecs_to_xloops_mult)

#define NSECS_TO_CYCLES(time_nsecs) \
	xloops_to_cycles((time_nsecs) * __nsecs_to_xloops_mult)

u64 notrace __delay_cycles(void);

#endif	/* _ASM_DELAY_CONST_H */
