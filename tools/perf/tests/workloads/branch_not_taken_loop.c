// SPDX-License-Identifier: GPL-2.0
#include <linux/compiler.h>
#include "../tests.h"

#ifdef __aarch64__
static void branch_not_taken(void)
{
	/* Keep x1 > x2 so B.LS is never taken. */
	asm volatile(
		"mov	x1, #2\n"
		"mov	x2, #1\n"
		"movz	x3, #0xffff\n"
		"movk	x3, #0x0010, lsl #16\n"
		"1: cmp	x1, x2\n"
		".Ltest_branch:\n"
		"b.ls	2f\n"
		".Lfallthrough:\n"
		"subs	x3, x3, #1\n"
		"b.ne	1b\n"
		"2:\n"
		: : : "x1", "x2", "x3", "cc");
}
#else
static void branch_not_taken(void) { }
#endif

static int branch_not_taken_loop(int argc __maybe_unused, const char **argv __maybe_unused)
{
	branch_not_taken();
	return 0;
}

DEFINE_WORKLOAD(branch_not_taken_loop);
