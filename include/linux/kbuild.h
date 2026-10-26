/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_KBUILD_H
#define __LINUX_KBUILD_H

#define DEFINE(sym, val) \
	asm volatile("\n.ascii \"->" #sym " %0 " #val "\"" : : "i" (val))

#define BLANK() do { asm volatile ("\n.ascii \"->BLANK()\"" : : ); } while (0)

#define OFFSET(sym, str, mem) \
	DEFINE(sym, offsetof(struct str, mem))

#define COMMENT(x) \
	asm volatile("\n.ascii \"->#" x "\"")

#endif
