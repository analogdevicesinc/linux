/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __TOOLS_LINUX_KBUILD_H
#define __TOOLS_LINUX_KBUILD_H

#define DEFINE(sym, val) \
	asm volatile("\n.ascii \"->" #sym " %0 " #val "\"" : : "i" (val))

#define OFFSET(sym, str, mem) \
	DEFINE(sym, __builtin_offsetof(struct str, mem))

#endif /* __TOOLS_LINUX_KBUILD_H */
