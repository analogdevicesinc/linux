/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_LINKAGE_H
#define __ASM_LINKAGE_H

#define cond_syscall(x)  asm(".weak\t" #x "\n" #x " = sys_ni_syscall")
#define SYSCALL_ALIAS(alias, name)					\
	asm ( #alias " = " #name "\n\t.globl " #alias)

#ifdef __ASSEMBLY__
/* Alpha uses .set for assembler options, not symbol assignments. */
#define SYM_END(name, sym_type)				\
	.type name sym_type ASM_NL			\
	.L__sym_size_##name = . - name ASM_NL		\
	.size name, .L__sym_size_##name
#endif

#endif
