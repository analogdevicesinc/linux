/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_SECCOMP_H
#define _ASM_SECCOMP_H

#include <linux/unistd.h>

#define __NR_seccomp_sigreturn_32 __NR_sigreturn

#ifdef CONFIG_SPARC64
# define SECCOMP_ARCH_NATIVE		AUDIT_ARCH_SPARC64
# define SECCOMP_ARCH_NATIVE_NR		NR_syscalls
# define SECCOMP_ARCH_NATIVE_NAME	"sparc64"
# ifdef CONFIG_COMPAT
#  define SECCOMP_ARCH_COMPAT		AUDIT_ARCH_SPARC
#  define SECCOMP_ARCH_COMPAT_NR	NR_syscalls
#  define SECCOMP_ARCH_COMPAT_NAME	"sparc"
# endif
#else
# define SECCOMP_ARCH_NATIVE		AUDIT_ARCH_SPARC
# define SECCOMP_ARCH_NATIVE_NR		NR_syscalls
# define SECCOMP_ARCH_NATIVE_NAME	"sparc"
#endif

#include <asm-generic/seccomp.h>

#endif /* _ASM_SECCOMP_H */
