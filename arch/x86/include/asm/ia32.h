/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_IA32_H
#define _ASM_X86_IA32_H

#ifdef CONFIG_IA32_EMULATION

#include <linux/compat.h>

/*
 * 32 bit structures for IA32 support.
 */

#include <uapi/asm/sigcontext.h>

/* signal.h */

struct ucontext_ia32 {
	unsigned int	  uc_flags;
	unsigned int 	  uc_link;
	compat_stack_t	  uc_stack;
	struct sigcontext_32 uc_mcontext;
	compat_sigset_t	  uc_sigmask;	/* mask last for extensibility */
};

/* This matches struct stat64 in glibc2.2, hence the absolutely
 * insane amounts of padding around dev_t's.
 */
struct stat64 {
	unsigned long long	st_dev;
	unsigned char		__pad0[4];

#define STAT64_HAS_BROKEN_ST_INO	1
	unsigned int		__st_ino;

	unsigned int		st_mode;
	unsigned int		st_nlink;

	unsigned int		st_uid;
	unsigned int		st_gid;

	unsigned long long	st_rdev;
	unsigned char		__pad3[4];

	long long		st_size;
	unsigned int		st_blksize;

	long long		st_blocks;/* Number 512-byte blocks allocated */

	unsigned 		st_atime;
	unsigned 		st_atime_nsec;
	unsigned 		st_mtime;
	unsigned 		st_mtime_nsec;
	unsigned 		st_ctime;
	unsigned 		st_ctime_nsec;

	unsigned long long	st_ino;
} __attribute__((packed));

extern bool __ia32_enabled;

static __always_inline bool ia32_enabled(void)
{
	return __ia32_enabled;
}

static inline void ia32_disable(void)
{
	__ia32_enabled = false;
}

#else /* !CONFIG_IA32_EMULATION */

static __always_inline bool ia32_enabled(void)
{
	return IS_ENABLED(CONFIG_X86_32);
}

static inline void ia32_disable(void) {}

#endif

static inline bool ia32_enabled_verbose(void)
{
	bool enabled = ia32_enabled();

	if (IS_ENABLED(CONFIG_IA32_EMULATION) && !enabled)
		pr_notice_once("32-bit emulation disabled. You can reenable with ia32_emulation=on\n");

	return enabled;
}

#endif /* _ASM_X86_IA32_H */
