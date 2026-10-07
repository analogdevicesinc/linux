/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Prevent the compiler from merging or refetching reads or writes. The
 * compiler is also forbidden from reordering successive instances of
 * READ_ONCE and WRITE_ONCE, but only when the compiler is aware of some
 * particular ordering. One way to make the compiler aware of ordering is to
 * put the two invocations of READ_ONCE or WRITE_ONCE in different C
 * statements.
 *
 * These two macros will also work on aggregate data types like structs or
 * unions.
 *
 * Their two major use cases are: (1) Mediating communication between
 * process-level code and irq/NMI handlers, all running on the same CPU,
 * and (2) Ensuring that the compiler does not fold, spindle, or otherwise
 * mutilate accesses that either do not require ordering or that interact
 * with an explicit memory barrier or atomic instruction that provides the
 * required ordering.
 */
#ifndef __ASM_GENERIC_RWONCE_H
#define __ASM_GENERIC_RWONCE_H

#ifndef __ASSEMBLY__

#include <linux/compiler_types.h>
#include <linux/kasan-checks.h>
#include <linux/kcsan-checks.h>

/*
 * Yes, this permits 64-bit accesses on 32-bit architectures. These will
 * actually be atomic in some cases (namely Armv7 + LPAE), but for others we
 * rely on the access being split into 2x32-bit accesses for a 32-bit quantity
 * (e.g. a virtual address) and a strong prevailing wind.
 */
#define compiletime_assert_rwonce_type(t)					\
	compiletime_assert(__native_word(t) || sizeof(t) == sizeof(long long),	\
		"Unsupported access size for {READ,WRITE}_ONCE().")

/*
 * Use __READ_ONCE() instead of READ_ONCE() if you do not require any
 * atomicity. Note that this may result in tears!
 */
#ifndef __READ_ONCE
#define __READ_ONCE(x)	(*(const volatile __unqual_scalar_typeof(x) *)&(x))
#endif

#define READ_ONCE(x)							\
({									\
	compiletime_assert_rwonce_type(x);				\
	__READ_ONCE(x);							\
})

#define __WRITE_ONCE(x, val)						\
do {									\
	*(volatile typeof(x) *)&(x) = (val);				\
} while (0)

#define WRITE_ONCE(x, val)						\
do {									\
	compiletime_assert_rwonce_type(x);				\
	__WRITE_ONCE(x, val);						\
} while (0)

/*
 * {READ,WRITE}_ONCE_THIS_CPU() identify accesses to a per-CPU variable
 * which are intended to race only with accesses from other execution
 * contexts on the same CPU, e.g. task and interrupt context.
 *
 * For now, these are simple wrappers around READ_ONCE() and
 * WRITE_ONCE().
 */
#define READ_ONCE_THIS_CPU(x)		READ_ONCE(x)
#define WRITE_ONCE_THIS_CPU(x, val)	WRITE_ONCE(x, val)

static __no_sanitize_or_inline
unsigned long __read_once_word_nocheck(const void *addr)
{
	return __READ_ONCE(*(unsigned long *)addr);
}

/*
 * Use READ_ONCE_NOCHECK() instead of READ_ONCE() if you need to load a
 * word from memory atomically but without telling KASAN/KCSAN. This is
 * usually used by unwinding code when walking the stack of a running process.
 */
#define READ_ONCE_NOCHECK(x)						\
({									\
	compiletime_assert(sizeof(x) == sizeof(unsigned long),		\
		"Unsupported access size for READ_ONCE_NOCHECK().");	\
	(typeof(x))__read_once_word_nocheck(&(x));			\
})

static __no_sanitize_or_inline
unsigned long read_word_at_a_time(const void *addr)
{
	/* open-coded instrument_read(addr, 1) */
	kasan_check_read(addr, 1);
	kcsan_check_read(addr, 1);

	/*
	 * This load can race with concurrent stores to out-of-bounds memory,
	 * but READ_ONCE() can't be used because it requires higher alignment
	 * than plain loads in arm64 builds with LTO.
	 */
	return *(unsigned long *)addr;
}

#endif /* __ASSEMBLY__ */
#endif	/* __ASM_GENERIC_RWONCE_H */
