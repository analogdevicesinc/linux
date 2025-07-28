// SPDX-License-Identifier: GPL-2.0

#include <uapi/asm-generic/errno.h>

typeof(__cvdso_getrandom) __kernel_getrandom;

ssize_t __kernel_getrandom(void *buffer, size_t len, unsigned int flags, void *opaque_state, size_t opaque_len)
{
	if (alternative_has_cap_likely(ARM64_HAS_FPSIMD))
		return __cvdso_getrandom(buffer, len, flags, opaque_state, opaque_len);

	if (unlikely(opaque_len == ~0UL && !buffer && !len && !flags))
		return -ENOSYS;
	return getrandom_syscall(buffer, len, flags);
}
