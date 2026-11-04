/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026 Sasha Levin <sashal@kernel.org>
 *
 * Compatibility helpers for KAPI selftests.
 *
 * __NR_open does not exist on architectures using the generic syscall table
 * (e.g. arm64, riscv); only __NR_openat does. There the wrapper falls back to
 * openat(AT_FDCWD, ...), which does not go through sys_open, so the sys_open
 * KAPI checks are not exercised.
 */
#ifndef KAPI_TEST_UTIL_H
#define KAPI_TEST_UTIL_H

#include <fcntl.h>
#include <unistd.h>
#include <sys/syscall.h>

#ifndef __NR_open
static inline long kapi_sys_open(const char *pathname, int flags, int mode)
{
	return syscall(__NR_openat, AT_FDCWD, pathname, flags, mode);
}
#else
static inline long kapi_sys_open(const char *pathname, int flags, int mode)
{
	return syscall(__NR_open, pathname, flags, mode);
}
#endif

#endif /* KAPI_TEST_UTIL_H */
