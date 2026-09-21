/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
#ifndef _UAPI_LINUX_CLOSE_RANGE_H
#define _UAPI_LINUX_CLOSE_RANGE_H

/*
 * A macro of one of these names defined before this header is parsed, by
 * a libc or by a program's own fallback, would replace the enumerator.
 */
#undef CLOSE_RANGE_UNSHARE
#undef CLOSE_RANGE_CLOEXEC

enum close_range_flags {
	/* Unshare the file descriptor table before closing file descriptors. */
	CLOSE_RANGE_UNSHARE	= (1U << 1),

	/* Set the FD_CLOEXEC bit instead of closing the file descriptor. */
	CLOSE_RANGE_CLOEXEC	= (1U << 2),
};

/* Keep #ifdef working and let glibc skip its own definitions. */
#define CLOSE_RANGE_UNSHARE		CLOSE_RANGE_UNSHARE
#define CLOSE_RANGE_CLOEXEC		CLOSE_RANGE_CLOEXEC

#endif /* _UAPI_LINUX_CLOSE_RANGE_H */

