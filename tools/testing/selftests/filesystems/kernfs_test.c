// SPDX-License-Identifier: GPL-2.0
#define _GNU_SOURCE
#define __SANE_USERSPACE_TYPES__

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/xattr.h>

#include "kselftest_harness.h"
#include "wrappers.h"

TEST(kernfs_listxattr)
{
	ssize_t len;
	int fd;

	/* Read-only file that can never have any extended attributes set.
	 * However, on systems with SELinux enabled, security.selinux xattr
	 * may be present. Skip the content check if any xattrs are found.
	 */
	fd = open("/sys/kernel/warn_count", O_RDONLY | O_CLOEXEC);
	ASSERT_GE(fd, 0);

	len = flistxattr(fd, NULL, 0);
	ASSERT_GE(len, 0);

	if (len > 0) {
		close(fd);
		SKIP(return, "xattrs present on /sys/kernel/warn_count, skipping xattr content check");
	}

	EXPECT_EQ(close(fd), 0);
}

TEST(kernfs_getxattr)
{
	int fd;
	char buf[1];

	/* Read-only file that can never have any extended attributes set. */
	fd = open("/sys/kernel/warn_count", O_RDONLY | O_CLOEXEC);
	ASSERT_GE(fd, 0);
	ASSERT_LT(fgetxattr(fd, "user.foo", buf, sizeof(buf)), 0);
	ASSERT_EQ(errno, ENODATA);
	EXPECT_EQ(close(fd), 0);
}

TEST_HARNESS_MAIN

