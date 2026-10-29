// SPDX-License-Identifier: GPL-2.0
/* The entries in /proc that can't be mountpoints. */

#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sched.h>
#include <stdio.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <unistd.h>

#include "kselftest_harness.h"

#ifndef OPEN_TREE_CLONE
#define OPEN_TREE_CLONE 1
#endif

#ifndef MOVE_MOUNT_F_EMPTY_PATH
#define MOVE_MOUNT_F_EMPTY_PATH 0x00000004
#endif

#ifndef MOVE_MOUNT_T_EMPTY_PATH
#define MOVE_MOUNT_T_EMPTY_PATH 0x00000040
#endif

#ifndef __NR_open_tree
#define __NR_open_tree 428
#endif

#ifndef __NR_move_mount
#define __NR_move_mount 429
#endif

static int sys_open_tree(int dfd, const char *path, unsigned int flags)
{
	return syscall(__NR_open_tree, dfd, path, flags);
}

static int sys_move_mount(int from_dfd, const char *from_path, int to_dfd,
			  const char *to_path, unsigned int flags)
{
	return syscall(__NR_move_mount, from_dfd, from_path, to_dfd, to_path,
		       flags);
}

/* Mount a bind of /dev/null right on the object @fd refers to. */
static int move_mount_null(int fd)
{
	int fd_tree, ret;

	fd_tree = sys_open_tree(AT_FDCWD, "/dev/null", OPEN_TREE_CLONE);
	if (fd_tree < 0)
		return -1;
	ret = sys_move_mount(fd_tree, "", fd, "",
			     MOVE_MOUNT_F_EMPTY_PATH | MOVE_MOUNT_T_EMPTY_PATH);
	close(fd_tree);
	return ret;
}

FIXTURE(unmountable) {
	char path[PATH_MAX];
};

FIXTURE_SETUP(unmountable)
{
	if (geteuid())
		SKIP(return, "mounting needs root");

	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(mount(NULL, "/", NULL, MS_PRIVATE | MS_REC, NULL), 0);
}

FIXTURE_TEARDOWN(unmountable)
{
}

/* Nothing may be mounted on a link into /proc/self itself. */
TEST_F(unmountable, self_links)
{
	static const char *const links[] = {
		"/proc/self", "/proc/thread-self", "/proc/mounts", "/proc/net",
	};

	for (size_t i = 0; i < ARRAY_SIZE(links); i++) {
		int fd;

		fd = open(links[i], O_PATH | O_NOFOLLOW | O_CLOEXEC);
		/* /proc/net needs CONFIG_NET */
		if (fd < 0 && errno == ENOENT)
			continue;
		ASSERT_GE(fd, 0);
		EXPECT_EQ(move_mount_null(fd), -1) {
			TH_LOG("mounted on %s", links[i]);
		}
		EXPECT_EQ(errno, ENOENT);
		close(fd);
	}
}

/* Mounting through a link into /proc/self lands on the entry of the caller. */
TEST_F(unmountable, through_links)
{
	struct stat st;

	ASSERT_EQ(mount("/dev/null", "/proc/mounts", NULL, MS_BIND, NULL), 0);
	/* the caller's /proc/<pid>/mounts is covered */
	ASSERT_EQ(stat("/proc/self/mounts", &st), 0);
	EXPECT_TRUE(S_ISCHR(st.st_mode));
	/* the link itself is not */
	ASSERT_EQ(lstat("/proc/mounts", &st), 0);
	EXPECT_TRUE(S_ISLNK(st.st_mode));
}

/* The entries below /proc/<pid>/net take no mount. */
TEST_F(unmountable, net_entries)
{
	if (access("/proc/self/net/dev", F_OK))
		SKIP(return, "needs CONFIG_NET");

	ASSERT_EQ(mount("/dev/null", "/proc/self/net/dev", NULL, MS_BIND, NULL), -1);
	ASSERT_EQ(errno, ENOENT);
}

/* Neither do the entries below /proc/<pid>/fd and /proc/<pid>/fdinfo. */
TEST_F(unmountable, fd_entries)
{
	int fd, fd_link;

	fd = open("/dev/null", O_RDONLY | O_CLOEXEC);
	ASSERT_GE(fd, 0);

	snprintf(self->path, sizeof(self->path), "/proc/self/fd/%d", fd);
	fd_link = open(self->path, O_PATH | O_NOFOLLOW | O_CLOEXEC);
	ASSERT_GE(fd_link, 0);
	EXPECT_EQ(move_mount_null(fd_link), -1);
	EXPECT_EQ(errno, ENOENT);
	close(fd_link);

	snprintf(self->path, sizeof(self->path), "/proc/self/fdinfo/%d", fd);
	EXPECT_EQ(mount("/dev/null", self->path, NULL, MS_BIND, NULL), -1);
	EXPECT_EQ(errno, ENOENT);
	close(fd);
}

TEST_HARNESS_MAIN
