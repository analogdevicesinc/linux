// SPDX-License-Identifier: GPL-2.0
/*
 * Reconfiguring a read-only superblock through fspick().
 *
 * fspick() creates a reconfiguration context whose flag mask is empty until
 * "ro" or "rw" is set. Filesystems decide whether to go read-write from
 * fc->sb_flags, so a reconfiguration without either must leave a read-only
 * superblock read-only. Check that writes through a read-write bind mount of
 * a read-only ext4 superblock are still refused after such a reconfiguration
 * and that an explicit "rw" still makes the superblock read-write.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#include "wrappers.h"
#include "freeze/loop_util.h"
#include "../kselftest_harness.h"

#define IMG_SIZE (64 << 20)

#ifndef MNT_DETACH
#define MNT_DETACH 2
#endif

static inline int sys_fspick(int dfd, const char *path, unsigned int flags)
{
	return syscall(__NR_fspick, dfd, path, flags);
}

static inline int sys_umount2(const char *path, int flags)
{
	return syscall(__NR_umount2, path, flags);
}

FIXTURE(fspick_ro) {
	char dir[64];
	char img[96];
	char mnt[96];
	char bind[96];
	int loop;
	bool mounted;
	bool bound;
};

static void fspick_ro_cleanup(FIXTURE_DATA(fspick_ro) *self)
{
	if (self->bound)
		sys_umount2(self->bind, MNT_DETACH);
	if (self->mounted)
		sys_umount2(self->mnt, MNT_DETACH);
	if (self->loop >= 0)
		loop_clear(self->loop);
	unlink(self->img);
	rmdir(self->bind);
	rmdir(self->mnt);
	rmdir(self->dir);
}

/* Mount @dev read-only and add a read-write bind mount of it. */
static int fspick_ro_mount(FIXTURE_DATA(fspick_ro) *self, const char *dev)
{
	if (sys_mount(dev, self->mnt, "ext4", MS_RDONLY, NULL))
		return -errno;
	self->mounted = true;

	/* A read-write bind mount only refuses writes if the superblock does. */
	if (sys_mount(self->mnt, self->bind, NULL, MS_BIND, NULL))
		return -errno;
	self->bound = true;
	if (sys_mount(NULL, self->bind, NULL, MS_REMOUNT | MS_BIND, NULL))
		return -errno;
	return 0;
}

FIXTURE_SETUP(fspick_ro)
{
	char cmd[160], dev[32];
	int fd, ret;

	self->loop = -1;
	self->mounted = false;
	self->bound = false;

	if (geteuid() != 0)
		SKIP(return, "test requires root");
	if (access("/dev/loop-control", R_OK | W_OK))
		SKIP(return, "test requires loop devices");

	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(sys_mount(NULL, "/", NULL, MS_REC | MS_PRIVATE, NULL), 0);

	snprintf(self->dir, sizeof(self->dir), "/tmp/fspick_ro.XXXXXX");
	ASSERT_NE(mkdtemp(self->dir), NULL);
	snprintf(self->img, sizeof(self->img), "%s/img", self->dir);
	snprintf(self->mnt, sizeof(self->mnt), "%s/mnt", self->dir);
	snprintf(self->bind, sizeof(self->bind), "%s/bind", self->dir);
	ASSERT_EQ(mkdir(self->mnt, 0700), 0);
	ASSERT_EQ(mkdir(self->bind, 0700), 0);

	fd = open(self->img, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
	ASSERT_GE(fd, 0);
	ASSERT_EQ(ftruncate(fd, IMG_SIZE), 0);

	snprintf(cmd, sizeof(cmd), "mkfs.ext4 -q -F %s >/dev/null 2>&1", self->img);
	ret = system(cmd);
	if (ret) {
		close(fd);
		fspick_ro_cleanup(self);
		SKIP(return, "mkfs.ext4 failed or is not available");
	}

	/* The harness skips the teardown if the setup fails, so clean up. */
	self->loop = loop_bind(fd);
	close(fd);
	if (self->loop < 0) {
		fspick_ro_cleanup(self);
		ASSERT_GE(self->loop, 0);
	}
	snprintf(dev, sizeof(dev), "/dev/loop%d", self->loop);

	ret = fspick_ro_mount(self, dev);
	if (ret) {
		fspick_ro_cleanup(self);
		if (ret == -ENODEV)
			SKIP(return, "kernel lacks ext4");
		ASSERT_EQ(ret, 0);
	}
}

FIXTURE_TEARDOWN(fspick_ro)
{
	fspick_ro_cleanup(self);
}

static int reconfigure(const char *path, const char *flag)
{
	int fd, ret = 0;

	fd = sys_fspick(AT_FDCWD, path, FSPICK_CLOEXEC);
	if (fd < 0)
		return -errno;
	if (flag && sys_fsconfig(fd, FSCONFIG_SET_FLAG, flag, NULL, 0))
		ret = -errno;
	if (!ret && sys_fsconfig(fd, FSCONFIG_CMD_RECONFIGURE, NULL, NULL, 0))
		ret = -errno;
	close(fd);
	return ret;
}

static int create_file(const char *dir)
{
	char path[128];
	int fd;

	snprintf(path, sizeof(path), "%s/file", dir);
	fd = open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
	if (fd < 0)
		return -errno;
	close(fd);
	return 0;
}

TEST_F(fspick_ro, reconfigure_keeps_ro)
{
	ASSERT_EQ(create_file(self->bind), -EROFS);

	ASSERT_EQ(reconfigure(self->mnt, NULL), 0);
	ASSERT_EQ(create_file(self->bind), -EROFS)
		TH_LOG("reconfiguration without \"rw\" made the superblock read-write");
}

TEST_F(fspick_ro, reconfigure_rw)
{
	ASSERT_EQ(reconfigure(self->mnt, "rw"), 0);
	ASSERT_EQ(create_file(self->bind), 0);
}

TEST_HARNESS_MAIN
