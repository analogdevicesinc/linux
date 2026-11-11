// SPDX-License-Identifier: GPL-2.0
/*
 * quotactl(2) on a frozen superblock.
 *
 * Commands that modify quota wait in quotactl_block() for a frozen
 * superblock to be thawed. A read-only superblock is frozen without
 * taking s_writers so that wait can't be built on sb_start_write(). Check
 * that a caller waiting on a frozen read-only or read-write superblock can
 * be killed and that it is woken up when the superblock is thawed.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/fs.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/quota.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include "../../kselftest_harness.h"
#include "loop_util.h"

#define IMG_SIZE (64 << 20)
#define POLL_MS 10
/* How long the child gets to block in quotactl() or to go away. */
#define WAIT_MS 3000

FIXTURE(frozen) {
	char dir[64];
	char img[96];
	char mnt[96];
	char dev[32];
	int loop;
	int mntfd;
	pid_t pid;
	bool frozen;
};

FIXTURE_VARIANT(frozen) {
	unsigned long flags;
};

FIXTURE_VARIANT_ADD(frozen, ro) {
	.flags = MS_RDONLY,
};

FIXTURE_VARIANT_ADD(frozen, rw) {
	.flags = 0,
};

/* Fork a child that calls quotactl(Q_SETQUOTA) on @dev and exits. */
static pid_t quotactl_child(const char *dev)
{
	pid_t pid;

	pid = fork();
	if (pid == 0) {
		struct if_dqblk dq = {};

		quotactl(QCMD(Q_SETQUOTA, USRQUOTA), dev, 0, (void *)&dq);
		_exit(0);
	}
	return pid;
}

/* Reap @pid within @ms milliseconds; true if it went away. */
static bool reap(pid_t pid, int *status, int ms)
{
	struct timespec ts = { .tv_nsec = POLL_MS * 1000 * 1000 };

	for (int i = 0; i < ms / POLL_MS; i++) {
		pid_t ret = waitpid(pid, status, WNOHANG);

		if (ret == pid)
			return true;
		if (ret < 0)
			return false;
		nanosleep(&ts, NULL);
	}
	return false;
}

/* Wait up to @ms milliseconds for @pid to sleep in the kernel. */
static bool blocked(pid_t pid, int ms)
{
	struct timespec ts = { .tv_nsec = POLL_MS * 1000 * 1000 };
	char path[64], buf[512], *p, state;

	snprintf(path, sizeof(path), "/proc/%d/stat", pid);
	for (int i = 0; i < ms / POLL_MS; i++) {
		ssize_t n;
		int fd;

		fd = open(path, O_RDONLY | O_CLOEXEC);
		if (fd < 0)
			return false;
		n = read(fd, buf, sizeof(buf) - 1);
		close(fd);
		if (n <= 0)
			return false;
		buf[n] = '\0';
		/* "pid (comm) state ..." */
		p = strrchr(buf, ')');
		if (p && sscanf(p, ") %c", &state) == 1 &&
		    (state == 'D' || state == 'S'))
			return true;
		nanosleep(&ts, NULL);
	}
	return false;
}

static void frozen_cleanup(FIXTURE_DATA(frozen) *self)
{
	int status;

	if (self->pid > 0)
		kill(self->pid, SIGKILL);
	/* The thaw also lets a waiter go that ignored SIGKILL. */
	if (self->frozen)
		ioctl(self->mntfd, FITHAW, 0);
	if (self->pid > 0)
		reap(self->pid, &status, WAIT_MS);
	if (self->mntfd >= 0) {
		close(self->mntfd);
		umount2(self->mnt, MNT_DETACH);
	}
	if (self->loop >= 0)
		loop_clear(self->loop);
	unlink(self->img);
	rmdir(self->mnt);
	rmdir(self->dir);
}

FIXTURE_SETUP(frozen)
{
	char cmd[160];
	int fd, ret;

	self->loop = -1;
	self->mntfd = -1;
	self->pid = -1;
	self->frozen = false;

	if (geteuid() != 0)
		SKIP(return, "test requires CAP_SYS_ADMIN");
	if (access("/dev/loop-control", R_OK | W_OK))
		SKIP(return, "test requires loop devices");
	/* Without a device this fails with ENODEV, or ENOSYS without quotactl(). */
	if (quotactl(QCMD(Q_GETINFO, USRQUOTA), NULL, 0, NULL) && errno == ENOSYS)
		SKIP(return, "kernel lacks quotactl()");

	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(mount(NULL, "/", NULL, MS_REC | MS_PRIVATE, NULL), 0);

	snprintf(self->dir, sizeof(self->dir), "/tmp/freeze_quotactl.XXXXXX");
	ASSERT_NE(mkdtemp(self->dir), NULL);
	snprintf(self->img, sizeof(self->img), "%s/img", self->dir);
	snprintf(self->mnt, sizeof(self->mnt), "%s/mnt", self->dir);
	/* The harness doesn't run the teardown when the setup fails. */
	ASSERT_EQ(mkdir(self->mnt, 0700), 0)
		frozen_cleanup(self);

	fd = open(self->img, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
	ASSERT_GE(fd, 0)
		frozen_cleanup(self);
	ASSERT_EQ(ftruncate(fd, IMG_SIZE), 0)
		frozen_cleanup(self);

	snprintf(cmd, sizeof(cmd), "mkfs.ext4 -q -F %s >/dev/null 2>&1", self->img);
	ret = system(cmd);
	if (ret) {
		close(fd);
		frozen_cleanup(self);
		SKIP(return, "mkfs.ext4 failed or is not available");
	}

	self->loop = loop_bind(fd);
	close(fd);
	if (loop_missing(self->loop)) {
		frozen_cleanup(self);
		SKIP(return, "test requires loop devices");
	}
	ASSERT_GE(self->loop, 0)
		frozen_cleanup(self);
	snprintf(self->dev, sizeof(self->dev), "/dev/loop%d", self->loop);

	if (mount(self->dev, self->mnt, "ext4", variant->flags, NULL)) {
		if (errno == ENODEV) {
			frozen_cleanup(self);
			SKIP(return, "kernel lacks ext4");
		}
		ASSERT_EQ(errno, 0)
			frozen_cleanup(self);
	}
	self->mntfd = open(self->mnt, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(self->mntfd, 0)
		frozen_cleanup(self);
}

FIXTURE_TEARDOWN(frozen)
{
	frozen_cleanup(self);
}

TEST_F(frozen, waiter_is_killable)
{
	int status;

	ASSERT_EQ(ioctl(self->mntfd, FIFREEZE, 0), 0);
	self->frozen = true;

	self->pid = quotactl_child(self->dev);
	ASSERT_GT(self->pid, 0);
	ASSERT_TRUE(blocked(self->pid, WAIT_MS))
		TH_LOG("quotactl() doesn't wait on a frozen superblock");

	ASSERT_EQ(kill(self->pid, SIGKILL), 0);
	ASSERT_TRUE(reap(self->pid, &status, WAIT_MS))
		TH_LOG("quotactl() waiter survived SIGKILL on a frozen superblock");
	self->pid = -1;
	ASSERT_TRUE(WIFSIGNALED(status));
	ASSERT_EQ(WTERMSIG(status), SIGKILL);
}

TEST_F(frozen, waiter_wakes_on_thaw)
{
	int status;

	ASSERT_EQ(ioctl(self->mntfd, FIFREEZE, 0), 0);
	self->frozen = true;

	self->pid = quotactl_child(self->dev);
	ASSERT_GT(self->pid, 0);
	ASSERT_TRUE(blocked(self->pid, WAIT_MS))
		TH_LOG("quotactl() doesn't wait on a frozen superblock");

	ASSERT_EQ(ioctl(self->mntfd, FITHAW, 0), 0);
	self->frozen = false;

	ASSERT_TRUE(reap(self->pid, &status, WAIT_MS))
		TH_LOG("quotactl() waiter wasn't woken by the thaw");
	self->pid = -1;
	ASSERT_TRUE(WIFEXITED(status));
	ASSERT_EQ(WEXITSTATUS(status), 0);
}

TEST_HARNESS_MAIN
