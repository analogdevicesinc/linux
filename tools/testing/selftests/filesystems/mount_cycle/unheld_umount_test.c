// SPDX-License-Identifier: GPL-2.0
/*
 * A mount that nothing else references is torn down before umount(2)
 * returns, lazy or not, before the last close of the file that kept it
 * alive returns, and before the task that took a mount namespace down
 * with it can be reaped. A loop device shows when that has happened: the
 * filesystem claims it, and an exclusive open of the device succeeds
 * only once the filesystem has let go.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <linux/loop.h>

#include "../../kselftest_harness.h"

#define IMAGE_SIZE	(1440 * 1024)
#define SECTOR		512

/* A blank FAT12 floppy image: boot sector, two FATs, an empty root directory. */
static int write_fat12(int fd)
{
	unsigned char sector[SECTOR] = {
		0xeb, 0x3c, 0x90, 'M', 'S', 'W', 'I', 'N', '4', '.', '1',
		[11] = 0x00, 0x02,	/* bytes per sector: 512 */
		[13] = 1,		/* sectors per cluster */
		[14] = 1, 0,		/* reserved sectors */
		[16] = 2,		/* FATs */
		[17] = 0xe0, 0x00,	/* root directory entries: 224 */
		[19] = 0x40, 0x0b,	/* total sectors: 2880 */
		[21] = 0xf0,		/* media descriptor */
		[22] = 9, 0,		/* sectors per FAT */
		[24] = 18, 0,		/* sectors per track */
		[26] = 2, 0,		/* heads */
		[38] = 0x29,		/* extended boot signature */
		[39] = 0x12, 0x34, 0x56, 0x78,
		[43] = 'N', 'O', ' ', 'N', 'A', 'M', 'E', ' ', ' ', ' ', ' ',
		[54] = 'F', 'A', 'T', '1', '2', ' ', ' ', ' ',
		[510] = 0x55, 0xaa,
	};
	unsigned char fat[SECTOR] = { 0xf0, 0xff, 0xff };

	if (pwrite(fd, sector, SECTOR, 0) != SECTOR)
		return -1;
	/* the first FAT and the second one, one sector each is enough */
	if (pwrite(fd, fat, SECTOR, 1 * SECTOR) != SECTOR ||
	    pwrite(fd, fat, SECTOR, 10 * SECTOR) != SECTOR)
		return -1;
	return ftruncate(fd, IMAGE_SIZE);
}

/* Bind a free loop device to the open image @ifd; the device number. */
static int loop_bind(int ifd)
{
	int cfd, lfd, n;
	char dev[32];

	cfd = open("/dev/loop-control", O_RDWR);
	if (cfd < 0)
		return -1;
	n = ioctl(cfd, LOOP_CTL_GET_FREE);
	close(cfd);
	if (n < 0)
		return -1;
	snprintf(dev, sizeof(dev), "/dev/loop%d", n);
	lfd = open(dev, O_RDWR);
	if (lfd < 0)
		return -1;
	if (ioctl(lfd, LOOP_SET_FD, ifd))
		n = -1;
	close(lfd);
	return n;
}

/* Mount the loop device at @mp: vfat, or msdos. */
static int loop_mount(const char *dev, const char *mp)
{
	if (!mount(dev, mp, "vfat", 0, NULL))
		return 0;
	return mount(dev, mp, "msdos", 0, NULL);
}

/*
 * An exclusive open of the device fails with EBUSY while a filesystem
 * holds it. No retries: the point is that it has been let go by now.
 */
static bool loop_free(const char *dev)
{
	int lfd = open(dev, O_RDONLY | O_EXCL);

	if (lfd < 0)
		return false;
	close(lfd);
	return true;
}

/* Tell the loop device to let go of its image. */
static void loop_clear(const char *dev)
{
	int lfd = open(dev, O_RDWR);

	if (lfd >= 0) {
		ioctl(lfd, LOOP_CLR_FD);
		close(lfd);
	}
}

FIXTURE(unheld_umount) {
	char dir[64];
	char mp[80];
	char dev[32];
};

FIXTURE_SETUP(unheld_umount)
{
	char img[80];
	int ifd, n;

	if (geteuid() != 0)
		SKIP(return, "test requires CAP_SYS_ADMIN");
	if (access("/dev/loop-control", R_OK | W_OK))
		SKIP(return, "test requires loop devices");

	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL), 0);

	snprintf(self->dir, sizeof(self->dir), "/tmp/unheld_umount.XXXXXX");
	ASSERT_NE(mkdtemp(self->dir), NULL);
	ASSERT_EQ(mount("tmpfs", self->dir, "tmpfs", 0, NULL), 0);

	snprintf(img, sizeof(img), "%s/img", self->dir);
	ifd = open(img, O_RDWR | O_CREAT | O_EXCL, 0600);
	ASSERT_GE(ifd, 0);
	ASSERT_EQ(write_fat12(ifd), 0);
	n = loop_bind(ifd);
	close(ifd);	/* the loop device holds the file from now on */
	ASSERT_GE(n, 0);
	snprintf(self->dev, sizeof(self->dev), "/dev/loop%d", n);

	snprintf(self->mp, sizeof(self->mp), "%s/mnt", self->dir);
	ASSERT_EQ(mkdir(self->mp, 0755), 0);
	/* the harness skips the teardown after a SKIP() here */
	if (loop_mount(self->dev, self->mp)) {
		loop_clear(self->dev);
		SKIP(return, "test requires a FAT filesystem");
	}
	umount2(self->mp, 0);
}

FIXTURE_TEARDOWN(unheld_umount)
{
	umount2(self->mp, MNT_DETACH);
	loop_clear(self->dev);
	umount2(self->dir, MNT_DETACH);
	rmdir(self->dir);
}

/* A synchronous umount(2) returns with the filesystem shut down. */
TEST_F(unheld_umount, sync_umount_releases_before_return)
{
	ASSERT_EQ(loop_mount(self->dev, self->mp), 0);
	ASSERT_FALSE(loop_free(self->dev));
	ASSERT_EQ(umount2(self->mp, 0), 0);
	EXPECT_TRUE(loop_free(self->dev))
		TH_LOG("%s is still claimed after umount(2) returned", self->dev);
}

/* So does a lazy one when nothing else references the mount. */
TEST_F(unheld_umount, lazy_umount_idle_releases_before_return)
{
	ASSERT_EQ(loop_mount(self->dev, self->mp), 0);
	ASSERT_FALSE(loop_free(self->dev));
	ASSERT_EQ(umount2(self->mp, MNT_DETACH), 0);
	EXPECT_TRUE(loop_free(self->dev))
		TH_LOG("%s is still claimed after umount2(MNT_DETACH) returned", self->dev);
}

/*
 * A lazy umount of a mount that a file descriptor holds leaves the
 * filesystem alive; the close that drops the last reference shuts it
 * down before it returns.
 */
TEST_F(unheld_umount, lazy_umount_held_releases_on_close)
{
	int fd;

	ASSERT_EQ(loop_mount(self->dev, self->mp), 0);
	fd = open(self->mp, O_PATH | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(fd, 0);
	ASSERT_EQ(umount2(self->mp, MNT_DETACH), 0);
	EXPECT_FALSE(loop_free(self->dev))
		TH_LOG("%s was released while a file on it was still open", self->dev);
	ASSERT_EQ(close(fd), 0);
	EXPECT_TRUE(loop_free(self->dev))
		TH_LOG("%s is still claimed after the last file on it was closed", self->dev);
}

/* A synchronous umount(2) of a held mount is refused, nothing changes. */
TEST_F(unheld_umount, sync_umount_held_refused)
{
	int fd;

	ASSERT_EQ(loop_mount(self->dev, self->mp), 0);
	fd = open(self->mp, O_PATH | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(fd, 0);
	EXPECT_EQ(umount2(self->mp, 0), -1);
	EXPECT_EQ(errno, EBUSY);
	EXPECT_FALSE(loop_free(self->dev));
	ASSERT_EQ(close(fd), 0);
	ASSERT_EQ(umount2(self->mp, 0), 0);
	EXPECT_TRUE(loop_free(self->dev));
}

/*
 * The mounts of a mount namespace go with its last task: by the time
 * that task can be reaped the filesystems it had mounted are shut down.
 */
TEST_F(unheld_umount, namespace_death_releases_before_reap)
{
	int status, to_parent[2], to_child[2];
	pid_t pid;
	char c;

	ASSERT_EQ(pipe(to_parent), 0);
	ASSERT_EQ(pipe(to_child), 0);
	pid = fork();
	ASSERT_GE(pid, 0);
	if (pid == 0) {
		close(to_parent[0]);
		close(to_child[1]);
		if (unshare(CLONE_NEWNS) || loop_mount(self->dev, self->mp))
			_exit(1);
		if (write(to_parent[1], "", 1) != 1)
			_exit(2);
		/* leave once the parent has looked, with the mount in place */
		if (read(to_child[0], &c, 1) != 0)
			_exit(3);
		_exit(0);
	}
	close(to_parent[1]);
	close(to_child[0]);
	ASSERT_EQ(read(to_parent[0], &c, 1), 1);
	EXPECT_FALSE(loop_free(self->dev))
		TH_LOG("%s is not claimed by the child's mount", self->dev);
	close(to_child[1]);
	ASSERT_EQ(waitpid(pid, &status, 0), pid);
	ASSERT_EQ(status, 0);
	close(to_parent[0]);
	EXPECT_TRUE(loop_free(self->dev))
		TH_LOG("%s is still claimed after the namespace's last task was reaped", self->dev);
}

TEST_HARNESS_MAIN
