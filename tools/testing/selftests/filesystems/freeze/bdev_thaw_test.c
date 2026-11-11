// SPDX-License-Identifier: GPL-2.0
/*
 * Thawing a superblock behind the block layer's back.
 *
 * A device-mapper suspend freezes the filesystem on the device through
 * bdev_freeze() with the same userspace holder FIFREEZE uses, so FITHAW can
 * drop that freeze while the device is still suspended. The resume then
 * finds nothing left to thaw. Check that it still thaws the device so that
 * it can be mounted again afterwards. The filesystem is mounted read-only
 * so the freeze and the thaw don't need to write to the suspended device.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <linux/dm-ioctl.h>
#include <linux/fs.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include "../../kselftest_harness.h"
#include "loop_util.h"

#define IMG_SIZE (64 << 20)

FIXTURE(bdev_thaw) {
	char dir[64];
	char img[96];
	char mnt[96];
	char node[96];
	char name[32];
	int loop;
	int mntfd;
	bool tmpfs;
	bool dm;
	bool suspended;
	bool mounted;
};

static int dm_ioctl(unsigned long cmd, const char *name, uint32_t flags, dev_t *dev)
{
	struct dm_ioctl dmi = {
		.version = { DM_VERSION_MAJOR, 0, 0 },
		.data_size = sizeof(dmi),
		.data_start = sizeof(dmi),
		.flags = flags,
	};
	int fd, ret = 0;

	strncpy(dmi.name, name, sizeof(dmi.name) - 1);
	fd = open("/dev/mapper/control", O_RDWR | O_CLOEXEC);
	if (fd < 0)
		return -errno;
	if (ioctl(fd, cmd, &dmi))
		ret = -errno;
	else if (dev)
		*dev = makedev((dmi.dev >> 8) & 0xfff,
			       (dmi.dev & 0xff) | ((dmi.dev >> 12) & 0xfff00));
	close(fd);
	return ret;
}

/* Remove @name now, or on its last close if udev still has it open. */
static int dm_remove(const char *name)
{
	return dm_ioctl(DM_DEV_REMOVE, name, DM_DEFERRED_REMOVE, NULL);
}

/* Create @name as a linear mapping of all of @backing and make it live. */
static int dm_create_linear(const char *name, dev_t backing, uint64_t sectors,
			    dev_t *dev)
{
	struct {
		struct dm_ioctl dmi;
		struct dm_target_spec spec;
		char params[64];
	} __attribute__((aligned(8))) t = {};
	int fd, ret;

	ret = dm_ioctl(DM_DEV_CREATE, name, 0, NULL);
	if (ret)
		return ret;

	t.dmi.version[0] = DM_VERSION_MAJOR;
	t.dmi.data_size = sizeof(t);
	t.dmi.data_start = sizeof(t.dmi);
	t.dmi.target_count = 1;
	strncpy(t.dmi.name, name, sizeof(t.dmi.name) - 1);
	t.spec.length = sectors;
	strcpy(t.spec.target_type, "linear");
	snprintf(t.params, sizeof(t.params), "%u:%u 0", major(backing), minor(backing));

	fd = open("/dev/mapper/control", O_RDWR | O_CLOEXEC);
	if (fd >= 0) {
		ret = ioctl(fd, DM_TABLE_LOAD, &t) ? -errno : 0;
		close(fd);
	} else {
		ret = -errno;
	}
	/* Resuming makes the loaded table live. */
	if (!ret)
		ret = dm_ioctl(DM_DEV_SUSPEND, name, 0, dev);
	if (ret)
		dm_remove(name);
	return ret;
}

static void bdev_thaw_cleanup(FIXTURE_DATA(bdev_thaw) *self)
{
	if (self->suspended)
		dm_ioctl(DM_DEV_SUSPEND, self->name, 0, NULL);
	if (self->mntfd >= 0)
		close(self->mntfd);
	if (self->mounted)
		umount2(self->mnt, MNT_DETACH);
	if (self->dm)
		dm_remove(self->name);
	if (self->loop >= 0)
		loop_clear(self->loop);
	unlink(self->node);
	unlink(self->img);
	rmdir(self->mnt);
	if (self->tmpfs)
		umount2(self->dir, MNT_DETACH);
	rmdir(self->dir);
}

FIXTURE_SETUP(bdev_thaw)
{
	char cmd[160], dev[32];
	struct stat st;
	dev_t dmdev;
	int fd, ret;

	self->loop = -1;
	self->mntfd = -1;
	self->tmpfs = false;
	self->dm = false;
	self->suspended = false;
	self->mounted = false;

	if (geteuid() != 0)
		SKIP(return, "test requires CAP_SYS_ADMIN");
	if (access("/dev/loop-control", R_OK | W_OK))
		SKIP(return, "test requires loop devices");
	if (access("/dev/mapper/control", R_OK | W_OK))
		SKIP(return, "test requires device-mapper");

	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(mount(NULL, "/", NULL, MS_REC | MS_PRIVATE, NULL), 0);

	snprintf(self->dir, sizeof(self->dir), "/tmp/bdev_thaw.XXXXXX");
	ASSERT_NE(mkdtemp(self->dir), NULL);
	/* /tmp may be mounted nodev, the dm node needs a tmpfs of our own. */
	ASSERT_EQ(mount("tmpfs", self->dir, "tmpfs", MS_NOSUID | MS_NOEXEC, NULL), 0)
		rmdir(self->dir);
	self->tmpfs = true;
	snprintf(self->img, sizeof(self->img), "%s/img", self->dir);
	snprintf(self->mnt, sizeof(self->mnt), "%s/mnt", self->dir);
	snprintf(self->node, sizeof(self->node), "%s/dm", self->dir);
	snprintf(self->name, sizeof(self->name), "bdev_thaw_%d", getpid());
	/* The harness doesn't run the teardown when the setup fails. */
	ASSERT_EQ(mkdir(self->mnt, 0700), 0)
		bdev_thaw_cleanup(self);

	fd = open(self->img, O_RDWR | O_CREAT | O_CLOEXEC, 0600);
	ASSERT_GE(fd, 0)
		bdev_thaw_cleanup(self);
	ASSERT_EQ(ftruncate(fd, IMG_SIZE), 0)
		bdev_thaw_cleanup(self);

	snprintf(cmd, sizeof(cmd), "mkfs.ext4 -q -F %s >/dev/null 2>&1", self->img);
	ret = system(cmd);
	if (ret) {
		close(fd);
		bdev_thaw_cleanup(self);
		SKIP(return, "mkfs.ext4 failed or is not available");
	}

	self->loop = loop_bind(fd);
	close(fd);
	if (loop_missing(self->loop)) {
		bdev_thaw_cleanup(self);
		SKIP(return, "test requires loop devices");
	}
	ASSERT_GE(self->loop, 0)
		bdev_thaw_cleanup(self);
	snprintf(dev, sizeof(dev), "/dev/loop%d", self->loop);
	ASSERT_EQ(stat(dev, &st), 0)
		bdev_thaw_cleanup(self);

	ASSERT_EQ(dm_create_linear(self->name, st.st_rdev, IMG_SIZE >> 9, &dmdev), 0)
		bdev_thaw_cleanup(self);
	self->dm = true;
	ASSERT_EQ(mknod(self->node, S_IFBLK | 0600, dmdev), 0)
		bdev_thaw_cleanup(self);

	if (mount(self->node, self->mnt, "ext4", MS_RDONLY, NULL)) {
		if (errno == ENODEV) {
			bdev_thaw_cleanup(self);
			SKIP(return, "kernel lacks ext4");
		}
		ASSERT_EQ(errno, 0)
			bdev_thaw_cleanup(self);
	}
	self->mounted = true;
	self->mntfd = open(self->mnt, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(self->mntfd, 0)
		bdev_thaw_cleanup(self);
}

FIXTURE_TEARDOWN(bdev_thaw)
{
	bdev_thaw_cleanup(self);
}

TEST_F(bdev_thaw, fithaw_during_dm_suspend)
{
	/* A suspend with lockfs freezes the filesystem through bdev_freeze(). */
	ASSERT_EQ(dm_ioctl(DM_DEV_SUSPEND, self->name, DM_SUSPEND_FLAG, NULL), 0);
	self->suspended = true;

	/* FITHAW drops the freeze the suspend holds. */
	if (ioctl(self->mntfd, FITHAW, 0)) {
		ASSERT_EQ(errno, EINVAL);
		SKIP(return, "FITHAW doesn't drop a device-mapper freeze");
	}

	/* The resume's bdev_thaw() finds nothing left to thaw. */
	ASSERT_EQ(dm_ioctl(DM_DEV_SUSPEND, self->name, 0, NULL), 0);
	self->suspended = false;

	close(self->mntfd);
	self->mntfd = -1;
	ASSERT_EQ(umount2(self->mnt, 0), 0);
	self->mounted = false;

	ASSERT_EQ(mount(self->node, self->mnt, "ext4", MS_RDONLY, NULL), 0)
		TH_LOG("device still frozen after the resume: %s", strerror(errno));
	self->mounted = true;
}

TEST_HARNESS_MAIN
