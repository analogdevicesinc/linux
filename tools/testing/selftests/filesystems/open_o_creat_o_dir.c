// SPDX-License-Identifier: GPL-2.0
#include <sys/stat.h>
#include <sys/vfs.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/magic.h>

#include "kselftest_harness.h"
#include "wrappers.h"

#define openat_o_mkdir_checked_flags(dfd, pathname, flags) ({	\
	struct stat __st;						\
	int __fd = openat_o_mkdir(dfd, pathname, flags, S_IRWXU);	\
	ASSERT_GE(__fd, 0);						\
	ASSERT_EQ(fstat(__fd, &__st), 0);				\
	EXPECT_TRUE(S_ISDIR(__st.st_mode));				\
	__fd;								\
})

#define openat_o_mkdir_checked(dfd, pathname) \
	openat_o_mkdir_checked_flags(dfd, pathname, O_RDONLY)

FIXTURE(open_o_creat_o_dir) {
	char dirpath[PATH_MAX];
	int dfd;
};

FIXTURE_SETUP(open_o_creat_o_dir)
{
	strcpy(self->dirpath, "/tmp/open_o_creat_o_dir_test.XXXXXX");
	ASSERT_NE(mkdtemp(self->dirpath), NULL);
	self->dfd = open(self->dirpath, O_DIRECTORY);
	ASSERT_GE(self->dfd, 0);
}

FIXTURE_TEARDOWN(open_o_creat_o_dir)
{
	close(self->dfd);
	rmdir(self->dirpath);
}

/* Does open_o_creat_o_dir return a fd at all? */
TEST_F(open_o_creat_o_dir, returns_fd)
{
	int fd = openat_o_mkdir_checked(self->dfd, "newdir");
	EXPECT_EQ(close(fd), 0);
	EXPECT_EQ(unlinkat(self->dfd, "newdir", AT_REMOVEDIR), 0);
}

/* The fd must refer to the directory that was just created. */
TEST_F(open_o_creat_o_dir, fd_is_created_dir)
{
	int fd;
	struct stat st_via_fd, st_via_path;
	char path[PATH_MAX];

	fd = openat_o_mkdir_checked(self->dfd, "checkdir");

	ASSERT_EQ(fstat(fd, &st_via_fd), 0);

	snprintf(path, sizeof(path), "%s/checkdir", self->dirpath);
	ASSERT_EQ(stat(path, &st_via_path), 0);

	EXPECT_EQ(st_via_fd.st_ino, st_via_path.st_ino);
	EXPECT_EQ(st_via_fd.st_dev, st_via_path.st_dev);

	EXPECT_EQ(close(fd), 0);
	EXPECT_EQ(rmdir(path), 0);
}

/* Missing parent component must fail with ENOENT. */
TEST_F(open_o_creat_o_dir, enoent_missing_parent)
{
	EXPECT_EQ(openat_o_mkdir(self->dfd, "nonexistent/child", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, ENOENT);
}

/* An invalid dfd must fail with EBADF. */
TEST_F(open_o_creat_o_dir, ebadf)
{
	EXPECT_EQ(openat_o_mkdir(FD_INVALID, "badfdir", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, EBADF);
}

/* A dfd that points to a file (not a directory) must fail with ENOTDIR. */
TEST_F(open_o_creat_o_dir, enotdir_dfd)
{
	int file_fd;

	file_fd = openat(self->dfd, "file",
			 O_CREAT | O_RDONLY, S_IRWXU);
	ASSERT_GE(file_fd, 0);

	EXPECT_EQ(openat_o_mkdir(file_fd, "subdir", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, ENOTDIR);

	EXPECT_EQ(close(file_fd), 0);
	EXPECT_EQ(unlinkat(self->dfd, "file", 0), 0);
}

/*
 * O_CREAT|O_DIRECTORY without O_EXCL must fail with EINVAL and must not
 * create anything at the path.
 */
TEST_F(open_o_creat_o_dir, requires_o_excl)
{
	struct stat st;

	EXPECT_EQ(openat(self->dfd, "noexcl", O_CREAT | O_DIRECTORY | O_RDONLY,
			 S_IRWXU), -1);
	EXPECT_EQ(errno, EINVAL);

	EXPECT_EQ(fstatat(self->dfd, "noexcl", &st, 0), -1);
	EXPECT_EQ(errno, ENOENT);
}

/*
 * Once the directory exists, repeating O_CREAT|O_DIRECTORY|O_EXCL must
 * fail with EEXIST.
 */
TEST_F(open_o_creat_o_dir, existing_dir_eexist)
{
	int excldir_fd;

	excldir_fd = openat_o_mkdir_checked(self->dfd, "excldir");

	EXPECT_EQ(openat_o_mkdir(self->dfd, "excldir", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, EEXIST);

	EXPECT_EQ(close(excldir_fd), 0);
	EXPECT_EQ(unlinkat(self->dfd, "excldir", AT_REMOVEDIR), 0);
}

/*
 * O_CREAT|O_DIRECTORY combined with a writable access mode must be
 * rejected: a directory cannot be opened for writing.
 */
TEST_F(open_o_creat_o_dir, rejects_writable_acc_mode)
{
	EXPECT_EQ(openat_o_mkdir(self->dfd, "rdwrdir", O_RDWR, S_IRWXU), -1);
	EXPECT_EQ(errno, ENOTDIR);
}

/*
 * openat(O_CREAT|O_DIRECTORY) with a trailing slash should work.
 */
TEST_F(open_o_creat_o_dir, trailing_slash)
{
	int fd = openat_o_mkdir_checked(self->dfd, "newdir/");
	EXPECT_EQ(close(fd), 0);
	EXPECT_EQ(unlinkat(self->dfd, "newdir", AT_REMOVEDIR), 0);
}

/*
 * openat(O_CREAT) with a trailing slash but without O_DIRECTORY
 * must fail with EISDIR and must not create anything at the path.
 */
TEST_F(open_o_creat_o_dir, trailing_slash_no_o_dir)
{
	int fd;
	struct stat st;

	fd = openat(self->dfd, "trailing/", O_CREAT | O_RDONLY, S_IRWXU);
	EXPECT_EQ(fd, -1);
	EXPECT_EQ(errno, EISDIR);

	EXPECT_EQ(fstatat(self->dfd, "trailing", &st, 0), -1);
	EXPECT_EQ(errno, ENOENT);
}

/*
 * The returned fd must be usable as a dfd for further *at() calls.
 */
TEST_F(open_o_creat_o_dir, fd_usable_as_dfd)
{
	int parent_fd, child_fd;
	char path[PATH_MAX];

	parent_fd = openat_o_mkdir_checked(self->dfd, "parent");
	child_fd = openat_o_mkdir_checked(parent_fd, "child");

	EXPECT_EQ(close(child_fd), 0);
	EXPECT_EQ(close(parent_fd), 0);

	snprintf(path, sizeof(path), "%s/parent/child", self->dirpath);
	EXPECT_EQ(rmdir(path), 0);
	snprintf(path, sizeof(path), "%s/parent", self->dirpath);
	EXPECT_EQ(rmdir(path), 0);
}

/*
 * O_EXCL means a trailing symlink is never followed, so O_CREAT|O_DIRECTORY
 * must refuse to create through a dangling one, and must not create
 * anything at the symlink target.
 */
TEST_F(open_o_creat_o_dir, dangling_symlink_eexist)
{
	struct stat st;

	ASSERT_EQ(symlinkat("danglink_target", self->dfd, "danglink"), 0);

	EXPECT_EQ(openat_o_mkdir(self->dfd, "danglink", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, EEXIST);

	/* Nothing must have been created at the target. */
	EXPECT_EQ(fstatat(self->dfd, "danglink_target", &st, 0), -1);
	EXPECT_EQ(errno, ENOENT);

	EXPECT_EQ(unlinkat(self->dfd, "danglink", 0), 0);
}

/*
 * A trailing symlink is an existing entry, even if it resolves to a
 * directory, so O_CREAT|O_DIRECTORY must fail with EEXIST.
 */
TEST_F(open_o_creat_o_dir, symlink_to_dir_eexist)
{
	ASSERT_EQ(mkdirat(self->dfd, "realdir", 0700), 0);
	ASSERT_EQ(symlinkat("realdir", self->dfd, "dirlink"), 0);

	EXPECT_EQ(openat_o_mkdir(self->dfd, "dirlink", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, EEXIST);

	EXPECT_EQ(unlinkat(self->dfd, "dirlink", 0), 0);
	EXPECT_EQ(unlinkat(self->dfd, "realdir", AT_REMOVEDIR), 0);
}

/*
 * O_TMPFILE is encoded as __O_TMPFILE|O_DIRECTORY.  Now that O_CREAT is no
 * longer rejected alongside O_DIRECTORY, O_TMPFILE|O_CREAT must still be
 * rejected explicitly so that it cannot create a persistent file on kernels
 * that predate O_TMPFILE.
 */
TEST_F(open_o_creat_o_dir, tmpfile_with_o_creat_einval)
{
	EXPECT_EQ(openat(self->dfd, ".", O_TMPFILE | O_CREAT | O_RDWR, S_IRWXU),
		  -1);
	EXPECT_EQ(errno, EINVAL);
}

/*
 * Open the first of @paths that is a mounted filesystem of type @magic.
 * Returns -1 if there is none.
 */
static int open_fs_root(const char *const *paths, long magic)
{
	for (; *paths; paths++) {
		struct statfs sfs;
		int fd;

		fd = open(*paths, O_DIRECTORY | O_RDONLY);
		if (fd < 0)
			continue;
		if (!fstatfs(fd, &sfs) && sfs.f_type == magic)
			return fd;
		close(fd);
	}
	return -1;
}

/*
 * kernfs: O_CREAT|O_DIRECTORY on cgroup2 must create a cgroup and return a
 * fd to that very cgroup.
 */
TEST(kernfs_cgroup2)
{
	static const char *const paths[] = {
		"/sys/fs/cgroup", "/sys/fs/cgroup/unified", NULL
	};
	struct stat st_fd, st_path;
	int root_fd, fd, procs_fd;
	char name[64];

	if (geteuid() != 0)
		SKIP(return, "needs root to create a cgroup");
	root_fd = open_fs_root(paths, CGROUP2_SUPER_MAGIC);
	if (root_fd < 0)
		SKIP(return, "cgroup2 is not mounted");

	snprintf(name, sizeof(name), "open_o_creat_o_dir.%d", getpid());

	fd = openat_o_mkdir_checked(root_fd, name);

	/* The fd refers to the cgroup that is now visible at the name. */
	ASSERT_EQ(fstat(fd, &st_fd), 0);
	ASSERT_EQ(fstatat(root_fd, name, &st_path, 0), 0);
	EXPECT_EQ(st_fd.st_ino, st_path.st_ino);
	EXPECT_EQ(st_fd.st_dev, st_path.st_dev);

	/* It is a populated cgroup, not just an empty directory. */
	procs_fd = openat(fd, "cgroup.procs", O_RDONLY);
	EXPECT_GE(procs_fd, 0);
	if (procs_fd >= 0)
		EXPECT_EQ(close(procs_fd), 0);

	EXPECT_EQ(close(fd), 0);
	EXPECT_EQ(unlinkat(root_fd, name, AT_REMOVEDIR), 0);
	EXPECT_EQ(close(root_fd), 0);
}

/*
 * tracefs sets SB_I_NO_ATOMIC_MKDIR: O_CREAT|O_DIRECTORY is refused with
 * EOPNOTSUPP before any lookup, whether or not the name exists, and
 * creates nothing.
 */
TEST(tracefs_eopnotsupp)
{
	static const char *const paths[] = {
		"/sys/kernel/tracing", "/sys/kernel/debug/tracing", NULL
	};
	int tracing_fd, inst_fd;
	char name[64];
	struct stat st;

	if (geteuid() != 0)
		SKIP(return, "needs root to access tracefs");
	tracing_fd = open_fs_root(paths, TRACEFS_MAGIC);
	if (tracing_fd < 0)
		SKIP(return, "tracefs is not mounted");
	inst_fd = openat(tracing_fd, "instances", O_DIRECTORY | O_RDONLY);
	if (inst_fd < 0) {
		close(tracing_fd);
		SKIP(return, "no tracefs instances directory");
	}

	snprintf(name, sizeof(name), "open_o_creat_o_dir.%d", getpid());

	/* Nonexistent name, where mkdir(2) would create an instance */
	EXPECT_EQ(openat_o_mkdir(inst_fd, name, O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, EOPNOTSUPP);
	EXPECT_EQ(fstatat(inst_fd, name, &st, 0), -1);
	EXPECT_EQ(errno, ENOENT);

	/* Existing name */
	EXPECT_EQ(openat_o_mkdir(tracing_fd, "instances", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, EOPNOTSUPP);

	EXPECT_EQ(close(inst_fd), 0);
	EXPECT_EQ(close(tracing_fd), 0);
}

TEST_HARNESS_MAIN
