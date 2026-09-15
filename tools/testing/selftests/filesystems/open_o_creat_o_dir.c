// SPDX-License-Identifier: GPL-2.0
#include <sys/stat.h>
#include <errno.h>
#include <limits.h>
#include <fcntl.h>

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
 * O_EXCL together with O_CREAT|O_DIRECTORY should succeed if the target
 * directory does not yet exist. After directory creation, repeating this
 * call must fail with EEXIST.
 */
TEST_F(open_o_creat_o_dir, o_excl_eexist)
{
	int excldir_fd;

	excldir_fd = openat_o_mkdir_checked_flags(self->dfd, "excldir", O_EXCL);

	EXPECT_EQ(openat_o_mkdir(excldir_fd, ".", O_EXCL, S_IRWXU), -1);
	EXPECT_EQ(errno, EEXIST);

	EXPECT_EQ(close(excldir_fd), 0);
	EXPECT_EQ(unlinkat(self->dfd, "excldir", AT_REMOVEDIR), 0);
}

/*
 * O_CREAT|O_DIRECTORY on a path that already exists as a regular file
 * must fail with ENOTDIR.
 */
TEST_F(open_o_creat_o_dir, existing_file_enotdir)
{
	int file_fd;

	file_fd = openat(self->dfd, "regfile",
			 O_CREAT | O_RDONLY, S_IRWXU);
	ASSERT_GE(file_fd, 0);
	EXPECT_EQ(close(file_fd), 0);

	EXPECT_EQ(openat_o_mkdir(self->dfd, "regfile", O_RDONLY, S_IRWXU), -1);
	EXPECT_EQ(errno, ENOTDIR);

	EXPECT_EQ(unlinkat(self->dfd, "regfile", 0), 0);
}

/*
 * O_CREAT|O_DIRECTORY combined with a writable access mode must be
 * rejected: a directory cannot be opened for writing.
 */
TEST_F(open_o_creat_o_dir, rejects_writable_acc_mode)
{
	EXPECT_EQ(openat_o_mkdir(self->dfd, "rdwrdir", O_RDWR, S_IRWXU), -1);
	EXPECT_EQ(errno, ENOTDIR);
	/* Clean up if the kernel created the directory anyway. */
	unlinkat(self->dfd, "rdwrdir", AT_REMOVEDIR);
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

	/* Best-effort cleanup in case the kernel left a file behind. */
	if (fd >= 0)
		close(fd);
	unlinkat(self->dfd, "trailing", 0);
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

TEST_HARNESS_MAIN
