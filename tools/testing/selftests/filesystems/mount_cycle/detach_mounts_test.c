// SPDX-License-Identifier: GPL-2.0
/*
 * unlink(), rmdir() and rename() may remove a name that is a mountpoint in
 * another mount namespace. The mounts on it are detached and the name is
 * unhashed. A walk that found the name before the removal must not end on
 * what the mounts covered, a mount on the root of a bind of the removed
 * object stays with the bind, and a walk that holds the removed name
 * already keeps working.
 *
 * Everything here runs on tmpfs. The races are bounded loops, the rest is
 * deterministic.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/vfs.h>
#include <sys/wait.h>
#include <linux/openat2.h>

#include "../../kselftest_harness.h"

#ifndef OPEN_TREE_CLONE
#define OPEN_TREE_CLONE		1
#endif
#ifndef OPEN_TREE_CLOEXEC
#define OPEN_TREE_CLOEXEC	O_CLOEXEC
#endif
#ifndef MOVE_MOUNT_F_EMPTY_PATH
#define MOVE_MOUNT_F_EMPTY_PATH	0x00000004
#endif

#define DIR_LEN		64
#define PATH_LEN	192
#define SECRET		"secret"
#define MASK		"mask"
#define RACE_ROUNDS	300
#define WALK_ROUNDS	120
#define WALKERS		4

/* exit codes of the children, each test says what they mean */
enum {
	CHILD_OK,
	CHILD_SETUP,
	CHILD_STEP1,
	CHILD_STEP2,
	CHILD_STEP3,
	CHILD_STEP4,
	CHILD_STEP5,
};

static int write_file(const char *path, const char *s)
{
	ssize_t n = -1;
	int fd;

	fd = open(path, O_WRONLY | O_CLOEXEC);
	if (fd >= 0) {
		n = write(fd, s, strlen(s));
		close(fd);
	}
	return n == (ssize_t)strlen(s) ? 0 : -1;
}

static int create_file(const char *path, const char *s)
{
	ssize_t n = -1;
	int fd;

	fd = open(path, O_WRONLY | O_CREAT | O_TRUNC | O_CLOEXEC, 0644);
	if (fd >= 0) {
		n = write(fd, s, strlen(s));
		close(fd);
	}
	return n == (ssize_t)strlen(s) ? 0 : -1;
}

/* Root in a new user namespace with a copy of the mount namespace. */
static int enter_userns(void)
{
	uid_t uid = getuid();
	gid_t gid = getgid();
	char map[32];

	prctl(PR_SET_DUMPABLE, 1);
	if (unshare(CLONE_NEWUSER | CLONE_NEWNS))
		return -1;
	if (write_file("/proc/self/setgroups", "deny") && errno != ENOENT)
		return -1;
	snprintf(map, sizeof(map), "0 %d 1", uid);
	if (write_file("/proc/self/uid_map", map))
		return -1;
	snprintf(map, sizeof(map), "0 %d 1", gid);
	if (write_file("/proc/self/gid_map", map))
		return -1;
	return setgid(0) || setuid(0) ? -1 : 0;
}

static int wait_child(pid_t pid)
{
	int status;

	if (waitpid(pid, &status, 0) != pid || !WIFEXITED(status))
		return -1;
	return WEXITSTATUS(status);
}

static int wait_byte(int fd)
{
	char c;

	return read(fd, &c, 1) == 1 ? 0 : -1;
}

static int send_byte(int fd)
{
	return write(fd, "x", 1) == 1 ? 0 : -1;
}

/* What @path reads, "" if it can't be read. */
static int read_word(const char *path, char *buf, size_t len)
{
	ssize_t n = 0;
	int fd;

	fd = open(path, O_RDONLY | O_CLOEXEC);
	if (fd >= 0) {
		n = read(fd, buf, len - 1);
		close(fd);
	}
	buf[n > 0 ? n : 0] = 0;
	return fd < 0 ? -1 : 0;
}

/* a copy of the directory @dfd with @cover mounted on @name in it */
static int covered_copy(int dfd, const char *cover, const char *name)
{
	int tfd, mfd;

	tfd = syscall(SYS_open_tree, dfd, "",
		      OPEN_TREE_CLONE | OPEN_TREE_CLOEXEC | AT_EMPTY_PATH);
	if (tfd < 0)
		return -1;
	mfd = syscall(SYS_open_tree, dfd, cover, OPEN_TREE_CLONE | OPEN_TREE_CLOEXEC);
	if (mfd < 0 || syscall(SYS_move_mount, mfd, "", tfd, name, MOVE_MOUNT_F_EMPTY_PATH)) {
		close(tfd);
		tfd = -1;
	}
	if (mfd >= 0)
		close(mfd);
	return tfd;
}

FIXTURE(detach_mounts) {
	char base[DIR_LEN];
	int to_child[2];
	int from_child[2];
};

FIXTURE_SETUP(detach_mounts)
{
	char p[PATH_LEN];

	if (geteuid())
		SKIP(return, "test requires root");

	snprintf(self->base, sizeof(self->base), "/tmp/detach_mounts.XXXXXX");
	ASSERT_NE(mkdtemp(self->base), NULL);
	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL), 0);
	ASSERT_EQ(mount("tmpfs", self->base, "tmpfs", 0, "mode=0755"), 0);
	snprintf(p, sizeof(p), "%s/F", self->base);
	ASSERT_EQ(mkdir(p, 0755), 0);
	ASSERT_EQ(mount("tmpfs", p, "tmpfs", 0, "mode=0755"), 0);
	snprintf(p, sizeof(p), "%s/F/secret", self->base);
	ASSERT_EQ(create_file(p, SECRET), 0);
	snprintf(p, sizeof(p), "%s/F/mask", self->base);
	ASSERT_EQ(create_file(p, MASK), 0);
	snprintf(p, sizeof(p), "%s/F/other", self->base);
	ASSERT_EQ(create_file(p, "other"), 0);
	snprintf(p, sizeof(p), "%s/T", self->base);
	ASSERT_EQ(mkdir(p, 0755), 0);
	snprintf(p, sizeof(p), "%s/T/x", self->base);
	ASSERT_EQ(create_file(p, "x"), 0);
	ASSERT_EQ(pipe(self->to_child), 0);
	ASSERT_EQ(pipe(self->from_child), 0);
}

FIXTURE_TEARDOWN(detach_mounts)
{
	umount2(self->base, MNT_DETACH);
	rmdir(self->base);
}

/*
 * The child binds F/secret on T/x (M1) and F/mask on top of it (M2) in a
 * mount namespace of its own and locks both by entering a user namespace.
 * It reads T/x before and after the parent removed the name F/secret, and
 * MASK is the only thing it may read: M2 sits on the root of M1, which
 * is reached by crossing onto M1, not by the name that went away.
 */
static int stacked_child(const char *base, int to_parent, int from_parent)
{
	char secret[PATH_LEN], mask[PATH_LEN], x[PATH_LEN], buf[16];

	snprintf(secret, sizeof(secret), "%s/F/secret", base);
	snprintf(mask, sizeof(mask), "%s/F/mask", base);
	snprintf(x, sizeof(x), "%s/T/x", base);

	if (unshare(CLONE_NEWNS))
		return CHILD_SETUP;
	if (mount(secret, x, NULL, MS_BIND, NULL) ||
	    mount(mask, x, NULL, MS_BIND, NULL))
		return CHILD_SETUP;
	if (enter_userns())
		return CHILD_SETUP;
	if (read_word(x, buf, sizeof(buf)) || strcmp(buf, MASK))
		return CHILD_STEP1;
	/* the cover is locked */
	if (!umount2(x, MNT_DETACH) || errno != EINVAL)
		return CHILD_STEP2;
	if (send_byte(to_parent) || wait_byte(from_parent))
		return CHILD_SETUP;
	if (read_word(x, buf, sizeof(buf)) || strcmp(buf, MASK))
		return CHILD_STEP3;
	return CHILD_OK;
}

static int stacked(FIXTURE_DATA(detach_mounts) *self, bool by_rename)
{
	char p[PATH_LEN], q[PATH_LEN];
	pid_t pid;

	pid = fork();
	if (pid < 0)
		return -1;
	if (pid == 0)
		_exit(stacked_child(self->base, self->from_child[1],
				    self->to_child[0]));
	if (wait_byte(self->from_child[0]))
		return -1;
	/* nothing is mounted on F/secret here */
	snprintf(p, sizeof(p), "%s/F/secret", self->base);
	snprintf(q, sizeof(q), "%s/F/other", self->base);
	if (by_rename ? rename(q, p) : unlink(p))
		return -1;
	if (send_byte(self->to_child[1]))
		return -1;
	return wait_child(pid);
}

TEST_F(detach_mounts, overmount_stays_after_unlink)
{
	int ret = stacked(self, false);

	TH_LOG("child exit code %d", ret);
	EXPECT_EQ(ret, CHILD_OK);
}

TEST_F(detach_mounts, overmount_stays_after_rename)
{
	int ret = stacked(self, true);

	TH_LOG("child exit code %d", ret);
	EXPECT_EQ(ret, CHILD_OK);
}

/*
 * The same stack, unlocked, and the owner takes it down by path after the
 * removal: the first umount removes M2 and shows the root of M1, the
 * removed file, the second removes M1 and shows T/x again.
 */
static int umount_order_child(const char *base, int to_parent, int from_parent)
{
	char secret[PATH_LEN], mask[PATH_LEN], x[PATH_LEN], buf[16];

	snprintf(secret, sizeof(secret), "%s/F/secret", base);
	snprintf(mask, sizeof(mask), "%s/F/mask", base);
	snprintf(x, sizeof(x), "%s/T/x", base);

	if (unshare(CLONE_NEWNS) || mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL))
		return CHILD_SETUP;
	if (mount(secret, x, NULL, MS_BIND, NULL) ||
	    mount(mask, x, NULL, MS_BIND, NULL))
		return CHILD_SETUP;
	if (send_byte(to_parent) || wait_byte(from_parent))
		return CHILD_SETUP;
	if (read_word(x, buf, sizeof(buf)) || strcmp(buf, MASK))
		return CHILD_STEP1;
	if (umount2(x, 0))
		return CHILD_STEP2;
	if (read_word(x, buf, sizeof(buf)) || strcmp(buf, SECRET))
		return CHILD_STEP3;
	if (umount2(x, 0))
		return CHILD_STEP4;
	if (read_word(x, buf, sizeof(buf)) || strcmp(buf, "x"))
		return CHILD_STEP5;
	return CHILD_OK;
}

TEST_F(detach_mounts, umount_takes_the_overmount_first)
{
	char p[PATH_LEN];
	pid_t pid;
	int ret;

	pid = fork();
	ASSERT_GE(pid, 0);
	if (pid == 0)
		_exit(umount_order_child(self->base, self->from_child[1],
					 self->to_child[0]));
	ASSERT_EQ(wait_byte(self->from_child[0]), 0);
	snprintf(p, sizeof(p), "%s/F/secret", self->base);
	ASSERT_EQ(unlink(p), 0);
	ASSERT_EQ(send_byte(self->to_child[1]), 0);
	ret = wait_child(pid);
	TH_LOG("child exit code %d", ret);
	EXPECT_EQ(ret, CHILD_OK);
}

/*
 * M1 binds F/secret on itself and M2 covers it. A walk that crossed onto
 * M1 before F/secret went away must not end on the file, whatever else
 * it finds there. The walk goes through a magic link so that the last
 * component is looked up in refwalk. Each round makes the mounts again
 * in a fresh copy of the mount namespace.
 */
static int selfbind_child(const char *base, int to_parent, int from_parent)
{
	char secret[PATH_LEN], mask[PATH_LEN], f[PATH_LEN], path[PATH_LEN];
	int hits = 0;

	snprintf(secret, sizeof(secret), "%s/F/secret", base);
	snprintf(mask, sizeof(mask), "%s/F/mask", base);
	snprintf(f, sizeof(f), "%s/F", base);

	for (int round = 0; round < RACE_ROUNDS; round++) {
		char buf[16];
		int dfd;

		if (wait_byte(from_parent))
			return CHILD_SETUP;
		if (unshare(CLONE_NEWNS))
			return CHILD_SETUP;
		if (mount(secret, secret, NULL, MS_BIND, NULL) ||
		    mount(mask, secret, NULL, MS_BIND, NULL))
			return CHILD_SETUP;
		/* on the copy of F that carries this round's mounts */
		dfd = open(f, O_PATH | O_DIRECTORY | O_CLOEXEC);
		if (dfd < 0)
			return CHILD_SETUP;
		snprintf(path, sizeof(path), "/proc/self/fd/%d/secret", dfd);
		if (send_byte(to_parent))
			return CHILD_SETUP;
		for (int i = 0; i < 20000; i++) {
			if (read_word(path, buf, sizeof(buf)))
				break;
			if (!strcmp(buf, SECRET)) {
				hits++;
				break;
			}
		}
		close(dfd);
		if (send_byte(to_parent))
			return CHILD_SETUP;
	}
	return hits ? CHILD_STEP1 : CHILD_OK;
}

TEST_F(detach_mounts, self_bind_race)
{
	char p[PATH_LEN];
	pid_t pid;
	int ret;

	pid = fork();
	ASSERT_GE(pid, 0);
	if (pid == 0)
		_exit(selfbind_child(self->base, self->from_child[1],
				     self->to_child[0]));
	snprintf(p, sizeof(p), "%s/F/secret", self->base);
	srand(getpid());
	for (int round = 0; round < RACE_ROUNDS; round++) {
		ASSERT_EQ(create_file(p, SECRET), 0);
		ASSERT_EQ(send_byte(self->to_child[1]), 0);
		ASSERT_EQ(wait_byte(self->from_child[0]), 0);
		usleep(rand() % 2000);
		ASSERT_EQ(unlink(p), 0);
		ASSERT_EQ(wait_byte(self->from_child[0]), 0);
	}
	ret = wait_child(pid);
	TH_LOG("child exit code %d", ret);
	EXPECT_EQ(ret, CHILD_OK);
}

/*
 * The bind M1 of S/secret on T2/x is a peer of the shared S. M2 on T2/x
 * sits on the root of M1 and propagates a copy onto S/secret. A mount
 * namespace made before that removes S/secret: the propagated copy goes,
 * M2 stays. The umount of M2 then looks the removed dentry up on every
 * peer of M1, S included, and must get nothing there.
 */
TEST_F(detach_mounts, overmount_stays_on_a_shared_source)
{
	char s[PATH_LEN], secret[PATH_LEN], t2[PATH_LEN], x[PATH_LEN];
	int ret, err;
	pid_t pid;

	snprintf(s, sizeof(s), "%s/S", self->base);
	snprintf(secret, sizeof(secret), "%s/S/secret", self->base);
	snprintf(t2, sizeof(t2), "%s/T2", self->base);
	snprintf(x, sizeof(x), "%s/T2/x", self->base);
	ASSERT_EQ(mkdir(s, 0755), 0);
	ASSERT_EQ(mount("tmpfs", s, "tmpfs", 0, NULL), 0);
	ASSERT_EQ(mount(NULL, s, NULL, MS_SHARED, NULL), 0);
	ASSERT_EQ(mkdir(t2, 0755), 0);
	ASSERT_EQ(mount("tmpfs", t2, "tmpfs", 0, NULL), 0);
	ASSERT_EQ(mount(NULL, t2, NULL, MS_PRIVATE, NULL), 0);
	ASSERT_EQ(mkdir(secret, 0755), 0);
	ASSERT_EQ(mkdir(x, 0755), 0);

	pid = fork();
	ASSERT_GE(pid, 0);
	if (pid == 0) {
		/* a namespace from before the mounts, its S private */
		if (unshare(CLONE_NEWNS) || mount(NULL, s, NULL, MS_PRIVATE, NULL))
			_exit(CHILD_SETUP);
		if (send_byte(self->from_child[1]) || wait_byte(self->to_child[0]))
			_exit(CHILD_SETUP);
		_exit(rmdir(secret) ? CHILD_STEP1 : CHILD_OK);
	}
	ASSERT_EQ(wait_byte(self->from_child[0]), 0);
	ASSERT_EQ(mount(secret, x, NULL, MS_BIND, NULL), 0);
	ASSERT_EQ(mount("tmpfs", x, "tmpfs", 0, NULL), 0);
	ASSERT_EQ(send_byte(self->to_child[1]), 0);
	ret = wait_child(pid);
	ASSERT_EQ(ret, CHILD_OK) {
		TH_LOG("child exit code %d", ret);
	}

	/* the propagated copy went with the name, M2 stayed on M1 */
	EXPECT_EQ(access(secret, F_OK), -1);
	EXPECT_EQ(errno, ENOENT);
	err = umount2(x, 0) ? errno : 0;
	EXPECT_EQ(err, 0) {
		TH_LOG("umount of M2: %s", strerror(err));
	}
	EXPECT_EQ(access(x, F_OK), 0);
	err = umount2(x, 0) ? errno : 0;
	EXPECT_EQ(err, 0) {
		TH_LOG("umount of M1: %s", strerror(err));
	}
}

/*
 * Three holders sit on names they never walked to: a chroot into D, a
 * directory descriptor on D and a cwd in E/sub. Another mount namespace
 * covers D and E with locked binds, then D, E/sub and E are removed. Every
 * step in place on a removed name has to keep working: ".." at the chroot
 * root and under RESOLVE_IN_ROOT, umount(".") in D, and ".." from the
 * removed E/sub up onto the removed E and out.
 */
static int chroot_child(const char *d, int to_parent, int from_parent)
{
	struct stat st;

	if (chroot(d) || chdir("/"))
		return CHILD_SETUP;
	if (send_byte(to_parent) || wait_byte(from_parent))
		return CHILD_SETUP;
	if (stat("/", &st))
		return CHILD_STEP1;
	if (stat("..", &st))
		return CHILD_STEP2;
	if (!umount2(".", 0) || errno != EINVAL)
		return CHILD_STEP3;
	if (chdir(".."))
		return CHILD_STEP4;
	return CHILD_OK;
}

static int below_child(const char *sub, int to_parent, int from_parent)
{
	struct stat st;

	if (chdir(sub))
		return CHILD_SETUP;
	if (send_byte(to_parent) || wait_byte(from_parent))
		return CHILD_SETUP;
	if (stat("..", &st))
		return CHILD_STEP1;
	if (chdir(".."))
		return CHILD_STEP2;
	if (chdir(".."))
		return CHILD_STEP3;
	return CHILD_OK;
}

static int armer_child(const char *src, const char *d, const char *e,
		       int to_parent, int from_parent)
{
	if (enter_userns())
		return CHILD_SETUP;
	if (mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL))
		return CHILD_SETUP;
	if (mount(src, d, NULL, MS_BIND, NULL) || mount(src, e, NULL, MS_BIND, NULL))
		return CHILD_SETUP;
	if (send_byte(to_parent) || wait_byte(from_parent))
		return CHILD_SETUP;
	return CHILD_OK;
}

TEST_F(detach_mounts, in_place_steps_on_removed_names)
{
	char d[PATH_LEN], e[PATH_LEN], sub[PATH_LEN], src[PATH_LEN];
	int ready[2], go_c[2], go_b[2], dfd, ret, fd;
	struct open_how how = {};
	pid_t armer, chr, blw;

	snprintf(d, sizeof(d), "%s/D", self->base);
	snprintf(e, sizeof(e), "%s/E", self->base);
	snprintf(sub, sizeof(sub), "%s/E/sub", self->base);
	snprintf(src, sizeof(src), "%s/src", self->base);
	ASSERT_EQ(mkdir(d, 0755), 0);
	ASSERT_EQ(mkdir(e, 0755), 0);
	ASSERT_EQ(mkdir(sub, 0755), 0);
	ASSERT_EQ(mkdir(src, 0755), 0);
	ASSERT_EQ(pipe(ready), 0);
	ASSERT_EQ(pipe(go_c), 0);
	ASSERT_EQ(pipe(go_b), 0);

	armer = fork();
	ASSERT_GE(armer, 0);
	if (armer == 0)
		_exit(armer_child(src, d, e, ready[1], self->to_child[0]));
	ASSERT_EQ(wait_byte(ready[0]), 0);
	chr = fork();
	ASSERT_GE(chr, 0);
	if (chr == 0)
		_exit(chroot_child(d, ready[1], go_c[0]));
	blw = fork();
	ASSERT_GE(blw, 0);
	if (blw == 0)
		_exit(below_child(sub, ready[1], go_b[0]));
	ASSERT_EQ(wait_byte(ready[0]), 0);
	ASSERT_EQ(wait_byte(ready[0]), 0);

	dfd = open(d, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(dfd, 0);
	/* a removed child doesn't count for rmdir, so E goes as well */
	EXPECT_EQ(rmdir(d), 0);
	EXPECT_EQ(rmdir(sub), 0);
	EXPECT_EQ(rmdir(e), 0);

	how.flags = O_PATH | O_DIRECTORY;
	how.resolve = RESOLVE_IN_ROOT;
	fd = syscall(SYS_openat2, dfd, "..", &how, sizeof(how));
	EXPECT_GE(fd, 0) {
		TH_LOG("openat2(dfd, \"..\", RESOLVE_IN_ROOT): %s", strerror(errno));
	}
	if (fd >= 0)
		close(fd);
	how.resolve = 0;
	fd = syscall(SYS_openat2, dfd, "..", &how, sizeof(how));
	EXPECT_GE(fd, 0) {
		TH_LOG("openat2(dfd, \"..\"): %s", strerror(errno));
	}
	if (fd >= 0)
		close(fd);
	fd = openat(dfd, ".", O_PATH | O_DIRECTORY | O_CLOEXEC);
	EXPECT_GE(fd, 0);
	if (fd >= 0)
		close(fd);
	close(dfd);

	ASSERT_EQ(send_byte(go_c[1]), 0);
	ret = wait_child(chr);
	TH_LOG("chroot child exit code %d", ret);
	EXPECT_EQ(ret, CHILD_OK);
	ASSERT_EQ(send_byte(go_b[1]), 0);
	ret = wait_child(blw);
	TH_LOG("cwd child exit code %d", ret);
	EXPECT_EQ(ret, CHILD_OK);
	ASSERT_EQ(send_byte(self->to_child[1]), 0);
	EXPECT_EQ(wait_child(armer), CHILD_OK);
}

/*
 * A mount on a proc entry below a held proc mount: rmdir of the proc
 * mountpoint from another namespace unmounts proc with the entry's mount
 * leaving its cover behind. Reaping the process then runs d_invalidate()
 * on its proc directory, which must not loop on the covered entry.
 */
TEST_F(detach_mounts, reaper_ends_with_a_cover_below)
{
	char u[PATH_LEN], fdinfo[PATH_LEN];
	int kill_p[2], reaped[2], ufd, err, ret;
	pid_t other, reaper, victim;
	struct pollfd pfd;

	snprintf(u, sizeof(u), "%s/u", self->base);
	ASSERT_EQ(mkdir(u, 0755), 0);
	ASSERT_EQ(pipe(kill_p), 0);
	ASSERT_EQ(pipe(reaped), 0);

	/* the namespace that removes u, nothing is mounted on it there */
	other = fork();
	ASSERT_GE(other, 0);
	if (other == 0) {
		if (unshare(CLONE_NEWNS) || mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL))
			_exit(CHILD_SETUP);
		if (send_byte(self->from_child[1]) || wait_byte(self->to_child[0]))
			_exit(CHILD_SETUP);
		_exit(rmdir(u) ? CHILD_STEP1 : CHILD_OK);
	}
	ASSERT_EQ(wait_byte(self->from_child[0]), 0);

	/* the reaper owns the victim, so a stuck reaping doesn't stall us */
	reaper = fork();
	ASSERT_GE(reaper, 0);
	if (reaper == 0) {
		victim = fork();
		if (victim == 0) {
			pause();
			_exit(0);
		}
		if (write(self->from_child[1], &victim, sizeof(victim)) != sizeof(victim) ||
		    wait_byte(kill_p[0]))
			_exit(CHILD_SETUP);
		kill(victim, SIGKILL);
		waitpid(victim, NULL, 0);
		_exit(send_byte(reaped[1]) ? CHILD_SETUP : CHILD_OK);
	}
	ASSERT_EQ(read(self->from_child[0], &victim, sizeof(victim)), (ssize_t)sizeof(victim));

	ASSERT_EQ(mount("proc", u, "proc", 0, NULL), 0);
	snprintf(fdinfo, sizeof(fdinfo), "%s/u/%d/fdinfo", self->base, victim);
	ASSERT_EQ(mount("tmpfs", fdinfo, "tmpfs", 0, NULL), 0);
	ufd = open(u, O_PATH | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(ufd, 0);

	ASSERT_EQ(send_byte(self->to_child[1]), 0);
	ret = wait_child(other);
	ASSERT_EQ(ret, CHILD_OK) {
		TH_LOG("remover exit code %d", ret);
	}

	ASSERT_EQ(send_byte(kill_p[1]), 0);
	pfd.fd = reaped[0];
	pfd.events = POLLIN;
	err = poll(&pfd, 1, 10000);
	EXPECT_EQ(err, 1) {
		TH_LOG("the reaper didn't come back in 10s");
		kill(reaper, SIGKILL);
	}
	waitpid(reaper, NULL, 0);
	close(ufd);
}

/*
 * A file that only a cover keeps a mountpoint: U binds T on T2/u, C binds a
 * file on U/f and U is held. rmdir of T2/u from another namespace unmounts
 * U and C leaves its cover on U at f. The unlink of T/f through T, where
 * nothing is mounted on it, must give the blocks back at once.
 */
TEST_F(detach_mounts, unlinked_file_is_freed_at_once)
{
	char t[PATH_LEN], f[PATH_LEN], t2[PATH_LEN], u[PATH_LEN], uf[PATH_LEN];
	char t4[PATH_LEN], c[PATH_LEN];
	static char buf[1 << 20];
	struct statfs sf;
	int ufd, fd, ret;
	pid_t other;
	long used;

	snprintf(t, sizeof(t), "%s/T3", self->base);
	snprintf(f, sizeof(f), "%s/T3/f", self->base);
	snprintf(t2, sizeof(t2), "%s/T2", self->base);
	snprintf(u, sizeof(u), "%s/T2/u", self->base);
	snprintf(uf, sizeof(uf), "%s/T2/u/f", self->base);
	snprintf(t4, sizeof(t4), "%s/T4", self->base);
	snprintf(c, sizeof(c), "%s/T4/c", self->base);
	ASSERT_EQ(mkdir(t, 0755), 0);
	ASSERT_EQ(mount("tmpfs", t, "tmpfs", 0, "size=96m"), 0);
	ASSERT_EQ(mkdir(t2, 0755), 0);
	ASSERT_EQ(mount("tmpfs", t2, "tmpfs", 0, NULL), 0);
	ASSERT_EQ(mkdir(t4, 0755), 0);
	ASSERT_EQ(mount("tmpfs", t4, "tmpfs", 0, NULL), 0);
	ASSERT_EQ(mkdir(u, 0755), 0);
	ASSERT_EQ(create_file(c, "c"), 0);
	fd = open(f, O_WRONLY | O_CREAT | O_CLOEXEC, 0644);
	ASSERT_GE(fd, 0);
	memset(buf, 'x', sizeof(buf));
	for (int i = 0; i < 64; i++)
		ASSERT_EQ(write(fd, buf, sizeof(buf)), (ssize_t)sizeof(buf));
	close(fd);

	other = fork();
	ASSERT_GE(other, 0);
	if (other == 0) {
		if (unshare(CLONE_NEWNS) || mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL))
			_exit(CHILD_SETUP);
		if (send_byte(self->from_child[1]) || wait_byte(self->to_child[0]))
			_exit(CHILD_SETUP);
		_exit(rmdir(u) ? CHILD_STEP1 : CHILD_OK);
	}
	ASSERT_EQ(wait_byte(self->from_child[0]), 0);
	ASSERT_EQ(mount(t, u, NULL, MS_BIND, NULL), 0);
	ASSERT_EQ(mount(c, uf, NULL, MS_BIND, NULL), 0);
	ufd = open(u, O_PATH | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(ufd, 0);
	ASSERT_EQ(statfs(t, &sf), 0);
	used = (long)((sf.f_blocks - sf.f_bfree) * sf.f_bsize >> 20);
	EXPECT_GE(used, 64);

	ASSERT_EQ(send_byte(self->to_child[1]), 0);
	ret = wait_child(other);
	ASSERT_EQ(ret, CHILD_OK) {
		TH_LOG("remover exit code %d", ret);
	}

	ASSERT_EQ(unlink(f), 0);
	ASSERT_EQ(statfs(t, &sf), 0);
	used = (long)((sf.f_blocks - sf.f_bfree) * sf.f_bsize >> 20);
	EXPECT_LT(used, 2) {
		TH_LOG("%ld MB still used after the unlink while the bind is held", used);
	}
	close(ufd);
}

/*
 * Walkers open a covered name through a copy of its directory while the
 * name is removed where nothing is mounted on it. A file that is unlinked
 * or renamed over, and an empty directory that is removed. The walkers may
 * read the mask, miss the name or find what replaced it. They must never
 * read what the mask covered or end on the removed directory, and an
 * ESTALE is for a walk that ran into a second removal.
 */
struct race {
	atomic_int stop;
	atomic_int publishing;
	pthread_rwlock_t lock;	/* the walkers use @dfd under it, the driver swaps it */
	int dfd;
	atomic_long walks, mask, enoent, estale, other, hits;
	ino_t secret_ino;
	const char *name;
	bool dir;
};

static void *race_walker(void *arg)
{
	struct race *r = arg;

	while (!atomic_load(&r->stop)) {
		char buf[16];
		struct stat st;
		ssize_t n;
		int fd;

		/* let the driver in, it waits for every reader to let go */
		while (atomic_load(&r->publishing) && !atomic_load(&r->stop))
			usleep(20);
		pthread_rwlock_rdlock(&r->lock);
		if (r->dfd < 0) {
			pthread_rwlock_unlock(&r->lock);
			usleep(50);
			continue;
		}
		fd = openat(r->dfd, r->name, O_RDONLY | O_CLOEXEC | (r->dir ? O_DIRECTORY : 0));
		atomic_fetch_add(&r->walks, 1);
		if (fd < 0) {
			if (errno == ENOENT)
				atomic_fetch_add(&r->enoent, 1);
			else if (errno == ESTALE)
				atomic_fetch_add(&r->estale, 1);
			else
				atomic_fetch_add(&r->other, 1);
		} else {
			if (r->dir) {
				if (fstat(fd, &st) || st.st_ino == r->secret_ino)
					atomic_fetch_add(&r->hits, 1);
				else
					atomic_fetch_add(&r->mask, 1);
			} else {
				n = read(fd, buf, sizeof(buf) - 1);
				buf[n > 0 ? n : 0] = 0;
				if (!strcmp(buf, SECRET))
					atomic_fetch_add(&r->hits, 1);
				else
					atomic_fetch_add(&r->mask, 1);
			}
			close(fd);
		}
		pthread_rwlock_unlock(&r->lock);
	}
	return NULL;
}

/* hand the walkers a new copy, or none; the old one is closed afterwards */
static int race_publish(struct race *r, int dfd)
{
	int old;

	atomic_store(&r->publishing, 1);
	pthread_rwlock_wrlock(&r->lock);
	old = r->dfd;
	r->dfd = dfd;
	pthread_rwlock_unlock(&r->lock);
	atomic_store(&r->publishing, 0);
	return old;
}

static void race_run(struct __test_metadata *_metadata, const char *base,
		     const char *name, bool dir, bool by_rename)
{
	struct race r = { .lock = PTHREAD_RWLOCK_INITIALIZER, .dfd = -1, .name = name, .dir = dir };
	char a[PATH_LEN], cover[PATH_LEN], victim[PATH_LEN], other[PATH_LEN];
	pthread_t t[WALKERS];
	struct stat st;
	int afd;

	snprintf(a, sizeof(a), "%s/A", base);
	snprintf(cover, sizeof(cover), "%s/A/cover", base);
	snprintf(victim, sizeof(victim), "%s/A/%s", base, name);
	snprintf(other, sizeof(other), "%s/A/other", base);
	ASSERT_EQ(mkdir(a, 0755), 0);
	ASSERT_EQ(mount("tmpfs", a, "tmpfs", 0, NULL), 0);
	if (dir) {
		ASSERT_EQ(mkdir(cover, 0755), 0);
	} else {
		ASSERT_EQ(create_file(cover, MASK), 0);
	}
	afd = open(a, O_PATH | O_DIRECTORY | O_CLOEXEC);
	ASSERT_GE(afd, 0);
	for (int i = 0; i < WALKERS; i++)
		ASSERT_EQ(pthread_create(&t[i], NULL, race_walker, &r), 0);
	srand(getpid());
	for (int round = 0; round < WALK_ROUNDS; round++) {
		int tfd, dfd;

		if (dir) {
			ASSERT_EQ(mkdir(victim, 0755), 0);
			ASSERT_EQ(stat(victim, &st), 0);
			r.secret_ino = st.st_ino;
		} else {
			ASSERT_EQ(create_file(victim, SECRET), 0);
			if (by_rename) {
				ASSERT_EQ(create_file(other, "other"), 0);
			}
		}
		tfd = covered_copy(afd, "cover", name);
		ASSERT_GE(tfd, 0);
		dfd = openat(tfd, ".", O_PATH | O_DIRECTORY | O_CLOEXEC);
		ASSERT_GE(dfd, 0);
		/* the copy is unmounted, dfd holds it */
		close(tfd);
		race_publish(&r, dfd);
		usleep(rand() % 1500);
		/* through A, where nothing is mounted on the name */
		if (dir) {
			ASSERT_EQ(rmdir(victim), 0);
		} else if (by_rename) {
			ASSERT_EQ(rename(other, victim), 0);
		} else {
			ASSERT_EQ(unlink(victim), 0);
		}
		usleep(rand() % 300);
		close(race_publish(&r, -1));
		if (!dir && by_rename) {
			ASSERT_EQ(unlink(victim), 0);
		}
	}
	atomic_store(&r.stop, 1);
	for (int i = 0; i < WALKERS; i++)
		pthread_join(t[i], NULL);
	close(afd);
	TH_LOG("%s %s: walks %ld mask %ld enoent %ld estale %ld other %ld hits %ld",
	       dir ? "directory" : "file", by_rename ? "renamed over" : "removed",
	       atomic_load(&r.walks), atomic_load(&r.mask), atomic_load(&r.enoent),
	       atomic_load(&r.estale), atomic_load(&r.other), atomic_load(&r.hits));
	EXPECT_EQ(atomic_load(&r.hits), 0);
	EXPECT_EQ(atomic_load(&r.other), 0);
	umount2(a, MNT_DETACH);
}

TEST_F(detach_mounts, walks_racing_unlink_of_a_covered_file)
{
	race_run(_metadata, self->base, "f", false, false);
}

TEST_F(detach_mounts, walks_racing_rename_over_a_covered_file)
{
	race_run(_metadata, self->base, "f", false, true);
}

TEST_F(detach_mounts, walks_racing_rmdir_of_a_covered_directory)
{
	race_run(_metadata, self->base, "d", true, false);
}

TEST_HARNESS_MAIN
