// SPDX-License-Identifier: GPL-2.0
/*
 * A refwalk that finds a name whose mounts are detached by an unlink
 * before it looked at them must not step onto what the mounts covered,
 * with or without LOOKUP_REVAL. The mount step fails with ESTALE and the
 * walk is redone with LOOKUP_REVAL, which can't find the removed name.
 * And it must not keep walking for as long as names keep being removed
 * under it.
 *
 * A raw fuse server provides the timing. It answers the revalidation of
 * a name only after a second thread has unlinked that name through the
 * fuse mount itself, while the walker walks an open_tree() copy of it
 * with a tmpfs file mounted over the name.
 *
 * Requires root to mount fuse.
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
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mount.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <linux/fuse.h>

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

#define NR_SERVERS	8
#define WALK_SECS	5

enum {
	ROOT_ID = FUSE_ROOT_ID,
	F1_ID,		/* /f1 */
	LNK_ID,		/* /lnk -> f1 */
	D_ID,		/* /d */
	F_ID,		/* /d/f */
};

/* what a walk must not read: the file the mount covered */
#define UNDER	"UNDER"
#define MARK	"MARK"

static int ffd = -1;
static char fmnt[128], markpath[128];

/* in revalidation: f1 is valid for 3600s, 0 makes every refwalk revalidate it */
static atomic_uint f1_valid;
static atomic_int f1_present;
static atomic_int unlink_f1;		/* unlink F/f1 in the walker's next lookup of f1 */
static atomic_int estale_readlink;	/* fail this pid's next readlink with ESTALE */
static atomic_int walker;

static atomic_int games;		/* make d/f findable and unlink it under the walker */
static atomic_int cover_tfd = -1;	/* cover d/f in this copy of F as well */
static atomic_int f_findable;
static atomic_long nr_unlinks;

/* ---- a helper thread runs the syscalls a server thread waits for ---- */

enum job_kind { JOB_NONE, JOB_STAT, JOB_UNLINK, JOB_COVER };

static pthread_mutex_t job_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t job_cond = PTHREAD_COND_INITIALIZER;
static enum job_kind job;
static char job_path[160];
static const char *job_name;
static int job_seq, done_seq, job_ret;

static int cover(int tfd, const char *name)
{
	int mfd, ret = 0;

	mfd = syscall(SYS_open_tree, AT_FDCWD, markpath,
		      OPEN_TREE_CLONE | OPEN_TREE_CLOEXEC);
	if (mfd < 0)
		return -errno;
	if (syscall(SYS_move_mount, mfd, "", tfd, name, MOVE_MOUNT_F_EMPTY_PATH))
		ret = -errno;
	close(mfd);
	return ret;
}

static void *helper(void *arg)
{
	pthread_mutex_lock(&job_lock);
	for (;;) {
		struct stat st;
		enum job_kind k;
		int ret, seq;

		while (job == JOB_NONE)
			pthread_cond_wait(&job_cond, &job_lock);
		k = job;
		seq = job_seq;
		job = JOB_NONE;
		pthread_mutex_unlock(&job_lock);

		if (k == JOB_STAT)
			ret = stat(job_path, &st) ? -errno : 0;
		else if (k == JOB_UNLINK)
			ret = unlink(job_path) ? -errno : 0;
		else
			ret = cover(atomic_load(&cover_tfd), job_name);

		pthread_mutex_lock(&job_lock);
		job_ret = ret;
		done_seq = seq;
		pthread_cond_broadcast(&job_cond);
	}
	return NULL;
}

/* run @k in the helper and wait for it, for 2s at most */
static int run_job(enum job_kind k, const char *name)
{
	static pthread_mutex_t one_at_a_time = PTHREAD_MUTEX_INITIALIZER;
	struct timespec dl;
	int seq, ret;

	pthread_mutex_lock(&one_at_a_time);
	clock_gettime(CLOCK_REALTIME, &dl);
	dl.tv_sec += 2;
	pthread_mutex_lock(&job_lock);
	seq = ++job_seq;
	job = k;
	job_name = name;
	snprintf(job_path, sizeof(job_path), "%s/%s", fmnt, name);
	pthread_cond_broadcast(&job_cond);
	while (done_seq != seq) {
		if (pthread_cond_timedwait(&job_cond, &job_lock, &dl) == ETIMEDOUT) {
			job_ret = -ETIMEDOUT;
			break;
		}
	}
	ret = done_seq == seq ? job_ret : -ETIMEDOUT;
	pthread_mutex_unlock(&job_lock);
	pthread_mutex_unlock(&one_at_a_time);
	return ret;
}

/* ---- the fuse server ---- */

static void reply(uint64_t unique, int error, const void *data, size_t len)
{
	struct fuse_out_header oh = {
		.len = sizeof(oh) + (error ? 0 : len),
		.error = error,
		.unique = unique,
	};
	struct iovec iov[2] = { { &oh, sizeof(oh) }, { (void *)data, len } };

	/* errors here are teardown races, the device is gone */
	if (writev(ffd, iov, (error || !len) ? 1 : 2) < 0)
		return;
}

static void fill_attr(struct fuse_attr *a, uint64_t id)
{
	memset(a, 0, sizeof(*a));
	a->ino = id;
	a->nlink = 1;
	a->blksize = 4096;
	if (id == ROOT_ID || id == D_ID) {
		a->mode = S_IFDIR | 0755;
		a->nlink = 2;
	} else if (id == LNK_ID) {
		a->mode = S_IFLNK | 0777;
		a->size = 2;
	} else {
		a->mode = S_IFREG | 0644;
		a->size = sizeof(UNDER) - 1;
	}
}

static void reply_entry(uint64_t unique, uint64_t id, uint64_t valid)
{
	struct fuse_entry_out e = {
		.nodeid = id,
		.generation = 1,
		.entry_valid = valid,
		.attr_valid = 3600,
	};

	fill_attr(&e.attr, id);
	reply(unique, 0, &e, sizeof(e));
}

static void lookup(struct fuse_in_header *ih, const char *name)
{
	int w = atomic_load(&walker);
	bool from_walker = w && (int)ih->pid == w;

	if (ih->nodeid == ROOT_ID && !strcmp(name, "f1")) {
		/*
		 * The answer was formed before the unlink went through. A
		 * server is free to send it afterwards.
		 */
		if (from_walker && atomic_exchange(&unlink_f1, 0)) {
			run_job(JOB_UNLINK, "f1");
		} else if (!atomic_load(&f1_present)) {
			reply(ih->unique, -ENOENT, NULL, 0);
			return;
		}
		reply_entry(ih->unique, F1_ID, atomic_load(&f1_valid));
	} else if (ih->nodeid == ROOT_ID && !strcmp(name, "lnk")) {
		reply_entry(ih->unique, LNK_ID, 3600);
	} else if (ih->nodeid == ROOT_ID && !strcmp(name, "d")) {
		/* the walker revalidates d: have d/f looked up (and covered) again */
		if (from_walker && atomic_load(&games)) {
			int ret = atomic_load(&cover_tfd) >= 0 ?
				  run_job(JOB_COVER, "d/f") : run_job(JOB_STAT, "d/f");

			if (!ret)
				atomic_store(&f_findable, 1);
		}
		reply_entry(ih->unique, D_ID, 0);
	} else if (ih->nodeid == D_ID && !strcmp(name, "f")) {
		/* the walker revalidates the d/f it found: unlink it first */
		if (from_walker && atomic_load(&games) &&
		    atomic_exchange(&f_findable, 0) &&
		    !run_job(JOB_UNLINK, "d/f"))
			atomic_fetch_add(&nr_unlinks, 1);
		reply_entry(ih->unique, F_ID, 0);
	} else {
		reply(ih->unique, -ENOENT, NULL, 0);
	}
}

static void *server(void *arg)
{
	size_t size = FUSE_MIN_READ_BUFFER + 65536;
	char *buf = malloc(size);
	struct fuse_in_header *ih = (void *)buf;
	void *in = buf + sizeof(*ih);

	if (!buf)
		return NULL;
	for (;;) {
		ssize_t n = read(ffd, buf, size);

		if (n < 0 && (errno == EINTR || errno == EAGAIN))
			continue;
		if (n < (ssize_t)sizeof(*ih))
			return NULL;	/* the mount is gone */

		switch (ih->opcode) {
		case FUSE_INIT: {
			struct fuse_init_out o = {
				.major = FUSE_KERNEL_VERSION,
				.minor = FUSE_KERNEL_MINOR_VERSION,
				.max_readahead = 65536,
				.max_write = 65536,
				.time_gran = 1,
			};

			reply(ih->unique, 0, &o, sizeof(o));
			break;
		}
		case FUSE_LOOKUP:
			lookup(ih, in);
			break;
		case FUSE_GETATTR: {
			struct fuse_attr_out o = { .attr_valid = 3600 };

			fill_attr(&o.attr, ih->nodeid);
			reply(ih->unique, 0, &o, sizeof(o));
			break;
		}
		case FUSE_READLINK: {
			int pid = atomic_load(&estale_readlink);

			/* one failure is enough to get the open to LOOKUP_REVAL */
			if (pid && (int)ih->pid == pid &&
			    atomic_compare_exchange_strong(&estale_readlink, &pid, 0))
				reply(ih->unique, -ESTALE, NULL, 0);
			else
				reply(ih->unique, 0, "f1", 2);
			break;
		}
		case FUSE_OPEN:
		case FUSE_OPENDIR: {
			struct fuse_open_out o = {
				.fh = ih->nodeid,
				.open_flags = FOPEN_DIRECT_IO,
			};

			reply(ih->unique, 0, &o, sizeof(o));
			break;
		}
		case FUSE_READ: {
			struct fuse_read_in *ri = in;
			size_t len = sizeof(UNDER) - 1;

			if (ri->offset >= len)
				reply(ih->unique, 0, NULL, 0);
			else
				reply(ih->unique, 0, UNDER + ri->offset, len - ri->offset);
			break;
		}
		case FUSE_UNLINK:
			if (ih->nodeid == ROOT_ID && !strcmp(in, "f1"))
				atomic_store(&f1_present, 0);
			reply(ih->unique, 0, NULL, 0);
			break;
		case FUSE_FLUSH:
		case FUSE_RELEASE:
		case FUSE_RELEASEDIR:
		case FUSE_ACCESS:
			reply(ih->unique, 0, NULL, 0);
			break;
		case FUSE_FORGET:
		case FUSE_BATCH_FORGET:
		case FUSE_INTERRUPT:
			break;
		case FUSE_DESTROY:
			reply(ih->unique, 0, NULL, 0);
			return NULL;
		default:
			reply(ih->unique, -ENOSYS, NULL, 0);
			break;
		}
	}
}

/* ---- the walker: a child that opens @path below @dfd once ---- */

struct walk {
	pid_t pid;
	int rfd;
	int err;
	char buf[16];
};

static int walk_start(struct walk *w, int dfd, const char *path, bool estale)
{
	int res[2], go[2];
	char c = 0;

	if (pipe2(res, O_CLOEXEC) || pipe2(go, O_CLOEXEC))
		return -errno;
	w->pid = fork();
	if (w->pid < 0)
		return -errno;
	if (!w->pid) {
		struct { int err; char buf[16]; } r = {};
		int fd;

		close(res[0]);
		close(go[1]);
		if (read(go[0], &c, 1) != 1)
			_exit(1);
		fd = openat(dfd, path, O_RDONLY | O_CLOEXEC);
		if (fd < 0)
			r.err = errno;
		else if (read(fd, r.buf, sizeof(r.buf) - 1) < 0)
			r.err = errno;
		if (write(res[1], &r, sizeof(r)) != sizeof(r))
			_exit(1);
		_exit(0);
	}
	close(res[1]);
	close(go[0]);
	w->rfd = res[0];
	atomic_store(&walker, w->pid);
	if (estale)
		atomic_store(&estale_readlink, w->pid);
	if (write(go[1], &c, 1) != 1)
		return -errno;
	close(go[1]);
	return 0;
}

/* 0 once the walker returned from open(), -ETIMEDOUT if it is still in there */
static int walk_wait(struct walk *w, int secs)
{
	struct pollfd pfd = { .fd = w->rfd, .events = POLLIN };
	struct { int err; char buf[16]; } r;
	int ret = -ETIMEDOUT;

	if (poll(&pfd, 1, secs * 1000) == 1 && read(w->rfd, &r, sizeof(r)) == sizeof(r)) {
		w->err = r.err;
		memcpy(w->buf, r.buf, sizeof(w->buf));
		ret = 0;
	} else {
		kill(w->pid, SIGKILL);
	}
	waitpid(w->pid, NULL, 0);
	close(w->rfd);
	atomic_store(&walker, 0);
	return ret;
}

FIXTURE(unlink_reval)
{
	char dir[64];
	int tfd;
};

FIXTURE_SETUP(unlink_reval)
{
	char opts[128];
	pthread_t t;
	int fd;

	self->tfd = -1;
	if (geteuid() != 0)
		SKIP(return, "need root to mount fuse");
	if (access("/dev/fuse", R_OK | W_OK))
		SKIP(return, "no /dev/fuse");

	ASSERT_EQ(0, unshare(CLONE_NEWNS));
	ASSERT_EQ(0, mount(NULL, "/", NULL, MS_REC | MS_PRIVATE, NULL));

	strcpy(self->dir, "/tmp/unlink_reval_XXXXXX");
	ASSERT_NE(NULL, mkdtemp(self->dir));
	ASSERT_EQ(0, mount("tmpfs", self->dir, "tmpfs", 0, "size=1M"));
	snprintf(fmnt, sizeof(fmnt), "%s/F", self->dir);
	snprintf(markpath, sizeof(markpath), "%s/mark", self->dir);
	ASSERT_EQ(0, mkdir(fmnt, 0755));
	fd = open(markpath, O_CREAT | O_WRONLY | O_CLOEXEC, 0644);
	ASSERT_GE(fd, 0);
	ASSERT_EQ((ssize_t)sizeof(MARK) - 1, write(fd, MARK, sizeof(MARK) - 1));
	close(fd);

	ffd = open("/dev/fuse", O_RDWR | O_CLOEXEC);
	ASSERT_GE(ffd, 0);
	snprintf(opts, sizeof(opts), "fd=%d,rootmode=40000,user_id=0,group_id=0", ffd);
	ASSERT_EQ(0, mount("fuse", fmnt, "fuse", MS_NOSUID | MS_NODEV, opts));
	for (int i = 0; i < NR_SERVERS; i++)
		ASSERT_EQ(0, pthread_create(&t, NULL, server, NULL));
	ASSERT_EQ(0, pthread_create(&t, NULL, helper, NULL));
}

FIXTURE_TEARDOWN(unlink_reval)
{
	if (self->tfd >= 0)
		close(self->tfd);
	if (fmnt[0])
		umount2(fmnt, MNT_DETACH);
	if (self->dir[0]) {
		umount2(self->dir, MNT_DETACH);
		rmdir(self->dir);
	}
}

/* a copy of F with MARK mounted on f1 */
static int cover_f1(struct __test_metadata *_metadata)
{
	char buf[16] = {};
	struct stat st;
	int tfd, fd;

	atomic_store(&f1_present, 1);
	EXPECT_EQ(0, stat(fmnt, &st));
	tfd = syscall(SYS_open_tree, AT_FDCWD, fmnt, OPEN_TREE_CLONE | OPEN_TREE_CLOEXEC);
	EXPECT_GE(tfd, 0);
	/* f1 is in the dcache before the walker gets there */
	EXPECT_EQ(0, fstatat(tfd, "f1", &st, 0));
	EXPECT_EQ(0, cover(tfd, "f1"));
	fd = openat(tfd, "f1", O_RDONLY | O_CLOEXEC);
	EXPECT_GE(fd, 0);
	EXPECT_EQ((ssize_t)sizeof(MARK) - 1, read(fd, buf, sizeof(buf) - 1));
	EXPECT_STREQ(MARK, buf);
	close(fd);
	return tfd;
}

/*
 * The first readlink of lnk fails with ESTALE, so the open of F'/lnk is
 * repeated with LOOKUP_REVAL. In that walk f1 is unlinked while it is
 * revalidated, which detaches MARK. The mount step fails with ESTALE and
 * a walk that already runs with LOOKUP_REVAL isn't redone again.
 */
TEST_F(unlink_reval, reval_walk)
{
	for (int i = 0; i < 4; i++) {
		struct walk w = {};

		atomic_store(&f1_valid, 3600);
		self->tfd = cover_f1(_metadata);
		ASSERT_GE(self->tfd, 0);

		atomic_store(&unlink_f1, 1);
		ASSERT_EQ(0, walk_start(&w, self->tfd, "lnk", true));
		ASSERT_EQ(0, walk_wait(&w, WALK_SECS)) {
			TH_LOG("open() of F'/lnk did not return");
		}
		EXPECT_EQ(0, atomic_load(&unlink_f1)) {
			TH_LOG("f1 was never revalidated");
		}
		ASSERT_NE(0, w.err) {
			TH_LOG("round %d: read \"%s\" under the detached MARK", i, w.buf);
		}
		EXPECT_EQ(ESTALE, w.err);
		close(self->tfd);
		self->tfd = -1;
	}
}

/*
 * The same in the walk without LOOKUP_REVAL: the mount step fails with
 * ESTALE, the walk is redone with LOOKUP_REVAL and can't find the
 * unlinked name.
 */
TEST_F(unlink_reval, ref_walk)
{
	for (int i = 0; i < 4; i++) {
		struct walk w = {};

		atomic_store(&f1_valid, 0);
		self->tfd = cover_f1(_metadata);
		ASSERT_GE(self->tfd, 0);

		atomic_store(&unlink_f1, 1);
		ASSERT_EQ(0, walk_start(&w, self->tfd, "f1", false));
		ASSERT_EQ(0, walk_wait(&w, WALK_SECS)) {
			TH_LOG("open() of F'/f1 did not return");
		}
		EXPECT_EQ(0, atomic_load(&unlink_f1)) {
			TH_LOG("f1 was never revalidated");
		}
		ASSERT_NE(0, w.err) {
			TH_LOG("round %d: read \"%s\" under the detached MARK", i, w.buf);
		}
		EXPECT_EQ(ENOENT, w.err);
		close(self->tfd);
		self->tfd = -1;
	}
}

/*
 * Every walk of F'/d/f finds a covered f that is unlinked while it is
 * revalidated: the revalidation of d has it looked up and covered again.
 * The open must neither read what MARK covered nor go on forever: the
 * redone walk runs into the second removal and fails with ESTALE.
 */
TEST_F(unlink_reval, covered_name_keeps_going)
{
	struct walk w = {};
	struct stat st;

	ASSERT_EQ(0, stat(fmnt, &st));
	self->tfd = syscall(SYS_open_tree, AT_FDCWD, fmnt, OPEN_TREE_CLONE | OPEN_TREE_CLOEXEC);
	ASSERT_GE(self->tfd, 0);
	/* d is in the dcache, so the walker only ever revalidates it */
	ASSERT_EQ(0, fstatat(self->tfd, "d", &st, 0));

	atomic_store(&cover_tfd, self->tfd);
	atomic_store(&games, 1);
	ASSERT_EQ(0, walk_start(&w, self->tfd, "d/f", false));
	ASSERT_EQ(0, walk_wait(&w, WALK_SECS)) {
		atomic_store(&games, 0);
		TH_LOG("open() of F'/d/f still running after %ds and %ld unlinks",
		       WALK_SECS, atomic_load(&nr_unlinks));
	}
	atomic_store(&games, 0);
	EXPECT_GE(atomic_load(&nr_unlinks), 1);
	if (w.err) {
		EXPECT_TRUE(w.err == ESTALE || w.err == ENOENT) {
			TH_LOG("open() of F'/d/f failed with %s", strerror(w.err));
		}
	} else {
		EXPECT_STREQ(MARK, w.buf) {
			TH_LOG("read \"%s\" under the detached MARK", w.buf);
		}
	}
}

/* The same without mounts: nothing to reveal, but it must end as well. */
TEST_F(unlink_reval, plain_name_keeps_going)
{
	struct walk w = {};
	struct stat st;
	char path[160];

	snprintf(path, sizeof(path), "%s/d/f", fmnt);
	ASSERT_EQ(0, fstatat(AT_FDCWD, fmnt, &st, 0));
	ASSERT_EQ(0, stat(path, &st));

	atomic_store(&games, 1);
	ASSERT_EQ(0, walk_start(&w, AT_FDCWD, path, false));
	ASSERT_EQ(0, walk_wait(&w, WALK_SECS)) {
		atomic_store(&games, 0);
		TH_LOG("open() of F/d/f still running after %ds and %ld unlinks",
		       WALK_SECS, atomic_load(&nr_unlinks));
	}
	atomic_store(&games, 0);
	EXPECT_GE(atomic_load(&nr_unlinks), 1);
	if (w.err) {
		EXPECT_EQ(ENOENT, w.err);
	}
}

TEST_HARNESS_MAIN
