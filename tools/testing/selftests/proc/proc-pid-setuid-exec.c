// SPDX-License-Identifier: GPL-2.0
/*
 * Files in /proc/<pid> that were opened before the task executed a set-user-ID
 * program must stop working if their opener has no access to the new program,
 * no matter who reads them.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <grp.h>
#include <limits.h>
#include <linux/userfaultfd.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#include <sys/sendfile.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include "../kselftest_harness.h"

#define TEST_UID	65534
#define TARGET_FD	100	/* open before and after the exec */
#define NEW_FD		101	/* only opened by the new program */
#define SELF_KSM_FD	102	/* ksm_stat, read by the new program */
#define HELPER_ARG	"helper"
#define BIN_DIR		"/tmp"
#define SUID_BIN	BIN_DIR "/suid"
#define PLAIN_BIN	BIN_DIR "/plain"
#define LISTED_NEW_FD	10	/* getdents_overtaken() saw NEW_FD */
#define OPEN_AS_ROOT	(1U << 0)
#define READ_AS_ROOT	(1U << 1)

enum probe {
	PROBE_FD_LOOKUP,
	PROBE_FDINFO,
	PROBE_KSM,
	PROBE_FD_DIR,
	PROBE_FDINFO_DIR,
	PROBE_KSM_SELF,		/* read by the new program itself, keep last */
	NR_PROBES,
};

static const char *const probe_name[NR_PROBES] = {
	[PROBE_FD_LOOKUP]	= "fstatat() below fd/ opened with O_PATH",
	[PROBE_FDINFO]		= "read() of fdinfo/<fd>",
	[PROBE_KSM]		= "read() of ksm_stat",
	[PROBE_FD_DIR]		= "getdents64() of fd/",
	[PROBE_FDINFO_DIR]	= "getdents64() of fdinfo/",
	[PROBE_KSM_SELF]	= "read() of ksm_stat by the new program",
};

enum result {
	WORKS,
	DENIED,
	FAILED,		/* any other error */
	MISSING,	/* couldn't be opened */
};

#define RES_SIZE	(sizeof(enum result) * NR_PROBES)

struct target {
	pid_t pid;
	int go;
	int ready;
	enum result ksm_self;	/* what the new program read from SELF_KSM_FD */
};

struct linux_dirent64 {
	uint64_t d_ino;
	int64_t d_off;
	unsigned short d_reclen;
	unsigned char d_type;
	char d_name[];
};

static enum result result_of(ssize_t ret)
{
	if (ret >= 0)
		return WORKS;
	return errno == EACCES ? DENIED : FAILED;
}

/* The new program reads the ksm_stat its predecessor opened. */
static enum result read_self_ksm(void)
{
	char buf[256];
	ssize_t ret;

	ret = read(SELF_KSM_FD, buf, sizeof(buf));
	if (ret < 0 && errno == EBADF)
		return MISSING;
	return result_of(ret);
}

/* Runs as the new program: open a file of its own and wait to be killed. */
static int helper(void)
{
	char res = read_self_ksm();
	int fd = open("/dev/null", O_RDONLY);

	if (fd < 0 || dup2(fd, NEW_FD) < 0 || write(TARGET_FD, &res, 1) != 1)
		return 1;
	pause();
	return 0;
}

static int copy_self(const char *path, mode_t mode)
{
	int in, out, ret = -1;
	ssize_t n;

	in = open("/proc/self/exe", O_RDONLY | O_CLOEXEC);
	if (in < 0)
		return -1;
	out = open(path, O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0700);
	if (out >= 0) {
		while ((n = sendfile(out, in, NULL, INT_MAX)) > 0)
			;
		if (!n && !fchmod(out, mode))
			ret = 0;
		close(out);
	}
	close(in);
	return ret;
}

static int drop_privileges(void)
{
	if (setgroups(0, NULL) ||
	    setresgid(TEST_UID, TEST_UID, TEST_UID) ||
	    setresuid(TEST_UID, TEST_UID, TEST_UID))
		return -1;
	return 0;
}

/* Starts out as TEST_UID and executes @bin when told to. */
static int start_target(struct target *t, const char *bin)
{
	int go[2], ready[2];
	char c;

	if (pipe2(go, O_CLOEXEC) || pipe2(ready, O_CLOEXEC))
		return -1;

	t->pid = fork();
	if (t->pid < 0)
		return -1;
	if (!t->pid) {
		int fd;

		/* Dropping privileges made it undumpable. */
		if (drop_privileges() || prctl(PR_SET_DUMPABLE, 1))
			_exit(1);
		/* TARGET_FD survives the exec, the new program reports on it. */
		if (dup2(ready[1], TARGET_FD) < 0)
			_exit(1);
		/* So does SELF_KSM_FD, the new program reads it. */
		fd = open("/proc/self/ksm_stat", O_RDONLY);
		if (fd >= 0 && (dup2(fd, SELF_KSM_FD) < 0 || close(fd)))
			_exit(1);
		if (write(TARGET_FD, "a", 1) != 1 || read(go[0], &c, 1) != 1)
			_exit(1);
		execl(bin, bin, HELPER_ARG, NULL);
		_exit(1);
	}
	close(go[0]);
	close(ready[1]);
	t->go = go[1];
	t->ready = ready[0];
	return read(t->ready, &c, 1) == 1 ? 0 : -1;
}

/* Lets the target exec and collects the new program's SELF_KSM_FD result. */
static int exec_target(struct target *t)
{
	char c;

	if (write(t->go, "x", 1) != 1 || read(t->ready, &c, 1) != 1)
		return -1;
	t->ksm_self = c;
	return 0;
}

static void stop_target(struct target *t)
{
	kill(t->pid, SIGKILL);
	waitpid(t->pid, NULL, 0);
	close(t->go);
	close(t->ready);
}

static void open_probes(pid_t pid, int fds[NR_PROBES])
{
	char path[64];

	snprintf(path, sizeof(path), "/proc/%d/fd", pid);
	fds[PROBE_FD_LOOKUP] = open(path, O_PATH | O_DIRECTORY | O_CLOEXEC);
	fds[PROBE_FD_DIR] = open(path, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
	snprintf(path, sizeof(path), "/proc/%d/fdinfo/%d", pid, TARGET_FD);
	fds[PROBE_FDINFO] = open(path, O_RDONLY | O_CLOEXEC);
	snprintf(path, sizeof(path), "/proc/%d/fdinfo", pid);
	fds[PROBE_FDINFO_DIR] = open(path, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
	snprintf(path, sizeof(path), "/proc/%d/ksm_stat", pid);
	fds[PROBE_KSM] = open(path, O_RDONLY | O_CLOEXEC);
	/* The target did that one itself. */
	fds[PROBE_KSM_SELF] = -1;
}

/* Open the probes as TEST_UID and hand them over. */
static int open_probes_unprivileged(pid_t pid, int fds[NR_PROBES])
{
	char cbuf[CMSG_SPACE(sizeof(int) * NR_PROBES)] = {};
	int present[NR_PROBES], sk[2], i, n = 0, status;
	struct iovec iov = { present, sizeof(present) };
	struct msghdr msg = { .msg_iov = &iov, .msg_iovlen = 1,
			      .msg_control = cbuf };
	struct cmsghdr *cmsg;
	int *rights;
	pid_t child;

	if (socketpair(AF_UNIX, SOCK_STREAM | SOCK_CLOEXEC, 0, sk))
		return -1;

	child = fork();
	if (child < 0)
		return -1;
	if (!child) {
		int mine[NR_PROBES];

		close(sk[0]);
		if (drop_privileges())
			_exit(1);
		open_probes(pid, mine);
		cmsg = (struct cmsghdr *)cbuf;
		rights = (int *)CMSG_DATA(cmsg);
		for (i = 0; i < NR_PROBES; i++) {
			present[i] = mine[i] >= 0;
			if (present[i])
				rights[n++] = mine[i];
		}
		cmsg->cmsg_level = SOL_SOCKET;
		cmsg->cmsg_type = SCM_RIGHTS;
		cmsg->cmsg_len = CMSG_LEN(sizeof(int) * n);
		msg.msg_controllen = n ? CMSG_SPACE(sizeof(int) * n) : 0;
		if (!n)
			msg.msg_control = NULL;
		_exit(sendmsg(sk[1], &msg, 0) != sizeof(present));
	}

	close(sk[1]);
	msg.msg_controllen = sizeof(cbuf);
	if (recvmsg(sk[0], &msg, 0) != sizeof(present) ||
	    waitpid(child, &status, 0) != child || status)
		return -1;
	cmsg = CMSG_FIRSTHDR(&msg);
	rights = cmsg ? (int *)CMSG_DATA(cmsg) : NULL;
	for (i = 0; i < NR_PROBES; i++)
		fds[i] = present[i] && rights ? rights[n++] : -1;
	close(sk[0]);
	return 0;
}

static enum result probe(enum probe p, int fd)
{
	char buf[4096], name[16];
	struct stat st;
	ssize_t ret;

	if (fd < 0)
		return MISSING;

	switch (p) {
	case PROBE_FD_LOOKUP:
		snprintf(name, sizeof(name), "%d", TARGET_FD);
		ret = fstatat(fd, name, &st, AT_SYMLINK_NOFOLLOW);
		break;
	case PROBE_FDINFO:
	case PROBE_KSM:
		ret = read(fd, buf, sizeof(buf));
		break;
	case PROBE_FD_DIR:
	case PROBE_FDINFO_DIR:
		/* The dots may come before the error. */
		while ((ret = syscall(SYS_getdents64, fd, buf, sizeof(buf))) > 0)
			;
		break;
	default:
		return FAILED;
	}
	return result_of(ret);
}

/* The fd/ lookup goes first, nothing may look fd/ up by name before it. */
static void probe_after_exec(struct target *t, int fds[NR_PROBES],
			     enum result res[NR_PROBES])
{
	int i;

	if (exec_target(t)) {
		for (i = 0; i < NR_PROBES; i++)
			res[i] = FAILED;
		return;
	}
	for (i = 0; i < NR_PROBES; i++)
		res[i] = i == PROBE_KSM_SELF ? t->ksm_self : probe(i, fds[i]);
}

/*
 * Open the probes as TEST_UID or as root, let the target execute @bin and use
 * them as TEST_UID or as root.
 */
static int run(const char *bin, unsigned int flags, enum result res[NR_PROBES])
{
	int fds[NR_PROBES], i, ret = -1, status, pipefd[2];
	struct target t;
	pid_t child;

	if (start_target(&t, bin))
		return -1;

	if (flags & OPEN_AS_ROOT)
		open_probes(t.pid, fds);
	else if (open_probes_unprivileged(t.pid, fds))
		goto out;

	if (flags & READ_AS_ROOT) {
		probe_after_exec(&t, fds, res);
		ret = 0;
		goto out_close;
	}

	if (pipe2(pipefd, O_CLOEXEC))
		goto out_close;
	child = fork();
	if (!child) {
		if (drop_privileges())
			_exit(1);
		probe_after_exec(&t, fds, res);
		_exit(write(pipefd[1], res, RES_SIZE) != RES_SIZE);
	}
	close(pipefd[1]);
	if (child > 0) {
		bool got = read(pipefd[0], res, RES_SIZE) == RES_SIZE;

		if (waitpid(child, &status, 0) == child && !status && got)
			ret = 0;
	}
	close(pipefd[0]);
out_close:
	for (i = 0; i < NR_PROBES; i++)
		if (fds[i] >= 0)
			close(fds[i]);
out:
	stop_target(&t);
	return ret;
}

FIXTURE(setuid_exec) {};

FIXTURE_SETUP(setuid_exec)
{
	if (geteuid())
		SKIP(return, "must run as root");
	ASSERT_EQ(unshare(CLONE_NEWNS), 0);
	ASSERT_EQ(mount(NULL, "/", NULL, MS_REC | MS_PRIVATE, NULL), 0);
	/* BIN_DIR may well be nosuid. */
	ASSERT_EQ(mount("tmpfs", BIN_DIR, "tmpfs", 0, "mode=0755"), 0);
	ASSERT_EQ(copy_self(SUID_BIN, 04755), 0);
	ASSERT_EQ(copy_self(PLAIN_BIN, 0755), 0);
}

FIXTURE_TEARDOWN(setuid_exec)
{
}

/* KSM may not be built in. */
#define EXPECT_PROBE(res, p, want)					\
	do {								\
		if (((p) == PROBE_KSM || (p) == PROBE_KSM_SELF) &&	\
		    (res)[p] == MISSING)				\
			break;						\
		EXPECT_EQ((res)[p], want) {				\
			TH_LOG("%s", probe_name[p]);			\
		}							\
	} while (0)

TEST_F(setuid_exec, unprivileged_opener)
{
	enum result res[NR_PROBES];
	int i;

	ASSERT_EQ(run(SUID_BIN, 0, res), 0);
	for (i = 0; i < NR_PROBES; i++)
		EXPECT_PROBE(res, i, DENIED);
}

TEST_F(setuid_exec, privileged_reader)
{
	enum result res[NR_PROBES];
	int i;

	ASSERT_EQ(run(SUID_BIN, READ_AS_ROOT, res), 0);
	/* Root may look up fd/ itself. */
	for (i = PROBE_FDINFO; i < NR_PROBES; i++)
		EXPECT_PROBE(res, i, DENIED);
}

TEST_F(setuid_exec, privileged_opener)
{
	enum result res[NR_PROBES];
	int i;

	ASSERT_EQ(run(SUID_BIN, OPEN_AS_ROOT, res), 0);
	/* Lookups go by the caller, SELF_KSM_FD was opened by the target. */
	EXPECT_PROBE(res, PROBE_FD_LOOKUP, DENIED);
	EXPECT_PROBE(res, PROBE_KSM_SELF, DENIED);
	for (i = PROBE_FDINFO; i < PROBE_KSM_SELF; i++)
		EXPECT_PROBE(res, i, WORKS);
}

TEST_F(setuid_exec, no_setuid)
{
	enum result res[NR_PROBES];
	int i;

	ASSERT_EQ(run(PLAIN_BIN, 0, res), 0);
	for (i = 0; i < NR_PROBES; i++)
		EXPECT_PROBE(res, i, WORKS);
}

struct getdents_args {
	int fd;
	char *buf;
	size_t len;
	ssize_t ret;
};

static void *getdents_thread(void *arg)
{
	struct getdents_args *g = arg;

	g->ret = syscall(SYS_getdents64, g->fd, g->buf, g->len);
	return NULL;
}

/*
 * getdents() of fd/ sleeps in its first copy to userspace while the target
 * executes the set-user-ID program. It must not go on to list the new
 * program's descriptors.
 */
static int getdents_overtaken(struct target *t)
{
	struct uffdio_api api = { .api = UFFD_API };
	struct uffdio_register reg = {};
	struct uffdio_zeropage zero = {};
	long psz = sysconf(_SC_PAGESIZE);
	struct getdents_args g = {};
	struct linux_dirent64 *d;
	struct uffd_msg msg;
	struct pollfd pfd;
	char path[64], new_fd[16];
	pthread_t thread;
	int uffd;

	/* Created with privileges, it catches the kernel's faults. */
	uffd = syscall(SYS_userfaultfd, O_CLOEXEC);
	if (uffd < 0)
		return KSFT_SKIP;
	g.buf = mmap(NULL, psz, PROT_READ | PROT_WRITE,
		     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
	if (g.buf == MAP_FAILED)
		return KSFT_FAIL;
	reg.range.start = (unsigned long)g.buf;
	reg.range.len = psz;
	reg.mode = UFFDIO_REGISTER_MODE_MISSING;
	if (ioctl(uffd, UFFDIO_API, &api) || ioctl(uffd, UFFDIO_REGISTER, &reg))
		return KSFT_FAIL;

	if (drop_privileges())
		return KSFT_FAIL;
	snprintf(path, sizeof(path), "/proc/%d/fd", t->pid);
	g.fd = open(path, O_RDONLY | O_DIRECTORY | O_CLOEXEC);
	g.len = psz;
	if (g.fd < 0 || pthread_create(&thread, NULL, getdents_thread, &g))
		return KSFT_FAIL;

	pfd = (struct pollfd){ .fd = uffd, .events = POLLIN };
	if (poll(&pfd, 1, -1) != 1 || read(uffd, &msg, sizeof(msg)) != sizeof(msg))
		return KSFT_FAIL;
	if (exec_target(t))
		return KSFT_FAIL;
	zero.range = reg.range;
	if (ioctl(uffd, UFFDIO_ZEROPAGE, &zero) || pthread_join(thread, NULL))
		return KSFT_FAIL;

	snprintf(new_fd, sizeof(new_fd), "%d", NEW_FD);
	for (long off = 0; off < g.ret; off += d->d_reclen) {
		d = (struct linux_dirent64 *)(g.buf + off);
		if (!strcmp(d->d_name, new_fd))
			return LISTED_NEW_FD;
	}
	return KSFT_PASS;
}

TEST_F(setuid_exec, getdents_overtaken)
{
	struct target t;
	int status;
	pid_t child;

	ASSERT_EQ(start_target(&t, SUID_BIN), 0);
	child = fork();
	ASSERT_GE(child, 0);
	if (!child)
		_exit(getdents_overtaken(&t));
	ASSERT_EQ(waitpid(child, &status, 0), child);
	stop_target(&t);
	ASSERT_TRUE(WIFEXITED(status));
	if (WEXITSTATUS(status) == KSFT_SKIP)
		SKIP(return, "no userfaultfd");
	ASSERT_NE(WEXITSTATUS(status), KSFT_FAIL);
	EXPECT_EQ(WEXITSTATUS(status), KSFT_PASS) {
		TH_LOG("getdents() listed a descriptor of the new program");
	}
}

int main(int argc, char **argv)
{
	if (argc == 2 && !strcmp(argv[1], HELPER_ARG))
		return helper();
	return test_harness_run(argc, argv);
}
