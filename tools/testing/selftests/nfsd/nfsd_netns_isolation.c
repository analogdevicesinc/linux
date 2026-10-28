// SPDX-License-Identifier: GPL-2.0
/*
 * Namespace-isolation tests for the NFSD control plane.
 *
 * NFSD's per-namespace settings used to sit behind one module-wide mutex,
 * and one of them -- the maximum READ/WRITE payload -- was a module-wide
 * variable reachable through a per-netns file. These tests pin down the
 * boundary: what one namespace does to its own server must not be visible
 * to, or block, another.
 *
 * Every namespace here gets a private net + mount namespace, a tmpfs on
 * /mnt so nothing escapes, and its own nfsd filesystem mounted on
 * /mnt/nfsd. The module creates /proc/fs/nfs, not /proc/fs/nfsd, so the
 * mount has to be made by hand; it is also what ties a running server to
 * this test, since nfsd_umount() stops the threads.
 *
 * Servers are created with NFSD_A_SERVER_SOCK_USERSPACE_RPCBIND so nothing
 * ever calls out to rpcbind, and restricted to NFSv4.1 so that
 * nfsd_needs_lockd() stays false and no lockd instance is started.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mount.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <net/if.h>
#include <netinet/in.h>

#include "../kselftest_harness.h"
#include "nfsd_netlink.h"

#define NFSD_MNT		"/mnt/nfsd"
#define TEST_PORT		20049

/* netns_enter() could not build a usable namespace; not a test failure. */
#define NETNS_NO_SETUP		INT_MIN

/*
 * Build a private net + mount namespace with an nfsd filesystem on
 * /mnt/nfsd and loopback up. Returns 0, or NETNS_NO_SETUP when the
 * environment will not allow it.
 */
static int netns_enter(void)
{
	struct ifreq ifr = {0};
	int s;

	if (unshare(CLONE_NEWNET | CLONE_NEWNS) < 0)
		return NETNS_NO_SETUP;
	if (mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL) < 0)
		return NETNS_NO_SETUP;

	/*
	 * Everything below is created inside this mount namespace only, so
	 * the mkdir cannot leave anything behind on the host.
	 */
	if (mount("tmpfs", "/mnt", "tmpfs", 0, NULL) < 0)
		return NETNS_NO_SETUP;
	if (mkdir(NFSD_MNT, 0755) < 0)
		return NETNS_NO_SETUP;
	if (mount("nfsd", NFSD_MNT, "nfsd", 0, NULL) < 0)
		return NETNS_NO_SETUP;

	s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s < 0)
		return NETNS_NO_SETUP;
	strcpy(ifr.ifr_name, "lo");
	if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0) {
		close(s);
		return NETNS_NO_SETUP;
	}
	ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
	if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0) {
		close(s);
		return NETNS_NO_SETUP;
	}
	close(s);

	nfsd_family = genl_resolve_nfsd();
	if (nfsd_family < 0)
		return NETNS_NO_SETUP;
	return 0;
}

/* ------------------- nfsdfs file access ------------------- */

static int nfsd_file_read(const char *name, char *buf, size_t len)
{
	char path[128];
	int fd, n;

	snprintf(path, sizeof(path), "%s/%s", NFSD_MNT, name);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -errno;
	n = read(fd, buf, len - 1);
	close(fd);
	if (n < 0)
		return -errno;
	buf[n] = '\0';
	return n;
}

static int nfsd_file_read_int(const char *name)
{
	char buf[64];
	int n = nfsd_file_read(name, buf, sizeof(buf));

	if (n < 0)
		return n;
	return atoi(buf);
}

static int nfsd_file_write_int(const char *name, int val)
{
	char path[128], buf[64];
	int fd, n, len;

	snprintf(path, sizeof(path), "%s/%s", NFSD_MNT, name);
	len = snprintf(buf, sizeof(buf), "%d\n", val);
	fd = open(path, O_WRONLY);
	if (fd < 0)
		return -errno;
	n = write(fd, buf, len);
	close(fd);
	return n < 0 ? -errno : 0;
}

/* ------------------- server lifecycle ------------------- */

/*
 * Bring up a v4.1-only server on a loopback listener, owning rpcbind
 * registration in userspace so the kernel never issues an rpcbind call.
 */
static int server_start(uint16_t port, int nthreads)
{
	char attrs[128];
	int off;
	int ret;

	ret = version_set_only(4, 1);
	if (ret)
		return ret;

	off = put_listener(attrs, 0, "tcp", port);
	off = put_attr(attrs, off, NFSD_A_SERVER_SOCK_USERSPACE_RPCBIND,
		       NULL, 0);
	ret = listener_set(attrs, off);
	if (ret)
		return ret;

	return threads_set(nthreads);
}

static void server_stop(void)
{
	threads_set(0);
	listener_set(NULL, 0);
}

/* ------------------- run a callback in a fresh namespace ------------------- */

/*
 * Fork a child into its own namespace, run @fn there and hand back what it
 * returned. Used for the checks that only need one namespace at a time.
 */
static int netns_run(int (*fn)(long), long arg)
{
	int p[2], ret = -EIO;
	pid_t pid;

	if (pipe(p) < 0)
		return -errno;

	pid = fork();
	if (pid < 0) {
		close(p[0]);
		close(p[1]);
		return -errno;
	}
	if (pid == 0) {
		int r = netns_enter();

		if (r == 0)
			r = fn(arg);
		if (write(p[1], &r, sizeof(r)) != sizeof(r))
			_exit(1);
		_exit(0);
	}

	close(p[1]);
	if (read(p[0], &ret, sizeof(ret)) != sizeof(ret))
		ret = -EIO;
	close(p[0]);
	waitpid(pid, NULL, 0);
	return ret;
}

/* ------------------- a peer namespace held open ------------------- */

/*
 * A second namespace running a server for as long as the test needs it.
 * The peer reports readiness on a pipe and waits for a byte before tearing
 * down, so the test can be sure the server is up while it pokes its own
 * namespace.
 */
struct peer {
	pid_t pid;
	int wake;		/* write here to let the peer exit */
	int ready;		/* peer writes its status here */
};

static int peer_start(struct peer *pr, uint16_t port)
{
	int wake[2], ready[2];
	pid_t ppid = getpid();
	char status;

	if (pipe(wake) < 0)
		return -errno;
	if (pipe(ready) < 0) {
		close(wake[0]);
		close(wake[1]);
		return -errno;
	}

	/* peer_stop() handles a dead peer itself; do not die of SIGPIPE first. */
	signal(SIGPIPE, SIG_IGN);

	pr->pid = fork();
	if (pr->pid < 0) {
		close(wake[0]); close(wake[1]);
		close(ready[0]); close(ready[1]);
		return -errno;
	}
	if (pr->pid == 0) {
		char c;
		int r;

		/* Hold no writer of our own, so read() below sees EOF. */
		close(wake[1]);
		close(ready[0]);
		/*
		 * A parent that dies without reaching peer_stop() would leave
		 * this namespace and its server pinned by a process blocked
		 * forever in read().
		 */
		prctl(PR_SET_PDEATHSIG, SIGKILL);
		if (getppid() != ppid)
			_exit(1);

		r = netns_enter();
		if (r == 0)
			r = server_start(port, 1);
		status = r == NETNS_NO_SETUP ? 'S' : (r ? 'E' : 'R');
		if (write(ready[1], &status, 1) != 1)
			_exit(1);
		/* Hold the namespace open until the test is done with it. */
		if (read(wake[0], &c, 1) == 1 && status == 'R')
			server_stop();
		_exit(0);
	}

	close(wake[0]);
	close(ready[1]);
	pr->wake = wake[1];
	pr->ready = ready[0];

	if (read(pr->ready, &status, 1) != 1)
		status = 'E';
	if (status == 'S')
		return NETNS_NO_SETUP;
	return status == 'R' ? 0 : -EIO;
}

static void peer_stop(struct peer *pr)
{
	char c = 'x';

	if (pr->pid <= 0)
		return;
	if (write(pr->wake, &c, 1) != 1)
		kill(pr->pid, SIGKILL);
	close(pr->wake);
	close(pr->ready);
	waitpid(pr->pid, NULL, 0);
	pr->pid = 0;
}

/* ===================== max_block_size is per-namespace ===================== */

static int read_max_blksize(long unused)
{
	(void)unused;
	return nfsd_file_read_int("max_block_size");
}

static int write_max_blksize(long val)
{
	int ret = nfsd_file_write_int("max_block_size", (int)val);

	if (ret)
		return ret;
	/* Report what stuck, so the caller knows the write was accepted. */
	return nfsd_file_read_int("max_block_size");
}

/*
 * max_block_size is reachable only through a per-netns nfsd filesystem, so
 * a write in one namespace must not be visible in another. Read it in a
 * throwaway namespace, change it in a second, then read it again in a
 * third: the two reads have to agree.
 *
 * No value is hardcoded. The default is derived from the size of memory,
 * so the test picks a target that differs from whatever this machine uses.
 */
TEST(max_blksize_is_per_netns)
{
	int before, after, wrote, target;

	before = netns_run(read_max_blksize, 0);
	if (before == NETNS_NO_SETUP)
		SKIP(return, "cannot set up a private nfsd namespace");
	ASSERT_GE(before, 0);

	/* Any legal value that is not the one this machine already reports. */
	target = (before == 262144) ? 131072 : 262144;

	wrote = netns_run(write_max_blksize, target);
	ASSERT_EQ(target, wrote)
		TH_LOG("second namespace did not accept max_block_size=%d",
		       target);

	after = netns_run(read_max_blksize, 0);
	ASSERT_GE(after, 0);
	EXPECT_EQ(before, after)
		TH_LOG("max_block_size leaked between namespaces: %d -> %d",
		       before, after);
}

/* ===================== a busy namespace does not busy others ===================== */

FIXTURE(nfsd_peer) {
	struct peer pr;
};

FIXTURE_SETUP(nfsd_peer)
{
	int ret;

	if (geteuid() != 0)
		SKIP(return, "must be run as root");

	/* The peer namespace comes first; it must not be ours. */
	memset(&self->pr, 0, sizeof(self->pr));
	ret = peer_start(&self->pr, TEST_PORT);
	if (ret == NETNS_NO_SETUP)
		SKIP(return, "cannot set up a private nfsd namespace");
	if (ret)
		SKIP(return, "peer namespace could not start a server: %d", ret);

	/* Now put this process in a namespace of its own, with no server. */
	if (netns_enter() != 0)
		SKIP(return, "cannot set up a private nfsd namespace");
}

FIXTURE_TEARDOWN(nfsd_peer)
{
	if (nfsd_family >= 0)
		server_stop();
	peer_stop(&self->pr);
}

/*
 * write_maxblksize() refuses with -EBUSY while that namespace has a serv.
 * The check is on nn->nfsd_serv, so a server belonging to someone else must
 * not trip it.
 */
TEST_F(nfsd_peer, maxblksize_busy_is_per_netns)
{
	int cur = nfsd_file_read_int("max_block_size");

	ASSERT_GE(cur, 0);
	EXPECT_EQ(0, nfsd_file_write_int("max_block_size",
					 cur == 262144 ? 131072 : 262144))
		TH_LOG("max_block_size refused while another netns has a server");
}

/*
 * NFSD_CMD_VERSION_SET returns -EBUSY once the namespace has a serv. A
 * server in the peer namespace must not reach us.
 */
TEST_F(nfsd_peer, version_set_busy_is_per_netns)
{
	EXPECT_EQ(0, version_set_only(4, 1))
		TH_LOG("VERSION_SET refused while another netns has a server");
}

/*
 * The grace and lease times are per-namespace too, and gated on the same
 * nn->nfsd_serv check.
 */
TEST_F(nfsd_peer, leasetime_busy_is_per_netns)
{
	EXPECT_EQ(0, nfsd_file_write_int("nfsv4leasetime", 60))
		TH_LOG("nfsv4leasetime refused while another netns has a server");
}

/*
 * pool_stats runs its seq_file under the mutex that also guards the serv.
 * Reading it here must not be affected by the peer's server, and must
 * report this namespace, which has no threads at all.
 */
TEST_F(nfsd_peer, pool_stats_readable_with_foreign_server)
{
	char buf[4096];
	int n = nfsd_file_read("pool_stats", buf, sizeof(buf));

	ASSERT_GE(n, 0)
		TH_LOG("pool_stats unreadable: %s", strerror(-n));
	EXPECT_NE(NULL, strstr(buf, "packets-arrived"));
}

/*
 * LISTENER_GET reports this namespace's listeners. With a server running
 * next door and none here, the list must come back empty rather than
 * showing the peer's.
 */
TEST_F(nfsd_peer, listener_get_does_not_show_foreign_listeners)
{
	char rbuf[8192];
	int n = genl_request_reply(NFSD_CMD_LISTENER_GET, rbuf, sizeof(rbuf));

	ASSERT_GT(n, 0);
	EXPECT_EQ(NULL, memmem(rbuf, n, "tcp", 4))
		TH_LOG("LISTENER_GET leaked a listener from another netns");
}

/*
 * The file cache is one host-wide object, but the flush that reaches it is
 * driven from a per-netns file. Doing it here while the peer has a server
 * up must be harmless -- this is the path that takes the cache lock with
 * nothing else held.
 */
TEST_F(nfsd_peer, expkey_flush_with_foreign_server)
{
	int fd = open("/proc/net/rpc/nfsd.fh/flush", O_WRONLY);

	if (fd < 0)
		SKIP(return, "no /proc/net/rpc/nfsd.fh/flush: %s",
		     strerror(errno));
	EXPECT_EQ(2, write(fd, "1\n", 2));
	close(fd);
}

TEST_HARNESS_MAIN
