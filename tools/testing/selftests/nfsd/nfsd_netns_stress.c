// SPDX-License-Identifier: GPL-2.0
/*
 * Concurrency soak for the NFSD control plane across network namespaces.
 *
 * NFSD's control plane is serialized by three locks, nested in this order:
 *
 *	nn->nfsd_mutex  ->  nfsd_global_mutex  ->  nfsd_file_cache_mutex
 *
 * Most of what used to be one module-wide mutex is now per-namespace, so
 * namespaces run their control planes in parallel and only meet on the
 * host-wide bits: the refcount that brings the open file cache and the
 * NFSv4 global tables up and down, and the address-notifier registration.
 *
 * This test exists to drive every one of those edges at once from many
 * namespaces. It asserts little by itself -- the point is to give lockdep
 * something to work with, so it is close to worthless without
 * CONFIG_PROVE_LOCKING=y. What it does check is that the kernel did not
 * warn: the taint word is sampled before and after, and a newly set
 * TAINT_WARN fails the run. That catches lockdep splats, WARN_ON()s and
 * refcount saturation alike.
 *
 * Each worker drives these, which between them cover every edge:
 *
 *   start a server	nn -> global (nfsd_users 0->1, notifiers 0->1)
 *			   -> cache (nfsd_file_cache_init)
 *   stop a server	nn -> global (1->0) -> cache (cache_shutdown)
 *   empty LISTENER_SET	creates and destroys a serv in one call, so the
 *			   host-wide refcount goes 0->1->0 on its own
 *   nfsd.fh/flush	the cache lock with nothing else held
 *   read filecache	likewise
 *   read pool_stats	nn, reached through svc_info.mutex
 *
 * Random churn alone is a weak race finder, so the run ends with
 * synchronized rounds where every worker attempts the host-wide 0->1
 * transition at the same instant. That is the window that only opened up
 * once the per-namespace lock stopped serializing namespaces against each
 * other.
 *
 * Tunable with NFSD_STRESS_WORKERS and NFSD_STRESS_SECS.
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
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/mount.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <net/if.h>
#include <netinet/in.h>

#include "../kselftest_harness.h"
#include "nfsd_netlink.h"

#define NFSD_MNT		"/mnt/nfsd"
#define EXPKEY_FLUSH		"/proc/net/rpc/nfsd.fh/flush"

/*
 * Every worker binds the same port. They are in different namespaces, so
 * that has to work; if isolation ever breaks, the second bind fails loudly
 * rather than the test quietly passing.
 */
#define STRESS_PORT		20049

#define DEFAULT_WORKERS		8
#define DEFAULT_SECS		5
#define MAX_WORKERS		64
#define SYNC_ROUNDS		20
#define BARRIER_TIMEOUT_SEC	30

/*
 * Generous: the harness caps a test at TEST_TIMEOUT_DEFAULT otherwise, and
 * the churn duration is tunable.
 */
#define SOAK_TIMEOUT_SEC	600

#define TAINT_WARN_BIT		(1UL << 9)

/* Worker exit codes. */
#define WORKER_OK		0
#define WORKER_FAIL		1
#define WORKER_NO_SETUP		2

/* ------------------- cross-process barrier ------------------- */

struct barrier {
	unsigned int n;
	unsigned int count;
	unsigned int generation;
	unsigned int aborted;	/* latched: a rendezvous was never completed */
};

/*
 * Spin with a deadline rather than blocking, so a worker that dies cannot
 * wedge the run.
 *
 * The first waiter to give up latches ->aborted, which kills the barrier
 * for good: every later call returns at once instead of waiting out its
 * own deadline. Without that latch a single missing worker costs
 * BARRIER_TIMEOUT_SEC on every remaining rendezvous -- three per round,
 * SYNC_ROUNDS rounds -- which runs into tens of minutes before the
 * harness timeout fires.
 *
 * A giving-up waiter leaves its ->count increment behind. That is fine:
 * once ->aborted is set nothing reads ->count again.
 *
 * Returns false if the rendezvous did not happen.
 */
static bool barrier_wait(struct barrier *b)
{
	unsigned int gen = __atomic_load_n(&b->generation, __ATOMIC_ACQUIRE);
	time_t deadline = time(NULL) + BARRIER_TIMEOUT_SEC;

	if (__atomic_load_n(&b->aborted, __ATOMIC_ACQUIRE))
		return false;

	if (__atomic_add_fetch(&b->count, 1, __ATOMIC_ACQ_REL) == b->n) {
		__atomic_store_n(&b->count, 0, __ATOMIC_RELEASE);
		__atomic_add_fetch(&b->generation, 1, __ATOMIC_ACQ_REL);
		return true;
	}
	while (__atomic_load_n(&b->generation, __ATOMIC_ACQUIRE) == gen) {
		if (__atomic_load_n(&b->aborted, __ATOMIC_ACQUIRE))
			return false;
		if (time(NULL) > deadline) {
			__atomic_store_n(&b->aborted, 1, __ATOMIC_RELEASE);
			return false;
		}
		sched_yield();
	}
	return true;
}

/* ------------------- namespace setup ------------------- */

static int netns_enter(void)
{
	struct ifreq ifr = {0};
	int s;

	if (unshare(CLONE_NEWNET | CLONE_NEWNS) < 0)
		return -errno;
	if (mount("", "/", NULL, MS_REC | MS_PRIVATE, NULL) < 0)
		return -errno;
	if (mount("tmpfs", "/mnt", "tmpfs", 0, NULL) < 0)
		return -errno;
	if (mkdir(NFSD_MNT, 0755) < 0)
		return -errno;
	if (mount("nfsd", NFSD_MNT, "nfsd", 0, NULL) < 0)
		return -errno;

	s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s < 0)
		return -errno;
	strcpy(ifr.ifr_name, "lo");
	if (ioctl(s, SIOCGIFFLAGS, &ifr) < 0) {
		close(s);
		return -errno;
	}
	ifr.ifr_flags |= IFF_UP | IFF_RUNNING;
	if (ioctl(s, SIOCSIFFLAGS, &ifr) < 0) {
		close(s);
		return -errno;
	}
	close(s);

	nfsd_family = genl_resolve_nfsd();
	if (nfsd_family < 0)
		return -ENOENT;
	return 0;
}

/* ------------------- the operations ------------------- */

static int read_nfsd_file(const char *name)
{
	char path[128], buf[4096];
	int fd, n;

	snprintf(path, sizeof(path), "%s/%s", NFSD_MNT, name);
	fd = open(path, O_RDONLY);
	if (fd < 0)
		return -errno;
	do {
		n = read(fd, buf, sizeof(buf));
	} while (n > 0);
	close(fd);
	return n < 0 ? -errno : 0;
}

/*
 * The only userspace trigger that reaches nfsd_file_cache_purge(), and so
 * the only one that takes the file cache lock with no other nfsd lock
 * held. NFSD_CMD_CACHE_FLUSH does not get there: cache_purge() never calls
 * the cache_detail's ->flush hook.
 */
static int expkey_flush(void)
{
	int fd = open(EXPKEY_FLUSH, O_WRONLY);
	int n;

	if (fd < 0)
		return -errno;
	n = write(fd, "1\n", 2);
	close(fd);
	return n < 0 ? -errno : 0;
}

/* Create a serv and tear it straight back down inside one call. */
static int serv_cycle(void)
{
	return listener_set(NULL, 0);
}

static int server_up(int nthreads)
{
	char attrs[128];
	int off, ret;

	off = put_listener(attrs, 0, "tcp", STRESS_PORT);
	off = put_attr(attrs, off, NFSD_A_SERVER_SOCK_USERSPACE_RPCBIND,
		       NULL, 0);
	ret = listener_set(attrs, off);
	if (ret)
		return ret;
	return threads_set(nthreads);
}

/* threads_set(0) drops the last thread, which destroys the serv with it. */
static int server_down(void)
{
	return threads_set(0);
}

/* ------------------- the worker ------------------- */

struct worker_err {
	const char *op;
	int err;
};

static struct worker_err worker_fault;

static bool fail(const char *op, int err)
{
	if (err == 0)
		return false;
	worker_fault.op = op;
	worker_fault.err = err;
	return true;
}

/*
 * Random churn between "no serv" and "serv with threads", with the
 * lock-crossing reads and flushes mixed in at both ends. Every call here
 * is one that must succeed in the state it is issued from, so any error is
 * a real failure rather than an expected race.
 */
static bool worker_churn(time_t deadline)
{
	bool up = false;

	while (time(NULL) < deadline) {
		int r = random() % 8;

		if (!up) {
			switch (r) {
			case 0:
			case 1:
				if (fail("serv_cycle", serv_cycle()))
					return false;
				break;
			case 2:
				if (fail("version_set", version_set_only(4, 1)))
					return false;
				break;
			case 3:
				if (fail("flush", expkey_flush()))
					return false;
				break;
			case 4:
				if (fail("filecache", read_nfsd_file("filecache")))
					return false;
				break;
			case 5:
				if (fail("pool_stats", read_nfsd_file("pool_stats")))
					return false;
				break;
			default:
				if (fail("server_up", server_up(1 + random() % 3)))
					return false;
				up = true;
				break;
			}
		} else {
			switch (r) {
			case 0:
				if (fail("flush", expkey_flush()))
					return false;
				break;
			case 1:
				if (fail("filecache", read_nfsd_file("filecache")))
					return false;
				break;
			case 2:
				if (fail("pool_stats", read_nfsd_file("pool_stats")))
					return false;
				break;
			case 3:
				if (fail("threads_set", threads_set(1 + random() % 3)))
					return false;
				break;
			default:
				if (fail("server_down", server_down()))
					return false;
				up = false;
				break;
			}
		}
	}

	if (up && fail("server_down", server_down()))
		return false;
	return true;
}

/*
 * Every worker arrives at the barrier with no serv, so the host-wide
 * refcount is at zero, then they all try to take it to one together. The
 * second barrier lines up the drop back to zero the same way.
 */
static bool worker_sync_rounds(struct barrier *b)
{
	int i;

	for (i = 0; i < SYNC_ROUNDS; i++) {
		/*
		 * A failed rendezvous means a peer is gone, which its own
		 * exit status already reports. Stop the rounds rather than
		 * stalling on every remaining barrier.
		 */
		if (!barrier_wait(b))
			return true;
		if (fail("sync server_up", server_up(1)))
			return false;

		if (!barrier_wait(b))
			return true;
		if (fail("sync flush", expkey_flush()))
			return false;

		if (!barrier_wait(b))
			return true;
		if (fail("sync server_down", server_down()))
			return false;
	}
	return true;
}

static int worker(int idx, struct barrier *b, unsigned int secs)
{
	int ret = netns_enter();

	if (ret) {
		/*
		 * Report setup trouble rather than a failure: a restricted
		 * environment is not a kernel bug.
		 */
		fprintf(stderr, "netns %d: setup: %s\n", idx, strerror(-ret));
		return WORKER_NO_SETUP;
	}

	/*
	 * Leave only NFSv4.1 enabled. A fresh namespace has v3 on, which
	 * makes nfsd_needs_lockd() true, and the lockd that comes up then
	 * tries to reach an rpcbind that is not there -- so every server
	 * start waits out an RPC timeout and floods the log. The version
	 * set sticks across serv teardown, so once is enough.
	 */
	ret = version_set_only(4, 1);
	if (ret) {
		fprintf(stderr, "netns %d: version_set: %s\n", idx,
			strerror(-ret));
		return WORKER_FAIL;
	}

	srandom(getpid() ^ (unsigned int)time(NULL));

	if (!worker_churn(time(NULL) + secs))
		goto fault;
	if (!worker_sync_rounds(b))
		goto fault;
	return WORKER_OK;

fault:
	fprintf(stderr, "netns %d: %s: %s\n", idx, worker_fault.op,
		strerror(-worker_fault.err));
	server_down();
	return WORKER_FAIL;
}

/*
 * Threads running outside the test's namespaces. Any at all pin the
 * host-wide refcount above zero for the whole run, so the 0->1 transition
 * the synchronized rounds are built around never happens.
 */
static int nfsd_threads_here(void)
{
	char buf[32] = "";
	int fd = open("/proc/fs/nfsd/threads", O_RDONLY);
	int n = 0;

	if (fd < 0)
		return 0;
	if (read(fd, buf, sizeof(buf) - 1) > 0)
		n = atoi(buf);
	close(fd);
	return n;
}

/* ------------------- taint ------------------- */

static unsigned long read_taint(void)
{
	unsigned long v = 0;
	FILE *f = fopen("/proc/sys/kernel/tainted", "r");

	if (!f)
		return 0;
	if (fscanf(f, "%lu", &v) != 1)
		v = 0;
	fclose(f);
	return v;
}

/* ------------------- the test ------------------- */

static unsigned int env_uint(const char *name, unsigned int def, unsigned int max)
{
	const char *s = getenv(name);
	unsigned long v;

	if (!s || !*s)
		return def;
	v = strtoul(s, NULL, 0);
	if (v == 0 || v > max)
		return def;
	return (unsigned int)v;
}

FIXTURE(soak) {
	int unused;
};

FIXTURE_SETUP(soak) { }
FIXTURE_TEARDOWN(soak) { }

TEST_F_TIMEOUT(soak, netns_control_plane_soak, SOAK_TIMEOUT_SEC)
{
	unsigned long taint_before, taint_after;
	unsigned int workers, secs;
	pid_t pid[MAX_WORKERS];
	int failed = 0, skipped = 0;
	unsigned int aborted;
	struct barrier *b;
	unsigned int i;
	long ncpu;

	if (geteuid() != 0)
		SKIP(return, "must be run as root");

	ncpu = sysconf(_SC_NPROCESSORS_ONLN);
	if (ncpu < 2)
		ncpu = 2;
	workers = env_uint("NFSD_STRESS_WORKERS",
			   ncpu < DEFAULT_WORKERS ? (unsigned int)ncpu
						  : DEFAULT_WORKERS,
			   MAX_WORKERS);
	secs = env_uint("NFSD_STRESS_SECS", DEFAULT_SECS, 3600);

	b = mmap(NULL, sizeof(*b), PROT_READ | PROT_WRITE,
		 MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	ASSERT_NE(MAP_FAILED, b);
	b->n = workers;
	b->count = 0;
	b->generation = 0;
	b->aborted = 0;

	TH_LOG("%u namespaces, %us of churn then %d synchronized rounds",
	       workers, secs, SYNC_ROUNDS);

	/*
	 * A server already running outside these namespaces holds the
	 * host-wide refcount above zero for the whole run, so the 0->1
	 * transition the synchronized rounds are built around never
	 * happens. Worth saying out loud rather than reporting coverage
	 * that was not there.
	 */
	if (nfsd_threads_here() > 0)
		TH_LOG("note: nfsd runs outside these namespaces; refcount never hits 0");

	taint_before = read_taint();

	for (i = 0; i < workers; i++) {
		pid[i] = fork();
		ASSERT_GE(pid[i], 0);
		if (pid[i] == 0)
			_exit(worker(i, b, secs));
	}

	for (i = 0; i < workers; i++) {
		int status = 0;

		waitpid(pid[i], &status, 0);
		if (!WIFEXITED(status)) {
			failed++;
			continue;
		}
		if (WEXITSTATUS(status) == WORKER_FAIL)
			failed++;
		else if (WEXITSTATUS(status) == WORKER_NO_SETUP)
			skipped++;
	}

	taint_after = read_taint();
	aborted = __atomic_load_n(&b->aborted, __ATOMIC_ACQUIRE);
	munmap(b, sizeof(*b));

	if (skipped == (int)workers)
		SKIP(return, "no worker could set up a private nfsd namespace");

	EXPECT_EQ(0, failed)
		TH_LOG("%d of %u namespaces reported an error", failed, workers);

	/*
	 * A namespace that never reached the rounds -- one that could not
	 * set up, say -- leaves the rest with nobody to meet. Say so: the
	 * synchronized part did not run to completion.
	 */
	if (aborted)
		TH_LOG("note: synchronized rounds cut short; a namespace did not arrive");

	/*
	 * The real result. Without CONFIG_PROVE_LOCKING there is very little
	 * here for the kernel to complain about, so say so rather than
	 * letting a quiet pass look like coverage.
	 */
	EXPECT_EQ(0, (taint_after & ~taint_before) & TAINT_WARN_BIT)
		TH_LOG("kernel warned during the run (taint %#lx -> %#lx); check dmesg",
		       taint_before, taint_after);

	/*
	 * lockdep_proc_init() puts these in procfs, not debugfs, and
	 * lockdep_chains is the one that appears only with
	 * CONFIG_PROVE_LOCKING -- the part that validates ordering rather
	 * than merely tracking. Root-only, but so is this test.
	 */
	if (access("/proc/lockdep_chains", R_OK) != 0)
		TH_LOG("note: CONFIG_PROVE_LOCKING looks absent; this proves little");
}

TEST_HARNESS_MAIN
