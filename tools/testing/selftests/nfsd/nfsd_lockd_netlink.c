// SPDX-License-Identifier: GPL-2.0
/*
 * Regression tests for lockd's generic-netlink configuration interface
 * (LOCKD_CMD_SERVER_SET / LOCKD_CMD_SERVER_GET).
 *
 * All three attributes are optional in the spec and each is applied on its
 * own by the kernel, but SERVER_SET used to demand a grace time and reject
 * anything else with -EINVAL. That made the tcp and udp ports unsettable by
 * themselves, which is exactly what nfsdctl asks for when /etc/nfs.conf has
 * a [lockd] port but no grace-time -- "nfsdctl autostart" then failed before
 * it had configured anything.
 *
 * Every test runs in a private network namespace. The settings are per-netns,
 * and a SERVER_SET in init_net would also overwrite the host's module-wide
 * nlm_grace_period/nlm_tcpport/nlm_udpport.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <linux/lockd_netlink.h>

#include "../kselftest_harness.h"
#include "nfsd_netlink.h"

/* fs/lockd/svc.c: nlm_grace_period_max */
#define GRACE_MAX		240

#define TEST_TCP_PORT		32531
#define TEST_UDP_PORT		32532

struct lockd_cfg {
	uint32_t gracetime;
	uint16_t tcp_port;
	uint16_t udp_port;
};

static int lockd_family = -1;

static int lockd_set(const char *attrs, int len)
{
	return genl_request_to(lockd_family, LOCKD_CMD_SERVER_SET, attrs, len);
}

static int lockd_get(struct lockd_cfg *cfg)
{
	const struct nlattr *grace, *tcp, *udp;
	char rbuf[4096];
	int n;

	n = genl_request_reply_attrs_to(lockd_family, LOCKD_CMD_SERVER_GET,
					NULL, 0, rbuf, sizeof(rbuf));
	if (n < 0)
		return n;

	grace = genl_find_attr(rbuf, n, LOCKD_A_SERVER_GRACETIME);
	tcp = genl_find_attr(rbuf, n, LOCKD_A_SERVER_TCP_PORT);
	udp = genl_find_attr(rbuf, n, LOCKD_A_SERVER_UDP_PORT);
	if (!grace || !tcp || !udp)
		return -ENOENT;

	cfg->gracetime = nla_u32(grace);
	cfg->tcp_port = nla_u16(tcp);
	cfg->udp_port = nla_u16(udp);
	return 0;
}

/* ------------------- SERVER_SET request builders ------------------- */

static int put_gracetime(char *buf, int off, uint32_t grace)
{
	return put_attr(buf, off, LOCKD_A_SERVER_GRACETIME, &grace,
			sizeof(grace));
}

static int put_tcp_port(char *buf, int off, uint16_t tcp)
{
	return put_attr(buf, off, LOCKD_A_SERVER_TCP_PORT, &tcp, sizeof(tcp));
}

static int put_udp_port(char *buf, int off, uint16_t udp)
{
	return put_attr(buf, off, LOCKD_A_SERVER_UDP_PORT, &udp, sizeof(udp));
}

static int put_ports(char *buf, int off, uint16_t tcp, uint16_t udp)
{
	off = put_tcp_port(buf, off, tcp);
	return put_udp_port(buf, off, udp);
}

FIXTURE(lockd_netlink) {
};

FIXTURE_SETUP(lockd_netlink)
{
	if (geteuid() != 0)
		SKIP(return, "must be run as root");
	if (unshare(CLONE_NEWNET) < 0)
		SKIP(return, "unshare(NEWNET): %s", strerror(errno));

	lockd_family = genl_resolve(LOCKD_FAMILY_NAME, sizeof(LOCKD_FAMILY_NAME));
	if (lockd_family < 0)
		SKIP(return, "lockd netlink family not registered");
}

FIXTURE_TEARDOWN(lockd_netlink)
{
	/* Nothing to undo: the settings die with the namespace. */
}

/* A fresh namespace starts out with everything at zero. */
TEST_F(lockd_netlink, defaults_are_zero)
{
	struct lockd_cfg cfg;

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(0, cfg.gracetime);
	EXPECT_EQ(0, cfg.tcp_port);
	EXPECT_EQ(0, cfg.udp_port);
}

/*
 * The regression: ports on their own, no grace time. This is the request
 * nfsdctl builds from a [lockd] section that only sets the ports.
 */
TEST_F(lockd_netlink, set_ports_without_gracetime)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_ports(attrs, 0, TEST_TCP_PORT, TEST_UDP_PORT);

	ASSERT_EQ(0, lockd_set(attrs, off))
		TH_LOG("SERVER_SET rejected a request with no gracetime");

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(TEST_TCP_PORT, cfg.tcp_port);
	EXPECT_EQ(TEST_UDP_PORT, cfg.udp_port);
	EXPECT_EQ(0, cfg.gracetime);
}

/* The mirror image: a grace time with no ports. */
TEST_F(lockd_netlink, set_gracetime_without_ports)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_gracetime(attrs, 0, 90);

	ASSERT_EQ(0, lockd_set(attrs, off));

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(90, cfg.gracetime);
	EXPECT_EQ(0, cfg.tcp_port);
	EXPECT_EQ(0, cfg.udp_port);
}

/* One attribute at a time leaves the others alone. */
TEST_F(lockd_netlink, attributes_are_set_independently)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_tcp_port(attrs, 0, TEST_TCP_PORT);
	ASSERT_EQ(0, lockd_set(attrs, off));

	off = put_udp_port(attrs, 0, TEST_UDP_PORT);
	ASSERT_EQ(0, lockd_set(attrs, off))
		TH_LOG("SERVER_SET rejected a UDP-only request");

	off = put_gracetime(attrs, 0, 30);
	ASSERT_EQ(0, lockd_set(attrs, off));

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(30, cfg.gracetime);
	EXPECT_EQ(TEST_TCP_PORT, cfg.tcp_port)
		TH_LOG("a later SET clobbered the tcp port");
	EXPECT_EQ(TEST_UDP_PORT, cfg.udp_port)
		TH_LOG("a gracetime-only SET clobbered the udp port");
}

TEST_F(lockd_netlink, set_all_three)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_gracetime(attrs, 0, GRACE_MAX);
	off = put_ports(attrs, off, TEST_TCP_PORT, TEST_UDP_PORT);

	ASSERT_EQ(0, lockd_set(attrs, off));

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(GRACE_MAX, cfg.gracetime);
	EXPECT_EQ(TEST_TCP_PORT, cfg.tcp_port);
	EXPECT_EQ(TEST_UDP_PORT, cfg.udp_port);
}

/* An empty SERVER_SET has nothing to do, but is not an error. */
TEST_F(lockd_netlink, empty_set_is_a_noop)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_gracetime(attrs, 0, 30);
	off = put_ports(attrs, off, TEST_TCP_PORT, TEST_UDP_PORT);
	ASSERT_EQ(0, lockd_set(attrs, off));

	ASSERT_EQ(0, lockd_set(NULL, 0));

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(30, cfg.gracetime);
	EXPECT_EQ(TEST_TCP_PORT, cfg.tcp_port);
	EXPECT_EQ(TEST_UDP_PORT, cfg.udp_port);
}

/*
 * The grace time is still range-checked, and the check runs before anything
 * is stored: a rejected request must not apply the ports it came with.
 */
TEST_F(lockd_netlink, gracetime_above_max_rejected)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_gracetime(attrs, 0, GRACE_MAX + 1);
	off = put_ports(attrs, off, TEST_TCP_PORT, TEST_UDP_PORT);

	EXPECT_EQ(-EINVAL, lockd_set(attrs, off));

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(0, cfg.gracetime);
	EXPECT_EQ(0, cfg.tcp_port)
		TH_LOG("a rejected SERVER_SET applied the tcp port anyway");
	EXPECT_EQ(0, cfg.udp_port);
}

/* Settings belong to the namespace that made them. */
TEST_F(lockd_netlink, settings_are_per_netns)
{
	struct lockd_cfg cfg;
	char attrs[64];
	int off;

	off = put_gracetime(attrs, 0, 30);
	off = put_ports(attrs, off, TEST_TCP_PORT, TEST_UDP_PORT);
	ASSERT_EQ(0, lockd_set(attrs, off));

	if (unshare(CLONE_NEWNET) < 0)
		SKIP(return, "second unshare(NEWNET): %s", strerror(errno));

	ASSERT_EQ(0, lockd_get(&cfg));
	EXPECT_EQ(0, cfg.gracetime)
		TH_LOG("grace time leaked out of the namespace that set it");
	EXPECT_EQ(0, cfg.tcp_port)
		TH_LOG("tcp port leaked out of the namespace that set it");
	EXPECT_EQ(0, cfg.udp_port);
}

TEST_HARNESS_MAIN
