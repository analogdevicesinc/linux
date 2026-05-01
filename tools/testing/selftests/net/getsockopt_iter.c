// SPDX-License-Identifier: GPL-2.0
/*
 * Quick test for getsockopt{_iter} tests.
 *
 * Each fixture targets one converted protocol and pins down the
 * returned-length / errno semantics across buffer-size variations,
 * an unknown optname and a bogus level.
 *
 * - netlink: NETLINK_PKTINFO covers the flag-style int path; the
 *   NETLINK_LIST_MEMBERSHIPS cases cover the size-discovery path
 *   that always reports the required buffer length back via optlen,
 *   even when the user buffer is too small to receive any group bits.
 * - vsock:   SO_VM_SOCKETS_BUFFER_SIZE covers the u64 path.
 *
 * Author: Breno Leitao <leitao@debian.org>
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <linux/netlink.h>
#include <linux/rtnetlink.h>
#include <linux/vm_sockets.h>
#include <sys/socket.h>
#include "kselftest_harness.h"

#ifndef AF_VSOCK
#define AF_VSOCK 40
#endif

/* ---------- netlink ---------- */

FIXTURE(netlink)
{
	int fd;
};

FIXTURE_SETUP(netlink)
{
	int group = RTNLGRP_LINK;

	self->fd = socket(AF_NETLINK, SOCK_RAW, NETLINK_ROUTE);
	if (self->fd < 0)
		SKIP(return, "AF_NETLINK socket: %s", strerror(errno));

	/* Joining a multicast group grows nlk->ngroups so the
	 * NETLINK_LIST_MEMBERSHIPS path has a non-zero size to report.
	 */
	if (setsockopt(self->fd, SOL_NETLINK, NETLINK_ADD_MEMBERSHIP,
		       &group, sizeof(group)) < 0)
		SKIP(return, "NETLINK_ADD_MEMBERSHIP: %s", strerror(errno));
}

FIXTURE_TEARDOWN(netlink)
{
	if (self->fd >= 0)
		close(self->fd);
}

TEST_F(netlink, pktinfo_exact)
{
	int val = -1;
	socklen_t optlen = sizeof(val);

	ASSERT_EQ(0, getsockopt(self->fd, SOL_NETLINK, NETLINK_PKTINFO,
				&val, &optlen));
	ASSERT_EQ(sizeof(int), optlen);
	ASSERT_TRUE(val == 0 || val == 1);
}

TEST_F(netlink, pktinfo_oversize_clamped)
{
	char buf[16] = {};
	socklen_t optlen = sizeof(buf);

	ASSERT_EQ(0, getsockopt(self->fd, SOL_NETLINK, NETLINK_PKTINFO,
				buf, &optlen));
	ASSERT_EQ(sizeof(int), optlen);
}

TEST_F(netlink, pktinfo_undersize)
{
	char buf[2] = {};
	socklen_t optlen = sizeof(buf);

	ASSERT_EQ(-1, getsockopt(self->fd, SOL_NETLINK, NETLINK_PKTINFO,
				 buf, &optlen));
	ASSERT_EQ(EINVAL, errno);
}

TEST_F(netlink, list_memberships_size_discovery)
{
	socklen_t optlen = 0;
	char dummy;

	ASSERT_EQ(0, getsockopt(self->fd, SOL_NETLINK,
				NETLINK_LIST_MEMBERSHIPS,
				&dummy, &optlen));
	ASSERT_GT(optlen, 0);
	ASSERT_EQ(0, optlen % sizeof(__u32));
}

TEST_F(netlink, list_memberships_full_read)
{
	__u32 buf[64] = {};
	socklen_t optlen = sizeof(buf);

	ASSERT_EQ(0, getsockopt(self->fd, SOL_NETLINK,
				NETLINK_LIST_MEMBERSHIPS,
				buf, &optlen));
	ASSERT_GT(optlen, 0);
	ASSERT_LE(optlen, sizeof(buf));
	ASSERT_EQ(0, optlen % sizeof(__u32));
}

TEST_F(netlink, bad_level)
{
	int val;
	socklen_t optlen = sizeof(val);

	ASSERT_EQ(-1, getsockopt(self->fd, SOL_SOCKET + 1, NETLINK_PKTINFO,
				 &val, &optlen));
	ASSERT_EQ(ENOPROTOOPT, errno);
}

TEST_F(netlink, bad_optname)
{
	int val;
	socklen_t optlen = sizeof(val);

	ASSERT_EQ(-1, getsockopt(self->fd, SOL_NETLINK, 0x7fff,
				 &val, &optlen));
	ASSERT_EQ(ENOPROTOOPT, errno);
}

/* ---------- vsock ---------- */

FIXTURE(vsock)
{
	int fd;
};

FIXTURE_SETUP(vsock)
{
	self->fd = socket(AF_VSOCK, SOCK_STREAM, 0);
	if (self->fd < 0)
		SKIP(return, "AF_VSOCK socket: %s", strerror(errno));
}

FIXTURE_TEARDOWN(vsock)
{
	if (self->fd >= 0)
		close(self->fd);
}

TEST_F(vsock, buffer_size_exact)
{
	uint64_t val = 0;
	socklen_t optlen = sizeof(val);

	ASSERT_EQ(0, getsockopt(self->fd, AF_VSOCK,
				SO_VM_SOCKETS_BUFFER_SIZE,
				&val, &optlen));
	ASSERT_EQ(sizeof(uint64_t), optlen);
	ASSERT_GT(val, 0);
}

TEST_F(vsock, buffer_size_oversize_clamped)
{
	char buf[16] = {};
	socklen_t optlen = sizeof(buf);

	ASSERT_EQ(0, getsockopt(self->fd, AF_VSOCK,
				SO_VM_SOCKETS_BUFFER_SIZE,
				buf, &optlen));
	ASSERT_EQ(sizeof(uint64_t), optlen);
}

TEST_F(vsock, buffer_size_undersize)
{
	char buf[4] = {};
	socklen_t optlen = sizeof(buf);

	ASSERT_EQ(-1, getsockopt(self->fd, AF_VSOCK,
				 SO_VM_SOCKETS_BUFFER_SIZE,
				 buf, &optlen));
	ASSERT_EQ(EINVAL, errno);
}

TEST_F(vsock, bad_level)
{
	uint64_t val;
	socklen_t optlen = sizeof(val);

	ASSERT_EQ(-1, getsockopt(self->fd, SOL_SOCKET + 1,
				 SO_VM_SOCKETS_BUFFER_SIZE,
				 &val, &optlen));
	ASSERT_EQ(ENOPROTOOPT, errno);
}

TEST_F(vsock, bad_optname)
{
	uint64_t val;
	socklen_t optlen = sizeof(val);

	ASSERT_EQ(-1, getsockopt(self->fd, AF_VSOCK, 0x7fff,
				 &val, &optlen));
	ASSERT_EQ(ENOPROTOOPT, errno);
}

TEST_HARNESS_MAIN
