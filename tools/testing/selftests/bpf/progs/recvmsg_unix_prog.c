// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Meta Platforms, Inc. and affiliates. */

#include "vmlinux.h"

#include <string.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_core_read.h>
#include "bpf_kfuncs.h"

__u8 SERVUN_ADDRESS[] = "\0bpf_cgroup_unix_test";

SEC("cgroup/recvmsg_unix")
int recvmsg_unix_prog(struct bpf_sock_addr *ctx)
{
	struct bpf_sock_addr_kern *sa_kern = bpf_cast_to_kern_ctx(ctx);
	struct sockaddr_un *sa_kern_unaddr;
	__u32 unaddrlen = offsetof(struct sockaddr_un, sun_path) +
			  sizeof(SERVUN_ADDRESS) - 1;
	int ret;

	ret = bpf_sock_addr_set_sun_path(sa_kern, SERVUN_ADDRESS,
					 sizeof(SERVUN_ADDRESS) - 1);
	if (ret)
		return 1;

	if (sa_kern->uaddrlen != unaddrlen)
		return 1;

	sa_kern_unaddr = bpf_core_cast(sa_kern->uaddr, struct sockaddr_un);
	if (memcmp(sa_kern_unaddr->sun_path, SERVUN_ADDRESS,
			sizeof(SERVUN_ADDRESS) - 1) != 0)
		return 1;

	return 1;
}

char _license[] SEC("license") = "GPL";
