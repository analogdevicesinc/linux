// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2020, Tessares SA. */
/* Copyright (c) 2024, Kylin Software */
/* mptcp_subflow.c and mptcp_bpf.h in plain C */

#include "bpf_tracing_net.h"

#include "bpf_experimental.h"
#include "bpf_misc.h"

/* list helpers from include/linux/list.h */
static inline int list_is_head(const struct list_head *list,
			       const struct list_head *head)
{
	return list == head;
}

#define list_entry(ptr, type, member)					\
	container_of(ptr, type, member)

#define list_first_entry(ptr, type, member)				\
	list_entry((ptr)->next, type, member)

#define list_next_entry(pos, member)					\
	list_entry((pos)->member.next, typeof(*(pos)), member)

#define list_entry_is_head(pos, head, member)				\
	list_is_head(&pos->member, (head))

#define list_for_each_entry(pos, head, member)				\
	for (pos = list_first_entry(head, typeof(*pos), member);	\
	     !list_entry_is_head(pos, head, member);			\
	     pos = list_next_entry(pos, member))

/* mptcp helpers from protocol.h */
#define mptcp_for_each_subflow(__msk, __subflow)			\
	list_for_each_entry(__subflow, &((__msk)->conn_list), node)

static struct sock *
mptcp_subflow_tcp_sock(const struct mptcp_subflow_context *subflow)
{
	return subflow->tcp_sock;
}

char _license[] SEC("license") = "GPL";

char cc[TCP_CA_NAME_MAX] = "reno";
int pid;

static int _check_getsockopt_subflow_mark(struct mptcp_sock *msk, struct bpf_sockopt *ctx)
{
	struct mptcp_subflow_context *subflow;
	int i = 0;

	mptcp_for_each_subflow(msk, subflow) {
		struct sock *ssk;

		ssk = mptcp_subflow_tcp_sock(bpf_core_cast(subflow,
							   struct mptcp_subflow_context));

		if (ssk->sk_mark != ++i) {
			ctx->retval = -2;
			break;
		}
	}

	return 1;
}

static int _check_getsockopt_subflow_cc(struct mptcp_sock *msk, struct bpf_sockopt *ctx)
{
	struct mptcp_subflow_context *subflow;

	mptcp_for_each_subflow(msk, subflow) {
		struct inet_connection_sock *icsk;
		struct sock *ssk;

		ssk = mptcp_subflow_tcp_sock(bpf_core_cast(subflow,
							   struct mptcp_subflow_context));
		icsk = bpf_core_cast(ssk, struct inet_connection_sock);

		if (ssk->sk_mark == 2 &&
		    __builtin_memcmp(icsk->icsk_ca_ops->name, cc, TCP_CA_NAME_MAX)) {
			ctx->retval = -2;
			break;
		}
	}

	return 1;
}

SEC("cgroup/getsockopt")
__success
int _getsockopt_subflow(struct bpf_sockopt *ctx)
{
	struct bpf_sock *sk = ctx->sk;
	struct mptcp_sock *msk;

	if (bpf_get_current_pid_tgid() >> 32 != pid)
		return 1;

	if (!sk || sk->protocol != IPPROTO_MPTCP ||
	    (!(ctx->level == SOL_SOCKET && ctx->optname == SO_MARK) &&
	     !(ctx->level == SOL_TCP && ctx->optname == TCP_CONGESTION)))
		return 1;

	msk = bpf_core_cast(sk, struct mptcp_sock);
	if (msk->pm.extra_subflows != 1) {
		ctx->retval = -1;
		return 1;
	}

	if (ctx->optname == SO_MARK)
		return _check_getsockopt_subflow_mark(msk, ctx);
	return _check_getsockopt_subflow_cc(msk, ctx);
}
