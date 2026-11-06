// SPDX-License-Identifier: GPL-2.0
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

SEC("tc")
__success
int skb_socket(struct __sk_buff *skb)
{
	struct bpf_sock *sk = skb->sk;

	return sk ? sk->family : 0;
}

SEC("sockops")
__success
int sockops_args_reply(struct bpf_sock_ops *skops)
{
	skops->reply = skops->args[1];
	return 0;
}

SEC("sk_msg")
__success
int sk_msg_data(struct sk_msg_md *msg)
{
	return msg->data < msg->data_end;
}

SEC("cgroup/getsockopt")
__success
int sockopt_optval(struct bpf_sockopt *ctx)
{
	return ctx->optval < ctx->optval_end;
}

SEC("tc")
__success
int skb_narrow_field(struct __sk_buff *skb)
{
	/* a nonzero byte offset becomes pointer arithmetic with preserve_access_index */
	return ((__u8 *)&skb->mark)[1];
}

#if __has_attribute(preserve_static_offset)
SEC("tc")
__success
int skb_merge_loads(struct __sk_buff *skb)
{
	__u32 value;

	if (skb->len)
		value = skb->pkt_type;
	else
		value = skb->mark;
	return value;
}
#endif

char _license[] SEC("license") = "GPL";
