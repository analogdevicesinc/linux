// SPDX-License-Identifier: GPL-2.0
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>

/* Only a prog at egress returns 2. At exit the verifier has 1 and 2 in one state. */
SEC("cgroup_skb/egress")
int one_or_two(struct __sk_buff *skb)
{
	return skb->len > 100 ? 2 : 1;
}

char _license[] SEC("license") = "GPL";
