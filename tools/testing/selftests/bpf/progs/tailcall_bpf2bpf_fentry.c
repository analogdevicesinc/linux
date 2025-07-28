// SPDX-License-Identifier: GPL-2.0
/* Copyright Leon Hwang */

#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

int count = 0;

SEC("fentry/subprog_tail")
int BPF_PROG(fentry, struct sk_buff *skb)
{
	count++;

	return 0;
}

char _license[] SEC("license") = "GPL";
