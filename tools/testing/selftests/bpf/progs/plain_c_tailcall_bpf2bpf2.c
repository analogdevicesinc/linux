// SPDX-License-Identifier: GPL-2.0
/* tailcall_bpf2bpf2.c in plain C */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_legacy.h"
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_PROG_ARRAY);
	__uint(max_entries, 1);
	__uint(key_size, sizeof(__u32));
	__uint(value_size, sizeof(__u32));
} jmp_table SEC(".maps");

static __noinline
int subprog_tail(struct __sk_buff *skb)
{
	int ret = 1;

	if (load_byte(skb, 0))
		bpf_tail_call_static(skb, &jmp_table, 1);
	else
		bpf_tail_call_static(skb, &jmp_table, 0);
	return ret;
}

int count = 0;

SEC("tc")
__success __retval(1)
#ifndef __clang__
__skip("GCC does not keep the skb in R6 for LD_ABS")
#endif
int classifier_0(struct __sk_buff *skb)
{
	count++;
	return subprog_tail(skb);
}

char __license[] SEC("license") = "GPL";
