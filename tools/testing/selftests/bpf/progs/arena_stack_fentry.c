// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

SEC("fentry")
int BPF_PROG(on_entry)
{
	return 0;
}

char _license[] SEC("license") = "GPL";
