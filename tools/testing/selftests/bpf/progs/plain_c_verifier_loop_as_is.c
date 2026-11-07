// SPDX-License-Identifier: GPL-2.0
/* verifier_loop_as_is.c in plain C */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, __u64[9]);
} map SEC(".maps");

/* 8 comes back to the head of the inner loop, and goes from there to the head of the outer one */
SEC("socket")
__success __retval(0)
#ifndef __clang__
__skip("GCC unrolls the inner loop")
#endif
int inner_loop_stops_at_8(void *ctx)
{
	__u32 key = 0;
	__u64 n = 0, *v;
	int i, j;

	v = bpf_map_lookup_elem(&map, &key);
	if (!v)
		return 0;
	for (i = 0; i < 100; i++) {
		for (j = 0; j < 100; j++) {
			if (*(volatile __u64 *)v == 1)
				continue;
			if (n < 8)
				v[n++] = 0;
		}
	}
	return v[n];
}

char _license[] SEC("license") = "GPL";
