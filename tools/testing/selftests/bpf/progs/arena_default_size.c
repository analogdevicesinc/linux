// SPDX-License-Identifier: GPL-2.0

#define BPF_NO_KFUNC_PROTOTYPES
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_experimental.h"
#include <bpf_arena_common.h>

/* max_entries is not set */
struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
} arena SEC(".maps");

SEC("syscall")
int alloc(void *ctx)
{
	return bpf_arena_alloc_pages(&arena, NULL, 2, NUMA_NO_NODE, 0) ? 0 : 1;
}

char _license[] SEC("license") = "GPL";
