// SPDX-License-Identifier: GPL-2.0

#define BPF_NO_KFUNC_PROTOTYPES
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_experimental.h"
#include <bpf_arena_common.h>

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 1);
} arena SEC(".maps");

/* the pointer is in arena, what it points to is not */
int y = 1;
int *py SEC(".addr_space.1") = &y;

SEC("syscall")
int prog(void *ctx)
{
	return y;
}

char _license[] SEC("license") = "GPL";
