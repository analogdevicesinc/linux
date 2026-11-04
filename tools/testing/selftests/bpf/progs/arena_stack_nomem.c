// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

/* The arena has one page. take_page() takes it, so there is no page for a stack. */
struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 1);
} arena SEC(".maps");

void *bpf_arena_alloc_pages(void *map, void *addr, __u32 page_cnt, int node_id,
			    __u64 flags) __ksym;

int runs;

SEC("syscall")
int take_page(void *ctx)
{
	return bpf_arena_alloc_pages(&arena, NULL, 1, -1 /* NUMA_NO_NODE */, 0) ? 0 : 1;
}

SEC("syscall")
__naked int returns_1(void)
{
	asm volatile (
	".8byte %[adj];"	/* r12 += 16 */
	"r1 = %[runs] ll;"
	"r0 = 1;"
	"*(u32 *)(r1 + 0) = r0;"
	"exit;"
	:
	: __imm_addr(runs),
	  __imm_insn(adj, BPF_RAW_INSN(BPF_ALU64 | BPF_ADD | BPF_K, 12, 0, 0, 16))
	: __clobber_all);
}

char _license[] SEC("license") = "GPL";
