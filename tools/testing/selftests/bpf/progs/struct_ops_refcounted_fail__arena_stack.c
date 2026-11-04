// SPDX-License-Identifier: GPL-2.0

#include <vmlinux.h>
#include <bpf/bpf_tracing.h>
#include "../test_kmods/bpf_testmod.h"
#include "bpf_misc.h"

char _license[] SEC("license") = "GPL";

#if defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64)

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 1);
} arena SEC(".maps");

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	bpf_task_release(NULL);
}

/* r12 += 16, compilers don't know r12 yet */
#define ADJ_16 (0x07 | 12 << 8 | 16ULL << 32)

static __naked __noinline __used void uses_arena_stack(void)
{
	asm volatile (
	".8byte %[adj];"
	"exit;"
	:
	: __imm_const(adj, ADJ_16)
	: __clobber_all);
}

/*
 * A program that gets no stack in arena is skipped: nobody would release the
 * task. It doesn't matter which function of the program uses r12.
 */
SEC("struct_ops/test_refcounted")
__failure __msg("program with __ref argument cannot use R12")
__naked int refcounted_fail__arena_stack(void)
{
	asm volatile (
	"r1 = *(u64 *)(r1 + 8);"
	"call %[bpf_task_release];"
	"call uses_arena_stack;"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_task_release)
	: __clobber_all);
}

SEC(".struct_ops.link")
struct bpf_testmod_ops testmod_ref_acquire = {
	.test_refcounted = (void *)refcounted_fail__arena_stack,
};

#endif
