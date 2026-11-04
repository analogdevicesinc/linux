// SPDX-License-Identifier: GPL-2.0

#include <vmlinux.h>
#include <bpf/bpf_tracing.h>
#include "bpf_misc.h"
#include "../test_kmods/bpf_testmod.h"
#include "../test_kmods/bpf_testmod_kfunc.h"

char _license[] SEC("license") = "GPL";

#if defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64)

/* One page: the stack of the cpu, or what take_page() takes so that there is no stack */
struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 1);
} arena SEC(".maps");

/* Compilers don't know r12 yet, vmlinux.h has no BPF_ALU64 and the like */
#define INSN(code, dst, src, off, imm) \
	((__u64)(code) | (__u64)(dst) << 8 | (__u64)(src) << 12 | \
	 (__u64)(__u16)(off) << 16 | (__u64)(__u32)(imm) << 32)
/* r12 += n */
#define ADJ(n)		INSN(0x07, 12, 0, 0, n)
/* *(u64 *)(r12 + off) = imm */
#define ST(off, imm)	INSN(0x7a, 12, 0, off, imm)
/* dst = *(u64 *)(r12 + off) */
#define LDX(dst, off)	INSN(0x79, dst, 12, off, 0)

/* args->a += 1, through the stack in arena */
static __naked __noinline __used int subprog(struct st_ops_args *args)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 8) = 1 */
	".8byte %[ldx];"	/* r0 = *(u64 *)(r12 - 8) */
	"r2 = *(u64 *)(r1 +0);"
	"r0 += r2;"
	"*(u64 *)(r1 +0) = r0;"
	"exit;"
	:
	: __imm_const(adj, ADJ(48)),
	  __imm_const(st, ST(-8, 1)),
	  __imm_const(ldx, LDX(BPF_REG_0, -8))
	: __clobber_all);
}

/*
 * The prologue and the store of ctx are in front of the code that takes a stack.
 * Every exit goes to the epilogue: the one without a stack is the first of them.
 */
__success
/* prologue */
__xlated("0: r6 = *(u64 *)(r1 +0)")
__xlated("1: r7 = *(u64 *)(r6 +0)")
__xlated("2: r7 += 1000")
__xlated("3: *(u64 *)(r6 +0) = r7")
/* save __u64 *ctx to stack */
__xlated("4: *(u64 *)(r10 -8) = r1")
/* take a stack */
__xlated("5: r2 = ")
__xlated("7: r2 = &(void __percpu *)(r2)")
__xlated("...")
__xlated("w12 = w0")
__xlated("r1 = r6")
__xlated("if r0 != 0x0 goto pc+")
/* no stack */
__xlated("r0 = 0")
/* epilogue */
__xlated("r1 = *(u64 *)(r10 -8)")
__xlated("r1 = *(u64 *)(r1 +0)")
__xlated("r6 = *(u64 *)(r1 +0)")
__xlated("r6 += 10000")
__xlated("*(u64 *)(r1 +0) = r6")
__xlated("r0 = r6")
__xlated("r0 *= 2")
__xlated("exit")
__xlated("...")
/* main prog */
__xlated("w12 += 16")
__xlated("*(u64 *)(r12 -8) = 10")
__xlated("...")
__xlated("call pc+")
/* give the stack back, then the epilogue */
__xlated("w12 += -16")
__xlated("if w12 & 0x10 goto pc+5")
__xlated("...")
__xlated("*(u64 *)(r2 +0) = 0")
__xlated("gotol pc-")
__xlated("...")
__xlated("r0 = r6")
__xlated("gotol pc-")
SEC("struct_ops/test_pro_epilogue")
__naked int test_pro_epilogue(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 8) = 10 */
	"r1 = *(u64 *)(r1 +0);"
	"r2 = *(u64 *)(r1 +0);"
	".8byte %[ldx];"	/* r3 = *(u64 *)(r12 - 8) */
	"r2 += r3;"
	"*(u64 *)(r1 +0) = r2;"
	"call subprog;"
	"exit;"
	:
	: __imm_const(adj, ADJ(16)),
	  __imm_const(st, ST(-8, 10)),
	  __imm_const(ldx, LDX(BPF_REG_3, -8))
	: __clobber_all);
}

/* No frame in the main prog, r12 is used by the function that it calls */
__success
SEC("struct_ops/test_prologue")
__naked int test_prologue(void)
{
	asm volatile (
	"r1 = *(u64 *)(r1 +0);"
	"call subprog;"
	"exit;"
	::: __clobber_all);
}

__success
SEC("struct_ops/test_epilogue")
__naked int test_epilogue(void)
{
	asm volatile (
	"r1 = *(u64 *)(r1 +0);"
	"call subprog;"
	"exit;"
	::: __clobber_all);
}

SEC("syscall")
__retval(1001) /* PROLOGUE_A [1000] + SUBPROG_A [1] */
int syscall_prologue(void *ctx)
{
	struct st_ops_args args = {};

	return bpf_kfunc_st_ops_test_prologue(&args);
}

SEC("syscall")
__retval(20002) /* (SUBPROG_A [1] + EPILOGUE_A [10000]) * 2 */
int syscall_epilogue(void *ctx)
{
	struct st_ops_args args = {};

	return bpf_kfunc_st_ops_test_epilogue(&args);
}

SEC("syscall")
__retval(22022) /* (PROLOGUE_A [1000] + MAIN_A [10] + SUBPROG_A [1] + EPILOGUE_A [10000]) * 2 */
int syscall_pro_epilogue(void *ctx)
{
	struct st_ops_args args = {};

	return bpf_kfunc_st_ops_test_pro_epilogue(&args);
}

static bool take_page(void)
{
	return bpf_arena_alloc_pages(&arena, NULL, 1, -1 /* NUMA_NO_NODE */, 0);
}

/* A prog that gets no stack is skipped. Its prologue and its epilogue are not. */
SEC("syscall")
__retval(1000) /* PROLOGUE_A [1000], the prog returns 0 */
int syscall_prologue_no_stack(void *ctx)
{
	struct st_ops_args args = {};

	if (!take_page() || bpf_kfunc_st_ops_test_prologue(&args))
		return -1;
	return args.a;
}

SEC("syscall")
__retval(20000) /* EPILOGUE_A [10000] * 2 */
int syscall_epilogue_no_stack(void *ctx)
{
	struct st_ops_args args = {};

	if (!take_page())
		return -1;
	return bpf_kfunc_st_ops_test_epilogue(&args);
}

SEC(".struct_ops.link")
struct bpf_testmod_st_ops pro_epilogue_arena_stack = {
	.test_prologue = (void *)test_prologue,
	.test_epilogue = (void *)test_epilogue,
	.test_pro_epilogue = (void *)test_pro_epilogue,
};

#endif
