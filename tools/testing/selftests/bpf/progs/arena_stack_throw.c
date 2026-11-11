// SPDX-License-Identifier: GPL-2.0

#include <stdbool.h>
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

extern void bpf_throw(__u64 cookie) __ksym;
extern void bpf_kfunc_common_test(void) __ksym;

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 64);
} arena SEC(".maps");

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	bpf_throw(0);
	bpf_kfunc_common_test();
}

long never, throws, regs[4];

/*
 * r6-r9 are in registers that a callee keeps. What the kfunc returns with is stored in regs[].
 * A program that can throw saves all such registers, so its own caller is not affected.
 */
SEC("syscall")
__naked int calls_kfunc(void)
{
	asm volatile (
	"r1 = %[never] ll;"
	"r1 = *(u64 *)(r1 + 0);"
	"if r1 == 0 goto +1;"
	"call %[bpf_throw];"
	"r6 = 6;"
	"r7 = 7;"
	"r8 = 8;"
	"r9 = 9;"
	"call %[bpf_kfunc_common_test];"
	"r1 = %[regs] ll;"
	"*(u64 *)(r1 + 0) = r6;"
	"*(u64 *)(r1 + 8) = r7;"
	"*(u64 *)(r1 + 16) = r8;"
	"*(u64 *)(r1 + 24) = r9;"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_throw),
	  __imm(bpf_kfunc_common_test),
	  __imm_addr(never),
	  __imm_addr(regs)
	: __clobber_all);
}

#ifdef __BPF_FEATURE_STACK_ARGUMENT

const volatile bool has_stack_arg = true;

static __naked __noinline __used
long callee_6args(long a, long b, long c, long d, long e, long f)
{
	asm volatile (
	"r0 = *(u64 *)(r11 + 8);"
	"exit;"
	::: __clobber_all);
}

/* The main program uses r12, has a stack argument and throws */
SEC("fentry/bpf_kfunc_common_test")
__naked int on_kfunc(void)
{
	asm volatile (
	".8byte %[adj];"	/* r12 += 16 */
	"r1 = %[throws] ll;"
	"r2 = 1;"
	"lock *(u64 *)(r1 + 0) += r2;"
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 8) = 6;"
	"call callee_6args;"
	"r1 = 0;"
	"call %[bpf_throw];"
	"exit;"
	:
	: __imm(bpf_throw),
	  __imm_addr(throws),
	  __imm_insn(adj, BPF_RAW_INSN(BPF_ALU64 | BPF_ADD | BPF_K, 12, 0, 0, 16))
	: __clobber_all);
}

#else

const volatile bool has_stack_arg = false;

SEC("fentry/bpf_kfunc_common_test")
int on_kfunc(void *ctx)
{
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
