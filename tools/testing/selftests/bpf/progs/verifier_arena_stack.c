// SPDX-License-Identifier: GPL-2.0

#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

#if defined(__TARGET_ARCH_x86) || defined(__TARGET_ARCH_arm64)

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 64);
} arena SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_PROG_ARRAY);
	__uint(max_entries, 1);
	__uint(key_size, sizeof(int));
	__uint(value_size, sizeof(int));
} jmp_table SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_USER_RINGBUF);
	__uint(max_entries, 4096);
} user_ringbuf SEC(".maps");

/* Compilers don't know r12 yet */
#define R12 12
/* r12 += n */
#define ADJ(n)		BPF_RAW_INSN(BPF_ALU64 | BPF_ADD | BPF_K, R12, 0, 0, n)
/* dst = r12 */
#define MOV(dst)	BPF_RAW_INSN(BPF_ALU64 | BPF_MOV | BPF_X, dst, R12, 0, 0)
/* dst = *(sz *)(r12 + off) */
#define LDX(sz, dst, off) BPF_RAW_INSN(BPF_LDX | BPF_MEM | sz, dst, R12, off, 0)
#define LDSX(sz, dst, off) BPF_RAW_INSN(BPF_LDX | BPF_MEMSX | sz, dst, R12, off, 0)
/* *(sz *)(r12 + off) = src */
#define STX(sz, off, src) BPF_RAW_INSN(BPF_STX | BPF_MEM | sz, R12, src, off, 0)
/* *(sz *)(r12 + off) = imm */
#define ST(sz, off, imm) BPF_RAW_INSN(BPF_ST | BPF_MEM | sz, R12, 0, off, imm)
/* dst = addr_space_cast(dst, 0, 1) */
#define CAST_KERN(dst)	BPF_RAW_INSN(BPF_ALU64 | BPF_MOV | BPF_X, dst, dst, BPF_ADDR_SPACE_CAST, 1)
/* gotox reg */
#define GOTOX(reg)	BPF_RAW_INSN(BPF_JMP | BPF_JA | BPF_X, reg, 0, 0, 0)

SEC("syscall")
__description("arena stack: load and store")
__success __retval(12)
__xlated("r0 = *(u64 *)(r12 -16)")
__xlated("r2 = *(u64 *)(r12 -8)")
__naked void load_store(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 16) = 7 */
	"r1 = 5;"
	".8byte %[stx];"	/* *(u64 *)(r12 - 8) = r1 */
	".8byte %[ldx0];"	/* r0 = *(u64 *)(r12 - 16) */
	".8byte %[ldx1];"	/* r2 = *(u64 *)(r12 - 8) */
	"r0 += r2;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 7)),
	  __imm_insn(stx, STX(BPF_DW, -8, BPF_REG_1)),
	  __imm_insn(ldx0, LDX(BPF_DW, BPF_REG_0, -16)),
	  __imm_insn(ldx1, LDX(BPF_DW, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: every size, sign extension")
__success __retval(0)
__naked void sizes(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 0x8899aabb ll;"
	".8byte %[stx];"	/* *(u64 *)(r12 - 16) = r1 */
	"r0 = 1;"
	".8byte %[ldx_b];"
	"if r2 != 0xbb goto 1f;"
	"r0 = 2;"
	".8byte %[ldx_h];"
	"if r2 != 0xaabb goto 1f;"
	"r0 = 3;"
	".8byte %[ldx_w];"
	"r3 = 0x8899aabb ll;"
	"if r2 != r3 goto 1f;"
	"r0 = 4;"
	".8byte %[ldsx_w];"
	"if r2 s>= 0 goto 1f;"
	"r0 = 5;"
	".8byte %[st_b];"	/* *(u8 *)(r12 - 15) = 0 */
	".8byte %[ldx_h];"
	"if r2 != 0xbb goto 1f;"
	"r0 = 0;"
"1:"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(stx, STX(BPF_DW, -16, BPF_REG_1)),
	  __imm_insn(ldx_b, LDX(BPF_B, BPF_REG_2, -16)),
	  __imm_insn(ldx_h, LDX(BPF_H, BPF_REG_2, -16)),
	  __imm_insn(ldx_w, LDX(BPF_W, BPF_REG_2, -16)),
	  __imm_insn(ldsx_w, LDSX(BPF_W, BPF_REG_2, -16)),
	  __imm_insn(st_b, ST(BPF_B, -15, 0))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: rX = r12 is the address of the frame")
__success __retval(9)
__naked void address(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[mov];"	/* r1 = r12 */
	"r1 += -8;"
	".8byte %[cast];"
	"r2 = 9;"
	"*(u64 *)(r1 + 0) = r2;"
	".8byte %[ldx];"	/* r0 = *(u64 *)(r12 - 8) */
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(mov, MOV(BPF_REG_1)),
	  __imm_insn(cast, CAST_KERN(BPF_REG_1)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -8))
	: __clobber_all);
}

static __naked __noinline __used long callee_with_frame(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st0];"
	".8byte %[st1];"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st0, ST(BPF_DW, -16, 10)),
	  __imm_insn(st1, ST(BPF_DW, -8, 10)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -16))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: the frame of the callee is above")
__success __retval(21)
__naked void call(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 16) = 1 */
	"call callee_with_frame;"
	"r6 = r0;"
	"call callee_with_frame;"
	"r0 += r6;"
	".8byte %[ldx];"	/* r1 = *(u64 *)(r12 - 16) */
	"r0 += r1;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 1)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -16))
	: __clobber_all);
}

/* no frame of its own, r12 goes through */
static __naked __noinline __used long callee_in_the_middle(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"call callee_with_frame;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: a function without a frame in the middle")
__success __retval(11)
__naked void call_through(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	"call callee_in_the_middle;"
	".8byte %[ldx];"
	"r0 += r1;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 1)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -16))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: helpers don't change r12")
__success __retval(3)
__naked void helper(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	"call %[bpf_get_prandom_u32];"
	"call %[bpf_ktime_get_ns];"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm(bpf_ktime_get_ns),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 3)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -16))
	: __clobber_all);
}

static __naked __noinline __used long store_through(long addr)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	".8byte %[cast];"
	"r2 = 5;"
	"*(u64 *)(r1 + 0) = r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(64)),
	  __imm_insn(st, ST(BPF_DW, -64, 77)),
	  __imm_insn(cast, CAST_KERN(BPF_REG_1))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: the callee writes to the frame of the caller")
__success __retval(5)
__naked void pass_address(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[mov];"
	"r1 += -16;"
	"call store_through;"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(mov, MOV(BPF_REG_1)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -16))
	: __clobber_all);
}

static __naked __noinline __used long unwinds(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	"r0 = 0;"
	".8byte %[unwind_exit];"
	:
	: __imm_insn(adj, ADJ(32)),
	  __imm_insn(st, ST(BPF_DW, -32, -1)),
	  __imm_insn(unwind_exit, BPF_RAW_INSN(BPF_JMP | BPF_EXIT, 0, BPF_UNWIND_EXIT, 0, 0))
	: __clobber_all);
}

/* its pad only carries on */
static __naked __noinline __used long unwinds_through(void)
{
	asm volatile (
	".8byte %[adj];"
	"call unwinds;"
	".8byte %[unwind_goto];"
	"r0 = 0;"
	"exit;"
	".8byte %[unwind_exit];"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(unwind_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, BPF_UNWIND_GOTO, 2, 0)),
	  __imm_insn(unwind_exit, BPF_RAW_INSN(BPF_JMP | BPF_EXIT, 0, BPF_UNWIND_EXIT, 0, 0))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: r12 in the landing pad")
__success __retval(5)
__naked void unwind(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 16) = 5 */
	"call unwinds_through;"
	".8byte %[unwind_goto];"
	"r0 = 1;"
	"exit;"
	".8byte %[ldx];"	/* r0 = *(u64 *)(r12 - 16) */
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 5)),
	  __imm_insn(unwind_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, BPF_UNWIND_GOTO, 2, 0)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -16))
	: __clobber_all);
}

#ifdef __BPF_FEATURE_STACK_ARGUMENT

extern __u64 bpf_kfunc_call_stack_arg(__u64 a, __u64 b, __u64 c, __u64 d, __u64 e, __u64 f,
				      __u64 g, __u64 h, __u64 i, __u64 j) __ksym;

struct prog_test_pair_arg {
	__u64 lo;
	__u64 hi;
};

extern __u64 bpf_kfunc_call_test_pair_arg_nofit(__u64 a, __u64 b, __u64 c, __u64 d,
						struct prog_test_pair_arg s) __ksym;
extern void bpf_throw(__u64 cookie) __ksym;

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	struct prog_test_pair_arg s = {};

	bpf_kfunc_call_stack_arg(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	bpf_kfunc_call_test_pair_arg_nofit(0, 0, 0, 0, s);
	bpf_throw(0);
}

static __naked __noinline __used
long callee_7args(long a, long b, long c, long d, long e, long f, long g)
{
	asm volatile (
	".8byte %[adj];"
	"r0 = *(u64 *)(r11 + 8);"
	".8byte %[stx0];"	/* *(u64 *)(r12 - 16) = f */
	"r0 = *(u64 *)(r11 + 16);"
	".8byte %[stx8];"	/* *(u64 *)(r12 - 8) = g */
	"r6 = r5;"
	"call %[bpf_get_prandom_u32];"
	".8byte %[ldx0];"
	".8byte %[ldx8];"
	"r0 *= 10;"
	"r0 += r1;"
	"r0 *= 10;"
	"r0 += r6;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(stx0, STX(BPF_DW, -16, BPF_REG_0)),
	  __imm_insn(stx8, STX(BPF_DW, -8, BPF_REG_0)),
	  __imm_insn(ldx0, LDX(BPF_DW, BPF_REG_0, -16)),
	  __imm_insn(ldx8, LDX(BPF_DW, BPF_REG_1, -8))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: a function with 7 arguments")
__success __retval(1675)
__naked void stack_args(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 16) = 1000 */
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 16) = 7;"
	"*(u64 *)(r11 - 8) = 6;"
	"call callee_7args;"
	".8byte %[ldx];"
	"r0 += r1;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 1000)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -16))
	: __clobber_all);
}

SEC("tc")
__description("arena stack: a kfunc with 10 arguments")
__success __retval(1055)
__naked void kfunc_stack_args(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 16) = 1000 */
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 40) = 10;"
	"*(u64 *)(r11 - 32) = 9;"
	"*(u64 *)(r11 - 24) = 8;"
	"*(u64 *)(r11 - 16) = 7;"
	"*(u64 *)(r11 - 8) = 6;"
	"call %[bpf_kfunc_call_stack_arg];"
	".8byte %[ldx];"
	"r0 += r1;"
	"exit;"
	:
	: __imm(bpf_kfunc_call_stack_arg),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 1000)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -16))
	: __clobber_all);
}

/*
 * bpf_throw() finds the registers that the main program saved from its stack pointer at the
 * call. x86 moves rsp for a kfunc that has stack arguments. bpf_throw() is not one of them.
 */
SEC("syscall")
__description("arena stack: no move of the stack pointer in front of bpf_throw()")
__success
__arch_x86_64
__jited("	movq	%r9, {{.*}}(%rbp)")
__jited("	callq	{{.*}}")
__naked void throw_stack_args_jit(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 16) = 7;"
	"*(u64 *)(r11 - 8) = 6;"
	"call callee_7args;"
	"r1 = r0;"
	"call %[bpf_throw];"
	"exit;"
	:
	: __imm(bpf_throw),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

/* The caller of the program goes on with the registers that the exception callback restores */
SEC("syscall")
__description("arena stack: the main program has stack arguments and throws")
__success __retval(675)
__naked void throw_stack_args(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 16) = 7;"
	"*(u64 *)(r11 - 8) = 6;"
	"call callee_7args;"
	"r1 = r0;"
	"call %[bpf_throw];"
	"exit;"
	:
	: __imm(bpf_throw),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

/* The second half of the struct is argument 6 for the JIT: slots count, not arguments */
SEC("tc")
__description("arena stack: a kfunc with 5 arguments in 6 slots")
__success __retval(1091)
__naked void kfunc_5args_6slots(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 16) = 1000 */
	"r1 = 1;"
	"r2 = 2;"
	"r3 = 3;"
	"r4 = 4;"
	"r5 = 5;"
	"*(u64 *)(r11 - 8) = 6;"
	"call %[bpf_kfunc_call_test_pair_arg_nofit];"
	".8byte %[ldx];"
	"r0 += r1;"
	"exit;"
	:
	: __imm(bpf_kfunc_call_test_pair_arg_nofit),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -16, 1000)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -16))
	: __clobber_all);
}

#endif /* __BPF_FEATURE_STACK_ARGUMENT */

SEC("syscall")
__description("arena stack: what is loaded is a number")
__failure __msg("R1 invalid mem access 'scalar'")
/* the report names the load as the origin of the number */
__msg("Causal path:")
__msg(">>> 3 | (79) r1 = *(u64 *)(r12 -16)")
__naked void loaded_is_number(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = r10;"
	".8byte %[stx];"
	".8byte %[ldx];"
	"r0 = *(u64 *)(r1 - 8);"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(stx, STX(BPF_DW, -16, BPF_REG_1)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -16))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: rX = r12 is a number")
__failure __msg("R1 invalid mem access 'scalar'")
__msg("Causal path:")
__msg(">>> 1 | (bf) r1 = r12")
__naked void address_is_number(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[mov];"
	"r0 = *(u64 *)(r1 + 0);"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(mov, MOV(BPF_REG_1))
	: __clobber_all);
}

#define BAD_INSN(name, desc, msg, first, second)			\
SEC("syscall")								\
__description("arena stack: " desc)					\
__failure __msg(msg)							\
__naked void name(void)							\
{									\
	asm volatile (							\
	".8byte %[first_insn];"						\
	".8byte %[second_insn];"					\
	"r0 = 0;"							\
	"exit;"								\
	:								\
	: __imm_insn(first_insn, first),				\
	  __imm_insn(second_insn, second)				\
	: __clobber_all);						\
}

#define NOP BPF_MOV64_IMM(BPF_REG_1, 0)

BAD_INSN(adj_not_first, "r12 += 16 is not the first insn",
	 "R12 can be changed by the first insn of a function only", NOP, ADJ(16))
BAD_INSN(adj_twice, "r12 += 16 twice",
	 "R12 can be changed by the first insn of a function only", ADJ(16), ADJ(16))
BAD_INSN(adj_negative, "r12 += -16", "invalid size of the frame", ADJ(-16), NOP)
BAD_INSN(adj_zero, "r12 += 0", "invalid size of the frame", ADJ(0), NOP)
BAD_INSN(adj_unaligned, "r12 += 8", "invalid size of the frame", ADJ(8), NOP)
BAD_INSN(adj_large, "r12 += 32784", "invalid size of the frame", ADJ(32784), NOP)
BAD_INSN(mov_to, "r12 = r1", "R12 is invalid", NOP, BPF_MOV64_REG(R12, BPF_REG_1))
BAD_INSN(mov_imm, "r12 = 0", "R12 is invalid", NOP, BPF_MOV64_IMM(R12, 0))
BAD_INSN(add_reg, "r12 += r1", "R12 is invalid", NOP, BPF_ALU64_REG(BPF_ADD, R12, BPF_REG_1))
BAD_INSN(adj32, "w12 += 16", "R12 is invalid", BPF_ALU32_IMM(BPF_ADD, R12, 16), NOP)
BAD_INSN(sub_imm, "r12 -= 16", "R12 is invalid", BPF_ALU64_IMM(BPF_SUB, R12, 16), NOP)
BAD_INSN(add_from, "r1 += r12", "R12 is invalid", NOP, BPF_ALU64_REG(BPF_ADD, BPF_REG_1, R12))
BAD_INSN(mov32_from, "w1 = w12", "R12 is invalid", NOP, BPF_MOV32_REG(BPF_REG_1, R12))
BAD_INSN(load_to, "r12 = *(u64 *)(r10 - 8)", "R12 is invalid", NOP,
	 BPF_LDX_MEM(BPF_DW, R12, BPF_REG_10, -8))
BAD_INSN(store_from, "*(u64 *)(r10 - 8) = r12", "R12 is invalid", NOP,
	 BPF_STX_MEM(BPF_DW, BPF_REG_10, R12, -8))
BAD_INSN(store_itself, "*(u64 *)(r12 - 8) = r12", "R12 is invalid", NOP,
	 BPF_STX_MEM(BPF_DW, R12, R12, -8))
BAD_INSN(atomic, "lock *(u64 *)(r12 - 8) += r1", "R12 is invalid", NOP,
	 BPF_ATOMIC_OP(BPF_DW, BPF_ADD, R12, BPF_REG_1, -8))
BAD_INSN(jump, "if r12 == 0 goto", "R12 is invalid", NOP, BPF_JMP_IMM(BPF_JEQ, R12, 0, 0))
BAD_INSN(r13, "r1 = r13", "R13 is invalid", NOP, BPF_MOV64_REG(BPF_REG_1, 13))
BAD_INSN(local_atomic, "atomic that is local to the cpu", "BPF_ATOMIC uses invalid atomic opcode",
	 BPF_ST_MEM(BPF_DW, BPF_REG_10, -8, 0),
	 BPF_ATOMIC_OP(BPF_DW, BPF_ADD | BPF_FETCH | 0x1000, BPF_REG_10, BPF_REG_1, -8))

SEC("syscall")
__description("arena stack: jump to r12 += 16")
__failure __msg("jump to the insn that changes R12")
__naked void jump_to_adj(void)
{
	asm volatile (
"1:"
	".8byte %[adj];"
	"call %[bpf_get_prandom_u32];"
	"if r0 != 0 goto 1b;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

/* adds the index and what the context points to, through its frame */
static __naked __noinline __used long loop_cb(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[stx16];"	/* *(u64 *)(r12 - 16) = r1 */
	"r4 = *(u64 *)(r2 + 0);"
	".8byte %[ldx16];"	/* r5 = *(u64 *)(r12 - 16) */
	"r4 += r5;"
	"*(u64 *)(r2 + 0) = r4;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(stx16, STX(BPF_DW, -16, BPF_REG_1)),
	  __imm_insn(ldx16, LDX(BPF_DW, BPF_REG_5, -16))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: r12 in a callback")
__success __retval(145)
__naked void callback(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 8) = 100 */
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = 10;"
	"r2 = loop_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	/* 0 + 1 + ... + 9 and what is in the frame of the main program */
	"r0 = *(u64 *)(r10 - 8);"
	".8byte %[ldx];"	/* r1 = *(u64 *)(r12 - 8) */
	"r0 += r1;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(st, ST(BPF_DW, -8, 100)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -8))
	: __clobber_all);
}

/* has no frame: its first insn is the call, not r12 += N */
static __naked __noinline __used long calls_loop_cb(void)
{
	asm volatile (
	"call loop_cb;"
	"exit;"
	::: __clobber_all);
}

/*
 * The main program names loop_cb() first, so libbpf puts it in front of the callback.
 * The code that takes a stack moves the first insn of the callback down.
 * The call has to find loop_cb() from there.
 */
SEC("syscall")
__description("arena stack: a callback starts with a call of a function in front of it")
__success __retval(145)
__xlated("call pc-{{[0-9]+}}")
__naked void callback_calls_backward(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = 100;"
	"r2 = r10;"
	"r2 += -8;"
	"call loop_cb;"
	"r1 = 10;"
	"r2 = calls_loop_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	/* 100 + 0 + 1 + ... + 9 */
	"r0 = *(u64 *)(r10 - 8);"
	"exit;"
	:
	: __imm(bpf_loop)
	: __clobber_all);
}

/* runs loop_cb() twice. It takes a stack for callee_with_frame() */
static __naked __noinline __used long loops_loop_cb(void)
{
	asm volatile (
	"r6 = loop_cb ll;"
	"r7 = r2;"
	"call callee_with_frame;"
	"r1 = 2;"
	"r2 = r6;"
	"r3 = r7;"
	"r4 = 0;"
	"call %[bpf_loop];"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_loop)
	: __clobber_all);
}

/* the same for the address of a function */
SEC("syscall")
__description("arena stack: a callback starts with the address of a function in front of it")
__success __retval(13)
__naked void callback_names_backward(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	/* 0 + 1 + 2 + 3 + 4 */
	"r1 = 5;"
	"r2 = loop_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	/* 3 * (0 + 1) */
	"r1 = 3;"
	"r2 = loops_loop_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	"r0 = *(u64 *)(r10 - 8);"
	"exit;"
	:
	: __imm(bpf_loop)
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: call of a callback that uses r12")
__failure __msg("loop_cb() uses R12 and is both called and a callback")
__naked void call_of_callback(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = 1;"
	"r2 = loop_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	"r1 = 1;"
	"r2 = r10;"
	"r2 += -8;"
	"call loop_cb;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

#ifdef CAN_USE_CALLX
SEC("syscall")
__description("arena stack: callx of a callback that uses r12")
__failure __msg("loop_cb() uses R12 and is both called and a callback")
__naked void callx_of_callback(void)
{
	asm volatile (
	".8byte %[adj];"
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = 1;"
	"r2 = loop_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	"r1 = 1;"
	"r2 = r10;"
	"r2 += -8;"
	"r6 = loop_cb ll;"
	"callx r6;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}
#endif

static __naked __noinline __used long drain_cb(void)
{
	asm volatile (
	".8byte %[adj];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

/* Only a helper calls it, like the callback of bpf_loop(): it can take a stack. */
SEC("syscall")
__description("arena stack: r12 in a callback of bpf_user_ringbuf_drain()")
__success __retval(0)
__naked void callback_of_drain(void)
{
	asm volatile (
	"r1 = %[user_ringbuf] ll;"
	"r2 = drain_cb ll;"
	"r3 = 0;"
	"r4 = 0;"
	"call %[bpf_user_ringbuf_drain];"
	"exit;"
	:
	: __imm(bpf_user_ringbuf_drain),
	  __imm_addr(user_ringbuf)
	: __clobber_all);
}

/* 200000 iterations: may_goto calls arch_bpf_timed_may_goto() three times */
SEC("syscall")
__description("arena stack: may_goto doesn't change r12")
__success __retval(42)
__naked void may_goto_keeps_r12(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 8) = 42 */
	".8byte %[mov6];"	/* r6 = r12 */
	"r7 = 0;"
"1:"
	".8byte %[may_goto];"
	"r7 += 1;"
	"if r7 < 200000 goto 1b;"
	"r0 = 1;"
	"if r7 != 200000 goto 2f;"
	"r0 = 2;"
	".8byte %[mov1];"	/* r1 = r12 */
	"if r1 != r6 goto 2f;"
	".8byte %[ldx];"	/* r0 = *(u64 *)(r12 - 8) */
"2:"
	"exit;"
	:
	: __imm_insn(adj, ADJ(32)),
	  __imm_insn(st, ST(BPF_DW, -8, 42)),
	  __imm_insn(mov6, MOV(BPF_REG_6)),
	  __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 2, 0)),
	  __imm_insn(mov1, MOV(BPF_REG_1)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -8))
	: __clobber_all);
}

/* counts the calls and spins until the time of may_goto is up */
static __naked __noinline __used long spin_cb(void)
{
	asm volatile (
	".8byte %[adj];"
	"r3 = *(u64 *)(r2 + 0);"
	"r3 += 1;"
	"*(u64 *)(r2 + 0) = r3;"
"1:"
	".8byte %[may_goto];"
	"goto 1b;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16)),
	  __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 1, 0))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: bpf_loop() ends when the time of may_goto is up")
__success __retval(121)
/* the second call of the callback returns 1 before it takes a stack */
__xlated("if r0 != 0x0 goto pc+2")
__xlated("r0 = 1")
__xlated("exit")
__xlated("r6 = ")
__xlated("r6 = &(void __percpu *)(r6)")
__xlated("...")
__xlated("w12 += 16")
__naked void loop_ends_early(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"		/* *(u64 *)(r12 - 8) = 100 */
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = %[nr_loops];"
	"r2 = spin_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	/* 2 calls, 1 of them ran */
	"r0 *= 10;"
	"r1 = *(u64 *)(r10 - 8);"
	"r0 += r1;"
	".8byte %[ldx];"	/* r1 = *(u64 *)(r12 - 8) */
	"r0 += r1;"
	"exit;"
	:
	: __imm(bpf_loop),
	  __imm_const(nr_loops, 1 << 23),
	  __imm_insn(adj, ADJ(32)),
	  __imm_insn(st, ST(BPF_DW, -8, 100)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_1, -8))
	: __clobber_all);
}

SEC("tc")
__description("arena stack: tail call")
__failure __msg("tail_calls are not allowed in programs that use R12")
__naked void tail_call(void)
{
	asm volatile (
	".8byte %[adj];"
	"r2 = %[jmp_table] ll;"
	"r3 = 0;"
	"call %[bpf_tail_call];"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_tail_call),
	  __imm_addr(jmp_table),
	  __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

SEC("tc")
__description("arena stack: ld_abs")
__failure __msg("BPF_LD_[ABS|IND] is not allowed in programs that use R12")
__naked void ld_abs(void)
{
	asm volatile (
	".8byte %[adj];"
	"r6 = r1;"
	"r0 = *(u8 *)skb[0];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

static __naked __noinline __used long frame_16(void)
{
	asm volatile (
	".8byte %[adj];"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16))
	: __clobber_all);
}

static __naked __noinline __used long frame_16k_calls_16(void)
{
	asm volatile (
	".8byte %[adj];"
	"call frame_16;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16384))
	: __clobber_all);
}

static __naked __noinline __used long frame_16k(void)
{
	asm volatile (
	".8byte %[adj];"
	".8byte %[st];"
	".8byte %[ldx];"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16384)),
	  __imm_insn(st, ST(BPF_DW, -16384, 4)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -16384))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: 32K")
__success __retval(4)
__naked void depth_max(void)
{
	asm volatile (
	".8byte %[adj];"
	"call frame_16k;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16384))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: more than 32K")
__failure __msg("combined size of the frames in the stack in arena is 32784. Too large")
__naked void depth_too_large(void)
{
	asm volatile (
	".8byte %[adj];"
	"call frame_16k_calls_16;"
	"exit;"
	:
	: __imm_insn(adj, ADJ(16384))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: gotox to r12 += 16")
__failure __msg("jump to the insn that changes R12")
__naked void gotox_to_adj(void)
{
	asm volatile (
	".pushsection .jumptables,\"\",@progbits;"
"jt_%=:"
	".quad out_%= - syscall;"
	".quad first_%= - syscall;"
	".size jt_%=, 16;"
	".global jt_%=;"
	".popsection;"
"first_%=:"
	".8byte %[adj];"
	/* r1 is ctx, then 0: the loop ends */
	"if r1 == 0 goto out_%=;"
	"call %[bpf_get_prandom_u32];"
	"r0 &= 1;"
	"r0 <<= 3;"
	"r2 = jt_%= ll;"
	"r2 += r0;"
	"r2 = *(u64 *)(r2 + 0);"
	"r1 = 0;"
	".8byte %[gotox];"	/* gotox r2 */
"out_%=:"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(adj, ADJ(16)),
	  __imm_insn(gotox, GOTOX(BPF_REG_2))
	: __clobber_all);
}

/*
 * The flag in the stack of the caller is 0, then 1: one jump to the first insn. The insns that
 * take the stack are in front of it and use r6-r8.
 */
static __naked __noinline __used long gotox_first_insn_cb(void)
{
	asm volatile (
	".pushsection .jumptables,\"\",@progbits;"
"jt_%=:"
	".quad first_%= - .text;"
	".size jt_%=, 8;"
	".global jt_%=;"
	".popsection;"
"first_%=:"
	"r3 = *(u64 *)(r2 + 0);"
	"if r3 != 0 goto second_%=;"
	"r3 = 1;"
	"*(u64 *)(r2 + 0) = r3;"
	"r6 = r2;"
	"r7 = 10;"
	"r8 = 100;"
	".8byte %[mov9];"	/* r9 = r12 */
	"r3 = jt_%= ll;"
	"r3 = *(u64 *)(r3 + 0);"
	".8byte %[gotox];"	/* gotox r3 */
"second_%=:"
	"r0 = *(u64 *)(r6 + 0);"
	"r0 += r7;"
	"r0 += r8;"
	".8byte %[mov3];"	/* r3 = r12 */
	"r3 -= r9;"
	"r0 += r3;"
	"*(u64 *)(r2 + 0) = r0;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(mov9, MOV(BPF_REG_9)),
	  __imm_insn(mov3, MOV(BPF_REG_3)),
	  __imm_insn(gotox, GOTOX(BPF_REG_3))
	: __clobber_all);
}

SEC("syscall")
__description("arena stack: gotox to the first insn of a callback")
__success __retval(111)
__naked void gotox_first_insn_of_callback(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r1 = 1;"
	"r2 = gotox_first_insn_cb ll;"
	"r3 = r10;"
	"r3 += -8;"
	"r4 = 0;"
	"call %[bpf_loop];"
	/* the flag through r6, r7, r8 and no change of r12: 1 + 10 + 100 + 0 */
	"r0 = *(u64 *)(r10 - 8);"
	"exit;"
	:
	: __imm(bpf_loop)
	: __clobber_all);
}

/*
 * The kernel's prologue is in front of the code that takes a stack. The exit in
 * the prologue has no frame to drop and no stack to give back.
 */
SEC("tc")
__description("arena stack: prologue of a prog that writes to the packet")
__success __retval(7)
__xlated("5: call unknown")		/* bpf_skb_pull_data() */
__xlated("6: if r0 == 0x0 goto pc+2")
__xlated("7: w0 = 2")
__xlated("8: exit")
__xlated("9: r1 = r6")
/* take a stack */
__xlated("10: r2 = 0x")
__xlated("12: r2 = &(void __percpu *)(r2)")
__xlated("...")
__xlated("r12 = *(u32 *)(r2 +8)")
/* main prog */
__xlated("w12 += 48")
__xlated("r2 = *(u64 *)(r1 +")
__naked void packet_write_prologue(void)
{
	asm volatile (
	".8byte %[adj];"
	"r2 = *(u32 *)(r1 + %[data]);"
	"r3 = *(u32 *)(r1 + %[data_end]);"
	"r4 = r2;"
	"r4 += 1;"
	"r0 = 1;"
	"if r4 > r3 goto 1f;"
	"r0 = 7;"
	"*(u8 *)(r2 + 0) = r0;"
	".8byte %[stx];"	/* *(u64 *)(r12 - 8) = r0 */
	".8byte %[ldx];"	/* r0 = *(u64 *)(r12 - 8) */
"1:"
	"exit;"
	:
	: __imm_insn(adj, ADJ(48)),
	  __imm_insn(stx, STX(BPF_DW, -8, BPF_REG_0)),
	  __imm_insn(ldx, LDX(BPF_DW, BPF_REG_0, -8)),
	  __imm_const(data, offsetof(struct __sk_buff, data)),
	  __imm_const(data_end, offsetof(struct __sk_buff, data_end))
	: __clobber_all);
}

#endif

char _license[] SEC("license") = "GPL";
