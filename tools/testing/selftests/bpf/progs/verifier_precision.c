// SPDX-License-Identifier: GPL-2.0
/* Copyright (C) 2023 SUSE LLC */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include <stdbool.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"
#include "bpf_kfuncs.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, __u64);
} precision_map SEC(".maps");

SEC("?raw_tp")
__success __log_level(2)
__msg("1: (87) r2 = -r2                      ; R2=-8")
__naked int bpf_neg(void)
{
	asm volatile (
		"r2 = 8;"
		"r2 = -r2;"
		"if r2 != -8 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("1: (d4) r2 = le16 r2                  ; R2=0")
__naked int bpf_end_to_le(void)
{
	asm volatile (
		"r2 = 0;"
		"r2 = le16 r2;"
		"if r2 != 0 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}


SEC("?raw_tp")
__success __log_level(2)
__msg("1: (dc) r2 = be16 r2                  ; R2=0")
__naked int bpf_end_to_be(void)
{
	asm volatile (
		"r2 = 0;"
		"r2 = be16 r2;"
		"if r2 != 0 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}

#if (defined(__TARGET_ARCH_arm64) || defined(__TARGET_ARCH_x86) || \
	(defined(__TARGET_ARCH_riscv) && __riscv_xlen == 64) || \
	defined(__TARGET_ARCH_arm) || defined(__TARGET_ARCH_s390) || \
	defined(__TARGET_ARCH_loongarch)) && __clang_major__ >= 18

SEC("?raw_tp")
__success __log_level(2)
__msg("1: (d7) r2 = bswap16 r2               ; R2=0")
__naked int bpf_end_bswap(void)
{
	asm volatile (
		"r2 = 0;"
		"r2 = bswap16 r2;"
		"if r2 != 0 goto 1f;"
		"r1 = r10;"
		"r1 += r2;"
	"1:"
		"r0 = 0;"
		"exit;"
		::: __clobber_all);
}

#ifdef CAN_USE_LOAD_ACQ_STORE_REL

SEC("?raw_tp")
__success __log_level(2)
__msg("2: (db) r2 = load_acquire((u64 *)(r10 -8)){{ *}}; R2=8 R10=fp0 fp-8=8")
__msg("4: (0f) r3 += r2                      ; R2=8 R3=fp8")
__naked int bpf_load_acquire(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	".8byte %[load_acquire_insn];" /* r2 = load_acquire((u64 *)(r10 - 8)); */
	"r3 = r10;"
	"r3 += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(load_acquire_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_LOAD_ACQ, BPF_REG_2, BPF_REG_10, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("1: (db) store_release((u64 *)(r10 -8), r1){{ *}}; R1=8 R10=fp0 fp-8=8")
__msg("4: (0f) r2 += r1                      ; R1=8 R2=fp8")
__naked int bpf_store_release(void)
{
	asm volatile (
	"r1 = 8;"
	".8byte %[store_release_insn];" /* store_release((u64 *)(r10 - 8), r1); */
	"r1 = *(u64 *)(r10 - 8);"
	"r2 = r10;"
	"r2 += r1;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(store_release_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_STORE_REL, BPF_REG_10, BPF_REG_1, -8))
	: __clobber_all);
}

#endif /* CAN_USE_LOAD_ACQ_STORE_REL */
#endif /* v4 instruction */

SEC("?raw_tp")
__success
__naked int state_loop_first_last_equal(void)
{
	asm volatile (
		"r0 = 0;"
	"l0_%=:"
		"r0 += 2;"
		/* scalar evolution has no formula for it: the loop is not counted */
		"r0 |= 0;"
		/* every few iterations we'll have a checkpoint here with
		 * first_idx == last_idx, potentially confusing precision
		 * backtracking logic
		 */
		"if r0 >= 10 goto l1_%=;"	/* checkpoint + mark_precise */
		"goto l0_%=;"
	"l1_%=:"
		"exit;"
		::: __clobber_common
	);
}

__used __naked static void __bpf_cond_op_r10(void)
{
	asm volatile (
	"r2 = 2314885393468386424 ll;"
	"goto +0;"
	"if r2 <= r10 goto +3;"
	"if r1 >= -1835016 goto +0;"
	"if r2 <= 8 goto +0;"
	"if r3 <= 0 goto +0;"
	"exit;"
	::: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("8: (bd) if r2 <= r10 goto pc+3")
__msg("9: (35) if r1 >= 0xffe3fff8 goto pc+0")
__msg("10: (b5) if r2 <= 0x8 goto pc+0")
__naked void bpf_cond_op_r10(void)
{
	asm volatile (
	"r3 = 0 ll;"
	"call __bpf_cond_op_r10;"
	"r0 = 0;"
	"exit;"
	::: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: (bf) r3 = r10")
__msg("4: (bd) if r3 <= r2 goto pc+1")
__msg("5: (b5) if r2 <= 0x8 goto pc+2")
__naked void bpf_cond_op_not_r10(void)
{
	asm volatile (
	"r0 = 0;"
	"r2 = 2314885393468386424 ll;"
	"r3 = r10;"
	"if r3 <= r2 goto +1;"
	"if r2 <= 8 goto +2;"
	"r0 = 2 ll;"
	"exit;"
	::: __clobber_all);
}

SEC("lsm.s/socket_connect")
__success __log_level(2)
__msg("0: (b7) r0 = 1                        ; R0=1")
__msg("1: (84) w0 = -w0                      ; R0=0xffffffff")
__naked int bpf_neg_2(void)
{
	/*
	 * lsm.s/socket_connect requires a return value within [-4095, 0].
	 * Returning -1 is allowed
	 */
	asm volatile (
	"r0 = 1;"
	"w0 = -w0;"
	"exit;"
	::: __clobber_all);
}

SEC("lsm.s/socket_connect")
__failure __msg("At program exit the register R0 has")
__naked int bpf_neg_3(void)
{
	/*
	 * lsm.s/socket_connect requires a return value within [-4095, 0].
	 * Returning -10000 is not allowed.
	 */
	asm volatile (
	"r0 = 10000;"
	"w0 = -w0;"
	"exit;"
	::: __clobber_all);
}

SEC("lsm.s/socket_connect")
__success __log_level(2)
__msg("0: (b7) r0 = 1                        ; R0=1")
__msg("1: (87) r0 = -r0                      ; R0=-1")
__naked int bpf_neg_4(void)
{
	/*
	 * lsm.s/socket_connect requires a return value within [-4095, 0].
	 * Returning -1 is allowed
	 */
	asm volatile (
	"r0 = 1;"
	"r0 = -r0;"
	"exit;"
	::: __clobber_all);
}

SEC("lsm.s/socket_connect")
__failure __msg("At program exit the register R0 has")
__naked int bpf_neg_5(void)
{
	/*
	 * lsm.s/socket_connect requires a return value within [-4095, 0].
	 * Returning -10000 is not allowed.
	 */
	asm volatile (
	"r0 = 10000;"
	"r0 = -r0;"
	"exit;"
	::: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: (db) r2 = atomic64_fetch_add((u64 *)(r10 -8), r2){{ *}}; R2=8 R10=fp0 fp-8=mmmmmmmm")
__msg("5: (0f) r3 += r2                      ; R2=8 R3=fp8")
__naked int bpf_atomic_fetch_add_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = 0;"
	".8byte %[fetch_add_insn];"	/* r2 = atomic_fetch_add(*(u64 *)(r10 - 8), r2) */
	"r3 = r10;"
	"r3 += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(fetch_add_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_ADD | BPF_FETCH, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: (db) r2 = atomic64_xchg((u64 *)(r10 -8), r2){{ *}}; R2=8 R10=fp0 fp-8=mmmmmmmm")
__msg("5: (0f) r3 += r2                      ; R2=8 R3=fp8")
__naked int bpf_atomic_xchg_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = 0;"
	".8byte %[xchg_insn];"		/* r2 = atomic_xchg(*(u64 *)(r10 - 8), r2) */
	"r3 = r10;"
	"r3 += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(xchg_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_XCHG, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: (db) r2 = atomic64_fetch_or((u64 *)(r10 -8), r2){{ *}}; R2=8 R10=fp0 fp-8=mmmmmmmm")
__msg("5: (0f) r3 += r2                      ; R2=8 R3=fp8")
__naked int bpf_atomic_fetch_or_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = 0;"
	".8byte %[fetch_or_insn];"	/* r2 = atomic_fetch_or(*(u64 *)(r10 - 8), r2) */
	"r3 = r10;"
	"r3 += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(fetch_or_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_OR | BPF_FETCH, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: (db) r2 = atomic64_fetch_and((u64 *)(r10 -8), r2){{ *}}; R2=8 R10=fp0 fp-8=mmmmmmmm")
__msg("5: (0f) r3 += r2                      ; R2=8 R3=fp8")
__naked int bpf_atomic_fetch_and_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = 0;"
	".8byte %[fetch_and_insn];"	/* r2 = atomic_fetch_and(*(u64 *)(r10 - 8), r2) */
	"r3 = r10;"
	"r3 += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(fetch_and_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_AND | BPF_FETCH, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: (db) r2 = atomic64_fetch_xor((u64 *)(r10 -8), r2){{ *}}; R2=8 R10=fp0 fp-8=mmmmmmmm")
__msg("5: (0f) r3 += r2                      ; R2=8 R3=fp8")
__naked int bpf_atomic_fetch_xor_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = 0;"
	".8byte %[fetch_xor_insn];"	/* r2 = atomic_fetch_xor(*(u64 *)(r10 - 8), r2) */
	"r3 = r10;"
	"r3 += r2;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(fetch_xor_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_XOR | BPF_FETCH, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("4: (db) r0 = atomic64_cmpxchg((u64 *)(r10 -8), r0, r2){{ *}}; R0=8 R2=0 R10=fp0 fp-8=mmmmmmmm")
__msg("6: (0f) r3 += r0                      ; R0=8 R3=fp8")
__naked int bpf_atomic_cmpxchg_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r0 = 0;"
	"r2 = 0;"
	".8byte %[cmpxchg_insn];"	/* r0 = atomic_cmpxchg(*(u64 *)(r10 - 8), r0, r2) */
	"r3 = r10;"
	"r3 += r0;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(cmpxchg_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_CMPXCHG, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

/*
 * Both the fetched value (r2) and a reread of the same stack slot (r3) are used.
 * The fetched value is known. After the atomic operation, the stack slot is
 * STACK_MISC, so what is reread is not.
 */
SEC("?raw_tp")
__success __log_level(2)
__msg("5: (bf) r4 = r2                       ; R2=8 R4=8")
__msg("6: (0f) r4 += r3                      ; R3=scalar() R4=scalar()")
__naked int bpf_atomic_fetch_add_dual_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = 0;"
	".8byte %[fetch_add_insn];"	/* r2 = atomic_fetch_add(*(u64 *)(r10 - 8), r2) */
	"r3 = *(u64 *)(r10 - 8);"
	"r4 = r2;"
	"r4 += r3;"
	"r4 &= 7;"
	"r5 = r10;"
	"r5 += r4;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(fetch_add_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_ADD | BPF_FETCH, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("6: (bf) r4 = r0                       ; R0=8 R4=8")
__msg("7: (0f) r4 += r3                      ; R3=scalar() R4=scalar()")
__naked int bpf_atomic_cmpxchg_dual_precision(void)
{
	asm volatile (
	"r1 = 8;"
	"*(u64 *)(r10 - 8) = r1;"
	"r0 = 8;"
	"r2 = 0;"
	".8byte %[cmpxchg_insn];"	/* r0 = atomic_cmpxchg(*(u64 *)(r10 - 8), r0, r2) */
	"r3 = *(u64 *)(r10 - 8);"
	"r4 = r0;"
	"r4 += r3;"
	"r4 &= 7;"
	"r5 = r10;"
	"r5 += r4;"
	"r0 = 0;"
	"exit;"
	:
	: __imm_insn(cmpxchg_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_CMPXCHG, BPF_REG_10, BPF_REG_2, -8))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("9: (db) r1 = atomic64_fetch_add((u64 *)(r0 +0), r1){{ *}}; R0=map_value({{.*}}) R1=scalar()")
__naked int bpf_atomic_fetch_add_map_precision(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = r10;"
	"r2 += -8;"
	"r1 = %[precision_map] ll;"
	"call %[bpf_map_lookup_elem];"
	"if r0 == 0 goto 1f;"
	"r1 = 0;"
	".8byte %[fetch_add_insn];"	/* r1 = atomic_fetch_add(*(u64 *)(r0 + 0), r1) */
	"r1 &= 7;"
	"r2 = r10;"
	"r2 += r1;"
	"1: r0 = 0;"
	"exit;"
	:
	: __imm_addr(precision_map),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(fetch_add_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_ADD | BPF_FETCH, BPF_REG_0, BPF_REG_1, 0))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("11: (db) r0 = atomic64_cmpxchg((u64 *)(r6 +0), r0, r1){{ *}}; R0=scalar() R1=0")
__naked int bpf_atomic_cmpxchg_map_precision(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = r10;"
	"r2 += -8;"
	"r1 = %[precision_map] ll;"
	"call %[bpf_map_lookup_elem];"
	"if r0 == 0 goto 1f;"
	"r6 = r0;"
	"r0 = 0;"
	"r1 = 0;"
	".8byte %[cmpxchg_insn];"	/* r0 = atomic_cmpxchg(*(u64 *)(r6 + 0), r0, r1) */
	"r0 &= 7;"
	"r2 = r10;"
	"r2 += r0;"
	"1: r0 = 0;"
	"exit;"
	:
	: __imm_addr(precision_map),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cmpxchg_insn,
		     BPF_ATOMIC_OP(BPF_DW, BPF_CMPXCHG, BPF_REG_6, BPF_REG_1, 0))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("9: (c3) r1 = atomic_fetch_add((u32 *)(r0 +0), r1){{ *}}; R0=map_value({{.*}}) R1=scalar(smin=0,smax=umax=0xffffffff,var_off=(0x0; 0xffffffff))")
__naked int bpf_atomic_fetch_add_32bit_precision(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = r10;"
	"r2 += -8;"
	"r1 = %[precision_map] ll;"
	"call %[bpf_map_lookup_elem];"
	"if r0 == 0 goto 1f;"
	"r1 = 0;"
	".8byte %[fetch_add_insn];"	/* r1 = atomic_fetch_add(*(u32 *)(r0 + 0), r1) */
	"r1 &= 7;"
	"r2 = r10;"
	"r2 += r1;"
	"1: r0 = 0;"
	"exit;"
	:
	: __imm_addr(precision_map),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(fetch_add_insn,
		     BPF_ATOMIC_OP(BPF_W, BPF_ADD | BPF_FETCH, BPF_REG_0, BPF_REG_1, 0))
	: __clobber_all);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("11: (c3) r0 = atomic_cmpxchg((u32 *)(r6 +0), r0, r1){{ *}}; R0=scalar(smin=0,smax=umax=0xffffffff,var_off=(0x0; 0xffffffff)) R1=0")
__naked int bpf_atomic_cmpxchg_32bit_precision(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	"r2 = r10;"
	"r2 += -8;"
	"r1 = %[precision_map] ll;"
	"call %[bpf_map_lookup_elem];"
	"if r0 == 0 goto 1f;"
	"r6 = r0;"
	"r0 = 0;"
	"r1 = 0;"
	".8byte %[cmpxchg_insn];"	/* r0 = atomic_cmpxchg(*(u32 *)(r6 + 0), r0, r1) */
	"r0 &= 7;"
	"r2 = r10;"
	"r2 += r0;"
	"1: r0 = 0;"
	"exit;"
	:
	: __imm_addr(precision_map),
	  __imm(bpf_map_lookup_elem),
	  __imm_insn(cmpxchg_insn,
		     BPF_ATOMIC_OP(BPF_W, BPF_CMPXCHG, BPF_REG_6, BPF_REG_1, 0))
	: __clobber_all);
}

/*
 * Verification takes two paths: with r1 being scalar zero on path (1)
 * and with r1 being some other scalar on path (2).
 * Check that the state of path (1) does not stand for path (2).
 */
SEC("?tc")
__failure __msg("R1 type=scalar expected=fp")
__naked int null_mem_arg_zero_size(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r1 = 42;"
		"if r0 > 42 goto 1f;"
		"r1 = 0;"
	"1:"
		"r2 = 0;"
		"r3 = 0;"
		"r4 = 0;"
		"r5 = 0;"
		/*
		 * ARG_PTR_TO_MEM | PTR_MAYBE_NULL parameter can be NULL,
		 * but can't be some other scalar value.
		 */
		"call %[bpf_csum_diff];"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32),
		  __imm(bpf_csum_diff)
		: __clobber_all);
}

__weak int subprog_mem_arg(int *p)
{
	if (p)
		return *p;
	return 0;
}

/*
 * Verification takes two paths: with r1 being scalar zero on path (1)
 * and with r1 being some other scalar on path (2).
 * Check that the state of path (1) does not stand for path (2).
 */
SEC("?raw_tp")
__failure __msg("R1 type=scalar expected=fp")
__naked int null_mem_arg_global_subprog(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r1 = 42;"
		"if r0 > 42 goto 1f;"
		"r1 = 0;"
	"1:"
		"call subprog_mem_arg;"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all);
}

/* Same as above, check that path with r3 == 0 does not stand for the path with r3 != 0 */
SEC("?tc")
__failure __msg("R3 type=scalar expected=fp")
int null_kfunc_arg_dynptr_slice(struct __sk_buff *skb)
{
	struct bpf_dynptr ptr;

	bpf_dynptr_from_skb(skb, 0, &ptr);
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r3 = 42;"
		"if r0 > 42 goto 1f;"
		"r3 = 0;"
	"1:"
		"r1 = %[ptr];"
		"r2 = 0;"
		"r4 = 8;"
		"call %[bpf_dynptr_slice];"
		:
		: __imm_ptr(ptr),
		  __imm(bpf_get_prandom_u32),
		  __imm(bpf_dynptr_slice)
		: __clobber_common);
	return 0;
}

void __kfunc_btf_root(void)
{
	bpf_dynptr_slice(0, 0, 0, 0);
}

char _license[] SEC("license") = "GPL";
