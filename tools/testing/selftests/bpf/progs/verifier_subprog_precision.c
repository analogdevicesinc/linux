// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2023 Meta Platforms, Inc. and affiliates. */

#include <errno.h>
#include <string.h>
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include <../../../tools/include/linux/filter.h>

int vals[] SEC(".data.vals") = {1, 2, 3, 4};

__naked __noinline __used
static unsigned long identity_subprog()
{
	/* the simplest *static* 64-bit identity function */
	asm volatile (
		"r0 = r1;"
		"exit;"
	);
}

__noinline __used
unsigned long global_identity_subprog(__u64 x)
{
	/* the simplest *global* 64-bit identity function */
	return x;
}

__naked __noinline __used
static unsigned long callback_subprog()
{
	/* the simplest callback function */
	asm volatile (
		"r0 = 0;"
		"exit;"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("7: (0f) r1 += r0                      ; R0=12 R1=map_value(map=.data.vals,ks=4,vs=16,imm=12)")
__naked int subprog_result_precise(void)
{
	asm volatile (
		"r6 = 3;"
		/* pass r6 through r1 into subprog to get it back as r0 */
		"r1 = r6;"
		"call identity_subprog;"
		/* now use subprog's returned value (which is a
		 * r6 -> r1 -> r0 chain), as index into vals array
		 */
		"r0 *= 4;"
		"r1 = %[vals];"
		/*
		 * here r0 has to be known: the constant went through the
		 * subprog call
		 */
		"r1 += r0;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6"
	);
}

__naked __noinline __used
static unsigned long fp_leaking_subprog()
{
	asm volatile (
		".8byte %[r0_eq_r10_cast_s8];"
		"exit;"
		:: __imm_insn(r0_eq_r10_cast_s8, BPF_MOVSX64_REG(BPF_REG_0, BPF_REG_10, 8))
	);
}

__naked __noinline __used
static unsigned long sneaky_fp_leaking_subprog()
{
	asm volatile (
		"r1 = r10;"
		".8byte %[r0_eq_r1_cast_s8];"
		"exit;"
		:: __imm_insn(r0_eq_r1_cast_s8, BPF_MOVSX64_REG(BPF_REG_0, BPF_REG_1, 8))
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: R0=scalar() R6=")
__msg("6: (0f) r1 += r0                      ; R0=scalar(smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4) R1=map_value(map=.data.vals,ks=4,vs=16,smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4)")
__naked int fp_precise_subprog_result(void)
{
	asm volatile (
		"call fp_leaking_subprog;"
		/* use subprog's returned value (which is derived from r10=fp
		 * register, so it's unknown), as index into vals array
		 */
		"r0 &= 3;"
		"r0 *= 4;"
		"r1 = %[vals];"
		"r1 += r0;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("3: R0=scalar() R6=")
__msg("6: (0f) r1 += r0                      ; R0=scalar(smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4) R1=map_value(map=.data.vals,ks=4,vs=16,smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4)")
__naked int sneaky_fp_precise_subprog_result(void)
{
	asm volatile (
		"call sneaky_fp_leaking_subprog;"
		/* use subprog's returned value (which is derived from r10=fp
		 * register, so it's unknown), as index into vals array
		 */
		"r0 &= 3;"
		"r0 *= 4;"
		"r1 = %[vals];"
		"r1 += r0;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("9: (0f) r1 += r0                      ; R0=scalar(smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4) R1=map_value(map=.data.vals,ks=4,vs=16,smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4)")
__naked int global_subprog_result_precise(void)
{
	asm volatile (
		"r6 = 3;"
		/* pass r6 through r1 into subprog to get it back as r0;
		 * given global_identity_subprog is global, nothing is known
		 * about what it returns
		 */
		"r1 = r6;"
		"call global_identity_subprog;"
		/* now use subprog's returned value (which is unknown now, so
		 * we need to clamp it), as index into vals array
		 */
		"if r0 < %[vals_arr_sz] goto 1f;"
		"r0 = %[vals_arr_sz] - 1;"
	"1:"
		"r0 *= 4;"
		"r1 = %[vals];"
		"r1 += r0;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals),
		  __imm_const(vals_arr_sz, ARRAY_SIZE(vals))
		: __clobber_common, "r6"
	);
}

__naked __noinline __used
static unsigned long loop_callback_bad()
{
	/* bpf_loop() callback that can return values outside of [0, 1] range */
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"if r0 s> 1000 goto 1f;"
		"r0 = 0;"
	"1:"
		"goto +0;"
		/* bpf_loop() expects [0, 1] values, so branch above skipping
		 * r0 = 0; should lead to a failure
		 */
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_common
	);
}

SEC("?raw_tp")
__failure __log_level(2)
/* 0 of the fallthrough code path doesn't stand for what the branch code path has */
__msg("At callback return the register R0 has {{.*}} should have been in [0, 1]")
__naked int callback_precise_return_fail(void)
{
	asm volatile (
		"r1 = 1;"			/* nr_loops */
		"r2 = %[loop_callback_bad];"	/* callback_fn */
		"r3 = 0;"			/* callback_ctx */
		"r4 = 0;"			/* flags */
		"call %[bpf_loop];"

		"r0 = 0;"
		"exit;"
		:
		: __imm_ptr(loop_callback_bad),
		  __imm(bpf_loop)
		: __clobber_common
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("9: (85) call bpf_loop#181")
__msg("17: frame1: R10=fp0 cb")
__msg("17: (b7) r0 = 0")
__msg("18: (95) exit")
__msg("returning from callee:")
__msg("to caller at 9:")
/* the callback changes nothing for the caller */
__msg("from 18 to 9: within")
__msg("14: (0f) r1 += r6                     ; R1=map_value(map=.data.vals,ks=4,vs=16,smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4) R6=scalar(smin=smin32=0,smax=umax=smax32=umax32=12,var_off=(0x0; 0xc),step=0+4)")
__naked int callback_result_precise(void)
{
	asm volatile (
		"r6 = 3;"

		/* call subprog and use result; r0 shouldn't propagate back to
		 * callback_subprog
		 */
		"r1 = r6;"			/* nr_loops */
		"r2 = %[callback_subprog];"	/* callback_fn */
		"r3 = 0;"			/* callback_ctx */
		"r4 = 0;"			/* flags */
		"call %[bpf_loop];"

		"r6 = r0;"
		"if r6 > 3 goto 1f;"
		"r6 *= 4;"
		"r1 = %[vals];"
		/* here the range of r6 has to be known */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
	"1:"
		"exit;"
		:
		: __imm_ptr(vals),
		  __imm_ptr(callback_subprog),
		  __imm(bpf_loop)
		: __clobber_common, "r6"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("7: (0f) r1 += r6                      ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=12) R6=12")
__naked int parent_callee_saved_reg_precise(void)
{
	asm volatile (
		"r6 = 3;"

		/* call subprog and ignore result; we need this call only to
		 * complicate jump history
		 */
		"r1 = 0;"
		"call identity_subprog;"

		"r6 *= 4;"
		"r1 = %[vals];"
		/* here r6 has to be known: the subprog call doesn't change it */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("7: (0f) r1 += r6                      ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=12) R6=12")
__naked int parent_callee_saved_reg_precise_global(void)
{
	asm volatile (
		"r6 = 3;"

		/* call subprog and ignore result; we need this call only to
		 * complicate jump history
		 */
		"r1 = 0;"
		"call global_identity_subprog;"

		"r6 *= 4;"
		"r1 = %[vals];"
		/* here r6 has to be known: the subprog call doesn't change it */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("12: (0f) r1 += r6                     ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=12) R6=12")
__msg("15: frame1: R10=fp0 cb")
__msg("15: (b7) r0 = 0")
__msg("16: (95) exit")
__msg("returning from callee:")
__msg("to caller at 9:")
/* r6 is what it was before the callback */
__msg("from 16 to 9: within")
__naked int parent_callee_saved_reg_precise_with_callback(void)
{
	asm volatile (
		"r6 = 3;"

		/* call subprog and ignore result; we need this call only to
		 * complicate jump history
		 */
		"r1 = 1;"			/* nr_loops */
		"r2 = %[callback_subprog];"	/* callback_fn */
		"r3 = 0;"			/* callback_ctx */
		"r4 = 0;"			/* flags */
		"call %[bpf_loop];"

		"r6 *= 4;"
		"r1 = %[vals];"
		/* here r6 has to be known: the callback doesn't change it */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals),
		  __imm_ptr(callback_subprog),
		  __imm(bpf_loop)
		: __clobber_common, "r6"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("9: (0f) r1 += r6                      ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=12) R6=12")
__naked int parent_stack_slot_precise(void)
{
	asm volatile (
		/* spill reg */
		"r6 = 3;"
		"*(u64 *)(r10 - 8) = r6;"

		/* call subprog and ignore result; we need this call only to
		 * complicate jump history
		 */
		"r1 = 0;"
		"call identity_subprog;"

		/* restore reg from stack; in this case we'll be carrying
		 * stack mask when going back into subprog through jump
		 * history
		 */
		"r6 = *(u64 *)(r10 - 8);"

		"r6 *= 4;"
		"r1 = %[vals];"
		/* here r6 has to be known: the subprog call doesn't change it */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("9: (0f) r1 += r6                      ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=12) R6=12")
__naked int parent_stack_slot_precise_global(void)
{
	asm volatile (
		/* spill reg */
		"r6 = 3;"
		"*(u64 *)(r10 - 8) = r6;"

		/* call subprog and ignore result; we need this call only to
		 * complicate jump history
		 */
		"r1 = 0;"
		"call global_identity_subprog;"

		/* restore reg from stack; in this case we'll be carrying
		 * stack mask when going back into subprog through jump
		 * history
		 */
		"r6 = *(u64 *)(r10 - 8);"

		"r6 *= 4;"
		"r1 = %[vals];"
		/* here r6 has to be known: the subprog call doesn't change it */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("14: (0f) r1 += r6                     ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=12) R6=12")
__msg("17: frame1: R10=fp0 cb")
__msg("17: (b7) r0 = 0")
__msg("18: (95) exit")
__msg("returning from callee:")
__msg("to caller at 10:")
/* fp-8 is what it was before the callback */
__msg("from 18 to 10: within")
__naked int parent_stack_slot_precise_with_callback(void)
{
	asm volatile (
		/* spill reg */
		"r6 = 3;"
		"*(u64 *)(r10 - 8) = r6;"

		/* ensure we have callback frame in jump history */
		"r1 = r6;"			/* nr_loops */
		"r2 = %[callback_subprog];"	/* callback_fn */
		"r3 = 0;"			/* callback_ctx */
		"r4 = 0;"			/* flags */
		"call %[bpf_loop];"

		/* restore reg from stack; in this case we'll be carrying
		 * stack mask when going back into subprog through jump
		 * history
		 */
		"r6 = *(u64 *)(r10 - 8);"

		"r6 *= 4;"
		"r1 = %[vals];"
		/* here r6 has to be known: the callback doesn't change fp-8 */
		"r1 += r6;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals),
		  __imm_ptr(callback_subprog),
		  __imm(bpf_loop)
		: __clobber_common, "r6"
	);
}

__noinline __used
static __u64 subprog_with_precise_arg(__u64 x)
{
	return vals[x]; /* x has to be known */
}

SEC("?raw_tp")
__success __log_level(2)
__msg("8: (0f) r2 += r1                      ; R1=12 R2=map_value(map=.data.vals,ks=4,vs=16,imm=12)")
__naked int subprog_arg_precise(void)
{
	asm volatile (
		"r6 = 3;"
		"r1 = r6;"
		/* subprog_with_precise_arg needs to know its argument */
		"call subprog_with_precise_arg;"
		"r0 += r6;"
		"exit;"
		:
		:
		: __clobber_common, "r6"
	);
}

/* r1 is pointer to stack slot;
 * r2 is a register to spill into that slot
 * subprog also spills r2 into its own stack slot
 */
__naked __noinline __used
static __u64 subprog_spill_reg_precise(void)
{
	asm volatile (
		/* spill to parent stack */
		"*(u64 *)(r1 + 0) = r2;"
		/* spill to subprog stack (we use -16 offset to avoid
		 * accidental confusion with parent's -8 stack slot in
		 * verifier log output)
		 */
		"*(u64 *)(r10 - 16) = r2;"
		/* use both spills as return result */
		"r0 = *(u64 *)(r10 - 16);"
		"r2 = *(u64 *)(r1 + 0);"
		"r0 += r2;"
		"exit;"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("10: (0f) r1 += r7                     ; R1=map_value(map=.data.vals,ks=4,vs=16,imm=4) R7=4")
__naked int subprog_spill_into_parent_stack_slot_precise(void)
{
	asm volatile (
		"r6 = 1;"

		/* pass pointer to stack slot and r6 to subprog;
		 * r6 will be spilled into fp-8 slot
		 */
		"r1 = r10;"
		"r1 += -8;"
		"r2 = r6;"
		"call subprog_spill_reg_precise;"

		/* restore reg from stack; in this case we'll be carrying
		 * stack mask when going back into subprog through jump
		 * history
		 */
		"r7 = *(u64 *)(r10 - 8);"

		"r7 *= 4;"
		"r1 = %[vals];"
		/*
		 * here r7 has to be known: it's what the subprog spilled into
		 * the stack slot of its caller
		 */
		"r1 += r7;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6", "r7"
	);
}

SEC("?raw_tp")
__success __log_level(2)
__msg("17: (0f) r1 += r0                     ; R0=4 R1=map_value(map=.data.vals,ks=4,vs=16,imm=4)")
__naked int stack_slot_aliases_precision(void)
{
	asm volatile (
		"r6 = 1;"
		/* pass r6 through r1 into subprog to get it back as r0 */
		"r1 = r6;"
		"call identity_subprog;"
		/* let's setup two registers that are aliased to r10 */
		"r7 = r10;"
		"r7 += -8;"			/* r7 = r10 - 8 */
		"r8 = r10;"
		"r8 += -32;"			/* r8 = r10 - 32 */
		/* now spill subprog's return value (a r6 -> r1 -> r0 chain)
		 * a few times through different stack pointer regs, making
		 * sure to use r10, r7, and r8 both in LDX and STX insns, and
		 * *importantly* also using a combination of const var_off and
		 * insn->off to validate that we record final stack slot
		 * correctly, instead of relying on just insn->off derivation,
		 * which is only valid for r10-based stack offset
		 */
		"*(u64 *)(r10 - 16) = r0;"
		"r0 = *(u64 *)(r7 - 8);"	/* r7 - 8 == r10 - 16 */
		"*(u64 *)(r8 + 16) = r0;"	/* r8 + 16 = r10 - 16 */
		"r0 = *(u64 *)(r8 + 16);"
		"*(u64 *)(r7 - 8) = r0;"
		"r0 = *(u64 *)(r10 - 16);"
		/* get ready to use r0 as an index into array */
		"r0 *= 4;"
		"r1 = %[vals];"
		/*
		 * here r0 has to be known: the constant went through the
		 * subprog call and all the stack spills and loads
		 */
		"r1 += r0;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common, "r6", "r7", "r8"
	);
}

struct {
        __uint(type, BPF_MAP_TYPE_PROG_ARRAY);
        __uint(max_entries, 1);
        __type(key, __u32);
        __type(value, __u32);
} map_array SEC(".maps");

__naked __noinline __used
static unsigned long identity_tail_call(void)
{
	/* the simplest identity function involving a tail call */
        asm volatile (
		"r6 = r2;"
		"r2 = %[map_array] ll;"
		"r3 = 0;"
		"call %[bpf_tail_call];"
		"r0 = r6;"
		"exit;"
		:
		: __imm(bpf_tail_call),
		  __imm_addr(map_array)
		: __clobber_all);
}

SEC("?raw_tp")
__failure __log_level(2)
__msg("13: (85) call bpf_tail_call#12")
__msg("returning from callee:")
__msg("frame1: R0=scalar() R6=3 R10=fp0")
__msg("to caller at 4:")
__msg("R0=scalar() R6=map_value(map=.data.vals,ks=4,vs=16) R10=fp0")
__msg("6: (0f) r1 += r0")
__msg("math between map_value pointer and register with unbounded min value is not allowed")
__naked int subprog_result_tail_call(void)
{
	asm volatile (
		"r2 = 3;"
		"call identity_tail_call;"
		"r0 *= 4;"
		"r1 = %[vals];"
		"r1 += r0;"
		"r0 = *(u32 *)(r1 + 0);"
		"exit;"
		:
		: __imm_ptr(vals)
		: __clobber_common
	);
}

__naked __noinline __used
static int ld_abs_subprog(void)
{
	asm volatile (
		"r6 = r1;"
		"r7 = r1;"
		".8byte %[ld_abs];"
		"exit;"
		:
		: __imm_insn(ld_abs, BPF_LD_ABS(BPF_W, 0))
		: __clobber_all);
}

/*
 * BPF_LD | BPF_ABS instruction is modelled as an early exit of the subprogram.
 * r7 is known on both paths.
 */
SEC("socket")
__success
__log_level(2)
/* the early exit and the exit of the subprogram give the same state */
__msg("from 10 to 2: within")
__msg("3: (0f) r1 += r7                      ; R1=fp-8 R7=-8")
__naked int ld_abs_backtrack_both_paths(void)
{
	asm volatile (
		"r7 = -8;"
		"call ld_abs_subprog;"
		"r1 = r10;"
		"r1 += r7;"
		"*(u64 *)(r1 + 0) = 0;"
		"r0 = 0;"
		"exit;"
		::: __clobber_all
	);
}

char _license[] SEC("license") = "GPL";
