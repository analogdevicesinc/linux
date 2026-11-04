// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2026 Meta Platforms, Inc and affiliates. */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

/*
 * Macro tricks to tersely define for long non-recursive call chains. Add
 * computation to the functions prevent tail recursion from reducing the
 * stack size to 0.
 */

#define CAT(a, b) a ## b
#define XCAT(a, b) CAT(a, b)

#define F_0 \
__attribute__((noinline))         \
int f0(unsigned long a)           \
{                                 \
	volatile long b = a + 16; \
	if (a == 0)               \
		return 0;         \
	return b;                 \
}

#define FN(n, prev) \
__attribute__((noinline))                        \
int XCAT(f, n)(unsigned long a)                  \
{                                                \
	volatile long b = XCAT(f, prev)(a - 1);  \
	if (!b)                                  \
		return 0;                        \
	return b + 1;                            \
}

/* Call chain 33 levels deep. */
#define F_1 F_0         FN(1, 0)
#define F_2 F_1         FN(2, 1)
#define F_3 F_2         FN(3, 2)
#define F_4 F_3         FN(4, 3)
#define F_5 F_4         FN(5, 4)
#define F_6 F_5         FN(6, 5)
#define F_7 F_6         FN(7, 6)
#define F_8 F_7         FN(8, 7)
#define F_9 F_8         FN(9, 8)
#define F_10 F_9        FN(10, 9)
#define F_11 F_10       FN(11, 10)
#define F_12 F_11       FN(12, 11)
#define F_13 F_12       FN(13, 12)
#define F_14 F_13       FN(14, 13)
#define F_15 F_14       FN(15, 14)
#define F_16 F_15       FN(16, 15)
#define F_17 F_16       FN(17, 16)
#define F_18 F_17       FN(18, 17)
#define F_19 F_18       FN(19, 18)
#define F_20 F_19       FN(20, 19)
#define F_21 F_20       FN(21, 20)
#define F_22 F_21       FN(22, 21)
#define F_23 F_22       FN(23, 22)
#define F_24 F_23       FN(24, 23)
#define F_25 F_24       FN(25, 24)
#define F_26 F_25       FN(26, 25)
#define F_27 F_26       FN(27, 26)
#define F_28 F_27       FN(28, 27)
#define F_29 F_28       FN(29, 28)
#define F_30 F_29       FN(30, 29)
#define F_31 F_30       FN(31, 30)
#define F_32 F_31       FN(32, 31)

/* Same, with a 480-byte frame, to exceed the 2 KiB budget of large stacks. */
#define FNB(n, prev) \
__attribute__((noinline))                        \
int XCAT(f, n)(unsigned long a)                  \
{                                                \
	volatile char buf[480] = {};             \
	volatile long b = XCAT(f, prev)(a - 1);  \
	if (!b)                                  \
		return 0;                        \
	return b + buf[479] + 1;                 \
}

#define F_33 F_32       FNB(33, 32)
#define F_34 F_33       FNB(34, 33)
#define F_35 F_34       FNB(35, 34)
#define F_36 F_35       FNB(36, 35)
#define F_37 F_36       FNB(37, 36)

#define CAT2(a, b) a ## b
#define XCAT2(a, b) CAT2(a, b)

#define F(n) XCAT2(F_, n)

F(37)

#define ASM_FUNC(linkage, name, insns)					\
linkage __naked __noinline __used int name(void)			\
{									\
	asm volatile (insns "r0 = 0; exit;"				\
		      :: __imm(bpf_get_prandom_u32) : __clobber_all);	\
}

/*
 * Every function calls the next one in both branches of an 'if': 32 calls are
 * executed, and there are 2^32 chains of calls.
 */
#undef F_0
#undef FN
#define F_0 ASM_FUNC(, either0, "")
#define FN(n, prev)							\
	ASM_FUNC(, either##n,						\
		 "call %[bpf_get_prandom_u32];"				\
		 "if r0 == 0 goto +2;"					\
		 "call either" #prev ";"				\
		 "goto +1;"						\
		 "call either" #prev ";")
F(32)

SEC("syscall")
__success
__load_if_JITed()
int global_func_many_chains(void *ctx)
{
	return either32();
}

/*
 * r1 is the number of calls that follow, so main -> fs15 .. fs0 and main -> fs31 .. fs16
 * are executed: 17 frames each. main -> fs31 .. fs0 is in the call graph: 33 frames.
 * libbpf puts fs15() .. fs0() in front of their second caller fs16().
 */
#undef F_0
#undef FN
#define F_0 ASM_FUNC(static, fs0, "")
#define FN(n, prev) ASM_FUNC(static, fs##n, "if r1 == 0 goto +2; r1 += -1; call fs" #prev ";")
F(31)

SEC("syscall")
__failure __msg("the call stack of 32 frames is too deep !")
__load_if_JITed()
__naked int static_func_deep_stack_second_caller(void)
{
	asm volatile ("r1 = 15; call fs15; r1 = 15; call fs31; exit;" ::: __clobber_all);
}

/*
 * fm0() is frame 31. On the path that is verified first fp-16 and r7 are linked, so 'r7 > 7'
 * bounds fp-16. On the second path they are not. To see that, the verifier needs fp-16 live and
 * fp-16 and r7 precise in the checkpoint at 'goto +0'. These marks go through all that holds
 * a frame number: the mask of frames in the liveness pass, for which r1 can point to each of
 * the 32 frames, the jump history, and the linked registers in it. The liveness pass counts
 * the frames of a chain of calls that passes a pointer to the stack: here r10 of the main prog.
 */
#undef F_0
#undef FN
#define F_0								\
	ASM_FUNC(static, fm0,						\
		 "call %[bpf_get_prandom_u32]; r6 = r0; r6 &= 0xff;"	\
		 "call %[bpf_get_prandom_u32]; r7 = r0; r7 &= 0xff;"	\
		 "call %[bpf_get_prandom_u32];"				\
		 "if r0 > 1 goto +1; r7 = r6;"				\
		 "*(u64 *)(r10 - 16) = r6; r1 = r10; r1 += -16;"	\
		 "r2 = -8; r3 = r10; r3 += r2;"				\
		 "*(u64 *)(r3 + 0) = r1; r1 = *(u64 *)(r3 + 0);"	\
		 "goto +0;"						\
		 "if r7 > 7 goto +3; r2 = *(u64 *)(r1 + 0);"		\
		 "if r2 <= 7 goto +1; r0 = *(u8 *)(r2 + 0);")
#define FN(n, prev) ASM_FUNC(static, fm##n, "call fm" #prev ";")
F(30)

SEC("syscall")
__failure __msg("R2 invalid mem access 'scalar'")
__flag(BPF_F_TEST_STATE_FREQ)
__naked int static_func_deep_stack_marks(void)
{
	asm volatile ("r1 = r10; call fm30; exit;" ::: __clobber_all);
}

/* Ensure that even 32 levels deep, the function verifies. */
SEC("syscall")
__success
int global_func_deep_stack_success(struct __sk_buff *skb)
{
	return f31(55);
}

/*
 * Check we actually honor stack limits (33 * 16 = 528 > 512 = MAX_STACK_DEPTH).
 * The stack depth is 16 because the verifier calls round_up_stack_depth() on
 * the size.
 */
SEC("syscall")
__load_if_no_large_stack()
__failure __msg("combined stack size of 34 calls")
int global_func_deep_stack_fail(struct __sk_buff *skb)
{
	return f32(123);
}

/*
 * Five 480-byte frames on top of the chain: 5 * 480 + 33 * 16 = 2928 bytes,
 * more than the 2 KiB budget of JITs with large stacks, and more than 512
 * bytes after the second frame everywhere else.
 */
SEC("syscall")
__failure __msg("combined stack size of {{[0-9]+}} calls")
int global_func_deep_stack_fail_large(struct __sk_buff *skb)
{
	return f37(123);
}
