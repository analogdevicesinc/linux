// SPDX-License-Identifier: GPL-2.0
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

/*
 * A join must not make up what neither state has where a jump ahead can see it:
 *	ret = lookup(&p);
 *	if (ret < 0)
 *		goto drop;
 *	use(p);
 * When lookup() is inlined (ret = -ENOENT, p is not set) and (ret = 0, p is set) meet in front of
 * the jump.
 */

SEC("socket")
__description("jumps ahead: a register is set when ret is 0")
__success __retval(0)
__naked void ret_and_register(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"w6 = -2;"
		"if r0 == 0 goto 1f;"
		"r8 = 0;"
		"w6 = 0;"
	"1:"
		"w7 = 0;"
		"if w6 s< 0 goto 2f;"
		"r0 = r8;"
		"exit;"
	"2:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

SEC("socket")
__description("jumps ahead: a register is not set and ret doesn't tell")
__failure __msg("R8 !read_ok")
__naked void ret_does_not_tell(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"w6 = 1;"
		"if r0 == 0 goto 1f;"
		"r8 = 0;"
		"w6 = 0;"
	"1:"
		"w7 = 0;"
		"if w6 s< 0 goto 2f;"
		"r0 = r8;"
		"exit;"
	"2:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/*
 * As in bpf_host.o of cilium:
 *	err = helper();
 *	ret = err < 0 ? -155 : 0;
 *	if (err < 0)
 *		goto out;
 *	...
 * out:
 *	if (ret < 0)
 *		goto drop;
 * The states differ in what two jumps look at, and in nothing else.
 * (err < 0, ret = 0) is neither of them.
 */
SEC("socket")
__description("jumps ahead: two numbers that go together")
__success
__naked void two_numbers(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"w6 = 0;"
		"if w0 s> -1 goto 1f;"
		"w6 = -155;"
	"1:"
		"w7 = 0;"
		"if w0 s< 0 goto 2f;"
		"r0 = 0;"
		"exit;"
	"2:"
		"if w6 s< 0 goto 3f;"
		"r0 = r8;"
	"3:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/* -134 and -135 go the same way */
SEC("socket")
__description("jumps ahead: states that go the same way are joined")
__success __log_level(2)
__msg("from 4 to 7: joined")
__msg("7: R6=scalar({{.*}}smin32=-135,smax32=-134{{.*}}) R10=fp0")
__msg("from 6 to 7: R6=0 R8=0 R10=fp0")
__naked void same_way(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"w6 = -134;"
		"if r0 == 1 goto 1f;"
		"w6 = -135;"
		"if r0 == 2 goto 1f;"
		"r8 = 0;"
		"w6 = 0;"
	"1:"
		"w7 = 0;"
		"if w6 s< 0 goto 2f;"
		"r0 = r8;"
		"exit;"
	"2:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/*
 *	ret = idx < 8 ? 0 : -EINVAL;
 *	if (ret < 0)
 *		goto drop;
 *	use(buf[idx]);
 * 0 and -EINVAL leave a gap, so the states are not joined, and idx keeps its range.
 */
SEC("socket")
__description("jumps ahead: ret vouches for the range of another number")
__success
__naked void ret_and_range(void)
{
	asm volatile (
		"r1 = 0;"
		"*(u64 *)(r10 - 8) = r1;"
		"call %[bpf_get_prandom_u32];"
		"r7 = r0;"
		"w6 = 0;"
		"if r7 < 8 goto 1f;"
		"w6 = -22;"
	"1:"
		"w8 = 0;"
		"if w6 s< 0 goto 2f;"
		"r1 = r10;"
		"r1 += -8;"
		"r1 += r7;"
		"r0 = *(u8 *)(r1 + 0);"
		"exit;"
	"2:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/*
 *	ok = i < 8 && x == 2;
 *	if (x == 7)
 *		n++;
 *	if (ok)
 *		use(buf[i]);
 * A compiler keeps ok in a register: "w6 &= 1; if w6 == 0". (ok = 0, i >= 8), (ok = 1, i < 8) and
 * (ok = 0, i < 8) meet in front of "if (x == 7)". 0 and 1 leave no gap, and i < 8 comes with
 * either. They are not joined when the jump that tells 0 from 1 ends this basic block or the next.
 */
#define FLAG_SET(max, no, yes)			\
	"r1 = 0;"				\
	"*(u64 *)(r10 - 8) = r1;"		\
	"call %[bpf_get_prandom_u32];"		\
	"r7 = r0;"				\
	"call %[bpf_get_prandom_u32];"		\
	"w6 = " #no ";"				\
	"if r7 > " #max " goto +3;"		\
	"w6 = " #yes ";"			\
	"if r0 == 2 goto +1;"			\
	"w6 = " #no ";"
#define FLAG_USE(unless)				\
	"if " #unless " goto +4;"		\
	"r1 = r10;"				\
	"r1 += -8;"				\
	"r1 += r7;"				\
	"r0 = *(u8 *)(r1 + 0);"
#define FLAG_PROG(name, max, no, yes, rest)	\
__naked void name(void)				\
{						\
	asm volatile (				\
	FLAG_SET(max, no, yes)			\
	rest					\
	"exit;"					\
	:: __imm(bpf_get_prandom_u32) : __clobber_all); \
}

SEC("socket")
__description("jumps ahead: a flag, another jump, the test of the flag")
__success
FLAG_PROG(flag_next_block, 7, 0, 1, "if r0 != 7 goto +1; r0 = 0; w6 &= 1;" FLAG_USE(w6 == 0))

SEC("socket")
__description("jumps ahead: a flag that says too much")
__failure __msg("invalid variable-offset read from stack R1")
FLAG_PROG(flag_next_block_bad, 8, 0, 1, "if r0 != 7 goto +1; r0 = 0; w6 &= 1;" FLAG_USE(w6 == 0))

/* "f = i < 8 ? in[i] : 0; ... if (f) use(buf[i]);" A compiler makes a flag of "f == 0" */
SEC("socket")
__description("jumps ahead: a flag that is 1 for no")
__success
FLAG_PROG(flag_for_no, 7, 1, 0, "if r0 != 7 goto +1; r0 = 0; w6 &= 1;" FLAG_USE(w6 != 0))

SEC("socket")
__description("jumps ahead: a flag is tested twice")
__success
FLAG_PROG(flag_twice, 7, 0, 1, FLAG_USE(w6 == 0) "r0 = 0;" FLAG_USE(w6 == 0))

SEC("socket")
__description("jumps ahead: the other value of a flag that is tested twice")
__failure __msg("math between fp pointer and register with unbounded min value")
FLAG_PROG(flag_twice_bad, 7, 0, 1, FLAG_USE(w6 == 0) "w6 ^= 1;" FLAG_USE(w6 == 0))

SEC("socket")
__description("jumps ahead: a flag is spilled, filled, copied and extended")
__success
FLAG_PROG(flag_moves, 7, 0, 1, "*(u64 *)(r10 - 16) = r6; r1 = *(u64 *)(r10 - 16);"
			       "r2 = r1; r2 <<= 32; r2 >>= 32;" FLAG_USE(r2 == 0))

/* if and else: two ends of basic blocks on every way to the test */
SEC("socket")
__description("jumps ahead: the test of a flag is too far")
__success __log_level(2)
__msg("from 8 to 10: joined")
FLAG_PROG(flag_too_far, 7, 0, 1, "if r0 != 7 goto +2; r0 = 0; goto +1; r0 = 1;"
				 "if w6 == 0 goto +1; r0 = 2;")

SEC("socket")
__description("jumps ahead: the jump tests another register than the flag")
__success __log_level(2)
__msg("from 8 to 10: joined")
FLAG_PROG(flag_not_tested, 7, 0, 1, "if r0 == 1 goto +1; r0 = r6;")

/* the first values of a counter */
SEC("socket")
__description("jumps ahead: 1 and 2 are no flag")
__success __log_level(2)
__msg("from 8 to 10: joined")
FLAG_PROG(flag_1_and_2, 7, 1, 2, "if w6 == 1 goto +1; r0 = 0;")

/*
 * A register that is set in one state and not in the other: these states are not joined,
 * whatever it is that tells them apart later.
 */
SEC("socket")
__description("set or not: a register is set when a flag is 1, flag + 4 is tested")
__success
__naked void flag_add(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r6 = 0;"
	"if r0 == 0 goto 1f;"
	"r8 = 0;"
	"r6 = 1;"
"1:"
	"r1 = r6;"
	"r1 += 4;"
	"if r1 != 5 goto 2f;"
	"r0 = r8;"
	"exit;"
"2:"
	"r0 = 0;"
	"exit;"
	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/* The first iteration sets r7 and the flag. The states meet at the head of the loop. */
SEC("socket")
__description("set or not: a loop sets a register and a flag")
__success
__naked void set_in_a_loop(void)
{
	asm volatile (
	"r6 = 0;"
"1:"
	".8byte %[may_goto];"
	"if r6 == 0 goto 2f;"
	"r8 = r7;"
"2:"
	"call %[bpf_get_prandom_u32];"
	"r7 = r0;"
	"r6 = 1;"
	"goto 1b;"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, 6, 0))
	: __clobber_all);
}

/*
 * 20 flags, 0 or 5, all are looked at later: 2^20 ways, and 0 and 5 leave a gap.
 * States are kept apart while there is room. After that they are joined.
 */
#define PICK_FLAG(slot)					\
	"call %[bpf_get_prandom_u32];"			\
	"r1 = 0;"					\
	"if r0 == 0 goto +1;"				\
	"r1 = 5;"					\
	"*(u64*)(r10 - " #slot ") = r1;"
#define USE_FLAG(slot)					\
	"r1 = *(u64*)(r10 - " #slot ");"		\
	"if r1 == 0 goto +1;"				\
	"r6 += 1;"

SEC("socket")
__description("jumps ahead: 20 flags that leave a gap")
__success
__naked void many_flags_gap(void)
{
	asm volatile (
	"r6 = 0;"
	PICK_FLAG(8) PICK_FLAG(16) PICK_FLAG(24) PICK_FLAG(32) PICK_FLAG(40) PICK_FLAG(48)
	PICK_FLAG(56) PICK_FLAG(64) PICK_FLAG(72) PICK_FLAG(80) PICK_FLAG(88) PICK_FLAG(96)
	PICK_FLAG(104) PICK_FLAG(112) PICK_FLAG(120) PICK_FLAG(128) PICK_FLAG(136) PICK_FLAG(144)
	PICK_FLAG(152) PICK_FLAG(160)
	USE_FLAG(8) USE_FLAG(16) USE_FLAG(24) USE_FLAG(32) USE_FLAG(40) USE_FLAG(48)
	USE_FLAG(56) USE_FLAG(64) USE_FLAG(72) USE_FLAG(80) USE_FLAG(88) USE_FLAG(96)
	USE_FLAG(104) USE_FLAG(112) USE_FLAG(120) USE_FLAG(128) USE_FLAG(136) USE_FLAG(144)
	USE_FLAG(152) USE_FLAG(160)
	"r0 = r6;"
	"exit;"
	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/* 33 numbers, each with an exit of its own */
static __naked __noinline __used int ret_exits(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = r0;"
	"if r1 != 0 goto +2; r0 = 0; exit;"
	"if r1 != 1 goto +2; r0 = 3; exit;"
	"if r1 != 2 goto +2; r0 = 6; exit;"
	"if r1 != 3 goto +2; r0 = 9; exit;"
	"if r1 != 4 goto +2; r0 = 12; exit;"
	"if r1 != 5 goto +2; r0 = 15; exit;"
	"if r1 != 6 goto +2; r0 = 18; exit;"
	"if r1 != 7 goto +2; r0 = 21; exit;"
	"if r1 != 8 goto +2; r0 = 24; exit;"
	"if r1 != 9 goto +2; r0 = 27; exit;"
	"if r1 != 10 goto +2; r0 = 30; exit;"
	"if r1 != 11 goto +2; r0 = 33; exit;"
	"if r1 != 12 goto +2; r0 = 36; exit;"
	"if r1 != 13 goto +2; r0 = 39; exit;"
	"if r1 != 14 goto +2; r0 = 42; exit;"
	"if r1 != 15 goto +2; r0 = 45; exit;"
	"if r1 != 16 goto +2; r0 = 48; exit;"
	"if r1 != 17 goto +2; r0 = 51; exit;"
	"if r1 != 18 goto +2; r0 = 54; exit;"
	"if r1 != 19 goto +2; r0 = 57; exit;"
	"if r1 != 20 goto +2; r0 = 60; exit;"
	"if r1 != 21 goto +2; r0 = 63; exit;"
	"if r1 != 22 goto +2; r0 = 66; exit;"
	"if r1 != 23 goto +2; r0 = 69; exit;"
	"if r1 != 24 goto +2; r0 = 72; exit;"
	"if r1 != 25 goto +2; r0 = 75; exit;"
	"if r1 != 26 goto +2; r0 = 78; exit;"
	"if r1 != 27 goto +2; r0 = 81; exit;"
	"if r1 != 28 goto +2; r0 = 84; exit;"
	"if r1 != 29 goto +2; r0 = 87; exit;"
	"if r1 != 30 goto +2; r0 = 90; exit;"
	"if r1 != 31 goto +2; r0 = 93; exit;"
	"r0 = 99;"
	"exit;"
	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/* 33^4 ways: not all of them are looked at */
SEC("socket")
__description("jumps ahead: one of 33 numbers out of a function, 4 times")
__success
__naked void rets_33_4_times(void)
{
	asm volatile (
	"call ret_exits; r6 = r0;"
	"call ret_exits; r7 = r0;"
	"call ret_exits; r8 = r0;"
	"call ret_exits; r9 = r0;"
	"r0 = 0;"
	"if r6 != 3 goto +1; r0 += 1;"
	"if r7 != 6 goto +1; r0 += 1;"
	"if r8 != 9 goto +1; r0 += 1;"
	"if r9 != 12 goto +1; r0 += 1;"
	"exit;"
	::: __clobber_all);
}

#define PICK_ANY(slot)					\
	"call %[bpf_get_prandom_u32];"			\
	"r0 <<= 32;"					\
	"r7 = r0;"					\
	"call %[bpf_get_prandom_u32];"			\
	"r0 |= r7;"					\
	"*(u64*)(r10 - " #slot ") = r0;"

/*
 * As in bpfj_jailer_free() of bpfjailer:
 *	for (i = 0; i < 8; i++) {
 *		p = a[i];
 *		if (!p)
 *			continue;
 *		put(p);
 *	}
 * The jump finds out about a[i] too, which nothing looks at again. All of a[] is live while
 * the loop goes on, because it is read through a pointer that moves. The states have to be
 * joined behind the jump, or there are 2^8 of them. They are, because a slot that is filled
 * through another register than r10 doesn't count as one that a jump ahead looks at.
 */
SEC("socket")
__description("0 or not: an array on the stack that a loop goes through")
__success __log_level(4)
__msg("max_states_per_insn {{[0-9]}} total_states")
__naked void array_of_nulls(void)
{
	asm volatile (
	"r6 = 0;"
	PICK_ANY(8) PICK_ANY(16) PICK_ANY(24) PICK_ANY(32)
	PICK_ANY(40) PICK_ANY(48) PICK_ANY(56) PICK_ANY(64)
	"r8 = r10;"
	"r8 += -64;"
	"r7 = 0;"
"1:"
	"r1 = *(u64*)(r8 + 0);"
	"if r1 == 0 goto 2f;"
	"r6 += 1;"
"2:"
	"r8 += 8;"
	"r7 += 1;"
	"if r7 < 8 goto 1b;"
	"r0 = r6;"
	"exit;"
	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 * As in xdp_synproxy_kern.c: hdr->ipv4 or hdr->ipv6 is there, the other one is 0.
 *	if (hdr->ipv4) ... else if (hdr->ipv6) ...
 * (0, x) and (y, 0) are not joined: (?, ?) has both, and r9 is not set.
 */
SEC("socket")
__description("0 or not: one of two things is there")
__success
__naked void one_of_two(void)
{
	asm volatile (
	PICK_ANY(8)
	"r6 = r0;"
	"r7 = 0;"
	"call %[bpf_get_prandom_u32];"
	"if r0 == 0 goto 1f;"
	"r7 = r6;"
	"r6 = 0;"
"1:"
	"r0 = 0;"
	"if r6 == 0 goto 2f;"
	"if r7 == 0 goto 2f;"
	"r0 = r9;"
"2:"
	"exit;"
	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 * As in bpf_sock.o of cilium:
 *	err = xlate();
 *	if (err == -EHOSTUNREACH || err == -ENOMEM) {
 *		bpf_set_retval(err);
 *		return 0;
 *	}
 *	return 1;
 * A compiler computes the condition where err is set. 0 and 1 touch, and it's not a jump that
 * looks at err.
 */
SEC("cgroup/connect4")
__description("jumps ahead: a flag goes together with what bpf_set_retval() takes")
__success
__naked void flag_and_retval(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"w8 = 0;"
		"w1 = 1;"
		"if r0 == 0 goto 1f;"
		"w8 = 1;"
		"w1 = -113;"
	"1:"
		"w0 = 1;"
		"if w8 == 0 goto 2f;"
		"call %[bpf_set_retval];"
		"w0 = 0;"
	"2:"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32), __imm(bpf_set_retval)
		: __clobber_all
	);
}

extern void bpf_throw(__u64 cookie) __ksym;

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	bpf_throw(0);
}

/*
 *	err = policy();
 *	count(a); count(b);
 *	if (err == -EPERM || err == -EACCES)
 *		bpf_throw(err);
 *	return 0;
 * There is no exception callback: the prog returns err. The jump that looks at the flag is too
 * far ahead to keep 0 and 1 of the flag apart.
 */
#define FLAG_AND_COOKIE(name, no, yes)			\
__arch_x86_64 __arch_arm64 __arch_s390x			\
__naked void name(void)					\
{							\
	asm volatile (					\
		"call %[bpf_get_prandom_u32];"		\
		"w8 = " #no ";"				\
		"r9 = 1;"				\
		"if r0 == 0 goto 1f;"			\
		"w8 = " #yes ";"			\
		"r9 = -13;"				\
	"1:"						\
		"call %[bpf_get_prandom_u32];"		\
		"if r0 == 0 goto +1;"			\
		"r7 = r0;"				\
		"call %[bpf_get_prandom_u32];"		\
		"if r0 == 0 goto +1;"			\
		"r7 = r0;"				\
		"if w8 == 0 goto 2f;"			\
		"r1 = r9;"				\
		"call %[bpf_throw];"			\
	"2:"						\
		"r0 = 0;"				\
		"exit;"					\
		:					\
		: __imm(bpf_get_prandom_u32), __imm(bpf_throw)	\
		: __clobber_all				\
	);						\
}

SEC("lsm/file_permission")
__description("jumps ahead: a flag goes together with what bpf_throw() takes")
__success
FLAG_AND_COOKIE(flag_and_cookie, 0, 1)

SEC("lsm/file_permission")
__description("jumps ahead: a flag goes together with what bpf_throw() does not take")
__failure __msg("At program exit the register R1 has smin=1 smax=1 should have been in [-4095, 0]")
FLAG_AND_COOKIE(flag_and_bad_cookie, 1, 0)

char buf[8];

/* As in test_seg6_loop.c without 32-bit insns: the copy is tested, the original is used */
SEC("socket")
__description("jumps ahead: the jump looks at a copy that two shifts extend")
__success
__naked void extended_copy(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r8 = 0;"
		"if r0 == 0 goto 1f;"
		"r8 = 5;"
	"1:"
		"r7 = r8;"
		"r7 <<= 32;"
		"r7 >>= 32;"
		"if r7 == 0 goto 2f;"
		"r8 += -5;"
		"r1 = %[buf] ll;"
		"r1 += r8;"
		"r0 = *(u8 *)(r1 + 0);"
		"exit;"
	"2:"
		"r0 = 0;"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32), __imm_addr(buf)
		: __clobber_all
	);
}

static __naked __noinline __used __u64 index_or_minus_1(void)
{
	asm volatile (
		"call %[bpf_get_prandom_u32];"
		"r1 = r0;"
		"r0 = 0xffffffff ll;"
		"if r1 > 7 goto 1f;"
		"r0 = r1;"
	"1:"
		"exit;"
		:
		: __imm(bpf_get_prandom_u32)
		: __clobber_all
	);
}

/*
 * As in profiler.inc.h without 32-bit insns:
 *	int index = find();
 *	if (index == -1)
 *		return 0;
 *	use(array[index]);
 * The numbers meet at the exit of find(). -1 doesn't fit in the jump, it is in a register.
 */
SEC("socket")
__description("jumps ahead: the caller looks at what the function returns")
__success
__naked void caller_looks(void)
{
	asm volatile (
		"call index_or_minus_1;"
		"r0 <<= 32;"
		"r1 = r0;"
		"r1 >>= 32;"
		"r2 = 0xffffffff ll;"
		"if r1 == r2 goto 1f;"
		"r0 s>>= 32;"
		"r1 = %[buf] ll;"
		"r1 += r0;"
		"r0 = *(u8 *)(r1 + 0);"
		"exit;"
	"1:"
		"r0 = 0;"
		"exit;"
		:
		: __imm_addr(buf)
		: __clobber_all
	);
}

/*
 * A state that arrives is compared with the states that it may be related to. Its shape tells:
 * the same pointers in the same places at the same constant offsets. These states are related
 * and don't look alike. They have to find each other.
 */
struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, char[48]);
} shape_map SEC(".maps");

/*
 * 16 states with R9 at 16 places of the stack get to "1:". With that many states at an insn
 * their shapes are looked at. Then comes one more, which @last makes of the 16th. The two are
 * related and don't have the same shape.
 */
#define SHAPE_16_PLACES	"8,16,24,32,40,48,56,64,72,80,88,96,104,112,120,128"
#define SHAPE_MANY(first, last, use)					\
	asm volatile (							\
		first							\
		"r1 = 0;"						\
		".irp off," SHAPE_16_PLACES ";"				\
		"*(u64 *)(r10 - \\off) = r1;"				\
		".endr;"						\
		"call %[bpf_get_prandom_u32];"				\
		"r6 = r0;"						\
		"call %[bpf_get_prandom_u32];"				\
		"r7 = r0;"						\
		"r7 &= 1;"						\
		".irp off," SHAPE_16_PLACES ";"				\
		"r9 = r10;"						\
		"r9 += -\\off;"						\
		"if r6 == \\off goto 1f;"				\
		".endr;"						\
		last							\
	"1:"								\
		use							\
		"r0 = *(u64 *)(r9 + 0);"				\
	"2:"								\
		"r0 = 0;"						\
		"exit;"							\
		:							\
		: __imm(bpf_get_prandom_u32), __imm(bpf_map_lookup_elem),	\
		  __imm_addr(shape_map),				\
		  __imm_const(data, offsetof(struct __sk_buff, data)),	\
		  __imm_const(data_end, offsetof(struct __sk_buff, data_end))	\
		: __clobber_all						\
	)

SEC("socket")
__description("shape: nothing in a register stands for a pointer")
__success __log_level(2)
__msg("processed 135 insns")
__naked void shape_unset(void)
{
	SHAPE_MANY("",
		   "r8 = r10;",
		   "if r7 < 2 goto +1;"
		   "r0 = *(u64 *)(r8 - 8);");
}

char _license[] SEC("license") = "GPL";
