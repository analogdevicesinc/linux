// SPDX-License-Identifier: GPL-2.0
/*
 * The jumps that tell how many iterations a loop has, in the forms that a compiler makes of
 *	for (i = first; i < N; i++)	from the last index
 *	for (i = N - 1; i >= 0; i--)	while (n--)	for (j = N - 1; j >= 0 && j < N; j--)
 *
 * Each iteration stores a byte and moves the pointer by one. The value has NR bytes. The programs
 * that load expect the count in the log. In those that have to be rejected a store is behind the
 * value at run time.
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

#define NR	2048

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, u32);
	__type(value, char[NR]);
} map SEC(".maps");

extern void bpf_rcu_read_lock(void) __ksym;
extern void bpf_rcu_read_unlock(void) __ksym;

/* BTF of a kfunc that only asm refers to is not generated */
void __kfunc_btf_root(void)
{
	bpf_rcu_read_lock();
	bpf_rcu_read_unlock();
}

#define OK(text, count)								\
	SEC("socket") __description("latch: " text) __success __log_level(2)	\
	__msg("header_count is " count)
#define BAD(text)								\
	SEC("socket") __description("latch: " text) __failure __msg("invalid access to map value")
/* the pointer goes on and on */
#define NO_COUNT(text)								\
	SEC("socket") __description("latch: " text) __failure

/* r6: the value, r7: a number that is not known. @init, then the loop. */
#define LOOP(name, init, body)							\
__naked void name(void)								\
{										\
	asm volatile ("								\
	r1 = 0;									\
	*(u32 *)(r10 - 4) = r1;							\
	r2 = r10;								\
	r2 += -4;								\
	r1 = %[map] ll;								\
	call %[bpf_map_lookup_elem];						\
	if r0 == 0 goto 9f;							\
	r6 = r0;								\
	call %[bpf_get_prandom_u32];						\
	w7 = w0;								\
" init "									\
1:	r1 = 0;									\
	*(u8 *)(r6 + 0) = r1;							\
	r6 += 1;								\
" body "									\
9:	r0 = 0;									\
	exit;									\
"	:									\
	: __imm(bpf_map_lookup_elem), __imm(bpf_get_prandom_u32), __imm_addr(map), \
	  __imm(bpf_rcu_read_lock), __imm(bpf_rcu_read_unlock)			\
	: __clobber_all);							\
}

/*
 * From the last index the latch is false in the first iteration. That is a count. The other way
 * out of the loop has one too: 4 billion.
 */
#define UP_TO_8	"if r8 >= r7 goto 9f; r8 += 1; if r8 < 8 goto 1b;"

OK("false at once, and another exit", "[0..1]")
LOOP(false_at_once, "r8 = 7; r6 += 2047;", UP_TO_8)

BAD("two iterations, and another exit")
LOOP(false_in_second, "r8 = 6; r6 += 2047;", UP_TO_8)

OK("false at once for all of a range", "[0..1]")
LOOP(false_at_once_range, "r8 = r7; r8 &= 3; r8 += 7; r6 += 2047;", UP_TO_8)

BAD("false at once for all of a range but one")
LOOP(false_at_once_range_bad, "r8 = r7; r8 &= 3; r8 += 6; r6 += 2047;", UP_TO_8)

/*
 * The latch compares pointers. One byte in front of the value is less than a byte in it,
 * although -1 is a large unsigned offset: the latch is true in the first iteration.
 */
#define PTR_UP(off)	"r8 += 1; r1 = r8; r1 += " #off "; if r1 < r9 goto 1b;"

OK("pointers", "8")
LOOP(ptr_up, "r8 = r6; r9 = r6; r9 += 8; r6 += 2040;", PTR_UP(0))

BAD("pointers, the first one is in front of the value")
LOOP(ptr_up_from_below, "r8 = r6; r9 = r6; r9 += 7; r6 += 2040;", PTR_UP(-2))

/*
 * The store behind the jump that counts is not reached in the last iteration. r9 is linear at the
 * jump, also when it moved in front of it, and has the range of one iteration less.
 */
#define AT_TOP(n)	"if w8 == " #n " goto 9f; *(u8 *)(r9 + 0) = r1; r9 += 1; w8 += 1;"
#define MOVED(n)	"r9 += 1; if w8 == " #n " goto 9f; *(u8 *)(r9 + 0) = r1; w8 += 1;"

OK("a store behind the jump that counts", "5")
LOOP(behind, "w8 = 0; r9 = r6; r9 += 2044;", AT_TOP(4) "goto 1b;")

BAD("a store behind the jump that counts, one too many")
LOOP(behind_bad, "w8 = 0; r9 = r6; r9 += 2044;", AT_TOP(5) "goto 1b;")

OK("a store behind the jump that counts, the pointer has moved", "5")
LOOP(behind_moved, "w8 = 0; r9 = r6; r9 += 2043;", MOVED(4) "goto 1b;")

BAD("a store behind the jump that counts, the pointer has moved too far")
LOOP(behind_moved_bad, "w8 = 0; r9 = r6; r9 += 2044;", MOVED(4) "goto 1b;")

/*
 * A number goes up by 3 and has moved in front of the jump. At the store it is @first or
 * @first + 3. Moved over S64_MAX it has another residue: 2^64 is not a multiple of 3. It keeps
 * the range of all three iterations then.
 */
#define NUM(by, back, first, off)						\
	"r3 = " #by " ll; r9 += r3; if w8 == 2 goto 9f; r2 = r9; r3 = " #first " ll; r2 -= r3;" \
	"r2 += r5; *(u8 *)(r2 + " #off ") = r1; r3 = " #back " ll; r9 += r3; w8 += 1; goto 1b;"
#define OVER(off)	NUM(100, -97, 0x8000000000000031, off)
#define FAR(off)	NUM(0x7fffffffffffffff, 0x8000000000000004, 0x7ffffffffffffffa, off)

OK("behind the jump that counts, a number has moved over S64_MAX", "3")
LOOP(moved_over, "w8 = 0; r5 = r6; r9 = 0x7fffffffffffffcd ll;", OVER(2041))

BAD("behind the jump that counts, a number has moved over S64_MAX, too far")
LOOP(moved_over_bad, "w8 = 0; r5 = r6; r9 = 0x7fffffffffffffcd ll;", OVER(2045))

OK("behind the jump that counts, a number has moved by S64_MAX", "3")
LOOP(moved_far, "w8 = 0; r5 = r6; r9 = -5;", FAR(2044))

BAD("behind the jump that counts, a number has moved by S64_MAX, one too far")
LOOP(moved_far_bad, "w8 = 0; r5 = r6; r9 = -5;", FAR(2045))

OK("a store behind the jump that counts two back edges", "5")
LOOP(behind_two, "w8 = 0; r9 = r6; r9 += 2044;", AT_TOP(4) "if r7 == 7 goto 1b; goto 1b;")

BAD("a store behind the jump that counts two back edges, one too many")
LOOP(behind_two_bad, "w8 = 0; r9 = r6; r9 += 2044;", AT_TOP(5) "if r7 == 7 goto 1b; goto 1b;")

/*
 * Both edges of the first jump that counts leave the loop: the one that is not taken at once, the
 * other one by way of a flag: w2 is 1 or 3. The store behind it is reached in the last iteration.
 */
#define BOTH_LEAVE								\
	"r9 += 1; w8 += 1; call %[bpf_get_prandom_u32]; if r0 == 0 goto 2f;"	\
	"if w8 >= 2 goto 3f; goto 9f;"						\
	"3: *(u8 *)(r9 + 0) = r0; w2 = 1; if r7 == 3 goto 4f; w2 = 3;"		\
	"4: if w2 == 0 goto 1b; goto 9f;"					\
	"2: if w8 < 4 goto 1b;"

OK("a store behind a jump that leaves on both edges", "[0..4]")
LOOP(behind_both, "w8 = 0; r9 = r6; r9 += 2043;", BOTH_LEAVE)

BAD("a store behind a jump that leaves on both edges, one too far")
LOOP(behind_both_bad, "w8 = 0; r9 = r6; r9 += 2044;", BOTH_LEAVE)

/* The statuses of five calls are nearer to the back edge than the jump that counts */
#define STATUS		"call %[bpf_get_prandom_u32]; if r0 == 0 goto 9f;"
#define FAR_UP(n)	"if w8 == " #n " goto 9f; w8 += 1;"			\
			STATUS STATUS STATUS STATUS STATUS "goto 1b;"

OK("behind five jumps that don't count", "[0..2048]")
LOOP(far_up, "w8 = 0;", FAR_UP(2047))

BAD("behind five jumps that don't count, one too many")
LOOP(far_up_bad, "w8 = 0;", FAR_UP(2048))

/*
 * The same in C: each frame appends up to 32 words. The index has no formula. Its range follows
 * from the number of frames: the iterations are looked at one by one.
 */
u64 word;

#define READ(dst)	bpf_probe_read_kernel(&dst, sizeof(dst), &word)
#define FRAMES(name, max)							\
int name(void *ctx)								\
{										\
	int i, j, n, idx = 0, frames = 0;					\
	u64 a, b, c, d, e, *out;						\
	u32 zero = 0;								\
										\
	out = bpf_map_lookup_elem(&map, &zero);					\
	if (!out)								\
		return -1;							\
	bpf_for(i, 0, 100) {							\
		if (frames == max)						\
			break;							\
		if (READ(a) || READ(b) || READ(c) || READ(d) || READ(e))	\
			break;							\
		n = 32 - (a & 1);						\
		for (j = 0; j < n; j++)						\
			out[idx + j] = b + c + d + e;				\
		idx += n;							\
		frames++;							\
	}									\
	return idx;								\
}

OK("frames of a stack walker", "[0..9]") __retval(256)
#ifndef __clang__
__skip("GCC unrolls the inner loop")
#endif
FRAMES(eight_frames, 8)

BAD("frames of a stack walker, one too many")
FRAMES(nine_frames, 9)

/* an int goes down to -1, or up from -N to 0: the distance is a 32-bit number */
#define DOWN_TO_M1	"w8 += -1; if w8 != -1 goto 1b;"
#define UP_TO_0		"w8 += 1; if w8 != 0 goto 1b;"

OK("down to -1 in 32 bits", "2048")
LOOP(down_to_m1, "w8 = 2047;", DOWN_TO_M1)

BAD("down to -1 in 32 bits, one too many")
LOOP(down_to_m1_bad, "w8 = 2048;", DOWN_TO_M1)

OK("up to 0 in 32 bits", "2048")
LOOP(up_to_0, "w8 = -2048;", UP_TO_0)

BAD("up to 0 in 32 bits, one too many")
LOOP(up_to_0_bad, "w8 = -2049;", UP_TO_0)

/* 'j >= 0 && j < N' is one unsigned test */
OK("down, one unsigned test", "2048")
LOOP(down_unsigned, "w8 = 2047;", "w8 += -1; if w8 < 2048 goto 1b;")

BAD("down, one unsigned test, one too many")
LOOP(down_unsigned_bad, "w8 = 2048;", "w8 += -1; if w8 < 2049 goto 1b;")

OK("down, one unsigned test, 64 bits", "2048")
LOOP(down_unsigned64, "r8 = 2047;", "r8 += -1; if r8 <= 2047 goto 1b;")

OK("down from a range, one unsigned test", "[0..2048]")
LOOP(down_unsigned_range, "if r7 > 2047 goto 9f;", "w7 += -1; if w7 < 2048 goto 1b;")

BAD("down from a range, one unsigned test, one too many")
LOOP(down_unsigned_range_bad, "if r7 > 2048 goto 9f;", "w7 += -1; if w7 < 2049 goto 1b;")

/* below 0 a 32-bit counter is 0xffffffff for a 64-bit jump */
NO_COUNT("down, a 64-bit test with a bound above 32 bits")
LOOP(down_unsigned_wide_bound, "w8 = 2047; r9 = 0x100000800 ll;", "w8 += -1; if r8 < r9 goto 1b;")

BAD("down, an unsigned and a signed 64-bit test")
LOOP(down_two_tests, "w8 = 4; r6 += 2043;",
     "w8 += -1; if r8 < 5 goto 2f; if r8 s>= 5 goto 1b; goto 9f; 2: if r8 < 5 goto 1b;")

/*
 * 'while (buf[i++] && i < N)': the jump tests r9, a copy of the counter r8 from before the
 * increment. It is made in front of the loop and at the end of each iteration.
 */
#define COPY(n)	"r8 += 1; if r9 > " #n " goto 8f; r9 = r8; if r7 != 0 goto 1b; 8:"
/* one more store, at the index that the counter is behind the loop */
#define AT_R8	"r5 += r8; *(u8 *)(r5 + 0) = r1;"

OK("a copy of the counter", "[0..2048]")
LOOP(copy, "r8 = 0; r9 = r8;", COPY(2046))

BAD("a copy of the counter, one too many")
LOOP(copy_bad, "r8 = 0; r9 = r8;", COPY(2047))

BAD("a copy of the counter, another number at the entry")
LOOP(copy_other_const, "r8 = 2047; r9 = 0; r6 += 2047;", COPY(2046))

OK("a copy of the counter, a range", "[0..2]")
LOOP(copy_range, "r5 = r6; r8 = r7; r8 &= 1; r8 += 2045; r9 = r8;", COPY(2045) AT_R8)

BAD("a copy of the counter, another number of the same range at the entry")
LOOP(copy_other_range,
     "r5 = r6; r8 = r7; r8 &= 1; r8 += 2045; r9 = r7; r9 >>= 1; r9 &= 1; r9 += 2045;",
     COPY(2045) AT_R8)

NO_COUNT("a copy of the counter that is one behind")
LOOP(copy_behind, "r8 = 0; r9 = r8;",
     "r8 += 1; if r9 > 2046 goto 9f; r9 = r8; r9 += -1; if r7 != 0 goto 1b;")

/* what the jump says about the copy holds for the counter */
OK("a copy of the counter, the jump bounds both", "[0..2048]")
LOOP(copy_bounds_both, "r5 = r6; if r7 > 2047 goto 9f; r8 = r7; r9 = r8;",
     "if r9 > 2046 goto 8f; r8 += 1; r9 = r8; if r7 != 0 goto 1b; 8:" AT_R8)

/*
 * Behind the jump that counts the counter has moved on and the copy has not: at the store r9 is
 * r8 - 1, or r8 + 1 when they go down. A test of r8 is no test of r9 there.
 */
#define NOT_EQUAL(step, leave, skip, off)					\
	"r8 += " #step "; if r9 " leave " goto 9f; if r8 " skip " goto 2f;"	\
	"r4 = r5; r4 += r9; *(u8 *)(r4 " off ") = r1; 2: r9 = r8; if r7 != 0 goto 1b;"

OK("a copy of the counter, one less behind the jump", "[0..2048]")
LOOP(copy_less, "r5 = r6; r8 = 0; r9 = r8;", NOT_EQUAL(1, "> 2046", "< 2", "- 1"))

BAD("a copy of the counter, one less behind the jump, not equal")
LOOP(copy_less_bad, "r5 = r6; r8 = 0; r9 = r8;", NOT_EQUAL(1, "> 2046", "< 2", "- 2"))

OK("a copy of the counter, one more behind the jump", "[0..2047]")
LOOP(copy_more, "r5 = r6; r8 = 2047; r9 = r8;", NOT_EQUAL(-1, "s< 2", "> 2045", "+ 1"))

BAD("a copy of the counter, one more behind the jump, not equal")
LOOP(copy_more_bad, "r5 = r6; r8 = 2047; r9 = r8;", NOT_EQUAL(-1, "s< 2", "> 2045", "+ 2"))

/*
 * r4 is what r9 was at the head. It is r9 that has to be equal to the counter when the loop is
 * entered, not r4. 64 insns more: too long for a walk.
 */
#define X4(insns)	insns insns insns insns
#define OF_COPY		"r3 = r4; r4 = r9; r8 += 1; if r8 > 2047 goto 9f;"	\
			"r3 = r4; r3 &= 4095; r3 += r5; *(u8 *)(r3 + 0) = r1;"	\
			X4(X4(X4("r3 = 0;"))) "r9 = r8; if r7 != 0 goto 1b;"

OK("a copy of a copy of the counter", "[0..2048]")
LOOP(copy_of_copy, "r5 = r6; r8 = 0; r9 = 0; r4 = 0;", OF_COPY)

BAD("a copy of what is no copy of the counter")
LOOP(copy_of_no_copy, "r5 = r6; r8 = 0; r9 = r7; r9 &= 4095; r4 = 0;", OF_COPY)

/* 'while (n--)': the jump that counts sets a flag, the jump that leaves the loop tests it */
#define FLAG(mid)	"w8 += 1; w2 = 1; if w8 == 0 goto 2f; w2 = 0; 2: " mid " if w2 != 1 goto 1b;"

OK("a flag", "[0..2048]")
LOOP(flag, "w8 = -2048;", FLAG("r3 = r6;"))

BAD("a flag, one too many")
LOOP(flag_bad, "w8 = -2049;", FLAG("r3 = r6;"))

/* A flag leaves the loop later: what is behind the jump runs in the last iteration too */
#define STORE		"*(u8 *)(r9 + 0) = r1; r9 += 1;"

OK("a flag, a store behind the jump that counts", "[0..2048]")
LOOP(flag_behind, "w8 = -2048; r9 = r6;", FLAG(STORE))

BAD("a flag, a store behind the jump that counts, one too far")
LOOP(flag_behind_bad, "w8 = -2048; r9 = r6; r9 += 1;", FLAG(STORE))

OK("a flag, a store behind it", "[0..2048]")
LOOP(flag_store, "w8 = -2048;",
     "w8 += 1; w2 = 1; *(u32 *)(r10 - 4) = r8; if w8 == 0 goto 2f; w2 = 0; 2: if w2 != 1 goto 1b;")

OK("a flag, a call behind it", "[0..2048]")
LOOP(flag_call, "w8 = -2048;",
     "w8 += 1; w9 = 1; if w8 == 0 goto 2f; w9 = 0; 2: call %[bpf_get_prandom_u32]; if w9 != 1 goto 1b;")

OK("a flag that is set in front of another jump", "[0..2048]")
LOOP(flag_far, "w8 = -2048;",
     "w8 += 1; w2 = 1; if r7 != 0 goto 3f; r3 = r6;"
     "3: if w8 == 0 goto 2f; w2 = 0; 2: if w2 != 1 goto 1b;")

/*
 * 'do { ...; more = ++i < N && cond; work; } while (more);': a compiler keeps the flag when the
 * work is more than some 10 insns. The flag has two reasons to be 0.
 */
#define MORE(count, mid)							\
	count "w9 = 1; if w7 != 0 goto 2f; w9 = 0; 2: " mid "w9 &= 1; if w9 != 0 goto 1b;"
#define UP_TO(n)	"w8 += 1; w9 = 0; if w8 > " #n " goto 2f;"
#define WORK4		"r1 = *(u8 *)(r10 - 4); r1 += 1; *(u8 *)(r10 - 4) = r1; r1 = r6;"
#define WORK20		WORK4 WORK4 WORK4 WORK4 WORK4
#define RND		"call %[bpf_get_prandom_u32];"

OK("more: work", "[0..2048]")
LOOP(more_work, "w8 = 0;", MORE(UP_TO(2047), WORK20))

BAD("more: work, one too many")
LOOP(more_work_bad, "w8 = 0;", MORE(UP_TO(2048), WORK20))

OK("more: work with a jump in it", "[0..2048]")
LOOP(more_jump, "w8 = 0;", MORE(UP_TO(2047), WORK4 "if r7 > 5 goto 4f;" WORK20 "4:"))

OK("more: work with calls in it", "[0..2048]")
LOOP(more_calls, "w8 = 0;", MORE(UP_TO(2047), RND WORK20 RND))

OK("more: the counter is spilled", "[0..2048]")
LOOP(more_spill, "r8 = 0; *(u64 *)(r10 - 16) = r8;",
     MORE("r8 = *(u64 *)(r10 - 16); w8 += 1; w9 = 0; *(u64 *)(r10 - 16) = r8;"
	  "if w8 > 2047 goto 2f;", RND "w8 = w0;" WORK20 RND))

#define NOT_TAKEN(n)	"w8 += 1; if w8 < " #n " goto 4f; w9 = 0; goto 2f; 4:"

OK("more: the edge that is not taken leaves", "[0..2048]")
LOOP(more_not_taken, "w8 = 0;", MORE(NOT_TAKEN(2048), WORK20))

BAD("more: the edge that is not taken leaves, one too many")
LOOP(more_not_taken_bad, "w8 = 0;", MORE(NOT_TAKEN(2049), WORK20))

#define JUMP		"if r7 > 5 goto 4f;" WORK4 "4:"
#define JUMP8		JUMP JUMP JUMP JUMP JUMP JUMP JUMP JUMP

OK("more: work with 40 jumps in it", "[0..2048]")
LOOP(more_jumps, "w8 = 0;", MORE(UP_TO(2047), JUMP8 JUMP8 JUMP8 JUMP8 JUMP8))

OK("more: behind five jumps that don't count", "[0..2048]")
LOOP(more_far, "w8 = 0;", MORE(UP_TO(2047), STATUS STATUS STATUS STATUS STATUS))

BAD("more: behind five jumps that don't count, one too many")
LOOP(more_far_bad, "w8 = 0;", MORE(UP_TO(2048), STATUS STATUS STATUS STATUS STATUS))

OK("more: the flag is 0 or 2 where two paths meet", "[0..2048]")
LOOP(more_even, "w8 = 0;", MORE(UP_TO(2047), "if r7 > 5 goto 4f; w9 += 2; 4:"))

OK("a flag, a jump on the way out knows a constant", "[0..2048]")
LOOP(flag_const, "w8 = -2048;",
     "w8 += 1; w2 = 1; if w8 == 0 goto 2f; w2 = 0;"
     "2: w3 = 0; if w2 != 0 goto 3f; goto 1b; 3: if w3 == 2 goto 1b;")

/* 'do { ...; err = -E2BIG; if (++i < N) err = f(); work; } while (!err);' */
#define ERR(last, other, test)							\
	"w8 += 1; w9 = " #last "; if w8 > 2047 goto 2f; w9 = " #other "; 2: " WORK20 test

OK("an error code", "[0..2048]")
LOOP(err, "w8 = 0;", ERR(-7, 0, "if w9 == 0 goto 1b;"))

OK("an error code, a signed test", "[0..2048]")
LOOP(err_signed, "w8 = 0;", ERR(-7, 0, "if w9 s>= 0 goto 1b;"))

OK("a size", "[0..2048]")
LOOP(size, "w8 = 0;", ERR(128, 0, "if w9 <= 63 goto 1b;"))

OK("a bit of a size", "[0..2048]")
LOOP(size_bit, "w8 = 0;", ERR(130, 0, "if w9 & 128 goto 9f; goto 1b;"))

/* in these the loop goes on when the counter gets to its last value */
NO_COUNT("a flag that a call overwrites")
LOOP(flag_call_r0, "w8 = -2048;",
     "w8 += 1; w0 = 1; if w8 == 0 goto 2f; w0 = 0; 2: call %[bpf_get_prandom_u32]; if w0 != 1 goto 1b;")

NO_COUNT("a flag that is set again")
LOOP(flag_set_again, "w8 = -2048;", FLAG("w2 = 0;"))

NO_COUNT("a flag that is changed")
LOOP(flag_changed, "w8 = -2048;", FLAG("w2 &= w7;"))

NO_COUNT("a flag that is set again on one path")
LOOP(flag_other_jump, "w8 = -2048;", FLAG("if r7 != 0 goto 4f; w2 = 0; 4:"))

NO_COUNT("a flag that is set again behind the jump that counts")
LOOP(flag_set_behind, "w8 = 0;",
     "w8 += 1; w3 = 0; if w8 > 2047 goto 2f; w3 = 5; goto 3f; 2: w3 = 2;"
     "3: if w3 == 0 goto 9f; if w8 <= w7 goto 1b;")

NO_COUNT("a flag that a load overwrites")
LOOP(flag_load, "w8 = -2048;", FLAG("r2 = *(u8 *)(r10 - 4);"))

NO_COUNT("the one number of its range that goes on")
LOOP(err_near, "w8 = 0;", ERR(101, w7, "if w9 == 101 goto 1b;"))

NO_COUNT("a bit that not all numbers of the range have")
LOOP(size_other_bit, "w8 = 0;", ERR(130, w7, "if w9 & 2 goto 1b;"))

NO_COUNT("a size that is changed")
LOOP(size_changed, "w8 = 0;", ERR(128, w7, "w9 &= 1; if w9 <= 63 goto 1b;"))

NO_COUNT("an error code of 64 bits, a 64-bit test")
LOOP(err_wide, "w8 = 0;",
     "w8 += 1; if w8 > 2047 goto 3f; r9 = r7; r9 <<= 32; r9 s>>= 32; goto 2f;"
     "3: r9 = -7; 2: if r9 s< 0 goto 1b;")

NO_COUNT("more: a back edge that doesn't test the flag")
LOOP(more_back_edge, "w8 = 0;",
     MORE(UP_TO(2047), "if w7 == 3 goto 5f;") "goto 9f; 5: if w8 == 2048 goto 1b;")

/* A nested loop can get may_goto. It goes where a jump of that loop goes, whatever the flag is. */
NO_COUNT("more: the way out leads through a nested loop")
LOOP(more_nested, "w8 = 0; call %[bpf_rcu_read_lock];",
     "w8 += 1; w9 = 0; if w8 > 2047 goto 2f; w9 = 1;"
     "2: if w9 == 5 goto 4f; if w9 != 0 goto 3f; if w9 == 0 goto 8f; 3: goto 2b;"
     "4: if w8 <= w7 goto 1b; 8: call %[bpf_rcu_read_unlock];")

/* May_goto at the head of the first loop goes to 7 with w9 = 1. The program gets there with 0. */
NO_COUNT("more: a flag that is a constant without may_goto")
LOOP(more_stale, "call %[bpf_rcu_read_lock];" RND "w4 = w0; w8 = 0; w9 = 0;"
     "5: if w4 == 0 goto 6f; w9 = 0; if w4 == 1 goto 7f; w9 = 1; goto 5b; 6: w9 = 1; goto 5b; 7:",
     "w5 = w9; w8 += 1; if w8 > 2047 goto 2f; w5 = 1;"
     "2: if w8 > w7 goto 8f; if w5 != 0 goto 1b; 8: call %[bpf_rcu_read_unlock];")

NO_COUNT("a flag, another way to the jump that counts")
LOOP(flag_two_ways, "w8 = -2048;",
     "w8 += 1; w2 = 0; if r7 != 0 goto 3f; w2 = 1; 3: if w8 == 0 goto 2f; w2 = 0; 2: if w2 != 1 goto 1b;")

char _license[] SEC("license") = "GPL";
