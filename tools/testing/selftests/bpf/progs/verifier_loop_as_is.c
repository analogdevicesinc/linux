// SPDX-License-Identifier: GPL-2.0
/*
 * The state that enters a loop that is not counted is left as it is. So is the state that ends
 * a walk through the iterations of such a loop. What stands for many iterations is a join of states
 * that were seen, not a guess that is written into one of them.
 */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "../../../include/linux/filter.h"
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, __u32);
	__type(value, __u64[9]);
} map SEC(".maps");

#define LOOKUP(reg)							\
	"r1 = 0;"							\
	"*(u32 *)(r10 - 4) = r1;"					\
	"r2 = r10;"							\
	"r2 += -4;"							\
	"r1 = %[map] ll;"						\
	"call %[bpf_map_lookup_elem];"					\
	#reg " = r0;"							\
	"if r0 == 0 goto 9f;"

#define END								\
"9:"									\
	"r0 = 0;"							\
	"exit;"								\
	:								\
	: __imm(bpf_map_lookup_elem), __imm_addr(map)			\
	: __clobber_all

/* END with more operands. may_goto leaves the loop: @off insns ahead */
#define END_WITH(more)							\
"9:"									\
	"r0 = 0;"							\
	"exit;"								\
	:								\
	: __imm(bpf_map_lookup_elem), __imm_addr(map), more		\
	: __clobber_all
#define MAY_GOTO(off) __imm_insn(may_goto, BPF_RAW_INSN(BPF_JMP | BPF_JCOND, 0, 0, off, 0))

#define OK(text)	SEC("socket") __description("as is: " text) __success __retval(0)
#define BAD(text)	SEC("socket") __description("as is: " text) __failure

/*
 *	do { n += 3; } while (v[0]);
 * Two iterations are looked at as they are. The second one ends the walk: its jump goes both
 * ways. Its state is joined with the one before, nothing stops n, so n is any number.
 * What comes back next is within that. The number of insns tells that there is no fourth pass.
 * The loop gets may_goto when the walk ends, and all is done again: two times 24 insns.
 */
OK("the state that ends a walk is joined with the one before")
__log_level(1) __msg("processed 48 insns")
__naked void two_passes(void)
{
	asm volatile (
	LOOKUP(r7)
	"r6 = 0;"
"1:"
	"r6 += 3;"
	"r1 = *(u32 *)(r7 + 0);"
	"if r1 != 0 goto 1b;"
	"r6 &= 63;"
	"r7 += r6;"
	"r0 = *(u8 *)(r7 + 8);"
	END);
}

/*
 *	x = (v[0] & 1) << 62; do { x |= x >> 1; if (x > 1 << 62) break; } while (v[1]);
 * The range at the head is 0..1 << 62 in every pass, the lower half is 0, and one more bit is not
 * known: bit 61, 60 .. 32. That is not followed bit by bit, 30 passes and more in a nest of loops:
 * the known bits are what the range says after the first one.
 */
OK("known bits are not lost one by one")
__log_level(1) __msg("processed 74 insns")
__naked void bits_one_by_one(void)
{
	asm volatile (
	LOOKUP(r7)
	"r6 = *(u64 *)(r7 + 0);"
	"r6 &= 1;"
	"r6 <<= 62;"
	"r8 = 1;"
	"r8 <<= 62;"
"1:"
	"r1 = r6;"
	"r1 >>= 1;"
	"r6 |= r1;"
	"if r6 > r8 goto 9f;"
	"r1 = *(u64 *)(r7 + 8);"
	"if r1 != 0 goto 1b;"
	"r6 >>= 60;"
	"r7 += r6;"
	"r0 = *(u8 *)(r7 + 0);"
	END);
}

/*
 *	if (x < 1000000) return; do { if (x && !v[2]) x--; } while (v[1]);
 * x is 1000000..U64_MAX, then 999999..U64_MAX. As a signed number it's S64_MIN..S64_MAX all the
 * time: no signed bound moves, and the range does, a million times if nothing is done about it.
 */
OK("a range moves and its signed bounds do not")
__log_level(1) __msg("processed 68 insns")
__naked void unsigned_bound_moves(void)
{
	asm volatile (
	LOOKUP(r7)
	"r6 = *(u64 *)(r7 + 0);"
	"if r6 < 1000000 goto 9f;"
"1:"
	"if r6 == 0 goto 2f;"
	"r1 = *(u64 *)(r7 + 16);"
	"if r1 != 0 goto 2f;"
	"r6 += -1;"
"2:"
	"r1 = *(u64 *)(r7 + 8);"
	"if r1 != 0 goto 1b;"
	"r6 &= 7;"
	"r7 += r6;"
	"r0 = *(u8 *)(r7 + 0);"
	END);
}

/*
 *	n = 3; while (v[0]) { if (v[0] != 1 && n) n--; }
 * n goes from 3 to 0. The bound that moves at the head of the loop is put at 0, the nearest of the
 * fixed stops, and that holds. Without it n goes on to fall, and n & 63 is up to 63.
 */
OK("a number that falls stops at 0")
__naked void stops_at_0(void)
{
	asm volatile (
	LOOKUP(r6)
	"r7 = 3;"
"1:"
	"r1 = *(u64 *)(r6 + 0);"
	"if r1 == 0 goto 3f;"
	"if r1 == 1 goto 2f;"
	"if r7 == 0 goto 2f;"
	"r7 += -1;"
"2:"
	"goto 1b;"
"3:"
	"r7 &= 63;"
	"r6 += r7;"
	"r0 = *(u8 *)(r6 + 68);"
	END);
}

/*
 *	sum = 0; n = 0; max = v[1]; if (max > 64) return;
 *	while (can_loop) { @test sum += (u8)v[0]; if (!(u8)v[0]) break; }
 *	@use ((u8 *)v)[sum]
 * The loop has may_goto, @off insns long, so it is neither counted nor walked. The number of insns
 * tells how many times it is looked at.
 */
#define SUM(name, test, add, off, use)					\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r7)							\
	"r6 = 0;"							\
	"r9 = 0;"							\
	"r8 = *(u64 *)(r7 + 8);"					\
	"if r8 s> 64 goto 9f;"						\
"1:"									\
	".8byte %[may_goto];"						\
	test								\
	"r1 = *(u8 *)(r7 + 0);"						\
	add								\
	"if r1 != 0 goto 1b;"						\
	use								\
	"r7 += r6;"							\
	"r0 = *(u8 *)(r7 + 0);"						\
	END_WITH(MAY_GOTO(off)));					\
}

/* No jump of the loop depends on the sum, no stop holds it: two passes, with 0 and any number */
OK("a sum is any number at once")
__log_level(1) __msg("processed 25 insns")
SUM(sum_two_passes, "", "r6 += r1;", 3, "r6 &= 63;")

BAD("a sum is any number at once, and no index")
__msg("register with unbounded min value is not allowed")
SUM(sum_two_passes_bad, "", "r6 += r1;", 3, "")

/* sum = x + sum + x; if (x != 3) n -= x; */
OK("a sum has many forms")
__log_level(1) __msg("processed 36 insns")
SUM(sum_forms, "",
    "r2 = r1; r2 += r6; r6 = r2; r6 += r1; if r1 == 3 goto +1; r9 -= r1;", 8, "r6 += r9; r6 &= 63;")

/* A test of the low half is sufficient behind the loop */
OK("a sum in 32 bits is any number of 32 bits at once")
__log_level(1) __msg("processed 25 insns")
SUM(sum32_two_passes, "", "w6 += w1;", 3, "if w6 > 63 goto 9f;")

/* 0..S32_MAX at the head, 0..63 behind the jump. Of any number the jump leaves S64_MIN..63. */
OK("a sum that a jump compares with a register is not forgotten")
SUM(sum_below_max, "if r6 s>= r8 goto 9f; r2 = r7; r2 += r6; r0 = *(u8 *)(r2 + 0);",
    "r6 += r1;", 7, "r6 &= 63;")

/* The same jump in a loop in the loop: do { if (sum >= max) return; } while (((u8 *)v)[sum]); */
OK("a sum that a jump of a nested loop compares with a register is not forgotten")
SUM(sum_below_max_nested,
    "2: if r6 s>= r8 goto 9f; r2 = r7; r2 += r6; r0 = *(u8 *)(r2 + 0); if r0 != 0 goto 2b;",
    "r6 += r1;", 8, "r6 &= 63;")

/* 0, 0..S32_MAX, 0..U32_MAX, any number. 0..S64_MAX would be one more pass, and wraps. */
OK("a sum that gets past U32_MAX is any number")
__log_level(1) __msg("processed 37 insns")
SUM(sum_past_u32_max, "if r6 == 77 goto +0;", "r6 += r1;", 4, "r6 &= 63;")

/* 0, S32_MIN..0, any number */
OK("a sum that gets past S32_MIN is any number")
__log_level(1) __msg("processed 32 insns")
SUM(sum_past_s32_min, "if r6 == 77 goto +0;", "r6 -= r1;", 4, "r6 &= 63;")

/*
 *	while (can_loop) { x = v[1] >> 1; if (x > sum) sum = x; if (!(u8)v[0]) break; }
 * 0..S64_MAX is what arrives. No guess has failed: the lower bound stays, the signed test needs it.
 */
OK("the largest of numbers up to S64_MAX is not negative")
SUM(largest, "r2 = *(u64 *)(r7 + 8); r2 >>= 1; if r2 <= r6 goto +1; r6 = r2;", "", 6,
    "if r6 s> 63 goto 9f;")

/*
 *	n = 0; while (v[0]) { if (v[0] != 1 && n < 8) n++; }
 *	((u8 *)v)[n + off]
 * n grows in some iterations only, and no edge of the jump that looks at it leaves the loop. n is
 * an index behind the loop, where no jump has cut it: what counts is its range at the head.
 * 7 goes on to the add, so 8 comes back at most.
 * @load, @store: n lives in a register, or in 32 bits of a stack slot.
 */
#define GROWS(name, load, store, off)					\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r6)							\
	"r7 = 0;"							\
	store								\
"1:"									\
	"r1 = *(u64 *)(r6 + 0);"					\
	"if r1 == 0 goto 3f;"						\
	"if r1 == 1 goto 2f;"						\
	load								\
	"if r7 >= 8 goto 2f;"						\
	"r7 += 1;"							\
	store								\
"2:"									\
	"goto 1b;"							\
"3:"									\
	load								\
	"r6 += r7;"							\
	"r0 = *(u8 *)(r6 + " #off ");"					\
	END);								\
}

OK("a jump of the loop stops a number at 8")
GROWS(stops_at_8, "", "", 63)

BAD("a jump of the loop stops a number at 8, one byte too far")
__msg("invalid access to map value, value_size=72 off=72 size=1")
GROWS(stops_at_8_bad, "", "", 64)

OK("a jump of the loop stops a number in a stack slot at 8")
GROWS(slot_stops_at_8, "r7 = *(u32 *)(r10 - 16);", "*(u32 *)(r10 - 16) = r7;", 63)

/*
 *	n = 0; for (i = 0; i < 100000; i++) { if (v[0] != 1 && n < 8) n++; }
 *	((u8 *)v)[n + off]
 * The loop has a count and is too long for a walk. The count tells what i is and that there is
 * no need for may_goto. It has nothing to say about n, which is joined and widened as above.
 */
#define COUNTED_GROWS(name, off)					\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r6)							\
	"r7 = 0;"							\
	"r8 = 0;"							\
"1:"									\
	"r1 = *(u64 *)(r6 + 0);"					\
	"if r1 == 1 goto 2f;"						\
	"if r7 >= 8 goto 2f;"						\
	"r7 += 1;"							\
"2:"									\
	"r8 += 1;"							\
	"if r8 < 100000 goto 1b;"					\
	"r6 += r7;"							\
	"r0 = *(u8 *)(r6 + " #off ");"					\
	END);								\
}

OK("a jump of a counted loop stops a number at 8")
__xlated("...")
__xlated("r8 += 1")
__xlated("if r8 < 0x186a0 goto pc-{{[0-9]+}}")
COUNTED_GROWS(counted_stops_at_8, 63)

BAD("a jump of a counted loop stops a number at 8, one byte too far")
__msg("invalid access to map value, value_size=72 off=72 size=1")
COUNTED_GROWS(counted_stops_at_8_bad, 64)

/* 8 comes back to the head of the inner loop, and goes from there to the head of the outer one */
OK("a jump of the inner loop stops a number at 8")
int inner_loop_stops_at_8(void *ctx)
{
	__u32 key = 0;
	__u64 n = 0, *v;
	int i, j;

	v = bpf_map_lookup_elem(&map, &key);
	if (!v)
		return 0;
	bpf_for(i, 0, 100) {
		bpf_for(j, 0, 100) {
			if (*(volatile __u64 *)v == 1)
				continue;
			if (n < 8)
				v[n++] = 0;
		}
	}
	return v[n];
}

/*
 *	err = 0; n = 0; @init
 *	while (can_loop) { n += 3; @body if (!v[0]) break; }
 *	@test never();
 * The loop has may_goto, @off insns long, so it is neither counted nor walked.
 */
#define ERR_LOOP(name, init, body, off, test)				\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r7)							\
	"r6 = 0;"							\
	"r8 = 0;"							\
	init								\
"1:"									\
	".8byte %[may_goto];"						\
	"r8 += 3;"							\
	body								\
	"r1 = *(u64 *)(r7 + 0);"					\
	"if r1 != 0 goto 1b;"						\
	test								\
	"r0 = r5;"							\
"2:"									\
	"r8 &= 7;"							\
	"r7 += r8;"							\
	"r0 = *(u8 *)(r7 + 0);"						\
	END_WITH(MAY_GOTO(off)));					\
}

/*
 *	if (v[1]) err = 8; ... if (err != 0 && err != 8) never();
 * Two states enter the loop. What comes around it with 0 is for the state with 0. Next to 8 it
 * looks like a number that grew.
 */
#define TWO_ENTRIES(name, write)					\
ERR_LOOP(name, "r1 = *(u64 *)(r7 + 8); if r1 == 0 goto 1f; r6 = 8;",	\
	 write, 4, "if r6 == 0 goto 2f; if r6 == 8 goto 2f;")

OK("what comes around a loop is for the state that it comes from")
TWO_ENTRIES(two_entries, "r1 = r8;")

BAD("what comes around a loop is for the state that it comes from, the loop writes")
__msg("R5 !read_ok")
TWO_ENTRIES(two_entries_bad, "r6 = r8;")

#define COUNT(name, write)						\
static __naked __noinline __used int name(void)				\
{									\
	asm volatile (							\
	"r1 = *(u64 *)(r2 + 0);"					\
	"r1 += 1;"							\
	"*(u64 *)(r2 + 0) = r1;"					\
	write								\
	"r0 = 0;"							\
	"exit;"								\
	::: __clobber_all);						\
}

COUNT(count, "")
COUNT(count_and_write, "*(u64 *)(r2 + 8) = r1;")

/* The same with bpf_loop(100, @cb, &n, 0) in place of the loop. err is in the slot above n. */
#define TWO_CALLS(name, cb)						\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r7)							\
	"r1 = 0;"							\
	"*(u64 *)(r10 - 16) = r1;"					\
	"*(u64 *)(r10 - 8) = r1;"					\
	"r1 = *(u64 *)(r7 + 8);"					\
	"if r1 == 0 goto 1f;"						\
	"r1 = 8;"							\
	"*(u64 *)(r10 - 8) = r1;"					\
"1:"									\
	"r1 = 100;"							\
	"r2 = " #cb " ll;"						\
	"r3 = r10;"							\
	"r3 += -16;"							\
	"r4 = 0;"							\
	"call %[bpf_loop];"						\
	"r6 = *(u64 *)(r10 - 8);"					\
	"if r6 == 0 goto 9f;"						\
	"if r6 == 8 goto 9f;"						\
	"r0 = r5;"							\
	END_WITH(__imm(bpf_loop)));					\
}

OK("what a callback comes back with is for the state that it comes from")
TWO_CALLS(two_calls, count)

BAD("what a callback comes back with is for the state that it comes from, it writes")
__msg("R5 !read_ok")
TWO_CALLS(two_calls_bad, count_and_write)

/*
 *	for (i = n; i >= 0; i--) sum += v[i];
 * A compiler counts in bytes and leaves the loop one element below the array:
 *	off = n * 8; do { sum += *(u64 *)(v + off); off -= @step; } while (off != -8);
 * n is a range, so the loop is not counted. Only -8 leaves it. The offset doesn't get past -8
 * because it is a multiple of 8, and that has to be known at the head of the loop.
 */
#define DOWN(name, step)						\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r7)							\
	"r6 = *(u32 *)(r7 + 64);"					\
	"if r6 > 7 goto 9f;"						\
	"r6 <<= 3;"							\
"1:"									\
	"r2 = r7;"							\
	"r2 += r6;"							\
	"r0 = *(u64 *)(r2 + 0);"					\
	"r6 += -" #step ";"						\
	"if r6 != -8 goto 1b;"						\
	END);								\
}

OK("down by 8 bytes, left at -8")
DOWN(down_by_8, 8)

/* 16, 4, -8 is fine, 8, -4 is not */
BAD("down by 12 bytes, left at -8")
__msg("min value is negative")
DOWN(down_by_12, 12)

static __naked __noinline __used void nothing(void)
{
	asm volatile ("r0 = 0; exit;" ::: __clobber_all);
}

/*
 *	if (v[0]) { n = 7; have = 1; }
 *	nothing();
 *	i = 0; do { if (v[0] != 3) i++; } while (i < 8 && v[0] != 2);
 *	if (have) return n;
 * (have = 0, n is not set) and (have = 1, n = 7) wait for the function. What comes out of it goes
 * to both at once, and the insn behind the call is the head of the loop: two walks begin, and no
 * insn is looked at in between. The state that ends one of them is joined with a state of
 * that walk. With a state of the other one it would be (have = 0 or 1, n is not set).
 */
#define TWO_WALKS(name, test)						\
__naked void name(void)							\
{									\
	asm volatile (							\
	LOOKUP(r8)							\
	"r9 = 0;"							\
	"r1 = *(u32 *)(r8 + 0);"					\
	"if r1 != 0 goto 1f;"						\
	"r7 = 0;"							\
	"goto 2f;"							\
"1:"									\
	"r6 = 7;"							\
	"r7 = 1;"							\
"2:"									\
	"call nothing;"							\
"3:"									\
	"r1 = *(u32 *)(r8 + 0);"					\
	"if r1 == 3 goto +1;"						\
	"r9 += 1;"							\
	"if r9 > 7 goto 4f;"						\
	"if r1 == 1 goto 3b;"						\
	"if r1 != 2 goto 3b;"						\
"4:"									\
	test								\
	"r0 = r6;"							\
	"exit;"								\
	END);								\
}

OK("two walks begin at the same time")
TWO_WALKS(two_walks_at_once, "if r7 == 0 goto 9f;")

/* n is read when it is not set */
BAD("two walks begin at the same time, the register is not set")
__msg("R6 !read_ok")
TWO_WALKS(two_walks_at_once_bad, "if r7 != 0 goto 9f;")

char _license[] SEC("license") = "GPL";
