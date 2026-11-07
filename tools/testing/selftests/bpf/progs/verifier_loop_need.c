// SPDX-License-Identifier: GPL-2.0
/*
 * A counted loop changes a number in a way that has no formula:
 *	for (i = 0; i < 1000; i++) x = x * 3 + 1;
 * The verifier can go through the iterations one by one to find out what x is. It does so only
 * if something ahead looks at x: a jump, an access to memory, a call. Otherwise what comes back
 * to the head of the loop is joined with what entered and widened: the loop is looked at
 * a few times, however many iterations it has.
 *
 * What looks at a number is found ahead of the main pass, and it is a guess. A use that is missed
 * rejects a program that would load after a walk. It never loads a program that is not safe.
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

/* a map value of 64 bytes that holds nothing but numbers */
u64 vals[8];

struct {
	__uint(type, BPF_MAP_TYPE_RINGBUF);
	__uint(max_entries, 4096);
} ringbuf SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 1);
} arena SEC(".maps");

#define OK(text)	SEC("socket") __description("need: " text) __success
#define BAD(text)	SEC("socket") __description("need: " text) __failure
#define NOT_WALKED	__log_level(2) __msg("nothing looks at it")
#define WALKED		__log_level(2) __not_msg("nothing looks at it")

/* The read is not allowed */
#define UNLESS(cond)	"if " cond " goto +1; r0 = *(u64 *)(r10 + 8);"

/* r6 is 1 or 2, and is multiplied by 3 in each of @n iterations. @use comes behind the loop. */
#define TIMES_3_IN(init, n, use)					\
	asm volatile (							\
	init								\
	"call %[bpf_get_prandom_u32];"					\
	"r6 = r0;"							\
	"r6 &= 1;"							\
	"r6 += 1;"							\
	"r7 = 0;"							\
"1:	r6 *= 3;"							\
	"r7 += 1;"							\
	"if r7 < " #n " goto 1b;"					\
	use								\
	"exit;"								\
	:: __imm(bpf_get_prandom_u32), __imm_addr(vals) : __clobber_all)
#define TIMES_3(n, use)	TIMES_3_IN("", n, use "r0 = 0;")

OK("a number that is only stored is not followed")
NOT_WALKED __msg("processed 17 insns")
__naked void stored_only(void)
{
	TIMES_3(1000, "*(u64 *)(r10 - 8) = r6;");
}

OK("a jump looks at the number")
WALKED
__naked void jump(void)
{
	TIMES_3(5, UNLESS("r6 <= 486"));
}

BAD("a jump looks at the number, which is too large")
__msg("invalid read from stack")
__naked void jump_too_large(void)
{
	TIMES_3(5, UNLESS("r6 <= 485"));
}

OK("the number is an offset in a map value that holds numbers")
WALKED
__naked void value_offset(void)
{
	/* 27 or 54 */
	TIMES_3(3, "r1 = %[vals] ll; r1 += r6; r0 = *(u8 *)(r1 + 9);");
}

#define VALUE_AT(reg, off)	"r1 = %[vals] ll; r1 += " #reg "; r0 = *(u8 *)(r1 + " #off ");"

__naked __noinline __used
static unsigned long value_at_plus_9(void)
{
	asm volatile ("r2 = r1;" VALUE_AT(r2, 9) "exit;" :: __imm_addr(vals) : __clobber_all);
}

OK("the number is an offset in a map value in a function")
WALKED
__naked void value_offset_argument(void)
{
	TIMES_3(3, "r1 = r6; call value_at_plus_9;");
}

__naked __noinline __used
static unsigned long times_27(void)
{
	TIMES_3_IN("", 3, "r0 = r6;");
}

OK("what a function returns is an offset in a map value")
WALKED
__naked void value_offset_returned(void)
{
	asm volatile ("call times_27;" VALUE_AT(r0, 9) "exit;"
		      :: __imm_addr(vals) : __clobber_all);
}

#if defined(__clang_major__) && __clang_major__ >= 23
struct pair {
	__u64 lo;	/* r0 */
	__u64 hi;	/* r2 */
};

__naked __noinline __used
static struct pair zero_and_times_27(void)
{
	TIMES_3_IN("", 3, "r0 = 0; r2 = r6;");
}

OK("the second half of what a function returns is an offset in a map value")
__load_if_JITed() WALKED
__naked void value_offset_returned_in_r2(void)
{
	asm volatile ("call zero_and_times_27;" VALUE_AT(r2, 9) "exit;"
		      :: __imm_addr(vals) : __clobber_all);
}
#endif

__naked __noinline __used
static unsigned long times_27_there(void)
{
	TIMES_3_IN("r8 = r1;", 3, "*(u64 *)(r8 + 0) = r6; r0 = 0;");
}

#define TIMES_27_AT_FP_8	"r1 = r10; r1 += -8; call times_27_there; r2 = *(u64 *)(r10 - 8);"

OK("a function stores the number in the stack of its caller")
WALKED
__naked void stored_for_the_caller(void)
{
	asm volatile (TIMES_27_AT_FP_8 VALUE_AT(r2, 9) "exit;"
		      :: __imm_addr(vals) : __clobber_all);
}

OK("the number is an offset in the stack")
WALKED
__naked void stack_offset(void)
{
	/* fp-9 or fp-18 */
	TIMES_3(2, "*(u64 *)(r10 - 8) = r7; *(u64 *)(r10 - 16) = r7; *(u64 *)(r10 - 24) = r7;"
		   "r6 = -r6; r1 = r10; r1 += r6; r0 = *(u8 *)(r1 + 0);");
}

__naked __noinline __used
static unsigned long times_9(void)
{
	TIMES_3_IN("", 2, "r0 = r6;");
}

/* fp-9 or fp-18, plus @off */
#define STACK_AT_MINUS_R0(off)						\
	"*(u64 *)(r10 - 8) = r0; *(u64 *)(r10 - 16) = r0; *(u64 *)(r10 - 24) = r0;"	\
	"r0 = -r0; r1 = r10; r1 += r0; r0 = *(u8 *)(r1 + " #off ");"

OK("what a function returns is an offset in the stack")
WALKED
__naked void stack_offset_returned(void)
{
	asm volatile ("call times_9;" STACK_AT_MINUS_R0(0) "exit;" ::: __clobber_all);
}

/* r6 is 1, and is multiplied by 3 in each of 2 iterations */
#define NINE(use)							\
	asm volatile (							\
	"r6 = 1;"							\
	"r7 = 0;"							\
"1:	r6 *= 3;"							\
	"r7 += 1;"							\
	"if r7 < 2 goto 1b;"						\
	use								\
	"r0 = 0;"							\
	"exit;"								\
	:: __imm(bpf_ringbuf_reserve), __imm(bpf_ringbuf_discard),	\
	   __imm_addr(ringbuf) : __clobber_all)

OK("the number is part of a size that has to be a constant")
WALKED
__naked void constant_argument(void)
{
	NINE("r1 = %[ringbuf] ll; r2 = 7; r2 += r6; r3 = 0; call %[bpf_ringbuf_reserve];"
	     "if r0 == 0 goto +3; r1 = r0; r2 = 0; call %[bpf_ringbuf_discard];");
}

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
/* Nothing checks where a pointer to arena points to */
OK("an address in arena is not followed")
NOT_WALKED
__naked void arena_address(void)
{
	asm volatile (
	"r1 = %[arena] ll;"
	"r6 = 1;"
	"r7 = 0;"
"1:	r1 = addr_space_cast(r6, 0x0, 0x1);"
	"r0 = *(u64 *)(r1 + 0);"
	"r6 *= 3;"
	"r7 += 1;"
	"if r7 < 1000 goto 1b;"
	"r0 = 0;"
	"exit;"
	:: __imm_addr(arena) : __clobber_all);
}
#endif

/*
 *	p = NULL; for (i = 0; i < 3; i++) { if (have) n += *p; p = &val; have = 1; }
 * No jump and no add looks at p.
 */
OK("the base of a load is 0 in the first iteration")
WALKED
__naked void base_of_a_load(void)
{
	asm volatile (
	"r6 = %[vals] ll;"
	"r7 = 0;"
	"r8 = 0;"
	"r9 = 0;"
"1:	if r9 == 0 goto 2f;"
	"r0 = *(u64 *)(r8 + 0);"
"2:	r8 = r6;"
	"r9 = 1;"
	"r7 += 1;"
	"if r7 < 3 goto 1b;"
	"r0 = 0;"
	"exit;"
	:: __imm_addr(vals) : __clobber_all);
}

/* r8 goes on by 0 or 8 bytes. Nothing tells how far but the iterations. */
OK("a pointer that moves is followed")
WALKED
__naked void pointer(void)
{
	asm volatile (
	"r8 = %[vals] ll;"
	"r7 = 0;"
"1:	r1 = r7;"
	"r1 &= 1;"
	"r1 <<= 3;"
	"r8 += r1;"
	"r7 += 1;"
	"if r7 < 14 goto 1b;"
	"r0 = *(u64 *)(r8 + 0);"
	"r0 = 0;"
	"exit;"
	:: __imm_addr(vals) : __clobber_all);
}

/* The sets have 64 slots of the stack. What is deeper is followed. */
OK("a jump looks at a slot below fp-512")
__load_if_large_stack()
WALKED
__naked void deep_slot(void)
{
	asm volatile (
	"r6 = 2;"
	"*(u64 *)(r10 - 520) = r6;"
	"r7 = 0;"
"1:	r6 = *(u64 *)(r10 - 520);"
	"r6 *= 3;"
	"*(u64 *)(r10 - 520) = r6;"
	"r7 += 1;"
	"if r7 < 5 goto 1b;"
	"r6 = *(u64 *)(r10 - 520);"
	UNLESS("r6 == 486")
	"r0 = 0;"
	"exit;"
	::: __clobber_all);
}

/* The iterator is not a number. It is at its end after 3 of 5 iterations. */
OK("a slot that holds an iterator is followed")
WALKED
__naked void iterator_slot(void)
{
	asm volatile (
	"r1 = r10;"
	"r1 += -8;"
	"r2 = 0;"
	"r3 = 3;"
	"call %[bpf_iter_num_new];"
	"r7 = 0;"
"1:	r1 = r10;"
	"r1 += -8;"
	"call %[bpf_iter_num_next];"
	"r7 += 1;"
	"if r7 < 5 goto 1b;"
	"r1 = r10;"
	"r1 += -8;"
	"call %[bpf_iter_num_destroy];"
	"r0 = 0;"
	"exit;"
	:: __imm(bpf_iter_num_new), __imm(bpf_iter_num_next), __imm(bpf_iter_num_destroy)
	: __clobber_all);
}

/*
 *	for (i = 0; i < n; i++) { if (i != skip) sum += prev; prev = (x ^ i) * 3; }
 * prev is not set when the loop is entered. A compiler doesn't set what is not read.
 */
#define SUM_OF_PREV(skip, n, use)					\
	asm volatile (							\
	"call %[bpf_get_prandom_u32];"					\
	"r6 = r0;"							\
	"r7 = 0;"							\
	"r9 = 0;"							\
"1:	if r7 == " #skip " goto 2f;"					\
	"r9 += r8;"							\
"2:	r8 = r6;"							\
	"r8 ^= r7;"							\
	"r8 *= 3;"							\
	"r7 += 1;"							\
	"if r7 < " #n " goto 1b;"					\
	use								\
	"r0 = 0;"							\
	"exit;"								\
	:: __imm(bpf_get_prandom_u32), __imm_addr(vals) : __clobber_all)

OK("what is not set at the entry is set from the second iteration on")
NOT_WALKED __msg("processed 22 insns")
__naked void set_in_the_first_iteration(void)
{
	SUM_OF_PREV(0, 40, "*(u64 *)(r10 - 8) = r9;");
}

BAD("what is not set at the entry is read in the first iteration")
__msg("R8 !read_ok")
__naked void read_in_the_first_iteration(void)
{
	SUM_OF_PREV(1, 40, "*(u64 *)(r10 - 8) = r9;");
}

/*
 *	for (i = 0; i < 4; i++) { if (ok) sum += v[x]; x = *v; ok = x <= max; }
 * x is loaded in every iteration and ok goes with it. One state for all iterations has both values
 * of ok for each x.
 */
#define LOADED_WITH_FLAG(max)						\
	asm volatile (							\
	"r6 = %[vals] ll;"						\
	"r7 = 0;"							\
	"r8 = 0;"							\
	"r9 = 0;"							\
"1:	if r9 == 0 goto 2f;"						\
	"r1 = r6;"							\
	"r1 += r8;"							\
	"r0 = *(u8 *)(r1 + 0);"						\
"2:	r8 = *(u64 *)(r6 + 0);"						\
	"r9 = 0;"							\
	"if r8 > " #max " goto 3f;"					\
	"r9 = 1;"							\
"3:	r7 += 1;"							\
	"if r7 < 4 goto 1b;"						\
	"r0 = 0;"							\
	"exit;"								\
	:: __imm_addr(vals) : __clobber_all)

OK("a short loop is walked for the flag that goes with what it loads")
__naked void loaded_with_flag(void)
{
	LOADED_WITH_FLAG(63);
}

BAD("the flag is set for a number that is too large")
__msg("invalid access to map value")
__naked void loaded_with_wrong_flag(void)
{
	LOADED_WITH_FLAG(64);
}

/*
 *	for (i = 0; i < 4; i++) { sum += v[x]; x = table[i]; }
 * The table is in .rodata: in a walk each x is a known number.
 */
const u8 in_vals[4] = { 8, 16, 56, 255 }, past_vals[4] = { 8, 16, 57, 0 };

#define INDEX_FROM_RODATA(table)					\
	asm volatile (							\
	"r6 = %[vals] ll;"						\
	"r7 = 0;"							\
	"r8 = 0;"							\
"1:	r1 = r6;"							\
	"r1 += r8;"							\
	"r0 = *(u64 *)(r1 + 0);"					\
	"r1 = %[" #table "] ll;"					\
	"r1 += r7;"							\
	"r8 = *(u8 *)(r1 + 0);"						\
	"r7 += 1;"							\
	"if r7 < 4 goto 1b;"						\
	"r0 = 0;"							\
	"exit;"								\
	:: __imm_addr(vals), __imm_addr(table) : __clobber_all)

OK("a short loop is walked for the numbers that it loads from .rodata")
__naked void index_from_rodata(void)
{
	INDEX_FROM_RODATA(in_vals);
}

BAD("a number from .rodata is too large")
__msg("invalid access to map value")
__naked void index_from_rodata_too_large(void)
{
	INDEX_FROM_RODATA(past_vals);
}

/* A slot holds a leftover pointer. The loop fills it with what it loads. It's only copied later. */
OK("no walk for what the loop loads if nothing looks at it")
__log_level(2) __msg("processed 17 insns")
__naked void leftover_in_a_slot(void)
{
	asm volatile (
	"r6 = %[vals] ll;"
	"*(u64 *)(r10 - 8) = r6;"
	"r7 = 0;"
"1:	if r7 >= 40 goto 2f;"
	"r1 = *(u64 *)(r6 + 0);"
	"*(u64 *)(r10 - 8) = r1;"
	"r7 += 1;"
	"goto 1b;"
"2:	r1 = *(u64 *)(r10 - 8);"
	"*(u64 *)(r6 + 0) = r1;"
	"r0 = 0;"
	"exit;"
	:: __imm_addr(vals) : __clobber_all);
}

/* What a call returns is made anew in every iteration, like what is loaded */
OK("a long loop carries what a call returns")
int call_in_a_long_loop(void *ctx)
{
	u64 sum = 0;
	u32 x = 0;
	int i;

	for (i = 0; i < 20000; i++) {
		sum += vals[x];
		x = bpf_get_prandom_u32() & 7;
	}
	return sum;
}

/* BTF FUNC records are not generated for kfuncs referenced from inline assembly */
void __kfunc_btf_root(void)
{
	bpf_iter_num_new(0, 0, 0);
	bpf_iter_num_next(0);
	bpf_iter_num_destroy(0);
}

char _license[] SEC("license") = "GPL";
