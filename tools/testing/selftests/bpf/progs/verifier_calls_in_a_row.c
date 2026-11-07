// SPDX-License-Identifier: GPL-2.0
/* A static subprog that is called from many places */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

/* gcc makes a copy of a function for an argument that is the same constant in most calls */
#if defined(__GNUC__) && !defined(__clang__)
#define __noclone __attribute__((noclone))
#else
#define __noclone
#endif

volatile int sel;
char arr[64];

#define SITES_4(call, n)  call(n) call(n + 1) call(n + 2) call(n + 3)
#define SITES_20(call)    SITES_4(call, 0) SITES_4(call, 4) SITES_4(call, 8) \
			  SITES_4(call, 12) SITES_4(call, 16)

/*
 * The states of the 20 calls are joined at the entry of the subprog one by one: x grows. The state
 * of the first call stays apart from them, however many joins it takes.
 *
 * kind 1: idx is an index. kind 3: idx is something else. 2 is not there.
 */
static __noinline __noclone int use(int kind, unsigned int idx, int x)
{
	if (kind == 1)
		return arr[idx] + x;
	return idx + x;
}

SEC("socket")
__description("20 calls in a row, a number that a jump tests")
__success
int many_sites_gap(void *ctx)
{
	int r = use(1, 5, 100);

#define USE(n) if (sel == n) r += use(3, 1000, n);
	SITES_20(USE)
	return r;
}

/* The second call gets what the first call returns */
static __noinline __noclone int inc(int x)
{
	return x + 1;
}

SEC("socket")
__description("x = inc(x), two times in a row")
__success __retval(2)
int inc_two_times(void *ctx)
{
	int x = 0;

	x = inc(x);
	x = inc(x);
	return x;
}

char four[4] SEC(".data.four") = { 10, 11, 12, 13 };

/*
 * For the caller the argument is r6 + 10. The subprog has no r6: r1 is linked to nothing there.
 * r1 is 6 .. 9 and r2 is r1 + 4, that is 10 .. 13, not r1 + 4 - 10.
 */
#define FOUR_AT_R1_PLUS_4(off)			\
	"r2 = r1;"				\
	"r2 += 4;"				\
	"r0 = 0;"				\
	"if r1 < 6 goto 1f;"			\
	"if r1 > 9 goto 1f;"			\
	"r3 = %[four] ll;"			\
	"r3 += r2;"				\
	"r0 = *(u8 *)(r3 " #off ");"		\
"1:"						\
	"exit;"

static __naked __noinline __used int four_at_0_to_3(void)
{
	asm volatile (FOUR_AT_R1_PLUS_4(- 10) :: __imm_addr(four) : __clobber_all);
}

static __naked __noinline __used int four_at_10_to_13(void)
{
	asm volatile (FOUR_AT_R1_PLUS_4(+ 0) :: __imm_addr(four) : __clobber_all);
}

#define CALL_WITH_R6_PLUS_10(func)		\
	"call %[bpf_get_prandom_u32];"		\
	"r6 = r0;"				\
	"r1 = r6;"				\
	"r1 += 10;"				\
	"call " #func ";"			\
	"r0 += r6;"				\
	"exit;"

SEC("socket")
__description("the argument is linked to a register of the caller")
__success
__naked void arg_is_linked(void)
{
	asm volatile (CALL_WITH_R6_PLUS_10(four_at_0_to_3)
		      :: __imm(bpf_get_prandom_u32) : __clobber_all);
}

SEC("socket")
__description("the argument is linked to a register of the caller, out of bounds")
__failure __msg("invalid access to map value, value_size=4 off=10 size=1")
__naked void arg_is_linked_bad(void)
{
	asm volatile (CALL_WITH_R6_PLUS_10(four_at_10_to_13)
		      :: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 * r1 is 1, 3 or 5, and the jumps look at it: the states of three calls stay apart at the first
 * insn. Those of 1 and 3 meet behind the second jump. What becomes of them meets that of 5 behind
 * the first one. From there on one state stands for the three, and what comes out is for
 * all three calls.
 * Returns r2 + 3, r2 + 2, r2.
 */
static __naked __noinline __used int three_kinds(void)
{
	asm volatile (
	"if r1 == 5 goto 2f;"
	"if r1 == 3 goto 1f;"
	"r2 += 1;"
"1:"
	"r2 += 2;"
"2:"
	"r0 = r2;"
	"exit;"
	::: __clobber_all);
}

#define CALL(kind, v)				\
	"r1 = " #kind ";"			\
	"r2 = " #v ";"				\
	"call three_kinds;"

#define LOAD					\
	"r1 = %[four] ll;"			\
	"r1 += r0;"				\
	"r0 = *(u8 *)(r1 + 0);"

/* The three calls wait at the same time */
#define BRANCHES(v1, load1, v3, load3, v5, load5)	\
	asm volatile (				\
	"r6 = *(u32 *)(r1 + %[len]);"		\
	"if r6 == 1 goto 1f;"			\
	"if r6 == 3 goto 3f;"			\
	CALL(5, v5)				\
	load5					\
	"exit;"					\
"1:"						\
	CALL(1, v1)				\
	load1					\
	"exit;"					\
"3:"						\
	CALL(3, v3)				\
	load3					\
	"exit;"					\
	:					\
	: __imm_addr(four),			\
	  __imm_const(len, offsetof(struct __sk_buff, len))	\
	: __clobber_all)

SEC("tc")
__description("three calls wait, their states meet in two steps")
__success __retval(10)
__naked void three_calls_wait(void)
{
	BRANCHES(0, LOAD, 0, LOAD, 0, LOAD);
}

SEC("tc")
__description("three calls wait, their states meet in two steps, the first one gets 7")
__failure __msg("invalid access to map value, value_size=4 off=7 size=1")
__naked void three_calls_wait_bad_1(void)
{
	BRANCHES(4, LOAD, 0, "", 0, "");
}

#define ADD	LOAD "r6 += r0;"

/* A call is done with before the next one is made */
#define ROW(k1, v1, load1, k2, v2, load2, k3, v3, load3)	\
	asm volatile (				\
	"r6 = 0;"				\
	CALL(k1, v1)				\
	load1					\
	CALL(k2, v2)				\
	load2					\
	CALL(k3, v3)				\
	load3					\
	"r0 = r6;"				\
	"exit;"					\
	:					\
	: __imm_addr(four)			\
	: __clobber_all)

SEC("tc")
__description("three calls in a row, their states meet in two steps")
__success __retval(35)
__naked void three_calls_in_a_row(void)
{
	ROW(1, 0, ADD, 3, 0, ADD, 5, 0, ADD);
}

SEC("tc")
__description("three calls in a row, the second one gets 6")
__failure __msg("invalid access to map value, value_size=4 off=6 size=1")
__naked void three_calls_in_a_row_bad_2(void)
{
	ROW(1, 0, "", 3, 4, ADD, 5, 0, "");
}

char _license[] SEC("license") = "GPL";
