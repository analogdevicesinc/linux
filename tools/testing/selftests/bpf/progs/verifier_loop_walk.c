// SPDX-License-Identifier: GPL-2.0
/*
 * A loop that is not counted is looked at iteration by iteration for a while: some thousand insns.
 * Each such walk counts the insns that are looked at in it, not those of the walks next to it.
 */
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, u32);
	__type(value, char[2048]);
} map SEC(".maps");

/* 0 and 2 stay apart up to the jump that looks at them */
#define SET(slot)								\
	"call %[bpf_get_prandom_u32]; r1 = 0; if r0 == 0 goto +1; r1 = 2;"	\
	"*(u64 *)(r10 - " #slot ") = r1;"
#define TEST(slot)	"r1 = *(u64 *)(r10 - " #slot "); if r1 == 1 goto 9f;"
#define PAD4		"r3 = r6; r3 += 1; r4 = r3; r4 += 1;"

/*
 * 32 states enter the loop one after the other. The counter has 8 bits, which has no formula. The
 * offset is 1, 2, 4 ...: only a walk tells that it stays in the value. One walk takes 300 insns.
 */
#define WALKS(name, trips)							\
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
"	SET(16) SET(24) SET(32) SET(40) SET(48) "				\
	w8 = 0;									\
	r9 = 1;									\
1:	r1 = r6;								\
	r1 += r9;								\
	r2 = 0;									\
	*(u8 *)(r1 + 0) = r2;							\
	r9 <<= 1;								\
"	PAD4 PAD4 PAD4 PAD4 "							\
	w8 += 1;								\
	w2 = w8;								\
	w2 &= 255;								\
	if w2 < " #trips " goto 1b;						\
"	TEST(16) TEST(24) TEST(32) TEST(40) TEST(48) "				\
9:	r0 = 0;									\
	exit;									\
"	:									\
	: __imm(bpf_map_lookup_elem), __imm(bpf_get_prandom_u32), __imm_addr(map) \
	: __clobber_all);							\
}

SEC("socket")
__description("walk: 32 walks at a time")
__success
WALKS(walks_32, 11)

/*
 *	for (i = 7; i >= 0; i--) { if (i >= n) continue; a[i] = 0; }
 * A compiler tests 'i >= n' in front of the loop too and jumps into the middle of it. Such a loop
 * has no count. It's walked to its end whichever way its jumps go: no may_goto is added.
 */
SEC("socket")
__description("walk: a loop with two entries")
__success __retval(0)
__arch_x86_64
__arch_arm64
__xlated("w3 += -1")
__xlated("if w3 s< 0x0 goto pc+2")
__xlated("if w3 < w2 goto pc-6")
__xlated("goto pc-4")
__xlated("exit")
__naked void two_entries(void)
{
	asm volatile ("								\
	call %[bpf_get_prandom_u32];						\
	r2 = r0;								\
	r2 &= 7;								\
	r0 = 0;									\
	w3 = 7;									\
	if w3 >= w2 goto 2f;							\
1:	r1 = r10;								\
	r1 += r3;								\
	*(u8 *)(r1 - 8) = r0;							\
2:	w3 += -1;								\
	if w3 s< 0 goto 3f;							\
	if w3 < w2 goto 1b;							\
	goto 2b;								\
3:	exit;									\
"	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 *	for (i = 0; (u8)i < 11; i++) { if (!rnd()) break; v[1 << i] = 0; }
 * One jump that leaves the loop goes both ways in each iteration, the other one is decided in
 * each iteration: the loop counts. The walk goes on. Only a walk tells what the offset is.
 */
SEC("socket")
__description("walk: goes on while an iteration decides a jump that leaves the loop")
__success
__naked void walk_goes_on(void)
{
	asm volatile ("								\
	r1 = 0;									\
	*(u32 *)(r10 - 4) = r1;							\
	r2 = r10;								\
	r2 += -4;								\
	r1 = %[map] ll;								\
	call %[bpf_map_lookup_elem];						\
	if r0 == 0 goto 9f;							\
	r6 = r0;								\
	w8 = 0;									\
	r9 = 1;									\
1:	call %[bpf_get_prandom_u32];						\
	if r0 == 0 goto 9f;							\
	r1 = r6;								\
	r1 += r9;								\
	*(u8 *)(r1 + 0) = r0;							\
	r9 <<= 1;								\
	w8 += 1;								\
	w2 = w8;								\
	w2 &= 255;								\
	if w2 < 11 goto 1b;							\
9:	r0 = 0;									\
	exit;									\
"	:									\
	: __imm(bpf_map_lookup_elem), __imm(bpf_get_prandom_u32), __imm_addr(map) \
	: __clobber_all);
}

static __naked __noinline __used int zero_or_one(void)
{
	asm volatile ("								\
	call %[bpf_get_prandom_u32];						\
	r0 &= 1;								\
	exit;									\
"	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 *	for (i = 0; i < 2; i++) off += zero_or_one();
 * A compiler makes a flag of i: there is nothing to count. Both trips are walked although the loop
 * calls a function. Only a walk tells that off is 2 at most.
 */
#define TWO_TRIPS(name, size)							\
__naked void name(void)								\
{										\
	asm volatile ("								\
	r6 = 1;									\
	r7 = 0;									\
1:	call zero_or_one;							\
	r7 += r0;								\
	r1 = r6;								\
	r6 = 0;									\
	if r1 != 0 goto 1b;							\
	r1 = r10;								\
	r1 += r7;								\
	r0 = 0;									\
	*(u8 *)(r1 - " #size ") = r0;						\
	exit;									\
"	::: __clobber_all);							\
}

SEC("socket")
__description("walk: two trips of a loop with a call")
__success __retval(0)
TWO_TRIPS(two_trips, 3)

SEC("socket")
__description("walk: two trips of a loop with a call, off by one")
__failure __msg("invalid variable-offset write to stack R1")
TWO_TRIPS(two_trips_off_by_one, 2)

/*
 *	while (rnd()) n++;
 * The second iteration decides nothing. The walk gives up at once: less than 100 insns.
 */
SEC("socket")
__description("walk: gives up when an iteration decides no jump that leaves the loop")
__success __log_level(4) __msg("processed {{..}} insns")
__naked void walk_gives_up(void)
{
	asm volatile ("								\
	r6 = 0;									\
1:	call %[bpf_get_prandom_u32];						\
	if r0 == 0 goto 2f;							\
	r6 += 1;								\
	goto 1b;								\
2:	r0 = 0;									\
	exit;									\
"	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 *	do { n++; more = rnd(); ... } while (more);
 * A compiler keeps the flag. States with 0 and with 1 are not joined in front of its test, so the
 * test is decided in each of them. That decides nothing: the walk gives up as above.
 */
SEC("socket")
__description("walk: gives up when only a flag that went both ways decides the jump")
__success __log_level(4) __msg("processed {{..}} insns")
__naked void walk_gives_up_flag(void)
{
	asm volatile ("								\
	r6 = 0;									\
1:	call %[bpf_get_prandom_u32];						\
	r6 += 1;								\
	w7 = 1;									\
	if r0 != 0 goto 2f;							\
	w7 = 0;									\
2:	w7 &= 1;								\
	if w7 != 0 goto 1b;							\
	r0 = 0;									\
	exit;									\
"	:: __imm(bpf_get_prandom_u32) : __clobber_all);
}

/*
 *	for (off = 1; !done; off <<= 1) { v[off] = 0; done = off > 512; if (rnd()) n++; }
 * done is known in every iteration. Another flag went both ways, and was tested before it: the
 * walk goes on. Only a walk tells what the offset is.
 */
SEC("socket")
__description("walk: goes on when the flag that leaves the loop is known")
__success
__naked void walk_goes_on_flag(void)
{
	asm volatile ("								\
	r1 = 0;									\
	*(u32 *)(r10 - 4) = r1;							\
	r2 = r10;								\
	r2 += -4;								\
	r1 = %[map] ll;								\
	call %[bpf_map_lookup_elem];						\
	if r0 == 0 goto 9f;							\
	r6 = r0;								\
	r9 = 1;									\
1:	r1 = r6;								\
	r1 += r9;								\
	r2 = 0;									\
	*(u8 *)(r1 + 0) = r2;							\
	w8 = 1;									\
	if r9 > 512 goto 2f;							\
	w8 = 0;									\
2:	r9 <<= 1;								\
	call %[bpf_get_prandom_u32];						\
	w7 = 1;									\
	if r0 != 0 goto 3f;							\
	w7 = 0;									\
3:	if w7 == 0 goto 4f;							\
	r2 = 0;									\
4:	if w8 == 0 goto 1b;							\
9:	r0 = 0;									\
	exit;									\
"	:									\
	: __imm(bpf_map_lookup_elem), __imm(bpf_get_prandom_u32), __imm_addr(map) \
	: __clobber_all);
}

static __naked __noinline __used int long_loop(void)
{
	asm volatile ("								\
	w3 = 0;									\
1:	w3 += 1;								\
	w2 = w3;								\
	w2 &= 0xffff;								\
	if w2 < 5000 goto 1b;							\
	r0 = 0;									\
	*(u64 *)(r1 + 0) = r0;							\
	exit;									\
"	::: __clobber_all);
}

/*
 * The walk doesn't get to the end of the loop. The function is looked at three times, for three
 * pointers. There is no walk the second and the third time: less than 10000 insns in all.
 */
SEC("socket")
__description("walk: no other try for a loop that is too long")
__success __log_level(4) __msg("processed {{....}} insns")
__naked void walk_fails_once(void)
{
	asm volatile ("								\
	r1 = r10;								\
	r1 += -8;								\
	call long_loop;								\
	r1 = r10;								\
	r1 += -16;								\
	call long_loop;								\
	r1 = r10;								\
	r1 += -24;								\
	call long_loop;								\
	r0 = 0;									\
	exit;									\
"	::: __clobber_all);
}

char _license[] SEC("license") = "GPL";
