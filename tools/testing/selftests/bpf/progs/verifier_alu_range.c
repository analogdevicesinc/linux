// SPDX-License-Identifier: GPL-2.0
/*
 * The count of a shift, or a divisor, is a range and not a constant:
 *	x >> [a, b]	[xmin >> b, xmax >> a]
 *	x << [a, b]	[xmin << a, xmax << b], if no bit is shifted out
 *	x s>> [a, b]	a larger count brings x closer to 0, or to -1
 *	x / [a, b]	[xmin / b, xmax / a], and x / 0 = 0
 *	x % [a, b]	[0, b - 1], x if it is below a, and x % 0 = x
 * Each program that has to be rejected gets to the bad read at run time.
 */
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

#define OK(text)	SEC("socket") __description("alu range: " text) __success __log_level(2)
#define BAD(text)	SEC("socket") __description("alu range: " text) __failure \
			__msg("invalid read from stack")

/* @reg is lo .. lo + mask */
#define RANGE(reg, lo, mask)						\
	"call %[bpf_get_prandom_u32];"					\
	#reg " = r0;"							\
	#reg " &= " #mask ";"						\
	#reg " += " #lo ";"

/* The read is not allowed */
#define UNLESS(cond)	"if " cond " goto +1; r0 = *(u64 *)(r10 + 8);"

/* r6 is the second operand. r7 and r8 are x. */
#define ALU(src_lo, src_mask, x_lo, x_mask, insns)			\
	asm volatile (							\
	RANGE(r6, src_lo, src_mask)					\
	RANGE(r7, x_lo, x_mask)						\
	"r8 = r7;"							\
	insns								\
	"r0 = 0;"							\
	"exit;"								\
	:: __imm(bpf_get_prandom_u32) : __clobber_all)

OK("[128, 255] >> [2, 5]")
__msg("r7 >>= r6 {{.*}} R7=scalar(smin=umin=smin32=umin32=4,smax=umax=smax32=umax32=63,var_off=(0x0; 0x3f))")
__msg("w8 >>= w6 {{.*}} R8=scalar(smin=umin=smin32=umin32=4,smax=umax=smax32=umax32=63,var_off=(0x0; 0x3f))")
__naked void rsh(void)
{
	ALU(2, 3, 128, 127, "r7 >>= r6; w8 >>= w6;"
	    UNLESS("r7 >= 4") UNLESS("r7 <= 63") UNLESS("w8 >= 4") UNLESS("w8 <= 63"));
}

BAD("255 >> 2 is 63")
__naked void rsh_max(void)
{
	ALU(2, 3, 128, 127, "r7 >>= r6;" UNLESS("r7 <= 62"));
}

OK("[3, 10] << [1, 4]")
__msg("r7 <<= r6 {{.*}} R7=scalar(smin=umin=smin32=umin32=6,smax=umax=smax32=umax32=160,var_off=(0x0; 0xfe))")
__msg("w8 <<= w6 {{.*}} R8=scalar(smin=umin=smin32=umin32=6,smax=umax=smax32=umax32=160,var_off=(0x0; 0xfe))")
__naked void lsh(void)
{
	ALU(1, 3, 3, 7, "r7 <<= r6; w8 <<= w6;"
	    UNLESS("r7 >= 6") UNLESS("r7 <= 160") UNLESS("w8 >= 6") UNLESS("w8 <= 160"));
}

BAD("10 << 4 is 160, 32 bits")
__naked void lsh32_max(void)
{
	ALU(1, 3, 3, 7, "w7 <<= w6;" UNLESS("w7 <= 159"));
}

BAD("4 << 62 is 0")
__naked void lsh_overflow(void)
{
	ALU(59, 3, 4, 1, "r7 <<= r6;" UNLESS("r7 != 0"));
}

BAD("4 << 30 is 0, 32 bits")
__naked void lsh32_overflow(void)
{
	ALU(27, 3, 4, 1, "w7 <<= w6;" UNLESS("w7 != 0"));
}

OK("[-128, -65] s>> [2, 5], [-128, 127] s>> [2, 5]")
__msg("r7 s>>= r6 {{.*}} R7=scalar(smin=smin32=-32,smax=smax32=-3,")
__msg("w8 s>>= w6 {{.*}} R8=scalar(smin=0,smax=umax=0xffffffff,smin32=-32,smax32=31,")
__naked void arsh(void)
{
	ALU(2, 3, -128, 63, "r7 s>>= r6;" UNLESS("r7 s>= -32") UNLESS("r7 s<= -3")
	    RANGE(r8, -128, 255) "w8 s>>= w6;" UNLESS("w8 s>= -32") UNLESS("w8 s<= 31"));
}

BAD("-65 s>> 5 is -3")
__naked void arsh_max(void)
{
	ALU(2, 3, -128, 63, "r7 s>>= r6;" UNLESS("r7 s<= -4"));
}

OK("[128, 255] / [2, 5], [128, 255] / [0, 3]")
__msg("r7 /= r6 {{.*}} R7=scalar(smin=umin=smin32=umin32=25,smax=umax=smax32=umax32=127,var_off=(0x0; 0x7f))")
__msg("w8 /= w6 {{.*}} R8=scalar(smin=umin=smin32=umin32=25,smax=umax=smax32=umax32=127,var_off=(0x0; 0x7f))")
__msg("r9 /= r6 {{.*}} R9=scalar(smin=smin32=0,smax=umax=smax32=umax32=255,var_off=(0x0; 0xff))")
__naked void div(void)
{
	ALU(2, 3, 128, 127, "r9 = r7; r7 /= r6; w8 /= w6;"
	    UNLESS("r7 >= 25") UNLESS("r7 <= 127") UNLESS("w8 >= 25") UNLESS("w8 <= 127")
	    "r6 += -2; r9 /= r6;" UNLESS("r9 <= 255"));
}

BAD("128 / 5 is 25, 32 bits")
__naked void div32_min(void)
{
	ALU(2, 3, 128, 127, "w7 /= w6;" UNLESS("w7 >= 26"));
}

BAD("x / 0 is 0")
__naked void div_by_zero(void)
{
	ALU(0, 3, 128, 127, "r7 /= r6;" UNLESS("r7 >= 42"));
}

OK("[128, 255] % [2, 5], [128, 255] % [256, 259], [128, 255] % [0, 3]")
__msg("r7 %= r6 {{.*}} R7=scalar(smin=smin32=0,smax=umax=smax32=umax32=4,var_off=(0x0; 0x7))")
__msg("w8 %= w6 {{.*}} R8=scalar(smin=smin32=0,smax=umax=smax32=umax32=4,var_off=(0x0; 0x7))")
__msg("r9 %= r6 {{.*}} R9=scalar(smin=umin=smin32=umin32=128,smax=umax=smax32=umax32=255,var_off=(0x80; 0x7f))")
__msg("w9 %= w6 {{.*}} R9=scalar(smin=smin32=0,smax=umax=smax32=umax32=255,var_off=(0x0; 0xff))")
__naked void mod(void)
{
	ALU(2, 3, 128, 127, "r9 = r7; r7 %%= r6; w8 %%= w6;" UNLESS("r7 <= 4") UNLESS("w8 <= 4")
	    "r6 += 254; r9 %%= r6;" UNLESS("r9 >= 128") UNLESS("r9 <= 255")
	    "r6 += -256; w9 %%= w6;" UNLESS("w9 <= 255"));
}

BAD("129 % 5 is 4")
__naked void mod_max(void)
{
	ALU(2, 3, 128, 127, "r7 %%= r6;" UNLESS("r7 <= 3"));
}

BAD("255 % 255 is 0")
__naked void mod_same(void)
{
	ALU(255, 3, 128, 127, "r7 %%= r6;" UNLESS("r7 >= 128"));
}

BAD("x % 0 is x, 32 bits")
__naked void mod32_by_zero(void)
{
	ALU(0, 3, 128, 127, "w7 %%= w6;" UNLESS("w7 <= 2"));
}

/* The cpu takes the count modulo 64 or 32, and a signed division can change the sign */
OK("not known: a count of 64 or 32, a signed division")
__msg("r7 >>= r6 {{.*}} R7=scalar()")
__msg("w8 >>= w6 {{.*}} R8=scalar()")
__msg("r7 s/= r6 {{.*}} R7=scalar()")
__msg("w8 s%= w6 {{.*}} R8=scalar()")
__naked void not_known(void)
{
	ALU(63, 1, 128, 127, "r7 >>= r6; r6 += -32; w8 >>= w6;"
	    RANGE(r7, 128, 127) "r8 = r7; r7 s/= r6; w8 s%%= w6;");
}

BAD("x >> 64 is x")
__naked void rsh_64(void)
{
	ALU(63, 1, 128, 127, "r7 >>= r6;" UNLESS("r7 == 0"));
}

char _license[] SEC("license") = "GPL";
