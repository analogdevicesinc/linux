// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (c) 2020 Facebook */
#include <stddef.h>
#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"

static __attribute__ ((noinline))
int f1(struct __sk_buff *skb)
{
	return skb->len;
}

static __attribute__ ((noinline))
int f2(int val, struct __sk_buff *skb)
{
	return f1(skb) + val;
}

static __attribute__ ((noinline))
int f3(int val, struct __sk_buff *skb, int var)
{
	return f2(var, skb) + val;
}

static __attribute__ ((noinline))
int f4(struct __sk_buff *skb)
{
	return f3(1, skb, 2);
}

static __attribute__ ((noinline))
int f5(struct __sk_buff *skb)
{
	return f4(skb);
}

static __attribute__ ((noinline))
int f6(struct __sk_buff *skb)
{
	return f5(skb);
}

static __attribute__ ((noinline))
int f7(struct __sk_buff *skb)
{
	return f6(skb);
}

static __attribute__ ((noinline))
int f8(struct __sk_buff *skb)
{
	return f7(skb);
}

static __attribute__ ((noinline))
int f9(struct __sk_buff *skb)
{
	return f8(skb);
}

static __attribute__ ((noinline))
int f10(struct __sk_buff *skb)
{
	return f9(skb);
}

static __attribute__ ((noinline))
int f11(struct __sk_buff *skb)
{
	return f10(skb);
}

static __attribute__ ((noinline))
int f12(struct __sk_buff *skb)
{
	return f11(skb);
}

static __attribute__ ((noinline))
int f13(struct __sk_buff *skb)
{
	return f12(skb);
}

static __attribute__ ((noinline))
int f14(struct __sk_buff *skb)
{
	return f13(skb);
}

static __attribute__ ((noinline))
int f15(struct __sk_buff *skb)
{
	return f14(skb);
}

static __attribute__ ((noinline))
int f16(struct __sk_buff *skb)
{
	return f15(skb);
}

static __attribute__ ((noinline))
int f17(struct __sk_buff *skb)
{
	return f16(skb);
}

static __attribute__ ((noinline))
int f18(struct __sk_buff *skb)
{
	return f17(skb);
}

static __attribute__ ((noinline))
int f19(struct __sk_buff *skb)
{
	return f18(skb);
}

static __attribute__ ((noinline))
int f20(struct __sk_buff *skb)
{
	return f19(skb);
}

static __attribute__ ((noinline))
int f21(struct __sk_buff *skb)
{
	return f20(skb);
}

static __attribute__ ((noinline))
int f22(struct __sk_buff *skb)
{
	return f21(skb);
}

static __attribute__ ((noinline))
int f23(struct __sk_buff *skb)
{
	return f22(skb);
}

static __attribute__ ((noinline))
int f24(struct __sk_buff *skb)
{
	return f23(skb);
}

static __attribute__ ((noinline))
int f25(struct __sk_buff *skb)
{
	return f24(skb);
}

static __attribute__ ((noinline))
int f26(struct __sk_buff *skb)
{
	return f25(skb);
}

static __attribute__ ((noinline))
int f27(struct __sk_buff *skb)
{
	return f26(skb);
}

static __attribute__ ((noinline))
int f28(struct __sk_buff *skb)
{
	return f27(skb);
}

static __attribute__ ((noinline))
int f29(struct __sk_buff *skb)
{
	return f28(skb);
}

static __attribute__ ((noinline))
int f30(struct __sk_buff *skb)
{
	return f29(skb);
}

static __attribute__ ((noinline))
int f31(struct __sk_buff *skb)
{
	return f30(skb);
}

static __attribute__ ((noinline))
int f32(struct __sk_buff *skb)
{
	return f31(skb);
}

SEC("tc")
__success
__load_if_JITed()
int global_func3_32_frames(struct __sk_buff *skb)
{
	return f31(skb);
}

#define PTR_FRAME(name, next)			\
static __attribute__ ((noinline))		\
long name(long *p)				\
{						\
	*p += 1;				\
	return next(p) + 1;			\
}

static __attribute__ ((noinline))
long p31(long *p)
{
	/* volatile: the compiler must not pass the value in place of the pointer */
	return *(volatile long *)p;
}

PTR_FRAME(p30, p31)
PTR_FRAME(p29, p30)
PTR_FRAME(p28, p29)
PTR_FRAME(p27, p28)
PTR_FRAME(p26, p27)
PTR_FRAME(p25, p26)
PTR_FRAME(p24, p25)
PTR_FRAME(p23, p24)
PTR_FRAME(p22, p23)
PTR_FRAME(p21, p22)
PTR_FRAME(p20, p21)
PTR_FRAME(p19, p20)
PTR_FRAME(p18, p19)
PTR_FRAME(p17, p18)
PTR_FRAME(p16, p17)
PTR_FRAME(p15, p16)
PTR_FRAME(p14, p15)
PTR_FRAME(p13, p14)
PTR_FRAME(p12, p13)
PTR_FRAME(p11, p12)
PTR_FRAME(p10, p11)
PTR_FRAME(p9, p10)
PTR_FRAME(p8, p9)
PTR_FRAME(p7, p8)
PTR_FRAME(p6, p7)
PTR_FRAME(p5, p6)
PTR_FRAME(p4, p5)
PTR_FRAME(p3, p4)
PTR_FRAME(p2, p3)
PTR_FRAME(p1, p2)
PTR_FRAME(p0, p1)

/*
 * What a subprog does with the stack of its callers is found for every chain of calls that
 * hands a pointer to the stack down. The function that does it calls itself for every frame.
 */
SEC("tc")
__success __retval(60)
__load_if_JITed()
int global_func3_32_frames_stack_pointer(struct __sk_buff *skb)
{
	long x = 0;

	return p1(&x);
}

/* That function sees the chain before the main pass does. */
SEC("tc")
__failure
__msg("p30() passes a pointer to the stack to p31()")
__msg("the call stack of 33 frames is too deep")
int global_func3_33_frames_stack_pointer(struct __sk_buff *skb)
{
	long x = 0;

	return p0(&x);
}

SEC("tc")
__failure __msg("the call stack of {{3[23]}} frames")
int global_func3(struct __sk_buff *skb)
{
	return f32(skb);
}
