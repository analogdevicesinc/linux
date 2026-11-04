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
__failure __msg("the call stack of {{3[23]}} frames")
int global_func3(struct __sk_buff *skb)
{
	return f32(skb);
}
