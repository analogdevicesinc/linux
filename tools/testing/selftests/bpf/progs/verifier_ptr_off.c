// SPDX-License-Identifier: GPL-2.0
/* One of two constants is added to a pointer. A compiler makes one add and one load of it. */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_core_read.h>
#include "bpf_misc.h"
#include "bpf_kfuncs.h"

struct with_timer {
	int before[4];
	struct bpf_timer timer;
	int after[4];
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct with_timer);
} timers SEC(".maps");

__u64 in_bss[8];

#define __imm_pkt								\
	__imm_const(data, offsetof(struct __sk_buff, data)),			\
	__imm_const(data_end, offsetof(struct __sk_buff, data_end))

/*
 *	len = v6 ? 40 : 20;
 *	if (data + len > data_end) return 0;
 *	if (v6) use(data[39]);
 * 1 and 2 touch, so nothing else keeps the states apart.
 */
SEC("tc")
__description("offset of a pointer: the length of a header goes with its kind")
__success __retval(0)
__naked void pkt_len_and_kind(void)
{
	asm volatile (
	"r6 = r1;"
	"call %[bpf_get_prandom_u32];"
	"r8 = 1;"
	"r9 = 20;"
	"if r0 == 0 goto 1f;"
	"r8 = 2;"
	"r9 = 40;"
"1:	r1 = *(u32 *)(r6 + %[data]);"
	"r3 = *(u32 *)(r6 + %[data_end]);"
	"r2 = r1;"
	"r2 += r9;"
	"if r2 > r3 goto 2f;"
	"if r8 != 2 goto 2f;"
	"r0 = *(u8 *)(r1 + 39);"
"2:	r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32), __imm_pkt
	: __clobber_all);
}

/* The same with c->skb->data: the context comes out of a struct on the stack */
SEC("tc")
__description("offset of a pointer: the packet of a context that was in a struct")
__success __retval(0)
__naked void pkt_of_ctx_in_struct(void)
{
	asm volatile (
	"*(u64 *)(r10 - 8) = r1;"
	"r7 = r10;"
	"r7 += -8;"
	"call %[bpf_get_prandom_u32];"
	"r8 = 1;"
	"r9 = 20;"
	"if r0 == 0 goto 1f;"
	"r8 = 2;"
	"r9 = 40;"
"1:	r6 = *(u64 *)(r7 + 0);"
	"r1 = *(u32 *)(r6 + %[data]);"
	"r3 = *(u32 *)(r6 + %[data_end]);"
	"r2 = r1;"
	"r2 += r9;"
	"if r2 > r3 goto 2f;"
	"if r8 != 2 goto 2f;"
	"r0 = *(u8 *)(r1 + 39);"
"2:	r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32), __imm_pkt
	: __clobber_all);
}

/* data - 1 or data + 30000. A range of the two plus 1 would vouch for 30001 bytes behind data. */
SEC("tc")
__description("offset of a pointer: one of two places, none of them in the packet")
__failure __msg("invalid access to packet, off=30000 size=1, R1(id=0,off=30000,r=0)")
__naked void pkt_two_places(void)
{
	asm volatile (
	"r6 = r1;"
	"call %[bpf_get_prandom_u32];"
	"r1 = *(u32 *)(r6 + %[data]);"
	"r3 = *(u32 *)(r6 + %[data_end]);"
	"r2 = r1;"
	"if r0 == 0 goto 1f;"
	"r2 += 30000;"
	"goto 2f;"
"1:	r2 += -1;"
"2:	r2 += 1;"
	"if r2 > r3 goto 3f;"
	"r0 = *(u8 *)(r1 + 30000);"
"3:	r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32), __imm_pkt
	: __clobber_all);
}

/* A range of offsets would go over the timer */
SEC("socket")
__description("offset of a pointer: in front of a timer or behind it")
__success __retval(0)
__naked void around_timer(void)
{
	asm volatile (
	"r1 = 0;"
	"*(u32 *)(r10 - 4) = r1;"
	"r2 = r10;"
	"r2 += -4;"
	"r1 = %[timers] ll;"
	"call %[bpf_map_lookup_elem];"
	"if r0 == 0 goto 2f;"
	"r6 = r0;"
	"call %[bpf_get_prandom_u32];"
	"r1 = %[before];"
	"if r0 == 0 goto 1f;"
	"r1 = %[after];"
"1:	r6 += r1;"
	"r0 = *(u32 *)(r6 + 0);"
"2:	r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32), __imm(bpf_map_lookup_elem), __imm_addr(timers),
	  __imm_const(before, offsetof(struct with_timer, before[3])),
	  __imm_const(after, offsetof(struct with_timer, after[0]))
	: __clobber_all);
}

/* Nothing looks at the offset but for its ends: the states are joined */
SEC("socket")
__description("offset of a pointer: a map value that holds nothing but numbers")
__success __retval(0) __log_level(2)
__msg("from 4 to 5: R1=scalar({{.*}}smin32=umin32=8,smax=umax=smax32=umax32=48{{.*}}) R10=fp0")
__msg("max_states_per_insn 1 ")
__naked void plain_value(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"r1 = 8;"
	"r2 = 0;"
	"if r0 == 0 goto 1f;"
	"r1 = 48;"
"1:	r6 = %[in_bss] ll;"
	"r6 += r1;"
	"r0 = *(u64 *)(r6 + 0);"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32), __imm_addr(in_bss)
	: __clobber_all);
}

const volatile struct {
	int size;
	int kind;
} cfgs[2] = { { 8, 1 }, { 32, 2 } };

/* What is at a known offset of a read-only map is known */
SEC("tc")
__description("offset of a pointer: one of two elements of a read-only map")
__success
int size_of_two(struct __sk_buff *skb)
{
	char buf[32];

	return bpf_skb_load_bytes(skb, 0, buf, (skb->mark ? &cfgs[0] : &cfgs[1])->size);
}

/* The prototypes of the two kfuncs say void *. The verifier makes a pointer to a struct of it. */
SEC("tc")
__description("offset of a pointer: one of two fields behind bpf_core_cast()")
__success
int field_behind_core_cast(struct __sk_buff *skb)
{
	struct task_struct *task = bpf_core_cast((void *)bpf_get_current_task(), struct task_struct);

	return skb->mark ? task->pid : task->tgid;
}

SEC("tc")
__description("offset of a pointer: one of two fields behind bpf_cast_to_kern_ctx()")
__success
int field_behind_kern_ctx(struct __sk_buff *ctx)
{
	struct sk_buff *skb = bpf_cast_to_kern_ctx(ctx);

	return ctx->mark ? skb->len : skb->data_len;
}

static __naked __noinline __used void random(void)
{
	asm volatile (
	"call %[bpf_get_prandom_u32];"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32)
	: __clobber_all);
}

/*
 * Two states enter the loop, one with each offset. r9 is not what it was when they come around.
 * Each of them is joined with the state at the head of the loop that it comes from.
 */
SEC("tp_btf/sched_switch")
__description("offset of a pointer: two offsets enter a loop")
__success
__naked void two_offsets_in_a_loop(void)
{
	asm volatile (
	"r6 = *(u64 *)(r1 + 8);"
	"call %[bpf_get_prandom_u32];"
	"r7 = %[pid];"
	"if r0 == 0 goto 1f;"
	"r7 = %[tgid];"
"1:	r9 = 0;"
"2:	if r9 == 7 goto 3f;"
	"r1 = r6;"
	"r1 += r7;"
	"r0 = *(u32 *)(r1 + 0);"
	"call random;"
	"r9 = r0;"
	"goto 2b;"
"3:	r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32),
	  __imm_const(pid, offsetof(struct task_struct, pid)),
	  __imm_const(tgid, offsetof(struct task_struct, tgid))
	: __clobber_all);
}

/* The offset is what changes in the loop: it's forgotten, and the loop ends */
SEC("tc")
__description("offset of a pointer: the offset changes in a loop")
__success
__naked void offset_changes_in_a_loop(void)
{
	asm volatile (
	"r6 = r1;"
	"r7 = 0;"
"1:	r1 = *(u32 *)(r6 + %[data]);"
	"r2 = *(u32 *)(r6 + %[data_end]);"
	"r7 &= 0xff;"
	"r1 += r7;"
	"r3 = r1;"
	"r3 += 4;"
	"if r3 > r2 goto 2f;"
	"r0 = *(u32 *)(r1 + 0);"
"2:	call %[bpf_get_prandom_u32];"
	"r7 += 3;"
	"if r0 != 7 goto 1b;"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_get_prandom_u32), __imm_pkt
	: __clobber_all);
}

char _license[] SEC("license") = "GPL";
