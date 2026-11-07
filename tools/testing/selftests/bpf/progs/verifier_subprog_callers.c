// SPDX-License-Identifier: GPL-2.0
/* A static subprog and its callers: what they hold, where they run */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_misc.h"
#include "bpf_experimental.h"

#define private(name) SEC(".bss." #name) __hidden __attribute__((aligned(8)))

struct elem {
	struct bpf_list_node l;
	__u64 data;
};

private(A) struct bpf_spin_lock lock_a;
private(A) struct bpf_list_head head_a __contains(elem, l);

int pid;

static __noinline void relock_a(void)
{
	bpf_spin_unlock(&lock_a);
	bpf_spin_lock(&lock_a);
}

/* Somebody else can pop and free the node while the lock is not held. */
SEC("tc")
__description("subprog drops the lock of its caller for a while, non-owning reference")
__failure __msg("invalid mem access 'scalar'")
int relock_non_owning_ref(struct __sk_buff *skb)
{
	struct elem *n;

	n = bpf_obj_new(typeof(*n));
	if (!n)
		return 0;
	bpf_spin_lock(&lock_a);
	bpf_list_push_front(&head_a, &n->l);
	relock_a();
	n->data = 42;
	bpf_spin_unlock(&lock_a);
	return 0;
}

static __noinline void rcu_window(void)
{
	bpf_rcu_read_unlock();
	bpf_rcu_read_lock();
}

SEC("?fentry.s/" SYS_PREFIX "sys_getpgid")
__description("subprog leaves the RCU section of its caller for a while")
__failure __msg("R1 must be a rcu pointer")
int rcu_window_rcu_ptr(void *ctx)
{
	struct task_struct *task = bpf_get_current_task_btf(), *parent;

	bpf_rcu_read_lock();
	parent = task->real_parent;
	if (!parent)
		goto out;
	rcu_window();
	parent = bpf_task_acquire(parent);
	if (parent) {
		pid = parent->pid;
		bpf_task_release(parent);
	}
out:
	bpf_rcu_read_unlock();
	return 0;
}

struct timer_elem {
	struct bpf_timer timer;
};

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct timer_elem);
} timers SEC(".maps");

static __noinline int leaves_preempt_off(void *map, int *key, struct timer_elem *val)
{
	bpf_preempt_disable();
	return 0;
}

/* The call and the callback have one frame each. The callback returns to the kernel. */
SEC("tc")
__description("subprog is called, then it is made a timer callback")
__failure __msg("cannot be used inside bpf_preempt_disable-ed region")
int call_then_timer_callback(struct __sk_buff *skb)
{
	struct timer_elem *val = bpf_map_lookup_elem(&timers, &(int){0});

	if (!val)
		return 0;
	leaves_preempt_off(NULL, NULL, NULL);
	bpf_preempt_enable();
	bpf_timer_set_callback(&val->timer, leaves_preempt_off);
	return 0;
}

/* return p + 14 > ctx->data_end ? -22 : 0; */
static __naked __noinline __used int has_eth(struct xdp_md *ctx, void *p)
{
	asm volatile (
	"r3 = *(u32 *)(r1 + %[data_end]);"
	"r2 += 14;"
	"r0 = -22;"
	"if r2 > r3 goto 1f;"
	"r0 = 0;"
"1:	exit;"
	:
	: __imm_const(data_end, offsetof(struct xdp_md, data_end))
	: __clobber_all);
}

/* The caller learns how much of the packet is there. */
SEC("?xdp")
__description("subprog compares a packet pointer of its caller with the end that it loads itself")
__success
__naked void subprog_loads_data_end(void)
{
	asm volatile (
	"r6 = *(u32 *)(r1 + %[data]);"
	"r2 = r6;"
	"call has_eth;"
	"if r0 != 0 goto 1f;"
	"r0 = *(u8 *)(r6 + 13);"
"1:	r0 = 1;"
	"exit;"
	:
	: __imm_const(data, offsetof(struct xdp_md, data))
	: __clobber_all);
}

struct tree_elem {
	long key;
	struct bpf_rb_node node;
};

private(D) struct bpf_spin_lock lock_d;
private(D) struct bpf_rb_root root_d __contains(tree_elem, node);

static bool less(struct bpf_rb_node *a, const struct bpf_rb_node *b)
{
	struct tree_elem *elem_a = container_of(a, struct tree_elem, node);
	struct tree_elem *elem_b = container_of(b, struct tree_elem, node);

	return elem_a->key < elem_b->key;
}

static __noinline long removes_and_recurses(struct tree_elem *e, int n)
{
	struct bpf_rb_node *node;

	if (n <= 0)
		return 0;
	node = bpf_rbtree_remove(&root_d, &e->node);
	if (node)
		bpf_rbtree_add(&root_d, node, less);
	n = removes_and_recurses(e, n - 1);
	barrier_var(n);
	return n + 1;
}

static bool less_changes_tree(struct bpf_rb_node *a, const struct bpf_rb_node *b)
{
	struct tree_elem *elem_a = container_of(a, struct tree_elem, node);
	struct tree_elem *elem_b = container_of(b, struct tree_elem, node);

	removes_and_recurses(elem_b, 1);
	return elem_a->key < elem_b->key;
}

/* bpf_rbtree_add() walks the tree while it calls the callback. */
SEC("?tc")
__description("recursive subprog changes the tree, it is called from the callback of rbtree_add")
__arch_x86_64
__arch_arm64
__failure __msg("bpf_rbtree_remove not allowed in rbtree cb")
int recursion_removes_in_rbtree_callback(void *ctx)
{
	struct tree_elem *e;

	e = bpf_obj_new(typeof(*e));
	if (!e)
		return 0;
	bpf_spin_lock(&lock_d);
	bpf_rbtree_add(&root_d, &e->node, less_changes_tree);
	bpf_spin_unlock(&lock_d);
	return 0;
}

/*
 * What a subprog does is what its callees do. The walk over the graph is done with second()
 * before it is done with first(), which second() calls: the prog calls second() and then
 * first(), so first() is looked at first, and in it the call that comes last.
 */
static __noinline int pulls_data(struct __sk_buff *skb)
{
	return bpf_skb_pull_data(skb, 0);
}

static __noinline int pulls_first(struct __sk_buff *skb, int n);

static __noinline int pulls_second(struct __sk_buff *skb, int n)
{
	if (n <= 0)
		return 0;
	return pulls_first(skb, n - 1) + 1;
}

static __noinline int pulls_first(struct __sk_buff *skb, int n)
{
	int r = pulls_data(skb);

	return r + pulls_second(skb, n);
}

SEC("?tc")
__description("recursive static subprogs, the one that is explored last changes packet data")
__arch_x86_64
__arch_arm64
__failure __msg("invalid mem access 'scalar'")
int recursion_changes_pkt_data(struct __sk_buff *skb)
{
	char *p = (void *)(long)skb->data, *end = (void *)(long)skb->data_end;
	int r;

	if (p + 1 > end)
		return 0;
	r = pulls_second(skb, 1);
	*p = 42;
	r += pulls_first(skb, 0);
	return r & 1;
}

static __naked __noinline __used long returns_third(void)
{
	asm volatile (
	"r0 = r1;"
	"r0 = r2;"
	"r0 = r3;"
	"exit;"
	::: __clobber_all);
}

/* What comes back has the range of r6, not the range of the pointer of the first call. */
#define PKT_POINTERS_OF_TWO_RANGES(func)	\
	asm volatile (				\
	"r9 = *(u32 *)(r1 + %[data]);"		\
	"r8 = *(u32 *)(r1 + %[data_end]);"	\
	"call %[bpf_get_prandom_u32];"		\
	"r7 = r0;"				\
	"r0 &= 0xff;"				\
	"r1 = 0;"				\
	"r2 = 3;"				\
	"r3 = r9;"				\
	"r3 += r0;"				\
	"call " #func ";"			\
	"r7 &= 1;"				\
	"r6 = r9;"				\
	"r6 += r7;"				\
	"r1 = 0;"				\
	"r2 = 3;"				\
	"r3 = r6;"				\
	"call " #func ";"			\
	"if r0 == 0 goto 1f;"			\
	"if r0 > r8 goto 1f;"			\
	"r0 = *(u8 *)(r6 + 100);"		\
"1:"						\
	"r0 = 0;"				\
	"exit;"					\
	:					\
	: __imm(bpf_get_prandom_u32),		\
	  __imm_const(data, offsetof(struct __sk_buff, data)),		\
	  __imm_const(data_end, offsetof(struct __sk_buff, data_end))	\
	: __clobber_all)

SEC("tc")
__description("two calls, the offset of the packet pointer has a smaller range in the second one")
__failure __msg("invalid access to packet")
__naked void pkt_pointers_of_two_ranges(void)
{
	PKT_POINTERS_OF_TWO_RANGES(returns_third);
}

static __naked __noinline __used long returns_third_r2_calls_down(void)
{
	asm volatile (
	"if r2 == 0 goto 1f;"
	"r2 += -1;"
	"call returns_third_r2_calls_down;"
	"exit;"
"1:"
	"r0 = r3;"
	"exit;"
	::: __clobber_all);
}

/* A recursive subprog that gets such a pointer needs the frames of its callers too. */
SEC("?tc")
__description("two calls of a recursive subprog, the second packet pointer has the smaller range")
__arch_x86_64
__arch_arm64
__failure __msg("invalid access to packet")
__naked void pkt_pointers_of_two_ranges_recursion(void)
{
	PKT_POINTERS_OF_TWO_RANGES(returns_third_r2_calls_down);
}

static __naked __noinline __used long tcp_sock_of(void)
{
	asm volatile (
	"call %[bpf_tcp_sock];"
	"exit;"
	:
	: __imm(bpf_tcp_sock)
	: __clobber_all);
}

static __naked __noinline __used long calls_tcp_sock_of(void)
{
	asm volatile (
	"call tcp_sock_of;"
	"exit;"
	::: __clobber_all);
}

/* The caller of tcp_sock_of() has the reference from its own caller. */
SEC("tc")
__description("reference goes down two calls, what is made from it is used after the release")
__failure __msg("R7 invalid mem access 'scalar'")
__naked void reference_two_calls_down(void)
{
	asm volatile (
	"r2 = 0;"
	"*(u64 *)(r10 - 8) = r2;"
	"*(u64 *)(r10 - 16) = r2;"
	"r2 = r10;"
	"r2 += -16;"
	"r3 = 12;"
	"r4 = 0;"
	"r5 = 0;"
	"call %[bpf_sk_lookup_tcp];"
	"if r0 == 0 goto 1f;"
	"r6 = r0;"
	"r1 = r6;"
	"call calls_tcp_sock_of;"
	"r7 = r0;"
	"r1 = r6;"
	"call %[bpf_sk_release];"
	"if r7 == 0 goto 1f;"
	"r0 = *(u32 *)(r7 + 0);"
"1:"
	"r0 = 0;"
	"exit;"
	:
	: __imm(bpf_sk_lookup_tcp),
	  __imm(bpf_sk_release)
	: __clobber_all);
}

struct inner_timers {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__uint(max_entries, 1);
	__type(key, int);
	__type(value, struct timer_elem);
} inner_timers SEC(".maps");

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY_OF_MAPS);
	__uint(max_entries, 2);
	__type(key, int);
	__array(values, struct inner_timers);
} outer_timers SEC(".maps");

static __naked __noinline __used long inner_map(void)
{
	asm volatile (
	"*(u32 *)(r10 - 4) = r1;"
	"r2 = r10;"
	"r2 += -4;"
	"r1 = %[outer_timers] ll;"
	"call %[bpf_map_lookup_elem];"
	"exit;"
	:
	: __imm(bpf_map_lookup_elem),
	  __imm_addr(outer_timers)
	: __clobber_all);
}

#define TWO_INNER_MAPS				\
	"r1 = 0;"				\
	"call inner_map;"			\
	"if r0 == 0 goto 1f;"			\
	"r6 = r0;"				\
	"r1 = 1;"				\
	"call inner_map;"			\
	"if r0 == 0 goto 1f;"			\
	"r7 = r0;"

/* The timer is in the map in r0. It is fine only if that is the map in r6. */
#define INIT_TIMER_WITH_R6			\
	"r1 = 0;"				\
	"*(u32 *)(r10 - 4) = r1;"		\
	"r2 = r10;"				\
	"r2 += -4;"				\
	"r1 = r0;"				\
	"call %[bpf_map_lookup_elem];"		\
	"if r0 == 0 goto 1f;"			\
	"r1 = r0;"				\
	"r2 = r6;"				\
	"r3 = 1;"				\
	"call %[bpf_timer_init];"		\
"1:"						\
	"r0 = 0;"				\
	"exit;"

#define INNER_MAP_IMMS				\
	__imm(bpf_get_prandom_u32),		\
	__imm(bpf_map_lookup_elem),		\
	__imm(bpf_timer_init)

/* Each call returns another map. */
SEC("tc")
__description("subprog returns an inner map, timer of one map is set up with another one")
__failure __msg("doesn't match map pointer in R2")
__naked void inner_maps_of_two_calls(void)
{
	asm volatile (
	TWO_INNER_MAPS
	INIT_TIMER_WITH_R6
	:
	: INNER_MAP_IMMS
	: __clobber_all);
}

/* As many as there is room for ids at the return, and one more. */
static __naked __noinline __used void saves_irq_flags_many_times(void)
{
	asm volatile (
	".rept 9153;"
	"r1 = r10;"
	"r1 += -8;"
	"call %[bpf_local_irq_save];"
	"r1 = 0;"
	"*(u64 *)(r10 - 8) = r1;"
	".endr;"
	"exit;"
	:
	: __imm(bpf_local_irq_save)
	: __clobber_all);
}

SEC("tc")
__description("subprog comes back with more ids than there is room for")
__failure __msg("too many ids at the call at insn")
__log_level(4)
int too_many_ids_at_the_return(void *ctx)
{
	unsigned long flags;

	saves_irq_flags_many_times();
	bpf_local_irq_save(&flags);
	bpf_local_irq_restore(&flags);
	return 0;
}

static __naked __noinline __used void leaves_rcu_section(void)
{
	asm volatile (
	"call %[bpf_rcu_read_unlock];"
	"call %[bpf_rcu_read_lock];"
	"exit;"
	:
	: __imm(bpf_rcu_read_unlock),
	  __imm(bpf_rcu_read_lock)
	: __clobber_all);
}

static __naked __noinline __used int returns_zero(void)
{
	asm volatile (
	"r0 = 0;"
	"exit;"
	::: __clobber_all);
}

/*
 * Three calls wait for the subprog. When it's known what the subprog does to the RCU section of
 * its callers each of them stops to wait and is verified in the context of its caller.
 * Then one more call waits.
 */
#define RCU_BRANCHES(then1, then2, then3)	\
	asm volatile (				\
	"r6 = *(u32 *)(r1 + %[len]);"		\
	"call %[bpf_rcu_read_lock];"		\
	"if r6 == 1 goto 1f;"			\
	"if r6 == 2 goto 2f;"			\
	"call leaves_rcu_section;"		\
	then3					\
	"goto 3f;"				\
"1:"						\
	"call leaves_rcu_section;"		\
	then1					\
	"goto 3f;"				\
"2:"						\
	"call leaves_rcu_section;"		\
	then2					\
"3:"						\
	"call %[bpf_rcu_read_unlock];"		\
	"call returns_zero;"			\
	"exit;"					\
	:					\
	: __imm(bpf_rcu_read_lock),		\
	  __imm(bpf_rcu_read_unlock),		\
	  __imm_const(len, offsetof(struct __sk_buff, len))	\
	: __clobber_all)

SEC("tc")
__description("three calls stop to wait")
__success __retval(0)
__naked void three_calls_stop_to_wait(void)
{
	RCU_BRANCHES("", "", "");
}

SEC("tc")
__description("three calls stop to wait, what follows the first one")
__failure __msg("R7 !read_ok")
__naked void three_calls_stop_to_wait_bad_1(void)
{
	RCU_BRANCHES("r0 = r7;", "", "");
}

char _license[] SEC("license") = "GPL";
