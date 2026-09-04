// SPDX-License-Identifier: GPL-2.0
#include <vmlinux.h>
#include <bpf/bpf_tracing.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_core_read.h>
#include "bpf_experimental.h"
#include "bpf_misc.h"

struct node_acquire {
	long key;
	long data;
	struct bpf_rb_node node;
	struct bpf_refcount refcount;
};

struct node_refcounted {
	long key;
	struct bpf_list_node list;
	struct bpf_refcount refcount;
};

struct node_refcount_only {
	long key;
	struct bpf_refcount refcount;
};

struct map_value_refcount_only {
	struct node_refcount_only __kptr *node;
};

extern void bpf_rcu_read_lock(void) __ksym;
extern void bpf_rcu_read_unlock(void) __ksym;

#define private(name) SEC(".data." #name) __hidden __attribute__((aligned(8)))
private(A) struct bpf_spin_lock glock;
private(A) struct bpf_rb_root groot __contains(node_acquire, node);
private(B) struct bpf_spin_lock lock;
private(B) struct bpf_list_head head __contains(node_refcounted, list);

struct {
	__uint(type, BPF_MAP_TYPE_ARRAY);
	__type(key, int);
	__type(value, struct map_value_refcount_only);
	__uint(max_entries, 1);
} stashed_refcount_only SEC(".maps");

static bool less(struct bpf_rb_node *a, const struct bpf_rb_node *b)
{
	struct node_acquire *node_a;
	struct node_acquire *node_b;

	node_a = container_of(a, struct node_acquire, node);
	node_b = container_of(b, struct node_acquire, node);

	return node_a->key < node_b->key;
}

SEC("?tc")
__failure __msg("Unreleased reference id=4 alloc_insn={{[0-9]+}}")
long rbtree_refcounted_node_ref_escapes(void *ctx)
{
	struct node_acquire *n, *m;

	n = bpf_obj_new(typeof(*n));
	if (!n)
		return 1;

	bpf_spin_lock(&glock);
	bpf_rbtree_add(&groot, &n->node, less);
	/* m becomes an owning ref but is never drop'd or added to a tree */
	m = bpf_refcount_acquire(n);
	bpf_spin_unlock(&glock);
	if (!m)
		return 2;

	m->key = 2;
	return 0;
}

SEC("?tc")
__failure __msg("Possibly NULL pointer passed to trusted R1")
__msg("requires a non-NULL value of type (void *)")
long refcount_acquire_maybe_null(void *ctx)
{
	struct node_acquire *n, *m;

	n = bpf_obj_new(typeof(*n));
	/* Intentionally not testing !n
	 * it's MAYBE_NULL for refcount_acquire
	 */
	m = bpf_refcount_acquire(n);
	if (m)
		bpf_obj_drop(m);
	if (n)
		bpf_obj_drop(n);

	return 0;
}

SEC("?tc")
__failure __msg("R1 is neither owning or non-owning ref")
__msg("expects a pointer to a BPF-managed refcounted object, but R1 is a context pointer")
long refcount_acquire_non_object(void *ctx)
{
	return bpf_refcount_acquire(ctx) != NULL;
}

SEC("?syscall")
__failure __msg("Possibly NULL pointer passed to trusted R1")
long refcount_acquire_rcu_map_kptr_unchecked_drop(void *ctx)
{
	struct map_value_refcount_only *mapval;
	struct node_refcount_only *tmp, *n, *m;
	int idx = 0;

	/* Force Clang to emit complete BTF for struct node_refcount_only. */
	tmp = bpf_obj_new(typeof(*tmp));
	if (!tmp)
		return 3;
	bpf_obj_drop(tmp);

	mapval = bpf_map_lookup_elem(&stashed_refcount_only, &idx);
	if (!mapval)
		return 1;

	bpf_rcu_read_lock();
	n = mapval->node;
	if (!n) {
		bpf_rcu_read_unlock();
		return 2;
	}
	m = bpf_refcount_acquire(n);
	bpf_rcu_read_unlock();

	bpf_obj_drop(m);

	return 0;
}

SEC("?tc")
__failure __msg("Unreleased reference id=3 alloc_insn={{[0-9]+}}")
long rbtree_refcounted_node_ref_escapes_owning_input(void *ctx)
{
	struct node_acquire *n, *m;

	n = bpf_obj_new(typeof(*n));
	if (!n)
		return 1;

	/* m becomes an owning ref but is never drop'd or added to a tree */
	m = bpf_refcount_acquire(n);
	m->key = 2;

	bpf_spin_lock(&glock);
	bpf_rbtree_add(&groot, &n->node, less);
	bpf_spin_unlock(&glock);

	return 0;
}

SEC("?tc")
__failure __msg("dereference of modified ptr_ ptr R1")
long refcount_acquire_list_node_offset(void *ctx)
{
	struct node_refcounted *node, *base, *ref;
	struct bpf_list_node *list_node;

	node = bpf_obj_new(typeof(*node));
	if (!node)
		return 1;

	bpf_spin_lock(&lock);
	bpf_list_push_front(&head, &node->list);
	list_node = bpf_list_pop_front(&head);
	bpf_spin_unlock(&lock);
	if (!list_node)
		return 2;

	base = container_of(list_node, struct node_refcounted, list);
	ref = bpf_refcount_acquire(list_node);
	if (ref)
		bpf_obj_drop(ref);
	bpf_obj_drop(base);
	return 0;
}

SEC("?fentry.s/" SYS_PREFIX "sys_getpgid")
__failure __msg("function calls are not allowed while holding a lock")
int BPF_PROG(rbtree_fail_sleepable_lock_across_rcu,
	     struct file *file, struct kobject *kobj,
	     struct bin_attribute *bin_attr, char *buf, loff_t off, size_t len)
{
	struct node_acquire *n;

	n = bpf_obj_new(typeof(*n));
	if (!n)
		return 0;

	/* spin_{lock,unlock} are in different RCU CS */
	bpf_rcu_read_lock();
	bpf_spin_lock(&glock);
	bpf_rbtree_add(&groot, &n->node, less);
	bpf_rcu_read_unlock();

	bpf_rcu_read_lock();
	bpf_spin_unlock(&glock);
	bpf_rcu_read_unlock();

	return 0;
}

char _license[] SEC("license") = "GPL";
