// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2025 Meta Platforms, Inc. and affiliates. */
/* rbtree_search.c in plain C */

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_misc.h"
#include "bpf_experimental.h"

struct node_data {
	struct bpf_refcount ref;
	struct bpf_rb_node r0;
	struct bpf_rb_node r1;
	int key0;
	int key1;
};

#define private(name) SEC(".data." #name) __hidden __attribute__((aligned(8)))
private(A) struct bpf_spin_lock glock0;
private(A) struct bpf_rb_root groot0 __contains(node_data, r0);

private(B) struct bpf_spin_lock glock1;
private(B) struct bpf_rb_root groot1 __contains(node_data, r1);

#define rb_entry(ptr, type, member) container_of(ptr, type, member)
#define NR_NODES 16

static bool less0(struct bpf_rb_node *a, const struct bpf_rb_node *b)
{
	struct node_data *node_a;
	struct node_data *node_b;

	node_a = rb_entry(a, struct node_data, r0);
	node_b = rb_entry(b, struct node_data, r0);

	return node_a->key0 < node_b->key0;
}

static bool less1(struct bpf_rb_node *a, const struct bpf_rb_node *b)
{
	struct node_data *node_a;
	struct node_data *node_b;

	node_a = rb_entry(a, struct node_data, r1);
	node_b = rb_entry(b, struct node_data, r1);

	return node_a->key1 < node_b->key1;
}

SEC("syscall")
__success __retval(0)
#ifndef __clang__
__skip("GCC walks the array on the stack with a pointer")
#endif
long rbtree_search(void *ctx)
{
	struct bpf_rb_node *rb_n, *rb_m, *gc_ns[NR_NODES];
	long lookup_key = NR_NODES / 2;
	struct node_data *n, *m;
	int i, nr_gc = 0;

	for (i = 0; i < NR_NODES; i++) {
		n = bpf_obj_new(typeof(*n));
		if (!n)
			return __LINE__;

		m = bpf_refcount_acquire(n);

		n->key0 = i;
		m->key1 = i;

		bpf_spin_lock(&glock0);
		bpf_rbtree_add(&groot0, &n->r0, less0);
		bpf_spin_unlock(&glock0);

		bpf_spin_lock(&glock1);
		bpf_rbtree_add(&groot1, &m->r1, less1);
		bpf_spin_unlock(&glock1);
	}

	n = NULL;
	bpf_spin_lock(&glock0);
	rb_n = bpf_rbtree_root(&groot0);
	while (1) {
		if (!rb_n) {
			bpf_spin_unlock(&glock0);
			return __LINE__;
		}

		n = rb_entry(rb_n, struct node_data, r0);
		if (lookup_key == n->key0)
			break;
		if (nr_gc < NR_NODES)
			gc_ns[nr_gc++] = rb_n;
		if (lookup_key < n->key0)
			rb_n = bpf_rbtree_left(&groot0, rb_n);
		else
			rb_n = bpf_rbtree_right(&groot0, rb_n);
	}

	if (!n || lookup_key != n->key0) {
		bpf_spin_unlock(&glock0);
		return __LINE__;
	}

	for (i = 0; i < nr_gc; i++) {
		rb_n = gc_ns[i];
		gc_ns[i] = bpf_rbtree_remove(&groot0, rb_n);
	}

	m = bpf_refcount_acquire(n);
	bpf_spin_unlock(&glock0);

	for (i = 0; i < nr_gc; i++) {
		rb_n = gc_ns[i];
		if (rb_n) {
			n = rb_entry(rb_n, struct node_data, r0);
			bpf_obj_drop(n);
		}
	}

	if (!m)
		return __LINE__;

	bpf_spin_lock(&glock1);
	rb_m = bpf_rbtree_remove(&groot1, &m->r1);
	bpf_spin_unlock(&glock1);
	bpf_obj_drop(m);
	if (!rb_m)
		return __LINE__;
	bpf_obj_drop(rb_entry(rb_m, struct node_data, r1));

	return 0;
}

char _license[] SEC("license") = "GPL";
