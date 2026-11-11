// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2024 Meta Platforms, Inc. and affiliates. */
/* arena_htab.c, bpf_arena_htab.h and bpf_arena_list.h in plain C */
#define BPF_NO_KFUNC_PROTOTYPES
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include <bpf/bpf_core_read.h>
#include "bpf_experimental.h"
#include "bpf_misc.h"

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 100); /* number of pages */
} arena SEC(".maps");

#include <errno.h>
#include "bpf_arena_alloc.h"
#include <bpf_arena_common.h>

struct arena_list_node;

typedef struct arena_list_node __arena arena_list_node_t;

struct arena_list_node {
	arena_list_node_t *next;
	arena_list_node_t * __arena *pprev;
};

struct arena_list_head {
	struct arena_list_node __arena *first;
};
typedef struct arena_list_head __arena arena_list_head_t;

#define list_entry(ptr, type, member) arena_container_of(ptr, type, member)

#define list_entry_safe(ptr, type, member) \
	({ typeof(*ptr) * ___ptr = (ptr); \
	 ___ptr ? ({ cast_kern(___ptr); list_entry(___ptr, type, member); }) : NULL; \
	 })

/* Safely walk link list elements. Deletion of elements is allowed. */
#define list_for_each_entry(pos, head, member)					\
	for (void * ___tmp = (pos = list_entry_safe((head)->first,		\
						    typeof(*(pos)), member),	\
			      (void *)0);					\
	     pos && ({ ___tmp = (void *)pos->member.next; 1; });                \
	     pos = list_entry_safe((void __arena *)___tmp, typeof(*(pos)), member))

static inline void list_add_head(arena_list_node_t *n, arena_list_head_t *h)
{
	arena_list_node_t *first = h->first, * __arena *tmp;

	cast_user(first);
	cast_kern(n);
	WRITE_ONCE(n->next, first);
	cast_kern(first);
	if (first) {
		tmp = &n->next;
		cast_user(tmp);
		WRITE_ONCE(first->pprev, tmp);
	}
	cast_user(n);
	WRITE_ONCE(h->first, n);

	tmp = &h->first;
	cast_user(tmp);
	cast_kern(n);
	WRITE_ONCE(n->pprev, tmp);
}

static inline void __list_del(arena_list_node_t *n)
{
	arena_list_node_t *next = n->next;
	arena_list_node_t * __arena *pprev = n->pprev;

	cast_user(next);
	cast_kern(pprev);
	WRITE_ONCE(*pprev, next);
	if (next) {
		cast_user(pprev);
		cast_kern(next);
		WRITE_ONCE(next->pprev, pprev);
	}
}

#define POISON_POINTER_DELTA 0

#define LIST_POISON1  ((void __arena *) 0x100 + POISON_POINTER_DELTA)
#define LIST_POISON2  ((void __arena *) 0x122 + POISON_POINTER_DELTA)

static inline void list_del(arena_list_node_t *n)
{
	__list_del(n);
	n->next = LIST_POISON1;
	n->pprev = LIST_POISON2;
}

struct htab_bucket {
	struct arena_list_head head;
};
typedef struct htab_bucket __arena htab_bucket_t;

struct htab {
	htab_bucket_t *buckets;
	int n_buckets;
};

static inline htab_bucket_t *__select_bucket(struct htab __arena *htab, __u32 hash)
{
	htab_bucket_t *b = htab->buckets;

	cast_kern(b);
	return &b[hash & (htab->n_buckets - 1)];
}

static inline arena_list_head_t *select_bucket(struct htab __arena *htab, __u32 hash)
{
	return &__select_bucket(htab, hash)->head;
}

struct hashtab_elem {
	int hash;
	int key;
	int value;
	struct arena_list_node hash_node;
};
typedef struct hashtab_elem __arena hashtab_elem_t;

static hashtab_elem_t *lookup_elem_raw(arena_list_head_t *head, __u32 hash, int key)
{
	hashtab_elem_t *l;

	list_for_each_entry(l, head, hash_node)
		if (l->hash == hash && l->key == key)
			return l;

	return NULL;
}

static int htab_hash(int key)
{
	return key;
}

__weak int htab_update_elem(struct htab __arena *htab, int key, int value)
{
	hashtab_elem_t *l_new = NULL, *l_old;
	arena_list_head_t *head;

	cast_kern(htab);
	head = select_bucket(htab, key);
	l_old = lookup_elem_raw(head, htab_hash(key), key);

	l_new = bpf_alloc(sizeof(*l_new));
	if (!l_new)
		return -ENOMEM;
	l_new->key = key;
	l_new->hash = htab_hash(key);
	l_new->value = value;

	list_add_head(&l_new->hash_node, head);
	if (l_old) {
		list_del(&l_old->hash_node);
		bpf_free(l_old);
	}
	return 0;
}

void htab_init(struct htab __arena *htab)
{
	void __arena *buckets = bpf_arena_alloc_pages(&arena, NULL, 2, NUMA_NO_NODE, 0);

	cast_user(buckets);
	htab->buckets = buckets;
	htab->n_buckets = 2 * PAGE_SIZE / sizeof(struct htab_bucket);
}

void __arena *htab_for_user;
bool skip = false;

char __arena arr1[100000];
char arr2[1000];

SEC("syscall")
__success __retval(0)
#if !defined(__clang__) && defined(BPF_ARENA_FORCE_ASM)
__skip("GCC calls htab_init(): a global function gets an arena pointer as a scalar")
#endif
int arena_htab_llvm(void *ctx)
{
#if defined(__BPF_FEATURE_ADDR_SPACE_CAST) || defined(BPF_ARENA_FORCE_ASM)
	struct htab __arena *htab;
	char __arena *arr = arr1;
	__u64 i;

	htab = bpf_alloc(sizeof(*htab));
	cast_kern(htab);
	htab_init(htab);

	/* first run. No old elems in the table */
	for (i = 0; i < 100000; i++) {
		htab_update_elem(htab, i, i);
		arr[i] = i;
	}

	/* should replace some elems with new ones */
	for (i = 0; i < 1000; i++) {
		htab_update_elem(htab, i, i);
		/* Access mem to make the verifier use bounded loop logic */
		arr2[i] = i;
	}
	cast_user(htab);
	htab_for_user = htab;
#else
	skip = true;
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
