// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2024 Meta Platforms, Inc. and affiliates. */
/* arena_list.c and bpf_arena_list.h in plain C */
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
#ifdef __TARGET_ARCH_arm64
	__ulong(map_extra, 0x1ull << 32); /* start of mmap() region */
#else
	__ulong(map_extra, 0x1ull << 44); /* start of mmap() region */
#endif
} arena SEC(".maps");

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

__attribute__((unused))
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

__attribute__((unused))
static inline void list_del(arena_list_node_t *n)
{
	__list_del(n);
	n->next = LIST_POISON1;
	n->pprev = LIST_POISON2;
}

struct elem {
	struct arena_list_node node;
	__u64 value;
};

struct arena_list_head __arena *list_head;
int list_sum;
int cnt;
bool skip = false;
const volatile bool nonsleepable = false;

#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
long __arena arena_sum;
int __arena test_val = 1;
struct arena_list_head __arena global_head;
#else
long arena_sum SEC(".addr_space.1");
int test_val SEC(".addr_space.1");
#endif

void bpf_rcu_read_lock(void) __ksym;
void bpf_rcu_read_unlock(void) __ksym;

SEC("syscall")
__success
int arena_list_add(void *ctx)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	__u64 i;

	list_head = &global_head;

	for (i = 0; i < cnt; i++) {
		struct elem __arena *n = bpf_alloc(sizeof(*n));

		test_val++;
		n->value = i;
		arena_sum += i;
		list_add_head(&n->node, list_head);
	}
#else
	skip = true;
#endif
	return 0;
}

SEC("syscall")
__success
int arena_list_del(void *ctx)
{
#ifdef __BPF_FEATURE_ADDR_SPACE_CAST
	struct elem __arena *n;
	int sum = 0;

	/* Take rcu_read_lock to test non-sleepable context */
	if (nonsleepable)
		bpf_rcu_read_lock();

	arena_sum = 0;
	list_for_each_entry(n, list_head, node) {
		sum += n->value;
		arena_sum += n->value;
		list_del(&n->node);
		bpf_free(n);
	}
	list_sum = sum;

	if (nonsleepable)
		bpf_rcu_read_unlock();
#else
	skip = true;
#endif
	return 0;
}

char _license[] SEC("license") = "GPL";
