// SPDX-License-Identifier: GPL-2.0

#define BPF_NO_KFUNC_PROTOTYPES
#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include "bpf_experimental.h"
#include <bpf_arena_common.h>

struct {
	__uint(type, BPF_MAP_TYPE_ARENA);
	__uint(map_flags, BPF_F_MMAPABLE);
	__uint(max_entries, 64);
} arena SEC(".maps");

bool skip;

#if defined(__BPF_FEATURE_ADDR_SPACE_CAST)

struct elem {
	struct elem __arena *next;
	int val;
};

int __arena x = 42;
int __arena arr[4] = { 1, 2, 3, 4 };
struct elem __arena n3 = { NULL, 3 };
struct elem __arena n2 = { &n3, 20 };
struct elem __arena n1 = { &n2, 100 };

/* variables that are zero and take no space in the file */
#define __arena_bss __arena SEC(".bss.addr_space.1")
char __arena_bss one;
long __arena_bss big[8192];
char __arena_bss aligned64[64] __attribute__((aligned(64)));

/* a pointer to a static variable is relocated against the section, not against a symbol */
static int __arena sx = 44;
static long __arena_bss sbig[16];

/* pointers to variables in arena that are stored in variables in arena */
int __arena * __arena px = &x;
int __arena * __arena parr = &arr[2];
struct elem __arena * __arena head = &n1;
long __arena * __arena pbig = &big[8000];
int __arena * __arena psx = &sx;
long __arena * __arena psbig = &sbig[3];

/* the size of .addr_space.1 is not a multiple of the alignment of .bss.addr_space.1 */
char __arena odd = 1;

SEC("syscall")
int use_ptrs(void *ctx)
{
	struct elem __arena *n;
	int i, sum = 0;

	if (*px != 42)
		return 1;
	*px = 43;
	if (x != 43)
		return 2;
	if (parr[0] != 3 || parr[1] != 4)
		return 3;
	for (n = head, i = 0; n && i < 10; n = n->next, i++)
		sum += n->val;
	if (sum != 123)
		return 4;
	if (*psx != 44)
		return 5;
	*psx = 45;
	if (sx != 45)
		return 6;
	return 0;
}

SEC("syscall")
int use_bss(void *ctx)
{
	unsigned long addr = (unsigned long)&aligned64;

	if (one || big[0] || big[8000] || big[8191] || aligned64[63])
		return 1;
	*pbig = 7;
	if (big[8000] != 7 || big[7999] || big[8001])
		return 2;
	big[8191] = 8;
	one = 1;
	/* variables that are not zero are where they were */
	if (arr[3] != 4)
		return 3;
	/* hide the address from the compiler, it knows that '& 63' is 0 */
	asm volatile ("" : "+r"(addr));
	if (addr & 63)
		return 4;
	if (sbig[2] || sbig[3] || sbig[4])
		return 5;
	*psbig = 9;
	if (sbig[3] != 9 || sbig[2] || sbig[4])
		return 6;
	return 0;
}

#else

/* The skeleton has to have what user space looks at */
struct elem {
	struct elem *next;
	int val;
};

int __arena_global x;
int __arena_global arr[4];
struct elem __arena_global n2;
struct elem __arena_global n1;
int * __arena_global px;
int * __arena_global parr;
struct elem * __arena_global head;
long * __arena_global pbig;
int * __arena_global psx;
long * __arena_global psbig;

SEC("syscall")
int use_ptrs(void *ctx)
{
	skip = true;
	return 0;
}

SEC("syscall")
int use_bss(void *ctx)
{
	skip = true;
	return 0;
}

#endif

char _license[] SEC("license") = "GPL";
