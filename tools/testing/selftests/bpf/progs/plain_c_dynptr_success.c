// SPDX-License-Identifier: GPL-2.0
/* Copyright (c) 2022 Facebook */
/* dynptr_success.c in plain C */

#include <vmlinux.h>
#include <string.h>
#include <stdbool.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>
#include "bpf_misc.h"
#include "errno.h"

#define PAGE_SIZE_64K 65536

char _license[] SEC("license") = "GPL";

int err;

static inline int bpf_memcmp(const char *a, const char *b, u32 size)
{
	int i;

	for (i = 0; i < size; i++) {
		if (a[i] != b[i])
			return a[i] < b[i] ? -1 : 1;
	}
	return 0;
}

static int fill_xdp(struct bpf_dynptr *ptr, __u32 start, __u32 n)
{
	__u8 b;
	int i;

	for (i = 0; i < n; i++) {
		b = (start + i) % 251;
		if (bpf_dynptr_write(ptr, start + i, &b, 1, 0))
			return 1;
	}
	return 0;
}

static int check_xdp(struct bpf_dynptr *ptr, __u32 at, __u32 n, __u32 from)
{
	__u8 b;
	int i;

	for (i = 0; i < n; i++) {
		if (bpf_dynptr_read(&b, 1, ptr, at + i, 0))
			return 1;
		if (b != (__u8)((from + i) % 251))
			return 1;
	}
	return 0;
}

SEC("?xdp")
__success
int test_dynptr_copy_xdp_overlap(struct xdp_md *xdp)
{
	void *data_end = (void *)(long)xdp->data_end;
	void *data = (void *)(long)xdp->data;
	__u32 off = data_end - data - 1000;
	struct bpf_dynptr ptr;

	bpf_dynptr_from_xdp(xdp, 0, &ptr);
	if (fill_xdp(&ptr, off, 1300)) {
		err = 1;
		return XDP_DROP;
	}
	err = bpf_dynptr_copy(&ptr, off + 100, &ptr, off, 1200);
	if (err)
		return XDP_DROP;
	if (check_xdp(&ptr, off + 100, 1200, off))
		err = 2;
	return XDP_DROP;
}

SEC("?xdp")
__success
int test_dynptr_copy_xdp_overlap_src_linear(struct xdp_md *xdp)
{
	void *data_end = (void *)(long)xdp->data_end;
	void *data = (void *)(long)xdp->data;
	__u32 lin = data_end - data;
	struct bpf_dynptr ptr;

	bpf_dynptr_from_xdp(xdp, 0, &ptr);
	if (fill_xdp(&ptr, lin - 64, 118)) {
		err = 1;
		return XDP_DROP;
	}
	err = bpf_dynptr_copy(&ptr, lin - 10, &ptr, lin - 64, 64);
	if (err)
		return XDP_DROP;
	if (check_xdp(&ptr, lin - 10, 64, lin - 64))
		err = 2;
	return XDP_DROP;
}

SEC("?xdp")
__success
int test_dynptr_copy_xdp_overlap_dst_frag(struct xdp_md *xdp)
{
	void *data_end = (void *)(long)xdp->data_end;
	void *data = (void *)(long)xdp->data;
	__u32 lin = data_end - data;
	struct bpf_dynptr ptr;

	bpf_dynptr_from_xdp(xdp, 0, &ptr);
	if (fill_xdp(&ptr, lin - 10, 84)) {
		err = 1;
		return XDP_DROP;
	}
	err = bpf_dynptr_copy(&ptr, lin + 10, &ptr, lin - 10, 64);
	if (err)
		return XDP_DROP;
	if (check_xdp(&ptr, lin + 10, 64, lin - 10))
		err = 2;
	return XDP_DROP;
}

#define DYNPTR_MEMSET_VAL 42

#define min_t(type, x, y) ({		\
	type __x = (x);			\
	type __y = (y);			\
	__x < __y ? __x : __y; })

SEC("xdp")
__success
int test_dynptr_memset_xdp_chunks(struct xdp_md *xdp)
{
	u32 data_sz, chunk_sz, offset = 0;
	const int max_chunks = 200;
	struct bpf_dynptr ptr_xdp;
	char expected_buf[32];
	char buf[32];
	int i;

	__builtin_memset(expected_buf, DYNPTR_MEMSET_VAL, sizeof(expected_buf));

	/* ptr_xdp is backed by non-contiguous memory */
	bpf_dynptr_from_xdp(xdp, 0, &ptr_xdp);
	data_sz = bpf_dynptr_size(&ptr_xdp);

	err = bpf_dynptr_memset(&ptr_xdp, 0, data_sz, DYNPTR_MEMSET_VAL);
	if (err) {
		/*
		 * bpf_dynptr_memset() eventually called bpf_xdp_pointer()
		 * where if data_sz is greater than 0xffff, -EFAULT will be
		 * returned. For 64K page size, data_sz is greater than
		 * 64K, so error is expected and let us zero out error and
		 * return success.
		 */
		if (data_sz >= PAGE_SIZE_64K)
			err = 0;
		goto out;
	}

	for (i = 0; i < max_chunks; i++) {
		offset = i * sizeof(buf);
		if (offset >= data_sz)
			goto out;
		chunk_sz = min_t(u32, sizeof(buf), data_sz - offset);
		err = bpf_dynptr_read(&buf, chunk_sz, &ptr_xdp, offset, 0);
		if (err)
			goto out;
		err = bpf_memcmp(buf, expected_buf, sizeof(buf));
		if (err)
			goto out;
	}
out:
	return XDP_DROP;
}
