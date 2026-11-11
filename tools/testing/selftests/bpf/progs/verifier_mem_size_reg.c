// SPDX-License-Identifier: GPL-2.0

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_endian.h>
#include "bpf_misc.h"
#include "bpf_kfuncs.h"

char _license[] SEC("license") = "GPL";

/*
 * A size that has to be a known constant is one of two constants, and a compiler makes one call
 * of the two. The constants must not be joined on the way to the call.
 */
SEC("?tc")
__description("one of two constants: __szk, the header of IPv4 or of IPv6")
__success
int dynptr_slice_two_sizes(struct __sk_buff *skb)
{
	char buf[sizeof(struct ipv6hdr)];
	__u32 sz = sizeof(struct iphdr);
	struct bpf_dynptr dptr;
	__u8 *p;

	bpf_dynptr_from_skb(skb, 0, &dptr);

	if (skb->protocol != bpf_htons(0x0800))
		sz = sizeof(struct ipv6hdr);
	p = bpf_dynptr_slice(&dptr, 14, buf, sz);
	if (p)
		return p[0] >> 4;
	return 0;
}

struct {
	__uint(type, BPF_MAP_TYPE_RINGBUF);
	__uint(max_entries, 4096);
} ringbuf SEC(".maps");

SEC("?tc")
__description("one of two constants: the size of bpf_ringbuf_reserve()")
__success
int ringbuf_reserve_two_sizes(struct __sk_buff *skb)
{
	__u32 sz = 8;
	char *p;

	if (skb->len > 100)
		sz = 16;
	p = bpf_ringbuf_reserve(&ringbuf, sz, 0);
	if (!p)
		return 0;
	p[7] = 1;
	bpf_ringbuf_discard(p, 0);
	return 0;
}

/* The __szk size of a kfunc memory/size pair when the nullable buffer is passed as NULL */
SEC("?tc")
__success
int dynptr_slice_null_buf_size_precise(struct __sk_buff *skb)
{
	struct bpf_dynptr dptr;
	char *p;

	bpf_dynptr_from_skb(skb, 0, &dptr);

	p = bpf_dynptr_slice(&dptr, 0, NULL, 8);
	if (p)
		return p[0];
	return 0;
}
