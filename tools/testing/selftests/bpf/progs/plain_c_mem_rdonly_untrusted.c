// SPDX-License-Identifier: GPL-2.0
/* mem_rdonly_untrusted.c in plain C */

#include <vmlinux.h>
#include <bpf/bpf_core_read.h>
#include "bpf_misc.h"
#include "bpf_kfuncs.h"

SEC("socket")
__success
int offset_not_tracked(void *ctx)
{
	int *p, i, s;

	p = bpf_rdonly_cast(0, 0);
	s = 0;
	for (i = 0; i < 1000 * 1000 * 1000; i++) {
		p++;
		s += *p;
	}
	return s;
}

char _license[] SEC("license") = "GPL";
