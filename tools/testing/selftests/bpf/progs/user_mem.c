// SPDX-License-Identifier: GPL-2.0

#include <vmlinux.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

#include "bpf_misc.h"

char _license[] SEC("license") = "GPL";

extern unsigned long bpf_user_anon_mmap(unsigned long len, unsigned long prot) __ksym;
extern int bpf_user_munmap(unsigned long addr, unsigned long len) __ksym;

enum {
	USER_MEM_MMAP = 1,
	USER_MEM_UNMAP,
};

int monitored_pid;
int action;
int ret;

unsigned long user_addr;
unsigned long map_len;
unsigned long map_prot;

SEC("uprobe.s//proc/self/exe:user_mem_trigger")
int user_mem_test(struct pt_regs *ctx)
{
	if ((bpf_get_current_pid_tgid() >> 32) != monitored_pid)
		return 0;

	switch (action) {
	case USER_MEM_MMAP:
		user_addr = bpf_user_anon_mmap(map_len, map_prot);
		break;
	case USER_MEM_UNMAP:
		ret = bpf_user_munmap(user_addr, map_len);
		break;
	}

	return 0;
}
