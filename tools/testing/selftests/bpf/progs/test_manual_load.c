// SPDX-License-Identifier: GPL-2.0

#include "vmlinux.h"
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

bool prog1_called = false;
bool prog2_called = false;
bool prog3_called = false;
bool prog4_called = false;
__u32 prog5_sz = 0;
__u32 test_pid = 0;

SEC("raw_tp/sys_enter")
int prog1(const void *ctx)
{
	if ((bpf_get_current_pid_tgid() >> 32) != test_pid)
		return 0;

	prog1_called = true;
	return 0;
}

SEC("raw_tp/sys_enter")
int prog2(const void *ctx)
{
	if ((bpf_get_current_pid_tgid() >> 32) != test_pid)
		return 0;

	prog2_called = true;
	return 0;
}

SEC("raw_tp/sys_enter")
int prog3(const void *ctx)
{
	if ((bpf_get_current_pid_tgid() >> 32) != test_pid)
		return 0;

	prog3_called = true;
	return 0;
}

SEC("!raw_tp/sys_enter")
int prog4(const void *ctx)
{
	if ((bpf_get_current_pid_tgid() >> 32) != test_pid)
		return 0;

	prog4_called = true;
	return 0;
}

/*
 * disabled at parse time; its attach target is resolved against module
 * BTF via bpf_program__set_attach_target() before it is switched to
 * MANUAL and loaded
 */
SEC("?fentry")
int BPF_PROG(prog5, struct file *file, struct kobject *kobj,
	     const struct bin_attribute *bin_attr, char *buf, loff_t off, size_t len)
{
	if ((bpf_get_current_pid_tgid() >> 32) != test_pid)
		return 0;

	prog5_sz = len;
	return 0;
}

char _license[] SEC("license") = "GPL";
