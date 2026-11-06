/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SELFTEST_KVM_TDX_TDX_H
#define SELFTEST_KVM_TDX_TDX_H

#include <linux/types.h>

/* TDX hypercall Leaf IDs */
#define TDVMCALL_GET_TD_VM_CALL_INFO		0x10000
#define TDVMCALL_MAP_GPA			0x10001
#define TDVMCALL_GET_QUOTE			0x10002
#define TDVMCALL_REPORT_FATAL_ERROR		0x10003
#define TDVMCALL_SETUP_EVENT_NOTIFY_INTERRUPT	0x10004

u64 __tdvmcall(u64 fn, u64 r12, u64 r13, u64 r14, u64 r15);

#endif /* SELFTEST_KVM_TDX_TDX_H */
