/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef SELFTEST_KVM_TDX_TDX_H
#define SELFTEST_KVM_TDX_TDX_H

#include <linux/types.h>

u64 __tdvmcall(u64 fn, u64 r12, u64 r13, u64 r14, u64 r15);

#endif /* SELFTEST_KVM_TDX_TDX_H */
