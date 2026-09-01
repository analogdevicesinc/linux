/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * The host<->hyp hypercall interface.
 *
 * Copyright (C) 2026 Google LLC
 * Author: Fuad Tabba <fuad.tabba@linux.dev>
 */

#ifndef __ARM64_KVM_HCALL_H__
#define __ARM64_KVM_HCALL_H__

#include <linux/arm-smccc.h>
#include <linux/bug.h>
#include <linux/errno.h>
#include <linux/types.h>

#include <asm/barrier.h>
#include <asm/kvm_asm.h>
#include <asm/virt.h>

typedef u16 pkvm_handle_t;

#ifndef __KVM_NVHE_HYPERVISOR__
#define kvm_call_hyp_nvhe(f, ...)					\
	({								\
		struct arm_smccc_res res;				\
									\
		arm_smccc_1_1_hvc(KVM_HOST_SMCCC_FUNC(f),		\
				  ##__VA_ARGS__, &res);			\
		if (WARN_ON(res.a0 != SMCCC_RET_SUCCESS))		\
			res.a1 = -EOPNOTSUPP;				\
									\
		res.a1;							\
	})

/*
 * The isb() below is there to guarantee the same behaviour on VHE as on !VHE,
 * where the eret to EL1 acts as a context synchronization event.
 */
#define kvm_call_hyp(f, ...)						\
	do {								\
		if (has_vhe()) {					\
			f(__VA_ARGS__);					\
			isb();						\
		} else {						\
			kvm_call_hyp_nvhe(f, ##__VA_ARGS__);		\
		}							\
	} while (0)

#define kvm_call_hyp_ret(f, ...)					\
	({								\
		typeof(f(__VA_ARGS__)) ret;				\
									\
		if (has_vhe()) {					\
			ret = f(__VA_ARGS__);				\
		} else {						\
			ret = kvm_call_hyp_nvhe(f, ##__VA_ARGS__);	\
		}							\
									\
		ret;							\
	})
#else /* __KVM_NVHE_HYPERVISOR__ */
#define kvm_call_hyp(f, ...) f(__VA_ARGS__)
#define kvm_call_hyp_ret(f, ...) f(__VA_ARGS__)
#define kvm_call_hyp_nvhe(f, ...) f(__VA_ARGS__)
#endif /* __KVM_NVHE_HYPERVISOR__ */

#endif /* __ARM64_KVM_HCALL_H__ */
