/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __KVM_X86_VMX_POSTED_INTR_H
#define __KVM_X86_VMX_POSTED_INTR_H
#include <asm/posted_intr.h>

void vmx_vcpu_pi_load(struct kvm_vcpu *vcpu, int cpu);
void vmx_vcpu_pi_put(struct kvm_vcpu *vcpu);
void pi_wakeup_handler(void);
void __init pi_init_cpu(int cpu);
bool pi_has_pending_interrupt(struct kvm_vcpu *vcpu);
int vmx_pi_update_irte(struct kvm *kvm, unsigned int host_irq,
		       uint32_t guest_irq, bool set);
void vmx_pi_start_assignment(struct kvm *kvm);

#endif /* __KVM_X86_VMX_POSTED_INTR_H */
