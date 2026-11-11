// SPDX-License-Identifier: GPL-2.0-only
/*
 * Test physical APIC ID routing for xAPIC-mode vCPUs with aliased IDs.
 */

#include <sys/ioctl.h>

#include "processor.h"
#include "test_util.h"
#include "kvm_util.h"
#include "apic.h"

#define MAX_XAPIC_ID		0xff

#define ALIAS_LOW_VCPU_ID	0x1d	/* i.e. a non-broadcast xAPIC ID */
#define ALIAS_HIGH_VCPU_ID	(ALIAS_LOW_VCPU_ID | 0x100)
#define ALIAS_VECTOR		0xe0
#define FULL_ID_VECTOR		0xe1

static void assert_lapic_xapic_mode(struct kvm_vcpu *vcpu)
{
	u64 apicbase = vcpu_get_msr(vcpu, MSR_IA32_APICBASE);

	TEST_ASSERT(apicbase & MSR_IA32_APICBASE_ENABLE,
		    "vCPU %u local APIC not in ENABLED state at vCPU RESET",
		    vcpu->id);
	TEST_ASSERT(!(apicbase & X2APIC_ENABLE),
		    "vCPU %u local APIC not in xAPIC mode at vCPU RESET",
		    vcpu->id);
}

static void assert_lapic_xapic_id(struct kvm_vcpu *vcpu, u32 expected_id)
{
	struct kvm_lapic_state lapic;
	u32 apic_id;

	vcpu_ioctl(vcpu, KVM_GET_LAPIC, &lapic);
	apic_id = *(u32 *)(lapic.regs + APIC_ID);
	apic_id = GET_APIC_ID_FIELD(apic_id);

	TEST_ASSERT(apic_id == expected_id,
		    "vCPU %u: expected xAPIC ID %u, read %u",
		    vcpu->id, expected_id, apic_id);
}

/* Set APIC Software Enable bit in SPIV to allow fixed interrupt delivery */
static void lapic_sw_enable(struct kvm_vcpu *vcpu)
{
	struct kvm_lapic_state lapic;

	vcpu_ioctl(vcpu, KVM_GET_LAPIC, &lapic);
	*(u32 *)(lapic.regs + APIC_SPIV) |= APIC_SPIV_APIC_ENABLED;
	vcpu_ioctl(vcpu, KVM_SET_LAPIC, &lapic);
}

/* Determine if @vector is marked pending in @vcpu's LAPIC IRR register */
static bool lapic_irr_has_vector(struct kvm_vcpu *vcpu, u8 vector)
{
	struct kvm_lapic_state lapic;
	u32 irr;

	vcpu_ioctl(vcpu, KVM_GET_LAPIC, &lapic);
	irr = *((u32 *)(lapic.regs + APIC_IRR +
		APIC_VECTOR_TO_REG_OFFSET(vector)));

	return irr & BIT(APIC_VECTOR_TO_BIT_NUMBER(vector));
}

static void inject_msi(struct kvm_vm *vm, u32 dest_id, u8 vector)
{
	struct kvm_msi msi = {
		/*
		 * On x86, address_lo bits [31:20] contain the MSI address
		 * prefix (0xfee), and bits [19:12] encode destination ID bits
		 * [7:0].
		 * destination mode (bit 2) is clear for physical addressing.
		 */
		.address_lo = APIC_DEFAULT_GPA | ((dest_id & 0xff) << 12),
		/*
		 * On x86 with KVM_X2APIC_API_USE_32BIT_IDS feature enabled,
		 * address_hi bits [31:8] encode bits [31:8] of the destination
		 * id. Bits [7:0] of address_hi must be zero.
		 */
		.address_hi = dest_id & 0xffffff00,
		/*
		 * Data bits [7:0] contain the vector. delivery mode bits [10:8]
		 * and trigger mode (bit 15) are set to 0 for fixed,
		 * edge-triggered delivery.
		 */
		.data = vector & 0xff,
	};

	TEST_ASSERT(__vm_ioctl(vm, KVM_SIGNAL_MSI, &msi) == 1,
		    "KVM_SIGNAL_MSI ioctl failed");
}

static void report_irq_delivery(bool test_passed, bool high_first,
				u32 dest_id, u8 vector,
				bool high_has_irq, bool low_has_irq)
{
	const char *first_id_type = high_first ? "Higher" : "Lower";
	u32 first_vcpu_id = high_first ? ALIAS_HIGH_VCPU_ID : ALIAS_LOW_VCPU_ID;

	ksft_test_result(test_passed,
			 "Destination %#x: %s-ID vCPU %u created first.\n",
			 dest_id, first_id_type, first_vcpu_id);
	ksft_print_msg("Expected IRQ only on vCPU %u.\n", dest_id);
	ksft_print_msg("Higher-ID vCPU %u: IRQ vector %#x %s.\n",
		       ALIAS_HIGH_VCPU_ID, vector,
		       high_has_irq ? "delivered" : "not delivered");
	ksft_print_msg("Lower-ID vCPU %u: IRQ vector %#x %s.\n",
		       ALIAS_LOW_VCPU_ID, vector,
		       low_has_irq ? "delivered" : "not delivered");
}

static void test_apic_id_routing(bool high_first)
{
	struct kvm_vcpu *low_vcpu, *high_vcpu;
	struct kvm_vm *vm;
	bool high_has_irq;
	bool low_has_irq;

	/* Create a VM with two vCPUs and enable 32-bit APIC IDs */
	vm = vm_create(2);
	vm_enable_cap(vm, KVM_CAP_X2APIC_API, KVM_X2APIC_API_USE_32BIT_IDS);

	/*
	 * Create two vCPUs which alias to the same physical xAPIC ID.
	 * With KVM_X2APIC_API_USE_32BIT_IDS enabled and vCPUs in xAPIC mode,
	 * interrupts must be delivered to the vCPU whose vCPU ID matches the
	 * xAPIC ID, regardless of vCPU creation order.
	 */
	if (high_first) {
		high_vcpu = vm_vcpu_add(vm, ALIAS_HIGH_VCPU_ID, NULL);
		low_vcpu = vm_vcpu_add(vm, ALIAS_LOW_VCPU_ID, NULL);
	} else {
		low_vcpu = vm_vcpu_add(vm, ALIAS_LOW_VCPU_ID, NULL);
		high_vcpu = vm_vcpu_add(vm, ALIAS_HIGH_VCPU_ID, NULL);
	}

	/* Ensure local APICs are enabled and in xAPIC mode */
	assert_lapic_xapic_mode(high_vcpu);
	assert_lapic_xapic_mode(low_vcpu);
	lapic_sw_enable(high_vcpu);
	lapic_sw_enable(low_vcpu);

	/* Verify both LAPICs expose the shared physical destination ID. */
	assert_lapic_xapic_id(high_vcpu, ALIAS_LOW_VCPU_ID);
	assert_lapic_xapic_id(low_vcpu, ALIAS_LOW_VCPU_ID);

	/* The vector to be tested must not be pending at this point */
	TEST_ASSERT(!lapic_irr_has_vector(high_vcpu, ALIAS_VECTOR),
		    "vCPU %u: vector %#x already pending before MSI injection",
		    high_vcpu->id, ALIAS_VECTOR);
	TEST_ASSERT(!lapic_irr_has_vector(low_vcpu, ALIAS_VECTOR),
		    "vCPU %u: vector %#x already pending before MSI injection",
		    low_vcpu->id, ALIAS_VECTOR);

	/*
	 * Inject an MSI message to the shared xAPIC ID. The vCPUs are not
	 * yet running, so KVM_GET_LAPIC can be used later to check for pending
	 * interrupts in IRR, and confirm interrupt was delivered to expected
	 * vCPU.
	 */
	inject_msi(vm, ALIAS_LOW_VCPU_ID, ALIAS_VECTOR);

	high_has_irq = lapic_irr_has_vector(high_vcpu, ALIAS_VECTOR);
	low_has_irq = lapic_irr_has_vector(low_vcpu, ALIAS_VECTOR);

	report_irq_delivery(!high_has_irq && low_has_irq,
			    high_first, ALIAS_LOW_VCPU_ID, ALIAS_VECTOR,
			    high_has_irq, low_has_irq);

	/*
	 * The higher-ID vCPU must remain reachable by its full x2APIC ID
	 * while in xAPIC mode, to allow vCPU hotplug. Use a different vector
	 * to avoid conflicts with the previous aliased test case.
	 */
	TEST_ASSERT(!lapic_irr_has_vector(high_vcpu, FULL_ID_VECTOR),
		    "vCPU %u: vector %#x already pending before MSI injection",
		    high_vcpu->id, FULL_ID_VECTOR);
	TEST_ASSERT(!lapic_irr_has_vector(low_vcpu, FULL_ID_VECTOR),
		    "vCPU %u: vector %#x already pending before MSI injection",
		    low_vcpu->id, FULL_ID_VECTOR);

	inject_msi(vm, ALIAS_HIGH_VCPU_ID, FULL_ID_VECTOR);

	high_has_irq = lapic_irr_has_vector(high_vcpu, FULL_ID_VECTOR);
	low_has_irq = lapic_irr_has_vector(low_vcpu, FULL_ID_VECTOR);

	report_irq_delivery(high_has_irq && !low_has_irq,
			    high_first, ALIAS_HIGH_VCPU_ID, FULL_ID_VECTOR,
			    high_has_irq, low_has_irq);
	kvm_vm_free(vm);
}

int main(void)
{
	kvm_static_assert(ALIAS_HIGH_VCPU_ID > MAX_XAPIC_ID);
	kvm_static_assert((ALIAS_HIGH_VCPU_ID & MAX_XAPIC_ID) ==
			  ALIAS_LOW_VCPU_ID);

	ksft_print_header();

	TEST_REQUIRE(kvm_check_cap(KVM_CAP_X2APIC_API) &
		     KVM_X2APIC_API_USE_32BIT_IDS);
	TEST_REQUIRE(kvm_has_cap(KVM_CAP_SIGNAL_MSI));
	/* Ensure max vcpu id is enough to hold high aliased ID */
	TEST_REQUIRE(kvm_check_cap(KVM_CAP_MAX_VCPU_ID) > ALIAS_HIGH_VCPU_ID);

	ksft_set_plan(4);

	test_apic_id_routing(false);
	test_apic_id_routing(true);

	ksft_finished();
}
