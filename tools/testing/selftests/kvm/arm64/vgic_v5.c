// SPDX-License-Identifier: GPL-2.0

#include <linux/kernel.h>
#include <sys/syscall.h>
#include <asm/kvm.h>
#include <asm/kvm_para.h>

#include <arm64/gic_v5.h>

#include "test_util.h"
#include "kvm_util.h"
#include "processor.h"
#include "vgic.h"

#define NR_VCPUS		1
#define VGIC_V5_DEFAULT_NR_SPIS	32
#define VGIC_V5_MAX_NR_SPIS	BIT(10)

static u64 max_phys_size;

#define VGIC_V5_TEST_IST_BASER_GPA	0x10000000ULL

struct vm_gic {
	struct kvm_vm *vm;
	int gic_fd;
	u32 gic_dev_type;
};

#define GUEST_CMD_IRQ_CDIA	10
#define GUEST_CMD_IRQ_DIEOI	11
#define GUEST_CMD_IS_AWAKE	12
#define GUEST_CMD_IS_READY	13

static void guest_irq_handler(struct ex_regs *regs)
{
	bool valid;
	u32 hwirq;
	u64 ia;
	static int count;

	/*
	 * We have pending interrupts. Should never actually enter WFI
	 * here!
	 */
	wfi();
	GUEST_SYNC(GUEST_CMD_IS_AWAKE);

	ia = gicr_insn(CDIA);
	valid = GICV5_GICR_CDIA_VALID(ia);

	GUEST_SYNC(GUEST_CMD_IRQ_CDIA);

	if (!valid)
		return;

	gsb_ack();
	isb();

	hwirq = FIELD_GET(GICV5_GICR_CDIA_INTID, ia);

	gic_insn(hwirq, CDDI);
	gic_insn(0, CDEOI);

	GUEST_SYNC(GUEST_CMD_IRQ_DIEOI);

	if (++count >= 2)
		GUEST_DONE();

	/* Ask for the next interrupt to be injected */
	GUEST_SYNC(GUEST_CMD_IS_READY);
}

static void guest_code(void)
{
	local_irq_disable();

	gicv5_cpu_enable_interrupts();
	local_irq_enable();

	/* Enable the SW_PPI (3) */
	write_sysreg_s(BIT_ULL(3), SYS_ICC_PPI_ENABLER0_EL1);

	/* Ask for the first interrupt to be injected */
	GUEST_SYNC(GUEST_CMD_IS_READY);

	/* Loop forever waiting for interrupts */
	for (;;)
		cpu_relax();
}

/* we don't want to assert on run execution, hence that helper */
static int run_vcpu(struct kvm_vcpu *vcpu)
{
	return __vcpu_run(vcpu) ? -errno : 0;
}

static void vm_gic_destroy(struct vm_gic *v)
{
	close(v->gic_fd);
	kvm_vm_free(v->vm);
}

struct vgic_region_attr {
	u64 attr;
	u64 size;
	u64 alignment;
};

static const struct vgic_region_attr gic_v5_irs_region = {
	.attr = KVM_VGIC_V5_ADDR_TYPE_IRS,
	.size = GICV5_IRS_SIZE,
	.alignment = GICV5_IRS_ALIGN,
};

struct vgic_irs_reg_attr {
	const char	*name;
	u64		attr;
};

#define IRS_REG(r)						\
	{							\
		.name		= #r,				\
		.attr		= r,				\
	}

static const struct vgic_irs_reg_attr gic_v5_irs_regs[] = {
	IRS_REG(GICV5_IRS_IDR0),
	IRS_REG(GICV5_IRS_IDR1),
	IRS_REG(GICV5_IRS_IDR2),
	IRS_REG(GICV5_IRS_IDR3),
	IRS_REG(GICV5_IRS_IDR4),
	IRS_REG(GICV5_IRS_IDR5),
	IRS_REG(GICV5_IRS_IDR6),
	IRS_REG(GICV5_IRS_IDR7),
	IRS_REG(GICV5_IRS_IIDR),
	IRS_REG(GICV5_IRS_AIDR),
	IRS_REG(GICV5_IRS_CR0),
	IRS_REG(GICV5_IRS_CR1),
	IRS_REG(GICV5_IRS_SYNCR),
	IRS_REG(GICV5_IRS_SYNC_STATUSR),
	IRS_REG(GICV5_IRS_SPI_VMR),
	IRS_REG(GICV5_IRS_SPI_SELR),
	IRS_REG(GICV5_IRS_SPI_DOMAINR),
	IRS_REG(GICV5_IRS_SPI_RESAMPLER),
	IRS_REG(GICV5_IRS_SPI_CFGR),
	IRS_REG(GICV5_IRS_SPI_STATUSR),
	IRS_REG(GICV5_IRS_PE_SELR),
	IRS_REG(GICV5_IRS_PE_STATUSR),
	IRS_REG(GICV5_IRS_PE_CR0),
	IRS_REG(GICV5_IRS_IST_BASER),
	IRS_REG(GICV5_IRS_IST_CFGR),
	IRS_REG(GICV5_IRS_IST_STATUSR),
	IRS_REG(GICV5_IRS_MAP_L2_ISTR),
	IRS_REG(GICV5_IRS_VMT_BASER),
	IRS_REG(GICV5_IRS_VMT_CFGR),
	IRS_REG(GICV5_IRS_VMT_STATUSR),
	IRS_REG(GICV5_IRS_VPE_SELR),
	IRS_REG(GICV5_IRS_VPE_DBR),
	IRS_REG(GICV5_IRS_VPE_HPPIR),
	IRS_REG(GICV5_IRS_VPE_CR0),
	IRS_REG(GICV5_IRS_VPE_STATUSR),
	IRS_REG(GICV5_IRS_VM_DBR),
	IRS_REG(GICV5_IRS_VM_SELR),
	IRS_REG(GICV5_IRS_VM_STATUSR),
	IRS_REG(GICV5_IRS_VMAP_L2_VMTR),
	IRS_REG(GICV5_IRS_VMAP_VMR),
	IRS_REG(GICV5_IRS_VMAP_VISTR),
	IRS_REG(GICV5_IRS_VMAP_L2_VISTR),
	IRS_REG(GICV5_IRS_VMAP_VPER),
	IRS_REG(GICV5_IRS_SAVE_VMR),
	IRS_REG(GICV5_IRS_SAVE_VM_STATUSR),
	IRS_REG(GICV5_IRS_MEC_IDR),
	IRS_REG(GICV5_IRS_MEC_MECID_R),
	IRS_REG(GICV5_IRS_MPAM_IDR),
	IRS_REG(GICV5_IRS_MPAM_PARTID_R),
	IRS_REG(GICV5_IRS_SWERR_STATUSR),
	IRS_REG(GICV5_IRS_SWERR_SYNDROMER0),
	IRS_REG(GICV5_IRS_SWERR_SYNDROMER1),
};

static void test_vgic_v5_addr_attrs(void)
{
	struct kvm_vcpu *vcpu;
	struct vm_gic v;
	u64 addr;
	int ret;

	v.gic_dev_type = KVM_DEV_TYPE_ARM_VGIC_V5;
	v.vm = __vm_create(VM_SHAPE_DEFAULT, NR_VCPUS, 0);
	v.gic_fd = kvm_create_device(v.vm, v.gic_dev_type);

	/* Check existing group/attributes */
	kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR, gic_v5_irs_region.attr);

	/* check non existing attribute */
	ret = __kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR, -1);
	TEST_ASSERT(ret && errno == ENXIO, "attribute not supported");

	/* get IRS base address before setting*/
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(!ret && addr == (-1ULL), "GICv5 IRS returns VGIC_ADDR_UNDEF");

	/* misaligned IRS address settings */
	addr = gic_v5_irs_region.alignment / 0x10;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(ret && errno == EINVAL, "GIC IRS base not aligned");

	/* out of range address */
	addr = max_phys_size;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(ret && errno == E2BIG, "IRS address beyond IPA limit");

	/* Space for half an IRS (an IRS is: 2 * irs.alignment). */
	addr = max_phys_size - gic_v5_irs_region.alignment;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(ret && errno == E2BIG,
			"half of the IRS is beyond IPA limit");

	/* set IRS base address @0x0*/
	addr = 0x00000;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(!ret, "GICv5 IRS base correctly set");

	/* get IRS base address */
	addr = 0xbad;
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(!ret && addr == 0, "GICv5 IRS base correctly set");

	/* Attempt to create a second IRS region */
	addr = 0xE0000;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
				    KVM_VGIC_V5_ADDR_TYPE_IRS, &addr);
	TEST_ASSERT(ret && errno == EEXIST, "GICv5 IRS base set again");

	vm_gic_destroy(&v);

	/* Try running a VM without ever setting the IRS base addr */
	v.vm = __vm_create(VM_SHAPE_DEFAULT, NR_VCPUS, 0);
	v.gic_fd = kvm_create_device(v.vm, v.gic_dev_type);
	vcpu = vm_vcpu_add(v.vm, 0, NULL);
	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_CTRL,
			    KVM_DEV_ARM_VGIC_CTRL_INIT, NULL);
	ret = run_vcpu(vcpu);
	TEST_ASSERT(ret && errno == ENXIO, "GICv5 IRS base not set");

	vm_gic_destroy(&v);
}

static void test_vgic_v5_nr_irqs_attrs(void)
{
	struct kvm_vcpu *vcpu;
	struct vm_gic v;
	u32 nr_irqs;
	int ret;

	v.gic_dev_type = KVM_DEV_TYPE_ARM_VGIC_V5;
	v.vm = __vm_create(VM_SHAPE_DEFAULT, NR_VCPUS, 0);
	v.gic_fd = kvm_create_device(v.vm, v.gic_dev_type);

	/* Check existing group/attribute */
	kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS, 0);

	/* Before userspace sets NR_IRQS, no SPI count has been selected. */
	nr_irqs = 0xbad;
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(!ret && nr_irqs == 0, "GICv5 NR_IRQS defaults to 0 before init");

	/* Too few SPIs */
	nr_irqs = VGIC_V5_DEFAULT_NR_SPIS - 1;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 NR_IRQS below minimum");

	/* Not a multiple of 32 */
	nr_irqs = VGIC_V5_DEFAULT_NR_SPIS + 1;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 NR_IRQS not 32-aligned");

	/* Larger than KVM's supported VGICv5 SPI count */
	nr_irqs = VGIC_V5_MAX_NR_SPIS + 32;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 NR_IRQS above maximum");

	/* Valid custom SPI count */
	nr_irqs = VGIC_V5_MAX_NR_SPIS;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(!ret, "GICv5 NR_IRQS accepts valid custom SPI count");

	nr_irqs = 0xbad;
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(!ret && nr_irqs == VGIC_V5_MAX_NR_SPIS,
		    "GICv5 NR_IRQS returns SPI count only");

	/* A second successful configuration attempt must be rejected. */
	nr_irqs = VGIC_V5_DEFAULT_NR_SPIS;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(ret && errno == EBUSY, "GICv5 NR_IRQS set twice");

	/* The maximum supported count must also initialize successfully. */
	vcpu = vm_vcpu_add(v.vm, 0, NULL);
	TEST_ASSERT(vcpu, "Failed to create vCPU");
	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_CTRL,
			    KVM_DEV_ARM_VGIC_CTRL_INIT, NULL);

	vm_gic_destroy(&v);

	/* If userspace does not set NR_IRQS, init selects the default. */
	v.vm = __vm_create(VM_SHAPE_DEFAULT, NR_VCPUS, 0);
	v.gic_fd = kvm_create_device(v.vm, v.gic_dev_type);
	vcpu = vm_vcpu_add(v.vm, 0, NULL);
	TEST_ASSERT(vcpu, "Failed to create vCPU");
	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_CTRL,
			    KVM_DEV_ARM_VGIC_CTRL_INIT, NULL);

	nr_irqs = 0xbad;
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(!ret && nr_irqs == VGIC_V5_DEFAULT_NR_SPIS,
		    "GICv5 NR_IRQS defaults to 32 SPIs after init");

	nr_irqs = VGIC_V5_DEFAULT_NR_SPIS * 2;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_NR_IRQS,
				    0, &nr_irqs);
	TEST_ASSERT(ret && errno == EBUSY, "GICv5 NR_IRQS set after init");

	vm_gic_destroy(&v);

}

static void test_vgic_v5_irs_regs_attrs(void)
{
	struct kvm_vcpu *vcpu;
	struct vm_gic v;
	u64 attr, val;
	int ret, i;

	v.gic_dev_type = KVM_DEV_TYPE_ARM_VGIC_V5;
	v.vm = __vm_create(VM_SHAPE_DEFAULT, NR_VCPUS, 0);
	v.gic_fd = kvm_create_device(v.vm, v.gic_dev_type);
	vcpu = vm_vcpu_add(v.vm, 0, NULL);
	TEST_ASSERT(vcpu, "Failed to create vCPU");

	/* IRS_REGS attributes can be probed before the IRS base is set. */
	kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
			    GICV5_IRS_IDR0);

	attr = GICV5_IRS_CONFIG_BASE_GPA;
	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
			    KVM_VGIC_V5_ADDR_TYPE_IRS, &attr);

	/* Check existing group/attribute */
	kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
			    GICV5_IRS_IDR0);

	/* IRS_REGS are not accessible before the VGIC is initialized. */
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, &val);
	TEST_ASSERT(ret && errno == EBUSY, "GICv5 IRS_REGS get before init");

	val = 0;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, &val);
	TEST_ASSERT(ret && errno == EBUSY, "GICv5 IRS_REGS set before init");

	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_CTRL,
			    KVM_DEV_ARM_VGIC_CTRL_INIT, NULL);

	/* Read all supported IRS regs and write the value back. */
	for (i = 0; i < ARRAY_SIZE(gic_v5_irs_regs); i++) {
		attr = gic_v5_irs_regs[i].attr;
		ret = __kvm_has_device_attr(v.gic_fd,
					    KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
					    attr);
		TEST_ASSERT(!ret, "GICv5 IRS_REGS missing %s",
			    gic_v5_irs_regs[i].name);

		val = 0xbad;
		ret = __kvm_device_attr_get(v.gic_fd,
					    KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
					    attr, &val);
		TEST_ASSERT(!ret, "GICv5 IRS_REGS get failed for %s",
			    gic_v5_irs_regs[i].name);

		ret = __kvm_device_attr_set(v.gic_fd,
					    KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
					    attr, &val);
		TEST_ASSERT(!ret, "GICv5 IRS_REGS set failed for %s",
			    gic_v5_irs_regs[i].name);
	}

	/* Check bad offsets */
	attr = 0x10000;
	ret = __kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    attr);
	TEST_ASSERT(ret && errno == ENXIO, "GICv5 IRS_REGS accepted bad offset");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    attr, &val);
	TEST_ASSERT(ret && errno == ENXIO, "GICv5 IRS_REGS get bad offset");

	/* Check alignment for 32-bit and 64-bit IRS regs. */
	attr = GICV5_IRS_IDR0 + 2;
	ret = __kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    attr);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted unaligned 32-bit attr");

	attr = GICV5_IRS_IST_BASER + 4;
	ret = __kvm_has_device_attr(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    attr);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted unaligned 64-bit attr");

	/* Check bad user pointers */
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, NULL);
	TEST_ASSERT(ret && errno == EFAULT, "GICv5 IRS_REGS get with bad pointer");

	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, NULL);
	TEST_ASSERT(ret && errno == EFAULT, "GICv5 IRS_REGS set with bad pointer");

	/* ID restore validation rejects unsupported values. */
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS get IDR0 failed");
	val &= ~GICV5_IRS_IDR0_INT_DOM;
	val |= FIELD_PREP(GICV5_IRS_IDR0_INT_DOM,
			  GICV5_IRS_IDR0_INT_DOM_SECURE);
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted bad IDR0 domain");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS get IDR0 failed");
	val |= GICV5_IRS_IDR0_SETLPI;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR0, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted unsupported IDR0");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR1, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS get IDR1 failed");
	val |= FIELD_PREP(GICV5_IRS_IDR1_PRIORITY_BITS, 0x7);
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR1, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted bad IDR1");

	val = 0;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR2, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted bad IDR2");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR5, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS get IDR5 failed");
	val++;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR5, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted bad IDR5");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR6, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS get IDR6 failed");
	val++;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR6, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted bad IDR6");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR7, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS get IDR7 failed");
	val++;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IDR7, &val);
	TEST_ASSERT(ret && errno == EINVAL, "GICv5 IRS_REGS accepted bad IDR7");

	/* Status registers read as idle through userspace accessors. */
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_SYNC_STATUSR, &val);
	TEST_ASSERT(!ret && val == GICV5_IRS_SYNC_STATUSR_IDLE,
		    "GICv5 IRS_REGS SYNC_STATUSR is not idle");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_SPI_STATUSR, &val);
	TEST_ASSERT(!ret && val == GICV5_IRS_SPI_STATUSR_IDLE,
		    "GICv5 IRS_REGS SPI_STATUSR is not idle");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_PE_STATUSR, &val);
	TEST_ASSERT(!ret && val == GICV5_IRS_PE_STATUSR_IDLE,
		    "GICv5 IRS_REGS PE_STATUSR is not idle");

	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IST_STATUSR, &val);
	TEST_ASSERT(!ret && val == GICV5_IRS_IST_STATUSR_IDLE,
		    "GICv5 IRS_REGS IST_STATUSR is not idle");

	/*
	 * Userspace restores IST_BASER without allocating or accessing an IST.
	 * Supply a synthetic, aligned guest address only to verify the register
	 * round trip.
	 */
	val = FIELD_PREP(GICV5_IRS_IST_BASER_ADDR_MASK,
			 VGIC_V5_TEST_IST_BASER_GPA >> GICV5_IRS_IST_BASER_ADDR_SHIFT);
	val |= GICV5_IRS_IST_BASER_VALID;
	ret = __kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IST_BASER, &val);
	TEST_ASSERT(!ret, "GICv5 IRS_REGS failed to restore IST_BASER");

	attr = 0xbad;
	ret = __kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_IRS_REGS,
				    GICV5_IRS_IST_BASER, &attr);
	TEST_ASSERT(!ret && attr == val, "GICv5 IRS_REGS IST_BASER restore mismatch");

	vm_gic_destroy(&v);
}

static void test_vgic_v5_ppis(u32 gic_dev_type)
{
	struct kvm_vcpu *vcpus[NR_VCPUS];
	struct ucall uc;
	u64 user_ppis[2];
	struct vm_gic v;
	uint64_t attr;
	int ret, i;

	v.gic_dev_type = gic_dev_type;
	v.vm = __vm_create(VM_SHAPE_DEFAULT, NR_VCPUS, 0);

	v.gic_fd = kvm_create_device(v.vm, gic_dev_type);

	for (i = 0; i < NR_VCPUS; i++)
		vcpus[i] = vm_vcpu_add(v.vm, i, guest_code);

	vm_init_descriptor_tables(v.vm);
	vm_install_exception_handler(v.vm, VECTOR_IRQ_CURRENT, guest_irq_handler);

	for (i = 0; i < NR_VCPUS; i++)
		vcpu_init_descriptor_tables(vcpus[i]);

	/* Set the address of the IRS before initialising the GIC */
	attr = GICV5_IRS_CONFIG_BASE_GPA;
	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_ADDR,
			    KVM_VGIC_V5_ADDR_TYPE_IRS, &attr);

	kvm_device_attr_set(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_CTRL,
			    KVM_DEV_ARM_VGIC_CTRL_INIT, NULL);

	/* Read out the PPIs that user space is allowed to drive. */
	kvm_device_attr_get(v.gic_fd, KVM_DEV_ARM_VGIC_GRP_CTRL,
			    KVM_DEV_ARM_VGIC_USERSPACE_PPIS, &user_ppis);

	/* We should always be able to drive the SW_PPI. */
	TEST_ASSERT(user_ppis[0] & BIT(GICV5_ARCH_PPI_SW_PPI),
		"SW_PPI is not drivable by userspace");

	while (1) {
		ret = run_vcpu(vcpus[0]);
		if (ret)
			break;

		switch (get_ucall(vcpus[0], &uc)) {
		case UCALL_SYNC:
			/*
			 * The guest is ready for the next level change. Set
			 * high if ready, and lower if it has been consumed.
			 */
			if (uc.args[1] == GUEST_CMD_IS_READY ||
			    uc.args[1] == GUEST_CMD_IRQ_DIEOI) {
				u64 irq;
				bool level = uc.args[1] == GUEST_CMD_IRQ_DIEOI ? 0 : 1;

				irq = FIELD_PREP(KVM_ARM_IRQ_NUM_MASK, 3);
				irq |= KVM_ARM_IRQ_TYPE_PPI << KVM_ARM_IRQ_TYPE_SHIFT;

				kvm_irq_line(v.vm, irq, level);
			} else if (uc.args[1] == GUEST_CMD_IS_AWAKE) {
				pr_info("Guest skipping WFI due to pending IRQ\n");
			} else if (uc.args[1] == GUEST_CMD_IRQ_CDIA) {
				pr_info("Guest acknowledged IRQ\n");
			}

			continue;
		case UCALL_ABORT:
			REPORT_GUEST_ASSERT(uc);
			break;
		case UCALL_DONE:
			goto done;
		default:
			TEST_FAIL("Unknown ucall %lu", uc.cmd);
		}
	}

done:
	TEST_ASSERT(ret == 0, "Failed to test GICv5 PPIs");

	vm_gic_destroy(&v);
}

/*
 * Returns 0 if it's possible to create GIC device of a given type (V5).
 */
int test_kvm_device(u32 gic_dev_type)
{
	struct kvm_vcpu *vcpus[NR_VCPUS];
	struct vm_gic v;
	int ret;

	v.vm = vm_create_with_vcpus(NR_VCPUS, guest_code, vcpus);

	/* try to create a non existing KVM device */
	ret = __kvm_test_create_device(v.vm, 0);
	TEST_ASSERT(ret && errno == ENODEV, "unsupported device");

	/* trial mode */
	ret = __kvm_test_create_device(v.vm, gic_dev_type);
	if (ret)
		return ret;
	v.gic_fd = kvm_create_device(v.vm, gic_dev_type);

	ret = __kvm_create_device(v.vm, gic_dev_type);
	TEST_ASSERT(ret < 0 && errno == EEXIST, "create GIC device twice");

	vm_gic_destroy(&v);

	return 0;
}

void run_tests(u32 gic_dev_type)
{
	pr_info("Test VGICv5 address attrs\n");
	test_vgic_v5_addr_attrs();

	pr_info("Test VGICv5 NR_IRQS attrs\n");
	test_vgic_v5_nr_irqs_attrs();

	pr_info("Test VGICv5 IRS_REGS attrs\n");
	test_vgic_v5_irs_regs_attrs();


	pr_info("Test VGICv5 PPIs\n");
	test_vgic_v5_ppis(gic_dev_type);
}

int main(int ac, char **av)
{
	int pa_bits, ret;

	pa_bits = vm_guest_mode_params[VM_MODE_DEFAULT].pa_bits;
	max_phys_size = 1ULL << pa_bits;

	test_disable_default_vgic();

	ret = test_kvm_device(KVM_DEV_TYPE_ARM_VGIC_V5);
	if (ret) {
		pr_info("No GICv5 support; Not running GIC_v5 tests.\n");
		exit(KSFT_SKIP);
	}

	pr_info("Running VGIC_V5 tests.\n");
	run_tests(KVM_DEV_TYPE_ARM_VGIC_V5);

	return 0;
}
