// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2025 ARM Limited, All Rights Reserved.
 */
#include <linux/bitops.h>
#include <linux/bsearch.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kvm.h>
#include <linux/kvm_host.h>
#include <kvm/iodev.h>
#include <kvm/arm_arch_timer.h>
#include <kvm/arm_vgic.h>

#include "vgic.h"
#include "vgic-mmio.h"
#include "vgic-v5-tables.h"

#define irs_caps kvm_vgic_global_state.vgic_v5_irs_caps

static struct vgic_dist *vgic_v5_get_vgic(struct kvm_vcpu *vcpu)
{
	return &vcpu->kvm->arch.vgic;
}

static struct vgic_v5_irs *vgic_v5_get_irs(struct kvm_vcpu *vcpu)
{
	return vcpu->kvm->arch.vgic.vgic_v5_irs_data;
}

static unsigned long vgic_v5_mmio_read_irs_misc(struct kvm_vcpu *vcpu,
						gpa_t addr, unsigned int len)
{
	struct vgic_v5_irs *irs = vgic_v5_get_irs(vcpu);
	const size_t offset = addr & (SZ_64K - 1);
	struct kvm_vcpu *target_vcpu;
	u8 vpe_id_bits;
	u64 value = 0;

	switch (offset) {
	case GICV5_IRS_IDR0:
		value = FIELD_PREP(GICV5_IRS_IDR0_INT_DOM, irs->idr0.domain);
		value |= FIELD_PREP(GICV5_IRS_IDR0_PA_RANGE, irs->idr0.pa_range);
		if (irs->idr0.virt)
			value |= GICV5_IRS_IDR0_VIRT;
		if (irs->idr0.setlpi)
			value |= GICV5_IRS_IDR0_SETLPI;
		if (irs->idr0.mec)
			value |= GICV5_IRS_IDR0_MEC;
		if (irs->idr0.mpam)
			value |= GICV5_IRS_IDR0_MPAM;
		if (irs->idr0.swe)
			value |= GICV5_IRS_IDR0_SWE;
		value |= FIELD_PREP(GICV5_IRS_IDR0_IRSID, irs->idr0.irs_id);
		break;
	case GICV5_IRS_IDR1:
		value = FIELD_PREP(GICV5_IRS_IDR1_PE_CNT,
				   atomic_read(&vcpu->kvm->online_vcpus));
		/*
		 * IRS_IDR1 encodes IAFFID_BITS as N - 1.
		 */
		vpe_id_bits = vgic_v5_vmte_vpe_id_bits(vcpu);
		value |= FIELD_PREP(GICV5_IRS_IDR1_IAFFID_BITS, vpe_id_bits - 1);
		value |= FIELD_PREP(GICV5_IRS_IDR1_PRIORITY_BITS, irs->idr1.priority_bits);
		break;
	case GICV5_IRS_IDR2:
		value = FIELD_PREP(GICV5_IRS_IDR2_ISTMD_SZ, irs->idr2.istmd_sz);
		if (irs->idr2.istmd)
			value |= GICV5_IRS_IDR2_ISTMD;
		value |= FIELD_PREP(GICV5_IRS_IDR2_IST_L2SZ, irs->idr2.ist_l2sz);
		if (irs->idr2.ist_levels)
			value |= GICV5_IRS_IDR2_IST_LEVELS;
		value |= FIELD_PREP(GICV5_IRS_IDR2_MIN_LPI_ID_BITS, irs->idr2.min_lpi_id_bits);
		value |= GICV5_IRS_IDR2_LPI;
		value |= FIELD_PREP(GICV5_IRS_IDR2_ID_BITS, irs->idr2.id_bits);
		break;
	case GICV5_IRS_IDR5:
		value = FIELD_PREP(GICV5_IRS_IDR5_SPI_RANGE, irs->idr5.spi_range);
		break;
	case GICV5_IRS_IDR6:
		value = FIELD_PREP(GICV5_IRS_IDR6_SPI_IRS_RANGE, irs->idr6.spi_irs_range);
		break;
	case GICV5_IRS_IDR7:
		value = FIELD_PREP(GICV5_IRS_IDR7_SPI_BASE, irs->idr7.spi_base);
		break;
	case GICV5_IRS_IIDR:
		/* Revision, Variant, ProductID are implementation defined */
		value = FIELD_PREP(GICV5_IRS_IIDR_PRODUCT_ID, PRODUCT_ID_KVM);
		value |= FIELD_PREP(GICV5_IRS_IIDR_VARIANT, 0);
		value |= FIELD_PREP(GICV5_IRS_IIDR_REVISION, 0);
		value |= FIELD_PREP(GICV5_IRS_IIDR_IMPLEMENTER, IMPLEMENTER_ARM);
		break;
	case GICV5_IRS_AIDR:
		value = FIELD_PREP(GICV5_IRS_AIDR_COMPONENT,
				   GICV5_AIDR_COMPONENT_IRS);
		value |= FIELD_PREP(GICV5_IRS_AIDR_ARCHMAJORREV,
				    GICV5_AIDR_ARCH_MAJ_REV_V5);
		value |= FIELD_PREP(GICV5_IRS_AIDR_ARCHMINORREV,
				    GICV5_AIDR_ARCH_MIN_REV_V0);
		break;
	case GICV5_IRS_CR0:
		/*
		 * The IRS is ALWAYS idle as we handle things instantaneously
		 * from a guest's viewpoint.
		 */
		value = GICV5_IRS_CR0_IDLE;
		if (READ_ONCE(vcpu->kvm->arch.vgic.enabled))
			value |= GICV5_IRS_CR0_IRSEN;
		break;
	case GICV5_IRS_CR1:
		if (irs->cr1.vped_wa)
			value |= GICV5_IRS_CR1_VPED_WA;
		if (irs->cr1.vped_ra)
			value |= GICV5_IRS_CR1_VPED_RA;
		if (irs->cr1.vmd_wa)
			value |= GICV5_IRS_CR1_VMD_WA;
		if (irs->cr1.vmd_ra)
			value |= GICV5_IRS_CR1_VMD_RA;
		if (irs->cr1.vpet_ra)
			value |= GICV5_IRS_CR1_VPET_RA;
		if (irs->cr1.vmt_ra)
			value |= GICV5_IRS_CR1_VMT_RA;
		if (irs->cr1.ist_wa)
			value |= GICV5_IRS_CR1_IST_WA;
		if (irs->cr1.ist_ra)
			value |= GICV5_IRS_CR1_IST_RA;
		value |= FIELD_PREP(GICV5_IRS_CR1_IC, irs->cr1.ic);
		value |= FIELD_PREP(GICV5_IRS_CR1_OC, irs->cr1.oc);
		value |= FIELD_PREP(GICV5_IRS_CR1_SH, irs->cr1.sh);
		break;
	case GICV5_IRS_SYNC_STATUSR:
		value = GICV5_IRS_SYNC_STATUSR_IDLE;
		break;
	case GICV5_IRS_PE_SELR:
		value = FIELD_PREP(GICV5_IRS_PE_SELR_IAFFID,
				   READ_ONCE(irs->pe_selr.iaffid));
		break;
	case GICV5_IRS_PE_STATUSR:
		/* We assume that the PE is Online if present. Always IDLE too */
		value = GICV5_IRS_PE_STATUSR_IDLE;

		/* Set ONLINE and V if IAFFID selects a present PE */
		if (kvm_get_vcpu_by_id(vcpu->kvm,
				       READ_ONCE(irs->pe_selr.iaffid))) {
			value |= GICV5_IRS_PE_STATUSR_ONLINE;
			value |= GICV5_IRS_PE_STATUSR_V;
		}
		break;
	case GICV5_IRS_PE_CR0: {
		u32 iaffid = READ_ONCE(irs->pe_selr.iaffid);

		/*
		 * Make sure that we are doing something reasonable first.
		 * Remember, the IAFFID is the same as the VPE_ID
		 */
		target_vcpu = kvm_get_vcpu_by_id(vcpu->kvm, iaffid);
		if (!target_vcpu) {
			kvm_debug("Guest programmed invalid IAFFID (0x%x) into the IRS_PE_SELR\n",
				  iaffid);
			break;
		}

		value = GICV5_IRS_PE_CR0_DPS;
		break;
	}
	default:
		return 0;
	}

	return value;
}

static void vgic_v5_mmio_write_irs_misc(struct kvm_vcpu *vcpu, gpa_t addr,
					unsigned int len, unsigned long val)
{
	struct vgic_v5_irs *irs = vgic_v5_get_irs(vcpu);
	struct vgic_dist *vgic = vgic_v5_get_vgic(vcpu);
	const size_t offset = addr & (SZ_64K - 1);

	switch (offset) {
	case GICV5_IRS_CR0:
		scoped_guard(mutex, &vcpu->kvm->arch.config_lock) {
			bool was_enabled = vgic->enabled;

			WRITE_ONCE(vgic->enabled, !!(val & GICV5_IRS_CR0_IRSEN));

			/* Reload other vCPUs on an edge */
			if (was_enabled != !!(val & GICV5_IRS_CR0_IRSEN))
				kvm_make_all_cpus_request(vcpu->kvm,
							  KVM_REQ_RELOAD_GICv5);
		}
		return;
	case GICV5_IRS_CR1:
		irs->cr1.sh = FIELD_GET(GICV5_IRS_CR1_SH, val);
		irs->cr1.oc = FIELD_GET(GICV5_IRS_CR1_OC, val);
		irs->cr1.ic = FIELD_GET(GICV5_IRS_CR1_IC, val);
		irs->cr1.ist_ra = !!(val & GICV5_IRS_CR1_IST_RA);
		irs->cr1.ist_wa = !!(val & GICV5_IRS_CR1_IST_WA);
		irs->cr1.vmt_ra = !!(val & GICV5_IRS_CR1_VMT_RA);
		irs->cr1.vpet_ra = !!(val & GICV5_IRS_CR1_VPET_RA);
		irs->cr1.vmd_ra = !!(val & GICV5_IRS_CR1_VMD_RA);
		irs->cr1.vmd_wa = !!(val & GICV5_IRS_CR1_VMD_WA);
		irs->cr1.vped_ra = !!(val & GICV5_IRS_CR1_VPED_RA);
		irs->cr1.vped_wa = !!(val & GICV5_IRS_CR1_VPED_WA);
		return;
	case GICV5_IRS_PE_SELR:
		WRITE_ONCE(irs->pe_selr.iaffid,
			   FIELD_GET(GICV5_IRS_PE_SELR_IAFFID, val));
		return;
	case GICV5_IRS_PE_CR0:
		/*
		 * We actually have nothing to do here as we don't support
		 * 1-of-N routing. The only thing that the guest can correctly
		 * write here is 0x1. However, there's no way to fault if it
		 * writes something else. This is effectively a WI in our case,
		 * but we keep it here for the purposes of documenting it.
		 */
		return;
	default:
		return;
	}
}

static bool vgic_v5_is_spi_selr_valid(struct vgic_v5_irs *irs, u32 id)
{
	/* Invalid - we don't have any SPIs at all */
	if (irs->idr5.spi_range == 0)
		return false;

	/* Invalid - we don't have any on this IRS */
	if (irs->idr6.spi_irs_range == 0)
		return false;

	/* Invalid - ID is less than min */
	if (id < irs->idr7.spi_base)
		return false;

	/* Invalid - ID is greater than max */
	if (id >= (irs->idr7.spi_base + irs->idr6.spi_irs_range))
		return false;

	return true;
}

static unsigned long vgic_v5_mmio_read_irs_spi(struct kvm_vcpu *vcpu,
					       gpa_t addr, unsigned int len)
{
	struct vgic_v5_irs *irs = vgic_v5_get_irs(vcpu);
	const size_t offset = addr & (SZ_64K - 1);
	struct vgic_irq *irq;
	u64 value = 0;

	switch (offset) {
	case GICV5_IRS_SPI_SELR:
		/* Return whatever was last written */
		value = FIELD_PREP(GICV5_IRS_SPI_SELR_ID, READ_ONCE(irs->spi_selr.id));
		break;
	case GICV5_IRS_SPI_STATUSR:
		/* We assume that we can always claim to be idle */
		value = GICV5_IRS_SPI_STATUSR_IDLE;
		if (vgic_v5_is_spi_selr_valid(irs, READ_ONCE(irs->spi_selr.id)))
			value |= GICV5_IRS_SPI_STATUSR_V;
		break;
	case GICV5_IRS_SPI_DOMAINR:
		value = FIELD_PREP(GICV5_IRS_SPI_DOMAINR_DOMAIN,
				   GICV5_IRS_SPI_DOMAINR_DOMAIN_NON_SECURE);
		break;
	case GICV5_IRS_SPI_CFGR: {
		u32 id = READ_ONCE(irs->spi_selr.id);

		if (!vgic_v5_is_spi_selr_valid(irs, id)) {
			/* Fault with IRS_SPI_SELR; return 0*/
			value = 0;
			break;
		}

		irq = vgic_get_irq(vcpu->kvm, vgic_v5_make_spi(id));
		if (!irq) {
			kvm_debug_ratelimited("Guest trying to access SPI not backed by KVM\n");
			value = 0;
			break;
		}

		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock) {
			if (irq->config == VGIC_CONFIG_LEVEL)
				value = GICV5_IRS_SPI_CFGR_TM;
		}

		vgic_put_irq(vcpu->kvm, irq);

		break;
	}
	default:
		return 0;
	}

	return value;
}

static void vgic_v5_mmio_write_irs_spi(struct kvm_vcpu *vcpu, gpa_t addr,
				       unsigned int len, unsigned long val)
{
	struct vgic_v5_irs *irs = vgic_v5_get_irs(vcpu);
	const size_t offset = addr & (SZ_64K - 1);
	struct vgic_irq *irq;

	switch (offset) {
	case GICV5_IRS_SPI_SELR:
		WRITE_ONCE(irs->spi_selr.id,
			   FIELD_GET(GICV5_IRS_SPI_SELR_ID, val));
		return;
	case GICV5_IRS_SPI_CFGR: {
		u32 id = READ_ONCE(irs->spi_selr.id);

		if (!vgic_v5_is_spi_selr_valid(irs, id))
			return;

		/*
		 * Find KVM's representation of the interrupt - we need to make
		 * sure that KVM's view agrees with the guest's, else interrupt
		 * injection won't work properly for level-triggered interrupts
		 * (we fail to handle the clearing of the pending state if KVM
		 * thinks that the interrupt is edge-triggered, which is the
		 * default.)
		 */
		irq = vgic_get_irq(vcpu->kvm, vgic_v5_make_spi(id));
		if (!irq)
			return;

		scoped_guard(raw_spinlock_irqsave, &irq->irq_lock) {
			if (val & GICV5_IRS_SPI_CFGR_TM)
				irq->config = VGIC_CONFIG_LEVEL;
			else
				irq->config = VGIC_CONFIG_EDGE;
		}

		vgic_put_irq(vcpu->kvm, irq);

		return;
	}
	default:
		return;
	}
}

static bool vgic_v5_ist_cfgr_valid(struct vgic_v5_irs *irs)
{
	unsigned int expected_istsz;

	if (irs->ist_cfgr.lpi_id_bits < irs->idr2.min_lpi_id_bits ||
	    irs->ist_cfgr.lpi_id_bits > irs->idr2.id_bits)
		return false;

	if (!irs->idr2.istmd)
		expected_istsz = GICV5_IRS_IST_CFGR_ISTSZ_4;
	else if (irs->ist_cfgr.lpi_id_bits >= irs->idr2.istmd_sz)
		expected_istsz = GICV5_IRS_IST_CFGR_ISTSZ_16;
	else
		expected_istsz = GICV5_IRS_IST_CFGR_ISTSZ_8;

	if (irs->ist_cfgr.istsz != expected_istsz)
		return false;

	if (irs->ist_cfgr.structure && !irs->idr2.ist_levels)
		return false;

	if (!irs->ist_cfgr.structure)
		return true;

	return irs->ist_cfgr.l2sz == irs->idr2.ist_l2sz;
}

static u64 vgic_v5_read_irs_ist_baser(struct vgic_v5_irs *irs)
{
	u64 value;

	value = FIELD_PREP(GICV5_IRS_IST_BASER_ADDR_MASK,
			   irs->ist_baser.addr >> GICV5_IRS_IST_BASER_ADDR_SHIFT);
	if (irs->ist_baser.valid)
		value |= GICV5_IRS_IST_BASER_VALID;

	return value;
}

static unsigned long vgic_v5_mmio_read_irs_ist(struct kvm_vcpu *vcpu,
					       gpa_t addr, unsigned int len)
{
	struct vgic_v5_irs *irs = vgic_v5_get_irs(vcpu);
	const size_t offset = addr & (SZ_64K - 1);
	u64 value = 0;

	switch (offset) {
	case GICV5_IRS_IST_STATUSR:
		return GICV5_IRS_IST_STATUSR_IDLE;
	case GICV5_IRS_IST_CFGR:
		if (irs->ist_cfgr.structure)
			value |= GICV5_IRS_IST_CFGR_STRUCTURE;
		value |= FIELD_PREP(GICV5_IRS_IST_CFGR_ISTSZ, irs->ist_cfgr.istsz);
		value |= FIELD_PREP(GICV5_IRS_IST_CFGR_L2SZ, irs->ist_cfgr.l2sz);
		value |= FIELD_PREP(GICV5_IRS_IST_CFGR_LPI_ID_BITS, irs->ist_cfgr.lpi_id_bits);
		break;
	case GICV5_IRS_IST_BASER:
	case GICV5_IRS_IST_BASER + 4:
		value = vgic_v5_read_irs_ist_baser(irs);
		return extract_bytes(value, offset & 7, len);
	default:
		return 0;
	}

	return value;
}

static void vgic_v5_mmio_write_irs_ist(struct kvm_vcpu *vcpu, gpa_t addr,
				       unsigned int len, unsigned long val)
{
	struct vgic_v5_irs *irs = vgic_v5_get_irs(vcpu);
	const size_t offset = addr & (SZ_64K - 1);
	enum gicv5_vcpu_cmd cmd = LPI_VIST_MAKE_INVALID;

	switch (offset) {
	case GICV5_IRS_IST_CFGR:
		scoped_guard(mutex, &vcpu->kvm->arch.config_lock) {
			if (irs->ist_baser.valid)
				return;

			irs->ist_cfgr.lpi_id_bits = FIELD_GET(GICV5_IRS_IST_CFGR_LPI_ID_BITS, val);
			irs->ist_cfgr.l2sz = FIELD_GET(GICV5_IRS_IST_CFGR_L2SZ, val);
			irs->ist_cfgr.istsz = FIELD_GET(GICV5_IRS_IST_CFGR_ISTSZ, val);
			irs->ist_cfgr.structure = !!(val & GICV5_IRS_IST_CFGR_STRUCTURE);
		}
		return;
	case GICV5_IRS_IST_BASER:
	case GICV5_IRS_IST_BASER + 4: {
		u64 baser;
		bool valid;

		guard(mutex)(&vcpu->kvm->arch.config_lock);

		baser = update_64bit_reg(vgic_v5_read_irs_ist_baser(irs),
					 offset & 4, len, val);
		valid = !!(baser & GICV5_IRS_IST_BASER_VALID);

		/* The address cannot be changed while the IST is valid. */
		if (irs->ist_baser.valid && valid)
			return;

		/* Valid -> Invalid */
		if (irs->ist_baser.valid && !valid) {
			/* Make the LPI IST invalid and then ... */
			if (irq_set_vcpu_affinity(vgic_v5_vpe_db(vcpu), &cmd))
				break;

			/*
			 * ... free the host IST if we successfully marked the
			 * IST as invalid. Frankly, if we failed to make the
			 * guest's IST as invalid, we're cooked because it means
			 * that the IRS may still be using the memory that we
			 * want to free. Hence, we leave it allocated and skip
			 * the clearing of valid bit in the baser.
			 */
			vgic_v5_lpi_ist_free(vcpu->kvm);
		} else if (!irs->ist_baser.valid && valid) { /* Invalid -> Valid */
			if (!vgic_v5_ist_cfgr_valid(irs))
				break;

			if (vgic_v5_lpi_ist_alloc(vcpu->kvm, irs->ist_cfgr.lpi_id_bits))
				break;
		}

		/* Now that we've handled the edges, update the valid bit and addr */
		irs->ist_baser.valid = valid;
		irs->ist_baser.addr = FIELD_GET(GICV5_IRS_IST_BASER_ADDR_MASK, baser)
			<< GICV5_IRS_IST_BASER_ADDR_SHIFT;

		return;
	}
	default:
		return;
	}
}

static const struct vgic_register_region vgic_v5_irs_registers[] = {
	/*
	 * This is the IRS_CONFIG_FRAME.
	 */
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR0, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR1, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR2, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR3, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR4, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR5, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR6, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IDR7, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IIDR, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_AIDR, vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_CR0, vgic_v5_mmio_read_irs_misc,
				  vgic_v5_mmio_write_irs_misc, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_CR1, vgic_v5_mmio_read_irs_misc,
				  vgic_v5_mmio_write_irs_misc, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SYNCR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SYNC_STATUSR,
				  vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SPI_VMR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SPI_SELR, vgic_v5_mmio_read_irs_spi,
				  vgic_v5_mmio_write_irs_spi, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SPI_DOMAINR, vgic_v5_mmio_read_irs_spi,
				  vgic_v5_mmio_write_irs_spi, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SPI_RESAMPLER, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SPI_CFGR, vgic_v5_mmio_read_irs_spi,
				  vgic_v5_mmio_write_irs_spi, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SPI_STATUSR,
				  vgic_v5_mmio_read_irs_spi, vgic_mmio_write_wi,
				  4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_PE_SELR, vgic_v5_mmio_read_irs_misc,
				  vgic_v5_mmio_write_irs_misc, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_PE_STATUSR,
				  vgic_v5_mmio_read_irs_misc,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_PE_CR0, vgic_v5_mmio_read_irs_misc,
				  vgic_v5_mmio_write_irs_misc, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IST_BASER, vgic_v5_mmio_read_irs_ist,
				  vgic_v5_mmio_write_irs_ist, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IST_CFGR, vgic_v5_mmio_read_irs_ist,
				  vgic_v5_mmio_write_irs_ist, 4,
				  VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_IST_STATUSR,
				  vgic_v5_mmio_read_irs_ist, vgic_mmio_write_wi,
				  4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_MAP_L2_ISTR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),

	/*
	 * The following registers are only for running VMs. They are not yet
	 * supported as we don't currently support nested, so expose them as
	 * read-as-zero/write-ignored.
	 */
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMT_BASER, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMT_CFGR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMT_STATUSR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VPE_SELR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VPE_DBR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VPE_HPPIR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VPE_CR0, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VPE_STATUSR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VM_DBR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VM_SELR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VM_STATUSR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMAP_L2_VMTR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMAP_VMR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMAP_VISTR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMAP_L2_VISTR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_VMAP_VPER, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SAVE_VMR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SAVE_VM_STATUSR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),

	/* MEC, MPAM, SWERR - all unimplemented */

	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_MEC_IDR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_MEC_MECID_R, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_MPAM_IDR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_MPAM_PARTID_R, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 4, VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SWERR_STATUSR, vgic_mmio_read_raz,
				  vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SWERR_SYNDROMER0,
				  vgic_mmio_read_raz, vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
	REGISTER_DESC_WITH_LENGTH(GICV5_IRS_SWERR_SYNDROMER1,
				  vgic_mmio_read_raz, vgic_mmio_write_wi, 8,
				  VGIC_ACCESS_64bit | VGIC_ACCESS_32bit),
};

unsigned int vgic_v5_init_irs_iodev(struct vgic_io_device *dev)
{
	dev->regions = vgic_v5_irs_registers;
	dev->nr_regions = ARRAY_SIZE(vgic_v5_irs_registers);

	kvm_iodevice_init(&dev->dev, &kvm_io_gic_ops);

	/* We represent both of the IRS frames back to back, so this is 128K */
	return KVM_VGIC_V5_IRS_SIZE;
}

int vgic_v5_register_irs_iodev(struct kvm *kvm, gpa_t irs_base_address)
{
	struct vgic_io_device *io_device = &kvm->arch.vgic.vgic_v5_irs_data->iodev;
	unsigned int len;

	/*
	 * Design choice: Force MMIO region to be 64k aligned. Simplifies
	 * pulling out registers.
	 */
	if (!IS_ALIGNED(irs_base_address, SZ_64K))
		return -EINVAL;

	len = vgic_v5_init_irs_iodev(io_device);

	io_device->base_addr = irs_base_address;
	io_device->iodev_type = IODEV_GICV5_IRS;
	io_device->redist_vcpu = NULL;

	return kvm_io_bus_register_dev(kvm, KVM_MMIO_BUS, irs_base_address, len,
				       &io_device->dev);
}

/**
 * kvm_vgic_v5_irs_init: initialize the IRS data structures
 * @kvm: kvm struct pointer
 * @nr_spis: number of spis, frozen by caller
 */
int kvm_vgic_v5_irs_init(struct kvm *kvm, unsigned int nr_spis)
{
	struct vgic_dist *dist = &kvm->arch.vgic;
	struct vgic_v5_irs *irs = dist->vgic_v5_irs_data;
	struct kvm_vcpu *vcpu0 = kvm_get_vcpu(kvm, 0);
	size_t nr_spi_bits;
	u64 mmfr0;
	int ret, i;

	/*
	 * We (KVM) allocate an Interrupt State Table (IST) for SPIs. The
	 * hardware mandates that lower 6 bits of the address are 0. Each ISTE
	 * is 4 bytes in size (or larger if metadata storage is required), so 16
	 * entries would be enough for alignment. We require userspace to
	 * provide at least 32 SPIs, so we always meet alignment requirements.
	 */
	if (nr_spis) {
		dist->spis = kcalloc(nr_spis, sizeof(struct vgic_irq),
				     GFP_KERNEL_ACCOUNT);
		if (!dist->spis)
			return -ENOMEM;

		/*
		 * In the following code we do not take the irq struct lock since
		 * no other action on irq structs can happen while the VGIC is
		 * not initialized yet.
		 */
		for (i = 0; i < nr_spis; i++) {
			struct vgic_irq *irq = &dist->spis[i];

			irq->intid = vgic_v5_make_spi(i);
			INIT_LIST_HEAD(&irq->ap_list);
			raw_spin_lock_init(&irq->irq_lock);
			irq->vcpu = NULL;
			irq->target_vcpu = vcpu0;
			refcount_set(&irq->refcount, 0);
			/*
			 * The guest controls the enable state, and again it is
			 * directly handled by the hardware. From our point of
			 * view it is always enabled.
			 */
			irq->enabled = 1;
			vgic_v5_set_spi_ops(irq);
		}

		nr_spi_bits = fls(roundup_pow_of_two(nr_spis)) - 1;

		ret = vgic_v5_spi_ist_alloc(kvm, nr_spi_bits);
		if (ret) {
			kfree(dist->spis);
			dist->spis = NULL;
			return ret;
		}
	}

	/* Set sane initial state for the IRS MMIO registers */

	irs->idr0.domain = GICV5_IRS_IDR0_INT_DOM_NON_SECURE;

	mmfr0 = read_sanitised_ftr_reg(SYS_ID_AA64MMFR0_EL1);
	irs->idr0.pa_range = cpuid_feature_extract_unsigned_field(mmfr0,
								  ID_AA64MMFR0_EL1_PARANGE_SHIFT);

	irs->idr0.virt = 0;
	irs->idr0.setlpi = 0;
	irs->idr0.mec = 0;
	irs->idr0.mpam = 0;
	irs->idr0.swe = 0;
	irs->idr0.irs_id = 0;

	irs->idr1.priority_bits = gicv5_global_data.irs_pri_bits - 1;

	/*
	 * Support 16-bits of ID space for the IRS. This should be sufficient
	 * for most applications, and the CPUIF is guaranteed to have at least
	 * 16-bits of ID space support (we actually present 16-bits there, even
	 * if the hardware supports more). Warn if the hardware doesn't support
	 * 16 bits, and use the smaller value. YMMV!
	 *
	 * As for the minimum number of ID bits, we match the hardware's
	 * capability.
	 */
	if (vgic_v5_irs_ist_id_bits(&irs_caps) < 16)
		pr_warn_ratelimited("Host IRS supports fewer than 16 ID bits for ISTs (%u)\n",
				    vgic_v5_irs_ist_id_bits(&irs_caps));

	irs->idr2.id_bits = min(16, vgic_v5_irs_ist_id_bits(&irs_caps));
	irs->idr2.min_lpi_id_bits = vgic_v5_irs_min_lpi_id_bits(&irs_caps);

	/* Only allow the guest to create Linear ISTs - simplifies Save/Restore */
	irs->idr2.ist_levels = 0;
	irs->idr2.ist_l2sz = GICV5_IRS_IST_CFGR_L2SZ_4K;
	irs->idr2.istmd = 0;
	irs->idr2.istmd_sz = 0;

	/* We have a single IRS, only. All SPIs reside here! */
	irs->idr5.spi_range = nr_spis;
	irs->idr6.spi_irs_range = nr_spis;
	irs->idr7.spi_base = 0;

	irs->cr1.sh = 0;
	irs->cr1.oc = 0;
	irs->cr1.ic = 0;
	irs->cr1.ist_ra = 0;
	irs->cr1.ist_wa = 0;
	irs->cr1.vmt_ra = 0;
	irs->cr1.vpet_ra = 0;
	irs->cr1.vmd_ra = 0;
	irs->cr1.vmd_wa = 0;
	irs->cr1.vped_ra = 0;
	irs->cr1.vped_wa = 0;

	irs->spi_selr.id = -1;

	irs->pe_selr.iaffid = -1;

	irs->ist_cfgr.lpi_id_bits = 0;
	irs->ist_cfgr.l2sz = 0;
	irs->ist_cfgr.istsz = 0;
	irs->ist_cfgr.structure = 0;

	irs->ist_baser.valid = 0;
	irs->ist_baser.addr = 0;

	return 0;
}
