// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2017 - Linaro Ltd
 * Author: Jintack Lim <jintack.lim@linaro.org>
 */

#include <linux/kvm_host.h>

#include <asm/esr.h>
#include <asm/kvm_hyp.h>
#include <asm/kvm_mmu.h>

enum trans_regime {
	TR_EL10,
	TR_EL20,
	TR_EL2,
};

struct s1_walk_info {
	u64	     		baddr;
	enum trans_regime	regime;
	unsigned int		max_oa_bits;
	unsigned int		pgshift;
	unsigned int		txsz;
	int 	     		sl;
	bool	     		hpd;
	bool	     		be;
	bool	     		s2;
};

struct s1_walk_result {
	union {
		struct {
			u64	desc;
			u64	pa;
			s8	level;
			u8	APTable;
			bool	UXNTable;
			bool	PXNTable;
		};
		struct {
			u8	fst;
			bool	ptw;
			bool	s2;
		};
	};
	bool	failed;
};

static void fail_s1_walk(struct s1_walk_result *wr, u8 fst, bool ptw, bool s2)
{
	wr->fst		= fst;
	wr->ptw		= ptw;
	wr->s2		= s2;
	wr->failed	= true;
}

#define S1_MMU_DISABLED		(-127)

static int get_ia_size(struct s1_walk_info *wi)
{
	return 64 - wi->txsz;
}

/* Return true if the IPA is out of the OA range */
static bool check_output_size(u64 ipa, struct s1_walk_info *wi)
{
	return wi->max_oa_bits < 48 && (ipa & GENMASK_ULL(47, wi->max_oa_bits));
}

/* Return the translation regime that applies to an AT instruction */
static enum trans_regime compute_translation_regime(struct kvm_vcpu *vcpu, u32 op)
{
	/*
	 * We only get here from guest EL2, so the translation
	 * regime AT applies to is solely defined by {E2H,TGE}.
	 */
	switch (op) {
	case OP_AT_S1E2R:
	case OP_AT_S1E2W:
	case OP_AT_S1E2A:
		return vcpu_el2_e2h_is_set(vcpu) ? TR_EL20 : TR_EL2;
		break;
	default:
		return (vcpu_el2_e2h_is_set(vcpu) &&
			vcpu_el2_tge_is_set(vcpu)) ? TR_EL20 : TR_EL10;
	}
}

static int setup_s1_walk(struct kvm_vcpu *vcpu, u32 op, struct s1_walk_info *wi,
			 struct s1_walk_result *wr, u64 va)
{
	u64 hcr, sctlr, tcr, tg, ps, ia_bits, ttbr;
	unsigned int stride, x;
	bool va55, tbi, lva, as_el0;

	hcr = __vcpu_sys_reg(vcpu, HCR_EL2);

	wi->regime = compute_translation_regime(vcpu, op);
	as_el0 = (op == OP_AT_S1E0R || op == OP_AT_S1E0W);

	va55 = va & BIT(55);

	if (wi->regime == TR_EL2 && va55)
		goto addrsz;

	wi->s2 = wi->regime == TR_EL10 && (hcr & (HCR_VM | HCR_DC));

	switch (wi->regime) {
	case TR_EL10:
		sctlr	= vcpu_read_sys_reg(vcpu, SCTLR_EL1);
		tcr	= vcpu_read_sys_reg(vcpu, TCR_EL1);
		ttbr	= (va55 ?
			   vcpu_read_sys_reg(vcpu, TTBR1_EL1) :
			   vcpu_read_sys_reg(vcpu, TTBR0_EL1));
		break;
	case TR_EL2:
	case TR_EL20:
		sctlr	= vcpu_read_sys_reg(vcpu, SCTLR_EL2);
		tcr	= vcpu_read_sys_reg(vcpu, TCR_EL2);
		ttbr	= (va55 ?
			   vcpu_read_sys_reg(vcpu, TTBR1_EL2) :
			   vcpu_read_sys_reg(vcpu, TTBR0_EL2));
		break;
	default:
		BUG();
	}

	tbi = (wi->regime == TR_EL2 ?
	       FIELD_GET(TCR_EL2_TBI, tcr) :
	       (va55 ?
		FIELD_GET(TCR_TBI1, tcr) :
		FIELD_GET(TCR_TBI0, tcr)));

	if (!tbi && (u64)sign_extend64(va, 55) != va)
		goto addrsz;

	va = (u64)sign_extend64(va, 55);

	/* Let's put the MMU disabled case aside immediately */
	switch (wi->regime) {
	case TR_EL10:
		/*
		 * If dealing with the EL1&0 translation regime, 3 things
		 * can disable the S1 translation:
		 *
		 * - HCR_EL2.DC = 1
		 * - HCR_EL2.{E2H,TGE} = {0,1}
		 * - SCTLR_EL1.M = 0
		 *
		 * The TGE part is interesting. If we have decided that this
		 * is EL1&0, then it means that either {E2H,TGE} == {1,0} or
		 * {0,x}, and we only need to test for TGE == 1.
		 */
		if (hcr & (HCR_DC | HCR_TGE)) {
			wr->level = S1_MMU_DISABLED;
			break;
		}
		fallthrough;
	case TR_EL2:
	case TR_EL20:
		if (!(sctlr & SCTLR_ELx_M))
			wr->level = S1_MMU_DISABLED;
		break;
	}

	if (wr->level == S1_MMU_DISABLED) {
		if (va >= BIT(kvm_get_pa_bits(vcpu->kvm)))
			goto addrsz;

		wr->pa = va;
		return 0;
	}

	wi->be = sctlr & SCTLR_ELx_EE;

	wi->hpd  = kvm_has_feat(vcpu->kvm, ID_AA64MMFR1_EL1, HPDS, IMP);
	wi->hpd &= (wi->regime == TR_EL2 ?
		    FIELD_GET(TCR_EL2_HPD, tcr) :
		    (va55 ?
		     FIELD_GET(TCR_HPD1, tcr) :
		     FIELD_GET(TCR_HPD0, tcr)));

	/* Someone was silly enough to encode TG0/TG1 differently */
	if (va55) {
		wi->txsz = FIELD_GET(TCR_T1SZ_MASK, tcr);
		tg = FIELD_GET(TCR_TG1_MASK, tcr);

		switch (tg << TCR_TG1_SHIFT) {
		case TCR_TG1_4K:
			wi->pgshift = 12;	 break;
		case TCR_TG1_16K:
			wi->pgshift = 14;	 break;
		case TCR_TG1_64K:
		default:	    /* IMPDEF: treat any other value as 64k */
			wi->pgshift = 16;	 break;
		}
	} else {
		wi->txsz = FIELD_GET(TCR_T0SZ_MASK, tcr);
		tg = FIELD_GET(TCR_TG0_MASK, tcr);

		switch (tg << TCR_TG0_SHIFT) {
		case TCR_TG0_4K:
			wi->pgshift = 12;	 break;
		case TCR_TG0_16K:
			wi->pgshift = 14;	 break;
		case TCR_TG0_64K:
		default:	    /* IMPDEF: treat any other value as 64k */
			wi->pgshift = 16;	 break;
		}
	}

	/* R_PLCGL, R_YXNYW */
	if (!kvm_has_feat_enum(vcpu->kvm, ID_AA64MMFR2_EL1, ST, 48_47)) {
		if (wi->txsz > 39)
			goto transfault_l0;
	} else {
		if (wi->txsz > 48 || (BIT(wi->pgshift) == SZ_64K && wi->txsz > 47))
			goto transfault_l0;
	}

	/* R_GTJBY, R_SXWGM */
	switch (BIT(wi->pgshift)) {
	case SZ_4K:
		lva = kvm_has_feat(vcpu->kvm, ID_AA64MMFR0_EL1, TGRAN4, 52_BIT);
		lva &= tcr & (wi->regime == TR_EL2 ? TCR_EL2_DS : TCR_DS);
		break;
	case SZ_16K:
		lva = kvm_has_feat(vcpu->kvm, ID_AA64MMFR0_EL1, TGRAN16, 52_BIT);
		lva &= tcr & (wi->regime == TR_EL2 ? TCR_EL2_DS : TCR_DS);
		break;
	case SZ_64K:
		lva = kvm_has_feat(vcpu->kvm, ID_AA64MMFR2_EL1, VARange, 52);
		break;
	}

	if ((lva && wi->txsz < 12) || (!lva && wi->txsz < 16))
		goto transfault_l0;

	ia_bits = get_ia_size(wi);

	/* R_YYVYV, I_THCZK */
	if ((!va55 && va > GENMASK(ia_bits - 1, 0)) ||
	    (va55 && va < GENMASK(63, ia_bits)))
		goto transfault_l0;

	/* I_ZFSYQ */
	if (wi->regime != TR_EL2 &&
	    (tcr & (va55 ? TCR_EPD1_MASK : TCR_EPD0_MASK)))
		goto transfault_l0;

	/* R_BNDVG and following statements */
	if (kvm_has_feat(vcpu->kvm, ID_AA64MMFR2_EL1, E0PD, IMP) &&
	    as_el0 && (tcr & (va55 ? TCR_E0PD1 : TCR_E0PD0)))
		goto transfault_l0;

	/* AArch64.S1StartLevel() */
	stride = wi->pgshift - 3;
	wi->sl = 3 - (((ia_bits - 1) - wi->pgshift) / stride);

	ps = (wi->regime == TR_EL2 ?
	      FIELD_GET(TCR_EL2_PS_MASK, tcr) : FIELD_GET(TCR_IPS_MASK, tcr));

	wi->max_oa_bits = min(get_kvm_ipa_limit(), ps_to_output_size(ps));

	/* Compute minimal alignment */
	x = 3 + ia_bits - ((3 - wi->sl) * stride + wi->pgshift);

	wi->baddr = ttbr & TTBRx_EL1_BADDR;

	/* R_VPBBF */
	if (check_output_size(wi->baddr, wi))
		goto addrsz;

	wi->baddr &= GENMASK_ULL(wi->max_oa_bits - 1, x);

	return 0;

addrsz:				/* Address Size Fault level 0 */
	fail_s1_walk(wr, ESR_ELx_FSC_ADDRSZ_L(0), false, false);
	return -EFAULT;

transfault_l0:			/* Translation Fault level 0 */
	fail_s1_walk(wr, ESR_ELx_FSC_FAULT_L(0), false, false);
	return -EFAULT;
}

static int walk_s1(struct kvm_vcpu *vcpu, struct s1_walk_info *wi,
		   struct s1_walk_result *wr, u64 va)
{
	u64 va_top, va_bottom, baddr, desc;
	int level, stride, ret;

	level = wi->sl;
	stride = wi->pgshift - 3;
	baddr = wi->baddr;

	va_top = get_ia_size(wi) - 1;

	while (1) {
		u64 index, ipa;

		va_bottom = (3 - level) * stride + wi->pgshift;
		index = (va & GENMASK_ULL(va_top, va_bottom)) >> (va_bottom - 3);

		ipa = baddr | index;

		if (wi->s2) {
			struct kvm_s2_trans s2_trans = {};

			ret = kvm_walk_nested_s2(vcpu, ipa, &s2_trans);
			if (ret) {
				fail_s1_walk(wr,
					     (s2_trans.esr & ~ESR_ELx_FSC_LEVEL) | level,
					     true, true);
				return ret;
			}

			if (!kvm_s2_trans_readable(&s2_trans)) {
				fail_s1_walk(wr, ESR_ELx_FSC_PERM_L(level),
					     true, true);

				return -EPERM;
			}

			ipa = kvm_s2_trans_output(&s2_trans);
		}

		ret = kvm_read_guest(vcpu->kvm, ipa, &desc, sizeof(desc));
		if (ret) {
			fail_s1_walk(wr, ESR_ELx_FSC_SEA_TTW(level),
				     true, false);
			return ret;
		}

		if (wi->be)
			desc = be64_to_cpu((__force __be64)desc);
		else
			desc = le64_to_cpu((__force __le64)desc);

		/* Invalid descriptor */
		if (!(desc & BIT(0)))
			goto transfault;

		/* Block mapping, check validity down the line */
		if (!(desc & BIT(1)))
			break;

		/* Page mapping */
		if (level == 3)
			break;

		/* Table handling */
		if (!wi->hpd) {
			wr->APTable  |= FIELD_GET(S1_TABLE_AP, desc);
			wr->UXNTable |= FIELD_GET(PMD_TABLE_UXN, desc);
			wr->PXNTable |= FIELD_GET(PMD_TABLE_PXN, desc);
		}

		baddr = desc & GENMASK_ULL(47, wi->pgshift);

		/* Check for out-of-range OA */
		if (check_output_size(baddr, wi))
			goto addrsz;

		/* Prepare for next round */
		va_top = va_bottom - 1;
		level++;
	}

	/* Block mapping, check the validity of the level */
	if (!(desc & BIT(1))) {
		bool valid_block = false;

		switch (BIT(wi->pgshift)) {
		case SZ_4K:
			valid_block = level == 1 || level == 2;
			break;
		case SZ_16K:
		case SZ_64K:
			valid_block = level == 2;
			break;
		}

		if (!valid_block)
			goto transfault;
	}

	if (check_output_size(desc & GENMASK(47, va_bottom), wi))
		goto addrsz;

	va_bottom += contiguous_bit_shift(desc, wi, level);

	wr->failed = false;
	wr->level = level;
	wr->desc = desc;
	wr->pa = desc & GENMASK(47, va_bottom);
	wr->pa |= va & GENMASK_ULL(va_bottom - 1, 0);

	return 0;

addrsz:
	fail_s1_walk(wr, ESR_ELx_FSC_ADDRSZ_L(level), true, false);
	return -EINVAL;
transfault:
	fail_s1_walk(wr, ESR_ELx_FSC_FAULT_L(level), true, false);
	return -ENOENT;
}

struct mmu_config {
	u64	ttbr0;
	u64	ttbr1;
	u64	tcr;
	u64	mair;
	u64	sctlr;
	u64	vttbr;
	u64	vtcr;
	u64	hcr;
};

static void __mmu_config_save(struct mmu_config *config)
{
	config->ttbr0	= read_sysreg_el1(SYS_TTBR0);
	config->ttbr1	= read_sysreg_el1(SYS_TTBR1);
	config->tcr	= read_sysreg_el1(SYS_TCR);
	config->mair	= read_sysreg_el1(SYS_MAIR);
	config->sctlr	= read_sysreg_el1(SYS_SCTLR);
	config->vttbr	= read_sysreg(vttbr_el2);
	config->vtcr	= read_sysreg(vtcr_el2);
	config->hcr	= read_sysreg(hcr_el2);
}

static void __mmu_config_restore(struct mmu_config *config)
{
	write_sysreg(config->hcr,	hcr_el2);

	/*
	 * ARM errata 1165522 and 1530923 require TGE to be 1 before
	 * we update the guest state.
	 */
	asm(ALTERNATIVE("nop", "isb", ARM64_WORKAROUND_SPECULATIVE_AT));

	write_sysreg_el1(config->ttbr0,	SYS_TTBR0);
	write_sysreg_el1(config->ttbr1,	SYS_TTBR1);
	write_sysreg_el1(config->tcr,	SYS_TCR);
	write_sysreg_el1(config->mair,	SYS_MAIR);
	write_sysreg_el1(config->sctlr,	SYS_SCTLR);
	write_sysreg(config->vttbr,	vttbr_el2);
	write_sysreg(config->vtcr,	vtcr_el2);
}

static bool at_s1e1p_fast(struct kvm_vcpu *vcpu, u32 op, u64 vaddr)
{
	u64 host_pan;
	bool fail;

	host_pan = read_sysreg_s(SYS_PSTATE_PAN);
	write_sysreg_s(*vcpu_cpsr(vcpu) & PSTATE_PAN, SYS_PSTATE_PAN);

	switch (op) {
	case OP_AT_S1E1RP:
		fail = __kvm_at(OP_AT_S1E1RP, vaddr);
		break;
	case OP_AT_S1E1WP:
		fail = __kvm_at(OP_AT_S1E1WP, vaddr);
		break;
	}

	write_sysreg_s(host_pan, SYS_PSTATE_PAN);

	return fail;
}

#define MEMATTR(ic, oc)		(MEMATTR_##oc << 4 | MEMATTR_##ic)
#define MEMATTR_NC		0b0100
#define MEMATTR_Wt		0b1000
#define MEMATTR_Wb		0b1100
#define MEMATTR_WbRaWa		0b1111

#define MEMATTR_IS_DEVICE(m)	(((m) & GENMASK(7, 4)) == 0)

static u8 s2_memattr_to_attr(u8 memattr)
{
	memattr &= 0b1111;

	switch (memattr) {
	case 0b0000:
	case 0b0001:
	case 0b0010:
	case 0b0011:
		return memattr << 2;
	case 0b0100:
		return MEMATTR(Wb, Wb);
	case 0b0101:
		return MEMATTR(NC, NC);
	case 0b0110:
		return MEMATTR(Wt, NC);
	case 0b0111:
		return MEMATTR(Wb, NC);
	case 0b1000:
		/* Reserved, assume NC */
		return MEMATTR(NC, NC);
	case 0b1001:
		return MEMATTR(NC, Wt);
	case 0b1010:
		return MEMATTR(Wt, Wt);
	case 0b1011:
		return MEMATTR(Wb, Wt);
	case 0b1100:
		/* Reserved, assume NC */
		return MEMATTR(NC, NC);
	case 0b1101:
		return MEMATTR(NC, Wb);
	case 0b1110:
		return MEMATTR(Wt, Wb);
	case 0b1111:
		return MEMATTR(Wb, Wb);
	default:
		unreachable();
	}
}

static u8 combine_s1_s2_attr(u8 s1, u8 s2)
{
	bool transient;
	u8 final = 0;

	/* Upgrade transient s1 to non-transient to simplify things */
	switch (s1) {
	case 0b0001 ... 0b0011:	/* Normal, Write-Through Transient */
		transient = true;
		s1 = MEMATTR_Wt | (s1 & GENMASK(1,0));
		break;
	case 0b0101 ... 0b0111:	/* Normal, Write-Back Transient */
		transient = true;
		s1 = MEMATTR_Wb | (s1 & GENMASK(1,0));
		break;
	default:
		transient = false;
	}

	/* S2CombineS1AttrHints() */
	if ((s1 & GENMASK(3, 2)) == MEMATTR_NC ||
	    (s2 & GENMASK(3, 2)) == MEMATTR_NC)
		final = MEMATTR_NC;
	else if ((s1 & GENMASK(3, 2)) == MEMATTR_Wt ||
		 (s2 & GENMASK(3, 2)) == MEMATTR_Wt)
		final = MEMATTR_Wt;
	else
		final = MEMATTR_Wb;

	if (final != MEMATTR_NC) {
		/* Inherit RaWa hints form S1 */
		if (transient) {
			switch (s1 & GENMASK(3, 2)) {
			case MEMATTR_Wt:
				final = 0;
				break;
			case MEMATTR_Wb:
				final = MEMATTR_NC;
				break;
			}
		}

		final |= s1 & GENMASK(1, 0);
	}

	return final;
}

#define ATTR_NSH	0b00
#define ATTR_RSV	0b01
#define ATTR_OSH	0b10
#define ATTR_ISH	0b11

static u8 compute_sh(u8 attr, u64 desc)
{
	u8 sh;

	/* Any form of device, as well as NC has SH[1:0]=0b10 */
	if (MEMATTR_IS_DEVICE(attr) || attr == MEMATTR(NC, NC))
		return ATTR_OSH;

	sh = FIELD_GET(PTE_SHARED, desc);
	if (sh == ATTR_RSV)		/* Reserved, mapped to NSH */
		sh = ATTR_NSH;

	return sh;
}

static u8 combine_sh(u8 s1_sh, u8 s2_sh)
{
	if (s1_sh == ATTR_OSH || s2_sh == ATTR_OSH)
		return ATTR_OSH;
	if (s1_sh == ATTR_ISH || s2_sh == ATTR_ISH)
		return ATTR_ISH;

	return ATTR_NSH;
}

static u64 compute_par_s12(struct kvm_vcpu *vcpu, u64 s1_par,
			   struct kvm_s2_trans *tr)
{
	u8 s1_parattr, s2_memattr, final_attr;
	u64 par;

	/* If S2 has failed to translate, report the damage */
	if (tr->esr) {
		par = SYS_PAR_EL1_RES1;
		par |= SYS_PAR_EL1_F;
		par |= SYS_PAR_EL1_S;
		par |= FIELD_PREP(SYS_PAR_EL1_FST, tr->esr);
		return par;
	}

	s1_parattr = FIELD_GET(SYS_PAR_EL1_ATTR, s1_par);
	s2_memattr = FIELD_GET(GENMASK(5, 2), tr->desc);

	if (__vcpu_sys_reg(vcpu, HCR_EL2) & HCR_FWB) {
		if (!kvm_has_feat(vcpu->kvm, ID_AA64PFR2_EL1, MTEPERM, IMP))
			s2_memattr &= ~BIT(3);

		/* Combination of R_VRJSW and R_RHWZM */
		switch (s2_memattr) {
		case 0b0101:
			if (MEMATTR_IS_DEVICE(s1_parattr))
				final_attr = s1_parattr;
			else
				final_attr = MEMATTR(NC, NC);
			break;
		case 0b0110:
		case 0b1110:
			final_attr = MEMATTR(WbRaWa, WbRaWa);
			break;
		case 0b0111:
		case 0b1111:
			/* Preserve S1 attribute */
			final_attr = s1_parattr;
			break;
		case 0b0100:
		case 0b1100:
		case 0b1101:
			/* Reserved, do something non-silly */
			final_attr = s1_parattr;
			break;
		default:
			/* MemAttr[2]=0, Device from S2 */
			final_attr = s2_memattr & GENMASK(1,0) << 2;
		}
	} else {
		/* Combination of R_HMNDG, R_TNHFM and R_GQFSF */
		u8 s2_parattr = s2_memattr_to_attr(s2_memattr);

		if (MEMATTR_IS_DEVICE(s1_parattr) ||
		    MEMATTR_IS_DEVICE(s2_parattr)) {
			final_attr = min(s1_parattr, s2_parattr);
		} else {
			/* At this stage, this is memory vs memory */
			final_attr  = combine_s1_s2_attr(s1_parattr & 0xf,
							 s2_parattr & 0xf);
			final_attr |= combine_s1_s2_attr(s1_parattr >> 4,
							 s2_parattr >> 4) << 4;
		}
	}

	if ((__vcpu_sys_reg(vcpu, HCR_EL2) & HCR_CD) &&
	    !MEMATTR_IS_DEVICE(final_attr))
		final_attr = MEMATTR(NC, NC);

	par  = FIELD_PREP(SYS_PAR_EL1_ATTR, final_attr);
	par |= tr->output & GENMASK(47, 12);
	par |= FIELD_PREP(SYS_PAR_EL1_SH,
			  combine_sh(FIELD_GET(SYS_PAR_EL1_SH, s1_par),
				     compute_sh(final_attr, tr->desc)));

	return par;
}

static u64 compute_par_s1(struct kvm_vcpu *vcpu, struct s1_walk_result *wr,
			  enum trans_regime regime)
{
	u64 par;

	if (wr->failed) {
		par = SYS_PAR_EL1_RES1;
		par |= SYS_PAR_EL1_F;
		par |= FIELD_PREP(SYS_PAR_EL1_FST, wr->fst);
		par |= wr->ptw ? SYS_PAR_EL1_PTW : 0;
		par |= wr->s2 ? SYS_PAR_EL1_S : 0;
	} else if (wr->level == S1_MMU_DISABLED) {
		/* MMU off or HCR_EL2.DC == 1 */
		par  = SYS_PAR_EL1_NSE;
		par |= wr->pa & GENMASK_ULL(47, 12);

		if (regime == TR_EL10 &&
		    (__vcpu_sys_reg(vcpu, HCR_EL2) & HCR_DC)) {
			par |= FIELD_PREP(SYS_PAR_EL1_ATTR,
					  MEMATTR(WbRaWa, WbRaWa));
			par |= FIELD_PREP(SYS_PAR_EL1_SH, ATTR_NSH);
		} else {
			par |= FIELD_PREP(SYS_PAR_EL1_ATTR, 0); /* nGnRnE */
			par |= FIELD_PREP(SYS_PAR_EL1_SH, ATTR_OSH);
		}
	} else {
		u64 mair, sctlr;
		u8 sh;

		par  = SYS_PAR_EL1_NSE;

		mair = (regime == TR_EL10 ?
			vcpu_read_sys_reg(vcpu, MAIR_EL1) :
			vcpu_read_sys_reg(vcpu, MAIR_EL2));

		mair >>= FIELD_GET(PTE_ATTRINDX_MASK, wr->desc) * 8;
		mair &= 0xff;

		sctlr = (regime == TR_EL10 ?
			 vcpu_read_sys_reg(vcpu, SCTLR_EL1) :
			 vcpu_read_sys_reg(vcpu, SCTLR_EL2));

		/* Force NC for memory if SCTLR_ELx.C is clear */
		if (!(sctlr & SCTLR_EL1_C) && !MEMATTR_IS_DEVICE(mair))
			mair = MEMATTR(NC, NC);

		par |= FIELD_PREP(SYS_PAR_EL1_ATTR, mair);
		par |= wr->pa & GENMASK_ULL(47, 12);

		sh = compute_sh(mair, wr->desc);
		par |= FIELD_PREP(SYS_PAR_EL1_SH, sh);
	}

	return par;
}

static bool pan3_enabled(struct kvm_vcpu *vcpu, enum trans_regime regime)
{
	u64 sctlr;

	if (!kvm_has_feat(vcpu->kvm, ID_AA64MMFR1_EL1, PAN, PAN3))
		return false;

	if (regime == TR_EL10)
		sctlr = vcpu_read_sys_reg(vcpu, SCTLR_EL1);
	else
		sctlr = vcpu_read_sys_reg(vcpu, SCTLR_EL2);

	return sctlr & SCTLR_EL1_EPAN;
}

static u64 handle_at_slow(struct kvm_vcpu *vcpu, u32 op, u64 vaddr)
{
	bool perm_fail, ur, uw, ux, pr, pw, px;
	struct s1_walk_result wr = {};
	struct s1_walk_info wi = {};
	int ret, idx;

	ret = setup_s1_walk(vcpu, op, &wi, &wr, vaddr);
	if (ret)
		goto compute_par;

	if (wr.level == S1_MMU_DISABLED)
		goto compute_par;

	idx = srcu_read_lock(&vcpu->kvm->srcu);

	ret = walk_s1(vcpu, &wi, &wr, vaddr);

	srcu_read_unlock(&vcpu->kvm->srcu, idx);

	if (ret)
		goto compute_par;

	/* FIXME: revisit when adding indirect permission support */
	/* AArch64.S1DirectBasePermissions() */
	if (wi.regime != TR_EL2) {
		switch (FIELD_GET(PTE_USER | PTE_RDONLY, wr.desc)) {
		case 0b00:
			pr = pw = true;
			ur = uw = false;
			break;
		case 0b01:
			pr = pw = ur = uw = true;
			break;
		case 0b10:
			pr = true;
			pw = ur = uw = false;
			break;
		case 0b11:
			pr = ur = true;
			pw = uw = false;
			break;
		}

		switch (wr.APTable) {
		case 0b00:
			break;
		case 0b01:
			ur = uw = false;
			break;
		case 0b10:
			pw = uw = false;
			break;
		case 0b11:
			pw = ur = uw = false;
			break;
		}

		/* We don't use px for anything yet, but hey... */
		px = !((wr.desc & PTE_PXN) || wr.PXNTable || uw);
		ux = !((wr.desc & PTE_UXN) || wr.UXNTable);

		if (op == OP_AT_S1E1RP || op == OP_AT_S1E1WP) {
			bool pan;

			pan = *vcpu_cpsr(vcpu) & PSR_PAN_BIT;
			pan &= ur || uw || (pan3_enabled(vcpu, wi.regime) && ux);
			pw &= !pan;
			pr &= !pan;
		}
	} else {
		ur = uw = ux = false;

		if (!(wr.desc & PTE_RDONLY)) {
			pr = pw = true;
		} else {
			pr = true;
			pw = false;
		}

		if (wr.APTable & BIT(1))
			pw = false;

		/* XN maps to UXN */
		px = !((wr.desc & PTE_UXN) || wr.UXNTable);
	}

	perm_fail = false;

	switch (op) {
	case OP_AT_S1E1RP:
	case OP_AT_S1E1R:
	case OP_AT_S1E2R:
		perm_fail = !pr;
		break;
	case OP_AT_S1E1WP:
	case OP_AT_S1E1W:
	case OP_AT_S1E2W:
		perm_fail = !pw;
		break;
	case OP_AT_S1E0R:
		perm_fail = !ur;
		break;
	case OP_AT_S1E0W:
		perm_fail = !uw;
		break;
	case OP_AT_S1E1A:
	case OP_AT_S1E2A:
		break;
	default:
		BUG();
	}

	if (perm_fail)
		fail_s1_walk(&wr, ESR_ELx_FSC_PERM_L(wr.level), false, false);

compute_par:
	return compute_par_s1(vcpu, &wr, wi.regime);
}

/*
 * Return the PAR_EL1 value as the result of a valid translation.
 *
 * If the translation is unsuccessful, the value may only contain
 * PAR_EL1.F, and cannot be taken at face value. It isn't an
 * indication of the translation having failed, only that the fast
 * path did not succeed, *unless* it indicates a S1 permission fault.
 */
static u64 __kvm_at_s1e01_fast(struct kvm_vcpu *vcpu, u32 op, u64 vaddr)
{
	struct mmu_config config;
	struct kvm_s2_mmu *mmu;
	bool fail;
	u64 par;

	par = SYS_PAR_EL1_F;

	/*
	 * We've trapped, so everything is live on the CPU. As we will
	 * be switching contexts behind everybody's back, disable
	 * interrupts while holding the mmu lock.
	 */
	guard(write_lock_irqsave)(&vcpu->kvm->mmu_lock);

	/*
	 * If HCR_EL2.{E2H,TGE} == {1,1}, the MMU context is already
	 * the right one (as we trapped from vEL2). If not, save the
	 * full MMU context.
	 */
	if (vcpu_el2_e2h_is_set(vcpu) && vcpu_el2_tge_is_set(vcpu))
		goto skip_mmu_switch;

	/*
	 * Obtaining the S2 MMU for a L2 is horribly racy, and we may not
	 * find it (recycled by another vcpu, for example). When this
	 * happens, admit defeat immediately and use the SW (slow) path.
	 */
	mmu = lookup_s2_mmu(vcpu);
	if (!mmu)
		return par;

	__mmu_config_save(&config);

	write_sysreg_el1(vcpu_read_sys_reg(vcpu, TTBR0_EL1),	SYS_TTBR0);
	write_sysreg_el1(vcpu_read_sys_reg(vcpu, TTBR1_EL1),	SYS_TTBR1);
	write_sysreg_el1(vcpu_read_sys_reg(vcpu, TCR_EL1),	SYS_TCR);
	write_sysreg_el1(vcpu_read_sys_reg(vcpu, MAIR_EL1),	SYS_MAIR);
	write_sysreg_el1(vcpu_read_sys_reg(vcpu, SCTLR_EL1),	SYS_SCTLR);
	__load_stage2(mmu, mmu->arch);

skip_mmu_switch:
	/* Clear TGE, enable S2 translation, we're rolling */
	write_sysreg((config.hcr & ~HCR_TGE) | HCR_VM,	hcr_el2);
	isb();

	switch (op) {
	case OP_AT_S1E1RP:
	case OP_AT_S1E1WP:
		fail = at_s1e1p_fast(vcpu, op, vaddr);
		break;
	case OP_AT_S1E1R:
		fail = __kvm_at(OP_AT_S1E1R, vaddr);
		break;
	case OP_AT_S1E1W:
		fail = __kvm_at(OP_AT_S1E1W, vaddr);
		break;
	case OP_AT_S1E0R:
		fail = __kvm_at(OP_AT_S1E0R, vaddr);
		break;
	case OP_AT_S1E0W:
		fail = __kvm_at(OP_AT_S1E0W, vaddr);
		break;
	case OP_AT_S1E1A:
		fail = __kvm_at(OP_AT_S1E1A, vaddr);
		break;
	default:
		WARN_ON_ONCE(1);
		fail = true;
		break;
	}

	if (!fail)
		par = read_sysreg_par();

	if (!(vcpu_el2_e2h_is_set(vcpu) && vcpu_el2_tge_is_set(vcpu)))
		__mmu_config_restore(&config);

	return par;
}

static bool par_check_s1_perm_fault(u64 par)
{
	u8 fst = FIELD_GET(SYS_PAR_EL1_FST, par);

	return  ((fst & ESR_ELx_FSC_TYPE) == ESR_ELx_FSC_PERM &&
		 !(par & SYS_PAR_EL1_S));
}

void __kvm_at_s1e01(struct kvm_vcpu *vcpu, u32 op, u64 vaddr)
{
	u64 par = __kvm_at_s1e01_fast(vcpu, op, vaddr);

	/*
	 * If PAR_EL1 reports that AT failed on a S1 permission fault, we
	 * know for sure that the PTW was able to walk the S1 tables and
	 * there's nothing else to do.
	 *
	 * If AT failed for any other reason, then we must walk the guest S1
	 * to emulate the instruction.
	 */
	if ((par & SYS_PAR_EL1_F) && !par_check_s1_perm_fault(par))
		par = handle_at_slow(vcpu, op, vaddr);

	vcpu_write_sys_reg(vcpu, par, PAR_EL1);
}

void __kvm_at_s1e2(struct kvm_vcpu *vcpu, u32 op, u64 vaddr)
{
	u64 par;

	/*
	 * We've trapped, so everything is live on the CPU. As we will be
	 * switching context behind everybody's back, disable interrupts...
	 */
	scoped_guard(write_lock_irqsave, &vcpu->kvm->mmu_lock) {
		struct kvm_s2_mmu *mmu;
		u64 val, hcr;
		bool fail;

		mmu = &vcpu->kvm->arch.mmu;

		val = hcr = read_sysreg(hcr_el2);
		val &= ~HCR_TGE;
		val |= HCR_VM;

		if (!vcpu_el2_e2h_is_set(vcpu))
			val |= HCR_NV | HCR_NV1;

		write_sysreg(val, hcr_el2);
		isb();

		par = SYS_PAR_EL1_F;

		switch (op) {
		case OP_AT_S1E2R:
			fail = __kvm_at(OP_AT_S1E1R, vaddr);
			break;
		case OP_AT_S1E2W:
			fail = __kvm_at(OP_AT_S1E1W, vaddr);
			break;
		case OP_AT_S1E2A:
			fail = __kvm_at(OP_AT_S1E1A, vaddr);
			break;
		default:
			WARN_ON_ONCE(1);
			fail = true;
		}

		isb();

		if (!fail)
			par = read_sysreg_par();

		write_sysreg(hcr, hcr_el2);
		isb();
	}

	/* We failed the translation, let's replay it in slow motion */
	if ((par & SYS_PAR_EL1_F) && !par_check_s1_perm_fault(par))
		par = handle_at_slow(vcpu, op, vaddr);

	vcpu_write_sys_reg(vcpu, par, PAR_EL1);
}

void __kvm_at_s12(struct kvm_vcpu *vcpu, u32 op, u64 vaddr)
{
	struct kvm_s2_trans out = {};
	u64 ipa, par;
	bool write;
	int ret;

	/* Do the stage-1 translation */
	switch (op) {
	case OP_AT_S12E1R:
		op = OP_AT_S1E1R;
		write = false;
		break;
	case OP_AT_S12E1W:
		op = OP_AT_S1E1W;
		write = true;
		break;
	case OP_AT_S12E0R:
		op = OP_AT_S1E0R;
		write = false;
		break;
	case OP_AT_S12E0W:
		op = OP_AT_S1E0W;
		write = true;
		break;
	default:
		WARN_ON_ONCE(1);
		return;
	}

	__kvm_at_s1e01(vcpu, op, vaddr);
	par = vcpu_read_sys_reg(vcpu, PAR_EL1);
	if (par & SYS_PAR_EL1_F)
		return;

	/*
	 * If we only have a single stage of translation (E2H=0 or
	 * TGE=1), exit early. Same thing if {VM,DC}=={0,0}.
	 */
	if (!vcpu_el2_e2h_is_set(vcpu) || vcpu_el2_tge_is_set(vcpu) ||
	    !(vcpu_read_sys_reg(vcpu, HCR_EL2) & (HCR_VM | HCR_DC)))
		return;

	/* Do the stage-2 translation */
	ipa = (par & GENMASK_ULL(47, 12)) | (vaddr & GENMASK_ULL(11, 0));
	out.esr = 0;
	ret = kvm_walk_nested_s2(vcpu, ipa, &out);
	if (ret < 0)
		return;

	/* Check the access permission */
	if (!out.esr &&
	    ((!write && !out.readable) || (write && !out.writable)))
		out.esr = ESR_ELx_FSC_PERM_L(out.level & 0x3);

	par = compute_par_s12(vcpu, par, &out);
	vcpu_write_sys_reg(vcpu, par, PAR_EL1);
}
