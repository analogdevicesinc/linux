/*
 * Copyright 2022 Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "amdgpu.h"
#include "nbio_v7_9.h"
#include "amdgpu_ip.h"
#include "amdgpu_ras.h"

#include "nbio/nbio_7_9_0_offset.h"
#include "nbio/nbio_7_9_0_sh_mask.h"
#include "ivsrcid/nbio/irqsrcs_nbif_7_4.h"
#include <uapi/linux/kfd_ioctl.h>

#define NPS_MODE_MASK 0x000000FFL

static const struct amdgpu_nbio_doorbell_layout
nbio_v7_9_doorbell_layout = {
	.ctrl = {
		.enable = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_ENABLE),
		.awid = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_AWID),
		.fence_enable = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_FENCE_ENABLE),
		.range_offset = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_RANGE_OFFSET),
		.range_size = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_RANGE_SIZE),
		.disable_64bit = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_64BIT_SUPPORT_DIS),
		.deduct_range_offset = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_NEED_DEDUCT_RANGE_OFFSET),
		.awaddr_31_28 = REG_FIELD_MASK(
			S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_AWADDR_31_28_VALUE),
	},
};

static void nbio_v7_9_init_doorbell_entry_registers(
	struct amdgpu_device *adev)
{
	static const u8 sdma_port[] = { 1, 2, 5, 6 };
	struct amdgpu_nbio_doorbell_entries *entries;
	u32 ctrl_base, dev_inst, local_inst;
	int i;

	memset(adev->nbio.doorbell_entries, 0,
	       sizeof(adev->nbio.doorbell_entries));
	adev->nbio.doorbell_layout = &nbio_v7_9_doorbell_layout;

	ctrl_base = SOC15_REG_OFFSET(NBIO, 0,
				     regS2A_DOORBELL_ENTRY_0_CTRL);

	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_IH];
	entries->num_entries = 1;
	entries->entry[0].port = 3;
	entries->entry[0].ctrl_reg = ctrl_base + 3;

	/*
	 * Unlike IH, SDMA and VCN use global SMN addresses. Their callers invoke
	 * the callbacks once per available instance to cover all AIDs across the
	 * die.
	 */
	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_SDMA];
	entries->num_entries = adev->sdma.num_instances;
	if (WARN_ON_ONCE(entries->num_entries >
			 AMDGPU_NBIO_MAX_DOORBELL_INSTANCES) ||
	    WARN_ON_ONCE(entries->num_entries &&
			 !adev->sdma.num_inst_per_aid)) {
		entries->num_entries = 0;
		return;
	}
	for (i = 0; i < entries->num_entries; i++) {
		dev_inst = GET_INST(SDMA0, i);
		local_inst = dev_inst % adev->sdma.num_inst_per_aid;
		if (WARN_ON_ONCE(local_inst >= ARRAY_SIZE(sdma_port))) {
			entries->num_entries = 0;
			return;
		}
		entries->entry[i].port = sdma_port[local_inst];
		entries->entry[i].ctrl_reg =
			regS2A_DOORBELL_ENTRY_0_CTRL + sdma_port[local_inst];
		entries->entry[i].use_ext = true;
		entries->entry[i].reg_inst = adev->sdma.instance[i].aid_id;
		entries->entry[i].ctrl_reg_base_idx =
			regS2A_DOORBELL_ENTRY_0_CTRL_BASE_IDX;
	}

	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_VCN];
	entries->num_entries = fls(adev->aid_mask);
	if (WARN_ON_ONCE(entries->num_entries >
			 AMDGPU_NBIO_MAX_DOORBELL_INSTANCES)) {
		entries->num_entries = 0;
		return;
	}
	for_each_inst(i, adev->aid_mask) {
		entries->entry[i].port = 4;
		entries->entry[i].ctrl_reg = regS2A_DOORBELL_ENTRY_4_CTRL;
		entries->entry[i].use_ext = true;
		entries->entry[i].reg_inst = i;
		entries->entry[i].ctrl_reg_base_idx =
			regS2A_DOORBELL_ENTRY_4_CTRL_BASE_IDX;
	}
}

static u32 nbio_v7_9_build_doorbell_range(u32 range_offset,
					  u32 range_size, u32 fence_enable)
{
	u32 doorbell_range = 0;

	doorbell_range = REG_SET_FIELD(doorbell_range,
		DOORBELL0_CTRL_ENTRY_0, BIF_DOORBELL0_RANGE_OFFSET_ENTRY,
		range_offset);
	doorbell_range = REG_SET_FIELD(doorbell_range,
		DOORBELL0_CTRL_ENTRY_0, BIF_DOORBELL0_RANGE_SIZE_ENTRY,
		range_size);
	doorbell_range = REG_SET_FIELD(doorbell_range,
		DOORBELL0_CTRL_ENTRY_0, DOORBELL0_FENCE_ENABLE_ENTRY,
		fence_enable);

	return doorbell_range;
}

static void nbio_v7_9_program_doorbell_range(struct amdgpu_device *adev,
					      u32 entry, u32 doorbell_range)
{
	WREG32_SOC15_OFFSET(NBIO, 0, regDOORBELL0_CTRL_ENTRY_0,
			     entry, doorbell_range);
}

static void nbio_v7_9_remap_hdp_registers(struct amdgpu_device *adev)
{
	WREG32_SOC15(NBIO, 0, regBIF_BX0_REMAP_HDP_MEM_FLUSH_CNTL,
		adev->rmmio_remap.reg_offset + KFD_MMIO_REMAP_HDP_MEM_FLUSH_CNTL);
	WREG32_SOC15(NBIO, 0, regBIF_BX0_REMAP_HDP_REG_FLUSH_CNTL,
		adev->rmmio_remap.reg_offset + KFD_MMIO_REMAP_HDP_REG_FLUSH_CNTL);
}

static u32 nbio_v7_9_get_rev_id(struct amdgpu_device *adev)
{
	u32 rev_id;

	/*
	 * fetch the sub-revision field from the IP-discovery table
	 * (returns zero if the table entry is not populated).
	 */
	if (amdgpu_sriov_vf(adev)) {
		rev_id = IP_VERSION_SUBREV(amdgpu_ip_version_full(adev, NBIO_HWIP, 0));
	} else {
		rev_id = RREG32_SOC15(NBIO, 0, regRCC_STRAP0_RCC_DEV0_EPF0_STRAP0);
		rev_id = REG_GET_FIELD(rev_id, RCC_STRAP0_RCC_DEV0_EPF0_STRAP0,
				STRAP_ATI_REV_ID_DEV0_F0);
	}

	return rev_id;
}

static void nbio_v7_9_mc_access_enable(struct amdgpu_device *adev, bool enable)
{
	if (enable)
		WREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_FB_EN,
			BIF_BX0_BIF_FB_EN__FB_READ_EN_MASK | BIF_BX0_BIF_FB_EN__FB_WRITE_EN_MASK);
	else
		WREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_FB_EN, 0);
}

static u32 nbio_v7_9_get_memsize(struct amdgpu_device *adev)
{
	return RREG32_SOC15(NBIO, 0, regRCC_DEV0_EPF0_RCC_CONFIG_MEMSIZE);
}

static void nbio_v7_9_sdma_doorbell_range(struct amdgpu_device *adev, int instance,
			bool use_doorbell, int doorbell_index, int doorbell_size)
{
	static const u8 sdma_awid[] = { 0xe, 0x8, 0x9, 0xa };
	static const u8 sdma_awaddr[] = { 0x1, 0x2, 0x8, 0x9 };
	struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.range_size = doorbell_size,
	};
	u32 doorbell_range, ctrl;
	int aid_id, dev_inst, local_inst;

	dev_inst = GET_INST(SDMA0, instance);
	aid_id = adev->sdma.instance[instance].aid_id;
	local_inst = dev_inst % adev->sdma.num_inst_per_aid;

	if (WARN_ON_ONCE((unsigned int)local_inst >= ARRAY_SIZE(sdma_awid)))
		return;

	if (!use_doorbell) {
		nbio_v7_9_program_doorbell_range(adev,
			1 + local_inst + 4 * aid_id, 0);
		amdgpu_nbio_program_doorbell_entry(adev,
			AMDGPU_NBIO_DOORBELL_CLIENT_SDMA, instance,
			NULL, NULL);
		return;
	}

	doorbell_range = nbio_v7_9_build_doorbell_range(
		doorbell_index, doorbell_size, 0);
	fields.awid = sdma_awid[local_inst];
	fields.range_offset = fields.awid;
	fields.awaddr_31_28 = sdma_awaddr[local_inst];
	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
			adev, &fields, &ctrl, NULL)))
		return;

	nbio_v7_9_program_doorbell_range(adev,
		1 + local_inst + 4 * aid_id, doorbell_range);
	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_SDMA, instance, &ctrl, NULL);
}

static void nbio_v7_9_vcn_doorbell_range(struct amdgpu_device *adev, bool use_doorbell,
					 int doorbell_index, int instance)
{
	struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.awid = 0x4,
		.range_offset = 0x4,
		.awaddr_31_28 = 0x4,
	};
	u32 doorbell_range, ctrl;
	u32 aid_id = instance;
	u32 range_size;

	if (!use_doorbell) {
		nbio_v7_9_program_doorbell_range(adev, 17 + aid_id, 0);
		amdgpu_nbio_program_doorbell_entry(adev,
			AMDGPU_NBIO_DOORBELL_CLIENT_VCN, aid_id,
			NULL, NULL);
		return;
	}

	range_size = (amdgpu_ip_version(adev, GC_HWIP, 0) ==
						IP_VERSION(9, 5, 0)) ?
						0xb : 0x9;
	doorbell_range = nbio_v7_9_build_doorbell_range(
		doorbell_index, range_size, aid_id ? 0x4 : 0);
	fields.range_size = range_size;
	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
			adev, &fields, &ctrl, NULL)))
		return;

	nbio_v7_9_program_doorbell_range(adev, 17 + aid_id, doorbell_range);
	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_VCN, aid_id, &ctrl, NULL);
}

static void nbio_v7_9_enable_doorbell_aperture(struct amdgpu_device *adev,
					       bool enable)
{
	/* Enable to allow doorbell pass thru on pre-silicon bare-metal */
	WREG32_SOC15(NBIO, 0, regBIFC_DOORBELL_ACCESS_EN_PF, 0xfffff);
	WREG32_FIELD15_PREREG(NBIO, 0, RCC_DEV0_EPF0_RCC_DOORBELL_APER_EN,
			BIF_DOORBELL_APER_EN, enable ? 1 : 0);
}

static void nbio_v7_9_enable_doorbell_selfring_aperture(struct amdgpu_device *adev,
							bool enable)
{
	u32 tmp = 0;

	if (enable) {
		tmp = REG_SET_FIELD(tmp, BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL,
				    DOORBELL_SELFRING_GPA_APER_EN, 1) |
		      REG_SET_FIELD(tmp, BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL,
				    DOORBELL_SELFRING_GPA_APER_MODE, 1) |
		      REG_SET_FIELD(tmp, BIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL,
				    DOORBELL_SELFRING_GPA_APER_SIZE, 0);

		WREG32_SOC15(NBIO, 0, regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_LOW,
			     lower_32_bits(adev->doorbell.base));
		WREG32_SOC15(NBIO, 0, regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_BASE_HIGH,
			     upper_32_bits(adev->doorbell.base));
	}

	WREG32_SOC15(NBIO, 0, regBIF_BX_PF0_DOORBELL_SELFRING_GPA_APER_CNTL, tmp);
}

static void nbio_v7_9_ih_doorbell_range(struct amdgpu_device *adev,
					bool use_doorbell, int doorbell_index)
{
	const struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.range_size = 0x8,
	};
	u32 doorbell_range, ctrl;

	if (!use_doorbell) {
		nbio_v7_9_program_doorbell_range(adev, 0, 0);
		amdgpu_nbio_program_doorbell_entry(adev,
			AMDGPU_NBIO_DOORBELL_CLIENT_IH, 0, NULL, NULL);
		return;
	}

	doorbell_range = nbio_v7_9_build_doorbell_range(
		doorbell_index, 0x8, 0);
	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
			adev, &fields, &ctrl, NULL)))
		return;

	nbio_v7_9_program_doorbell_range(adev, 0, doorbell_range);
	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_IH, 0, &ctrl, NULL);
}


static void nbio_v7_9_update_medium_grain_clock_gating(struct amdgpu_device *adev,
						       bool enable)
{
}

static void nbio_v7_9_update_medium_grain_light_sleep(struct amdgpu_device *adev,
						      bool enable)
{
}

static void nbio_v7_9_get_clockgating_state(struct amdgpu_device *adev,
					    u64 *flags)
{
}

static void nbio_v7_9_ih_control(struct amdgpu_device *adev)
{
	u32 interrupt_cntl;

	/* setup interrupt control */
	WREG32_SOC15(NBIO, 0, regBIF_BX0_INTERRUPT_CNTL2, adev->dummy_page_addr >> 8);
	interrupt_cntl = RREG32_SOC15(NBIO, 0, regBIF_BX0_INTERRUPT_CNTL);
	/* INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK=0 - dummy read disabled with msi, enabled without msi
	 * INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK=1 - dummy read controlled by IH_DUMMY_RD_EN
	 */
	interrupt_cntl =
		REG_SET_FIELD(interrupt_cntl, BIF_BX0_INTERRUPT_CNTL, IH_DUMMY_RD_OVERRIDE, 0);
	/* INTERRUPT_CNTL__IH_REQ_NONSNOOP_EN_MASK=1 if ring is in non-cacheable memory, e.g., vram */
	interrupt_cntl =
		REG_SET_FIELD(interrupt_cntl, BIF_BX0_INTERRUPT_CNTL, IH_REQ_NONSNOOP_EN, 0);
	WREG32_SOC15(NBIO, 0, regBIF_BX0_INTERRUPT_CNTL, interrupt_cntl);
}

static u32 nbio_v7_9_get_hdp_flush_req_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX_PF0_GPU_HDP_FLUSH_REQ);
}

static u32 nbio_v7_9_get_hdp_flush_done_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX_PF0_GPU_HDP_FLUSH_DONE);
}

static u32 nbio_v7_9_get_pcie_index_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX0_PCIE_INDEX2);
}

static u32 nbio_v7_9_get_pcie_data_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX0_PCIE_DATA2);
}

static u32 nbio_v7_9_get_pcie_index_hi_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX0_PCIE_INDEX2_HI);
}

const struct nbio_hdp_flush_reg nbio_v7_9_hdp_flush_reg = {
	.ref_and_mask_cp0 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP0_MASK,
	.ref_and_mask_cp1 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP1_MASK,
	.ref_and_mask_cp2 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP2_MASK,
	.ref_and_mask_cp3 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP3_MASK,
	.ref_and_mask_cp4 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP4_MASK,
	.ref_and_mask_cp5 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP5_MASK,
	.ref_and_mask_cp6 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP6_MASK,
	.ref_and_mask_cp7 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP7_MASK,
	.ref_and_mask_cp8 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP8_MASK,
	.ref_and_mask_cp9 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__CP9_MASK,
	.ref_and_mask_sdma0 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__SDMA0_MASK,
	.ref_and_mask_sdma1 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__SDMA1_MASK,
	.ref_and_mask_sdma2 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG0_MASK,
	.ref_and_mask_sdma3 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG1_MASK,
	.ref_and_mask_sdma4 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG2_MASK,
	.ref_and_mask_sdma5 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG3_MASK,
	.ref_and_mask_sdma6 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG4_MASK,
	.ref_and_mask_sdma7 = BIF_BX_PF0_GPU_HDP_FLUSH_DONE__RSVD_ENG5_MASK,
};

static void nbio_v7_9_enable_doorbell_interrupt(struct amdgpu_device *adev,
						bool enable)
{
	WREG32_FIELD15_PREREG(NBIO, 0, BIF_BX0_BIF_DOORBELL_INT_CNTL,
			      DOORBELL_INTERRUPT_DISABLE, enable ? 0 : 1);
}

static int nbio_v7_9_get_compute_partition_mode(struct amdgpu_device *adev)
{
	u32 tmp, px;

	tmp = RREG32_SOC15(NBIO, 0, regBIF_BX_PF0_PARTITION_COMPUTE_STATUS);
	px = REG_GET_FIELD(tmp, BIF_BX_PF0_PARTITION_COMPUTE_STATUS,
			   PARTITION_MODE);

	return px;
}

static bool nbio_v7_9_is_nps_switch_requested(struct amdgpu_device *adev)
{
	u32 tmp;

	tmp = RREG32_SOC15(NBIO, 0, regBIF_BX_PF0_PARTITION_MEM_STATUS);
	tmp = REG_GET_FIELD(tmp, BIF_BX_PF0_PARTITION_MEM_STATUS,
			    CHANGE_STATUE);

	/* 0x8 - NPS switch requested */
	return (tmp == 0x8);
}
static u32 nbio_v7_9_get_memory_partition_mode(struct amdgpu_device *adev,
					       u32 *supp_modes)
{
	u32 tmp;

	tmp = RREG32_SOC15(NBIO, 0, regBIF_BX_PF0_PARTITION_MEM_STATUS);
	tmp = REG_GET_FIELD(tmp, BIF_BX_PF0_PARTITION_MEM_STATUS, NPS_MODE);

	if (supp_modes) {
		*supp_modes =
			RREG32_SOC15(NBIO, 0, regBIF_BX_PF0_PARTITION_MEM_CAP);
	}

	return ffs(tmp);
}

static void nbio_v7_9_init_registers(struct amdgpu_device *adev)
{
	u32 inst_mask;
	int i;

	nbio_v7_9_init_doorbell_entry_registers(adev);

	WREG32_SOC15(NBIO, 0, regXCC_DOORBELL_FENCE,
		0xff & ~(adev->gfx.xcc_mask));

	WREG32_SOC15(NBIO, 0, regBIFC_GFX_INT_MONITOR_MASK, 0x7ff);

	inst_mask = adev->aid_mask & ~1U;
	for_each_inst(i, inst_mask) {
		WREG32_SOC15_EXT(NBIO, i, regXCC_DOORBELL_FENCE, i,
			XCC_DOORBELL_FENCE__SHUB_SLV_MODE_MASK);

	}

	if (!amdgpu_sriov_vf(adev)) {
		u32 baco_cntl;
		for_each_inst(i, adev->aid_mask) {
			baco_cntl = RREG32_SOC15(NBIO, i, regBIF_BX0_BACO_CNTL);
			if (baco_cntl & (BIF_BX0_BACO_CNTL__BACO_DUMMY_EN_MASK |
					 BIF_BX0_BACO_CNTL__BACO_EN_MASK)) {
				baco_cntl &= ~(
					BIF_BX0_BACO_CNTL__BACO_DUMMY_EN_MASK |
					BIF_BX0_BACO_CNTL__BACO_EN_MASK);
				dev_dbg(adev->dev,
					"Unsetting baco dummy mode %x",
					baco_cntl);
				WREG32_SOC15(NBIO, i, regBIF_BX0_BACO_CNTL,
					     baco_cntl);
			}
		}
	}
}

#define MMIO_REG_HOLE_OFFSET 0x1A000

static void nbio_v7_9_set_reg_remap(struct amdgpu_device *adev)
{
	if (!amdgpu_sriov_vf(adev) && (PAGE_SIZE <= 4096)) {
		adev->rmmio_remap.reg_offset = MMIO_REG_HOLE_OFFSET;
		adev->rmmio_remap.bus_addr = adev->rmmio_base + MMIO_REG_HOLE_OFFSET;
	} else {
		adev->rmmio_remap.reg_offset =
			SOC15_REG_OFFSET(
				NBIO, 0,
				regBIF_BX_DEV0_EPF0_VF0_HDP_MEM_COHERENCY_FLUSH_CNTL)
			<< 2;
		adev->rmmio_remap.bus_addr = 0;
	}
}

const struct amdgpu_nbio_funcs nbio_v7_9_funcs = {
	.get_hdp_flush_req_offset = nbio_v7_9_get_hdp_flush_req_offset,
	.get_hdp_flush_done_offset = nbio_v7_9_get_hdp_flush_done_offset,
	.get_pcie_index_offset = nbio_v7_9_get_pcie_index_offset,
	.get_pcie_data_offset = nbio_v7_9_get_pcie_data_offset,
	.get_pcie_index_hi_offset = nbio_v7_9_get_pcie_index_hi_offset,
	.get_rev_id = nbio_v7_9_get_rev_id,
	.mc_access_enable = nbio_v7_9_mc_access_enable,
	.get_memsize = nbio_v7_9_get_memsize,
	.sdma_doorbell_range = nbio_v7_9_sdma_doorbell_range,
	.vcn_doorbell_range = nbio_v7_9_vcn_doorbell_range,
	.enable_doorbell_aperture = nbio_v7_9_enable_doorbell_aperture,
	.enable_doorbell_selfring_aperture = nbio_v7_9_enable_doorbell_selfring_aperture,
	.ih_doorbell_range = nbio_v7_9_ih_doorbell_range,
	.enable_doorbell_interrupt = nbio_v7_9_enable_doorbell_interrupt,
	.update_medium_grain_clock_gating = nbio_v7_9_update_medium_grain_clock_gating,
	.update_medium_grain_light_sleep = nbio_v7_9_update_medium_grain_light_sleep,
	.get_clockgating_state = nbio_v7_9_get_clockgating_state,
	.ih_control = nbio_v7_9_ih_control,
	.remap_hdp_registers = nbio_v7_9_remap_hdp_registers,
	.get_compute_partition_mode = nbio_v7_9_get_compute_partition_mode,
	.get_memory_partition_mode = nbio_v7_9_get_memory_partition_mode,
	.is_nps_switch_requested = nbio_v7_9_is_nps_switch_requested,
	.init_registers = nbio_v7_9_init_registers,
	.set_reg_remap = nbio_v7_9_set_reg_remap,
};

static void nbio_v7_9_query_ras_error_count(struct amdgpu_device *adev,
					void *ras_error_status)
{
}

static void nbio_v7_9_handle_ras_controller_intr_no_bifring(struct amdgpu_device *adev)
{
	uint32_t bif_doorbell_intr_cntl;
	struct ras_manager *obj = amdgpu_ras_find_obj(adev, adev->nbio.ras_if);
	struct ras_err_data err_data;
	struct amdgpu_ras *ras = amdgpu_ras_get_context(adev);

	if (amdgpu_ras_error_data_init(&err_data))
		return;

	bif_doorbell_intr_cntl = RREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_DOORBELL_INT_CNTL);

	if (REG_GET_FIELD(bif_doorbell_intr_cntl,
		BIF_BX0_BIF_DOORBELL_INT_CNTL, RAS_CNTLR_INTERRUPT_STATUS)) {
		/* driver has to clear the interrupt status when bif ring is disabled */
		bif_doorbell_intr_cntl = REG_SET_FIELD(bif_doorbell_intr_cntl,
						BIF_BX0_BIF_DOORBELL_INT_CNTL,
						RAS_CNTLR_INTERRUPT_CLEAR, 1);
		WREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_DOORBELL_INT_CNTL, bif_doorbell_intr_cntl);

		if (ras && !ras->disable_ras_err_cnt_harvest && obj) {
			/*
			 * clear error status after ras_controller_intr
			 * according to hw team and count ue number
			 * for query
			 */
			nbio_v7_9_query_ras_error_count(adev, &err_data);

			/* logging on error cnt and printing for awareness */
			obj->err_data.ue_count += err_data.ue_count;
			obj->err_data.ce_count += err_data.ce_count;

			if (err_data.ce_count)
				dev_info(adev->dev, "%ld correctable hardware "
						"errors detected in %s block\n",
						obj->err_data.ce_count,
						get_ras_block_str(adev->nbio.ras_if));

			if (err_data.ue_count)
				dev_info(adev->dev, "%ld uncorrectable hardware "
						"errors detected in %s block\n",
						obj->err_data.ue_count,
						get_ras_block_str(adev->nbio.ras_if));
		}

		dev_info(adev->dev, "RAS controller interrupt triggered "
					"by NBIF error\n");
	}

	amdgpu_ras_error_data_fini(&err_data);
}

static void nbio_v7_9_handle_ras_err_event_athub_intr_no_bifring(struct amdgpu_device *adev)
{
	uint32_t bif_doorbell_intr_cntl;

	bif_doorbell_intr_cntl = RREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_DOORBELL_INT_CNTL);

	if (REG_GET_FIELD(bif_doorbell_intr_cntl,
		BIF_BX0_BIF_DOORBELL_INT_CNTL, RAS_ATHUB_ERR_EVENT_INTERRUPT_STATUS)) {
		/* driver has to clear the interrupt status when bif ring is disabled */
		bif_doorbell_intr_cntl = REG_SET_FIELD(bif_doorbell_intr_cntl,
						BIF_BX0_BIF_DOORBELL_INT_CNTL,
						RAS_ATHUB_ERR_EVENT_INTERRUPT_CLEAR, 1);

		WREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_DOORBELL_INT_CNTL, bif_doorbell_intr_cntl);

		amdgpu_ras_global_ras_isr(adev);
	}
}

static int nbio_v7_9_set_ras_controller_irq_state(struct amdgpu_device *adev,
						  struct amdgpu_irq_src *src,
						  unsigned type,
						  enum amdgpu_interrupt_state state)
{
	/* Dummy function, there is no initialization operation in driver */

	return 0;
}

static int nbio_v7_9_process_ras_controller_irq(struct amdgpu_device *adev,
						struct amdgpu_irq_src *source,
						struct amdgpu_iv_entry *entry)
{
	/* By design, the ih cookie for ras_controller_irq should be written
	 * to BIFring instead of general iv ring. However, due to known bif ring
	 * hw bug, it has to be disabled. There is no chance the process function
	 * will be involked. Just left it as a dummy one.
	 */
	return 0;
}

static int nbio_v7_9_set_ras_err_event_athub_irq_state(struct amdgpu_device *adev,
						       struct amdgpu_irq_src *src,
						       unsigned type,
						       enum amdgpu_interrupt_state state)
{
	/* Dummy function, there is no initialization operation in driver */

	return 0;
}

static int nbio_v7_9_process_err_event_athub_irq(struct amdgpu_device *adev,
						 struct amdgpu_irq_src *source,
						 struct amdgpu_iv_entry *entry)
{
	/* By design, the ih cookie for err_event_athub_irq should be written
	 * to BIFring instead of general iv ring. However, due to known bif ring
	 * hw bug, it has to be disabled. There is no chance the process function
	 * will be involked. Just left it as a dummy one.
	 */
	return 0;
}

static const struct amdgpu_irq_src_funcs nbio_v7_9_ras_controller_irq_funcs = {
	.set = nbio_v7_9_set_ras_controller_irq_state,
	.process = nbio_v7_9_process_ras_controller_irq,
};

static const struct amdgpu_irq_src_funcs nbio_v7_9_ras_err_event_athub_irq_funcs = {
	.set = nbio_v7_9_set_ras_err_event_athub_irq_state,
	.process = nbio_v7_9_process_err_event_athub_irq,
};

static int nbio_v7_9_init_ras_controller_interrupt (struct amdgpu_device *adev)
{
	int r;

	/* init the irq funcs */
	adev->nbio.ras_controller_irq.funcs =
		&nbio_v7_9_ras_controller_irq_funcs;
	adev->nbio.ras_controller_irq.num_types = 1;

	/* register ras controller interrupt */
	r = amdgpu_irq_add_id(adev, SOC15_IH_CLIENTID_BIF,
			      NBIF_7_4__SRCID__RAS_CONTROLLER_INTERRUPT,
			      &adev->nbio.ras_controller_irq);

	return r;
}

static int nbio_v7_9_init_ras_err_event_athub_interrupt (struct amdgpu_device *adev)
{

	int r;

	/* init the irq funcs */
	adev->nbio.ras_err_event_athub_irq.funcs =
		&nbio_v7_9_ras_err_event_athub_irq_funcs;
	adev->nbio.ras_err_event_athub_irq.num_types = 1;

	/* register ras err event athub interrupt */
	r = amdgpu_irq_add_id(adev, SOC15_IH_CLIENTID_BIF,
			      NBIF_7_4__SRCID__ERREVENT_ATHUB_INTERRUPT,
			      &adev->nbio.ras_err_event_athub_irq);

	return r;
}

const struct amdgpu_ras_block_hw_ops nbio_v7_9_ras_hw_ops = {
	.query_ras_error_count = nbio_v7_9_query_ras_error_count,
};

struct amdgpu_nbio_ras nbio_v7_9_ras = {
	.ras_block = {
		.ras_comm = {
			.name = "pcie_bif",
			.block = AMDGPU_RAS_BLOCK__PCIE_BIF,
			.type = AMDGPU_RAS_ERROR__MULTI_UNCORRECTABLE,
		},
		.hw_ops = &nbio_v7_9_ras_hw_ops,
		.ras_late_init = amdgpu_nbio_ras_late_init,
	},
	.handle_ras_controller_intr_no_bifring = nbio_v7_9_handle_ras_controller_intr_no_bifring,
	.handle_ras_err_event_athub_intr_no_bifring = nbio_v7_9_handle_ras_err_event_athub_intr_no_bifring,
	.init_ras_controller_interrupt = nbio_v7_9_init_ras_controller_interrupt,
	.init_ras_err_event_athub_interrupt = nbio_v7_9_init_ras_err_event_athub_interrupt,
};
