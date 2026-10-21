/*
 * Copyright 2025 Advanced Micro Devices, Inc.
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
#include "nbio/nbio_6_3_2_offset.h"
#include "nbio/nbio_6_3_2_sh_mask.h"

#include "amdgpu.h"
#include "nbio_v6_3_2.h"

static const struct amdgpu_nbio_doorbell_layout
nbio_v6_3_2_doorbell_layout = {
	.ctrl = {
		.enable = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_ENABLE),
		.awid = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_AWID),
		.fence_enable = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_FENCE_ENABLE),
		.range_offset = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_RANGE_OFFSET),
		.range_size = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_RANGE_SIZE),
		.disable_64bit = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_64BIT_SUPPORT_DIS),
		.deduct_range_offset = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_NEED_DEDUCT_RANGE_OFFSET),
		.drop = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_DROP_EN),
		.awaddr_31_28 = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL,
			S2A_DOORBELL_PORT0_AWADDR_31_28_VALUE),
	},
	.ctrl1 = {
		.target_port_type = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1,
			S2A_DOORBELL_PORT0_TARGET_PORT_TYPE),
		.target_die_id = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1,
			S2A_DOORBELL_PORT0_TARGET_DIEID),
		.target_port_id = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1,
			S2A_DOORBELL_PORT0_TARGET_PORT_ID),
		.override_poison_data = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1,
			S2A_DOORBELL_PORT0_OVERRIDE_POISON_DATA),
		.disable_4k_page_decode = REG_FIELD_MASK(
			GDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1,
			S2A_DOORBELL_PORT0_4K_PAGE_DECODE_DISABLE),
	},
	.has_ctrl1 = true,
};

static u32 nbio_v6_3_2_get_pcie_index_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX0_PCIE_INDEX2);
}

static u32 nbio_v6_3_2_get_pcie_data_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX0_PCIE_DATA2);
}

static u32 nbio_v6_3_2_get_pcie_index_hi_offset(struct amdgpu_device *adev)
{
	return SOC15_REG_OFFSET(NBIO, 0, regBIF_BX0_PCIE_INDEX2_HI);
}

static u32 nbio_v6_3_2_get_rev_id(struct amdgpu_device *adev)
{
	u32 tmp;

	/* TODO: RCC_STRAP0_RCC_DEV0_EPF0_STRAP0 is not accessible from
	 * guest side. It requires bootloader to update specific fields
	 * in ip discovery table to identify soc revision id.
	 * Return 0 when the function is called from guest side until
	 * bootloader change is available.
	 */
	if (amdgpu_sriov_vf(adev))
		return 0;

	tmp = RREG32_SOC15(NBIO, 0, regRCC_STRAP0_RCC_DEV0_EPF0_STRAP0);
	tmp = REG_GET_FIELD(tmp, RCC_STRAP0_RCC_DEV0_EPF0_STRAP0,
			    STRAP_ATI_REV_ID_DEV0_F0);

	return tmp;
}

static void nbio_v6_3_2_mc_access_enable(struct amdgpu_device *adev,
					 bool enable)
{
	if (enable)
		WREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_FB_EN,
			BIF_BX0_BIF_FB_EN__FB_READ_EN_MASK | BIF_BX0_BIF_FB_EN__FB_WRITE_EN_MASK);
	else
		WREG32_SOC15(NBIO, 0, regBIF_BX0_BIF_FB_EN, 0);
}

static void nbio_v6_3_2_init_doorbell_entry_registers(
	struct amdgpu_device *adev)
{
	struct amdgpu_nbio_doorbell_entries *entries;
	u32 ctrl_base, ctrl1_base;
	u32 i;

	memset(adev->nbio.doorbell_entries, 0,
	       sizeof(adev->nbio.doorbell_entries));
	adev->nbio.doorbell_layout = &nbio_v6_3_2_doorbell_layout;

	ctrl_base = SOC15_REG_OFFSET(
		NBIO, 0, regGDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL);
	ctrl1_base = SOC15_REG_OFFSET(
		NBIO, 0, regGDC_S2A0_S2A_DOORBELL_ENTRY_0_CTRL1);

	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_CP];
	entries->num_entries = 1;
	entries->entry[0].port = 0;
	entries->entry[0].ctrl_reg = ctrl_base;
	entries->entry[0].ctrl1_reg = ctrl1_base;

	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_IH];
	entries->num_entries = 1;
	entries->entry[0].port = 1;
	entries->entry[0].ctrl_reg = ctrl_base + 2;
	entries->entry[0].ctrl1_reg = ctrl1_base + 2;

	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_VCN];
	entries->num_entries = 4;
	for (i = 0; i < entries->num_entries; i++) {
		entries->entry[i].port = i + 2;
		entries->entry[i].ctrl_reg = ctrl_base + (i + 2) * 2;
		entries->entry[i].ctrl1_reg = ctrl1_base + (i + 2) * 2;
	}

	entries = &adev->nbio.doorbell_entries[
		AMDGPU_NBIO_DOORBELL_CLIENT_SDMA];
	entries->num_entries = 1;
	entries->entry[0].port = 6;
	entries->entry[0].ctrl_reg = ctrl_base + 6 * 2;
	entries->entry[0].ctrl1_reg = ctrl1_base + 6 * 2;
}

static void nbio_v6_3_2_init_registers(struct amdgpu_device *adev)
{
	nbio_v6_3_2_init_doorbell_entry_registers(adev);

	WREG32_SOC15(NBIO, 0, regXCD_DOORBELL_FENCE_1,
		(0xff & ~(adev->gfx.xcc_mask)) <<
		XCD_DOORBELL_FENCE_1__XCD_0_DOORBELL_DISABLE__SHIFT);
}

static u32 nbio_v6_3_2_get_memsize(struct amdgpu_device *adev)
{
	return RREG32_SOC15(NBIO, 0, regRCC_DEV0_EPF0_RCC_CONFIG_MEMSIZE);
}

static u32 nbio_v6_3_2_get_rom_offset(struct amdgpu_device *adev)
{
	u32 data, rom_offset;

	data = RREG32_SOC15(NBIO, 0, regREGS_ROM_OFFSET_CTRL);
	rom_offset = REG_GET_FIELD(data, REGS_ROM_OFFSET_CTRL, ROM_OFFSET);

	/* Temp W/A for nbio v6.3.2. Retrieving rom_offset as 1 but vbios
	 * image is actually located from offset 2
	 */
	rom_offset += 1;

	return rom_offset;
}

static void nbio_v6_3_2_enable_doorbell_aperture(struct amdgpu_device *adev,
						 bool enable)
{
	/* Enable to allow doorbell pass thru on pre-silicon bare-metal */
	WREG32_SOC15(NBIO, 0, regGDC0_DOORBELL_ACCESS_EN_PF, 0xfffff);
	WREG32_FIELD15_PREREG(NBIO, 0, RCC_DEV0_EPF0_RCC_DOORBELL_APER_EN,
			BIF_DOORBELL_APER_EN, enable ? 1 : 0);
}

static void nbio_v6_3_2_enable_doorbell_selfring_aperture(struct amdgpu_device *adev,
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

static void nbio_v6_3_2_enable_doorbell_interrupt(struct amdgpu_device *adev,
						  bool enable)
{
	WREG32_FIELD15_PREREG(NBIO, 0, BIF_BX0_BIF_DOORBELL_INT_CNTL,
			      DOORBELL_INTERRUPT_DISABLE, enable ? 0 : 1);
}

static void nbio_v6_3_2_ih_control(struct amdgpu_device *adev)
{
	u32 interrupt_cntl;

	/* setup interrupt control */
	WREG32_SOC15(NBIO, 0, regBIF_BX0_INTERRUPT_CNTL2, adev->dummy_page_addr >> 8);

	interrupt_cntl = RREG32_SOC15(NBIO, 0, regBIF_BX0_INTERRUPT_CNTL);
	/*
	 * BIF_BX0_INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK=0 - dummy read disabled with msi, enabled without msi
	 * BIF_BX0_INTERRUPT_CNTL__IH_DUMMY_RD_OVERRIDE_MASK=1 - dummy read controlled by IH_DUMMY_RD_EN
	 */
	interrupt_cntl = REG_SET_FIELD(interrupt_cntl, BIF_BX0_INTERRUPT_CNTL,
				       IH_DUMMY_RD_OVERRIDE, 0);

	/* BIF_BX0_INTERRUPT_CNTL__IH_REQ_NONSNOOP_EN_MASK=1 if ring is in non-cacheable memory, e.g., vram */
	interrupt_cntl = REG_SET_FIELD(interrupt_cntl, BIF_BX0_INTERRUPT_CNTL,
				       IH_REQ_NONSNOOP_EN, 0);

	WREG32_SOC15(NBIO, 0, regBIF_BX0_INTERRUPT_CNTL, interrupt_cntl);
}

static void nbio_v6_3_2_ih_doorbell_range(struct amdgpu_device *adev,
					  bool use_doorbell, int doorbell_index)
{
	const struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.range_offset = doorbell_index,
		.range_size = 8,
		.target_port_type = 0x3,
	};
	u32 ctrl, ctrl1;

	if (!use_doorbell) {
		amdgpu_nbio_program_doorbell_entry(adev,
			AMDGPU_NBIO_DOORBELL_CLIENT_IH, 0, NULL, NULL);
		return;
	}

	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
				adev, &fields, &ctrl, &ctrl1)))
		return;

	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_IH, 0, &ctrl, &ctrl1);
}

static void nbio_v6_3_2_gc_doorbell_init(struct amdgpu_device *adev)
{
	const struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.awid = 0x3,
		.awaddr_31_28 = 0x3,
		.target_port_type = 0x3,
	};
	u32 ctrl, ctrl1;

	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
				adev, &fields, &ctrl, &ctrl1)))
		return;

	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_CP, 0, &ctrl, &ctrl1);
}

static void nbio_v6_3_2_sdma_doorbell_range(struct amdgpu_device *adev,
					    int instance, bool use_doorbell,
					    int doorbell_index,
					    int doorbell_size)
{
	const struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.awid = 0xe,
		.range_offset = doorbell_index,
		.range_size = doorbell_size,
		.awaddr_31_28 = 0xe,
		.target_port_type = 0x3,
	};
	u32 ctrl, ctrl1;

	if (instance != 0)
		return;

	if (!use_doorbell) {
		amdgpu_nbio_program_doorbell_entry(adev,
			AMDGPU_NBIO_DOORBELL_CLIENT_SDMA, instance,
			NULL, NULL);
		return;
	}

	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
				adev, &fields, &ctrl, &ctrl1)))
		return;

	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_SDMA, instance, &ctrl, &ctrl1);
}

static void nbio_v6_3_2_vcn_doorbell_range(struct amdgpu_device *adev,
					   bool use_doorbell, int doorbell_index,
					   int instance)
{
	static const u8 vcn_component[] = { 0x4, 0x7, 0x4, 0x7 };
	static const u8 vcn_target_die[] = { 0, 0, 3, 3 };
	struct amdgpu_nbio_doorbell_fields fields = {
		.enable = true,
		.range_offset = doorbell_index,
		.range_size = 11,
		.target_port_type = 0x3,
	};
	u32 ctrl, ctrl1;

	if (WARN_ON_ONCE((unsigned int)instance >=
			 ARRAY_SIZE(vcn_component)))
		return;

	if (!use_doorbell) {
		amdgpu_nbio_program_doorbell_entry(adev,
			AMDGPU_NBIO_DOORBELL_CLIENT_VCN, instance,
			NULL, NULL);
		return;
	}

	fields.awid = vcn_component[instance];
	fields.awaddr_31_28 = fields.awid;
	fields.target_die_id = vcn_target_die[instance];
	if (WARN_ON_ONCE(amdgpu_nbio_build_doorbell_entry(
				adev, &fields, &ctrl, &ctrl1)))
		return;

	amdgpu_nbio_program_doorbell_entry(adev,
		AMDGPU_NBIO_DOORBELL_CLIENT_VCN, instance, &ctrl, &ctrl1);
}

static int nbio_v6_3_2_get_compute_partition_mode(struct amdgpu_device *adev)
{
	u32 tmp, px;

	tmp = RREG32_SOC15(NBIO, 0, regBIF_BX_PF0_PARTITION_COMPUTE_STATUS);
	px = REG_GET_FIELD(tmp, BIF_BX_PF0_PARTITION_COMPUTE_STATUS,
			   PARTITION_MODE);

	return px;
}

static bool nbio_v6_3_2_is_nps_switch_requested(struct amdgpu_device *adev)
{
	u32 tmp;

	tmp = RREG32_SOC15(NBIO, 0, regBIF_BX_PF0_PARTITION_MEM_STATUS);
	tmp = REG_GET_FIELD(tmp, BIF_BX_PF0_PARTITION_MEM_STATUS,
			    CHANGE_STATUS);

	/* 0x8 - NPS switch requested */
	return (tmp == 0x8);
}
static u32 nbio_v6_3_2_get_memory_partition_mode(struct amdgpu_device *adev,
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

const struct amdgpu_nbio_funcs nbio_v6_3_2_funcs = {
	.get_pcie_index_offset = nbio_v6_3_2_get_pcie_index_offset,
	.get_pcie_data_offset = nbio_v6_3_2_get_pcie_data_offset,
	.get_pcie_index_hi_offset = nbio_v6_3_2_get_pcie_index_hi_offset,
	.get_rev_id = nbio_v6_3_2_get_rev_id,
	.mc_access_enable = nbio_v6_3_2_mc_access_enable,
	.get_memsize = nbio_v6_3_2_get_memsize,
	.enable_doorbell_aperture = nbio_v6_3_2_enable_doorbell_aperture,
	.enable_doorbell_selfring_aperture = nbio_v6_3_2_enable_doorbell_selfring_aperture,
	.enable_doorbell_interrupt = nbio_v6_3_2_enable_doorbell_interrupt,
	.get_compute_partition_mode = nbio_v6_3_2_get_compute_partition_mode,
	.get_memory_partition_mode = nbio_v6_3_2_get_memory_partition_mode,
	.is_nps_switch_requested = nbio_v6_3_2_is_nps_switch_requested,
	.ih_control = nbio_v6_3_2_ih_control,
	.ih_doorbell_range = nbio_v6_3_2_ih_doorbell_range,
	.gc_doorbell_init = nbio_v6_3_2_gc_doorbell_init,
	.sdma_doorbell_range = nbio_v6_3_2_sdma_doorbell_range,
	.vcn_doorbell_range = nbio_v6_3_2_vcn_doorbell_range,
	.init_registers = nbio_v6_3_2_init_registers,
	.get_rom_offset = nbio_v6_3_2_get_rom_offset,
};
