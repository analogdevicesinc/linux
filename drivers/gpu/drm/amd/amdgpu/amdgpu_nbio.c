/*
 * Copyright (C) 2019  Advanced Micro Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN
 * AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "amdgpu.h"
#include "amdgpu_ras.h"
#include "soc15_common.h"

int amdgpu_nbio_ras_sw_init(struct amdgpu_device *adev)
{
	int err;
	struct amdgpu_nbio_ras *ras;

	if (!adev->nbio.ras)
		return 0;

	ras = adev->nbio.ras;
	err = amdgpu_ras_register_ras_block(adev, &ras->ras_block);
	if (err) {
		dev_err(adev->dev, "Failed to register pcie_bif ras block!\n");
		return err;
	}

	strcpy(ras->ras_block.ras_comm.name, "pcie_bif");
	ras->ras_block.ras_comm.block = AMDGPU_RAS_BLOCK__PCIE_BIF;
	ras->ras_block.ras_comm.type = AMDGPU_RAS_ERROR__MULTI_UNCORRECTABLE;
	adev->nbio.ras_if = &ras->ras_block.ras_comm;

	return 0;
}

u64 amdgpu_nbio_get_pcie_replay_count(struct amdgpu_device *adev)
{
	if (adev->nbio.funcs && adev->nbio.funcs->get_pcie_replay_count)
		return adev->nbio.funcs->get_pcie_replay_count(adev);

	return 0;
}

bool amdgpu_nbio_is_replay_cnt_supported(struct amdgpu_device *adev)
{
	if (amdgpu_sriov_vf(adev) || !adev->asic_funcs ||
	    !adev->asic_funcs->get_pcie_replay_count ||
	    (!adev->nbio.funcs || !adev->nbio.funcs->get_pcie_replay_count))
		return false;

	return true;
}

int amdgpu_nbio_ras_late_init(struct amdgpu_device *adev, struct ras_common_if *ras_block)
{
	int r;
	r = amdgpu_ras_block_late_init(adev, ras_block);
	if (r)
		return r;

	if (amdgpu_ras_is_supported(adev, ras_block->block)) {
		r = amdgpu_irq_get(adev, &adev->nbio.ras_controller_irq, 0);
		if (r)
			goto late_fini;
		r = amdgpu_irq_get(adev, &adev->nbio.ras_err_event_athub_irq, 0);
		if (r)
			goto late_fini;
	}

	return 0;
late_fini:
	amdgpu_ras_block_late_fini(adev, ras_block);
	return r;
}


void amdgpu_nbio_program_aspm(struct amdgpu_device *adev)
{
	if (!amdgpu_device_should_use_aspm(adev))
		return;

	if (adev->nbio.funcs->program_aspm)
		adev->nbio.funcs->program_aspm(adev);
}

static int amdgpu_nbio_set_doorbell_field(u32 *reg, u32 mask, u32 value)
{
	u32 shift;

	if (!mask)
		return value ? -EOPNOTSUPP : 0;

	shift = __ffs(mask);
	if (value > (mask >> shift))
		return -ERANGE;

	*reg |= (value << shift) & mask;
	return 0;
}

int amdgpu_nbio_build_doorbell_entry(
	struct amdgpu_device *adev,
	const struct amdgpu_nbio_doorbell_fields *fields,
	u32 *ctrl, u32 *ctrl1)
{
	const struct amdgpu_nbio_doorbell_layout *layout;
	int r;

	if (!fields || !ctrl)
		return -EINVAL;

	layout = adev->nbio.doorbell_layout;
	if (!layout)
		return -EINVAL;

	*ctrl = 0;
	if (ctrl1)
		*ctrl1 = 0;

	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.enable,
					   fields->enable);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.awid,
					   fields->awid);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.fence_enable,
					   fields->fence_enable);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.range_offset,
					   fields->range_offset);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.range_size,
					   fields->range_size);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.disable_64bit,
					   fields->disable_64bit);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl,
					   layout->ctrl.deduct_range_offset,
					   fields->deduct_range_offset);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.drop,
					   fields->drop);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl, layout->ctrl.awaddr_31_28,
					   fields->awaddr_31_28);
	if (r)
		return r;

	if (!ctrl1)
		return 0;
	if (!layout->has_ctrl1)
		return -EOPNOTSUPP;

	r = amdgpu_nbio_set_doorbell_field(ctrl1,
					   layout->ctrl1.target_port_type,
					   fields->target_port_type);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl1,
					   layout->ctrl1.target_die_id,
					   fields->target_die_id);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl1,
					   layout->ctrl1.target_port_id,
					   fields->target_port_id);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl1,
					   layout->ctrl1.override_poison_data,
					   fields->override_poison_data);
	if (r)
		return r;
	r = amdgpu_nbio_set_doorbell_field(ctrl1,
					   layout->ctrl1.disable_4k_page_decode,
					   fields->disable_4k_page_decode);
	if (r)
		return r;

	return 0;
}

void amdgpu_nbio_program_doorbell_entry(
	struct amdgpu_device *adev,
	enum amdgpu_nbio_doorbell_client client,
	u32 instance, const u32 *ctrl, const u32 *ctrl1)
{
	struct amdgpu_nbio_doorbell_entries *entries;
	struct amdgpu_nbio_doorbell_entry *entry;

	if (WARN_ON_ONCE(client >= AMDGPU_NBIO_DOORBELL_CLIENT_COUNT))
		return;

	entries = &adev->nbio.doorbell_entries[client];
	if (WARN_ON_ONCE(instance >= entries->num_entries))
		return;

	entry = &entries->entry[instance];
	if (WARN_ON_ONCE(!entry->ctrl_reg))
		return;

	if (entry->use_ext) {
		/* CTRL1 programming is not needed when use_ext is set. */
		if (WARN_ON_ONCE(entry->ctrl1_reg || ctrl1))
			return;

		WREG32_PCIE_EXT(
			(adev->reg_offset[NBIO_HWIP][entry->reg_inst]
			 [entry->ctrl_reg_base_idx] + entry->ctrl_reg) * 4 +
			amdgpu_reg_get_smn_base64(
				adev, NBIO_HWIP, entry->reg_inst),
			ctrl ? *ctrl : 0);
		return;
	}

	if (WARN_ON_ONCE(ctrl1 && !entry->ctrl1_reg))
		return;

	/* A NULL CTRL disables and clears the entry. */
	WREG32_SOC15_IP(NBIO, entry->ctrl_reg, ctrl ? *ctrl : 0);
	if (entry->ctrl1_reg && (!ctrl || ctrl1))
		WREG32_SOC15_IP(NBIO, entry->ctrl1_reg,
				    ctrl ? *ctrl1 : 0);
}
