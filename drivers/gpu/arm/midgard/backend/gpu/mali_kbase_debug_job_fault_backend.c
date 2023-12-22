// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2012-2023 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <mali_kbase.h>
#include <device/mali_kbase_device.h>
#include <hw_access/mali_kbase_hw_access.h>
#include "mali_kbase_debug_job_fault.h"

#if IS_ENABLED(CONFIG_DEBUG_FS)

/*GPU_CONTROL_REG(r)*/
static unsigned int gpu_control_reg_snapshot[] = { GPU_CONTROL_ENUM(GPU_ID),
						   GPU_CONTROL_ENUM(SHADER_READY),
						   GPU_CONTROL_ENUM(TILER_READY),
						   GPU_CONTROL_ENUM(L2_READY) };

/* JOB_CONTROL_REG(r) */
static unsigned int job_control_reg_snapshot[] = { JOB_CONTROL_ENUM(JOB_IRQ_MASK),
						   JOB_CONTROL_ENUM(JOB_IRQ_STATUS) };

/* JOB_SLOT_REG(n,r) */
static unsigned int job_slot_reg_snapshot[] = {
	JOB_SLOT_ENUM(0, HEAD) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, TAIL) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, AFFINITY) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, CONFIG) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, STATUS) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, HEAD_NEXT) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, AFFINITY_NEXT) - JOB_SLOT_BASE_ENUM(0),
	JOB_SLOT_ENUM(0, CONFIG_NEXT) - JOB_SLOT_BASE_ENUM(0)
};

/*MMU_CONTROL_REG(r)*/
static unsigned int mmu_reg_snapshot[] = { MMU_CONTROL_ENUM(IRQ_MASK),
					   MMU_CONTROL_ENUM(IRQ_STATUS) };

/* MMU_AS_REG(n,r) */
static unsigned int as_reg_snapshot[] = { MMU_AS_ENUM(0, TRANSTAB) - MMU_AS_BASE_ENUM(0),
					  MMU_AS_ENUM(0, TRANSCFG) - MMU_AS_BASE_ENUM(0),
					  MMU_AS_ENUM(0, MEMATTR) - MMU_AS_BASE_ENUM(0),
					  MMU_AS_ENUM(0, FAULTSTATUS) - MMU_AS_BASE_ENUM(0),
					  MMU_AS_ENUM(0, FAULTADDRESS) - MMU_AS_BASE_ENUM(0),
					  MMU_AS_ENUM(0, STATUS) - MMU_AS_BASE_ENUM(0) };

bool kbase_debug_job_fault_reg_snapshot_init(struct kbase_context *kctx, int reg_range)
{
	uint i, j;
	int offset = 0;
	uint slot_number;
	uint as_number;

	if (kctx->reg_dump == NULL)
		return false;

	slot_number = kctx->kbdev->gpu_props.num_job_slots;
	as_number = kctx->kbdev->gpu_props.num_address_spaces;

	/* get the GPU control registers*/
	for (i = 0; i < ARRAY_SIZE(gpu_control_reg_snapshot); i++) {
		kctx->reg_dump[offset] = gpu_control_reg_snapshot[i];
		if (kbase_reg_is_size64(kctx->kbdev, kctx->reg_dump[offset]))
			offset += 4;
		else
			offset += 2;
	}

	/* get the Job control registers*/
	for (i = 0; i < ARRAY_SIZE(job_control_reg_snapshot); i++) {
		kctx->reg_dump[offset] = job_control_reg_snapshot[i];
		if (kbase_reg_is_size64(kctx->kbdev, kctx->reg_dump[offset]))
			offset += 4;
		else
			offset += 2;
	}

	/* get the Job Slot registers*/
	for (j = 0; j < slot_number; j++) {
		for (i = 0; i < ARRAY_SIZE(job_slot_reg_snapshot); i++) {
			kctx->reg_dump[offset] = JOB_SLOT_BASE_OFFSET(j) + job_slot_reg_snapshot[i];
			if (kbase_reg_is_size64(kctx->kbdev, kctx->reg_dump[offset]))
				offset += 4;
			else
				offset += 2;
		}
	}

	/* get the MMU registers*/
	for (i = 0; i < ARRAY_SIZE(mmu_reg_snapshot); i++) {
		kctx->reg_dump[offset] = mmu_reg_snapshot[i];
		if (kbase_reg_is_size64(kctx->kbdev, kctx->reg_dump[offset]))
			offset += 4;
		else
			offset += 2;
	}

	/* get the Address space registers*/
	for (j = 0; j < as_number; j++) {
		for (i = 0; i < ARRAY_SIZE(as_reg_snapshot); i++) {
			kctx->reg_dump[offset] = MMU_AS_BASE_OFFSET(j) + as_reg_snapshot[i];
			if (kbase_reg_is_size64(kctx->kbdev, kctx->reg_dump[offset]))
				offset += 4;
			else
				offset += 2;
		}
	}

	WARN_ON(offset >= (reg_range * 2 / 4));

	/* set the termination flag*/
	kctx->reg_dump[offset] = REGISTER_DUMP_TERMINATION_FLAG;
	kctx->reg_dump[offset + 1] = REGISTER_DUMP_TERMINATION_FLAG;

	dev_dbg(kctx->kbdev->dev, "kbase_job_fault_reg_snapshot_init:%d\n", offset);

	return true;
}

bool kbase_job_fault_get_reg_snapshot(struct kbase_context *kctx)
{
	int offset = 0;
	u32 reg_enum;
	u64 val64;

	if (kctx->reg_dump == NULL)
		return false;

	while (kctx->reg_dump[offset] != REGISTER_DUMP_TERMINATION_FLAG) {
		reg_enum = kctx->reg_dump[offset];
		/* Get register offset from enum */
		kbase_reg_get_offset(kctx->kbdev, reg_enum, &kctx->reg_dump[offset]);

		if (kbase_reg_is_size64(kctx->kbdev, reg_enum)) {
			val64 = kbase_reg_read64(kctx->kbdev, reg_enum);

			/* offset computed offset to get _HI offset */
			kctx->reg_dump[offset + 2] = kctx->reg_dump[offset] + 4;

			kctx->reg_dump[offset + 1] = (u32)(val64 & 0xFFFFFFFF);
			kctx->reg_dump[offset + 3] = (u32)(val64 >> 32);
			offset += 4;
		} else {
			kctx->reg_dump[offset + 1] = kbase_reg_read32(kctx->kbdev, reg_enum);
			offset += 2;
		}
	}
	return true;
}

#endif
