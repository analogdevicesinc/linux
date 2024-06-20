// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2023-2024 ARM Limited. All rights reserved.
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

#include <linux/io.h>
#include <linux/types.h>

#include <mali_kbase.h>
#include "mali_kbase_hw_access.h"
#include "mali_kbase_hw_access_regmap.h"

#include <uapi/gpu/arm/midgard/gpu/mali_kbase_gpu_id.h>

#define KBASE_REGMAP_ACCESS_ALWAYS_POWERED (1U << 16)

static u32 always_powered_regs[] = {

#if !MALI_USE_CSF
	PTM_AW_IRQ_CLEAR,
	PTM_AW_IRQ_INJECTION,
	PTM_AW_IRQ_MASK,
	PTM_AW_IRQ_RAWSTAT,
	PTM_AW_IRQ_STATUS,
	PTM_AW_MESSAGE__PTM_INCOMING_MESSAGE0,
	PTM_AW_MESSAGE__PTM_INCOMING_MESSAGE1,
	PTM_AW_MESSAGE__PTM_OUTGOING_MESSAGE0,
	PTM_AW_MESSAGE__PTM_OUTGOING_MESSAGE1,
	PTM_AW_MESSAGE__PTM_OUTGOING_MESSAGE_STATUS,
	PTM_ID,
#endif /* !MALI_USE_CSF */
};

static void kbasep_reg_setup_always_powered_registers(struct kbase_device *kbdev)
{
	u32 i;


#if !MALI_USE_CSF
	if (kbdev->gpu_props.gpu_id.arch_id < GPU_ID_ARCH_MAKE(9, 14, 0))
		return;
#endif /* MALI_USE_CSF */

	for (i = 0; i < ARRAY_SIZE(always_powered_regs); i++) {
		u32 reg_enum = always_powered_regs[i];

		if (!kbase_reg_is_valid(kbdev, reg_enum))
			continue;

		kbdev->regmap.flags[reg_enum] |= KBASE_REGMAP_ACCESS_ALWAYS_POWERED;
	}
}

bool kbase_reg_is_powered_access_allowed(struct kbase_device *kbdev, u32 reg_enum)
{
	if (kbdev->regmap.flags[reg_enum] & KBASE_REGMAP_ACCESS_ALWAYS_POWERED)
		return true;
	return kbdev->pm.backend.gpu_powered;
}

bool kbase_reg_is_size64(struct kbase_device *kbdev, u32 reg_enum)
{
	if (WARN_ON(reg_enum >= kbdev->regmap.size))
		return false;

	return kbdev->regmap.flags[reg_enum] & KBASE_REGMAP_WIDTH_64_BIT;
}

bool kbase_reg_is_size32(struct kbase_device *kbdev, u32 reg_enum)
{
	if (WARN_ON(reg_enum >= kbdev->regmap.size))
		return false;

	return kbdev->regmap.flags[reg_enum] & KBASE_REGMAP_WIDTH_32_BIT;
}

bool kbase_reg_is_valid(struct kbase_device *kbdev, u32 reg_enum)
{
	return reg_enum < kbdev->regmap.size && kbdev->regmap.flags[reg_enum] != 0;
}

bool kbase_reg_is_accessible(struct kbase_device *kbdev, u32 reg_enum, u32 flags)
{
#ifdef CONFIG_MALI_DEBUG
	if (WARN(!kbase_reg_is_valid(kbdev, reg_enum), "Invalid register enum 0x%x: %s", reg_enum,
		 kbase_reg_get_enum_string(reg_enum)))
		return false;
	if (WARN((kbdev->regmap.flags[reg_enum] & flags) != flags,
		 "Invalid register access permissions 0x%x: %s", reg_enum,
		 kbase_reg_get_enum_string(reg_enum)))
		return false;
#else
	CSTD_UNUSED(kbdev);
	CSTD_UNUSED(reg_enum);
	CSTD_UNUSED(flags);
#endif

	return true;
}

bool kbase_reg_is_init(struct kbase_device *kbdev)
{
	return (kbdev->regmap.regs != NULL) && (kbdev->regmap.flags != NULL);
}

int kbase_reg_get_offset(struct kbase_device *kbdev, u32 reg_enum, u32 *offset)
{
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum, 0)))
		return -EINVAL;

	*offset = kbdev->regmap.regs[reg_enum] - kbdev->reg;
	return 0;
}

int kbase_reg_get_enum(struct kbase_device *kbdev, u32 offset, u32 *reg_enum)
{
	size_t i = 0;
	void __iomem *ptr = kbdev->reg + offset;

	for (i = 0; i < kbdev->regmap.size; i++) {
		if (kbdev->regmap.regs[i] == ptr) {
			*reg_enum = (u32)i;
			return 0;
		}
	}

	return -EINVAL;
}

int kbase_regmap_init(struct kbase_device *kbdev)
{
	u32 lut_arch_id;

	if (WARN_ON(kbdev->dev == NULL))
		return -ENODEV;

	if (!IS_ENABLED(CONFIG_MALI_NO_MALI) && WARN_ON(kbdev->reg == NULL))
		return -ENXIO;

	lut_arch_id = kbase_regmap_backend_init(kbdev);

	if (kbdev->regmap.regs == NULL || kbdev->regmap.flags == NULL) {
		kbase_regmap_term(kbdev);
		return -ENOMEM;
	}

	kbasep_reg_setup_always_powered_registers(kbdev);

	dev_info(kbdev->dev, "Register LUT %08x initialized for GPU arch 0x%08x\n", lut_arch_id,
		 kbdev->gpu_props.gpu_id.arch_id);


	return 0;
}

void kbase_regmap_term(struct kbase_device *kbdev)
{
	kfree(kbdev->regmap.regs);
	kfree(kbdev->regmap.flags);

	kbdev->regmap.regs = NULL;
	kbdev->regmap.flags = NULL;
	kbdev->regmap.size = 0;
}
