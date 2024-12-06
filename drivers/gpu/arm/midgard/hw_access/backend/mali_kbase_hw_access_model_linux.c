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
#include <hw_access/mali_kbase_hw_access.h>
#include <hw_access/mali_kbase_hw_access_regmap_legacy.h>
#include <backend/gpu/mali_kbase_model_linux.h>

u64 kbase_reg_get_gpu_id(struct kbase_device *kbdev)
{
	unsigned long flags;
	u32 val[2] = { 0 };

	spin_lock_irqsave(&kbdev->reg_op_lock, flags);
	midgard_model_read_reg(kbdev->model, GPU_CONTROL_REG(GPU_ID), &val[0]);
	spin_unlock_irqrestore(&kbdev->reg_op_lock, flags);


	return (u64)val[0] | ((u64)val[1] << 32);
}

u32 kbase_reg_read32(struct kbase_device *kbdev, u32 reg_enum)
{
	unsigned long flags;
	u32 val = 0;
	u32 offset;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return 0;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_READ | KBASE_REGMAP_WIDTH_32_BIT)))
		return 0;

	offset = kbdev->regmap.regs[reg_enum] - kbdev->reg;

	spin_lock_irqsave(&kbdev->reg_op_lock, flags);
	midgard_model_read_reg(kbdev->model, offset, &val);
	spin_unlock_irqrestore(&kbdev->reg_op_lock, flags);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_reg_read32);

u64 kbase_reg_read64(struct kbase_device *kbdev, u32 reg_enum)
{
	unsigned long flags;
	u32 val32[2] = { 0 };
	u32 offset;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return 0;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_READ | KBASE_REGMAP_WIDTH_64_BIT)))
		return 0;

	offset = kbdev->regmap.regs[reg_enum] - kbdev->reg;

	spin_lock_irqsave(&kbdev->reg_op_lock, flags);
	midgard_model_read_reg(kbdev->model, offset, &val32[0]);
	midgard_model_read_reg(kbdev->model, offset + 4, &val32[1]);
	spin_unlock_irqrestore(&kbdev->reg_op_lock, flags);

	return (u64)val32[0] | ((u64)val32[1] << 32);
}
KBASE_EXPORT_TEST_API(kbase_reg_read64);

u64 kbase_reg_read64_coherent(struct kbase_device *kbdev, u32 reg_enum)
{
	unsigned long flags;
	u32 hi1 = 0, hi2 = 0, lo = 0;
	u32 offset;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return 0;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_READ | KBASE_REGMAP_WIDTH_64_BIT)))
		return 0;

	offset = kbdev->regmap.regs[reg_enum] - kbdev->reg;

	spin_lock_irqsave(&kbdev->reg_op_lock, flags);
	do {
		midgard_model_read_reg(kbdev->model, offset + 4, &hi1);
		midgard_model_read_reg(kbdev->model, offset, &lo);
		midgard_model_read_reg(kbdev->model, offset + 4, &hi2);
	} while (hi1 != hi2);
	spin_unlock_irqrestore(&kbdev->reg_op_lock, flags);

	return lo | (((u64)hi1) << 32);
}
KBASE_EXPORT_TEST_API(kbase_reg_read64_coherent);

void kbase_reg_write32(struct kbase_device *kbdev, u32 reg_enum, u32 value)
{
	unsigned long flags;
	u32 offset;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_WRITE | KBASE_REGMAP_WIDTH_32_BIT)))
		return;

	offset = kbdev->regmap.regs[reg_enum] - kbdev->reg;

	spin_lock_irqsave(&kbdev->reg_op_lock, flags);
	midgard_model_write_reg(kbdev->model, offset, value);
	spin_unlock_irqrestore(&kbdev->reg_op_lock, flags);
}
KBASE_EXPORT_TEST_API(kbase_reg_write32);

void kbase_reg_write64(struct kbase_device *kbdev, u32 reg_enum, u64 value)
{
	unsigned long flags;
	u32 offset;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_WRITE | KBASE_REGMAP_WIDTH_64_BIT)))
		return;

	offset = kbdev->regmap.regs[reg_enum] - kbdev->reg;

	spin_lock_irqsave(&kbdev->reg_op_lock, flags);
	midgard_model_write_reg(kbdev->model, offset, value & 0xFFFFFFFF);
	midgard_model_write_reg(kbdev->model, offset + 4, value >> 32);
	spin_unlock_irqrestore(&kbdev->reg_op_lock, flags);
}
KBASE_EXPORT_TEST_API(kbase_reg_write64);
