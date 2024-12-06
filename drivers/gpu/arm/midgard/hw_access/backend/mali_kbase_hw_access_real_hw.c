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
#include <linux/mali_hw_access.h>
#include <mali_kbase_io.h>

u64 kbase_reg_get_gpu_id(struct kbase_device *kbdev)
{
	u32 val[2] = { 0 };

	val[0] = mali_readl(kbdev->reg);


	return (u64)val[0] | ((u64)val[1] << 32);
}

u32 kbase_reg_read32(struct kbase_device *kbdev, u32 reg_enum)
{
	u32 val;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return 0;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_READ | KBASE_REGMAP_WIDTH_32_BIT)))
		return 0;

	val = mali_readl(kbdev->regmap.regs[reg_enum]);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(kbdev->io_history.enabled))
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum], val, 0);
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "r32: reg %08x val %08x",
		(u32)(kbdev->regmap.regs[reg_enum] - kbdev->reg), val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_reg_read32);

u64 kbase_reg_read64(struct kbase_device *kbdev, u32 reg_enum)
{
	u64 val;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return 0;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_READ | KBASE_REGMAP_WIDTH_64_BIT)))
		return 0;

	val = mali_readq(kbdev->regmap.regs[reg_enum]);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(kbdev->io_history.enabled)) {
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum], (u32)val, 0);
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum] + 4,
				     (u32)(val >> 32), 0);
	}
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "r64: reg %08x val %016llx",
		(u32)(kbdev->regmap.regs[reg_enum] - kbdev->reg), val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_reg_read64);

u64 kbase_reg_read64_coherent(struct kbase_device *kbdev, u32 reg_enum)
{
	u64 val;

	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return 0;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_READ | KBASE_REGMAP_WIDTH_64_BIT)))
		return 0;

	val = mali_readq_coherent(kbdev->regmap.regs[reg_enum]);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(kbdev->io_history.enabled)) {
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum], (u32)val, 0);
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum] + 4,
				     (u32)(val >> 32), 0);
	}
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "r64: reg %08x val %016llx",
		(u32)(kbdev->regmap.regs[reg_enum] - kbdev->reg), val);

	return val;
}
KBASE_EXPORT_TEST_API(kbase_reg_read64_coherent);

void kbase_reg_write32(struct kbase_device *kbdev, u32 reg_enum, u32 value)
{
	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_WRITE | KBASE_REGMAP_WIDTH_32_BIT)))
		return;

	mali_writel(value, kbdev->regmap.regs[reg_enum]);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(kbdev->io_history.enabled))
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum], value, 1);
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "w32: reg %08x val %08x",
		(u32)(kbdev->regmap.regs[reg_enum] - kbdev->reg), value);
}
KBASE_EXPORT_TEST_API(kbase_reg_write32);

void kbase_reg_write64(struct kbase_device *kbdev, u32 reg_enum, u64 value)
{
	if (WARN_ON(!kbase_reg_is_powered_access_allowed(kbdev, reg_enum)))
		return;
	if (unlikely(!kbase_reg_is_accessible(kbdev, reg_enum,
					      KBASE_REGMAP_PERM_WRITE | KBASE_REGMAP_WIDTH_64_BIT)))
		return;

	mali_writeq(value, kbdev->regmap.regs[reg_enum]);

#if IS_ENABLED(CONFIG_DEBUG_FS)
	if (unlikely(kbdev->io_history.enabled)) {
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum], (u32)value,
				     1);
		kbase_io_history_add(&kbdev->io_history, kbdev->regmap.regs[reg_enum] + 4,
				     (u32)(value >> 32), 1);
	}
#endif /* CONFIG_DEBUG_FS */

	dev_dbg(kbdev->dev, "w64: reg %08x val %016llx",
		(u32)(kbdev->regmap.regs[reg_enum] - kbdev->reg), value);
}
KBASE_EXPORT_TEST_API(kbase_reg_write64);
