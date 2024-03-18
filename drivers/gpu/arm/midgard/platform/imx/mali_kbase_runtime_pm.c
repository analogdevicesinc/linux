// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * COPYRIGHT 2015-2023 ARM Limited. All rights reserved.
 * COPYRIGHT 2023 - 2024 NXP
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
#include <mali_kbase_defs.h>
#include <device/mali_kbase_device.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>

#include "mali_kbase_config_platform.h"

static void enable_gpu_power_control(struct kbase_device *kbdev)
{
	unsigned int i;

#if defined(CONFIG_REGULATOR)
	for (i = 0; i < kbdev->nr_regulators; i++) {
		if (WARN_ON(kbdev->regulators[i] == NULL))
			;
		else if (!regulator_is_enabled(kbdev->regulators[i]))
			WARN_ON(regulator_enable(kbdev->regulators[i]));
	}
#endif

	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (WARN_ON(kbdev->clocks[i] == NULL))
			;
		else if (!__clk_is_enabled(kbdev->clocks[i]))
			WARN_ON(clk_prepare_enable(kbdev->clocks[i]));
	}
}

static void disable_gpu_power_control(struct kbase_device *kbdev)
{
	unsigned int i;

	for (i = 0; i < kbdev->nr_clocks; i++) {
		if (WARN_ON(kbdev->clocks[i] == NULL))
			;
		else if (__clk_is_enabled(kbdev->clocks[i])) {
			clk_disable_unprepare(kbdev->clocks[i]);
			WARN_ON(__clk_is_enabled(kbdev->clocks[i]));
		}
	}

#if defined(CONFIG_REGULATOR)
	for (i = 0; i < kbdev->nr_regulators; i++) {
		if (WARN_ON(kbdev->regulators[i] == NULL))
			;
		else if (regulator_is_enabled(kbdev->regulators[i]))
			WARN_ON(regulator_disable(kbdev->regulators[i]));
	}
#endif

}

static int pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = 1; /* Assume GPU has been powered off */
	int error;
	unsigned long flags;
	struct imx_platform_ctx *ictx = kbdev->platform_context;

	dev_dbg(kbdev->dev, "%s %pK\n", __func__, (void *)kbdev->dev->pm_domain);

	if (pm_runtime_enabled(kbdev->dev)) {
		error = pm_runtime_get_sync(kbdev->dev);
		dev_dbg(kbdev->dev, "power on pm_runtime_get_sync returned %d\n", error);
		if (ictx && (ictx->init_blk_ctrl == 0)
				&& !IS_ERR_OR_NULL(ictx->reg_blk_ctrl)) {
			ictx->init_blk_ctrl = 1;
			writel(0x1, ictx->reg_blk_ctrl + 0x8);
		}
		if (error == 1)
			ret = 0; //gpu still powered on.
	}

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	WARN_ON(kbdev->pm.backend.gpu_powered);
	if (likely(kbdev->csf.firmware_inited)) {
		WARN_ON(!kbdev->pm.active_count);
		WARN_ON(kbdev->pm.runtime_active);
	}
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	enable_gpu_power_control(kbdev);
	CSTD_UNUSED(error);

	return ret;
}

static void pm_callback_power_off(struct kbase_device *kbdev)
{
	unsigned long flags;
	struct imx_platform_ctx *ictx = kbdev->platform_context;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	WARN_ON(kbdev->pm.backend.gpu_powered);
	if (likely(kbdev->csf.firmware_inited)) {
#ifdef CONFIG_MALI_DEBUG
		WARN_ON(kbase_csf_scheduler_get_nr_active_csgs(kbdev));
#endif
		WARN_ON(kbdev->pm.backend.mcu_state != KBASE_MCU_OFF);
	}
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	/* Power down the GPU immediately */
	disable_gpu_power_control(kbdev);

	pm_runtime_mark_last_busy(kbdev->dev);
	pm_runtime_put_autosuspend(kbdev->dev);

	ictx->init_blk_ctrl = 0;
}

#ifdef KBASE_PM_RUNTIME
static int kbase_device_runtime_init(struct kbase_device *kbdev)
{
	int ret = 0;

	pm_runtime_set_autosuspend_delay(kbdev->dev, AUTO_SUSPEND_DELAY);
	pm_runtime_use_autosuspend(kbdev->dev);

	pm_runtime_set_active(kbdev->dev);
	pm_runtime_enable(kbdev->dev);

	if (!pm_runtime_enabled(kbdev->dev)) {
		dev_warn(kbdev->dev, "pm_runtime not enabled");
		ret = -EINVAL;
	} else if (atomic_read(&kbdev->dev->power.usage_count)) {
		dev_warn(kbdev->dev, "%s: Device runtime usage count unexpectedly non zero %d",
				__func__, atomic_read(&kbdev->dev->power.usage_count));
		ret = -EINVAL;
	}

	return ret;
}

static void kbase_device_runtime_disable(struct kbase_device *kbdev)
{

	pm_runtime_disable(kbdev->dev);
}
#endif /* KBASE_PM_RUNTIME */

static int pm_callback_runtime_on(struct kbase_device *kbdev)
{
	enable_gpu_power_control(kbdev);
	return 0;
}

static void pm_callback_runtime_off(struct kbase_device *kbdev)
{
	disable_gpu_power_control(kbdev);
}

static void pm_callback_resume(struct kbase_device *kbdev)
{
	int ret = 0;

	ret = pm_callback_runtime_on(kbdev);

	WARN_ON(ret);
}

static void pm_callback_suspend(struct kbase_device *kbdev)
{
	struct imx_platform_ctx *ictx = kbdev->platform_context;
	pm_callback_runtime_off(kbdev);
	ictx->init_blk_ctrl = 0;
}

struct kbase_pm_callback_conf pm_callbacks = {
	.power_on_callback = pm_callback_power_on,
	.power_off_callback = pm_callback_power_off,
	.power_suspend_callback = pm_callback_suspend,
	.power_resume_callback = pm_callback_resume,

	.power_runtime_init_callback = kbase_device_runtime_init,
	.power_runtime_term_callback = kbase_device_runtime_disable,
	.power_runtime_on_callback = pm_callback_runtime_on,
	.power_runtime_off_callback = pm_callback_runtime_off,
};
