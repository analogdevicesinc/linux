/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * COPYRIGHT 2014-2017, 2020-2022 ARM Limited. All rights reserved.
 * COPYRIGHT 2023 NXP
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

/**
 * POWER_MANAGEMENT_CALLBACKS - Power management configuration
 *
 * Attached value: pointer to @ref kbase_pm_callback_conf
 * Default value: See @ref kbase_pm_callback_conf
 */
#define POWER_MANAGEMENT_CALLBACKS (&pm_callbacks)

/**
 * PLATFORM_FUNCS - Platform specific configuration functions
 *
 * Attached value: pointer to @ref kbase_platform_funcs_conf
 * Default value: See @ref kbase_platform_funcs_conf
 */
#define PLATFORM_FUNCS (&platform_funcs)

#define CLK_RATE_TRACE_OPS (&clk_rate_trace_ops)

extern struct kbase_pm_callback_conf pm_callbacks;
extern struct kbase_clk_rate_trace_op_conf clk_rate_trace_ops;
extern struct kbase_platform_funcs_conf platform_funcs;
/**
 * AUTO_SUSPEND_DELAY - Autosuspend delay
 *
 * The delay time (in milliseconds) to be used for autosuspend
 */
#define AUTO_SUSPEND_DELAY (100)

struct imx_platform_ctx {
	struct kbase_device *kbdev;
	void __iomem *reg_blk_ctrl;
	int init_blk_ctrl;
	void __iomem *reg_tcm;
	int dumpStarted;
};

int imx_waveform_start(struct kbase_device *kbdev);
int imx_waveform_stop(struct kbase_device *kbdev);
