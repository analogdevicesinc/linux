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

#include <linux/notifier.h>
#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <linux/clk.h>
#include "mali_kbase_config_platform.h"

static LIST_HEAD(gpu_clk_notifier_list);

/**
 * gpu_clk_rate_change_call_notifier - call clk notifier chain
 * @msg: clk notifier type (see include/linux/clk.h)
 * @v: clock notifier value
 *
 * Triggers a notifier call chain on the clk rate-change notification
 * for 'clk'.  Passes a pointer to the struct clk and the previous
 * and current rates to the notifier callback.  Intended to be called by
 * gpu driver only.  Returns NOTIFY_DONE from the last driver
 * called if all went well, or NOTIFY_STOP or NOTIFY_BAD immediately if
 * a driver returns that.
 */
static int gpu_clk_rate_change_call_notifiers(unsigned long msg, void *v)
{
	struct clk_notifier *cn;
	struct clk_notifier_data *n = v;
	int ret = NOTIFY_DONE;

	list_for_each_entry(cn, &gpu_clk_notifier_list, node) {
		if ((cn->clk == n->clk) && (n->old_rate != n->new_rate) ) {
			ret = srcu_notifier_call_chain(&cn->notifier_head, msg, n);
			if (ret & NOTIFY_STOP_MASK)
				return ret;
		}
	}

	return ret;
}

static void *enumerate_gpu_clk(struct kbase_device *kbdev, unsigned int index)
{
	if (index >= kbdev->nr_clocks)
		return NULL;

	return kbdev->clocks[index];
}

static unsigned long get_gpu_clk_rate(struct kbase_device *kbdev, void *gpu_clk_handle)
{
	CSTD_UNUSED(kbdev);

	return clk_get_rate((struct clk *)gpu_clk_handle);
}

static int gpu_clk_notifier_register(struct kbase_device *kbdev, void *gpu_clk_handle,
		struct notifier_block *nb)
{
	struct clk_notifier *cn;

	compiletime_assert(offsetof(struct clk_notifier_data, clk) ==
			offsetof(struct kbase_gpu_clk_notifier_data, gpu_clk_handle),
			"mismatch in the offset of clk member");

	compiletime_assert(
			sizeof(((struct clk_notifier_data *)0)->clk) ==
			sizeof(((struct kbase_gpu_clk_notifier_data *)0)->gpu_clk_handle),
			"mismatch in the size of clk member");

	if (kbdev->dev_gpuperf == NULL)
		return clk_notifier_register((struct clk *)gpu_clk_handle, nb);

	/* search the list of notifiers for this clk */
	list_for_each_entry(cn, &gpu_clk_notifier_list, node)
		if (cn->clk == gpu_clk_handle)
			goto found;

	/* if clk wasn't in the notifier list, allocate new clk_notifier */
	cn = kzalloc(sizeof(*cn), GFP_KERNEL);
	if (!cn)
		return -ENOMEM;

	cn->clk = gpu_clk_handle;
	srcu_init_notifier_head(&cn->notifier_head);

	list_add(&cn->node, &gpu_clk_notifier_list);

found:
	return srcu_notifier_chain_register(&cn->notifier_head, nb);
}

static void gpu_clk_notifier_unregister(struct kbase_device *kbdev, void *gpu_clk_handle,
		struct notifier_block *nb)
{
	struct clk_notifier *cn;

	if (kbdev->dev_gpuperf == NULL)
		clk_notifier_unregister((struct clk *)gpu_clk_handle, nb);

	list_for_each_entry(cn, &gpu_clk_notifier_list, node) {
		if (cn->clk == gpu_clk_handle) {
			srcu_notifier_chain_unregister(&cn->notifier_head, nb);

			/* XXX the notifier code should handle this better */
			if (!cn->notifier_head.head) {
				srcu_cleanup_notifier_head(&cn->notifier_head);
				list_del(&cn->node);
				kfree(cn);
			}
			break;
		}
	}
}

struct kbase_clk_rate_trace_op_conf clk_rate_trace_ops = {
	.get_gpu_clk_rate = get_gpu_clk_rate,
	.enumerate_gpu_clk = enumerate_gpu_clk,
	.gpu_clk_notifier_register = gpu_clk_notifier_register,
	.gpu_clk_notifier_unregister = gpu_clk_notifier_unregister,
	.clk_change_notifier =  gpu_clk_rate_change_call_notifiers,
};
