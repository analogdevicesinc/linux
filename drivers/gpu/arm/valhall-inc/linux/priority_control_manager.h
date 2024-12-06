/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2020-2023 ARM Limited. All rights reserved.
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

#ifndef _PRIORITY_CONTROL_MANAGER_H_
#define _PRIORITY_CONTROL_MANAGER_H_

#include <linux/mm.h>
#include <linux/of.h>
#include <linux/version.h>

struct priority_control_manager_device;

/**
 * DOC: PCM notifier callback types
 *
 * ADD_PRIORITIZED_PROCESS - indicate that work items for this process should be
 *                           given priority over the work items from other
 *                           processes that were assigned the same static
 *                           priority level. Processes that would benefit from
 *                           being added to this list includes foreground
 *                           applications, as well as any other latency-sensitive
 *                           applications.
 *
 * REMOVE_PRIORITIZED_PROCESS - indicate that work items for this process
 *                              should no longer be prioritized over other work
 *                              items given the same static priority level.
 */
#define ADD_PRIORITIZED_PROCESS 0
#define REMOVE_PRIORITIZED_PROCESS 1

/**
 * struct pcm_prioritized_process_notifier_data - change of prioritized process
 *                                                list passed to the callback
 *
 * @pid: PID of the process being added/removed
 */
struct pcm_prioritized_process_notifier_data {
	uint32_t pid;
};

/**
 * struct priority_control_manager_ops - Callbacks for priority control manager operations
 *
 * @pcm_scheduler_priority_check: Callback to check if scheduling priority level can be requested
 *                                pcm_dev: The priority control manager through which the
 *                                         request is being made.
 *                                task: The task struct of the process requesting the
 *                                      priority check.
 *                                requested_priority: The priority level being requested.
 *
 *                                The returned value will be:
 *                                The same as requested_priority if the process has permission to
 *                                use requested_priority.A lower priority value if the process does
 *                                not have permission to use requested_priority
 *
 *                                requested_priority has the following value range:
 *                                0-3 : Priority level, 0 being highest and 3 being lowest
 *
 *                                Return: The priority that would actually be given, could be lower
 *                                than requested_priority
 *
 * @pcm_prioritized_process_notifier_register: register a callback for changes to the
 *                                             list of prioritized processes
 *                                             pcm_dev: The priority control manager through
 *                                                      which the request is being made.
 *                                             nb: notifier block with callback function pointer
 *                                             On Success returns 0 otherwise -1
 *
 * @pcm_prioritized_process_notifier_unregister: unregister the callback for changes to the
 *                                               list of prioritized processes
 *                                               pcm_dev: The priority control manager through
 *                                                        which the request is being made.
 *                                               nb: notifier block which will be unregistered
 *                                               On Success returns 0 otherwise -1
 */
struct priority_control_manager_ops {
	int (*pcm_scheduler_priority_check)(struct priority_control_manager_device *pcm_dev,
					    struct task_struct *task, int requested_priority);

	int (*pcm_prioritized_process_notifier_register)(
		struct priority_control_manager_device *pcm_dev, struct notifier_block *nb);

	int (*pcm_prioritized_process_notifier_unregister)(
		struct priority_control_manager_device *pcm_dev, struct notifier_block *nb);
};

/**
 * struct priority_control_manager_device - Device structure for priority
 *                                          control manager
 *
 * @ops:   Callbacks associated with this device
 * @data:  Pointer to device private data
 * @owner: Pointer to the module owner
 *
 * This structure should be registered with the platform device using
 * platform_set_drvdata().
 */
struct priority_control_manager_device {
	struct priority_control_manager_ops ops;
	void *data;
	struct module *owner;
};

#endif /* _PRIORITY_CONTROL_MANAGER_H_ */
