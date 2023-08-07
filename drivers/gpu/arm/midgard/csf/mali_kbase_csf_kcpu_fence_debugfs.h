/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2018-2023 ARM Limited. All rights reserved.
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
#ifndef _KBASE_CSF_KCPU_FENCE_SIGNAL_DEBUGFS_H_
#define _KBASE_CSF_KCPU_FENCE_SIGNAL_DEBUGFS_H_

struct kbase_device;

/*
 * kbase_csf_fence_timer_debugfs_init - Initialize fence signal timeout debugfs
 *                                      entries.
 * @kbdev: Kbase device.
 *
 * Return: 0 on success, -1 on failure.
 */
int kbase_csf_fence_timer_debugfs_init(struct kbase_device *kbdev);

/*
 * kbase_csf_fence_timer_debugfs_term - Terminate fence signal timeout debugfs
 *                                      entries.
 * @kbdev: Kbase device.
 */
void kbase_csf_fence_timer_debugfs_term(struct kbase_device *kbdev);

#endif /* _KBASE_CSF_KCPU_FENCE_SIGNAL_DEBUGFS_H_ */
