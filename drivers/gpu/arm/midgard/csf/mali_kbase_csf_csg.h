/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023 ARM Limited. All rights reserved.
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

#ifndef _KBASE_CSF_CSG_H_
#define _KBASE_CSF_CSG_H_

/* Forward declaration */
struct kbase_context;
struct kbase_device;
struct kbasep_printer;

#define MALI_CSF_CSG_DUMP_VERSION 0

/**
 * kbase_csf_csg_update_status() - Update on-slot gpu group statuses
 *
 * @kbdev: Pointer to the device.
 */
void kbase_csf_csg_update_status(struct kbase_device *kbdev);

/**
 * kbasep_csf_csg_dump_print() - Dump all gpu groups information to file
 *
 * @kctx: The kbase_context which gpu group dumped belongs to.
 * @kbpr: Pointer to printer instance.
 *
 * Return: Return 0 for dump successfully, or error code.
 */
int kbasep_csf_csg_dump_print(struct kbase_context *const kctx, struct kbasep_printer *kbpr);

/**
 * kbasep_csf_csg_active_dump_print() - Dump on-slot gpu groups information to file
 *
 * @kbdev: Pointer to the device.
 * @kbpr: Pointer to printer instance.
 *
 * Return: Return 0 for dump successfully, or error code.
 */
int kbasep_csf_csg_active_dump_print(struct kbase_device *kbdev, struct kbasep_printer *kbpr);

#endif /* _KBASE_CSF_CSG_H_ */
