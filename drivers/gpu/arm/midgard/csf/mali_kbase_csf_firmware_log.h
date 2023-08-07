/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2022 ARM Limited. All rights reserved.
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

#ifndef _KBASE_CSF_FIRMWARE_LOG_H_
#define _KBASE_CSF_FIRMWARE_LOG_H_

#include <mali_kbase.h>

/*
 * Firmware log dumping buffer size.
 */
#define FIRMWARE_LOG_DUMP_BUF_SIZE PAGE_SIZE

/**
 * kbase_csf_firmware_log_init - Initialize firmware log handling.
 *
 * @kbdev: Pointer to the Kbase device
 *
 * Return: The initialization error code.
 */
int kbase_csf_firmware_log_init(struct kbase_device *kbdev);

/**
 * kbase_csf_firmware_log_term - Terminate firmware log handling.
 *
 * @kbdev: Pointer to the Kbase device
 */
void kbase_csf_firmware_log_term(struct kbase_device *kbdev);

/**
 * kbase_csf_firmware_log_dump_buffer - Read remaining data in the firmware log
 *                                  buffer and print it to dmesg.
 *
 * @kbdev: Pointer to the Kbase device
 */
void kbase_csf_firmware_log_dump_buffer(struct kbase_device *kbdev);

#endif /* _KBASE_CSF_FIRMWARE_LOG_H_ */
