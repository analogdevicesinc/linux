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

#ifndef _KBASE_CSF_UTIL_H_
#define _KBASE_CSF_UTIL_H_

/* Forward declaration */
struct kbase_device;
struct kbasep_printer;
struct seq_file;

/**
 * enum kbasep_printer_type - Enumeration representing the different printing output types
 *
 * @KBASEP_PRINT_TYPE_INVALID:  Invalid printing output (default).
 * @KBASEP_PRINT_TYPE_DEV_INFO: Print to dmesg at info level.
 * @KBASEP_PRINT_TYPE_DEV_WARN: Print to dmesg at warning level.
 * @KBASEP_PRINT_TYPE_DEV_ERR:  Print to dmesg at error level.
 * @KBASEP_PRINT_TYPE_SEQ_FILE: Print to file.
 * @KBASEP_PRINT_TYPE_CNT:      Never set explicitly.
 */
enum kbasep_printer_type {
	KBASEP_PRINT_TYPE_INVALID = 0,
	KBASEP_PRINT_TYPE_DEV_INFO,
	KBASEP_PRINT_TYPE_DEV_WARN,
	KBASEP_PRINT_TYPE_DEV_ERR,
	KBASEP_PRINT_TYPE_SEQ_FILE,
	KBASEP_PRINT_TYPE_CNT,
};

/**
 * kbasep_printer_buffer_init() - Helper function to initialise a printer to a buffer.
 *
 * @kbdev: Pointer to the device.
 * @type:  Printing output type. Only the following types are supported:
 *         @KBASEP_PRINT_TYPE_DEV_INFO, @KBASEP_PRINT_TYPE_DEV_WARN and
 *         @KBASEP_PRINT_TYPE_DEV_ERR.
 *
 * Return: The kbasep_printer instance pointer or NULL on error.
 */
struct kbasep_printer *kbasep_printer_buffer_init(struct kbase_device *kbdev,
						  enum kbasep_printer_type type);
/**
 * kbasep_printer_file_init() - Helper function to initialise a printer to a file.
 *
 * @file: The seq_file for printing to.
 *
 * Return: The kbasep_printer instance pointer or NULL on error.
 */
struct kbasep_printer *kbasep_printer_file_init(struct seq_file *file);

/**
 * kbasep_printer_term() - Helper function to terminate printer.
 *
 * @kbpr: The print output device.
 */
void kbasep_printer_term(struct kbasep_printer *kbpr);

/**
 * kbasep_printer_buffer_flush() - Helper function to flush printer buffer to dmesg.
 *
 * @kbpr: The print output device.
 */
void kbasep_printer_buffer_flush(struct kbasep_printer *kbpr);

/**
 * kbasep_puts() - Print string using kbasep_printer instance.
 *
 * @kbpr: The kbasep_printer instance.
 * @str: The string to print.
 */
void kbasep_puts(struct kbasep_printer *kbpr, const char *str);

/**
 * kbasep_print() - Helper function to print to either debugfs file or a dmesg buffer.
 *
 * @kbpr: The print output device.
 * @fmt:  The message to print.
 * @...:  Arguments to format the message.
 */
void kbasep_print(struct kbasep_printer *kbpr, const char *fmt, ...);

#endif /* _KBASE_CSF_UTIL_H_ */
