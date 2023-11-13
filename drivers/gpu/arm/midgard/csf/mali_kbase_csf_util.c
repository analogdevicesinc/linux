// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
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

#include "mali_kbase_csf_util.h"
#include <mali_kbase.h>
#include <linux/kfifo.h>
#include <linux/printk.h>
#include <linux/seq_file.h>
#include <linux/version_compat_defs.h>
#include <linux/vmalloc.h>

#define KBASEP_PRINTER_BUFFER_MAX_SIZE (2 * PAGE_SIZE)

#define KBASEP_PRINT_FORMAT_BUFFER_MAX_SIZE 256

/**
 * struct kbasep_printer - Object representing a logical printer device.
 *
 * @type:  The print output type.
 * @fifo:  address of the fifo to be used.
 * @kbdev: The kbase device.
 * @file:  The seq_file for printing to. This is NULL if printing to dmesg.
 */
struct kbasep_printer {
	enum kbasep_printer_type type;
	struct kbase_device *kbdev;
	struct seq_file *file;
	DECLARE_KFIFO_PTR(fifo, char);
};

/**
 * kbasep_printer_alloc() - Allocate a kbasep_printer instance.
 *
 * @type: The type of kbasep_printer to be allocated.
 *
 * Return: The kbasep_printer instance pointer or NULL on error.
 */
static inline struct kbasep_printer *kbasep_printer_alloc(enum kbasep_printer_type type)
{
	struct kbasep_printer *kbpr = NULL;

	if (type == KBASEP_PRINT_TYPE_INVALID || type >= KBASEP_PRINT_TYPE_CNT) {
		pr_err("printer type not supported");
		return NULL;
	}

	kbpr = vzalloc(sizeof(struct kbasep_printer));
	if (kbpr) {
		kbpr->type = type;
		kbpr->file = NULL;
	}

	return kbpr;
}

/**
 * kbasep_printer_validate() - Validate kbasep_printer instance.
 *
 * @kbpr: The kbasep_printer instance to be validated.
 *
 * Return: true if the instance is correctly configured else false.
 */
static inline bool kbasep_printer_validate(const struct kbasep_printer *kbpr)
{
	if (!kbpr || kbpr->type == KBASEP_PRINT_TYPE_INVALID || kbpr->type >= KBASEP_PRINT_TYPE_CNT)
		return false;

	switch (kbpr->type) {
	case KBASEP_PRINT_TYPE_DEV_INFO:
	case KBASEP_PRINT_TYPE_DEV_WARN:
	case KBASEP_PRINT_TYPE_DEV_ERR:
		if (kbpr->kbdev == NULL || !kfifo_initialized(&kbpr->fifo))
			return false;
		break;
	case KBASEP_PRINT_TYPE_SEQ_FILE:
		if (kbpr->file == NULL)
			return false;
		break;
	default:
		return false;
	}

	return true;
}

struct kbasep_printer *kbasep_printer_buffer_init(struct kbase_device *kbdev,
						  enum kbasep_printer_type type)
{
	struct kbasep_printer *kbpr = NULL;

	if (WARN_ON_ONCE((kbdev == NULL || !(type == KBASEP_PRINT_TYPE_DEV_INFO ||
					     type == KBASEP_PRINT_TYPE_DEV_WARN ||
					     type == KBASEP_PRINT_TYPE_DEV_ERR))))
		return NULL;

	kbpr = kbasep_printer_alloc(type);

	if (kbpr) {
		if (kfifo_alloc(&kbpr->fifo, KBASEP_PRINTER_BUFFER_MAX_SIZE, GFP_KERNEL)) {
			kfree(kbpr);
			return NULL;
		}
		kbpr->kbdev = kbdev;
	}

	return kbpr;
}

struct kbasep_printer *kbasep_printer_file_init(struct seq_file *file)
{
	struct kbasep_printer *kbpr = NULL;

	if (WARN_ON_ONCE(file == NULL))
		return NULL;

	kbpr = kbasep_printer_alloc(KBASEP_PRINT_TYPE_SEQ_FILE);

	if (kbpr)
		kbpr->file = file;

	return kbpr;
}

void kbasep_printer_term(struct kbasep_printer *kbpr)
{
	if (kbpr) {
		if (kfifo_initialized(&kbpr->fifo))
			kfifo_free(&kbpr->fifo);
		vfree(kbpr);
	}
}

void kbasep_printer_buffer_flush(struct kbasep_printer *kbpr)
{
	char buffer[KBASEP_PRINT_FORMAT_BUFFER_MAX_SIZE];
	unsigned int i;

	if (WARN_ON_ONCE(!kbasep_printer_validate(kbpr)))
		return;

	if (kfifo_is_empty(&kbpr->fifo))
		return;

	while (!kfifo_is_empty(&kbpr->fifo)) {
		/* copy elements to fill the local string buffer */
		size_t copied = kfifo_out_peek(&kbpr->fifo, buffer,
					       KBASEP_PRINT_FORMAT_BUFFER_MAX_SIZE - 1);
		buffer[copied] = '\0';
		/* pop all fifo copied elements until the first new-line char or
		 * the last copied element
		 */
		for (i = 0; i < copied; i++) {
			kfifo_skip(&kbpr->fifo);
			if (buffer[i] == '\n') {
				buffer[i + 1] = '\0';
				break;
			}
		}

		switch (kbpr->type) {
		case KBASEP_PRINT_TYPE_DEV_INFO:
			dev_info(kbpr->kbdev->dev, buffer);
			break;
		case KBASEP_PRINT_TYPE_DEV_WARN:
			dev_warn(kbpr->kbdev->dev, buffer);
			break;
		case KBASEP_PRINT_TYPE_DEV_ERR:
			dev_err(kbpr->kbdev->dev, buffer);
			break;
		default:
			pr_err("printer not supported");
		}
	}
}

void kbasep_puts(struct kbasep_printer *kbpr, const char *str)
{
	int len = 0;

	if (WARN_ON_ONCE(!kbasep_printer_validate(kbpr))) {
		pr_warn("%s", str);
		return;
	}

	switch (kbpr->type) {
	case KBASEP_PRINT_TYPE_DEV_INFO:
	case KBASEP_PRINT_TYPE_DEV_WARN:
	case KBASEP_PRINT_TYPE_DEV_ERR:
		len = strlen(str);
		if (len <= kfifo_avail(&kbpr->fifo))
			kfifo_in(&kbpr->fifo, str, len);
		break;
	case KBASEP_PRINT_TYPE_SEQ_FILE:
		seq_printf(kbpr->file, str);
		break;
	default:
		pr_err("printer not supported");
	}
}

__attribute__((format(__printf__, 2, 3))) void kbasep_print(struct kbasep_printer *kbpr,
							    const char *fmt, ...)
{
	int len = 0;
	char buffer[KBASEP_PRINT_FORMAT_BUFFER_MAX_SIZE];
	va_list arglist;

	va_start(arglist, fmt);
	len = vsnprintf(buffer, KBASEP_PRINT_FORMAT_BUFFER_MAX_SIZE, fmt, arglist);
	if (len <= 0) {
		pr_err("message write to the buffer failed");
		goto exit;
	}

	if (WARN_ON_ONCE(!kbasep_printer_validate(kbpr)))
		pr_warn("%s", buffer);

	if (kbpr->type == KBASEP_PRINT_TYPE_SEQ_FILE)
		seq_printf(kbpr->file, buffer);
	else if (len <= kfifo_avail(&kbpr->fifo))
		kfifo_in(&kbpr->fifo, buffer, len);
exit:
	va_end(arglist);
}
