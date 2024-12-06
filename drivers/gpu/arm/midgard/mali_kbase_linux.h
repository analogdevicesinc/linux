/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2010-2024 ARM Limited. All rights reserved.
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
 * DOC: Base kernel APIs, Linux implementation.
 */

#ifndef _KBASE_LINUX_H_
#define _KBASE_LINUX_H_

/* All things that are needed for the Linux port. */
#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/atomic.h>

#if IS_ENABLED(MALI_KERNEL_TEST_API)
#define KBASE_EXPORT_TEST_API(func) EXPORT_SYMBOL(func)
/* Note: due to the 2-layer macro translation, using the NULL _etype does not
 * compile, and one workaround is to use ERRNO_NULL instead.
 */
#define KBASE_ALLOW_ERROR_INJECTION_TEST_API(func, etype) ALLOW_ERROR_INJECTION(func, etype)
#else
#define KBASE_EXPORT_TEST_API(func)
#define KBASE_ALLOW_ERROR_INJECTION_TEST_API(func, etype)
#endif

#define KBASE_EXPORT_SYMBOL(func) EXPORT_SYMBOL(func)

#endif /* _KBASE_LINUX_H_ */
