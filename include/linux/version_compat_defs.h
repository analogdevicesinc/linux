/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2022-2023 ARM Limited. All rights reserved.
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

#ifndef _VERSION_COMPAT_DEFS_H_
#define _VERSION_COMPAT_DEFS_H_

#include <linux/version.h>
#include <linux/highmem.h>
#include <linux/timer.h>

#if (KERNEL_VERSION(4, 4, 267) < LINUX_VERSION_CODE)
#include <linux/overflow.h>
#endif

#if (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
#include <linux/bits.h>
#else
#include <linux/bitops.h>
#endif

#if KERNEL_VERSION(4, 16, 0) > LINUX_VERSION_CODE
typedef unsigned int __poll_t;
#endif

#if KERNEL_VERSION(4, 9, 78) >= LINUX_VERSION_CODE

#ifndef EPOLLHUP
#define EPOLLHUP POLLHUP
#endif

#ifndef EPOLLERR
#define EPOLLERR POLLERR
#endif

#ifndef EPOLLIN
#define EPOLLIN POLLIN
#endif

#ifndef EPOLLRDNORM
#define EPOLLRDNORM POLLRDNORM
#endif

#endif

#if KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE
/* This is defined inside kbase for matching the default to kernel's
 * mmap_min_addr, used inside file mali_kbase_mmap.c.
 * Note: the value is set at compile time, matching a kernel's configuration
 * value. It would not be able to track any runtime update of mmap_min_addr.
 */
#ifdef CONFIG_MMU
#define kbase_mmap_min_addr CONFIG_DEFAULT_MMAP_MIN_ADDR

#ifdef CONFIG_LSM_MMAP_MIN_ADDR
#if (CONFIG_LSM_MMAP_MIN_ADDR > CONFIG_DEFAULT_MMAP_MIN_ADDR)
/* Replace the default definition with CONFIG_LSM_MMAP_MIN_ADDR */
#undef kbase_mmap_min_addr
#define kbase_mmap_min_addr CONFIG_LSM_MMAP_MIN_ADDR
#pragma message "kbase_mmap_min_addr compiled to CONFIG_LSM_MMAP_MIN_ADDR, no runtime update!"
#endif /* (CONFIG_LSM_MMAP_MIN_ADDR > CONFIG_DEFAULT_MMAP_MIN_ADDR) */
#endif /* CONFIG_LSM_MMAP_MIN_ADDR */

#if (kbase_mmap_min_addr == CONFIG_DEFAULT_MMAP_MIN_ADDR)
#pragma message "kbase_mmap_min_addr compiled to CONFIG_DEFAULT_MMAP_MIN_ADDR, no runtime update!"
#endif

#else /* CONFIG_MMU */
#define kbase_mmap_min_addr (0UL)
#pragma message "kbase_mmap_min_addr compiled to (0UL), no runtime update!"
#endif /* CONFIG_MMU */
#endif /* KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE */

static inline void kbase_timer_setup(struct timer_list *timer,
				     void (*callback)(struct timer_list *timer))
{
#if KERNEL_VERSION(4, 14, 0) > LINUX_VERSION_CODE
	setup_timer(timer, (void (*)(unsigned long))callback, (unsigned long)timer);
#else
	timer_setup(timer, callback, 0);
#endif
}

#ifndef WRITE_ONCE
#ifdef ASSIGN_ONCE
#define WRITE_ONCE(x, val) ASSIGN_ONCE(val, x)
#else
#define WRITE_ONCE(x, val) (ACCESS_ONCE(x) = (val))
#endif
#endif

#ifndef READ_ONCE
#define READ_ONCE(x) ACCESS_ONCE(x)
#endif

static inline void *kbase_kmap(struct page *p)
{
#if KERNEL_VERSION(5, 11, 0) <= LINUX_VERSION_CODE
	return kmap_local_page(p);
#else
	return kmap(p);
#endif /* KERNEL_VERSION(5, 11, 0) */
}

static inline void *kbase_kmap_atomic(struct page *p)
{
#if KERNEL_VERSION(5, 11, 0) <= LINUX_VERSION_CODE
	return kmap_local_page(p);
#else
	return kmap_atomic(p);
#endif /* KERNEL_VERSION(5, 11, 0) */
}

static inline void kbase_kunmap(struct page *p, void *address)
{
#if KERNEL_VERSION(5, 11, 0) <= LINUX_VERSION_CODE
	kunmap_local(address);
#else
	kunmap(p);
#endif /* KERNEL_VERSION(5, 11, 0) */
}

static inline void kbase_kunmap_atomic(void *address)
{
#if KERNEL_VERSION(5, 11, 0) <= LINUX_VERSION_CODE
	kunmap_local(address);
#else
	kunmap_atomic(address);
#endif /* KERNEL_VERSION(5, 11, 0) */
}

/* Some of the older 4.4 kernel patch versions do
 * not contain the overflow check functions. However,
 * they are based on compiler instrinsics, so they
 * are simple to reproduce.
 */
#if (KERNEL_VERSION(4, 4, 267) >= LINUX_VERSION_CODE)
/* Some of the older 4.4 kernel patch versions do
 * not contain the overflow check functions. However,
 * they are based on compiler instrinsics, so they
 * are simple to reproduce.
 */
#define check_mul_overflow(a, b, d) __builtin_mul_overflow(a, b, d)
#endif

#endif /* _VERSION_COMPAT_DEFS_H_ */
