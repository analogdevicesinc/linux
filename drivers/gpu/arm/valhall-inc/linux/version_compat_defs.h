/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2022-2024 ARM Limited. All rights reserved.
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
#include <linux/iopoll.h>
#include <linux/bitmap.h>
#include <linux/math64.h>
#include <linux/moduleparam.h>

#if (KERNEL_VERSION(4, 4, 267) < LINUX_VERSION_CODE)
#include <linux/overflow.h>
#endif

#include <linux/bitops.h>
#if (KERNEL_VERSION(4, 19, 0) <= LINUX_VERSION_CODE)
#include <linux/bits.h>
#endif

#ifndef BITS_PER_TYPE
#define BITS_PER_TYPE(type) (sizeof(type) * BITS_PER_BYTE)
#endif

#if KERNEL_VERSION(4, 16, 0) > LINUX_VERSION_CODE
typedef unsigned int __poll_t;

#ifndef HRTIMER_MODE_REL_SOFT
#define HRTIMER_MODE_REL_SOFT HRTIMER_MODE_REL
#endif
#endif /* KERNEL_VERSION(4, 16, 0) > LINUX_VERSION_CODE */

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
#define KBASE_COMPILED_MMAP_MIN_ADDR_MSG \
	"* MALI kbase_mmap_min_addr compiled to CONFIG_LSM_MMAP_MIN_ADDR, no runtime update possible! *"
#endif /* (CONFIG_LSM_MMAP_MIN_ADDR > CONFIG_DEFAULT_MMAP_MIN_ADDR) */
#endif /* CONFIG_LSM_MMAP_MIN_ADDR */

#if (kbase_mmap_min_addr == CONFIG_DEFAULT_MMAP_MIN_ADDR)
#define KBASE_COMPILED_MMAP_MIN_ADDR_MSG \
	"* MALI kbase_mmap_min_addr compiled to CONFIG_DEFAULT_MMAP_MIN_ADDR, no runtime update possible! *"
#endif

#else /* CONFIG_MMU */
#define kbase_mmap_min_addr (0UL)
#define KBASE_COMPILED_MMAP_MIN_ADDR_MSG \
	"* MALI kbase_mmap_min_addr compiled to (0UL), no runtime update possible! *"
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

#ifndef CSTD_UNUSED
#define CSTD_UNUSED(x) ((void)(x))
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
	CSTD_UNUSED(p);
	kunmap_local(address);
#else
	CSTD_UNUSED(address);
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
#define check_add_overflow(a, b, d) __builtin_add_overflow(a, b, d)
#define check_sub_overflow(a, b, d) __builtin_sub_overflow(a, b, d)
#endif

/*
 * There was a big rename in the 4.10 kernel (fence* -> dma_fence*),
 * with most of the related functions keeping the same signatures.
 */

#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)

#include <linux/fence.h>

#define dma_fence fence
#define dma_fence_ops fence_ops
#define dma_fence_cb fence_cb
#define dma_fence_context_alloc(a) fence_context_alloc(a)
#define dma_fence_init(a, b, c, d, e) fence_init(a, b, c, d, e)
#define dma_fence_get(a) fence_get(a)
#define dma_fence_put(a) fence_put(a)
#define dma_fence_signal(a) fence_signal(a)
#define dma_fence_is_signaled(a) fence_is_signaled(a)
#define dma_fence_add_callback(a, b, c) fence_add_callback(a, b, c)
#define dma_fence_remove_callback(a, b) fence_remove_callback(a, b)
#define dma_fence_default_wait fence_default_wait

#if (KERNEL_VERSION(4, 9, 68) <= LINUX_VERSION_CODE)
#define dma_fence_get_status(a) (fence_is_signaled(a) ? (a)->error ?: 1 : 0)
#else
#define dma_fence_get_status(a) (fence_is_signaled(a) ? (a)->status ?: 1 : 0)
#endif

#else

#include <linux/dma-fence.h>

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)
#define dma_fence_get_status(a) (dma_fence_is_signaled(a) ? (a)->status ?: 1 : 0)
#endif

#endif /* < 4.10.0 */

static inline void dma_fence_set_error_helper(
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
	struct fence *fence,
#else
	struct dma_fence *fence,
#endif
	int error)
{
#if (KERNEL_VERSION(4, 11, 0) <= LINUX_VERSION_CODE)
	dma_fence_set_error(fence, error);
#elif (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE && \
       KERNEL_VERSION(4, 9, 68) <= LINUX_VERSION_CODE)
	fence_set_error(fence, error);
#else
	fence->status = error;
#endif
}

#include <linux/mm.h>
#if (KERNEL_VERSION(6, 1, 25) > LINUX_VERSION_CODE)
static inline void vm_flags_set(struct vm_area_struct *vma, vm_flags_t flags)
{
	vma->vm_flags |= flags;
}
static inline void vm_flags_clear(struct vm_area_struct *vma, vm_flags_t flags)
{
	vma->vm_flags &= ~flags;
}
#endif

static inline void kbase_unpin_user_buf_page(struct page *page)
{
#if KERNEL_VERSION(5, 9, 0) > LINUX_VERSION_CODE
	put_page(page);
#else
	unpin_user_page(page);
#endif
}

static inline long kbase_get_user_pages(unsigned long start, unsigned long nr_pages,
					unsigned int gup_flags, struct page **pages,
					struct vm_area_struct **vmas)
{
#if ((KERNEL_VERSION(6, 5, 0) > LINUX_VERSION_CODE) && !defined(__ANDROID_COMMON_KERNEL__)) || \
	((KERNEL_VERSION(6, 4, 0) > LINUX_VERSION_CODE) && defined(__ANDROID_COMMON_KERNEL__))
	return get_user_pages(start, nr_pages, gup_flags, pages, vmas);
#else
	return get_user_pages(start, nr_pages, gup_flags, pages);
#endif
}

static inline long kbase_pin_user_pages_remote(struct task_struct *tsk, struct mm_struct *mm,
					       unsigned long start, unsigned long nr_pages,
					       unsigned int gup_flags, struct page **pages,
					       struct vm_area_struct **vmas, int *locked)
{
#if KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE
	return get_user_pages_remote(tsk, mm, start, nr_pages, gup_flags, pages, vmas);
#elif KERNEL_VERSION(5, 6, 0) > LINUX_VERSION_CODE
	return get_user_pages_remote(tsk, mm, start, nr_pages, gup_flags, pages, vmas, locked);
#elif KERNEL_VERSION(5, 9, 0) > LINUX_VERSION_CODE
	return pin_user_pages_remote(tsk, mm, start, nr_pages, gup_flags, pages, vmas, locked);
#elif ((KERNEL_VERSION(6, 5, 0) > LINUX_VERSION_CODE) && !defined(__ANDROID_COMMON_KERNEL__)) || \
	((KERNEL_VERSION(6, 4, 0) > LINUX_VERSION_CODE) && defined(__ANDROID_COMMON_KERNEL__))
	return pin_user_pages_remote(mm, start, nr_pages, gup_flags, pages, vmas, locked);
#else
	return pin_user_pages_remote(mm, start, nr_pages, gup_flags, pages, locked);
#endif
}

#if (KERNEL_VERSION(6, 4, 0) <= LINUX_VERSION_CODE)
#define KBASE_CLASS_CREATE(owner, name) class_create(name)
#else
#define KBASE_CLASS_CREATE(owner, name) class_create(owner, name)
#endif /* (KERNEL_VERSION(6, 4, 0) <= LINUX_VERSION_CODE) */

#if KERNEL_VERSION(5, 0, 0) > LINUX_VERSION_CODE
#define kbase_totalram_pages() totalram_pages
#else
#define kbase_totalram_pages() totalram_pages()
#endif /* KERNEL_VERSION(5, 0, 0) > LINUX_VERSION_CODE */

#ifndef read_poll_timeout_atomic
#define read_poll_timeout_atomic(op, val, cond, delay_us, timeout_us, delay_before_read, args...) \
	({                                                                                        \
		const u64 __timeout_us = (timeout_us);                                            \
		s64 __left_ns = __timeout_us * NSEC_PER_USEC;                                     \
		const unsigned long __delay_us = (delay_us);                                      \
		const u64 __delay_ns = __delay_us * NSEC_PER_USEC;                                \
		if (delay_before_read && __delay_us)                                              \
			udelay(__delay_us);                                                       \
		if (__timeout_us)                                                                 \
			__left_ns -= __delay_ns;                                                  \
		do {                                                                              \
			(val) = op(args);                                                         \
			if (__timeout_us) {                                                       \
				if (__delay_us) {                                                 \
					udelay(__delay_us);                                       \
					__left_ns -= __delay_ns;                                  \
				}                                                                 \
				__left_ns--;                                                      \
			}                                                                         \
		} while (!(cond) && (!__timeout_us || (__left_ns > 0)));                          \
		(cond) ? 0 : -ETIMEDOUT;                                                          \
	})
#endif

#if (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE)

#define kbase_refcount_t atomic_t
#define kbase_refcount_read(x) atomic_read(x)
#define kbase_refcount_set(x, v) atomic_set(x, v)
#define kbase_refcount_dec_and_test(x) atomic_dec_and_test(x)
#define kbase_refcount_dec(x) atomic_dec(x)
#define kbase_refcount_inc_not_zero(x) atomic_inc_not_zero(x)
#define kbase_refcount_inc(x) atomic_inc(x)

#else

#include <linux/refcount.h>

#define kbase_refcount_t refcount_t
#define kbase_refcount_read(x) refcount_read(x)
#define kbase_refcount_set(x, v) refcount_set(x, v)
#define kbase_refcount_dec_and_test(x) refcount_dec_and_test(x)
#define kbase_refcount_dec(x) refcount_dec(x)
#define kbase_refcount_inc_not_zero(x) refcount_inc_not_zero(x)
#define kbase_refcount_inc(x) refcount_inc(x)

#endif /* (KERNEL_VERSION(4, 11, 0) > LINUX_VERSION_CODE) */

#if (KERNEL_VERSION(4, 16, 0) > LINUX_VERSION_CODE)
/* Null definition */
#define ALLOW_ERROR_INJECTION(fname, err_type)
#endif /* (KERNEL_VERSION(4, 16, 0) > LINUX_VERSION_CODE) */

#if KERNEL_VERSION(6, 0, 0) > LINUX_VERSION_CODE
#define KBASE_REGISTER_SHRINKER(reclaim, name, priv_data) register_shrinker(reclaim)

/* clang-format off */
#elif ((KERNEL_VERSION(6, 7, 0) > LINUX_VERSION_CODE) && \
	!(defined(__ANDROID_COMMON_KERNEL__) && (KERNEL_VERSION(6, 6, 0) == LINUX_VERSION_CODE)))
/* clang-format on */
#define KBASE_REGISTER_SHRINKER(reclaim, name, priv_data) register_shrinker(reclaim, name)

#else
#define KBASE_REGISTER_SHRINKER(reclaim, name, priv_data) \
	do {                                              \
		reclaim->private_data = priv_data;        \
		shrinker_register(reclaim);               \
	} while (0)

#endif /* KERNEL_VERSION(6, 0, 0) > LINUX_VERSION_CODE */
/* clang-format off */
#if ((KERNEL_VERSION(6, 7, 0) > LINUX_VERSION_CODE) && \
	!(defined(__ANDROID_COMMON_KERNEL__) && (KERNEL_VERSION(6, 6, 0) == LINUX_VERSION_CODE)))
/* clang-format on */
#define KBASE_UNREGISTER_SHRINKER(reclaim) unregister_shrinker(&reclaim)
#define KBASE_GET_KBASE_DATA_FROM_SHRINKER(s, type, var) container_of(s, type, var)
#define DEFINE_KBASE_SHRINKER struct shrinker
#define KBASE_INIT_RECLAIM(var, attr, name) (&((var)->attr))
#define KBASE_SET_RECLAIM(var, attr, reclaim) ((var)->attr = (*reclaim))

#else
#define KBASE_UNREGISTER_SHRINKER(reclaim) shrinker_free(reclaim)
#define KBASE_GET_KBASE_DATA_FROM_SHRINKER(s, type, var) s->private_data
#define DEFINE_KBASE_SHRINKER struct shrinker *
#define KBASE_SHRINKER_ALLOC(name) shrinker_alloc(0, name)
#define KBASE_INIT_RECLAIM(var, attr, name) (KBASE_SHRINKER_ALLOC(name))
#define KBASE_SET_RECLAIM(var, attr, reclaim) ((var)->attr = reclaim)

#endif

static inline int kbase_param_set_uint_minmax(const char *val, const struct kernel_param *kp,
					      unsigned int min, unsigned int max)
{
#if (KERNEL_VERSION(5, 15, 0) > LINUX_VERSION_CODE)
	uint uint_val;
	int ret;

	if (!val)
		return -EINVAL;

	ret = kstrtouint(val, 0, &uint_val);

	if (ret == 0) {
		if (uint_val < min || uint_val > max)
			return -EINVAL;

		*((uint *)kp->arg) = uint_val;
	}

	return ret;
#else
	return param_set_uint_minmax(val, kp, min, max);
#endif
}

#if (KERNEL_VERSION(4, 20, 0) <= LINUX_VERSION_CODE)
#include <linux/compiler_attributes.h>
#endif
#ifndef __maybe_unused
#define __maybe_unused __attribute__((unused))
#endif

#if KERNEL_VERSION(5, 4, 103) <= LINUX_VERSION_CODE
#define mali_sysfs_emit(buf, fmt, ...) sysfs_emit(buf, fmt, __VA_ARGS__)
#else
#define mali_sysfs_emit(buf, fmt, ...) scnprintf(buf, PAGE_SIZE, fmt, __VA_ARGS__)
#endif

#if KERNEL_VERSION(5, 10, 0) > LINUX_VERSION_CODE
#include <linux/devfreq.h>
#include <linux/of_platform.h>

static inline struct devfreq *devfreq_get_devfreq_by_node(struct device_node *node)
{
	struct platform_device *pdev = of_find_device_by_node(node);

	if (!pdev || !node)
		return NULL;

	return devfreq_get_devfreq_by_phandle(&pdev->dev, 0);
}
#endif

#if (KERNEL_VERSION(5, 16, 0) <= LINUX_VERSION_CODE &&       \
	KERNEL_VERSION(5, 18, 0) > LINUX_VERSION_CODE) ||       \
	(KERNEL_VERSION(5, 11, 0) <= LINUX_VERSION_CODE &&   \
	KERNEL_VERSION(5, 15, 85) >= LINUX_VERSION_CODE) || \
	(KERNEL_VERSION(5, 10, 200) >= LINUX_VERSION_CODE)
/*
 * Kernel revisions
 *  - up to 5.10.200
 *  - between 5.11.0 and 5.15.85 inclusive
 *  - between 5.16.0 and 5.17.15 inclusive
 * do not provide an implementation of
 * size_add, size_sub and size_mul.
 * The implementations below provides
 * backward compatibility implementations of these functions.
 */

static inline size_t __must_check size_mul(size_t factor1, size_t factor2)
{
	size_t ret_val;

	if (check_mul_overflow(factor1, factor2, &ret_val))
		return SIZE_MAX;
	return ret_val;
}

static inline size_t __must_check size_add(size_t addend1, size_t addend2)
{
	size_t ret_val;

	if (check_add_overflow(addend1, addend2, &ret_val))
		return SIZE_MAX;
	return ret_val;
}

static inline size_t __must_check size_sub(size_t minuend, size_t subtrahend)
{
	size_t ret_val;

	if (minuend == SIZE_MAX || subtrahend == SIZE_MAX ||
	    check_sub_overflow(minuend, subtrahend, &ret_val))
		return SIZE_MAX;
	return ret_val;
}
#endif

#if KERNEL_VERSION(5, 5, 0) > LINUX_VERSION_CODE
static inline unsigned long bitmap_get_value8(const unsigned long *map, unsigned long start)
{
	const size_t index = BIT_WORD(start);
	const unsigned long offset = start % BITS_PER_LONG;

	return (map[index] >> offset) & 0xFF;
}

static inline unsigned long find_next_clump8(unsigned long *clump, const unsigned long *addr,
					     unsigned long size, unsigned long offset)
{
	offset = find_next_bit(addr, size, offset);
	if (offset == size)
		return size;

	offset = round_down(offset, 8);
	*clump = bitmap_get_value8(addr, offset);

	return offset;
}

#define find_first_clump8(clump, bits, size) find_next_clump8((clump), (bits), (size), 0)

#define for_each_set_clump8(start, clump, bits, size)                                 \
	for ((start) = find_first_clump8(&(clump), (bits), (size)); (start) < (size); \
	     (start) = find_next_clump8(&(clump), (bits), (size), (start) + 8))
#endif

#endif /* _VERSION_COMPAT_DEFS_H_ */
