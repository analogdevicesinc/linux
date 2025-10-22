// SPDX-License-Identifier: GPL-2.0-only
#include <linux/mm.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/compiler.h>
#include <linux/export.h>
#include <linux/err.h>
#include <linux/sched.h>
#include <linux/sched/mm.h>
#include <linux/sched/signal.h>
#include <linux/sched/task_stack.h>
#include <linux/security.h>
#include <linux/swap.h>
#include <linux/swapops.h>
#include <linux/sysctl.h>
#include <linux/mman.h>
#include <linux/hugetlb.h>
#include <linux/vmalloc.h>
#include <linux/userfaultfd_k.h>
#include <linux/elf.h>
#include <linux/elf-randomize.h>
#include <linux/personality.h>
#include <linux/random.h>
#include <linux/processor.h>
#include <linux/sizes.h>
#include <linux/compat.h>
#include <linux/fsnotify.h>
#include <linux/page_idle.h>

#include <linux/uaccess.h>

#include <kunit/visibility.h>

#include "internal.h"
#include "swap.h"

/**
 * kfree_const - conditionally free memory
 * @x: pointer to the memory
 *
 * Function calls kfree only if @x is not in .rodata section.
 */
void kfree_const(const void *x)
{
	if (!is_kernel_rodata((unsigned long)x))
		kfree(x);
}
EXPORT_SYMBOL(kfree_const);

/**
 * __kmemdup_nul - Create a NUL-terminated string from @s, which might be unterminated.
 * @s: The data to copy
 * @len: The size of the data, not including the NUL terminator
 * @gfp: the GFP mask used in the kmalloc() call when allocating memory
 *
 * Return: newly allocated copy of @s with NUL-termination or %NULL in
 * case of error
 */
static __always_inline char *__kmemdup_nul(const char *s, size_t len, gfp_t gfp)
{
	char *buf;

	/* '+1' for the NUL terminator */
	buf = kmalloc_track_caller(len + 1, gfp);
	if (!buf)
		return NULL;

	memcpy(buf, s, len);
	/* Ensure the buf is always NUL-terminated, regardless of @s. */
	buf[len] = '\0';
	return buf;
}

/**
 * kstrdup - allocate space for and copy an existing string
 * @s: the string to duplicate
 * @gfp: the GFP mask used in the kmalloc() call when allocating memory
 *
 * Return: newly allocated copy of @s or %NULL in case of error
 */
noinline
char *kstrdup(const char *s, gfp_t gfp)
{
	return s ? __kmemdup_nul(s, strlen(s), gfp) : NULL;
}
EXPORT_SYMBOL(kstrdup);

/**
 * kstrdup_const - conditionally duplicate an existing const string
 * @s: the string to duplicate
 * @gfp: the GFP mask used in the kmalloc() call when allocating memory
 *
 * Note: Strings allocated by kstrdup_const should be freed by kfree_const and
 * must not be passed to krealloc().
 *
 * Return: source string if it is in .rodata section otherwise
 * fallback to kstrdup.
 */
const char *kstrdup_const(const char *s, gfp_t gfp)
{
	if (is_kernel_rodata((unsigned long)s))
		return s;

	return kstrdup(s, gfp);
}
EXPORT_SYMBOL(kstrdup_const);

/**
 * kstrndup - allocate space for and copy an existing string
 * @s: the string to duplicate
 * @max: read at most @max chars from @s
 * @gfp: the GFP mask used in the kmalloc() call when allocating memory
 *
 * Note: Use kmemdup_nul() instead if the size is known exactly.
 *
 * Return: newly allocated copy of @s or %NULL in case of error
 */
char *kstrndup(const char *s, size_t max, gfp_t gfp)
{
	return s ? __kmemdup_nul(s, strnlen(s, max), gfp) : NULL;
}
EXPORT_SYMBOL(kstrndup);

/**
 * kmemdup - duplicate region of memory
 *
 * @src: memory region to duplicate
 * @len: memory region length
 * @gfp: GFP mask to use
 *
 * Return: newly allocated copy of @src or %NULL in case of error,
 * result is physically contiguous. Use kfree() to free.
 */
void *kmemdup_noprof(const void *src, size_t len, gfp_t gfp)
{
	void *p;

	p = kmalloc_node_track_caller_noprof(len, gfp, NUMA_NO_NODE, _RET_IP_);
	if (p)
		memcpy(p, src, len);
	return p;
}
EXPORT_SYMBOL(kmemdup_noprof);

/**
 * kmemdup_array - duplicate a given array.
 *
 * @src: array to duplicate.
 * @count: number of elements to duplicate from array.
 * @element_size: size of each element of array.
 * @gfp: GFP mask to use.
 *
 * Return: duplicated array of @src or %NULL in case of error,
 * result is physically contiguous. Use kfree() to free.
 */
void *kmemdup_array(const void *src, size_t count, size_t element_size, gfp_t gfp)
{
	return kmemdup(src, size_mul(element_size, count), gfp);
}
EXPORT_SYMBOL(kmemdup_array);

/**
 * kvmemdup - duplicate region of memory
 *
 * @src: memory region to duplicate
 * @len: memory region length
 * @gfp: GFP mask to use
 *
 * Return: newly allocated copy of @src or %NULL in case of error,
 * result may be not physically contiguous. Use kvfree() to free.
 */
void *kvmemdup(const void *src, size_t len, gfp_t gfp)
{
	void *p;

	p = kvmalloc(len, gfp);
	if (p)
		memcpy(p, src, len);
	return p;
}
EXPORT_SYMBOL(kvmemdup);

/**
 * kmemdup_nul - Create a NUL-terminated string from unterminated data
 * @s: The data to stringify
 * @len: The size of the data
 * @gfp: the GFP mask used in the kmalloc() call when allocating memory
 *
 * Return: newly allocated copy of @s with NUL-termination or %NULL in
 * case of error
 */
char *kmemdup_nul(const char *s, size_t len, gfp_t gfp)
{
	return s ? __kmemdup_nul(s, len, gfp) : NULL;
}
EXPORT_SYMBOL(kmemdup_nul);

static kmem_buckets *user_buckets __ro_after_init;

static int __init init_user_buckets(void)
{
	user_buckets = kmem_buckets_create("memdup_user", 0, 0, INT_MAX, NULL);

	return 0;
}
subsys_initcall(init_user_buckets);

/**
 * memdup_user - duplicate memory region from user space
 *
 * @src: source address in user space
 * @len: number of bytes to copy
 *
 * Return: an ERR_PTR() on failure.  Result is physically
 * contiguous, to be freed by kfree().
 */
void *memdup_user(const void __user *src, size_t len)
{
	void *p;

	p = kmem_buckets_alloc_track_caller(user_buckets, len, GFP_USER | __GFP_NOWARN);
	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, src, len)) {
		kfree(p);
		return ERR_PTR(-EFAULT);
	}

	return p;
}
EXPORT_SYMBOL(memdup_user);

/**
 * vmemdup_user - duplicate memory region from user space
 *
 * @src: source address in user space
 * @len: number of bytes to copy
 *
 * Return: an ERR_PTR() on failure.  Result may be not
 * physically contiguous.  Use kvfree() to free.
 */
void *vmemdup_user(const void __user *src, size_t len)
{
	void *p;

	p = kmem_buckets_valloc(user_buckets, len, GFP_USER);
	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, src, len)) {
		kvfree(p);
		return ERR_PTR(-EFAULT);
	}

	return p;
}
EXPORT_SYMBOL(vmemdup_user);

/**
 * strndup_user - duplicate an existing string from user space
 * @s: The string to duplicate
 * @n: Maximum number of bytes to copy, including the trailing NUL.
 *
 * Return: newly allocated copy of @s or an ERR_PTR() in case of error
 */
char *strndup_user(const char __user *s, long n)
{
	char *p;
	long length;

	length = strnlen_user(s, n);

	if (!length)
		return ERR_PTR(-EFAULT);

	if (length > n)
		return ERR_PTR(-EINVAL);

	p = memdup_user(s, length);

	if (IS_ERR(p))
		return p;

	p[length - 1] = '\0';

	return p;
}
EXPORT_SYMBOL(strndup_user);

/**
 * memdup_user_nul - duplicate memory region from user space and NUL-terminate
 *
 * @src: source address in user space
 * @len: number of bytes to copy
 *
 * Return: an ERR_PTR() on failure.
 */
void *memdup_user_nul(const void __user *src, size_t len)
{
	char *p;

	p = kmem_buckets_alloc_track_caller(user_buckets, len + 1, GFP_USER | __GFP_NOWARN);
	if (!p)
		return ERR_PTR(-ENOMEM);

	if (copy_from_user(p, src, len)) {
		kfree(p);
		return ERR_PTR(-EFAULT);
	}
	p[len] = '\0';

	return p;
}
EXPORT_SYMBOL(memdup_user_nul);

/* Check if the vma is being used as a stack by this task */
int vma_is_stack_for_current(const struct vm_area_struct *vma)
{
	struct task_struct * __maybe_unused t = current;

	return (vma->vm_start <= KSTK_ESP(t) && vma->vm_end >= KSTK_ESP(t));
}

/*
 * Change backing file, only valid to use during initial VMA setup.
 */
void vma_set_file(struct vm_area_struct *vma, struct file *file)
{
	/* Changing an anonymous vma with this is illegal */
	get_file(file);
	swap(vma->vm_file, file);
	fput(file);
}
EXPORT_SYMBOL(vma_set_file);

#ifndef STACK_RND_MASK
#define STACK_RND_MASK (0x7ff >> (PAGE_SHIFT - 12))     /* 8MB of VA */
#endif

unsigned long randomize_stack_top(unsigned long stack_top)
{
	unsigned long random_variable = 0;

	if (current->flags & PF_RANDOMIZE) {
		random_variable = get_random_long();
		random_variable &= STACK_RND_MASK;
		random_variable <<= PAGE_SHIFT;
	}
#ifdef CONFIG_STACK_GROWSUP
	return PAGE_ALIGN(stack_top) + random_variable;
#else
	return PAGE_ALIGN(stack_top) - random_variable;
#endif
}

/**
 * randomize_page - Generate a random, page aligned address
 * @start:	The smallest acceptable address the caller will take.
 * @range:	The size of the area, starting at @start, within which the
 *		random address must fall.
 *
 * If @start + @range would overflow, @range is capped.
 *
 * NOTE: Historical use of randomize_range, which this replaces, presumed that
 * @start was already page aligned.  We now align it regardless.
 *
 * Return: A page aligned address within [start, start + range).  On error,
 * @start is returned.
 */
unsigned long randomize_page(unsigned long start, unsigned long range)
{
	if (!PAGE_ALIGNED(start)) {
		range -= PAGE_ALIGN(start) - start;
		start = PAGE_ALIGN(start);
	}

	if (start > ULONG_MAX - range)
		range = ULONG_MAX - start;

	range >>= PAGE_SHIFT;

	if (range == 0)
		return start;

	return start + (get_random_long() % range << PAGE_SHIFT);
}

#ifdef CONFIG_ARCH_WANT_DEFAULT_TOPDOWN_MMAP_LAYOUT
unsigned long __weak arch_randomize_brk(struct mm_struct *mm)
{
	/* Is the current task 32bit ? */
	if (!IS_ENABLED(CONFIG_64BIT) || is_compat_task())
		return randomize_page(mm->brk, SZ_32M);

	return randomize_page(mm->brk, SZ_1G);
}

unsigned long arch_mmap_rnd(void)
{
	unsigned long rnd;

#ifdef CONFIG_HAVE_ARCH_MMAP_RND_COMPAT_BITS
	if (is_compat_task())
		rnd = get_random_long() & ((1UL << mmap_rnd_compat_bits) - 1);
	else
#endif /* CONFIG_HAVE_ARCH_MMAP_RND_COMPAT_BITS */
		rnd = get_random_long() & ((1UL << mmap_rnd_bits) - 1);

	return rnd << PAGE_SHIFT;
}

static int mmap_is_legacy(const struct rlimit *rlim_stack)
{
	if (current->personality & ADDR_COMPAT_LAYOUT)
		return 1;

	/* On parisc the stack always grows up - so a unlimited stack should
	 * not be an indicator to use the legacy memory layout. */
	if (rlim_stack->rlim_cur == RLIM_INFINITY &&
		!IS_ENABLED(CONFIG_STACK_GROWSUP))
		return 1;

	return sysctl_legacy_va_layout;
}

/*
 * Leave enough space between the mmap area and the stack to honour ulimit in
 * the face of randomisation.
 */
#define MIN_GAP		(SZ_128M)
#define MAX_GAP		(STACK_TOP / 6 * 5)

static unsigned long mmap_base(const unsigned long rnd, const struct rlimit *rlim_stack)
{
#ifdef CONFIG_STACK_GROWSUP
	/*
	 * For an upwards growing stack the calculation is much simpler.
	 * Memory for the maximum stack size is reserved at the top of the
	 * task. mmap_base starts directly below the stack and grows
	 * downwards.
	 */
	return PAGE_ALIGN_DOWN(mmap_upper_limit(rlim_stack) - rnd);
#else
	unsigned long gap = rlim_stack->rlim_cur;
	unsigned long pad = stack_guard_gap;

	/* Account for stack randomization if necessary */
	if (current->flags & PF_RANDOMIZE)
		pad += (STACK_RND_MASK << PAGE_SHIFT);

	/* Values close to RLIM_INFINITY can overflow. */
	if (gap + pad > gap)
		gap += pad;

	if (gap < MIN_GAP && MIN_GAP < MAX_GAP)
		gap = MIN_GAP;
	else if (gap > MAX_GAP)
		gap = MAX_GAP;

	return PAGE_ALIGN(STACK_TOP - gap - rnd);
#endif
}

void arch_pick_mmap_layout(struct mm_struct *mm, const struct rlimit *rlim_stack)
{
	unsigned long random_factor = 0UL;

	if (current->flags & PF_RANDOMIZE)
		random_factor = arch_mmap_rnd();

	if (mmap_is_legacy(rlim_stack)) {
		mm->mmap_base = TASK_UNMAPPED_BASE + random_factor;
		mm_flags_clear(MMF_TOPDOWN, mm);
	} else {
		mm->mmap_base = mmap_base(random_factor, rlim_stack);
		mm_flags_set(MMF_TOPDOWN, mm);
	}
}
#elif defined(CONFIG_MMU) && !defined(HAVE_ARCH_PICK_MMAP_LAYOUT)
void arch_pick_mmap_layout(struct mm_struct *mm, const struct rlimit *rlim_stack)
{
	mm->mmap_base = TASK_UNMAPPED_BASE;
	mm_flags_clear(MMF_TOPDOWN, mm);
}
#endif
#ifdef CONFIG_MMU
EXPORT_SYMBOL_IF_KUNIT(arch_pick_mmap_layout);
#endif

/**
 * __account_locked_vm - account locked pages to an mm's locked_vm
 * @mm:          mm to account against
 * @pages:       number of pages to account
 * @inc:         %true if @pages should be considered positive, %false if not
 * @task:        task used to check RLIMIT_MEMLOCK
 * @bypass_rlim: %true if checking RLIMIT_MEMLOCK should be skipped
 *
 * Assumes @task and @mm are valid (i.e. at least one reference on each), and
 * that mmap_lock is held as writer.
 *
 * Return:
 * * 0       on success
 * * -ENOMEM if RLIMIT_MEMLOCK would be exceeded.
 */
int __account_locked_vm(struct mm_struct *mm, unsigned long pages, bool inc,
			const struct task_struct *task, bool bypass_rlim)
{
	unsigned long locked_vm, limit;
	int ret = 0;

	mmap_assert_write_locked(mm);

	locked_vm = mm->locked_vm;
	if (inc) {
		if (!bypass_rlim) {
			limit = task_rlimit(task, RLIMIT_MEMLOCK) >> PAGE_SHIFT;
			if (locked_vm + pages > limit)
				ret = -ENOMEM;
		}
		if (!ret)
			mm->locked_vm = locked_vm + pages;
	} else {
		WARN_ON_ONCE(pages > locked_vm);
		mm->locked_vm = locked_vm - pages;
	}

	pr_debug("%s: [%d] caller %ps %c%lu %lu/%lu%s\n", __func__, task->pid,
		 (void *)_RET_IP_, (inc) ? '+' : '-', pages << PAGE_SHIFT,
		 locked_vm << PAGE_SHIFT, task_rlimit(task, RLIMIT_MEMLOCK),
		 ret ? " - exceeded" : "");

	return ret;
}
EXPORT_SYMBOL_GPL(__account_locked_vm);

/**
 * account_locked_vm - account locked pages to an mm's locked_vm
 * @mm:          mm to account against, may be NULL
 * @pages:       number of pages to account
 * @inc:         %true if @pages should be considered positive, %false if not
 *
 * Assumes a non-NULL @mm is valid (i.e. at least one reference on it).
 *
 * Return:
 * * 0       on success, or if mm is NULL
 * * -ENOMEM if RLIMIT_MEMLOCK would be exceeded.
 */
int account_locked_vm(struct mm_struct *mm, unsigned long pages, bool inc)
{
	int ret;

	if (pages == 0 || !mm)
		return 0;

	mmap_write_lock(mm);
	ret = __account_locked_vm(mm, pages, inc, current,
				  capable(CAP_IPC_LOCK));
	mmap_write_unlock(mm);

	return ret;
}
EXPORT_SYMBOL_GPL(account_locked_vm);

unsigned long vm_mmap_pgoff(struct file *file, unsigned long addr,
	unsigned long len, unsigned long prot,
	unsigned long flag, unsigned long pgoff)
{
	loff_t off = (loff_t)pgoff << PAGE_SHIFT;
	unsigned long ret;
	struct mm_struct *mm = current->mm;
	unsigned long populate;
	LIST_HEAD(uf);

	ret = security_mmap_file(file, prot, flag);
	if (!ret)
		ret = fsnotify_mmap_perm(file, prot, off, len);
	if (!ret) {
		if (mmap_write_lock_killable(mm))
			return -EINTR;
		ret = do_mmap(file, addr, len, prot, flag, 0, pgoff, &populate,
			      &uf);
		mmap_write_unlock(mm);
		userfaultfd_unmap_complete(mm, &uf);
		if (populate)
			mm_populate(ret, populate);
	}
	return ret;
}

/*
 * Perform a userland memory mapping into the current process address space. See
 * the comment for do_mmap() for more details on this operation in general.
 *
 * This differs from do_mmap() in that:
 *
 * a. An offset parameter is provided rather than pgoff, which is both checked
 *    for overflow and page alignment.
 * b. mmap locking is performed on the caller's behalf.
 * c. Userfaultfd unmap events and memory population are handled.
 *
 * This means that this function performs essentially the same work as if
 * userland were invoking mmap (2).
 *
 * Returns either an error, or the address at which the requested mapping has
 * been performed.
 */
unsigned long vm_mmap(struct file *file, unsigned long addr,
	unsigned long len, unsigned long prot,
	unsigned long flag, unsigned long offset)
{
	if (unlikely(offset + PAGE_ALIGN(len) < offset))
		return -EINVAL;
	if (unlikely(offset_in_page(offset)))
		return -EINVAL;

	return vm_mmap_pgoff(file, addr, len, prot, flag, offset >> PAGE_SHIFT);
}
EXPORT_SYMBOL(vm_mmap);

/**
 * __vmalloc_array - allocate memory for a virtually contiguous array.
 * @n: number of elements.
 * @size: element size.
 * @flags: the type of memory to allocate (see kmalloc).
 */
void *__vmalloc_array_noprof(size_t n, size_t size, gfp_t flags)
{
	size_t bytes;

	if (unlikely(check_mul_overflow(n, size, &bytes)))
		return NULL;
	return __vmalloc_noprof(bytes, flags);
}
EXPORT_SYMBOL(__vmalloc_array_noprof);

/**
 * vmalloc_array - allocate memory for a virtually contiguous array.
 * @n: number of elements.
 * @size: element size.
 */
void *vmalloc_array_noprof(size_t n, size_t size)
{
	return __vmalloc_array_noprof(n, size, GFP_KERNEL);
}
EXPORT_SYMBOL(vmalloc_array_noprof);

/**
 * __vcalloc - allocate and zero memory for a virtually contiguous array.
 * @n: number of elements.
 * @size: element size.
 * @flags: the type of memory to allocate (see kmalloc).
 */
void *__vcalloc_noprof(size_t n, size_t size, gfp_t flags)
{
	return __vmalloc_array_noprof(n, size, flags | __GFP_ZERO);
}
EXPORT_SYMBOL(__vcalloc_noprof);

/**
 * vcalloc - allocate and zero memory for a virtually contiguous array.
 * @n: number of elements.
 * @size: element size.
 */
void *vcalloc_noprof(size_t n, size_t size)
{
	return __vmalloc_array_noprof(n, size, GFP_KERNEL | __GFP_ZERO);
}
EXPORT_SYMBOL(vcalloc_noprof);

struct anon_vma *folio_anon_vma(const struct folio *folio)
{
	unsigned long mapping = (unsigned long)folio->mapping;

	if ((mapping & FOLIO_MAPPING_FLAGS) != FOLIO_MAPPING_ANON)
		return NULL;
	return (void *)(mapping - FOLIO_MAPPING_ANON);
}

/**
 * folio_mapping - Find the mapping where this folio is stored.
 * @folio: The folio.
 *
 * For folios which are in the page cache, return the mapping that this
 * page belongs to.  Folios in the swap cache return the swap mapping
 * this page is stored in (which is different from the mapping for the
 * swap file or swap device where the data is stored).
 *
 * You can call this for folios which aren't in the swap cache or page
 * cache and it will return NULL.
 */
struct address_space *folio_mapping(const struct folio *folio)
{
	struct address_space *mapping;

	/* This happens if someone calls flush_dcache_page on slab page */
	if (unlikely(folio_test_slab(folio)))
		return NULL;

	if (unlikely(folio_test_swapcache(folio)))
		return swap_address_space(folio->swap);

	mapping = folio->mapping;
	if ((unsigned long)mapping & FOLIO_MAPPING_FLAGS)
		return NULL;

	return mapping;
}
EXPORT_SYMBOL(folio_mapping);

/**
 * folio_copy - Copy the contents of one folio to another.
 * @dst: Folio to copy to.
 * @src: Folio to copy from.
 *
 * The bytes in the folio represented by @src are copied to @dst.
 * Assumes the caller has validated that @dst is at least as large as @src.
 * Can be called in atomic context for order-0 folios, but if the folio is
 * larger, it may sleep.
 */
void folio_copy(struct folio *dst, struct folio *src)
{
	long i = 0;
	long nr = folio_nr_pages(src);

	for (;;) {
		copy_highpage(folio_page(dst, i), folio_page(src, i));
		if (++i == nr)
			break;
		cond_resched();
	}
}
EXPORT_SYMBOL(folio_copy);

int folio_mc_copy(struct folio *dst, struct folio *src)
{
	long nr = folio_nr_pages(src);
	long i = 0;

	for (;;) {
		if (copy_mc_highpage(folio_page(dst, i), folio_page(src, i)))
			return -EHWPOISON;
		if (++i == nr)
			break;
		cond_resched();
	}

	return 0;
}
EXPORT_SYMBOL(folio_mc_copy);

int sysctl_overcommit_memory __read_mostly = OVERCOMMIT_GUESS;
static int sysctl_overcommit_ratio __read_mostly = 50;
static unsigned long sysctl_overcommit_kbytes __read_mostly;
int sysctl_max_map_count __read_mostly = DEFAULT_MAX_MAP_COUNT;
unsigned long sysctl_user_reserve_kbytes __read_mostly = 1UL << 17; /* 128MB */
unsigned long sysctl_admin_reserve_kbytes __read_mostly = 1UL << 13; /* 8MB */

#ifdef CONFIG_SYSCTL

static int overcommit_ratio_handler(const struct ctl_table *table, int write,
				void *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_dointvec(table, write, buffer, lenp, ppos);
	if (ret == 0 && write)
		sysctl_overcommit_kbytes = 0;
	return ret;
}

static void sync_overcommit_as(struct work_struct *dummy)
{
	percpu_counter_sync(&vm_committed_as);
}

static int overcommit_policy_handler(const struct ctl_table *table, int write,
				void *buffer, size_t *lenp, loff_t *ppos)
{
	struct ctl_table t;
	int new_policy = -1;
	int ret;

	/*
	 * The deviation of sync_overcommit_as could be big with loose policy
	 * like OVERCOMMIT_ALWAYS/OVERCOMMIT_GUESS. When changing policy to
	 * strict OVERCOMMIT_NEVER, we need to reduce the deviation to comply
	 * with the strict "NEVER", and to avoid possible race condition (even
	 * though user usually won't too frequently do the switching to policy
	 * OVERCOMMIT_NEVER), the switch is done in the following order:
	 *	1. changing the batch
	 *	2. sync percpu count on each CPU
	 *	3. switch the policy
	 */
	if (write) {
		t = *table;
		t.data = &new_policy;
		ret = proc_dointvec_minmax(&t, write, buffer, lenp, ppos);
		if (ret || new_policy == -1)
			return ret;

		mm_compute_batch(new_policy);
		if (new_policy == OVERCOMMIT_NEVER)
			schedule_on_each_cpu(sync_overcommit_as);
		sysctl_overcommit_memory = new_policy;
	} else {
		ret = proc_dointvec_minmax(table, write, buffer, lenp, ppos);
	}

	return ret;
}

static int overcommit_kbytes_handler(const struct ctl_table *table, int write,
				void *buffer, size_t *lenp, loff_t *ppos)
{
	int ret;

	ret = proc_doulongvec_minmax(table, write, buffer, lenp, ppos);
	if (ret == 0 && write)
		sysctl_overcommit_ratio = 0;
	return ret;
}

static const struct ctl_table util_sysctl_table[] = {
	{
		.procname	= "overcommit_memory",
		.data		= &sysctl_overcommit_memory,
		.maxlen		= sizeof(sysctl_overcommit_memory),
		.mode		= 0644,
		.proc_handler	= overcommit_policy_handler,
		.extra1		= SYSCTL_ZERO,
		.extra2		= SYSCTL_TWO,
	},
	{
		.procname	= "overcommit_ratio",
		.data		= &sysctl_overcommit_ratio,
		.maxlen		= sizeof(sysctl_overcommit_ratio),
		.mode		= 0644,
		.proc_handler	= overcommit_ratio_handler,
	},
	{
		.procname	= "overcommit_kbytes",
		.data		= &sysctl_overcommit_kbytes,
		.maxlen		= sizeof(sysctl_overcommit_kbytes),
		.mode		= 0644,
		.proc_handler	= overcommit_kbytes_handler,
	},
	{
		.procname	= "user_reserve_kbytes",
		.data		= &sysctl_user_reserve_kbytes,
		.maxlen		= sizeof(sysctl_user_reserve_kbytes),
		.mode		= 0644,
		.proc_handler	= proc_doulongvec_minmax,
	},
	{
		.procname	= "admin_reserve_kbytes",
		.data		= &sysctl_admin_reserve_kbytes,
		.maxlen		= sizeof(sysctl_admin_reserve_kbytes),
		.mode		= 0644,
		.proc_handler	= proc_doulongvec_minmax,
	},
};

static int __init init_vm_util_sysctls(void)
{
	register_sysctl_init("vm", util_sysctl_table);
	return 0;
}
subsys_initcall(init_vm_util_sysctls);
#endif /* CONFIG_SYSCTL */

/*
 * Committed memory limit enforced when OVERCOMMIT_NEVER policy is used
 */
unsigned long vm_commit_limit(void)
{
	unsigned long allowed;

	if (sysctl_overcommit_kbytes)
		allowed = sysctl_overcommit_kbytes >> (PAGE_SHIFT - 10);
	else
		allowed = ((totalram_pages() - hugetlb_total_pages())
			   * sysctl_overcommit_ratio / 100);
	allowed += total_swap_pages;

	return allowed;
}

/*
 * Make sure vm_committed_as in one cacheline and not cacheline shared with
 * other variables. It can be updated by several CPUs frequently.
 */
struct percpu_counter vm_committed_as ____cacheline_aligned_in_smp;

/*
 * The global memory commitment made in the system can be a metric
 * that can be used to drive ballooning decisions when Linux is hosted
 * as a guest. On Hyper-V, the host implements a policy engine for dynamically
 * balancing memory across competing virtual machines that are hosted.
 * Several metrics drive this policy engine including the guest reported
 * memory commitment.
 *
 * The time cost of this is very low for small platforms, and for big
 * platform like a 2S/36C/72T Skylake server, in worst case where
 * vm_committed_as's spinlock is under severe contention, the time cost
 * could be about 30~40 microseconds.
 */
unsigned long vm_memory_committed(void)
{
	return percpu_counter_sum_positive(&vm_committed_as);
}
EXPORT_SYMBOL_GPL(vm_memory_committed);

/*
 * Check that a process has enough memory to allocate a new virtual
 * mapping. 0 means there is enough memory for the allocation to
 * succeed and -ENOMEM implies there is not.
 *
 * We currently support three overcommit policies, which are set via the
 * vm.overcommit_memory sysctl.  See Documentation/mm/overcommit-accounting.rst
 *
 * Strict overcommit modes added 2002 Feb 26 by Alan Cox.
 * Additional code 2002 Jul 20 by Robert Love.
 *
 * cap_sys_admin is 1 if the process has admin privileges, 0 otherwise.
 *
 * Note this is a helper function intended to be used by LSMs which
 * wish to use this logic.
 */
int __vm_enough_memory(const struct mm_struct *mm, long pages, int cap_sys_admin)
{
	long allowed;
	unsigned long bytes_failed;

	vm_acct_memory(pages);

	/*
	 * Sometimes we want to use more memory than we have
	 */
	if (sysctl_overcommit_memory == OVERCOMMIT_ALWAYS)
		return 0;

	if (sysctl_overcommit_memory == OVERCOMMIT_GUESS) {
		if (pages > totalram_pages() + total_swap_pages)
			goto error;
		return 0;
	}

	allowed = vm_commit_limit();
	/*
	 * Reserve some for root
	 */
	if (!cap_sys_admin)
		allowed -= sysctl_admin_reserve_kbytes >> (PAGE_SHIFT - 10);

	/*
	 * Don't let a single process grow so big a user can't recover
	 */
	if (mm) {
		long reserve = sysctl_user_reserve_kbytes >> (PAGE_SHIFT - 10);

		allowed -= min_t(long, mm->total_vm / 32, reserve);
	}

	if (percpu_counter_read_positive(&vm_committed_as) < allowed)
		return 0;
error:
	bytes_failed = pages << PAGE_SHIFT;
	pr_warn_ratelimited("%s: pid: %d, comm: %s, bytes: %lu not enough memory for the allocation\n",
			    __func__, current->pid, current->comm, bytes_failed);
	vm_unacct_memory(pages);

	return -ENOMEM;
}

/**
 * get_cmdline() - copy the cmdline value to a buffer.
 * @task:     the task whose cmdline value to copy.
 * @buffer:   the buffer to copy to.
 * @buflen:   the length of the buffer. Larger cmdline values are truncated
 *            to this length.
 *
 * Return: the size of the cmdline field copied. Note that the copy does
 * not guarantee an ending NULL byte.
 */
int get_cmdline(struct task_struct *task, char *buffer, int buflen)
{
	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(task);
	unsigned long arg_start, arg_end, env_start, env_end;
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;	/* Shh! No looking before we're done */

	spin_lock(&mm->arg_lock);
	arg_start = mm->arg_start;
	arg_end = mm->arg_end;
	env_start = mm->env_start;
	env_end = mm->env_end;
	spin_unlock(&mm->arg_lock);

	len = arg_end - arg_start;

	if (len > buflen)
		len = buflen;

	res = access_process_vm(task, arg_start, buffer, len, FOLL_FORCE);

	/*
	 * If the nul at the end of args has been overwritten, then
	 * assume application is using setproctitle(3).
	 */
	if (res > 0 && buffer[res-1] != '\0' && len < buflen) {
		len = strnlen(buffer, res);
		if (len < res) {
			res = len;
		} else {
			len = env_end - env_start;
			if (len > buflen - res)
				len = buflen - res;
			res += access_process_vm(task, env_start,
						 buffer+res, len,
						 FOLL_FORCE);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	return res;
}

int __weak memcmp_pages(struct page *page1, struct page *page2)
{
	char *addr1, *addr2;
	int ret;

	addr1 = kmap_local_page(page1);
	addr2 = kmap_local_page(page2);
	ret = memcmp(addr1, addr2, PAGE_SIZE);
	kunmap_local(addr2);
	kunmap_local(addr1);
	return ret;
}

#ifdef CONFIG_PRINTK
/**
 * mem_dump_obj - Print available provenance information
 * @object: object for which to find provenance information.
 *
 * This function uses pr_cont(), so that the caller is expected to have
 * printed out whatever preamble is appropriate.  The provenance information
 * depends on the type of object and on how much debugging is enabled.
 * For example, for a slab-cache object, the slab name is printed, and,
 * if available, the return address and stack trace from the allocation
 * and last free path of that object.
 */
void mem_dump_obj(void *object)
{
	const char *type;

	if (kmem_dump_obj(object))
		return;

	if (vmalloc_dump_obj(object))
		return;

	if (is_vmalloc_addr(object))
		type = "vmalloc memory";
	else if (virt_addr_valid(object))
		type = "non-slab/vmalloc memory";
	else if (object == NULL)
		type = "NULL pointer";
	else if (object == ZERO_SIZE_PTR)
		type = "zero-size pointer";
	else
		type = "non-paged memory";

	pr_cont(" %s\n", type);
}
EXPORT_SYMBOL_GPL(mem_dump_obj);
#endif

/*
 * A driver might set a page logically offline -- PageOffline() -- and
 * turn the page inaccessible in the hypervisor; after that, access to page
 * content can be fatal.
 *
 * Some special PFN walkers -- i.e., /proc/kcore -- read content of random
 * pages after checking PageOffline(); however, these PFN walkers can race
 * with drivers that set PageOffline().
 *
 * page_offline_freeze()/page_offline_thaw() allows for a subsystem to
 * synchronize with such drivers, achieving that a page cannot be set
 * PageOffline() while frozen.
 *
 * page_offline_begin()/page_offline_end() is used by drivers that care about
 * such races when setting a page PageOffline().
 */
static DECLARE_RWSEM(page_offline_rwsem);

void page_offline_freeze(void)
{
	down_read(&page_offline_rwsem);
}

void page_offline_thaw(void)
{
	up_read(&page_offline_rwsem);
}

void page_offline_begin(void)
{
	down_write(&page_offline_rwsem);
}
EXPORT_SYMBOL(page_offline_begin);

void page_offline_end(void)
{
	up_write(&page_offline_rwsem);
}
EXPORT_SYMBOL(page_offline_end);

#ifndef flush_dcache_folio
void flush_dcache_folio(struct folio *folio)
{
	long i, nr = folio_nr_pages(folio);

	for (i = 0; i < nr; i++)
		flush_dcache_page(folio_page(folio, i));
}
EXPORT_SYMBOL(flush_dcache_folio);
#endif

/**
 * __compat_vma_mmap() - See description for compat_vma_mmap()
 * for details. This is the same operation, only with a specific file operations
 * struct which may or may not be the same as vma->vm_file->f_op.
 * @f_op: The file operations whose .mmap_prepare() hook is specified.
 * @file: The file which backs or will back the mapping.
 * @vma: The VMA to apply the .mmap_prepare() hook to.
 * Returns: 0 on success or error.
 */
int __compat_vma_mmap(const struct file_operations *f_op,
		struct file *file, struct vm_area_struct *vma)
{
	struct vm_area_desc desc = {
		.mm = vma->vm_mm,
		.file = file,
		.start = vma->vm_start,
		.end = vma->vm_end,

		.pgoff = vma->vm_pgoff,
		.vm_file = vma->vm_file,
		.vm_flags = vma->vm_flags,
		.page_prot = vma->vm_page_prot,

		.action.type = MMAP_NOTHING, /* Default */
	};
	int err;

	err = f_op->mmap_prepare(&desc);
	if (err)
		return err;

	mmap_action_prepare(&desc.action, &desc);
	set_vma_from_desc(vma, &desc);
	return mmap_action_complete(&desc.action, vma);
}
EXPORT_SYMBOL(__compat_vma_mmap);

/**
 * compat_vma_mmap() - Apply the file's .mmap_prepare() hook to an
 * existing VMA and execute any requested actions.
 * @file: The file which possesss an f_op->mmap_prepare() hook.
 * @vma: The VMA to apply the .mmap_prepare() hook to.
 *
 * Ordinarily, .mmap_prepare() is invoked directly upon mmap(). However, certain
 * stacked filesystems invoke a nested mmap hook of an underlying file.
 *
 * Until all filesystems are converted to use .mmap_prepare(), we must be
 * conservative and continue to invoke these stacked filesystems using the
 * deprecated .mmap() hook.
 *
 * However we have a problem if the underlying file system possesses an
 * .mmap_prepare() hook, as we are in a different context when we invoke the
 * .mmap() hook, already having a VMA to deal with.
 *
 * compat_vma_mmap() is a compatibility function that takes VMA state,
 * establishes a struct vm_area_desc descriptor, passes to the underlying
 * .mmap_prepare() hook and applies any changes performed by it.
 *
 * Once the conversion of filesystems is complete this function will no longer
 * be required and will be removed.
 *
 * Returns: 0 on success or error.
 */
int compat_vma_mmap(struct file *file, struct vm_area_struct *vma)
{
	return __compat_vma_mmap(file->f_op, file, vma);
}
EXPORT_SYMBOL(compat_vma_mmap);

static void set_ps_flags(struct page_snapshot *ps, const struct folio *folio,
			 const struct page *page)
{
	/*
	 * Only the first page of a high-order buddy page has PageBuddy() set.
	 * So we have to check manually whether this page is part of a high-
	 * order buddy page.
	 */
	if (PageBuddy(page))
		ps->flags |= PAGE_SNAPSHOT_PG_BUDDY;
	else if (page_count(page) == 0 && is_free_buddy_page(page))
		ps->flags |= PAGE_SNAPSHOT_PG_BUDDY;

	if (folio_test_idle(folio))
		ps->flags |= PAGE_SNAPSHOT_PG_IDLE;
}

/**
 * snapshot_page() - Create a snapshot of a struct page
 * @ps: Pointer to a struct page_snapshot to store the page snapshot
 * @page: The page to snapshot
 *
 * Create a snapshot of the page and store both its struct page and struct
 * folio representations in @ps.
 *
 * A snapshot is marked as "faithful" if the compound state of @page was
 * stable and allowed safe reconstruction of the folio representation. In
 * rare cases where this is not possible (e.g. due to folio splitting),
 * snapshot_page() falls back to treating @page as a single page and the
 * snapshot is marked as "unfaithful". The snapshot_page_is_faithful()
 * helper can be used to check for this condition.
 */
void snapshot_page(struct page_snapshot *ps, const struct page *page)
{
	unsigned long head, nr_pages = 1;
	struct folio *foliop;
	int loops = 5;

	ps->pfn = page_to_pfn(page);
	ps->flags = PAGE_SNAPSHOT_FAITHFUL;

again:
	memset(&ps->folio_snapshot, 0, sizeof(struct folio));
	memcpy(&ps->page_snapshot, page, sizeof(*page));
	head = ps->page_snapshot.compound_head;
	if ((head & 1) == 0) {
		ps->idx = 0;
		foliop = (struct folio *)&ps->page_snapshot;
		if (!folio_test_large(foliop)) {
			set_ps_flags(ps, page_folio(page), page);
			memcpy(&ps->folio_snapshot, foliop,
			       sizeof(struct page));
			return;
		}
		foliop = (struct folio *)page;
	} else {
		foliop = (struct folio *)(head - 1);
		ps->idx = folio_page_idx(foliop, page);
	}

	if (ps->idx < MAX_FOLIO_NR_PAGES) {
		memcpy(&ps->folio_snapshot, foliop, 2 * sizeof(struct page));
		nr_pages = folio_nr_pages(&ps->folio_snapshot);
		if (nr_pages > 1)
			memcpy(&ps->folio_snapshot.__page_2, &foliop->__page_2,
			       sizeof(struct page));
		set_ps_flags(ps, foliop, page);
	}

	if (ps->idx > nr_pages) {
		if (loops-- > 0)
			goto again;
		clear_compound_head(&ps->page_snapshot);
		foliop = (struct folio *)&ps->page_snapshot;
		memcpy(&ps->folio_snapshot, foliop, sizeof(struct page));
		ps->flags = 0;
		ps->idx = 0;
	}
}

static int mmap_action_finish(struct mmap_action *action,
		const struct vm_area_struct *vma, int err)
{
	/*
	 * If an error occurs, unmap the VMA altogether and return an error. We
	 * only clear the newly allocated VMA, since this function is only
	 * invoked if we do NOT merge, so we only clean up the VMA we created.
	 */
	if (err) {
		const size_t len = vma_pages(vma) << PAGE_SHIFT;

		do_munmap(current->mm, vma->vm_start, len, NULL);

		if (action->error_hook) {
			/* We may want to filter the error. */
			err = action->error_hook(err);

			/* The caller should not clear the error. */
			VM_WARN_ON_ONCE(!err);
		}
		return err;
	}

	if (action->success_hook)
		return action->success_hook(vma);

	return 0;
}

#ifdef CONFIG_MMU
/**
 * mmap_action_prepare - Perform preparatory setup for an VMA descriptor
 * action which need to be performed.
 * @desc: The VMA descriptor to prepare for @action.
 * @action: The action to perform.
 */
void mmap_action_prepare(struct mmap_action *action,
			 struct vm_area_desc *desc)
{
	switch (action->type) {
	case MMAP_NOTHING:
		break;
	case MMAP_REMAP_PFN:
		remap_pfn_range_prepare(desc, action->remap.start_pfn);
		break;
	case MMAP_IO_REMAP_PFN:
		io_remap_pfn_range_prepare(desc, action->remap.start_pfn,
					   action->remap.size);
		break;
	}
}
EXPORT_SYMBOL(mmap_action_prepare);

/**
 * mmap_action_complete - Execute VMA descriptor action.
 * @action: The action to perform.
 * @vma: The VMA to perform the action upon.
 *
 * Similar to mmap_action_prepare().
 *
 * Return: 0 on success, or error, at which point the VMA will be unmapped.
 */
int mmap_action_complete(struct mmap_action *action,
			 struct vm_area_struct *vma)
{
	int err = 0;

	switch (action->type) {
	case MMAP_NOTHING:
		break;
	case MMAP_REMAP_PFN:
		err = remap_pfn_range_complete(vma, action->remap.start,
				action->remap.start_pfn, action->remap.size,
				action->remap.pgprot);
		break;
	case MMAP_IO_REMAP_PFN:
		err = io_remap_pfn_range_complete(vma, action->remap.start,
				action->remap.start_pfn, action->remap.size,
				action->remap.pgprot);
		break;
	}

	return mmap_action_finish(action, vma, err);
}
EXPORT_SYMBOL(mmap_action_complete);
#else
void mmap_action_prepare(struct mmap_action *action,
			struct vm_area_desc *desc)
{
	switch (action->type) {
	case MMAP_NOTHING:
		break;
	case MMAP_REMAP_PFN:
	case MMAP_IO_REMAP_PFN:
		WARN_ON_ONCE(1); /* nommu cannot handle these. */
		break;
	}
}
EXPORT_SYMBOL(mmap_action_prepare);

int mmap_action_complete(struct mmap_action *action,
			struct vm_area_struct *vma)
{
	int err = 0;

	switch (action->type) {
	case MMAP_NOTHING:
		break;
	case MMAP_REMAP_PFN:
	case MMAP_IO_REMAP_PFN:
		WARN_ON_ONCE(1); /* nommu cannot handle this. */

		err = -EINVAL;
		break;
	}

	return mmap_action_finish(action, vma, err);
}
EXPORT_SYMBOL(mmap_action_complete);
#endif

#ifdef CONFIG_MMU
/**
 * folio_pte_batch - detect a PTE batch for a large folio
 * @folio: The large folio to detect a PTE batch for.
 * @ptep: Page table pointer for the first entry.
 * @pte: Page table entry for the first page.
 * @max_nr: The maximum number of table entries to consider.
 *
 * This is a simplified variant of folio_pte_batch_flags().
 *
 * Detect a PTE batch: consecutive (present) PTEs that map consecutive
 * pages of the same large folio in a single VMA and a single page table.
 *
 * All PTEs inside a PTE batch have the same PTE bits set, excluding the PFN,
 * the accessed bit, writable bit, dirt-bit and soft-dirty bit.
 *
 * ptep must map any page of the folio. max_nr must be at least one and
 * must be limited by the caller so scanning cannot exceed a single VMA and
 * a single page table.
 *
 * Return: the number of table entries in the batch.
 */
unsigned int folio_pte_batch(struct folio *folio, pte_t *ptep, pte_t pte,
		unsigned int max_nr)
{
	return folio_pte_batch_flags(folio, NULL, ptep, &pte, max_nr, 0);
}
#endif /* CONFIG_MMU */

#if defined(CONFIG_SPARSEMEM) && !defined(CONFIG_SPARSEMEM_VMEMMAP)
/**
 * page_range_contiguous - test whether the page range is contiguous
 * @page: the start of the page range.
 * @nr_pages: the number of pages in the range.
 *
 * Test whether the page range is contiguous, such that they can be iterated
 * naively, corresponding to iterating a contiguous PFN range.
 *
 * This function should primarily only be used for debug checks, or when
 * working with page ranges that are not naturally contiguous (e.g., pages
 * within a folio are).
 *
 * Returns true if contiguous, otherwise false.
 */
bool page_range_contiguous(const struct page *page, unsigned long nr_pages)
{
	const unsigned long start_pfn = page_to_pfn(page);
	const unsigned long end_pfn = start_pfn + nr_pages;
	unsigned long pfn;

	/*
	 * The memmap is allocated per memory section, so no need to check
	 * within the first section. However, we need to check each other
	 * spanned memory section once, making sure the first page in a
	 * section could similarly be reached by just iterating pages.
	 */
	for (pfn = ALIGN(start_pfn, PAGES_PER_SECTION);
	     pfn < end_pfn; pfn += PAGES_PER_SECTION)
		if (unlikely(page + (pfn - start_pfn) != pfn_to_page(pfn)))
			return false;
	return true;
}
EXPORT_SYMBOL(page_range_contiguous);
#endif
