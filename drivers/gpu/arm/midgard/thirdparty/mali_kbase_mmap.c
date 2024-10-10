/*
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 */

#include "linux/mman.h"
#include <linux/version_compat_defs.h>
#include <mali_kbase.h>
#include <mali_kbase_reg_track.h>

/* mali_kbase_mmap.c
 *
 * This file contains Linux specific implementation of
 * kbase_context_get_unmapped_area() interface.
 */

/**
 * shader_code_align_and_check() - Align the specified pointer according to shader code
 *                     requirement.
 *
 * @gap_end:           Highest possible start address for alignment. The caller must ensure
 *                     the input has already been properly aligned with info contained fields.
 * @info:              vm_unmapped_area_info structure passed, containing alignment, length
 *                     and limits for the allocation
 *
 * The function only undertakes the shader code alignment adjustment. It's the caller's
 * responsibility that the input value provided via gap_end has already been properly aligned
 * in compliance to the fields specified in the info structure. Irrespective the return result,
 * the value of the variable pointed by the pointer gap_end may have been decreased in
 * reaching the required alignment, but will not drop below info->low_limit.
 *
 * Return: true if gap_end is now aligned correctly, false otherwise
 */
static bool shader_code_align_and_check(unsigned long *gap_end, struct vm_unmapped_area_info *info)
{
	unsigned long align_adjust = (info->align_offset ? info->align_offset : info->length);
	unsigned long align_floor = info->low_limit + align_adjust;

	/* Check for 4GB address inner high-bit pattern, make adjustment if all zeros */
	if (0 == (*gap_end & BASE_MEM_MASK_4GB) && *gap_end >= align_floor)
		(*gap_end) -= align_adjust;
	if (0 == ((*gap_end + info->length) & BASE_MEM_MASK_4GB) && *gap_end >= align_floor)
		(*gap_end) -= align_adjust;

	return ((*gap_end & BASE_MEM_MASK_4GB) && ((*gap_end + info->length) & BASE_MEM_MASK_4GB));
}

/**
 * align_4gb_no_straddle() - Align the specified pointer not to straddle over a 4_GB boundary.
 *
 * @gap_end:           Highest possible start address for alignment. The caller must ensure
 *                     the input has already been properly aligned with info contained fields.
 * @info:              vm_unmapped_area_info structure passed, containing alignment, length
 *                     and limits for the allocation
 *
 * The function only undertakes the 4GB boundary alignment adjustment. It's the caller's
 * responsibility that the input value provided via gap_end has already been properly aligned
 * in compliance to the fields specified in the info structure.
 *
 * Return: true is always expected and the gap_end is aligned correctly, false can only
 *         be possible when the code has been wrongly modified.
 */
static bool align_4gb_no_straddle(unsigned long *gap_end, struct vm_unmapped_area_info *info)
{
	unsigned long start = *gap_end;
	unsigned long end = *gap_end + info->length;
	unsigned long mask = ~((unsigned long)U32_MAX);

	/* Check if 4GB boundary is straddled */
	if ((start & mask) != ((end - 1) & mask)) {
		unsigned long offset = end - (end & mask);
		/* This is to ensure that alignment doesn't get
		 * disturbed in an attempt to prevent straddling at
		 * 4GB boundary. The GPU VA is aligned to 2MB when the
		 * allocation size is > 2MB and there is enough CPU &
		 * GPU virtual space.
		 */
		unsigned long rounded_offset = ALIGN(offset, info->align_mask + 1);

		start -= rounded_offset;
		end -= rounded_offset;

		/* Patch gap_end to use new starting address for VA region */
		*gap_end = start;

		/* The preceding 4GB boundary shall not get straddled,
		 * even after accounting for the alignment, as the
		 * size of allocation is limited to 4GB and the initial
		 * start location was already aligned.
		 */
		if (WARN_ONCE((start & mask) != ((end - 1) & mask),
			      "Alignment unexpected straddles over 4GB boundary!"))
			return false;
	}

	return true;
}

#if (KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE) || !defined(__ANDROID_COMMON_KERNEL__)
/**
 * align_and_check() - Align the specified pointer to the provided alignment and
 *                     check that it is still in range. For Kernel versions below
 *                     6.1, it requires that the length of the alignment is already
 *                     extended by a worst-case alignment mask.
 * @gap_end:           Highest possible start address for allocation (end of gap in
 *                     address space)
 * @gap_start:         Start address of current memory area / gap in address space
 * @info:              vm_unmapped_area_info structure passed to caller, containing
 *                     alignment, length and limits for the allocation
 * @is_shader_code:    True if the allocation is for shader code (which has
 *                     additional alignment requirements)
 * @is_same_4gb_page:  True if the allocation needs to reside completely within
 *                     a 4GB chunk
 *
 * Return: true if gap_end is now aligned correctly and is still in range,
 *         false otherwise
 */
static bool align_and_check(unsigned long *gap_end, unsigned long gap_start,
			    struct vm_unmapped_area_info *info, bool is_shader_code,
			    bool is_same_4gb_page)
{
	/* Compute highest gap address at the desired alignment */
	*gap_end -= info->length;
	*gap_end -= (*gap_end - info->align_offset) & info->align_mask;

	if (is_shader_code) {
		if (!shader_code_align_and_check(gap_end, info))
			return false;
	} else if (is_same_4gb_page)
		if (!align_4gb_no_straddle(gap_end, info))
			return false;

	if ((*gap_end < info->low_limit) || (*gap_end < gap_start))
		return false;

	return true;
}
#endif

/**
 * kbase_unmapped_area_topdown() - allocates new areas top-down from
 *                                 below the stack limit.
 * @info:              Information about the memory area to allocate.
 * @is_shader_code:    Boolean which denotes whether the allocated area is
 *                      intended for the use by shader core in which case a
 *                      special alignment requirements apply.
 * @is_same_4gb_page: Boolean which indicates whether the allocated area needs
 *                    to reside completely within a 4GB chunk.
 *
 * The unmapped_area_topdown() function in the Linux kernel is not exported
 * using EXPORT_SYMBOL_GPL macro. To allow us to call this function from a
 * module and also make use of the fact that some of the requirements for
 * the unmapped area are known in advance, we implemented an extended version
 * of this function and prefixed it with 'kbase_'.
 *
 * The difference in the call parameter list comes from the fact that
 * kbase_unmapped_area_topdown() is called with additional parameters which
 * are provided to indicate whether the allocation is for a shader core memory,
 * which has additional alignment requirements, and whether the allocation can
 * straddle a 4GB boundary.
 *
 * The modification of the original Linux function lies in how the computation
 * of the highest gap address at the desired alignment is performed once the
 * gap with desirable properties is found. For this purpose a special function
 * is introduced (@ref align_and_check()) which beside computing the gap end
 * at the desired alignment also performs additional alignment checks for the
 * case when the memory is executable shader core memory, for which it is
 * ensured that the gap does not end on a 4GB boundary, and for the case when
 * memory needs to be confined within a 4GB chunk.
 *
 * Return: address of the found gap end (high limit) if area is found;
 *         -ENOMEM if search is unsuccessful
 */

static unsigned long kbase_unmapped_area_topdown(struct vm_unmapped_area_info *info,
						 bool is_shader_code, bool is_same_4gb_page)
{
#if (KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE)
	struct mm_struct *mm = current->mm;
	struct vm_area_struct *vma;
	unsigned long length, low_limit, high_limit, gap_start, gap_end;

	/* Adjust search length to account for worst case alignment overhead */
	length = info->length + info->align_mask;
	if (length < info->length)
		return -ENOMEM;

	/*
	 * Adjust search limits by the desired length.
	 * See implementation comment at top of unmapped_area().
	 */
	gap_end = info->high_limit;
	if (gap_end < length)
		return -ENOMEM;
	high_limit = gap_end - length;

	if (info->low_limit > high_limit)
		return -ENOMEM;
	low_limit = info->low_limit + length;

	/* Check highest gap, which does not precede any rbtree node */
	gap_start = mm->highest_vm_end;
	if (gap_start <= high_limit) {
		if (align_and_check(&gap_end, gap_start, info, is_shader_code, is_same_4gb_page))
			return gap_end;
	}

	/* Check if rbtree root looks promising */
	if (RB_EMPTY_ROOT(&mm->mm_rb))
		return -ENOMEM;
	vma = rb_entry(mm->mm_rb.rb_node, struct vm_area_struct, vm_rb);
	if (vma->rb_subtree_gap < length)
		return -ENOMEM;

	while (true) {
		/* Visit right subtree if it looks promising */
		gap_start = vma->vm_prev ? vma->vm_prev->vm_end : 0;
		if (gap_start <= high_limit && vma->vm_rb.rb_right) {
			struct vm_area_struct *right =
				rb_entry(vma->vm_rb.rb_right, struct vm_area_struct, vm_rb);
			if (right->rb_subtree_gap >= length) {
				vma = right;
				continue;
			}
		}

check_current:
		/* Check if current node has a suitable gap */
		gap_end = vma->vm_start;
		if (gap_end < low_limit)
			return -ENOMEM;
		if (gap_start <= high_limit && gap_end - gap_start >= length) {
			/* We found a suitable gap. Clip it with the original
			 * high_limit.
			 */
			if (gap_end > info->high_limit)
				gap_end = info->high_limit;

			if (align_and_check(&gap_end, gap_start, info, is_shader_code,
					    is_same_4gb_page))
				return gap_end;
		}

		/* Visit left subtree if it looks promising */
		if (vma->vm_rb.rb_left) {
			struct vm_area_struct *left =
				rb_entry(vma->vm_rb.rb_left, struct vm_area_struct, vm_rb);
			if (left->rb_subtree_gap >= length) {
				vma = left;
				continue;
			}
		}

		/* Go back up the rbtree to find next candidate node */
		while (true) {
			struct rb_node *prev = &vma->vm_rb;

			if (!rb_parent(prev))
				return -ENOMEM;
			vma = rb_entry(rb_parent(prev), struct vm_area_struct, vm_rb);
			if (prev == vma->vm_rb.rb_right) {
				gap_start = vma->vm_prev ? vma->vm_prev->vm_end : 0;
				goto check_current;
			}
		}
	}
#else /* KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE */
#ifdef __ANDROID_COMMON_KERNEL__
	struct vm_unmapped_area_info tmp_info = *info;
	unsigned long length;

	tmp_info.flags |= VM_UNMAPPED_AREA_TOPDOWN;
	if (!(is_shader_code || is_same_4gb_page))
		return vm_unmapped_area(&tmp_info);

	length = info->length + info->align_mask;

	/* Due to additional alignment requirement, shader_code or same_4gb_page
	 * needs iterations for alignment search and confirmation check.
	 */
	while (true) {
		unsigned long saved_high_lmt = tmp_info.high_limit;
		unsigned long gap_end, start, rev_high_limit;

		gap_end = vm_unmapped_area(&tmp_info);
		if (IS_ERR_VALUE(gap_end))
			return gap_end;

		start = gap_end;
		if (is_shader_code) {
			bool shader_code_aligned;
			unsigned long align_cmp_ref;

			while (true) {
				/* Save the start value for progress check. the loop needs
				 * to end if the alignment can't progress any further.
				 * In summary, the loop ends condition here is either:
				 *  1. shader_code_aligned is true; or
				 *  2. align_cmp_ref == gap_end.
				 */
				align_cmp_ref = gap_end;

				shader_code_aligned =
					shader_code_align_and_check(&gap_end, &tmp_info);
				if (shader_code_aligned || (align_cmp_ref == gap_end))
					break;
			}

			if (shader_code_aligned) {
				if (start == gap_end)
					return gap_end;

				rev_high_limit = gap_end + length;
			} else
				break;
		} else {
			/* must be same_4gb_page case */
			if (likely(align_4gb_no_straddle(&gap_end, &tmp_info))) {
				if (start == gap_end)
					return gap_end;

				rev_high_limit = gap_end + length;
			} else
				break;
		}

		if (rev_high_limit < info->low_limit)
			break;

		if (WARN_ONCE(rev_high_limit >= saved_high_lmt,
			      "Unexpected recurring high_limit in search, %lx => %lx\n"
			      "\tinfo-input: limit=[%lx, %lx], mask=%lx, len=%lx\n",
			      saved_high_lmt, rev_high_limit, info->low_limit, info->high_limit,
			      info->align_mask, info->length))
			rev_high_limit = saved_high_lmt -
					 (info->align_offset ? info->align_offset : info->length);

		/* Repeat the search with a decreasing rev_high_limit */
		tmp_info.high_limit = rev_high_limit;
	}
#else /* __ANDROID_COMMON_KERNEL__ */
	unsigned long length, high_limit;

	MA_STATE(mas, &current->mm->mm_mt, 0, 0);

	/* Adjust search length to account for worst case alignment overhead */
	length = info->length + info->align_mask;
	if (length < info->length)
		return -ENOMEM;

	high_limit = info->high_limit;
	if ((high_limit - info->low_limit) < length)
		return -ENOMEM;

	while (true) {
		unsigned long gap_start, gap_end;
		unsigned long saved_high_lmt = high_limit;

		if (mas_empty_area_rev(&mas, info->low_limit, high_limit - 1, length))
			return -ENOMEM;

		gap_end = mas.last + 1;
		gap_start = mas.index;

		if (align_and_check(&gap_end, gap_start, info, is_shader_code, is_same_4gb_page))
			return gap_end;

		if (gap_end < info->low_limit)
			return -ENOMEM;


		/* Adjust next search high limit */
		high_limit = gap_end + length;

		if (WARN_ONCE(high_limit >= saved_high_lmt,
			      "Unexpected recurring high_limit in search, %lx => %lx\n"
			      "\tinfo-input: limit=[%lx, %lx], mask=%lx, len=%lx\n",
			      saved_high_lmt, high_limit, info->low_limit, info->high_limit,
			      info->align_mask, info->length))
			high_limit = saved_high_lmt -
				     (info->align_offset ? info->align_offset : info->length);
		mas_reset(&mas);
	}
#endif /* __ANDROID_COMMON_KERNEL__ */
#endif /* KERNEL_VERSION(6, 1, 0) > LINUX_VERSION_CODE */
	return -ENOMEM;
}

/* This function is based on Linux kernel's arch_get_unmapped_area, but
 * simplified slightly. Modifications come from the fact that some values
 * about the memory area are known in advance.
 */
unsigned long kbase_context_get_unmapped_area(struct kbase_context *const kctx,
					      const unsigned long addr, const unsigned long len,
					      const unsigned long pgoff, const unsigned long flags)
{
	struct mm_struct *mm = current->mm;
	struct vm_unmapped_area_info info;
	unsigned long align_offset = 0;
	unsigned long align_mask = 0;
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	unsigned long high_limit = arch_get_mmap_base(addr, mm->mmap_base);
	unsigned long low_limit = max_t(unsigned long, PAGE_SIZE, kbase_mmap_min_addr);
#else
	unsigned long high_limit = mm->mmap_base;
	unsigned long low_limit = PAGE_SIZE;
#endif
	unsigned int cpu_va_bits = BITS_PER_LONG;
	unsigned int gpu_pc_bits = kctx->kbdev->gpu_props.log2_program_counter_size;
	bool is_shader_code = false;
	bool is_same_4gb_page = false;
	unsigned long ret;

	/* the 'nolock' form is used here:
	 * - the base_pfn of the SAME_VA zone does not change
	 * - in normal use, va_size_pages is constant once the first allocation
	 *   begins
	 *
	 * However, in abnormal use this function could be processing whilst
	 * another new zone is being setup in a different thread (e.g. to
	 * borrow part of the SAME_VA zone). In the worst case, this path may
	 * witness a higher SAME_VA end_pfn than the code setting up the new
	 * zone.
	 *
	 * This is safe because once we reach the main allocation functions,
	 * we'll see the updated SAME_VA end_pfn and will determine that there
	 * is no free region at the address found originally by too large a
	 * same_va_end_addr here, and will fail the allocation gracefully.
	 */
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get_nolock(kctx, SAME_VA_ZONE);
	u64 same_va_end_addr = kbase_reg_zone_end_pfn(zone) << PAGE_SHIFT;
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
	const unsigned long mmap_end = arch_get_mmap_end(addr, len, flags);

	/* requested length too big for entire address space */
	if (len > mmap_end - kbase_mmap_min_addr)
		return -ENOMEM;
#endif

	/* err on fixed address */
	if ((flags & MAP_FIXED) || addr)
		return -EINVAL;

#if IS_ENABLED(CONFIG_64BIT)
	/* too big? */
	if (len > TASK_SIZE - SZ_2M)
		return -ENOMEM;

	if (!kbase_ctx_flag(kctx, KCTX_COMPAT)) {
		high_limit = min_t(unsigned long, high_limit, same_va_end_addr);

		/* If there's enough (> 33 bits) of GPU VA space, align
		 * to 2MB boundaries.
		 */
		if (kctx->kbdev->gpu_props.mmu.va_bits > 33) {
			if (len >= SZ_2M) {
				align_offset = SZ_2M;
				align_mask = SZ_2M - 1;
			}
		}

		low_limit = SZ_2M;
	} else {
		cpu_va_bits = 32;
	}
#endif /* CONFIG_64BIT */
	if ((PFN_DOWN(BASE_MEM_COOKIE_BASE) <= pgoff) &&
	    (PFN_DOWN(BASE_MEM_FIRST_FREE_ADDRESS) > pgoff)) {
		int cookie = pgoff - PFN_DOWN(BASE_MEM_COOKIE_BASE);
		struct kbase_va_region *reg;

		/* Need to hold gpu vm lock when using reg */
		kbase_gpu_vm_lock(kctx);
		reg = kctx->pending_regions[cookie];
		if (!reg) {
			kbase_gpu_vm_unlock(kctx);
			return -EINVAL;
		}
		if (!(reg->flags & KBASE_REG_GPU_NX)) {
			if (cpu_va_bits > gpu_pc_bits) {
				align_offset = 1ULL << gpu_pc_bits;
				align_mask = align_offset - 1;
				is_shader_code = true;
			}
#if !MALI_USE_CSF
		} else if (reg->flags & KBASE_REG_TILER_ALIGN_TOP) {
			unsigned long extension_bytes =
				(unsigned long)(reg->extension << PAGE_SHIFT);
			/* kbase_check_alloc_sizes() already satisfies
			 * these checks, but they're here to avoid
			 * maintenance hazards due to the assumptions
			 * involved
			 */
			WARN_ON(reg->extension > (ULONG_MAX >> PAGE_SHIFT));
			WARN_ON(reg->initial_commit > (ULONG_MAX >> PAGE_SHIFT));
			WARN_ON(!is_power_of_2(extension_bytes));
			align_mask = extension_bytes - 1;
			align_offset = extension_bytes - (reg->initial_commit << PAGE_SHIFT);
#endif /* !MALI_USE_CSF */
		} else if (reg->flags & KBASE_REG_GPU_VA_SAME_4GB_PAGE) {
			is_same_4gb_page = true;
		}
		kbase_gpu_vm_unlock(kctx);
	} else if (!IS_ENABLED(CONFIG_64BIT)) {
		return kbase_mm_get_unmapped_area_helper(mm, kctx->filp, addr, len, pgoff, flags);
	}

	info.flags = 0;
	info.length = len;
	info.low_limit = low_limit;
	info.high_limit = high_limit;
	info.align_offset = align_offset;
	info.align_mask = align_mask;

	ret = kbase_unmapped_area_topdown(&info, is_shader_code, is_same_4gb_page);

	if (IS_ERR_VALUE(ret) && high_limit == mm->mmap_base && high_limit < same_va_end_addr) {
#if (KERNEL_VERSION(6, 1, 0) <= LINUX_VERSION_CODE)
		/* Retry above TASK_UNMAPPED_BASE */
		info.low_limit = TASK_UNMAPPED_BASE;
		info.high_limit = min_t(u64, mmap_end, same_va_end_addr);
#else
		/* Retry above mmap_base */
		info.low_limit = mm->mmap_base;
		info.high_limit = min_t(u64, TASK_SIZE, same_va_end_addr);
#endif

		ret = kbase_unmapped_area_topdown(&info, is_shader_code, is_same_4gb_page);
	}

	return ret;
}
