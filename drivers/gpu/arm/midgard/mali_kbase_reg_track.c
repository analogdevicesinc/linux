// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2023-2024 ARM Limited. All rights reserved.
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
#include <mali_kbase_reg_track.h>
#include <mali_kbase.h>

unsigned long kbase_zone_to_bits(enum kbase_memory_zone zone)
{
	return ((((unsigned long)zone) & ((1 << KBASE_REG_ZONE_BITS) - 1ul))
		<< KBASE_REG_ZONE_SHIFT);
}

enum kbase_memory_zone kbase_bits_to_zone(unsigned long zone_bits)
{
	return (enum kbase_memory_zone)(((zone_bits)&KBASE_REG_ZONE_MASK) >> KBASE_REG_ZONE_SHIFT);
}
KBASE_EXPORT_TEST_API(kbase_bits_to_zone);

char *kbase_reg_zone_get_name(enum kbase_memory_zone zone)
{
	switch (zone) {
	case SAME_VA_ZONE:
		return "SAME_VA";
	case CUSTOM_VA_ZONE:
		return "CUSTOM_VA";
	case EXEC_VA_ZONE:
		return "EXEC_VA";
#if MALI_USE_CSF
	case MCU_SHARED_ZONE:
		return "MCU_SHARED";
	case EXEC_FIXED_VA_ZONE:
		return "EXEC_FIXED_VA";
	case FIXED_VA_ZONE:
		return "FIXED_VA";
#endif
	default:
		return NULL;
	}
}
KBASE_EXPORT_TEST_API(kbase_reg_zone_get_name);

struct kbase_reg_zone *kbase_ctx_reg_zone_get_nolock(struct kbase_context *kctx,
						     enum kbase_memory_zone zone)
{
	WARN_ON(!kbase_is_ctx_reg_zone(zone));

	return &kctx->reg_zone[zone];
}

struct kbase_reg_zone *kbase_ctx_reg_zone_get(struct kbase_context *kctx,
					      enum kbase_memory_zone zone)
{
	lockdep_assert_held(&kctx->reg_lock);
	return kbase_ctx_reg_zone_get_nolock(kctx, zone);
}
KBASE_EXPORT_TEST_API(kbase_ctx_reg_zone_get);

static size_t kbase_get_num_cpu_va_bits(struct kbase_context *kctx)
{
#if defined(CONFIG_ARM64)
	/* VA_BITS can be as high as 48 bits, but all bits are available for
	 * both user and kernel.
	 */
	size_t cpu_va_bits = VA_BITS;
#elif defined(CONFIG_X86_64)
	/* x86_64 can access 48 bits of VA, but the 48th is used to denote
	 * kernel (1) vs userspace (0), so the max here is 47.
	 */
	size_t cpu_va_bits = 47;
#elif defined(CONFIG_ARM) || defined(CONFIG_X86_32)
	size_t cpu_va_bits = sizeof(void *) * BITS_PER_BYTE;
#else
#error "Unknown CPU VA width for this architecture"
#endif

	if (kbase_ctx_compat_mode(kctx))
		cpu_va_bits = 32;

	return cpu_va_bits;
}

/**
 * kbase_gpu_pfn_to_rbtree - find the rb-tree tracking the region with the indicated GPU
 *                           page frame number
 * @kctx: kbase context
 * @gpu_pfn: GPU PFN address
 *
 * Context: any context.
 *
 * Return: reference to the rb-tree root, NULL if not found
 */
static struct rb_root *kbase_gpu_pfn_to_rbtree(struct kbase_context *kctx, u64 gpu_pfn)
{
	enum kbase_memory_zone zone_idx;
	struct kbase_reg_zone *zone;

	for (zone_idx = 0; zone_idx < CONTEXT_ZONE_MAX; zone_idx++) {
		zone = &kctx->reg_zone[zone_idx];
		if ((gpu_pfn >= zone->base_pfn) && (gpu_pfn < kbase_reg_zone_end_pfn(zone)))
			return &zone->reg_rbtree;
	}

	return NULL;
}

/* This function inserts a region into the tree. */
static void kbase_region_tracker_insert(struct kbase_va_region *new_reg)
{
	const u64 start_pfn = new_reg->start_pfn;
	struct rb_node **link = NULL;
	struct rb_node *parent = NULL;
	struct rb_root *rbtree = NULL;

	rbtree = new_reg->rbtree;

	link = &(rbtree->rb_node);
	/* Find the right place in the tree using tree search */
	while (*link) {
		struct kbase_va_region *old_reg;

		parent = *link;
		old_reg = rb_entry(parent, struct kbase_va_region, rblink);

		/* RBTree requires no duplicate entries. */
		KBASE_DEBUG_ASSERT(old_reg->start_pfn != start_pfn);

		if (old_reg->start_pfn > start_pfn)
			link = &(*link)->rb_left;
		else
			link = &(*link)->rb_right;
	}

	/* Put the new node there, and rebalance tree */
	rb_link_node(&(new_reg->rblink), parent, link);

	rb_insert_color(&(new_reg->rblink), rbtree);
}

static struct kbase_va_region *find_region_enclosing_range_rbtree(struct rb_root *rbtree,
								  u64 start_pfn, size_t nr_pages)
{
	struct rb_node *rbnode;
	struct kbase_va_region *reg;
	const u64 end_pfn = start_pfn + nr_pages;

	rbnode = rbtree->rb_node;

	while (rbnode) {
		u64 tmp_start_pfn, tmp_end_pfn;

		reg = rb_entry(rbnode, struct kbase_va_region, rblink);
		tmp_start_pfn = reg->start_pfn;
		tmp_end_pfn = reg->start_pfn + reg->nr_pages;

		/* If start is lower than this, go left. */
		if (start_pfn < tmp_start_pfn)
			rbnode = rbnode->rb_left;
		/* If end is higher than this, then go right. */
		else if (end_pfn > tmp_end_pfn)
			rbnode = rbnode->rb_right;
		else /* Enclosing */
			return reg;
	}

	return NULL;
}

static struct kbase_va_region *kbase_find_region_enclosing_address(struct rb_root *rbtree,
								   u64 gpu_addr)
{
	const u64 gpu_pfn = gpu_addr >> PAGE_SHIFT;
	struct rb_node *rbnode;
	struct kbase_va_region *reg;

	rbnode = rbtree->rb_node;

	while (rbnode) {
		u64 tmp_start_pfn, tmp_end_pfn;

		reg = rb_entry(rbnode, struct kbase_va_region, rblink);
		tmp_start_pfn = reg->start_pfn;
		tmp_end_pfn = reg->start_pfn + reg->nr_pages;

		/* If start is lower than this, go left. */
		if (gpu_pfn < tmp_start_pfn)
			rbnode = rbnode->rb_left;
		/* If end is higher than this, then go right. */
		else if (gpu_pfn >= tmp_end_pfn)
			rbnode = rbnode->rb_right;
		else /* Enclosing */
			return reg;
	}

	return NULL;
}

/* Find region enclosing given address. */
struct kbase_va_region *
kbase_region_tracker_find_region_enclosing_address(struct kbase_context *kctx, u64 gpu_addr)
{
	u64 gpu_pfn = gpu_addr >> PAGE_SHIFT;
	struct rb_root *rbtree = NULL;

	KBASE_DEBUG_ASSERT(kctx != NULL);

	lockdep_assert_held(&kctx->reg_lock);

	rbtree = kbase_gpu_pfn_to_rbtree(kctx, gpu_pfn);
	if (unlikely(!rbtree))
		return NULL;

	return kbase_find_region_enclosing_address(rbtree, gpu_addr);
}
KBASE_EXPORT_TEST_API(kbase_region_tracker_find_region_enclosing_address);

static struct kbase_va_region *kbase_find_region_base_address(struct rb_root *rbtree, u64 gpu_addr)
{
	u64 gpu_pfn = gpu_addr >> PAGE_SHIFT;
	struct rb_node *rbnode = NULL;
	struct kbase_va_region *reg = NULL;

	rbnode = rbtree->rb_node;

	while (rbnode) {
		reg = rb_entry(rbnode, struct kbase_va_region, rblink);
		if (reg->start_pfn > gpu_pfn)
			rbnode = rbnode->rb_left;
		else if (reg->start_pfn < gpu_pfn)
			rbnode = rbnode->rb_right;
		else
			return reg;
	}

	return NULL;
}

/* Find region with given base address */
struct kbase_va_region *kbase_region_tracker_find_region_base_address(struct kbase_context *kctx,
								      u64 gpu_addr)
{
	const u64 gpu_pfn = gpu_addr >> PAGE_SHIFT;
	struct rb_root *rbtree = NULL;

	lockdep_assert_held(&kctx->reg_lock);

	rbtree = kbase_gpu_pfn_to_rbtree(kctx, gpu_pfn);
	if (unlikely(!rbtree))
		return NULL;

	return kbase_find_region_base_address(rbtree, gpu_addr);
}
KBASE_EXPORT_TEST_API(kbase_region_tracker_find_region_base_address);

/* Find region meeting given requirements */
static struct kbase_va_region *
kbase_region_tracker_find_region_meeting_reqs(struct kbase_va_region *reg_reqs, size_t nr_pages,
					      size_t align_offset, size_t align_mask,
					      u64 *out_start_pfn)
{
	struct rb_node *rbnode = NULL;
	struct kbase_va_region *reg = NULL;
	struct rb_root *rbtree = NULL;

	/* Note that this search is a linear search, as we do not have a target
	 * address in mind, so does not benefit from the rbtree search
	 */
	rbtree = reg_reqs->rbtree;

	for (rbnode = rb_first(rbtree); rbnode; rbnode = rb_next(rbnode)) {
		reg = rb_entry(rbnode, struct kbase_va_region, rblink);
		if ((reg->nr_pages >= nr_pages) && (reg->flags & KBASE_REG_FREE)) {
			/* Check alignment */
			u64 start_pfn = reg->start_pfn;

			/* When align_offset == align, this sequence is
			 * equivalent to:
			 *   (start_pfn + align_mask) & ~(align_mask)
			 *
			 * Otherwise, it aligns to n*align + offset, for the
			 * lowest value n that makes this still >start_pfn
			 */
			start_pfn += align_mask;
			start_pfn -= (start_pfn - align_offset) & (align_mask);

			if (!(reg_reqs->flags & KBASE_REG_GPU_NX)) {
				/* Can't end at 4GB boundary */
				if (0 == ((start_pfn + nr_pages) & BASE_MEM_PFN_MASK_4GB))
					start_pfn += align_offset;

				/* Can't start at 4GB boundary */
				if (0 == (start_pfn & BASE_MEM_PFN_MASK_4GB))
					start_pfn += align_offset;

				if (!((start_pfn + nr_pages) & BASE_MEM_PFN_MASK_4GB) ||
				    !(start_pfn & BASE_MEM_PFN_MASK_4GB))
					continue;
			} else if (reg_reqs->flags & KBASE_REG_GPU_VA_SAME_4GB_PAGE) {
				u64 end_pfn = start_pfn + nr_pages - 1;

				if ((start_pfn & ~BASE_MEM_PFN_MASK_4GB) !=
				    (end_pfn & ~BASE_MEM_PFN_MASK_4GB))
					start_pfn = end_pfn & ~BASE_MEM_PFN_MASK_4GB;
			}

			if ((start_pfn >= reg->start_pfn) &&
			    (start_pfn <= (reg->start_pfn + reg->nr_pages - 1)) &&
			    ((start_pfn + nr_pages - 1) <= (reg->start_pfn + reg->nr_pages - 1))) {
				*out_start_pfn = start_pfn;
				return reg;
			}
		}
	}

	return NULL;
}

/**
 * kbase_remove_va_region - Remove a region object from the global list.
 *
 * @kbdev: The kbase device
 * @reg: Region object to remove
 *
 * The region reg is removed, possibly by merging with other free and
 * compatible adjacent regions.  It must be called with the context
 * region lock held. The associated memory is not released (see
 * kbase_free_alloced_region). Internal use only.
 */
void kbase_remove_va_region(struct kbase_device *kbdev, struct kbase_va_region *reg)
{
	struct rb_node *rbprev;
	struct kbase_reg_zone *zone = container_of(reg->rbtree, struct kbase_reg_zone, reg_rbtree);
	struct kbase_va_region *prev = NULL;
	struct rb_node *rbnext;
	struct kbase_va_region *next = NULL;
	struct rb_root *reg_rbtree = NULL;
	struct kbase_va_region *orig_reg = reg;

	int merged_front = 0;
	int merged_back = 0;

	reg_rbtree = reg->rbtree;

	if (WARN_ON(RB_EMPTY_ROOT(reg_rbtree)))
		return;

	/* Try to merge with the previous block first */
	rbprev = rb_prev(&(reg->rblink));
	if (rbprev) {
		prev = rb_entry(rbprev, struct kbase_va_region, rblink);
		if (prev->flags & KBASE_REG_FREE) {
			/* We're compatible with the previous VMA, merge with
			 * it, handling any gaps for robustness.
			 */
			u64 prev_end_pfn = prev->start_pfn + prev->nr_pages;

			WARN_ON((kbase_bits_to_zone(prev->flags)) !=
				(kbase_bits_to_zone(reg->flags)));
			if (!WARN_ON(reg->start_pfn < prev_end_pfn))
				prev->nr_pages += reg->start_pfn - prev_end_pfn;
			prev->nr_pages += reg->nr_pages;
			rb_erase(&(reg->rblink), reg_rbtree);
			reg = prev;
			merged_front = 1;
		}
	}

	/* Try to merge with the next block second */
	/* Note we do the lookup here as the tree may have been rebalanced. */
	rbnext = rb_next(&(reg->rblink));
	if (rbnext) {
		next = rb_entry(rbnext, struct kbase_va_region, rblink);
		if (next->flags & KBASE_REG_FREE) {
			/* We're compatible with the next VMA, merge with it,
			 * handling any gaps for robustness.
			 */
			u64 reg_end_pfn = reg->start_pfn + reg->nr_pages;

			WARN_ON((kbase_bits_to_zone(next->flags)) !=
				(kbase_bits_to_zone(reg->flags)));
			if (!WARN_ON(next->start_pfn < reg_end_pfn))
				next->nr_pages += next->start_pfn - reg_end_pfn;
			next->start_pfn = reg->start_pfn;
			next->nr_pages += reg->nr_pages;
			rb_erase(&(reg->rblink), reg_rbtree);
			merged_back = 1;
		}
	}

	if (merged_front && merged_back) {
		/* We already merged with prev, free it */
		kfree(reg);
	} else if (!(merged_front || merged_back)) {
		/* If we failed to merge then we need to add a new block */

		/*
		 * We didn't merge anything. Try to add a new free
		 * placeholder, and in any case, remove the original one.
		 */
		struct kbase_va_region *free_reg;

		free_reg = kbase_alloc_free_region(zone, reg->start_pfn, reg->nr_pages);
		if (!free_reg) {
			/* In case of failure, we cannot allocate a replacement
			 * free region, so we will be left with a 'gap' in the
			 * region tracker's address range (though, the rbtree
			 * will itself still be correct after erasing
			 * 'reg').
			 *
			 * The gap will be rectified when an adjacent region is
			 * removed by one of the above merging paths. Other
			 * paths will gracefully fail to allocate if they try
			 * to allocate in the gap.
			 *
			 * There is nothing that the caller can do, since free
			 * paths must not fail. The existing 'reg' cannot be
			 * repurposed as the free region as callers must have
			 * freedom of use with it by virtue of it being owned
			 * by them, not the region tracker insert/remove code.
			 */
			dev_warn(
				kbdev->dev,
				"Could not alloc a replacement free region for 0x%.16llx..0x%.16llx",
				(unsigned long long)reg->start_pfn << PAGE_SHIFT,
				(unsigned long long)(reg->start_pfn + reg->nr_pages) << PAGE_SHIFT);
			rb_erase(&(reg->rblink), reg_rbtree);

			goto out;
		}
		rb_replace_node(&(reg->rblink), &(free_reg->rblink), reg_rbtree);
	}

	/* This operation is always safe because the function never frees
	 * the region. If the region has been merged to both front and back,
	 * then it's the previous region that is supposed to be freed.
	 */
	orig_reg->start_pfn = 0;

out:
	return;
}
KBASE_EXPORT_TEST_API(kbase_remove_va_region);

/**
 * kbase_insert_va_region_nolock - Insert a VA region to the list,
 * replacing the existing one.
 *
 * @kbdev: The kbase device
 * @new_reg: The new region to insert
 * @at_reg: The region to replace
 * @start_pfn: The Page Frame Number to insert at
 * @nr_pages: The number of pages of the region
 *
 * Return: 0 on success, error code otherwise.
 */
static int kbase_insert_va_region_nolock(struct kbase_device *kbdev,
					 struct kbase_va_region *new_reg,
					 struct kbase_va_region *at_reg, u64 start_pfn,
					 size_t nr_pages)
{
	struct rb_root *reg_rbtree = NULL;
	struct kbase_reg_zone *zone =
		container_of(at_reg->rbtree, struct kbase_reg_zone, reg_rbtree);
	int err = 0;

	CSTD_UNUSED(kbdev);

	reg_rbtree = at_reg->rbtree;

	/* Must be a free region */
	KBASE_DEBUG_ASSERT((at_reg->flags & KBASE_REG_FREE) != 0);
	/* start_pfn should be contained within at_reg */
	KBASE_DEBUG_ASSERT((start_pfn >= at_reg->start_pfn) &&
			   (start_pfn < at_reg->start_pfn + at_reg->nr_pages));
	/* at least nr_pages from start_pfn should be contained within at_reg */
	KBASE_DEBUG_ASSERT(start_pfn + nr_pages <= at_reg->start_pfn + at_reg->nr_pages);
	/* having at_reg means the rb_tree should not be empty */
	if (WARN_ON(RB_EMPTY_ROOT(reg_rbtree)))
		return -ENOMEM;

	new_reg->start_pfn = start_pfn;
	new_reg->nr_pages = nr_pages;

	/* Regions are a whole use, so swap and delete old one. */
	if (at_reg->start_pfn == start_pfn && at_reg->nr_pages == nr_pages) {
		rb_replace_node(&(at_reg->rblink), &(new_reg->rblink), reg_rbtree);
		kfree(at_reg);
	}
	/* New region replaces the start of the old one, so insert before. */
	else if (at_reg->start_pfn == start_pfn) {
		at_reg->start_pfn += nr_pages;
		KBASE_DEBUG_ASSERT(at_reg->nr_pages >= nr_pages);
		at_reg->nr_pages -= nr_pages;

		kbase_region_tracker_insert(new_reg);
	}
	/* New region replaces the end of the old one, so insert after. */
	else if ((at_reg->start_pfn + at_reg->nr_pages) == (start_pfn + nr_pages)) {
		at_reg->nr_pages -= nr_pages;

		kbase_region_tracker_insert(new_reg);
	}
	/* New region splits the old one, so insert and create new */
	else {
		struct kbase_va_region *new_front_reg;

		new_front_reg = kbase_alloc_free_region(zone, at_reg->start_pfn,
							start_pfn - at_reg->start_pfn);

		if (new_front_reg) {
			at_reg->nr_pages -= nr_pages + new_front_reg->nr_pages;
			at_reg->start_pfn = start_pfn + nr_pages;

			kbase_region_tracker_insert(new_front_reg);
			kbase_region_tracker_insert(new_reg);
		} else {
			err = -ENOMEM;
		}
	}

	return err;
}

/**
 * kbase_add_va_region - Add a VA region to the region list for a context.
 *
 * @kctx: kbase context containing the region
 * @reg: the region to add
 * @addr: the address to insert the region at
 * @nr_pages: the number of pages in the region
 * @align: the minimum alignment in pages
 *
 * Return: 0 on success, error code otherwise.
 */
int kbase_add_va_region(struct kbase_context *kctx, struct kbase_va_region *reg, u64 addr,
			size_t nr_pages, size_t align)
{
	int err = 0;
	struct kbase_device *kbdev = kctx->kbdev;
	const size_t cpu_va_bits = kbase_get_num_cpu_va_bits(kctx);
	const size_t gpu_pc_bits = kbdev->gpu_props.log2_program_counter_size;

	KBASE_DEBUG_ASSERT(kctx != NULL);
	KBASE_DEBUG_ASSERT(reg != NULL);

	lockdep_assert_held(&kctx->reg_lock);

	/* The executable allocation from the SAME_VA zone should already have an
	 * appropriately aligned GPU VA chosen for it.
	 * Also, executable allocations from EXEC_VA don't need the special
	 * alignment.
	 */
#if MALI_USE_CSF
	/* The same is also true for the EXEC_FIXED_VA zone.
	 */
#endif
	if (!(reg->flags & KBASE_REG_GPU_NX) && !addr &&
#if MALI_USE_CSF
	    ((kbase_bits_to_zone(reg->flags)) != EXEC_FIXED_VA_ZONE) &&
#endif
	    ((kbase_bits_to_zone(reg->flags)) != EXEC_VA_ZONE)) {
		if (cpu_va_bits > gpu_pc_bits) {
			align = max(align, (size_t)((1ULL << gpu_pc_bits) >> PAGE_SHIFT));
		}
	}

	do {
		err = kbase_add_va_region_rbtree(kbdev, reg, addr, nr_pages, align);
		if (err != -ENOMEM)
			break;

		/*
		 * If the allocation is not from the same zone as JIT
		 * then don't retry, we're out of VA and there is
		 * nothing which can be done about it.
		 */
		if ((kbase_bits_to_zone(reg->flags)) != CUSTOM_VA_ZONE)
			break;
	} while (kbase_jit_evict(kctx));

	return err;
}
KBASE_EXPORT_TEST_API(kbase_add_va_region);

/**
 * kbase_add_va_region_rbtree - Insert a region into its corresponding rbtree
 *
 * @kbdev: The kbase device
 * @reg: The region to add
 * @addr: The address to add the region at, or 0 to map at any available address
 * @nr_pages: The size of the region in pages
 * @align: The minimum alignment in pages
 *
 * Insert a region into the rbtree that was specified when the region was
 * created. If addr is 0 a free area in the rbtree is used, otherwise the
 * specified address is used.
 *
 * Return: 0 on success, error code otherwise.
 */
int kbase_add_va_region_rbtree(struct kbase_device *kbdev, struct kbase_va_region *reg, u64 addr,
			       size_t nr_pages, size_t align)
{
	struct device *const dev = kbdev->dev;
	struct rb_root *rbtree = NULL;
	struct kbase_va_region *tmp;
	const u64 gpu_pfn = addr >> PAGE_SHIFT;
	int err = 0;

	rbtree = reg->rbtree;

	if (!align)
		align = 1;

	/* must be a power of 2 */
	KBASE_DEBUG_ASSERT(is_power_of_2(align));
	KBASE_DEBUG_ASSERT(nr_pages > 0);

	/* Path 1: Map a specific address. Find the enclosing region,
	 * which *must* be free.
	 */
	if (gpu_pfn) {
		KBASE_DEBUG_ASSERT(!(gpu_pfn & (align - 1)));

		tmp = find_region_enclosing_range_rbtree(rbtree, gpu_pfn, nr_pages);
		if (kbase_is_region_invalid(tmp)) {
			dev_warn(
				dev,
				"Enclosing region not found or invalid: 0x%08llx gpu_pfn, %zu nr_pages",
				gpu_pfn, nr_pages);
			err = -ENOMEM;
			goto exit;
		} else if (!kbase_is_region_free(tmp)) {
			dev_warn(
				dev,
				"!(tmp->flags & KBASE_REG_FREE): tmp->start_pfn=0x%llx tmp->flags=0x%llx tmp->nr_pages=0x%zx gpu_pfn=0x%llx nr_pages=0x%zx\n",
				tmp->start_pfn, tmp->flags, tmp->nr_pages, gpu_pfn, nr_pages);
			err = -ENOMEM;
			goto exit;
		}

		err = kbase_insert_va_region_nolock(kbdev, reg, tmp, gpu_pfn, nr_pages);
		if (err) {
			dev_warn(dev, "Failed to insert va region");
			err = -ENOMEM;
		}
	} else {
		/* Path 2: Map any free address which meets the requirements. */
		u64 start_pfn;
		size_t align_offset = align;
		size_t align_mask = align - 1;

#if !MALI_USE_CSF
		if ((reg->flags & KBASE_REG_TILER_ALIGN_TOP)) {
			WARN(align > 1,
			     "%s with align %lx might not be honored for KBASE_REG_TILER_ALIGN_TOP memory",
			     __func__, (unsigned long)align);
			align_mask = reg->extension - 1;
			align_offset = reg->extension - reg->initial_commit;
		}
#endif /* !MALI_USE_CSF */

		tmp = kbase_region_tracker_find_region_meeting_reqs(reg, nr_pages, align_offset,
								    align_mask, &start_pfn);
		if (tmp) {
			err = kbase_insert_va_region_nolock(kbdev, reg, tmp, start_pfn, nr_pages);
			if (unlikely(err)) {
				dev_warn(
					dev,
					"Failed to insert region: 0x%08llx start_pfn, %zu nr_pages",
					start_pfn, nr_pages);
			}
		} else {
			dev_dbg(dev,
				"Failed to find a suitable region: %zu nr_pages, %zu align_offset, %zu align_mask\n",
				nr_pages, align_offset, align_mask);
			err = -ENOMEM;
		}
	}

exit:
	return err;
}

struct kbase_context *kbase_reg_to_kctx(struct kbase_va_region *reg)
{
	struct rb_root *rbtree = reg->rbtree;
	struct kbase_reg_zone *zone = container_of(rbtree, struct kbase_reg_zone, reg_rbtree);

	if (!kbase_is_ctx_reg_zone(zone->id))
		return NULL;

	return container_of(zone - zone->id, struct kbase_context, reg_zone[0]);
}

/**
 * kbase_region_tracker_erase_rbtree - Free memory for a region tracker
 *
 * @rbtree: Root of the red-black tree to erase.
 *
 * This will free all the regions within the region tracker.
 */
static void kbase_region_tracker_erase_rbtree(struct rb_root *rbtree)
{
	struct rb_node *rbnode;
	struct kbase_va_region *reg;

	do {
		rbnode = rb_first(rbtree);
		if (rbnode) {
			rb_erase(rbnode, rbtree);
			reg = rb_entry(rbnode, struct kbase_va_region, rblink);
			WARN_ON(kbase_refcount_read(&reg->va_refcnt) != 1);
			if (kbase_is_page_migration_enabled()) {
				struct kbase_context *kctx = kbase_reg_to_kctx(reg);

				if (kctx)
					kbase_gpu_munmap(kctx, reg);
			}
			/* Reset the start_pfn - as the rbtree is being
			 * destroyed and we've already erased this region, there
			 * is no further need to attempt to remove it.
			 * This won't affect the cleanup if the region was
			 * being used as a sticky resource as the cleanup
			 * related to sticky resources anyways need to be
			 * performed before the term of region tracker.
			 */
			reg->start_pfn = 0;
			kbase_free_alloced_region(reg);
		}
	} while (rbnode);
}

void kbase_reg_zone_term(struct kbase_reg_zone *zone)
{
	kbase_region_tracker_erase_rbtree(&zone->reg_rbtree);
}

static size_t kbase_get_same_va_bits(struct kbase_context *kctx)
{
	return min_t(size_t, kbase_get_num_cpu_va_bits(kctx), kctx->kbdev->gpu_props.mmu.va_bits);
}

static int kbase_reg_zone_same_va_init(struct kbase_context *kctx, u64 gpu_va_limit)
{
	int err;
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, SAME_VA_ZONE);
	const size_t same_va_bits = kbase_get_same_va_bits(kctx);
	const u64 base_pfn = 1u;
	u64 nr_pages = (1ULL << (same_va_bits - PAGE_SHIFT)) - base_pfn;

	CSTD_UNUSED(gpu_va_limit);

	lockdep_assert_held(&kctx->reg_lock);

#if MALI_USE_CSF
	if ((base_pfn + nr_pages) > KBASE_REG_ZONE_EXEC_VA_BASE_64) {
		/* Depending on how the kernel is configured, it's possible (eg on aarch64) for
		 * same_va_bits to reach 48 bits. Cap same_va_pages so that the same_va zone
		 * doesn't cross into the exec_va zone.
		 */
		nr_pages = KBASE_REG_ZONE_EXEC_VA_BASE_64 - base_pfn;
	}
#endif
	err = kbase_reg_zone_init(kctx->kbdev, zone, SAME_VA_ZONE, base_pfn, nr_pages);
	if (err)
		return -ENOMEM;

	kctx->gpu_va_end = base_pfn + nr_pages;

	return 0;
}

static void kbase_reg_zone_same_va_term(struct kbase_context *kctx)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, SAME_VA_ZONE);

	kbase_reg_zone_term(zone);
}

static int kbase_reg_zone_custom_va_init(struct kbase_context *kctx, u64 gpu_va_limit)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, CUSTOM_VA_ZONE);
	u64 nr_pages = KBASE_REG_ZONE_CUSTOM_VA_SIZE;

	/* If the context does not support CUSTOM_VA zones, then we don't need to
	 * proceed past this point, and can pretend that it was initialized properly.
	 * In practice, this will mean that the zone metadata structure will be zero
	 * initialized and not contain a valid zone ID.
	 */
	if (!kbase_ctx_compat_mode(kctx))
		return 0;

	if (gpu_va_limit <= KBASE_REG_ZONE_CUSTOM_VA_BASE)
		return -EINVAL;

	/* If the current size of TMEM is out of range of the
	 * virtual address space addressable by the MMU then
	 * we should shrink it to fit
	 */
	if ((KBASE_REG_ZONE_CUSTOM_VA_BASE + KBASE_REG_ZONE_CUSTOM_VA_SIZE) >= gpu_va_limit)
		nr_pages = gpu_va_limit - KBASE_REG_ZONE_CUSTOM_VA_BASE;

	if (kbase_reg_zone_init(kctx->kbdev, zone, CUSTOM_VA_ZONE, KBASE_REG_ZONE_CUSTOM_VA_BASE,
				nr_pages))
		return -ENOMEM;

	/* On JM systems, this is the last memory zone that gets initialized,
	 * so the GPU VA ends right after the end of the CUSTOM_VA zone. On CSF,
	 * setting here is harmless, as the FIXED_VA initializer will overwrite
	 * it.
	 */
	kctx->gpu_va_end += nr_pages;

	return 0;
}

static void kbase_reg_zone_custom_va_term(struct kbase_context *kctx)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, CUSTOM_VA_ZONE);

	kbase_reg_zone_term(zone);
}

static inline u64 kbase_get_exec_va_zone_base(struct kbase_context *kctx)
{
	u64 base_pfn;

#if MALI_USE_CSF
	base_pfn = KBASE_REG_ZONE_EXEC_VA_BASE_64;
	if (kbase_ctx_compat_mode(kctx))
		base_pfn = KBASE_REG_ZONE_EXEC_VA_BASE_32;
#else
	CSTD_UNUSED(kctx);
	/* EXEC_VA zone's codepaths are slightly easier when its base_pfn is
	 * initially U64_MAX
	 */
	base_pfn = U64_MAX;
#endif

	return base_pfn;
}

static inline int kbase_reg_zone_exec_va_init(struct kbase_context *kctx, u64 gpu_va_limit)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, EXEC_VA_ZONE);
	const u64 base_pfn = kbase_get_exec_va_zone_base(kctx);
	u64 nr_pages = KBASE_REG_ZONE_EXEC_VA_SIZE;

	CSTD_UNUSED(gpu_va_limit);

#if !MALI_USE_CSF
	nr_pages = 0;
#endif

	return kbase_reg_zone_init(kctx->kbdev, zone, EXEC_VA_ZONE, base_pfn, nr_pages);
}

static void kbase_reg_zone_exec_va_term(struct kbase_context *kctx)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, EXEC_VA_ZONE);

	kbase_reg_zone_term(zone);
}

#if MALI_USE_CSF
static inline u64 kbase_get_exec_fixed_va_zone_base(struct kbase_context *kctx)
{
	return kbase_get_exec_va_zone_base(kctx) + KBASE_REG_ZONE_EXEC_VA_SIZE;
}

static int kbase_reg_zone_exec_fixed_va_init(struct kbase_context *kctx, u64 gpu_va_limit)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, EXEC_FIXED_VA_ZONE);
	const u64 base_pfn = kbase_get_exec_fixed_va_zone_base(kctx);

	CSTD_UNUSED(gpu_va_limit);

	return kbase_reg_zone_init(kctx->kbdev, zone, EXEC_FIXED_VA_ZONE, base_pfn,
				   KBASE_REG_ZONE_EXEC_FIXED_VA_SIZE);
}

static void kbase_reg_zone_exec_fixed_va_term(struct kbase_context *kctx)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, EXEC_FIXED_VA_ZONE);

	WARN_ON(!list_empty(&kctx->csf.event_pages_head));
	kbase_reg_zone_term(zone);
}

static int kbase_reg_zone_fixed_va_init(struct kbase_context *kctx, u64 gpu_va_limit)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, FIXED_VA_ZONE);
	const u64 base_pfn =
		kbase_get_exec_fixed_va_zone_base(kctx) + KBASE_REG_ZONE_EXEC_FIXED_VA_SIZE;
	u64 fixed_va_end = KBASE_REG_ZONE_FIXED_VA_END_64;
	u64 nr_pages;
	CSTD_UNUSED(gpu_va_limit);

	if (kbase_ctx_compat_mode(kctx))
		fixed_va_end = KBASE_REG_ZONE_FIXED_VA_END_32;

	nr_pages = fixed_va_end - base_pfn;

	if (kbase_reg_zone_init(kctx->kbdev, zone, FIXED_VA_ZONE, base_pfn, nr_pages))
		return -ENOMEM;

	kctx->gpu_va_end = fixed_va_end;

	return 0;
}

static void kbase_reg_zone_fixed_va_term(struct kbase_context *kctx)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get(kctx, FIXED_VA_ZONE);

	kbase_reg_zone_term(zone);
}
#endif

typedef int kbase_memory_zone_init(struct kbase_context *kctx, u64 gpu_va_limit);
typedef void kbase_memory_zone_term(struct kbase_context *kctx);

struct kbase_memory_zone_init_meta {
	kbase_memory_zone_init *init;
	kbase_memory_zone_term *term;
	char *error_msg;
};

static const struct kbase_memory_zone_init_meta zones_init[] = {
	[SAME_VA_ZONE] = { kbase_reg_zone_same_va_init, kbase_reg_zone_same_va_term,
			   "Could not initialize SAME_VA zone" },
	[CUSTOM_VA_ZONE] = { kbase_reg_zone_custom_va_init, kbase_reg_zone_custom_va_term,
			     "Could not initialize CUSTOM_VA zone" },
	[EXEC_VA_ZONE] = { kbase_reg_zone_exec_va_init, kbase_reg_zone_exec_va_term,
			   "Could not initialize EXEC_VA zone" },
#if MALI_USE_CSF
	[EXEC_FIXED_VA_ZONE] = { kbase_reg_zone_exec_fixed_va_init,
				 kbase_reg_zone_exec_fixed_va_term,
				 "Could not initialize EXEC_FIXED_VA zone" },
	[FIXED_VA_ZONE] = { kbase_reg_zone_fixed_va_init, kbase_reg_zone_fixed_va_term,
			    "Could not initialize FIXED_VA zone" },
#endif
};

int kbase_region_tracker_init(struct kbase_context *kctx)
{
	const u64 gpu_va_bits = kctx->kbdev->gpu_props.mmu.va_bits;
	const u64 gpu_va_limit = (1ULL << gpu_va_bits) >> PAGE_SHIFT;
	int err;
	unsigned int i;

	/* Take the lock as kbase_free_alloced_region requires it */
	kbase_gpu_vm_lock(kctx);

	for (i = 0; i < ARRAY_SIZE(zones_init); i++) {
		err = zones_init[i].init(kctx, gpu_va_limit);
		if (unlikely(err)) {
			dev_err(kctx->kbdev->dev, "%s, err = %d\n", zones_init[i].error_msg, err);
			goto term;
		}
	}
#if MALI_USE_CSF
	INIT_LIST_HEAD(&kctx->csf.event_pages_head);
#endif
	kctx->jit_va = false;

	kbase_gpu_vm_unlock(kctx);

	return 0;
term:
	while (i-- > 0)
		zones_init[i].term(kctx);

	kbase_gpu_vm_unlock(kctx);
	return err;
}

void kbase_region_tracker_term(struct kbase_context *kctx)
{
	unsigned int i;

	WARN(kctx->as_nr != KBASEP_AS_NR_INVALID,
	     "kctx-%d_%d must first be scheduled out to flush GPU caches+tlbs before erasing remaining regions",
	     kctx->tgid, kctx->id);

	kbase_gpu_vm_lock(kctx);

	for (i = 0; i < ARRAY_SIZE(zones_init); i++)
		zones_init[i].term(kctx);

	kbase_gpu_vm_unlock(kctx);
}

static bool kbase_has_exec_va_zone_locked(struct kbase_context *kctx)
{
	struct kbase_reg_zone *exec_va_zone;

	lockdep_assert_held(&kctx->reg_lock);
	exec_va_zone = kbase_ctx_reg_zone_get(kctx, EXEC_VA_ZONE);

	return (exec_va_zone->base_pfn != U64_MAX);
}

bool kbase_has_exec_va_zone(struct kbase_context *kctx)
{
	bool has_exec_va_zone;

	kbase_gpu_vm_lock(kctx);
	has_exec_va_zone = kbase_has_exec_va_zone_locked(kctx);
	kbase_gpu_vm_unlock(kctx);

	return has_exec_va_zone;
}
KBASE_EXPORT_TEST_API(kbase_has_exec_va_zone);

/**
 * kbase_region_tracker_has_allocs - Determine if any allocations have been made
 * on a context's region tracker
 *
 * @kctx: KBase context
 *
 * Check the context to determine if any allocations have been made yet from
 * any of its zones. This check should be done before resizing a zone, e.g. to
 * make space to add a second zone.
 *
 * Whilst a zone without allocations can be resized whilst other zones have
 * allocations, we still check all of @kctx 's zones anyway: this is a stronger
 * guarantee and should be adhered to when creating new zones anyway.
 *
 * Allocations from kbdev zones are not counted.
 *
 * Return: true if any allocs exist on any zone, false otherwise
 */
static bool kbase_region_tracker_has_allocs(struct kbase_context *kctx)
{
	unsigned int zone_idx;

	lockdep_assert_held(&kctx->reg_lock);

	for (zone_idx = 0; zone_idx < MEMORY_ZONE_MAX; zone_idx++) {
		struct kbase_reg_zone *zone;
		struct kbase_va_region *reg;
		u64 zone_base_addr;
		enum kbase_memory_zone reg_zone;

		if (!kbase_is_ctx_reg_zone(zone_idx))
			continue;

		zone = kbase_ctx_reg_zone_get(kctx, zone_idx);
		zone_base_addr = zone->base_pfn << PAGE_SHIFT;

		reg = kbase_region_tracker_find_region_base_address(kctx, zone_base_addr);

		if (!zone->va_size_pages) {
			WARN(reg,
			     "Should not have found a region that starts at 0x%.16llx for zone %s",
			     (unsigned long long)zone_base_addr, kbase_reg_zone_get_name(zone_idx));
			continue;
		}

		if (WARN(!reg,
			 "There should always be a region that starts at 0x%.16llx for zone %s, couldn't find it",
			 (unsigned long long)zone_base_addr, kbase_reg_zone_get_name(zone_idx)))
			return true; /* Safest return value */

		reg_zone = kbase_bits_to_zone(reg->flags);
		if (WARN(reg_zone != zone_idx,
			 "The region that starts at 0x%.16llx should be in zone %s but was found in the wrong zone %s",
			 (unsigned long long)zone_base_addr, kbase_reg_zone_get_name(zone_idx),
			 kbase_reg_zone_get_name(reg_zone)))
			return true; /* Safest return value */

		/* Unless the region is completely free, of the same size as
		 * the original zone, then it has allocs
		 */
		if ((!(reg->flags & KBASE_REG_FREE)) || (reg->nr_pages != zone->va_size_pages))
			return true;
	}

	/* All zones are the same size as originally made, so there are no
	 * allocs
	 */
	return false;
}

static int kbase_region_tracker_init_jit_64(struct kbase_context *kctx, u64 jit_va_pages)
{
	struct kbase_va_region *same_va_reg;
	struct kbase_reg_zone *same_va_zone, *custom_va_zone;
	u64 same_va_zone_base_addr;
	u64 jit_va_start;

	lockdep_assert_held(&kctx->reg_lock);

	/*
	 * Modify the same VA free region after creation. The caller has
	 * ensured that allocations haven't been made, as any allocations could
	 * cause an overlap to happen with existing same VA allocations and the
	 * custom VA zone.
	 */
	same_va_zone = kbase_ctx_reg_zone_get(kctx, SAME_VA_ZONE);
	same_va_zone_base_addr = same_va_zone->base_pfn << PAGE_SHIFT;

	same_va_reg = kbase_region_tracker_find_region_base_address(kctx, same_va_zone_base_addr);
	if (WARN(!same_va_reg,
		 "Already found a free region at the start of every zone, but now cannot find any region for zone SAME_VA base 0x%.16llx",
		 (unsigned long long)same_va_zone_base_addr))
		return -ENOMEM;

	/* kbase_region_tracker_has_allocs() in the caller has already ensured
	 * that all of the zones have no allocs, so no need to check that again
	 * on same_va_reg
	 */
	WARN_ON((!(same_va_reg->flags & KBASE_REG_FREE)) ||
		same_va_reg->nr_pages != same_va_zone->va_size_pages);

	if (same_va_reg->nr_pages < jit_va_pages || same_va_zone->va_size_pages < jit_va_pages)
		return -ENOMEM;

	/* It's safe to adjust the same VA zone now */
	same_va_reg->nr_pages -= jit_va_pages;
	same_va_zone->va_size_pages -= jit_va_pages;
	jit_va_start = kbase_reg_zone_end_pfn(same_va_zone);

	/*
	 * Create a custom VA zone at the end of the VA for allocations which
	 * JIT can use so it doesn't have to allocate VA from the kernel. Note
	 * that while the zone has already been zero-initialized during the
	 * region tracker initialization, we can just overwrite it.
	 */
	custom_va_zone = kbase_ctx_reg_zone_get(kctx, CUSTOM_VA_ZONE);
	if (kbase_reg_zone_init(kctx->kbdev, custom_va_zone, CUSTOM_VA_ZONE, jit_va_start,
				jit_va_pages))
		return -ENOMEM;

	return 0;
}

int kbase_region_tracker_init_jit(struct kbase_context *kctx, u64 jit_va_pages, int max_allocations,
				  int trim_level, int group_id, u64 phys_pages_limit)
{
	int err = 0;

	if (trim_level < 0 || trim_level > BASE_JIT_MAX_TRIM_LEVEL)
		return -EINVAL;

	if (group_id < 0 || group_id >= MEMORY_GROUP_MANAGER_NR_GROUPS)
		return -EINVAL;

	if (phys_pages_limit > jit_va_pages)
		return -EINVAL;

#if MALI_JIT_PRESSURE_LIMIT_BASE
	if (phys_pages_limit != jit_va_pages)
		kbase_ctx_flag_set(kctx, KCTX_JPL_ENABLED);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

	kbase_gpu_vm_lock(kctx);

	/* Verify that a JIT_VA zone has not been created already. */
	if (kctx->jit_va) {
		err = -EINVAL;
		goto exit_unlock;
	}

	/* If in 64-bit, we always lookup the SAME_VA zone. To ensure it has no
	 * allocs, we can ensure there are no allocs anywhere.
	 *
	 * This check is also useful in 32-bit, just to make sure init of the
	 * zone is always done before any allocs.
	 */
	if (kbase_region_tracker_has_allocs(kctx)) {
		err = -ENOMEM;
		goto exit_unlock;
	}

	if (!kbase_ctx_compat_mode(kctx))
		err = kbase_region_tracker_init_jit_64(kctx, jit_va_pages);
	/*
	 * Nothing to do for 32-bit clients, JIT uses the existing
	 * custom VA zone.
	 */

	if (!err) {
		kctx->jit_max_allocations = max_allocations;
		kctx->trim_level = trim_level;
		kctx->jit_va = true;
		kctx->jit_group_id = group_id;
#if MALI_JIT_PRESSURE_LIMIT_BASE
		kctx->jit_phys_pages_limit = phys_pages_limit;
		dev_dbg(kctx->kbdev->dev, "phys_pages_limit set to %llu\n", phys_pages_limit);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */
	}

exit_unlock:
	kbase_gpu_vm_unlock(kctx);

	return err;
}
KBASE_EXPORT_TEST_API(kbase_region_tracker_init_jit);

int kbase_region_tracker_init_exec(struct kbase_context *kctx, u64 exec_va_pages)
{
#if !MALI_USE_CSF
	struct kbase_reg_zone *exec_va_zone;
	struct kbase_reg_zone *target_zone;
	struct kbase_va_region *target_reg;
	u64 target_zone_base_addr;
	enum kbase_memory_zone target_zone_id;
	u64 exec_va_start;
	int err;
#endif

	/* The EXEC_VA zone shall be created by making space either:
	 * - for 64-bit clients, at the end of the process's address space
	 * - for 32-bit clients, in the CUSTOM zone
	 *
	 * Firstly, verify that the number of EXEC_VA pages requested by the
	 * client is reasonable and then make sure that it is not greater than
	 * the address space itself before calculating the base address of the
	 * new zone.
	 */
	if (exec_va_pages == 0 || exec_va_pages > KBASE_REG_ZONE_EXEC_VA_MAX_PAGES)
		return -EINVAL;

#if MALI_USE_CSF
	/* For CSF GPUs we now setup the EXEC_VA zone during initialization,
	 * so this request is a null-op.
	 */
	CSTD_UNUSED(kctx);
	return 0;
#else
	kbase_gpu_vm_lock(kctx);

	/* Verify that we've not already created a EXEC_VA zone, and that the
	 * EXEC_VA zone must come before JIT's CUSTOM_VA.
	 */
	if (kbase_has_exec_va_zone_locked(kctx) || kctx->jit_va) {
		err = -EPERM;
		goto exit_unlock;
	}

	if (exec_va_pages > kctx->gpu_va_end) {
		err = -ENOMEM;
		goto exit_unlock;
	}

	/* Verify no allocations have already been made */
	if (kbase_region_tracker_has_allocs(kctx)) {
		err = -ENOMEM;
		goto exit_unlock;
	}

	if (kbase_ctx_compat_mode(kctx)) {
		/* 32-bit client: take from CUSTOM_VA zone */
		target_zone_id = CUSTOM_VA_ZONE;
	} else {
		/* 64-bit client: take from SAME_VA zone */
		target_zone_id = SAME_VA_ZONE;
	}

	target_zone = kbase_ctx_reg_zone_get(kctx, target_zone_id);
	target_zone_base_addr = target_zone->base_pfn << PAGE_SHIFT;

	target_reg = kbase_region_tracker_find_region_base_address(kctx, target_zone_base_addr);
	if (WARN(!target_reg,
		 "Already found a free region at the start of every zone, but now cannot find any region for zone base 0x%.16llx zone %s",
		 (unsigned long long)target_zone_base_addr,
		 kbase_reg_zone_get_name(target_zone_id))) {
		err = -ENOMEM;
		goto exit_unlock;
	}
	/* kbase_region_tracker_has_allocs() above has already ensured that all
	 * of the zones have no allocs, so no need to check that again on
	 * target_reg
	 */
	WARN_ON((!(target_reg->flags & KBASE_REG_FREE)) ||
		target_reg->nr_pages != target_zone->va_size_pages);

	if (target_reg->nr_pages <= exec_va_pages || target_zone->va_size_pages <= exec_va_pages) {
		err = -ENOMEM;
		goto exit_unlock;
	}

	/* Taken from the end of the target zone */
	exec_va_start = kbase_reg_zone_end_pfn(target_zone) - exec_va_pages;
	exec_va_zone = kbase_ctx_reg_zone_get(kctx, EXEC_VA_ZONE);
	if (kbase_reg_zone_init(kctx->kbdev, exec_va_zone, EXEC_VA_ZONE, exec_va_start,
				exec_va_pages))
		return -ENOMEM;

	/* Update target zone and corresponding region */
	target_reg->nr_pages -= exec_va_pages;
	target_zone->va_size_pages -= exec_va_pages;
	err = 0;

exit_unlock:
	kbase_gpu_vm_unlock(kctx);
	return err;
#endif /* MALI_USE_CSF */
}
KBASE_EXPORT_TEST_API(kbase_region_tracker_init_exec);

#if MALI_USE_CSF
void kbase_mcu_shared_interface_region_tracker_term(struct kbase_device *kbdev)
{
	kbase_reg_zone_term(&kbdev->csf.mcu_shared_zone);
}

int kbase_mcu_shared_interface_region_tracker_init(struct kbase_device *kbdev)
{
	return kbase_reg_zone_init(kbdev, &kbdev->csf.mcu_shared_zone, MCU_SHARED_ZONE,
				   KBASE_REG_ZONE_MCU_SHARED_BASE, MCU_SHARED_ZONE_SIZE);
}
#endif

/**
 * kbase_alloc_free_region - Allocate a free region object.
 *
 * @zone:      The memory zone the new region object will be part of.
 * @start_pfn: The Page Frame Number in GPU virtual address space.
 * @nr_pages:  The size of the region in pages.
 *
 * The allocated object is not part of any list yet, and is flagged as
 * KBASE_REG_FREE. No mapping is allocated yet.
 *
 * zone is CUSTOM_VA_ZONE or SAME_VA_ZONE.
 *
 * Return: pointer to the allocated region object on success, NULL otherwise.
 */
struct kbase_va_region *kbase_alloc_free_region(struct kbase_reg_zone *zone, u64 start_pfn,
						size_t nr_pages)
{
	struct kbase_va_region *new_reg;

	/* 64-bit address range is the max */
	KBASE_DEBUG_ASSERT(start_pfn + nr_pages <= (U64_MAX / PAGE_SIZE));

	if (unlikely(!nr_pages))
		return NULL;

	if (WARN_ON(!zone))
		return NULL;

	if (unlikely(!zone->base_pfn || !zone->va_size_pages))
		return NULL;

	new_reg = kmem_cache_zalloc(zone->cache, GFP_KERNEL);

	if (!new_reg)
		return NULL;

	kbase_refcount_set(&new_reg->va_refcnt, 1);
	atomic64_set(&new_reg->no_user_free_count, 0);
	new_reg->cpu_alloc = NULL; /* no alloc bound yet */
	new_reg->gpu_alloc = NULL; /* no alloc bound yet */
	new_reg->rbtree = &zone->reg_rbtree;
	new_reg->flags = kbase_zone_to_bits(zone->id) | KBASE_REG_FREE;

	new_reg->flags |= KBASE_REG_GROWABLE;

	new_reg->start_pfn = start_pfn;
	new_reg->nr_pages = nr_pages;

	INIT_LIST_HEAD(&new_reg->jit_node);
	INIT_LIST_HEAD(&new_reg->link);

	return new_reg;
}
KBASE_EXPORT_TEST_API(kbase_alloc_free_region);

struct kbase_va_region *kbase_ctx_alloc_free_region(struct kbase_context *kctx,
						    enum kbase_memory_zone id, u64 start_pfn,
						    size_t nr_pages)
{
	struct kbase_reg_zone *zone = kbase_ctx_reg_zone_get_nolock(kctx, id);

	return kbase_alloc_free_region(zone, start_pfn, nr_pages);
}
KBASE_EXPORT_TEST_API(kbase_ctx_alloc_free_region);

/**
 * kbase_free_alloced_region - Free a region object.
 *
 * @reg: Region
 *
 * The described region must be freed of any mapping.
 *
 * If the region is not flagged as KBASE_REG_FREE, the region's
 * alloc object will be released.
 * It is a bug if no alloc object exists for non-free regions.
 *
 * If region is MCU_SHARED_ZONE it is freed
 */
void kbase_free_alloced_region(struct kbase_va_region *reg)
{
#if MALI_USE_CSF
	if (kbase_bits_to_zone(reg->flags) == MCU_SHARED_ZONE) {
		kfree(reg);
		return;
	}
#endif
	if (!(reg->flags & KBASE_REG_FREE)) {
		struct kbase_context *kctx = kbase_reg_to_kctx(reg);

		if (WARN_ON(!kctx))
			return;

		if (WARN_ON(kbase_is_region_invalid(reg)))
			return;

		dev_dbg(kctx->kbdev->dev, "Freeing memory region %pK\n of zone %s", (void *)reg,
			kbase_reg_zone_get_name(kbase_bits_to_zone(reg->flags)));
#if MALI_USE_CSF
		if (reg->flags & KBASE_REG_CSF_EVENT)
			/*
			 * This should not be reachable if called from 'mcu_shared' functions
			 * such as:
			 * kbase_csf_firmware_mcu_shared_mapping_init
			 * kbase_csf_firmware_mcu_shared_mapping_term
			 */

			kbase_unlink_event_mem_page(kctx, reg);
#endif

		mutex_lock(&kctx->jit_evict_lock);

		/*
		 * The physical allocation should have been removed from the
		 * eviction list before this function is called. However, in the
		 * case of abnormal process termination or the app leaking the
		 * memory kbase_mem_free_region is not called so it can still be
		 * on the list at termination time of the region tracker.
		 */
		if (!list_empty(&reg->gpu_alloc->evict_node)) {
			/*
			 * Unlink the physical allocation before unmaking it
			 * evictable so that the allocation isn't grown back to
			 * its last backed size as we're going to unmap it
			 * anyway.
			 */
			reg->cpu_alloc->reg = NULL;
			if (reg->cpu_alloc != reg->gpu_alloc)
				reg->gpu_alloc->reg = NULL;

			mutex_unlock(&kctx->jit_evict_lock);

			/*
			 * If a region has been made evictable then we must
			 * unmake it before trying to free it.
			 * If the memory hasn't been reclaimed it will be
			 * unmapped and freed below, if it has been reclaimed
			 * then the operations below are no-ops.
			 */
			if (reg->flags & BASEP_MEM_DONT_NEED) {
				KBASE_DEBUG_ASSERT(reg->cpu_alloc->type == KBASE_MEM_TYPE_NATIVE);
				kbase_mem_evictable_unmake(reg->gpu_alloc);
			}
		} else {
			mutex_unlock(&kctx->jit_evict_lock);
		}

		/*
		 * Remove the region from the sticky resource metadata
		 * list should it be there.
		 */
		kbase_sticky_resource_release_force(kctx, NULL, reg->start_pfn << PAGE_SHIFT);

		kbase_mem_phy_alloc_put(reg->cpu_alloc);
		kbase_mem_phy_alloc_put(reg->gpu_alloc);

		reg->flags |= KBASE_REG_VA_FREED;
		kbase_va_region_alloc_put(kctx, reg);
	} else {
		kfree(reg);
	}
}
KBASE_EXPORT_TEST_API(kbase_free_alloced_region);

int kbase_reg_zone_init(struct kbase_device *kbdev, struct kbase_reg_zone *zone,
			enum kbase_memory_zone id, u64 base_pfn, u64 va_size_pages)
{
	struct kbase_va_region *reg;

	*zone = (struct kbase_reg_zone){ .reg_rbtree = RB_ROOT,
					 .base_pfn = base_pfn,
					 .va_size_pages = va_size_pages,
					 .id = id,
					 .cache = kbdev->va_region_slab };

	if (unlikely(!va_size_pages))
		return 0;

	reg = kbase_alloc_free_region(zone, base_pfn, va_size_pages);
	if (unlikely(!reg))
		return -ENOMEM;

	kbase_region_tracker_insert(reg);

	return 0;
}
