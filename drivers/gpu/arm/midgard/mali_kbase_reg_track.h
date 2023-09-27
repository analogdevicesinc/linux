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
#ifndef _KBASE_REG_TRACK_H_
#define _KBASE_REG_TRACK_H_

#include <linux/types.h>
#include <linux/rbtree.h>

/* Forward declarations of required types. To avoid increasing the compilation
 * times of files that include this header, we want to avoid getting too many
 * transitive dependencies on both custom and kernel headers.
 */
struct kbase_context;
struct kbase_va_region;
struct kbase_device;
struct kmem_cache;

#if MALI_USE_CSF
/* Space for 8 different zones */
#define KBASE_REG_ZONE_BITS 3
#else
/* Space for 4 different zones */
#define KBASE_REG_ZONE_BITS 2
#endif

/**
 * KBASE_REG_ZONE_MAX - Maximum number of GPU memory region zones
 */
#if MALI_USE_CSF
#define KBASE_REG_ZONE_MAX 6ul
#else
#define KBASE_REG_ZONE_MAX 4ul
#endif

/* The bits 11-13 (inclusive) of the kbase_va_region flag are reserved
 * for information about the zone in which it was allocated.
 */
#define KBASE_REG_ZONE_SHIFT (11ul)
#define KBASE_REG_ZONE_MASK (((1 << KBASE_REG_ZONE_BITS) - 1ul) << KBASE_REG_ZONE_SHIFT)

#if KBASE_REG_ZONE_MAX > (1 << KBASE_REG_ZONE_BITS)
#error "Too many zones for the number of zone bits defined"
#endif

#define KBASE_REG_ZONE_CUSTOM_VA_BASE (0x100000000ULL >> PAGE_SHIFT)

#if MALI_USE_CSF
/* only used with 32-bit clients */
/* On a 32bit platform, custom VA should be wired from 4GB to 2^(43).
 */
#define KBASE_REG_ZONE_CUSTOM_VA_SIZE (((1ULL << 43) >> PAGE_SHIFT) - KBASE_REG_ZONE_CUSTOM_VA_BASE)
#else
/* only used with 32-bit clients */
/* On a 32bit platform, custom VA should be wired from 4GB to the VA limit of the
 * GPU. Unfortunately, the Linux mmap() interface limits us to 2^32 pages (2^44
 * bytes, see mmap64 man page for reference).  So we put the default limit to the
 * maximum possible on Linux and shrink it down, if required by the GPU, during
 * initialization.
 */
#define KBASE_REG_ZONE_CUSTOM_VA_SIZE (((1ULL << 44) >> PAGE_SHIFT) - KBASE_REG_ZONE_CUSTOM_VA_BASE)
/* end 32-bit clients only */
#endif

/* The starting address and size of the GPU-executable zone are dynamic
 * and depend on the platform and the number of pages requested by the
 * user process, with an upper limit of 4 GB.
 */
#define KBASE_REG_ZONE_EXEC_VA_MAX_PAGES ((1ULL << 32) >> PAGE_SHIFT) /* 4 GB */
#define KBASE_REG_ZONE_EXEC_VA_SIZE KBASE_REG_ZONE_EXEC_VA_MAX_PAGES

#if MALI_USE_CSF
#define KBASE_REG_ZONE_MCU_SHARED_BASE (0x04000000ULL >> PAGE_SHIFT)
#define MCU_SHARED_ZONE_SIZE (((0x08000000ULL) >> PAGE_SHIFT) - KBASE_REG_ZONE_MCU_SHARED_BASE)

/* For CSF GPUs, the EXEC_VA zone is always 4GB in size, and starts at 2^47 for 64-bit
 * clients, and 2^43 for 32-bit clients.
 */
#define KBASE_REG_ZONE_EXEC_VA_BASE_64 ((1ULL << 47) >> PAGE_SHIFT)
#define KBASE_REG_ZONE_EXEC_VA_BASE_32 ((1ULL << 43) >> PAGE_SHIFT)

/* Executable zone supporting FIXED/FIXABLE allocations.
 * It is always 4GB in size.
 */
#define KBASE_REG_ZONE_EXEC_FIXED_VA_SIZE KBASE_REG_ZONE_EXEC_VA_MAX_PAGES

/* Non-executable zone supporting FIXED/FIXABLE allocations.
 * It extends from (2^47) up to (2^48)-1, for 64-bit userspace clients, and from
 * (2^43) up to (2^44)-1 for 32-bit userspace clients. For the same reason,
 * the end of the FIXED_VA zone for 64-bit clients is (2^48)-1.
 */
#define KBASE_REG_ZONE_FIXED_VA_END_64 ((1ULL << 48) >> PAGE_SHIFT)
#define KBASE_REG_ZONE_FIXED_VA_END_32 ((1ULL << 44) >> PAGE_SHIFT)

#endif

/**
 * enum kbase_memory_zone - Kbase memory zone identifier
 * @SAME_VA_ZONE: Memory zone for allocations where the GPU and CPU VA coincide.
 * @CUSTOM_VA_ZONE: When operating in compatibility mode, this zone is used to
 *                  allow 32-bit userspace (either on a 32-bit device or a
 *                  32-bit application on a 64-bit device) to address the entirety
 *                  of the GPU address space. The @CUSTOM_VA_ZONE is also used
 *                  for JIT allocations: on 64-bit systems, the zone is created
 *                  by reducing the size of the SAME_VA zone by a user-controlled
 *                  amount, whereas on 32-bit systems, it is created as part of
 *                  the existing CUSTOM_VA_ZONE
 * @EXEC_VA_ZONE: Memory zone used to track GPU-executable memory. The start
 *                and end of this zone depend on the individual platform,
 *                and it is initialized upon user process request.
 * @EXEC_FIXED_VA_ZONE: Memory zone used to contain GPU-executable memory
 *                      that also permits FIXED/FIXABLE allocations.
 * @FIXED_VA_ZONE: Memory zone used to allocate memory at userspace-supplied
 *                 addresses.
 * @MCU_SHARED_ZONE: Memory zone created for mappings shared between the MCU
 *                   and Kbase. Currently this is the only zone type that is
 *                   created on a per-device, rather than a per-context
 *                   basis.
 * @MEMORY_ZONE_MAX: Sentinel value used for iterating over all the memory zone
 *                   identifiers.
 * @CONTEXT_ZONE_MAX: Sentinel value used to keep track of the last per-context
 *                    zone for iteration.
 */
enum kbase_memory_zone {
	SAME_VA_ZONE,
	CUSTOM_VA_ZONE,
	EXEC_VA_ZONE,
#if IS_ENABLED(MALI_USE_CSF)
	EXEC_FIXED_VA_ZONE,
	FIXED_VA_ZONE,
	MCU_SHARED_ZONE,
#endif
	MEMORY_ZONE_MAX,
#if IS_ENABLED(MALI_USE_CSF)
	CONTEXT_ZONE_MAX = FIXED_VA_ZONE + 1
#else
	CONTEXT_ZONE_MAX = EXEC_VA_ZONE + 1
#endif
};

/**
 * struct kbase_reg_zone - GPU memory zone information and region tracking
 * @reg_rbtree: RB tree used to track kbase memory regions.
 * @base_pfn: Page Frame Number in GPU virtual address space for the start of
 *            the Zone
 * @va_size_pages: Size of the Zone in pages
 * @id: Memory zone identifier
 * @cache: Pointer to a per-device slab allocator to allow for quickly allocating
 *         new regions
 *
 * Track information about a zone KBASE_REG_ZONE() and related macros.
 * In future, this could also store the &rb_root that are currently in
 * &kbase_context and &kbase_csf_device.
 */
struct kbase_reg_zone {
	struct rb_root reg_rbtree;
	u64 base_pfn;
	u64 va_size_pages;
	enum kbase_memory_zone id;
	struct kmem_cache *cache;
};

/**
 * kbase_zone_to_bits - Convert a memory zone @zone to the corresponding
 *                      bitpattern, for ORing together with other flags.
 * @zone: Memory zone
 *
 * Return: Bitpattern with the appropriate bits set.
 */
unsigned long kbase_zone_to_bits(enum kbase_memory_zone zone);

/**
 * kbase_bits_to_zone - Convert the bitpattern @zone_bits to the corresponding
 *                      zone identifier
 * @zone_bits: Memory allocation flag containing a zone pattern
 *
 * Return: Zone identifier for valid zone bitpatterns,
 */
enum kbase_memory_zone kbase_bits_to_zone(unsigned long zone_bits);

/**
 * kbase_mem_zone_get_name - Get the string name for a given memory zone
 * @zone: Memory zone identifier
 *
 * Return: string for valid memory zone, NULL otherwise
 */
char *kbase_reg_zone_get_name(enum kbase_memory_zone zone);

/**
 * kbase_is_ctx_reg_zone - Determine whether a zone is associated with a
 *                         context or with the device
 * @zone: Zone identifier
 *
 * Return: True if @zone is a context zone, False otherwise
 */
static inline bool kbase_is_ctx_reg_zone(enum kbase_memory_zone zone)
{
#if MALI_USE_CSF
	return !(zone == MCU_SHARED_ZONE);
#else
	return true;
#endif
}

/**
 * kbase_region_tracker_init - Initialize the region tracker data structure
 * @kctx: kbase context
 *
 * Return: 0 if success, negative error code otherwise.
 */
int kbase_region_tracker_init(struct kbase_context *kctx);

/**
 * kbase_region_tracker_init_jit - Initialize the just-in-time memory
 *                                 allocation region
 * @kctx:             Kbase context.
 * @jit_va_pages:     Size of the JIT region in pages.
 * @max_allocations:  Maximum number of allocations allowed for the JIT region.
 *                    Valid range is 0..%BASE_JIT_ALLOC_COUNT.
 * @trim_level:       Trim level for the JIT region.
 *                    Valid range is 0..%BASE_JIT_MAX_TRIM_LEVEL.
 * @group_id:         The physical group ID from which to allocate JIT memory.
 *                    Valid range is 0..(%MEMORY_GROUP_MANAGER_NR_GROUPS-1).
 * @phys_pages_limit: Maximum number of physical pages to use to back the JIT
 *                    region. Must not exceed @jit_va_pages.
 *
 * Return: 0 if success, negative error code otherwise.
 */
int kbase_region_tracker_init_jit(struct kbase_context *kctx, u64 jit_va_pages, int max_allocations,
				  int trim_level, int group_id, u64 phys_pages_limit);

/**
 * kbase_region_tracker_init_exec - Initialize the GPU-executable memory region
 * @kctx: kbase context
 * @exec_va_pages: Size of the JIT region in pages.
 *                 It must not be greater than 4 GB.
 *
 * Return: 0 if success, negative error code otherwise.
 */
int kbase_region_tracker_init_exec(struct kbase_context *kctx, u64 exec_va_pages);

/**
 * kbase_region_tracker_term - Terminate the JIT region
 * @kctx: kbase context
 */
void kbase_region_tracker_term(struct kbase_context *kctx);

/**
 * kbase_region_tracker_term - Terminate the JIT region
 * @kctx: kbase context
 */
void kbase_region_tracker_term(struct kbase_context *kctx);

/**
 * kbase_region_tracker_find_region_enclosing_address - Find the region containing
 *                                                      a given GPU VA.
 *
 * @kctx: kbase context containing the region
 * @gpu_addr: pointer to check
 *
 * Context: must be called with region lock held.
 *
 * Return: pointer to the valid region on success, NULL otherwise
 *
 */
struct kbase_va_region *
kbase_region_tracker_find_region_enclosing_address(struct kbase_context *kctx, u64 gpu_addr);

/**
 * kbase_region_tracker_find_region_base_address - Check that a pointer is
 *                                                 actually a valid region.
 * @kctx: kbase context containing the region
 * @gpu_addr: pointer to check
 *
 * Must be called with context lock held.
 *
 * Return: pointer to the valid region on success, NULL otherwise
 */
struct kbase_va_region *kbase_region_tracker_find_region_base_address(struct kbase_context *kctx,
								      u64 gpu_addr);

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
void kbase_remove_va_region(struct kbase_device *kbdev, struct kbase_va_region *reg);

/**
 * kbase_reg_to_kctx - Obtain the kbase context tracking a VA region.
 * @reg: VA region
 *
 * Return:
 * * pointer to kbase context of the memory allocation
 * * NULL if the region does not belong to a kbase context (for instance,
 *   if the allocation corresponds to a shared MCU region on CSF).
 */
struct kbase_context *kbase_reg_to_kctx(struct kbase_va_region *reg);

struct kbase_va_region *kbase_alloc_free_region(struct kbase_reg_zone *zone, u64 start_pfn,
						size_t nr_pages);

struct kbase_va_region *kbase_ctx_alloc_free_region(struct kbase_context *kctx,
						    enum kbase_memory_zone id, u64 start_pfn,
						    size_t nr_pages);

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
			size_t nr_pages, size_t align);

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
 * Note that this method should be removed when we get the per-zone locks, as
 * there will be no compelling use-case for manually separating the allocation
 * and the tracking operations.
 *
 * Return: 0 on success, error code otherwise.
 */
int kbase_add_va_region_rbtree(struct kbase_device *kbdev, struct kbase_va_region *reg, u64 addr,
			       size_t nr_pages, size_t align);

/**
 * kbase_free_alloced_region - Free a region object.
 *
 * @reg: VA region
 *
 * The indicated region must be freed of any mapping. Regions with the following
 * flags have special handling:
 * *
 *
 * If the region is not flagged as KBASE_REG_FREE, the region's
 * alloc object will be released.
 * It is a bug if no alloc object exists for non-free regions.
 *
 * If region is MCU_SHARED it is freed.
 */
void kbase_free_alloced_region(struct kbase_va_region *reg);

/**
 * kbase_reg_zone_init - Initialize a zone in @kctx
 * @kbdev: Pointer to kbase device in order to initialize the VA region cache
 * @zone: Memory zone
 * @id: Memory zone identifier to facilitate lookups
 * @base_pfn: Page Frame Number in GPU virtual address space for the start of
 *            the Zone
 * @va_size_pages: Size of the Zone in pages
 *
 * Return:
 * * 0 on success
 * * -ENOMEM on error
 */
int kbase_reg_zone_init(struct kbase_device *kbdev, struct kbase_reg_zone *zone,
			enum kbase_memory_zone id, u64 base_pfn, u64 va_size_pages);

void kbase_reg_zone_term(struct kbase_reg_zone *zone);

/**
 * kbase_ctx_reg_zone_get_nolock - Get a zone from @kctx where the caller does
 *                                 not have @kctx 's region lock
 * @kctx: Pointer to kbase context
 * @zone: Zone identifier
 *
 * This should only be used in performance-critical paths where the code is
 * resilient to a race with the zone changing, and only when the zone is tracked
 * by the @kctx.
 *
 * Return: The zone corresponding to @zone
 */
struct kbase_reg_zone *kbase_ctx_reg_zone_get_nolock(struct kbase_context *kctx,
						     enum kbase_memory_zone zone);

/**
 * kbase_ctx_reg_zone_get - Get a memory zone from @kctx
 * @kctx: Pointer to kbase context
 * @zone: Zone identifier
 *
 * Note that the zone is not refcounted, so there is no corresponding operation to
 * put the zone back.
 *
 * Return: The zone corresponding to @zone
 */
struct kbase_reg_zone *kbase_ctx_reg_zone_get(struct kbase_context *kctx,
					      enum kbase_memory_zone zone);

/**
 * kbase_reg_zone_end_pfn - return the end Page Frame Number of @zone
 * @zone: zone to query
 *
 * Return: The end of the zone corresponding to @zone
 */
static inline u64 kbase_reg_zone_end_pfn(struct kbase_reg_zone *zone)
{
	if (WARN_ON(!zone))
		return 0;

	return zone->base_pfn + zone->va_size_pages;
}

#endif /* _KBASE_REG_TRACK_H_ */
