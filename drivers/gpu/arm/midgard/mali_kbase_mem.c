// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2010-2023 ARM Limited. All rights reserved.
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
 * DOC: Base kernel memory APIs
 */
#include <linux/dma-buf.h>
#include <linux/kernel.h>
#include <linux/bug.h>
#include <linux/compat.h>
#include <linux/version.h>
#include <linux/log2.h>
#if IS_ENABLED(CONFIG_OF)
#include <linux/of_platform.h>
#endif

#include <mali_kbase_config.h>
#include <mali_kbase.h>
#include <mali_kbase_reg_track.h>
#include <hw_access/mali_kbase_hw_access_regmap.h>
#include <mali_kbase_cache_policy.h>
#include <mali_kbase_hw.h>
#include <tl/mali_kbase_tracepoints.h>
#include <mali_kbase_native_mgm.h>
#include <mali_kbase_mem_pool_group.h>
#include <mmu/mali_kbase_mmu.h>
#include <mali_kbase_config_defaults.h>
#include <mali_kbase_trace_gpu_mem.h>
#include <linux/version_compat_defs.h>

#define VA_REGION_SLAB_NAME_PREFIX "va-region-slab-"
#define VA_REGION_SLAB_NAME_SIZE (DEVNAME_SIZE + sizeof(VA_REGION_SLAB_NAME_PREFIX) + 1)

#if MALI_JIT_PRESSURE_LIMIT_BASE

/*
 * Alignment of objects allocated by the GPU inside a just-in-time memory
 * region whose size is given by an end address
 *
 * This is the alignment of objects allocated by the GPU, but possibly not
 * fully written to. When taken into account with
 * KBASE_GPU_ALLOCATED_OBJECT_MAX_BYTES it gives the maximum number of bytes
 * that the JIT memory report size can exceed the actual backed memory size.
 */
#define KBASE_GPU_ALLOCATED_OBJECT_ALIGN_BYTES (128u)

/*
 * Maximum size of objects allocated by the GPU inside a just-in-time memory
 * region whose size is given by an end address
 *
 * This is the maximum size of objects allocated by the GPU, but possibly not
 * fully written to. When taken into account with
 * KBASE_GPU_ALLOCATED_OBJECT_ALIGN_BYTES it gives the maximum number of bytes
 * that the JIT memory report size can exceed the actual backed memory size.
 */
#define KBASE_GPU_ALLOCATED_OBJECT_MAX_BYTES (512u)

#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

/*
 * kbase_large_page_state - flag indicating kbase handling of large pages
 * @LARGE_PAGE_AUTO: large pages get selected if the GPU hardware supports them
 * @LARGE_PAGE_ON: large pages get selected regardless of GPU support
 * @LARGE_PAGE_OFF: large pages get disabled regardless of GPU support
 */
enum kbase_large_page_state { LARGE_PAGE_AUTO, LARGE_PAGE_ON, LARGE_PAGE_OFF, LARGE_PAGE_MAX };

static enum kbase_large_page_state large_page_conf =
	IS_ENABLED(CONFIG_LARGE_PAGE_SUPPORT) ? LARGE_PAGE_AUTO : LARGE_PAGE_OFF;

static int set_large_page_conf(const char *val, const struct kernel_param *kp)
{
	char *user_input = strstrip((char *)val);

	if (!IS_ENABLED(CONFIG_LARGE_PAGE_SUPPORT))
		return 0;

	if (!strcmp(user_input, "auto"))
		large_page_conf = LARGE_PAGE_AUTO;
	else if (!strcmp(user_input, "on"))
		large_page_conf = LARGE_PAGE_ON;
	else if (!strcmp(user_input, "off"))
		large_page_conf = LARGE_PAGE_OFF;

	return 0;
}

static int get_large_page_conf(char *buffer, const struct kernel_param *kp)
{
	char *out;

	switch (large_page_conf) {
	case LARGE_PAGE_AUTO:
		out = "auto";
		break;
	case LARGE_PAGE_ON:
		out = "on";
		break;
	case LARGE_PAGE_OFF:
		out = "off";
		break;
	default:
		out = "default";
		break;
	}

	return scnprintf(buffer, PAGE_SIZE, "%s\n", out);
}

static const struct kernel_param_ops large_page_config_params = {
	.set = set_large_page_conf,
	.get = get_large_page_conf,
};

module_param_cb(large_page_conf, &large_page_config_params, NULL, 0444);
__MODULE_PARM_TYPE(large_page_conf, "charp");
MODULE_PARM_DESC(large_page_conf, "User override for large page usage on supporting platforms.");

/**
 * kbasep_mem_page_size_init - Initialize kbase device for 2MB page.
 * @kbdev: Pointer to the device.
 *
 * This function must be called only when a kbase device is initialized.
 */
static void kbasep_mem_page_size_init(struct kbase_device *kbdev)
{
	if (!IS_ENABLED(CONFIG_LARGE_PAGE_SUPPORT)) {
		kbdev->pagesize_2mb = false;
		dev_info(kbdev->dev, "Large page support was disabled at compile-time!");
		return;
	}

	switch (large_page_conf) {
	case LARGE_PAGE_AUTO: {
		kbdev->pagesize_2mb = kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_LARGE_PAGE_ALLOC);
		dev_info(kbdev->dev, "Large page allocation set to %s after hardware feature check",
			 kbdev->pagesize_2mb ? "true" : "false");
		break;
	}
	case LARGE_PAGE_ON: {
		kbdev->pagesize_2mb = true;
		if (!kbase_hw_has_feature(kbdev, BASE_HW_FEATURE_LARGE_PAGE_ALLOC))
			dev_warn(kbdev->dev,
				 "Enabling large page allocations on unsupporting GPU!");
		else
			dev_info(kbdev->dev, "Large page allocation override: turned on\n");
		break;
	}
	case LARGE_PAGE_OFF: {
		kbdev->pagesize_2mb = false;
		dev_info(kbdev->dev, "Large page allocation override: turned off\n");
		break;
	}
	default: {
		kbdev->pagesize_2mb = false;
		dev_info(kbdev->dev, "Invalid large page override, turning off large pages\n");
		break;
	}
	}

	/* We want the final state of the setup to be reflected in the module parameter,
	 * so that userspace could read it to figure out the state of the configuration
	 * if necessary.
	 */
	if (kbdev->pagesize_2mb)
		large_page_conf = LARGE_PAGE_ON;
	else
		large_page_conf = LARGE_PAGE_OFF;
}

int kbase_mem_init(struct kbase_device *kbdev)
{
	int err = 0;
	struct kbasep_mem_device *memdev;
	char va_region_slab_name[VA_REGION_SLAB_NAME_SIZE];
#if IS_ENABLED(CONFIG_OF)
	struct device_node *mgm_node = NULL;
#endif

	KBASE_DEBUG_ASSERT(kbdev);

	memdev = &kbdev->memdev;

	kbasep_mem_page_size_init(kbdev);

	scnprintf(va_region_slab_name, VA_REGION_SLAB_NAME_SIZE, VA_REGION_SLAB_NAME_PREFIX "%s",
		  kbdev->devname);

	/* Initialize slab cache for kbase_va_regions */
	kbdev->va_region_slab =
		kmem_cache_create(va_region_slab_name, sizeof(struct kbase_va_region), 0, 0, NULL);
	if (kbdev->va_region_slab == NULL) {
		dev_err(kbdev->dev, "Failed to create va_region_slab\n");
		return -ENOMEM;
	}

	kbase_mem_migrate_init(kbdev);
	kbase_mem_pool_group_config_set_max_size(&kbdev->mem_pool_defaults,
						 KBASE_MEM_POOL_MAX_SIZE_KCTX);

	spin_lock_init(&kbdev->gpu_mem_usage_lock);
	kbdev->process_root = RB_ROOT;
	kbdev->dma_buf_root = RB_ROOT;
	mutex_init(&kbdev->dma_buf_lock);

#ifdef IR_THRESHOLD
	atomic_set(&memdev->ir_threshold, IR_THRESHOLD);
#else
	atomic_set(&memdev->ir_threshold, DEFAULT_IR_THRESHOLD);
#endif

	kbdev->mgm_dev = &kbase_native_mgm_dev;

#if IS_ENABLED(CONFIG_OF)
	/* Check to see whether or not a platform-specific memory group manager
	 * is configured and available.
	 */
	mgm_node = of_parse_phandle(kbdev->dev->of_node, "physical-memory-group-manager", 0);
	if (!mgm_node) {
		dev_info(kbdev->dev, "No memory group manager is configured\n");
	} else {
		struct platform_device *const pdev = of_find_device_by_node(mgm_node);

		if (!pdev) {
			dev_err(kbdev->dev, "The configured memory group manager was not found\n");
		} else {
			kbdev->mgm_dev = platform_get_drvdata(pdev);
			if (!kbdev->mgm_dev) {
				dev_info(kbdev->dev, "Memory group manager is not ready\n");
				err = -EPROBE_DEFER;
			} else if (!try_module_get(kbdev->mgm_dev->owner)) {
				dev_err(kbdev->dev, "Failed to get memory group manger module\n");
				err = -ENODEV;
				kbdev->mgm_dev = NULL;
			} else {
				dev_info(kbdev->dev, "Memory group manager successfully loaded\n");
			}
		}
		of_node_put(mgm_node);
	}
#endif

	if (likely(!err)) {
		struct kbase_mem_pool_group_config mem_pool_defaults;

		kbase_mem_pool_group_config_set_max_size(&mem_pool_defaults,
							 KBASE_MEM_POOL_MAX_SIZE_KBDEV);

		err = kbase_mem_pool_group_init(&kbdev->mem_pools, kbdev, &mem_pool_defaults, NULL);
	}

	return err;
}

void kbase_mem_halt(struct kbase_device *kbdev)
{
	CSTD_UNUSED(kbdev);
}

void kbase_mem_term(struct kbase_device *kbdev)
{
	struct kbasep_mem_device *memdev;
	int pages;

	KBASE_DEBUG_ASSERT(kbdev);

	memdev = &kbdev->memdev;

	pages = atomic_read(&memdev->used_pages);
	if (pages != 0)
		dev_warn(kbdev->dev, "%s: %d pages in use!\n", __func__, pages);

	kbase_mem_pool_group_term(&kbdev->mem_pools);

	kbase_mem_migrate_term(kbdev);

	kmem_cache_destroy(kbdev->va_region_slab);
	kbdev->va_region_slab = NULL;

	WARN_ON(kbdev->total_gpu_pages);
	WARN_ON(!RB_EMPTY_ROOT(&kbdev->process_root));
	WARN_ON(!RB_EMPTY_ROOT(&kbdev->dma_buf_root));
	mutex_destroy(&kbdev->dma_buf_lock);

	if (kbdev->mgm_dev)
		module_put(kbdev->mgm_dev->owner);
}
KBASE_EXPORT_TEST_API(kbase_mem_term);

int kbase_gpu_mmap(struct kbase_context *kctx, struct kbase_va_region *reg, u64 addr,
		   size_t nr_pages, size_t align, enum kbase_caller_mmu_sync_info mmu_sync_info)
{
	int err;
	size_t i = 0;
	unsigned long attr;
	unsigned long mask = ~KBASE_REG_MEMATTR_MASK;
	unsigned long gwt_mask = ~0UL;
	int group_id;
	struct kbase_mem_phy_alloc *alloc;

#ifdef CONFIG_MALI_CINSTR_GWT
	if (kctx->gwt_enabled)
		gwt_mask = ~KBASE_REG_GPU_WR;
#endif

	if ((kctx->kbdev->system_coherency == COHERENCY_ACE) && (reg->flags & KBASE_REG_SHARE_BOTH))
		attr = KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_OUTER_WA);
	else
		attr = KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_WRITE_ALLOC);

	KBASE_DEBUG_ASSERT(kctx != NULL);
	KBASE_DEBUG_ASSERT(reg != NULL);

	err = kbase_add_va_region(kctx, reg, addr, nr_pages, align);
	if (err)
		return err;

	alloc = reg->gpu_alloc;
	group_id = alloc->group_id;

	if (reg->gpu_alloc->type == KBASE_MEM_TYPE_ALIAS) {
		u64 const stride = alloc->imported.alias.stride;

		KBASE_DEBUG_ASSERT(alloc->imported.alias.aliased);
		for (i = 0; i < alloc->imported.alias.nents; i++) {
			if (alloc->imported.alias.aliased[i].alloc) {
				err = kbase_mmu_insert_aliased_pages(
					kctx->kbdev, &kctx->mmu, reg->start_pfn + (i * stride),
					alloc->imported.alias.aliased[i].alloc->pages +
						alloc->imported.alias.aliased[i].offset,
					alloc->imported.alias.aliased[i].length,
					reg->flags & gwt_mask, kctx->as_nr, group_id, mmu_sync_info,
					NULL);
				if (err)
					goto bad_aliased_insert;

				/* Note: mapping count is tracked at alias
				 * creation time
				 */
			} else {
				err = kbase_mmu_insert_single_aliased_page(
					kctx, reg->start_pfn + i * stride, kctx->aliasing_sink_page,
					alloc->imported.alias.aliased[i].length,
					(reg->flags & mask & gwt_mask) | attr, group_id,
					mmu_sync_info);

				if (err)
					goto bad_aliased_insert;
			}
		}
	} else {
		/* Imported user buffers have dedicated state transitions.
		 * The intended outcome is still the same: creating a GPU mapping,
		 * but only if the user buffer has already advanced to the expected
		 * state and has acquired enough resources.
		 */
		if (reg->gpu_alloc->type == KBASE_MEM_TYPE_IMPORTED_USER_BUF) {
			/* The region is always supposed to be EMPTY at this stage.
			 * If the region is coherent with the CPU then all resources are
			 * acquired, including physical pages and DMA addresses, and a
			 * GPU mapping is created.
			 */
			switch (alloc->imported.user_buf.state) {
			case KBASE_USER_BUF_STATE_EMPTY: {
				if (reg->flags & KBASE_REG_SHARE_BOTH) {
					err = kbase_user_buf_from_empty_to_gpu_mapped(kctx, reg);
					reg->gpu_alloc->imported.user_buf
						.current_mapping_usage_count++;
				}
				break;
			}
			default: {
				WARN(1, "Unexpected state %d for imported user buffer\n",
				     alloc->imported.user_buf.state);
				break;
			}
			}
		} else if (reg->gpu_alloc->type == KBASE_MEM_TYPE_IMPORTED_UMM) {
			err = kbase_mmu_insert_pages_skip_status_update(
				kctx->kbdev, &kctx->mmu, reg->start_pfn,
				kbase_get_gpu_phy_pages(reg), kbase_reg_current_backed_size(reg),
				reg->flags & gwt_mask, kctx->as_nr, group_id, mmu_sync_info, reg);
		} else {
			err = kbase_mmu_insert_pages(kctx->kbdev, &kctx->mmu, reg->start_pfn,
						     kbase_get_gpu_phy_pages(reg),
						     kbase_reg_current_backed_size(reg),
						     reg->flags & gwt_mask, kctx->as_nr, group_id,
						     mmu_sync_info, reg);
		}

		if (err)
			goto bad_insert;
		kbase_mem_phy_alloc_gpu_mapped(alloc);
	}

	if (reg->flags & KBASE_REG_IMPORT_PAD && !WARN_ON(reg->nr_pages < reg->gpu_alloc->nents) &&
	    reg->gpu_alloc->type == KBASE_MEM_TYPE_IMPORTED_UMM &&
	    reg->gpu_alloc->imported.umm.current_mapping_usage_count) {
		/* For padded imported dma-buf or user-buf memory, map the dummy
		 * aliasing page from the end of the imported pages, to the end of
		 * the region using a read only mapping.
		 *
		 * Only map when it's imported dma-buf memory that is currently
		 * mapped.
		 *
		 * Assume reg->gpu_alloc->nents is the number of actual pages
		 * in the dma-buf memory.
		 */
		err = kbase_mmu_insert_single_imported_page(
			kctx, reg->start_pfn + reg->gpu_alloc->nents, kctx->aliasing_sink_page,
			reg->nr_pages - reg->gpu_alloc->nents,
			(reg->flags | KBASE_REG_GPU_RD) & ~KBASE_REG_GPU_WR, KBASE_MEM_GROUP_SINK,
			mmu_sync_info);
		if (err)
			goto bad_insert;
	}

	return err;

bad_aliased_insert:
	while (i-- > 0) {
		struct tagged_addr *phys_alloc = NULL;
		u64 const stride = alloc->imported.alias.stride;

		if (alloc->imported.alias.aliased[i].alloc != NULL)
			phys_alloc = alloc->imported.alias.aliased[i].alloc->pages +
				     alloc->imported.alias.aliased[i].offset;

		kbase_mmu_teardown_pages(kctx->kbdev, &kctx->mmu, reg->start_pfn + (i * stride),
					 phys_alloc, alloc->imported.alias.aliased[i].length,
					 alloc->imported.alias.aliased[i].length, kctx->as_nr);
	}
bad_insert:
	kbase_remove_va_region(kctx->kbdev, reg);

	return err;
}

KBASE_EXPORT_TEST_API(kbase_gpu_mmap);

static void kbase_user_buf_unmap(struct kbase_context *kctx, struct kbase_va_region *reg);

int kbase_gpu_munmap(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	int err = 0;
	struct kbase_mem_phy_alloc *alloc;

	if (reg->start_pfn == 0)
		return 0;

	if (!reg->gpu_alloc)
		return -EINVAL;

	alloc = reg->gpu_alloc;

	/* Tear down GPU page tables, depending on memory type. */
	switch (alloc->type) {
	case KBASE_MEM_TYPE_ALIAS: {
		size_t i = 0;
		/* Due to the way the number of valid PTEs and ATEs are tracked
			 * currently, only the GPU virtual range that is backed & mapped
			 * should be passed to the page teardown function, hence individual
			 * aliased regions needs to be unmapped separately.
			 */
		for (i = 0; i < alloc->imported.alias.nents; i++) {
			struct tagged_addr *phys_alloc = NULL;
			int err_loop;

			if (alloc->imported.alias.aliased[i].alloc != NULL)
				phys_alloc = alloc->imported.alias.aliased[i].alloc->pages +
					     alloc->imported.alias.aliased[i].offset;

			err_loop = kbase_mmu_teardown_pages(
				kctx->kbdev, &kctx->mmu,
				reg->start_pfn + (i * alloc->imported.alias.stride), phys_alloc,
				alloc->imported.alias.aliased[i].length,
				alloc->imported.alias.aliased[i].length, kctx->as_nr);

			if (WARN_ON_ONCE(err_loop))
				err = err_loop;
		}
	} break;
	case KBASE_MEM_TYPE_IMPORTED_UMM: {
		size_t nr_phys_pages = reg->nr_pages;
		size_t nr_virt_pages = reg->nr_pages;
		/* If the region has import padding and falls under the threshold for
			 * issuing a partial GPU cache flush, we want to reduce the number of
			 * physical pages that get flushed.

			 * This is symmetric with case of mapping the memory, which first maps
			 * each imported physical page to a separate virtual page, and then
			 * maps the single aliasing sink page to each of the virtual padding
			 * pages.
			 */
		if (reg->flags & KBASE_REG_IMPORT_PAD)
			nr_phys_pages = alloc->nents + 1;

		err = kbase_mmu_teardown_imported_pages(kctx->kbdev, &kctx->mmu, reg->start_pfn,
							alloc->pages, nr_phys_pages, nr_virt_pages,
							kctx->as_nr);
	} break;
	case KBASE_MEM_TYPE_IMPORTED_USER_BUF: {
		/* Progress through all stages to destroy the GPU mapping and release
		 * all resources.
		 */
		switch (alloc->imported.user_buf.state) {
		case KBASE_USER_BUF_STATE_GPU_MAPPED: {
			alloc->imported.user_buf.current_mapping_usage_count = 0;
			kbase_mem_phy_alloc_ref_read(alloc) ?
				      kbase_user_buf_from_gpu_mapped_to_pinned(kctx, reg) :
				      kbase_user_buf_from_gpu_mapped_to_empty(kctx, reg);
			break;
		}
		case KBASE_USER_BUF_STATE_DMA_MAPPED: {
			kbase_mem_phy_alloc_ref_read(alloc) ?
				      kbase_user_buf_from_dma_mapped_to_pinned(kctx, reg) :
				      kbase_user_buf_from_dma_mapped_to_empty(kctx, reg);
			break;
		}
		case KBASE_USER_BUF_STATE_PINNED: {
			if (!kbase_mem_phy_alloc_ref_read(alloc))
				kbase_user_buf_from_pinned_to_empty(kctx, reg);
			break;
		}
		case KBASE_USER_BUF_STATE_EMPTY: {
			/* Nothing to do. This is a legal possibility, because an imported
			 * memory handle can be destroyed just after creation without being
			 * used.
			 */
			break;
		}
		default: {
			WARN(1, "Unexpected state %d for imported user buffer\n",
			     alloc->imported.user_buf.state);
			break;
		}
		}
		break;
	}
	default: {
		size_t nr_reg_pages = kbase_reg_current_backed_size(reg);

		err = kbase_mmu_teardown_pages(kctx->kbdev, &kctx->mmu, reg->start_pfn,
					       alloc->pages, nr_reg_pages, nr_reg_pages,
					       kctx->as_nr);
	} break;
	}

	if (alloc->type != KBASE_MEM_TYPE_ALIAS)
		kbase_mem_phy_alloc_gpu_unmapped(reg->gpu_alloc);

	return err;
}

static struct kbase_cpu_mapping *kbasep_find_enclosing_cpu_mapping(struct kbase_context *kctx,
								   unsigned long uaddr, size_t size,
								   u64 *offset)
{
	struct vm_area_struct *vma;
	struct kbase_cpu_mapping *map;
	unsigned long vm_pgoff_in_region;
	unsigned long vm_off_in_region;
	unsigned long map_start;
	size_t map_size;

	lockdep_assert_held(kbase_mem_get_process_mmap_lock());

	if ((uintptr_t)uaddr + size < (uintptr_t)uaddr) /* overflow check */
		return NULL;

	vma = find_vma_intersection(current->mm, uaddr, uaddr + size);

	if (!vma || vma->vm_start > uaddr)
		return NULL;
	if (vma->vm_ops != &kbase_vm_ops)
		/* Not ours! */
		return NULL;

	map = vma->vm_private_data;

	if (map->kctx != kctx)
		/* Not from this context! */
		return NULL;

	vm_pgoff_in_region = vma->vm_pgoff - map->region->start_pfn;
	vm_off_in_region = vm_pgoff_in_region << PAGE_SHIFT;
	map_start = vma->vm_start - vm_off_in_region;
	map_size = map->region->nr_pages << PAGE_SHIFT;

	if ((uaddr + size) > (map_start + map_size))
		/* Not within the CPU mapping */
		return NULL;

	*offset = (uaddr - vma->vm_start) + vm_off_in_region;

	return map;
}

int kbasep_find_enclosing_cpu_mapping_offset(struct kbase_context *kctx, unsigned long uaddr,
					     size_t size, u64 *offset)
{
	struct kbase_cpu_mapping *map;

	kbase_os_mem_map_lock(kctx);

	map = kbasep_find_enclosing_cpu_mapping(kctx, uaddr, size, offset);

	kbase_os_mem_map_unlock(kctx);

	if (!map)
		return -EINVAL;

	return 0;
}

KBASE_EXPORT_TEST_API(kbasep_find_enclosing_cpu_mapping_offset);

int kbasep_find_enclosing_gpu_mapping_start_and_offset(struct kbase_context *kctx, u64 gpu_addr,
						       size_t size, u64 *start, u64 *offset)
{
	struct kbase_va_region *region;

	kbase_gpu_vm_lock(kctx);

	region = kbase_region_tracker_find_region_enclosing_address(kctx, gpu_addr);

	if (!region) {
		kbase_gpu_vm_unlock(kctx);
		return -EINVAL;
	}

	*start = region->start_pfn << PAGE_SHIFT;

	*offset = gpu_addr - *start;

	if (((region->start_pfn + region->nr_pages) << PAGE_SHIFT) < (gpu_addr + size)) {
		kbase_gpu_vm_unlock(kctx);
		return -EINVAL;
	}

	kbase_gpu_vm_unlock(kctx);

	return 0;
}

KBASE_EXPORT_TEST_API(kbasep_find_enclosing_gpu_mapping_start_and_offset);

void kbase_sync_single(struct kbase_context *kctx, struct tagged_addr t_cpu_pa,
		       struct tagged_addr t_gpu_pa, off_t offset, size_t size,
		       enum kbase_sync_type sync_fn)
{
	struct page *cpu_page;
	phys_addr_t cpu_pa = as_phys_addr_t(t_cpu_pa);
	phys_addr_t gpu_pa = as_phys_addr_t(t_gpu_pa);

	cpu_page = pfn_to_page(PFN_DOWN(cpu_pa));

	if (likely(cpu_pa == gpu_pa)) {
		dma_addr_t dma_addr;

		WARN_ON(!cpu_page);
		WARN_ON((size_t)offset + size > PAGE_SIZE);

		dma_addr = kbase_dma_addr_from_tagged(t_cpu_pa) + (dma_addr_t)offset;

		if (sync_fn == KBASE_SYNC_TO_CPU)
			dma_sync_single_for_cpu(kctx->kbdev->dev, dma_addr, size,
						DMA_BIDIRECTIONAL);
		else if (sync_fn == KBASE_SYNC_TO_DEVICE)
			dma_sync_single_for_device(kctx->kbdev->dev, dma_addr, size,
						   DMA_BIDIRECTIONAL);
	} else {
		void *src = NULL;
		void *dst = NULL;
		struct page *gpu_page;
		dma_addr_t dma_addr;

		if (WARN(!gpu_pa, "No GPU PA found for infinite cache op"))
			return;

		gpu_page = pfn_to_page(PFN_DOWN(gpu_pa));
		dma_addr = kbase_dma_addr_from_tagged(t_gpu_pa) + (dma_addr_t)offset;

		if (sync_fn == KBASE_SYNC_TO_DEVICE) {
			src = ((unsigned char *)kbase_kmap(cpu_page)) + offset;
			dst = ((unsigned char *)kbase_kmap(gpu_page)) + offset;
		} else if (sync_fn == KBASE_SYNC_TO_CPU) {
			dma_sync_single_for_cpu(kctx->kbdev->dev, dma_addr, size,
						DMA_BIDIRECTIONAL);
			src = ((unsigned char *)kbase_kmap(gpu_page)) + offset;
			dst = ((unsigned char *)kbase_kmap(cpu_page)) + offset;
		}

		memcpy(dst, src, size);
		kbase_kunmap(gpu_page, src);
		kbase_kunmap(cpu_page, dst);
		if (sync_fn == KBASE_SYNC_TO_DEVICE)
			dma_sync_single_for_device(kctx->kbdev->dev, dma_addr, size,
						   DMA_BIDIRECTIONAL);
	}
}

static int kbase_do_syncset(struct kbase_context *kctx, struct basep_syncset *sset,
			    enum kbase_sync_type sync_fn)
{
	int err = 0;
	struct kbase_va_region *reg;
	struct kbase_cpu_mapping *map;
	unsigned long start;
	size_t size;
	struct tagged_addr *cpu_pa;
	struct tagged_addr *gpu_pa;
	u64 page_off, page_count;
	u64 i;
	u64 offset;
	size_t sz;

	kbase_os_mem_map_lock(kctx);
	kbase_gpu_vm_lock(kctx);

	/* find the region where the virtual address is contained */
	reg = kbase_region_tracker_find_region_enclosing_address(kctx,
								 sset->mem_handle.basep.handle);
	if (kbase_is_region_invalid_or_free(reg)) {
		dev_warn(kctx->kbdev->dev, "Can't find a valid region at VA 0x%016llX",
			 sset->mem_handle.basep.handle);
		err = -EINVAL;
		goto out_unlock;
	}

	/*
	 * Handle imported memory before checking for KBASE_REG_CPU_CACHED. The
	 * CPU mapping cacheability is defined by the owner of the imported
	 * memory, and not by kbase, therefore we must assume that any imported
	 * memory may be cached.
	 */
	if (kbase_mem_is_imported(reg->gpu_alloc->type)) {
		err = kbase_mem_do_sync_imported(kctx, reg, sync_fn);
		goto out_unlock;
	}

	if (!(reg->flags & KBASE_REG_CPU_CACHED))
		goto out_unlock;

	start = (uintptr_t)sset->user_addr;
	size = (size_t)sset->size;

	map = kbasep_find_enclosing_cpu_mapping(kctx, start, size, &offset);
	if (!map) {
		dev_warn(kctx->kbdev->dev, "Can't find CPU mapping 0x%016lX for VA 0x%016llX",
			 start, sset->mem_handle.basep.handle);
		err = -EINVAL;
		goto out_unlock;
	}

	page_off = offset >> PAGE_SHIFT;
	offset &= ~PAGE_MASK;
	page_count = (size + offset + (PAGE_SIZE - 1)) >> PAGE_SHIFT;
	cpu_pa = kbase_get_cpu_phy_pages(reg);
	gpu_pa = kbase_get_gpu_phy_pages(reg);

	if (page_off > reg->nr_pages || page_off + page_count > reg->nr_pages) {
		/* Sync overflows the region */
		err = -EINVAL;
		goto out_unlock;
	}

	if (page_off >= reg->gpu_alloc->nents) {
		/* Start of sync range is outside the physically backed region
		 * so nothing to do
		 */
		goto out_unlock;
	}

	/* Sync first page */
	sz = MIN(((size_t)PAGE_SIZE - offset), size);

	kbase_sync_single(kctx, cpu_pa[page_off], gpu_pa[page_off], (off_t)offset, sz, sync_fn);

	/* Calculate the size for last page */
	sz = ((start + size - 1) & ~PAGE_MASK) + 1;

	/* Limit the sync range to the physically backed region */
	if (page_off + page_count > reg->gpu_alloc->nents) {
		page_count = reg->gpu_alloc->nents - page_off;
		/* Since we limit the pages then size for last page
		 * is the whole page
		 */
		sz = PAGE_SIZE;
	}

	/* Sync middle pages (if any) */
	for (i = 1; page_count > 2 && i < page_count - 1; i++) {
		kbase_sync_single(kctx, cpu_pa[page_off + i], gpu_pa[page_off + i], 0, PAGE_SIZE,
				  sync_fn);
	}

	/* Sync last page (if any) */
	if (page_count > 1) {
		kbase_sync_single(kctx, cpu_pa[page_off + page_count - 1],
				  gpu_pa[page_off + page_count - 1], 0, sz, sync_fn);
	}

out_unlock:
	kbase_gpu_vm_unlock(kctx);
	kbase_os_mem_map_unlock(kctx);
	return err;
}

int kbase_sync_now(struct kbase_context *kctx, struct basep_syncset *sset)
{
	int err = -EINVAL;

	KBASE_DEBUG_ASSERT(kctx != NULL);
	KBASE_DEBUG_ASSERT(sset != NULL);

	if (sset->mem_handle.basep.handle & ~PAGE_MASK) {
		dev_warn(kctx->kbdev->dev, "mem_handle: passed parameter is invalid");
		return -EINVAL;
	}

	switch (sset->type) {
	case BASE_SYNCSET_OP_MSYNC:
		err = kbase_do_syncset(kctx, sset, KBASE_SYNC_TO_DEVICE);
		break;

	case BASE_SYNCSET_OP_CSYNC:
		err = kbase_do_syncset(kctx, sset, KBASE_SYNC_TO_CPU);
		break;

	default:
		dev_warn(kctx->kbdev->dev, "Unknown msync op %d\n", sset->type);
		break;
	}

	return err;
}

KBASE_EXPORT_TEST_API(kbase_sync_now);

/* vm lock must be held */
int kbase_mem_free_region(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	int err;

	KBASE_DEBUG_ASSERT(kctx != NULL);
	KBASE_DEBUG_ASSERT(reg != NULL);
	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	lockdep_assert_held(&kctx->reg_lock);

	if (kbase_va_region_is_no_user_free(reg)) {
		dev_warn(kctx->kbdev->dev,
			 "Attempt to free GPU memory whose freeing by user space is forbidden!\n");
		return -EINVAL;
	}

	/* If a region has been made evictable then we must unmake it
	 * before trying to free it.
	 * If the memory hasn't been reclaimed it will be unmapped and freed
	 * below, if it has been reclaimed then the operations below are no-ops.
	 */
	if (reg->flags & KBASE_REG_DONT_NEED) {
		WARN_ON(reg->cpu_alloc->type != KBASE_MEM_TYPE_NATIVE);
		mutex_lock(&kctx->jit_evict_lock);
		/* Unlink the physical allocation before unmaking it evictable so
		 * that the allocation isn't grown back to its last backed size
		 * as we're going to unmap it anyway.
		 */
		reg->cpu_alloc->reg = NULL;
		if (reg->cpu_alloc != reg->gpu_alloc)
			reg->gpu_alloc->reg = NULL;
		mutex_unlock(&kctx->jit_evict_lock);
		kbase_mem_evictable_unmake(reg->gpu_alloc);
	}

	err = kbase_gpu_munmap(kctx, reg);
	if (err) {
		dev_warn(kctx->kbdev->dev, "Could not unmap from the GPU...\n");
		goto out;
	}

#if MALI_USE_CSF
	if (((kbase_bits_to_zone(reg->flags)) == FIXED_VA_ZONE) ||
	    ((kbase_bits_to_zone(reg->flags)) == EXEC_FIXED_VA_ZONE)) {
		if (reg->flags & KBASE_REG_FIXED_ADDRESS)
			atomic64_dec(&kctx->num_fixed_allocs);
		else
			atomic64_dec(&kctx->num_fixable_allocs);
	}
#endif

	/* This will also free the physical pages */
	kbase_free_alloced_region(reg);

out:
	return err;
}

KBASE_EXPORT_TEST_API(kbase_mem_free_region);

/**
 * kbase_mem_free - Free the region from the GPU and unregister it.
 *
 * @kctx:  KBase context
 * @gpu_addr: GPU address to free
 *
 * This function implements the free operation on a memory segment.
 * It will loudly fail if called with outstanding mappings.
 *
 * Return: 0 on success.
 */
int kbase_mem_free(struct kbase_context *kctx, u64 gpu_addr)
{
	int err = 0;
	struct kbase_va_region *reg;

	KBASE_DEBUG_ASSERT(kctx != NULL);
	dev_dbg(kctx->kbdev->dev, "%s 0x%llx in kctx %pK\n", __func__, gpu_addr, (void *)kctx);

	if ((gpu_addr & ~PAGE_MASK) && (gpu_addr >= PAGE_SIZE)) {
		dev_warn(kctx->kbdev->dev, "%s: gpu_addr parameter is invalid", __func__);
		return -EINVAL;
	}

	if (gpu_addr == 0) {
		dev_warn(
			kctx->kbdev->dev,
			"gpu_addr 0 is reserved for the ringbuffer and it's an error to try to free it using %s\n",
			__func__);
		return -EINVAL;
	}
	kbase_gpu_vm_lock(kctx);

	if (gpu_addr >= BASE_MEM_COOKIE_BASE && gpu_addr < BASE_MEM_FIRST_FREE_ADDRESS) {
		unsigned int cookie = PFN_DOWN(gpu_addr - BASE_MEM_COOKIE_BASE);

		reg = kctx->pending_regions[cookie];
		if (!reg) {
			err = -EINVAL;
			goto out_unlock;
		}

		/* ask to unlink the cookie as we'll free it */

		kctx->pending_regions[cookie] = NULL;
		bitmap_set(kctx->cookies, cookie, 1);

		kbase_free_alloced_region(reg);
	} else {
		/* A real GPU va */
		/* Validate the region */
		reg = kbase_region_tracker_find_region_base_address(kctx, gpu_addr);
		if (kbase_is_region_invalid_or_free(reg)) {
			dev_warn(kctx->kbdev->dev, "%s called with nonexistent gpu_addr 0x%llX",
				 __func__, gpu_addr);
			err = -EINVAL;
			goto out_unlock;
		}

		if ((kbase_bits_to_zone(reg->flags)) == SAME_VA_ZONE) {
			/* SAME_VA must be freed through munmap */
			dev_warn(kctx->kbdev->dev, "%s called on SAME_VA memory 0x%llX", __func__,
				 gpu_addr);
			err = -EINVAL;
			goto out_unlock;
		}
		err = kbase_mem_free_region(kctx, reg);
	}

out_unlock:
	kbase_gpu_vm_unlock(kctx);
	return err;
}

KBASE_EXPORT_TEST_API(kbase_mem_free);

int kbase_update_region_flags(struct kbase_context *kctx, struct kbase_va_region *reg,
			      unsigned long flags)
{
	KBASE_DEBUG_ASSERT(reg != NULL);
	KBASE_DEBUG_ASSERT((flags & ~((1ul << BASE_MEM_FLAGS_NR_BITS) - 1)) == 0);

	reg->flags |= kbase_cache_enabled(flags, reg->nr_pages);
	/* all memory is now growable */
	reg->flags |= KBASE_REG_GROWABLE;

	if (flags & BASE_MEM_GROW_ON_GPF)
		reg->flags |= KBASE_REG_PF_GROW;

	if (flags & BASE_MEM_PROT_CPU_WR)
		reg->flags |= KBASE_REG_CPU_WR;

	if (flags & BASE_MEM_PROT_CPU_RD)
		reg->flags |= KBASE_REG_CPU_RD;

	if (flags & BASE_MEM_PROT_GPU_WR)
		reg->flags |= KBASE_REG_GPU_WR;

	if (flags & BASE_MEM_PROT_GPU_RD)
		reg->flags |= KBASE_REG_GPU_RD;

	if (0 == (flags & BASE_MEM_PROT_GPU_EX))
		reg->flags |= KBASE_REG_GPU_NX;

	if (!kbase_device_is_cpu_coherent(kctx->kbdev)) {
		if (flags & BASE_MEM_COHERENT_SYSTEM_REQUIRED && !(flags & BASE_MEM_UNCACHED_GPU))
			return -EINVAL;
	} else if (flags & (BASE_MEM_COHERENT_SYSTEM | BASE_MEM_COHERENT_SYSTEM_REQUIRED)) {
		reg->flags |= KBASE_REG_SHARE_BOTH;
	}

	if (!(reg->flags & KBASE_REG_SHARE_BOTH) && flags & BASE_MEM_COHERENT_LOCAL) {
		reg->flags |= KBASE_REG_SHARE_IN;
	}

#if !MALI_USE_CSF
	if (flags & BASE_MEM_TILER_ALIGN_TOP)
		reg->flags |= KBASE_REG_TILER_ALIGN_TOP;
#endif /* !MALI_USE_CSF */

#if MALI_USE_CSF
	if (flags & BASE_MEM_CSF_EVENT) {
		reg->flags |= KBASE_REG_CSF_EVENT;
		reg->flags |= KBASE_REG_PERMANENT_KERNEL_MAPPING;

		if (!(reg->flags & KBASE_REG_SHARE_BOTH)) {
			/* On non coherent platforms need to map as uncached on
			 * both sides.
			 */
			reg->flags &= ~KBASE_REG_CPU_CACHED;
			reg->flags &= ~KBASE_REG_GPU_CACHED;
		}
	}
#endif

	/* Set up default MEMATTR usage */
	if (!(reg->flags & KBASE_REG_GPU_CACHED)) {
		if (kctx->kbdev->mmu_mode->flags & KBASE_MMU_MODE_HAS_NON_CACHEABLE) {
			/* Override shareability, and MEMATTR for uncached */
			reg->flags &= ~(KBASE_REG_SHARE_IN | KBASE_REG_SHARE_BOTH);
			reg->flags |= KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_NON_CACHEABLE);
		} else {
			dev_warn(kctx->kbdev->dev,
				 "Can't allocate GPU uncached memory due to MMU in Legacy Mode\n");
			return -EINVAL;
		}
#if MALI_USE_CSF
	} else if (reg->flags & KBASE_REG_CSF_EVENT) {
		WARN_ON(!(reg->flags & KBASE_REG_SHARE_BOTH));

		reg->flags |= KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_SHARED);
#endif
	} else if (kctx->kbdev->system_coherency == COHERENCY_ACE &&
		   (reg->flags & KBASE_REG_SHARE_BOTH)) {
		reg->flags |= KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_DEFAULT_ACE);
	} else {
		reg->flags |= KBASE_REG_MEMATTR_INDEX(KBASE_MEMATTR_INDEX_DEFAULT);
	}

	if (flags & BASEP_MEM_PERMANENT_KERNEL_MAPPING)
		reg->flags |= KBASE_REG_PERMANENT_KERNEL_MAPPING;

	if (flags & BASEP_MEM_NO_USER_FREE) {
		kbase_gpu_vm_lock(kctx);
		kbase_va_region_no_user_free_inc(reg);
		kbase_gpu_vm_unlock(kctx);
	}

	if (flags & BASE_MEM_GPU_VA_SAME_4GB_PAGE)
		reg->flags |= KBASE_REG_GPU_VA_SAME_4GB_PAGE;

#if MALI_USE_CSF
	if (flags & BASE_MEM_FIXED)
		reg->flags |= KBASE_REG_FIXED_ADDRESS;
#endif

	return 0;
}

static int mem_account_inc(struct kbase_context *kctx, int nr_pages_inc)
{
	int new_page_count = atomic_add_return(nr_pages_inc, &kctx->used_pages);

	atomic_add(nr_pages_inc, &kctx->kbdev->memdev.used_pages);
	kbase_process_page_usage_inc(kctx, nr_pages_inc);
	kbase_trace_gpu_mem_usage_inc(kctx->kbdev, kctx, nr_pages_inc);

	return new_page_count;
}

static int mem_account_dec(struct kbase_context *kctx, int nr_pages_dec)
{
	int new_page_count = atomic_sub_return(nr_pages_dec, &kctx->used_pages);

	atomic_sub(nr_pages_dec, &kctx->kbdev->memdev.used_pages);
	kbase_process_page_usage_dec(kctx, nr_pages_dec);
	kbase_trace_gpu_mem_usage_dec(kctx->kbdev, kctx, nr_pages_dec);

	return new_page_count;
}

int kbase_alloc_phy_pages_helper(struct kbase_mem_phy_alloc *alloc, size_t nr_pages_requested)
{
	int new_page_count __maybe_unused;
	size_t nr_left = nr_pages_requested;
	int res;
	struct kbase_context *kctx;
	struct kbase_device *kbdev;
	struct tagged_addr *tp;
	/* The number of pages to account represents the total amount of memory
	 * actually allocated. If large pages are used, they are taken into account
	 * in full, even if only a fraction of them is used for sub-allocation
	 * to satisfy the memory allocation request.
	 */
	size_t nr_pages_to_account = 0;

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_NATIVE) ||
	    WARN_ON(alloc->imported.native.kctx == NULL) ||
	    WARN_ON(alloc->group_id >= MEMORY_GROUP_MANAGER_NR_GROUPS)) {
		return -EINVAL;
	}

	if (alloc->reg) {
		if (nr_pages_requested > alloc->reg->nr_pages - alloc->nents)
			goto invalid_request;
	}

	kctx = alloc->imported.native.kctx;
	kbdev = kctx->kbdev;

	if (nr_pages_requested == 0)
		goto done; /*nothing to do*/

	/* Increase mm counters before we allocate pages so that this
	 * allocation is visible to the OOM killer. The actual count
	 * of pages will be amended later, if necessary, but for the
	 * moment it is safe to account for the amount initially
	 * requested.
	 */
	new_page_count = mem_account_inc(kctx, nr_pages_requested);
	tp = alloc->pages + alloc->nents;

	/* Check if we have enough pages requested so we can allocate a large
	 * page (512 * 4KB = 2MB )
	 */
	if (kbdev->pagesize_2mb && nr_left >= NUM_PAGES_IN_2MB_LARGE_PAGE) {
		size_t nr_lp = nr_left / NUM_PAGES_IN_2MB_LARGE_PAGE;

		res = kbase_mem_pool_alloc_pages(&kctx->mem_pools.large[alloc->group_id],
						 nr_lp * NUM_PAGES_IN_2MB_LARGE_PAGE, tp, true,
						 kctx->task);

		if (res > 0) {
			nr_left -= (size_t)res;
			tp += res;
			nr_pages_to_account += res;
		}

		if (nr_left) {
			struct kbase_sub_alloc *sa, *temp_sa;

			spin_lock(&kctx->mem_partials_lock);

			list_for_each_entry_safe(sa, temp_sa, &kctx->mem_partials, link) {
				unsigned int pidx = 0;

				while (nr_left) {
					pidx = find_next_zero_bit(
						sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE, pidx);
					bitmap_set(sa->sub_pages, pidx, 1);
					*tp++ = as_tagged_tag(page_to_phys(sa->page + pidx),
							      FROM_PARTIAL);
					nr_left--;

					if (bitmap_full(sa->sub_pages,
							NUM_PAGES_IN_2MB_LARGE_PAGE)) {
						/* unlink from partial list when full */
						list_del_init(&sa->link);
						break;
					}
				}
			}
			spin_unlock(&kctx->mem_partials_lock);
		}

		/* only if we actually have a chunk left <512. If more it indicates
		 * that we couldn't allocate a 2MB above, so no point to retry here.
		 */
		if (nr_left > 0 && nr_left < NUM_PAGES_IN_2MB_LARGE_PAGE) {
			/* create a new partial and suballocate the rest from it */
			struct page *np = NULL;

			do {
				int err;

				np = kbase_mem_pool_alloc(&kctx->mem_pools.large[alloc->group_id]);
				if (np)
					break;

				err = kbase_mem_pool_grow(&kctx->mem_pools.large[alloc->group_id],
							  1, kctx->task);
				if (err)
					break;
			} while (1);

			if (np) {
				size_t i;
				struct kbase_sub_alloc *sa;
				struct page *p;

				sa = kmalloc(sizeof(*sa), GFP_KERNEL);
				if (!sa) {
					kbase_mem_pool_free(&kctx->mem_pools.large[alloc->group_id],
							    np, false);
					goto no_new_partial;
				}

				/* store pointers back to the control struct */
				np->lru.next = (void *)sa;
				for (p = np; p < np + NUM_PAGES_IN_2MB_LARGE_PAGE; p++)
					p->lru.prev = (void *)np;
				INIT_LIST_HEAD(&sa->link);
				bitmap_zero(sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE);
				sa->page = np;

				for (i = 0; i < nr_left; i++)
					*tp++ = as_tagged_tag(page_to_phys(np + i), FROM_PARTIAL);

				bitmap_set(sa->sub_pages, 0, nr_left);
				nr_left = 0;

				/* A large page has been used for a sub-allocation: account
				 * for the whole of the large page, and not just for the
				 * sub-pages that have been used.
				 */
				nr_pages_to_account += NUM_PAGES_IN_2MB_LARGE_PAGE;

				/* expose for later use */
				spin_lock(&kctx->mem_partials_lock);
				list_add(&sa->link, &kctx->mem_partials);
				spin_unlock(&kctx->mem_partials_lock);
			}
		}
	}

no_new_partial:
	if (nr_left) {
		res = kbase_mem_pool_alloc_pages(&kctx->mem_pools.small[alloc->group_id], nr_left,
						 tp, false, kctx->task);
		if (res <= 0)
			goto alloc_failed;

		nr_pages_to_account += res;
	}

	alloc->nents += nr_pages_requested;

	/* Amend the page count with the number of pages actually used. */
	if (nr_pages_to_account > nr_pages_requested)
		new_page_count = mem_account_inc(kctx, nr_pages_to_account - nr_pages_requested);
	else if (nr_pages_to_account < nr_pages_requested)
		new_page_count = mem_account_dec(kctx, nr_pages_requested - nr_pages_to_account);

	KBASE_TLSTREAM_AUX_PAGESALLOC(kbdev, kctx->id, (u64)new_page_count);

done:
	return 0;

alloc_failed:
	/* The first step of error recovery is freeing any allocation that
	 * might have succeeded. The function can be in this condition only
	 * in one case: it tried to allocate a combination of 2 MB and small
	 * pages but only the former step succeeded. In this case, calculate
	 * the number of 2 MB pages to release and free them.
	 */
	if (nr_left != nr_pages_requested) {
		size_t nr_pages_to_free = nr_pages_requested - nr_left;

		alloc->nents += nr_pages_to_free;
		kbase_free_phy_pages_helper(alloc, nr_pages_to_free);
	}

	/* Undo the preliminary memory accounting that was done early on
	 * in the function. If only small pages are used: nr_left is equal
	 * to nr_pages_requested. If a combination of 2 MB and small pages was
	 * attempted: nr_pages_requested is equal to the sum of nr_left
	 * and nr_pages_to_free, and the latter has already been freed above.
	 *
	 * Also notice that there's no need to update the page count
	 * because memory allocation was rolled back.
	 */
	mem_account_dec(kctx, nr_left);

invalid_request:
	return -ENOMEM;
}

static size_t free_partial_locked(struct kbase_context *kctx, struct kbase_mem_pool *pool,
				  struct tagged_addr tp)
{
	struct page *p, *head_page;
	struct kbase_sub_alloc *sa;
	size_t nr_pages_to_account = 0;

	lockdep_assert_held(&pool->pool_lock);
	lockdep_assert_held(&kctx->mem_partials_lock);

	p = as_page(tp);
	head_page = (struct page *)p->lru.prev;
	sa = (struct kbase_sub_alloc *)head_page->lru.next;
	clear_bit(p - head_page, sa->sub_pages);
	if (bitmap_empty(sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE)) {
		list_del(&sa->link);
		kbase_mem_pool_free_locked(pool, head_page, true);
		kfree(sa);
		nr_pages_to_account = NUM_PAGES_IN_2MB_LARGE_PAGE;
	} else if (bitmap_weight(sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE) ==
		   NUM_PAGES_IN_2MB_LARGE_PAGE - 1) {
		/* expose the partial again */
		list_add(&sa->link, &kctx->mem_partials);
	}

	return nr_pages_to_account;
}

struct tagged_addr *kbase_alloc_phy_pages_helper_locked(struct kbase_mem_phy_alloc *alloc,
							struct kbase_mem_pool *pool,
							size_t nr_pages_requested,
							struct kbase_sub_alloc **prealloc_sa)
{
	int new_page_count __maybe_unused;
	size_t nr_left = nr_pages_requested;
	int res;
	struct kbase_context *kctx;
	struct kbase_device *kbdev;
	struct tagged_addr *tp;
	struct tagged_addr *new_pages = NULL;
	/* The number of pages to account represents the total amount of memory
	 * actually allocated. If large pages are used, they are taken into account
	 * in full, even if only a fraction of them is used for sub-allocation
	 * to satisfy the memory allocation request.
	 */
	size_t nr_pages_to_account = 0;

	KBASE_DEBUG_ASSERT(alloc->type == KBASE_MEM_TYPE_NATIVE);
	KBASE_DEBUG_ASSERT(alloc->imported.native.kctx);

	lockdep_assert_held(&pool->pool_lock);

	kctx = alloc->imported.native.kctx;
	kbdev = kctx->kbdev;

	if (!kbdev->pagesize_2mb)
		WARN_ON(pool->order);

	if (alloc->reg) {
		if (nr_pages_requested > alloc->reg->nr_pages - alloc->nents)
			goto invalid_request;
	}

	lockdep_assert_held(&kctx->mem_partials_lock);

	if (nr_pages_requested == 0)
		goto done; /*nothing to do*/

	/* Increase mm counters before we allocate pages so that this
	 * allocation is visible to the OOM killer. The actual count
	 * of pages will be amended later, if necessary, but for the
	 * moment it is safe to account for the amount initially
	 * requested.
	 */
	new_page_count = mem_account_inc(kctx, nr_pages_requested);
	tp = alloc->pages + alloc->nents;
	new_pages = tp;

	if (kbdev->pagesize_2mb && pool->order) {
		size_t nr_lp = nr_left / NUM_PAGES_IN_2MB_LARGE_PAGE;

		res = kbase_mem_pool_alloc_pages_locked(pool, nr_lp * NUM_PAGES_IN_2MB_LARGE_PAGE,
							tp);

		if (res > 0) {
			nr_left -= (size_t)res;
			tp += res;
			nr_pages_to_account += res;
		}

		if (nr_left) {
			struct kbase_sub_alloc *sa, *temp_sa;

			list_for_each_entry_safe(sa, temp_sa, &kctx->mem_partials, link) {
				unsigned int pidx = 0;

				while (nr_left) {
					pidx = find_next_zero_bit(
						sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE, pidx);
					bitmap_set(sa->sub_pages, pidx, 1);
					*tp++ = as_tagged_tag(page_to_phys(sa->page + pidx),
							      FROM_PARTIAL);
					nr_left--;

					if (bitmap_full(sa->sub_pages,
							NUM_PAGES_IN_2MB_LARGE_PAGE)) {
						/* unlink from partial list when
						 * full
						 */
						list_del_init(&sa->link);
						break;
					}
				}
			}
		}

		/* only if we actually have a chunk left <512. If more it
		 * indicates that we couldn't allocate a 2MB above, so no point
		 * to retry here.
		 */
		if (nr_left > 0 && nr_left < NUM_PAGES_IN_2MB_LARGE_PAGE) {
			/* create a new partial and suballocate the rest from it
			 */
			struct page *np = NULL;

			np = kbase_mem_pool_alloc_locked(pool);

			if (np) {
				size_t i;
				struct kbase_sub_alloc *const sa = *prealloc_sa;
				struct page *p;

				/* store pointers back to the control struct */
				np->lru.next = (void *)sa;
				for (p = np; p < np + NUM_PAGES_IN_2MB_LARGE_PAGE; p++)
					p->lru.prev = (void *)np;
				INIT_LIST_HEAD(&sa->link);
				bitmap_zero(sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE);
				sa->page = np;

				for (i = 0; i < nr_left; i++)
					*tp++ = as_tagged_tag(page_to_phys(np + i), FROM_PARTIAL);

				bitmap_set(sa->sub_pages, 0, nr_left);
				nr_left = 0;

				/* A large page has been used for sub-allocation: account
				 * for the whole of the large page, and not just for the
				 * sub-pages that have been used.
				 */
				nr_pages_to_account += NUM_PAGES_IN_2MB_LARGE_PAGE;

				/* Indicate to user that we'll free this memory
				 * later.
				 */
				*prealloc_sa = NULL;

				/* expose for later use */
				list_add(&sa->link, &kctx->mem_partials);
			}
		}
		if (nr_left)
			goto alloc_failed;
	} else {
		res = kbase_mem_pool_alloc_pages_locked(pool, nr_left, tp);
		if (res <= 0)
			goto alloc_failed;
		nr_pages_to_account += res;
	}

	/* Amend the page count with the number of pages actually used. */
	if (nr_pages_to_account > nr_pages_requested)
		new_page_count = mem_account_inc(kctx, nr_pages_to_account - nr_pages_requested);
	else if (nr_pages_to_account < nr_pages_requested)
		new_page_count = mem_account_dec(kctx, nr_pages_requested - nr_pages_to_account);

	KBASE_TLSTREAM_AUX_PAGESALLOC(kbdev, kctx->id, (u64)new_page_count);

	alloc->nents += nr_pages_requested;

done:
	return new_pages;

alloc_failed:
	/* The first step of error recovery is freeing any allocation that
	 * might have succeeded. The function can be in this condition only
	 * in one case: it tried to allocate a combination of 2 MB and small
	 * pages but only the former step succeeded. In this case, calculate
	 * the number of 2 MB pages to release and free them.
	 */
	if (nr_left != nr_pages_requested) {
		size_t nr_pages_to_free = nr_pages_requested - nr_left;

		struct tagged_addr *start_free = alloc->pages + alloc->nents;

		if (kbdev->pagesize_2mb && pool->order) {
			while (nr_pages_to_free) {
				if (is_huge_head(*start_free)) {
					kbase_mem_pool_free_pages_locked(
						pool, NUM_PAGES_IN_2MB_LARGE_PAGE, start_free,
						false, /* not dirty */
						true); /* return to pool */
					nr_pages_to_free -= NUM_PAGES_IN_2MB_LARGE_PAGE;
					start_free += NUM_PAGES_IN_2MB_LARGE_PAGE;
				} else if (is_partial(*start_free)) {
					free_partial_locked(kctx, pool, *start_free);
					nr_pages_to_free--;
					start_free++;
				}
			}
		} else {
			kbase_mem_pool_free_pages_locked(pool, nr_pages_to_free, start_free,
							 false, /* not dirty */
							 true); /* return to pool */
		}
	}

	/* Undo the preliminary memory accounting that was done early on
	 * in the function. The code above doesn't undo memory accounting
	 * so this is the only point where the function has to undo all
	 * of the pages accounted for at the top of the function.
	 */
	mem_account_dec(kctx, nr_pages_requested);

invalid_request:
	return NULL;
}

static size_t free_partial(struct kbase_context *kctx, int group_id, struct tagged_addr tp)
{
	struct page *p, *head_page;
	struct kbase_sub_alloc *sa;
	size_t nr_pages_to_account = 0;

	p = as_page(tp);
	head_page = (struct page *)p->lru.prev;
	sa = (struct kbase_sub_alloc *)head_page->lru.next;
	spin_lock(&kctx->mem_partials_lock);
	clear_bit(p - head_page, sa->sub_pages);
	if (bitmap_empty(sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE)) {
		list_del(&sa->link);
		kbase_mem_pool_free(&kctx->mem_pools.large[group_id], head_page, true);
		kfree(sa);
		nr_pages_to_account = NUM_PAGES_IN_2MB_LARGE_PAGE;
	} else if (bitmap_weight(sa->sub_pages, NUM_PAGES_IN_2MB_LARGE_PAGE) ==
		   NUM_PAGES_IN_2MB_LARGE_PAGE - 1) {
		/* expose the partial again */
		list_add(&sa->link, &kctx->mem_partials);
	}
	spin_unlock(&kctx->mem_partials_lock);

	return nr_pages_to_account;
}

int kbase_free_phy_pages_helper(struct kbase_mem_phy_alloc *alloc, size_t nr_pages_to_free)
{
	struct kbase_context *kctx = alloc->imported.native.kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	bool syncback;
	bool reclaimed = (alloc->evicted != 0);
	struct tagged_addr *start_free;
	int new_page_count __maybe_unused;
	size_t freed = 0;
	/* The number of pages to account represents the total amount of memory
	 * actually freed. If large pages are used, they are taken into account
	 * in full, even if only a fraction of them is used for sub-allocation
	 * to satisfy the memory allocation request.
	 */
	size_t nr_pages_to_account = 0;

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_NATIVE) ||
	    WARN_ON(alloc->imported.native.kctx == NULL) ||
	    WARN_ON(alloc->nents < nr_pages_to_free) ||
	    WARN_ON(alloc->group_id >= MEMORY_GROUP_MANAGER_NR_GROUPS)) {
		return -EINVAL;
	}

	/* early out if nothing to do */
	if (nr_pages_to_free == 0)
		return 0;

	start_free = alloc->pages + alloc->nents - nr_pages_to_free;

	syncback = alloc->properties & KBASE_MEM_PHY_ALLOC_ACCESSED_CACHED;

	/* pad start_free to a valid start location */
	while (nr_pages_to_free && is_huge(*start_free) && !is_huge_head(*start_free)) {
		nr_pages_to_free--;
		start_free++;
	}

	while (nr_pages_to_free) {
		if (is_huge_head(*start_free)) {
			/* This is a 2MB entry, so free all the 512 pages that
			 * it points to
			 */
			kbase_mem_pool_free_pages(&kctx->mem_pools.large[alloc->group_id],
						  NUM_PAGES_IN_2MB_LARGE_PAGE, start_free, syncback,
						  reclaimed);
			nr_pages_to_free -= NUM_PAGES_IN_2MB_LARGE_PAGE;
			start_free += NUM_PAGES_IN_2MB_LARGE_PAGE;
			freed += NUM_PAGES_IN_2MB_LARGE_PAGE;
			nr_pages_to_account += NUM_PAGES_IN_2MB_LARGE_PAGE;
		} else if (is_partial(*start_free)) {
			nr_pages_to_account += free_partial(kctx, alloc->group_id, *start_free);
			nr_pages_to_free--;
			start_free++;
			freed++;
		} else {
			struct tagged_addr *local_end_free;

			local_end_free = start_free;
			while (nr_pages_to_free && !is_huge(*local_end_free) &&
			       !is_partial(*local_end_free)) {
				local_end_free++;
				nr_pages_to_free--;
			}
			kbase_mem_pool_free_pages(&kctx->mem_pools.small[alloc->group_id],
						  (size_t)(local_end_free - start_free), start_free,
						  syncback, reclaimed);
			freed += (size_t)(local_end_free - start_free);
			nr_pages_to_account += (size_t)(local_end_free - start_free);
			start_free += local_end_free - start_free;
		}
	}

	alloc->nents -= freed;

	if (!reclaimed) {
		/* If the allocation was not reclaimed then all freed pages
		 * need to be accounted.
		 */
		new_page_count = mem_account_dec(kctx, nr_pages_to_account);
		KBASE_TLSTREAM_AUX_PAGESALLOC(kbdev, kctx->id, (u64)new_page_count);
	} else if (freed != nr_pages_to_account) {
		/* If the allocation was reclaimed then alloc->nents pages
		 * have already been accounted for.
		 *
		 * Only update the number of pages to account if there is
		 * a discrepancy to correct, due to the fact that large pages
		 * were partially allocated at the origin.
		 */
		if (freed > nr_pages_to_account)
			new_page_count = mem_account_inc(kctx, freed - nr_pages_to_account);
		else
			new_page_count = mem_account_dec(kctx, nr_pages_to_account - freed);
		KBASE_TLSTREAM_AUX_PAGESALLOC(kbdev, kctx->id, (u64)new_page_count);
	}

	return 0;
}

void kbase_free_phy_pages_helper_locked(struct kbase_mem_phy_alloc *alloc,
					struct kbase_mem_pool *pool, struct tagged_addr *pages,
					size_t nr_pages_to_free)
{
	struct kbase_context *kctx = alloc->imported.native.kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	bool syncback;
	struct tagged_addr *start_free;
	size_t freed = 0;
	/* The number of pages to account represents the total amount of memory
	 * actually freed. If large pages are used, they are taken into account
	 * in full, even if only a fraction of them is used for sub-allocation
	 * to satisfy the memory allocation request.
	 */
	size_t nr_pages_to_account = 0;
	int new_page_count;

	KBASE_DEBUG_ASSERT(alloc->type == KBASE_MEM_TYPE_NATIVE);
	KBASE_DEBUG_ASSERT(alloc->imported.native.kctx);
	KBASE_DEBUG_ASSERT(alloc->nents >= nr_pages_to_free);

	lockdep_assert_held(&pool->pool_lock);
	lockdep_assert_held(&kctx->mem_partials_lock);

	/* early out if state is inconsistent. */
	if (alloc->evicted) {
		dev_err(kbdev->dev, "%s unexpectedly called for evicted region", __func__);
		return;
	}

	/* early out if nothing to do */
	if (!nr_pages_to_free)
		return;

	start_free = pages;

	syncback = alloc->properties & KBASE_MEM_PHY_ALLOC_ACCESSED_CACHED;

	/* pad start_free to a valid start location */
	while (nr_pages_to_free && is_huge(*start_free) && !is_huge_head(*start_free)) {
		nr_pages_to_free--;
		start_free++;
	}

	while (nr_pages_to_free) {
		if (is_huge_head(*start_free)) {
			/* This is a 2MB entry, so free all the 512 pages that
			 * it points to
			 */
			WARN_ON(!pool->order);
			kbase_mem_pool_free_pages_locked(pool, NUM_PAGES_IN_2MB_LARGE_PAGE,
							 start_free, syncback, false);
			nr_pages_to_free -= NUM_PAGES_IN_2MB_LARGE_PAGE;
			start_free += NUM_PAGES_IN_2MB_LARGE_PAGE;
			freed += NUM_PAGES_IN_2MB_LARGE_PAGE;
			nr_pages_to_account += NUM_PAGES_IN_2MB_LARGE_PAGE;
		} else if (is_partial(*start_free)) {
			WARN_ON(!pool->order);
			nr_pages_to_account += free_partial_locked(kctx, pool, *start_free);
			nr_pages_to_free--;
			start_free++;
			freed++;
		} else {
			struct tagged_addr *local_end_free;

			WARN_ON(pool->order);
			local_end_free = start_free;
			while (nr_pages_to_free && !is_huge(*local_end_free) &&
			       !is_partial(*local_end_free)) {
				local_end_free++;
				nr_pages_to_free--;
			}
			kbase_mem_pool_free_pages_locked(pool,
							 (size_t)(local_end_free - start_free),
							 start_free, syncback, false);
			freed += (size_t)(local_end_free - start_free);
			nr_pages_to_account += (size_t)(local_end_free - start_free);
			start_free += local_end_free - start_free;
		}
	}

	alloc->nents -= freed;

	new_page_count = mem_account_dec(kctx, nr_pages_to_account);
	KBASE_TLSTREAM_AUX_PAGESALLOC(kbdev, kctx->id, (u64)new_page_count);
}
KBASE_EXPORT_TEST_API(kbase_free_phy_pages_helper_locked);

void kbase_mem_kref_free(struct kref *kref)
{
	struct kbase_mem_phy_alloc *alloc;

	alloc = container_of(kref, struct kbase_mem_phy_alloc, kref);

	switch (alloc->type) {
	case KBASE_MEM_TYPE_NATIVE: {
		if (!WARN_ON(!alloc->imported.native.kctx)) {
			if (alloc->permanent_map)
				kbase_phy_alloc_mapping_term(alloc->imported.native.kctx, alloc);

			/*
			 * The physical allocation must have been removed from
			 * the eviction list before trying to free it.
			 */
			mutex_lock(&alloc->imported.native.kctx->jit_evict_lock);
			WARN_ON(!list_empty(&alloc->evict_node));
			mutex_unlock(&alloc->imported.native.kctx->jit_evict_lock);

			kbase_process_page_usage_dec(alloc->imported.native.kctx,
						     alloc->imported.native.nr_struct_pages);
		}
		kbase_free_phy_pages_helper(alloc, alloc->nents);
		break;
	}
	case KBASE_MEM_TYPE_ALIAS: {
		/* just call put on the underlying phy allocs */
		size_t i;
		struct kbase_aliased *aliased;

		aliased = alloc->imported.alias.aliased;
		if (aliased) {
			for (i = 0; i < alloc->imported.alias.nents; i++)
				if (aliased[i].alloc) {
					kbase_mem_phy_alloc_gpu_unmapped(aliased[i].alloc);
					kbase_mem_phy_alloc_put(aliased[i].alloc);
				}
			vfree(aliased);
		}
		break;
	}
	case KBASE_MEM_TYPE_RAW:
		/* raw pages, external cleanup */
		break;
	case KBASE_MEM_TYPE_IMPORTED_UMM:
		if (!IS_ENABLED(CONFIG_MALI_DMA_BUF_MAP_ON_DEMAND)) {
			WARN_ONCE(alloc->imported.umm.current_mapping_usage_count != 1,
				  "WARNING: expected exactly 1 mapping, got %d",
				  alloc->imported.umm.current_mapping_usage_count);
#if (KERNEL_VERSION(6, 1, 55) <= LINUX_VERSION_CODE)
			dma_buf_unmap_attachment_unlocked(alloc->imported.umm.dma_attachment,
							  alloc->imported.umm.sgt,
							  DMA_BIDIRECTIONAL);
#else
			dma_buf_unmap_attachment(alloc->imported.umm.dma_attachment,
						 alloc->imported.umm.sgt, DMA_BIDIRECTIONAL);
#endif
			kbase_remove_dma_buf_usage(alloc->imported.umm.kctx, alloc);
		}
		dma_buf_detach(alloc->imported.umm.dma_buf, alloc->imported.umm.dma_attachment);
		dma_buf_put(alloc->imported.umm.dma_buf);
		break;
	case KBASE_MEM_TYPE_IMPORTED_USER_BUF:
		switch (alloc->imported.user_buf.state) {
		case KBASE_USER_BUF_STATE_PINNED:
		case KBASE_USER_BUF_STATE_DMA_MAPPED:
		case KBASE_USER_BUF_STATE_GPU_MAPPED: {
			/* It's too late to undo all of the operations that might have been
			 * done on an imported USER_BUFFER handle, as references have been
			 * lost already.
			 *
			 * The only thing that can be done safely and that is crucial for
			 * the rest of the system is releasing the physical pages that have
			 * been pinned and that are still referenced by the physical
			 * allocationl.
			 */
			kbase_user_buf_unpin_pages(alloc);
			alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_EMPTY;
			break;
		}
		case KBASE_USER_BUF_STATE_EMPTY: {
			/* Nothing to do. */
			break;
		}
		default: {
			WARN(1, "Unexpected free of type %d state %d\n", alloc->type,
			     alloc->imported.user_buf.state);
			break;
		}
		}

		if (alloc->imported.user_buf.mm)
			mmdrop(alloc->imported.user_buf.mm);
		if (alloc->properties & KBASE_MEM_PHY_ALLOC_LARGE)
			vfree(alloc->imported.user_buf.pages);
		else
			kfree(alloc->imported.user_buf.pages);
		break;
	default:
		WARN(1, "Unexpected free of type %d\n", alloc->type);
		break;
	}

	/* Free based on allocation type */
	if (alloc->properties & KBASE_MEM_PHY_ALLOC_LARGE)
		vfree(alloc);
	else
		kfree(alloc);
}

KBASE_EXPORT_TEST_API(kbase_mem_kref_free);

int kbase_alloc_phy_pages(struct kbase_va_region *reg, size_t vsize, size_t size)
{
	KBASE_DEBUG_ASSERT(reg != NULL);
	KBASE_DEBUG_ASSERT(vsize > 0);

	/* validate user provided arguments */
	if (size > vsize || vsize > reg->nr_pages)
		goto out_term;

	/* Prevent vsize*sizeof from wrapping around.
	 * For instance, if vsize is 2**29+1, we'll allocate 1 byte and the alloc won't fail.
	 */
	if ((size_t)vsize > ((size_t)-1 / sizeof(*reg->cpu_alloc->pages)))
		goto out_term;

	KBASE_DEBUG_ASSERT(vsize != 0);

	if (kbase_alloc_phy_pages_helper(reg->cpu_alloc, size) != 0)
		goto out_term;

	reg->cpu_alloc->reg = reg;
	if (reg->cpu_alloc != reg->gpu_alloc) {
		if (kbase_alloc_phy_pages_helper(reg->gpu_alloc, size) != 0)
			goto out_rollback;
		reg->gpu_alloc->reg = reg;
	}

	return 0;

out_rollback:
	kbase_free_phy_pages_helper(reg->cpu_alloc, size);
out_term:
	return -1;
}
KBASE_EXPORT_TEST_API(kbase_alloc_phy_pages);

void kbase_set_phy_alloc_page_status(struct kbase_mem_phy_alloc *alloc,
				     enum kbase_page_status status)
{
	u32 i = 0;

	for (; i < alloc->nents; i++) {
		struct tagged_addr phys = alloc->pages[i];
		struct kbase_page_metadata *page_md = kbase_page_private(as_page(phys));

		/* Skip the small page that is part of a large page, as the large page is
		 * excluded from the migration process.
		 */
		if (is_huge(phys) || is_partial(phys))
			continue;

		if (!page_md)
			continue;

		spin_lock(&page_md->migrate_lock);
		page_md->status = PAGE_STATUS_SET(page_md->status, (u8)status);
		spin_unlock(&page_md->migrate_lock);
	}
}

bool kbase_check_alloc_flags(unsigned long flags)
{
	/* Only known input flags should be set. */
	if (flags & ~BASE_MEM_FLAGS_INPUT_MASK)
		return false;

	/* At least one flag should be set */
	if (flags == 0)
		return false;

	/* Either the GPU or CPU must be reading from the allocated memory */
	if ((flags & (BASE_MEM_PROT_CPU_RD | BASE_MEM_PROT_GPU_RD)) == 0)
		return false;

	/* Either the GPU or CPU must be writing to the allocated memory */
	if ((flags & (BASE_MEM_PROT_CPU_WR | BASE_MEM_PROT_GPU_WR)) == 0)
		return false;

	/* GPU executable memory cannot:
	 * - Be written by the GPU
	 * - Be grown on GPU page fault
	 */
	if ((flags & BASE_MEM_PROT_GPU_EX) &&
	    (flags & (BASE_MEM_PROT_GPU_WR | BASE_MEM_GROW_ON_GPF)))
		return false;

#if !MALI_USE_CSF
	/* GPU executable memory also cannot have the top of its initial
	 * commit aligned to 'extension'
	 */
	if ((flags & BASE_MEM_PROT_GPU_EX) && (flags & BASE_MEM_TILER_ALIGN_TOP))
		return false;
#endif /* !MALI_USE_CSF */

	/* To have an allocation lie within a 4GB chunk is required only for
	 * TLS memory, which will never be used to contain executable code.
	 */
	if ((flags & BASE_MEM_GPU_VA_SAME_4GB_PAGE) && (flags & BASE_MEM_PROT_GPU_EX))
		return false;

#if !MALI_USE_CSF
	/* TLS memory should also not be used for tiler heap */
	if ((flags & BASE_MEM_GPU_VA_SAME_4GB_PAGE) && (flags & BASE_MEM_TILER_ALIGN_TOP))
		return false;
#endif /* !MALI_USE_CSF */

	/* GPU should have at least read or write access otherwise there is no
	 * reason for allocating.
	 */
	if ((flags & (BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR)) == 0)
		return false;

	/* BASE_MEM_IMPORT_SHARED is only valid for imported memory */
	if ((flags & BASE_MEM_IMPORT_SHARED) == BASE_MEM_IMPORT_SHARED)
		return false;

	/* BASE_MEM_IMPORT_SYNC_ON_MAP_UNMAP is only valid for imported memory
	 */
	if ((flags & BASE_MEM_IMPORT_SYNC_ON_MAP_UNMAP) == BASE_MEM_IMPORT_SYNC_ON_MAP_UNMAP)
		return false;

	/* Should not combine BASE_MEM_COHERENT_LOCAL with
	 * BASE_MEM_COHERENT_SYSTEM
	 */
	if ((flags & (BASE_MEM_COHERENT_LOCAL | BASE_MEM_COHERENT_SYSTEM)) ==
	    (BASE_MEM_COHERENT_LOCAL | BASE_MEM_COHERENT_SYSTEM))
		return false;

#if MALI_USE_CSF
	if ((flags & BASE_MEM_SAME_VA) && (flags & (BASE_MEM_FIXABLE | BASE_MEM_FIXED)))
		return false;

	if ((flags & BASE_MEM_FIXABLE) && (flags & BASE_MEM_FIXED))
		return false;
#endif

	return true;
}

bool kbase_check_import_flags(unsigned long flags)
{
	/* Only known input flags should be set. */
	if (flags & ~BASE_MEM_FLAGS_INPUT_MASK)
		return false;

	/* At least one flag should be set */
	if (flags == 0)
		return false;

	/* Imported memory cannot be GPU executable */
	if (flags & BASE_MEM_PROT_GPU_EX)
		return false;

	/* Imported memory cannot grow on page fault */
	if (flags & BASE_MEM_GROW_ON_GPF)
		return false;

#if MALI_USE_CSF
	/* Imported memory cannot be fixed */
	if ((flags & (BASE_MEM_FIXED | BASE_MEM_FIXABLE)))
		return false;
#else
	/* Imported memory cannot be aligned to the end of its initial commit */
	if (flags & BASE_MEM_TILER_ALIGN_TOP)
		return false;
#endif /* !MALI_USE_CSF */

	/* GPU should have at least read or write access otherwise there is no
	 * reason for importing.
	 */
	if ((flags & (BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR)) == 0)
		return false;

	/* Protected memory cannot be read by the CPU */
	if ((flags & BASE_MEM_PROTECTED) && (flags & BASE_MEM_PROT_CPU_RD))
		return false;

	return true;
}

int kbase_check_alloc_sizes(struct kbase_context *kctx, unsigned long flags, u64 va_pages,
			    u64 commit_pages, u64 large_extension)
{
	struct device *dev = kctx->kbdev->dev;
	u32 gpu_pc_bits = kctx->kbdev->gpu_props.log2_program_counter_size;
	u64 gpu_pc_pages_max = 1ULL << gpu_pc_bits >> PAGE_SHIFT;
	struct kbase_va_region test_reg;

	/* kbase_va_region's extension member can be of variable size, so check against that type */
	test_reg.extension = large_extension;

#define KBASE_MSG_PRE "GPU allocation attempted with "

	if (va_pages == 0) {
		dev_warn(dev, KBASE_MSG_PRE "0 va_pages!");
		return -EINVAL;
	}

	if (va_pages > KBASE_MEM_ALLOC_MAX_SIZE) {
		dev_warn(dev, KBASE_MSG_PRE "va_pages==%lld larger than KBASE_MEM_ALLOC_MAX_SIZE!",
			 (unsigned long long)va_pages);
		return -ENOMEM;
	}

	/* Note: commit_pages is checked against va_pages during
	 * kbase_alloc_phy_pages()
	 */

	/* Limit GPU executable allocs to GPU PC size */
	if ((flags & BASE_MEM_PROT_GPU_EX) && (va_pages > gpu_pc_pages_max)) {
		dev_warn(dev,
			 KBASE_MSG_PRE
			 "BASE_MEM_PROT_GPU_EX and va_pages==%lld larger than GPU PC range %lld",
			 (unsigned long long)va_pages, (unsigned long long)gpu_pc_pages_max);

		return -EINVAL;
	}

	if ((flags & BASE_MEM_GROW_ON_GPF) && (test_reg.extension == 0)) {
		dev_warn(dev, KBASE_MSG_PRE "BASE_MEM_GROW_ON_GPF but extension == 0\n");
		return -EINVAL;
	}

#if !MALI_USE_CSF
	if ((flags & BASE_MEM_TILER_ALIGN_TOP) && (test_reg.extension == 0)) {
		dev_warn(dev, KBASE_MSG_PRE "BASE_MEM_TILER_ALIGN_TOP but extension == 0\n");
		return -EINVAL;
	}

	if (!(flags & (BASE_MEM_GROW_ON_GPF | BASE_MEM_TILER_ALIGN_TOP)) &&
	    test_reg.extension != 0) {
		dev_warn(
			dev, KBASE_MSG_PRE
			"neither BASE_MEM_GROW_ON_GPF nor BASE_MEM_TILER_ALIGN_TOP set but extension != 0\n");
		return -EINVAL;
	}
#else
	if (!(flags & BASE_MEM_GROW_ON_GPF) && test_reg.extension != 0) {
		dev_warn(dev, KBASE_MSG_PRE "BASE_MEM_GROW_ON_GPF not set but extension != 0\n");
		return -EINVAL;
	}
#endif /* !MALI_USE_CSF */

#if !MALI_USE_CSF
	/* BASE_MEM_TILER_ALIGN_TOP memory has a number of restrictions */
	if (flags & BASE_MEM_TILER_ALIGN_TOP) {
#define KBASE_MSG_PRE_FLAG KBASE_MSG_PRE "BASE_MEM_TILER_ALIGN_TOP and "
		unsigned long small_extension;

		if (large_extension > BASE_MEM_TILER_ALIGN_TOP_EXTENSION_MAX_PAGES) {
			dev_warn(dev, KBASE_MSG_PRE_FLAG "extension==%lld pages exceeds limit %lld",
				 (unsigned long long)large_extension,
				 BASE_MEM_TILER_ALIGN_TOP_EXTENSION_MAX_PAGES);
			return -EINVAL;
		}
		/* For use with is_power_of_2, which takes unsigned long, so
		 * must ensure e.g. on 32-bit kernel it'll fit in that type
		 */
		small_extension = (unsigned long)large_extension;

		if (!is_power_of_2(small_extension)) {
			dev_warn(dev, KBASE_MSG_PRE_FLAG "extension==%ld not a non-zero power of 2",
				 small_extension);
			return -EINVAL;
		}

		if (commit_pages > large_extension) {
			dev_warn(dev, KBASE_MSG_PRE_FLAG "commit_pages==%ld exceeds extension==%ld",
				 (unsigned long)commit_pages, (unsigned long)large_extension);
			return -EINVAL;
		}
#undef KBASE_MSG_PRE_FLAG
	}
#else
	CSTD_UNUSED(commit_pages);
#endif /* !MALI_USE_CSF */

	if ((flags & BASE_MEM_GPU_VA_SAME_4GB_PAGE) && (va_pages > (BASE_MEM_PFN_MASK_4GB + 1))) {
		dev_warn(
			dev,
			KBASE_MSG_PRE
			"BASE_MEM_GPU_VA_SAME_4GB_PAGE and va_pages==%lld greater than that needed for 4GB space",
			(unsigned long long)va_pages);
		return -EINVAL;
	}

	return 0;
#undef KBASE_MSG_PRE
}

void kbase_gpu_vm_lock(struct kbase_context *kctx)
{
	KBASE_DEBUG_ASSERT(kctx != NULL);
	mutex_lock(&kctx->reg_lock);
}

KBASE_EXPORT_TEST_API(kbase_gpu_vm_lock);

void kbase_gpu_vm_unlock(struct kbase_context *kctx)
{
	KBASE_DEBUG_ASSERT(kctx != NULL);
	mutex_unlock(&kctx->reg_lock);
}

KBASE_EXPORT_TEST_API(kbase_gpu_vm_unlock);

#if IS_ENABLED(CONFIG_DEBUG_FS)
struct kbase_jit_debugfs_data {
	int (*func)(struct kbase_jit_debugfs_data *data);
	struct mutex lock;
	struct kbase_context *kctx;
	u64 active_value;
	u64 pool_value;
	u64 destroy_value;
	char buffer[50];
};

static int kbase_jit_debugfs_common_open(struct inode *inode, struct file *file,
					 int (*func)(struct kbase_jit_debugfs_data *))
{
	struct kbase_jit_debugfs_data *data;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->func = func;
	mutex_init(&data->lock);
	data->kctx = (struct kbase_context *)inode->i_private;

	file->private_data = data;

	return nonseekable_open(inode, file);
}

static ssize_t kbase_jit_debugfs_common_read(struct file *file, char __user *buf, size_t len,
					     loff_t *ppos)
{
	struct kbase_jit_debugfs_data *data;
	size_t size;
	int ret;

	data = (struct kbase_jit_debugfs_data *)file->private_data;
	mutex_lock(&data->lock);

	if (*ppos) {
		size = strnlen(data->buffer, sizeof(data->buffer));
	} else {
		if (!data->func) {
			ret = -EACCES;
			goto out_unlock;
		}

		if (data->func(data)) {
			ret = -EACCES;
			goto out_unlock;
		}

		size = (size_t)scnprintf(data->buffer, sizeof(data->buffer), "%llu,%llu,%llu\n",
					 data->active_value, data->pool_value, data->destroy_value);
	}

	ret = simple_read_from_buffer(buf, len, ppos, data->buffer, size);

out_unlock:
	mutex_unlock(&data->lock);
	return ret;
}

static int kbase_jit_debugfs_common_release(struct inode *inode, struct file *file)
{
	CSTD_UNUSED(inode);

	kfree(file->private_data);
	return 0;
}

#define KBASE_JIT_DEBUGFS_DECLARE(__fops, __func)                          \
	static int __fops##_open(struct inode *inode, struct file *file)   \
	{                                                                  \
		return kbase_jit_debugfs_common_open(inode, file, __func); \
	}                                                                  \
	static const struct file_operations __fops = {                     \
		.owner = THIS_MODULE,                                      \
		.open = __fops##_open,                                     \
		.release = kbase_jit_debugfs_common_release,               \
		.read = kbase_jit_debugfs_common_read,                     \
		.write = NULL,                                             \
		.llseek = generic_file_llseek,                             \
	}

static int kbase_jit_debugfs_count_get(struct kbase_jit_debugfs_data *data)
{
	struct kbase_context *kctx = data->kctx;
	struct list_head *tmp;

	mutex_lock(&kctx->jit_evict_lock);
	list_for_each(tmp, &kctx->jit_active_head) {
		data->active_value++;
	}

	list_for_each(tmp, &kctx->jit_pool_head) {
		data->pool_value++;
	}

	list_for_each(tmp, &kctx->jit_destroy_head) {
		data->destroy_value++;
	}
	mutex_unlock(&kctx->jit_evict_lock);

	return 0;
}
KBASE_JIT_DEBUGFS_DECLARE(kbase_jit_debugfs_count_fops, kbase_jit_debugfs_count_get);

static int kbase_jit_debugfs_vm_get(struct kbase_jit_debugfs_data *data)
{
	struct kbase_context *kctx = data->kctx;
	struct kbase_va_region *reg;

	mutex_lock(&kctx->jit_evict_lock);
	list_for_each_entry(reg, &kctx->jit_active_head, jit_node) {
		data->active_value += reg->nr_pages;
	}

	list_for_each_entry(reg, &kctx->jit_pool_head, jit_node) {
		data->pool_value += reg->nr_pages;
	}

	list_for_each_entry(reg, &kctx->jit_destroy_head, jit_node) {
		data->destroy_value += reg->nr_pages;
	}
	mutex_unlock(&kctx->jit_evict_lock);

	return 0;
}
KBASE_JIT_DEBUGFS_DECLARE(kbase_jit_debugfs_vm_fops, kbase_jit_debugfs_vm_get);

static int kbase_jit_debugfs_phys_get(struct kbase_jit_debugfs_data *data)
{
	struct kbase_context *kctx = data->kctx;
	struct kbase_va_region *reg;

	mutex_lock(&kctx->jit_evict_lock);
	list_for_each_entry(reg, &kctx->jit_active_head, jit_node) {
		data->active_value += reg->gpu_alloc->nents;
	}

	list_for_each_entry(reg, &kctx->jit_pool_head, jit_node) {
		data->pool_value += reg->gpu_alloc->nents;
	}

	list_for_each_entry(reg, &kctx->jit_destroy_head, jit_node) {
		data->destroy_value += reg->gpu_alloc->nents;
	}
	mutex_unlock(&kctx->jit_evict_lock);

	return 0;
}
KBASE_JIT_DEBUGFS_DECLARE(kbase_jit_debugfs_phys_fops, kbase_jit_debugfs_phys_get);

#if MALI_JIT_PRESSURE_LIMIT_BASE
static int kbase_jit_debugfs_used_get(struct kbase_jit_debugfs_data *data)
{
	struct kbase_context *kctx = data->kctx;
	struct kbase_va_region *reg;

#if !MALI_USE_CSF
	mutex_lock(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */
	mutex_lock(&kctx->jit_evict_lock);
	list_for_each_entry(reg, &kctx->jit_active_head, jit_node) {
		data->active_value += reg->used_pages;
	}
	mutex_unlock(&kctx->jit_evict_lock);
#if !MALI_USE_CSF
	mutex_unlock(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */

	return 0;
}

KBASE_JIT_DEBUGFS_DECLARE(kbase_jit_debugfs_used_fops, kbase_jit_debugfs_used_get);

static int kbase_mem_jit_trim_pages_from_region(struct kbase_context *kctx,
						struct kbase_va_region *reg, size_t pages_needed,
						size_t *freed, bool shrink);

static int kbase_jit_debugfs_trim_get(struct kbase_jit_debugfs_data *data)
{
	struct kbase_context *kctx = data->kctx;
	struct kbase_va_region *reg;

#if !MALI_USE_CSF
	mutex_lock(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */
	kbase_gpu_vm_lock(kctx);
	mutex_lock(&kctx->jit_evict_lock);
	list_for_each_entry(reg, &kctx->jit_active_head, jit_node) {
		int err;
		size_t freed = 0u;

		err = kbase_mem_jit_trim_pages_from_region(kctx, reg, SIZE_MAX, &freed, false);

		if (err) {
			/* Failed to calculate, try the next region */
			continue;
		}

		data->active_value += freed;
	}
	mutex_unlock(&kctx->jit_evict_lock);
	kbase_gpu_vm_unlock(kctx);
#if !MALI_USE_CSF
	mutex_unlock(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */

	return 0;
}

KBASE_JIT_DEBUGFS_DECLARE(kbase_jit_debugfs_trim_fops, kbase_jit_debugfs_trim_get);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

void kbase_jit_debugfs_init(struct kbase_context *kctx)
{
	/* prevent unprivileged use of debug file system
	 * in old kernel version
	 */
	const mode_t mode = 0444;

	/* Caller already ensures this, but we keep the pattern for
	 * maintenance safety.
	 */
	if (WARN_ON(!kctx) || WARN_ON(IS_ERR_OR_NULL(kctx->kctx_dentry)))
		return;

	/* Debugfs entry for getting the number of JIT allocations. */
	debugfs_create_file("mem_jit_count", mode, kctx->kctx_dentry, kctx,
			    &kbase_jit_debugfs_count_fops);

	/*
	 * Debugfs entry for getting the total number of virtual pages
	 * used by JIT allocations.
	 */
	debugfs_create_file("mem_jit_vm", mode, kctx->kctx_dentry, kctx,
			    &kbase_jit_debugfs_vm_fops);

	/*
	 * Debugfs entry for getting the number of physical pages used
	 * by JIT allocations.
	 */
	debugfs_create_file("mem_jit_phys", mode, kctx->kctx_dentry, kctx,
			    &kbase_jit_debugfs_phys_fops);
#if MALI_JIT_PRESSURE_LIMIT_BASE
	/*
	 * Debugfs entry for getting the number of pages used
	 * by JIT allocations for estimating the physical pressure
	 * limit.
	 */
	debugfs_create_file("mem_jit_used", mode, kctx->kctx_dentry, kctx,
			    &kbase_jit_debugfs_used_fops);

	/*
	 * Debugfs entry for getting the number of pages that could
	 * be trimmed to free space for more JIT allocations.
	 */
	debugfs_create_file("mem_jit_trim", mode, kctx->kctx_dentry, kctx,
			    &kbase_jit_debugfs_trim_fops);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */
}
#endif /* CONFIG_DEBUG_FS */

/**
 * kbase_jit_destroy_worker - Deferred worker which frees JIT allocations
 * @work: Work item
 *
 * This function does the work of freeing JIT allocations whose physical
 * backing has been released.
 */
static void kbase_jit_destroy_worker(struct work_struct *work)
{
	struct kbase_context *kctx;
	struct kbase_va_region *reg;

	kctx = container_of(work, struct kbase_context, jit_work);
	do {
		mutex_lock(&kctx->jit_evict_lock);
		if (list_empty(&kctx->jit_destroy_head)) {
			mutex_unlock(&kctx->jit_evict_lock);
			break;
		}

		reg = list_first_entry(&kctx->jit_destroy_head, struct kbase_va_region, jit_node);

		list_del(&reg->jit_node);
		mutex_unlock(&kctx->jit_evict_lock);

		kbase_gpu_vm_lock(kctx);

		/*
		 * Incrementing the refcount is prevented on JIT regions.
		 * If/when this ever changes we would need to compensate
		 * by implementing "free on putting the last reference",
		 * but only for JIT regions.
		 */
		WARN_ON(atomic64_read(&reg->no_user_free_count) > 1);
		kbase_va_region_no_user_free_dec(reg);
		kbase_mem_free_region(kctx, reg);
		kbase_gpu_vm_unlock(kctx);
	} while (1);
}

int kbase_jit_init(struct kbase_context *kctx)
{
	mutex_lock(&kctx->jit_evict_lock);
	INIT_LIST_HEAD(&kctx->jit_active_head);
	INIT_LIST_HEAD(&kctx->jit_pool_head);
	INIT_LIST_HEAD(&kctx->jit_destroy_head);
	INIT_WORK(&kctx->jit_work, kbase_jit_destroy_worker);

#if MALI_USE_CSF
	mutex_init(&kctx->csf.kcpu_queues.jit_lock);
	INIT_LIST_HEAD(&kctx->csf.kcpu_queues.jit_cmds_head);
	INIT_LIST_HEAD(&kctx->csf.kcpu_queues.jit_blocked_queues);
#else /* !MALI_USE_CSF */
	INIT_LIST_HEAD(&kctx->jctx.jit_atoms_head);
	INIT_LIST_HEAD(&kctx->jctx.jit_pending_alloc);
#endif /* MALI_USE_CSF */
	mutex_unlock(&kctx->jit_evict_lock);

	return 0;
}

/* Check if the allocation from JIT pool is of the same size as the new JIT
 * allocation and also, if BASE_JIT_ALLOC_MEM_TILER_ALIGN_TOP is set, meets
 * the alignment requirements.
 */
static bool meet_size_and_tiler_align_top_requirements(const struct kbase_va_region *walker,
						       const struct base_jit_alloc_info *info)
{
	bool meet_reqs = true;

	if (walker->nr_pages != info->va_pages)
		meet_reqs = false;

#if !MALI_USE_CSF
	if (meet_reqs && (info->flags & BASE_JIT_ALLOC_MEM_TILER_ALIGN_TOP)) {
		size_t align = info->extension;
		size_t align_mask = align - 1;

		if ((walker->start_pfn + info->commit_pages) & align_mask)
			meet_reqs = false;
	}
#endif /* !MALI_USE_CSF */

	return meet_reqs;
}

#if MALI_JIT_PRESSURE_LIMIT_BASE
/* Function will guarantee *@freed will not exceed @pages_needed
 */
static int kbase_mem_jit_trim_pages_from_region(struct kbase_context *kctx,
						struct kbase_va_region *reg, size_t pages_needed,
						size_t *freed, bool shrink)
{
	int err = 0;
	size_t available_pages = 0u;
	const size_t old_pages = kbase_reg_current_backed_size(reg);
	size_t new_pages = old_pages;
	size_t to_free = 0u;
	size_t max_allowed_pages = old_pages;

#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */
	lockdep_assert_held(&kctx->reg_lock);

	/* Is this a JIT allocation that has been reported on? */
	if (reg->used_pages == reg->nr_pages)
		goto out;

	if (!(reg->flags & KBASE_REG_HEAP_INFO_IS_SIZE)) {
		/* For address based memory usage calculation, the GPU
		 * allocates objects of up to size 's', but aligns every object
		 * to alignment 'a', with a < s.
		 *
		 * It also doesn't have to write to all bytes in an object of
		 * size 's'.
		 *
		 * Hence, we can observe the GPU's address for the end of used
		 * memory being up to (s - a) bytes into the first unallocated
		 * page.
		 *
		 * We allow for this and only warn when it exceeds this bound
		 * (rounded up to page sized units). Note, this is allowed to
		 * exceed reg->nr_pages.
		 */
		max_allowed_pages += PFN_UP(KBASE_GPU_ALLOCATED_OBJECT_MAX_BYTES -
					    KBASE_GPU_ALLOCATED_OBJECT_ALIGN_BYTES);
	} else if (reg->flags & KBASE_REG_TILER_ALIGN_TOP) {
		/* The GPU could report being ready to write to the next
		 * 'extension' sized chunk, but didn't actually write to it, so we
		 * can report up to 'extension' size pages more than the backed
		 * size.
		 *
		 * Note, this is allowed to exceed reg->nr_pages.
		 */
		max_allowed_pages += reg->extension;

		/* Also note that in these GPUs, the GPU may make a large (>1
		 * page) initial allocation but not actually write out to all
		 * of it. Hence it might report that a much higher amount of
		 * memory was used than actually was written to. This does not
		 * result in a real warning because on growing this memory we
		 * round up the size of the allocation up to an 'extension' sized
		 * chunk, hence automatically bringing the backed size up to
		 * the reported size.
		 */
	}

	if (old_pages < reg->used_pages) {
		/* Prevent overflow on available_pages, but only report the
		 * problem if it's in a scenario where used_pages should have
		 * been consistent with the backed size
		 *
		 * Note: In case of a size-based report, this legitimately
		 * happens in common use-cases: we allow for up to this size of
		 * memory being used, but depending on the content it doesn't
		 * have to use all of it.
		 *
		 * Hence, we're much more quiet about that in the size-based
		 * report case - it's not indicating a real problem, it's just
		 * for information
		 */
		if (max_allowed_pages < reg->used_pages) {
			if (!(reg->flags & KBASE_REG_HEAP_INFO_IS_SIZE))
				dev_warn(
					kctx->kbdev->dev,
					"%s: current backed pages %zu < reported used pages %zu (allowed to be up to %zu) on JIT 0x%llx vapages %zu\n",
					__func__, old_pages, reg->used_pages, max_allowed_pages,
					reg->start_pfn << PAGE_SHIFT, reg->nr_pages);
			else
				dev_dbg(kctx->kbdev->dev,
					"%s: no need to trim, current backed pages %zu < reported used pages %zu on size-report for JIT 0x%llx vapages %zu\n",
					__func__, old_pages, reg->used_pages,
					reg->start_pfn << PAGE_SHIFT, reg->nr_pages);
		}
		/* In any case, no error condition to report here, caller can
		 * try other regions
		 */

		goto out;
	}
	available_pages = old_pages - reg->used_pages;
	to_free = min(available_pages, pages_needed);

	if (shrink) {
		new_pages -= to_free;

		err = kbase_mem_shrink(kctx, reg, new_pages);
	}
out:
	trace_mali_jit_trim_from_region(reg, to_free, old_pages, available_pages, new_pages);
	*freed = to_free;
	return err;
}

/**
 * kbase_mem_jit_trim_pages - Trim JIT regions until sufficient pages have been
 * freed
 * @kctx: Pointer to the kbase context whose active JIT allocations will be
 * checked.
 * @pages_needed: The maximum number of pages to trim.
 *
 * This functions checks all active JIT allocations in @kctx for unused pages
 * at the end, and trim the backed memory regions of those allocations down to
 * the used portion and free the unused pages into the page pool.
 *
 * Specifying @pages_needed allows us to stop early when there's enough
 * physical memory freed to sufficiently bring down the total JIT physical page
 * usage (e.g. to below the pressure limit)
 *
 * Return: Total number of successfully freed pages
 */
static size_t kbase_mem_jit_trim_pages(struct kbase_context *kctx, size_t pages_needed)
{
	struct kbase_va_region *reg, *tmp;
	size_t total_freed = 0;

#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */
	lockdep_assert_held(&kctx->reg_lock);
	lockdep_assert_held(&kctx->jit_evict_lock);

	list_for_each_entry_safe(reg, tmp, &kctx->jit_active_head, jit_node) {
		int err;
		size_t freed = 0u;

		err = kbase_mem_jit_trim_pages_from_region(kctx, reg, pages_needed, &freed, true);

		if (err) {
			/* Failed to trim, try the next region */
			continue;
		}

		total_freed += freed;
		WARN_ON(freed > pages_needed);
		pages_needed -= freed;
		if (!pages_needed)
			break;
	}

	trace_mali_jit_trim(total_freed);

	return total_freed;
}
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

static int kbase_jit_grow(struct kbase_context *kctx, const struct base_jit_alloc_info *info,
			  struct kbase_va_region *reg, struct kbase_sub_alloc **prealloc_sas,
			  enum kbase_caller_mmu_sync_info mmu_sync_info)
{
	size_t delta;
	size_t pages_required;
	size_t old_size;
	struct kbase_mem_pool *pool;
	int ret = -ENOMEM;
	struct tagged_addr *gpu_pages;

	if (info->commit_pages > reg->nr_pages) {
		/* Attempted to grow larger than maximum size */
		return -EINVAL;
	}

	lockdep_assert_held(&kctx->reg_lock);

	/* Make the physical backing no longer reclaimable */
	if (!kbase_mem_evictable_unmake(reg->gpu_alloc))
		goto update_failed;

	if (reg->gpu_alloc->nents >= info->commit_pages)
		goto done;

	/* Allocate some more pages */
	delta = info->commit_pages - reg->gpu_alloc->nents;
	pages_required = delta;

	if (kctx->kbdev->pagesize_2mb && pages_required >= NUM_PAGES_IN_2MB_LARGE_PAGE) {
		pool = &kctx->mem_pools.large[kctx->jit_group_id];
		/* Round up to number of 2 MB pages required */
		pages_required += (NUM_PAGES_IN_2MB_LARGE_PAGE - 1);
		pages_required /= NUM_PAGES_IN_2MB_LARGE_PAGE;
	} else {
		pool = &kctx->mem_pools.small[kctx->jit_group_id];
	}

	if (reg->cpu_alloc != reg->gpu_alloc)
		pages_required *= 2;

	spin_lock(&kctx->mem_partials_lock);
	kbase_mem_pool_lock(pool);

	/* As we can not allocate memory from the kernel with the vm_lock held,
	 * grow the pool to the required size with the lock dropped. We hold the
	 * pool lock to prevent another thread from allocating from the pool
	 * between the grow and allocation.
	 */
	while (kbase_mem_pool_size(pool) < pages_required) {
		size_t pool_delta = pages_required - kbase_mem_pool_size(pool);
		int ret;

		kbase_mem_pool_unlock(pool);
		spin_unlock(&kctx->mem_partials_lock);

		kbase_gpu_vm_unlock(kctx);
		ret = kbase_mem_pool_grow(pool, pool_delta, kctx->task);
		kbase_gpu_vm_lock(kctx);

		if (ret)
			goto update_failed;

		spin_lock(&kctx->mem_partials_lock);
		kbase_mem_pool_lock(pool);
	}

	if (reg->gpu_alloc->nents > info->commit_pages) {
		kbase_mem_pool_unlock(pool);
		spin_unlock(&kctx->mem_partials_lock);
		dev_warn(
			kctx->kbdev->dev,
			"JIT alloc grown beyond the required number of initially required pages, this grow no longer needed.");
		goto done;
	}

	old_size = reg->gpu_alloc->nents;
	delta = info->commit_pages - old_size;
	gpu_pages =
		kbase_alloc_phy_pages_helper_locked(reg->gpu_alloc, pool, delta, &prealloc_sas[0]);
	if (!gpu_pages) {
		kbase_mem_pool_unlock(pool);
		spin_unlock(&kctx->mem_partials_lock);
		goto update_failed;
	}

	if (reg->cpu_alloc != reg->gpu_alloc) {
		struct tagged_addr *cpu_pages;

		cpu_pages = kbase_alloc_phy_pages_helper_locked(reg->cpu_alloc, pool, delta,
								&prealloc_sas[1]);
		if (!cpu_pages) {
			kbase_free_phy_pages_helper_locked(reg->gpu_alloc, pool, gpu_pages, delta);
			kbase_mem_pool_unlock(pool);
			spin_unlock(&kctx->mem_partials_lock);
			goto update_failed;
		}
	}
	kbase_mem_pool_unlock(pool);
	spin_unlock(&kctx->mem_partials_lock);

	ret = kbase_mem_grow_gpu_mapping(kctx, reg, info->commit_pages, old_size, mmu_sync_info);
	/*
	 * The grow failed so put the allocation back in the
	 * pool and return failure.
	 */
	if (ret)
		goto update_failed;

done:
	ret = 0;

	/* Update attributes of JIT allocation taken from the pool */
	reg->initial_commit = info->commit_pages;
	reg->extension = info->extension;

update_failed:
	return ret;
}

static void trace_jit_stats(struct kbase_context *kctx, u32 bin_id, u32 max_allocations)
{
	const u32 alloc_count = kctx->jit_current_allocations_per_bin[bin_id];
	struct kbase_device *kbdev = kctx->kbdev;

	struct kbase_va_region *walker;
	u32 va_pages = 0;
	u32 ph_pages = 0;

	mutex_lock(&kctx->jit_evict_lock);
	list_for_each_entry(walker, &kctx->jit_active_head, jit_node) {
		if (walker->jit_bin_id != bin_id)
			continue;

		va_pages += walker->nr_pages;
		ph_pages += walker->gpu_alloc->nents;
	}
	mutex_unlock(&kctx->jit_evict_lock);

	KBASE_TLSTREAM_AUX_JIT_STATS(kbdev, kctx->id, bin_id, max_allocations, alloc_count,
				     va_pages, ph_pages);
}

#if MALI_JIT_PRESSURE_LIMIT_BASE
/**
 * get_jit_phys_backing() - calculate the physical backing of all JIT
 * allocations
 *
 * @kctx: Pointer to the kbase context whose active JIT allocations will be
 * checked
 *
 * Return: number of pages that are committed by JIT allocations
 */
static size_t get_jit_phys_backing(struct kbase_context *kctx)
{
	struct kbase_va_region *walker;
	size_t backing = 0;

	lockdep_assert_held(&kctx->jit_evict_lock);

	list_for_each_entry(walker, &kctx->jit_active_head, jit_node) {
		backing += kbase_reg_current_backed_size(walker);
	}

	return backing;
}

void kbase_jit_trim_necessary_pages(struct kbase_context *kctx, size_t needed_pages)
{
	size_t jit_backing = 0;
	size_t pages_to_trim = 0;

#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */
	lockdep_assert_held(&kctx->reg_lock);
	lockdep_assert_held(&kctx->jit_evict_lock);

	jit_backing = get_jit_phys_backing(kctx);

	/* It is possible that this is the case - if this is the first
	 * allocation after "ignore_pressure_limit" allocation.
	 */
	if (jit_backing > kctx->jit_phys_pages_limit) {
		pages_to_trim += (jit_backing - kctx->jit_phys_pages_limit) + needed_pages;
	} else {
		size_t backed_diff = kctx->jit_phys_pages_limit - jit_backing;

		if (needed_pages > backed_diff)
			pages_to_trim += needed_pages - backed_diff;
	}

	if (pages_to_trim) {
		size_t trimmed_pages = kbase_mem_jit_trim_pages(kctx, pages_to_trim);

		/* This should never happen - we already asserted that
		 * we are not violating JIT pressure limit in earlier
		 * checks, which means that in-flight JIT allocations
		 * must have enough unused pages to satisfy the new
		 * allocation
		 */
		WARN_ON(trimmed_pages < pages_to_trim);
	}
}
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

/**
 * jit_allow_allocate() - check whether basic conditions are satisfied to allow
 * a new JIT allocation
 *
 * @kctx: Pointer to the kbase context
 * @info: Pointer to JIT allocation information for the new allocation
 * @ignore_pressure_limit: Flag to indicate whether JIT pressure limit check
 * should be ignored
 *
 * Return: true if allocation can be executed, false otherwise
 */
static bool jit_allow_allocate(struct kbase_context *kctx, const struct base_jit_alloc_info *info,
			       bool ignore_pressure_limit)
{
#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#else /* MALI_USE_CSF */
	lockdep_assert_held(&kctx->csf.kcpu_queues.jit_lock);
#endif /* !MALI_USE_CSF */

#if MALI_JIT_PRESSURE_LIMIT_BASE
	if (!ignore_pressure_limit &&
	    ((kctx->jit_phys_pages_limit <= kctx->jit_current_phys_pressure) ||
	     (info->va_pages > (kctx->jit_phys_pages_limit - kctx->jit_current_phys_pressure)))) {
		dev_dbg(kctx->kbdev->dev,
			"Max JIT page allocations limit reached: active pages %llu, max pages %llu\n",
			kctx->jit_current_phys_pressure + info->va_pages,
			kctx->jit_phys_pages_limit);
		return false;
	}
#else
	CSTD_UNUSED(ignore_pressure_limit);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

	if (kctx->jit_current_allocations >= kctx->jit_max_allocations) {
		/* Too many current allocations */
		dev_dbg(kctx->kbdev->dev,
			"Max JIT allocations limit reached: active allocations %d, max allocations %d\n",
			kctx->jit_current_allocations, kctx->jit_max_allocations);
		return false;
	}

	if (info->max_allocations > 0 &&
	    kctx->jit_current_allocations_per_bin[info->bin_id] >= info->max_allocations) {
		/* Too many current allocations in this bin */
		dev_dbg(kctx->kbdev->dev,
			"Per bin limit of max JIT allocations reached: bin_id %d, active allocations %d, max allocations %d\n",
			info->bin_id, kctx->jit_current_allocations_per_bin[info->bin_id],
			info->max_allocations);
		return false;
	}

	return true;
}

static struct kbase_va_region *find_reasonable_region(const struct base_jit_alloc_info *info,
						      struct list_head *pool_head,
						      bool ignore_usage_id)
{
	struct kbase_va_region *closest_reg = NULL;
	struct kbase_va_region *walker;
	size_t current_diff = SIZE_MAX;

	list_for_each_entry(walker, pool_head, jit_node) {
		if ((ignore_usage_id || walker->jit_usage_id == info->usage_id) &&
		    walker->jit_bin_id == info->bin_id &&
		    meet_size_and_tiler_align_top_requirements(walker, info)) {
			size_t min_size, max_size, diff;

			/*
			 * The JIT allocations VA requirements have been met,
			 * it's suitable but other allocations might be a
			 * better fit.
			 */
			min_size = min_t(size_t, walker->gpu_alloc->nents, info->commit_pages);
			max_size = max_t(size_t, walker->gpu_alloc->nents, info->commit_pages);
			diff = max_size - min_size;

			if (current_diff > diff) {
				current_diff = diff;
				closest_reg = walker;
			}

			/* The allocation is an exact match */
			if (current_diff == 0)
				break;
		}
	}

	return closest_reg;
}

struct kbase_va_region *kbase_jit_allocate(struct kbase_context *kctx,
					   const struct base_jit_alloc_info *info,
					   bool ignore_pressure_limit)
{
	struct kbase_va_region *reg = NULL;
	struct kbase_sub_alloc *prealloc_sas[2] = { NULL, NULL };
	int i;

	/* Calls to this function are inherently synchronous, with respect to
	 * MMU operations.
	 */
	const enum kbase_caller_mmu_sync_info mmu_sync_info = CALLER_MMU_SYNC;

#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#else /* MALI_USE_CSF */
	lockdep_assert_held(&kctx->csf.kcpu_queues.jit_lock);
#endif /* !MALI_USE_CSF */

	if (!jit_allow_allocate(kctx, info, ignore_pressure_limit))
		return NULL;

	if (kctx->kbdev->pagesize_2mb) {
		/* Preallocate memory for the sub-allocation structs */
		for (i = 0; i != ARRAY_SIZE(prealloc_sas); ++i) {
			prealloc_sas[i] = kmalloc(sizeof(*prealloc_sas[i]), GFP_KERNEL);
			if (!prealloc_sas[i])
				goto end;
		}
	}

	kbase_gpu_vm_lock(kctx);
	mutex_lock(&kctx->jit_evict_lock);

	/*
	 * Scan the pool for an existing allocation which meets our
	 * requirements and remove it.
	 */
	if (info->usage_id != 0)
		/* First scan for an allocation with the same usage ID */
		reg = find_reasonable_region(info, &kctx->jit_pool_head, false);

	if (!reg)
		/* No allocation with the same usage ID, or usage IDs not in
		 * use. Search for an allocation we can reuse.
		 */
		reg = find_reasonable_region(info, &kctx->jit_pool_head, true);

	if (reg) {
#if MALI_JIT_PRESSURE_LIMIT_BASE
		size_t needed_pages = 0;
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */
		int ret;

		/*
		 * Remove the found region from the pool and add it to the
		 * active list.
		 */
		list_move(&reg->jit_node, &kctx->jit_active_head);

		WARN_ON(reg->gpu_alloc->evicted);

		/*
		 * Remove the allocation from the eviction list as it's no
		 * longer eligible for eviction. This must be done before
		 * dropping the jit_evict_lock
		 */
		list_del_init(&reg->gpu_alloc->evict_node);

#if MALI_JIT_PRESSURE_LIMIT_BASE
		if (!ignore_pressure_limit) {
			if (info->commit_pages > reg->gpu_alloc->nents)
				needed_pages = info->commit_pages - reg->gpu_alloc->nents;

			/* Update early the recycled JIT region's estimate of
			 * used_pages to ensure it doesn't get trimmed
			 * undesirably. This is needed as the recycled JIT
			 * region has been added to the active list but the
			 * number of used pages for it would be zero, so it
			 * could get trimmed instead of other allocations only
			 * to be regrown later resulting in a breach of the JIT
			 * physical pressure limit.
			 * Also that trimming would disturb the accounting of
			 * physical pages, i.e. the VM stats, as the number of
			 * backing pages would have changed when the call to
			 * kbase_mem_evictable_unmark_reclaim is made.
			 *
			 * The second call to update pressure at the end of
			 * this function would effectively be a nop.
			 */
			kbase_jit_report_update_pressure(kctx, reg, info->va_pages,
							 KBASE_JIT_REPORT_ON_ALLOC_OR_FREE);

			kbase_jit_request_phys_increase_locked(kctx, needed_pages);
		}
#endif
		mutex_unlock(&kctx->jit_evict_lock);

		/* kbase_jit_grow() can release & reacquire 'kctx->reg_lock',
		 * so any state protected by that lock might need to be
		 * re-evaluated if more code is added here in future.
		 */
		ret = kbase_jit_grow(kctx, info, reg, prealloc_sas, mmu_sync_info);

#if MALI_JIT_PRESSURE_LIMIT_BASE
		if (!ignore_pressure_limit)
			kbase_jit_done_phys_increase(kctx, needed_pages);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

		kbase_gpu_vm_unlock(kctx);

		if (ret) {
			/*
			 * An update to an allocation from the pool failed,
			 * chances are slim a new allocation would fare any
			 * better so return the allocation to the pool and
			 * return the function with failure.
			 */
			dev_dbg(kctx->kbdev->dev,
				"JIT allocation resize failed: va_pages 0x%llx, commit_pages 0x%llx\n",
				info->va_pages, info->commit_pages);
#if MALI_JIT_PRESSURE_LIMIT_BASE
			/* Undo the early change made to the recycled JIT
			 * region's estimate of used_pages.
			 */
			if (!ignore_pressure_limit) {
				kbase_jit_report_update_pressure(kctx, reg, 0,
								 KBASE_JIT_REPORT_ON_ALLOC_OR_FREE);
			}
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */
			mutex_lock(&kctx->jit_evict_lock);
			list_move(&reg->jit_node, &kctx->jit_pool_head);
			mutex_unlock(&kctx->jit_evict_lock);
			reg = NULL;
			goto end;
		} else {
			/* A suitable JIT allocation existed on the evict list, so we need
			 * to make sure that the NOT_MOVABLE property is cleared.
			 */
			if (kbase_is_page_migration_enabled()) {
				kbase_gpu_vm_lock(kctx);
				mutex_lock(&kctx->jit_evict_lock);
				kbase_set_phy_alloc_page_status(reg->gpu_alloc, ALLOCATED_MAPPED);
				mutex_unlock(&kctx->jit_evict_lock);
				kbase_gpu_vm_unlock(kctx);
			}
		}
	} else {
		/* No suitable JIT allocation was found so create a new one */
		u64 flags = BASE_MEM_PROT_CPU_RD | BASE_MEM_PROT_GPU_RD | BASE_MEM_PROT_GPU_WR |
			    BASE_MEM_GROW_ON_GPF | BASE_MEM_COHERENT_LOCAL | BASEP_MEM_NO_USER_FREE;
		u64 gpu_addr;

#if !MALI_USE_CSF
		if (info->flags & BASE_JIT_ALLOC_MEM_TILER_ALIGN_TOP)
			flags |= BASE_MEM_TILER_ALIGN_TOP;
#endif /* !MALI_USE_CSF */

		flags |= kbase_mem_group_id_set(kctx->jit_group_id);
#if MALI_JIT_PRESSURE_LIMIT_BASE
		if (!ignore_pressure_limit) {
			flags |= BASEP_MEM_PERFORM_JIT_TRIM;
			/* The corresponding call to 'done_phys_increase' would
			 * be made inside the kbase_mem_alloc().
			 */
			kbase_jit_request_phys_increase_locked(kctx, info->commit_pages);
		}
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

		mutex_unlock(&kctx->jit_evict_lock);
		kbase_gpu_vm_unlock(kctx);

		reg = kbase_mem_alloc(kctx, info->va_pages, info->commit_pages, info->extension,
				      &flags, &gpu_addr, mmu_sync_info);
		if (!reg) {
			/* Most likely not enough GPU virtual space left for
			 * the new JIT allocation.
			 */
			dev_dbg(kctx->kbdev->dev,
				"Failed to allocate JIT memory: va_pages 0x%llx, commit_pages 0x%llx\n",
				info->va_pages, info->commit_pages);
			goto end;
		}

		if (!ignore_pressure_limit) {
			/* Due to enforcing of pressure limit, kbase_mem_alloc
			 * was instructed to perform the trimming which in turn
			 * would have ensured that the new JIT allocation is
			 * already in the jit_active_head list, so nothing to
			 * do here.
			 */
			WARN_ON(list_empty(&reg->jit_node));
		} else {
			mutex_lock(&kctx->jit_evict_lock);
			list_add(&reg->jit_node, &kctx->jit_active_head);
			mutex_unlock(&kctx->jit_evict_lock);
		}
	}

	/* Similarly to tiler heap init, there is a short window of time
	 * where the (either recycled or newly allocated, in our case) region has
	 * "no user free" count incremented but is still missing the DONT_NEED flag, and
	 * doesn't yet have the ACTIVE_JIT_ALLOC flag either. Temporarily leaking the
	 * allocation is the least bad option that doesn't lead to a security issue down the
	 * line (it will eventually be cleaned up during context termination).
	 *
	 * We also need to call kbase_gpu_vm_lock regardless, as we're updating the region
	 * flags.
	 */
	kbase_gpu_vm_lock(kctx);
	if (unlikely(atomic64_read(&reg->no_user_free_count) > 1)) {
		kbase_gpu_vm_unlock(kctx);
		dev_err(kctx->kbdev->dev, "JIT region has no_user_free_count > 1!\n");

		mutex_lock(&kctx->jit_evict_lock);
		list_move(&reg->jit_node, &kctx->jit_pool_head);
		mutex_unlock(&kctx->jit_evict_lock);

		reg = NULL;
		goto end;
	}

	trace_mali_jit_alloc(reg, info->id);

	kctx->jit_current_allocations++;
	kctx->jit_current_allocations_per_bin[info->bin_id]++;

	trace_jit_stats(kctx, info->bin_id, info->max_allocations);

	reg->jit_usage_id = info->usage_id;
	reg->jit_bin_id = info->bin_id;
	reg->flags |= KBASE_REG_ACTIVE_JIT_ALLOC;
#if MALI_JIT_PRESSURE_LIMIT_BASE
	if (info->flags & BASE_JIT_ALLOC_HEAP_INFO_IS_SIZE)
		reg->flags = reg->flags | KBASE_REG_HEAP_INFO_IS_SIZE;
	reg->heap_info_gpu_addr = info->heap_info_gpu_addr;
	kbase_jit_report_update_pressure(kctx, reg, info->va_pages,
					 KBASE_JIT_REPORT_ON_ALLOC_OR_FREE);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */
	kbase_gpu_vm_unlock(kctx);

end:
	for (i = 0; i != ARRAY_SIZE(prealloc_sas); ++i)
		kfree(prealloc_sas[i]);

	return reg;
}

void kbase_jit_free(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	u64 old_pages;

#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#else /* MALI_USE_CSF */
	lockdep_assert_held(&kctx->csf.kcpu_queues.jit_lock);
#endif /* !MALI_USE_CSF */

	/* JIT id not immediately available here, so use 0u */
	trace_mali_jit_free(reg, 0u);

	/* Get current size of JIT region */
	old_pages = kbase_reg_current_backed_size(reg);
	if (reg->initial_commit < old_pages) {
		/* Free trim_level % of region, but don't go below initial
		 * commit size
		 */
		u64 new_size = MAX(reg->initial_commit,
				   div_u64(old_pages * (100ULL - kctx->trim_level), 100ULL));
		u64 delta = old_pages - new_size;

		if (delta) {
			mutex_lock(&kctx->reg_lock);
			kbase_mem_shrink(kctx, reg, old_pages - delta);
			mutex_unlock(&kctx->reg_lock);
		}
	}

#if MALI_JIT_PRESSURE_LIMIT_BASE
	reg->heap_info_gpu_addr = 0;
	kbase_jit_report_update_pressure(kctx, reg, 0, KBASE_JIT_REPORT_ON_ALLOC_OR_FREE);
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

	kctx->jit_current_allocations--;
	kctx->jit_current_allocations_per_bin[reg->jit_bin_id]--;

	trace_jit_stats(kctx, reg->jit_bin_id, UINT_MAX);

	kbase_mem_evictable_mark_reclaim(reg->gpu_alloc);

	kbase_gpu_vm_lock(kctx);
	reg->flags |= KBASE_REG_DONT_NEED;
	reg->flags &= ~KBASE_REG_ACTIVE_JIT_ALLOC;
	kbase_mem_shrink_cpu_mapping(kctx, reg, 0, reg->gpu_alloc->nents);
	kbase_gpu_vm_unlock(kctx);

	/*
	 * Add the allocation to the eviction list and the jit pool, after this
	 * point the shrink can reclaim it, or it may be reused.
	 */
	mutex_lock(&kctx->jit_evict_lock);

	/* This allocation can't already be on a list. */
	WARN_ON(!list_empty(&reg->gpu_alloc->evict_node));
	list_add(&reg->gpu_alloc->evict_node, &kctx->evict_list);
	atomic_add(reg->gpu_alloc->nents, &kctx->evict_nents);

	list_move(&reg->jit_node, &kctx->jit_pool_head);

	/* Inactive JIT regions should be freed by the shrinker and not impacted
	 * by page migration. Once freed, they will enter into the page migration
	 * state machine via the mempools.
	 */
	if (kbase_is_page_migration_enabled())
		kbase_set_phy_alloc_page_status(reg->gpu_alloc, NOT_MOVABLE);
	mutex_unlock(&kctx->jit_evict_lock);
}

void kbase_jit_backing_lost(struct kbase_va_region *reg)
{
	struct kbase_context *kctx = kbase_reg_to_kctx(reg);

	if (WARN_ON(!kctx))
		return;

	lockdep_assert_held(&kctx->jit_evict_lock);

	/*
	 * JIT allocations will always be on a list, if the region
	 * is not on a list then it's not a JIT allocation.
	 */
	if (list_empty(&reg->jit_node))
		return;

	/*
	 * Freeing the allocation requires locks we might not be able
	 * to take now, so move the allocation to the free list and kick
	 * the worker which will do the freeing.
	 */
	list_move(&reg->jit_node, &kctx->jit_destroy_head);

	schedule_work(&kctx->jit_work);
}

bool kbase_jit_evict(struct kbase_context *kctx)
{
	struct kbase_va_region *reg = NULL;

	lockdep_assert_held(&kctx->reg_lock);

	/* Free the oldest allocation from the pool */
	mutex_lock(&kctx->jit_evict_lock);
	if (!list_empty(&kctx->jit_pool_head)) {
		reg = list_entry(kctx->jit_pool_head.prev, struct kbase_va_region, jit_node);
		list_del(&reg->jit_node);
		list_del_init(&reg->gpu_alloc->evict_node);
	}
	mutex_unlock(&kctx->jit_evict_lock);

	if (reg) {
		/*
		 * Incrementing the refcount is prevented on JIT regions.
		 * If/when this ever changes we would need to compensate
		 * by implementing "free on putting the last reference",
		 * but only for JIT regions.
		 */
		WARN_ON(atomic64_read(&reg->no_user_free_count) > 1);
		kbase_va_region_no_user_free_dec(reg);
		kbase_mem_free_region(kctx, reg);
	}

	return (reg != NULL);
}

void kbase_jit_term(struct kbase_context *kctx)
{
	struct kbase_va_region *walker;

	/* Free all allocations for this context */

	kbase_gpu_vm_lock(kctx);
	mutex_lock(&kctx->jit_evict_lock);
	/* Free all allocations from the pool */
	while (!list_empty(&kctx->jit_pool_head)) {
		walker = list_first_entry(&kctx->jit_pool_head, struct kbase_va_region, jit_node);
		list_del(&walker->jit_node);
		list_del_init(&walker->gpu_alloc->evict_node);
		mutex_unlock(&kctx->jit_evict_lock);
		/*
		 * Incrementing the refcount is prevented on JIT regions.
		 * If/when this ever changes we would need to compensate
		 * by implementing "free on putting the last reference",
		 * but only for JIT regions.
		 */
		WARN_ON(atomic64_read(&walker->no_user_free_count) > 1);
		kbase_va_region_no_user_free_dec(walker);
		kbase_mem_free_region(kctx, walker);
		mutex_lock(&kctx->jit_evict_lock);
	}

	/* Free all allocations from active list */
	while (!list_empty(&kctx->jit_active_head)) {
		walker = list_first_entry(&kctx->jit_active_head, struct kbase_va_region, jit_node);
		list_del(&walker->jit_node);
		list_del_init(&walker->gpu_alloc->evict_node);
		mutex_unlock(&kctx->jit_evict_lock);
		/*
		 * Incrementing the refcount is prevented on JIT regions.
		 * If/when this ever changes we would need to compensate
		 * by implementing "free on putting the last reference",
		 * but only for JIT regions.
		 */
		WARN_ON(atomic64_read(&walker->no_user_free_count) > 1);
		kbase_va_region_no_user_free_dec(walker);
		kbase_mem_free_region(kctx, walker);
		mutex_lock(&kctx->jit_evict_lock);
	}
#if MALI_JIT_PRESSURE_LIMIT_BASE
	WARN_ON(kctx->jit_phys_pages_to_be_allocated);
#endif
	mutex_unlock(&kctx->jit_evict_lock);
	kbase_gpu_vm_unlock(kctx);

	/*
	 * Flush the freeing of allocations whose backing has been freed
	 * (i.e. everything in jit_destroy_head).
	 */
	cancel_work_sync(&kctx->jit_work);
}

#if MALI_JIT_PRESSURE_LIMIT_BASE
void kbase_trace_jit_report_gpu_mem_trace_enabled(struct kbase_context *kctx,
						  struct kbase_va_region *reg, unsigned int flags)
{
	/* Offset to the location used for a JIT report within the GPU memory
	 *
	 * This constants only used for this debugging function - not useful
	 * anywhere else in kbase
	 */
	const u64 jit_report_gpu_mem_offset = sizeof(u64) * 2;

	u64 addr_start;
	struct kbase_vmap_struct mapping;
	u64 *ptr;

	if (reg->heap_info_gpu_addr == 0ull)
		goto out;

	/* Nothing else to trace in the case the memory just contains the
	 * size. Other tracepoints already record the relevant area of memory.
	 */
	if (reg->flags & KBASE_REG_HEAP_INFO_IS_SIZE)
		goto out;

	addr_start = reg->heap_info_gpu_addr - jit_report_gpu_mem_offset;

	ptr = kbase_vmap_prot(kctx, addr_start, KBASE_JIT_REPORT_GPU_MEM_SIZE, KBASE_REG_CPU_RD,
			      &mapping);
	if (!ptr) {
		dev_warn(kctx->kbdev->dev,
			 "%s: JIT start=0x%llx unable to map memory near end pointer %llx\n",
			 __func__, reg->start_pfn << PAGE_SHIFT, addr_start);
		goto out;
	}

	trace_mali_jit_report_gpu_mem(addr_start, reg->start_pfn << PAGE_SHIFT, ptr, flags);

	kbase_vunmap(kctx, &mapping);
out:
	return;
}
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

#if MALI_JIT_PRESSURE_LIMIT_BASE
void kbase_jit_report_update_pressure(struct kbase_context *kctx, struct kbase_va_region *reg,
				      u64 new_used_pages, unsigned int flags)
{
	u64 diff;

#if !MALI_USE_CSF
	lockdep_assert_held(&kctx->jctx.lock);
#endif /* !MALI_USE_CSF */

	trace_mali_jit_report_pressure(
		reg, new_used_pages,
		kctx->jit_current_phys_pressure + new_used_pages - reg->used_pages, flags);

	if (WARN_ON(new_used_pages > reg->nr_pages))
		return;

	if (reg->used_pages > new_used_pages) {
		/* We reduced the number of used pages */
		diff = reg->used_pages - new_used_pages;

		if (!WARN_ON(diff > kctx->jit_current_phys_pressure))
			kctx->jit_current_phys_pressure -= diff;

		reg->used_pages = new_used_pages;
	} else {
		/* We increased the number of used pages */
		diff = new_used_pages - reg->used_pages;

		if (!WARN_ON(diff > U64_MAX - kctx->jit_current_phys_pressure))
			kctx->jit_current_phys_pressure += diff;

		reg->used_pages = new_used_pages;
	}
}
#endif /* MALI_JIT_PRESSURE_LIMIT_BASE */

int kbase_user_buf_pin_pages(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	struct kbase_mem_phy_alloc *alloc = reg->gpu_alloc;
	struct page **pages = alloc->imported.user_buf.pages;
	unsigned long address = alloc->imported.user_buf.address;
	struct mm_struct *mm = alloc->imported.user_buf.mm;
	struct tagged_addr *pa = kbase_get_gpu_phy_pages(reg);
	long pinned_pages;
	long i;
	int write;

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_IMPORTED_USER_BUF))
		return -EINVAL;

	if (WARN_ON(alloc->nents))
		return -EINVAL;

	if (WARN_ON(reg->gpu_alloc->imported.user_buf.mm != current->mm))
		return -EINVAL;

	if (WARN_ON(!(reg->flags & KBASE_REG_CPU_CACHED)))
		return -EINVAL;

	write = reg->flags & (KBASE_REG_CPU_WR | KBASE_REG_GPU_WR);

	pinned_pages = kbase_pin_user_pages_remote(NULL, mm, address,
						   alloc->imported.user_buf.nr_pages,
						   write ? FOLL_WRITE : 0, pages, NULL, NULL);

	if (pinned_pages <= 0)
		return pinned_pages;

	if (pinned_pages != (long)alloc->imported.user_buf.nr_pages) {
		/* Above code already ensures there will not have been a CPU
		 * mapping by ensuring alloc->nents is 0
		 */
		for (i = 0; i < pinned_pages; i++)
			kbase_unpin_user_buf_page(pages[i]);
		return -ENOMEM;
	}

	/* The driver is allowed to create CPU mappings now that physical pages
	 * have been pinned. Update physical allocation in a consistent way:
	 * update the number of available physical pages and at the same time
	 * fill the array of physical pages with tagged addresses.
	 */
	for (i = 0; i < pinned_pages; i++)
		pa[i] = as_tagged(page_to_phys(pages[i]));
	alloc->nents = (size_t)pinned_pages;

	return 0;
}

void kbase_user_buf_unpin_pages(struct kbase_mem_phy_alloc *alloc)
{
	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_IMPORTED_USER_BUF))
		return;

	if (alloc->nents) {
		struct page **pages = alloc->imported.user_buf.pages;
		long i;

		WARN_ON(alloc->nents != alloc->imported.user_buf.nr_pages);

		for (i = 0; i < alloc->nents; i++)
			kbase_unpin_user_buf_page(pages[i]);

		alloc->nents = 0;
	}
}

int kbase_user_buf_dma_map_pages(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	struct kbase_mem_phy_alloc *alloc = reg->gpu_alloc;
	struct page **pages = alloc->imported.user_buf.pages;
	struct device *dev = kctx->kbdev->dev;
	int write;
	size_t i, pinned_pages, dma_mapped_pages;
	enum dma_data_direction dma_dir;

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_IMPORTED_USER_BUF))
		return -EINVAL;

	write = reg->flags & (KBASE_REG_CPU_WR | KBASE_REG_GPU_WR);
	dma_dir = write ? DMA_BIDIRECTIONAL : DMA_TO_DEVICE;
	pinned_pages = reg->gpu_alloc->nents;

	/* Manual CPU cache synchronization.
	 *
	 * The driver disables automatic CPU cache synchronization because the
	 * memory pages that enclose the imported region may also contain
	 * sub-regions which are not imported and that are allocated and used
	 * by the user process. This may be the case of memory at the beginning
	 * of the first page and at the end of the last page. Automatic CPU cache
	 * synchronization would force some operations on those memory allocations,
	 * unbeknown to the user process: in particular, a CPU cache invalidate
	 * upon unmapping would destroy the content of dirty CPU caches and cause
	 * the user process to lose CPU writes to the non-imported sub-regions.
	 *
	 * When the GPU claims ownership of the imported memory buffer, it shall
	 * commit CPU writes for the whole of all pages that enclose the imported
	 * region, otherwise the initial content of memory would be wrong.
	 */
	for (i = 0; i < pinned_pages; i++) {
		dma_addr_t dma_addr;
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
		dma_addr = dma_map_page(dev, pages[i], 0, PAGE_SIZE, dma_dir);
#else
		dma_addr = dma_map_page_attrs(dev, pages[i], 0, PAGE_SIZE, dma_dir,
					      DMA_ATTR_SKIP_CPU_SYNC);
#endif
		if (dma_mapping_error(dev, dma_addr))
			goto unwind_dma_map;

		alloc->imported.user_buf.dma_addrs[i] = dma_addr;

		dma_sync_single_for_device(dev, dma_addr, PAGE_SIZE, dma_dir);
	}

	return 0;

unwind_dma_map:
	dma_mapped_pages = i;

	/* Run the unmap loop in the same order as map loop, and perform again
	 * CPU cache synchronization to re-write the content of dirty CPU caches
	 * to memory as a precautionary measure.
	 */
	for (i = 0; i < dma_mapped_pages; i++) {
		dma_addr_t dma_addr = alloc->imported.user_buf.dma_addrs[i];

		dma_sync_single_for_device(dev, dma_addr, PAGE_SIZE, dma_dir);
#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
		dma_unmap_page(dev, dma_addr, PAGE_SIZE, dma_dir);
#else
		dma_unmap_page_attrs(dev, dma_addr, PAGE_SIZE, dma_dir, DMA_ATTR_SKIP_CPU_SYNC);
#endif
	}

	return -ENOMEM;
}

/**
 * kbase_user_buf_map - Create GPU mapping for a user buffer.
 * @kctx: kbase context.
 * @reg:  The region associated with the imported user buffer.
 *
 * The caller must have ensured that physical pages have been pinned and that
 * DMA mappings have been obtained prior to calling this function.
 *
 * Return: zero on success or negative number on failure.
 */
static int kbase_user_buf_map(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	size_t pinned_pages = 0;
	struct kbase_mem_phy_alloc *alloc;
	struct page **pages;
	struct tagged_addr *pa;
	size_t i;
	unsigned long gwt_mask = ~0UL;
	int ret;
	/* Calls to this function are inherently asynchronous, with respect to
	 * MMU operations.
	 */
	const enum kbase_caller_mmu_sync_info mmu_sync_info = CALLER_MMU_ASYNC;

	lockdep_assert_held(&kctx->reg_lock);

	alloc = reg->gpu_alloc;

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_IMPORTED_USER_BUF))
		return -EINVAL;

	pa = kbase_get_gpu_phy_pages(reg);
	pinned_pages = alloc->nents;
	pages = alloc->imported.user_buf.pages;

	for (i = 0; i < pinned_pages; i++)
		pa[i] = as_tagged(page_to_phys(pages[i]));

#ifdef CONFIG_MALI_CINSTR_GWT
	if (kctx->gwt_enabled)
		gwt_mask = ~KBASE_REG_GPU_WR;
#endif

	ret = kbase_mmu_insert_pages_skip_status_update(kctx->kbdev, &kctx->mmu, reg->start_pfn, pa,
							kbase_reg_current_backed_size(reg),
							reg->flags & gwt_mask, kctx->as_nr,
							alloc->group_id, mmu_sync_info, NULL);

	return ret;
}

/* user_buf_sync_read_only_page - This function handles syncing a single page that has read access,
 *                                only, on both the CPU and * GPU, so it is ready to be unmapped.
 * @kctx: kbase context
 * @imported_size: the number of bytes to sync
 * @dma_addr: DMA address of the bytes to be sync'd
 * @offset_within_page: (unused) offset of the bytes within the page. Passed so that the calling
 * signature is identical to user_buf_sync_writable_page().
 */
static void user_buf_sync_read_only_page(struct kbase_context *kctx, unsigned long imported_size,
					 dma_addr_t dma_addr, unsigned long offset_within_page)
{
	/* Manual cache synchronization.
	 *
	 * Writes from neither the CPU nor GPU are possible via this mapping,
	 * so we just sync the entire page to the device.
	 */
	CSTD_UNUSED(offset_within_page);

	dma_sync_single_for_device(kctx->kbdev->dev, dma_addr, imported_size, DMA_TO_DEVICE);
}

/* user_buf_sync_writable_page - This function handles syncing a single page that has read
 *                                and writable access, from either (or both of) the CPU and GPU,
 *                                so it is ready to be unmapped.
 * @kctx: kbase context
 * @imported_size: the number of bytes to unmap
 * @dma_addr: DMA address of the bytes to be unmapped
 * @offset_within_page: offset of the bytes within the page. This is the offset to the subrange of
 *                      the memory that is "imported" and so is intended for GPU access. Areas of
 *                      the page outside of this - whilst still GPU accessible - are not intended
 *                      for use by GPU work, and should also not be modified as the userspace CPU
 *                      threads may be modifying them.
 */
static void user_buf_sync_writable_page(struct kbase_context *kctx, unsigned long imported_size,
					dma_addr_t dma_addr, unsigned long offset_within_page)
{
	/* Manual CPU cache synchronization.
	 *
	 * When the GPU returns ownership of the buffer to the CPU, the driver
	 * needs to treat imported and non-imported memory differently.
	 *
	 * The first case to consider is non-imported sub-regions at the
	 * beginning of the first page and at the end of last page. For these
	 * sub-regions: CPU cache shall be committed with a clean+invalidate,
	 * in order to keep the last CPU write.
	 *
	 * Imported region prefers the opposite treatment: this memory has been
	 * legitimately mapped and used by the GPU, hence GPU writes shall be
	 * committed to memory, while CPU cache shall be invalidated to make
	 * sure that CPU reads the correct memory content.
	 *
	 * The following diagram shows the expect value of the variables
	 * used in this loop in the corner case of an imported region encloed
	 * by a single memory page:
	 *
	 * page boundary ->|---------- | <- dma_addr (initial value)
	 *                 |           |
	 *                 | - - - - - | <- offset_within_page
	 *                 |XXXXXXXXXXX|\
	 *                 |XXXXXXXXXXX| \
	 *                 |XXXXXXXXXXX|  }- imported_size
	 *                 |XXXXXXXXXXX| /
	 *                 |XXXXXXXXXXX|/
	 *                 | - - - - - | <- offset_within_page + imported_size
	 *                 |           |\
	 *                 |           | }- PAGE_SIZE - imported_size -
	 *                 |           |/   offset_within_page
	 *                 |           |
	 * page boundary ->|-----------|
	 *
	 * If the imported region is enclosed by more than one page, then
	 * offset_within_page = 0 for any page after the first.
	 */

	/* Only for first page: handle non-imported range at the beginning. */
	if (offset_within_page > 0) {
		dma_sync_single_for_device(kctx->kbdev->dev, dma_addr, offset_within_page,
					   DMA_BIDIRECTIONAL);
		dma_addr += offset_within_page;
	}

	/* For every page: handle imported range. */
	if (imported_size > 0)
		dma_sync_single_for_cpu(kctx->kbdev->dev, dma_addr, imported_size,
					DMA_BIDIRECTIONAL);

	/* Only for last page (that may coincide with first page):
	 * handle non-imported range at the end.
	 */
	if ((imported_size + offset_within_page) < PAGE_SIZE) {
		dma_addr += imported_size;
		dma_sync_single_for_device(kctx->kbdev->dev, dma_addr,
					   PAGE_SIZE - imported_size - offset_within_page,
					   DMA_BIDIRECTIONAL);
	}
}

void kbase_user_buf_dma_unmap_pages(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	long i;
	struct kbase_mem_phy_alloc *alloc = reg->gpu_alloc;
	unsigned long offset_within_page = alloc->imported.user_buf.address & ~PAGE_MASK;
	unsigned long remaining_size = alloc->imported.user_buf.size;

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_IMPORTED_USER_BUF))
		return;

	for (i = 0; i < alloc->imported.user_buf.nr_pages; i++) {
		/* The DMA unmapping operation affects the whole of every page,
		 * but cache maintenance shall be limited only to the imported
		 * address range.
		 *
		 * Notice: this is a temporary variable that is used for DMA sync
		 * operations, and that could be incremented by an offset if the
		 * current page contains both imported and non-imported memory
		 * sub-regions.
		 *
		 * It is valid to add an offset to this value, because the offset
		 * is always kept within the physically contiguous dma-mapped range
		 * and there's no need to translate to physical address to offset it.
		 *
		 * This variable is not going to be used for the actual DMA unmap
		 * operation, that shall always use the original DMA address of the
		 * whole memory page.
		 */
		unsigned long imported_size = MIN(remaining_size, PAGE_SIZE - offset_within_page);
		dma_addr_t dma_addr = alloc->imported.user_buf.dma_addrs[i];
		struct page **pages = alloc->imported.user_buf.pages;
		bool writable = (reg->flags & (KBASE_REG_CPU_WR | KBASE_REG_GPU_WR));
		enum dma_data_direction dma_dir = writable ? DMA_BIDIRECTIONAL : DMA_TO_DEVICE;

		if (writable)
			user_buf_sync_writable_page(kctx, imported_size, dma_addr,
						    offset_within_page);
		else
			user_buf_sync_read_only_page(kctx, imported_size, dma_addr,
						     offset_within_page);

#if (KERNEL_VERSION(4, 10, 0) > LINUX_VERSION_CODE)
		dma_unmap_page(kctx->kbdev->dev, alloc->imported.user_buf.dma_addrs[i], PAGE_SIZE,
			       dma_dir);
#else
		dma_unmap_page_attrs(kctx->kbdev->dev, alloc->imported.user_buf.dma_addrs[i],
				     PAGE_SIZE, dma_dir, DMA_ATTR_SKIP_CPU_SYNC);
#endif

		if (writable)
			set_page_dirty_lock(pages[i]);

		remaining_size -= imported_size;
		offset_within_page = 0;
	}
}

/**
 * kbase_user_buf_unmap - Destroy GPU mapping for a user buffer.
 * @kctx: kbase context.
 * @reg:  The region associated with the imported user buffer.
 *
 * Destroy the GPU mapping for an imported user buffer. Notice that this
 * function doesn't release DMA mappings and doesn't unpin physical pages.
 */
static void kbase_user_buf_unmap(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	struct kbase_mem_phy_alloc *alloc = reg->gpu_alloc;

	lockdep_assert_held(&kctx->reg_lock);

	if (WARN_ON(alloc->type != KBASE_MEM_TYPE_IMPORTED_USER_BUF))
		return;

	if (WARN_ON(alloc->imported.user_buf.current_mapping_usage_count > 0))
		return;

	if (!kbase_is_region_invalid_or_free(reg)) {
		kbase_mmu_teardown_imported_pages(kctx->kbdev, &kctx->mmu, reg->start_pfn,
						  alloc->pages, kbase_reg_current_backed_size(reg),
						  kbase_reg_current_backed_size(reg), kctx->as_nr);
	}
}

int kbase_mem_copy_to_pinned_user_pages(struct page **dest_pages, void *src_page, size_t *to_copy,
					unsigned int nr_pages, unsigned int *target_page_nr,
					size_t offset)
{
	void *target_page = kbase_kmap(dest_pages[*target_page_nr]);

	size_t chunk = PAGE_SIZE - offset;

	if (!target_page) {
		pr_err("%s: kmap failure", __func__);
		return -ENOMEM;
	}

	chunk = min(chunk, *to_copy);

	memcpy(target_page + offset, src_page, chunk);
	*to_copy -= chunk;

	kbase_kunmap(dest_pages[*target_page_nr], target_page);

	*target_page_nr += 1;
	if (*target_page_nr >= nr_pages || *to_copy == 0)
		return 0;

	target_page = kbase_kmap(dest_pages[*target_page_nr]);
	if (!target_page) {
		pr_err("%s: kmap failure", __func__);
		return -ENOMEM;
	}

	KBASE_DEBUG_ASSERT(target_page);

	chunk = min(offset, *to_copy);
	memcpy(target_page, src_page + PAGE_SIZE - offset, chunk);
	*to_copy -= chunk;

	kbase_kunmap(dest_pages[*target_page_nr], target_page);

	return 0;
}

int kbase_map_external_resource(struct kbase_context *kctx, struct kbase_va_region *reg,
				struct mm_struct *locked_mm)
{
	int err = 0;
	struct kbase_mem_phy_alloc *alloc = reg->gpu_alloc;
	enum kbase_user_buf_state user_buf_original_state;

	lockdep_assert_held(&kctx->reg_lock);

	/* decide what needs to happen for this resource */
	switch (reg->gpu_alloc->type) {
	case KBASE_MEM_TYPE_IMPORTED_USER_BUF: {
		user_buf_original_state = reg->gpu_alloc->imported.user_buf.state;

		if ((reg->gpu_alloc->imported.user_buf.mm != locked_mm) && (!reg->gpu_alloc->nents))
			return -EINVAL;

		/* This function is reachable through many code paths, and the imported
		 * memory handle could be in any of the possible states: consider all
		 * of them as a valid starting point, and progress through all stages
		 * until creating a GPU mapping or increasing the reference count if
		 * the handle is already mapped.
		 *
		 * Error recovery restores the original state and goes no further.
		 */
		switch (user_buf_original_state) {
		case KBASE_USER_BUF_STATE_EMPTY:
		case KBASE_USER_BUF_STATE_PINNED:
		case KBASE_USER_BUF_STATE_DMA_MAPPED: {
			if (user_buf_original_state == KBASE_USER_BUF_STATE_EMPTY)
				err = kbase_user_buf_from_empty_to_gpu_mapped(kctx, reg);
			else if (user_buf_original_state == KBASE_USER_BUF_STATE_PINNED)
				err = kbase_user_buf_from_pinned_to_gpu_mapped(kctx, reg);
			else
				err = kbase_user_buf_from_dma_mapped_to_gpu_mapped(kctx, reg);

			if (err)
				return err;

			break;
		}
		case KBASE_USER_BUF_STATE_GPU_MAPPED: {
			if (reg->gpu_alloc->imported.user_buf.current_mapping_usage_count == 0)
				return -EINVAL;
			break;
		}
		default:
			dev_dbg(kctx->kbdev->dev,
				"Invalid external resource GPU allocation state (%x) on mapping",
				reg->gpu_alloc->imported.user_buf.state);
			return -EINVAL;
		}

		/* If the state was valid and the transition is happening, then the handle
		 * must be in GPU_MAPPED state now and the reference counter of GPU mappings
		 * can be safely incremented.
		 */
		reg->gpu_alloc->imported.user_buf.current_mapping_usage_count++;
		break;
	}
	case KBASE_MEM_TYPE_IMPORTED_UMM: {
		err = kbase_mem_umm_map(kctx, reg);
		if (err)
			return err;
		break;
	}
	default:
		dev_dbg(kctx->kbdev->dev,
			"Invalid external resource GPU allocation type (%x) on mapping",
			alloc->type);
		return -EINVAL;
	}

	kbase_va_region_alloc_get(kctx, reg);
	kbase_mem_phy_alloc_get(alloc);
	return 0;
}

void kbase_unmap_external_resource(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	/* gpu_alloc was used in kbase_map_external_resources, so we need to use it for the
	 * unmapping operation.
	 */
	struct kbase_mem_phy_alloc *alloc = reg->gpu_alloc;

	lockdep_assert_held(&kctx->reg_lock);

	switch (alloc->type) {
	case KBASE_MEM_TYPE_IMPORTED_UMM: {
		kbase_mem_umm_unmap(kctx, reg, alloc);
	} break;
	case KBASE_MEM_TYPE_IMPORTED_USER_BUF: {
		switch (alloc->imported.user_buf.state) {
		case KBASE_USER_BUF_STATE_GPU_MAPPED: {
			alloc->imported.user_buf.current_mapping_usage_count--;
			if (alloc->imported.user_buf.current_mapping_usage_count == 0)
				kbase_user_buf_from_gpu_mapped_to_pinned(kctx, reg);
			break;
		}
		case KBASE_USER_BUF_STATE_DMA_MAPPED: {
			kbase_user_buf_from_dma_mapped_to_pinned(kctx, reg);
			break;
		}
		case KBASE_USER_BUF_STATE_PINNED:
		case KBASE_USER_BUF_STATE_EMPTY:
		default: {
			/* nothing to do */
		} break;
		}
	} break;
	default:
		WARN(1, "Invalid external resource GPU allocation type (%x) on unmapping",
		     alloc->type);
		return;
	}
	kbase_mem_phy_alloc_put(alloc);
	kbase_va_region_alloc_put(kctx, reg);
}

static inline u64 kbasep_get_va_gpu_addr(struct kbase_va_region *reg)
{
	return reg->start_pfn << PAGE_SHIFT;
}

struct kbase_ctx_ext_res_meta *kbase_sticky_resource_acquire(struct kbase_context *kctx,
							     u64 gpu_addr)
{
	struct kbase_ctx_ext_res_meta *meta = NULL;
	struct kbase_ctx_ext_res_meta *walker;
	struct kbase_va_region *reg;

	lockdep_assert_held(&kctx->reg_lock);

	/*
	 * Walk the per context external resource metadata list for the
	 * metadata which matches the region which is being acquired.
	 */
	reg = kbase_region_tracker_find_region_enclosing_address(kctx, gpu_addr);
	if (kbase_is_region_invalid_or_free(reg))
		goto failed;

	list_for_each_entry(walker, &kctx->ext_res_meta_head, ext_res_node) {
		if (walker->reg == reg) {
			meta = walker;
			meta->ref++;
			break;
		}
	}

	/* If no metadata exists in the list, create one. */
	if (!meta) {
		/* Allocate the metadata object */
		meta = kzalloc(sizeof(*meta), GFP_KERNEL);
		if (!meta)
			goto failed;
		/*
		 * Fill in the metadata object and acquire a reference
		 * for the physical resource.
		 */
		meta->reg = reg;

		/* Map the external resource to the GPU allocation of the region
		 * and acquire the reference to the VA region
		 */
		if (kbase_map_external_resource(kctx, meta->reg, NULL))
			goto fail_map;
		meta->ref = 1;

		list_add(&meta->ext_res_node, &kctx->ext_res_meta_head);
	}

	return meta;

fail_map:
	kfree(meta);
failed:
	return NULL;
}

static struct kbase_ctx_ext_res_meta *find_sticky_resource_meta(struct kbase_context *kctx,
								u64 gpu_addr)
{
	struct kbase_ctx_ext_res_meta *walker;
	struct kbase_va_region *reg;
	lockdep_assert_held(&kctx->reg_lock);

	/*
	 * Walk the per context external resource metadata list for the
	 * metadata which matches the region which is being released.
	 */
	reg = kbase_region_tracker_find_region_enclosing_address(kctx, gpu_addr);
	if (!reg)
		return NULL;

	list_for_each_entry(walker, &kctx->ext_res_meta_head, ext_res_node) {
		if (walker->reg == reg)
			return walker;
	}

	return NULL;
}

static void release_sticky_resource_meta(struct kbase_context *kctx,
					 struct kbase_ctx_ext_res_meta *meta)
{
	kbase_unmap_external_resource(kctx, meta->reg);
	list_del(&meta->ext_res_node);
	kfree(meta);
}

bool kbase_sticky_resource_release(struct kbase_context *kctx, struct kbase_ctx_ext_res_meta *meta,
				   u64 gpu_addr)
{
	lockdep_assert_held(&kctx->reg_lock);

	/* Search of the metadata if one isn't provided. */
	if (!meta)
		meta = find_sticky_resource_meta(kctx, gpu_addr);

	/* No metadata so just return. */
	if (!meta)
		return false;

	if (--meta->ref != 0)
		return true;

	release_sticky_resource_meta(kctx, meta);

	return true;
}

bool kbase_sticky_resource_release_force(struct kbase_context *kctx,
					 struct kbase_ctx_ext_res_meta *meta, u64 gpu_addr)
{
	lockdep_assert_held(&kctx->reg_lock);

	/* Search of the metadata if one isn't provided. */
	if (!meta)
		meta = find_sticky_resource_meta(kctx, gpu_addr);

	/* No metadata so just return. */
	if (!meta)
		return false;

	release_sticky_resource_meta(kctx, meta);

	return true;
}

int kbase_sticky_resource_init(struct kbase_context *kctx)
{
	INIT_LIST_HEAD(&kctx->ext_res_meta_head);

	return 0;
}

void kbase_sticky_resource_term(struct kbase_context *kctx)
{
	struct kbase_ctx_ext_res_meta *walker;

	lockdep_assert_held(&kctx->reg_lock);

	/*
	 * Free any sticky resources which haven't been unmapped.
	 *
	 * Note:
	 * We don't care about refcounts at this point as no future
	 * references to the meta data will be made.
	 * Region termination would find these if we didn't free them
	 * here, but it's more efficient if we do the clean up here.
	 */
	while (!list_empty(&kctx->ext_res_meta_head)) {
		walker = list_first_entry(&kctx->ext_res_meta_head, struct kbase_ctx_ext_res_meta,
					  ext_res_node);

		kbase_sticky_resource_release_force(kctx, walker, 0);
	}
}

void kbase_user_buf_empty_init(struct kbase_va_region *reg)
{
	reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_EMPTY;
	/* Code currently manages transitions among 4 states.
	 * This is a reminder that code needs to be updated if a new state
	 * is introduced.
	 */
	BUILD_BUG_ON(KBASE_USER_BUF_STATE_COUNT != 4);
}

int kbase_user_buf_from_empty_to_pinned(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	int ret;

	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);

	if (reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_EMPTY)
		return -EINVAL;

	ret = kbase_user_buf_pin_pages(kctx, reg);

	if (!ret)
		reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_PINNED;

	return ret;
}

int kbase_user_buf_from_empty_to_dma_mapped(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	int ret;

	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);

	if (reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_EMPTY)
		return -EINVAL;

	ret = kbase_user_buf_pin_pages(kctx, reg);

	if (ret)
		goto pin_pages_fail;

	ret = kbase_user_buf_dma_map_pages(kctx, reg);

	if (!ret)
		reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_DMA_MAPPED;
	else
		goto dma_map_pages_fail;

	return ret;

dma_map_pages_fail:
	/* The user buffer could already have been previously pinned before
	 * entering this function, and hence there could potentially be CPU
	 * mappings of it.
	 */
	kbase_mem_shrink_cpu_mapping(kctx, reg, 0, reg->gpu_alloc->nents);
	kbase_user_buf_unpin_pages(reg->gpu_alloc);
pin_pages_fail:
	return ret;
}

int kbase_user_buf_from_empty_to_gpu_mapped(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	int ret;

	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);

	if (reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_EMPTY)
		return -EINVAL;

	ret = kbase_user_buf_pin_pages(kctx, reg);

	if (ret)
		goto pin_pages_fail;

	ret = kbase_user_buf_dma_map_pages(kctx, reg);

	if (ret)
		goto dma_map_pages_fail;

	ret = kbase_user_buf_map(kctx, reg);

	if (!ret)
		reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_GPU_MAPPED;
	else
		goto user_buf_map_fail;

	return ret;

user_buf_map_fail:
	kbase_user_buf_dma_unmap_pages(kctx, reg);
dma_map_pages_fail:
	/* The user buffer could already have been previously pinned before
	 * entering this function, and hence there could potentially be CPU
	 * mappings of it.
	 */
	kbase_mem_shrink_cpu_mapping(kctx, reg, 0, reg->gpu_alloc->nents);
	kbase_user_buf_unpin_pages(reg->gpu_alloc);
pin_pages_fail:
	return ret;
}

void kbase_user_buf_from_pinned_to_empty(struct kbase_context *kctx, struct kbase_va_region *reg)
{
	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	if (WARN_ON(reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_PINNED))
		return;
	kbase_user_buf_unpin_pages(reg->gpu_alloc);
	reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_EMPTY;
}

int kbase_user_buf_from_pinned_to_gpu_mapped(struct kbase_context *kctx,
					     struct kbase_va_region *reg)
{
	int ret;

	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	lockdep_assert_held(&kctx->reg_lock);

	if (reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_PINNED)
		return -EINVAL;

	ret = kbase_user_buf_dma_map_pages(kctx, reg);

	if (ret)
		goto dma_map_pages_fail;

	ret = kbase_user_buf_map(kctx, reg);

	if (!ret)
		reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_GPU_MAPPED;
	else
		goto user_buf_map_fail;

	return ret;

user_buf_map_fail:
	kbase_user_buf_dma_unmap_pages(kctx, reg);
dma_map_pages_fail:
	return ret;
}

void kbase_user_buf_from_dma_mapped_to_pinned(struct kbase_context *kctx,
					      struct kbase_va_region *reg)
{
	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	if (WARN_ON(reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_DMA_MAPPED))
		return;
#if !MALI_USE_CSF
	kbase_mem_shrink_cpu_mapping(kctx, reg, 0, reg->gpu_alloc->nents);
#endif
	kbase_user_buf_dma_unmap_pages(kctx, reg);

	reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_PINNED;
}

void kbase_user_buf_from_dma_mapped_to_empty(struct kbase_context *kctx,
					     struct kbase_va_region *reg)
{
	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	if (WARN_ON(reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_DMA_MAPPED))
		return;
#if !MALI_USE_CSF
	kbase_mem_shrink_cpu_mapping(kctx, reg, 0, reg->gpu_alloc->nents);
#endif
	kbase_user_buf_dma_unmap_pages(kctx, reg);

	/* Termination code path: fall through to next state transition. */
	reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_PINNED;
	kbase_user_buf_from_pinned_to_empty(kctx, reg);
}

int kbase_user_buf_from_dma_mapped_to_gpu_mapped(struct kbase_context *kctx,
						 struct kbase_va_region *reg)
{
	int ret;

	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);

	if (reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_DMA_MAPPED)
		return -EINVAL;

	ret = kbase_user_buf_map(kctx, reg);

	if (!ret)
		reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_GPU_MAPPED;

	return ret;
}

void kbase_user_buf_from_gpu_mapped_to_pinned(struct kbase_context *kctx,
					      struct kbase_va_region *reg)
{
	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	if (WARN_ON(reg->gpu_alloc->imported.user_buf.state != KBASE_USER_BUF_STATE_GPU_MAPPED))
		return;
	kbase_user_buf_unmap(kctx, reg);
	kbase_user_buf_dma_unmap_pages(kctx, reg);
	reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_PINNED;
}

void kbase_user_buf_from_gpu_mapped_to_empty(struct kbase_context *kctx,
					     struct kbase_va_region *reg)
{
	dev_dbg(kctx->kbdev->dev, "%s %pK in kctx %pK\n", __func__, (void *)reg, (void *)kctx);
	kbase_user_buf_unmap(kctx, reg);

	/* Termination code path: fall through to next state transition. */
	reg->gpu_alloc->imported.user_buf.state = KBASE_USER_BUF_STATE_DMA_MAPPED;
	kbase_user_buf_from_dma_mapped_to_empty(kctx, reg);
}
