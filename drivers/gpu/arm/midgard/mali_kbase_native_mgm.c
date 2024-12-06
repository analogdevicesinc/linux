// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2019-2024 ARM Limited. All rights reserved.
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

#include <linux/gfp.h>
#include <linux/mm.h>
#include <linux/memory_group_manager.h>

#include <mali_kbase.h>
#include <mali_kbase_native_mgm.h>

#include <thirdparty/mm.h>

/**
 * kbase_native_mgm_alloc - Native physical memory allocation method
 *
 * @mgm_dev:  The memory group manager the request is being made through.
 * @group_id: A physical memory group ID, which must be valid but is not used.
 *            Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @gfp_mask: Bitmask of Get Free Page flags affecting allocator behavior.
 * @order:    Page order for physical page size.
 *            order = 0 refers to small pages
 *            order != 0 refers to 2 MB pages, so
 *            order = 9 (when small page size is 4KB,  2^9 *  4KB = 2 MB)
 *            order = 7 (when small page size is 16KB, 2^7 * 16KB = 2 MB)
 *            order = 5 (when small page size is 64KB, 2^5 * 64KB = 2 MB)
 *
 * Delegates all memory allocation requests to the kernel's alloc_pages
 * function.
 *
 * Return: Pointer to allocated page, or NULL if allocation failed.
 */
static struct page *kbase_native_mgm_alloc(struct memory_group_manager_device *mgm_dev,
					   unsigned int group_id, gfp_t gfp_mask,
					   unsigned int order)
{
	/*
	 * Check that the base and the mgm defines, from separate header files,
	 * for the max number of memory groups are compatible.
	 */
	BUILD_BUG_ON(BASE_MEM_GROUP_COUNT != MEMORY_GROUP_MANAGER_NR_GROUPS);
	/*
	 * Check that the mask used for storing the memory group ID is big
	 * enough for the largest possible memory group ID.
	 */
	BUILD_BUG_ON((BASEP_CONTEXT_MMU_GROUP_ID_MASK >> BASEP_CONTEXT_MMU_GROUP_ID_SHIFT) <
		     (BASE_MEM_GROUP_COUNT - 1));

	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);

	return alloc_pages(gfp_mask, order);
}

/**
 * kbase_native_mgm_free - Native physical memory freeing method
 *
 * @mgm_dev:  The memory group manager the request is being made through.
 * @group_id: A physical memory group ID, which must be valid but is not used.
 *            Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @page:     Address of the struct associated with a page of physical
 *            memory that was allocated by calling kbase_native_mgm_alloc
 *            with the same argument values.
 * @order:    Page order for physical page size (order=0 means 4 KiB,
 *            order=9 means 2 MiB).
 *
 * Delegates all memory freeing requests to the kernel's __free_pages function.
 */
static void kbase_native_mgm_free(struct memory_group_manager_device *mgm_dev,
				  unsigned int group_id, struct page *page, unsigned int order)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);

	__free_pages(page, order);
}

/**
 * kbase_native_mgm_vmf_insert_pfn_prot - Native method to map a page on the CPU
 *
 * @mgm_dev:  The memory group manager the request is being made through.
 * @group_id: A physical memory group ID, which must be valid but is not used.
 *            Its valid range is 0 .. MEMORY_GROUP_MANAGER_NR_GROUPS-1.
 * @vma:      The virtual memory area to insert the page into.
 * @addr:     An address contained in @vma to assign to the inserted page.
 * @pfn:      The kernel Page Frame Number to insert at @addr in @vma.
 * @pgprot:   Protection flags for the inserted page.
 *
 * Called from a CPU virtual memory page fault handler. Delegates all memory
 * mapping requests to the kernel's vmf_insert_pfn_prot function.
 *
 * Return: Type of fault that occurred or VM_FAULT_NOPAGE if the page table
 *         entry was successfully installed.
 */
static vm_fault_t kbase_native_mgm_vmf_insert_pfn_prot(struct memory_group_manager_device *mgm_dev,
						       unsigned int group_id,
						       struct vm_area_struct *vma,
						       unsigned long addr, unsigned long pfn,
						       pgprot_t pgprot)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);

	return vmf_insert_pfn_prot(vma, addr, pfn, pgprot);
}

static u64 kbase_native_mgm_update_gpu_pte(struct memory_group_manager_device *mgm_dev,
					   unsigned int group_id, unsigned int pbha_id,
					   unsigned int pte_flags, int mmu_level, u64 pte)
{
	if (WARN_ON(group_id >= MEMORY_GROUP_MANAGER_NR_GROUPS))
		return pte;

	if ((pte_flags & BIT(MMA_VIOLATION)) && pbha_id) {
		pr_warn_once("MMA violation! Applying PBHA override workaround to PTE\n");
		pte |= ((u64)pbha_id << PTE_PBHA_SHIFT) & PTE_PBHA_MASK;
	}

	/* Address could be translated into a different bus address here */
	pte |= ((u64)1 << PTE_RES_BIT_MULTI_AS_SHIFT);

	return pte;
}

static u64 kbase_native_mgm_pte_to_original_pte(struct memory_group_manager_device *mgm_dev,
						unsigned int group_id, int mmu_level, u64 pte)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(group_id);
	CSTD_UNUSED(mmu_level);

	/* Undo the group ID modification */
	pte &= ~PTE_PBHA_MASK;
	/* Undo the bit set */
	pte &= ~((u64)1 << PTE_RES_BIT_MULTI_AS_SHIFT);

	return pte;
}

static bool kbase_native_mgm_get_import_memory_cached_access_permitted(
	struct memory_group_manager_device *mgm_dev,
	struct memory_group_manager_import_data *import_data)
{
	CSTD_UNUSED(mgm_dev);
	CSTD_UNUSED(import_data);

	return true;
}

struct memory_group_manager_device kbase_native_mgm_dev = {
	.ops = { .mgm_alloc_page = kbase_native_mgm_alloc,
		 .mgm_free_page = kbase_native_mgm_free,
		 .mgm_get_import_memory_id = NULL,
		 .mgm_vmf_insert_pfn_prot = kbase_native_mgm_vmf_insert_pfn_prot,
		 .mgm_update_gpu_pte = kbase_native_mgm_update_gpu_pte,
		 .mgm_pte_to_original_pte = kbase_native_mgm_pte_to_original_pte,
		 .mgm_get_import_memory_cached_access_permitted =
			 kbase_native_mgm_get_import_memory_cached_access_permitted },
	.data = NULL
};
