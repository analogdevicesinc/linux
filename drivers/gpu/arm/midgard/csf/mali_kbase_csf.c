// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2018-2024 ARM Limited. All rights reserved.
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

#include <mali_kbase.h>
#include <mali_kbase_caps.h>
#include <gpu/mali_kbase_gpu_fault.h>
#include <mali_kbase_reset_gpu.h>
#include "mali_kbase_csf.h"
#include "backend/gpu/mali_kbase_pm_internal.h"
#include <linux/export.h>
#include <linux/priority_control_manager.h>
#include <linux/shmem_fs.h>
#include <csf/mali_kbase_csf_cpu_queue.h>
#include <csf/mali_kbase_csf_registers.h>
#include <csf/mali_kbase_csf_firmware_log.h>
#include "mali_kbase_csf_tiler_heap.h"
#include <mmu/mali_kbase_mmu.h>
#include "mali_kbase_csf_timeout.h"
#include <csf/ipa_control/mali_kbase_csf_ipa_control.h>
#include <mali_kbase_hwaccess_time.h>
#include "mali_kbase_csf_event.h"
#include "mali_kbase_ctx_sched.h"
#include <tl/mali_kbase_tracepoints.h>
#include "mali_kbase_csf_mcu_shared_reg.h"
#include <linux/version_compat_defs.h>
#include <mali_kbase_io.h>

#define CS_REQ_EXCEPTION_MASK (CS_REQ_FAULT_MASK | CS_REQ_FATAL_MASK)
#define CS_ACK_EXCEPTION_MASK (CS_ACK_FAULT_MASK | CS_ACK_FATAL_MASK)

#define CS_RING_BUFFER_MAX_SIZE ((uint32_t)(1 << 31)) /* 2GiB */
#define CS_RING_BUFFER_MIN_SIZE ((uint32_t)4096)

/* 0.2 second assuming 600 MHz GPU clock, which is double of iterator disabling timeout */
#define MAX_PROGRESS_TIMEOUT_EVENT_DELAY ((u32)120000000)

#define PROTM_ALLOC_MAX_RETRIES ((u8)5)

const u8 kbasep_csf_queue_group_priority_to_relative[BASE_QUEUE_GROUP_PRIORITY_COUNT] = {
	KBASE_QUEUE_GROUP_PRIORITY_HIGH, KBASE_QUEUE_GROUP_PRIORITY_MEDIUM,
	KBASE_QUEUE_GROUP_PRIORITY_LOW, KBASE_QUEUE_GROUP_PRIORITY_REALTIME
};
const u8 kbasep_csf_relative_to_queue_group_priority[KBASE_QUEUE_GROUP_PRIORITY_COUNT] = {
	BASE_QUEUE_GROUP_PRIORITY_REALTIME, BASE_QUEUE_GROUP_PRIORITY_HIGH,
	BASE_QUEUE_GROUP_PRIORITY_MEDIUM, BASE_QUEUE_GROUP_PRIORITY_LOW
};

static bool dump_oops_in_dmesg;
module_param(dump_oops_in_dmesg, bool, 0644);
MODULE_PARM_DESC(dump_oops_in_dmesg, "On FIRMWARE_INTERNAL_ERROR dump the OOPs message to dmesg.");

static bool dump_ktrace_in_dmesg;
module_param(dump_ktrace_in_dmesg, bool, 0644);
MODULE_PARM_DESC(dump_ktrace_in_dmesg,
		 "On FIRMWARE_INTERNAL_ERROR dump the Ktrace message to dmesg.");

/*
 * struct irq_idle_and_protm_track - Object that tracks the idle and protected mode
 *                                   request information in an interrupt case across
 *                                   groups.
 *
 * @protm_grp: Possibly schedulable group that requested protected mode in the interrupt.
 *             If NULL, no such case observed in the tracked interrupt case.
 * @idle_seq:  The highest priority group that notified idle. If no such instance in the
 *             interrupt case, marked with the largest field value: U32_MAX.
 * @idle_slot: The slot number if @p idle_seq is valid in the given tracking case.
 */
struct irq_idle_and_protm_track {
	struct kbase_queue_group *protm_grp;
	u32 idle_seq;
	s8 idle_slot;
};

/**
 * kbasep_ctx_user_reg_page_mapping_term() - Terminate resources for USER Register Page.
 *
 * @kctx:   Pointer to the kbase context
 */
static void kbasep_ctx_user_reg_page_mapping_term(struct kbase_context *kctx)
{
	struct kbase_device *kbdev = kctx->kbdev;

	if (unlikely(kctx->csf.user_reg.vma))
		dev_err(kbdev->dev, "VMA for USER Register page exist on termination of ctx %d_%d",
			kctx->tgid, kctx->id);
	if (WARN_ON_ONCE(!list_empty(&kctx->csf.user_reg.link)))
		list_del_init(&kctx->csf.user_reg.link);
}

/**
 * kbasep_ctx_user_reg_page_mapping_init() - Initialize resources for USER Register Page.
 * @kctx:   Pointer to the kbase context
 *
 * This function must be called only when a kbase context is instantiated.
 *
 * @return: 0 on success.
 */
static int kbasep_ctx_user_reg_page_mapping_init(struct kbase_context *kctx)
{
	INIT_LIST_HEAD(&kctx->csf.user_reg.link);

	return 0;
}

static void put_user_pages_mmap_handle(struct kbase_context *kctx, struct kbase_queue *queue)
{
	unsigned long cookie_nr;

	lockdep_assert_held(&kctx->csf.lock);

	if (queue->handle == BASEP_MEM_INVALID_HANDLE)
		return;

	cookie_nr = PFN_DOWN(queue->handle - BASEP_MEM_CSF_USER_IO_PAGES_HANDLE);

	if (!WARN_ON(kctx->csf.user_pages_info[cookie_nr] != queue)) {
		/* free up cookie */
		kctx->csf.user_pages_info[cookie_nr] = NULL;
		bitmap_set(kctx->csf.cookies, cookie_nr, 1);
	}

	queue->handle = BASEP_MEM_INVALID_HANDLE;
}

/* Reserve a cookie, to be returned as a handle to userspace for creating
 * the CPU mapping of the pair of input/output pages and Hw doorbell page.
 * Will return 0 in case of success otherwise negative on failure.
 */
static int get_user_pages_mmap_handle(struct kbase_context *kctx, struct kbase_queue *queue)
{
	unsigned long cookie, cookie_nr;

	lockdep_assert_held(&kctx->csf.lock);

	if (bitmap_empty(kctx->csf.cookies, KBASE_CSF_NUM_USER_IO_PAGES_HANDLE)) {
		dev_err(kctx->kbdev->dev, "No csf cookies available for allocation!");
		return -ENOMEM;
	}

	/* allocate a cookie */
	cookie_nr = find_first_bit(kctx->csf.cookies, KBASE_CSF_NUM_USER_IO_PAGES_HANDLE);
	if (kctx->csf.user_pages_info[cookie_nr]) {
		dev_err(kctx->kbdev->dev, "Inconsistent state of csf cookies!");
		return -EINVAL;
	}
	kctx->csf.user_pages_info[cookie_nr] = queue;
	bitmap_clear(kctx->csf.cookies, cookie_nr, 1);

	/* relocate to correct base */
	cookie = cookie_nr + PFN_DOWN(BASEP_MEM_CSF_USER_IO_PAGES_HANDLE);
	cookie <<= PAGE_SHIFT;

	queue->handle = (u64)cookie;

	return 0;
}

static void init_user_io_pages(struct kbase_queue *queue)
{
	u64 *input_addr = queue->user_io_addr;
	u64 *output_addr64 = queue->user_io_addr + PAGE_SIZE / sizeof(u64);
	u32 *output_addr32 = (u32 *)(queue->user_io_addr + PAGE_SIZE / sizeof(u64));

	/*
	 * CS_INSERT and CS_EXTRACT registers contain 64-bit memory addresses which
	 * should be accessed atomically. Here we update them 32-bits at a time, but
	 * as this is initialisation code, non-atomic accesses are safe.
	 */
	input_addr[CS_INSERT_LO / sizeof(*input_addr)] = 0;
	input_addr[CS_EXTRACT_INIT_LO / sizeof(*input_addr)] = 0;
	output_addr64[CS_EXTRACT_LO / sizeof(*output_addr64)] = 0;
	output_addr32[CS_ACTIVE / sizeof(*output_addr32)] = 0;
}

/**
 * kernel_free_user_io_pages() - Unmap and free the CS_USER IO pages
 *
 * @kctx: Pointer to the kbase context.
 * @phys: Pointer to the physical pages to unmap and free.
 * @user_io_addr: Pointer to the CPU VA mapping of CS_USER IO pages.
 *
 * This function will unmap CS_USER IO pages from the CPU, and free
 * the physical pages.
 * Finally, Kbase page counter is updated to report the correct memory usage.
 */
static void kernel_free_user_io_pages(struct kbase_context *kctx, struct tagged_addr *phys,
				      u64 *user_io_addr)
{
	/* Early return in case IO pages were already freed. */
	if (WARN_ON(user_io_addr == NULL))
		return;

	kbase_gpu_vm_lock(kctx);

	vunmap(user_io_addr);

	WARN_ON(atomic_read(&kctx->permanent_mapped_pages) < KBASEP_NUM_CS_USER_IO_PAGES);
	atomic_sub(KBASEP_NUM_CS_USER_IO_PAGES, &kctx->permanent_mapped_pages);

	kbase_gpu_vm_unlock(kctx);

	kbase_mem_pool_free_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_IO],
				  KBASEP_NUM_CS_USER_IO_PAGES, phys, true, false);
	kbase_process_page_usage_dec(kctx, KBASEP_NUM_CS_USER_IO_PAGES);
}

/**
 * kernel_alloc_user_io_pages() - Allocate and map the CS_USER IO pages
 *
 * @kctx: Pointer to the kbase context.
 * @phys: Output pointer to the physical pages allocated by the function.
 * @user_io_addr: Output pointer to the CPU VA mapping of CS_USER IO pages.
 * @user_io_gpu_va: Output pointer to the GPU VA mapping of CS_USER IO pages.
 *                  Always set to 0 as initial VA, which indicates it is not mapped.
 *                  The actual mapping is done on CSG register programing following
 *                  a CSF scheduler action.
 *
 * This function will allocate CS_USER IO pages and map them to the CPU.
 * Finally, Kbase page counter is updated to report the correct memory usage.
 *
 * Return: 0 on success, negative error code otherwise.
 */
static int kernel_alloc_user_io_pages(struct kbase_context *kctx, struct tagged_addr *phys,
				      u64 **user_io_addr, u64 *user_io_gpu_va)
{
	struct page *page_list[2];
	pgprot_t cpu_map_prot;
	unsigned long flags;
	u64 *cpu_addr;
	int ret = 0;
	size_t i;

	if (kbase_mem_pool_alloc_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_IO],
				       KBASEP_NUM_CS_USER_IO_PAGES, phys, false,
				       kctx->task) != KBASEP_NUM_CS_USER_IO_PAGES) {
		ret = -ENOMEM;
		goto exit;
	}

	kbase_gpu_vm_lock(kctx);

	if (ARRAY_SIZE(page_list) > (KBASE_PERMANENTLY_MAPPED_MEM_LIMIT_PAGES -
				     (unsigned int)atomic_read(&kctx->permanent_mapped_pages))) {
		ret = -ENOMEM;
		goto unlock;
	}

	/* The pages are mapped to Userspace also, so use the same mapping
	 * attributes as used inside the CPU page fault handler.
	 */
	if (kctx->kbdev->system_coherency == COHERENCY_NONE)
		cpu_map_prot = pgprot_writecombine(PAGE_KERNEL);
	else
		cpu_map_prot = PAGE_KERNEL;

	for (i = 0; i < ARRAY_SIZE(page_list); i++)
		page_list[i] = as_page(phys[i]);

	cpu_addr = vmap(page_list, ARRAY_SIZE(page_list), VM_MAP, cpu_map_prot);

	if (!cpu_addr)
		ret = -ENOMEM;
	else
		atomic_add(ARRAY_SIZE(page_list), &kctx->permanent_mapped_pages);

	kbase_csf_scheduler_spin_lock(kctx->kbdev, &flags);
	*user_io_addr = cpu_addr;

	/* user_io_gpu_va is only mapped when scheduler decides to put the queue
	 * on slot at runtime. Initialize it to 0, signalling no mapping.
	 */
	*user_io_gpu_va = 0;
	kbase_csf_scheduler_spin_unlock(kctx->kbdev, flags);

unlock:
	kbase_gpu_vm_unlock(kctx);

	if (ret) {
		kbase_mem_pool_free_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_IO],
					  KBASEP_NUM_CS_USER_IO_PAGES, phys, false, false);
	} else
		kbase_process_page_usage_inc(kctx, KBASEP_NUM_CS_USER_IO_PAGES);

exit:
	if (ret) {
		/* Marking both the phys to zero for indicating there is no phys allocated */
		phys[0].tagged_addr = 0;
		phys[1].tagged_addr = 0;
	}

	return ret;
}

static void term_queue_group(struct kbase_queue_group *group);
static void get_queue(struct kbase_queue *queue);
static bool release_queue(struct kbase_queue *queue);

void kbase_csf_free_command_stream_user_pages(struct kbase_context *kctx, struct kbase_queue *queue)
{
	/* Free the resources, if allocated phys for this queue.
	 * This condition is only true when mali_kbase_supports_csg_cs_user_page_allocation()
	 * is disable.
	 */
	if (!mali_kbase_supports_csg_cs_user_page_allocation(kctx->api_version)) {
		kernel_free_user_io_pages(kctx, queue->phys, queue->user_io_addr);
		queue->user_io_addr = NULL;
	}

	/* The user_io_gpu_va should have been unmapped inside the scheduler */
	WARN_ONCE(queue->user_io_gpu_va, "Userio pages appears still have mapping");

	/* If the queue has already been terminated by userspace
	 * then the ref count for queue object will drop to 0 here.
	 */
	release_queue(queue);
}
KBASE_EXPORT_TEST_API(kbase_csf_free_command_stream_user_pages);

int kbase_csf_alloc_command_stream_user_pages(struct kbase_context *kctx, struct kbase_queue *queue)
{
	struct kbase_device *kbdev = kctx->kbdev;
	int ret = 0;

	lockdep_assert_held(&kctx->csf.lock);

	if (!mali_kbase_supports_csg_cs_user_page_allocation(kctx->api_version)) {
		ret = kernel_alloc_user_io_pages(kctx, queue->phys, &queue->user_io_addr,
						 &queue->user_io_gpu_va);
		if (ret)
			return ret;

	} else {
		unsigned long flags;

		kbase_gpu_vm_lock(kctx);
		kbase_csf_scheduler_spin_lock(kctx->kbdev, &flags);

		if (unlikely(!queue->group)) {
			kbase_csf_scheduler_spin_unlock(kctx->kbdev, flags);
			kbase_gpu_vm_unlock(kctx);

			return -EINVAL;
		}

		queue->user_io_addr =
			queue->group->user_io_addr +
			queue->csi_index * CS_USER_INPUT_BLOCK_SIZE / sizeof(*queue->user_io_addr);

		kbase_csf_scheduler_spin_unlock(kctx->kbdev, flags);
		kbase_gpu_vm_unlock(kctx);
	}

	init_user_io_pages(queue);

	mutex_lock(&kbdev->csf.reg_lock);
	if (kbdev->csf.db_file_offsets > (U32_MAX - BASEP_QUEUE_NR_MMAP_USER_PAGES + 1))
		kbdev->csf.db_file_offsets = 0;

	queue->db_file_offset = kbdev->csf.db_file_offsets;
	kbdev->csf.db_file_offsets += BASEP_QUEUE_NR_MMAP_USER_PAGES;
	WARN(kbase_refcount_read(&queue->refcount) != 1,
	     "Incorrect refcounting for queue object\n");
	/* This is the second reference taken on the queue object and
	 * would be dropped only when the IO mapping is removed either
	 * explicitly by userspace or implicitly by kernel on process exit.
	 */
	get_queue(queue);
	queue->bind_state = KBASE_CSF_QUEUE_BOUND;
	mutex_unlock(&kbdev->csf.reg_lock);

	return 0;
}
KBASE_EXPORT_TEST_API(kbase_csf_alloc_command_stream_user_pages);

static struct kbase_queue_group *find_queue_group(struct kbase_context *kctx, u8 group_handle)
{
	uint index = group_handle;

	lockdep_assert_held(&kctx->csf.lock);

	if (index < MAX_QUEUE_GROUP_NUM && kctx->csf.queue_groups[index]) {
		if (WARN_ON(kctx->csf.queue_groups[index]->handle != index))
			return NULL;
		return kctx->csf.queue_groups[index];
	}

	return NULL;
}

struct kbase_queue_group *kbase_csf_find_queue_group(struct kbase_context *kctx, u8 group_handle)
{
	return find_queue_group(kctx, group_handle);
}
KBASE_EXPORT_TEST_API(kbase_csf_find_queue_group);

int kbase_csf_queue_group_handle_is_valid(struct kbase_context *kctx, u8 group_handle)
{
	struct kbase_queue_group *group;

	mutex_lock(&kctx->csf.lock);
	group = find_queue_group(kctx, group_handle);
	mutex_unlock(&kctx->csf.lock);

	return group ? 0 : -EINVAL;
}

static struct kbase_queue *find_queue(struct kbase_context *kctx, u64 base_addr)
{
	struct kbase_queue *queue;

	lockdep_assert_held(&kctx->csf.lock);

	list_for_each_entry(queue, &kctx->csf.queue_list, link) {
		if (base_addr == queue->base_addr)
			return queue;
	}

	return NULL;
}

static void get_queue(struct kbase_queue *queue)
{
	WARN_ON(!kbase_refcount_inc_not_zero(&queue->refcount));
}

/**
 * release_queue() - Release a reference to a GPU queue
 *
 * @queue: The queue to release.
 *
 * Return: true if the queue has been released.
 *
 * The queue will be released when its reference count reaches zero.
 */
static bool release_queue(struct kbase_queue *queue)
{
	lockdep_assert_held(&queue->kctx->csf.lock);
	if (kbase_refcount_dec_and_test(&queue->refcount)) {
		/* The queue can't still be on the per context list. */
		WARN_ON(!list_empty(&queue->link));
		WARN_ON(queue->group);
		dev_dbg(queue->kctx->kbdev->dev,
			"Remove any pending command queue fatal from ctx %d_%d", queue->kctx->tgid,
			queue->kctx->id);

		/* After this the Userspace would be able to free the
		 * memory for GPU queue. In case the Userspace missed
		 * terminating the queue, the cleanup will happen on
		 * context termination where tear down of region tracker
		 * would free up the GPU queue memory.
		 */
		kbase_gpu_vm_lock(queue->kctx);
		kbase_va_region_no_user_free_dec(queue->queue_reg);
		kbase_gpu_vm_unlock(queue->kctx);

		kfree(queue);

		return true;
	}

	return false;
}

static void oom_event_worker(struct work_struct *data);
static void cs_error_worker(struct work_struct *data);

/* Between reg and reg_ex, one and only one must be null */
static int csf_queue_register_internal(struct kbase_context *kctx,
				       struct kbase_ioctl_cs_queue_register *reg,
				       struct kbase_ioctl_cs_queue_register_ex *reg_ex)
{
	struct kbase_queue *queue;
	int ret = 0;
	struct kbase_va_region *region;
	u64 queue_addr;
	size_t queue_size;

	/* Only one pointer expected, otherwise coding error */
	if ((reg == NULL && reg_ex == NULL) || (reg && reg_ex)) {
		dev_dbg(kctx->kbdev->dev, "Error, one and only one param-ptr expected!");
		return -EINVAL;
	}

	/* struct kbase_ioctl_cs_queue_register_ex contains a full
	 * struct kbase_ioctl_cs_queue_register at the start address. So
	 * the pointer can be safely cast to pointing to a
	 * kbase_ioctl_cs_queue_register object.
	 */
	if (reg_ex)
		reg = (struct kbase_ioctl_cs_queue_register *)reg_ex;

	/* Validate the queue priority */
	if (reg->priority > BASE_QUEUE_MAX_PRIORITY)
		return -EINVAL;

	queue_addr = reg->buffer_gpu_addr;
	queue_size = reg->buffer_size >> PAGE_SHIFT;

	mutex_lock(&kctx->csf.lock);

	/* Check if queue is already registered */
	if (find_queue(kctx, queue_addr) != NULL) {
		ret = -EINVAL;
		goto out;
	}

	/* Check if the queue address is valid */
	kbase_gpu_vm_lock(kctx);
	region = kbase_region_tracker_find_region_enclosing_address(kctx, queue_addr);

	if (kbase_is_region_invalid_or_free(region) || kbase_is_region_shrinkable(region) ||
	    region->gpu_alloc->type != KBASE_MEM_TYPE_NATIVE) {
		ret = -ENOENT;
		goto out_unlock_vm;
	}

	if (queue_size > (region->nr_pages - ((queue_addr >> PAGE_SHIFT) - region->start_pfn))) {
		ret = -EINVAL;
		goto out_unlock_vm;
	}

	/* Check address validity on cs_trace buffer etc. Don't care
	 * if not enabled (i.e. when size is 0).
	 */
	if (reg_ex && reg_ex->ex_buffer_size) {
		size_t buf_pages = (reg_ex->ex_buffer_size + (1UL << PAGE_SHIFT) - 1) >> PAGE_SHIFT;
		struct kbase_va_region *region_ex =
			kbase_region_tracker_find_region_enclosing_address(kctx,
									   reg_ex->ex_buffer_base);

		if (kbase_is_region_invalid_or_free(region_ex)) {
			ret = -ENOENT;
			goto out_unlock_vm;
		}

		if (buf_pages > (region_ex->nr_pages -
				 ((reg_ex->ex_buffer_base >> PAGE_SHIFT) - region_ex->start_pfn))) {
			ret = -EINVAL;
			goto out_unlock_vm;
		}

		region_ex = kbase_region_tracker_find_region_enclosing_address(
			kctx, reg_ex->ex_offset_var_addr);
		if (kbase_is_region_invalid_or_free(region_ex)) {
			ret = -ENOENT;
			goto out_unlock_vm;
		}
	}

	queue = kzalloc(sizeof(struct kbase_queue), GFP_KERNEL);

	if (!queue) {
		ret = -ENOMEM;
		goto out_unlock_vm;
	}

	queue->kctx = kctx;
	queue->base_addr = queue_addr;

	queue->queue_reg = region;
	kbase_va_region_no_user_free_inc(region);

	queue->size = (queue_size << PAGE_SHIFT);
	queue->csi_index = KBASEP_IF_NR_INVALID;

	queue->priority = reg->priority;
	/* Default to a safe value, this would be updated on binding */
	queue->group_priority = KBASE_QUEUE_GROUP_PRIORITY_LOW;
	kbase_refcount_set(&queue->refcount, 1);

	queue->bind_state = KBASE_CSF_QUEUE_UNBOUND;
	queue->handle = BASEP_MEM_INVALID_HANDLE;
	queue->doorbell_nr = KBASEP_USER_DB_NR_INVALID;

	queue->blocked_reason = CS_STATUS_BLOCKED_REASON_REASON_UNBLOCKED;

	queue->clear_faults = true;

	INIT_LIST_HEAD(&queue->link);
	atomic_set(&queue->pending_kick, 0);
	INIT_LIST_HEAD(&queue->pending_kick_link);
	INIT_WORK(&queue->oom_event_work, oom_event_worker);
	INIT_WORK(&queue->cs_error_work, cs_error_worker);
	list_add(&queue->link, &kctx->csf.queue_list);

	region->user_data = queue;

	/* Initialize the cs_trace configuration parameters, When buffer_size
	 * is 0, trace is disabled. Here we only update the fields when
	 * enabled, otherwise leave them as default zeros.
	 */
	if (reg_ex && reg_ex->ex_buffer_size) {
		u32 cfg = CS_INSTR_CONFIG_EVENT_SIZE_SET(0U, reg_ex->ex_event_size);
		cfg = CS_INSTR_CONFIG_EVENT_STATE_SET(cfg, reg_ex->ex_event_state);

		queue->trace_cfg = cfg;
		queue->trace_buffer_size = reg_ex->ex_buffer_size;
		queue->trace_buffer_base = reg_ex->ex_buffer_base;
		queue->trace_offset_ptr = reg_ex->ex_offset_var_addr;
	}

out_unlock_vm:
	kbase_gpu_vm_unlock(kctx);
out:
	mutex_unlock(&kctx->csf.lock);

	return ret;
}

int kbase_csf_queue_register(struct kbase_context *kctx, struct kbase_ioctl_cs_queue_register *reg)
{
	/* Validate the ring buffer configuration parameters */
	if (reg->buffer_size < CS_RING_BUFFER_MIN_SIZE ||
	    reg->buffer_size > CS_RING_BUFFER_MAX_SIZE ||
	    reg->buffer_size & (reg->buffer_size - 1) || !reg->buffer_gpu_addr ||
	    reg->buffer_gpu_addr & ~PAGE_MASK)
		return -EINVAL;

	return csf_queue_register_internal(kctx, reg, NULL);
}

int kbase_csf_queue_register_ex(struct kbase_context *kctx,
				struct kbase_ioctl_cs_queue_register_ex *reg)
{
	struct kbase_csf_global_iface const *const iface = &kctx->kbdev->csf.global_iface;
	u32 const glb_version = iface->version;
	u32 instr = iface->instr_features;
	u8 max_size = GLB_INSTR_FEATURES_EVENT_SIZE_MAX_GET(instr);
	const u8 event_size = reg->ex_event_size;
	u64 min_buf_size;

	/* If cs_trace_command not supported, the call fails */
	if (glb_version < kbase_csf_interface_version(1, 1, 0))
		return -EPERM;

	/* Sanity check to avoid shift-out-of-bounds */
	if (event_size >= 32)
		return -EINVAL;

	min_buf_size = ((u64)1 << event_size) * GLB_INSTR_FEATURES_OFFSET_UPDATE_RATE_GET(instr);
	if (min_buf_size > UINT32_MAX)
		return -EINVAL;

	/* Validate the ring buffer configuration parameters */
	if (reg->buffer_size < CS_RING_BUFFER_MIN_SIZE ||
	    reg->buffer_size > CS_RING_BUFFER_MAX_SIZE ||
	    reg->buffer_size & (reg->buffer_size - 1) || !reg->buffer_gpu_addr ||
	    reg->buffer_gpu_addr & ~PAGE_MASK)
		return -EINVAL;

	/* Validate the cs_trace configuration parameters */
	if (reg->ex_buffer_size &&
	    ((event_size > max_size) || (reg->ex_buffer_size & (reg->ex_buffer_size - 1)) ||
	     (reg->ex_buffer_size < (u32)min_buf_size)))
		return -EINVAL;

	return csf_queue_register_internal(kctx, NULL, reg);
}

static void unbind_queue(struct kbase_context *kctx, struct kbase_queue *queue);

static void wait_pending_queue_kick(struct kbase_queue *queue)
{
	struct kbase_context *const kctx = queue->kctx;

	/* Drain a pending queue kick if any. It should no longer be
	 * possible to issue further queue kicks at this point: either the
	 * queue has been unbound, or the context is being terminated.
	 *
	 * Signal kbase_csf_scheduler_kthread() to allow for the
	 * eventual completion of the current iteration. Once it's done the
	 * event_wait wait queue shall be signalled.
	 */
	complete(&kctx->kbdev->csf.scheduler.kthread_signal);
	wait_event(kctx->kbdev->csf.event_wait, atomic_read(&queue->pending_kick) == 0);
}

void kbase_csf_queue_terminate(struct kbase_context *kctx,
			       struct kbase_ioctl_cs_queue_terminate *term)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_queue *queue;
	int err;
	bool reset_prevented = false;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating queue (buffer_addr=0x%.16llx), attempting to terminate regardless",
			term->buffer_gpu_addr);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);
	queue = find_queue(kctx, term->buffer_gpu_addr);

	if (queue) {
		/* As the GPU queue has been terminated by the
		 * user space, undo the actions that were performed when the
		 * queue was registered i.e. remove the queue from the per
		 * context list & release the initial reference. The subsequent
		 * lookups for the queue in find_queue() would fail.
		 */
		list_del_init(&queue->link);

		/* Stop the CSI to which queue was bound */
		unbind_queue(kctx, queue);

		kbase_gpu_vm_lock(kctx);
		if (!WARN_ON(!queue->queue_reg))
			queue->queue_reg->user_data = NULL;
		kbase_gpu_vm_unlock(kctx);

		mutex_unlock(&kctx->csf.lock);
		/* The GPU reset can be allowed now as the queue has been unbound. */
		if (reset_prevented) {
			kbase_reset_gpu_allow(kbdev);
			reset_prevented = false;
		}
		wait_pending_queue_kick(queue);
		/* The work items can be cancelled as Userspace is terminating the queue */
		cancel_work_sync(&queue->oom_event_work);
		cancel_work_sync(&queue->cs_error_work);
		mutex_lock(&kctx->csf.lock);

		release_queue(queue);
	}

	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);
}

int kbase_csf_queue_bind(struct kbase_context *kctx, union kbase_ioctl_cs_queue_bind *bind)
{
	struct kbase_queue *queue;
	struct kbase_queue_group *group;
	u8 max_streams;
	int ret = -EINVAL;

	mutex_lock(&kctx->csf.lock);

	group = find_queue_group(kctx, bind->in.group_handle);
	queue = find_queue(kctx, bind->in.buffer_gpu_addr);

	if (!group || !queue)
		goto out;

	/* For the time being, all CSGs have the same number of CSs
	 * so we check CSG 0 for this number
	 */
	max_streams = kctx->kbdev->csf.global_iface.groups[0].stream_num;

	if (bind->in.csi_index >= max_streams)
		goto out;

	if (group->run_state == KBASE_CSF_GROUP_TERMINATED)
		goto out;

	if (queue->group || group->bound_queues[bind->in.csi_index])
		goto out;

	ret = get_user_pages_mmap_handle(kctx, queue);
	if (ret)
		goto out;

	bind->out.mmap_handle = queue->handle;
	group->bound_queues[bind->in.csi_index] = queue;
	queue->group = group;
	queue->group_priority = group->priority;
	queue->csi_index = (s8)bind->in.csi_index;
	queue->bind_state = KBASE_CSF_QUEUE_BIND_IN_PROGRESS;

out:
	mutex_unlock(&kctx->csf.lock);

	return ret;
}

/**
 * get_bound_queue_group() - Get the group to which a queue was bound
 *
 * @queue: Pointer to the queue for this group
 *
 * Return: The group to which this queue was bound, or NULL on error.
 */
static struct kbase_queue_group *get_bound_queue_group(struct kbase_queue *queue)
{
	struct kbase_context *kctx = queue->kctx;
	struct kbase_queue_group *group;

	lockdep_assert_held(&kctx->csf.lock);

	if (queue->bind_state == KBASE_CSF_QUEUE_UNBOUND)
		return NULL;

	if (!queue->group)
		return NULL;

	if (queue->csi_index <= KBASEP_IF_NR_INVALID) {
		dev_warn(kctx->kbdev->dev, "CS interface index is incorrect\n");
		return NULL;
	}

	group = queue->group;

	if (group->bound_queues[queue->csi_index] != queue) {
		dev_warn(kctx->kbdev->dev, "Incorrect mapping between queues & queue groups\n");
		return NULL;
	}

	return group;
}

void kbase_csf_ring_csg_doorbell(struct kbase_device *kbdev, int slot)
{
	if (WARN_ON(slot < 0))
		return;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	kbase_csf_ring_csg_slots_doorbell(kbdev, (u32)(1 << slot));
}

void kbase_csf_ring_csg_slots_doorbell(struct kbase_device *kbdev, u32 slot_bitmap)
{
	const u32 allowed_bitmap = (u32)((1U << kbdev->csf.global_iface.group_num) - 1);
	u32 value;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	if (WARN_ON(slot_bitmap > allowed_bitmap))
		return;

	/* The access to GLB_DB_REQ/ACK needs to be ordered with respect to CSG_REQ/ACK and
	 * CSG_DB_REQ/ACK to avoid a scenario where a CSI request overlaps with a CSG request
	 * or 2 CSI requests overlap and FW ends up missing the 2nd request.
	 * Memory barrier is required, both on Host and FW side, to guarantee the ordering.
	 *
	 * 'osh' is used as CPU and GPU would be in the same Outer shareable domain.
	 */
	dmb(osh);

	value = kbase_csf_fw_io_global_read(&kbdev->csf.fw_io, GLB_DB_ACK);
	value ^= slot_bitmap;
	kbase_csf_fw_io_global_write_mask(&kbdev->csf.fw_io, GLB_DB_REQ, value, slot_bitmap);

	kbase_csf_ring_doorbell(kbdev, CSF_KERNEL_DOORBELL_NR);
}

void kbase_csf_ring_cs_user_doorbell(struct kbase_device *kbdev, struct kbase_queue *queue)
{
	mutex_lock(&kbdev->csf.reg_lock);

	if (queue->doorbell_nr != KBASEP_USER_DB_NR_INVALID)
		kbase_csf_ring_doorbell(kbdev, queue->doorbell_nr);

	mutex_unlock(&kbdev->csf.reg_lock);
}

void kbase_csf_ring_cs_kernel_doorbell(struct kbase_device *kbdev, int csi_index, int csg_nr,
				       bool ring_csg_doorbell)
{
	struct kbase_csf_cmd_stream_group_info *ginfo;
	u32 value;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	if (WARN_ON(csg_nr < 0) || WARN_ON((u32)csg_nr >= kbdev->csf.global_iface.group_num))
		return;

	ginfo = &kbdev->csf.global_iface.groups[csg_nr];

	if (WARN_ON(csi_index < 0) || WARN_ON((u32)csi_index >= ginfo->stream_num))
		return;

	/* The access to CSG_DB_REQ/ACK needs to be ordered with respect to
	 * CS_REQ/ACK to avoid a scenario where CSG_DB_REQ/ACK becomes visibile to
	 * FW before CS_REQ/ACK is set.
	 *
	 * 'osh' is used as CPU and GPU would be in the same outer shareable domain.
	 */
	dmb(osh);

	value = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, csg_nr, CSG_DB_ACK);
	value ^= (1U << csi_index);
	kbase_csf_fw_io_group_write_mask(&kbdev->csf.fw_io, csg_nr, CSG_DB_REQ, value,
					 1U << csi_index);

	if (likely(ring_csg_doorbell))
		kbase_csf_ring_csg_doorbell(kbdev, csg_nr);
}

int kbase_csf_queue_group_clear_faults(struct kbase_context *kctx,
				       struct kbase_ioctl_queue_group_clear_faults *faults)
{
	void __user *user_bufs = u64_to_user_ptr(faults->addr);
	u32 i;
	struct kbase_device *kbdev = kctx->kbdev;
	const u32 nr_queues = faults->nr_queues;

	if (unlikely(nr_queues > kbdev->csf.global_iface.groups[0].stream_num)) {
		dev_warn(kbdev->dev, "Invalid nr_queues %u", nr_queues);
		return -EINVAL;
	}

	for (i = 0; i < nr_queues; ++i) {
		u64 buf_gpu_addr;
		struct kbase_va_region *region;

		if (copy_from_user(&buf_gpu_addr, user_bufs, sizeof(buf_gpu_addr)))
			return -EFAULT;
		mutex_lock(&kctx->csf.lock);
		kbase_gpu_vm_lock(kctx);
		region = kbase_region_tracker_find_region_enclosing_address(kctx, buf_gpu_addr);
		if (likely(!kbase_is_region_invalid_or_free(region))) {
			struct kbase_queue *queue = region->user_data;

			queue->clear_faults = true;
		} else {
			dev_warn(kbdev->dev, "GPU queue %u without a valid command buffer region",
				 i);
			kbase_gpu_vm_unlock(kctx);
			mutex_unlock(&kctx->csf.lock);
			return -EFAULT;
		}
		kbase_gpu_vm_unlock(kctx);
		mutex_unlock(&kctx->csf.lock);
		user_bufs = (void __user *)((uintptr_t)user_bufs + sizeof(buf_gpu_addr));
	}

	return 0;
}

int kbase_csf_queue_kick(struct kbase_context *kctx, struct kbase_ioctl_cs_queue_kick *kick)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_va_region *region;
	int err = 0;

	KBASE_TLSTREAM_TL_KBASE_GPUCMDQUEUE_KICK(kbdev, kctx->id, kick->buffer_gpu_addr);

	/* GPU work submission happening asynchronously to prevent the contention with
	 * scheduler lock and as the result blocking application thread. For this reason,
	 * the vm_lock is used here to get the reference to the queue based on its buffer_gpu_addr
	 * from the context list of active va_regions.
	 * Once the target queue is found the pending flag is set to one atomically avoiding
	 * a race between submission ioctl thread and the work item.
	 */
	kbase_gpu_vm_lock(kctx);
	region = kbase_region_tracker_find_region_enclosing_address(kctx, kick->buffer_gpu_addr);
	if (!kbase_is_region_invalid_or_free(region)) {
		struct kbase_queue *queue = region->user_data;

		if (queue && (queue->bind_state == KBASE_CSF_QUEUE_BOUND)) {
			spin_lock(&kbdev->csf.pending_gpuq_kick_queues_lock);
			if (list_empty(&queue->pending_kick_link)) {
				/* Queue termination shall block until this
				 * kick has been handled.
				 */
				atomic_inc(&queue->pending_kick);
				list_add_tail(
					&queue->pending_kick_link,
					&kbdev->csf.pending_gpuq_kick_queues[queue->group_priority]);
				if (atomic_cmpxchg(&kbdev->csf.pending_gpuq_kicks, false, true) ==
				    false)
					complete(&kbdev->csf.scheduler.kthread_signal);
			}
			spin_unlock(&kbdev->csf.pending_gpuq_kick_queues_lock);
		}
	} else {
		dev_dbg(kbdev->dev,
			"Attempt to kick GPU queue without a valid command buffer region");
		err = -EFAULT;
	}
	kbase_gpu_vm_unlock(kctx);

	return err;
}

static void unbind_stopped_queue(struct kbase_context *kctx, struct kbase_queue *queue)
{
	lockdep_assert_held(&kctx->csf.lock);

	if (WARN_ON(queue->csi_index < 0))
		return;

	if (queue->bind_state != KBASE_CSF_QUEUE_UNBOUND) {
		unsigned long flags;

		kbase_csf_scheduler_spin_lock(kctx->kbdev, &flags);
		bitmap_clear(queue->group->protm_pending_bitmap, (unsigned int)queue->csi_index, 1);
		KBASE_KTRACE_ADD_CSF_GRP_Q(kctx->kbdev, CSI_PROTM_PEND_CLEAR, queue->group, queue,
					   queue->group->protm_pending_bitmap[0]);
		queue->group->bound_queues[queue->csi_index] = NULL;
		queue->group = NULL;
		kbase_csf_scheduler_spin_unlock(kctx->kbdev, flags);

		put_user_pages_mmap_handle(kctx, queue);
		WARN_ON_ONCE(queue->doorbell_nr != KBASEP_USER_DB_NR_INVALID);
		queue->bind_state = KBASE_CSF_QUEUE_UNBOUND;
	}
}
/**
 * unbind_queue() - Remove the linkage between a GPU command queue and the group
 *		    to which it was bound or being bound.
 *
 * @kctx:	Address of the kbase context within which the queue was created.
 * @queue:	Pointer to the queue to be unlinked.
 *
 * This function will also send the stop request to firmware for the CS
 * if the group to which the GPU command queue was bound is scheduled.
 *
 * This function would be called when :-
 * - queue is being unbound. This would happen when the IO mapping
 *   created on bind is removed explicitly by userspace or the process
 *   is getting exited.
 * - queue group is being terminated which still has queues bound
 *   to it. This could happen on an explicit terminate request from userspace
 *   or when the kbase context is being terminated.
 * - queue is being terminated without completing the bind operation.
 *   This could happen if either the queue group is terminated
 *   after the CS_QUEUE_BIND ioctl but before the 2nd part of bind operation
 *   to create the IO mapping is initiated.
 * - There is a failure in executing the 2nd part of bind operation, inside the
 *   mmap handler, which creates the IO mapping for queue.
 */

static void unbind_queue(struct kbase_context *kctx, struct kbase_queue *queue)
{
	kbase_reset_gpu_assert_failed_or_prevented(kctx->kbdev);
	lockdep_assert_held(&kctx->csf.lock);

	if (queue->bind_state != KBASE_CSF_QUEUE_UNBOUND) {
		if (queue->bind_state == KBASE_CSF_QUEUE_BOUND)
			kbase_csf_scheduler_queue_stop(queue);

		unbind_stopped_queue(kctx, queue);
	}
}

void kbase_csf_queue_unbind(struct kbase_queue *queue, bool process_exit)
{
	struct kbase_context *kctx = queue->kctx;

	lockdep_assert_held(&kctx->csf.lock);

	/* As the process itself is exiting, the termination of queue group can
	 * be done which would be much faster than stopping of individual
	 * queues. This would ensure a faster exit for the process especially
	 * in the case where CSI gets stuck.
	 * The CSI STOP request will wait for the in flight work to drain
	 * whereas CSG TERM request would result in an immediate abort or
	 * cancellation of the pending work.
	 */
	if (process_exit) {
		struct kbase_queue_group *group = get_bound_queue_group(queue);

		if (group)
			term_queue_group(group);

		WARN_ON(queue->bind_state != KBASE_CSF_QUEUE_UNBOUND);
	} else {
		unbind_queue(kctx, queue);
	}

	kbase_csf_free_command_stream_user_pages(kctx, queue);
}

void kbase_csf_queue_unbind_stopped(struct kbase_queue *queue)
{
	struct kbase_context *kctx = queue->kctx;

	lockdep_assert_held(&kctx->csf.lock);

	WARN_ON(queue->bind_state == KBASE_CSF_QUEUE_BOUND);
	unbind_stopped_queue(kctx, queue);

	kbase_csf_free_command_stream_user_pages(kctx, queue);
}

/**
 * find_free_group_handle() - Find a free handle for a queue group
 *
 * @kctx: Address of the kbase context within which the queue group
 *        is to be created.
 *
 * Return: a queue group handle on success, or a negative error code on failure.
 */
static int find_free_group_handle(struct kbase_context *const kctx)
{
	/* find the available index in the array of CSGs per this context */
	int idx, group_handle = -ENOMEM;

	lockdep_assert_held(&kctx->csf.lock);

	for (idx = 0; (idx != MAX_QUEUE_GROUP_NUM) && (group_handle < 0); idx++) {
		if (!kctx->csf.queue_groups[idx])
			group_handle = idx;
	}

	return group_handle;
}

/**
 * iface_has_enough_streams() - Check that at least one CSG supports
 *                              a given number of CS
 *
 * @kbdev:  Instance of a GPU platform device that implements a CSF interface.
 * @cs_min: Minimum number of CSs required.
 *
 * Return: true if at least one CSG supports the given number
 *         of CSs (or more); otherwise false.
 */
static bool iface_has_enough_streams(struct kbase_device *const kbdev, u32 const cs_min)
{
	bool has_enough = false;
	struct kbase_csf_cmd_stream_group_info *const groups = kbdev->csf.global_iface.groups;
	const u32 group_num = kbdev->csf.global_iface.group_num;
	u32 i;

	for (i = 0; (i < group_num) && !has_enough; i++) {
		if (groups[i].stream_num >= cs_min)
			has_enough = true;
	}

	return has_enough;
}

/**
 * create_normal_suspend_buffer() - Create normal-mode suspend buffer per
 *					queue group
 *
 * @kctx:	Pointer to kbase context where the queue group is created at
 * @s_buf:	Pointer to suspend buffer that is attached to queue group
 *
 * Return: 0 if phy-pages for the suspend buffer is successfully allocated.
 *	   Otherwise -ENOMEM or error code.
 */
static int create_normal_suspend_buffer(struct kbase_context *const kctx,
					struct kbase_normal_suspend_buffer *s_buf)
{
	const size_t nr_pages = PFN_UP(kctx->kbdev->csf.global_iface.groups[0].suspend_size);
	int err;

	lockdep_assert_held(&kctx->csf.lock);

	/* The suspend buffer's mapping address is valid only when the CSG is to
	 * run on slot, initializing it 0, signalling the buffer is not mapped.
	 */
	s_buf->gpu_va = 0;

	s_buf->phy = kcalloc(nr_pages, sizeof(*s_buf->phy), GFP_KERNEL);

	if (!s_buf->phy)
		return -ENOMEM;

	/* Get physical page for a normal suspend buffer */
	err = kbase_mem_pool_alloc_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], nr_pages,
					 &s_buf->phy[0], false, kctx->task);

	if (err < 0) {
		kfree(s_buf->phy);
		return err;
	}

	kbase_process_page_usage_inc(kctx, nr_pages);
	return 0;
}

static void timer_event_worker(struct work_struct *data);
static void term_normal_suspend_buffer(struct kbase_context *const kctx,
				       struct kbase_normal_suspend_buffer *s_buf);

/**
 * create_suspend_buffers() - Setup normal and protected mode
 *				suspend buffers.
 *
 * @kctx:	Address of the kbase context within which the queue group
 *		is to be created.
 * @group:	Pointer to GPU command queue group data.
 *
 * Return: 0 if suspend buffers are successfully allocated. Otherwise -ENOMEM.
 */
static int create_suspend_buffers(struct kbase_context *const kctx,
				  struct kbase_queue_group *const group)
{
	if (create_normal_suspend_buffer(kctx, &group->normal_suspend_buf)) {
		dev_err(kctx->kbdev->dev, "Failed to create normal suspend buffer\n");
		return -ENOMEM;
	}

	/* Protected suspend buffer, runtime binding so just initialize it */
	group->protected_suspend_buf.gpu_va = 0;
	group->protected_suspend_buf.pma = NULL;
	group->protected_suspend_buf.alloc_retries = 0;

	return 0;
}

/**
 * generate_group_uid() - Makes an ID unique to all kernel base devices
 *                        and contexts, for a queue group and CSG.
 *
 * Return:      A unique ID in the form of an unsigned 32-bit integer
 */
static u32 generate_group_uid(void)
{
	static atomic_t global_csg_uid = ATOMIC_INIT(0);

	return (u32)atomic_inc_return(&global_csg_uid);
}

/**
 * create_queue_group() - Create a queue group
 *
 * @kctx:	Address of the kbase context within which the queue group
 *		is to be created.
 * @create:	Address of a structure which contains details of the
 *		queue group which is to be created.
 *
 * Return: a queue group handle on success, or a negative error code on failure.
 */
static int create_queue_group(struct kbase_context *const kctx,
			      union kbase_ioctl_cs_queue_group_create *const create)
{
	int group_handle = find_free_group_handle(kctx);
	struct kbase_queue_group *group;
	int err = 0;
	int j;

	lockdep_assert_held(&kctx->csf.lock);

	if (group_handle < 0) {
		dev_dbg(kctx->kbdev->dev, "All queue group handles are already in use");

		err = group_handle;
		goto exit;
	}

	group = kzalloc(sizeof(struct kbase_queue_group), GFP_KERNEL);
	if (!group) {
		err = -ENOMEM;
		goto exit;
	}

#if IS_ENABLED(CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD)
	group->prev_act = false;
#endif
	group->kctx = kctx;
	group->handle = group_handle;
	group->csg_nr = KBASEP_CSG_NR_INVALID;

	group->tiler_mask = create->in.tiler_mask;
	group->fragment_mask = create->in.fragment_mask;
	group->compute_mask = create->in.compute_mask;

	group->tiler_max = create->in.tiler_max;
	group->fragment_max = create->in.fragment_max;
	group->compute_max = create->in.compute_max;
	group->csi_handlers = create->in.csi_handlers;
	group->priority = kbase_csf_priority_queue_group_priority_to_relative(
		kbase_csf_priority_check(kctx->kbdev, create->in.priority));
	group->doorbell_nr = KBASEP_USER_DB_NR_INVALID;
	group->faulted = false;
	group->cs_unrecoverable = false;
	group->reevaluate_idle_status = false;
	group->idle_on_stop = false;

	group->csg_reg = NULL;
	group->csg_reg_bind_retries = 0;

	group->dvs_buf = create->in.dvs_buf;

	group->neural_max = create->in.neural_max;
	group->neural_mask = create->in.neural_mask;
	group->comp_pri_threshold = create->in.comp_pri_threshold;
	group->comp_pri_ratio = create->in.comp_pri_ratio;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	group->deschedule_deferred_cnt = 0;
#endif

	group->cs_fault_report_enable = create->in.cs_fault_report_enable;


	group->group_uid = generate_group_uid();
	create->out.group_uid = group->group_uid;

	INIT_LIST_HEAD(&group->link);
	INIT_LIST_HEAD(&group->link_to_schedule);
	INIT_LIST_HEAD(&group->error_fatal.link);
	INIT_WORK(&group->timer_event_work, timer_event_worker);
	INIT_LIST_HEAD(&group->protm_event_work);
	group->progress_timer_state = 0;
	atomic_set(&group->pending_protm_event_work, 0);
	bitmap_zero(group->protm_pending_bitmap, BASEP_GPU_QUEUE_PER_QUEUE_GROUP_MAX);

	group->run_state = KBASE_CSF_GROUP_INACTIVE;
	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_INACTIVE, group, group->run_state);

	err = create_suspend_buffers(kctx, group);
	if (err < 0)
		goto exit_kfree_group;

	if (mali_kbase_supports_csg_cs_user_page_allocation(kctx->api_version)) {
		err = kernel_alloc_user_io_pages(kctx, group->phys, &group->user_io_addr,
						 &group->user_io_gpu_va);
		if (err < 0)
			goto exit_terminate_suspend_buffer;
	}

	kctx->csf.queue_groups[group_handle] = group;

	for (j = 0; j < BASEP_GPU_QUEUE_PER_QUEUE_GROUP_MAX; j++)
		group->bound_queues[j] = NULL;

	return group_handle;

exit_terminate_suspend_buffer:
	term_normal_suspend_buffer(kctx, &group->normal_suspend_buf);

exit_kfree_group:
	kfree(group);

exit:
	return err;
}

static bool dvs_supported(u32 csf_version)
{
	if (GLB_VERSION_MAJOR_GET(csf_version) < 3)
		return false;

	if (GLB_VERSION_MAJOR_GET(csf_version) == 3)
		if (GLB_VERSION_MINOR_GET(csf_version) < 2)
			return false;

	return true;
}

static bool compute_endpoint_prioritization_supported(struct kbase_device *kbdev)
{
	struct kbase_gpu_id_props *gpu_id = &kbdev->gpu_props.gpu_id;

	return (gpu_id->arch_major >= 14);
}

int kbase_csf_queue_group_create(struct kbase_context *const kctx,
				 union kbase_ioctl_cs_queue_group_create *const create)
{
	int err = 0;
	const u32 tiler_count = hweight64(create->in.tiler_mask);
	const u32 fragment_count = hweight64(create->in.fragment_mask);
	const u32 compute_count = hweight64(create->in.compute_mask);
	const u32 neural_count = hweight64(create->in.neural_mask);
	const u8 comp_pri_threshold = create->in.comp_pri_threshold;
	const u8 comp_pri_ratio = create->in.comp_pri_ratio;
	const bool compute_ep_prio_supported =
		compute_endpoint_prioritization_supported(kctx->kbdev);

	mutex_lock(&kctx->csf.lock);

	if ((create->in.tiler_max > tiler_count) || (create->in.fragment_max > fragment_count) ||
	    (create->in.neural_max > neural_count) || (create->in.compute_max > compute_count)) {
		dev_dbg(kctx->kbdev->dev, "Invalid maximum number of endpoints for a queue group");
		err = -EINVAL;
	} else if (create->in.priority >= BASE_QUEUE_GROUP_PRIORITY_COUNT) {
		dev_dbg(kctx->kbdev->dev, "Invalid queue group priority %u",
			(unsigned int)create->in.priority);
		err = -EINVAL;
	} else if (!iface_has_enough_streams(kctx->kbdev, create->in.cs_min)) {
		dev_dbg(kctx->kbdev->dev, "No CSG has at least %d CSs", create->in.cs_min);
		err = -EINVAL;
	} else if (create->in.csi_handlers & ~BASE_CSF_EXCEPTION_HANDLER_FLAGS_MASK) {
		dev_warn(kctx->kbdev->dev, "Unknown exception handler flags set: %u",
			 create->in.csi_handlers & ~BASE_CSF_EXCEPTION_HANDLER_FLAGS_MASK);
		err = -EINVAL;
	} else if (!dvs_supported(kctx->kbdev->csf.global_iface.version) && create->in.dvs_buf) {
		dev_warn(kctx->kbdev->dev,
			 "GPU does not support DVS but userspace is trying to use it");
		err = -EINVAL;
	} else if (dvs_supported(kctx->kbdev->csf.global_iface.version) &&
		   !CSG_DVS_BUF_BUFFER_POINTER_GET(create->in.dvs_buf) &&
		   CSG_DVS_BUF_BUFFER_SIZE_GET(create->in.dvs_buf)) {
		dev_warn(kctx->kbdev->dev, "DVS buffer pointer is null but size is not 0");
		err = -EINVAL;
	} else if (neural_count && !kbase_csf_dev_has_ne(kctx->kbdev)) {
		dev_warn(kctx->kbdev->dev, "Device does not support Neural Engine feature");
		err = -EINVAL;
	} else if ((comp_pri_threshold || comp_pri_ratio) && !compute_ep_prio_supported) {
		dev_warn(kctx->kbdev->dev,
			 "Device does not support compute endpoint prioritization feature");
		err = -EINVAL;
	} else {
		/* For the CSG which satisfies the condition for having
		 * the needed number of CSs, check whether it also conforms
		 * with the requirements for at least one of its CSs having
		 * the iterator of the needed type
		 * (note: for CSF v1.0 all CSs in a CSG will have access to
		 * the same iterators)
		 */
		const int group_handle = create_queue_group(kctx, create);

		if (group_handle >= 0)
			create->out.group_handle = group_handle;
		else
			err = group_handle;
	}

	mutex_unlock(&kctx->csf.lock);

	return err;
}

/**
 * term_normal_suspend_buffer() - Free normal-mode suspend buffer of queue group
 *
 * @kctx:	Pointer to kbase context where queue group belongs to
 * @s_buf:	Pointer to queue group suspend buffer to be freed
 */
static void term_normal_suspend_buffer(struct kbase_context *const kctx,
				       struct kbase_normal_suspend_buffer *s_buf)
{
	const size_t nr_pages = PFN_UP(kctx->kbdev->csf.global_iface.groups[0].suspend_size);

	lockdep_assert_held(&kctx->csf.lock);

	/* The group should not have a bind remaining on any suspend buf region */
	WARN_ONCE(s_buf->gpu_va, "Suspend buffer address should be 0 at termination");

	kbase_mem_pool_free_pages(&kctx->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], nr_pages,
				  &s_buf->phy[0], false, false);
	kbase_process_page_usage_dec(kctx, nr_pages);

	kfree(s_buf->phy);
	s_buf->phy = NULL;
}

/**
 * term_protected_suspend_buffer() - Free protected-mode suspend buffer of
 *					queue group
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @sbuf: Pointer to queue group suspend buffer to be freed
 */
static void term_protected_suspend_buffer(struct kbase_device *const kbdev,
					  struct kbase_protected_suspend_buffer *sbuf)
{
	WARN_ONCE(sbuf->gpu_va, "Suspend buf should have been unmapped inside scheduler!");
	if (sbuf->pma) {
		const size_t nr_pages = PFN_UP(kbdev->csf.global_iface.groups[0].suspend_size);
		kbase_csf_protected_memory_free(kbdev, sbuf->pma, nr_pages, true);
		sbuf->pma = NULL;
	}
}

void kbase_csf_term_descheduled_queue_group(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;

	/* Currently each group supports the same number of CS */
	u32 max_streams = kctx->kbdev->csf.global_iface.groups[0].stream_num;
	u32 i;

	lockdep_assert_held(&kctx->csf.lock);

	WARN_ON(group->run_state != KBASE_CSF_GROUP_INACTIVE &&
		group->run_state != KBASE_CSF_GROUP_FAULT_EVICTED);

	for (i = 0; i < max_streams; i++) {
		struct kbase_queue *queue = group->bound_queues[i];

		/* The group is already being evicted from the scheduler */
		if (queue)
			unbind_stopped_queue(kctx, queue);
	}

	term_normal_suspend_buffer(kctx, &group->normal_suspend_buf);
	if (kctx->kbdev->csf.pma_dev)
		term_protected_suspend_buffer(kctx->kbdev, &group->protected_suspend_buf);

	if (mali_kbase_supports_csg_cs_user_page_allocation(kctx->api_version)) {
		kernel_free_user_io_pages(kctx, group->phys, group->user_io_addr);
		group->user_io_addr = NULL;
	}

	group->run_state = KBASE_CSF_GROUP_TERMINATED;
	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, CSF_GROUP_TERMINATED, group, group->run_state);
}

/**
 * term_queue_group() - Terminate a GPU command queue group.
 *
 * @group: Pointer to GPU command queue group data.
 *
 * Terminates a GPU command queue group. From the userspace perspective the
 * group will still exist but it can't bind new queues to it. Userspace can
 * still add work in queues bound to the group but it won't be executed. (This
 * is because the IO mapping created upon binding such queues is still intact.)
 */
static void term_queue_group(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;

	kbase_reset_gpu_assert_failed_or_prevented(kctx->kbdev);
	lockdep_assert_held(&kctx->csf.lock);

	/* Stop the group and evict it from the scheduler */
	kbase_csf_scheduler_group_deschedule(group);

	if (group->run_state == KBASE_CSF_GROUP_TERMINATED)
		return;

	dev_dbg(kctx->kbdev->dev, "group %d terminating", group->handle);

	kbase_csf_term_descheduled_queue_group(group);
}

/**
 * wait_group_deferred_deschedule_completion() - Wait for refcount of the group
 *     to become 0 that was taken when the group deschedule had to be deferred.
 *
 * @group: Pointer to GPU command queue group that is being deleted.
 *
 * This function is called when Userspace deletes the group and after the group
 * has been descheduled. The function synchronizes with the other threads that were
 * also trying to deschedule the group whilst the dumping was going on for a fault.
 * Please refer the documentation of wait_for_dump_complete_on_group_deschedule()
 * for more details.
 */
static void wait_group_deferred_deschedule_completion(struct kbase_queue_group *group)
{
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct kbase_context *kctx = group->kctx;

	lockdep_assert_held(&kctx->csf.lock);

	if (likely(!group->deschedule_deferred_cnt))
		return;

	mutex_unlock(&kctx->csf.lock);
	wait_event(kctx->kbdev->csf.event_wait, !group->deschedule_deferred_cnt);
	mutex_lock(&kctx->csf.lock);
#endif
}

static void cancel_queue_group_events(struct kbase_queue_group *group)
{
	cancel_work_sync(&group->timer_event_work);

	/* Drain a pending protected mode request if any */
	kbase_csf_scheduler_wait_for_kthread_pending_work(group->kctx->kbdev,
							  &group->pending_protm_event_work);
}

static void remove_pending_group_fatal_error(struct kbase_queue_group *group)
{
	struct kbase_context *kctx = group->kctx;

	dev_dbg(kctx->kbdev->dev, "Remove any pending group fatal error from context %pK\n",
		(void *)group->kctx);

	kbase_csf_event_remove_error(kctx, &group->error_fatal);
}

void kbase_csf_queue_group_terminate(struct kbase_context *kctx, u8 group_handle)
{
	struct kbase_queue_group *group;
	int err;
	bool reset_prevented = false;
	struct kbase_device *const kbdev = kctx->kbdev;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating group %d, attempting to terminate regardless",
			group_handle);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	group = find_queue_group(kctx, group_handle);

	if (group) {
		kctx->csf.queue_groups[group_handle] = NULL;
		/* Stop the running of the given group */
		term_queue_group(group);
		mutex_unlock(&kctx->csf.lock);

		if (reset_prevented) {
			/* Allow GPU reset before cancelling the group specific
			 * work item to avoid potential deadlock.
			 * Reset prevention isn't needed after group termination.
			 */
			kbase_reset_gpu_allow(kbdev);
			reset_prevented = false;
		}

		/* Cancel any pending event callbacks. If one is in progress
		 * then this thread waits synchronously for it to complete (which
		 * is why we must unlock the context first). We already ensured
		 * that no more callbacks can be enqueued by terminating the group.
		 */
		cancel_queue_group_events(group);

		mutex_lock(&kctx->csf.lock);

		/* Clean up after the termination */
		remove_pending_group_fatal_error(group);

		wait_group_deferred_deschedule_completion(group);
	}

	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);

	kfree(group);
}
KBASE_EXPORT_TEST_API(kbase_csf_queue_group_terminate);

#if IS_ENABLED(CONFIG_MALI_VECTOR_DUMP) || MALI_UNIT_TEST
int kbase_csf_queue_group_suspend(struct kbase_context *kctx,
				  struct kbase_suspend_copy_buffer *sus_buf, u8 group_handle)
{
	struct kbase_device *const kbdev = kctx->kbdev;
	int err;
	struct kbase_queue_group *group;

	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err) {
		dev_warn(kbdev->dev, "Unsuccessful GPU reset detected when suspending group %d",
			 group_handle);
		return err;
	}
	mutex_lock(&kctx->csf.lock);

	group = find_queue_group(kctx, group_handle);
	if (group)
		err = kbase_csf_scheduler_group_copy_suspend_buf(group, sus_buf);
	else
		err = -EINVAL;

	mutex_unlock(&kctx->csf.lock);
	kbase_reset_gpu_allow(kbdev);

	return err;
}
#endif

void kbase_csf_add_group_fatal_error(struct kbase_queue_group *const group,
				     struct base_gpu_queue_group_error const *const err_payload)
{
	struct base_csf_notification error;

	if (WARN_ON(!group))
		return;

	if (WARN_ON(!err_payload))
		return;

	error = (struct base_csf_notification){
		.type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
		.payload = { .csg_error = { .handle = group->handle, .error = *err_payload } }
	};

	kbase_csf_event_add_error(group->kctx, &group->error_fatal, &error);
}

void kbase_csf_active_queue_groups_reset(struct kbase_device *kbdev, struct kbase_context *kctx)
{
	struct list_head evicted_groups;
	struct kbase_queue_group *group;
	int i;

	INIT_LIST_HEAD(&evicted_groups);

	mutex_lock(&kctx->csf.lock);

	kbase_csf_scheduler_evict_ctx_slots(kbdev, kctx, &evicted_groups);
	while (!list_empty(&evicted_groups)) {
		group = list_first_entry(&evicted_groups, struct kbase_queue_group, link);

		dev_dbg(kbdev->dev, "Context %d_%d active group %d terminated", kctx->tgid,
			kctx->id, group->handle);
		kbase_csf_term_descheduled_queue_group(group);
		list_del_init(&group->link);
	}

	/* Acting on the queue groups that are pending to be terminated. */
	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		group = kctx->csf.queue_groups[i];
		if (group && group->run_state == KBASE_CSF_GROUP_FAULT_EVICTED)
			kbase_csf_term_descheduled_queue_group(group);
	}

	mutex_unlock(&kctx->csf.lock);
}

int kbase_csf_ctx_init(struct kbase_context *kctx)
{
	int err = -ENOMEM;

	INIT_LIST_HEAD(&kctx->csf.queue_list);
	INIT_LIST_HEAD(&kctx->csf.link);
	atomic_set(&kctx->csf.pending_sync_update, 0);

	kbase_csf_event_init(kctx);

	/* Mark all the cookies as 'free' */
	bitmap_fill(kctx->csf.cookies, KBASE_CSF_NUM_USER_IO_PAGES_HANDLE);

	kctx->csf.wq = alloc_workqueue("mali_kbase_csf_wq", WQ_UNBOUND, 1);

	if (likely(kctx->csf.wq)) {
		err = kbase_csf_scheduler_context_init(kctx);

		if (likely(!err)) {
			err = kbase_csf_kcpu_queue_context_init(kctx);

			if (likely(!err)) {
				err = kbase_csf_tiler_heap_context_init(kctx);

				if (likely(!err)) {
					mutex_init(&kctx->csf.lock);

					err = kbasep_ctx_user_reg_page_mapping_init(kctx);

					if (likely(!err))
						kbase_csf_cpu_queue_init(kctx);

					if (unlikely(err))
						kbase_csf_tiler_heap_context_term(kctx);
				}

				if (unlikely(err))
					kbase_csf_kcpu_queue_context_term(kctx);
			}

			if (unlikely(err))
				kbase_csf_scheduler_context_term(kctx);
		}

		if (unlikely(err))
			destroy_workqueue(kctx->csf.wq);
	}

	return err;
}

void kbase_csf_ctx_report_page_fault_for_active_groups(struct kbase_context *kctx,
						       struct kbase_fault *fault)
{
	struct base_gpu_queue_group_error err_payload =
		(struct base_gpu_queue_group_error){ .error_type = BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
						     .payload = { .fatal_group = {
									  .sideband = fault->addr,
									  .status = fault->status,
								  } } };
	struct kbase_device *kbdev = kctx->kbdev;
	const u32 num_groups = kbdev->csf.global_iface.group_num;
	unsigned long flags;
	int csg_nr;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	for (csg_nr = 0; csg_nr < num_groups; csg_nr++) {
		struct kbase_queue_group *const group =
			kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;

		if (!group || (group->kctx != kctx))
			continue;

		group->faulted = true;
		kbase_csf_add_group_fatal_error(group, &err_payload);
	}
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
}

void kbase_csf_ctx_handle_fault(struct kbase_context *kctx, struct kbase_fault *fault,
				bool fw_unresponsive)
{
	int gr;
	bool reported = false;
	struct base_gpu_queue_group_error err_payload;

	if (WARN_ON(!kctx))
		return;

	if (WARN_ON(!fault))
		return;

	err_payload =
		(struct base_gpu_queue_group_error){ .error_type = BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
						     .payload = { .fatal_group = {
									  .sideband = fault->addr,
									  .status = fault->status,
								  } } };

	lockdep_assert_held(&kctx->csf.lock);

	for (gr = 0; gr < MAX_QUEUE_GROUP_NUM; gr++) {
		struct kbase_queue_group *const group = kctx->csf.queue_groups[gr];

		if (group && group->run_state != KBASE_CSF_GROUP_TERMINATED) {
			/* If the FW is known to have become unresponsive, then the wait for CSG
			 * termination request can be safely skipped.
			 */
			if (fw_unresponsive)
				group->cs_unrecoverable = true;

			term_queue_group(group);
			/* This would effectively be a NOP if the fatal error was already added to
			 * the error_list by kbase_csf_ctx_report_page_fault_for_active_groups().
			 */
			kbase_csf_add_group_fatal_error(group, &err_payload);
			reported = true;
		}
	}

	if (reported)
		kbase_event_wakeup(kctx);
}

void kbase_csf_ctx_term(struct kbase_context *kctx)
{
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_as *as = NULL;
	unsigned long flags;
	u32 i;
	int err;
	bool reset_prevented = false;

	/* As the kbase context is terminating, its debugfs sub-directory would
	 * have been removed already and so would be the debugfs file created
	 * for queue groups & kcpu queues, hence no need to explicitly remove
	 * those debugfs files.
	 */

	/* Wait for a GPU reset if it is happening, prevent it if not happening */
	err = kbase_reset_gpu_prevent_and_wait(kbdev);
	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating csf context (%d_%d), attempting to terminate regardless",
			kctx->tgid, kctx->id);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	/* Iterate through the queue groups that were not terminated by
	 * userspace and issue the term request to firmware for them.
	 */
	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		struct kbase_queue_group *group = kctx->csf.queue_groups[i];

		if (group) {
			remove_pending_group_fatal_error(group);
			term_queue_group(group);
		}
	}

	mutex_unlock(&kctx->csf.lock);

	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);

	kbase_csf_event_wait_remove(kctx, kbase_csf_scheduler_check_group_sync_update_cb, kctx);

	/* wait until there is no more protm work */
	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		struct kbase_queue_group *group = kctx->csf.queue_groups[i];

		if (group) {
			/* Drain a pending protected mode request if any */
			kbase_csf_scheduler_wait_for_kthread_pending_work(
				group->kctx->kbdev, &group->pending_protm_event_work);
		}
	}

	/* Drain pending SYNC_UPDATE work if any */
	kbase_csf_scheduler_wait_for_kthread_pending_work(kctx->kbdev,
							  &kctx->csf.pending_sync_update);

	/* Now that all queue groups have been terminated, there can be no
	 * more OoM or timer event interrupts but there can be inflight work
	 * items. Destroying the wq will implicitly flush those work items.
	 */
	destroy_workqueue(kctx->csf.wq);

	/* Wait for the firmware error work item to also finish as it could
	 * be affecting this outgoing context also.
	 */
	flush_work(&kctx->kbdev->csf.glb_fatal_work);

	/* A work item to handle page_fault/bus_fault/gpu_fault could be
	 * pending for the outgoing context. Flush the workqueue that will
	 * execute that work item.
	 */
	spin_lock_irqsave(&kctx->kbdev->hwaccess_lock, flags);
	if (kctx->as_nr != KBASEP_AS_NR_INVALID)
		as = &kctx->kbdev->as[kctx->as_nr];
	spin_unlock_irqrestore(&kctx->kbdev->hwaccess_lock, flags);
	if (as)
		flush_workqueue(as->pf_wq);

	mutex_lock(&kctx->csf.lock);

	for (i = 0; i < MAX_QUEUE_GROUP_NUM; i++) {
		kfree(kctx->csf.queue_groups[i]);
		kctx->csf.queue_groups[i] = NULL;
	}

	/* Iterate through the queues that were not terminated by
	 * userspace and do the required cleanup for them.
	 */
	while (!list_empty(&kctx->csf.queue_list)) {
		struct kbase_queue *queue;

		queue = list_first_entry(&kctx->csf.queue_list, struct kbase_queue, link);

		list_del_init(&queue->link);

		mutex_unlock(&kctx->csf.lock);
		wait_pending_queue_kick(queue);
		mutex_lock(&kctx->csf.lock);

		/* The reference held when the IO mapping was created on bind
		 * would have been dropped otherwise the termination of Kbase
		 * context itself wouldn't have kicked-in. So there shall be
		 * only one reference left that was taken when queue was
		 * registered.
		 */
		WARN_ON(kbase_refcount_read(&queue->refcount) != 1);

		release_queue(queue);
	}

	mutex_unlock(&kctx->csf.lock);

	kbasep_ctx_user_reg_page_mapping_term(kctx);
	kbase_csf_tiler_heap_context_term(kctx);
	kbase_csf_kcpu_queue_context_term(kctx);
	kbase_csf_scheduler_context_term(kctx);
	kbase_csf_event_term(kctx);

	mutex_destroy(&kctx->csf.lock);
}

int kbase_csf_cs_get_pending_oom(struct kbase_device *kbdev, struct kbase_queue *queue,
				 int const slot_id)
{
	u32 stream_id = queue->csi_index;
	u32 vt_start, vt_end, frag_end;
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long flags;

	if (WARN_ON(slot_id < 0 || slot_id >= kbdev->csf.global_iface.group_num))
		return -EINVAL;

	if (queue->oom_track.state == KBASE_CSF_QUEUE_OOM_PENDING) {
		dev_dbg(kbdev->dev,
			"Tiler OOM request is already being handled, tracking state %s: queue %d group %d (ctx %d_%d)",
			kbase_csf_queue_oom_state_str(queue->oom_track.state), stream_id,
			queue->group->handle, queue->kctx->tgid, queue->kctx->id);
		return -EBUSY;
	}

	if (queue->oom_track.state == KBASE_CSF_QUEUE_OOM_ERROR_ABORT) {
		dev_dbg(kbdev->dev, "Incorrect Tiler OOM state (%s): queue %d group %d (ctx %d_%d)",
			kbase_csf_queue_oom_state_str(queue->oom_track.state), stream_id,
			queue->group->handle, queue->kctx->tgid, queue->kctx->id);
		return -EINVAL;
	}

	/* It is possible that FW will be unresponsive at this point, so force FW IO transaction for
	 * read-only operations.
	 */
	kbase_csf_fw_io_open_force(fw_io, &flags);

	vt_start = kbase_csf_fw_io_stream_read(fw_io, slot_id, stream_id, CS_HEAP_VT_START);
	vt_end = kbase_csf_fw_io_stream_read(fw_io, slot_id, stream_id, CS_HEAP_VT_END);
	frag_end = kbase_csf_fw_io_stream_read(fw_io, slot_id, stream_id, CS_HEAP_FRAG_END);

	if ((frag_end > vt_end) || (vt_end >= vt_start)) {
		/* Marking the state in error, ready for the group to be terminated later */
		queue->oom_track = (struct kbase_queue_oom_transact){
			.state = KBASE_CSF_QUEUE_OOM_ERROR_ABORT
		};

		dev_warn(
			kbdev->dev,
			"Invalid Heap statistics provided by firmware: vt_start %d, vt_end %d, frag_end %d\n",
			vt_start, vt_end, frag_end);
	} else {
		u64 gpu_heap_va =
			kbase_csf_fw_io_stream_read(fw_io, slot_id, stream_id, CS_HEAP_ADDRESS_LO) |
			((u64)kbase_csf_fw_io_stream_read(fw_io, slot_id, stream_id,
							  CS_HEAP_ADDRESS_HI)
			 << 32);

		queue->oom_track =
			(struct kbase_queue_oom_transact){ .state = KBASE_CSF_QUEUE_OOM_PENDING,
							   .info.rp_in_flight = vt_start - frag_end,
							   .info.pending_frag_cnt =
								   vt_end - frag_end,
							   .info.heap_va = gpu_heap_va };
	}
	kbase_csf_fw_io_close(fw_io, flags);

	return 0;
}
KBASE_EXPORT_TEST_API(kbase_csf_cs_get_pending_oom);

void kbase_csf_program_cs_oom_prepared_chunk(struct kbase_queue *queue, u32 slot_id, u32 cs_oom_req)
{
	struct kbase_context *const kctx = queue->kctx;
	struct kbase_csf_fw_io *fw_io = &kctx->kbdev->csf.fw_io;
	u64 new_chunk_ptr = queue->oom_track.info.chunk_ptr;
	int stream_id = queue->csi_index;

	kbase_csf_fw_io_assert_opened(fw_io);
	if (queue->oom_track.state != KBASE_CSF_QUEUE_OOM_COMPLETE)
		dev_WARN(kctx->kbdev->dev,
			 "Incorrect Tiler OOM state (%s): queue %d group %d (ctx %d_%d)",
			 kbase_csf_queue_oom_state_str(queue->oom_track.state), slot_id,
			 queue->group->handle, queue->kctx->tgid, queue->kctx->id);

	kbase_csf_fw_io_stream_write(fw_io, slot_id, stream_id, CS_TILER_HEAP_START_LO,
				     new_chunk_ptr & 0xFFFFFFFF);
	kbase_csf_fw_io_stream_write(fw_io, slot_id, stream_id, CS_TILER_HEAP_START_HI,
				     new_chunk_ptr >> 32);
	kbase_csf_fw_io_stream_write(fw_io, slot_id, stream_id, CS_TILER_HEAP_END_LO,
				     new_chunk_ptr & 0xFFFFFFFF);
	kbase_csf_fw_io_stream_write(fw_io, slot_id, stream_id, CS_TILER_HEAP_END_HI,
				     new_chunk_ptr >> 32);
	kbase_csf_fw_io_stream_write_mask(fw_io, slot_id, stream_id, CS_REQ, cs_oom_req,
					  CS_REQ_TILER_OOM_MASK);
}

int kbase_csf_cs_prepare_pending_oom_tiler_heap_chunk(struct kbase_queue *queue)
{
	struct kbase_context *const kctx = queue->kctx;
	u64 new_chunk_ptr = 0;
	int err;

	lockdep_assert_held(&kctx->kbdev->csf.scheduler.lock);

	if (queue->oom_track.state != KBASE_CSF_QUEUE_OOM_PENDING) {
		dev_WARN(kctx->kbdev->dev,
			 "Group-%d_%d_queue-%d wrong tiler OoM state(%s) for OoM chunk alloc",
			 kctx->tgid, queue->group->handle, queue->csi_index,
			 kbase_csf_queue_oom_state_str(queue->oom_track.state));
		err = -EINVAL;
		goto update_track_state;
	}

	err = kbase_csf_tiler_heap_alloc_new_chunk(kctx, queue->oom_track.info.heap_va,
						   queue->oom_track.info.rp_in_flight,
						   queue->oom_track.info.pending_frag_cnt,
						   &new_chunk_ptr);

	if ((queue->group->csi_handlers & BASE_CSF_TILER_OOM_EXCEPTION_FLAG) &&
	    (queue->oom_track.info.pending_frag_cnt == 0) && (err == -ENOMEM || err == -EBUSY)) {
		/* The group allows incremental rendering, trigger it */
		new_chunk_ptr = 0;
		err = 0;
		dev_dbg(kctx->kbdev->dev, "Group-%d_%d enter incremental render\n", kctx->tgid,
			queue->group->handle);
	} else if (err == -EBUSY) {
		/* Acknowledge with a NULL chunk (firmware will then wait for
		 * the fragment jobs to complete and release chunks)
		 */
		new_chunk_ptr = 0;
		err = 0;
	}

update_track_state:
	if (err) {
		/* Update the state for consistency */
		queue->oom_track.state = KBASE_CSF_QUEUE_OOM_ERROR_ABORT;
	} else {
		queue->oom_track.info.chunk_ptr = new_chunk_ptr;
		queue->oom_track.state = KBASE_CSF_QUEUE_OOM_COMPLETE;
	}

	return err;
}

int kbase_csf_free_oom_tiler_heap_chunk(struct kbase_queue *queue)
{
	struct kbase_context *const kctx = queue->kctx;
	int err = 0;

	lockdep_assert_held(&kctx->kbdev->csf.scheduler.lock);

	if (queue->oom_track.state != KBASE_CSF_QUEUE_OOM_COMPLETE) {
		dev_WARN(kctx->kbdev->dev,
			 "Group-%d_%d_queue-%d wrong tiler OoM state(%s) for OoM chunk free",
			 kctx->tgid, queue->group->handle, queue->csi_index,
			 kbase_csf_queue_oom_state_str(queue->oom_track.state));
		err = -EINVAL;
		goto update_track_state;
	}

	if (!queue->oom_track.info.chunk_ptr)
		/* The chunk was not allocated, nothing to be freed. */
		goto update_track_state;

	err = kbase_csf_tiler_heap_free_chunk(kctx, queue->oom_track.info.heap_va,
					      queue->oom_track.info.chunk_ptr);

update_track_state:
	if (err) {
		dev_WARN(kctx->kbdev->dev, "Group-%d_%d_queue-%d : failed to free OoM chunk",
			 kctx->tgid, queue->group->handle, queue->csi_index);
		queue->oom_track.state = KBASE_CSF_QUEUE_OOM_ERROR_ABORT;
	} else {
		dev_dbg(kctx->kbdev->dev, "Group-%d_%d_queue-%d : OoM chunk freed", kctx->tgid,
			queue->group->handle, queue->csi_index);
		queue->oom_track.info.chunk_ptr = 0;
		queue->oom_track.state = KBASE_CSF_QUEUE_OOM_PENDING;
	}

	return err;
}
KBASE_EXPORT_TEST_API(kbase_csf_free_oom_tiler_heap_chunk);

/**
 * handle_oom_event() - Handle the OoM event generated by the firmware for the
 *                    CSI.
 *
 * @queue:  Pointer to the queue the oom-event belongs to.
 * @slot_num: the slot number where the queue is residing.
 *
 * This function will handle the OoM event request from the firmware for the
 * CS. It will use the tracked oom information to allocate a new chunk.
 * It will also update the CS's kernel input page with the address
 * of a new chunk that was allocated.
 *
 * Return: 0 if successfully handled the request, otherwise a negative error
 *         code on failure.
 */
static int handle_oom_event(struct kbase_queue *const queue, u32 slot_num)
{
	struct kbase_context *const kctx = queue->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	int stream_id = queue->csi_index;
	u32 cs_oom_ack, cs_oom_req;
	unsigned long flags, fw_io_flags;
	int err;

	lockdep_assert_held(&kctx->csf.lock);
	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (queue->oom_track.state == KBASE_CSF_QUEUE_OOM_ERROR_ABORT)
		return -EINVAL;

	/* The queue (group) could have already undergone suspend-resume cycle, nothing to do */
	if (queue->oom_track.state == KBASE_CSF_QUEUE_OOM_NONE ||
	    queue->oom_track.state == KBASE_CSF_QUEUE_OOM_COMPLETE)
		return 0;

	cs_oom_ack = kbase_csf_fw_io_stream_read(fw_io, slot_num, stream_id, CS_ACK) &
		     CS_ACK_TILER_OOM_MASK;
	cs_oom_req = kbase_csf_fw_io_stream_input_read(fw_io, slot_num, stream_id, CS_REQ) &
		     CS_REQ_TILER_OOM_MASK;

	/* Expecting the req/ack to be consistent with oom_track.state, i.e. should differ */
	if (WARN_ON(cs_oom_ack == cs_oom_req))
		return 0;

	err = kbase_csf_cs_prepare_pending_oom_tiler_heap_chunk(queue);
	if (err)
		return err;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	/* In case of an unresponsive FW, we need to free the allocated chunk.
	 * The chunk will be allocated again during GPU resume.
	 */
	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
		if (queue->oom_track.state == KBASE_CSF_QUEUE_OOM_COMPLETE) {
			if (kbase_csf_free_oom_tiler_heap_chunk(queue))
				dev_warn(kbdev->dev,
					 "Failed to free OoM chunk (slot id %d, stream id %d)",
					 slot_num, stream_id);
		}
		return 0;
	}
	kbase_csf_program_cs_oom_prepared_chunk(queue, slot_num, cs_oom_ack);
	kbase_csf_ring_cs_kernel_doorbell(kbdev, stream_id, slot_num, true);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);

	return 0;
}

/**
 * report_tiler_oom_error() - Report a CSG error due to a tiler heap OOM event
 *
 * @group: Pointer to the GPU command queue group that encountered the error
 */
static void report_tiler_oom_error(struct kbase_queue_group *group)
{
	struct base_csf_notification const
		error = { .type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
			  .payload = {
				  .csg_error = {
					  .handle = group->handle,
					  .error = {
						  .error_type =
							  BASE_GPU_QUEUE_GROUP_ERROR_TILER_HEAP_OOM,
					  } } } };

	kbase_csf_event_add_error(group->kctx, &group->error_fatal, &error);
	kbase_event_wakeup(group->kctx);
}

static void flush_gpu_cache_on_fatal_error(struct kbase_device *kbdev)
{
	kbase_pm_lock(kbdev);
	/* With the advent of partial cache flush, dirty cache lines could
	 * be left in the GPU L2 caches by terminating the queue group here
	 * without waiting for proper cache maintenance. A full cache flush
	 * here will prevent these dirty cache lines from being arbitrarily
	 * evicted later and possible causing memory corruption.
	 */
	if (kbase_io_is_gpu_powered(kbdev)) {
		kbase_gpu_start_cache_clean(kbdev, GPU_COMMAND_CACHE_CLN_INV_L2_LSC);
		if (kbase_gpu_wait_cache_clean_timeout(
			    kbdev, kbase_get_timeout_ms(kbdev, MMU_AS_INACTIVE_WAIT_TIMEOUT)))
			dev_warn(
				kbdev->dev,
				"[%llu] Timeout waiting for CACHE_CLN_INV_L2_LSC to complete after fatal error",
				kbase_backend_get_cycle_cnt(kbdev));
	}

	kbase_pm_unlock(kbdev);
}

/**
 * kbase_queue_oom_event() - Handle tiler out-of-memory for a GPU command queue.
 *
 * @queue: Pointer to queue for which out-of-memory event was received.
 *
 * Called with the CSF locked for the affected GPU virtual address space.
 * Do not call in interrupt context.
 *
 * Handles tiler out-of-memory for a GPU command queue and then clears the
 * notification to allow the firmware to report out-of-memory again in future.
 * If the out-of-memory condition was successfully handled then this function
 * rings the relevant doorbell to notify the firmware; otherwise, it terminates
 * the GPU command queue group to which the queue is bound and notify a waiting
 * user space client of the failure.
 */
static void kbase_queue_oom_event(struct kbase_queue *const queue)
{
	struct kbase_context *const kctx = queue->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	struct kbase_queue_group *group;
	int slot_num, err;

	lockdep_assert_held(&kctx->csf.lock);

	group = get_bound_queue_group(queue);
	if (!group) {
		dev_warn(kctx->kbdev->dev, "queue not bound\n");
		return;
	}

	kbase_csf_scheduler_lock(kbdev);

	if (queue->oom_track.state != KBASE_CSF_QUEUE_OOM_ERROR_ABORT) {
		slot_num = kbase_csf_scheduler_group_get_slot(group);

		/* The group could have gone off slot before this work item got
		 * a chance to execute.
		 */
		if (slot_num < 0)
			goto unlock;

		/* If the bound group is on slot yet the kctx is marked with disabled
		 * on address-space fault, the group is pending to be killed. So skip
		 * the inflight oom operation.
		 */
		if (kbase_ctx_flag(kctx, KCTX_AS_DISABLED_ON_FAULT))
			goto unlock;

		err = handle_oom_event(queue, slot_num);
	} else
		err = -EINVAL;

	if (unlikely(err)) {
		dev_warn(kbdev->dev,
			 "Queue group to be terminated, couldn't handle the OoM event\n");
		kbase_debug_csf_fault_notify(kbdev, kctx, DF_TILER_OOM);
		kbase_csf_scheduler_unlock(kbdev);
		term_queue_group(group);
		flush_gpu_cache_on_fatal_error(kbdev);
		report_tiler_oom_error(group);
		queue->oom_track.state = KBASE_CSF_QUEUE_OOM_NONE;
		return;
	}
unlock:
	kbase_csf_scheduler_unlock(kbdev);
}

/**
 * oom_event_worker() - Tiler out-of-memory handler called from a workqueue.
 *
 * @data: Pointer to a work_struct embedded in GPU command queue data.
 *
 * Handles a tiler out-of-memory condition for a GPU command queue and then
 * releases a reference that was added to prevent the queue being destroyed
 * while this work item was pending on a workqueue.
 */
static void oom_event_worker(struct work_struct *data)
{
	struct kbase_queue *queue = container_of(data, struct kbase_queue, oom_event_work);
	struct kbase_context *kctx = queue->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	int reset_prevent_err = kbase_reset_gpu_try_prevent(kbdev);

	mutex_lock(&kctx->csf.lock);
	if (likely(!reset_prevent_err)) {
		kbase_queue_oom_event(queue);
	} else {
		dev_warn(kbdev->dev,
			 "Unable to prevent GPU reset, couldn't handle the OoM event\n");
	}
	mutex_unlock(&kctx->csf.lock);
	if (likely(!reset_prevent_err))
		kbase_reset_gpu_allow(kbdev);
}

/**
 * report_group_timeout_error() - Report the timeout error for the group to
 *                                userspace.
 *
 * @group: Pointer to the group for which timeout error occurred
 */
static void report_group_timeout_error(struct kbase_queue_group *const group)
{
	struct base_csf_notification const
		error = { .type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR,
			  .payload = {
				  .csg_error = {
					  .handle = group->handle,
					  .error = {
						  .error_type = BASE_GPU_QUEUE_GROUP_ERROR_TIMEOUT,
					  } } } };

	dev_warn(group->kctx->kbdev->dev,
		 "Notify the event notification thread, forward progress timeout (%llu cycles)\n",
		 kbase_csf_timeout_get(group->kctx->kbdev));

	kbase_csf_event_add_error(group->kctx, &group->error_fatal, &error);
	kbase_event_wakeup(group->kctx);
}

/**
 * timer_event_worker() - Handle the progress timeout error for the group
 *
 * @data: Pointer to a work_struct embedded in GPU command queue group data.
 *
 * Terminate the CSG and report the error to userspace
 */
static void timer_event_worker(struct work_struct *data)
{
	struct kbase_queue_group *const group =
		container_of(data, struct kbase_queue_group, timer_event_work);
	struct kbase_context *const kctx = group->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	bool reset_prevented = false;
	int err = kbase_reset_gpu_prevent_and_wait(kbdev);

	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating group %d on progress timeout, attempting to terminate regardless",
			group->handle);
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	term_queue_group(group);
	flush_gpu_cache_on_fatal_error(kbdev);
	report_group_timeout_error(group);

	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);
}

/**
 * handle_progress_timer_events() - Progress timer timeout events handler.
 *
 * @kbdev:     Instance of a GPU platform device that implements a CSF interface.
 * @slot_mask: Bitmap reflecting the slots on which progress timer timeouts happen.
 *
 * Notify a waiting user space client of the timeout.
 * Enqueue a work item to terminate the group and notify the event notification
 * thread of progress timeout fault for the GPU command queue group.
 * Ignore fragment timeout if it is following a compute timeout.
 */
static void handle_progress_timer_events(struct kbase_device *const kbdev, unsigned long *slot_mask)
{
	u32 max_csg_slots = kbdev->csf.global_iface.group_num;
	u32 csg_nr;
	struct kbase_queue_group *group = NULL;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	if (likely(bitmap_empty(slot_mask, BASEP_QUEUE_GROUP_MAX)))
		return;

	/* Log each timeout and Update timestamp of compute progress timeout */
	for_each_set_bit(csg_nr, slot_mask, max_csg_slots) {
		group = kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;
		group->progress_timer_state = kbase_csf_fw_io_group_read(&kbdev->csf.fw_io, csg_nr,
									 CSG_PROGRESS_TIMER_STATE);

		dev_info(
			kbdev->dev,
			"[%llu] Iterator PROGRESS_TIMER timeout notification received for group %u of ctx %d_%d on slot %u with state %x",
			kbase_backend_get_cycle_cnt(kbdev), group->handle, group->kctx->tgid,
			group->kctx->id, csg_nr, group->progress_timer_state);

		if (CSG_PROGRESS_TIMER_STATE_GET(group->progress_timer_state) ==
		    CSG_PROGRESS_TIMER_STATE_COMPUTE)
			kbdev->csf.compute_progress_timeout_cc = kbase_backend_get_cycle_cnt(kbdev);
	}

	/* Ignore fragment timeout if it is following a compute timeout.
	 * Otherwise, terminate the command stream group.
	 */
	for_each_set_bit(csg_nr, slot_mask, max_csg_slots) {
		group = kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;

		/* Check if it is a fragment timeout right after another compute timeout.
		 * In such case, kill compute CSG and give fragment CSG a second chance
		 */
		if (CSG_PROGRESS_TIMER_STATE_GET(group->progress_timer_state) ==
		    CSG_PROGRESS_TIMER_STATE_FRAGMENT) {
			u64 cycle_counter = kbase_backend_get_cycle_cnt(kbdev);
			u64 compute_progress_timeout_cc = kbdev->csf.compute_progress_timeout_cc;

			if (compute_progress_timeout_cc <= cycle_counter &&
			    cycle_counter <= compute_progress_timeout_cc +
						     MAX_PROGRESS_TIMEOUT_EVENT_DELAY) {
				dev_info(
					kbdev->dev,
					"Ignored Fragment iterator timeout for group %d on slot %d",
					group->handle, group->csg_nr);
				continue;
			}
		}

		kbase_debug_csf_fault_notify(group->kctx->kbdev, group->kctx,
					     DF_PROGRESS_TIMER_TIMEOUT);
		queue_work(group->kctx->csf.wq, &group->timer_event_work);
	}
}

/**
 * alloc_grp_protected_suspend_buffer_pages() -  Allocate physical pages from the protected
 *                                               memory for the protected mode suspend buffer.
 * @group: Pointer to the GPU queue group.
 *
 * Return: 0 if suspend buffer allocation is successful or if its already allocated, otherwise
 * negative error value.
 */
static int alloc_grp_protected_suspend_buffer_pages(struct kbase_queue_group *const group)
{
	struct kbase_device *const kbdev = group->kctx->kbdev;
	struct kbase_context *kctx = group->kctx;
	struct tagged_addr *phys = NULL;
	struct kbase_protected_suspend_buffer *sbuf = &group->protected_suspend_buf;
	size_t nr_pages;
	int err = 0;

	if (likely(sbuf->pma))
		return 0;

	nr_pages = PFN_UP(kbdev->csf.global_iface.groups[0].suspend_size);
	phys = kcalloc(nr_pages, sizeof(*phys), GFP_KERNEL);
	if (unlikely(!phys)) {
		err = -ENOMEM;
		goto phys_free;
	}

	mutex_lock(&kctx->csf.lock);
	kbase_csf_scheduler_lock(kbdev);

	if (unlikely(!group->csg_reg)) {
		/* The only chance of the bound csg_reg is removed from the group is
		 * that it has been put off slot by the scheduler and the csg_reg resource
		 * is contended by other groups. In this case, it needs another occasion for
		 * mapping the pma, which needs a bound csg_reg. Since the group is already
		 * off-slot, returning no error is harmless as the scheduler, when place the
		 * group back on-slot again would do the required MMU map operation on the
		 * allocated and retained pma.
		 */
		WARN_ON(group->csg_nr >= 0);
		dev_dbg(kbdev->dev, "No bound csg_reg for group_%d_%d_%d to enter protected mode",
			group->kctx->tgid, group->kctx->id, group->handle);
		goto unlock;
	}

	/* Allocate the protected mode pages */
	sbuf->pma = kbase_csf_protected_memory_alloc(kbdev, phys, nr_pages, true);
	if (unlikely(!sbuf->pma)) {
		err = -ENOMEM;
		goto unlock;
	}

	/* Map the bound susp_reg to the just allocated pma pages */
	err = kbase_csf_mcu_shared_group_update_pmode_map(kbdev, group);

unlock:
	kbase_csf_scheduler_unlock(kbdev);
	mutex_unlock(&kctx->csf.lock);
phys_free:
	kfree(phys);
	return err;
}

static void report_group_fatal_error(struct kbase_queue_group *const group)
{
	struct base_gpu_queue_group_error const
		err_payload = { .error_type = BASE_GPU_QUEUE_GROUP_ERROR_FATAL,
				.payload = { .fatal_group = {
						     .status = GPU_EXCEPTION_TYPE_SW_FAULT_0,
					     } } };

	kbase_csf_add_group_fatal_error(group, &err_payload);
	kbase_event_wakeup(group->kctx);
}

/**
 * handle_fault_event() - Handler for CS fault.
 *
 * @queue:  Pointer to queue for which fault event was received.
 * @group_id: GPU CSG-index the faulted-queue located at.
 * @cs_ack: Value of the CS_ACK register in the CS kernel input page used for
 *          the queue.
 *
 * Print required information about the CS fault and notify the user space client
 * about the fault.
 */
static void handle_fault_event(struct kbase_queue *const queue, u32 group_id, const u32 cs_ack)
{
	struct kbase_device *const kbdev = queue->kctx->kbdev;
	u32 stream_id = queue->csi_index;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	kbase_csf_report_cs_fault_info(queue, group_id, true);


	/* If dump-on-fault daemon is waiting for a fault, wake up the daemon.
	 * Acknowledging the fault is deferred to the bottom-half until the wait
	 * of the dump completion is done.
	 *
	 * Otherwise acknowledge the fault and ring the doorbell for the faulty queue
	 * to enter into recoverable state.
	 */
	if (likely(!kbase_debug_csf_fault_notify(kbdev, queue->kctx, DF_CS_FAULT))) {
		kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, group_id, stream_id, CS_REQ,
						  cs_ack, CS_REQ_FAULT_MASK);
		kbase_csf_ring_cs_kernel_doorbell(kbdev, queue->csi_index, group_id, true);
		queue->cs_error_acked = true;
	}

	if (!queue_work(queue->kctx->csf.wq, &queue->cs_error_work))
		dev_dbg(kbdev->dev, "%s: failed to enqueue a work", __func__);
}

static void report_queue_error(struct kbase_queue *const queue, u32 cs_error, u64 cs_error_info,
			       struct kbase_queue_group *group, bool fatal)
{
	struct base_csf_notification error = { .type = BASE_CSF_NOTIFICATION_GPU_QUEUE_GROUP_ERROR };

	if (!queue)
		return;

	if (WARN_ON_ONCE(!group))
		return;

	error.payload.csg_error.handle = group->handle;
	if (fatal) {
		error.payload.csg_error.error.error_type = BASE_GPU_QUEUE_GROUP_QUEUE_ERROR_FATAL;
		error.payload.csg_error.error.payload.fatal_queue.sideband = cs_error_info;
		error.payload.csg_error.error.payload.fatal_queue.status = cs_error;
		error.payload.csg_error.error.payload.fatal_queue.csi_index = queue->csi_index;
		if (queue->cs_error_has_trace) {
			error.payload.csg_error.error.payload.fatal_queue.trace_id0 =
				queue->cs_error_trace_id0;
			error.payload.csg_error.error.payload.fatal_queue.trace_id1 =
				queue->cs_error_trace_id1;
			error.payload.csg_error.error.payload.fatal_queue.trace_task =
				queue->cs_error_trace_task;
		}
		error.payload.csg_error.error.payload.fatal_queue.has_extra =
			queue->cs_error_has_trace ? 1 : 0;
	} else {
		error.payload.csg_error.error.error_type = BASE_GPU_QUEUE_GROUP_QUEUE_ERROR_FAULT;
		error.payload.csg_error.error.payload.fault_queue.sideband = cs_error_info;
		error.payload.csg_error.error.payload.fault_queue.status = cs_error;
		error.payload.csg_error.error.payload.fault_queue.csi_index = queue->csi_index;
		if (queue->cs_error_has_trace) {
			error.payload.csg_error.error.payload.fault_queue.trace_id0 =
				queue->cs_error_trace_id0;
			error.payload.csg_error.error.payload.fault_queue.trace_id1 =
				queue->cs_error_trace_id1;
			error.payload.csg_error.error.payload.fault_queue.trace_task =
				queue->cs_error_trace_task;
		}
		error.payload.csg_error.error.payload.fault_queue.has_extra =
			queue->cs_error_has_trace ? 1 : 0;
	}
	kbase_csf_event_add_error(queue->kctx, &group->error_fatal, &error);
	kbase_event_wakeup(queue->kctx);

	if (!fatal)
		queue->clear_faults = false;
}

/**
 * cs_error_worker() - Handle the CS_FATAL/CS_FAULT error for the GPU queue
 *
 * @data: Pointer to a work_struct embedded in GPU command queue.
 *
 * Terminate the CSG for CS_FATAL and report the error to userspace.
 */
static void cs_error_worker(struct work_struct *const data)
{
	struct kbase_queue *const queue = container_of(data, struct kbase_queue, cs_error_work);
	const u32 cs_fatal_exception_type = CS_FATAL_EXCEPTION_TYPE_GET(queue->cs_error);
	struct kbase_context *const kctx = queue->kctx;
	struct kbase_device *const kbdev = kctx->kbdev;
	struct kbase_queue_group *group;
	bool reset_prevented = false;
	int err;
	const bool cs_fatal = queue->cs_error_fatal;

	kbase_debug_csf_fault_wait_completion(kbdev);
	err = kbase_reset_gpu_prevent_and_wait(kbdev);

	if (err)
		dev_warn(
			kbdev->dev,
			"Unsuccessful GPU reset detected when terminating group to handle fatal event, attempting to terminate regardless");
	else
		reset_prevented = true;

	mutex_lock(&kctx->csf.lock);

	group = get_bound_queue_group(queue);
	if (!group) {
		dev_warn(kbdev->dev, "queue not bound when handling an error event");
		goto unlock;
	}

	if (!cs_fatal) {
		if (group->cs_fault_report_enable && queue->clear_faults)
			report_queue_error(queue, queue->cs_error, queue->cs_error_info, group,
					   false);
		if (unlikely(!queue->cs_error_acked)) {
			unsigned long flags, fw_io_flags;
			int slot_num;

			kbase_csf_scheduler_spin_lock(kbdev, &flags);
			if (kbase_csf_fw_io_open(&kbdev->csf.fw_io, &fw_io_flags)) {
				kbase_csf_scheduler_spin_unlock(kbdev, flags);
				goto unlock;
			}
			slot_num = kbase_csf_scheduler_group_get_slot_locked(group);
			if (likely(slot_num >= 0)) {
				u32 const cs_ack = kbase_csf_fw_io_stream_read(
					&kbdev->csf.fw_io, slot_num, queue->csi_index, CS_ACK);
				u32 const cs_req = kbase_csf_fw_io_stream_input_read(
					&kbdev->csf.fw_io, slot_num, queue->csi_index, CS_REQ);

				/* Acknowledge the fault and ring the doorbell for the queue
				 * if it hasn't yet done.
				 */
				if ((cs_ack & CS_ACK_FAULT_MASK) != (cs_req & CS_REQ_FAULT_MASK)) {
					kbase_csf_fw_io_stream_write_mask(
						&kbdev->csf.fw_io, slot_num, queue->csi_index,
						CS_REQ, cs_ack, CS_REQ_FAULT_MASK);
					kbase_csf_ring_cs_kernel_doorbell(kbdev, queue->csi_index,
									  slot_num, true);
				}
			}
			kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);
			kbase_csf_scheduler_spin_unlock(kbdev, flags);
		}
	} else {
		term_queue_group(group);
		flush_gpu_cache_on_fatal_error(kbdev);
		/* For an invalid GPU page fault, CS_BUS_FAULT fatal error is expected after the
		 * page fault handler disables the AS of faulty context. Need to skip reporting the
		 * CS_BUS_FAULT fatal error to the Userspace as it doesn't have the full fault info.
		 * Page fault handler will report the fatal error with full page fault info.
		 */
		if ((cs_fatal_exception_type == CS_FATAL_EXCEPTION_TYPE_CS_BUS_FAULT) &&
		    group->faulted) {
			dev_dbg(kbdev->dev,
				"Skipped reporting CS_BUS_FAULT for queue %d of group %d of ctx %d_%d",
				queue->csi_index, group->handle, kctx->tgid, kctx->id);
		} else {
			report_queue_error(queue, queue->cs_error, queue->cs_error_info, group,
					   true);
		}
	}

unlock:
	mutex_unlock(&kctx->csf.lock);
	if (reset_prevented)
		kbase_reset_gpu_allow(kbdev);
}

/**
 * handle_fatal_event() - Handler for CS fatal.
 *
 * @queue:    Pointer to queue for which fatal event was received.
 * @group_id: CSG index.
 * @cs_ack: Value of the CS_ACK register in the CS kernel input page used for
 *          the queue.
 *
 * Notify a waiting user space client of the CS fatal and prints meaningful
 * information.
 * Enqueue a work item to terminate the group and report the fatal error
 * to user space.
 */
static void handle_fatal_event(struct kbase_queue *const queue, u32 group_id, u32 cs_ack)
{
	struct kbase_device *const kbdev = queue->kctx->kbdev;
	u32 stream_id = queue->csi_index;
	const u32 cs_fatal_exception_type = kbase_csf_report_cs_fatal_info(queue, group_id, true);

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	/* Assert holding fw_io lock for write accesses */
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	kbase_debug_csf_fault_notify(kbdev, queue->kctx, DF_CS_FATAL);
	if (cs_fatal_exception_type == CS_FATAL_EXCEPTION_TYPE_CS_UNRECOVERABLE)
		if (kbase_prepare_to_reset_gpu(queue->kctx->kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu(queue->kctx->kbdev);

	if (cs_fatal_exception_type == CS_FATAL_EXCEPTION_TYPE_FIRMWARE_INTERNAL_ERROR) {
		if (dump_oops_in_dmesg)
			kbase_csf_firmware_log_dump_buffer(kbdev);
		if (dump_ktrace_in_dmesg)
			KBASE_KTRACE_RBUF_DUMP(kbdev);
	}

	queue_work(queue->kctx->csf.wq, &queue->cs_error_work);

	kbase_csf_fw_io_stream_write_mask(&kbdev->csf.fw_io, group_id, stream_id, CS_REQ, cs_ack,
					  CS_REQ_FATAL_MASK);

}

u32 kbase_csf_report_cs_fatal_info(struct kbase_queue *const queue, u32 slot_id, bool atomic_ctx)
{
	struct kbase_device *const kbdev = queue->kctx->kbdev;
	u32 stream_id = queue->csi_index;
	const u32 cs_fatal =
		kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id, stream_id, CS_FATAL);
	const u64 cs_fatal_info = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id, stream_id,
							      CS_FATAL_INFO_LO) |
				  ((u64)kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id,
								    stream_id, CS_FATAL_INFO_HI)
				   << 32);
	const u32 cs_fatal_exception_type = CS_FATAL_EXCEPTION_TYPE_GET(cs_fatal);
	const u32 cs_fatal_exception_data = CS_FATAL_EXCEPTION_DATA_GET(cs_fatal);
	const u64 cs_fatal_info_exception_data = CS_FATAL_INFO_EXCEPTION_DATA_GET(cs_fatal_info);
	bool has_trace_info = false;
	bool skip_fault_report = kbase_ctx_flag(queue->kctx, KCTX_PAGE_FAULT_REPORT_SKIP);

	u32 cs_fatal_trace_id0;
	u32 cs_fatal_trace_id1;
	u32 cs_fatal_trace_task;

	struct kbase_gpu_id_props *gpu_id = &kbdev->gpu_props.gpu_id;

	if ((gpu_id->arch_major > 14) || ((gpu_id->arch_major == 14) && (gpu_id->arch_rev >= 4)))
		has_trace_info = true;

	if (atomic_ctx)
		kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	else
		lockdep_assert_held(&kbdev->csf.scheduler.lock);

	if (has_trace_info) {
		cs_fatal_trace_id0 = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id,
								 stream_id, CS_FATAL_TRACE_ID0);
		cs_fatal_trace_id1 = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id,
								 stream_id, CS_FATAL_TRACE_ID1);
		cs_fatal_trace_task = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id,
								  stream_id, CS_FATAL_TRACE_TASK);
		queue->cs_error_trace_id0 = cs_fatal_trace_id0;
		queue->cs_error_trace_id1 = cs_fatal_trace_id1;
		queue->cs_error_trace_task = cs_fatal_trace_task;
		if (!skip_fault_report) {
			dev_warn(kbdev->dev,
				 "Ctx %d_%d Group %d CSG %d CSI: %d\n"
				 "CS_FATAL.EXCEPTION_TYPE: 0x%x (%s)\n"
				 "CS_FATAL.EXCEPTION_DATA: 0x%x\n"
				 "CS_FATAL_INFO.EXCEPTION_DATA: 0x%llx\n"
				 "CS_FATAL_TRACE_ID0.EXCEPTION_TRACE_ID0: 0x%x\n"
				 "CS_FATAL_TRACE_ID1.EXCEPTION_TRACE_ID1:  0x%x\n"
				 "CS_FATAL_TRACE_TASK.EXCEPTION_TRACE_TASK: 0x%x\n",
				 queue->kctx->tgid, queue->kctx->id, queue->group->handle,
				 queue->group->csg_nr, queue->csi_index, cs_fatal_exception_type,
				 kbase_gpu_exception_name(cs_fatal_exception_type),
				 cs_fatal_exception_data, cs_fatal_info_exception_data,
				 cs_fatal_trace_id0, cs_fatal_trace_id1, cs_fatal_trace_task);
		}
	}
	if (!has_trace_info && !skip_fault_report)
		dev_warn(kbdev->dev,
			 "Ctx %d_%d Group %d CSG %d CSI: %d\n"
			 "CS_FATAL.EXCEPTION_TYPE: 0x%x (%s)\n"
			 "CS_FATAL.EXCEPTION_DATA: 0x%x\n"
			 "CS_FATAL_INFO.EXCEPTION_DATA: 0x%llx\n",
			 queue->kctx->tgid, queue->kctx->id, queue->group->handle,
			 queue->group->csg_nr, queue->csi_index, cs_fatal_exception_type,
			 kbase_gpu_exception_name(cs_fatal_exception_type), cs_fatal_exception_data,
			 cs_fatal_info_exception_data);

	if (cs_fatal_exception_type == CS_FATAL_EXCEPTION_TYPE_CS_UNRECOVERABLE)
		queue->group->cs_unrecoverable = true;

	queue->cs_error = cs_fatal;
	queue->cs_error_info = cs_fatal_info;
	queue->cs_error_fatal = true;
	queue->cs_error_has_trace = has_trace_info;

	return cs_fatal_exception_type;
}

void kbase_csf_report_cs_fault_info(struct kbase_queue *const queue, u32 slot_id, bool atomic_ctx)
{
	struct kbase_device *const kbdev = queue->kctx->kbdev;
	u32 stream_id = queue->csi_index;
	const u32 cs_fault =
		kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id, stream_id, CS_FAULT);
	const u64 cs_fault_info = kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id, stream_id,
							      CS_FAULT_INFO_LO) |
				  ((u64)kbase_csf_fw_io_stream_read(&kbdev->csf.fw_io, slot_id,
								    stream_id, CS_FAULT_INFO_HI)
				   << 32);
	const u8 cs_fault_exception_type = CS_FAULT_EXCEPTION_TYPE_GET(cs_fault);
	const u32 cs_fault_exception_data = CS_FAULT_EXCEPTION_DATA_GET(cs_fault);
	const u64 cs_fault_info_exception_data = CS_FAULT_INFO_EXCEPTION_DATA_GET(cs_fault_info);
	bool has_trace_info = false;
	bool skip_fault_report = kbase_ctx_flag(queue->kctx, KCTX_PAGE_FAULT_REPORT_SKIP);


	if (atomic_ctx)
		kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	else
		lockdep_assert_held(&kbdev->csf.scheduler.lock);


	if (!has_trace_info && !skip_fault_report)
		dev_warn(kbdev->dev,
			 "Ctx %d_%d Group %d CSG %d CSI: %d\n"
			 "CS_FAULT.EXCEPTION_TYPE: 0x%x (%s)\n"
			 "CS_FAULT.EXCEPTION_DATA: 0x%x\n"
			 "CS_FAULT_INFO.EXCEPTION_DATA: 0x%llx\n",
			 queue->kctx->tgid, queue->kctx->id, queue->group->handle,
			 queue->group->csg_nr, queue->csi_index, cs_fault_exception_type,
			 kbase_gpu_exception_name(cs_fault_exception_type), cs_fault_exception_data,
			 cs_fault_info_exception_data);

	queue->cs_error = cs_fault;
	queue->cs_error_info = cs_fault_info;
	queue->cs_error_fatal = false;
	queue->cs_error_acked = false;
}

int kbase_csf_handle_pending_oom_interrupt(struct kbase_queue *const queue, u32 group_id)
{
	struct kbase_device *const kbdev = queue->kctx->kbdev;
	struct workqueue_struct *wq = queue->kctx->csf.wq;

	if (!kbase_csf_cs_get_pending_oom(kbdev, queue, group_id)) {
		if (!queue_work(wq, &queue->oom_event_work)) {
			/* The work item shall not have been already queued, there can be only
			 * one pending OoM event for a  queue.
			 */
			dev_warn(kbdev->dev,
				 "Tiler OOM work already queued: queue %d group %d (ctx %d_%d)",
				 queue->csi_index, group_id, queue->kctx->tgid, queue->kctx->id);
			return -EBUSY;
		}
	}
	return 0;
}
KBASE_EXPORT_TEST_API(kbase_csf_handle_pending_oom_interrupt);

/**
 * process_cs_interrupts() - Process interrupts for a CS.
 *
 * @group:  Pointer to GPU command queue group data.
 * @group_id: CSG index.
 * @irqreq: CSG's IRQ request bitmask (one bit per CS).
 * @irqack: CSG's IRQ acknowledge bitmask (one bit per CS).
 * @track: Pointer that tracks the highest scanout priority idle CSG
 *         and any newly potentially viable protected mode requesting
 *          CSG in current IRQ context.
 *
 * If the interrupt request bitmask differs from the acknowledge bitmask
 * then the firmware is notifying the host of an event concerning those
 * CSs indicated by bits whose value differs. The actions required
 * are then determined by examining which notification flags differ between
 * the request and acknowledge registers for the individual CS(s).
 *
 * Return: -ENODEV on unresponsive MCU, 0 otherwise.
 */
static int process_cs_interrupts(struct kbase_queue_group *const group, u32 group_id,
				 u32 const irqreq, u32 const irqack,
				 struct irq_idle_and_protm_track *track)
{
	struct kbase_device *const kbdev = group->kctx->kbdev;
	u32 remaining = irqreq ^ irqack;
	bool protm_pend = false;
	const bool group_suspending = !kbase_csf_scheduler_group_events_enabled(kbdev, group);
	unsigned long fw_io_flags;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	while (remaining != 0) {
		unsigned int const i = (unsigned int)ffs((int)remaining) - 1;
		struct kbase_queue *const queue = group->bound_queues[i];

		remaining &= ~(1U << i);

		/* The queue pointer can be NULL, but if it isn't NULL then it
		 * cannot disappear since scheduler spinlock is held and before
		 * freeing a bound queue it has to be first unbound which
		 * requires scheduler spinlock.
		 */
		if (queue && !WARN_ON(queue->csi_index != (s8)i)) {
			struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
			u32 stream_id = i;
			u32 const cs_req = kbase_csf_fw_io_stream_input_read(fw_io, group_id,
									     stream_id, CS_REQ);
			u32 const cs_ack =
				kbase_csf_fw_io_stream_read(fw_io, group_id, stream_id, CS_ACK);

			if ((cs_ack & CS_ACK_FATAL_MASK) != (cs_req & CS_REQ_FATAL_MASK)) {
				if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
					return -ENODEV;

				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_FAULT, group, queue,
							   cs_req ^ cs_ack);
				handle_fatal_event(queue, group_id, cs_ack);
				kbase_csf_fw_io_close(fw_io, fw_io_flags);
			}

			if ((cs_ack & CS_ACK_FAULT_MASK) != (cs_req & CS_REQ_FAULT_MASK)) {
				if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
					return -ENODEV;

				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_FAULT, group, queue,
							   cs_req ^ cs_ack);
				handle_fault_event(queue, group_id, cs_ack);
				kbase_csf_fw_io_close(fw_io, fw_io_flags);
			}

			/* PROTM_PEND and TILER_OOM can be safely ignored
			 * because they will be raised again if the group
			 * is assigned a CSG slot in future.
			 */
			if (group_suspending) {
				u32 const cs_req_remain = cs_req & ~CS_REQ_EXCEPTION_MASK;
				u32 const cs_ack_remain = cs_ack & ~CS_ACK_EXCEPTION_MASK;

				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev,
							   CSI_INTERRUPT_GROUP_SUSPENDS_IGNORED,
							   group, queue,
							   cs_req_remain ^ cs_ack_remain);
				continue;
			}

			if (((cs_req & CS_REQ_TILER_OOM_MASK) ^ (cs_ack & CS_ACK_TILER_OOM_MASK))) {
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_TILER_OOM, group,
							   queue, cs_req ^ cs_ack);

				kbase_csf_handle_pending_oom_interrupt(queue, group_id);
			}

			if ((cs_req & CS_REQ_PROTM_PEND_MASK) ^ (cs_ack & CS_ACK_PROTM_PEND_MASK)) {
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_INTERRUPT_PROTM_PEND, group,
							   queue, cs_req ^ cs_ack);

				dev_dbg(kbdev->dev,
					"Protected mode entry request for queue on csi %d bound to group-%d on slot %d",
					queue->csi_index, group->handle, group->csg_nr);

				bitmap_set(group->protm_pending_bitmap, i, 1);
				KBASE_KTRACE_ADD_CSF_GRP_Q(kbdev, CSI_PROTM_PEND_SET, group, queue,
							   group->protm_pending_bitmap[0]);
				protm_pend = true;
			}
		}
	}

	if (protm_pend) {
		struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

		if (scheduler->tick_protm_pending_seq > group->scan_seq_num) {
			scheduler->tick_protm_pending_seq = group->scan_seq_num;
			track->protm_grp = group;
		}

		if (!group->protected_suspend_buf.pma)
			kbase_csf_scheduler_enqueue_protm_event_work(group);

		if (test_bit(group->csg_nr, scheduler->csg_slots_idle_mask)) {
			clear_bit(group->csg_nr, scheduler->csg_slots_idle_mask);
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_CLEAR, group,
						 scheduler->csg_slots_idle_mask[0]);
			dev_dbg(kbdev->dev, "Group-%d on slot %d de-idled by protm request",
				group->handle, group->csg_nr);
		}
	}

	return 0;
}

/**
 * process_csg_interrupts() - Process interrupts for a CSG.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @csg_nr: CSG number.
 * @track: Pointer that tracks the highest idle CSG and the newly possible viable
 *         protected mode requesting group, in current IRQ context.
 * @progress_timeout_slot_mask: slot mask to indicate on which slot progress timeout
 *         happens.
 *
 * Handles interrupts for a CSG and for CSs within it.
 *
 * If the CSG's request register value differs from its acknowledge register
 * then the firmware is notifying the host of an event concerning the whole
 * group. The actions required are then determined by examining which
 * notification flags differ between those two register values.
 *
 * See process_cs_interrupts() for details of per-stream interrupt handling.
 *
 * Return: -ENODEV on unresponsive MCU, 0 otherwise.
 */
static int process_csg_interrupts(struct kbase_device *const kbdev, u32 const csg_nr,
				  struct irq_idle_and_protm_track *track,
				  unsigned long *progress_timeout_slot_mask)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	struct kbase_queue_group *group = NULL;
	u32 req, ack, irqreq, irqack;
	int err;
	unsigned long fw_io_flags;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (WARN_ON(csg_nr >= kbdev->csf.global_iface.group_num))
		return 0;

	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		return -ENODEV;

	req = kbase_csf_fw_io_group_input_read(fw_io, csg_nr, CSG_REQ);
	ack = kbase_csf_fw_io_group_read(fw_io, csg_nr, CSG_ACK);
	irqreq = kbase_csf_fw_io_group_read(fw_io, csg_nr, CSG_IRQ_REQ);
	irqack = kbase_csf_fw_io_group_input_read(fw_io, csg_nr, CSG_IRQ_ACK);

	/* There may not be any pending CSG/CS interrupts to process */
	if ((req == ack) && (irqreq == irqack)) {
		kbase_csf_fw_io_close(fw_io, fw_io_flags);
		return 0;
	}

	/* Immediately set IRQ_ACK bits to be same as the IRQ_REQ bits before
	 * examining the CS_ACK & CS_REQ bits. This would ensure that Host
	 * doesn't misses an interrupt for the CS in the race scenario where
	 * whilst Host is servicing an interrupt for the CS, firmware sends
	 * another interrupt for that CS.
	 */
	kbase_csf_fw_io_group_write(fw_io, csg_nr, CSG_IRQ_ACK, irqreq);

	group = kbase_csf_scheduler_get_group_on_slot(kbdev, csg_nr);

	/* The group pointer can be NULL here if interrupts for the group
	 * (like SYNC_UPDATE, IDLE notification) were delayed and arrived
	 * just after the suspension of group completed. However if not NULL
	 * then the group pointer cannot disappear even if User tries to
	 * terminate the group whilst this loop is running as scheduler
	 * spinlock is held and for freeing a group that is resident on a CSG
	 * slot scheduler spinlock is required.
	 */
	if (!group) {
		kbase_csf_fw_io_close(fw_io, fw_io_flags);
		return 0;
	}

	if (WARN_ON((u32)kbase_csf_scheduler_group_get_slot_locked(group) != csg_nr)) {
		kbase_csf_fw_io_close(fw_io, fw_io_flags);
		return 0;
	}

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_PROCESS_START, group, (u64)csg_nr);

	kbase_csf_handle_csg_sync_update(kbdev, csg_nr, group, req, ack);
	kbase_csf_fw_io_close(fw_io, fw_io_flags);

	if ((req ^ ack) & CSG_REQ_IDLE_MASK) {
		struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;

		if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
			return -ENODEV;

		KBASE_TLSTREAM_TL_KBASE_DEVICE_CSG_IDLE(kbdev, kbdev->id, csg_nr);

		kbase_csf_fw_io_group_write_mask(fw_io, csg_nr, CSG_REQ, ack, CSG_REQ_IDLE_MASK);
		kbase_csf_fw_io_close(fw_io, fw_io_flags);

		set_bit(csg_nr, scheduler->csg_slots_idle_mask);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_SLOT_IDLE_SET, group,
					 scheduler->csg_slots_idle_mask[0]);
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_IDLE, group, req ^ ack);
		dev_dbg(kbdev->dev, "Idle notification received for Group %u on slot %u\n",
			group->handle, csg_nr);

		if (atomic_read(&scheduler->non_idle_offslot_grps)) {
			/* If there are non-idle CSGs waiting for a slot, fire
			 * a tock for a replacement.
			 */
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_NON_IDLE_GROUPS, group,
						 req ^ ack);
			kbase_csf_scheduler_invoke_tock(kbdev);
		} else {
			KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_NO_NON_IDLE_GROUPS, group,
						 req ^ ack);
		}

		if (group->scan_seq_num < track->idle_seq) {
			track->idle_seq = group->scan_seq_num;
			track->idle_slot = (s8)csg_nr;
		}
	}

	if ((req ^ ack) & CSG_REQ_PROGRESS_TIMER_EVENT_MASK) {
		if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
			return -ENODEV;

		kbase_csf_fw_io_group_write_mask(fw_io, csg_nr, CSG_REQ, ack,
						 CSG_REQ_PROGRESS_TIMER_EVENT_MASK);
		kbase_csf_fw_io_close(fw_io, fw_io_flags);

		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_PROGRESS_TIMER_EVENT, group,
					 req ^ ack);

		set_bit(csg_nr, progress_timeout_slot_mask);

	}

	err = process_cs_interrupts(group, csg_nr, irqreq, irqack, track);

	KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_PROCESS_END, group,
				 ((u64)req ^ ack) | (((u64)irqreq ^ irqack) << 32));
	return err;
}

/**
 * process_prfcnt_interrupts() - Process performance counter interrupts.
 *
 * @kbdev:   Instance of a GPU platform device that implements a CSF interface.
 * @glb_req: Global request register value.
 * @glb_ack: Global acknowledge register value.
 *
 * Handles interrupts issued by the firmware that relate to the performance
 * counters. For example, on completion of a performance counter sample. It is
 * expected that the scheduler spinlock is already held on calling this
 * function.
 *
 * Return: -ENODEV on unresponsive MCU, 0 otherwise.
 */
static int process_prfcnt_interrupts(struct kbase_device *kbdev, u32 glb_req, u32 glb_ack)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	unsigned long fw_io_flags;

	lockdep_assert_held(&kbdev->csf.scheduler.interrupt_lock);

	/* Process PRFCNT_SAMPLE interrupt. */
	if (kbdev->csf.hwcnt.request_pending &&
	    ((glb_req & GLB_REQ_PRFCNT_SAMPLE_MASK) == (glb_ack & GLB_REQ_PRFCNT_SAMPLE_MASK))) {
		kbdev->csf.hwcnt.request_pending = false;

		dev_dbg(kbdev->dev, "PRFCNT_SAMPLE done interrupt received.");

		kbase_hwcnt_backend_csf_on_prfcnt_sample(&kbdev->hwcnt_gpu_iface);
	}

	/* Process PRFCNT_ENABLE interrupt. */
	if (kbdev->csf.hwcnt.enable_pending &&
	    ((glb_req & GLB_REQ_PRFCNT_ENABLE_MASK) == (glb_ack & GLB_REQ_PRFCNT_ENABLE_MASK))) {
		kbdev->csf.hwcnt.enable_pending = false;

		dev_dbg(kbdev->dev, "PRFCNT_ENABLE status changed interrupt received.");

		if (glb_ack & GLB_REQ_PRFCNT_ENABLE_MASK)
			kbase_hwcnt_backend_csf_on_prfcnt_enable(&kbdev->hwcnt_gpu_iface);
		else
			kbase_hwcnt_backend_csf_on_prfcnt_disable(&kbdev->hwcnt_gpu_iface);
	}

	/* Process PRFCNT_THRESHOLD interrupt. */
	if ((glb_req ^ glb_ack) & GLB_REQ_PRFCNT_THRESHOLD_MASK) {
		dev_dbg(kbdev->dev, "PRFCNT_THRESHOLD interrupt received.");
		if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
			dev_dbg(kbdev->dev,
				"Skipping PRFCNT_THRESHOLD interrupt handling due to unresponsive MCU.");
			return -ENODEV;
		}

		kbase_hwcnt_backend_csf_on_prfcnt_threshold(&kbdev->hwcnt_gpu_iface);

		/* Set the GLB_REQ.PRFCNT_THRESHOLD flag back to
		 * the same value as GLB_ACK.PRFCNT_THRESHOLD
		 * flag in order to enable reporting of another
		 * PRFCNT_THRESHOLD event.
		 */
		kbase_csf_fw_io_global_write_mask(fw_io, GLB_REQ, glb_ack,
						  GLB_REQ_PRFCNT_THRESHOLD_MASK);
		kbase_csf_fw_io_close(fw_io, fw_io_flags);
	}

	/* Process PRFCNT_OVERFLOW interrupt. */
	if ((glb_req ^ glb_ack) & GLB_REQ_PRFCNT_OVERFLOW_MASK) {
		dev_dbg(kbdev->dev, "PRFCNT_OVERFLOW interrupt received.");
		if (kbase_csf_fw_io_open(fw_io, &fw_io_flags)) {
			dev_dbg(kbdev->dev,
				"Skipping PRFCNT_OVERFLOW interrupt handling due to unresponsive MCU.");
			return -ENODEV;
		}

		kbase_hwcnt_backend_csf_on_prfcnt_overflow(&kbdev->hwcnt_gpu_iface);

		/* Set the GLB_REQ.PRFCNT_OVERFLOW flag back to
		 * the same value as GLB_ACK.PRFCNT_OVERFLOW
		 * flag in order to enable reporting of another
		 * PRFCNT_OVERFLOW event.
		 */
		kbase_csf_fw_io_global_write_mask(fw_io, GLB_REQ, glb_ack,
						  GLB_REQ_PRFCNT_OVERFLOW_MASK);
		kbase_csf_fw_io_close(fw_io, fw_io_flags);
	}

	return 0;
}

/**
 * check_protm_enter_req_complete() - Check if PROTM_ENTER request completed
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @glb_req: Global request register value.
 * @glb_ack: Global acknowledge register value.
 *
 * This function checks if the PROTM_ENTER Global request had completed and
 * appropriately sends notification about the protected mode entry to components
 * like IPA, HWC, IPA_CONTROL.
 */
static inline void check_protm_enter_req_complete(struct kbase_device *kbdev, u32 glb_req,
						  u32 glb_ack)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (likely(!kbdev->csf.scheduler.active_protm_grp))
		return;

	if (kbdev->protected_mode)
		return;

	if ((glb_req & GLB_REQ_PROTM_ENTER_MASK) != (glb_ack & GLB_REQ_PROTM_ENTER_MASK))
		return;

	dev_dbg(kbdev->dev, "Protected mode entry interrupt received");

	kbdev->protected_mode = true;

	kbase_ipa_protection_mode_switch_event(kbdev);
	kbase_ipa_control_protm_entered(kbdev);
	kbase_hwcnt_backend_csf_protm_entered(&kbdev->hwcnt_gpu_iface);
}

/**
 * process_protm_exit() - Handle the protected mode exit interrupt
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @glb_ack: Global acknowledge register value.
 *
 * This function handles the PROTM_EXIT interrupt and sends notification
 * about the protected mode exit to components like HWC, IPA_CONTROL.
 *
 * Return: -ENODEV on unresponsive MCU, 0 otherwise.
 */
static inline int process_protm_exit(struct kbase_device *kbdev, u32 glb_ack)
{
	struct kbase_csf_fw_io *fw_io = &kbdev->csf.fw_io;
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	unsigned long fw_io_flags;

	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	dev_dbg(kbdev->dev, "Protected mode exit interrupt received");

	if (kbase_csf_fw_io_open(fw_io, &fw_io_flags))
		return -ENODEV;

	kbase_csf_fw_io_global_write_mask(&kbdev->csf.fw_io, GLB_REQ, glb_ack,
					  GLB_REQ_PROTM_EXIT_MASK);

	kbase_csf_fw_io_close(&kbdev->csf.fw_io, fw_io_flags);

	if (likely(scheduler->active_protm_grp)) {
		KBASE_KTRACE_ADD_CSF_GRP(kbdev, SCHEDULER_PROTM_EXIT, scheduler->active_protm_grp,
					 0u);
		scheduler->active_protm_grp = NULL;
	} else {
		dev_warn(kbdev->dev, "PROTM_EXIT interrupt after no pmode group");
	}

	if (!WARN_ON(!kbdev->protected_mode)) {
		kbdev->protected_mode = false;
		kbase_ipa_control_protm_exited(kbdev);
		kbase_hwcnt_backend_csf_protm_exited(&kbdev->hwcnt_gpu_iface);
	}

#if IS_ENABLED(CONFIG_MALI_CORESIGHT)
	kbase_debug_coresight_csf_enable_pmode_exit(kbdev);
#endif /* IS_ENABLED(CONFIG_MALI_CORESIGHT) */

	return 0;
}

static inline void process_tracked_info_for_protm(struct kbase_device *kbdev,
						  struct irq_idle_and_protm_track *track)
{
	struct kbase_csf_scheduler *scheduler = &kbdev->csf.scheduler;
	struct kbase_queue_group *group = track->protm_grp;
	u32 current_protm_pending_seq = scheduler->tick_protm_pending_seq;

	kbase_csf_scheduler_spin_lock_assert_held(kbdev);

	if (likely(current_protm_pending_seq == KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID))
		return;

	/* Handle protm from the tracked information */
	if (track->idle_seq < current_protm_pending_seq) {
		/* If the protm enter was prevented due to groups priority, then fire a tock
		 * for the scheduler to re-examine the case.
		 */
		dev_dbg(kbdev->dev, "Attempt pending protm from idle slot %d\n", track->idle_slot);
		kbase_csf_scheduler_invoke_tock(kbdev);
	} else if (group) {
		u32 i, num_groups = kbdev->csf.global_iface.group_num;
		struct kbase_queue_group *grp;
		bool tock_triggered = false;

		/* A new protm request, and track->idle_seq is not sufficient, check across
		 * previously notified idle CSGs in the current tick/tock cycle.
		 */
		for_each_set_bit(i, scheduler->csg_slots_idle_mask, num_groups) {
			if (i == (u32)track->idle_slot)
				continue;
			grp = kbase_csf_scheduler_get_group_on_slot(kbdev, i);
			/* If not NULL then the group pointer cannot disappear as the
			 * scheduler spinlock is held.
			 */
			if (grp == NULL)
				continue;

			if (grp->scan_seq_num < current_protm_pending_seq) {
				tock_triggered = true;
				dev_dbg(kbdev->dev,
					"Attempt new protm from tick/tock idle slot %d\n", i);
				kbase_csf_scheduler_invoke_tock(kbdev);
				break;
			}
		}

		if (!tock_triggered) {
			dev_dbg(kbdev->dev, "Group-%d on slot-%d start protm work\n", group->handle,
				group->csg_nr);
			kbase_csf_scheduler_enqueue_protm_event_work(group);
		}
	}
}

static void order_job_irq_clear_with_iface_mem_read(void)
{
	/* Ensure that write to the JOB_IRQ_CLEAR is ordered with regards to the
	 * read from interface memory. The ordering is needed considering the way
	 * FW & Kbase writes to the JOB_IRQ_RAWSTAT and JOB_IRQ_CLEAR registers
	 * without any synchronization. Without the barrier there is no guarantee
	 * about the ordering, the write to IRQ_CLEAR can take effect after the read
	 * from interface memory and that could cause a problem for the scenario where
	 * FW sends back to back notifications for the same CSG for events like
	 * SYNC_UPDATE and IDLE, but Kbase gets a single IRQ and observes only the
	 * first event. Similar thing can happen with glb events like CFG_ALLOC_EN
	 * acknowledgment and GPU idle notification.
	 *
	 *       MCU                                    CPU
	 *  ---------------                         ----------------
	 *  Update interface memory                 Write to IRQ_CLEAR to clear current IRQ
	 *  <barrier>                               <barrier>
	 *  Write to IRQ_RAWSTAT to raise new IRQ   Read interface memory
	 */

	/* CPU and GPU would be in the same Outer shareable domain */
	dmb(osh);
}

static const char *const glb_fatal_status_errors[GLB_FATAL_STATUS_VALUE_COUNT] = {
	[GLB_FATAL_STATUS_VALUE_OK] = "OK",
	[GLB_FATAL_STATUS_VALUE_ASSERT] = "Firmware assert triggered",
	[GLB_FATAL_STATUS_VALUE_UNEXPECTED_EXCEPTION] =
		"Hardware raised an exception firmware did not expect",
	[GLB_FATAL_STATUS_VALUE_HANG] = "Firmware hangs and watchdog timer expired",
	[GLB_FATAL_STATUS_VALUE_POWER_DELEGATION] =
		"Host did not delegate control over the required power domains",
	[GLB_FATAL_STATUS_VALUE_UNEXPECTED_REQUEST] = "Unexpected GLB_REQ request",
	[GLB_FATAL_STATUS_VALUE_CORE_MASK] = "No cores available",
	[GLB_FATAL_STATUS_VALUE_DMAC_FAILURE] = "DMA controller failure",
};

/**
 * handle_glb_fatal_event() - Handle the GLB fatal event
 *
 * @kbdev:        Instance of GPU device.
 * @global_iface: CSF global interface
 */
static void handle_glb_fatal_event(struct kbase_device *kbdev,
				   const struct kbase_csf_global_iface *const global_iface)
{
	const char *error_string = NULL;
	const u32 fatal_status = kbase_csf_fw_io_global_read(&kbdev->csf.fw_io, GLB_FATAL_STATUS);

	lockdep_assert_held(&kbdev->hwaccess_lock);
	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	dev_warn(kbdev->dev, "MCU encountered unrecoverable error");

	if (fatal_status < GLB_FATAL_STATUS_VALUE_COUNT)
		error_string = glb_fatal_status_errors[fatal_status];
	else {
		dev_err(kbdev->dev, "Invalid GLB_FATAL_STATUS (%u)", fatal_status);
		return;
	}

	if (fatal_status == GLB_FATAL_STATUS_VALUE_OK)
		dev_err(kbdev->dev, "GLB_FATAL_STATUS(OK) must be set with proper reason");
	else {
		/* Retrieve GLB_FATAL_INFO from GLB output */
		const u64 fatal_info =
			((u64)kbase_csf_fw_io_global_read(&kbdev->csf.fw_io, GLB_FATAL_INFO_HI)
			 << 32) |
			(u64)kbase_csf_fw_io_global_read(&kbdev->csf.fw_io, GLB_FATAL_INFO_LO);

		dev_warn(kbdev->dev, "GLB_FATAL_STATUS: %s", error_string);
		dev_warn(kbdev->dev, "GLB_FATAL_INFO: 0x%llx", fatal_info);
		queue_work(system_wq, &kbdev->csf.glb_fatal_work);
	}
}

void kbase_csf_interrupt(struct kbase_device *kbdev, u32 val)
{
	bool deferred_handling_glb_idle_irq = false;
	int err;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	KBASE_KTRACE_ADD(kbdev, CSF_INTERRUPT_START, NULL, val);

	do {
		unsigned long flags, fw_io_flags;
		u32 csg_interrupts = val & ~JOB_IRQ_GLOBAL_IF;
		bool glb_idle_irq_received = false;

		kbase_reg_write32(kbdev, JOB_CONTROL_ENUM(JOB_IRQ_CLEAR), val);
		order_job_irq_clear_with_iface_mem_read();

		if (csg_interrupts != 0) {
			struct irq_idle_and_protm_track track = { .protm_grp = NULL,
								  .idle_seq = U32_MAX,
								  .idle_slot = S8_MAX };
			DECLARE_BITMAP(progress_timeout_csgs, BASEP_QUEUE_GROUP_MAX) = { 0 };

			kbase_csf_scheduler_spin_lock(kbdev, &flags);

			/* Looping through and track the highest idle and protm groups.
			 * Also track the groups for which progress timer timeout happened.
			 */
			while (csg_interrupts != 0) {
				u32 const csg_nr = (u32)ffs((int)csg_interrupts) - 1;

				err = process_csg_interrupts(kbdev, csg_nr, &track,
							     progress_timeout_csgs);
				/* Stop processing interrupts in case of an unresponsive MCU */
				if (err) {
					kbase_csf_scheduler_spin_unlock(kbdev, flags);
					goto exit;
				}

				csg_interrupts &= ~(1U << csg_nr);
			}
			/* Handle protm from the tracked information */
			process_tracked_info_for_protm(kbdev, &track);
			/* Handle pending progress timeout(s) */
			handle_progress_timer_events(kbdev, progress_timeout_csgs);

			kbase_csf_scheduler_spin_unlock(kbdev, flags);
		}

		if (val & JOB_IRQ_GLOBAL_IF) {
			const struct kbase_csf_global_iface *const global_iface =
				&kbdev->csf.global_iface;

			kbdev->csf.interrupt_received = true;

			if (!kbdev->csf.firmware_reloaded)
				kbase_csf_firmware_reload_completed(kbdev);
			else if (kbdev->csf.fw_io.pages.output) {
				u32 glb_req, glb_ack;

				kbase_csf_scheduler_spin_lock(kbdev, &flags);

				glb_req = kbase_csf_fw_io_global_input_read(&kbdev->csf.fw_io,
									    GLB_REQ);
				glb_ack = kbase_csf_fw_io_global_read(&kbdev->csf.fw_io, GLB_ACK);
				KBASE_KTRACE_ADD(kbdev, CSF_INTERRUPT_GLB_REQ_ACK, NULL,
						 glb_req ^ glb_ack);

				check_protm_enter_req_complete(kbdev, glb_req, glb_ack);

				if ((glb_req ^ glb_ack) & GLB_REQ_PROTM_EXIT_MASK) {
					/* Stop processing interrupts in case of
					 * an unresponsive MCU.
					 */
					err = process_protm_exit(kbdev, glb_ack);
					if (err) {
						kbase_csf_scheduler_spin_unlock(kbdev, flags);
						goto exit;
					}
				}
				/* Handle IDLE Hysteresis notification event */
				if ((glb_req ^ glb_ack) & GLB_REQ_IDLE_EVENT_MASK) {
					{
						dev_dbg(kbdev->dev,
							"Idle-hysteresis event flagged");

						if (kbase_csf_fw_io_open(&kbdev->csf.fw_io,
									 &fw_io_flags)) {
							kbase_csf_scheduler_spin_unlock(kbdev,
											flags);
							goto exit;
						}
						kbase_csf_fw_io_global_write_mask(
							&kbdev->csf.fw_io, GLB_REQ, glb_ack,
							GLB_REQ_IDLE_EVENT_MASK);
						kbase_csf_fw_io_close(&kbdev->csf.fw_io,
								      fw_io_flags);

						if (atomic_read(
							    &kbdev->csf.scheduler.fw_soi_enabled)) {
							/* The FW is going to sleep, we shall:
							 * - Enable fast GPU idle handling to avoid
							 *   confirming CSGs status in gpu_idle_worker().
							 * - Enable doorbell mirroring to minimise the
							 *   chance of KBase raising kernel doorbells which
							 *   would cause the FW to be woken up.
							 */
							kbdev->csf.scheduler.fast_gpu_idle_handling =
								true;
							kbase_pm_enable_db_mirror_interrupt(kbdev);
						}

						glb_idle_irq_received = true;
						/* Defer handling this IRQ to account for a race condition
						 * where the idle worker could be executed before we have
						 * finished handling all pending IRQs (including CSG IDLE
						 * IRQs).
						 */
						deferred_handling_glb_idle_irq = true;
					}
				}

				if (glb_ack & GLB_ACK_FATAL_MASK)
					handle_glb_fatal_event(kbdev, global_iface);

				/* Stop processing interrupts in case of an unresponsive MCU */
				err = process_prfcnt_interrupts(kbdev, glb_req, glb_ack);
				kbase_csf_scheduler_spin_unlock(kbdev, flags);
				if (err)
					goto exit;

				/* Invoke the MCU state machine as a state transition
				 * might have completed.
				 */
				kbase_pm_update_state(kbdev);
			}
		}

		if (!glb_idle_irq_received)
			break;
		/* Attempt to serve potential IRQs that might have occurred
		 * whilst handling the previous IRQ. In case we have observed
		 * the GLB IDLE IRQ without all CSGs having been marked as
		 * idle, the GPU would be treated as no longer idle and left
		 * powered on.
		 */
		val = kbase_reg_read32(kbdev, JOB_CONTROL_ENUM(JOB_IRQ_STATUS));
	} while (val);

	if (deferred_handling_glb_idle_irq) {
		unsigned long flags;

		kbase_csf_scheduler_spin_lock(kbdev, &flags);
		kbase_csf_scheduler_process_gpu_idle_event(kbdev);
		kbase_csf_scheduler_spin_unlock(kbdev, flags);
	}
exit:
	wake_up_all(&kbdev->csf.event_wait);

	KBASE_KTRACE_ADD(kbdev, CSF_INTERRUPT_END, NULL, val);
}

/**
 * handle_glb_fatal - Handler for GLB FATAL.
 *
 * @kbdev:  Pointer to kbase device
 *
 * Report global fatal error to user space for all GPU command queue groups
 * in the device, terminate them and reset GPU.
 */
static void handle_glb_fatal(struct kbase_device *const kbdev)
{
	int as;

	for (as = 0; as < kbdev->nr_hw_address_spaces; as++) {
		unsigned long flags;
		struct kbase_context *kctx;
		struct kbase_fault fault = (struct kbase_fault){
			.status = GPU_EXCEPTION_TYPE_SW_FAULT_1,
		};

		if (as == MCU_AS_NR)
			continue;
		/* Only handle the fault for an active address space. Lock is
		 * taken here to atomically get reference to context in an
		 * active address space and retain its refcount.
		 */
		spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
		kctx = kbase_ctx_sched_as_to_ctx_nolock(kbdev, (size_t)as);
		if (kctx) {
			kbase_ctx_sched_retain_ctx_refcount(kctx);
			spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		} else {
			spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
			continue;
		}
		if (!kbase_reset_gpu_try_prevent(kbdev)) {
			mutex_lock(&kctx->csf.lock);
			kbase_csf_ctx_handle_fault(kctx, &fault, true);
			mutex_unlock(&kctx->csf.lock);

			kbase_reset_gpu_allow(kbdev);
		}
		kbase_ctx_sched_release_ctx_lock(kctx);
	}
	if (kbase_prepare_to_reset_gpu(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
		kbase_reset_gpu(kbdev);
}

void kbase_csf_glb_fatal_worker(struct work_struct *const data)
{
	struct kbase_device *const kbdev =
		container_of(data, struct kbase_device, csf.glb_fatal_work);

	handle_glb_fatal(kbdev);
}

void kbase_csf_handle_csg_sync_update(struct kbase_device *const kbdev, u32 group_id,
				      struct kbase_queue_group *group, u32 req, u32 ack)
{
	kbase_csf_scheduler_spin_lock_assert_held(kbdev);
	kbase_csf_fw_io_assert_opened(&kbdev->csf.fw_io);

	if ((req ^ ack) & CSG_REQ_SYNC_UPDATE_MASK) {
		kbase_csf_fw_io_group_write_mask(&kbdev->csf.fw_io, group_id, CSG_REQ, ack,
						 CSG_REQ_SYNC_UPDATE_MASK);

		KBASE_KTRACE_ADD_CSF_GRP(kbdev, CSG_INTERRUPT_SYNC_UPDATE, group, req ^ ack);

		/* SYNC_UPDATE events shall invalidate GPU idle event */
		atomic_set(&kbdev->csf.scheduler.gpu_no_longer_idle, true);

		kbase_csf_event_signal_cpu_only(group->kctx);
	}
}

void kbase_csf_doorbell_mapping_term(struct kbase_device *kbdev)
{
	if (kbdev->csf.db_filp) {
		struct page *page = as_page(kbdev->csf.dummy_db_page);

		/* This is a shared dummy sink page for avoiding potential segmentation fault
		 * to user-side library when a csi is off slot. Additionally, the call is on
		 * module unload path, so the page can be left uncleared before returning it
		 * back to kbdev memory pool.
		 */
		kbase_mem_pool_free(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], page, false);

		fput(kbdev->csf.db_filp);
	}
}

int kbase_csf_doorbell_mapping_init(struct kbase_device *kbdev)
{
	struct tagged_addr phys;
	struct file *filp;
	int ret;

	filp = shmem_file_setup("mali csf db", MAX_LFS_FILESIZE, VM_NORESERVE);
	if (IS_ERR(filp))
		return PTR_ERR(filp);

	ret = kbase_mem_pool_alloc_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], 1, &phys,
					 false, NULL);

	if (ret <= 0) {
		fput(filp);
		return ret;
	}

	kbdev->csf.db_filp = filp;
	kbdev->csf.dummy_db_page = phys;
	kbdev->csf.db_file_offsets = 0;

	return 0;
}

void kbase_csf_pending_gpuq_kick_queues_init(struct kbase_device *kbdev)
{
	size_t i;

	atomic_set(&kbdev->csf.pending_gpuq_kicks, false);
	for (i = 0; i != ARRAY_SIZE(kbdev->csf.pending_gpuq_kick_queues); ++i)
		INIT_LIST_HEAD(&kbdev->csf.pending_gpuq_kick_queues[i]);
	spin_lock_init(&kbdev->csf.pending_gpuq_kick_queues_lock);
}

void kbase_csf_pending_gpuq_kick_queues_term(struct kbase_device *kbdev)
{
	size_t i;

	spin_lock(&kbdev->csf.pending_gpuq_kick_queues_lock);
	for (i = 0; i != ARRAY_SIZE(kbdev->csf.pending_gpuq_kick_queues); ++i) {
		if (!list_empty(&kbdev->csf.pending_gpuq_kick_queues[i]))
			dev_warn(kbdev->dev,
				 "Some GPU queue kicks for priority %zu were not handled", i);
	}
	spin_unlock(&kbdev->csf.pending_gpuq_kick_queues_lock);
}

void kbase_csf_free_dummy_user_reg_page(struct kbase_device *kbdev)
{
	if (kbdev->csf.user_reg.filp) {
		struct page *page = as_page(kbdev->csf.user_reg.dummy_page);

		/* This is a shared dummy page in place of the real USER Register page just
		 * before the GPU is powered down. Additionally, the call is on module unload
		 * path, so the page can be left uncleared before returning it back to kbdev
		 * memory pool.
		 */
		kbase_mem_pool_free(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], page, false);
		fput(kbdev->csf.user_reg.filp);
	}
}

int kbase_csf_setup_dummy_user_reg_page(struct kbase_device *kbdev)
{
	struct tagged_addr phys;
	struct file *filp;
	struct page *page;
	u32 *addr;

	kbdev->csf.user_reg.filp = NULL;

	filp = shmem_file_setup("mali csf user_reg", MAX_LFS_FILESIZE, VM_NORESERVE);
	if (IS_ERR(filp)) {
		dev_err(kbdev->dev, "failed to get an unlinked file for user_reg");
		return PTR_ERR(filp);
	}

	if (kbase_mem_pool_alloc_pages(&kbdev->mem_pools.small[KBASE_MEM_GROUP_CSF_FW], 1, &phys,
				       false, NULL) <= 0) {
		fput(filp);
		return -ENOMEM;
	}

	page = as_page(phys);
	addr = kbase_kmap_atomic(page);

	/* Write a special value for the latest flush register inside the
	 * dummy page
	 */
	addr[LATEST_FLUSH / sizeof(u32)] = POWER_DOWN_LATEST_FLUSH_VALUE;

	kbase_sync_single_for_device(kbdev, kbase_dma_addr(page) + LATEST_FLUSH, sizeof(u32),
				     DMA_BIDIRECTIONAL);
	kbase_kunmap_atomic(addr);

	kbdev->csf.user_reg.filp = filp;
	kbdev->csf.user_reg.dummy_page = phys;
	kbdev->csf.user_reg.file_offset = 0;
	return 0;
}

u8 kbase_csf_priority_check(struct kbase_device *kbdev, u8 req_priority)
{
	struct priority_control_manager_device *pcm_device = kbdev->pcm_dev;
	u8 out_priority = req_priority;

	if (pcm_device) {
		req_priority = kbase_csf_priority_queue_group_priority_to_relative(req_priority);
		out_priority = pcm_device->ops.pcm_scheduler_priority_check(pcm_device, current,
									    req_priority);
		out_priority = kbase_csf_priority_relative_to_queue_group_priority(out_priority);
	}

	return out_priority;
}

void kbase_csf_process_queue_kick(struct kbase_queue *queue)
{
	struct kbase_context *kctx = queue->kctx;
	struct kbase_device *kbdev = kctx->kbdev;
	bool retry_kick = false;
	int err = kbase_reset_gpu_prevent_and_wait(kbdev);

	if (err) {
		dev_err(kbdev->dev, "Unsuccessful GPU reset detected when kicking queue");
		goto out_release_queue;
	}

	mutex_lock(&kctx->csf.lock);

	if (queue->bind_state != KBASE_CSF_QUEUE_BOUND)
		goto out_allow_gpu_reset;

	err = kbase_csf_scheduler_queue_start(queue);
	if (unlikely(err)) {
		dev_dbg(kbdev->dev, "Failed to start queue");
		if (err == -EBUSY) {
			retry_kick = true;

			spin_lock(&kbdev->csf.pending_gpuq_kick_queues_lock);
			if (list_empty(&queue->pending_kick_link)) {
				/* A failed queue kick shall be pushed to the
				 * back of the queue to avoid potential abuse.
				 */
				list_add_tail(
					&queue->pending_kick_link,
					&kbdev->csf.pending_gpuq_kick_queues[queue->group_priority]);
				spin_unlock(&kbdev->csf.pending_gpuq_kick_queues_lock);
			} else {
				spin_unlock(&kbdev->csf.pending_gpuq_kick_queues_lock);
				WARN_ON(atomic_read(&queue->pending_kick) == 0);
			}

			complete(&kbdev->csf.scheduler.kthread_signal);
		}
	}

out_allow_gpu_reset:
	if (likely(!retry_kick)) {
		WARN_ON(atomic_read(&queue->pending_kick) == 0);
		atomic_dec(&queue->pending_kick);
	}

	mutex_unlock(&kctx->csf.lock);

	kbase_reset_gpu_allow(kbdev);

	return;
out_release_queue:
	WARN_ON(atomic_read(&queue->pending_kick) == 0);
	atomic_dec(&queue->pending_kick);
}

void kbase_csf_process_protm_event_request(struct kbase_queue_group *group)
{
	struct kbase_protected_suspend_buffer *sbuf = &group->protected_suspend_buf;
	int err = 0;

	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, PROTM_EVENT_WORKER_START, group, 0u);

	err = alloc_grp_protected_suspend_buffer_pages(group);
	if (!err) {
		kbase_csf_scheduler_group_protm_enter(group);
	} else if (err == -ENOMEM && sbuf->alloc_retries <= PROTM_ALLOC_MAX_RETRIES) {
		sbuf->alloc_retries++;
		/* try again to allocate pages */
		kbase_csf_scheduler_enqueue_protm_event_work(group);
	} else if (sbuf->alloc_retries >= PROTM_ALLOC_MAX_RETRIES || err != -ENOMEM) {
		dev_err(group->kctx->kbdev->dev,
			"Failed to allocate physical pages for Protected mode suspend buffer for the group %d of context %d_%d",
			group->handle, group->kctx->tgid, group->kctx->id);
		report_group_fatal_error(group);
	}

	KBASE_KTRACE_ADD_CSF_GRP(group->kctx->kbdev, PROTM_EVENT_WORKER_END, group, 0u);
}
