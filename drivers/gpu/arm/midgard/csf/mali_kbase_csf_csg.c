// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
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

#include "mali_kbase_csf_csg.h"
#include "mali_kbase_csf_scheduler.h"
#include "mali_kbase_csf_util.h"
#include <mali_kbase.h>
#include <linux/delay.h>
#include <backend/gpu/mali_kbase_pm_internal.h>

/* Wait time to be used cumulatively for all the CSG slots.
 * Since scheduler lock is held when STATUS_UPDATE request is sent, there won't be
 * any other Host request pending on the FW side and usually FW would be responsive
 * to the Doorbell IRQs as it won't do any polling for a long time and also it won't
 * have to wait for any HW state transition to complete for publishing the status.
 * So it is reasonable to expect that handling of STATUS_UPDATE request would be
 * relatively very quick.
 */
#define STATUS_UPDATE_WAIT_TIMEOUT_NS 500

/* Number of nearby commands around the "cmd_ptr" of GPU queues.
 *
 *     [cmd_ptr - MAX_NR_NEARBY_INSTR, cmd_ptr + MAX_NR_NEARBY_INSTR].
 */
#define MAX_NR_NEARBY_INSTR 32

/* The bitmask of CSG slots for which the STATUS_UPDATE request completed.
 * The access to it is serialized with scheduler lock, so at a time it would
 * get used either for "active_groups" or per context "groups".
 */
static DECLARE_BITMAP(csg_slots_status_updated, MAX_SUPPORTED_CSGS);

/* String header for dumping cs user I/O status information */
#define KBASEP_CSF_CSG_DUMP_CS_HEADER_USER_IO \
	"Bind Idx,     Ringbuf addr,     Size, Prio,    Insert offset,   Extract offset, Active, Doorbell\n"

/* String representation of WAITING */
#define WAITING "Waiting"

/* String representation of NOT_WAITING */
#define NOT_WAITING "Not waiting"

/**
 * csg_slot_status_update_finish() - Complete STATUS_UPDATE request for a group slot.
 *
 * @kbdev:  Pointer to kbase device.
 * @csg_nr: The group slot number.
 *
 * Return: Non-zero if not complete, otherwise zero.
 */
static bool csg_slot_status_update_finish(struct kbase_device *kbdev, u32 csg_nr)
{
	struct kbase_csf_cmd_stream_group_info const *const ginfo =
		&kbdev->csf.global_iface.groups[csg_nr];

	return !((kbase_csf_firmware_csg_input_read(ginfo, CSG_REQ) ^
		  kbase_csf_firmware_csg_output(ginfo, CSG_ACK)) &
		 CSG_REQ_STATUS_UPDATE_MASK);
}

/**
 * csg_slots_status_update_finish() - Complete STATUS_UPDATE requests for all group slots.
 *
 * @kbdev:      Pointer to kbase device.
 * @slots_mask: The group slots mask.
 *
 * Return: Non-zero if not complete, otherwise zero.
 */
static bool csg_slots_status_update_finish(struct kbase_device *kbdev,
					   const unsigned long *slots_mask)
{
	const u32 max_csg_slots = kbdev->csf.global_iface.group_num;
	bool changed = false;
	u32 csg_nr;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	for_each_set_bit(csg_nr, slots_mask, max_csg_slots) {
		if (csg_slot_status_update_finish(kbdev, csg_nr)) {
			set_bit(csg_nr, csg_slots_status_updated);
			changed = true;
		}
	}

	return changed;
}

/**
 * wait_csg_slots_status_update_finish() - Wait completion of STATUS_UPDATE requests for all
 *                                         group slots.
 *
 * @kbdev:  Pointer to kbase device.
 * @slots_mask: The group slots mask.
 */
static void wait_csg_slots_status_update_finish(struct kbase_device *kbdev,
						unsigned long *slots_mask)
{
	const u32 max_csg_slots = kbdev->csf.global_iface.group_num;
	long remaining = kbase_csf_timeout_in_jiffies(STATUS_UPDATE_WAIT_TIMEOUT_NS);

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	bitmap_zero(csg_slots_status_updated, max_csg_slots);

	while (!bitmap_empty(slots_mask, max_csg_slots) && remaining) {
		remaining = wait_event_timeout(kbdev->csf.event_wait,
					       csg_slots_status_update_finish(kbdev, slots_mask),
					       remaining);
		if (likely(remaining)) {
			bitmap_andnot(slots_mask, slots_mask, csg_slots_status_updated,
				      max_csg_slots);
		} else {
			dev_warn(kbdev->dev, "STATUS_UPDATE request timed out for slots 0x%lx",
				 slots_mask[0]);
		}
	}
}

/**
 * blocked_reason_to_string() - Convert blocking reason id to a string
 *
 * @reason_id: blocked_reason
 *
 * Return: Suitable string
 */
static const char *blocked_reason_to_string(u32 reason_id)
{
	/* possible blocking reasons of a cs */
	static const char *const cs_blocked_reason[] = {
		[CS_STATUS_BLOCKED_REASON_REASON_UNBLOCKED] = "UNBLOCKED",
		[CS_STATUS_BLOCKED_REASON_REASON_WAIT] = "WAIT",
		[CS_STATUS_BLOCKED_REASON_REASON_PROGRESS_WAIT] = "PROGRESS_WAIT",
		[CS_STATUS_BLOCKED_REASON_REASON_SYNC_WAIT] = "SYNC_WAIT",
		[CS_STATUS_BLOCKED_REASON_REASON_DEFERRED] = "DEFERRED",
		[CS_STATUS_BLOCKED_REASON_REASON_RESOURCE] = "RESOURCE",
		[CS_STATUS_BLOCKED_REASON_REASON_FLUSH] = "FLUSH"
	};

	if (WARN_ON(reason_id >= ARRAY_SIZE(cs_blocked_reason)))
		return "UNKNOWN_BLOCKED_REASON_ID";

	return cs_blocked_reason[reason_id];
}

/**
 * sb_source_supported() - Check SB_SOURCE GLB version support
 *
 * @glb_version:  The GLB version
 *
 * Return: False or true on success.
 */
static bool sb_source_supported(u32 glb_version)
{
	bool supported = false;

	if (((GLB_VERSION_MAJOR_GET(glb_version) == 3) &&
	     (GLB_VERSION_MINOR_GET(glb_version) >= 5)) ||
	    ((GLB_VERSION_MAJOR_GET(glb_version) == 2) &&
	     (GLB_VERSION_MINOR_GET(glb_version) >= 6)) ||
	    ((GLB_VERSION_MAJOR_GET(glb_version) == 1) &&
	     (GLB_VERSION_MINOR_GET(glb_version) >= 3)))
		supported = true;

	return supported;
}


/**
 * kbasep_csf_csg_active_dump_cs_status_wait() - Dump active queue sync status information.
 *
 * @kctx:                 Pointer to kbase context.
 * @kbpr:                 Pointer to printer instance.
 * @glb_version:          The GLB version.
 * @wait_status:          The CS_STATUS_WAIT value.
 * @wait_sync_value:      The queue's cached sync value.
 * @wait_sync_live_value: The queue's sync object current value.
 * @wait_sync_pointer:    The queue's sync object pointer.
 * @sb_status:            The CS_STATUS_SCOREBOARDS value.
 * @blocked_reason:       The CS_STATUS_BLCOKED_REASON value.
 */
static void kbasep_csf_csg_active_dump_cs_status_wait(struct kbase_context *kctx,
						      struct kbasep_printer *kbpr, u32 glb_version,
						      u32 wait_status, u32 wait_sync_value,
						      u64 wait_sync_live_value,
						      u64 wait_sync_pointer, u32 sb_status,
						      u32 blocked_reason)
{
	kbasep_print(kbpr, "SB_MASK: %d\n", CS_STATUS_WAIT_SB_MASK_GET(wait_status));
	if (sb_source_supported(glb_version))
		kbasep_print(kbpr, "SB_SOURCE: %d\n", CS_STATUS_WAIT_SB_SOURCE_GET(wait_status));

	{
		kbasep_print(kbpr, "PROGRESS_WAIT: %s\n",
			     CS_STATUS_WAIT_PROGRESS_WAIT_GET(wait_status) ? WAITING : NOT_WAITING);
	}
	kbasep_print(kbpr, "PROTM_PEND: %s\n",
		     CS_STATUS_WAIT_PROTM_PEND_GET(wait_status) ? WAITING : NOT_WAITING);
	kbasep_print(kbpr, "SYNC_WAIT: %s\n",
		     CS_STATUS_WAIT_SYNC_WAIT_GET(wait_status) ? WAITING : NOT_WAITING);
	kbasep_print(kbpr, "WAIT_CONDITION: %s\n",
		     CS_STATUS_WAIT_SYNC_WAIT_CONDITION_GET(wait_status) ? "greater than" :
										 "less or equal");
	kbasep_print(kbpr, "SYNC_POINTER: 0x%llx\n", wait_sync_pointer);
	kbasep_print(kbpr, "SYNC_VALUE: %d\n", wait_sync_value);
	kbasep_print(kbpr, "SYNC_LIVE_VALUE: 0x%016llx\n", wait_sync_live_value);
	kbasep_print(kbpr, "SB_STATUS: %u\n", CS_STATUS_SCOREBOARDS_NONZERO_GET(sb_status));
	kbasep_print(kbpr, "BLOCKED_REASON: %s\n",
		     blocked_reason_to_string(CS_STATUS_BLOCKED_REASON_REASON_GET(blocked_reason)));
}

/**
 * kbasep_csf_csg_active_dump_cs_trace() - Dump active queue CS trace information.
 *
 * @kctx:   Pointer to kbase context.
 * @kbpr:   Pointer to printer instance.
 * @stream: Pointer to command stream information.
 */
static void
kbasep_csf_csg_active_dump_cs_trace(struct kbase_context *kctx, struct kbasep_printer *kbpr,
				    struct kbase_csf_cmd_stream_info const *const stream)
{
	u32 val = kbase_csf_firmware_cs_input_read(stream, CS_INSTR_BUFFER_BASE_LO);
	u64 addr = ((u64)kbase_csf_firmware_cs_input_read(stream, CS_INSTR_BUFFER_BASE_HI) << 32) |
		   val;
	val = kbase_csf_firmware_cs_input_read(stream, CS_INSTR_BUFFER_SIZE);

	kbasep_print(kbpr, "CS_TRACE_BUF_ADDR: 0x%16llx, SIZE: %u\n", addr, val);

	/* Write offset variable address (pointer) */
	val = kbase_csf_firmware_cs_input_read(stream, CS_INSTR_BUFFER_OFFSET_POINTER_LO);
	addr = ((u64)kbase_csf_firmware_cs_input_read(stream, CS_INSTR_BUFFER_OFFSET_POINTER_HI)
		<< 32) |
	       val;
	kbasep_print(kbpr, "CS_TRACE_BUF_OFFSET_PTR: 0x%16llx\n", addr);

	/* EVENT_SIZE and EVENT_STATEs */
	val = kbase_csf_firmware_cs_input_read(stream, CS_INSTR_CONFIG);
	kbasep_print(kbpr, "TRACE_EVENT_SIZE: 0x%x, TRACE_EVENT_STATES 0x%x\n",
		     CS_INSTR_CONFIG_EVENT_SIZE_GET(val), CS_INSTR_CONFIG_EVENT_STATE_GET(val));
}

/**
 * kbasep_csf_read_cmdbuff_value() - Read a command from a queue offset.
 *
 * @queue:          Address of a GPU command queue to examine.
 * @cmdbuff_offset: GPU address offset in queue's memory buffer.
 *
 * Return: Encoded CSF command (64-bit)
 */
static u64 kbasep_csf_read_cmdbuff_value(struct kbase_queue *queue, u32 cmdbuff_offset)
{
	u64 page_off = cmdbuff_offset >> PAGE_SHIFT;
	u64 offset_within_page = cmdbuff_offset & ~PAGE_MASK;
	struct page *page = as_page(queue->queue_reg->gpu_alloc->pages[page_off]);
	u64 *cmdbuff = vmap(&page, 1, VM_MAP, pgprot_noncached(PAGE_KERNEL));
	u64 value;

	if (!cmdbuff) {
		struct kbase_context *kctx = queue->kctx;

		dev_info(kctx->kbdev->dev, "%s failed to map the buffer page for read a command!",
			 __func__);
		/* Return an alternative 0 for dumping operation*/
		value = 0;
	} else {
		value = cmdbuff[offset_within_page / sizeof(u64)];
		vunmap(cmdbuff);
	}

	return value;
}

/**
 * kbasep_csf_csg_active_dump_cs_status_cmd_ptr() - Dump CMD_PTR information and nearby commands.
 *
 * @kbpr:    Pointer to printer instance.
 * @queue:   Address of a GPU command queue to examine.
 * @cmd_ptr: CMD_PTR address.
 */
static void kbasep_csf_csg_active_dump_cs_status_cmd_ptr(struct kbasep_printer *kbpr,
							 struct kbase_queue *queue, u64 cmd_ptr)
{
	u64 cmd_ptr_offset;
	u64 cursor, end_cursor, instr;
	u32 nr_nearby_instr_size;
	struct kbase_va_region *reg;

	kbase_gpu_vm_lock(queue->kctx);
	reg = kbase_region_tracker_find_region_enclosing_address(queue->kctx, cmd_ptr);
	if (reg && !(reg->flags & KBASE_REG_FREE) && (reg->flags & KBASE_REG_CPU_RD) &&
	    (reg->gpu_alloc->type == KBASE_MEM_TYPE_NATIVE)) {
		kbasep_print(kbpr, "CMD_PTR region nr_pages: %zu\n", reg->nr_pages);
		nr_nearby_instr_size = MAX_NR_NEARBY_INSTR * sizeof(u64);
		cmd_ptr_offset = cmd_ptr - queue->base_addr;
		cursor = (cmd_ptr_offset > nr_nearby_instr_size) ?
				       cmd_ptr_offset - nr_nearby_instr_size :
				       0;
		end_cursor = cmd_ptr_offset + nr_nearby_instr_size;
		if (end_cursor > queue->size)
			end_cursor = queue->size;
		kbasep_print(kbpr,
			     "queue:GPU-%u-%u-%u at:0x%.16llx cmd_ptr:0x%.16llx "
			     "dump_begin:0x%.16llx dump_end:0x%.16llx\n",
			     queue->kctx->id, queue->group->handle, queue->csi_index,
			     (queue->base_addr + cursor), cmd_ptr, (queue->base_addr + cursor),
			     (queue->base_addr + end_cursor));
		while ((cursor < end_cursor)) {
			instr = kbasep_csf_read_cmdbuff_value(queue, (u32)cursor);
			if (instr != 0)
				kbasep_print(kbpr,
					     "queue:GPU-%u-%u-%u at:0x%.16llx cmd:0x%.16llx\n",
					     queue->kctx->id, queue->group->handle,
					     queue->csi_index, (queue->base_addr + cursor), instr);
			cursor += sizeof(u64);
		}
	}
	kbase_gpu_vm_unlock(queue->kctx);
}

/**
 * kbasep_csf_csg_active_dump_queue() - Dump GPU command queue debug information.
 *
 * @kbpr:  Pointer to printer instance.
 * @queue: Address of a GPU command queue to examine
 */
static void kbasep_csf_csg_active_dump_queue(struct kbasep_printer *kbpr, struct kbase_queue *queue)
{
	u64 *addr;
	u32 *addr32;
	u64 cs_extract;
	u64 cs_insert;
	u32 cs_active;
	u64 wait_sync_pointer;
	u32 wait_status, wait_sync_value;
	u32 sb_status;
	u32 blocked_reason;
	struct kbase_vmap_struct *mapping;
	u64 *evt;
	u64 wait_sync_live_value;
	u32 glb_version;
	u64 cmd_ptr;

	if (!queue)
		return;

	glb_version = queue->kctx->kbdev->csf.global_iface.version;

	if (WARN_ON(queue->csi_index == KBASEP_IF_NR_INVALID || !queue->group))
		return;

	addr = queue->user_io_addr;
	cs_insert = addr[CS_INSERT_LO / sizeof(*addr)];

	addr = queue->user_io_addr + PAGE_SIZE / sizeof(*addr);
	cs_extract = addr[CS_EXTRACT_LO / sizeof(*addr)];

	addr32 = (u32 *)(queue->user_io_addr + PAGE_SIZE / sizeof(*addr));
	cs_active = addr32[CS_ACTIVE / sizeof(*addr32)];

	kbasep_puts(kbpr, KBASEP_CSF_CSG_DUMP_CS_HEADER_USER_IO);
	kbasep_print(kbpr, "%8d, %16llx, %8x, %4u, %16llx, %16llx, %6u, %8d\n", queue->csi_index,
		     queue->base_addr, queue->size, queue->priority, cs_insert, cs_extract,
		     cs_active, queue->doorbell_nr);

	/* Print status information for blocked group waiting for sync object. For on-slot queues,
	 * if cs_trace is enabled, dump the interface's cs_trace configuration.
	 */
	if (kbase_csf_scheduler_group_get_slot(queue->group) < 0) {
		kbasep_print(kbpr, "SAVED_CMD_PTR: 0x%llx\n", queue->saved_cmd_ptr);
		if (CS_STATUS_WAIT_SYNC_WAIT_GET(queue->status_wait)) {
			wait_status = queue->status_wait;
			wait_sync_value = queue->sync_value;
			wait_sync_pointer = queue->sync_ptr;
			sb_status = queue->sb_status;
			blocked_reason = queue->blocked_reason;

			evt = (u64 *)kbase_phy_alloc_mapping_get(queue->kctx, wait_sync_pointer,
								 &mapping);
			if (evt) {
				wait_sync_live_value = evt[0];
				kbase_phy_alloc_mapping_put(queue->kctx, mapping);
			} else {
				wait_sync_live_value = U64_MAX;
			}

			kbasep_csf_csg_active_dump_cs_status_wait(
				queue->kctx, kbpr, glb_version, wait_status, wait_sync_value,
				wait_sync_live_value, wait_sync_pointer, sb_status, blocked_reason);
		}
		kbasep_csf_csg_active_dump_cs_status_cmd_ptr(kbpr, queue, queue->saved_cmd_ptr);
	} else {
		struct kbase_device const *const kbdev = queue->group->kctx->kbdev;
		struct kbase_csf_cmd_stream_group_info const *const ginfo =
			&kbdev->csf.global_iface.groups[queue->group->csg_nr];
		struct kbase_csf_cmd_stream_info const *const stream =
			&ginfo->streams[queue->csi_index];
		u32 req_res;

		if (WARN_ON(!stream))
			return;

		cmd_ptr = kbase_csf_firmware_cs_output(stream, CS_STATUS_CMD_PTR_LO);
		cmd_ptr |= (u64)kbase_csf_firmware_cs_output(stream, CS_STATUS_CMD_PTR_HI) << 32;
		req_res = kbase_csf_firmware_cs_output(stream, CS_STATUS_REQ_RESOURCE);

		kbasep_print(kbpr, "CMD_PTR: 0x%llx\n", cmd_ptr);
		kbasep_print(kbpr, "REQ_RESOURCE [COMPUTE]: %d\n",
			     CS_STATUS_REQ_RESOURCE_COMPUTE_RESOURCES_GET(req_res));
		kbasep_print(kbpr, "REQ_RESOURCE [FRAGMENT]: %d\n",
			     CS_STATUS_REQ_RESOURCE_FRAGMENT_RESOURCES_GET(req_res));
		kbasep_print(kbpr, "REQ_RESOURCE [TILER]: %d\n",
			     CS_STATUS_REQ_RESOURCE_TILER_RESOURCES_GET(req_res));
		kbasep_print(kbpr, "REQ_RESOURCE [IDVS]: %d\n",
			     CS_STATUS_REQ_RESOURCE_IDVS_RESOURCES_GET(req_res));

		wait_status = kbase_csf_firmware_cs_output(stream, CS_STATUS_WAIT);
		wait_sync_value = kbase_csf_firmware_cs_output(stream, CS_STATUS_WAIT_SYNC_VALUE);
		wait_sync_pointer =
			kbase_csf_firmware_cs_output(stream, CS_STATUS_WAIT_SYNC_POINTER_LO);
		wait_sync_pointer |=
			(u64)kbase_csf_firmware_cs_output(stream, CS_STATUS_WAIT_SYNC_POINTER_HI)
			<< 32;

		sb_status = kbase_csf_firmware_cs_output(stream, CS_STATUS_SCOREBOARDS);
		blocked_reason = kbase_csf_firmware_cs_output(stream, CS_STATUS_BLOCKED_REASON);

		evt = (u64 *)kbase_phy_alloc_mapping_get(queue->kctx, wait_sync_pointer, &mapping);
		if (evt) {
			wait_sync_live_value = evt[0];
			kbase_phy_alloc_mapping_put(queue->kctx, mapping);
		} else {
			wait_sync_live_value = U64_MAX;
		}

		kbasep_csf_csg_active_dump_cs_status_wait(queue->kctx, kbpr, glb_version,
							  wait_status, wait_sync_value,
							  wait_sync_live_value, wait_sync_pointer,
							  sb_status, blocked_reason);
		/* Dealing with cs_trace */
		if (kbase_csf_scheduler_queue_has_trace(queue))
			kbasep_csf_csg_active_dump_cs_trace(queue->kctx, kbpr, stream);
		else
			kbasep_print(kbpr, "NO CS_TRACE\n");
		kbasep_csf_csg_active_dump_cs_status_cmd_ptr(kbpr, queue, cmd_ptr);
	}
}

/**
 * kbasep_csf_csg_active_dump_group() - Dump an active group.
 *
 * @kbpr:  Pointer to printer instance.
 * @group: GPU group.
 */
static void kbasep_csf_csg_active_dump_group(struct kbasep_printer *kbpr,
					     struct kbase_queue_group *const group)
{
	if (kbase_csf_scheduler_group_get_slot(group) >= 0) {
		struct kbase_device *const kbdev = group->kctx->kbdev;
		u32 ep_c, ep_r;
		char exclusive;
		char idle = 'N';
		struct kbase_csf_cmd_stream_group_info const *const ginfo =
			&kbdev->csf.global_iface.groups[group->csg_nr];
		u8 slot_priority = kbdev->csf.scheduler.csg_slots[group->csg_nr].priority;

		ep_c = kbase_csf_firmware_csg_output(ginfo, CSG_STATUS_EP_CURRENT);
		ep_r = kbase_csf_firmware_csg_output(ginfo, CSG_STATUS_EP_REQ);

		if (CSG_STATUS_EP_REQ_EXCLUSIVE_COMPUTE_GET(ep_r))
			exclusive = 'C';
		else if (CSG_STATUS_EP_REQ_EXCLUSIVE_FRAGMENT_GET(ep_r))
			exclusive = 'F';
		else
			exclusive = '0';

		if (kbase_csf_firmware_csg_output(ginfo, CSG_STATUS_STATE) &
		    CSG_STATUS_STATE_IDLE_MASK)
			idle = 'Y';

		if (!test_bit(group->csg_nr, csg_slots_status_updated)) {
			kbasep_print(kbpr, "*** Warn: Timed out for STATUS_UPDATE on slot %d\n",
				     group->csg_nr);
			kbasep_print(kbpr, "*** The following group-record is likely stale\n");
		}
			kbasep_print(
				kbpr,
				"GroupID, CSG NR, CSG Prio, Run State, Priority, C_EP(Alloc/Req),"
				" F_EP(Alloc/Req), T_EP(Alloc/Req), Exclusive, Idle\n");
			kbasep_print(
				kbpr,
				"%7d, %6d, %8d, %9d, %8d, %11d/%3d, %11d/%3d, %11d/%3d, %9c, %4c\n",
				group->handle, group->csg_nr, slot_priority, group->run_state,
				group->priority, CSG_STATUS_EP_CURRENT_COMPUTE_EP_GET(ep_c),
				CSG_STATUS_EP_REQ_COMPUTE_EP_GET(ep_r),
				CSG_STATUS_EP_CURRENT_FRAGMENT_EP_GET(ep_c),
				CSG_STATUS_EP_REQ_FRAGMENT_EP_GET(ep_r),
				CSG_STATUS_EP_CURRENT_TILER_EP_GET(ep_c),
				CSG_STATUS_EP_REQ_TILER_EP_GET(ep_r), exclusive, idle);

	} else {
		kbasep_print(kbpr, "GroupID, CSG NR, Run State, Priority\n");
		kbasep_print(kbpr, "%7d, %6d, %9d, %8d\n", group->handle, group->csg_nr,
			     group->run_state, group->priority);
	}

	if (group->run_state != KBASE_CSF_GROUP_TERMINATED) {
		unsigned int i;

		kbasep_print(kbpr, "Bound queues:\n");

		for (i = 0; i < MAX_SUPPORTED_STREAMS_PER_GROUP; i++)
			kbasep_csf_csg_active_dump_queue(kbpr, group->bound_queues[i]);
	}
}

void kbase_csf_csg_update_status(struct kbase_device *kbdev)
{
	u32 max_csg_slots = kbdev->csf.global_iface.group_num;
	DECLARE_BITMAP(used_csgs, MAX_SUPPORTED_CSGS) = { 0 };
	u32 csg_nr;
	unsigned long flags;

	lockdep_assert_held(&kbdev->csf.scheduler.lock);

	/* Global doorbell ring for CSG STATUS_UPDATE request or User doorbell
	 * ring for Extract offset update, shall not be made when MCU has been
	 * put to sleep otherwise it will undesirably make MCU exit the sleep
	 * state. Also it isn't really needed as FW will implicitly update the
	 * status of all on-slot groups when MCU sleep request is sent to it.
	 */
	if (kbdev->csf.scheduler.state == SCHED_SLEEPING) {
		/* Wait for the MCU sleep request to complete. */
		kbase_pm_wait_for_desired_state(kbdev);
		bitmap_copy(csg_slots_status_updated, kbdev->csf.scheduler.csg_inuse_bitmap,
			    max_csg_slots);
		return;
	}

	for (csg_nr = 0; csg_nr < max_csg_slots; csg_nr++) {
		struct kbase_queue_group *const group =
			kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;
		if (!group)
			continue;
		/* Ring the User doorbell for FW to update the Extract offset */
		kbase_csf_ring_doorbell(kbdev, group->doorbell_nr);
		set_bit(csg_nr, used_csgs);
	}

	/* Return early if there are no on-slot groups */
	if (bitmap_empty(used_csgs, max_csg_slots))
		return;

	kbase_csf_scheduler_spin_lock(kbdev, &flags);
	for_each_set_bit(csg_nr, used_csgs, max_csg_slots) {
		struct kbase_csf_cmd_stream_group_info const *const ginfo =
			&kbdev->csf.global_iface.groups[csg_nr];
		kbase_csf_firmware_csg_input_mask(ginfo, CSG_REQ,
						  ~kbase_csf_firmware_csg_output(ginfo, CSG_ACK),
						  CSG_REQ_STATUS_UPDATE_MASK);
	}

	BUILD_BUG_ON(MAX_SUPPORTED_CSGS > (sizeof(used_csgs[0]) * BITS_PER_BYTE));
	kbase_csf_ring_csg_slots_doorbell(kbdev, used_csgs[0]);
	kbase_csf_scheduler_spin_unlock(kbdev, flags);
	wait_csg_slots_status_update_finish(kbdev, used_csgs);
	/* Wait for the user doorbell ring to take effect */
	msleep(100);
}

int kbasep_csf_csg_dump_print(struct kbase_context *const kctx, struct kbasep_printer *kbpr)
{
	u32 gr;
	struct kbase_device *kbdev;

	if (WARN_ON(!kctx))
		return -EINVAL;

	kbdev = kctx->kbdev;

	kbasep_print(kbpr,
		     "CSF groups status (version: v" __stringify(MALI_CSF_CSG_DUMP_VERSION) "):\n");

	mutex_lock(&kctx->csf.lock);
	kbase_csf_scheduler_lock(kbdev);
	kbase_csf_csg_update_status(kbdev);
	kbasep_print(kbpr, "Ctx %d_%d\n", kctx->tgid, kctx->id);
	for (gr = 0; gr < MAX_QUEUE_GROUP_NUM; gr++) {
		struct kbase_queue_group *const group = kctx->csf.queue_groups[gr];

		if (!group)
			continue;

		kbasep_csf_csg_active_dump_group(kbpr, group);
	}
	kbase_csf_scheduler_unlock(kbdev);
	mutex_unlock(&kctx->csf.lock);

	return 0;
}

int kbasep_csf_csg_active_dump_print(struct kbase_device *kbdev, struct kbasep_printer *kbpr)
{
	u32 csg_nr;
	u32 num_groups;

	if (WARN_ON(!kbdev))
		return -EINVAL;

	num_groups = kbdev->csf.global_iface.group_num;

	kbasep_print(kbpr, "CSF active groups status (version: v" __stringify(
				   MALI_CSF_CSG_DUMP_VERSION) "):\n");

	kbase_csf_scheduler_lock(kbdev);
	kbase_csf_csg_update_status(kbdev);
	for (csg_nr = 0; csg_nr < num_groups; csg_nr++) {
		struct kbase_queue_group *const group =
			kbdev->csf.scheduler.csg_slots[csg_nr].resident_group;

		if (!group)
			continue;

		kbasep_print(kbpr, "Ctx %d_%d\n", group->kctx->tgid, group->kctx->id);

		kbasep_csf_csg_active_dump_group(kbpr, group);
	}
	kbase_csf_scheduler_unlock(kbdev);

	return 0;
}
