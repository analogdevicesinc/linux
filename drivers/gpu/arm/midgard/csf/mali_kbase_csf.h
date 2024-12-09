/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
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

#ifndef _KBASE_CSF_H_
#define _KBASE_CSF_H_

#include "mali_kbase_csf_kcpu.h"
#include "mali_kbase_csf_scheduler.h"
#include "mali_kbase_csf_firmware.h"
#include "mali_kbase_csf_protected_memory.h"
#include "mali_kbase_hwaccess_time.h"

/* Indicate invalid CS h/w interface
 */
#define KBASEP_IF_NR_INVALID ((s8)-1)

/* Indicate invalid CSG number for a GPU command queue group
 */
#define KBASEP_CSG_NR_INVALID ((s8)-1)

/* Indicate invalid user doorbell number for a GPU command queue
 */
#define KBASEP_USER_DB_NR_INVALID ((s8)-1)

/* Indicates an invalid value for the scan out sequence number, used to
 * signify there is no group that has protected mode execution pending.
 */
#define KBASEP_TICK_PROTM_PEND_SCAN_SEQ_NR_INVALID (U32_MAX)

#define FIRMWARE_IDLE_HYSTERESIS_TIME_NS (10 * 1000 * 1000) /* Default 10 milliseconds */

/* Idle hysteresis time can be scaled down when GPU sleep feature is used */
#define FIRMWARE_IDLE_HYSTERESIS_GPU_SLEEP_SCALER (5)

/**
 * kbase_csf_ctx_init - Initialize the CSF interface for a GPU address space.
 *
 * @kctx:	Pointer to the kbase context which is being initialized.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
int kbase_csf_ctx_init(struct kbase_context *kctx);

/**
 * kbase_csf_ctx_handle_fault - Terminate queue groups & notify fault upon
 *                              GPU bus fault, MMU page fault or similar.
 *
 * @kctx:            Pointer to faulty kbase context.
 * @fault:           Pointer to the fault.
 * @fw_unresponsive: Whether or not the FW is deemed unresponsive
 *
 * This function terminates all GPU command queue groups in the context and
 * notifies the event notification thread of the fault. If the FW is deemed
 * unresponsive, e.g. when recovering from a GLB_FATAL, it will not wait
 * for the groups to be terminated by the MCU, since in this case it will
 * time-out anyway.
 */
void kbase_csf_ctx_handle_fault(struct kbase_context *kctx, struct kbase_fault *fault,
				bool fw_unresponsive);

/**
 * kbase_csf_ctx_report_page_fault_for_active_groups - Notify Userspace about GPU page fault
 *                                                   for active groups of the faulty context.
 *
 * @kctx:       Pointer to faulty kbase context.
 * @fault:      Pointer to the fault.
 *
 * This function notifies the event notification thread of the GPU page fault.
 */
void kbase_csf_ctx_report_page_fault_for_active_groups(struct kbase_context *kctx,
						       struct kbase_fault *fault);

/**
 * kbase_csf_ctx_term - Terminate the CSF interface for a GPU address space.
 *
 * @kctx:	Pointer to the kbase context which is being terminated.
 *
 * This function terminates any remaining CSGs and CSs which weren't destroyed
 * before context termination.
 */
void kbase_csf_ctx_term(struct kbase_context *kctx);

/**
 * kbase_csf_queue_register - Register a GPU command queue.
 *
 * @kctx:	Pointer to the kbase context within which the
 *		queue is to be registered.
 * @reg:	Pointer to the structure which contains details of the
 *		queue to be registered within the provided
 *		context.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_queue_register(struct kbase_context *kctx, struct kbase_ioctl_cs_queue_register *reg);

/**
 * kbase_csf_queue_register_ex - Register a GPU command queue with
 *                               extended format.
 *
 * @kctx:	Pointer to the kbase context within which the
 *		queue is to be registered.
 * @reg:	Pointer to the structure which contains details of the
 *		queue to be registered within the provided
 *		context, together with the extended parameter fields
 *              for supporting cs trace command.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_queue_register_ex(struct kbase_context *kctx,
				struct kbase_ioctl_cs_queue_register_ex *reg);

/**
 * kbase_csf_queue_terminate - Terminate a GPU command queue.
 *
 * @kctx:	Pointer to the kbase context within which the
 *		queue is to be terminated.
 * @term:	Pointer to the structure which identifies which
 *		queue is to be terminated.
 */
void kbase_csf_queue_terminate(struct kbase_context *kctx,
			       struct kbase_ioctl_cs_queue_terminate *term);

/**
 * kbase_csf_free_command_stream_user_pages() - Free queue resources
 *                                              from bind time.
 *
 * @kctx: Address of the kbase context within which the queue was created.
 * @queue: Pointer to the queue to be unlinked.
 *
 * This function releases the hardware doorbell page assigned to the queue
 * and releases the reference taken on the queue.
 *
 * When mali_kbase_supports_csg_cs_user_page_allocation() is false:
 *  This function will free the pair of CS_USER IO physical pages allocated
 *  for a GPU command queue, that were mapped into the process address space
 *  to enable direct submission of commands to the hardware.
 *
 *  This function will be called only when the mapping is being removed and
 *  so the resources for queue will not get freed up until the mapping is
 *  removed even though userspace could have terminated the queue.
 *  Kernel will ensure that the termination of Kbase context would only be
 *  triggered after the mapping is removed.
 *
 *  If an explicit or implicit unbind was missed by the userspace then the
 *  mapping will persist. On process exit kernel itself will remove the mapping.
 *
 * When mali_kbase_supports_csg_cs_user_page_allocation() is true:
 *  No specific actions are required for CS_USER IO pages. CSG termination
 *  will take care of it.
 */
void kbase_csf_free_command_stream_user_pages(struct kbase_context *kctx,
					      struct kbase_queue *queue);

/**
 * kbase_csf_alloc_command_stream_user_pages() - Allocate queue resources
 *                                               at bind time.
 *
 * @kctx: Pointer to the kbase context within which the resources
 *        for the queue are being allocated.
 * @queue: Pointer to the queue for which to allocate resources.
 *
 * This function reserves a hardware doorbell page for the queue and
 * takes a reference on the queue.
 *
 * When mali_kbase_supports_csg_cs_user_page_allocation() is false:
 *   The function allocates a pair of User mode input/output pages for a
 *   GPU command queue and maps them in the shared interface segment of MCU
 *   firmware address space.
 *
 * When mali_kbase_supports_csg_cs_user_page_allocation() is true:
 *   A slot of size CS_USER_INPUT_BLOCK_SIZE is assigned to the queue in
 *   the CS_USER IO page owned by the CSG.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_alloc_command_stream_user_pages(struct kbase_context *kctx,
					      struct kbase_queue *queue);

/**
 * kbase_csf_queue_bind - Bind a GPU command queue to a queue group.
 *
 * @kctx:	The kbase context.
 * @bind:	Pointer to the union which specifies a queue group and a
 *		queue to be bound to that group.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_queue_bind(struct kbase_context *kctx, union kbase_ioctl_cs_queue_bind *bind);

/**
 * kbase_csf_queue_unbind - Unbind a GPU command queue from a queue group
 *			    to which it has been bound and free
 *			    resources allocated for this queue if there
 *			    are any.
 *
 * @queue:	Pointer to queue to be unbound.
 * @process_exit: Flag to indicate if process exit is happening.
 */
void kbase_csf_queue_unbind(struct kbase_queue *queue, bool process_exit);

/**
 * kbase_csf_queue_unbind_stopped - Unbind a GPU command queue in the case
 *                                  where it was never started.
 * @queue:      Pointer to queue to be unbound.
 *
 * Variant of kbase_csf_queue_unbind() for use on error paths for cleaning up
 * queues that failed to fully bind.
 */
void kbase_csf_queue_unbind_stopped(struct kbase_queue *queue);

/**
 * kbase_csf_queue_kick - Schedule a GPU command queue on the firmware
 *
 * @kctx:   The kbase context.
 * @kick:   Pointer to the struct which specifies the queue
 *          that needs to be scheduled.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_queue_kick(struct kbase_context *kctx, struct kbase_ioctl_cs_queue_kick *kick);

/**
 * kbase_csf_find_queue_group - Find the queue group corresponding
 *                                         to the indicated handle.
 *
 * @kctx:          The kbase context under which the queue group exists.
 * @group_handle:  Handle for the group which uniquely identifies it within
 *                 the context with which it was created.
 *
 * This function is used to find the queue group when passed a handle.
 *
 * Return: Pointer to a queue group on success, NULL on failure
 */
struct kbase_queue_group *kbase_csf_find_queue_group(struct kbase_context *kctx, u8 group_handle);

/**
 * kbase_csf_queue_group_handle_is_valid - Find if the given queue group handle
 *                                         is valid.
 *
 * @kctx:		The kbase context under which the queue group exists.
 * @group_handle:	Handle for the group which uniquely identifies it within
 *			the context with which it was created.
 *
 * This function is used to determine if the queue group handle is valid.
 *
 * Return:		0 on success, or negative on failure.
 */
int kbase_csf_queue_group_handle_is_valid(struct kbase_context *kctx, u8 group_handle);

/**
 * kbase_csf_queue_group_clear_faults - Re-enable CS Fault reporting.
 *
 * @kctx:	Pointer to the kbase context within which the
 *		CS Faults for the queues has to be re-enabled.
 * @clear_faults:	Pointer to the structure which contains details of the
 *		queues for which the CS Fault reporting has to be re-enabled.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_queue_group_clear_faults(struct kbase_context *kctx,
				       struct kbase_ioctl_queue_group_clear_faults *clear_faults);

/**
 * kbase_csf_queue_group_create - Create a GPU command queue group.
 *
 * @kctx:	Pointer to the kbase context within which the
 *		queue group is to be created.
 * @create:	Pointer to the structure which contains details of the
 *		queue group which is to be created within the
 *		provided kbase context.
 *
 * Return:	0 on success, or negative on failure.
 */
int kbase_csf_queue_group_create(struct kbase_context *kctx,
				 union kbase_ioctl_cs_queue_group_create *create);

/**
 * kbase_csf_queue_group_terminate - Terminate a GPU command queue group.
 *
 * @kctx:		Pointer to the kbase context within which the
 *			queue group is to be terminated.
 * @group_handle:	Pointer to the structure which identifies the queue
 *			group which is to be terminated.
 */
void kbase_csf_queue_group_terminate(struct kbase_context *kctx, u8 group_handle);

/**
 * kbase_csf_term_descheduled_queue_group - Terminate a GPU command queue
 *                                          group that is not operational
 *                                          inside the scheduler.
 *
 * @group:	Pointer to the structure which identifies the queue
 *		group to be terminated. The function assumes that the caller
 *		is sure that the given group is not operational inside the
 *		scheduler. If in doubt, use its alternative:
 *		@ref kbase_csf_queue_group_terminate().
 */
void kbase_csf_term_descheduled_queue_group(struct kbase_queue_group *group);

#if IS_ENABLED(CONFIG_MALI_VECTOR_DUMP) || MALI_UNIT_TEST
/**
 * kbase_csf_queue_group_suspend - Suspend a GPU command queue group
 *
 * @kctx:		The kbase context for which the queue group is to be
 *			suspended.
 * @sus_buf:		Pointer to the structure which contains details of the
 *			user buffer and its kernel pinned pages.
 * @group_handle:	Handle for the group which uniquely identifies it within
 *			the context within which it was created.
 *
 * This function is used to suspend a queue group and copy the suspend buffer.
 *
 * Return:		0 on success or negative value if failed to suspend
 *			queue group and copy suspend buffer contents.
 */
int kbase_csf_queue_group_suspend(struct kbase_context *kctx,
				  struct kbase_suspend_copy_buffer *sus_buf, u8 group_handle);
#endif

/**
 * kbase_csf_add_group_fatal_error - Report a fatal group error to userspace
 *
 * @group:       GPU command queue group.
 * @err_payload: Error payload to report.
 */
void kbase_csf_add_group_fatal_error(struct kbase_queue_group *const group,
				     struct base_gpu_queue_group_error const *const err_payload);

/**
 * kbase_csf_interrupt - Handle interrupts issued by CSF firmware.
 *
 * @kbdev: The kbase device to handle an IRQ for
 * @val:   The value of JOB IRQ status register which triggered the interrupt
 */
void kbase_csf_interrupt(struct kbase_device *kbdev, u32 val);

/**
 * kbase_csf_handle_csg_sync_update - Handle SYNC_UPDATE notification for the group.
 *
 * @kbdev: The kbase device to handle the SYNC_UPDATE interrupt.
 * @group_id: CSG index.
 * @group: Pointer to the GPU command queue group.
 * @req:   CSG_REQ register value corresponding to @group.
 * @ack:   CSG_ACK register value corresponding to @group.
 */
void kbase_csf_handle_csg_sync_update(struct kbase_device *const kbdev, u32 group_id,
				      struct kbase_queue_group *group, u32 req, u32 ack);

/**
 * kbase_csf_doorbell_mapping_init - Initialize the fields that facilitates
 *                                   the update of userspace mapping of HW
 *                                   doorbell page.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * The function creates a file and allocates a dummy page to facilitate the
 * update of userspace mapping to point to the dummy page instead of the real
 * HW doorbell page after the suspend of queue group.
 *
 * Return: 0 on success, or negative on failure.
 */
int kbase_csf_doorbell_mapping_init(struct kbase_device *kbdev);

/**
 * kbase_csf_doorbell_mapping_term - Free the dummy page & close the file used
 *                         to update the userspace mapping of HW doorbell page
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 */
void kbase_csf_doorbell_mapping_term(struct kbase_device *kbdev);

/**
 * kbase_csf_setup_dummy_user_reg_page - Setup the dummy page that is accessed
 *                                       instead of the User register page after
 *                                       the GPU power down.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * The function allocates a dummy page which is used to replace the User
 * register page in the userspace mapping after the power down of GPU.
 * On the power up of GPU, the mapping is updated to point to the real
 * User register page. The mapping is used to allow access to LATEST_FLUSH
 * register from userspace.
 *
 * Return: 0 on success, or negative on failure.
 */
int kbase_csf_setup_dummy_user_reg_page(struct kbase_device *kbdev);

/**
 * kbase_csf_free_dummy_user_reg_page - Free the dummy page that was used
 *                                      to replace the User register page
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 */
void kbase_csf_free_dummy_user_reg_page(struct kbase_device *kbdev);

/**
 * kbase_csf_pending_gpuq_kick_queues_init - Initialize the data used for handling
 *                                           GPU queue kicks.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 */
void kbase_csf_pending_gpuq_kick_queues_init(struct kbase_device *kbdev);

/**
 * kbase_csf_pending_gpuq_kick_queues_term - De-initialize the data used for handling
 *                                           GPU queue kicks.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 */
void kbase_csf_pending_gpuq_kick_queues_term(struct kbase_device *kbdev);

/**
 * kbase_csf_ring_csg_doorbell - ring the doorbell for a CSG interface.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @slot: Index of CSG interface for ringing the door-bell.
 *
 * The function kicks a notification on the CSG interface to firmware.
 */
void kbase_csf_ring_csg_doorbell(struct kbase_device *kbdev, int slot);

/**
 * kbase_csf_ring_csg_slots_doorbell - ring the doorbell for a set of CSG
 *                                     interfaces.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @slot_bitmap: bitmap for the given slots, slot-0 on bit-0, etc.
 *
 * The function kicks a notification on a set of CSG interfaces to firmware.
 */
void kbase_csf_ring_csg_slots_doorbell(struct kbase_device *kbdev, u32 slot_bitmap);

/**
 * kbase_csf_ring_cs_kernel_doorbell - ring the kernel doorbell for a CSI
 *                                     assigned to a GPU queue
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @csi_index: ID of the CSI assigned to the GPU queue.
 * @csg_nr:    Index of the CSG slot assigned to the queue
 *             group to which the GPU queue is bound.
 * @ring_csg_doorbell: Flag to indicate if the CSG doorbell needs to be rung
 *                     after updating the CSG_DB_REQ. So if this flag is false
 *                     the doorbell interrupt will not be sent to FW.
 *                     The flag is supposed be false only when the input page
 *                     for bound GPU queues is programmed at the time of
 *                     starting/resuming the group on a CSG slot.
 *
 * The function sends a doorbell interrupt notification to the firmware for
 * a CSI assigned to a GPU queue.
 */
void kbase_csf_ring_cs_kernel_doorbell(struct kbase_device *kbdev, int csi_index, int csg_nr,
				       bool ring_csg_doorbell);

/**
 * kbase_csf_ring_cs_user_doorbell - ring the user doorbell allocated for a
 *                                   queue.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @queue: Pointer to the queue for ringing the door-bell.
 *
 * The function kicks a notification to the firmware on the doorbell assigned
 * to the queue.
 */
void kbase_csf_ring_cs_user_doorbell(struct kbase_device *kbdev, struct kbase_queue *queue);

/**
 * kbase_csf_active_queue_groups_reset - Reset the state of all active GPU
 *                            command queue groups associated with the context.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @kctx:  The kbase context.
 *
 * This function will iterate through all the active/scheduled GPU command
 * queue groups associated with the context, deschedule and mark them as
 * terminated (which will then lead to unbinding of all the queues bound to
 * them) and also no more work would be allowed to execute for them.
 *
 * This is similar to the action taken in response to an unexpected OoM event.
 */
void kbase_csf_active_queue_groups_reset(struct kbase_device *kbdev, struct kbase_context *kctx);

/**
 * kbase_csf_priority_check - Check the priority requested
 *
 * @kbdev:        Device pointer
 * @req_priority: Requested priority
 *
 * This will determine whether the requested priority can be satisfied.
 *
 * Return: The same or lower priority than requested.
 */
u8 kbase_csf_priority_check(struct kbase_device *kbdev, u8 req_priority);

extern const u8 kbasep_csf_queue_group_priority_to_relative[BASE_QUEUE_GROUP_PRIORITY_COUNT];
extern const u8 kbasep_csf_relative_to_queue_group_priority[KBASE_QUEUE_GROUP_PRIORITY_COUNT];

/**
 * kbase_csf_priority_relative_to_queue_group_priority - Convert relative to base priority
 *
 * @priority: kbase relative priority
 *
 * This will convert the monotonically increasing realtive priority to the
 * fixed base priority list.
 *
 * Return: base_queue_group_priority priority.
 */
static inline u8 kbase_csf_priority_relative_to_queue_group_priority(u8 priority)
{
	if (priority >= KBASE_QUEUE_GROUP_PRIORITY_COUNT)
		priority = KBASE_QUEUE_GROUP_PRIORITY_LOW;
	return kbasep_csf_relative_to_queue_group_priority[priority];
}

/**
 * kbase_csf_priority_queue_group_priority_to_relative - Convert base priority to relative
 *
 * @priority: base_queue_group_priority priority
 *
 * This will convert the fixed base priority list to monotonically increasing realtive priority.
 *
 * Return: kbase relative priority.
 */
static inline u8 kbase_csf_priority_queue_group_priority_to_relative(u8 priority)
{
	/* Apply low priority in case of invalid priority */
	if (priority >= BASE_QUEUE_GROUP_PRIORITY_COUNT)
		priority = BASE_QUEUE_GROUP_PRIORITY_LOW;
	return kbasep_csf_queue_group_priority_to_relative[priority];
}

/**
 * kbase_csf_ktrace_gpu_cycle_cnt - Wrapper to retrieve the GPU cycle counter
 *                                  value for Ktrace purpose.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * This function is just a wrapper to retrieve the GPU cycle counter value, to
 * avoid any overhead on Release builds where Ktrace is disabled by default.
 *
 * Return: Snapshot of the GPU cycle count register.
 */
static inline u64 kbase_csf_ktrace_gpu_cycle_cnt(struct kbase_device *kbdev)
{
#if KBASE_KTRACE_ENABLE
	return kbase_backend_get_cycle_cnt(kbdev);
#else
	CSTD_UNUSED(kbdev);
	return 0;
#endif
}

/**
 * kbase_csf_process_queue_kick() - Process a pending kicked GPU command queue.
 *
 * @queue: Pointer to the queue to process.
 *
 * This function starts the pending queue, for which the work
 * was previously submitted via ioctl call from application thread.
 * If the queue is already scheduled and resident, it will be started
 * right away, otherwise once the group is made resident.
 */
void kbase_csf_process_queue_kick(struct kbase_queue *queue);

/**
 * kbase_csf_process_protm_event_request - Handle protected mode switch request
 *
 * @group: The group to handle protected mode request
 *
 * Request to switch to protected mode.
 */
void kbase_csf_process_protm_event_request(struct kbase_queue_group *group);

/**
 * kbase_csf_glb_fatal_worker - Worker function for handling GLB FATAL error
 *
 * @data: Pointer to a work_struct embedded in kbase device.
 *
 * Handle the GLB fatal error
 */
void kbase_csf_glb_fatal_worker(struct work_struct *const data);

/**
 * kbase_csf_queue_oom_state_str() - Helper function to get string
 *                                   for kbase queue OoM tracking state.
 * @state: kbase OoM track state
 *
 * Return: string representation of kbase OoM track state
 */
static inline const char *kbase_csf_queue_oom_state_str(enum kbase_csf_queue_oom_state state)
{
	switch (state) {
	case KBASE_CSF_QUEUE_OOM_NONE:
		return "KBASE_CSF_QUEUE_OOM_NONE";
	case KBASE_CSF_QUEUE_OOM_PENDING:
		return "KBASE_CSF_QUEUE_OOM_PENDING";
	case KBASE_CSF_QUEUE_OOM_COMPLETE:
		return "KBASE_CSF_QUEUE_OOM_COMPLETE";
	case KBASE_CSF_QUEUE_OOM_ERROR_ABORT:
		return "KBASE_CSF_QUEUE_OOM_ERROR_ABORT";
	default:
		return "[UnknownState]";
	}
}

/**
 * kbase_csf_cs_get_pending_oom - Get the data for the pending OoM event.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 * @queue: Pointer to the queue to process.
 * @slot_id: Slot index where the CSG is residing.
 *
 * The OoM data and the request state is saved in the queue's OoM tracking
 * structure.
 *
 * Return: 0 on success,
 *         -EINVAL if slot_id is invalid, or tiler OoM state is incorrect.
 *         -EBUSY if tiler OoM is already in KBASE_CSF_QUEUE_OOM_PENDING state.
 */
int kbase_csf_cs_get_pending_oom(struct kbase_device *kbdev, struct kbase_queue *queue,
				 int const slot_id);

/**
 * kbase_csf_program_cs_oom_prepared_chunk - Program the prepared OoM chunk to CSF.
 *
 * @queue: Pointer to the queue to process.
 * @slot_id: Slot index where the CSG is residing.
 * @cs_oom_req: Value for CS_REQ reg to clear the pending Tiler OoM request.
 *
 */
void kbase_csf_program_cs_oom_prepared_chunk(struct kbase_queue *queue, u32 slot_id,
					     u32 cs_oom_req);

/**
 * kbase_csf_cs_prepare_pending_oom_tiler_heap_chunk - Prepare the chunk for
 * the pending Tiler OoM event.
 *
 * @queue: Pointer to the queue to process.
 *
 * The pointer to the allocated chunk is saved in the queue's OoM tracking data and
 * the OoM tracking state is updated.
 *
 * Return: 0 on success,
 *         -EINVAL if tiler OoM tracking state is incorrect.
 *         negative error code on allocation failure.
 */
int kbase_csf_cs_prepare_pending_oom_tiler_heap_chunk(struct kbase_queue *queue);

/**
 * kbase_csf_free_oom_tiler_heap_chunk - Free the allocated tiler OoM chunk
 *
 * @queue: Pointer to the queue to process.
 *
 * This function should be used to free the allocated chunk if the chunk can't be
 * programmed to FW. OoM tracking data and state are updated.
 *
 * Return: 0 on success,
 *         -EINVAL if tiler OoM tracking state is incorrect.
 *         negative error code on freeing failure.
 */
int kbase_csf_free_oom_tiler_heap_chunk(struct kbase_queue *queue);

/**
 * kbase_csf_handle_pending_oom_interrupt() - Handler for a tiler heap OoM request IRQ.
 *
 * @queue:    Pointer to queue for which OoM event was received.
 * @group_id: CSG index.
 *
 * Get pending OoM request and enqueue the OoM event work.
 *
 * Return: 0 on success,
 *         -EBUSY when trying to enqueue an already-queued OoM work.
 */
int kbase_csf_handle_pending_oom_interrupt(struct kbase_queue *const queue, u32 group_id);

/**
 * kbase_csf_report_cs_fault_info() - Assmble the CS fault event information.
 *
 * @queue:   Pointer to queue for which a fault event was received.
 * @slot_id: On-slot CSG index, where the queue fault was raised.
 * @atomic_ctx: Calling from an interrupt handler, or from a kthread.
 *
 * Assembles the CS fault information and prints it out in a meaningful way in the log. The
 * function is expected to be only called when the caller is notified with a valid CS fault
 * event and the queue/bound-csg resides in the given slot.
 */
void kbase_csf_report_cs_fault_info(struct kbase_queue *const queue, u32 slot_id, bool atomic_ctx);

/**
 * kbase_csf_report_cs_fatal_info() - Assmble the CS fatal information.
 *
 * @queue:    Pointer to queue for which fatal event was received.
 * @slot_id: On-slot CSG index, where the queue fatal error was raised.
 * @atomic_ctx: Calling from an interrupt handler, or from a kthread.
 *
 * Assembles the CS fatal information and prints it out in a meaningful way in the log. The
 * function is expected to be only called when the caller is notified with a valid CS fatal
 * error event and the queue/bound-csg resides in the given slot.
 *
 * Return: the extracted CS_FATAL_EXCEPTION_TYPE.
 */
u32 kbase_csf_report_cs_fatal_info(struct kbase_queue *const queue, u32 slot_id, bool atomic_ctx);

/**
 * kbase_csf_dev_has_ne - Report whether the device has Neural Engine support.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * Return: true on Neural Engine supported, otherwise false.
 */
static inline bool kbase_csf_dev_has_ne(struct kbase_device *kbdev)
{
	return kbdev->gpu_props.gpu_features.neural_engine;
}

/**
 * kbase_csf_dev_has_rtu - Report whether the device has Ray Traversal support.
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * Return: true if Ray Traversal supported, otherwise false.
 */
static inline bool kbase_csf_dev_has_rtu(struct kbase_device *kbdev)
{
	return kbdev->gpu_props.gpu_features.ray_traversal;
}
#endif /* _KBASE_CSF_H_ */
