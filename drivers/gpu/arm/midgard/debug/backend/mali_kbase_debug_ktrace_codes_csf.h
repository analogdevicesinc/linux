/* SPDX-License-Identifier: GPL-2.0 */
/*
 *
 * (C) COPYRIGHT 2020 ARM Limited. All rights reserved.
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

/*
 * ***** IMPORTANT: THIS IS NOT A NORMAL HEADER FILE         *****
 * *****            DO NOT INCLUDE DIRECTLY                  *****
 * *****            THE LACK OF HEADER GUARDS IS INTENTIONAL *****
 */

/*
 * The purpose of this header file is just to contain a list of trace code
 * identifiers
 *
 * When updating this file, also remember to update
 * mali_kbase_debug_linux_ktrace_csf.h
 *
 * IMPORTANT: THIS FILE MUST NOT BE USED FOR ANY OTHER PURPOSE OTHER THAN THAT
 * DESCRIBED IN mali_kbase_debug_ktrace_codes.h
 */

#if 0 /* Dummy section to avoid breaking formatting */
int dummy_array[] = {
#endif
	/*
	 * Generic CSF events
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(EVICT_CTX_SLOTS),
	/* info_val[0:7]   == fw version_minor
	 * info_val[15:8]  == fw version_major
	 * info_val[63:32] == fw version_hash
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(FIRMWARE_BOOT),
	KBASE_KTRACE_CODE_MAKE_CODE(FIRMWARE_REBOOT),
	KBASE_KTRACE_CODE_MAKE_CODE(SCHEDULER_TOCK),
	KBASE_KTRACE_CODE_MAKE_CODE(SCHEDULER_TICK),
	KBASE_KTRACE_CODE_MAKE_CODE(SCHEDULER_RESET),
	KBASE_KTRACE_CODE_MAKE_CODE(SCHEDULER_EXIT_PROTM),
	KBASE_KTRACE_CODE_MAKE_CODE(SYNC_UPDATE_EVENT),

	/*
	 * Group events
	 */
	/* info_val[2:0] == CSG_REQ state issued
	 * info_val[19:16] == as_nr
	 * info_val[63:32] == endpoint config (max number of endpoints allowed)
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_SLOT_START),
	/* info_val == CSG_REQ state issued */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_SLOT_STOP),
	/* info_val == CSG_ACK state */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_SLOT_STARTED),
	/* info_val == CSG_ACK state */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_SLOT_STOPPED),
	/* info_val == slot cleaned */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_SLOT_CLEANED),
	/* info_val == previous priority */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_PRIO_UPDATE),
	/* info_val == CSG_REQ ^ CSG_ACK */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_SYNC_UPDATE_INTERRUPT),
	/* info_val == CSG_REQ ^ CSG_ACK */
	KBASE_KTRACE_CODE_MAKE_CODE(CSG_IDLE_INTERRUPT),
	KBASE_KTRACE_CODE_MAKE_CODE(GROUP_SYNC_UPDATE_DONE),
	/* info_val == run state of the group */
	KBASE_KTRACE_CODE_MAKE_CODE(GROUP_DESCHEDULE),
	/* info_val == run state of the group */
	KBASE_KTRACE_CODE_MAKE_CODE(GROUP_SCHEDULE),
	/* info_val[31:0] == new run state of the evicted group
	 * info_val[63:32] == number of runnable groups
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(GROUP_EVICT_SCHED),
	KBASE_KTRACE_CODE_MAKE_CODE(SCHEDULER_ENTER_PROTM),
	/* info_val[31:0] == number of GPU address space slots in use
	 * info_val[63:32] == number of runnable groups
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(SCHEDULER_TOP_GRP),

	/*
	 * Group + Queue events
	 */
	/* info_val == queue->enabled */
	KBASE_KTRACE_CODE_MAKE_CODE(CSI_START),
	/* info_val == queue->enabled before stop */
	KBASE_KTRACE_CODE_MAKE_CODE(CSI_STOP),
	KBASE_KTRACE_CODE_MAKE_CODE(CSI_STOP_REQUESTED),
	/* info_val == CS_REQ ^ CS_ACK */
	KBASE_KTRACE_CODE_MAKE_CODE(CSI_FAULT_INTERRUPT),
	/* info_val == CS_REQ ^ CS_ACK */
	KBASE_KTRACE_CODE_MAKE_CODE(CSI_TILER_OOM_INTERRUPT),
	/* info_val == group->run_State (for group the queue is bound to) */
	KBASE_KTRACE_CODE_MAKE_CODE(QUEUE_START),
	KBASE_KTRACE_CODE_MAKE_CODE(QUEUE_STOP),

	/*
	 * KCPU queue events
	 */
	/* KTrace info_val == KCPU queue fence context
	 * KCPU extra_info_val == N/A.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(KCPU_QUEUE_NEW),
	/* KTrace info_val == Number of pending commands in KCPU queue when
	 * it is destroyed.
	 * KCPU extra_info_val == Number of CQS wait operations present in
	 * the KCPU queue when it is destroyed.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(KCPU_QUEUE_DESTROY),
	/* KTrace info_val == CQS event memory address
	 * KCPU extra_info_val == Upper 32 bits of event memory, i.e. contents
	 * of error field.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(CQS_SET),
	/* KTrace info_val == Number of CQS objects to be waited upon
	 * KCPU extra_info_val == N/A.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(CQS_WAIT_START),
	/* KTrace info_val == CQS event memory address
	 * KCPU extra_info_val == 1 if CQS was signaled with an error and queue
	 * inherited the error, otherwise 0.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(CQS_WAIT_END),
	/* KTrace info_val == Fence context
	 * KCPU extra_info_val == Fence seqno.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(FENCE_SIGNAL),
	/* KTrace info_val == Fence context
	 * KCPU extra_info_val == Fence seqno.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(FENCE_WAIT_START),
	/* KTrace info_val == Fence context
	 * KCPU extra_info_val == Fence seqno.
	 */
	KBASE_KTRACE_CODE_MAKE_CODE(FENCE_WAIT_END),

#if 0 /* Dummy section to avoid breaking formatting */
};
#endif

/* ***** THE LACK OF HEADER GUARDS IS INTENTIONAL ***** */
