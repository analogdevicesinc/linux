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
#include "mali_kbase_csf_sync.h"
#include "mali_kbase_csf_util.h"
#include <mali_kbase.h>
#include <linux/version_compat_defs.h>

#if IS_ENABLED(CONFIG_SYNC_FILE)
#include "mali_kbase_sync.h"
#endif

#define CQS_UNREADABLE_LIVE_VALUE "(unavailable)"

#define CSF_SYNC_DUMP_SIZE 256

/* Number of nearby commands around the "extract_ptr" of GPU queues.
 *
 *     [extract_ptr - MAX_NR_NEARBY_INSTR, extract_ptr + MAX_NR_NEARBY_INSTR].
 */
#define MAX_NR_NEARBY_INSTR 32

/**
 * kbasep_csf_sync_get_cqs_live_u32() - Obtain live (u32) value for a CQS object.
 *
 * @kctx:     The context of the queue.
 * @obj_addr: Pointer to the CQS live 32-bit value.
 * @live_val: Pointer to the u32 that will be set to the CQS object's current, live
 *            value.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int kbasep_csf_sync_get_cqs_live_u32(struct kbase_context *kctx, u64 obj_addr, u32 *live_val)
{
	struct kbase_vmap_struct *mapping;
	u32 *const cpu_ptr = (u32 *)kbase_phy_alloc_mapping_get(kctx, obj_addr, &mapping);

	if (!cpu_ptr)
		return -1;

	*live_val = *cpu_ptr;
	kbase_phy_alloc_mapping_put(kctx, mapping);
	return 0;
}

/**
 * kbasep_csf_sync_get_cqs_live_u64() - Obtain live (u64) value for a CQS object.
 *
 * @kctx:     The context of the queue.
 * @obj_addr: Pointer to the CQS live value (32 or 64-bit).
 * @live_val: Pointer to the u64 that will be set to the CQS object's current, live
 *            value.
 *
 * Return: 0 if successful or a negative error code on failure.
 */
static int kbasep_csf_sync_get_cqs_live_u64(struct kbase_context *kctx, u64 obj_addr, u64 *live_val)
{
	struct kbase_vmap_struct *mapping;
	u64 *cpu_ptr = (u64 *)kbase_phy_alloc_mapping_get(kctx, obj_addr, &mapping);

	if (!cpu_ptr)
		return -1;

	*live_val = *cpu_ptr;
	kbase_phy_alloc_mapping_put(kctx, mapping);
	return 0;
}

/**
 * kbasep_csf_sync_print_kcpu_fence_wait_or_signal() - Print details of a CSF SYNC Fence Wait
 *                                                     or Fence Signal command, contained in a
 *                                                     KCPU queue.
 *
 * @buffer:   The buffer to write to.
 * @length:   The length of text in the buffer.
 * @cmd:      The KCPU Command to be printed.
 * @cmd_name: The name of the command: indicates either a fence SIGNAL or WAIT.
 */
static void kbasep_csf_sync_print_kcpu_fence_wait_or_signal(char *buffer, int *length,
							    struct kbase_kcpu_command *cmd,
							    const char *cmd_name)
{
	struct dma_fence *fence = NULL;
	struct kbase_kcpu_command_fence_info *fence_info;
	struct kbase_sync_fence_info info;
	const char *timeline_name = NULL;
	bool is_signaled = false;

	fence_info = &cmd->info.fence;
	if (kbase_kcpu_command_fence_has_force_signaled(fence_info))
		return;

	fence = kbase_fence_get(fence_info);
	if (WARN_ON(!fence))
		return;

	kbase_sync_fence_info_get(fence, &info);
	timeline_name = fence->ops->get_timeline_name(fence);
	is_signaled = info.status > 0;

	*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
			    "cmd:%s obj:0x%pK live_value:0x%.8x | ", cmd_name, fence, is_signaled);

	/* Note: fence->seqno was u32 until 5.1 kernel, then u64 */
	*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
			    "timeline_name:%s timeline_context:0x%.16llx fence_seqno:0x%.16llx",
			    timeline_name, fence->context, (u64)fence->seqno);

	kbase_fence_put(fence);
}

/**
 * kbasep_csf_sync_print_kcpu_cqs_wait() - Print details of a CSF SYNC CQS Wait command,
 *                                         contained in a KCPU queue.
 *
 * @kctx:   The kbase context.
 * @buffer: The buffer to write to.
 * @length: The length of text in the buffer.
 * @cmd:    The KCPU Command to be printed.
 */
static void kbasep_csf_sync_print_kcpu_cqs_wait(struct kbase_context *kctx, char *buffer,
						int *length, struct kbase_kcpu_command *cmd)
{
	size_t i;

	for (i = 0; i < cmd->info.cqs_wait.nr_objs; i++) {
		struct base_cqs_wait_info *cqs_obj = &cmd->info.cqs_wait.objs[i];

		u32 live_val;
		int ret = kbasep_csf_sync_get_cqs_live_u32(kctx, cqs_obj->addr, &live_val);
		bool live_val_valid = (ret >= 0);

		*length +=
			snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				 "cmd:CQS_WAIT_OPERATION obj:0x%.16llx live_value:", cqs_obj->addr);

		if (live_val_valid)
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    "0x%.16llx", (u64)live_val);
		else
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    CQS_UNREADABLE_LIVE_VALUE);

		*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				    " | op:gt arg_value:0x%.8x", cqs_obj->val);
	}
}

/**
 * kbasep_csf_sync_print_kcpu_cqs_set() - Print details of a CSF SYNC CQS
 *                                        Set command, contained in a KCPU queue.
 *
 * @kctx:   The kbase context.
 * @buffer: The buffer to write to.
 * @length: The length of text in the buffer.
 * @cmd:    The KCPU Command to be printed.
 */
static void kbasep_csf_sync_print_kcpu_cqs_set(struct kbase_context *kctx, char *buffer,
					       int *length, struct kbase_kcpu_command *cmd)
{
	size_t i;

	for (i = 0; i < cmd->info.cqs_set.nr_objs; i++) {
		struct base_cqs_set *cqs_obj = &cmd->info.cqs_set.objs[i];

		u32 live_val;
		int ret = kbasep_csf_sync_get_cqs_live_u32(kctx, cqs_obj->addr, &live_val);
		bool live_val_valid = (ret >= 0);

		*length +=
			snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				 "cmd:CQS_SET_OPERATION obj:0x%.16llx live_value:", cqs_obj->addr);

		if (live_val_valid)
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    "0x%.16llx", (u64)live_val);
		else
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    CQS_UNREADABLE_LIVE_VALUE);

		*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				    " | op:add arg_value:0x%.8x", 1);
	}
}

/**
 * kbasep_csf_sync_get_wait_op_name() - Print the name of a CQS Wait Operation.
 *
 * @op: The numerical value of operation.
 *
 * Return: const static pointer to the command name, or '??' if unknown.
 */
static const char *kbasep_csf_sync_get_wait_op_name(basep_cqs_wait_operation_op op)
{
	const char *string;

	switch (op) {
	case BASEP_CQS_WAIT_OPERATION_LE:
		string = "le";
		break;
	case BASEP_CQS_WAIT_OPERATION_GT:
		string = "gt";
		break;
	default:
		string = "??";
		break;
	}
	return string;
}

/**
 * kbasep_csf_sync_get_set_op_name() - Print the name of a CQS Set Operation.
 *
 * @op: The numerical value of operation.
 *
 * Return: const static pointer to the command name, or '??' if unknown.
 */
static const char *kbasep_csf_sync_get_set_op_name(basep_cqs_set_operation_op op)
{
	const char *string;

	switch (op) {
	case BASEP_CQS_SET_OPERATION_ADD:
		string = "add";
		break;
	case BASEP_CQS_SET_OPERATION_SET:
		string = "set";
		break;
	default:
		string = "???";
		break;
	}
	return string;
}

/**
 * kbasep_csf_sync_print_kcpu_cqs_wait_op() - Print details of a CSF SYNC CQS
 *                                            Wait Operation command, contained
 *                                            in a KCPU queue.
 *
 * @kctx:   The kbase context.
 * @buffer: The buffer to write to.
 * @length: The length of text in the buffer.
 * @cmd:    The KCPU Command to be printed.
 */
static void kbasep_csf_sync_print_kcpu_cqs_wait_op(struct kbase_context *kctx, char *buffer,
						   int *length, struct kbase_kcpu_command *cmd)
{
	size_t i;

	for (i = 0; i < cmd->info.cqs_wait.nr_objs; i++) {
		struct base_cqs_wait_operation_info *wait_op =
			&cmd->info.cqs_wait_operation.objs[i];
		const char *op_name = kbasep_csf_sync_get_wait_op_name(wait_op->operation);

		u64 live_val;
		int ret = kbasep_csf_sync_get_cqs_live_u64(kctx, wait_op->addr, &live_val);

		bool live_val_valid = (ret >= 0);

		*length +=
			snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				 "cmd:CQS_WAIT_OPERATION obj:0x%.16llx live_value:", wait_op->addr);

		if (live_val_valid)
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    "0x%.16llx", live_val);
		else
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    CQS_UNREADABLE_LIVE_VALUE);

		*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				    " | op:%s arg_value:0x%.16llx", op_name, wait_op->val);
	}
}

/**
 * kbasep_csf_sync_print_kcpu_cqs_set_op() - Print details of a CSF SYNC CQS
 *                                           Set Operation command, contained
 *                                           in a KCPU queue.
 *
 * @kctx:   The kbase context.
 * @buffer: The buffer to write to.
 * @length: The length of text in the buffer.
 * @cmd:    The KCPU Command to be printed.
 */
static void kbasep_csf_sync_print_kcpu_cqs_set_op(struct kbase_context *kctx, char *buffer,
						  int *length, struct kbase_kcpu_command *cmd)
{
	size_t i;

	for (i = 0; i < cmd->info.cqs_set_operation.nr_objs; i++) {
		struct base_cqs_set_operation_info *set_op = &cmd->info.cqs_set_operation.objs[i];
		const char *op_name = kbasep_csf_sync_get_set_op_name(
			(basep_cqs_set_operation_op)set_op->operation);

		u64 live_val;
		int ret = kbasep_csf_sync_get_cqs_live_u64(kctx, set_op->addr, &live_val);

		bool live_val_valid = (ret >= 0);

		*length +=
			snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				 "cmd:CQS_SET_OPERATION obj:0x%.16llx live_value:", set_op->addr);

		if (live_val_valid)
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    "0x%.16llx", live_val);
		else
			*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
					    CQS_UNREADABLE_LIVE_VALUE);

		*length += snprintf(buffer + *length, CSF_SYNC_DUMP_SIZE - *length,
				    " | op:%s arg_value:0x%.16llx", op_name, set_op->val);
	}
}

/**
 * kbasep_csf_sync_kcpu_print_queue() - Print debug data for a KCPU queue
 *
 * @kctx:  The kbase context.
 * @kbpr:  Pointer to printer instance.
 * @queue: Pointer to the KCPU queue.
 */
static void kbasep_csf_sync_kcpu_print_queue(struct kbase_context *kctx,
					     struct kbase_kcpu_command_queue *queue,
					     struct kbasep_printer *kbpr)
{
	char started_or_pending;
	struct kbase_kcpu_command *cmd;
	size_t i;

	if (WARN_ON(!queue))
		return;

	lockdep_assert_held(&kctx->csf.kcpu_queues.lock);
	mutex_lock(&queue->lock);

	for (i = 0; i != queue->num_pending_cmds; ++i) {
		char buffer[CSF_SYNC_DUMP_SIZE];
		int length = 0;

		started_or_pending = ((i == 0) && queue->command_started) ? 'S' : 'P';
		length += snprintf(buffer, CSF_SYNC_DUMP_SIZE, "queue:KCPU-%d-%d exec:%c ",
				   kctx->id, queue->id, started_or_pending);

		cmd = &queue->commands[(u8)(queue->start_offset + i)];
		switch (cmd->type) {
#if IS_ENABLED(CONFIG_SYNC_FILE)
		case BASE_KCPU_COMMAND_TYPE_FENCE_SIGNAL:
			kbasep_csf_sync_print_kcpu_fence_wait_or_signal(buffer, &length, cmd,
									"FENCE_SIGNAL");
			break;
		case BASE_KCPU_COMMAND_TYPE_FENCE_WAIT:
			kbasep_csf_sync_print_kcpu_fence_wait_or_signal(buffer, &length, cmd,
									"FENCE_WAIT");
			break;
#endif
		case BASE_KCPU_COMMAND_TYPE_CQS_WAIT:
			kbasep_csf_sync_print_kcpu_cqs_wait(kctx, buffer, &length, cmd);
			break;
		case BASE_KCPU_COMMAND_TYPE_CQS_SET:
			kbasep_csf_sync_print_kcpu_cqs_set(kctx, buffer, &length, cmd);
			break;
		case BASE_KCPU_COMMAND_TYPE_CQS_WAIT_OPERATION:
			kbasep_csf_sync_print_kcpu_cqs_wait_op(kctx, buffer, &length, cmd);
			break;
		case BASE_KCPU_COMMAND_TYPE_CQS_SET_OPERATION:
			kbasep_csf_sync_print_kcpu_cqs_set_op(kctx, buffer, &length, cmd);
			break;
		default:
			length += snprintf(buffer + length, CSF_SYNC_DUMP_SIZE - length,
					   ", U, Unknown blocking command");
			break;
		}

		length += snprintf(buffer + length, CSF_SYNC_DUMP_SIZE - length, "\n");
		kbasep_print(kbpr, buffer);
	}

	mutex_unlock(&queue->lock);
}

int kbasep_csf_sync_kcpu_dump_print(struct kbase_context *kctx, struct kbasep_printer *kbpr)
{
	unsigned long queue_idx;

	mutex_lock(&kctx->csf.kcpu_queues.lock);

	kbasep_print(kbpr, "CSF KCPU queues sync info (version: v" __stringify(
				   MALI_CSF_SYNC_DUMP_VERSION) "):\n");

	kbasep_print(kbpr, "KCPU queues for ctx %d:\n", kctx->id);

	queue_idx = find_first_bit(kctx->csf.kcpu_queues.in_use, KBASEP_MAX_KCPU_QUEUES);

	while (queue_idx < KBASEP_MAX_KCPU_QUEUES) {
		kbasep_csf_sync_kcpu_print_queue(kctx, kctx->csf.kcpu_queues.array[queue_idx],
						 kbpr);

		queue_idx = find_next_bit(kctx->csf.kcpu_queues.in_use, KBASEP_MAX_KCPU_QUEUES,
					  queue_idx + 1);
	}

	mutex_unlock(&kctx->csf.kcpu_queues.lock);

	return 0;
}

/* GPU queue related values */
#define GPU_CSF_MOVE_OPCODE ((u64)0x1)
#define GPU_CSF_MOVE32_OPCODE ((u64)0x2)
#define GPU_CSF_SYNC_ADD_OPCODE ((u64)0x25)
#define GPU_CSF_SYNC_SET_OPCODE ((u64)0x26)
#define GPU_CSF_SYNC_WAIT_OPCODE ((u64)0x27)
#define GPU_CSF_SYNC_ADD64_OPCODE ((u64)0x33)
#define GPU_CSF_SYNC_SET64_OPCODE ((u64)0x34)
#define GPU_CSF_SYNC_WAIT64_OPCODE ((u64)0x35)
#define GPU_CSF_CALL_OPCODE ((u64)0x20)

#define MAX_NR_GPU_CALLS (5)
#define INSTR_OPCODE_MASK ((u64)0xFF << 56)
#define INSTR_OPCODE_GET(value) ((value & INSTR_OPCODE_MASK) >> 56)
#define MOVE32_IMM_MASK ((u64)0xFFFFFFFFFUL)
#define MOVE_DEST_MASK ((u64)0xFF << 48)
#define MOVE_DEST_GET(value) ((value & MOVE_DEST_MASK) >> 48)
#define MOVE_IMM_MASK ((u64)0xFFFFFFFFFFFFUL)
#define SYNC_SRC0_MASK ((u64)0xFF << 40)
#define SYNC_SRC1_MASK ((u64)0xFF << 32)
#define SYNC_SRC0_GET(value) (u8)((value & SYNC_SRC0_MASK) >> 40)
#define SYNC_SRC1_GET(value) (u8)((value & SYNC_SRC1_MASK) >> 32)
#define SYNC_WAIT_CONDITION_MASK ((u64)0xF << 28)
#define SYNC_WAIT_CONDITION_GET(value) (u8)((value & SYNC_WAIT_CONDITION_MASK) >> 28)

/* Enumeration for types of GPU queue sync events for
 * the purpose of dumping them through sync.
 */
enum sync_gpu_sync_type {
	CSF_GPU_SYNC_WAIT,
	CSF_GPU_SYNC_SET,
	CSF_GPU_SYNC_ADD,
	NUM_CSF_GPU_SYNC_TYPES
};

/**
 * kbasep_csf_get_move_immediate_value() - Get the immediate values for sync operations
 *                                         from a MOVE instruction.
 *
 * @move_cmd:        Raw MOVE instruction.
 * @sync_addr_reg:   Register identifier from SYNC_* instruction.
 * @compare_val_reg: Register identifier from SYNC_* instruction.
 * @sync_val:        Pointer to store CQS object address for sync operation.
 * @compare_val:     Pointer to store compare value for sync operation.
 *
 * Return: True if value is obtained by checking for correct register identifier,
 * or false otherwise.
 */
static bool kbasep_csf_get_move_immediate_value(u64 move_cmd, u64 sync_addr_reg,
						u64 compare_val_reg, u64 *sync_val,
						u64 *compare_val)
{
	u64 imm_mask;

	/* Verify MOVE instruction and get immediate mask */
	if (INSTR_OPCODE_GET(move_cmd) == GPU_CSF_MOVE32_OPCODE)
		imm_mask = MOVE32_IMM_MASK;
	else if (INSTR_OPCODE_GET(move_cmd) == GPU_CSF_MOVE_OPCODE)
		imm_mask = MOVE_IMM_MASK;
	else
		/* Error return */
		return false;

	/* Verify value from MOVE instruction and assign to variable */
	if (sync_addr_reg == MOVE_DEST_GET(move_cmd))
		*sync_val = move_cmd & imm_mask;
	else if (compare_val_reg == MOVE_DEST_GET(move_cmd))
		*compare_val = move_cmd & imm_mask;
	else
		/* Error return */
		return false;

	return true;
}

/** kbasep_csf_read_ringbuffer_value() - Reads a u64 from the ringbuffer at a provided
 *                                       offset.
 *
 * @queue:            Pointer to the queue.
 * @ringbuff_offset:  Ringbuffer offset.
 *
 * Return: the u64 in the ringbuffer at the desired offset.
 */
static u64 kbasep_csf_read_ringbuffer_value(struct kbase_queue *queue, u32 ringbuff_offset)
{
	u64 page_off = ringbuff_offset >> PAGE_SHIFT;
	u64 offset_within_page = ringbuff_offset & ~PAGE_MASK;
	struct page *page = as_page(queue->queue_reg->gpu_alloc->pages[page_off]);
	u64 *ringbuffer = vmap(&page, 1, VM_MAP, pgprot_noncached(PAGE_KERNEL));
	u64 value;

	if (!ringbuffer) {
		struct kbase_context *kctx = queue->kctx;

		dev_err(kctx->kbdev->dev, "%s failed to map the buffer page for read a command!",
			__func__);
		/* Return an alternative 0 for dumping operation*/
		value = 0;
	} else {
		value = ringbuffer[offset_within_page / sizeof(u64)];
		vunmap(ringbuffer);
	}

	return value;
}

/**
 * kbasep_csf_print_gpu_sync_op() - Print sync operation info for given sync command.
 *
 * @kbpr:             Pointer to printer instance.
 * @kctx:             Pointer to kbase context.
 * @queue:            Pointer to the GPU command queue.
 * @ringbuff_offset:  Offset to index the ring buffer with, for the given sync command.
 *                    (Useful for finding preceding MOVE commands)
 * @instr_addr:       GPU command address.
 * @sync_cmd:         Entire u64 of the sync command, which has both sync address and
 *                    comparison-value encoded in it.
 * @type:             Type of GPU sync command (e.g. SYNC_SET, SYNC_ADD, SYNC_WAIT).
 * @is_64bit:         Bool to indicate if operation is 64 bit (true) or 32 bit (false).
 * @follows_wait:     Bool to indicate if the operation follows at least one wait
 *                    operation. Used to determine whether it's pending or started.
 */
static void kbasep_csf_print_gpu_sync_op(struct kbasep_printer *kbpr, struct kbase_context *kctx,
					 struct kbase_queue *queue, u32 ringbuff_offset,
					 u64 instr_addr, u64 sync_cmd, enum sync_gpu_sync_type type,
					 bool is_64bit, bool follows_wait)
{
	u64 sync_addr = 0, compare_val = 0, live_val = 0, ringbuffer_boundary_check;
	u64 move_cmd;
	u8 sync_addr_reg, compare_val_reg, wait_condition = 0;
	int err;

	static const char *const gpu_sync_type_name[] = { "SYNC_WAIT", "SYNC_SET", "SYNC_ADD" };
	static const char *const gpu_sync_type_op[] = {
		"wait", /* This should never be printed, only included to simplify indexing */
		"set", "add"
	};

	if (type >= NUM_CSF_GPU_SYNC_TYPES) {
		dev_warn(kctx->kbdev->dev, "Expected GPU queue sync type is unknown!");
		return;
	}

	/* 1. Get Register identifiers from SYNC_* instruction */
	sync_addr_reg = SYNC_SRC0_GET(sync_cmd);
	compare_val_reg = SYNC_SRC1_GET(sync_cmd);

	if (ringbuff_offset < sizeof(u64)) {
		dev_warn(kctx->kbdev->dev,
			 "Unexpected wraparound detected between %s & MOVE instruction",
			 gpu_sync_type_name[type]);
		return;
	}
	/* 2. Get values from first MOVE command */
	ringbuff_offset -= sizeof(u64);
	move_cmd = kbasep_csf_read_ringbuffer_value(queue, ringbuff_offset);

	/* We expect there to be at least 2 preceding MOVE instructions for CQS, or 3 preceding
	 * MOVE instructions for Timeline CQS, and Base will always arrange for these
	 * MOVE + SYNC instructions to be contiguously located, and is therefore never expected
	 * to be wrapped around the ringbuffer boundary. The following check takes place after
	 * the ringbuffer has been decremented, and already points to the first MOVE command,
	 * so that it can be determined if it's a 32-bit MOVE (so 2 vs 1 preceding MOVE commands
	 * will be checked).
	 * This is to maintain compatibility with older userspace; a check is done to ensure that
	 * the MOVE opcode found was a 32-bit MOVE, and if so, it has determined that a newer
	 * userspace is being used and will continue to read the next 32-bit MOVE to recover the
	 * compare/set value in the wait/set operation. If not, the single 48-bit value found
	 * will be used.
	 */
	ringbuffer_boundary_check =
		(INSTR_OPCODE_GET(move_cmd) == GPU_CSF_MOVE32_OPCODE && is_64bit) ? 2 : 1;
	if (unlikely(ringbuff_offset < (ringbuffer_boundary_check * sizeof(u64)))) {
		dev_warn(kctx->kbdev->dev,
			 "Unexpected wraparound detected between %s & MOVE instruction",
			 gpu_sync_type_name[type]);
		return;
	}
	/* For 64-bit SYNC commands, the first MOVE command read in will actually use 1 register
	 * above the compare value register in the sync command, as this will store the higher
	 * 32-bits of 64-bit compare value. The compare value register read above will be read
	 * afterwards.
	 */
	if (!kbasep_csf_get_move_immediate_value(move_cmd, sync_addr_reg,
						 compare_val_reg + (is_64bit ? 1 : 0), &sync_addr,
						 &compare_val))
		return;

	/* 64-bit WAITs or SETs are split into 2 32-bit MOVEs. sync_val would contain the higher
	 * 32 bits, so the lower 32-bits are retrieved afterwards, to recover the full u64 value.
	 */
	if (INSTR_OPCODE_GET(move_cmd) == GPU_CSF_MOVE32_OPCODE && is_64bit) {
		u64 compare_val_lower = 0;

		ringbuff_offset -= sizeof(u64);
		move_cmd = kbasep_csf_read_ringbuffer_value(queue, ringbuff_offset);

		if (!kbasep_csf_get_move_immediate_value(move_cmd, sync_addr_reg, compare_val_reg,
							 &sync_addr, &compare_val_lower))
			return;
		/* Mask off upper 32 bits of compare_val_lower, and combine with the higher 32 bits
		 * to restore the original u64 compare value.
		 */
		compare_val = (compare_val << 32) | (compare_val_lower & ((u64)U32_MAX));
	}

	/* 3. Get values from next MOVE command, which should be the CQS object address */
	ringbuff_offset -= sizeof(u64);
	move_cmd = kbasep_csf_read_ringbuffer_value(queue, ringbuff_offset);
	if (!kbasep_csf_get_move_immediate_value(move_cmd, sync_addr_reg, compare_val_reg,
						 &sync_addr, &compare_val))
		return;

	/* 4. Get CQS object value */
	if (is_64bit)
		err = kbasep_csf_sync_get_cqs_live_u64(kctx, sync_addr, &live_val);
	else
		err = kbasep_csf_sync_get_cqs_live_u32(kctx, sync_addr, (u32 *)(&live_val));

	if (err)
		return;

	/* 5. Print info */
	kbasep_print(kbpr, "queue:GPU-%u-%u-%u exec:%c at:0x%.16llx cmd:%s ", kctx->id,
		     queue->group->handle, queue->csi_index,
		     queue->enabled && !follows_wait ? 'S' : 'P', instr_addr,
		     gpu_sync_type_name[type]);

	if (queue->group->csg_nr == KBASEP_CSG_NR_INVALID)
		kbasep_print(kbpr, "slot:-");
	else
		kbasep_print(kbpr, "slot:%d", (int)queue->group->csg_nr);

	kbasep_print(kbpr, " obj:0x%.16llx live_value:0x%.16llx | ", sync_addr, live_val);

	if (type == CSF_GPU_SYNC_WAIT) {
		wait_condition = SYNC_WAIT_CONDITION_GET(sync_cmd);
		kbasep_print(kbpr, "op:%s ", kbasep_csf_sync_get_wait_op_name(wait_condition));
	} else
		kbasep_print(kbpr, "op:%s ", gpu_sync_type_op[type]);

	kbasep_print(kbpr, "arg_value:0x%.16llx\n", compare_val);
}

/**
 * kbasep_csf_dump_active_queue_sync_info() - Print GPU command queue sync information.
 *
 * @kbpr:  Pointer to printer instance.
 * @queue: Address of a GPU command queue to examine.
 *
 * This function will iterate through each command in the ring buffer of the given GPU queue from
 * CS_EXTRACT, and if is a SYNC_* instruction it will attempt to decode the sync operation and
 * print relevant information to the sync file.
 * This function will stop iterating once the CS_INSERT address is reached by the cursor (i.e.
 * when there are no more commands to view) or a number of consumed GPU CALL commands have
 * been observed.
 */
static void kbasep_csf_dump_active_queue_sync_info(struct kbasep_printer *kbpr,
						   struct kbase_queue *queue)
{
	struct kbase_context *kctx;
	u64 *addr;
	u64 cs_extract, cs_insert, instr, cursor, end_cursor;
	u32 nr_nearby_instr_size;
	bool follows_wait = false;
	int nr_calls = 0;

	if (!queue)
		return;

	kctx = queue->kctx;

	addr = queue->user_io_addr;
	cs_insert = addr[CS_INSERT_LO / sizeof(*addr)];

	addr = queue->user_io_addr + PAGE_SIZE / sizeof(*addr);
	cs_extract = addr[CS_EXTRACT_LO / sizeof(*addr)];

	nr_nearby_instr_size =
		min((MAX_NR_NEARBY_INSTR * (u32)sizeof(u64)), ((queue->size / 2) & ~(0x7u)));
	cursor = (cs_extract + queue->size - nr_nearby_instr_size) & ((u64)queue->size - 1);
	end_cursor = min(cs_insert, ((cs_extract + nr_nearby_instr_size) & ((u64)queue->size - 1)));

	if (!is_power_of_2(queue->size)) {
		dev_warn(kctx->kbdev->dev, "GPU queue %u size of %u not a power of 2",
			 queue->csi_index, queue->size);
		return;
	}

	kbasep_print(
		kbpr,
		"queue:GPU-%u-%u-%u size:%u cs_insert:%8llx cs_extract:%8llx dump_begin:%8llx dump_end:%8llx\n",
		kctx->id, queue->group->handle, queue->csi_index, queue->size, cs_insert,
		cs_extract, cursor, end_cursor);

	while ((cs_insert != cs_extract) && (cursor != end_cursor) &&
	       (nr_calls < MAX_NR_GPU_CALLS)) {
		bool instr_is_64_bit = false;
		u32 cursor_ringbuff_offset = (u32)cursor;

		/* Find instruction that cursor is currently on */
		instr = kbasep_csf_read_ringbuffer_value(queue, cursor_ringbuff_offset);

		switch (INSTR_OPCODE_GET(instr)) {
		case GPU_CSF_SYNC_ADD64_OPCODE:
		case GPU_CSF_SYNC_SET64_OPCODE:
		case GPU_CSF_SYNC_WAIT64_OPCODE:
			instr_is_64_bit = true;
			break;
		default:
			break;
		}
		switch (INSTR_OPCODE_GET(instr)) {
		case GPU_CSF_SYNC_ADD_OPCODE:
		case GPU_CSF_SYNC_ADD64_OPCODE:
			kbasep_csf_print_gpu_sync_op(kbpr, kctx, queue, cursor_ringbuff_offset,
						     cursor, instr, CSF_GPU_SYNC_ADD,
						     instr_is_64_bit, follows_wait);
			break;
		case GPU_CSF_SYNC_SET_OPCODE:
		case GPU_CSF_SYNC_SET64_OPCODE:
			kbasep_csf_print_gpu_sync_op(kbpr, kctx, queue, cursor_ringbuff_offset,
						     cursor, instr, CSF_GPU_SYNC_SET,
						     instr_is_64_bit, follows_wait);
			break;
		case GPU_CSF_SYNC_WAIT_OPCODE:
		case GPU_CSF_SYNC_WAIT64_OPCODE:
			kbasep_csf_print_gpu_sync_op(kbpr, kctx, queue, cursor_ringbuff_offset,
						     cursor, instr, CSF_GPU_SYNC_WAIT,
						     instr_is_64_bit, follows_wait);
			follows_wait = true; /* Future commands will follow at least one wait */
			break;
		case GPU_CSF_CALL_OPCODE:
			nr_calls++;
			kbasep_print(kbpr,
				     "queue:GPU-%u-%u-%u exec:%c at:0x%.16llx cmd:0x%.16llx\n",
				     kctx->id, queue->group->handle, queue->csi_index,
				     queue->enabled && !follows_wait ? 'S' : 'P', cursor, instr);
			break;
		default:
			/* NOP instructions without metadata are not printed. */
			if (instr) {
				kbasep_print(
					kbpr,
					"queue:GPU-%u-%u-%u exec:%c at:0x%.16llx cmd:0x%.16llx\n",
					kctx->id, queue->group->handle, queue->csi_index,
					queue->enabled && !follows_wait ? 'S' : 'P', cursor, instr);
			}
			break;
		}

		cursor = (cursor + sizeof(u64)) & ((u64)queue->size - 1);
	}
}

/**
 * kbasep_csf_dump_active_group_sync_state() - Prints SYNC commands in all GPU queues of
 *                                             the provided queue group.
 *
 * @kctx:  The kbase context
 * @kbpr:  Pointer to printer instance.
 * @group: Address of a GPU command group to iterate through.
 *
 * This function will iterate through each queue in the provided GPU queue group and
 * print its SYNC related commands.
 */
static void kbasep_csf_dump_active_group_sync_state(struct kbase_context *kctx,
						    struct kbasep_printer *kbpr,
						    struct kbase_queue_group *const group)
{
	unsigned int i;

	kbasep_print(kbpr, "GPU queues for group %u (slot %d) of ctx %d_%d\n", group->handle,
		     group->csg_nr, kctx->tgid, kctx->id);

	for (i = 0; i < MAX_SUPPORTED_STREAMS_PER_GROUP; i++)
		kbasep_csf_dump_active_queue_sync_info(kbpr, group->bound_queues[i]);
}

int kbasep_csf_sync_gpu_dump_print(struct kbase_context *kctx, struct kbasep_printer *kbpr)
{
	u32 gr;
	struct kbase_device *kbdev;

	if (WARN_ON(!kctx))
		return -EINVAL;

	kbdev = kctx->kbdev;
	kbase_csf_scheduler_lock(kbdev);
	kbase_csf_csg_update_status(kbdev);

	kbasep_print(kbpr, "CSF GPU queues sync info (version: v" __stringify(
				   MALI_CSF_SYNC_DUMP_VERSION) "):\n");

	for (gr = 0; gr < kbdev->csf.global_iface.group_num; gr++) {
		struct kbase_queue_group *const group =
			kbdev->csf.scheduler.csg_slots[gr].resident_group;
		if (!group || group->kctx != kctx)
			continue;
		kbasep_csf_dump_active_group_sync_state(kctx, kbpr, group);
	}

	kbase_csf_scheduler_unlock(kbdev);

	return 0;
}
