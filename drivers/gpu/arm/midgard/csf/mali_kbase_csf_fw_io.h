/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2024 ARM Limited. All rights reserved.
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

#ifndef _KBASE_CSF_FW_IO_H_
#define _KBASE_CSF_FW_IO_H_

#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/spinlock.h>

/** The wait completed because the GPU was lost. */
#define KBASE_CSF_FW_IO_WAIT_GPU_LOST 1

/** The wait was aborted because of an unexpected event. */
#define KBASE_CSF_FW_IO_WAIT_UNSUPPORTED 255

/**
 * enum kbase_csf_fw_io_status_bits - Status bits for firmware I/O interface.
 *
 * @KBASE_FW_IO_STATUS_GPU_SUSPENDED: The GPU is suspended.
 * @KBASE_FW_IO_STATUS_NUM_BITS: Number of bits used to encode the state.
 */
enum kbase_csf_fw_io_status_bits {
	KBASE_FW_IO_STATUS_GPU_SUSPENDED = 0,
	KBASE_FW_IO_STATUS_NUM_BITS,
};

/**
 * struct kbase_csf_fw_io - Manager of firmware input/output interface.
 *
 * @lock: Mutex to serialize access to the interface.
 * @status: Internal status of the MCU interface.
 */
struct kbase_csf_fw_io {
	spinlock_t lock;
	DECLARE_BITMAP(status, KBASE_FW_IO_STATUS_NUM_BITS);
};

struct kbase_csf_global_iface;
struct kbase_csf_cmd_stream_group_info;
struct kbase_csf_cmd_stream_info;

/**
 * kbase_csf_fw_io_init() - Initialize manager of firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to initialize.
 */
void kbase_csf_fw_io_init(struct kbase_csf_fw_io *fw_io);

/**
 * kbase_csf_fw_io_term() - Terminate manager of firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to terminate.
 */
void kbase_csf_fw_io_term(struct kbase_csf_fw_io *fw_io);

/**
 * kbase_csf_fw_io_open() - Start a transaction with the firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to open.
 *
 * Return: 0 on success, otherwise an error code reflecting the status of the
 *         interface.
 */
static inline int kbase_csf_fw_io_open(struct kbase_csf_fw_io *fw_io)
{
	if (test_bit(KBASE_FW_IO_STATUS_GPU_SUSPENDED, fw_io->status))
		return -KBASE_CSF_FW_IO_WAIT_GPU_LOST;

	spin_lock(&fw_io->lock);

	return 0;
}

/**
 * kbase_csf_fw_io_open_force() - Force a transaction with the firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to open.
 *
 * This function forces the start of a transaction regardless of the status
 * of the interface.
 */
static inline void kbase_csf_fw_io_open_force(struct kbase_csf_fw_io *fw_io)
{
	spin_lock(&fw_io->lock);
}

/**
 * kbase_csf_fw_io_close() - End a transaction with the firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to close.
 */
static inline void kbase_csf_fw_io_close(struct kbase_csf_fw_io *fw_io)
{
	spin_unlock(&fw_io->lock);
}

/**
 * kbase_csf_fw_io_assert_opened() - Assert if a transaction with the firmware input/output
 *                                   interface has started.
 *
 * @fw_io: Firmware I/O interface.
 */
static inline void kbase_csf_fw_io_assert_opened(struct kbase_csf_fw_io *fw_io)
{
	lockdep_assert_held(&fw_io->lock);
}

/**
 * kbase_csf_fw_io_global_write() - Write a word in the global input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @iface:  CSF interface provided by the firmware.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_global_write(struct kbase_csf_fw_io *fw_io,
				  const struct kbase_csf_global_iface *iface, u32 offset,
				  u32 value);

/**
 * kbase_csf_fw_io_global_write_mask() - Write part of a word in the global input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @iface:  CSF interface provided by the firmware.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 * @mask:   Bitmask with the bits to be modified set.
 */
void kbase_csf_fw_io_global_write_mask(struct kbase_csf_fw_io *fw_io,
				       const struct kbase_csf_global_iface *iface, u32 offset,
				       u32 value, u32 mask);

/**
 * kbase_csf_fw_io_global_input_read() - Read a word in the global input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @iface:  CSF interface provided by the firmware.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the global input page.
 */
u32 kbase_csf_fw_io_global_input_read(struct kbase_csf_fw_io *fw_io,
				      const struct kbase_csf_global_iface *iface, u32 offset);

/**
 * kbase_csf_fw_io_global_read() - Read a word in the global output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @iface:  CSF interface provided by the firmware.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the global output page.
 */
u32 kbase_csf_fw_io_global_read(struct kbase_csf_fw_io *fw_io,
				const struct kbase_csf_global_iface *iface, u32 offset);

/**
 * kbase_csf_fw_io_group_write() - Write a word in a CSG's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSG interface provided by the firmware.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_group_write(struct kbase_csf_fw_io *fw_io,
				 const struct kbase_csf_cmd_stream_group_info *info, u32 offset,
				 u32 value);

/**
 * kbase_csf_fw_io_group_write_mask() - Write part of a word in a CSG's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSG interface provided by the firmware.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 * @mask:   Bitmask with the bits to be modified set.
 */
void kbase_csf_fw_io_group_write_mask(struct kbase_csf_fw_io *fw_io,
				      const struct kbase_csf_cmd_stream_group_info *info,
				      u32 offset, u32 value, u32 mask);

/**
 * kbase_csf_fw_io_group_input_read() - Read a word in a CSG's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSG interface provided by the firmware.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from a CSG's input page.
 */
u32 kbase_csf_fw_io_group_input_read(struct kbase_csf_fw_io *fw_io,
				     const struct kbase_csf_cmd_stream_group_info *info,
				     u32 offset);

/**
 * kbase_csf_fw_io_group_read() - Read a word in a CSG's output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSG interface provided by the firmware.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the CSG's output page.
 */
u32 kbase_csf_fw_io_group_read(struct kbase_csf_fw_io *fw_io,
			       const struct kbase_csf_cmd_stream_group_info *info, u32 offset);

/**
 * kbase_csf_fw_io_stream_write() - Write a word in a CS's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSI interface provided by the firmware.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_stream_write(struct kbase_csf_fw_io *fw_io,
				  const struct kbase_csf_cmd_stream_info *info, u32 offset,
				  u32 value);

/**
 * kbase_csf_fw_io_stream_write_mask() - Write part of a word in a CS's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSI interface provided by the firmware.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 * @mask:   Bitmask with the bits to be modified set.
 */
void kbase_csf_fw_io_stream_write_mask(struct kbase_csf_fw_io *fw_io,
				       const struct kbase_csf_cmd_stream_info *info, u32 offset,
				       u32 value, u32 mask);

/**
 * kbase_csf_fw_io_stream_input_read() - Read a word in a CS's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSI interface provided by the firmware.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from a CS's input page.
 */
u32 kbase_csf_fw_io_stream_input_read(struct kbase_csf_fw_io *fw_io,
				      const struct kbase_csf_cmd_stream_info *info, u32 offset);

/**
 * kbase_csf_fw_io_stream_read() - Read a word in a CS's output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @info:   CSI interface provided by the firmware.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the CS's output page.
 */
u32 kbase_csf_fw_io_stream_read(struct kbase_csf_fw_io *fw_io,
				const struct kbase_csf_cmd_stream_info *info, u32 offset);

/**
 * kbase_csf_fw_io_set_status() - Set a FW I/O status bit.
 *
 * @fw_io:      Firmware I/O manager.
 * @status_bit: Status bit to set.
 */
void kbase_csf_fw_io_set_status(struct kbase_csf_fw_io *fw_io,
				enum kbase_csf_fw_io_status_bits status_bit);

/**
 * kbase_csf_fw_io_clear_status() - Clear a FW I/O status bit.
 *
 * @fw_io:      Firmware I/O manager.
 * @status_bit: Status bit to clear.
 */
void kbase_csf_fw_io_clear_status(struct kbase_csf_fw_io *fw_io,
				  enum kbase_csf_fw_io_status_bits status_bit);

/**
 * kbase_csf_fw_io_test_status() - Test a FW I/O status bit.
 *
 * @fw_io:      Firmware I/O manager.
 * @status_bit: Status bit to test.
 *
 * Return: Value of the tested status bit.
 */
bool kbase_csf_fw_io_test_status(struct kbase_csf_fw_io *fw_io,
				 enum kbase_csf_fw_io_status_bits status_bit);

/**
 * kbase_csf_fw_io_wait_event_timeout() - Wait until condition gets true, timeout
 * occurs or a FW I/O status bit is set. The rest of the functionalities is equal
 * to wait_event_timeout().
 *
 * @fw_io:     Firmware I/O manager.
 * @wq_head:   The waitqueue to wait on.
 * @condition: C expression for the event to wait for
 * @timeout:   Timeout, in jiffies
 *
 * Return: Remaining jiffies (at least 1) on success,
 *         0 on timeout,
 *         negative KBASE_CSF_FW_IO_WAIT_* error codes otherwise.
 */
#define kbase_csf_fw_io_wait_event_timeout(fw_io, wq_head, condition, timeout)                \
	({                                                                                    \
		int __ret;                                                                    \
		int __wait_remaining = wait_event_timeout(                                    \
			wq_head, condition || kbasep_csf_fw_io_check_status(fw_io), timeout); \
		__ret = kbasep_csf_fw_io_handle_wait_result(fw_io, __wait_remaining);         \
		__ret;                                                                        \
	})

/**
 * kbasep_csf_fw_io_check_status() - Private function to check if any FW I/O status bit is set.
 *
 * @fw_io: Firmware I/O manager.
 *
 * Return: True if any FW I/O status bit is set, false otherwise.
 */
static inline bool kbasep_csf_fw_io_check_status(struct kbase_csf_fw_io *fw_io)
{
	return !bitmap_empty(fw_io->status, KBASE_FW_IO_STATUS_NUM_BITS);
}

/**
 * kbasep_csf_fw_io_handle_wait_result() - Private function to handle the wait_event_timeout()
 * result.
 *
 * @fw_io:          Firmware I/O manager
 * @wait_remaining: Remaining jiffies returned by wait_event_timeout()
 *
 * Return: Remaining jiffies (at least 1) on success,
 *         0 on timeout,
 *         negative KBASE_CSF_FW_IO_WAIT_* error codes otherwise.
 */
static inline int kbasep_csf_fw_io_handle_wait_result(struct kbase_csf_fw_io *fw_io,
						      int wait_remaining)
{
	/* Check for any FW IO status bit set */
	if (!bitmap_empty(fw_io->status, KBASE_FW_IO_STATUS_NUM_BITS))
		return (test_bit(KBASE_FW_IO_STATUS_GPU_SUSPENDED, fw_io->status)) ?
				     -KBASE_CSF_FW_IO_WAIT_GPU_LOST :
				     -KBASE_CSF_FW_IO_WAIT_UNSUPPORTED;

	return wait_remaining;
}
#endif
