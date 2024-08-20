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

/**
 * enum kbasep_csf_fw_io_status_bits - Status bits for firmware I/O interface.
 *
 * @KBASEP_FW_IO_STATUS_GPU_SUSPENDED: The GPU is suspended.
 * @KBASEP_FW_IO_STATUS_NUM_BITS: Number of bits used to encode the status.
 */
enum kbasep_csf_fw_io_status_bits {
	KBASEP_FW_IO_STATUS_GPU_SUSPENDED = 0,
	KBASEP_FW_IO_STATUS_NUM_BITS,
};

/**
 * struct kbasep_csf_fw_io_stream_pages - Addresses to CS I/O pages.
 *
 * @input: Address of CS input page.
 * @output: Address of CS output page.
 */
struct kbasep_csf_fw_io_stream_pages {
	void *input;
	void *output;
};

/**
 * struct kbasep_csf_fw_io_group_pages - Addresses to CSG I/O pages.
 *
 * @input: Address of CSG input page.
 * @output: Address of CSG output page.
 * @streams_pages: Array of CSs' I/O pages.
 */
struct kbasep_csf_fw_io_group_pages {
	void *input;
	void *output;
	struct kbasep_csf_fw_io_stream_pages *streams_pages;
};

/**
 * struct kbasep_csf_fw_io_pages - Addresses to FW I/O pages.
 *
 * @input: Address of global input page.
 * @output: Address of global output page.
 * @groups_pages: Array of CSGs' I/O pages.
 */
struct kbasep_csf_fw_io_pages {
	void *input;
	void *output;
	struct kbasep_csf_fw_io_group_pages *groups_pages;
};

/**
 * struct kbase_csf_fw_io - Manager of firmware input/output interface.
 *
 * @lock: Mutex to serialize access to the interface.
 * @status: Internal status of the MCU interface.
 * @pages: Addresses to FW I/O pages
 * @kbdev: Pointer to the instance of a GPU platform device that implements a CSF interface.
 */
struct kbase_csf_fw_io {
	spinlock_t lock;
	DECLARE_BITMAP(status, KBASEP_FW_IO_STATUS_NUM_BITS);
	struct kbasep_csf_fw_io_pages pages;
	struct kbase_device *kbdev;
};

/**
 * kbase_csf_fw_io_init() - Initialize manager of firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to initialize.
 * @kbdev: Pointer to the instance of a GPU platform device that implements a CSF interface.
 */
void kbase_csf_fw_io_init(struct kbase_csf_fw_io *fw_io, struct kbase_device *kbdev);

/**
 * kbase_csf_fw_io_term() - Terminate manager of firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to terminate.
 */
void kbase_csf_fw_io_term(struct kbase_csf_fw_io *fw_io);

/**
 * kbase_csf_fw_io_groups_pages_init() - Initialize an array for CSGs' I/O pages.
 *
 * @fw_io: Firmware I/O interface.
 * @group_num: Number of CSGs.
 *
 * Return: 0 on success, -ENOMEM on failed initialization.
 */
int kbase_csf_fw_io_groups_pages_init(struct kbase_csf_fw_io *fw_io, u32 group_num);

/**
 * kbase_csf_fw_io_streams_pages_init() - Initialize an array for CSs' I/O pages.
 *
 * @fw_io: Firmware I/O interface.
 * @group_id: CSG index.
 * @stream_num: Number of CSs.
 *
 * Return: 0 on success, -ENOMEM on failed initialization.
 */
int kbase_csf_fw_io_streams_pages_init(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_num);

/**
 * kbase_csf_fw_io_set_global_pages() - Set addresses to global I/O pages.
 *
 * @fw_io: Firmware I/O interface.
 * @input: Source address to global input page.
 * @output: Source address to global output page.
 */
void kbase_csf_fw_io_set_global_pages(struct kbase_csf_fw_io *fw_io, void *input, void *output);

/**
 * kbase_csf_fw_io_set_group_pages() - Set addresses to CSG's I/O pages.
 *
 * @fw_io: Firmware I/O interface.
 * @group_id: CSG index.
 * @input: Source address to CSG's input page.
 * @output: Source address to CSG's output page.
 */
void kbase_csf_fw_io_set_group_pages(struct kbase_csf_fw_io *fw_io, u32 group_id, void *input,
				     void *output);

/**
 * kbase_csf_fw_io_set_stream_pages() - Set addresses to CS's I/O pages.
 *
 * @fw_io: Firmware I/O interface.
 * @group_id: CSG index.
 * @stream_id: CS index.
 * @input: Source address to CS's input page.
 * @output: Source address to CS's output page.
 */
void kbase_csf_fw_io_set_stream_pages(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				      void *input, void *output);

/**
 * kbase_csf_fw_io_pages_term() - Terminate arrays of addresses to CSGs and CSs I/O pages.
 *
 * @fw_io: Firmware I/O interface.
 * @group_num: Number of CSGs.
 */
void kbase_csf_fw_io_pages_term(struct kbase_csf_fw_io *fw_io, u32 group_num);

/**
 * kbase_csf_fw_io_open() - Start a transaction with the firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to open.
 * @flags: Pointer to the memory location that would store the previous
 *	   interrupt state
 *
 * Return: 0 on success, otherwise an error code reflecting the status of the
 *         interface.
 */
static inline int kbase_csf_fw_io_open(struct kbase_csf_fw_io *fw_io, unsigned long *flags)
{
	if (test_bit(KBASEP_FW_IO_STATUS_GPU_SUSPENDED, fw_io->status))
		return -KBASE_CSF_FW_IO_WAIT_GPU_LOST;

	spin_lock_irqsave(&fw_io->lock, *flags);

	return 0;
}

/**
 * kbase_csf_fw_io_open_force() - Force a transaction with the firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to open.
 * @flags: Pointer to the memory location that would store the previous
 *	   interrupt state
 *
 * This function forces the start of a transaction regardless of the status
 * of the interface.
 */
static inline void kbase_csf_fw_io_open_force(struct kbase_csf_fw_io *fw_io, unsigned long *flags)
{
	spin_lock_irqsave(&fw_io->lock, *flags);
}

/**
 * kbase_csf_fw_io_close() - End a transaction with the firmware input/output interface.
 *
 * @fw_io: Firmware I/O interface to close.
 * @flags: Previously stored interrupt state when FW IO interrupt
 *	   spinlock was acquired.
 */
static inline void kbase_csf_fw_io_close(struct kbase_csf_fw_io *fw_io, unsigned long flags)
{
	spin_unlock_irqrestore(&fw_io->lock, flags);
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
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_global_write(struct kbase_csf_fw_io *fw_io, u32 offset, u32 value);

/**
 * kbase_csf_fw_io_global_write_mask() - Write part of a word in the global input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 * @mask:   Bitmask with the bits to be modified set.
 */
void kbase_csf_fw_io_global_write_mask(struct kbase_csf_fw_io *fw_io, u32 offset, u32 value,
				       u32 mask);

/**
 * kbase_csf_fw_io_global_input_read() - Read a word in the global input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the global input page.
 */
u32 kbase_csf_fw_io_global_input_read(struct kbase_csf_fw_io *fw_io, u32 offset);

/**
 * kbase_csf_fw_io_global_read() - Read a word in the global output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the global output page.
 */
u32 kbase_csf_fw_io_global_read(struct kbase_csf_fw_io *fw_io, u32 offset);

/**
 * kbase_csf_fw_io_group_write() - Write a word in a CSG's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id: CSG index.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_group_write(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset,
				 u32 value);

/**
 * kbase_csf_fw_io_group_write_mask() - Write part of a word in a CSG's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id: CSG index.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 * @mask:   Bitmask with the bits to be modified set.
 */
void kbase_csf_fw_io_group_write_mask(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset,
				      u32 value, u32 mask);

/**
 * kbase_csf_fw_io_group_input_read() - Read a word in a CSG's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id: CSG index.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from a CSG's input page.
 */
u32 kbase_csf_fw_io_group_input_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset);

/**
 * kbase_csf_fw_io_group_read() - Read a word in a CSG's output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id: CSG index.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the CSG's output page.
 */
u32 kbase_csf_fw_io_group_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset);


/**
 * kbase_csf_fw_io_stream_write() - Write a word in a CS's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id:  CSG index.
 * @stream_id: CS index.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_stream_write(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				  u32 offset, u32 value);

/**
 * kbase_csf_fw_io_stream_write_mask() - Write part of a word in a CS's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id:  CSG index.
 * @stream_id: CS index.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 * @mask:   Bitmask with the bits to be modified set.
 */
void kbase_csf_fw_io_stream_write_mask(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				       u32 offset, u32 value, u32 mask);

/**
 * kbase_csf_fw_io_stream_input_read() - Read a word in a CS's input page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id:  CSG index.
 * @stream_id: CS index.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from a CS's input page.
 */
u32 kbase_csf_fw_io_stream_input_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				      u32 offset);

/**
 * kbase_csf_fw_io_stream_read() - Read a word in a CS's output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id:  CSG index.
 * @stream_id: CS index.
 * @offset: Offset of the word to be read, in bytes.
 *
 * Return: Value of the word read from the CS's output page.
 */
u32 kbase_csf_fw_io_stream_read(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 stream_id,
				u32 offset);

/**
 * kbase_csf_fw_io_set_status_gpu_suspended() - Set GPU_SUSPENDED FW I/O status bit.
 *
 * @fw_io:      Firmware I/O manager.
 */
void kbase_csf_fw_io_set_status_gpu_suspended(struct kbase_csf_fw_io *fw_io);

/**
 * kbase_csf_fw_io_clear_status_gpu_suspended() - Clear GPU_SUSPENDED FW I/O status bit.
 *
 * @fw_io:      Firmware I/O manager.
 */
void kbase_csf_fw_io_clear_status_gpu_suspended(struct kbase_csf_fw_io *fw_io);

/**
 * kbase_csf_fw_io_check_status_gpu_suspended() - Check if GPU_SUSPENDED FW I/O status bit is set.
 *
 * @fw_io: Firmware I/O manager.
 *
 * Return: True if GPU_SUSPENDED FW I/O status bit is set, false otherwise.
 */
bool kbase_csf_fw_io_check_status_gpu_suspended(struct kbase_csf_fw_io *fw_io);

/**
 * kbase_csf_fw_io_wait_event_timeout() - Wait until condition gets true, timeout
 * occurs or a GPU_SUSPENDED FW I/O status bit is set. The rest of the functionalities is equal
 * to wait_event_timeout().
 *
 * @fw_io:     Firmware I/O manager.
 * @wq_head:   The waitqueue to wait on.
 * @condition: C expression for the event to wait for
 * @timeout:   Timeout, in jiffies
 *
 * Return: Remaining jiffies (at least 1) on success,
 *         0 on timeout,
 *         negative KBASE_CSF_FW_IO_WAIT_LOST error if GPU_SUSPENDED FW I/O status bit is set.
 */
#define kbase_csf_fw_io_wait_event_timeout(fw_io, wq_head, condition, timeout)                     \
	({                                                                                         \
		int __ret;                                                                         \
		int __wait_remaining = wait_event_timeout(                                         \
			wq_head, (condition) || kbase_csf_fw_io_check_status_gpu_suspended(fw_io), \
			timeout);                                                                  \
		__ret = kbasep_csf_fw_io_handle_wait_result(fw_io, __wait_remaining);              \
		__ret;                                                                             \
	})

/**
 * kbasep_csf_fw_io_handle_wait_result() - Private function to handle the wait_event_timeout()
 * result.
 *
 * @fw_io:          Firmware I/O manager
 * @wait_remaining: Remaining jiffies returned by wait_event_timeout()
 *
 * Return: Remaining jiffies (at least 1) on success,
 *         0 on timeout,
 *         negative KBASE_CSF_FW_IO_WAIT_LOST error if GPU_SUSPENDED FW I/O status bit is set.
 */
static inline int kbasep_csf_fw_io_handle_wait_result(struct kbase_csf_fw_io *fw_io,
						      int wait_remaining)
{
	return kbase_csf_fw_io_check_status_gpu_suspended(fw_io) ? -KBASE_CSF_FW_IO_WAIT_GPU_LOST :
									 wait_remaining;
}

#if IS_ENABLED(CONFIG_MALI_DEBUG) || IS_ENABLED(CONFIG_MALI_NO_MALI)
/**
 * kbase_csf_fw_io_mock_fw_global_write() - Mock a FW write to the global output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_mock_fw_global_write(struct kbase_csf_fw_io *fw_io, u32 offset, u32 value);

/**
 * kbase_csf_fw_io_mock_fw_group_write() - Mock a FW write to CSG's output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id:  CSG index.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_mock_fw_group_write(struct kbase_csf_fw_io *fw_io, u32 group_id, u32 offset,
					 u32 value);

/**
 * kbase_csf_fw_io_mock_fw_stream_write() - Mock a FW write to CS's output page.
 *
 * @fw_io:  Firmware I/O manager.
 * @group_id:  CSG index.
 * @stream_id: CS index.
 * @offset: Offset of the word to write, in bytes.
 * @value:  Value to be written.
 */
void kbase_csf_fw_io_mock_fw_stream_write(struct kbase_csf_fw_io *fw_io, u32 group_id,
					  u32 stream_id, u32 offset, u32 value);

#endif /* IS_ENABLED(CONFIG_MALI_DEBUG) || IS_ENABLED(CONFIG_MALI_NO_MALI) */

#endif /* _KBASE_CSF_FW_IO_H_ */
