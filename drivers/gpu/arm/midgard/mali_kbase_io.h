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

#ifndef _KBASE_IO_H_
#define _KBASE_IO_H_

#include <linux/sched.h>
#include <linux/types.h>
#include <linux/wait.h>

struct kbase_device;
struct kbase_io;

/**
 * enum kbase_io_status_bits - Status bits for kbase I/O interface.
 *
 * @KBASE_IO_STATUS_GPU_SUSPENDED: The GPU is suspended.
 * @KBASE_IO_STATUS_GPU_OFF: The GPU is OFF.
 * @KBASE_IO_STATUS_GPU_LOST: The GPU is LOST.
 * @KBASE_IO_STATUS_NUM_BITS: Number of bits used to encode the status.
 */
enum kbase_io_status_bits {
	KBASE_IO_STATUS_GPU_SUSPENDED = 0,
	KBASE_IO_STATUS_GPU_OFF,
	KBASE_IO_STATUS_GPU_LOST,
	KBASE_IO_STATUS_NUM_BITS,
};

/**
 * kbase_io_init() - Initialize manager of kbase input/output interface.
 *
 * @kbdev: Pointer to the instance of a GPU platform device.
 *
 * Return: 0 on success, -ENOMEM on failed initialization.
 */
int __must_check kbase_io_init(struct kbase_device *kbdev);

/**
 * kbase_io_term() - Terminate manager of kbase input/output interface.
 *
 * @kbdev: Pointer to the instance of a GPU platform device.
 */
void kbase_io_term(struct kbase_device *kbdev);

/**
 * kbase_io_set_status() - Set a kbase I/O status bit.
 *
 * @io:         Kbase I/O structure
 * @status_bit: Status bit to set.
 */
void kbase_io_set_status(struct kbase_io *io, enum kbase_io_status_bits status_bit);

/**
 * kbase_io_clear_status() - Clear a kbase I/O status bit.
 *
 * @io:         Kbase I/O structure
 * @status_bit: Status bit to clear.
 */
void kbase_io_clear_status(struct kbase_io *io, enum kbase_io_status_bits status_bit);

/**
 * kbase_io_test_status() - Test a kbase I/O status bit.
 *
 * @kbdev: Pointer to kbase device structure.
 * @status_bit: Status bit to test.
 *
 * Return: Value of the tested status bit.
 */
bool kbase_io_test_status(struct kbase_device *kbdev, enum kbase_io_status_bits status_bit);

/**
 * kbase_io_is_gpu_powered() - Check if the GPU is powered or not
 *
 * @kbdev: Pointer to kbase device structure.
 *
 * Return: TRUE if the gpu is powered ON or FALSE otherwise.
 */
bool kbase_io_is_gpu_powered(struct kbase_device *kbdev);

/**
 * kbase_io_is_gpu_lost() - Check if the GPU is lost
 *
 * @kbdev: Pointer to kbase device structure.
 *
 * Return: TRUE if the gpu is lost or FALSE otherwise.
 */
bool kbase_io_is_gpu_lost(struct kbase_device *kbdev);

/**
 * kbase_io_has_gpu() - Check if GPU is available
 *
 * @kbdev: Pointer to kbase device structure.
 *
 * Return: This function checks and returns TRUE if GPU is powered on or otherwise FALSE.
 */
bool kbase_io_has_gpu(struct kbase_device *kbdev);

#if defined(CONFIG_DEBUG_FS) && !IS_ENABLED(CONFIG_MALI_NO_MALI)

/**
 * kbase_io_history_init - initialize data struct for register access history
 *
 * @h: The register history to initialize
 * @n: The number of register accesses that the buffer could hold
 *
 * Return: 0 if successfully initialized, failure otherwise
 */
int kbase_io_history_init(struct kbase_io_history *h, u16 n);

/**
 * kbase_io_history_term - uninit all resources for the register access history
 *
 * @h: The register history to terminate
 */
void kbase_io_history_term(struct kbase_io_history *h);

/**
 * kbase_io_history_dump - print the register history to the kernel ring buffer
 *
 * @kbdev: Pointer to kbase_device containing the register history to dump
 */
void kbase_io_history_dump(struct kbase_device *kbdev);

/**
 * kbasep_regs_history_debugfs_init - add debugfs entries for register history
 *
 * @kbdev: Pointer to kbase_device containing the register history
 */
void kbasep_regs_history_debugfs_init(struct kbase_device *kbdev);

/* kbase_io_history_add - add new entry to the register access history
 *
 * @h: Pointer to the history data structure
 * @addr: Register address
 * @value: The value that is either read from or written to the register
 * @write: 1 if it's a register write, 0 if it's a read
 */
void kbase_io_history_add(struct kbase_io_history *h, void __iomem const *addr, u32 value,
			  u8 write);

#else /* !defined(CONFIG_DEBUG_FS) || IS_ENABLED(CONFIG_MALI_NO_MALI) */

#define kbase_io_history_init(...) (0)

#define kbase_io_history_term CSTD_NOP

#define kbase_io_history_dump CSTD_NOP

#define kbasep_regs_history_debugfs_init CSTD_NOP

#endif /* defined(CONFIG_DEBUG_FS) && !IS_ENABLED(CONFIG_MALI_NO_MALI) */

#endif /* _KBASE_IO_H_ */
