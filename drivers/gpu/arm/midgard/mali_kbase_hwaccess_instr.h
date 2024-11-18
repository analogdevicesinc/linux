/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2014-2015, 2017-2018, 2020-2021 ARM Limited. All rights reserved.
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
 * HW Access instrumentation common APIs
 */

#ifndef _KBASE_HWACCESS_INSTR_H_
#define _KBASE_HWACCESS_INSTR_H_

#include <backend/gpu/mali_kbase_instr_defs.h>

/**
 * struct kbase_instr_hwcnt_enable - Enable hardware counter collection.
 * @dump_buffer:       GPU address to write counters to.
 * @dump_buffer_bytes: Size in bytes of the buffer pointed to by dump_buffer.
 * @fe_bm:             counters selection bitmask (Front End).
 * @shader_bm:         counters selection bitmask (Shader).
 * @tiler_bm:          counters selection bitmask (Tiler).
 * @mmu_l2_bm:         counters selection bitmask (MMU_L2).
 * @counter_set:       the performance counter set to use.
 */
struct kbase_instr_hwcnt_enable {
	u64 dump_buffer;
	u64 dump_buffer_bytes;
	u32 fe_bm;
	u32 shader_bm;
	u32 tiler_bm;
	u32 mmu_l2_bm;
	u8 counter_set;
};

/**
 * kbase_instr_hwcnt_enable_internal() - Enable HW counters collection
 * @kbdev:	Kbase device
 * @kctx:	Kbase context
 * @enable:	HW counter setup parameters
 *
 * Context: might sleep, waiting for reset to complete
 *
 * Return: 0 on success
 */
int kbase_instr_hwcnt_enable_internal(struct kbase_device *kbdev,
				struct kbase_context *kctx,
				struct kbase_instr_hwcnt_enable *enable);

/**
 * kbase_instr_hwcnt_disable_internal() - Disable HW counters collection
 * @kctx: Kbase context
 *
 * Context: might sleep, waiting for an ongoing dump to complete
 *
 * Return: 0 on success
 */
int kbase_instr_hwcnt_disable_internal(struct kbase_context *kctx);

/**
 * kbase_instr_hwcnt_request_dump() - Request HW counter dump from GPU
 * @kctx:	Kbase context
 *
 * Caller must either wait for kbase_instr_hwcnt_dump_complete() to return true,
 * of call kbase_instr_hwcnt_wait_for_dump().
 *
 * Return: 0 on success
 */
int kbase_instr_hwcnt_request_dump(struct kbase_context *kctx);

/**
 * kbase_instr_hwcnt_wait_for_dump() - Wait until pending HW counter dump has
 *				       completed.
 * @kctx:	Kbase context
 *
 * Context: will sleep, waiting for dump to complete
 *
 * Return: 0 on success
 */
int kbase_instr_hwcnt_wait_for_dump(struct kbase_context *kctx);

/**
 * kbase_instr_hwcnt_dump_complete - Tell whether the HW counters dump has
 *				     completed
 * @kctx:	Kbase context
 * @success:	Set to true if successful
 *
 * Context: does not sleep.
 *
 * Return: true if the dump is complete
 */
bool kbase_instr_hwcnt_dump_complete(struct kbase_context *kctx,
						bool * const success);

/**
 * kbase_instr_hwcnt_clear() - Clear HW counters
 * @kctx:	Kbase context
 *
 * Context: might sleep, waiting for reset to complete
 *
 * Return: 0 on success
 */
int kbase_instr_hwcnt_clear(struct kbase_context *kctx);

/**
 * kbase_instr_backend_init() - Initialise the instrumentation backend
 * @kbdev:	Kbase device
 *
 * This function should be called during driver initialization.
 *
 * Return: 0 on success
 */
int kbase_instr_backend_init(struct kbase_device *kbdev);

/**
 * kbase_instr_backend_term() - Terminate the instrumentation backend
 * @kbdev:	Kbase device
 *
 * This function should be called during driver termination.
 */
void kbase_instr_backend_term(struct kbase_device *kbdev);

#ifdef CONFIG_MALI_PRFCNT_SET_SELECT_VIA_DEBUG_FS
/**
 * kbase_instr_backend_debugfs_init() - Add a debugfs entry for the
 *                                      hardware counter set.
 * @kbdev: kbase device
 */
void kbase_instr_backend_debugfs_init(struct kbase_device *kbdev);
#endif

/**
 * kbase_instr_hwcnt_on_unrecoverable_error() - JM HWC instr backend function
 *                                              called when unrecoverable errors
 *                                              are detected.
 * @kbdev: Kbase device
 *
 * This should be called on encountering errors that can only be recovered from
 * with reset, or that may put HWC logic in state that could result in hang. For
 * example, when HW becomes unresponsive.
 *
 * Caller requires kbdev->hwaccess_lock held.
 */
void kbase_instr_hwcnt_on_unrecoverable_error(struct kbase_device *kbdev);

/**
 * kbase_instr_hwcnt_on_before_reset() - JM HWC instr backend function to be
 *                                       called immediately before a reset.
 *                                       Takes us out of the unrecoverable
 *                                       error state, if we were in it.
 * @kbdev: Kbase device
 */
void kbase_instr_hwcnt_on_before_reset(struct kbase_device *kbdev);

#endif /* _KBASE_HWACCESS_INSTR_H_ */
