/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2014, 2018-2021, 2024 ARM Limited. All rights reserved.
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

#ifndef _KBASE_BACKEND_TIME_H_
#define _KBASE_BACKEND_TIME_H_

/**
 * struct kbase_backend_time - System timestamp attributes.
 *
 * @multiplier:		Numerator of the converter's fraction.
 * @divisor:		Denominator of the converter's fraction.
 * @gpu_timestamp_offset: Cached CPU to GPU TS offset computed whenever whole system
 *                        enters into standby mode where CPU Monotonic time is suspend.
 * @device_scaled_timeouts: Timeouts in milliseconds that were scaled to be
 *                          consistent with the minimum MCU frequency. This
 *                          array caches the results of all of the conversions
 *                          for ease of use later on.
 *
 * According to Generic timer spec, system timer:
 * - Increments at a fixed frequency
 * - Starts operating from zero
 *
 * Hence CPU time is a linear function of System Time.
 *
 * CPU_ts = alpha * SYS_ts + beta
 *
 * Where
 * - alpha = 10^9/SYS_ts_freq
 * - beta is calculated by two timer samples taken at the same time:
 *   beta = CPU_ts_s - SYS_ts_s * alpha
 *
 * Since alpha is a rational number, we minimizing possible
 * rounding error by simplifying the ratio. Thus alpha is stored
 * as a simple `multiplier / divisor` ratio.
 *
 */
struct kbase_backend_time {
#if MALI_USE_CSF
	u64 multiplier;
	u64 divisor;
	s64 gpu_timestamp_offset;
#endif
	unsigned int device_scaled_timeouts[KBASE_TIMEOUT_SELECTOR_COUNT];
};

#if MALI_USE_CSF
/**
 * kbase_backend_time_convert_gpu_to_cpu() - Convert GPU timestamp to CPU timestamp.
 *
 * @kbdev:	Kbase device pointer
 * @gpu_ts:	System timestamp value to converter.
 *
 * Return: The CPU timestamp.
 */
u64 __maybe_unused kbase_backend_time_convert_gpu_to_cpu(struct kbase_device *kbdev, u64 gpu_ts);

/**
 * kbase_backend_update_gpu_timestamp_offset() - Updates GPU timestamp offset register with the
 *                                               cached value.
 *
 * @kbdev:	Kbase device pointer
 *
 * Compute the new cached value for GPU timestamp offset if the previously cached value has been
 * invalidated and update the GPU timestamp offset register with the cached value.
 */
void kbase_backend_update_gpu_timestamp_offset(struct kbase_device *kbdev);

/**
 * kbase_backend_invalidate_gpu_timestamp_offset() - Invalidate cached GPU timestamp offset value
 *
 * @kbdev:	Kbase device pointer
 *
 * This function invalidates cached GPU timestamp offset value whenever system suspend
 * is about to happen where CPU TS counter will be stopped.
 */
void kbase_backend_invalidate_gpu_timestamp_offset(struct kbase_device *kbdev);

#if MALI_UNIT_TEST
/**
 * kbase_backend_read_gpu_timestamp_offset_reg() - Read GPU TIMESTAMP OFFSET Register
 *
 * @kbdev:	Kbase device pointer
 *
 * This function read GPU TIMESTAMP OFFSET Register with proper register access
 *
 * Return: GPU TIMESTAMP OFFSET Register value, as unsigned 64 bit value
 */
u64 kbase_backend_read_gpu_timestamp_offset_reg(struct kbase_device *kbdev);
#endif
#endif

/**
 * kbase_backend_get_gpu_time() - Get current GPU time
 * @kbdev:              Device pointer
 * @cycle_counter:      Pointer to u64 to store cycle counter in.
 * @system_time:        Pointer to u64 to store system time in
 * @ts:                 Pointer to struct timespec to store current monotonic
 *			time in
 */
void kbase_backend_get_gpu_time(struct kbase_device *kbdev, u64 *cycle_counter, u64 *system_time,
				struct timespec64 *ts);

/**
 * kbase_backend_get_gpu_time_norequest() - Get current GPU time without
 *                                          request/release cycle counter
 * @kbdev:		Device pointer
 * @cycle_counter:	Pointer to u64 to store cycle counter in
 * @system_time:	Pointer to u64 to store system time in
 * @ts:			Pointer to struct timespec to store current monotonic
 *			time in
 */
void kbase_backend_get_gpu_time_norequest(struct kbase_device *kbdev, u64 *cycle_counter,
					  u64 *system_time, struct timespec64 *ts);

/**
 * kbase_device_set_timeout_ms - Set an unscaled device timeout in milliseconds,
 *                               subject to the maximum timeout constraint.
 *
 * @kbdev:            KBase device pointer.
 * @selector:         The specific timeout that should be scaled.
 * @timeout_ms:    The timeout in cycles which should be scaled.
 *
 * This function writes the absolute timeout in milliseconds to the table of
 * precomputed device timeouts, while estabilishing an upped bound on the individual
 * timeout of UINT_MAX milliseconds.
 */
void kbase_device_set_timeout_ms(struct kbase_device *kbdev, enum kbase_timeout_selector selector,
				 unsigned int timeout_ms);

/**
 * kbase_device_set_timeout - Calculate the given timeout using the provided
 *                            timeout cycles and multiplier.
 *
 * @kbdev:            KBase device pointer.
 * @selector:         The specific timeout that should be scaled.
 * @timeout_cycles:    The timeout in cycles which should be scaled.
 * @cycle_multiplier: A multiplier applied to the number of cycles, allowing
 *                    the callsite to scale the minimum timeout based on the
 *                    host device.
 *
 * This function writes the scaled timeout to the per-device table to avoid
 * having to recompute the timeouts every single time that the related methods
 * are called.
 */
void kbase_device_set_timeout(struct kbase_device *kbdev, enum kbase_timeout_selector selector,
			      u64 timeout_cycles, u32 cycle_multiplier);

/**
 * kbase_get_timeout_ms - Choose a timeout value to get a timeout scaled
 *                        GPU frequency, using a choice from
 *                        kbase_timeout_selector.
 *
 * @kbdev:	KBase device pointer.
 * @selector:	Value from kbase_scaled_timeout_selector enum.
 *
 * Return:	Timeout in milliseconds, as an unsigned integer.
 */
unsigned int kbase_get_timeout_ms(struct kbase_device *kbdev, enum kbase_timeout_selector selector);

/**
 * kbase_backend_get_cycle_cnt - Reads the GPU cycle counter
 *
 * @kbdev: Instance of a GPU platform device that implements a CSF interface.
 *
 * Return: Snapshot of the GPU cycle count register.
 */
u64 kbase_backend_get_cycle_cnt(struct kbase_device *kbdev);

/**
 * kbase_arch_timer_get_cntfrq - Get system timestamp counter frequency.
 *
 * @kbdev: Instance of a GPU platform device.
 *
 * Return: Frequency in Hz
 */
u64 kbase_arch_timer_get_cntfrq(struct kbase_device *kbdev);

/**
 * kbase_backend_time_init() - Initialize system timestamp converter.
 *
 * @kbdev:	Kbase device pointer
 *
 * This function should only be called after GPU is powered-up and
 * L2 cached power-up has been initiated.
 *
 * Return: Zero on success, error code otherwise.
 */
int kbase_backend_time_init(struct kbase_device *kbdev);

#endif /* _KBASE_BACKEND_TIME_H_ */
