// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2014-2016, 2018-2021 ARM Limited. All rights reserved.
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
#include <mali_kbase_hwaccess_time.h>
#include <device/mali_kbase_device.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <mali_kbase_config_defaults.h>

void kbase_backend_get_gpu_time_norequest(struct kbase_device *kbdev,
					  u64 *cycle_counter,
					  u64 *system_time,
					  struct timespec64 *ts)
{
	u32 hi1, hi2;

	if (cycle_counter)
		*cycle_counter = kbase_backend_get_cycle_cnt(kbdev);

	if (system_time) {
		/* Read hi, lo, hi to ensure a coherent u64 */
		do {
			hi1 = kbase_reg_read(kbdev,
					     GPU_CONTROL_REG(TIMESTAMP_HI));
			*system_time = kbase_reg_read(kbdev,
					     GPU_CONTROL_REG(TIMESTAMP_LO));
			hi2 = kbase_reg_read(kbdev,
					     GPU_CONTROL_REG(TIMESTAMP_HI));
		} while (hi1 != hi2);
		*system_time |= (((u64) hi1) << 32);
	}

	/* Record the CPU's idea of current time */
	if (ts != NULL)
#if (KERNEL_VERSION(4, 17, 0) > LINUX_VERSION_CODE)
		*ts = ktime_to_timespec64(ktime_get_raw());
#else
		ktime_get_raw_ts64(ts);
#endif
}

#if !MALI_USE_CSF
/**
 * timedwait_cycle_count_active() - Timed wait till CYCLE_COUNT_ACTIVE is active
 *
 * @kbdev: Kbase device
 *
 * Return: true if CYCLE_COUNT_ACTIVE is active within the timeout.
 */
static bool timedwait_cycle_count_active(struct kbase_device *kbdev)
{
	bool success = false;
	const unsigned int timeout = 100;
	const unsigned long remaining = jiffies + msecs_to_jiffies(timeout);

	while (time_is_after_jiffies(remaining)) {
		if ((kbase_reg_read(kbdev, GPU_CONTROL_REG(GPU_STATUS)) &
		     GPU_STATUS_CYCLE_COUNT_ACTIVE)) {
			success = true;
			break;
		}
	}
	return success;
}
#endif

void kbase_backend_get_gpu_time(struct kbase_device *kbdev, u64 *cycle_counter,
				u64 *system_time, struct timespec64 *ts)
{
#if !MALI_USE_CSF
	kbase_pm_request_gpu_cycle_counter(kbdev);
	WARN_ONCE(kbdev->pm.backend.l2_state != KBASE_L2_ON,
		  "L2 not powered up");
	WARN_ONCE((!timedwait_cycle_count_active(kbdev)),
		  "Timed out on CYCLE_COUNT_ACTIVE");
#endif
	kbase_backend_get_gpu_time_norequest(kbdev, cycle_counter, system_time,
					     ts);
#if !MALI_USE_CSF
	kbase_pm_release_gpu_cycle_counter(kbdev);
#endif
}

unsigned int kbase_get_timeout_ms(struct kbase_device *kbdev,
				  enum kbase_timeout_selector selector)
{
	/* Timeout calculation:
	 * dividing number of cycles by freq in KHz automatically gives value
	 * in milliseconds. nr_cycles will have to be multiplied by 1e3 to
	 * get result in microseconds, and 1e6 to get result in nanoseconds.
	 */

	u64 timeout, nr_cycles = 0;
	u64 freq_khz = kbdev->lowest_gpu_freq_khz;

	WARN_ON(!freq_khz);

	switch (selector) {
	/* use Firmware timeout if invalid selection */
	default:
#if !MALI_USE_CSF
		WARN(1, "Invalid timeout selector used! Using default value");
		timeout = JM_DEFAULT_TIMEOUT_CYCLES;
		CSTD_UNUSED(nr_cycles);
#else
		WARN(1,
		     "Invalid timeout selector used! Using CSF Firmware timeout");
		fallthrough;
	case CSF_FIRMWARE_TIMEOUT:
		nr_cycles = CSF_FIRMWARE_TIMEOUT_CYCLES;
		timeout = div_u64(nr_cycles, freq_khz);
		/* cap CSF FW timeout to FIRMWARE_PING_INTERVAL_MS
		 * if calculated timeout exceeds it. This should be adapted to a
		 * direct timeout comparison once the FIRMWARE_PING_INTERVAL_MS
		 * option is added to this timeout function. A compile-time check
		 * such as BUILD_BUG_ON can also be done once the firmware ping
		 * interval in cycles becomes available as a macro.
		 */
		if (timeout > FIRMWARE_PING_INTERVAL_MS) {
			dev_dbg(kbdev->dev, "Capped CSF_FIRMWARE_TIMEOUT %llu to %d",
				timeout, FIRMWARE_PING_INTERVAL_MS);
			timeout = FIRMWARE_PING_INTERVAL_MS;
		}
#endif
		break;
	}
	return (unsigned int)timeout;
}

u64 kbase_backend_get_cycle_cnt(struct kbase_device *kbdev)
{
	u32 hi1, hi2, lo;

	/* Read hi, lo, hi to ensure a coherent u64 */
	do {
		hi1 = kbase_reg_read(kbdev,
					GPU_CONTROL_REG(CYCLE_COUNT_HI));
		lo = kbase_reg_read(kbdev,
					GPU_CONTROL_REG(CYCLE_COUNT_LO));
		hi2 = kbase_reg_read(kbdev,
					GPU_CONTROL_REG(CYCLE_COUNT_HI));
	} while (hi1 != hi2);

	return lo | (((u64) hi1) << 32);
}
