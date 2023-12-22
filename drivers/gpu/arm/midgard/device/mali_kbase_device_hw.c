// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2014-2023 ARM Limited. All rights reserved.
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
#include <gpu/mali_kbase_gpu_fault.h>
#include <backend/gpu/mali_kbase_instr_internal.h>
#include <backend/gpu/mali_kbase_pm_internal.h>
#include <device/mali_kbase_device.h>
#include <mali_kbase_reset_gpu.h>
#include <mmu/mali_kbase_mmu.h>

bool kbase_is_gpu_removed(struct kbase_device *kbdev)
{
	if (!IS_ENABLED(CONFIG_MALI_ARBITER_SUPPORT))
		return false;

	return (kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_ID)) == 0);
}

/**
 * busy_wait_cache_operation - Wait for a pending cache flush to complete
 *
 * @kbdev:   Pointer of kbase device.
 * @irq_bit: IRQ bit cache flush operation to wait on.
 *
 * It will reset GPU if the wait fails.
 *
 * Return: 0 on success, error code otherwise.
 */
static int busy_wait_cache_operation(struct kbase_device *kbdev, u32 irq_bit)
{
	const ktime_t wait_loop_start = ktime_get_raw();
	const u32 wait_time_ms = kbase_get_timeout_ms(kbdev, MMU_AS_INACTIVE_WAIT_TIMEOUT);
	bool completed = false;
	s64 diff;
	u32 irq_bits_to_check = irq_bit;

	/* hwaccess_lock must be held to prevent concurrent threads from
	 * cleaning the IRQ bits, otherwise it could be possible for this thread
	 * to lose the event it is waiting for. In particular, concurrent attempts
	 * to reset the GPU could go undetected and this thread would miss
	 * the completion of the cache flush operation it is waiting for.
	 */
	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* Add the RESET_COMPLETED bit. If this bit is set, then the GPU has
	 * been reset which implies that any cache flush operation has been
	 * completed, too.
	 */
	{
		irq_bits_to_check |= RESET_COMPLETED;
	}

	do {
		unsigned int i;

		for (i = 0; i < 1000; i++) {
			if (kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_RAWSTAT)) &
			    irq_bits_to_check) {
				completed = true;
				break;
			}
		}

		diff = ktime_to_ms(ktime_sub(ktime_get_raw(), wait_loop_start));
	} while ((diff < wait_time_ms) && !completed);

	if (!completed) {
		char *irq_flag_name;

		switch (irq_bit) {
		case CLEAN_CACHES_COMPLETED:
			irq_flag_name = "CLEAN_CACHES_COMPLETED";
			break;
		case FLUSH_PA_RANGE_COMPLETED:
			irq_flag_name = "FLUSH_PA_RANGE_COMPLETED";
			break;
		default:
			irq_flag_name = "UNKNOWN";
			break;
		}

		dev_err(kbdev->dev,
			"Stuck waiting on %s bit, might be due to unstable GPU clk/pwr or possible faulty FPGA connector\n",
			irq_flag_name);

		if (kbase_prepare_to_reset_gpu_locked(kbdev, RESET_FLAGS_NONE))
			kbase_reset_gpu_locked(kbdev);

		return -EBUSY;
	}

	KBASE_KTRACE_ADD(kbdev, CORE_GPU_IRQ_CLEAR, NULL, irq_bit);
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_CLEAR), irq_bit);

	return 0;
}

#if MALI_USE_CSF

int kbase_gpu_cache_flush_pa_range_and_busy_wait(struct kbase_device *kbdev, phys_addr_t phys,
						 size_t nr_bytes, u32 flush_op)
{
	u64 start_pa, end_pa;
	int ret = 0;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* 1. Clear the interrupt FLUSH_PA_RANGE_COMPLETED bit. */
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_CLEAR), FLUSH_PA_RANGE_COMPLETED);

	/* 2. Issue GPU_CONTROL.COMMAND.FLUSH_PA_RANGE operation. */
	start_pa = phys;
	end_pa = start_pa + nr_bytes - 1;

	kbase_reg_write64(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND_ARG0), start_pa);
	kbase_reg_write64(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND_ARG1), end_pa);
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND), flush_op);

	/* 3. Busy-wait irq status to be enabled. */
	ret = busy_wait_cache_operation(kbdev, (u32)FLUSH_PA_RANGE_COMPLETED);

	return ret;
}
#endif /* MALI_USE_CSF */

int kbase_gpu_cache_flush_and_busy_wait(struct kbase_device *kbdev, u32 flush_op)
{
	int need_to_wake_up = 0;
	int ret = 0;

	/* hwaccess_lock must be held to avoid any sync issue with
	 * kbase_gpu_start_cache_clean() / kbase_clean_caches_done()
	 */
	lockdep_assert_held(&kbdev->hwaccess_lock);

	/* 1. Check if kbdev->cache_clean_in_progress is set.
	 *    If it is set, it means there are threads waiting for
	 *    CLEAN_CACHES_COMPLETED irq to be raised and that the
	 *    corresponding irq mask bit is set.
	 *    We'll clear the irq mask bit and busy-wait for the cache
	 *    clean operation to complete before submitting the cache
	 *    clean command required after the GPU page table update.
	 *    Pended flush commands will be merged to requested command.
	 */
	if (kbdev->cache_clean_in_progress) {
		/* disable irq first */
		u32 irq_mask = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_MASK));

		kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_MASK),
				  irq_mask & ~CLEAN_CACHES_COMPLETED);

		/* busy wait irq status to be enabled */
		ret = busy_wait_cache_operation(kbdev, CLEAN_CACHES_COMPLETED);
		if (ret)
			return ret;

		/* merge pended command if there's any */
		flush_op = GPU_COMMAND_FLUSH_CACHE_MERGE(kbdev->cache_clean_queued, flush_op);

		/* enable wake up notify flag */
		need_to_wake_up = 1;
	} else {
		/* Clear the interrupt CLEAN_CACHES_COMPLETED bit. */
		kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_CLEAR), CLEAN_CACHES_COMPLETED);
	}

	/* 2. Issue GPU_CONTROL.COMMAND.FLUSH_CACHE operation. */
	KBASE_KTRACE_ADD(kbdev, CORE_GPU_CLEAN_INV_CACHES, NULL, flush_op);
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND), flush_op);

	/* 3. Busy-wait irq status to be enabled. */
	ret = busy_wait_cache_operation(kbdev, CLEAN_CACHES_COMPLETED);
	if (ret)
		return ret;

	/* 4. Wake-up blocked threads when there is any. */
	if (need_to_wake_up)
		kbase_gpu_cache_clean_wait_complete(kbdev);

	return ret;
}

void kbase_gpu_start_cache_clean_nolock(struct kbase_device *kbdev, u32 flush_op)
{
	u32 irq_mask;

	lockdep_assert_held(&kbdev->hwaccess_lock);

	if (kbdev->cache_clean_in_progress) {
		/* If this is called while another clean is in progress, we
		 * can't rely on the current one to flush any new changes in
		 * the cache. Instead, accumulate all cache clean operations
		 * and trigger that immediately after this one finishes.
		 */
		kbdev->cache_clean_queued =
			GPU_COMMAND_FLUSH_CACHE_MERGE(kbdev->cache_clean_queued, flush_op);
		return;
	}

	/* Enable interrupt */
	irq_mask = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_MASK));
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_MASK), irq_mask | CLEAN_CACHES_COMPLETED);

	KBASE_KTRACE_ADD(kbdev, CORE_GPU_CLEAN_INV_CACHES, NULL, flush_op);
	kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND), flush_op);

	kbdev->cache_clean_in_progress = true;
}

void kbase_gpu_start_cache_clean(struct kbase_device *kbdev, u32 flush_op)
{
	unsigned long flags;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	kbase_gpu_start_cache_clean_nolock(kbdev, flush_op);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
}

void kbase_gpu_cache_clean_wait_complete(struct kbase_device *kbdev)
{
	lockdep_assert_held(&kbdev->hwaccess_lock);

	kbdev->cache_clean_queued = 0;
	kbdev->cache_clean_in_progress = false;
	wake_up(&kbdev->cache_clean_wait);
}

void kbase_clean_caches_done(struct kbase_device *kbdev)
{
	u32 irq_mask;
	unsigned long flags;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (kbdev->cache_clean_in_progress) {
		/* Clear the interrupt CLEAN_CACHES_COMPLETED bit if set.
		 * It might have already been done by kbase_gpu_cache_flush_and_busy_wait.
		 */
		KBASE_KTRACE_ADD(kbdev, CORE_GPU_IRQ_CLEAR, NULL, CLEAN_CACHES_COMPLETED);
		kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_CLEAR), CLEAN_CACHES_COMPLETED);

		if (kbdev->cache_clean_queued) {
			u32 pended_flush_op = kbdev->cache_clean_queued;

			kbdev->cache_clean_queued = 0;

			KBASE_KTRACE_ADD(kbdev, CORE_GPU_CLEAN_INV_CACHES, NULL, pended_flush_op);
			kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_COMMAND), pended_flush_op);
		} else {
			/* Disable interrupt */
			irq_mask = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_MASK));
			kbase_reg_write32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_MASK),
					  irq_mask & ~CLEAN_CACHES_COMPLETED);

			kbase_gpu_cache_clean_wait_complete(kbdev);
		}
	}

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
}

static inline bool get_cache_clean_flag(struct kbase_device *kbdev)
{
	bool cache_clean_in_progress;
	unsigned long flags;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	cache_clean_in_progress = kbdev->cache_clean_in_progress;
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	return cache_clean_in_progress;
}

void kbase_gpu_wait_cache_clean(struct kbase_device *kbdev)
{
	while (get_cache_clean_flag(kbdev)) {
		if (wait_event_interruptible(kbdev->cache_clean_wait,
					     !kbdev->cache_clean_in_progress))
			dev_warn(kbdev->dev, "Wait for cache clean is interrupted");
	}
}

int kbase_gpu_wait_cache_clean_timeout(struct kbase_device *kbdev, unsigned int wait_timeout_ms)
{
	long remaining = (long)msecs_to_jiffies(wait_timeout_ms);
	int result = 0;

	while (remaining && get_cache_clean_flag(kbdev)) {
		remaining = wait_event_timeout(kbdev->cache_clean_wait,
					       !kbdev->cache_clean_in_progress, remaining);
	}

	if (!remaining) {
		dev_err(kbdev->dev,
			"Cache clean timed out. Might be caused by unstable GPU clk/pwr or faulty system");

		if (kbase_prepare_to_reset_gpu_locked(kbdev, RESET_FLAGS_HWC_UNRECOVERABLE_ERROR))
			kbase_reset_gpu_locked(kbdev);

		result = -ETIMEDOUT;
	}

	return result;
}
