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
#include <device/mali_kbase_device.h>
#include <backend/gpu/mali_kbase_irq_internal.h>

#include <linux/interrupt.h>

#if IS_ENABLED(CONFIG_MALI_REAL_HW)
static void *kbase_tag(void *ptr, u32 tag)
{
	return (void *)(((uintptr_t)ptr) | tag);
}
#endif

static void *kbase_untag(void *ptr)
{
	return (void *)(((uintptr_t)ptr) & ~(uintptr_t)3);
}

static irqreturn_t kbase_job_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = kbase_untag(data);
	u32 val;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return IRQ_NONE;
	}

	val = kbase_reg_read32(kbdev, JOB_CONTROL_ENUM(JOB_IRQ_STATUS));

	if (!val) {
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return IRQ_NONE;
	}

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

#if MALI_USE_CSF
	/* call the csf interrupt handler */
	kbase_csf_interrupt(kbdev, val);
#else
	kbase_job_done(kbdev, val);
#endif

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t kbase_mmu_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = kbase_untag(data);
	u32 val;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return IRQ_NONE;
	}

	atomic_inc(&kbdev->faults_pending);

	val = kbase_reg_read32(kbdev, MMU_CONTROL_ENUM(IRQ_STATUS));

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	if (!val) {
		atomic_dec(&kbdev->faults_pending);
		return IRQ_NONE;
	}

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbase_mmu_interrupt(kbdev, val);

	atomic_dec(&kbdev->faults_pending);

	return IRQ_HANDLED;
}


static irqreturn_t kbase_gpuonly_irq_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = kbase_untag(data);
	u32 gpu_irq_status;
	irqreturn_t irq_state = IRQ_NONE;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return IRQ_NONE;
	}

	gpu_irq_status = kbase_reg_read32(kbdev, GPU_CONTROL_ENUM(GPU_IRQ_STATUS));

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	if (gpu_irq_status) {
		dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, gpu_irq_status);
		kbase_gpu_interrupt(kbdev, gpu_irq_status);

		irq_state = IRQ_HANDLED;
	}

	return irq_state;
}

/**
 * kbase_gpu_irq_handler - GPU interrupt handler
 * @irq:  IRQ number
 * @data: Data associated with this IRQ (i.e. kbdev)
 *
 * Return: IRQ_HANDLED if any interrupt request has been successfully handled.
 *         IRQ_NONE otherwise.
 */
static irqreturn_t kbase_gpu_irq_handler(int irq, void *data)
{
	irqreturn_t irq_state = kbase_gpuonly_irq_handler(irq, data);
	return irq_state;
}

/**
 * kbase_combined_irq_handler - Combined interrupt handler for all interrupts
 * @irq:  IRQ number
 * @data: Data associated with this IRQ (i.e. kbdev)
 *
 * This handler will be used for the GPU with single interrupt line.
 *
 * Return: IRQ_HANDLED if any interrupt request has been successfully handled.
 *         IRQ_NONE otherwise.
 */
static irqreturn_t kbase_combined_irq_handler(int irq, void *data)
{
	irqreturn_t irq_state = IRQ_NONE;

	if (kbase_job_irq_handler(irq, data) == IRQ_HANDLED)
		irq_state = IRQ_HANDLED;
	if (kbase_mmu_irq_handler(irq, data) == IRQ_HANDLED)
		irq_state = IRQ_HANDLED;
	if (kbase_gpu_irq_handler(irq, data) == IRQ_HANDLED)
		irq_state = IRQ_HANDLED;

	return irq_state;
}

static irq_handler_t kbase_handler_table[] = {
	[JOB_IRQ_TAG] = kbase_job_irq_handler,
	[MMU_IRQ_TAG] = kbase_mmu_irq_handler,
	[GPU_IRQ_TAG] = kbase_gpu_irq_handler,
};

irq_handler_t kbase_get_interrupt_handler(struct kbase_device *kbdev, u32 irq_tag)
{
	if (kbdev->nr_irqs == 1)
		return kbase_combined_irq_handler;
	else if (irq_tag < ARRAY_SIZE(kbase_handler_table))
		return kbase_handler_table[irq_tag];
	else
		return NULL;
}

#if IS_ENABLED(CONFIG_MALI_REAL_HW)
#ifdef CONFIG_MALI_DEBUG
int kbase_set_custom_irq_handler(struct kbase_device *kbdev, irq_handler_t custom_handler,
				 u32 irq_tag)
{
	int result = 0;
	irq_handler_t handler = custom_handler;
	const int irq = (kbdev->nr_irqs == 1) ? 0 : irq_tag;

	if (unlikely(!((irq_tag >= JOB_IRQ_TAG) && (irq_tag <= GPU_IRQ_TAG)))) {
		dev_err(kbdev->dev, "Invalid irq_tag (%d)\n", irq_tag);
		return -EINVAL;
	}

	/* Release previous handler */
	if (kbdev->irqs[irq].irq)
		free_irq(kbdev->irqs[irq].irq, kbase_tag(kbdev, irq));

	/* If a custom handler isn't provided use the default handler */
	if (!handler)
		handler = kbase_get_interrupt_handler(kbdev, irq_tag);

	if (request_irq(kbdev->irqs[irq].irq, handler,
			kbdev->irqs[irq].flags | ((kbdev->nr_irqs == 1) ? 0 : IRQF_SHARED),
			dev_name(kbdev->dev), kbase_tag(kbdev, irq)) != 0) {
		result = -EINVAL;
		dev_err(kbdev->dev, "Can't request interrupt %u (index %u)\n", kbdev->irqs[irq].irq,
			irq_tag);
		if (IS_ENABLED(CONFIG_SPARSE_IRQ))
			dev_err(kbdev->dev,
				"CONFIG_SPARSE_IRQ enabled - is the interrupt number correct for this config?\n");
	}

	return result;
}

KBASE_EXPORT_TEST_API(kbase_set_custom_irq_handler);

/* test correct interrupt assigment and reception by cpu */
struct kbasep_irq_test {
	struct hrtimer timer;
	wait_queue_head_t wait;
	int triggered;
	u32 timeout;
};

static struct kbasep_irq_test kbasep_irq_test_data;

#define IRQ_TEST_TIMEOUT 500

static irqreturn_t kbase_job_irq_test_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = kbase_untag(data);
	u32 val;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return IRQ_NONE;
	}

	val = kbase_reg_read32(kbdev, JOB_CONTROL_ENUM(JOB_IRQ_STATUS));

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbasep_irq_test_data.triggered = 1;
	wake_up(&kbasep_irq_test_data.wait);

	kbase_reg_write32(kbdev, JOB_CONTROL_ENUM(JOB_IRQ_CLEAR), val);

	return IRQ_HANDLED;
}

static irqreturn_t kbase_mmu_irq_test_handler(int irq, void *data)
{
	unsigned long flags;
	struct kbase_device *kbdev = kbase_untag(data);
	u32 val;

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);

	if (!kbdev->pm.backend.gpu_powered) {
		/* GPU is turned off - IRQ is not for us */
		spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);
		return IRQ_NONE;
	}

	val = kbase_reg_read32(kbdev, MMU_CONTROL_ENUM(IRQ_STATUS));

	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	if (!val)
		return IRQ_NONE;

	dev_dbg(kbdev->dev, "%s: irq %d irqstatus 0x%x\n", __func__, irq, val);

	kbasep_irq_test_data.triggered = 1;
	wake_up(&kbasep_irq_test_data.wait);

	kbase_reg_write32(kbdev, MMU_CONTROL_ENUM(IRQ_CLEAR), val);

	return IRQ_HANDLED;
}

static enum hrtimer_restart kbasep_test_interrupt_timeout(struct hrtimer *timer)
{
	struct kbasep_irq_test *test_data = container_of(timer, struct kbasep_irq_test, timer);

	test_data->timeout = 1;
	test_data->triggered = 1;
	wake_up(&test_data->wait);
	return HRTIMER_NORESTART;
}

/**
 * validate_interrupt - Validate an interrupt
 * @kbdev: Kbase device
 * @tag:   Tag to choose the interrupt
 *
 * To validate the settings for the interrupt, write a value on RAWSTAT
 * register to trigger interrupt. Then with custom interrupt handler
 * check whether the interrupt happens within reasonable time.
 *
 * Return: 0 if validating interrupt succeeds.
 */
static int validate_interrupt(struct kbase_device *const kbdev, u32 tag)
{
	int err = 0;
	irq_handler_t handler;
	const int irq = (kbdev->nr_irqs == 1) ? 0 : tag;
	u32 old_mask_val;
	u16 mask_offset;
	u16 rawstat_offset;

	switch (tag) {
	case JOB_IRQ_TAG:
		handler = kbase_job_irq_test_handler;
		rawstat_offset = JOB_CONTROL_ENUM(JOB_IRQ_RAWSTAT);
		mask_offset = JOB_CONTROL_ENUM(JOB_IRQ_MASK);
		break;
	case MMU_IRQ_TAG:
		handler = kbase_mmu_irq_test_handler;
		rawstat_offset = MMU_CONTROL_ENUM(IRQ_RAWSTAT);
		mask_offset = MMU_CONTROL_ENUM(IRQ_MASK);
		break;
	case GPU_IRQ_TAG:
		/* already tested by pm_driver - bail out */
		return 0;
	default:
		dev_err(kbdev->dev, "Invalid tag (%d)\n", tag);
		return -EINVAL;
	}

	/* store old mask */
	old_mask_val = kbase_reg_read32(kbdev, mask_offset);
	/* mask interrupts */
	kbase_reg_write32(kbdev, mask_offset, 0x0);

	if (kbdev->irqs[irq].irq) {
		/* release original handler and install test handler */
		if (kbase_set_custom_irq_handler(kbdev, handler, tag) != 0) {
			err = -EINVAL;
		} else {
			kbasep_irq_test_data.timeout = 0;
			hrtimer_init(&kbasep_irq_test_data.timer, CLOCK_MONOTONIC,
				     HRTIMER_MODE_REL);
			kbasep_irq_test_data.timer.function = kbasep_test_interrupt_timeout;

			/* trigger interrupt */
			kbase_reg_write32(kbdev, mask_offset, 0x1);
			kbase_reg_write32(kbdev, rawstat_offset, 0x1);

			hrtimer_start(&kbasep_irq_test_data.timer,
				      HR_TIMER_DELAY_MSEC(IRQ_TEST_TIMEOUT), HRTIMER_MODE_REL);

			wait_event(kbasep_irq_test_data.wait, kbasep_irq_test_data.triggered != 0);

			if (kbasep_irq_test_data.timeout != 0) {
				dev_err(kbdev->dev, "Interrupt %u (index %u) didn't reach CPU.\n",
					kbdev->irqs[irq].irq, irq);
				err = -EINVAL;
			} else {
				dev_dbg(kbdev->dev, "Interrupt %u (index %u) reached CPU.\n",
					kbdev->irqs[irq].irq, irq);
			}

			hrtimer_cancel(&kbasep_irq_test_data.timer);
			kbasep_irq_test_data.triggered = 0;

			/* mask interrupts */
			kbase_reg_write32(kbdev, mask_offset, 0x0);

			/* release test handler */
			free_irq(kbdev->irqs[irq].irq, kbase_tag(kbdev, irq));
		}

		/* restore original interrupt */
		if (request_irq(kbdev->irqs[irq].irq, kbase_get_interrupt_handler(kbdev, tag),
				kbdev->irqs[irq].flags | ((kbdev->nr_irqs == 1) ? 0 : IRQF_SHARED),
				dev_name(kbdev->dev), kbase_tag(kbdev, irq))) {
			dev_err(kbdev->dev, "Can't restore original interrupt %u (index %u)\n",
				kbdev->irqs[irq].irq, tag);
			err = -EINVAL;
		}
	}
	/* restore old mask */
	kbase_reg_write32(kbdev, mask_offset, old_mask_val);

	return err;
}

#if IS_ENABLED(CONFIG_MALI_REAL_HW)
int kbase_validate_interrupts(struct kbase_device *const kbdev)
{
	int err;

	init_waitqueue_head(&kbasep_irq_test_data.wait);
	kbasep_irq_test_data.triggered = 0;

	/* A suspend won't happen during startup/insmod */
	kbase_pm_context_active(kbdev);

	err = validate_interrupt(kbdev, JOB_IRQ_TAG);
	if (err) {
		dev_err(kbdev->dev,
			"Interrupt JOB_IRQ didn't reach CPU. Check interrupt assignments.\n");
		goto out;
	}

	err = validate_interrupt(kbdev, MMU_IRQ_TAG);
	if (err) {
		dev_err(kbdev->dev,
			"Interrupt MMU_IRQ didn't reach CPU. Check interrupt assignments.\n");
		goto out;
	}

	dev_dbg(kbdev->dev, "Interrupts are correctly assigned.\n");

out:
	kbase_pm_context_idle(kbdev);

	return err;
}
#endif /* CONFIG_MALI_REAL_HW */
#endif /* CONFIG_MALI_DEBUG */

int kbase_install_interrupts(struct kbase_device *kbdev)
{
	u32 i;

	for (i = 0; i < kbdev->nr_irqs; i++) {
		const int result = request_irq(
			kbdev->irqs[i].irq, kbase_get_interrupt_handler(kbdev, i),
			kbdev->irqs[i].flags | ((kbdev->nr_irqs == 1) ? 0 : IRQF_SHARED),
			dev_name(kbdev->dev), kbase_tag(kbdev, i));
		if (result) {
			dev_err(kbdev->dev, "Can't request interrupt %u (index %u)\n",
				kbdev->irqs[i].irq, i);
			goto release;
		}
	}

	return 0;

release:
	if (IS_ENABLED(CONFIG_SPARSE_IRQ))
		dev_err(kbdev->dev,
			"CONFIG_SPARSE_IRQ enabled - is the interrupt number correct for this config?\n");

	while (i-- > 0)
		free_irq(kbdev->irqs[i].irq, kbase_tag(kbdev, i));

	return -EINVAL;
}

void kbase_release_interrupts(struct kbase_device *kbdev)
{
	u32 i;

	for (i = 0; i < kbdev->nr_irqs; i++) {
		if (kbdev->irqs[i].irq)
			free_irq(kbdev->irqs[i].irq, kbase_tag(kbdev, i));
	}
}

void kbase_synchronize_irqs(struct kbase_device *kbdev)
{
	u32 i;

	for (i = 0; i < kbdev->nr_irqs; i++) {
		if (kbdev->irqs[i].irq)
			synchronize_irq(kbdev->irqs[i].irq);
	}
}

KBASE_EXPORT_TEST_API(kbase_synchronize_irqs);

#endif /* IS_ENABLED(CONFIG_MALI_REAL_HW) */
