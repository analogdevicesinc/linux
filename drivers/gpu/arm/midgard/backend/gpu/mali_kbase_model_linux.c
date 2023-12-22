// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2010-2023 ARM Limited. All rights reserved.
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
 * Model Linux Framework interfaces.
 */

#include <mali_kbase.h>
#include <hw_access/mali_kbase_hw_access_regmap.h>

#include "backend/gpu/mali_kbase_model_linux.h"
#include "device/mali_kbase_device.h"
#include "mali_kbase_irq_internal.h"

#include <linux/kthread.h>

struct model_irq_data {
	struct kbase_device *kbdev;
	struct work_struct work;
};

#define DEFINE_SERVE_IRQ(irq_handler)                                                          \
	static void serve_##irq_handler(struct work_struct *work)                              \
	{                                                                                      \
		struct model_irq_data *data = container_of(work, struct model_irq_data, work); \
		struct kbase_device *kbdev = data->kbdev;                                      \
		irq_handler(kbdev);                                                            \
		kmem_cache_free(kbdev->irq_slab, data);                                        \
	}

static void job_irq(struct kbase_device *kbdev)
{
	/* Make sure no worker is already serving this IRQ */
	while (atomic_cmpxchg(&kbdev->serving_job_irq, 1, 0) == 1)
		kbase_get_interrupt_handler(kbdev, JOB_IRQ_TAG)(0, kbdev);
}
DEFINE_SERVE_IRQ(job_irq)

static void gpu_irq(struct kbase_device *kbdev)
{
	/* Make sure no worker is already serving this IRQ */
	while (atomic_cmpxchg(&kbdev->serving_gpu_irq, 1, 0) == 1)
		kbase_get_interrupt_handler(kbdev, GPU_IRQ_TAG)(0, kbdev);
}
DEFINE_SERVE_IRQ(gpu_irq)

static void mmu_irq(struct kbase_device *kbdev)
{
	/* Make sure no worker is already serving this IRQ */
	while (atomic_cmpxchg(&kbdev->serving_mmu_irq, 1, 0) == 1)
		kbase_get_interrupt_handler(kbdev, MMU_IRQ_TAG)(0, kbdev);
}
DEFINE_SERVE_IRQ(mmu_irq)


void gpu_device_raise_irq(void *model, u32 irq)
{
	struct model_irq_data *data;
	struct kbase_device *kbdev = gpu_device_get_data(model);

	KBASE_DEBUG_ASSERT(kbdev);

	data = kmem_cache_alloc(kbdev->irq_slab, GFP_ATOMIC);
	if (data == NULL)
		return;

	data->kbdev = kbdev;

	switch (irq) {
	case MODEL_LINUX_JOB_IRQ:
		INIT_WORK(&data->work, serve_job_irq);
		atomic_set(&kbdev->serving_job_irq, 1);
		break;
	case MODEL_LINUX_GPU_IRQ:
		INIT_WORK(&data->work, serve_gpu_irq);
		atomic_set(&kbdev->serving_gpu_irq, 1);
		break;
	case MODEL_LINUX_MMU_IRQ:
		INIT_WORK(&data->work, serve_mmu_irq);
		atomic_set(&kbdev->serving_mmu_irq, 1);
		break;
	default:
		dev_warn(kbdev->dev, "Unknown IRQ");
		kmem_cache_free(kbdev->irq_slab, data);
		data = NULL;
		break;
	}

	if (data != NULL)
		queue_work(kbdev->irq_workq, &data->work);
}

int kbase_install_interrupts(struct kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev);

	atomic_set(&kbdev->serving_job_irq, 0);
	atomic_set(&kbdev->serving_gpu_irq, 0);
	atomic_set(&kbdev->serving_mmu_irq, 0);

	kbdev->irq_workq = alloc_ordered_workqueue("dummy irq queue", 0);
	if (kbdev->irq_workq == NULL)
		return -ENOMEM;

	kbdev->irq_slab =
		kmem_cache_create("dummy_irq_slab", sizeof(struct model_irq_data), 0, 0, NULL);
	if (kbdev->irq_slab == NULL) {
		destroy_workqueue(kbdev->irq_workq);
		return -ENOMEM;
	}

	kbdev->nr_irqs = 3;


	return 0;
}

void kbase_release_interrupts(struct kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev);
	destroy_workqueue(kbdev->irq_workq);
	kmem_cache_destroy(kbdev->irq_slab);
}

void kbase_synchronize_irqs(struct kbase_device *kbdev)
{
	KBASE_DEBUG_ASSERT(kbdev);
	flush_workqueue(kbdev->irq_workq);
}

KBASE_EXPORT_TEST_API(kbase_synchronize_irqs);

int kbase_set_custom_irq_handler(struct kbase_device *kbdev, irq_handler_t custom_handler,
				 u32 irq_tag)
{
	return 0;
}

KBASE_EXPORT_TEST_API(kbase_set_custom_irq_handler);

int kbase_gpu_device_create(struct kbase_device *kbdev)
{
	kbdev->model = midgard_model_create(kbdev);
	if (kbdev->model == NULL)
		return -ENOMEM;

	spin_lock_init(&kbdev->reg_op_lock);

	return 0;
}

/**
 * kbase_gpu_device_destroy - Destroy GPU device
 *
 * @kbdev: kbase device
 */
void kbase_gpu_device_destroy(struct kbase_device *kbdev)
{
	midgard_model_destroy(kbdev->model);
}
