/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
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

/*
 * Backend specific IRQ APIs
 */

#ifndef _KBASE_IRQ_INTERNAL_H_
#define _KBASE_IRQ_INTERNAL_H_

/* GPU IRQ Tags */
#define JOB_IRQ_TAG 0
#define MMU_IRQ_TAG 1
#define GPU_IRQ_TAG 2

/**
 * kbase_install_interrupts - Install IRQs handlers.
 *
 * @kbdev: The kbase device
 *
 * This function must be called once only when a kbase device is initialized.
 *
 * Return: 0 on success. Error code (negative) on failure.
 */
int kbase_install_interrupts(struct kbase_device *kbdev);

/**
 * kbase_release_interrupts - Uninstall IRQs handlers.
 *
 * @kbdev: The kbase device
 *
 * This function needs to be called when a kbase device is terminated.
 */
void kbase_release_interrupts(struct kbase_device *kbdev);

/**
 * kbase_synchronize_irqs - Ensure that all IRQ handlers have completed
 *                          execution
 * @kbdev: The kbase device
 */
void kbase_synchronize_irqs(struct kbase_device *kbdev);

#ifdef CONFIG_MALI_DEBUG
#if IS_ENABLED(CONFIG_MALI_REAL_HW)
/**
 * kbase_validate_interrupts - Validate interrupts
 *
 * @kbdev: The kbase device
 *
 * This function will be called once when a kbase device is initialized
 * to check whether interrupt handlers are configured appropriately.
 * If interrupt numbers and/or flags defined in the device tree are
 * incorrect, then the validation might fail.
 * The whold device initialization will fail if it returns error code.
 *
 * Return: 0 on success. Error code (negative) on failure.
 */
int kbase_validate_interrupts(struct kbase_device *const kbdev);
#endif /* IS_ENABLED(CONFIG_MALI_REAL_HW) */
#endif /* CONFIG_MALI_DEBUG */

/**
 * kbase_get_interrupt_handler - Return default interrupt handler
 * @kbdev:   Kbase device
 * @irq_tag: Tag to choose the handler
 *
 * If single interrupt line is used the combined interrupt handler
 * will be returned regardless of irq_tag. Otherwise the corresponding
 * interrupt handler will be returned.
 *
 * Return: Interrupt handler corresponding to the tag. NULL on failure.
 */
irq_handler_t kbase_get_interrupt_handler(struct kbase_device *kbdev, u32 irq_tag);

/**
 * kbase_set_custom_irq_handler - Set a custom IRQ handler
 *
 * @kbdev: The kbase device for which the handler is to be registered
 * @custom_handler: Handler to be registered
 * @irq_tag: Interrupt tag
 *
 * Register given interrupt handler for requested interrupt tag
 * In the case where irq handler is not specified, the default handler shall be
 * registered
 *
 * Return: 0 case success, error code otherwise
 */
int kbase_set_custom_irq_handler(struct kbase_device *kbdev, irq_handler_t custom_handler,
				 u32 irq_tag);

#endif /* _KBASE_IRQ_INTERNAL_H_ */
