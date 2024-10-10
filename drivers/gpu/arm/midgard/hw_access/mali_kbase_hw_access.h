/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023-2024 ARM Limited. All rights reserved.
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

#ifndef _MALI_KBASE_HW_ACCESS_H_
#define _MALI_KBASE_HW_ACCESS_H_

#include <linux/version_compat_defs.h>

#define KBASE_REGMAP_PERM_READ (1U << 0)
#define KBASE_REGMAP_PERM_WRITE (1U << 1)
#define KBASE_REGMAP_WIDTH_32_BIT (1U << 2)
#define KBASE_REGMAP_WIDTH_64_BIT (1U << 3)

#define KBASE_REG_READ(kbdev, reg_enum)                                             \
	(kbase_reg_is_size64(kbdev, reg_enum) ? kbase_reg_read64(kbdev, reg_enum) : \
						      kbase_reg_read32(kbdev, reg_enum))

#define KBASE_REG_WRITE(kbdev, reg_enum, value)                                             \
	(kbase_reg_is_size64(kbdev, reg_enum) ? kbase_reg_write64(kbdev, reg_enum, value) : \
						      kbase_reg_write32(kbdev, reg_enum, value))

/**
 * kbase_reg_read32 - read from 32-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_GPU_OFF is not set).
 *
 * Return: Value in desired register
 */
u32 kbase_reg_read32(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_read64 - read from 64-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_GPU_OFF is not set).
 *
 * Return: Value in desired register
 */
u64 kbase_reg_read64(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_read64_coherent - read from 64-bit GPU register while ensuring
 *                             that hi1 == hi2
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_GPU_OFF is not set).
 *
 * Return: Value in desired register
 */
u64 kbase_reg_read64_coherent(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_write32 - write to 32-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 * @value:    Value to write
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_GPU_OFF is not set).
 */
void kbase_reg_write32(struct kbase_device *kbdev, u32 reg_enum, u32 value);

/**
 * kbase_reg_write64 - write to 64-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 * @value:    Value to write
 *
 * Caller must ensure the GPU is powered (KBASE_IO_STATUS_GPU_OFF is not set).
 */
void kbase_reg_write64(struct kbase_device *kbdev, u32 reg_enum, u64 value);

/**
 * kbase_reg_is_size64 - check GPU register size is 64-bit
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Return: boolean if register is 64-bit
 */
bool kbase_reg_is_size64(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_is_size32 - check GPU register size is 32-bit
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Return: boolean if register is 32-bit
 */
bool kbase_reg_is_size32(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_is_valid - check register enum is valid and present in regmap
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Return: boolean if register is present and valid
 */
bool kbase_reg_is_valid(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_is_accessible - check register enum is accessible
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 * @flags:    Register permissions and size checks
 *
 * Return: boolean if register is accessible
 */
bool kbase_reg_is_accessible(struct kbase_device *kbdev, u32 reg_enum, u32 flags);

/**
 * kbase_reg_is_powered_access_allowed - check if registered is accessible given
 * current power state
 *
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Return: boolean if register is accessible
 */
bool kbase_reg_is_powered_access_allowed(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_is_init - check if regmap is initialized
 *
 * @kbdev:     Kbase device pointer
 * Return:     boolean if regmap is initialized
 */
bool kbase_reg_is_init(struct kbase_device *kbdev);

/**
 * kbase_reg_get_offset - get register offset from enum
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 * @offset:   Pointer to store value of register offset
 *
 * Return: 0 on success, otherwise a standard Linux error code
 */
int kbase_reg_get_offset(struct kbase_device *kbdev, u32 reg_enum, u32 *offset);

/**
 * kbase_reg_get_enum - get enum from register offset
 * @kbdev:    Kbase device pointer
 * @offset:   Register offset
 * @reg_enum: Pointer to store enum value
 *
 * Return: 0 on success, otherwise a standard Linux error code
 */
int kbase_reg_get_enum(struct kbase_device *kbdev, u32 offset, u32 *reg_enum);

#ifdef CONFIG_MALI_DEBUG
/**
 * kbase_reg_get_enum_string - get the string for a particular enum
 * @reg_enum: Register enum
 *
 * Return: string containing the name of enum
 */
const char *kbase_reg_get_enum_string(u32 reg_enum);
#endif /* CONFIG_MALI_DEBUG */

/**
 * kbase_reg_get_gpu_id - get GPU ID from register or dummy model
 * @kbdev:    Kbase device pointer
 *
 * Return: GPU ID on success, 0 otherwise.
 */
u64 kbase_reg_get_gpu_id(struct kbase_device *kbdev);

/**
 * kbase_regmap_init - regmap init function
 * @kbdev:    Kbase device pointer
 *
 * Return: 0 if successful, otherwise a standard Linux error code
 */
int kbase_regmap_init(struct kbase_device *kbdev);

/**
 * kbase_regmap_backend_init - Initialize register mapping backend
 * @kbdev:    Kbase device pointer
 *
 * Return: the arch_id of the selected look-up table.
 */
u32 kbase_regmap_backend_init(struct kbase_device *kbdev);

/**
 * kbase_regmap_term - regmap term function
 * @kbdev:    Kbase device pointer
 */
void kbase_regmap_term(struct kbase_device *kbdev);

/**
 * kbase_reg_poll32_timeout - Poll a 32 bit register with timeout
 * @kbdev:             Kbase device pointer
 * @reg_enum:          Register enum
 * @val:               Variable for result of read
 * @cond:              Condition to be met
 * @delay_us:          Delay between each poll (in uS)
 * @timeout_us:        Timeout (in uS)
 * @delay_before_read: If true delay for @delay_us before read
 *
 * Return: 0 if condition is met, -ETIMEDOUT if timed out.
 */
#define kbase_reg_poll32_timeout(kbdev, reg_enum, val, cond, delay_us, timeout_us,       \
				 delay_before_read)                                      \
	mali_read_poll_timeout_atomic(kbase_reg_read32, val, cond, delay_us, timeout_us, \
				      delay_before_read, kbdev, reg_enum)

/**
 * kbase_reg_poll64_timeout - Poll a 64 bit register with timeout
 * @kbdev:             Kbase device pointer
 * @reg_enum:          Register enum
 * @val:               Variable for result of read
 * @cond:              Condition to be met
 * @delay_us:          Delay between each poll (in uS)
 * @timeout_us:        Timeout (in uS)
 * @delay_before_read: If true delay for @delay_us before read
 *
 * Return: 0 if condition is met, -ETIMEDOUT if timed out.
 */
#define kbase_reg_poll64_timeout(kbdev, reg_enum, val, cond, delay_us, timeout_us,       \
				 delay_before_read)                                      \
	mali_read_poll_timeout_atomic(kbase_reg_read64, val, cond, delay_us, timeout_us, \
				      delay_before_read, kbdev, reg_enum)

/**
 * kbase_reg_gpu_irq_all - Return a mask for all GPU IRQ sources
 * @is_legacy:       Indicates a legacy GPU IRQ mask.
 *
 * Return: a mask for all GPU IRQ sources.
 *
 * Note that the following sources are not included:
 * CLEAN_CACHES_COMPLETED - Used separately for cache operation.
 * DOORBELL_MIRROR - Do not have it included for GPU_IRQ_REG_COMMON
 *                   as it can't be cleared by GPU_IRQ_CLEAR, thus interrupt storm might happen
 */
static inline u32 kbase_reg_gpu_irq_all(bool is_legacy)
{
	u32 mask = GPU_IRQ_REG_COMMON;

	if (is_legacy) {
#if MALI_USE_CSF
		mask |= (RESET_COMPLETED | POWER_CHANGED_ALL);
#endif /* MALI_USE_CSF */
		/* Include POWER_CHANGED_SINGLE in debug builds for use in irq latency test. */
		if (IS_ENABLED(CONFIG_MALI_DEBUG))
			mask |= POWER_CHANGED_SINGLE;
	}

	return mask;
}
#endif /* _MALI_KBASE_HW_ACCESS_H_ */
