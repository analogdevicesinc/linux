/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 *
 * (C) COPYRIGHT 2023 ARM Limited. All rights reserved.
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

#define KBASE_REGMAP_PERM_READ (1U << 0)
#define KBASE_REGMAP_PERM_WRITE (1U << 1)
#define KBASE_REGMAP_WIDTH_32_BIT (1U << 2)
#define KBASE_REGMAP_WIDTH_64_BIT (1U << 3)

/**
 * kbase_reg_read32 - read from 32-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
 *
 * Return: Value in desired register
 */
u32 kbase_reg_read32(struct kbase_device *kbdev, u32 reg_enum);

/**
 * kbase_reg_read64 - read from 64-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 *
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
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
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
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
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
 */
void kbase_reg_write32(struct kbase_device *kbdev, u32 reg_enum, u32 value);

/**
 * kbase_reg_write64 - write to 64-bit GPU register
 * @kbdev:    Kbase device pointer
 * @reg_enum: Register enum
 * @value:    Value to write
 *
 * Caller must ensure the GPU is powered (@kbdev->pm.gpu_powered != false).
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

#endif /* _MALI_KBASE_HW_ACCESS_H_ */
