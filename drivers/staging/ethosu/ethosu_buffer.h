/*
 * Copyright (c) 2020,2022 ARM Limited.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
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
 * SPDX-License-Identifier: GPL-2.0-only
 */

#ifndef ETHOSU_BUFFER_H
#define ETHOSU_BUFFER_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include <linux/kref.h>
#include <linux/types.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct ethosu_device;
struct device;

/**
 * struct ethosu_buffer - Buffer
 * @dev:		Device
 * @file:		File
 * @kref:		Reference counting
 * @capacity:		Maximum capacity of the buffer
 * @offset:		Offset to first byte of buffer
 * @size:		Size of the data in the buffer
 * @cpu_addr:		Kernel mapped address
 * @dma_addr:		DMA address
 * @dma_addr_orig:	Original DMA address before range mapping
 *
 * 'offset + size' must not be larger than 'capacity'.
 */
struct ethosu_buffer {
	struct ethosu_device *edev;
	struct file          *file;
	struct kref          kref;
	size_t               capacity;
	size_t               offset;
	size_t               size;
	void                 *cpu_addr;
	dma_addr_t           dma_addr;
	dma_addr_t           dma_addr_orig;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * ethosu_buffer_create() - Create buffer
 *
 * This function must be called in the context of a user space process.
 *
 * Return: fd on success, else error code.
 */
int ethosu_buffer_create(struct ethosu_device *edev,
			 size_t capacity);

/**
 * ethosu_buffer_get_from_fd() - Get buffer handle from fd
 *
 * This function must be called from a user space context.
 *
 * Return: Pointer on success, else ERR_PTR.
 */
struct ethosu_buffer *ethosu_buffer_get_from_fd(int fd);

/**
 * ethosu_buffer_get() - Put buffer
 */
void ethosu_buffer_get(struct ethosu_buffer *buf);

/**
 * ethosu_buffer_put() - Put buffer
 */
void ethosu_buffer_put(struct ethosu_buffer *buf);

/**
 * ethosu_buffer_resize() - Resize and validate buffer
 *
 * Return: 0 on success, else error code.
 */
int ethosu_buffer_resize(struct ethosu_buffer *buf,
			 size_t size,
			 size_t offset);

#endif /* ETHOSU_BUFFER_H */
