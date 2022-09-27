/*
 * Copyright (c) 2020,2022 Arm Limited.
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

#ifndef ETHOSU_NETWORK_H
#define ETHOSU_NETWORK_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include <linux/kref.h>
#include <linux/types.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct ethosu_buffer;
struct ethosu_device;
struct ethosu_uapi_network_create;
struct device;
struct file;

struct ethosu_network {
	struct ethosu_device *edev;
	struct file          *file;
	struct kref          kref;
	struct ethosu_buffer *buf;
	u32                  index;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * ethosu_network_create() - Create network
 *
 * This function must be called in the context of a user space process.
 *
 * Return: fd on success, else error code.
 */
int ethosu_network_create(struct ethosu_device *edev,
			  struct ethosu_uapi_network_create *uapi);

/**
 * ethosu_network_get_from_fd() - Get network handle from fd
 *
 * This function must be called from a user space context.
 *
 * Return: Pointer on success, else ERR_PTR.
 */
struct ethosu_network *ethosu_network_get_from_fd(int fd);

/**
 * ethosu_network_get() - Get network
 */
void ethosu_network_get(struct ethosu_network *net);

/**
 * ethosu_network_put() - Put network
 *
 * Return: 1 if object was removed, else 0.
 */
int ethosu_network_put(struct ethosu_network *net);

#endif /* ETHOSU_NETWORK_H */
