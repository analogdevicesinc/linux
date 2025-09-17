/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2021-2022 Analog Devices Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program
 */

/*
 * Authors:
 *   Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

#ifndef _ICAP_LINUX_KERNEL_RPMSG_H_
#define _ICAP_LINUX_KERNEL_RPMSG_H_

/**
 * @file icap_linux_kernel_rpmsg.h
 * @author Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 * @brief ICAP `icap_transport` definition for Linux kernel platform.
 *
 * @copyright Copyright 2021-2022 Analog Devices Inc.
 *
 */

#include <linux/types.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <linux/rpmsg.h>
#include <linux/skbuff.h>

/**
 * @brief ICAP `icap_transport` for Linux kernel ICAP implementation.
 *
 */
struct icap_transport {
	/** @brief This field needs to be set to appropriate `struct rpmsg_device`
	 * before ICAP initialization icap_application_init() or icap_device_init().
	 */
	struct rpmsg_device *rpdev;
	struct mutex rpdev_lock;
	spinlock_t skb_spinlock;
	struct sk_buff_head response_queue;
	struct wait_queue_head response_event;
	struct mutex response_lock;
	struct mutex platform_lock;
};

#endif /* _ICAP_LINUX_KERNEL_RPMSG_H_ */
