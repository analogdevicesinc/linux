/*
 * (C) COPYRIGHT 2020 ARM Limited. All rights reserved.
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

#ifndef ETHOSU_MAILBOX_H
#define ETHOSU_MAILBOX_H

/****************************************************************************
 * Includes
 ****************************************************************************/
#include "ethosu_core_interface.h"

#include <linux/types.h>
#include <linux/mailbox_client.h>
#include <linux/workqueue.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct device;
struct ethosu_buffer;
struct ethosu_device;
struct ethosu_core_msg;
struct ethosu_core_queue;
struct resource;

typedef void (*ethosu_mailbox_cb)(void *user_arg);

struct ethosu_mailbox {
	struct device            *dev;
	struct workqueue_struct  *wq;
	struct work_struct       work;
	struct ethosu_core_queue __iomem *in_queue;
	struct ethosu_core_queue __iomem *out_queue;
	struct mbox_client       client;
	struct mbox_chan         *rx;
	struct mbox_chan         *tx;
	ethosu_mailbox_cb        callback;
	void                     *user_arg;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * ethosu_mailbox_init() - Initialize mailbox
 *
 * Return: 0 on success, else error code.
 */
int ethosu_mailbox_init(struct ethosu_mailbox *mbox,
			struct device *dev,
			struct resource *in_queue,
			struct resource *out_queue,
			ethosu_mailbox_cb callback,
			void *user_arg);

/**
 * ethosu_mailbox_deinit() - Deinitialize mailbox
 */
void ethosu_mailbox_deinit(struct ethosu_mailbox *mbox);

/**
 * ethosu_mailbox_read() - Read message from mailbox
 *
 * Return: 0 message read, else error code.
 */
int ethosu_mailbox_read(struct ethosu_mailbox *mbox,
			struct ethosu_core_msg *header,
			void *data,
			size_t length);

/**
 * ethosu_mailbox_reset() - Reset to end of queue
 */
void ethosu_mailbox_reset(struct ethosu_mailbox *mbox);

/**
 * ethosu_mailbox_ping() - Send ping message
 *
 * Return: 0 on success, else error code.
 */
int ethosu_mailbox_ping(struct ethosu_mailbox *mbox);

/**
 * ethosu_mailbox_pong() - Send pong response
 *
 * Return: 0 on success, else error code.
 */
int ethosu_mailbox_pong(struct ethosu_mailbox *mbox);

/**
 * ethosu_mailbox_version_response - Send version request
 *
 * Return: 0 on succes, else error code
 */
int ethosu_mailbox_version_request(struct ethosu_mailbox *mbox);

/**
 * ethosu_mailbox_capabilities_request() - Send capabilities request
 *
 * Return: 0 on success, else error code.
 */
int ethosu_mailbox_capabilities_request(struct ethosu_mailbox *mbox,
					void *user_arg);

/**
 * ethosu_mailbox_inference() - Send inference
 *
 * Return: 0 on success, else error code.
 */
int ethosu_mailbox_inference(struct ethosu_mailbox *mbox,
			     void *user_arg,
			     uint32_t ifm_count,
			     struct ethosu_buffer **ifm,
			     uint32_t ofm_count,
			     struct ethosu_buffer **ofm,
			     struct ethosu_buffer *network,
			     uint8_t *pmu_event_config,
			     uint8_t pmu_event_config_count,
			     uint8_t pmu_cycle_counter_enable);

#endif /* ETHOSU_MAILBOX_H */
