// SPDX-License-Identifier: GPL-2.0-or-later

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
 * along with this program.
 */

/*
 * Authors:
 *   Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

/**
 * @file icap_linux_kernel_rpmsg.c
 * @author Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 * @brief ICAP implementation for Linux kernel rpmsg platform.
 *
 * @copyright Copyright 2021-2022 Analog Devices Inc.
 *
 */

#include "icap_transport.h"

#ifdef ICAP_LINUX_KERNEL_RPMSG

#include <linux/types.h>
#include <linux/kobject.h>

#define __ICAP_MSG_TIMEOUT usecs_to_jiffies(ICAP_MSG_TIMEOUT_US)

s32 icap_init_transport(struct icap_instance *icap)
{
	struct icap_transport *transport = &icap->transport;

	mutex_init(&transport->rpdev_lock);
	mutex_init(&transport->platform_lock);
	mutex_init(&transport->response_lock);
	spin_lock_init(&transport->skb_spinlock);
	init_waitqueue_head(&transport->response_event);
	skb_queue_head_init(&transport->response_queue);
	return 0;
}

s32 icap_deinit_transport(struct icap_instance *icap)
{
	struct icap_transport *transport = &icap->transport;
	struct sk_buff *skb;
	unsigned long flags;

	mutex_lock(&transport->rpdev_lock);

	spin_lock_irqsave(&transport->skb_spinlock, flags);
	while (!skb_queue_empty(&transport->response_queue)) {
		skb = skb_dequeue(&transport->response_queue);
		kfree_skb(skb);
	}
	spin_unlock_irqrestore(&transport->skb_spinlock, flags);

	transport->rpdev = NULL;

	mutex_unlock(&transport->rpdev_lock);
	return 0;
}

s32 icap_verify_remote(struct icap_instance *icap,
		union icap_remote_addr *src_addr)
{
	/* rpmsg endpoints on linux are one to one - no need to verify src address*/
	return 0;
}

s32 icap_send_platform(struct icap_instance *icap, void *data, u32 size)
{
	struct icap_transport *transport = &icap->transport;
	int32_t ret;

	mutex_lock(&transport->rpdev_lock);
	if (transport->rpdev != NULL)
		ret = rpmsg_send(transport->rpdev->ept, data, size);
	else
		ret = -ICAP_ERROR_BROKEN_CON;
	mutex_unlock(&transport->rpdev_lock);
	return ret;
}

struct _icap_wait_hint {
	u32 received;
	u32 seq_num;
	u32 msg_cmd;
};

static
struct sk_buff *_find_seq_num(struct sk_buff_head *queue, u32 seq_num)
{
	struct _icap_wait_hint *hint;
	struct sk_buff *skb;

	skb_queue_walk(queue, skb) {
		hint = (struct _icap_wait_hint *)skb->head;
		if (hint->seq_num == seq_num)
			return skb;
	}
	return NULL;
}

s32 icap_prepare_wait(struct icap_instance *icap, struct icap_msg *msg)
{
	struct icap_transport *transport = &icap->transport;
	struct sk_buff *skb;
	struct _icap_wait_hint *hint;
	unsigned long flags;
	int32_t ret = 0;

	mutex_lock(&transport->rpdev_lock);

	if (transport->rpdev == NULL) {
		ret = -ICAP_ERROR_BROKEN_CON;
		goto prepare_wait_unlock;
	}

	skb = alloc_skb(sizeof(struct _icap_wait_hint) + sizeof(struct icap_msg), GFP_KERNEL);
	if (!skb) {
		ret = -ENOMEM;
		goto prepare_wait_unlock;
	}

	skb_reserve(skb, sizeof(struct _icap_wait_hint));

	hint = (struct _icap_wait_hint *)skb->head;
	hint->received = 0;
	hint->msg_cmd = msg->header.cmd;
	hint->seq_num = msg->header.seq_num;

	spin_lock_irqsave(&transport->skb_spinlock, flags);
	skb_queue_tail(&transport->response_queue, skb);
	spin_unlock_irqrestore(&transport->skb_spinlock, flags);

prepare_wait_unlock:
	mutex_unlock(&transport->rpdev_lock);
	return ret;
}

s32 icap_response_notify(struct icap_instance *icap, struct icap_msg *response)
{
	struct icap_transport *transport = &icap->transport;
	struct sk_buff *skb;
	struct _icap_wait_hint *hint;
	unsigned long flags;
	uint32_t size;
	int32_t ret;

	spin_lock_irqsave(&transport->skb_spinlock, flags);
	skb = _find_seq_num(&transport->response_queue, response->header.seq_num);
	if (skb != NULL) {
		/* Waiter found, copy msg to its buffer */
		size = sizeof(response->header) + response->header.payload_len;
		skb_put_data(skb, response, size);
		hint = (struct _icap_wait_hint *)skb->head;
		hint->received = 1;
		wake_up_interruptible_all(&transport->response_event);
		ret = 0;
	} else {
		/*
		 * Got a unexpected or very late message,
		 * waiter could timeout and remove from the hint from the queue.
		 * Drop the message.
		 */
		ret = -ICAP_ERROR_TIMEOUT;
	}
	spin_unlock_irqrestore(&transport->skb_spinlock, flags);
	return ret;
}

s32 icap_wait_for_response(struct icap_instance *icap, u32 seq_num,
		struct icap_msg *response)
{
	struct icap_transport *transport = &icap->transport;
	struct device *dev;
	u8 icap_id;
	char _env[64];
	char *envp[] = { _env, NULL };
	long timeout;
	unsigned long flags;
	struct sk_buff *skb;
	struct _icap_wait_hint *hint;
	struct icap_msg *tmp_msg;
	int32_t ret;

	mutex_lock(&transport->response_lock);
	spin_lock_irqsave(&transport->skb_spinlock, flags);
	skb = _find_seq_num(&transport->response_queue, seq_num);
	spin_unlock_irqrestore(&transport->skb_spinlock, flags);
	mutex_unlock(&transport->response_lock);

	if (skb == NULL) {
		/* This should never happen */
		return -ICAP_ERROR_PROTOCOL;
	}
	hint = (struct _icap_wait_hint *)skb->head;

	timeout = wait_event_interruptible_timeout(transport->response_event,
						   hint->received,
						   __ICAP_MSG_TIMEOUT);

	/* Remove the skb from response queue */
	mutex_lock(&transport->response_lock);
	spin_lock_irqsave(&transport->skb_spinlock, flags);
	skb_unlink(skb, &transport->response_queue);
	spin_unlock_irqrestore(&transport->skb_spinlock, flags);
	mutex_unlock(&transport->response_lock);

	if (timeout > 0) {
		/* Got response in time */
		tmp_msg = (struct icap_msg *)skb->data;
		if (tmp_msg->header.type == ICAP_NAK) {
			ret = tmp_msg->payload.s32;
		} else {
			if (response)
				memcpy(response, skb->data, skb->len);
			ret = 0;
		}
	} else if (timeout < 0) {
		/* Got error */
		ret = timeout;
	} else {
		mutex_lock(&transport->rpdev_lock);

		if (transport->rpdev == NULL) {
			ret = -ICAP_ERROR_BROKEN_CON;
		} else {
			dev = &transport->rpdev->dev;
			icap_id = transport->rpdev->dst;
			/* Timeout */
			snprintf(_env, sizeof(_env), "EVENT=ICAP%d_MSG%d_TIMEOUT",
				 icap_id, hint->msg_cmd);
			kobject_uevent_env(&dev->kobj, KOBJ_CHANGE, envp);
			ret = -ETIMEDOUT;
		}
		mutex_unlock(&transport->rpdev_lock);
	}

	kfree_skb(skb);
	return ret;
}

void icap_platform_lock(struct icap_instance *icap)
{
	struct icap_transport *transport = &icap->transport;

	mutex_lock(&transport->platform_lock);
}

void icap_platform_unlock(struct icap_instance *icap)
{
	struct icap_transport *transport = &icap->transport;

	mutex_unlock(&transport->platform_lock);
}

#endif /* ICAP_LINUX_KERNEL_RPMSG */
