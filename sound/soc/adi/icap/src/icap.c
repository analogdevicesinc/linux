// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  Copyright 2021-2022 Analog Devices Inc.
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

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

/**
 * @file icap.c
 * @author Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 * @brief ICAP (Inter Core Audio Protocol) platform independent source code.
 *
 * @copyright Copyright 2021-2022 Analog Devices Inc.
 */

#include "../include/icap_application.h"
#include "../include/icap_device.h"
#include "platform/icap_transport.h"

/**
 * @brief ICAP instance type.
 *
 * ICAP specifies communication between application and device.
 * Application side sends audio for playback and receives recorded audio.
 * Device side receives audio  for playback and sends recorded audio.
 */
enum icap_instance_type {
	ICAP_APPLICATION_INSTANCE = 0, /**< ICAP application instance. */
	ICAP_DEVICE_INSTANCE = 1, /**< ICAP device instance. */
};

int32_t icap_application_init(struct icap_instance *icap, char *name,
		struct icap_application_callbacks *cb, void *priv)
{
	if ((icap == NULL) || (cb == NULL))
		return -ICAP_ERROR_INVALID;

	if (icap->callbacks != NULL)
		return -ICAP_ERROR_BUSY;

	icap->name = name;
	icap->type = ICAP_APPLICATION_INSTANCE;
	icap->priv = priv;
	icap->callbacks = cb;
	icap->seq_num = 0;
	return icap_init_transport(icap);
}

int32_t icap_application_deinit(struct icap_instance *icap)
{
	if (icap->callbacks == NULL)
		return 0;

	icap->callbacks = NULL;
	return icap_deinit_transport(icap);
}

int32_t icap_device_init(struct icap_instance *icap, char *name,
		struct icap_device_callbacks *cb, void *priv)
{
	if ((icap == NULL) || (cb == NULL))
		return -ICAP_ERROR_INVALID;

	icap->name = name;
	icap->type = ICAP_DEVICE_INSTANCE;
	icap->priv = priv;
	icap->callbacks = cb;
	icap->seq_num = 0;
	return icap_init_transport(icap);
}

int32_t icap_device_deinit(struct icap_instance *icap)
{
	if (icap->callbacks == NULL)
		return 0;

	icap->callbacks = NULL;
	return icap_deinit_transport(icap);
}

static
s32 icap_send_msg(struct icap_instance *icap, enum icap_msg_cmd cmd,
		void *data, u32 size, u32 sync, struct icap_msg *response)
{
	struct icap_msg msg;
	uint32_t seq_num;
	int32_t ret;

	/* Copy data to msg payload */
	if (data) {
		if (size > sizeof(msg.payload))
			return -ICAP_ERROR_MSG_LEN;
		memcpy(&msg.payload, data, size);
	} else {
		size = 0;
	}

	/* Increment the seq_num */
	icap_platform_lock(icap);
	icap->seq_num++;
	seq_num = icap->seq_num;
	icap_platform_unlock(icap);

	/* Initialize msg header */
	msg.header.protocol_version = ICAP_PROTOCOL_VERSION;
	msg.header.seq_num = seq_num;
	msg.header.cmd = cmd;
	msg.header.type = ICAP_MSG;
	memset(&msg.header.reserved, 0, sizeof(msg.header.reserved));
	msg.header.payload_len = size;

	size += sizeof(msg.header);

	if (sync) {
		ret = icap_prepare_wait(icap, &msg);
		if (ret)
			return ret;
	}

	ret = icap_send_platform(icap, &msg, size);

	if (!sync)
		return ret;

	return icap_wait_for_response(icap, seq_num, response);
}

static
s32 icap_send_response(struct icap_instance *icap, enum icap_msg_cmd cmd,
		enum icap_msg_type type, u32 seq_num, void *data, u32 size)
{
	struct icap_msg msg;

	/* Copy data to response payload */
	if (data) {
		if (size > sizeof(msg.payload))
			return -ICAP_ERROR_MSG_LEN;
		memcpy(&msg.payload, data, size);
	} else {
		size = 0;
	}

	/* Initialize msg header */
	msg.header.protocol_version = ICAP_PROTOCOL_VERSION;
	msg.header.seq_num = seq_num;
	msg.header.cmd = cmd;
	msg.header.type = type;
	memset(&msg.header.reserved, 0, sizeof(msg.header.reserved));
	msg.header.payload_len = size;

	size += sizeof(msg.header);

	return icap_send_platform(icap, &msg, size);
}

static
s32 icap_send_ack(struct icap_instance *icap, enum icap_msg_cmd cmd,
		u32 seq_num, void *data, u32 size)
{
	return icap_send_response(icap, cmd, ICAP_ACK, seq_num, data, size);
}

static
s32 icap_send_nak(struct icap_instance *icap, enum icap_msg_cmd cmd,
		u32 seq_num, s32 error)
{
	return icap_send_response(icap, cmd, ICAP_NAK, seq_num, &error, sizeof(error));
}

s32 icap_get_subdevices(struct icap_instance *icap)
{
	struct icap_msg response;
	int32_t ret;

	ret = icap_send_msg(icap, ICAP_MSG_GET_DEV_NUM, NULL, 0, 1, &response);
	if (ret)
		return ret;

	if (response.header.payload_len != sizeof(uint32_t))
		return -ICAP_ERROR_MSG_LEN;

	return response.payload.s32;
}

s32 icap_get_subdevice_features(struct icap_instance *icap, u32 subdev_id,
		struct icap_subdevice_features *features)
{
	struct icap_msg response;
	int32_t ret;

	if (features == NULL)
		return -ICAP_ERROR_INVALID;

	ret = icap_send_msg(icap, ICAP_MSG_GET_DEV_FEATURES, &subdev_id,
			    sizeof(subdev_id), 1, &response);
	if (ret)
		return ret;

	if (response.header.payload_len != sizeof(struct icap_subdevice_features))
		return -ICAP_ERROR_MSG_LEN;

	memcpy(features, &response.payload, sizeof(struct icap_subdevice_features));
	return 0;
}

s32 icap_subdevice_init(struct icap_instance *icap,
		struct icap_subdevice_params *params)
{
	if (params == NULL)
		return -ICAP_ERROR_INVALID;

	return icap_send_msg(icap, ICAP_MSG_DEV_INIT, params,
			     sizeof(struct icap_subdevice_params), 1, NULL);
}

s32 icap_subdevice_deinit(struct icap_instance *icap, u32 subdev_id)
{
	return icap_send_msg(icap, ICAP_MSG_DEV_DEINIT, &subdev_id, sizeof(subdev_id), 1, NULL);
}

s32 icap_add_src(struct icap_instance *icap, struct icap_buf_descriptor *buf)
{
	struct icap_msg response;
	int32_t ret;

	if (buf == NULL)
		return -ICAP_ERROR_INVALID;

	ret = icap_send_msg(icap, ICAP_MSG_ADD_SRC, buf,
			    sizeof(struct icap_buf_descriptor), 1, &response);
	if (ret)
		return ret;

	if (response.header.payload_len != sizeof(uint32_t))
		return -ICAP_ERROR_MSG_LEN;

	return response.payload.s32;
}

s32 icap_add_dst(struct icap_instance *icap, struct icap_buf_descriptor *buf)
{
	struct icap_msg response;
	int32_t ret;

	if (buf == NULL)
		return -ICAP_ERROR_INVALID;

	ret = icap_send_msg(icap, ICAP_MSG_ADD_DST, buf,
			    sizeof(struct icap_buf_descriptor), 1, &response);
	if (ret)
		return ret;

	if (response.header.payload_len != sizeof(uint32_t))
		return -ICAP_ERROR_MSG_LEN;

	return response.payload.s32;
}

s32 icap_remove_src(struct icap_instance *icap, u32 buf_id)
{
	return icap_send_msg(icap, ICAP_MSG_REMOVE_SRC, &buf_id, sizeof(buf_id), 1, NULL);
}

s32 icap_remove_dst(struct icap_instance *icap, u32 buf_id)
{
	return icap_send_msg(icap, ICAP_MSG_REMOVE_DST, &buf_id, sizeof(buf_id), 1, NULL);
}

s32 icap_start(struct icap_instance *icap, u32 subdev_id)
{
	return icap_send_msg(icap, ICAP_MSG_START, &subdev_id, sizeof(subdev_id), 1, NULL);
}

s32 icap_stop(struct icap_instance *icap, u32 subdev_id)
{
	return icap_send_msg(icap, ICAP_MSG_STOP, &subdev_id, sizeof(subdev_id), 1, NULL);
}

s32 icap_pause(struct icap_instance *icap, u32 subdev_id)
{
	return icap_send_msg(icap, ICAP_MSG_PAUSE, &subdev_id, sizeof(subdev_id), 1, NULL);
}

s32 icap_resume(struct icap_instance *icap, u32 subdev_id)
{
	return icap_send_msg(icap, ICAP_MSG_RESUME, &subdev_id, sizeof(subdev_id), 1, NULL);
}

s32 icap_frags(struct icap_instance *icap, struct icap_buf_offsets *offsets)
{
	if (offsets == NULL)
		return -ICAP_ERROR_INVALID;

	return icap_send_msg(icap, ICAP_MSG_BUF_OFFSETS, offsets,
			     sizeof(struct icap_buf_offsets), 1, NULL);
}

s32 icap_frag_ready(struct icap_instance *icap, struct icap_buf_frags *frags)
{
	if (frags == NULL)
		return -ICAP_ERROR_INVALID;

	return icap_send_msg(icap, ICAP_MSG_FRAG_READY, frags,
			     sizeof(struct icap_buf_frags), 0, NULL);
}

s32 icap_xrun(struct icap_instance *icap, struct icap_buf_frags *frags)
{
	if (frags == NULL)
		return -ICAP_ERROR_INVALID;

	return icap_send_msg(icap, ICAP_MSG_XRUN, frags, sizeof(struct icap_buf_frags), 0, NULL);
}

s32 icap_error(struct icap_instance *icap, u32 error)
{
	return icap_send_msg(icap, ICAP_MSG_ERROR, &error, sizeof(error), 0, NULL);
}

static
s32 icap_application_parse_response(struct icap_instance *icap,
		struct icap_msg *msg)
{
	/*
	 * Currently all responses to application are for synchronous messages
	 * notify the waiter.
	 */
	return icap_response_notify(icap, msg);
}

static
s32 icap_application_parse_msg(struct icap_instance *icap,
		struct icap_msg *msg)
{
	struct icap_application_callbacks *cb = icap->callbacks;
	struct icap_msg_header *msg_header = &msg->header;
	int32_t send_generic_ack = 1;
	uint32_t buf_id;
	int32_t ret = 0;

	switch (msg_header->cmd) {
	case ICAP_MSG_FRAG_READY:
		if (cb->frag_ready) {
			buf_id = msg->payload.frags.buf_id;
			ret = cb->frag_ready(icap, &msg->payload.frags);
			if (ret == 0) {
				icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
					      msg_header->seq_num, &buf_id, sizeof(buf_id));
				send_generic_ack = 0;
			}
		} else {
			buf_id = msg->payload.frags.buf_id;
			icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
				      msg_header->seq_num, &buf_id, sizeof(buf_id));
			send_generic_ack = 0;
		}
		break;
	case ICAP_MSG_XRUN:
		if (cb->xrun) {
			buf_id = msg->payload.frags.buf_id;
			ret = cb->xrun(icap, &msg->payload.frags);
			if (ret == 0) {
				icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
					      msg_header->seq_num, &buf_id, sizeof(buf_id));
				send_generic_ack = 0;
			}
		} else {
			buf_id = msg->payload.frags.buf_id;
			icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
				      msg_header->seq_num, &buf_id, sizeof(buf_id));
			send_generic_ack = 0;
		}
		break;
	case ICAP_MSG_ERROR:
		if (cb->error)
			ret = cb->error(icap, msg->payload.s32);
		break;
	default:
		ret = -ICAP_ERROR_MSG_ID;
		break;
	}

	if (send_generic_ack) {
		if (ret)
			icap_send_nak(icap, (enum icap_msg_cmd)msg_header->cmd,
				      msg_header->seq_num, ret);
		else
			icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
				      msg_header->seq_num, NULL, 0);
	}

	return 0;
}

static
s32 icap_device_parse_response(struct icap_instance *icap,
		struct icap_msg *msg)
{
	struct icap_device_callbacks *cb = (struct icap_device_callbacks *)icap->callbacks;
	struct icap_msg_header *msg_header = &msg->header;
	int32_t ret = 0;
	int32_t error;

	/*
	 * Currently all responses to device are for asynchronous messages
	 * execute a response callback.
	 */

	if (msg_header->type == ICAP_NAK)
		error = msg->payload.s32;
	else
		error = 0;

	switch (msg_header->cmd) {
	case ICAP_MSG_FRAG_READY:
		if (cb->frag_ready_response) {
			if (msg_header->type == ICAP_ACK)
				error = msg->payload.s32;
			ret = cb->frag_ready_response(icap, error);
		}
		break;
	case ICAP_MSG_XRUN:
		if (cb->xrun_response) {
			if (msg_header->type == ICAP_ACK)
				error = msg->payload.s32;
			ret = cb->xrun_response(icap, error);
		}
		break;
	case ICAP_MSG_ERROR:
		if (cb->error_response)
			ret = cb->error_response(icap, error);
		break;
	default:
		ret = -ICAP_ERROR_MSG_ID;
		break;
	}
	return ret;
}

static
s32 icap_device_parse_msg(struct icap_instance *icap, struct icap_msg *msg)
{
	struct icap_device_callbacks *cb = (struct icap_device_callbacks *)icap->callbacks;
	struct icap_msg_header *msg_header = &msg->header;
	int32_t send_generic_ack = 1;
	int32_t ret = 0;
	uint32_t buf_id;
	uint32_t dev_num;
	struct icap_subdevice_features features;

	switch (msg_header->cmd) {
	case ICAP_MSG_GET_DEV_NUM:
		if (cb->get_subdevices) {
			ret = cb->get_subdevices(icap);
			if (ret >= 0) {
				dev_num = ret;
				icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
					      msg_header->seq_num, &dev_num, sizeof(dev_num));
				send_generic_ack = 0;
			}
		}
		break;
	case ICAP_MSG_GET_DEV_FEATURES:
		if (cb->get_subdevice_features) {
			ret = cb->get_subdevice_features(icap, msg->payload.u32, &features);
			if (ret >= 0) {
				icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
					      msg_header->seq_num, &features,
					      sizeof(struct icap_subdevice_features));
				send_generic_ack = 0;
			}
		}
		break;
	case ICAP_MSG_DEV_INIT:
		if (cb->subdevice_init)
			ret = cb->subdevice_init(icap, &msg->payload.dev_params);
		break;
	case ICAP_MSG_DEV_DEINIT:
		if (cb->subdevice_deinit)
			ret = cb->subdevice_deinit(icap, msg->payload.u32);
		break;
	case ICAP_MSG_ADD_SRC:
		if (cb->add_src) {
			ret = cb->add_src(icap, &msg->payload.buf);
			if (ret >= 0) {
				buf_id = ret;
				icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
					      msg_header->seq_num, &buf_id, sizeof(buf_id));
				send_generic_ack = 0;
			}
		}
		break;
	case ICAP_MSG_ADD_DST:
		if (cb->add_dst) {
			ret = cb->add_dst(icap, &msg->payload.buf);
			if (ret >= 0) {
				buf_id = ret;
				icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
					      msg_header->seq_num, &buf_id, sizeof(buf_id));
				send_generic_ack = 0;
			}
		}
		break;
	case ICAP_MSG_REMOVE_SRC:
		if (cb->remove_src)
			ret = cb->remove_src(icap, msg->payload.u32);
		break;
	case ICAP_MSG_REMOVE_DST:
		if (cb->remove_dst)
			ret = cb->remove_dst(icap, msg->payload.u32);
		break;
	case ICAP_MSG_START:
		if (cb->start)
			ret = cb->start(icap, msg->payload.u32);
		break;
	case ICAP_MSG_STOP:
		if (cb->stop)
			ret = cb->stop(icap, msg->payload.u32);
		break;
	case ICAP_MSG_PAUSE:
		if (cb->pause)
			ret = cb->pause(icap, msg->payload.u32);
		break;
	case ICAP_MSG_RESUME:
		if (cb->resume)
			ret = cb->resume(icap, msg->payload.u32);
		break;
	case ICAP_MSG_BUF_OFFSETS:
		if (cb->frags)
			ret = cb->frags(icap, &msg->payload.offsets);
		break;
	default:
		ret = -ICAP_ERROR_MSG_ID;
		break;
	}

	if (send_generic_ack) {
		if (ret)
			icap_send_nak(icap, (enum icap_msg_cmd)msg_header->cmd,
				      msg_header->seq_num, ret);
		else
			icap_send_ack(icap, (enum icap_msg_cmd)msg_header->cmd,
				      msg_header->seq_num, NULL, 0);
	}
	return 0;
}

s32 icap_parse_msg(struct icap_instance *icap,
		union icap_remote_addr *src_addr, void *data, u32 size)
{
	struct icap_msg *msg = (struct icap_msg *)data;
	struct icap_msg_header *msg_header = &msg->header;
	int32_t ret;

	if (icap->callbacks == NULL)
		return -ICAP_ERROR_INIT;

	if (msg_header->protocol_version != ICAP_PROTOCOL_VERSION)
		return -ICAP_ERROR_PROTOCOL_NOT_SUP;

	if (size != sizeof(struct icap_msg_header) + msg_header->payload_len)
		return -ICAP_ERROR_MSG_LEN;

	ret = icap_verify_remote(icap, src_addr);
	if (ret)
		return ret;

	if ((msg_header->type == ICAP_ACK) || (msg_header->type == ICAP_NAK)) {
		if (icap->type == ICAP_APPLICATION_INSTANCE)
			return icap_application_parse_response(icap, msg);
		else
			return icap_device_parse_response(icap, msg);
	}

	if (msg_header->type == ICAP_MSG) {
		if (icap->type == ICAP_APPLICATION_INSTANCE)
			return icap_application_parse_msg(icap, msg);
		else
			return icap_device_parse_msg(icap, msg);
	}

	return -ICAP_ERROR_MSG_TYPE;
}
