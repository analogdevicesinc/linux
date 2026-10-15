/* SPDX-License-Identifier: (GPL-2.0-or-later OR Apache-2.0) */

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

#ifndef _ICAP_TRANSPORT_H_
#define _ICAP_TRANSPORT_H_

/**
 * @file icap_transport.h
 * @author Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 * @brief Private header file with messaging definitions.
 *
 * @copyright Copyright 2021-2022 Analog Devices Inc.
 *
 */

#include "../../include/icap.h"

#define ICAP_PROTOCOL_VERSION (1)

/**
 * @brief ICAP message type
 *
 * Each ICAP message should have corresponding response from remote core.
 */
enum icap_msg_type {
	ICAP_MSG = 0, /**< Message. */
	ICAP_ACK = 1, /**< Positive response. */
	ICAP_NAK = 2, /**< Negative response. */
};

/**
 * @brief ICAP command definitions
 *
 */
enum icap_msg_cmd {
	/* Control commands */
	ICAP_MSG_GET_DEV_NUM = 9, /**< Get number of subdevices. */
	ICAP_MSG_GET_DEV_FEATURES = 10, /**< Get subdevice features. */
	ICAP_MSG_DEV_INIT = 11, /**< Init subdevice. */
	ICAP_MSG_DEV_DEINIT = 12, /**< Deinit subdevice. */

	/* Stream commands */
	ICAP_MSG_ADD_SRC = 50, /**< Add source buffer. */
	ICAP_MSG_ADD_DST = 51, /**< Add destination buffer. */
	ICAP_MSG_REMOVE_SRC = 52, /**< Remove source buffer. */
	ICAP_MSG_REMOVE_DST = 53, /**< Remove destination buffer. */
	ICAP_MSG_START = 54, /**< Start subdevice. */
	ICAP_MSG_STOP = 55, /**< Stop subdevice. */
	ICAP_MSG_PAUSE = 56, /**< Pause subdevice. */
	ICAP_MSG_RESUME = 57, /**< Resume subdevice. */
	ICAP_MSG_BUF_OFFSETS = 58, /* < Send offsets for new fragments,
				    * used in #ICAP_BUF_SCATTERED.
				    */
	ICAP_MSG_FRAG_READY = 59, /**< Audio fragment consumed. */
	ICAP_MSG_XRUN = 60, /**< Report buffer xrun. */

	/* Other messages */
	ICAP_MSG_ERROR = 200, /**< Report error. */
};

/**
 * @brief Message payload, valid union field depends on command.
 *
 */
ICAP_PACKED_BEGIN
union icap_msg_payload {
	u8 bytes[ICAP_BUF_NAME_LEN];
	char name[ICAP_BUF_NAME_LEN];
	u32 u32;
	s32 s32;
	struct icap_buf_descriptor buf;
	struct icap_buf_frags frags;
	struct icap_buf_offsets offsets;
	struct icap_subdevice_features features;
	struct icap_subdevice_params dev_params;
} ICAP_PACKED_END;

/**
 * @brief Message header with control fields.
 *
 */
ICAP_PACKED_BEGIN
struct icap_msg_header {
	u32 protocol_version; /**< ICAP protocol version. */
	u32 seq_num; /**< Sequence number of a message, increments every msg.*/
	u32 cmd; /**< Command ID of the message.*/
	u32 type; /* < Specifies if message or response to a message:
			* ICAP_MSG, ICAP_ACK, ICAP_NAK.
			*/
	u32 reserved[5]; /**< Reserved for future use.*/
	u32 payload_len; /**< Payload length in bytes.*/
} ICAP_PACKED_END;

/**
 * @brief ICAP message definition.
 *
 */
ICAP_PACKED_BEGIN
struct icap_msg {
	struct icap_msg_header header;
	union icap_msg_payload payload;
} ICAP_PACKED_END;

/**
 * @brief Initializes platform specific transport layer.
 *
 * @param icap Pointer to ICAP instance.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
s32 icap_init_transport(struct icap_instance *icap);

/**
 * @brief Releases platform specific transport layer.
 *
 * @param icap Pointer to ICAP instance.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
s32 icap_deinit_transport(struct icap_instance *icap);

/**
 * @brief Verifies if source address is correct.
 *
 * @param icap Pointer to ICAP instance.
 * @param src_addr Source address to verify.
 * @return int32_t Returns 0 when address is correct, -ICAP_ERROR_REMOTE_ADDR if wrong.
 */
s32 icap_verify_remote(struct icap_instance *icap, union icap_remote_addr *src_addr);

/**
 * @brief Send ICAP message using platform specific transport.
 *
 * @param icap Pointer to ICAP instance.
 * @param data Pointer to ICAP message.
 * @param size Totall size of the ICAP message.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
s32 icap_send_platform(struct icap_instance *icap, void *data, u32 size);

/**
 * @brief Notifies about received response, may unblock a thread waiting for the response.
 * May be called in interrupt context.
 *
 * @param icap Pointer to ICAP instance.
 * @param response Pointer to response message received.
 * @return int32_t Returns -ICAP_ERROR_TIMEOUT if nobody waits
 * for the message, 0 otherwise.
 */
s32 icap_response_notify(struct icap_instance *icap, struct icap_msg *response);

/**
 * @brief Allows platform to prepare for expected response before sending the message.
 *
 * @param icap Pointer to ICAP instance.
 * @param msg Pointer to ICAP message which response to is expected.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
s32 icap_prepare_wait(struct icap_instance *icap, struct icap_msg *msg);

/**
 * @brief Puts the thread into sleep while waiting for response.
 *
 * @param icap Pointer to ICAP instance.
 * @param seq_num Sequence number of the expected response.
 * @param response If not NULL the expected response is copied to the struct.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
s32 icap_wait_for_response(struct icap_instance *icap,
			       u32 seq_num, struct icap_msg *response);

/**
 * @brief Lock critical section.
 *
 * @param icap Pointer to ICAP instance.
 */
void icap_platform_lock(struct icap_instance *icap);

/**
 * @brief Unlock critical section.
 *
 * @param icap Pointer to ICAP instance.
 */
void icap_platform_unlock(struct icap_instance *icap);

#endif /* _ICAP_TRANSPORT_H_ */
