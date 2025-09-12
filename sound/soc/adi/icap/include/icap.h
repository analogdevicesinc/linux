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
 * along with this program.
 */

/*
 * Authors:
 *   Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 */

#ifndef _ICAP_H_
#define _ICAP_H_

/**
 * @file icap.h
 * @author Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 * @brief Common ICAP header for both application and device side.
 * Don't include this file directly.
 * Include icap_application.h or icap_device.h file instead.
 *
 * @copyright 2021-2022 Analog Devices Inc.
 *
 */

#include "icap_config.h"
#include "icap_compiler.h"

#if defined(ICAP_LINUX_KERNEL_RPMSG)
#include "icap_linux_kernel_rpmsg.h"
#elif defined(ICAP_BM_RPMSG_LITE)
#include "icap_bm_rpmsg-lite.h"
#elif defined(ICAP_LINUX_RPMSG_CHARDEV)
#include "icap_linux_rpmsg_chardev.h"
#else
#error "Invalid platform"
#endif

/**
 * @defgroup error_codes Return error codes
 * @{
 */
#define ICAP_ERROR_INIT 6
#define ICAP_ERROR_NOMEM 12
#define ICAP_ERROR_BUSY 16
#define ICAP_ERROR_INVALID 22
#define ICAP_ERROR_BROKEN_CON 32
#define ICAP_ERROR_MSG_TYPE 42
#define ICAP_ERROR_PROTOCOL 71
#define ICAP_ERROR_MSG_ID 74
#define ICAP_ERROR_REMOTE_ADDR 78
#define ICAP_ERROR_MSG_LEN 90
#define ICAP_ERROR_PROTOCOL_NOT_SUP 93
#define ICAP_ERROR_TIMEOUT 110
#define ICAP_ERROR_NO_BUFS 233
#define ICAP_ERROR_NOT_SUP 252
/**@}*/

/**
 * @defgroup sample_format Sample format
 * @{
 */
#define ICAP_FORMAT_S8 0
#define ICAP_FORMAT_U8 1
#define ICAP_FORMAT_S16_LE 2
#define ICAP_FORMAT_S16_BE 3
#define ICAP_FORMAT_U16_LE 4
#define ICAP_FORMAT_U16_BE 5
#define ICAP_FORMAT_S24_LE 6
#define ICAP_FORMAT_S24_BE 7
#define ICAP_FORMAT_U24_LE 8
#define ICAP_FORMAT_U24_BE 9
#define ICAP_FORMAT_S32_LE 10
#define ICAP_FORMAT_S32_BE 11
#define ICAP_FORMAT_U32_LE 12
#define ICAP_FORMAT_U32_BE 13
#define ICAP_FORMAT_FLOAT_LE 14
#define ICAP_FORMAT_FLOAT_BE 15
#define ICAP_FORMAT_FLOAT64_LE 16
#define ICAP_FORMAT_FLOAT64_BE 17
/**@}*/

/**
 * @defgroup sample_format_bit Sample format bit field
 * @{
 */
#define ICAP_FMTBIT_S8 (1 << ICAP_FORMAT_S8)
#define ICAP_FMTBIT_U8 (1 << ICAP_FORMAT_U8)
#define ICAP_FMTBIT_S16_LE (1 << ICAP_FORMAT_S16_LE)
#define ICAP_FMTBIT_S16_BE (1 << ICAP_FORMAT_S16_BE)
#define ICAP_FMTBIT_U16_LE (1 << ICAP_FORMAT_U16_LE)
#define ICAP_FMTBIT_U16_BE (1 << ICAP_FORMAT_U16_BE)
#define ICAP_FMTBIT_S24_LE (1 << ICAP_FORMAT_S24_LE)
#define ICAP_FMTBIT_S24_BE (1 << ICAP_FORMAT_S24_BE)
#define ICAP_FMTBIT_U24_LE (1 << ICAP_FORMAT_U24_LE)
#define ICAP_FMTBIT_U24_BE (1 << ICAP_FORMAT_U24_BE)
#define ICAP_FMTBIT_S32_LE (1 << ICAP_FORMAT_S32_LE)
#define ICAP_FMTBIT_S32_BE (1 << ICAP_FORMAT_S32_BE)
#define ICAP_FMTBIT_U32_LE (1 << ICAP_FORMAT_U32_LE)
#define ICAP_FMTBIT_U32_BE (1 << ICAP_FORMAT_U32_BE)
#define ICAP_FMTBIT_FLOAT_LE (1 << ICAP_FORMAT_FLOAT_LE)
#define ICAP_FMTBIT_FLOAT_BE (1 << ICAP_FORMAT_FLOAT_BE)
#define ICAP_FMTBIT_FLOAT64_LE (1 << ICAP_FORMAT_FLOAT64_LE)
#define ICAP_FMTBIT_FLOAT64_BE (1 << ICAP_FORMAT_FLOAT64_BE)
/**@}*/

/**
 * @defgroup sample_rate Sample rate bit field
 * @{
 */
/** @brief 5.512kHz sample rate */
#define ICAP_RATE_5512	(1 << 0)
/** @brief 8kHz sample rate */
#define ICAP_RATE_8000	(1 << 1)
/** @brief 11.025kHz sample rate */
#define ICAP_RATE_11025	(1 << 2)
/** @brief 16kHz sample rate */
#define ICAP_RATE_16000	(1 << 3)
/** @brief 22.05kHz sample rate */
#define ICAP_RATE_22050	(1 << 4)
/** @brief 32kHz sample rate */
#define ICAP_RATE_32000	(1 << 5)
/** @brief 44.1kHz sample rate */
#define ICAP_RATE_44100	(1 << 6)
/** @brief 48kHz sample rate */
#define ICAP_RATE_48000	(1 << 7)
/** @brief 64kHz sample rate */
#define ICAP_RATE_64000	(1 << 8)
/** @brief 88.2kHz sample rate */
#define ICAP_RATE_88200	(1 << 9)
/** @brief 96kHz sample rate */
#define ICAP_RATE_96000	(1 << 10)
/** @brief 176.4kHz sample rate */
#define ICAP_RATE_176400	(1 << 11)
/** @brief 192kHz sample rate */
#define ICAP_RATE_192000	(1 << 12)
/** @brief 352.8kHz sample rate */
#define ICAP_RATE_352800	(1 << 13)
/** @brief 384kHz sample rate */
#define ICAP_RATE_384000	(1 << 14)
/** @brief Linear range of frequencies */
#define ICAP_RATE_ALL_FREQ	(1 << 30)

/** @brief Range 8kHz to 44.1kHz */
#define ICAP_RATES_8000_44100 ( \
		ICAP_RATE_8000 | ICAP_RATE_11025 | ICAP_RATE_16000 | \
		ICAP_RATE_22050 | ICAP_RATE_32000 | ICAP_RATE_44100)
/** @brief Range 8kHz to 48kHz */
#define ICAP_RATES_8000_48000 (ICAP_RATES_8000_44100 | ICAP_RATE_48000)
/** @brief Range 8kHz to 96kHz */
#define ICAP_RATES_8000_96000 ( \
		ICAP_RATES_8000_48000 | ICAP_RATE_64000 | \
		ICAP_RATE_88200 | ICAP_RATE_96000)
/** @brief Range 8kHz to 192kHz */
#define ICAP_RATES_8000_192000 (ICAP_RATES_8000_96000 | ICAP_RATE_176400 | ICAP_RATE_192000)
/** @brief Range 8kHz to 384kHz */
#define ICAP_RATES_8000_384000 (ICAP_RATES_8000_192000 | ICAP_RATE_352800 | ICAP_RATE_384000)
/**@}*/

/** @brief Max length of ICAP buffer name /ref icap_buf_descriptor.name */
#define ICAP_BUF_NAME_LEN (64)
#define ICAP_BUF_MAX_FRAGS_OFFSETS_NUM (64)

/** @brief ICAP subdevice type */
enum icap_dev_type {
	ICAP_DEV_PLAYBACK = 0, /**< Playback subdevice */
	ICAP_DEV_RECORD = 1, /**< Record subdevice */
};

/** @brief Audio buffer type, defines buffer type in the icap_buf_descriptor.type */
enum icap_buf_type {
	/** Audio fragments are in sequence, separated by gaps, if
	 * icap_buf_descriptor.gap_size = 0 the buffer is continuous
	 */
	ICAP_BUF_CIRCURAL = 0,

	/* Audio fragments are scattered, needs new #icap_buf_offsets
	 * after fragments are consumed
	 */
	ICAP_BUF_SCATTERED = 1,
};

/** @brief ICAP instance, initialized by icap_device_init() or
 * icap_application_init() except of some members in #icap_transport
 */
struct icap_instance {
	/** @brief Platform specific transport internals, some fields of this struct
	 * must be initialized before icap_device_init() or icap_application_init()
	 */
	struct icap_transport transport;

	/** @brief Optional ICAP instance name */
	char *name;

	/** @brief ICAP instance type, one of the #icap_instance_type */
	u32 type;

	/** @brief Private pointer for caller use */
	void *priv;

	/** @brief Pointer to callbacks #icap_device_callbacks or icap_application_callbacks */
	void *callbacks;

	/** @brief Internal counter for messages */
	u32 seq_num;
};

/**
 * @defgroup msg_structs Message structs send and received by application and device sides.
 * @{
 */
/** @brief ICAP buffer descriptor */
ICAP_PACKED_BEGIN
struct icap_buf_descriptor {
	/** @brief Optional buffer name */
	char name[ICAP_BUF_NAME_LEN];

	/** @brief Subdevice id to which the buffer should be attached */
	s32 subdev_id;

	/** @brief Pointer to shared memory with the audio data */
	u64 buf;

	/** @brief Size of the shared memory */
	u32 buf_size;

	/** @brief Buffer type, one of the #icap_buf_type */
	u32 type;

	/** @brief Gaps between audio fragments in the shared memory */
	u32 gap_size;

	/** @brief Audio fragments size in the shared memory */
	u32 frag_size;

	/** @brief Number of channels */
	u32 channels;

	/** @brief Sample format, one of the @ref sample_format */
	u32 format;

	/** @brief Sample rate, integer frequency value which corresponds to @ref sample_rate */
	u32 rate;

	/* @brief Set this flag if ICAP device must report that it
	 * consumed audio fragment from this buffer
	 */
	u32 report_frags;
} ICAP_PACKED_END;

/** @brief Struct send by icap_frag_ready() device function */
ICAP_PACKED_BEGIN
struct icap_buf_frags {
	/** @brief Indicates buffer which the fragments were consumed */
	u32 buf_id;

	/** @brief Indicates how many audio fragments were consumed */
	u32 frags;
} ICAP_PACKED_END;

/* @brief Struct send by icap_frags() application function,
 * used with #ICAP_BUF_SCATTERED buffer type
 */
ICAP_PACKED_BEGIN
struct icap_buf_offsets {
	/** @brief Indicates buffer which the offset table refers to */
	u32 buf_id;

	/** @brief Number of valid offsets in the table */
	u32 num;

	/** @brief Offset table with the new audio fragments */
	u32 frags_offsets[ICAP_BUF_MAX_FRAGS_OFFSETS_NUM];
} ICAP_PACKED_END;

/** @brief Subdevice requested by icap_get_subdevice_features() */
ICAP_PACKED_BEGIN
struct icap_subdevice_features {
	/** @brief One of the #icap_dev_type */
	u32 type;

	/** @brief Max number of supported source buffers */
	u32 src_buf_max;

	/** @brief Max number of supported destination buffers */
	u32 dst_buf_max;

	/** @brief Min number of supported channels */
	u32 channels_min;

	/** @brief Max number of supported channels */
	u32 channels_max;

	/** @brief Supported sample formats, bitfield @ref sample_format_bit */
	u32 formats;

	/** @brief Supported sample rates, bitfield @ref sample_rate */
	u32 rates;
} ICAP_PACKED_END;

/** @brief Subdevice params to be initialized with, send by icap_subdevice_init() */
struct icap_subdevice_params {
	/** @brief Subdevice id to be initialized */
	u32 subdev_id;

	/** @brief Number of channels requested */
	u32 channels;

	/** @brief Sample format requested, one of the @ref sample_format */
	u32 format;

	/** @brief Integer value of the sample rate frequency */
	u32 rate;
} ICAP_PACKED_END;

/**@}*/

/** @brief Used to verify remote address, only rpmsg supported currently */
union icap_remote_addr {
	u32 rpmsg_addr;
	void *tcpip_addr;
};

/**
 * @defgroup msg_handlers Handling of the ICAP messages.
 * @{
 */

/**
 * @brief Parse received ICAP message.
 *
 * @param icap Pointer to icap instance
 * @param src_addr Source address of the message
 * @param data Pointer to a message buffer
 * @param size Length of the message
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_parse_msg(struct icap_instance *icap, union icap_remote_addr *src_addr,
		       void *data, u32 size);

/**
 * @brief Save a message in a queue to be parsed later by icap_loop()
 *
 * @param icap Pointer to icap instance
 * @param src_addr Source address of the message
 * @param data Pointer to a message buffer
 * @param size Length of the message
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_put_msg(struct icap_instance *icap, union icap_remote_addr *src_addr,
		     void *data, u32 size);

/**
 * @brief Parses messages saved by icap_put_msg()
 *
 * @param icap Pointer to icap instance
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
s32 icap_loop(struct icap_instance *icap);

/**@}*/

#endif /* _ICAP_H_ */
