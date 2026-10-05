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

#ifndef _ICAP_DEVICE_H_
#define _ICAP_DEVICE_H_

/**
 * @file icap_device.h
 * @author Piotr Wojtaszczyk <piotr.wojtaszczyk@timesys.com>
 * @brief ICAP definitions for device side.
 *
 * @copyright Copyright 2021-2022 Analog Devices Inc.
 *
 */

#include "icap.h"

/**
 * @addtogroup dev_functions
 * @{
 */

/**
 * @brief Callbacks used on device side, executed when appropriate application message
 * received by device side, please see @ref app_functions.
 *
 * The #get_subdevices and #get_subdevice_features are mandatory,
 * other callbacks are optional. If a callback isn't implemented ICAP by default
 * responses with #ICAP_ACK to the message received.
 * Implementation of a callback should return 0 on success (except #get_subdevices),
 * a negative error code on failure or if a received parameter is invalid.
 *
 */
struct icap_device_callbacks {
	/** @brief Mandatory - Device callback for icap_get_subdevices(),
	 * the callback should return number of supported subdevices.
	 */
	int32_t (*get_subdevices)(struct icap_instance *icap);

	/* @brief Mandatory - Device callback for get_subdevice_features(),
	 * returns features of a subdevice.
	 */
	int32_t (*get_subdevice_features)(struct icap_instance *icap, uint32_t subdev_id,
					  struct icap_subdevice_features *features);
	int32_t (*subdevice_init)(struct icap_instance *icap,
				  struct icap_subdevice_params *params);
	int32_t (*subdevice_deinit)(struct icap_instance *icap, uint32_t subdev_id);
	int32_t (*add_src)(struct icap_instance *icap, struct icap_buf_descriptor *buf);
	int32_t (*add_dst)(struct icap_instance *icap, struct icap_buf_descriptor *buf);
	int32_t (*remove_src)(struct icap_instance *icap, uint32_t buf_id);
	int32_t (*remove_dst)(struct icap_instance *icap, uint32_t buf_id);
	int32_t (*start)(struct icap_instance *icap, uint32_t subdev_id);
	int32_t (*stop)(struct icap_instance *icap, uint32_t subdev_id);
	int32_t (*pause)(struct icap_instance *icap, uint32_t subdev_id);
	int32_t (*resume)(struct icap_instance *icap, uint32_t subdev_id);
	int32_t (*frags)(struct icap_instance *icap, struct icap_buf_offsets *offsets);

	/** @brief Callback executed when a response to icap_frag_ready() is received. */
	int32_t (*frag_ready_response)(struct icap_instance *icap, int32_t buf_id);

	/** @brief Callback executed when a response to icap_xrun() is received. */
	int32_t (*xrun_response)(struct icap_instance *icap, int32_t buf_id);

	/** @brief Callback executed when a response to icap_error() is received. */
	int32_t (*error_response)(struct icap_instance *icap, int32_t error);
};

/**@}*/

/**
 * @addtogroup init_functions
 * @{
 */

/**
 * @brief Initializes device side ICAP instance,
 * requires some fields in icap_instance.transport initialized depending on platform.
 *
 * @param icap Pointer to new instance struct, the struct can be empty
 *  except some fields in icap_instance.transport.
 * @param name Optional ICAP instance name.
 * @param cb Pointer to #icap_device_callbacks.
 * @param priv Private pointer for caller use.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_device_init(struct icap_instance *icap, char *name,
			 struct icap_device_callbacks *cb, void *priv);

/**
 * @brief Deinitialize ICAP instance and frees allocated resources.
 *
 * @param icap Pointer to ICAP instance.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_device_deinit(struct icap_instance *icap);

/**@}*/

/**
 * @defgroup dev_functions Device side functions
 *
 * Each function sends appropriate ICAP message to ICAP application which
 * triggers appropriate application callback #icap_application_callbacks
 * (if implemented), doesn't wait for response therefore it's safe to use the
 * functions in interrupt context.
 *
 * When a response is received to the ICAP message appropriate callback is
 * executed:
 * - icap_device_callbacks.frag_ready_response()
 * - icap_device_callbacks.xrun_response()
 * - icap_device_callbacks.error_response()
 *
 * @{
 */

/**
 * @brief Device should call this function when icap_buf_descriptor.report_frags
 * is set and an audio fragment/s was consumed from the buffer by the device.
 *
 * @param icap Pointer to ICAP instance.
 * @param frags Pointer to struct containing buffer id and number of fragments consumed.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_frag_ready(struct icap_instance *icap, struct icap_buf_frags *frags);

/**
 * @brief Device can call this function if xrun event is detected.
 *
 * @param icap Pointer to ICAP instance.
 * @param frags Pointer to struct containing buffer id and number of fragments affected.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_xrun(struct icap_instance *icap, struct icap_buf_frags *frags);

/**
 * @brief Device can call this function to report an error condition in the device.
 *
 * @param icap Pointer to ICAP instance.
 * @param error Positive error code.
 * @return int32_t Returns 0 on success, negative error code on failure.
 */
int32_t icap_error(struct icap_instance *icap, uint32_t error);

/**@}*/

#endif /* _ICAP_DEVICE_H_ */
