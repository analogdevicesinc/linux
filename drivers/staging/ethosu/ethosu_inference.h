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

#ifndef ETHOSU_INFERENCE_H
#define ETHOSU_INFERENCE_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_rpmsg.h"
#include "uapi/ethosu.h"

#include <linux/kref.h>
#include <linux/types.h>
#include <linux/wait.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct ethosu_buffer;
struct ethosu_core_inference_rsp;
struct ethosu_device;
struct ethosu_network;
struct ethosu_uapi_inference_create;
struct file;

/**
 * struct ethosu_inference - Inference struct
 * @edev:			Arm Ethos-U device
 * @file:			File handle
 * @kref:			Reference counter
 * @waitq:			Wait queue
 * @done:			Wait condition is done
 * @ifm:			Pointer to IFM buffer
 * @ofm:			Pointer to OFM buffer
 * @net:			Pointer to network
 * @status:			Inference status
 * @pmu_event_config:		PMU event configuration
 * @pmu_event_count:		PMU event count after inference
 * @pmu_cycle_counter_enable:	PMU cycle counter config
 * @pmu_cycle_counter_count:	PMU cycle counter count after inference
 * @msg:			Rpmsg message
 */
struct ethosu_inference {
	struct ethosu_device    *edev;
	struct file             *file;
	struct kref             kref;
	wait_queue_head_t       waitq;
	bool                    done;
	uint32_t                ifm_count;
	struct ethosu_buffer    *ifm[ETHOSU_FD_MAX];
	uint32_t                ofm_count;
	struct ethosu_buffer    *ofm[ETHOSU_FD_MAX];
	struct ethosu_network   *net;
	enum ethosu_uapi_status status;
	uint8_t                 pmu_event_config[ETHOSU_PMU_EVENT_MAX];
	uint32_t                pmu_event_count[ETHOSU_PMU_EVENT_MAX];
	uint32_t                pmu_cycle_counter_enable;
	uint64_t                pmu_cycle_counter_count;
	uint32_t                inference_type;
	struct ethosu_rpmsg_msg msg;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * ethosu_inference_create() - Create inference
 *
 * This function must be called in the context of a user space process.
 *
 * Return: fd on success, else error code.
 */
int ethosu_inference_create(struct ethosu_device *edev,
			    struct ethosu_network *net,
			    struct ethosu_uapi_inference_create *uapi);

/**
 * ethosu_inference_get_from_fd() - Get inference handle from fd
 *
 * This function must be called from a user space context.
 *
 * Return: Pointer on success, else ERR_PTR.
 */
struct ethosu_inference *ethosu_inference_get_from_fd(int fd);

/**
 * ethosu_inference_get() - Get inference
 */
void ethosu_inference_get(struct ethosu_inference *inf);

/**
 * ethosu_inference_put() - Put inference
 *
 * Return: 1 if object was removed, else 0.
 */
int ethosu_inference_put(struct ethosu_inference *inf);

/**
 * ethosu_inference_rsp() - Handle inference response
 */
void ethosu_inference_rsp(struct ethosu_device *edev,
			  struct ethosu_core_inference_rsp *rsp);

#endif /* ETHOSU_INFERENCE_H */
