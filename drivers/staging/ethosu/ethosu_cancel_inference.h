/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Arm Limited.
 */

#ifndef ETHOSU_CANCEL_INFERENCE_H
#define ETHOSU_CANCEL_INFERENCE_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_rpmsg.h"
#include "uapi/ethosu.h"

#include <linux/types.h>
#include <linux/completion.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct ethosu_core_cancel_inference_rsp;
struct ethosu_device;
struct ethosu_uapi_cancel_inference_status;
struct ethosu_inference;

struct ethosu_cancel_inference {
	struct ethosu_device                       *edev;
	struct ethosu_inference                    *inf;
	struct ethosu_uapi_cancel_inference_status *uapi;
	struct completion                          done;
	struct ethosu_rpmsg_msg                    msg;
	int                                        errno;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * ethosu_cancel_inference_request() - Send cancel inference request
 *
 * Return: 0 on success, error code otherwise.
 */
int ethosu_cancel_inference_request(struct ethosu_inference *inf,
				    struct ethosu_uapi_cancel_inference_status *uapi);

/**
 * ethosu_cancel_inference_rsp() - Handle cancel inference response
 */
void ethosu_cancel_inference_rsp(struct ethosu_device *edev,
				 struct ethosu_core_cancel_inference_rsp *rsp);

#endif /* ETHOSU_CANCEL_INFERENCE_H */
