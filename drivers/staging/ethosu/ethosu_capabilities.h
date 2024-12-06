/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Arm Limited.
 */

#ifndef ETHOSU_CAPABILITIES_H
#define ETHOSU_CAPABILITIES_H

/****************************************************************************
 * Includes
 ****************************************************************************/

#include "ethosu_core_interface.h"
#include "ethosu_rpmsg.h"

#include <linux/types.h>
#include <linux/completion.h>

/****************************************************************************
 * Types
 ****************************************************************************/

struct ethosu_device;
struct ethosu_uapi_device_capabilities;

/**
 * struct ethosu_capabilities - Capabilities internal struct
 */
struct ethosu_capabilities {
	struct ethosu_device                   *edev;
	struct completion                      done;
	struct ethosu_uapi_device_capabilities *uapi;
	struct ethosu_rpmsg_msg                msg;
	int                                    errno;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

int ethosu_capabilities_request(struct ethosu_device *edev,
				struct ethosu_uapi_device_capabilities *uapi);

void ethosu_capability_rsp(struct ethosu_device *edev,
			   struct ethosu_core_msg_capabilities_rsp *rsp);

#endif
