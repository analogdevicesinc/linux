/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Arm Limited.
 */

#ifndef ETHOSU_NETWORK_INFO_H
#define ETHOSU_NETWORK_INFO_H

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
struct ethosu_network;
struct ethosu_uapi_network_info;

struct ethosu_network_info {
	struct ethosu_device            *edev;
	struct ethosu_network           *net;
	struct ethosu_uapi_network_info *uapi;
	struct completion               done;
	int                             errno;
	struct ethosu_rpmsg_msg         msg;
};

/****************************************************************************
 * Functions
 ****************************************************************************/

/**
 * ethosu_network_info_request() - Send a network info request
 *
 * This function must be called in the context of a user space process.
 *
 * Return: 0 on success, .
 */
int ethosu_network_info_request(struct ethosu_network *net,
				struct ethosu_uapi_network_info *uapi);

/**
 * ethosu_network_info_rsp() - Handle network info response.
 */
void ethosu_network_info_rsp(struct ethosu_device *edev,
			     struct ethosu_core_network_info_rsp *rsp);

#endif /* ETHOSU_NETWORK_INFO_H */
