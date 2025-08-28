// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_MDIO_H__
#define __ADRV906X_MDIO_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include "adrv906x-net.h"

int adrv906x_mdio_register(struct adrv906x_eth_if *eth_if, struct device_node *mdio_np);
void adrv906x_mdio_unregister(struct adrv906x_eth_if *eth_if);

#endif /* __ADRV906X_MDIO_H__ */
