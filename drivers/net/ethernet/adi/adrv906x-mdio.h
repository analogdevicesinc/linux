// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_MDIO_H__
#define __ADRV906X_MDIO_H__

#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/io.h>

int adrv906x_mdio_probe(struct platform_device *pdev, struct net_device *ndev,
			struct device_node *mdio_np);
#endif /* __ADRV906X_MDIO_H__ */
