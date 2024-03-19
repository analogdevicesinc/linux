// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#ifndef __ADRV906X_MACSEC_EXT_H__
#define __ADRV906X_MACSEC_EXT_H__

#include <linux/types.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/atomic.h>
#include <linux/io.h>
#include "macsec/cco_macsec.h"

struct adrv906x_macsec_priv {
	struct device *dev;
	struct cco_macsec_priv priv;
	void *base;
	int irq;
	bool enabled;
};

struct adrv906x_macsec_priv *adrv906x_macsec_get(struct net_device *netdev);
void adrv906x_macsec_commonport_status_update(struct net_device *netdev);
int adrv906x_macsec_probe(struct platform_device *pdev, struct net_device *netdev,
			  struct device_node *np);

#endif /* __ADRV906X_MACSEC_EXT_H__ */
