// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include "adrv906x-net.h"

struct adrv906x_mdio_priv {
	struct device *dev;
	void __iomem *pcs_base[MAX_NETDEV_NUM];
};

static int adrv906x_pseudo_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 val)
{
	struct adrv906x_mdio_priv *priv = bus->priv;
	u32 offset;

	offset = 4 * (regnum & 0xFFFF);

	iowrite32(val, priv->pcs_base[mii_id % MAX_NETDEV_NUM] + offset);

	return 0;
}

static int adrv906x_pseudo_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct adrv906x_mdio_priv *priv = bus->priv;
	u32 offset;
	int ret;

	offset = 4 * (regnum & 0xFFFF);

	ret = ioread32(priv->pcs_base[mii_id % MAX_NETDEV_NUM] + offset) & 0xFFFF;

	return ret;
}

int adrv906x_mdio_probe(struct platform_device *pdev, struct net_device *ndev,
			struct device_node *mdio_np)
{
	struct device *dev = &pdev->dev;
	struct adrv906x_mdio_priv *priv;
	struct mii_bus *bus;
	int idx, ret;
	u32 reg, len;

	bus = devm_mdiobus_alloc_size(&pdev->dev, sizeof(*priv));
	if (!bus) {
		dev_err(dev, "failed to allocate private driver data");
		return -ENOMEM;
	}

	priv = bus->priv;
	priv->dev = &pdev->dev;

	for (idx = 0; idx < MAX_NETDEV_NUM; idx++) {
		of_property_read_u32_index(mdio_np, "reg", 2 * idx, &reg);
		of_property_read_u32_index(mdio_np, "reg", 2 * idx + 1, &len);
		priv->pcs_base[idx] = devm_ioremap(dev, reg, len);

		if (IS_ERR(priv->pcs_base[idx]))
			return PTR_ERR(priv->pcs_base[idx]);
	}

	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%d", pdev->name, pdev->id);
	bus->name = "adrv906x-pseudo-mdio";
	bus->read = adrv906x_pseudo_mdio_read,
	bus->write = adrv906x_pseudo_mdio_write,
	bus->parent = priv->dev;

	ret = of_mdiobus_register(bus, mdio_np);
	if (ret) {
		dev_err(dev, "failed to register mdio bus");
		return ret;
	}

	return 0;
}
