// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/phy.h>
#include <linux/io.h>
#include "adrv906x-tsu.h"
#include "adrv906x-net.h"

void adrv906x_tsu_set_phy_delay(void __iomem *base, u32 phy_delay_tx, u32 phy_delay_rx)
{
	iowrite32(phy_delay_tx, base + ADRV906X_TSU_STATIC_PHY_DELAY_TX);
	iowrite32(phy_delay_rx, base + ADRV906X_TSU_STATIC_PHY_DELAY_RX);
}

void adrv906x_tsu_set_ptp_timestamping_mode(void __iomem *base)
{
	u32 mode, val;

	mode = ADRV906X_PTP_TIMESTAMPING_MODE_TWO_STEP;

	val = ioread32(base + ADRV906X_TSU_TIMESTAMPING_MODE);
	val &= ~ADRV906X_PTP_TIMESTAMPING_MODE;
	val |= (mode & ADRV906X_PTP_TIMESTAMPING_MODE);

	iowrite32(val, base + ADRV906X_TSU_TIMESTAMPING_MODE);
}

void adrv906x_tsu_set_speed(struct adrv906x_tsu *tsu, struct phy_device *phydev)
{
	u32 mode, val;
	void __iomem *base;

	base = tsu->base;
	mode = (phydev->speed == SPEED_25000) ? ADRV906X_CORE_SPEED_25G : ADRV906X_CORE_SPEED_10G;

	val = ioread32(base + ADRV906X_TSU_TIMESTAMPING_MODE);
	val &= ~ADRV906X_CORE_SPEED;
	val |= (mode & ADRV906X_CORE_SPEED);

	iowrite32(val, base + ADRV906X_TSU_TIMESTAMPING_MODE);
}

int adrv906x_tsu_setup(struct platform_device *pdev, struct adrv906x_tsu *tsu, struct device_node *eth_port_np)
{
	struct device *dev = &pdev->dev;
	u32 val, frac_val, reg, len;
	int ret, port_id;

	ret = of_property_read_u32(eth_port_np, "id", &port_id);
	if (ret < 0) {
		dev_err(dev, "dt: id missing");
		return -ENODEV;
	} else if (port_id >= MAX_NETDEV_NUM) {
		dev_err(dev, "dt: id %d  is out of range", port_id);
		return -ENODEV;
	}

	of_property_read_u32_index(eth_port_np, "reg", 6, &reg);
	of_property_read_u32_index(eth_port_np, "reg", 7, &len);

	tsu->base = devm_ioremap(dev, reg, len);
	if (IS_ERR(tsu->base))
		return PTR_ERR(tsu->base);

	ret = of_property_read_u32(eth_port_np, "static-phy-delay-tx-ns", &val);
	if (ret < 0)
		dev_warn(dev, "dt: static-phy-delay-tx-ns missing, using 0");

	ret = of_property_read_u32(eth_port_np, "static-phy-delay-tx-frac-ns", &frac_val);
	if (ret < 0) {
		dev_warn(dev, "dt: static-phy-delay-tx-frac-ns missing, using 0");
		frac_val = 0;
	}

	tsu->phy_delay_tx = (val << 16) | (frac_val & 0xFFFF);
	dev_info(dev, "tsu static phy delay tx 0x%08x", tsu->phy_delay_tx);

	ret = of_property_read_u32(eth_port_np, "static-phy-delay-rx-ns", &val);
	if (ret < 0) {
		dev_warn(dev, "dt: static-phy-delay-rx-ns missing, using 0");
		val = 0;
	}

	ret = of_property_read_u32(eth_port_np, "static-phy-delay-rx-frac-ns", &frac_val);
	if (ret < 0) {
		dev_warn(dev, "dt: static-phy-delay-rx-frac-ns missing, using 0");
		frac_val = 0;
	}

	tsu->phy_delay_rx = (val << 16) | (frac_val & 0xFFFF);
	dev_info(dev, "tsu static phy delay rx 0x%08x", tsu->phy_delay_rx);

	adrv906x_tsu_set_phy_delay(tsu->base, tsu->phy_delay_tx, tsu->phy_delay_rx);
	adrv906x_tsu_set_ptp_timestamping_mode(tsu->base);

	return 0;
}
