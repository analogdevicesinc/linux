/*
 * Cadence Macb mdio controller driver
 *
 * Copyright (C) 2014 - 2018 Xilinx, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/io.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/ptp_clock_kernel.h>
#include "macb.h"


struct macb_mdio_data {
	void __iomem *regs;

	struct clk *pclk;
	struct clk *hclk;
};

#define macb_mdio_reg_writel(bp, offset, value)	\
	writel_relaxed(value, bp->regs + offset)
#define macb_mdio_writel(bp, reg, value)	\
	macb_mdio_reg_writel(bp, MACB_##reg, value)

#define macb_mdio_reg_readl(bp, offset)	readl_relaxed(bp->regs + offset)
#define macb_mdio_readl(bp, reg)	macb_mdio_reg_readl(bp, MACB_##reg)

#define MACB_MDIO_TIMEOUT	1000

static int macb_mdio_wait_for_idle(struct macb_mdio_data *bp)
{
	ulong timeout;

	timeout = jiffies + msecs_to_jiffies(MACB_MDIO_TIMEOUT);
	/* wait for end of transfer */
	while (1) {
		if (MACB_BFEXT(IDLE, macb_mdio_readl(bp, NSR)))
			break;

		if (time_after_eq(jiffies, timeout)) {
			//netdev_err(bp->dev, "wait for end of transfer timed out\n");
			return -ETIMEDOUT;
		}

		cpu_relax();
	}

	return 0;
}

static int macb_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct macb_mdio_data *bp = bus->priv;
	int value;
	int err;

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	macb_mdio_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF) |
				   MACB_BF(RW, MACB_MAN_READ) |
				   MACB_BF(PHYA, mii_id) |
				   MACB_BF(REGA, regnum) |
				   MACB_BF(CODE, MACB_MAN_CODE)));

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	value = MACB_BFEXT(DATA, macb_mdio_readl(bp, MAN));

	return value;
}

static int macb_mdio_write(struct mii_bus *bus, int mii_id, int regnum,
			   u16 value)
{
	struct macb_mdio_data *bp = bus->priv;
	int err;

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	macb_mdio_writel(bp, MAN, (MACB_BF(SOF, MACB_MAN_SOF) |
				   MACB_BF(RW, MACB_MAN_WRITE) |
				   MACB_BF(PHYA, mii_id) |
				   MACB_BF(REGA, regnum) |
				   MACB_BF(CODE, MACB_MAN_CODE) |
				   MACB_BF(DATA, value)));

	err = macb_mdio_wait_for_idle(bp);
	if (err < 0)
		return err;

	return 0;
}

static u32 gem_mdc_clk_div(struct macb_mdio_data *bp)
{
	u32 config;
	unsigned long pclk_hz = clk_get_rate(bp->pclk);

	if (pclk_hz <= 20000000)
		config = GEM_BF(CLK, GEM_CLK_DIV8);
	else if (pclk_hz <= 40000000)
		config = GEM_BF(CLK, GEM_CLK_DIV16);
	else if (pclk_hz <= 80000000)
		config = GEM_BF(CLK, GEM_CLK_DIV32);
	else if (pclk_hz <= 120000000)
		config = GEM_BF(CLK, GEM_CLK_DIV48);
	else if (pclk_hz <= 160000000)
		config = GEM_BF(CLK, GEM_CLK_DIV64);
	else
		config = GEM_BF(CLK, GEM_CLK_DIV96);

	return config;
}

static int macb_mdio_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	struct mii_bus *bus;
	struct macb_mdio_data *bp;
	struct resource *res;
	int ret;
	u32 config, i;

	bus = mdiobus_alloc_size(sizeof(*bp));
	if (!bus)
		return -ENOMEM;

	bus->name = "macb_mii_bus";
	bus->read = &macb_mdio_read;
	bus->write = &macb_mdio_write;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-mii", dev_name(&pdev->dev));
	bus->parent = &pdev->dev;

	bp = bus->priv;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	bp->regs = devm_ioremap(&pdev->dev, res->start, resource_size(res));
	if (IS_ERR(bp->regs)) {
		ret = PTR_ERR(bp->regs);
		goto err_out_free_mdiobus;
	}

	bp->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(bp->pclk)) {
		ret = PTR_ERR(bp->pclk);
		dev_err(&pdev->dev, "failed to get macb_clk (%u)\n", ret);
		goto err_out_free_mdiobus;
	}

	bp->hclk = devm_clk_get(&pdev->dev, "hclk");
	if (IS_ERR(bp->hclk)) {
		ret = PTR_ERR(bp->hclk);
		dev_err(&pdev->dev, "failed to get hclk (%u)\n", ret);
		goto err_out_free_mdiobus;
	}

	ret = clk_prepare_enable(bp->pclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pclk (%u)\n", ret);
		goto err_out_free_mdiobus;
	}

	ret = clk_prepare_enable(bp->hclk);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable hclk (%u)\n", ret);
		goto err_disable_pclk;
	}

	platform_set_drvdata(pdev, bus);

	/* Enable management port */
	config = macb_mdio_readl(bp, NCR);
	config |= MACB_BIT(MPE);
	macb_mdio_writel(bp, NCR, config);
	config = gem_mdc_clk_div(bp);
	macb_mdio_writel(bp, NCFGR, config);

	np = pdev->dev.of_node;
	if (np) {
		/* try dt phy registration */
		ret = of_mdiobus_register(bus, np);

		/* Fallback to standard phy registration if no phy were
		 * found during dt phy registration
		 */
		if (!ret && !phy_find_first(bus)) {
			for (i = 0; i < PHY_MAX_ADDR; i++) {
				struct phy_device *phydev;

				phydev = mdiobus_scan(bus, i);
				if (IS_ERR(phydev) &&
				    PTR_ERR(phydev) != -ENODEV) {
					ret = PTR_ERR(phydev);
					break;
				}
			}

			if (ret)
				goto err_out_unregister_bus;
		}
	} else {
		ret = of_mdiobus_register(bus, np);
	}

	if (ret)
		goto err_disable_pclk;

	return 0;

err_out_unregister_bus:
	mdiobus_unregister(bus);
err_disable_pclk:
	clk_disable_unprepare(bp->pclk);
	clk_disable_unprepare(bp->hclk);
err_out_free_mdiobus:
	mdiobus_free(bus);
	return ret;
}

static int macb_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);
	struct macb_mdio_data *bp = bus->priv;
	u32 config;

	/* Disable management port */
	config = macb_mdio_readl(bp, NCR);
	config &= ~MACB_BIT(MPE);
	macb_mdio_writel(bp, NCR, config);
	mdiobus_unregister(bus);
	clk_disable_unprepare(bp->hclk);
	clk_disable_unprepare(bp->pclk);
	mdiobus_free(bus);

	return 0;
}

static const struct of_device_id macb_mdio_dt_ids[] = {
	{ .compatible = "cdns,macb-mdio" },

};
MODULE_DEVICE_TABLE(of, macb_mdio_dt_ids);

static struct platform_driver macb_mdio_driver = {
	.probe = macb_mdio_probe,
	.remove = macb_mdio_remove,
	.driver = {
		.name = "macb-mdio",
		.of_match_table = macb_mdio_dt_ids,
	},
};

module_platform_driver(macb_mdio_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cadence MACB MDIO driver");
MODULE_AUTHOR("Xilinx");
