/*
 * Copyright 2018 NXP
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/phy/phy-mixel-mipi-dsi.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <soc/imx8/sc/sci.h>

#define DPHY_PD_DPHY			0x00
#define DPHY_M_PRG_HS_PREPARE		0x04
#define DPHY_MC_PRG_HS_PREPARE		0x08
#define DPHY_M_PRG_HS_ZERO		0x0c
#define DPHY_MC_PRG_HS_ZERO		0x10
#define DPHY_M_PRG_HS_TRAIL		0x14
#define DPHY_MC_PRG_HS_TRAIL		0x18
#define DPHY_PD_PLL			0x1c
#define DPHY_TST			0x20
#define DPHY_CN				0x24
#define DPHY_CM				0x28
#define DPHY_CO				0x2c
#define DPHY_LOCK			0x30
#define DPHY_LOCK_BYP			0x34

#define MBPS(x) ((x) * 1000000)

#define DATA_RATE_MAX_SPEED MBPS(1500)
#define DATA_RATE_MIN_SPEED MBPS(80)

#define CN_BUF	0xcb7a89c0
#define CO_BUF	0x63
#define CM(x)	( \
		((x) <  32)?0xe0|((x)-16) : \
		((x) <  64)?0xc0|((x)-32) : \
		((x) < 128)?0x80|((x)-64) : \
		((x) - 128))
#define CN(x)	(((x) == 1)?0x1f : (((CN_BUF)>>((x)-1))&0x1f))
#define CO(x)	((CO_BUF)>>(8-(x))&0x3)

/* PHY power on is LOW_ENABLE */
#define PWR_ON	0
#define PWR_OFF	1

struct pll_divider {
	u32 cm;
	u32 cn;
	u32 co;
};

struct devtype {
	bool have_sc;
	u8 reg_tx_rcal;
	u8 reg_auto_pd_en;
	u8 reg_rxlprp;
	u8 reg_rxcdrp;
	u8 reg_rxhs_settle;
	u8 reg_bypass_pll;
};

struct mixel_mipi_phy_priv {
	struct device	*dev;
	void __iomem	*base;
	const struct devtype	*plat_data;
	sc_rsrc_t	mipi_id;
	struct pll_divider divider;
	struct mutex	lock;
	unsigned long	data_rate;
};


static inline u32 phy_read(struct phy *phy, unsigned int reg)
{
	struct mixel_mipi_phy_priv *priv = phy_get_drvdata(phy);

	return readl(priv->base + reg);
}

static inline void phy_write(struct phy *phy, u32 value, unsigned int reg)
{
	struct mixel_mipi_phy_priv *priv = phy_get_drvdata(phy);

	writel(value, priv->base + reg);
}

/*
 * mixel_phy_mipi_set_phy_speed:
 * Input params:
 *	bit_clk: PHY PLL needed output clock
 *	ref_clk: reference input clock for the PHY PLL
 *
 * Returns:
 *	0: if the bit_clk can be achieved for the given ref_clk
 *	-EINVAL: otherwise
 */
int mixel_phy_mipi_set_phy_speed(struct phy *phy,
				 unsigned long bit_clk,
				 unsigned long ref_clk,
				 bool best_match)
{
	struct mixel_mipi_phy_priv *priv = dev_get_drvdata(phy->dev.parent);
	u32 div_rate;
	u32 numerator = 0;
	u32 denominator = 1;

	if (bit_clk > DATA_RATE_MAX_SPEED || bit_clk < DATA_RATE_MIN_SPEED)
		return -EINVAL;

	/* simulated fixed point with 3 decimals */
	div_rate = (bit_clk * 1000) / ref_clk;

	while (denominator <= 256) {
		if (div_rate % 1000 == 0)
			numerator = div_rate / 1000;
		if (numerator > 15)
			break;
		denominator = denominator << 1;
		div_rate = div_rate << 1;
	}

	/* CM ranges between 16 and 255 */
	/* CN ranges between 1 and 32 */
	/* CO is power of 2: 1, 2, 4, 8 */
	if (best_match && numerator < 16)
		numerator = div_rate / 1000;

	if (best_match && numerator > 255) {
		while (numerator > 255 && denominator > 1) {
			numerator = DIV_ROUND_UP(numerator, 2);
			denominator = denominator >> 1;
		}
	}

	if (numerator < 16 || numerator > 255)
		return -EINVAL;

	if (best_match)
		numerator = DIV_ROUND_UP(numerator, denominator) * denominator;

	priv->divider.cn = 1;
	if (denominator > 8) {
		priv->divider.cn = denominator >> 3;
		denominator = 8;
	}
	priv->divider.co = denominator;
	priv->divider.cm = numerator;

	priv->data_rate = bit_clk;

	return 0;
}
EXPORT_SYMBOL_GPL(mixel_phy_mipi_set_phy_speed);

static int mixel_mipi_phy_enable(struct phy *phy, u32 reset)
{
	struct mixel_mipi_phy_priv *priv = phy_get_drvdata(phy);
	sc_err_t sci_err = 0;
	sc_ipc_t ipc_handle = 0;
	u32 mu_id;

	sci_err = sc_ipc_getMuID(&mu_id);
	if (sci_err != SC_ERR_NONE) {
		dev_err(&phy->dev, "Failed to get MU ID (%d)\n", sci_err);
		return -ENODEV;
	}
	sci_err = sc_ipc_open(&ipc_handle, mu_id);
	if (sci_err != SC_ERR_NONE) {
		dev_err(&phy->dev, "Failed to open IPC (%d)\n", sci_err);
		return -ENODEV;
	}

	sci_err = sc_misc_set_control(ipc_handle,
				      priv->mipi_id,
				      SC_C_PHY_RESET,
				      reset);
	if (sci_err != SC_ERR_NONE) {
		dev_err(&phy->dev, "Failed to reset DPHY (%d)\n", sci_err);
		sc_ipc_close(ipc_handle);
		return -ENODEV;
	}

	sc_ipc_close(ipc_handle);

	return 0;
}

/*
 * We tried our best here to use the values as specified in
 * Reference Manual, but we got unstable results. So, these values
 * are hacked from their original explanation as found in RM.
 */
static void mixel_phy_set_prg_regs(struct phy *phy)
{
	struct mixel_mipi_phy_priv *priv = phy_get_drvdata(phy);
	unsigned int hs_reg;

	/* MC_PRG_HS_PREPARE = 1.0 * Ttxescape if DPHY_MC_PRG_HS_PREPARE = 0
	 *
	 * MC_PRG_HS_PREPARE = 1.5 * Ttxescape if DPHY_MC_PRG_HS_PREPARE = 1
	 *
	 * Assume Ftxescape is 18-20 MHz with DPHY_MC_PRG_HS_PREPARE = 0,
	 * this gives 55-50 ns.
	 * The specification is 38 to 95 ns.
	 */
	phy_write(phy, 0x00, DPHY_MC_PRG_HS_PREPARE);

	/* PRG_HS_PREPARE
	 * for  PRG_HS_PREPARE = 00, THS-PREPARE = 1   * TxClkEsc Period
	 *      PRG_HS_PREPARE = 01, THS-PREPARE = 1.5 * TxClkEsc Period
	 *      PRG_HS_PREPARE = 10, THS-PREPARE = 2   * TxClkEsc Period
	 *      PRG_HS_PREPARE = 11, THS-PREPARE = 2.5 * TxClkEsc Period
	 *
	 *      The specification for THS-PREPARE is
	 *	     Min (40ns + 4*UI)
	 *           Max 85ns +6*UI
	 */
	if (priv->data_rate <= MBPS(61))
		phy_write(phy, 0x03, DPHY_M_PRG_HS_PREPARE);
	else if (priv->data_rate <= MBPS(90))
		phy_write(phy, 0x02, DPHY_M_PRG_HS_PREPARE);
	else if (priv->data_rate <= MBPS(500))
		phy_write(phy, 0x01, DPHY_M_PRG_HS_PREPARE);
	else
		phy_write(phy, 0x00, DPHY_M_PRG_HS_PREPARE);

	/* MC_PRG_HS_ZERO
	 *
	 *  T-CLK-ZERO = ( MC_PRG_HS_ZERO + 3) * (TxByteClkHS Period)
	 *
	 *  The minimum specification for THS-PREPARE is 262 ns.
	 *
	 */
	hs_reg =
		/* simplified equation y = .034x - 2.5
		 *
		 * This a linear interpolation of the values from the
		 * PHY user guide
		 */
		(34 * (priv->data_rate/1000000) - 2500) / 1000;

	if (hs_reg < 1)
		hs_reg = 1;
	phy_write(phy, hs_reg, DPHY_MC_PRG_HS_ZERO);

	/* M_PRG_HS_ZERO
	 *
	 *  TT-HS-ZERO =(M_PRG_HS_ZERO + 6) * (TxByteClkHS Period)
	 *
	 *  The minimum specification for THS-ZERO 105ns + 6*UI.
	 *
	 */
	hs_reg =
		/* simplified equation y = .0144x - 4.75
		 *
		 * This a linear interpolation of the values from the
		 * PHY user guide
		 */

		(144 * (priv->data_rate/1000000) - 47500) / 10000;

	if (hs_reg < 1)
		hs_reg = 1;
	phy_write(phy, hs_reg, DPHY_M_PRG_HS_ZERO);

	/* MC_PRG_HS_TRAIL and M_PRG_HS_TRAIL
	 *
	 *  THS-TRAIL =(PRG_HS_TRAIL) * (TxByteClkHS Period)
	 *
	 *  The specification for THS-TRAIL is
	 *	     Min     (60ns   + 4*UI)
	 *           Typical (82.5ns + 8*UI)
	 *           Max     (105ns  + 12*UI)
	 *
	 */

	hs_reg =
		/* simplified equation y = .0103x + 1
		 *
		 * This a linear interpolation of the values from the
		 * PHY user guide
		 */
		(103 * (priv->data_rate/1000000) + 10000) / 10000;

	if (hs_reg > 15)
		hs_reg = 15;
	if (hs_reg < 1)
		hs_reg = 1;

	phy_write(phy, hs_reg, DPHY_MC_PRG_HS_TRAIL);
	phy_write(phy, hs_reg, DPHY_M_PRG_HS_TRAIL);

	/* M_PRG_RXHS_SETTLE */
	if (priv->plat_data->reg_rxhs_settle == 0xFF)
		return;
	if (priv->data_rate < MBPS(80))
		phy_write(phy, 0x0d, priv->plat_data->reg_rxhs_settle);
	else if (priv->data_rate < MBPS(90))
		phy_write(phy, 0x0c, priv->plat_data->reg_rxhs_settle);
	else if (priv->data_rate < MBPS(125))
		phy_write(phy, 0x0b, priv->plat_data->reg_rxhs_settle);
	else if (priv->data_rate < MBPS(150))
		phy_write(phy, 0x0a, priv->plat_data->reg_rxhs_settle);
	else if (priv->data_rate < MBPS(225))
		phy_write(phy, 0x09, priv->plat_data->reg_rxhs_settle);
	else if (priv->data_rate < MBPS(500))
		phy_write(phy, 0x08, priv->plat_data->reg_rxhs_settle);
	else
		phy_write(phy, 0x07, priv->plat_data->reg_rxhs_settle);

}

static int mixel_mipi_phy_init(struct phy *phy)
{
	struct mixel_mipi_phy_priv *priv = dev_get_drvdata(phy->dev.parent);

	mutex_lock(&priv->lock);

	phy_write(phy, PWR_OFF, DPHY_PD_PLL);
	phy_write(phy, PWR_OFF, DPHY_PD_DPHY);

	mixel_phy_set_prg_regs(phy);

	phy_write(phy, 0x00, DPHY_LOCK_BYP);
	if (priv->plat_data->reg_tx_rcal != 0xFF)
		phy_write(phy, 0x01, priv->plat_data->reg_tx_rcal);
	if (priv->plat_data->reg_auto_pd_en != 0xFF)
		phy_write(phy, 0x00, priv->plat_data->reg_auto_pd_en);
	if (priv->plat_data->reg_rxlprp != 0xFF)
		phy_write(phy, 0x02, priv->plat_data->reg_rxlprp);
	if (priv->plat_data->reg_rxcdrp != 0xFF)
		phy_write(phy, 0x02, priv->plat_data->reg_rxcdrp);
	phy_write(phy, 0x25, DPHY_TST);

	/* VCO = REF_CLK * CM / CN * CO */
	if (priv->divider.cm < 16 || priv->divider.cm > 255 ||
		priv->divider.cn < 1 || priv->divider.cn > 32 ||
		priv->divider.co < 1 || priv->divider.co > 8) {
		dev_err(&phy->dev, "Invalid CM/CN/CO values! (%u/%u/%u)\n",
			priv->divider.cm, priv->divider.cn, priv->divider.co);
		mutex_unlock(&priv->lock);
		return -EINVAL;
	}
	dev_dbg(&phy->dev, "Using CM:%u CN:%u CO:%u\n",
		 priv->divider.cm, priv->divider.cn, priv->divider.co);
	phy_write(phy, CM(priv->divider.cm), DPHY_CM);
	phy_write(phy, CN(priv->divider.cn), DPHY_CN);
	phy_write(phy, CO(priv->divider.co), DPHY_CO);

	mutex_unlock(&priv->lock);

	return 0;
}

static int mixel_mipi_phy_exit(struct phy *phy)
{
	phy_write(phy, 0, DPHY_CM);
	phy_write(phy, 0, DPHY_CN);
	phy_write(phy, 0, DPHY_CO);

	return 0;
}

static int mixel_mipi_phy_power_on(struct phy *phy)
{
	struct mixel_mipi_phy_priv *priv = phy_get_drvdata(phy);
	u32 lock, timeout;
	int ret = 0;

	mutex_lock(&priv->lock);

	phy_write(phy, PWR_ON, DPHY_PD_PLL);

	timeout = 100;
	while (!(lock = phy_read(phy, DPHY_LOCK))) {
		udelay(10);
		if (--timeout == 0) {
			dev_err(&phy->dev, "Could not get DPHY lock!\n");
			phy_write(phy, PWR_OFF, DPHY_PD_PLL);
			mutex_unlock(&priv->lock);
			return -EINVAL;
		}
	}
	dev_dbg(&phy->dev, "DPHY lock acquired after %d tries\n",
		(100 - timeout));

	phy_write(phy, PWR_ON, DPHY_PD_DPHY);

	if (priv->plat_data->have_sc)
		ret = mixel_mipi_phy_enable(phy, 1);

	mutex_unlock(&priv->lock);

	return ret;
}

static int mixel_mipi_phy_power_off(struct phy *phy)
{
	struct mixel_mipi_phy_priv *priv = phy_get_drvdata(phy);
	int ret = 0;

	mutex_lock(&priv->lock);

	phy_write(phy, PWR_OFF, DPHY_PD_PLL);
	phy_write(phy, PWR_OFF, DPHY_PD_DPHY);

	if (priv->plat_data->have_sc)
		ret = mixel_mipi_phy_enable(phy, 0);

	mutex_unlock(&priv->lock);

	return ret;
}

static const struct phy_ops mixel_mipi_phy_ops = {
	.init = mixel_mipi_phy_init,
	.exit = mixel_mipi_phy_exit,
	.power_on = mixel_mipi_phy_power_on,
	.power_off = mixel_mipi_phy_power_off,
	.owner = THIS_MODULE,
};

static struct devtype imx8qm_dev = {
	.have_sc = true,
	.reg_tx_rcal = 0xFF,
	.reg_auto_pd_en = 0x38,
	.reg_rxlprp = 0x3c,
	.reg_rxcdrp = 0x40,
	.reg_rxhs_settle = 0x44,
	.reg_bypass_pll = 0xFF,
};
static struct devtype imx8qxp_dev = {
	.have_sc = true,
	.reg_tx_rcal = 0xFF,
	.reg_auto_pd_en = 0x38,
	.reg_rxlprp = 0x3c,
	.reg_rxcdrp = 0x40,
	.reg_rxhs_settle = 0x44,
	.reg_bypass_pll = 0xFF,
};
static struct devtype imx8mq_dev = {
	.have_sc = false,
	.reg_tx_rcal = 0x38,
	.reg_auto_pd_en = 0x3c,
	.reg_rxlprp = 0x40,
	.reg_rxcdrp = 0x44,
	.reg_rxhs_settle = 0x48,
	.reg_bypass_pll = 0x4c,
};

static const struct of_device_id mixel_mipi_phy_of_match[] = {
	{ .compatible = "mixel,imx8qm-mipi-dsi-phy", .data = &imx8qm_dev },
	{ .compatible = "mixel,imx8qxp-mipi-dsi-phy", .data = &imx8qxp_dev },
	{ .compatible = "mixel,imx8mq-mipi-dsi-phy", .data = &imx8mq_dev },
	{}
};
MODULE_DEVICE_TABLE(of, mixel_mipi_phy_of_match);

static int mixel_mipi_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	const struct of_device_id *of_id =
		of_match_device(mixel_mipi_phy_of_match, dev);
	struct phy_provider *phy_provider;
	struct mixel_mipi_phy_priv *priv;
	struct resource *res;
	struct phy *phy;
	int phy_id = 0;

	if (!np)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res)
		return -ENODEV;

	priv->base = devm_ioremap(dev, res->start, SZ_256);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	priv->plat_data = of_id->data;

	phy_id = of_alias_get_id(np, "dsi_phy");
	if (phy_id < 0) {
		dev_err(dev, "No dsi_phy alias found!");
		return phy_id;
	}

	priv->mipi_id = phy_id?SC_R_MIPI_1:SC_R_MIPI_0;

	priv->dev = dev;

	mutex_init(&priv->lock);
	dev_set_drvdata(dev, priv);

	phy = devm_phy_create(dev, np, &mixel_mipi_phy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create phy\n");
		return PTR_ERR(phy);
	}
	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static struct platform_driver mixel_mipi_phy_driver = {
	.probe	= mixel_mipi_phy_probe,
	.driver = {
		.name = "mixel-mipi-dsi-phy",
		.of_match_table	= mixel_mipi_phy_of_match,
	}
};
module_platform_driver(mixel_mipi_phy_driver);

MODULE_AUTHOR("NXP Semiconductor");
MODULE_DESCRIPTION("Mixel MIPI-DSI PHY driver");
MODULE_LICENSE("GPL v2");
