// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 NXP
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* Registers about DPHY control from CSI controller */
#define CSIS_N_LANES		0x04
#define N_LANES(x)		FIELD_PREP(GENMASK(2, 0), ((x) - 1))

#define CSIS_DPHY_SHUTDOWNZ	0x40
#define PHY_SHUTDOWNZ		BIT(0)

#define CSIS_DPHY_RSTZ		0x44
#define PHY_RSTZ		BIT(0)

#define CSIS_DPHY_RX		0x48
#define CSIS_DPHY_STOPSTATE	0x4C

#define CSIS_DPHY_TEST_CTRL0	0x50
#define PHY_TESTCLR		BIT(0)
#define PHY_TESTCLK		BIT(1)

#define CSIS_DPHY_TEST_CTRL1	0x54
#define PHY_TESTIN_MASK		GENMASK(7, 0)
#define PHY_TESTIN(x)		FIELD_PREP(PHY_TESTIN_MASK, (x))
#define PHY_TESTOUT_MASK	GENMASK(15, 8)
#define PHY_TESTOUT(x)		FIELD_GET(PHY_TESTOUT_MASK, (x))

struct dw_dphy;

enum dphy_reg_id {
	DPHY_RX_CFGCLKFREQRANGE = 0,
	DPHY_RX_HSFREQRANGE,

	/* iMX95 Only */
	DPHY_RX_DATA_LANE_EN,
	DPHY_RX_DATA_LANE_BASEDIR,
	DPHY_RX_DATA_LANE_FORCETXSTOPMODE,
	DPHY_RX_DATA_LANE_FORCERXMODE,
	DPHY_RX_ENABLE_CLK_EXT,
	DPHY_RX_PHY_ENABLE_BYP,
};

struct dw_dphy_reg {
	u16 offset;
	u8 mask;
	u8 shift;
};

#define PHY_REG(_offset, _width, _shift) \
	{ .offset = _offset, .mask = BIT(_width) - 1, .shift = _shift, }

struct dw_dphy_config_ops {
	void (*config)(struct dw_dphy *priv);
};

struct dw_dphy_drv_data {
	const struct dw_dphy_reg *regs;
	const struct dw_dphy_config_ops *cfg_ops;
	u32 regs_size;
	u32 max_lanes;
	u32 max_data_rate; /* Mbps */
};

struct dw_dphy {
	struct device *dev;
	struct regmap *dphy_regmap;
	struct regmap *csis_regmap;
	struct clk *cfg_clk;

	const struct dw_dphy_drv_data *drv_data;
	struct phy_configure_opts_mipi_dphy config;

	u16 hsfreqrange;
	u16 cfgclkfreqrange;
};

struct dphy_mbps_hsfreqrange_map {
	u16 mbps;
	u16 hsfreqrange;
};

/*
 * Data rate to high speed frequency range map table
 */
static const struct dphy_mbps_hsfreqrange_map hsfreqrange_table[] = {
	{ .mbps = 80,  .hsfreqrange = 0x00 },
	{ .mbps = 90,  .hsfreqrange = 0x10 },
	{ .mbps = 100, .hsfreqrange = 0x20 },
	{ .mbps = 110, .hsfreqrange = 0x30 },
	{ .mbps = 120, .hsfreqrange = 0x01 },
	{ .mbps = 130, .hsfreqrange = 0x11 },
	{ .mbps = 140, .hsfreqrange = 0x21 },
	{ .mbps = 150, .hsfreqrange = 0x31 },
	{ .mbps = 160, .hsfreqrange = 0x02 },
	{ .mbps = 170, .hsfreqrange = 0x12 },
	{ .mbps = 180, .hsfreqrange = 0x22 },
	{ .mbps = 190, .hsfreqrange = 0x32 },
	{ .mbps = 205, .hsfreqrange = 0x03 },
	{ .mbps = 220, .hsfreqrange = 0x13 },
	{ .mbps = 235, .hsfreqrange = 0x23 },
	{ .mbps = 250, .hsfreqrange = 0x33 },
	{ .mbps = 275, .hsfreqrange = 0x04 },
	{ .mbps = 300, .hsfreqrange = 0x14 },
	{ .mbps = 325, .hsfreqrange = 0x25 },
	{ .mbps = 350, .hsfreqrange = 0x35 },
	{ .mbps = 400, .hsfreqrange = 0x05 },
	{ .mbps = 450, .hsfreqrange = 0x16 },
	{ .mbps = 500, .hsfreqrange = 0x26 },
	{ .mbps = 550, .hsfreqrange = 0x37 },
	{ .mbps = 600, .hsfreqrange = 0x07 },
	{ .mbps = 650, .hsfreqrange = 0x18 },
	{ .mbps = 700, .hsfreqrange = 0x28 },
	{ .mbps = 750, .hsfreqrange = 0x39 },
	{ .mbps = 800, .hsfreqrange = 0x09 },
	{ .mbps = 850, .hsfreqrange = 0x19 },
	{ .mbps = 900, .hsfreqrange = 0x29 },
	{ .mbps = 950, .hsfreqrange = 0x3a },
	{ .mbps = 1000, .hsfreqrange = 0x0a },
	{ .mbps = 1050, .hsfreqrange = 0x1a },
	{ .mbps = 1100, .hsfreqrange = 0x2a },
	{ .mbps = 1150, .hsfreqrange = 0x3b },
	{ .mbps = 1200, .hsfreqrange = 0x0b },
	{ .mbps = 1250, .hsfreqrange = 0x1b },
	{ .mbps = 1300, .hsfreqrange = 0x2b },
	{ .mbps = 1350, .hsfreqrange = 0x3c },
	{ .mbps = 1400, .hsfreqrange = 0x0c },
	{ .mbps = 1450, .hsfreqrange = 0x1c },
	{ .mbps = 1500, .hsfreqrange = 0x2c },
	{ .mbps = 1550, .hsfreqrange = 0x3d },
	{ .mbps = 1600, .hsfreqrange = 0x0d },
	{ .mbps = 1650, .hsfreqrange = 0x1d },
	{ .mbps = 1700, .hsfreqrange = 0x2e },
	{ .mbps = 1750, .hsfreqrange = 0x3e },
	{ .mbps = 1800, .hsfreqrange = 0x0e },
	{ .mbps = 1850, .hsfreqrange = 0x1e },
	{ .mbps = 1900, .hsfreqrange = 0x1f },
	{ .mbps = 1950, .hsfreqrange = 0x3f },
	{ .mbps = 2000, .hsfreqrange = 0x0f },
	{ .mbps = 2050, .hsfreqrange = 0x40 },
	{ .mbps = 2100, .hsfreqrange = 0x41 },
	{ .mbps = 2150, .hsfreqrange = 0x42 },
	{ .mbps = 2200, .hsfreqrange = 0x43 },
	{ .mbps = 2250, .hsfreqrange = 0x44 },
	{ .mbps = 2300, .hsfreqrange = 0x45 },
	{ .mbps = 2350, .hsfreqrange = 0x46 },
	{ .mbps = 2400, .hsfreqrange = 0x47 },
	{ .mbps = 2450, .hsfreqrange = 0x48 },
	{ .mbps = 2500, .hsfreqrange = 0x49 },
	{ /* sentinel */ },
};

static inline void csis_write(struct dw_dphy *priv, unsigned int offset, u32 val)
{
	regmap_write(priv->csis_regmap, offset, val);
}

static inline int csis_read(struct dw_dphy *priv, unsigned int offset)
{
	u32 val;

	regmap_read(priv->csis_regmap, offset, &val);
	return val;
}

static int dphy_write(struct dw_dphy *priv, unsigned int index, u32 val)
{
	const struct dw_dphy_reg *reg;
	u32 mask;

	if (index >= priv->drv_data->regs_size) {
		dev_err(priv->dev, "Index out of range in %s\n", __func__);
		return -EINVAL;
	}

	reg = &priv->drv_data->regs[index];
	mask = reg->mask << reg->shift;
	val <<= reg->shift;

	return regmap_update_bits(priv->dphy_regmap, reg->offset, mask, val);
}

static void dw_dphy_dump_regs(struct dw_dphy *priv)
{
#define DW_DPHY_CSIS_DEBUG_REG(name)		{name, #name}
	static const struct {
		u32 offset;
		const char * const name;
	} csis_registers[] = {
		DW_DPHY_CSIS_DEBUG_REG(CSIS_N_LANES),
		DW_DPHY_CSIS_DEBUG_REG(CSIS_DPHY_SHUTDOWNZ),
		DW_DPHY_CSIS_DEBUG_REG(CSIS_DPHY_RSTZ),
		DW_DPHY_CSIS_DEBUG_REG(CSIS_DPHY_RX),
		DW_DPHY_CSIS_DEBUG_REG(CSIS_DPHY_TEST_CTRL0),
		DW_DPHY_CSIS_DEBUG_REG(CSIS_DPHY_TEST_CTRL1),
	};

	unsigned int i;
	u32 cfg;

	dev_dbg(priv->dev, "--- DPHY registers from CSIS ---");

	for (i = 0; i < ARRAY_SIZE(csis_registers); i++) {
		cfg = csis_read(priv, csis_registers[i].offset);
		dev_dbg(priv->dev, "%14s[0x%02x]: 0x%08x\n",
			csis_registers[i].name, csis_registers[i].offset, cfg);
	}
}

static int dw_dphy_init(struct phy *phy)
{
	struct dw_dphy *priv = phy_get_drvdata(phy);
	int ret;

	ret = phy_pm_runtime_get_sync(phy);
	if (ret < 0)
		return ret;

	return clk_prepare_enable(priv->cfg_clk);
}

static int dw_dphy_exit(struct phy *phy)
{
	struct dw_dphy *priv = phy_get_drvdata(phy);

	clk_disable_unprepare(priv->cfg_clk);
	phy_pm_runtime_put(phy);
	return 0;
}

static int dw_dphy_power_on(struct phy *phy)
{
	struct dw_dphy *priv = phy_get_drvdata(phy);
	const struct dw_dphy_drv_data *drv_data = priv->drv_data;
	struct phy_configure_opts_mipi_dphy *config = &priv->config;
	u32 val;

	/* Release Synopsys DPHY test codes from reset */
	csis_write(priv, CSIS_DPHY_RSTZ, 0x0);
	csis_write(priv, CSIS_DPHY_SHUTDOWNZ, 0x0);

	/* Set testclr=1'b1 */
	val = csis_read(priv, CSIS_DPHY_TEST_CTRL0);
	val |= PHY_TESTCLR;
	csis_write(priv, CSIS_DPHY_TEST_CTRL0, val);

	/* Wait for at least 15ns */
	ndelay(15);

	/* Config the number of active lanes */
	csis_write(priv, CSIS_N_LANES, N_LANES(config->lanes));

	drv_data->cfg_ops->config(priv);

	/* Release PHY from reset */
	csis_write(priv, CSIS_DPHY_SHUTDOWNZ, 0x1);
	csis_write(priv, CSIS_DPHY_RSTZ, 0x1);

	dw_dphy_dump_regs(priv);
	return 0;
}

static int dw_dphy_power_off(struct phy *phy)
{
	struct dw_dphy *priv = phy_get_drvdata(phy);

	csis_write(priv, CSIS_N_LANES, 0);
	csis_write(priv, CSIS_DPHY_RSTZ, 0x0);
	csis_write(priv, CSIS_DPHY_SHUTDOWNZ, 0x0);
	return 0;
}

static u8 get_hsfreqrange_by_mpbs(u64 mbps)
{
	const struct dphy_mbps_hsfreqrange_map *value;
	const struct dphy_mbps_hsfreqrange_map *prev_value = NULL;

	for (value = hsfreqrange_table; value->mbps; value++) {
		if (value->mbps >= mbps)
			break;
		prev_value = value;
	}

	if (prev_value &&
	    ((mbps - prev_value->mbps) <= (value->mbps - mbps)))
		value = prev_value;

	if (!value->mbps) {
		pr_err("Unsupported PHY speed (%llu Mbps)", mbps);
		return -ERANGE;
	}

	return value->hsfreqrange;
}

static int dw_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct dw_dphy *priv = phy_get_drvdata(phy);
	struct device *dev = priv->dev;
	const struct dw_dphy_drv_data *drv_data = priv->drv_data;
	struct phy_configure_opts_mipi_dphy *config = &opts->mipi_dphy;
	u64 data_rate_mbps;

	if (config->lanes > drv_data->max_lanes) {
		dev_err(dev, "The number of lanes has exceeded the maximum value\n");
		return -EINVAL;
	}

	data_rate_mbps = div_u64(config->hs_clk_rate, 1000 * 1000);
	if (data_rate_mbps < 80 ||
	    data_rate_mbps > drv_data->max_data_rate) {
		dev_err(dev, "Out-of-bound lane rate %llu\n", data_rate_mbps);
		return -EINVAL;
	}

	dev_dbg(dev, "Number of lanes: %d, data rate=%llu(Mbps)\n",
		config->lanes, data_rate_mbps);

	priv->hsfreqrange = get_hsfreqrange_by_mpbs(data_rate_mbps);
	priv->config = *config;

	return 0;
}

static int dw_dphy_reset(struct phy *phy)
{
	struct dw_dphy *priv = phy_get_drvdata(phy);
	u32 val;

	/* Apply PHY Reset */
	csis_write(priv, CSIS_DPHY_RSTZ, 0x0);
	csis_write(priv, CSIS_DPHY_SHUTDOWNZ, 0x0);
	ndelay(15);

	csis_write(priv, CSIS_DPHY_SHUTDOWNZ, 0x1);
	ndelay(15);
	csis_write(priv, CSIS_DPHY_RSTZ, 0x1);

	/* Set PHY_TST_CTRL0, bit[0] */
	val = csis_read(priv, CSIS_DPHY_TEST_CTRL0);
	val |= PHY_TESTCLR;
	csis_write(priv, CSIS_DPHY_TEST_CTRL0, val);

	/* Clear PHY_TST_CTRL0, bit[0] */
	val = csis_read(priv, CSIS_DPHY_TEST_CTRL0);
	val &= ~PHY_TESTCLR;
	csis_write(priv, CSIS_DPHY_TEST_CTRL0, val);

	return 0;
}

static const struct phy_ops dw_dphy_ops = {
	.init = dw_dphy_init,
	.exit = dw_dphy_exit,
	.power_on = dw_dphy_power_on,
	.power_off = dw_dphy_power_off,
	.configure = dw_dphy_configure,
	.reset = dw_dphy_reset,
	.owner = THIS_MODULE,
};

/* -----------------------------------------------------------------------------
 * i.MX93 PHY config
 **/

#define IMX93_BLK_CSI				0x48

static const struct dw_dphy_reg imx93_dphy_regs[] = {
	[DPHY_RX_CFGCLKFREQRANGE] = PHY_REG(IMX93_BLK_CSI, 6, 0),
	[DPHY_RX_HSFREQRANGE] = PHY_REG(IMX93_BLK_CSI, 7, 8),
};

static void imx93_dphy_config(struct dw_dphy *priv)
{
	/* Configure the PHY frequency range */
	dphy_write(priv, DPHY_RX_CFGCLKFREQRANGE, priv->cfgclkfreqrange);
	dphy_write(priv, DPHY_RX_HSFREQRANGE, priv->hsfreqrange);
}

static const struct dw_dphy_config_ops imx93_dphy_cfg_ops = {
	.config = imx93_dphy_config,
};

static const struct dw_dphy_drv_data imx93_dphy_drvdata = {
	.regs = imx93_dphy_regs,
	.regs_size = ARRAY_SIZE(imx93_dphy_regs),
	.cfg_ops = &imx93_dphy_cfg_ops,
	.max_lanes = 2,
	.max_data_rate = 1500,
};

/* -----------------------------------------------------------------------------
 * i.MX95 PHY config
 **/

/* STANDALONE DPHY CSR */
#define IMX95_CSR_PHY_MODE_CTRL			0x00
#define IMX95_CSR_PHY_FREQ_CTRL			0x04
#define IMX95_CSR_PHY_TEST_MODE_CTRL		0x08
#define IMX95_CSR_PHY_TEST_MODE_STS		0x0C

static const struct dw_dphy_reg imx95_dphy_regs[] = {
	[DPHY_RX_CFGCLKFREQRANGE] = PHY_REG(IMX95_CSR_PHY_FREQ_CTRL, 6, 0),
	[DPHY_RX_HSFREQRANGE] = PHY_REG(IMX95_CSR_PHY_FREQ_CTRL, 7, 16),
	[DPHY_RX_DATA_LANE_EN] = PHY_REG(IMX95_CSR_PHY_MODE_CTRL, 4, 4),
	[DPHY_RX_DATA_LANE_BASEDIR] = PHY_REG(IMX95_CSR_PHY_TEST_MODE_CTRL, 1, 0),
	[DPHY_RX_DATA_LANE_FORCETXSTOPMODE] = PHY_REG(IMX95_CSR_PHY_TEST_MODE_CTRL, 1, 4),
	[DPHY_RX_DATA_LANE_FORCERXMODE] = PHY_REG(IMX95_CSR_PHY_TEST_MODE_CTRL, 4, 11),
	[DPHY_RX_ENABLE_CLK_EXT] = PHY_REG(IMX95_CSR_PHY_TEST_MODE_CTRL, 1, 12),
	[DPHY_RX_PHY_ENABLE_BYP] = PHY_REG(IMX95_CSR_PHY_TEST_MODE_CTRL, 1, 14),
};

static void imx95_dphy_config(struct dw_dphy *priv)
{
	struct phy_configure_opts_mipi_dphy *config = &priv->config;
	u32 active_lanes = GENMASK(config->lanes - 1, 0);

	/* Configure the PHY frequency range */
	dphy_write(priv, DPHY_RX_CFGCLKFREQRANGE, priv->cfgclkfreqrange);
	dphy_write(priv, DPHY_RX_HSFREQRANGE, priv->hsfreqrange);

	dphy_write(priv, DPHY_RX_DATA_LANE_BASEDIR, 1);
	ndelay(15);

	dphy_write(priv, DPHY_RX_DATA_LANE_FORCERXMODE, active_lanes);
	ndelay(15);

	dphy_write(priv, DPHY_RX_DATA_LANE_EN, active_lanes);
	dphy_write(priv, DPHY_RX_DATA_LANE_FORCERXMODE, 0);
	dphy_write(priv, DPHY_RX_ENABLE_CLK_EXT, 1);
	dphy_write(priv, DPHY_RX_PHY_ENABLE_BYP, 1);
}

static const struct dw_dphy_config_ops imx95_dphy_cfg_ops = {
	.config = imx95_dphy_config,
};

static const struct dw_dphy_drv_data imx95_dphy_drvdata = {
	.regs = imx95_dphy_regs,
	.regs_size = ARRAY_SIZE(imx95_dphy_regs),
	.cfg_ops = &imx95_dphy_cfg_ops,
	.max_lanes = 4,
	.max_data_rate = 2500,
};

static const struct of_device_id dw_dphy_of_match[] = {
	{ .compatible = "fsl,imx93-dphy-rx", .data = &imx93_dphy_drvdata},
	{ .compatible = "fsl,imx95-dphy-rx", .data = &imx95_dphy_drvdata},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw_dphy_of_match);

static int dw_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *phy_provider;
	struct dw_dphy *priv;
	struct phy *phy;
	unsigned long cfg_rate;

	if (!dev->parent || !dev->parent->of_node)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;
	priv->drv_data = of_device_get_match_data(dev);

	priv->dphy_regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(priv->dphy_regmap)) {
		dev_err(dev, "Failed to DPHY regmap\n");
		return -ENODEV;
	}

	priv->csis_regmap = syscon_regmap_lookup_by_phandle(np, "fsl,csis");
	if (IS_ERR(priv->csis_regmap))
		return dev_err_probe(dev, PTR_ERR(priv->csis_regmap),
				     "failed to get csi controller\n");

	priv->cfg_clk = devm_clk_get(dev, "phy_cfg");
	if (IS_ERR(priv->cfg_clk)) {
		dev_err(dev, "Failed to get DPHY config clock\n");
		return PTR_ERR(priv->cfg_clk);
	}

	/* cfgclkfreqrange[5:0] = round[(cfg_clk(MHz) - 17) * 4] */
	cfg_rate = clk_get_rate(priv->cfg_clk);
	if (!cfg_rate) {
		dev_err(dev, "Failed to get PHY config clock rate\n");
		return -EINVAL;
	}
	priv->cfgclkfreqrange = (div_u64(cfg_rate, 1000 * 1000) - 17) * 4;

	pm_runtime_enable(dev);

	phy = devm_phy_create(dev, np, &dw_dphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create PHY\n");
		pm_runtime_disable(dev);
		return PTR_ERR(phy);
	}
	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static int dw_dphy_remove(struct platform_device *pdev)
{
	pm_runtime_disable(&pdev->dev);
	return 0;
}

static struct platform_driver dw_dphy_driver = {
	.probe	= dw_dphy_probe,
	.remove	= dw_dphy_remove,
	.driver = {
		.name = "dw-dphy-rx",
		.of_match_table	= dw_dphy_of_match,
	}
};
module_platform_driver(dw_dphy_driver);

MODULE_DESCRIPTION("i.MX9 Synopsys DesignWare MIPI DPHY Rx driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
