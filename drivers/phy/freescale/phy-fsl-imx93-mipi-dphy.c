// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 NXP
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/math.h>
#include <linux/mfd/syscon.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/phy/phy.h>
#include <linux/phy/phy-mipi-dphy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

/* DPHY registers */
#define DSI_REG			0x4c
#define  CFGCLKFREQRANGE_MASK	GENMASK(5, 0)
#define  CFGCLKFREQRANGE(x)	FIELD_PREP(CFGCLKFREQRANGE_MASK, (x))
#define  CLKSEL_MASK		GENMASK(7, 6)
#define  CLKSEL_STOP		FIELD_PREP(CLKSEL_MASK, 0)
#define  CLKSEL_GEN		FIELD_PREP(CLKSEL_MASK, 1)
#define  CLKSEL_EXT		FIELD_PREP(CLKSEL_MASK, 2)
#define  HSFREQRANGE_MASK	GENMASK(14, 8)
#define  HSFREQRANGE(x)		FIELD_PREP(HSFREQRANGE_MASK, (x))
#define  UPDATE_PLL		BIT(17)
#define  SHADOW_CLR		BIT(18)
#define  CLK_EXT		BIT(19)

#define DSI_WRITE_REG0		0x50
#define  M_MASK			GENMASK(9, 0)
#define  M(x)			FIELD_PREP(M_MASK, ((x) - 2))
#define  N_MASK			GENMASK(13, 10)
#define  N(x)			FIELD_PREP(N_MASK, ((x) - 1))
#define  VCO_CTRL_MASK		GENMASK(19, 14)
#define  VCO_CTRL(x)		FIELD_PREP(VCO_CTRL_MASK, (x))
#define  PROP_CTRL_MASK		GENMASK(25, 20)
#define  PROP_CTRL(x)		FIELD_PREP(PROP_CTRL_MASK, (x))
#define  INT_CTRL_MASK		GENMASK(31, 26)
#define  INT_CTRL(x)		FIELD_PREP(INT_CTRL_MASK, (x))

#define DSI_WRITE_REG1		0x54
#define  GMP_CTRL_MASK		GENMASK(1, 0)
#define  GMP_CTRL(x)		FIELD_PREP(GMP_CTRL_MASK, (x))
#define  CPBIAS_CTRL_MASK	GENMASK(8, 2)
#define  CPBIAS_CTRL(x)		FIELD_PREP(CPBIAS_CTRL_MASK, (x))
#define  PLL_SHADOW_CTRL	BIT(9)

#define MHZ(x)			((x) * 1000000UL)

#define REF_CLK_RATE_MAX	MHZ(64)
#define REF_CLK_RATE_MIN	MHZ(2)
#define FOUT_MAX		MHZ(1250)
#define FOUT_MIN		MHZ(40)
#define FVCO_DIV_FACTOR		MHZ(80)

#define MBPS(x)			((x) * 1000000UL)

#define DATA_RATE_MAX_SPEED	MBPS(2500)
#define DATA_RATE_MIN_SPEED	MBPS(80)

#define M_MAX			625UL
#define M_MIN			64UL

#define N_MAX			16U
#define N_MIN			1U

struct dw_dphy_cfg {
	u32 m;	/* PLL Feedback Multiplication Ratio */
	u32 n;	/* PLL Input Frequency Division Ratio */
};

struct dw_dphy_priv {
	struct regmap *regmap;
	struct clk *ref_clk;
	struct clk *cfg_clk;
	unsigned long ref_clk_rate;
};

struct dw_dphy_vco_prop {
	unsigned int max_fout;
	u8 vco_cntl;
	u8 prop_cntl;
};

struct dw_dphy_hsfreqrange {
	unsigned int max_mbps;
	u8 hsfreqrange;
};

/* Databook Table 3-13 Charge-pump Programmability */
static const struct dw_dphy_vco_prop vco_prop_map[] = {
	{   55, 0x3f, 0x0d },
	{   82, 0x37, 0x0d },
	{  110, 0x2f, 0x0d },
	{  165, 0x27, 0x0d },
	{  220, 0x1f, 0x0d },
	{  330, 0x17, 0x0d },
	{  440, 0x0f, 0x0d },
	{  660, 0x07, 0x0d },
	{ 1149, 0x03, 0x0d },
	{ 1152, 0x01, 0x0d },
	{ 1250, 0x01, 0x0e },
};

/* Databook Table 5-7 Frequency Ranges and Defaults */
static const struct dw_dphy_hsfreqrange hsfreqrange_map[] = {
	{   89, 0x00 },
	{   99, 0x10 },
	{  109, 0x20 },
	{  119, 0x30 },
	{  129, 0x01 },
	{  139, 0x11 },
	{  149, 0x21 },
	{  159, 0x31 },
	{  169, 0x02 },
	{  179, 0x12 },
	{  189, 0x22 },
	{  204, 0x32 },
	{  219, 0x03 },
	{  234, 0x13 },
	{  249, 0x23 },
	{  274, 0x33 },
	{  299, 0x04 },
	{  324, 0x14 },
	{  349, 0x25 },
	{  399, 0x35 },
	{  449, 0x05 },
	{  499, 0x16 },
	{  549, 0x26 },
	{  599, 0x37 },
	{  649, 0x07 },
	{  699, 0x18 },
	{  749, 0x28 },
	{  799, 0x39 },
	{  849, 0x09 },
	{  899, 0x19 },
	{  949, 0x29 },
	{  999, 0x3a },
	{ 1049, 0x0a },
	{ 1099, 0x1a },
	{ 1149, 0x2a },
	{ 1199, 0x3b },
	{ 1249, 0x0b },
	{ 1299, 0x1b },
	{ 1349, 0x2b },
	{ 1399, 0x3c },
	{ 1449, 0x0c },
	{ 1499, 0x1c },
	{ 1549, 0x2c },
	{ 1599, 0x3d },
	{ 1649, 0x0d },
	{ 1699, 0x1d },
	{ 1749, 0x2e },
	{ 1799, 0x3e },
	{ 1849, 0x0e },
	{ 1899, 0x1e },
	{ 1949, 0x2f },
	{ 1999, 0x3f },
	{ 2049, 0x0f },
	{ 2099, 0x40 },
	{ 2149, 0x41 },
	{ 2199, 0x42 },
	{ 2249, 0x43 },
	{ 2299, 0x44 },
	{ 2349, 0x45 },
	{ 2399, 0x46 },
	{ 2449, 0x47 },
	{ 2499, 0x48 },
	{ 2500, 0x49 },
};

static int phy_write(struct phy *phy, u32 value, unsigned int reg)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = regmap_write(priv->regmap, reg, value);
	if (ret < 0)
		dev_err(&phy->dev, "failed to write reg %u: %d\n", reg, ret);
	return ret;
}

static inline unsigned long data_rate_to_fout(unsigned long data_rate)
{
	/* Fout is half of data rate */
	return data_rate / 2;
}

static int
dw_dphy_config_from_opts(struct phy *phy,
			 struct phy_configure_opts_mipi_dphy *dphy_opts,
			 struct dw_dphy_cfg *cfg)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);
	unsigned long fin = priv->ref_clk_rate;
	unsigned long fout;
	unsigned long best_fout = 0;
	unsigned int fvco_div;
	unsigned int min_n, max_n, n, best_n;
	unsigned long m, best_m;
	unsigned long min_delta = ULONG_MAX;
	unsigned long tmp, delta;

	if (dphy_opts->hs_clk_rate < DATA_RATE_MIN_SPEED ||
	    dphy_opts->hs_clk_rate > DATA_RATE_MAX_SPEED) {
		dev_dbg(&phy->dev, "invalid data rate per lane: %lu\n",
			dphy_opts->hs_clk_rate);
		return -EINVAL;
	}

	fout = data_rate_to_fout(dphy_opts->hs_clk_rate);

	/* Fout = Fvco / Fvco_div = (Fin * M) / (Fvco_div * N) */
	fvco_div = 8UL / min(DIV_ROUND_UP(fout, FVCO_DIV_FACTOR), 8UL);

	/* limitation: 2MHz <= Fin / N <= 8MHz */
	min_n = DIV_ROUND_UP(fin, MHZ(8));
	max_n = DIV_ROUND_DOWN_ULL(fin, MHZ(2));

	/* clamp possible N(s) */
	min_n = clamp(min_n, N_MIN, N_MAX);
	max_n = clamp(max_n, N_MIN, N_MAX);

	dev_dbg(&phy->dev, "Fout = %lu, Fvco_div = %u, n_range = [%u, %u]\n",
		fout, fvco_div, min_n, max_n);

	for (n = min_n; n <= max_n; n++) {
		/* M = (Fout * N * Fvco_div) / Fin */
		tmp = fout * n * fvco_div;
		m = DIV_ROUND_CLOSEST(tmp, fin);

		/* check M range */
		if (m < M_MIN || m > M_MAX)
			continue;

		/* calculate temporary Fout */
		tmp = m * fin;
		do_div(tmp, n * fvco_div);
		if (tmp < FOUT_MIN || tmp > FOUT_MAX)
			continue;

		delta = abs(fout - tmp);
		if (delta < min_delta) {
			best_n = n;
			best_m = m;
			min_delta = delta;
			best_fout = tmp;
		}
	}

	if (best_fout) {
		cfg->m = best_m;
		cfg->n = best_n;
		dphy_opts->hs_clk_rate = best_fout * 2;
		dev_dbg(&phy->dev, "best Fout = %lu, m = %u, n = %u\n",
			best_fout, cfg->m, cfg->n);
	} else {
		dev_dbg(&phy->dev, "failed to find best Fout\n");
		return -EINVAL;
	}

	return 0;
}

static void dw_dphy_clear_shadow(struct phy *phy)
{
	/* Select clock generation first. */
	phy_write(phy, CLKSEL_GEN, DSI_REG);

	/* Clear shadow after clock selection is done a while. */
	usleep_range(1, 2);
	phy_write(phy, CLKSEL_GEN | SHADOW_CLR, DSI_REG);

	/*
	 * A minimum pulse of 5ns on shadow_clear signal,
	 * according to Databook Figure 3-3 Initialization Timing Diagram.
	 */
	usleep_range(1, 2);
	phy_write(phy, CLKSEL_GEN, DSI_REG);
}

static u32 dw_dphy_get_cfgclkrange(struct phy *phy)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);

	return (clk_get_rate(priv->cfg_clk) / MHZ(1) - 17) * 4;
}

static u8
dw_dphy_get_hsfreqrange(struct phy_configure_opts_mipi_dphy *dphy_opts)
{
	unsigned int mbps = dphy_opts->hs_clk_rate / MHZ(1);
	int i;

	for (i = 0; i < ARRAY_SIZE(hsfreqrange_map); i++)
		if (mbps <= hsfreqrange_map[i].max_mbps)
			return hsfreqrange_map[i].hsfreqrange;

	return 0;
}

static u8 dw_dphy_get_vco(struct phy_configure_opts_mipi_dphy *dphy_opts)
{
	unsigned int fout = data_rate_to_fout(dphy_opts->hs_clk_rate) / MHZ(1);
	int i;

	for (i = 0; i < ARRAY_SIZE(vco_prop_map); i++)
		if (fout <= vco_prop_map[i].max_fout)
			return vco_prop_map[i].vco_cntl;

	return 0;
}

static u8 dw_dphy_get_prop(struct phy_configure_opts_mipi_dphy *dphy_opts)
{
	unsigned int fout = data_rate_to_fout(dphy_opts->hs_clk_rate) / MHZ(1);
	int i;

	for (i = 0; i < ARRAY_SIZE(vco_prop_map); i++)
		if (fout <= vco_prop_map[i].max_fout)
			return vco_prop_map[i].prop_cntl;

	return 0;
}

static int dw_dphy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct dw_dphy_cfg cfg = { 0 };
	u32 val;
	int ret;

	ret = dw_dphy_config_from_opts(phy, &opts->mipi_dphy, &cfg);
	if (ret)
		return ret;

	dw_dphy_clear_shadow(phy);

	/* reg */
	val = CLKSEL_GEN |
	      CFGCLKFREQRANGE(dw_dphy_get_cfgclkrange(phy)) |
	      HSFREQRANGE(dw_dphy_get_hsfreqrange(&opts->mipi_dphy));
	phy_write(phy, val, DSI_REG);

	/* w_reg0 */
	val = M(cfg.m) | N(cfg.n) | INT_CTRL(0) |
	      VCO_CTRL(dw_dphy_get_vco(&opts->mipi_dphy)) |
	      PROP_CTRL(dw_dphy_get_prop(&opts->mipi_dphy));
	phy_write(phy, val, DSI_WRITE_REG0);

	/* w_reg1 */
	phy_write(phy, GMP_CTRL(1) | CPBIAS_CTRL(0x10), DSI_WRITE_REG1);

	return 0;
}

static int dw_dphy_validate(struct phy *phy, enum phy_mode mode, int submode,
			    union phy_configure_opts *opts)
{
	struct dw_dphy_cfg cfg = { 0 };

	if (mode != PHY_MODE_MIPI_DPHY)
		return -EINVAL;

	return dw_dphy_config_from_opts(phy, &opts->mipi_dphy, &cfg);
}

static void dw_dphy_clear_reg(struct phy *phy)
{
	phy_write(phy, 0, DSI_REG);
	phy_write(phy, 0, DSI_WRITE_REG0);
	phy_write(phy, 0, DSI_WRITE_REG1);
}

static int dw_dphy_init(struct phy *phy)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = pm_runtime_get_sync(&phy->dev);
	if (ret < 0) {
		dev_err(&phy->dev, "failed to get PM runtime: %d\n", ret);
		return ret;
	}

	ret = clk_prepare_enable(priv->cfg_clk);
	if (ret < 0) {
		pm_runtime_put(&phy->dev);
		dev_err(&phy->dev, "failed to enable config clock: %d\n", ret);
		return ret;
	}

	dw_dphy_clear_reg(phy);

	return 0;
}

static int dw_dphy_exit(struct phy *phy)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);

	dw_dphy_clear_reg(phy);
	clk_disable_unprepare(priv->cfg_clk);
	pm_runtime_put(&phy->dev);

	return 0;
}

static int dw_dphy_update_pll(struct phy *phy)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);
	int ret;

	ret = regmap_update_bits(priv->regmap, DSI_REG, UPDATE_PLL, UPDATE_PLL);
	if (ret < 0) {
		dev_err(&phy->dev, "failed to set UPDATE_PLL: %d\n", ret);
		return ret;
	}

	/*
	 * The updatepll signal should be asserted for a minimum of four clkin
	 * cycles, according to Databook Figure 3-3 Initialization Timing
	 * Diagram.
	 */
	usleep_range(3, 10);

	ret = regmap_update_bits(priv->regmap, DSI_REG, UPDATE_PLL, 0);
	if (ret < 0) {
		dev_err(&phy->dev, "failed to clear UPDATE_PLL: %d\n", ret);
		return ret;
	}

	return 0;
}

static int dw_dphy_power_on(struct phy *phy)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);
	struct device *dev = &phy->dev;
	int ret;

	ret = clk_prepare_enable(priv->ref_clk);
	if (ret < 0) {
		dev_err(dev, "failed to enable ref clock: %d\n", ret);
		return ret;
	}

	/*
	 * At least 10 refclk cycles are required before updatePLL assertion,
	 * according to Databook Figure 3-3 Initialization Timing Diagram.
	 */
	usleep_range(5, 10);

	ret = dw_dphy_update_pll(phy);
	if (ret < 0) {
		clk_disable_unprepare(priv->ref_clk);
		return ret;
	}

	return 0;
}

static int dw_dphy_power_off(struct phy *phy)
{
	struct dw_dphy_priv *priv = phy_get_drvdata(phy);

	dw_dphy_clear_reg(phy);
	clk_disable_unprepare(priv->ref_clk);

	return 0;
}

static const struct phy_ops dw_dphy_phy_ops = {
	.init = dw_dphy_init,
	.exit = dw_dphy_exit,
	.power_on = dw_dphy_power_on,
	.power_off = dw_dphy_power_off,
	.configure = dw_dphy_configure,
	.validate = dw_dphy_validate,
	.owner = THIS_MODULE,
};

static const struct of_device_id dw_dphy_of_match[] = {
	{ .compatible = "fsl,imx93-mipi-dphy" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, dw_dphy_of_match);

static int dw_dphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct phy_provider *phy_provider;
	struct dw_dphy_priv *priv;
	struct phy *phy;
	int ret;

	if (!np)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = syscon_node_to_regmap(np->parent);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err_probe(dev, ret, "failed to get regmap\n");
		return ret;
	}

	priv->cfg_clk = devm_clk_get(dev, "phy_cfg");
	if (IS_ERR(priv->cfg_clk)) {
		ret = PTR_ERR(priv->cfg_clk);
		dev_err_probe(dev, ret, "failed to get config clock\n");
		return ret;
	}

	priv->ref_clk = devm_clk_get(dev, "phy_ref");
	if (IS_ERR(priv->ref_clk)) {
		ret = PTR_ERR(priv->ref_clk);
		dev_err_probe(dev, ret, "failed to get ref clock\n");
		return ret;
	}

	priv->ref_clk_rate = clk_get_rate(priv->ref_clk);
	if (priv->ref_clk_rate < REF_CLK_RATE_MIN ||
	    priv->ref_clk_rate > REF_CLK_RATE_MAX) {
		dev_err(dev, "invalid ref clock rate %lu\n",
			priv->ref_clk_rate);
		return -EINVAL;
	}
	dev_dbg(dev, "ref clock rate: %lu\n", priv->ref_clk_rate);

	dev_set_drvdata(dev, priv);

	pm_runtime_enable(dev);

	phy = devm_phy_create(dev, np, &dw_dphy_phy_ops);
	if (IS_ERR(phy)) {
		ret = PTR_ERR(phy);
		dev_err(dev, "failed to create PHY %ld\n", PTR_ERR(phy));
		goto err;
	}
	phy_set_drvdata(phy, priv);

	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(phy_provider)) {
		ret = PTR_ERR(phy_provider);
		dev_err(dev, "failed to register PHY provider: %d\n", ret);
		goto err;
	}

	return 0;
err:
	pm_runtime_disable(dev);
	return ret;
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
		.name = "dw-mipi-dphy",
		.of_match_table	= dw_dphy_of_match,
	}
};
module_platform_driver(dw_dphy_driver);

MODULE_DESCRIPTION("Freescale i.MX93 Synopsys DesignWare MIPI DPHY driver");
MODULE_AUTHOR("NXP Semiconductor");
MODULE_LICENSE("GPL");
