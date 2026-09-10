// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek DisplayPort PHY driver
 *
 * Copyright (c) 2022, BayLibre Inc.
 * Copyright (c) 2022, MediaTek Inc.
 *
 * Major refactoring
 * Copyright (c) 2026, Collabora Ltd.
 *                     AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>
 */

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>

#define MTK_DP_PHY_MAX_LANES		4

/* DP_PHYA_GLB_FORCE_CTRL_1 */
#define CKM_CKTX0_EN_FORCE_MODE		BIT(10)

/* DP_PHYD_PLL_CTL_1 */
#define TPLL_SSC_EN			BIT(3)

/* DP_PHYD_BIT_RATE */
#define PHYD_DIG_RG_BIT_RATE		GENMASK(1, 0)
#  define BIT_RATE_RBR			0
#  define BIT_RATE_HBR			1
#  define BIT_RATE_HBR2			2
#  define BIT_RATE_HBR3			3

/* DP_PHYD_SW_RST */
#define PHYD_DIG_GLB_SW_RST_B		GENMASK(7, 0)
#  define DP_GLB_SW_RST_PHYD		BIT(0)
#  define DP_GLB_SW_RST_TFIFO_ANA	BIT(1)
#  define DP_GLB_SW_RST_XTAL_CLK	BIT(2)
#  define DP_GLB_SW_RST_MAIN_LINK	BIT(3)

/* DP_PHYD_AUX_RX_CTL */
#define PHYD_DIG_DPAUX_RX_EN		BIT(0)
#define PHYD_DIG_XTP_GLB_CKDET_EN	BIT(1)
#define PHYD_DIG_DPAUX_RX_DEGLITCH_EN	BIT(2)

#define DRIVING_PARAM_0_DEFAULT	0x0
#define DRIVING_PARAM_1_DEFAULT	0x0
#define DRIVING_PARAM_2_DEFAULT	0x0

/* DP_PHYD_TX_CTL_0 */
#define PHYD_TX_LN_EN			GENMASK(7, 4)

#define XTP_LN_TX_LCTXC0_SW0_PRE0_DEFAULT	BIT(4)
#define XTP_LN_TX_LCTXC0_SW0_PRE1_DEFAULT	(BIT(10) | BIT(12))
#define XTP_LN_TX_LCTXC0_SW0_PRE2_DEFAULT	GENMASK(20, 19)
#define XTP_LN_TX_LCTXC0_SW0_PRE3_DEFAULT	GENMASK(29, 29)
#define DRIVING_PARAM_3_DEFAULT	(XTP_LN_TX_LCTXC0_SW0_PRE0_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW0_PRE1_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW0_PRE2_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW0_PRE3_DEFAULT)

#define XTP_LN_TX_LCTXC0_SW1_PRE0_DEFAULT	GENMASK(4, 3)
#define XTP_LN_TX_LCTXC0_SW1_PRE1_DEFAULT	GENMASK(12, 9)
#define XTP_LN_TX_LCTXC0_SW1_PRE2_DEFAULT	(BIT(18) | BIT(21))
#define XTP_LN_TX_LCTXC0_SW2_PRE0_DEFAULT	GENMASK(29, 29)
#define DRIVING_PARAM_4_DEFAULT	(XTP_LN_TX_LCTXC0_SW1_PRE0_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW1_PRE1_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW1_PRE2_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW2_PRE0_DEFAULT)

#define XTP_LN_TX_LCTXC0_SW2_PRE1_DEFAULT	(BIT(3) | BIT(5))
#define XTP_LN_TX_LCTXC0_SW3_PRE0_DEFAULT	GENMASK(13, 12)
#define DRIVING_PARAM_5_DEFAULT	(XTP_LN_TX_LCTXC0_SW2_PRE1_DEFAULT | \
				 XTP_LN_TX_LCTXC0_SW3_PRE0_DEFAULT)

#define XTP_LN_TX_LCTXCP1_SW0_PRE0_DEFAULT	0
#define XTP_LN_TX_LCTXCP1_SW0_PRE1_DEFAULT	GENMASK(10, 10)
#define XTP_LN_TX_LCTXCP1_SW0_PRE2_DEFAULT	GENMASK(19, 19)
#define XTP_LN_TX_LCTXCP1_SW0_PRE3_DEFAULT	GENMASK(28, 28)
#define DRIVING_PARAM_6_DEFAULT	(XTP_LN_TX_LCTXCP1_SW0_PRE0_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW0_PRE1_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW0_PRE2_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW0_PRE3_DEFAULT)

#define XTP_LN_TX_LCTXCP1_SW1_PRE0_DEFAULT	0
#define XTP_LN_TX_LCTXCP1_SW1_PRE1_DEFAULT	GENMASK(10, 9)
#define XTP_LN_TX_LCTXCP1_SW1_PRE2_DEFAULT	GENMASK(19, 18)
#define XTP_LN_TX_LCTXCP1_SW2_PRE0_DEFAULT	0
#define DRIVING_PARAM_7_DEFAULT	(XTP_LN_TX_LCTXCP1_SW1_PRE0_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW1_PRE1_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW1_PRE2_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW2_PRE0_DEFAULT)

#define XTP_LN_TX_LCTXCP1_SW2_PRE1_DEFAULT	GENMASK(3, 3)
#define XTP_LN_TX_LCTXCP1_SW3_PRE0_DEFAULT	0
#define DRIVING_PARAM_8_DEFAULT	(XTP_LN_TX_LCTXCP1_SW2_PRE1_DEFAULT | \
				 XTP_LN_TX_LCTXCP1_SW3_PRE0_DEFAULT)

enum mtk_dp_phya_ana_glb_regidx {
	DP_PHYA_GLB_FORCE_CTRL_0,
	DP_PHYA_GLB_FORCE_CTRL_1,
	DP_PHYA_GLOBAL_MAX
};

enum mtk_dp_phyd_dig_lane_regidx {
	DP_PHYD_LAN_DRIVING_PARAM_0,
	DP_PHYD_LAN_MAX
};

enum mtk_dp_phyd_dig_glb_regidx {
	DP_PHYD_PLL_CTL_0,
	DP_PHYD_PLL_CTL_1,
	DP_PHYD_SW_RST,
	DP_PHYD_BIT_RATE,
	DP_PHYD_AUX_RX_CTL,
	DP_PHYD_TX_CTL_0,
	DP_PHYD_GLOBAL_MAX
};

static const u8 mt8195_phy_ana_glb_regs[DP_PHYA_GLOBAL_MAX] = {
	[DP_PHYA_GLB_FORCE_CTRL_0] = 0x30,
	[DP_PHYA_GLB_FORCE_CTRL_1] = 0x34,
};

static const u8 mt8195_phy_dig_lane_regs[DP_PHYD_LAN_MAX] = {
	[DP_PHYD_LAN_DRIVING_PARAM_0] = 0x2c,
};

static const u8 mt8195_phy_dig_glb_regs[DP_PHYD_GLOBAL_MAX] = {
	[DP_PHYD_PLL_CTL_0] = 0x10,
	[DP_PHYD_PLL_CTL_1] = 0x14,
	[DP_PHYD_SW_RST] = 0x38,
	[DP_PHYD_BIT_RATE] = 0x3c,
	[DP_PHYD_AUX_RX_CTL] = 0x40,
	[DP_PHYD_TX_CTL_0] = 0x44,
};

/**
 * struct mtk_dp_phy_pdata - Platform data and defaults for MediaTek DP/eDP PHY
 * @off_ana_glb:    Base offset for dptx_phyd_sifslv_ana_glb
 * @off_dig_glb:    Base offset for dptx_phyd_sifslv_dig_glb
 * @off_dig_lane:   Base offsets for dptx_phyd_sifslv_dig_lan (for each lane)
 * @regs_ana_glb:   Register (layout) offsets for ana_glb
 * @regs_dig_glb:   Register (layout) offsets for dig_glb
 * @regs_dig_lane:  Register (layout) offsets for dig_lan
 */
struct mtk_dp_phy_pdata {
	/* Register offsets */
	u16 off_ana_glb;
	u16 off_dig_glb;
	u16 off_dig_lane[MTK_DP_PHY_MAX_LANES];

	/* Register maps */
	const u8 *regs_ana_glb;
	const u8 *regs_dig_glb;
	const u8 *regs_dig_lane;
};

struct mtk_dp_phy {
	struct device *dev;
	struct regmap *regmap;
	const struct mtk_dp_phy_pdata *pdata;
};

static int mtk_dp_phy_init(struct phy *phy)
{
	struct mtk_dp_phy *dp_phy = phy_get_drvdata(phy);
	const struct mtk_dp_phy_pdata *pdata = dp_phy->pdata;
	const u32 reg = pdata->regs_dig_lane[DP_PHYD_LAN_DRIVING_PARAM_0];
	static const u32 driving_params[] = {
		DRIVING_PARAM_0_DEFAULT,
		DRIVING_PARAM_1_DEFAULT,
		DRIVING_PARAM_2_DEFAULT,
		DRIVING_PARAM_3_DEFAULT,
		DRIVING_PARAM_4_DEFAULT,
		DRIVING_PARAM_5_DEFAULT,
		DRIVING_PARAM_6_DEFAULT,
		DRIVING_PARAM_7_DEFAULT,
		DRIVING_PARAM_8_DEFAULT
	};
	int i, ret;

	/*
	 * Assume that all lanes need the same driving parameters: this
	 * will bulk write from DRIVING_PARAM_0 to DRIVING_PARAM_8 on
	 * all lanes (a grand total of [9 * num_lanes] 32-bit writes)
	 */
	for (i = 0; i < MTK_DP_PHY_MAX_LANES; i++) {
		ret = regmap_bulk_write(dp_phy->regmap,
					pdata->off_dig_lane[i] + reg,
					driving_params,
					ARRAY_SIZE(driving_params));
		if (ret)
			return ret;
	};

	return 0;
}

static int mtk_dp_phy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	struct mtk_dp_phy *dp_phy = phy_get_drvdata(phy);
	const struct mtk_dp_phy_pdata *pdata = dp_phy->pdata;
	u32 val;
	int i;

	if (opts->dp.set_rate) {
		const u32 reg_bit_rate = pdata->regs_dig_glb[DP_PHYD_BIT_RATE];

		switch (opts->dp.link_rate) {
		default:
			dev_err(&phy->dev,
				"Implementation error, unknown linkrate %x\n",
				opts->dp.link_rate);
			return -EINVAL;
		case 1620:
			val = BIT_RATE_RBR;
			break;
		case 2700:
			val = BIT_RATE_HBR;
			break;
		case 5400:
			val = BIT_RATE_HBR2;
			break;
		case 8100:
			val = BIT_RATE_HBR3;
			break;
		}
		regmap_write(dp_phy->regmap, pdata->off_dig_glb + reg_bit_rate, val);
	}

	if (opts->dp.set_lanes) {
		const u32 reg_dig_tx_ctl = pdata->regs_dig_glb[DP_PHYD_TX_CTL_0];

		val = 0;
		for (i = 0; i < opts->dp.lanes; i++)
			val |= FIELD_PREP(PHYD_TX_LN_EN, BIT(i));

		regmap_update_bits(dp_phy->regmap, pdata->off_dig_glb + reg_dig_tx_ctl,
				   PHYD_TX_LN_EN, val);
	}

	regmap_update_bits(dp_phy->regmap,
			   pdata->off_dig_glb + pdata->regs_dig_glb[DP_PHYD_PLL_CTL_1],
			   TPLL_SSC_EN, opts->dp.ssc ? TPLL_SSC_EN : 0);

	return 0;
}

static int mtk_dp_phy_power_on(struct phy *phy)
{
	struct mtk_dp_phy *dp_phy = phy_get_drvdata(phy);
	const struct mtk_dp_phy_pdata *pdata = dp_phy->pdata;
	const u8 *regs_dig = pdata->regs_dig_glb;
	const u8 *regs_ana = pdata->regs_ana_glb;

	/* Enable AUX Channel with RX De-Glitch and input clock detection */
	regmap_write(dp_phy->regmap,
		     pdata->off_dig_glb + regs_dig[DP_PHYD_AUX_RX_CTL],
		     PHYD_DIG_DPAUX_RX_EN |
		     PHYD_DIG_XTP_GLB_CKDET_EN |
		     PHYD_DIG_DPAUX_RX_DEGLITCH_EN);

	regmap_clear_bits(dp_phy->regmap,
			  pdata->off_ana_glb + regs_ana[DP_PHYA_GLB_FORCE_CTRL_1],
			  CKM_CKTX0_EN_FORCE_MODE);

	return 0;
}

static int mtk_dp_phy_disable_all_lanes(struct mtk_dp_phy *dp_phy)
{
	const struct mtk_dp_phy_pdata *pdata = dp_phy->pdata;
	const u8 *regs = pdata->regs_dig_glb;
	int ret;
	u32 val;

	/* Get mask of currently enabled lane */
	regmap_read(dp_phy->regmap, pdata->off_dig_glb + regs[DP_PHYD_TX_CTL_0], &val);
	val = FIELD_GET(PHYD_TX_LN_EN, val);
	if (val == 0)
		return 0;

	/* Disable all lanes (needs to be done one by one, from last to first) */
	do {
		u32 lane_num = fls(val) - 1;
		val &= ~BIT(lane_num);

		regmap_clear_bits(dp_phy->regmap,
				  pdata->off_dig_glb + regs[DP_PHYD_TX_CTL_0],
				  FIELD_PREP(PHYD_TX_LN_EN, BIT(lane_num)));
	} while (val);

	return 0;
}

static int mtk_dp_phy_power_off(struct phy *phy)
{
	struct mtk_dp_phy *dp_phy = phy_get_drvdata(phy);
	const struct mtk_dp_phy_pdata *pdata = dp_phy->pdata;
	const u8 *regs_dig = pdata->regs_dig_glb;
	const u8 *regs_ana = pdata->regs_ana_glb;
	int ret;

	regmap_set_bits(dp_phy->regmap,
			pdata->off_ana_glb + regs_ana[DP_PHYA_GLB_FORCE_CTRL_1],
			CKM_CKTX0_EN_FORCE_MODE);

	/* Disable RX unconditionally */
	regmap_write(dp_phy->regmap,
		     pdata->off_dig_glb + regs_dig[DP_PHYD_AUX_RX_CTL], 0);

	ret = mtk_dp_phy_disable_all_lanes(dp_phy);
	if (ret) {
		dev_err(dp_phy->dev, "Could not disable lanes for poweroff!\n");
		return ret;
	}

	return 0;
}

static int mtk_dp_phy_reset(struct phy *phy)
{
	struct mtk_dp_phy *dp_phy = phy_get_drvdata(phy);
	const struct mtk_dp_phy_pdata *pdata = dp_phy->pdata;
	const u32 reg_rst = pdata->regs_dig_glb[DP_PHYD_SW_RST];
	int ret;

	/* Clearing bits sets reset state */
	regmap_clear_bits(dp_phy->regmap, pdata->off_dig_glb + reg_rst, DP_GLB_SW_RST_PHYD);

	/* PHYD needs 50uS to guarantee reset done */
	usleep_range(50, 200);

	/* Setting bits means go out of reset */
	regmap_set_bits(dp_phy->regmap, pdata->off_dig_glb + reg_rst, DP_GLB_SW_RST_PHYD);

	/* Disable all lanes and continue reset even if this fails, but notify */
	ret = mtk_dp_phy_disable_all_lanes(dp_phy);
	if (ret)
		dev_err(dp_phy->dev, "Could not disable lanes during reset!\n");

	return 0;
}

static const struct phy_ops mtk_dp_phy_dev_ops = {
	.init = mtk_dp_phy_init,
	.power_on = mtk_dp_phy_power_on,
	.power_off = mtk_dp_phy_power_off,
	.configure = mtk_dp_phy_configure,
	.reset = mtk_dp_phy_reset,
	.owner = THIS_MODULE,
};

static void mtk_dp_phy_legacy_remove_lookup(void *data)
{
	struct phy *phy = data;
	struct mtk_dp_phy *dp_phy = phy_get_drvdata(phy);

	phy_remove_lookup(phy, "dp", dev_name(dp_phy->dev));
}

static const struct mtk_dp_phy_pdata mt8195_dp_phy_data;

static int mtk_dp_phy_legacy_probe(struct platform_device *pdev, struct mtk_dp_phy *dp_phy)
{
	struct device *dev = &pdev->dev;
	struct phy *phy;
	int ret;

	/*
	 * If legacy platform driver probe, assume this is MT8195 or compatible
	 * with a devicetree that was not migrated to the new, proper bindings.
	 */
	dp_phy->pdata = &mt8195_dp_phy_data;
	dp_phy->regmap = *(struct regmap **)dev->platform_data;
	if (!dp_phy->regmap)
		return dev_err_probe(dev, -EINVAL, "No platform data available\n");

	phy = devm_phy_create(dev, NULL, &mtk_dp_phy_dev_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy),
				     "Failed to create DP PHY\n");

	phy_set_drvdata(phy, dp_phy);
	ret = phy_create_lookup(phy, "dp", dev_name(dev));
	if (ret)
		return ret;

	ret = devm_add_action_or_reset(dev, mtk_dp_phy_legacy_remove_lookup, phy);
	if (ret)
		return ret;

	return 0;
}

static const struct regmap_config mtk_dp_phy_regmap_cfg = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.disable_locking = true,
};

static int mtk_dp_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *provider;
	struct mtk_dp_phy *dp_phy;
	void __iomem *base;
	struct phy *phy;
	int ret;

	dp_phy = devm_kzalloc(dev, sizeof(*dp_phy), GFP_KERNEL);
	if (!dp_phy)
		return -ENOMEM;

	dp_phy->dev = dev;

	/* If there's no devicetree, go for legacy pdev probe */
	if (!dev->of_node)
		return mtk_dp_phy_legacy_probe(pdev, dp_phy);

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	dp_phy->regmap = devm_regmap_init_mmio(dev, base, &mtk_dp_phy_regmap_cfg);
	if (IS_ERR(dp_phy->regmap))
		return PTR_ERR(dp_phy->regmap);

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	dp_phy->pdata = device_get_match_data(dev);

	phy = devm_phy_create(dev, NULL, &mtk_dp_phy_dev_ops);
	if (IS_ERR(phy))
		return dev_err_probe(dev, PTR_ERR(phy),
				     "Failed to create DP PHY\n");

	phy_set_drvdata(phy, dp_phy);

	provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);
	if (IS_ERR(provider))
		return PTR_ERR(provider);

	return 0;
}

static const struct mtk_dp_phy_pdata mt8195_dp_phy_data = {
	.off_ana_glb = 0x0,
	.off_dig_glb = 0x1000,
	.off_dig_lane = (const u16[]) { 0x1100, 0x1200, 0x1300, 0x1400 },
	.regs_ana_glb = mt8195_phy_ana_glb_regs,
	.regs_dig_glb = mt8195_phy_dig_glb_regs,
	.regs_dig_lane = mt8195_phy_dig_lane_regs,
};

static const struct of_device_id mtk_dp_phy_of_match[] = {
	{ .compatible = "mediatek,mt8195-dp-phy", .data = &mt8195_dp_phy_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_dp_phy_of_match);

static struct platform_driver mtk_dp_phy_driver = {
	.probe = mtk_dp_phy_probe,
	.driver = {
		.name = "mediatek-dp-phy",
		.of_match_table = mtk_dp_phy_of_match,
	},
};
module_platform_driver(mtk_dp_phy_driver);

MODULE_AUTHOR("AngeloGioacchino Del Regno <angelogioacchino.delregno@collabora.com>");
MODULE_AUTHOR("Markus Schneider-Pargmann <msp@baylibre.com>");
MODULE_DESCRIPTION("MediaTek DisplayPort PHY Driver");
MODULE_LICENSE("GPL");
