// SPDX-License-Identifier: GPL-2.0
/*
 * phy-adi-sdhci.c - PHY driver for ADI sdhci PHY.
 *
 * Copyright (c) 2023, Analog Devices Incorporated, All Rights Reserved
 *
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>

/* PHY register offsets */
#define SDHCI_PHY_CNFG_R_OFF                     (0x00U)
#define SDHCI_PHY_CMDPAD_CNFG_R_OFF              (0x04U)
#define SDHCI_PHY_DATPAD_CNFG_R_OFF              (0x06U)
#define SDHCI_PHY_STBPAD_CNFG_R_OFF              (0x0AU)
#define SDHCI_PHY_RSTNPAD_CNFG_R_OFF             (0x0CU)
#define SDHCI_PHY_COMMDL_CNFG_R_OFF              (0x1CU)
#define SDHCI_PHY_SDCLKDL_CNFG_R_OFF             (0x1DU)
#define SDHCI_PHY_SDCLKDL_DC_R_OFF               (0x1EU)
#define SDHCI_PHY_SMPLDL_CNFG_R_OFF              (0x20U)
#define SDHCI_PHY_ATDL_CNFG_R_OFF                (0x21U)
#define SDHCI_PHY_DLL_CTRL_R_OFF                 (0x24U)
#define SDHCI_PHY_DLL_CNFG1_R_OFF                (0x25U)
#define SDHCI_PHY_DLL_CNFG2_R_OFF                (0x26U)
#define SDHCI_PHY_DLLDL_CNFG_R_OFF               (0x28U)
#define SDHCI_PHY_DLLLBT_CNFG_R_OFF              (0x2CU)
#define SDHCI_PHY_DLL_STATUS_R_OFF               (0x2EU)

/* PHY register field bit masks */
#define SDHCI_PHY_RSTN_BM                        BIT(0)
#define SDHCI_PHY_POWERGOOD_BM                   BIT(1)
#define SDHCI_PHY_DLL_EN_BM                      BIT(0)
#define SDHCI_PHY_LOCK_STS_BM                    BIT(0)
#define SDHCI_PHY_ERROR_STS_BM                   BIT(1)
#define SDHCI_PHY_SLVDLY_BM                      GENMASK(5, 4)
#define SDHCI_PHY_WAIT_CYCLE_BM                  GENMASK(2, 0)
#define SDHCI_PHY_JUMPSTEP_BM                    GENMASK(6, 0)
#define SDHCI_PHY_MST_INPSEL_BM                  GENMASK(2, 1)
#define SDHCI_PHY_SLV_INPSEL_BM                  GENMASK(6, 5)
#define SDHCI_RXSEL_BM                           GENMASK(2, 0)
#define SDHCI_WEAKPULL_EN_BM                     GENMASK(4, 3)
#define SDHCI_INPSEL_CNFG_BM                     GENMASK(3, 2)
#define SDHCI_DLSTEP_SEL_BM                      BIT(0)
#define SDHCI_CCLK_DC_BM                         GENMASK(6, 0)

/* PHY register field bit positions */
#define SDHCI_PHY_SLVDLY_POS                     (4U)
#define SDHCI_PHY_WAIT_CYCLE_POS                 (0U)
#define SDHCI_PHY_JUMPSTEP_POS                   (0U)
#define SDHCI_PHY_MST_INPSEL_POS                 (1U)
#define SDHCI_PHY_SLV_INPSEL_POS                 (5U)
#define SDHCI_PHY_DLL_EN_POS                     (0U)
#define SDHCI_PAD_SP_POS                         (16U)
#define SDHCI_PAD_SN_POS                         (20U)
#define SDHCI_WEAKPULL_EN_POS                    (3U)
#define SDHCI_INPSEL_CNFG_POS                    (2U)
#define SDHCI_UPDATE_DC_POS                      (4U)
#define SDHCI_DLSTEP_SEL_POS                     (0U)
#define SDHCI_CCLK_DC_POS                        (0U)

/* PHY register field values */
#define SDHCI_PHY_PAD_SN                         (0x8U)
#define SDHCI_PHY_PAD_SP                         (0x8U)
#define SDHCI_PHY_SLVDLY                         (0x2U)
#define SDHCI_PHY_WAIT_CYCLE                     (0x0U)
#define SDHCI_PHY_JUMPSTEP                       (0x20U)
#define SDHCI_PHY_MST_INPSEL                     (0x0U)
#define SDHCI_PHY_SLV_INPSEL                     (0x3U)
#define SDHCI_PHY_LBT_LOADVAL                    (0x12U)
#define SDHCI_PHY_DLL_EN                         (0x1U)
#define SDHCI_PHY_LOCK_STS                       (0x1U)
#define SDHCI_PHY_ERROR_STS                      (0x0U)
#define SDHCI_RXSEL_CMD_PAD                      (0x1U)
#define SDHCI_RXSEL_DAT_PAD                      (0x1U)
#define SDHCI_RXSEL_RST_N_PAD                    (0x1U)
#define SDHCI_RXSEL_STB_N_PAD                    (0x1U)
#define SDHCI_WEAKPULL_EN_CMD_PAD                (0x1U)
#define SDHCI_WEAKPULL_EN_DAT_PAD                (0x1U)
#define SDHCI_WEAKPULL_EN_RST_N_PAD              (0x1U)
#define SDHCI_WEAKPULL_EN_STB_PAD                (0x2U)
#define SDHCI_UPDATE_DC                          (0x1U)
#define SDHCI_DLSTEP_SEL                         (0x1U)
#define SDHCI_DEFAULT_CCLK_DC_LEGACY             (0x78U)
#define SDHCI_DEFAULT_CCLK_DC_HS200              (0x0U)
#define SDHCI_DEFAULT_CCLK_DC_HS400              (0x8U)

/* PHY powergood timeout value */
#define SDHCI_PHY_TIMEOUT_100_MS                 (100U)

/* PHY 0 Delay Lines input selection */
#define SDHCI_PHY_0_DL_0_INPSEL_IDL0_IN                (0)
#define SDHCI_PHY_0_DL_0_INPSEL_ITST_CLKIN             (1)
#define SDHCI_PHY_0_DL_0_INPSEL_ZERO                   (2)
#define SDHCI_PHY_0_DL_0_INPSEL_IDL1_IN                (3)

#define SDHCI_PHY_0_DL_1_INPSEL_IDL1_IN                (0)
#define SDHCI_PHY_0_DL_1_INPSEL_ITST_CLKIN             (1)
#define SDHCI_PHY_0_DL_1_INPSEL_ZERO                   (2)
#define SDHCI_PHY_0_DL_1_INPSEL_OY_CLK                 (3)

#define SDHCI_PHY_0_DL_2_INPSEL_IDL2_IN                (0)
#define SDHCI_PHY_0_DL_2_INPSEL_ITST_CLKIN             (1)
#define SDHCI_PHY_0_DL_2_INPSEL_ODL1_OUT               (2)
#define SDHCI_PHY_0_DL_2_INPSEL_IDL1_IN                (3)

/* SDHCI PHY configure operations */
#define SDHCI_PHY_OPS_CFG_DLL_NO_CLK                   (1U)
#define SDHCI_PHY_OPS_ENABLE_DLL_AFTER_CLK             (2U)
#define SDHCI_PHY_OPS_SET_DELAY                        (3U)

/* HS timing */
#define MMC_TIMING_LEGACY       0
#define MMC_TIMING_MMC_HS       1
#define MMC_TIMING_MMC_DDR52    8
#define MMC_TIMING_MMC_HS200    9
#define MMC_TIMING_MMC_HS400    10

struct adi_sdhci_phy {
	void __iomem *base;
	u32 dcode_legacy;
	u32 dcode_hs200;
	u32 dcode_hs400;
};

static void adi_sdhci_phy_writel(struct adi_sdhci_phy *adi_phy, u32 val, int reg)
{
	writel(val, adi_phy->base + reg);
}

static void adi_sdhci_phy_writew(struct adi_sdhci_phy *adi_phy, u16 val, int reg)
{
	writew(val, adi_phy->base + reg);
}

static void adi_sdhci_phy_writeb(struct adi_sdhci_phy *adi_phy, u8 val, int reg)
{
	writeb(val, adi_phy->base + reg);
}

static u32 adi_sdhci_phy_readl(struct adi_sdhci_phy *adi_phy, int reg)
{
	return readl(adi_phy->base + reg);
}

static u16 adi_sdhci_phy_readw(struct adi_sdhci_phy *adi_phy, int reg)
{
	return readw(adi_phy->base + reg);
}

static u8 adi_sdhci_phy_readb(struct adi_sdhci_phy *adi_phy, int reg)
{
	return readb(adi_phy->base + reg);
}

static int adi_sdhci_phy_config_dll(struct phy *phy, bool enable)
{
	struct adi_sdhci_phy *adi_phy = phy_get_drvdata(phy);
	u8 u8_val;
	u8 timeout;
	int err = 0;

	if (enable == false) {
		/* DLL configuration (PHY instance 2)*/

		/* sdhci PHY DLL slave's update delay input */
		u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_DLL_CNFG1_R_OFF) &
			 ~(SDHCI_PHY_SLVDLY_BM | SDHCI_PHY_WAIT_CYCLE_BM);
		u8_val |= (SDHCI_PHY_SLVDLY << SDHCI_PHY_SLVDLY_POS) |
			  (SDHCI_PHY_WAIT_CYCLE << SDHCI_PHY_WAIT_CYCLE_POS);
		adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_DLL_CNFG1_R_OFF);

		/* sdhci PHY DLL's jump step input */
		u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_DLL_CNFG2_R_OFF) &
			 ~(SDHCI_PHY_JUMPSTEP_BM);
		u8_val |= (SDHCI_PHY_JUMPSTEP << SDHCI_PHY_JUMPSTEP_POS);
		adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_DLL_CNFG2_R_OFF);

		/* sdhci PHY Clock select for slave DL */
		u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_DLLDL_CNFG_R_OFF) &
			 ~(SDHCI_PHY_MST_INPSEL_BM | SDHCI_PHY_SLV_INPSEL_BM);
		u8_val |= (SDHCI_PHY_MST_INPSEL << SDHCI_PHY_MST_INPSEL_POS) |
			  (SDHCI_PHY_SLV_INPSEL << SDHCI_PHY_SLV_INPSEL_POS);
		adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_DLLDL_CNFG_R_OFF);

		/* sdhci PHY Low bandwidth timer */
		adi_sdhci_phy_writeb(adi_phy, SDHCI_PHY_LBT_LOADVAL, SDHCI_PHY_DLLLBT_CNFG_R_OFF);
	} else {
		/* sdhci PHY Control settings - DLL enable */
		u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_DLL_CTRL_R_OFF) &
			 ~(SDHCI_PHY_DLL_EN_BM);
		u8_val |= (SDHCI_PHY_DLL_EN << SDHCI_PHY_DLL_EN_POS);
		adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_DLL_CTRL_R_OFF);

		/* Wait for DLL lock */
		timeout = SDHCI_PHY_TIMEOUT_100_MS;
		while (0U == (adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_DLL_STATUS_R_OFF) & SDHCI_PHY_LOCK_STS_BM)) {
			if (timeout-- > 0) {
				udelay(1000U);
			} else {
				pr_err("%s: PHY DLL has not locked.\n", __func__);
				return -ETIMEDOUT;
			}
		}
		if (0U != (adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_DLL_STATUS_R_OFF) & SDHCI_PHY_ERROR_STS_BM)) {
			pr_err("%s: PHY DLL is lock to default with errors.\n", __func__);
			return -ETIMEDOUT;
		}
	}

	return err;
}

static int adi_sdhci_phy_set_delay(struct phy *phy, u8 hs_timing)
{
	struct adi_sdhci_phy *adi_phy = phy_get_drvdata(phy);
	u8 dl0;
	u8 dl1;
	u8 dl2;
	u8 dl1_code;
	u8 u8_val;
	u8 u8_aux;

	/* PHY instance 1 Delay Lines  */
	if ((hs_timing == MMC_TIMING_MMC_HS400) || (hs_timing == MMC_TIMING_MMC_HS200)) {
		dl0 = SDHCI_PHY_0_DL_0_INPSEL_IDL1_IN;
		dl1 = SDHCI_PHY_0_DL_1_INPSEL_IDL1_IN;
		dl2 = SDHCI_PHY_0_DL_2_INPSEL_IDL1_IN;
		if (hs_timing == MMC_TIMING_MMC_HS400)
			dl1_code = adi_phy->dcode_hs400;
		else
			dl1_code = adi_phy->dcode_hs200;
	} else {
		dl0 = SDHCI_PHY_0_DL_0_INPSEL_ZERO;
		dl1 = SDHCI_PHY_0_DL_1_INPSEL_IDL1_IN;
		dl2 = SDHCI_PHY_0_DL_2_INPSEL_ODL1_OUT;
		dl1_code = adi_phy->dcode_legacy;
	}

	/* sdhci autotuning clock DelayLine configuration settings */
	u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_ATDL_CNFG_R_OFF) & ~SDHCI_INPSEL_CNFG_BM;
	u8_val |= (dl0 << SDHCI_INPSEL_CNFG_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_ATDL_CNFG_R_OFF);

	/* sdhci tx clock DelayLine configuration settings */
	u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_SDCLKDL_CNFG_R_OFF) & ~SDHCI_INPSEL_CNFG_BM;
	u8_val |= (dl1 << SDHCI_INPSEL_CNFG_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_SDCLKDL_CNFG_R_OFF);

	/* eMMC clk_tx DelayLine value settings
	 *   Note: Card clock must be disabled (the framework do it before calling this function)
	 */
	u8_val |= (SDHCI_UPDATE_DC << SDHCI_UPDATE_DC_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_SDCLKDL_CNFG_R_OFF);
	u8_aux = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_SDCLKDL_DC_R_OFF) &
		 ~SDHCI_CCLK_DC_BM;
	u8_aux |= (dl1_code << SDHCI_CCLK_DC_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_aux, SDHCI_PHY_SDCLKDL_DC_R_OFF);
	u8_val &= ~(SDHCI_UPDATE_DC << SDHCI_UPDATE_DC_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_SDCLKDL_CNFG_R_OFF);

	/* sdhci rx sampling clock DelayLine configuration settings */
	u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_SMPLDL_CNFG_R_OFF) & ~SDHCI_INPSEL_CNFG_BM;
	u8_val |= (dl2 << SDHCI_INPSEL_CNFG_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_SMPLDL_CNFG_R_OFF);

	return 0;
}

static int adi_sdhci_phy_init(struct phy *phy)
{
	struct adi_sdhci_phy *adi_phy = phy_get_drvdata(phy);
	u32 u32_val;
	u16 u16_val;
	u8 u8_val;
	u8 timeout;

	/* sdhci PHY general configuration */
	u32_val = adi_sdhci_phy_readl(adi_phy, SDHCI_PHY_CNFG_R_OFF);
	u32_val |= ((SDHCI_PHY_PAD_SP << SDHCI_PAD_SP_POS) |
		    (SDHCI_PHY_PAD_SN << SDHCI_PAD_SN_POS));
	adi_sdhci_phy_writel(adi_phy, u32_val, SDHCI_PHY_CNFG_R_OFF);

	/* sdhci PHY's command/response PAD settings */
	u16_val = adi_sdhci_phy_readw(adi_phy, SDHCI_PHY_CMDPAD_CNFG_R_OFF) &
		  ~(SDHCI_RXSEL_BM | SDHCI_WEAKPULL_EN_BM);
	u16_val |= (SDHCI_RXSEL_CMD_PAD |
		    (SDHCI_WEAKPULL_EN_CMD_PAD << SDHCI_WEAKPULL_EN_POS));
	adi_sdhci_phy_writew(adi_phy, u16_val, SDHCI_PHY_CMDPAD_CNFG_R_OFF);

	/* sdhci PHY's Data PAD settings */
	u16_val = adi_sdhci_phy_readw(adi_phy, SDHCI_PHY_DATPAD_CNFG_R_OFF) &
		  ~(SDHCI_RXSEL_BM | SDHCI_WEAKPULL_EN_BM);
	u16_val |= (SDHCI_RXSEL_DAT_PAD |
		    (SDHCI_WEAKPULL_EN_DAT_PAD << SDHCI_WEAKPULL_EN_POS));
	adi_sdhci_phy_writew(adi_phy, u16_val, SDHCI_PHY_DATPAD_CNFG_R_OFF);

	/* sdhci PHY's RSTN PAD settings */
	u16_val = adi_sdhci_phy_readw(adi_phy, SDHCI_PHY_RSTNPAD_CNFG_R_OFF) &
		  ~(SDHCI_RXSEL_BM | SDHCI_WEAKPULL_EN_BM);
	u16_val |= (SDHCI_RXSEL_RST_N_PAD |
		    (SDHCI_WEAKPULL_EN_RST_N_PAD << SDHCI_WEAKPULL_EN_POS));
	adi_sdhci_phy_writew(adi_phy, u16_val, SDHCI_PHY_RSTNPAD_CNFG_R_OFF);

	/* sdhci PHY's Strobe PAD settings */
	u16_val = adi_sdhci_phy_readw(adi_phy, SDHCI_PHY_STBPAD_CNFG_R_OFF) &
		  ~(SDHCI_RXSEL_BM | SDHCI_WEAKPULL_EN_BM);
	u16_val |= (SDHCI_RXSEL_STB_N_PAD |
		    (SDHCI_WEAKPULL_EN_STB_PAD << SDHCI_WEAKPULL_EN_POS));
	adi_sdhci_phy_writew(adi_phy, u16_val, SDHCI_PHY_STBPAD_CNFG_R_OFF);

	/* Configure Delay Lines (PHY instance 0) for legacy mode */
	adi_sdhci_phy_set_delay(phy, MMC_TIMING_LEGACY);

	/* eMMC DelayLine's per step delay selection */
	u8_val = adi_sdhci_phy_readb(adi_phy, SDHCI_PHY_COMMDL_CNFG_R_OFF) &
		 ~SDHCI_DLSTEP_SEL_BM;
	u8_val |= (SDHCI_DLSTEP_SEL << SDHCI_DLSTEP_SEL_POS);
	adi_sdhci_phy_writeb(adi_phy, u8_val, SDHCI_PHY_COMMDL_CNFG_R_OFF);

	/* Wait max 100ms for the PHY Powergood to be 1. As per JEDEC Spec v5.1,
	 * supply power-up time for sdhci operating at 1.8V is 25ms, but we give
	 * more time for the PHY to powerup. */
	timeout = SDHCI_PHY_TIMEOUT_100_MS;
	while (0U == (adi_sdhci_phy_readl(adi_phy, SDHCI_PHY_CNFG_R_OFF) & SDHCI_PHY_POWERGOOD_BM)) {
		if (timeout-- > 0) {
			udelay(1000U);
		} else {
			pr_err("%s: PHY Powergood status never asserted.\n", __func__);
			return -ETIMEDOUT;
		}
	}

	/* De-assert PHY Reset */
	u32_val = adi_sdhci_phy_readl(adi_phy, SDHCI_PHY_CNFG_R_OFF);
	u32_val |= SDHCI_PHY_RSTN_BM;
	adi_sdhci_phy_writel(adi_phy, u32_val, SDHCI_PHY_CNFG_R_OFF);

	return 0;
}

/*
 * Argument 'opts' can be casted to any of the available structures
 *     union phy_configure_opts {
 *         struct phy_configure_opts_mipi_dphy	mipi_dphy; <- MIPI
 *         struct phy_configure_opts_dp		dp;        <- Display Port
 *     };
 *
 * It would be nice to add a new one for MMC purposes:
 *         struct phy_configure_opts_sdhci	sdhci;
 *
 * By now let's reuse 'dp' one for MMC purposes
 */
static int adi_sdhci_phy_configure(struct phy *phy, union phy_configure_opts *opts)
{
	unsigned int event = opts->dp.link_rate;
	unsigned int arg1 = opts->dp.lanes;
	int err;

	switch (event) {
	case SDHCI_PHY_OPS_CFG_DLL_NO_CLK:
		err = adi_sdhci_phy_config_dll(phy, false);
		break;
	case SDHCI_PHY_OPS_ENABLE_DLL_AFTER_CLK:
		err = adi_sdhci_phy_config_dll(phy, true);
		break;
	case SDHCI_PHY_OPS_SET_DELAY:
		err = adi_sdhci_phy_set_delay(phy, arg1);
		break;
	default:
		err = -EINVAL;
		break;
	}

	return err;
}
static const struct phy_ops adi_sdhci_phy_ops = {
	.init		= adi_sdhci_phy_init,
	.configure	= adi_sdhci_phy_configure,
	.owner		= THIS_MODULE,
};

static void adi_sdhci_phy_device_tree(struct platform_device *pdev, struct adi_sdhci_phy *adi_phy)
{
	struct device_node *np;

	adi_phy->dcode_legacy = SDHCI_DEFAULT_CCLK_DC_LEGACY;
	adi_phy->dcode_hs200 = SDHCI_DEFAULT_CCLK_DC_HS200;
	adi_phy->dcode_hs400 = SDHCI_DEFAULT_CCLK_DC_HS400;

	np = pdev->dev.of_node;
	if (np) {
		of_property_read_u32(np, "adi,dcode-legacy", &adi_phy->dcode_legacy);
		of_property_read_u32(np, "adi,dcode-hs200", &adi_phy->dcode_hs200);
		of_property_read_u32(np, "adi,dcode-hs400", &adi_phy->dcode_hs400);
	}
}

static int adi_sdhci_phy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adi_sdhci_phy *adi_phy;
	struct phy *generic_phy;
	struct phy_provider *phy_provider;

	if (!dev->parent)
		return -ENODEV;

	adi_phy = devm_kzalloc(dev, sizeof(*adi_phy), GFP_KERNEL);
	if (!adi_phy)
		return -ENOMEM;

	adi_phy->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adi_phy->base))
		return PTR_ERR(adi_phy->base);

	adi_sdhci_phy_device_tree(pdev, adi_phy);

	generic_phy = devm_phy_create(dev, dev->of_node, &adi_sdhci_phy_ops);
	if (IS_ERR(generic_phy)) {
		dev_err(dev, "failed to create PHY\n");
		return PTR_ERR(generic_phy);
	}

	phy_set_drvdata(generic_phy, adi_phy);
	phy_provider = devm_of_phy_provider_register(dev, of_phy_simple_xlate);

	return PTR_ERR_OR_ZERO(phy_provider);
}

static const struct of_device_id adi_sdhci_phy_dt_ids[] = {
	{ .compatible = "adi,sdhci-phy" },
	{}
};

MODULE_DEVICE_TABLE(of, adi_sdhci_phy_dt_ids);

static struct platform_driver adi_sdhci_driver = {
	.probe			= adi_sdhci_phy_probe,
	.driver			= {
		.name		= "adi-sdhci-phy",
		.of_match_table = adi_sdhci_phy_dt_ids,
	},
};

module_platform_driver(adi_sdhci_driver);

MODULE_AUTHOR("Analog Devices Inc.");
MODULE_DESCRIPTION("ADI SDHCI PHY driver");
MODULE_LICENSE("GPL v2");
