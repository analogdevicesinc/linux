// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>
#include "adrv906x-phy-serdes.h"

#define ADRV906X_PHY_ID                                  0x00000000

#define ADRV906X_MAX_PHYS                                2

#define ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN                 BIT(0)

/* ADI PCS registers */
#define ADRV906X_PCS_STATUS_3_REG                        9
#define ADRV906X_PCS_SEED_A0_REG                         34
#define ADRV906X_PCS_SEED_A1_REG                         35
#define ADRV906X_PCS_SEED_A2_REG                         36
#define ADRV906X_PCS_SEED_A3_REG                         37
#define ADRV906X_PCS_SEED_B0_REG                         38
#define ADRV906X_PCS_SEED_B1_REG                         39
#define ADRV906X_PCS_SEED_B2_REG                         40
#define ADRV906X_PCS_SEED_B3_REG                         41
#define ADRV906X_PCS_TEST_CTRL_REG                       42
#define ADRV906X_PCS_TEST_ERROR_CNT_REG                  43
#define ADRV906X_PCS_BER_HIGH_REG                        44
#define ADRV906X_PCS_ERROR_BLOCKS_REG                    45

#define ADRV906X_PCS_GENERAL_TX_REG                      46
#define ADRV906X_PCS_GENERAL_RX_REG                      47
#define   ADRV906X_PCS_GENERAL_SCRAMBLER_BYPASS_EN       BIT(15)
#define   ADRV906X_PCS_GENERAL_CPRI_EN                   BIT(14)
#define   ADRV906X_PCS_GENERAL_SERDES_BUS_WIDTH_MSK      GENMASK(13, 7)
#define   ADRV906X_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH  0x2000
#define   ADRV906X_PCS_GENERAL_SERDES_32_BITS_BUS_WIDTH  0x1000
#define   ADRV906X_PCS_GENERAL_HALF_DUTY_CYCLE_EN        BIT(4)
#define   ADRV906X_PCS_GENERAL_XGMII_BUS_WIDTH_MSK       BIT(3)
#define   ADRV906X_PCS_GENERAL_64_BITS_XGMII             0x0008
#define   ADRV906X_PCS_GENERAL_32_BITS_XGMII             0x0000
#define   ADRV906X_PCS_GENERAL_PRBS23_TESTPATTERN_EN     BIT(2)
#define   ADRV906X_PCS_GENERAL_PRBS7_TESTPATTERN_EN      BIT(1)
#define   ADRV906X_PCS_GENERAL_PATH_RESET                BIT(0)

#define ADRV906X_PCS_CFG_TX_REG                          48
#define   ADRV906X_PCS_CFG_TX_BIT_DELAY_MSK              GENMASK(15, 9)
#define   ADRV906X_PCS_CFG_TX_BUF_INIT_MSK               GENMASK(8, 1)
#define   ADRV906X_PCS_CFG_TX_BUF_INIT                   0x000A
#define   ADRV906X_PCS_CFG_TX_BUF_BYPASS_EN              BIT(0)

#define ADRV906X_PCS_CFG_RX_REG                          49
#define   ADRV906X_PCS_CFG_RX_GEARBOX_BYPASS_EN          BIT(12)
#define   ADRV906X_PCS_CFG_RX_SERDES_LOOPBACK_EN         BIT(11)
#define   ADRV906X_PCS_CFG_RX_CORE_IF_LOOPBACK_EN        BIT(10)
#define   ADRV906X_PCS_CFG_RX_COMMA_SEARCH_DIS           BIT(9)
#define   ADRV906X_PCS_CFG_RX_BUF_INIT_MSK               GENMASK(8, 1)
#define   ADRV906X_PCS_CFG_RX_BUF_INIT                   0x000A
#define   ADRV906X_PCS_CFG_RX_BUF_BYPASS_EN              BIT(0)

#define ADRV906X_PCS_BUF_STAT_TX_REG                     50
#define ADRV906X_PCS_BUF_STAT_RX_REG                     51
#define ADRV906X_PCS_DELAY_RX_REG                        52
#define ADRV906X_PCS_DISP_ERR_REG                        53
#define ADRV906X_PCS_CODE_ERR_REG                        54
#define ADRV906X_PCS_CPCS_SHCV_REG                       55

#define ADRV906X_PCS_RS_FEC_CTRL_REG                     200
#define   ADRV906X_PCS_RS_FEC_CTRL_EN                    BIT(2)

/* Configuration values of PCS specific registers */
#define ADRV906X_PCS_CTRL2_TYPE_SEL_MSK                  GENMASK(3, 0)  /* PCS type selection */
#define ADRV906X_PCS_CTRL2_10GBR                         0x0000
#define ADRV906X_PCS_CTRL2_25GBR                         0x0007         /* 25GBASE-R type */
#define ADRV906X_PCS_CTRL1_SPEED10G (MDIO_CTRL1_SPEEDSELEXT | 0x00)     /* 10 Gb/s */
#define ADRV906X_PCS_CTRL1_SPEED25G (MDIO_CTRL1_SPEEDSELEXT | 0x14)     /* 25 Gb/s */

#define ADRV906X_PCS_STAT2_25GBR                         0x0080         /* 25GBASE-R */
#define ADRV906X_PCS_STAT2_10GBR                         0x0001         /* 10GBASE-R */

#define ADRV906X_TSU_STATIC_PHY_DELAY_RX                 0x0000003C
#define ADRV906X_TSU_STATIC_PHY_DELAY_TX                 0x00000040
#define ADRV906X_TSU_TIMESTAMPING_MODE                   0x00000038
#define   ADRV906X_CORE_SPEED                            BIT(8)
#define   ADRV906X_CORE_SPEED_10G                        0x00000000
#define   ADRV906X_CORE_SPEED_25G                        0x00000100
#define   ADRV906X_PTP_TIMESTAMPING_MODE                 GENMASK(1, 0)
#define   ADRV906X_PTP_TIMESTAMPING_MODE_TWO_STEP        0x00000000     /* Two-step */
#define   ADRV906X_PTP_TIMESTAMPING_MODE_ONE_STEP        0x00000001     /* One-step */
#define   ADRV906X_PTP_TIMESTAMPING_MODE_TRANSP          0x00000002     /* Transparent Clock */

struct adrv906x_mdio_priv {
	struct device *dev;
	void __iomem *pcs_base[ADRV906X_MAX_PHYS];
	void __iomem *tsu_base[ADRV906X_MAX_PHYS];
};

struct adrv906x_tsu {
	u32 phy_delay_tx;
	u32 phy_delay_rx;
};

struct adrv906x_phy_priv {
	struct adrv906x_serdes serdes;
	struct adrv906x_tsu tsu;
};

struct adrv906x_phy_hw_stat {
	const char *string;
	u8 reg;
	u8 shift;
	u8 bits;
};

/* Elastic buffer and synchronization stats */
static const struct adrv906x_phy_hw_stat adrv906x_phy_hw_stats[] = {
	{ "tx_buf_stat_out_of_bounds_ind", ADRV906X_PCS_BUF_STAT_TX_REG, 15, 1	},
	{ "tx_buf_stat_read_occupancy",	   ADRV906X_PCS_BUF_STAT_TX_REG, 8,  7	},
	{ "tx_buf_stat_fine_occupancy",	   ADRV906X_PCS_BUF_STAT_TX_REG, 0,  8	},
	{ "rx_buf_stat_out_of_bounds_ind", ADRV906X_PCS_BUF_STAT_RX_REG, 15, 1	},
	{ "rx_buf_stat_read_occupancy",	   ADRV906X_PCS_BUF_STAT_RX_REG, 8,  7	},
	{ "rx_buf_stat_fine_occupancy",	   ADRV906X_PCS_BUF_STAT_RX_REG, 0,  8	},
	{ "rx_bit_slip_cnt",		   ADRV906X_PCS_DELAY_RX_REG,	 8,  7	},
	{ "rx_delay_byte_cnt",		   ADRV906X_PCS_DELAY_RX_REG,	 4,  3	},
	{ "rx_buf_fine_occupancy_cnt",	   ADRV906X_PCS_DELAY_RX_REG,	 0,  4	},
	{ "disp_error_cnt",		   ADRV906X_PCS_DISP_ERR_REG,	 0,  16 },
	{ "code_error_cnt",		   ADRV906X_PCS_CODE_ERR_REG,	 0,  16 },
	{ "cpc_shcv_error_cnt",		   ADRV906X_PCS_CPCS_SHCV_REG,	 0,  16 },
};

static void adrv906x_parse_tsu_phy_delay(struct phy_device *phydev)
{
	struct adrv906x_phy_priv *adrv906x_phy = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	struct device_node *of_node = dev->of_node;
	u32 val, frac_val;
	int ret;

	ret = of_property_read_u32(of_node, "static-phy-delay-tx-ns", &val);
	if (ret < 0) {
		phydev_warn(phydev,
			    "dt: static-phy-delay-tx-ns missing, using 0");
		val = 0;
	}
	ret = of_property_read_u32(of_node, "static-phy-delay-tx-frac-ns",
				   &frac_val);
	if (ret < 0) {
		phydev_warn(phydev,
			    "dt: static-phy-delay-tx-frac-ns missing, using 0");
		frac_val = 0;
	}
	adrv906x_phy->tsu.phy_delay_tx = (val << 16) | (frac_val & 0xFFFF);
	phydev_info(phydev, "tsu static phy delay tx 0x%08x",
		    adrv906x_phy->tsu.phy_delay_tx);

	ret = of_property_read_u32(of_node, "static-phy-delay-rx-ns", &val);
	if (ret < 0) {
		phydev_warn(phydev,
			    "dt: static-phy-delay-rx-ns missing, using 0");
		val = 0;
	}
	ret = of_property_read_u32(of_node, "static-phy-delay-rx-frac-ns",
				   &frac_val);
	if (ret < 0) {
		phydev_warn(phydev,
			    "dt: static-phy-delay-rx-frac-ns missing, using 0");
		frac_val = 0;
	}
	adrv906x_phy->tsu.phy_delay_rx = (val << 16) | (frac_val & 0xFFFF);
	phydev_info(phydev, "tsu static phy delay rx 0x%08x",
		    adrv906x_phy->tsu.phy_delay_rx);
}

static void adrv906x_tsu_set_phy_delay(struct phy_device *phydev)
{
	struct adrv906x_phy_priv *adrv906x_phy = phydev->priv;
	struct adrv906x_tsu *tsu = &adrv906x_phy->tsu;
	struct mii_bus *bus = phydev->mdio.bus;
	struct adrv906x_mdio_priv *adrv906x_mdio = bus->priv;
	void __iomem *base = adrv906x_mdio->tsu_base[phydev->mdio.addr];

	iowrite32(tsu->phy_delay_tx, base + ADRV906X_TSU_STATIC_PHY_DELAY_TX);
	iowrite32(tsu->phy_delay_rx, base + ADRV906X_TSU_STATIC_PHY_DELAY_RX);
}

static void adrv906x_tsu_set_ptp_timestamping_mode(struct phy_device *phydev)
{
	struct mii_bus *bus = phydev->mdio.bus;
	struct adrv906x_mdio_priv *adrv906x_mdio = bus->priv;
	void __iomem *base = adrv906x_mdio->tsu_base[phydev->mdio.addr];
	u32 mode, val;

	mode = ADRV906X_PTP_TIMESTAMPING_MODE_TWO_STEP;

	val = ioread32(base + ADRV906X_TSU_TIMESTAMPING_MODE);
	val &= ~ADRV906X_PTP_TIMESTAMPING_MODE;
	val |= (mode & ADRV906X_PTP_TIMESTAMPING_MODE);

	iowrite32(val, base + ADRV906X_TSU_TIMESTAMPING_MODE);
}

static void adrv906x_tsu_set_speed(struct phy_device *phydev)
{
	struct mii_bus *bus = phydev->mdio.bus;
	struct adrv906x_mdio_priv *adrv906x_mdio = bus->priv;
	void __iomem *base = adrv906x_mdio->tsu_base[phydev->mdio.addr];
	u32 mode, val;

	mode = (phydev->speed == SPEED_25000) ? ADRV906X_CORE_SPEED_25G : ADRV906X_CORE_SPEED_10G;

	val = ioread32(base + ADRV906X_TSU_TIMESTAMPING_MODE);
	val &= ~ADRV906X_CORE_SPEED;
	val |= (mode & ADRV906X_CORE_SPEED);

	iowrite32(val, base + ADRV906X_TSU_TIMESTAMPING_MODE);
}

static int adrv906x_pseudo_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 val)
{
	struct adrv906x_mdio_priv *priv = bus->priv;
	u32 offset;

	if (mii_id >= ADRV906X_MAX_PHYS)
		return -EADDRNOTAVAIL;

	offset = 4 * (regnum & 0xFFFF);

	iowrite32(val, priv->pcs_base[mii_id] + offset);

	return 0;
}

static int adrv906x_pseudo_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct adrv906x_mdio_priv *priv = bus->priv;
	u32 offset;
	int ret;

	if (mii_id >= ADRV906X_MAX_PHYS)
		return -EADDRNOTAVAIL;

	offset = 4 * (regnum & 0xFFFF);

	ret = ioread32(priv->pcs_base[mii_id] + offset) & 0xFFFF;

	return ret;
}

static int adrv906x_pseudo_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);

	adrv906x_serdes_genl_unregister_family();
	mdiobus_unregister(bus);

	return 0;
}

static int adrv906x_pseudo_mdio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adrv906x_mdio_priv *priv;
	struct mii_bus *bus;
	int idx, ret;

	bus = devm_mdiobus_alloc_size(&pdev->dev, sizeof(*priv));
	if (!bus) {
		dev_err(dev, "failed to allocate private driver data");
		return -ENOMEM;
	}

	priv = bus->priv;
	priv->dev = &pdev->dev;

	for (idx = 0; idx < ADRV906X_MAX_PHYS; idx++) {
		priv->pcs_base[idx] = devm_platform_ioremap_resource(pdev, 2 * idx);
		if (IS_ERR(priv->pcs_base[idx]))
			return PTR_ERR(priv->pcs_base[idx]);
		priv->tsu_base[idx] = devm_platform_ioremap_resource(pdev, 2 * idx + 1);
		if (IS_ERR(priv->tsu_base[idx]))
			return PTR_ERR(priv->tsu_base[idx]);
	}

	bus->name = "adrv906x-pseudo-mdio";
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%d", pdev->name, pdev->id);
	bus->read = adrv906x_pseudo_mdio_read,
	bus->write = adrv906x_pseudo_mdio_write,
	bus->parent = priv->dev;

	ret = of_mdiobus_register(bus, pdev->dev.of_node);
	if (ret) {
		dev_err(dev, "failed to register mdio bus");
		return ret;
	}

	ret = adrv906x_serdes_genl_register_family();
	if (ret) {
		dev_err(dev, "register generic netlink family failed");
		return ret;
	}

	platform_set_drvdata(pdev, bus);

	return 0;
}

static const struct of_device_id adrv906x_mdio_of_match[] = {
	{ .compatible = "adi,adrv906x-mdio", },
	{ },
};

static struct platform_driver adrv906x_mdio_driver = {
	.driver			= {
		.name		= "adrv906x-pseudo-mdio",
		.owner		= THIS_MODULE,
		.of_match_table = adrv906x_mdio_of_match,
	},
	.probe			= adrv906x_pseudo_mdio_probe,
	.remove			= adrv906x_pseudo_mdio_remove,
};

static bool adrv906x_phy_valid_speed(int speed)
{
	switch (speed) {
	case SPEED_10000:
		return true;
	case SPEED_25000:
		return true;
	default:
		return false;
	}
}

static int adrv906x_phy_get_sset_count(struct phy_device *phydev)
{
	return ARRAY_SIZE(adrv906x_phy_hw_stats);
}

void adrv906x_phy_get_strings(struct phy_device *phydev, u8 *data)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(adrv906x_phy_hw_stats); i++)
		strlcpy(data + i * ETH_GSTRING_LEN,
			adrv906x_phy_hw_stats[i].string, ETH_GSTRING_LEN);
}

static u64 adrv906x_phy_get_stat(struct phy_device *phydev, int i)
{
	const struct adrv906x_phy_hw_stat *stat = &adrv906x_phy_hw_stats[i];
	u32 val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, stat->reg);
	val >>= stat->shift;
	val = val & ((1 << stat->bits) - 1);

	return val;
}

void adrv906x_phy_get_stats(struct phy_device *phydev,
			    struct ethtool_stats *stats, u64 *data)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(adrv906x_phy_hw_stats); i++)
		data[i] = adrv906x_phy_get_stat(phydev, i);
}

static int adrv906x_phy_get_features(struct phy_device *phydev)
{
	u32 val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_STAT2);

	if (val & ADRV906X_PCS_STAT2_10GBR)
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT, phydev->supported);
	if (val & ADRV906X_PCS_STAT2_25GBR) {
		linkmode_set_bit(ETHTOOL_LINK_MODE_25000baseCR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_FEC_RS_BIT, phydev->supported);
	}

	linkmode_set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, phydev->supported);
	linkmode_copy(phydev->advertising, phydev->supported);

	return 0;
}

static void adrv906x_phy_path_enable(struct phy_device *phydev, bool enable)
{
	phy_modify_mmd_changed(phydev, MDIO_MMD_PCS, ADRV906X_PCS_GENERAL_RX_REG,
			       ADRV906X_PCS_GENERAL_PATH_RESET, !enable);
	phy_modify_mmd_changed(phydev, MDIO_MMD_PCS, ADRV906X_PCS_GENERAL_TX_REG,
			       ADRV906X_PCS_GENERAL_PATH_RESET, !enable);
}

static void adrv906x_phy_reset_datapath(struct phy_device *phydev)
{
	adrv906x_phy_path_enable(phydev, false);
	adrv906x_phy_path_enable(phydev, true);
}

static int adrv906x_phy_suspend(struct phy_device *phydev)
{
	adrv906x_phy_path_enable(phydev, false);
	adrv906x_serdes_cal_stop(phydev);

	return 0;
}

static void adrv906x_link_change_notify(struct phy_device *phydev)
{
	adrv906x_tsu_set_speed(phydev);

	/* TODO  set delay */
}

static int adrv906x_phy_resume(struct phy_device *phydev)
{
	adrv906x_phy_path_enable(phydev, true);

	return 0;
}

int adrv906x_phy_read_status(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_STAT1);
	phydev->link = !!(val & MDIO_STAT1_LSTATUS);

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL2);
	if ((val & ADRV906X_PCS_CTRL2_TYPE_SEL_MSK) == MDIO_PCS_CTRL2_10GBR) {
		phydev->speed = SPEED_10000;
		phydev->duplex = DUPLEX_FULL;
	} else if ((val & ADRV906X_PCS_CTRL2_TYPE_SEL_MSK) == ADRV906X_PCS_CTRL2_25GBR) {
		phydev->speed = SPEED_25000;
		phydev->duplex = DUPLEX_FULL;
	} else {
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_UNKNOWN;
	}

	return 0;
}

static int adrv906x_phy_config_pcs_baser_mode(struct phy_device *phydev)
{
	int ctrl1, ctrl2, cfg_tx, cfg_rx, gen_tx, gen_rx;

	if (!adrv906x_phy_valid_speed(phydev->speed)) {
		phydev_err(phydev,
			   "unsupported speed: %d", phydev->speed);
		return -EINVAL;
	}

	adrv906x_phy_path_enable(phydev, false);

	ctrl2 = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL2);
	ctrl2 &= ~ADRV906X_PCS_CTRL2_TYPE_SEL_MSK;

	ctrl1 = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1);
	ctrl1 &= ~MDIO_CTRL1_SPEEDSEL;

	if (phydev->speed == SPEED_25000) {
		ctrl1 |= ADRV906X_PCS_CTRL1_SPEED25G;
		ctrl2 |= ADRV906X_PCS_CTRL2_25GBR;
	} else {
		ctrl1 |= ADRV906X_PCS_CTRL1_SPEED10G;
		ctrl2 |= ADRV906X_PCS_CTRL2_10GBR;
	}

	cfg_tx = ADRV906X_PCS_CFG_TX_BUF_INIT;
	cfg_rx = ADRV906X_PCS_CFG_RX_BUF_INIT;
	gen_tx = ADRV906X_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH |
		 ADRV906X_PCS_GENERAL_64_BITS_XGMII;
	gen_rx = ADRV906X_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH |
		 ADRV906X_PCS_GENERAL_64_BITS_XGMII;

	phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1, ctrl1);
	phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL2, ctrl2);
	phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_CFG_TX_REG, cfg_tx);
	phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_CFG_RX_REG, cfg_rx);
	phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_GENERAL_TX_REG, gen_tx);
	phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_GENERAL_RX_REG, gen_rx);

	if (phydev->speed == SPEED_25000 && phydev->dev_flags & ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN)
		phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_RS_FEC_CTRL_REG,
			      ADRV906X_PCS_RS_FEC_CTRL_EN);
	else
		phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_RS_FEC_CTRL_REG, 0);

	adrv906x_phy_path_enable(phydev, true);

	return 0;
}

static int adrv906x_phy_config_aneg(struct phy_device *phydev)
{
	int ret;

	if (phydev->duplex != DUPLEX_FULL)
		return -EINVAL;

	if (phydev->autoneg != AUTONEG_DISABLE)
		return -EINVAL;

	if (!adrv906x_phy_valid_speed(phydev->speed))
		return -EINVAL;

	ret = adrv906x_phy_config_pcs_baser_mode(phydev);
	if (ret)
		return ret;

	ret = adrv906x_serdes_cal_start(phydev);
	if (ret)
		return ret;

	return genphy_c45_an_disable_aneg(phydev);
}

static int adrv906x_phy_aneg_done(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_STAT1);

	return !!(val & MDIO_STAT1_LSTATUS);
}

static int adrv906x_phy_set_loopback(struct phy_device *phydev, bool enable)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_CFG_RX_REG);

	if (enable)
		val |= ADRV906X_PCS_CFG_RX_SERDES_LOOPBACK_EN;
	else
		val &= ~ADRV906X_PCS_CFG_RX_SERDES_LOOPBACK_EN;

	phy_write_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_CFG_RX_REG, val);

	return 0;
}

static int adrv906x_phy_config_init(struct phy_device *phydev)
{
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->duplex = DUPLEX_FULL;
	phydev->port = PORT_FIBRE;
	phydev->speed = 25000;
	phydev->dev_flags |= ADRV906X_PHY_FLAGS_PCS_RS_FEC_EN;

	adrv906x_tsu_set_ptp_timestamping_mode(phydev);

	return 0;
}

static int adrv906x_phy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct adrv906x_phy_priv *adrv906x_phy;
	u32 mmd_mask = MDIO_DEVS_PCS;
	int ret;

	if (!phydev->is_c45 ||
	    (phydev->c45_ids.devices_in_package & mmd_mask) != mmd_mask)
		return -ENODEV;

	adrv906x_phy = devm_kzalloc(dev, sizeof(*adrv906x_phy), GFP_KERNEL);
	if (!adrv906x_phy)
		return -ENOMEM;
	phydev->priv = adrv906x_phy;

	adrv906x_parse_tsu_phy_delay(phydev);
	adrv906x_tsu_set_phy_delay(phydev);

	ret = adrv906x_serdes_open(phydev, &adrv906x_phy->serdes, adrv906x_phy_reset_datapath);
	if (ret)
		return ret;

	return 0;
}

static void adrv906x_phy_remove(struct phy_device *phydev)
{
	adrv906x_serdes_close(phydev);
}

static struct phy_driver adrv906x_phy_driver[] = {
	{
		PHY_ID_MATCH_EXACT(ADRV906X_PHY_ID),
		.name = "adrv906x-phy",
		.probe = adrv906x_phy_probe,
		.remove = adrv906x_phy_remove,
		.config_init = adrv906x_phy_config_init,
		.soft_reset = genphy_soft_reset,
		.config_aneg = adrv906x_phy_config_aneg,
		.aneg_done = adrv906x_phy_aneg_done,
		.read_status = adrv906x_phy_read_status,
		.get_sset_count = adrv906x_phy_get_sset_count,
		.get_strings = adrv906x_phy_get_strings,
		.get_stats = adrv906x_phy_get_stats,
		.get_features = adrv906x_phy_get_features,
		.set_loopback = adrv906x_phy_set_loopback,
		.resume = adrv906x_phy_resume,
		.suspend = adrv906x_phy_suspend,
		.link_change_notify = adrv906x_link_change_notify,
	},
};

static int __init adrv906x_phy_init(void)
{
	int ret;

	ret = phy_drivers_register(adrv906x_phy_driver,
				   ARRAY_SIZE(adrv906x_phy_driver),
				   THIS_MODULE);
	if (ret)
		return ret;
	ret = platform_driver_register(&adrv906x_mdio_driver);
	if (ret)
		phy_drivers_unregister(adrv906x_phy_driver,
				       ARRAY_SIZE(adrv906x_phy_driver));
	return ret;
}
module_init(adrv906x_phy_init);

static void __exit adrv906x_phy_exit(void)
{
	platform_driver_unregister(&adrv906x_mdio_driver);
	phy_drivers_unregister(adrv906x_phy_driver,
			       ARRAY_SIZE(adrv906x_phy_driver));
}
module_exit(adrv906x_phy_exit);

static struct mdio_device_id __maybe_unused adrv906x_phy_ids[] = {
	{ PHY_ID_MATCH_MODEL(ADRV906X_PHY_ID) },
	{				      }
};

MODULE_DEVICE_TABLE(mdio, adrv906x_phy_ids);

MODULE_DESCRIPTION("ADRV906X Gigabit Ethernet PHY driver");
MODULE_AUTHOR("Slawomir Kulig <slawomir.kulig@analog.com>");
MODULE_LICENSE("GPL");
