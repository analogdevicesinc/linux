// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/platform_device.h>

#define ADRV906X_PHY_ID                             0x00000000

#define ADRV906X_PCS_RX_PATH                             0
#define ADRV906X_PCS_TX_PATH                             1

#define ADRV906X_MAX_NUM_OF_PCS                          2

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
#define   MDIO_PCS_GENERAL_SCRAMBLER_BYPASS_EN      BIT(15)             /* Bypass scrambler in 64b/66b encoder/decoder enabled */
#define   MDIO_PCS_GENERAL_CPRI_EN                  BIT(14)             /* CPRI enabled for 64b/66b encoder/decoder and RS-FEC enabled */
#define   MDIO_PCS_GENERAL_SERDES_BUS_WIDTH_MSK     GENMASK(13, 7)      /* SerDes input/output bus width mask */
#define   MDIO_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH 0x2000              /* SerDes input/output 64 bits bus width */
#define   MDIO_PCS_GENERAL_SERDES_32_BITS_BUS_WIDTH 0x1000              /* SerDes input/output 32 bits bus width */
#define   MDIO_PCS_GENERAL_HALF_DUTY_CYCLE_EN       BIT(4)              /* Half duty cycle enabled */
#define   MDIO_PCS_GENERAL_XGMII_BUS_WIDTH_MSK      BIT(3)              /* XGMII interface bus width mask */
#define   MDIO_PCS_GENERAL_64_BITS_XGMII            0x0008              /* XGMII interface is 64 bits */
#define   MDIO_PCS_GENERAL_32_BITS_XGMII            0x0000              /* XGMII interface is 32 bits */
#define   MDIO_PCS_GENERAL_PRBS23_TESTPATTERN_EN    BIT(2)              /* PRBS23 test-pattern mode enabled */
#define   MDIO_PCS_GENERAL_PRBS7_TESTPATTERN_EN     BIT(1)              /* PRBS7 test-pattern mode enabled */
#define   MDIO_PCS_GENERAL_PATH_RESET               BIT(0)              /* Reset transmit/receive path */

#define ADRV906X_PCS_CFG_TX_REG                          48
#define   MDIO_PCS_CFG_TX_BIT_DELAY_MSK             GENMASK(15, 9)      /* bit delay introduced by transmitter */
#define   MDIO_PCS_CFG_TX_BUF_INIT_MSK              GENMASK(8, 1)       /* Initial fill level of the TX elastic buffer mask */
#define   MDIO_PCS_CFG_TX_BUF_INIT                  0x000A              /* Initial fill level of the TX elastic buffer for 10/25G */
#define   MDIO_PCS_CFG_TX_BUF_BYPASS_EN             BIT(0)              /* TX elastic buffer bypass enabled */

#define ADRV906X_PCS_CFG_RX_REG                          49
#define   MDIO_PCS_CFG_RX_GEARBOX_BYPASS_EN         BIT(12)             /* RX gearboxes bypassed enabled */
#define   MDIO_PCS_CFG_RX_SERDES_LOOPBACK_EN        BIT(11)             /* Loopback at SerDes interface enabled */
#define   MDIO_PCS_CFG_RX_CORE_IF_LOOPBACK_EN       BIT(10)             /* Loopback at core interface enabled */
#define   MDIO_PCS_CFG_RX_COMMA_SEARCH_DIS          BIT(9)              /* Comma-search disabled */
#define   MDIO_PCS_CFG_RX_BUF_INIT_MSK              GENMASK(8, 1)       /* Initial fill level of the RX elastic buffer mask */
#define   MDIO_PCS_CFG_RX_BUF_INIT                  0x000A              /* Initial fill level of the RX elastic buffer for 10/25G */
#define   MDIO_PCS_CFG_RX_BUF_BYPASS_EN             BIT(0)              /* RX elastic buffer bypass enabled */

#define ADRV906X_PCS_BUF_STAT_TX_REG                     50
#define ADRV906X_PCS_BUF_STAT_RX_REG                     51
#define ADRV906X_PCS_DELAY_RX_REG                        52
#define ADRV906X_PCS_DISP_ERR_REG                        53
#define ADRV906X_PCS_CODE_ERR_REG                        54
#define ADRV906X_PCS_CPCS_SHCV_REG                       55

/* Configuration values of PCS specific registers */
#define MDIO_PCS_CTRL2_TYPE_SEL_MSK                 GENMASK(3, 0)       /* PCS type selection */
#define MDIO_PCS_CTRL2_25GBR                        0x0007              /* 25GBASE-R type */
#define MDIO_CTRL1_SPEED25G (MDIO_CTRL1_SPEEDSELEXT | 0x14)             /* 25 Gb/s */

#define MDIO_PCS_STAT2_25GBR                        0x0080              /* 25GBASE-R ability */
#define MDIO_PCS_STAT2_10GBR                        0x0001              /* 10GBASE-R ability */

struct adrv906x_phy_priv {
	struct device *dev;
	void __iomem *base[ADRV906X_MAX_NUM_OF_PCS];
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

static int adrv906x_pseudo_mdio_write(struct mii_bus *bus, int mii_id, int regnum, u16 val)
{
	struct adrv906x_phy_priv *priv = bus->priv;
	u32 offset;

	if (mii_id >= ADRV906X_MAX_NUM_OF_PCS)
		return -EADDRNOTAVAIL;

	offset = 4 * (regnum & 0xFFFF);

	iowrite32(val, priv->base[mii_id] + offset);

	return 0;
}

static int adrv906x_pseudo_mdio_read(struct mii_bus *bus, int mii_id, int regnum)
{
	struct adrv906x_phy_priv *priv = bus->priv;
	u32 offset;
	int ret;

	if (mii_id >= ADRV906X_MAX_NUM_OF_PCS)
		return -EADDRNOTAVAIL;

	offset = 4 * (regnum & 0xFFFF);

	ret = ioread32(priv->base[mii_id] + offset) & 0xFFFF;

	return ret;
}

static int adrv906x_pseudo_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);

	mdiobus_unregister(bus);

	return 0;
}

static int adrv906x_pseudo_mdio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct adrv906x_phy_priv *priv;
	struct mii_bus *bus;
	u32 idx;
	int ret;

	bus = devm_mdiobus_alloc_size(&pdev->dev, sizeof(*priv));
	if (!bus) {
		dev_err(dev, "Failed to allocate private driver data!");
		return -ENOMEM;
	}

	priv = bus->priv;
	priv->dev = &pdev->dev;

	for (idx = 0; idx < ADRV906X_MAX_NUM_OF_PCS; idx++) {
		priv->base[idx] = devm_platform_ioremap_resource(pdev, idx);
		if (IS_ERR(priv->base[idx]))
			return PTR_ERR(priv->base[idx]);
	}

	bus->name = "adi-adrv906x-pseudo-mdio";
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-%d", pdev->name, pdev->id);
	bus->read = adrv906x_pseudo_mdio_read,
	bus->write = adrv906x_pseudo_mdio_write,
	bus->parent = priv->dev;
	ret = of_mdiobus_register(bus, pdev->dev.of_node);

	if (ret) {
		dev_err(dev, "Failed to register MDIO bus");
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
		.name		= "adi-adrv906x-pseudo-mdio",
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

	val = phy_read_mmd(phydev, MDIO_MMD_VEND1, stat->reg);
	if (val < 0)
		return val;
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
	if (val < 0)
		return val;

	if (val & MDIO_PCS_STAT2_10GBR)
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseT_Full_BIT, phydev->supported);
	if (val & MDIO_PCS_STAT2_25GBR)
		linkmode_set_bit(ETHTOOL_LINK_MODE_25000baseCR_Full_BIT, phydev->supported);

	linkmode_set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, phydev->supported);
	linkmode_copy(phydev->advertising, phydev->supported);

	return 0;
}

static int adrv906x_phy_reset_main_path(struct phy_device *phydev, int dir, bool enable)
{
	int reg;

	reg = (dir == ADRV906X_PCS_RX_PATH) ? ADRV906X_PCS_GENERAL_RX_REG : ADRV906X_PCS_GENERAL_TX_REG;

	return phy_modify_mmd_changed(phydev,
				      MDIO_MMD_VEND1, reg, MDIO_PCS_GENERAL_PATH_RESET, enable);
}

static int adrv906x_phy_suspend(struct phy_device *phydev)
{
	int ret;

	ret = adrv906x_phy_reset_main_path(phydev, ADRV906X_PCS_TX_PATH, true);
	if (ret < 0)
		return ret;
	ret = adrv906x_phy_reset_main_path(phydev, ADRV906X_PCS_RX_PATH, true);
	if (ret < 0)
		return ret;

	return 0;
}

static int adrv906x_phy_resume(struct phy_device *phydev)
{
	int ret;

	ret = adrv906x_phy_reset_main_path(phydev, ADRV906X_PCS_TX_PATH, false);
	if (ret < 0)
		return ret;
	ret = adrv906x_phy_reset_main_path(phydev, ADRV906X_PCS_RX_PATH, false);
	if (ret < 0)
		return ret;

	return 0;
}

int adrv906x_phy_read_status(struct phy_device *phydev)
{
	int status1, ctrl1;

	status1 = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_STAT1);
	if (status1 < 0)
		return status1;

	ctrl1 = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ctrl1 < 0)
		return ctrl1;

	phydev->link = !!(status1 & MDIO_STAT1_LSTATUS);

	if ((ctrl1 & MDIO_CTRL1_SPEEDSEL) == MDIO_CTRL1_SPEED10G) {
		phydev->speed = SPEED_10000;
		phydev->duplex = DUPLEX_FULL;
	} else if ((ctrl1 & MDIO_CTRL1_SPEEDSEL) == MDIO_CTRL1_SPEED25G) {
		phydev->speed = SPEED_25000;
		phydev->duplex = DUPLEX_FULL;
	} else {
		phydev->speed = SPEED_UNKNOWN;
		phydev->duplex = DUPLEX_UNKNOWN;
	}

	return 0;
}

static int adrv906x_phy_config_baser_mode(struct phy_device *phydev)
{
	int ctrl1, ctrl2, cfg_tx, cfg_rx, gen_tx, gen_rx, ret;

	if (!adrv906x_phy_valid_speed(phydev->speed)) {
		phydev_err(phydev,
			   "Unsupported speed: %d", phydev->speed);
		return -EINVAL;
	}

	ctrl2 = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL2);
	if (ctrl2 < 0)
		return ctrl2;
	ctrl2 &= ~MDIO_PCS_CTRL2_TYPE_SEL_MSK;

	ctrl1 = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1);
	if (ctrl1 < 0)
		return ctrl1;
	ctrl1 &= ~MDIO_CTRL1_SPEEDSEL;

	switch (phydev->speed) {
	case SPEED_10000:
		ctrl1 |= MDIO_CTRL1_SPEED10G;
		ctrl2 |= MDIO_PCS_CTRL2_10GBR;
		break;
	case SPEED_25000:
		ctrl1 |= MDIO_CTRL1_SPEED25G;
		ctrl2 |= MDIO_PCS_CTRL2_25GBR;
		break;
	default:
		return -EINVAL;
	}

	cfg_tx = MDIO_PCS_CFG_TX_BUF_INIT;
	cfg_rx = MDIO_PCS_CFG_RX_BUF_INIT;
	gen_tx = MDIO_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH
		 | MDIO_PCS_GENERAL_PATH_RESET
		 | MDIO_PCS_GENERAL_64_BITS_XGMII;
	gen_rx = MDIO_PCS_GENERAL_SERDES_64_BITS_BUS_WIDTH
		 | MDIO_PCS_GENERAL_PATH_RESET
		 | MDIO_PCS_GENERAL_64_BITS_XGMII;

	ret = phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL1, ctrl1);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_PCS, MDIO_CTRL2, ctrl2);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, ADRV906X_PCS_CFG_TX_REG, cfg_tx);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, ADRV906X_PCS_CFG_RX_REG, cfg_rx);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, ADRV906X_PCS_GENERAL_TX_REG, gen_tx);
	if (ret < 0)
		return ret;
	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1, ADRV906X_PCS_GENERAL_RX_REG, gen_rx);
	if (ret < 0)
		return ret;

	ret = adrv906x_phy_reset_main_path(phydev, ADRV906X_PCS_TX_PATH, false);
	if (ret < 0)
		return ret;
	ret = adrv906x_phy_reset_main_path(phydev, ADRV906X_PCS_RX_PATH, false);
	if (ret < 0)
		return ret;

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

	ret = adrv906x_phy_config_baser_mode(phydev);
	if (ret < 0)
		return ret;

	return genphy_c45_an_disable_aneg(phydev);
}

static int adrv906x_phy_aneg_done(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_STAT1);
	if (val < 0)
		return val;

	return !!(val & MDIO_STAT1_LSTATUS);
}

static int adrv906x_phy_config_init(struct phy_device *phydev)
{
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->duplex = DUPLEX_FULL;
	phydev->port = PORT_FIBRE;
	phydev->speed = 10000;

	return 0;
}

static int adrv906x_phy_probe(struct phy_device *phydev)
{
	u32 mmd_mask = MDIO_DEVS_PCS;

	if (!phydev->is_c45 ||
	    (phydev->c45_ids.devices_in_package & mmd_mask) != mmd_mask)
		return -ENODEV;

	return 0;
}

static struct phy_driver adrv906x_phy_driver[] = {
	{
		PHY_ID_MATCH_EXACT(ADRV906X_PHY_ID),
		.name = "adi-adrv906x-phy",
		.probe = adrv906x_phy_probe,
		.config_init = adrv906x_phy_config_init,
		.soft_reset = genphy_soft_reset,
		.config_aneg = adrv906x_phy_config_aneg,
		.aneg_done = adrv906x_phy_aneg_done,
		.read_status = adrv906x_phy_read_status,
		.get_sset_count = adrv906x_phy_get_sset_count,
		.get_strings = adrv906x_phy_get_strings,
		.get_stats = adrv906x_phy_get_stats,
		.get_features = adrv906x_phy_get_features,
		.resume = adrv906x_phy_resume,
		.suspend = adrv906x_phy_suspend,
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
