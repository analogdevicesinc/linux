// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2024, Analog Devices Incorporated, All Rights Reserved
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_mdio.h>
#include <linux/sfp.h>
#include <linux/platform_device.h>
#include "adrv906x-phy.h"
#include "adrv906x-phy-serdes.h"
#include "adrv906x-net.h"
#include "adrv906x-tsu.h"

struct adrv906x_phy_priv {
	struct adrv906x_serdes serdes;
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

static void adrv906x_phy_get_strings(struct phy_device *phydev, u8 *data)
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

static void adrv906x_phy_get_stats(struct phy_device *phydev,
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

	if (val & ADRV906X_PCS_STAT2_10GBR) {
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseCR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseKR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseSR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseLR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_10000baseLRM_Full_BIT, phydev->supported);
	}
	if (val & ADRV906X_PCS_STAT2_25GBR) {
		linkmode_set_bit(ETHTOOL_LINK_MODE_25000baseCR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_25000baseKR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_25000baseSR_Full_BIT, phydev->supported);
		linkmode_set_bit(ETHTOOL_LINK_MODE_FEC_RS_BIT, phydev->supported);
	}

	linkmode_set_bit(ETHTOOL_LINK_MODE_FIBRE_BIT, phydev->supported);
	linkmode_copy(phydev->advertising, phydev->supported);

	return 0;
}

static void adrv906x_phy_rx_path_enable(struct phy_device *phydev, bool enable)
{
	phy_modify_mmd_changed(phydev, MDIO_MMD_PCS, ADRV906X_PCS_GENERAL_RX_REG,
			       ADRV906X_PCS_GENERAL_PATH_RESET, !enable);
}

static void adrv906x_phy_tx_path_enable(struct phy_device *phydev, bool enable)
{
	phy_modify_mmd_changed(phydev, MDIO_MMD_PCS, ADRV906X_PCS_GENERAL_TX_REG,
			       ADRV906X_PCS_GENERAL_PATH_RESET, !enable);
}

static int adrv906x_phy_suspend(struct phy_device *phydev)
{
	adrv906x_phy_rx_path_enable(phydev, false);
	adrv906x_phy_tx_path_enable(phydev, false);
	adrv906x_serdes_cal_stop(phydev);

	return 0;
}

static void adrv906x_phy_link_change_notify(struct phy_device *phydev)
{
	struct net_device *netdev = phydev->attached_dev;
	struct adrv906x_eth_dev *adrv906x_dev = netdev_priv(netdev);
	struct adrv906x_tsu *tsu = &adrv906x_dev->tsu;
	u32 bit_slip, buf_delay_tx, buf_delay_rx;
	bool rs_fec_enabled = false;
	int i, val;

	if (!phydev->link)
		return;

	if (adrv906x_tod_cfg_cdc_delay < 0) {
		phydev_err(phydev, "tod cfg_cdc_delay not initialized yet");
		return;
	}

	/* We need a dummy read to get the correct value. */
	phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_BRMGBT_STAT2);
	val = phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_BRMGBT_STAT2);
	if (!(val & ADRV906X_PCS_BRMGBT_STAT2_LBLKLK)) {
		phydev_warn(phydev, "pcs not locked and synced to the ethernet block");
		return;
	}

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_RS_FEC_CTRL_REG);
	if (val & ADRV906X_PCS_RS_FEC_CTRL_EN) {
		val = phy_read_mmd(phydev, MDIO_MMD_PCS,
				   ADRV906X_PCS_RS_FEC_STAT_REG);
		if (!(val & ADRV906X_PCS_RS_FEC_STAT_ALIGN)) {
			phydev_warn(phydev, "rs-fec is not locked and aligned");
			return;
		}

		rs_fec_enabled = true;
	}

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_DELAY_RX_REG);
	bit_slip = FIELD_GET(ADRV906X_PCS_DELAY_RX_BIT_SLIP, val);

	buf_delay_rx = U32_MAX;
	for (i = 0; i < 10; i++) {
		u32 d;

		val = phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_BUF_STAT_RX_REG);
		d = FIELD_GET(ADRV906X_PCS_BUF_STAT_RX_FINE_DELAY, val);
		if (d < buf_delay_rx)
			buf_delay_rx = d;
	}

	buf_delay_tx = U32_MAX;
	for (i = 0; i < 10; i++) {
		u32 d;

		val = phy_read_mmd(phydev, MDIO_MMD_PCS, ADRV906X_PCS_BUF_STAT_TX_REG);
		d = FIELD_GET(ADRV906X_PCS_BUF_STAT_TX_FINE_DELAY, val);
		if (d < buf_delay_tx)
			buf_delay_tx = d;
	}

	adrv906x_tsu_calculate_phy_delay(tsu, phydev->speed, rs_fec_enabled,
					 bit_slip, buf_delay_tx, buf_delay_rx);

	phydev_info(phydev, "static phy delay tx: 0x%08x", tsu->phy_delay_tx);
	phydev_info(phydev, "static phy delay rx: 0x%08x", tsu->phy_delay_rx);

	adrv906x_tsu_set_phy_delay(tsu);
}

static int adrv906x_phy_resume(struct phy_device *phydev)
{
	adrv906x_phy_rx_path_enable(phydev, true);
	adrv906x_phy_tx_path_enable(phydev, true);

	return 0;
}

static int adrv906x_phy_read_status(struct phy_device *phydev)
{
	int val;

	val = phy_read_mmd(phydev, MDIO_MMD_PCS, MDIO_STAT1);

	if (phydev->dev_flags & ADRV906X_PHY_FLAGS_LOOPBACK_TEST)
		phydev->link = 1;
	else
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

static int adrv906x_phy_set_loopback(struct phy_device *phydev, bool enable)
{
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

static int adrv906x_phy_config_init(struct phy_device *phydev)
{
	phydev->autoneg = AUTONEG_DISABLE;
	phydev->duplex = DUPLEX_FULL;
	phydev->port = PORT_FIBRE;

	return 0;
}

static int adrv906x_phy_module_info(struct phy_device *dev,
				    struct ethtool_modinfo *modinfo)
{
	int ret = -EOPNOTSUPP;

	if (dev->sfp_bus)
		ret = sfp_get_module_info(dev->sfp_bus, modinfo);

	return ret;
}

static const struct sfp_upstream_ops adrv906x_sfp_ops = {
	.attach = phy_sfp_attach,
	.detach = phy_sfp_detach,
};

static int adrv906x_phy_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct adrv906x_phy_priv *adrv906x_phy;
	struct device_node *np = dev->of_node;
	u32 mmd_mask = MDIO_DEVS_PCS;
	int ret;

	if (!phydev->is_c45 ||
	    (phydev->c45_ids.devices_in_package & mmd_mask) != mmd_mask)
		return -ENODEV;

	adrv906x_phy = devm_kzalloc(dev, sizeof(*adrv906x_phy), GFP_KERNEL);
	if (!adrv906x_phy)
		return -ENOMEM;

	phydev->priv = adrv906x_phy;

	if (of_property_read_u32(np, "speed", &phydev->speed)) {
		phydev->speed = SPEED_25000;
	} else {
		if (phydev->speed != SPEED_10000 && phydev->speed != SPEED_25000) {
			dev_warn(dev, "dt: phy unsupported speed: %d, defaulting to 25000", phydev->speed);
			phydev->speed = SPEED_25000;
		}
	}

	ret = adrv906x_serdes_open(phydev, &adrv906x_phy->serdes,
				   adrv906x_phy_tx_path_enable, adrv906x_phy_rx_path_enable);
	if (ret)
		return ret;

	return phy_sfp_probe(phydev, &adrv906x_sfp_ops);
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
		.set_loopback = adrv906x_phy_set_loopback,
		.get_sset_count = adrv906x_phy_get_sset_count,
		.get_strings = adrv906x_phy_get_strings,
		.get_stats = adrv906x_phy_get_stats,
		.get_features = adrv906x_phy_get_features,
		.resume = adrv906x_phy_resume,
		.suspend = adrv906x_phy_suspend,
		.link_change_notify = adrv906x_phy_link_change_notify,
		.module_info = adrv906x_phy_module_info,
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

	ret = adrv906x_serdes_genl_register_family();
	if (ret) {
		pr_err("%s : register generic netlink family failed", __func__);
		return ret;
	}

	return ret;
}
module_init(adrv906x_phy_init);

static void __exit adrv906x_phy_exit(void)
{
	phy_drivers_unregister(adrv906x_phy_driver,
			       ARRAY_SIZE(adrv906x_phy_driver));

	adrv906x_serdes_genl_unregister_family();
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
