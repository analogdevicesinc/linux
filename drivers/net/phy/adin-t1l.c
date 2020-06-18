// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/**
 *  Driver for Analog Devices Industrial Ethernet T1L PHYs
 *
 * Copyright 2020 Analog Devices Inc.
 */
#include <linux/kernel.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/phy.h>
#include <linux/property.h>

#include "adin-compat.h"

#define PHY_ID_ADIN1100				0x0283bc80

#define ADIN_T1L_FEATURES			(SUPPORTED_10baseT_Full)

#define ADIN_B10L_PCS_CNTRL			0x08e6
#define   ADIN_PCS_CNTRL_B10L_LB_PCS_EN		BIT(14)

#define ADIN_B10L_PMA_CNTRL			0x08f6
#define   ADIN_PMA_CNTRL_B10L_LB_PMA_LOC_EN	BIT(0)

#define ADIN_B10L_PMA_STAT			0x08f7
#define   ADIN_PMA_STAT_B10L_LB_PMA_LOC_ABLE	BIT(13)
#define   ADIN_PMA_STAT_B10L_TX_LVL_HI_ABLE	BIT(12)

#define ADIN_AN_ADV_ABILITY_L			0x0202
#define ADIN_AN_ADV_ABILITY_M			0x0203
#define ADIN_AN_ADV_ABILITY_H			0x0204U
#define   ADIN_AN_ADV_B10L_TX_LVL_HI_ABL	BIT(13)
#define   ADIN_AN_ADV_B10L_TX_LVL_HI_REQ	BIT(12)

#define ADIN_AN_LP_ADV_ABILITY_L		0x0205

#define ADIN_AN_LP_ADV_ABILITY_M		0x0206
#define   ADIN_AN_LP_ADV_B10L			BIT(14)
#define   ADIN_AN_LP_ADV_B1000			BIT(7)
#define   ADIN_AN_LP_ADV_B10S_FD		BIT(6)
#define   ADIN_AN_LP_ADV_B100			BIT(5)
#define   ADIN_AN_LP_ADV_MST			BIT(4)

#define ADIN_AN_LP_ADV_ABILITY_H		0x0207
#define   ADIN_AN_LP_ADV_B10L_EEE		BIT(14)
#define   ADIN_AN_LP_ADV_B10L_TX_LVL_HI_ABL	BIT(13)
#define   ADIN_AN_LP_ADV_B10L_TX_LVL_HI_REQ	BIT(12)
#define   ADIN_AN_LP_ADV_B10S_HD		BIT(11)

#define ADIN_CRSM_SFT_PD_CNTRL			0x8812
#define   ADIN_CRSM_SFT_PD_CNTRL_EN		BIT(0)

#define ADIN_CRSM_STAT				0x8818
#define   ADIN_CRSM_SFT_PD_RDY			BIT(1)

#define ADIN_MAC_IF_LOOPBACK			0x803d
#define   ADIN_MAC_IF_LOOPBACK_EN		BIT(0)
#define   ADIN_MAC_IF_REMOTE_LOOPBACK_EN	BIT(2)

/**
 * struct adin_priv - ADIN PHY driver private data
 * tx_level_24v			set if the PHY supports 2.4V TX levels (10BASE-T1L)
 */
struct adin_priv {
	unsigned int		tx_level_24v:1;
};

/**
 * The ADIN T1L PHY supports Clause 45-only access, but is not a C45 device
 * in the strictest sense of how Linux considers C45 capable devices.
 *
 * For once, it doesn't support C45 package IDs, so it won't probe
 * via the normal get_phy_c45_devs_in_pkg() path.
 * Secondly, the chip implements mostly 10BASE-T1L regs, so autonegotiation
 * requires some other regs which are IEEE spec-ed, but not standard in Linux.
 *
 */

static u32 adin_mdio_addr_xlate(int devad, u16 regnum)
{
	u32 addr = MII_ADDR_C45 | (devad << 16) | (regnum & 0xffff);

	switch (devad) {
	/**
	 * BASE-T1L auto-negotiation regs start at 0x0200.
	 * The Control & Status bitfields are similar to standard regs.
	 */
	case MDIO_MMD_AN:
		switch (regnum) {
		case MDIO_STAT1:
		case MDIO_CTRL1:
			addr |= 0x0200;
			break;
		}
	}

	return addr;
}

static int adin_read_mmd(struct phy_device *phydev, int devad, u16 regnum)
{
	u32 addr = adin_mdio_addr_xlate(devad, regnum);

	return __mdiobus_read(phydev->mdio.bus, phydev->mdio.addr, addr);
}

static int adin_write_mmd(struct phy_device *phydev, int devad, u16 regnum,
			  u16 val)
{
	u32 addr = adin_mdio_addr_xlate(devad, regnum);

	return __mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, addr, val);
}

static int adin_match_phy_device(struct phy_device *phydev)
{
	u32 id;
	int rc;

	/**
	 * Need to call adin_read_mmd() directly here, because at this point
	 * in time, the driver isn't attached to the PHY device.
	 */
	rc = adin_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_DEVID1);
	if (rc < 0)
		return rc;

	id = rc << 16;

	rc = adin_read_mmd(phydev, MDIO_MMD_PMAPMD, MDIO_DEVID2);
	if (rc < 0)
		return rc;

	id |= rc;

	switch (id) {
	case PHY_ID_ADIN1100:
		return 1;
	default:
		return 0;
	}
}

static u32 adin_mii_adv_m_to_ethtool_adv_t(u32 adv)
{
	u32 result = 0;

	if (adv & ADIN_AN_LP_ADV_B10L)
		result |= ADVERTISED_10baseT_Full;
	if (adv & ADIN_AN_LP_ADV_B1000) {
		result |= ADVERTISED_1000baseT_Half;
		result |= ADVERTISED_1000baseT_Full;
	}
	if (adv & ADIN_AN_LP_ADV_B10S_FD)
		result |= ADVERTISED_10baseT_Full;
	if (adv & ADIN_AN_LP_ADV_B100)
		result |= ADVERTISED_100baseT_Full;

	return result;
}

static inline u32 adin_mii_adv_h_to_ethtool_adv_t(u32 adv)
{
	u32 result = 0;

	if (adv & ADIN_AN_LP_ADV_B10S_HD)
		result |= ADVERTISED_10baseT_Half;

	return result;
}

static int adin_read_lpa(struct phy_device *phydev)
{
	int val;

	phydev->lp_advertising = 0;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, MDIO_STAT1);
	if (val < 0)
		return val;

	if (!(val & MDIO_AN_STAT1_COMPLETE)) {
                phydev->pause = 0;
                phydev->asym_pause = 0;

                return 0;
        }

	phydev->lp_advertising |= ADVERTISED_Autoneg;

	/* Read the link partner's base page advertisement */
	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_L);
	if (val < 0)
		return val;

	phydev->pause = val & LPA_PAUSE_CAP ? 1 : 0;
	phydev->asym_pause = val & LPA_PAUSE_ASYM ? 1 : 0;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_M);
	if (val < 0)
		return val;

	phydev->lp_advertising |= adin_mii_adv_m_to_ethtool_adv_t(val);

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_H);
	if (val < 0)
		return val;

	phydev->lp_advertising |= adin_mii_adv_h_to_ethtool_adv_t(val);

	return 0;
}

static int adin_read_status(struct phy_device *phydev)
{
	int ret;

	ret = genphy_c45_read_link(phydev);
	if (ret)
		return ret;

	phydev->speed = SPEED_UNKNOWN;
	phydev->duplex = DUPLEX_UNKNOWN;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	if (phydev->autoneg == AUTONEG_ENABLE) {
		ret = adin_read_lpa(phydev);
		if (ret)
			return ret;

		phy_resolve_aneg_linkmode(phydev);
	} else {
		/* Only one mode & duplex supported */
		phydev->lp_advertising = 0;
		phydev->speed = SPEED_10;
		phydev->duplex = DUPLEX_FULL;
	}

	return ret;
}

static int adin_config_aneg(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;
	int ret;

	/**
	 * FIXME: don't allow autonegotiation disable for now; link will stop
	 * working. Only 10 Mbps full duplex is supported. And the bits
	 * for forcing a link-speed on C45 are not quite ready for this
	 * version of Linux (4.19.0).
	 */
	if (phydev->autoneg == AUTONEG_DISABLE)
		return 0;

	if (priv->tx_level_24v)
		ret = phy_set_bits_mmd(phydev, MDIO_MMD_AN,
				       ADIN_AN_ADV_ABILITY_H,
				       ADIN_AN_ADV_B10L_TX_LVL_HI_ABL |
				       ADIN_AN_ADV_B10L_TX_LVL_HI_REQ);
	else
		ret = phy_clear_bits_mmd(phydev, MDIO_MMD_AN,
					 ADIN_AN_ADV_ABILITY_H,
					 ADIN_AN_ADV_B10L_TX_LVL_HI_ABL |
					 ADIN_AN_ADV_B10L_TX_LVL_HI_REQ);

	if (ret < 0)
		return ret;

	return genphy_c45_check_and_restart_aneg(phydev, true);
}

static int adin_aneg_done(struct phy_device *phydev)
{
	bool tx_level_24v;
	bool lp_tx_level_24v;
	int val, mask;
	int status;

	status = genphy_c45_aneg_done(phydev);
	if (status < 0)
		return status;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_LP_ADV_ABILITY_H);
	if (val < 0)
		return val;

	mask = ADIN_AN_LP_ADV_B10L_TX_LVL_HI_ABL | ADIN_AN_LP_ADV_B10L_TX_LVL_HI_REQ;
	lp_tx_level_24v = (val & mask) == mask;

	val = phy_read_mmd(phydev, MDIO_MMD_AN, ADIN_AN_ADV_ABILITY_H);
	if (val < 0)
		return val;

	mask = ADIN_AN_ADV_B10L_TX_LVL_HI_ABL | ADIN_AN_ADV_B10L_TX_LVL_HI_REQ;
	tx_level_24v = (val & mask) == mask;

	if (tx_level_24v && lp_tx_level_24v)
		phydev_info(phydev, "Negotiated 2.4V TX level\n");
	else
		phydev_info(phydev, "Negotiated 1.0V TX level\n");

	return status;
}

static int adin_set_powerdown_mode(struct phy_device *phydev, bool en)
{
	int ret, timeout;
	u16 val;

	if (en)
		val = ADIN_CRSM_SFT_PD_CNTRL_EN;
	else
		val = 0;

	ret = phy_write_mmd(phydev, MDIO_MMD_VEND1,
			    ADIN_CRSM_SFT_PD_CNTRL, val);
	if (ret < 0)
		return ret;

	timeout = 30;
	while (timeout-- > 0) {
		ret = phy_read_mmd(phydev, MDIO_MMD_VEND1,
				   ADIN_CRSM_STAT);
		if (ret < 0)
			return ret;

		if ((ret & ADIN_CRSM_SFT_PD_RDY) == val)
			return 0;

		msleep(1);
	}

	return -ETIMEDOUT;
}

static int adin_suspend(struct phy_device *phydev)
{
	return adin_set_powerdown_mode(phydev, true);
}

static int adin_resume(struct phy_device *phydev)
{
	return adin_set_powerdown_mode(phydev, false);
}

static int adin_set_loopback(struct phy_device *phydev, bool enable)
{
	int ret;

	if (enable)
		return phy_set_bits_mmd(phydev, MDIO_MMD_PCS,
					ADIN_B10L_PCS_CNTRL,
					ADIN_PCS_CNTRL_B10L_LB_PCS_EN);

	/* MAC interface block loopback */
	ret = phy_clear_bits_mmd(phydev, MDIO_MMD_VEND1, ADIN_MAC_IF_LOOPBACK,
				 ADIN_MAC_IF_LOOPBACK_EN |
				 ADIN_MAC_IF_REMOTE_LOOPBACK_EN);
	if (ret < 0)
		return ret;

	/* PCS loopback (according to 10BASE-T1L spec) */
	ret = phy_clear_bits_mmd(phydev, MDIO_MMD_PCS, ADIN_B10L_PCS_CNTRL,
				 ADIN_PCS_CNTRL_B10L_LB_PCS_EN);
	if (ret < 0)
		return ret;

	/* PMA loopback (according to 10BASE-T1L spec) */
	return phy_clear_bits_mmd(phydev, MDIO_MMD_PMAPMD, ADIN_B10L_PMA_CNTRL,
				  ADIN_PMA_CNTRL_B10L_LB_PMA_LOC_EN);
}

static int adin_config_init(struct phy_device *phydev)
{
	struct adin_priv *priv = phydev->priv;
	struct device *dev = &phydev->mdio.dev;
	int ret;

	phydev->supported = SUPPORTED_10baseT_Full;
	phydev->advertising = SUPPORTED_10baseT_Full;

	ret = phy_read_mmd(phydev, MDIO_MMD_PMAPMD, ADIN_B10L_PMA_STAT);
	if (ret < 0)
		return ret;

	/* This depends on the voltage level from the power source */
	priv->tx_level_24v = !!(ret & ADIN_PMA_STAT_B10L_TX_LVL_HI_ABLE);

	phydev_dbg(phydev, "PHY supports 2.4V TX level: %s\n",
		   priv->tx_level_24v ? "yes" : "no");

	if (priv->tx_level_24v &&
	    device_property_present(dev, "adi,disable-2-4-v-tx-level")) {
		phydev_info(phydev,
			    "PHY supports 2.4V TX level, but disabled via config\n");
		priv->tx_level_24v = 0;
	}

	return 0;
}

static int adin_probe(struct phy_device *phydev)
{
	struct device *dev = &phydev->mdio.dev;
	struct adin_priv *priv;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	phydev->priv = priv;

	return 0;
}

static struct phy_driver adin_driver[] = {
	{
		PHY_ID_MATCH_MODEL(PHY_ID_ADIN1100),
		.name			= "ADIN1100",
		.features		= ADIN_T1L_FEATURES, /* FIXME: remove this when the `get_features` hook becomes available */
		.match_phy_device	= adin_match_phy_device,
		.probe			= adin_probe,
		.config_init		= adin_config_init,
		.config_aneg		= adin_config_aneg,
		.aneg_done		= adin_aneg_done,
		.read_status		= adin_read_status,
		.read_mmd		= adin_read_mmd,
		.write_mmd		= adin_write_mmd,
		.set_loopback		= adin_set_loopback,
		.suspend		= adin_suspend,
		.resume			= adin_resume,
	},
};

module_phy_driver(adin_driver);

static struct mdio_device_id __maybe_unused adin_tbl[] = {
	{ PHY_ID_MATCH_MODEL(PHY_ID_ADIN1100) },
	{ }
};

MODULE_DEVICE_TABLE(mdio, adin_tbl);
MODULE_DESCRIPTION("Analog Devices Industrial Ethernet T1L PHY driver");
MODULE_LICENSE("Dual BSD/GPL");
