// SPDX-License-Identifier: GPL-2.0
/* Phylib wrapper driver for MoreThanIP copper backplane AN/LT
 * (Auto-Negotiation and Link Training) core
 *
 * Copyright 2023 NXP
 */

#include <linux/kernel.h>
#include <linux/mii.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/phylink.h>
#include <linux/phy/phy.h>

#include "../pcs/mtip_backplane.h"

#define PHY_ID_FSL_MTIP_ANLT			0x0083e400

struct mtip_phy {
	struct mtip_backplane *anlt;
	struct phy *serdes;
	bool suspended;
};

static int mtip_phy_get_features(struct phy_device *phydev)
{
	struct mtip_phy *priv = phydev->priv;

	bitmap_fill(phydev->supported, __ETHTOOL_LINK_MODE_MASK_NBITS);

	return mtip_backplane_validate(priv->serdes, phydev->supported);
}

static int mtip_phy_suspend(struct phy_device *phydev)
{
	struct mtip_phy *priv = phydev->priv;

	if (priv->suspended)
		return 0;

	priv->suspended = true;

	return mtip_backplane_suspend(priv->anlt);
}

static int mtip_phy_resume(struct phy_device *phydev)
{
	struct mtip_phy *priv = phydev->priv;

	if (!priv->suspended)
		return 0;

	priv->suspended = false;

	return mtip_backplane_resume(priv->anlt);
}

static int mtip_phy_config_aneg(struct phy_device *phydev)
{
	bool autoneg = phydev->autoneg == AUTONEG_ENABLE;
	struct mtip_phy *priv = phydev->priv;
	int err;

	/* Only allow advertising what this PHY supports. The device tree
	 * property "max-speed" may further limit the speed and thus the
	 * link modes. Similar to genphy_config_advert().
	 */
	linkmode_and(phydev->advertising, phydev->advertising,
		     phydev->supported);

	err = mtip_backplane_config_aneg(priv->anlt, autoneg,
					 phydev->advertising);
	if (err < 0)
		return err;

	if (err > 0)
		mtip_backplane_an_restart(priv->anlt);

	return 0;
}

static int mtip_phy_read_status(struct phy_device *phydev)
{
	struct phylink_link_state state = {};
	struct mtip_phy *priv = phydev->priv;

	/* mtip_backplane_get_state() tests Autoneg, and phylink_resolve_c73()
	 * tests the link modes
	 */
	linkmode_copy(state.advertising, phydev->advertising);

	mtip_backplane_get_state(priv->anlt, &state);

	phydev->speed = state.speed;
	phydev->duplex = state.duplex;
	phydev->link = state.link;
	phydev->autoneg_complete = state.an_complete;
	linkmode_copy(phydev->lp_advertising, state.lp_advertising);

	if (phydev->autoneg == AUTONEG_ENABLE && phydev->autoneg_complete)
		phy_resolve_aneg_pause(phydev);

	return 0;
}

static int mtip_phy_config_init(struct phy_device *phydev)
{
	return 0;
}

static int mtip_phy_probe(struct phy_device *phydev)
{
	struct mdio_device *mdiodev = &phydev->mdio;
	struct device_node *dn = mdiodev->dev.of_node;
	struct mtip_phy *priv;
	int err;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv) {
		err = -ENOMEM;
		goto out;
	}

	priv->serdes = of_phy_get(dn, NULL);
	if (IS_ERR(priv->serdes)) {
		err = PTR_ERR(priv->serdes);
		goto out_free_priv;
	}

	err = phy_init(priv->serdes);
	if (err) {
		dev_err(&mdiodev->dev,
			"Failed to initialize SerDes: %pe\n",
			ERR_PTR(err));
		goto out_put_serdes;
	}

	priv->anlt = mtip_backplane_create(mdiodev, priv->serdes,
					   MTIP_MODEL_AUTODETECT);
	if (IS_ERR(priv->anlt)) {
		err = PTR_ERR(priv->anlt);
		goto out_exit_serdes;
	}

	priv->suspended = true;

	phydev->priv = priv;

	return 0;

out_exit_serdes:
	phy_exit(priv->serdes);
out_put_serdes:
	of_phy_put(priv->serdes);
out_free_priv:
	kfree(priv);
out:
	return err;
}

static void mtip_phy_remove(struct phy_device *phydev)
{
	struct mtip_phy *priv = phydev->priv;

	mtip_backplane_destroy(priv->anlt);
	phy_exit(priv->serdes);
	of_phy_put(priv->serdes);
	kfree(priv);
}

static struct phy_driver mtip_backplane_driver[] = {
	{
		PHY_ID_MATCH_MODEL(PHY_ID_FSL_MTIP_ANLT),
		.flags			= PHY_IS_INTERNAL,
		.name			= "MTIP AN/LT",
		.probe			= mtip_phy_probe,
		.remove			= mtip_phy_remove,
		.get_features		= mtip_phy_get_features,
		.suspend		= mtip_phy_suspend,
		.resume			= mtip_phy_resume,
		.config_aneg		= mtip_phy_config_aneg,
		.read_status		= mtip_phy_read_status,
		.config_init		= mtip_phy_config_init,
	},
};

module_phy_driver(mtip_backplane_driver);

static struct mdio_device_id __maybe_unused mtip_tbl[] = {
	{ PHY_ID_MATCH_MODEL(PHY_ID_FSL_MTIP_ANLT) },
	{ },
};

MODULE_DEVICE_TABLE(mdio, mtip_tbl);

MODULE_AUTHOR("Vladimir Oltean <vladimir.oltean@nxp.com>");
MODULE_DESCRIPTION("MTIP Backplane phylib wrapper");
MODULE_LICENSE("GPL");
