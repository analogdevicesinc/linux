// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2021 NXP */

#include <linux/acpi.h>
#include <linux/property.h>
#include <linux/fsl/mc.h>
#include <linux/msi.h>
#include "dpaa2-eth.h"
#include "dpaa2-mac.h"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("Freescale DPAA2 MAC Driver");

struct dpaa2_mac_link_mode_map {
	u64 dpmac_lm;
	enum ethtool_link_mode_bit_indices ethtool_lm;
};

static const struct dpaa2_mac_link_mode_map dpaa2_mac_lm_map[] = {
	{DPMAC_ADVERTISED_10BASET_FULL, ETHTOOL_LINK_MODE_10baseT_Full_BIT},
	{DPMAC_ADVERTISED_100BASET_FULL, ETHTOOL_LINK_MODE_100baseT_Full_BIT},
	{DPMAC_ADVERTISED_1000BASET_FULL, ETHTOOL_LINK_MODE_1000baseT_Full_BIT},
	{DPMAC_ADVERTISED_10000BASET_FULL, ETHTOOL_LINK_MODE_10000baseT_Full_BIT},
	{DPMAC_ADVERTISED_AUTONEG, ETHTOOL_LINK_MODE_Autoneg_BIT},
};

static void dpaa2_mac_ksettings_change(struct dpaa2_mac *priv)
{
	struct fsl_mc_device *mc_dev = priv->mc_dev;
	struct dpmac_link_cfg link_cfg = { 0 };
	int err, i;

	err = dpmac_get_link_cfg(priv->mc_io, 0,
				 mc_dev->mc_handle,
				 &link_cfg);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_get_link_cfg() = %d\n", err);
		return;
	}

	/* There are some circumstances (MC bugs) when the advertising is all
	 * zeroes. Just skip any configuration if this happens.
	 */
	if (!link_cfg.advertising)
		return;

	phylink_ethtool_ksettings_get(priv->phylink, &priv->kset);

	priv->kset.base.speed = link_cfg.rate;
	priv->kset.base.duplex = !!(link_cfg.options & DPMAC_LINK_OPT_HALF_DUPLEX);

	ethtool_link_ksettings_zero_link_mode(&priv->kset, advertising);
	for (i = 0; i < ARRAY_SIZE(dpaa2_mac_lm_map); i++) {
		if (link_cfg.advertising & dpaa2_mac_lm_map[i].dpmac_lm)
			__set_bit(dpaa2_mac_lm_map[i].ethtool_lm,
				  priv->kset.link_modes.advertising);
	}

	if (link_cfg.options & DPMAC_LINK_OPT_AUTONEG) {
		priv->kset.base.autoneg = AUTONEG_ENABLE;
		__set_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			  priv->kset.link_modes.advertising);
	} else {
		priv->kset.base.autoneg = AUTONEG_DISABLE;
		__clear_bit(ETHTOOL_LINK_MODE_Autoneg_BIT,
			    priv->kset.link_modes.advertising);
	}

	phylink_ethtool_ksettings_set(priv->phylink, &priv->kset);
}

static irqreturn_t dpaa2_mac_irq_handler(int irq_num, void *arg)
{
	struct device *dev = (struct device *)arg;
	struct fsl_mc_device *dpmac_dev;
	struct net_device *net_dev;
	struct dpaa2_mac *priv;
	u32 status = ~0;
	int err;

	dpmac_dev = to_fsl_mc_device(dev);
	net_dev = dev_get_drvdata(dev);
	priv = netdev_priv(net_dev);

	err = dpmac_get_irq_status(priv->mc_io, 0, dpmac_dev->mc_handle,
				  DPMAC_IRQ_INDEX, &status);
	if (err) {
		netdev_err(net_dev, "dpmac_get_irq_status() = %d\n", err);
		return IRQ_HANDLED;
	}

	rtnl_lock();
	if (status & DPMAC_IRQ_EVENT_LINK_CFG_REQ)
		dpaa2_mac_ksettings_change(priv);

	if (status & DPMAC_IRQ_EVENT_LINK_DOWN_REQ) {
		if (priv->phy_req_state) {
			phylink_stop(priv->phylink);
			priv->phy_req_state = 0;
		}
	}

	if (status & DPMAC_IRQ_EVENT_LINK_UP_REQ) {
		if (!priv->phy_req_state) {
			priv->phy_req_state = 1;
			phylink_start(priv->phylink);
		}
	}
	rtnl_unlock();

	dpmac_clear_irq_status(priv->mc_io, 0, dpmac_dev->mc_handle,
			       DPMAC_IRQ_INDEX, status);

	return IRQ_HANDLED;
}

static int dpaa2_mac_setup_irqs(struct fsl_mc_device *mc_dev)
{
	struct fsl_mc_device_irq *irq;
	int err = 0;

	err = fsl_mc_allocate_irqs(mc_dev);
	if (err) {
		dev_err(&mc_dev->dev, "fsl_mc_allocate_irqs err %d\n", err);
		return err;
	}

	irq = mc_dev->irqs[0];
	err = devm_request_threaded_irq(&mc_dev->dev, irq->virq,
					NULL, &dpaa2_mac_irq_handler,
					IRQF_NO_SUSPEND | IRQF_ONESHOT,
					dev_name(&mc_dev->dev), &mc_dev->dev);
	if (err) {
		dev_err(&mc_dev->dev, "devm_request_threaded_irq err %d\n",
			err);
		goto free_irq;
	}

	err = dpmac_set_irq_mask(mc_dev->mc_io, 0, mc_dev->mc_handle,
				 DPMAC_IRQ_INDEX, DPMAC_IRQ_EVENT_LINK_CFG_REQ |
				 DPMAC_IRQ_EVENT_LINK_UP_REQ |
				 DPMAC_IRQ_EVENT_LINK_DOWN_REQ);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_set_irq_mask err %d\n", err);
		goto free_irq;
	}
	err = dpmac_set_irq_enable(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, 1);
	if (err) {
		dev_err(&mc_dev->dev, "dpmac_set_irq_enable err %d\n", err);
		goto free_irq;
	}

	return 0;

free_irq:
	fsl_mc_free_irqs(mc_dev);

	return err;
}

static void dpaa2_mac_teardown_irqs(struct fsl_mc_device *mc_dev)
{
	int err;

	err = dpmac_set_irq_enable(mc_dev->mc_io, 0, mc_dev->mc_handle,
				   DPMAC_IRQ_INDEX, 0);
	if (err)
		dev_err(&mc_dev->dev, "dpmac_set_irq_enable err %d\n", err);

	fsl_mc_free_irqs(mc_dev);
}

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS

static int dpaa2_mac_netdev_open(struct net_device *net_dev)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return 0;

	if (priv->phy_req_state)
		return 0;

	priv->phy_req_state = 1;
	phylink_start(priv->phylink);
	return 0;
}

static int dpaa2_mac_netdev_stop(struct net_device *net_dev)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return 0;
	if (!priv->phy_req_state)
		return 0;

	priv->phy_req_state = 0;
	phylink_stop(priv->phylink);

	return 0;
}

static netdev_tx_t dpaa2_mac_drop_frame(struct sk_buff *skb,
					struct net_device *net_dev)
{
	/* These interfaces don't support I/O, they are only
	 * for control and debu
	 */
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static void dpaa2_mac_get_drvinfo(struct net_device *net_dev,
				  struct ethtool_drvinfo *drvinfo)
{
	strscpy(drvinfo->driver, KBUILD_MODNAME, sizeof(drvinfo->driver));
	strscpy(drvinfo->bus_info, dev_name(net_dev->dev.parent->parent),
		sizeof(drvinfo->bus_info));
}

static int dpaa2_mac_get_link_ksettings(struct net_device *net_dev,
					struct ethtool_link_ksettings *ks)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (dpaa2_mac_is_type_phy(priv))
		return phylink_ethtool_ksettings_get(priv->phylink, ks);

	return -EOPNOTSUPP;
}

static int dpaa2_mac_set_link_ksettings(struct net_device *net_dev,
					const struct ethtool_link_ksettings *ks)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return -EOPNOTSUPP;

	return phylink_ethtool_ksettings_set(priv->phylink, ks);
}

static int dpaa2_mac_ioctl(struct net_device *net_dev, struct ifreq *rq, int cmd)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	if (!dpaa2_mac_is_type_phy(priv))
		return -EOPNOTSUPP;

	return phylink_mii_ioctl(priv->phylink, rq, cmd);
}

static void dpaa2_mac_ethtool_get_strings(struct net_device *net_dev,
					  u32 stringset, u8 *data)
{
	if (stringset != ETH_SS_STATS)
		return;

	dpaa2_mac_get_strings(data);
}

static void dpaa2_mac_ethtool_get_stats(struct net_device *net_dev,
					struct ethtool_stats *stats,
					u64 *data)
{
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	dpaa2_mac_get_ethtool_stats(priv, data);
}

static int dpaa2_mac_ethtool_get_sset_count(struct net_device *dev, int sset)
{
	if (sset != ETH_SS_STATS)
		return -EOPNOTSUPP;

	return dpaa2_mac_get_sset_count();
}

static const struct net_device_ops dpaa2_mac_ndo_ops = {
	.ndo_open		= &dpaa2_mac_netdev_open,
	.ndo_stop		= &dpaa2_mac_netdev_stop,
	.ndo_start_xmit		= &dpaa2_mac_drop_frame,
	.ndo_eth_ioctl		= &dpaa2_mac_ioctl,
};

static const struct ethtool_ops dpaa2_mac_ethtool_ops = {
	.get_drvinfo		= &dpaa2_mac_get_drvinfo,
	.get_link_ksettings	= &dpaa2_mac_get_link_ksettings,
	.set_link_ksettings	= &dpaa2_mac_set_link_ksettings,
	.get_strings		= &dpaa2_mac_ethtool_get_strings,
	.get_ethtool_stats	= &dpaa2_mac_ethtool_get_stats,
	.get_sset_count		= &dpaa2_mac_ethtool_get_sset_count,
};
#endif

static int dpaa2_mac_probe(struct fsl_mc_device *mc_dev)
{
	struct device *dev = &mc_dev->dev;
	struct fsl_mc_device *peer_dev;
	struct net_device *net_dev;
	struct dpaa2_mac *priv;
	int err;

	/* If the DPMAC is connected to a DPNI from the currect DPRC, then
	 * there is no need for this standalone MAC driver, the dpaa2-eth will
	 * take care of anything related to MAC/PHY
	 */
	peer_dev = fsl_mc_get_endpoint(mc_dev, 0);
	if (!IS_ERR_OR_NULL(peer_dev) && peer_dev->dev.type == &fsl_mc_bus_dpni_type)
		return -EPROBE_DEFER;
	if (!IS_ERR_OR_NULL(peer_dev) && peer_dev->dev.type == &fsl_mc_bus_dpsw_type)
		return -EPROBE_DEFER;

	net_dev = alloc_etherdev(sizeof(*priv));
	if (!net_dev) {
		dev_err(dev, "alloc_etherdev error\n");
		return -ENOMEM;
	}

	priv = netdev_priv(net_dev);
	priv->mc_dev = mc_dev;
	priv->net_dev = net_dev;
	priv->phy_req_state = 0;

	SET_NETDEV_DEV(net_dev, dev);

#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	snprintf(net_dev->name, IFNAMSIZ, "mac%d", mc_dev->obj_desc.id);

	/* register netdev just to make it visible to the user */
	net_dev->netdev_ops = &dpaa2_mac_ndo_ops;
	net_dev->ethtool_ops = &dpaa2_mac_ethtool_ops;

	err = register_netdev(net_dev);
	if (err) {
		dev_err(dev, "register_netdev error %d\n", err);
		goto free_netdev;
	}
#endif

	dev_set_drvdata(dev, net_dev);

	err = fsl_mc_portal_allocate(mc_dev, 0, &mc_dev->mc_io);
	if (err) {
		if (err == -ENXIO)
			err = -EPROBE_DEFER;
		else
			dev_err(dev, "fsl_mc_portal_allocate err %d\n", err);
		goto unregister_netdev;
	}
	priv->mc_io = mc_dev->mc_io;

	err = dpaa2_mac_open(priv);
	if (err)
		goto free_portal;

	err = dpaa2_mac_setup_irqs(mc_dev);
	if (err) {
		err = -EFAULT;
		goto close_mac;
	}

	if (dpaa2_mac_is_type_phy(priv)) {
		err = dpaa2_mac_connect(priv);
		if (err) {
			dev_err_probe(dev, err, "Error connecting to the MAC endpoint\n");
			goto teardown_irqs;
		}
	}

	return 0;

teardown_irqs:
	dpaa2_mac_teardown_irqs(mc_dev);
close_mac:
	dpaa2_mac_close(priv);
free_portal:
	fsl_mc_portal_free(mc_dev->mc_io);
unregister_netdev:
#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	unregister_netdev(net_dev);
free_netdev:
#endif
	free_netdev(net_dev);

	return err;
}

static void dpaa2_mac_remove(struct fsl_mc_device *mc_dev)
{
	struct device *dev = &mc_dev->dev;
	struct net_device *net_dev = dev_get_drvdata(dev);
	struct dpaa2_mac *priv = netdev_priv(net_dev);

	dpaa2_mac_teardown_irqs(mc_dev);

	if (dpaa2_mac_is_type_phy(priv))
		dpaa2_mac_disconnect(priv);
	dpaa2_mac_close(priv);

	fsl_mc_portal_free(mc_dev->mc_io);
	dev_set_drvdata(dev, NULL);
#ifdef CONFIG_FSL_DPAA2_MAC_NETDEVS
	unregister_netdev(net_dev);
#endif
	free_netdev(net_dev);
}

static const struct fsl_mc_device_id dpaa2_mac_match_id_table[] = {
	{
		.vendor = FSL_MC_VENDOR_FREESCALE,
		.obj_type = "dpmac",
	},
	{ .vendor = 0x0 }
};
MODULE_DEVICE_TABLE(fslmc, dpaa2_mac_match_id_table);

static struct fsl_mc_driver dpaa2_mac_driver = {
	.driver = {
		.name = "fsl_dpaa2_mac",
		.owner = THIS_MODULE,
	},
	.probe = dpaa2_mac_probe,
	.remove = dpaa2_mac_remove,
	.match_id_table = dpaa2_mac_match_id_table
};

module_fsl_mc_driver(dpaa2_mac_driver);
