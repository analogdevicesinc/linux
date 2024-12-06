// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2023 NXP */
#include <linux/if_vlan.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/pcs-lynx.h>
#include <linux/phy/phy.h>
#include <linux/pcs/pcs-xpcs.h>
#include <linux/regulator/consumer.h>

#include "enetc_pf.h"

void enetc_get_ip_revision(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	u32 val;

	val = enetc_global_rd(hw, ENETC_G_EIPBRR0);
	si->revision = val & EIPBRR0_REVISION;
}

static int enetc_set_si_hw_addr(struct enetc_pf *pf, int si, u8 *mac_addr)
{
	struct enetc_hw *hw = &pf->si->hw;

	if (pf->hw_ops->set_si_primary_mac)
		pf->hw_ops->set_si_primary_mac(hw, si, mac_addr);
	else
		return -EOPNOTSUPP;

	return 0;
}

int enetc_pf_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct sockaddr *saddr = addr;
	int err;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	err = enetc_set_si_hw_addr(pf, 0, saddr->sa_data);
	if (err)
		return err;

	eth_hw_addr_set(ndev, saddr->sa_data);

	return 0;
}

static int enetc_setup_mac_address(struct device_node *np, struct enetc_pf *pf,
				   int si)
{
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_hw *hw = &pf->si->hw;
	u8 mac_addr[ETH_ALEN] = { 0 };
	int err;

	/* (1) try to get the MAC address from the device tree */
	if (np) {
		err = of_get_mac_address(np, mac_addr);
		if (err == -EPROBE_DEFER)
			return err;
	}

	/* (2) bootloader supplied MAC address */
	if (is_zero_ether_addr(mac_addr) && pf->hw_ops->get_si_primary_mac)
		pf->hw_ops->get_si_primary_mac(hw, si, mac_addr);

	/* (3) choose a random one */
	if (is_zero_ether_addr(mac_addr)) {
		eth_random_addr(mac_addr);
		dev_info(dev, "no MAC address specified for SI%d, using %pM\n",
			 si, mac_addr);
	}

	err = enetc_set_si_hw_addr(pf, si, mac_addr);
	if (err)
		return err;

	if (!si)
		memcpy(pf->mac_addr_base, mac_addr, ETH_ALEN);

	return 0;
}

int enetc_setup_mac_addresses(struct device_node *np, struct enetc_pf *pf)
{
	int err, i;

	/* The PF might take its MAC from the device tree */
	err = enetc_setup_mac_address(np, pf, 0);
	if (err)
		return err;

	for (i = 0; i < pf->total_vfs; i++) {
		if (is_enetc_rev1(pf->si)) {
			err = enetc_setup_mac_address(NULL, pf, i + 1);
		} else {
			u8 mac_addr[ETH_ALEN];

			memcpy(mac_addr, pf->mac_addr_base, ETH_ALEN);
			eth_addr_add(mac_addr, i + 1);
			if (!is_valid_ether_addr(mac_addr)) {
				eth_random_addr(mac_addr);
				dev_info(&pf->si->pdev->dev,
					 "SI%d: Invalid MAC addr, using %pM\n",
					 i + 1, mac_addr);
			}
			err = enetc_set_si_hw_addr(pf, i + 1, mac_addr);
		}
		if (err)
			return err;
	}

	return 0;
}

static void enetc_set_si_vlan_promisc(struct enetc_pf *pf, int index, bool en)
{
	struct enetc_hw *hw = &pf->si->hw;

	if (en)
		pf->vlan_promisc_simap |= BIT(index);
	else
		pf->vlan_promisc_simap &= ~BIT(index);

	if (pf->hw_ops->set_si_vlan_promisc)
		pf->hw_ops->set_si_vlan_promisc(hw, pf->vlan_promisc_simap);
}

int enetc_vlan_rx_add_vid(struct net_device *ndev, __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	struct enetc_pf *pf;
	int idx;

	pf = enetc_si_priv(si);

	__set_bit(vid, si->active_vlans);

	idx = enetc_vid_hash_idx(vid);
	if (!__test_and_set_bit(idx, si->vlan_ht_filter) &&
	    pf->hw_ops->set_si_vlan_hash_filter)
		pf->hw_ops->set_si_vlan_hash_filter(hw, 0, *si->vlan_ht_filter);

	return 0;
}

int enetc_vlan_rx_del_vid(struct net_device *ndev, __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	struct enetc_pf *pf;
	int idx;

	pf = enetc_si_priv(si);

	if (__test_and_clear_bit(vid, si->active_vlans)) {
		idx = enetc_vid_hash_idx(vid);
		enetc_refresh_vlan_ht_filter(si);
		if (!test_bit(idx, si->vlan_ht_filter) &&
		    pf->hw_ops->set_si_vlan_hash_filter)
			pf->hw_ops->set_si_vlan_hash_filter(hw, 0, *si->vlan_ht_filter);
	}

	return 0;
}

int enetc_pf_set_vf_mac(struct net_device *ndev, int vf, u8 *mac)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_vf_state *vf_state;
	int err;

	if (vf >= pf->total_vfs)
		return -EINVAL;

	if (!is_valid_ether_addr(mac))
		return -EADDRNOTAVAIL;

	vf_state = &pf->vf_state[vf];
	vf_state->flags |= ENETC_VF_FLAG_PF_SET_MAC;

	err = enetc_set_si_hw_addr(pf, vf + 1, mac);
	if (err)
		return err;

	return 0;
}

int enetc_pf_set_vf_vlan(struct net_device *ndev, int vf, u16 vlan,
			 u8 qos, __be16 proto)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &pf->si->hw;

	if (priv->si->errata & ENETC_ERR_VLAN_ISOL)
		return -EOPNOTSUPP;

	if (vf >= pf->total_vfs || vlan >= VLAN_N_VID || qos > 7)
		return -EINVAL;

	if (proto != htons(ETH_P_8021Q))
		/* only C-tags supported for now */
		return -EPROTONOSUPPORT;

	if (pf->hw_ops->set_si_based_vlan)
		pf->hw_ops->set_si_based_vlan(hw, vf + 1, vlan, qos);
	else
		return -EOPNOTSUPP;

	return 0;
}

int enetc_pf_set_vf_spoofchk(struct net_device *ndev, int vf, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &pf->si->hw;

	if (vf >= pf->total_vfs)
		return -EINVAL;

	if (pf->hw_ops->set_si_anti_spoofing)
		pf->hw_ops->set_si_anti_spoofing(hw, vf + 1, en);
	else
		return -EOPNOTSUPP;

	return 0;
}

int enetc_pf_set_vf_trust(struct net_device *ndev, int vf, bool setting)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_vf_state *vf_state;

	if (vf >= pf->num_vfs)
		return -EINVAL;

	vf_state = &pf->vf_state[vf];

	if (setting)
		vf_state->flags |= ENETC_VF_FLAG_TRUSTED;
	else
		vf_state->flags &= ~ENETC_VF_FLAG_TRUSTED;

	return 0;
}

int enetc_pf_get_vf_config(struct net_device *ndev, int vf,
			   struct ifla_vf_info *ivi)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_hw *hw = &pf->si->hw;
	struct enetc_vf_state *vf_state;

	if (vf >= pf->num_vfs)
		return -EINVAL;

	vf_state = &pf->vf_state[vf];

	ivi->vf = vf;

	if (pf->hw_ops->get_si_primary_mac)
		pf->hw_ops->get_si_primary_mac(hw, vf + 1, ivi->mac);

	if (pf->hw_ops->get_si_based_vlan)
		pf->hw_ops->get_si_based_vlan(hw, vf + 1, &ivi->vlan,
					      &ivi->qos);

	ivi->trusted = !!(vf_state->flags & ENETC_VF_FLAG_TRUSTED);

	return 0;
}

int enetc_pf_set_features(struct net_device *ndev, netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	int err;

	if (changed & NETIF_F_HW_TC) {
		err = enetc_set_tc_flower(ndev, !!(features & NETIF_F_HW_TC));
		if (err)
			return err;
	}

	if (changed & NETIF_F_HW_VLAN_CTAG_FILTER) {
		struct enetc_pf *pf = enetc_si_priv(priv->si);
		bool en = !(features & NETIF_F_HW_VLAN_CTAG_FILTER);

		enetc_set_si_vlan_promisc(pf, 0, en);
	}

	if (changed & NETIF_F_LOOPBACK && pf->hw_ops->set_loopback)
		pf->hw_ops->set_loopback(ndev, !!(features & NETIF_F_LOOPBACK));

	enetc_set_features(ndev, features);

	return 0;
}

int enetc_pf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
		      void *type_data)
{
	switch (type) {
	case TC_QUERY_CAPS:
		return enetc_qos_query_caps(ndev, type_data);
	case TC_SETUP_QDISC_MQPRIO:
		return enetc_setup_tc_mqprio(ndev, type_data);
	case TC_SETUP_QDISC_TAPRIO:	/* time aware priority shaper */
		return enetc_setup_tc_taprio(ndev, type_data);
	case TC_SETUP_QDISC_CBS:
		return enetc_setup_tc_cbs(ndev, type_data);
	case TC_SETUP_QDISC_ETF:	/* earliest txtime first */
		return enetc_setup_tc_txtime(ndev, type_data);
	case TC_SETUP_BLOCK:
		return enetc_setup_tc_psfp(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

void enetc_pf_netdev_setup(struct enetc_si *si, struct net_device *ndev,
			   const struct net_device_ops *ndev_ops)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_WOL << 1) - 1;
	ndev->netdev_ops = ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;

	ndev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM |
			    NETIF_F_HW_VLAN_CTAG_TX | NETIF_F_HW_VLAN_CTAG_RX |
			    NETIF_F_HW_VLAN_CTAG_FILTER | NETIF_F_LOOPBACK |
			    NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 |
			    NETIF_F_GSO_UDP_L4;
	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG | NETIF_F_RXCSUM |
			 NETIF_F_HW_VLAN_CTAG_TX |
			 NETIF_F_HW_VLAN_CTAG_RX |
			 NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 |
			 NETIF_F_GSO_UDP_L4;
	ndev->vlan_features = NETIF_F_SG | NETIF_F_HW_CSUM |
			      NETIF_F_TSO | NETIF_F_TSO6 | NETIF_F_GSO_UDP_L4;

	if (si->num_rss) {
		ndev->hw_features |= NETIF_F_RXHASH;
		ndev->features |= NETIF_F_RXHASH;
	}

	/* If driver handles unicast address filtering, it should set
	 * IFF_UNICAST_FLT in its priv_flags. (Refer to the description
	 * of the ndo_set_rx_mode())
	 */
	ndev->priv_flags |= IFF_UNICAST_FLT;
	ndev->xdp_features = NETDEV_XDP_ACT_BASIC | NETDEV_XDP_ACT_REDIRECT |
			     NETDEV_XDP_ACT_NDO_XMIT | NETDEV_XDP_ACT_RX_SG |
			     NETDEV_XDP_ACT_NDO_XMIT_SG;

	if (is_enetc_rev1(si)) {
		ndev->max_mtu = ENETC_MAX_MTU;
		priv->max_frags_bd = ENETC_MAX_SKB_FRAGS;
	} else {
		ndev->max_mtu = ENETC4_MAX_MTU;
		priv->max_frags_bd = ENETC4_MAX_SKB_FRAGS;
		priv->active_offloads |= ENETC_F_CHECKSUM;
		priv->shared_tx_rings = true;
	}

	if (si->hw_features & ENETC_SI_F_RSC)
		ndev->hw_features |= NETIF_F_LRO;

	if (si->hw_features & ENETC_SI_F_LSO)
		priv->active_offloads |= ENETC_F_LSO;

	if (si->hw_features & ENETC_SI_F_PSFP && !enetc_set_tc_flower(ndev, true)) {
		ndev->features |= NETIF_F_HW_TC;
		ndev->hw_features |= NETIF_F_HW_TC;
	}

	/* pick up primary MAC address from SI */
	enetc_load_primary_mac_addr(&si->hw, ndev);
}

static int enetc_mdio_probe(struct enetc_pf *pf, struct device_node *np)
{
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_mdio_priv *mdio_priv;
	struct mii_bus *bus;
	int err;

	bus = devm_mdiobus_alloc_size(dev, sizeof(*mdio_priv));
	if (!bus)
		return -ENOMEM;

	bus->name = "Freescale ENETC MDIO Bus";
	bus->read = enetc_mdio_read_c22;
	bus->write = enetc_mdio_write_c22;
	bus->read_c45 = enetc_mdio_read_c45;
	bus->write_c45 = enetc_mdio_write_c45;
	bus->parent = dev;
	mdio_priv = bus->priv;
	mdio_priv->hw = &pf->si->hw;
	if (is_enetc_rev4(pf->si))
		mdio_priv->mdio_base = ENETC4_EMDIO_BASE;
	else
		mdio_priv->mdio_base = ENETC_EMDIO_BASE;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s", dev_name(dev));

	err = of_mdiobus_register(bus, np);
	if (err)
		return dev_err_probe(dev, err, "cannot register MDIO bus\n");

	pf->mdio = bus;

	return 0;
}

static void enetc_mdio_remove(struct enetc_pf *pf)
{
	if (pf->mdio)
		mdiobus_unregister(pf->mdio);
}

static int enetc_imdio_create(struct enetc_pf *pf)
{
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_mdio_priv *mdio_priv;
	struct phylink_pcs *phylink_pcs;
	struct mii_bus *bus;
	struct phy *serdes;
	size_t num_phys;
	int err;

	serdes = devm_of_phy_optional_get(dev, dev->of_node, NULL);
	if (IS_ERR(serdes))
		return PTR_ERR(serdes);

	num_phys = serdes ? 1 : 0;

	bus = mdiobus_alloc_size(sizeof(*mdio_priv));
	if (!bus)
		return -ENOMEM;

	bus->name = "Freescale ENETC internal MDIO Bus";
	bus->read = enetc_mdio_read_c22;
	bus->write = enetc_mdio_write_c22;
	bus->read_c45 = enetc_mdio_read_c45;
	bus->write_c45 = enetc_mdio_write_c45;
	bus->parent = dev;
	bus->phy_mask = ~0;
	mdio_priv = bus->priv;
	mdio_priv->hw = &pf->si->hw;
	if (is_enetc_rev4(pf->si))
		mdio_priv->mdio_base = ENETC4_PM_IMDIO_BASE;
	else
		mdio_priv->mdio_base = ENETC_PM_IMDIO_BASE;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-imdio", dev_name(dev));

	mdio_priv->regulator = devm_regulator_get_optional(dev, "serdes");
	if (IS_ERR(mdio_priv->regulator)) {
		err = PTR_ERR(mdio_priv->regulator);
		if (err == -EPROBE_DEFER)
			goto free_mdio_bus;
		mdio_priv->regulator = NULL;
	}

	if (mdio_priv->regulator) {
		err = regulator_enable(mdio_priv->regulator);
		if (err) {
			dev_err(dev, "fail to enable phy-supply\n");
			goto free_mdio_bus;
		}
	}

	err = mdiobus_register(bus);
	if (err) {
		dev_err(dev, "cannot register internal MDIO bus (%d)\n", err);
		goto free_mdio_bus;
	}

	if (is_enetc_rev1(pf->si)) {
		phylink_pcs = lynx_pcs_create_mdiodev(bus, 0, &serdes, num_phys);
		if (IS_ERR(phylink_pcs)) {
			err = PTR_ERR(phylink_pcs);
			dev_err(dev, "cannot create lynx pcs (%d)\n", err);
			goto unregister_mdiobus;
		}
	} else {
		phylink_pcs = xpcs_create_mdiodev_with_phy(bus, 0, 16, pf->if_mode);
		if (IS_ERR(phylink_pcs)) {
			err = PTR_ERR(phylink_pcs);
			dev_err(dev, "cannot create xpcs mdiodev (%d)\n", err);
			goto unregister_mdiobus;
		}
	}

	pf->imdio = bus;
	pf->pcs = phylink_pcs;

	return 0;

unregister_mdiobus:
	mdiobus_unregister(bus);
free_mdio_bus:
	mdiobus_free(bus);
	return err;
}

static void enetc_imdio_remove(struct enetc_pf *pf)
{
	struct enetc_mdio_priv *mdio_priv;

	if (pf->pcs) {
		if (is_enetc_rev1(pf->si))
			lynx_pcs_destroy(pf->pcs);
		else
			xpcs_pcs_destroy(pf->pcs);
	}

	if (pf->imdio) {
		mdio_priv = pf->imdio->priv;

		mdiobus_unregister(pf->imdio);
		if (mdio_priv && mdio_priv->regulator)
			regulator_disable(mdio_priv->regulator);
		mdiobus_free(pf->imdio);
	}
}

static bool enetc_port_has_pcs(struct enetc_pf *pf)
{
	return (pf->if_mode == PHY_INTERFACE_MODE_SGMII ||
		pf->if_mode == PHY_INTERFACE_MODE_1000BASEX ||
		pf->if_mode == PHY_INTERFACE_MODE_2500BASEX ||
		pf->if_mode == PHY_INTERFACE_MODE_10GBASER ||
		pf->if_mode == PHY_INTERFACE_MODE_USXGMII ||
		pf->if_mode == PHY_INTERFACE_MODE_XGMII);
}

int enetc_mdiobus_create(struct enetc_pf *pf, struct device_node *node)
{
	struct device_node *mdio_np;
	int err;

	mdio_np = of_get_child_by_name(node, "mdio");
	if (mdio_np) {
		err = enetc_mdio_probe(pf, mdio_np);

		of_node_put(mdio_np);
		if (err)
			return err;
	}

	if (enetc_port_has_pcs(pf)) {
		err = enetc_imdio_create(pf);
		if (err) {
			enetc_mdio_remove(pf);
			return err;
		}
	}

	return 0;
}

void enetc_mdiobus_destroy(struct enetc_pf *pf)
{
	enetc_mdio_remove(pf);

	if (enetc_port_has_pcs(pf))
		enetc_imdio_remove(pf);
}

int enetc_phylink_create(struct enetc_ndev_priv *priv,
			 struct device_node *node,
			 const struct phylink_mac_ops *pl_mac_ops)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct enetc_si *si = priv->si;
	struct phylink *phylink;
	int err;

	pf->phylink_config.dev = &priv->ndev->dev;
	pf->phylink_config.type = PHYLINK_NETDEV;

	if (is_enetc_rev1(si))
		pf->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
			MAC_10 | MAC_100 | MAC_1000 | MAC_2500FD;
	else
		pf->phylink_config.mac_capabilities = MAC_ASYM_PAUSE | MAC_SYM_PAUSE |
			MAC_10 | MAC_100 | MAC_1000FD | MAC_2500FD | MAC_10000FD;

	__set_bit(PHY_INTERFACE_MODE_INTERNAL,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_SGMII,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_RMII,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_1000BASEX,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_2500BASEX,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_10GBASER,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_USXGMII,
		  pf->phylink_config.supported_interfaces);
	__set_bit(PHY_INTERFACE_MODE_XGMII,
		  pf->phylink_config.supported_interfaces);

	phy_interface_set_rgmii(pf->phylink_config.supported_interfaces);

	phylink = phylink_create(&pf->phylink_config, of_fwnode_handle(node),
				 pf->if_mode, pl_mac_ops);
	if (IS_ERR(phylink)) {
		err = PTR_ERR(phylink);
		return err;
	}

	priv->phylink = phylink;

	return 0;
}

void enetc_phylink_destroy(struct enetc_ndev_priv *priv)
{
	phylink_destroy(priv->phylink);
}

/* Messaging */
static u16 enetc_msg_pf_set_vf_primary_mac_addr(struct enetc_pf *pf, int vf_id)
{
	struct enetc_vf_state *vf_state = &pf->vf_state[vf_id];
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_mac_exact_filter *msg;
	union enetc_pf_msg pf_msg;
	char *addr;

	msg = (struct enetc_msg_mac_exact_filter *)msg_swbd->vaddr;
	addr = msg->mac[0].addr;
	if (vf_state->flags & ENETC_VF_FLAG_PF_SET_MAC) {
		dev_warn(dev, "Attempt to override PF set mac addr for VF%d\n",
			 vf_id);
		if (!enetc_pf_is_vf_trusted(pf, vf_id)) {
			pf_msg.class_id = ENETC_MSG_CLASS_ID_PERMISSION_DENY;
			return pf_msg.code;
		}
	}

	if (enetc_set_si_hw_addr(pf, vf_id + 1, addr))
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
	else
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static struct enetc_mac_list_entry
	*enetc_mac_list_lookup_entry(struct enetc_pf *pf, const unsigned char *addr)
{
	struct enetc_mac_list_entry *entry;

	hlist_for_each_entry(entry, &pf->mac_list, node)
		if (ether_addr_equal(entry->mfe.mac, addr))
			return entry;

	return NULL;
}

static inline void enetc_mac_list_add_entry(struct enetc_pf *pf,
					    struct enetc_mac_list_entry *entry)
{
	hlist_add_head(&entry->node, &pf->mac_list);
}

static inline void enetc_mac_list_del_entry(struct enetc_mac_list_entry *entry)
{
	hlist_del(&entry->node);
	kfree(entry);
}

static void enetc_mac_list_del_matched_entries(struct enetc_pf *pf, u16 si_bit,
					       struct enetc_mac_entry *mac,
					       int mac_cnt)
{
	struct enetc_mac_list_entry *entry;
	int i;

	for (i = 0; i < mac_cnt; i++) {
		entry = enetc_mac_list_lookup_entry(pf, mac[i].addr);
		if (entry) {
			entry->mfe.si_bitmap &= ~si_bit;
			if (!entry->mfe.si_bitmap) {
				enetc_mac_list_del_entry(entry);
				pf->num_mac_fe--;
			}
		}
	}
}

int enetc_pf_set_mac_exact_filter(struct enetc_pf *pf, int si_id,
				  struct enetc_mac_entry *mac,
				  int mac_cnt)
{
	int mf_max_num = pf->caps.mac_filter_num;
	struct enetc_mac_list_entry *entry;
	struct maft_entry_data data = {0};
	struct enetc_si *si = pf->si;
	int i = 0, used_cnt = 0;
	u16 si_bit = BIT(si_id);
	int mf_num;

	guard(mutex)(&pf->mac_list_lock);

	/* Check MAC filter table whether has enough available entries */
	hlist_for_each_entry(entry, &pf->mac_list, node) {
		for (i = 0; i < mac_cnt; i++) {
			if (ether_addr_equal(entry->mfe.mac, mac[i].addr)) {
				used_cnt++;

				if (mf_max_num - used_cnt < mac_cnt)
					return -ENOSPC;

				break;
			}
		}
	}

	mf_num = pf->num_mac_fe;
	/* Update mac_list */
	for (i = 0; i < mac_cnt; i++) {
		entry = enetc_mac_list_lookup_entry(pf, mac[i].addr);
		if (!entry) {
			entry = kzalloc(sizeof(*entry), GFP_KERNEL);
			if (unlikely(!entry)) {
				/* Restore MAC list to before update if an error occurs*/
				enetc_mac_list_del_matched_entries(pf, si_bit, mac,
								   i + 1);
				return -ENOMEM;
			}
			ether_addr_copy(entry->mfe.mac, mac[i].addr);
			entry->mfe.si_bitmap = si_bit;
			enetc_mac_list_add_entry(pf, entry);
			pf->num_mac_fe++;
		} else {
			entry->mfe.si_bitmap |= si_bit;
		}
	}

	/* Clear MAC filter table */
	for (i = 0; i < mf_num; i++)
		ntmp_maft_delete_entry(&si->ntmp.cbdrs, i);

	i = 0;
	hlist_for_each_entry(entry, &pf->mac_list, node) {
		data.cfge.si_bitmap = cpu_to_le16(entry->mfe.si_bitmap);
		ether_addr_copy(data.keye.mac_addr, entry->mfe.mac);
		ntmp_maft_add_entry(&si->ntmp.cbdrs, i++, &data);
	}

	return 0;
}

static u16 enetc_msg_pf_add_vf_mac_entries(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_mac_exact_filter *msg;
	struct enetc_si *si = pf->si;
	union enetc_pf_msg pf_msg;
	bool no_resource = false;
	int err;

	if (is_enetc_rev1(si)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_mac_exact_filter *)msg_swbd->vaddr;
	if (msg->mac_cnt > pf->caps.mac_filter_num) {
		no_resource = true;
		goto no_resource_check;
	}

	err = enetc_pf_set_mac_exact_filter(pf, vf_id + 1, msg->mac,
					    msg->mac_cnt);
	if (err)
		no_resource = true;

no_resource_check:
	if (no_resource) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_MAC_FILTER;
		pf_msg.class_code = ENETC_PF_RC_MAC_FILTER_NO_RESOURCE;
		return pf_msg.code;
	}

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static int enetc_msg_validate_delete_macs(struct enetc_pf *pf, u16 si_bit,
					  struct enetc_mac_entry *mac,
					  int mac_cnt)
{
	struct enetc_mac_list_entry *entry;
	int i;

	for (i = 0; i < mac_cnt; i++) {
		entry = enetc_mac_list_lookup_entry(pf, mac[i].addr);
		if (entry && (entry->mfe.si_bitmap & si_bit))
			continue;

		return -EINVAL;
	}

	return 0;
}

static u16 enetc_msg_pf_del_vf_mac_entries(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_mac_exact_filter *msg;
	struct enetc_mac_list_entry *entry;
	struct maft_entry_data data = {0};
	struct enetc_si *si = pf->si;
	u16 si_bit = BIT(vf_id + 1);
	union enetc_pf_msg pf_msg;
	int i, mf_num, err;

	if (is_enetc_rev1(si)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_mac_exact_filter *)msg_swbd->vaddr;

	guard(mutex)(&pf->mac_list_lock);

	err = enetc_msg_validate_delete_macs(pf, si_bit, msg->mac,
					     msg->mac_cnt);
	if (err) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_MAC_FILTER;
		pf_msg.class_code = ENETC_PF_RC_MAC_FILTER_MAC_NOT_FOUND;
		return pf_msg.code;
	}

	mf_num = pf->num_mac_fe;
	enetc_mac_list_del_matched_entries(pf, si_bit, msg->mac,
					   msg->mac_cnt);

	/* Clear MAC filter table */
	for (i = 0; i < mf_num; i++)
		ntmp_maft_delete_entry(&si->ntmp.cbdrs, i);

	i = 0;
	hlist_for_each_entry(entry, &pf->mac_list, node) {
		data.cfge.si_bitmap = cpu_to_le16(entry->mfe.si_bitmap);
		ether_addr_copy(data.keye.mac_addr, entry->mfe.mac);
		ntmp_maft_add_entry(&si->ntmp.cbdrs, i++, &data);
	}

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static u16 enetc_msg_pf_set_vf_mac_hash_filter(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_mac_hash_filter *msg;
	struct enetc_hw *hw = &pf->si->hw;
	union enetc_pf_msg pf_msg;
	int si_id = vf_id + 1;
	u64 hash_tbl;

	if (!enetc_pf_is_vf_trusted(pf, vf_id)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_PERMISSION_DENY;
		return pf_msg.code;
	}

	if (!pf->hw_ops->set_si_mac_hash_filter) {
		dev_err(dev, "MAC hash filter is not supported\n");
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_mac_hash_filter *)msg_swbd->vaddr;
	/* Currently, hardware only supports 64 bits table size */
	if (msg->size != ENETC_MAC_HASH_TABLE_SIZE_64) {
		dev_err(dev, "MAC hash table size exceeds 64 bits\n");
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	if (msg->type == ENETC_MAC_FILTER_TYPE_UC) {
		hash_tbl = (u64)(msg->hash_tbl[1]) << 32 | msg->hash_tbl[0];
		pf->hw_ops->set_si_mac_hash_filter(hw, si_id, UC, hash_tbl);
	} else if (msg->type == ENETC_MAC_FILTER_TYPE_MC) {
		hash_tbl = (u64)(msg->hash_tbl[1]) << 32 | msg->hash_tbl[0];
		pf->hw_ops->set_si_mac_hash_filter(hw, si_id, MC, hash_tbl);
	} else {
		hash_tbl = (u64)(msg->hash_tbl[1]) << 32 | msg->hash_tbl[0];
		pf->hw_ops->set_si_mac_hash_filter(hw, si_id, UC, hash_tbl);
		hash_tbl = (u64)(msg->hash_tbl[3]) << 32 | msg->hash_tbl[2];
		pf->hw_ops->set_si_mac_hash_filter(hw, si_id, MC, hash_tbl);
	}

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static bool enetc_msg_mac_type_check(int type, const u8 *addr)
{
	if (type == ENETC_MAC_FILTER_TYPE_UC)
		return !is_multicast_ether_addr(addr);
	else if (type == ENETC_MAC_FILTER_TYPE_MC)
		return is_multicast_ether_addr(addr);
	else
		return true;
}

void enetc_pf_flush_mac_exact_filter(struct enetc_pf *pf, int si_id,
				     int mac_type)
{
	struct enetc_mac_list_entry *entry;
	struct maft_entry_data data = {0};
	struct enetc_si *si = pf->si;
	u16 si_bit = BIT(si_id);
	struct hlist_node *tmp;
	int i, mf_num;

	if (is_enetc_rev1(si))
		return;

	guard(mutex)(&pf->mac_list_lock);

	mf_num = pf->num_mac_fe;
	hlist_for_each_entry_safe(entry, tmp, &pf->mac_list, node) {
		if (enetc_msg_mac_type_check(mac_type, entry->mfe.mac) &&
		    entry->mfe.si_bitmap & si_bit) {
			entry->mfe.si_bitmap &= ~si_bit;
			if (!entry->mfe.si_bitmap) {
				enetc_mac_list_del_entry(entry);
				pf->num_mac_fe--;
			}
		}
	}

	for (i = 0; i < mf_num; i++)
		ntmp_maft_delete_entry(&si->ntmp.cbdrs, i);

	i = 0;
	hlist_for_each_entry(entry, &pf->mac_list, node) {
		data.cfge.si_bitmap = cpu_to_le16(entry->mfe.si_bitmap);
		ether_addr_copy(data.keye.mac_addr, entry->mfe.mac);
		ntmp_maft_add_entry(&si->ntmp.cbdrs, i++, &data);
	}
}

static void enetc_pf_flush_si_mac_filter(struct enetc_pf *pf, int si_id,
					 int mac_type)
{
	struct enetc_hw *hw = &pf->si->hw;

	enetc_pf_flush_mac_exact_filter(pf, si_id, mac_type);

	if (!pf->hw_ops->set_si_mac_hash_filter)
		return;

	if (mac_type & ENETC_MAC_FILTER_TYPE_UC)
		pf->hw_ops->set_si_mac_hash_filter(hw, si_id, UC, 0);

	if (mac_type & ENETC_MAC_FILTER_TYPE_MC)
		pf->hw_ops->set_si_mac_hash_filter(hw, si_id, MC, 0);
}

static u16 enetc_msg_pf_flush_vf_mac_entries(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_mac_filter_flush *msg;
	struct enetc_si *si = pf->si;
	union enetc_pf_msg pf_msg;

	if (is_enetc_rev1(si)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_mac_filter_flush *)msg_swbd->vaddr;
	enetc_pf_flush_si_mac_filter(pf, vf_id + 1, msg->type);

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static u16 enetc_msg_pf_set_vf_mac_promisc_mode(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_mac_promsic_mode *msg;
	struct enetc_hw *hw = &pf->si->hw;
	union enetc_pf_msg pf_msg;
	bool promisc_mode = false;
	int si_id = vf_id + 1;
	int mac_type;

	if (!enetc_pf_is_vf_trusted(pf, vf_id)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_PERMISSION_DENY;
		return pf_msg.code;
	}

	if (!pf->hw_ops->set_si_mac_promisc) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_mac_promsic_mode *)msg_swbd->vaddr;
	if (msg->type == ENETC_MAC_FILTER_TYPE_UC)
		mac_type = ENETC_MAC_FILTER_TYPE_UC;
	else if (msg->type == ENETC_MAC_FILTER_TYPE_MC)
		mac_type = ENETC_MAC_FILTER_TYPE_MC;
	else
		mac_type = ENETC_MAC_FILTER_TYPE_ALL;

	if (msg->promisc_mode == ENETC_MAC_PROMISC_MODE_ENABLE)
		promisc_mode = true;

	if (msg->flush_macs)
		enetc_pf_flush_si_mac_filter(pf, si_id, msg->type);

	if (mac_type & ENETC_MAC_FILTER_TYPE_UC)
		pf->hw_ops->set_si_mac_promisc(hw, si_id, UC, promisc_mode);

	if (mac_type & ENETC_MAC_FILTER_TYPE_MC)
		pf->hw_ops->set_si_mac_promisc(hw, si_id, MC, promisc_mode);

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static u16 enetc_msg_handle_mac_filter(struct enetc_msg_header *msg_hdr,
				       struct enetc_pf *pf, int vf_id)
{
	union enetc_pf_msg pf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_SET_PRIMARY_MAC:
		return enetc_msg_pf_set_vf_primary_mac_addr(pf, vf_id);
	case ENETC_MSG_ADD_EXACT_MAC_ENTRIES:
		return enetc_msg_pf_add_vf_mac_entries(pf, vf_id);
	case ENETC_MSG_DEL_EXACT_MAC_ENTRIES:
		return enetc_msg_pf_del_vf_mac_entries(pf, vf_id);
	case ENETC_MSG_SET_MAC_HASH_TABLE:
		return enetc_msg_pf_set_vf_mac_hash_filter(pf, vf_id);
	case ENETC_MSG_FLUSH_MAC_ENTRIES:
		return enetc_msg_pf_flush_vf_mac_entries(pf, vf_id);
	case ENETC_MSG_SET_MAC_PROMISC_MODE:
		return enetc_msg_pf_set_vf_mac_promisc_mode(pf, vf_id);
	default:
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;

		return pf_msg.code;
	}
}

static struct enetc_vlan_list_entry
	*enetc_vlan_list_lookup_entry(struct enetc_pf *pf,
				      struct enetc_vlan_entry *vlan)
{
	struct enetc_vlan_list_entry *entry;

	hlist_for_each_entry(entry, &pf->vlan_list, node)
		if (entry->vfe.vid == vlan->vid &&
		    entry->vfe.tpid == vlan->tpid)
			return entry;

	return NULL;
}

static inline void enetc_vlan_list_add_entry(struct enetc_pf *pf,
					     struct enetc_vlan_list_entry *entry)
{
	hlist_add_head(&entry->node, &pf->vlan_list);
}

static inline void enetc_vlan_list_del_entry(struct enetc_vlan_list_entry *entry)
{
	hlist_del(&entry->node);
	kfree(entry);
}

static void enetc_vlan_list_del_matched_entries(struct enetc_pf *pf, u16 si_bit,
						struct enetc_vlan_entry *vlan,
						int vlan_cnt)
{
	struct enetc_vlan_list_entry *entry;
	int i;

	for (i = 0; i < vlan_cnt; i++) {
		entry = enetc_vlan_list_lookup_entry(pf, &vlan[i]);
		if (entry) {
			entry->vfe.si_bitmap &= ~si_bit;
			if (!entry->vfe.si_bitmap) {
				enetc_vlan_list_del_entry(entry);
				pf->num_vlan_fe--;
			}
		}
	}
}

static void enetc_vfe_to_vaft_data(struct enetc_vfe *vfe,
				   struct vaft_entry_data *vaft)
{
	vaft->keye.tpid = vfe->tpid;
	vaft->keye.vlan_id = cpu_to_le16(vfe->vid);
	vaft->cfge.si_bitmap = cpu_to_le16(vfe->si_bitmap);
}

static int enetc_pf_set_vlan_exact_filter(struct enetc_pf *pf, int si_id,
					  struct enetc_vlan_entry *vlan,
					  int vlan_cnt)
{
	int vf_max_num = pf->caps.vlan_filter_num;
	struct enetc_vlan_list_entry *entry;
	struct vaft_entry_data data = {0};
	struct enetc_si *si = pf->si;
	int i = 0, used_cnt = 0;
	u16 si_bit = BIT(si_id);
	int vf_num;

	guard(mutex)(&pf->vlan_list_lock);

	/* Check VLAN filter table whether has enough available entries */
	hlist_for_each_entry(entry, &pf->vlan_list, node) {
		for (i = 0; i < vlan_cnt; i++) {
			if (entry->vfe.vid == vlan[i].vid &&
			    entry->vfe.tpid == vlan[i].tpid) {
				used_cnt++;

				if (vf_max_num - used_cnt < vlan_cnt)
					return -ENOSPC;

				break;
			}
		}
	}

	vf_num = pf->num_vlan_fe;
	/* Update VLAN list */
	for (i = 0; i < vlan_cnt; i++) {
		entry = enetc_vlan_list_lookup_entry(pf, &vlan[i]);
		if (!entry) {
			entry = kzalloc(sizeof(*entry), GFP_KERNEL);
			if (unlikely(!entry)) {
				enetc_vlan_list_del_matched_entries(pf, si_bit, vlan,
								    i + 1);
				return -ENOMEM;
			}
			entry->vfe.vid = vlan[i].vid;
			entry->vfe.tpid = vlan[i].tpid;
			entry->vfe.si_bitmap = si_bit;
			enetc_vlan_list_add_entry(pf, entry);
			pf->num_vlan_fe++;
		} else {
			entry->vfe.si_bitmap |= si_bit;
		}
	}

	/* Clear VLAN filter table */
	for (i = 0; i < vf_num; i++)
		ntmp_vaft_delete_entry(&si->ntmp.cbdrs, i);

	i = 0;
	hlist_for_each_entry(entry, &pf->vlan_list, node) {
		enetc_vfe_to_vaft_data(&entry->vfe, &data);
		ntmp_vaft_add_entry(&si->ntmp.cbdrs, i++, &data);
	}

	return 0;
}

static u16 enetc_msg_pf_add_vf_vlan_entries(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_vlan_exact_filter *msg;
	struct enetc_si *si = pf->si;
	union enetc_pf_msg pf_msg;
	bool no_resource = false;
	int err;

	if (is_enetc_rev1(si)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_vlan_exact_filter *)msg_swbd->vaddr;
	if (msg->vlan_cnt > pf->caps.vlan_filter_num) {
		no_resource = true;
		goto no_resource_check;
	}

	err = enetc_pf_set_vlan_exact_filter(pf, vf_id + 1, msg->vlan,
					     msg->vlan_cnt);
	if (err)
		no_resource = true;

no_resource_check:
	if (no_resource) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_VLAN_FILTER;
		pf_msg.class_code = ENETC_PF_RC_VLAN_FILTER_NO_RESOURCE;
		return pf_msg.code;
	}

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static int enetc_msg_validate_delete_vlans(struct enetc_pf *pf, u16 si_bit,
					   struct enetc_vlan_entry *vlan,
					   int vlan_cnt)
{
	struct enetc_vlan_list_entry *entry;
	int i;

	for (i = 0; i < vlan_cnt; i++) {
		entry = enetc_vlan_list_lookup_entry(pf, &vlan[i]);
		if (entry && (entry->vfe.si_bitmap & si_bit))
			continue;

		return -EINVAL;
	}

	return 0;
}

static u16 enetc_msg_pf_del_vf_vlan_entries(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_vlan_exact_filter *msg;
	struct enetc_vlan_list_entry *entry;
	struct vaft_entry_data data = {0};
	struct enetc_si *si = pf->si;
	u16 si_bit = BIT(vf_id + 1);
	union enetc_pf_msg pf_msg;
	int i, vf_num, err;

	if (is_enetc_rev1(si)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_vlan_exact_filter *)msg_swbd->vaddr;
	guard(mutex)(&pf->vlan_list_lock);

	err = enetc_msg_validate_delete_vlans(pf, si_bit, msg->vlan,
					      msg->vlan_cnt);
	if (err) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_VLAN_FILTER;
		pf_msg.class_code = ENETC_PF_RC_VLAN_FILTER_VLAN_NOT_FOUND;
		return pf_msg.code;
	}

	vf_num = pf->num_vlan_fe;
	enetc_vlan_list_del_matched_entries(pf, si_bit, msg->vlan,
					    msg->vlan_cnt);

	for (i = 0; i < vf_num; i++)
		ntmp_vaft_delete_entry(&si->ntmp.cbdrs, i);

	i = 0;
	hlist_for_each_entry(entry, &pf->vlan_list, node) {
		enetc_vfe_to_vaft_data(&entry->vfe, &data);
		ntmp_vaft_add_entry(&si->ntmp.cbdrs, i++, &data);
	}

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static u16 enetc_msg_pf_set_vf_vlan_hash_filter(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_vlan_hash_filter *msg;
	struct enetc_hw *hw = &pf->si->hw;
	union enetc_pf_msg pf_msg;
	int si_id = vf_id + 1;
	u64 hash_tbl;

	if (!enetc_pf_is_vf_trusted(pf, vf_id)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_PERMISSION_DENY;
		return pf_msg.code;
	}

	if (!pf->hw_ops->set_si_vlan_hash_filter) {
		dev_err(dev, "VLAN hash filter is not supported\n");
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_vlan_hash_filter *)msg_swbd->vaddr;
	/* Currently, hardware only supports 64 bits table size */
	if (msg->size != ENETC_VLAN_HASH_TABLE_SIZE_64) {
		dev_err(dev, "VLAN hash table size exceeds 64 bits\n");
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	hash_tbl = (u64)(msg->hash_tbl[1]) << 32 | msg->hash_tbl[0];
	pf->hw_ops->set_si_vlan_hash_filter(hw, si_id, hash_tbl);

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static void enetc_pf_flush_vlan_exact_filter(struct enetc_pf *pf, int si_id)
{
	struct enetc_vlan_list_entry *entry;
	struct vaft_entry_data data = {0};
	struct enetc_si *si = pf->si;
	u16 si_bit = BIT(si_id);
	struct hlist_node *tmp;
	int i, vf_num;

	if (is_enetc_rev1(si))
		return;

	guard(mutex)(&pf->vlan_list_lock);

	vf_num = pf->num_vlan_fe;
	hlist_for_each_entry_safe(entry, tmp, &pf->vlan_list, node) {
		if (entry->vfe.si_bitmap & si_bit) {
			entry->vfe.si_bitmap &= ~si_bit;
			if (!entry->vfe.si_bitmap) {
				enetc_vlan_list_del_entry(entry);
				pf->num_vlan_fe--;
			}
		}
	}

	for (i = 0; i < vf_num; i++)
		ntmp_vaft_delete_entry(&si->ntmp.cbdrs, i);

	i = 0;
	hlist_for_each_entry(entry, &pf->vlan_list, node) {
		enetc_vfe_to_vaft_data(&entry->vfe, &data);
		ntmp_vaft_add_entry(&si->ntmp.cbdrs, i++, &data);
	}
}

static void enetc_pf_flush_si_vlan_filter(struct enetc_pf *pf, int si_id)
{
	enetc_pf_flush_vlan_exact_filter(pf, si_id);
	if (pf->hw_ops->set_si_vlan_hash_filter) {
		struct enetc_hw *hw = &pf->si->hw;

		pf->hw_ops->set_si_vlan_hash_filter(hw, si_id, 0);
	}
}

static u16 enetc_msg_pf_flush_vf_vlan_entries(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_vlan_filter_flush *msg;
	struct enetc_si *si = pf->si;
	union enetc_pf_msg pf_msg;

	if (is_enetc_rev1(si)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_vlan_filter_flush *)msg_swbd->vaddr;
	enetc_pf_flush_si_vlan_filter(pf, vf_id + 1);

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static u16 enetc_msg_pf_set_vf_vlan_promisc_mode(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct enetc_msg_vlan_promsic_mode *msg;
	union enetc_pf_msg pf_msg;
	bool promisc_mode = false;
	int si_id = vf_id + 1;

	if (!enetc_pf_is_vf_trusted(pf, vf_id)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_PERMISSION_DENY;
		return pf_msg.code;
	}

	msg = (struct enetc_msg_vlan_promsic_mode *)msg_swbd->vaddr;
	if (msg->promisc_mode == ENETC_VLAN_PROMISC_MODE_ENABLE)
		promisc_mode = true;

	if (msg->flush_vlans)
		enetc_pf_flush_si_vlan_filter(pf, si_id);

	enetc_set_si_vlan_promisc(pf, si_id, promisc_mode);

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;

	return pf_msg.code;
}

static u16 enetc_msg_handle_vlan_filter(struct enetc_msg_header *msg_hdr,
					struct enetc_pf *pf, int vf_id)
{
	union enetc_pf_msg pf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_ADD_EXACT_VLAN_ENTRIES:
		return enetc_msg_pf_add_vf_vlan_entries(pf, vf_id);
	case ENETC_MSG_DEL_EXACT_VLAN_ENTRIES:
		return enetc_msg_pf_del_vf_vlan_entries(pf, vf_id);
	case ENETC_MSG_SET_VLAN_HASH_TABLE:
		return enetc_msg_pf_set_vf_vlan_hash_filter(pf, vf_id);
	case ENETC_MSG_FLUSH_VLAN_ENTRIES:
		return enetc_msg_pf_flush_vf_vlan_entries(pf, vf_id);
	case ENETC_MSG_SET_VLAN_PROMISC_MODE:
		return enetc_msg_pf_set_vf_vlan_promisc_mode(pf, vf_id);
	default:
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;

		return pf_msg.code;
	}
}

static u16 enetc_msg_pf_reply_link_status(struct enetc_pf *pf)
{
	struct net_device *ndev = pf->si->ndev;
	union enetc_pf_msg pf_msg;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	if (netif_carrier_ok(ndev))
		pf_msg.class_code = ENETC_PF_NC_LINK_STATUS_UP;
	else
		pf_msg.class_code = ENETC_PF_NC_LINK_STATUS_DOWN;

	return pf_msg.code;
}

int enetc_pf_send_msg(struct enetc_pf *pf, u32 msg_code, u16 ms_mask)
{
	struct enetc_si *si = pf->si;
	u32 psimsgsr;
	int err;

	psimsgsr = PSIMSGSR_SET_MC(msg_code);
	psimsgsr |= ms_mask;

	guard(mutex)(&si->msg_lock);
	enetc_wr(&si->hw, ENETC_PSIMSGSR, psimsgsr);
	err = read_poll_timeout(enetc_rd, psimsgsr,
				!(psimsgsr & ms_mask),
				100, 100000, false, &si->hw, ENETC_PSIMSGSR);

	return err;
}

static void enetc_pf_send_link_status_msg(struct enetc_pf *pf, u16 ms_mask)
{
	struct device *dev = &pf->si->pdev->dev;
	struct net_device *ndev = pf->si->ndev;
	union enetc_pf_msg pf_msg = { 0 };
	u32 msg_code;
	int err;

	if (!ms_mask)
		return;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	if (netif_carrier_ok(ndev))
		pf_msg.class_code = ENETC_PF_NC_LINK_STATUS_UP;
	else
		pf_msg.class_code = ENETC_PF_NC_LINK_STATUS_DOWN;

	msg_code = pf_msg.code;
	err = enetc_pf_send_msg(pf, msg_code, ms_mask);
	if (err)
		dev_err(dev, "PF notifies link status failed\n");
}

static u16 enetc_msg_register_link_status_notify(struct enetc_pf *pf, int vf_id,
						 bool notify)
{
	struct enetc_hw *hw = &pf->si->hw;
	union enetc_pf_msg pf_msg;
	u32 msg_code, val;

	pf->vf_link_status_notify[vf_id] = notify;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_SUCCESS;
	msg_code = pf_msg.code;

	/* Reply to VF */
	val = ENETC_SIMSGSR_SET_MC(msg_code);
	val |= ENETC_PSIMSGRR_MR(vf_id); /* w1c */
	enetc_wr(hw, ENETC_PSIMSGRR, val);

	/* Notify VF the current link status */
	if (notify)
		enetc_pf_send_link_status_msg(pf, PSIMSGSR_MS(vf_id));

	return 0;
}

static u16 enetc_msg_handle_link_status(struct enetc_msg_header *msg_hdr,
					struct enetc_pf *pf, int vf_id)
{
	union enetc_pf_msg pf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_CURRENT_LINK_STATUS:
		return enetc_msg_pf_reply_link_status(pf);
	case ENETC_MSG_REGISTER_LINK_CHANGE_NOTIFY:
		return enetc_msg_register_link_status_notify(pf, vf_id, true);
	case ENETC_MSG_UNREGISTER_LINK_CHANGE_NOTIFY:
		return enetc_msg_register_link_status_notify(pf, vf_id, false);
	default:
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;

		return pf_msg.code;
	}
}

static u16 enetc_msg_pf_reply_link_speed(struct enetc_pf *pf)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	struct ethtool_link_ksettings link_info = {0};
	union enetc_pf_msg pf_msg;

	rtnl_lock();
	if (!priv->phylink ||
	    phylink_ethtool_ksettings_get(priv->phylink, &link_info)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		rtnl_unlock();

		return pf_msg.code;
	}
	rtnl_unlock();

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_SPEED;

	switch (link_info.base.speed) {
	case SPEED_10:
		if (link_info.base.duplex == DUPLEX_HALF)
			pf_msg.class_code = ENETC_MSG_SPEED_10M_HD;
		else
			pf_msg.class_code = ENETC_MSG_SPEED_10M_FD;
		break;
	case SPEED_100:
		if (link_info.base.duplex == DUPLEX_HALF)
			pf_msg.class_code = ENETC_MSG_SPEED_100M_HD;
		else
			pf_msg.class_code = ENETC_MSG_SPEED_100M_FD;
		break;
	case SPEED_1000:
		pf_msg.class_code = ENETC_MSG_SPEED_1000M;
		break;
	case SPEED_2500:
		pf_msg.class_code = ENETC_MSG_SPEED_2500M;
		break;
	case SPEED_5000:
		pf_msg.class_code = ENETC_MSG_SPEED_5G;
		break;
	case SPEED_10000:
		pf_msg.class_code = ENETC_MSG_SPEED_10G;
		break;
	case SPEED_25000:
		pf_msg.class_code = ENETC_MSG_SPEED_25G;
		break;
	case SPEED_50000:
		pf_msg.class_code = ENETC_MSG_SPEED_50G;
		break;
	case SPEED_100000:
		pf_msg.class_code = ENETC_MSG_SPEED_100G;
		break;
	default:
		pf_msg.class_code = ENETC_MSG_SPEED_UNKNOWN;
	}

	return pf_msg.code;
}

static u16 enetc_msg_handle_link_speed(struct enetc_msg_header *msg_hdr,
				       struct enetc_pf *pf, int vf_id)
{
	union enetc_pf_msg pf_msg;

	switch (msg_hdr->cmd_id) {
	case ENETC_MSG_GET_CURRENT_LINK_SPEED:
		return enetc_msg_pf_reply_link_speed(pf);
	default:
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;

		return pf_msg.code;
	}
}

static bool enetc_msg_check_crc16(void *msg_addr, u32 msg_size)
{
	u8 *data_buf = msg_addr + 2;
	u8 data_size = msg_size - 2;
	u16 verify_val;

	if (msg_size > ENETC_DEFAULT_MSG_SIZE)
		return false;

	verify_val = crc_itu_t(ENETC_CRC_INIT, data_buf, data_size);
	verify_val = crc_itu_t(verify_val, msg_addr, 2);
	if (verify_val)
		return false;

	return true;
}

void enetc_msg_handle_rxmsg(struct enetc_pf *pf, int vf_id, u16 *msg_code)
{
	struct enetc_msg_swbd *msg_swbd = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_header *msg_hdr;
	union enetc_pf_msg pf_msg;
	u32 msg_size;

	msg_hdr = (struct enetc_msg_header *)msg_swbd->vaddr;
	msg_size = ENETC_MSG_SIZE(msg_hdr->len);
	if (!enetc_msg_check_crc16(msg_swbd->vaddr, msg_size)) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CRC_ERROR;
		*msg_code = pf_msg.code;

		dev_err(dev, "VSI to PSI Message CRC16 error\n");

		return;
	}

	/* Currently, we don't support asynchronous action */
	if (msg_hdr->cookie) {
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		*msg_code = pf_msg.code;

		dev_err(dev, "Cookie field is not supported yet\n");

		return;
	}

	switch (msg_hdr->class_id) {
	case ENETC_MSG_CLASS_ID_MAC_FILTER:
		*msg_code = enetc_msg_handle_mac_filter(msg_hdr, pf, vf_id);
		break;
	case ENETC_MSG_CLASS_ID_VLAN_FILTER:
		*msg_code = enetc_msg_handle_vlan_filter(msg_hdr, pf, vf_id);
		break;
	case ENETC_MSG_CLASS_ID_LINK_STATUS:
		*msg_code = enetc_msg_handle_link_status(msg_hdr, pf, vf_id);
		break;
	case ENETC_MSG_CLASS_ID_LINK_SPEED:
		*msg_code = enetc_msg_handle_link_speed(msg_hdr, pf, vf_id);
		break;
	default:
		pf_msg.class_id = ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT;
		*msg_code = pf_msg.code;
	}
}

#ifdef CONFIG_PCI_IOV
int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err;

	if (enetc_pf_is_owned_by_mcore(pdev)) {
		err = pci_sriov_configure_simple(pdev, num_vfs);
		if (err < 0)
			dev_err(&pdev->dev,
				"pci_sriov_configure_simple err %d\n", err);

		return err;
	}

	si = pci_get_drvdata(pdev);
	pf = enetc_si_priv(si);

	if (!num_vfs) {
		pci_disable_sriov(pdev);
		enetc_msg_psi_free(pf);
		pf->num_vfs = 0;
	} else {
		pf->num_vfs = num_vfs;

		err = enetc_msg_psi_init(pf);
		if (err) {
			dev_err(&pdev->dev, "enetc_msg_psi_init (%d)\n", err);
			goto err_msg_psi;
		}

		err = pci_enable_sriov(pdev, num_vfs);
		if (err) {
			dev_err(&pdev->dev, "pci_enable_sriov err %d\n", err);
			goto err_en_sriov;
		}
	}

	return num_vfs;

err_en_sriov:
	enetc_msg_psi_free(pf);
err_msg_psi:
	pf->num_vfs = 0;

	return err;
}
#else
int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	return 0;
}
#endif
