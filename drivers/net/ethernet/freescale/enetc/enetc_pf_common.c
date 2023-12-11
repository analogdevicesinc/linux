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
	    pf->hw_ops->set_si_vlan_filter)
		pf->hw_ops->set_si_vlan_filter(hw, 0, *si->vlan_ht_filter);

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
		    pf->hw_ops->set_si_vlan_filter)
			pf->hw_ops->set_si_vlan_filter(hw, 0, *si->vlan_ht_filter);
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

int enetc_pf_set_features(struct net_device *ndev, netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	int err;

	if (changed & NETIF_F_HW_TC) {
		err = enetc_set_psfp(ndev, !!(features & NETIF_F_HW_TC));
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

	if (si->num_rss)
		ndev->hw_features |= NETIF_F_RXHASH;

	/* If driver handles unicast address filtering, it should set
	 * IFF_UNICAST_FLT in its priv_flags. (Refer to the description
	 * of the ndo_set_rx_mode())
	 */
	ndev->priv_flags |= IFF_UNICAST_FLT;
	if (is_enetc_rev1(si)) {
		ndev->max_mtu = ENETC_MAX_MTU;
		priv->max_frags_bd = ENETC_MAX_SKB_FRAGS;
		ndev->xdp_features = NETDEV_XDP_ACT_BASIC | NETDEV_XDP_ACT_REDIRECT |
				     NETDEV_XDP_ACT_NDO_XMIT | NETDEV_XDP_ACT_RX_SG |
				     NETDEV_XDP_ACT_NDO_XMIT_SG;
	} else {
		ndev->max_mtu = ENETC4_MAX_MTU;
		priv->max_frags_bd = ENETC4_MAX_SKB_FRAGS;
		priv->active_offloads |= ENETC_F_CHECKSUM | ENETC_F_LSO;
	}

	if (si->hw_features & ENETC_SI_F_PSFP && !enetc_psfp_enable(priv)) {
		priv->active_offloads |= ENETC_F_QCI;
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
	struct enetc_mdio_priv *mdio_priv = pf->imdio->priv;

	if (pf->pcs) {
		if (is_enetc_rev1(pf->si))
			lynx_pcs_destroy(pf->pcs);
		else
			xpcs_pcs_destroy(pf->pcs);
	}

	if (pf->imdio) {
		mdiobus_unregister(pf->imdio);
		mdiobus_free(pf->imdio);
	}

	if (mdio_priv->regulator)
		regulator_disable(mdio_priv->regulator);
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
static u16 enetc_msg_pf_set_vf_primary_mac_addr(struct enetc_pf *pf,
						int vf_id)
{
	struct enetc_vf_state *vf_state = &pf->vf_state[vf_id];
	struct enetc_msg_swbd *msg = &pf->rxmsg[vf_id];
	struct enetc_msg_cmd_set_primary_mac *cmd;
	struct device *dev = &pf->si->pdev->dev;
	u16 cmd_id;
	char *addr;
	int err;

	cmd = (struct enetc_msg_cmd_set_primary_mac *)msg->vaddr;
	cmd_id = cmd->header.id;
	if (cmd_id != ENETC_MSG_CMD_MNG_ADD)
		return ENETC_MSG_CMD_STATUS_FAIL;

	addr = cmd->mac.sa_data;
	if (vf_state->flags & ENETC_VF_FLAG_PF_SET_MAC) {
		dev_warn(dev, "Attempt to override PF set mac addr for VF%d\n",
			 vf_id);
		goto end;
	}

	err = enetc_set_si_hw_addr(pf, vf_id + 1, addr);
	if (err)
		return -ENETC_MSG_CMD_STATUS_FAIL;

end:
	return ENETC_MSG_CMD_STATUS_OK;
}

static u16 enetc_msg_psi_set_vsi_rx_mode(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg = &pf->rxmsg[vf_id];
	struct enetc_msg_config_mac_filter *cmd;
	struct enetc_hw *hw = &pf->si->hw;
	int si_id = vf_id + 1;

	cmd = (struct enetc_msg_config_mac_filter *)msg->vaddr;
	if (cmd->header.id != ENETC_MSG_CMD_MNG_ADD)
		return ENETC_MSG_CMD_STATUS_FAIL;

	if (!pf->hw_ops->set_si_mac_promisc || !pf->hw_ops->set_si_mac_filter)
		return ENETC_MSG_CMD_STATUS_FAIL;

	/* Set unicast promiscuous mode and hash filter. */
	pf->hw_ops->set_si_mac_promisc(hw, si_id, UC, !!cmd->uc_promisc);
	pf->hw_ops->set_si_mac_filter(hw, si_id, UC, *cmd->uc_hash_table);

	/* Set multicast promiscuous mode and hash filter. */
	pf->hw_ops->set_si_mac_promisc(hw, si_id, MC, !!cmd->mc_promisc);
	pf->hw_ops->set_si_mac_filter(hw, si_id, MC, *cmd->mc_hash_table);

	return ENETC_MSG_CMD_STATUS_OK;
}

static u16 enetc_msg_psi_set_vsi_vlan_filter(struct enetc_pf *pf, int vf_id)
{
	struct enetc_msg_swbd *msg = &pf->rxmsg[vf_id];
	struct enetc_msg_config_vlan_filter *cmd;
	struct enetc_hw *hw = &pf->si->hw;
	int si_id = vf_id + 1;

	if (!pf->hw_ops->set_si_vlan_filter)
		return ENETC_MSG_CMD_STATUS_FAIL;

	cmd = (struct enetc_msg_config_vlan_filter *)msg->vaddr;
	if (cmd->vlan_promisc) {
		/* Clear VLAN filter hash table. */
		pf->hw_ops->set_si_vlan_filter(hw, si_id, 0);
		enetc_set_si_vlan_promisc(pf, si_id, true);
	} else {
		/* Disable the VLAN promiscuous mode of the VSI if the VLAN
		 * promiscuous mode is enabled before.
		 */
		if (pf->vlan_promisc_simap & BIT(si_id))
			enetc_set_si_vlan_promisc(pf, si_id, false);

		pf->hw_ops->set_si_vlan_filter(hw, si_id, *cmd->vlan_hash_table);
	}

	return ENETC_MSG_CMD_STATUS_OK;
}

void enetc_msg_handle_rxmsg(struct enetc_pf *pf, int vf_id, u16 *status)
{
	struct enetc_msg_swbd *msg = &pf->rxmsg[vf_id];
	struct device *dev = &pf->si->pdev->dev;
	struct enetc_msg_cmd_header *cmd_hdr;
	u16 cmd_type;

	cmd_hdr = (struct enetc_msg_cmd_header *)msg->vaddr;
	cmd_type = cmd_hdr->type;

	switch (cmd_type) {
	case ENETC_MSG_CMD_MNG_MAC:
		*status = enetc_msg_pf_set_vf_primary_mac_addr(pf, vf_id);
		break;
	case ENETC_MSG_CMD_MNG_RX_MAC_FILTER:
		*status = enetc_msg_psi_set_vsi_rx_mode(pf, vf_id);
		break;
	case ENETC_MSG_CMD_MNG_RX_VLAN_FILTER:
		*status = enetc_msg_psi_set_vsi_vlan_filter(pf, vf_id);
		break;
	default:
		*status = ENETC_MSG_CMD_NOT_SUPPORT;
		dev_err(dev, "command not supported (cmd_type: 0x%x)\n",
			cmd_type);
	}
}

#ifdef CONFIG_PCI_IOV
int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);
	int err;

	if (!num_vfs) {
		pci_disable_sriov(pdev);
		enetc_msg_psi_free(pf);
		kfree(pf->vf_state);
		pf->num_vfs = 0;
	} else {
		pf->num_vfs = num_vfs;

		pf->vf_state = kcalloc(num_vfs, sizeof(struct enetc_vf_state),
				       GFP_KERNEL);
		if (!pf->vf_state) {
			pf->num_vfs = 0;
			return -ENOMEM;
		}

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
	kfree(pf->vf_state);
	pf->num_vfs = 0;

	return err;
}
#else
int enetc_sriov_configure(struct pci_dev *pdev, int num_vfs)
{
	return 0;
}
#endif
