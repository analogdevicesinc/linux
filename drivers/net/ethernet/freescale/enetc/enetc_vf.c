// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2017-2019 NXP */

#include <linux/module.h>
#include "enetc.h"

#define ENETC_DRV_NAME_STR "ENETC VF driver"

/* Messaging */
static void enetc_msg_vsi_write_msg(struct enetc_hw *hw,
				    struct enetc_msg_swbd *msg)
{
	u32 val;

	val = enetc_vsi_set_msize(msg->size) | lower_32_bits(msg->dma);
	enetc_wr(hw, ENETC_VSIMSGSNDAR1, upper_32_bits(msg->dma));
	enetc_wr(hw, ENETC_VSIMSGSNDAR0, val);
}

static int enetc_msg_vsi_send(struct enetc_si *si, struct enetc_msg_swbd *msg)
{
	u32 vsimsgsr;
	int err;

	enetc_msg_vsi_write_msg(&si->hw, msg);
	/* may be called in a spin_lock context */
	err = read_poll_timeout_atomic(enetc_rd, vsimsgsr,
				       !(vsimsgsr & ENETC_VSIMSGSR_MB),
				       100, 100000, false, &si->hw, ENETC_VSIMSGSR);
	if (err)
		return err;

	/* check for message delivery error */
	if (vsimsgsr & ENETC_VSIMSGSR_MS) {
		dev_err(&si->pdev->dev, "Transfer error when copying the data.\n");
		return -EIO;
	}

	/* Check the user-defined completion status. */
	if (ENETC_SIMSGSR_GET_MC(vsimsgsr)) {
		dev_err(&si->pdev->dev, "VSI command execute error: %d\n",
			ENETC_SIMSGSR_GET_MC(vsimsgsr));
		if (ENETC_SIMSGSR_GET_MC(vsimsgsr) == ENETC_MSG_CMD_NOT_SUPPORT)
			return -EOPNOTSUPP;
		else
			return -EIO;
	}

	return 0;
}

static int enetc_msg_vsi_set_primary_mac_addr(struct enetc_ndev_priv *priv,
					      struct sockaddr *saddr)
{
	struct enetc_msg_cmd_set_primary_mac *cmd;
	struct enetc_msg_swbd msg;
	int err;

	msg.size = ALIGN(sizeof(struct enetc_msg_cmd_set_primary_mac), 64);
	msg.vaddr = dma_alloc_coherent(priv->dev, msg.size, &msg.dma,
				       GFP_KERNEL);
	if (!msg.vaddr) {
		dev_err(priv->dev,
			"Failed to alloc Tx msg (size: %d) for primary MAC\n",
			msg.size);
		return -ENOMEM;
	}

	cmd = (struct enetc_msg_cmd_set_primary_mac *)msg.vaddr;
	cmd->header.type = ENETC_MSG_CMD_MNG_MAC;
	cmd->header.id = ENETC_MSG_CMD_MNG_ADD;
	memcpy(&cmd->mac, saddr, sizeof(struct sockaddr));

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg);

	dma_free_coherent(priv->dev, msg.size, msg.vaddr, msg.dma);

	return err;
}

static int enetc_vf_set_mac_addr(struct net_device *ndev, void *addr)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct sockaddr *saddr = addr;
	int err;

	if (!is_valid_ether_addr(saddr->sa_data))
		return -EADDRNOTAVAIL;

	err = enetc_msg_vsi_set_primary_mac_addr(priv, saddr);
	if (err)
		return err;

	eth_hw_addr_set(ndev, saddr->sa_data);

	return 0;
}

static int enetc_msg_vsi_set_mac_filter(struct enetc_ndev_priv *priv,
					u8 uc_promisc, u8 mc_promisc,
					struct enetc_mac_filter *uc_filter,
					struct enetc_mac_filter *mc_filter)
{
	struct enetc_msg_config_mac_filter *cmd;
	struct enetc_msg_swbd msg;
	int err;

	msg.size = ALIGN(sizeof(*cmd), 64);
	msg.vaddr = dma_alloc_coherent(priv->dev, msg.size, &msg.dma,
				       GFP_ATOMIC);
	if (!msg.vaddr) {
		err = -ENOMEM;
		dev_err(priv->dev,
			"Failed to alloc Tx msg (size: %d) for MAC filter\n",
			msg.size);
		goto end;
	}

	cmd = (struct enetc_msg_config_mac_filter *)msg.vaddr;
	memset(cmd, 0, sizeof(*cmd));
	cmd->header.type = ENETC_MSG_CMD_MNG_RX_MAC_FILTER;
	cmd->header.id = ENETC_MSG_CMD_MNG_ADD;
	cmd->uc_promisc = uc_promisc;
	cmd->mc_promisc = mc_promisc;
	if (!uc_promisc)
		memcpy(cmd->uc_hash_table, uc_filter->mac_hash_table,
		       sizeof(cmd->uc_hash_table));

	if (!mc_promisc)
		memcpy(cmd->mc_hash_table, mc_filter->mac_hash_table,
		       sizeof(cmd->mc_hash_table));

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg);

	dma_free_coherent(priv->dev, msg.size, msg.vaddr, msg.dma);

end:
	return err;
}

/* Notice that the driver only implements hash table filtering of the
 * VSI MAC filter.
 */
static void enetc_vf_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_mac_filter *uc_filter;
	struct enetc_mac_filter *mc_filter;
	struct enetc_si *si = priv->si;
	struct netdev_hw_addr *ha;
	bool uc_promisc = false;
	bool mc_promisc = false;

	if (is_enetc_rev1(si))
		return;

	uc_filter = &si->mac_filter[UC];
	mc_filter = &si->mac_filter[MC];

	if (ndev->flags & IFF_PROMISC) {
		uc_promisc = true;
		mc_promisc = true;
	} else if (ndev->flags & IFF_ALLMULTI) {
		mc_promisc = true;
	}

	enetc_reset_mac_addr_filter(uc_filter);
	enetc_reset_mac_addr_filter(mc_filter);

	/* If unicast promisc mode is disabled, set unicast filter rules. */
	if (!uc_promisc) {
		netdev_for_each_uc_addr(ha, ndev)
			enetc_add_mac_addr_ht_filter(uc_filter, ha->addr);
	}

	/* If multicast promisc mode is disabled, set multicast filter rules. */
	if (!mc_promisc) {
		netdev_for_each_mc_addr(ha, ndev) {
			if (!is_multicast_ether_addr(ha->addr))
				continue;

			enetc_add_mac_addr_ht_filter(mc_filter, ha->addr);
		}
	}

	enetc_msg_vsi_set_mac_filter(priv, uc_promisc, mc_promisc,
				     uc_filter, mc_filter);
}

/* Notice that the driver only implements hash table filtering of the
 * VSI VLAN filter.
 */
static int enetc_msg_vsi_set_vlan_filter(struct enetc_ndev_priv *priv,
					 bool vlan_promisc)
{
	struct enetc_msg_config_vlan_filter *cmd;
	struct enetc_si *si = priv->si;
	struct enetc_msg_swbd msg;
	int err;

	msg.size = ALIGN(sizeof(*cmd), 64);
	msg.vaddr = dma_alloc_coherent(priv->dev, msg.size, &msg.dma,
				       GFP_KERNEL);
	if (!msg.vaddr) {
		err = -ENOMEM;
		dev_err(priv->dev,
			"Failed to alloc Tx msg (size: %d) for VLAN filter\n",
			msg.size);
		goto end;
	}

	cmd = (struct enetc_msg_config_vlan_filter *)msg.vaddr;
	cmd->header.type = ENETC_MSG_CMD_MNG_RX_VLAN_FILTER;
	if (vlan_promisc) {
		cmd->vlan_promisc = 1;
	} else {
		cmd->vlan_promisc = 0;
		memcpy(cmd->vlan_hash_table, si->vlan_ht_filter,
		       sizeof(cmd->vlan_hash_table));
	}

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg);

	dma_free_coherent(priv->dev, msg.size, msg.vaddr, msg.dma);

end:
	return err;
}

static int enetc_vf_vlan_rx_add_vid(struct net_device *ndev,
				    __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int idx;

	if (is_enetc_rev1(si))
		return -EOPNOTSUPP;

	__set_bit(vid, si->active_vlans);

	idx = enetc_vid_hash_idx(vid);
	if (!__test_and_set_bit(idx, si->vlan_ht_filter))
		enetc_msg_vsi_set_vlan_filter(priv, false);

	return 0;
}

static int enetc_vf_vlan_rx_del_vid(struct net_device *ndev,
				    __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int idx;

	if (is_enetc_rev1(si))
		return -EOPNOTSUPP;

	if (__test_and_clear_bit(vid, si->active_vlans)) {
		idx = enetc_vid_hash_idx(vid);
		enetc_refresh_vlan_ht_filter(si);
		if (!test_bit(idx, si->vlan_ht_filter))
			enetc_msg_vsi_set_vlan_filter(priv, false);
	}

	return 0;
}

static int enetc_vf_set_features(struct net_device *ndev,
				 netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	if (changed & NETIF_F_HW_VLAN_CTAG_FILTER) {
		bool vlan_promisc;

		vlan_promisc = !(features & NETIF_F_HW_VLAN_CTAG_FILTER);
		enetc_msg_vsi_set_vlan_filter(priv, vlan_promisc);
	}

	enetc_set_features(ndev, features);

	return 0;
}

static int enetc_vf_setup_tc(struct net_device *ndev, enum tc_setup_type type,
			     void *type_data)
{
	switch (type) {
	case TC_SETUP_QDISC_MQPRIO:
		return enetc_setup_tc_mqprio(ndev, type_data);
	default:
		return -EOPNOTSUPP;
	}
}

/* Probing/ Init */
static const struct net_device_ops enetc_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_vf_set_mac_addr,
	.ndo_set_rx_mode	= enetc_vf_set_rx_mode,
	.ndo_vlan_rx_add_vid	= enetc_vf_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= enetc_vf_vlan_rx_del_vid,
	.ndo_set_features	= enetc_vf_set_features,
	.ndo_eth_ioctl		= enetc_ioctl,
	.ndo_setup_tc		= enetc_vf_setup_tc,
};

static void enetc_vf_netdev_setup(struct enetc_si *si, struct net_device *ndev,
				  const struct net_device_ops *ndev_ops)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	SET_NETDEV_DEV(ndev, &si->pdev->dev);
	priv->ndev = ndev;
	priv->si = si;
	priv->dev = &si->pdev->dev;
	si->ndev = ndev;

	priv->msg_enable = (NETIF_MSG_IFUP << 1) - 1;
	ndev->netdev_ops = ndev_ops;
	enetc_set_ethtool_ops(ndev);
	ndev->watchdog_timeo = 5 * HZ;

	if (is_enetc_rev1(si)) {
		ndev->max_mtu = ENETC_MAX_MTU;
		priv->max_frags_bd = ENETC_MAX_SKB_FRAGS;
	} else {
		ndev->max_mtu = ENETC4_MAX_MTU;
		priv->active_offloads |= ENETC_F_CHECKSUM | ENETC_F_LSO;
		priv->max_frags_bd = ENETC4_MAX_SKB_FRAGS;
	}

	ndev->hw_features = NETIF_F_SG | NETIF_F_RXCSUM |
			    NETIF_F_HW_VLAN_CTAG_TX |
			    NETIF_F_HW_VLAN_CTAG_RX |
			    NETIF_F_HW_VLAN_CTAG_FILTER |
			    NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 |
			    NETIF_F_GSO_UDP_L4;
	ndev->features = NETIF_F_HIGHDMA | NETIF_F_SG | NETIF_F_RXCSUM |
			 NETIF_F_HW_VLAN_CTAG_TX |
			 NETIF_F_HW_VLAN_CTAG_RX |
			 NETIF_F_HW_CSUM | NETIF_F_TSO | NETIF_F_TSO6 |
			 NETIF_F_GSO_UDP_L4;
	ndev->vlan_features = NETIF_F_SG | NETIF_F_HW_CSUM |
			      NETIF_F_TSO | NETIF_F_TSO6 | NETIF_F_GSO_UDP_L4;

	/* If driver handles unicast address filtering, it should set
	 * IFF_UNICAST_FLT in its priv_flags. (Refer to the description
	 * of the ndo_set_rx_mode())
	 */
	ndev->priv_flags |= IFF_UNICAST_FLT;

	if (si->num_rss)
		ndev->hw_features |= NETIF_F_RXHASH;

	/* pick up primary MAC address from SI */
	enetc_load_primary_mac_addr(&si->hw, ndev);
}

static int enetc_vf_probe(struct pci_dev *pdev,
			  const struct pci_device_id *ent)
{
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	struct enetc_si *si;
	int err;

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, 0);
	if (err)
		return dev_err_probe(&pdev->dev, err, "PCI probing failed\n");

	si = pci_get_drvdata(pdev);

	enetc_get_si_caps(si);

	ndev = alloc_etherdev_mq(sizeof(*priv), ENETC_MAX_NUM_TXQS);
	if (!ndev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "netdev creation failed\n");
		goto err_alloc_netdev;
	}

	enetc_vf_netdev_setup(si, ndev, &enetc_ndev_ops);

	priv = netdev_priv(ndev);

	enetc_init_si_rings_params(priv);

	err = enetc_init_cbdr(si);
	if (err)
		goto err_init_cbdr;

	err = enetc_alloc_si_resources(priv);
	if (err) {
		dev_err(&pdev->dev, "SI resource alloc failed\n");
		goto err_alloc_si_res;
	}

	err = enetc_configure_si(priv);
	if (err) {
		dev_err(&pdev->dev, "Failed to configure SI\n");
		goto err_config_si;
	}

	err = enetc_alloc_msix(priv);
	if (err) {
		dev_err(&pdev->dev, "MSIX alloc failed\n");
		goto err_alloc_msix;
	}

	err = register_netdev(ndev);
	if (err)
		goto err_reg_netdev;

	netif_carrier_off(ndev);

	return 0;

err_reg_netdev:
	enetc_free_msix(priv);
err_config_si:
err_alloc_msix:
	enetc_free_si_resources(priv);
err_alloc_si_res:
	enetc_free_cbdr(si);
err_init_cbdr:
	si->ndev = NULL;
	free_netdev(ndev);
err_alloc_netdev:
	enetc_pci_remove(pdev);

	return err;
}

static void enetc_vf_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_ndev_priv *priv;

	priv = netdev_priv(si->ndev);
	unregister_netdev(si->ndev);

	enetc_free_msix(priv);

	enetc_free_si_resources(priv);
	enetc_free_cbdr(si);

	free_netdev(si->ndev);

	enetc_pci_remove(pdev);
}

static const struct pci_device_id enetc_vf_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_VF) },
	{ PCI_DEVICE(PCI_VENDOR_ID_NXP2, PCI_DEVICE_ID_NXP2_ENETC_VF) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_vf_id_table);

static struct pci_driver enetc_vf_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_vf_id_table,
	.probe = enetc_vf_probe,
	.remove = enetc_vf_remove,
};
module_pci_driver(enetc_vf_driver);

MODULE_DESCRIPTION(ENETC_DRV_NAME_STR);
MODULE_LICENSE("Dual BSD/GPL");
