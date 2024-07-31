// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2017-2019 NXP */

#include <linux/module.h>
#include "enetc.h"

#define ENETC_DRV_NAME_STR "ENETC VF driver"

/* Messaging */
/* Notice this function needs to be called after filling the message body,
 * because CRC16 needs to be calculated here.
 */
static void enetc_msg_vf_fill_common_header(struct enetc_msg_swbd *msg_swbd,
					    u8 class_id, u8 cmd_id, u8 proto_ver,
					    u8 cookie)
{
	struct enetc_msg_header *hdr = msg_swbd->vaddr;
	u8 *data_buf = ((u8 *)msg_swbd->vaddr) + 2; /* skip crc16 field */
	u32 data_size = msg_swbd->size - 2;
	u16 crc16;

	hdr->class_id = class_id;
	hdr->cmd_id = cmd_id;
	hdr->len = ENETC_MSG_EXT_BODY_LEN(msg_swbd->size);

	crc16 = crc_itu_t(ENETC_CRC_INIT, data_buf, data_size);
	hdr->crc16 = htons(crc16);
}

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
	struct device *dev = &si->pdev->dev;
	union enetc_pf_msg pf_msg;
	u32 vsimsgsr;
	int err;

	scoped_guard(mutex, &si->msg_lock) {
		enetc_msg_vsi_write_msg(&si->hw, msg);
		err = read_poll_timeout(enetc_rd, vsimsgsr,
					!(vsimsgsr & ENETC_VSIMSGSR_MB),
					100, 100000, false, &si->hw, ENETC_VSIMSGSR);
	}

	if (err) {
		dev_err(dev, "VSI to PSI message timeout.\n");
		return err;
	}

	/* check for message delivery error */
	if (vsimsgsr & ENETC_VSIMSGSR_MS) {
		dev_err(dev, "Transfer error when copying the data.\n");
		return -EIO;
	}

	pf_msg.code = ENETC_SIMSGSR_GET_MC(vsimsgsr);
	/* Check the user-defined completion status. */
	if (pf_msg.class_id != ENETC_MSG_CLASS_ID_CMD_SUCCESS) {
		switch (pf_msg.class_id) {
		case ENETC_MSG_CLASS_ID_PERMISSION_DENY:
			return -EACCES;
		case ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT:
			err = -EOPNOTSUPP;
			break;
		case ENETC_MSG_CLASS_ID_PSI_BUSY:
			err = -EBUSY;
			break;
		case ENETC_MSG_CLASS_ID_CMD_TIMEOUT:
			err = -ETIME;
			break;
		case ENETC_MSG_CLASS_ID_MAC_FILTER:
			if (pf_msg.class_code == ENETC_PF_RC_MAC_FILTER_NO_RESOURCE)
				return -ENOSPC;

			err = -EINVAL;
			break;
		case ENETC_MSG_CLASS_ID_VLAN_FILTER:
			if (pf_msg.class_code == ENETC_PF_RC_VLAN_FILTER_NO_RESOURCE)
				err = -ENOSPC;

			err = -EINVAL;
			break;
		default:
			err = -EIO;
		}
	}

	if (err)
		dev_err(dev, "VSI command execute error: 0x%04x\n", pf_msg.code);

	return err;
}

static int enetc_msg_vf_register_link_status_notify(struct enetc_si *si, bool notify)
{
	struct device *dev = &si->pdev->dev;
	struct enetc_msg_link_status *msg;
	struct enetc_msg_swbd msg_swbd;
	u8 cmd_id;
	int err;

	msg_swbd.size = ALIGN(sizeof(*msg), ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr)
		return -ENOMEM;

	cmd_id = notify ? ENETC_MSG_REGISTER_LINK_CHANGE_NOTIFY :
			  ENETC_MSG_UNREGISTER_LINK_CHANGE_NOTIFY;
	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_LINK_STATUS,
					cmd_id, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(si, &msg_swbd);

	dma_free_coherent(dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

	return err;
}

static int enetc_msg_vsi_set_primary_mac_addr(struct enetc_ndev_priv *priv,
					      struct sockaddr *saddr)
{
	struct enetc_msg_mac_exact_filter *msg;
	struct enetc_msg_swbd msg_swbd;
	u32 msg_size;
	int err;

	msg_size = struct_size(msg, mac, 1);
	msg_swbd.size = ALIGN(msg_size, ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr) {
		dev_err(priv->dev,
			"Failed to alloc Tx msg (size: %d) for primary MAC\n",
			msg_swbd.size);
		return -ENOMEM;
	}

	msg = (struct enetc_msg_mac_exact_filter *)msg_swbd.vaddr;
	msg->mac_cnt = 1;
	memcpy(&msg->mac[0].addr, saddr->sa_data, ETH_ALEN);
	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_MAC_FILTER,
					ENETC_MSG_SET_PRIMARY_MAC, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

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

static void enetc_msg_vf_set_mac_promisc(struct enetc_ndev_priv *priv,
					 int type, bool en)
{
	struct enetc_msg_mac_promsic_mode *msg;
	struct enetc_msg_swbd msg_swbd;

	if (!(type & ENETC_MAC_FILTER_TYPE_ALL))
		return;

	msg_swbd.size = ALIGN(sizeof(*msg), ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr)
		return;

	msg = (struct enetc_msg_mac_promsic_mode *)msg_swbd.vaddr;
	msg->type = type & ENETC_MAC_FILTER_TYPE_ALL;
	msg->promisc_mode = en ? ENETC_MAC_PROMISC_MODE_ENABLE :
				 ENETC_MAC_PROMISC_MODE_DISABLE;
	/* Delete MAC exact filter and hash filter by default */
	msg->flush_macs = en ? ENETC_MAC_FILTER_FLUSH : 0;
	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_MAC_FILTER,
					ENETC_MSG_SET_MAC_PROMISC_MODE, 0, 0);

	/* send the command and wait */
	enetc_msg_vsi_send(priv->si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);
}

static int enetc_msg_vf_flush_mac_filter(struct net_device *ndev, int type)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_msg_mac_filter_flush *msg;
	struct enetc_msg_swbd msg_swbd;
	int err;

	msg_swbd.size = ALIGN(sizeof(*msg), ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr)
		return -ENOMEM;

	msg = (struct enetc_msg_mac_filter_flush *)msg_swbd.vaddr;
	msg->type = type & ENETC_MAC_FILTER_TYPE_ALL;
	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_MAC_FILTER,
					ENETC_MSG_FLUSH_MAC_ENTRIES, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

	return err;
}

static int enetc_msg_vf_set_mac_exact_filter(struct net_device *ndev, int type)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_msg_mac_exact_filter *msg;
	struct enetc_msg_swbd msg_swbd;
	struct netdev_hw_addr *ha;
	u8 si_mac[ETH_ALEN];
	int mac_cnt = 0;
	u32 msg_size;
	int err;

	enetc_get_si_primary_mac(&priv->si->hw, si_mac);

	netif_addr_lock_bh(ndev);
	if (type & ENETC_MAC_FILTER_TYPE_UC)
		mac_cnt += netdev_uc_count(ndev);

	if (type & ENETC_MAC_FILTER_TYPE_MC)
		mac_cnt += netdev_mc_count(ndev);

	msg_size = struct_size(msg, mac, mac_cnt);
	if (msg_size > ENETC_1KB_SIZE) {
		netif_addr_unlock_bh(ndev);
		return -EOPNOTSUPP;
	}

	msg_swbd.size = ALIGN(msg_size, ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_ATOMIC);
	if (!msg_swbd.vaddr) {
		netif_addr_unlock_bh(ndev);
		return -ENOMEM;
	}

	mac_cnt = 0;
	msg = (struct enetc_msg_mac_exact_filter *)msg_swbd.vaddr;

	if (type & ENETC_MAC_FILTER_TYPE_UC) {
		netdev_for_each_uc_addr(ha, ndev) {
			if (!is_valid_ether_addr(ha->addr) ||
			    ether_addr_equal(ha->addr, si_mac))
				continue;

			ether_addr_copy(msg->mac[mac_cnt++].addr, ha->addr);
		}
	}

	if (type & ENETC_MAC_FILTER_TYPE_MC) {
		netdev_for_each_mc_addr(ha, ndev) {
			if (!is_multicast_ether_addr(ha->addr))
				continue;

			ether_addr_copy(msg->mac[mac_cnt++].addr, ha->addr);
		}
	}
	netif_addr_unlock_bh(ndev);

	msg->mac_cnt = mac_cnt;
	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_MAC_FILTER,
					ENETC_MSG_ADD_EXACT_MAC_ENTRIES, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(priv->si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

	return err;
}

static int enetc_msg_vf_set_mac_hash_filter(struct net_device *ndev,
					    int type, bool clear)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_msg_mac_hash_filter *msg;
	struct enetc_mac_filter *mac_filter;
	struct enetc_msg_swbd msg_swbd;
	struct enetc_si *si = priv->si;
	struct netdev_hw_addr *ha;
	u32 msg_size, tbl_size;
	u64 *hash_tbl_base;
	int err;

	if (type == ENETC_MAC_FILTER_TYPE_ALL)
		tbl_size = ENETC_MADDR_HASH_TBL_SZ * 2;
	else
		tbl_size = ENETC_MADDR_HASH_TBL_SZ;

	msg_size = struct_size(msg, hash_tbl, tbl_size / 32);
	msg_swbd.size = ALIGN(msg_size, ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr)
		return -ENOMEM;

	msg = (struct enetc_msg_mac_hash_filter *)msg_swbd.vaddr;
	msg->type = type & ENETC_MAC_FILTER_TYPE_ALL;
	msg->size = ENETC_MAC_HASH_TABLE_SIZE_64;

	hash_tbl_base = (u64 *)msg->hash_tbl;
	netif_addr_lock_bh(ndev);
	if (type & ENETC_MAC_FILTER_TYPE_UC) {
		if (clear) {
			*hash_tbl_base = 0;
		} else {
			mac_filter = &si->mac_filter[UC];
			enetc_reset_mac_addr_filter(mac_filter);
			netdev_for_each_uc_addr(ha, ndev)
				enetc_add_mac_addr_ht_filter(mac_filter, ha->addr);

			memcpy(hash_tbl_base, mac_filter->mac_hash_table,
			       sizeof(mac_filter->mac_hash_table));
		}

		hash_tbl_base++;
	}

	if (type & ENETC_MAC_FILTER_TYPE_MC) {
		if (clear) {
			*hash_tbl_base = 0;
		} else {
			mac_filter = &si->mac_filter[MC];
			enetc_reset_mac_addr_filter(mac_filter);
			netdev_for_each_mc_addr(ha, ndev)
				enetc_add_mac_addr_ht_filter(mac_filter, ha->addr);

			memcpy(hash_tbl_base, mac_filter->mac_hash_table,
			       sizeof(mac_filter->mac_hash_table));
		}
	}
	netif_addr_unlock_bh(ndev);

	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_MAC_FILTER,
					ENETC_MSG_SET_MAC_HASH_TABLE, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

	return err;
}

static void enetc_vf_set_mac_filter(struct net_device *ndev, int type)
{
	if (!(type & ENETC_MAC_FILTER_TYPE_ALL))
		return;

	enetc_msg_vf_flush_mac_filter(ndev, type);
	if (enetc_msg_vf_set_mac_exact_filter(ndev, type))
		/* Fallback to use MAC hash filter */
		enetc_msg_vf_set_mac_hash_filter(ndev, type, false);
}

static void enetc_vf_do_set_rx_mode(struct work_struct *work)
{
	struct enetc_si *si = container_of(work, struct enetc_si, rx_mode_task);
	struct enetc_ndev_priv *priv = netdev_priv(si->ndev);
	struct net_device *ndev = si->ndev;

	if (ndev->flags & IFF_PROMISC) {
		enetc_msg_vf_set_mac_promisc(priv, ENETC_MAC_FILTER_TYPE_ALL, true);
	} else if (ndev->flags & IFF_ALLMULTI) {
		enetc_msg_vf_set_mac_promisc(priv, ENETC_MAC_FILTER_TYPE_MC, true);
		enetc_msg_vf_set_mac_promisc(priv, ENETC_MAC_FILTER_TYPE_UC, false);
		enetc_vf_set_mac_filter(ndev, ENETC_MAC_FILTER_TYPE_UC);
	} else {
		enetc_msg_vf_set_mac_promisc(priv, ENETC_MAC_FILTER_TYPE_ALL, false);
		enetc_vf_set_mac_filter(ndev, ENETC_MAC_FILTER_TYPE_ALL);
	}
}

static void enetc_vf_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;

	if (is_enetc_rev1(si))
		return;

	queue_work(si->workqueue, &si->rx_mode_task);
}

static int enetc_msg_vf_set_vlan_hash_filter(struct enetc_ndev_priv *priv)
{
	struct enetc_msg_vlan_hash_filter *msg;
	struct enetc_msg_swbd msg_swbd;
	struct enetc_si *si = priv->si;
	u32 msg_size;
	int err;

	msg_size = struct_size(msg, hash_tbl, ENETC_VLAN_HT_SIZE / 32);
	msg_swbd.size = ALIGN(msg_size, ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr)
		return -ENOMEM;

	msg = (struct enetc_msg_vlan_hash_filter *)msg_swbd.vaddr;
	msg->size = ENETC_VLAN_HASH_TABLE_SIZE_64;

	memcpy(msg->hash_tbl, si->vlan_ht_filter, sizeof(si->vlan_ht_filter));

	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_VLAN_FILTER,
					ENETC_MSG_SET_VLAN_HASH_TABLE, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

	return err;
}

static int enetc_msg_vf_set_vlan_promisc(struct enetc_ndev_priv *priv, bool en)
{
	struct enetc_msg_vlan_promsic_mode *msg;
	struct enetc_msg_swbd msg_swbd;
	struct enetc_si *si = priv->si;
	int err;

	msg_swbd.size = ALIGN(sizeof(*msg), ENETC_MSG_ALIGN);
	msg_swbd.vaddr = dma_alloc_coherent(priv->dev, msg_swbd.size,
					    &msg_swbd.dma, GFP_KERNEL);
	if (!msg_swbd.vaddr)
		return -ENOMEM;

	msg = (struct enetc_msg_vlan_promsic_mode *)msg_swbd.vaddr;
	msg->promisc_mode = en ? ENETC_VLAN_PROMISC_MODE_ENABLE :
				 ENETC_VLAN_PROMISC_MODE_DISABLE;
	enetc_msg_vf_fill_common_header(&msg_swbd, ENETC_MSG_CLASS_ID_VLAN_FILTER,
					ENETC_MSG_SET_VLAN_PROMISC_MODE, 0, 0);

	/* send the command and wait */
	err = enetc_msg_vsi_send(si, &msg_swbd);

	dma_free_coherent(priv->dev, msg_swbd.size, msg_swbd.vaddr, msg_swbd.dma);

	return err;
}

static int enetc_vf_vlan_rx_add_vid(struct net_device *ndev,
				    __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int idx, err = 0;

	if (is_enetc_rev1(si))
		return -EOPNOTSUPP;

	__set_bit(vid, si->active_vlans);

	idx = enetc_vid_hash_idx(vid);
	if (!__test_and_set_bit(idx, si->vlan_ht_filter))
		err = enetc_msg_vf_set_vlan_hash_filter(priv);

	if (err) {
		__clear_bit(idx, si->vlan_ht_filter);
		__clear_bit(vid, si->active_vlans);
	}

	return err;
}

static int enetc_vf_vlan_rx_del_vid(struct net_device *ndev,
				    __be16 prot, u16 vid)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	int idx, err = 0;

	if (is_enetc_rev1(si))
		return -EOPNOTSUPP;

	if (__test_and_clear_bit(vid, si->active_vlans)) {
		idx = enetc_vid_hash_idx(vid);
		enetc_refresh_vlan_ht_filter(si);
		if (!test_bit(idx, si->vlan_ht_filter))
			err = enetc_msg_vf_set_vlan_hash_filter(priv);

		if (err) {
			__set_bit(idx, si->vlan_ht_filter);
			__set_bit(vid, si->active_vlans);
		}
	}

	return err;
}

static int enetc_vf_set_features(struct net_device *ndev,
				 netdev_features_t features)
{
	netdev_features_t changed = ndev->features ^ features;
	struct enetc_ndev_priv *priv = netdev_priv(ndev);

	if (changed & NETIF_F_HW_VLAN_CTAG_FILTER) {
		bool vlan_promisc;

		vlan_promisc = !(features & NETIF_F_HW_VLAN_CTAG_FILTER);
		enetc_msg_vf_set_vlan_promisc(priv, vlan_promisc);
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
	.ndo_bpf		= enetc_setup_bpf,
	.ndo_xdp_xmit		= enetc_xdp_xmit,
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
		priv->active_offloads |= ENETC_F_CHECKSUM;
		priv->max_frags_bd = ENETC4_MAX_SKB_FRAGS;
		priv->shared_tx_rings = true;
	}

	if (si->hw_features & ENETC_SI_F_LSO)
		priv->active_offloads |= ENETC_F_LSO;

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

	ndev->xdp_features = NETDEV_XDP_ACT_BASIC | NETDEV_XDP_ACT_REDIRECT |
			     NETDEV_XDP_ACT_NDO_XMIT | NETDEV_XDP_ACT_RX_SG |
			     NETDEV_XDP_ACT_NDO_XMIT_SG;

	/* If driver handles unicast address filtering, it should set
	 * IFF_UNICAST_FLT in its priv_flags. (Refer to the description
	 * of the ndo_set_rx_mode())
	 */
	ndev->priv_flags |= IFF_UNICAST_FLT;

	if (si->num_rss) {
		ndev->hw_features |= NETIF_F_RXHASH;
		ndev->features |= NETIF_F_RXHASH;
	}

	if (si->hw_features & ENETC_SI_F_RSC)
		ndev->hw_features |= NETIF_F_LRO;

	/* pick up primary MAC address from SI */
	enetc_load_primary_mac_addr(&si->hw, ndev);
}

static void enetc_vf_enable_mr_int(struct enetc_hw *hw, bool en)
{
	u32 val;

	val = enetc_rd(hw, ENETC_VSIIER);
	val = u32_replace_bits(val, en ? 1 : 0, VSIIER_MRIE);
	enetc_wr(hw, ENETC_VSIIER, val);
}

static void enetc_vf_msg_handle_link_status(struct enetc_si *si, u8 class_code)
{
	struct net_device *ndev = si->ndev;

	switch (class_code) {
	case ENETC_PF_NC_LINK_STATUS_UP:
		if (!netif_carrier_ok(ndev)) {
			netif_carrier_on(ndev);
			netdev_info(ndev, "Link is Up\n");
		}
		break;
	case ENETC_PF_NC_LINK_STATUS_DOWN:
		if (netif_carrier_ok(ndev)) {
			netif_carrier_off(ndev);
			netdev_info(ndev, "Link is Down\n");
		}
		break;
	}
}

static void enetc_vf_msg_task(struct work_struct *work)
{
	struct enetc_si *si = container_of(work, struct enetc_si, msg_task);
	struct enetc_hw *hw = &si->hw;
	union enetc_pf_msg pf_msg;
	u32 val;

	val = enetc_rd(hw, ENETC_VSIMSGRR);
	pf_msg.code = VSIMSGRR_GET_MC(val);
	switch (pf_msg.class_id) {
	case ENETC_MSG_CLASS_ID_LINK_STATUS:
		enetc_vf_msg_handle_link_status(si, pf_msg.class_code);
		break;
	default:
		dev_err(&si->pdev->dev,
			"Unknown Message Class ID (0x%02x) from PF\n",
			pf_msg.class_id);
	}

	enetc_wr(hw, ENETC_VSIIDR, VSIIDR_MR);
	enetc_vf_enable_mr_int(hw, true);
}

static irqreturn_t enetc_vf_msg_msix_handler(int irq, void *data)
{
	struct enetc_si *si = (struct enetc_si *)data;

	enetc_vf_enable_mr_int(&si->hw, false);
	queue_work(si->workqueue, &si->msg_task);

	return IRQ_HANDLED;
}

static int enetc_vf_register_msg_msix(struct enetc_si *si)
{
	int irq, err;

	snprintf(si->msg_int_name, sizeof(si->msg_int_name), "%s-pfmsg",
		 si->ndev->name);
	irq = pci_irq_vector(si->pdev, ENETC_SI_INT_IDX);
	err = request_irq(irq, enetc_vf_msg_msix_handler, 0,
			  si->msg_int_name, si);
	if (err) {
		dev_err(&si->pdev->dev,
			"VF messaging: request_irq() failed!\n");
		return err;
	}

	/* set one IRQ entry for PSI-to-VSI messaging */
	enetc_wr(&si->hw, ENETC_SIMSIVR, ENETC_SI_INT_IDX);

	/* Enable message received interrupt */
	enetc_vf_enable_mr_int(&si->hw, true);

	return 0;
}

static void enetc_vf_free_msg_msix(struct enetc_si *si)
{
	int irq = pci_irq_vector(si->pdev, ENETC_SI_INT_IDX);

	cancel_work_sync(&si->msg_task);
	enetc_vf_enable_mr_int(&si->hw, false);
	free_irq(irq, si);
}

static int enetc_vf_probe(struct pci_dev *pdev,
			  const struct pci_device_id *ent)
{
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	struct enetc_si *si;
	char wq_name[24];
	int err;

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, 0);
	if (err)
		return dev_err_probe(&pdev->dev, err, "PCI probing failed\n");

	si = pci_get_drvdata(pdev);
	INIT_WORK(&si->rx_mode_task, enetc_vf_do_set_rx_mode);
	snprintf(wq_name, sizeof(wq_name), "enetc-%s", pci_name(pdev));
	si->workqueue = create_singlethread_workqueue(wq_name);
	if (!si->workqueue) {
		err = -ENOMEM;
		goto err_create_wq;
	}

	if (!is_enetc_rev1(si)) {
		INIT_WORK(&si->msg_task, enetc_vf_msg_task);
		si->vf_register_msg_msix = enetc_vf_register_msg_msix;
		si->vf_free_msg_msix = enetc_vf_free_msg_msix;
		si->vf_register_link_status_notify =
			enetc_msg_vf_register_link_status_notify;
	}

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
	destroy_workqueue(si->workqueue);
err_create_wq:
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

	destroy_workqueue(si->workqueue);

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
