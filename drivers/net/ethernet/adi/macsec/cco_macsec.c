// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2023-2025, Analog Devices Incorporated, All Rights Reserved
 */

#include "cco_macsec.h"
#include <net/sock.h>

static u32  default_txsc_pn_thr = 0xff000000;   module_param(default_txsc_pn_thr, uint, 0644);
static u32  debug_max_framesize = 0;            module_param(debug_max_framesize, uint, 0644);
static bool debug_xpn = false;                  module_param(debug_xpn, bool, 0644);
static bool debug_sw_macsec = false;            module_param(debug_sw_macsec, bool, 0644);

// Find the SecY index from the provided ctx->secy pointer and netdev_priv(ctx->netdev) secy_array.
static bool get_secy(struct macsec_context *ctx, u32 *secy_index)
{
	const struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	int i;

	for (i = 0; i < macsec_priv->capabilities.no_of_secys && i < CCO_MACSEC_SECY_MAX; ++i)
		if (ctx->secy == macsec_priv->secy_array[i]) {
			*secy_index = i;
			return true;
		}
	return false;
}

// Find the first free SecY index
static bool get_free_secy(struct macsec_context *ctx, u32 *secy_index)
{
	const struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	int i;

	for (i = 0; i < macsec_priv->capabilities.no_of_secys && i < CCO_MACSEC_SECY_MAX; ++i)
		if (!macsec_priv->secy_array[i]) {
			*secy_index = i;
			return true;
		}
	return false;
}

// Count number of SecYs
static u32 count_secy(struct net_device *netdev)
{
	const struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(netdev);
	int i;
	u32 cnt = 0;

	for (i = 0; i < macsec_priv->capabilities.no_of_secys && i < CCO_MACSEC_SECY_MAX; ++i)
	if (macsec_priv->secy_array[i]) {
		cnt++;
	}
	return cnt;
}

// Find SecY index from SecY pointer
static bool find_secy(struct macsec_context *ctx, u32 *secy_index)
{
	const struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	const struct macsec_secy *secy = ctx->secy;
	int i;

	for (i = 0; i < macsec_priv->capabilities.no_of_secys && i < CCO_MACSEC_SECY_MAX; ++i)
		if (macsec_priv->secy_array[i] == secy) {
			*secy_index = i;
			return true;
		}
	return false;
}

// Find the first free Rx-SC index
static bool get_free_rxsc(struct macsec_context *ctx, u32 secy_index, u32 *rxsc_index)
{
	const struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	int i;

	for (i = 0; i < macsec_priv->capabilities.maxRxChannels && i < CCO_MACSEC_RXSC_MAX; ++i)
		if (!macsec_priv->rxsc_array[secy_index][i]) {
			*rxsc_index = i;
			return true;
		}
	return false;
}

// Find Rx-SC index from Rx-SC pointer
static bool find_rxsc(const struct cco_macsec_priv *macsec_priv, const struct macsec_rx_sc *rx_sc,
		      u32 secy_index, u32 *rxsc_index)
{
	int i;

	for (i = 0; i < macsec_priv->capabilities.maxRxChannels && i < CCO_MACSEC_RXSC_MAX; ++i)
		if (macsec_priv->rxsc_array[secy_index][i] == rx_sc) {
			*rxsc_index = i;
			return true;
		}
	return false;
}

// write SecY + Tx-SC registers
static void write_secy_txsc(struct net_device *netdev,
			    const struct macsec_secy *secy, const u32 secy_index,
			    const u8 disable, const u8 sa_enabled,
			    const u8 vlan_in_clear, const u8 conf_offs)
{
	u8 validate_frames = 0, val;
	u32 max_framesize;

	if (debug_max_framesize)
		max_framesize = debug_max_framesize;
	else
		cco_macsec_max_framesize_get(netdev, &max_framesize);

	// Configure the SecY registers:
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_PORT_CONFIG_BASE_ADDR,
			  disable ? 0 : (vlan_in_clear << SECY_CONFIG_PORT_CONFIG_VLANINCLEAR_WR_SHIFT) |
			  SECY_CONFIG_PORT_CONFIG_CONTROLLEDPORTENABLED_WR_MASK);
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_TX_CONFIG_BASE_ADDR,
			  disable ? 0 :
			  (secy->tx_sc.scb ? SECY_CONFIG_TX_CONFIG_USESCB_WR_MASK : 0)                |
			  (secy->tx_sc.end_station ? SECY_CONFIG_TX_CONFIG_USEES_WR_MASK : 0)         |
			  (secy->tx_sc.send_sci ? SECY_CONFIG_TX_CONFIG_ALWAYSINCLUDESCI_WR_MASK : 0) |
			  (secy->protect_frames ? SECY_CONFIG_TX_CONFIG_PROTECTFRAMES_WR_MASK : 0));
	if (secy->validate_frames == MACSEC_VALIDATE_DISABLED)
		validate_frames = 1; // disabled
	else if (secy->validate_frames == MACSEC_VALIDATE_CHECK)
		validate_frames = 2; // check
	else if (secy->validate_frames == MACSEC_VALIDATE_STRICT)
		validate_frames = 3; // strict
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_RX_CONFIG_BASE_ADDR,
			  disable ? 0 :
			  (secy->replay_protect ? SECY_CONFIG_RX_CONFIG_REPLAYPROTECT_WR_MASK : 0) |
			  (validate_frames << SECY_CONFIG_RX_CONFIG_VALIDATEFRAMES_WR_SHIFT));
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_RX_REPLAYWINDOW_WR_BASE_ADDR,
			  disable ? 0 : secy->replay_window);
	if (secy->key_len == (128/8)) {
		if (!secy->xpn)
			val = 0; // 0x0: AES-GCM-128
		else
			val = 2; // 0x2: AES-GCM-XPN-128
	} else { // secy->key_len == (256/8))
		if (!secy->xpn)
			val = 1; // 0x1: AES-GCM-256
		else
			val = 3; // 0x3: AES-GCM-XPN-256
	}
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_CIPHERSUITE_CONFIG_BASE_ADDR,
			  disable ? 0 :
			  (conf_offs << SECY_CONFIG_CIPHERSUITE_CONFIG_CONFIDENTIALITYOFFSET_WR_SHIFT)  |
			  (val << SECY_CONFIG_CIPHERSUITE_CONFIG_CURRENTCIPHERSUITE_WR_SHIFT)   |
			  (0 << SECY_CONFIG_CIPHERSUITE_CONFIG_REQUIRECONFIDENTIALITY_WR_SHIFT));
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_MAX_FRAME_LENGHT_WR_BASE_ADDR,
			  disable ? 0 : max_framesize);
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_CONFIG_CTRL_BASE_ADDR,
			  SECY_CONFIG_CONFIG_CTRL_CIPHERSUITE_CONFIG_EN_MASK |
			  SECY_CONFIG_CONFIG_CTRL_RX_CONFIG_EN_MASK          |
			  SECY_CONFIG_CONFIG_CTRL_TX_CONFIG_EN_MASK          |
			  SECY_CONFIG_CONFIG_CTRL_PORT_CONFIG_EN_MASK        |
			  SECY_CONFIG_CONFIG_CTRL_WR_TRIGGER_MASK            |
			  (secy_index << SECY_CONFIG_CONFIG_CTRL_SECY_INDEX_SHIFT));

	// Configure the Tx-SC registers:
	cco_macsec_reg_wr(netdev, TRANSMITSC_BASE_ADDR + TRANSMITSC_TX_SC_SCI_0_WR_BASE_ADDR,
			  disable ? 0 : ((((u8*)&secy->sci)[4] << 24) | (((u8*)&secy->sci)[5] << 16) |
					 (((u8*)&secy->sci)[6] <<  8) | ((u8*)&secy->sci)[7]));
	cco_macsec_reg_wr(netdev, TRANSMITSC_BASE_ADDR + TRANSMITSC_TX_SC_SCI_1_WR_BASE_ADDR,
			  disable ? 0 : ((((u8*)&secy->sci)[0] << 24) | (((u8*)&secy->sci)[1] << 16) |
					 (((u8*)&secy->sci)[2] <<  8) | ((u8*)&secy->sci)[3]));
	if ((sa_enabled & (0x10 << secy->tx_sc.encoding_sa)) && !disable) {
		cco_macsec_reg_wr(netdev, TRANSMITSC_BASE_ADDR + TRANSMITSC_TX_SC_TRANSMITSC_CFG_BASE_ADDR,
				  ((1 << secy->tx_sc.encoding_sa) << TRANSMITSC_TX_SC_TRANSMITSC_CFG_ENABLETRANSMIT_SA_WR_SHIFT));
	} else {
		cco_macsec_reg_wr(netdev, TRANSMITSC_BASE_ADDR + TRANSMITSC_TX_SC_TRANSMITSC_CFG_BASE_ADDR, 0);
	}
	cco_macsec_reg_wr(netdev, TRANSMITSC_BASE_ADDR + TRANSMITSC_TX_SC_CTRL_BASE_ADDR,
			  TRANSMITSC_TX_SC_CTRL_WR_TRIGGER_MASK |
			  (secy_index << TRANSMITSC_TX_SC_CTRL_SECY_INDEX_SHIFT));
}

static int cco_macsec_add_secy(struct macsec_context *ctx)
{
	struct macsec_secy *secy = ctx->secy;
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u32 secy_index;
	u8 vlan_in_clear, conf_offs;
	u32 secy_cnt, val;

	if (debug_xpn)
		secy->xpn = true;

	// In the prepare phase, check that ctx->secy->xpn, ctx->secy->key_len and ctx->secy->icv_len
	// matches MACsec IP capabilities; otherwise fail. Also check that a new SecY index can be allocated.
	if (secy->xpn) {
		if (!(macsec_priv->capabilities.available_ciphersuites &
		      (CCO_CS_AES_GCM_XPN_128 | CCO_CS_AES_GCM_XPN_256)))
			return -EOPNOTSUPP;
	} else if (!(macsec_priv->capabilities.available_ciphersuites &
		     (CCO_CS_AES_GCM_128 | CCO_CS_AES_GCM_256)))
		return -EOPNOTSUPP;
	if ((secy->key_len == (128/8)) &&
	    !(macsec_priv->capabilities.available_ciphersuites &
	      (CCO_CS_AES_GCM_128 | CCO_CS_AES_GCM_XPN_128)))
		return -EOPNOTSUPP;
	if ((secy->key_len == (256/8)) &&
	    !(macsec_priv->capabilities.available_ciphersuites &
	      (CCO_CS_AES_GCM_256 | CCO_CS_AES_GCM_XPN_256)))
		return -EOPNOTSUPP;
	if (secy->icv_len != macsec_priv->capabilities.ICVLength)
		return -EOPNOTSUPP;
	if (!secy->netdev)
		return EINVAL;
	if (!get_free_secy(ctx, &secy_index))
		return -ENOSPC;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	secy_cnt = count_secy(ctx->netdev);
	if (secy_cnt == 0) {
		val = cco_macsec_reg_rd(ctx->netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR);
		cco_macsec_reg_wr(ctx->netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR,
				val | MACSEC_CORE_GENERAL_CTRL_MACSEC_EN_MASK);
	}
	memset(&macsec_priv->dev_stats[secy_index], 0, sizeof(macsec_priv->dev_stats[secy_index]));
	memset(&macsec_priv->txsc_stats[secy_index], 0, sizeof(macsec_priv->txsc_stats[secy_index]));
	memset(&macsec_priv->port_stats[secy_index], 0, sizeof(macsec_priv->port_stats[secy_index]));
	memset(&macsec_priv->uport_stats[secy_index], 0, sizeof(macsec_priv->uport_stats[secy_index]));
	memset(&macsec_priv->ext_port_stats[secy_index], 0, sizeof(macsec_priv->ext_port_stats[secy_index]));

	vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
	conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
	write_secy_txsc(ctx->netdev, secy, secy_index, 0, macsec_priv->sa_enabled[secy_index][0], vlan_in_clear, conf_offs);

	macsec_priv->secy_array[secy_index] = secy;
	macsec_priv->secy_ifIndex[secy_index] = secy->netdev->ifindex;
	macsec_priv->txsc_ext[secy_index].createdTime = jiffies;
	macsec_priv->txsc_ext[secy_index].startedTime =
		macsec_priv->txsc_ext[secy_index].createdTime;
	macsec_priv->txsc_ext[secy_index].stoppedTime =
		macsec_priv->txsc_ext[secy_index].createdTime;
	return 0;
}

static int cco_macsec_upd_secy(struct macsec_context *ctx)
{
	struct macsec_secy *secy = ctx->secy;
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u32 secy_index;
	u8 vlan_in_clear, conf_offs;

	if (debug_xpn)
		secy->xpn = true;

	// In the prepare phase, check that ctx->secy->xpn, ctx->secy->key_len and ctx->secy->icv_len
	// matches MACsec IP capabilities; otherwise fail. Also check that SecY index can be found.
	if (secy->xpn &&
	    (macsec_priv->capabilities.available_ciphersuites & (CCO_CS_AES_GCM_XPN_128 | CCO_CS_AES_GCM_XPN_256)) == 0)
		return -EOPNOTSUPP;
	if (secy->key_len == (128/8) &&
	    (macsec_priv->capabilities.available_ciphersuites & (CCO_CS_AES_GCM_128 | CCO_CS_AES_GCM_XPN_128)) == 0)
		return -EOPNOTSUPP;
	if (secy->key_len == (256/8) &&
	    (macsec_priv->capabilities.available_ciphersuites & (CCO_CS_AES_GCM_256 | CCO_CS_AES_GCM_XPN_256)) == 0)
		return -EOPNOTSUPP;
	if (secy->icv_len != macsec_priv->capabilities.ICVLength)
		return -EOPNOTSUPP;
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;
	if (!secy->netdev)
		return EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
	conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
	write_secy_txsc(ctx->netdev, secy, secy_index, 0, macsec_priv->sa_enabled[secy_index][0], vlan_in_clear, conf_offs);
	macsec_priv->secy_ifIndex[secy_index] = secy->netdev->ifindex;

	return 0;
}

static void write_rxsc(struct net_device *netdev,
		       struct macsec_rx_sc *rx_sc, const u32 secy_index,
		       const u32 peer_index, const u8 disable)
{
	// Configure the Rx-SC registers:
	cco_macsec_reg_wr(netdev, RECEIVESC_BASE_ADDR + RECEIVESC_RX_SC_SCI_0_WR_BASE_ADDR,
			  disable ? 0 : ((((u8*)&rx_sc->sci)[4] << 24) | (((u8*)&rx_sc->sci)[5] << 16) |
					 (((u8*)&rx_sc->sci)[6] <<  8) | ((u8*)&rx_sc->sci)[7]));
	cco_macsec_reg_wr(netdev, RECEIVESC_BASE_ADDR + RECEIVESC_RX_SC_SCI_1_WR_BASE_ADDR,
			  disable ? 0 : ((((u8*)&rx_sc->sci)[0] << 24) | (((u8*)&rx_sc->sci)[1] << 16) |
					 (((u8*)&rx_sc->sci)[2] <<  8) | ((u8*)&rx_sc->sci)[3]));
	cco_macsec_reg_wr(netdev, RECEIVESC_BASE_ADDR + RECEIVESC_RX_SC_CTRL_BASE_ADDR,
			  RECEIVESC_RX_SC_CTRL_WR_TRIGGER_MASK        |
			  (peer_index << RECEIVESC_RX_SC_CTRL_PEER_INDEX_SHIFT) |
			  (secy_index << RECEIVESC_RX_SC_CTRL_SECY_INDEX_SHIFT));
}

static void clear_rxsa(struct macsec_context *ctx, struct cco_macsec_priv *macsec_priv,
		       const u32 secy_index, const u32 peer_index, const u8 assoc_num, const u32 key_index)
{
	// Configure Rx-SA registers (disable):
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_AN_BASE_ADDR,
			  (assoc_num << RECEIVESA_RX_SA_AN_VAL_SHIFT));
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_KEY_INDEX_WR_BASE_ADDR,
			  key_index << RECEIVESA_RX_SA_KEY_INDEX_WR_VAL_SHIFT);
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_RECEIVESA_CFG_BASE_ADDR, 0);
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
			  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
			  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
			  RECEIVESA_RX_SA_CTRL_WR_TRIGGER_MASK);

	macsec_priv->sa_enabled[secy_index][peer_index] &= ~(1 << assoc_num);

	macsec_priv->key_refcnt[key_index]--;
	if (macsec_priv->key_refcnt[key_index] == 0)
		macsec_priv->key_use[key_index] = 0;
	macsec_priv->rxsa_ext[secy_index][peer_index][assoc_num].createdTime = 0;
	macsec_priv->rxsa_ext[secy_index][peer_index][assoc_num].startedTime = 0;
	macsec_priv->rxsa_ext[secy_index][peer_index][assoc_num].stoppedTime = 0;
}

static void clear_rxsc(struct macsec_context *ctx, struct cco_macsec_priv *macsec_priv,
		       struct macsec_rx_sc *rx_sc,
		       const u32 secy_index, const u32 peer_index)
{
	struct macsec_rx_sa *rx_sa;
	u8 sa_ix;
	u32 i;
	int key_index;

	// clear the Rx-SC registers:
	write_rxsc(ctx->netdev, rx_sc, secy_index, peer_index, 1);

	macsec_priv->rxsc_array[secy_index][peer_index] = NULL;
	macsec_priv->rxsc_ext[secy_index][peer_index].createdTime = 0;
	macsec_priv->rxsc_ext[secy_index][peer_index].startedTime = 0;
	macsec_priv->rxsc_ext[secy_index][peer_index].stoppedTime = 0;

	// Delete any Rx-SA's also:
	for (sa_ix = 0; sa_ix < MACSEC_NUM_AN; ++sa_ix) {
		rx_sa = rtnl_dereference(rx_sc->sa[sa_ix]);
		if (!rx_sa)
			continue;
		// check if rx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
		// and whether it is enabled for Rx (check key_use).
		key_index = -1;
		for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_rx && i < CCO_MACSEC_KEYS; ++i) {
			if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_RX))
				continue;
			if (memcmp(rx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
				// found key.id
				key_index = i;
				break;
			}
		}
		if (key_index < 0)
			continue;
		clear_rxsa(ctx, macsec_priv, secy_index, peer_index, sa_ix, key_index);
	}
}

static void clear_txsa(struct macsec_context *ctx, struct cco_macsec_priv *macsec_priv,
		       const u32 secy_index, const u8 assoc_num, const u32 key_index)
{
	// Configure Tx-SA registers (disable):
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
			  (assoc_num << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
			  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR, 0);
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
			  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
			  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);

	macsec_priv->sa_enabled[secy_index][0] &= ~(0x10 << assoc_num);

	macsec_priv->key_refcnt[key_index]--;
	if (macsec_priv->key_refcnt[key_index] == 0)
		macsec_priv->key_use[key_index] = 0;
	macsec_priv->txsa_ext[secy_index][assoc_num].createdTime = 0;
	macsec_priv->txsa_ext[secy_index][assoc_num].startedTime = 0;
	macsec_priv->txsa_ext[secy_index][assoc_num].stoppedTime = 0;
}

static int cco_macsec_del_secy(struct macsec_context *ctx)
{
	struct macsec_secy *secy = ctx->secy;
	struct macsec_rx_sc *rx_sc;
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_tx_sa *tx_sa;
	u32 secy_index, peer_index, i, secy_cnt, val;
	u8 sa_ix;
	int key_index;

	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	write_secy_txsc(ctx->netdev, secy, secy_index, 1, 0, 0, 0);

	// Delete any Rx-SCs (and Rx-SA's):
	for (rx_sc = rcu_dereference_bh(secy->rx_sc); rx_sc;
	     rx_sc = rcu_dereference_bh(rx_sc->next)) {
		// get the peer_index:
		if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
			continue;
		clear_rxsc(ctx, macsec_priv, rx_sc, secy_index, peer_index);
	}
	// Delete any Tx-SA's:
	for (sa_ix = 0; sa_ix < MACSEC_NUM_AN; ++sa_ix) {
		tx_sa = rcu_dereference_bh(secy->tx_sc.sa[sa_ix]);
		if (!tx_sa)
			continue;
		// check if tx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
		// and whether it is enabled for Tx (check key_use).
		key_index = -1;
		for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_tx && i < CCO_MACSEC_KEYS; ++i) {
			if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_TX))
				continue;
			if (memcmp(tx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
				// found key.id
				key_index = i;
				break;
			}
		}
		if (key_index < 0)
			continue;
		clear_txsa(ctx, macsec_priv, secy_index, sa_ix, key_index);
	}

	macsec_priv->secy_vlan_in_clear[secy_index] = 0;
	macsec_priv->secy_array[secy_index] = NULL;
	macsec_priv->secy_ifIndex[secy_index] = 0;
	macsec_priv->txsc_ext[secy_index].createdTime = 0;
	macsec_priv->txsc_ext[secy_index].startedTime = 0;
	macsec_priv->txsc_ext[secy_index].stoppedTime = 0;
	secy_cnt = count_secy(ctx->netdev);
	if (secy_cnt == 0) {
		val = cco_macsec_reg_rd(ctx->netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR);
		cco_macsec_reg_wr(ctx->netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR,
				val & ~MACSEC_CORE_GENERAL_CTRL_MACSEC_EN_MASK);
	}
	return 0;
}

static int cco_macsec_add_rxsc(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u8 disable = ctx->rx_sc->active ? 0 : 1;
	u32 secy_index, peer_index;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// check that a new Rx-SC can be added:
	if (!get_free_rxsc(ctx, secy_index, &peer_index))
		return -ENOSPC;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	memset(&macsec_priv->rxsc_stats[secy_index][peer_index], 0, sizeof(macsec_priv->rxsc_stats[secy_index][peer_index]));

	write_rxsc(ctx->netdev, ctx->rx_sc, secy_index, peer_index, disable);

	macsec_priv->rxsc_array[secy_index][peer_index] = ctx->rx_sc;
	macsec_priv->rxsc_ext[secy_index][peer_index].createdTime = jiffies;
	macsec_priv->rxsc_ext[secy_index][peer_index].startedTime =
		macsec_priv->rxsc_ext[secy_index][peer_index].createdTime;
	macsec_priv->rxsc_ext[secy_index][peer_index].stoppedTime =
		macsec_priv->rxsc_ext[secy_index][peer_index].createdTime;
	return 0;
}

static int cco_macsec_upd_rxsc(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sc *rx_sc = ctx->rx_sc;
	u8 disable = rx_sc->active ? 0 : 1;
	u32 secy_index, peer_index;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	// update the Rx-SC registers:
	write_rxsc(ctx->netdev, rx_sc, secy_index, peer_index, disable);
	if (disable)
		macsec_priv->rxsc_ext[secy_index][peer_index].stoppedTime = jiffies;
	else if (macsec_priv->sa_enabled[secy_index][peer_index] & 0x0f)
		macsec_priv->rxsc_ext[secy_index][peer_index].startedTime = jiffies;

	return 0;
}

static int cco_macsec_del_rxsc(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sc *rx_sc = ctx->rx_sc;
	u32 secy_index, peer_index;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	clear_rxsc(ctx, macsec_priv, rx_sc, secy_index, peer_index);
	return 0;
}

static int cco_macsec_add_rxsa(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sc *rx_sc = ctx->sa.rx_sa->sc;
	u8 disable = ctx->sa.rx_sa->active ? 0 : 1, val;
	u32 secy_index, peer_index, i;
	int key_index = -1;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	// check if ctx->sa.rx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
	// and whether it is enabled for Rx (check key_use).
	for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_rx && i < CCO_MACSEC_KEYS; ++i) {
		if (!macsec_priv->key_use[i]) {
			if (key_index < 0)
				key_index = i; // first free entry
			continue;
		}
		if (memcmp(ctx->sa.rx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
			// found key.id
			key_index = i;
			break;
		}
	}
	if (key_index < 0)
		// not found and no room for a new
		return -ENOSPC;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	if (!(macsec_priv->key_use[key_index] & CCO_MACSEC_KEY_RX)) {
		u8 key_tx = (macsec_priv->key_use[key_index] & CCO_MACSEC_KEY_TX) ? 1 : 0;
		macsec_priv->key_use[key_index] |= CCO_MACSEC_KEY_RX;
		memcpy(macsec_priv->key_id_table[key_index], ctx->sa.rx_sa->key.id, MACSEC_KEYID_LEN);
		// Configure the Rx-SA key:
		if (ctx->secy->key_len == (128/8)) {
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_0_BASE_ADDR,
					  (ctx->sa.key[12] << 24) | (ctx->sa.key[13] << 16) |
					  (ctx->sa.key[14] << 8) | ctx->sa.key[15]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_1_BASE_ADDR,
					  (ctx->sa.key[8] << 24) | (ctx->sa.key[9] << 16) |
					  (ctx->sa.key[10] << 8) | ctx->sa.key[11]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_2_BASE_ADDR,
					  (ctx->sa.key[4] << 24) | (ctx->sa.key[5] << 16) |
					  (ctx->sa.key[6] << 8) | ctx->sa.key[7]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_3_BASE_ADDR,
					  (ctx->sa.key[0] << 24) | (ctx->sa.key[1] << 16) |
					  (ctx->sa.key[2] << 8) | ctx->sa.key[3]);
			if (!ctx->secy->xpn)
				val = 0; // 0x0: AES-GCM-128
			else
				val = 2; // 0x2: AES-GCM-XPN-128
		} else {
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_0_BASE_ADDR,
					  (ctx->sa.key[28] << 24) | (ctx->sa.key[29] << 16) |
					  (ctx->sa.key[30] << 8) | ctx->sa.key[31]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_1_BASE_ADDR,
					  (ctx->sa.key[24] << 24) | (ctx->sa.key[25] << 16) |
					  (ctx->sa.key[26] << 8) | ctx->sa.key[27]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_2_BASE_ADDR,
					  (ctx->sa.key[20] << 24) | (ctx->sa.key[21] << 16) |
					  (ctx->sa.key[22] << 8) | ctx->sa.key[23]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_3_BASE_ADDR,
					  (ctx->sa.key[16] << 24) | (ctx->sa.key[17] << 16) |
					  (ctx->sa.key[18] << 8) | ctx->sa.key[19]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_4_BASE_ADDR,
					  (ctx->sa.key[12] << 24) | (ctx->sa.key[13] << 16) |
					  (ctx->sa.key[14] << 8) | ctx->sa.key[15]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_5_BASE_ADDR,
					  (ctx->sa.key[8] << 24) | (ctx->sa.key[9] << 16) |
					  (ctx->sa.key[10] << 8) | ctx->sa.key[11]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_6_BASE_ADDR,
					  (ctx->sa.key[4] << 24) | (ctx->sa.key[5] << 16) |
					  (ctx->sa.key[6] << 8) | ctx->sa.key[7]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_7_BASE_ADDR,
					  (ctx->sa.key[0] << 24) | (ctx->sa.key[1] << 16) |
					  (ctx->sa.key[2] << 8) | ctx->sa.key[3]);
			if (!ctx->secy->xpn)
				val = 1; // 0x1: AES-GCM-256
			else
				val = 3; // 0x3: AES-GCM-XPN-256
		}
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SALT_0_BASE_ADDR,
				  (ctx->sa.rx_sa->key.salt.bytes[8] << 24) | (ctx->sa.rx_sa->key.salt.bytes[9] << 16) |
				  (ctx->sa.rx_sa->key.salt.bytes[10] << 8) | ctx->sa.rx_sa->key.salt.bytes[11]);
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SALT_1_BASE_ADDR,
				  (ctx->sa.rx_sa->key.salt.bytes[4] << 24) | (ctx->sa.rx_sa->key.salt.bytes[5] << 16) |
				  (ctx->sa.rx_sa->key.salt.bytes[6] << 8) | ctx->sa.rx_sa->key.salt.bytes[7]);
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SALT_2_BASE_ADDR,
				  (ctx->sa.rx_sa->key.salt.bytes[0] << 24) | (ctx->sa.rx_sa->key.salt.bytes[1] << 16) |
				  (ctx->sa.rx_sa->key.salt.bytes[2] << 8) | ctx->sa.rx_sa->key.salt.bytes[3]);
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_CTRL_BASE_ADDR,
				  (key_index << CIPHERSUITE_CS_CTRL_INDEX_SHIFT)     |
				  (key_tx << CIPHERSUITE_CS_CTRL_TRANSMIT_SEL_SHIFT) |
				  (val << CIPHERSUITE_CS_CTRL_AES_MODE_SEL_SHIFT)    |
				  CIPHERSUITE_CS_CTRL_RECEIVE_SEL_MASK               |
				  CIPHERSUITE_CS_CTRL_WR_TRIGGER_MASK);
	}
	macsec_priv->key_refcnt[key_index]++;

	// Configure Rx-SA registers:
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_AN_BASE_ADDR,
			  (ctx->sa.assoc_num << RECEIVESA_RX_SA_AN_VAL_SHIFT));
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_NEXTPN_BASE_ADDR,
			  (ctx->sa.rx_sa->next_pn << RECEIVESA_RX_SA_NEXTPN_VAL_SHIFT));
	if (!ctx->secy->replay_protect)
		cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
				  (ctx->sa.rx_sa->next_pn << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
	else if (ctx->sa.rx_sa->next_pn >= ctx->secy->replay_window)
		cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
				  ((ctx->sa.rx_sa->next_pn - ctx->secy->replay_window) << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
	else
		cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
				  (0 << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_KEY_INDEX_WR_BASE_ADDR,
			  key_index << RECEIVESA_RX_SA_KEY_INDEX_WR_VAL_SHIFT);
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_SSCI_WR_BASE_ADDR, ctx->sa.rx_sa->ssci);
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_RECEIVESA_CFG_BASE_ADDR,
			  disable ? 0 : RECEIVESA_RX_SA_RECEIVESA_CFG_ENABLERECEIVE_WR_MASK);
	cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
			  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
			  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
			  RECEIVESA_RX_SA_CTRL_WR_TRIGGER_MASK);

	macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].createdTime = jiffies;
	macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].startedTime =
		macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].createdTime;
	macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].stoppedTime =
		macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].createdTime;
	if (disable)
		macsec_priv->sa_enabled[secy_index][peer_index] &= ~(1 << ctx->sa.assoc_num);
	else {
		if ((macsec_priv->sa_enabled[secy_index][peer_index] & 0x0f) == 0)
			// first Rx-SA to become active, so Rx-SC is now active:
			macsec_priv->rxsc_ext[secy_index][peer_index].startedTime = jiffies;
		macsec_priv->sa_enabled[secy_index][peer_index] |= (1 << ctx->sa.assoc_num);
	}

	return 0;
}

static int cco_macsec_upd_rxsa(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sc *rx_sc = ctx->sa.rx_sa->sc;
	u8 disable = ctx->sa.rx_sa->active ? 0 : 1;
	u8 isEnabled;
	u32 secy_index, peer_index, i;
	int key_index = -1;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	// check if ctx->sa.rx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
	// and whether it is enabled for Rx (check key_use).
	for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_rx && i < CCO_MACSEC_KEYS; ++i) {
		if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_RX))
			continue;
		if (memcmp(ctx->sa.rx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
			// found key.id
			key_index = i;
			break;
		}
	}
	if (key_index < 0)
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	isEnabled = (macsec_priv->sa_enabled[secy_index][peer_index] >> ctx->sa.assoc_num) & 1;

	if (isEnabled) {
		// currently enabled
		// Configure Rx-SA registers (common part):
		cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_AN_BASE_ADDR,
				  (ctx->sa.assoc_num << RECEIVESA_RX_SA_AN_VAL_SHIFT));
		cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_KEY_INDEX_WR_BASE_ADDR,
				  key_index << RECEIVESA_RX_SA_KEY_INDEX_WR_VAL_SHIFT);
		if (disable) {
			// Configure Rx-SA registers (disable):
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_RECEIVESA_CFG_BASE_ADDR, 0);
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
					  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
					  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
					  RECEIVESA_RX_SA_CTRL_WR_TRIGGER_MASK);
			macsec_priv->sa_enabled[secy_index][peer_index] &= ~(1 << ctx->sa.assoc_num);
			macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].stoppedTime = jiffies;
			if ((macsec_priv->sa_enabled[secy_index][peer_index] & 0x0f) == 0)
				// no Rx-SA active now, so Rx-SC is no longer active:
				macsec_priv->rxsc_ext[secy_index][peer_index].stoppedTime = jiffies;
		} else {
			// Configure Rx-SA registers (update PN):
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_UPDTNEXTPN_BASE_ADDR,
					  (ctx->sa.rx_sa->next_pn << RECEIVESA_RX_SA_UPDTNEXTPN_VAL_SHIFT));
			if (ctx->sa.rx_sa->next_pn >= ctx->secy->replay_window)
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_UPDTLOWESTPN_BASE_ADDR,
						  ((ctx->sa.rx_sa->next_pn - ctx->secy->replay_window) << RECEIVESA_RX_SA_UPDTLOWESTPN_VAL_SHIFT));
			else
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_UPDTLOWESTPN_BASE_ADDR,
						  (0 << RECEIVESA_RX_SA_UPDTLOWESTPN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
					  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
					  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
					  RECEIVESA_RX_SA_CTRL_UPDATE_PN_TRIGGER_MASK);
		}
	} else {
		// currently disabled
		if (!disable) {
			// Configure Rx-SA registers (common part):
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_AN_BASE_ADDR,
					  (ctx->sa.assoc_num << RECEIVESA_RX_SA_AN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_KEY_INDEX_WR_BASE_ADDR,
					  key_index << RECEIVESA_RX_SA_KEY_INDEX_WR_VAL_SHIFT);
			// Configure Rx-SA registers (enable):
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_NEXTPN_BASE_ADDR,
					  (ctx->sa.rx_sa->next_pn << RECEIVESA_RX_SA_NEXTPN_VAL_SHIFT));
			if (!ctx->secy->replay_protect)
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
						  (ctx->sa.rx_sa->next_pn << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
			else if (ctx->sa.rx_sa->next_pn >= ctx->secy->replay_window)
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
						  ((ctx->sa.rx_sa->next_pn - ctx->secy->replay_window) << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
			else
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
						  (0 << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_SSCI_WR_BASE_ADDR, ctx->sa.rx_sa->ssci);
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_RECEIVESA_CFG_BASE_ADDR,
					  disable ? 0 : RECEIVESA_RX_SA_RECEIVESA_CFG_ENABLERECEIVE_WR_MASK);
			cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
					  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
					  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
					  RECEIVESA_RX_SA_CTRL_WR_TRIGGER_MASK);
			if ((macsec_priv->sa_enabled[secy_index][peer_index] & 0x0f) == 0)
				// first Rx-SA to become active, so Rx-SC is now active:
				macsec_priv->rxsc_ext[secy_index][peer_index].startedTime = jiffies;
			macsec_priv->sa_enabled[secy_index][peer_index] |= (1 << ctx->sa.assoc_num);
			macsec_priv->rxsa_ext[secy_index][peer_index][ctx->sa.assoc_num].startedTime = jiffies;
		}
	}

	return 0;
}

static int cco_macsec_del_rxsa(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sc *rx_sc = ctx->sa.rx_sa->sc;
	u32 secy_index, peer_index, i;
	int key_index = -1;
	u8 enable_mask;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	// check if ctx->sa.rx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
	// and whether it is enabled for Rx (check key_use).
	for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_rx && i < CCO_MACSEC_KEYS; ++i) {
		if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_RX))
			continue;
		if (memcmp(ctx->sa.rx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
			// found key.id
			key_index = i;
			break;
		}
	}
	if (key_index < 0)
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	enable_mask = macsec_priv->sa_enabled[secy_index][peer_index] & 0x0f;
	clear_rxsa(ctx, macsec_priv, secy_index, peer_index, ctx->sa.assoc_num, key_index);
	if (enable_mask && (macsec_priv->sa_enabled[secy_index][peer_index] & 0x0f) == 0)
		// no Rx-SA active now, so Rx-SC is no longer active:
		macsec_priv->rxsc_ext[secy_index][peer_index].stoppedTime = jiffies;
	return 0;
}

static int cco_macsec_add_txsa(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u8 disable = ctx->sa.tx_sa->active ? 0 : 1, val;
	u8 vlan_in_clear, conf_offs;
	u32 secy_index, i;
	int key_index = -1;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// check if ctx->sa.tx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
	// and whether it is enabled for Tx (check key_use).
	for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_tx && i < CCO_MACSEC_KEYS; ++i) {
		if (!macsec_priv->key_use[i]) {
			if (key_index < 0)
				key_index = i; // first free entry
			continue;
		}
		if (memcmp(ctx->sa.tx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
			// found key.id
			key_index = i;
			break;
		}
	}
	if (key_index < 0)
		// not found and no room for a new
		return -ENOSPC;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	if (!(macsec_priv->key_use[key_index] & CCO_MACSEC_KEY_TX)) {
		u8 key_rx = (macsec_priv->key_use[key_index] & CCO_MACSEC_KEY_RX) ? 1 : 0;
		macsec_priv->key_use[key_index] |= CCO_MACSEC_KEY_TX;
		memcpy(macsec_priv->key_id_table[key_index], ctx->sa.tx_sa->key.id, MACSEC_KEYID_LEN);
		// Configure the Tx-SA key:
		if (ctx->secy->key_len == (128/8)) {
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_0_BASE_ADDR,
					  (ctx->sa.key[12] << 24) | (ctx->sa.key[13] << 16) |
					  (ctx->sa.key[14] << 8) | ctx->sa.key[15]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_1_BASE_ADDR,
					  (ctx->sa.key[8] << 24) | (ctx->sa.key[9] << 16) |
					  (ctx->sa.key[10] << 8) | ctx->sa.key[11]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_2_BASE_ADDR,
					  (ctx->sa.key[4] << 24) | (ctx->sa.key[5] << 16) |
					  (ctx->sa.key[6] << 8) | ctx->sa.key[7]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_3_BASE_ADDR,
					  (ctx->sa.key[0] << 24) | (ctx->sa.key[1] << 16) |
					  (ctx->sa.key[2] << 8) | ctx->sa.key[3]);
			if (!ctx->secy->xpn)
				val = 0; // 0x0: AES-GCM-128
			else
				val = 2; // 0x2: AES-GCM-XPN-128
		} else {
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_0_BASE_ADDR,
					  (ctx->sa.key[28] << 24) | (ctx->sa.key[29] << 16) |
					  (ctx->sa.key[30] << 8) | ctx->sa.key[31]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_1_BASE_ADDR,
					  (ctx->sa.key[24] << 24) | (ctx->sa.key[25] << 16) |
					  (ctx->sa.key[26] << 8) | ctx->sa.key[27]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_2_BASE_ADDR,
					  (ctx->sa.key[20] << 24) | (ctx->sa.key[21] << 16) |
					  (ctx->sa.key[22] << 8) | ctx->sa.key[23]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_3_BASE_ADDR,
					  (ctx->sa.key[16] << 24) | (ctx->sa.key[17] << 16) |
					  (ctx->sa.key[18] << 8) | ctx->sa.key[19]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_4_BASE_ADDR,
					  (ctx->sa.key[12] << 24) | (ctx->sa.key[13] << 16) |
					  (ctx->sa.key[14] << 8) | ctx->sa.key[15]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_5_BASE_ADDR,
					  (ctx->sa.key[8] << 24) | (ctx->sa.key[9] << 16) |
					  (ctx->sa.key[10] << 8) | ctx->sa.key[11]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_6_BASE_ADDR,
					  (ctx->sa.key[4] << 24) | (ctx->sa.key[5] << 16) |
					  (ctx->sa.key[6] << 8) | ctx->sa.key[7]);
			cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SAK_7_BASE_ADDR,
					  (ctx->sa.key[0] << 24) | (ctx->sa.key[1] << 16) |
					  (ctx->sa.key[2] << 8) | ctx->sa.key[3]);
			if (!ctx->secy->xpn)
				val = 1; // 0x1: AES-GCM-256
			else
				val = 3; // 0x3: AES-GCM-XPN-256
		}
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SALT_0_BASE_ADDR,
				  (ctx->sa.tx_sa->key.salt.bytes[8] << 24) | (ctx->sa.tx_sa->key.salt.bytes[9] << 16) |
				  (ctx->sa.tx_sa->key.salt.bytes[10] << 8) | ctx->sa.tx_sa->key.salt.bytes[11]);
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SALT_1_BASE_ADDR,
				  (ctx->sa.tx_sa->key.salt.bytes[4] << 24) | (ctx->sa.tx_sa->key.salt.bytes[5] << 16) |
				  (ctx->sa.tx_sa->key.salt.bytes[6] << 8) | ctx->sa.tx_sa->key.salt.bytes[7]);
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_SALT_2_BASE_ADDR,
				  (ctx->sa.tx_sa->key.salt.bytes[0] << 24) | (ctx->sa.tx_sa->key.salt.bytes[1] << 16) |
				  (ctx->sa.tx_sa->key.salt.bytes[2] << 8) | ctx->sa.tx_sa->key.salt.bytes[3]);
		cco_macsec_reg_wr(ctx->netdev, CIPHERSUITE_BASE_ADDR + CIPHERSUITE_CS_CTRL_BASE_ADDR,
				  (key_index << CIPHERSUITE_CS_CTRL_INDEX_SHIFT) |
				  (key_rx << CIPHERSUITE_CS_CTRL_RECEIVE_SEL_SHIFT) |
				  (val << CIPHERSUITE_CS_CTRL_AES_MODE_SEL_SHIFT)   |
				  CIPHERSUITE_CS_CTRL_TRANSMIT_SEL_MASK             |
				  CIPHERSUITE_CS_CTRL_WR_TRIGGER_MASK);
	}
	macsec_priv->key_refcnt[key_index]++;

	// Configure Tx-SA registers:
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
			  (ctx->sa.assoc_num << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_NEXT_PN_BASE_ADDR,
			  (ctx->sa.tx_sa->next_pn << TRANSMITSA_TX_SA_NEXT_PN_VAL_SHIFT));
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
			  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_SSCI_WR_BASE_ADDR, ctx->sa.tx_sa->ssci);
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR,
			  (ctx->secy->tx_sc.encrypt ? TRANSMITSA_TX_SA_TRANSMITSA_CFG_CONFIDENTIALITY_WR_MASK : 0));
	cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
			  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
			  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);

	macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].createdTime = jiffies;
	macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].startedTime =
		macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].createdTime;
	macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].stoppedTime =
		macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].createdTime;
	if (disable)
		macsec_priv->sa_enabled[secy_index][0] &= ~(0x10 << ctx->sa.assoc_num);
	else {
		macsec_priv->sa_enabled[secy_index][0] |= (0x10 << ctx->sa.assoc_num);
		if (ctx->secy->tx_sc.encoding_sa == ctx->sa.assoc_num) {
			// Tx-SA used by Tx-SC becomes active, so Tx-SC is now active:
			macsec_priv->txsc_ext[secy_index].startedTime = jiffies;
			vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
			conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
			write_secy_txsc(ctx->netdev, ctx->secy, secy_index, 0, macsec_priv->sa_enabled[secy_index][0], vlan_in_clear, conf_offs);
		}
	}

	return 0;
}

static int cco_macsec_upd_txsa(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u8 disable = ctx->sa.tx_sa->active ? 0 : 1;
	u8 isEnabled, vlan_in_clear, conf_offs;
	u32 secy_index, i;
	int key_index = -1;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// check if ctx->sa.tx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
	// and whether it is enabled for Tx (check key_use).
	for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_tx && i < CCO_MACSEC_KEYS; ++i) {
		if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_TX))
			continue;
		if (memcmp(ctx->sa.tx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
			// found key.id
			key_index = i;
			break;
		}
	}
	if (key_index < 0)
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	isEnabled = (macsec_priv->sa_enabled[secy_index][0] >> (ctx->sa.assoc_num + 4)) & 1;

	if (isEnabled) {
		// currently enabled
		if (disable) {
			// Configure Tx-SA registers (disable):
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
					  (ctx->sa.assoc_num << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
					  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR, 0);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
					  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
					  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);
			macsec_priv->sa_enabled[secy_index][0] &= ~(0x10 << ctx->sa.assoc_num);
			macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].stoppedTime = jiffies;
			if (ctx->secy->tx_sc.encoding_sa == ctx->sa.assoc_num) {
				// Tx-SA used by Tx-SC becomes inactive, so Tx-SC is no longer active:
				macsec_priv->txsc_ext[secy_index].stoppedTime = jiffies;
				vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
				conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
				write_secy_txsc(ctx->netdev, ctx->secy, secy_index, 0, 0, vlan_in_clear, conf_offs);
			}
		} else {
			// Configure Tx-SA registers (e.g. update PN):
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
					  (ctx->sa.assoc_num << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
					  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_NEXT_PN_BASE_ADDR,
					  (ctx->sa.tx_sa->next_pn << TRANSMITSA_TX_SA_NEXT_PN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_SSCI_WR_BASE_ADDR, ctx->sa.tx_sa->ssci);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR,
					  disable ? 0 : (ctx->secy->tx_sc.encrypt ? TRANSMITSA_TX_SA_TRANSMITSA_CFG_CONFIDENTIALITY_WR_MASK : 0));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
					  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
					  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);
        }
	} else {
		// currently disabled
		if (!disable) {
			// Configure Tx-SA registers (enable):
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
					  (ctx->sa.assoc_num << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
					  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_NEXT_PN_BASE_ADDR,
					  (ctx->sa.tx_sa->next_pn << TRANSMITSA_TX_SA_NEXT_PN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_SSCI_WR_BASE_ADDR, ctx->sa.tx_sa->ssci);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR,
					  disable ? 0 : (ctx->secy->tx_sc.encrypt ? TRANSMITSA_TX_SA_TRANSMITSA_CFG_CONFIDENTIALITY_WR_MASK : 0));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
					  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
					  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);
			macsec_priv->sa_enabled[secy_index][0] |= (0x10 << ctx->sa.assoc_num);
			macsec_priv->txsa_ext[secy_index][ctx->sa.assoc_num].startedTime = jiffies;
			if (ctx->secy->tx_sc.encoding_sa == ctx->sa.assoc_num) {
				// Tx-SA used by Tx-SC becomes active, so Tx-SC is now active:
				macsec_priv->txsc_ext[secy_index].startedTime = jiffies;
				vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
				conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
				write_secy_txsc(ctx->netdev, ctx->secy, secy_index, 0, macsec_priv->sa_enabled[secy_index][0], vlan_in_clear, conf_offs);
			}
		}
	}

	return 0;
}

static int cco_macsec_del_txsa(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u32 secy_index, i;
	int key_index = -1;
	u8 enable_mask, vlan_in_clear, conf_offs;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// check if ctx->sa.tx_sa->key.id exists in the netdev_priv(ctx->netdev) key_id_table
	// and whether it is enabled for Tx (check key_use).
	for (i = 0; i < macsec_priv->capabilities.no_of_key_entries_tx && i < CCO_MACSEC_KEYS; ++i) {
		if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_TX))
			continue;
		if (memcmp(ctx->sa.tx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
			// found key.id
			key_index = i;
			break;
		}
	}
	if (key_index < 0)
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	enable_mask = macsec_priv->sa_enabled[secy_index][0] & 0xf0;
	clear_txsa(ctx, macsec_priv, secy_index, ctx->sa.assoc_num, key_index);
	if (ctx->secy->tx_sc.encoding_sa == ctx->sa.assoc_num) {
		// Tx-SA used by Tx-SC becomes inactive, so Tx-SC is no longer active:
		macsec_priv->txsc_ext[secy_index].stoppedTime = jiffies;
		vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
		conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
		write_secy_txsc(ctx->netdev, ctx->secy, secy_index, 0, 0, vlan_in_clear, conf_offs);
	}
	return 0;
}

static int cco_macsec_get_dev_stats(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u32 secy_index;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// trigger a read of per SecY stats:
	cco_macsec_reg_wr(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_STATS_CTRL_BASE_ADDR,
			  STATISTICS_STATS_CTRL_PORT_STATS_RD_TRIGGER_MASK       |
			  (secy_index << STATISTICS_STATS_CTRL_SECY_INDEX_SHIFT));

	// read counters:
	macsec_priv->dev_stats[secy_index].OutPktsUntagged  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSUNTAGGED_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].InPktsUntagged   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSUNTAGGED_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].OutPktsTooLong   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSTOOLONG_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].InPktsNoTag      += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOTAG_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].InPktsBadTag     += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSBADTAG_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].InPktsNoSCI      += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSA_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].InPktsUnknownSCI += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSAERROR_BASE_ADDR);
	macsec_priv->dev_stats[secy_index].InPktsOverrun    += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSOVERRUN_BASE_ADDR);
	*ctx->stats.dev_stats = macsec_priv->dev_stats[secy_index];

	// store other port stats:
	macsec_priv->port_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINOCTETS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINUCASTPKTS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINMULTICASTPKTS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINBROADCASTPKTS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINDISCARDS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINERRORS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTOCTETS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTUCASTPKTS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTMULTICASTPKTS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTBROADCASTPKTS_BASE_ADDR);
	macsec_priv->port_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTERRORS_BASE_ADDR);

	// store other uport stats:
	macsec_priv->uport_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINOCTETS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINUCASTPKTS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINMULTICASTPKTS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINBROADCASTPKTS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINDISCARDS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINERRORS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTOCTETS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTUCASTPKTS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTMULTICASTPKTS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTBROADCASTPKTS_BASE_ADDR);
	macsec_priv->uport_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTERRORS_BASE_ADDR);

	// store other ext port stats:
	macsec_priv->ext_port_stats[secy_index].compTxDisable         += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_TX_DISABLE_BASE_ADDR);
	macsec_priv->ext_port_stats[secy_index].compRxDisable         += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_RX_DISABLE_BASE_ADDR);
	macsec_priv->ext_port_stats[secy_index].txSecYDisable         += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_SECYDISABLE_BASE_ADDR);
	macsec_priv->ext_port_stats[secy_index].rxSecYDisable         += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_SECYDISABLE_BASE_ADDR);
	macsec_priv->ext_port_stats[secy_index].txReceivingDisable    += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_RECEIVINGDISABLE_BASE_ADDR);
	macsec_priv->ext_port_stats[secy_index].rxTransmittingDisable += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_TRANSMITTINGDISABLE_BASE_ADDR);

	return 0;
}

static int cco_macsec_get_tx_sc_stats(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	u32 secy_index;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// trigger a read of Tx-SC stats:
	cco_macsec_reg_wr(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_STATS_CTRL_BASE_ADDR,
			  STATISTICS_STATS_CTRL_TX_STATS_RD_TRIGGER_MASK         |
			  (secy_index << STATISTICS_STATS_CTRL_SECY_INDEX_SHIFT));

	// read counters:
	macsec_priv->txsc_stats[secy_index].OutPktsProtected   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSPROTECTED_BASE_ADDR);
	macsec_priv->txsc_stats[secy_index].OutPktsEncrypted   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSENCRYPTED_BASE_ADDR);
	macsec_priv->txsc_stats[secy_index].OutOctetsProtected += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTOCTETSPROTECTED_BASE_ADDR);
	macsec_priv->txsc_stats[secy_index].OutOctetsEncrypted += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTOCTETSENCRYPTED_BASE_ADDR);
	*ctx->stats.tx_sc_stats = macsec_priv->txsc_stats[secy_index];

	return 0;
}

static int cco_macsec_get_tx_sa_stats(struct macsec_context *ctx)
{
	struct macsec_tx_sa *tx_sa;
	u32 secy_index, next_pn;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// no per-SA stats:
	memset(ctx->stats.tx_sa_stats, 0, sizeof(*ctx->stats.tx_sa_stats));

	// but do update tx_sa->next_pn:
	tx_sa = rcu_dereference_bh(ctx->secy->tx_sc.sa[ctx->sa.assoc_num]);
	cco_macsec_reg_wr(ctx->netdev, STATUS_BASE_ADDR + STATUS_ST_CTRL_BASE_ADDR,
			  STATUS_ST_CTRL_RD_TRIGGER_MASK                       |
			  (ctx->sa.assoc_num << STATUS_ST_CTRL_SA_INDEX_SHIFT) |
			  (secy_index << STATUS_ST_CTRL_SECY_INDEX_SHIFT));
	next_pn = cco_macsec_reg_rd(ctx->netdev, STATUS_BASE_ADDR + STATUS_ST_TX_SA_NEXT_PN_BASE_ADDR);
	spin_lock_bh(&tx_sa->lock);
	tx_sa->next_pn = next_pn;
	spin_unlock_bh(&tx_sa->lock);

	return 0;
}

static int cco_macsec_get_rx_sc_stats(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sc *rx_sc = ctx->rx_sc;
	u32 secy_index, peer_index;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// trigger a read of Rx-SC stats:
	cco_macsec_reg_wr(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_STATS_CTRL_BASE_ADDR,
			  STATISTICS_STATS_CTRL_RX_STATS_RD_TRIGGER_MASK         |
			  (peer_index << STATISTICS_STATS_CTRL_PEER_INDEX_SHIFT) |
			  (secy_index << STATISTICS_STATS_CTRL_SECY_INDEX_SHIFT));
	// read counters:
	macsec_priv->rxsc_stats[secy_index][peer_index].InOctetsValidated += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INOCTETSVALIDATED_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InOctetsDecrypted += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INOCTETSDECRYPTED_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InPktsUnchecked   += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSUNCHECKED_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InPktsDelayed     += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSDELAYED_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InPktsOK          += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSOK_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InPktsInvalid     += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSINVALID_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InPktsLate        += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSLATE_BASE_ADDR);
	macsec_priv->rxsc_stats[secy_index][peer_index].InPktsNotValid    += cco_macsec_reg_rd(ctx->netdev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOTVALID_BASE_ADDR);
	// macsec_priv->rxsc_stats[secy_index][peer_index].InPktsNotUsingSA  = 0; // deprecated in IEEE 802.1AE-2018
	// macsec_priv->rxsc_stats[secy_index][peer_index].InPktsUnusedSA    = 0; // deprecated in IEEE 802.1AE-2018
	*ctx->stats.rx_sc_stats = macsec_priv->rxsc_stats[secy_index][peer_index];

	return 0;
}

static int cco_macsec_get_rx_sa_stats(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_rx_sa *rx_sa;
	struct macsec_rx_sc *rx_sc = ctx->rx_sc;
	u32 secy_index, peer_index, next_pn;

	// get the SecY:
	if (!find_secy(ctx, &secy_index))
		return -EINVAL;

	// get the peer_index:
	if (!find_rxsc(macsec_priv, rx_sc, secy_index, &peer_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// no per-SA stats:
	memset(ctx->stats.rx_sa_stats, 0, sizeof(*ctx->stats.rx_sa_stats));

	// but do update rx_sa->next_pn:
	rx_sa = rcu_dereference_bh(rx_sc->sa[ctx->sa.assoc_num]);
	cco_macsec_reg_wr(ctx->netdev, STATUS_BASE_ADDR + STATUS_ST_CTRL_BASE_ADDR,
			  STATUS_ST_CTRL_RD_TRIGGER_MASK                       |
			  (ctx->sa.assoc_num << STATUS_ST_CTRL_SA_INDEX_SHIFT) |
			  (peer_index << STATUS_ST_CTRL_PEER_INDEX_SHIFT)      |
			  (secy_index << STATUS_ST_CTRL_SECY_INDEX_SHIFT));
	next_pn = cco_macsec_reg_rd(ctx->netdev, STATUS_BASE_ADDR + STATUS_ST_RX_SA_NEXTPN_BASE_ADDR);
	spin_lock_bh(&rx_sa->lock);
	rx_sa->next_pn = next_pn;
	spin_unlock_bh(&rx_sa->lock);

	return 0;
}

static int cco_macsec_dev_open(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_secy *secy = ctx->secy;
	struct macsec_rx_sc *rx_sc;
	struct macsec_rx_sa *rx_sa;
	struct macsec_tx_sa *tx_sa;
	u32 secy_index, peer_index, i;
	int key_index;
	u8 vlan_in_clear, conf_offs, disable, sa_ix;

	if (!get_secy(ctx, &secy_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	if (macsec_priv->secy_stopped[secy_index]) {
		// 1. Update the SecY and Tx SC configuration:
		vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
		conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
		write_secy_txsc(ctx->netdev, secy, secy_index, 0, macsec_priv->sa_enabled[secy_index][0], vlan_in_clear, conf_offs);

		// 2. Add/update each of the Rx SCs found in the macsec_priv->rxsc_array[secy_index].
		//    For each Rx SC, add/update any Rx SAs found in the ‘sa’ array for the Rx SC.
		for (peer_index = 0; peer_index < CCO_MACSEC_RXSC_MAX; ++peer_index) {
			rx_sc = macsec_priv->rxsc_array[secy_index][peer_index];
			if (!rx_sc)
				continue;
			disable = rx_sc->active ? 0 : 1;
			write_rxsc(ctx->netdev, rx_sc, secy_index, peer_index, disable);
			for (sa_ix = 0; sa_ix < MACSEC_NUM_AN; ++sa_ix) {
				rx_sa = rtnl_dereference(rx_sc->sa[sa_ix]);
				if (!rx_sa)
					continue;
				// find key_index:
				for (i = 0, key_index = -1; i < macsec_priv->capabilities.no_of_key_entries_rx && i < CCO_MACSEC_KEYS; ++i) {
					if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_RX))
						continue;
					if (memcmp(rx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
						// found key.id
						key_index = i;
						break;
					}
				}
				if (key_index < 0) {
					// this should not happen
					netdev_warn(ctx->netdev, "%s secy_index=%u peer_index=%u Rx-SA #%u key not found, should not happen\n", __func__, secy_index, peer_index, sa_ix);
					continue;
				}
				disable = rx_sa->active ? 0 : 1;
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_AN_BASE_ADDR,
						  (sa_ix << RECEIVESA_RX_SA_AN_VAL_SHIFT));
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_NEXTPN_BASE_ADDR,
						  (rx_sa->next_pn << RECEIVESA_RX_SA_NEXTPN_VAL_SHIFT));
				if (!ctx->secy->replay_protect)
					cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
							  (rx_sa->next_pn << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
				else if (rx_sa->next_pn >= secy->replay_window)
					cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
							  ((rx_sa->next_pn - secy->replay_window) << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
				else
					cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_LOWESTPN_BASE_ADDR,
							  (0 << RECEIVESA_RX_SA_LOWESTPN_VAL_SHIFT));
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_KEY_INDEX_WR_BASE_ADDR,
						  key_index << RECEIVESA_RX_SA_KEY_INDEX_WR_VAL_SHIFT);
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_SSCI_WR_BASE_ADDR, rx_sa->ssci);
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_RECEIVESA_CFG_BASE_ADDR,
						  disable ? 0 : RECEIVESA_RX_SA_RECEIVESA_CFG_ENABLERECEIVE_WR_MASK);
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
						  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
						  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
						  RECEIVESA_RX_SA_CTRL_WR_TRIGGER_MASK);
			}
		}

		// 3. Add/update any Tx SAs found in the ctx->secy->tx_sc Tx SC structure.
		for (sa_ix = 0; sa_ix < MACSEC_NUM_AN; ++sa_ix) {
			tx_sa = rtnl_dereference(secy->tx_sc.sa[sa_ix]);
			if (!tx_sa)
				continue;
			// find key_index:
			for (i = 0, key_index = -1; i < macsec_priv->capabilities.no_of_key_entries_tx && i < CCO_MACSEC_KEYS; ++i) {
				if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_TX))
					continue;
				if (memcmp(tx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
					// found key.id
					key_index = i;
					break;
				}
			}
			if (key_index < 0) {
				// this should not happen
				netdev_warn(ctx->netdev, "%s secy_index=%u peer_index=%u Tx-SA #%u key not found, should not happen\n", __func__, secy_index, peer_index, sa_ix);
				continue;
			}
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
					  (sa_ix << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_NEXT_PN_BASE_ADDR,
					  (tx_sa->next_pn << TRANSMITSA_TX_SA_NEXT_PN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
					  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_SSCI_WR_BASE_ADDR, tx_sa->ssci);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR,
					  (ctx->secy->tx_sc.encrypt ? TRANSMITSA_TX_SA_TRANSMITSA_CFG_CONFIDENTIALITY_WR_MASK : 0));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
					  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
					  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);
		}
		macsec_priv->secy_stopped[secy_index] = 0;
	}
	return 0;
}

static int cco_macsec_dev_stop(struct macsec_context *ctx)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(ctx->netdev);
	struct macsec_secy *secy = ctx->secy;
	struct macsec_rx_sc *rx_sc;
	struct macsec_rx_sa *rx_sa;
	struct macsec_tx_sa *tx_sa;
	u32 secy_index, peer_index, i;
	int key_index;
	u8 sa_ix;

	if (!get_secy(ctx, &secy_index))
		return -EINVAL;

	if (ctx->prepare)
		return 0;

	// netdev_info(ctx->netdev, "%s\n", __func__);

	if (macsec_priv->secy_stopped[secy_index] == 0) {
		// 1. Delete the SecY and Tx SC registers:
		write_secy_txsc(ctx->netdev, secy, secy_index, 1, 0, 0, 0);

		// 2. Delete each of the Rx SCs found in the macsec_priv->rxsc_array[secy_index].
		//    For each Rx SC, delete any Rx SAs found in the ‘sa’ array for the Rx SC.
		for (peer_index = 0; peer_index < CCO_MACSEC_RXSC_MAX; ++peer_index) {
			rx_sc = macsec_priv->rxsc_array[secy_index][peer_index];
			if (!rx_sc)
				continue;
			write_rxsc(ctx->netdev, rx_sc, secy_index, peer_index, 1);
			for (sa_ix = 0; sa_ix < MACSEC_NUM_AN; ++sa_ix) {
				rx_sa = rtnl_dereference(rx_sc->sa[sa_ix]);
				if (!rx_sa)
					continue;
				// find key_index:
				for (i = 0, key_index = -1; i < macsec_priv->capabilities.no_of_key_entries_rx && i < CCO_MACSEC_KEYS; ++i) {
					if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_RX))
						continue;
					if (memcmp(rx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
						// found key.id
						key_index = i;
						break;
					}
				}
				if (key_index < 0) {
					// this should not happen
					netdev_warn(ctx->netdev, "%s Rx-SA key not found, should not happen\n", __func__);
					continue;
				}
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_AN_BASE_ADDR,
						  (sa_ix << RECEIVESA_RX_SA_AN_VAL_SHIFT));
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_KEY_INDEX_WR_BASE_ADDR,
						  key_index << RECEIVESA_RX_SA_KEY_INDEX_WR_VAL_SHIFT);
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_RECEIVESA_CFG_BASE_ADDR, 0);
				cco_macsec_reg_wr(ctx->netdev, RECEIVESA_BASE_ADDR + RECEIVESA_RX_SA_CTRL_BASE_ADDR,
						  (peer_index << RECEIVESA_RX_SA_CTRL_PEER_INDEX_SHIFT) |
						  (secy_index << RECEIVESA_RX_SA_CTRL_SECY_INDEX_SHIFT) |
						  RECEIVESA_RX_SA_CTRL_WR_TRIGGER_MASK);
			}
		}

		// 3. Delete any Tx SAs found in the ctx->secy->tx_sc Tx SC structure.
		for (sa_ix = 0; sa_ix < MACSEC_NUM_AN; ++sa_ix) {
			tx_sa = rtnl_dereference(secy->tx_sc.sa[sa_ix]);
			if (!tx_sa)
				continue;
			// find key_index:
			for (i = 0, key_index = -1; i < macsec_priv->capabilities.no_of_key_entries_tx && i < CCO_MACSEC_KEYS; ++i) {
				if (!(macsec_priv->key_use[i] & CCO_MACSEC_KEY_TX))
					continue;
				if (memcmp(tx_sa->key.id, macsec_priv->key_id_table[i], MACSEC_KEYID_LEN) == 0) {
					// found key.id
					key_index = i;
					break;
				}
			}
			if (key_index < 0) {
				// this should not happen
				netdev_warn(ctx->netdev, "%s secy_index=%u peer_index=%u Tx-SA #%u key not found, should not happen\n", __func__, secy_index, peer_index, sa_ix);
				continue;
			}
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
					  (sa_ix << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR,
					  key_index << TRANSMITSA_TX_SA_KEY_INDEX_WR_VAL_SHIFT);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR, 0);
			cco_macsec_reg_wr(ctx->netdev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
					  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
					  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);
		}
		macsec_priv->secy_stopped[secy_index] = 1;
	}
	return 0;
}

static const struct macsec_ops cco_macsec_ops = {
	/* Device wide */
	.mdo_dev_open = cco_macsec_dev_open,
	.mdo_dev_stop = cco_macsec_dev_stop,
	/* SecY */
	.mdo_add_secy = cco_macsec_add_secy,
	.mdo_upd_secy = cco_macsec_upd_secy,
	.mdo_del_secy = cco_macsec_del_secy,
	/* Security channels */
	.mdo_add_rxsc = cco_macsec_add_rxsc,
	.mdo_upd_rxsc = cco_macsec_upd_rxsc,
	.mdo_del_rxsc = cco_macsec_del_rxsc,
	/* Security associations */
	.mdo_add_rxsa = cco_macsec_add_rxsa,
	.mdo_upd_rxsa = cco_macsec_upd_rxsa,
	.mdo_del_rxsa = cco_macsec_del_rxsa,
	.mdo_add_txsa = cco_macsec_add_txsa,
	.mdo_upd_txsa = cco_macsec_upd_txsa,
	.mdo_del_txsa = cco_macsec_del_txsa,
	/* Statistics */
	.mdo_get_dev_stats   = cco_macsec_get_dev_stats,
	.mdo_get_tx_sc_stats = cco_macsec_get_tx_sc_stats,
	.mdo_get_tx_sa_stats = cco_macsec_get_tx_sa_stats,
	.mdo_get_rx_sc_stats = cco_macsec_get_rx_sc_stats,
	.mdo_get_rx_sa_stats = cco_macsec_get_rx_sa_stats,
};

static int cco_macsec_get_capabilities(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_secy_ext(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_set_secy_ext(struct sk_buff *skb, struct genl_info *info);
static int cco_macsec_get_rx_traffic_rule(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_set_rx_traffic_rule(struct sk_buff *skb, struct genl_info *info);
static int cco_macsec_get_tx_traffic_rule(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_set_tx_traffic_rule(struct sk_buff *skb, struct genl_info *info);
static int cco_macsec_get_port_stats(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_uport_stats(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_ext_port_stats(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_txsc_ext(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_rxsc_ext(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_txsa_ext(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_get_rxsa_ext(struct sk_buff *skb, struct netlink_callback *cb);
static int cco_macsec_clear_stats(struct sk_buff *skb, struct genl_info *info);

static const struct nla_policy cco_macsec_genl_policy[NUM_CCO_MACSEC_ATTR] = {
	[CCO_MACSEC_ATTR_CAPABILITIES] = {
		.type = NLA_BINARY,
		.len = sizeof(struct cco_macsec_capabilities) },
	[CCO_MACSEC_ATTR_SECY_EXT] = {
		.type = NLA_BINARY,
		.len = sizeof(struct cco_macsec_secy_ext) },
	[CCO_MACSEC_ATTR_IFINDEX] = { .type = NLA_U32 },
	[CCO_MACSEC_ATTR_INDEX]   = { .type = NLA_U32 },
	[CCO_MACSEC_ATTR_SCI]     = { .type = NLA_U64 },
	[CCO_MACSEC_ATTR_TRAFFIC_RULE] = {
		.type = NLA_BINARY,
		.len = sizeof(struct cco_macsec_traffic_rule) },
	[CCO_MACSEC_ATTR_PORT_STATS] = {
		.type = NLA_BINARY,
		.len = sizeof(struct cco_macsec_port_stats) },
	[CCO_MACSEC_ATTR_TIME_STATS] = {
		.type = NLA_BINARY,
		.len = sizeof(struct cco_macsec_time_stats) },
};

static const struct genl_small_ops cco_macsec_genl_ops[] = {
	{
		.cmd = CCO_MACSEC_CMD_GET_CAPABILITIES,
		.dumpit = cco_macsec_get_capabilities,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_SECY_EXT,
		.dumpit = cco_macsec_get_secy_ext,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_SET_SECY_EXT,
		.doit = cco_macsec_set_secy_ext,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_RX_TRAFFIC_RULE,
		.dumpit = cco_macsec_get_rx_traffic_rule,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_SET_RX_TRAFFIC_RULE,
		.doit = cco_macsec_set_rx_traffic_rule,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_TX_TRAFFIC_RULE,
		.dumpit = cco_macsec_get_tx_traffic_rule,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_SET_TX_TRAFFIC_RULE,
		.doit = cco_macsec_set_tx_traffic_rule,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_PORT_STATS,
		.dumpit = cco_macsec_get_port_stats,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_UPORT_STATS,
		.dumpit = cco_macsec_get_uport_stats,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_EXT_PORT_STATS,
		.dumpit = cco_macsec_get_ext_port_stats,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_TXSC_EXT,
		.dumpit = cco_macsec_get_txsc_ext,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_RXSC_EXT,
		.dumpit = cco_macsec_get_rxsc_ext,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_TXSA_EXT,
		.dumpit = cco_macsec_get_txsa_ext,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_GET_RXSA_EXT,
		.dumpit = cco_macsec_get_rxsa_ext,
		.flags = GENL_ADMIN_PERM,
	},
	{
		.cmd = CCO_MACSEC_CMD_CLEAR_STATS,
		.doit = cco_macsec_clear_stats,
		.flags = GENL_ADMIN_PERM,
	},
};

static struct genl_family cco_macsec_fam __ro_after_init = {
	.name        = CCO_MACSEC_GENL_NAME,
	.hdrsize     = 0,
	.version     = CCO_MACSEC_GENL_VERSION,
	.maxattr     = CCO_MACSEC_ATTR_MAX,
	.policy      = cco_macsec_genl_policy,
	.netnsok     = true,
	.module      = THIS_MODULE,
	.small_ops   = cco_macsec_genl_ops,
	.n_small_ops = ARRAY_SIZE(cco_macsec_genl_ops),
};

static int cco_macsec_get_capabilities(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	dev_idx = cb->args[0];

	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_CAPABILITIES);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		if (nla_put(skb, CCO_MACSEC_ATTR_CAPABILITIES, sizeof(macsec_priv->capabilities), &macsec_priv->capabilities)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_secy_ext(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	const struct macsec_secy *secy = NULL;
	struct cco_macsec_secy_ext secy_ext = {};
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 secy_index = 0, ifIndex;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;
		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_SECY_EXT);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		secy_ext.vlan_in_clear          = macsec_priv->secy_vlan_in_clear[secy_index];
		secy_ext.confidentiality_offset = macsec_priv->secy_confidentiality_offs[secy_index];
		secy_ext.secy_txsc_pn_thr       = cco_macsec_reg_rd(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_TX_SC_PN_THRESHOLD_BASE_ADDR);

		if (nla_put(skb, CCO_MACSEC_ATTR_SECY_EXT, sizeof(secy_ext), &secy_ext)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_set_secy_ext(struct sk_buff *skb, struct genl_info *info)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct nlattr **attrs = info->attrs;
	struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_secy_ext secy_ext = {};
	const struct macsec_secy *secy = NULL;
	u32 secy_index = 0, ifIndex;

	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_SECY_EXT])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);
	nla_memcpy(&secy_ext, attrs[CCO_MACSEC_ATTR_SECY_EXT], sizeof(secy_ext));

	rtnl_lock();
	for_each_netdev(net, dev) {
		if ((dev->features & NETIF_F_HW_MACSEC) &&
		    dev->macsec_ops == &cco_macsec_ops) {
			macsec_priv = cco_macsec_get_priv(dev);
			for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
				if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
					secy = macsec_priv->secy_array[secy_index];
					break;
				}
			if (secy) {
				macsec_priv->secy_vlan_in_clear[secy_index] = secy_ext.vlan_in_clear;
				macsec_priv->secy_confidentiality_offs[secy_index] = secy_ext.confidentiality_offset;
				// Configure the SecY registers:
				write_secy_txsc(dev, secy, secy_index, 0, macsec_priv->sa_enabled[secy_index][0], secy_ext.vlan_in_clear, secy_ext.confidentiality_offset);
				cco_macsec_reg_wr(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_TX_SC_PN_THRESHOLD_BASE_ADDR, secy_ext.secy_txsc_pn_thr);
				break;
			}
		}
	}
	rtnl_unlock();
	return 0;
}

static int cco_macsec_clear_stats(struct sk_buff *skb, struct genl_info *info)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct nlattr **attrs = info->attrs;
	struct cco_macsec_priv *macsec_priv;
	const struct macsec_secy *secy = NULL;
	u32 secy_index = 0, ifIndex, peer_index;

	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);

	rtnl_lock();
	for_each_netdev(net, dev) {
		if ((dev->features & NETIF_F_HW_MACSEC) &&
		    dev->macsec_ops == &cco_macsec_ops) {
			macsec_priv = cco_macsec_get_priv(dev);
			for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
				if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
					secy = macsec_priv->secy_array[secy_index];
					break;
				}
			if (secy) {
				memset(&macsec_priv->dev_stats[secy_index], 0, sizeof(macsec_priv->dev_stats[secy_index]));
				memset(&macsec_priv->txsc_stats[secy_index], 0, sizeof(macsec_priv->txsc_stats[secy_index]));
				memset(&macsec_priv->port_stats[secy_index], 0, sizeof(macsec_priv->port_stats[secy_index]));
				memset(&macsec_priv->uport_stats[secy_index], 0, sizeof(macsec_priv->uport_stats[secy_index]));
				memset(&macsec_priv->ext_port_stats[secy_index], 0, sizeof(macsec_priv->ext_port_stats[secy_index]));
				for (peer_index = 0; peer_index < CCO_MACSEC_RXSC_MAX; ++peer_index)
					memset(&macsec_priv->rxsc_stats[secy_index][peer_index], 0, sizeof(macsec_priv->rxsc_stats[secy_index][peer_index]));
				break;
			}
		}
	}
	rtnl_unlock();
	return 0;
}

static int cco_macsec_get_rx_traffic_rule(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_traffic_rule rule = {};
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 index, val;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_INDEX])
		return -EINVAL;
	index = nla_get_u32(attrs[CCO_MACSEC_ATTR_INDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_RX_TRAFFIC_RULE);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		macsec_priv = cco_macsec_get_priv(dev);
		if (index >= macsec_priv->capabilities.no_tt_entries_rx) {
			rtnl_unlock();
			return -EINVAL;
		}

		// trigger a read of Traffic Map rule (Rx):
		cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_CTRL_BASE_ADDR,
				  TRAFFIC_MAP_TT_CTRL_RX_CONFIG_EN_MASK    |
				  TRAFFIC_MAP_TT_CTRL_RD_TRIGGER_MASK      |
				  TRAFFIC_MAP_TT_CTRL_FIELD_SELECT_WR_MASK |
				  (index << TRAFFIC_MAP_TT_CTRL_INDEX_SHIFT));
		val = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_0_RD_BASE_ADDR);
		rule.macAddr[0]  = val >> 24;
		rule.macAddr[1]  = val >> 16;
		rule.macAddr[2]  = val >> 8;
		rule.macAddr[3]  = val;
		val = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_1_RD_BASE_ADDR);
		rule.macAddr[4]  = val >> 8;
		rule.macAddr[5]  = val;
		rule.vlanId      = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_VLAN_RD_BASE_ADDR);
		rule.ethType     = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_ETYPE_RD_BASE_ADDR);
		rule.fieldSelect = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_FIELD_SELECT_RD_BASE_ADDR);
		rule.other       = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_OTHER_RD_BASE_ADDR);
		rule.secy_index  = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_SECY_RD_BASE_ADDR);

		if (nla_put(skb, CCO_MACSEC_ATTR_TRAFFIC_RULE, sizeof(rule), &rule)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_set_rx_traffic_rule(struct sk_buff *skb, struct genl_info *info)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct nlattr **attrs = info->attrs;
	struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_traffic_rule rule = {};
	u32 index, val;

	if (!attrs[CCO_MACSEC_ATTR_INDEX])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_TRAFFIC_RULE])
		return -EINVAL;
	index = nla_get_u32(attrs[CCO_MACSEC_ATTR_INDEX]);
	nla_memcpy(&rule, attrs[CCO_MACSEC_ATTR_TRAFFIC_RULE], sizeof(rule));

	rtnl_lock();
	for_each_netdev(net, dev) {
		if ((dev->features & NETIF_F_HW_MACSEC) &&
		    dev->macsec_ops == &cco_macsec_ops) {
			macsec_priv = cco_macsec_get_priv(dev);
			if (index >= macsec_priv->capabilities.no_tt_entries_rx) {
				rtnl_unlock();
				return -EINVAL;
			}
			// trigger a write of Traffic Map rule (Rx):
			val = ((rule.macAddr[0] << 24) | (rule.macAddr[1] << 16) |
			       (rule.macAddr[2] <<  8) | rule.macAddr[3]);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_0_WR_BASE_ADDR, val);
			val = (rule.macAddr[4] <<  8) | rule.macAddr[5];
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_1_WR_BASE_ADDR, val);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_VLAN_WR_BASE_ADDR, rule.vlanId);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_ETYPE_WR_BASE_ADDR, rule.ethType);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_OTHER_WR_BASE_ADDR, rule.other);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_SECY_WR_BASE_ADDR, rule.secy_index);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_CTRL_BASE_ADDR,
					  TRAFFIC_MAP_TT_CTRL_RX_CONFIG_EN_MASK                           |
					  TRAFFIC_MAP_TT_CTRL_WR_TRIGGER_MASK                             |
					  (rule.fieldSelect << TRAFFIC_MAP_TT_CTRL_FIELD_SELECT_WR_SHIFT) |
					  (index << TRAFFIC_MAP_TT_CTRL_INDEX_SHIFT));
			break;
		}
	}
	rtnl_unlock();
	return 0;
}

static int cco_macsec_get_tx_traffic_rule(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_traffic_rule rule = {};
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 index, val;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_INDEX])
		return -EINVAL;
	index = nla_get_u32(attrs[CCO_MACSEC_ATTR_INDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_TX_TRAFFIC_RULE);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		macsec_priv = cco_macsec_get_priv(dev);
		if (index >= macsec_priv->capabilities.no_tt_entries_tx) {
			rtnl_unlock();
			return -EINVAL;
		}

		// trigger a read of Traffic Map rule (Tx):
		cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_CTRL_BASE_ADDR,
				  TRAFFIC_MAP_TT_CTRL_TX_CONFIG_EN_MASK    |
				  TRAFFIC_MAP_TT_CTRL_RD_TRIGGER_MASK      |
				  TRAFFIC_MAP_TT_CTRL_FIELD_SELECT_WR_MASK |
				  (index << TRAFFIC_MAP_TT_CTRL_INDEX_SHIFT));
		val = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_0_RD_BASE_ADDR);
		rule.macAddr[0]  = val >> 24;
		rule.macAddr[1]  = val >> 16;
		rule.macAddr[2]  = val >> 8;
		rule.macAddr[3]  = val;
		val = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_1_RD_BASE_ADDR);
		rule.macAddr[4]  = val >> 8;
		rule.macAddr[5]  = val;
		rule.vlanId      = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_VLAN_RD_BASE_ADDR);
		rule.ethType     = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_ETYPE_RD_BASE_ADDR);
		rule.fieldSelect = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_FIELD_SELECT_RD_BASE_ADDR);
		rule.other       = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_OTHER_RD_BASE_ADDR);
		rule.secy_index  = cco_macsec_reg_rd(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_SECY_RD_BASE_ADDR);

		if (nla_put(skb, CCO_MACSEC_ATTR_TRAFFIC_RULE, sizeof(rule), &rule)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_set_tx_traffic_rule(struct sk_buff *skb, struct genl_info *info)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct nlattr **attrs = info->attrs;
	struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_traffic_rule rule = {};
	u32 index, val;

	if (!attrs[CCO_MACSEC_ATTR_INDEX])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_TRAFFIC_RULE])
		return -EINVAL;
	index = nla_get_u32(attrs[CCO_MACSEC_ATTR_INDEX]);
	nla_memcpy(&rule, attrs[CCO_MACSEC_ATTR_TRAFFIC_RULE], sizeof(rule));

	rtnl_lock();
	for_each_netdev(net, dev) {
		if ((dev->features & NETIF_F_HW_MACSEC) &&
		    dev->macsec_ops == &cco_macsec_ops) {
			macsec_priv = cco_macsec_get_priv(dev);
			if (index >= macsec_priv->capabilities.no_tt_entries_tx) {
				rtnl_unlock();
				return -EINVAL;
			}
			// trigger a write of Traffic Map rule (Tx):
			val = ((rule.macAddr[0] << 24) | (rule.macAddr[1] << 16) |
			       (rule.macAddr[2] <<  8) | rule.macAddr[3]);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_0_WR_BASE_ADDR, val);
			val = (rule.macAddr[4] <<  8) | rule.macAddr[5];
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_1_WR_BASE_ADDR, val);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_VLAN_WR_BASE_ADDR, rule.vlanId);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_ETYPE_WR_BASE_ADDR, rule.ethType);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_OTHER_WR_BASE_ADDR, rule.other);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_SECY_WR_BASE_ADDR, rule.secy_index);
			cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_CTRL_BASE_ADDR,
					  TRAFFIC_MAP_TT_CTRL_TX_CONFIG_EN_MASK                           |
					  TRAFFIC_MAP_TT_CTRL_WR_TRIGGER_MASK                             |
					  (rule.fieldSelect << TRAFFIC_MAP_TT_CTRL_FIELD_SELECT_WR_SHIFT) |
					  (index << TRAFFIC_MAP_TT_CTRL_INDEX_SHIFT));
			break;
		}
	}
	rtnl_unlock();
	return 0;
}

static int cco_macsec_get_port_stats(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_port_stats port_stats = {};
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	const struct macsec_secy *secy = NULL;
	u32 secy_index = 0, ifIndex;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;

		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_PORT_STATS);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		// trigger a read of per SecY port stats:
		cco_macsec_reg_wr(dev, STATISTICS_BASE_ADDR + STATISTICS_STATS_CTRL_BASE_ADDR,
				  STATISTICS_STATS_CTRL_PORT_STATS_RD_TRIGGER_MASK |
				  (secy_index << STATISTICS_STATS_CTRL_SECY_INDEX_SHIFT));

		// read counters:
		macsec_priv->port_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINOCTETS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINUCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINMULTICASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINBROADCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINDISCARDS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINERRORS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTOCTETS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTUCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTMULTICASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTBROADCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTERRORS_BASE_ADDR);
		port_stats = macsec_priv->port_stats[secy_index];

		// store other port stats:
		macsec_priv->dev_stats[secy_index].OutPktsUntagged  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSUNTAGGED_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsUntagged   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSUNTAGGED_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].OutPktsTooLong   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSTOOLONG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsNoTag      += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOTAG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsBadTag     += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSBADTAG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsNoSCI      += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSA_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsUnknownSCI += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSAERROR_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsOverrun    += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSOVERRUN_BASE_ADDR);

		// store other uport stats:
		macsec_priv->uport_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINOCTETS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINUCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINMULTICASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINBROADCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINDISCARDS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINERRORS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTOCTETS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTUCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTMULTICASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTBROADCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTERRORS_BASE_ADDR);

		// store other ext port stats:
		macsec_priv->ext_port_stats[secy_index].compTxDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_TX_DISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].compRxDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_RX_DISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].txSecYDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_SECYDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].rxSecYDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_SECYDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].txReceivingDisable    += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_RECEIVINGDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].rxTransmittingDisable += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_TRANSMITTINGDISABLE_BASE_ADDR);

		if (nla_put(skb, CCO_MACSEC_ATTR_PORT_STATS, sizeof(port_stats), &port_stats)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_uport_stats(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_port_stats port_stats = {};
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	const struct macsec_secy *secy = NULL;
	u32 secy_index = 0, ifIndex;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;

		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_UPORT_STATS);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		// trigger a read of per SecY port stats:
		cco_macsec_reg_wr(dev, STATISTICS_BASE_ADDR + STATISTICS_STATS_CTRL_BASE_ADDR,
				  STATISTICS_STATS_CTRL_PORT_STATS_RD_TRIGGER_MASK |
				  (secy_index << STATISTICS_STATS_CTRL_SECY_INDEX_SHIFT));

		// read counters:
		macsec_priv->uport_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINOCTETS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINUCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINMULTICASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINBROADCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINDISCARDS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINERRORS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTOCTETS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTUCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTMULTICASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTBROADCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTERRORS_BASE_ADDR);
		port_stats = macsec_priv->uport_stats[secy_index];

		// store other port stats:
		macsec_priv->dev_stats[secy_index].OutPktsUntagged  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSUNTAGGED_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsUntagged   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSUNTAGGED_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].OutPktsTooLong   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSTOOLONG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsNoTag      += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOTAG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsBadTag     += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSBADTAG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsNoSCI      += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSA_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsUnknownSCI += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSAERROR_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsOverrun    += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSOVERRUN_BASE_ADDR);

		// store other port stats:
		macsec_priv->port_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINOCTETS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINUCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINMULTICASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINBROADCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINDISCARDS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINERRORS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTOCTETS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTUCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTMULTICASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTBROADCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTERRORS_BASE_ADDR);

		// store other ext port stats:
		macsec_priv->ext_port_stats[secy_index].compTxDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_TX_DISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].compRxDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_RX_DISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].txSecYDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_SECYDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].rxSecYDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_SECYDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].txReceivingDisable    += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_RECEIVINGDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].rxTransmittingDisable += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_TRANSMITTINGDISABLE_BASE_ADDR);

		if (nla_put(skb, CCO_MACSEC_ATTR_PORT_STATS, sizeof(port_stats), &port_stats)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_ext_port_stats(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	struct cco_macsec_priv *macsec_priv;
	struct cco_macsec_ext_port_stats ext_port_stats = {};
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	const struct macsec_secy *secy = NULL;
	u32 secy_index = 0, ifIndex;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;

		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_EXT_PORT_STATS);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		// trigger a read of per SecY port stats:
		cco_macsec_reg_wr(dev, STATISTICS_BASE_ADDR + STATISTICS_STATS_CTRL_BASE_ADDR,
				  STATISTICS_STATS_CTRL_PORT_STATS_RD_TRIGGER_MASK |
				  (secy_index << STATISTICS_STATS_CTRL_SECY_INDEX_SHIFT));

		// read counters:
		macsec_priv->ext_port_stats[secy_index].compTxDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_TX_DISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].compRxDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_COMP_RX_DISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].txSecYDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_SECYDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].rxSecYDisable         += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_SECYDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].txReceivingDisable    += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_RECEIVINGDISABLE_BASE_ADDR);
		macsec_priv->ext_port_stats[secy_index].rxTransmittingDisable += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_TRANSMITTINGDISABLE_BASE_ADDR);
		ext_port_stats = macsec_priv->ext_port_stats[secy_index];

		// store other port stats:
		macsec_priv->dev_stats[secy_index].OutPktsUntagged  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSUNTAGGED_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsUntagged   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSUNTAGGED_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].OutPktsTooLong   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_TX_OUTPKTSTOOLONG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsNoTag      += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOTAG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsBadTag     += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSBADTAG_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsNoSCI      += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSA_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsUnknownSCI += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSNOSAERROR_BASE_ADDR);
		macsec_priv->dev_stats[secy_index].InPktsOverrun    += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_RX_INPKTSOVERRUN_BASE_ADDR);

		// store other port stats:
		macsec_priv->port_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINOCTETS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINUCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINMULTICASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINBROADCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINDISCARDS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFINERRORS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTOCTETS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTUCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTMULTICASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTBROADCASTPKTS_BASE_ADDR);
		macsec_priv->port_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_CP_IFOUTERRORS_BASE_ADDR);

		// store other uport stats:
		macsec_priv->uport_stats[secy_index].ifInOctets   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINOCTETS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInUcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINUCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInMcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINMULTICASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInBcPkts   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINBROADCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInDiscards += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINDISCARDS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifInErrors   += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFINERRORS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutOctets  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTOCTETS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutUcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTUCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutMcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTMULTICASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutBcPkts  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTBROADCASTPKTS_BASE_ADDR);
		macsec_priv->uport_stats[secy_index].ifOutErrors  += cco_macsec_reg_rd(dev, STATISTICS_BASE_ADDR + STATISTICS_PS_UP_IFOUTERRORS_BASE_ADDR);

		if (nla_put(skb, CCO_MACSEC_ATTR_EXT_PORT_STATS, sizeof(ext_port_stats), &ext_port_stats)) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_txsc_ext(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	const struct macsec_secy *secy = NULL;
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 secy_index = 0, ifIndex;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;
		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_TXSC_EXT);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		if (nla_put(skb, CCO_MACSEC_ATTR_TIME_STATS, sizeof(macsec_priv->txsc_ext[0]), &macsec_priv->txsc_ext[secy_index])) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_rxsc_ext(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	const struct macsec_secy *secy = NULL;
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 secy_index = 0, ifIndex, index;
	sci_t sci;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_SCI])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);
	sci = (sci_t)nla_get_u64(attrs[CCO_MACSEC_ATTR_SCI]);

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;

		for (index = 0; index < CCO_MACSEC_RXSC_MAX; ++index)
			if (macsec_priv->rxsc_array[secy_index][index] && macsec_priv->rxsc_array[secy_index][index]->sci == sci)
				// found
				break;
		if (index >= CCO_MACSEC_RXSC_MAX)
			goto next;
		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_RXSC_EXT);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		if (nla_put(skb, CCO_MACSEC_ATTR_TIME_STATS, sizeof(macsec_priv->rxsc_ext[0][0]), &macsec_priv->rxsc_ext[secy_index][index])) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_txsa_ext(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	const struct macsec_secy *secy = NULL;
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 secy_index = 0, ifIndex, sa_ix;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_INDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);
	sa_ix = nla_get_u32(attrs[CCO_MACSEC_ATTR_INDEX]);
	if (sa_ix >= MACSEC_NUM_AN)
		return -EINVAL;

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;
		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_TXSA_EXT);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		if (nla_put(skb, CCO_MACSEC_ATTR_TIME_STATS, sizeof(macsec_priv->txsa_ext[0][0]), &macsec_priv->txsa_ext[secy_index][sa_ix])) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static int cco_macsec_get_rxsa_ext(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	struct net_device *dev;
	const struct cco_macsec_priv *macsec_priv;
	const struct macsec_secy *secy = NULL;
	const struct genl_dumpit_info *genl_info;
	struct nlattr **attrs;
	u32 secy_index = 0, ifIndex, index, sa_ix;
	sci_t sci;
	void *hdr;
	int dev_idx, d;
	u8 found = 0;

	genl_info = genl_dumpit_info(cb);
	if (!genl_info)
		return -EINVAL;
	attrs = genl_info->attrs;
	if (!attrs)
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_IFINDEX])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_SCI])
		return -EINVAL;
	if (!attrs[CCO_MACSEC_ATTR_INDEX])
		return -EINVAL;
	ifIndex = nla_get_u32(attrs[CCO_MACSEC_ATTR_IFINDEX]);
	sci = (sci_t)nla_get_u64(attrs[CCO_MACSEC_ATTR_SCI]);
	sa_ix = nla_get_u32(attrs[CCO_MACSEC_ATTR_INDEX]);
	if (sa_ix >= MACSEC_NUM_AN)
		return -EINVAL;

	dev_idx = cb->args[0];
	d = 0;
	rtnl_lock();

	cb->seq = 1;

	for_each_netdev(net, dev) {
		if (d < dev_idx || found)
			goto next;

		if (!((dev->features & NETIF_F_HW_MACSEC) &&
		      dev->macsec_ops == &cco_macsec_ops))
			goto next;

		macsec_priv = cco_macsec_get_priv(dev);
		for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index)
			if (macsec_priv->secy_array[secy_index] && macsec_priv->secy_ifIndex[secy_index] == ifIndex) {
				secy = macsec_priv->secy_array[secy_index];
				break;
			}
		if (!secy)
			goto next;

		for (index = 0; index < CCO_MACSEC_RXSC_MAX; ++index)
			if (macsec_priv->rxsc_array[secy_index][index] && macsec_priv->rxsc_array[secy_index][index]->sci == sci)
				// found
				break;
		if (index >= CCO_MACSEC_RXSC_MAX)
			goto next;
		hdr = genlmsg_put(skb, NETLINK_CB(cb->skb).portid, cb->nlh->nlmsg_seq,
				  &cco_macsec_fam, NLM_F_MULTI, CCO_MACSEC_CMD_GET_RXSA_EXT);
		if (!hdr)
			goto done;
		genl_dump_check_consistent(cb, hdr);

		if (nla_put(skb, CCO_MACSEC_ATTR_TIME_STATS, sizeof(macsec_priv->rxsa_ext[0][0][0]), &macsec_priv->rxsa_ext[secy_index][index][sa_ix])) {
			genlmsg_cancel(skb, hdr);
			goto done;
		}

		genlmsg_end(skb, hdr);
		found = 1;
next:
		d++;
	}

done:
	rtnl_unlock();
	cb->args[0] = d;
	return skb->len;
}

static void get_macsec_capabilities(struct net_device *netdev, struct cco_macsec_capabilities* p)
{
	u32 val, msb;

	val = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_AES_GCM_128_CS_IDENTIFIER_0_BASE_ADDR);
	msb = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_AES_GCM_128_CS_IDENTIFIER_1_BASE_ADDR);
	p->aes_gcm_128_cs_id = ((u64)msb << 32) | val;

	val = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_AES_GCM_256_CS_IDENTIFIER_0_BASE_ADDR);
	msb = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_AES_GCM_256_CS_IDENTIFIER_1_BASE_ADDR);
	p->aes_gcm_256_cs_id = ((u64)msb << 32) | val;

	val = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_CAPABILITIES_1_BASE_ADDR);
	p->no_of_peers            = (val & MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_PEERS_MASK) >> MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_PEERS_SHIFT;
	p->no_of_key_entries_rx   = (val & MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_CS_ENTRIES_RX_MASK) >> MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_CS_ENTRIES_RX_SHIFT;
	p->no_of_key_entries_tx   = (val & MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_CS_ENTRIES_TX_MASK) >> MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_CS_ENTRIES_TX_SHIFT;
	p->no_of_secys            = (val & MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_SECYS_MASK) >> MACSEC_CORE_IP_CAPABILITIES_1_NO_OF_SECYS_SHIFT;
	netdev_info(netdev, "%s: no_of_peers=%u no_of_key_entries_rx=%u no_of_key_entries_tx=%u no_of_secys=%u\n",
		    __func__, p->no_of_peers, p->no_of_key_entries_rx, p->no_of_key_entries_tx, p->no_of_secys);

	val = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_CAPABILITIES_2_BASE_ADDR);
	p->no_tt_entries_rx       = (val & MACSEC_CORE_IP_CAPABILITIES_2_NO_TT_ENTRIES_RX_MASK) >> MACSEC_CORE_IP_CAPABILITIES_2_NO_TT_ENTRIES_RX_SHIFT;
	p->no_tt_entries_tx       = (val & MACSEC_CORE_IP_CAPABILITIES_2_NO_TT_ENTRIES_TX_MASK) >> MACSEC_CORE_IP_CAPABILITIES_2_NO_TT_ENTRIES_TX_SHIFT;
	p->confidentiality_offs   = (val & MACSEC_CORE_IP_CAPABILITIES_2_CONFIDENTIALITY_OFFSET_MASK) >> MACSEC_CORE_IP_CAPABILITIES_2_CONFIDENTIALITY_OFFSET_SHIFT;
	p->available_ciphersuites = (val & MACSEC_CORE_IP_CAPABILITIES_2_AVAILABLE_CIPHERSUITES_MASK) >> MACSEC_CORE_IP_CAPABILITIES_2_AVAILABLE_CIPHERSUITES_SHIFT;
	p->vlan_in_clear          = (val & MACSEC_CORE_IP_CAPABILITIES_2_VLAN_IN_CLEAR_MASK) >> MACSEC_CORE_IP_CAPABILITIES_2_VLAN_IN_CLEAR_SHIFT;
	netdev_info(netdev, "%s: no_tt_entries_rx=%u no_tt_entries_tx=%u confidentiality_offs=%u available_ciphersuites=%u vlan_in_clear=%u\n",
		    __func__, p->no_tt_entries_rx, p->no_tt_entries_tx, p->confidentiality_offs, p->available_ciphersuites, p->vlan_in_clear);

	val = cco_macsec_reg_rd(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_CS_CAPABILITY_BASE_ADDR);
	p->ICVLength              = (val & MACSEC_CORE_IP_CS_CAPABILITY_ICVLENGTH_MASK) >> MACSEC_CORE_IP_CS_CAPABILITY_ICVLENGTH_SHIFT;
	p->changesDataLength      = (val & MACSEC_CORE_IP_CS_CAPABILITY_CHANGESDATALENGTH_MASK) >> MACSEC_CORE_IP_CS_CAPABILITY_CHANGESDATALENGTH_SHIFT;
	p->offsConfidentiality    = (val & MACSEC_CORE_IP_CS_CAPABILITY_OFFSETCONFIDENTIALITY_MASK) >> MACSEC_CORE_IP_CS_CAPABILITY_OFFSETCONFIDENTIALITY_SHIFT;
	p->integrityProtection    = (val & MACSEC_CORE_IP_CS_CAPABILITY_INTEGRITYPROTECTION_MASK) >> MACSEC_CORE_IP_CS_CAPABILITY_INTEGRITYPROTECTION_SHIFT;
	netdev_info(netdev, "%s: ICVLength=%u changesDataLength=%u offsConfidentiality=%u integrityProtection=%u\n",
		    __func__, p->ICVLength, p->changesDataLength, p->offsConfidentiality, p->integrityProtection);

	// trigger read from SecY#0:
	cco_macsec_reg_wr(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_CONFIG_CTRL_BASE_ADDR,
			  SECY_CONFIG_CONFIG_CTRL_TX_CONFIG_EN_MASK |
			  SECY_CONFIG_CONFIG_CTRL_RX_CONFIG_EN_MASK |
			  SECY_CONFIG_CONFIG_CTRL_RD_TRIGGER_MASK);

	val = cco_macsec_reg_rd(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_TX_CONFIG_BASE_ADDR);
	p->maxTxKeys     = (val & SECY_CONFIG_TX_CONFIG_MAXTRANSMITKEYS_MASK) >> SECY_CONFIG_TX_CONFIG_MAXTRANSMITKEYS_SHIFT;
	p->maxTxChannels = (val & SECY_CONFIG_TX_CONFIG_MAXTRANSMITCHANNELS_MASK) >> SECY_CONFIG_TX_CONFIG_MAXTRANSMITCHANNELS_SHIFT;

	val = cco_macsec_reg_rd(netdev, SECY_CONFIG_BASE_ADDR + SECY_CONFIG_RX_CONFIG_BASE_ADDR);
	p->maxRxKeys     = (val & SECY_CONFIG_RX_CONFIG_MAXRECEIVEKEYS_MASK) >> SECY_CONFIG_RX_CONFIG_MAXRECEIVEKEYS_SHIFT;
	p->maxRxChannels = (val & SECY_CONFIG_RX_CONFIG_MAXRECEIVECHANNELS_MASK) >> SECY_CONFIG_RX_CONFIG_MAXRECEIVECHANNELS_SHIFT;
	netdev_info(netdev, "%s: maxTxKeys=%u maxTxChannels=%u maxRxKeys=%u maxRxChannels=%u\n",
		    __func__, p->maxTxKeys, p->maxTxChannels, p->maxRxKeys, p->maxRxChannels);
}

int cco_macsec_init(struct net_device *dev)
{
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(dev);
	u32 id, ver;
	int err;
	const struct macsec_secy secy = {};

	id  = cco_macsec_reg_rd(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_ID_BASE_ADDR);
	ver = cco_macsec_reg_rd(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_IP_VERSION_BASE_ADDR);
	netdev_info(dev, "%s: Probe MACsec device: id=0x%08x version=0x%08x\n", __func__, id, ver);
	// only major version must match for driver to work:
	if (id != CCO_MACSEC_IP_ID ||
		(ver & MACSEC_CORE_IP_VERSION_MAJOR_MASK) != (CCO_MACSEC_MAJOR_VER << MACSEC_CORE_IP_VERSION_MAJOR_SHIFT)) {
		netdev_warn(dev, "%s MACsec device not supported IP_ID=0x%08x version=0x%08x\n", __func__, id, ver);
		return -1;
	}

	err = genl_register_family(&cco_macsec_fam);
	if (err) {
		netdev_info(dev, "%s genl_register_family() failed, err=%i\n", __func__, err);
		return err;
	}

	memset(macsec_priv, 0, sizeof(*macsec_priv));
	get_macsec_capabilities(dev, &macsec_priv->capabilities);
	cco_macsec_reg_wr(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR, 0);
	cco_macsec_reg_wr(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_TX_SC_PN_THRESHOLD_BASE_ADDR, default_txsc_pn_thr);

	// enable common port:
	write_secy_txsc(dev, &secy, 0, 1, 0, 0, 0);

	// Write default Traffic Map rules to bypass EAPol messages (EthType 0x888E):
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_0_WR_BASE_ADDR, 0);
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_MAC_ADDR_1_WR_BASE_ADDR, 0);
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_VLAN_WR_BASE_ADDR, 0);
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_ETYPE_WR_BASE_ADDR, ETH_P_PAE); // 0x888E
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_OTHER_WR_BASE_ADDR, 0);
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_SECY_WR_BASE_ADDR, TRAFFIC_MAP_TT_SECY_WR_VAL_MASK); // all ones = bypass
	cco_macsec_reg_wr(dev, TRAFFIC_MAP_BASE_ADDR + TRAFFIC_MAP_TT_CTRL_BASE_ADDR,
			  TRAFFIC_MAP_TT_CTRL_TX_CONFIG_EN_MASK            |
			  TRAFFIC_MAP_TT_CTRL_RX_CONFIG_EN_MASK            |
			  TRAFFIC_MAP_TT_CTRL_WR_TRIGGER_MASK              |
			  (4 << TRAFFIC_MAP_TT_CTRL_FIELD_SELECT_WR_SHIFT) | // 4=EtherType
			  (0 << TRAFFIC_MAP_TT_CTRL_INDEX_SHIFT));

	if (debug_sw_macsec)
		return 0;

	dev->features |= NETIF_F_HW_MACSEC;
	dev->macsec_ops = &cco_macsec_ops;

	return 0;
}

void cco_macsec_exit(struct net_device *dev)
{
	cco_macsec_reg_wr(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR, 0);
	genl_unregister_family(&cco_macsec_fam);
}

void cco_macsec_commonport_status_update(struct net_device *netdev, u8 operational, u8 enabled)
{
	u32 secy_cnt = count_secy(netdev);
	cco_macsec_reg_wr(netdev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_GENERAL_CTRL_BASE_ADDR,
			  (secy_cnt ? MACSEC_CORE_GENERAL_CTRL_MACSEC_EN_MASK : 0)                             |
			  ((operational ? 1 : 0) << MACSEC_CORE_GENERAL_CTRL_COMMONPORT_MAC_OPERATIONAL_SHIFT) |
			  ((enabled ? 1 : 0) << MACSEC_CORE_GENERAL_CTRL_COMMONPORT_MAC_ENABLED_SHIFT));
}

// MACsec interrupt callback (PN exhaustion)
irqreturn_t cco_macsec_isr(int irq, void *dev_id)
{
	struct net_device *dev = (struct net_device *)dev_id;
	struct cco_macsec_priv *macsec_priv = cco_macsec_get_priv(dev);
	struct macsec_secy *secy;
	struct macsec_tx_sa *tx_sa;
	u32 regval;
	u8 sa_ix, vlan_in_clear, conf_offs;
	u32 secy_index;


	regval = cco_macsec_reg_rd(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_INTERRUPT_BASE_ADDR);
	// identify which SecY(s)/Tx SA's is near PN exhaustion:
	for (secy_index = 0; secy_index < macsec_priv->capabilities.no_of_secys && secy_index < CCO_MACSEC_SECY_MAX; ++secy_index) {
		if (!(regval & ((1 << (secy_index + MACSEC_CORE_INTERRUPT_ALMOST_PN_EXHAUSTION_SHIFT)) |
				(1 << (secy_index + MACSEC_CORE_INTERRUPT_PN_EXHAUSTION_SHIFT)))))
			continue;
		// secy_index has PN exhaustion, validate:
		secy = macsec_priv->secy_array[secy_index];
		if (unlikely(!secy)) {
			netdev_warn(dev, "PN threshold expired, but secy_index=%u no longer exists", secy_index);
			continue;
		}
		if (macsec_priv->secy_stopped[secy_index] ||
		    !netif_running(secy->netdev)          ||
		    !secy->tx_sc.active) {
			netdev_warn(dev, "PN threshold expired on down TX SC");
			continue;
		}
		// get Tx SA in use:
		sa_ix = secy->tx_sc.encoding_sa;
		tx_sa = rtnl_dereference(secy->tx_sc.sa[sa_ix]);
		if (unlikely(!tx_sa)) {
			netdev_warn(dev, "PN threshold expired on invalid TX SA");
			continue;
		}
		netdev_info(dev, "PN threshold expired for secy_index=%u, Tx sa_ix=%u, transitioning to !oper", secy_index, sa_ix);
		macsec_pn_wrapped((struct macsec_secy *)secy, tx_sa);
		// Configure Tx-SA registers (disable):
		cco_macsec_reg_wr(dev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_AN_BASE_ADDR,
				  (sa_ix << TRANSMITSA_TX_SA_AN_VAL_SHIFT));
		cco_macsec_reg_wr(dev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_KEY_INDEX_WR_BASE_ADDR, 0);
		cco_macsec_reg_wr(dev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_TRANSMITSA_CFG_BASE_ADDR, 0);
		cco_macsec_reg_wr(dev, TRANSMITSA_BASE_ADDR + TRANSMITSA_TX_SA_CTRL_BASE_ADDR,
				  (secy_index << TRANSMITSA_TX_SA_CTRL_SECY_INDEX_SHIFT) |
				  TRANSMITSA_TX_SA_CTRL_WR_TRIGGER_MASK);
		macsec_priv->sa_enabled[secy_index][0] &= ~(0x10 << sa_ix);
		macsec_priv->txsa_ext[secy_index][sa_ix].stoppedTime = jiffies;
		if (secy->tx_sc.encoding_sa == sa_ix) {
			// Tx-SA used by Tx-SC becomes inactive, so Tx-SC is no longer active:
			macsec_priv->txsc_ext[secy_index].stoppedTime = jiffies;
			vlan_in_clear = macsec_priv->secy_vlan_in_clear[secy_index];
			conf_offs = macsec_priv->secy_confidentiality_offs[secy_index];
			write_secy_txsc(dev, secy, secy_index, 0, 0, vlan_in_clear, conf_offs);
		}
	}
	// clear all interrupts handled:
	cco_macsec_reg_wr(dev, MACSEC_CORE_BASE_ADDR + MACSEC_CORE_INTERRUPT_BASE_ADDR, regval);
	return IRQ_HANDLED;
}
