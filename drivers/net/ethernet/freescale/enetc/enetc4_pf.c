// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2023 NXP */
#include <linux/module.h>
#include <linux/of_net.h>
#include <linux/of_platform.h>
#include <linux/clk.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>
#include <linux/fsl/netc_global.h>

#include "enetc_pf.h"

#define ENETC_SI_MAX_RING_NUM	8

static void enetc4_get_port_caps(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_ECAPR0);
	pf->caps.wol = (val & ECAPR0_WO) ? 1 : 0;

	val = enetc_port_rd(hw, ENETC4_ECAPR1);
	pf->caps.num_vsi = (val & ECAPR1_NUM_VSI) >> 24;
	pf->caps.num_msix = ((val & ECAPR1_NUM_MSIX) >> 12) + 1;

	val = enetc_port_rd(hw, ENETC4_ECAPR2);
	pf->caps.num_rx_bdr = (val & ECAPR2_NUM_RX_BDR) >> 16;
	pf->caps.num_tx_bdr = val & ECAPR2_NUM_TX_BDR;

	val = enetc_port_rd(hw, ENETC4_PMCAPR);
	pf->caps.half_duplex = (val & PMCAPR_HD) ? 1 : 0;

	val = enetc_port_rd(hw, ENETC4_PSIMAFCAPR);
	pf->caps.mac_filter_num = val & PSIMAFCAPR_NUM_MAC_AFTE;

	val = enetc_port_rd(hw, ENETC4_PSIVLANFCAPR);
	pf->caps.vlan_filter_num = val & PSIVLANFCAPR_NUM_VLAN_FTE;

	val = enetc_port_rd(hw, ENETC4_IPFTCAPR);
	pf->caps.ipf_words_num = val & IPFTCAPR_NUM_WORDS;
}

static void enetc4_pf_set_tc_msdu(struct enetc_hw *hw, u32 *max_sdu)
{
	int tc;

	for (tc = 0; tc < 8; tc++) {
		u32 val = ENETC4_MAC_MAXFRM_SIZE;

		if (max_sdu[tc])
			val = max_sdu[tc] + VLAN_ETH_HLEN;

		val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);
		enetc_port_wr(hw, ENETC4_PTCTMSDUR(tc), val);
	}
}

static void enetc4_pf_reset_tc_msdu(struct enetc_hw *hw)
{
	u32 val = ENETC4_MAC_MAXFRM_SIZE;
	int tc;

	val = u32_replace_bits(val, SDU_TYPE_MPDU, PTCTMSDUR_SDU_TYPE);

	for (tc = 0; tc < 8; tc++)
		enetc_port_wr(hw, ENETC4_PTCTMSDUR(tc), val);
}

static void enetc4_set_trx_frame_size(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;

	enetc_port_mac_wr(si, ENETC4_PM_MAXFRM(0),
			  ENETC_SET_MAXFRM(ENETC4_MAC_MAXFRM_SIZE));

	enetc4_pf_reset_tc_msdu(&si->hw);
}

/* Allocate the number of MSI-X vectors for per SI. */
static void enetc4_set_si_msix_num(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	int i, num_msix, total_si;
	u32 val;

	total_si = pf->caps.num_vsi + 1;

	num_msix = pf->caps.num_msix / total_si +
		   pf->caps.num_msix % total_si - 1;
	val = num_msix & 0x3f;
	enetc_port_wr(hw, ENETC4_PSICFGR2(0), val);

	num_msix = pf->caps.num_msix / total_si - 1;
	val = num_msix & 0x3f;
	for (i = 0; i < pf->caps.num_vsi; i++)
		enetc_port_wr(hw, ENETC4_PSICFGR2(i + 1), val);
}

static u32 enetc4_psicfgr0_val_construct(bool is_vf, u32 num_tx_bdr, u32 num_rx_bdr)
{
	u32 val;

	val = ENETC_PSICFGR0_SET_TXBDR(num_tx_bdr);
	val |= ENETC_PSICFGR0_SET_RXBDR(num_rx_bdr);
	val |= ENETC_PSICFGR0_SIVC(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);

	if (is_vf)
		val |= ENETC_PSICFGR0_VTE | ENETC_PSICFGR0_SIVIE;

	return val;
}

static void enetc4_devlink_allocate_rings(struct enetc_pf *pf)
{
	struct enetc_devlink_priv *devl_priv = pf->devl_priv;
	u32 num_si =  pf->caps.num_vsi + 1;
	struct enetc_hw *hw = &pf->si->hw;
	u32 num_rings, val;
	int i;

	for (i = 0; i < num_si && i < ENETC_MAX_SI_NUM; i++) {
		num_rings = devl_priv->si_num_rings[i];
		val = enetc4_psicfgr0_val_construct(i > 0, num_rings, num_rings);
		enetc_port_wr(hw, ENETC4_PSICFGR0(i), val);
	}
}

static void enetc4_default_rings_allocation(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;
	u32 num_rx_bdr, num_tx_bdr, val;
	u32 vf_tx_bdr, vf_rx_bdr;
	int i, rx_rem, tx_rem;

	if (pf->caps.num_rx_bdr < ENETC_SI_MAX_RING_NUM + pf->caps.num_vsi)
		num_rx_bdr = pf->caps.num_rx_bdr - pf->caps.num_vsi;
	else
		num_rx_bdr = ENETC_SI_MAX_RING_NUM;

	if (pf->caps.num_tx_bdr < ENETC_SI_MAX_RING_NUM + pf->caps.num_vsi)
		num_tx_bdr = pf->caps.num_tx_bdr - pf->caps.num_vsi;
	else
		num_tx_bdr = ENETC_SI_MAX_RING_NUM;

	val = enetc4_psicfgr0_val_construct(false, num_tx_bdr, num_rx_bdr);
	enetc_port_wr(hw, ENETC4_PSICFGR0(0), val);

	num_rx_bdr = pf->caps.num_rx_bdr - num_rx_bdr;
	rx_rem = num_rx_bdr % pf->caps.num_vsi;
	num_rx_bdr = num_rx_bdr / pf->caps.num_vsi;

	num_tx_bdr = pf->caps.num_tx_bdr - num_tx_bdr;
	tx_rem = num_tx_bdr % pf->caps.num_vsi;
	num_tx_bdr = num_tx_bdr / pf->caps.num_vsi;

	for (i = 0; i < pf->caps.num_vsi; i++) {
		vf_tx_bdr = (i < tx_rem) ? num_tx_bdr + 1 : num_tx_bdr;
		vf_rx_bdr = (i < rx_rem) ? num_rx_bdr + 1 : num_rx_bdr;
		val = enetc4_psicfgr0_val_construct(true, vf_tx_bdr, vf_rx_bdr);
		enetc_port_wr(hw, ENETC4_PSICFGR0(i + 1), val);
	}
}

static void enetc4_allocate_si_rings(struct enetc_pf *pf)
{
	if (!pf->devl_priv->si_num_rings[0]) {
		enetc4_default_rings_allocation(pf);
	} else {
		enetc4_devlink_allocate_rings(pf);
	}
}

static void enetc4_port_si_configure(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;

	enetc4_allocate_si_rings(pf);

	/* Outer VLAN tag will be used for VLAN filtering */
	enetc_port_wr(hw, ENETC4_PSIVLANFMR, PSIVLANFMR_VS);

	/* enforce VLAN promisc mode for all SIs */
	pf->vlan_promisc_simap = ENETC_VLAN_PROMISC_MAP_ALL;
	if (pf->hw_ops->set_si_vlan_promisc)
		pf->hw_ops->set_si_vlan_promisc(hw, pf->vlan_promisc_simap);

	/* Disable SI MAC multicast & unicast promiscuous */
	enetc_port_wr(hw, ENETC4_PSIPMMR, 0);

	enetc4_set_si_msix_num(pf);
}

static void enetc4_set_default_rss_key(struct enetc_hw *hw)
{
	u8 hash_key[ENETC_RSSHASH_KEY_SIZE];

	/* set up hash key */
	get_random_bytes(hash_key, ENETC_RSSHASH_KEY_SIZE);
	enetc_set_rss_key(hw, hash_key);
}

static void enetc4_set_isit_key_construct_rule(struct enetc_hw *hw)
{
	u32 val;

	/* Key construction rule 0: SMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_SMACP | ISIDKCCR0_OVIDP;
	enetc_port_wr(hw, ENETC4_ISIDKC0CR0, val);

	/* Key construction rule 1: DMAC + VID */
	val = ISIDKCCR0_VALID | ISIDKCCR0_DMACP | ISIDKCCR0_OVIDP;
	enetc_port_wr(hw, ENETC4_ISIDKC1CR0, val);

	/* Enable key construction rule 0 and 1 */
	val = enetc_port_rd(hw, ENETC4_PISIDCR);
	val |= PISIDCR_KC0EN | PISIDCR_KC1EN;
	enetc_port_wr(hw, ENETC4_PISIDCR, val);
}

static void enetc4_configure_port(struct enetc_pf *pf)
{
	struct enetc_hw *hw = &pf->si->hw;

	enetc4_port_si_configure(pf);

	enetc4_set_trx_frame_size(pf);

	enetc4_set_default_rss_key(hw);

	enetc4_set_isit_key_construct_rule(hw);

	/* Master enable for all SIs */
	enetc_port_wr(hw, ENETC4_PMR, PMR_SI0_EN | PMR_SI1_EN | PMR_SI2_EN);

	/* Enable port transmit/receive */
	enetc_port_wr(hw, ENETC4_POR, 0);
}

static int enetc4_pf_set_mac_exact_filter(struct enetc_pf *pf, int type)
{
	struct enetc_mac_entry *mac_tbl __free(kfree);
	int mf_max_num = pf->caps.mac_filter_num;
	struct net_device *ndev = pf->si->ndev;
	struct netdev_hw_addr *ha;
	u8 si_mac[ETH_ALEN];
	int mac_cnt = 0;

	mac_tbl = kcalloc(mf_max_num, sizeof(*mac_tbl), GFP_KERNEL);
	if (!mac_tbl)
		return -ENOMEM;

	enetc_get_si_primary_mac(&pf->si->hw, si_mac);

	netif_addr_lock_bh(ndev);
	if (type & ENETC_MAC_FILTER_TYPE_UC) {
		netdev_for_each_uc_addr(ha, ndev) {
			if (!is_valid_ether_addr(ha->addr) ||
			    ether_addr_equal(ha->addr, si_mac))
				continue;

			if (mac_cnt >= mf_max_num)
				goto err_nospace_out;

			ether_addr_copy(mac_tbl[mac_cnt++].addr, ha->addr);
		}
	}

	if (type & ENETC_MAC_FILTER_TYPE_MC) {
		netdev_for_each_mc_addr(ha, ndev) {
			if (!is_multicast_ether_addr(ha->addr))
				continue;

			if (mac_cnt >= mf_max_num)
				goto err_nospace_out;

			ether_addr_copy(mac_tbl[mac_cnt++].addr, ha->addr);
		}
	}
	netif_addr_unlock_bh(ndev);

	return enetc_pf_set_mac_exact_filter(pf, 0, mac_tbl, mac_cnt);

err_nospace_out:
	netif_addr_unlock_bh(ndev);

	return -ENOSPC;
}

static void enetc4_pf_set_mac_hash_filter(struct enetc_pf *pf, int type)
{
	struct net_device *ndev = pf->si->ndev;
	struct enetc_mac_filter *mac_filter;
	struct enetc_hw *hw = &pf->si->hw;
	struct enetc_si *si = pf->si;
	struct netdev_hw_addr *ha;

	netif_addr_lock_bh(ndev);
	if (type & ENETC_MAC_FILTER_TYPE_UC) {
		mac_filter = &si->mac_filter[UC];
		enetc_reset_mac_addr_filter(mac_filter);
		netdev_for_each_uc_addr(ha, ndev)
			enetc_add_mac_addr_ht_filter(mac_filter, ha->addr);

		pf->hw_ops->set_si_mac_hash_filter(hw, 0, UC,
						   *mac_filter->mac_hash_table);
	}

	if (type & ENETC_MAC_FILTER_TYPE_MC) {
		mac_filter = &si->mac_filter[MC];
		enetc_reset_mac_addr_filter(mac_filter);
		netdev_for_each_mc_addr(ha, ndev)
			enetc_add_mac_addr_ht_filter(mac_filter, ha->addr);

		pf->hw_ops->set_si_mac_hash_filter(hw, 0, MC,
						   *mac_filter->mac_hash_table);
	}
	netif_addr_unlock_bh(ndev);
}

static void enetc4_pf_set_mac_filter(struct enetc_pf *pf, int type)
{
	if (!(type & ENETC_MAC_FILTER_TYPE_ALL))
		return;

	if (enetc4_pf_set_mac_exact_filter(pf, type))
		/* Fallback to use MAC hash filter */
		enetc4_pf_set_mac_hash_filter(pf, type);
}

static void enetc4_pf_do_set_rx_mode(struct work_struct *work)
{
	struct enetc_si *si = container_of(work, struct enetc_si,
					   rx_mode_task);
	struct enetc_ndev_priv *priv = netdev_priv(si->ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct net_device *ndev = si->ndev;
	struct enetc_hw *hw = &si->hw;
	bool uc_promisc = false;
	bool mc_promisc = false;
	int type = 0;

	if (!pf->hw_ops->set_si_mac_hash_filter ||
	    !pf->hw_ops->set_si_mac_promisc)
		return;

	if (ndev->flags & IFF_PROMISC) {
		uc_promisc = true;
		mc_promisc = true;
	} else if (ndev->flags & IFF_ALLMULTI) {
		mc_promisc = true;
		type = ENETC_MAC_FILTER_TYPE_UC;
	} else {
		type = ENETC_MAC_FILTER_TYPE_ALL;
	}

	pf->hw_ops->set_si_mac_promisc(hw, 0, UC, uc_promisc);
	pf->hw_ops->set_si_mac_promisc(hw, 0, MC, mc_promisc);

	/* Clear MAC filter */
	enetc_pf_flush_mac_exact_filter(pf, 0, ENETC_MAC_FILTER_TYPE_ALL);
	pf->hw_ops->set_si_mac_hash_filter(hw, 0, UC, 0);
	pf->hw_ops->set_si_mac_hash_filter(hw, 0, MC, 0);

	enetc4_pf_set_mac_filter(pf, type);
}

static void enetc4_pf_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;

	queue_work(si->workqueue, &si->rx_mode_task);
}

static const struct net_device_ops enetc4_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_pf_set_mac_addr,
	.ndo_set_rx_mode	= enetc4_pf_set_rx_mode,
	.ndo_vlan_rx_add_vid	= enetc_vlan_rx_add_vid,
	.ndo_vlan_rx_kill_vid	= enetc_vlan_rx_del_vid,
	.ndo_set_vf_mac		= enetc_pf_set_vf_mac,
	.ndo_set_vf_vlan	= enetc_pf_set_vf_vlan,
	.ndo_set_vf_spoofchk	= enetc_pf_set_vf_spoofchk,
	.ndo_set_vf_trust	= enetc_pf_set_vf_trust,
	.ndo_get_vf_config	= enetc_pf_get_vf_config,
	.ndo_set_features	= enetc_pf_set_features,
	.ndo_eth_ioctl		= enetc_ioctl,
	.ndo_setup_tc		= enetc_pf_setup_tc,
	.ndo_bpf		= enetc_setup_bpf,
	.ndo_xdp_xmit		= enetc_xdp_xmit,
};

static void enetc4_mac_config(struct enetc_pf *pf, unsigned int mode,
			      phy_interface_t phy_mode)
{
	struct enetc_ndev_priv *priv = netdev_priv(pf->si->ndev);
	struct enetc_si *si = pf->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val &= ~(PM_IF_MODE_IFMODE | PM_IF_MODE_ENA);

	switch (phy_mode) {
	case PHY_INTERFACE_MODE_RGMII:
	case PHY_INTERFACE_MODE_RGMII_ID:
	case PHY_INTERFACE_MODE_RGMII_RXID:
	case PHY_INTERFACE_MODE_RGMII_TXID:
		val |= IFMODE_RGMII;
		/* We need to enable auto-negotiation for the MAC
		 * if its RGMII interface support In-Band status.
		 */
		if (phylink_autoneg_inband(mode))
			val |= PM_IF_MODE_ENA;
		break;
	case PHY_INTERFACE_MODE_RMII:
		val |= IFMODE_RMII;
		break;
	case PHY_INTERFACE_MODE_SGMII:
	case PHY_INTERFACE_MODE_2500BASEX:
		val |= IFMODE_SGMII;
		break;
	case PHY_INTERFACE_MODE_10GBASER:
	case PHY_INTERFACE_MODE_XGMII:
	case PHY_INTERFACE_MODE_USXGMII:
		val |= IFMODE_XGMII;
		break;
	default:
		dev_err(priv->dev,
			"Unsupported PHY mode:%d\n", phy_mode);
		return;
	}

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static struct phylink_pcs *
enetc4_pl_mac_select_pcs(struct phylink_config *config, phy_interface_t iface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	return pf->pcs;
}

static void enetc4_pl_mac_config(struct phylink_config *config,
				 unsigned int mode,
				 const struct phylink_link_state *state)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	enetc4_mac_config(pf, mode, state->interface);
}

static void enetc4_set_port_speed(struct enetc_ndev_priv *priv, int speed)
{
	u32 old_speed = priv->speed;
	u32 val;

	if (speed == old_speed)
		return;

	val = enetc_port_rd(&priv->si->hw, ENETC4_PCR);
	val &= ~PCR_PSPEED;

	switch (speed) {
	case SPEED_10:
	case SPEED_100:
	case SPEED_1000:
	case SPEED_2500:
	case SPEED_10000:
		val |= (PCR_PSPEED & PCR_PSPEED_VAL(speed));
		break;
	default:
		val |= (PCR_PSPEED & PCR_PSPEED_VAL(SPEED_10));
	}

	priv->speed = speed;
	enetc_port_wr(&priv->si->hw, ENETC4_PCR, val);
}

static void enetc4_set_rgmii_mac(struct enetc_pf *pf, int speed, int duplex)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_M10 | PM_IF_MODE_REVMII);

	switch (speed) {
	case SPEED_1000:
		val = u32_replace_bits(val, SSP_1G, PM_IF_MODE_SSP);
		break;
	case SPEED_100:
		val = u32_replace_bits(val, SSP_100M, PM_IF_MODE_SSP);
		break;
	case SPEED_10:
		val = u32_replace_bits(val, SSP_10M, PM_IF_MODE_SSP);
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_set_rmii_mac(struct enetc_pf *pf, int speed, int duplex)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_IF_MODE(0));
	val = old_val & ~(PM_IF_MODE_ENA | PM_IF_MODE_SSP);

	switch (speed) {
	case SPEED_100:
		val &= ~PM_IF_MODE_M10;
		break;
	case SPEED_10:
		val |= PM_IF_MODE_M10;
	}

	val = u32_replace_bits(val, duplex == DUPLEX_FULL ? 0 : 1,
			       PM_IF_MODE_HD);

	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_IF_MODE(0), val);
}

static void enetc4_set_hd_flow_control(struct enetc_pf *pf, bool enable)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	if (!pf->caps.half_duplex)
		return;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, enable ? 1 : 0, PM_CMD_CFG_HD_FCEN);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_set_rx_pause(struct enetc_pf *pf, bool rx_pause)
{
	struct enetc_si *si = pf->si;
	u32 old_val, val;

	old_val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val = u32_replace_bits(old_val, rx_pause ? 0 : 1, PM_CMD_CFG_PAUSE_IGN);
	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_set_tx_pause(struct enetc_pf *pf, int num_rxbdr, bool tx_pause)
{
	u32 pause_off_thresh = 0, pause_on_thresh = 0;
	u32 init_quanta = 0, refresh_quanta = 0;
	struct enetc_hw *hw = &pf->si->hw;
	u32 rbmr, old_rbmr;
	int i;

	for (i = 0; i < num_rxbdr; i++) {
		old_rbmr = enetc_rxbdr_rd(hw, i, ENETC_RBMR);
		rbmr = u32_replace_bits(old_rbmr, tx_pause ? 1 : 0, ENETC_RBMR_CM);
		if (rbmr == old_rbmr)
			continue;

		enetc_rxbdr_wr(hw, i, ENETC_RBMR, rbmr);
	}

	if (tx_pause) {
		/* When the port first enters congestion, send a PAUSE request
		 * with the maximum number of quanta. When the port exits
		 * congestion, it will automatically send a PAUSE frame with
		 * zero quanta.
		 */
		init_quanta = 0xffff;

		/* Also, set up the refresh timer to send follow-up PAUSE
		 * frames at half the quanta value, in case the congestion
		 * condition persists.
		 */
		refresh_quanta = 0xffff / 2;

		/* Start emitting PAUSE frames when 3 large frames (or more
		 * smaller frames) have accumulated in the FIFO waiting to be
		 * DMAed to the RX ring.
		 */
		pause_on_thresh = 3 * ENETC4_MAC_MAXFRM_SIZE;
		pause_off_thresh = 1 * ENETC4_MAC_MAXFRM_SIZE;
	}

	enetc_port_mac_wr(pf->si, ENETC4_PM_PAUSE_QUANTA(0), init_quanta);
	enetc_port_mac_wr(pf->si, ENETC4_PM_PAUSE_THRESH(0), refresh_quanta);
	enetc_port_wr(hw, ENETC4_PPAUONTR, pause_on_thresh);
	enetc_port_wr(hw, ENETC4_PPAUOFFTR, pause_off_thresh);
}

static void enetc4_enable_mac(struct enetc_pf *pf, bool en)
{
	struct enetc_si *si = pf->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	val &= ~(PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN);
	val |= en ? (PM_CMD_CFG_TX_EN | PM_CMD_CFG_RX_EN) : 0;

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_pf_send_link_status_msg(struct enetc_pf *pf, bool up)
{
	struct device *dev = &pf->si->pdev->dev;
	union enetc_pf_msg pf_msg;
	u16 ms_mask = 0;
	int i, err;

	for (i = 0; i < pf->num_vfs; i++)
		if (pf->vf_link_status_notify[i])
			ms_mask |= PSIMSGSR_MS(i);

	if (!ms_mask)
		return;

	pf_msg.class_id = ENETC_MSG_CLASS_ID_LINK_STATUS;
	pf_msg.class_code = up ? ENETC_PF_NC_LINK_STATUS_UP :
				 ENETC_PF_NC_LINK_STATUS_DOWN;

	err = enetc_pf_send_msg(pf, pf_msg.code, ms_mask);
	if (err)
		dev_err(dev, "PF notifies link status failed\n");
}

static void enetc4_pl_mac_link_up(struct phylink_config *config,
				  struct phy_device *phy, unsigned int mode,
				  phy_interface_t interface, int speed,
				  int duplex, bool tx_pause, bool rx_pause)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;
	bool hd_fc = false;

	priv = netdev_priv(si->ndev);
	enetc4_set_port_speed(priv, speed);

	if (!phylink_autoneg_inband(mode) &&
	    phy_interface_mode_is_rgmii(interface))
		enetc4_set_rgmii_mac(pf, speed, duplex);

	if (interface == PHY_INTERFACE_MODE_RMII)
		enetc4_set_rmii_mac(pf, speed, duplex);

	if (duplex == DUPLEX_FULL) {
		/* When preemption is enabled, generation of PAUSE frames
		 * must be disabled, as stated in the IEEE 802.3 standard.
		 */
		if (priv->active_offloads & ENETC_F_QBU)
			tx_pause = false;
	} else { /* DUPLEX_HALF */
		if (tx_pause || rx_pause)
			hd_fc = true;

		/* As per 802.3 annex 31B, PAUSE frames are only supported
		 * when the link is configured for full duplex operation.
		 */
		tx_pause = false;
		rx_pause = false;
	}

	enetc4_set_hd_flow_control(pf, hd_fc);
	enetc4_set_tx_pause(pf, priv->num_rx_rings, tx_pause);
	enetc4_set_rx_pause(pf, rx_pause);
	enetc4_enable_mac(pf, true);

	priv->eee.eee_active = phylink_init_eee(priv->phylink, true) >= 0;
	enetc_eee_mode_set(si->ndev, priv->eee.eee_active);

	if (si->hw_features & ENETC_SI_F_QBU)
		enetc_mm_link_state_update(priv, true);

	enetc4_pf_send_link_status_msg(pf, true);
}

static void enetc4_pl_mac_link_down(struct phylink_config *config,
				    unsigned int mode,
				    phy_interface_t interface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;

	priv = netdev_priv(si->ndev);

	priv->eee.eee_active = false;
	enetc_eee_mode_set(si->ndev, priv->eee.eee_active);

	if (si->hw_features & ENETC_SI_F_QBU)
		enetc_mm_link_state_update(priv, false);

	enetc4_pf_send_link_status_msg(pf, false);
	enetc4_enable_mac(pf, false);
}

static const struct phylink_mac_ops enetc_pl_mac_ops = {
	.mac_select_pcs = enetc4_pl_mac_select_pcs,
	.mac_config = enetc4_pl_mac_config,
	.mac_link_up = enetc4_pl_mac_link_up,
	.mac_link_down = enetc4_pl_mac_link_down,
};

static int enetc4_alloc_cls_rules(struct enetc_ndev_priv *priv)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);

	/* Each ingress port filter entry occupies 2 words at least. */
	priv->max_ipf_entries = pf->caps.ipf_words_num / 2;
	priv->cls_rules = kcalloc(priv->max_ipf_entries, sizeof(*priv->cls_rules),
				  GFP_KERNEL);
	if (!priv->cls_rules)
		return -ENOMEM;

	return 0;
}

static void enetc4_free_cls_rules(struct enetc_ndev_priv *priv)
{
	kfree(priv->cls_rules);
}

static void enetc4_pf_set_si_primary_mac(struct enetc_hw *hw, int si, const u8 *addr)
{
	u16 lower = get_unaligned_le16(addr + 4);
	u32 upper = get_unaligned_le32(addr);

	if (si != 0) {
		__raw_writel(upper, hw->port + ENETC4_PSIPMAR0(si));
		__raw_writew(lower, hw->port + ENETC4_PSIPMAR1(si));
	} else {
		__raw_writel(upper, hw->port + ENETC4_PMAR0);
		__raw_writew(lower, hw->port + ENETC4_PMAR1);
	}
}

static void enetc4_pf_get_si_primary_mac(struct enetc_hw *hw, int si, u8 *addr)
{
	u32 upper;
	u16 lower;

	upper = __raw_readl(hw->port + ENETC4_PSIPMAR0(si));
	lower = __raw_readw(hw->port + ENETC4_PSIPMAR1(si));

	put_unaligned_le32(upper, addr);
	put_unaligned_le16(lower, addr + 4);
}

static void enetc4_pf_set_si_based_vlan(struct enetc_hw *hw, int si,
					u16 vlan, u8 qos)
{
	u32 val = 0;

	if (vlan) {
		val = PSIVLANR_E | (vlan & PSIVLANR_VID);
		val = u32_replace_bits(val, qos, PSIVLANR_PCP);
	}

	enetc_port_wr(hw, ENETC4_PSIVLANR(si), val);
}

static void enetc4_pf_get_si_based_vlan(struct enetc_hw *hw, int si,
					u32 *vid, u32 *pcp)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSIVLANR(si));

	*vid = val & PSIVLANR_VID;
	*pcp = (val & PSIVLANR_PCP) >> PSIVLANR_PCP_OFF;
}

static void enetc4_pf_set_si_anti_spoofing(struct enetc_hw *hw, int si, bool en)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSICFGR0(si));

	val = (val & ~PSICFGR0_ANTI_SPOOFING) | (en ? PSICFGR0_ANTI_SPOOFING : 0);
	enetc_port_wr(hw, ENETC4_PSICFGR0(si), val);
}

static void enetc4_pf_set_si_vlan_promisc(struct enetc_hw *hw, char si_map)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSIPVMR);

	val = u32_replace_bits(val, ENETC_PSIPVMR_SET_VP(si_map),
			       ENETC_VLAN_PROMISC_MAP_ALL);
	enetc_port_wr(hw, ENETC4_PSIPVMR, val);
}

static void enetc4_pf_set_si_mac_promisc(struct enetc_hw *hw, int si, int type, bool en)
{
	u32 val = enetc_port_rd(hw, ENETC4_PSIPMMR);

	if (type == UC) {
		if (en)
			val |= ENETC_PSIPMR_SET_UP(si);
		else
			val &= ~ENETC_PSIPMR_SET_UP(si);
	} else { /* Multicast promiscuous mode. */
		if (en)
			val |= ENETC_PSIPMR_SET_MP(si);
		else
			val &= ~ENETC_PSIPMR_SET_MP(si);
	}

	enetc_port_wr(hw, ENETC4_PSIPMMR, val);
}

static void enetc4_pf_set_si_mac_hash_filter(struct enetc_hw *hw, int si,
					     int type, u64 hash)
{
	if (type == UC) {
		enetc_port_wr(hw, ENETC4_PSIUMHFR0(si), lower_32_bits(hash));
		enetc_port_wr(hw, ENETC4_PSIUMHFR1(si), upper_32_bits(hash));
	} else { /* MC */
		enetc_port_wr(hw, ENETC4_PSIMMHFR0(si), lower_32_bits(hash));
		enetc_port_wr(hw, ENETC4_PSIMMHFR1(si), upper_32_bits(hash));
	}
}

static void enetc4_pf_set_si_vlan_hash_filter(struct enetc_hw *hw, int si, u64 hash)
{
	enetc_port_wr(hw, ENETC4_PSIVHFR0(si), lower_32_bits(hash));
	enetc_port_wr(hw, ENETC4_PSIVHFR1(si), upper_32_bits(hash));
}

static void enetc4_pf_set_loopback(struct net_device *ndev, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	u32 val;

	val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));
	/* Enable or disable loopback. */
	val = u32_replace_bits(val, en ? 1 : 0, PM_CMD_CFG_LOOP_EN);
	/* Default to select MAC level loopback mode if loopback is enabled. */
	val = u32_replace_bits(val, en ? LPBCK_MODE_MAC_LEVEL : 0,
			       PM_CMD_CFG_LPBK_MODE);

	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);
}

static void enetc4_pf_set_tc_tsd(struct enetc_hw *hw, int tc, bool en)
{
	enetc_port_wr(hw, ENETC4_PTCTSDR(tc), en ? PTCTSDR_TSDE : 0);
}

static bool enetc4_pf_get_time_gating(struct enetc_hw *hw)
{
	return !!(enetc_port_rd(hw, ENETC4_PTGSCR) & PTGSCR_TGE);
}

static void enetc4_pf_set_time_gating(struct enetc_hw *hw, bool en)
{
	u32 old_val, val;

	old_val = enetc_port_rd(hw, ENETC4_PTGSCR);
	val = u32_replace_bits(old_val, en ? 1 : 0, PTGSCR_TGE);
	if (val != old_val)
		enetc_port_wr(hw, ENETC4_PTGSCR, val);
}

static const struct enetc_pf_hw_ops enetc4_pf_hw_ops = {
	.set_si_primary_mac = enetc4_pf_set_si_primary_mac,
	.get_si_primary_mac = enetc4_pf_get_si_primary_mac,
	.set_si_based_vlan = enetc4_pf_set_si_based_vlan,
	.get_si_based_vlan = enetc4_pf_get_si_based_vlan,
	.set_si_anti_spoofing = enetc4_pf_set_si_anti_spoofing,
	.set_si_vlan_promisc = enetc4_pf_set_si_vlan_promisc,
	.set_si_mac_promisc = enetc4_pf_set_si_mac_promisc,
	.set_si_mac_hash_filter = enetc4_pf_set_si_mac_hash_filter,
	.set_si_vlan_hash_filter = enetc4_pf_set_si_vlan_hash_filter,
	.set_loopback = enetc4_pf_set_loopback,
	.set_tc_tsd = enetc4_pf_set_tc_tsd,
	.set_tc_msdu = enetc4_pf_set_tc_msdu,
	.reset_tc_msdu = enetc4_pf_reset_tc_msdu,
	.get_time_gating = enetc4_pf_get_time_gating,
	.set_time_gating = enetc4_pf_set_time_gating,
};

static void enetc4_get_ntmp_caps(struct enetc_si *si)
{
	struct ntmp_caps *caps = &si->ntmp.caps;
	struct enetc_hw *hw = &si->hw;
	u32 reg;

	/* Get the max number of entris of RP table */
	reg = enetc_port_rd(hw, ENETC4_RPITCAPR);
	caps->rpt_num_entries = reg & RPITCAPR_NUM_ENTRIES;

	/* Get the max number of entris of IS table */
	reg = enetc_port_rd(hw, ENETC4_ISITCAPR);
	caps->ist_num_entries = reg & ISITCAPR_NUM_ENTRIES;

	/* Get the max number of entris of SGI table */
	reg = enetc_port_rd(hw, ENETC4_SGIITCAPR);
	caps->sgit_num_entries = reg & SGITCAPR_NUM_ENTRIES;

	/* Get the max number of entris of ISC table */
	reg = enetc_port_rd(hw, ENETC4_ISCICAPR);
	caps->isct_num_entries = reg & ISCICAPR_NUM_ENTRIES;

	/* Get the max number of words of SGCL table */
	reg = enetc_port_rd(hw, ENETC4_SGCLITCAPR);
	caps->sgclt_num_words = reg & SGCLITCAPR_NUM_WORDS;
}

static u64 enetc4_get_current_time(struct enetc_si *si)
{
	u32 time_l, time_h;
	u64 current_time;

	time_l = enetc_rd_hot(&si->hw, ENETC_SICTR0);
	time_h = enetc_rd_hot(&si->hw, ENETC_SICTR1);
	current_time = (u64)time_h << 32 | time_l;

	return current_time;
}

static u64 enetc4_adjust_base_time(struct ntmp_priv *ntmp, u64 base_time,
				   u32 cycle_time)
{
	struct enetc_si *si = ntmp_to_enetc_si(ntmp);
	u64 current_time, delta, n;

	current_time = enetc4_get_current_time(si);
	if (base_time >= current_time)
		return base_time;

	delta = current_time - base_time;
	n = DIV_ROUND_UP_ULL(delta, cycle_time);
	base_time += (n * (u64)cycle_time);

	return base_time;
}

static u32 enetc4_get_tgst_free_words(struct ntmp_priv *ntmp)
{
	struct enetc_si *si = ntmp_to_enetc_si(ntmp);
	struct enetc_hw *hw = &si->hw;
	u32 words_in_use;
	u32 total_words;

	/* Notice that the admin gate list should be delete first before call
	 * this function, so the ENETC4_PTGAGLLR[ADMIN_GATE_LIST_LENGTH] equal
	 * to zero. That is, the ENETC4_TGSTMOR only contains the words of the
	 * operational gate control list.
	 */
	words_in_use = enetc_port_rd(hw, ENETC4_TGSTMOR) & TGSTMOR_NUM_WORDS;
	total_words = enetc_port_rd(hw, ENETC4_TGSTCAPR) & TGSTCAPR_NUM_WORDS;

	return total_words - words_in_use;
}

static int enetc4_ntmp_bitmap_init(struct ntmp_priv *ntmp)
{
	ntmp->ist_eid_bitmap = bitmap_zalloc(ntmp->caps.ist_num_entries,
					     GFP_KERNEL);
	if (!ntmp->ist_eid_bitmap)
		return -ENOMEM;

	ntmp->sgit_eid_bitmap = bitmap_zalloc(ntmp->caps.sgit_num_entries,
					      GFP_KERNEL);
	if (!ntmp->sgit_eid_bitmap)
		goto free_ist_bitmap;

	ntmp->sgclt_word_bitmap = bitmap_zalloc(ntmp->caps.sgclt_num_words,
						GFP_KERNEL);
	if (!ntmp->sgclt_word_bitmap)
		goto free_sgit_bitmap;

	ntmp->isct_eid_bitmap = bitmap_zalloc(ntmp->caps.isct_num_entries,
					      GFP_KERNEL);
	if (!ntmp->isct_eid_bitmap)
		goto free_sgclt_bitmap;

	ntmp->rpt_eid_bitmap = bitmap_zalloc(ntmp->caps.rpt_num_entries,
					     GFP_KERNEL);
	if (!ntmp->rpt_eid_bitmap)
		goto free_isct_bitmap;

	return 0;

free_isct_bitmap:
	bitmap_free(ntmp->isct_eid_bitmap);
	ntmp->isct_eid_bitmap = NULL;

free_sgclt_bitmap:
	bitmap_free(ntmp->sgclt_word_bitmap);
	ntmp->sgclt_word_bitmap = NULL;

free_sgit_bitmap:
	bitmap_free(ntmp->sgit_eid_bitmap);
	ntmp->sgit_eid_bitmap = NULL;

free_ist_bitmap:
	bitmap_free(ntmp->ist_eid_bitmap);
	ntmp->ist_eid_bitmap = NULL;

	return -ENOMEM;
}

static void enetc4_ntmp_bitmap_free(struct ntmp_priv *ntmp)
{
	bitmap_free(ntmp->rpt_eid_bitmap);
	ntmp->rpt_eid_bitmap = NULL;

	bitmap_free(ntmp->isct_eid_bitmap);
	ntmp->isct_eid_bitmap = NULL;

	bitmap_free(ntmp->sgclt_word_bitmap);
	ntmp->sgclt_word_bitmap = NULL;

	bitmap_free(ntmp->sgit_eid_bitmap);
	ntmp->sgit_eid_bitmap = NULL;

	bitmap_free(ntmp->ist_eid_bitmap);
	ntmp->ist_eid_bitmap = NULL;
}

static int enetc4_init_ntmp_priv(struct enetc_si *si)
{
	struct ntmp_priv *ntmp = &si->ntmp;
	int err;

	ntmp->dev_type = NETC_DEV_ENETC;

	if (si->revision == NETC_REVISION_4_1)
		ntmp->errata = NTMP_ERR052134;

	err = enetc_init_cbdr(si);
	if (err)
		return err;

	enetc4_get_ntmp_caps(si);
	err = enetc4_ntmp_bitmap_init(ntmp);
	if (err)
		goto free_cbdr;

	ntmp->adjust_base_time = enetc4_adjust_base_time;
	ntmp->get_tgst_free_words = enetc4_get_tgst_free_words;

	INIT_HLIST_HEAD(&ntmp->flower_list);
	mutex_init(&ntmp->flower_lock);

	return 0;

free_cbdr:
	enetc_free_cbdr(si);

	return err;
}

static void enetc4_deinit_ntmp_priv(struct enetc_si *si)
{
	enetc4_clear_flower_list(si);
	mutex_destroy(&si->ntmp.flower_lock);
	enetc4_ntmp_bitmap_free(&si->ntmp);
	enetc_free_cbdr(si);
}

static int enetc4_pf_init(struct enetc_pf *pf)
{
	struct device *dev = &pf->si->pdev->dev;
	int err;

	enetc_get_ip_revision(pf->si);

	/* Initialize the MAC address for PF and VFs */
	err = enetc_setup_mac_addresses(dev->of_node, pf);
	if (err) {
		dev_err(dev, "Failed to set MAC addresses\n");
		return err;
	}

	err =  enetc4_init_ntmp_priv(pf->si);
	if (err) {
		dev_err(dev, "Failed to init CBDR\n");
		return err;
	}

	enetc4_configure_port(pf);

	return 0;
}

static void enetc4_pf_deinit(struct enetc_pf *pf)
{
	enetc4_deinit_ntmp_priv(pf->si);
}

static int enetc4_link_init(struct enetc_ndev_priv *priv,
			    struct device_node *node)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	struct device *dev = priv->dev;
	int err;

	err = of_get_phy_mode(node, &pf->if_mode);
	if (err) {
		dev_err(dev, "Failed to get PHY mode\n");
		return err;
	}

	err = enetc_mdiobus_create(pf, node);
	if (err) {
		dev_err(dev, "Failed to create MDIO bus\n");
		return err;
	}

	err = enetc_phylink_create(priv, node, &enetc_pl_mac_ops);
	if (err) {
		dev_err(dev, "Failed to create phylink\n");
		goto err_phylink_create;
	}

	return 0;

err_phylink_create:
	enetc_mdiobus_destroy(pf);

	return err;
}

static void enetc4_link_deinit(struct enetc_ndev_priv *priv)
{
	struct enetc_pf *pf = enetc_si_priv(priv->si);

	enetc_phylink_destroy(priv);
	enetc_mdiobus_destroy(pf);
}

static int enetc4_pf_netdev_create(struct enetc_si *si)
{
	struct device *dev = &si->pdev->dev;
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	int err;

	ndev = alloc_etherdev_mqs(sizeof(struct enetc_ndev_priv),
				  si->num_tx_rings, si->num_rx_rings);
	if (!ndev)
		return  -ENOMEM;

	priv = netdev_priv(ndev);
	mutex_init(&priv->mm_lock);

	if (si->pdev->rcec)
		priv->rcec = si->pdev->rcec;

	priv->ref_clk = devm_clk_get_optional(dev, "enet_ref_clk");
	if (IS_ERR(priv->ref_clk)) {
		dev_err(dev, "Get enet_ref_clk failed\n");
		err = PTR_ERR(priv->ref_clk);
		goto err_clk_get;
	}

	enetc_pf_netdev_setup(si, ndev, &enetc4_ndev_ops);

	enetc_init_si_rings_params(priv);
	err = enetc_configure_si(priv);
	if (err) {
		dev_err(dev, "Failed to configure SI\n");
		goto err_config_si;
	}

	err = enetc4_alloc_cls_rules(priv);
	if (err) {
		dev_err(dev, "Failed to alloc cls rules memory\n");
		goto err_alloc_cls_rules;
	}

	err = enetc_alloc_msix(priv);
	if (err) {
		dev_err(dev, "Failed to alloc MSI-X\n");
		goto err_alloc_msix;
	}

	err = enetc4_link_init(priv, dev->of_node);
	if (err)
		goto err_link_init;

	err = register_netdev(ndev);
	if (err) {
		dev_err(dev, "Failed to register netdev\n");
		goto err_reg_netdev;
	}

	return 0;

err_reg_netdev:
	enetc4_link_deinit(priv);
err_link_init:
	enetc_free_msix(priv);
err_alloc_msix:
	enetc4_free_cls_rules(priv);
err_alloc_cls_rules:
err_config_si:
err_clk_get:
	mutex_destroy(&priv->mm_lock);
	free_netdev(ndev);

	return err;
}

static void enetc4_pf_netdev_destroy(struct enetc_si *si)
{
	struct net_device *ndev = si->ndev;
	struct enetc_ndev_priv *priv;

	priv = netdev_priv(ndev);
	unregister_netdev(ndev);
	enetc4_link_deinit(priv);
	enetc_free_msix(priv);
	enetc4_free_cls_rules(priv);
	mutex_destroy(&priv->mm_lock);
	free_netdev(ndev);
}

static void enetc4_pf_destroy_vlan_list(struct enetc_pf *pf)
{
	struct enetc_vlan_list_entry *entry;
	struct hlist_node *tmp;

	guard(mutex)(&pf->vlan_list_lock);
	hlist_for_each_entry_safe(entry, tmp, &pf->vlan_list, node) {
		hlist_del(&entry->node);
		kfree(entry);
	}

	pf->num_vlan_fe = 0;
}

static void enetc4_pf_destroy_mac_list(struct enetc_pf *pf)
{
	struct enetc_mac_list_entry *entry;
	struct hlist_node *tmp;

	guard(mutex)(&pf->mac_list_lock);
	hlist_for_each_entry_safe(entry, tmp, &pf->mac_list, node) {
		hlist_del(&entry->node);
		kfree(entry);
	}

	pf->num_mac_fe = 0;
}

static int enetc4_pf_unload(struct enetc_pf *pf)
{
	struct enetc_si *si = pf->si;

	drain_workqueue(si->workqueue);
	enetc4_pf_netdev_destroy(si);
	enetc4_pf_deinit(pf);
	enetc4_pf_destroy_vlan_list(pf);
	enetc4_pf_destroy_mac_list(pf);
	pci_disable_device(si->pdev);

	return 0;
}

static int enetc4_pf_load(struct enetc_pf *pf)
{
	struct pci_dev *pdev = pf->si->pdev;
	struct enetc_si *si = pf->si;
	int err;

	pcie_flr(pdev);
	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "Failed to enable ENETC\n");
		return err;
	}

	pci_set_master(pdev);

	err = enetc4_pf_init(pf);
	if (err)
		goto err_pf_init;

	enetc_get_si_caps(si);
	err = enetc4_pf_netdev_create(si);
	if (err)
		goto err_netdev_create;

	return 0;

err_netdev_create:
	enetc4_pf_deinit(pf);
err_pf_init:
	pci_disable_device(pdev);

	return err;
}

static int enetc4_init_devlink(struct enetc_pf *pf)
{
	struct enetc_devlink_priv *devl_priv = pf->devl_priv;
	struct devlink *devlink = priv_to_devlink(devl_priv);
	int err;

	devl_priv->pf_load = enetc4_pf_load;
	devl_priv->pf_unload = enetc4_pf_unload;

	err = enetc_devlink_params_register(devlink);
	if (err)
		return err;

	devlink_register(devlink);

	enetc_devlink_init_params(devlink);

	return 0;
}

static void enetc4_deinit_devlink(struct enetc_pf *pf)
{
	struct devlink *devlink = priv_to_devlink(pf->devl_priv);

	devlink_unregister(devlink);
	enetc_devlink_params_unregister(devlink);
}

static void enetc4_get_psi_hw_features(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC4_PCAPR);
	if (val & PCAPR_TGS)
		si->hw_features |= ENETC_SI_F_QBV;

	val = enetc_port_rd(hw, ENETC4_PMCAPR);
	if (PMCAPR_GET_FP(val) == PMCAPR_FP_SUPP) {
		si->hw_features |= ENETC_SI_F_QBU;
		si->pmac_offset = ENETC4_PMAC_OFFSET;
	}

	val = enetc_port_rd(hw, ENETC4_IPCAPR);
	if (val & IPCAPR_ISID)
		si->hw_features |= ENETC_SI_F_PSFP;
}

static int enetc4_pf_struct_init(struct enetc_si *si)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct device *dev = &si->pdev->dev;
	int err;

	pf->si = si;
	pf->total_vfs = pci_sriov_get_totalvfs(si->pdev);
	if (pf->total_vfs) {
		pf->vf_state = kcalloc(pf->total_vfs, sizeof(struct enetc_vf_state),
				       GFP_KERNEL);
		if (!pf->vf_state)
			return -ENOMEM;
	}

	enetc4_get_psi_hw_features(si);
	enetc4_get_port_caps(pf);
	enetc_pf_register_hw_ops(pf, &enetc4_pf_hw_ops);

	err = enetc_devlink_alloc(pf);
	if (err) {
		dev_err(dev, "Failed to alloc devlink\n");
		goto free_vf_state;
	}

	err = enetc4_init_devlink(pf);
	if (err) {
		dev_err(dev, "Failed to init devlink\n");
		goto free_vf_state;
	}

	INIT_HLIST_HEAD(&pf->mac_list);
	mutex_init(&pf->mac_list_lock);
	INIT_HLIST_HEAD(&pf->vlan_list);
	mutex_init(&pf->vlan_list_lock);

	return 0;

free_vf_state:
	kfree(pf->vf_state);

	return err;
}

static void enetc4_pf_struct_deinit(struct enetc_pf *pf)
{
	enetc4_pf_destroy_vlan_list(pf);
	mutex_destroy(&pf->vlan_list_lock);
	enetc4_pf_destroy_mac_list(pf);
	mutex_destroy(&pf->mac_list_lock);
	enetc4_deinit_devlink(pf);
	kfree(pf->vf_state);
}

static int enetc4_pf_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct device *dev = &pdev->dev;
	struct enetc_si *si;
	struct enetc_pf *pf;
	char wq_name[24];
	int err;

	if (enetc_pf_is_owned_by_mcore(pdev))
		return 0;

	err = netc_check_emdio_state();
	if (err)
		return err;

	pinctrl_pm_select_default_state(dev);

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, sizeof(*pf));
	if (err) {
		dev_err(dev, "PCIe probing failed\n");
		return err;
	}

	/* si is the private data. */
	si = pci_get_drvdata(pdev);
	if (!si->hw.port || !si->hw.global) {
		err = -ENODEV;
		dev_err(dev, "Couldn't map PF only space!\n");
		goto err_enetc_pci_probe;
	}

	err = enetc4_pf_struct_init(si);
	if (err)
		goto err_pf_struct_init;

	pf = enetc_si_priv(si);
	INIT_WORK(&si->rx_mode_task, enetc4_pf_do_set_rx_mode);
	snprintf(wq_name, sizeof(wq_name), "enetc-%s", pci_name(pdev));
	si->workqueue = create_singlethread_workqueue(wq_name);
	if (!si->workqueue) {
		err = -ENOMEM;
		goto err_create_wq;
	}

	err = enetc4_pf_init(pf);
	if (err)
		goto err_pf_init;

	enetc_get_si_caps(si);
	err = enetc4_pf_netdev_create(si);
	if (err)
		goto err_netdev_create;

	enetc_create_debugfs(si);

	err = netc_emdio_consumer_register(dev);
	if (err) {
		dev_err(dev, "Failed to add EMDIO consumer\n");
		goto err_add_emdio_consumer;
	}

	return 0;

err_add_emdio_consumer:
	enetc_remove_debugfs(si);
err_netdev_create:
	enetc4_pf_deinit(pf);
err_pf_init:
	destroy_workqueue(si->workqueue);
err_create_wq:
	enetc4_pf_struct_deinit(pf);
err_pf_struct_init:
err_enetc_pci_probe:
	enetc_pci_remove(pdev);

	return err;
}

static void enetc4_pf_remove(struct pci_dev *pdev)
{
	struct enetc_si *si;
	struct enetc_pf *pf;

	if (enetc_pf_is_owned_by_mcore(pdev)) {
		pci_sriov_configure_simple(pdev, 0);
		return;
	}

	si = pci_get_drvdata(pdev);
	enetc_remove_debugfs(si);

	pf = enetc_si_priv(si);
	if (pf->num_vfs)
		enetc_sriov_configure(pdev, 0);

	enetc4_pf_netdev_destroy(si);
	enetc4_pf_deinit(pf);
	destroy_workqueue(si->workqueue);
	enetc4_pf_struct_deinit(pf);
	enetc_pci_remove(pdev);
}

/* Only ENETC PF Function can be probed. */
static const struct pci_device_id enetc4_pf_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_NXP2, PCI_DEVICE_ID_NXP2_ENETC_PF) },
	{ PCI_DEVICE(PCI_VENDOR_ID_NXP2, ENETC_PF_VIRTUAL_DEVID) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc4_pf_id_table);

#ifdef CONFIG_PCI_IOV
static int enetc4_sriov_suspend_resume_configure(struct pci_dev *pdev, bool suspend)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);
	int err;

	if (pf->num_vfs == 0)
		return 0;

	if (suspend) {
		pci_disable_sriov(pdev);
		enetc_msg_psi_free(pf);
	} else {
		err = enetc_msg_psi_init(pf);
		if (err) {
			dev_err(&pdev->dev, "enetc_msg_psi_init (%d)\n", err);
			return err;
		}

		err = pci_enable_sriov(pdev, pf->num_vfs);
		if (err) {
			dev_err(&pdev->dev, "pci_enable_sriov err %d\n", err);
			goto err_en_sriov;
		}
	}

	return 0;

err_en_sriov:
	enetc_msg_psi_free(pf);

	return err;
}
#else
static int enetc4_sriov_suspend_resume_configure(struct pci_dev *pdev, bool suspend)
{
	return 0;
}
#endif

static int enetc4_pf_imdio_regulator_enable(struct enetc_pf *pf)
{
	struct enetc_mdio_priv *mdio_priv;
	int err = 0;

	if (!pf->imdio)
		return -EINVAL;
	mdio_priv = pf->imdio->priv;

	if (mdio_priv && mdio_priv->regulator)
		err = regulator_enable(mdio_priv->regulator);

	return err;
}

static void enetc4_pf_imdio_regulator_disable(struct enetc_pf *pf)
{
	struct enetc_mdio_priv *mdio_priv;

	if (!pf->imdio)
		return;
	mdio_priv = pf->imdio->priv;

	if (mdio_priv && mdio_priv->regulator)
		regulator_disable(mdio_priv->regulator);
}

static void enetc4_pf_power_down(struct enetc_si *si)
{
	struct enetc_ndev_priv *priv = netdev_priv(si->ndev);
	struct enetc_pf *pf = enetc_si_priv(si);
	struct pci_dev *pdev = si->pdev;

	if (pf->pcs)
		enetc4_pf_imdio_regulator_disable(pf);
	enetc_free_msix(priv);
	pci_disable_device(pdev);
	pcie_flr(pdev);
}

static int enetc4_pf_power_up(struct pci_dev *pdev, struct device_node *node)
{
	struct enetc_ndev_priv *priv;
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err;

	si = pci_get_drvdata(pdev);
	pf = enetc_si_priv(si);
	priv = netdev_priv(si->ndev);

	err = pci_enable_device_mem(pdev);
	if (err) {
		dev_err(&pdev->dev, "device enable failed\n");
		return err;
	}

	pci_set_master(pdev);
	pci_restore_state(pdev);

	err = enetc_init_cbdr(si);
	if (err)
		goto err_init_cbdr;

	err = enetc_setup_mac_addresses(node, pf);
	if (err)
		goto err_init_address;

	enetc_load_primary_mac_addr(&si->hw, priv->ndev);

	enetc4_configure_port(pf);

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

	if (pf->pcs) {
		err = enetc4_pf_imdio_regulator_enable(pf);
		if (err) {
			dev_err(&pdev->dev, "imdio regulator enable failed\n");
			goto err_imdio_reg_enable;
		}
	}

	return 0;

err_imdio_reg_enable:
	enetc_free_msix(priv);
err_alloc_msix:
err_config_si:
err_init_address:
	enetc_free_cbdr(si);
err_init_cbdr:
	pci_disable_device(pdev);

	return err;
}

static void enetc4_pf_set_wol(struct enetc_si *si, bool en)
{
	u32 val = enetc_port_mac_rd(si, ENETC4_PM_CMD_CFG(0));

	if (en)
		val |= PM_CMD_CFG_MG;
	else
		val &= ~PM_CMD_CFG_MG;
	enetc_port_mac_wr(si, ENETC4_PM_CMD_CFG(0), val);

	enetc_port_mac_wr(si, ENETC4_PLPMR, en ? PLPMR_WME : 0);
}

static int __maybe_unused enetc4_pf_suspend(struct device *dev)
{
	struct pci_dev *pdev = to_pci_dev(dev);
	struct enetc_ndev_priv *priv;
	struct enetc_si *si;
	struct enetc_pf *pf;

	if (enetc_pf_is_owned_by_mcore(pdev))
		return 0;

	si = pci_get_drvdata(pdev);
	pf = enetc_si_priv(si);
	priv = netdev_priv(si->ndev);

	if (!netif_running(si->ndev)) {
		rtnl_lock();
		enetc4_pf_power_down(si);
		rtnl_unlock();
		return 0;
	}

	if (netc_ierb_may_wakeonlan() == 0)
		enetc4_sriov_suspend_resume_configure(pdev, true);

	netif_device_detach(si->ndev);

	rtnl_lock();
	enetc_suspend(si->ndev, netc_ierb_may_wakeonlan() > 0);

	if (netc_ierb_may_wakeonlan() > 0) {
		pci_pme_active(pdev, true);

		enetc4_pf_set_wol(si, true);

		pci_save_state(pdev);
		pci_disable_device(pdev);
		pci_set_power_state(pdev, PCI_D3hot);
		phylink_suspend(priv->phylink, true);
	} else {
		phylink_suspend(priv->phylink, false);
		enetc4_pf_power_down(si);
	}
	rtnl_unlock();

	return 0;
}

static int __maybe_unused enetc4_pf_resume(struct device *dev)
{
	struct device_node *node = dev->of_node;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct enetc_ndev_priv *priv;
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err;

	if (enetc_pf_is_owned_by_mcore(pdev))
		return 0;

	si = pci_get_drvdata(pdev);
	pf = enetc_si_priv(si);
	priv = netdev_priv(si->ndev);
	if (!netif_running(si->ndev)) {
		rtnl_lock();
		err = enetc4_pf_power_up(pdev, node);
		goto err_unlock_rtnl;
	}

	rtnl_lock();

	if (netc_ierb_may_wakeonlan() > 0) {
		pci_set_power_state(pdev, PCI_D0);
		err = pci_enable_device(pdev);
		if (err)
			goto err_unlock_rtnl;
		pci_restore_state(pdev);
		enetc4_pf_set_wol(si, false);
	} else {
		err = enetc4_pf_power_up(pdev, node);
		if (err)
			goto err_unlock_rtnl;
	}

	phylink_resume(priv->phylink);
	enetc_resume(si->ndev, netc_ierb_may_wakeonlan() > 0);

	rtnl_unlock();

	netif_device_attach(si->ndev);

	if (netc_ierb_may_wakeonlan() == 0)
		enetc4_sriov_suspend_resume_configure(pdev, false);

	return 0;

err_unlock_rtnl:
	rtnl_unlock();
	return err;
}

static SIMPLE_DEV_PM_OPS(enetc4_pf_pm_ops, enetc4_pf_suspend, enetc4_pf_resume);

static struct pci_driver enetc4_pf_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc4_pf_id_table,
	.probe = enetc4_pf_probe,
	.remove = enetc4_pf_remove,
	.driver.pm = &enetc4_pf_pm_ops,
#ifdef CONFIG_PCI_IOV
	.sriov_configure = enetc_sriov_configure,
#endif
};
module_pci_driver(enetc4_pf_driver);

MODULE_DESCRIPTION("ENETC4 PF Driver");
MODULE_LICENSE("Dual BSD/GPL");
