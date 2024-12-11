// SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause)
/* Copyright 2017-2019 NXP */

#include <linux/unaligned.h>
#include <linux/mdio.h>
#include <linux/module.h>
#include <linux/fsl/enetc_mdio.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/pcs-lynx.h>
#include "enetc_ierb.h"
#include "enetc_pf.h"

#define ENETC_DRV_NAME_STR "ENETC PF driver"

static void enetc_pf_get_primary_mac_addr(struct enetc_hw *hw, int si, u8 *addr)
{
	u32 upper = __raw_readl(hw->port + ENETC_PSIPMAR0(si));
	u16 lower = __raw_readw(hw->port + ENETC_PSIPMAR1(si));

	put_unaligned_le32(upper, addr);
	put_unaligned_le16(lower, addr + 4);
}

static void enetc_pf_set_primary_mac_addr(struct enetc_hw *hw, int si,
					  const u8 *addr)
{
	u32 upper = get_unaligned_le32(addr);
	u16 lower = get_unaligned_le16(addr + 4);

	__raw_writel(upper, hw->port + ENETC_PSIPMAR0(si));
	__raw_writew(lower, hw->port + ENETC_PSIPMAR1(si));
}

static void enetc_set_vlan_promisc(struct enetc_hw *hw, char si_map)
{
	u32 val = enetc_port_rd(hw, ENETC_PSIPVMR);

	val = u32_replace_bits(val, ENETC_PSIPVMR_SET_VP(si_map),
			       ENETC_VLAN_PROMISC_MAP_ALL);
	enetc_port_wr(hw, ENETC_PSIPVMR, val);
}

static void enetc_set_isol_vlan(struct enetc_hw *hw, int si, u16 vlan, u8 qos)
{
	u32 val = 0;

	if (vlan)
		val = ENETC_PSIVLAN_EN | ENETC_PSIVLAN_SET_QOS(qos) | vlan;

	enetc_port_wr(hw, ENETC_PSIVLANR(si), val);
}

static void enetc_add_mac_addr_em_filter(struct enetc_mac_filter *filter,
					 const unsigned char *addr)
{
	/* add exact match addr */
	ether_addr_copy(filter->mac_addr, addr);
	filter->mac_addr_cnt++;
}

static void enetc_pf_set_si_mac_promisc(struct enetc_hw *hw, int si, int type, bool en)
{
	u32 val = enetc_port_rd(hw, ENETC_PSIPMR);

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

	enetc_port_wr(hw, ENETC_PSIPMR, val);
}

static void enetc_set_mac_ht_flt(struct enetc_hw *hw, int si_idx, int type,
				 u64 hash)
{
	struct enetc_si *si = container_of(hw, struct enetc_si, hw);
	bool err = si->errata & ENETC_ERR_UCMCSWP;

	if (type == UC) {
		enetc_port_wr(hw, ENETC_PSIUMHFR0(si_idx, err),
			      lower_32_bits(hash));
		enetc_port_wr(hw, ENETC_PSIUMHFR1(si_idx),
			      upper_32_bits(hash));
	} else { /* MC */
		enetc_port_wr(hw, ENETC_PSIMMHFR0(si_idx, err),
			      lower_32_bits(hash));
		enetc_port_wr(hw, ENETC_PSIMMHFR1(si_idx),
			      upper_32_bits(hash));
	}
}

static void enetc_sync_mac_filters(struct enetc_si *si)
{
	struct enetc_mac_filter *f = si->mac_filter;
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	int i, pos;

	if (!pf->hw_ops->set_si_mac_hash_filter)
		return;

	pos = EMETC_MAC_ADDR_FILT_RES;

	for (i = 0; i < MADDR_TYPE; i++, f++) {
		bool em = (f->mac_addr_cnt == 1) && (i == UC);
		bool clear = !f->mac_addr_cnt;

		if (clear) {
			if (i == UC)
				enetc_clear_mac_flt_entry(si, pos);

			/* Clean MAC hash filter. */
			pf->hw_ops->set_si_mac_hash_filter(hw, 0, i, 0);
			continue;
		}

		/* exact match filter */
		if (em) {
			int err;

			/* Clean MAC hash filter. */
			pf->hw_ops->set_si_mac_hash_filter(hw, 0, i, 0);

			err = enetc_set_mac_flt_entry(si, pos, f->mac_addr,
						      BIT(0));
			if (!err)
				continue;

			/* fallback to HT filtering */
			dev_warn(&si->pdev->dev, "fallback to HT filt (%d)\n",
				 err);
		}

		/* hash table filter, clear EM filter for UC entries */
		if (i == UC)
			enetc_clear_mac_flt_entry(si, pos);

		pf->hw_ops->set_si_mac_hash_filter(hw, 0, i, *f->mac_hash_table);
	}
}

static void enetc_pf_set_rx_mode(struct net_device *ndev)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_pf *pf = enetc_si_priv(priv->si);
	bool uprom = false, mprom = false;
	struct enetc_mac_filter *filter;
	struct enetc_si *si = priv->si;
	struct enetc_hw *hw = &si->hw;
	struct netdev_hw_addr *ha;
	bool em;

	if (ndev->flags & IFF_PROMISC) {
		uprom = true;
		mprom = true;
	} else if (ndev->flags & IFF_ALLMULTI) {
		mprom = true;
	}

	/* first 2 filter entries belong to PF */
	if (!uprom) {
		/* Update unicast filters */
		filter = &si->mac_filter[UC];
		enetc_reset_mac_addr_filter(filter);

		em = (netdev_uc_count(ndev) == 1);
		netdev_for_each_uc_addr(ha, ndev) {
			if (em) {
				enetc_add_mac_addr_em_filter(filter, ha->addr);
				break;
			}

			enetc_add_mac_addr_ht_filter(filter, ha->addr);
		}
	}

	if (!mprom) {
		/* Update multicast filters */
		filter = &si->mac_filter[MC];
		enetc_reset_mac_addr_filter(filter);

		netdev_for_each_mc_addr(ha, ndev) {
			if (!is_multicast_ether_addr(ha->addr))
				continue;

			enetc_add_mac_addr_ht_filter(filter, ha->addr);
		}
	}

	if (!uprom || !mprom)
		/* update PF entries */
		enetc_sync_mac_filters(si);

	if (pf->hw_ops->set_si_mac_promisc) {
		pf->hw_ops->set_si_mac_promisc(hw, 0, UC, uprom);
		pf->hw_ops->set_si_mac_promisc(hw, 0, MC, mprom);
	}
}

static void enetc_set_vlan_ht_filter(struct enetc_hw *hw, int si_idx,
				     u64 hash)
{
	enetc_port_wr(hw, ENETC_PSIVHFR0(si_idx), lower_32_bits(hash));
	enetc_port_wr(hw, ENETC_PSIVHFR1(si_idx), upper_32_bits(hash));
}

static void enetc_set_loopback(struct net_device *ndev, bool en)
{
	struct enetc_ndev_priv *priv = netdev_priv(ndev);
	struct enetc_si *si = priv->si;
	u32 reg;

	reg = enetc_port_mac_rd(si, ENETC_PM0_IF_MODE);
	if (reg & ENETC_PM0_IFM_RG) {
		/* RGMII mode */
		reg = (reg & ~ENETC_PM0_IFM_RLP) |
		      (en ? ENETC_PM0_IFM_RLP : 0);
		enetc_port_mac_wr(si, ENETC_PM0_IF_MODE, reg);
	} else {
		/* assume SGMII mode */
		reg = enetc_port_mac_rd(si, ENETC_PM0_CMD_CFG);
		reg = (reg & ~ENETC_PM0_CMD_XGLP) |
		      (en ? ENETC_PM0_CMD_XGLP : 0);
		reg = (reg & ~ENETC_PM0_CMD_PHY_TX_EN) |
		      (en ? ENETC_PM0_CMD_PHY_TX_EN : 0);
		enetc_port_mac_wr(si, ENETC_PM0_CMD_CFG, reg);
	}
}

static void enetc_pf_set_si_anti_spoofing(struct enetc_hw *hw, int si, bool en)
{
	u32 val = enetc_port_rd(hw, ENETC_PSICFGR0(si));

	val = (val & ~ENETC_PSICFGR0_ASE) | (en ? ENETC_PSICFGR0_ASE : 0);
	enetc_port_wr(hw, ENETC_PSICFGR0(si), val);
}

static void enetc_pf_set_tc_tsd(struct enetc_hw *hw, int tc, bool en)
{
	enetc_port_wr(hw, ENETC_PTCTSDR(tc), en ? ENETC_TSDE : 0);
}

static bool enetc_pf_get_time_gating(struct enetc_hw *hw)
{
	return !!(enetc_rd(hw, ENETC_PTGCR) & ENETC_PTGCR_TGE);
}

static void enetc_pf_set_time_gating(struct enetc_hw *hw, bool en)
{
	u32 old_val, val;

	old_val = enetc_rd(hw, ENETC_PTGCR);
	val = u32_replace_bits(old_val, en ? 1 : 0, ENETC_PTGCR_TGE);
	if (val != old_val)
		enetc_wr(hw, ENETC_PTGCR, val);
}

static void enetc_port_assign_rfs_entries(struct enetc_si *si)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	int num_entries, vf_entries, i;
	u32 val;

	/* split RFS entries between functions */
	val = enetc_port_rd(hw, ENETC_PRFSCAPR);
	num_entries = ENETC_PRFSCAPR_GET_NUM_RFS(val);
	vf_entries = num_entries / (pf->total_vfs + 1);

	for (i = 0; i < pf->total_vfs; i++)
		enetc_port_wr(hw, ENETC_PSIRFSCFGR(i + 1), vf_entries);
	enetc_port_wr(hw, ENETC_PSIRFSCFGR(0),
		      num_entries - vf_entries * pf->total_vfs);

	/* enable RFS on port */
	enetc_port_wr(hw, ENETC_PRFSMR, ENETC_PRFSMR_RFSE);
}

static void enetc_port_get_caps(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	u32 val;

	val = enetc_port_rd(hw, ENETC_PCAPR0);

	if (val & ENETC_PCAPR0_QBV)
		si->hw_features |= ENETC_SI_F_QBV;

	if (val & ENETC_PCAPR0_QBU) {
		si->hw_features |= ENETC_SI_F_QBU;
		si->pmac_offset = ENETC_PMAC_OFFSET;
	}

	if (val & ENETC_PCAPR0_PSFP)
		si->hw_features |= ENETC_SI_F_PSFP;
}

static void enetc_port_si_configure(struct enetc_si *si)
{
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_hw *hw = &si->hw;
	int num_rings, i;
	u32 val;

	enetc_port_get_caps(si);

	val = enetc_port_rd(hw, ENETC_PCAPR0);
	num_rings = min(ENETC_PCAPR0_RXBDR(val), ENETC_PCAPR0_TXBDR(val));

	val = ENETC_PSICFGR0_SET_TXBDR(ENETC_PF_NUM_RINGS);
	val |= ENETC_PSICFGR0_SET_RXBDR(ENETC_PF_NUM_RINGS);

	if (unlikely(num_rings < ENETC_PF_NUM_RINGS)) {
		val = ENETC_PSICFGR0_SET_TXBDR(num_rings);
		val |= ENETC_PSICFGR0_SET_RXBDR(num_rings);

		dev_warn(&si->pdev->dev, "Found %d rings, expected %d!\n",
			 num_rings, ENETC_PF_NUM_RINGS);

		num_rings = 0;
	}

	/* Add default one-time settings for SI0 (PF) */
	val |= ENETC_PSICFGR0_SIVC(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);

	enetc_port_wr(hw, ENETC_PSICFGR0(0), val);

	if (num_rings)
		num_rings -= ENETC_PF_NUM_RINGS;

	/* Configure the SIs for each available VF */
	val = ENETC_PSICFGR0_SIVC(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);
	val |= ENETC_PSICFGR0_VTE | ENETC_PSICFGR0_SIVIE;

	if (num_rings) {
		num_rings /= pf->total_vfs;
		val |= ENETC_PSICFGR0_SET_TXBDR(num_rings);
		val |= ENETC_PSICFGR0_SET_RXBDR(num_rings);
	}

	for (i = 0; i < pf->total_vfs; i++)
		enetc_port_wr(hw, ENETC_PSICFGR0(i + 1), val);

	/* Port level VLAN settings */
	val = ENETC_PVCLCTR_OVTPIDL(ENETC_VLAN_TYPE_C | ENETC_VLAN_TYPE_S);
	enetc_port_wr(hw, ENETC_PVCLCTR, val);
	/* use outer tag for VLAN filtering */
	enetc_port_wr(hw, ENETC_PSIVLANFMR, ENETC_PSIVLANFMR_VS);
}

static void enetc_set_ptcmsdur(struct enetc_hw *hw, u32 *max_sdu)
{
	int tc;

	for (tc = 0; tc < 8; tc++) {
		u32 val = ENETC_MAC_MAXFRM_SIZE;

		if (max_sdu[tc])
			val = max_sdu[tc] + VLAN_ETH_HLEN;

		enetc_port_wr(hw, ENETC_PTCMSDUR(tc), val);
	}
}

static void enetc_reset_ptcmsdur(struct enetc_hw *hw)
{
	int tc;

	for (tc = 0; tc < 8; tc++)
		enetc_port_wr(hw, ENETC_PTCMSDUR(tc), ENETC_MAC_MAXFRM_SIZE);
}

static const struct enetc_pf_hw_ops enetc_pf_hw_ops = {
	.set_si_primary_mac = enetc_pf_set_primary_mac_addr,
	.get_si_primary_mac = enetc_pf_get_primary_mac_addr,
	.set_si_based_vlan = enetc_set_isol_vlan,
	.set_si_vlan_promisc = enetc_set_vlan_promisc,
	.set_si_mac_promisc = enetc_pf_set_si_mac_promisc,
	.set_si_mac_hash_filter = enetc_set_mac_ht_flt,
	.set_si_vlan_hash_filter = enetc_set_vlan_ht_filter,
	.set_loopback = enetc_set_loopback,
	.set_si_anti_spoofing = enetc_pf_set_si_anti_spoofing,
	.set_tc_tsd = enetc_pf_set_tc_tsd,
	.set_tc_msdu = enetc_set_ptcmsdur,
	.reset_tc_msdu = enetc_reset_ptcmsdur,
	.get_time_gating = enetc_pf_get_time_gating,
	.set_time_gating = enetc_pf_set_time_gating,
};

static void enetc_configure_port_mac(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;

	enetc_port_mac_wr(si, ENETC_PM0_MAXFRM,
			  ENETC_SET_MAXFRM(ENETC_RX_MAXFRM_SIZE));

	enetc_reset_ptcmsdur(hw);

	enetc_port_mac_wr(si, ENETC_PM0_CMD_CFG, ENETC_PM0_CMD_PHY_TX_EN |
			  ENETC_PM0_CMD_TXP | ENETC_PM0_PROMISC);

	/* On LS1028A, the MAC RX FIFO defaults to 2, which is too high
	 * and may lead to RX lock-up under traffic. Set it to 1 instead,
	 * as recommended by the hardware team.
	 */
	enetc_port_mac_wr(si, ENETC_PM0_RX_FIFO, ENETC_PM0_RX_FIFO_VAL);
}

static void enetc_mac_config(struct enetc_si *si, phy_interface_t phy_mode)
{
	u32 val;

	if (phy_interface_mode_is_rgmii(phy_mode)) {
		val = enetc_port_mac_rd(si, ENETC_PM0_IF_MODE);
		val &= ~(ENETC_PM0_IFM_EN_AUTO | ENETC_PM0_IFM_IFMODE_MASK);
		val |= ENETC_PM0_IFM_IFMODE_GMII | ENETC_PM0_IFM_RG;
		enetc_port_mac_wr(si, ENETC_PM0_IF_MODE, val);
	}

	if (phy_mode == PHY_INTERFACE_MODE_USXGMII) {
		val = ENETC_PM0_IFM_FULL_DPX | ENETC_PM0_IFM_IFMODE_XGMII;
		enetc_port_mac_wr(si, ENETC_PM0_IF_MODE, val);
	}
}

static void enetc_mac_enable(struct enetc_si *si, bool en)
{
	u32 val = enetc_port_mac_rd(si, ENETC_PM0_CMD_CFG);

	val &= ~(ENETC_PM0_TX_EN | ENETC_PM0_RX_EN);
	val |= en ? (ENETC_PM0_TX_EN | ENETC_PM0_RX_EN) : 0;

	enetc_port_mac_wr(si, ENETC_PM0_CMD_CFG, val);
}

static void enetc_configure_port(struct enetc_pf *pf)
{
	u8 hash_key[ENETC_RSSHASH_KEY_SIZE];
	struct enetc_hw *hw = &pf->si->hw;

	enetc_configure_port_mac(pf->si);

	enetc_port_si_configure(pf->si);

	/* set up hash key */
	get_random_bytes(hash_key, ENETC_RSSHASH_KEY_SIZE);
	enetc_set_rss_key(hw, hash_key);

	/* split up RFS entries */
	enetc_port_assign_rfs_entries(pf->si);

	/* enforce VLAN promisc mode for all SIs */
	pf->vlan_promisc_simap = ENETC_VLAN_PROMISC_MAP_ALL;
	if (pf->hw_ops->set_si_vlan_promisc)
		pf->hw_ops->set_si_vlan_promisc(hw, pf->vlan_promisc_simap);

	enetc_port_wr(hw, ENETC_PSIPMR, 0);

	/* enable port */
	enetc_port_wr(hw, ENETC_PMR, ENETC_PMR_EN);
}

static const struct net_device_ops enetc_ndev_ops = {
	.ndo_open		= enetc_open,
	.ndo_stop		= enetc_close,
	.ndo_start_xmit		= enetc_xmit,
	.ndo_get_stats		= enetc_get_stats,
	.ndo_set_mac_address	= enetc_pf_set_mac_addr,
	.ndo_set_rx_mode	= enetc_pf_set_rx_mode,
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

static struct phylink_pcs *
enetc_pl_mac_select_pcs(struct phylink_config *config, phy_interface_t iface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	return pf->pcs;
}

static void enetc_pl_mac_config(struct phylink_config *config,
				unsigned int mode,
				const struct phylink_link_state *state)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);

	enetc_mac_config(pf->si, state->interface);
}

static void enetc_force_rgmii_mac(struct enetc_si *si, int speed, int duplex)
{
	u32 old_val, val;

	old_val = val = enetc_port_mac_rd(si, ENETC_PM0_IF_MODE);

	if (speed == SPEED_1000) {
		val &= ~ENETC_PM0_IFM_SSP_MASK;
		val |= ENETC_PM0_IFM_SSP_1000;
	} else if (speed == SPEED_100) {
		val &= ~ENETC_PM0_IFM_SSP_MASK;
		val |= ENETC_PM0_IFM_SSP_100;
	} else if (speed == SPEED_10) {
		val &= ~ENETC_PM0_IFM_SSP_MASK;
		val |= ENETC_PM0_IFM_SSP_10;
	}

	if (duplex == DUPLEX_FULL)
		val |= ENETC_PM0_IFM_FULL_DPX;
	else
		val &= ~ENETC_PM0_IFM_FULL_DPX;

	if (val == old_val)
		return;

	enetc_port_mac_wr(si, ENETC_PM0_IF_MODE, val);
}

static void enetc_sched_speed_set(struct enetc_ndev_priv *priv, int speed)
{
	struct enetc_hw *hw = &priv->si->hw;
	u32 old_speed = priv->speed;
	u32 pspeed, tmp;

	if (speed == old_speed)
		return;

	switch (speed) {
	case SPEED_1000:
		pspeed = ENETC_PMR_PSPEED_1000M;
		break;
	case SPEED_2500:
		pspeed = ENETC_PMR_PSPEED_2500M;
		break;
	case SPEED_100:
		pspeed = ENETC_PMR_PSPEED_100M;
		break;
	case SPEED_10:
	default:
		pspeed = ENETC_PMR_PSPEED_10M;
	}

	priv->speed = speed;
	tmp = enetc_port_rd(hw, ENETC_PMR);
	enetc_port_wr(hw, ENETC_PMR, (tmp & ~ENETC_PMR_PSPEED_MASK) | pspeed);
}

static void enetc_pl_mac_link_up(struct phylink_config *config,
				 struct phy_device *phy, unsigned int mode,
				 phy_interface_t interface, int speed,
				 int duplex, bool tx_pause, bool rx_pause)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	u32 pause_off_thresh = 0, pause_on_thresh = 0;
	u32 init_quanta = 0, refresh_quanta = 0;
	struct enetc_hw *hw = &pf->si->hw;
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;
	u32 rbmr, cmd_cfg;
	int idx;

	priv = netdev_priv(si->ndev);

	if (si->hw_features & ENETC_SI_F_QBV)
		enetc_sched_speed_set(priv, speed);

	if (!phylink_autoneg_inband(mode) &&
	    phy_interface_mode_is_rgmii(interface))
		enetc_force_rgmii_mac(si, speed, duplex);

	/* Flow control */
	for (idx = 0; idx < priv->num_rx_rings; idx++) {
		rbmr = enetc_rxbdr_rd(hw, idx, ENETC_RBMR);

		if (tx_pause)
			rbmr |= ENETC_RBMR_CM;
		else
			rbmr &= ~ENETC_RBMR_CM;

		enetc_rxbdr_wr(hw, idx, ENETC_RBMR, rbmr);
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
		pause_on_thresh = 3 * ENETC_MAC_MAXFRM_SIZE;
		pause_off_thresh = 1 * ENETC_MAC_MAXFRM_SIZE;
	}

	enetc_port_mac_wr(si, ENETC_PM0_PAUSE_QUANTA, init_quanta);
	enetc_port_mac_wr(si, ENETC_PM0_PAUSE_THRESH, refresh_quanta);
	enetc_port_wr(hw, ENETC_PPAUONTR, pause_on_thresh);
	enetc_port_wr(hw, ENETC_PPAUOFFTR, pause_off_thresh);

	cmd_cfg = enetc_port_mac_rd(si, ENETC_PM0_CMD_CFG);

	if (rx_pause)
		cmd_cfg &= ~ENETC_PM0_PAUSE_IGN;
	else
		cmd_cfg |= ENETC_PM0_PAUSE_IGN;

	enetc_port_mac_wr(si, ENETC_PM0_CMD_CFG, cmd_cfg);

	enetc_mac_enable(si, true);

	if (si->hw_features & ENETC_SI_F_QBU)
		enetc_mm_link_state_update(priv, true);
}

static void enetc_pl_mac_link_down(struct phylink_config *config,
				   unsigned int mode,
				   phy_interface_t interface)
{
	struct enetc_pf *pf = phylink_to_enetc_pf(config);
	struct enetc_si *si = pf->si;
	struct enetc_ndev_priv *priv;

	priv = netdev_priv(si->ndev);

	if (si->hw_features & ENETC_SI_F_QBU)
		enetc_mm_link_state_update(priv, false);

	enetc_mac_enable(si, false);
}

static const struct phylink_mac_ops enetc_mac_phylink_ops = {
	.mac_select_pcs = enetc_pl_mac_select_pcs,
	.mac_config = enetc_pl_mac_config,
	.mac_link_up = enetc_pl_mac_link_up,
	.mac_link_down = enetc_pl_mac_link_down,
};

/* Initialize the entire shared memory for the flow steering entries
 * of this port (PF + VFs)
 */
static int enetc_init_port_rfs_memory(struct enetc_si *si)
{
	struct enetc_cmd_rfse rfse = {0};
	struct enetc_hw *hw = &si->hw;
	int num_rfs, i, err = 0;
	u32 val;

	val = enetc_port_rd(hw, ENETC_PRFSCAPR);
	num_rfs = ENETC_PRFSCAPR_GET_NUM_RFS(val);

	for (i = 0; i < num_rfs; i++) {
		err = enetc_set_fs_entry(si, &rfse, i);
		if (err)
			break;
	}

	return err;
}

static int enetc_init_port_rss_memory(struct enetc_si *si)
{
	struct enetc_hw *hw = &si->hw;
	int num_rss, err;
	int *rss_table;
	u32 val;

	val = enetc_port_rd(hw, ENETC_PRSSCAPR);
	num_rss = ENETC_PRSSCAPR_GET_NUM_RSS(val);
	if (!num_rss)
		return 0;

	rss_table = kcalloc(num_rss, sizeof(*rss_table), GFP_KERNEL);
	if (!rss_table)
		return -ENOMEM;

	err = enetc_set_rss_table(si, rss_table, num_rss);

	kfree(rss_table);

	return err;
}

static int enetc_pf_register_with_ierb(struct pci_dev *pdev)
{
	struct platform_device *ierb_pdev;
	struct device_node *ierb_node;

	ierb_node = of_find_compatible_node(NULL, NULL,
					    "fsl,ls1028a-enetc-ierb");
	if (!ierb_node || !of_device_is_available(ierb_node))
		return -ENODEV;

	ierb_pdev = of_find_device_by_node(ierb_node);
	of_node_put(ierb_node);

	if (!ierb_pdev)
		return -EPROBE_DEFER;

	return enetc_ierb_register_pf(ierb_pdev, pdev);
}

static struct enetc_si *enetc_psi_create(struct pci_dev *pdev)
{
	struct enetc_si *si;
	int err;

	err = enetc_pci_probe(pdev, KBUILD_MODNAME, sizeof(struct enetc_pf));
	if (err) {
		dev_err_probe(&pdev->dev, err, "PCI probing failed\n");
		goto out;
	}

	si = pci_get_drvdata(pdev);
	if (!si->hw.port || !si->hw.global) {
		err = -ENODEV;
		dev_err(&pdev->dev, "could not map PF space, probing a VF?\n");
		goto out_pci_remove;
	}

	err = enetc_init_cbdr(si);
	if (err)
		goto out_pci_remove;

	err = enetc_init_port_rfs_memory(si);
	if (err) {
		dev_err(&pdev->dev, "Failed to initialize RFS memory\n");
		goto out_teardown_cbdr;
	}

	err = enetc_init_port_rss_memory(si);
	if (err) {
		dev_err(&pdev->dev, "Failed to initialize RSS memory\n");
		goto out_teardown_cbdr;
	}

	return si;

out_teardown_cbdr:
	enetc_free_cbdr(si);
out_pci_remove:
	enetc_pci_remove(pdev);
out:
	return ERR_PTR(err);
}

static void enetc_psi_destroy(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);

	enetc_free_cbdr(si);
	enetc_pci_remove(pdev);
}

static int enetc_pf_probe(struct pci_dev *pdev,
			  const struct pci_device_id *ent)
{
	struct device_node *node = pdev->dev.of_node;
	struct enetc_ndev_priv *priv;
	struct net_device *ndev;
	struct enetc_si *si;
	struct enetc_pf *pf;
	int err;

	err = enetc_pf_register_with_ierb(pdev);
	if (err == -EPROBE_DEFER)
		return err;
	if (err)
		dev_warn(&pdev->dev,
			 "Could not register with IERB driver: %pe, please update the device tree\n",
			 ERR_PTR(err));

	si = enetc_psi_create(pdev);
	if (IS_ERR(si)) {
		err = PTR_ERR(si);
		goto err_psi_create;
	}

	pf = enetc_si_priv(si);
	pf->si = si;
	pf->total_vfs = pci_sriov_get_totalvfs(pdev);
	if (pf->total_vfs) {
		pf->vf_state = kcalloc(pf->total_vfs, sizeof(struct enetc_vf_state),
				       GFP_KERNEL);
		if (!pf->vf_state)
			goto err_alloc_vf_state;
	}

	enetc_pf_register_hw_ops(pf, &enetc_pf_hw_ops);

	err = enetc_setup_mac_addresses(node, pf);
	if (err)
		goto err_setup_mac_addresses;

	enetc_configure_port(pf);

	enetc_get_si_caps(si);

	ndev = alloc_etherdev_mq(sizeof(*priv), ENETC_MAX_NUM_TXQS);
	if (!ndev) {
		err = -ENOMEM;
		dev_err(&pdev->dev, "netdev creation failed\n");
		goto err_alloc_netdev;
	}

	enetc_pf_netdev_setup(si, ndev, &enetc_ndev_ops);

	priv = netdev_priv(ndev);

	mutex_init(&priv->mm_lock);

	enetc_init_si_rings_params(priv);

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

	err = of_get_phy_mode(node, &pf->if_mode);
	if (err) {
		dev_err(&pdev->dev, "Failed to read PHY mode\n");
		goto err_phy_mode;
	}

	err = enetc_mdiobus_create(pf, node);
	if (err)
		goto err_mdiobus_create;

	err = enetc_phylink_create(priv, node, &enetc_mac_phylink_ops);
	if (err)
		goto err_phylink_create;

	err = register_netdev(ndev);
	if (err)
		goto err_reg_netdev;

	enetc_tsn_pf_init(ndev, pdev);

	return 0;

err_reg_netdev:
	enetc_phylink_destroy(priv);
err_phylink_create:
	enetc_mdiobus_destroy(pf);
err_mdiobus_create:
err_phy_mode:
	enetc_free_msix(priv);
err_config_si:
err_alloc_msix:
	enetc_free_si_resources(priv);
err_alloc_si_res:
	si->ndev = NULL;
	free_netdev(ndev);
err_alloc_netdev:
err_setup_mac_addresses:
	kfree(pf->vf_state);
err_alloc_vf_state:
	enetc_psi_destroy(pdev);
err_psi_create:
	return err;
}

static void enetc_pf_remove(struct pci_dev *pdev)
{
	struct enetc_si *si = pci_get_drvdata(pdev);
	struct enetc_pf *pf = enetc_si_priv(si);
	struct enetc_ndev_priv *priv;

	priv = netdev_priv(si->ndev);

	if (pf->num_vfs)
		enetc_sriov_configure(pdev, 0);

	enetc_tsn_pf_deinit(si->ndev);

	unregister_netdev(si->ndev);

	enetc_phylink_destroy(priv);
	enetc_mdiobus_destroy(pf);

	enetc_free_msix(priv);

	enetc_free_si_resources(priv);

	free_netdev(si->ndev);
	kfree(pf->vf_state);

	enetc_psi_destroy(pdev);
}

static void enetc_fixup_clear_rss_rfs(struct pci_dev *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct enetc_si *si;

	/* Only apply quirk for disabled functions. For the ones
	 * that are enabled, enetc_pf_probe() will apply it.
	 */
	if (node && of_device_is_available(node))
		return;

	si = enetc_psi_create(pdev);
	if (!IS_ERR(si))
		enetc_psi_destroy(pdev);
}
DECLARE_PCI_FIXUP_FINAL(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_PF,
			enetc_fixup_clear_rss_rfs);

static const struct pci_device_id enetc_pf_id_table[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_FREESCALE, ENETC_DEV_ID_PF) },
	{ 0, } /* End of table. */
};
MODULE_DEVICE_TABLE(pci, enetc_pf_id_table);

static struct pci_driver enetc_pf_driver = {
	.name = KBUILD_MODNAME,
	.id_table = enetc_pf_id_table,
	.probe = enetc_pf_probe,
	.remove = enetc_pf_remove,
#ifdef CONFIG_PCI_IOV
	.sriov_configure = enetc_sriov_configure,
#endif
};
module_pci_driver(enetc_pf_driver);

MODULE_DESCRIPTION(ENETC_DRV_NAME_STR);
MODULE_LICENSE("Dual BSD/GPL");
