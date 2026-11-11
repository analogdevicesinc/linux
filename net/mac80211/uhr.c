// SPDX-License-Identifier: GPL-2.0-only
/*
 * UHR handling
 *
 * Copyright(c) 2025-2026 Intel Corporation
 */

#include "ieee80211_i.h"

void
ieee80211_uhr_cap_ie_to_sta_uhr_cap(struct ieee80211_sub_if_data *sdata,
				    struct ieee80211_supported_band *sband,
				    const struct ieee80211_uhr_cap *uhr_cap,
				    bool from_ap,
				    struct link_sta_info *link_sta)
{
	struct ieee80211_sta_uhr_cap *sta_uhr_cap = &link_sta->pub->uhr_cap;
	const struct ieee80211_uhr_cap_dbe *dbe;
	int idx = 0;

	memset(sta_uhr_cap, 0, sizeof(*sta_uhr_cap));

	if (!ieee80211_get_uhr_iftype_cap_vif(sband, &sdata->vif))
		return;

	sta_uhr_cap->has_uhr = true;
	sta_uhr_cap->mac = uhr_cap->mac;
	sta_uhr_cap->phy = uhr_cap->phy;

	if (!from_ap)
		return;

	dbe = ieee80211_uhr_dbe_cap(uhr_cap);
	if (!dbe)
		return;

	/* must have EHT as well, EHT function must be called first */
	if (WARN_ON(!link_sta->pub->eht_cap.has_eht))
		return;

	if (dbe->cap & IEEE80211_UHR_MAC_CAP_DBE_EHT_MCS_MAP_160_PRES) {
		const struct ieee80211_uhr_cap_dbe_bwcap *bw = &dbe->bwcap[idx++];
		u8 *phy_cap = link_sta->pub->eht_cap.eht_cap_elem.phy_cap_info;

		link_sta->pub->eht_cap.eht_mcs_nss_supp.bw._160 = bw->eht_mcs_map;
		link_sta->pub->he_cap.he_cap_elem.phy_cap_info[0] |=
			IEEE80211_HE_PHY_CAP0_CHANNEL_WIDTH_SET_160MHZ_IN_5G;

		phy_cap[1] =
			u8_replace_bits(phy_cap[1],
					u8_get_bits(bw->cap,
						    IEEE80211_UHR_MAC_CAP_DBE_CAP_BEAMFORMEE_SS),
					IEEE80211_EHT_PHY_CAP1_BEAMFORMEE_SS_160MHZ_MASK);
		phy_cap[2] =
			u8_replace_bits(phy_cap[2],
					u8_get_bits(bw->cap,
						    IEEE80211_UHR_MAC_CAP_DBE_CAP_NUM_SND_DIMS),
					IEEE80211_EHT_PHY_CAP2_SOUNDING_DIM_160MHZ_MASK);
		if (bw->cap & IEEE80211_UHR_MAC_CAP_DBE_CAP_NON_OFDMA_UL_MUMIMO)
			phy_cap[7] |= IEEE80211_EHT_PHY_CAP7_NON_OFDMA_UL_MU_MIMO_160MHZ;
		if (bw->cap & IEEE80211_UHR_MAC_CAP_DBE_CAP_MU_BEAMFORMER)
			phy_cap[7] |= IEEE80211_EHT_PHY_CAP7_MU_BEAMFORMER_160MHZ;
	}

	if (dbe->cap & IEEE80211_UHR_MAC_CAP_DBE_EHT_MCS_MAP_320_PRES) {
		const struct ieee80211_uhr_cap_dbe_bwcap *bw = &dbe->bwcap[idx++];
		u8 *phy_cap = link_sta->pub->eht_cap.eht_cap_elem.phy_cap_info;
		u8 snd_dims = u8_get_bits(bw->cap, IEEE80211_UHR_MAC_CAP_DBE_CAP_NUM_SND_DIMS);

		link_sta->pub->eht_cap.eht_mcs_nss_supp.bw._320 = bw->eht_mcs_map;
		phy_cap[0] |= IEEE80211_EHT_PHY_CAP0_320MHZ_IN_6GHZ;

		phy_cap[1] =
			u8_replace_bits(phy_cap[1],
					u8_get_bits(bw->cap,
						    IEEE80211_UHR_MAC_CAP_DBE_CAP_BEAMFORMEE_SS),
					IEEE80211_EHT_PHY_CAP1_BEAMFORMEE_SS_320MHZ_MASK);
		phy_cap[2] =
			u8_replace_bits(phy_cap[2], snd_dims,
					IEEE80211_EHT_PHY_CAP2_SOUNDING_DIM_320MHZ_MASK);
		phy_cap[3] =
			u8_replace_bits(phy_cap[3], snd_dims >> 2,
					IEEE80211_EHT_PHY_CAP3_SOUNDING_DIM_320MHZ_MASK);
		if (bw->cap & IEEE80211_UHR_MAC_CAP_DBE_CAP_NON_OFDMA_UL_MUMIMO)
			phy_cap[7] |= IEEE80211_EHT_PHY_CAP7_NON_OFDMA_UL_MU_MIMO_320MHZ;
		if (bw->cap & IEEE80211_UHR_MAC_CAP_DBE_CAP_MU_BEAMFORMER)
			phy_cap[7] |= IEEE80211_EHT_PHY_CAP7_MU_BEAMFORMER_320MHZ;
	}
}
