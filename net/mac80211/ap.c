// SPDX-License-Identifier: GPL-2.0-only
/*
 * AP handling
 *
 * Partially
 * Copyright (C) 2026 Intel Corporation
 */

#include "driver-ops.h"
#include "ieee80211_i.h"

static void
ieee80211_send_eml_op_mode_notif(struct ieee80211_sub_if_data *sdata,
				 struct ieee80211_mgmt *req, int opt_len)
{
	int len = IEEE80211_MIN_ACTION_SIZE(eml_omn);
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_mgmt *mgmt;
	struct sk_buff *skb;

	len += opt_len; /* optional len */
	skb = dev_alloc_skb(local->tx_headroom + len);
	if (!skb)
		return;

	skb_reserve(skb, local->tx_headroom);
	mgmt = skb_put_zero(skb, len);
	mgmt->frame_control = cpu_to_le16(IEEE80211_FTYPE_MGMT |
					  IEEE80211_STYPE_ACTION);
	memcpy(mgmt->da, req->sa, ETH_ALEN);
	memcpy(mgmt->sa, sdata->vif.addr, ETH_ALEN);
	memcpy(mgmt->bssid, sdata->vif.addr, ETH_ALEN);

	mgmt->u.action.category = WLAN_CATEGORY_PROTECTED_EHT;
	mgmt->u.action.action_code =
		WLAN_PROTECTED_EHT_ACTION_EML_OP_MODE_NOTIF;
	mgmt->u.action.eml_omn.dialog_token =
		req->u.action.eml_omn.dialog_token;
	mgmt->u.action.eml_omn.control = req->u.action.eml_omn.control &
		~(IEEE80211_EML_CTRL_EMLSR_PARAM_UPDATE |
		  IEEE80211_EML_CTRL_INDEV_COEX_ACT);
	/* Copy optional fields from the received notification frame */
	memcpy(mgmt->u.action.eml_omn.variable,
	       req->u.action.eml_omn.variable, opt_len);

	ieee80211_tx_skb(sdata, skb);
}

static void
ieee80211_rx_eml_op_mode_notif(struct ieee80211_sub_if_data *sdata,
			       struct sk_buff *skb)
{
	int len = IEEE80211_MIN_ACTION_SIZE(eml_omn);
	enum nl80211_iftype type = ieee80211_vif_type_p2p(&sdata->vif);
	struct ieee80211_rx_status *status = IEEE80211_SKB_RXCB(skb);
	const struct wiphy_iftype_ext_capab *ift_ext_capa;
	struct ieee80211_mgmt *mgmt = (void *)skb->data;
	struct ieee80211_local *local = sdata->local;
	u8 control = mgmt->u.action.eml_omn.control;
	u8 *ptr = mgmt->u.action.eml_omn.variable;
	struct ieee80211_eml_params eml_params = {
		.link_id = status->link_id,
		.control = control,
	};
	struct sta_info *sta;
	int opt_len = 0;

	if (!ieee80211_vif_is_mld(&sdata->vif))
		return;

	/* eMLSR and eMLMR can't be enabled at the same time */
	if ((control & IEEE80211_EML_CTRL_EMLSR_MODE) &&
	    (control & IEEE80211_EML_CTRL_EMLMR_MODE))
		return;

	if ((control & IEEE80211_EML_CTRL_EMLMR_MODE) &&
	    (control & IEEE80211_EML_CTRL_EMLSR_PARAM_UPDATE))
		return;

	ift_ext_capa = cfg80211_get_iftype_ext_capa(local->hw.wiphy, type);
	if (!ift_ext_capa)
		return;

	if (!status->link_valid)
		return;

	sta = sta_info_get_bss(sdata, mgmt->sa);
	if (!sta)
		return;

	if (control & IEEE80211_EML_CTRL_EMLSR_MODE) {
		u8 emlsr_param_update_len;

		if (!(ift_ext_capa->eml_capabilities &
		      IEEE80211_EML_CAP_EMLSR_SUPP))
			return;

		opt_len += sizeof(__le16); /* eMLSR link_bitmap */
		/*
		 * eMLSR param update field is not part of Notification frame
		 * sent by the AP to client so account it separately.
		 */
		emlsr_param_update_len =
			!!(control & IEEE80211_EML_CTRL_EMLSR_PARAM_UPDATE);

		if (skb->len < len + opt_len + emlsr_param_update_len)
			return;

		if (control & IEEE80211_EML_CTRL_EMLSR_PARAM_UPDATE) {
			u8 pad_delay, trans_delay;

			pad_delay = u8_get_bits(ptr[2],
						IEEE80211_EML_EMLSR_PAD_DELAY);
			if (pad_delay >
			    IEEE80211_EML_CAP_EML_PADDING_DELAY_256US)
				return;

			trans_delay = u8_get_bits(ptr[2],
					IEEE80211_EML_EMLSR_TRANS_DELAY);
			if (trans_delay >
			    IEEE80211_EML_CAP_EMLSR_TRANSITION_DELAY_256US)
				return;

			/* Update sta padding and transition delay */
			sta->sta.eml_cap =
				u8_replace_bits(sta->sta.eml_cap,
						pad_delay,
						IEEE80211_EML_CAP_EML_PADDING_DELAY);
			sta->sta.eml_cap =
				u8_replace_bits(sta->sta.eml_cap,
						trans_delay,
						IEEE80211_EML_CAP_EML_TRANSITION_DELAY);
		}
	}

	if (control & IEEE80211_EML_CTRL_EMLMR_MODE) {
		u8 mcs_map_size;
		int i;

		if (!(ift_ext_capa->eml_capabilities &
		      IEEE80211_EML_CAP_EMLMR_SUPPORT))
			return;

		opt_len += sizeof(__le16); /* eMLMR link_bitmap */
		opt_len++; /* eMLMR mcs_map_count */
		if (skb->len < len + opt_len)
			return;

		eml_params.emlmr_mcs_map_count = ptr[2];
		if (eml_params.emlmr_mcs_map_count > 2)
			return;

		mcs_map_size = 3 * (1 + eml_params.emlmr_mcs_map_count);
		opt_len += mcs_map_size;
		if (skb->len < len + opt_len)
			return;

		for (i = 0; i < mcs_map_size; i++) {
			u8 rx_mcs, tx_mcs;

			rx_mcs = u8_get_bits(ptr[3 + i],
					     IEEE80211_EML_EMLMR_RX_MCS_MAP);
			if (rx_mcs > 8)
				return;

			tx_mcs = u8_get_bits(ptr[3 + i],
					     IEEE80211_EML_EMLMR_TX_MCS_MAP);
			if (tx_mcs > 8)
				return;
		}

		memcpy(eml_params.emlmr_mcs_map_bw, &ptr[3], mcs_map_size);
	}

	if ((control & IEEE80211_EML_CTRL_EMLSR_MODE) ||
	    (control & IEEE80211_EML_CTRL_EMLMR_MODE)) {
		eml_params.link_bitmap = get_unaligned_le16(ptr);
		if ((eml_params.link_bitmap & sdata->vif.active_links) !=
		    eml_params.link_bitmap)
			return;
	}

	if (drv_set_eml_op_mode(sdata, &sta->sta, &eml_params))
		return;

	ieee80211_send_eml_op_mode_notif(sdata, mgmt, opt_len);
}

void ieee80211_ap_rx_queued_frame(struct ieee80211_sub_if_data *sdata,
				  struct sk_buff *skb)
{
	struct ieee80211_mgmt *mgmt = (void *)skb->data;

	/* rx.c cannot queue any non-action frames to AP interfaces */
	if (WARN_ON(!ieee80211_is_action(mgmt->frame_control)))
		return;

	switch (mgmt->u.action.category) {
	case WLAN_CATEGORY_PROTECTED_EHT:
		switch (mgmt->u.action.action_code) {
		case WLAN_PROTECTED_EHT_ACTION_EML_OP_MODE_NOTIF:
			ieee80211_rx_eml_op_mode_notif(sdata, skb);
			break;
		}
		break;
	}
}
