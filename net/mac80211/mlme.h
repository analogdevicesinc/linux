/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MLME declarations.
 *
 * Copyright (C) 2026 Intel Corporation
 */
#ifndef IEEE80211_MLME_H
#define IEEE80211_MLME_H
#include "ieee80211_i.h"

#define PRESENT_ELEMS_MAX	8

void run_again(struct ieee80211_sub_if_data *sdata, unsigned long timeout);
void __ieee80211_disconnect(struct ieee80211_sub_if_data *sdata);
u8 ieee80211_mgd_next_dialog_token(struct ieee80211_sub_if_data *sdata);
size_t ieee80211_add_link_elems(struct ieee80211_sub_if_data *sdata,
				struct sk_buff *skb, u16 *capab,
				const struct element *ext_capa,
				const u8 *extra_elems, size_t extra_elems_len,
				unsigned int link_id,
				struct ieee80211_link_data *link,
				u16 *present_elems,
				struct ieee80211_mgd_assoc_data *assoc_data);
void ieee80211_add_non_inheritance_elem(struct sk_buff *skb,
					const u16 *outer, const u16 *inner);
int ieee80211_link_common_elems_size(struct ieee80211_sub_if_data *sdata,
				     enum nl80211_iftype iftype,
				     struct cfg80211_bss *cbss,
				     size_t elems_len);
void ieee80211_determine_our_sta_mode(struct ieee80211_sub_if_data *sdata,
				      struct ieee80211_supported_band *sband,
				      struct cfg80211_assoc_request *req,
				      bool wmm_used, int link_id,
				      struct ieee80211_conn_settings *conn);

struct ieee80211_assoc_config_link_params {
	struct link_sta_info *link_sta;
	struct cfg80211_bss *cbss;
	struct ieee80211_mgmt *mgmt;
	enum ieee80211_parse_action_frame action_frame;
	const u8 *elem_start;
	unsigned int elem_len;
	bool skip_bw_check;
};

bool ieee80211_assoc_config_link(struct ieee80211_mgd_assoc_data *assoc_data,
				 struct ieee80211_link_data *link,
				 const struct ieee80211_assoc_config_link_params *params,
				 u64 *changed);
int ieee80211_calc_channel(struct ieee80211_sub_if_data *sdata,
			   struct ieee80211_link_data *link,
			   struct link_sta_info *link_sta, int link_id,
			   struct cfg80211_bss *cbss, bool mlo,
			   struct ieee80211_conn_settings *conn,
			   unsigned long *userspace_selectors);
int ieee80211_mgd_get_ap_ht_vht_capa(struct ieee80211_sub_if_data *sdata,
				     struct ieee80211_mgd_assoc_data *assoc_data,
				     int link_id);
int ieee80211_mgd_setup_link_sta(struct ieee80211_link_data *link,
				 struct link_sta_info *link_sta,
				 struct cfg80211_bss *cbss);
void ieee80211_mgd_link_beacon_info(struct ieee80211_link_data *link,
				    struct cfg80211_bss *cbss);
u64 ieee80211_link_set_associated(struct ieee80211_link_data *link,
				  struct cfg80211_bss *cbss);
void ieee80211_send_uhr_omp_req_dbe(struct ieee80211_sub_if_data *sdata,
				    u16 link_mask, bool initial);
void ieee80211_mgd_reset_ap_state(struct ieee80211_sub_if_data *sdata);

#endif /* IEEE80211_MLME_H */
