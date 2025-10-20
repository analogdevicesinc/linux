// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
/*
 * Copyright (C) 2024 - 2025 Intel Corporation
 */
#include <net/cfg80211.h>
#include <net/mac80211.h>

#include "mld.h"
#include "roc.h"
#include "hcmd.h"
#include "iface.h"
#include "sta.h"
#include "mlo.h"

#include "fw/api/context.h"
#include "fw/api/time-event.h"

#define AUX_ROC_MAX_DELAY MSEC_TO_TU(200)

static void
iwl_mld_vif_iter_emlsr_block_roc(void *data, u8 *mac, struct ieee80211_vif *vif)
{
	struct iwl_mld_vif *mld_vif = iwl_mld_vif_from_mac80211(vif);
	int *result = data;
	int ret;

	ret = iwl_mld_block_emlsr_sync(mld_vif->mld, vif,
				       IWL_MLD_EMLSR_BLOCKED_ROC,
				       iwl_mld_get_primary_link(vif));
	if (ret)
		*result = ret;
}

struct iwl_mld_roc_iter_data {
	enum iwl_roc_activity activity;
	struct ieee80211_vif *vif;
	bool found;
};

static void iwl_mld_find_roc_vif_iter(void *data, u8 *mac,
				      struct ieee80211_vif *vif)
{
	struct iwl_mld_vif *mld_vif = iwl_mld_vif_from_mac80211(vif);
	struct iwl_mld_roc_iter_data *roc_data = data;

	if (mld_vif->roc_activity != roc_data->activity)
		return;

	/* The FW supports one ROC of each type simultaneously */
	if (WARN_ON(roc_data->found)) {
		roc_data->vif = NULL;
		return;
	}

	roc_data->found = true;
	roc_data->vif = vif;
}

static struct ieee80211_vif *
iwl_mld_find_roc_vif(struct iwl_mld *mld, enum iwl_roc_activity activity)
{
	struct iwl_mld_roc_iter_data roc_data = {
		.activity = activity,
		.found = false,
	};

	ieee80211_iterate_active_interfaces_mtx(mld->hw,
						IEEE80211_IFACE_ITER_NORMAL,
						iwl_mld_find_roc_vif_iter,
						&roc_data);

	return roc_data.vif;
}

int iwl_mld_start_roc(struct ieee80211_hw *hw, struct ieee80211_vif *vif,
		      struct ieee80211_channel *channel, int duration,
		      enum ieee80211_roc_type type)
{
	struct iwl_mld *mld = IWL_MAC80211_GET_MLD(hw);
	struct iwl_mld_vif *mld_vif = iwl_mld_vif_from_mac80211(vif);
	struct iwl_mld_int_sta *aux_sta = &mld_vif->aux_sta;
	struct iwl_roc_req cmd = {
		.action = cpu_to_le32(FW_CTXT_ACTION_ADD),
	};
	enum iwl_roc_activity activity;
	int ret = 0;

	lockdep_assert_wiphy(mld->wiphy);

	if (vif->type != NL80211_IFTYPE_P2P_DEVICE &&
	    vif->type != NL80211_IFTYPE_STATION) {
		IWL_ERR(mld, "NOT SUPPORTED: ROC on vif->type %d\n",
			vif->type);

		return -EOPNOTSUPP;
	}

	if (vif->type == NL80211_IFTYPE_P2P_DEVICE) {
		switch (type) {
		case IEEE80211_ROC_TYPE_NORMAL:
			activity = ROC_ACTIVITY_P2P_DISC;
			break;
		case IEEE80211_ROC_TYPE_MGMT_TX:
			activity = ROC_ACTIVITY_P2P_NEG;
			break;
		default:
			WARN_ONCE(1, "Got an invalid P2P ROC type\n");
			return -EINVAL;
		}
	} else {
		activity = ROC_ACTIVITY_HOTSPOT;
	}

	/* The FW supports one ROC of each type simultaneously */
	if (WARN_ON(iwl_mld_find_roc_vif(mld, activity)))
		return -EBUSY;

	ieee80211_iterate_active_interfaces_mtx(mld->hw,
						IEEE80211_IFACE_ITER_NORMAL,
						iwl_mld_vif_iter_emlsr_block_roc,
						&ret);
	if (ret)
		return ret;

	ret = iwl_mld_add_aux_sta(mld, aux_sta);
	if (ret)
		return ret;

	cmd.activity = cpu_to_le32(activity);
	cmd.sta_id = cpu_to_le32(aux_sta->sta_id);
	cmd.channel_info.channel = cpu_to_le32(channel->hw_value);
	cmd.channel_info.band = iwl_mld_nl80211_band_to_fw(channel->band);
	cmd.channel_info.width = IWL_PHY_CHANNEL_MODE20;
	cmd.max_delay = cpu_to_le32(AUX_ROC_MAX_DELAY);
	cmd.duration = cpu_to_le32(MSEC_TO_TU(duration));

	memcpy(cmd.node_addr, vif->addr, ETH_ALEN);

	ret = iwl_mld_send_cmd_pdu(mld, WIDE_ID(MAC_CONF_GROUP, ROC_CMD),
				   &cmd);
	if (ret) {
		IWL_ERR(mld, "Couldn't send the ROC_CMD\n");
		return ret;
	}

	mld_vif->roc_activity = activity;

	return 0;
}

static void
iwl_mld_vif_iter_emlsr_unblock_roc(void *data, u8 *mac,
				   struct ieee80211_vif *vif)
{
	struct iwl_mld_vif *mld_vif = iwl_mld_vif_from_mac80211(vif);

	iwl_mld_unblock_emlsr(mld_vif->mld, vif, IWL_MLD_EMLSR_BLOCKED_ROC);
}

static void iwl_mld_destroy_roc(struct iwl_mld *mld,
				struct ieee80211_vif *vif,
				struct iwl_mld_vif *mld_vif)
{
	mld_vif->roc_activity = ROC_NUM_ACTIVITIES;

	ieee80211_iterate_active_interfaces_mtx(mld->hw,
						IEEE80211_IFACE_ITER_NORMAL,
						iwl_mld_vif_iter_emlsr_unblock_roc,
						NULL);

	/* wait until every tx has seen that roc_activity has been reset */
	synchronize_net();
	/* from here, no new tx will be added
	 * we can flush the Tx on the queues
	 */

	iwl_mld_flush_link_sta_txqs(mld, mld_vif->aux_sta.sta_id);

	iwl_mld_remove_aux_sta(mld, vif);
}

int iwl_mld_cancel_roc(struct ieee80211_hw *hw,
		       struct ieee80211_vif *vif)
{
	struct iwl_mld *mld = IWL_MAC80211_GET_MLD(hw);
	struct iwl_mld_vif *mld_vif = iwl_mld_vif_from_mac80211(vif);
	struct iwl_roc_req cmd = {
		.action = cpu_to_le32(FW_CTXT_ACTION_REMOVE),
	};
	int ret;

	lockdep_assert_wiphy(mld->wiphy);

	if (WARN_ON(vif->type != NL80211_IFTYPE_P2P_DEVICE &&
		    vif->type != NL80211_IFTYPE_STATION))
		return -EOPNOTSUPP;

	/* No roc activity running it's probably already done */
	if (mld_vif->roc_activity == ROC_NUM_ACTIVITIES)
		return 0;

	cmd.activity = cpu_to_le32(mld_vif->roc_activity);

	ret = iwl_mld_send_cmd_pdu(mld, WIDE_ID(MAC_CONF_GROUP, ROC_CMD),
				   &cmd);
	if (ret)
		IWL_ERR(mld, "Couldn't send the command to cancel the ROC\n");

	/* We may have raced with the firmware expiring the ROC instance at
	 * this very moment. In that case, we can have a notification in the
	 * async processing queue. However, none can arrive _after_ this as
	 * ROC_CMD was sent synchronously, i.e. we waited for a response and
	 * the firmware cannot refer to this ROC after the response. Thus,
	 * if we just cancel the notification (if there's one) we'll be at a
	 * clean state for any possible next ROC.
	 */
	iwl_mld_cancel_notifications_of_object(mld, IWL_MLD_OBJECT_TYPE_ROC,
					       mld_vif->roc_activity);

	iwl_mld_destroy_roc(mld, vif, mld_vif);

	return 0;
}

void iwl_mld_handle_roc_notif(struct iwl_mld *mld,
			      struct iwl_rx_packet *pkt)
{
	const struct iwl_roc_notif *notif = (void *)pkt->data;
	u32 activity = le32_to_cpu(notif->activity);
	struct iwl_mld_vif *mld_vif;
	struct ieee80211_vif *vif;

	vif = iwl_mld_find_roc_vif(mld, activity);
	if (IWL_FW_CHECK(mld, !vif,
			 "unexpected ROC notif from FW for activity %d\n",
			 activity))
		return;

	mld_vif = iwl_mld_vif_from_mac80211(vif);
	/* It is possible that the ROC was canceled
	 * but the notification was already fired.
	 */
	if (mld_vif->roc_activity != activity)
		return;

	if (le32_to_cpu(notif->success) &&
	    le32_to_cpu(notif->started)) {
		/* We had a successful start */
		ieee80211_ready_on_channel(mld->hw);
	} else {
		/* ROC was not successful, tell the firmware to remove it */
		if (le32_to_cpu(notif->started))
			iwl_mld_cancel_roc(mld->hw, vif);
		else
			iwl_mld_destroy_roc(mld, vif, mld_vif);
		/* we need to let know mac80211 about end OR
		 * an unsuccessful start
		 */
		ieee80211_remain_on_channel_expired(mld->hw);
	}
}
