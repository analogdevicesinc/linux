// SPDX-License-Identifier: GPL-2.0-only
/*
 * mac80211_hwsim_nan - NAN software simulation for mac80211_hwsim
 * Copyright (C) 2025 Intel Corporation
 */

#include "mac80211_hwsim_i.h"

/* Defined as the lower 23 bits being zero */
#define DW0_TSF_MASK		GENMASK(22, 0)

/* DWs are repeated every 512 TUs */
#define DWST_TU			512
#define DWST_TSF_MASK		(ieee80211_tu_to_usec(DWST_TU) - 1)

#define SLOT_TU			16
#define SLOT_TSF_MASK		(ieee80211_tu_to_usec(DWST_TU) - 1)

/* The 2.4 GHz DW is at the start, the 5 GHz is in slot 8 (after 128 TUs) */
#define DW_5G_OFFSET_TU		128

#define SLOT_24GHZ_DW		0
#define SLOT_5GHZ_DW		(DW_5G_OFFSET_TU / SLOT_TU)

/* The special DW0 happens every 16 DWSTs (8192 TUs) */
static_assert(16 * DWST_TU * 1024 == 8192 * 1024);
static_assert(DW0_TSF_MASK + 1 == 8192 * 1024);

static u8 hwsim_nan_cluster_id[ETH_ALEN];

static u64 hwsim_nan_get_timer_tsf(struct mac80211_hwsim_data *data)
{
	ktime_t expires = hrtimer_get_expires(&data->nan.slot_timer);

	return mac80211_hwsim_boottime_to_tsf(data, expires);
}

static u8 hwsim_nan_slot_from_tsf(u64 tsf)
{
	return (tsf & DWST_TSF_MASK) / ieee80211_tu_to_usec(SLOT_TU);
}

static void
mac80211_hwsim_nan_schedule_slot(struct mac80211_hwsim_data *data, u8 slot)
{
	u64 tsf = hwsim_nan_get_timer_tsf(data);

	/* Only called by mac80211_hwsim_nan_dw_timer from softirq context */
	lockdep_assert_in_softirq();

	tsf &= ~DWST_TSF_MASK;
	tsf += ieee80211_tu_to_usec(slot * SLOT_TU);

	hrtimer_set_expires(&data->nan.slot_timer,
			    mac80211_hwsim_tsf_to_boottime(data, tsf));
}

static void
mac80211_hwsim_nan_exec_state_transitions(struct mac80211_hwsim_data *data)
{
	/*
	 * Handle NAN role and state transitions at the end of the DW period
	 * in accordance to Wi-Fi Aware version 4.0 section 3.3.7 point 2, i.e.
	 * end of 5 GHz DW if enabled else at the end of the 2.4 GHz DW.
	 *
	 * TODO: Implement
	 */
}

enum hrtimer_restart
mac80211_hwsim_nan_slot_timer(struct hrtimer *timer)
{
	struct mac80211_hwsim_data *data =
		container_of(timer, struct mac80211_hwsim_data,
			     nan.slot_timer);
	struct ieee80211_hw *hw = data->hw;
	struct ieee80211_channel *notify_dw_chan = NULL;
	u64 tsf = hwsim_nan_get_timer_tsf(data);
	u8 slot = hwsim_nan_slot_from_tsf(tsf);
	bool dwst_of_dw0 = false;
	bool dw_end = false;

	if (!data->nan.device_vif)
		return HRTIMER_NORESTART;

	if ((tsf & DW0_TSF_MASK & ~DWST_TSF_MASK) == 0)
		dwst_of_dw0 = true;


	switch (slot) {
	case SLOT_24GHZ_DW:
		wiphy_dbg(data->hw->wiphy, "Start of 2.4 GHz DW, is DW0=%d\n",
			  dwst_of_dw0);
		data->nan.channel = ieee80211_get_channel(hw->wiphy, 2437);
		break;

	case SLOT_24GHZ_DW + 1:
		if (!(data->nan.bands & BIT(NL80211_BAND_5GHZ))) {
			notify_dw_chan = ieee80211_get_channel(hw->wiphy, 2437);
			dw_end = true;
		} else {
			notify_dw_chan = ieee80211_get_channel(hw->wiphy, 5745);
		}
		break;

	case SLOT_5GHZ_DW:
		if (data->nan.bands & BIT(NL80211_BAND_5GHZ)) {
			wiphy_dbg(data->hw->wiphy, "Start of 5 GHz DW\n");
			data->nan.channel =
				ieee80211_get_channel(hw->wiphy, 5745);
		}
		break;

	case SLOT_5GHZ_DW + 1:
		if (data->nan.bands & BIT(NL80211_BAND_5GHZ)) {
			notify_dw_chan =
				ieee80211_get_channel(hw->wiphy, 2437);
			dw_end = true;
		}
		break;
	}

	if (dw_end)
		mac80211_hwsim_nan_exec_state_transitions(data);

	if (data->nan.notify_dw && notify_dw_chan) {
		struct wireless_dev *wdev =
			ieee80211_vif_to_wdev(data->nan.device_vif);

		cfg80211_next_nan_dw_notif(wdev, notify_dw_chan, GFP_ATOMIC);
	}

	mac80211_hwsim_nan_schedule_slot(data, slot + 1);

	return HRTIMER_RESTART;
}

int mac80211_hwsim_nan_start(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct cfg80211_nan_conf *conf)
{
	struct mac80211_hwsim_data *data = hw->priv;
	struct wireless_dev *wdev = ieee80211_vif_to_wdev(vif);

	if (vif->type != NL80211_IFTYPE_NAN)
		return -EINVAL;

	if (data->nan.device_vif)
		return -EALREADY;

	/* set this before starting the timer, as preemption might occur */
	data->nan.device_vif = vif;
	data->nan.bands = conf->bands;
	data->nan.channel = ieee80211_get_channel(hw->wiphy, 2437);

	/* Just run this "soon" and start in a random schedule position */
	hrtimer_start(&data->nan.slot_timer,
		      ns_to_ktime(10 * NSEC_PER_USEC),
		      HRTIMER_MODE_REL_SOFT);

	if (!is_zero_ether_addr(conf->cluster_id) &&
	    is_zero_ether_addr(hwsim_nan_cluster_id)) {
		memcpy(hwsim_nan_cluster_id, conf->cluster_id, ETH_ALEN);
	} else if (is_zero_ether_addr(hwsim_nan_cluster_id)) {
		hwsim_nan_cluster_id[0] = 0x50;
		hwsim_nan_cluster_id[1] = 0x6f;
		hwsim_nan_cluster_id[2] = 0x9a;
		hwsim_nan_cluster_id[3] = 0x01;
		hwsim_nan_cluster_id[4] = get_random_u8();
		hwsim_nan_cluster_id[5] = get_random_u8();
	}

	data->nan.notify_dw = conf->enable_dw_notification;

	cfg80211_nan_cluster_joined(wdev, hwsim_nan_cluster_id, true,
				    GFP_KERNEL);

	return 0;
}

int mac80211_hwsim_nan_stop(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif)
{
	struct mac80211_hwsim_data *data = hw->priv;
	struct mac80211_hwsim_data *data2;
	bool nan_cluster_running = false;

	if (vif->type != NL80211_IFTYPE_NAN || !data->nan.device_vif ||
	    data->nan.device_vif != vif)
		return -EINVAL;

	hrtimer_cancel(&data->nan.slot_timer);
	data->nan.device_vif = NULL;

	spin_lock_bh(&hwsim_radio_lock);
	list_for_each_entry(data2, &hwsim_radios, list) {
		if (data2->nan.device_vif) {
			nan_cluster_running = true;
			break;
		}
	}
	spin_unlock_bh(&hwsim_radio_lock);

	if (!nan_cluster_running)
		memset(hwsim_nan_cluster_id, 0, ETH_ALEN);

	return 0;
}

int mac80211_hwsim_nan_change_config(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     struct cfg80211_nan_conf *conf,
				     u32 changes)
{
	struct mac80211_hwsim_data *data = hw->priv;

	if (vif->type != NL80211_IFTYPE_NAN)
		return -EINVAL;

	if (!data->nan.device_vif)
		return -EINVAL;

	wiphy_debug(hw->wiphy, "nan_config_changed: changes=0x%x\n", changes);

	/* Handle only the changes we care about for simulation purposes */
	if (changes & CFG80211_NAN_CONF_CHANGED_BANDS)
		data->nan.bands = conf->bands;

	if (changes & CFG80211_NAN_CONF_CHANGED_CONFIG)
		data->nan.notify_dw = conf->enable_dw_notification;

	return 0;
}
