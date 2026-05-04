// SPDX-License-Identifier: GPL-2.0-only
/*
 * mac80211_hwsim_nan - NAN software simulation for mac80211_hwsim
 * Copyright (C) 2025 Intel Corporation
 */

#include "mac80211_hwsim_i.h"

static u8 hwsim_nan_cluster_id[ETH_ALEN];

enum hrtimer_restart
mac80211_hwsim_nan_dw_start(struct hrtimer *timer)
{
	struct mac80211_hwsim_data *data =
		container_of(timer, struct mac80211_hwsim_data,
			     nan.timer);
	struct ieee80211_hw *hw = data->hw;
	u64 orig_tsf = mac80211_hwsim_get_tsf(hw, NULL), tsf = orig_tsf;
	u32 dw_int = 512 * 1024;
	u64 until_dw;

	if (!data->nan.device_vif)
		return HRTIMER_NORESTART;

	if (data->nan.bands & BIT(NL80211_BAND_5GHZ)) {
		if (data->nan.curr_dw_band == NL80211_BAND_2GHZ) {
			dw_int = 128 * 1024;
			data->nan.curr_dw_band = NL80211_BAND_5GHZ;
		} else if (data->nan.curr_dw_band == NL80211_BAND_5GHZ) {
			data->nan.curr_dw_band = NL80211_BAND_2GHZ;
		}
	}

	until_dw = dw_int - do_div(tsf, dw_int);

	/* The timer might fire just before the actual DW, in which case
	 * update the timeout to the actual next DW
	 */
	if (until_dw < dw_int / 2)
		until_dw += dw_int;

	/* The above do_div() call directly modifies the 'tsf' variable, thus,
	 * use a copy so that the print below would show the original TSF.
	 */
	wiphy_debug(hw->wiphy,
		    "%s: tsf=%llx, curr_dw_band=%u, next_dw=%llu\n",
		    __func__, orig_tsf, data->nan.curr_dw_band,
		    until_dw);

	hrtimer_forward_now(&data->nan.timer,
			    ns_to_ktime(until_dw * NSEC_PER_USEC));

	if (data->nan.notify_dw) {
		struct ieee80211_channel *ch;
		struct wireless_dev *wdev =
			ieee80211_vif_to_wdev(data->nan.device_vif);

		if (data->nan.curr_dw_band == NL80211_BAND_5GHZ)
			ch = ieee80211_get_channel(hw->wiphy, 5745);
		else
			ch = ieee80211_get_channel(hw->wiphy, 2437);

		cfg80211_next_nan_dw_notif(wdev, ch, GFP_ATOMIC);
	}

	return HRTIMER_RESTART;
}

int mac80211_hwsim_nan_start(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct cfg80211_nan_conf *conf)
{
	struct mac80211_hwsim_data *data = hw->priv;
	u64 tsf = mac80211_hwsim_get_tsf(hw, NULL);
	u32 dw_int = 512 * 1000;
	u64 until_dw = dw_int - do_div(tsf, dw_int);
	struct wireless_dev *wdev = ieee80211_vif_to_wdev(vif);

	if (vif->type != NL80211_IFTYPE_NAN)
		return -EINVAL;

	if (data->nan.device_vif)
		return -EALREADY;

	/* set this before starting the timer, as preemption might occur */
	data->nan.device_vif = vif;
	data->nan.bands = conf->bands;
	data->nan.curr_dw_band = NL80211_BAND_2GHZ;

	wiphy_debug(hw->wiphy, "nan_started, next_dw=%llu\n",
		    until_dw);

	hrtimer_start(&data->nan.timer,
		      ns_to_ktime(until_dw * NSEC_PER_USEC),
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

	hrtimer_cancel(&data->nan.timer);
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
	if (changes & CFG80211_NAN_CONF_CHANGED_BANDS) {
		data->nan.bands = conf->bands;
		data->nan.curr_dw_band = NL80211_BAND_2GHZ;
	}

	if (changes & CFG80211_NAN_CONF_CHANGED_CONFIG)
		data->nan.notify_dw = conf->enable_dw_notification;

	return 0;
}
