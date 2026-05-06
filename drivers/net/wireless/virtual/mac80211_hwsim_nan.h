// SPDX-License-Identifier: GPL-2.0-only
/*
 * mac80211_hwsim_nan - NAN software simulation for mac80211_hwsim
 * Copyright (C) 2025 Intel Corporation
 */

#ifndef __MAC80211_HWSIM_NAN_H
#define __MAC80211_HWSIM_NAN_H

struct mac80211_hwsim_nan_data {
	struct ieee80211_vif *device_vif;
	u8 bands;

	/* Current channel of the NAN device */
	struct ieee80211_channel *channel;

	struct hrtimer slot_timer;
	bool notify_dw;
};

enum hrtimer_restart
mac80211_hwsim_nan_slot_timer(struct hrtimer *timer);

int mac80211_hwsim_nan_start(struct ieee80211_hw *hw,
			     struct ieee80211_vif *vif,
			     struct cfg80211_nan_conf *conf);

int mac80211_hwsim_nan_stop(struct ieee80211_hw *hw,
			    struct ieee80211_vif *vif);

int mac80211_hwsim_nan_change_config(struct ieee80211_hw *hw,
				     struct ieee80211_vif *vif,
				     struct cfg80211_nan_conf *conf,
				     u32 changes);

#endif /* __MAC80211_HWSIM_NAN_H */
