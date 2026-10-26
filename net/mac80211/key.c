// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2002-2005, Instant802 Networks, Inc.
 * Copyright 2005-2006, Devicescape Software, Inc.
 * Copyright 2006-2007	Jiri Benc <jbenc@suse.cz>
 * Copyright 2007-2008	Johannes Berg <johannes@sipsolutions.net>
 * Copyright 2013-2014  Intel Mobile Communications GmbH
 * Copyright 2015-2017	Intel Deutschland GmbH
 * Copyright 2018-2020, 2022-2026  Intel Corporation
 */

#include <crypto/utils.h>
#include <linux/if_ether.h>
#include <linux/etherdevice.h>
#include <linux/list.h>
#include <linux/rcupdate.h>
#include <linux/rtnetlink.h>
#include <linux/slab.h>
#include <linux/export.h>
#include <net/mac80211.h>
#include <linux/unaligned.h>
#include "ieee80211_i.h"
#include "driver-ops.h"
#include "debugfs_key.h"
#include "sta_info.h"
#include "aes_ccm.h"
#include "aes_cmac.h"
#include "aes_gmac.h"
#include "aes_gcm.h"


/**
 * DOC: Key handling basics
 *
 * Key handling in mac80211 is done based on per-interface (sub_if_data)
 * keys and per-station keys. Since each station belongs to an interface,
 * each station key also belongs to that interface.
 *
 * Hardware acceleration is done on a best-effort basis for algorithms
 * that are implemented in software,  for each key the hardware is asked
 * to enable that key for offloading but if it cannot do that the key is
 * simply kept for software encryption (unless it is for an algorithm
 * that isn't implemented in software).
 * There is currently no way of knowing whether a key is handled in SW
 * or HW except by looking into debugfs.
 *
 * All key management is internally protected by a mutex. Within all
 * other parts of mac80211, key references are, just as STA structure
 * references, protected by RCU. Note, however, that some things are
 * unprotected, namely the key->sta dereferences within the hardware
 * acceleration functions. This means that sta_info_destroy() must
 * remove the key which waits for an RCU grace period.
 */

static const u8 bcast_addr[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

static void
update_vlan_tailroom_need_count(struct ieee80211_sub_if_data *sdata, int delta)
{
	struct ieee80211_sub_if_data *vlan;

	if (sdata->vif.type != NL80211_IFTYPE_AP)
		return;

	/* crypto_tx_tailroom_needed_cnt is protected by this */
	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	rcu_read_lock();

	list_for_each_entry_rcu(vlan, &sdata->u.ap.vlans, u.vlan.list)
		vlan->crypto_tx_tailroom_needed_cnt += delta;

	rcu_read_unlock();
}

static void increment_tailroom_need_count(struct ieee80211_sub_if_data *sdata)
{
	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	update_vlan_tailroom_need_count(sdata, 1);
	sdata->crypto_tx_tailroom_needed_cnt++;
}

static void decrease_tailroom_need_count(struct ieee80211_sub_if_data *sdata,
					 int delta)
{
	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	WARN_ON_ONCE(sdata->crypto_tx_tailroom_needed_cnt < delta);

	update_vlan_tailroom_need_count(sdata, -delta);
	sdata->crypto_tx_tailroom_needed_cnt -= delta;
}

static bool ieee80211_key_needs_tailroom(struct ieee80211_key *key)
{
	if (key->conf.flags & (IEEE80211_KEY_FLAG_GENERATE_MMIC |
			       IEEE80211_KEY_FLAG_PUT_MIC_SPACE |
			       IEEE80211_KEY_FLAG_RESERVE_TAILROOM))
		return true;

	return !(key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE);
}

static int ieee80211_key_enable_hw_accel(struct ieee80211_key *key)
{
	struct ieee80211_sub_if_data *sdata = key->sdata;
	struct sta_info *sta;
	int ret = -EOPNOTSUPP;

	might_sleep();
	lockdep_assert_wiphy(key->local->hw.wiphy);

	if (key->flags & KEY_FLAG_TAINTED) {
		/* If we get here, it's during resume and the key is
		 * tainted so shouldn't be used/programmed any more.
		 * However, its flags may still indicate that it was
		 * programmed into the device (since we're in resume)
		 * so clear that flag now to avoid trying to remove
		 * it again later.
		 */
		if (!ieee80211_key_needs_tailroom(key))
			increment_tailroom_need_count(sdata);

		key->flags &= ~KEY_FLAG_UPLOADED_TO_HARDWARE;
		return -EINVAL;
	}

	if (!key->local->ops->set_key)
		goto out_unsupported;

	sta = key->sta;

	/*
	 * Allow installation of a per-STA GTK if per-STA GTK is supported
	 * by the driver or the interface is a NAN Data interface (as
	 * per-station GTKs are required to be supported if secure NAN is
	 * supported).
	 */
	if (sta && !(key->conf.flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
		switch (sdata->vif.type) {
		case NL80211_IFTYPE_NAN_DATA:
			break;
		case NL80211_IFTYPE_ADHOC:
		case NL80211_IFTYPE_MESH_POINT:
			if (!(ieee80211_hw_check(&key->local->hw,
						 SUPPORTS_PER_STA_GTK)))
				goto out_unsupported;
			break;
		case NL80211_IFTYPE_STATION:
			/*
			 * unless requested otherwise, NULL out
			 * the STA for client group keys
			 */
			if (!ieee80211_hw_check(&key->local->hw,
						PER_STA_AP_GTK))
				sta = NULL;
			break;
		default:
			goto out_unsupported;
		}
	}

	if (sta && !sta->uploaded)
		goto out_unsupported;

	if (sdata->vif.type == NL80211_IFTYPE_AP_VLAN) {
		/*
		 * The driver doesn't know anything about VLAN interfaces.
		 * Hence, don't send GTKs for VLAN interfaces to the driver.
		 */
		if (!(key->conf.flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
			ret = 1;
			goto out_unsupported;
		}
	}

	if (key->conf.link_id >= 0 && sdata->vif.active_links &&
	    !(sdata->vif.active_links & BIT(key->conf.link_id)))
		return 0;

	ret = drv_set_key(key->local, SET_KEY, sdata,
			  sta ? &sta->sta : NULL, &key->conf);

	if (!ret) {
		key->flags |= KEY_FLAG_UPLOADED_TO_HARDWARE;

		if (!ieee80211_key_needs_tailroom(key))
			decrease_tailroom_need_count(sdata, 1);

		WARN_ON((key->conf.flags & IEEE80211_KEY_FLAG_PUT_IV_SPACE) &&
			(key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_IV));

		WARN_ON((key->conf.flags & IEEE80211_KEY_FLAG_PUT_MIC_SPACE) &&
			(key->conf.flags & IEEE80211_KEY_FLAG_GENERATE_MMIC));

		return 0;
	}

	if (ret != -ENOSPC && ret != -EOPNOTSUPP && ret != 1)
		sdata_err(sdata,
			  "failed to set key (%d, %pM) to hardware (%d)\n",
			  key->conf.keyidx,
			  sta ? sta->sta.addr : bcast_addr, ret);

 out_unsupported:
	/* Control Integrity Protocol can only be done in hardware */
	if (key->conf.flags & IEEE80211_KEY_FLAG_CIP)
		return -EINVAL;

	switch (key->conf.cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
	case WLAN_CIPHER_SUITE_TKIP:
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		/* all of these we can do in software - if driver can */
		if (ret == 1)
			return 0;
		if (ieee80211_hw_check(&key->local->hw, SW_CRYPTO_CONTROL))
			return -EINVAL;
		return 0;
	default:
		return -EINVAL;
	}
}

static void ieee80211_key_disable_hw_accel(struct ieee80211_key *key)
{
	struct ieee80211_sub_if_data *sdata;
	struct ieee80211_sta *pubsta;
	struct sta_info *sta;
	int ret;

	might_sleep();

	if (!key || !key->local->ops->set_key)
		return;

	if (!(key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE))
		return;

	sta = key->sta;
	sdata = key->sdata;

	lockdep_assert_wiphy(key->local->hw.wiphy);

	if (key->conf.link_id >= 0 && sdata->vif.active_links &&
	    !(sdata->vif.active_links & BIT(key->conf.link_id)))
		return;

	if (!ieee80211_key_needs_tailroom(key))
		increment_tailroom_need_count(sdata);

	pubsta = sta ? &sta->sta : NULL;
	/* unless requested otherwise, NULL out the STA for client group keys */
	if (sdata->vif.type == NL80211_IFTYPE_STATION &&
	    !(key->conf.flags & IEEE80211_KEY_FLAG_PAIRWISE) &&
	    !ieee80211_hw_check(&key->local->hw, PER_STA_AP_GTK))
		pubsta = NULL;

	key->flags &= ~KEY_FLAG_UPLOADED_TO_HARDWARE;
	ret = drv_set_key(key->local, DISABLE_KEY, sdata, pubsta, &key->conf);

	if (ret)
		sdata_err(sdata,
			  "failed to remove key (%d, %pM) from hardware (%d)\n",
			  key->conf.keyidx,
			  pubsta ? pubsta->addr : bcast_addr, ret);
}

static int _ieee80211_set_tx_key(struct ieee80211_key *key, bool force)
{
	struct sta_info *sta = key->sta;
	struct ieee80211_local *local = key->local;

	lockdep_assert_wiphy(local->hw.wiphy);

	set_sta_flag(sta, WLAN_STA_USES_ENCRYPTION);

	sta->ptk_idx = key->conf.keyidx;

	if (force || !ieee80211_hw_check(&local->hw, AMPDU_KEYBORDER_SUPPORT))
		clear_sta_flag(sta, WLAN_STA_BLOCK_BA);
	ieee80211_check_fast_xmit(sta);

	return 0;
}

int ieee80211_set_tx_key(struct ieee80211_key *key)
{
	return _ieee80211_set_tx_key(key, false);
}

static void ieee80211_pairwise_rekey(struct ieee80211_key *old,
				     struct ieee80211_key *new)
{
	struct ieee80211_local *local = new->local;
	struct sta_info *sta = new->sta;
	int i;

	lockdep_assert_wiphy(local->hw.wiphy);

	if (new->conf.flags & IEEE80211_KEY_FLAG_NO_AUTO_TX) {
		/* Extended Key ID key install, initial one or rekey */

		if (sta->ptk_idx != INVALID_PTK_KEYIDX &&
		    !ieee80211_hw_check(&local->hw, AMPDU_KEYBORDER_SUPPORT)) {
			/* Aggregation Sessions with Extended Key ID must not
			 * mix MPDUs with different keyIDs within one A-MPDU.
			 * Tear down running Tx aggregation sessions and block
			 * new Rx/Tx aggregation requests during rekey to
			 * ensure there are no A-MPDUs when the driver is not
			 * supporting A-MPDU key borders. (Blocking Tx only
			 * would be sufficient but WLAN_STA_BLOCK_BA gets the
			 * job done for the few ms we need it.)
			 */
			set_sta_flag(sta, WLAN_STA_BLOCK_BA);
			for (i = 0; i <  IEEE80211_NUM_TIDS; i++)
				__ieee80211_stop_tx_ba_session(sta, i,
							       AGG_STOP_LOCAL_REQUEST);
		}
	} else if (old) {
		/* Rekey without Extended Key ID.
		 * Aggregation sessions are OK when running on SW crypto.
		 * A broken remote STA may cause issues not observed with HW
		 * crypto, though.
		 */
		if (!(old->flags & KEY_FLAG_UPLOADED_TO_HARDWARE))
			return;

		/* Stop Tx till we are on the new key */
		old->flags |= KEY_FLAG_TAINTED;
		ieee80211_clear_fast_xmit(sta);
		if (ieee80211_hw_check(&local->hw, AMPDU_AGGREGATION)) {
			set_sta_flag(sta, WLAN_STA_BLOCK_BA);
			ieee80211_sta_tear_down_BA_sessions(sta,
							    AGG_STOP_LOCAL_REQUEST);
		}
		if (!wiphy_ext_feature_isset(local->hw.wiphy,
					     NL80211_EXT_FEATURE_CAN_REPLACE_PTK0)) {
			pr_warn_ratelimited("Rekeying PTK for STA %pM but driver can't safely do that.",
					    sta->sta.addr);
			/* Flushing the driver queues *may* help prevent
			 * the clear text leaks and freezes.
			 */
			ieee80211_flush_queues(local, old->sdata, false);
		}
	}
}

static void __ieee80211_set_default_key(struct ieee80211_link_data *link,
					int idx, bool uni, bool multi)
{
	struct ieee80211_sub_if_data *sdata = link->sdata;
	struct ieee80211_key *key = NULL;

	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	if (idx >= 0 && idx < NUM_DEFAULT_KEYS) {
		key = wiphy_dereference(sdata->local->hw.wiphy,
					sdata->keys[idx]);
		if (!key)
			key = wiphy_dereference(sdata->local->hw.wiphy,
						link->tx_gtk[idx]);
	}

	if (uni) {
		rcu_assign_pointer(sdata->default_unicast_key, key);
		ieee80211_check_fast_xmit_iface(sdata);
		if (sdata->vif.type != NL80211_IFTYPE_AP_VLAN)
			drv_set_default_unicast_key(sdata->local, sdata, idx);
	}

	if (multi)
		rcu_assign_pointer(link->default_multicast_key, key);

	ieee80211_debugfs_key_update_default(sdata);
}

void ieee80211_set_default_key(struct ieee80211_link_data *link, int idx,
			       bool uni, bool multi)
{
	lockdep_assert_wiphy(link->sdata->local->hw.wiphy);

	__ieee80211_set_default_key(link, idx, uni, multi);
}

static void
__ieee80211_set_default_mgmt_key(struct ieee80211_link_data *link, int idx)
{
	struct ieee80211_sub_if_data *sdata = link->sdata;
	struct ieee80211_key *key = NULL;

	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	if (idx >= NUM_DEFAULT_KEYS &&
	    idx < NUM_DEFAULT_KEYS + NUM_DEFAULT_MGMT_KEYS)
		key = wiphy_dereference(sdata->local->hw.wiphy,
					link->tx_gtk[idx]);

	rcu_assign_pointer(link->default_mgmt_key, key);

	ieee80211_debugfs_key_update_default(sdata);
}

void ieee80211_set_default_mgmt_key(struct ieee80211_link_data *link,
				    int idx)
{
	lockdep_assert_wiphy(link->sdata->local->hw.wiphy);

	__ieee80211_set_default_mgmt_key(link, idx);
}

static void
__ieee80211_set_default_beacon_key(struct ieee80211_link_data *link, int idx)
{
	struct ieee80211_sub_if_data *sdata = link->sdata;
	struct ieee80211_key *key = NULL;

	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	if (idx >= NUM_DEFAULT_KEYS + NUM_DEFAULT_MGMT_KEYS &&
	    idx < NUM_DEFAULT_KEYS + NUM_DEFAULT_MGMT_KEYS +
	    NUM_DEFAULT_BEACON_KEYS)
		key = wiphy_dereference(sdata->local->hw.wiphy,
					link->tx_gtk[idx]);

	rcu_assign_pointer(link->default_beacon_key, key);

	ieee80211_debugfs_key_update_default(sdata);
}

void ieee80211_set_default_beacon_key(struct ieee80211_link_data *link,
				      int idx)
{
	lockdep_assert_wiphy(link->sdata->local->hw.wiphy);

	__ieee80211_set_default_beacon_key(link, idx);
}

int ieee80211_key_slot_lookup(struct ieee80211_sub_if_data *sdata,
			      int link_id, u8 key_idx, u32 cipher,
			      enum nl80211_key_type type,
			      const u8 *mac_addr,
			      struct ieee80211_key_slot *slot)
{
	struct ieee80211_local *local __maybe_unused = sdata->local;
	bool pairwise = type == NL80211_KEYTYPE_PAIRWISE;
	bool cigtk = type == NL80211_KEYTYPE_CIGTK;
	bool is_wep = cipher == WLAN_CIPHER_SUITE_WEP40 ||
		      cipher == WLAN_CIPHER_SUITE_WEP104;
	int err;

	memset(slot, 0, sizeof(*slot));

	if (mac_addr) {
		slot->sta = sta_info_get_bss(sdata, mac_addr);
		if (!slot->sta) {
			err = -ENOENT;
			goto fail;
		}

		if (pairwise && key_idx < NUM_DEFAULT_KEYS) {
			slot->key = &slot->sta->ptk[key_idx];
			return 0;
		}

		if (link_id >= 0) {
			slot->link_sta = wiphy_dereference(local->hw.wiphy,
							   slot->sta->link[link_id]);
			if (!slot->link_sta) {
				err = -ENOENT;
				goto fail;
			}
		} else if (!ieee80211_vif_is_mld(&sdata->vif)) {
			slot->link_sta = &slot->sta->deflink;
		} else {
			err = -EINVAL;
			goto fail;
		}

		if (cigtk) {
			slot->key = &slot->link_sta->rx_cigtk[key_idx];
			return 0;
		}

		if (!pairwise &&
		    key_idx < NUM_DEFAULT_KEYS +
			      NUM_DEFAULT_MGMT_KEYS +
			      NUM_DEFAULT_BEACON_KEYS) {
			slot->key = &slot->link_sta->rx_gtk[key_idx];
			return 0;
		}

		err = -EINVAL;
		goto fail;
	}

	if (is_wep && key_idx < NUM_DEFAULT_KEYS) {
		/* WEP keys can also be default/deflink TX keys */
		slot->link = &sdata->deflink;
		slot->key = &sdata->keys[key_idx];
		return 0;
	}

	if (link_id >= 0) {
		slot->link = sdata_dereference(sdata->link[link_id], sdata);
		if (!slot->link) {
			err = -ENOLINK;
			goto fail;
		}
	} else {
		slot->link = &sdata->deflink;
	}

	if (cigtk) {
		slot->key = &slot->link->tx_cigtk[key_idx];
		return 0;
	}

	slot->key = &slot->link->tx_gtk[key_idx];
	if (cipher != 0)
		return 0;

	/* handle the special case for key lookup with cipher==0 */
	if (rcu_access_pointer(*slot->key))
		return 0;
	if (key_idx < NUM_DEFAULT_KEYS) {
		slot->key = &sdata->keys[key_idx];
		return 0;
	}

	err = -EINVAL;
fail:
	memset(slot, 0, sizeof(*slot));
	return err;
}

static int ieee80211_key_replace(struct ieee80211_sub_if_data *sdata,
				 struct ieee80211_key_slot *slot,
				 struct ieee80211_key *old,
				 struct ieee80211_key *new)
{
	bool defunikey, defmultikey, defmgmtkey, defbeaconkey;
	struct ieee80211_local *local = sdata->local;
	bool pairwise;
	int ret = 0;
	u32 flags;

	lockdep_assert_wiphy(local->hw.wiphy);

	/* the old key is passed in for convenience, but validate it */
	WARN_ON(old != rcu_access_pointer(*slot->key));

	/* caller must provide at least one old/new */
	if (WARN_ON(!new && !old))
		return 0;

	flags = old ? old->conf.flags : new->conf.flags;
	pairwise = flags & IEEE80211_KEY_FLAG_PAIRWISE;

	WARN_ON(new && old && new->conf.keyidx != old->conf.keyidx);
	WARN_ON(new && old && new->conf.link_id != old->conf.link_id);

	if (new && slot->sta && pairwise) {
		/* Unicast rekey needs special handling. With Extended Key ID
		 * old is still NULL for the first rekey.
		 */
		ieee80211_pairwise_rekey(old, new);
	}

	if (old) {
		if (old->flags & KEY_FLAG_UPLOADED_TO_HARDWARE) {
			ieee80211_key_disable_hw_accel(old);

			if (new)
				ret = ieee80211_key_enable_hw_accel(new);
		}
	} else {
		if (!local->wowlan) {
			ret = ieee80211_key_enable_hw_accel(new);
		} else if (new->conf.link_id < 0 || !sdata->vif.active_links ||
			 BIT(new->conf.link_id) & sdata->vif.active_links) {
			new->flags |= KEY_FLAG_UPLOADED_TO_HARDWARE;
			if (!ieee80211_key_needs_tailroom(new))
				decrease_tailroom_need_count(sdata, 1);
		}
	}

	if (ret)
		return ret;

	if (new) {
		spin_lock_bh(&sdata->local->key_lock);
		list_add_tail(&new->list, &sdata->key_list);
		spin_unlock_bh(&sdata->local->key_lock);
	}

	rcu_assign_pointer(*slot->key, new);

	if (slot->sta) {
		if (pairwise) {
			if (new &&
			    !(new->conf.flags & IEEE80211_KEY_FLAG_NO_AUTO_TX))
				_ieee80211_set_tx_key(new, true);

			/*
			 * Only needed for transition from no key -> key.
			 * Still triggers unnecessary when using Extended Key ID
			 * and installing the second key ID the first time.
			 */
			if (new && !old)
				ieee80211_check_fast_rx(slot->sta);
		}
	} else if (slot->link) {
		struct ieee80211_link_data *link = slot->link;

		defunikey = old &&
			old == wiphy_dereference(local->hw.wiphy,
						 sdata->default_unicast_key);
		defmultikey = old &&
			old == wiphy_dereference(local->hw.wiphy,
						 link->default_multicast_key);
		defmgmtkey = old &&
			old == wiphy_dereference(local->hw.wiphy,
						 link->default_mgmt_key);
		defbeaconkey = old &&
			old == wiphy_dereference(local->hw.wiphy,
						 link->default_beacon_key);

		if (defunikey && !new)
			__ieee80211_set_default_key(link, -1, true, false);
		if (defmultikey && !new)
			__ieee80211_set_default_key(link, -1, false, true);
		if (defmgmtkey && !new)
			__ieee80211_set_default_mgmt_key(link, -1);
		if (defbeaconkey && !new)
			__ieee80211_set_default_beacon_key(link, -1);

		if (defunikey && new)
			__ieee80211_set_default_key(link, new->conf.keyidx,
						    true, false);
		if (defmultikey && new)
			__ieee80211_set_default_key(link, new->conf.keyidx,
						    false, true);
		if (defmgmtkey && new)
			__ieee80211_set_default_mgmt_key(link,
							 new->conf.keyidx);
		if (defbeaconkey && new)
			__ieee80211_set_default_beacon_key(link,
							   new->conf.keyidx);
	}

	if (old) {
		spin_lock_bh(&sdata->local->key_lock);
		list_del(&old->list);
		spin_unlock_bh(&sdata->local->key_lock);
	}

	return 0;
}

struct ieee80211_key *
ieee80211_key_alloc(u32 cipher, int idx, size_t key_len,
		    const u8 *key_data,
		    size_t seq_len, const u8 *seq)
{
	struct ieee80211_key *key;
	int i, j, err;

	if (WARN_ON(idx < 0 ||
		    idx >= NUM_DEFAULT_KEYS + NUM_DEFAULT_MGMT_KEYS +
		    NUM_DEFAULT_BEACON_KEYS))
		return ERR_PTR(-EINVAL);

	key = kzalloc(sizeof(struct ieee80211_key) + key_len, GFP_KERNEL);
	if (!key)
		return ERR_PTR(-ENOMEM);

	/*
	 * Default to software encryption; we'll later upload the
	 * key to the hardware if possible.
	 */
	key->conf.flags = 0;
	key->flags = 0;

	key->conf.link_id = -1;
	key->conf.cipher = cipher;
	key->conf.keyidx = idx;
	key->conf.keylen = key_len;
	switch (cipher) {
	case WLAN_CIPHER_SUITE_WEP40:
	case WLAN_CIPHER_SUITE_WEP104:
		key->conf.iv_len = IEEE80211_WEP_IV_LEN;
		key->conf.icv_len = IEEE80211_WEP_ICV_LEN;
		break;
	case WLAN_CIPHER_SUITE_TKIP:
		key->conf.iv_len = IEEE80211_TKIP_IV_LEN;
		key->conf.icv_len = IEEE80211_TKIP_ICV_LEN;
		if (seq) {
			for (i = 0; i < IEEE80211_NUM_TIDS; i++) {
				key->u.tkip.rx[i].iv32 =
					get_unaligned_le32(&seq[2]);
				key->u.tkip.rx[i].iv16 =
					get_unaligned_le16(seq);
			}
		}
		spin_lock_init(&key->u.tkip.txlock);
		break;
	case WLAN_CIPHER_SUITE_CCMP:
		key->conf.iv_len = IEEE80211_CCMP_HDR_LEN;
		key->conf.icv_len = IEEE80211_CCMP_MIC_LEN;
		if (seq) {
			for (i = 0; i < IEEE80211_NUM_TIDS + 1; i++)
				for (j = 0; j < IEEE80211_CCMP_PN_LEN; j++)
					key->u.ccmp.rx_pn[i][j] =
						seq[IEEE80211_CCMP_PN_LEN - j - 1];
		}
		/*
		 * Initialize AES key state here as an optimization so that
		 * it does not need to be initialized for every packet.
		 */
		key->u.ccmp.tfm = ieee80211_aes_key_setup_encrypt(
			key_data, key_len, IEEE80211_CCMP_MIC_LEN);
		if (IS_ERR(key->u.ccmp.tfm)) {
			err = PTR_ERR(key->u.ccmp.tfm);
			kfree(key);
			return ERR_PTR(err);
		}
		break;
	case WLAN_CIPHER_SUITE_CCMP_256:
		key->conf.iv_len = IEEE80211_CCMP_256_HDR_LEN;
		key->conf.icv_len = IEEE80211_CCMP_256_MIC_LEN;
		for (i = 0; seq && i < IEEE80211_NUM_TIDS + 1; i++)
			for (j = 0; j < IEEE80211_CCMP_256_PN_LEN; j++)
				key->u.ccmp.rx_pn[i][j] =
					seq[IEEE80211_CCMP_256_PN_LEN - j - 1];
		/* Initialize AES key state here as an optimization so that
		 * it does not need to be initialized for every packet.
		 */
		key->u.ccmp.tfm = ieee80211_aes_key_setup_encrypt(
			key_data, key_len, IEEE80211_CCMP_256_MIC_LEN);
		if (IS_ERR(key->u.ccmp.tfm)) {
			err = PTR_ERR(key->u.ccmp.tfm);
			kfree(key);
			return ERR_PTR(err);
		}
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		key->conf.iv_len = 0;
		if (cipher == WLAN_CIPHER_SUITE_AES_CMAC)
			key->conf.icv_len = sizeof(struct ieee80211_mmie);
		else
			key->conf.icv_len = sizeof(struct ieee80211_mmie_16);
		if (seq)
			for (j = 0; j < IEEE80211_CMAC_PN_LEN; j++)
				key->u.aes_cmac.rx_pn[j] =
					seq[IEEE80211_CMAC_PN_LEN - j - 1];
		/*
		 * Initialize AES key state here as an optimization so that
		 * it does not need to be initialized for every packet.
		 */
		err = aes_cmac_preparekey(&key->u.aes_cmac.key, key_data,
					  key_len);
		if (err) {
			kfree(key);
			return ERR_PTR(err);
		}
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		key->conf.iv_len = 0;
		key->conf.icv_len = sizeof(struct ieee80211_mmie_16);
		if (seq)
			for (j = 0; j < IEEE80211_GMAC_PN_LEN; j++)
				key->u.aes_gmac.rx_pn[j] =
					seq[IEEE80211_GMAC_PN_LEN - j - 1];
		/* Initialize AES key state here as an optimization so that
		 * it does not need to be initialized for every packet.
		 */
		key->u.aes_gmac.tfm =
			ieee80211_aes_gmac_key_setup(key_data, key_len);
		if (IS_ERR(key->u.aes_gmac.tfm)) {
			err = PTR_ERR(key->u.aes_gmac.tfm);
			kfree(key);
			return ERR_PTR(err);
		}
		break;
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
		key->conf.iv_len = IEEE80211_GCMP_HDR_LEN;
		key->conf.icv_len = IEEE80211_GCMP_MIC_LEN;
		for (i = 0; seq && i < IEEE80211_NUM_TIDS + 1; i++)
			for (j = 0; j < IEEE80211_GCMP_PN_LEN; j++)
				key->u.gcmp.rx_pn[i][j] =
					seq[IEEE80211_GCMP_PN_LEN - j - 1];
		/* Initialize AES key state here as an optimization so that
		 * it does not need to be initialized for every packet.
		 */
		key->u.gcmp.tfm = ieee80211_aes_gcm_key_setup_encrypt(key_data,
								      key_len);
		if (IS_ERR(key->u.gcmp.tfm)) {
			err = PTR_ERR(key->u.gcmp.tfm);
			kfree(key);
			return ERR_PTR(err);
		}
		break;
	}
	memcpy(key->conf.key, key_data, key_len);
	INIT_LIST_HEAD(&key->list);

	return key;
}

static void ieee80211_key_free_common(struct ieee80211_key *key)
{
	switch (key->conf.cipher) {
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
		ieee80211_aes_key_free(key->u.ccmp.tfm);
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		ieee80211_aes_gmac_key_free(key->u.aes_gmac.tfm);
		break;
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
		ieee80211_aes_gcm_key_free(key->u.gcmp.tfm);
		break;
	}
	kfree_sensitive(key);
}

static void __ieee80211_key_cleanup(struct ieee80211_key *key,
				    bool delay_tailroom)
{
	if (key->local) {
		struct ieee80211_sub_if_data *sdata = key->sdata;

		ieee80211_debugfs_key_remove(key);

		if (delay_tailroom) {
			/* see ieee80211_delayed_tailroom_dec */
			sdata->crypto_tx_tailroom_pending_dec++;
			wiphy_delayed_work_queue(sdata->local->hw.wiphy,
						 &sdata->dec_tailroom_needed_wk,
						 HZ / 2);
		} else {
			decrease_tailroom_need_count(sdata, 1);
		}
	}
}

static void __ieee80211_key_destroy(struct ieee80211_key *key,
				    bool delay_tailroom)
{
	__ieee80211_key_cleanup(key, delay_tailroom);

	ieee80211_key_free_common(key);
}

static void ieee80211_key_destroy(struct ieee80211_key *key,
				  bool delay_tailroom)
{
	if (!key)
		return;

	/*
	 * Synchronize so the TX path can no longer be using this
	 * key before we free/remove it.
	 */
	synchronize_net();

	__ieee80211_key_destroy(key, delay_tailroom);
}

void ieee80211_key_free_unused(struct ieee80211_key *key)
{
	if (!key)
		return;

	WARN_ON(key->sdata || key->local);
	ieee80211_key_free_common(key);
}

static bool ieee80211_key_identical(struct ieee80211_sub_if_data *sdata,
				    struct ieee80211_key *old,
				    struct ieee80211_key *new)
{
	u8 tkip_old[WLAN_KEY_LEN_TKIP], tkip_new[WLAN_KEY_LEN_TKIP];
	u8 *tk_old, *tk_new;

	if (!old || new->conf.keylen != old->conf.keylen)
		return false;

	tk_old = old->conf.key;
	tk_new = new->conf.key;

	/*
	 * In station mode, don't compare the TX MIC key, as it's never used
	 * and offloaded rekeying may not care to send it to the host. This
	 * is the case in iwlwifi, for example.
	 */
	if (sdata->vif.type == NL80211_IFTYPE_STATION &&
	    new->conf.cipher == WLAN_CIPHER_SUITE_TKIP &&
	    new->conf.keylen == WLAN_KEY_LEN_TKIP &&
	    !(new->conf.flags & IEEE80211_KEY_FLAG_PAIRWISE)) {
		memcpy(tkip_old, tk_old, WLAN_KEY_LEN_TKIP);
		memcpy(tkip_new, tk_new, WLAN_KEY_LEN_TKIP);
		memset(tkip_old + NL80211_TKIP_DATA_OFFSET_TX_MIC_KEY, 0, 8);
		memset(tkip_new + NL80211_TKIP_DATA_OFFSET_TX_MIC_KEY, 0, 8);
		tk_old = tkip_old;
		tk_new = tkip_new;
	}

	return !crypto_memneq(tk_old, tk_new, new->conf.keylen);
}

int ieee80211_key_link(struct ieee80211_sub_if_data *sdata,
		       struct ieee80211_key_slot *slot,
		       struct ieee80211_key *key)
{
	/*
	 * We want to delay tailroom updates only for station - in that
	 * case it helps roaming speed, but in other cases it hurts and
	 * can cause warnings to appear.
	 */
	bool delay_tailroom = sdata->vif.type == NL80211_IFTYPE_STATION;
	bool pairwise = key->conf.flags & IEEE80211_KEY_FLAG_PAIRWISE;
	static atomic_t key_color = ATOMIC_INIT(0);
	struct ieee80211_key *old_key = NULL;
	int idx = key->conf.keyidx;
	int ret;

	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	old_key = wiphy_dereference(sdata->local->hw.wiphy, *slot->key);

	if (pairwise) {
		struct ieee80211_key *alt_key;

		alt_key = wiphy_dereference(sdata->local->hw.wiphy,
					    slot->sta->ptk[idx ^ 1]);

		/*
		 * The rekey code assumes that the old and new key are using
		 * the same cipher. Enforce the assumption for pairwise keys.
		 * NAN Data interfaces are exempt: Wi-Fi Aware v4.0 section 7.4
		 * requires upgrading the ND-TKSA when a new NDP negotiates a
		 * stronger cipher suite.
		 */
		if (sdata->vif.type != NL80211_IFTYPE_NAN_DATA &&
		    ((alt_key && alt_key->conf.cipher != key->conf.cipher) ||
		     (old_key && old_key->conf.cipher != key->conf.cipher))) {
			ret = -EOPNOTSUPP;
			goto out;
		}

		/* Set CIP flag if enabled for the station */
		if (slot->sta->sta.cip)
			key->conf.flags |= IEEE80211_KEY_FLAG_CIP;
	} else {
		/* Non-pairwise keys must also not switch the cipher on rekey */
		if (old_key && old_key->conf.cipher != key->conf.cipher) {
			ret = -EOPNOTSUPP;
			goto out;
		}
	}

	/*
	 * Silently accept key re-installation without really installing the
	 * new version of the key to avoid nonce reuse or replay issues.
	 */
	if (ieee80211_key_identical(sdata, old_key, key)) {
		ret = -EALREADY;
		goto out;
	}

	key->local = sdata->local;
	key->sdata = sdata;
	key->sta = slot->sta;

	/*
	 * Assign a unique ID to every key so we can easily prevent mixed
	 * key and fragment cache attacks.
	 */
	key->color = atomic_inc_return(&key_color);

	/* keep this flag for easier access later */
	if (pairwise && slot->sta->sta.spp_amsdu)
		key->conf.flags |= IEEE80211_KEY_FLAG_SPP_AMSDU;

	/* A CIP related key must be GCMP-256 (really GMAC-256) */
	if ((key->conf.flags & IEEE80211_KEY_FLAG_CIP) &&
	    key->conf.cipher != WLAN_CIPHER_SUITE_GCMP_256) {
		ret = -EINVAL;
		goto out;
	}

	increment_tailroom_need_count(sdata);

	ret = ieee80211_key_replace(sdata, slot, old_key, key);
	if (ret) {
		decrease_tailroom_need_count(sdata, 1);
		key->local = NULL;
		key->sdata = NULL;
		goto out;
	}

	ieee80211_debugfs_key_add(key);
	ieee80211_key_destroy(old_key, delay_tailroom);

	return 0;
 out:
	ieee80211_key_free_unused(key);
	return ret;
}

void ieee80211_key_free(struct ieee80211_sub_if_data *sdata,
			struct ieee80211_key_slot *slot,
			bool delay_tailroom)
{
	struct ieee80211_key *key;

	key = sdata_dereference(*slot->key, sdata);
	if (!key)
		return;

	/* replace key with nothing if it was ever used */
	if (key->sdata)
		ieee80211_key_replace(key->sdata, slot, key, NULL);
	ieee80211_key_destroy(key, delay_tailroom);
}

void ieee80211_reenable_keys(struct ieee80211_sub_if_data *sdata)
{
	struct ieee80211_key *key;
	struct ieee80211_sub_if_data *vlan;

	lockdep_assert_wiphy(sdata->local->hw.wiphy);

	sdata->crypto_tx_tailroom_needed_cnt = 0;
	sdata->crypto_tx_tailroom_pending_dec = 0;

	if (sdata->vif.type == NL80211_IFTYPE_AP) {
		list_for_each_entry(vlan, &sdata->u.ap.vlans, u.vlan.list) {
			vlan->crypto_tx_tailroom_needed_cnt = 0;
			vlan->crypto_tx_tailroom_pending_dec = 0;
		}
	}

	if (ieee80211_sdata_running(sdata)) {
		list_for_each_entry(key, &sdata->key_list, list) {
			if (!(key->flags & KEY_FLAG_TAINTED))
				increment_tailroom_need_count(sdata);
			ieee80211_key_enable_hw_accel(key);
		}
	}
}

static void
ieee80211_key_iter(struct ieee80211_hw *hw,
		   struct ieee80211_vif *vif,
		   struct ieee80211_key *key,
		   void (*iter)(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				struct ieee80211_sta *sta,
				struct ieee80211_key_conf *key,
				void *data),
		   void *iter_data)
{
	struct sta_info *sta = key->sta;

	if (!(key->flags & KEY_FLAG_UPLOADED_TO_HARDWARE))
		return;

	/* unless requested otherwise, NULL out the STA for client group keys */
	if (vif->type == NL80211_IFTYPE_STATION &&
	    !(key->conf.flags & IEEE80211_KEY_FLAG_PAIRWISE) &&
	    !ieee80211_hw_check(&key->local->hw, PER_STA_AP_GTK))
		sta = NULL;

	/* skip keys of station in removal process (if driver knew about STA) */
	if (sta && sta->removed)
		return;

	iter(hw, vif, sta ? &sta->sta : NULL, &key->conf, iter_data);
}

void ieee80211_iter_keys(struct ieee80211_hw *hw,
			 struct ieee80211_vif *vif,
			 void (*iter)(struct ieee80211_hw *hw,
				      struct ieee80211_vif *vif,
				      struct ieee80211_sta *sta,
				      struct ieee80211_key_conf *key,
				      void *data),
			 void *iter_data)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_key *key, *tmp;
	struct ieee80211_sub_if_data *sdata;

	lockdep_assert_wiphy(hw->wiphy);

	if (vif) {
		sdata = vif_to_sdata(vif);
		list_for_each_entry_safe(key, tmp, &sdata->key_list, list)
			ieee80211_key_iter(hw, vif, key, iter, iter_data);
	} else {
		list_for_each_entry(sdata, &local->interfaces, list)
			list_for_each_entry_safe(key, tmp,
						 &sdata->key_list, list)
				ieee80211_key_iter(hw, &sdata->vif, key,
						   iter, iter_data);
	}
}
EXPORT_SYMBOL(ieee80211_iter_keys);

static void
_ieee80211_iter_keys_atomic(struct ieee80211_hw *hw,
			    struct ieee80211_sub_if_data *sdata,
			    void (*iter)(struct ieee80211_hw *hw,
					 struct ieee80211_vif *vif,
					 struct ieee80211_sta *sta,
					 struct ieee80211_key_conf *key,
					 void *data),
			    void *iter_data)
{
	struct ieee80211_key *key;

	list_for_each_entry(key, &sdata->key_list, list)
		ieee80211_key_iter(hw, &sdata->vif, key, iter, iter_data);
}

void ieee80211_iter_keys_atomic(struct ieee80211_hw *hw,
				struct ieee80211_vif *vif,
				void (*iter)(struct ieee80211_hw *hw,
					     struct ieee80211_vif *vif,
					     struct ieee80211_sta *sta,
					     struct ieee80211_key_conf *key,
					     void *data),
				void *iter_data)
{
	struct ieee80211_local *local = hw_to_local(hw);
	struct ieee80211_sub_if_data *sdata;

	spin_lock_bh(&local->key_lock);
	if (vif) {
		sdata = vif_to_sdata(vif);
		_ieee80211_iter_keys_atomic(hw, sdata, iter, iter_data);
	} else {
		rcu_read_lock();
		list_for_each_entry_rcu(sdata, &local->interfaces, list)
			_ieee80211_iter_keys_atomic(hw, sdata, iter, iter_data);
		rcu_read_unlock();
	}
	spin_unlock_bh(&local->key_lock);
}
EXPORT_SYMBOL(ieee80211_iter_keys_atomic);

static void ieee80211_free_keys_iface(struct ieee80211_sub_if_data *sdata,
				      struct list_head *keys)
{
	decrease_tailroom_need_count(sdata,
				     sdata->crypto_tx_tailroom_pending_dec);
	sdata->crypto_tx_tailroom_pending_dec = 0;

	ieee80211_debugfs_key_remove_mgmt_default(sdata);
	ieee80211_debugfs_key_remove_beacon_default(sdata);

	for (int i = 0; i < ARRAY_SIZE(sdata->keys); i++) {
		struct ieee80211_key_slot slot = {
			/* WEP keys can also be default/deflink TX keys */
			.link = &sdata->deflink,
			.key = &sdata->keys[i],
		};
		struct ieee80211_key *key;

		key = sdata_dereference(*slot.key, sdata);
		if (!key)
			continue;

		ieee80211_key_replace(sdata, &slot, key, NULL);
		list_add_tail(&key->list, keys);
	}

	ieee80211_debugfs_key_update_default(sdata);
}

void ieee80211_remove_link_keys(struct ieee80211_link_data *link,
				struct list_head *keys)
{
	struct ieee80211_sub_if_data *sdata = link->sdata;
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_key_slot slot = {
		.link = link,
	};

	lockdep_assert_wiphy(local->hw.wiphy);

	for (int i = 0; i < ARRAY_SIZE(link->tx_gtk); i++) {
		struct ieee80211_key *key;

		slot.key = &link->tx_gtk[i];
		key = sdata_dereference(*slot.key, sdata);
		if (!key)
			continue;

		ieee80211_key_replace(sdata, &slot, key, NULL);
		list_add_tail(&key->list, keys);
	}

	for (int i = 0; i < ARRAY_SIZE(link->tx_cigtk); i++) {
		struct ieee80211_key *key;

		slot.key = &link->tx_cigtk[i];
		key = sdata_dereference(*slot.key, sdata);
		if (!key)
			continue;

		ieee80211_key_replace(sdata, &slot, key, NULL);
		list_add_tail(&key->list, keys);
	}
}

void ieee80211_free_key_list(struct ieee80211_local *local,
			     struct list_head *keys)
{
	struct ieee80211_key *key, *tmp;

	lockdep_assert_wiphy(local->hw.wiphy);

	list_for_each_entry_safe(key, tmp, keys, list)
		__ieee80211_key_destroy(key, false);
}

void ieee80211_free_keys(struct ieee80211_sub_if_data *sdata,
			 bool force_synchronize)
{
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_sub_if_data *vlan;
	struct ieee80211_sub_if_data *master;
	struct ieee80211_key *key, *tmp;
	LIST_HEAD(keys);

	wiphy_delayed_work_cancel(local->hw.wiphy,
				  &sdata->dec_tailroom_needed_wk);

	lockdep_assert_wiphy(local->hw.wiphy);

	ieee80211_remove_link_keys(&sdata->deflink, &keys);
	ieee80211_free_keys_iface(sdata, &keys);

	if (sdata->vif.type == NL80211_IFTYPE_AP) {
		list_for_each_entry(vlan, &sdata->u.ap.vlans, u.vlan.list) {
			ieee80211_remove_link_keys(&vlan->deflink, &keys);
			ieee80211_free_keys_iface(vlan, &keys);
		}
	}

	WARN_ON(!list_empty(&sdata->key_list));

	if (!list_empty(&keys) || force_synchronize)
		synchronize_net();
	list_for_each_entry_safe(key, tmp, &keys, list)
		__ieee80211_key_destroy(key, false);

	if (sdata->vif.type == NL80211_IFTYPE_AP_VLAN) {
		if (sdata->bss) {
			master = container_of(sdata->bss,
					      struct ieee80211_sub_if_data,
					      u.ap);

			WARN_ON_ONCE(sdata->crypto_tx_tailroom_needed_cnt !=
				     master->crypto_tx_tailroom_needed_cnt);
		}
	} else {
		WARN_ON_ONCE(sdata->crypto_tx_tailroom_needed_cnt ||
			     sdata->crypto_tx_tailroom_pending_dec);
	}

	if (sdata->vif.type == NL80211_IFTYPE_AP) {
		list_for_each_entry(vlan, &sdata->u.ap.vlans, u.vlan.list)
			WARN_ON_ONCE(vlan->crypto_tx_tailroom_needed_cnt ||
				     vlan->crypto_tx_tailroom_pending_dec);
	}
}

void ieee80211_unlink_link_sta_keys(struct ieee80211_local *local,
				    struct link_sta_info *link_sta)
{
	struct ieee80211_key_slot slot = {
		.sta = link_sta->sta,
		.link_sta = link_sta,
	};

	lockdep_assert_wiphy(local->hw.wiphy);

	for (int i = 0; i < ARRAY_SIZE(link_sta->rx_gtk); i++) {
		struct ieee80211_key *key;
		int ret;

		slot.key = &link_sta->rx_gtk[i];
		key = wiphy_dereference(local->hw.wiphy, *slot.key);
		if (!key)
			continue;
		ret = ieee80211_key_replace(key->sdata, &slot, key, NULL);
		if (WARN(ret, "failed to remove STA link key (%d)\n", ret))
			continue;
		list_add(&key->list, &link_sta->key_destroy_list);
		__ieee80211_key_cleanup(key, key->sdata->vif.type ==
					     NL80211_IFTYPE_STATION);
	}

	for (int i = 0; i < ARRAY_SIZE(link_sta->rx_cigtk); i++) {
		struct ieee80211_key *key;
		int ret;

		slot.key = &link_sta->rx_cigtk[i];
		key = wiphy_dereference(local->hw.wiphy, *slot.key);
		if (!key)
			continue;
		ret = ieee80211_key_replace(key->sdata, &slot, key, NULL);
		if (WARN(ret, "failed to remove STA link key (%d)\n", ret))
			continue;
		list_add(&key->list, &link_sta->key_destroy_list);
		__ieee80211_key_cleanup(key, key->sdata->vif.type ==
					     NL80211_IFTYPE_STATION);
	}
}

void ieee80211_free_link_sta_keys(struct link_sta_info *link_sta)
{
	struct ieee80211_key *key, *tmp;

	list_for_each_entry_safe(key, tmp, &link_sta->key_destroy_list, list) {
		list_del(&key->list);
		ieee80211_key_free_common(key);
	}
}

void ieee80211_free_sta_keys(struct ieee80211_local *local,
			     struct sta_info *sta)
{
	struct ieee80211_key_slot slot = {
		.sta = sta,
	};

	lockdep_assert_wiphy(local->hw.wiphy);

	for (int i = 0; i < ARRAY_SIZE(sta->ptk); i++) {
		struct ieee80211_key *key;

		slot.key = &sta->ptk[i];
		key = wiphy_dereference(local->hw.wiphy, *slot.key);
		if (!key)
			continue;
		ieee80211_key_replace(key->sdata, &slot, key, NULL);
		__ieee80211_key_destroy(key, key->sdata->vif.type ==
					NL80211_IFTYPE_STATION);
	}
}

void ieee80211_delayed_tailroom_dec(struct wiphy *wiphy,
				    struct wiphy_work *wk)
{
	struct ieee80211_sub_if_data *sdata;

	sdata = container_of(wk, struct ieee80211_sub_if_data,
			     dec_tailroom_needed_wk.work);

	/*
	 * The reason for the delayed tailroom needed decrementing is to
	 * make roaming faster: during roaming, all keys are first deleted
	 * and then new keys are installed. The first new key causes the
	 * crypto_tx_tailroom_needed_cnt to go from 0 to 1, which invokes
	 * the cost of synchronize_net() (which can be slow). Avoid this
	 * by deferring the crypto_tx_tailroom_needed_cnt decrementing on
	 * key removal for a while, so if we roam the value is larger than
	 * zero and no 0->1 transition happens.
	 *
	 * The cost is that if the AP switching was from an AP with keys
	 * to one without, we still allocate tailroom while it would no
	 * longer be needed. However, in the typical (fast) roaming case
	 * within an ESS this usually won't happen.
	 */

	decrease_tailroom_need_count(sdata,
				     sdata->crypto_tx_tailroom_pending_dec);
	sdata->crypto_tx_tailroom_pending_dec = 0;
}

void ieee80211_gtk_rekey_notify(struct ieee80211_vif *vif, const u8 *bssid,
				const u8 *replay_ctr, gfp_t gfp)
{
	struct ieee80211_sub_if_data *sdata = vif_to_sdata(vif);

	trace_api_gtk_rekey_notify(sdata, bssid, replay_ctr);

	cfg80211_gtk_rekey_notify(sdata->dev, bssid, replay_ctr, gfp);
}
EXPORT_SYMBOL_GPL(ieee80211_gtk_rekey_notify);

void ieee80211_get_key_rx_seq(struct ieee80211_key_conf *keyconf,
			      int tid, struct ieee80211_key_seq *seq)
{
	struct ieee80211_key *key;
	const u8 *pn;

	key = container_of(keyconf, struct ieee80211_key, conf);

	switch (key->conf.cipher) {
	case WLAN_CIPHER_SUITE_TKIP:
		if (WARN_ON(tid < 0 || tid >= IEEE80211_NUM_TIDS))
			return;
		seq->tkip.iv32 = key->u.tkip.rx[tid].iv32;
		seq->tkip.iv16 = key->u.tkip.rx[tid].iv16;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
		if (WARN_ON(tid < -1 || tid >= IEEE80211_NUM_TIDS))
			return;
		if (tid < 0)
			pn = key->u.ccmp.rx_pn[IEEE80211_NUM_TIDS];
		else
			pn = key->u.ccmp.rx_pn[tid];
		memcpy(seq->ccmp.pn, pn, IEEE80211_CCMP_PN_LEN);
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		if (WARN_ON(tid != 0))
			return;
		pn = key->u.aes_cmac.rx_pn;
		memcpy(seq->aes_cmac.pn, pn, IEEE80211_CMAC_PN_LEN);
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		if (WARN_ON(tid != 0))
			return;
		pn = key->u.aes_gmac.rx_pn;
		memcpy(seq->aes_gmac.pn, pn, IEEE80211_GMAC_PN_LEN);
		break;
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
		if (WARN_ON(tid < -1 || tid >= IEEE80211_NUM_TIDS))
			return;
		if (tid < 0)
			pn = key->u.gcmp.rx_pn[IEEE80211_NUM_TIDS];
		else
			pn = key->u.gcmp.rx_pn[tid];
		memcpy(seq->gcmp.pn, pn, IEEE80211_GCMP_PN_LEN);
		break;
	}
}
EXPORT_SYMBOL(ieee80211_get_key_rx_seq);

void ieee80211_set_key_rx_seq(struct ieee80211_key_conf *keyconf,
			      int tid, struct ieee80211_key_seq *seq)
{
	struct ieee80211_key *key;
	u8 *pn;

	key = container_of(keyconf, struct ieee80211_key, conf);

	switch (key->conf.cipher) {
	case WLAN_CIPHER_SUITE_TKIP:
		if (WARN_ON(tid < 0 || tid >= IEEE80211_NUM_TIDS))
			return;
		key->u.tkip.rx[tid].iv32 = seq->tkip.iv32;
		key->u.tkip.rx[tid].iv16 = seq->tkip.iv16;
		break;
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
		if (WARN_ON(tid < -1 || tid >= IEEE80211_NUM_TIDS))
			return;
		if (tid < 0)
			pn = key->u.ccmp.rx_pn[IEEE80211_NUM_TIDS];
		else
			pn = key->u.ccmp.rx_pn[tid];
		memcpy(pn, seq->ccmp.pn, IEEE80211_CCMP_PN_LEN);
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		if (WARN_ON(tid != 0))
			return;
		pn = key->u.aes_cmac.rx_pn;
		memcpy(pn, seq->aes_cmac.pn, IEEE80211_CMAC_PN_LEN);
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		if (WARN_ON(tid != 0))
			return;
		pn = key->u.aes_gmac.rx_pn;
		memcpy(pn, seq->aes_gmac.pn, IEEE80211_GMAC_PN_LEN);
		break;
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
		if (WARN_ON(tid < -1 || tid >= IEEE80211_NUM_TIDS))
			return;
		if (tid < 0)
			pn = key->u.gcmp.rx_pn[IEEE80211_NUM_TIDS];
		else
			pn = key->u.gcmp.rx_pn[tid];
		memcpy(pn, seq->gcmp.pn, IEEE80211_GCMP_PN_LEN);
		break;
	default:
		WARN_ON(1);
		break;
	}
}
EXPORT_SYMBOL_GPL(ieee80211_set_key_rx_seq);

struct ieee80211_key_conf *
ieee80211_gtk_rekey_add(struct ieee80211_vif *vif,
			u8 idx, u8 *key_data, u8 key_len,
			int link_id, bool cigtk)
{
	struct ieee80211_sub_if_data *sdata = vif_to_sdata(vif);
	struct ieee80211_local *local = sdata->local;
	struct ieee80211_key_slot slot = {};
	struct link_sta_info *link_sta;
	struct ieee80211_key *prev_key;
	struct ieee80211_key *key;
	struct sta_info *sta;
	int err;
	struct ieee80211_link_data *link_data =
		link_id < 0 ? &sdata->deflink :
		sdata_dereference(sdata->link[link_id], sdata);

	if (WARN_ON(!link_data))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(!local->wowlan))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(vif->type != NL80211_IFTYPE_STATION))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(idx >= NUM_DEFAULT_KEYS + NUM_DEFAULT_MGMT_KEYS +
		    NUM_DEFAULT_BEACON_KEYS))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(cigtk && idx >= NUM_CTRL_KEYS))
		return ERR_PTR(-EINVAL);

	sta = sta_info_get(sdata, sdata->vif.cfg.ap_addr);
	if (WARN_ON(!sta))
		return ERR_PTR(-ENOENT);

	if (link_id < 0) {
		link_sta = &sta->deflink;
	} else {
		link_sta = wiphy_dereference(local->hw.wiphy,
					     sta->link[link_id]);
		if (WARN_ON(!link_sta))
			return ERR_PTR(-ENOLINK);
	}

	slot.sta = sta;
	slot.link_sta = link_sta;

	if (cigtk)
		slot.key = &link_sta->rx_cigtk[idx];
	else
		slot.key = &link_sta->rx_gtk[idx];

	prev_key = wiphy_dereference(local->hw.wiphy, *slot.key);

	if (!prev_key) {
		if (cigtk) {
			prev_key = wiphy_dereference(local->hw.wiphy,
						     link_sta->rx_cigtk[idx ^ 1]);
		} else if (idx < NUM_DEFAULT_KEYS) {
			for (int i = 0; i < NUM_DEFAULT_KEYS; i++) {
				if (i == idx)
					continue;
				prev_key = wiphy_dereference(local->hw.wiphy,
							     link_sta->rx_gtk[i]);
				if (prev_key)
					break;
			}
		} else {
			/* For IGTK we have 4 and 5 and for BIGTK - 6 and 7 */
			prev_key = wiphy_dereference(local->hw.wiphy,
						     link_sta->rx_gtk[idx ^ 1]);
		}
	}

	if (WARN_ON(!prev_key))
		return ERR_PTR(-EINVAL);

	if (WARN_ON(key_len < prev_key->conf.keylen))
		return ERR_PTR(-EINVAL);

	key = ieee80211_key_alloc(prev_key->conf.cipher, idx,
				  prev_key->conf.keylen, key_data,
				  0, NULL);
	if (IS_ERR(key))
		return ERR_CAST(key);

	if (sdata->u.mgd.mfp != IEEE80211_MFP_DISABLED)
		key->conf.flags |= IEEE80211_KEY_FLAG_RX_MGMT;

	if (prev_key->conf.flags & IEEE80211_KEY_FLAG_CIP)
		key->conf.flags |= IEEE80211_KEY_FLAG_CIP;

	key->conf.link_id = link_data->link_id;

	err = ieee80211_key_link(sdata, &slot, key);
	if (err)
		return ERR_PTR(err);

	return &key->conf;
}
EXPORT_SYMBOL_GPL(ieee80211_gtk_rekey_add);

void ieee80211_key_mic_failure(struct ieee80211_key_conf *keyconf)
{
	struct ieee80211_key *key;

	key = container_of(keyconf, struct ieee80211_key, conf);

	switch (key->conf.cipher) {
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		key->u.aes_cmac.icverrors++;
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		key->u.aes_gmac.icverrors++;
		break;
	default:
		/* ignore the others for now, we don't keep counters now */
		break;
	}
}
EXPORT_SYMBOL_GPL(ieee80211_key_mic_failure);

void ieee80211_key_replay(struct ieee80211_key_conf *keyconf)
{
	struct ieee80211_key *key;

	key = container_of(keyconf, struct ieee80211_key, conf);

	switch (key->conf.cipher) {
	case WLAN_CIPHER_SUITE_CCMP:
	case WLAN_CIPHER_SUITE_CCMP_256:
		key->u.ccmp.replays++;
		break;
	case WLAN_CIPHER_SUITE_AES_CMAC:
	case WLAN_CIPHER_SUITE_BIP_CMAC_256:
		key->u.aes_cmac.replays++;
		break;
	case WLAN_CIPHER_SUITE_BIP_GMAC_128:
	case WLAN_CIPHER_SUITE_BIP_GMAC_256:
		key->u.aes_gmac.replays++;
		break;
	case WLAN_CIPHER_SUITE_GCMP:
	case WLAN_CIPHER_SUITE_GCMP_256:
		key->u.gcmp.replays++;
		break;
	}
}
EXPORT_SYMBOL_GPL(ieee80211_key_replay);

int ieee80211_key_switch_links(struct ieee80211_sub_if_data *sdata,
			       unsigned long del_links_mask,
			       unsigned long add_links_mask)
{
	struct ieee80211_key *key;
	int ret;

	list_for_each_entry(key, &sdata->key_list, list) {
		if (key->conf.link_id < 0 ||
		    !(del_links_mask & BIT(key->conf.link_id)))
			continue;

		ieee80211_key_disable_hw_accel(key);
	}

	list_for_each_entry(key, &sdata->key_list, list) {
		if (key->conf.link_id < 0 ||
		    !(add_links_mask & BIT(key->conf.link_id)))
			continue;

		ret = ieee80211_key_enable_hw_accel(key);
		if (ret)
			return ret;
	}

	return 0;
}
