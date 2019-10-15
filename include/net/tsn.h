/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2017-2023 NXP */

#ifndef __TSN_H__
#define __TSN_H__

#include <linux/notifier.h>
#include <uapi/linux/tsn.h>

enum tsn_notifier_type {
	TSN_QBV_CONFIGCHANGETIME_ARRIVE = 1,
};

struct tsn_notifier_info {
	struct net_device *dev;
	union {
		struct tsn_qbv_conf qbv_notify;
		struct tsn_qci_psfp_sgi_conf qci_notify;
	} ntdata;
};

static inline struct net_device *
tsn_notifier_info_to_dev(const struct tsn_notifier_info *info)
{
	return info->dev;
}

struct tsn_ops {
	void (*device_init)(struct net_device *ndev);
	void (*device_deinit)(struct net_device *ndev);
	u32 (*get_capability)(struct net_device *ndev);
	/* Qbv standard */
	int (*qbv_set)(struct net_device *ndev, struct tsn_qbv_conf *qbvconf);
	int (*qbv_get)(struct net_device *ndev, struct tsn_qbv_conf *qbvconf);
	int (*qbv_get_status)(struct net_device *ndev,
				struct tsn_qbv_status *qbvstat);
	int (*cb_streamid_set)(struct net_device *ndev, u32 index,
				bool enable, struct tsn_cb_streamid *sid);
	int (*cb_streamid_get)(struct net_device *ndev, u32 index,
				struct tsn_cb_streamid *sid);
	int (*cb_streamid_counters_get)(struct net_device *ndev, u32 index,
				struct tsn_cb_streamid_counters *sidcounter);
	int (*qci_get_maxcap)(struct net_device *ndev,
				struct tsn_qci_psfp_stream_param *qcicapa);
	int (*qci_sfi_set)(struct net_device *ndev, u32 index, bool enable,
				struct tsn_qci_psfp_sfi_conf *sficonf);
	/* return: 0 stream filter instance not valid
	 * 1 stream filter instance valid
	 * -1 error happened
	 */
	int (*qci_sfi_get)(struct net_device *ndev, u32 index,
			   struct tsn_qci_psfp_sfi_conf *sficonf);
	int (*qci_sfi_counters_get)(struct net_device *ndev, u32 index,
				    struct tsn_qci_psfp_sfi_counters *sficounter);
	int (*qci_sgi_set)(struct net_device *ndev, u32 index,
			   struct tsn_qci_psfp_sgi_conf *sgiconf);
	int (*qci_sgi_get)(struct net_device *ndev, u32 index,
			   struct tsn_qci_psfp_sgi_conf *sgiconf);
	int (*qci_sgi_status_get)(struct net_device *ndev, u16 index,
				  struct tsn_psfp_sgi_status *sgistat);
	int (*qci_fmi_set)(struct net_device *ndev, u32 index, bool enable,
			   struct tsn_qci_psfp_fmi *fmi);
	int (*qci_fmi_get)(struct net_device *ndev, u32 index,
			   struct tsn_qci_psfp_fmi *fmi,
			   struct tsn_qci_psfp_fmi_counters *counters);
	int (*cbs_set)(struct net_device *ndev, u8 tc, u8 bw);
	int (*cbs_get)(struct net_device *ndev, u8 tc);
	/* To set a 8 bits vector shows 8 traffic classes
	 * preemtable(1) or express(0)
	 */
	int (*qbu_set)(struct net_device *ndev, u8 ptvector);
	/* To get port preemtion status */
	int (*qbu_get)(struct net_device *ndev,
		       struct tsn_preempt_status *preemptstat);
	int (*tsd_set)(struct net_device *ndev, struct tsn_tsd *tsd);
	int (*tsd_get)(struct net_device *ndev, struct tsn_tsd_status *stats);
	int (*ct_set)(struct net_device *ndev, u8 cut_thru);
	int (*cbgen_set)(struct net_device *ndev, u32 index,
			 struct tsn_seq_gen_conf *seqgen);
	int (*cbrec_set)(struct net_device *ndev, u32 index,
			 struct tsn_seq_rec_conf *seqrec);
	int (*cb_get)(struct net_device *ndev, u32 index,
		      struct tsn_cb_status *c);
	int (*dscp_set)(struct net_device *ndev, bool enable, const u8 dscp_ix,
			struct tsn_qos_switch_dscp_conf *c);
	int (*pcpmap_set)(struct net_device *ndev,
			  struct tsn_qos_switch_pcp_conf *c);
};

enum ethdev_type {
	TSN_SWITCH,
	TSN_ENDPOINT,
};

#define GROUP_OFFSET_SWITCH 256

struct tsn_port {
	u16 groupid;
	const struct tsn_ops *tsnops;
	struct net_device *netdev;
	struct list_head list;
	enum ethdev_type type;
	u8 tc_nums;
	struct tsn_notifier_info nd;
};

struct tsn_port *tsn_get_port(struct net_device *ndev);
int register_tsn_notifier(struct notifier_block *nb);
int unregister_tsn_notifier(struct notifier_block *nb);
int call_tsn_notifiers(unsigned long val, struct net_device *dev,
			     struct tsn_notifier_info *info);
int tsn_port_register(struct net_device *netdev, const struct tsn_ops *tsnops,
		      u16 groupid);
void tsn_port_unregister(struct net_device *netdev);
int tsn_qbv_tc_taprio_compat_set(struct net_device *netdev,
				 struct tsn_qbv_conf *shaper_config,
				 bool previously_enabled);
int tsn_qbu_ethtool_mm_compat_get(struct net_device *netdev,
				  struct tsn_preempt_status *preemptstat);
int tsn_qbu_ethtool_mm_compat_set(struct net_device *netdev,
				  u8 preemptible_tcs);
#endif
