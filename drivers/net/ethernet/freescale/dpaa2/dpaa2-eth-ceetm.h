/* Copyright 2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *	 notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *	 notice, this list of conditions and the following disclaimer in the
 *	 documentation and/or other materials provided with the distribution.
 *     * Neither the name of Freescale Semiconductor nor the
 *	 names of its contributors may be used to endorse or promote products
 *	 derived from this software without specific prior written permission.
 *
 *
 * ALTERNATIVELY, this software may be distributed under the terms of the
 * GNU General Public License ("GPL") as published by the Free Software
 * Foundation, either version 2 of that License or (at your option) any
 * later version.
 *
 * THIS SOFTWARE IS PROVIDED BY Freescale Semiconductor ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL Freescale Semiconductor BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __DPAA2_ETH_CEETM_H
#define __DPAA2_ETH_CEETM_H

#include <net/pkt_sched.h>
#include <net/pkt_cls.h>
#include <net/netlink.h>

#include "dpaa2-eth.h"

/* For functional purposes, there are num_tx_queues pfifo qdiscs through which
 * frames reach the driver. Their handles start from 1:21. Handles 1:1 to 1:20
 * are reserved for the maximum 32 CEETM channels (majors and minors are in
 * hex).
 */
#define PFIFO_MIN_OFFSET		0x21

#define DPAA2_CEETM_MIN_WEIGHT		100
#define DPAA2_CEETM_MAX_WEIGHT		24800

#define DPAA2_CEETM_TD_THRESHOLD	1000

enum wbfs_group_type {
	WBFS_GRP_A,
	WBFS_GRP_B,
	WBFS_GRP_LARGE
};

enum {
	DPAA2_CEETM_TCA_UNSPEC,
	DPAA2_CEETM_TCA_COPT,
	DPAA2_CEETM_TCA_QOPS,
	DPAA2_CEETM_TCA_MAX,
};

/* CEETM configuration types */
enum dpaa2_ceetm_type {
	CEETM_ROOT = 1,
	CEETM_PRIO,
};

enum {
	STRICT_PRIORITY = 0,
	WEIGHTED_A,
	WEIGHTED_B,
};

struct dpaa2_ceetm_shaping_cfg {
	__u64 cir; /* committed information rate */
	__u64 eir; /* excess information rate */
	__u16 cbs; /* committed burst size */
	__u16 ebs; /* excess burst size */
	__u8 coupled; /* shaper coupling */
};

extern const struct nla_policy ceetm_policy[DPAA2_CEETM_TCA_MAX];

struct dpaa2_ceetm_class;
struct dpaa2_ceetm_qdisc_stats;
struct dpaa2_ceetm_class_stats;

/* corresponds to CEETM shaping at LNI level */
struct dpaa2_root_q {
	struct Qdisc **qdiscs;
	struct dpaa2_ceetm_qdisc_stats __percpu *qstats;
};

/* corresponds to the number of priorities a channel serves */
struct dpaa2_prio_q {
	struct dpaa2_ceetm_class *parent;
	struct dpni_tx_priorities_cfg tx_prio_cfg;
};

struct dpaa2_ceetm_qdisc {
	struct Qdisc_class_hash clhash;
	struct tcf_proto *filter_list; /* qdisc attached filters */
	struct tcf_block *block;

	enum dpaa2_ceetm_type type; /* ROOT/PRIO */
	bool shaped;
	union {
		struct dpaa2_root_q root;
		struct dpaa2_prio_q prio;
	};
};

/* CEETM Qdisc configuration parameters */
struct dpaa2_ceetm_tc_qopt {
	enum dpaa2_ceetm_type type;
	__u16 shaped;
	__u8 prio_group_A;
	__u8 prio_group_B;
	__u8 separate_groups;
};

/* root class - corresponds to a channel */
struct dpaa2_root_c {
	struct dpaa2_ceetm_shaping_cfg shaping_cfg;
	u32 ch_id;
};

/* prio class - corresponds to a strict priority queue (group) */
struct dpaa2_prio_c {
	struct dpaa2_ceetm_class_stats __percpu *cstats;
	u32 qpri;
	u8 mode;
	u16 weight;
};

struct dpaa2_ceetm_class {
	struct Qdisc_class_common common;
	struct tcf_proto *filter_list; /* class attached filters */
	struct tcf_block *block;
	struct Qdisc *parent;
	struct Qdisc *child;

	enum dpaa2_ceetm_type type; /* ROOT/PRIO */
	bool shaped;
	union {
		struct dpaa2_root_c root;
		struct dpaa2_prio_c prio;
	};
};

/* CEETM Class configuration parameters */
struct dpaa2_ceetm_tc_copt {
	enum dpaa2_ceetm_type type;
	struct dpaa2_ceetm_shaping_cfg shaping_cfg;
	__u16 shaped;
	__u8 mode;
	__u16 weight;
};

/* CEETM stats */
struct dpaa2_ceetm_qdisc_stats {
	__u32 drops;
};

struct dpaa2_ceetm_class_stats {
	/* Software counters */
	struct gnet_stats_basic_sync bstats;
	__u32 ern_drop_count;
};

struct dpaa2_ceetm_tc_xstats {
	__u64 ceetm_dequeue_bytes;
	__u64 ceetm_dequeue_frames;
	__u64 ceetm_reject_bytes;
	__u64 ceetm_reject_frames;
};

#ifdef CONFIG_FSL_DPAA2_ETH_CEETM
int __init dpaa2_ceetm_register(void);
void __exit dpaa2_ceetm_unregister(void);
int dpaa2_ceetm_classify(struct sk_buff *skb, struct Qdisc *sch,
			 int *qdid, u8 *qpri);
#else
static inline int dpaa2_ceetm_register(void)
{
	return 0;
}

static inline void dpaa2_ceetm_unregister(void) {}

static inline int dpaa2_ceetm_classify(struct sk_buff *skb, struct Qdisc *sch,
				       int *qdid, u8 *qpri)
{
	return 0;
}
#endif

static inline bool dpaa2_eth_ceetm_is_enabled(struct dpaa2_eth_priv *priv)
{
	return priv->ceetm_en;
}

#endif
