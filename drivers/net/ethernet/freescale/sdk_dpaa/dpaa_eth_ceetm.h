/* Copyright 2008-2016 Freescale Semiconductor Inc.
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

#ifndef __DPAA_ETH_CEETM_H
#define __DPAA_ETH_CEETM_H

#include <net/pkt_sched.h>
#include <net/pkt_cls.h>
#include <net/netlink.h>
#include <lnxwrp_fm.h>

#include "mac.h"
#include "dpaa_eth_common.h"

/* Mask to determine the sub-portal id from a channel number */
#define CHANNEL_SP_MASK		0x1f
/* The number of the last channel that services DCP0, connected to FMan 0.
 * Value validated for B4 and T series platforms.
 */
#define DCP0_MAX_CHANNEL	0x80f
/* A2V=1 - field A2 is valid
 * A0V=1 - field A0 is valid - enables frame confirmation
 * OVOM=1 - override operation mode bits with values from A2
 * EBD=1 - external buffers are deallocated at the end of the FMan flow
 * NL=0 - the BMI releases all the internal buffers
 */
#define CEETM_CONTEXT_A		0x1a00000080000000
/* The ratio between the superior and inferior congestion state thresholds. The
 * lower threshold is set to 7/8 of the superior one (as the default for WQ
 * scheduling).
 */
#define CEETM_CCGR_RATIO	0.875
/* For functional purposes, there are num_tx_queues pfifo qdiscs through which
 * frames reach the driver. Their handles start from 1:21. Handles 1:1 to 1:20
 * are reserved for the maximum 32 CEETM channels (majors and minors are in
 * hex).
 */
#define PFIFO_MIN_OFFSET	0x21

/* A maximum of 8 CQs can be linked to a CQ channel or to a WBFS scheduler. */
#define CEETM_MAX_PRIO_QCOUNT	8
#define CEETM_MAX_WBFS_QCOUNT	8
#define CEETM_MIN_WBFS_QCOUNT	4

/* The id offsets of the CQs belonging to WBFS groups (ids 8-11/15 for group A
 * and/or 12-15 for group B).
 */
#define WBFS_GRP_A_OFFSET	8
#define WBFS_GRP_B_OFFSET	12

#define WBFS_GRP_A	1
#define WBFS_GRP_B	2
#define WBFS_GRP_LARGE	3

enum {
	TCA_CEETM_UNSPEC,
	TCA_CEETM_COPT,
	TCA_CEETM_QOPS,
	__TCA_CEETM_MAX,
};

/* CEETM configuration types */
enum {
	CEETM_ROOT = 1,
	CEETM_PRIO,
	CEETM_WBFS
};

#define TCA_CEETM_MAX (__TCA_CEETM_MAX - 1)
extern const struct nla_policy ceetm_policy[TCA_CEETM_MAX + 1];

struct ceetm_class;
struct ceetm_qdisc_stats;
struct ceetm_class_stats;

struct ceetm_fq {
	struct qman_fq fq;
	struct net_device *net_dev;
	struct ceetm_class *ceetm_cls;
	int congested; /* Congestion status */
};

struct root_q {
	struct Qdisc **qdiscs;
	__u16 overhead;
	__u32 rate;
	__u32 ceil;
	struct qm_ceetm_sp *sp;
	struct qm_ceetm_lni *lni;
	struct ceetm_qdisc_stats __percpu *qstats;
};

struct prio_q {
	__u16 qcount;
	struct ceetm_class *parent;
	struct qm_ceetm_channel *ch;
};

struct wbfs_q {
	__u16 qcount;
	int group_type;
	struct ceetm_class *parent;
	struct qm_ceetm_channel *ch;
	__u16 cr;
	__u16 er;
};

struct ceetm_qdisc {
	int type; /* LNI/CHNL/WBFS */
	bool shaped;
	union {
		struct root_q root;
		struct prio_q prio;
		struct wbfs_q wbfs;
	};
	struct Qdisc_class_hash clhash;
	struct tcf_proto *filter_list; /* qdisc attached filters */
	struct tcf_block *block;
};

/* CEETM Qdisc configuration parameters */
struct tc_ceetm_qopt {
	__u32 type;
	__u16 shaped;
	__u16 qcount;
	__u16 overhead;
	__u32 rate;
	__u32 ceil;
	__u16 cr;
	__u16 er;
	__u8 qweight[CEETM_MAX_WBFS_QCOUNT];
};

struct root_c {
	unsigned int rate;
	unsigned int ceil;
	unsigned int tbl;
	bool wbfs_grp_a;
	bool wbfs_grp_b;
	bool wbfs_grp_large;
	struct Qdisc *child;
};

struct prio_c {
	bool cr;
	bool er;
	struct ceetm_fq *fq; /* Hardware FQ instance Handle */
	struct qm_ceetm_lfq *lfq;
	struct qm_ceetm_cq *cq; /* Hardware Class Queue instance Handle */
	struct qm_ceetm_ccg *ccg;
	/* only one wbfs can be linked to one priority CQ */
	struct Qdisc *child;
	struct ceetm_class_stats __percpu *cstats;
};

struct wbfs_c {
	__u8 weight; /* The weight of the class between 1 and 248 */
	struct ceetm_fq *fq; /* Hardware FQ instance Handle */
	struct qm_ceetm_lfq *lfq;
	struct qm_ceetm_cq *cq; /* Hardware Class Queue instance Handle */
	struct qm_ceetm_ccg *ccg;
	struct ceetm_class_stats __percpu *cstats;
};

struct ceetm_class {
	struct Qdisc_class_common common;
	struct tcf_proto *filter_list; /* class attached filters */
	struct tcf_block *block;
	struct Qdisc *parent;
	struct qm_ceetm_channel *ch;
	bool shaped;
	int type; /* ROOT/PRIO/WBFS */
	union {
		struct root_c root;
		struct prio_c prio;
		struct wbfs_c wbfs;
	};
};

/* CEETM Class configuration parameters */
struct tc_ceetm_copt {
	__u32 type;
	__u16 shaped;
	__u32 rate;
	__u32 ceil;
	__u16 tbl;
	__u16 cr;
	__u16 er;
	__u8 weight;
};

/* CEETM stats */
struct ceetm_qdisc_stats {
	__u32 drops;
};

struct ceetm_class_stats {
	/* Software counters */
	struct gnet_stats_basic_sync bstats;
	__u32 ern_drop_count;
	__u32 congested_count;
};

struct tc_ceetm_xstats {
	__u32 ern_drop_count;
	__u32 congested_count;
	/* Hardware counters */
	__u64 frame_count;
	__u64 byte_count;
};

int __hot ceetm_tx(struct sk_buff *skb, struct net_device *net_dev);
#endif
