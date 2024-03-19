/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * NTMP table request and response data buffer formats
 * Copyright 2023 NXP
 * Copyright (C) 2023 Wei Fang <wei.fang@nxp.com>
 */
#ifndef __NTMP_FORMATS_H
#define __NTMP_FORMATS_H
#include <linux/fsl/ntmp.h>

#pragma pack(1)
struct common_req_data {
	__le16 update_act;
	u8 dbg_opt;
	u8 query_act:4;
	u8 tbl_ver:4;
};

/* MAC Address Filter Table Request and Response Data Buffer Format */
struct maft_keye_data {
	u8 mac_addr[ETH_ALEN];
	__le16 resv;
};

struct maft_cfge_data {
	__le16 si_bitmap;
	__le16 resv;
};

/* struct for query or delete operation */
struct maft_req_qd {
	struct common_req_data crd;
	__le32 entry_id;
};

/* struct for add operation */
struct maft_req_add {
	struct common_req_data crd;
	__le32 entry_id;
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

/*struct for response data buffer */
struct maft_resp_query {
	__le32 entry_id;
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

/* VLAM Address Filter table Request and Response Data Buffer Format */
struct vaft_keye_data {
	__le16 vlan_id; /* bit0~11: vlan_id */
	u8 tpid:2;
	u8 resv1:6;
	u8 resv2;
};

struct vaft_cfge_data {
	__le16 si_bitmap;
	__le16 resv;
};

/* VLAN Address Filter Table Add action */
struct vaft_req_add {
	struct common_req_data crd;
	__le32 entry_id;
	struct vaft_keye_data keye;
	struct vaft_cfge_data cfge;
};

/* VLAN Address Filter Table Query or Delete action */
struct vaft_req_qd {
	struct common_req_data crd;
	__le32 entry_id;
};

/* VLAN Address Filter Table Response to Query action */
struct vaft_resp_query {
	__le32 entry_id;
	struct vaft_keye_data keye;
	struct vaft_cfge_data cfge;
};

/* RSS Table Request and Response Data Buffer Format */
struct rsst_req_query {
	struct common_req_data crd;
	__le32 entry_id;
};

/* struct for update operation */
struct rsst_req_update {
	struct common_req_data crd;
	__le32 entry_id;
	u8 groups[];
};

/* Time Gate Scheduling Table Resquet and Response Data Buffer Format */
struct tgst_ge {
	__le32 interval;
	u8 tc_state;
	u8 resv;
	__le16 hr_cb;	/* bit0~3: hr_cbd, bit4~15 reserved */
};

struct tgst_cfge_data {
	__le64 admin_bt;
	__le32 admin_ct;
	__le32 admin_ct_ext;
	__le16 admin_cl_len;
	__le16 resv;
	struct tgst_ge ge[];
};

/* struct for update operation */
struct tgst_req_update {
	struct common_req_data crd;
	__le32 entry_id;
	struct tgst_cfge_data cfge;
};

struct tgst_req_query {
	struct common_req_data crd;
	__le32 entry_id;
};

struct tgst_olse_data {
	__le64 cfg_ct;
	__le64 cfg_ce;
	__le64 oper_bt;
	__le32 oper_ct;
	__le32 oper_ct_ext;
	__le16 oper_cl_len;
	__le16 resv;
	struct tgst_ge ge[];
};

struct tgst_resp_status {
	__le64 cfg_ct;
	__le32 status_resv;
};

struct tgst_resp_query {
	struct tgst_resp_status status;
	__le32 entry_id;
	u8 data[];
};

/* Rate Policer Table Request and Response Data Buffer Format */
struct rpt_cfge_data {
	__le32 cir;
	__le32 cbs;
	__le32 eir;
	__le32 ebs;
	u8 mren:1;
	u8 doy:1;
	u8 cm:1;
	u8 cf:1;
	u8 ndor:1;
	u8 sdu_type:2;
	u8 resv0:1;
	u8 resv1;
};

struct rpt_fee_data {
	u8 fen:1;
	u8 resv:7;
};

struct rpt_stse_data {
	__le64 byte_count;
	__le32 drop_frames;
	__le32 rev0;
	__le32 dr0_grn_frames;
	__le32 rev1;
	__le32 dr1_grn_frames;
	__le32 rev2;
	__le32 dr2_ylw_frames;
	__le32 rev3;
	__le32 remark_ylw_frames;
	__le32 rev4;
	__le32 dr3_red_frames;
	__le32 rev5;
	__le32 remark_red_frames;
	__le32 rev6;
	__le32 lts;
	__le32 bci;
	__le32 bcf_bcs;	/* bit0~30: bcf, bit31: bcs */
	__le32 bei;
	__le32 bef_bes; /* bit0~30: bef, bit31: bes */
};

struct rpt_pse_data {
	u8 mr:1;
	u8 rev:7;
};

struct rpt_req_ua {
	struct common_req_data crd;
	__le32 entry_id;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
};

struct rpt_req_nua {
	struct common_req_data crd;
	__le32 entry_id;
};

struct rpt_resp_query {
	__le32 entry_id;
	struct rpt_stse_data stse;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
	struct rpt_pse_data pse;
};

/* Ingress Stream Identification Table Resquet and Response Data Buffer Format */

/* Notice that current frame key only support NULL stream ID and
 * SMAC and VLAN stream ID, same with LS1028A.
 */
struct frame_key {
	u8 mac[ETH_ALEN];
	u8 vlan_h;	// Most significant byte of the 2 bytes
	u8 vlan_l;	// Least significant byte of the 2 bytes
	__le32 resv[2];
};

struct isit_access_key {
	union {
		__le32 entry_id;
		__le32 resume_eid;
		__le32 key_type; /* bit0~1: key type, other bits: reserved */
	};
	union {
		__le32 resv[4];
		struct frame_key fk;
	};
};

struct isit_key_data {
	__le32 key_type; /* bit0~1: key type, other bits: reserved */
	u8 frame_key[NTMP_ISIT_FRAME_KEY_LEN];
};

/* struct for update or add operation*/
struct isit_req_ua {
	struct common_req_data crd;
	struct isit_access_key ak;
	__le32 is_eid;
};

/* struct for not update or add operation, such as delete, query */
struct isit_req_nua {
	struct common_req_data crd;
	struct isit_access_key ak;
};

struct isit_resp_query {
	__le32 status;
	__le32 entry_id;
	struct isit_key_data key;
	__le32 is_eid;
};

struct isit_resp_nq {
	__le32 status;
};

/* Ingress Stream Table version 0 Resquet and Response Data Buffer Format */
struct ist_cfge_data {
	u8 sfe:1;
	u8 resv0:3;
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	u8 resv1:4;
	u8 resv2:2;
	u8 orp:1;
	u8 osgi:1;
	u8 resv3:4;
	u8 fa:3;
	u8 sdu_type:2;
	u8 resv4:3;
	__le16 msdu;
	__le16 resv5[3];
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 resv6[2];
	__le32 isc_eid;
	__le32 resv7;
	__le16 si_bitmap;
};

/* struct for update or add operation*/
struct ist_req_ua {
	struct common_req_data crd;
	__le32 entry_id;
	struct ist_cfge_data cfge;
};

struct ist_req_nua {
	struct common_req_data crd;
	__le32 entry_id;
};

struct ist_resp_query {
	__le32 entry_id;
	struct ist_cfge_data cfge;
};

/* Ingress Stream filter Table Resquet and Response Data Buffer Format */
struct isft_access_key {
	union {
		__le32 entry_id;
		__le32 resume_eid;
		__le32 is_eid;
	};
	u8 pcp:3;	/* reserved for entry id and resume_eid */
	u8 resv0:5;
	u8 resv1[3];
};

struct isft_cfge_data {
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	u8 resv0:2;
	u8 osgi:1;
	u8 resv1:1;
	u8 orp:1;
	u8 sdu_type:2;
	u8 resv2:1;
	__le16 msdu;
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 isc_eid;
};

struct isft_keye_data {
	__le32 is_eid;
	u8 pcp:3;
	u8 resv0:5;
	u8 resv1[3];
};

struct isft_req_ua {
	struct common_req_data crd;
	struct isft_access_key ak;
	struct isft_cfge_data cfge;
};

struct isft_req_nua {
	struct common_req_data crd;
	struct isft_access_key ak;
};

struct isft_resp_query {
	__le32 status;
	__le32 entry_id;
	struct isft_keye_data keye;
	struct isft_cfge_data cfge;
};

struct isft_resp_nq {
	__le32 status;
};

/* Stream Gate Instance Table Resquet and Response Data Buffer Format */
struct sgit_acfge_data {
	__le32 admin_sgcl_eid;
	__le64 admin_bt;
	__le32 admin_ct_ext;
};

struct sgit_cfge_data {
	u8 oexen:1;
	u8 irxen:1;
	u8 sdu_type:2;
	u8 resv:4;
};

struct sgit_icfge_data {
	u8 ipv:4;
	u8 oipv:1;
	u8 gst:1;
	u8 resv:2;
};

struct sgit_sgise_data {
	__le32 oper_sgcl_eid;
	__le64 cfg_ct;
	__le64 oper_bt;
	__le32 oper_ct_ext;
	u8 oex:1;
	u8 irx:1;
	u8 state:3;
	u8 resv:3;
};

struct sgit_req_ua {
	struct common_req_data crd;
	__le32 entry_id;
	struct sgit_acfge_data acfge;
	struct sgit_cfge_data cfge;
	struct sgit_icfge_data icfge;
};

struct sgit_req_nua {
	struct common_req_data crd;
	__le32 entry_id;
};

struct sgit_resp_query {
	__le32 entry_id;
	struct sgit_sgise_data sgise;
	struct sgit_cfge_data cfge;
	struct sgit_icfge_data icfge;
	u8 resv;
	struct sgit_acfge_data acfge;
};

/* Stream Gate Control List Table Request and Response Data Buffer Format */
struct sgclt_ge {
	__le32 interval;
	u8 iom[3];
	u8 ipv:4;
	u8 oipv:1;
	u8 resv:1;
	u8 iomen:1;
	u8 gtst:1;
};

struct sgclt_cfge_data {
	__le32 ct;
	u8 list_len;
	u8 resv1;
	u8 ext_oipv:1;
	u8 ext_ipv:4;
	u8 resv2:1;
	u8 ext_gtst:1;
	u8 resv3:1;
	u8 resv4;
	struct sgclt_ge ge[];
};

struct sgclt_req_add {
	struct common_req_data crd;
	__le32 entry_id;
	struct sgclt_cfge_data cfge;
};

/* struct for delete or query operation */
struct sgclt_req_qd {
	struct common_req_data crd;
	__le32 entry_id;
};

struct sgclt_resp_query {
	__le32 entry_id;
	u8 ref_count;
	u8 resv[3];
	struct sgclt_cfge_data cfge;
};

/* Ingress Stream Count Table Request and Response Data Buffer Format */
struct isct_req_data {
	struct common_req_data crd;
	__le32 entry_id;
};

struct isct_resp_query {
	__le32 entry_id;
	__le32 rx_count;
	__le32 resv1;
	__le32 msdu_drop_count;
	__le32 resv2;
	__le32 policer_drop_count;
	__le32 resv3;
	__le32 sg_drop_count;
	__le32 resv4;
};

/* Ingress Port Filter Table Request and Response Data Buffer Format */
struct ipft_pld_byte {
	u8 data;
	u8 mask;
};

struct ipft_keye_data {
	__le16 precedence;
	__le16 resv0[3];
	__le16 frm_attr_flags;
	__le16 frm_attr_flags_mask;
	__le16 dscp; /* bit0~5: dscp, bit6~11: mask, bit12~15: reserved */
	__le16 src_port; /* bit0~4: src_port, bit5~9: mask, bit10~15: reserved */
	__be16 outer_vlan_tci;
	__be16 outer_vlan_tci_mask;
	u8 dmac[ETH_ALEN];
	u8 dmac_mask[ETH_ALEN];
	u8 smac[ETH_ALEN];
	u8 smac_mask[ETH_ALEN];
	__be16 inner_vlan_tci;
	__be16 inner_vlan_tci_mask;
	__be16 ethertype;
	__be16 ethertype_mask;
	u8 ip_protocol;
	u8 ip_protocol_mask;
	__le16 resv1[7];
	__be32 ip_src[4];
	__le32 resv2[2];
	__be32 ip_src_mask[4];
	__be16 l4_src_port;
	__be16 l4_src_port_mask;
	__le32 resv3;
	__be32 ip_dst[4];
	__le32 resv4[2];
	__be32 ip_dst_mask[4];
	__be16 l4_dst_port;
	__be16 l4_dst_port_mask;
	__le32 resv5;
	struct ipft_pld_byte byte[NTMP_IPFT_MAX_PLD_LEN];
};

struct ipft_cfge_data {
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	__le16 filter; /* bit0~2: fltfa, bit4: wolte, bit5~6: flta, bit7~8: rpr */
	u8 resv;
	__le32 flta_tgt;
};

struct ipft_req_add {
	struct common_req_data crd;
	struct ipft_keye_data keye;
	struct ipft_cfge_data cfge;
};

/* request data format of query or delete action */
struct ipft_req_qd {
	struct common_req_data crd;
	__le32 entry_id;
	__le32 resv[52];
};

struct ipft_stse_data {
	__le64 match_count;
};

struct ipft_resp_query {
	__le32 status;
	__le32 entry_id;
	struct ipft_keye_data keye;
	struct ipft_stse_data stse;
	struct ipft_cfge_data cfge;
};

/* response data format of non-query action */
struct ipft_resp_nq {
	__le32 status;
};

#pragma pack()

#endif
