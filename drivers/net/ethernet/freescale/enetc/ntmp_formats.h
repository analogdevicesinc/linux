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

struct common_resp_query {
	__le32 entry_id;
};

struct common_resp_nq {
	__le32 status;
};

/* Generic structure for 'delete' or 'query' by entry ID  */
struct ntmp_qd_by_eid {
	struct common_req_data crd;
	__le32 entry_id;
};

/* MAC Address Filter Table Request and Response Data Buffer Format */
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
struct vaft_req_add {
	struct common_req_data crd;
	__le32 entry_id;
	struct vaft_keye_data keye;
	struct vaft_cfge_data cfge;
};

/* VLAN Address Filter Table Response to Query action */
struct vaft_resp_query {
	__le32 entry_id;
	struct vaft_keye_data keye;
	struct vaft_cfge_data cfge;
};

/* RSS Table Request and Response Data Buffer Format */
struct rsst_req_update {
	struct common_req_data crd;
	__le32 entry_id;
	u8 groups[];
};

/* Time Gate Scheduling Table Resquet and Response Data Buffer Format */
struct tgst_req_update {
	struct common_req_data crd;
	__le32 entry_id;
	struct tgst_cfge_data cfge;
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
struct rpt_req_ua {
	struct common_req_data crd;
	__le32 entry_id;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
};

struct rpt_resp_query {
	__le32 entry_id;
	struct rpt_stse_data stse;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
	struct rpt_pse_data pse;
};

/* Ingress Stream Identification Table Resquet and Response Data Buffer Format */
struct isit_ak_eid {
	__le32 entry_id;
	__le32 resv[4];
};

struct isit_ak_search {
	__le32 resume_eid;
	__le32 resv[4];
};

union isit_access_key {
	struct isit_ak_eid eid;
	struct isit_keye_data keye;
	struct isit_ak_search search;
};

/* struct for update or add operation*/
struct isit_req_ua {
	struct common_req_data crd;
	union isit_access_key ak;
	__le32 is_eid;
};

/* struct for not update or add operation, such as delete, query */
struct isit_req_qd {
	struct common_req_data crd;
	union isit_access_key ak;
};

struct isit_resp_query {
	__le32 status;
	__le32 entry_id;
	struct isit_keye_data keye;
	__le32 is_eid;
};

/* Ingress Stream Table version 0 Resquet and Response Data Buffer Format */
struct ist_req_ua {
	struct common_req_data crd;
	__le32 entry_id;
	struct ist_cfge_data cfge;
};

struct ist_resp_query {
	__le32 entry_id;
	struct ist_cfge_data cfge;
};

/* Ingress Stream filter Table Resquet and Response Data Buffer Format */
struct isft_ak_eid {
	__le32 entry_id;
	__le32 resv;
};

struct isft_ak_search {
	__le32 resume_eid;
	__le32 resv;
};

union isft_access_key {
	struct isft_ak_eid eid;
	struct isft_keye_data keye;
	struct isft_ak_search search;
};

struct isft_req_ua {
	struct common_req_data crd;
	union isft_access_key ak;
	struct isft_cfge_data cfge;
};

struct isft_req_qd {
	struct common_req_data crd;
	union isft_access_key ak;
};

struct isft_resp_query {
	__le32 status;
	__le32 entry_id;
	struct isft_keye_data keye;
	struct isft_cfge_data cfge;
};

/* Stream Gate Instance Table Resquet and Response Data Buffer Format */
struct sgit_req_ua {
	struct common_req_data crd;
	__le32 entry_id;
	struct sgit_acfge_data acfge;
	struct sgit_cfge_data cfge;
	struct sgit_icfge_data icfge;
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
struct sgclt_req_add {
	struct common_req_data crd;
	__le32 entry_id;
	struct sgclt_cfge_data cfge;
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
	struct isct_stse_data stse;
};

/* Ingress Port Filter Table Request and Response Data Buffer Format */
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

struct ipft_resp_query {
	__le32 status;
	__le32 entry_id;
	struct ipft_keye_data keye;
	__le64 match_count; /* STSE_DATA */
	struct ipft_cfge_data cfge;
};

#pragma pack()

#endif
