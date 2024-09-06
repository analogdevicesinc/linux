/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2022 NXP */
#ifndef __NETC_NTMP_H
#define __NETC_NTMP_H

#include <linux/bitops.h>
#include <net/tc_act/tc_gate.h>

/* define NTMP Operation Commands */
#define NTMP_CMD_DELETE			BIT(0)
#define NTMP_CMD_UPDATE			BIT(1)
#define NTMP_CMD_QUERY			BIT(2)
#define NTMP_CMD_ADD			BIT(3)
#define NTMP_CMD_QD			(NTMP_CMD_QUERY | NTMP_CMD_DELETE)
#define NTMP_CMD_QU			(NTMP_CMD_QUERY | NTMP_CMD_UPDATE)
#define NTMP_CMD_AU			(NTMP_CMD_ADD | NTMP_CMD_UPDATE)
#define NTMP_CMD_AQ			(NTMP_CMD_ADD | NTMP_CMD_QUERY)
#define NTMP_CMD_AQU			(NTMP_CMD_AQ | NTMP_CMD_UPDATE)

#define NTMP_NULL_ENTRY_ID		0xffffffffU

#define NTMP_TGST_MAX_ENTRY_NUM		64
#define NTMP_SGCLT_MAX_GE_NUM		256

#define NTMP_SGIT_MAX_CT_PLUS_CT_EXT	0x3fffffffU

#define NTMP_ISIT_KEY_TYPE0_SMAC_VLAN	0
#define NTMP_ISIT_KEY_TYPE1_DMAC_VLAN	1
#define NTMP_ISIT_FRAME_KEY_LEN		16

#define NTMP_IST_FA_DISCARD		0
#define NTMP_IST_FA_NO_SI_BITMAP	1
#define NTMP_IST_FA_SI_BITMAP		2

#define NTMP_IPFT_MAX_PLD_LEN		24

/* Ingress stream table forwarding actions of Switch */
#define NTMP_IST_SWITCH_FA_DISCARD	0
#define NTMP_IST_SWITCH_FA_REDIRECT	1
#define NTMP_IST_SWITCH_FA_SF		2
#define NTMP_IST_SWITCH_FA_BF		3
#define NTMP_IST_SWITCH_FA_SF_COPY	4
#define NTMP_IST_SWITCH_FA_BF_COPY	5

/* Ingress port filter table frame attribute flags */
#define NTMP_IPFT_FAF_OVLAN		BIT(2)
#define NTMP_IPFT_FAF_IVLAN		BIT(3)
#define NTMP_IPFT_FAF_STC		GENMASK(6, 4)
#define  NTMP_IPFT_FAF_STC_NTAG		0
#define  NTMP_IPFT_FAF_STC_RTAG2_0	BIT(4)
#define  NTMP_IPFT_FAF_STC_RTAG		BIT(5)
#define  NTMP_IPFT_FAF_STC_HSR		GENMASK(5, 4)
#define NTMP_IPFT_FAF_IP_HDR		BIT(7)
#define NTMP_IPFT_FAF_IP_VER6		BIT(8)
#define NTMP_IPFT_FAF_IP_OPT		BIT(9)
#define NTMP_IPFT_FAF_L4_CODE		GENMASK(11, 10)
#define  NTMP_IPFT_FAF_TCP_HDR		BIT(10)
#define  NTMP_IPFT_FAF_UDP_HDR		BIT(11)
#define  NTMP_IPFT_FAF_SCTP_HDR		GENMASK(11, 10)
#define NTMP_IPFT_FAF_WOL_MAGIC		BIT(12)
#define NTMP_IPFT_FAF_IP_AH		BIT(13)

#define NETC_CBDR_TIMEOUT		1000 /* us */
#define NETC_CBDR_BD_NUM		256
#define NETC_CBDR_BASE_ADDR_ALIGN	128
#define NETC_CBD_DATA_ADDR_ALIGN	16
#define NETC_CBDRMR_EN			BIT(31)

#define NTMP_RESP_HDR_ERR		GENMASK(11, 0)
#define NTMP_TGST_HR_CB_GE		GENMASK(3, 0)
#define NTMP_ISIT_KEY_TYPE		GENMASK(1, 0)
#define NTMP_IPFT_DSCP			GENMASK(5, 0)
#define NTMP_IPFT_DSCP_MASK		GENMASK(11, 6)
#define NTMP_IPFT_SRC_PORT		GENMASK(4, 0)
#define NTMP_IPFT_SRC_PORT_MASK		GENMASK(9, 6)
#define NTMP_IPFT_FLTFA_DISCARD		0
#define NTMP_IPFT_FLTFA_PERMIT		1
#define NTMP_IPFT_FLTFA_REDIRECT	2 /* Switch only */
#define NTMP_IPFT_FLTFA_COPY		3 /* Switch only */
#define NTMP_IPFT_FLTA_NO_ACTION	0
#define NTMP_IPFT_FLTA_RATE_POLICE	1
#define NTMP_IPFT_FLTA_IS		2
#define NTMP_IPFT_FLTA_SI_BITMAP	3

#define NTMP_STREAM_GATE_STATE_CLOSE	0
#define NTMP_STREAM_GATE_STATE_OPEN	1

/* NTMP errata */
#define NTMP_ERR052134			BIT(0)

#pragma pack(1)

/* The format of conctrol buffer descriptor */
union netc_cbd {
	struct {
		__le64 addr;
		__le32 len;
		u8 cmd;
		u8 resv1:4;
		u8 access_method:4;
		u8 table_id;
		u8 hdr_ver:6;
		u8 cci:1;
		u8 rr:1;
		__le32 resv2[3];
		__le32 npf;
	} ntmp_req_hdr;	/* NTMP Request Message Header Format */

	struct {
		__le32 resv1[3];
		__le16 num_matched;
		__le16 error_rr; /* bit0~11: error, bit12~14: reserved, bit15: rr */
		__le32 resv3[4];
	} ntmp_resp_hdr; /* NTMP Response Message Header Format */
};

struct maft_keye_data {
	u8 mac_addr[ETH_ALEN];
	__le16 resv;
};

struct maft_cfge_data {
	__le16 si_bitmap;
	__le16 resv;
};

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

struct tgst_ge {
	__le32 interval;
	u8 tc_state;
	u8 resv0;
	u8 hr_cb:4;
	u8 resv1:4;
	u8 resv2;
};

struct tgst_cfge_data {
	__le64 admin_bt;
	__le32 admin_ct;
	__le32 admin_ct_ext;
	__le16 admin_cl_len;
	__le16 resv;
	struct tgst_ge ge[];
};

struct tgst_olse_data {
	__le64 oper_cfg_ct;
	__le64 oper_cfg_ce;
	__le64 oper_bt;
	__le32 oper_ct;
	__le32 oper_ct_ext;
	__le16 oper_cl_len;
	__le16 resv;
	struct tgst_ge ge[];
};

struct isit_keye_data {
	u8 key_type:2;
	u8 src_port_id:5; /* Only valid for switch */
	u8 spm:1; /* Only valid for switch */
	u8 resv[3];
	u8 frame_key[NTMP_ISIT_FRAME_KEY_LEN];
};

struct ist_cfge_data {
	u8 sfe:1;
	u8 rrt:1; /* version 1 */
	u8 bl2f:1; /* Only applicable to ENETC and version 1 */
	u8 resv0:1;
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	u8 imire:1; /* Only applicable to NETC switch */
	u8 timecape:1; /* Only applicable to NETC switch */
	u8 resv1:1;
	u8 sppd:1; /* Only applicable to NETC switch */
	u8 isqga:2; /* Only applicable to NETC switch */
	u8 orp:1;
	u8 osgi:1;
	u8 hr:4; /* Only applicable to NETC switch */
	union {
		struct {
			u8 fa:3;
			u8 sdu_type:2;
			u8 resv2:3;
		};

		struct {
			u8 fa:4;
			u8 sdu_type:2;
			u8 sdfa:1;
			u8 osdfa:1;
		} v1;
	};
	__le16 msdu;
	/* bits 0~6: IFME_LEN_CHANGE, bits 7~11: EPORT, bits 12~13: OETEID,
	 * bits 14~15: CTD
	 */
	__le16 switch_cfg; /* Only applicable to NETC switch */
	__le32 isqg_eid; /* Only applicable to NETC switch */
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 ifm_eid; /* Only applicable to NETC switch */
	__le32 et_eid; /* Only applicable to NETC switch */
	__le32 isc_eid;
	/* bits 0~23: EGRESS_PORT_BITMAP. For version 1, bits 24~27: EVMEID */
	__le32 bitmap_evmeid; /* Only applicable to NETC switch */
	__le16 si_bitmap;
};

union ist_switch_cfg {
	struct {
		u16 ifme_len_change:7;
		u16 eport:5;
		u16 oeteid:2;
		u16 ctd:2;
	};
	u16 val;
};

struct isft_keye_data {
	__le32 is_eid;
	u8 pcp:3;
	u8 resv0:5;
	u8 resv1[3];
};

struct isft_cfge_data {
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	u8 imire:1; /* Only applicable to NETC switch */
	u8 timecape:1; /* Only applicable to NETC switch */
	u8 osgi:1;
	u8 ctd:1; /* Only applicable to NETC switch */
	u8 orp:1;
	u8 sdu_type:2;
	u8 resv:1;
	__le16 msdu;
	__le32 rp_eid;
	__le32 sgi_eid;
	__le32 isc_eid;
};

struct sgit_acfge_data {
	__le32 admin_sgcl_eid;
	__le64 admin_base_time;
	__le32 admin_cycle_time_ext;
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
	u8 ctd:1; /* Only applicable to NETC switch */
	u8 resv:1;
};

struct sgit_sgise_data {
	__le32 oper_sgcl_eid;
	__le64 config_change_time;
	__le64 oper_base_time;
	__le32 oper_cycle_time_ext;
	u8 oex:1;
	u8 irx:1;
	u8 state:3;
	u8 resv:3;
};

struct sgclt_ge {
	__le32 interval;
	u8 iom[3];
	u8 ipv:4;
	u8 oipv:1;
	u8 ctd:1; /* Only applicable to NETC switch */
	u8 iomen:1;
	u8 gtst:1;
};

struct sgclt_cfge_data {
	__le32 cycle_time;
	u8 list_length;
	u8 resv0;
	u8 ext_oipv:1;
	u8 ext_ipv:4;
	u8 ext_ctd:1; /* Only applicable to NETC switch */
	u8 ext_gtst:1;
	u8 resv1:1;
	u8 resv2;
	struct sgclt_ge ge[];
};

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

struct isct_stse_data {
	__le32 rx_count;
	__le32 resv0;
	__le32 msdu_drop_count;
	__le32 resv1;
	__le32 policer_drop_count;
	__le32 resv2;
	__le32 sg_drop_count;
	__le32 resv3;
};

struct ipft_pld_byte {
	u8 data;
	u8 mask;
};

union ipft_src_port {
	struct {
		u16 id:5;
		u16 mask:5;
		u16 resv:6;
	};
	u16 val;
};

struct ipft_keye_data {
	__le16 precedence;
	__le16 resv0[3];
	__le16 frm_attr_flags;
	__le16 frm_attr_flags_mask;
	/* bit 0~5: dscp, bit 6~11: mask, bit 12~15: reserved */
	__le16 dscp;
	/* bit 0~4: src_port, bit 5~9: mask, bit 10~15: reserved.
	 * Note that this field is reserved for ENETC
	 */
	__le16 src_port;
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
	u8 fltfa:3;
	u8 imire:1; /* Only applicable to NETC switch */
	u8 wolte:1;
	u8 flta:2;
	u8 rpr_l:1;
	u8 rpr_h:1;
	u8 ctd:1; /* Only applicable to NETC switch */
	u8 hr:4; /* Only applicable to NETC switch */
	u8 timecape:1; /* Only applicable to NETC switch */
	u8 rrt:1; /* Only applicable to NETC switch */
	u8 bl2f:1; /* Only applicable to ENETC */
	u8 resv:3;
	u8 evmeid:4; /* Only applicable to NETC switch */
	__le32 flta_tgt;
};

#pragma pack()

struct netc_cbdr_regs {
	void __iomem *pir;
	void __iomem *cir;
	void __iomem *mr;

	void __iomem *bar0;
	void __iomem *bar1;
	void __iomem *lenr;
};

enum ntmp_table_version {
	NTMP_TBL_VER0 = 0, /* MUST be 0 */
	NTMP_TBL_VER1,
};

struct netc_tbl_vers {
	u8 maft_ver;
	u8 vaft_ver;
	u8 rsst_ver;
	u8 tgst_ver;
	u8 rpt_ver;
	u8 ipft_ver;
	u8 isit_ver;
	u8 ist_ver;
	u8 isft_ver;
	u8 sgit_ver;
	u8 sgclt_ver;
	u8 isct_ver;
};

struct netc_cbdr {
	struct netc_cbdr_regs regs;

	int bd_num;
	int next_to_use;
	int next_to_clean;

	int dma_size;
	void *addr_base;
	void *addr_base_align;
	dma_addr_t dma_base;
	dma_addr_t dma_base_align;

	spinlock_t ring_lock; /* Avoid race condition */
};

struct netc_cbdrs {
	int cbdr_num;	/* number of control BD ring */
	int cbdr_size;	/* number of BDs per control BD ring */
	struct device *dma_dev;
	struct netc_cbdr *ring;
	struct netc_tbl_vers tbl;
};

enum netc_dev_type {
	NETC_DEV_ENETC,
	NETC_DEV_SWITCH
};

struct ntmp_caps {
	int rpt_num_entries;
	int isct_num_entries;
	int ist_num_entries;
	int sgit_num_entries;
	int sgclt_num_words;
};

struct ntmp_priv {
	enum netc_dev_type dev_type;
	struct netc_cbdrs cbdrs;
	u32 errata;

	struct ntmp_caps caps;
	/* bitmap of table entry ID */
	unsigned long *ist_eid_bitmap;
	unsigned long *rpt_eid_bitmap;
	unsigned long *sgit_eid_bitmap;
	unsigned long *isct_eid_bitmap;
	unsigned long *sgclt_word_bitmap;

	struct hlist_head flower_list;
	struct mutex flower_lock; /* flower_list lock */

	u64 (*adjust_base_time)(struct ntmp_priv *priv, u64 bt, u32 ct);
	u32 (*get_tgst_free_words)(struct ntmp_priv *priv);
};

struct maft_entry_data {
	struct maft_keye_data keye;
	struct maft_cfge_data cfge;
};

struct vaft_entry_data {
	struct vaft_keye_data keye;
	struct vaft_cfge_data cfge;
};

struct tgst_query_data {
	__le64 config_change_time;
	__le64 admin_bt;
	__le32 admin_ct;
	__le32 admin_ct_ext;
	__le16 admin_cl_len;
	__le64 oper_cfg_ct;
	__le64 oper_cfg_ce;
	__le64 oper_bt;
	__le32 oper_ct;
	__le32 oper_ct_ext;
	__le16 oper_cl_len;
	struct tgst_ge olse_ge[NTMP_TGST_MAX_ENTRY_NUM];
	struct tgst_ge cfge_ge[NTMP_TGST_MAX_ENTRY_NUM];
};

struct ntmp_isit_entry {
	u32 entry_id;  /* hardware assigns entry ID */
	struct isit_keye_data keye;
	__le32 is_eid; /* cfge data */
};

struct ntmp_ist_entry {
	u32 entry_id; /* software assigns entry ID */
	struct ist_cfge_data cfge;
};

struct ntmp_isft_entry {
	u32 entry_id; /* hardware assigns entry ID */
	struct isft_keye_data keye;
	struct isft_cfge_data cfge;
};

struct ntmp_sgit_entry {
	u32 entry_id; /* software assigns entry ID */
	struct sgit_acfge_data acfge;
	struct sgit_cfge_data cfge;
	struct sgit_icfge_data icfge;
	struct sgit_sgise_data sgise;
};

struct ntmp_sgclt_entry {
	u32 entry_id;
	u8 ref_count; /* SGCLSE_DATA */
	struct sgclt_cfge_data cfge; /* Must be last member */
};

struct ntmp_rpt_entry {
	u32 entry_id;
	struct rpt_cfge_data cfge;
	struct rpt_fee_data fee;
	struct rpt_stse_data stse;
	struct rpt_pse_data pse;
};

struct ntmp_isct_entry {
	u32 entry_id;
	struct isct_stse_data stse;
};

struct ntmp_ipft_entry {
	u32 entry_id;
	struct ipft_keye_data keye;
	struct ipft_cfge_data cfge;
	__le64 match_count; /* STSE_DATA */
};

#if IS_ENABLED(CONFIG_NXP_NETC_LIB)
int netc_setup_cbdr(struct device *dev, int cbd_num, struct netc_cbdr_regs *regs,
		    struct netc_cbdr *cbdr);
void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr);

/* NTMP APIs */
u32 ntmp_lookup_free_eid(unsigned long *bitmap, u32 bitmap_size);
void ntmp_clear_eid_bitmap(unsigned long *bitmap, u32 entry_id);
u32 ntmp_lookup_free_words(unsigned long *bitmap, u32 bitmap_size,
			   u32 entry_size);
void ntmp_clear_words_bitmap(unsigned long *bitmap, u32 entry_id,
			     u32 num_words);
int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct maft_entry_data *data);
int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct maft_entry_data *data);
int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_vaft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			struct vaft_entry_data *data);
int ntmp_vaft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct vaft_entry_data *data);
int ntmp_vaft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs, u32 *table,
				    int count, bool query);
int ntmp_tgst_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct tgst_query_data *data);
int ntmp_tgst_update_admin_gate_list(struct netc_cbdrs *cbdrs, u32 entry_id,
				     struct tgst_cfge_data *cfge);
int ntmp_tgst_delete_admin_gate_list(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_rpt_entry *entry);
int ntmp_rpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ntmp_rpt_entry *entry);
int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_isit_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isit_entry *entry);
int ntmp_isit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_isit_entry *entry);
int ntmp_isit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
				 struct ntmp_ist_entry *entry);
int ntmp_ist_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			 struct ist_cfge_data *cfge);
int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_isft_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
				  struct ntmp_isft_entry *entry);
int ntmp_isft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_isft_entry *entry);
int ntmp_isft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_sgit_add_or_update_entry(struct netc_cbdrs *cbdrs,
				  struct ntmp_sgit_entry *entry);
int ntmp_sgit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  struct ntmp_sgit_entry *entry);
int ntmp_sgit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_sgclt_add_entry(struct netc_cbdrs *cbdrs,
			 struct ntmp_sgclt_entry *entry);
int ntmp_sgclt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
int ntmp_sgclt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			   struct ntmp_sgclt_entry *entry, u32 cfge_size);
int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			    int cmd, struct isct_stse_data *data);
int ntmp_ipft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
			struct ntmp_ipft_entry *entry);
int ntmp_ipft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
			  bool update, struct ntmp_ipft_entry *entry);
int ntmp_ipft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id);
#else
static inline int netc_setup_cbdr(struct device *dev, int cbd_num,
				  struct netc_cbdr_regs *regs,
				  struct netc_cbdr *cbdr)
{
	return 0;
}

static inline void netc_teardown_cbdr(struct device *dev, struct netc_cbdr *cbdr)
{
}

/* NTMP APIs */
static inline u32 ntmp_lookup_free_eid(unsigned long *bitmap, u32 size)
{
	return 0;
}

static inline void ntmp_clear_eid_bitmap(unsigned long *bitmap, u32 entry_id)
{
}

static inline u32 ntmp_lookup_free_words(unsigned long *bitmap, u32 bitmap_size,
			   u32 entry_size)
{
	return NTMP_NULL_ENTRY_ID;
}

static inline void ntmp_clear_words_bitmap(unsigned long *bitmap, u32 entry_id,
					   u32 num_words)
{
}

static inline int ntmp_maft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct maft_entry_data *data)
{
	return 0;
}

static inline int ntmp_maft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct maft_entry_data *data)
{
	return 0;
}

static inline int ntmp_maft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_vaft_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				      struct vaft_entry_data *data)
{
	return 0;
}

static inline int ntmp_vaft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct vaft_entry_data *data)
{
	return 0;
}

static inline int ntmp_vaft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_rsst_query_or_update_entry(struct netc_cbdrs *cbdrs,
						  u32 *table, int count,
						  bool query)
{
	return 0;
}

static inline int ntmp_tgst_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct tgst_query_data *data)
{
	return 0;
}

static inline int ntmp_tgst_update_admin_gate_list(struct netc_cbdrs *cbdrs,
						   u32 entry_id,
						   struct tgst_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_tgst_delete_admin_gate_list(struct netc_cbdrs *cbdrs,
						   u32 entry_id)
{
	return 0;
}

static inline int ntmp_rpt_add_or_update_entry(struct netc_cbdrs *cbdrs,
					       struct ntmp_rpt_entry *entry)
{
	return 0;
}

static inline int ntmp_rpt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				       struct ntmp_rpt_entry *entry)
{
	return 0;
}

static inline int ntmp_rpt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isit_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
						struct ntmp_isit_entry *entry)
{
	return 0;
}

static inline int ntmp_isit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct ntmp_isit_entry *entry)
{
	return 0;
}

static inline int ntmp_isit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_ist_add_or_update_entry(struct netc_cbdrs *cbdrs,
					       struct ntmp_ist_entry *entry)
{
	return 0;
}

static inline int ntmp_ist_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				       struct ist_cfge_data *cfge)
{
	return 0;
}

static inline int ntmp_ist_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isft_add_or_update_entry(struct netc_cbdrs *cbdrs, bool add,
						struct ntmp_isft_entry *entry)
{
	return 0;
}

static inline int ntmp_isft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct ntmp_isft_entry *entry)
{
	return 0;
}

static inline int ntmp_isft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_sgit_add_or_update_entry(struct netc_cbdrs *cbdrs,
						struct ntmp_sgit_entry *entry)
{
	return 0;
}

static inline int ntmp_sgit_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					struct ntmp_sgit_entry *entry)
{
	return 0;
}

static inline int ntmp_sgit_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_sgclt_add_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
				       struct ntmp_sgclt_entry *entry)
{
	return 0;
}

static inline int ntmp_sgclt_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}

static inline int ntmp_sgclt_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					 struct ntmp_sgclt_entry *entry, u32 cfge_size)
{
	return 0;
}

static inline int ntmp_isct_operate_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					  int cmd, struct isct_stse_data *stse)
{
	return 0;
}

static inline int ntmp_ipft_add_entry(struct netc_cbdrs *cbdrs, u32 *entry_id,
				      struct ntmp_ipft_entry *entry)
{
	return 0;
}

static inline int ntmp_ipft_query_entry(struct netc_cbdrs *cbdrs, u32 entry_id,
					bool update, struct ntmp_ipft_entry *entry)
{
	return 0;
}

static inline int ntmp_ipft_delete_entry(struct netc_cbdrs *cbdrs, u32 entry_id)
{
	return 0;
}
#endif

#endif /* ENETC_NTMP_H */
