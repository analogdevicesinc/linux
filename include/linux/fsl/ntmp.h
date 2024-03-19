/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2022 NXP */
#ifndef ENETC_NTMP_H
#define ENETC_NTMP_H

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

#define NTMP_TGST_MAX_CT_PLUS_CT_EXT	0xffffffffU
#define NTMP_SGIT_MAX_CT_PLUS_CT_EXT	0X3fffffff

#define NTMP_ISIT_KEY_TYPE0_SMAC_VLAN	0
#define NTMP_ISIT_KEY_TYPE1_DMAC_VLAN	1
#define NTMP_ISIT_FRMAE_UNTAG		0
#define NTMP_ISIT_FRMAE_TAG		1
#define NTMP_ISIT_FRAME_KEY_LEN		16

#define NTMP_IST_FA_DISCARD		0
#define NTMP_IST_FA_NO_SI_BITMAP	1
#define NTMP_IST_FA_SI_BITMAP		2

#define NTMP_ISFT_FLAG_OIPV		BIT(0)
#define NTMP_ISFT_FLAG_ODR		BIT(1)
#define NTMP_ISFT_FLAG_OSGI		BIT(2)
#define NTMP_ISFT_FLAG_ORP		BIT(3)

#define NTMP_IPFT_MAX_PLD_LEN		24
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
#define NTMP_IPFT_FLTFA			GENMASK(2, 0)
#define NTMP_IPFT_WOLTE			BIT(4)
#define NTMP_IPFT_FLTA			GENMASK(6, 5)
#define  NTMP_IPFT_FLTA_SI_BITMAP	3
#define NTMP_IPFT_RPR			GENMASK(8, 7)

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

#pragma pack()

struct netc_cbdr_regs {
	void __iomem *pir;
	void __iomem *cir;
	void __iomem *mr;

	void __iomem *bar0;
	void __iomem *bar1;
	void __iomem *lenr;

	/* station interface current time register */
	void __iomem *sictr0;
	void __iomem *sictr1;
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
	struct device *dma_dev;

	spinlock_t ring_lock; /* Avoid race condition */
};

struct ntmp_mfe {
	u8 mac[ETH_ALEN];
	u16 si_bitmap;
};

struct ntmp_vfe {
	u16 vid;
	u8 tpid;
	u16 si_bitmap;
};

struct ntmp_tgst_ge {
	u32 tc_gates;
	u32 interval;
	u16 oper_type; /* bit0~3: hr_cbd, bit4~15 reserved */
};

struct ntmp_tgst_cfg {
	u16 num_entries;
	u64 base_time;
	u64 cycle_time;
	u64 cycle_time_extension;
	struct ntmp_tgst_ge entries[];
};

struct ntmp_tgst_info {
	u64 status;
	u32 entry_id;
	u64 admin_bt;
	u32 admin_ct;
	u32 admin_ct_ext;
	u16 admin_cl_len;
	u64 cfg_ct;
	u64 cfg_ce;
	u64 oper_bt;
	u32 oper_ct;
	u32 oper_ct_ext;
	u16 oper_cl_len;
	struct ntmp_tgst_ge admin[NTMP_TGST_MAX_ENTRY_NUM];
	struct ntmp_tgst_ge oper[NTMP_TGST_MAX_ENTRY_NUM];
};

struct ntmp_isit_cfg {
	u32 entry_id;    /* hardware assigns entry ID */
	u8 mac[ETH_ALEN];
	u32 key_type;
	u16 vid;
	u8 tagged;
	u32 is_eid;     /* software assigns entry ID */
};

struct ntmp_isit_info {
	u32 key_type; /* bit0~1: key type, other bits: reserved */
	u8 key[NTMP_ISIT_FRAME_KEY_LEN];
	u32 is_eid;
};

struct ntmp_ist_cfg {
	u32 entry_id;      /* software assigns entry ID */
	u32 rp_eid;
	u32 sgi_eid;
	u32 isc_eid;
	u16 msdu;
	u8 sfe:1;
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	u8 orp:1;
	u8 osgi:1;
	u8 fa:3;
	u8 sdu_type:2;
	u16 si_bitmap;
};

struct ntmp_ist_info {
	u32 sfe:1;
	u32 ipv:4;
	u32 oipv:1;
	u32 dr:2;
	u32 odr:1;
	u32 orp:1;
	u32 osgi:1;
	u32 fa:3;
	u32 sdu_type:2;
	u16 msdu;
	u16 si_bitmap;
	u32 rp_eid;
	u32 sgi_eid;
	u32 isc_eid;
};

struct ntmp_isft_cfg {
	u32 entry_id;    /* hardware assigns entry ID */
	u32 is_eid;
	u32 rp_eid;
	u32 sgi_eid;
	u32 isc_eid;
	u16 msdu;
	u8 priority;
	u32 or_flags;
};

struct ntmp_isft_info {
	u32 is_eid;
	u16 pcp:3;
	u16 ipv:4;
	u16 oipv:1;
	u16 dr:2;
	u16 odr:1;
	u16 osgi:1;
	u16 orp:1;
	u16 sdu_type:2;
	u16 msdu;
	u32 rp_eid;
	u32 sgi_eid;
	u32 isc_eid;
};

struct ntmp_sgit_cfg {
	u32 entry_id;        /* software assigns entry ID */
	s8 init_ipv;
	u32 admin_sgcl_eid; /* software assigns entry ID */
	u64 admin_bt;
	u32 admin_ct_ext;
	u32 sgcl_ct;
	refcount_t refcount;
	struct hlist_node node;
};

struct ntmp_sgclt_cfg {
	u32 entry_id;
	s8 init_ipv;
	u32 ct;
	u32 num_gates;
	struct action_gate_entry entries[];
};

struct ntmp_rpt_cfg {
	u32 entry_id;
	u32 cir;
	u32 cbs;
	u32 eir;
	u32 ebs;
	refcount_t refcount;
	struct hlist_node node;
};

struct ntmp_rpt_sts {
	u64 byte_cnt;
	u32 drop_frames;
	u32 dr0_grn_frames;
	u32 dr1_grn_frames;
	u32 dr2_ylw_frames;
	u32 remark_ylw_frames;
	u32 dr3_red_frames;
	u32 remark_red_frames;
};

struct ntmp_rpt_cfge {
	u32 cir;
	u32 cbs;
	u32 eir;
	u32 ebs;
	u8 mren:1;
	u8 doy:1;
	u8 cm:1;
	u8 cf:1;
	u8 ndor:1;
	u8 sdu_type:2;
};

struct ntmp_rpt_info {
	struct ntmp_rpt_sts sts;
	struct ntmp_rpt_cfge cfg;
	bool fen;
	u8 mr;
};

struct ntmp_isct_cfg {
	u32 entry_id;
};

struct ntmp_sgit_info {
	u64 cfg_ct;
	u64 oper_bt;
	u64 admin_bt;
	u32 oper_ct_ext;
	u32 admin_ct_ext;
	u32 oper_sgcl_eid;
	u32 admin_sgcl_eid;
	u32 oex:1;
	u32 irx:1;
	u32 state:3;
	u32 oexen:1;
	u32 irxen:1;
	u32 sdu_type:2;
	u32 ipv:4;
	u32 oipv:1;
	u32 gst:1;
};

struct ntmp_sgclt_ge {
	u32 interval;
	u32 iom:24;
	u32 ipv:4;
	u32 oipv:1;
	u32 iomen:1;
	u32 gtst:1;
};

struct ntmp_sgclt_info {
	u32 cycle_time;
	u8 ref_count;
	u16 list_len;
	u8 ext_gtst:1;
	u8 ext_ipv:4;
	u8 ext_oipv:1;
	struct ntmp_sgclt_ge ge[NTMP_SGCLT_MAX_GE_NUM];
};

struct ntmp_isct_info {
	u32 rx_count;
	u32 msdu_drop_count;
	u32 policer_drop_count;
	u32 sg_drop_count;
};

struct ipft_pld_data {
	u8 data;
	u8 mask;
};

struct ntmp_ipft_key {
	u16 precedence;
	u16 frm_attr_flags;
	u16 frm_attr_flags_mask;
	u16 dscp; /* bit0~5: dscp, bit6~11: mask, bit12~15: reserved */
	u16 src_port; /* bit0~4: src_port, bit5~9: mask, bit10~15: reserved */
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
	__be32 ip_src[4];
	__be32 ip_src_mask[4];
	__be16 l4_src_port;
	__be16 l4_src_port_mask;
	__be32 ip_dst[4];
	__be32 ip_dst_mask[4];
	__be16 l4_dst_port;
	__be16 l4_dst_port_mask;
	struct ipft_pld_data byte[NTMP_IPFT_MAX_PLD_LEN];
};

struct ntmp_ipft_cfg {
	u8 ipv:4;
	u8 oipv:1;
	u8 dr:2;
	u8 odr:1;
	u16 filter; /* bit0~2: fltfa, bit4: wolte, bit5~6: flta, bit7~8: rpr */
	u32 flta_tgt;
};

struct ntmp_ipft_info {
	struct ntmp_ipft_key key;
	u64 match_count;
	struct ntmp_ipft_cfg cfg;
};

#if IS_ENABLED(CONFIG_FSL_NTMP)
int netc_setup_cbdr(struct device *dev, int cbd_num,
		    struct netc_cbdr_regs *regs,
		    struct netc_cbdr *cbdr);
void netc_free_cbdr(struct netc_cbdr *cbdr);

/* NTMP APIs */
int ntmp_maft_add_entry(struct netc_cbdr *cbdr, u32 entry_id,
			const char *mac_addr, int si_bitmap);
int ntmp_maft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_mfe *entry);
int ntmp_maft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_vaft_add_entry(struct netc_cbdr *cbdr, u32 entry_id,
			struct ntmp_vfe *vfe);
int ntmp_vaft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_vfe *entry);
int ntmp_vaft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_rsst_query_or_update_entry(struct netc_cbdr *cbdr, u32 *table,
				    int count, bool query);
int ntmp_tgst_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_tgst_info *info);
int ntmp_tgst_update_admin_gate_list(struct netc_cbdr *cbdr, u32 entry_id,
				     struct ntmp_tgst_cfg *cfg);
int ntmp_tgst_delete_admin_gate_list(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_rpt_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_rpt_cfg *cfg);
int ntmp_rpt_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			 struct ntmp_rpt_info *info);
int ntmp_rpt_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_isit_add_or_update_entry(struct netc_cbdr *cbdr,
				  struct ntmp_isit_cfg *cfg, bool add);
int ntmp_isit_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_isit_info *info);
int ntmp_isit_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_ist_add_or_update_entry(struct netc_cbdr *cbdr, struct ntmp_ist_cfg *cfg);
int ntmp_ist_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			 struct ntmp_ist_info *info);
int ntmp_ist_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_isft_add_or_update_entry(struct netc_cbdr *cbdr,
				  struct ntmp_isft_cfg *cfg, bool add);
int ntmp_isft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_isft_info *info);
int ntmp_isft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_sgit_add_or_update_entry(struct netc_cbdr *cbdr,
				  struct ntmp_sgit_cfg *cfg);
int ntmp_sgit_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_sgit_info *info);
int ntmp_sgit_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_sgclt_add_entry(struct netc_cbdr *cbdr, struct ntmp_sgclt_cfg *cfg);
int ntmp_sgclt_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
int ntmp_sgclt_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			   struct ntmp_sgclt_info *info);
int ntmp_isct_operate_entry(struct netc_cbdr *cbdr, u32 entry_id,
			    int cmd, struct ntmp_isct_info *info);
int ntmp_ipft_add_entry(struct netc_cbdr *cbdr, struct ntmp_ipft_key *key,
			struct ntmp_ipft_cfg *cfg, u32 *entry_id);
int ntmp_ipft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
			  struct ntmp_ipft_info *info);
int ntmp_ipft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id);
#else
static inline int netc_setup_cbdr(struct device *dev, int cbd_num,
				  struct netc_cbdr_regs *regs,
				  struct netc_cbdr *cbdr)
{
	return 0;
}

static inline void netc_free_cbdr(struct netc_cbdr *cbdr)
{
}

/* NTMP APIs */
static inline int ntmp_maft_add_entry(struct netc_cbdr *cbdr, u32 entry_id,
				      const char *mac_addr, int si_bitmap)
{
	return 0;
}

static inline int ntmp_maft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_mfe *entry)
{
	return 0;
}

static inline int ntmp_maft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_vaft_add_entry(struct netc_cbdr *cbdr, u32 entry_id,
				      struct ntmp_vfe *vfe)
{
	return 0;
}

static inline int ntmp_vaft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_vfe *entry)
{
	return 0;
}

static inline int ntmp_vaft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_rsst_query_or_update_entry(struct netc_cbdr *cbdr,
						  u32 *table, int count,
						  bool query)
{
	return 0;
}

static inline int ntmp_tgst_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_tgst_info *info)
{
	return 0;
}

static inline int ntmp_tgst_update_admin_gate_list(struct netc_cbdr *cbdr,
						   u32 entry_id,
						   struct ntmp_tgst_cfg *cfg)
{
	return 0;
}

static inline int ntmp_tgst_delete_admin_gate_list(struct netc_cbdr *cbdr,
						   u32 entry_id)
{
	return 0;
}

static inline int ntmp_rpt_add_or_update_entry(struct netc_cbdr *cbdr,
					       struct ntmp_rpt_cfg *cfg)
{
	return 0;
}

static inline int ntmp_rpt_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
				       struct ntmp_rpt_info *info)
{
	return 0;
}

static inline int ntmp_rpt_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isit_add_or_update_entry(struct netc_cbdr *cbdr,
						struct ntmp_isit_cfg *cfg,
						bool add)
{
	return 0;
}

static inline int ntmp_isit_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_isit_info *info)
{
	return 0;
}

static inline int ntmp_isit_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_ist_add_or_update_entry(struct netc_cbdr *cbdr,
					       struct ntmp_ist_cfg *cfg)
{
	return 0;
}

static inline int ntmp_ist_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
				       struct ntmp_ist_info *info)
{
	return 0;
}

static inline int ntmp_ist_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_isft_add_or_update_entry(struct netc_cbdr *cbdr,
						struct ntmp_isft_cfg *cfg,
						bool add)
{
	return 0;
}

static inline int ntmp_isft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_isft_info *info)
{
	return 0;
}

static inline int ntmp_isft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_sgit_add_or_update_entry(struct netc_cbdr *cbdr,
						struct ntmp_sgit_cfg *cfg)
{
	return 0;
}

static inline int ntmp_sgit_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_sgit_info *info)
{
	return 0;
}

static inline int ntmp_sgit_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_sgclt_add_entry(struct netc_cbdr *cbdr,
				       struct ntmp_sgclt_cfg *cfg)
{
	return 0;
}

static inline int ntmp_sgclt_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}

static inline int ntmp_sgclt_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					 struct ntmp_sgclt_info *info)
{
	return 0;
}

static inline int ntmp_isct_operate_entry(struct netc_cbdr *cbdr, u32 entry_id,
					  int cmd, struct ntmp_isct_info *info)
{
	return 0;
}

static inline int ntmp_ipft_add_entry(struct netc_cbdr *cbdr,
				      struct ntmp_ipft_key *key,
				      struct ntmp_ipft_cfg *cfg,
				      u32 *entry_id)
{
	return 0;
}

static inline int ntmp_ipft_query_entry(struct netc_cbdr *cbdr, u32 entry_id,
					struct ntmp_ipft_info *info)
{
	return 0;
}

static inline int ntmp_ipft_delete_entry(struct netc_cbdr *cbdr, u32 entry_id)
{
	return 0;
}
#endif

#endif /* ENETC_NTMP_H */
