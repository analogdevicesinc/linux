/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2024 NXP */
#ifndef __NETC_LIB_H
#define __NETC_LIB_H

#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/fsl/ntmp.h>

#define NETC_REVISION_4_1		0x0401
#define NETC_REVISION_4_3		0x0403

#define is_en(x)	(x) ? "Enabled" : "Disabled"
#define is_yes(x)	(x) ? "Yes" : "No"

enum netc_flower_type {
	FLOWER_TYPE_PSFP,
};

enum netc_key_tbl_type {
	FLOWER_KEY_TBL_ISIT,
	FLOWER_KEY_TBL_IPFT,
};

struct netc_flower_key_tbl {
	enum netc_key_tbl_type tbl_type;
	union {
		struct ntmp_isit_entry *isit_entry;
		struct ntmp_ipft_entry *ipft_entry;
	};
	struct ntmp_ist_entry *ist_entry;
	refcount_t refcount;
};

struct netc_flower_rule {
	u32 port_id;
	unsigned long cookie;
	enum netc_flower_type flower_type;
	struct netc_flower_key_tbl *key_tbl;
	struct ntmp_isft_entry *isft_entry;
	/* Valid if flower_type is FLOWER_TYPE_PSFP */
	u32 sgclt_eid;
	u64 lastused; /* Last used time, jiffies */
	struct hlist_node node;
};

struct netc_flower {
	u64 key_acts; /* essential actions */
	u64 opt_acts; /* optional actions */
	u64 keys;
	enum netc_flower_type type;
};

struct isit_psfp_frame_key {
	u8 mac[ETH_ALEN];
	u8 vlan_h; /* Most significant byte of the 2 bytes */
	u8 vlan_l; /* Least significant byte of the 2 bytes */
	u8 resv[8];
};

struct netc_psfp_tbl_entries {
	struct ntmp_isit_entry *isit_entry;
	struct ntmp_ist_entry *ist_entry;
	struct ntmp_isft_entry *isft_entry;
	struct ntmp_sgit_entry *sgit_entry;
	struct ntmp_sgclt_entry *sgclt_entry;
	struct ntmp_isct_entry *isct_entry;
	struct ntmp_rpt_entry *rpt_entry;
};

#if IS_ENABLED(CONFIG_NXP_NETC_LIB)
/* tc flower API */
struct netc_flower_rule *
netc_find_flower_rule_by_cookie(struct ntmp_priv *priv, int port_id,
				unsigned long cookie);
struct netc_flower_rule *
netc_find_flower_rule_by_key(struct ntmp_priv *priv,
			     enum netc_key_tbl_type tbl_type, void *key);
void netc_init_ist_entry_eids(struct ntmp_priv *priv,
			      struct ntmp_ist_entry *ist_entry);
void netc_free_flower_key_tbl(struct ntmp_priv *priv,
			      struct netc_flower_key_tbl *key_tbl);
int netc_police_entry_validate(struct ntmp_priv *priv,
			       const struct flow_action *action,
			       const struct flow_action_entry *police_entry,
			       struct netlink_ext_ack *extack);
void netc_rpt_entry_config(struct flow_action_entry *police_entry,
			   struct ntmp_rpt_entry *rpt_entry);
int netc_setup_psfp(struct ntmp_priv *priv, int port_id,
		    struct flow_cls_offload *f);
void netc_delete_psfp_flower_rule(struct ntmp_priv *priv,
				  struct netc_flower_rule *rule);
int netc_psfp_flower_stat(struct ntmp_priv *priv, struct netc_flower_rule *rule,
			  u64 *byte_cnt, u64 *pkt_cnt, u64 *drop_cnt);
int netc_setup_taprio(struct ntmp_priv *priv, u32 entry_id,
		      struct tc_taprio_qopt_offload *f);

/* debugfs API */
int netc_kstrtouint(const char __user *buffer, size_t count, loff_t *ppos, u32 *val);
void netc_show_psfp_flower(struct seq_file *s, struct netc_flower_rule *rule);
int netc_show_isit_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_ist_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_isft_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_sgit_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_sgclt_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_isct_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_rpt_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_ipft_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
int netc_show_tgst_entry(struct ntmp_priv *priv, struct seq_file *s, u32 entry_id);
#else
static inline int netc_kstrtouint(const char __user *buffer, size_t count,
				  loff_t *ppos, u32 *val)
{
	return 0;
}

static inline struct netc_flower_rule *
netc_find_flower_rule_by_cookie(struct ntmp_priv *priv, int port_id,
				unsigned long cookie)
{
	return NULL;
}

static inline struct netc_flower_rule *
netc_find_flower_rule_by_key(struct ntmp_priv *priv,
			     enum netc_key_tbl_type tbl_type, void *key)
{
	return NULL;
}

static inline void netc_init_ist_entry_eids(struct ntmp_priv *priv,
					    struct ntmp_ist_entry *ist_entry)
{
}

static inline void netc_free_flower_key_tbl(struct ntmp_priv *priv,
					    struct netc_flower_key_tbl *key_tbl)
{
}

static inline int netc_police_entry_validate(struct ntmp_priv *priv,
					    const struct flow_action *action,
					    const struct flow_action_entry *police_entry,
					    struct netlink_ext_ack *extack)
{
	return 0;
}

static inline void netc_rpt_entry_config(struct flow_action_entry *police_entry,
					 struct ntmp_rpt_entry *rpt_entry)
{
}

static inline int netc_setup_psfp(struct ntmp_priv *priv, int port_id,
		    struct flow_cls_offload *f)
{
	return 0;
}

static inline void
netc_delete_psfp_flower_rule(struct ntmp_priv *priv,
			     struct netc_flower_rule *rule)
{
}

static inline int netc_psfp_flower_stat(struct ntmp_priv *priv,
					struct netc_flower_rule *rule,
					u64 *byte_cnt, u64 *pkt_cnt,
					u64 *drop_cnt)
{
	return 0;
}

static inline int netc_setup_taprio(struct ntmp_priv *priv, u32 entry_id,
				    struct tc_taprio_qopt_offload *f)
{
	return 0;
}

static inline void netc_show_psfp_flower(struct seq_file *s,
					 struct netc_flower_rule *rule)
{
}

static inline int netc_show_isit_entry(struct ntmp_priv *priv,
				       struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_ist_entry(struct ntmp_priv *priv,
				      struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_isft_entry(struct ntmp_priv *priv,
				       struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_sgit_entry(struct ntmp_priv *priv,
				       struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_sgclt_entry(struct ntmp_priv *priv,
					struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_isct_entry(struct ntmp_priv *priv,
				       struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_rpt_entry(struct ntmp_priv *priv,
				      struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_ipft_entry(struct ntmp_priv *priv,
				       struct seq_file *s, u32 entry_id)
{
	return 0;
}

static inline int netc_show_tgst_entry(struct ntmp_priv *priv,
				       struct seq_file *s, u32 entry_id)
{
	return 0;
}

#endif

#endif /* __NETC_LIB_H */
