/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2016, 2020, 2024 NXP
 */
#ifndef _FSL_DPNI_CMD_H
#define _FSL_DPNI_CMD_H

#include "dpni.h"

/* DPNI Version */
#define DPNI_VER_MAJOR				7
#define DPNI_VER_MINOR				0
#define DPNI_CMD_BASE_VERSION			1
#define DPNI_CMD_2ND_VERSION			2
#define DPNI_CMD_3RD_VERSION			3
#define DPNI_CMD_ID_OFFSET			4

#define DPNI_CMD(id)	(((id) << DPNI_CMD_ID_OFFSET) | DPNI_CMD_BASE_VERSION)
#define DPNI_CMD_V2(id)	(((id) << DPNI_CMD_ID_OFFSET) | DPNI_CMD_2ND_VERSION)
#define DPNI_CMD_V3(id)	(((id) << DPNI_CMD_ID_OFFSET) | DPNI_CMD_3RD_VERSION)

#define DPNI_CMDID_OPEN					DPNI_CMD(0x801)
#define DPNI_CMDID_CLOSE				DPNI_CMD(0x800)
#define DPNI_CMDID_CREATE				DPNI_CMD(0x901)
#define DPNI_CMDID_DESTROY				DPNI_CMD(0x900)
#define DPNI_CMDID_GET_API_VERSION			DPNI_CMD(0xa01)

#define DPNI_CMDID_ENABLE				DPNI_CMD(0x002)
#define DPNI_CMDID_DISABLE				DPNI_CMD(0x003)
#define DPNI_CMDID_GET_ATTR				DPNI_CMD(0x004)
#define DPNI_CMDID_GET_ATTR_V2				DPNI_CMD_V2(0x004)
#define DPNI_CMDID_RESET				DPNI_CMD(0x005)
#define DPNI_CMDID_IS_ENABLED				DPNI_CMD(0x006)

#define DPNI_CMDID_SET_IRQ				DPNI_CMD(0x010)
#define DPNI_CMDID_GET_IRQ				DPNI_CMD(0x011)
#define DPNI_CMDID_SET_IRQ_ENABLE			DPNI_CMD(0x012)
#define DPNI_CMDID_GET_IRQ_ENABLE			DPNI_CMD(0x013)
#define DPNI_CMDID_SET_IRQ_MASK				DPNI_CMD(0x014)
#define DPNI_CMDID_GET_IRQ_MASK				DPNI_CMD(0x015)
#define DPNI_CMDID_GET_IRQ_STATUS			DPNI_CMD(0x016)
#define DPNI_CMDID_CLEAR_IRQ_STATUS			DPNI_CMD(0x017)

#define DPNI_CMDID_SET_POOLS				DPNI_CMD_V3(0x200)
#define DPNI_CMDID_SET_ERRORS_BEHAVIOR			DPNI_CMD(0x20B)

#define DPNI_CMDID_GET_QDID				DPNI_CMD(0x210)
#define DPNI_CMDID_GET_TX_DATA_OFFSET			DPNI_CMD(0x212)
#define DPNI_CMDID_GET_LINK_STATE			DPNI_CMD(0x215)
#define DPNI_CMDID_SET_MAX_FRAME_LENGTH			DPNI_CMD(0x216)
#define DPNI_CMDID_GET_MAX_FRAME_LENGTH			DPNI_CMD(0x217)
#define DPNI_CMDID_SET_LINK_CFG				DPNI_CMD(0x21A)
#define DPNI_CMDID_SET_TX_SHAPING			DPNI_CMD_V2(0x21B)

#define DPNI_CMDID_SET_MCAST_PROMISC			DPNI_CMD(0x220)
#define DPNI_CMDID_GET_MCAST_PROMISC			DPNI_CMD(0x221)
#define DPNI_CMDID_SET_UNICAST_PROMISC			DPNI_CMD(0x222)
#define DPNI_CMDID_GET_UNICAST_PROMISC			DPNI_CMD(0x223)
#define DPNI_CMDID_SET_PRIM_MAC				DPNI_CMD(0x224)
#define DPNI_CMDID_GET_PRIM_MAC				DPNI_CMD(0x225)
#define DPNI_CMDID_ADD_MAC_ADDR				DPNI_CMD(0x226)
#define DPNI_CMDID_REMOVE_MAC_ADDR			DPNI_CMD(0x227)
#define DPNI_CMDID_CLR_MAC_FILTERS			DPNI_CMD(0x228)

#define DPNI_CMDID_SET_RX_TC_DIST			DPNI_CMD(0x235)

#define DPNI_CMDID_ENABLE_VLAN_FILTER			DPNI_CMD(0x230)
#define DPNI_CMDID_ADD_VLAN_ID				DPNI_CMD_V2(0x231)
#define DPNI_CMDID_REMOVE_VLAN_ID			DPNI_CMD(0x232)

#define DPNI_CMDID_SET_QOS_TBL				DPNI_CMD(0x240)
#define DPNI_CMDID_ADD_QOS_ENT				DPNI_CMD(0x241)
#define DPNI_CMDID_REMOVE_QOS_ENT			DPNI_CMD(0x242)
#define DPNI_CMDID_CLR_QOS_TBL				DPNI_CMD(0x243)
#define DPNI_CMDID_ADD_FS_ENT				DPNI_CMD(0x244)
#define DPNI_CMDID_REMOVE_FS_ENT			DPNI_CMD(0x245)
#define DPNI_CMDID_CLR_FS_ENT				DPNI_CMD(0x246)

#define DPNI_CMDID_SET_TX_PRIORITIES			DPNI_CMD_V2(0x250)
#define DPNI_CMDID_GET_STATISTICS			DPNI_CMD_V2(0x25D)
#define DPNI_CMDID_RESET_STATISTICS			DPNI_CMD(0x25E)
#define DPNI_CMDID_GET_QUEUE				DPNI_CMD(0x25F)
#define DPNI_CMDID_SET_QUEUE				DPNI_CMD(0x260)
#define DPNI_CMDID_GET_TAILDROP				DPNI_CMD(0x261)
#define DPNI_CMDID_SET_TAILDROP				DPNI_CMD(0x262)

#define DPNI_CMDID_GET_PORT_MAC_ADDR			DPNI_CMD(0x263)

#define DPNI_CMDID_GET_BUFFER_LAYOUT			DPNI_CMD(0x264)
#define DPNI_CMDID_SET_BUFFER_LAYOUT			DPNI_CMD(0x265)

#define DPNI_CMDID_SET_TX_CONFIRMATION_MODE		DPNI_CMD(0x266)
#define DPNI_CMDID_SET_CONGESTION_NOTIFICATION		DPNI_CMD(0x267)
#define DPNI_CMDID_GET_CONGESTION_NOTIFICATION		DPNI_CMD(0x268)
#define DPNI_CMDID_SET_EARLY_DROP			DPNI_CMD(0x269)
#define DPNI_CMDID_GET_EARLY_DROP			DPNI_CMD(0x26A)
#define DPNI_CMDID_GET_OFFLOAD				DPNI_CMD(0x26B)
#define DPNI_CMDID_SET_OFFLOAD				DPNI_CMD(0x26C)

#define DPNI_CMDID_SET_RX_FS_DIST			DPNI_CMD(0x273)
#define DPNI_CMDID_SET_RX_HASH_DIST			DPNI_CMD(0x274)
#define DPNI_CMDID_GET_LINK_CFG				DPNI_CMD(0x278)

#define DPNI_CMDID_SET_SINGLE_STEP_CFG			DPNI_CMD(0x279)
#define DPNI_CMDID_GET_SINGLE_STEP_CFG			DPNI_CMD_V2(0x27a)

#define DPNI_CMDID_IS_MACSEC_CAPABLE			DPNI_CMD(0x2a0)
#define DPNI_CMDID_ADD_SECY				DPNI_CMD(0x2a1)
#define DPNI_CMDID_REMOVE_SECY				DPNI_CMD(0x2a2)
#define DPNI_CMDID_SECY_SET_STATE			DPNI_CMD(0x2a3)
#define DPNI_CMDID_SECY_SET_PROTECT			DPNI_CMD(0x2a4)
#define DPNI_CMDID_SECY_SET_REPLAY_PROTECT		DPNI_CMD(0x2a5)
#define DPNI_CMDID_SECY_ADD_TX_SA			DPNI_CMD(0x2a6)
#define DPNI_CMDID_SECY_REMOVE_TX_SA			DPNI_CMD(0x2a7)
#define DPNI_CMDID_SECY_SET_ACTIVE_TX_SA		DPNI_CMD(0x2a8)
#define DPNI_CMDID_SECY_ADD_RX_SC			DPNI_CMD(0x2a9)
#define DPNI_CMDID_SECY_REMOVE_RX_SC			DPNI_CMD(0x2aa)
#define DPNI_CMDID_SECY_SET_RX_SC_STATE			DPNI_CMD(0x2ab)
#define DPNI_CMDID_SECY_ADD_RX_SA			DPNI_CMD(0x2ac)
#define DPNI_CMDID_SECY_REMOVE_RX_SA			DPNI_CMD(0x2ad)
#define DPNI_CMDID_SECY_SET_RX_SA_NEXT_PN		DPNI_CMD(0x2ae)
#define DPNI_CMDID_SECY_SET_RX_SA_STATE			DPNI_CMD(0x2af)

#define DPNI_CMDID_SECY_GET_STATS			DPNI_CMD(0x2b0)
#define DPNI_CMDID_SECY_GET_TX_SC_STATS			DPNI_CMD(0x2b1)
#define DPNI_CMDID_SECY_GET_TX_SA_STATS			DPNI_CMD(0x2b2)
#define DPNI_CMDID_SECY_GET_RX_SC_STATS			DPNI_CMD(0x2b3)
#define DPNI_CMDID_SECY_GET_RX_SA_STATS			DPNI_CMD(0x2b4)
#define DPNI_CMDID_GET_MACSEC_STATS			DPNI_CMD(0x2b5)

/* Macros for accessing command fields smaller than 1byte */
#define DPNI_MASK(field)	\
	GENMASK(DPNI_##field##_SHIFT + DPNI_##field##_SIZE - 1, \
		DPNI_##field##_SHIFT)

#define dpni_set_field(var, field, val)	\
	((var) |= (((val) << DPNI_##field##_SHIFT) & DPNI_MASK(field)))
#define dpni_get_field(var, field)	\
	(((var) & DPNI_MASK(field)) >> DPNI_##field##_SHIFT)

struct dpni_cmd_open {
	__le32 dpni_id;
};

#define DPNI_BACKUP_POOL(val, order)	(((val) & 0x1) << (order))

struct dpni_cmd_pool {
	__le16 dpbp_id;
	u8 priority_mask;
	u8 pad;
};

struct dpni_cmd_set_pools {
	u8 num_dpbp;
	u8 backup_pool_mask;
	u8 pad;
	u8 pool_options;
	struct dpni_cmd_pool pool[DPNI_MAX_DPBP];
	__le16 buffer_size[DPNI_MAX_DPBP];
};

/* The enable indication is always the least significant bit */
#define DPNI_ENABLE_SHIFT		0
#define DPNI_ENABLE_SIZE		1

struct dpni_rsp_is_enabled {
	u8 enabled;
};

struct dpni_rsp_get_irq {
	/* response word 0 */
	__le32 irq_val;
	__le32 pad;
	/* response word 1 */
	__le64 irq_addr;
	/* response word 2 */
	__le32 irq_num;
	__le32 type;
};

struct dpni_cmd_set_irq_enable {
	u8 enable;
	u8 pad[3];
	u8 irq_index;
};

struct dpni_cmd_get_irq_enable {
	__le32 pad;
	u8 irq_index;
};

struct dpni_rsp_get_irq_enable {
	u8 enabled;
};

struct dpni_cmd_set_irq_mask {
	__le32 mask;
	u8 irq_index;
};

struct dpni_cmd_get_irq_mask {
	__le32 pad;
	u8 irq_index;
};

struct dpni_rsp_get_irq_mask {
	__le32 mask;
};

struct dpni_cmd_get_irq_status {
	__le32 status;
	u8 irq_index;
};

struct dpni_rsp_get_irq_status {
	__le32 status;
};

struct dpni_cmd_clear_irq_status {
	__le32 status;
	u8 irq_index;
};

struct dpni_rsp_get_attr {
	/* response word 0 */
	__le32 options;
	u8 num_queues;
	u8 num_rx_tcs;
	u8 mac_filter_entries;
	u8 num_tx_tcs;
	/* response word 1 */
	u8 vlan_filter_entries;
	u8 pad1;
	u8 qos_entries;
	u8 pad2;
	__le16 fs_entries;
	__le16 pad3;
	/* response word 2 */
	u8 qos_key_size;
	u8 fs_key_size;
	__le16 wriop_version;
};

#define DPNI_ERROR_ACTION_SHIFT		0
#define DPNI_ERROR_ACTION_SIZE		4
#define DPNI_FRAME_ANN_SHIFT		4
#define DPNI_FRAME_ANN_SIZE		1

struct dpni_cmd_set_errors_behavior {
	__le32 errors;
	/* from least significant bit: error_action:4, set_frame_annotation:1 */
	u8 flags;
};

/* There are 3 separate commands for configuring Rx, Tx and Tx confirmation
 * buffer layouts, but they all share the same parameters.
 * If one of the functions changes, below structure needs to be split.
 */

#define DPNI_PASS_TS_SHIFT		0
#define DPNI_PASS_TS_SIZE		1
#define DPNI_PASS_PR_SHIFT		1
#define DPNI_PASS_PR_SIZE		1
#define DPNI_PASS_FS_SHIFT		2
#define DPNI_PASS_FS_SIZE		1

struct dpni_cmd_get_buffer_layout {
	u8 qtype;
};

struct dpni_rsp_get_buffer_layout {
	/* response word 0 */
	u8 pad0[6];
	/* from LSB: pass_timestamp:1, parser_result:1, frame_status:1 */
	u8 flags;
	u8 pad1;
	/* response word 1 */
	__le16 private_data_size;
	__le16 data_align;
	__le16 head_room;
	__le16 tail_room;
};

struct dpni_cmd_set_buffer_layout {
	/* cmd word 0 */
	u8 qtype;
	u8 pad0[3];
	__le16 options;
	/* from LSB: pass_timestamp:1, parser_result:1, frame_status:1 */
	u8 flags;
	u8 pad1;
	/* cmd word 1 */
	__le16 private_data_size;
	__le16 data_align;
	__le16 head_room;
	__le16 tail_room;
};

struct dpni_cmd_set_offload {
	u8 pad[3];
	u8 dpni_offload;
	__le32 config;
};

struct dpni_cmd_get_offload {
	u8 pad[3];
	u8 dpni_offload;
};

struct dpni_rsp_get_offload {
	__le32 pad;
	__le32 config;
};

struct dpni_cmd_get_qdid {
	u8 qtype;
};

struct dpni_rsp_get_qdid {
	__le16 qdid;
};

struct dpni_rsp_get_tx_data_offset {
	__le16 data_offset;
};

struct dpni_cmd_get_statistics {
	u8 page_number;
	u8 param;
};

struct dpni_rsp_get_statistics {
	__le64 counter[DPNI_STATISTICS_CNT];
};

struct dpni_cmd_link_cfg {
	/* cmd word 0 */
	__le64 pad0;
	/* cmd word 1 */
	__le32 rate;
	__le32 pad1;
	/* cmd word 2 */
	__le64 options;
};

#define DPNI_LINK_STATE_SHIFT		0
#define DPNI_LINK_STATE_SIZE		1

struct dpni_rsp_get_link_state {
	/* response word 0 */
	__le32 pad0;
	/* from LSB: up:1 */
	u8 flags;
	u8 pad1[3];
	/* response word 1 */
	__le32 rate;
	__le32 pad2;
	/* response word 2 */
	__le64 options;
};

struct dpni_cmd_set_max_frame_length {
	__le16 max_frame_length;
};

struct dpni_rsp_get_max_frame_length {
	__le16 max_frame_length;
};

struct dpni_cmd_set_multicast_promisc {
	u8 enable;
};

struct dpni_rsp_get_multicast_promisc {
	u8 enabled;
};

struct dpni_cmd_set_unicast_promisc {
	u8 enable;
};

struct dpni_rsp_get_unicast_promisc {
	u8 enabled;
};

struct dpni_cmd_set_primary_mac_addr {
	__le16 pad;
	u8 mac_addr[6];
};

struct dpni_rsp_get_primary_mac_addr {
	__le16 pad;
	u8 mac_addr[6];
};

struct dpni_rsp_get_port_mac_addr {
	__le16 pad;
	u8 mac_addr[6];
};

struct dpni_cmd_add_mac_addr {
	__le16 pad;
	u8 mac_addr[6];
};

struct dpni_cmd_remove_mac_addr {
	__le16 pad;
	u8 mac_addr[6];
};

#define DPNI_UNICAST_FILTERS_SHIFT	0
#define DPNI_UNICAST_FILTERS_SIZE	1
#define DPNI_MULTICAST_FILTERS_SHIFT	1
#define DPNI_MULTICAST_FILTERS_SIZE	1

struct dpni_cmd_clear_mac_filters {
	/* from LSB: unicast:1, multicast:1 */
	u8 flags;
};

#define DPNI_SEPARATE_GRP_SHIFT 0
#define DPNI_SEPARATE_GRP_SIZE  1
#define DPNI_MODE_1_SHIFT		0
#define DPNI_MODE_1_SIZE		4
#define DPNI_MODE_2_SHIFT		4
#define DPNI_MODE_2_SIZE		4

struct dpni_cmd_set_tx_priorities {
	__le16 flags;
	u8 prio_group_A;
	u8 prio_group_B;
	__le32 pad0;
	u8 modes[4];
	__le32 pad1;
	__le64 pad2;
	__le16 delta_bandwidth[8];
};

#define DPNI_DIST_MODE_SHIFT		0
#define DPNI_DIST_MODE_SIZE		4
#define DPNI_MISS_ACTION_SHIFT		4
#define DPNI_MISS_ACTION_SIZE		4

struct dpni_cmd_set_rx_tc_dist {
	/* cmd word 0 */
	__le16 dist_size;
	u8 tc_id;
	/* from LSB: dist_mode:4, miss_action:4 */
	u8 flags;
	__le16 pad0;
	__le16 default_flow_id;
	/* cmd word 1..5 */
	__le64 pad1[5];
	/* cmd word 6 */
	__le64 key_cfg_iova;
};

/* dpni_set_rx_tc_dist extension (structure of the DMA-able memory at
 * key_cfg_iova)
 */
struct dpni_mask_cfg {
	u8 mask;
	u8 offset;
};

#define DPNI_EFH_TYPE_SHIFT		0
#define DPNI_EFH_TYPE_SIZE		4
#define DPNI_EXTRACT_TYPE_SHIFT		0
#define DPNI_EXTRACT_TYPE_SIZE		4

struct dpni_dist_extract {
	/* word 0 */
	u8 prot;
	/* EFH type stored in the 4 least significant bits */
	u8 efh_type;
	u8 size;
	u8 offset;
	__le32 field;
	/* word 1 */
	u8 hdr_index;
	u8 constant;
	u8 num_of_repeats;
	u8 num_of_byte_masks;
	/* Extraction type is stored in the 4 LSBs */
	u8 extract_type;
	u8 pad[3];
	/* word 2 */
	struct dpni_mask_cfg masks[4];
};

struct dpni_ext_set_rx_tc_dist {
	/* extension word 0 */
	u8 num_extracts;
	u8 pad[7];
	/* words 1..25 */
	struct dpni_dist_extract extracts[DPKG_MAX_NUM_OF_EXTRACTS];
};

struct dpni_cmd_get_queue {
	u8 qtype;
	u8 tc;
	u8 index;
};

#define DPNI_DEST_TYPE_SHIFT		0
#define DPNI_DEST_TYPE_SIZE		4
#define DPNI_STASH_CTRL_SHIFT		6
#define DPNI_STASH_CTRL_SIZE		1
#define DPNI_HOLD_ACTIVE_SHIFT		7
#define DPNI_HOLD_ACTIVE_SIZE		1

struct dpni_rsp_get_queue {
	/* response word 0 */
	__le64 pad0;
	/* response word 1 */
	__le32 dest_id;
	__le16 pad1;
	u8 dest_prio;
	/* From LSB: dest_type:4, pad:2, flc_stash_ctrl:1, hold_active:1 */
	u8 flags;
	/* response word 2 */
	__le64 flc;
	/* response word 3 */
	__le64 user_context;
	/* response word 4 */
	__le32 fqid;
	__le16 qdbin;
};

struct dpni_cmd_set_queue {
	/* cmd word 0 */
	u8 qtype;
	u8 tc;
	u8 index;
	u8 options;
	__le32 pad0;
	/* cmd word 1 */
	__le32 dest_id;
	__le16 pad1;
	u8 dest_prio;
	u8 flags;
	/* cmd word 2 */
	__le64 flc;
	/* cmd word 3 */
	__le64 user_context;
};

struct dpni_cmd_set_taildrop {
	/* cmd word 0 */
	u8 congestion_point;
	u8 qtype;
	u8 tc;
	u8 index;
	__le32 pad0;
	/* cmd word 1 */
	/* Only least significant bit is relevant */
	u8 enable;
	u8 pad1;
	u8 units;
	u8 pad2;
	__le32 threshold;
};

struct dpni_cmd_get_taildrop {
	u8 congestion_point;
	u8 qtype;
	u8 tc;
	u8 index;
};

struct dpni_rsp_get_taildrop {
	/* cmd word 0 */
	__le64 pad0;
	/* cmd word 1 */
	/* only least significant bit is relevant */
	u8 enable;
	u8 pad1;
	u8 units;
	u8 pad2;
	__le32 threshold;
};

struct dpni_rsp_get_api_version {
	__le16 major;
	__le16 minor;
};

#define DPNI_RX_FS_DIST_ENABLE_SHIFT	0
#define DPNI_RX_FS_DIST_ENABLE_SIZE	1
struct dpni_cmd_set_rx_fs_dist {
	__le16 dist_size;
	u8 enable;
	u8 tc;
	__le16 miss_flow_id;
	__le16 pad;
	__le64 key_cfg_iova;
};

#define DPNI_RX_HASH_DIST_ENABLE_SHIFT	0
#define DPNI_RX_HASH_DIST_ENABLE_SIZE	1
struct dpni_cmd_set_rx_hash_dist {
	__le16 dist_size;
	u8 enable;
	u8 tc;
	__le32 pad;
	__le64 key_cfg_iova;
};

struct dpni_cmd_add_fs_entry {
	/* cmd word 0 */
	__le16 options;
	u8 tc_id;
	u8 key_size;
	__le16 index;
	__le16 flow_id;
	/* cmd word 1 */
	__le64 key_iova;
	/* cmd word 2 */
	__le64 mask_iova;
	/* cmd word 3 */
	__le64 flc;
};

struct dpni_cmd_remove_fs_entry {
	/* cmd word 0 */
	__le16 pad0;
	u8 tc_id;
	u8 key_size;
	__le32 pad1;
	/* cmd word 1 */
	__le64 key_iova;
	/* cmd word 2 */
	__le64 mask_iova;
};

#define DPNI_DISCARD_ON_MISS_SHIFT	0
#define DPNI_DISCARD_ON_MISS_SIZE	1

struct dpni_cmd_set_qos_table {
	__le32 pad;
	u8 default_tc;
	/* only the LSB */
	u8 discard_on_miss;
	__le16 pad1[21];
	__le64 key_cfg_iova;
};

struct dpni_cmd_add_qos_entry {
	__le16 pad;
	u8 tc_id;
	u8 key_size;
	__le16 index;
	__le16 pad1;
	__le64 key_iova;
	__le64 mask_iova;
};

struct dpni_cmd_remove_qos_entry {
	u8 pad[3];
	u8 key_size;
	__le32 pad1;
	__le64 key_iova;
	__le64 mask_iova;
};

#define DPNI_DEST_TYPE_SHIFT		0
#define DPNI_DEST_TYPE_SIZE		4
#define DPNI_CONG_UNITS_SHIFT		4
#define DPNI_CONG_UNITS_SIZE		2

struct dpni_cmd_set_congestion_notification {
	/* cmd word 0 */
	u8 qtype;
	u8 tc;
	u8 pad[6];
	/* cmd word 1 */
	__le32 dest_id;
	__le16 notification_mode;
	u8 dest_priority;
	/* from LSB: dest_type: 4 units:2 */
	u8 type_units;
	/* cmd word 2 */
	__le64 message_iova;
	/* cmd word 3 */
	__le64 message_ctx;
	/* cmd word 4 */
	__le32 threshold_entry;
	__le32 threshold_exit;
};

#define DPNI_COUPLED_SHIFT	0
#define DPNI_COUPLED_SIZE	1

struct dpni_cmd_set_tx_shaping {
	__le16 tx_cr_max_burst_size;
	__le16 tx_er_max_burst_size;
	__le32 pad;
	__le32 tx_cr_rate_limit;
	__le32 tx_er_rate_limit;
	/* from LSB: coupled:1 */
	u8 coupled;
};

#define DPNI_PTP_ENABLE_SHIFT			0
#define DPNI_PTP_ENABLE_SIZE			1
#define DPNI_PTP_CH_UPDATE_SHIFT		1
#define DPNI_PTP_CH_UPDATE_SIZE			1

struct dpni_cmd_single_step_cfg {
	__le16 flags;
	__le16 offset;
	__le32 peer_delay;
	__le32 ptp_onestep_reg_base;
	__le32 pad0;
};

struct dpni_rsp_single_step_cfg {
	__le16 flags;
	__le16 offset;
	__le32 peer_delay;
	__le32 ptp_onestep_reg_base;
	__le32 pad0;
};

struct dpni_cmd_enable_vlan_filter {
	/* only the LSB */
	u8 en;
};

struct dpni_cmd_vlan_id {
	u8 flags;
	u8 tc_id;
	u8 flow_id;
	u8 pad;
	__le16 vlan_id;
};

#define DPNI_MACSEC_SHIFT	0
#define DPNI_MACSEC_SIZE	1

struct dpni_rsp_is_macsec_capable {
	u8 en;
};

#define DPNI_SECY_CIPHER_SUITE_SIZE		1
#define DPNI_SECY_CIPHER_SUITE_SHIFT		0

#define DPNI_SECY_CONFIDENTIALITY_SIZE		1
#define DPNI_SECY_CONFIDENTIALITY_SHIFT		1

#define DPNI_SECY_IS_PTP_SIZE			1
#define DPNI_SECY_IS_PTP_SHIFT			2

#define DPNI_SECY_VALIDATION_MODE_SIZE		2
#define DPNI_SECY_VALIDATION_MODE_SHIFT		3

struct dpni_cmd_add_secy {
	__le64 tx_sci;
	u8 flags;
	u8 co_offset;
	u8 max_rx_sc;
};

struct dpni_rsp_add_secy {
	u8 secy_id;
};

struct dpni_cmd_remove_secy {
	u8 secy_id;
};

#define DPNI_SECY_ACTIVE_SIZE		1
#define DPNI_SECY_ACTIVE_SHIFT		0

struct dpni_cmd_secy_set_state {
	u8 secy_id;
	u8 flags;
};

#define DPNI_SECY_TX_PROTECT_SIZE	1
#define DPNI_SECY_TX_PROTECT_SHIFT	0

struct dpni_cmd_secy_set_protect {
	u8 secy_id;
	u8 flags;
};

#define DPNI_SECY_REPLAY_PROTECT_EN_SIZE	1
#define DPNI_SECY_REPLAY_PROTECT_EN_SHIFT	0

struct dpni_cmd_secy_set_replay_protect {
	u8 secy_id;
	u8 flags;
	__le16 res;
	__le32 replay_window;
};

struct dpni_cmd_secy_add_tx_sa {
	u8 key[32];
	__le32 next_pn;
	u8 secy_id;
	u8 an;
};

struct dpni_cmd_secy_remove_tx_sa {
	u8 secy_id;
	u8 an;
};

struct dpni_cmd_secy_set_tx_sa {
	u8 secy_id;
	u8 an;
};

struct dpni_cmd_secy_rx_sc {
	u8 secy_id;
	u8 res[7];
	__le64 sci;
};

struct dpni_cmd_secy_set_rx_sc_state {
	u8 secy_id;
	u8 en;
	u8 res[6];
	__le64 sci;
};

struct dpni_cmd_secy_add_rx_sa {
	u8 key[32];
	__le32 lowest_pn;
	u8 secy_id;
	u8 an;
	__le16 res;
	__le64 sci;
};

struct dpni_cmd_secy_remove_rx_sa {
	u8 secy_id;
	u8 an;
	u8 res[6];
	__le64 sci;
};

struct dpni_cmd_secy_set_rx_sa_next_pn {
	u8 secy_id;
	u8 an;
	__le16 res;
	__le32 next_pn;
	__le64 sci;
};

struct dpni_cmd_secy_set_rx_sa_state {
	u8 secy_id;
	u8 an;
	u8 en;
	u8 res[5];
	__le64 sci;
};

struct dpni_cmd_secy_get_stats {
	u8 secy_id;
	u8 page;
};

struct dpni_cmd_secy_get_tx_sc_stats {
	u8 secy_id;
};

struct dpni_cmd_secy_get_tx_sa_stats {
	u8 secy_id;
	u8 an;
};

struct dpni_rsp_stats32 {
	__le32 counter[DPNI_STATISTICS_32_CNT];
};

struct dpni_cmd_secy_get_rx_sc_stats {
	u8 secy_id;
	u8 page;
	u8 res[6];
	__le64 sci;
};

struct dpni_cmd_secy_get_rx_sa_stats {
	u8 secy_id;
	u8 an;
	u8 res[6];
	__le64 sci;
};

#endif /* _FSL_DPNI_CMD_H */
