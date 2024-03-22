/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/* Copyright 2024 NXP */
#include <linux/crc-itu-t.h>

#define ENETC_CRC_INIT		0xffff
#define ENETC_MSG_ALIGN		32

#define ENETC_MSG_EXT_BODY_LEN(l)	((l) / ENETC_MSG_ALIGN - 1)
#define ENETC_MSG_SIZE(l)		(((l) + 1) * ENETC_MSG_ALIGN)

/* Common Class ID for PSI-TO-VSI and VSI-TO-PSI messages */
#define ENETC_MSG_CLASS_ID_MAC_FILTER		0x20
#define ENETC_MSG_CLASS_ID_VLAN_FILTER		0x21
#define ENETC_MSG_CLASS_ID_LINK_STATUS		0x80
#define ENETC_MSG_CLASS_ID_LINK_SPEED		0x81

/* Class ID for PSI-TO-VSI messages */
#define ENETC_MSG_CLASS_ID_CMD_SUCCESS		0x1
#define ENETC_MSG_CLASS_ID_PERMISSION_DENY	0x2
#define ENETC_MSG_CLASS_ID_CMD_NOT_SUPPORT	0x3
#define ENETC_MSG_CLASS_ID_PSI_BUSY		0x4
#define ENETC_MSG_CLASS_ID_CRC_ERROR		0x5
#define ENETC_MSG_CLASS_ID_PROTO_NOT_SUPPORT	0x6
#define ENETC_MSG_CLASS_ID_INVALID_MSG_LEN	0x7
#define ENETC_MSG_CLASS_ID_CMD_TIMEOUT		0x8
#define ENETC_MSG_CLASS_ID_CMD_DEFERED		0xf

/* Class-specific error return codes for MAC filter */
#define ENETC_PF_RC_MAC_FILTER_INVALID_MAC		0x0
#define ENETC_PF_RC_MAC_FILTER_DUPLICATE_MAC		0x1
#define ENETC_PF_RC_MAC_FILTER_MAC_NOT_FOUND		0x2
#define ENETC_PF_RC_MAC_FILTER_NO_RESOURCE		0x3

/* Class-specific error return codes for VLAN filter */
#define ENETC_PF_RC_VLAN_FILTER_INVALID_VLAN		0x0
#define ENETC_PF_RC_VLAN_FILTER_DUPLICATE_VLAN		0x1
#define ENETC_PF_RC_VLAN_FILTER_VLAN_NOT_FOUND		0x2
#define ENETC_PF_RC_VLAN_FILTER_NO_RESOURCE		0x3

/* Class-specific notification codes for link status */
#define ENETC_PF_NC_LINK_STATUS_UP			0x0
#define ENETC_PF_NC_LINK_STATUS_DOWN			0x1

#define ENETC_MAC_FILTER_TYPE_UC	BIT(0)
#define ENETC_MAC_FILTER_TYPE_MC	BIT(1)
#define ENETC_MAC_FILTER_TYPE_ALL	(ENETC_MAC_FILTER_TYPE_UC | \
					 ENETC_MAC_FILTER_TYPE_MC)
#define ENETC_MAC_PROMISC_MODE_DISABLE	0
#define ENETC_MAC_PROMISC_MODE_ENABLE	1
#define ENETC_MAC_FILTER_FLUSH		1
#define ENETC_MAC_HASH_TABLE_SIZE_64	0
#define ENETC_MAC_HASH_TABLE_SIZE_128	1
#define ENETC_MAC_HASH_TABLE_SIZE_256	2
#define ENETC_MAC_HASH_TABLE_SIZE_512	3

#define ENETC_VLAN_CTAG_TPID		0
#define ENETC_VLAN_STAG_TPID		1
#define ENETC_VLAN_HASH_TABLE_SIZE_64	0
#define ENETC_VLAN_HASH_TABLE_SIZE_128	1
#define ENETC_VLAN_HASH_TABLE_SIZE_256	2
#define ENETC_VLAN_HASH_TABLE_SIZE_512	3
#define ENETC_VLAN_PROMISC_MODE_DISABLE	0
#define ENETC_VLAN_PROMISC_MODE_ENABLE	1

enum enetc_msg_mac_filter_cmd_id {
	ENETC_MSG_SET_PRIMARY_MAC,
	ENETC_MSG_ADD_EXACT_MAC_ENTRIES,
	ENETC_MSG_DEL_EXACT_MAC_ENTRIES,
	ENETC_MSG_SET_MAC_HASH_TABLE,
	ENETC_MSG_FLUSH_MAC_ENTRIES,
	ENETC_MSG_SET_MAC_PROMISC_MODE,
};

enum enetc_msg_vlan_filter_cmd_id {
	ENETC_MSG_ADD_EXACT_VLAN_ENTRIES,
	ENETC_MSG_DEL_EXACT_VLAN_ENTRIES,
	ENETC_MSG_SET_VLAN_HASH_TABLE,
	ENETC_MSG_FLUSH_VLAN_ENTRIES,
	ENETC_MSG_SET_VLAN_PROMISC_MODE,
};

enum enetc_msg_link_status_cmd_id {
	ENETC_MSG_GET_CURRENT_LINK_STATUS,
	ENETC_MSG_REGISTER_LINK_CHANGE_NOTIFY,
	ENETC_MSG_UNREGISTER_LINK_CHANGE_NOTIFY,
};

enum enetc_msg_link_speed_cmd_id {
	ENETC_MSG_GET_CURRENT_LINK_SPEED,
	ENETC_MSG_REGISTER_SPEED_CHANGE_NOTIFY,
	ENETC_MSG_UNREGISTER_SPEED_CHANGE_NOTIFY,
};

enum enetc_msg_link_speed_val {
	ENETC_MSG_SPEED_UNKNOWN,
	ENETC_MSG_SPEED_10M_HD,
	ENETC_MSG_SPEED_10M_FD,
	ENETC_MSG_SPEED_100M_HD,
	ENETC_MSG_SPEED_100M_FD,
	ENETC_MSG_SPEED_1000M,
	ENETC_MSG_SPEED_2500M,
	ENETC_MSG_SPEED_5G,
	ENETC_MSG_SPEED_10G,
	ENETC_MSG_SPEED_25G,
	ENETC_MSG_SPEED_50G,
	ENETC_MSG_SPEED_100G,
};

struct enetc_msg_swbd {
	void *vaddr;
	dma_addr_t dma;
	int size;
};

/* The format of PSI-TO-VSI message, only a 16-bits code */
union enetc_pf_msg {
	struct {
		u8 cookie:4;
		u8 class_code:4;
		u8 class_id;
	};
	u16 code;
};

/* The formats of VSI-TO-PSI messages */
#pragma pack(1)

struct enetc_msg_header {
	__be16 crc16;
	u8 class_id;
	u8 cmd_id;
	u8 proto_ver;
	u8 len;
	u8 resv0;
	u8 cookie:4;
	u8 resv1:4;
	u8 resv2[8];
};

/* message format of class_id 0x20, cmd_id: 0x0~0x2
 * cmd_id 0: set primary MAC
 * cmd_id 1: add MAC address filter table entries
 * cmd_id 2: delete MAC address filter table entries
 */
struct enetc_mac_entry {
	u8 addr[ETH_ALEN];
};

struct enetc_msg_mac_exact_filter {
	struct enetc_msg_header hdr;
	u8 mac_cnt;
	u8 resv[3];
	struct enetc_mac_entry mac[];
};

/* message format of class_id 0x20, cmd_id: 0x3, set MAC hash table */
struct enetc_msg_mac_hash_filter {
	struct enetc_msg_header hdr;
	u8 size:6;
	u8 type:2;
	u32 hash_tbl[];
};

/* message format of class_id 0x20, cmd_id: 0x4, flush MAC address
 * filter table entries
 */
struct enetc_msg_mac_filter_flush {
	struct enetc_msg_header hdr;
	u8 resv:6;
	u8 type:2;
};

/* message format of class_id 0x20, cmd_id: 0x5, set MAC promiscuous mode */
struct enetc_msg_mac_promsic_mode {
	struct enetc_msg_header hdr;
	u8 flush_macs:1;
	u8 promisc_mode:1;
	u8 resv:4;
	u8 type:2;
};

/* message format of class_id 0x21, cmd_id: 0x0~0x1
 * cmd_id 0: add VLAN address filter table entries
 * cmd_id 1: delete VLAN address filter table entries
 */
struct enetc_vlan_entry {
	u16 vid:12;
	u16 resv0:4;
	u8 tpid:2;
	u8 resv1:6;
	u8 resv2;
};

struct enetc_msg_vlan_exact_filter {
	struct enetc_msg_header hdr;
	u8 vlan_cnt;
	u8 resv[3];
	struct enetc_vlan_entry vlan[];
};

/* message format of class_id 0x21, cmd_id: 0x2, set VLAN hash table */
struct enetc_msg_vlan_hash_filter {
	struct enetc_msg_header hdr;
	u8 size;
	u8 resv[3];
	u32 hash_tbl[];
};

/* message format of class_id 0x21, cmd_id: 0x3, flush VLAN address
 * filter table entries
 */
struct enetc_msg_vlan_filter_flush {
	struct enetc_msg_header hdr;
};

/* message format of class_id 0x21, cmd_id: 0x4, set VLAN promiscuous mode */
struct enetc_msg_vlan_promsic_mode {
	struct enetc_msg_header hdr;
	u8 flush_vlans:1;
	u8 promisc_mode:1;
	u8 resv:6;
};

/* message format of class_id 0x80, cmd_id 0x0~0x2
 * cmd_id 0x0: get the current link status
 * cmd_id 0x1: register to link status change notification
 * cmd_id 0x2: unregister from link status change notification
 */
struct enetc_msg_link_status {
	struct enetc_msg_header hdr;
};

/* message format of class_id 0x80, cmd_id 0x0~0x2
 * cmd_id 0x0: get the current link speed
 * cmd_id 0x1: register to link speed change notification
 * cmd_id 0x2: unregister from link speed change notification
 */
struct enetc_msg_link_speed {
	struct enetc_msg_header hdr;
};

#pragma pack()
