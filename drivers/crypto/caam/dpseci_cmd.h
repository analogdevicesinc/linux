/* SPDX-License-Identifier: (GPL-2.0+ OR BSD-3-Clause) */
/*
 * Copyright 2013-2016 Freescale Semiconductor Inc.
 * Copyright 2017-2018 NXP
 */

#ifndef _DPSECI_CMD_H_
#define _DPSECI_CMD_H_

/* DPSECI Version */
#define DPSECI_VER_MAJOR				5
#define DPSECI_VER_MINOR				3

#define DPSECI_VER(maj, min)	(((maj) << 16) | (min))
#define DPSECI_VERSION		DPSECI_VER(DPSECI_VER_MAJOR, DPSECI_VER_MINOR)

/* Command versioning */
#define DPSECI_CMD_BASE_VERSION		1
#define DPSECI_CMD_BASE_VERSION_V2	2
#define DPSECI_CMD_BASE_VERSION_V3	3
#define DPSECI_CMD_ID_OFFSET		4

#define DPSECI_CMD_V1(id)	(((id) << DPSECI_CMD_ID_OFFSET) | \
				 DPSECI_CMD_BASE_VERSION)

#define DPSECI_CMD_V2(id)	(((id) << DPSECI_CMD_ID_OFFSET) | \
				 DPSECI_CMD_BASE_VERSION_V2)

#define DPSECI_CMD_V3(id)	(((id) << DPSECI_CMD_ID_OFFSET) | \
				 DPSECI_CMD_BASE_VERSION_V3)

/* Command IDs */
#define DPSECI_CMDID_CLOSE				DPSECI_CMD_V1(0x800)
#define DPSECI_CMDID_OPEN				DPSECI_CMD_V1(0x809)
#define DPSECI_CMDID_CREATE				DPSECI_CMD_V3(0x909)
#define DPSECI_CMDID_DESTROY				DPSECI_CMD_V1(0x989)
#define DPSECI_CMDID_GET_API_VERSION			DPSECI_CMD_V1(0xa09)

#define DPSECI_CMDID_ENABLE				DPSECI_CMD_V1(0x002)
#define DPSECI_CMDID_DISABLE				DPSECI_CMD_V1(0x003)
#define DPSECI_CMDID_GET_ATTR				DPSECI_CMD_V1(0x004)
#define DPSECI_CMDID_RESET				DPSECI_CMD_V1(0x005)
#define DPSECI_CMDID_IS_ENABLED				DPSECI_CMD_V1(0x006)

#define DPSECI_CMDID_SET_IRQ_ENABLE			DPSECI_CMD_V1(0x012)
#define DPSECI_CMDID_GET_IRQ_ENABLE			DPSECI_CMD_V1(0x013)
#define DPSECI_CMDID_SET_IRQ_MASK			DPSECI_CMD_V1(0x014)
#define DPSECI_CMDID_GET_IRQ_MASK			DPSECI_CMD_V1(0x015)
#define DPSECI_CMDID_GET_IRQ_STATUS			DPSECI_CMD_V1(0x016)
#define DPSECI_CMDID_CLEAR_IRQ_STATUS			DPSECI_CMD_V1(0x017)

#define DPSECI_CMDID_SET_RX_QUEUE			DPSECI_CMD_V1(0x194)
#define DPSECI_CMDID_GET_RX_QUEUE			DPSECI_CMD_V1(0x196)
#define DPSECI_CMDID_GET_TX_QUEUE			DPSECI_CMD_V1(0x197)
#define DPSECI_CMDID_GET_SEC_ATTR			DPSECI_CMD_V2(0x198)
#define DPSECI_CMDID_GET_SEC_COUNTERS			DPSECI_CMD_V1(0x199)
#define DPSECI_CMDID_SET_OPR				DPSECI_CMD_V1(0x19A)
#define DPSECI_CMDID_GET_OPR				DPSECI_CMD_V1(0x19B)
#define DPSECI_CMDID_SET_CONGESTION_NOTIFICATION	DPSECI_CMD_V1(0x170)
#define DPSECI_CMDID_GET_CONGESTION_NOTIFICATION	DPSECI_CMD_V1(0x171)

/* Macros for accessing command fields smaller than 1 byte */
#define DPSECI_MASK(field)	\
	GENMASK(DPSECI_##field##_SHIFT + DPSECI_##field##_SIZE - 1,	\
		DPSECI_##field##_SHIFT)

#define dpseci_set_field(var, field, val)	\
	((var) |= (((val) << DPSECI_##field##_SHIFT) & DPSECI_MASK(field)))

#define dpseci_get_field(var, field)	\
	(((var) & DPSECI_MASK(field)) >> DPSECI_##field##_SHIFT)

struct dpseci_cmd_open {
	__le32 dpseci_id;
};

struct dpseci_cmd_create {
	u8 priorities[8];
	u8 num_tx_queues;
	u8 num_rx_queues;
	u8 pad0[6];
	__le32 options;
	__le32 pad1;
	u8 priorities2[8];
};

struct dpseci_cmd_destroy {
	__le32 object_id;
};

#define DPSECI_ENABLE_SHIFT	0
#define DPSECI_ENABLE_SIZE	1

struct dpseci_rsp_is_enabled {
	u8 is_enabled;
};

struct dpseci_cmd_irq_enable {
	u8 enable_state;
	u8 pad[3];
	u8 irq_index;
};

struct dpseci_rsp_get_irq_enable {
	u8 enable_state;
};

struct dpseci_cmd_irq_mask {
	__le32 mask;
	u8 irq_index;
};

struct dpseci_cmd_irq_status {
	__le32 status;
	u8 irq_index;
};

struct dpseci_rsp_get_attributes {
	__le32 id;
	__le32 pad0;
	u8 num_tx_queues;
	u8 num_rx_queues;
	u8 pad1[6];
	__le32 options;
};

#define DPSECI_DEST_TYPE_SHIFT	0
#define DPSECI_DEST_TYPE_SIZE	4

#define DPSECI_ORDER_PRESERVATION_SHIFT	0
#define DPSECI_ORDER_PRESERVATION_SIZE	1

struct dpseci_cmd_queue {
	__le32 dest_id;
	u8 priority;
	u8 queue;
	u8 dest_type;
	u8 pad;
	__le64 user_ctx;
	union {
		__le32 options;
		__le32 fqid;
	};
	u8 order_preservation_en;
};

struct dpseci_rsp_get_tx_queue {
	__le32 pad;
	__le32 fqid;
	u8 priority;
};

struct dpseci_rsp_get_sec_attr {
	__le16 ip_id;
	u8 major_rev;
	u8 minor_rev;
	u8 era;
	u8 pad0[3];
	u8 deco_num;
	u8 zuc_auth_acc_num;
	u8 zuc_enc_acc_num;
	u8 pad1;
	u8 snow_f8_acc_num;
	u8 snow_f9_acc_num;
	u8 crc_acc_num;
	u8 pad2;
	u8 pk_acc_num;
	u8 kasumi_acc_num;
	u8 rng_acc_num;
	u8 pad3;
	u8 md_acc_num;
	u8 arc4_acc_num;
	u8 des_acc_num;
	u8 aes_acc_num;
	u8 ccha_acc_num;
	u8 ptha_acc_num;
};

struct dpseci_rsp_get_sec_counters {
	__le64 dequeued_requests;
	__le64 ob_enc_requests;
	__le64 ib_dec_requests;
	__le64 ob_enc_bytes;
	__le64 ob_prot_bytes;
	__le64 ib_dec_bytes;
	__le64 ib_valid_bytes;
};

struct dpseci_rsp_get_api_version {
	__le16 major;
	__le16 minor;
};

struct dpseci_cmd_opr {
	__le16 pad;
	u8 index;
	u8 options;
	u8 pad1[7];
	u8 oloe;
	u8 oeane;
	u8 olws;
	u8 oa;
	u8 oprrws;
};

#define DPSECI_OPR_RIP_SHIFT		0
#define DPSECI_OPR_RIP_SIZE		1
#define DPSECI_OPR_ENABLE_SHIFT		1
#define DPSECI_OPR_ENABLE_SIZE		1
#define DPSECI_OPR_TSEQ_NLIS_SHIFT	0
#define DPSECI_OPR_TSEQ_NLIS_SIZE	1
#define DPSECI_OPR_HSEQ_NLIS_SHIFT	0
#define DPSECI_OPR_HSEQ_NLIS_SIZE	1

struct dpseci_rsp_get_opr {
	__le64 pad;
	u8 flags;
	u8 pad0[2];
	u8 oloe;
	u8 oeane;
	u8 olws;
	u8 oa;
	u8 oprrws;
	__le16 nesn;
	__le16 pad1;
	__le16 ndsn;
	__le16 pad2;
	__le16 ea_tseq;
	u8 tseq_nlis;
	u8 pad3;
	__le16 ea_hseq;
	u8 hseq_nlis;
	u8 pad4;
	__le16 ea_hptr;
	__le16 pad5;
	__le16 ea_tptr;
	__le16 pad6;
	__le16 opr_vid;
	__le16 pad7;
	__le16 opr_id;
};

#define DPSECI_CGN_DEST_TYPE_SHIFT	0
#define DPSECI_CGN_DEST_TYPE_SIZE	4
#define DPSECI_CGN_UNITS_SHIFT		4
#define DPSECI_CGN_UNITS_SIZE		2

struct dpseci_cmd_congestion_notification {
	__le32 dest_id;
	__le16 notification_mode;
	u8 priority;
	u8 options;
	__le64 message_iova;
	__le64 message_ctx;
	__le32 threshold_entry;
	__le32 threshold_exit;
};

#endif /* _DPSECI_CMD_H_ */
