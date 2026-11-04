/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Intel Bluetooth YoYo Debug Framework
 *
 * Copyright (C) 2026 Intel Corporation
 *
 */

#ifndef __BTINTEL_PCIE_DEBUG_H__
#define __BTINTEL_PCIE_DEBUG_H__

#include <linux/list.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/firmware.h>

/* External YoYo binary file */
#define BTINTEL_PCIE_YOYO_FW_NAME	"intel/ibt_debug_yoyo.bin"

/* TLV type codes */
#define BTINTEL_PCIE_TLV_TYPE_DBG_INFO		0x1000005
#define BTINTEL_PCIE_TLV_TYPE_DBG_BUF_ALLOC		0x1000006
#define BTINTEL_PCIE_TLV_TYPE_DBG_HCMD		0x1000007
#define BTINTEL_PCIE_TLV_TYPE_DBG_REGION		0x1000008
#define BTINTEL_PCIE_TLV_TYPE_DBG_TRIGGER		0x1000009
#define BTINTEL_PCIE_TLV_TYPE_DBG_CONFIG_SET		0x100000A
#define BTINTEL_PCIE_TLV_TYPE_DBG_PRESET_OVERRIDE	0x100000B

/* Supported TLV type versions */
#define BTINTEL_PCIE_DBG_INFO_TLV_VERSION	2
#define BTINTEL_PCIE_BUF_ALLOC_TLV_VERSION	1
/* Debug Info TLV */
#define BTINTEL_PCIE_CONFIG_NAME_LEN		64
#define BTINTEL_PCIE_DOMAIN_BT			0
#define BTINTEL_PCIE_CORE_TYPE_BT		1
/* Buffer allocation TLV */
#define BTINTEL_PCIE_MAX_BUF_ALLOC		3
#define BTINTEL_PCIE_MAX_BUF_FRAGS		16
#define BTINTEL_PCIE_MIN_BUF_SIZE		(32 * 1024)
#define BTINTEL_PCIE_MAX_BUF_SIZE		(4 * 1024 * 1024)

enum btintel_pcie_image_tlv_type {
	BTINTEL_PCIE_IMAGE_TLV_TYPE_EXTERNAL	= 0x1,
	BTINTEL_PCIE_IMAGE_TLV_TYPE_INTERNAL	= 0x2,
	BTINTEL_PCIE_IMAGE_TLV_TYPE_UNKNOWN	= 0xff,
};

/* Allocation IDs */
enum btintel_dbg_alloc_id {
	BTINTEL_PCIE_ALLOC_ID_INVALID	= 0,
	BTINTEL_PCIE_ALLOC_ID_DBGC1		= 1,
	BTINTEL_PCIE_ALLOC_ID_DBGC2		= 2,
	BTINTEL_PCIE_ALLOC_ID_DBGC3		= 3,
	BTINTEL_PCIE_ALLOC_ID_MAX,
};

/* Buffer locations */
enum btintel_dbg_buf_location {
	BTINTEL_PCIE_BUF_LOC_INVALID		= 0,
	BTINTEL_PCIE_BUF_LOC_SRAM		= 1,
	BTINTEL_PCIE_BUF_LOC_DRAM		= 2,
	BTINTEL_PCIE_BUF_LOC_NPK		= 3,
	BTINTEL_PCIE_BUF_LOC_MAX,
};

/* TLV header */
struct btintel_pcie_tlv_hdr {
	__le32 type;
	__le32 length;
} __packed;

/* Debug header common to all debug TLVs */
struct btintel_pcie_dbg_hdr {
	__le32 tlv_data_version;
	__le32 domain;
} __packed;

/* Address/value pair */
struct btintel_pcie_addr_value {
	__le32 address;
	__le32 value;
} __packed;

/*
 * struct btintel_pcie_tlv_dbg_info - YoYo DEBUG_INFO TLV
 * @hdr: TLV type and payload length
 * @dbg_hdr: Debug TLV version and domain
 * @image_type: Firmware image type
 * @core_type: Firmware core type
 * @config_name: Fixed-size name of the firmware configuration
 *
 * Describes the firmware image targeted by the YoYo configuration. Fields
 * are stored in little-endian format in the packed firmware TLV.
 */
struct btintel_pcie_tlv_dbg_info {
	struct btintel_pcie_tlv_hdr hdr;
	struct btintel_pcie_dbg_hdr dbg_hdr;
	__le32 image_type;
	__le32 core_type;
	u8 config_name[BTINTEL_PCIE_CONFIG_NAME_LEN];
} __packed;

/*
 * struct btintel_pcie_tlv_buf_alloc - YoYo buffer allocation TLV
 * @hdr: Common TLV header
 * @dbg_hdr: Debug TLV version and domain
 * @alloc_id: DBGC allocation identifier (DBGC1..DBGC3)
 * @buf_location: Requested buffer location (SRAM / DRAM / NPK)
 * @req_buf_size: Total requested allocation size in bytes
 * @max_num_frags: Maximum number of buffer fragments (1..16)
 * @min_buf_size: Minimum buffer size in bytes
 *
 * Describes the buffer sizing override for one DBGC allocation.
 */
struct btintel_pcie_tlv_buf_alloc {
	struct btintel_pcie_tlv_hdr hdr;
	struct btintel_pcie_dbg_hdr dbg_hdr;
	__le32 alloc_id;
	__le32 buf_location;
	__le32 req_buf_size;
	__le32 max_num_frags;
	__le32 min_buf_size;
} __packed;

/* Parsed buffer allocation entry in the YoYo context */
struct btintel_pcie_buf_alloc {
	u32	alloc_id;
	u32	buf_location;
	u32	req_buf_size;
	u32	max_num_frags;
	u32	min_buf_size;
	bool	present;
};

/*
 * struct btintel_pcie_yoyo_ctx- Runtime state for YoYo debug configuration
 * @enabled: Whether the loaded YoYo configuration passed validation
 * @region_list: Parsed debug-region entries
 * @trigger_list: Parsed debug-trigger entries
 * @hcmd_list: Parsed HCI command entries
 * @config_set_list: Parsed configuration-set entries
 * @buf_alloc: Parsed buffer allocation entries
 * @num_buf_allocs: Number of parsed buffer allocation entries
 * @hdev: Associated HCI device; NULL until successful HCI registration
 *
 * Stores the validation state and parsed TLV data for the optional YoYo
 * firmware configuration.
 */
struct btintel_pcie_yoyo_ctx {
	bool			 enabled;
	bool			 dbg_info_present;
	struct list_head	 region_list;
	struct list_head	 trigger_list;
	struct list_head	 hcmd_list;
	struct list_head	 config_set_list;
	struct btintel_pcie_buf_alloc buf_alloc[BTINTEL_PCIE_MAX_BUF_ALLOC];
	int			 num_buf_allocs;
	struct hci_dev		 *hdev;
};

int  btintel_pcie_yoyo_init(struct btintel_pcie_yoyo_ctx *ctx);
void btintel_pcie_yoyo_cleanup(struct btintel_pcie_yoyo_ctx *ctx);

/* Load and parse the external .bin file */
int  btintel_pcie_yoyo_load_and_parse(struct btintel_pcie_yoyo_ctx *ctx,
				      struct device *dev);

const struct btintel_pcie_buf_alloc *
btintel_pcie_yoyo_get_buf_alloc(struct btintel_pcie_yoyo_ctx *ctx, u32 alloc_id);

#endif /* __BTINTEL_PCIE_DEBUG_H__ */
