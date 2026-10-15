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
 * struct btintel_pcie_yoyo_ctx- Runtime state for YoYo debug configuration
 * @enabled: Whether the loaded YoYo configuration passed validation
 * @region_list: Parsed debug-region entries
 * @trigger_list: Parsed debug-trigger entries
 * @hcmd_list: Parsed HCI command entries
 * @config_set_list: Parsed configuration-set entries
 * @hdev: Associated HCI device; NULL until successful HCI registration
 *
 * Stores the validation state and parsed TLV data for the optional YoYo
 * firmware configuration.
 */
struct btintel_pcie_yoyo_ctx {
	bool			 enabled;

	struct list_head	 region_list;
	struct list_head	 trigger_list;
	struct list_head	 hcmd_list;
	struct list_head	 config_set_list;

	struct hci_dev		 *hdev;
};

int  btintel_pcie_yoyo_init(struct btintel_pcie_yoyo_ctx *ctx);
void btintel_pcie_yoyo_cleanup(struct btintel_pcie_yoyo_ctx *ctx);

/* Load and parse the external .bin file */
int  btintel_pcie_yoyo_load_and_parse(struct btintel_pcie_yoyo_ctx *ctx,
				      struct device *dev);

#endif /* __BTINTEL_PCIE_DEBUG_H__ */
