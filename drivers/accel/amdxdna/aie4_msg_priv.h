/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2026, Advanced Micro Devices, Inc.
 */

#ifndef _AIE4_MSG_PRIV_H_
#define _AIE4_MSG_PRIV_H_

#include <linux/sizes.h>
#include <linux/types.h>

enum aie4_msg_opcode {
	AIE4_MSG_OP_IDENTIFY                         = 0x10002,
	AIE4_MSG_OP_SUSPEND                          = 0x10003,
	AIE4_MSG_OP_SET_RUNTIME_CONFIG               = 0x10007,
	AIE4_MSG_OP_QUERY_CERT_FIRMWARE_VERSION      = 0x1000F,

	AIE4_MSG_OP_CREATE_VFS                       = 0x20001,
	AIE4_MSG_OP_DESTROY_VFS                      = 0x20002,

	AIE4_MSG_OP_CREATE_PARTITION                 = 0x30001,
	AIE4_MSG_OP_DESTROY_PARTITION                = 0x30002,
	AIE4_MSG_OP_CREATE_HW_CONTEXT                = 0x30003,
	AIE4_MSG_OP_DESTROY_HW_CONTEXT               = 0x30004,
	AIE4_MSG_OP_AIE_TILE_INFO                    = 0x30006,
	AIE4_MSG_OP_AIE_VERSION_INFO                 = 0x30007,
	AIE4_MSG_OP_POWER_OVERRIDE                   = 0x3000B,
	AIE4_MSG_OP_GET_DPM_FREQ_TABLE               = 0x30012,
	AIE4_MSG_OP_GET_CURRENT_DPM_LEVEL            = 0x30013,

	AIE4_MSG_OP_ATTACH_WORK_BUFFER               = 0x40001,
	AIE4_MSG_OP_START_FW_LOG                     = 0x40003,
};

enum aie4_msg_status {
	AIE4_MSG_STATUS_SUCCESS = 0x0,
	AIE4_MSG_STATUS_ERROR = 0x1,
	AIE4_MSG_STATUS_NOTSUPP = 0x2,
	MAX_AIE4_MSG_STATUS_CODE = 0x4,
};

enum aie4_msg_context_priority_band {
	AIE4_CONTEXT_PRIORITY_BAND_IDLE = 0,
	AIE4_CONTEXT_PRIORITY_BAND_NORMAL,
	AIE4_CONTEXT_PRIORITY_BAND_FOCUS,
	AIE4_CONTEXT_PRIORITY_BAND_REAL_TIME,
	AIE4_CONTEXT_PRIORITY_BAND_COUNT
};

struct aie4_msg_identify_req {
	__u32 rsvd;
} __packed;

struct aie4_msg_identify_resp {
	enum aie4_msg_status status;
	__u32 fw_major;
	__u32 fw_minor;
	__u32 fw_patch;
	__u32 fw_build;
} __packed;

struct aie4_msg_suspend_req {
	__u32 rsvd;
} __packed;

struct aie4_msg_suspend_resp {
	enum aie4_msg_status status;
} __packed;

/* Type selector for AIE4_MSG_OP_SET_RUNTIME_CONFIG. */
enum aie4_msg_runtime_config_type {
	AIE4_RUNTIME_CONFIG_CTX_SWITCH_HYSTERESIS	= 0xD,
	AIE4_MAX_RUNTIME_CONFIG
};

struct aie4_msg_set_runtime_cfg_req {
	__u32 type;
	__u8 data[4];
} __packed;

struct aie4_msg_set_runtime_cfg_resp {
	enum aie4_msg_status status;
} __packed;

/* Maximum payload for runtime config firmware message. */
#define AIE4_RUNTIME_CFG_MAX_DATA_SIZE 16

/* Context switch hysteresis timeout in microseconds (0 disables). */
struct aie4_msg_runtime_config_ctx_switch_hysteresis {
	__u32 timeout_us;
} __packed;

struct aie4_msg_create_vfs_req {
	__u32 vf_cnt;
} __packed;

struct aie4_msg_create_vfs_resp {
	enum aie4_msg_status status;
} __packed;

struct aie4_msg_destroy_vfs_req {
	__u32 rsvd;
} __packed;

struct aie4_msg_destroy_vfs_resp {
	enum aie4_msg_status status;
} __packed;

struct aie4_msg_create_partition_req {
	__u32 partition_col_start;
	__u32 partition_col_count;
} __packed;

struct aie4_msg_create_partition_resp {
	enum aie4_msg_status status;
	__u32 partition_id;
} __packed;

struct aie4_msg_destroy_partition_req {
	__u32 partition_id;
} __packed;

struct aie4_msg_destroy_partition_resp {
	enum aie4_msg_status status;
} __packed;

struct aie4_msg_create_hw_context_req {
	__u32 partition_id;
	__u32 request_num_tiles;
	__u32 hsa_addr_high;
	__u32 hsa_addr_low;
#define AIE4_MSG_PASID GENMASK(19, 0)
#define AIE4_MSG_PASID_VLD GENMASK(31, 31)
	__u32 pasid;
	__u8 priority_band;
	__u8 priority_level;
	__u16 restore_id;
} __packed;

struct aie4_msg_create_hw_context_resp {
	enum aie4_msg_status status;
	__u32 hw_context_id;
	__u32 doorbell_offset;
	__u32 job_complete_msix_idx;
} __packed;

struct aie4_msg_destroy_hw_context_req {
	__u32 hw_context_id;
#define AIE4_MSG_GRACEFUL_FLAG GENMASK(0, 0)
	__u32 graceful_flag;
} __packed;

struct aie4_msg_destroy_hw_context_resp {
	enum aie4_msg_status status;
	__u16 restore_id;
	__u16 resvd;
} __packed;

struct aie4_tile_info {
	__u32 size;
	__u16 major;
	__u16 minor;
	__u16 cols;
	__u16 rows;
	__u16 core_rows;
	__u16 mem_rows;
	__u16 shim_rows;
	__u16 core_row_start;
	__u16 mem_row_start;
	__u16 shim_row_start;
	__u16 core_dma_channels;
	__u16 mem_dma_channels;
	__u16 shim_dma_channels;
	__u16 core_locks;
	__u16 mem_locks;
	__u16 shim_locks;
	__u16 core_events;
	__u16 mem_events;
	__u16 shim_events;
	__u16 resvd;
} __packed;

struct aie4_msg_aie4_tile_info_req {
	__u32 resvd;
} __packed;

struct aie4_msg_aie4_tile_info_resp {
	enum aie4_msg_status status;
	struct aie4_tile_info info;
} __packed;

struct aie4_msg_aie4_version_info_req {
	__u32 resvd;
} __packed;

struct aie4_msg_aie4_version_info_resp {
	enum aie4_msg_status status;
	__u16 major;
	__u16 minor;
} __packed;

struct aie4_msg_query_cert_firmware_version_req {
	__u32 resvd;
} __packed;

struct aie4_msg_query_cert_firmware_version_resp {
	enum aie4_msg_status status;
	__u8 major_version;
	__u8 minor_version;
	__u8 git_hash[41];
	__u8 date[11];
	__u8 hotfix;
	__u8 build;
	__u16 host_queue_major;
	__u16 host_queue_minor;
} __packed;

struct aie4_msg_power_override_req {
	__u32 power_mode;
} __packed;

struct aie4_msg_power_override_resp {
	enum aie4_msg_status status;
} __packed;

#define AIE4_MAX_DPM_LEVEL_COUNT	10

struct aie4_dpm_table {
	__u32 num_levels;
	__u32 values[AIE4_MAX_DPM_LEVEL_COUNT];
} __packed;

/* AIE4_MSG_OP_GET_DPM_FREQ_TABLE */
struct aie4_msg_get_dpm_freq_table_req {
	__u32 rsvd;
} __packed;

struct aie4_msg_get_dpm_freq_table_resp {
	enum aie4_msg_status status;
	struct aie4_dpm_table aieclk_table;
	struct aie4_dpm_table npuhclk_table;
} __packed;

/* AIE4_MSG_OP_GET_CURRENT_DPM_LEVEL */
struct aie4_msg_get_dpm_level_req {
	__u32 rsvd;
} __packed;

struct aie4_msg_get_dpm_level_resp {
	enum aie4_msg_status status;
	__u32 aieclk_dpm_level;
	__u32 npuhclk_dpm_level;
} __packed;

#define AIE4_WORK_BUFFER_MIN_SIZE      SZ_4M

struct aie4_msg_attach_work_buffer_req {
	__u64 buff_addr;
	__u32 reserved;
	__u32 buff_size;
} __packed;

struct aie4_msg_attach_work_buffer_resp {
	enum aie4_msg_status status;
} __packed;

/* Dynamic firmware log levels. */
enum aie4_fw_log_level {
	AIE4_FW_LOG_LEVEL_OFF,
	AIE4_FW_LOG_LEVEL_ERR,
	AIE4_FW_LOG_LEVEL_WRN,
	AIE4_FW_LOG_LEVEL_INF,
	AIE4_FW_LOG_LEVEL_DBG,
	AIE4_FW_LOG_LEVEL_MAX,
};

#define AIE4_FW_LOG_BUF_SIZE      SZ_1M

struct aie4_msg_start_fw_log_req {
	__u64 buff_addr;
	__u32 buff_size;
	__u32 log_level;
	__u32 reserved;
} __packed;

struct aie4_msg_start_fw_log_resp {
	enum aie4_msg_status status;
} __packed;

#endif /* _AIE4_MSG_PRIV_H_ */
