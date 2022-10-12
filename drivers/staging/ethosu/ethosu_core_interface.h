/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (c) 2022 Arm Limited.
 */

#ifndef ETHOSU_CORE_INTERFACE_H
#define ETHOSU_CORE_INTERFACE_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#endif

#ifdef __cplusplus
namespace EthosU {
#endif

/** Maximum number of IFM/OFM buffers per inference */
#define ETHOSU_CORE_BUFFER_MAX 4
/** Maximum number of dimensions for input and output */
#define ETHOSU_CORE_DIM_MAX 4

/** Maximum number of PMU counters to be returned for inference */
#define ETHOSU_CORE_PMU_MAX 4

#define ETHOSU_CORE_MSG_MAGIC 0x41457631
#define ETHOSU_CORE_MSG_VERSION_MAJOR 0
#define ETHOSU_CORE_MSG_VERSION_MINOR 2
#define ETHOSU_CORE_MSG_VERSION_PATCH 0

#define ETHOSU_CORE_INFERENCE_MODEL 0
#define ETHOSU_CORE_INFERENCE_OP    1

/**
 * enum ethosu_core_msg_type - Message types
 *
 * Types for the messages sent between the host and the core subsystem.
 */
enum ethosu_core_msg_type {
	ETHOSU_CORE_MSG_ERR = 1,
	ETHOSU_CORE_MSG_PING,
	ETHOSU_CORE_MSG_PONG,
	ETHOSU_CORE_MSG_INFERENCE_REQ,
	ETHOSU_CORE_MSG_INFERENCE_RSP,
	ETHOSU_CORE_MSG_VERSION_REQ,
	ETHOSU_CORE_MSG_VERSION_RSP,
	ETHOSU_CORE_MSG_CAPABILITIES_REQ,
	ETHOSU_CORE_MSG_CAPABILITIES_RSP,
	ETHOSU_CORE_MSG_NETWORK_INFO_REQ,
	ETHOSU_CORE_MSG_NETWORK_INFO_RSP,
	ETHOSU_CORE_MSG_CANCEL_INFERENCE_REQ,
	ETHOSU_CORE_MSG_CANCEL_INFERENCE_RSP,
	ETHOSU_CORE_MSG_POWER_REQ,
	ETHOSU_CORE_MSG_POWER_RSP,
	ETHOSU_CORE_MSG_MAX
};

/**
 * struct ethosu_core_msg - Message header
 */
struct ethosu_core_msg {
	uint32_t magic;
	uint32_t type;
	uint32_t length;
};

/**
 * struct ethosu_core_queue_header - Message queue header
 */
struct ethosu_core_queue_header {
	uint32_t size;
	uint32_t read;
	uint32_t write;
};

/**
 * struct ethosu_core_queue - Message queue
 *
 * Dynamically sized message queue.
 */
struct ethosu_core_queue {
	struct ethosu_core_queue_header header;
	uint8_t                         data[];
};

/**
 * enum ethosu_core_status - Status
 */
enum ethosu_core_status {
	ETHOSU_CORE_STATUS_OK,
	ETHOSU_CORE_STATUS_ERROR,
	ETHOSU_CORE_STATUS_RUNNING,
	ETHOSU_CORE_STATUS_REJECTED,
	ETHOSU_CORE_STATUS_ABORTED,
	ETHOSU_CORE_STATUS_ABORTING,
};

/**
 * struct ethosu_core_buffer - Buffer descriptor
 *
 * Pointer and size to a buffer within the Ethos-U
 * address space.
 */
struct ethosu_core_buffer {
	uint32_t ptr;
	uint32_t size;
};

/**
 * enum ethosu_core_network_type - Network buffer type
 */
enum ethosu_core_network_type {
	ETHOSU_CORE_NETWORK_BUFFER = 1,
	ETHOSU_CORE_NETWORK_INDEX
};

/**
 * struct ethosu_core_network_buffer - Network buffer
 */
struct ethosu_core_network_buffer {
	u32 type;
	union {
		struct ethosu_core_buffer buffer;
		u32                       index;
	};
};

/**
 * struct ethosu_core_inference_req - Inference request
 */
struct ethosu_core_inference_req {
	uint64_t                  user_arg;
	uint32_t                  ifm_count;
	struct ethosu_core_buffer ifm[ETHOSU_CORE_BUFFER_MAX];
	uint32_t                  ofm_count;
	struct ethosu_core_buffer ofm[ETHOSU_CORE_BUFFER_MAX];
	struct ethosu_core_network_buffer network;
	uint8_t                   pmu_event_config[ETHOSU_CORE_PMU_MAX];
	uint32_t                  pmu_cycle_counter_enable;
	uint32_t                  inference_type;
};

/**
 * struct ethosu_core_inference_rsp - Inference response
 */
struct ethosu_core_inference_rsp {
	uint64_t user_arg;
	uint32_t ofm_count;
	uint32_t ofm_size[ETHOSU_CORE_BUFFER_MAX];
	uint32_t status;
	uint8_t  pmu_event_config[ETHOSU_CORE_PMU_MAX];
	uint32_t pmu_event_count[ETHOSU_CORE_PMU_MAX];
	uint32_t pmu_cycle_counter_enable;
	uint64_t pmu_cycle_counter_count;
};

/**
 * struct ethosu_core_network_info_req - Network information request
 */
struct ethosu_core_network_info_req {
	u64                               user_arg;
	struct ethosu_core_network_buffer network;
};

/**
 * struct ethosu_core_network_info_rsp - Network information response
 */
struct ethosu_core_network_info_rsp {
	u64      user_arg;
	char     desc[32];
	u32      is_vela;
	u32      ifm_count;
	u32      ifm_size[ETHOSU_CORE_BUFFER_MAX];
	u32      ifm_types[ETHOSU_CORE_BUFFER_MAX];
	u32      ifm_offset[ETHOSU_CORE_BUFFER_MAX];
	u32      ifm_dims[ETHOSU_CORE_BUFFER_MAX];
	u32      ifm_shapes[ETHOSU_CORE_BUFFER_MAX][ETHOSU_CORE_DIM_MAX];
	u32      ofm_count;
	u32      ofm_size[ETHOSU_CORE_BUFFER_MAX];
	u32      ofm_types[ETHOSU_CORE_BUFFER_MAX];
	u32      ofm_offset[ETHOSU_CORE_BUFFER_MAX];
	u32      ofm_dims[ETHOSU_CORE_BUFFER_MAX];
	u32      ofm_shapes[ETHOSU_CORE_BUFFER_MAX][ETHOSU_CORE_DIM_MAX];
	u32      status;
};

/**
 * struct ethosu_core_msg_version - Message protocol version
 */
struct ethosu_core_msg_version {
	uint8_t major;
	uint8_t minor;
	uint8_t patch;
	uint8_t _reserved;
};

/**
 * struct ethosu_core_capabilities_req - Message capabilities request
 */
struct ethosu_core_capabilities_req {
	uint64_t user_arg;
};

/**
 * struct ethosu_core_capabilities_rsp - Message capabilities response
 */
struct ethosu_core_msg_capabilities_rsp {
	uint64_t user_arg;
	uint32_t version_status;
	uint32_t version_minor;
	uint32_t version_major;
	uint32_t product_major;
	uint32_t arch_patch_rev;
	uint32_t arch_minor_rev;
	uint32_t arch_major_rev;
	uint32_t driver_patch_rev;
	uint32_t driver_minor_rev;
	uint32_t driver_major_rev;
	uint32_t macs_per_cc;
	uint32_t cmd_stream_version;
	uint32_t custom_dma;
};

/**
 * struct ethosu_core_cancel_inference_req - Message cancel inference request
 */
struct ethosu_core_cancel_inference_req {
	u64 user_arg;
	u64 inference_handle;
};

/**
 * struct ethosu_core_cancel_inference_rsp - Message cancel inference response
 */
struct ethosu_core_cancel_inference_rsp {
	u64 user_arg;
	u32 status;
};

enum ethosu_core_power_req_type {
	ETHOSU_CORE_POWER_REQ_SUSPEND = 0,
	ETHOSU_CORE_POWER_REQ_RESUME
};

/**
 * struct ethosu_core_power_req - Message power request
 */
struct ethosu_core_power_req {
	enum ethosu_core_power_req_type type;
};

/**
 * enum ethosu_core_msg_err_type - Error types
 */
enum ethosu_core_msg_err_type {
	ETHOSU_CORE_MSG_ERR_GENERIC = 0,
	ETHOSU_CORE_MSG_ERR_UNSUPPORTED_TYPE,
	ETHOSU_CORE_MSG_ERR_INVALID_PAYLOAD,
	ETHOSU_CORE_MSG_ERR_INVALID_SIZE,
	ETHOSU_CORE_MSG_ERR_INVALID_MAGIC,
	ETHOSU_CORE_MSG_ERR_MAX
};

/**
 * struct ethosu_core_msg_err - Error message struct
 */
struct ethosu_core_msg_err {
	uint32_t type;     /* optional use of extra error code */
	char     msg[128];
};
#ifdef __cplusplus
} /*namespace EthosU */
#endif

#endif
