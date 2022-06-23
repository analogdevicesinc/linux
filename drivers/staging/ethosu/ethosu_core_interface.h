/*
 * (C) COPYRIGHT 2020 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 * SPDX-License-Identifier: GPL-2.0-only
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
#define ETHOSU_CORE_BUFFER_MAX 16

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
	ETHOSU_CORE_MSG_POWER_REQ,
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

enum ethosu_core_status {
	ETHOSU_CORE_STATUS_OK,
	ETHOSU_CORE_STATUS_ERROR
};

struct ethosu_core_buffer {
	uint32_t ptr;
	uint32_t size;
};

struct ethosu_core_inference_req {
	uint64_t                  user_arg;
	uint32_t                  ifm_count;
	struct ethosu_core_buffer ifm[ETHOSU_CORE_BUFFER_MAX];
	uint32_t                  ofm_count;
	struct ethosu_core_buffer ofm[ETHOSU_CORE_BUFFER_MAX];
	struct ethosu_core_buffer network;
	uint8_t                   pmu_event_config[ETHOSU_CORE_PMU_MAX];
	uint32_t                  pmu_cycle_counter_enable;
	uint32_t                  inference_type;
};

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
 * struct ethosu_core_msg_verson - Message protocol version
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
